/*
 ============================================================================
 BICOPTER OS - WEBSOCKET MASTER FIRMWARE (PRO BLACKBOX EDITION)
 Features: Lean Flight Logic, 50Hz Telemetry, Setpoint Tracking, Live Trim
 ============================================================================
 latest firmware that has the wifi tuning capability with real time pid graph and telemetry , tuning etc compatible with the index.html file
 dated 26.03.2026
*/

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

const char* ssid = "Bicopter_OS";
const char* password = "12345678";
WebSocketsServer webSocket = WebSocketsServer(81);

// --- Hardware Setup ---
#define RC_ROLL_PIN  34
#define RC_PITCH_PIN 35
#define RC_THR_PIN   25 
#define RC_AUX_PIN   36 
#define RC_YAW_PIN   2  

const int SERVO_FRONT_CH = 0, SERVO_BACK_CH = 1;
const int ESC_FRONT_CH = 14, ESC_BACK_CH = 15;
int FRONT_MID = 300, BACK_MID = 300; // Will be overwritten by EEPROM if saved
const int SERVO_LIMIT = 85;
const int ESC_MIN = 150, ESC_IDLE = 225, ESC_MAX = 410;

// --- Flight Variables ---
float AKP = 1.930, RKP = 0.130, RKI = 0.030, RKD = 0.000, RFF = 50.0;
float PAKP = 2.500, PRKP = 0.150, PRKI = 0.010, PRKD = 0.020; 
float YKP = 0.250, YKI = 0.010, YKD = 0.000; 

const float I_TERM_LIMIT = 250.0, MAX_RATE = 400.0; 

// --- Sensors & State ---
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
float roll_angle = 0, pitch_angle = 0, roll_i = 0, pitch_i = 0, yaw_i = 0;
float target_roll = 0, target_pitch = 0; 
float gr_bias = 0, gp_bias = 0, gz_bias = 0;
unsigned long last_time, last_telem_time = 0;

volatile unsigned long rc_roll_val=1500, rc_pitch_val=1500, rc_thr_val=1000, rc_aux_val=1000, rc_yaw_val=1500;
volatile unsigned long r_start, p_start, t_start, a_start, y_start, last_rc_time = 0;

// NEW MAGIC NUMBER: Forces EEPROM to rebuild with the new Servo Trims
const uint32_t EEPROM_MAGIC = 0xB1C0DE51; 

// --- Logic Helpers ---
float applyDeadband(float value, float deadband) {
  if(abs(value) < deadband) return 0.0;
  return (value > 0) ? value - deadband : value + deadband;
}

void calibrateSensors() {
  gr_bias = gp_bias = gz_bias = 0;
  for(int i=0; i<200; i++) {
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    gr_bias += g.gyro.y; gp_bias += g.gyro.x; gz_bias += g.gyro.z; delay(2);
  }
  gr_bias /= 200.0; gp_bias /= 200.0; gz_bias /= 200.0;
}

// --- EEPROM Management ---
void saveToEEPROM() {
  int addr = 0;
  EEPROM.put(addr, EEPROM_MAGIC); addr += sizeof(EEPROM_MAGIC);
  EEPROM.put(addr, AKP); addr += sizeof(AKP); EEPROM.put(addr, RKP); addr += sizeof(RKP);
  EEPROM.put(addr, RKI); addr += sizeof(RKI); EEPROM.put(addr, RKD); addr += sizeof(RKD);
  EEPROM.put(addr, PAKP); addr += sizeof(PAKP); EEPROM.put(addr, PRKP); addr += sizeof(PRKP);
  EEPROM.put(addr, PRKI); addr += sizeof(PRKI); EEPROM.put(addr, PRKD); addr += sizeof(PRKD);
  EEPROM.put(addr, YKP); addr += sizeof(YKP); EEPROM.put(addr, YKI); addr += sizeof(YKI);
  EEPROM.put(addr, YKD); addr += sizeof(YKD);
  // Save Trims
  EEPROM.put(addr, FRONT_MID); addr += sizeof(FRONT_MID);
  EEPROM.put(addr, BACK_MID); addr += sizeof(BACK_MID);
  EEPROM.commit();
}

void loadFromEEPROM() {
  uint32_t magic; int addr = 0;
  EEPROM.get(addr, magic); addr += sizeof(magic);
  if(magic == EEPROM_MAGIC) {
    EEPROM.get(addr, AKP); addr += sizeof(AKP); EEPROM.get(addr, RKP); addr += sizeof(RKP);
    EEPROM.get(addr, RKI); addr += sizeof(RKI); EEPROM.get(addr, RKD); addr += sizeof(RKD);
    EEPROM.get(addr, PAKP); addr += sizeof(PAKP); EEPROM.get(addr, PRKP); addr += sizeof(PRKP);
    EEPROM.get(addr, PRKI); addr += sizeof(PRKI); EEPROM.get(addr, PRKD); addr += sizeof(PRKD);
    EEPROM.get(addr, YKP); addr += sizeof(YKP); EEPROM.get(addr, YKI); addr += sizeof(YKI);
    EEPROM.get(addr, YKD); addr += sizeof(YKD);
    // Load Trims
    EEPROM.get(addr, FRONT_MID); addr += sizeof(FRONT_MID);
    EEPROM.get(addr, BACK_MID); addr += sizeof(BACK_MID);
  }
}

// --- RC Interrupts ---
void IRAM_ATTR readRoll()  { if(digitalRead(RC_ROLL_PIN)) r_start = micros(); else { rc_roll_val = micros()-r_start; last_rc_time = micros(); } }
void IRAM_ATTR readPitch() { if(digitalRead(RC_PITCH_PIN)) p_start = micros(); else { rc_pitch_val = micros()-p_start; last_rc_time = micros(); } }
void IRAM_ATTR readThr()   { if(digitalRead(RC_THR_PIN)) t_start = micros(); else { rc_thr_val = micros()-t_start; last_rc_time = micros(); } }
void IRAM_ATTR readAux()   { if(digitalRead(RC_AUX_PIN)) a_start = micros(); else { rc_aux_val = micros()-a_start; last_rc_time = micros(); } }
void IRAM_ATTR readYaw()   { if(digitalRead(RC_YAW_PIN)) y_start = micros(); else { rc_yaw_val = micros()-y_start; last_rc_time = micros(); } }

// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      String cmd = doc["cmd"];
      if (cmd == "set_pid") {
        AKP = doc["akp"]; RKP = doc["rkp"]; RKI = doc["rki"]; RKD = doc["rkd"];
        PAKP = doc["pakp"]; PRKP = doc["prkp"]; PRKI = doc["prki"]; PRKD = doc["prkd"];
        YKP = doc["ykp"]; YKI = doc["yki"]; YKD = doc["ykd"];
      } 
      else if (cmd == "set_trim") {
        int raw_f = doc["f_mid"];
        int raw_b = doc["b_mid"];
        // Smart Conversion: If UI sends microseconds (e.g., 1500), map it to PCA9685 ticks.
        // If UI sends PCA ticks directly (e.g., 300), keep it raw.
        FRONT_MID = (raw_f > 800) ? (raw_f * 4096 / 20000) : raw_f;
        BACK_MID = (raw_b > 800) ? (raw_b * 4096 / 20000) : raw_b;
      }
      else if (cmd == "cal") { calibrateSensors(); } 
      else if (cmd == "save") { saveToEEPROM(); }
    }
  }
}

void setup() {
  Wire.begin(); mpu.begin();
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  pwm.begin(); pwm.setPWMFreq(50);
  EEPROM.begin(512); loadFromEEPROM();
  WiFi.softAP(ssid, password);
  webSocket.begin(); webSocket.onEvent(webSocketEvent);

  pinMode(RC_ROLL_PIN, INPUT); pinMode(RC_PITCH_PIN, INPUT); pinMode(RC_THR_PIN, INPUT); 
  pinMode(RC_AUX_PIN, INPUT); pinMode(RC_YAW_PIN, INPUT);
  attachInterrupt(RC_ROLL_PIN, readRoll, CHANGE); attachInterrupt(RC_PITCH_PIN, readPitch, CHANGE);
  attachInterrupt(RC_THR_PIN, readThr, CHANGE); attachInterrupt(RC_AUX_PIN, readAux, CHANGE);
  attachInterrupt(RC_YAW_PIN, readYaw, CHANGE);

  pwm.setPWM(ESC_FRONT_CH, 0, ESC_MIN); pwm.setPWM(ESC_BACK_CH, 0, ESC_MIN);
  calibrateSensors(); last_time = micros();
}

void loop() {
  webSocket.loop();
  
  sensors_event_t a, g, t_s; mpu.getEvent(&a, &g, &t_s);
  unsigned long now = micros(); float dt = (now - last_time) / 1000000.0f; 
  if (dt <= 0 || dt > 0.05) dt = 0.01; last_time = now;

  float gr = (g.gyro.y - gr_bias) * 57.3; 
  float gp = (g.gyro.x - gp_bias) * 57.3; 
  float gz = (g.gyro.z - gz_bias) * 57.3;
  roll_angle = 0.98*(roll_angle + gr*dt) + 0.02*(atan2(a.acceleration.x, a.acceleration.z)*57.3);
  pitch_angle = 0.98*(pitch_angle + gp*dt) + 0.02*(atan2(a.acceleration.y, a.acceleration.z)*57.3);

  noInterrupts(); 
  long p_r=rc_roll_val, p_p=rc_pitch_val, p_t=rc_thr_val, p_a=rc_aux_val, p_y=rc_yaw_val; 
  unsigned long sig=micros()-last_rc_time; 
  interrupts();

  if (sig > 100000 || p_t < 800 || p_a < 1500) { p_r=p_p=p_y=1500; p_t=1000; }

  float r_stick = applyDeadband((p_r-1500.0)/500.0, 0.05);
  float p_stick = applyDeadband((p_p-1500.0)/500.0, 0.05);
  float y_stick = applyDeadband((p_y-1500.0)/500.0, 0.05);

  target_roll = r_stick * 30.0;
  target_pitch = p_stick * 30.0;

  float r_des = constrain(AKP * (target_roll - roll_angle), -MAX_RATE, MAX_RATE);
  float p_des = constrain(PAKP * (target_pitch - pitch_angle), -MAX_RATE, MAX_RATE);
  float y_des = y_stick * 400.0;
  
  float r_err = r_des - gr; float p_err = p_des - gp; float y_err = y_des - gz;

  if(p_t > 1050) {
    roll_i = constrain(roll_i + r_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    pitch_i = constrain(pitch_i + p_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    yaw_i = constrain(yaw_i + y_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
  } else { roll_i = pitch_i = yaw_i = 0; }

  float r_out = (RKP*r_err) + (RKI*roll_i) + (RKD*-gr) + (r_stick*RFF);
  float p_out = (PRKP*p_err) + (PRKI*pitch_i) + (PRKD*-gp);
  float y_out = (YKP*y_err) + (YKI*yaw_i) + (YKD*-gz);

  if(p_t > 1050) {
    pwm.setPWM(SERVO_FRONT_CH, 0, constrain(FRONT_MID + (int)r_out + (int)y_out, FRONT_MID-SERVO_LIMIT, FRONT_MID+SERVO_LIMIT));
    pwm.setPWM(SERVO_BACK_CH, 0, constrain(BACK_MID - (int)r_out + (int)y_out, BACK_MID-SERVO_LIMIT, BACK_MID+SERVO_LIMIT));
    int b = map(p_t, 1050, 2000, ESC_IDLE, ESC_MAX);
    pwm.setPWM(ESC_FRONT_CH, 0, constrain(b - (int)p_out, ESC_MIN, ESC_MAX));
    pwm.setPWM(ESC_BACK_CH, 0, constrain(b + (int)p_out, ESC_MIN, ESC_MAX));
  } else {
    pwm.setPWM(SERVO_FRONT_CH, 0, FRONT_MID); pwm.setPWM(SERVO_BACK_CH, 0, BACK_MID);
    pwm.setPWM(ESC_FRONT_CH, 0, ESC_MIN); pwm.setPWM(ESC_BACK_CH, 0, ESC_MIN);
  }

  // --- UPGRADED TELEMETRY STREAM (50Hz) ---
  if (millis() - last_telem_time > 20) {
    last_telem_time = millis();
    StaticJsonDocument<256> t_doc;
    t_doc["type"] = "telem";
    t_doc["r"] = roll_angle; t_doc["p"] = pitch_angle;
    t_doc["t_r"] = target_roll; t_doc["t_p"] = target_pitch; 
    JsonArray rc = t_doc.createNestedArray("rc");
    rc.add(p_r); rc.add(p_p); rc.add(p_t); rc.add(p_y); rc.add(p_a);
    String jsonString; serializeJson(t_doc, jsonString);
    webSocket.broadcastTXT(jsonString);
  }
}
