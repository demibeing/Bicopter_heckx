/*
 ============================================================================
 BICOPTER OS - FLIGHT READY (WIFI TELEMETRY & TUNING)
 Sensor: MPU6050_light 
 Receiver: 5-Wire PWM (Simplified Interrupt Logic)
 Features: Live WebSocket Telemetry, Remote Tuning, EEPROM Save
 ============================================================================
*/

#include <Wire.h>
#include <MPU6050_light.h> 
#include <ESP32Servo.h> 
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// --- Network Setup ---
const char* ssid = "Bicopter_OS";
const char* password = "12345678";
WebSocketsServer webSocket = WebSocketsServer(81);

// --- Hardware Setup ---
#define RC_ROLL_PIN  35
#define RC_PITCH_PIN 34
#define RC_THR_PIN   25
#define RC_AUX_PIN   36
#define RC_YAW_PIN   2

const int ESC_FRONT_PIN = 16;
const int ESC_BACK_PIN  = 17;
const int SERVO_FRONT_PIN = 26;
const int SERVO_BACK_PIN  = 27;

const int SERVO_FRONT_CH = 2;
const int SERVO_BACK_CH  = 3;

// --- Tuning & Trims ---
int FRONT_MID = 310; 
int BACK_MID  = 310;
const int SERVO_LIMIT = 60; 

// --- ESC Translated Limits (Microseconds) ---
const int ESC_MIN_US  = 732;   
const int ESC_IDLE_US = 1099;  
const int ESC_MAX_US  = 2002;  
const float TICK_TO_US = 4.8828f;

// -------- PID VALUES --------
float AKP = 1.930, RKP = 0.130, RKI = 0.030, RKD = 0.000, RFF = 50.0;
float PAKP = 2.500, PRKP = 0.150, PRKI = 0.010, PRKD = 0.020;
float YKP = 0.250, YKI = 0.010, YKD = 0.000;

const float I_TERM_LIMIT = 250.0;
const float MAX_RATE = 400.0;

// --- State & Sensors ---
MPU6050 mpu(Wire); 
Servo escFront;
Servo escBack;

float roll_angle = 0, pitch_angle = 0;
float roll_i = 0, pitch_i = 0, yaw_i = 0;
float target_roll = 0, target_pitch = 0;

// Filter Variables for D-Term
float prev_gr = 0, prev_gp = 0, prev_gz = 0;
float dr_filtered = 0, dp_filtered = 0, dz_filtered = 0;
const float D_CUTOFF_HZ = 30.0; 

unsigned long last_time = 0;
unsigned long last_telem_time = 0; 
unsigned long loop_timer = 0;
const unsigned long LOOP_TIME_US = 4000; // 250Hz control loop

volatile unsigned long r_start=0, p_start=0, t_start=0, a_start=0, y_start=0;
volatile unsigned long rc_roll_val=1500, rc_pitch_val=1500, rc_thr_val=1000, rc_aux_val=1000, rc_yaw_val=1500;

// EEPROM Magic Number
const uint32_t EEPROM_MAGIC = 0xB1C0DE52; 

// --- Helper ---
float applyDeadband(float value, float deadband) {
  if(abs(value) < deadband) return 0;
  return (value > 0) ? value - deadband : value + deadband;
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
    EEPROM.get(addr, FRONT_MID); addr += sizeof(FRONT_MID);
    EEPROM.get(addr, BACK_MID); addr += sizeof(BACK_MID);
  }
}

// --- RC Interrupts ---
void IRAM_ATTR readRoll()  { if (digitalRead(RC_ROLL_PIN)) r_start = micros(); else rc_roll_val = micros() - r_start; }
void IRAM_ATTR readPitch() { if (digitalRead(RC_PITCH_PIN)) p_start = micros(); else rc_pitch_val = micros() - p_start; }
void IRAM_ATTR readThr()   { if (digitalRead(RC_THR_PIN)) t_start = micros(); else rc_thr_val = micros() - t_start; }
void IRAM_ATTR readAux()   { if (digitalRead(RC_AUX_PIN)) a_start = micros(); else rc_aux_val = micros() - a_start; }
void IRAM_ATTR readYaw()   { if (digitalRead(RC_YAW_PIN)) y_start = micros(); else rc_yaw_val = micros() - y_start; }

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
        // Smart Conversion for ledc resolution if UI sends microseconds
        FRONT_MID = (raw_f > 800) ? (raw_f * 4096 / 20000) : raw_f;
        BACK_MID = (raw_b > 800) ? (raw_b * 4096 / 20000) : raw_b;
      }
      else if (cmd == "cal") { mpu.calcOffsets(); } 
      else if (cmd == "save") { saveToEEPROM(); }
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  EEPROM.begin(512); 
  loadFromEEPROM();

  WiFi.softAP(ssid, password);
  webSocket.begin(); 
  webSocket.onEvent(webSocketEvent);

  byte status = mpu.begin();
  if (status != 0) { while (1) { delay(10); } }

  pinMode(RC_ROLL_PIN, INPUT); pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_THR_PIN, INPUT);  pinMode(RC_AUX_PIN, INPUT); pinMode(RC_YAW_PIN, INPUT);

  attachInterrupt(RC_ROLL_PIN, readRoll, CHANGE);
  attachInterrupt(RC_PITCH_PIN, readPitch, CHANGE);
  attachInterrupt(RC_THR_PIN, readThr, CHANGE);
  attachInterrupt(RC_AUX_PIN, readAux, CHANGE);
  attachInterrupt(RC_YAW_PIN, readYaw, CHANGE);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  escFront.setPeriodHertz(50); escBack.setPeriodHertz(50);
  escFront.attach(ESC_FRONT_PIN, 500, 2500); escBack.attach(ESC_BACK_PIN, 500, 2500);

  ledcSetup(SERVO_FRONT_CH, 50, 12); ledcAttachPin(SERVO_FRONT_PIN, SERVO_FRONT_CH);
  ledcSetup(SERVO_BACK_CH, 50, 12);  ledcAttachPin(SERVO_BACK_PIN, SERVO_BACK_CH);

  ledcWrite(SERVO_FRONT_CH, FRONT_MID); ledcWrite(SERVO_BACK_CH, BACK_MID);
  
  delay(1000);
  escFront.writeMicroseconds(ESC_MIN_US); escBack.writeMicroseconds(ESC_MIN_US);
  delay(3000);

  mpu.calcOffsets(); 

  last_time = micros(); loop_timer = micros();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  webSocket.loop(); // Keep WebSocket alive

  unsigned long now = micros();
  unsigned long elapsed = now - loop_timer;
  if (elapsed < LOOP_TIME_US) {
    if (LOOP_TIME_US - elapsed > 1000) delay(1); 
    return; 
  }
  loop_timer = now;
  
  mpu.update(); 
  float dt = (now-last_time)/1000000.0;
  if(dt<=0 || dt>0.05) dt=0.01;
  last_time = now;

  long p_r = rc_roll_val, p_p = rc_pitch_val, p_t = rc_thr_val, p_y = rc_yaw_val;

  float gr = mpu.getGyroY(), gp = mpu.getGyroX(), gz = mpu.getGyroZ();
  float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();

  roll_angle = 0.98*(roll_angle + gr*dt) + 0.02*(atan2(ax, az)*57.3);
  pitch_angle = 0.98*(pitch_angle + gp*dt) + 0.02*(atan2(ay, az)*57.3);

  // --- D-Term Filtering (From Wi-Fi Code) ---
  float dr_raw = (gr - prev_gr) / dt;
  float dp_raw = (gp - prev_gp) / dt;
  float dz_raw = (gz - prev_gz) / dt;
  prev_gr = gr; prev_gp = gp; prev_gz = gz;

  float rc_filter = 1.0f / (2.0f * PI * D_CUTOFF_HZ);
  float alpha = dt / (rc_filter + dt);
  
  dr_filtered = dr_filtered + alpha * (dr_raw - dr_filtered);
  dp_filtered = dp_filtered + alpha * (dp_raw - dp_filtered);
  dz_filtered = dz_filtered + alpha * (dz_raw - dz_filtered);

  float r_stick = applyDeadband((p_r-1500.0)/500.0,0.05);
  float p_stick = applyDeadband((p_p-1500.0)/500.0,0.05);
  float y_stick = applyDeadband((p_y-1500.0)/500.0,0.05);

  target_roll = r_stick*30.0;
  target_pitch = p_stick*30.0;

  float r_des = constrain(AKP*(target_roll-roll_angle),-MAX_RATE,MAX_RATE);
  float p_des = constrain(PAKP*(target_pitch-pitch_angle),-MAX_RATE,MAX_RATE);
  float y_des = y_stick*400.0;

  float r_err = r_des-gr, p_err = p_des-gp, y_err = y_des-gz;

  if (p_t > 1050) {
    roll_i = constrain(roll_i + r_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    pitch_i = constrain(pitch_i + p_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    yaw_i = constrain(yaw_i + y_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
  } else { roll_i = pitch_i = yaw_i = 0; }

  // PID calculations utilizing the Filtered D-Term
  float r_out = (RKP*r_err) + (RKI*roll_i) - (RKD*dr_filtered) + (r_stick*RFF);
  float y_out = (YKP*y_err) + (YKI*yaw_i) - (YKD*dz_filtered);
  float p_out = (PRKP*p_err) + (PRKI*pitch_i) - (PRKD*dp_filtered);

  // Write to Servos (Opposing tilt signs)
  int front_servo_cmd = constrain(FRONT_MID - (int)r_out + (int)y_out, FRONT_MID - SERVO_LIMIT, FRONT_MID + SERVO_LIMIT);
  int back_servo_cmd  = constrain(BACK_MID  + (int)r_out + (int)y_out, BACK_MID  - SERVO_LIMIT, BACK_MID  + SERVO_LIMIT);

  ledcWrite(SERVO_FRONT_CH, front_servo_cmd);
  ledcWrite(SERVO_BACK_CH, back_servo_cmd);

  // Write to ESCs
  if (p_t < 1050) {
    escFront.writeMicroseconds(ESC_MIN_US);
    escBack.writeMicroseconds(ESC_MIN_US);
  } else {
    int base_throttle_us = map(p_t, 1000, 2000, ESC_IDLE_US, ESC_MAX_US);
    int p_out_us = (int)(p_out * TICK_TO_US);

    escFront.writeMicroseconds(constrain(base_throttle_us - p_out_us, ESC_MIN_US, ESC_MAX_US));
    escBack.writeMicroseconds(constrain(base_throttle_us + p_out_us, ESC_MIN_US, ESC_MAX_US));
  }

  // --- TELEMETRY STREAM (JSON over WebSocket) ---
  if (millis() - last_telem_time > 20) {
    last_telem_time = millis();
    StaticJsonDocument<256> t_doc;
    t_doc["type"] = "telem";
    t_doc["r"] = roll_angle; t_doc["p"] = pitch_angle;
    t_doc["t_r"] = target_roll; t_doc["t_p"] = target_pitch; 
    JsonArray rc = t_doc.createNestedArray("rc");
    rc.add(p_r); rc.add(p_p); rc.add(p_t); rc.add(p_y); 
    String jsonString; serializeJson(t_doc, jsonString);
    webSocket.broadcastTXT(jsonString);
  }
}
