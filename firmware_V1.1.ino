//latest code with direct esp32 pins and telemetry
//failsafe,proper priority loop
//dated: 03-04-2026


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

// --- Hardware Pins ---
#define RC_ROLL_PIN  35
#define RC_PITCH_PIN 34
#define RC_THR_PIN   25
#define RC_YAW_PIN   2
#define RC_AUX_PIN   39

const int ESC_FRONT_PIN = 16;
const int ESC_BACK_PIN  = 17;
const int SERVO_FRONT_PIN = 26;
const int SERVO_BACK_PIN  = 27;

// --- Limits & Trims (Strictly Microseconds) ---
int FRONT_MID_US = 1400; // Adjusted from your tick math
int BACK_MID_US  = 1500;
const int SERVO_LIMIT_US = 300; 

const int ESC_MIN_US  = 1000; 
const int ESC_IDLE_US = 1050;  
const int ESC_MAX_US  = 2000;  

// -------- PID VALUES --------
float AKP = 1.930, RKP = 0.130, RKI = 0.030, RKD = 0.000, RFF = 50.0;
float PAKP = 2.500, PRKP = 0.150, PRKI = 0.010, PRKD = 0.020;
float YKP = 0.250, YKI = 0.010, YKD = 0.000;

const float I_TERM_LIMIT = 250.0;
const float MAX_RATE = 400.0;

// --- System State ---
MPU6050 mpu(Wire); 
Servo escFront, escBack, servoFront, servoBack;

float roll_angle = 0, pitch_angle = 0;
float roll_i = 0, pitch_i = 0, yaw_i = 0;
float target_roll = 0, target_pitch = 0;

float prev_gr = 0, prev_gp = 0, prev_gz = 0;
float dr_filtered = 0, dp_filtered = 0, dz_filtered = 0;
const float D_CUTOFF_HZ = 30.0; 

unsigned long loop_timer = 0;
unsigned long last_telem_time = 0; 
const unsigned long LOOP_TIME_US = 4000; // 250Hz

// RC Interrupt Variables
volatile unsigned long r_start=0, p_start=0, t_start=0, a_start=0, y_start=0;
volatile long rc_roll_val=1500, rc_pitch_val=1500, rc_thr_val=1000, rc_aux_val=1000, rc_yaw_val=1500;
volatile unsigned long last_rc_time = 0; 

// Safe RC Variables for Loop
long p_r = 1500, p_p = 1500, p_t = 1000, p_y = 1500, p_a = 1000;
unsigned long signal_age = 0;

bool calibration_requested = false;
bool is_armed = false;

const uint32_t EEPROM_MAGIC = 0xB1C0DE54; // Updated to wipe old tick data

// --- Helpers ---
float applyDeadband(float value, float deadband) {
  if(abs(value) < deadband) return 0;
  return (value > 0) ? value - deadband : value + deadband;
}

void resetPIDState() {
  roll_i = pitch_i = yaw_i = 0;
  prev_gr = mpu.getGyroY(); prev_gp = mpu.getGyroX(); prev_gz = mpu.getGyroZ();
  dr_filtered = dp_filtered = dz_filtered = 0;
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
  EEPROM.put(addr, FRONT_MID_US); addr += sizeof(FRONT_MID_US);
  EEPROM.put(addr, BACK_MID_US); addr += sizeof(BACK_MID_US);
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
    EEPROM.get(addr, FRONT_MID_US); addr += sizeof(FRONT_MID_US);
    EEPROM.get(addr, BACK_MID_US); addr += sizeof(BACK_MID_US);
  }
}

// --- RC Interrupts ---
void IRAM_ATTR readRoll()  { if(digitalRead(RC_ROLL_PIN)) r_start = micros(); else { rc_roll_val = micros() - r_start; last_rc_time = micros(); } }
void IRAM_ATTR readPitch() { if(digitalRead(RC_PITCH_PIN)) p_start = micros(); else { rc_pitch_val = micros() - p_start; last_rc_time = micros(); } }
void IRAM_ATTR readThr()   { if(digitalRead(RC_THR_PIN)) t_start = micros(); else { rc_thr_val = micros() - t_start; last_rc_time = micros(); } }
void IRAM_ATTR readAux()   { if(digitalRead(RC_AUX_PIN)) a_start = micros(); else { rc_aux_val = micros() - a_start; last_rc_time = micros(); } }
void IRAM_ATTR readYaw()   { if(digitalRead(RC_YAW_PIN)) y_start = micros(); else { rc_yaw_val = micros() - y_start; last_rc_time = micros(); } }

// --- WebSocket Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<256> doc;
    if (!deserializeJson(doc, payload)) {
      String cmd = doc["cmd"];
      if (cmd == "set_pid") {
        if(doc.containsKey("akp")) AKP = doc["akp"]; if(doc.containsKey("rkp")) RKP = doc["rkp"];
        if(doc.containsKey("rki")) RKI = doc["rki"]; if(doc.containsKey("rkd")) RKD = doc["rkd"];
        if(doc.containsKey("pakp")) PAKP = doc["pakp"]; if(doc.containsKey("prkp")) PRKP = doc["prkp"];
        if(doc.containsKey("prki")) PRKI = doc["prki"]; if(doc.containsKey("prkd")) PRKD = doc["prkd"];
        if(doc.containsKey("ykp")) YKP = doc["ykp"]; if(doc.containsKey("yki")) YKI = doc["yki"];
        if(doc.containsKey("ykd")) YKD = doc["ykd"];
      } 
      else if (cmd == "set_trim") {
        if(doc.containsKey("f_mid")) FRONT_MID_US = doc["f_mid"];
        if(doc.containsKey("b_mid")) BACK_MID_US = doc["b_mid"];
      }
      else if (cmd == "cal") { calibration_requested = true; } 
      else if (cmd == "save") { saveToEEPROM(); }
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000);

  EEPROM.begin(512); 
  loadFromEEPROM();

  WiFi.softAP(ssid, password);
  webSocket.begin(); 
  webSocket.onEvent(webSocketEvent);

  if (mpu.begin() != 0) {
    while (1) { Serial.println("MPU Fail"); delay(500); }
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(RC_ROLL_PIN, INPUT); pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_THR_PIN, INPUT);  pinMode(RC_AUX_PIN, INPUT); pinMode(RC_YAW_PIN, INPUT);

  attachInterrupt(RC_ROLL_PIN, readRoll, CHANGE);
  attachInterrupt(RC_PITCH_PIN, readPitch, CHANGE);
  attachInterrupt(RC_THR_PIN, readThr, CHANGE);
  attachInterrupt(RC_AUX_PIN, readAux, CHANGE);
  attachInterrupt(RC_YAW_PIN, readYaw, CHANGE);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  
  escFront.setPeriodHertz(50); escBack.setPeriodHertz(50);
  servoFront.setPeriodHertz(50); servoBack.setPeriodHertz(50);
  
  escFront.attach(ESC_FRONT_PIN, 1000, 2000); escBack.attach(ESC_BACK_PIN, 1000, 2000);
  servoFront.attach(SERVO_FRONT_PIN, 500, 2500); servoBack.attach(SERVO_BACK_PIN, 500, 2500);

  escFront.writeMicroseconds(ESC_MIN_US); escBack.writeMicroseconds(ESC_MIN_US);
  servoFront.writeMicroseconds(FRONT_MID_US); servoBack.writeMicroseconds(BACK_MID_US);
  delay(2000);

  Serial.println("Calibrating MPU...");
  mpu.calcOffsets(true, true); 
  resetPIDState();

  loop_timer = micros();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // 1. Handle Background Tasks (WebSockets)
  webSocket.loop(); 

  // 2. Handle Runtime Calibration Request
  if (calibration_requested) {
    escFront.writeMicroseconds(ESC_MIN_US); escBack.writeMicroseconds(ESC_MIN_US);
    servoFront.writeMicroseconds(FRONT_MID_US); servoBack.writeMicroseconds(BACK_MID_US);
    mpu.calcOffsets(true, true);
    resetPIDState();
    calibration_requested = false;
    loop_timer = micros(); // Prevent dt spike
  }

  // 3. Strict Timing Control
  unsigned long now = micros();
  if (now - loop_timer < LOOP_TIME_US) return; 
  
  float dt = (now - loop_timer) / 1000000.0f;
  if(dt <= 0 || dt > 0.05) dt = 0.01; // Sanity check
  loop_timer = now;

  // 4. Safe RC Polling & Failsafe Check
  noInterrupts();
  p_r = rc_roll_val; p_p = rc_pitch_val; p_t = rc_thr_val; p_y = rc_yaw_val; p_a = rc_aux_val;
  signal_age = micros() - last_rc_time;
  interrupts();

  // Failsafe: Signal lost OR throttle wildly out of bounds
  if (signal_age > 100000 || p_t < 800 || p_t > 2200) { 
    p_r = 1500; p_p = 1500; p_y = 1500; p_t = 1000; p_a = 1000; 
  }

  bool kill_switch_active = (p_a > 1500) || (signal_age > 100000);
  is_armed = (p_t > 1050 && !kill_switch_active);

  // 5. Sensor Update
  mpu.update(); 
  float gr = mpu.getGyroY(), gp = mpu.getGyroX(), gz = mpu.getGyroZ();
  float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();

  roll_angle = 0.98*(roll_angle + gr*dt) + 0.02*(atan2(ax, az)*57.3);
  pitch_angle = 0.98*(pitch_angle + gp*dt) + 0.02*(atan2(ay, az)*57.3);

  // D-Term Filtering
  float dr_raw = (gr - prev_gr) / dt;
  float dp_raw = (gp - prev_gp) / dt;
  float dz_raw = (gz - prev_gz) / dt;
  prev_gr = gr; prev_gp = gp; prev_gz = gz;

  float rc_filter = 1.0f / (2.0f * PI * D_CUTOFF_HZ);
  float alpha = dt / (rc_filter + dt);
  dr_filtered += alpha * (dr_raw - dr_filtered);
  dp_filtered += alpha * (dp_raw - dp_filtered);
  dz_filtered += alpha * (dz_raw - dz_filtered);

  // 6. PID Math
  float r_stick = applyDeadband((p_r-1500.0)/500.0, 0.05);
  float p_stick = applyDeadband((p_p-1500.0)/500.0, 0.05);
  float y_stick = applyDeadband((p_y-1500.0)/500.0, 0.05);

  target_roll = r_stick * 30.0;
  target_pitch = p_stick * 30.0;

  float r_des = constrain(AKP*(target_roll-roll_angle), -MAX_RATE, MAX_RATE);
  float p_des = constrain(PAKP*(target_pitch-pitch_angle), -MAX_RATE, MAX_RATE);
  float y_des = y_stick * 400.0;

  float r_err = r_des - gr; 
  float p_err = p_des - gp; 
  float y_err = y_des - gz;

  if (is_armed) {
    roll_i = constrain(roll_i + r_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    pitch_i = constrain(pitch_i + p_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    yaw_i = constrain(yaw_i + y_err*dt, -I_TERM_LIMIT, I_TERM_LIMIT);
  } else {
    resetPIDState(); // Prevents windup and jumping while sitting on the desk
  }

  float r_out = (RKP*r_err) + (RKI*roll_i) - (RKD*dr_filtered) + (r_stick*RFF);
  float y_out = (YKP*y_err) + (YKI*yaw_i) - (YKD*dz_filtered);
  float p_out = (PRKP*p_err) + (PRKI*pitch_i) - (PRKD*dp_filtered);

  // 7. Output Routing
  int f_cmd_us = constrain(FRONT_MID_US - (int)r_out + (int)y_out, FRONT_MID_US - SERVO_LIMIT_US, FRONT_MID_US + SERVO_LIMIT_US);
  int b_cmd_us = constrain(BACK_MID_US  + (int)r_out + (int)y_out, BACK_MID_US  - SERVO_LIMIT_US, BACK_MID_US  + SERVO_LIMIT_US);

  servoFront.writeMicroseconds(f_cmd_us);
  servoBack.writeMicroseconds(b_cmd_us);

  if (!is_armed) {
    escFront.writeMicroseconds(ESC_MIN_US);
    escBack.writeMicroseconds(ESC_MIN_US);
  } 
  else {
    int base_throttle_us = map(p_t, 1000, 2000, ESC_IDLE_US, ESC_MAX_US);
    escFront.writeMicroseconds(constrain(base_throttle_us - (int)p_out, ESC_MIN_US, ESC_MAX_US));
    escBack.writeMicroseconds(constrain(base_throttle_us + (int)p_out, ESC_MIN_US, ESC_MAX_US));
  }

  // 8. Telemetry Stream
  if (millis() - last_telem_time > 50) { // Reduced to 20Hz to prevent network lag
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
