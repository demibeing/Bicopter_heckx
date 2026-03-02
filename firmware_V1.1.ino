/*
 ============================================================================
  BICOPTER PRO FLIGHT CONTROLLER v3.2 (Feedforward + Aux Kill Switch)
  
  New Feature: 
  - Roll Feedforward (RFF): Directly links RC stick to Servo output.
  - Keeps stabilization target at 30° but allows full mechanical servo throw.
 ============================================================================
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// ============================================================================
// 1. HARDWARE PINS & CHANNELS
// ============================================================================
#define RC_ROLL_PIN  34
#define RC_PITCH_PIN 35
#define RC_THR_PIN   25 
#define RC_AUX_PIN   36 

#define SERVO_FREQ 50    
#define PCA9685_ADDR 0x40

const int SERVO_FRONT_CH = 0;
const int SERVO_BACK_CH  = 1;
const int ESC_FRONT_CH   = 14;
const int ESC_BACK_CH    = 15;

// ============================================================================
// 2. LIMITS & OFFSETS
// ============================================================================
const int FRONT_MID = 300; 
const int BACK_MID  = 300; 
const int SERVO_LIMIT = 85; 

const int ESC_MIN  = 150;   
const int ESC_IDLE = 225;   
const int ESC_MAX  = 410;   

const int CORRECTION_DIR_ROLL = 1; 

// ============================================================================
// 3. PID & FEEDFORWARD GAINS
// ============================================================================
float AKP = 1.93;
float RKP = 0.13;
float RKI = 0.03;
float RKD = 0.00;
float RFF = 50.0; // <--- NEW: Roll Feedforward. Increase for more "snap"

float PAKP = 3.0;
float PRKP = 0.15;
float PRKI = 0.05;
float PRKD = 0.05; 

const float I_TERM_LIMIT = 250.0;
const float MAX_DES_RATE = 400.0; 

// ============================================================================
// 4. GLOBAL STATE
// ============================================================================
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float roll_angle = 0, pitch_angle = 0, roll_i = 0, pitch_i = 0;
float gr_bias = 0, gp_bias = 0; 
unsigned long last_time;

volatile unsigned long rc_roll_val=1500, rc_pitch_val=1500, rc_thr_val=1000, rc_aux_val=1000;
volatile unsigned long r_start, p_start, t_start, a_start;
volatile unsigned long last_rc_time = 0; 

const int EEPROM_SIZE = 128;
const uint32_t EEPROM_MAGIC = 0xB1C0DE25; // Updated magic version

// ============================================================================
// 5. INTERRUPTS, SAVE/LOAD
// ============================================================================
void IRAM_ATTR readRoll()  { if(digitalRead(RC_ROLL_PIN)) r_start = micros(); else { rc_roll_val = micros()-r_start; last_rc_time = micros(); } }
void IRAM_ATTR readPitch() { if(digitalRead(RC_PITCH_PIN)) p_start = micros(); else rc_pitch_val = micros()-p_start; }
void IRAM_ATTR readThr()   { if(digitalRead(RC_THR_PIN)) t_start = micros(); else { rc_thr_val = micros()-t_start; last_rc_time = micros(); } }
void IRAM_ATTR readAux()   { if(digitalRead(RC_AUX_PIN)) a_start = micros(); else { rc_aux_val = micros()-a_start; last_rc_time = micros(); } }

void saveGains() {
  EEPROM.writeUInt(0, EEPROM_MAGIC);
  float g[] = {AKP, RKP, RKI, RKD, PAKP, PRKP, PRKI, PRKD, RFF};
  for(int i=0; i<9; i++) EEPROM.writeFloat(4 + (i*4), g[i]);
  EEPROM.commit();
  Serial.println("SAVED");
}

void loadGains() {
  if(EEPROM.readUInt(0) != EEPROM_MAGIC) return; 
  AKP = EEPROM.readFloat(4);  RKP = EEPROM.readFloat(8);
  RKI = EEPROM.readFloat(12); RKD = EEPROM.readFloat(16);
  PAKP = EEPROM.readFloat(20); PRKP = EEPROM.readFloat(24);
  PRKI = EEPROM.readFloat(28); PRKD = EEPROM.readFloat(32);
  RFF  = EEPROM.readFloat(36); // Load Feedforward
}

// ============================================================================
// 7. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!mpu.begin()) while(1);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  pwm.begin(); pwm.setPWMFreq(SERVO_FREQ);

  pwm.setPWM(ESC_FRONT_CH, 0, ESC_MIN);
  pwm.setPWM(ESC_BACK_CH, 0, ESC_MIN);
  delay(3000); 

  for(int i=0; i<200; i++){
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    gr_bias += g.gyro.y; gp_bias += g.gyro.x; delay(2);
  }
  gr_bias /= 200.0; gp_bias /= 200.0;

  pinMode(RC_ROLL_PIN, INPUT); pinMode(RC_PITCH_PIN, INPUT); 
  pinMode(RC_THR_PIN, INPUT);  pinMode(RC_AUX_PIN, INPUT);
  
  attachInterrupt(RC_ROLL_PIN, readRoll, CHANGE);
  attachInterrupt(RC_PITCH_PIN, readPitch, CHANGE);
  attachInterrupt(RC_THR_PIN, readThr, CHANGE);
  attachInterrupt(RC_AUX_PIN, readAux, CHANGE);

  EEPROM.begin(EEPROM_SIZE);
  loadGains();
  last_time = micros();
}

// ============================================================================
// 8. LOOP
// ============================================================================
void loop() {
  // --- SERIAL PARSER ---
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n'); line.trim();
    if (line.startsWith("AKP ")) AKP = line.substring(4).toFloat();
    else if (line.startsWith("RKP ")) RKP = line.substring(4).toFloat();
    else if (line.startsWith("RKI ")) RKI = line.substring(4).toFloat();
    else if (line.startsWith("RKD ")) RKD = line.substring(4).toFloat();
    else if (line.startsWith("RFF ")) RFF = line.substring(4).toFloat(); // Support tuning RFF
    else if (line.startsWith("PAKP ")) PAKP = line.substring(5).toFloat();
    else if (line.startsWith("PRKP ")) PRKP = line.substring(5).toFloat();
    else if (line.startsWith("PRKI ")) PRKI = line.substring(5).toFloat();
    else if (line.startsWith("PRKD ")) PRKD = line.substring(5).toFloat();
    else if (line == "SAVE") saveGains();
    else if (line == "REQ") Serial.printf("SYNC,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", AKP, RKP, RKI, RKD, PAKP, PRKP, PRKI, PRKD, RFF);
  }

  // --- SENSORS ---
  sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
  unsigned long now = micros();
  float dt = (now - last_time) / 1000000.0f; 
  if (dt <= 0 || dt > 0.05) dt = 0.01; 
  last_time = now;

  float gr = (g.gyro.y - gr_bias) * 57.2958;
  float gp = (g.gyro.x - gp_bias) * 57.2958;
  roll_angle = 0.99 * (roll_angle + gr * dt) + 0.01 * (atan2(a.acceleration.x, a.acceleration.z) * 57.2958);
  pitch_angle = 0.99 * (pitch_angle + gp * dt) + 0.01 * (atan2(a.acceleration.y, a.acceleration.z) * 57.2958);

  // --- RC & FAILSAFE ---
  noInterrupts(); 
  long p_roll = rc_roll_val; long p_pitch = rc_pitch_val;
  long p_thr = rc_thr_val;  long p_aux = rc_aux_val;
  unsigned long signal_age = micros() - last_rc_time;
  interrupts(); 

  bool kill_switch_active = (p_aux < 1500); 
  bool failsafe_active = (signal_age > 100000 || p_thr < 800 || p_thr > 2200 || p_roll < 800 || kill_switch_active);

  if (failsafe_active) { p_roll = 1500; p_pitch = 1500; p_thr = 1000; }

  // --- ROLL LOGIC (WITH FEEDFORWARD) ---
  float roll_stick_pos = (p_roll - 1500.0) / 500.0; // Normalized -1.0 to 1.0
  float target_r = roll_stick_pos * 30.0;          // Stabilization limit still 30 deg
  float rc_roll_ff = roll_stick_pos * RFF;         // Direct stick bypass

  float r_des_rate = constrain(AKP * (target_r - roll_angle), -MAX_DES_RATE, MAX_DES_RATE);
  float r_err = r_des_rate - gr;
  
  if (p_thr > 1050 && !failsafe_active) roll_i = constrain(roll_i + r_err * dt, -I_TERM_LIMIT, I_TERM_LIMIT);
  else roll_i = 0; 
  
  // Total output = PID + Feedforward
  float r_out = (RKP * r_err) + (RKI * roll_i) + (RKD * -gr) + rc_roll_ff; 
  int r_def = constrain((int)(r_out * CORRECTION_DIR_ROLL), -SERVO_LIMIT, SERVO_LIMIT);
  
  pwm.setPWM(SERVO_FRONT_CH, 0, FRONT_MID + r_def);
  pwm.setPWM(SERVO_BACK_CH, 0, BACK_MID - r_def);

  // --- PITCH LOGIC (MOTORS) ---
  float target_p = ((p_pitch - 1500.0) / 500.0) * 30.0;
  float p_des_rate = constrain(PAKP * (target_p - pitch_angle), -MAX_DES_RATE, MAX_DES_RATE);
  float p_err = p_des_rate - gp;
  
  if (p_thr > 1050 && !failsafe_active) pitch_i = constrain(pitch_i + p_err * dt, -I_TERM_LIMIT, I_TERM_LIMIT);
  else pitch_i = 0;
  
  float p_out = (PRKP * p_err) + (PRKI * pitch_i) + (PRKD * -gp);

  int esc_f = ESC_MIN, esc_b = ESC_MIN;
  if (p_thr > 1050 && !failsafe_active) {
    int base_throttle = map(p_thr, 1050, 2000, ESC_IDLE, ESC_MAX);
    esc_f = constrain(base_throttle - (int)p_out, ESC_MIN, ESC_MAX);
    esc_b = constrain(base_throttle + (int)p_out, ESC_MIN, ESC_MAX);
  }
  pwm.setPWM(ESC_FRONT_CH, 0, esc_f);
  pwm.setPWM(ESC_BACK_CH, 0, esc_b);

  // --- TELEMETRY ---
  static unsigned long tlm_t = 0;
  if(millis() - tlm_t > 100) { 
    tlm_t = millis();
    Serial.printf("TLM,%.1f,%.1f,%d,%d\n", roll_angle, pitch_angle, esc_f, esc_b);
  }
}
