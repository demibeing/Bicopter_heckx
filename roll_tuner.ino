/*
 bicopter_tuner.ino
 - Serial PID tuning + telemetry + EEPROM save/load
 - Protocol (send over serial as plain text lines):
    AKP <value>   -> set angle Kp
    RKP <value>   -> set rate Kp
    RKI <value>   -> set rate Ki
    RKD <value>   -> set rate Kd
    SAVE          -> save gains to EEPROM
    LOAD          -> load gains from EEPROM
    REQTLM        -> request one telemetry line immediately
 - Telemetry output (periodic):
    TLM,roll_angle,gyro_rate,target_roll,deflection
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define RC_ROLL_PIN 34
#define SERVO_FREQ 50
#define PCA9685_ADDR 0x40

const int SERVO_FRONT_CH = 0;
const int SERVO_BACK_CH  = 1;
const int FRONT_MID = 300;
const int BACK_MID  = 300;
const int SERVO_LIMIT = 85;
const int CORRECTION_DIRECTION = 1;

// PID gains (tunable)
float roll_angle_kp = 3.0;
float roll_rate_kp  = 0.8;
float roll_rate_ki  = 0.05;
float roll_rate_kd  = 0.12;

const float I_TERM_LIMIT = 250.0;
const float MAX_DESIRED_RATE = 400.0;

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float roll_angle = 0.0;
float gyro_roll_bias = 0.0;
const float alpha = 0.995;

volatile unsigned long rc_roll_start = 0;
volatile unsigned long rc_roll_val = 1500;
volatile unsigned long last_rc_time = 0;

float gyro_rate = 0.0;
float target_roll = 0.0;
float roll_i_term = 0.0;
unsigned long last_time;

unsigned long last_tlm_time = 0;
const unsigned long TLM_INTERVAL_MS = 50; // 20 Hz telemetry

// EEPROM layout (addresses)
const int EEPROM_SIZE = 64;
const int ADDR_MAGIC = 0;
const int ADDR_GAINS = 4; // store 4 floats after magic
const uint32_t EEPROM_MAGIC = 0xB10C1234;

void IRAM_ATTR readRoll() {
  unsigned long now = micros();
  if (digitalRead(RC_ROLL_PIN) == HIGH) {
    rc_roll_start = now;
  } else {
    rc_roll_val = now - rc_roll_start;
    last_rc_time = now;
  }
}

// Helper: write/read float to EEPROM
void eepromWriteFloat(int addr, float val) {
  byte *p = (byte*)(void*)&val;
  for (int i = 0; i < 4; ++i) {
    EEPROM.write(addr + i, p[i]);
  }
}
float eepromReadFloat(int addr) {
  float v = 0.0;
  byte *p = (byte*)(void*)&v;
  for (int i = 0; i < 4; ++i) {
    p[i] = EEPROM.read(addr + i);
  }
  return v;
}

void saveGainsToEEPROM() {
  EEPROM.writeInt(ADDR_MAGIC, EEPROM_MAGIC >> 16); // small compatibility hack
  eepromWriteFloat(ADDR_GAINS + 0, roll_angle_kp);
  eepromWriteFloat(ADDR_GAINS + 4, roll_rate_kp);
  eepromWriteFloat(ADDR_GAINS + 8, roll_rate_ki);
  eepromWriteFloat(ADDR_GAINS + 12, roll_rate_kd);
  EEPROM.commit();
  Serial.println("SAVED");
}

void loadGainsFromEEPROM() {
  // simple check (not robust across platforms)
  // We'll just load values and print them.
  float akp = eepromReadFloat(ADDR_GAINS + 0);
  float rkp = eepromReadFloat(ADDR_GAINS + 4);
  float rki = eepromReadFloat(ADDR_GAINS + 8);
  float rkd = eepromReadFloat(ADDR_GAINS + 12);

  // If EEPROM is blank, values might be garbage. We only accept reasonable ranges:
  if (akp > 0 && akp < 100) roll_angle_kp = akp;
  if (rkp > 0 && rkp < 100) roll_rate_kp = rkp;
  if (rki >= 0 && rki < 10) roll_rate_ki = rki;
  if (rkd >= 0 && rkd < 10) roll_rate_kd = rkd;

  Serial.println("LOADED");
  Serial.print("AKP "); Serial.println(roll_angle_kp);
  Serial.print("RKP "); Serial.println(roll_rate_kp);
  Serial.print("RKI "); Serial.println(roll_rate_ki);
  Serial.print("RKD "); Serial.println(roll_rate_kd);
}

void processSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Commands: AKP <val>, RKP <val>, RKI <val>, RKD <val>, SAVE, LOAD, REQTLM
  if (line.startsWith("AKP")) {
    roll_angle_kp = line.substring(3).toFloat();
    Serial.print("AKP "); Serial.println(roll_angle_kp);
  } else if (line.startsWith("RKP")) {
    roll_rate_kp = line.substring(3).toFloat();
    Serial.print("RKP "); Serial.println(roll_rate_kp);
  } else if (line.startsWith("RKI")) {
    roll_rate_ki = line.substring(3).toFloat();
    Serial.print("RKI "); Serial.println(roll_rate_ki);
  } else if (line.startsWith("RKD")) {
    roll_rate_kd = line.substring(3).toFloat();
    Serial.print("RKD "); Serial.println(roll_rate_kd);
  } else if (line == "SAVE") {
    saveGainsToEEPROM();
  } else if (line == "LOAD") {
    loadGainsFromEEPROM();
  } else if (line == "REQTLM") {
    // request immediate telemetry
    Serial.print("TLM,");
    Serial.print(roll_angle); Serial.print(",");
    Serial.print(gyro_rate); Serial.print(",");
    Serial.print(target_roll); Serial.print(",");
    // deflection approx compute (for telemetry)
    float angle_error = target_roll - roll_angle;
    float desired_rate = roll_angle_kp * angle_error;
    float rate_error = desired_rate - gyro_rate;
    float p_term = roll_rate_kp * rate_error;
    float d_term = roll_rate_kd * (-gyro_rate);
    float pid_output = p_term + (roll_rate_ki * roll_i_term) + d_term;
    int deflection = (int)(pid_output * CORRECTION_DIRECTION);
    Serial.println(deflection);
  } else {
    Serial.print("ERR Unknown: "); Serial.println(line);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU FAIL");
    while (1) delay(10);
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Gyro calibration
  gyro_roll_bias = 0.0;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_roll_bias += g.gyro.y;
    delay(5);
  }
  gyro_roll_bias /= 200.0;

  pinMode(RC_ROLL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_ROLL_PIN), readRoll, CHANGE);

  last_time = micros();

  EEPROM.begin(EEPROM_SIZE);
  loadGainsFromEEPROM();
  Serial.println("READY");
}

String serialBuffer = "";

void loop() {
  // --- Serial command handling (non-blocking) ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialLine(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  // --- Sensors ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - last_time) / 1000000.0f;
  if (dt <= 0 || dt > 0.05) dt = 0.01;
  last_time = now;

  gyro_rate = (g.gyro.y - gyro_roll_bias) * (180.0 / PI);
  float accel_angle = atan2(a.acceleration.x, a.acceleration.z) * (180.0 / PI);
  roll_angle = alpha * (roll_angle + gyro_rate * dt) + (1.0 - alpha) * accel_angle;

  // Safe RC read
  noInterrupts();
  long pulse = rc_roll_val;
  unsigned long signal_age = micros() - last_rc_time;
  interrupts();
  if (signal_age > 100000 || pulse < 800 || pulse > 2200) pulse = 1500;
  float stick_input = (float)pulse - 1500.0;
  target_roll = (stick_input / 500.0) * 35.0;

  // Cascade PID
  float angle_error = target_roll - roll_angle;
  float desired_rate = roll_angle_kp * angle_error;
  desired_rate = constrain(desired_rate, -MAX_DESIRED_RATE, MAX_DESIRED_RATE);

  float rate_error = desired_rate - gyro_rate;

  float p_term = roll_rate_kp * rate_error;
  float d_term = roll_rate_kd * (-gyro_rate);
  float pid_no_i = p_term + d_term;
  int predicted_deflection = (int)(pid_no_i * CORRECTION_DIRECTION);
  predicted_deflection = constrain(predicted_deflection, -SERVO_LIMIT, SERVO_LIMIT);

  // Anti-windup: integrate only if predicted_deflection not saturated
  if (abs(predicted_deflection) < SERVO_LIMIT - 5) {
    roll_i_term += rate_error * dt;
  }
  roll_i_term = constrain(roll_i_term, -I_TERM_LIMIT, I_TERM_LIMIT);

  float pid_output = p_term + (roll_rate_ki * roll_i_term) + d_term;
  int deflection = (int)(pid_output * CORRECTION_DIRECTION);
  deflection = constrain(deflection, -SERVO_LIMIT, SERVO_LIMIT);

  int front_out = FRONT_MID + deflection;
  int back_out  = BACK_MID - deflection;

  pwm.setPWM(SERVO_FRONT_CH, 0, constrain(front_out, FRONT_MID - SERVO_LIMIT, FRONT_MID + SERVO_LIMIT));
  pwm.setPWM(SERVO_BACK_CH, 0, constrain(back_out, BACK_MID - SERVO_LIMIT, BACK_MID + SERVO_LIMIT));

  // Periodic telemetry
  unsigned long now_ms = millis();
  if (now_ms - last_tlm_time >= TLM_INTERVAL_MS) {
    last_tlm_time = now_ms;
    Serial.print("TLM,");
    Serial.print(roll_angle); Serial.print(",");
    Serial.print(gyro_rate); Serial.print(",");
    Serial.print(target_roll); Serial.print(",");
    Serial.println(deflection);
  }
}
