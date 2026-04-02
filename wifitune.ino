/*
 ============================================================================
 BICOPTER OS - FLIGHT READY
 Sensor: MPU6050_light 
 Receiver: 5-Wire PWM (Simplified Interrupt Logic)
 ============================================================================
*/

#include <Wire.h>
#include <MPU6050_light.h> 
#include <ESP32Servo.h> // Added for precise ESC microsecond control

// --- Hardware Setup ---
#define RC_ROLL_PIN  35
#define RC_PITCH_PIN 34
#define RC_THR_PIN   25
#define RC_AUX_PIN   36
#define RC_YAW_PIN   2

//esc connected pins of esp32 (white and violet)
const int ESC_FRONT_PIN = 16;
const int ESC_BACK_PIN  = 17;

const int SERVO_FRONT_PIN = 26;
const int SERVO_BACK_PIN  = 27;

// --- PWM Channels (Required for ESP32 Core v2.x) ---
// Note: ESC channels removed as ESP32Servo auto-allocates them
const int SERVO_FRONT_CH = 2;
const int SERVO_BACK_CH  = 3;

// --- Tuning ---
int FRONT_MID = 310; // Updated to match your successful test
int BACK_MID  = 310;

const int SERVO_LIMIT = 60; // Adjusted based on your 250-370 range

// --- ESC Translated Limits (Microseconds) ---
const int ESC_MIN_US  = 732;   // Was 150 ticks
const int ESC_IDLE_US = 1099;  // Was 225 ticks
const int ESC_MAX_US  = 2002;  // Was 410 ticks
const float TICK_TO_US = 4.8828f;

// --------
// PID VALUES FOR TUNING
// --------

float AKP = 1.930;
float RKP = 0.130;
float RKI = 0.030;
float RKD = 0.000;
float RFF = 50.0;

float PAKP = 2.500;
float PRKP = 0.150;
float PRKI = 0.010;
float PRKD = 0.020;

float YKP = 0.250;
float YKI = 0.010;
float YKD = 0.000;

const float I_TERM_LIMIT = 250.0;
const float MAX_RATE = 400.0;

// END OF PID VALUES 

// --- MPU ---
MPU6050 mpu(Wire); 

// --- State ---
Servo escFront;
Servo escBack;

float roll_angle = 0;
float pitch_angle = 0;

float roll_i = 0;
float pitch_i = 0;
float yaw_i = 0;

unsigned long last_time = 0;
unsigned long last_debug_time = 0; 

// Loop timing variable
unsigned long loop_timer = 0;
const unsigned long LOOP_TIME_US = 4000; // 250Hz control loop

// --- RC Variables ---
volatile unsigned long r_start=0, p_start=0, t_start=0, a_start=0, y_start=0;
volatile unsigned long rc_roll_val=1500, rc_pitch_val=1500, rc_thr_val=1000, rc_aux_val=1000, rc_yaw_val=1500;

// --- Helper ---
float applyDeadband(float value, float deadband) {
  if(abs(value) < deadband) return 0;
  return (value > 0) ? value - deadband : value + deadband;
}

// ---------------- INTERRUPTS OF RC (Integrated from successful test)

void IRAM_ATTR readRoll() {
  if (digitalRead(RC_ROLL_PIN)) r_start = micros();
  else rc_roll_val = micros() - r_start;
}

void IRAM_ATTR readPitch() {
  if (digitalRead(RC_PITCH_PIN)) p_start = micros();
  else rc_pitch_val = micros() - p_start;
}

void IRAM_ATTR readThr() {
  if (digitalRead(RC_THR_PIN)) t_start = micros();
  else rc_thr_val = micros() - t_start;
}

void IRAM_ATTR readAux() {
  if (digitalRead(RC_AUX_PIN)) a_start = micros();
  else rc_aux_val = micros() - a_start;
}

void IRAM_ATTR readYaw() {
  if (digitalRead(RC_YAW_PIN)) y_start = micros();
  else rc_yaw_val = micros() - y_start;
}
// END OF RC INTERRUPTS


// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("Failed to find MPU chip!");
    while (1) { delay(10); }
  }

  // Setup Input Pins
  pinMode(RC_ROLL_PIN, INPUT);
  pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_THR_PIN, INPUT);
  pinMode(RC_AUX_PIN, INPUT);
  pinMode(RC_YAW_PIN, INPUT);

  // Attach Interrupts
  attachInterrupt(RC_ROLL_PIN, readRoll, CHANGE);
  attachInterrupt(RC_PITCH_PIN, readPitch, CHANGE);
  attachInterrupt(RC_THR_PIN, readThr, CHANGE);
  attachInterrupt(RC_AUX_PIN, readAux, CHANGE);
  attachInterrupt(RC_YAW_PIN, readYaw, CHANGE);

  // Setup ESP32Servo for ESCs
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  escFront.setPeriodHertz(50);
  escBack.setPeriodHertz(50);
  escFront.attach(ESC_FRONT_PIN, 500, 2500);
  escBack.attach(ESC_BACK_PIN, 500, 2500);

  // Setup LEDC PWM for Servos (Channel, Frequency = 50Hz, Resolution = 12-bit)
  ledcSetup(SERVO_FRONT_CH, 50, 12);
  ledcAttachPin(SERVO_FRONT_PIN, SERVO_FRONT_CH);
  ledcSetup(SERVO_BACK_CH, 50, 12);
  ledcAttachPin(SERVO_BACK_PIN, SERVO_BACK_CH);

  // Force servos to their neutral (perpendicular) position during startup/calibration
  ledcWrite(SERVO_FRONT_CH, FRONT_MID);
  ledcWrite(SERVO_BACK_CH, BACK_MID);
  // -----------------------------
  delay(1000);
  Serial.print("servos set to neutral position");
  // Initialize ESCs to precise 732us MIN throttle and wait to ensure arming
  escFront.writeMicroseconds(ESC_MIN_US);
  escBack.writeMicroseconds(ESC_MIN_US);
  delay(3000);

  // Auto-calibrates Gyro and Accelerometer offsets
  Serial.println("Calibrating MPU... Keep it level and steady!");
  mpu.calcOffsets(); 
  Serial.println("Calibration Done!");

  last_time = micros();
  loop_timer = micros();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // 1. Throttle the PID and MPU loop to run exactly at 250Hz
  unsigned long now = micros();
  unsigned long elapsed = now - loop_timer;

  if (elapsed < LOOP_TIME_US) {
    // Hand CPU time back to the ESP32 OS to feed the Watchdog
    if (LOOP_TIME_US - elapsed > 1000) {
      delay(1); 
    }
    return; // Exit early and start the loop over
  }
  loop_timer = now;

  // 2. Fetch fresh data from the sensor
  mpu.update(); 

  float dt = (now-last_time)/1000000.0;
  if(dt<=0 || dt>0.05) dt=0.01;
  last_time = now;

  // 3. Read RC Values (Directly from the working interrupt logic)
  long p_r = rc_roll_val;
  long p_p = rc_pitch_val;
  long p_t = rc_thr_val;
  long p_y = rc_yaw_val;

  //reading mpu values from the sensor...
  // MPU6050_light returns gyro in deg/s directly!
  float gr = mpu.getGyroY();
  float gp = mpu.getGyroX();
  float gz = mpu.getGyroZ();

  // MPU6050_light returns accel in g's
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  //------------------------------------------

  // Complementary filter
  roll_angle = 0.98*(roll_angle + gr*dt) + 0.02*(atan2(ax, az)*57.3);
  pitch_angle = 0.98*(pitch_angle + gp*dt) + 0.02*(atan2(ay, az)*57.3);

  //---------------------------------------------------

  // Debug prints
  if (millis() - last_debug_time > 250) {
    Serial.printf("R: %ld | P: %ld | T: %ld | Y: %ld\n", p_r, p_p, p_t, p_y);
    last_debug_time = millis();
  }

  // Calculate Stick Inputs with Deadband
  float r_stick = applyDeadband((p_r-1500.0)/500.0,0.05);
  float p_stick = applyDeadband((p_p-1500.0)/500.0,0.05);
  float y_stick = applyDeadband((p_y-1500.0)/500.0,0.05);

  // Target angles based on stick position
  float target_roll = r_stick*30.0;
  float target_pitch = p_stick*30.0;

  // PID calculations
  float r_des = constrain(AKP*(target_roll-roll_angle),-MAX_RATE,MAX_RATE);
  float p_des = constrain(PAKP*(target_pitch-pitch_angle),-MAX_RATE,MAX_RATE);
  float y_des = y_stick*400.0;

  float r_err = r_des-gr;
  float p_err = p_des-gp;
  float y_err = y_des-gz;

  roll_i += r_err*dt;
  pitch_i += p_err*dt;
  yaw_i += y_err*dt;

  roll_i = constrain(roll_i,-I_TERM_LIMIT,I_TERM_LIMIT);
  pitch_i = constrain(pitch_i,-I_TERM_LIMIT,I_TERM_LIMIT);
  yaw_i = constrain(yaw_i,-I_TERM_LIMIT,I_TERM_LIMIT);

  float r_out = (RKP*r_err)+(RKI*roll_i)+(r_stick*RFF);
  float y_out = (YKP*y_err)+(YKI*yaw_i);
  float p_out = (PRKP*p_err)+(PRKI*pitch_i);

  // Write to Servos (Roll PID correction signs reversed to oppose tilt)
  int front_servo_cmd = constrain(FRONT_MID - (int)r_out + (int)y_out, FRONT_MID - SERVO_LIMIT, FRONT_MID + SERVO_LIMIT);
  int back_servo_cmd  = constrain(BACK_MID  + (int)r_out + (int)y_out, BACK_MID  - SERVO_LIMIT, BACK_MID  + SERVO_LIMIT);

  ledcWrite(SERVO_FRONT_CH, front_servo_cmd);
  ledcWrite(SERVO_BACK_CH, back_servo_cmd);

  //---------------------------------------------------------
  
  // Write to ESCs (Updated using exact microsecond logic)
  // Cut motors if throttle is at the absolute bottom to prevent ground spinning
  if (p_t < 1050) {
    escFront.writeMicroseconds(ESC_MIN_US);
    escBack.writeMicroseconds(ESC_MIN_US);
    roll_i = 0; // Reset I-term on ground to prevent spool-up
    pitch_i = 0;
    yaw_i = 0;
  } else {
    // Map throttle and apply PID scaled to microseconds
    int base_throttle_us = map(p_t, 1000, 2000, ESC_IDLE_US, ESC_MAX_US);
    int p_out_us = (int)(p_out * TICK_TO_US);

    escFront.writeMicroseconds(constrain(base_throttle_us - p_out_us, ESC_MIN_US, ESC_MAX_US));
    escBack.writeMicroseconds(constrain(base_throttle_us + p_out_us, ESC_MIN_US, ESC_MAX_US));
  }
}