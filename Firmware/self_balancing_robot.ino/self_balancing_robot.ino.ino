#define IR_RECEIVE_PIN 4
#define NO_LED_FEEDBACK_CODE
#define USE_CALLBACK_FOR_TINY_RECEIVER

#include <TinyIRReceiver.hpp>
#include <Wire.h>

// Control tuning
const float BASE_THETA_REF = 0.94;
const float KP = 90.0;
const float KI = 40.0;
const float KD = 1.8;
const int PWM_MIN = 20;
const float I_TERM_MAX = 80.0;
const float FILTER_TAU = 2.0;

// Drive / speed loop
const float DRIVE_RPM = 40.0;
const float DRIVE_RAMP_RATE = 30.0;
const float STOP_RAMP_RATE = 60.0;
const unsigned long ENCODER_INTERVAL_US = 20000UL;
const float SPEED_FILTER_ALPHA = 0.50;
const float SPEED_KV = 0.025;
const float MAX_SPEED_REF_CORRECTION = 1.50;

// IR commands
const uint8_t IR_CMD_UP = 0x46;
const uint8_t IR_CMD_LEFT = 0x44;
const uint8_t IR_CMD_DOWN = 0x15;
const uint8_t IR_CMD_RIGHT = 0x43;
const uint8_t IR_CMD_OK = 0x40;

// Startup
const int CAL_DONE_PWM = 60;
const unsigned long CAL_DONE_PULSE_MS = 120;
const unsigned long POSITION_TIME_MS = 4000;

// Motor pins
const int AIN1 = 5;
const int AIN2 = 6;
const int PWMA = 9;
const int BIN1 = 7;
const int BIN2 = 8;
const int PWMB = 10;
const int STBY = 11;

// Encoder pins / calibration
const int ENC_LEFT_A = 2;
const int ENC_LEFT_B = 12;
const int ENC_RIGHT_A = 3;
const int ENC_RIGHT_B = 13;
const float LEFT_COUNTS_PER_REV = 685.6;
const float RIGHT_COUNTS_PER_REV = 693.8;

// MPU6050
const uint8_t MPU_ADDR = 0x68;

// State
float theta = 0.0;
float gyroBias = 0.0;
float integralTerm = 0.0;
float thetaRef = BASE_THETA_REF;
float requestedRPM = 0.0;
float targetRPM = 0.0;
float leftRPM = 0.0;
float rightRPM = 0.0;
float robotRPMRaw = 0.0;
float robotRPMFiltered = 0.0;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long previousLeftCount = 0;
long previousRightCount = 0;

unsigned long lastMicros = 0;
unsigned long lastEncoderMicros = 0;
unsigned long lastRampMicros = 0;

volatile uint8_t pendingIRCommand = 0;
volatile bool irCommandPending = false;

void writeMPU(uint8_t reg, uint8_t value);
bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz);
void driveMotors(float command);
void driveForward(int pwm);
void driveBackward(int pwm);
void stopMotors();
void calibrationDonePulse();
void leftEncoderISR();
void rightEncoderISR();
void getEncoderCounts(long &left, long &right);
void updateEncoderSpeed();
void updateSpeedController();
void processPendingIRCommand();
void updateSpeedRamp();

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  stopMotors();

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, RISING);

  initPCIInterruptForTinyIRReceiver();

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  Wire.clearWireTimeoutFlag();

  writeMPU(0x6B, 0x00);  // Wake MPU6050
  delay(100);
  writeMPU(0x1B, 0x00);  // Gyro ±250 deg/s
  writeMPU(0x1C, 0x00);  // Accelerometer ±2 g
  writeMPU(0x1A, 0x02);  // DLPF CFG = 2
  delay(500);

  // Gyro calibration: robot must remain still; can be on its side
  long gyroSum = 0;
  int goodSamples = 0;
  const int CAL_SAMPLES = 1000;

  while (goodSamples < CAL_SAMPLES) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      gyroSum += gx;
      goodSamples++;
    }
    delay(2);
  }

  gyroBias = (gyroSum / (float)goodSamples) / 131.0;

  calibrationDonePulse();
  stopMotors();
  digitalWrite(STBY, LOW);
  delay(POSITION_TIME_MS);

  int16_t ax, ay, az, gx, gy, gz;
  while (!readMPU(ax, ay, az, gx, gy, gz)) {}
  theta = atan2((float)ay, -(float)az) * 180.0 / PI;
  integralTerm = 0.0;

  getEncoderCounts(previousLeftCount, previousRightCount);
  leftRPM = 0.0;
  rightRPM = 0.0;
  robotRPMRaw = 0.0;
  robotRPMFiltered = 0.0;
  thetaRef = BASE_THETA_REF;
  requestedRPM = 0.0;
  targetRPM = 0.0;

  lastEncoderMicros = micros();
  lastRampMicros = micros();
  lastMicros = micros();

  digitalWrite(STBY, HIGH);
}

void loop() {
  processPendingIRCommand();
  updateSpeedRamp();
  updateEncoderSpeed();
  updateSpeedController();

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  if (dt <= 0.0 || dt > 0.05) return;

  int16_t ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) return;

  float thetaAcc = atan2((float)ay, -(float)az) * 180.0 / PI;
  float omega = -(gx / 131.0 - gyroBias);

  // Complementary filter
  float alpha = FILTER_TAU / (FILTER_TAU + dt);
  theta = alpha * (theta + omega * dt) + (1.0 - alpha) * thetaAcc;

  float error = theta - thetaRef;

  integralTerm += KI * error * dt;
  if (fabs(targetRPM) < 3.0) {
    integralTerm += KI * error * dt;
  }

  integralTerm = constrain(integralTerm, -I_TERM_MAX, I_TERM_MAX);

  float command = KP * error + integralTerm + KD * omega;
  command = constrain(command, -255.0, 255.0);
  driveMotors(command);
}

void leftEncoderISR() {
  if (digitalRead(ENC_LEFT_B)) leftEncoderCount++;
  else leftEncoderCount--;
}

void rightEncoderISR() {
  if (digitalRead(ENC_RIGHT_B)) rightEncoderCount++;
  else rightEncoderCount--;
}

void getEncoderCounts(long &left, long &right) {
  noInterrupts();
  left = leftEncoderCount;
  right = rightEncoderCount;
  interrupts();
}

void updateEncoderSpeed() {
  unsigned long now = micros();
  unsigned long dtUs = now - lastEncoderMicros;
  if (dtUs < ENCODER_INTERVAL_US) return;

  long leftNow, rightNow;
  getEncoderCounts(leftNow, rightNow);

  long deltaLeftRaw = leftNow - previousLeftCount;
  long deltaRightRaw = rightNow - previousRightCount;
  float dtSeconds = dtUs / 1000000.0;

  leftRPM = (deltaLeftRaw / LEFT_COUNTS_PER_REV) * (60.0 / dtSeconds);
  rightRPM = -((deltaRightRaw / RIGHT_COUNTS_PER_REV) * (60.0 / dtSeconds));
  robotRPMRaw = 0.5 * (leftRPM + rightRPM);
  robotRPMFiltered += SPEED_FILTER_ALPHA * (robotRPMRaw - robotRPMFiltered);

  previousLeftCount = leftNow;
  previousRightCount = rightNow;
  lastEncoderMicros = now;
}

void handleReceivedTinyIRData() {
  if (TinyIRReceiverData.Flags & IRDATA_FLAGS_PARITY_FAILED) return;
  if (TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT) return;

  uint8_t command = TinyIRReceiverData.Command;
  if (command == IR_CMD_UP || command == IR_CMD_DOWN || command == IR_CMD_OK ||) 
  {
    pendingIRCommand = command;
    irCommandPending = true;
  }
}

void processPendingIRCommand() {
  if (!irCommandPending) return;

  noInterrupts();
  uint8_t command = pendingIRCommand;
  irCommandPending = false;
  interrupts();

  if (command == IR_CMD_UP) requestedRPM = DRIVE_RPM;
  else if (command == IR_CMD_DOWN) requestedRPM = -DRIVE_RPM;
  else if (command == IR_CMD_OK) requestedRPM = 0.0;
}

void updateSpeedRamp() {
  unsigned long now = micros();
  unsigned long dtUs = now - lastRampMicros;
  if (dtUs == 0) return;

  lastRampMicros = now;
  float dtSeconds = dtUs / 1000000.0;
  if (dtSeconds > 0.05) dtSeconds = 0.05;

  float rampRate = fabs(requestedRPM) < 0.001 ? STOP_RAMP_RATE : DRIVE_RAMP_RATE;
  float maxStep = rampRate * dtSeconds;
  float difference = requestedRPM - targetRPM;

  if (difference > maxStep) targetRPM += maxStep;
  else if (difference < -maxStep) targetRPM -= maxStep;
  else targetRPM = requestedRPM;
}

void updateSpeedController() {
  float speedError = targetRPM - robotRPMFiltered;
  float correction = SPEED_KV * speedError;
  correction = constrain(correction,
                         -MAX_SPEED_REF_CORRECTION,
                         MAX_SPEED_REF_CORRECTION);
  thetaRef = BASE_THETA_REF + correction;
}

void driveMotors(float command) {
  float magnitude = fabs(command);
  if (magnitude < 2.0) {
    stopMotors();
    return;
  }

  int pwm = (int)magnitude;
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  pwm = constrain(pwm, PWM_MIN, 255);

  if (command > 0) driveForward(pwm);
  else driveBackward(pwm);
}

void calibrationDonePulse() {
  digitalWrite(STBY, HIGH);
  driveForward(CAL_DONE_PWM);
  delay(CAL_DONE_PULSE_MS);
  stopMotors();
  digitalWrite(STBY, LOW);
}

void driveForward(int pwm) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void driveBackward(int pwm) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void writeMPU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (received != 14) return false;

  ax = ((int16_t)Wire.read() << 8) | Wire.read();
  ay = ((int16_t)Wire.read() << 8) | Wire.read();
  az = ((int16_t)Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  gx = ((int16_t)Wire.read() << 8) | Wire.read();
  gy = ((int16_t)Wire.read() << 8) | Wire.read();
  gz = ((int16_t)Wire.read() << 8) | Wire.read();

  return true;
}
