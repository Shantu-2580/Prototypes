#include <Wire.h>          // I2C communication library (used for MPU6050)
#include <Bluepad32.h>     // Bluetooth game controller library

// ==========================================
// CONTROLLER CONFIG
// Handles gamepad connection and movement input
// movementOffset → dynamically shifts robot tilt to move forward/backward
// ==========================================
GamepadPtr myGamepad = nullptr;
float movementOffset = 0.0;
const float maxOffset = 5.0;       // Maximum tilt angle shift for movement
const int GAMEPAD_DEADZONE = 20;   // Joystick center deadzone
const unsigned long WATCHDOG_MS = 500;  // Controller timeout (ms)
unsigned long lastGamepadUpdate = 0;

// ==========================================
// PIN DEFINITIONS
// Maps ESP32 GPIO pins to motor driver (TB6612FNG)
// PWM pins control speed, IN pins control direction
// ==========================================
#define PIN_PWM_A  33
#define PIN_IN_A1  14
#define PIN_IN_A2  15

#define PIN_PWM_B  32
#define PIN_IN_B1  26
#define PIN_IN_B2  27

// ==========================================
// PID PARAMETERS
// Core control system constants - TUNE THESE FOR YOUR ROBOT
// Start with Kp, then Kd, then Ki
// ==========================================
double Kp = 25.0;    // Proportional gain → reacts to tilt error
double Ki = 0.8;     // Integral gain → corrects long-term drift
double Kd = 1.2;     // Derivative gain → reduces oscillations

double base_setpoint = 0.0;   // Calibrated upright angle (set during startup)
double setpoint;              // Final target tilt (adjusted via controller)
double input, output;
double error, lastError, cumulativeError;

// ==========================================
// FILTER & TIMING
// Handles sensor fusion and loop timing
// dt → fixed loop interval (100Hz = 10ms)
// alpha → complementary filter blending factor (0.96 = trust gyro 96%, accel 4%)
// ==========================================
unsigned long lastTime;
const double dt = 0.01;
double robotAngle = 0.0;
const double alpha = 0.96;

const int MPU_addr = 0x68;  // I2C address of MPU6050

// ==========================================
// MOTOR & SAFETY CONSTANTS
// ==========================================
const int MIN_PWM = 25;           // Motor deadband compensation (TB6612FNG)
const double MAX_ANGLE = 45.0;    // Fall detection threshold (degrees)
const double INTEGRAL_LIMIT = 400.0;  // Anti-windup limit

// ==========================================
// CALIBRATION STATE
// ==========================================
bool calibrated = false;
const int CALIBRATION_SAMPLES = 200;  // 2 seconds at 100Hz
int calibrationCount = 0;
double calibrationSum = 0.0;

// ==========================================
// GAMEPAD CALLBACKS
// Automatically triggered when controller connects/disconnects
// ==========================================
void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;  // Store active controller reference
  Serial.println("Gamepad connected!");
}

void onDisconnectedGamepad(GamepadPtr gp) {
  myGamepad = nullptr;  // Clear controller reference
  movementOffset = 0;
  Serial.println("Gamepad disconnected!");
}

// ==========================================
// SETUP FUNCTION
// Runs once at startup
// Initializes Serial, I2C, motors, MPU6050, and Bluetooth controller
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(100);  // Allow serial to stabilize
  Serial.println("\n=== Self-Balancing Bot Starting ===");

  Wire.begin(21, 22);  // Initialize I2C (SDA = 21, SCL = 22)

  // Configure motor driver pins as outputs
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_IN_A1, OUTPUT);
  pinMode(PIN_IN_A2, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  pinMode(PIN_IN_B1, OUTPUT);
  pinMode(PIN_IN_B2, OUTPUT);

  // Stop motors initially
  analogWrite(PIN_PWM_A, 0);
  analogWrite(PIN_PWM_B, 0);
  digitalWrite(PIN_IN_A1, LOW);
  digitalWrite(PIN_IN_A2, LOW);
  digitalWrite(PIN_IN_B1, LOW);
  digitalWrite(PIN_IN_B2, LOW);

  // Initialize MPU6050
  if (!initMPU6050()) {
    Serial.println("FATAL: MPU6050 initialization failed!");
    while (1) {
      delay(1000);  // Halt forever
    }
  }

  // Initialize Bluetooth controller handling
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();  // Allows new pairing
  Serial.println("Bluetooth ready - pair your controller");

  lastTime = millis();
}

// ==========================================
// MPU6050 INITIALIZATION & VERIFICATION
// ==========================================
bool initMPU6050() {
  // Check WHO_AM_I register (should return 0x68)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_addr, (size_t)1, (bool)true);

  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    if (whoami != 0x68) {
      Serial.printf("MPU6050 WHO_AM_I = 0x%02X (expected 0x68)\n", whoami);
      return false;
    }
    Serial.println("MPU6050 detected (WHO_AM_I = 0x68)");
  } else {
    Serial.println("MPU6050 not responding on I2C");
    return false;
  }

  // Wake up MPU6050 (disable sleep mode)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Clear sleep bit
  Wire.endTransmission(true);

  // Set accelerometer range to ±2g (default)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // ±2g
  Wire.endTransmission(true);

  // Set gyroscope range to ±250°/s (default)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission(true);

  // Set DLPF (Digital Low Pass Filter) to 44Hz for accel, 42Hz for gyro
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // CONFIG
  Wire.write(0x03);  // DLPF_CFG = 3
  Wire.endTransmission(true);

  delay(100);  // Allow sensor to stabilize
  Serial.println("MPU6050 configured successfully");
  return true;
}

// ==========================================
// MAIN LOOP
// Runs continuously
// Handles controller input + executes control loop at 100Hz
// ==========================================
void loop() {
  BP32.update();  // Update controller state

  // ======================================
  // CONTROLLER INPUT PROCESSING
  // Converts joystick input into tilt offset
  // ======================================
  if (myGamepad && myGamepad->isConnected()) {
    int yAxis = myGamepad->axisY();  // Forward/backward input (-512 to 512)

    // Apply deadzone to prevent drift
    if (abs(yAxis) > GAMEPAD_DEADZONE) {
      movementOffset = (float)yAxis / 512.0 * maxOffset;
    } else {
      movementOffset = 0;
    }
    lastGamepadUpdate = millis();

  } else if (millis() - lastGamepadUpdate > WATCHDOG_MS) {
    // Safety: stop if controller lost or disconnected
    movementOffset = 0;
  }

  // Final setpoint = calibrated base tilt + user input
  setpoint = base_setpoint + movementOffset;

  // ======================================
  // TIMED CONTROL LOOP (100Hz)
  // Ensures stable and consistent PID execution
  // ======================================
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 10) {
    readSensorData();   // Get current tilt

    // Run calibration during first 2 seconds
    if (!calibrated) {
      runCalibration();
    } else {
      calculatePID();     // Compute correction
      driveMotors();      // Apply correction
    }

    lastTime = currentTime;
  }
}

// ==========================================
// CALIBRATION ROUTINE
// Averages accelerometer angle over 2 seconds while stationary
// ==========================================
void runCalibration() {
  calibrationSum += robotAngle;
  calibrationCount++;

  if (calibrationCount >= CALIBRATION_SAMPLES) {
    base_setpoint = calibrationSum / CALIBRATION_SAMPLES;
    calibrated = true;
    Serial.printf("Calibration complete! Base setpoint = %.2f deg\n", base_setpoint);
    Serial.println("PID control active. Robot should balance now.");
  }
}

// ==========================================
// SENSOR DATA ACQUISITION
// Reads MPU6050 accelerometer + gyroscope
// Applies complementary filter to estimate tilt angle
// ==========================================
void readSensorData() {
  int16_t AcX, AcY, AcZ, GyX;

  // Request sensor data from MPU6050 (single transmission)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starting register: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_addr, (size_t)14, (bool)true);

  // Read accelerometer values (6 bytes)
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  Wire.read() << 8 | Wire.read();  // Skip temperature (2 bytes)

  // Read gyroscope X value (2 bytes)
  GyX = Wire.read() << 8 | Wire.read();

  // Convert accelerometer data to angle (degrees)
  // atan2(AcY, AcZ) gives angle from vertical in radians
  double accAngle = atan2((double)AcY, (double)AcZ) * 57.2958;  // rad to deg

  // Convert gyro data to angular velocity (deg/s)
  // Sensitivity: 131 LSB/°/s for ±250°/s range
  double gyroRate = (double)GyX / 131.0;

  // Complementary filter:
  // Combines fast gyro response + stable accel reading
  // robotAngle = alpha * (robotAngle + gyroRate * dt) + (1 - alpha) * accAngle
  robotAngle = alpha * (robotAngle + gyroRate * dt) + (1.0 - alpha) * accAngle;

  input = robotAngle;  // Input to PID controller
}

// ==========================================
// PID CONTROL LOGIC
// Computes correction required to maintain balance
// Includes anti-windup and output limiting
// ==========================================
void calculatePID() {
  error = input - setpoint;

  // Proportional term → reacts to current tilt
  double pTerm = Kp * error;

  // Integral term → accumulates past error
  cumulativeError += error * dt;

  // Anti-windup: clamp integral term
  cumulativeError = constrain(cumulativeError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  double iTerm = Ki * cumulativeError;

  // Derivative term → predicts future trend
  double dTerm = Kd * ((error - lastError) / dt);

  // Total PID output (unconstrained)
  double outputUnconstrained = pTerm + iTerm + dTerm;

  // Anti-windup: stop integrating when output saturates
  if (outputUnconstrained > 255.0 || outputUnconstrained < -255.0) {
    cumulativeError -= error * dt;  // Undo the integration step
  }

  // Limit output to motor PWM range
  output = constrain(outputUnconstrained, -255.0, 255.0);
  lastError = error;

  // Debug telemetry (1Hz at 100Hz loop)
  static int debugCounter = 0;
  if (++debugCounter >= 100) {
    Serial.printf("Angle: %.2f | Setpoint: %.2f | Out: %.1f | P: %.1f I: %.1f D: %.1f\n",
      robotAngle, setpoint, output, pTerm, iTerm, dTerm);
    debugCounter = 0;
  }
}

// ==========================================
// MOTOR CONTROL
// Converts PID output into motor speed + direction
// Includes deadband compensation and fall detection
// ==========================================
void driveMotors() {
  int motorSpeed = abs((int)output);

  // Safety: stop motors if robot falls too much (>45 degrees)
  if (abs(robotAngle) > MAX_ANGLE) {
    analogWrite(PIN_PWM_A, 0);
    analogWrite(PIN_PWM_B, 0);
    digitalWrite(PIN_IN_A1, LOW);
    digitalWrite(PIN_IN_A2, LOW);
    digitalWrite(PIN_IN_B1, LOW);
    digitalWrite(PIN_IN_B2, LOW);
    return;
  }

  // Direction control based on tilt correction
  if (output > 0) {
    // Move forward
    digitalWrite(PIN_IN_A1, HIGH);
    digitalWrite(PIN_IN_A2, LOW);
    digitalWrite(PIN_IN_B1, HIGH);
    digitalWrite(PIN_IN_B2, LOW);
  } else {
    // Move backward
    digitalWrite(PIN_IN_A1, LOW);
    digitalWrite(PIN_IN_A2, HIGH);
    digitalWrite(PIN_IN_B1, LOW);
    digitalWrite(PIN_IN_B2, HIGH);
  }

  // Apply deadband compensation: motors need minimum PWM to overcome friction
  if (motorSpeed > 0) {
    motorSpeed = max(motorSpeed, MIN_PWM);
  }

  // Apply speed using PWM
  analogWrite(PIN_PWM_A, motorSpeed);
  analogWrite(PIN_PWM_B, motorSpeed);
}