#include <Wire.h>          // I2C communication library (used for MPU6050)
#include <Bluepad32.h>    // Bluetooth game controller library

// ==========================================
// CONTROLLER CONFIG
// Handles gamepad connection and movement input
// movementOffset → dynamically shifts robot tilt to move forward/backward
// ==========================================
GamepadPtr myGamepad = nullptr;
float movementOffset = 0.0;
const float maxOffset = 5.0;  // Maximum tilt angle shift for movement

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
// Core control system constants
// These determine how aggressively the robot balances
// ==========================================
double Kp = Add_Values_Later   // Proportional gain → reacts to tilt error
double Ki = Add_Values_Later  // Integral gain → corrects long-term drift
double Kd = Add_Values_Later   // Derivative gain → reduces oscillations

double base_setpoint = Add_Values_Later  // Default upright angle
double setpoint;              // Final target tilt (adjusted via controller)
double input, output;
double error, lastError, cumulativeError;

// ==========================================
// FILTER & TIMING
// Handles sensor fusion and loop timing
// dt → fixed loop interval (100Hz)
// alpha → complementary filter blending factor
// ==========================================
unsigned long lastTime;
const double dt = 0.01;
double robotAngle = 0.0;
const double alpha = 0.96;

const int MPU_addr = 0x68;  // I2C address of MPU6050

// ==========================================
// GAMEPAD CALLBACKS
// Automatically triggered when controller connects/disconnects
// ==========================================
void onConnectedGamepad(GamepadPtr gp) {
myGamepad = gp;  // Store active controller reference
}

void onDisconnectedGamepad(GamepadPtr gp) {
myGamepad = nullptr;  // Clear controller reference
}

// ==========================================
// SETUP FUNCTION
// Runs once at startup
// Initializes Serial, I2C, motors, MPU6050, and Bluetooth controller
// ==========================================
void setup() {
Serial.begin(115200);
Wire.begin(21, 22);  // Initialize I2C (SDA = 21, SCL = 22)

// Configure motor driver pins as outputs
pinMode(PIN_PWM_A, OUTPUT);
pinMode(PIN_IN_A1, OUTPUT);
pinMode(PIN_IN_A2, OUTPUT);
pinMode(PIN_PWM_B, OUTPUT);
pinMode(PIN_IN_B1, OUTPUT);
pinMode(PIN_IN_B2, OUTPUT);

// Wake up MPU6050 (disable sleep mode)
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);

// Initialize Bluetooth controller handling
BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
BP32.forgetBluetoothKeys();  // Allows new pairing

lastTime = millis();
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
int yAxis = myGamepad->axisY();  // Forward/backward input

```
// Map joystick range (-512 to 512) → tilt offset
movementOffset = (float)yAxis / 512.0 * maxOffset;
```

} else {
movementOffset = 0;  // No controller → stay balanced
}

// Final setpoint = base tilt + user input
setpoint = base_setpoint + movementOffset;

// ======================================
// TIMED CONTROL LOOP (100Hz)
// Ensures stable and consistent PID execution
// ======================================
unsigned long currentTime = millis();
if (currentTime - lastTime >= 10) {
readSensorData();   // Get current tilt
calculatePID();     // Compute correction
driveMotors();      // Apply correction


lastTime = currentTime;


}
}

// ==========================================
// SENSOR DATA ACQUISITION
// Reads MPU6050 accelerometer + gyroscope
// Applies complementary filter to estimate tilt angle
// ==========================================
void readSensorData() {
int16_t AcX, AcY, AcZ, GyX;

// Request sensor data from MPU6050
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom((uint8_t)MPU_addr, (size_t)14, (bool)true);

// Read accelerometer values
AcX = Wire.read() << 8 | Wire.read();
AcY = Wire.read() << 8 | Wire.read();
AcZ = Wire.read() << 8 | Wire.read();

Wire.read() << 8 | Wire.read();  // Skip temperature

// Read gyroscope value
GyX = Wire.read() << 8 | Wire.read();

// Convert accelerometer data to angle
double accAngle = atan2((double)AcY, (double)AcZ) * 57.2958;

// Convert gyro data to angular velocity
double gyroRate = (double)GyX / 131.0;

// Complementary filter:
// Combines fast gyro response + stable accel reading
robotAngle = alpha * (robotAngle + gyroRate * dt) + (1.0 - alpha) * accAngle;

input = robotAngle;  // Input to PID controller
}

// ==========================================
// PID CONTROL LOGIC
// Computes correction required to maintain balance
// ==========================================
void calculatePID() {
error = input - setpoint;

// Proportional term → reacts to current tilt
double pTerm = Kp * error;

// Integral term → accumulates past error
cumulativeError += error * dt;
cumulativeError = constrain(cumulativeError, -400.0, 400.0);
double iTerm = Ki * cumulativeError;

// Derivative term → predicts future trend
double dTerm = Kd * ((error - lastError) / dt);

// Total PID output
output = pTerm + iTerm + dTerm;
lastError = error;

// Limit output to motor PWM range
output = constrain(output, -255.0, 255.0);
}

// ==========================================
// MOTOR CONTROL
// Converts PID output into motor speed + direction
// ==========================================
void driveMotors() {
int motorSpeed = abs((int)output);

// Safety: stop motors if robot falls too much
if (abs(robotAngle) > 45.0) {
analogWrite(PIN_PWM_A, 0);
analogWrite(PIN_PWM_B, 0);
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

// Apply speed using PWM
analogWrite(PIN_PWM_A, motorSpeed);
analogWrite(PIN_PWM_B, motorSpeed);
}
