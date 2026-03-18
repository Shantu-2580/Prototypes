#include <AFMotor.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 3); // RX, TX

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

void setup() {
  Serial.begin(9600);       // 🔥 ADD THIS
  espSerial.begin(9600);

  motor1.setSpeed(255);
  motor2.setSpeed(255);
}
void forward() {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);  // 🔥 reversed
}

void backward() {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);   // 🔥 reversed
}

void left() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void right() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}
void stopRobot() {
  Serial.println("STOP");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

void loop() {
  if (espSerial.available()) {
    char cmd = espSerial.read();

    Serial.print("Received: ");  // 🔥 DEBUG
    Serial.println(cmd);

    switch(cmd) {
      case 'F': forward(); break;
      case 'B': backward(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      case 'S': stopRobot(); break;
    }
  }
}