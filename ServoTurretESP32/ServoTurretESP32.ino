#include <ESP32Servo.h>

Servo panServo;
Servo tiltServo;

const int panPin = 14;      // Adjust based on wiring
const int tiltPin = 12;     // Adjust based on wiring
const int pumpPin = 13;     // Relay or MOSFET control pin

int panAngle = 90;
int tiltAngle = 90;
bool pumpOn = false;

void setup() {
  Serial.begin(115200);

  panServo.attach(panPin);
  tiltServo.attach(tiltPin);

  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW); // Pump off initially

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.startsWith("pan:")) {
      int p = cmd.indexOf("pan:");
      int t = cmd.indexOf("tilt:");
      int f = cmd.indexOf("fire:");

      panAngle = cmd.substring(p + 4, t).toInt();
      tiltAngle = cmd.substring(t + 5, f).toInt();
      pumpOn = cmd.substring(f + 5).toInt();

      panAngle = constrain(panAngle, 0, 180);
      tiltAngle = constrain(tiltAngle, 0, 180);

      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
      digitalWrite(pumpPin, pumpOn ? HIGH : LOW);
    }
  }
}
