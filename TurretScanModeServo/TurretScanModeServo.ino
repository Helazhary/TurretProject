#include <Servo.h>

Servo myServoPan;
Servo myServoTilt;

int panAngle = 90;
int tiltAngle = 90;
String inputString = "";

void setup() {
  myServoPan.attach(10);
  myServoTilt.attach(9);

  myServoPan.write(panAngle);
  myServoTilt.write(tiltAngle);
  Serial.setTimeout(10);
  Serial.begin(9600);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      // Parse angles
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        int pan = inputString.substring(0, commaIndex).toInt();
        int tilt = inputString.substring(commaIndex + 1).toInt();

        // Clamp angles
        pan = constrain(pan, 0, 180);
        tilt = constrain(tilt, 0, 180);

        myServoPan.write(pan);
        myServoTilt.write(tilt);
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }
}
