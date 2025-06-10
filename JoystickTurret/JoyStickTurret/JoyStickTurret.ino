#include <Servo.h>

// Servo objects
Servo panServo;
Servo tiltServo;
Servo extraServo; // Laser servo

// Pins
const int joyXPin = A0;
const int joyYPin = A1;
const int buttonPin = 2;
const int panServoPin = 9;
const int tiltServoPin = 10;
const int extraServoPin = 11;

// Angle tracking
int panAngle = 90;
int tiltAngle = 90;

// Movement speed
const int stepSize = 1;
const int deadzone = 100;

// Button state tracking
bool lastButtonState = HIGH;
bool laserOn = false;

void setup() {
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  extraServo.attach(extraServoPin);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  extraServo.write(90); // Laser off position

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  int joyX = analogRead(joyXPin);
  int joyY = analogRead(joyYPin);

  // Handle pan
  if (joyX < 512 - deadzone) {
    panAngle -= stepSize;
  } else if (joyX > 512 + deadzone) {
    panAngle += stepSize;
  }

  // Handle tilt
  if (joyY < 512 - deadzone) {
    tiltAngle += stepSize;
  } else if (joyY > 512 + deadzone) {
    tiltAngle -= stepSize;
  }

  // Clamp angles
  panAngle = constrain(panAngle, 0, 180);
  tiltAngle = constrain(tiltAngle, 0, 180);

  // Move servos
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  // Read current button state
  bool currentButtonState = digitalRead(buttonPin);

  // Toggle logic: detect rising edge
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    laserOn = !laserOn; // Toggle
    Serial.print("Laser toggled: ");
    Serial.println(laserOn ? "ON" : "OFF");
  }

  lastButtonState = currentButtonState;

  // Set laser servo position
  if (laserOn) {
    extraServo.write(60); // Laser on position
  } else {
    extraServo.write(90); // Laser off position
  }

  delay(20);
}
