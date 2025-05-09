#include <Servo.h>
#include <AFMotor.h>
#define Echo A0
#define Trig A1
#define motor 10
#define Speed 150
#define spoint 103

char value;
int distance;
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}

void loop() {
  // Obstacle(); // Uncomment if needed
  control();
}

void control() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);

    // Handle both Bluetooth and Voice commands
    if (value == 'F') { // Bluetooth forward
      forward();
    } else if (value == 'B') { // Bluetooth backward
      backward();
    } else if (value == 'L') { // Bluetooth left
      left();
    } else if (value == 'R') { // Bluetooth right
      right();
    } else if (value == 'S') { // Bluetooth stop
      Stop();
    } else if (value == '^') { // Voice forward (timed)
      forward();
      delay(2000);  // Move for 2 seconds
      Stop();
    } else if (value == '-') { // Voice backward (timed)
      backward();
      delay(2000);  // Move for 2 seconds
      Stop();
    } else if (value == '<') { // Voice left
      int leftDistance = leftsee();
      servo.write(spoint);
      delay(800);
      if (leftDistance >= 10) {
        left();
        delay(500);
        Stop();
      } else {
        Stop();
      }
    } else if (value == '>') { // Voice right
      int rightDistance = rightsee();
      servo.write(spoint);
      delay(800);
      if (rightDistance >= 10) {
        right();
        delay(500);
        Stop();
      } else {
        Stop();
      }
    } else if (value == '*') { // Voice stop
      Stop();
    }
  }
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  return t / 29 / 2;
}

int rightsee() {
  servo.write(20);
  delay(800);
  return ultrasonic();
}

int leftsee() {
  servo.write(180);
  delay(800);
  return ultrasonic();
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void right() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void left() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}