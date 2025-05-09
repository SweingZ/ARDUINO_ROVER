#include <Servo.h>
#include <AFMotor.h>
#define Echo A0
#define Trig A1
#define motor 10
#define DEFAULT_SPEED 150
#define spoint 103

// Gear speeds
#define FIRST_GEAR 150
#define SECOND_GEAR 170
#define THIRD_GEAR 190
#define FOURTH_GEAR 210

char value;
int distance;
int currentSpeed = DEFAULT_SPEED;
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
  setSpeed(DEFAULT_SPEED); // Initialize with default speed
}

void loop() {
  // Obstacle(); // Uncomment if needed
  control();
}

void control() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);

    // Handle Bluetooth, Voice, and Gesture commands
    if (value == 'F') { // Bluetooth forward (uses default speed)
      setSpeed(DEFAULT_SPEED);
      forward();
    } else if (value == 'B') { // Bluetooth/gesture backward
      setSpeed(DEFAULT_SPEED);
      backward();
    } else if (value == 'L') { // Bluetooth/gesture left
      setSpeed(DEFAULT_SPEED);
      left();
    } else if (value == 'R') { // Bluetooth/gesture right
      setSpeed(DEFAULT_SPEED);
      right();
    } else if (value == 'S') { // Bluetooth/gesture stop
      Stop();
    } 
    // Gear commands from gesture control
    else if (value == '1') { // First gear (F1)
      setSpeed(FIRST_GEAR);
      forward();
    } else if (value == '2') { // Second gear (F2)
      setSpeed(SECOND_GEAR);
      forward();
    } else if (value == '3') { // Third gear (F3)
      setSpeed(THIRD_GEAR);
      forward();
    } else if (value == '4') { // Fourth gear (F4)
      setSpeed(FOURTH_GEAR);
      forward();
    }
    // Voice commands (unchanged)
    else if (value == '^') { // Voice forward (timed)
      setSpeed(DEFAULT_SPEED);
      forward();
      delay(2000);
      Stop();
    } else if (value == '-') { // Voice backward (timed)
      setSpeed(DEFAULT_SPEED);
      backward();
      delay(2000);
      Stop();
    } else if (value == '<') { // Voice left
      int leftDistance = leftsee();
      servo.write(spoint);
      delay(800);
      if (leftDistance >= 10) {
        setSpeed(DEFAULT_SPEED);
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
        setSpeed(DEFAULT_SPEED);
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

void setSpeed(int speed) {
  currentSpeed = speed;
  M1.setSpeed(currentSpeed);
  M2.setSpeed(currentSpeed);
  M3.setSpeed(currentSpeed);
  M4.setSpeed(currentSpeed);
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