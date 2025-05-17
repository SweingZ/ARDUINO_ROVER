#include <Servo.h>
#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

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
bool autoPilotMode = false;  // Track if we're in auto-pilot mode
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  servo.write(spoint);  // Center the servo at startup
  setSpeed(DEFAULT_SPEED);

    /* Initialize the HMC5883 sensor */
  if(!mag.begin()) {
    Serial.println("Could not find HMC5883 sensor!");
    while(1);
  }
}

void loop() {
  control();  // Always check for commands first
  
  // Send compass data periodically (every 500ms)
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 500) {
    sendCompassData();
    lastSendTime = millis();
  }
  
  if (autoPilotMode) {
    Obstacle();  // Run obstacle avoidance if still in auto-pilot
  }
}

void control() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);

    // Handle Bluetooth, Voice, and Gesture commands
    if (value == 'F') { // Bluetooth forward
      autoPilotMode = false;
      setSpeed(DEFAULT_SPEED);
      forward();
    } else if (value == 'B') { // Backward
      autoPilotMode = false;
      setSpeed(DEFAULT_SPEED);
      backward();
    } else if (value == 'L') { // Left
      autoPilotMode = false;
      setSpeed(DEFAULT_SPEED);
      left();
    } else if (value == 'R') { // Right
      autoPilotMode = false;
      setSpeed(DEFAULT_SPEED);
      right();
    } else if (value == 'S') { // Stop
      autoPilotMode = false;
      Stop();
    } 
    // Gear commands
    else if (value == '1') { // First gear
      autoPilotMode = false;
      setSpeed(FIRST_GEAR);
      forward();
    } else if (value == '2') { // Second gear
      autoPilotMode = false;
      setSpeed(SECOND_GEAR);
      forward();
    } else if (value == '3') { // Third gear
      autoPilotMode = false;
      setSpeed(THIRD_GEAR);
      forward();
    } else if (value == '4') { // Fourth gear
      autoPilotMode = false;
      setSpeed(FOURTH_GEAR);
      forward();
    } else if (value == 'A') { // Autopilot Mode
      autoPilotMode = true;
      setSpeed(DEFAULT_SPEED);
    }
    // Voice commands
    else if (value == '^') { // Voice forward
      autoPilotMode = false;
      setSpeed(DEFAULT_SPEED);
      forward();
      delay(2000);
      Stop();
    } else if (value == '-') { // Voice backward
      autoPilotMode = false;
      setSpeed(DEFAULT_SPEED);
      backward();
      delay(2000);
      Stop();
    } else if (value == '<') { // Voice left
      autoPilotMode = false;
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
      autoPilotMode = false;
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
      autoPilotMode = false;
      Stop();
    }
  }
}

void Obstacle() {
  distance = ultrasonic();
  
  if (distance <= 30) {  // Obstacle detected
    backward();
    delay(200);
    Stop();
    delay(200);
    
    // Check left distance
    servo.write(180);  // Look left
    delay(800);        // Wait for servo to move
    int leftDist = ultrasonic();
    
    // Check right distance
    servo.write(20);   // Look right
    delay(800);        // Wait for servo to move
    int rightDist = ultrasonic();
    
    // Return servo to center
    servo.write(spoint);
    delay(500);
    
    // Decide which way to turn
    if (leftDist > rightDist) {
      left();
      delay(500);  // Turn for 500ms
    } else {
      right();
      delay(500);  // Turn for 500ms
    }
    
    Stop();
    delay(200);
  } else {
    // No obstacle, keep moving forward
    forward();
  }

  delay(50);
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

void sendCompassData() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Calculate heading */
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22; // Update this for your location
  heading += declinationAngle;
  
  // Normalize to 0-2π
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
  
  // Convert to degrees
  float headingDegrees = heading * 180/M_PI;
  
  /* Send data in a compact format: C,X,Y,Z,H */
  Serial.print("C,");
  Serial.print(event.magnetic.x); Serial.print(",");
  Serial.print(event.magnetic.y); Serial.print(",");
  Serial.print(event.magnetic.z); Serial.print(",");
  Serial.println(headingDegrees);
}