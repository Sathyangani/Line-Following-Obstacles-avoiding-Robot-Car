#define MS A2      // Middle Sensor
#define LS A4      // Left sensor
#define RS A0      // Right sensor

#define enA 5 // Enable1 L298 Pin enA
#define enB 10 // Enable2 L298 Pin enB
#define LM1 6  // left motor
#define LM2 7  // left motor
#define RM1 8  // right motor
#define RM2 9  // right motor

// Define pins for Ultrasonic sensor
#define trigPin A5
#define echoPin A3
#define obstacleDistance 10 // Distance in cm to consider an obstacle

// Define pin for Buzzer
#define buzzerPin 3

bool wasObstacleDetected = false;

void setup() {
  Serial.begin(9600);
  pinMode(MS, INPUT);
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);  // Set the buzzer pin as output

  analogWrite(enA, 90); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 90);
}

void loop() {
  bool middleSensor = digitalRead(MS);
  bool leftSensor = digitalRead(LS);
  bool rightSensor = digitalRead(RS);
  int distance = measureDistance();

  if (distance < obstacleDistance) { // If an obstacle is detected
    stopMotors();
    Serial.println("Obstacle detected, stopping");
    digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
    wasObstacleDetected = true; // Set flag to true
  } else {
    digitalWrite(buzzerPin, LOW); // Turn off the buzzer
    if (wasObstacleDetected) {
      // Resume forward movement if obstacle was previously detected
      Serial.println("No obstacle, resuming forward");
      moveForward();
      wasObstacleDetected = false; // Reset the flag
    } else {
      if (middleSensor) { // Middle Sensor On Line
        if (!leftSensor && !rightSensor) { // LS and RS not on line
          Serial.println("Move Forward");
          moveForward();
        } else if (leftSensor && !rightSensor) { // Sharp Left
          Serial.println("Sharp Left");
          sharpLeft();
        } else if (!leftSensor && rightSensor) { // Sharp Right
          Serial.println("Sharp Right");
          sharpRight();
        } else if (leftSensor && rightSensor) { // Stop
          stopMotors();
          Serial.println("Stop");
        }
      } else if (leftSensor && !middleSensor && !rightSensor) {
        shapeRight();
      } else if (!leftSensor && !middleSensor && rightSensor) {
        shapeLeft();
      }
    }
  }

  delay(5);
}

void moveForward() {
  analogWrite(enA, 90); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 90);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}

void sharpLeft() {
  analogWrite(enA, 200); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 200);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
  delay(1000);
}

void sharpRight() {
  analogWrite(enA, 200); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 200);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  delay(1000);
}

void shapeRight() {
  analogWrite(enA, 100); // Reduce speed for turning
  analogWrite(enB, 100); // Reduce speed for turning
  digitalWrite(LM1, LOW); // Left Motor backward Pin 
  digitalWrite(LM2, HIGH); // Left Motor forward Pin 
  digitalWrite(RM1, LOW); // Right Motor forward Pin 
  digitalWrite(RM2, HIGH); // Right Motor backward Pin 
}

void shapeLeft() {
  analogWrite(enA, 100); // Reduce speed for turning
  analogWrite(enB, 100); // Reduce speed for turning
  digitalWrite(LM1, HIGH); // Left Motor backward Pin 
  digitalWrite(LM2, LOW); // Left Motor forward Pin 
  digitalWrite(RM1, HIGH); // Right Motor forward Pin 
  digitalWrite(RM2, LOW); // Right Motor backward Pin 
}

void stopMotors() {
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
}

int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Convert duration to distance
  return distance;
}
