#include <Servo.h>
#include "DeviceDriverSet_xxx0.h"

// Define IO pin
#define PWMA 5       // Controls power to right motor
#define PWMB 6       // Controls power to left motor
#define AIN 7        // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN 8        // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY 3       // Place H-Bridge in standby if LOW, Run if HIGH
#define modeSwitch 2 // Mode Switch input

// Define Ultrasonic Sensor pins
#define TRIG_PIN 13  // Trigger pin of the ultrasonic sensor
#define ECHO_PIN 12  // Echo pin of the ultrasonic sensor

// Line sensor pins
#define RIGHT_SENSOR A0
#define MIDDLE_SENSOR A1
#define LEFT_SENSOR A2

// Threshold for white line detection
#define WHITE_THRESHOLD 35  // Values less than or equal to 35 indicate white line

// Define maximum distance to detect (in cm)
#define MAX_DISTANCE 80      // Maximum detection range
#define DETECTION_THRESHOLD 80  // Distance to trigger detection response

// Motor speeds
#define NORMAL_SPEED 50     // Speed for normal movement
#define SCAN_MOTOR_SPEED 130 // Slower speed for scanning turns
#define TURN_SPEED 170       // Speed for turning the car to face object
#define CHARGE_SPEED 255     // Maximum speed for charging at opponent

// Scan parameters
#define SCAN_DURATION 100    // How long to scan in each direction (ms)
#define SCAN_DELAY 20        // Delay between scanning actions (ms)
#define TURN_DURATION 120    // Duration to turn the car (adjust based on testing)

// Direction constants
#define DIR_NONE 0
#define DIR_LEFT 1
#define DIR_RIGHT 2

// Define servo positions for scanning
#define SERVO_CENTER 90
#define SERVO_SLIGHT_LEFT 135  // New position between center and left
#define SERVO_LEFT 178 //160 original
#define SERVO_SLIGHT_RIGHT 45  // New position between center and right
#define SERVO_RIGHT 2 //20 orifinal

// State machine states
#define STATE_NORMAL_DRIVE 0
#define STATE_SCANNING 1
#define STATE_TRACKING 2
#define STATE_BOUNDARY_DETECTED 3

bool completedFullScan = false;
bool initialStart = false;

// Create servo object
DeviceDriverSet_Servo myServo;

// Track current servo position
int currentServoPosition = SERVO_CENTER;

// Current state of the robot
int currentState = STATE_NORMAL_DRIVE;

// Function prototypes
void trackObject(long distance = -1);
bool whiteLineDetected();

// Function to display sensor values (for debugging)
void displaySensorValues() {
  int rightValue = analogRead(RIGHT_SENSOR);
  int middleValue = analogRead(MIDDLE_SENSOR);
  int leftValue = analogRead(LEFT_SENSOR);
  
  Serial.print("Line Sensors - Right: ");
  Serial.print(rightValue);
  Serial.print("  Middle: ");
  Serial.print(middleValue);
  Serial.print("  Left: ");
  Serial.println(leftValue);
  
  long distance = getDistance();
  Serial.print("Ultrasonic Distance: ");
  if (distance > 0) {
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("No object detected");
  }
}

// Function to check if any sensor detects a white line
bool whiteLineDetected() {
  int rightValue = analogRead(RIGHT_SENSOR);
  int middleValue = analogRead(MIDDLE_SENSOR);
  int leftValue = analogRead(LEFT_SENSOR);
  
  // Check if any sensor value is less than or equal to WHITE_THRESHOLD
  if (rightValue <= WHITE_THRESHOLD || middleValue <= WHITE_THRESHOLD || leftValue <= WHITE_THRESHOLD) {
    return true;
  }
  return false;
}

// Function for hard braking
void hardBrake() {
  // Reverse motor direction briefly to create active braking
  digitalWrite(AIN, LOW);  // Reverse direction on Right
  digitalWrite(BIN, LOW);  // Reverse direction on Left
  analogWrite(PWMA, 255);  // Full power
  analogWrite(PWMB, 255);  // Full power
  delay(300);              // Brief reverse pulse (shorter for battle responsiveness)
  
  // Then stop motors completely
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);


}

// Function to turn around
void turnAround() {
  Serial.println("Performing 180-degree turn...");
  
  // Set one motor forward, one backward to spin in place
  digitalWrite(AIN, HIGH);  // Right motor forward
  digitalWrite(BIN, LOW);   // Left motor backward
  
  // Apply power to both motors
  analogWrite(PWMA, 200);  // Right motor power
  analogWrite(PWMB, 200);  // Left motor power
  
  // Use millis() instead of delay
  unsigned long turnStartTime = millis();
  while (millis() - turnStartTime < 450) {
    // This loop will exit after 450ms
    // You could add a very short delay here to avoid CPU overload
    delay(1);
  }
  
  // Stop motors
  digitalWrite(PWMA, 0);
  digitalWrite(PWMB, 0);
  
  // Short pause after turning
  unsigned long pauseStartTime = millis();
  while (millis() - pauseStartTime < 50) {
    // Short pause after turning
    delay(1);
  }
  
  Serial.println("Turn complete");
}

// Function to align the car with the detected object
void alignCarWithObject(int direction, bool slightTurn) {
  Serial.print("Aligning car with opponent on the ");
  
  // Calculate turn duration based on whether it's a slight or extreme turn
  int turnDuration;
  if (slightTurn) {
    turnDuration = TURN_DURATION;  // Normal duration for slight turns
  } else {
    turnDuration = TURN_DURATION * 1.75;  // Longer duration for extreme turns
  }
  
  if (direction == DIR_LEFT) {
    Serial.println(slightTurn ? "slight left..." : "extreme left...");
    
    // Rotate the car to the left
    rotateLeft(TURN_SPEED);
  } else if (direction == DIR_RIGHT) {
    Serial.println(slightTurn ? "slight right..." : "extreme right...");
    
    // Rotate the car to the right
    rotateRight(TURN_SPEED);
  }
  
  // Use millis() instead of delay
  unsigned long turnStartTime = millis();
  while (millis() - turnStartTime < turnDuration) {
    // This loop will exit after turnDuration milliseconds
    // You could add a very short delay here to avoid CPU overload
    delay(1);
  }
  
  stopMotors();
  
  // After turning, center the camera
  moveServo(SERVO_CENTER);
  Serial.println("Alignment complete. Car is now facing the opponent.");
}

// Function to move servo and update current position
void moveServo(int angle) {
  myServo.DeviceDriverSet_Servo_control(angle);
  currentServoPosition = angle;
  
  // Use millis() instead of delay
  unsigned long servoMoveTime = millis();
  while (millis() - servoMoveTime < 15) { // Reduced from 30ms
    // This loop will exit after 15ms
    delay(1);
  }
}

// Function to track object once it's detected
void trackObject(long distance = -1) {
  Serial.println("Tracking and charging at opponent");
  
  // Make sure we have a valid distance
  if (distance <= 0) {
    distance = getDistance();
  }
  
  unsigned long trackingStartTime = millis();
  unsigned long lastDetectionTime = trackingStartTime;
  unsigned long lastCheckTime = 0; // New variable for timing checks
  bool objectLost = false;
  
  while (!objectLost) {
    unsigned long currentMillis = millis();
    
    // Check for white line and distance update every 20ms instead of using delay
    if (currentMillis - lastCheckTime >= 20) {
      lastCheckTime = currentMillis;
      
      // Check for white boundary line
      if (whiteLineDetected()) {
        Serial.println("WHITE LINE DETECTED during tracking! Stopping charge!");
        hardBrake();
        turnAround();
        return; // Exit tracking mode immediately
      }
      
      // Get current distance
      distance = getDistance();
      
      // If object is still detected
      if (distance > 0 && distance <= DETECTION_THRESHOLD) {
        lastDetectionTime = currentMillis; // Update last detection time
        
        // Determine speed based on distance for a dynamic charge
        int speedToUse;
        if (distance <= 30) {
          speedToUse = CHARGE_SPEED; // Full speed for close targets
          Serial.println("Target close! Full charge speed!");
        } else if (distance <= 60) {
          speedToUse = 200; // Medium-high speed for medium distance
          Serial.println("Approaching target at high speed");
        } else {
          speedToUse = 170; // Medium speed for distant targets
          Serial.println("Moving toward distant target");
        }
        
        // Charge at the opponent with the appropriate speed
        moveForward(speedToUse);
      }
      // Object no longer detected
      else {
        // Check if object has been lost for more than 500ms
        if (currentMillis - lastDetectionTime > 500) {
          Serial.println("Opponent lost. Returning to scanning mode.");
          stopMotors();
          objectLost = true;
        }
      }
      
      // Check if we've been tracking for too long (5 seconds max)
      if (currentMillis - trackingStartTime > 5000) {
        Serial.println("Tracking timeout. Returning to scanning mode.");
        stopMotors();
        objectLost = true;
      }
    }
    
    // No delay here - the loop will run as fast as possible
    // and timing is controlled by the if statement above
  }
}

// Function to rotate left
void rotateLeft(int speed) {
  // Turn left: Left motor backward, Right motor forward
  digitalWrite(AIN, HIGH);  // Right motor forward
  digitalWrite(BIN, LOW);   // Left motor backward
  
  // Set motor speeds
  analogWrite(PWMA, speed);  // Right motor
  analogWrite(PWMB, speed);  // Left motor
}

// Function to rotate right
void rotateRight(int speed) {
  // Turn right: Left motor forward, Right motor backward
  digitalWrite(AIN, LOW);   // Right motor backward
  digitalWrite(BIN, HIGH);  // Left motor forward
  
  // Set motor speeds
  analogWrite(PWMA, speed);  // Right motor
  analogWrite(PWMB, speed);  // Left motor
}

// Function to stop motors
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Function to move forward
void moveForward(int speed) {
  // Both motors forward
  digitalWrite(AIN, HIGH);  // Right motor forward
  digitalWrite(BIN, HIGH);  // Left motor forward
  
  // Set motor speeds
  analogWrite(PWMA, speed);  // Right motor
  analogWrite(PWMB, speed);  // Left motor
}

// Function to get distance from ultrasonic sensor
long getDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin
  long duration = pulseIn(ECHO_PIN, HIGH, 8000); // Reduced timeout for faster response
  
  // Calculate the distance
  long distance = duration / 58;  // Using the standard formula
  
  // Check if the distance is valid
  if (distance == 0 || distance > MAX_DISTANCE) {
    return -1; // Invalid measurement
  }
  
  return distance;
}

void setup() {
  // Motor control pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Line sensor pins
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MIDDLE_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  
  // Mode switch input
  pinMode(modeSwitch, INPUT);
  
  // Enable motors
  digitalWrite(STBY, HIGH);
  
  // Initially motors off
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  
  // Initialize servo at center position
  myServo.DeviceDriverSet_Servo_Init(SERVO_CENTER);
  currentServoPosition = SERVO_CENTER;
  
  // Begin serial communication
  Serial.begin(9600);
  Serial.println("Stationary Sumo Battle Robot - Ready");
}

void loop() {

  if (!initialStart) {
    delay(2000);
    initialStart = true;
  }
  
  // Main battle loop - never exits until power off or reset
  battleMode();
}

void start1() {
  moveForward(210);
  delay(700);
  
  // Set one motor forward, one backward to spin in place
  digitalWrite(AIN, HIGH);  // Right motor forward
  digitalWrite(BIN, LOW);   // Left motor backward
  
  // Apply power to both motors
  analogWrite(PWMA, 200);  // Right motor power
  analogWrite(PWMB, 200);  // Left motor power
  
  // Use millis() instead of delay
  unsigned long turnStartTime = millis();
  while (millis() - turnStartTime < 375) {
    // This loop will exit after 450ms
    // You could add a very short delay here to avoid CPU overload
    delay(1);
  }
  
  // Stop motors
  digitalWrite(PWMA, 0);
  digitalWrite(PWMB, 0);
  
  // Short pause after turning
  unsigned long pauseStartTime = millis();
  while (millis() - pauseStartTime < 50) {
    // Short pause after turning
    delay(1);
  }
  
  Serial.println("Turn complete");
}

void start2 () {
  rotateLeft(150);
  delay(210);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(1);
}

void start3 () {
  rotateLeft(150);
  delay(250);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(1);
  moveForward(210);
  delay(500);
  // Set one motor forward, one backward to spin in place
  digitalWrite(AIN, LOW);  // Right motor forward
  digitalWrite(BIN, HIGH);   // Left motor backward
  
  // Apply power to both motors
  analogWrite(PWMB, 200);  // Right motor power
  analogWrite(PWMA, 200);  // Left motor power
  
  // Use millis() instead of delay
  unsigned long turnStartTime = millis();
  while (millis() - turnStartTime < 410) {
    // This loop will exit after 450ms
    // You could add a very short delay here to avoid CPU overload
    delay(1);
  }
  
  // Stop motors
  digitalWrite(PWMA, 0);
  digitalWrite(PWMB, 0);
  
  // Short pause after turning
  unsigned long pauseStartTime = millis();
  while (millis() - pauseStartTime < 50) {
    // Short pause after turning
    delay(1);
  }
  
  Serial.println("Turn complete");

}

void battleMode() {
  Serial.println("Beginning battle mode");
  Serial.println("----------------------------");
  
  // Initial state is normal driving
  currentState = STATE_NORMAL_DRIVE;
  
  while (true) {
    // Priority 1: Always check for white boundary line no matter what state we're in
    if (whiteLineDetected()) {
      Serial.println("WHITE LINE DETECTED! Boundary avoidance activated!");
      currentState = STATE_BOUNDARY_DETECTED;
      
      // Emergency stop
      hardBrake();
    }
    
    // Handle different states
    switch (currentState) {
      case STATE_NORMAL_DRIVE:
        // While stationary, scan for opponents
        scanForOpponent();
        break;
        
      case STATE_TRACKING:
        // Already tracking an opponent, handled in the trackObject function
        trackObject();
        // After tracking completes (or opponent lost), resume normal drive
        currentState = STATE_NORMAL_DRIVE;
        break;

      case STATE_BOUNDARY_DETECTED:
        // Handle boundary detection in the state machine
        turnAround();
        
        // After turning around, resume normal drive state
        currentState = STATE_NORMAL_DRIVE;
        break;
    }
    
    // Small delay to avoid taxing the processor
    delay(10);
  }
}

// Function to scan for opponents while stationary


// Modify scanForOpponent() function
void scanForOpponent() {
  // Each time this is called, we'll check one position
  // Cycle through positions: CENTER -> SLIGHT_LEFT -> SLIGHT_RIGHT -> LEFT -> RIGHT

  if (whiteLineDetected()) {
    Serial.println("WHITE LINE DETECTED during scanning!");
    currentState = STATE_BOUNDARY_DETECTED;
    return;
  }

  static int scanPosition = 0; // Keep track of which position we're checking
  long distance;
  
  // Move servo to the current scan position
  switch (scanPosition) {
    case 0: // CENTER
      moveServo(SERVO_CENTER);
      break;
    case 1: // SLIGHT_LEFT
      moveServo(SERVO_SLIGHT_LEFT);
      break;
    case 2: // SLIGHT_RIGHT
      moveServo(SERVO_SLIGHT_RIGHT);
      break;
    case 3: // LEFT
      moveServo(SERVO_LEFT);
      break;
    case 4: // RIGHT
      moveServo(SERVO_RIGHT);
      break;
  }
  
  // Measure distance at current position
  distance = getDistance();
  
  // If opponent detected
  if (distance > 0 && distance <= DETECTION_THRESHOLD) {
    Serial.print("Opponent detected! Position: ");
    Serial.print(scanPosition);
    Serial.print(", Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Reset scan tracking
    completedFullScan = false;
    
    // Stop for alignment
    stopMotors();
    
    // Align with opponent based on where it was detected
    switch (scanPosition) {
      case 0: // CENTER - no alignment needed
        break;
        
      case 1: // SLIGHT_LEFT
        alignCarWithObject(DIR_LEFT, true);
        break;
        
      case 2: // SLIGHT_RIGHT
        alignCarWithObject(DIR_RIGHT, true);
        break;
        
      case 3: // LEFT
        alignCarWithObject(DIR_LEFT, false);
        break;
        
      case 4: // RIGHT
        alignCarWithObject(DIR_RIGHT, false);
        break;
    }
    
    // Track and charge at the opponent
    currentState = STATE_TRACKING;
    return;
  }
  
  // Move to next scan position
  scanPosition = (scanPosition + 1) % 5;
  
  // Check if we've completed a full scan sequence
  if (scanPosition == 0) {
    completedFullScan = true;
  }
  
  // If we've completed a full scan without detecting an opponent, spin
  if (completedFullScan) {
    Serial.println("No opponent detected. Spinning 180 degrees.");
    turnAround();
    completedFullScan = false; // Reset for next scan cycle
  }
}