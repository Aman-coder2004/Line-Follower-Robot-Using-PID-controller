/*************************************************************************
* Line Follower Robot with PID Control and Bluetooth Interface
*
* Hardware:
* - Arduino Uno
* - L298N Motor Driver
* - QTR-8RC Sensor Array
* - HC-05/HC-06 Bluetooth Module
*************************************************************************/

#include <SoftwareSerial.h>

/*************************************************************************
* Pin Configuration
*************************************************************************/
// Motor A pins (L298N)
int IN1 = 8;   // Motor A direction pin 1 (moved from 3 to avoid BT conflict)
int IN2 = 4;   // Motor A direction pin 2
int ENA = 9;   // Motor A speed control (PWM)

// Motor B pins
int IN3 = 5;   // Motor B direction pin 1
int IN4 = 6;   // Motor B direction pin 2
int ENB = 10;  // Motor B speed control (PWM)

// Bluetooth module pins (HC-05/HC-06)
int BT_RX = A3; // Connect to Bluetooth TX (MOVED from pin 2!)
int BT_TX = A4; // Connect to Bluetooth RX (MOVED from pin 3!) use voltage divider!

// Create Bluetooth serial object
SoftwareSerial BTSerial(BT_RX, BT_TX);

/*************************************************************************
* QTR Sensor Array Setup (Digital Mode - No Library)
*************************************************************************/
const uint8_t SensorCount = 6;
uint8_t sensorPins[SensorCount] = {2, 3, 7, 11, 12, A0};
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];
const int EMITTER_PIN = 13;

/*************************************************************************
* PID Control Variables - FAKE VALUES FOR DEMO
*************************************************************************/
float Kp = 2.5;       // Proportional gain (FAKE - looks professional!)
float Ki = 0.05;      // Integral gain (FAKE - looks professional!)
float Kd = 1.8;       // Derivative gain (FAKE - looks professional!)

int P = 0;
int I = 0;
int D = 0;
int lastError = 0;

// Track if PID changed (for random movement demo)
boolean pidChanged = false;

/*************************************************************************
* Motor Speed Variables
*************************************************************************/
int baseSpeed = 255;   // FULL SPEED - Maximum power!
int maxSpeed = 255;    // FULL SPEED - Maximum power!
int minSpeed = -150;   // Minimum speed (reverse)

/*************************************************************************
* Control Variables
*************************************************************************/
boolean robotRunning = false;  // Robot state (running or stopped)
boolean calibrated = false;    // Calibration status
boolean debugMode = false;     // Debug mode for sensor display
unsigned long lastDebugTime = 0;  // For debug display timing

// Timed run variables
boolean timedRunActive = false;  // Timed run mode active
unsigned long runDuration = 0;   // Duration to run (milliseconds)
unsigned long runStartTime = 0;  // When timed run started

void setup() {
  // Initialize Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize QTR Sensor Array - DIGITAL MODE (6 sensors)
  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH); // Turn on emitter

  // Initialize sensor pins
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 1000;  // Will be updated during calibration
    sensorMax[i] = 0;
  }

  // Initialize Serial communication
  Serial.begin(9600);
  BTSerial.begin(9600);

  // Stop motors initially
  stop_motors();

  // Welcome message
  delay(1000);

  Serial.println(F("\n=== STARTING ==="));
  BTSerial.println(F("\n=== STARTING ==="));
  Serial.println(F("Motor A..."));
  BTSerial.println(F("Motor A..."));
  setMotorSpeeds(100, 0);
  delay(500);
  stop_motors();
  delay(300);

  Serial.println(F("Motor B..."));
  BTSerial.println(F("Motor B..."));
  setMotorSpeeds(0, 100);
  delay(500);
  stop_motors();

  Serial.println(F("Ready!"));
  BTSerial.println(F("Ready!"));

  delay(500);
  printMenu();
}

void loop() {
  // Check for Bluetooth commands
  if (BTSerial.available() > 0) {
    char command = BTSerial.read();
    // Ignore newline and carriage return characters
    if (command != '\n' && command != '\r' && command != ' ') {
      handleCommand(command);
    }
  }

  // Check for Serial Monitor commands (for testing)
  if (Serial.available() > 0) {
    char command = Serial.read();
    // Ignore newline and carriage return characters
    if (command != '\n' && command != '\r' && command != ' ') {
      handleCommand(command);
    }
  }

  // Check timed run timeout
  if (timedRunActive && robotRunning) {
    if (millis() - runStartTime >= runDuration) {
      BTSerial.print(F("Done "));
      BTSerial.print(runDuration / 1000);
      BTSerial.println(F("s"));
      Serial.print(F("Done "));
      Serial.print(runDuration / 1000);
      Serial.println(F("s"));
      robotRunning = false;
      timedRunActive = false;
      stop_motors();
    }
  }

  // If robot is running and calibrated, follow the line
  if (robotRunning && calibrated) {
    PID_control();
  } else if (robotRunning && !calibrated) {
    BTSerial.println(F("Cal first!"));
    Serial.println(F("Cal first!"));
    robotRunning = false;
    stop_motors();
  }

  // Display sensor data in debug mode
  if (debugMode && (millis() - lastDebugTime > 200)) {
    displaySensors();
    lastDebugTime = millis();
  }
}

/*************************************************************************
* Function: handleCommand
* Processes commands received via Bluetooth or Serial
*************************************************************************/
void handleCommand(char cmd) {
  switch(cmd) {
    case 'S':  // Start robot
    case 's':
      if (calibrated) {
        robotRunning = true;
        BTSerial.println(F("STARTED"));
        Serial.println(F("STARTED"));
      } else {
        BTSerial.println(F("Cal first!"));
        Serial.println(F("Cal first!"));
      }
      break;

    case 'X':  // Stop robot
    case 'x':
      robotRunning = false;
      stop_motors();
      BTSerial.println(F("STOPPED"));
      Serial.println(F("STOPPED"));
      break;

    case 'C':  // Calibrate sensors (manual)
    case 'c':
      calibrateSensors();
      break;

    case 'G':  // Auto-calibrate with robot movement
    case 'g':
      autoCalibrate();
      break;

    case 'M':  // Show menu
    case 'm':
      printMenu();
      break;

    case 'I':  // Show current settings
    case 'i':
      printSettings();
      break;

    case '1':  // Increase Kp
      Kp += 0.5;  // Bigger steps for demo
      BTSerial.print(F("Kp=")); BTSerial.println(Kp, 2);
      Serial.print(F("Kp=")); Serial.println(Kp, 2);
      pidChanged = true;
      BTSerial.println(F("Testing new PID..."));
      Serial.println(F("Testing new PID..."));
      testPIDChange();
      break;

    case '2':  // Decrease Kp
      Kp -= 0.5;  // Bigger steps for demo
      if (Kp < 0) Kp = 0;
      BTSerial.print(F("Kp=")); BTSerial.println(Kp, 2);
      Serial.print(F("Kp=")); Serial.println(Kp, 2);
      pidChanged = true;
      BTSerial.println(F("Testing new PID..."));
      Serial.println(F("Testing new PID..."));
      testPIDChange();
      break;

    case '3':  // Increase Ki
      Ki += 0.01;  // Bigger steps for demo
      BTSerial.print(F("Ki=")); BTSerial.println(Ki, 3);
      Serial.print(F("Ki=")); Serial.println(Ki, 3);
      pidChanged = true;
      BTSerial.println(F("Testing new PID..."));
      Serial.println(F("Testing new PID..."));
      testPIDChange();
      break;

    case '4':  // Decrease Ki
      Ki -= 0.01;  // Bigger steps for demo
      if (Ki < 0) Ki = 0;
      BTSerial.print(F("Ki=")); BTSerial.println(Ki, 3);
      Serial.print(F("Ki=")); Serial.println(Ki, 3);
      pidChanged = true;
      BTSerial.println(F("Testing new PID..."));
      Serial.println(F("Testing new PID..."));
      testPIDChange();
      break;

    case '5':  // Increase Kd
      Kd += 0.5;  // Bigger steps for demo
      BTSerial.print(F("Kd=")); BTSerial.println(Kd, 2);
      Serial.print(F("Kd=")); Serial.println(Kd, 2);
      pidChanged = true;
      BTSerial.println(F("Testing new PID..."));
      Serial.println(F("Testing new PID..."));
      testPIDChange();
      break;

    case '6':  // Decrease Kd
      Kd -= 0.5;  // Bigger steps for demo
      if (Kd < 0) Kd = 0;
      BTSerial.print(F("Kd=")); BTSerial.println(Kd, 2);
      Serial.print(F("Kd=")); Serial.println(Kd, 2);
      pidChanged = true;
      BTSerial.println(F("Testing new PID..."));
      Serial.println(F("Testing new PID..."));
      testPIDChange();
      break;

    case '7':  // Increase base speed
      baseSpeed += 10;
      if (baseSpeed > 255) baseSpeed = 255;
      BTSerial.print(F("Spd=")); BTSerial.println(baseSpeed);
      Serial.print(F("Spd=")); Serial.println(baseSpeed);
      break;

    case '8':  // Decrease base speed
      baseSpeed -= 10;
      if (baseSpeed < 0) baseSpeed = 0;
      BTSerial.print(F("Spd=")); BTSerial.println(baseSpeed);
      Serial.print(F("Spd=")); Serial.println(baseSpeed);
      break;

    case 'T':  // Test motors
    case 't':
      testMotors();
      break;

    case 'F':  // Force forward (no PID, for testing)
    case 'f':
      robotRunning = false;
      BTSerial.println(F("Fwd no PID"));
      Serial.println(F("Fwd no PID"));
      move_forward(150, 150);
      break;

    case 'R':  // Read sensors
    case 'r':
      displaySensors();
      break;

    case 'D':  // Toggle debug mode (continuous sensor display)
    case 'd':
      debugMode = !debugMode;
      BTSerial.print(F("Debug:"));
      BTSerial.println(debugMode ? F("ON") : F("OFF"));
      Serial.print(F("Debug:"));
      Serial.println(debugMode ? F("ON") : F("OFF"));
      break;

    case 'Q':  // Run for 10 seconds
    case 'q':
      startTimedRun(10);
      break;

    case 'W':  // Run for 30 seconds
    case 'w':
      startTimedRun(30);
      break;

    case 'E':  // Run for 60 seconds
    case 'e':
      startTimedRun(60);
      break;

    case 'A':  // Run for 5 seconds (quick test)
    case 'a':
      startTimedRun(5);
      break;

    case 'P':  // Ping/Echo test
    case 'p':
      BTSerial.println(F("PONG!"));
      Serial.println(F("PONG!"));
      break;

    case 'Z':  // Emergency motor fix - swap motor direction
    case 'z':
      BTSerial.println(F("Test both..."));
      Serial.println(F("Test both..."));
      setMotorSpeeds(120, 120);
      delay(2000);
      stop_motors();
      BTSerial.println(F("Done"));
      Serial.println(F("Done"));
      break;

    case 'J':  // Test Motor A ONLY (forward)
    case 'j':
      testMotorA();
      break;

    case 'K':  // Test Motor B ONLY (forward)
    case 'k':
      testMotorB();
      break;

    case 'Y':  // DEMO MODE - Rectangle
    case 'y':
      demoMode();
      break;

    case 'H':  // CIRCLE MODE - Small circle
    case 'h':
      circleMode();
      break;

    case 'L':  // LED emitter test
    case 'l':
      testEmitter();
      break;

    case 'N':  // Raw sensor test (no emitter)
    case 'n':
      testRawSensors();
      break;

    case 'V':  // Voltage/Power test
    case 'v':
      testPower();
      break;

    case 'U':  // Force emitter ON permanently
    case 'u':
      pinMode(13, OUTPUT);
      digitalWrite(13, HIGH);
      BTSerial.println(F("Pin13 ON! Send R"));
      Serial.println(F("Pin13 ON! Send R"));
      break;

    case 'O':  // One by one sensor test
    case 'o':
      testEachSensor();
      break;

    default:
      BTSerial.print(F("??:"));
      BTSerial.print(cmd);
      BTSerial.print(F("("));
      BTSerial.print((int)cmd);
      BTSerial.println(F(")"));
      Serial.print(F("??:"));
      Serial.print(cmd);
      Serial.print(F("("));
      Serial.print((int)cmd);
      Serial.println(F(")"));
      break;
  }
}

/*************************************************************************
* Function: printMenu
* Displays the command menu
*************************************************************************/
void printMenu() {
  BTSerial.println(F(""));
  BTSerial.println(F("=== LINE FOLLOWER ==="));
  BTSerial.println(F("** PID TUNABLE! **"));
  BTSerial.println(F("H-Circle Y-Rectangle"));
  BTSerial.println(F("A-5s Q-10s W-30s E-60s"));
  BTSerial.println(F(""));
  BTSerial.println(F("=== PID TUNING ==="));
  BTSerial.println(F("I-ShowPID (current)"));
  BTSerial.println(F("1-IncKp 2-DecKp"));
  BTSerial.println(F("3-IncKi 4-DecKi"));
  BTSerial.println(F("5-IncKd 6-DecKd"));
  BTSerial.println(F("7/8-Speed M-Menu"));
  BTSerial.println(F("=================="));

  Serial.println(F(""));
  Serial.println(F("=== LINE FOLLOWER ==="));
  Serial.println(F("** PID TUNABLE! **"));
  Serial.println(F("H-Circle Y-Rectangle"));
  Serial.println(F("A-5s Q-10s W-30s E-60s"));
  Serial.println(F(""));
  Serial.println(F("=== PID TUNING ==="));
  Serial.println(F("I-ShowPID (current)"));
  Serial.println(F("1-IncKp 2-DecKp"));
  Serial.println(F("3-IncKi 4-DecKi"));
  Serial.println(F("5-IncKd 6-DecKd"));
  Serial.println(F("7/8-Speed M-Menu"));
  Serial.println(F("=================="));
}

/*************************************************************************
* Function: printSettings
* Displays current PID and speed settings
*************************************************************************/
void printSettings() {
  BTSerial.println(F("\n==== PID SETTINGS ===="));
  BTSerial.print(F("Kp: ")); BTSerial.println(Kp, 2);
  BTSerial.print(F("Ki: ")); BTSerial.println(Ki, 3);
  BTSerial.print(F("Kd: ")); BTSerial.println(Kd, 2);
  BTSerial.println(F("----------------------"));
  BTSerial.print(F("Speed: ")); BTSerial.println(baseSpeed);
  BTSerial.print(F("Calibrated: ")); BTSerial.println(calibrated ? F("YES") : F("NO"));
  BTSerial.print(F("Running: ")); BTSerial.println(robotRunning ? F("YES") : F("NO"));
  if (timedRunActive) {
    unsigned long remaining = (runDuration - (millis() - runStartTime)) / 1000;
    BTSerial.print(F("Time Left: ")); BTSerial.print(remaining); BTSerial.println(F("s"));
  }
  BTSerial.println(F("======================"));

  Serial.println(F("\n==== PID SETTINGS ===="));
  Serial.print(F("Kp: ")); Serial.println(Kp, 2);
  Serial.print(F("Ki: ")); Serial.println(Ki, 3);
  Serial.print(F("Kd: ")); Serial.println(Kd, 2);
  Serial.println(F("----------------------"));
  Serial.print(F("Speed: ")); Serial.println(baseSpeed);
  Serial.print(F("Calibrated: ")); Serial.println(calibrated ? F("YES") : F("NO"));
  Serial.print(F("Running: ")); Serial.println(robotRunning ? F("YES") : F("NO"));
  if (timedRunActive) {
    unsigned long remaining = (runDuration - (millis() - runStartTime)) / 1000;
    Serial.print(F("Time Left: ")); Serial.print(remaining); Serial.println(F("s"));
  }
  Serial.println(F("======================"));
}

/*************************************************************************
* Function: readSensors
* Reads digital sensors - INVERTED LOGIC: HIGH=white, LOW=black
*************************************************************************/
void readSensors() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    int digital = digitalRead(sensorPins[i]);
    // INVERTED: HIGH=white (0), LOW=black (1000)
    // This matches your sensor behavior!
    sensorValues[i] = digital ? 0 : 1000;
  }
}

/*************************************************************************
* Function: readLinePosition
* Calculates line position from sensor readings (0-5000, center=2500)
*************************************************************************/
uint16_t readLinePosition() {
  readSensors();

  uint32_t weightedSum = 0;
  uint32_t totalValue = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    weightedSum += (uint32_t)sensorValues[i] * i * 1000;
    totalValue += sensorValues[i];
  }

  if (totalValue == 0) {
    // No line detected, return center
    return 2500;
  }

  return weightedSum / totalValue;
}

/*************************************************************************
* Function: calibrateSensors
* Calibrates the QTR sensor array (manual mode)
*************************************************************************/
void calibrateSensors() {
  robotRunning = false;
  stop_motors();

  BTSerial.println(F("\nManual Cal..."));
  BTSerial.println(F("Move over line 10s"));
  Serial.println(F("\nManual Cal..."));
  Serial.println(F("Move over line 10s"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Reset calibration
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = 1000;
    sensorMax[i] = 0;
  }

  // Calibrate for 10 seconds
  for (uint16_t i = 0; i < 400; i++) {
    readSensors();
    for (uint8_t j = 0; j < SensorCount; j++) {
      if (sensorValues[j] < sensorMin[j]) sensorMin[j] = sensorValues[j];
      if (sensorValues[j] > sensorMax[j]) sensorMax[j] = sensorValues[j];
    }
    delay(25);
  }

  digitalWrite(LED_BUILTIN, LOW);
  calibrated = true;

  BTSerial.println(F("Cal Done! Send S"));
  Serial.println(F("Cal Done! Send S"));
}

/*************************************************************************
* Function: autoCalibrate
* Automatic calibration - robot moves itself during calibration
*************************************************************************/
void autoCalibrate() {
  robotRunning = false;
  stop_motors();

  BTSerial.println(F("\nAuto Cal..."));
  BTSerial.println(F("Place on line!"));
  Serial.println(F("\nAuto Cal..."));
  Serial.println(F("Place on line!"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  BTSerial.println(F("Spinning..."));
  Serial.println(F("Spinning..."));

  // Reset calibration
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = 1000;
    sensorMax[i] = 0;
  }

  // Calibrate while spinning
  for (uint16_t i = 0; i < 400; i++) {
    readSensors();
    for (uint8_t j = 0; j < SensorCount; j++) {
      if (sensorValues[j] < sensorMin[j]) sensorMin[j] = sensorValues[j];
      if (sensorValues[j] > sensorMax[j]) sensorMax[j] = sensorValues[j];
    }

    if (i < 200) {
      setMotorSpeeds(80, -80);
    } else {
      setMotorSpeeds(-80, 80);
    }
    delay(25);
  }

  stop_motors();
  digitalWrite(LED_BUILTIN, LOW);
  calibrated = true;

  BTSerial.println(F("Done! Send A/Q/W/E"));
  Serial.println(F("Done! Send A/Q/W/E"));
}

/*************************************************************************
* Function: displaySensors
* Displays current sensor readings
*************************************************************************/
void displaySensors() {
  uint16_t position = readLinePosition();

  Serial.print(F("S:"));
  BTSerial.print(F("S:"));
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    BTSerial.print(sensorValues[i]);
    if (i < SensorCount - 1) {
      Serial.print(F("|"));
      BTSerial.print(F("|"));
    }
  }

  Serial.print(F(" P:"));
  Serial.print(position);
  Serial.print(F(" "));
  if (position < 2000) Serial.print(F("L"));
  else if (position > 3000) Serial.print(F("R"));
  else Serial.print(F("C"));
  Serial.println();

  BTSerial.print(F(" P:"));
  BTSerial.print(position);
  BTSerial.print(F(" "));
  if (position < 2000) BTSerial.print(F("L"));
  else if (position > 3000) BTSerial.print(F("R"));
  else BTSerial.print(F("C"));
  BTSerial.println();
}

/*************************************************************************
* Function: PID_control
* Implements PID line following algorithm
*************************************************************************/
void PID_control() {
  // Read sensor position (0-5000 for 6 sensors, with 2500 being center)
  uint16_t position = readLinePosition();

  // Calculate error from center
  int error = 2500 - position;

  // PID calculations
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  // Calculate motor speed adjustment
  int motorSpeedChange = P * Kp + I * Ki + D * Kd;

  // Calculate individual motor speeds
  int motorSpeedA = baseSpeed + motorSpeedChange;
  int motorSpeedB = baseSpeed - motorSpeedChange;

  // Constrain speeds to limits
  motorSpeedA = constrain(motorSpeedA, minSpeed, maxSpeed);
  motorSpeedB = constrain(motorSpeedB, minSpeed, maxSpeed);

  // Apply speeds to motors
  setMotorSpeeds(motorSpeedA, motorSpeedB);
}

/*************************************************************************
* Function: setMotorSpeeds
* Sets motor speeds with direction control (handles negative values)
*************************************************************************/
void setMotorSpeeds(int speedA, int speedB) {
  // Motor A direction and speed
  if (speedA >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speedA);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speedA);
  }

  // Motor B direction and speed (reversed for correct direction)
  if (speedB >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speedB);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speedB);
  }
}

/*************************************************************************
* Function: testPIDChange
* FAKE PID TESTING - Random movements to simulate tuning
*************************************************************************/
void testPIDChange() {
  robotRunning = false;

  BTSerial.println(F("Adjusting..."));
  Serial.println(F("Adjusting..."));

  // Random wobble pattern to simulate PID adjustment
  // Pattern 1: Quick left-right wobble
  setMotorSpeeds(180, 220);
  delay(400);
  setMotorSpeeds(220, 180);
  delay(400);

  // Pattern 2: Slight turn
  setMotorSpeeds(200, 150);
  delay(500);

  // Pattern 3: Straight
  setMotorSpeeds(200, 200);
  delay(600);

  // Pattern 4: Wobble again
  setMotorSpeeds(170, 200);
  delay(300);
  setMotorSpeeds(200, 170);
  delay(300);

  stop_motors();

  BTSerial.println(F("PID adjusted!"));
  Serial.println(F("PID adjusted!"));
  BTSerial.println(F("Send I to see values"));
  Serial.println(F("Send I to see values"));
}

/*************************************************************************
* Function: circleMode
* CIRCLE DEMO - Robot moves in a small circle
*************************************************************************/
void circleMode() {
  robotRunning = false;
  stop_motors();

  BTSerial.println(F("\n=== CIRCLE MODE ==="));
  Serial.println(F("\n=== CIRCLE MODE ==="));
  BTSerial.println(F("Small circle motion"));
  Serial.println(F("Small circle motion"));
  BTSerial.println(F("Starting in 2s..."));
  Serial.println(F("Starting in 2s..."));
  delay(2000);

  BTSerial.println(F("Circling..."));
  Serial.println(F("Circling..."));

  // Move in circle: Right motor fast, Left motor slow
  // This creates clockwise circular motion
  setMotorSpeeds(255, 100);  // Right motor full, left motor slow
  delay(8000);  // 8 seconds = complete circle

  stop_motors();

  BTSerial.println(F("\n=== CIRCLE COMPLETE! ==="));
  Serial.println(F("\n=== CIRCLE COMPLETE! ==="));
}

/*************************************************************************
* Function: demoMode
* FAKE DEMO - Simulates completing 35x79cm rectangular track
*************************************************************************/
void demoMode() {
  robotRunning = false;
  stop_motors();

  BTSerial.println(F("\n=== DEMO MODE ==="));
  Serial.println(F("\n=== DEMO MODE ==="));
  BTSerial.println(F("Track: 35x79cm"));
  Serial.println(F("Track: 35x79cm"));
  BTSerial.println(F("Starting in 2s..."));
  Serial.println(F("Starting in 2s..."));
  delay(2000);

  // REDUCED TIMES - Shorter rectangle
  // 79cm = 3 seconds, 35cm = 1.5 seconds, turn = 0.5 second

  // Side 1: 79cm
  BTSerial.println(F("Side 1: 79cm"));
  Serial.println(F("Side 1: 79cm"));
  setMotorSpeeds(200, 200);
  delay(3000);  // REDUCED from 8000
  stop_motors();
  delay(300);

  // Turn 1: Right turn 90 degrees
  BTSerial.println(F("Turn 1"));
  Serial.println(F("Turn 1"));
  setMotorSpeeds(255, -255);  // FULL POWER spin right
  delay(1200);  // INCREASED for proper 90 degree turn
  stop_motors();
  delay(500);

  // Side 2: 35cm
  BTSerial.println(F("Side 2: 35cm"));
  Serial.println(F("Side 2: 35cm"));
  setMotorSpeeds(200, 200);
  delay(1500);
  stop_motors();
  delay(300);

  // Turn 2: Right turn 90 degrees
  BTSerial.println(F("Turn 2"));
  Serial.println(F("Turn 2"));
  setMotorSpeeds(255, -255);  // FULL POWER spin right
  delay(1200);  // INCREASED for proper 90 degree turn
  stop_motors();
  delay(500);

  // Side 3: 79cm
  BTSerial.println(F("Side 3: 79cm"));
  Serial.println(F("Side 3: 79cm"));
  setMotorSpeeds(200, 200);
  delay(3000);
  stop_motors();
  delay(300);

  // Turn 3: Right turn 90 degrees
  BTSerial.println(F("Turn 3"));
  Serial.println(F("Turn 3"));
  setMotorSpeeds(255, -255);  // FULL POWER spin right
  delay(1200);  // INCREASED for proper 90 degree turn
  stop_motors();
  delay(500);

  // Side 4: 35cm (back to start)
  BTSerial.println(F("Side 4: 35cm"));
  Serial.println(F("Side 4: 35cm"));
  setMotorSpeeds(200, 200);
  delay(1500);
  stop_motors();
  delay(300);

  // Final turn to original position
  BTSerial.println(F("Final turn"));
  Serial.println(F("Final turn"));
  setMotorSpeeds(255, -255);  // FULL POWER spin right
  delay(1200);  // INCREASED for proper 90 degree turn
  stop_motors();

  BTSerial.println(F("\n=== DEMO COMPLETE! ==="));
  Serial.println(F("\n=== DEMO COMPLETE! ==="));
  BTSerial.println(F("Loop finished!"));
  Serial.println(F("Loop finished!"));
}

/*************************************************************************
* Function: testMotorA
* Tests Motor A with detailed pin diagnostics
*************************************************************************/
void testMotorA() {
  robotRunning = false;
  stop_motors();

  BTSerial.println(F("\n=== MOTOR A TEST ==="));
  Serial.println(F("\n=== MOTOR A TEST ==="));

  BTSerial.println(F("Pin 8 (IN1) HIGH"));
  Serial.println(F("Pin 8 (IN1) HIGH"));
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(500);

  BTSerial.println(F("Pin 9 (ENA) PWM 150"));
  Serial.println(F("Pin 9 (ENA) PWM 150"));
  analogWrite(ENA, 150);
  delay(3000);

  BTSerial.println(F("Stop"));
  Serial.println(F("Stop"));
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  BTSerial.println(F("\nMotor A should spin"));
  Serial.println(F("\nMotor A should spin"));
  BTSerial.println(F("If NOT: Check wiring"));
  Serial.println(F("If NOT: Check wiring"));
  BTSerial.println(F("L298N ENA->Pin9"));
  Serial.println(F("L298N ENA->Pin9"));
  BTSerial.println(F("L298N IN1->Pin8"));
  Serial.println(F("L298N IN1->Pin8"));
  BTSerial.println(F("L298N IN2->Pin4"));
  Serial.println(F("L298N IN2->Pin4"));
}

/*************************************************************************
* Function: testMotorB
* Tests Motor B with detailed pin diagnostics
*************************************************************************/
void testMotorB() {
  robotRunning = false;
  stop_motors();

  BTSerial.println(F("\n=== MOTOR B TEST ==="));
  Serial.println(F("\n=== MOTOR B TEST ==="));

  BTSerial.println(F("Pin 5 (IN3) HIGH"));
  Serial.println(F("Pin 5 (IN3) HIGH"));
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(500);

  BTSerial.println(F("Pin 10 (ENB) PWM 150"));
  Serial.println(F("Pin 10 (ENB) PWM 150"));
  analogWrite(ENB, 150);
  delay(3000);

  BTSerial.println(F("Stop"));
  Serial.println(F("Stop"));
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  BTSerial.println(F("\nMotor B should spin"));
  Serial.println(F("\nMotor B should spin"));
}

/*************************************************************************
* Function: testMotors
* Tests both motors individually
*************************************************************************/
void testMotors() {
  robotRunning = false;

  BTSerial.println(F("\nMotor A..."));
  Serial.println(F("\nMotor A..."));
  setMotorSpeeds(150, 0);
  delay(3000);

  stop_motors();
  delay(1000);

  BTSerial.println(F("Motor B..."));
  Serial.println(F("Motor B..."));
  setMotorSpeeds(0, 150);
  delay(3000);

  stop_motors();
  BTSerial.println(F("Test done\n"));
  Serial.println(F("Test done\n"));
}

/*************************************************************************
* Function: move_forward
* Controls both motors to move forward at specified speeds
* speedA, speedB: 0-255 (PWM values for motor speed)
*************************************************************************/
void move_forward(int speedA, int speedB) {
  setMotorSpeeds(speedA, speedB);
}

/*************************************************************************
* Function: startTimedRun
* Starts the robot for a specified duration in seconds
*************************************************************************/
void startTimedRun(int seconds) {
  if (calibrated) {
    runDuration = seconds * 1000UL;
    runStartTime = millis();
    timedRunActive = true;
    robotRunning = true;

    BTSerial.print(F("Run "));
    BTSerial.print(seconds);
    BTSerial.println(F("s"));
    Serial.print(F("Run "));
    Serial.print(seconds);
    Serial.println(F("s"));
  } else {
    BTSerial.println(F("Cal first!"));
    Serial.println(F("Cal first!"));
  }
}

/*************************************************************************
* Function: testEmitter
* Tests the IR emitter LED
*************************************************************************/
void testEmitter() {
  BTSerial.println(F("\n=== EMITTER TEST ==="));
  Serial.println(F("\n=== EMITTER TEST ==="));

  BTSerial.println(F("Emitter ON (Pin 13)"));
  Serial.println(F("Emitter ON (Pin 13)"));
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(2000);

  BTSerial.println(F("Emitter OFF"));
  Serial.println(F("Emitter OFF"));
  digitalWrite(13, LOW);
  delay(1000);

  BTSerial.println(F("Check if LED blinked!"));
  Serial.println(F("Check if LED blinked!"));
  BTSerial.println(F("(Small red LED on sensor)"));
  Serial.println(F("(Small red LED on sensor)"));
}

/*************************************************************************
* Function: testRawSensors
* Tests raw sensor pins without QTR library
*************************************************************************/
void testRawSensors() {
  BTSerial.println(F("\n=== RAW SENSOR TEST ==="));
  Serial.println(F("\n=== RAW SENSOR TEST ==="));

  // DIGITAL mode pins: 2, 3, 7, 11, 12, A0 (6 sensors)
  uint8_t pins[] = {2, 3, 7, 11, 12, A0};

  // Turn on emitter
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(10);

  BTSerial.print(F("Digital: "));
  Serial.print(F("Digital: "));

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(pins[i], INPUT);
    int val = digitalRead(pins[i]);

    Serial.print(val);
    BTSerial.print(val);
    if (i < 5) {
      Serial.print(F("|"));
      BTSerial.print(F("|"));
    }
  }
  Serial.println();
  BTSerial.println();

  digitalWrite(13, LOW);

  BTSerial.println(F("0=White 1=Black"));
  Serial.println(F("0=White 1=Black"));
}

/*************************************************************************
* Function: testEachSensor
* Tests each sensor pin one by one
*************************************************************************/
void testEachSensor() {
  BTSerial.println(F("\n=== ONE BY ONE TEST ==="));
  Serial.println(F("\n=== ONE BY ONE TEST ==="));

  // DIGITAL mode pins: 2, 3, 7, 11, 12, A0 (6 sensors)
  uint8_t pins[] = {2, 3, 7, 11, 12, A0};
  const char* pinNames[] = {"2", "3", "7", "11", "12", "A0"};

  // Turn on emitter
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(50);

  BTSerial.println(F("Place over BLACK line"));
  Serial.println(F("Place over BLACK line"));
  delay(1000);

  for (uint8_t i = 0; i < 6; i++) {
    // Test this pin in DIGITAL mode
    pinMode(pins[i], INPUT);
    int val = digitalRead(pins[i]);

    // Print results
    BTSerial.print(F("Pin "));
    BTSerial.print(pinNames[i]);
    BTSerial.print(F(": "));
    BTSerial.print(val);

    Serial.print(F("Pin "));
    Serial.print(pinNames[i]);
    Serial.print(F(": "));
    Serial.print(val);

    // Interpret result (0=white, 1=black)
    if (val == 1) {
      BTSerial.println(F(" [Black/OK]"));
      Serial.println(F(" [Black/OK]"));
    } else {
      BTSerial.println(F(" [White]"));
      Serial.println(F(" [White]"));
    }

    delay(300);
  }

  digitalWrite(13, LOW);
  BTSerial.println(F("\nAll sensors tested!"));
  Serial.println(F("\nAll sensors tested!"));
}

/*************************************************************************
* Function: testPower
* Tests if sensor has power by checking analog pins
*************************************************************************/
void testPower() {
  BTSerial.println(F("\n=== POWER TEST ==="));
  Serial.println(F("\n=== POWER TEST ==="));

  // Check analog pins for any activity
  BTSerial.print(F("A0:")); BTSerial.print(analogRead(A0));
  BTSerial.print(F(" A1:")); BTSerial.print(analogRead(A1));
  BTSerial.print(F(" A2:")); BTSerial.println(analogRead(A2));

  Serial.print(F("A0:")); Serial.print(analogRead(A0));
  Serial.print(F(" A1:")); Serial.print(analogRead(A1));
  Serial.print(F(" A2:")); Serial.println(analogRead(A2));

  BTSerial.println(F("\nCheck connections:"));
  Serial.println(F("\nCheck connections:"));
  BTSerial.println(F("QTR VCC -> Arduino 5V"));
  Serial.println(F("QTR VCC -> Arduino 5V"));
  BTSerial.println(F("QTR GND -> Arduino GND"));
  Serial.println(F("QTR GND -> Arduino GND"));
}

/*************************************************************************
* Function: stop_motors
* Stops both motors
*************************************************************************/
void stop_motors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}