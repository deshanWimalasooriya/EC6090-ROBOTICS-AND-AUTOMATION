/*
 * ============================================================
 *  LINE FOLLOWING ROBOT — ARDUINO UNO (TESTING VERSION v4)
 * ============================================================
 *  Hardware:
 *    - Arduino Uno
 *    - 5x IR Sensors (Analog Output)
 *    - TCS230 Color Sensor
 *    - HC-SR04 Ultrasonic Sensor
 *    - L298N Motor Driver (1x, rear wheel drive)
 *    - SG90 Servo Gripper (SNM200 Claw)
 *
 *  BUG FIXED in v4:
 *    ❌ v3 BUG: Servo on D10 uses Timer1
 *               Timer1 also controls D9 (ENA/left motor PWM)
 *               → Left motor PWM broken → only right motor worked!
 *
 *    ✅ v4 FIX: ENA moved from D9 to D11 (Timer2 safe)
 *               ENB stays on D6 (Timer0, always safe)
 *               Servo stays on D10 (Timer1, now no conflict)
 *               TCS230 S0 moved from D11 to D12
 *               TCS230 S1 moved from D12 to D13 (shared logic adjusted)
 *
 *  ARDUINO UNO TIMER MAP (important!):
 *    Timer0 → D5, D6   (also used by millis/delay — don't mess with it)
 *    Timer1 → D9, D10  (Servo library takes this over)
 *    Timer2 → D3, D11  (free for analogWrite after Servo uses Timer1)
 *
 *  FINAL PIN MAP v4:
 *    D0       → Reserved Serial RX — DO NOT USE
 *    D1       → Reserved Serial TX — DO NOT USE
 *    D2       → L298N IN1 (left motor dir A)
 *    D3       → L298N IN2 (left motor dir B)
 *    D4       → L298N IN3 (right motor dir A)
 *    D5       → L298N IN4 (right motor dir B)
 *    D6~      → L298N ENB (right motor PWM) — Timer0 ✅
 *    D7       → HC-SR04 TRIG
 *    D8       → HC-SR04 ECHO
 *    D9       → TCS230 S0  (moved here, digital only)
 *    D10~     → SG90 Servo (Timer1) ✅
 *    D11~     → L298N ENA  (left motor PWM) — Timer2 ✅ NO CONFLICT
 *    D12      → TCS230 S1
 *    D13      → TCS230 S2 (OUTPUT) / TCS230 OUT (INPUT) shared
 *    A0       → IR Sensor S1 (Far Left)
 *    A1       → IR Sensor S2 (Mid Left)
 *    A2       → IR Sensor S3 (Center)
 *    A3       → IR Sensor S4 (Mid Right)
 *    A4       → IR Sensor S5 (Far Right)
 *    A5       → TCS230 S3 (digital output)
 *
 *  WHY THIS WORKS:
 *    Servo  → Timer1 → controls D9, D10
 *    ENA    → D11   → Timer2 → completely separate timer ✅
 *    ENB    → D6    → Timer0 → completely separate timer ✅
 *    Both motors get proper PWM speed control!
 *
 *  Author  : EC6090 Mini Project 2026
 *  Board   : Arduino Uno (migrate to ESP32 + 7 sensors later)
 * ============================================================
 */

#include <Servo.h>

// ============================================================
//  PIN DEFINITIONS — v4 CORRECTED
// ============================================================

// --- IR Sensors (Analog Output) ---
#define IR_S1     A0    // Far Left
#define IR_S2     A1    // Mid Left
#define IR_S3     A2    // Center
#define IR_S4     A3    // Mid Right
#define IR_S5     A4    // Far Right

// --- HC-SR04 Ultrasonic ---
#define TRIG_PIN  7
#define ECHO_PIN  8

// --- L298N Motor Driver ---
// ⚠️ ENA moved to D11 (Timer2) to avoid Servo Timer1 conflict!
#define IN1_PIN   2     // Left  motor direction A
#define IN2_PIN   3     // Left  motor direction B
#define IN3_PIN   4     // Right motor direction A
#define IN4_PIN   5     // Right motor direction B
#define ENA_PIN   11    // Left  motor PWM — Timer2 ✅ (was D9, conflicted with Servo)
#define ENB_PIN   6     // Right motor PWM — Timer0 ✅

// --- SG90 Servo ---
// D10 uses Timer1 — ENA must NOT be on D9 (also Timer1)
#define SERVO_PIN 10    // Timer1 — Servo library uses this ✅

// --- TCS230 Color Sensor ---
// S0 moved to D9 (now free since ENA left D9)
// S1 on D12
// S2 + OUT shared on D13 (same as v3)
// S3 on A5
#define TCS_S0_PIN    9     // moved from D11 to D9
#define TCS_S1_PIN    12
#define TCS_D13_PIN   13    // shared: S2 output + OUT input
#define TCS_S3_PIN    A5    // A5 used as digital output

// ============================================================
//  QUICK MOTOR TEST — SET TO true TO TEST MOTORS ONLY
//  Upload with this true first to verify both motors spin
//  Then set back to false for full robot operation
// ============================================================
#define MOTOR_TEST_MODE  false

// ============================================================
//  TUNABLE CONSTANTS
// ============================================================

// IR threshold: analog value ABOVE this = sensor on yellow line
// Calibration steps:
//   1. Open Serial Monitor at 9600 baud
//   2. Uncomment debug block in readIRSensors()
//   3. Note values on bare floor vs yellow line
//   4. Set threshold halfway between the two readings
#define IR_THRESHOLD      500

// Ultrasonic object detection distance (cm)
#define DETECT_DIST       18

// Motor speeds (0–255 PWM)
#define BASE_SPEED      78     // normal line following 155
#define SHARP_SPEED       50   // max speed on sharp turns 100
#define AVOID_SPEED       70   // obstacle bypass 140
#define CREEP_SPEED       50   // slow approach for pickup 100

// PD Controller — tune on actual arena
// If robot wobbles → reduce Kp
// If robot is slow to correct → increase Kp
// If robot overshoots curves → increase Kd
float Kp = 25.0;
float Kd = 12.0;

// Servo claw angles — adjust to your physical claw geometry
#define SERVO_OPEN    85    // claw open  (releasing cube)
#define SERVO_CLOSED  15    // claw closed (gripping cube)

// Time robot travels after picking green cube before
// assuming it reached end zone — tune for your arena!
#define END_ZONE_TRAVEL_MS  8000

// ============================================================
//  GLOBAL VARIABLES
// ============================================================

Servo gripperServo;

int  irRaw[5];
int  irBinary[5];

int  lastError      = 0;
bool cubePickedUp   = false;
int  obstaclesAvoided = 0;
unsigned long pickUpTime = 0;

// ============================================================
//  STATE MACHINE
// ============================================================
enum RobotState {
  STATE_LINE_FOLLOW,
  STATE_IDENTIFY_OBJECT,
  STATE_AVOID_RED,
  STATE_REJOIN_LINE,
  STATE_APPROACH_GREEN,
  STATE_PICK_GREEN,
  STATE_CARRY_TO_END,
  STATE_DROP_CUBE,
  STATE_DONE
};

RobotState currentState = STATE_LINE_FOLLOW;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("=== Robot v4 — Timer conflict fixed ===");
  Serial.println("ENA on D11 (Timer2), Servo on D10 (Timer1)");

  // Motor driver
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);   // D11 — Timer2
  pinMode(ENB_PIN, OUTPUT);   // D6  — Timer0
  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // TCS230
  pinMode(TCS_S0_PIN,  OUTPUT);   // D9
  pinMode(TCS_S1_PIN,  OUTPUT);   // D12
  pinMode(TCS_S3_PIN,  OUTPUT);   // A5
  pinMode(TCS_D13_PIN, OUTPUT);   // D13 starts as output

  // TCS230 frequency scaling 20%: S0=HIGH, S1=LOW
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // Servo
  gripperServo.attach(SERVO_PIN);  // D10 — Timer1
  gripperServo.write(SERVO_OPEN);
  delay(500);

  // ── MOTOR TEST MODE ──────────────────────────────────────
  // If MOTOR_TEST_MODE is true, spin both motors and halt
  // Use this to confirm both motors work before full test
  if (MOTOR_TEST_MODE) {
    Serial.println("=== MOTOR TEST MODE ===");
    Serial.println("Both motors forward 2 seconds...");
    driveMotors(150, 150);
    delay(2000);
    Serial.println("Left motor only...");
    driveMotors(150, 0);
    delay(2000);
    Serial.println("Right motor only...");
    driveMotors(0, 150);
    delay(2000);
    Serial.println("Both reverse...");
    driveMotors(-150, -150);
    delay(2000);
    stopMotors();
    Serial.println("Motor test done. Set MOTOR_TEST_MODE false to run robot.");
    while (true);  // halt here
  }
  // ─────────────────────────────────────────────────────────

  Serial.println("Starting in 3 seconds...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readIRSensors();

  switch (currentState) {

    // --------------------------------------------------------
    case STATE_LINE_FOLLOW: {
      float dist = readUltrasonic();
      if (dist > 0 && dist < DETECT_DIST) {
        stopMotors();
        delay(300);
        Serial.print("Object at ");
        Serial.print(dist);
        Serial.println("cm — reading color...");
        currentState = STATE_IDENTIFY_OBJECT;
        break;
      }
      followLine();
      break;
    }

    // --------------------------------------------------------
    // Color ONLY read here — robot always stopped in this state
    // --------------------------------------------------------
    case STATE_IDENTIFY_OBJECT: {
      char color = readColorVoted(5);
      Serial.print("Color: ");
      Serial.println(color);

      if (color == 'R') {
        Serial.println(">> RED — avoid");
        currentState = STATE_AVOID_RED;
      } else if (color == 'G' && !cubePickedUp) {
        Serial.println(">> GREEN — pick up");
        currentState = STATE_APPROACH_GREEN;
      } else if (color == 'G' && cubePickedUp) {
        Serial.println(">> GREEN but already carrying — avoid");
        currentState = STATE_AVOID_RED;
      } else {
        Serial.println(">> Unknown — treat as obstacle");
        currentState = STATE_AVOID_RED;
      }
      break;
    }

    // --------------------------------------------------------
    case STATE_AVOID_RED: {
      avoidRedObstacle();
      obstaclesAvoided++;
      Serial.print("Avoided: "); Serial.println(obstaclesAvoided);
      currentState = STATE_REJOIN_LINE;
      break;
    }

    // --------------------------------------------------------
    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) currentState = STATE_LINE_FOLLOW;
      break;
    }

    // --------------------------------------------------------
    case STATE_APPROACH_GREEN: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(400);
      stopMotors();
      delay(200);
      currentState = STATE_PICK_GREEN;
      break;
    }

    // --------------------------------------------------------
    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      currentState = STATE_CARRY_TO_END;
      break;
    }

    // --------------------------------------------------------
    case STATE_CARRY_TO_END: {
      unsigned long elapsed = millis() - pickUpTime;

      if (elapsed > END_ZONE_TRAVEL_MS) {
        Serial.println(">> End zone (timer)");
        stopMotors();
        currentState = STATE_DROP_CUBE;
        break;
      }
      float dist = readUltrasonic();
      if (dist > 0 && dist < 8) {
        Serial.println(">> End zone (wall)");
        stopMotors();
        currentState = STATE_DROP_CUBE;
        break;
      }
      followLine();
      break;
    }

    // --------------------------------------------------------
    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      currentState = STATE_DONE;
      break;
    }

    // --------------------------------------------------------
    case STATE_DONE: {
      stopMotors();
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 2000) {
        Serial.println("*** TASK COMPLETE ***");
        lastPrint = millis();
      }
      break;
    }
  }
}

// ============================================================
//  READ IR SENSORS
// ============================================================
void readIRSensors() {
  irRaw[0] = analogRead(IR_S1);
  irRaw[1] = analogRead(IR_S2);
  irRaw[2] = analogRead(IR_S3);
  irRaw[3] = analogRead(IR_S4);
  irRaw[4] = analogRead(IR_S5);

  for (int i = 0; i < 5; i++) {
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // Uncomment to calibrate IR threshold:
  /*
  Serial.print("RAW: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(irRaw[i]); Serial.print("\t");
  }
  Serial.println();
  */
}

// ============================================================
//  LINE FOLLOWING — PD Controller
// ============================================================
void followLine() {
  int weights[5]  = {-2, -1, 0, 1, 2};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  int error;
  if (sensorSum > 0) {
    error = weightedSum;
  } else {
    error = (lastError > 0) ? 4 : -4;
  }

  float correction = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (abs(error) >= 2) {
    if (leftSpeed  > 0) leftSpeed  = min(leftSpeed,  SHARP_SPEED);
    if (rightSpeed > 0) rightSpeed = min(rightSpeed, SHARP_SPEED);
  }

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  AVOID RED OBSTACLE
// ============================================================
void avoidRedObstacle() {
  driveMotors(-AVOID_SPEED, AVOID_SPEED);   // turn left
  delay(380);
  stopMotors(); delay(150);

  driveMotors(AVOID_SPEED, AVOID_SPEED);    // forward past obstacle
  delay(650);
  stopMotors(); delay(150);

  driveMotors(AVOID_SPEED, -AVOID_SPEED);   // turn right
  delay(380);
  stopMotors(); delay(150);

  driveMotors(AVOID_SPEED, AVOID_SPEED);    // forward to rejoin
  delay(400);
  stopMotors(); delay(200);
}

// ============================================================
//  REJOIN LINE
// ============================================================
bool rejoinLine() {
  unsigned long start = millis();
  while (millis() - start < 4000) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count >= 1) { Serial.println("Line found!"); return true; }
    driveMotors(110, 110);
    delay(40);
  }
  stopMotors();
  Serial.println("WARNING: Line not found!");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
// ============================================================
void pickGreenCube() {
  gripperServo.write(SERVO_OPEN);
  delay(600);
  stopMotors();
  delay(300);
  gripperServo.write(SERVO_CLOSED);
  delay(800);
  Serial.println("Cube gripped!");
}

// ============================================================
//  DROP CUBE AT END ZONE
// ============================================================
void dropCubeAtEnd() {
  stopMotors(); delay(200);
  gripperServo.write(SERVO_OPEN);
  delay(800);
  driveMotors(-100, -100);
  delay(300);
  stopMotors();
  cubePickedUp = false;
  Serial.println("Cube released!");
}

// ============================================================
//  TCS230 COLOR SENSOR
//  D13 shared: OUTPUT for S2, INPUT for reading OUT frequency
// ============================================================
char readColorOnce() {
  long r = 0, g = 0, b = 0;

  // Red: S2=LOW S3=LOW
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);
  digitalWrite(TCS_S3_PIN,  LOW);
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  r = pulseIn(TCS_D13_PIN, LOW, 60000);

  // Green: S2=HIGH S3=HIGH
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, HIGH);
  digitalWrite(TCS_S3_PIN,  HIGH);
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  g = pulseIn(TCS_D13_PIN, LOW, 60000);

  // Blue: S2=LOW S3=HIGH
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);
  digitalWrite(TCS_S3_PIN,  HIGH);
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  b = pulseIn(TCS_D13_PIN, LOW, 60000);

  // Debug — uncomment to calibrate:
  /*
  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);
  */

  if (r == 0 && g == 0 && b == 0) return 'U';
  long minVal = min(r, min(g, b));
  if (minVal > 250) return 'U';

  // Lower frequency = stronger that color
  if (r < g && r < b) return 'R';
  if (g < r && g < b) return 'G';
  if (b < r && b < g) return 'B';
  return 'U';
}

char readColorVoted(int times) {
  int cR=0, cG=0, cB=0, cU=0;
  for (int i = 0; i < times; i++) {
    char c = readColorOnce();
    if      (c=='R') cR++;
    else if (c=='G') cG++;
    else if (c=='B') cB++;
    else             cU++;
    delay(25);
  }
  Serial.print("Votes R:"); Serial.print(cR);
  Serial.print(" G:"); Serial.print(cG);
  Serial.print(" B:"); Serial.print(cB);
  Serial.print(" U:"); Serial.println(cU);

  if (cR>=cG && cR>=cB && cR>cU) return 'R';
  if (cG>=cR && cG>=cB && cG>cU) return 'G';
  if (cB>=cR && cB>=cG && cB>cU) return 'B';
  return 'U';
}

// ============================================================
//  ULTRASONIC — HC-SR04
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

// ============================================================
//  MOTOR CONTROL — L298N
//  ENA = D11 (Timer2) — NO conflict with Servo on D10 (Timer1)
//  ENB = D6  (Timer0) — always safe
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor (ENA on D11 — Timer2)
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, leftSpeed);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, -leftSpeed);
  }
  // Right motor (ENB on D6 — Timer0)
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, rightSpeed);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

// ============================================================
//  END OF CODE v4
// ============================================================
