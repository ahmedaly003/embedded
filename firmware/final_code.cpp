// Maze Solver - Phase 0 (Final Complete Version)
// Arduino Mega 2560 | ECNG 4504 Project
//
// FEATURES:
// - Adaptive Turns (Turn until line found, not fixed time)
// - Iterative Nudging (Handles T-intersection "all-black blobs")
// - End-Zone Detection (All Black after max nudges = Stop)
// - Overshoot Recovery (Reverses if nudge goes too far)
// - 360° Avoidance (Strict time limits on search)
// - Path Simplification (xBy algorithm for optimal solving)
// - State Machine (TRAINING -> SOLVING -> DONE)
// - 5-Minute Timeout (Charter Compliant) - 1
// - Real-time Serial Status (115200 baud)
//
// CHARTER COMPLIANCE:
// - C++ Only (No BLE in Phase 0)
// - Autonomous Navigation
// - Timeout Enforcement
// - Failure Avoidance (360°, Line Loss)
// ============================================================
#include <EEPROM.h>



typedef enum {
T_INTERSECTION,
PLUS_INTERSECTION,
RIGHT_ELBOW,
LEFT_ELBOW,
FORWARD_LEFT,
FORWARD_RIGHT,
DEAD_END,
TARGET,
NONE
} IntersectionType_t;


// --- EEPROM Memory Map ---
#define EEPROM_MAGIC_ADDR 0 // 1 byte: 'M' means valid data exists
#define EEPROM_PATH_LEN_ADDR 1 // 1 byte: Length of optimized path
#define EEPROM_PATH_START 2 // 100 bytes: The path array ('L', 'R', 'S')
#define MAX_EEPROM_PATH 100

#define EEPROM_LOG_LEN_ADDR 102 // 2 bytes (int): Length of logs
#define EEPROM_LOG_START 104 // Rest of EEPROM: Intersection logs
#define MAX_LOGS 400 // Max intersections to log

// --- Log Variables ---
char intersectionLog[MAX_LOGS];
int logCount = 0;

void logIntersection(char type) {
if (logCount < MAX_LOGS) {
intersectionLog[logCount++] = type;
}
}

// ─────────────────── 1. PIN DEFINITIONS ───────────────────
// Motor Driver (L298N-style)
const int PIN_MOTOR_LEFT_PWM = 9;
const int PIN_MOTOR_LEFT_IN1 = 8;
const int PIN_MOTOR_LEFT_IN2 = 7;

const int PIN_MOTOR_RIGHT_PWM = 3;
const int PIN_MOTOR_RIGHT_IN3 = 5;
const int PIN_MOTOR_RIGHT_IN4 = 4;

// IR Sensor Array (5 sensors: FL, L, C, R, FR)
// LOW = Black Line | HIGH = White Background
const int NUM_SENSORS = 5;
const int PIN_SENSORS[NUM_SENSORS] = {6, 2, 12, 11, 10};

// ─────────────────── 2. TUNABLE CONSTANTS ───────────────────
// ⚠️ TUNE THESE VALUES FOR YOUR ROBOT BEFORE DEMO

// Motor Speeds (0-255 PWM)
const int SPEED_FORWARD = 80; // Normal line-following
const int SPEED_NUDGE = 85; // Slow speed for intersection clearing
const int SPEED_TURN = 110; // Pivot turn speed
const int SPEED_SEARCH = 140; // Line recovery spin speed
const int SPEED_CORRECTION = 100; // Gentle correction speed

// Timing (milliseconds)
const unsigned long TIME_NUDGE_SINGLE = 70; // Duration of ONE nudge pulse
const int MAX_NUDGES = 3; // Max nudges before forcing decision
const unsigned long MAX_TURN_TIME = 7000; // Safety timeout for adaptive turns
const unsigned long TIME_SEARCH_TOTAL = 1200; // MAX total search time (Prevents 360° penalty)
const unsigned long TIMEOUT_TRAINING = 300000; // 5 minutes (Charter Constraint)

// Sensor Reading
const int SENSOR_SAMPLES = 3;// Samples per read for debounce
const unsigned long SAMPLE_DELAY = 500; // Microseconds between samples

// ─────────────────── 2. TUNABLE CONSTANTS ───────────────────
// Motor Trim Values (Adjust to match wheel speeds)
// Start at 0. If robot drifts LEFT, increase LEFT_TRIM or decrease RIGHT_TRIM.
// Range: 0 to 50 (usually small adjustments are enough)
const int MOTOR_LEFT_TRIM = 10; // ⚠️ TUNE THIS
const int MOTOR_RIGHT_TRIM = 0; // ⚠️ TUNE THIS


void setMotors(int leftSpeed, int rightSpeed, bool leftRev = false, bool rightRev = false) {
// ✅ Apply trim compensation
leftSpeed += MOTOR_LEFT_TRIM;
rightSpeed += MOTOR_RIGHT_TRIM;

// Constrain AFTER trimming to ensure valid PWM range (0-255)
leftSpeed = constrain(leftSpeed, 0, 255);
rightSpeed = constrain(rightSpeed, 0, 255);

// Left Motor
analogWrite(PIN_MOTOR_LEFT_PWM, leftSpeed);
digitalWrite(PIN_MOTOR_LEFT_IN1, leftRev ? HIGH : LOW);
digitalWrite(PIN_MOTOR_LEFT_IN2, leftRev ? LOW : HIGH);

// Right Motor
analogWrite(PIN_MOTOR_RIGHT_PWM, rightSpeed);
digitalWrite(PIN_MOTOR_RIGHT_IN3, rightRev ? HIGH : LOW);
digitalWrite(PIN_MOTOR_RIGHT_IN4, rightRev ? LOW : HIGH);
}


// ─────────────────── 3. STATE & PATH VARIABLES ───────────────────
enum RobotState { STATE_TRAINING, STATE_SOLVING, STATE_MANUAL, STATE_DONE };
RobotState currentState = STATE_MANUAL; // Start in manual mode waiting for '1'

char path[100]; // Stores moves: 'L'=Left, 'R'=Right, 'S'=Straight, 'B'=Back
int pathLength = 0; // Current length of recorded path
int solveIndex = 0; // Current index when replaying path

unsigned long trainingStartMillis = 0;
unsigned long searchStartMillis = 0;

unsigned long lastLogTime = 0; // ← ADD THIS

// ─────────────────── 4. SETUP ───────────────────
void setup() {
// Fast Serial for real-time status (Charter: "output indication in real-time")
Serial.begin(9600);
Serial.begin(9600);
while (!Serial); // Wait for serial port (optional, prevents missed first logs)

Serial.println(F("=== ECNG 4504 Maze Solver - Phase 0 ==="));
Serial.println(F("▶️ System Initialized"));

// Motor Pins
pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
pinMode(PIN_MOTOR_RIGHT_IN3, OUTPUT);
pinMode(PIN_MOTOR_RIGHT_IN4, OUTPUT);

// Sensor Pins
for (int i = 0; i < NUM_SENSORS; i++) {
pinMode(PIN_SENSORS[i], INPUT);
}

stopMotors();
trainingStartMillis = millis();
// Serial.println(F("▶️ State: TRAINING"));
Serial.println(F("Sensor array ready — logs will print every 250 ms for normal following"));
delay(1000); // Brief pause before moving
}

// ─────────────────── 6. SENSOR READING ───────────────────
// Debounced reading to reduce noise

void readSensors(int values[]) {
int local_readings[5] = {0};
for(int i = 0; i < SENSOR_SAMPLES; i++)
{
local_readings[0] += digitalRead(PIN_SENSORS[0]);
local_readings[1] += digitalRead(PIN_SENSORS[1]);
local_readings[2] += digitalRead(PIN_SENSORS[2]);
local_readings[3] += digitalRead(PIN_SENSORS[3]);
local_readings[4] += digitalRead(PIN_SENSORS[4]);
delayMicroseconds(500);
}
values[0] = local_readings[0] > SENSOR_SAMPLES/2 ? 1 : 0;
values[1] = local_readings[1] > SENSOR_SAMPLES/2 ? 1 : 0;
values[2] = local_readings[2] > SENSOR_SAMPLES/2 ? 1 : 0;
values[3] = local_readings[3] > SENSOR_SAMPLES/2 ? 1 : 0;
values[4] = local_readings[4] > SENSOR_SAMPLES/2 ? 1 : 0;

}

// void readSensors(int values[]) {
// for (int i = 0; i < NUM_SENSORS; i++) {
// int blackCount = 0;
// for (int j = 0; j < SENSOR_SAMPLES; j++) {
// if (digitalRead(PIN_SENSORS[i]) == LOW) blackCount++;
// delayMicroseconds(SAMPLE_DELAY);
// }
// // Majority vote: >50% black = on line
// values[i] = (blackCount > SENSOR_SAMPLES / 2) ? LOW : HIGH;
// }

// // Serial.print(F("STATUS | Sensors: "));
// // for (int i = 0; i < NUM_SENSORS; i++) {
// // Serial.print(onLine(i, values) ? "1" : "0");
// // Serial.print(" ");
// // }
// }

int sensor_readings[5];

IntersectionType_t get_current_intersection(){
IntersectionType_t local_current_intersection = NONE;
readSensors(sensor_readings);

// =========================================================
// 🛑 1. BULLETPROOF DEAD END DETECTION (Active Scan)
// =========================================================
if (sensor_readings[0] == 1 && sensor_readings[1] == 1 && sensor_readings[2] == 1 && sensor_readings[3] == 1 && sensor_readings[4] == 1) {

// We lost the line. Let's drive forward for a fraction of a second
// to see if the line comes back (gap in tape, glare, or scratch).
unsigned long deadEndTimer = millis();
bool falseAlarm = false;

driveForward();

// actively scan for up to 150ms
while (millis() - deadEndTimer < 300) {
readSensors(sensor_readings);

// If ANY sensor catches the black line again, it's NOT a dead end!
if (sensor_readings[0] == 0 || sensor_readings[1] == 0 || sensor_readings[2] == 0 || sensor_readings[3] == 0 || sensor_readings[4] == 0) {
falseAlarm = true;
break; // Break out of the scanning loop instantly
}
}

if (!falseAlarm) {
// We drove forward for 150ms and saw absolutely nothing but white.
// This is a verified, 100% real dead end.
stopMotors();
return DEAD_END;
}

// If falseAlarm was true, the code just ignores the dead end and
// drops down to the normal line-following/intersection logic below!
}

// =========================================================
// 🛡️ 2. AUTO-ALIGNMENT GUARD (Anti-Drift)
// =========================================================
if (sensor_readings[0] == 0 && sensor_readings[2] == 1) {
setMotors(SPEED_TURN, SPEED_TURN, true, false); // Pivot Left
unsigned long alignTimer = millis();
while(sensor_readings[2] == 1 && millis() - alignTimer < 500) { readSensors(sensor_readings); }
stopMotors(); delay(40); readSensors(sensor_readings);
}
else if (sensor_readings[4] == 0 && sensor_readings[2] == 1) {
setMotors(SPEED_TURN, SPEED_TURN, false, true); // Pivot Right
unsigned long alignTimer = millis();
while(sensor_readings[2] == 1 && millis() - alignTimer < 500) { readSensors(sensor_readings); }
stopMotors(); delay(40); readSensors(sensor_readings);
}

if (sensor_readings[0] == 1 && sensor_readings[4] == 1) {
return NONE; // False alarm, keep driving
}

// =========================================================
// 3. ACTUAL INTERSECTION LOGIC (With End Zone Timeout)
// =========================================================
bool flag_is_possible_Plus = false;

// ⏱️ START THE CLOCK!
// We will measure how long it takes to cross the black tape.
unsigned long crossTimer = millis();

// --- LEFT SENSOR HIT ---
if (sensor_readings[0] == 0) {
driveForward();
while(0 == sensor_readings[0]) {
readSensors(sensor_readings);

// 🏁 END ZONE CHECK: If it takes >300ms to cross, it's a giant box, not a line!
if (millis() - crossTimer > 300) { stopMotors(); return TARGET; }

if(0 == sensor_readings[4]) {
flag_is_possible_Plus = true;
break;
}
}

if(flag_is_possible_Plus) {
driveForward();
while(0 == sensor_readings[4] || 0 == sensor_readings[0]) {
readSensors(sensor_readings);

// 🏁 END ZONE CHECK
if (millis() - crossTimer > 300) { stopMotors(); return TARGET; }
}
delay(40);
readSensors(sensor_readings);

if(0 == sensor_readings[1] || 0 == sensor_readings[2] || 0 == sensor_readings[3]) {
local_current_intersection = PLUS_INTERSECTION;
} else {
local_current_intersection = T_INTERSECTION;
}
} else {
delay(40);
readSensors(sensor_readings);
if(0 == sensor_readings[1] || 0 == sensor_readings[2] || 0 == sensor_readings[3]) {
local_current_intersection = FORWARD_LEFT;
} else {
local_current_intersection = LEFT_ELBOW;
}
}
}
// --- RIGHT SENSOR HIT ---
else if (sensor_readings[4] == 0) {
driveForward();
while(0 == sensor_readings[4]) {
readSensors(sensor_readings);

// 🏁 END ZONE CHECK
if (millis() - crossTimer > 300) { stopMotors(); return TARGET; }

if(0 == sensor_readings[0]) {
flag_is_possible_Plus = true;
break;
}
}

if(flag_is_possible_Plus) {
driveForward();
while(0 == sensor_readings[0] || 0 == sensor_readings[4]) {
readSensors(sensor_readings);

// 🏁 END ZONE CHECK
if (millis() - crossTimer > 300) { stopMotors(); return TARGET; }
}
delay(40);
readSensors(sensor_readings);

if(0 == sensor_readings[1] || 0 == sensor_readings[2] || 0 == sensor_readings[3]) {
local_current_intersection = PLUS_INTERSECTION;
} else {
local_current_intersection = T_INTERSECTION;
}
} else {
delay(40);
readSensors(sensor_readings);
if(0 == sensor_readings[1] || 0 == sensor_readings[2] || 0 == sensor_readings[3]) {
local_current_intersection = FORWARD_RIGHT;
} else {
local_current_intersection = RIGHT_ELBOW;
}
}
}

return local_current_intersection;
}
// ─────────────────── 5. MAIN LOOP ───────────────────

void saveDataToEEPROM() {
Serial.println(F("💾 Saving Path and Logs to EEPROM..."));

// 1. Write Path Length & Data
EEPROM.write(EEPROM_PATH_LEN_ADDR, pathLength);
for (int i = 0; i < pathLength; i++) {
EEPROM.write(EEPROM_PATH_START + i, path[i]);
}

// 2. Write Log Length & Data (using .put() because length is an integer)
EEPROM.put(EEPROM_LOG_LEN_ADDR, logCount);
for (int i = 0; i < logCount; i++) {
EEPROM.write(EEPROM_LOG_START + i, intersectionLog[i]);
}

// 3. Write the Magic Byte LAST.
// This ensures if power is lost during saving, it won't be marked as valid.
EEPROM.write(EEPROM_MAGIC_ADDR, 'M');

Serial.println(F("✅ Save Complete!"));
}

// ─────────────────── BLUETOOTH COMMAND HANDLER ───────────────────
void checkBluetoothCommands() {
if (Serial.available() > 0) {
char cmd = Serial.read();

// DEBUG: Print exactly what we received
Serial.print(F("🧩 [BT DEBUG] Received: '"));
Serial.print(cmd);
Serial.print(F("' (ASCII: "));
Serial.print((int)cmd);
Serial.println(F(")"));

// Serial.println(cmd);

// --- State Changes ---
if (cmd == 'T') {
currentState = STATE_TRAINING;
trainingStartMillis = millis(); // Reset timeout timer
EEPROM.write(EEPROM_MAGIC_ADDR, 0);
pathLength = 0;
path[0] = '\0';
logCount = 0;
Serial.println(F("▶️ Mode: TRAINING STARTED"));
}
else if (cmd == 'M') {
currentState = STATE_MANUAL;
stopMotors();
Serial.println(F("⏸️ Mode: MANUAL / EMERGENCY STOP"));
}
else if (cmd == 'S') {
if (EEPROM.read(EEPROM_MAGIC_ADDR) == 'M') {
Serial.println(F("📂 Valid EEPROM found! Loading path..."));

pathLength = EEPROM.read(EEPROM_PATH_LEN_ADDR);
for (int i = 0; i < pathLength; i++) {
path[i] = EEPROM.read(EEPROM_PATH_START + i);
}
path[pathLength] = '\0'; // Null terminate

Serial.print(F("Loaded Path: "));
Serial.println(path);

solveIndex = 0;
currentState = STATE_SOLVING;
} else {
Serial.println(F("❌ ERROR: No valid path in EEPROM. Must train first!"));
}
}

// --- Manual Movement (Only works if in MANUAL mode) ---
else if (currentState == STATE_MANUAL) {
if (cmd == 'W') setMotors(SPEED_FORWARD, SPEED_FORWARD);
else if (cmd == 'S') setMotors(SPEED_FORWARD, SPEED_FORWARD, true, true); // Reverse
else if (cmd == 'A') setMotors(SPEED_TURN, SPEED_TURN, true, false); // Pivot Left
else if (cmd == 'D') setMotors(SPEED_TURN, SPEED_TURN, false, true); // Pivot Right
else if (cmd == 'X') stopMotors();
}

// --- Global Safety Stop ---
// If X is pressed while training/solving, force a stop.
if (cmd == 'X' && currentState != STATE_MANUAL) {
stopMotors();
}

// --- E: EXTRACT LOGS ---
else if (cmd == 'E') {
if (EEPROM.read(EEPROM_MAGIC_ADDR) == 'M') {
Serial.println(F("📊 --- EEPROM LOG EXTRACTION ---"));

// Extract Path
int savedPathLen = EEPROM.read(EEPROM_PATH_LEN_ADDR);
Serial.print(F("Optimized Path (Len "));
Serial.print(savedPathLen);
Serial.print(F("): "));
for (int i = 0; i < savedPathLen; i++) {
Serial.print((char)EEPROM.read(EEPROM_PATH_START + i));
}
Serial.println();

// Extract Logs
int savedLogLen = 0;
EEPROM.get(EEPROM_LOG_LEN_ADDR, savedLogLen);
Serial.print(F("Raw Intersections Logged (Count "));
Serial.print(savedLogLen);
Serial.print(F("): "));
for (int i = 0; i < savedLogLen; i++) {
Serial.print((char)EEPROM.read(EEPROM_LOG_START + i));
}
Serial.println();
Serial.println(F("--------------------------------"));
} else {
Serial.println(F("❌ ERROR: EEPROM is empty or corrupted."));
}
}
}
}

// ─────────────────── 5. MAIN LOOP ───────────────────
void loop() {
// 1. Always check for incoming PC commands first
checkBluetoothCommands();

if(currentState == STATE_MANUAL) {
int s[5];
readSensors(s);
printSensorStatus(s);
}

// 2. Charter Constraint: 5-minute training timeout
if (currentState == STATE_TRAINING && millis() - trainingStartMillis > TIMEOUT_TRAINING) {
Serial.println(F("⚠ TIMEOUT: Training exceeded 5 minutes"));
stopMotors();
currentState = STATE_DONE;
}

// 3. Finite State Machine
switch (currentState) {
case STATE_TRAINING:
doTraining();
break;
case STATE_SOLVING:
doSolving();
break;
case STATE_MANUAL:
// Do nothing autonomously. checkBluetoothCommands() handles manual movement.
break;
case STATE_DONE:
stopMotors();
// Stay in DONE state indefinitely
break;
}

delay(5); // Minimal loop delay for responsiveness
}




// Helper: Check if specific sensor is on line
bool onLine(int idx, const int values[]) {
return values[idx] == LOW;
}

void printSensorStatus(const int s[]) {
// if (millis() - lastLogTime < 250) return; // throttle — no flood
// lastLogTime = millis();

// Serial.print(F("STATUS | Sensors: "));
// for (int i = 0; i < NUM_SENSORS; i++) {
// Serial.print(s[i]);
// Serial.print(" ");
// }
// Serial.print(F("| State: "));
// Serial.println(currentState == STATE_TRAINING ? "TRAINING" : "SOLVING");
}

// ─────────────────── 7. MOTOR CONTROL ───────────────────
// Unified motor control with direction parameters
// ─────────────────── 7. MOTOR CONTROL (Updated with Trim) ───────────────────

void driveForward() { setMotors(SPEED_FORWARD, SPEED_FORWARD); }
void nudgeForward() { setMotors(SPEED_NUDGE, SPEED_NUDGE); }
void stopMotors() { setMotors(0, 0); }

// ─────────────────── 8. MOVEMENT PRIMITIVES ───────────────────

// ✅ Adaptive Turn Left: Pivots until center sensor finds line
// ─────────────────── TURN RIGHT 90° (FIXED) ───────────────────
void turnRight90() {
Serial.println(F("🔄 Pivoting RIGHT..."));
stopMotors();
delay(50);

// Pivot: Left Forward, Right Reverse
setMotors(SPEED_TURN, SPEED_TURN, false, true);
delay(250);
unsigned long turnStart = millis();
bool lineFound = false;

while (millis() - turnStart < MAX_TURN_TIME) {
int s[NUM_SENSORS];
readSensors(s);

// ✅ FIXED: Center black AND Right-side white (cleared intersection blob)
bool centerBlack = onLine(2, s);
bool rightWhite = !onLine(3, s) && !onLine(4, s); // R and FR must be white

if (centerBlack && rightWhite) {
Serial.println(F("✓ Center found line, right side clear"));
lineFound = true;
break;
}

// Debug: Show why turn didn't stop (optional, remove after tuning)
// Serial.print("C="); Serial.print(centerBlack);
// Serial.print(" R-side="); Serial.println(rightWhite);

delay(5);
}

stopMotors();
delay(50);

if (!lineFound) {
Serial.println(F("⚠ TIMEOUT: Line not found"));
}
}

// ─────────────────── TURN LEFT 90° (FIXED) ───────────────────
void turnLeft90() {
Serial.println(F("🔄 Pivoting LEFT..."));
stopMotors();
delay(50);

// Pivot: Left Reverse, Right Forward
setMotors(SPEED_TURN, SPEED_TURN, true, false);
delay(250);

unsigned long turnStart = millis();
bool lineFound = false;

while (millis() - turnStart < MAX_TURN_TIME) {
int s[NUM_SENSORS];
readSensors(s);

// ✅ FIXED: Center black AND Left-side white (cleared intersection blob)
bool centerBlack = onLine(2, s);
bool leftWhite = !onLine(0, s) && !onLine(1, s); // FL and L must be white

if (centerBlack && leftWhite) {
Serial.println(F("✓ Center found line, left side clear"));
lineFound = true;
break;
}

delay(5);
}

stopMotors();
delay(50);

if (!lineFound) {
Serial.println(F("⚠ TIMEOUT: Line not found"));
}
}

void turnAround180() {
Serial3.println(F("🛑 DEAD END DETECTED: Executing 180 Turn..."));
stopMotors();
delay(50);

// Pivot Right (Left Forward, Right Reverse)
setMotors(SPEED_TURN, SPEED_TURN, false, true);

// 🛑 BLIND TURN: Give the robot time to spin completely off the line.
// 400ms is a good starting point to get past the 90-degree mark.
delay(400);

unsigned long turnStart = millis();
bool lineFound = false;

// We give it double the normal MAX_TURN_TIME since it has to spin twice as far
while (millis() - turnStart < (MAX_TURN_TIME * 2)) {
int s[NUM_SENSORS];
readSensors(s);

// Stop turning the exact moment the CENTER sensor finds the return line
if (s[2] == 0) {
Serial3.println(F("✓ Center found the line! 180 complete."));
lineFound = true;
break;
}

delay(5);
}

stopMotors();
delay(50); // Give the robot a moment to settle physical momentum

if (!lineFound) {
Serial3.println(F("⚠ TIMEOUT: Failed to find line during 180 turn"));
}
}

// ✅ Safe Search: Limited total spin time to avoid 360° penalty
void searchForLine() {
Serial.println(F("🔍 Searching..."));
stopMotors();
searchStartMillis = millis();

// Spin RIGHT first
setMotors(SPEED_SEARCH, SPEED_SEARCH, false, true);
while (millis() - searchStartMillis < TIME_SEARCH_TOTAL / 2) {
int s[NUM_SENSORS]; readSensors(s);
if (onLine(2,s) || onLine(3,s) || onLine(4,s)) {
stopMotors();
Serial.println(F("✓ Line found (Right)"));
return;
}
delay(5);
}

// Spin LEFT if not found
setMotors(SPEED_SEARCH, SPEED_SEARCH, true, false);
while (millis() - searchStartMillis < TIME_SEARCH_TOTAL) {
int s[NUM_SENSORS]; readSensors(s);
if (onLine(0,s) || onLine(1,s) || onLine(2,s)) {
stopMotors();
Serial.println(F("✓ Line found (Left)"));
return;
}
delay(5);
}

stopMotors();
Serial.println(F("✗ Search failed - Stopping to avoid 360° penalty"));
}

// ─────────────────── 9. PATH LOGIC ───────────────────
void recordMove(char move) {
if (pathLength >= 98) {
Serial.println(F("⚠ Path buffer full"));
return;
}
path[pathLength++] = move;
path[pathLength] = '\0';
simplifyPath();
}

// Simplify path using "xBy" pattern reduction (Left-Hand Rule Optimization)
void simplifyPath() {
if (pathLength < 3 || path[pathLength - 2] != 'B') return;

char a = path[pathLength - 3];
char c = path[pathLength - 1];
char rep = 0;

if (a == 'L' && c == 'R') rep = 'B';
else if (a == 'L' && c == 'S') rep = 'R';
else if (a == 'L' && c == 'L') rep = 'S';
else if (a == 'S' && c == 'L') rep = 'R';
else if (a == 'S' && c == 'S') rep = 'B';
else if (a == 'S' && c == 'R') rep = 'L';
else if (a == 'R' && c == 'L') rep = 'B';
else if (a == 'R' && c == 'S') rep = 'L';

if (rep != 0) {
pathLength -= 3;
path[pathLength++] = rep;
path[pathLength] = '\0';
simplifyPath(); // Recursive check for chained simplifications
}
}

// ─────────────────── 10. INTERSECTION HANDLER (UPDATED) ───────────────────
// Handles T-intersections, Crosses, and End-Zones
// ─────────────────── 10. INTERSECTION HANDLER (UPDATED per request) ───────────────────
// Handles T-intersections, Crosses, and End-Zones
// NEW LOGIC: If center black OR all white after nudging → Turn LEFT
void handleComplexIntersection(bool isTraining) {
Serial.println(F("⚠ Complex Intersection (Clearing Blob)"));

int s[NUM_SENSORS];
bool allBlack = true;
bool allWhite = false;
int nudgeCount = 0;

// ── STEP 1: Nudge forward to clear the intersection "blob" ──
// Keep nudging until sensors change OR max nudges reached
for (nudgeCount = 0; nudgeCount < MAX_NUDGES; nudgeCount++) {
nudgeForward();
delay(TIME_NUDGE_SINGLE);
stopMotors();
delay(30);

readSensors(s);
bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);
allBlack = (fl && l && c && r && fr);
allWhite = (!fl && !l && !c && !r && !fr);

// Exit loop once we see a change (not all black anymore)
if (!allBlack) {
Serial.print(F("✓ Blob cleared after "));
Serial.print(nudgeCount + 1);
Serial.println(F(" nudge(s)"));
break;
}
}

printSensorStatus(s); // shows exact sensor pattern after nudge

// ── STEP 2: Re-evaluate sensors after nudging ──
readSensors(s);
bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);

allBlack = (fl && l && c && r && fr);
allWhite = (!fl && !l && !c && !r && !fr);
bool centerBlack = onLine(2, s); // ✅ Center sensor sees black

// ── STEP 3: END OF MAZE DETECTION ──
// If still ALL BLACK after max nudges → This is the end zone (black box)
if (allBlack && nudgeCount >= MAX_NUDGES - 1) {
Serial.println(F("🏁 END OF MAZE (All Black after max nudges)"));
finalizeMaze(isTraining);
return;
}

// ── STEP 4: OVERSHOOT DETECTION (All White) ──
// If all sensors see WHITE after nudge → We overshot the intersection
if (allWhite) {
Serial.println(F("⚠ All White after nudge - Overshoot detected"));

// Try to recover by reversing slightly
setMotors(SPEED_NUDGE, SPEED_NUDGE, true, true); // Reverse both motors
delay(150);
stopMotors();
delay(30);

readSensors(s);
bool recovered = onLine(0,s)||onLine(1,s)||onLine(2,s)||onLine(3,s)||onLine(4,s);

if (recovered) {
Serial.println(F("✓ Overshoot recovered - Re-evaluating"));
// Re-read sensors after recovery
readSensors(s);
fl = onLine(0,s); l = onLine(1,s); c = onLine(2,s); r = onLine(3,s); fr = onLine(4,s);
allBlack = (fl && l && c && r && fr);
allWhite = (!fl && !l && !c && !r && !fr);
centerBlack = onLine(2, s);
} else {
// Could not recover - treat as dead end → Turn LEFT per your request
Serial.println(F("✗ Could not recover - Turning LEFT (Left-Hand Rule fallback)"));
if (isTraining) {
recordMove('L'); // Record left turn for path optimization
}
turnLeft90();
return;
}
}

// ── STEP 5: DECISION BASED ON YOUR REQUESTED LOGIC ──
// ✅ If CENTER BLACK OR ALL WHITE → Turn LEFT
char decision = 'S'; // Default fallback

if (centerBlack || allWhite) {
Serial.println(F("↰ Decision: CENTER BLACK or ALL WHITE → Turn LEFT"));
decision = 'L';
}
// Fallback: Use standard Left-Hand Rule if above condition not met
else if (fl || l) {
Serial.println(F("↰ Decision: Left branch detected → Turn LEFT"));
decision = 'L';
}
else if (fr || r) {
Serial.println(F("↱ Decision: Right branch detected → Turn RIGHT"));
decision = 'R';
}
else {
Serial.println(F("⚠ Decision: No direction → Backtrack (Dead end)"));
decision = 'B';
}

// Record and execute decision
if (isTraining) {
recordMove(decision);
}

if (decision == 'L') {
turnLeft90();
}
else if (decision == 'R') {
turnRight90();
}
else if (decision == 'B') {
turnAround180();
}
// 'S' continues forward (should not reach here with current logic)
}

void finalizeMaze(bool isTraining) {
stopMotors();
delay(300);

if (isTraining) {
Serial.println(F("✅ TRAINING COMPLETE"));
Serial.print(F("📋 Optimized Path: "));
Serial.println(path);
Serial.println(F("⚠️ PLEASE REPOSITION ROBOT AT START"));
Serial.println(F("⚠️ Then press RESET to begin SOLVING")); // Just use reset button

// currentState = STATE_SOLVING;
solveIndex = 0;

// Simple delay instead of button wait
delay(2000); // 2-second pause before solving
Serial.println(F("▶️ Starting SOLVING phase..."));
delay(1000);
} else {
Serial.println(F("🏆 MAZE SOLVED!"));
currentState = STATE_DONE;
}
}

// ─────────────────── 11. STATE HANDLERS ───────────────────

// TRAINING PHASE
void doTraining() {
int s[NUM_SENSORS];
readSensors(s);

IntersectionType_t local_intersection_type = get_current_intersection();

if(T_INTERSECTION == local_intersection_type) {
Serial.println("T_INTERSECTION: type intersection detected");
recordMove('L');
logIntersection('T');
turnLeft90();
} else if(PLUS_INTERSECTION == local_intersection_type) {
Serial.println("PLUS_INTERSECTION: type intersection detected");
recordMove('L');
logIntersection('P');
turnLeft90();
} else if(LEFT_ELBOW == local_intersection_type) {
Serial.println("LEFT_ELBOW: type intersection detected");
recordMove('L');
logIntersection('l');
turnLeft90();
} else if(RIGHT_ELBOW == local_intersection_type) {
Serial.println("RIGHT_ELBOW: type intersection detected");
recordMove('R');
logIntersection('r');
turnRight90();
} else if(FORWARD_LEFT == local_intersection_type) {
Serial.println("FOWRARD_LEFT: type intersection detected");
recordMove('L');
logIntersection('F');
turnLeft90();
} else if(FORWARD_RIGHT == local_intersection_type) {
Serial.println("FORWARD_RIGHT: type intersection detected");
recordMove('S');
logIntersection('f');
driveForward();
} else if(DEAD_END == local_intersection_type) {
Serial.println("DEAD_END: type intersection detected");
recordMove('B');
logIntersection('D');
turnAround180();
} else if(TARGET == local_intersection_type) {
Serial.println("TARGET: Target Reached!");
logIntersection('E');
finalizeMaze(true); // Calls your finalize function to stop motors and prep for solving!
saveDataToEEPROM();
return;
}

bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);

// Complex Intersection (All 5 Sensors)
// if (fl && l && c && r && fr) {
// Serial.println("Complex Intersection");
// handleComplexIntersection(true);
// return;
// }

// Dead End (All White)
// if (!fl && !l && !c && !r && !fr) {
// Serial.println("Dead End");
// recordMove('B');
// turnAround180();
// return;
// }
// Simple Left Turn (Far Left + Center, NOT Far Right)
// if (fl && l && c && !r && !fr) {
// Serial.println("Simple Left Turn");
// recordMove('L');
// turnLeft90();
// return;
// }

// // Simple Right Turn (Far Right + Center, NOT Far Left)
// if (fr && r && c && !l && !fl) {
// Serial.println("Simple Right Turn");
// recordMove('R');
// turnRight90();
// return;
// }

// Normal Line Following + throttled status
// printSensorStatus(s); // ← meaningful slow log

if (c && !l && !fl && !r && !fr) {
// Serial.println(F("→ Straight line following"));
driveForward();
}
else if ((l || fl) && !r && !fr) {
// Serial.println(F("→ Gentle LEFT correction"));
setMotors(SPEED_TURN, SPEED_TURN, true, false);
}
else if ((r || fr) && !l && !fl) {
// Serial.println(F("→ Gentle RIGHT correction"));
setMotors(SPEED_TURN, SPEED_TURN, false, true);
}
// else if (!fl && !l && !c && !r && !fr) {
// Serial.println(F("→ All white → searchForLine"));
// searchForLine();
// }
else {
printSensorStatus(s);
// Serial.println(F("→ Fallback forward"));
driveForward();
}
}

// SOLVING PHASE
// ─────────────────── SOLVING PHASE ───────────────────
void doSolving() {
int s[NUM_SENSORS];
readSensors(s);

IntersectionType_t local_intersection_type = get_current_intersection();

// for debugging
if(T_INTERSECTION == local_intersection_type) {
Serial.println("T_INTERSECTION: type intersection detected");
} else if(PLUS_INTERSECTION == local_intersection_type) {
Serial.println("PLUS_INTERSECTION: type intersection detected");
} else if(LEFT_ELBOW == local_intersection_type) {
Serial.println("LEFT_ELBOW: type intersection detected");
} else if(RIGHT_ELBOW == local_intersection_type) {
Serial.println("RIGHT_ELBOW: type intersection detected");
} else if(FORWARD_LEFT == local_intersection_type) {
Serial.println("FOWRARD_LEFT: type intersection detected");
} else if(FORWARD_RIGHT == local_intersection_type) {
Serial.println("FORWARD_RIGHT: type intersection detected");
} else if(DEAD_END == local_intersection_type) {
Serial.println("DEAD_END: type intersection detected");
} else if(TARGET == local_intersection_type) {
Serial.println("TARGET: Target Reached!");
// return;
}

// ==========================================
// 1. INTERSECTION HANDLING (Fault-Tolerant)
// ==========================================

if (T_INTERSECTION == local_intersection_type ||
PLUS_INTERSECTION == local_intersection_type ||
LEFT_ELBOW == local_intersection_type ||
RIGHT_ELBOW == local_intersection_type ||
FORWARD_LEFT == local_intersection_type ||
FORWARD_RIGHT == local_intersection_type) {

if (solveIndex < pathLength) {
char nextMove = path[solveIndex];
char moveToExecute = 'S'; // Default fallback
bool consumePath = false;

// 1. LEFT ELBOW: Only Left is possible
if (local_intersection_type == LEFT_ELBOW) {
moveToExecute = 'L';
if (nextMove == 'L') consumePath = true;
}
// 2. RIGHT ELBOW: Only Right is possible
else if (local_intersection_type == RIGHT_ELBOW) {
moveToExecute = 'R';
if (nextMove == 'R') consumePath = true;
}
// 3. FORWARD-LEFT: Can go Left or Straight
else if (local_intersection_type == FORWARD_LEFT) {
if (nextMove == 'L' || nextMove == 'S') {
moveToExecute = nextMove;
consumePath = true;
} else {
moveToExecute = 'S'; // Path said 'R', but we can't! Default to Straight.
}
}
// 4. FORWARD-RIGHT: Can go Right or Straight
else if (local_intersection_type == FORWARD_RIGHT) {
if (nextMove == 'R' || nextMove == 'S') {
moveToExecute = nextMove;
consumePath = true;
} else {
moveToExecute = 'S'; // Path said 'L', but we can't! Default to Straight.
}
}
// 5. T-INTERSECTION: Can go Left or Right
else if (local_intersection_type == T_INTERSECTION) {
if (nextMove == 'L' || nextMove == 'R') {
moveToExecute = nextMove;
consumePath = true;
} else {
moveToExecute = 'R'; // Path said 'S', but we can't! Guess Right.
}
}
// 6. PLUS INTERSECTION: Any move is valid
else if (local_intersection_type == PLUS_INTERSECTION) {
moveToExecute = nextMove;
consumePath = true;
}

// Apply the decision
if (consumePath) {
solveIndex++; // Move was valid! Consume it from the array.
Serial.print("Valid move! Executing: ");
} else {
Serial.print("Mismatch! Path says ");
Serial.print(nextMove);
Serial.print(" but forced to execute: ");
}
Serial.println(moveToExecute);

// Execute the hardware turn
if (moveToExecute == 'L') turnLeft90();
else if (moveToExecute == 'R') turnRight90();
else driveForward();

} else {
Serial.println("❌ ERROR: Path array ended early!");
stopMotors();
currentState = STATE_DONE;
}
return;
}

// ==========================================
// 2. NORMAL LINE FOLLOWING (Fall-through)
// ==========================================
// If the intersection type didn't trigger any of the above, we just follow the line.
else{
bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);

if (c && !l && !fl && !r && !fr) {
driveForward();
}
else if ((l || fl) && !r && !fr) {
// SMOOTH LEFT CORRECTION (Deadband fix applied: 30 instead of reverse)
setMotors(SPEED_TURN, SPEED_TURN, true, false);
}
else if ((r || fr) && !l && !fl) {
// SMOOTH RIGHT CORRECTION (Deadband fix applied: 30 instead of reverse)
setMotors(SPEED_TURN, SPEED_TURN, false, true);
}
else {
// printSensorStatus(s);
// driveForward();
}
}
}
