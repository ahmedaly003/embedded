// Maze Solver - Phase 1 (BLE Edition)
// Arduino Mega 2560 | ECNG 4504 Project
//
// CHANGES FROM PHASE 0:
// - Command input moved from Serial (USB) to Serial3 (HM-10 BLE Module)
// - Serial3 wired to TX3 (pin 14) and RX3 (pin 15)
// - Added periodic STATUS broadcast every 200ms for Python dashboard
// - Status format: "SENS:XXXXX|STATE:XXXXXX|PATH:XXX|IDX:X"
// - USB Serial kept for development/debug monitoring only
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
#define EEPROM_MAGIC_ADDR   0
#define EEPROM_PATH_LEN_ADDR 1
#define EEPROM_PATH_START   2
#define MAX_EEPROM_PATH     100
#define EEPROM_LOG_LEN_ADDR 102
#define EEPROM_LOG_START    104
#define MAX_LOGS            400

// --- Log Variables ---
char intersectionLog[MAX_LOGS];
int logCount = 0;

void logIntersection(char type) {
  if (logCount < MAX_LOGS) {
    intersectionLog[logCount++] = type;
  }
}

// ─────────────────── 1. PIN DEFINITIONS ───────────────────
const int PIN_MOTOR_LEFT_PWM  = 9;
const int PIN_MOTOR_LEFT_IN1  = 8;
const int PIN_MOTOR_LEFT_IN2  = 7;
const int PIN_MOTOR_RIGHT_PWM = 3;
const int PIN_MOTOR_RIGHT_IN3 = 5;
const int PIN_MOTOR_RIGHT_IN4 = 4;

const int NUM_SENSORS = 5;
const int PIN_SENSORS[NUM_SENSORS] = {6, 2, 12, 11, 10};

// ─────────────────── 2. TUNABLE CONSTANTS ───────────────────
const int SPEED_FORWARD    = 80;
const int SPEED_NUDGE      = 85;
const int SPEED_TURN       = 110;
const int SPEED_SEARCH     = 140;
const int SPEED_CORRECTION = 100;

const unsigned long TIME_NUDGE_SINGLE  = 70;
const int MAX_NUDGES                   = 3;
const unsigned long MAX_TURN_TIME      = 7000;
const unsigned long TIME_SEARCH_TOTAL  = 1200;
const unsigned long TIMEOUT_TRAINING   = 300000;

const int SENSOR_SAMPLES  = 3;
const unsigned long SAMPLE_DELAY = 500;

const int MOTOR_LEFT_TRIM  = 10;
const int MOTOR_RIGHT_TRIM = 0;

// ─────────────────── 3. STATE & PATH VARIABLES ───────────────────
enum RobotState { STATE_TRAINING, STATE_SOLVING, STATE_MANUAL, STATE_DONE };
RobotState currentState = STATE_MANUAL;

char path[100];
int pathLength  = 0;
int solveIndex  = 0;

unsigned long trainingStartMillis = 0;
unsigned long searchStartMillis   = 0;
unsigned long lastLogTime         = 0;

int sensor_readings[5];

// ─────────────────── FORWARD DECLARATIONS ───────────────────
void readSensors(int values[]);
void driveForward();
void nudgeForward();
void stopMotors();
void turnLeft90();
void turnRight90();
void turnAround180();
void searchForLine();
void recordMove(char move);
void simplifyPath();
void finalizeMaze(bool isTraining);
void handleComplexIntersection(bool isTraining);
void doTraining();
void doSolving();
void printSensorStatus(const int s[]);
bool onLine(int idx, const int values[]);
void setMotors(int leftSpeed, int rightSpeed, bool leftRev = false, bool rightRev = false);
IntersectionType_t get_current_intersection();
void saveDataToEEPROM();
void checkBluetoothCommands();
void broadcastStatus();

// ─────────────────── STATUS BROADCAST ───────────────────
// Sends a compact status string to BLE every 200ms
// Format: "SENS:10001|STATE:TRAINING|PATH:LLR|IDX:2\n"
unsigned long lastStatusBroadcast = 0;
const unsigned long STATUS_INTERVAL = 200; // ms

void broadcastStatus() {
  if (millis() - lastStatusBroadcast < STATUS_INTERVAL) return;
  lastStatusBroadcast = millis();

  int s[NUM_SENSORS];
  readSensors(s);

  // Build sensor string "10001"
  char sensorStr[6];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorStr[i] = s[i] == 0 ? '1' : '0'; // 1=on line, 0=off line
  }
  sensorStr[5] = '\0';

  // State string
  const char* stateStr;
  switch (currentState) {
    case STATE_TRAINING: stateStr = "TRAINING"; break;
    case STATE_SOLVING:  stateStr = "SOLVING";  break;
    case STATE_MANUAL:   stateStr = "MANUAL";   break;
    case STATE_DONE:     stateStr = "DONE";     break;
    default:             stateStr = "UNKNOWN";  break;
  }

  // Send over BLE (Serial3)
  Serial3.print(F("SENS:"));
  Serial3.print(sensorStr);
  Serial3.print(F("|STATE:"));
  Serial3.print(stateStr);
  Serial3.print(F("|PATH:"));
  Serial3.print(pathLength > 0 ? path : "-");
  Serial3.print(F("|IDX:"));
  Serial3.println(solveIndex);
}

// ─────────────────── 4. SETUP ───────────────────
void setup() {
  // USB Serial — debug/monitoring only
  Serial.begin(9600);
  Serial.println(F("=== ECNG 4504 Maze Solver - Phase 1 (BLE) ==="));
  Serial.println(F("USB: Debug only | BLE (Serial3): Command interface"));

  // ✅ HM-10 BLE Module on TX3/RX3 (pins 14/15)
  Serial3.begin(9600);
  Serial3.println(F("HM-10 BLE Ready"));

  // Motor Pins
  pinMode(PIN_MOTOR_LEFT_PWM,  OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN1,  OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN2,  OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN4, OUTPUT);

  // Sensor Pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(PIN_SENSORS[i], INPUT);
  }

  stopMotors();
  trainingStartMillis = millis();
  Serial.println(F("Ready. Waiting for BLE command..."));
  delay(1000);
}

// ─────────────────── 5. MAIN LOOP ───────────────────
void loop() {
  checkBluetoothCommands();

  // Periodic status broadcast to Python dashboard
  broadcastStatus();

  if (currentState == STATE_TRAINING && millis() - trainingStartMillis > TIMEOUT_TRAINING) {
    Serial.println(F("TIMEOUT: Training exceeded 5 minutes"));
    Serial3.println(F("ERR:TIMEOUT"));
    stopMotors();
    currentState = STATE_DONE;
  }

  switch (currentState) {
    case STATE_TRAINING: doTraining(); break;
    case STATE_SOLVING:  doSolving();  break;
    case STATE_MANUAL:   break;
    case STATE_DONE:
      stopMotors();
      break;
  }

  delay(5);
}

// ─────────────────── 6. BLUETOOTH COMMAND HANDLER ───────────────────
// NOW USES Serial3 (HM-10 on TX3/RX3 pins 14/15)
// USB Serial (Serial) is kept for debug mirroring only
void checkBluetoothCommands() {
  if (Serial3.available() > 0) {
    char cmd = Serial3.read();

    // Mirror to USB for debugging
    Serial.print(F("[BLE RX] '"));
    Serial.print(cmd);
    Serial.print(F("' (ASCII: "));
    Serial.print((int)cmd);
    Serial.println(F(")"));

    // --- State Changes ---
    if (cmd == 'T') {
      currentState = STATE_TRAINING;
      trainingStartMillis = millis();
      EEPROM.write(EEPROM_MAGIC_ADDR, 0);
      pathLength = 0;
      path[0] = '\0';
      logCount = 0;
      Serial3.println(F("OK:TRAINING_STARTED"));
      Serial.println(F("BLE: TRAINING STARTED"));
    }
    else if (cmd == 'M') {
      currentState = STATE_MANUAL;
      stopMotors();
      Serial3.println(F("OK:MANUAL"));
      Serial.println(F("BLE: MANUAL MODE"));
    }
    else if (cmd == 'S') {
      if (EEPROM.read(EEPROM_MAGIC_ADDR) == 'M') {
        pathLength = EEPROM.read(EEPROM_PATH_LEN_ADDR);
        for (int i = 0; i < pathLength; i++) {
          path[i] = EEPROM.read(EEPROM_PATH_START + i);
        }
        path[pathLength] = '\0';
        Serial3.print(F("OK:SOLVING|PATH:"));
        Serial3.println(path);
        Serial.print(F("BLE: SOLVING | Path: "));
        Serial.println(path);
        solveIndex = 0;
        currentState = STATE_SOLVING;
      } else {
        Serial3.println(F("ERR:NO_PATH"));
        Serial.println(F("BLE: ERR - No valid path in EEPROM"));
      }
    }
    else if (cmd == 'X') {
      stopMotors();
      if (currentState != STATE_MANUAL) currentState = STATE_MANUAL;
      Serial3.println(F("OK:STOPPED"));
    }
    else if (cmd == 'E') {
      if (EEPROM.read(EEPROM_MAGIC_ADDR) == 'M') {
        int savedPathLen = EEPROM.read(EEPROM_PATH_LEN_ADDR);
        Serial3.print(F("LOG:PATH_LEN:"));
        Serial3.println(savedPathLen);
        Serial3.print(F("LOG:PATH:"));
        for (int i = 0; i < savedPathLen; i++) {
          Serial3.print((char)EEPROM.read(EEPROM_PATH_START + i));
        }
        Serial3.println();

        int savedLogLen = 0;
        EEPROM.get(EEPROM_LOG_LEN_ADDR, savedLogLen);
        Serial3.print(F("LOG:INTERSECTIONS:"));
        for (int i = 0; i < savedLogLen; i++) {
          Serial3.print((char)EEPROM.read(EEPROM_LOG_START + i));
        }
        Serial3.println();
        Serial3.println(F("LOG:END"));
      } else {
        Serial3.println(F("ERR:EEPROM_EMPTY"));
      }
    }

    // --- Manual Movement (Only in MANUAL mode) ---
    else if (currentState == STATE_MANUAL) {
      if      (cmd == 'W') { setMotors(SPEED_FORWARD, SPEED_FORWARD); Serial3.println(F("OK:FWD")); }
      else if (cmd == 'A') { setMotors(SPEED_TURN, SPEED_TURN, true, false); Serial3.println(F("OK:LEFT")); }
      else if (cmd == 'D') { setMotors(SPEED_TURN, SPEED_TURN, false, true); Serial3.println(F("OK:RIGHT")); }
    }
  }
}

// ─────────────────── 7. SENSOR READING ───────────────────
void readSensors(int values[]) {
  int local_readings[5] = {0};
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    local_readings[0] += digitalRead(PIN_SENSORS[0]);
    local_readings[1] += digitalRead(PIN_SENSORS[1]);
    local_readings[2] += digitalRead(PIN_SENSORS[2]);
    local_readings[3] += digitalRead(PIN_SENSORS[3]);
    local_readings[4] += digitalRead(PIN_SENSORS[4]);
    delayMicroseconds(500);
  }
  for (int i = 0; i < 5; i++) {
    values[i] = local_readings[i] > SENSOR_SAMPLES / 2 ? 1 : 0;
  }
}

// ─────────────────── 8. MOTOR CONTROL ───────────────────
void setMotors(int leftSpeed, int rightSpeed, bool leftRev, bool rightRev) {
  leftSpeed  += MOTOR_LEFT_TRIM;
  rightSpeed += MOTOR_RIGHT_TRIM;
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(PIN_MOTOR_LEFT_PWM, leftSpeed);
  digitalWrite(PIN_MOTOR_LEFT_IN1, leftRev ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, leftRev ? LOW  : HIGH);

  analogWrite(PIN_MOTOR_RIGHT_PWM, rightSpeed);
  digitalWrite(PIN_MOTOR_RIGHT_IN3, rightRev ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN4, rightRev ? LOW  : HIGH);
}

void driveForward() { setMotors(SPEED_FORWARD, SPEED_FORWARD); }
void nudgeForward()  { setMotors(SPEED_NUDGE, SPEED_NUDGE); }
void stopMotors()    { setMotors(0, 0); }

bool onLine(int idx, const int values[]) {
  return values[idx] == LOW;
}

void printSensorStatus(const int s[]) { /* kept for compatibility */ }

// ─────────────────── 9. MOVEMENT PRIMITIVES ───────────────────
void turnRight90() {
  Serial3.println(F("ACT:TURN_RIGHT"));
  stopMotors(); delay(50);
  setMotors(SPEED_TURN, SPEED_TURN, false, true);
  delay(250);
  unsigned long turnStart = millis();
  while (millis() - turnStart < MAX_TURN_TIME) {
    int s[NUM_SENSORS]; readSensors(s);
    if (onLine(2, s) && !onLine(3, s) && !onLine(4, s)) break;
    delay(5);
  }
  stopMotors(); delay(50);
}

void turnLeft90() {
  Serial3.println(F("ACT:TURN_LEFT"));
  stopMotors(); delay(50);
  setMotors(SPEED_TURN, SPEED_TURN, true, false);
  delay(250);
  unsigned long turnStart = millis();
  while (millis() - turnStart < MAX_TURN_TIME) {
    int s[NUM_SENSORS]; readSensors(s);
    if (onLine(2, s) && !onLine(0, s) && !onLine(1, s)) break;
    delay(5);
  }
  stopMotors(); delay(50);
}

void turnAround180() {
  Serial3.println(F("ACT:TURN_180"));
  stopMotors(); delay(50);
  setMotors(SPEED_TURN, SPEED_TURN, false, true);
  delay(400);
  unsigned long turnStart = millis();
  while (millis() - turnStart < MAX_TURN_TIME * 2) {
    int s[NUM_SENSORS]; readSensors(s);
    if (s[2] == 0) break;
    delay(5);
  }
  stopMotors(); delay(50);
}

void searchForLine() {
  stopMotors();
  searchStartMillis = millis();
  setMotors(SPEED_SEARCH, SPEED_SEARCH, false, true);
  while (millis() - searchStartMillis < TIME_SEARCH_TOTAL / 2) {
    int s[NUM_SENSORS]; readSensors(s);
    if (onLine(2,s)||onLine(3,s)||onLine(4,s)) { stopMotors(); return; }
    delay(5);
  }
  setMotors(SPEED_SEARCH, SPEED_SEARCH, true, false);
  while (millis() - searchStartMillis < TIME_SEARCH_TOTAL) {
    int s[NUM_SENSORS]; readSensors(s);
    if (onLine(0,s)||onLine(1,s)||onLine(2,s)) { stopMotors(); return; }
    delay(5);
  }
  stopMotors();
}

// ─────────────────── 10. PATH LOGIC ───────────────────
void recordMove(char move) {
  if (pathLength >= 98) return;
  path[pathLength++] = move;
  path[pathLength]   = '\0';
  simplifyPath();
}

void simplifyPath() {
  if (pathLength < 3 || path[pathLength - 2] != 'B') return;
  char a = path[pathLength - 3];
  char c = path[pathLength - 1];
  char rep = 0;
  if      (a=='L' && c=='R') rep='B';
  else if (a=='L' && c=='S') rep='R';
  else if (a=='L' && c=='L') rep='S';
  else if (a=='S' && c=='L') rep='R';
  else if (a=='S' && c=='S') rep='B';
  else if (a=='S' && c=='R') rep='L';
  else if (a=='R' && c=='L') rep='B';
  else if (a=='R' && c=='S') rep='L';
  if (rep != 0) {
    pathLength -= 3;
    path[pathLength++] = rep;
    path[pathLength]   = '\0';
    simplifyPath();
  }
}

// ─────────────────── 11. INTERSECTION DETECTION ───────────────────
IntersectionType_t get_current_intersection() {
  IntersectionType_t local_current_intersection = NONE;
  readSensors(sensor_readings);

  if (sensor_readings[0]==1 && sensor_readings[1]==1 && sensor_readings[2]==1 &&
      sensor_readings[3]==1 && sensor_readings[4]==1) {
    unsigned long deadEndTimer = millis();
    bool falseAlarm = false;
    driveForward();
    while (millis() - deadEndTimer < 300) {
      readSensors(sensor_readings);
      if (sensor_readings[0]==0||sensor_readings[1]==0||sensor_readings[2]==0||
          sensor_readings[3]==0||sensor_readings[4]==0) { falseAlarm = true; break; }
    }
    if (!falseAlarm) { stopMotors(); return DEAD_END; }
  }

  if (sensor_readings[0]==0 && sensor_readings[2]==1) {
    setMotors(SPEED_TURN, SPEED_TURN, true, false);
    unsigned long alignTimer = millis();
    while (sensor_readings[2]==1 && millis()-alignTimer<500) { readSensors(sensor_readings); }
    stopMotors(); delay(40); readSensors(sensor_readings);
  } else if (sensor_readings[4]==0 && sensor_readings[2]==1) {
    setMotors(SPEED_TURN, SPEED_TURN, false, true);
    unsigned long alignTimer = millis();
    while (sensor_readings[2]==1 && millis()-alignTimer<500) { readSensors(sensor_readings); }
    stopMotors(); delay(40); readSensors(sensor_readings);
  }

  if (sensor_readings[0]==1 && sensor_readings[4]==1) return NONE;

  bool flag_is_possible_Plus = false;
  unsigned long crossTimer = millis();

  if (sensor_readings[0] == 0) {
    driveForward();
    while (0 == sensor_readings[0]) {
      readSensors(sensor_readings);
      if (millis() - crossTimer > 300) { stopMotors(); return TARGET; }
      if (0 == sensor_readings[4]) { flag_is_possible_Plus = true; break; }
    }
    if (flag_is_possible_Plus) {
      driveForward();
      while (0==sensor_readings[4] || 0==sensor_readings[0]) {
        readSensors(sensor_readings);
        if (millis()-crossTimer > 300) { stopMotors(); return TARGET; }
      }
      delay(40); readSensors(sensor_readings);
      local_current_intersection = (0==sensor_readings[1]||0==sensor_readings[2]||0==sensor_readings[3]) ? PLUS_INTERSECTION : T_INTERSECTION;
    } else {
      delay(40); readSensors(sensor_readings);
      local_current_intersection = (0==sensor_readings[1]||0==sensor_readings[2]||0==sensor_readings[3]) ? FORWARD_LEFT : LEFT_ELBOW;
    }
  } else if (sensor_readings[4] == 0) {
    driveForward();
    while (0 == sensor_readings[4]) {
      readSensors(sensor_readings);
      if (millis()-crossTimer > 300) { stopMotors(); return TARGET; }
      if (0 == sensor_readings[0]) { flag_is_possible_Plus = true; break; }
    }
    if (flag_is_possible_Plus) {
      driveForward();
      while (0==sensor_readings[0] || 0==sensor_readings[4]) {
        readSensors(sensor_readings);
        if (millis()-crossTimer > 300) { stopMotors(); return TARGET; }
      }
      delay(40); readSensors(sensor_readings);
      local_current_intersection = (0==sensor_readings[1]||0==sensor_readings[2]||0==sensor_readings[3]) ? PLUS_INTERSECTION : T_INTERSECTION;
    } else {
      delay(40); readSensors(sensor_readings);
      local_current_intersection = (0==sensor_readings[1]||0==sensor_readings[2]||0==sensor_readings[3]) ? FORWARD_RIGHT : RIGHT_ELBOW;
    }
  }

  return local_current_intersection;
}

// ─────────────────── 12. EEPROM ───────────────────
void saveDataToEEPROM() {
  Serial3.println(F("OK:SAVING_EEPROM"));
  EEPROM.write(EEPROM_PATH_LEN_ADDR, pathLength);
  for (int i = 0; i < pathLength; i++) EEPROM.write(EEPROM_PATH_START + i, path[i]);
  EEPROM.put(EEPROM_LOG_LEN_ADDR, logCount);
  for (int i = 0; i < logCount; i++) EEPROM.write(EEPROM_LOG_START + i, intersectionLog[i]);
  EEPROM.write(EEPROM_MAGIC_ADDR, 'M');
  Serial3.println(F("OK:EEPROM_SAVED"));
}

// ─────────────────── 13. FINALIZE ───────────────────
void finalizeMaze(bool isTraining) {
  stopMotors(); delay(300);
  if (isTraining) {
    Serial3.print(F("OK:TRAINING_COMPLETE|PATH:"));
    Serial3.println(path);
    Serial.print(F("Training complete. Path: ")); Serial.println(path);
    solveIndex = 0;
    delay(3000);
  } else {
    Serial3.println(F("OK:MAZE_SOLVED"));
    Serial.println(F("MAZE SOLVED!"));
    currentState = STATE_DONE;
  }
}

// ─────────────────── 14. TRAINING ───────────────────
void doTraining() {
  int s[NUM_SENSORS];
  readSensors(s);
  IntersectionType_t t = get_current_intersection();

  if      (t == T_INTERSECTION)    { Serial3.println(F("INT:T"));     recordMove('L'); logIntersection('T'); turnLeft90(); }
  else if (t == PLUS_INTERSECTION) { Serial3.println(F("INT:PLUS"));  recordMove('L'); logIntersection('P'); turnLeft90(); }
  else if (t == LEFT_ELBOW)        { Serial3.println(F("INT:L_ELB")); recordMove('L'); logIntersection('l'); turnLeft90(); }
  else if (t == RIGHT_ELBOW)       { Serial3.println(F("INT:R_ELB")); recordMove('R'); logIntersection('r'); turnRight90(); }
  else if (t == FORWARD_LEFT)      { Serial3.println(F("INT:FWD_L")); recordMove('L'); logIntersection('F'); turnLeft90(); }
  else if (t == FORWARD_RIGHT)     { Serial3.println(F("INT:FWD_R")); recordMove('S'); logIntersection('f'); driveForward(); }
  else if (t == DEAD_END)          { Serial3.println(F("INT:DEAD"));  recordMove('B'); logIntersection('D'); turnAround180(); }
  else if (t == TARGET)            { Serial3.println(F("INT:TARGET")); logIntersection('E'); finalizeMaze(true); saveDataToEEPROM(); return; }

  bool fl=onLine(0,s), l=onLine(1,s), c=onLine(2,s), r=onLine(3,s), fr=onLine(4,s);
  if      (c && !l && !fl && !r && !fr)  driveForward();
  else if ((l||fl) && !r && !fr)         setMotors(SPEED_TURN, SPEED_TURN, true, false);
  else if ((r||fr) && !l && !fl)         setMotors(SPEED_TURN, SPEED_TURN, false, true);
  else                                   driveForward();
}

// ─────────────────── 15. SOLVING ───────────────────
void doSolving() {
  int s[NUM_SENSORS];
  readSensors(s);
  IntersectionType_t t = get_current_intersection();

  if (t == TARGET) { finalizeMaze(false); return; }

  if (t == T_INTERSECTION || t == PLUS_INTERSECTION ||
      t == LEFT_ELBOW     || t == RIGHT_ELBOW       ||
      t == FORWARD_LEFT   || t == FORWARD_RIGHT) {

    if (solveIndex >= pathLength) {
      Serial3.println(F("ERR:PATH_ENDED_EARLY"));
      stopMotors(); currentState = STATE_DONE; return;
    }

    char nextMove     = path[solveIndex];
    char moveToExecute = 'S';
    bool consumePath  = false;

    if      (t == LEFT_ELBOW)    { moveToExecute='L'; if(nextMove=='L') consumePath=true; }
    else if (t == RIGHT_ELBOW)   { moveToExecute='R'; if(nextMove=='R') consumePath=true; }
    else if (t == FORWARD_LEFT)  { if(nextMove=='L'||nextMove=='S'){moveToExecute=nextMove;consumePath=true;}else moveToExecute='S'; }
    else if (t == FORWARD_RIGHT) { if(nextMove=='R'||nextMove=='S'){moveToExecute=nextMove;consumePath=true;}else moveToExecute='S'; }
    else if (t == T_INTERSECTION){ if(nextMove=='L'||nextMove=='R'){moveToExecute=nextMove;consumePath=true;}else moveToExecute='R'; }
    else if (t == PLUS_INTERSECTION){ moveToExecute=nextMove; consumePath=true; }

    if (consumePath) {
      solveIndex++;
      Serial3.print(F("SOLVE:EXEC:")); Serial3.println(moveToExecute);
    }
    if      (moveToExecute=='L') turnLeft90();
    else if (moveToExecute=='R') turnRight90();
    else                         driveForward();
    return;
  }

  bool fl=onLine(0,s), l=onLine(1,s), c=onLine(2,s), r=onLine(3,s), fr=onLine(4,s);
  if      (c && !l && !fl && !r && !fr)  driveForward();
  else if ((l||fl) && !r && !fr)         setMotors(SPEED_TURN, SPEED_TURN, true, false);
  else if ((r||fr) && !l && !fl)         setMotors(SPEED_TURN, SPEED_TURN, false, true);
}