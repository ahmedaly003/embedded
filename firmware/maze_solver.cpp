cpp
// ============================================================
// Maze Solver - Phase 0 (FIXED - No more stopping at simple turns)
// Arduino Mega 2560 | ECNG 4504
// ============================================================

const int pinLeftPWM  = 9;  const int pinLeftIN1  = 8;  const int pinLeftIN2  = 7;
const int pinRightPWM = 3;  const int pinRightIN3 = 5;  const int pinRightIN4 = 4;

const int numSensors = 5;
const int pinSensors[numSensors] = {6, 2, 12, 11, 10};

// ─────────────────── TUNABLES ───────────────────
const int speedForward    = 160;
const int speedNudge      = 80;
const int speedGentleTurn = 80;
const int speedSharpTurn  = 160;

const unsigned long timeTurn90      = 430;
const unsigned long timeNudge       = 65;
const unsigned long timeoutTraining = 300000;

// ─────────────────── STATE & PATH ───────────────────
enum State { TRAINING, SOLVING, DONE };
State robotState = TRAINING;

char path[100];
int pathLength = 0;
int solveIndex = 0;
unsigned long trainingStartMillis = 0;

// ─────────────────── SETUP ───────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("=== MAZE SOLVER - FIXED SIMPLE TURNS ===");
  
  for (int i = 0; i < numSensors; i++) pinMode(pinSensors[i], INPUT);
  pinMode(pinLeftPWM, OUTPUT); pinMode(pinLeftIN1, OUTPUT); pinMode(pinLeftIN2, OUTPUT);
  pinMode(pinRightPWM, OUTPUT); pinMode(pinRightIN3, OUTPUT); pinMode(pinRightIN4, OUTPUT);
  
  stopMotors();
  trainingStartMillis = millis();
  Serial.println("▶️ Starting TRAINING...");
  delay(1000);
}

// ─────────────────── MOTOR (correct forward) ───────────────────
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0)  { digitalWrite(pinLeftIN1, LOW);  digitalWrite(pinLeftIN2, HIGH); analogWrite(pinLeftPWM, leftSpeed); }
  else if (leftSpeed < 0) { digitalWrite(pinLeftIN1, HIGH); digitalWrite(pinLeftIN2, LOW);  analogWrite(pinLeftPWM, -leftSpeed); }
  else { digitalWrite(pinLeftIN1, LOW); digitalWrite(pinLeftIN2, LOW); analogWrite(pinLeftPWM, 0); }

  if (rightSpeed > 0) { digitalWrite(pinRightIN3, LOW);  digitalWrite(pinRightIN4, HIGH); analogWrite(pinRightPWM, rightSpeed); }
  else if (rightSpeed < 0) { digitalWrite(pinRightIN3, HIGH); digitalWrite(pinRightIN4, LOW);  analogWrite(pinRightPWM, -rightSpeed); }
  else { digitalWrite(pinRightIN3, LOW); digitalWrite(pinRightIN4, LOW); analogWrite(pinRightPWM, 0); }
}

void driveForward()    { setMotors(speedForward, speedForward); }
void nudgeForward()    { setMotors(speedNudge, speedNudge); }
void turnLeftGentle()  { setMotors(60, speedForward); }
void turnRightGentle() { setMotors(speedForward, 60); }
void stopMotors()      { setMotors(0, 0); }

void turnLeft90() {
  stopMotors(); delay(40);
  setMotors(-speedSharpTurn, speedSharpTurn);
  delay(timeTurn90);
  stopMotors(); delay(70);
}

void turnRight90() {
  stopMotors(); delay(40);
  setMotors(speedSharpTurn, -speedSharpTurn);
  delay(timeTurn90);
  stopMotors(); delay(70);
}

void turnAround180() { turnRight90(); delay(100); turnRight90(); }

// ─────────────────── SENSOR READING ───────────────────
void readSensors(int values[]) {
  for (int i = 0; i < numSensors; i++) {
    int blackCount = 0;
    for (int j = 0; j < 3; j++) {
      if (digitalRead(pinSensors[i]) == LOW) blackCount++;
      delayMicroseconds(150);
    }
    values[i] = (blackCount >= 2) ? LOW : HIGH;
  }
}
bool onLine(int idx, const int values[]) { return values[idx] == LOW; }

// ─────────────────── PATH LOGIC ───────────────────
void recordMove(char move) {
  if (pathLength >= 98) return;
  path[pathLength++] = move; path[pathLength] = '\0';
  simplifyPath();
}
void simplifyPath() {
  if (pathLength < 3 || path[pathLength-2] != 'B') return;
  char a = path[pathLength-3], c = path[pathLength-1], rep = 0;
  if (a=='L' && c=='R') rep='B'; else if (a=='L' && c=='S') rep='R';
  else if (a=='L' && c=='L') rep='S'; else if (a=='S' && c=='L') rep='R';
  else if (a=='S' && c=='S') rep='B'; else if (a=='S' && c=='R') rep='L';
  else if (a=='R' && c=='L') rep='B'; else if (a=='R' && c=='S') rep='L';
  if (rep) { pathLength -= 3; path[pathLength++] = rep; path[pathLength] = '\0'; simplifyPath(); }
}

// ─────────────────── COMPLEX INTERSECTION ONLY (T / Cross) ───────────────────
void handleComplexIntersection(bool isTraining) {
  Serial.println("Complex Intersection");
  nudgeForward(); delay(timeNudge); stopMotors(); delay(40);
  
  int s[numSensors]; readSensors(s);
  bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);
  
  if (!fl && !l && !c && !r && !fr) {
    Serial.println("🏁 END OF MAZE");
    if (isTraining) { robotState = SOLVING; solveIndex = 0; }
    else robotState = DONE;
    return;
  }
  
  char decision = 'S';
  if (isTraining) {
    if (fl || l)      decision = 'L';
    else if (fr || r) decision = 'R';
    else if (c)       decision = 'S';
    else              decision = 'B';
    recordMove(decision);
  } else if (solveIndex < pathLength) {
    decision = path[solveIndex++];
  }
  
  if (decision == 'L') turnLeft90();
  else if (decision == 'R') turnRight90();
  else if (decision == 'B') turnAround180();
}

// ─────────────────── SEARCH FOR LINE ───────────────────
void searchForLine() {
  Serial.println("🔍 Searching...");
  stopMotors();
  unsigned long start = millis();

  // Sweep left - wide radius
  setMotors(0, speedSharpTurn);
  while (millis() - start < 2000) {
    int s[numSensors]; readSensors(s);
    if (onLine(0,s) || onLine(4,s)) { stopMotors(); return; }
  }

  // Sweep right - wide radius
  setMotors(speedSharpTurn, 0);
  while (millis() - start < 4000) {
    int s[numSensors]; readSensors(s);
    if (onLine(0,s) || onLine(4,s)) { stopMotors(); return; }
  }

  // Couldn't find line — turn 180
  stopMotors();
  Serial.println("⚠ Line lost — turning 180.");
  recordMove('B');
  turnAround180();
}

// ─────────────────── TRAINING ───────────────────
void doTraining() {
  int s[numSensors]; readSensors(s);
  bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);
  
  if (fl && c) {
    recordMove('L');
    turnLeft90();
    return;
  }
  
  if (fr && c) {
    recordMove('R');
    turnRight90();
    return;
  }
  
  if (fl && l && c && r && fr) {
    handleComplexIntersection(true);
    return;
  }
  
  if (!fl && !l && !c && !r && !fr) {
    searchForLine();
    return;
  }
  
  if (c) driveForward();
  else if (l || fl) turnLeftGentle();
  else if (r || fr) turnRightGentle();
  else searchForLine();
}

// ─────────────────── SOLVING ───────────────────
void doSolving() {
  int s[numSensors]; readSensors(s);
  bool fl = onLine(0,s), l = onLine(1,s), c = onLine(2,s), r = onLine(3,s), fr = onLine(4,s);
  
  if (fl && l && c) { if (solveIndex < pathLength && path[solveIndex] == 'L') { solveIndex++; turnLeft90(); } return; }
  if (fr && r && c) { if (solveIndex < pathLength && path[solveIndex] == 'R') { solveIndex++; turnRight90(); } return; }
  
  if (fl && l && c && r && fr) { handleComplexIntersection(false); return; }
  
  if (!fl && !l && !c && !r && !fr) { turnAround180(); return; }
  
  if (c) driveForward();
  else if (l || fl) turnLeftGentle();
  else if (r || fr) turnRightGentle();
  else searchForLine();
}

// ─────────────────── LOOP ───────────────────
void loop() {
  if (robotState == TRAINING && millis() - trainingStartMillis > timeoutTraining) {
    Serial.println("⚠ Timeout"); stopMotors(); robotState = DONE;
  }
  
  if (robotState == TRAINING) doTraining();
  else if (robotState == SOLVING) doSolving();
  else stopMotors();
  
  delay(8);
}