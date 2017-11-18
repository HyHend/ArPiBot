#include <Servo.h> 

Servo ultrasonicSensorServo;

/**
 * Serial "protocol":
 * 
 * Sends (ultrasonic measurement in cm at angle):
 * - USM (int)angle (int)measuredDistanceCM
 *  
 * Receives (M=Motordrive, S=SensorAngle, SB=SensorAngleBetween):
 * - M (int)velocity (int)rotation
 * - S (int)angle
 * // - SB (int)angleFrom (int)angleTo (boolean)lookAround
 */

// Pin assignments
int ECHO = A4;
int TRIG = A5;
int M0 = 6;
int M1 = 7;
int M2 = 8;
int M3 = 9;
int ENA = 5;
int ENB = 11;

// Current ultrasonic sensor location
int ultrasonicSensorServoAngle;
bool ultrasonicSensorServoAngleIncreasing = false;
int requiredUltrasonicSensorServoAngle = 90;

// Array with ultrasonic measured distances (6 degree buckets)
//int ultrasonicDistances[30] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
//                              1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

// Minimal moving time for servo
int servoMinMovingTime = 10;

// Loop times
int motorLoopMillis = 100;
unsigned long previousMotorLoopMillis = 0;
int sensorAngleLoopMillis = 50;
unsigned long previousSensorAngleLoopMillis = 0;
int ultrasonicSensorLoopMillis = 14;    // 180deg, 420ms, 14ms, 30 measurements 
unsigned long previousUltrasonicSensorLoopMillis = 0;

// Start of us measurement
unsigned long ultrasonicMeasureStart = 0;
unsigned long ultrasonicResultStart = 0;
int ultrasonicTimeoutUs = 11658;   //201cm * 58 (= us the roundtrip takes)

// Sensor is moving, ends at:
unsigned long ultrasonicSensorServoMovingEnd = 0;

// Remember current velocity, rotation
int currentVelocity = 0;
int currentRotation = 0;

/**
 * Drive forward
 *   int velocity: -255-255 (-255=full back, 0=stop, 255=full forward)
 *   int rotation: -255-255 (-255=full left, 0=straight, 255=full right)
 */
void _mDrive(int velocity, int rotation)
{
  if(currentVelocity == velocity
      && currentRotation == rotation) {
    return;
  }
  currentVelocity = velocity;
  currentRotation = rotation;
  
  int lV; 
  int rV;

   // Depending on rotation, calculate lV, rV
  if(rotation < 0) {
    // Left
    int lRotationDrive = 255 - (abs(rotation) * 2);   //(-) 0-127 results in 0-255, (-) 127-255 results in -255 - 0
    lV = (int) round((((float) velocity/255)*((float) lRotationDrive/255))*255);   // Scale according to velocity
    rV = velocity;   // The left side is steering
  }
  else if(rotation > 0) {
    // Right
    lV = velocity;   // The right side is steering
    int rRotationDrive = 255 - (abs(rotation) * 2);   //(-) 0-127 results in 0-255, (-) 127-255 results in -255 - 0
    rV = (int) round((((float) velocity/255)*((float) rRotationDrive/255))*255);   // Scale according to velocity
  }
  else {
    // Straight
    lV = velocity;
    rV = velocity;
  }
  
  _mDriveSub(lV, rV);
}

/**
 * Drives motors directly.
 * int lV: left motor speed -255-255 (<0 = backwards, >0 = forwards, 0 = stop)
 * int rV: right motor speed -255-255 (<0 = backwards, >0 = forwards, 0 = stop)
 */
void _mDriveSub(int lV, int rV) 
{
  // Set speeds
  analogWrite(ENA,abs(lV));
  analogWrite(ENB,abs(rV));
  
  // Left motor
  if(lV < 0) {
    // Backwards
    digitalWrite(M0,LOW);
    digitalWrite(M1,HIGH);
  }
  else if(lV > 0) {
    // Forwards
    digitalWrite(M0,HIGH);
    digitalWrite(M1,LOW);
  }
  else {
    // Stop
    digitalWrite(ENA,LOW);
  }

  // Right motor
  if(rV < 0) {
    // Backwards
    digitalWrite(M2,HIGH);
    digitalWrite(M3,LOW);
  }
  else if(rV > 0) {
    // Forwards
    digitalWrite(M2,LOW);
    digitalWrite(M3,HIGH);
  }
  else {
    // Stop
    digitalWrite(ENB,LOW);
  }
}

/**
 * Aims ultrasonic sensor
 * int angle: 0-180, left to right
 * Returns time it'll probably take to aim
 */
int moveUltrasonicSensorServoToAngle(int angle) 
{
  if(angle < 0) { angle = 0; }     // Fix for sensor min location
  if(angle > 180) { angle = 180; } // Fix for sensor max location

  int movingDistance = abs(angle - ultrasonicSensorServoAngle);
  
  // Angle increasing or decreasing?
  ultrasonicSensorServoAngleIncreasing = angle > ultrasonicSensorServoAngle;
  
  // Move servo
  ultrasonicSensorServo.write(angle);
  ultrasonicSensorServoAngle = angle;

  // Calculate, return waiting time
  int wait = (int) round(servoMinMovingTime + movingDistance * 2.5);
  return wait;
}

/**
 * Moves sensor between minDeg and maxDeg
 * Note, only when called in loop
 * Looks around 1 in x (randomly) times when lookAround == true
 */
void moveUltrasonicSensorBetween(int minDeg, int maxDeg, unsigned long currentMillis, bool lookAround) {
  if(lookAround && random(0, 4) == 1) {
    if(currentMillis >= ultrasonicSensorServoMovingEnd) {
      if(maxDeg < 90){
        requiredUltrasonicSensorServoAngle = 160;
      }
      else {
        requiredUltrasonicSensorServoAngle = 20;
      }
    }
  }
  else {
    if(currentMillis >= ultrasonicSensorServoMovingEnd
        && requiredUltrasonicSensorServoAngle < maxDeg) {
      requiredUltrasonicSensorServoAngle = maxDeg;
    }
    else if(currentMillis >= ultrasonicSensorServoMovingEnd
        && requiredUltrasonicSensorServoAngle > minDeg) {
      requiredUltrasonicSensorServoAngle = minDeg;
    }
  }
}

/**
 * On Init
 */
void setup() 
{ 
  ultrasonicSensorServo.attach(3);  // Attach servo on pin 3
  Serial.begin(9600);
  Serial.setTimeout(3);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(M0,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  // Initial 1s delay to allow robot to be put on ground ;)
  delay(1000);
} 

/** 
 *  getValue("a b c", " ", 1) will return a
 *  getValue("a b c", " ", 2) will return b
 *  https://stackoverflow.com/questions/9072320/split-string-into-string-array
 **/
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i) == separator || i == maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
 * Continuously called on runtime
 * 
 * Serial 
 * - M (int)velocity (int)rotation
 * - S (int)angle
 * // - SB (int)angleFrom (int)angleTo (boolean)lookAround
 */
void loop() 
{ 
  // Get current millis, micros
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();
  
  // Motor runtime
  if((previousMotorLoopMillis + motorLoopMillis) < currentMillis) {
    previousMotorLoopMillis = currentMillis;

    // Read serial
    String serRec = Serial.readString();
    if(serRec.length() > 0){
      Serial.println(serRec);   // Echo back

      if(serRec.startsWith("M")) {
        // Received Motor command
        int velocity = getValue(serRec, ' ', 1).toInt();
        int rotation = getValue(serRec, ' ', 2).toInt();
        _mDrive(velocity, rotation);
      }
//      else if(serRec.startsWith("SB")) {
//        // Received Servo (between) command
//        int minDeg = getValue(serRec, ' ', 1).toInt();
//        int maxDeg = getValue(serRec, ' ', 2).toInt();
//        boolean lookAround = (boolean) getValue(serRec, " ", 3).toInt();
//        moveUltrasonicSensorBetween(minDeg, maxDeg, currentMillis, lookAround);
//      }
      else if(serRec.startsWith("S")) {
        // Received Servo command
        int angle = getValue(serRec, ' ', 1).toInt();
        requiredUltrasonicSensorServoAngle = angle;
      }
    }
    // Act on (serial) received commands
    //TODO motor
    //TODO ultrasonicSensor
  }
//  if((previousMotorLoopMillis + motorLoopMillis) < currentMillis) {
//    previousMotorLoopMillis = currentMillis;
//
//    // Runtime
//    // Get closest distance
//    int lowest = 0;
//    for(int i = 0; i < 30; i++) {
//      if(ultrasonicDistances[i] < ultrasonicDistances[lowest]) {
//        lowest = i;
//      }
//    }
//
//    // Define distances
//    int veryNear = 15;
//    int near = 30;
//    int medNear = 75;
//
//    // Follow nearest object with ultrasonic sensor
//    if(ultrasonicDistances[lowest] > medNear) {
//      // Rotate sensor
//      moveUltrasonicSensorBetween(0, 180, currentMillis, false);
//    }
//    else if(ultrasonicDistances[lowest] > near
//        && ultrasonicDistances[lowest] <= medNear) {
//      // Rotate sensor
//      int minDeg = max(0, (lowest*6) - 54);
//      int maxDeg = min(180, (lowest*6) + 54);
//    }
//    else if(ultrasonicDistances[lowest] <= near) {
//      // Drive
//      int rotation = 0;
//      int velocity = 130;
//      _mDrive(velocity, rotation);
//      
//      // Rotate sensor
//      int minDeg = max(0, (lowest*6) - 24);
//      int maxDeg = min(180, (lowest*6) + 24);
//      moveUltrasonicSensorBetween(minDeg, maxDeg, currentMillis, true);
//    }
//
//    // Steer clear, but folow objects
//    // If ahead is not possible, go back
//    int velocity;
//    int rotation;
//    boolean aheadVeryNearClear = true;
//    boolean aheadNearClear = true;
//    boolean aheadMedNearClear = true;
//    for(int i = 11; i <= 18; i++) {
//      if(ultrasonicDistances[i] < veryNear) {
//        aheadVeryNearClear = false;
//      }
//      if(ultrasonicDistances[i] < near) {
//        aheadNearClear = false;
//      }
//      if(ultrasonicDistances[i] < medNear) {
//        aheadMedNearClear = false;
//      }
//    }
//
//    // If there's an object ahead, right in front, deal with it NOW
//    if(!aheadVeryNearClear) {
//      // Is right or left clear?, go back in that direction
//      int rightMinDistance = 100;
//      int leftMinDistance = 100;
//      for(int i = 0; i < 5; i++) {
//        if(ultrasonicDistances[i] < rightMinDistance) {
//          rightMinDistance = ultrasonicDistances[i];
//        }
//      }
//      for(int i = 25; i < 30; i++) {
//        if(ultrasonicDistances[i] < leftMinDistance) {
//          leftMinDistance = ultrasonicDistances[i];
//        }
//      }
//
//      // Choose free direction
//      if(rightMinDistance > near
//          && rightMinDistance > leftMinDistance) {
//        velocity = -140;
//        rotation = 127 + max((int) round(rightMinDistance / 4), 100);
//      }
//      else if(leftMinDistance > near
//        && rightMinDistance <= leftMinDistance) {
//        velocity = -140;
//        rotation = -127 - max((int) round(leftMinDistance / 4), 100);
//      }
//      else {
//        velocity = -90;
//        rotation = 0;
//      }
//    }
//    // Elseif is temporarily (TODO)
//    else if(currentVelocity > 0 
//        || aheadMedNearClear) {
//      velocity = 76;
//      rotation = 0;
//    }
//    else if(currentVelocity > 0 
//        || aheadNearClear) {
//      velocity = 40;
//      rotation = 0;
//    }
//    _mDrive(velocity, rotation);
//
//    // At random moments, print distances
//    if(random(0, 20) == 7) {
//      Serial.print("Dst: [");
//      for(int i = 0; i < 30; i++) {
//        Serial.print(ultrasonicDistances[i]);
//        if(i < 29){ Serial.print(", "); };
//      }
//      Serial.println("]");
//    }
//  }

  // Ultrasonic Sensor angle/init runtime
  if((previousSensorAngleLoopMillis + sensorAngleLoopMillis) < currentMillis) {
    previousSensorAngleLoopMillis = currentMillis;

    // Runtime
    if(currentMillis > ultrasonicSensorServoMovingEnd
      && ultrasonicSensorServoAngle != requiredUltrasonicSensorServoAngle) {
      // Sensor is not currently being moved, position is different than current. Move to new position
      int wait;
      wait = moveUltrasonicSensorServoToAngle(requiredUltrasonicSensorServoAngle);
    }
  }
  
  // Ultrasonic Sensor runtime
  if((previousUltrasonicSensorLoopMillis + ultrasonicSensorLoopMillis) < currentMillis) {
    previousUltrasonicSensorLoopMillis = currentMillis;

    // Runtime
    if(ultrasonicMeasureStart <= 0) {
      // No measurement busy yet, start one (ultrasonic ping)
      ultrasonicMeasureStart = currentMicros;
      digitalWrite(TRIG, HIGH);
    }
  }

  // Ultrasonic Sensor (ping end) handler
  if(ultrasonicMeasureStart - currentMicros > 20 
      && digitalRead(TRIG) == HIGH) {
    // After 20 us, pull trigger low again (end ultrasonic ping)
    digitalWrite(TRIG, LOW);
  }

  // Ultrasonic Sensor (ping return) handler
  if(ultrasonicMeasureStart > 20
     && ultrasonicResultStart <= 0 
     && digitalRead(ECHO) == HIGH) {
    // Start (up) edge of return signal
    ultrasonicResultStart = currentMicros;
  }

  // Ultrasonic Sensor (ping return) handler
  if(ultrasonicMeasureStart > 20
     && ultrasonicResultStart > 0 
     && (digitalRead(ECHO) == LOW
     || (currentMicros - ultrasonicResultStart) > ultrasonicTimeoutUs)) {
    // End (down) edge of return signal, calculate result
    float ultrasonicMeasureDistance = (currentMicros - ultrasonicResultStart) / 58;
    
    // Use current angle of sensor, save in correct position in array (use 180/30, 6 degree wide buckets, calculate in what bucket and save)
    // (ultrasonicSensorServoMovingEnd - currentmillis) / 2,5 = degrees from angle (at what side?)
    int currentAngle = ultrasonicSensorServoAngle;
    if(ultrasonicSensorServoMovingEnd > currentMillis) {
      int diff = (int) round(max(0, (ultrasonicSensorServoMovingEnd - currentMillis) / 2.5));
      if(ultrasonicSensorServoAngleIncreasing) {
        currentAngle = ultrasonicSensorServoAngle - diff;
      }
      else {
        currentAngle = ultrasonicSensorServoAngle + diff;
      }
    }

//    // Save measurement in correct bucket
    int currentBucket = (int) round(min(currentAngle, 180) / 6);  // 30 buckets of 6 degrees width
//    if(ultrasonicMeasureDistance <= 200) {
//      // Note: A high distance probably indicates an incorrect measurement (TODO or no obstacles at all?)
//      //       For example, there could be sound absorbing material in front of the sensor
//      // .     We don't save these large values (TODO is this the best solution?)
//      ultrasonicDistances[currentBucket] = ultrasonicMeasureDistance;
//    }

    // Send current bucket measurement over serial
    // USM (int)angle (int)measuredDistanceCM
    Serial.print("USM ");
    Serial.print(currentMillis);
    Serial.print(" ");
    Serial.print((currentBucket*6));
    Serial.print(" ");
    Serial.println((int) ultrasonicMeasureDistance);

    // RST
    ultrasonicMeasureStart = 0;
    ultrasonicResultStart = 0;
  }                   
}

