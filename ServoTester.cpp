#include "ServoTester.h"
#include <avr/sleep.h>

ServoTester::ServoTester(int8_t aServoPin, int8_t aPotiPin, int8_t aGreenLedPin, int8_t aRedLedPin)
 : myBatterySupervisionPin(SERVO_TESTER_NOTUSED),
   myModeControlPin(SERVO_TESTER_NOTUSED),
   myDefaultFunction(SERVO_TESTER_FUNC_POSITION_TEST)
{
  myLedGreen = new StatusLED(aGreenLedPin);
  myLedRed = new StatusLED(aRedLedPin);
  myServo = new Servo();
  myServoPin = aServoPin;
  myPotiPin = aPotiPin;
  myState = S_UNSET;
}

void ServoTester::init() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("init()");
  Serial.println();
  #endif
  Serial.print("ServoTester ");
  Serial.print(" - Startup");
  Serial.println();
  // attaches the servo on pin to the servo object
  Servo::setRefreshFreq(50);
  myServo->attach(myServoPin);
  pinMode(myPotiPin, INPUT);
  uint16_t usStep               = 500/4; // 25% Steps from as -125%,-100%,-75%,-50%,-25%,0%,25%,50%,75%,100%,125%
  myServoPosInUsMin      = 1000; // - usStep;
  myServoPosInUsMax      = 2000; // + usStep;
  myCheckCnt = 0;
  myCheckError = 0;
  myTestCnt = 0;

  randomSeed(analogRead(-1)); // initializes the pseudo-random number generator

  // which function should be run
  if (myModeControlPin != SERVO_TESTER_NOTUSED && !digitalRead(myModeControlPin)) {
    myDefaultFunction = SERVO_TESTER_FUNC_FOLLOWS_POTI;
  } else {
    myDefaultFunction = SERVO_TESTER_FUNC_POSITION_TEST;
  }

  switch (myDefaultFunction) {
    case SERVO_TESTER_FUNC_POSITION_TEST:
      Serial.print("ServoTester - Mode-0 - Servo wird automatisch über Sensor überprüft");
      Serial.println();
      myInitDone = initServoPositionTest();
      break;
    case SERVO_TESTER_FUNC_FOLLOWS_POTI:
      Serial.print("ServoTest - Mode-1 - Servo wird über das Poti / den Sensor gesteuert");
      Serial.println();
      myInitDone = initServoFollowsPoti();
      break;
  }

}

bool ServoTester::initServoFollowsPoti() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("initServoFollowsPoti()");
  Serial.println();
  #endif
  return true;
}

void ServoTester::setState(ServoTester::State aState) {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG: setState(): ");
  Serial.print(aState);
  Serial.println();
  #endif
  if (myState == aState) return;
  myState = aState;
  switch (myState) {
    case S_ERROR_SENSOR:
      // Serial.println("DEBUG: S_ERROR_SENSOR");
      myLedGreen->off();
      myLedRed->off();
      myLedGreen->blink(300);
      delay(300);
      myLedRed->blink(300);
      break;
    case S_INIT:
      // Serial.println("DEBUG: S_INIT");
      myLedGreen->off();
      myLedRed->off();
      myLedGreen->blink(300);
      myLedRed->blink(300);
      break;
    case S_ERROR_INIT:
      // Serial.println("DEBUG: S_ERROR_INIT");
      myLedGreen->off();
      myLedRed->off();
      myLedGreen->blink(1500);
      break;
    case S_ERROR_BATT:
      // Serial.println("DEBUG: S_ERROR_BATT");
      myLedGreen->off();
      myLedRed->off();
      myLedGreen->blinka(100, 300);
      break;
    case S_RUNNING:
      // Serial.println("DEBUG: S_RUNNING");
      myLedRed->off();
      myLedGreen->on();
      break;
    case S_FINISHED:
      // Serial.println("DEBUG: S_FINISHED");
      myLedGreen->blinka(3000, 500);
      break;
    case S_SHUTDOWN:
      // Serial.println("DEBUG: S_SHUTDOWN");
      myLedGreen->off();
      myLedRed->off();
      myLedGreen->blinka(100, 2000);
      break;
  }
}

bool ServoTester::initServoPositionTest() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("initServoPositionTest()");
  Serial.println();
  #endif
  setState(S_INIT);

  myServo->writeMicroseconds(1500);
  checkBatteryVoltage(0);

  initServoPositionTest_ZeroPos();
  calibratePoti();
  servoSpeedTest();
  return true;
}

void ServoTester::initServoPositionTest_ZeroPos() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("initServoPositionTest_ZeroPos()");
  Serial.println();
  #endif
  uint16_t servoPos;
  uint16_t lastServoPos = 1000;
  Serial.print("Prüfung Servo-Positionssensor in Servo-Mittelposition ...");
  Serial.println();

  // do the middle position and moving test n time
  for (int i=0; i<3;i++) {

    myServo->writeMicroseconds(1500);
    delay(1000);

    while(true) {
      // check 0-Position
      servoPos = getServoPositionSensorValue();
      int differenceFromMiddle = abs(512-servoPos);
      if (abs(servoPos-lastServoPos) > 2 || differenceFromMiddle < 30 ) {

        if (differenceFromMiddle > 30) {
          int b = 500 - differenceFromMiddle;
          b = b*2;
          Serial.print("Fehler Mittelposition (");
          Serial.print(abs(512-servoPos));
          Serial.print("): bitte Servohebel zentrieren ... ");
          Serial.print(b);
          Serial.println();
          setState(S_ERROR_SENSOR);
          // delay(10000);
          // return false;
        } else {
          // Serial.print("Mittelposition (");
          // Serial.print(abs(512-servoPos));
          // Serial.print("): erreicht ");
          // Serial.println();
          setState(S_INIT);
          delay(5000);
          Serial.print("Mittelposition erreicht ...");
          Serial.print("warte 5s");
          Serial.println();
          if (initServoPositionTest_CheckPosSensor()) {
            // Serial.print("Sensorprüfung ok");
            // Serial.println();
            delay(500);
            return;
          }
          Serial.print("ERROR: Sensorprüfung fehlgeschlagen");
          Serial.print("... wiederhole Mittelpositionsdetektion und Sensorprüfung");
          Serial.println();
          delay(2000);
        }
        lastServoPos = servoPos;
      }
      delay(500);
    }
  }
}

bool ServoTester::initServoPositionTest_CheckPosSensor() {
  bool retVal = true;
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("initServoPositionTest_CheckPosSensor()");
  Serial.println();
  #endif
  int servoPos;
  servoPos = getServoPositionSensorValue();

  // check poti movement
    // Serial.print("Prüfe Positionssensor ... ");
    // Serial.println();

    uint8_t sensorDoesNotMove=0;
    for (int idx = 0 ; idx < 5; idx++) {
      myServo->writeMicroseconds(1450);
      delay(500);
      uint16_t lastSensorValue = 0;
      for (int pos=1450; pos < 1551; pos+=20) {
        myServo->writeMicroseconds(pos);
        delay(100);
        uint16_t sensorValue = getServoPositionSensorValue();
        Serial.print("DEBUG: Positionsensor: ");
        Serial.print("");
            Serial.print(pos);
            Serial.print("/");
            Serial.print(sensorValue);
            Serial.println();
        if (abs(sensorValue - lastSensorValue) < 1) {
          sensorDoesNotMove++;
          if (sensorDoesNotMove > 5) {
            setState(S_ERROR_SENSOR);
            Serial.print("ERROR: Positionsensor bewegt sich nicht korrekt");
            Serial.print(" (");
            Serial.print(sensorValue);
            Serial.print("/");
            Serial.print(lastSensorValue);
            Serial.print(")");
            Serial.println();
            retVal = false;
          }
        }
        lastSensorValue = sensorValue;
        switch (pos) {
          case 1450:
             if (!comparePositionData(0, idx, sensorValue, 5)) {
               retVal = false;
             }
             break;
          case 1500:
             if (!comparePositionData(1, idx, sensorValue, 5)) {
               retVal = false;
             }
          break;
          case 1550:
             if (!comparePositionData(2, idx, sensorValue, 5)) {
               retVal = false;
             }
          break;
        }

        if ( false == retVal ) {
          break;
        }
      }
      if ( false == retVal ) {
        break;
      }
    }
  return retVal;
}

void ServoTester::calibratePoti() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("calibratePoti()");
  Serial.println();
  #endif
  Serial.println("Kalibrierung des Poti-Weges:");
  myServo->writeMicroseconds(900);
  delay(500);
  for (int i=0; i<10;i++) {
    myServo->writeMicroseconds(910+i*10);
    delay(100);
  }
  myServo->writeMicroseconds(1000);
  for (int i = 0 ; i < 15; i++) {
    delay(100);
  }
  myPotiMinPos = getServoPositionSensorValue();

  Serial.print("   Servo-Signal: ");
  Serial.print(myServo->readMicroseconds());
  Serial.print("us ");
  Serial.print(" ergibt min. Poti-Wert: ");
  Serial.print(myPotiMinPos);
  Serial.println();

  myServo->writeMicroseconds(1900);
  delay(500);
  for (int i=0; i<10;i++) {
    myServo->writeMicroseconds(1910+i*10);
    delay(100);
  }
  myServo->writeMicroseconds(2000);
  for (int i = 0 ; i < 15; i++) {
    delay(100);
  }

  myPotiMaxPos = getServoPositionSensorValue();
  Serial.print("   Servo-Signal: ");
  Serial.print(myServo->readMicroseconds());
  Serial.print("us ");
  Serial.print(" ergibt max. Poti-Wert: ");
  Serial.print(myPotiMaxPos);
  Serial.println();

  delay(3000);

  if (abs((myPotiMaxPos - myPotiMinPos )) < 100) {
    Serial.print("ERROR: Kalibrierung des Poti fehlgeschlagen ");
    Serial.print(myPotiMinPos);
    Serial.print("/");
    Serial.print(myPotiMaxPos);
    Serial.println();
    setState(S_ERROR_INIT);
    return ;
  }
}

void ServoTester::servoSpeedTest() {
  // Speed test
  Serial.print("Servo Speedtest");
    Serial.print(" bei Batteriespannung: ");
    Serial.print(" Ubatt=");
    char outString[15];
    dtostrf(((float) getBatteryVoltage()/1000), 3, 1, outString);
    Serial.print(outString);
    Serial.print("V");
  Serial.println();

  uint16_t posInUs;
  uint16_t potiPos;
  int pos;
  float precision;

    for (int i = -100; i < +101; i=i+200) {
      posInUs = myServoPosInUsMax;
      potiPos = myPotiMaxPos;
      if (i == -100) {
        posInUs = myServoPosInUsMin;
        potiPos = myPotiMinPos;
      }

    Serial.print("   Speedtest ");
    Serial.print(i);
    Serial.print("%: ");
    Serial.println();
    myServo->writeMicroseconds(1500);
    delay(1000);
    unsigned long start = millis();
    myServo->writeMicroseconds(posInUs);
    delay(10);
    do {
      pos = getServoPositionSensorValue();
      precision = (float)pos/potiPos;
    } while (!(precision > 0.9 && precision < 1.1));
    unsigned long speedval = millis() - start;
    Serial.print("     Endposition (");
    Serial.print(i);
    Serial.print("%/");
    Serial.print(posInUs);
    Serial.print("us): in ");
    Serial.print(speedval);
    Serial.print("ms");
    Serial.println();
  }
  delay(3000);
}


/*
 * return the postion of the potentiometer as a value of 0-1023
 * due to mechanical restrictions, the effective range is much smaller (300-600)
 */
uint16_t ServoTester::getPotiPostion() {
  return analogRead(myPotiPin);
}

//
bool ServoTester::comparePositionData(uint8_t aTag, uint8_t aMeasureCnt, uint16_t aPos, uint8_t aDeviation) {
  static uint16_t cmpData[5];
  bool retVal = false;
  float deviation = (float) aDeviation;

  // the first position data is the reference
  if (aMeasureCnt == 0) {
    cmpData[aTag] = aPos;
    retVal = true;
  } else {
    float compValue =
      abs(100.0f - (((float) aPos / cmpData[aTag])*100.0f));
    if ( compValue <= deviation) {
      retVal = true;
    }
  }
  if (retVal == false) {
    Serial.print("WARN: comparePositionData()");
    Serial.print("(");
    Serial.print(aPos);
    Serial.print("/");
    Serial.print(cmpData[aTag]);
    Serial.print(")");
    Serial.println();
  }
  return retVal;
}

bool ServoTester::checkPosition() {
  bool retVal = false;
  myCheckCnt++;
  unsigned int loopCnt=0;
  #if SERVO_TESTER_DEBUG > 3
  bool warn=false;
  #endif
  while(true) {
    loopCnt++;
    uint16_t potiPostion = getServoPositionSensorValue();
    int servoSignal = myServo->readMicroseconds();
    // Die erwartete
    // int potiCalcPosition = (((float)(myPotiMaxPos-myPotiMinPos)/1000)*(servoSignal-1000))+myPotiMinPos;
    int potiCalcPosition =  map(servoSignal, 1000, 2000, myPotiMinPos, myPotiMaxPos) ;
    float result = (float)potiPostion/potiCalcPosition;
    if (result > 0.96 && result < 1.04) {
      #if SERVO_TESTER_DEBUG > 10
      if (warn) {
       printReadArray();
       printData(potiPostion, potiCalcPosition);
      }
     #endif
     retVal = true;
     break;
   } else {
     if (loopCnt > 5) {
       printReadArray();
       Serial.print("ERROR: #");
       Serial.print(loopCnt);
       Serial.print(" ");
       printData(potiPostion, potiCalcPosition);
       myCheckError++;
       break;
     } else {
       #if SERVO_TESTER_DEBUG > 10
       warn=false;
       printReadArray();
       Serial.print("WARN: #");
       Serial.print(loopCnt);
       Serial.print(" ");
       printData(potiPostion, potiCalcPosition);
       #endif
     }
    }
  }
  return retVal;
}


void ServoTester::printTestResult() {
    static bool notdone = true;
    if (notdone)  {
      notdone = false;
      Serial.print("End-Testergebnis: ");
      Serial.println();
      Serial.print("=================================");
      Serial.println();
      Serial.print(myNow/1000/60);
      Serial.print("min ");
      Serial.print("Test-Info:");
      Serial.print(" #-Test-Sessions :");
      Serial.print(myTestCnt);
      Serial.print(" mit #-Messungen :");
      Serial.print(myCheckCnt);
      Serial.print(" davon mit Abweichungen : ");
      Serial.print(myCheckError);
      Serial.println();
    }
}

void ServoTester::printData(int aPotiPos, int aCalcPotiPos) {
        Serial.print(" Session #: ");
        Serial.print(myTestCnt);
        Serial.print(" Poti: ");
        Serial.print(aPotiPos);
        Serial.print("(");
        Serial.print(myPotiMinPos);
        Serial.print("/");
        Serial.print(myPotiMaxPos);
        Serial.print(")");
        Serial.print(" CalcPoti: ");
        Serial.print(aCalcPotiPos);
        Serial.print(" CheckResult: ");
        Serial.print((float)aPotiPos/aCalcPotiPos);
        Serial.print(" Pulse in us: ");
        Serial.print(myServo->readMicroseconds());
        Serial.println();
}

void ServoTester::runServoPositionTest() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("runServoPostionTest()");
  Serial.println();
  #endif
  static unsigned long startTime = 0;
  unsigned long time;

  if (startTime == 0) {
    Serial.print("Starte Servo-Test (");
    Serial.print(SERVO_TESTER_TEST_DURATION_IN_MIN);
    Serial.print("min): Batteriespannung: ");
    Serial.print(" Ubatt=");
    char outString[15];
    dtostrf(((float) getBatteryVoltage()/1000), 3, 1, outString);
    Serial.print(outString);
    Serial.print("V");
    Serial.println();
    myTimeTicker = 0;
    startTime = myNow;
    setState(S_RUNNING);
    checkBatteryVoltage(0);
  }
  checkBatteryVoltage();
  time = myNow - startTime;

  if (time > myTimeTicker && (myTimeTicker/1000/60 < SERVO_TESTER_TEST_DURATION_IN_MIN)) {
    // will be printed once per minute
    ledShowResult();
    toggleServoFrequency();
    myTimeTicker += 1000L*60;
    Serial.print(time/1000/60);
    Serial.print("min ");
    Serial.print("Test-Info:");
    Serial.print(" PWM-Freq=");
    Serial.print(Servo::getRefreshFreq());
    Serial.print("Hz Ubatt=");
    char outString[5];
    dtostrf(((float) getBatteryVoltage()/1000), 3, 1, outString);
    Serial.print(outString);
    Serial.print("V #-Test-Sessions :");
    Serial.print(myTestCnt);
    Serial.print(" mit #-Messungen :");
    Serial.print(myCheckCnt);
    Serial.print(" davon mit Abweichungen : ");
    Serial.print(myCheckError);
    Serial.println();
  } else if ((myTimeTicker/1000/60) == SERVO_TESTER_TEST_DURATION_IN_MIN) {
    // end test run after 2h
    printTestResult();
    myServo->writeMicroseconds(1500);
    myServo->detach();
    setState(S_FINISHED);
    while (true) {
      delay(5000);
      checkBatteryVoltage(5);
    }
  }

  if (true) {
    ledShowResult();

    // random position test
    myServo->writeMicroseconds(1500);
    // Serial.print("random PT: ");
    // Serial.println(myServo->readMicroseconds());
    delay(100);
    uint16_t randomPos = random (1000, 2001);
    myServo->writeMicroseconds(randomPos);
    // Serial.print("random PT: ");
    // Serial.println(myServo->readMicroseconds());
    delay(100);
    // wait  till postion is reached
    // Serial.println("check this position");
    if (randomPos > 0) {
      for (int i = 5 ; i < 5; i++) {
        Serial.print("warte auf Zufalls-Servo-Position:");
        Serial.println(getPotiPostion());
        delay(40);
      }
      checkPosition();
    }
    myTestCnt++;
  }

  static uint8_t testCharakteristic = 2;

  if (testCharakteristic == 1) {
    // continues position test 1000 -> 2000
    int steps=1000;
    myServo->writeMicroseconds(1000);
    delay(500);
    for (int i=0; i<steps;i++) {
      myServo->writeMicroseconds(1000+1000/steps*i);
      delay(5);
      if (true || i%10 == random(0,15)) {
        for (int j = 5 ; j < 5; j++) {
          Serial.print("warte auf Linear-Servo-Position:");
          Serial.println(getPotiPostion());
          delay(20);
        }
        checkPosition();
      }
    }
    myServo->detach();
    delay(750);
    myServo->attach(myServoPin);
  } else if (testCharakteristic == 2) {
    myServo->writeMicroseconds(1500);
    delay(500);
    for (int rs=1; rs<11;rs++) {
      while (myServo->readMicroseconds() < 1500 + 50*rs) {
        myServo->writeMicroseconds(myServo->readMicroseconds()+1);
        delay(5);
        // Serial.print("1 PT: ");
        // Serial.println(myServo->readMicroseconds());
        if (random(0,501) < 10)
          checkPosition();
      }
      while (myServo->readMicroseconds() > 1500 - 50*rs) {
        myServo->writeMicroseconds(myServo->readMicroseconds()-1);
        // Serial.print("2 PT: ");
        // Serial.println(myServo->readMicroseconds());
        if (random(0,101) < 10)
          checkPosition();
      }
    }
  }
}

/*
 * return the servo position as a value in range 0-1023.
 * due to mechanical restrictions the effective range is much smaller (300-600)
 * the value is read several times and compared to avoid scatterning values.
 */
#define MIN_CORRECT 0.98
#define MAX_CORRECT 1.02
uint16_t ServoTester::getServoPositionSensorValue() {
  #if SERVO_TESTER_DEBUG > 4
  Serial.print("DEBUG - trace: ");
  Serial.print("getServoPositionSensorValue()");
  Serial.println();
  #endif
  int maxWait = 200;
  uint8_t delayPotiRead=3; // delay between poti reads

  myPotiReadCnt=0;
  while (true) {
    myPotiReadCnt++;
    myPotiReadArray[4] = myPotiReadArray[3];
    myPotiReadArray[3] = myPotiReadArray[2];
    myPotiReadArray[2] = myPotiReadArray[1];
    myPotiReadArray[1] = myPotiReadArray[0];
    myPotiReadArray[0] = getPotiPostion()+1; // +1 to avoid divison by zero

    if (myPotiReadCnt > 5 && SERVO_TESTER_DEBUG > 4){
      printReadArray();
    }
    if (( myPotiReadCnt > 5
      && ((float)myPotiReadArray[0]/myPotiReadArray[1]) > MIN_CORRECT && ((float)myPotiReadArray[0]/myPotiReadArray[1]) < MAX_CORRECT
      && ((float)myPotiReadArray[0]/myPotiReadArray[2]) > MIN_CORRECT && ((float)myPotiReadArray[0]/myPotiReadArray[2]) < MAX_CORRECT
      && ((float)myPotiReadArray[0]/myPotiReadArray[3]) > MIN_CORRECT && ((float)myPotiReadArray[0]/myPotiReadArray[3]) < MAX_CORRECT
      && ((float)myPotiReadArray[0]/myPotiReadArray[4]) > MIN_CORRECT && ((float)myPotiReadArray[0]/myPotiReadArray[4]) < MAX_CORRECT)
      || myPotiReadCnt*delayPotiRead > maxWait) {
        break;
      }

    delay(delayPotiRead);
  }
  #if SERVO_TESTER_DEBUG > 6
  Serial.print("#reads: ");
  Serial.print(myPotiReadCnt);
  Serial.print(" value: ");
  Serial.print(myPotiReadArray[0]-1);
  Serial.println();
  #endif
  return myPotiReadArray[0]-1;
}

void ServoTester::printReadArray(){
    Serial.print("PotiPos: #");
    Serial.print(myPotiReadCnt-5);
    Serial.print(" ");
    Serial.print((float)myPotiReadArray[0]/((myPotiReadArray[0]+myPotiReadArray[1]+myPotiReadArray[2]+myPotiReadArray[3]+myPotiReadArray[4])/5));
    for (int i=0; i<5;i++) {
      Serial.print(",");
      Serial.print(myPotiReadArray[i]);
    }
    Serial.println();
}

void ServoTester::runServoFollowsPoti() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("runServoFollowsPoti()");
  Serial.println();
  #endif
  delay(10);
  static int servoPosInUs = 1500;
  static int servoPosInPercent = 0;
  static int calcUsPos    = 0;

  int usStep               = 500/4; // 25% Steps from as -125%,-100%,-75%,-50%,-25%,0%,25%,50%,75%,100%,125%
  int servoPosInUsMin      = 1000 - usStep;
  int servoPosInUsMax      = 2000 + usStep;

  int potiPosMin     = 220;
  int potiPosMax     = 820;
  int steps          = 11;
  int potiSteps = (potiPosMax - potiPosMin)/(2*steps-1)+1;  // 29
  int rangePotiPosMin;
  int rangePotiPosMax;
  int servoPosInUsRangeMin;
  int servoPosInUsRangeMax;
  int rangeIdx;
  int potiPos = getPotiPostion();
  for (int i=0; i < 2*steps-1; i++) {
    rangeIdx = i;
    // there are 11 snapped positions in 25% Steps from as -125%,-100%,-75%,-50%,-25%,0%,25%,50%,75%,100%,125%
    // between the 11 snapped positions, there are 11-1 continuos regions
    rangePotiPosMin = potiPosMin+i*potiSteps;  // 200,
    rangePotiPosMax = rangePotiPosMin + potiSteps;
    if (i%2 == 0) {
      // snapped position handling
      // Serial.print("snapped Range ");
      int servoPosInUsSnapped  = servoPosInUsMin + i*usStep/2; // ~875, 1000, 1125, ...
      if (rangePotiPosMin <= potiPos && potiPos <= rangePotiPosMax) {
        servoPosInUs=servoPosInUsSnapped;
        break;
      }
    } else {
      // continuous position handling
      // Serial.print("contin. Range ");
      servoPosInUsRangeMin = servoPosInUsMin + (i-1)*usStep/2; // ~875, 1000, 1125, ...
      servoPosInUsRangeMax = servoPosInUsRangeMin + usStep;
      if (rangePotiPosMin <= potiPos && potiPos <= rangePotiPosMax) {
        servoPosInUs =  map(potiPos, rangePotiPosMin, rangePotiPosMax, servoPosInUsRangeMin, servoPosInUsRangeMax);
        break;
      }
    }
    if (potiPos < potiPosMin) {
      servoPosInUs = servoPosInUsMin;
    }
    if (potiPos > potiPosMax) {
      servoPosInUs = servoPosInUsMax;
    }

    //   Serial.print("[");
    //   Serial.print(i);
    //   Serial.print("] ");
    //   Serial.print(rangePotiPosMin);
    //   Serial.print("-");
    //   Serial.print(rangePotiPosMax);
    //   Serial.println();
  }

  myServo->writeMicroseconds(servoPosInUs);
  servoPosInPercent = map(servoPosInUs, 1000, 2000, -100, 100);
  static int lastServoPosInPercent = -200;
  static int lastPotiPos = -1;

  static uint16_t  lastCurrent = 0;
  int16_t  current =  getServoCurrent();
  // if (servoPosInPercent != lastServoPosInPercent || abs(lastCurrent-current) > 10 ) {
  if (servoPosInPercent != lastServoPosInPercent ) {
    switch (servoPosInPercent) {
    case 0:
      myLedGreen->clearBlinkPattern();
      myLedGreen->on();
      break;
    case 25:
    case -25:
      myLedGreen->blink(200, 1, 500, STATUS_LED_FOREVER);
      break;
    case 50:
    case -50:
      myLedGreen->blink(200, 2, 500, STATUS_LED_FOREVER);
      break;
    case 75:
    case -75:
      myLedGreen->blink(200, 3, 500, STATUS_LED_FOREVER);
      break;
    case 100:
    case -100:
      myLedGreen->blink(200, 4, 500, STATUS_LED_FOREVER);
      break;
    case 125:
    case -125:
      myLedGreen->blink(200, 5, 500, STATUS_LED_FOREVER);
      break;
      default:
      myLedGreen->clearBlinkPattern();
      myLedGreen->off();
      break;
    }
  }
  // if (servoPosInPercent != lastServoPosInPercent || abs(lastCurrent-current) > 10 ) {
  if (abs( potiPos - lastPotiPos) > 1 ) { // }|| true  ) {
    lastPotiPos = potiPos;
    Serial.print(" Ubatt=");
    char outString[15];
    dtostrf(((float) getBatteryVoltage()/1000), 3, 1, outString);
    Serial.print(outString);
    Serial.print(" Iservo=");
    sprintf(outString, "%03d", current);
    Serial.print(outString);
    Serial.print("mA Poti Position: ");
    Serial.print(potiPos);
    Serial.print(" Servo Pos: ");
    Serial.print(servoPosInUs);
    Serial.print("us / ");
    Serial.print(servoPosInPercent);
    Serial.print("% ");
    if (rangeIdx%2 == 0) {
      Serial.print(" snapp-");
    } else {
      Serial.print(" cont.-");
    }
    Serial.print(" Range [");
    Serial.print(rangeIdx);
    Serial.print("] ");
    Serial.print(rangePotiPosMin);
    Serial.print("-");
    Serial.print(rangePotiPosMax);
    if (rangeIdx%2 == 1) {
      Serial.print(", ");
      Serial.print(servoPosInUsRangeMin);
      Serial.print("-");
      Serial.print(servoPosInUsRangeMax);
    }
    Serial.println();
    lastServoPosInPercent = servoPosInPercent;
  }
}

void ServoTester::ledShowResult() {
  #if SERVO_TESTER_DEBUG > 2
  Serial.print("DEBUG - trace: ");
  Serial.print("ledShowResult()");
  Serial.println();
  #endif
  static int lastErrorCnt=0;
  if (myCheckError > 0) {
    if (myCheckError != lastErrorCnt) {
      lastErrorCnt = myCheckError;
      myLedRed->clearBlinkPattern();
      myLedRed->addBlinkPattern(100, 300, myCheckError);
      myLedRed->addBlinkPattern(0, 1500, 1);
      myLedRed->startBlinkPattern(STATUS_LED_FOREVER);
    }
  } else {
    myLedRed->off();
  }
}

void ServoTester::run() {
  #if SERVO_TESTER_DEBUG > 0
  Serial.print("DEBUG - trace: ");
  Serial.print("run()");
  Serial.println();
  #endif
  if (myInitDone != true) {
    Serial.print("ERROR: init() with problems");
    Serial.println();
    delay (10000);
    return;
  }
  myNow = millis();
  switch (myDefaultFunction) {
    case SERVO_TESTER_FUNC_FOLLOWS_POTI:
      runServoFollowsPoti();
      break;
    case SERVO_TESTER_FUNC_POSITION_TEST:
      runServoPositionTest();
      break;
  }
}

void ServoTester::initCurrentSensor(uint8_t aPin) {
  myCurrentSensorPin = aPin;
}

void ServoTester::initBatterySupervision(uint8_t aPin, uint16_t aWarningVoltage, uint16_t aErrorVoltage) {
  myBatterySupervisionPin = aPin;
  myBatterySupervisionWarnVoltage = aWarningVoltage;
  myBatterySupervisionErrorVoltage = aErrorVoltage;
}

uint16_t ServoTester::getBatteryVoltage() {
  // get the voltage of the servo supply battery, measured by a voltage divider 10k/10k Ohm
  return (analogRead(myBatterySupervisionPin) / 1023.0) * V_REF * 2;
}

uint16_t ServoTester::getServoCurrent() {
  // get the current of the current sensor ACS712-05, given as analog voltage value
  int16_t val = analogRead(myCurrentSensorPin);
  val = val-511;
  val = max(0, val);
  val = abs(val) / 1023.0 * V_REF; // mV
  // val = val/185.0; // 185mV per Amp
  return val;
}

void ServoTester::initModeControl(uint8_t aPin) {
  myModeControlPin = aPin;
  pinMode(myModeControlPin, INPUT_PULLUP);
}


void ServoTester::setDefaultFunction(uint8_t aFunction) {
  myDefaultFunction = aFunction;
}

void ServoTester::toggleServoFrequency() {
  static int f = 50;
  if (f > 100) f = 10;
  Servo::setRefreshFreq(f);
  f=f+10;
}

bool ServoTester::checkBatteryVoltage(uint8_t aRepeat) {
  uint8_t batteryLowCnt=0;
  uint8_t batteryWarnCnt=0;

  if (myBatterySupervisionPin == SERVO_TESTER_NOTUSED) {
    return true;
  }

  uint16_t voltage = getBatteryVoltage();
  if (voltage < SERVO_TESTER_BATT_STOP_LEVEL) {
    batteryLowCnt++;
    if (batteryLowCnt > aRepeat) {
      Serial.print("Achtung !!");
      Serial.println();
      Serial.print("ERROR: Versorgungsspannung zu niedrig: ");
      char outString[15];
      Serial.print(voltage);
      Serial.print("mV/");
      dtostrf(((float) getBatteryVoltage()/1000), 3, 1, outString);
      Serial.print(outString);
      Serial.print("V");
      Serial.println();
      Serial.print("Testfunktion wird deaktiviert");
      Serial.println();
      setState(S_ERROR_BATT);
      delay(10000);
      shutdown();
    }
  } else if (voltage < SERVO_TESTER_BATT_WARN_LEVEL) {
    batteryWarnCnt++;
    if (batteryWarnCnt > aRepeat) {
      setState(S_ERROR_BATT);
    }
  } else {
    // reset counter
    batteryWarnCnt=0;
    batteryLowCnt=0;
  }
  return (batteryLowCnt == 0);
}

#ifdef MODULE_TEST
void ServoTester::moduleTest() {
  Serial.println("ServoTester::moduleTest() running:");
  test_comparePositionData();
}

bool ServoTester::test_comparePositionData() {
  Serial.print("ServoTester::test_comparePositionData() running:");
  Serial.println();
  assert(comparePositionData(0, 0, 1300, 5));
  assert(comparePositionData(0, 1, 1300, 5));
  assert(comparePositionData(0, 2, 1302, 5));
  assert(comparePositionData(0, 2, 1302, 5));
  assert(!comparePositionData(0, 3, 1602, 5));

  assert(comparePositionData(1, 0, 1000, 5));
  assert(comparePositionData(1, 1, 1030, 5));
  for (int i=950; i < 1050; i++) {
    assert(comparePositionData(1, 2, i, 5));
  }
  for (int i=900; i < 949; i++) {
    assert(!comparePositionData(1, 2, i, 5));
  }
  for (int i=1060; i < 1100; i++) {
    assert(!comparePositionData(1, 2, i, 5));
  }
  Serial.println("ServoTester::test_comparePositionData() finished");
  return true;
}
#endif

void wakeUpNow()        // here the interrupt is handled after wakeup
{
}

void ServoTester::shutdown() {
  setState(S_SHUTDOWN);
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * https://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we
   * choose the according
   * sleep mode: SLEEP_MODE_PWR_DOWN
   *
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
   * accidentally pushed interrupt button doesn't interrupt
   * our running program. if you want to be able to run
   * interrupt code besides the sleep function, place it in
   * setup() for example.
   *
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.
   *
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */

  attachInterrupt(8,wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
                                     // wakeUpNow when pin 2 gets LOW

  sleep_mode();            // here the device is actually put to sleep!!
                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
                           // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
                            // wakeUpNow code will not be executed
                            // during normal running time.
}

#ifdef MODULE_TEST
// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link.
    Serial.println("ERROR: assertion error at:");
    Serial.print("  function: ");
    Serial.println(__func);
    Serial.print("  file: ");
    Serial.print(__file);
    Serial.print(":");
    Serial.println(__lineno, DEC);
    Serial.print("  assertion expression: ");
    Serial.println(__sexp);
    Serial.flush();
    // abort program execution.
    abort();
}
#endif
