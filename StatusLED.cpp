#include <MsTimer2.h>

#include "StatusLED.h"

#define STATUS_LED_RES 100
// #define STATUS_LED_DEBUG
bool StatusLED::myTimerTriggerStarted=false;
StatusLED *StatusLED::myFirstLed;


#ifdef STATUS_LED_DEBUG
void StatusLED::printDebug(int aIntContext) {
  printDebug(aIntContext, "-");
}

void StatusLED::printDebug(int aIntContext, char const * aStringContext) {
        Serial.print(millis());
        Serial.print(" # ");
        Serial.print(aIntContext);
        Serial.print(" #Str ");
        Serial.print(aStringContext);
        Serial.print(" LED :");
        Serial.print(myPin);
        Serial.print( " new state : ");
        Serial.print(myStatus);
        Serial.print( " on time : ");
        Serial.print(myBlinkDataWork.on.time);
        Serial.print( " on state : ");
        Serial.print(myBlinkDataWork.on.state);
        Serial.print( " off time : ");
        Serial.print(myBlinkDataWork.off.time);
        Serial.print( " off state : ");
        Serial.print(myBlinkDataWork.off.state);
        Serial.print( " state count : ");
        Serial.print(myBlinkDataWork.count);
        Serial.print( " pattern count : ");
        Serial.print(myPatternCount);
        Serial.println();
}
#endif

void StatusLED::startBlinkState() {
   if ( this->myBlinkDataWork.on.time == 0 ) {
     #ifdef STATUS_LED_DEBUG
     this->printDebug(-100, "on skipped");
     #endif
     this->myBlinkDataWork.on.state = finished;
     this->myBlinkDataWork.off.state = processing;
   } else {
     #ifdef STATUS_LED_DEBUG
     this->printDebug(-100, "on notstarted -> processing !!ON!!");
     #endif
     this->myBlinkDataWork.on.state = processing;
     this->changeState(true);
   }
}

void StatusLED::timerTrigger() {
  // this method is called each 100ms
  static int cnt=0;
  cnt++;
  StatusLED *led;

  for ( led = myFirstLed; led != 0; led = led->myNextLed ) {
    if (led->myActive) {
      if (led->myBlinkDataWork.on.state == notstarted) {
        led->startBlinkState();
      } else
      if (led->myBlinkDataWork.on.state == processing) {
        led->myBlinkDataWork.on.time--;
        if ( led->myBlinkDataWork.on.time == 0 ) {
          #ifdef STATUS_LED_DEBUG
          led->printDebug(-100, "on finished - off processing !!OFF!!");
          #endif
          led->changeState(false);
          led->myBlinkDataWork.on.state = finished;
          led->myBlinkDataWork.off.state = processing;
        } else {
          #ifdef STATUS_LED_DEBUG
          led->printDebug(-100, "on processing");
          #endif
        }
      } else
      if (led->myBlinkDataWork.off.state == processing) {
        led->myBlinkDataWork.off.time--;
        if ( led->myBlinkDataWork.off.time == 0 ) {
          #ifdef STATUS_LED_DEBUG
          led->printDebug(-100, "off processing -> finished");
          #endif
          led->myBlinkDataWork.off.state = finished;
          led->myBlinkDataWork.count--;
          if (led->myBlinkDataWork.count == 0 ) {
            #ifdef STATUS_LED_DEBUG
            led->printDebug(-100, "blink state finished");
            #endif
            // myBlinkDataWork.count == 0
            // == this blink state is finished
            // use the next pattern in array
            led->myPatternIndex++;
            if (led->myPatternIndex > led->myPatternArraySize-1) {
              #ifdef STATUS_LED_DEBUG
              led->printDebug(-100, "end of blink array reached");
              #endif
              // end of pattern array reached
              if (led->myPatternCount != STATUS_LED_FOREVER) {
                led->myPatternCount--;
              }
              if (led->myPatternCount == 0) {
                #ifdef STATUS_LED_DEBUG
                led->printDebug(-100, "end of pattern count reached");
                #endif
                // end of pattern count reached, led blink ends
                led->myActive = false;
              } else {
                // start from the beginning of the pattern array one more time
                #ifdef STATUS_LED_DEBUG
                led->printDebug(-100, "start from begining of the pattern !!ON!!");
                #endif
                led->myPatternIndex = 0;
                led->myCurrentBlinkData = &led->myBlinkDataArr[led->myPatternIndex];
                led->myBlinkDataWork = *led->myCurrentBlinkData;
                led->startBlinkState();
              }
            } else {
              // use next pattern in array
              #ifdef STATUS_LED_DEBUG
              led->printDebug(-100, "use next pattern in array !!ON!!");
              #endif
              led->myCurrentBlinkData = &led->myBlinkDataArr[led->myPatternIndex];
              led->myBlinkDataWork = *led->myCurrentBlinkData;
              led->startBlinkState();
            }
          } else {
            // myBlinkDataWork.count != 0
            // == this blink state is not finished
            // it should be repeated
            #ifdef STATUS_LED_DEBUG
            led->printDebug(-100, "blink state repeat !!ON!!");
            #endif
            int count = led->myBlinkDataWork.count;
            led->myBlinkDataWork = *led->myCurrentBlinkData;
            led->myBlinkDataWork.count = count;
            led->startBlinkState();
          }

        } else {
          #ifdef STATUS_LED_DEBUG
          led->printDebug(-100, "off processing");
          #endif
        }
      }
    }
  }
}

StatusLED::StatusLED(uint8_t ledPin) : myNextLed(0), myActive(false){
	this->myPin=ledPin;
	this->myStatus=LOW;
  clearBlinkPattern();
	pinMode(this->myPin,OUTPUT);
  #ifdef STATUS_LED_DEBUG
  Serial.print("new StatusLED:");
  Serial.println(this->myPin);
  #endif
  if (!StatusLED::myTimerTriggerStarted) {
    StatusLED::myTimerTriggerStarted = true;
    // Serial.println("start timer");
	  MsTimer2::set(50, &StatusLED::timerTrigger); // 100ms period
    MsTimer2::start();
  }
  myNextLed = myFirstLed;
  myFirstLed = this;
}

bool StatusLED::getState(){ return myStatus; }

void StatusLED::changeState(bool aState) {
  if (aState) {
	  digitalWrite(myPin,HIGH);
  } else {
	  digitalWrite(myPin,LOW);
  }
	this->myStatus=aState;
}

void StatusLED::on(void){
  clearBlinkPattern();
  changeState(true);
}

void StatusLED::off(void){
  clearBlinkPattern();
  changeState(false);
}

void StatusLED::toggle(void){
  clearBlinkPattern();
	changeState(!myStatus);
}

void StatusLED::startBlinkPattern(int aCount) {
  if (aCount == 0) {
    #ifdef STATUS_LED_DEBUG
    Serial.print("ERROR - StatusLED::startBlinkPattern : ");
    Serial.print("arg aCount must not be 0");
    Serial.println();
    #endif
    return;
  }
  changeState(false);
  myPatternCount = aCount;
  myPatternIndex = 0;
  myCurrentBlinkData = &myBlinkDataArr[myPatternIndex];
  myBlinkDataWork = *myCurrentBlinkData;
  myActive = true;
  #ifdef STATUS_LED_DEBUG
  Serial.print("startBlinkPattern : ");
  Serial.println();
  printDebug(-2);
  #endif
}

void StatusLED::clearBlinkPattern(){
  myActive = false;
  myPatternCount = 0;
  for (int i=0; i<STATUS_LED_PATTERN_MAX_SIZE; i++) {
    myBlinkDataArr[i].count = -1;
    myBlinkDataArr[i].on.time = 0;
    myBlinkDataArr[i].on.state = notstarted;
    myBlinkDataArr[i].off.time = 0;
    myBlinkDataArr[i].off.state = notstarted;
  }
  myPatternArraySize = 0;
  myPatternIndex = 0;
}

void StatusLED::addBlinkPattern(unsigned int aBlinkTime, int aBlinkCounts){
  addBlinkPattern(aBlinkTime, aBlinkTime, aBlinkCounts);
}

void StatusLED::addBlinkPattern(unsigned int aOnBlinkTime, int aOffBlinkTime, int aBlinkCounts){

  if (aBlinkCounts == 0) return;

  myPatternArraySize++;
  if (myPatternArraySize > STATUS_LED_PATTERN_MAX_SIZE) {
    #ifdef STATUS_LED_DEBUG
    Serial.print("ERROR - StatusLED::addBlinkPattern : ");
    Serial.print("STATUS_LED_PATTERN_MAX_SIZE reached ;-((");
    Serial.println();
    #endif
    return;
  }

  BlinkData *blinkData = &myBlinkDataArr[myPatternArraySize-1];
  blinkData->count = aBlinkCounts;
  struct BlinkState *onState = &blinkData->on;
  struct BlinkState *offState = &blinkData->off;

  unsigned int time = aOnBlinkTime/100;
  onState->time = time;

  time = aOffBlinkTime/100;
  offState->time = time;


  #ifdef STATUS_LED_DEBUG
  Serial.print("addBlinkPattern : ");
  Serial.print(", index: ");
  Serial.print(myPatternArraySize-1);
  Serial.print(", on time: ");
  Serial.print(myBlinkDataArr[myPatternArraySize-1].on.time);
  Serial.print(", on state: ");
  Serial.print(myBlinkDataArr[myPatternArraySize-1].on.state);
  Serial.print(", off time: ");
  Serial.print(myBlinkDataArr[myPatternArraySize-1].off.time);
  Serial.print(", off state: ");
  Serial.print(myBlinkDataArr[myPatternArraySize-1].off.state);
  Serial.print(", count: ");
  Serial.print(myBlinkDataArr[myPatternArraySize-1].count);
  Serial.println();
  #endif
}

void StatusLED::blink(unsigned int aBlinkTime, int aBlinkCount, unsigned int aPauseTime, int aCylcleCount){
  blinka(aBlinkTime, aBlinkTime, aBlinkCount, aPauseTime, aCylcleCount);
}

void StatusLED::blinka(unsigned int aOnBlinkTime, unsigned int aOffBlinkTime, int aBlinkCount, unsigned int aPauseTime, int aCylcleCount){
  #ifdef STATUS_LED_DEBUG
  Serial.print("blink on: ");
  Serial.print(aOnBlinkTime);
  Serial.print("blink off: ");
  Serial.print(aOffBlinkTime);
  Serial.print(", blink count");
  Serial.print(aBlinkCounts);
  Serial.print(", pause");
  Serial.print(aPauseTime);
  Serial.print(", cycle count");
  Serial.print(aCycleCount);
  Serial.println();
  #endif
  if (aOffBlinkTime < 100) aOffBlinkTime = 100;
  clearBlinkPattern();
  addBlinkPattern(aOnBlinkTime, aOffBlinkTime, aBlinkCount);
  if (aPauseTime != 0) {
    addBlinkPattern(0, aPauseTime, 1);
  }
  startBlinkPattern(aCylcleCount);
}
