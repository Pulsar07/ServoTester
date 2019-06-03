
// Version history
// V1.00 : inital version with base blink support
// V1.10 : refactored without pause and sym. blink
// V1.20 : refactored with asymmetric blink

#ifndef STATUSLED_H
#define STATUSLED_H

#include <Arduino.h>

#define STATUS_LED_FOREVER -1
#define STATUS_LED_PATTERN_MAX_SIZE 5

/**
 * \class StatusLED
 *
 * \brief class providing non blocking led blink patterns for multiple leds
 *
 * this class provides non blocking led blink battern based on one MsTimer2
 * interrupt for multipe connected led on arduinos output pins.
 * Blink on and off blink times can be defined separatly, repeated and combined
 * Example:
 * StatusLED mLedRed = StatusLED(pinLedRed);
 * mLedRed.clearBlinkPattern();
 * mLedRed.addBlinkPattern(100, 400, 3); // ...
 * mLedRed.addBlinkPattern(400, 100, 3); // ---
 * mLedRed.addBlinkPattern(100, 400, 3); // ...
 * mLedRed.addBlinkPattern(0, 2000, 1); // 2s pause
 * mLedRed.startBlinkPattern(STATUS_LED_FOREVER);

 * \author Author: Rainer Stransky
 *
 * \copyright This project is released under the GNU Public License v3
 *          see https://www.gnu.org/licenses/gpl.html.
 * Contact: opensource@so-fa.de
 *
 */
class StatusLED{
  public:
    /** \brief creates a StatusLED with the given pin
      * \param aLedPin number of the output pin the led is connected to
      */
    StatusLED(uint8_t aLedPin);
    /** \brief get the current blink state on/off
      * \return true for on, false for off
      */
	  bool getState();
    /** \brief set the led to light
      */
    void on();
    /** \brief set the led to not light
      */
	  void off();

    /** \brief set the led light state to toggle
      */
	  void toggle();

    /** \brief stop previous blink pattern and  start a asymmetric single blink pattern
      * \param aOnBlinkTime defines the time the led is on in milliseconds.
      * \param aOffBlinkTime defines the time the led is off in milliseconds.
      * \param aBlinkCount defines how often the blink state should change within one cylce
      * \param aPauseTime defines the time the led pauses after each cycle
      * \param aCycleCount defines how often the cylce should repeat
      */
	  void blinka(unsigned int aOnBlinkTime, unsigned int aOffBlinkTime, int aBlinkCount=1, unsigned int aPauseTime=0, int aCycleCount=STATUS_LED_FOREVER);

    /** \brief stop previous blink pattern and  start a symmetric single blink pattern
      * \param aBlinkTime defines the time the led is on/off in milliseconds.
      * \param aBlinkCount defines how often the blink state should change within one cylce
      * \param aPauseTime defines the time the led pauses after each cycle
      * \param aCycleCount defines how often the cylce should repeat
      */
	  void blink(unsigned int aBlinkTime, int aBlinkCount=1, unsigned int aPauseTime=0, int aCycleCount=STATUS_LED_FOREVER);

    /** \brief add a asymmetric blink pattern
      * \param aOnBlinkTime defines the time the led is on in milliseconds.
      * if this paramter is 0, only the aOffBlinkTime is used.
      * This can be used for definition of a blink pause.
      * \param aOffBlinkTime defines the time the led is off in milliseconds.
      * \param aBlinkCounts defines how often this pattern should be used
      */
    void addBlinkPattern(unsigned int aOnBlinkTime, int aOffBlinkTime, int aBlinkCounts);

    /** \brief add a symmetric link pattern
      * \param aBlinkTime defines the time the led is on and off in milliseconds.
      * this time is used for each state on and off
      * \param aBlinkCounts defines how often this pattern should be used
      */
    void addBlinkPattern(unsigned int aBlinkTime, int aBlinkCounts);

    /** \brief starts processing the previous defined blink patterns
      * \param aCount defines how often the list of patterns should be repeated
      */
    void startBlinkPattern(int aCount=1);

    /** \brief stop the led blinking and delete all defined blink patterns
      */
    void clearBlinkPattern();

    /** \brief this method is called each 100ms by MsTimer2
      */
    static void timerTrigger();

  private:
    enum BlinkProcessingState {  // <-- the use of typedef is optional
      notstarted,
      processing,
      finished
    };

    struct BlinkState {
      int time;
      BlinkProcessingState state = notstarted;
    };

    struct BlinkData {
      int count;
      struct BlinkState on;
      struct BlinkState off;
    };

	  bool myStatus;
	  bool myActive;
	  uint8_t myPin;
	  struct BlinkData myBlinkDataArr[STATUS_LED_PATTERN_MAX_SIZE];
	  struct BlinkData* myCurrentBlinkData;
	  struct BlinkData myBlinkDataWork;
    volatile int8_t myPatternArraySize;
    volatile int8_t myPatternIndex;
	  volatile int myPatternCount;
    static bool myTimerTriggerStarted;
    class StatusLED *myNextLed;
    static StatusLED *myFirstLed;
    void printDebug(int cnt, char const *);
    void printDebug(int cnt);
    void changeState(bool aState);
    void startBlinkState();
};

#endif
