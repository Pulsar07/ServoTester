// Version history
// V1.00 : inital version
// V1.03 : full functional initial version
// V1.10 : - changed to async StatusLED
//         - debug code refactored to #define SERVOTESTER_DEBUG
//         - non precise poti value fixed by using more precise poti
//         - added Speedtest
// V1.11 : - Mode 1 added: servo control, with blink at 11 snapped positions
//           in 25% Steps from as -125%,-100%,-75%,-50%,-25%,0%,25%,50%,75%,100%,125%
// V1.20 : - HWVersion 2
//         - new feature: battery supervision
//         - new servoPostionTest initialization
//         - refactored
// V1.21 : - StatusLED with pause
// V2.00 : - refactored from ServoTester.ino -> ServerTester class


#ifndef SERVO_TESTER_H
#define SERVO_TESTER_H

// #define MODULE_TEST
#ifdef MODULE_TEST
#define __ASSERT_USE_STDERR
#include <assert.h>
#endif

#include <Arduino.h>
#include <Servo.h>
#include "StatusLED.h"

#define SERVO_TESTER_DEBUG 0

#define V_REF 5000
#define SERVO_TESTER_TEST_DURATION_IN_MIN 120
#define SERVO_TESTER_NOTUSED -1
#define SERVO_TESTER_FUNC_POSITION_TEST 1
#define SERVO_TESTER_FUNC_FOLLOWS_POTI 2
#define SERVO_TESTER_BATT_STOP_LEVEL 7000
#define SERVO_TESTER_BATT_WARN_LEVEL 7200


/**
 * \class ServoTester
 *
 * \brief provides test and controlling functions for a combination of a RC-Servo and a optional mechanical coupled position sensor (potentiometer)
 *
 * \author Author: Rainer Stransky
 *
 * \copyright This project is released under the GNU Public License v3
 *          see https://www.gnu.org/licenses/gpl.html.
 * Contact: opensource@so-fa.de
 *
 */
class ServoTester {
  public:
    /** \brief creates a StatusLED with the given pin
      * \param aServoPin adress of the output pin the servo is connected to
      * \param aPotiPin adress of the analog input pin the position sensor / poti is connected to
      * \param aGreenLedPin adress of the output pin the green status led is connected to
      * \param aRedLedPin adress of the output pin the red error status led is connected to
      * \param aVersionString version string of the ServoTester to be used at text messages
      */
    ServoTester(int8_t aServoPin, int8_t aPotiPin, int8_t aGreenLedPin, int8_t aRedLedPin, const char* aVersionString);

    /** \brief attaches a optional analog input pin for battery supervision
      * \param aPin adress of the analog input pin
      * \param aWarningVoltage voltage warning threshold in mV
      * \param aErrorVoltage voltage error threshold in mV
      * if the error threshold is reached, the arduino is set to sleep and the test run is interrupted
      */
    void attachBatterySupervisionPin(int8_t aPin, uint16_t aWarningVoltage, uint16_t aErrorVoltage);

    /** \brief get the battery voltage
      * \return battery voltage in mV
      */
    uint16_t getBatteryVoltage();

    /** \brief attaches a optional input pin to switch between "servo position test" and "servo follows poti"
      * \param aPin adress of the input pin
      */
    void attachControlPin(uint8_t aPin);

    /** \brief if no control pin is attached the function to be used can be set here
      * \param aFunction = SERVO_TESTER_POSITION_TEST || SERVO_TESTER_FOLLOWS_POTI
      */
    void setDefaultFunction(uint8_t aFunction);


    /** \brief runs the servo tests, and has to be called in the arduino loop
      */
    void run();

    /** \brief runs the initialization, and has to be called in the arduino init
      */
    void init();

    #ifdef MODULE_TEST
    void moduleTest();
    #endif
  private:
    const char* myVersionString;
    StatusLED* myLedGreen;
    StatusLED* myLedRed;
    int8_t myServoPin;
    int8_t myPotiPin;
    Servo* myServo;
    int8_t myBatterySupervisionPin;
    int8_t myControlPin;
    uint16_t myBatterySupervisionWarnVoltage;
    uint16_t myBatterySupervisionErrorVoltage;
    uint8_t myDefaultFunction;
    uint16_t myPotiMinPos;
    uint16_t myPotiMaxPos;
    uint16_t myServoPosInUsMin;
    uint16_t myServoPosInUsMax;
    void ledShowResult();
    void shutdown();

    /** \brief checks the level of the battery voltage
      * \param aVoltage the
      */
    bool checkBatteryVoltage(uint8_t aRepeat=10);
    /** \brief runs the servo position test procedure
      */
    void runServoPositionTest();
    /** \brief do the initialization of the PositionTest
      */
    bool initServoPositionTest();
    bool myInitDone=false;
    /** \brief do the middle position / zero position initialization
      */
    void initServoPositionTest_ZeroPos();
    /** \brief check if a smooth movement in the servo middle range does force reproducable position sensor data
      * \return true if the position sensor data is a set of reproducable sensor data
      */
    bool initServoPositionTest_CheckPosSensor();
    void runServoFollowsPoti();
    bool initServoFollowsPoti();
    uint16_t getServoPositionSensorValue();
    uint16_t getPotiPostion();
    /** \brief read the servo position value and compare it with the sensor state
      * if the sensor state does not fit to the position value, wait a little and try it again (max 5 times)
      * \return true if the position sensor data fits to the servo position value, false else
      */
    bool checkPosition();
    uint16_t myCheckCnt;
    uint16_t myCheckError;
    uint32_t myTestCnt;
    void printTestResult();
    unsigned long myNow;
    unsigned long myTimeTicker;
    void printData(int aPotiPos, int aCalcPotiPos);
    void printReadArray();
    uint16_t myPotiReadArray[5];
    uint16_t myPotiReadCnt;

    enum State {  // <-- the use of typedef is optional
      S_INIT,
      S_RUNNING,
      S_FINISHED,
      S_ERROR_SENSOR,
      S_ERROR_INIT,
      S_ERROR_BATT,
      S_SHUTDOWN,
      S_UNSET
    };
    ServoTester::State myState;

    void setState(ServoTester::State aState);

    /** \brief calibrates the servo movement in relation to the positionssensor/poti voltage values
    */
    void calibratePoti();
    /** \brief runs a servo speed test
    */
    void servoSpeedTest();
    /** \brief compare a position with a previous stored position
      * \param aTag the tag of the position as a index value between 0-4. So only 5 different position are supported in parallel
      * \param aMeasureCnt is the number of the measurement. For aMeasure == 0, the position is only stored as reference and true is returned
      * \param aPos is the measured position value
      * \param aDeviation is the deviation in percent which should be accepted be return the compare result as true
      * \return true of the given position is with the deviation range, or if the aMeasureCnt == 0 , false otherwise
      */
    bool comparePositionData(uint8_t aTag, uint8_t aMeasureCnt, uint16_t aPos, uint8_t aDeviation);
    #ifdef MODULE_TEST
    bool test_comparePositionData();
    #endif
};

#endif // SERVO_TESTER_H
