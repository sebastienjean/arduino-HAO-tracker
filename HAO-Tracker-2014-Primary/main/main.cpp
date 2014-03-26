/*
 Copyright (C) 2012 Sebastien Jean <baz dot jean at gmail dot com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>

// Globals includes
#include "pins.h"
#include "defs.h"

// Libs includes
#include <SD.h>
#include <FSK600BaudTA900TB1500Mod.h>
#include <DS1302.h>
#include <GPS3D.h>
#include <AnalogChannelAnalogSensor.h>
#include <MockAnalogSensor.h>
#include <AnalogSensors.h>
#include <Counter.h>
#include <Counters.h>
#include <Logger.h>

// Modules includes
#include "KiwiFrameBuilder.h"
#include "CustomFrameBuilder.h"
#include "Tone.h"

// -----------------------
// GPS related definitions
// -----------------------

/**
 * Serial port used for GPS
 */
#define serialNmeaGPSPort Serial1

Logger sdLogger (&SD, LOG_FILE_PATH);

/**
 * GPS3D object, whose raw NMEA sentences will be forwarded on debug serial
 */
GPS3D nmeaGPS(&serialNmeaGPSPort, &SERIAL_DEBUG);

/**
 * External buffer used for NMEA RMC sentence parsing and storage
 */
char nmeaRmcSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];

/**
 * External buffer used for NMEA GGA sentence parsing and storage
 */
char nmeaGgaSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];

/**
 * HAO previous altitude (used during ascending phase)
 */
double previousAltitude;

// ---------------------------------
// FSK modulator related definitions
// ---------------------------------

/**
 * FSK modulator object
 */
FSK600BaudTA900TB1500Mod fskModulator(FSK_MODULATOR_TX_PIN);

// ----------------------------
// Persistent counters related definitions
// ----------------------------

/**
 * Frame persistent counter
 */
Counter frameCounter(FRAME_COUNTER_BASE_ADDRESS);

/**
 * Reset persistent counter
 */
Counter resetCounter(RESET_COUNTER_BASE_ADDRESS);

/**
 * Flight phase persistent counter
 */
Counter currentFlightPhaseCounter(CURRENT_FLIGHT_PHASE_COUNTER_BASE_ADDRESS);

/**
 * HAO stillness duration (in seconds) persistent counter
 */
Counter stillnessDurationInLoopsCounter(STILLNESS_DURATION_IN_SECONDS_COUNTER_BASE_ADDRESS);

/**
 * Current flight phase duration persistent counter
 */
Counter currentFlightPhaseDurationCounter(FLIGHT_PHASE_DURATION_COUNTER_BASE_ADDRESS);

/**
 * Array of counters to be included in custom frame
 */
Counter* countersArray[4] =
  { &frameCounter,
    &resetCounter,
    &currentFlightPhaseCounter,
    &currentFlightPhaseDurationCounter };

/**
 * Counters to be included in custom frame
 */
Counters counters((Counter **) &countersArray, 4);

// ----------------------------------
// Analog sensors related definitions
// ----------------------------------

/**
 * Internal temperature analog sensor
 */
AnalogChannelAnalogSensor internalTemperatureAnalogSensor(INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * Differential pressure analog sensor
 */
AnalogChannelAnalogSensor differentialPressureAnalogSensor(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);


/**
 * Battery temperature analog sensor
 */
AnalogChannelAnalogSensor batteryTemperatureAnalogSensor(BATTERY_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

MockAnalogSensor middleTemperatureAnalogSensor(100);

MockAnalogSensor externalTemperatureAnalogSensor(200);
MockAnalogSensor externalHumidityAnalogSensor(300);
MockAnalogSensor upLuminosityAnalogSensor(400);
MockAnalogSensor side1LuminosityAnalogSensor(500);
MockAnalogSensor side2LuminosityAnalogSensor(600);
MockAnalogSensor soundLevelAnalogSensor(700);

/**
 * Voltage analog sensor
 */
// N.B. this analog sensor is handled separately from the others since kiwi frame does
AnalogChannelAnalogSensor voltage(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);


/**
 * Array of analog sensors to be included in custom and (partially) in kiwi frame
 */
AnalogSensor* sensorsArray[10] =
  { &internalTemperatureAnalogSensor,
    &middleTemperatureAnalogSensor,
    &externalTemperatureAnalogSensor,
    &externalHumidityAnalogSensor,
    &differentialPressureAnalogSensor,
    &upLuminosityAnalogSensor,
    &side1LuminosityAnalogSensor,
    &side2LuminosityAnalogSensor,
    &soundLevelAnalogSensor,
    &batteryTemperatureAnalogSensor };

/**
 * Analog sensors to be included in custom frame
 */
AnalogSensors customFrameAnalogSensors((AnalogSensor **) &sensorsArray, 10);

/**
 * Analog sensors to be included in kiwi frame
 */
AnalogSensors kiwiFrameAnalogSensors((AnalogSensor **) &sensorsArray, 8);

// -----------------------------------
// Real Time Clock related definitions
// -----------------------------------

/**
 * RTC object
 */
DS1302 rtc(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);

// --------------------------------
// Custom frame related definitions
// --------------------------------

/**
 * External buffer used to build and send custom frame
 */
char customFrame[CUSTOM_FRAME_MAX_LENGTH];

/**
 * Custom frame builder object
 */
CustomFrameBuilder customFrameBuilder(&counters, &customFrameAnalogSensors, &voltage, &rtc, &nmeaGPS);

// ------------------------------
// KIWI frame related definitions
// ------------------------------

/**
 * External buffer used to build and send Kiwi frame
 */
unsigned char kiwiFrame[KIWI_FRAME_LENGTH];

/**
 * Kiwi frame builder object
 */
KiwiFrameBuilder kiwiFrameBuilder(&kiwiFrameAnalogSensors, &voltage);

Tone toneGenerator(FSK_MODULATOR_TX_PIN);

/**
 * Internal function used to send debug info both on debug serial and radio.
 * @param message a NUL-terminated string, supposed to include line-termination chars if needed.
 * @param string length (included line-termination chars)
 */
void
playMarioTheme()
{
  int oct = 5;
  int bpm = 216;

  toneGenerator.melody(Mi, oct, croche, bpm);
  toneGenerator.melody(Mi, oct, croche, bpm);
  toneGenerator.melody(0, oct, dsoupir, bpm);
  toneGenerator.melody(Mi, oct, croche, bpm);
  toneGenerator.melody(0, oct, dsoupir, bpm);
  toneGenerator.melody(Do, oct, croche, bpm);
  toneGenerator.melody(Mi, oct, noire, bpm);

  toneGenerator.melody(Sol, oct, noire, bpm);
  toneGenerator.melody(0, oct, soupir, bpm);
  toneGenerator.melody(Sol, oct - 1, noire, bpm);
  toneGenerator.melody(0, oct, soupir, bpm);
}

void
debugInfo(char *message, int chars)
{
  SERIAL_DEBUG.write((unsigned char *) message, chars);
  fskModulator.modulateBytes(message, chars);
}

/**
 * Internal function used to play Mario Theme each time a transition occurs.
 */
boolean
initSD()
{
  pinMode(SD_CARD_CHIP_SELECT_PIN, OUTPUT);
  return SD.begin(SD_CARD_CHIP_SELECT_PIN);
}


/**
 * Internal function used to initialize logging (SD).
 *
 * @return logging initialization success status
 */
void
initDebugSerial()
{
  SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUDRATE);
}

void
initGpsSerial()
{
  serialNmeaGPSPort.begin(SERIAL_NMEA_GPS_BAUDRATE);
}

void
initUserSwitch()
{
  pinMode(USER_SWITCH_PIN, INPUT);
  digitalWrite(USER_SWITCH_PIN, HIGH);
}

boolean
initLogging()
{
  return initSD();
}

/**
 * Internal function used to clear all persistent data (log file, counters).
 * Waiting one second for user to decide if all has to be cleared, clears all if needed.
 *
 * @return log file deletion status
 */
boolean
clearAllPersistentData()
{
  delay(1000);

  if (digitalRead(USER_SWITCH_PIN) == LOW)
  {
    // reset all counters
    counters.reset();
    stillnessDurationInLoopsCounter.reset();

    return sdLogger.reset();
  }
  return false;
}

/**
 * Internal function used to initialize takeoff switch
 */
void
initTakeOffSwitch()
{
  pinMode(TAKE_OFF_SWITCH_PIN, INPUT);
  digitalWrite(TAKE_OFF_SWITCH_PIN, HIGH);
}

/**
 * Takeoff detection (using takeoff switch)
 *
 * @return <tt>true</tt if takeoff has been detected, <tt>false</tt> else
 */
boolean
isAboutToTakeOff()
{
  return (digitalRead(TAKE_OFF_SWITCH_PIN) == HIGH);
}

/**
 * Internal function used to initialize user switch
 */
/**
 * Internal function used to initialize debug
 */
/**
 * Internal function used to initialize SD
 */
/**
 * Internal function used to initialize GPS serial
 */
void
switchToNextFlightPhase()
{
  currentFlightPhaseCounter.increment(1);
  currentFlightPhaseDurationCounter.set(0);
}

void
commonLoop()
{
  debugInfo("@CL >\r\n", 7);

  /* Loop start sequence */

  /*
   redLED.quicklyMakeBlinkSeveralTimes(1);
   greenLED.quicklyMakeBlinkSeveralTimes(1);
   */
  delay(1000);

  /* kiwi frame building */
  kiwiFrameBuilder.buildKiwiFrame(kiwiFrame);

  /* kiwi frame transmission */
  //fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  debugInfo("\r\n", 2);
  debugInfo((char *)kiwiFrame, KIWI_FRAME_LENGTH);
  delay(50);
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  debugInfo((char *)kiwiFrame, KIWI_FRAME_LENGTH);
  delay(50);
  debugInfo((char *)kiwiFrame, KIWI_FRAME_LENGTH);
  delay(50);
  debugInfo("\r\n", 2);

  /*
   greenLED.quicklyMakeBlinkSeveralTimes(2);
   */

  /* positioning data reading (and debug) */
  nmeaGPS.readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer);

  /*
   blueLED.showStatus(nmeaGPS.getFix());
   */

  /* NMEA sentences logging */
  sdLogger.logMessage(nmeaRmcSentenceBuffer, false);
  sdLogger.logMessage(nmeaGgaSentenceBuffer, false);
  delay(500);

  /* NMEA sentences transmission */
  fskModulator.modulateBytes(nmeaRmcSentenceBuffer, strlen(nmeaRmcSentenceBuffer));
  fskModulator.modulateBytes(nmeaGgaSentenceBuffer, strlen(nmeaGgaSentenceBuffer));

  /*
   greenLED.quicklyMakeBlinkSeveralTimes(3);
   */

  /* custom frame building */
  customFrameBuilder.buildCustomFrame(customFrame);

  /* custom frame debug */SERIAL_DEBUG.print(customFrame);

  /* custom frame logging */
  sdLogger.logMessage(customFrame, false);
  /* pause half a second to ensure SD asynchronous writing to be finished */
  delay(500);

  /* custom frame transmission */
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  /*
   redLED.quicklyMakeBlinkSeveralTimes(1);
   */

  /* frame counter update */
  frameCounter.increment(1);

  debugInfo("@CL <\r\n", 7);
}

/**
 * Internal function called when detecting transition from flight phase 0 to 1
 */
void
flightPhase0to1Transition()
{
  debugInfo("@T-0-1\r\n", 8);
}


/**
 * Internal function called when detecting transition from flight phase 1 to 2
 */
void
flightPhase1to2Transition()
{
  debugInfo("@T-1-2\r\n", 8);
}


/**
 * Internal function called when detecting transition from flight phase 2 to 3
 */
void
flightPhase2to3Transition()
{
  debugInfo("@T-2-3\r\n", 8);
}



/**
 * Internal function called when detecting transition from flight phase 3 to 4
 */
void
flightPhase3to4Transition()
{
  debugInfo("@T-3-4\r\n", 8);
}

void
flightPhase4to5Transition()
{
  debugInfo("@T-4-5\r\n", 8);
}


/**
 * Flight phase 0 sub-loop.
 *
 * - cameras off, common loop, pause of 15s between frames
 * - exits when takeoff switch is activated
 *
 * @return <tt>true</tt> if takeoff switch is detected as activated, <tt>false</tt> else
 */
boolean
flightPhase0Loop()
{
  debugInfo("@P0L >\r\n", 8);


  /* Detecting take-off */
  if (isAboutToTakeOff())
  {
    debugInfo("@Takeoff!\r\n", 11);
    debugInfo("@P0L <\r\n", 8);
    return true;
  }

  delay(FLIGHT_PHASE_0_PAUSE_MILLIS);
  debugInfo("@P0L <\r\n", 8);
  return false;
}

/**
 * Flight phase 1 sub-loop.
 *
 * - camera recording (rotor to ground), common loop, no delay between frames
 * - exits when maximum altitude or duration are reached
 *
 * @return <tt>true</tt> if flight phase transition has been detected, <tt>false</tt> else
 */
boolean
flightPhase1Loop()
{
  debugInfo("@P1L >\r\n", 8);
  delay(FLIGHT_PHASE_1_PAUSE_MILLIS);
  debugInfo("@P1L <\r\n", 8);
  /* flight phase transition detection */
  //if (((nmeaGPS.getFix()) && (nmeaGPS.getAltitude() > FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER))
  //    || (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_1_MAX_SECONDS_DURATION))
  if (nmeaGPS.getFix())
  {
    if (nmeaGPS.getAltitude() > FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER)
      return true;
  }
  else
    return (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_1_MAX_SECONDS_DURATION);
}

/**
 * Flight phase 2 sub-loop.
 *
 * - camera 33% recording (rotor to horizon), common loop, no delay between frames
 * - exits when maximum altitude or duration are reached
 *
 * @return <tt>true</tt> if flight phase transition has been detected, <tt>false</tt> else
 */
boolean
flightPhase2Loop()
{
  debugInfo("@P2L >\r\n", 8);
  delay(FLIGHT_PHASE_2_PAUSE_MILLIS);
  debugInfo("@P2L <\r\n", 8);

  /* flight phase transition detection */

  if (nmeaGPS.getFix())
  {
    if (nmeaGPS.getAltitude() > FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER)
      return true;
  }
  else
    return (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_2_MAX_SECONDS_DURATION);

  return false;
}

/**
 * Flight phase 3 sub-loop.
 *
 * - camera recording (rotor to sky), common loop, no delay between frames
 * - exits when minimum altitude or maximum duration are reached
 *
 * @return <tt>true</tt> if flight phase transition has been detected, <tt>false</tt> else
 */
boolean
flightPhase3Loop()
{
  debugInfo("@P3L >\r\n", 8);
  delay(FLIGHT_PHASE_3_PAUSE_MILLIS);
  debugInfo("@P3L <\r\n", 8);

  /* flight phase transition detection */
  if ((nmeaGPS.getFix()) && (nmeaGPS.getAltitude() < FLIGHT_PHASE_3_TO_4_ALTITUDE_TRIGGER))
  {
    previousAltitude = nmeaGPS.getAltitude();
    return true;
  }
  return false;
}

/**
 * Flight phase 4 sub-loop.
 *
 * - camera recording (rotor to ground), common loop, no delay between frames
 * - exits when ground or maximum duration are reached
 *
 * @return <tt>true</tt> if flight phase transition has been detected, <tt>false</tt> else
 */
boolean
flightPhase4Loop()
{
  debugInfo("@P4L >\r\n", 8);
  delay(FLIGHT_PHASE_4_PAUSE_MILLIS);
  debugInfo("@P4L <\r\n", 8);

  /* flight phase transition detection */
  if (nmeaGPS.getFix())
  {
    int deltaAltitude = previousAltitude - nmeaGPS.getAltitude();
    previousAltitude = nmeaGPS.getAltitude();

    if (deltaAltitude < DELTA_ALTITUDE_IN_METERS_CONSIDERED_AS_STILLNESS)
      stillnessDurationInLoopsCounter.increment(1);
    else
      stillnessDurationInLoopsCounter.set(0);

    return (stillnessDurationInLoopsCounter.read() > STILLNESS_DURATION_IN_LOOPS_LIMIT);
  }
  return false;
}

/**
 * Flight phase 5 sub-loop.
 *
 * - camera off, common loop, 15s between frames
 *
 * @return <tt>false</tt>
 */
boolean
flightPhase5Loop()
{
  debugInfo("@P5L >\r\n", 8);
  delay(FLIGHT_PHASE_5_PAUSE_MILLIS);
  debugInfo("@P5L <\r\n", 8);
  return false;
}

/**
 * Arduino's setup function, called once at startup, after init
 */
void
setup()
{
  initTakeOffSwitch();
  initUserSwitch();
  initDebugSerial();

  debugInfo("@Reset\n\r", 8);

  debugInfo("@Mario Time!\r\n", 14);
  playMarioTheme();

  /*
   leds.on();
   delay(1000);
   leds.off();
   */

  debugInfo("@SD_I...", 8);
  if (!initLogging())
  {
    debugInfo("KO\r\n", 4);
    /*
     orangeLED.showStatus(false);
     */
  }
  else
  {
    debugInfo("OK\r\n", 4);
    /*
     orangeLED.showStatus(true);
     */
  }

  if (clearAllPersistentData())
  {
    debugInfo("@Clear\r\n", 8);
    /*
     leds.quicklyMakeBlinkSeveralTimes(10);
     orangeLED.showStatus(true);
     */
  }
  else
  {
    debugInfo("@Restart\r\n", 10);
  }
  /*
   orangeLED.showStatus(LOGGER.logMessage("Reset", true));
   */

  initGpsSerial();
  previousAltitude = 0;

  debugInfo("@Cam_I\r\n", 8);
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  unsigned long startOfLoopTimeMillis = millis();

  commonLoop();

  switch (currentFlightPhaseCounter.read())
  {
    case BEFORE_TAKING_OFF_FLIGHT_PHASE:
      if (flightPhase0Loop())
      {
        debugInfo("@Mario Time!\r\n", 14);
        playMarioTheme();

        flightPhase0to1Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case ASCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      if (flightPhase1Loop())
      {
        debugInfo("@Mario Time!\r\n", 14);
        playMarioTheme();

        flightPhase1to2Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case ASCENDING_BETWEEN_LOWER_AND_UPPER_LIMIT_FLIGHT_PHASE:
      if (flightPhase2Loop())
      {
        debugInfo("@Mario Time!\r\n", 14);
        playMarioTheme();

        flightPhase2to3Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case BEFORE_BURST_FLIGHT_PHASE:
      if (flightPhase3Loop())
      {
        debugInfo("@Mario Time!\r\n", 14);
        playMarioTheme();

        flightPhase3to4Transition();
        switchToNextFlightPhase();
        return;
      }
      break;
    case DESCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      if (flightPhase4Loop())
      {
        debugInfo("@Mario Time!\r\n", 14);
        playMarioTheme();

        flightPhase4to5Transition();
        switchToNextFlightPhase();
        return;
      }
      break;
    case AFTER_LANDING_FLIGHT_PHASE:
      flightPhase5Loop();
      break;
  }

  currentFlightPhaseDurationCounter.increment(((millis() - startOfLoopTimeMillis) * 5) / 2500);
}

/**
 * Application's main (what else to say?)
 * @return (never)
 */
int
main(void)
{
  init();

  setup();

  for (;;)
    loop();

  return 0;
}
