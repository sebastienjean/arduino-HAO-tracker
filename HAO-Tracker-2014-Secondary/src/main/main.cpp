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
// MAIN SECONDARY ==> COMMENT ADDED TO BUILD MAIN

// Globals includes
#include <pins.h>
#include <main/defs.h>

// Libs includes
#include <SD.h>

#include <Counter.h>
#include <Counters.h>
#include <core/Logger.h>
#include <SD/SDFileLogger.h>

#include <core/GPS.h>
#include <core/GPS3D.h>
#include <FCOEV2.h>

// Modules include
#include <modules/rotor/Rotor.h>
#include <modules/framebuilder/CustomFrameBuilder.h>

// -----------------------
// GPS related definitions
// -----------------------

/**
 * Serial port used for GPS
 */
#define serialNmeaGPSPort Serial1

SDFileLogger sdLogger(&SD, LOG_FILE_PATH);

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
 * Motorized camera fragment-at-frame persistent counter
 */
Counter motorizedCameraFragmentAtFrameCounter(MOTORIZED_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Ground camera fragment-at-frame persistent counter
 */
Counter groundCameraFragmentAtFrameCounter(GROUND_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Sky camera fragment-at-frame persistent counter
 */
Counter skyCameraFragmentAtFrameCounter(SKY_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Horizon camera fragment-at-frame persistent counter
 */
Counter horizonCameraFragmentAtFrameCounter(HORIZON_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Current flight phase duration persistent counter
 */
Counter currentFlightPhaseDurationCounter(FLIGHT_PHASE_DURATION_COUNTER_BASE_ADDRESS);

/**
 * Array of counters to be included in custom frame
 */
Counter* countersArray[8] =
  { &frameCounter,
    &resetCounter,
    &currentFlightPhaseCounter,
    &currentFlightPhaseDurationCounter,
    &motorizedCameraFragmentAtFrameCounter,
    &groundCameraFragmentAtFrameCounter,
    &skyCameraFragmentAtFrameCounter,
    &horizonCameraFragmentAtFrameCounter };

/**
 * External buffer used to build and send custom frame
 */
char customFrame[CUSTOM_FRAME_MAX_LENGTH];

/**
 * Counters to be included in custom frame
 */
Counters counters(countersArray, 8);

/**
 * Custom frame builder object
 */
CustomFrameBuilder customFrameBuilder(&counters);

// --------------------------
// Camera related definitions
// --------------------------
/**
 * Motorized camera object
 */
FCOEV2 motorizedCamera(MOTORIZED_CAMERA_PWM_PIN, MOTORIZED_CAMERA_PWR_PIN);

/**
 * Ground camera object
 */
FCOEV2 groundCamera(GROUND_CAMERA_PWM_PIN, GROUND_CAMERA_PWR_PIN);

/**
 * Sky camera object
 */
FCOEV2 skyCamera(SKY_CAMERA_PWM_PIN, SKY_CAMERA_PWR_PIN);

/**
 * Horizon camera object
 */
FCOEV2 horizonCamera(HORIZON_CAMERA_PWM_PIN, HORIZON_CAMERA_PWR_PIN);

/**
 * Rotor object
 */
Rotor rotor(ROTOR_PWM_PIN);

volatile unsigned long startOfLoopTimeMillis;

/**
 * Internal function used to send debug info both on debug serial and radio.
 * @param message a NUL-terminated string, supposed to include line-termination chars if needed.
 * @param string length (included line-termination chars)
 */
void
debugInfo(char *message, int chars)
{
  SERIAL_DEBUG.write((unsigned char *) message, chars);
}

/**
 * Internal function used to to initialize SD.
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
/**
 * Internal function used to clear persistent data (counters and SD) if requested by user at reset.
 *
 * @return <tt>true</tt> if all persistent data have been cleared
 */
boolean
clearAllPersistentDataOnRequest()
{
  delay(1000);

  if (digitalRead(USER_SWITCH_PIN) == LOW)
  {
    debugInfo("@Clear\r\n", 8);
    // reset all counters
    counters.reset();

    if (sdLogger.clear())
    {
      debugInfo("@SD Cleared\r\n", 13);
    }
    return true;
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
void
initUserSwitch()
{
  pinMode(USER_SWITCH_PIN, INPUT);
  digitalWrite(USER_SWITCH_PIN, HIGH);
}

/**
 * Internal function used to initialize debug
 */
void
initDebugSerial()
{
  SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUDRATE);
}

/**
 * Internal function used to initialize GPS serial
 */
void
initGpsSerial()
{
  serialNmeaGPSPort.begin(SERIAL_NMEA_GPS_BAUDRATE);
}

/**
 * Internal function used to initialize cameras
 * (i.e. set them to default or last known state)
 *
 */
void
initCameras()
{
  /* moving rotor to B, M, T */

  rotor.goBottom();
  delay(500);
  rotor.goMiddle();
  delay(500);
  rotor.goTop();
  delay(500);

  /* turning cameras off */

  debugInfo("@Cam-All-Off\r\n", 14);
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);
}

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

  /* custom frame building */
  customFrameBuilder.buildCustomFrame(customFrame);

  /* custom frame debug */
  SERIAL_DEBUG.print(customFrame);

  /* custom frame logging */
  sdLogger.logMessage(customFrame, false);

  /* pause half a second to ensure SD asynchronous writing to be finished */
  delay(500);

  /* positioning data reading (and debug) */
  nmeaGPS.readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer);

  /* NMEA sentences logging */
  sdLogger.logMessage(nmeaRmcSentenceBuffer, false);
  sdLogger.logMessage(nmeaGgaSentenceBuffer, false);
  delay(500);

  /*
   greenLED.quicklyMakeBlinkSeveralTimes(3);
   */

  /* frame counter update */
  frameCounter.increment(1);

  debugInfo("@CL <\r\n", 7);
}

/**
 * Cameras behavior during flight phase 0
 *
 * - All cameras are supposed to be off
 */
void
flightPhase0CameraProcessing()
{
  /* Does nothing special for the moment, except ensuring camera are off */
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);
}

/**
 * Internal function called when detecting transition from flight phase 0 to 1
 */
void
flightPhase0to1Transition()
{
  debugInfo("@T-0-1\r\n", 8);

  /* turning camera On, default mode is video */
  debugInfo("@Cam-All-Off\r\n", 13);
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  debugInfo("@Cam-All-On\r\n", 13);
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  horizonCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  motorizedCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  groundCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 1);
  skyCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 2);
  horizonCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 3);

  /* rotor in middle position */

  debugInfo("@Rotor-M\r\n", 10);
  rotor.goMiddle();

  /* starting recording */

  debugInfo("@Cam-All-Action\r\n", 17);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);
}

/**
 * Cameras behavior during flight phase 1
 *
 * - Motorized camera is in "ground" position
 * - all cameras are taking videos, fragmented every 15 loops
 */
void
flightPhase1CameraProcessing()
{
  if (frameCounter.read() == motorizedCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Rotor-M\r\n", 10);
    rotor.goMiddle();

    debugInfo("@Cam-M-VF\r\n", 11);
    debugInfo("@Cam-M-Off\r\n", 13);
    motorizedCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-M-On\r\n", 12);
    motorizedCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-M-A\r\n", 11);
    motorizedCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    motorizedCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == groundCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Cam-G-VF\r\n", 11);
    debugInfo("@Cam-G-Off\r\n", 13);
    groundCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-G-On\r\n", 12);
    groundCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-G-A\r\n", 11);
    groundCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    groundCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == skyCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Cam-S-VF\r\n", 11);
    debugInfo("@Cam-S-Off\r\n", 13);
    skyCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-S-On\r\n", 12);
    skyCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-S-A\r\n", 11);
    skyCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    skyCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == horizonCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Cam-H-VF\r\n", 11);
    debugInfo("@Cam-H-Off\r\n", 13);
    horizonCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-H-On\r\n", 12);
    horizonCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-H-A\r\n", 11);
    horizonCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    horizonCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }
}

/**
 * Internal function called when detecting transition from flight phase 1 to 2
 */
void
flightPhase1to2Transition()
{
  debugInfo("@T-1-2\r\n", 8);

  /* stopping recording */

  debugInfo("@Cam-All-Stop\r\n", 15);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  /* turning camera off and on again */

  debugInfo("@Cam-All-Off\r\n", 14);
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  debugInfo("@Cam-All-On\r\n", 13);
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  horizonCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  debugInfo("@Rotor-M\r\n", 10);
  rotor.goMiddle();

  debugInfo("@Cam-All-SeP\r\n", 14);
  motorizedCamera.switchToNextMode();
  groundCamera.switchToNextMode();
  skyCamera.switchToNextMode();
  horizonCamera.switchToNextMode();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  debugInfo("@Cam-All-Action\r\n", 17);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  horizonCameraFragmentAtFrameCounter.set(frameCounter.read() + (10 * VIDEO_FRAGMENTATION_LOOPS));

}

/**
 * Cameras behavior during flight phase 2
 *
 * - Motorized camera is in "horizon" position
 * - all cameras are taking videos, during 15 loops, with a pause of 30 loops between each
 */
void
flightPhase2CameraProcessing()
{
  if (frameCounter.read() == horizonCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@SeP-F\r\n", 8);
    flightPhase1to2Transition();
  }
}

/**
 * Internal function called when detecting transition from flight phase 2 to 3
 */
void
flightPhase2to3Transition()
{
  debugInfo("@T-2-3\r\n", 8);

  /* stopping recording */

  debugInfo("@Cam-All-Stop\r\n", 15);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  /* turning camera off and on again */

  debugInfo("@Cam-All-Off\r\n", 14);
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  debugInfo("@Cam-All-On\r\n", 13);
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  horizonCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  motorizedCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  groundCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 1);
  skyCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 2);
  horizonCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 2);

  debugInfo("@Rotor-T\r\n", 10);
  rotor.goTop();

  debugInfo("@Cam-All-Action\r\n", 17);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);
}

/**
 * Cameras behavior during flight phase 3
 *
 * - Motorized camera is in "sky" position
 * - all cameras are taking videos, fragmented every 15 loops
 */
void
flightPhase3CameraProcessing()
{
  if (frameCounter.read() == motorizedCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Rotor-T\r\n", 10);
    rotor.goTop();

    debugInfo("@Cam-M-VF\r\n", 11);
    debugInfo("@Cam-M-Off\r\n", 13);
    motorizedCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-M-On\r\n", 12);
    motorizedCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-M-A\r\n", 11);
    motorizedCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    motorizedCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == groundCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Cam-G-VF\r\n", 11);
    debugInfo("@Cam-G-Off\r\n", 13);
    groundCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-G-On\r\n", 12);
    groundCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-G-A\r\n", 11);
    groundCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    groundCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == skyCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Cam-S-VF\r\n", 11);
    debugInfo("@Cam-S-Off\r\n", 13);
    skyCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-S-On\r\n", 12);
    skyCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-S-A\r\n", 11);
    skyCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    skyCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == horizonCameraFragmentAtFrameCounter.read())
  {
    debugInfo("@Cam-H-VF\r\n", 11);
    debugInfo("@Cam-H-Off\r\n", 13);
    horizonCamera.switchOff();
    delay(SWITCH_MODE_PAUSE_MILLIS);
    debugInfo("@Cam-H-On\r\n", 12);
    horizonCamera.switchOn();
    delay(SWITCH_ON_PAUSE_MILLIS);
    debugInfo("@Cam-H-A\r\n", 11);
    horizonCamera.toggleAction();
    delay(SWITCH_MODE_PAUSE_MILLIS);

    horizonCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }
}

/**
 * Internal function called when detecting transition from flight phase 3 to 4
 */
void
flightPhase3to4Transition()
{
  debugInfo("@T-3-4\r\n", 8);

  /* stopping recording */
  debugInfo("@Cam-All-Stop\r\n", 15);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  /* turning camera off and on again */

  debugInfo("@Cam-All-Off\r\n", 14);
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  debugInfo("@Cam-All-On\r\n", 13);
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  horizonCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  motorizedCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  groundCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 1);
  skyCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 2);
  horizonCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS + 3);

  debugInfo("@Rotor-M\r\n", 10);
  rotor.goMiddle();

  debugInfo("@Cam-All-Action\r\n", 17);
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);
}

/**
 * Cameras behavior during flight phase 4
 *
 * - Motorized camera is in "ground" position
 * - all cameras are taking videos, fragmented every 15 loops
 */
void
flightPhase4CameraProcessing()
{
  // same as Phase 1
  flightPhase1CameraProcessing();
}

void
flightPhase4to5Transition()
{
  debugInfo("@T-4-5\r\n", 8);

  /* stopping recording */

  debugInfo("@Cam-All-Stop", 15);

  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  horizonCamera.toggleAction();

  /* turning camera off and on again */

  debugInfo("@Cam-All-Off", 14);
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);
}

/**
 * Cameras behavior during flight phase 5
 *
 * - All cameras are supposed to be off
 */
void
flightPhase5CameraProcessing()
{
  /* Does nothing special for the moment, except ensuring camera are off */
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  horizonCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);
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

  debugInfo("@P0L-C >\r\n", 10);
  flightPhase0CameraProcessing();
  debugInfo("@P0L-C <\r\n", 10);

  /* Detecting take-off */
  if (isAboutToTakeOff())
  {
    debugInfo("@Takeoff!\r\n", 11);
    delay(SWITCH_MODE_PAUSE_MILLIS);
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
  debugInfo("@P1L-C >\r\n", 10);
  flightPhase1CameraProcessing();
  debugInfo("@P1L-C <\r\n", 10);
  delay(FLIGHT_PHASE_1_PAUSE_MILLIS);
  debugInfo("@P1L <\r\n", 8);

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

  debugInfo("@P2L-C >\r\n", 10);
  flightPhase2CameraProcessing();
  debugInfo("@P2L-C <\r\n", 10);
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

  debugInfo("@P3L-C >\r\n", 10);
  flightPhase3CameraProcessing();
  debugInfo("@P3L-C <\r\n", 10);
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

  debugInfo("@P4L-C >\r\n", 10);
  flightPhase4CameraProcessing();
  debugInfo("@P4L-C <\r\n", 10);
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

  debugInfo("@P5L-C >\r\n", 10);
  flightPhase5CameraProcessing();
  debugInfo("@P5L-C <\r\n", 10);

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

  debugInfo("@SD_I...", 8);
  if (!initLogging())
    debugInfo("KO\r\n", 4);
  else
    debugInfo("OK\r\n", 4);

  if (!clearAllPersistentDataOnRequest())
  {
    debugInfo("@Restart\r\n", 10);
    resetCounter.increment(1);
  }

  initGpsSerial();
  previousAltitude = 0;

  debugInfo("@Cam_I\r\n", 8);
  initCameras();

  debugInfo("@CFP\r\n", 6);
  switch (currentFlightPhaseCounter.read())
  {
    case ASCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      flightPhase0to1Transition();
      break;
    case ASCENDING_BETWEEN_LOWER_AND_UPPER_LIMIT_FLIGHT_PHASE:
      flightPhase1to2Transition();
      break;
    case BEFORE_BURST_FLIGHT_PHASE:
      flightPhase2to3Transition();
      break;
    case DESCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      flightPhase3to4Transition();
      break;
    default:
      break;
  }
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  startOfLoopTimeMillis = millis();

  commonLoop();

  switch (currentFlightPhaseCounter.read())
  {
    case BEFORE_TAKING_OFF_FLIGHT_PHASE:
      if (flightPhase0Loop())
      {
        debugInfo("@flight step 0\r\n", 16);

        flightPhase0to1Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case ASCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      if (flightPhase1Loop())
      {
        debugInfo("@flight step 1\r\n", 16);

        flightPhase1to2Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case ASCENDING_BETWEEN_LOWER_AND_UPPER_LIMIT_FLIGHT_PHASE:
      if (flightPhase2Loop())
      {
        debugInfo("@flight step 2\r\n", 16);

        flightPhase2to3Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case BEFORE_BURST_FLIGHT_PHASE:
      if (flightPhase3Loop())
      {
        debugInfo("@flight step 3\r\n", 16);

        flightPhase3to4Transition();
        switchToNextFlightPhase();
        return;
      }
      break;
    case DESCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      if (flightPhase4Loop())
      {
        debugInfo("@flight step 4\r\n", 16);

        flightPhase4to5Transition();
        switchToNextFlightPhase();
        return;
      }
      break;
    case AFTER_LANDING_FLIGHT_PHASE:
      flightPhase5Loop();
      break;
  }

  currentFlightPhaseDurationCounter.increment((millis() - startOfLoopTimeMillis) / 1000);
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

