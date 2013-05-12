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
#include <GPS.h>
#include <GPS3D.h>
#include <FCOEV2.h>
#include <Rotor.h>

// Modules includes
#include "AnalogSensor.h"
#include "AnalogSensors.h"
#include "Led.h"
#include "Leds.h"
#include "Counter.h"
#include "Counters.h"
#include "KiwiFrameBuilder.h"
#include "CustomFrameBuilder.h"
#include "Logger.h"
#include "Tone.h"

// -----------------------
// GPS related definitions
// -----------------------

/**
 * Serial port used for GPS
 */
#define serialNmeaGPSPort Serial1

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
 * HAO previous altitude (during ascending phase)
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
 * Motorized camera recording status persistent counter
 */
Counter motorizedCameraRecordingStatusCounter(MOTORIZED_CAMERA_RECORDING_STATUS_COUNTER_BASE_ADDRESS);

/**
 * Ground camera recording status persistent counter
 */
Counter groundCameraRecordingStatusCounter(GROUND_CAMERA_RECORDING_STATUS_COUNTER_BASE_ADDRESS);

/**
 * Sky camera recording status persistent counter
 */
Counter skyCameraRecordingStatusCounter(SKY_CAMERA_RECORDING_STATUS_COUNTER_BASE_ADDRESS);

/**
 * Motorized camera mode persistent counter
 */
Counter motorizedCameraFragmentAtFrameCounter(MOTORIZED_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Motorized camera mode persistent counter
 */
Counter motorizedCameraOffAtFrameCounter(MOTORIZED_CAMERA_OFF_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Motorized camera mode persistent counter
 */
Counter motorizedCameraOnAtFrameCounter(MOTORIZED_CAMERA_ON_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Motorized camera mode persistent counter
 */
Counter motorizedCameraRecordAtFrameCounter(MOTORIZED_CAMERA_RECORD_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Ground camera mode persistent counter
 */
Counter groundCameraFragmentAtFrameCounter(GROUND_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Ground camera mode persistent counter
 */
Counter groundCameraOffAtFrameCounter(GROUND_CAMERA_OFF_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Ground camera mode persistent counter
 */
Counter groundCameraOnAtFrameCounter(GROUND_CAMERA_ON_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Ground camera mode persistent counter
 */
Counter groundCameraRecordAtFrameCounter(GROUND_CAMERA_RECORD_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Sky camera mode persistent counter
 */
Counter skyCameraFragmentAtFrameCounter(SKY_CAMERA_FRAGMENT_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Sky camera mode persistent counter
 */
Counter skyCameraOffAtFrameCounter(SKY_CAMERA_OFF_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Sky camera mode persistent counter
 */
Counter skyCameraOnAtFrameCounter(SKY_CAMERA_ON_AT_FRAME_COUNTER_BASE_ADDRESS);

/**
 * Sky camera mode persistent counter
 */
Counter skyCameraRecordAtFrameCounter(SKY_CAMERA_RECORD_AT_FRAME_COUNTER_BASE_ADDRESS);
/**
 * Current flight phase persistent counter
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
Counters counters(countersArray, 4);

// ----------------------------------
// Analog sensors related definitions
// ----------------------------------

/**
 * External temperature analog sensor
 */
AnalogSensor externalTemperatureAnalogSensor(EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * External humidity analog sensor
 */
AnalogSensor externalHumidityAnalogSensor(EXTERNAL_HUMIDITY_ANALOG_SENSOR_CHANNEL);

/**
 * Internal temperature analog sensor
 */
AnalogSensor internalTemperatureAnalogSensor(INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * Accurate luminosity analog sensor
 */
AnalogSensor upLuminosityAnalogSensor(UP_LUMINOSITY_ANALOG_SENSOR_CHANNEL);

/**
 * Coarse luminosity analog sensor, side 1
 */
AnalogSensor side1LuminosityAnalogSensor(SIDE1_LUMINOSITY_ANALOG_SENSOR_CHANNEL);

/**
 * Coarse luminosity analog sensor, side 2
 */
AnalogSensor side2LuminosityAnalogSensor(SIDE2_LUMINOSITY_ANALOG_SENSOR_CHANNEL);

/**
 * Differential pressure analog sensor
 */
AnalogSensor differentialPressureAnalogSensor(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);

/**
 * Sound level analog sensor
 */
AnalogSensor soundLevelAnalogSensor(SOUND_LEVEL_ANALOG_SENSOR_CHANNEL);

/**
 * Battery temperature analog sensor
 */
AnalogSensor batteryTemperatureAnalogSensor(BATTERY_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * Voltage analog sensor
 */
// N.B. this analog sensor is handled separately from the others since kiwi frame does
AnalogSensor voltage(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);

/**
 * Middle temperature analog sensor
 */
AnalogSensor middleTemperatureAnalogSensor(MIDDLE_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

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
AnalogSensors customFrameAnalogSensors(sensorsArray, 10);

/**
 * Analog sensors to be included in kiwi frame
 */
AnalogSensors kiwiFrameAnalogSensors(sensorsArray, 8);

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

Rotor rotor(ROTOR_PWM_PIN);

Tone toneGenerator(FSK_MODULATOR_TX_PIN);

// ------------------------
// LEDs related definitions
// ------------------------

/**
 * Red LED object
 */
Led redLED(RED_LED_PIN);

/**
 * Orange LED object
 */
Led orangeLED(ORANGE_LED_PIN);

/**
 * Green LED object
 */
Led greenLED(GREEN_LED_PIN);

/**
 * Blue LED object
 */
Led blueLED(BLUE_LED_PIN);

/**
 * Array of LEDs
 */
Led* ledArray[4] =
  { &redLED,
    &orangeLED,
    &greenLED,
    &blueLED };
Leds leds(ledArray, 4);

/**
 *
 * @return
 */
void
playIntro()
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

/**
 * Internal function used to initialize logging (SD).
 *
 * @return logging initialization success status
 */
boolean
initLogging()
{
  return LOGGER.begin(LOG_FILE_PATH, SD_CHIP_SELECT_PIN);
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

    motorizedCameraRecordingStatusCounter.reset();
    groundCameraRecordingStatusCounter.reset();
    skyCameraRecordingStatusCounter.reset();

    return LOGGER.reset();
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
  rotor.goBottom();
  delay(500);
  rotor.goMiddle();
  delay(500);
  rotor.goTop();
  delay(500);

  /* turning cameras off and on again (following Roy's advice) */
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  delay(500);
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  /* restarting recording if needed */
  if (motorizedCameraRecordingStatusCounter.read() == CAMERA_RUNNING)
  {
    motorizedCamera.toggleAction();
  }
  if (groundCameraRecordingStatusCounter.read() == CAMERA_RUNNING)
  {
    groundCamera.toggleAction();
  }
  if (skyCameraRecordingStatusCounter.read() == CAMERA_RUNNING)
  {
    skyCamera.toggleAction();
  }
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
  SERIAL_DEBUG.println(F("@CL >"));

  /* Loop start sequence */

  redLED.quicklyMakeBlinkSeveralTimes(1);
  greenLED.quicklyMakeBlinkSeveralTimes(1);

  /* kiwi frame building */
  kiwiFrameBuilder.buildKiwiFrame(kiwiFrame);

  /* kiwi frame transmission */
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  delay(250);
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  delay(250);
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);

  greenLED.quicklyMakeBlinkSeveralTimes(2);

  /* positioning data reading (and debug) */
  nmeaGPS.readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer);

  blueLED.showStatus(nmeaGPS.getFix());

  /* NMEA sentences logging */
  LOGGER.logMessage(nmeaRmcSentenceBuffer, false);
  LOGGER.logMessage(nmeaGgaSentenceBuffer, false);
  delay(500);

  /* NMEA sentences transmission */
  fskModulator.modulateBytes(nmeaRmcSentenceBuffer, strlen(nmeaRmcSentenceBuffer));
  fskModulator.modulateBytes(nmeaGgaSentenceBuffer, strlen(nmeaGgaSentenceBuffer));

  greenLED.quicklyMakeBlinkSeveralTimes(3);

  /* custom frame building */
  customFrameBuilder.buildCustomFrame(customFrame);

  /* custom frame debug */
  SERIAL_DEBUG.print(customFrame);

  /* custom frame logging */
  LOGGER.logMessage(customFrame, false);
  /* pause half a second to ensure SD asynchronous writing to be finished */
  delay(500);

  /* custom frame transmission */
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  redLED.quicklyMakeBlinkSeveralTimes(1);

  /* frame counter update */
  frameCounter.increment(1);
  SERIAL_DEBUG.println(F("@CL <"));
}

  /**
   * Cameras behavior during flight phase 1
   *
   * - All cameras are supposed to be off
   */
void
flightPhase0CameraProcessing()
{
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  /* Does nothing special for the moment */
}

/**
 * Internal function called when detecting transition from flight phase 0 to 1
 */
void
flightPhase0to1Transition()
{
  playIntro();

  SERIAL_DEBUG.println(F("@Transition-0-1"));

  /* turning camera On, default mode is video */
  SERIAL_DEBUG.println(F("@Cam-All-On"));
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  motorizedCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  groundCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS+1);
  skyCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS+2);

  /* rotor in bottom position */
  rotor.goBottom();

  /* starting recording */
  SERIAL_DEBUG.println(F("@Cam-All-Action"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  motorizedCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  groundCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  skyCameraRecordingStatusCounter.set(CAMERA_RUNNING);
}

  /**
   * Cameras behavior during flight phase 1
   *
   * - Motorized camera is in "sky" position
   * - all cameras are taking videos, fragmented every 5 loops
   */
void
flightPhase1CameraProcessing()
{
  if (frameCounter.read() == motorizedCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-M-Video-fragment"));
    motorizedCamera.toggleAction();
    rotor.goBottom();
    delay(2000);
    motorizedCamera.toggleAction();
    motorizedCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == groundCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-G-Video-fragment"));
    groundCamera.toggleAction();
    delay(2000);
    groundCamera.toggleAction();
    groundCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == skyCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-S-Video-fragment"));
    skyCamera.toggleAction();
    delay(2000);
    skyCamera.toggleAction();
    skyCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }
}

    /**
     * Internal function called when detecting transition from flight phase 1 to 2
     */
void
flightPhase1to2Transition()
{
  playIntro();

  SERIAL_DEBUG.println(F("@Transition-1-2"));

  /* stopping recording */
  SERIAL_DEBUG.println(F("@Cam-All-StopRecording"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  motorizedCameraRecordingStatusCounter.set(CAMERA_IDLE);
  groundCameraRecordingStatusCounter.set(CAMERA_IDLE);
  skyCameraRecordingStatusCounter.set(CAMERA_IDLE);

  /* turning camera off and on again */
  SERIAL_DEBUG.println(F("@Cam-All-Off"));
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  SERIAL_DEBUG.println(F("@Cam-All-On"));
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  rotor.goMiddle();

  SERIAL_DEBUG.println(F("@Cam-All-Action"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  motorizedCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  groundCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  skyCameraRecordingStatusCounter.set(CAMERA_RUNNING);

  motorizedCameraOffAtFrameCounter.set(frameCounter.read()+VIDEO_FRAGMENTATION_LOOPS);
  motorizedCameraOnAtFrameCounter.reset();
  motorizedCameraRecordAtFrameCounter.reset();
  groundCameraOffAtFrameCounter.set(frameCounter.read()+VIDEO_FRAGMENTATION_LOOPS+1);
  groundCameraOnAtFrameCounter.reset();
  groundCameraRecordAtFrameCounter.reset();
  skyCameraOffAtFrameCounter.set(frameCounter.read()+VIDEO_FRAGMENTATION_LOOPS+2);
  skyCameraOnAtFrameCounter.reset();
  skyCameraRecordAtFrameCounter.reset();
}

void
flightPhase2CameraProcessing()
{
  if (frameCounter.read() == motorizedCameraOffAtFrameCounter.read())
  {
    motorizedCamera.toggleAction();
    delay(1000);
    motorizedCamera.switchOn();
    motorizedCameraRecordingStatusCounter.set(CAMERA_IDLE);
    motorizedCameraOnAtFrameCounter.set(frameCounter.read() + (2 * VIDEO_FRAGMENTATION_LOOPS));
  }

  if (frameCounter.read() == motorizedCameraOnAtFrameCounter.read())
  {
    motorizedCamera.switchOn();
    // no use to wait a lot, the camera will power up during next loop
    delay(SWITCH_MODE_PAUSE_MILLIS);
    delay(SWITCH_MODE_PAUSE_MILLIS);
    motorizedCameraRecordAtFrameCounter.set(frameCounter.read() + 1);
  }

  if (frameCounter.read() == motorizedCameraRecordAtFrameCounter.read())
  {
    rotor.goMiddle();
    motorizedCamera.toggleAction();
    motorizedCameraRecordingStatusCounter.set(CAMERA_RUNNING);
    motorizedCameraOffAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == groundCameraOffAtFrameCounter.read())
  {
    groundCamera.toggleAction();
    delay(1000);
    groundCameraRecordingStatusCounter.set(CAMERA_IDLE);
    groundCameraOnAtFrameCounter.set(frameCounter.read() + (2 * VIDEO_FRAGMENTATION_LOOPS));
  }

  if (frameCounter.read() == groundCameraOnAtFrameCounter.read())
  {
    groundCamera.switchOn();
    // no use to wait a lot, the camera will power up during next loop
    delay(SWITCH_MODE_PAUSE_MILLIS);
    delay(SWITCH_MODE_PAUSE_MILLIS);
    groundCameraRecordAtFrameCounter.set(frameCounter.read() + 1);
  }

  if (frameCounter.read() == groundCameraRecordAtFrameCounter.read())
  {
    groundCamera.toggleAction();
    groundCameraRecordingStatusCounter.set(CAMERA_RUNNING);
    groundCameraOffAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == skyCameraOffAtFrameCounter.read())
  {
    skyCamera.toggleAction();
    delay(1000);
    skyCameraRecordingStatusCounter.set(CAMERA_IDLE);
    skyCameraOnAtFrameCounter.set(frameCounter.read() + (2 * VIDEO_FRAGMENTATION_LOOPS));
  }

  if (frameCounter.read() == skyCameraOnAtFrameCounter.read())
  {
    skyCamera.switchOn();
    // no use to wait a lot, the camera will power up during next loop
    delay(SWITCH_MODE_PAUSE_MILLIS);
    delay(SWITCH_MODE_PAUSE_MILLIS);
    skyCameraRecordAtFrameCounter.set(frameCounter.read() + 1);
  }

  if (frameCounter.read() == skyCameraRecordAtFrameCounter.read())
  {
    skyCamera.toggleAction();
    skyCameraRecordingStatusCounter.set(CAMERA_RUNNING);
    skyCameraOffAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  }
}

/**
 * Internal function called when detecting transition from flight phase 2 to 3
 */
void
flightPhase2to3Transition()
{
  playIntro();

  SERIAL_DEBUG.println(F("@Transition-2-3"));

  /* stopping recording */
  SERIAL_DEBUG.println(F("@Cam-All-StopRecording"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  motorizedCameraRecordingStatusCounter.set(CAMERA_IDLE);
  groundCameraRecordingStatusCounter.set(CAMERA_IDLE);
  skyCameraRecordingStatusCounter.set(CAMERA_IDLE);

  /* turning camera off and on again */
  SERIAL_DEBUG.println(F("@Cam-All-Off"));
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  SERIAL_DEBUG.println(F("@Cam-All-On"));
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  motorizedCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  groundCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS+1);
  skyCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS+2);

  SERIAL_DEBUG.println(F("@Cam-All-Action"));
  rotor.goTop();
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);
  motorizedCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  groundCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  skyCameraRecordingStatusCounter.set(CAMERA_RUNNING);
}

void
flightPhase3CameraProcessing()
{
  if (frameCounter.read() == motorizedCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-M-Video-fragment"));
    motorizedCamera.toggleAction();
    rotor.goTop();
    delay(2000);
    motorizedCamera.toggleAction();
    motorizedCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == groundCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-G-Video-fragment"));
    groundCamera.toggleAction();
    delay(2000);
    groundCamera.toggleAction();
    groundCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == skyCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-S-Video-fragment"));
    skyCamera.toggleAction();
    delay(2000);
    skyCamera.toggleAction();
    skyCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }
}

    /**
     * Internal function called when detecting transition from flight phase 3 to 4
     */
void
flightPhase3to4Transition()
{
  playIntro();

  SERIAL_DEBUG.println(F("@Transition-3-4"));

  /* stopping recording */
  SERIAL_DEBUG.println(F("@Cam-All-StopRecording"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  motorizedCameraRecordingStatusCounter.set(CAMERA_IDLE);
  groundCameraRecordingStatusCounter.set(CAMERA_IDLE);
  skyCameraRecordingStatusCounter.set(CAMERA_IDLE);

  /* turning camera off and on again */
  SERIAL_DEBUG.println(F("@Cam-All-Off"));
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  SERIAL_DEBUG.println(F("@Cam-All-On"));
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  motorizedCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS);
  groundCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS+1);
  skyCameraFragmentAtFrameCounter.set(frameCounter.read() + VIDEO_FRAGMENTATION_LOOPS+2);

  rotor.goBottom();

  SERIAL_DEBUG.println(F("@Cam-All-Action"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();
  delay(SWITCH_MODE_PAUSE_MILLIS);

  motorizedCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  groundCameraRecordingStatusCounter.set(CAMERA_RUNNING);
  skyCameraRecordingStatusCounter.set(CAMERA_RUNNING);
}

void
flightPhase4CameraProcessing()
{
  if (frameCounter.read() == motorizedCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-M-Video-fragment"));
    motorizedCamera.toggleAction();
    rotor.goBottom();
    delay(2000);
    motorizedCamera.toggleAction();
    motorizedCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == groundCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-G-Video-fragment"));
    groundCamera.toggleAction();
    delay(2000);
    groundCamera.toggleAction();
    groundCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }

  if (frameCounter.read() == skyCameraFragmentAtFrameCounter.read())
  {
    SERIAL_DEBUG.println(F("@Cam-S-Video-fragment"));
    skyCamera.toggleAction();
    delay(2000);
    skyCamera.toggleAction();
    skyCameraFragmentAtFrameCounter.increment(VIDEO_FRAGMENTATION_LOOPS);
  }
}

void
flightPhase4to5Transition()
{
  playIntro();

  SERIAL_DEBUG.println(F("@Transition-4-5"));

  /* stopping recording */
  SERIAL_DEBUG.println(F("@Cam-All-StopRecording"));
  motorizedCamera.toggleAction();
  groundCamera.toggleAction();
  skyCamera.toggleAction();

  motorizedCameraRecordingStatusCounter.set(CAMERA_IDLE);
  groundCameraRecordingStatusCounter.set(CAMERA_IDLE);
  skyCameraRecordingStatusCounter.set(CAMERA_IDLE);

  /* turning camera off and on again */
  SERIAL_DEBUG.println(F("@Cam-All-Off"));
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
}

  /**
   * Flight phase 0 sub-loop.
   *
   * - does nothing special but pausing for 30 seconds
   * - exits when takeoff switch is activated
   *
   * @return <tt>true</tt> if takeoff switch is detected as activated, <tt>false</tt> else
   */
boolean
flightPhase0Loop()
{
  SERIAL_DEBUG.println(F("@P0L >"));

  SERIAL_DEBUG.println(F("@P0L-C >"));
  flightPhase0CameraProcessing();
  SERIAL_DEBUG.println(F("@P0L-C <"));

  /* Detecting take-off */
  if (isAboutToTakeOff())
  {
    SERIAL_DEBUG.println(F("@About to take off!"));
    delay(2000);
    SERIAL_DEBUG.println(F("@P0L <"));
    return true;
  }

  delay(FLIGHT_PHASE_0_PAUSE_MILLIS);
  SERIAL_DEBUG.println(F("@P0L <"));
  return false;
}

  /**
   * Flight phase 1 sub-loop.
   *
   * - ...
   * - exits when ...
   *
   * @return <tt>true</tt> if flight phase transition has been detected, <tt>false</tt> else
   */
boolean
flightPhase1Loop()
{
  SERIAL_DEBUG.println(F("@P1L >"));

  SERIAL_DEBUG.println(F("@P1L-C >"));
  flightPhase1CameraProcessing();
  SERIAL_DEBUG.println(F("@P1L-C <"));

  /* flight phase transition detection */
  if (((nmeaGPS.getFix()) && (nmeaGPS.getAltitude() > FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER))|| (currentFlightPhaseDurationCounter.read() >FLIGHT_PHASE_1_MAX_SECONDS_DURATION))
  {
    SERIAL_DEBUG.println(F("@P1L <"));
    return true;
  }

  delay(FLIGHT_PHASE_1_PAUSE_MILLIS);
  SERIAL_DEBUG.println(F("@P1L <"));
  return false;
}

boolean
flightPhase2Loop()
{
  SERIAL_DEBUG.println(F("@P2L >"));

  SERIAL_DEBUG.println(F("@P2L-C >"));
  flightPhase2CameraProcessing();
  SERIAL_DEBUG.println(F("@P2L-C <"));

  /* flight phase transition detection */

  if (((nmeaGPS.getFix())&&(nmeaGPS.getAltitude() > FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER))||(currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_2_MAX_SECONDS_DURATION))
  {
    SERIAL_DEBUG.println(F("@P2L <"));
    return true;
  }
  delay(FLIGHT_PHASE_2_PAUSE_MILLIS);
  SERIAL_DEBUG.println(F("@P2L <"));
  return false;
}

boolean
flightPhase3Loop()
{
  SERIAL_DEBUG.println(F("@P3L >"));

  SERIAL_DEBUG.println(F("@P3L-C >"));
  flightPhase3CameraProcessing();
  SERIAL_DEBUG.println(F("@P3L-C <"));

  /* flight phase transition detection */
  if ((nmeaGPS.getFix())&&(nmeaGPS.getAltitude() < FLIGHT_PHASE_3_TO_4_ALTITUDE_TRIGGER))
  {
    SERIAL_DEBUG.println(F("@P3L <"));
    previousAltitude = nmeaGPS.getAltitude();
    return true;
  }
  delay(FLIGHT_PHASE_3_PAUSE_MILLIS);
  SERIAL_DEBUG.println(F("@P3L <"));
  return false;
}

boolean
flightPhase4Loop()
{
  SERIAL_DEBUG.println(F("@P4L >"));

  SERIAL_DEBUG.println(F("@P4L-C >"));
  flightPhase4CameraProcessing();
  SERIAL_DEBUG.println(F("@P4L-C <"));

  /* flight phase transition detection */
  if (nmeaGPS.getFix())
  {
    int deltaAltitude = previousAltitude - nmeaGPS.getAltitude();
    previousAltitude = nmeaGPS.getAltitude();

    if (deltaAltitude < DELTA_ALTITUDE_IN_METERS_CONSIDERED_AS_STILLNESS)
    stillnessDurationInLoopsCounter.increment(1);
    else
    stillnessDurationInLoopsCounter.set(0);

    if (stillnessDurationInLoopsCounter.read()> STILLNESS_DURATION_IN_LOOPS_LIMIT)
    SERIAL_DEBUG.println(F("@P4L <"));
    return true;
  }

  delay(FLIGHT_PHASE_4_PAUSE_MILLIS);
  SERIAL_DEBUG.println(F("@P4L <"));
  return false;
}

boolean
flightPhase5Loop()
{
  SERIAL_DEBUG.println(F("@P5L >"));
  delay(FLIGHT_PHASE_5_PAUSE_MILLIS);
  SERIAL_DEBUG.println(F("@P5L <"));
  return false;
}

/**
 * Arduino's setup function, called once at startup, after init
 */
void
setup()
{
  playIntro();

  leds.on();
  delay(1000);
  leds.off();

  initTakeOffSwitch();

  initUserSwitch();

  initDebugSerial();

  SERIAL_DEBUG.println(F("@Reset"));

  SERIAL_DEBUG.print(F("@SD_I..."));

  if (!initLogging())
  {
    SERIAL_DEBUG.println(F("KO"));
    orangeLED.showStatus(false);
  }
  else
  {
    SERIAL_DEBUG.println(F("OK"));
    orangeLED.showStatus(true);
  }

  if (clearAllPersistentData())
  {
    SERIAL_DEBUG.println(F("@Clear"));
    leds.quicklyMakeBlinkSeveralTimes(10);
    orangeLED.showStatus(true);
  }
  else
  {
    SERIAL_DEBUG.println(F("@Restart"));
  }

  orangeLED.showStatus(LOGGER.logMessage("Reset", true));

  initGpsSerial();

  previousAltitude = 0;


  SERIAL_DEBUG.println(F("@Cam_I"));
  initCameras();
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
        flightPhase0to1Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case ASCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      if (flightPhase1Loop())
      {
        flightPhase1to2Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case ASCENDING_BETWEEN_LOWER_AND_UPPER_LIMIT_FLIGHT_PHASE:
      if (flightPhase2Loop())
      {
        flightPhase2to3Transition();
        switchToNextFlightPhase();
        return;
      }
      break;

    case BEFORE_BURST_FLIGHT_PHASE:
      if (flightPhase3Loop())
      {
        flightPhase3to4Transition();
        switchToNextFlightPhase();
        return;
      }
      break;
    case DESCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      if (flightPhase4Loop())
      {
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

