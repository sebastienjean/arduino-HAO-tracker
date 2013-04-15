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
#include "DS1302_RTC.h"

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

// HAO previous altitude (during ascending phase)
int previousAltitude;

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
Counter stillnessDurationInSecondsCounter(STILLNESS_DURATION_IN_SECONDS_COUNTER_BASE_ADDRESS);

/**
 * Motorized camera running status persistent counter
 */
Counter motorizedCameraRunningStatusCounter(MOTORIZED_CAMERA_RUNNING_STATUS_COUNTER_BASE_ADDRESS);

/**
 * Ground camera running status persistent counter
 */
Counter groundCameraRunningStatusCounter(GROUND_CAMERA_RUNNING_STATUS_COUNTER_BASE_ADDRESS);

/**
 * Sky camera running status persistent counter
 */
Counter skyCameraRunningStatusCounter(SKY_CAMERA_RUNNING_STATUS_COUNTER_BASE_ADDRESS);

/**
 * Motorized camera mode persistent counter
 */
Counter motorizedCameraModeCounter(MOTORIZED_CAMERA_MODE_COUNTER_BASE_ADDRESS);

/**
 * Ground camera mode persistent counter
 */
Counter groundCameraModeCounter(GROUND_CAMERA_MODE_COUNTER_BASE_ADDRESS);

/**
 * Sky camera mode persistent counter
 */
Counter skyCameraModeCounter(SKY_CAMERA_MODE_COUNTER_BASE_ADDRESS);

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
 * Differential pressure analog sensor
 */
AnalogSensor differentialPressureAnalogSensor(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);

/**
 * Absolute pressure analog sensor
 */
AnalogSensor absolutePressureAnalogSensor(ABSOLUTE_PRESSURE_ANALOG_SENSOR_CHANNEL);

/**
 * External temperature analog sensor
 */
AnalogSensor externalTemperatureAnalogSensor(EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * Internal temperature analog sensor
 */
AnalogSensor internalTemperatureAnalogSensor(INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * Voltage analogsensor
 */
// N.B. this analog sensor is handled separately from the others since kiwi frame does
AnalogSensor voltage(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);

/**
 * Array of analog sensors to be included in custom frame
 */
AnalogSensor* sensorsArray[4] =
  { &differentialPressureAnalogSensor,
    &absolutePressureAnalogSensor,
    &externalTemperatureAnalogSensor,
    &internalTemperatureAnalogSensor };

/**
 * Analog sensors to be included in custom frame
 */
AnalogSensors sensors(sensorsArray, 4);

// -----------------------------------
// Real Time Clock related definitions
// -----------------------------------

/**
 * RTC object
 */
DS1302_RTC rtc(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);

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
CustomFrameBuilder customFrameBuilder(&counters, &sensors, &voltage, &rtc, &nmeaGPS);

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
KiwiFrameBuilder kiwiFrameBuilder(&sensors, &voltage);

// --------------------------
// Camera related definitions
// --------------------------

/**
 * Motorized camera object
 */
FCOEV2 motorizedCamera(MOTORIZED_CAMERA_PWM_PIN);

/**
 * Ground camera object
 */
FCOEV2 groundCamera(GROUND_CAMERA_PWM_PIN);

/**
 * Sky camera object
 */
FCOEV2 skyCamera(SKY_CAMERA_PWM_PIN);

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
    stillnessDurationInSecondsCounter.reset();

    motorizedCameraRunningStatusCounter.reset();
    groundCameraRunningStatusCounter.reset();
    skyCameraRunningStatusCounter.reset();

    motorizedCameraModeCounter.reset();
    groundCameraModeCounter.reset();
    skyCameraModeCounter.reset();

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
  /* turning cameras off and on again (following advice) */
  motorizedCamera.switchOff();
  groundCamera.switchOff();
  skyCamera.switchOff();
  delay(500);
  motorizedCamera.switchOn();
  groundCamera.switchOn();
  skyCamera.switchOn();
  delay(SWITCH_ON_PAUSE_MILLIS);

  /* switching to last known mode */
  motorizedCamera.switchToMode(motorizedCameraModeCounter.read());
  groundCamera.switchToMode(groundCameraModeCounter.read());
  skyCamera.switchToMode(skyCameraModeCounter.read());

  /* toggling action if needed */
  if (motorizedCamera.getRunningStatus() == CAMERA_RUNNING)
  {
    motorizedCamera.toggleAction();
  }
  if (groundCamera.getRunningStatus() == CAMERA_RUNNING)
  {
    groundCamera.toggleAction();
  }
  if (skyCamera.getRunningStatus() == CAMERA_RUNNING)
  {
    skyCamera.toggleAction();
  }
}

void
commonLoop()
{
  SERIAL_DEBUG.println(F("@CL"));

  /* Loop start sequence */

  redLED.quicklyMakeBlinkSeveralTimes(1);
  greenLED.quicklyMakeBlinkSeveralTimes(1);

  /* kiwi frame building */
  kiwiFrameBuilder.buildKiwiFrame(kiwiFrame);

  /* kiwi frame transmission */
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
  delay(1000);

  /* frame counter update */
  frameCounter.increment(1);
}

  /**
   * Function for HAO's cameras.
   */
void
flightPhase1CameraProcessing()
{
  // TODO Servo -> GROUND
  if (motorizedCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (motorizedCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        motorizedCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCamMobile"));
        delay(1000);
        motorizedCamera.toggleAction();
      }
    }
    else
    {
      motorizedCamera.toggleAction();
    }
  }
  else
  {
    motorizedCamera.switchToMode(MODE_VIDEO);
  }

  // GROUND CAMERA MANAGEMENT
  if (groundCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (groundCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        groundCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCamGround"));
        delay(1000);
        groundCamera.toggleAction();
      }
    }
    else
    {
      groundCamera.toggleAction();
    }
  }
  else
  {
    groundCamera.switchToMode(MODE_VIDEO);
  }

  // sky camera management
  if (skyCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (skyCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        skyCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCAM3"));
        delay(1000);
        skyCamera.toggleAction();
      }
    }
    else
    {
      skyCamera.toggleAction();
    }
  }
  else
  {
    skyCamera.switchToMode(MODE_VIDEO);
  }
}

void
flightPhase2CameraProcessing()
{
  // mobile camera management
  // TODO Switch servo GROUND/HORIZON/SKY
  if (motorizedCamera.getCurrentMode() == MODE_PHOTO_SINGLE)
  {
    SERIAL_DEBUG.println(F("@MobilePhoto"));
    motorizedCamera.toggleAction();
  }
  else
  {
    motorizedCamera.switchToMode(MODE_PHOTO_SINGLE);
  }

  // ground camera management
        if (groundCamera.getCurrentMode() == MODE_PHOTO_SINGLE)
        {
          SERIAL_DEBUG.println(F("@GroundPhoto"));
          groundCamera.toggleAction();
        }
        else
        {
          groundCamera.switchToMode(MODE_PHOTO_SINGLE);
        }

        // sky camera management
        if (skyCamera.getCurrentMode() == MODE_PHOTO_SINGLE)
        {
          SERIAL_DEBUG.println(F("@SkyPhoto"));
          skyCamera.toggleAction();
        }
        else
        {
          skyCamera.switchToMode(MODE_PHOTO_SINGLE);
        }

      }

void
flightPhase3CameraProcessing()
{
  // mobile camera management
  // TODO Servo -> SKY
  if (motorizedCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (motorizedCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        motorizedCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCamMobile"));
        delay(1000);
        motorizedCamera.toggleAction();
      }
    }
    else
    {
      motorizedCamera.toggleAction();
    }
  }
  else
  {
    motorizedCamera.switchToMode(MODE_VIDEO);
  }

  // GROUND CAMERA MANAGEMENT
  if (groundCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (groundCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        groundCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCamGround"));
        delay(1000);
        groundCamera.toggleAction();
      }
    }
    else
    {
      groundCamera.toggleAction();
    }
  }
  else
  {
    groundCamera.switchToMode(MODE_VIDEO);
  }

  // sky camera management
  if (skyCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (skyCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        skyCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCAM3"));
        delay(1000);
        skyCamera.toggleAction();
      }
    }
    else
    {
      skyCamera.toggleAction();
    }
  }
  else
  {
    skyCamera.switchToMode(MODE_VIDEO);
  }
}

void
flightPhase4CameraProcessing()
{
  // mobile camera management
  // TODO Servo -> HORIZON
  if (motorizedCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (motorizedCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        motorizedCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCamMobile"));
        delay(1000);
        motorizedCamera.toggleAction();
      }
    }
    else
    {
      motorizedCamera.toggleAction();
    }
  }
  else
  {
    motorizedCamera.switchToMode(MODE_VIDEO);
  }

  // GROUND CAMERA MANAGEMENT
  if (groundCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (groundCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        groundCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCamGround"));
        delay(1000);
        groundCamera.toggleAction();
      }
    }
    else
    {
      groundCamera.toggleAction();
    }
  }
  else
  {
    groundCamera.switchToMode(MODE_VIDEO);
  }

  // sky camera management
  if (skyCamera.getCurrentMode() == MODE_VIDEO)
  {
    if (skyCamera.getRunningStatus())
    {
      if (frameCounter.read() % DELAY_FRAGMENTATION == 0)
      {
        skyCamera.toggleAction();
        SERIAL_DEBUG.println(F("@FragCAM3"));
        delay(1000);
        skyCamera.toggleAction();
      }
    }
    else
    {
      skyCamera.toggleAction();
    }
  }
  else
  {
    skyCamera.switchToMode(MODE_VIDEO);
  }
}

boolean
flightPhase0Loop()
{
  SERIAL_DEBUG.println(F("@P0L>"));

  /* Detecting take-off */
  if (digitalRead(TAKE_OFF_SWITCH_PIN) == LOW)
  {

     return true;
  }



      // Check ending moment of the phase thanks to time.
      if (currentFlightPhaseDurationCounter.read() > 20)
      {
        // Switch to mode video
        motorizedCamera.switchToMode(MODE_VIDEO);
        motorizedCameraModeCounter.set(MODE_VIDEO);
        groundCamera.switchToMode(MODE_VIDEO);
        groundCameraModeCounter.set(MODE_VIDEO);
        skyCamera.switchToMode(MODE_VIDEO);
        skyCameraModeCounter.set(MODE_VIDEO);

        // Start record
        motorizedCamera.toggleAction();
        motorizedCameraRunningStatusCounter.set(CAMERA_ON);
        groundCamera.toggleAction();
        groundCameraRunningStatusCounter.set(CAMERA_ON);
        skyCamera.toggleAction();
        skyCameraRunningStatusCounter.set(CAMERA_ON);

        return true;
      }
      /*******************************************************************************/
      // TODO check flight phase transition condition
      return false;
    }

boolean
flightPhase1Loop()
{
  SERIAL_DEBUG.println(F("@P1L"));
  delay(FLIGHT_PHASE_1_PAUSE_DURATION);

  // Management of cameras on phase 1
      flightPhase1CameraProcessing();

      if (nmeaGPS.getFix())
      {
        if (nmeaGPS.getAltitude() > FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER)
        {
          // stop record
          if (! motorizedCamera.getRunningStatus())
          motorizedCamera.toggleAction();
          if (! groundCamera.getRunningStatus())
          groundCamera.toggleAction();
          if (! skyCamera.getRunningStatus())
          skyCamera.toggleAction();

          // Switch to mode photo
          motorizedCamera.switchToMode(MODE_PHOTO_SINGLE);
          groundCamera.switchToMode(MODE_PHOTO_SINGLE);
          skyCamera.switchToMode(MODE_PHOTO_SINGLE);

          return true;
        }
      }

      // Check ending moment of the phase thanks to time.
      if (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_1_MAX_DURATION)
      {
        // stop record
        if (! motorizedCamera.getRunningStatus())
        motorizedCamera.toggleAction();
        if (! groundCamera.getRunningStatus())
        groundCamera.toggleAction();
        if (! skyCamera.getRunningStatus())
        skyCamera.toggleAction();

        // Switch to mode photo
        motorizedCamera.switchToMode(MODE_PHOTO_SINGLE);
        groundCamera.switchToMode(MODE_PHOTO_SINGLE);
        skyCamera.switchToMode(MODE_PHOTO_SINGLE);

        return true;
      }
      return false;
    }

boolean
flightPhase2Loop()
{
  SERIAL_DEBUG.println(F("@P2L"));
  delay(FLIGHT_PHASE_2_PAUSE_DURATION);

  /* Management of cameras on phase 2 */

  flightPhase2CameraProcessing();

  if (nmeaGPS.getFix())
  {
    if (nmeaGPS.getAltitude() > FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER)
    {
      // stop record
      if (! motorizedCamera.getRunningStatus())
      motorizedCamera.toggleAction();
      if (! groundCamera.getRunningStatus())
      groundCamera.toggleAction();
      if (! skyCamera.getRunningStatus())
      skyCamera.toggleAction();

      // Switch to mode video
      motorizedCamera.switchToMode(MODE_VIDEO);
      groundCamera.switchToMode(MODE_VIDEO);
      skyCamera.switchToMode(MODE_VIDEO);

      // Start record
      motorizedCamera.toggleAction();
      groundCamera.toggleAction();
      skyCamera.toggleAction();

      previousAltitude = nmeaGPS.getAltitude();
      return true;
    }
  }

  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_2_MAX_DURATION)
  {
    // stop record
    if (! motorizedCamera.getRunningStatus())
    motorizedCamera.toggleAction();
    if (! groundCamera.getRunningStatus())
    groundCamera.toggleAction();
    if (! skyCamera.getRunningStatus())
    skyCamera.toggleAction();

    // Switch to mode video
    motorizedCamera.switchToMode(MODE_VIDEO);
    groundCamera.switchToMode(MODE_VIDEO);
    skyCamera.switchToMode(MODE_VIDEO);

    // Start record
    motorizedCamera.toggleAction();
    groundCamera.toggleAction();
    skyCamera.toggleAction();

    return true;
  }
  return false;
}

boolean
flightPhase3Loop()
{
  SERIAL_DEBUG.println(F("@P3L"));
  delay(FLIGHT_PHASE_3_PAUSE_DURATION);

  // Management of cameras on phase 3
      flightPhase3CameraProcessing();

      if (nmeaGPS.getFix())
      {
        int deltaAltitude = previousAltitude - nmeaGPS.getAltitude();
        previousAltitude = nmeaGPS.getAltitude();

        if (deltaAltitude > HAO_FALLING_TRIGGER)
        {
          // stop record
          if (! motorizedCamera.getRunningStatus())
          motorizedCamera.toggleAction();
          if (! groundCamera.getRunningStatus())
          groundCamera.toggleAction();
          if (! skyCamera.getRunningStatus())
          skyCamera.toggleAction();

          // Switch to mode video
          motorizedCamera.switchToMode(MODE_VIDEO);
          groundCamera.switchToMode(MODE_VIDEO);
          skyCamera.switchToMode(MODE_VIDEO);

          // Start record
          motorizedCamera.toggleAction();
          groundCamera.toggleAction();
          skyCamera.toggleAction();

          return true;
        }
      }

      // Check ending moment of the phase thanks to time.
      if (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_3_MAX_DURATION)
      {
        // stop record
        if (! motorizedCamera.getRunningStatus())
        motorizedCamera.toggleAction();
        if (! groundCamera.getRunningStatus())
        groundCamera.toggleAction();
        if (! skyCamera.getRunningStatus())
        skyCamera.toggleAction();

        // Switch to mode video
        motorizedCamera.switchToMode(MODE_VIDEO);
        motorizedCameraModeCounter.set(MODE_VIDEO);
        groundCamera.switchToMode(MODE_VIDEO);
        groundCameraModeCounter.set(MODE_VIDEO);
        skyCamera.switchToMode(MODE_VIDEO);
        skyCameraModeCounter.set(MODE_VIDEO);

        // Start record
        motorizedCamera.toggleAction();
        motorizedCameraRunningStatusCounter.set(CAMERA_ON);
        groundCamera.toggleAction();
        groundCameraRunningStatusCounter.set(CAMERA_ON);
        skyCamera.toggleAction();
        skyCameraRunningStatusCounter.set(CAMERA_ON);

        return true;
      }
      return false;
    }

boolean
flightPhase4Loop()
{
  SERIAL_DEBUG.println(F("@P4L"));
  delay(FLIGHT_PHASE_4_PAUSE_DURATION);

  // Management of cameras on phase 4
      flightPhase4CameraProcessing();

      if (nmeaGPS.getFix())
      {
        if (nmeaGPS.getAltitude() < FLIGHT_PHASE_4_TO_5_ALTITUDE_TRIGGER)
        {
          // stop record
          if (! motorizedCamera.getRunningStatus())
          motorizedCamera.toggleAction();
          if (! groundCamera.getRunningStatus())
          groundCamera.toggleAction();
          if (! skyCamera.getRunningStatus())
          skyCamera.toggleAction();

          // Switch to mode video
          motorizedCamera.switchToMode(MODE_VIDEO);
          motorizedCameraModeCounter.set(MODE_VIDEO);
          groundCamera.switchToMode(MODE_VIDEO);
          groundCameraModeCounter.set(MODE_VIDEO);
          skyCamera.switchToMode(MODE_VIDEO);
          skyCameraModeCounter.set(MODE_VIDEO);

          // Start record
          motorizedCamera.toggleAction();
          motorizedCameraRunningStatusCounter.set(CAMERA_ON);
          groundCamera.toggleAction();
          groundCameraRunningStatusCounter.set(CAMERA_ON);
          skyCamera.toggleAction();
          skyCameraRunningStatusCounter.set(CAMERA_ON);

          return true;
        }
      }

      // Check ending moment of the phase thanks to time.
      if (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_4_MAX_DURATION)
      {
        // stop record
        if (! motorizedCamera.getRunningStatus())
        motorizedCamera.toggleAction();
        if (! groundCamera.getRunningStatus())
        groundCamera.toggleAction();
        if (! skyCamera.getRunningStatus())
        skyCamera.toggleAction();

        // Switch to mode video
        motorizedCamera.switchToMode(MODE_VIDEO);
        motorizedCameraModeCounter.set(MODE_VIDEO);
        groundCamera.switchToMode(MODE_VIDEO);
        groundCameraModeCounter.set(MODE_VIDEO);
        skyCamera.switchToMode(MODE_VIDEO);
        skyCameraModeCounter.set(MODE_VIDEO);

        // Start record
        motorizedCamera.toggleAction();
        motorizedCameraRunningStatusCounter.set(CAMERA_ON);
        groundCamera.toggleAction();
        groundCameraRunningStatusCounter.set(CAMERA_ON);
        skyCamera.toggleAction();
        skyCameraRunningStatusCounter.set(CAMERA_ON);

        return true;
      }

      return false;
    }

boolean
flightPhase5Loop()
{
  SERIAL_DEBUG.println(F("@P5L"));

  delay(FLIGHT_PHASE_5_PAUSE_DURATION);

  // Management of cameras on phase 4
      flightPhase4CameraProcessing();

  // Check ending moment of the phase with GPS (if fix ok)
      if (nmeaGPS.getFix())
      {
        int deltaAltitude = previousAltitude - nmeaGPS.getAltitude();
        previousAltitude = nmeaGPS.getAltitude();

        if (deltaAltitude < 100)
        stillnessDurationInSecondsCounter.increment(1);
        else
        stillnessDurationInSecondsCounter.set(0);

        if (stillnessDurationInSecondsCounter.read()> STILLNESS_DURATION_IN_SECONDS_LIMIT)
        return true;
      }

      // Check ending moment of the phase thanks to time.
      if (currentFlightPhaseDurationCounter.read() > FLIGHT_PHASE_5_MAX_DURATION)
      {
        return true;
      }

      return false;
    }

boolean
flightPhase6Loop()
{
  SERIAL_DEBUG.println(F("@P6L"));
  delay(FLIGHT_PHASE_6_PAUSE_DURATION);

  // DO STEP 6

      return false;
    }

  /**
   * Arduino's setup function, called once at startup, after init
   */
void
setup()
{
  leds.on();
  delay(1000);
  leds.off();

  initTakeOffSwitch();

  initUserSwitch();

  initDebugSerial();

  SERIAL_DEBUG.println(F("R"));

  SERIAL_DEBUG.print(F("SD Init..."));

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
    SERIAL_DEBUG.println(F("Clear"));
    leds.quicklyMakeBlinkSeveralTimes(10);
    orangeLED.showStatus(true);
  }

  orangeLED.showStatus(LOGGER.logMessage("R", true));

  initGpsSerial();

  previousAltitude = 0;

  initCameras();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  unsigned long startOfLoopTimeMillis = millis();
  boolean flightPhaseHasToBeIncremented;

  commonLoop();

  switch (currentFlightPhaseCounter.read())
  {
    case BEFORE_TAKING_OFF_FLIGHT_PHASE:
      flightPhaseHasToBeIncremented = flightPhase0Loop();
      break;

    case ASCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE:
      flightPhaseHasToBeIncremented = flightPhase1Loop();
      break;

    case ASCENDING_BETWEEN_LOWER_AND_UPPER_LIMIT_FLIGHT_PHASE :
      flightPhaseHasToBeIncremented = flightPhase2Loop();
      break;

    case BEFORE_BURST_FLIGHT_PHASE:
      flightPhaseHasToBeIncremented = flightPhase3Loop();
      break;
    case DESCENDING_ABOVE_5000M_FLIGHT_PHASE:
      flightPhaseHasToBeIncremented = flightPhase4Loop();
      break;
    case BEFORE_LANDING_FLIGHT_PHASE:
      flightPhaseHasToBeIncremented = flightPhase5Loop();
      break;
    case AFTER_LANDING_FLIGHT_PHASE:
      flightPhaseHasToBeIncremented = flightPhase6Loop();
      break;
  }

  if (flightPhaseHasToBeIncremented)
  {
    currentFlightPhaseCounter.increment(1);
    currentFlightPhaseDurationCounter.set(0);
  }
  else
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

