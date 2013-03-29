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
#if defined(GPS_SERIAL_RX_PIN)
#include <SoftwareSerial.h>
#endif

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
#if defined(GPS_SERIAL_RX_PIN)
SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX_PIN, GPS_SERIAL_TX_PIN);
#else
#define serialNmeaGPSPort Serial1
#endif

GPS3D nmeaGPS(&serialNmeaGPSPort, &SERIAL_DEBUG);
char nmeaRmcSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];
char nmeaGgaSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];

// ---------------------------------
// FSK modulator related definitions
// ---------------------------------
FSK600BaudTA900TB1500Mod fskModulator(FSK_MODULATOR_TX_PIN);

// ----------------------------
// Counters related definitions
// ----------------------------
Counter frameCounter(FRAME_COUNTER_BASE_ADDRESS);
Counter resetCounter(RESET_COUNTER_BASE_ADDRESS);
Counter currentFlightPhaseCounter(CURRENT_FLIGHT_PHASE_COUNTER_BASE_ADDRESS);
Counter timeOfUnmovingBallCounter(
    TIME_OF_UNMOVING_BALL_COUNTER_BASE_ADDRESS);
Counter currentFlightPhaseDurationCounter (
    FLIGHT_PHASE_DURATION_COUNTER_BASE_ADDRESS);
Counter* countersArray[4] =
  { &frameCounter, &resetCounter, &currentFlightPhaseCounter,
      &currentFlightPhaseDurationCounter};
Counters counters(countersArray, 4);

// ----------------------------------
// Analog sensors related definitions
// ----------------------------------
// main sensors
AnalogSensor differentialPressureAnalogSensor(
    DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor absolutePressureAnalogSensor(
    ABSOLUTE_PRESSURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor externalTemperatureAnalogSensor(
    EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor internalTemperatureAnalogSensor(
    INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor* sensorsArray[4] =
  { &differentialPressureAnalogSensor, &absolutePressureAnalogSensor,
      &externalTemperatureAnalogSensor, &internalTemperatureAnalogSensor };
AnalogSensors sensors(sensorsArray, 4);

// HAO previous altitude (during ascending phase)
int previousAltitude;

// voltage sensor
// N.B. this analog sensor is handled seperately from the others since kiwi frame does
AnalogSensor voltage(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);

// -----------------------------------
// Real Time Clock related definitions
// -----------------------------------
DS1302_RTC rtc(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);

// --------------------------------
// Custom frame related definitions
// --------------------------------
char customFrame[CUSTOM_FRAME_MAX_LENGTH];
CustomFrameBuilder customFrameBuilder(&counters, &sensors, &voltage, &rtc,
    &nmeaGPS);

// ------------------------------
// KIWI frame related definitions
// ------------------------------
unsigned char kiwiFrame[KIWI_FRAME_LENGTH];
KiwiFrameBuilder kiwiFrameBuilder(&sensors, &voltage);

// ------------------------
// LEDs related definitions
// ------------------------
Led red_LED(RED_LED_PIN);
Led orange_LED(ORANGE_LED_PIN);
Led green_LED(GREEN_LED_PIN);
Led blue_LED(BLUE_LED_PIN);

Led* ledArray[4] =
  { &red_LED, &orange_LED, &green_LED, &blue_LED };
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
 * Internal function used to clear log file.
 * Waiting one second for user to decide if log file has to be deleted, deletes it if needed.
 *
 * @return log file has been deletion status
 */
boolean
deleteLogFileIfUserClaimsTo()
{
  delay(1000);

  // Reset counter update
  resetCounter.increment(1);

  if (digitalRead(USER_BUTTON_PIN) == LOW)
    {
      // reset all counters
      counters.reset();
      return LOGGER.reset();
    }
  return false;
}

/**
 * Internal function used to initialize user switch
 */
void
initUserButton()
{
  pinMode(USER_BUTTON_PIN, INPUT);
  digitalWrite(USER_BUTTON_PIN, HIGH);
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

void
commonLoop()
{
  SERIAL_DEBUG.println(F("@CL"));
  // show loop start sequence
  red_LED.quicklyMakeBlinkSeveralTimes(1);
  green_LED.quicklyMakeBlinkSeveralTimes(1);

  // kiwi frame building
  kiwiFrameBuilder.buildKiwiFrame(kiwiFrame);

  // kiwi frame transmission
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);

  green_LED.quicklyMakeBlinkSeveralTimes(2);

  // Positioning data reading (and debug)
  nmeaGPS.readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer);

  blue_LED.showStatus(nmeaGPS.getFix());

  // NMEA sentences logging
  LOGGER.logMessage(nmeaRmcSentenceBuffer, false);
  LOGGER.logMessage(nmeaGgaSentenceBuffer, false);
  delay(500);

  // NMEA sentences transmission
  fskModulator.modulateBytes(nmeaRmcSentenceBuffer,
      strlen(nmeaRmcSentenceBuffer));

  fskModulator.modulateBytes(nmeaGgaSentenceBuffer,
      strlen(nmeaGgaSentenceBuffer));

  green_LED.quicklyMakeBlinkSeveralTimes(3);

  // custom frame building
  customFrameBuilder.buildCustomFrame(customFrame);

  // custom frame debug
  SERIAL_DEBUG.print(customFrame);

  // custom frame logging
  LOGGER.logMessage(customFrame, false);
  delay(500);

  // custom frame transmission
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  red_LED.quicklyMakeBlinkSeveralTimes(1);
  delay(1000);

  // frame counter update
  frameCounter.increment(1);
}

bool
flightPhaseNumber0Loop()
{
  SERIAL_DEBUG.println(F("@P0L"));
  delay(FLIGHT_PHASE_0_PAUSE_DURATION);
  // DO STEP 0

/*******************************************************************************/
  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() > 50)
    {
      return true;
    }
/*******************************************************************************/
  // TODO check flight phase transition condition

  return false;
}

bool
flightPhaseNumber1Loop()
{
  SERIAL_DEBUG.println(F("@P1L"));
  delay(FLIGHT_PHASE_1_PAUSE_DURATION);
  // DO STEP 1


  if (nmeaGPS.getFix())
    {
      if (nmeaGPS.getAltitude() > FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER)
        return true;
    }

  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() == FLIGHT_PHASE_1_MAX_DURATION)
    {
      return true;
    }
  return false;
}

bool
flightPhaseNumber2Loop()
{
  SERIAL_DEBUG.println(F("@P2L"));
  delay(FLIGHT_PHASE_2_PAUSE_DURATION);
  // DO STEP 2

  if (nmeaGPS.getFix())
    {
      if (nmeaGPS.getAltitude() > FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER) {
      return true;
      previousAltitude = nmeaGPS.getAltitude();}
    }

  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() == FLIGHT_PHASE_2_MAX_DURATION)
    {
      return true;
    }
  return false;
}

bool
flightPhaseNumber3Loop()
{
  SERIAL_DEBUG.println(F("@P3L"));
  delay(FLIGHT_PHASE_3_PAUSE_DURATION);

  if (nmeaGPS.getFix())
    {
      int deltaAltitude = previousAltitude - nmeaGPS.getAltitude();
      previousAltitude = nmeaGPS.getAltitude();

      if (deltaAltitude > HAO_FALLING_TRIGGER)
        return true;
    }

  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() == FLIGHT_PHASE_3_MAX_DURATION)
    {
      return true;
    }
  return false;
}

bool
flightPhaseNumber4Loop()
{
  // DO STEP 4

  SERIAL_DEBUG.println(F("@P4L"));
  delay(FLIGHT_PHASE_4_PAUSE_DURATION);

  if (nmeaGPS.getFix())
    {
      if (nmeaGPS.getAltitude() < FLIGHT_PHASE_4_TO_5_ALTITUDE_TRIGGER)
        return true;
    }

  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() == FLIGHT_PHASE_4_MAX_DURATION)
    {
      return true;
    }

  return false;
}

bool
flightPhaseNumber5Loop()
{
  SERIAL_DEBUG.println(F("@P5L"));

  delay(FLIGHT_PHASE_5_PAUSE_DURATION);

  // DO STEP 5

  // Check ending moment of the phase with GPS (if fix ok)
  if (nmeaGPS.getFix())
    {
      int deltaAltitude = previousAltitude - nmeaGPS.getAltitude();
      previousAltitude = nmeaGPS.getAltitude();

      if (deltaAltitude < 100)
        timeOfUnmovingBallCounter.increment(1);
      else
        timeOfUnmovingBallCounter.set(0);

      if (timeOfUnmovingBallCounter.read()== TIME_LIMIT_OF_UNMOVING_BALL)
        return true;
    }

  // Check ending moment of the phase thanks to time.
  if (currentFlightPhaseDurationCounter.read() == FLIGHT_PHASE_5_MAX_DURATION)
    {
      return true;
    }

  return false;
}

bool
flightPhaseNumber6Loop()
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
  previousAltitude = 0;

  leds.on();
  delay(1000);
  leds.off();

  initUserButton();

  initDebugSerial();

  SERIAL_DEBUG.println(F("R"));

  SERIAL_DEBUG.print(F("SD Init..."));

  if (!initLogging())
    {
      SERIAL_DEBUG.println(F("KO"));
      orange_LED.showStatus(false);
    }
  else
    {
      SERIAL_DEBUG.println(F("OK"));
      orange_LED.showStatus(true);

      if (deleteLogFileIfUserClaimsTo())
        {
          SERIAL_DEBUG.println(F("SD Clear"));
          leds.quicklyMakeBlinkSeveralTimes(10);
          orange_LED.showStatus(true);
        }
    }

  orange_LED.showStatus(LOGGER.logMessage("R", true));

  initGpsSerial();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  unsigned long x = millis();
  bool incrementPhase;

  commonLoop();


  switch (currentFlightPhaseCounter.read())
    {
  case BEFORE_TAKING_OFF_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber0Loop();
    break;

  case ASCENDING_BELOW_5000M_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber1Loop();
    break;

  case ASCENDING_BETWEEN_5000M_AND_20000M_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber2Loop();
    break;

  case BEFORE_BURST_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber3Loop();
    break;
  case DESCENDING_ABOVE_5000M_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber4Loop();
    break;
  case BEFORE_LANDING_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber5Loop();
    break;
  case AFTER_LANDING_FLIGHT_PHASE:
    incrementPhase=flightPhaseNumber6Loop();
    break;
    }

  if (incrementPhase)
    {
      currentFlightPhaseCounter.increment(1);
      currentFlightPhaseDurationCounter.set(0);
    }
  else
    currentFlightPhaseDurationCounter.increment((millis() - x) / 1000);

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

