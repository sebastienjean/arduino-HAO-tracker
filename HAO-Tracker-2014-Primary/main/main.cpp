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
#include <SDFileLogger.h>

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

SDFileLogger sdFileLogger(&SD, LOG_FILE_PATH);

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
 * Array of counters to be included in custom frame
 */
Counter* countersArray[2] =
  { &frameCounter,
    &resetCounter, };

/**
 * Counters to be included in custom frame
 */
Counters counters((Counter **) &countersArray, 2);

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
 * Internal function used to initialize serial debug.
 */
void
initDebugSerial()
{
  SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUDRATE);
}

/**
 * Internal function used to initialize GPS serial port.
 */
void
initGpsSerial()
{
  serialNmeaGPSPort.begin(SERIAL_NMEA_GPS_BAUDRATE);
}

/**
 * Internal function used to initialize user switch.
 */
void
initUserSwitch()
{
  pinMode(USER_SWITCH_PIN, INPUT);
  digitalWrite(USER_SWITCH_PIN, HIGH);
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

    if (sdFileLogger.clear())
    {
      debugInfo("@SD Cleared\r\n", 13);
    }
    return true;
  }
  return false;
}

/**
 * Arduino's setup function, called once at startup, after init
 */
void
setup()
{
  initUserSwitch();
  initDebugSerial();

  debugInfo("@Reset\n\r", 8);

  debugInfo("@Mario Time!\r\n", 14);
  playMarioTheme();

  debugInfo("@SD_I...", 8);
  if (!initLogging())
  {
    debugInfo("KO\r\n", 4);
  }
  else
  {
    debugInfo("OK\r\n", 4);
  }

  if (!clearAllPersistentDataOnRequest())
  {
    debugInfo("@Restart\r\n", 10);
  }

  initGpsSerial();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  unsigned long startOfLoopTimeMillis = millis();

  debugInfo("@CL >\r\n", 7);

  /* Loop start sequence */

  delay(1000);

  /* kiwi frame building */
  kiwiFrameBuilder.buildKiwiFrame(kiwiFrame);

  /* kiwi frame transmission */
  debugInfo("\r\n", 2);
  debugInfo((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  delay(50);
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  debugInfo((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  delay(50);
  debugInfo((char *) kiwiFrame, KIWI_FRAME_LENGTH);
  delay(50);
  debugInfo("\r\n", 2);

  /* positioning data reading (and debug) */
  nmeaGPS.readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer);

  /* NMEA sentences logging */
  sdFileLogger.logMessage(nmeaRmcSentenceBuffer, false);
  sdFileLogger.logMessage(nmeaGgaSentenceBuffer, false);
  delay(500);

  /* NMEA sentences transmission */
  fskModulator.modulateBytes(nmeaRmcSentenceBuffer, strlen(nmeaRmcSentenceBuffer));
  fskModulator.modulateBytes(nmeaGgaSentenceBuffer, strlen(nmeaGgaSentenceBuffer));

  /* custom frame building */
  customFrameBuilder.buildCustomFrame(customFrame);

  /* custom frame debug */
  SERIAL_DEBUG.print(customFrame);

  /* custom frame logging */
  sdFileLogger.logMessage(customFrame, false);

  /* pause half a second to ensure SD asynchronous writing to be finished */
  delay(500);

  /* custom frame transmission */
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  /* frame counter update */
  frameCounter.increment(1);

  debugInfo("@CL <\r\n", 7);
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
