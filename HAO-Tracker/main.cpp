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
#include <pins.h>
#include <defs.h>

// libs
#if defined(GPS_SERIAL_RX_PIN)
#include <SoftwareSerial.h>
#endif
#include <SD.h>
#include <FSK600BaudTA900TB1500Mod.h>
#include <GPS.h>
#include <GPS3D.h>

// modules
#include <AnalogSensor.h>
#include <AnalogSensors.h>
#include <Led.h>
#include <Leds.h>
#include <kiwiFrameBuilder_module.h>
#include <customFrameBuilder_module.h>
#include <Logger.h>
#include <DS1302_RTC.h>


// Software serial link used by GPS
#if defined(GPS_SERIAL_RX_PIN)
SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX_PIN, GPS_SERIAL_TX_PIN);
#else
#define serialNmeaGPSPort Serial1
#endif

// GPS
GPS3D nmeaGPS(&serialNmeaGPSPort, &SERIAL_DEBUG);
char nmeaRmcSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];
char nmeaGgaSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];

// FSK modulator
FSK600BaudTA900TB1500Mod fskModulator(FSK_MODULATOR_TX_PIN);

// customFrameBuilder
char customFrame[CUSTOM_FRAME_MAX_LENGTH];

// RTC
DS1302_RTC RTC(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);

// LEDS
// Todo rename variables (lower case)
Led Red_LED(RED_LED_PIN);
Led Orange_LED(ORANGE_LED_PIN);
Led Green_LED(GREEN_LED_PIN);
Led Blue_LED(BLUE_LED_PIN);

Led* ledArray[4] = {&Red_LED, &Orange_LED, &Green_LED, &Blue_LED};
Leds All_Leds(ledArray, 4);

// Analog Sensors
AnalogSensor differentialPressureAnalogSensor(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor absolutePressureAnalogSensor(ABSOLUTE_PRESSURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor externalTemperatureAnalogSensor(EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor internalTemperatureAnalogSensor(INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);
AnalogSensor* sensorsArray[4] = { &differentialPressureAnalogSensor,
                                  &absolutePressureAnalogSensor,
                                  &externalTemperatureAnalogSensor,
                                  &internalTemperatureAnalogSensor};
AnalogSensors sensors(sensorsArray, 4);

/**
 * Initializes logging (SD).
 * @return logging initialization success status
 */
boolean initLogging()
{
  LOGGER.begin(LOG_FILE_PATH, SD_CHIP_SELECT_PIN);
}

/**
 * Waits one second for user to decide if log file has to be deleted, deletes it if needed.
 * @return log file has been deletion status
 */
boolean deleteLogFileIfUserClaimsTo()
{
  delay(1000);

  if (digitalRead(USER_BUTTON_PIN) == LOW)
  {
      return LOGGER.reset();
  }
  return false;
}

/**
 * Initializes User switch.
 */
void initUserButton()
{
  pinMode(USER_BUTTON_PIN, INPUT);
  digitalWrite(USER_BUTTON_PIN, HIGH);
}

/**
 * Initializes serial debug communication.
 */
void initSerialDebug()
{
  SERIAL_DEBUG.begin(600);
}

/**
 * Initializes GPS.
 */
void initGPS()
{
  // GPS on software serial at 4800 Baud
  serialNmeaGPSPort.begin(SERIAL_NMEA_GPS_BAUDRATE);
}

/**
 * Arduino's setup function, called once at startup, after init
 */
void setup()
{
  All_Leds.on();
  delay(1000);
  All_Leds.off();

  initUserButton();

  initSerialDebug();

  SERIAL_DEBUG.println(F("R"));

  SERIAL_DEBUG.print(F("SD Init..."));

  if (!initLogging())
  {
    SERIAL_DEBUG.println(F("KO"));
    Orange_LED.showStatus(false);
  }
  else
  {
    SERIAL_DEBUG.println(F("OK"));
    Orange_LED.showStatus(true);

    if (deleteLogFileIfUserClaimsTo())
      SERIAL_DEBUG.println(F("SD Clear"));
  }

  Orange_LED.showStatus(LOGGER.logMessage("R", true));

  initGPS();

  All_Leds.off();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void loop()
{
  // show loop start sequence
  Red_LED.quicklyMakeBlinkSeveralTimes(1);
  Green_LED.quicklyMakeBlinkSeveralTimes(1);

  // kiwi frame building
  buildKiwiFrame();

  // kiwi frame transmission
  fskModulator.modulateBytes((char *) kiwiFrame, KIWI_FRAME_LENGTH);

  Green_LED.quicklyMakeBlinkSeveralTimes(2);
  // Positioning data reading (and debug)
  nmeaGPS.readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer);

  Blue_LED.showStatus(nmeaGPS.getFix());

  // NMEA sentences logging
  LOGGER.logMessage(nmeaRmcSentenceBuffer, false);
  LOGGER.logMessage(nmeaGgaSentenceBuffer, false);

  // NMEA sentences transmission
  fskModulator.modulateBytes(nmeaRmcSentenceBuffer, strlen(nmeaRmcSentenceBuffer));
  fskModulator.modulateBytes(nmeaGgaSentenceBuffer, strlen(nmeaRmcSentenceBuffer));

  Green_LED.quicklyMakeBlinkSeveralTimes(3);
  // custom frame building
  buildCustomFrame(customFrame);

  // custom frame debug
  SERIAL_DEBUG.print(customFrame);

  // custom frame logging
  LOGGER.logMessage(customFrame, false);

  // custom frame transmission
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  Red_LED.quicklyMakeBlinkSeveralTimes(1);
  delay(1000);
}

/**
 * Application's main (what else to say?)
 * @return (never)
 */
int main(void)
{
  init();

  setup();

  for (;;)
    loop();

  return 0;
}

