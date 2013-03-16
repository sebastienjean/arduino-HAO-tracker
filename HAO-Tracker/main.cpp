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
#include <Counter.h>
#include <Counters.h>
#include <KiwiFrameBuilder.h>
#include <CustomFrameBuilder.h>
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

// Counters
Counter frameCounter(FRAME_COUNTER_BASE_ADDRESS);
Counter resetCounter(RESET_COUNTER_BASE_ADDRESS);
Counter* countersArray[2] = { &frameCounter, &resetCounter};
Counters counters(countersArray, 2);

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

// Voltage monitor
// N.B. this analog sensor is handled seperately from the others since kiwi frame does
AnalogSensor voltage(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);

// RTC
DS1302_RTC rtc(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);

// customFrameBuilder
char customFrame[CUSTOM_FRAME_MAX_LENGTH];
CustomFrameBuilder customFrameBuilder(&counters, &sensors, &voltage, &rtc, &nmeaGPS);

// Kiwi Frame
unsigned char kiwiFrame[KIWI_FRAME_LENGTH];
KiwiFrameBuilder kiwiFrameBuilder(&sensors, &voltage);

// LEDS
// Todo rename variables (lower case)
Led red_LED(RED_LED_PIN);
Led orange_LED(ORANGE_LED_PIN);
Led green_LED(GREEN_LED_PIN);
Led blue_LED(BLUE_LED_PIN);

Led* ledArray[4] = {&red_LED, &orange_LED, &green_LED, &blue_LED};
Leds leds(ledArray, 4);

/**
 * Initializes logging (SD).
 * @return logging initialization success status
 */
boolean initLogging()
{
  return LOGGER.begin(LOG_FILE_PATH, SD_CHIP_SELECT_PIN);
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
  leds.on();
  delay(1000);
  leds.off();

  initUserButton();

  initSerialDebug();

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

  initGPS();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void loop()
{
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

  // NMEA sentences transmission
  fskModulator.modulateBytes(nmeaRmcSentenceBuffer, strlen(nmeaRmcSentenceBuffer));
  fskModulator.modulateBytes(nmeaGgaSentenceBuffer, strlen(nmeaRmcSentenceBuffer));

  green_LED.quicklyMakeBlinkSeveralTimes(3);
  // custom frame building
  customFrameBuilder.buildCustomFrame(customFrame);

  // custom frame debug
  SERIAL_DEBUG.print(customFrame);

  // custom frame logging
  LOGGER.logMessage(customFrame, false);

  // custom frame transmission
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  red_LED.quicklyMakeBlinkSeveralTimes(1);
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

