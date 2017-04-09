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
#include <pins.h>
#include <main/defs.h>
#include <avr/wdt.h>

// Modules includes
#include <modules/framebuilder/CustomFrameBuilder.h>
#include <modules/themeplayer/MarioThemePlayer.h>

// Libs includes
#include <SD.h>
#include <core/Led.h>
#include <SD/SDFileLogger.h>
#include <FSK600BaudTA900TB1500Mod.h>
#include <core/BuiltInAnalogSensor.h>
#include <adc/mcp3428/MCP3428AnalogToDigitalConverter.h>
#include <sensor/SR04DistancePseudoAnalogSensor.h>


SDFileLogger sdFileLogger(&SD, LOG_FILE_PATH, LOG_POST_WRITE_DELAY_MILLIS);

/**
 * GPS3D object for GPS1, whose raw NMEA sentences will be forwarded on debug serial
 */
GPS3D nmeaGPS1(&serialNmeaGPS1Port, &SERIAL_DEBUG);

/**
 * GPS3D object for GPS2, whose raw NMEA sentences will be forwarded on debug serial
 */
GPS3D nmeaGPS2(&serialNmeaGPS2Port, &SERIAL_DEBUG);

/**
 * GPS3D object, whose raw NMEA sentences will be forwarded on debug serial
 */
GPS3D nmeaGPS3(&serialNmeaGPS3Port, &SERIAL_DEBUG);

/**
 * External buffer used for NMEA RMC sentence parsing and storage
 */
char nmeaRmcSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];

/**
 * External buffer used for NMEA GGA sentence parsing and storage
 */
char nmeaGgaSentenceBuffer[MAX_NMEA_SENTENCE_LENGTH];


/**
 * Array of gps
 */
GPS3D* gpsArray[SERIAL_NMEA_GPS_AMOUNT] =
  { &nmeaGPS1, &nmeaGPS2, &nmeaGPS3};

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
Counter* countersArray[NUMBER_OF_COUNTERS_IN_CUSTOM_FRAME] =
  { &frameCounter,
    &resetCounter };

/**
 * Counters to be included in custom frame
 */
Counters counters((Counter **) &countersArray, NUMBER_OF_COUNTERS_IN_CUSTOM_FRAME);

// ----------------------------------
// Analog sensors related definitions
// ----------------------------------

/**
 * Internal temperature analog sensor
 */
BuiltInAnalogSensor internalTemperatureAnalogSensor(INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * Differential pressure analog sensor
 */
BuiltInAnalogSensor differentialPressureAnalogSensor(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);

/**
 * Battery voltage analog sensor
 */
BuiltInAnalogSensor batteryHalfInputVoltageAnalogSensor(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);

/**
 * Battery temperature analog sensor
 */
BuiltInAnalogSensor batteryTemperatureAnalogSensor(BATTERY_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

/**
 * On board first MCP3428 I2C ADC
 */
MCP3428AnalogToDigitalConverter onBoardFirstMCP3428(MCP3428_0_ADDRESS_BIT0, MCP3428_0_ADDRESS_BIT1);

/**
 * On board second MCP3428 I2C ADC
 */
MCP3428AnalogToDigitalConverter onBoardSecondMCP3428(MCP3428_1_ADDRESS_BIT0, MCP3428_1_ADDRESS_BIT1);

/**
 * External temperature sensor
 */
AnalogSensor externalTemperatureAnalogSensor(&onBoardFirstMCP3428, 0);

/**
 * Middle temperature sensor
 */
AnalogSensor middleTemperatureAnalogSensor(&onBoardFirstMCP3428, 1);

/**
 * Humidity sensor
 */
AnalogSensor externalHumidityAnalogSensor(&onBoardFirstMCP3428, 2);

/**
 * Visible luminosity sensor
 */
AnalogSensor visibleLuminosityAnalogSensor(&onBoardSecondMCP3428, 0);

/**
 * IR luminosity sensor
 */
AnalogSensor irLuminosityAnalogSensor(&onBoardSecondMCP3428, 1);

/**
 * Distance sensor
 */
SR04DistancePseudoAnalogSensor distancePseudoAnalogSensor(SR04_TRIG_PIN, SR04_ECHO_PIN);

/**
 * Array of analog sensors to be included in custom frame
 */
AnalogSensor* sensorsArray[NUMBER_OF_ANALOG_SENSORS_IN_CUSTOM_FRAME] =
  { &internalTemperatureAnalogSensor,
    &externalTemperatureAnalogSensor,
    &middleTemperatureAnalogSensor,
    &externalHumidityAnalogSensor,
    &differentialPressureAnalogSensor,
    &visibleLuminosityAnalogSensor,
    &irLuminosityAnalogSensor,
    &batteryTemperatureAnalogSensor,
    &batteryHalfInputVoltageAnalogSensor,
    &distancePseudoAnalogSensor};

/**
 * Analog sensors to be included in custom frame
 */
AnalogSensors customFrameAnalogSensors((AnalogSensor **) &sensorsArray, NUMBER_OF_ANALOG_SENSORS_IN_CUSTOM_FRAME);

/**
 * Green LED (inverted-logic)
 */
Led greenLed(GREEN_LED_PIN, false);

/**
 * Red LED (inverted-logic)
 */
Led redLed(RED_LED_PIN, false);

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
CustomFrameBuilder customFrameBuilder(&counters, &customFrameAnalogSensors, &rtc, (GPS3D **) &gpsArray);

/**
 * Mario theme player (just because it is so cool to play it)
 */
MarioThemePlayer marioThemePlayer(FSK_MODULATOR_TX_PIN);

/**
 * Internal function used to send debug info both on debug serial and radio.
 * @param message a NUL-terminated string, supposed to include line-termination chars)
 */
void
debugInfo(char *message)
{
  SERIAL_DEBUG.print(message);
  fskModulator.modulateBytes(message, strlen(message));
}

/**
 * Internal function used to to initialize SD.
 */
/**
 * Internal function used to initialize serial debug.
 */
void
initDebugSerial()
{
  SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUDRATE);
  debugInfo("@Reset\r\n");
}

/**
 * Internal function used to initialize GPS serial port(s).
 */
void
initGpsSerial()
{
  serialNmeaGPS1Port.begin(SERIAL_NMEA_GPS1_BAUDRATE);
  serialNmeaGPS2Port.begin(SERIAL_NMEA_GPS2_BAUDRATE);
  serialNmeaGPS3Port.begin(SERIAL_NMEA_GPS3_BAUDRATE);
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
    debugInfo("@Clear\r\n");
    // reset all counters
    counters.reset();

    if (sdFileLogger.clear())
    {
      debugInfo("@SD Cleared\r\n");
    }
    return true;
  }
  return false;
}

/**
 * LEDs initialization
 */
void
initSD()
{
  pinMode(SD_CARD_CHIP_SELECT_PIN, OUTPUT);

  debugInfo("@SD_");

  if (!SD.begin(SD_CARD_CHIP_SELECT_PIN))
  {
    debugInfo("KO\r\n");
  }
  else
  {
    debugInfo("OK\r\n");
  }

  if (!clearAllPersistentDataOnRequest())
  {
    debugInfo("@Restart\r\n");
    resetCounter.increment(1);
  }
}

void
initLeds()
{
  redLed.off();
  greenLed.off();
  redLed.quicklyMakeBlinkSeveralTimes(3);
}

/**
 * Arduino's setup function, called once at startup, after init
 */
void
setup()
{
  wdt_disable();
  initDebugSerial();
  rtc.writeProtect(false);
  rtc.halt(false);
  initLeds();
  initUserSwitch();
  marioThemePlayer.playMarioTheme();
  initSD();
  initGpsSerial();
  delay(2000);
  wdt_enable(WDTO_8S);
}

/**
 *  GPS related sub-loop:
 *  Positioning data reading, logging (SD/debug) and transmission
 */
void
processGpsData()
{
  for (uint8_t index=0; index<SERIAL_NMEA_GPS_AMOUNT; index++)
  {
    /* positioning data reading (and debug) */
    if (gpsArray[index]->readPositioningData(nmeaRmcSentenceBuffer, nmeaGgaSentenceBuffer) == GPS_OK)
    {
      sdFileLogger.logMessage(nmeaRmcSentenceBuffer, false);
      sdFileLogger.logMessage(nmeaGgaSentenceBuffer, false);

      wdt_reset();

      /* NMEA sentences transmission */
      fskModulator.modulateBytes(nmeaRmcSentenceBuffer, strlen(nmeaRmcSentenceBuffer));
      fskModulator.modulateBytes(nmeaGgaSentenceBuffer, strlen(nmeaGgaSentenceBuffer));

      wdt_reset();
    }
    else
    {
      SERIAL_DEBUG.print("$GP_KO\r\n");
      fskModulator.modulateBytes("$GP_KO\r\n", 8);

      wdt_reset();
    }
  }
}

/**
 *  Custom frame building, logging (SD/debug) and transmission
 */
void
processCustomFrame()
{
  /* custom frame building */
  customFrameBuilder.buildCustomFrame(customFrame);

  wdt_reset();

  /* custom frame debug */
  SERIAL_DEBUG.print(customFrame);

  wdt_reset();

  /* custom frame logging */
  sdFileLogger.logMessage(customFrame, false);

  wdt_reset();

  /* custom frame transmission */
  fskModulator.modulateBytes(customFrame, strlen(customFrame));

  wdt_reset();
}

/**
 * End of loop related code:
 *      frame counter update
 */
void
processEndOfLoop()
{
  frameCounter.increment(1);
  wdt_reset();
}

/**
 * Start of loop related code:
 *      LED signaling
 */
void
processStartOfLoop()
{
  greenLed.quicklyMakeBlinkSeveralTimes(3);
  wdt_reset();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  processStartOfLoop();
  processGpsData();
  processCustomFrame();
  processEndOfLoop();
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
