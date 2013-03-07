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
#include <SD.h>
#include <GPS.h>
#include <pins.h>
#include <defs.h>
#if defined(GPS_SERIAL_RX)
#include <SoftwareSerial.h>
#endif
#include <FSK600BaudTA900TB1500Mod.h>

#include <sensors_module.h>
#include <leds_module.h>
#include <kiwiFrameBuilder_module.h>
#include <logging_module.h>

// FSK modulator
FSK600BaudTA900TB1500Mod fskModulator(FSK_MODULATOR_TX);

// Software serial link used by GPS
#if defined(GPS_SERIAL_RX)
SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX, GPS_SERIAL_TX);
#else
#define serialNmeaGPSPort Serial1
#endif

// GPS
GPS serialNmeaGPS(&serialNmeaGPSPort, SERIAL_NMEA_GPS_READING_MILLIS_TIMEOUT,SERIAL_NMEA_GPS_READING_CHARS_TIMEOUT);

// sensor data, as ASCII
char sensorString[SENSOR_DATA_ASCII_STRING_LENGTH];

// sensor string size
int sensorStringSize;

// buffer for NMEA sentence reading
char nmeaSentence[MAX_NMEA_SENTENCE_LENGTH];

/**
 * Initializes User switch.
 */
void initUserButton()
{
  pinMode(USER_BUTTON, INPUT);
  digitalWrite(USER_BUTTON, HIGH);
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
  initSensors();

  initLEDs();

  showLEDsStartupSequence();

  initUserButton();

  initSerialDebug();

  SERIAL_DEBUG.println(F("R"));

  SERIAL_DEBUG.print(F("SD Init..."));

  if (!initLogging())
  {
    SERIAL_DEBUG.println(F("KO"));
    showStatus(0);
  }
  else
  {
    SERIAL_DEBUG.println(F("OK"));
    showStatus(1);

    if (deleteLogFileIfUserClaimsTo())
      SERIAL_DEBUG.println(F("SD Clear"));
  }

  logMessage("R", true);

  initGPS();
}

void appendTimeToSensorString()
{
  // seconds elapsed since last reset, as a decimal coded ASCII string
  itoa(millis() / 1000, sensorString+sensorStringSize, 10);
  sensorStringSize = strlen(sensorString);
}

void appendFieldSeparatorToSensorString()
{
  sensorString[sensorStringSize++] = SENSOR_STRING_FIELD_SEPARATOR;
}

void appendAnalogValueToSensorString(int value)
{
  itoa(value, sensorString + sensorStringSize, 10);
  sensorStringSize = strlen(sensorString);
}

void terminateSensorString()
{
  sensorString[sensorStringSize++]='\r';
  sensorString[sensorStringSize++]='\n';
  sensorString[sensorStringSize] = '\0';
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void loop()
{
  sensorStringSize = 0;

  GPS_status_enum gpsReadingStatus;

  // local time processing
  appendTimeToSensorString();
  appendFieldSeparatorToSensorString();

  // sensors reading
  readSensors();

  // kiwi frame building
  buildKiwiFrame();

  // absolute pressure processing
  appendAnalogValueToSensorString(absolutePressureSensorValue);
  appendFieldSeparatorToSensorString();

  // differential pressure processing
  appendAnalogValueToSensorString(differentialPressureSensorValue);
  appendFieldSeparatorToSensorString();

  // internal temperature pressure processing
  appendAnalogValueToSensorString(internalTemperatureSensorValue);
  appendFieldSeparatorToSensorString();

  // external temperature pressure processing
  appendAnalogValueToSensorString(externalTemperatureSensorValue);
  appendFieldSeparatorToSensorString();

  // battery voltage processing
  appendAnalogValueToSensorString(batteryVoltageSensorValue);

  // end of frame processing
  terminateSensorString();

  // Kiwi Frame transmission
  fskModulator.modulateBytes(kiwiFrame, KIWI_FRAME_LENGTH);

  // sensor string logging
  logMessage(sensorString, false);

  // sensor string debug
  SERIAL_DEBUG.print(sensorString);

  // sensor string transmission (with line termination)

  fskModulator.modulateBytes((unsigned char *) sensorString, sensorStringSize);

  // NMEA RMC sentence reading
  gpsReadingStatus = serialNmeaGPS.readRMC(nmeaSentence);
  switch (gpsReadingStatus)
  {
    case GPS_OK:
      for (int cpt = 0; cpt < MAX_NMEA_SENTENCE_LENGTH; cpt++)
      {
        if ((cpt > 5) && (cpt <= 25) && (nmeaSentence[cpt] == 'A'))
          {
            digitalWrite(ORANGE_LED, HIGH);
            delay(100);
            digitalWrite(ORANGE_LED, LOW);
            delay(100);
            digitalWrite(ORANGE_LED, HIGH);
            delay(100);
            digitalWrite(ORANGE_LED, LOW);
          }
      }
      break;
    case GPS_TIMEOUT:
      strcpy(nmeaSentence, "GPS TO\r\n");
      break;
    default:
      strcpy(nmeaSentence, "GPS E\r\n");
      break;
  }

  // NMEA RMC sentence debug
  SERIAL_DEBUG.print(nmeaSentence);

  // NMEA RMC sentence transmission
  fskModulator.modulateBytes((unsigned char *) nmeaSentence, strlen(nmeaSentence));

  // NMEA RMC sentence logging
  logMessage(nmeaSentence, false);

  // NMEA GGA sentence reading
  gpsReadingStatus = serialNmeaGPS.readGGA(nmeaSentence);
  switch (gpsReadingStatus)
  {
    case GPS_OK:
      break;
    case GPS_TIMEOUT:
      strcpy(nmeaSentence, "GPS TO\r\n");
      break;
    default:
      strcpy(nmeaSentence, "GPS E\r\n");
      break;
  }

  // NMEA GGA sentence debug
  SERIAL_DEBUG.print(nmeaSentence);

  // NMEA GGA sentence transmission
  fskModulator.modulateBytes((unsigned char *) nmeaSentence, strlen(nmeaSentence));

  // NMEA GGA sentence logging
  logMessage(nmeaSentence, false);
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

