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
#include <customFrameBuilder_module.h>
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

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void loop()
{
  GPS_status_enum gpsReadingStatus;

  // sensors reading
  readSensors();

  // kiwi frame building
  buildKiwiFrame();

  // kiwi Frame transmission
  fskModulator.modulateBytes(kiwiFrame, KIWI_FRAME_LENGTH);

  // custom frame building
  buildCustomFrame();

  // custom frame logging
  logMessage(customFrame, false);

  // custom frame debug
  SERIAL_DEBUG.print(customFrame);

  // custom frame transmission
  fskModulator.modulateBytes((unsigned char *) customFrame, customFrameLength);

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

