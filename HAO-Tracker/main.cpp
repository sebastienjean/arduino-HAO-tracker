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
#include <GPS3D.h>
/*#include <gps_module.h>*/

// Software serial link used by GPS
#if defined(GPS_SERIAL_RX)
SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX, GPS_SERIAL_TX);
#else
#define serialNmeaGPSPort Serial1
#endif

// GPS
GPS3D nmeaGPS(&serialNmeaGPSPort, &SERIAL_DEBUG);


// FSK modulator
FSK600BaudTA900TB1500Mod fskModulator(FSK_MODULATOR_TX);


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

  // sensors reading
  readSensors();

  // kiwi frame building
  buildKiwiFrame();

  // kiwi Frame transmission
  fskModulator.modulateBytes(kiwiFrame, KIWI_FRAME_LENGTH);

  // Positioning data reading
  nmeaGPS.readPositioningData();

  // custom frame building
  buildCustomFrame();

  // custom frame logging
  logMessage(customFrame, false);

  // custom frame debug
  SERIAL_DEBUG.print(customFrame);

  // custom frame transmission
  fskModulator.modulateBytes((unsigned char *) customFrame, customFrameLength);

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

