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

// log File
File logFile;

// absolute pressure sensor value
int absolutePressureSensorValue = 0;

// differential pressure sensor value
int differentialPressureSensorValue = 0;

// internal temperature sensor value
int internalTemperatureSensorValue = 0;

// external temperature sensor value
int externalTemperatureSensorValue = 0;

// battery voltage sensor value
int batteryVoltageSensorValue = 0;

// buffer for NMEA sentence reading
char nmeaSentence[MAX_NMEA_SENTENCE_LENGTH];

// KIWI frame
unsigned char kiwiFrame[KIWI_FRAME_LENGTH];

// KIWI frame checksum
unsigned char kiwiFrameChecksum;

// Offset of the next value to be inserted in KIWI frame, while building it
int kiwiFrameNextOffset;

/**
 * Initializes LEDs wirings.
 */
void initLEDs()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
}

/**
 * Initializes SD shield.
 * @return SD initialization success status (boolean)
 */
int initSdShield()
{
  pinMode(SD_CARD_CHIP_SELECT, OUTPUT);
  return SD.begin(SD_CARD_CHIP_SELECT);
}

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
 * Resets KIWI frame contents.
 */
void resetKiwiFrame()
{
  for (int i = 0; i < 11; i++)
          kiwiFrame[i] = 0x00;
  kiwiFrame[0] = 0xFF;
}

/**
 * Plays LEDs startup sequence.
 */
void showLEDsStartupSequence()
{
  digitalWrite(RED_LED, HIGH);
  digitalWrite(ORANGE_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  delay(1000);
  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}

/**
 * Displays status (OK/KO) using red/green LEDs.
 */
void showStatus(int status)
{
  if (status)
  {
    digitalWrite(GREEN_LED,HIGH);
    digitalWrite(RED_LED,LOW);
  }
  else
  {
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(RED_LED,HIGH);
  }
}

/**
 * Blinks a given LED at 5Hz a given number of times.
 * @param led the LED to blink
 * @param times the number of times the LED should blink
 */
void quicklyMakeSomeLedBlinkSeveralTimes(int led, int times)
{
  for (int i=0;i<times;i++)
  {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }
}

/**
 * Logs a message on the SD card.
 * @param message the string to be logged
 * (line termination characters are appended)
 * @param times the number of times the LED should blink
 */
int logMessageOnSdCard(char *message)
{
  logFile = SD.open(LOG_FILE_PATH, FILE_WRITE);
  if (logFile)
  {
    logFile.println(message);
    logFile.close();
    quicklyMakeSomeLedBlinkSeveralTimes(ORANGE_LED, 2);
  }
  else
    quicklyMakeSomeLedBlinkSeveralTimes(ORANGE_LED, 5);

  return logFile;
}

/**
 * Waits for user to decide if log file has to be deleted, deletes if if needed
 * @return <tt>true</tt> if log file has been claimed to be deleted (and has been deleted), <tt>false</tt> if not
 */
int deleteLogFileIfUserClaimsTo()
{
  // User has one second to decide
  delay(1000);

  if (digitalRead(USER_BUTTON) == LOW)
  {
      SD.remove(LOG_FILE_PATH);
      quicklyMakeSomeLedBlinkSeveralTimes(RED_LED, 10);
      return true;
  }
  return false;
}

/**
 * Arduino's setup function, called once at startup, after init
 */
void setup()
{
  initLEDs();

  showLEDsStartupSequence();

  initUserButton();

  initSerialDebug();

  SERIAL_DEBUG.println(F("R"));

  SERIAL_DEBUG.print(F("SD Init..."));

  if (!initSdShield())
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

  logMessageOnSdCard("R");

  initGPS();

  resetKiwiFrame();
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
  sensorString[sensorStringSize] = '\0';
}

void appendAnalogValueToKiwiFrame(int value)
{
  kiwiFrame[kiwiFrameNextOffset] = (unsigned char) (value / 4);
  if (kiwiFrame[kiwiFrameNextOffset] == 0xFF)
    kiwiFrame[kiwiFrameNextOffset] = 0xFE;
  kiwiFrameNextOffset++;
}

void computeKiwiFrameChecksum()
{
  for (int cpt = 1; cpt < KIWI_FRAME_LENGTH - 1; cpt++)
      kiwiFrameChecksum = (unsigned char) ((kiwiFrameChecksum + kiwiFrame[cpt]) % 256);

  kiwiFrameChecksum = (unsigned char) (kiwiFrameChecksum / 2);
  kiwiFrame[KIWI_FRAME_LENGTH] = kiwiFrameChecksum;
}

void modulateBytes(unsigned char *bytes, int length)
{
  for (int cpt = 0; cpt < length; cpt++)
    fskModulator.write(bytes[cpt]);
  fskModulator.off();
}

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void loop()
{
  sensorStringSize = 0;
  kiwiFrameChecksum = 0;
  kiwiFrameNextOffset = 1; // OxFF header at offset 0 is inserted in setup
  GPS_status_enum gpsReadingStatus;

  // local time processing
  appendTimeToSensorString();
  appendFieldSeparatorToSensorString();

  // absolute pressure processing
  absolutePressureSensorValue = analogRead(ABSOLUTE_PRESSURE_ANALOG_SENSOR);
  appendAnalogValueToSensorString(absolutePressureSensorValue);
  appendFieldSeparatorToSensorString();
  appendAnalogValueToKiwiFrame(absolutePressureSensorValue);

  // differential pressure processing
  differentialPressureSensorValue = analogRead(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR);
  appendAnalogValueToSensorString(differentialPressureSensorValue);
  appendFieldSeparatorToSensorString();
  appendAnalogValueToKiwiFrame(differentialPressureSensorValue);

  // internal temperature pressure processing
  internalTemperatureSensorValue = analogRead(INTERNAL_TEMPERATURE_ANALOG_SENSOR);
  appendAnalogValueToSensorString(internalTemperatureSensorValue);
  appendFieldSeparatorToSensorString();
  appendAnalogValueToKiwiFrame(internalTemperatureSensorValue);

  // external temperature pressure processing
  externalTemperatureSensorValue = analogRead(EXTERNAL_TEMPERATURE_ANALOG_SENSOR);
  appendAnalogValueToSensorString(externalTemperatureSensorValue);
  appendFieldSeparatorToSensorString();
  appendAnalogValueToKiwiFrame(externalTemperatureSensorValue);

  // battery voltage processing
  batteryVoltageSensorValue = analogRead(BATTERY_VOLTAGE_ANALOG_SENSOR);
  appendAnalogValueToSensorString(batteryVoltageSensorValue);
  appendAnalogValueToKiwiFrame(batteryVoltageSensorValue);

  // end of frame processing
  terminateSensorString();

  appendAnalogValueToKiwiFrame(0);
  appendAnalogValueToKiwiFrame(0);
  appendAnalogValueToKiwiFrame(0);
  appendAnalogValueToKiwiFrame(batteryVoltageSensorValue / 2);
  computeKiwiFrameChecksum();

  // Kiwi Frame transmission
  modulateBytes(kiwiFrame, KIWI_FRAME_LENGTH);

  // sensor string logging
  logMessageOnSdCard(sensorString);

  // sensor string debug
  SERIAL_DEBUG.println(sensorString);

  // sensor string transmission (with line termination)
  sensorString[sensorStringSize++]='\r';
  sensorString[sensorStringSize++]='\n';
  modulateBytes((unsigned char *) sensorString, sensorStringSize);

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
  modulateBytes((unsigned char *) nmeaSentence, strlen(nmeaSentence));

  // NMEA RMC sentence logging
  // TODO fix line termination dup bug
  logMessageOnSdCard(nmeaSentence);

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
  modulateBytes((unsigned char *) nmeaSentence, strlen(nmeaSentence));

  // NMEA GGA sentence logging
  // TODO fix line termination dup bug
  logMessageOnSdCard(nmeaSentence);
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

