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

unsigned char kiwiFrame[KIWI_FRAME_LENGTH];

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
void
setup()
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

/**
 * Arduino's loop function, called in loop (incredible, isn't it ?)
 */
void
loop()
{
  int sensorStringOffset = 0;
  unsigned char chk = 0;
  GPS_status_enum gpsStatus;

  // millis since last reset processing
  itoa(millis() / 1000, sensorString, 10);
  sensorStringOffset = strlen(sensorString);
  sensorString[sensorStringOffset++] = ',';
  // absolute pressure processing
  absolutePressureSensorValue = analogRead(ABSOLUTE_PRESSURE_ANALOG_SENSOR);
  itoa(absolutePressureSensorValue, sensorString + sensorStringOffset, 10);
  sensorStringOffset = strlen(sensorString);
  sensorString[sensorStringOffset++] = ',';
  kiwiFrame[1] = (unsigned char) (absolutePressureSensorValue / 4);
  if (kiwiFrame[1] == 0xFF)
    kiwiFrame[1] = 0xFE;

  // differential pressure processing
  differentialPressureSensorValue = analogRead(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR);
  itoa(differentialPressureSensorValue, sensorString + sensorStringOffset, 10);
  sensorStringOffset = strlen(sensorString);
  sensorString[sensorStringOffset++] = ',';
  kiwiFrame[2] = (unsigned char) (differentialPressureSensorValue / 4);
  if (kiwiFrame[2] == 0xFF)
    kiwiFrame[2] = 0xFE;

  // internal temperature pressure processing
  internalTemperatureSensorValue = analogRead(INTERNAL_TEMPERATURE_ANALOG_SENSOR);
  itoa(internalTemperatureSensorValue, sensorString + sensorStringOffset, 10);
  sensorStringOffset = strlen(sensorString);
  sensorString[sensorStringOffset++] = ',';
  kiwiFrame[3] = (unsigned char) (internalTemperatureSensorValue / 4);
  if (kiwiFrame[3] == 0xFF)
    kiwiFrame[3] = 0xFE;

  // external temperature pressure processing
  externalTemperatureSensorValue = analogRead(EXTERNAL_TEMPERATURE_ANALOG_SENSOR);
  itoa(externalTemperatureSensorValue, sensorString + sensorStringOffset, 10);
  sensorStringOffset = strlen(sensorString);
  sensorString[sensorStringOffset++] = ',';
  kiwiFrame[4] = (unsigned char) (externalTemperatureSensorValue / 4);
  if (kiwiFrame[4] == 0xFF)
    kiwiFrame[4] = 0xFE;

  // battery voltage processing
  batteryVoltageSensorValue = analogRead(BATTERY_VOLTAGE_ANALOG_SENSOR);
  itoa(batteryVoltageSensorValue, sensorString + sensorStringOffset, 10);
  sensorStringOffset = strlen(sensorString);
  sensorString[sensorStringOffset++] = '\r';
  sensorString[sensorStringOffset++] = '\n';
  kiwiFrame[5] = (unsigned char) (batteryVoltageSensorValue / 4);
  if (kiwiFrame[5] == 0xFF)
    kiwiFrame[5] = 0xFE;
  kiwiFrame[9] = (unsigned char) (batteryVoltageSensorValue / 8);

  for (int cpt = 1; cpt < KIWI_FRAME_LENGTH - 1; cpt++)
    chk = (unsigned char) ((chk + kiwiFrame[cpt]) % 256);

  chk = (unsigned char) (chk / 2);
  kiwiFrame[KIWI_FRAME_LENGTH] = chk;

  // Kiwi Frame transmission
  for (int cpt = 0; cpt < KIWI_FRAME_LENGTH; cpt++)
    fskModulator.write(kiwiFrame[cpt]);
  fskModulator.off();

  // Logging
  logFile = SD.open(LOG_FILE_PATH, FILE_WRITE);
  if (logFile)
    {
      Serial.println(F("log file access success"));
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
      digitalWrite(GREEN_LED, LOW);
      delay(100);
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
      digitalWrite(GREEN_LED, LOW);
      logFile.print(sensorString);
      logFile.close();
    }
  else
    {
      Serial.println(F("log file access failure"));
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
      delay(100);
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
    }
      //wdt_reset();

      // Sensor data processing

      // Debug
  Serial.print(sensorString);

  // Transmission
  for (int cpt = 0; cpt < strlen(sensorString); cpt++)
    fskModulator.write(sensorString[cpt]);
  fskModulator.off();

  // NMEA RMC
  gpsStatus = serialNmeaGPS.readRMC(nmeaSentence);
  switch (gpsStatus)
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
  //wdt_reset();

  // Debug
  Serial.print(nmeaSentence);

  // Transmission
  for (int cpt = 0; cpt < strlen(nmeaSentence); cpt++)
    fskModulator.write(nmeaSentence[cpt]);
  fskModulator.off();

  // Logging
  logFile = SD.open(LOG_FILE_PATH, FILE_WRITE);
  if (logFile)
    {
      Serial.println(F("log file access success"));
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
      digitalWrite(GREEN_LED, LOW);
      delay(100);
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
      digitalWrite(GREEN_LED, LOW);
      logFile.print(nmeaSentence);
      logFile.close();
    }
  else
    {
      Serial.println(F("log file access failure"));
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
      delay(100);
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
    }
  // NMEA RMC
  gpsStatus = serialNmeaGPS.readGGA(nmeaSentence);
  switch (gpsStatus)
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

  // Debug
  Serial.print(nmeaSentence);

  // Transmission
  for (int cpt = 0; cpt < strlen(nmeaSentence); cpt++)
    fskModulator.write(nmeaSentence[cpt]);
  fskModulator.off();

  // Logging
    logFile = SD.open(LOG_FILE_PATH, FILE_WRITE);
    if (logFile)
      {
        Serial.println(F("log file access success"));
        digitalWrite(GREEN_LED, HIGH);
        delay(100);
        digitalWrite(GREEN_LED, LOW);
        delay(100);
        digitalWrite(GREEN_LED, HIGH);
        delay(100);
        digitalWrite(GREEN_LED, LOW);
        logFile.print(nmeaSentence);
        logFile.close();
      }
    else
      {
        Serial.println(F("log file access failure"));
        digitalWrite(RED_LED, HIGH);
        delay(100);
        digitalWrite(RED_LED, LOW);
        delay(100);
        digitalWrite(RED_LED, HIGH);
        delay(100);
        digitalWrite(RED_LED, LOW);
      }
  //wdt_reset();
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

