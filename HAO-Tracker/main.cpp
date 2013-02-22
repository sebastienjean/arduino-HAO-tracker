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
#include <SoftwareSerial.h>
#include <FSK600BaudTA900TB1500Mod.h>
#include <pins.h>

// SD logfile path
#define LOGFILE "data.txt"

#define SENSOR_DATA_ASCII_STRING_LENGTH 40

#define KIWI_FRAME_LENGTH 11

// FSK modulator
FSK600BaudTA900TB1500Mod fskModulator(FSK_MODULATOR_TX);

// Software serial link used by GPS
#if defined(GPS_SERIAL_RX)
SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX, GPS_SERIAL_TX);
#else
#define serialNmeaGPSPort Serial1
#endif

// GPS
GPS serialNmeaGPS(&serialNmeaGPSPort, 2000,2000);

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
/**
 * Arduino's setup function, called once at startup, after init
 */
void
setup()
{
  // LEDs init
  pinMode(RED_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(ORANGE_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  delay(1000);
  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);

  pinMode(USER_BUTTON, INPUT);
  digitalWrite(USER_BUTTON, HIGH);

  // Serial debug at 600 baud
  Serial.begin(600);

  // SD card init
  Serial.print(F("SD Init..."));
  pinMode(SD_CARD_CHIP_SELECT, OUTPUT);
  if (!SD.begin(SD_CARD_CHIP_SELECT))
    {
      Serial.println(F("KO"));
      digitalWrite(RED_LED,HIGH);
    }
  else
    {
      Serial.println(F("OK"));
      digitalWrite(GREEN_LED,HIGH);
      delay(1000);
      if (digitalRead(USER_BUTTON) == 0)
        {
          // delete the file:
          Serial.println(F("SD clear..."));
          SD.remove(LOGFILE);
          for (int i=0;i<5;i++)
            {
              digitalWrite(ORANGE_LED, HIGH);
              delay(100);
              digitalWrite(ORANGE_LED, LOW);
              delay(100);
            }
        }
      Serial.println(F("RESET"));
      // opening logFile
      logFile = SD.open(LOGFILE, FILE_WRITE);
      if (logFile)
        {
          logFile.println(F("Logging Reset..."));
          logFile.close();
          Serial.println(F("OK"));
          for (int i=0;i<5;i++)
            {
              digitalWrite(GREEN_LED, HIGH);
              delay(100);
              digitalWrite(GREEN_LED, LOW);
              delay(100);
            }
        }
      // if the file isn't open, pop up an error:
      else
        {
          Serial.println(F("KO"));
          for (int i=0;i<5;i++)
            {
              digitalWrite(RED_LED, HIGH);
              delay(100);
              digitalWrite(RED_LED, LOW);
              delay(100);
            }
        }

    }

      // GPS on software serial at 4800 Baud
  serialNmeaGPSPort.begin(4800);

  for (int i = 0; i < 11; i++)
    kiwiFrame[i] = 0x00;
  kiwiFrame[0] = 0xFF;

  // wdt_enable(WDTO_8S);
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
  logFile = SD.open(LOGFILE, FILE_WRITE);
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
  logFile = SD.open(LOGFILE, FILE_WRITE);
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
    logFile = SD.open(LOGFILE, FILE_WRITE);
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

