#include <Arduino.h>
#include <gps_module.h>
#include <GPS.h>
#include <leds_module.h>
#include <pins.h>
#include <defs.h>
#if defined(GPS_SERIAL_RX)
#include <SoftwareSerial.h>
#endif

// Software serial link used by GPS
#if defined(GPS_SERIAL_RX)
SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX, GPS_SERIAL_TX);
#else
#define serialNmeaGPSPort Serial1
#endif

// GPS
GPS serialNmeaGPS(&serialNmeaGPSPort, SERIAL_NMEA_GPS_READING_MILLIS_TIMEOUT,SERIAL_NMEA_GPS_READING_CHARS_TIMEOUT);

// buffer for NMEA-RMC sentence reading
char nmeaRmcSentence[MAX_NMEA_SENTENCE_LENGTH];

// buffer for NMEA-GGA sentence reading
char nmeaGgaSentence[MAX_NMEA_SENTENCE_LENGTH];

void initGPS()
{
  // GPS on software serial at 4800 Baud
  serialNmeaGPSPort.begin(SERIAL_NMEA_GPS_BAUDRATE);
}

void readNmeaRmcSentence()
{
  GPS_status_enum gpsReadingStatus;

  // NMEA RMC sentence reading
  gpsReadingStatus = serialNmeaGPS.readRMC(nmeaRmcSentence);
  switch (gpsReadingStatus)
  {
    case GPS_OK:
      for (int cpt = 0; cpt < MAX_NMEA_SENTENCE_LENGTH; cpt++)
      {
        if ((cpt > 5) && (cpt <= 25) && (nmeaRmcSentence[cpt] == 'A'))
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
      strcpy(nmeaRmcSentence, "GPS TO\r\n");
      break;
    default:
      strcpy(nmeaRmcSentence, "GPS E\r\n");
      break;
  }
}

void readNmeaGgaSentence()
{
  GPS_status_enum gpsReadingStatus;

  // NMEA GGA sentence reading
  gpsReadingStatus = serialNmeaGPS.readGGA(nmeaGgaSentence);
  switch (gpsReadingStatus)
  {
    case GPS_OK:
      break;
    case GPS_TIMEOUT:
      strcpy(nmeaGgaSentence, "GPS TO\r\n");
      break;
    default:
      strcpy(nmeaGgaSentence, "GPS E\r\n");
      break;
  }
}
