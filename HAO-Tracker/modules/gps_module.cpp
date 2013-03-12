//#include <Arduino.h>
//#include <gps_module.h>
//#include <GPS.h>
//#include <leds_module.h>
//#include <customFrameBuilder_module.h>
//#include <pins.h>
//#include <defs.h>
//#if defined(GPS_SERIAL_RX)
//#include <SoftwareSerial.h>
//#endif
//
//// Software serial link used by GPS
//#if defined(GPS_SERIAL_RX)
//SoftwareSerial serialNmeaGPSPort(GPS_SERIAL_RX, GPS_SERIAL_TX);
//#else
//#define serialNmeaGPSPort Serial1
//#endif
//
//// GPS
//GPS nmeaGPS(&serialNmeaGPSPort, SERIAL_NMEA_GPS_READING_MILLIS_TIMEOUT,SERIAL_NMEA_GPS_READING_CHARS_TIMEOUT);
//
//// buffer for NMEA-RMC sentence reading
//char nmeaRmcSentence[MAX_NMEA_SENTENCE_LENGTH];
//
//// buffer for NMEA-GGA sentence reading
//char nmeaGgaSentence[MAX_NMEA_SENTENCE_LENGTH];
//
//int i;
//
//// Begin of values needed for custom sentence
//int timeBegin=7;
//int latitudeBegin=17;
//int longitudeBegin=27;
//int nbSatBegin=40;
//int altitudeBegin=47;
//int dataStatusBegin=7;
//int speedOverGroundBegin=30;
//
//// String of values
//char gpsTime[32];
//char gpsLatitude[32];
//char gpsLongitude[32];
//char gpsNbSat[32];
//char gpsAltitude[32];
//char gpsDataStatus[32];
//char gpsSpeedOverGround[32];
//
//void initGPS()
//{
//  // GPS on software serial at 4800 Baud
//  serialNmeaGPSPort.begin(SERIAL_NMEA_GPS_BAUDRATE);
//}
//
//void readNmeaRmcSentence()
//{
//  GPS_status_enum gpsReadingStatus;
//
//  // NMEA RMC sentence reading
//  gpsReadingStatus = nmeaGPS.readRMC(nmeaRmcSentence);
//  switch (gpsReadingStatus)
//  {
//    case GPS_OK:
//      for (int cpt = 0; cpt < MAX_NMEA_SENTENCE_LENGTH; cpt++)
//      {
//        if ((cpt > 5) && (cpt <= 25) && (nmeaRmcSentence[cpt] == 'A'))
//          {
//            digitalWrite(ORANGE_LED, HIGH);
//            delay(100);
//            digitalWrite(ORANGE_LED, LOW);
//            delay(100);
//            digitalWrite(ORANGE_LED, HIGH);
//            delay(100);
//            digitalWrite(ORANGE_LED, LOW);
//          }
//      }
//      break;
//    case GPS_TIMEOUT:
//      strcpy(nmeaRmcSentence, "GPS TO\r\n");
//      break;
//    default:
//      strcpy(nmeaRmcSentence, "GPS E\r\n");
//      break;
//  }
//}
//
//void readNmeaGgaSentence()
//{
//  GPS_status_enum gpsReadingStatus;
//
//  // NMEA GGA sentence reading
//  gpsReadingStatus = nmeaGPS.readGGA(nmeaGgaSentence);
//  switch (gpsReadingStatus)
//  {
//    case GPS_OK:
//      break;
//    case GPS_TIMEOUT:
//      strcpy(nmeaGgaSentence, "GPS TO\r\n");
//      break;
//    default:
//      strcpy(nmeaGgaSentence, "GPS E\r\n");
//      break;
//  }
//}
//
//void getTime()
//{
//  // Get time from NMEA GGA Sentence
//  int j = 0 ;
//  for(i=timeBegin;i!=CUSTOM_FRAME_FIELD_SEPARATOR;i++)
//  {
//      gpsTime[j]= nmeaGgaSentence[i];
//      j++;
//  }
//}
//
//void getLatitude()
//{
//  // Get time from NMEA GGA Sentence
//  int j = 0 ;
//  i=latitudeBegin;
//  while(i!=CUSTOM_FRAME_FIELD_SEPARATOR)
//    {
//      gpsLatitude[j]= nmeaGgaSentence[i];
//      i++;
//      j++;
//    }
//  i++;
//  gpsLatitude[j]= nmeaGgaSentence[i];
//}
//
//void getLongitude()
//{
//  // Get time from NMEA GGA Sentence
//  int j = 0 ;
//  i=longitudeBegin;
//  while(i!=CUSTOM_FRAME_FIELD_SEPARATOR)
//    {
//      gpsLongitude[j]= nmeaGgaSentence[i];
//      i++;
//      j++;
//    }
//  i++;
//  gpsLongitude[j]= nmeaGgaSentence[i];
//}
//
//void getNbSat()
//{
//  // Get time from NMEA GGA Sentence
//  int j = 0;
//  for(i=nbSatBegin;i!=CUSTOM_FRAME_FIELD_SEPARATOR;i++)
//  {
//      gpsNbSat[j]= nmeaGgaSentence[i];
//      j++;
//  }
//}
//
//void getAltitude()
//{
//  // Get time from NMEA GGA Sentence
//  int j = 0;
//  for(i=altitudeBegin;i!=CUSTOM_FRAME_FIELD_SEPARATOR;i++)
//  {
//      gpsAltitude[j]= nmeaGgaSentence[i];
//      j++;
//  }
//}
//
//void getDataStatus()
//{
//  // Get time from NMEA RMC Sentence
//  int j = 0 ;
//    for(i=dataStatusBegin;i!=CUSTOM_FRAME_FIELD_SEPARATOR;i++)
//    {
//        gpsDataStatus[j]= nmeaRmcSentence[i];
//        j++;
//    }
//}
//
//void getSpeedOverGround()
//{
//  // Get time from NMEA RMC Sentence
//  int j = 0;
//      for(i=speedOverGroundBegin;i!=CUSTOM_FRAME_FIELD_SEPARATOR;i++)
//      {
//          gpsSpeedOverGround[j]= nmeaRmcSentence[i];
//          j++;
//      }
//}
