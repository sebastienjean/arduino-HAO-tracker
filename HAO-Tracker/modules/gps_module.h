/*
 * gps_module.h
 *
 *  Created on: 8 mars 2013
 *      Author: BERARD / SIGNOBOS
 */

// buffer for NMEA-RMC sentence reading
extern char nmeaRmcSentence[];

// buffer for NMEA-GGA sentence reading
extern char nmeaGgaSentence[];


extern char gpsTime[32];
extern char gpsLatitude[32];
extern char gpsLongitude[32];
extern char gpsNbSat[32];
extern char gpsAltitude[32];
extern char gpsDataStatus[32];
extern char gpsSpeedOverGround[32];

void initGPS();

void readNmeaRmcSentence();

void readNmeaGgaSentence();

void getTime();

void getLatitude();

void getLongitude();

void getNbSat();

void getAltitude();

void getDataStatus();

void getSpeedOverGround();
