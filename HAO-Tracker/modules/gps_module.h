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

void initGPS();

void readNmeaRmcSentence();

void readNmeaGgaSentence();
