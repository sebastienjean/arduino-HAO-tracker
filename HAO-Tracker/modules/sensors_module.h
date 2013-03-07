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

// absolute pressure sensor value
extern int absolutePressureSensorValue;

// differential pressure sensor value
extern int differentialPressureSensorValue;

// internal temperature sensor value
extern int internalTemperatureSensorValue;

// external temperature sensor value
extern int externalTemperatureSensorValue;

// battery voltage sensor value
extern int batteryVoltageSensorValue;

/**
 * Inits sensors.
 */
void initSensors(void);

/**
 * Reads all sensors values and stores them in appropriate variables.
 *
 */
void readSensors(void);
