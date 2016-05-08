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
#ifndef PINS_h
#define PINS_h

/**
 * User switch, used during reset to clear counters and erase SD file content
 */
#define USER_SWITCH_PIN 5

/**
 * output pin for FSK modulated signal
 */
#define FSK_MODULATOR_TX_PIN 6

/**
 * Internal temperature sensor analog channel
 */
#define INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL A0

/**
 * Differential pressure sensor analog channel
 */
#define DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL A1

/**
 * Battery voltage sensor analog channel
 */
#define BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL A2

/**
 * Battery temperature sensor analog channel
 */
#define BATTERY_TEMPERATURE_ANALOG_SENSOR_CHANNEL A7

/**
 * SD card chip-select pin
 */
#define SD_CARD_CHIP_SELECT_PIN 53

/**
 * RealTime Clock chip enable pin
 */
#define RTC_CE_PIN 7

/**
 * RealTime Clock IO pin
 */
#define RTC_IO_PIN 9

/**
 * RealTime Clock SCLK pin
 */
#define RTC_SCLK_PIN 10

/**
 * Red LED pin
 */
#define RED_LED_PIN 11

/**
 * Green LED pin
 */
#define GREEN_LED_PIN 8

/**
 * SR04 TRIG pin
 */
#define SR04_TRIG_PIN 3

/**
 * SR04 ECHO pin
 */
#define SR04_ECHO_PIN 4

#endif
