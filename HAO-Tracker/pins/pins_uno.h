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

#ifndef PINS_UNO_h
#define PINS_UNO_h

#define USER_BUTTON_PIN 5

#define RED_LED_PIN 3

#define ORANGE_LED_PIN 4

#define GREEN_LED_PIN 11

#define BLUE_LED_PIN 12

#define FSK_MODULATOR_TX_PIN 6

#define GPS_SERIAL_RX_PIN 2

// unused but required for software serial initialization
#define GPS_SERIAL_TX_PIN 3

#define DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL A0

#define ABSOLUTE_PRESSURE_ANALOG_SENSOR_CHANNEL A1

#define EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL A2

#define INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL A3

#define BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL A4

#define SD_CARD_CHIP_SELECT_PIN 8

#define RTC_CE_PIN   7

#define RTC_IO_PIN   9

#define RTC_SCLK_PIN 10

#endif