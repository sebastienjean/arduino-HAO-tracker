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

#define USER_SWITCH_PIN 5

#define TAKE_OFF_SWITCH_PIN 4

#define RED_LED_PIN 46

#define ORANGE_LED_PIN 47

#define GREEN_LED_PIN 48

#define BLUE_LED_PIN 49

#define FSK_MODULATOR_TX_PIN 6

#define EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL A0
#define EXTERNAL_HUMIDITY_ANALOG_SENSOR_CHANNEL A1
#define INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL A2
#define LIGHT_HIGH_ANALOG_SENSOR_CHANNEL A3
#define LIGHT_SIDE1_ANALOG_SENSOR_CHANNEL A4
#define LIGHT_SIDE2_ANALOG_SENSOR_CHANNEL A5
#define BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL A6
#define DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL A7
#define SOUND_ANALOG_SENSOR_CHANNEL A8
#define NTC_LI_ION_TEMP_SENSOR_CHANNEL A9

#define SD_CARD_CHIP_SELECT_PIN 53

#define RTC_CE_PIN   7
#define RTC_IO_PIN   9
#define RTC_SCLK_PIN 10

#define LCD_CARD_E_PIN 39
#define LCD_CARD_RS_PIN 41
#define LCD_CARD_D4_PIN 49
#define LCD_CARD_D5_PIN 47
#define LCD_CARD_D6_PIN 45
#define LCD_CARD_D7_PIN 43

#define BPLED_CARD_SW1_PIN 29
#define BPLED_CARD_SW2_PIN 27
#define BPLED_CARD_SW3_PIN 25

#define BPLED_CARD_LED1_PIN 37
#define BPLED_CARD_LED2_PIN 35
#define BPLED_CARD_LED3_PIN 33
#define BPLED_CARD_LED4_PIN 31

#define SOUND_OUT_PIN 22

#define MOTORIZED_CAMERA_PWM_PIN 24

#define GROUND_CAMERA_PWM_PIN 26

#define SKY_CAMERA_PWM_PIN 28

#define ROTOR_PWM_PIN 30

#define MOTORIZED_CAMERA_PWR_PIN 34

#define GROUND_CAMERA_PWR_PIN 36

#define SKY_CAMERA_PWR_PIN 38

#endif
