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
#include <pins.h>
#include <defs.h>

#include <leds_module.h>

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
 * Turns all LEDs on.
 */
void allLEDsOn()
{
  digitalWrite(RED_LED, HIGH);
  digitalWrite(ORANGE_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
}

/**
 * Turns all LEDs off.
 */
void allLEDsOff(void)
{
  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}

/**
 * Turns a single LED on.
 *
 * @param ledNumber the led to turn on
 */
void singleLedOn(int ledNumber)
{
  digitalWrite(ledNumber, HIGH);
}

/**
 * Turns a single LED off.
 *
 * @param ledNumber the led to turn off
 */
void singleLedOff(int ledNumber)
{
  digitalWrite(ledNumber, LOW);
}

/**
 * Plays LEDs startup sequence.
 */
void showLEDsStartupSequence()
{
  allLEDsOn();
  delay(1000);
  allLEDsOff();
}

/**
 * Displays status (OK/KO) using a given LED
 * @param ledNumber the led to use
 * @param status the status to display (OK/KO)
 */
void showStatus(int ledNumber, boolean status)
{
  digitalWrite(ledNumber, status);
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


