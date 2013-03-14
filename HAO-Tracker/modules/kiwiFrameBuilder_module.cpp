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
#include <kiwiFrameBuilder_module.h>
#include <analogSensors_module.h>

// kiwi frame
unsigned char kiwiFrame[KIWI_FRAME_LENGTH];

/**
 * Sets the content of a given kiwi frame channel field (fields 1 to 8), from an integer value supposed to be the
 * digital-to-analog conversion of an analog input (i.e. the return of a call to analogRead).
 * @param fieldNumber the field number to set (in [1, 8])
 * @param analogReadValue the value to which the field has to be set
 */
void setKiwiFrameChannelFieldFromAnalogReadValue(int fieldNumber, int analogReadValue)
{
  if ((fieldNumber < 1) || (fieldNumber > 8)) return;

  kiwiFrame[fieldNumber] = (unsigned char) (analogReadValue / 4);
  if (kiwiFrame[fieldNumber] == 0xFF)
    kiwiFrame[fieldNumber] = 0xFE;
}

/**
 * Sets the content of a the kiwi frame voltage field (field 9), from an int value supposed to be the
 * digital-to-analog conversion of an analog input (i.e. the return of a call to analogRead).
 * @param analogReadValue the value to which the field has to be set
 */
void setKiwiFrameVoltageFieldFromAnalogReadValue(int analogReadValue)
{
  kiwiFrame[9] = (unsigned char) (analogReadValue / 8);
  if (kiwiFrame[9] == 0xFF)
    kiwiFrame[9] = 0xFE;
}

/**
 * Computes and sets the kiwi frame checksum field (at offset KIWI_FRAME_LENGTH-1]
 * (all other fields are supposed to be set before a call to this function).
 */
void computeKiwiFrameChecksum()
{
  unsigned char kiwiFrameChecksum;
  for (int cpt = 1; cpt < KIWI_FRAME_LENGTH - 1; cpt++)
    kiwiFrameChecksum = (unsigned char) ((kiwiFrameChecksum + kiwiFrame[cpt]) % 256);

  kiwiFrameChecksum = (unsigned char) (kiwiFrameChecksum / 2);
  kiwiFrame[KIWI_FRAME_LENGTH-1] = kiwiFrameChecksum;
}

/**
 * Builds kiwi frame (channel fields, voltage field, checksum) from values retrieved from sensors variables.
 */
void buildKiwiFrame()
{
  // start-of-frame
  kiwiFrame[0] = 0xFF;

  // channel 1: absolute pressure
  setKiwiFrameChannelFieldFromAnalogReadValue(1,readSensor(1));

  // channel 2: differential pressure
  setKiwiFrameChannelFieldFromAnalogReadValue(2,readSensor(2));

  // channel 3: internal temperature
  setKiwiFrameChannelFieldFromAnalogReadValue(3,readSensor(3));

  // channel 4: external temperature
  setKiwiFrameChannelFieldFromAnalogReadValue(4,readSensor(4));

  // channel 5: <empty>
  kiwiFrame[5] = 0x00;

  // channel 6: <empty>
  kiwiFrame[6] = 0x00;

  // channel 7: <empty>
  kiwiFrame[7] = 0x00;

  // channel 8: <empty>
  kiwiFrame[8] = 0x00;

  // voltage
  setKiwiFrameVoltageFieldFromAnalogReadValue(readSensor(4));

  // checksum
  computeKiwiFrameChecksum();
}
