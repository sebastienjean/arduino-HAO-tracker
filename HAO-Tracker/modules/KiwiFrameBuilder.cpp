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
#include <defs.h>
#include <KiwiFrameBuilder.h>
#include <AnalogSensor.h>
#include <AnalogSensors.h>


/**
 * Sets the content of a given kiwi frame channel field (fields 1 to 8), from an integer value supposed to be the
 * digital-to-analog conversion of an analog input (i.e. the return of a call to analogRead).
 * @param fieldNumber the field number to set (in [1, 8])
 * @param analogReadValue the value to which the field has to be set
 */
void KiwiFrameBuilder::setKiwiFrameChannelFieldFromAnalogReadValue(int fieldNumber, int analogReadValue)
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
void KiwiFrameBuilder::setKiwiFrameVoltageFieldFromAnalogReadValue(int analogReadValue)
{
  this->kiwiFrame[9] = (unsigned char) (analogReadValue / 8);
  if (this->kiwiFrame[9] == 0xFF)
    this->kiwiFrame[9] = 0xFE;
}

/**
 * Computes and sets the kiwi frame checksum field (at offset KIWI_FRAME_LENGTH-1]
 * (all other fields are supposed to be set before a call to this function).
 */
void KiwiFrameBuilder::computeKiwiFrameChecksum()
{
  unsigned char kiwiFrameChecksum;
  for (int cpt = 1; cpt < KIWI_FRAME_LENGTH - 1; cpt++)
    kiwiFrameChecksum = (unsigned char) ((kiwiFrameChecksum + this->kiwiFrame[cpt]) % 256);

  kiwiFrameChecksum = (unsigned char) (kiwiFrameChecksum / 2);
  this->kiwiFrame[KIWI_FRAME_LENGTH-1] = kiwiFrameChecksum;
}

KiwiFrameBuilder::KiwiFrameBuilder(AnalogSensors *sensors, AnalogSensor *voltage)
{
  this->sensors = sensors;
  this->voltage = voltage;
}

/**
 * Builds kiwi frame (channel fields, voltage field, checksum) from values retrieved from sensors variables.
 */
void KiwiFrameBuilder::buildKiwiFrame(unsigned char *kiwiFrame)
{
  this->kiwiFrame = kiwiFrame;

  // start-of-frame
  this->kiwiFrame[0] = 0xFF;

  for (int i=1;i<KIWI_FRAME_LENGTH;i++)
  {
      this->kiwiFrame[i] = 0x00;
  }

  // channels : analog sensors
  for (int i=1;(i<this->sensors->getAmount())&&(i<KIWI_FRAME_CHANNELS_AMOUNT);i++)
  {
      setKiwiFrameChannelFieldFromAnalogReadValue(1, this->sensors->read(i));
  }

  // voltage
  setKiwiFrameVoltageFieldFromAnalogReadValue(this->voltage->read());

  // checksum
  computeKiwiFrameChecksum();
}

