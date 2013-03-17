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

#include "defs.h"
#include "KiwiFrameBuilder.h"

#include "AnalogSensor.h"
#include "AnalogSensors.h"

void
KiwiFrameBuilder::setKiwiFrameChannelField(int fieldNumber, int value)
{
  if ((fieldNumber < 1) || (fieldNumber > 8))
    return;

  kiwiFrame[fieldNumber] = (unsigned char) (value / 4);
  if (kiwiFrame[fieldNumber] == 0xFF)
    kiwiFrame[fieldNumber] = 0xFE;
}

void
KiwiFrameBuilder::setKiwiFrameVoltageField(int value)
{
  this->kiwiFrame[9] = (unsigned char) (value / 8);
  if (this->kiwiFrame[9] == 0xFF)
    this->kiwiFrame[9] = 0xFE;
}

void
KiwiFrameBuilder::computeKiwiFrameChecksum()
{
  unsigned char kiwiFrameChecksum;
  for (int cpt = 1; cpt < KIWI_FRAME_LENGTH - 1; cpt++)
    kiwiFrameChecksum = (unsigned char) ((kiwiFrameChecksum
        + this->kiwiFrame[cpt]) % 256);

  kiwiFrameChecksum = (unsigned char) (kiwiFrameChecksum / 2);
  this->kiwiFrame[KIWI_FRAME_LENGTH - 1] = kiwiFrameChecksum;
}

KiwiFrameBuilder::KiwiFrameBuilder(AnalogSensors *sensors,
    AnalogSensor *voltage)
{
  this->sensors = sensors;
  this->voltage = voltage;
}

void
KiwiFrameBuilder::buildKiwiFrame(unsigned char *kiwiFrame)
{
  this->kiwiFrame = kiwiFrame;

  // start-of-frame
  this->kiwiFrame[0] = 0xFF;

  for (int i = 1; i < KIWI_FRAME_LENGTH; i++)
    {
      this->kiwiFrame[i] = 0x00;
    }

  // channels : analog sensors
  for (int i = 1;
      (i < this->sensors->getAmount()) && (i < KIWI_FRAME_CHANNELS_AMOUNT); i++)
    {
      setKiwiFrameChannelField(1, this->sensors->read(i));
    }

  // voltage
  setKiwiFrameVoltageField(this->voltage->read());

  // checksum
  computeKiwiFrameChecksum();
}

