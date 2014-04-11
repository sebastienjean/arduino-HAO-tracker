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

#include "KiwiFrameBuilder.h"

void
KiwiFrameBuilder::setKiwiFrameChannelField(uint8_t fieldNumber, uint16_t value, uint8_t adcResolution)
{
  if ((fieldNumber < 1) || (fieldNumber > KIWI_FRAME_CHANNELS_AMOUNT))
    return;
  for ( uint8_t resolutionFactor = adcResolution - KIWI_ADC_RESOLUTION; resolutionFactor > 0; resolutionFactor--)
    value = value / 2;

  kiwiFrame[fieldNumber] = (uint8_t) value;
  if (kiwiFrame[fieldNumber] == 0xFF)
    kiwiFrame[fieldNumber] = 0xFE;
}

void
KiwiFrameBuilder::setKiwiFrameVoltageField(uint16_t value)
{
  float voltageAdcStep = KIWI_ADC_VOLTAGE_REFERENCE;

  for ( uint8_t resolution = (this->voltageAnalogSensor->getAdcResolution()); resolution > 0; resolution--)
    voltageAdcStep = voltageAdcStep / 2.0;

  float realVoltage = (value*voltageAdcStep)*this->voltageDownscalingFactor;
  float kiwiVoltage = realVoltage  / ((float) KIWI_VOLTAGE_DOWNSCALING_FACTOR);
  uint16_t kiwiValue = (uint16_t) (kiwiVoltage / voltageAdcStep);

  this->setKiwiFrameChannelField(9, kiwiValue,this->voltageAnalogSensor->getAdcResolution());
}

void
KiwiFrameBuilder::computeKiwiFrameChecksum()
{
  unsigned int kiwiFrameChecksum = 0;
  for (int cpt = 1; cpt < KIWI_FRAME_LENGTH - 1; cpt++)
    kiwiFrameChecksum = kiwiFrameChecksum
        + (unsigned char) (this->kiwiFrame[cpt]);

  kiwiFrameChecksum = kiwiFrameChecksum % 256;
  kiwiFrameChecksum = kiwiFrameChecksum / 2;
  this->kiwiFrame[KIWI_FRAME_LENGTH - 1] = (unsigned char) kiwiFrameChecksum;
}

KiwiFrameBuilder::KiwiFrameBuilder(AnalogSensors *sensors, AnalogSensor *voltage, float voltageDownscalingFactor)
{
  this->analogSensors = sensors;
  this->voltageAnalogSensor = voltage;
  this->voltageDownscalingFactor = voltageDownscalingFactor;
}

void
KiwiFrameBuilder::buildKiwiFrame(unsigned char *kiwiFrame)
{
  this->kiwiFrame = kiwiFrame;

  // start-of-frame
  this->kiwiFrame[0] = 0xFF;

  for (uint8_t kiwiFrameOffset = 1; kiwiFrameOffset < KIWI_FRAME_LENGTH; kiwiFrameOffset++)
    {
      this->kiwiFrame[kiwiFrameOffset] = 0x00;
    }

  // channels : analog sensors
  for (uint8_t analogSensorNumber = 0;
      (analogSensorNumber < this->analogSensors->getAmount()) && (analogSensorNumber < KIWI_FRAME_CHANNELS_AMOUNT); analogSensorNumber++)
    {
      AnalogSensor *analogSensor = this->analogSensors->getAnalogSensor(analogSensorNumber);
      setKiwiFrameChannelField(analogSensorNumber+1, analogSensor->read(), analogSensor->getAdcResolution());
    }

  // voltage
  setKiwiFrameVoltageField(this->voltageAnalogSensor->read());

  // checksum
  computeKiwiFrameChecksum();
}

