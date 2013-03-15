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

#ifndef KIWI_FRAME_BUILDER_h
#define KIWI_FRAME_BUILDER_h

#include <VoltageMonitor.h>
#include <AnalogSensors.h>

#define KIWI_FRAME_LENGTH 11
#define KIWI_FRAME_CHANNELS_AMOUNT 8

class KiwiFrameBuilder
{
private:

  unsigned char *kiwiFrame;
  VoltageMonitor *voltageMonitor;
  AnalogSensors *sensors;

  /**
   * Sets the content of a given kiwi frame channel field (fields 1 to 8), from an integer value supposed to be the
   * digital-to-analog conversion of an analog input (i.e. the return of a call to analogRead).
   * @param fieldNumber the field number to set (in [1, 8])
   * @param analogReadValue the value to which the field has to be set
   */
  void setKiwiFrameChannelFieldFromAnalogReadValue(int fieldNumber, int analogReadValue);
  /**
   * Sets the content of a the kiwi frame voltage field (field 9), from an int value supposed to be the
   * digital-to-analog conversion of an analog input (i.e. the return of a call to analogRead).
   * @param analogReadValue the value to which the field has to be set
   */
  void setKiwiFrameVoltageFieldFromAnalogReadValue(int analogReadValue);

  /**
   * Computes and sets the kiwi frame checksum field (at offset KIWI_FRAME_LENGTH-1]
   * (all other fields are supposed to be set before a call to this function).
   */
  void computeKiwiFrameChecksum();

  public:

  KiwiFrameBuilder(VoltageMonitor *voltageMonitor, AnalogSensors *sensors);

  /**
   * Builds kiwi frame (channel fields, voltage field, checksum) from values retrieved from sensors variables.
   */
  void buildKiwiFrame(unsigned char *kiwiFrame);
};

#endif



