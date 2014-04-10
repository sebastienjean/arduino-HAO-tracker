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

#include <AnalogSensor.h>
#include <AnalogSensors.h>

/**
 * Kiwi frame length
 */
#define KIWI_FRAME_LENGTH 11

/**
 * Number of KIWI frame fields available for sensor data
 */
#define KIWI_FRAME_CHANNELS_AMOUNT 8

/**
 * This class allows to build KIWI (CNES/Planete-Sciences legacy format) frame to be sent by the HAO, retrieving data from sensors, RTC, GPS, ...
 * The considered frame format is the one defined in v3.0 of CNES Kiwi Millenium user manual (12/03/2009)
 */
class KiwiFrameBuilder
{
private:

  /**
   * Internal pointer on kiwi frame buffer used across several internal functions
   */
  unsigned char *kiwiFrame;

  /**
   * Pointer to sensors
   */
  AnalogSensors *sensors;

  /**
   * Pointer to voltage sensor
   */
  AnalogSensor *voltage;

  /**
   * Sets the content of a given kiwi frame channel field (fields 1 to 8), from an analog sensor value
   *
   * @param fieldNumber the field number to set (in [1, 8])
   * @param value sensor value
   */
  void
  setKiwiFrameChannelField(int fieldNumber, int value);

  /**
   * Sets the content of a the kiwi frame voltage field (field 9), from a voltage sensor value
   *
   * @param value voltage sensor value
   */
  void
  setKiwiFrameVoltageField(int value);

  /**
   * Computes and sets the kiwi frame checksum field (at offset KIWI_FRAME_LENGTH-1]
   * (all other fields are supposed to be set before a call to this function).
   */
  void
  computeKiwiFrameChecksum();

public:

  /**
   * Creates a new kiwi frame builder retrieving data from given providers.
   *
   * @param sensors analog sensor data provider
   * @param voltage voltage data provider
   */
  KiwiFrameBuilder(AnalogSensors *sensors, AnalogSensor *voltage);

  /**
   * Builds kiwi frame (channel fields, voltage field, checksum) from values retrieved from data providers.
   * @param kiwiFrame an external buffer of at least <tt>KIWI_FRAME_LENGTH</tt> bytes,
   * to store kiwi frame bytes
   */
  void
  buildKiwiFrame(unsigned char *kiwiFrame);
};

#endif

