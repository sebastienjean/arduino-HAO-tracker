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

#ifndef AD7995_h
#define AD7995_h

#define AD7995_0_ADDRESS  0b01010000

#define AD7995_1_ADDRESS  0b01010010

/**
 * This class allows to handle an I2C AD7995 (both 0 or 1 models) ADC
 */
class AD7995
{
private:

  /**
   * Address on the I2C bus
   */
  int address;

public:

  /**
   * Creates a AD7995 ADC instance.
   *
   * @param address address on the I2C bus
   */
  AD7995(int address);

  /**
   * Reads analog sensor value.
   *
   * @param channel analog channel number
   * @return analog sensor value
   */
  int
  read(int channel);

};

#endif

