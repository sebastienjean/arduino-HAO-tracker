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

#ifndef COUNTER_h
#define COUNTER_h

class Counter
{
private:

  int baseAddress;

public:

  Counter(int baseAddress);

  /**
   * Reads counter value.
   *
   * @return counter value
   *
   */
  int read(void);

  /**
   * Sets counter value.
   *
   * @param value counter value
   *
   */
  void set(int value);

  /**
   * Increments counter value.
   *
   * @param value the increment value
   *
   */
  void increment(int value);
};

#endif


