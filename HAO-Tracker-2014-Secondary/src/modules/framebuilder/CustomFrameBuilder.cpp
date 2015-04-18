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
#include <stdlib.h>

#include <Counters.h>

#include <main/defs.h>

#include <modules/framebuilder/CustomFrameBuilder.h>

CustomFrameBuilder::CustomFrameBuilder(Counters *counters)
{
  this->counters = counters;
}


void
CustomFrameBuilder::appendStartOfFrameChar()
{
  this->whereToAppend++[0] = CUSTOM_FRAME_START_OF_FRAME_CHAR;
}

void
CustomFrameBuilder::appendFieldSeparatorChar()
{
  this->whereToAppend++[0] = CUSTOM_FRAME_FIELD_SEPARATOR_CHAR;
}

void
CustomFrameBuilder::appendEndOfFrameString()
{
  strcpy(this->whereToAppend, CUSTOM_FRAME_END_OF_FRAME_STRING);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendHaoName()
{
  strcpy(this->whereToAppend, HAO_NAME);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendCounters()
{
  for (int i = 1; i <= this->counters->getAmount(); i++)
  {
    // append next counter value
    itoa(this->counters->read(i), this->whereToAppend, 10);
    this->whereToAppend += strlen(this->whereToAppend);

    // separator (if not last)
    if (i < this->counters->getAmount())
    {
      this->appendFieldSeparatorChar();
    }
  }
}

void
CustomFrameBuilder::appendSystemTime()
{
  // seconds elapsed since last reset, as a decimal coded ASCII string
  itoa((millis() * 5) / 2500, this->whereToAppend, 10);
  this->whereToAppend += strlen(this->whereToAppend);
}


void
CustomFrameBuilder::buildCustomFrame(char *customFrame)
{
  this->whereToAppend = customFrame;

  this->appendStartOfFrameChar();

  // HAO name
  this->appendHaoName();

  // separator
  this->appendFieldSeparatorChar();

  // counters
  this->appendCounters();

  // separator
  this->appendFieldSeparatorChar();

  // system time
  this->appendSystemTime();

  // end of frame
  this->appendEndOfFrameString();
}
