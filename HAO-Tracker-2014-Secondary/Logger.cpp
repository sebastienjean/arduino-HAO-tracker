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
#include <SD.h>

#include "Logger.h"

boolean
Logger::begin(char * filePath, int sd_CS_Pin)
{
  this->filePath = filePath;
  pinMode(sd_CS_Pin, OUTPUT);
  return SD.begin(sd_CS_Pin);
}

boolean
Logger::logMessage(char *message, boolean newLine)
{
  File logFile = SD.open(this->filePath, FILE_WRITE);
  if (logFile)
    {
      if (newLine)
        logFile.println(message);
      else
        logFile.print(message);
      logFile.close();
      return true;
    }
  return false;
}

boolean
Logger::reset(void)
{
  return SD.remove(this->filePath);
}

Logger LOGGER;
