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
#include <pins.h>
#include <SD.h>

#include <Logger.h>

// log File
  char * filePath;


  boolean Logger::begin(char * filePath, int sd_CS_Pin)
  {
    this->filePath = filePath;
    pinMode(SD_CARD_CHIP_SELECT_PIN, OUTPUT);
    return SD.begin(SD_CARD_CHIP_SELECT_PIN);
  }
  /**
   * Logs a message.
   * @param message the string to be logged
   * @param newLine line termination characters appending (if true)
   * @return logging success status
   */
  boolean Logger::logMessage(char *message, boolean newLine)
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
  /**
   * Erase log file content
   * @return log file deletion status
   */
  boolean Logger::reset(void)
  {
    return SD.remove(this->filePath);
  }

  Logger LOGGER;
