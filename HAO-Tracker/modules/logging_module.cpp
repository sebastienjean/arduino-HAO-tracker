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
#include <pins.h>
#include <defs.h>

#include <leds_module.h>
#include <logging_module.h>

// log File
File logFile;

/**
 * Initializes logging (SD).
 * @return logging initialization success status
 */
boolean initLogging()
{
  pinMode(SD_CARD_CHIP_SELECT_PIN, OUTPUT);
  return SD.begin(SD_CARD_CHIP_SELECT_PIN);
}

/**
 * Logs a message.
 * @param message the string to be logged
 * @param newLine line termination characters appending (if true)
 * @return logging success status
 */
boolean logMessage(char *message, boolean newLine)
{
  logFile = SD.open(LOG_FILE_PATH, FILE_WRITE);
  if (logFile)
  {
    if (newLine)
      logFile.println(message);
    else
      logFile.print(message);
    logFile.close();
  }
  return logFile;
}

/**
 * Waits one second for user to decide if log file has to be deleted, deletes it if needed.
 * @return log file has been deletion status
 */
boolean deleteLogFileIfUserClaimsTo()
{
  delay(1000);

  if (digitalRead(USER_BUTTON_PIN) == LOW)
  {
      return SD.remove(LOG_FILE_PATH);
  }
  return false;
}
