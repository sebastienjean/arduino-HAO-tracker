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

#ifndef LOGGER_h
#define LOGGER_h

class Logger
{
private:

  char *filePath;

public:

  boolean begin(char *path, int sd_CE_Pin);

  /**
   * Logs a message.
   * @param message the string to be logged
   * @param newLine line termination characters appending (if true)
   * @return logging success status
   */
  boolean logMessage(char *message, boolean newLine);

  /**
   * Erase log file content
   * @return log file deletion status
   */
  boolean reset(void);
};

extern Logger LOGGER;
#endif

