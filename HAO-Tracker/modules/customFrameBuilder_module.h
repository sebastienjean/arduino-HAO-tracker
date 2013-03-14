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

#define CUSTOM_FRAME_MAX_LENGTH 150

# define CUSTOM_FRAME_FIELD_SEPARATOR_CHAR ','

# define CUSTOM_FRAME_START_OF_FRAME_CHAR '#'

#define CUSTOM_FRAME_END_OF_FRAME_STRING "\r\n"

/**
 * Builds custom frame (time, location, sensor data, ...)
 *
 * @param customFrame the external buffer used to store custom frame
 */
void buildCustomFrame(char * customFrame);
