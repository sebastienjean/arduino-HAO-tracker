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

#define CUSTOM_FRAME_MAX_LENGTH 40

# define CUSTOM_FRAME_FIELD_SEPARATOR ','
// custom frame, as an ASCII string
extern char customFrame[];

// custom frame length
extern int customFrameLength;

/**
 * Builds custom frame (time, sensors data, ...) from values retrieved from global variables.
 */
void buildCustomFrame(void);
