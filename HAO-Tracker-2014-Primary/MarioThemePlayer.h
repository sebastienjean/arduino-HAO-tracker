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

#include <AudioToneGenerator.h>
#include <Note.h>
#include <NoteGenerator.h>
#include <MelodyGenerator.h>

#define MARIO_THEME_MELODY_LENGTH  13

#define MARIO_THEME_BPM 150

class MarioThemePlayer
{
private:

  MelodyGenerator *melodyGenerator;

  static const Note* MARIO_THEME_MELODY[];

public:

  MarioThemePlayer(int outputPin);

  /**
   * Plays Mario theme
   */
  void
  playMarioTheme();

};

