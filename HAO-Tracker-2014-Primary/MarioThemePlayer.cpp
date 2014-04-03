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

#include <MarioThemePlayer.h>

  const Note* MarioThemePlayer::MARIO_THEME_MELODY[MARIO_THEME_MELODY_LENGTH] =
      {
       new Note(Note::E, 6, Note::SIXTEENTH),
       new Note(Note::SILENCE, 6, Note::TWO_HUNDRED_SIXTY_FOURTH),
       new Note(Note::E, 6, Note::EIGHTH),
       new Note(Note::SILENCE, 6, Note::TWO_HUNDRED_SIXTY_FOURTH),
       new Note(Note::E, 6, Note::SIXTEENTH),
       new Note(Note::SILENCE, 6, Note::SIXTEENTH),
       new Note(Note::C, 6, Note::SIXTEENTH),
       new Note(Note::SILENCE, 6, Note::TWO_HUNDRED_SIXTY_FOURTH),
       new Note(Note::E, 6, Note::EIGHTH),
       new Note(Note::SILENCE, 6, Note::TWO_HUNDRED_SIXTY_FOURTH),
       new Note(Note::G, 6, Note::QUARTER),
       new Note(Note::SILENCE, 6, Note::TWO_HUNDRED_SIXTY_FOURTH),
       new Note(Note::G, 5, Note::QUARTER)
      };

MarioThemePlayer::MarioThemePlayer(int outputPin)
{
  this->melodyGenerator = new MelodyGenerator(new NoteGenerator(new AudioToneGenerator(outputPin)));
  this->melodyGenerator->setNotes(MARIO_THEME_MELODY_LENGTH, (Note **) (&MARIO_THEME_MELODY));
}

void
MarioThemePlayer::playMarioTheme()
{
  this->melodyGenerator->generateMelody(MARIO_THEME_BPM);
}


