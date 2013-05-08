/*#include <jouer.h>

int pin = 10;
Jouer jouer(pin);

void setup()
{
}

void loop()
{}*/

#include <Tone.h>

#include <Arduino.h>

Tone::Tone(int pin)
{
	pinMode(this->tonePin,OUTPUT);
	this->tonePin = pin;
}

int Tone::noteAFreq(int note, int oct)
{
    int freq = note;
    int i = 0;
    while(i != oct)
    {
        freq = (freq * 2);
        i++;
    }
    return(freq);
}

double Tone::mesureATmp(int bpm, double mesure)
{
    double tmp = 0;
    tmp = (mesure / bpm) * 60 * 1000;
    return(tmp);
}

void Tone::melody(int note, int oct, double mesure, int bpm)
{
    int freq;
    double tmp;
    freq = noteAFreq(note, oct);
    tmp = mesureATmp(bpm, mesure);
    tone(this->tonePin, freq, tmp);
    
    int pauseBetweenNotes = tmp* 1.30;
    delay(pauseBetweenNotes);
    noTone(this->tonePin);
}

