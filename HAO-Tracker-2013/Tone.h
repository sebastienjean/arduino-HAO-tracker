#ifndef TONE_h
#define TONE_h

#include <Arduino.h>

#define Do 32.70
#define Reb 34.65
#define Re 36.71
#define Mib 38.89
#define Mi 41.20
#define Fa 43.65
#define Solb 46.25
#define Sol 49
#define Lab 51.91
#define La 55
#define Sib 58.27
#define Si 61.74


#define carre 8
#define ronde 4
#define blanche 2
#define noire 1
#define croche 0.5
#define dcroche 0.25
#define tcroche 0.125
#define qcroche 0.0625

#define carrep 12
#define rondep 6
#define blanchep 3
#define noirep 1.5
#define crochep 0.75
#define dcrochep 0.375
#define tcrochep 0.1875
#define qcrochep 0.09375

#define batonp 8
#define pause 4
#define dpause 2
#define soupir 1
#define dsoupir 0.5
#define qsoupir 0.25
#define hsoupir 0.125
#define ssoupir 0.0625

class Tone
{
	public:
		Tone(int pin);
		void melody(int note, int oct, double mesure, int bpm);
	private:
		int tonePin;

		int noteAFreq(int note, int oct);

		double mesureATmp(int bpm, double mesure);
};

#endif
