#include "TrivOscillatorEMJ.h"

/*
CTrivOscillator: implements a trivial oscillator with several waveform choices

Copyright (c) 2017 Will Pirkle
Free for academic use.
*/

CTrivOscillatorEMJ::CTrivOscillatorEMJ()
{
	oscFrequency = 0.f;
	sampleRate = 0.f;
	oscWaveform = TRI;
	modCounter = 0.f;
	phaseInc = 0.f;
	modCounterQP = 0.f;

	reset();
}


CTrivOscillatorEMJ::~CTrivOscillatorEMJ()
{
}
