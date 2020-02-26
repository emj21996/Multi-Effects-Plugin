/*
	RackAFX(TM)
	Applications Programming Interface
	Derived Class Object Definition
	Copyright(c) Tritone Systems Inc. 2006-2012

	Your plug-in must implement the constructor,
	destructor and virtual Plug-In API Functions below.
*/

#pragma once

// base class
#include "plugin.h"

// **** REVERB FILES
#include "CombFilter.h"
#include "Delay.h"
#include "DelayAPF.h"
#include "LPFCombFilter.h"
#include "OnePoleLPF.h"

// **** ENVELOPE
#include "BiQuadEMJ.h"
#include "EnvelopeDetectorEMJ.h"
#include "FiltersEMJ.h"

// **** DELAY
#include "DelayLineEMJ.h"
#include "TrivOscillatorEMJ.h"


// un-comment for advanced GUI API: see www.willpirkle.com for details and sample code
/*
#include "GUIViewAttributes.h"
#include "../vstgui4/vstgui/vstgui.h"
*/

// un-comment for pure custom VSTGUI: see www.willpirkle.com for details and sample code
//#include "VSTGUIController.h"

class CFinalProjectEMJ : public CPlugIn
{
public:
	// RackAFX Plug-In API Member Methods:
	// The followung 5 methods must be impelemented for a meaningful Plug-In
	//
	// 1. One Time Initialization
	CFinalProjectEMJ();

	// 2. One Time Destruction
	virtual ~CFinalProjectEMJ(void);

	// 3. The Prepare For Play Function is called just before audio streams
	virtual bool __stdcall prepareForPlay();

	// 4. processAudioFrame() processes an audio input to create an audio output
	virtual bool __stdcall processAudioFrame(float* pInputBuffer, float* pOutputBuffer, UINT uNumInputChannels, UINT uNumOutputChannels);

	// 5. userInterfaceChange() occurs when the user moves a control.
	virtual bool __stdcall userInterfaceChange(int nControlIndex);

	// OPTIONAL ADVANCED METHODS ------------------------------------------------------------------------------------------------
	// These are more advanced; see the website for more details
	//
	// 6. initialize() is called once just after creation; if you need to use Plug-In -> Host methods
	//				   such as sendUpdateGUI(), you must do them here and NOT in the constructor
	virtual bool __stdcall initialize();

	// 7. joystickControlChange() occurs when the user moves a control.
	virtual bool __stdcall joystickControlChange(float fControlA, float fControlB, float fControlC, float fControlD, float fACMix, float fBDMix);

	// 8. process buffers instead of Frames:
	// NOTE: set m_bWantBuffers = true to use this function
	virtual bool __stdcall processRackAFXAudioBuffer(float* pInputBuffer, float* pOutputBuffer, UINT uNumInputChannels, UINT uNumOutputChannels, UINT uBufferSize);

	// 9. rocess buffers instead of Frames:
	// NOTE: set m_bWantVSTBuffers = true to use this function
	virtual bool __stdcall processVSTAudioBuffer(float** inBuffer, float** outBuffer, UINT uNumChannels, int inFramesToProcess);

	// 10. MIDI Note On Event
	virtual bool __stdcall midiNoteOn(UINT uChannel, UINT uMIDINote, UINT uVelocity);

	// 11. MIDI Note Off Event
	virtual bool __stdcall midiNoteOff(UINT uChannel, UINT uMIDINote, UINT uVelocity, bool bAllNotesOff);


	// 12. MIDI Modulation Wheel uModValue = 0 -> 127
	virtual bool __stdcall midiModWheel(UINT uChannel, UINT uModValue);

	// 13. MIDI Pitch Bend
	//					nActualPitchBendValue = -8192 -> 8191, 0 is center, corresponding to the 14-bit MIDI value
	//					fNormalizedPitchBendValue = -1.0 -> +1.0, 0 is at center by using only -8191 -> +8191
	virtual bool __stdcall midiPitchBend(UINT uChannel, int nActualPitchBendValue, float fNormalizedPitchBendValue);

	// 14. MIDI Timing Clock (Sunk to BPM) function called once per clock
	virtual bool __stdcall midiClock();


	// 15. all MIDI messages -
	// NOTE: set m_bWantAllMIDIMessages true to get everything else (other than note on/off)
	virtual bool __stdcall midiMessage(unsigned char cChannel, unsigned char cStatus, unsigned char cData1, unsigned char cData2);

	// 16. initUI() is called only once from the constructor; you do not need to write or call it. Do NOT modify this function
	virtual bool __stdcall initUI();

	// 17. Custom GUI and Advanced GUI API support
	virtual void* __stdcall showGUI(void* pInfo);

	// 18. process aux inputs (for sidechain capability; optional, not used in FX book projects)
	virtual bool __stdcall processAuxInputBus(audioProcessData* pAudioProcessData);

	// 19. thread-safe implementation for updating GUI controls from plugin
	virtual bool __stdcall checkUpdateGUI(int nControlIndex, float fValue, CLinkedList<GUI_PARAMETER>& guiParameters, bool bLoadingPreset);

	// --- override for sample accurate MIDI support
	virtual void __stdcall processRackAFXMessage(UINT uMessage, PROCESS_INFO& processInfo);

	// --- function to handle VST sample accurate parameter updates
	void doVSTSampleAccurateParamUpdates();

	// --- declare an EventList interface
	IMidiEventList* m_pMidiEventList;

	// Add your code here: ----------------------------------------------------------- //

	// ***** FOR ENEVLOPE FOLLOWER
		// two LPF BiQuads for the filters
		CBiQuadEMJ m_LeftLPF;
		CBiQuadEMJ m_RightLPF;

		CBiQuadEMJ m_LeftHSF;
		CBiQuadEMJ m_RightHSF;

		CBiQuadEMJ m_LeftPARA;
		CBiQuadEMJ m_RightPARA;

		CFiltersEMJ m_HSF;

		// function to calculate the new fc from the Envelope Value
		float calculateCutoffFreq(float fEnvelopeSample);

		float calculateGain(float fEnvelopeSample);

		// Two functions to calculate the BiQuad Coeffs
		void calculateLeftLPFCoeffs(float fCutoffFreq, float fQ);
		void calculateRightLPFCoeffs(float fCutoffFreq, float fQ);

		// min/max variables
		float m_fMinCutoffFreq;
		float m_fMaxCutoffFreq;

		float m_f_peakGain;

		// envelope detectors
		CEnvelopeDetectorEMJ m_LeftDetector;
		CEnvelopeDetectorEMJ m_RightDetector;
		// have to replace these detectors with PEQ detectors 

	// **** FOR DELAY
		// --- our lonely LFO; maybe it needs friends?
			CTrivOscillatorEMJ lfo;

			// for HPF in normal Delay feedback loop
			void calculateHPFCoeffs(float fCutoffFreq, float fQ);	//first order LPF 

			void calculateLinkwitzHigh(float fCutoffFreq, float fQ);
			void calculateLinkwitzLow(float fCutoffFreq, float fQ);

			// --- delay lines
			CDelayLineEMJ m_LeftDelay;
			CDelayLineEMJ m_RightDelay;

			// for HPF in delay fb loop
			CBiQuad m_HPF;

			// for bass chorus Linkwitz - Riley filters
			CBiQuad m_LHPF;
			CBiQuad m_LLPF;

			// --- modulated delay boundaries
			float minDelayMod_mSec;
			float maxDelayMod_mSec;

			// --- calculate the mod delay in mSec based on LFO value
			float calculateModDelayTime_mSec(float modValue);

			enum { LEFT, RIGHT }; // left and right delays 
			float doNormalDelay(float xn, UINT channel);

			// modified delay function for mod delay 
			float doModDelay(float xn, float delay_mSec, float feedback, UINT channel);




	// **** FOR REVERB 

			CDelay m_PreDelay;
			CDelayAPF m_OutputDly1;

			CDelayAPF m_fAPF4_In;
			CDelayAPF m_fAPF7_In;
			CDelayAPF m_fAPF10_In;

			// parallel Comb Bank 1
			CCombFilter m_ParallelCF_1;
			CCombFilter m_ParallelCF_2;
			CCombFilter m_ParallelCF_3;
			CCombFilter m_ParallelCF_4;


			// All Pass Fitler Bank
			CDelayAPF m_APF1;
			CDelayAPF m_APF2;
			CDelay m_Delay3; // sum
			CDelayAPF m_APF4;
			CDelayAPF m_APF5;
			CDelay m_Delay6; // sum
			CDelayAPF m_APF7;
			CDelayAPF m_APF8;
			CDelay m_Delay9; // sum
			CDelayAPF m_APF10;
			CDelayAPF m_APF11;
			CDelay m_Delay12; // sum

							  // damping
			COnePoleLPF m_DampingLPF1;
			COnePoleLPF m_DampingLPF2;

			// function to cook all member object's variables at once
			void cookVariables();	 

		// **** TREMOLO
			// a single LFO
			CWaveTable m_LFO;

			// a function to calculate the amplitude based on LFO
			float calculateGainFactor(float fLFOSample);

			// calculate constant power pan gains
			void calculatePannerGainFactor(float fLFOSample, float* pLeftVolume, float* pRightVolume);




	// END OF USER CODE -------------------------------------------------------------- //


	// ADDED BY RACKAFX -- DO NOT EDIT THIS CODE!!! ----------------------------------- //
	//  **--0x07FD--**

	float m_fPreGain_dB;
	float m_fThreshold;
	float m_fAttack_mSec;
	float m_fRelease_mSec;
	float m_fQ;
	float m_Fc;
	float m_f_maxGain;
	float m_fPEQ_Fc;
	float m_fPEQ_Q;
	float m_fPEQ_Gain_dB;
	float m_fDelay_mSec;
	float m_fFeedback_pct;
	float m_fDelayMix_pct;
	float m_f_HPF;
	float m_fRate;
	float m_fDepth_pct;
	float m_fResonance_pct;
	float m_fWet_pct_modDelay;
	float m_f_Fc_Hz;
	float m_fWet_pct_reverb;
	float m_f_APF1_Delay_mSec;
	float m_f_APF2_Delay_mSec;
	float m_f_APF3_Delay_mSec;
	float m_f_APF4_Delay_mSec;
	float m_f_APF5_Delay_mSec;
	float m_f_APF6_Delay_mSec;
	float m_f_APF7_Delay_mSec;
	float m_f_APF8_Delay_mSec;
	float m_fDelay1_mSec;
	float m_fDelay2_mSec;
	float m_fDelay3_mSec;
	float m_fDelay4_mSec;
	float m_fKrt_1;
	float m_fLPF_g;
	float m_fModRate;
	float m_fModDepth;
	UINT m_uFilterStructure;
	enum{DF,TDF,CF,TCF};
	UINT m_f_GC;
	enum{SWITCH_OFF,SWITCH_ON};
	UINT m_uCoupleQ;
	UINT m_uDelayType_2;
	enum{delay,flanger,basschorus};
	UINT m_uDirection;
	enum{UP,DOWN};
	UINT m_uPluginType;
	enum{Envelope,Delay,Reverb,Trem};
	UINT m_uLFOType;
	enum{TRI,SINE,SQUARE,SAW};
	UINT m_uMode;
	enum{Tremolo,Panner};

	// **--0x1A7F--**
	// ------------------------------------------------------------------------------- //

};
