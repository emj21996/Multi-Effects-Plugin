/*
	RackAFX(TM)
	Applications Programming Interface
	Derived Class Object Implementation
*/


#include "FinalProjectEMJ.h"


#define MIN_CHORUS_DELAY_MSEC 10.f
#define MAX_CHORUS_DELAY_MSEC 60.f

// --- these ARE standards, but there is no reason you couldn't mess around with them...
#define MIN_FLANGER_DELAY_MSEC 0.01f
#define MAX_FLANGER_DELAY_MSEC 7.f


/* constructor()
	You can initialize variables here.
	You can also allocate memory here as long is it does not
	require the plugin to be fully instantiated. If so, allocate in init()

*/
CFinalProjectEMJ::CFinalProjectEMJ()
{
	// Added by RackAFX - DO NOT REMOVE
	//
	// initUI() for GUI controls: this must be called before initializing/using any GUI variables
	initUI();
	// END initUI()

	// built in initialization
	m_PlugInName = "FinalProjectEMJ";

	// Default to Stereo Operation:
	// Change this if you want to support more/less channels
	m_uMaxInputChannels = 2;
	m_uMaxOutputChannels = 2;

	// use of MIDI controllers to adjust sliders/knobs
	m_bEnableMIDIControl = true;		// by default this is enabled

	// DO NOT CHANGE let RackAFX change it for you; use Edit Project to alter
	m_bUseCustomVSTGUI = false;

	// output only - SYNTH - plugin DO NOT CHANGE let RackAFX change it for you; use Edit Project to alter
	m_bOutputOnlyPlugIn = false;

	// change to true if you want all MIDI messages
	m_bWantAllMIDIMessages = false;

	// un-comment this for VST/AU Buffer-style processing
	// m_bWantVSTBuffers = true;

	// Finish initializations here
	m_pMidiEventList = NULL; // --- for sample accurate MIDI in VST, AU (AAX has its own method)

	// **** ENVELOPE
		// Finish initializations here
		// set our Min and Max Modulation points
		m_fMinCutoffFreq = 100.0;
		m_fMaxCutoffFreq = m_Fc;

	// **** DELAY
		// --- init for flanger
		minDelayMod_mSec = MIN_FLANGER_DELAY_MSEC;
		maxDelayMod_mSec = MAX_FLANGER_DELAY_MSEC;

}


/* destructor()
	Destroy dynamically allocated variables
*/
CFinalProjectEMJ::~CFinalProjectEMJ(void)
{


}

/*
initialize()
	Called by the client after creation; the parent window handle is now valid
	so you can use the Plug-In -> Host functions here
	See the website www.willpirkle.com for more details
*/
bool __stdcall CFinalProjectEMJ::initialize()
{
	// Add your code here

	// **** ENVELOPE
	m_LeftLPF.setFilterStructure(DF);
	m_RightLPF.setFilterStructure(DF);

	return true;
}

// **** ENVELOPE FILTER FUNCTIONS

	// for LPF
	float CFinalProjectEMJ::calculateCutoffFreq(float fEnvelopeSample)
	{
		// modulate from min upwards
		if (m_uDirection == UP)
			return fEnvelopeSample*(m_fMaxCutoffFreq - m_fMinCutoffFreq) + m_fMinCutoffFreq;
		else // modulate from max downwards
			return m_fMaxCutoffFreq - fEnvelopeSample*(m_fMaxCutoffFreq - m_fMinCutoffFreq);

		return m_fMinCutoffFreq;
	}

	// for HSF since the value of the HSF is controlled by gain
	float CFinalProjectEMJ::calculateGain(float fEnvelopeSample)
	{
		return fEnvelopeSample * (m_f_maxGain);
	}

	// LEFT LPF: Note same as Resonant LPF from Chapter 6
	void CFinalProjectEMJ::calculateLeftLPFCoeffs(float fCutoffFreq, float fQ)
	{
		// use same terms as book
		float theta_c = 2.0*pi*fCutoffFreq / (float)m_nSampleRate;
		float d = 1.0 / fQ;

		float fBetaNumerator = 1.0 - ((d / 2.0)*(sin(theta_c)));
		float fBetaDenominator = 1.0 + ((d / 2.0)*(sin(theta_c)));

		float fBeta = 0.5*(fBetaNumerator / fBetaDenominator);

		float fGamma = (0.5 + fBeta)*(cos(theta_c));

		float fAlpha = (0.5 + fBeta - fGamma) / 2.0;

		// left channel
		m_LeftLPF.m_f_a0 = fAlpha;
		m_LeftLPF.m_f_a1 = 2.0*fAlpha;
		m_LeftLPF.m_f_a2 = fAlpha;
		m_LeftLPF.m_f_b1 = -2.0*fGamma; // if b's are negative in the difference equation
		m_LeftLPF.m_f_b2 = 2.0*fBeta;
	}

	void CFinalProjectEMJ::calculateRightLPFCoeffs(float fCutoffFreq, float fQ)
	{
		float theta_c = 2.0*pi*fCutoffFreq / (float)m_nSampleRate;
		float d = 1.0 / fQ;

		float fBetaNumerator = 1.0 - ((d / 2.0)*(sin(theta_c)));
		float fBetaDenominator = 1.0 + ((d / 2.0)*(sin(theta_c)));

		float fBeta = 0.5*(fBetaNumerator / fBetaDenominator);

		float fGamma = (0.5 + fBeta)*(cos(theta_c));

		float fAlpha = (0.5 + fBeta - fGamma) / 2.0;

		// right channel
		m_RightLPF.m_f_a0 = fAlpha;
		m_RightLPF.m_f_a1 = 2.0*fAlpha;
		m_RightLPF.m_f_a2 = fAlpha;
		m_RightLPF.m_f_b1 = -2.0*fGamma; // if b's are negative in the difference equation
		m_RightLPF.m_f_b2 = 2.0*fBeta;
	}



// **** DELAY FILTER FUNCTIONS

	// generic biquad 2nd order HPF for delay feedback loop 
	void CFinalProjectEMJ::calculateHPFCoeffs(float fCutoffFreq, float fQ)
	{
		// first order HPF 
		//float theta_c = 2.0*pi*fCutoffFreq / (float)m_nSampleRate;
		//float gamma = 2.f + cos(theta_c);
		//m_f_b1 = gamma - sqrt((gamma*gamma) - 1);
		//m_f_a0 = 1 - m_f_b1;

		// 2nd order biquad
		float theta_c = 2.0*pi*fCutoffFreq / (float)m_nSampleRate;
		float d = 1.0 / fQ;
		// intermediate values
		float fBetaNumerator = 1.0 - ((d / 2.0)*(sin(theta_c)));
		float fBetaDenominator = 1.0 + ((d / 2.0)*(sin(theta_c)));
		// beta
		float fBeta = 0.5*(fBetaNumerator / fBetaDenominator);
		// gamma
		float fGamma = (0.5 + fBeta)*(cos(theta_c));
		// alpha
		float fAlpha = (0.5 + fBeta + fGamma) / 2.0;

		// left channel
		m_HPF.m_f_a0 = fAlpha;
		m_HPF.m_f_a1 = -(2.0*fAlpha);
		m_HPF.m_f_a2 = fAlpha;
		m_HPF.m_f_b1 = -2.0*fGamma;
		m_HPF.m_f_b2 = 2.0*fBeta;

	}

	// Linkwitz - Riley HPF for chorus (modulated signal)
	void CFinalProjectEMJ::calculateLinkwitzHigh(float fCutoffFreq, float fQ)
	{
		float theta_c = pi*fCutoffFreq / (float)m_nSampleRate;
		float omega_c = pi*fCutoffFreq;
		float k = omega_c / tan(theta_c);
		float delta = (k*k) + (omega_c*omega_c) + 2 * k*omega_c;

		m_LHPF.m_f_a0 = (k*k) / delta;
		m_LHPF.m_f_a1 = -2 * (k*k) / delta;
		m_LHPF.m_f_a2 = (k*k) / delta;
		m_LHPF.m_f_b1 = (-2 * (k*k) + 2 * (omega_c*omega_c)) / delta;
		m_LHPF.m_f_b2 = (-2 * k*omega_c + (k*k) + (omega_c*omega_c)) / delta;

	}

	// Linkwitz - Rilery LPF for chorus (dry signal)
	void CFinalProjectEMJ::calculateLinkwitzLow(float fCutoffFreq, float fQ)
	{
		float theta_c = pi*fCutoffFreq / (float)m_nSampleRate;
		float omega_c = pi*fCutoffFreq;
		float k = omega_c / tan(theta_c);
		float delta = (k*k) + (omega_c*omega_c) + 2 * k*omega_c;

		m_LLPF.m_f_a0 = (omega_c*omega_c) / delta;
		m_LLPF.m_f_a1 = 2 * ((omega_c*omega_c) / delta);
		m_LLPF.m_f_a2 = (omega_c*omega_c) / delta;
		m_LLPF.m_f_b1 = (-2 * (k*k) + 2 * (omega_c*omega_c)) / delta;
		m_LLPF.m_f_b2 = (-2 * k*omega_c + (k*k) + (omega_c*omega_c)) / delta;

	}


	// **** REVERB FUNCTION

	// function to cook all variables at once
	void CFinalProjectEMJ::cookVariables()
	{

		//m_PreDelay.setDelay_mSec(m_fPreDelay_mSec);
		//m_PreDelay.setOutputAttenuation_dB(m_fPreDelay_Atten_dB);

		// set sample rate for tank
		m_APF1.setSampleRate(m_nSampleRate);
		m_APF2.setSampleRate(m_nSampleRate);
		m_Delay3.setSampleRate(m_nSampleRate);
		m_APF4.setSampleRate(m_nSampleRate);
		m_APF5.setSampleRate(m_nSampleRate);
		m_Delay6.setSampleRate(m_nSampleRate);
		m_APF7.setSampleRate(m_nSampleRate);
		m_APF8.setSampleRate(m_nSampleRate);
		m_Delay9.setSampleRate(m_nSampleRate);
		m_APF10.setSampleRate(m_nSampleRate);
		m_APF11.setSampleRate(m_nSampleRate);
		m_Delay12.setSampleRate(m_nSampleRate);

		m_fAPF4_In.setSampleRate(m_nSampleRate);
		m_fAPF7_In.setSampleRate(m_nSampleRate);
		m_fAPF10_In.setSampleRate(m_nSampleRate);

		m_OutputDly1.setSampleRate(m_nSampleRate);

		// set delays first...
		// rn theyre all set to the same delay time, but can create knobs for each one 
		// APF 
		m_APF1.setDelay_mSec(m_f_APF1_Delay_mSec);
		m_APF2.setDelay_mSec(m_f_APF2_Delay_mSec);
		m_Delay3.setDelay_mSec(m_fDelay1_mSec); // sum
		m_APF4.setDelay_mSec(m_f_APF3_Delay_mSec);
		m_APF5.setDelay_mSec(m_f_APF4_Delay_mSec);
		m_Delay6.setDelay_mSec(m_fDelay2_mSec); // sum
		m_APF7.setDelay_mSec(m_f_APF5_Delay_mSec);
		m_APF8.setDelay_mSec(m_f_APF6_Delay_mSec);
		m_Delay9.setDelay_mSec(m_fDelay3_mSec); // sum
		m_APF10.setDelay_mSec(m_f_APF7_Delay_mSec);
		m_APF11.setDelay_mSec(m_f_APF8_Delay_mSec);
		m_Delay12.setDelay_mSec(m_fDelay4_mSec); // sum



												 // ...then calcualte APF g's from RT60:

		m_APF1.setAPF_g(0.5);
		m_APF2.setAPF_g(0.5);
		m_APF4.setAPF_g(0.5);
		m_APF5.setAPF_g(0.5);
		m_APF7.setAPF_g(0.5);
		m_APF8.setAPF_g(0.5);
		m_APF10.setAPF_g(0.5);
		m_APF11.setAPF_g(0.5);

		m_DampingLPF1.setLPF_g(m_fLPF_g);
		m_DampingLPF2.setLPF_g(m_fLPF_g);


	}


	// **** TREMOLO FUNCTIONS

	float CFinalProjectEMJ::calculateGainFactor(float fLFOSample)
	{
		// first multiply the value by depth/100
		float fOutput = fLFOSample*(m_fModDepth / 100.0);

		// then add the value (1 - m_fModDepth/100.0)
		fOutput += 1 - m_fModDepth / 100.0;

		return fOutput;
	}

	void CFinalProjectEMJ::calculatePannerGainFactor(float fLFOSample, float* pLeftVolume, float* pRightVolume)
	{
		// calc sin/cos quadrant location
		float fPan = ((m_fModDepth / 100.0)*fLFOSample + 1)*pi / 4.0;

		// equal power calculation
		*pLeftVolume = cos(fPan);
		*pRightVolume = sin(fPan);
	}

/*
processRackAFXMessage()
	Called for a variety of reasons, but we override here to pick up a MIDIEventList interface
	for sample accurate MIDI in VST/AU NOTE: this is only for use in processVSTBuffer()
	If you use processAudioFrame( ) you do not need to bother with this as you already
	have sample accurate MIDI in v6.8.0.5 and above
	See the website www.willpirkle.com for more details
*/
void __stdcall CFinalProjectEMJ::processRackAFXMessage(UINT uMessage, PROCESS_INFO& processInfo)
{
	// --- always call base class first
	CPlugIn::processRackAFXMessage(uMessage, processInfo);

	// --- for MIDI Event list handling (AU, VST, AAX with processVSTBuffer())
	if(uMessage == midiEventList)
	{
		m_pMidiEventList = processInfo.pIMidiEventList;
	}
}

/* prepareForPlay()
	Called by the client after Play() is initiated but before audio streams

	You can perform buffer flushes and per-run intializations.
	You can check the following variables and use them if needed:

	m_nNumWAVEChannels;
	m_nSampleRate;
	m_nBitDepth;

	NOTE: the above values are only valid during prepareForPlay() and
		  processAudioFrame() because the user might change to another wave file,
		  or use the sound card, oscillators, or impulse response mechanisms

    NOTE: if you alloctae memory in this function, destroy it in the destructor above
*/
bool __stdcall CFinalProjectEMJ::prepareForPlay()
{
	// Add your code here:

	// **** ENVELOPE
	if (m_uPluginType == Envelope)
	{

		m_LeftLPF.flushDelays();
		m_RightLPF.flushDelays();

		m_LeftDetector.init((float)m_nSampleRate, m_fAttack_mSec, m_fRelease_mSec, false, DETECT_MODE_RMS, false);
		m_RightDetector.init((float)m_nSampleRate, m_fAttack_mSec, m_fRelease_mSec, false, DETECT_MODE_RMS, false);

		// calculate the initial values

		calculateLeftLPFCoeffs(m_fMinCutoffFreq, m_fQ);
		calculateRightLPFCoeffs(m_fMinCutoffFreq, m_fQ);

		// added 
		m_LeftDetector.prepareForPlay();
		m_RightDetector.prepareForPlay();
	}



	// **** DELAY
	else if (m_uPluginType == Delay)
	{
		// --- init the LFO
		lfo.setSampleRate((float)m_nSampleRate);
		lfo.setOscFrequency(m_fRate); // --- init value
		lfo.setOscWaveform(m_uLFOType); // --- init value
		lfo.reset();

		// for HPF in fb loop 
		m_HPF.flushDelays();
		if (m_uDelayType_2 == delay)
		{
			calculateHPFCoeffs(m_f_HPF, 0.71);	// for delay fb loop 
		}

		// for Bass Chorus
		m_LLPF.flushDelays();
		m_LHPF.flushDelays();
		//calculateLinkwitzHigh(1000.f, m_fResonance_pct);		// 1k Hz is random frequency, couldnt get a knob to modulate it 
		//calculateLinkwitzLow(1000.f, m_fResonance_pct);		// resonance knob is the q 
		calculateLinkwitzHigh(m_f_Fc_Hz, m_fResonance_pct);
		calculateLinkwitzLow(m_f_Fc_Hz, m_fResonance_pct);


		// --- init the delay lines for 300mSec max delay - note this is sample-rate-dependent
		//     see the function - this will also flush the delay line
		m_LeftDelay.init(2 * m_nSampleRate);
		m_RightDelay.init(2 * m_nSampleRate);

		// --- set the sample rate
		m_LeftDelay.setSampleRate(m_nSampleRate);
		m_RightDelay.setSampleRate(m_nSampleRate);
	}



	// **** REVERB
	else if (m_uPluginType == Reverb)
	{
		m_PreDelay.init(2.f*m_nSampleRate);

		// 100 mSec each max
		m_APF1.init(0.1*(m_nSampleRate));
		m_APF2.init(0.1*(m_nSampleRate));
		m_Delay3.init(0.1*(m_nSampleRate)); // sum
		m_APF4.init(0.1*(m_nSampleRate));
		m_APF5.init(0.1*(m_nSampleRate));
		m_Delay6.init(0.1*(m_nSampleRate)); // sum
		m_APF7.init(0.1*(m_nSampleRate));
		m_APF8.init(0.1*(m_nSampleRate));
		m_Delay9.init(0.1*(m_nSampleRate)); // sum
		m_APF10.init(0.1*(m_nSampleRate));
		m_APF11.init(0.1*(m_nSampleRate));
		m_Delay12.init(0.1*(m_nSampleRate)); // sum

		m_OutputDly1.init(0.1*(m_nSampleRate));

		m_DampingLPF1.init();
		m_DampingLPF2.init();

		// reset delays
		m_APF1.resetDelay();
		m_APF2.resetDelay();
		m_Delay3.resetDelay(); // sum
		m_APF4.resetDelay();
		m_APF5.resetDelay();
		m_Delay6.resetDelay(); // sum
		m_APF7.resetDelay();
		m_APF8.resetDelay();
		m_Delay9.resetDelay(); // sum
		m_APF10.resetDelay();
		m_APF11.resetDelay();
		m_Delay12.resetDelay(); // sum

		m_OutputDly1.resetDelay();

		m_fAPF4_In.resetDelay();
		m_fAPF7_In.resetDelay();
		m_fAPF10_In.resetDelay();

		m_DampingLPF1.init();
		m_DampingLPF2.init();

		// cook everything
		cookVariables();

	}


	
	// **** TREMOLO
	else if (m_uPluginType == Trem)
	{
		m_LFO.m_fFrequency_Hz = m_fModRate;
		m_LFO.m_uPolarity = m_uMode == Panner ? 0 : 1;  // 0 = bipolar, 1 = unipolar
		m_LFO.m_uTableMode = 0; // normal, no band limiting
		m_LFO.m_uOscType = m_uLFOType;
		m_LFO.setSampleRate(m_nSampleRate); // really important!

											// the LFO prepareForPlay() calls reset() and cookFrequency()
		m_LFO.prepareForPlay();

	}

	// --- let base class do its thing
	return CPlugIn::prepareForPlay();
}



float CFinalProjectEMJ::calculateModDelayTime_mSec(float modValue)
{
	// flanger mod delay time 
	//return modValue*(maxDelayMod_mSec - minDelayMod_mSec) + minDelayMod_mSec;

	// --- flanger/vibrato
	if (m_uDelayType_2 == flanger)
		return modValue*(maxDelayMod_mSec - minDelayMod_mSec) + minDelayMod_mSec;

	// --- otherwise this is chorus, rotates around central point, halfway between min and max delay time
	//     must use bipolar LFO for chorus!
	float range = (MAX_CHORUS_DELAY_MSEC - MIN_CHORUS_DELAY_MSEC) / 2.f;
	float midpoint = range + MIN_CHORUS_DELAY_MSEC;

	return modValue*(range)+midpoint;
}

// --- Delay Sequence:
//     1) read delay line (for multi-tap, read multiple times)
//     2) form the feedback values
//     3) write input to delay line
//     4) create wet + dry output value
//

// normal delay w/ HPF in fb loop 
float CFinalProjectEMJ::doNormalDelay(float xn, UINT channel)
{
	// --- 1) read delay decode channel 
	float delay = channel == LEFT ? m_LeftDelay.readDelay_mSec(m_fDelay_mSec) : m_RightDelay.readDelay_mSec(m_fDelay_mSec);

	// --- 2) form feedback = input + fb*delay
	// Original
	//float delayInput = xn + (m_fResonance_pct / 100.f)*delay;

	float delayInput = xn + (m_fResonance_pct / 100.f)*m_HPF.doBiQuad(delay);

	// --- 3) write input to delay line
	if (channel == LEFT)
		m_LeftDelay.writeDelay(delayInput);
	else
		m_RightDelay.writeDelay(delayInput);

	// --- 4) create output
	float mix = m_fDelayMix_pct / 100.f;

	// --- wet + dry
	//float output = mix*delay + (1.f - mix)*xn;
	float output = mix*m_HPF.doBiQuad(delay) + (1.f - mix)*xn;

	return output;
}


// for mod delay section, needed th eadded variables 
float CFinalProjectEMJ::doModDelay(float xn, float delay_mSec, float feedback, UINT channel)
{
	// --- 1) read delay decode channel
	float delay = channel == LEFT ? m_LeftDelay.readDelay_mSec(delay_mSec) : m_RightDelay.readDelay_mSec(delay_mSec);

	// --- 2) form feedback = input + fb*delay
	float delayInput = xn + (feedback / 100.f)*delay;

	// --- 3) write input to delay line
	if (channel == LEFT)
		m_LeftDelay.writeDelay(delayInput);
	else
		m_RightDelay.writeDelay(delayInput);

	return delay;
}






/* processAudioFrame

// ALL VALUES IN AND OUT ON THE RANGE OF -1.0 TO + 1.0

LEFT INPUT = pInputBuffer[0];
RIGHT INPUT = pInputBuffer[1]

LEFT OUTPUT = pInputBuffer[0]
RIGHT OUTPUT = pOutputBuffer[1]

HOST INFORMATION is available in m_HostProcessInfo:

// --- for RackAFX and all derivative projects:
m_HostProcessInfo.uAbsoluteSampleBufferIndex = sample index of top of current audio buffer
m_HostProcessInfo.dAbsoluteSampleBufferTime = time (sec) of sample in top of current audio buffer
m_HostProcessInfo.dBPM = Host Tempo setting in BPM
m_HostProcessInfo.fTimeSigNumerator = Host Time Signature Numerator (if supported by host)
m_HostProcessInfo.uTimeSigDenomintor = Host Time Signature Denominator (if supported by host)

// --- see the definition of HOST_INFO in the pluginconstants.h file for variables that are
//     unique to AU, AAX and VST for use in your ported projects!
*/
bool __stdcall CFinalProjectEMJ::processAudioFrame(float* pInputBuffer, float* pOutputBuffer, UINT uNumInputChannels, UINT uNumOutputChannels)
{
	// --- for VST3 plugins only
	doVSTSampleAccurateParamUpdates();

	// --- smooth parameters (if enabled) DO NOT REMOVE
	smoothParameterValues();

	// **** ENVELOPE
	if (m_uPluginType == Envelope)
	{
		// set biquad structure
		if (m_uFilterStructure == DF)
		{
			m_LeftLPF.setFilterStructure(m_uFilterStructure);
			m_RightLPF.setFilterStructure(m_uFilterStructure);
		}
		else if (m_uFilterStructure == TDF)
		{
			m_LeftLPF.setFilterStructure(m_uFilterStructure);
			m_RightLPF.setFilterStructure(m_uFilterStructure);
		}
		else if (m_uFilterStructure == CF)
		{
			m_LeftLPF.setFilterStructure(m_uFilterStructure);
			m_RightLPF.setFilterStructure(m_uFilterStructure);
		}
		else if (m_uFilterStructure == TCF)
		{
			m_LeftLPF.setFilterStructure(m_uFilterStructure);
			m_RightLPF.setFilterStructure(m_uFilterStructure);
		}

		//
		// Do LEFT (MONO) Channel; there is always at least one input, one output
		float fGain = pow(10, m_fPreGain_dB / 20.0);
		float fDetectLeft = m_LeftDetector.detect(fGain*pInputBuffer[0]);

		// set mod freq to minimum (un-triggered)
		float fModFreqLeft = m_fMinCutoffFreq;
		float fModGainLeft = m_fMinCutoffFreq;

		// if threshold triggered, calculate new LPF cutoff
		if (fDetectLeft >= m_fThreshold)
			{
				fModFreqLeft = calculateCutoffFreq(fDetectLeft - m_fThreshold);
			}
		else
		{
			fModGainLeft = calculateGain(fDetectLeft - m_fThreshold);
		}

		// use the mod freq and user supplied-Q to calc the fc

			calculateLeftLPFCoeffs(fModFreqLeft, m_fQ);
			// do the BiQuads
			pOutputBuffer[0] = m_f_peakGain * m_LeftLPF.doBiQuad(pInputBuffer[0]);
		


		// detect the other channel
		float fDetectRight = m_RightDetector.detect(fGain*pInputBuffer[1]);

		// set mod freq to minimum (un-triggered)
		float fModFreqRight = m_fMinCutoffFreq;
		float fModGainRight = m_fMinCutoffFreq;

		// if threshold triggered, calculate new LPF cutoff
		if (fDetectRight >= m_fThreshold)
			{
				fModFreqRight = calculateCutoffFreq(fDetectRight - m_fThreshold);
			}
			else
			{
				fModGainRight = calculateGain(fDetectRight - m_fThreshold);
			}

		// use the mod freq and user supplied-Q to calc the fc
		calculateRightLPFCoeffs(fModFreqRight, m_fQ);

		// Mono-In, Stereo-Out (AUX Effect)
		if (uNumInputChannels == 1 && uNumOutputChannels == 2)
			pOutputBuffer[1] = pOutputBuffer[0];



		// Stereo-In, Stereo-Out (INSERT Effect)
		if (uNumInputChannels == 2 && uNumOutputChannels == 2)
		{
				// do the BiQuads
				pOutputBuffer[1] = m_f_peakGain * m_RightLPF.doBiQuad(pInputBuffer[1]);
		}
	}


	// **** DELAY
	else if (m_uPluginType == Delay)
	{
		/* --- Mod Delay Sequence:
		1) create the modulator value (here from LFO)
		2) calculate the current delay value
		3) read the delay value, note feedback value varies
		4) mix wet + dry according to algorithm
		*/

		// --- create modulator value
		float lfoOut = 0.f;
		float lfoQPOut = 0.f;

		// --- LFO, output is bipolar
		lfo.doOscillate(lfoOut, lfoQPOut);
		float ModOutput = 0.f;

		//lfoOut = bipolarToUnipolar(lfoOut);
		//lfoQPOut = bipolarToUnipolar(lfoQPOut);
		// --- Flanger/Vibrato: unipolar modulator
		if (m_uDelayType_2 == flanger)
		{
			lfoOut = bipolarToUnipolar(lfoOut);
			lfoQPOut = bipolarToUnipolar(lfoQPOut);
		}



		// --- calculate the mod value, left channel always uses lfoOut
		float modDelay_mSec = calculateModDelayTime_mSec(lfoOut);

		// --- setup feedback: what happens if you let the chorus have feedback?
		//float feedback = 0.f;
		//feedback = m_fResonance_pct;
		float feedback = m_uDelayType_2 == flanger ? m_fResonance_pct : 0.f;

		// --- get the delayed value
		float delayLeft = doModDelay(pInputBuffer[0], (m_fDepth_pct / 100.f)*modDelay_mSec, feedback, LEFT);

		//for both delay and chorus 
		ModOutput = 0.5*pInputBuffer[0] + 0.5*delayLeft;


		// output = input -- change this for meaningful processing
		//
		// Do LEFT (MONO) Channel; there is always at least one input/one output
		// (INSERT Effect)

		float xn = pInputBuffer[0];

		// ********* BASS CHORUS *********
		// split signal into high and low passed 
		float LowPassedAudio = m_LLPF.doBiQuad(xn);
		float HighPassedAudio = m_LHPF.doBiQuad(xn);

		// use just high passed audio to do chorus effect
		float HPF_Output = 0.5*HighPassedAudio + 0.5*delayLeft;

		// send output high passed audio back into feedback
		float ChorusOutput = (m_fWet_pct_modDelay / 100)*HPF_Output + (1 - m_fWet_pct_modDelay / 100) *LowPassedAudio;



		if (m_uDelayType_2 == delay)
		{
			pOutputBuffer[0] = doNormalDelay(xn, LEFT);
		}
		else if (m_uDelayType_2 == flanger)
		{
			pOutputBuffer[0] = ModOutput;
		}
		else if (m_uDelayType_2 == basschorus)
		{
			//pOutputBuffer[0] = ModOutput;		// normal chorus 
			//pOutputBuffer[0] = pInputBuffer[0];	// untouched output
			pOutputBuffer[0] = ChorusOutput;
		}

		// Mono-In, Stereo-Out (AUX Effect)
		if (uNumInputChannels == 1 && uNumOutputChannels == 2)
			pOutputBuffer[1] = pInputBuffer[0];

		// Stereo-In, Stereo-Out (INSERT Effect)
		if (uNumInputChannels == 2 && uNumOutputChannels == 2)
		{
			float xn = pInputBuffer[1];

			// --- calculate the mod value, left channel always uses lfoOut
			float modDelay_mSec = calculateModDelayTime_mSec(lfoOut);

			// --- setup feedback: what happens if you let the chorus have feedback?
			//float feedback = 0.f;
			//feedback = m_fResonance_pct;
			float feedback = m_uDelayType_2 == flanger ? m_fResonance_pct : 0.f;

			// --- get the delayed value
			float delayRight = doModDelay(pInputBuffer[0], (m_fDepth_pct / 100.f)*modDelay_mSec, feedback, RIGHT);

			ModOutput = 0.5*pInputBuffer[1] + 0.5*delayRight;


			// ********* BASS CHORUS *********
			// split signal into high and low passed 
			float LowPassedAudio = m_LLPF.doBiQuad(xn);
			float HighPassedAudio = m_LHPF.doBiQuad(xn);

			// use just high passed audio to do chorus effect
			float HPF_Output = 0.5*HighPassedAudio + 0.5*delayRight;

			// send output high passed audio back into feedback
			float ChorusOutput = (m_fWet_pct_modDelay / 100)*HPF_Output + (1 - m_fWet_pct_modDelay / 100) *LowPassedAudio;


			if (m_uDelayType_2 == delay)
			{
				pOutputBuffer[1] = doNormalDelay(xn, RIGHT);
			}
			else if (m_uDelayType_2 == flanger)
			{
				pOutputBuffer[1] = ModOutput;
			}
			else if (m_uDelayType_2 == basschorus)
			{
				//pOutputBuffer[1] = ModOutput;		// normal chorus 
				//pOutputBuffer[1] = pInputBuffer[1];	// untouched output
				pOutputBuffer[1] = ChorusOutput;
			}
		}

	}


	// **** REVERB
	else if (m_uPluginType == Reverb)
	{
		//
		// Form our input = L + R (if there is a R)
		//
		float fInputSample = pInputBuffer[0];
		if (uNumInputChannels == 2)
		{
			// mix
			fInputSample += pInputBuffer[1];
			// attenuate by 0.5
			fInputSample *= 0.5;
		}


		// begin series signal push

		float fFeedback = 0.f;

		float fInput1 = fInputSample + fFeedback;

		// LINE 1
		float fAPF1_Out = 0.f;
		m_APF1.processAudio(&fInput1, &fAPF1_Out);

		float fAPF2_Out = 0.f;
		m_APF2.processAudio(&fAPF1_Out, &fAPF2_Out);

		float fDelay3_Out = 0.f;
		m_Delay3.processAudio(&fAPF2_Out, &fDelay3_Out);
		// output
		float fDelay3_LeftOut = m_Delay3.readDelayAt(0.4*m_fDelay1_mSec);
		float fDelay3_RightOut = m_Delay3.readDelayAt(0.6*m_fDelay1_mSec);
		float fkrt_1 = m_fKrt_1 * fDelay3_Out;


		// LINE 2
		float fInput2 = fInputSample + fkrt_1;
		float fAPF4_Out = 0.f;
		m_APF4.processAudio(&fInput2, &fAPF4_Out);

		float fAPF5_Out = 0.f;
		m_APF5.processAudio(&fAPF4_Out, &fAPF5_Out);

		float fDelay6_Out = 0.f;
		m_Delay6.processAudio(&fAPF5_Out, &fDelay6_Out);
		// output
		float fDelay6_LeftOut = m_Delay6.readDelayAt(0.4*m_fDelay2_mSec);
		float fDelay6_RightOut = m_Delay6.readDelayAt(0.6*m_fDelay2_mSec);
		float fkrt_2 = m_fKrt_1 * fDelay6_Out;

		// LINE 3
		float fInput3 = fInputSample + fkrt_2;
		float fAPF7_Out = 0.f;
		m_APF7.processAudio(&fInput3, &fAPF7_Out);

		float fAPF8_Out = 0.f;
		m_APF8.processAudio(&fAPF7_Out, &fAPF8_Out);

		float fDelay9_Out = 0.f;
		m_Delay9.processAudio(&fAPF8_Out, &fDelay9_Out);
		// output
		float fDelay9_LeftOut = m_Delay9.readDelayAt(0.4*m_fDelay3_mSec);
		float fDelay9_RightOut = m_Delay9.readDelayAt(0.6*m_fDelay3_mSec);
		float fkrt_3 = m_fKrt_1 * fDelay9_Out;

		// LINE 4
		float fInput4 = fInputSample + fkrt_3;
		float fAPF10_Out = 0.f;
		m_APF10.processAudio(&fInput4, &fAPF10_Out);

		float fAPF11_Out = 0.f;
		m_APF11.processAudio(&fAPF10_Out, &fAPF11_Out);

		float fDelay12_Out = 0.f;
		m_Delay12.processAudio(&fAPF11_Out, &fDelay12_Out);
		//output
		float fDelay12_LeftOut = m_Delay12.readDelayAt(0.4*m_fDelay4_mSec);
		float fDelay12_RightOut = m_Delay12.readDelayAt(0.6*m_fDelay4_mSec);
		fFeedback = m_fKrt_1*fDelay12_Out;


		//float fDelay_LeftSum = 0.25*fDelay3_LeftOut + 0.25*fDelay6_LeftOut + 0.25*fDelay9_LeftOut + 0.25*fDelay12_LeftOut;
		//float fDelay_RightSum = 0.25*fDelay3_RightOut + 0.25*fDelay6_RightOut + 0.25*fDelay9_RightOut + 0.25*fDelay12_RightOut;

		float fDelay_LeftSum = 0.25*fInput2 + 0.25*fInput3 + 0.25*fInput4 + 0.25*fFeedback;

		// ADDING 2 DAMPING LPF AFTER VERB TANK
		float fDamping_LPF_1_Out = 0;
		m_DampingLPF1.processAudio(&fDelay_LeftSum, &fDamping_LPF_1_Out);

		float fDamping_LPF_2_Out = 0;
		m_DampingLPF2.processAudio(&fDamping_LPF_1_Out, &fDamping_LPF_2_Out);


		// form output = (100-Wet)/100*x(n) + (Wet/100)*fAPF_3_Out
		pOutputBuffer[0] = ((100.0 - m_fWet_pct_reverb) / 100.0)*fInputSample +
			(m_fWet_pct_reverb / 100.0)*(fDamping_LPF_2_Out);
		//(m_fWet_pct / 100.0)*(fDelay_LeftSum);

		// Do RIGHT Channel if there is one
		if (uNumOutputChannels == 2)
		{
			// form output = (100-Wet)/100*x(n) + (Wet/100)*fAPF_4_Out
			pOutputBuffer[1] = ((100.0 - m_fWet_pct_reverb) / 100.0)*fInputSample +
				(m_fWet_pct_reverb / 100.0)*(fDamping_LPF_2_Out);
			//(m_fWet_pct / 100.0)*(fDelay_LeftSum);
		}

	}



	// **** TREMOLO
	else if (m_uPluginType == Trem)
	{
		// Do LEFT (MONO) Channel; there is always at least one input/one output
		float fYn = 0; // normal output
		float fYqn = 0; // quad phase output

						// call the LFO function; we only need first output
		m_LFO.doOscillate(&fYn, &fYqn);

		// setup necessary variables
		float fGnL = 1.0;
		float fGnR = 1.0;
		float fMonoIn = 0.0;

		// branch
		if (m_uMode == Tremolo)
			fGnL = calculateGainFactor(fYn);
		else // panner sums inputs
		{
			if (uNumInputChannels == 1)
				fMonoIn = pInputBuffer[0];
			else
				fMonoIn = 0.5*(pInputBuffer[0] + pInputBuffer[1]);

			calculatePannerGainFactor(fYn, &fGnL, &fGnR);
		}

		// do MONO (Left) channel
		pOutputBuffer[0] = pInputBuffer[0] * fGnL;

		// Mono-In, Stereo-Out (AUX Effect)
		if (uNumInputChannels == 1 && uNumOutputChannels == 2)
			pOutputBuffer[1] = pOutputBuffer[0]; // just copy

												 // Stereo-In, Stereo-Out (INSERT Effect)
		if (uNumInputChannels == 2 && uNumOutputChannels == 2)
		{
			// branch
			if (m_uMode == Tremolo)
			{
				// do right channel, use same gain as left
				pOutputBuffer[1] = pInputBuffer[1] * fGnL;
			}
			else
			{
				// do right channel, its value
				pOutputBuffer[1] = pInputBuffer[1] * fGnR;
			}
		}
	}


	return true;
}


/* ADDED BY RACKAFX -- DO NOT EDIT THIS CODE!!! ----------------------------------- //
   	**--0x2983--**

UIList Index	Variable Name					Control Index		
-------------------------------------------------------------------
0				m_fPreGain_dB                     0
1				m_fThreshold                      1
2				m_fAttack_mSec                    2
3				m_fRelease_mSec                   3
4				m_fQ                              4
5				m_Fc                              5
6				m_f_maxGain                       6
7				m_fPEQ_Fc                         7
8				m_fPEQ_Q                          8
9				m_fPEQ_Gain_dB                    9
10				m_fDelay_mSec                     20
11				m_fFeedback_pct                   21
12				m_fDelayMix_pct                   22
13				m_f_HPF                           23
14				m_fRate                           25
15				m_fDepth_pct                      26
16				m_fResonance_pct                  27
17				m_fWet_pct_modDelay               28
18				m_f_Fc_Hz                         29
19				m_fWet_pct_reverb                 100
20				m_f_APF1_Delay_mSec               101
21				m_f_APF2_Delay_mSec               102
22				m_f_APF3_Delay_mSec               103
23				m_f_APF4_Delay_mSec               104
24				m_f_APF5_Delay_mSec               105
25				m_f_APF6_Delay_mSec               106
26				m_f_APF7_Delay_mSec               107
27				m_f_APF8_Delay_mSec               108
28				m_fDelay1_mSec                    111
29				m_fDelay2_mSec                    112
30				m_fDelay3_mSec                    113
31				m_fDelay4_mSec                    114
32				m_fKrt_1                          116
33				m_fLPF_g                          117
34				m_fModRate                        130
35				m_fModDepth                       131
36				m_uFilterStructure                43
37				m_f_GC                            45
38				m_uCoupleQ                        46
39				m_uDelayType_2                    44
40				m_uDirection                      42
41				m_uPluginType                     41
42				m_uLFOType                        2048
43				m_uMode                           2049

	Assignable Buttons               Index
-----------------------------------------------
	B1                                50
	B2                                51
	B3                                52

-----------------------------------------------
	Joystick List Boxes (Classic)    Index
-----------------------------------------------
	Drop List A                       60
	Drop List B                       61
	Drop List C                       62
	Drop List D                       63

-----------------------------------------------

	**--0xFFDD--**
// ------------------------------------------------------------------------------- */
// Add your UI Handler code here ------------------------------------------------- //
//
bool __stdcall CFinalProjectEMJ::userInterfaceChange(int nControlIndex)
{
	// decode the control index, or delete the switch and use brute force calls
	switch(nControlIndex)
	{
	
	// **** ENVELOPE
	case 2: // Attack
	{
		m_LeftDetector.setAttackTime(m_fAttack_mSec);
		m_RightDetector.setAttackTime(m_fAttack_mSec);
		break;
	}

	case 3: // Release
	{
		m_LeftDetector.setReleaseTime(m_fRelease_mSec);
		m_RightDetector.setReleaseTime(m_fRelease_mSec);
		break;
	}

	case 4: // Q
	{
		// update coeffs for next time
			calculateLeftLPFCoeffs(m_fMinCutoffFreq, m_fQ);
			calculateRightLPFCoeffs(m_fMinCutoffFreq, m_fQ);

			// Gain Compensation
			if (m_f_GC == SWITCH_ON)
				if (m_fQ > 0.707)	// if coefficient is greater than 1dB
				{
					float m_f_peakdB = 20 * log10((m_fQ*m_fQ) / (sqrt((m_fQ*m_fQ) - 0.25)));		// float gain in dB
					m_f_peakGain = pow(10, (-m_f_peakdB / 2) / 20);
				}
				else
				{
					m_f_peakGain = 1.0;
				}
			else
			{
				m_f_peakGain = 1.0;
			}

			// CoupleQ
			if (m_uCoupleQ == SWITCH_ON)
			{
				// convert Hz to KiloHz so Q can read it in range of 0-20
				float kilo = pow(10, -3);
				float m_Fc_KHz = (m_Fc * kilo);
				if (m_Fc_KHz > 0)
				{
					m_fQ = m_Fc_KHz;
				}
			}
			else
			{
				return m_fQ;
			}
			break;
	}

	case 7: // PEQ Fc
	case 8: // PEQ Q
	case 9: // PEQ dB Gain
	{
		m_LeftDetector.setPEQParameters(m_fPEQ_Fc, m_fPEQ_Q, m_fPEQ_Gain_dB);
		m_RightDetector.setPEQParameters(m_fPEQ_Fc, m_fPEQ_Q, m_fPEQ_Gain_dB);

		break;
	}


	// **** DELAY 

	case 20:
	{
		break;
	}

	case 23:  // HPF
	{
		calculateHPFCoeffs(m_f_HPF, 0.71); // defaulting to no Q, can add a Q knob if i want 
	}

	case 25:  // rate
	{
		lfo.setOscFrequency(m_fRate);
	}

	case 29:  // bass cutoff
	{
		calculateLinkwitzHigh(m_f_Fc_Hz, m_fResonance_pct);
		calculateLinkwitzLow(m_f_Fc_Hz, m_fResonance_pct);
	}

	case 2048: // lfo 
	{
		lfo.setOscWaveform(m_uLFOType);
	}

	case 44:  // delay type
	{
		if (m_uDelayType_2 == flanger)
		{
			minDelayMod_mSec = MIN_FLANGER_DELAY_MSEC;
			maxDelayMod_mSec = MAX_FLANGER_DELAY_MSEC;
		}
		else // chorus
		{
			minDelayMod_mSec = MIN_CHORUS_DELAY_MSEC;
			maxDelayMod_mSec = MAX_CHORUS_DELAY_MSEC;
		}
		break;
	}


	// **** REVERB

	cookVariables();

	// **** TREMOLO
	case 70:  // rate
	{
		m_LFO.m_fFrequency_Hz = m_fModRate;
		m_LFO.cookFrequency();
		break;
	}

	case 2049:	// tremolo or panner
	{
		if (m_uMode == Panner)
			m_LFO.m_uPolarity = 0;  // 0 = bipolar, 1 = unipolar
		else // is tremolo
			m_LFO.m_uPolarity = 1;  // 0 = bipolar, 1 = unipolar
		break;
	}

		default:
			break;
	}

	return true;
}








// --- message for updating GUI from plugin; see the comment block above userInterfaceChange( ) for
//     the index values to use when sending outbound parameter changes
//     see www.willpirkle.com for information on using this function to update the GUI from your plugin
//     The bLoadingPreset flag will be set if this is being called as a result of a preset load; you may
//     want to ignore GUI updates for presets depending on how your update works!
//     NOTE: this function will be called even if no audio is flowing (unlike userInterfaceChange( ) which
//           will only get called while in the audio processing loop (see Thread Safety document on website)
bool __stdcall CFinalProjectEMJ::checkUpdateGUI(int nControlIndex, float fValue, CLinkedList<GUI_PARAMETER>& guiParameters, bool bLoadingPreset)
{
	// decode the control index
	switch(nControlIndex)
	{
		case 0:
		{
			// return true; // if update needed
			// break;		// if no update needed
		}

		default:
			break;
	}
	return false;
}

// --- process aux inputs
//     This function will be called once for each Aux Input bus, currently:
//
//     Aux Input 1: Sidechain
//     May add more input busses in the future
//
//     see www.willpirkle.com for info on using the Aux input bus
bool __stdcall CFinalProjectEMJ::processAuxInputBus(audioProcessData* pAudioProcessData)
{
	/* --- pick up pointers to the Aux Input busses for sidechain or other Aux processing

	     	Ordinarily, you just copy the buffer pointers to member variables and then
	    	use the pointers in your process( ) function.

	     	However, you also have the option of pre-processing the Aux inputs in-place
	     	in these buffers, though no idea when you might need that...

	Example:
	if(pAudioProcessData->uInputBus == 1)
	{
		// --- save varius pointers, in practice you only save the
		//     pointer you need for your particular process( ) function
		//     Note these are member variables you need to declare on your own in the .h file!
		//
		m_pSidechainFrameBuffer = pAudioProcessData->pFrameInputBuffer; // <-- for processAudioFrame( )
		m_pSidechainRAFXBuffer = pAudioProcessData->pRAFXInputBuffer;	// <-- for processRackAFXAudioBuffer
		m_ppSidechainVSTBuffer = pAudioProcessData->ppVSTInputBuffer;	// <-- for processVSTAudioBuffer

		// --- sidechain input activation/channels
		m_bSidechainEnabled = pAudioProcessData->bInputEnabled;
		m_uSidechainChannelCount = pAudioProcessData->uNumInputChannels;
	}

	*/

	return true;
}

/* joystickControlChange

	Indicates the user moved the joystick point; the variables are the relative mixes
	of each axis; the values will add up to 1.0

			B
			|
		A -	x -	C
			|
			D

	The point in the very center (x) would be:
	fControlA = 0.25
	fControlB = 0.25
	fControlC = 0.25
	fControlD = 0.25

	AC Mix = projection on X Axis (0 -> 1)
	BD Mix = projection on Y Axis (0 -> 1)
*/
bool __stdcall CFinalProjectEMJ::joystickControlChange(float fControlA, float fControlB, float fControlC, float fControlD, float fACMix, float fBDMix)
{
	// add your code here

	return true;
}



/* processAudioBuffer

	// ALL VALUES IN AND OUT ON THE RANGE OF -1.0 TO + 1.0

	The I/O buffers are interleaved depending on the number of channels. If uNumChannels = 2, then the
	buffer is L/R/L/R/L/R etc...

	if uNumChannels = 6 then the buffer is L/R/C/Sub/BL/BR etc...

	It is up to you to decode and de-interleave the data.

	To use this function set m_bWantBuffers = true in your constructor.

	******************************
	********* IMPORTANT! *********
	******************************
	If you are going to ultimately use <Make VST> or <Make AU> to port your project and you want to process
	buffers instead of frames, you need to override processVSTAudioBuffer() below instead;

	processRackAFXAudioBuffer() is NOT supported in <Make VST>, <Make AU>, and <Make AAX>

	HOST INFORMATION is available in m_HostProcessInfo:

	m_HostProcessInfo.uAbsoluteSampleBufferIndex = sample index of top of current audio buffer
	m_HostProcessInfo.dAbsoluteSampleBufferTime = time (sec) of sample in top of current audio buffer
	m_HostProcessInfo.dBPM = Host Tempo setting in BPM
	m_HostProcessInfo.fTimeSigNumerator = Host Time Signature Numerator (if supported by host)
	m_HostProcessInfo.uTimeSigDenomintor = Host Time Signature Denominator (if supported by host)
*/
bool __stdcall CFinalProjectEMJ::processRackAFXAudioBuffer(float* pInputBuffer, float* pOutputBuffer,
													   UINT uNumInputChannels, UINT uNumOutputChannels,
													   UINT uBufferSize)
{

	for(UINT i=0; i<uBufferSize; i++)
	{
		// smooth parameters (if enabled) DO NOT REMOVE
		smoothParameterValues(); // done on a per-sample-interval basis

		// pass through code
		pOutputBuffer[i] = pInputBuffer[i];
	}


	return true;
}



/* processVSTAudioBuffer

	// ALL VALUES IN AND OUT ON THE RANGE OF -1.0 TO + 1.0

	NOTE: You do not have to implement this function if you don't want to; the processAudioFrame()
	will still work; however this using function will be more CPU efficient for your plug-in, and will
	override processAudioFrame().

	To use this function set m_bWantVSTBuffers = true in your constructor.

	The VST input and output buffers are pointers-to-pointers. The pp buffers are the same depth as uNumChannels, so
	if uNumChannels = 2, then ppInputs would contain two pointers,

		inBuffer[0] = a pointer to the LEFT buffer of data
		inBuffer[1] = a pointer to the RIGHT buffer of data

	Similarly, outBuffer would have 2 pointers, one for left and one for right.

	For 5.1 audio you would get 6 pointers in each buffer.

*/
bool __stdcall CFinalProjectEMJ::processVSTAudioBuffer(float** inBuffer, float** outBuffer, UINT uNumChannels, int inFramesToProcess)
{
	// PASS Through example
	// MONO First
	float* pInputL  = inBuffer[0];
	float* pOutputL = outBuffer[0];
	float* pInputR  = NULL;
	float* pOutputR = NULL;

	// if STEREO,
	if(inBuffer[1])
		pInputR = inBuffer[1];

	if(outBuffer[1])
		pOutputR = outBuffer[1];

	// Process audio by de-referencing ptrs
	// this is siple pass through code
	unsigned int uSample = 0;
	while (--inFramesToProcess >= 0)
	{
		// --- fire midi events (AU, VST2, AAX buffer processing only; not needed if you use processAudioFrame())
		if (m_pMidiEventList)
			m_pMidiEventList->fireMidiEvent(uSample++);

		// --- sample accurate automation for VST3 only
		doVSTSampleAccurateParamUpdates();

		// --- smooth parameters (if enabled) DO NOT REMOVE
		smoothParameterValues(); // done on a per-sample-interval basis

		// --- Left channel processing
		*pOutputL = *pInputL;

		// --- If there is a right channel
		if(pInputR && pOutputR)
			*pOutputR = *pInputR;
		else if(pOutputR) // 1->2 mapping
			*pOutputR = *pOutputL;

		// --- advance pointers
		pInputL++;
		pOutputL++;
		if(pInputR) pInputR++;
		if(pOutputR) pOutputR++;
	}
	// --- all OK
	return true;
}

bool __stdcall CFinalProjectEMJ::midiNoteOn(UINT uChannel, UINT uMIDINote, UINT uVelocity)
{
	return true;
}

bool __stdcall CFinalProjectEMJ::midiNoteOff(UINT uChannel, UINT uMIDINote, UINT uVelocity, bool bAllNotesOff)
{
	return true;
}

// uModValue = 0->127
bool __stdcall CFinalProjectEMJ::midiModWheel(UINT uChannel, UINT uModValue)
{
	return true;
}

// nActualPitchBendValue 		= -8192 -> +8191, 0 at center
// fNormalizedPitchBendValue 	= -1.0  -> +1.0,  0 at center
bool __stdcall CFinalProjectEMJ::midiPitchBend(UINT uChannel, int nActualPitchBendValue, float fNormalizedPitchBendValue)
{
	return true;
}

// MIDI Clock
// http://home.roadrunner.com/~jgglatt/tech/midispec/clock.htm
/* There are 24 MIDI Clocks in every quarter note. (12 MIDI Clocks in an eighth note, 6 MIDI Clocks in a 16th, etc).
   Therefore, when a slave device counts down the receipt of 24 MIDI Clock messages, it knows that one quarter note
   has passed. When the slave counts off another 24 MIDI Clock messages, it knows that another quarter note has passed.
   Etc. Of course, the rate that the master sends these messages is based upon the master's tempo.

   For example, for a tempo of 120 BPM (ie, there are 120 quarter notes in every minute), the master sends a MIDI clock
   every 20833 microseconds. (ie, There are 1,000,000 microseconds in a second. Therefore, there are 60,000,000
   microseconds in a minute. At a tempo of 120 BPM, there are 120 quarter notes per minute. There are 24 MIDI clocks
   in each quarter note. Therefore, there should be 24 * 120 MIDI Clocks per minute.
   So, each MIDI Clock is sent at a rate of 60,000,000/(24 * 120) microseconds).
*/
bool __stdcall CFinalProjectEMJ::midiClock()
{

	return true;
}

// any midi message other than note on, note off, pitchbend, mod wheel or clock
bool __stdcall CFinalProjectEMJ::midiMessage(unsigned char cChannel, unsigned char cStatus, unsigned char
						   				  cData1, unsigned char cData2)
{
	return true;
}

/* doVSTSampleAccurateParamUpdates
	Short handler for VST3 sample accurate automation added in v6.8.0.5
	There is nothing for you to modify here.
*/
void CFinalProjectEMJ::doVSTSampleAccurateParamUpdates()
{
	// --- for sample accurate parameter automation in VST3 plugins; ignore otherwise
	if (!m_ppControlTable) return; /// should NEVER happen
	for (int i = 0; i < m_uControlListCount; i++)
	{
		if (m_ppControlTable[i] && m_ppControlTable[i]->pvAddlData)
		{
			double dValue = 0;
			if (((IParamUpdateQueue *)m_ppControlTable[i]->pvAddlData)->getNextValue(dValue))
			{
				setNormalizedParameter(m_ppControlTable[i], dValue, true);
			}
		}
	}
}

// --- showGUI()
//     This is the main interface function for all GUIs, including custom GUIs
//     It is also where you deal with Custom Controls (see Advanced GUI API)
void* __stdcall CFinalProjectEMJ::showGUI(void* pInfo)
{
	// --- ALWAYS try base class first in case of future updates
	void* result = CPlugIn::showGUI(pInfo);
	if(result)
		return result;

	/* Uncomment if using advanced GUI API: see www.willpirkle.com for details and sample code
	// --- uncloak the info struct
	VSTGUI_VIEW_INFO* info = (VSTGUI_VIEW_INFO*)pInfo;
	if(!info) return NULL;

	switch(info->message)
	{
		case GUI_DID_OPEN:
		{
			return NULL;
		}
		case GUI_WILL_CLOSE:
		{
			return NULL;
		}
		case GUI_CUSTOMVIEW:
		{
			// --- create custom view, return a CView* cloaked as void* or NULL if not supported
			return NULL;
		}

		case GUI_HAS_USER_CUSTOM:
		{
			// --- set this variable to true if you have a custom GUI
			info->bHasUserCustomView = false;
			return NULL;
		}

		// --- create your custom VSTGUI4 object using the CVSTGUIController (supplied),
		//     a subclass of the CVSTGUIController that you supply, a VSTGUI4 object
		//     that is derived at least from: VSTGUIEditorInterface, CControlListener, CBaseObject
		//     see VSTGUIController.h for an example
		//
		//     open() sets the new size of the window in info->size
		//     return a pointer to the newly created object
		case GUI_USER_CUSTOM_OPEN:
		{
			return NULL;
		}
		// --- call the close() function and delete the controller object
		case GUI_USER_CUSTOM_CLOSE:
		{
			return NULL;
		}
		// --- handle paint-specific timer stuff
		case GUI_TIMER_PING:
		{

			return NULL;
		}
	} */

	return NULL;
}

// --- DO NOT EDIT OR DELETE THIS FUNCTION ----------------------------------------------- //
bool __stdcall CFinalProjectEMJ::initUI()
{
	// ADDED BY RACKAFX -- DO NOT EDIT THIS CODE!!! -------------------------------------- //
	if(m_UIControlList.count() > 0)
		return true;

// **--0xDEA7--**

	int nIndexer = 0;
	m_fPreGain_dB = 12.000000;
	CUICtrl* ui0 = new CUICtrl;
	ui0->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui0->uControlId = 0;
	ui0->bLogSlider = false;
	ui0->bExpSlider = false;
	ui0->fUserDisplayDataLoLimit = 0.000000;
	ui0->fUserDisplayDataHiLimit = 20.000000;
	ui0->uUserDataType = floatData;
	ui0->fInitUserIntValue = 0;
	ui0->fInitUserFloatValue = 12.000000;
	ui0->fInitUserDoubleValue = 0;
	ui0->fInitUserUINTValue = 0;
	ui0->m_pUserCookedIntData = NULL;
	ui0->m_pUserCookedFloatData = &m_fPreGain_dB;
	ui0->m_pUserCookedDoubleData = NULL;
	ui0->m_pUserCookedUINTData = NULL;
	ui0->cControlUnits = "dB";
	ui0->cVariableName = "m_fPreGain_dB";
	ui0->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui0->dPresetData[0] = 12.000000;ui0->dPresetData[1] = 0.000000;ui0->dPresetData[2] = 0.000000;ui0->dPresetData[3] = 0.000000;ui0->dPresetData[4] = 0.000000;ui0->dPresetData[5] = 0.000000;ui0->dPresetData[6] = 0.000000;ui0->dPresetData[7] = 0.000000;ui0->dPresetData[8] = 0.000000;ui0->dPresetData[9] = 0.000000;ui0->dPresetData[10] = 0.000000;ui0->dPresetData[11] = 0.000000;ui0->dPresetData[12] = 0.000000;ui0->dPresetData[13] = 0.000000;ui0->dPresetData[14] = 0.000000;ui0->dPresetData[15] = 0.000000;
	ui0->cControlName = "Pre Gain";
	ui0->bOwnerControl = false;
	ui0->bMIDIControl = false;
	ui0->uMIDIControlCommand = 176;
	ui0->uMIDIControlName = 3;
	ui0->uMIDIControlChannel = 0;
	ui0->nGUIRow = nIndexer++;
	ui0->nGUIColumn = -1;
	ui0->bEnableParamSmoothing = false;
	ui0->fSmoothingTimeInMs = 100.00;
	ui0->uControlTheme[0] = 0; ui0->uControlTheme[1] = 0; ui0->uControlTheme[2] = 0; ui0->uControlTheme[3] = 0; ui0->uControlTheme[4] = 0; ui0->uControlTheme[5] = 0; ui0->uControlTheme[6] = 0; ui0->uControlTheme[7] = 0; ui0->uControlTheme[8] = 0; ui0->uControlTheme[9] = 0; ui0->uControlTheme[10] = 0; ui0->uControlTheme[11] = 0; ui0->uControlTheme[12] = 0; ui0->uControlTheme[13] = 0; ui0->uControlTheme[14] = 0; ui0->uControlTheme[15] = 0; ui0->uControlTheme[16] = 2; ui0->uControlTheme[17] = 0; ui0->uControlTheme[18] = 0; ui0->uControlTheme[19] = 0; ui0->uControlTheme[20] = 0; ui0->uControlTheme[21] = 0; ui0->uControlTheme[22] = 0; ui0->uControlTheme[23] = 0; ui0->uControlTheme[24] = 0; ui0->uControlTheme[25] = 0; ui0->uControlTheme[26] = 0; ui0->uControlTheme[27] = 0; ui0->uControlTheme[28] = 0; ui0->uControlTheme[29] = 0; ui0->uControlTheme[30] = 0; ui0->uControlTheme[31] = 0; 
	ui0->uFluxCapControl[0] = 0; ui0->uFluxCapControl[1] = 0; ui0->uFluxCapControl[2] = 0; ui0->uFluxCapControl[3] = 0; ui0->uFluxCapControl[4] = 0; ui0->uFluxCapControl[5] = 0; ui0->uFluxCapControl[6] = 0; ui0->uFluxCapControl[7] = 0; ui0->uFluxCapControl[8] = 0; ui0->uFluxCapControl[9] = 0; ui0->uFluxCapControl[10] = 0; ui0->uFluxCapControl[11] = 0; ui0->uFluxCapControl[12] = 0; ui0->uFluxCapControl[13] = 0; ui0->uFluxCapControl[14] = 0; ui0->uFluxCapControl[15] = 0; ui0->uFluxCapControl[16] = 0; ui0->uFluxCapControl[17] = 0; ui0->uFluxCapControl[18] = 0; ui0->uFluxCapControl[19] = 0; ui0->uFluxCapControl[20] = 0; ui0->uFluxCapControl[21] = 0; ui0->uFluxCapControl[22] = 0; ui0->uFluxCapControl[23] = 0; ui0->uFluxCapControl[24] = 0; ui0->uFluxCapControl[25] = 0; ui0->uFluxCapControl[26] = 0; ui0->uFluxCapControl[27] = 0; ui0->uFluxCapControl[28] = 0; ui0->uFluxCapControl[29] = 0; ui0->uFluxCapControl[30] = 0; ui0->uFluxCapControl[31] = 0; ui0->uFluxCapControl[32] = 0; ui0->uFluxCapControl[33] = 0; ui0->uFluxCapControl[34] = 0; ui0->uFluxCapControl[35] = 0; ui0->uFluxCapControl[36] = 0; ui0->uFluxCapControl[37] = 0; ui0->uFluxCapControl[38] = 0; ui0->uFluxCapControl[39] = 0; ui0->uFluxCapControl[40] = 0; ui0->uFluxCapControl[41] = 0; ui0->uFluxCapControl[42] = 0; ui0->uFluxCapControl[43] = 0; ui0->uFluxCapControl[44] = 0; ui0->uFluxCapControl[45] = 0; ui0->uFluxCapControl[46] = 0; ui0->uFluxCapControl[47] = 0; ui0->uFluxCapControl[48] = 0; ui0->uFluxCapControl[49] = 0; ui0->uFluxCapControl[50] = 0; ui0->uFluxCapControl[51] = 0; ui0->uFluxCapControl[52] = 0; ui0->uFluxCapControl[53] = 0; ui0->uFluxCapControl[54] = 0; ui0->uFluxCapControl[55] = 0; ui0->uFluxCapControl[56] = 0; ui0->uFluxCapControl[57] = 0; ui0->uFluxCapControl[58] = 0; ui0->uFluxCapControl[59] = 0; ui0->uFluxCapControl[60] = 0; ui0->uFluxCapControl[61] = 0; ui0->uFluxCapControl[62] = 0; ui0->uFluxCapControl[63] = 0; 
	ui0->fFluxCapData[0] = 0.000000; ui0->fFluxCapData[1] = 0.000000; ui0->fFluxCapData[2] = 0.000000; ui0->fFluxCapData[3] = 0.000000; ui0->fFluxCapData[4] = 0.000000; ui0->fFluxCapData[5] = 0.000000; ui0->fFluxCapData[6] = 0.000000; ui0->fFluxCapData[7] = 0.000000; ui0->fFluxCapData[8] = 0.000000; ui0->fFluxCapData[9] = 0.000000; ui0->fFluxCapData[10] = 0.000000; ui0->fFluxCapData[11] = 0.000000; ui0->fFluxCapData[12] = 0.000000; ui0->fFluxCapData[13] = 0.000000; ui0->fFluxCapData[14] = 0.000000; ui0->fFluxCapData[15] = 0.000000; ui0->fFluxCapData[16] = 0.000000; ui0->fFluxCapData[17] = 0.000000; ui0->fFluxCapData[18] = 0.000000; ui0->fFluxCapData[19] = 0.000000; ui0->fFluxCapData[20] = 0.000000; ui0->fFluxCapData[21] = 0.000000; ui0->fFluxCapData[22] = 0.000000; ui0->fFluxCapData[23] = 0.000000; ui0->fFluxCapData[24] = 0.000000; ui0->fFluxCapData[25] = 0.000000; ui0->fFluxCapData[26] = 0.000000; ui0->fFluxCapData[27] = 0.000000; ui0->fFluxCapData[28] = 0.000000; ui0->fFluxCapData[29] = 0.000000; ui0->fFluxCapData[30] = 0.000000; ui0->fFluxCapData[31] = 0.000000; ui0->fFluxCapData[32] = 0.000000; ui0->fFluxCapData[33] = 0.000000; ui0->fFluxCapData[34] = 0.000000; ui0->fFluxCapData[35] = 0.000000; ui0->fFluxCapData[36] = 0.000000; ui0->fFluxCapData[37] = 0.000000; ui0->fFluxCapData[38] = 0.000000; ui0->fFluxCapData[39] = 0.000000; ui0->fFluxCapData[40] = 0.000000; ui0->fFluxCapData[41] = 0.000000; ui0->fFluxCapData[42] = 0.000000; ui0->fFluxCapData[43] = 0.000000; ui0->fFluxCapData[44] = 0.000000; ui0->fFluxCapData[45] = 0.000000; ui0->fFluxCapData[46] = 0.000000; ui0->fFluxCapData[47] = 0.000000; ui0->fFluxCapData[48] = 0.000000; ui0->fFluxCapData[49] = 0.000000; ui0->fFluxCapData[50] = 0.000000; ui0->fFluxCapData[51] = 0.000000; ui0->fFluxCapData[52] = 0.000000; ui0->fFluxCapData[53] = 0.000000; ui0->fFluxCapData[54] = 0.000000; ui0->fFluxCapData[55] = 0.000000; ui0->fFluxCapData[56] = 0.000000; ui0->fFluxCapData[57] = 0.000000; ui0->fFluxCapData[58] = 0.000000; ui0->fFluxCapData[59] = 0.000000; ui0->fFluxCapData[60] = 0.000000; ui0->fFluxCapData[61] = 0.000000; ui0->fFluxCapData[62] = 0.000000; ui0->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui0);
	delete ui0;


	m_fThreshold = 0.200000;
	CUICtrl* ui1 = new CUICtrl;
	ui1->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui1->uControlId = 1;
	ui1->bLogSlider = false;
	ui1->bExpSlider = false;
	ui1->fUserDisplayDataLoLimit = 0.000000;
	ui1->fUserDisplayDataHiLimit = 1.000000;
	ui1->uUserDataType = floatData;
	ui1->fInitUserIntValue = 0;
	ui1->fInitUserFloatValue = 0.200000;
	ui1->fInitUserDoubleValue = 0;
	ui1->fInitUserUINTValue = 0;
	ui1->m_pUserCookedIntData = NULL;
	ui1->m_pUserCookedFloatData = &m_fThreshold;
	ui1->m_pUserCookedDoubleData = NULL;
	ui1->m_pUserCookedUINTData = NULL;
	ui1->cControlUnits = "";
	ui1->cVariableName = "m_fThreshold";
	ui1->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui1->dPresetData[0] = 0.200000;ui1->dPresetData[1] = 0.000000;ui1->dPresetData[2] = 0.000000;ui1->dPresetData[3] = 0.000000;ui1->dPresetData[4] = 0.000000;ui1->dPresetData[5] = 0.000000;ui1->dPresetData[6] = 0.000000;ui1->dPresetData[7] = 0.000000;ui1->dPresetData[8] = 0.000000;ui1->dPresetData[9] = 0.000000;ui1->dPresetData[10] = 0.000000;ui1->dPresetData[11] = 0.000000;ui1->dPresetData[12] = 0.000000;ui1->dPresetData[13] = 0.000000;ui1->dPresetData[14] = 0.000000;ui1->dPresetData[15] = 0.000000;
	ui1->cControlName = "Threshold";
	ui1->bOwnerControl = false;
	ui1->bMIDIControl = false;
	ui1->uMIDIControlCommand = 176;
	ui1->uMIDIControlName = 3;
	ui1->uMIDIControlChannel = 0;
	ui1->nGUIRow = nIndexer++;
	ui1->nGUIColumn = -1;
	ui1->bEnableParamSmoothing = false;
	ui1->fSmoothingTimeInMs = 100.00;
	ui1->uControlTheme[0] = 0; ui1->uControlTheme[1] = 0; ui1->uControlTheme[2] = 0; ui1->uControlTheme[3] = 0; ui1->uControlTheme[4] = 0; ui1->uControlTheme[5] = 0; ui1->uControlTheme[6] = 0; ui1->uControlTheme[7] = 0; ui1->uControlTheme[8] = 0; ui1->uControlTheme[9] = 0; ui1->uControlTheme[10] = 0; ui1->uControlTheme[11] = 0; ui1->uControlTheme[12] = 0; ui1->uControlTheme[13] = 0; ui1->uControlTheme[14] = 0; ui1->uControlTheme[15] = 0; ui1->uControlTheme[16] = 2; ui1->uControlTheme[17] = 0; ui1->uControlTheme[18] = 0; ui1->uControlTheme[19] = 0; ui1->uControlTheme[20] = 0; ui1->uControlTheme[21] = 0; ui1->uControlTheme[22] = 0; ui1->uControlTheme[23] = 0; ui1->uControlTheme[24] = 0; ui1->uControlTheme[25] = 0; ui1->uControlTheme[26] = 0; ui1->uControlTheme[27] = 0; ui1->uControlTheme[28] = 0; ui1->uControlTheme[29] = 0; ui1->uControlTheme[30] = 0; ui1->uControlTheme[31] = 0; 
	ui1->uFluxCapControl[0] = 0; ui1->uFluxCapControl[1] = 0; ui1->uFluxCapControl[2] = 0; ui1->uFluxCapControl[3] = 0; ui1->uFluxCapControl[4] = 0; ui1->uFluxCapControl[5] = 0; ui1->uFluxCapControl[6] = 0; ui1->uFluxCapControl[7] = 0; ui1->uFluxCapControl[8] = 0; ui1->uFluxCapControl[9] = 0; ui1->uFluxCapControl[10] = 0; ui1->uFluxCapControl[11] = 0; ui1->uFluxCapControl[12] = 0; ui1->uFluxCapControl[13] = 0; ui1->uFluxCapControl[14] = 0; ui1->uFluxCapControl[15] = 0; ui1->uFluxCapControl[16] = 0; ui1->uFluxCapControl[17] = 0; ui1->uFluxCapControl[18] = 0; ui1->uFluxCapControl[19] = 0; ui1->uFluxCapControl[20] = 0; ui1->uFluxCapControl[21] = 0; ui1->uFluxCapControl[22] = 0; ui1->uFluxCapControl[23] = 0; ui1->uFluxCapControl[24] = 0; ui1->uFluxCapControl[25] = 0; ui1->uFluxCapControl[26] = 0; ui1->uFluxCapControl[27] = 0; ui1->uFluxCapControl[28] = 0; ui1->uFluxCapControl[29] = 0; ui1->uFluxCapControl[30] = 0; ui1->uFluxCapControl[31] = 0; ui1->uFluxCapControl[32] = 0; ui1->uFluxCapControl[33] = 0; ui1->uFluxCapControl[34] = 0; ui1->uFluxCapControl[35] = 0; ui1->uFluxCapControl[36] = 0; ui1->uFluxCapControl[37] = 0; ui1->uFluxCapControl[38] = 0; ui1->uFluxCapControl[39] = 0; ui1->uFluxCapControl[40] = 0; ui1->uFluxCapControl[41] = 0; ui1->uFluxCapControl[42] = 0; ui1->uFluxCapControl[43] = 0; ui1->uFluxCapControl[44] = 0; ui1->uFluxCapControl[45] = 0; ui1->uFluxCapControl[46] = 0; ui1->uFluxCapControl[47] = 0; ui1->uFluxCapControl[48] = 0; ui1->uFluxCapControl[49] = 0; ui1->uFluxCapControl[50] = 0; ui1->uFluxCapControl[51] = 0; ui1->uFluxCapControl[52] = 0; ui1->uFluxCapControl[53] = 0; ui1->uFluxCapControl[54] = 0; ui1->uFluxCapControl[55] = 0; ui1->uFluxCapControl[56] = 0; ui1->uFluxCapControl[57] = 0; ui1->uFluxCapControl[58] = 0; ui1->uFluxCapControl[59] = 0; ui1->uFluxCapControl[60] = 0; ui1->uFluxCapControl[61] = 0; ui1->uFluxCapControl[62] = 0; ui1->uFluxCapControl[63] = 0; 
	ui1->fFluxCapData[0] = 0.000000; ui1->fFluxCapData[1] = 0.000000; ui1->fFluxCapData[2] = 0.000000; ui1->fFluxCapData[3] = 0.000000; ui1->fFluxCapData[4] = 0.000000; ui1->fFluxCapData[5] = 0.000000; ui1->fFluxCapData[6] = 0.000000; ui1->fFluxCapData[7] = 0.000000; ui1->fFluxCapData[8] = 0.000000; ui1->fFluxCapData[9] = 0.000000; ui1->fFluxCapData[10] = 0.000000; ui1->fFluxCapData[11] = 0.000000; ui1->fFluxCapData[12] = 0.000000; ui1->fFluxCapData[13] = 0.000000; ui1->fFluxCapData[14] = 0.000000; ui1->fFluxCapData[15] = 0.000000; ui1->fFluxCapData[16] = 0.000000; ui1->fFluxCapData[17] = 0.000000; ui1->fFluxCapData[18] = 0.000000; ui1->fFluxCapData[19] = 0.000000; ui1->fFluxCapData[20] = 0.000000; ui1->fFluxCapData[21] = 0.000000; ui1->fFluxCapData[22] = 0.000000; ui1->fFluxCapData[23] = 0.000000; ui1->fFluxCapData[24] = 0.000000; ui1->fFluxCapData[25] = 0.000000; ui1->fFluxCapData[26] = 0.000000; ui1->fFluxCapData[27] = 0.000000; ui1->fFluxCapData[28] = 0.000000; ui1->fFluxCapData[29] = 0.000000; ui1->fFluxCapData[30] = 0.000000; ui1->fFluxCapData[31] = 0.000000; ui1->fFluxCapData[32] = 0.000000; ui1->fFluxCapData[33] = 0.000000; ui1->fFluxCapData[34] = 0.000000; ui1->fFluxCapData[35] = 0.000000; ui1->fFluxCapData[36] = 0.000000; ui1->fFluxCapData[37] = 0.000000; ui1->fFluxCapData[38] = 0.000000; ui1->fFluxCapData[39] = 0.000000; ui1->fFluxCapData[40] = 0.000000; ui1->fFluxCapData[41] = 0.000000; ui1->fFluxCapData[42] = 0.000000; ui1->fFluxCapData[43] = 0.000000; ui1->fFluxCapData[44] = 0.000000; ui1->fFluxCapData[45] = 0.000000; ui1->fFluxCapData[46] = 0.000000; ui1->fFluxCapData[47] = 0.000000; ui1->fFluxCapData[48] = 0.000000; ui1->fFluxCapData[49] = 0.000000; ui1->fFluxCapData[50] = 0.000000; ui1->fFluxCapData[51] = 0.000000; ui1->fFluxCapData[52] = 0.000000; ui1->fFluxCapData[53] = 0.000000; ui1->fFluxCapData[54] = 0.000000; ui1->fFluxCapData[55] = 0.000000; ui1->fFluxCapData[56] = 0.000000; ui1->fFluxCapData[57] = 0.000000; ui1->fFluxCapData[58] = 0.000000; ui1->fFluxCapData[59] = 0.000000; ui1->fFluxCapData[60] = 0.000000; ui1->fFluxCapData[61] = 0.000000; ui1->fFluxCapData[62] = 0.000000; ui1->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui1);
	delete ui1;


	m_fAttack_mSec = 25.000000;
	CUICtrl* ui2 = new CUICtrl;
	ui2->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui2->uControlId = 2;
	ui2->bLogSlider = false;
	ui2->bExpSlider = false;
	ui2->fUserDisplayDataLoLimit = 0.000000;
	ui2->fUserDisplayDataHiLimit = 100.000000;
	ui2->uUserDataType = floatData;
	ui2->fInitUserIntValue = 0;
	ui2->fInitUserFloatValue = 25.000000;
	ui2->fInitUserDoubleValue = 0;
	ui2->fInitUserUINTValue = 0;
	ui2->m_pUserCookedIntData = NULL;
	ui2->m_pUserCookedFloatData = &m_fAttack_mSec;
	ui2->m_pUserCookedDoubleData = NULL;
	ui2->m_pUserCookedUINTData = NULL;
	ui2->cControlUnits = "mSec";
	ui2->cVariableName = "m_fAttack_mSec";
	ui2->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui2->dPresetData[0] = 25.000000;ui2->dPresetData[1] = 0.000000;ui2->dPresetData[2] = 0.000000;ui2->dPresetData[3] = 0.000000;ui2->dPresetData[4] = 0.000000;ui2->dPresetData[5] = 0.000000;ui2->dPresetData[6] = 0.000000;ui2->dPresetData[7] = 0.000000;ui2->dPresetData[8] = 0.000000;ui2->dPresetData[9] = 0.000000;ui2->dPresetData[10] = 0.000000;ui2->dPresetData[11] = 0.000000;ui2->dPresetData[12] = 0.000000;ui2->dPresetData[13] = 0.000000;ui2->dPresetData[14] = 0.000000;ui2->dPresetData[15] = 0.000000;
	ui2->cControlName = "Attack";
	ui2->bOwnerControl = false;
	ui2->bMIDIControl = false;
	ui2->uMIDIControlCommand = 176;
	ui2->uMIDIControlName = 3;
	ui2->uMIDIControlChannel = 0;
	ui2->nGUIRow = nIndexer++;
	ui2->nGUIColumn = -1;
	ui2->bEnableParamSmoothing = false;
	ui2->fSmoothingTimeInMs = 100.00;
	ui2->uControlTheme[0] = 0; ui2->uControlTheme[1] = 0; ui2->uControlTheme[2] = 0; ui2->uControlTheme[3] = 0; ui2->uControlTheme[4] = 0; ui2->uControlTheme[5] = 0; ui2->uControlTheme[6] = 0; ui2->uControlTheme[7] = 0; ui2->uControlTheme[8] = 0; ui2->uControlTheme[9] = 0; ui2->uControlTheme[10] = 0; ui2->uControlTheme[11] = 0; ui2->uControlTheme[12] = 0; ui2->uControlTheme[13] = 0; ui2->uControlTheme[14] = 0; ui2->uControlTheme[15] = 0; ui2->uControlTheme[16] = 2; ui2->uControlTheme[17] = 0; ui2->uControlTheme[18] = 0; ui2->uControlTheme[19] = 0; ui2->uControlTheme[20] = 0; ui2->uControlTheme[21] = 0; ui2->uControlTheme[22] = 0; ui2->uControlTheme[23] = 0; ui2->uControlTheme[24] = 0; ui2->uControlTheme[25] = 0; ui2->uControlTheme[26] = 0; ui2->uControlTheme[27] = 0; ui2->uControlTheme[28] = 0; ui2->uControlTheme[29] = 0; ui2->uControlTheme[30] = 0; ui2->uControlTheme[31] = 0; 
	ui2->uFluxCapControl[0] = 0; ui2->uFluxCapControl[1] = 0; ui2->uFluxCapControl[2] = 0; ui2->uFluxCapControl[3] = 0; ui2->uFluxCapControl[4] = 0; ui2->uFluxCapControl[5] = 0; ui2->uFluxCapControl[6] = 0; ui2->uFluxCapControl[7] = 0; ui2->uFluxCapControl[8] = 0; ui2->uFluxCapControl[9] = 0; ui2->uFluxCapControl[10] = 0; ui2->uFluxCapControl[11] = 0; ui2->uFluxCapControl[12] = 0; ui2->uFluxCapControl[13] = 0; ui2->uFluxCapControl[14] = 0; ui2->uFluxCapControl[15] = 0; ui2->uFluxCapControl[16] = 0; ui2->uFluxCapControl[17] = 0; ui2->uFluxCapControl[18] = 0; ui2->uFluxCapControl[19] = 0; ui2->uFluxCapControl[20] = 0; ui2->uFluxCapControl[21] = 0; ui2->uFluxCapControl[22] = 0; ui2->uFluxCapControl[23] = 0; ui2->uFluxCapControl[24] = 0; ui2->uFluxCapControl[25] = 0; ui2->uFluxCapControl[26] = 0; ui2->uFluxCapControl[27] = 0; ui2->uFluxCapControl[28] = 0; ui2->uFluxCapControl[29] = 0; ui2->uFluxCapControl[30] = 0; ui2->uFluxCapControl[31] = 0; ui2->uFluxCapControl[32] = 0; ui2->uFluxCapControl[33] = 0; ui2->uFluxCapControl[34] = 0; ui2->uFluxCapControl[35] = 0; ui2->uFluxCapControl[36] = 0; ui2->uFluxCapControl[37] = 0; ui2->uFluxCapControl[38] = 0; ui2->uFluxCapControl[39] = 0; ui2->uFluxCapControl[40] = 0; ui2->uFluxCapControl[41] = 0; ui2->uFluxCapControl[42] = 0; ui2->uFluxCapControl[43] = 0; ui2->uFluxCapControl[44] = 0; ui2->uFluxCapControl[45] = 0; ui2->uFluxCapControl[46] = 0; ui2->uFluxCapControl[47] = 0; ui2->uFluxCapControl[48] = 0; ui2->uFluxCapControl[49] = 0; ui2->uFluxCapControl[50] = 0; ui2->uFluxCapControl[51] = 0; ui2->uFluxCapControl[52] = 0; ui2->uFluxCapControl[53] = 0; ui2->uFluxCapControl[54] = 0; ui2->uFluxCapControl[55] = 0; ui2->uFluxCapControl[56] = 0; ui2->uFluxCapControl[57] = 0; ui2->uFluxCapControl[58] = 0; ui2->uFluxCapControl[59] = 0; ui2->uFluxCapControl[60] = 0; ui2->uFluxCapControl[61] = 0; ui2->uFluxCapControl[62] = 0; ui2->uFluxCapControl[63] = 0; 
	ui2->fFluxCapData[0] = 0.000000; ui2->fFluxCapData[1] = 0.000000; ui2->fFluxCapData[2] = 0.000000; ui2->fFluxCapData[3] = 0.000000; ui2->fFluxCapData[4] = 0.000000; ui2->fFluxCapData[5] = 0.000000; ui2->fFluxCapData[6] = 0.000000; ui2->fFluxCapData[7] = 0.000000; ui2->fFluxCapData[8] = 0.000000; ui2->fFluxCapData[9] = 0.000000; ui2->fFluxCapData[10] = 0.000000; ui2->fFluxCapData[11] = 0.000000; ui2->fFluxCapData[12] = 0.000000; ui2->fFluxCapData[13] = 0.000000; ui2->fFluxCapData[14] = 0.000000; ui2->fFluxCapData[15] = 0.000000; ui2->fFluxCapData[16] = 0.000000; ui2->fFluxCapData[17] = 0.000000; ui2->fFluxCapData[18] = 0.000000; ui2->fFluxCapData[19] = 0.000000; ui2->fFluxCapData[20] = 0.000000; ui2->fFluxCapData[21] = 0.000000; ui2->fFluxCapData[22] = 0.000000; ui2->fFluxCapData[23] = 0.000000; ui2->fFluxCapData[24] = 0.000000; ui2->fFluxCapData[25] = 0.000000; ui2->fFluxCapData[26] = 0.000000; ui2->fFluxCapData[27] = 0.000000; ui2->fFluxCapData[28] = 0.000000; ui2->fFluxCapData[29] = 0.000000; ui2->fFluxCapData[30] = 0.000000; ui2->fFluxCapData[31] = 0.000000; ui2->fFluxCapData[32] = 0.000000; ui2->fFluxCapData[33] = 0.000000; ui2->fFluxCapData[34] = 0.000000; ui2->fFluxCapData[35] = 0.000000; ui2->fFluxCapData[36] = 0.000000; ui2->fFluxCapData[37] = 0.000000; ui2->fFluxCapData[38] = 0.000000; ui2->fFluxCapData[39] = 0.000000; ui2->fFluxCapData[40] = 0.000000; ui2->fFluxCapData[41] = 0.000000; ui2->fFluxCapData[42] = 0.000000; ui2->fFluxCapData[43] = 0.000000; ui2->fFluxCapData[44] = 0.000000; ui2->fFluxCapData[45] = 0.000000; ui2->fFluxCapData[46] = 0.000000; ui2->fFluxCapData[47] = 0.000000; ui2->fFluxCapData[48] = 0.000000; ui2->fFluxCapData[49] = 0.000000; ui2->fFluxCapData[50] = 0.000000; ui2->fFluxCapData[51] = 0.000000; ui2->fFluxCapData[52] = 0.000000; ui2->fFluxCapData[53] = 0.000000; ui2->fFluxCapData[54] = 0.000000; ui2->fFluxCapData[55] = 0.000000; ui2->fFluxCapData[56] = 0.000000; ui2->fFluxCapData[57] = 0.000000; ui2->fFluxCapData[58] = 0.000000; ui2->fFluxCapData[59] = 0.000000; ui2->fFluxCapData[60] = 0.000000; ui2->fFluxCapData[61] = 0.000000; ui2->fFluxCapData[62] = 0.000000; ui2->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui2);
	delete ui2;


	m_fRelease_mSec = 50.000000;
	CUICtrl* ui3 = new CUICtrl;
	ui3->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui3->uControlId = 3;
	ui3->bLogSlider = false;
	ui3->bExpSlider = false;
	ui3->fUserDisplayDataLoLimit = 20.000000;
	ui3->fUserDisplayDataHiLimit = 250.000000;
	ui3->uUserDataType = floatData;
	ui3->fInitUserIntValue = 0;
	ui3->fInitUserFloatValue = 50.000000;
	ui3->fInitUserDoubleValue = 0;
	ui3->fInitUserUINTValue = 0;
	ui3->m_pUserCookedIntData = NULL;
	ui3->m_pUserCookedFloatData = &m_fRelease_mSec;
	ui3->m_pUserCookedDoubleData = NULL;
	ui3->m_pUserCookedUINTData = NULL;
	ui3->cControlUnits = "mSec";
	ui3->cVariableName = "m_fRelease_mSec";
	ui3->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui3->dPresetData[0] = 50.000000;ui3->dPresetData[1] = 0.000000;ui3->dPresetData[2] = 0.000000;ui3->dPresetData[3] = 0.000000;ui3->dPresetData[4] = 0.000000;ui3->dPresetData[5] = 0.000000;ui3->dPresetData[6] = 0.000000;ui3->dPresetData[7] = 0.000000;ui3->dPresetData[8] = 0.000000;ui3->dPresetData[9] = 0.000000;ui3->dPresetData[10] = 0.000000;ui3->dPresetData[11] = 0.000000;ui3->dPresetData[12] = 0.000000;ui3->dPresetData[13] = 0.000000;ui3->dPresetData[14] = 0.000000;ui3->dPresetData[15] = 0.000000;
	ui3->cControlName = "Release";
	ui3->bOwnerControl = false;
	ui3->bMIDIControl = false;
	ui3->uMIDIControlCommand = 176;
	ui3->uMIDIControlName = 3;
	ui3->uMIDIControlChannel = 0;
	ui3->nGUIRow = nIndexer++;
	ui3->nGUIColumn = -1;
	ui3->bEnableParamSmoothing = false;
	ui3->fSmoothingTimeInMs = 100.00;
	ui3->uControlTheme[0] = 0; ui3->uControlTheme[1] = 0; ui3->uControlTheme[2] = 0; ui3->uControlTheme[3] = 0; ui3->uControlTheme[4] = 0; ui3->uControlTheme[5] = 0; ui3->uControlTheme[6] = 0; ui3->uControlTheme[7] = 0; ui3->uControlTheme[8] = 0; ui3->uControlTheme[9] = 0; ui3->uControlTheme[10] = 0; ui3->uControlTheme[11] = 0; ui3->uControlTheme[12] = 0; ui3->uControlTheme[13] = 0; ui3->uControlTheme[14] = 0; ui3->uControlTheme[15] = 0; ui3->uControlTheme[16] = 2; ui3->uControlTheme[17] = 0; ui3->uControlTheme[18] = 0; ui3->uControlTheme[19] = 0; ui3->uControlTheme[20] = 0; ui3->uControlTheme[21] = 0; ui3->uControlTheme[22] = 0; ui3->uControlTheme[23] = 0; ui3->uControlTheme[24] = 0; ui3->uControlTheme[25] = 0; ui3->uControlTheme[26] = 0; ui3->uControlTheme[27] = 0; ui3->uControlTheme[28] = 0; ui3->uControlTheme[29] = 0; ui3->uControlTheme[30] = 0; ui3->uControlTheme[31] = 0; 
	ui3->uFluxCapControl[0] = 0; ui3->uFluxCapControl[1] = 0; ui3->uFluxCapControl[2] = 0; ui3->uFluxCapControl[3] = 0; ui3->uFluxCapControl[4] = 0; ui3->uFluxCapControl[5] = 0; ui3->uFluxCapControl[6] = 0; ui3->uFluxCapControl[7] = 0; ui3->uFluxCapControl[8] = 0; ui3->uFluxCapControl[9] = 0; ui3->uFluxCapControl[10] = 0; ui3->uFluxCapControl[11] = 0; ui3->uFluxCapControl[12] = 0; ui3->uFluxCapControl[13] = 0; ui3->uFluxCapControl[14] = 0; ui3->uFluxCapControl[15] = 0; ui3->uFluxCapControl[16] = 0; ui3->uFluxCapControl[17] = 0; ui3->uFluxCapControl[18] = 0; ui3->uFluxCapControl[19] = 0; ui3->uFluxCapControl[20] = 0; ui3->uFluxCapControl[21] = 0; ui3->uFluxCapControl[22] = 0; ui3->uFluxCapControl[23] = 0; ui3->uFluxCapControl[24] = 0; ui3->uFluxCapControl[25] = 0; ui3->uFluxCapControl[26] = 0; ui3->uFluxCapControl[27] = 0; ui3->uFluxCapControl[28] = 0; ui3->uFluxCapControl[29] = 0; ui3->uFluxCapControl[30] = 0; ui3->uFluxCapControl[31] = 0; ui3->uFluxCapControl[32] = 0; ui3->uFluxCapControl[33] = 0; ui3->uFluxCapControl[34] = 0; ui3->uFluxCapControl[35] = 0; ui3->uFluxCapControl[36] = 0; ui3->uFluxCapControl[37] = 0; ui3->uFluxCapControl[38] = 0; ui3->uFluxCapControl[39] = 0; ui3->uFluxCapControl[40] = 0; ui3->uFluxCapControl[41] = 0; ui3->uFluxCapControl[42] = 0; ui3->uFluxCapControl[43] = 0; ui3->uFluxCapControl[44] = 0; ui3->uFluxCapControl[45] = 0; ui3->uFluxCapControl[46] = 0; ui3->uFluxCapControl[47] = 0; ui3->uFluxCapControl[48] = 0; ui3->uFluxCapControl[49] = 0; ui3->uFluxCapControl[50] = 0; ui3->uFluxCapControl[51] = 0; ui3->uFluxCapControl[52] = 0; ui3->uFluxCapControl[53] = 0; ui3->uFluxCapControl[54] = 0; ui3->uFluxCapControl[55] = 0; ui3->uFluxCapControl[56] = 0; ui3->uFluxCapControl[57] = 0; ui3->uFluxCapControl[58] = 0; ui3->uFluxCapControl[59] = 0; ui3->uFluxCapControl[60] = 0; ui3->uFluxCapControl[61] = 0; ui3->uFluxCapControl[62] = 0; ui3->uFluxCapControl[63] = 0; 
	ui3->fFluxCapData[0] = 0.000000; ui3->fFluxCapData[1] = 0.000000; ui3->fFluxCapData[2] = 0.000000; ui3->fFluxCapData[3] = 0.000000; ui3->fFluxCapData[4] = 0.000000; ui3->fFluxCapData[5] = 0.000000; ui3->fFluxCapData[6] = 0.000000; ui3->fFluxCapData[7] = 0.000000; ui3->fFluxCapData[8] = 0.000000; ui3->fFluxCapData[9] = 0.000000; ui3->fFluxCapData[10] = 0.000000; ui3->fFluxCapData[11] = 0.000000; ui3->fFluxCapData[12] = 0.000000; ui3->fFluxCapData[13] = 0.000000; ui3->fFluxCapData[14] = 0.000000; ui3->fFluxCapData[15] = 0.000000; ui3->fFluxCapData[16] = 0.000000; ui3->fFluxCapData[17] = 0.000000; ui3->fFluxCapData[18] = 0.000000; ui3->fFluxCapData[19] = 0.000000; ui3->fFluxCapData[20] = 0.000000; ui3->fFluxCapData[21] = 0.000000; ui3->fFluxCapData[22] = 0.000000; ui3->fFluxCapData[23] = 0.000000; ui3->fFluxCapData[24] = 0.000000; ui3->fFluxCapData[25] = 0.000000; ui3->fFluxCapData[26] = 0.000000; ui3->fFluxCapData[27] = 0.000000; ui3->fFluxCapData[28] = 0.000000; ui3->fFluxCapData[29] = 0.000000; ui3->fFluxCapData[30] = 0.000000; ui3->fFluxCapData[31] = 0.000000; ui3->fFluxCapData[32] = 0.000000; ui3->fFluxCapData[33] = 0.000000; ui3->fFluxCapData[34] = 0.000000; ui3->fFluxCapData[35] = 0.000000; ui3->fFluxCapData[36] = 0.000000; ui3->fFluxCapData[37] = 0.000000; ui3->fFluxCapData[38] = 0.000000; ui3->fFluxCapData[39] = 0.000000; ui3->fFluxCapData[40] = 0.000000; ui3->fFluxCapData[41] = 0.000000; ui3->fFluxCapData[42] = 0.000000; ui3->fFluxCapData[43] = 0.000000; ui3->fFluxCapData[44] = 0.000000; ui3->fFluxCapData[45] = 0.000000; ui3->fFluxCapData[46] = 0.000000; ui3->fFluxCapData[47] = 0.000000; ui3->fFluxCapData[48] = 0.000000; ui3->fFluxCapData[49] = 0.000000; ui3->fFluxCapData[50] = 0.000000; ui3->fFluxCapData[51] = 0.000000; ui3->fFluxCapData[52] = 0.000000; ui3->fFluxCapData[53] = 0.000000; ui3->fFluxCapData[54] = 0.000000; ui3->fFluxCapData[55] = 0.000000; ui3->fFluxCapData[56] = 0.000000; ui3->fFluxCapData[57] = 0.000000; ui3->fFluxCapData[58] = 0.000000; ui3->fFluxCapData[59] = 0.000000; ui3->fFluxCapData[60] = 0.000000; ui3->fFluxCapData[61] = 0.000000; ui3->fFluxCapData[62] = 0.000000; ui3->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui3);
	delete ui3;


	m_fQ = 5.000000;
	CUICtrl* ui4 = new CUICtrl;
	ui4->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui4->uControlId = 4;
	ui4->bLogSlider = false;
	ui4->bExpSlider = false;
	ui4->fUserDisplayDataLoLimit = 0.500000;
	ui4->fUserDisplayDataHiLimit = 20.000000;
	ui4->uUserDataType = floatData;
	ui4->fInitUserIntValue = 0;
	ui4->fInitUserFloatValue = 5.000000;
	ui4->fInitUserDoubleValue = 0;
	ui4->fInitUserUINTValue = 0;
	ui4->m_pUserCookedIntData = NULL;
	ui4->m_pUserCookedFloatData = &m_fQ;
	ui4->m_pUserCookedDoubleData = NULL;
	ui4->m_pUserCookedUINTData = NULL;
	ui4->cControlUnits = "";
	ui4->cVariableName = "m_fQ";
	ui4->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui4->dPresetData[0] = 5.000000;ui4->dPresetData[1] = 0.000000;ui4->dPresetData[2] = 0.000000;ui4->dPresetData[3] = 0.000000;ui4->dPresetData[4] = 0.000000;ui4->dPresetData[5] = 0.000000;ui4->dPresetData[6] = 0.000000;ui4->dPresetData[7] = 0.000000;ui4->dPresetData[8] = 0.000000;ui4->dPresetData[9] = 0.000000;ui4->dPresetData[10] = 0.000000;ui4->dPresetData[11] = 0.000000;ui4->dPresetData[12] = 0.000000;ui4->dPresetData[13] = 0.000000;ui4->dPresetData[14] = 0.000000;ui4->dPresetData[15] = 0.000000;
	ui4->cControlName = "Q";
	ui4->bOwnerControl = false;
	ui4->bMIDIControl = false;
	ui4->uMIDIControlCommand = 176;
	ui4->uMIDIControlName = 3;
	ui4->uMIDIControlChannel = 0;
	ui4->nGUIRow = nIndexer++;
	ui4->nGUIColumn = -1;
	ui4->bEnableParamSmoothing = false;
	ui4->fSmoothingTimeInMs = 100.00;
	ui4->uControlTheme[0] = 0; ui4->uControlTheme[1] = 0; ui4->uControlTheme[2] = 0; ui4->uControlTheme[3] = 0; ui4->uControlTheme[4] = 0; ui4->uControlTheme[5] = 0; ui4->uControlTheme[6] = 0; ui4->uControlTheme[7] = 0; ui4->uControlTheme[8] = 0; ui4->uControlTheme[9] = 0; ui4->uControlTheme[10] = 0; ui4->uControlTheme[11] = 0; ui4->uControlTheme[12] = 0; ui4->uControlTheme[13] = 0; ui4->uControlTheme[14] = 0; ui4->uControlTheme[15] = 0; ui4->uControlTheme[16] = 2; ui4->uControlTheme[17] = 0; ui4->uControlTheme[18] = 0; ui4->uControlTheme[19] = 0; ui4->uControlTheme[20] = 0; ui4->uControlTheme[21] = 0; ui4->uControlTheme[22] = 0; ui4->uControlTheme[23] = 0; ui4->uControlTheme[24] = 0; ui4->uControlTheme[25] = 0; ui4->uControlTheme[26] = 0; ui4->uControlTheme[27] = 0; ui4->uControlTheme[28] = 0; ui4->uControlTheme[29] = 0; ui4->uControlTheme[30] = 0; ui4->uControlTheme[31] = 0; 
	ui4->uFluxCapControl[0] = 0; ui4->uFluxCapControl[1] = 0; ui4->uFluxCapControl[2] = 0; ui4->uFluxCapControl[3] = 0; ui4->uFluxCapControl[4] = 0; ui4->uFluxCapControl[5] = 0; ui4->uFluxCapControl[6] = 0; ui4->uFluxCapControl[7] = 0; ui4->uFluxCapControl[8] = 0; ui4->uFluxCapControl[9] = 0; ui4->uFluxCapControl[10] = 0; ui4->uFluxCapControl[11] = 0; ui4->uFluxCapControl[12] = 0; ui4->uFluxCapControl[13] = 0; ui4->uFluxCapControl[14] = 0; ui4->uFluxCapControl[15] = 0; ui4->uFluxCapControl[16] = 0; ui4->uFluxCapControl[17] = 0; ui4->uFluxCapControl[18] = 0; ui4->uFluxCapControl[19] = 0; ui4->uFluxCapControl[20] = 0; ui4->uFluxCapControl[21] = 0; ui4->uFluxCapControl[22] = 0; ui4->uFluxCapControl[23] = 0; ui4->uFluxCapControl[24] = 0; ui4->uFluxCapControl[25] = 0; ui4->uFluxCapControl[26] = 0; ui4->uFluxCapControl[27] = 0; ui4->uFluxCapControl[28] = 0; ui4->uFluxCapControl[29] = 0; ui4->uFluxCapControl[30] = 0; ui4->uFluxCapControl[31] = 0; ui4->uFluxCapControl[32] = 0; ui4->uFluxCapControl[33] = 0; ui4->uFluxCapControl[34] = 0; ui4->uFluxCapControl[35] = 0; ui4->uFluxCapControl[36] = 0; ui4->uFluxCapControl[37] = 0; ui4->uFluxCapControl[38] = 0; ui4->uFluxCapControl[39] = 0; ui4->uFluxCapControl[40] = 0; ui4->uFluxCapControl[41] = 0; ui4->uFluxCapControl[42] = 0; ui4->uFluxCapControl[43] = 0; ui4->uFluxCapControl[44] = 0; ui4->uFluxCapControl[45] = 0; ui4->uFluxCapControl[46] = 0; ui4->uFluxCapControl[47] = 0; ui4->uFluxCapControl[48] = 0; ui4->uFluxCapControl[49] = 0; ui4->uFluxCapControl[50] = 0; ui4->uFluxCapControl[51] = 0; ui4->uFluxCapControl[52] = 0; ui4->uFluxCapControl[53] = 0; ui4->uFluxCapControl[54] = 0; ui4->uFluxCapControl[55] = 0; ui4->uFluxCapControl[56] = 0; ui4->uFluxCapControl[57] = 0; ui4->uFluxCapControl[58] = 0; ui4->uFluxCapControl[59] = 0; ui4->uFluxCapControl[60] = 0; ui4->uFluxCapControl[61] = 0; ui4->uFluxCapControl[62] = 0; ui4->uFluxCapControl[63] = 0; 
	ui4->fFluxCapData[0] = 0.000000; ui4->fFluxCapData[1] = 0.000000; ui4->fFluxCapData[2] = 0.000000; ui4->fFluxCapData[3] = 0.000000; ui4->fFluxCapData[4] = 0.000000; ui4->fFluxCapData[5] = 0.000000; ui4->fFluxCapData[6] = 0.000000; ui4->fFluxCapData[7] = 0.000000; ui4->fFluxCapData[8] = 0.000000; ui4->fFluxCapData[9] = 0.000000; ui4->fFluxCapData[10] = 0.000000; ui4->fFluxCapData[11] = 0.000000; ui4->fFluxCapData[12] = 0.000000; ui4->fFluxCapData[13] = 0.000000; ui4->fFluxCapData[14] = 0.000000; ui4->fFluxCapData[15] = 0.000000; ui4->fFluxCapData[16] = 0.000000; ui4->fFluxCapData[17] = 0.000000; ui4->fFluxCapData[18] = 0.000000; ui4->fFluxCapData[19] = 0.000000; ui4->fFluxCapData[20] = 0.000000; ui4->fFluxCapData[21] = 0.000000; ui4->fFluxCapData[22] = 0.000000; ui4->fFluxCapData[23] = 0.000000; ui4->fFluxCapData[24] = 0.000000; ui4->fFluxCapData[25] = 0.000000; ui4->fFluxCapData[26] = 0.000000; ui4->fFluxCapData[27] = 0.000000; ui4->fFluxCapData[28] = 0.000000; ui4->fFluxCapData[29] = 0.000000; ui4->fFluxCapData[30] = 0.000000; ui4->fFluxCapData[31] = 0.000000; ui4->fFluxCapData[32] = 0.000000; ui4->fFluxCapData[33] = 0.000000; ui4->fFluxCapData[34] = 0.000000; ui4->fFluxCapData[35] = 0.000000; ui4->fFluxCapData[36] = 0.000000; ui4->fFluxCapData[37] = 0.000000; ui4->fFluxCapData[38] = 0.000000; ui4->fFluxCapData[39] = 0.000000; ui4->fFluxCapData[40] = 0.000000; ui4->fFluxCapData[41] = 0.000000; ui4->fFluxCapData[42] = 0.000000; ui4->fFluxCapData[43] = 0.000000; ui4->fFluxCapData[44] = 0.000000; ui4->fFluxCapData[45] = 0.000000; ui4->fFluxCapData[46] = 0.000000; ui4->fFluxCapData[47] = 0.000000; ui4->fFluxCapData[48] = 0.000000; ui4->fFluxCapData[49] = 0.000000; ui4->fFluxCapData[50] = 0.000000; ui4->fFluxCapData[51] = 0.000000; ui4->fFluxCapData[52] = 0.000000; ui4->fFluxCapData[53] = 0.000000; ui4->fFluxCapData[54] = 0.000000; ui4->fFluxCapData[55] = 0.000000; ui4->fFluxCapData[56] = 0.000000; ui4->fFluxCapData[57] = 0.000000; ui4->fFluxCapData[58] = 0.000000; ui4->fFluxCapData[59] = 0.000000; ui4->fFluxCapData[60] = 0.000000; ui4->fFluxCapData[61] = 0.000000; ui4->fFluxCapData[62] = 0.000000; ui4->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui4);
	delete ui4;


	m_Fc = 5000.000000;
	CUICtrl* ui5 = new CUICtrl;
	ui5->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui5->uControlId = 5;
	ui5->bLogSlider = false;
	ui5->bExpSlider = false;
	ui5->fUserDisplayDataLoLimit = 100.000000;
	ui5->fUserDisplayDataHiLimit = 10000.000000;
	ui5->uUserDataType = floatData;
	ui5->fInitUserIntValue = 0;
	ui5->fInitUserFloatValue = 5000.000000;
	ui5->fInitUserDoubleValue = 0;
	ui5->fInitUserUINTValue = 0;
	ui5->m_pUserCookedIntData = NULL;
	ui5->m_pUserCookedFloatData = &m_Fc;
	ui5->m_pUserCookedDoubleData = NULL;
	ui5->m_pUserCookedUINTData = NULL;
	ui5->cControlUnits = "Hz";
	ui5->cVariableName = "m_Fc";
	ui5->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui5->dPresetData[0] = 5000.000000;ui5->dPresetData[1] = 0.000000;ui5->dPresetData[2] = 0.000000;ui5->dPresetData[3] = 0.000000;ui5->dPresetData[4] = 0.000000;ui5->dPresetData[5] = 0.000000;ui5->dPresetData[6] = 0.000000;ui5->dPresetData[7] = 0.000000;ui5->dPresetData[8] = 0.000000;ui5->dPresetData[9] = 0.000000;ui5->dPresetData[10] = 0.000000;ui5->dPresetData[11] = 0.000000;ui5->dPresetData[12] = 0.000000;ui5->dPresetData[13] = 0.000000;ui5->dPresetData[14] = 0.000000;ui5->dPresetData[15] = 0.000000;
	ui5->cControlName = "Fc";
	ui5->bOwnerControl = false;
	ui5->bMIDIControl = false;
	ui5->uMIDIControlCommand = 176;
	ui5->uMIDIControlName = 3;
	ui5->uMIDIControlChannel = 0;
	ui5->nGUIRow = nIndexer++;
	ui5->nGUIColumn = -1;
	ui5->bEnableParamSmoothing = false;
	ui5->fSmoothingTimeInMs = 100.00;
	ui5->uControlTheme[0] = 0; ui5->uControlTheme[1] = 0; ui5->uControlTheme[2] = 0; ui5->uControlTheme[3] = 0; ui5->uControlTheme[4] = 0; ui5->uControlTheme[5] = 0; ui5->uControlTheme[6] = 0; ui5->uControlTheme[7] = 0; ui5->uControlTheme[8] = 0; ui5->uControlTheme[9] = 0; ui5->uControlTheme[10] = 0; ui5->uControlTheme[11] = 0; ui5->uControlTheme[12] = 0; ui5->uControlTheme[13] = 0; ui5->uControlTheme[14] = 0; ui5->uControlTheme[15] = 0; ui5->uControlTheme[16] = 2; ui5->uControlTheme[17] = 0; ui5->uControlTheme[18] = 0; ui5->uControlTheme[19] = 0; ui5->uControlTheme[20] = 0; ui5->uControlTheme[21] = 0; ui5->uControlTheme[22] = 0; ui5->uControlTheme[23] = 0; ui5->uControlTheme[24] = 0; ui5->uControlTheme[25] = 0; ui5->uControlTheme[26] = 0; ui5->uControlTheme[27] = 0; ui5->uControlTheme[28] = 0; ui5->uControlTheme[29] = 0; ui5->uControlTheme[30] = 0; ui5->uControlTheme[31] = 0; 
	ui5->uFluxCapControl[0] = 0; ui5->uFluxCapControl[1] = 0; ui5->uFluxCapControl[2] = 0; ui5->uFluxCapControl[3] = 0; ui5->uFluxCapControl[4] = 0; ui5->uFluxCapControl[5] = 0; ui5->uFluxCapControl[6] = 0; ui5->uFluxCapControl[7] = 0; ui5->uFluxCapControl[8] = 0; ui5->uFluxCapControl[9] = 0; ui5->uFluxCapControl[10] = 0; ui5->uFluxCapControl[11] = 0; ui5->uFluxCapControl[12] = 0; ui5->uFluxCapControl[13] = 0; ui5->uFluxCapControl[14] = 0; ui5->uFluxCapControl[15] = 0; ui5->uFluxCapControl[16] = 0; ui5->uFluxCapControl[17] = 0; ui5->uFluxCapControl[18] = 0; ui5->uFluxCapControl[19] = 0; ui5->uFluxCapControl[20] = 0; ui5->uFluxCapControl[21] = 0; ui5->uFluxCapControl[22] = 0; ui5->uFluxCapControl[23] = 0; ui5->uFluxCapControl[24] = 0; ui5->uFluxCapControl[25] = 0; ui5->uFluxCapControl[26] = 0; ui5->uFluxCapControl[27] = 0; ui5->uFluxCapControl[28] = 0; ui5->uFluxCapControl[29] = 0; ui5->uFluxCapControl[30] = 0; ui5->uFluxCapControl[31] = 0; ui5->uFluxCapControl[32] = 0; ui5->uFluxCapControl[33] = 0; ui5->uFluxCapControl[34] = 0; ui5->uFluxCapControl[35] = 0; ui5->uFluxCapControl[36] = 0; ui5->uFluxCapControl[37] = 0; ui5->uFluxCapControl[38] = 0; ui5->uFluxCapControl[39] = 0; ui5->uFluxCapControl[40] = 0; ui5->uFluxCapControl[41] = 0; ui5->uFluxCapControl[42] = 0; ui5->uFluxCapControl[43] = 0; ui5->uFluxCapControl[44] = 0; ui5->uFluxCapControl[45] = 0; ui5->uFluxCapControl[46] = 0; ui5->uFluxCapControl[47] = 0; ui5->uFluxCapControl[48] = 0; ui5->uFluxCapControl[49] = 0; ui5->uFluxCapControl[50] = 0; ui5->uFluxCapControl[51] = 0; ui5->uFluxCapControl[52] = 0; ui5->uFluxCapControl[53] = 0; ui5->uFluxCapControl[54] = 0; ui5->uFluxCapControl[55] = 0; ui5->uFluxCapControl[56] = 0; ui5->uFluxCapControl[57] = 0; ui5->uFluxCapControl[58] = 0; ui5->uFluxCapControl[59] = 0; ui5->uFluxCapControl[60] = 0; ui5->uFluxCapControl[61] = 0; ui5->uFluxCapControl[62] = 0; ui5->uFluxCapControl[63] = 0; 
	ui5->fFluxCapData[0] = 0.000000; ui5->fFluxCapData[1] = 0.000000; ui5->fFluxCapData[2] = 0.000000; ui5->fFluxCapData[3] = 0.000000; ui5->fFluxCapData[4] = 0.000000; ui5->fFluxCapData[5] = 0.000000; ui5->fFluxCapData[6] = 0.000000; ui5->fFluxCapData[7] = 0.000000; ui5->fFluxCapData[8] = 0.000000; ui5->fFluxCapData[9] = 0.000000; ui5->fFluxCapData[10] = 0.000000; ui5->fFluxCapData[11] = 0.000000; ui5->fFluxCapData[12] = 0.000000; ui5->fFluxCapData[13] = 0.000000; ui5->fFluxCapData[14] = 0.000000; ui5->fFluxCapData[15] = 0.000000; ui5->fFluxCapData[16] = 0.000000; ui5->fFluxCapData[17] = 0.000000; ui5->fFluxCapData[18] = 0.000000; ui5->fFluxCapData[19] = 0.000000; ui5->fFluxCapData[20] = 0.000000; ui5->fFluxCapData[21] = 0.000000; ui5->fFluxCapData[22] = 0.000000; ui5->fFluxCapData[23] = 0.000000; ui5->fFluxCapData[24] = 0.000000; ui5->fFluxCapData[25] = 0.000000; ui5->fFluxCapData[26] = 0.000000; ui5->fFluxCapData[27] = 0.000000; ui5->fFluxCapData[28] = 0.000000; ui5->fFluxCapData[29] = 0.000000; ui5->fFluxCapData[30] = 0.000000; ui5->fFluxCapData[31] = 0.000000; ui5->fFluxCapData[32] = 0.000000; ui5->fFluxCapData[33] = 0.000000; ui5->fFluxCapData[34] = 0.000000; ui5->fFluxCapData[35] = 0.000000; ui5->fFluxCapData[36] = 0.000000; ui5->fFluxCapData[37] = 0.000000; ui5->fFluxCapData[38] = 0.000000; ui5->fFluxCapData[39] = 0.000000; ui5->fFluxCapData[40] = 0.000000; ui5->fFluxCapData[41] = 0.000000; ui5->fFluxCapData[42] = 0.000000; ui5->fFluxCapData[43] = 0.000000; ui5->fFluxCapData[44] = 0.000000; ui5->fFluxCapData[45] = 0.000000; ui5->fFluxCapData[46] = 0.000000; ui5->fFluxCapData[47] = 0.000000; ui5->fFluxCapData[48] = 0.000000; ui5->fFluxCapData[49] = 0.000000; ui5->fFluxCapData[50] = 0.000000; ui5->fFluxCapData[51] = 0.000000; ui5->fFluxCapData[52] = 0.000000; ui5->fFluxCapData[53] = 0.000000; ui5->fFluxCapData[54] = 0.000000; ui5->fFluxCapData[55] = 0.000000; ui5->fFluxCapData[56] = 0.000000; ui5->fFluxCapData[57] = 0.000000; ui5->fFluxCapData[58] = 0.000000; ui5->fFluxCapData[59] = 0.000000; ui5->fFluxCapData[60] = 0.000000; ui5->fFluxCapData[61] = 0.000000; ui5->fFluxCapData[62] = 0.000000; ui5->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui5);
	delete ui5;


	m_f_maxGain = 5.000000;
	CUICtrl* ui6 = new CUICtrl;
	ui6->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui6->uControlId = 6;
	ui6->bLogSlider = false;
	ui6->bExpSlider = false;
	ui6->fUserDisplayDataLoLimit = 0.000000;
	ui6->fUserDisplayDataHiLimit = 20.000000;
	ui6->uUserDataType = floatData;
	ui6->fInitUserIntValue = 0;
	ui6->fInitUserFloatValue = 5.000000;
	ui6->fInitUserDoubleValue = 0;
	ui6->fInitUserUINTValue = 0;
	ui6->m_pUserCookedIntData = NULL;
	ui6->m_pUserCookedFloatData = &m_f_maxGain;
	ui6->m_pUserCookedDoubleData = NULL;
	ui6->m_pUserCookedUINTData = NULL;
	ui6->cControlUnits = "dB";
	ui6->cVariableName = "m_f_maxGain";
	ui6->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui6->dPresetData[0] = 5.000000;ui6->dPresetData[1] = 0.000000;ui6->dPresetData[2] = 0.000000;ui6->dPresetData[3] = 0.000000;ui6->dPresetData[4] = 0.000000;ui6->dPresetData[5] = 0.000000;ui6->dPresetData[6] = 0.000000;ui6->dPresetData[7] = 0.000000;ui6->dPresetData[8] = 0.000000;ui6->dPresetData[9] = 0.000000;ui6->dPresetData[10] = 0.000000;ui6->dPresetData[11] = 0.000000;ui6->dPresetData[12] = 0.000000;ui6->dPresetData[13] = 0.000000;ui6->dPresetData[14] = 0.000000;ui6->dPresetData[15] = 0.000000;
	ui6->cControlName = "Gain Max";
	ui6->bOwnerControl = false;
	ui6->bMIDIControl = false;
	ui6->uMIDIControlCommand = 176;
	ui6->uMIDIControlName = 3;
	ui6->uMIDIControlChannel = 0;
	ui6->nGUIRow = nIndexer++;
	ui6->nGUIColumn = -1;
	ui6->bEnableParamSmoothing = false;
	ui6->fSmoothingTimeInMs = 100.00;
	ui6->uControlTheme[0] = 0; ui6->uControlTheme[1] = 0; ui6->uControlTheme[2] = 0; ui6->uControlTheme[3] = 0; ui6->uControlTheme[4] = 0; ui6->uControlTheme[5] = 0; ui6->uControlTheme[6] = 0; ui6->uControlTheme[7] = 0; ui6->uControlTheme[8] = 0; ui6->uControlTheme[9] = 0; ui6->uControlTheme[10] = 0; ui6->uControlTheme[11] = 0; ui6->uControlTheme[12] = 0; ui6->uControlTheme[13] = 0; ui6->uControlTheme[14] = 0; ui6->uControlTheme[15] = 0; ui6->uControlTheme[16] = 2; ui6->uControlTheme[17] = 0; ui6->uControlTheme[18] = 0; ui6->uControlTheme[19] = 0; ui6->uControlTheme[20] = 0; ui6->uControlTheme[21] = 0; ui6->uControlTheme[22] = 0; ui6->uControlTheme[23] = 0; ui6->uControlTheme[24] = 0; ui6->uControlTheme[25] = 0; ui6->uControlTheme[26] = 0; ui6->uControlTheme[27] = 0; ui6->uControlTheme[28] = 0; ui6->uControlTheme[29] = 0; ui6->uControlTheme[30] = 0; ui6->uControlTheme[31] = 0; 
	ui6->uFluxCapControl[0] = 0; ui6->uFluxCapControl[1] = 0; ui6->uFluxCapControl[2] = 0; ui6->uFluxCapControl[3] = 0; ui6->uFluxCapControl[4] = 0; ui6->uFluxCapControl[5] = 0; ui6->uFluxCapControl[6] = 0; ui6->uFluxCapControl[7] = 0; ui6->uFluxCapControl[8] = 0; ui6->uFluxCapControl[9] = 0; ui6->uFluxCapControl[10] = 0; ui6->uFluxCapControl[11] = 0; ui6->uFluxCapControl[12] = 0; ui6->uFluxCapControl[13] = 0; ui6->uFluxCapControl[14] = 0; ui6->uFluxCapControl[15] = 0; ui6->uFluxCapControl[16] = 0; ui6->uFluxCapControl[17] = 0; ui6->uFluxCapControl[18] = 0; ui6->uFluxCapControl[19] = 0; ui6->uFluxCapControl[20] = 0; ui6->uFluxCapControl[21] = 0; ui6->uFluxCapControl[22] = 0; ui6->uFluxCapControl[23] = 0; ui6->uFluxCapControl[24] = 0; ui6->uFluxCapControl[25] = 0; ui6->uFluxCapControl[26] = 0; ui6->uFluxCapControl[27] = 0; ui6->uFluxCapControl[28] = 0; ui6->uFluxCapControl[29] = 0; ui6->uFluxCapControl[30] = 0; ui6->uFluxCapControl[31] = 0; ui6->uFluxCapControl[32] = 0; ui6->uFluxCapControl[33] = 0; ui6->uFluxCapControl[34] = 0; ui6->uFluxCapControl[35] = 0; ui6->uFluxCapControl[36] = 0; ui6->uFluxCapControl[37] = 0; ui6->uFluxCapControl[38] = 0; ui6->uFluxCapControl[39] = 0; ui6->uFluxCapControl[40] = 0; ui6->uFluxCapControl[41] = 0; ui6->uFluxCapControl[42] = 0; ui6->uFluxCapControl[43] = 0; ui6->uFluxCapControl[44] = 0; ui6->uFluxCapControl[45] = 0; ui6->uFluxCapControl[46] = 0; ui6->uFluxCapControl[47] = 0; ui6->uFluxCapControl[48] = 0; ui6->uFluxCapControl[49] = 0; ui6->uFluxCapControl[50] = 0; ui6->uFluxCapControl[51] = 0; ui6->uFluxCapControl[52] = 0; ui6->uFluxCapControl[53] = 0; ui6->uFluxCapControl[54] = 0; ui6->uFluxCapControl[55] = 0; ui6->uFluxCapControl[56] = 0; ui6->uFluxCapControl[57] = 0; ui6->uFluxCapControl[58] = 0; ui6->uFluxCapControl[59] = 0; ui6->uFluxCapControl[60] = 0; ui6->uFluxCapControl[61] = 0; ui6->uFluxCapControl[62] = 0; ui6->uFluxCapControl[63] = 0; 
	ui6->fFluxCapData[0] = 0.000000; ui6->fFluxCapData[1] = 0.000000; ui6->fFluxCapData[2] = 0.000000; ui6->fFluxCapData[3] = 0.000000; ui6->fFluxCapData[4] = 0.000000; ui6->fFluxCapData[5] = 0.000000; ui6->fFluxCapData[6] = 0.000000; ui6->fFluxCapData[7] = 0.000000; ui6->fFluxCapData[8] = 0.000000; ui6->fFluxCapData[9] = 0.000000; ui6->fFluxCapData[10] = 0.000000; ui6->fFluxCapData[11] = 0.000000; ui6->fFluxCapData[12] = 0.000000; ui6->fFluxCapData[13] = 0.000000; ui6->fFluxCapData[14] = 0.000000; ui6->fFluxCapData[15] = 0.000000; ui6->fFluxCapData[16] = 0.000000; ui6->fFluxCapData[17] = 0.000000; ui6->fFluxCapData[18] = 0.000000; ui6->fFluxCapData[19] = 0.000000; ui6->fFluxCapData[20] = 0.000000; ui6->fFluxCapData[21] = 0.000000; ui6->fFluxCapData[22] = 0.000000; ui6->fFluxCapData[23] = 0.000000; ui6->fFluxCapData[24] = 0.000000; ui6->fFluxCapData[25] = 0.000000; ui6->fFluxCapData[26] = 0.000000; ui6->fFluxCapData[27] = 0.000000; ui6->fFluxCapData[28] = 0.000000; ui6->fFluxCapData[29] = 0.000000; ui6->fFluxCapData[30] = 0.000000; ui6->fFluxCapData[31] = 0.000000; ui6->fFluxCapData[32] = 0.000000; ui6->fFluxCapData[33] = 0.000000; ui6->fFluxCapData[34] = 0.000000; ui6->fFluxCapData[35] = 0.000000; ui6->fFluxCapData[36] = 0.000000; ui6->fFluxCapData[37] = 0.000000; ui6->fFluxCapData[38] = 0.000000; ui6->fFluxCapData[39] = 0.000000; ui6->fFluxCapData[40] = 0.000000; ui6->fFluxCapData[41] = 0.000000; ui6->fFluxCapData[42] = 0.000000; ui6->fFluxCapData[43] = 0.000000; ui6->fFluxCapData[44] = 0.000000; ui6->fFluxCapData[45] = 0.000000; ui6->fFluxCapData[46] = 0.000000; ui6->fFluxCapData[47] = 0.000000; ui6->fFluxCapData[48] = 0.000000; ui6->fFluxCapData[49] = 0.000000; ui6->fFluxCapData[50] = 0.000000; ui6->fFluxCapData[51] = 0.000000; ui6->fFluxCapData[52] = 0.000000; ui6->fFluxCapData[53] = 0.000000; ui6->fFluxCapData[54] = 0.000000; ui6->fFluxCapData[55] = 0.000000; ui6->fFluxCapData[56] = 0.000000; ui6->fFluxCapData[57] = 0.000000; ui6->fFluxCapData[58] = 0.000000; ui6->fFluxCapData[59] = 0.000000; ui6->fFluxCapData[60] = 0.000000; ui6->fFluxCapData[61] = 0.000000; ui6->fFluxCapData[62] = 0.000000; ui6->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui6);
	delete ui6;


	m_fPEQ_Fc = 1000.000000;
	CUICtrl* ui7 = new CUICtrl;
	ui7->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui7->uControlId = 7;
	ui7->bLogSlider = false;
	ui7->bExpSlider = true;
	ui7->fUserDisplayDataLoLimit = 100.000000;
	ui7->fUserDisplayDataHiLimit = 10000.000000;
	ui7->uUserDataType = floatData;
	ui7->fInitUserIntValue = 0;
	ui7->fInitUserFloatValue = 1000.000000;
	ui7->fInitUserDoubleValue = 0;
	ui7->fInitUserUINTValue = 0;
	ui7->m_pUserCookedIntData = NULL;
	ui7->m_pUserCookedFloatData = &m_fPEQ_Fc;
	ui7->m_pUserCookedDoubleData = NULL;
	ui7->m_pUserCookedUINTData = NULL;
	ui7->cControlUnits = "Hz";
	ui7->cVariableName = "m_fPEQ_Fc";
	ui7->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui7->dPresetData[0] = 1000.000000;ui7->dPresetData[1] = 0.000000;ui7->dPresetData[2] = 0.000000;ui7->dPresetData[3] = 0.000000;ui7->dPresetData[4] = 0.000000;ui7->dPresetData[5] = 0.000000;ui7->dPresetData[6] = 0.000000;ui7->dPresetData[7] = 0.000000;ui7->dPresetData[8] = 0.000000;ui7->dPresetData[9] = 0.000000;ui7->dPresetData[10] = 0.000000;ui7->dPresetData[11] = 0.000000;ui7->dPresetData[12] = 0.000000;ui7->dPresetData[13] = 0.000000;ui7->dPresetData[14] = 0.000000;ui7->dPresetData[15] = 0.000000;
	ui7->cControlName = "PEQ Fc";
	ui7->bOwnerControl = false;
	ui7->bMIDIControl = false;
	ui7->uMIDIControlCommand = 176;
	ui7->uMIDIControlName = 3;
	ui7->uMIDIControlChannel = 0;
	ui7->nGUIRow = nIndexer++;
	ui7->nGUIColumn = -1;
	ui7->bEnableParamSmoothing = false;
	ui7->fSmoothingTimeInMs = 100.00;
	ui7->uControlTheme[0] = 0; ui7->uControlTheme[1] = 0; ui7->uControlTheme[2] = 0; ui7->uControlTheme[3] = 0; ui7->uControlTheme[4] = 0; ui7->uControlTheme[5] = 0; ui7->uControlTheme[6] = 0; ui7->uControlTheme[7] = 0; ui7->uControlTheme[8] = 2; ui7->uControlTheme[9] = 0; ui7->uControlTheme[10] = 0; ui7->uControlTheme[11] = 0; ui7->uControlTheme[12] = 0; ui7->uControlTheme[13] = 0; ui7->uControlTheme[14] = 0; ui7->uControlTheme[15] = 0; ui7->uControlTheme[16] = 2; ui7->uControlTheme[17] = 0; ui7->uControlTheme[18] = 0; ui7->uControlTheme[19] = 0; ui7->uControlTheme[20] = 0; ui7->uControlTheme[21] = 0; ui7->uControlTheme[22] = 0; ui7->uControlTheme[23] = 0; ui7->uControlTheme[24] = 0; ui7->uControlTheme[25] = 0; ui7->uControlTheme[26] = 0; ui7->uControlTheme[27] = 0; ui7->uControlTheme[28] = 0; ui7->uControlTheme[29] = 0; ui7->uControlTheme[30] = 0; ui7->uControlTheme[31] = 0; 
	ui7->uFluxCapControl[0] = 0; ui7->uFluxCapControl[1] = 0; ui7->uFluxCapControl[2] = 0; ui7->uFluxCapControl[3] = 0; ui7->uFluxCapControl[4] = 0; ui7->uFluxCapControl[5] = 0; ui7->uFluxCapControl[6] = 0; ui7->uFluxCapControl[7] = 0; ui7->uFluxCapControl[8] = 0; ui7->uFluxCapControl[9] = 0; ui7->uFluxCapControl[10] = 0; ui7->uFluxCapControl[11] = 0; ui7->uFluxCapControl[12] = 0; ui7->uFluxCapControl[13] = 0; ui7->uFluxCapControl[14] = 0; ui7->uFluxCapControl[15] = 0; ui7->uFluxCapControl[16] = 0; ui7->uFluxCapControl[17] = 0; ui7->uFluxCapControl[18] = 0; ui7->uFluxCapControl[19] = 0; ui7->uFluxCapControl[20] = 0; ui7->uFluxCapControl[21] = 0; ui7->uFluxCapControl[22] = 0; ui7->uFluxCapControl[23] = 0; ui7->uFluxCapControl[24] = 0; ui7->uFluxCapControl[25] = 0; ui7->uFluxCapControl[26] = 0; ui7->uFluxCapControl[27] = 0; ui7->uFluxCapControl[28] = 0; ui7->uFluxCapControl[29] = 0; ui7->uFluxCapControl[30] = 0; ui7->uFluxCapControl[31] = 0; ui7->uFluxCapControl[32] = 0; ui7->uFluxCapControl[33] = 0; ui7->uFluxCapControl[34] = 0; ui7->uFluxCapControl[35] = 0; ui7->uFluxCapControl[36] = 0; ui7->uFluxCapControl[37] = 0; ui7->uFluxCapControl[38] = 0; ui7->uFluxCapControl[39] = 0; ui7->uFluxCapControl[40] = 0; ui7->uFluxCapControl[41] = 0; ui7->uFluxCapControl[42] = 0; ui7->uFluxCapControl[43] = 0; ui7->uFluxCapControl[44] = 0; ui7->uFluxCapControl[45] = 0; ui7->uFluxCapControl[46] = 0; ui7->uFluxCapControl[47] = 0; ui7->uFluxCapControl[48] = 0; ui7->uFluxCapControl[49] = 0; ui7->uFluxCapControl[50] = 0; ui7->uFluxCapControl[51] = 0; ui7->uFluxCapControl[52] = 0; ui7->uFluxCapControl[53] = 0; ui7->uFluxCapControl[54] = 0; ui7->uFluxCapControl[55] = 0; ui7->uFluxCapControl[56] = 0; ui7->uFluxCapControl[57] = 0; ui7->uFluxCapControl[58] = 0; ui7->uFluxCapControl[59] = 0; ui7->uFluxCapControl[60] = 0; ui7->uFluxCapControl[61] = 0; ui7->uFluxCapControl[62] = 0; ui7->uFluxCapControl[63] = 0; 
	ui7->fFluxCapData[0] = 0.000000; ui7->fFluxCapData[1] = 0.000000; ui7->fFluxCapData[2] = 0.000000; ui7->fFluxCapData[3] = 0.000000; ui7->fFluxCapData[4] = 0.000000; ui7->fFluxCapData[5] = 0.000000; ui7->fFluxCapData[6] = 0.000000; ui7->fFluxCapData[7] = 0.000000; ui7->fFluxCapData[8] = 0.000000; ui7->fFluxCapData[9] = 0.000000; ui7->fFluxCapData[10] = 0.000000; ui7->fFluxCapData[11] = 0.000000; ui7->fFluxCapData[12] = 0.000000; ui7->fFluxCapData[13] = 0.000000; ui7->fFluxCapData[14] = 0.000000; ui7->fFluxCapData[15] = 0.000000; ui7->fFluxCapData[16] = 0.000000; ui7->fFluxCapData[17] = 0.000000; ui7->fFluxCapData[18] = 0.000000; ui7->fFluxCapData[19] = 0.000000; ui7->fFluxCapData[20] = 0.000000; ui7->fFluxCapData[21] = 0.000000; ui7->fFluxCapData[22] = 0.000000; ui7->fFluxCapData[23] = 0.000000; ui7->fFluxCapData[24] = 0.000000; ui7->fFluxCapData[25] = 0.000000; ui7->fFluxCapData[26] = 0.000000; ui7->fFluxCapData[27] = 0.000000; ui7->fFluxCapData[28] = 0.000000; ui7->fFluxCapData[29] = 0.000000; ui7->fFluxCapData[30] = 0.000000; ui7->fFluxCapData[31] = 0.000000; ui7->fFluxCapData[32] = 0.000000; ui7->fFluxCapData[33] = 0.000000; ui7->fFluxCapData[34] = 0.000000; ui7->fFluxCapData[35] = 0.000000; ui7->fFluxCapData[36] = 0.000000; ui7->fFluxCapData[37] = 0.000000; ui7->fFluxCapData[38] = 0.000000; ui7->fFluxCapData[39] = 0.000000; ui7->fFluxCapData[40] = 0.000000; ui7->fFluxCapData[41] = 0.000000; ui7->fFluxCapData[42] = 0.000000; ui7->fFluxCapData[43] = 0.000000; ui7->fFluxCapData[44] = 0.000000; ui7->fFluxCapData[45] = 0.000000; ui7->fFluxCapData[46] = 0.000000; ui7->fFluxCapData[47] = 0.000000; ui7->fFluxCapData[48] = 0.000000; ui7->fFluxCapData[49] = 0.000000; ui7->fFluxCapData[50] = 0.000000; ui7->fFluxCapData[51] = 0.000000; ui7->fFluxCapData[52] = 0.000000; ui7->fFluxCapData[53] = 0.000000; ui7->fFluxCapData[54] = 0.000000; ui7->fFluxCapData[55] = 0.000000; ui7->fFluxCapData[56] = 0.000000; ui7->fFluxCapData[57] = 0.000000; ui7->fFluxCapData[58] = 0.000000; ui7->fFluxCapData[59] = 0.000000; ui7->fFluxCapData[60] = 0.000000; ui7->fFluxCapData[61] = 0.000000; ui7->fFluxCapData[62] = 0.000000; ui7->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui7);
	delete ui7;


	m_fPEQ_Q = 5.000000;
	CUICtrl* ui8 = new CUICtrl;
	ui8->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui8->uControlId = 8;
	ui8->bLogSlider = false;
	ui8->bExpSlider = false;
	ui8->fUserDisplayDataLoLimit = 0.200000;
	ui8->fUserDisplayDataHiLimit = 20.000000;
	ui8->uUserDataType = floatData;
	ui8->fInitUserIntValue = 0;
	ui8->fInitUserFloatValue = 5.000000;
	ui8->fInitUserDoubleValue = 0;
	ui8->fInitUserUINTValue = 0;
	ui8->m_pUserCookedIntData = NULL;
	ui8->m_pUserCookedFloatData = &m_fPEQ_Q;
	ui8->m_pUserCookedDoubleData = NULL;
	ui8->m_pUserCookedUINTData = NULL;
	ui8->cControlUnits = "";
	ui8->cVariableName = "m_fPEQ_Q";
	ui8->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui8->dPresetData[0] = 5.000000;ui8->dPresetData[1] = 0.000000;ui8->dPresetData[2] = 0.000000;ui8->dPresetData[3] = 0.000000;ui8->dPresetData[4] = 0.000000;ui8->dPresetData[5] = 0.000000;ui8->dPresetData[6] = 0.000000;ui8->dPresetData[7] = 0.000000;ui8->dPresetData[8] = 0.000000;ui8->dPresetData[9] = 0.000000;ui8->dPresetData[10] = 0.000000;ui8->dPresetData[11] = 0.000000;ui8->dPresetData[12] = 0.000000;ui8->dPresetData[13] = 0.000000;ui8->dPresetData[14] = 0.000000;ui8->dPresetData[15] = 0.000000;
	ui8->cControlName = "PEQ Q";
	ui8->bOwnerControl = false;
	ui8->bMIDIControl = false;
	ui8->uMIDIControlCommand = 176;
	ui8->uMIDIControlName = 3;
	ui8->uMIDIControlChannel = 0;
	ui8->nGUIRow = nIndexer++;
	ui8->nGUIColumn = -1;
	ui8->bEnableParamSmoothing = false;
	ui8->fSmoothingTimeInMs = 100.00;
	ui8->uControlTheme[0] = 0; ui8->uControlTheme[1] = 0; ui8->uControlTheme[2] = 0; ui8->uControlTheme[3] = 0; ui8->uControlTheme[4] = 0; ui8->uControlTheme[5] = 0; ui8->uControlTheme[6] = 0; ui8->uControlTheme[7] = 0; ui8->uControlTheme[8] = 2; ui8->uControlTheme[9] = 0; ui8->uControlTheme[10] = 0; ui8->uControlTheme[11] = 0; ui8->uControlTheme[12] = 0; ui8->uControlTheme[13] = 0; ui8->uControlTheme[14] = 0; ui8->uControlTheme[15] = 0; ui8->uControlTheme[16] = 2; ui8->uControlTheme[17] = 0; ui8->uControlTheme[18] = 0; ui8->uControlTheme[19] = 0; ui8->uControlTheme[20] = 0; ui8->uControlTheme[21] = 0; ui8->uControlTheme[22] = 0; ui8->uControlTheme[23] = 0; ui8->uControlTheme[24] = 0; ui8->uControlTheme[25] = 0; ui8->uControlTheme[26] = 0; ui8->uControlTheme[27] = 0; ui8->uControlTheme[28] = 0; ui8->uControlTheme[29] = 0; ui8->uControlTheme[30] = 0; ui8->uControlTheme[31] = 0; 
	ui8->uFluxCapControl[0] = 0; ui8->uFluxCapControl[1] = 0; ui8->uFluxCapControl[2] = 0; ui8->uFluxCapControl[3] = 0; ui8->uFluxCapControl[4] = 0; ui8->uFluxCapControl[5] = 0; ui8->uFluxCapControl[6] = 0; ui8->uFluxCapControl[7] = 0; ui8->uFluxCapControl[8] = 0; ui8->uFluxCapControl[9] = 0; ui8->uFluxCapControl[10] = 0; ui8->uFluxCapControl[11] = 0; ui8->uFluxCapControl[12] = 0; ui8->uFluxCapControl[13] = 0; ui8->uFluxCapControl[14] = 0; ui8->uFluxCapControl[15] = 0; ui8->uFluxCapControl[16] = 0; ui8->uFluxCapControl[17] = 0; ui8->uFluxCapControl[18] = 0; ui8->uFluxCapControl[19] = 0; ui8->uFluxCapControl[20] = 0; ui8->uFluxCapControl[21] = 0; ui8->uFluxCapControl[22] = 0; ui8->uFluxCapControl[23] = 0; ui8->uFluxCapControl[24] = 0; ui8->uFluxCapControl[25] = 0; ui8->uFluxCapControl[26] = 0; ui8->uFluxCapControl[27] = 0; ui8->uFluxCapControl[28] = 0; ui8->uFluxCapControl[29] = 0; ui8->uFluxCapControl[30] = 0; ui8->uFluxCapControl[31] = 0; ui8->uFluxCapControl[32] = 0; ui8->uFluxCapControl[33] = 0; ui8->uFluxCapControl[34] = 0; ui8->uFluxCapControl[35] = 0; ui8->uFluxCapControl[36] = 0; ui8->uFluxCapControl[37] = 0; ui8->uFluxCapControl[38] = 0; ui8->uFluxCapControl[39] = 0; ui8->uFluxCapControl[40] = 0; ui8->uFluxCapControl[41] = 0; ui8->uFluxCapControl[42] = 0; ui8->uFluxCapControl[43] = 0; ui8->uFluxCapControl[44] = 0; ui8->uFluxCapControl[45] = 0; ui8->uFluxCapControl[46] = 0; ui8->uFluxCapControl[47] = 0; ui8->uFluxCapControl[48] = 0; ui8->uFluxCapControl[49] = 0; ui8->uFluxCapControl[50] = 0; ui8->uFluxCapControl[51] = 0; ui8->uFluxCapControl[52] = 0; ui8->uFluxCapControl[53] = 0; ui8->uFluxCapControl[54] = 0; ui8->uFluxCapControl[55] = 0; ui8->uFluxCapControl[56] = 0; ui8->uFluxCapControl[57] = 0; ui8->uFluxCapControl[58] = 0; ui8->uFluxCapControl[59] = 0; ui8->uFluxCapControl[60] = 0; ui8->uFluxCapControl[61] = 0; ui8->uFluxCapControl[62] = 0; ui8->uFluxCapControl[63] = 0; 
	ui8->fFluxCapData[0] = 0.000000; ui8->fFluxCapData[1] = 0.000000; ui8->fFluxCapData[2] = 0.000000; ui8->fFluxCapData[3] = 0.000000; ui8->fFluxCapData[4] = 0.000000; ui8->fFluxCapData[5] = 0.000000; ui8->fFluxCapData[6] = 0.000000; ui8->fFluxCapData[7] = 0.000000; ui8->fFluxCapData[8] = 0.000000; ui8->fFluxCapData[9] = 0.000000; ui8->fFluxCapData[10] = 0.000000; ui8->fFluxCapData[11] = 0.000000; ui8->fFluxCapData[12] = 0.000000; ui8->fFluxCapData[13] = 0.000000; ui8->fFluxCapData[14] = 0.000000; ui8->fFluxCapData[15] = 0.000000; ui8->fFluxCapData[16] = 0.000000; ui8->fFluxCapData[17] = 0.000000; ui8->fFluxCapData[18] = 0.000000; ui8->fFluxCapData[19] = 0.000000; ui8->fFluxCapData[20] = 0.000000; ui8->fFluxCapData[21] = 0.000000; ui8->fFluxCapData[22] = 0.000000; ui8->fFluxCapData[23] = 0.000000; ui8->fFluxCapData[24] = 0.000000; ui8->fFluxCapData[25] = 0.000000; ui8->fFluxCapData[26] = 0.000000; ui8->fFluxCapData[27] = 0.000000; ui8->fFluxCapData[28] = 0.000000; ui8->fFluxCapData[29] = 0.000000; ui8->fFluxCapData[30] = 0.000000; ui8->fFluxCapData[31] = 0.000000; ui8->fFluxCapData[32] = 0.000000; ui8->fFluxCapData[33] = 0.000000; ui8->fFluxCapData[34] = 0.000000; ui8->fFluxCapData[35] = 0.000000; ui8->fFluxCapData[36] = 0.000000; ui8->fFluxCapData[37] = 0.000000; ui8->fFluxCapData[38] = 0.000000; ui8->fFluxCapData[39] = 0.000000; ui8->fFluxCapData[40] = 0.000000; ui8->fFluxCapData[41] = 0.000000; ui8->fFluxCapData[42] = 0.000000; ui8->fFluxCapData[43] = 0.000000; ui8->fFluxCapData[44] = 0.000000; ui8->fFluxCapData[45] = 0.000000; ui8->fFluxCapData[46] = 0.000000; ui8->fFluxCapData[47] = 0.000000; ui8->fFluxCapData[48] = 0.000000; ui8->fFluxCapData[49] = 0.000000; ui8->fFluxCapData[50] = 0.000000; ui8->fFluxCapData[51] = 0.000000; ui8->fFluxCapData[52] = 0.000000; ui8->fFluxCapData[53] = 0.000000; ui8->fFluxCapData[54] = 0.000000; ui8->fFluxCapData[55] = 0.000000; ui8->fFluxCapData[56] = 0.000000; ui8->fFluxCapData[57] = 0.000000; ui8->fFluxCapData[58] = 0.000000; ui8->fFluxCapData[59] = 0.000000; ui8->fFluxCapData[60] = 0.000000; ui8->fFluxCapData[61] = 0.000000; ui8->fFluxCapData[62] = 0.000000; ui8->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui8);
	delete ui8;


	m_fPEQ_Gain_dB = 0.000000;
	CUICtrl* ui9 = new CUICtrl;
	ui9->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui9->uControlId = 9;
	ui9->bLogSlider = false;
	ui9->bExpSlider = false;
	ui9->fUserDisplayDataLoLimit = -20.000000;
	ui9->fUserDisplayDataHiLimit = 20.000000;
	ui9->uUserDataType = floatData;
	ui9->fInitUserIntValue = 0;
	ui9->fInitUserFloatValue = 0.000000;
	ui9->fInitUserDoubleValue = 0;
	ui9->fInitUserUINTValue = 0;
	ui9->m_pUserCookedIntData = NULL;
	ui9->m_pUserCookedFloatData = &m_fPEQ_Gain_dB;
	ui9->m_pUserCookedDoubleData = NULL;
	ui9->m_pUserCookedUINTData = NULL;
	ui9->cControlUnits = "dB";
	ui9->cVariableName = "m_fPEQ_Gain_dB";
	ui9->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui9->dPresetData[0] = 0.000000;ui9->dPresetData[1] = 0.000000;ui9->dPresetData[2] = 0.000000;ui9->dPresetData[3] = 0.000000;ui9->dPresetData[4] = 0.000000;ui9->dPresetData[5] = 0.000000;ui9->dPresetData[6] = 0.000000;ui9->dPresetData[7] = 0.000000;ui9->dPresetData[8] = 0.000000;ui9->dPresetData[9] = 0.000000;ui9->dPresetData[10] = 0.000000;ui9->dPresetData[11] = 0.000000;ui9->dPresetData[12] = 0.000000;ui9->dPresetData[13] = 0.000000;ui9->dPresetData[14] = 0.000000;ui9->dPresetData[15] = 0.000000;
	ui9->cControlName = "PEQ Gain";
	ui9->bOwnerControl = false;
	ui9->bMIDIControl = false;
	ui9->uMIDIControlCommand = 176;
	ui9->uMIDIControlName = 3;
	ui9->uMIDIControlChannel = 0;
	ui9->nGUIRow = nIndexer++;
	ui9->nGUIColumn = -1;
	ui9->bEnableParamSmoothing = false;
	ui9->fSmoothingTimeInMs = 100.00;
	ui9->uControlTheme[0] = 0; ui9->uControlTheme[1] = 0; ui9->uControlTheme[2] = 0; ui9->uControlTheme[3] = 0; ui9->uControlTheme[4] = 0; ui9->uControlTheme[5] = 0; ui9->uControlTheme[6] = 0; ui9->uControlTheme[7] = 0; ui9->uControlTheme[8] = 2; ui9->uControlTheme[9] = 0; ui9->uControlTheme[10] = 0; ui9->uControlTheme[11] = 0; ui9->uControlTheme[12] = 0; ui9->uControlTheme[13] = 0; ui9->uControlTheme[14] = 0; ui9->uControlTheme[15] = 0; ui9->uControlTheme[16] = 2; ui9->uControlTheme[17] = 0; ui9->uControlTheme[18] = 0; ui9->uControlTheme[19] = 0; ui9->uControlTheme[20] = 0; ui9->uControlTheme[21] = 0; ui9->uControlTheme[22] = 0; ui9->uControlTheme[23] = 0; ui9->uControlTheme[24] = 0; ui9->uControlTheme[25] = 0; ui9->uControlTheme[26] = 0; ui9->uControlTheme[27] = 0; ui9->uControlTheme[28] = 0; ui9->uControlTheme[29] = 0; ui9->uControlTheme[30] = 0; ui9->uControlTheme[31] = 0; 
	ui9->uFluxCapControl[0] = 0; ui9->uFluxCapControl[1] = 0; ui9->uFluxCapControl[2] = 0; ui9->uFluxCapControl[3] = 0; ui9->uFluxCapControl[4] = 0; ui9->uFluxCapControl[5] = 0; ui9->uFluxCapControl[6] = 0; ui9->uFluxCapControl[7] = 0; ui9->uFluxCapControl[8] = 0; ui9->uFluxCapControl[9] = 0; ui9->uFluxCapControl[10] = 0; ui9->uFluxCapControl[11] = 0; ui9->uFluxCapControl[12] = 0; ui9->uFluxCapControl[13] = 0; ui9->uFluxCapControl[14] = 0; ui9->uFluxCapControl[15] = 0; ui9->uFluxCapControl[16] = 0; ui9->uFluxCapControl[17] = 0; ui9->uFluxCapControl[18] = 0; ui9->uFluxCapControl[19] = 0; ui9->uFluxCapControl[20] = 0; ui9->uFluxCapControl[21] = 0; ui9->uFluxCapControl[22] = 0; ui9->uFluxCapControl[23] = 0; ui9->uFluxCapControl[24] = 0; ui9->uFluxCapControl[25] = 0; ui9->uFluxCapControl[26] = 0; ui9->uFluxCapControl[27] = 0; ui9->uFluxCapControl[28] = 0; ui9->uFluxCapControl[29] = 0; ui9->uFluxCapControl[30] = 0; ui9->uFluxCapControl[31] = 0; ui9->uFluxCapControl[32] = 0; ui9->uFluxCapControl[33] = 0; ui9->uFluxCapControl[34] = 0; ui9->uFluxCapControl[35] = 0; ui9->uFluxCapControl[36] = 0; ui9->uFluxCapControl[37] = 0; ui9->uFluxCapControl[38] = 0; ui9->uFluxCapControl[39] = 0; ui9->uFluxCapControl[40] = 0; ui9->uFluxCapControl[41] = 0; ui9->uFluxCapControl[42] = 0; ui9->uFluxCapControl[43] = 0; ui9->uFluxCapControl[44] = 0; ui9->uFluxCapControl[45] = 0; ui9->uFluxCapControl[46] = 0; ui9->uFluxCapControl[47] = 0; ui9->uFluxCapControl[48] = 0; ui9->uFluxCapControl[49] = 0; ui9->uFluxCapControl[50] = 0; ui9->uFluxCapControl[51] = 0; ui9->uFluxCapControl[52] = 0; ui9->uFluxCapControl[53] = 0; ui9->uFluxCapControl[54] = 0; ui9->uFluxCapControl[55] = 0; ui9->uFluxCapControl[56] = 0; ui9->uFluxCapControl[57] = 0; ui9->uFluxCapControl[58] = 0; ui9->uFluxCapControl[59] = 0; ui9->uFluxCapControl[60] = 0; ui9->uFluxCapControl[61] = 0; ui9->uFluxCapControl[62] = 0; ui9->uFluxCapControl[63] = 0; 
	ui9->fFluxCapData[0] = 0.000000; ui9->fFluxCapData[1] = 0.000000; ui9->fFluxCapData[2] = 0.000000; ui9->fFluxCapData[3] = 0.000000; ui9->fFluxCapData[4] = 0.000000; ui9->fFluxCapData[5] = 0.000000; ui9->fFluxCapData[6] = 0.000000; ui9->fFluxCapData[7] = 0.000000; ui9->fFluxCapData[8] = 0.000000; ui9->fFluxCapData[9] = 0.000000; ui9->fFluxCapData[10] = 0.000000; ui9->fFluxCapData[11] = 0.000000; ui9->fFluxCapData[12] = 0.000000; ui9->fFluxCapData[13] = 0.000000; ui9->fFluxCapData[14] = 0.000000; ui9->fFluxCapData[15] = 0.000000; ui9->fFluxCapData[16] = 0.000000; ui9->fFluxCapData[17] = 0.000000; ui9->fFluxCapData[18] = 0.000000; ui9->fFluxCapData[19] = 0.000000; ui9->fFluxCapData[20] = 0.000000; ui9->fFluxCapData[21] = 0.000000; ui9->fFluxCapData[22] = 0.000000; ui9->fFluxCapData[23] = 0.000000; ui9->fFluxCapData[24] = 0.000000; ui9->fFluxCapData[25] = 0.000000; ui9->fFluxCapData[26] = 0.000000; ui9->fFluxCapData[27] = 0.000000; ui9->fFluxCapData[28] = 0.000000; ui9->fFluxCapData[29] = 0.000000; ui9->fFluxCapData[30] = 0.000000; ui9->fFluxCapData[31] = 0.000000; ui9->fFluxCapData[32] = 0.000000; ui9->fFluxCapData[33] = 0.000000; ui9->fFluxCapData[34] = 0.000000; ui9->fFluxCapData[35] = 0.000000; ui9->fFluxCapData[36] = 0.000000; ui9->fFluxCapData[37] = 0.000000; ui9->fFluxCapData[38] = 0.000000; ui9->fFluxCapData[39] = 0.000000; ui9->fFluxCapData[40] = 0.000000; ui9->fFluxCapData[41] = 0.000000; ui9->fFluxCapData[42] = 0.000000; ui9->fFluxCapData[43] = 0.000000; ui9->fFluxCapData[44] = 0.000000; ui9->fFluxCapData[45] = 0.000000; ui9->fFluxCapData[46] = 0.000000; ui9->fFluxCapData[47] = 0.000000; ui9->fFluxCapData[48] = 0.000000; ui9->fFluxCapData[49] = 0.000000; ui9->fFluxCapData[50] = 0.000000; ui9->fFluxCapData[51] = 0.000000; ui9->fFluxCapData[52] = 0.000000; ui9->fFluxCapData[53] = 0.000000; ui9->fFluxCapData[54] = 0.000000; ui9->fFluxCapData[55] = 0.000000; ui9->fFluxCapData[56] = 0.000000; ui9->fFluxCapData[57] = 0.000000; ui9->fFluxCapData[58] = 0.000000; ui9->fFluxCapData[59] = 0.000000; ui9->fFluxCapData[60] = 0.000000; ui9->fFluxCapData[61] = 0.000000; ui9->fFluxCapData[62] = 0.000000; ui9->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui9);
	delete ui9;


	m_fDelay_mSec = 0.200000;
	CUICtrl* ui10 = new CUICtrl;
	ui10->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui10->uControlId = 20;
	ui10->bLogSlider = false;
	ui10->bExpSlider = false;
	ui10->fUserDisplayDataLoLimit = 0.000000;
	ui10->fUserDisplayDataHiLimit = 2000.000000;
	ui10->uUserDataType = floatData;
	ui10->fInitUserIntValue = 0;
	ui10->fInitUserFloatValue = 0.200000;
	ui10->fInitUserDoubleValue = 0;
	ui10->fInitUserUINTValue = 0;
	ui10->m_pUserCookedIntData = NULL;
	ui10->m_pUserCookedFloatData = &m_fDelay_mSec;
	ui10->m_pUserCookedDoubleData = NULL;
	ui10->m_pUserCookedUINTData = NULL;
	ui10->cControlUnits = "mSec";
	ui10->cVariableName = "m_fDelay_mSec";
	ui10->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui10->dPresetData[0] = 0.200000;ui10->dPresetData[1] = 0.000000;ui10->dPresetData[2] = 0.000000;ui10->dPresetData[3] = 0.000000;ui10->dPresetData[4] = 0.000000;ui10->dPresetData[5] = 0.000000;ui10->dPresetData[6] = 0.000000;ui10->dPresetData[7] = 0.000000;ui10->dPresetData[8] = 0.000000;ui10->dPresetData[9] = 0.000000;ui10->dPresetData[10] = 0.000000;ui10->dPresetData[11] = 0.000000;ui10->dPresetData[12] = 0.000000;ui10->dPresetData[13] = 0.000000;ui10->dPresetData[14] = 0.000000;ui10->dPresetData[15] = 0.000000;
	ui10->cControlName = "Delay Time";
	ui10->bOwnerControl = false;
	ui10->bMIDIControl = false;
	ui10->uMIDIControlCommand = 176;
	ui10->uMIDIControlName = 3;
	ui10->uMIDIControlChannel = 0;
	ui10->nGUIRow = nIndexer++;
	ui10->nGUIColumn = -1;
	ui10->bEnableParamSmoothing = false;
	ui10->fSmoothingTimeInMs = 100.00;
	ui10->uControlTheme[0] = 0; ui10->uControlTheme[1] = 0; ui10->uControlTheme[2] = 0; ui10->uControlTheme[3] = 0; ui10->uControlTheme[4] = 0; ui10->uControlTheme[5] = 0; ui10->uControlTheme[6] = 0; ui10->uControlTheme[7] = 0; ui10->uControlTheme[8] = 4; ui10->uControlTheme[9] = 0; ui10->uControlTheme[10] = 0; ui10->uControlTheme[11] = 0; ui10->uControlTheme[12] = 0; ui10->uControlTheme[13] = 0; ui10->uControlTheme[14] = 0; ui10->uControlTheme[15] = 0; ui10->uControlTheme[16] = 2; ui10->uControlTheme[17] = 0; ui10->uControlTheme[18] = 0; ui10->uControlTheme[19] = 0; ui10->uControlTheme[20] = 0; ui10->uControlTheme[21] = 0; ui10->uControlTheme[22] = 0; ui10->uControlTheme[23] = 0; ui10->uControlTheme[24] = 0; ui10->uControlTheme[25] = 0; ui10->uControlTheme[26] = 0; ui10->uControlTheme[27] = 0; ui10->uControlTheme[28] = 0; ui10->uControlTheme[29] = 0; ui10->uControlTheme[30] = 0; ui10->uControlTheme[31] = 0; 
	ui10->uFluxCapControl[0] = 0; ui10->uFluxCapControl[1] = 0; ui10->uFluxCapControl[2] = 0; ui10->uFluxCapControl[3] = 0; ui10->uFluxCapControl[4] = 0; ui10->uFluxCapControl[5] = 0; ui10->uFluxCapControl[6] = 0; ui10->uFluxCapControl[7] = 0; ui10->uFluxCapControl[8] = 0; ui10->uFluxCapControl[9] = 0; ui10->uFluxCapControl[10] = 0; ui10->uFluxCapControl[11] = 0; ui10->uFluxCapControl[12] = 0; ui10->uFluxCapControl[13] = 0; ui10->uFluxCapControl[14] = 0; ui10->uFluxCapControl[15] = 0; ui10->uFluxCapControl[16] = 0; ui10->uFluxCapControl[17] = 0; ui10->uFluxCapControl[18] = 0; ui10->uFluxCapControl[19] = 0; ui10->uFluxCapControl[20] = 0; ui10->uFluxCapControl[21] = 0; ui10->uFluxCapControl[22] = 0; ui10->uFluxCapControl[23] = 0; ui10->uFluxCapControl[24] = 0; ui10->uFluxCapControl[25] = 0; ui10->uFluxCapControl[26] = 0; ui10->uFluxCapControl[27] = 0; ui10->uFluxCapControl[28] = 0; ui10->uFluxCapControl[29] = 0; ui10->uFluxCapControl[30] = 0; ui10->uFluxCapControl[31] = 0; ui10->uFluxCapControl[32] = 0; ui10->uFluxCapControl[33] = 0; ui10->uFluxCapControl[34] = 0; ui10->uFluxCapControl[35] = 0; ui10->uFluxCapControl[36] = 0; ui10->uFluxCapControl[37] = 0; ui10->uFluxCapControl[38] = 0; ui10->uFluxCapControl[39] = 0; ui10->uFluxCapControl[40] = 0; ui10->uFluxCapControl[41] = 0; ui10->uFluxCapControl[42] = 0; ui10->uFluxCapControl[43] = 0; ui10->uFluxCapControl[44] = 0; ui10->uFluxCapControl[45] = 0; ui10->uFluxCapControl[46] = 0; ui10->uFluxCapControl[47] = 0; ui10->uFluxCapControl[48] = 0; ui10->uFluxCapControl[49] = 0; ui10->uFluxCapControl[50] = 0; ui10->uFluxCapControl[51] = 0; ui10->uFluxCapControl[52] = 0; ui10->uFluxCapControl[53] = 0; ui10->uFluxCapControl[54] = 0; ui10->uFluxCapControl[55] = 0; ui10->uFluxCapControl[56] = 0; ui10->uFluxCapControl[57] = 0; ui10->uFluxCapControl[58] = 0; ui10->uFluxCapControl[59] = 0; ui10->uFluxCapControl[60] = 0; ui10->uFluxCapControl[61] = 0; ui10->uFluxCapControl[62] = 0; ui10->uFluxCapControl[63] = 0; 
	ui10->fFluxCapData[0] = 0.000000; ui10->fFluxCapData[1] = 0.000000; ui10->fFluxCapData[2] = 0.000000; ui10->fFluxCapData[3] = 0.000000; ui10->fFluxCapData[4] = 0.000000; ui10->fFluxCapData[5] = 0.000000; ui10->fFluxCapData[6] = 0.000000; ui10->fFluxCapData[7] = 0.000000; ui10->fFluxCapData[8] = 0.000000; ui10->fFluxCapData[9] = 0.000000; ui10->fFluxCapData[10] = 0.000000; ui10->fFluxCapData[11] = 0.000000; ui10->fFluxCapData[12] = 0.000000; ui10->fFluxCapData[13] = 0.000000; ui10->fFluxCapData[14] = 0.000000; ui10->fFluxCapData[15] = 0.000000; ui10->fFluxCapData[16] = 0.000000; ui10->fFluxCapData[17] = 0.000000; ui10->fFluxCapData[18] = 0.000000; ui10->fFluxCapData[19] = 0.000000; ui10->fFluxCapData[20] = 0.000000; ui10->fFluxCapData[21] = 0.000000; ui10->fFluxCapData[22] = 0.000000; ui10->fFluxCapData[23] = 0.000000; ui10->fFluxCapData[24] = 0.000000; ui10->fFluxCapData[25] = 0.000000; ui10->fFluxCapData[26] = 0.000000; ui10->fFluxCapData[27] = 0.000000; ui10->fFluxCapData[28] = 0.000000; ui10->fFluxCapData[29] = 0.000000; ui10->fFluxCapData[30] = 0.000000; ui10->fFluxCapData[31] = 0.000000; ui10->fFluxCapData[32] = 0.000000; ui10->fFluxCapData[33] = 0.000000; ui10->fFluxCapData[34] = 0.000000; ui10->fFluxCapData[35] = 0.000000; ui10->fFluxCapData[36] = 0.000000; ui10->fFluxCapData[37] = 0.000000; ui10->fFluxCapData[38] = 0.000000; ui10->fFluxCapData[39] = 0.000000; ui10->fFluxCapData[40] = 0.000000; ui10->fFluxCapData[41] = 0.000000; ui10->fFluxCapData[42] = 0.000000; ui10->fFluxCapData[43] = 0.000000; ui10->fFluxCapData[44] = 0.000000; ui10->fFluxCapData[45] = 0.000000; ui10->fFluxCapData[46] = 0.000000; ui10->fFluxCapData[47] = 0.000000; ui10->fFluxCapData[48] = 0.000000; ui10->fFluxCapData[49] = 0.000000; ui10->fFluxCapData[50] = 0.000000; ui10->fFluxCapData[51] = 0.000000; ui10->fFluxCapData[52] = 0.000000; ui10->fFluxCapData[53] = 0.000000; ui10->fFluxCapData[54] = 0.000000; ui10->fFluxCapData[55] = 0.000000; ui10->fFluxCapData[56] = 0.000000; ui10->fFluxCapData[57] = 0.000000; ui10->fFluxCapData[58] = 0.000000; ui10->fFluxCapData[59] = 0.000000; ui10->fFluxCapData[60] = 0.000000; ui10->fFluxCapData[61] = 0.000000; ui10->fFluxCapData[62] = 0.000000; ui10->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui10);
	delete ui10;


	m_fFeedback_pct = 0.000000;
	CUICtrl* ui11 = new CUICtrl;
	ui11->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui11->uControlId = 21;
	ui11->bLogSlider = false;
	ui11->bExpSlider = false;
	ui11->fUserDisplayDataLoLimit = -100.000000;
	ui11->fUserDisplayDataHiLimit = 100.000000;
	ui11->uUserDataType = floatData;
	ui11->fInitUserIntValue = 0;
	ui11->fInitUserFloatValue = 0.000000;
	ui11->fInitUserDoubleValue = 0;
	ui11->fInitUserUINTValue = 0;
	ui11->m_pUserCookedIntData = NULL;
	ui11->m_pUserCookedFloatData = &m_fFeedback_pct;
	ui11->m_pUserCookedDoubleData = NULL;
	ui11->m_pUserCookedUINTData = NULL;
	ui11->cControlUnits = "%";
	ui11->cVariableName = "m_fFeedback_pct";
	ui11->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui11->dPresetData[0] = 0.000000;ui11->dPresetData[1] = 0.000000;ui11->dPresetData[2] = 0.000000;ui11->dPresetData[3] = 0.000000;ui11->dPresetData[4] = 0.000000;ui11->dPresetData[5] = 0.000000;ui11->dPresetData[6] = 0.000000;ui11->dPresetData[7] = 0.000000;ui11->dPresetData[8] = 0.000000;ui11->dPresetData[9] = 0.000000;ui11->dPresetData[10] = 0.000000;ui11->dPresetData[11] = 0.000000;ui11->dPresetData[12] = 0.000000;ui11->dPresetData[13] = 0.000000;ui11->dPresetData[14] = 0.000000;ui11->dPresetData[15] = 0.000000;
	ui11->cControlName = "Feedback";
	ui11->bOwnerControl = false;
	ui11->bMIDIControl = false;
	ui11->uMIDIControlCommand = 176;
	ui11->uMIDIControlName = 3;
	ui11->uMIDIControlChannel = 0;
	ui11->nGUIRow = nIndexer++;
	ui11->nGUIColumn = -1;
	ui11->bEnableParamSmoothing = false;
	ui11->fSmoothingTimeInMs = 100.00;
	ui11->uControlTheme[0] = 0; ui11->uControlTheme[1] = 0; ui11->uControlTheme[2] = 0; ui11->uControlTheme[3] = 0; ui11->uControlTheme[4] = 0; ui11->uControlTheme[5] = 0; ui11->uControlTheme[6] = 0; ui11->uControlTheme[7] = 0; ui11->uControlTheme[8] = 4; ui11->uControlTheme[9] = 0; ui11->uControlTheme[10] = 0; ui11->uControlTheme[11] = 0; ui11->uControlTheme[12] = 0; ui11->uControlTheme[13] = 0; ui11->uControlTheme[14] = 0; ui11->uControlTheme[15] = 0; ui11->uControlTheme[16] = 2; ui11->uControlTheme[17] = 0; ui11->uControlTheme[18] = 0; ui11->uControlTheme[19] = 0; ui11->uControlTheme[20] = 0; ui11->uControlTheme[21] = 0; ui11->uControlTheme[22] = 0; ui11->uControlTheme[23] = 0; ui11->uControlTheme[24] = 0; ui11->uControlTheme[25] = 0; ui11->uControlTheme[26] = 0; ui11->uControlTheme[27] = 0; ui11->uControlTheme[28] = 0; ui11->uControlTheme[29] = 0; ui11->uControlTheme[30] = 0; ui11->uControlTheme[31] = 0; 
	ui11->uFluxCapControl[0] = 0; ui11->uFluxCapControl[1] = 0; ui11->uFluxCapControl[2] = 0; ui11->uFluxCapControl[3] = 0; ui11->uFluxCapControl[4] = 0; ui11->uFluxCapControl[5] = 0; ui11->uFluxCapControl[6] = 0; ui11->uFluxCapControl[7] = 0; ui11->uFluxCapControl[8] = 0; ui11->uFluxCapControl[9] = 0; ui11->uFluxCapControl[10] = 0; ui11->uFluxCapControl[11] = 0; ui11->uFluxCapControl[12] = 0; ui11->uFluxCapControl[13] = 0; ui11->uFluxCapControl[14] = 0; ui11->uFluxCapControl[15] = 0; ui11->uFluxCapControl[16] = 0; ui11->uFluxCapControl[17] = 0; ui11->uFluxCapControl[18] = 0; ui11->uFluxCapControl[19] = 0; ui11->uFluxCapControl[20] = 0; ui11->uFluxCapControl[21] = 0; ui11->uFluxCapControl[22] = 0; ui11->uFluxCapControl[23] = 0; ui11->uFluxCapControl[24] = 0; ui11->uFluxCapControl[25] = 0; ui11->uFluxCapControl[26] = 0; ui11->uFluxCapControl[27] = 0; ui11->uFluxCapControl[28] = 0; ui11->uFluxCapControl[29] = 0; ui11->uFluxCapControl[30] = 0; ui11->uFluxCapControl[31] = 0; ui11->uFluxCapControl[32] = 0; ui11->uFluxCapControl[33] = 0; ui11->uFluxCapControl[34] = 0; ui11->uFluxCapControl[35] = 0; ui11->uFluxCapControl[36] = 0; ui11->uFluxCapControl[37] = 0; ui11->uFluxCapControl[38] = 0; ui11->uFluxCapControl[39] = 0; ui11->uFluxCapControl[40] = 0; ui11->uFluxCapControl[41] = 0; ui11->uFluxCapControl[42] = 0; ui11->uFluxCapControl[43] = 0; ui11->uFluxCapControl[44] = 0; ui11->uFluxCapControl[45] = 0; ui11->uFluxCapControl[46] = 0; ui11->uFluxCapControl[47] = 0; ui11->uFluxCapControl[48] = 0; ui11->uFluxCapControl[49] = 0; ui11->uFluxCapControl[50] = 0; ui11->uFluxCapControl[51] = 0; ui11->uFluxCapControl[52] = 0; ui11->uFluxCapControl[53] = 0; ui11->uFluxCapControl[54] = 0; ui11->uFluxCapControl[55] = 0; ui11->uFluxCapControl[56] = 0; ui11->uFluxCapControl[57] = 0; ui11->uFluxCapControl[58] = 0; ui11->uFluxCapControl[59] = 0; ui11->uFluxCapControl[60] = 0; ui11->uFluxCapControl[61] = 0; ui11->uFluxCapControl[62] = 0; ui11->uFluxCapControl[63] = 0; 
	ui11->fFluxCapData[0] = 0.000000; ui11->fFluxCapData[1] = 0.000000; ui11->fFluxCapData[2] = 0.000000; ui11->fFluxCapData[3] = 0.000000; ui11->fFluxCapData[4] = 0.000000; ui11->fFluxCapData[5] = 0.000000; ui11->fFluxCapData[6] = 0.000000; ui11->fFluxCapData[7] = 0.000000; ui11->fFluxCapData[8] = 0.000000; ui11->fFluxCapData[9] = 0.000000; ui11->fFluxCapData[10] = 0.000000; ui11->fFluxCapData[11] = 0.000000; ui11->fFluxCapData[12] = 0.000000; ui11->fFluxCapData[13] = 0.000000; ui11->fFluxCapData[14] = 0.000000; ui11->fFluxCapData[15] = 0.000000; ui11->fFluxCapData[16] = 0.000000; ui11->fFluxCapData[17] = 0.000000; ui11->fFluxCapData[18] = 0.000000; ui11->fFluxCapData[19] = 0.000000; ui11->fFluxCapData[20] = 0.000000; ui11->fFluxCapData[21] = 0.000000; ui11->fFluxCapData[22] = 0.000000; ui11->fFluxCapData[23] = 0.000000; ui11->fFluxCapData[24] = 0.000000; ui11->fFluxCapData[25] = 0.000000; ui11->fFluxCapData[26] = 0.000000; ui11->fFluxCapData[27] = 0.000000; ui11->fFluxCapData[28] = 0.000000; ui11->fFluxCapData[29] = 0.000000; ui11->fFluxCapData[30] = 0.000000; ui11->fFluxCapData[31] = 0.000000; ui11->fFluxCapData[32] = 0.000000; ui11->fFluxCapData[33] = 0.000000; ui11->fFluxCapData[34] = 0.000000; ui11->fFluxCapData[35] = 0.000000; ui11->fFluxCapData[36] = 0.000000; ui11->fFluxCapData[37] = 0.000000; ui11->fFluxCapData[38] = 0.000000; ui11->fFluxCapData[39] = 0.000000; ui11->fFluxCapData[40] = 0.000000; ui11->fFluxCapData[41] = 0.000000; ui11->fFluxCapData[42] = 0.000000; ui11->fFluxCapData[43] = 0.000000; ui11->fFluxCapData[44] = 0.000000; ui11->fFluxCapData[45] = 0.000000; ui11->fFluxCapData[46] = 0.000000; ui11->fFluxCapData[47] = 0.000000; ui11->fFluxCapData[48] = 0.000000; ui11->fFluxCapData[49] = 0.000000; ui11->fFluxCapData[50] = 0.000000; ui11->fFluxCapData[51] = 0.000000; ui11->fFluxCapData[52] = 0.000000; ui11->fFluxCapData[53] = 0.000000; ui11->fFluxCapData[54] = 0.000000; ui11->fFluxCapData[55] = 0.000000; ui11->fFluxCapData[56] = 0.000000; ui11->fFluxCapData[57] = 0.000000; ui11->fFluxCapData[58] = 0.000000; ui11->fFluxCapData[59] = 0.000000; ui11->fFluxCapData[60] = 0.000000; ui11->fFluxCapData[61] = 0.000000; ui11->fFluxCapData[62] = 0.000000; ui11->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui11);
	delete ui11;


	m_fDelayMix_pct = 50.000000;
	CUICtrl* ui12 = new CUICtrl;
	ui12->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui12->uControlId = 22;
	ui12->bLogSlider = false;
	ui12->bExpSlider = false;
	ui12->fUserDisplayDataLoLimit = 0.000000;
	ui12->fUserDisplayDataHiLimit = 100.000000;
	ui12->uUserDataType = floatData;
	ui12->fInitUserIntValue = 0;
	ui12->fInitUserFloatValue = 50.000000;
	ui12->fInitUserDoubleValue = 0;
	ui12->fInitUserUINTValue = 0;
	ui12->m_pUserCookedIntData = NULL;
	ui12->m_pUserCookedFloatData = &m_fDelayMix_pct;
	ui12->m_pUserCookedDoubleData = NULL;
	ui12->m_pUserCookedUINTData = NULL;
	ui12->cControlUnits = "%";
	ui12->cVariableName = "m_fDelayMix_pct";
	ui12->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui12->dPresetData[0] = 50.000000;ui12->dPresetData[1] = 0.000000;ui12->dPresetData[2] = 0.000000;ui12->dPresetData[3] = 0.000000;ui12->dPresetData[4] = 0.000000;ui12->dPresetData[5] = 0.000000;ui12->dPresetData[6] = 0.000000;ui12->dPresetData[7] = 0.000000;ui12->dPresetData[8] = 0.000000;ui12->dPresetData[9] = 0.000000;ui12->dPresetData[10] = 0.000000;ui12->dPresetData[11] = 0.000000;ui12->dPresetData[12] = 0.000000;ui12->dPresetData[13] = 0.000000;ui12->dPresetData[14] = 0.000000;ui12->dPresetData[15] = 0.000000;
	ui12->cControlName = "Delay Mix";
	ui12->bOwnerControl = false;
	ui12->bMIDIControl = false;
	ui12->uMIDIControlCommand = 176;
	ui12->uMIDIControlName = 3;
	ui12->uMIDIControlChannel = 0;
	ui12->nGUIRow = nIndexer++;
	ui12->nGUIColumn = -1;
	ui12->bEnableParamSmoothing = false;
	ui12->fSmoothingTimeInMs = 100.00;
	ui12->uControlTheme[0] = 0; ui12->uControlTheme[1] = 0; ui12->uControlTheme[2] = 0; ui12->uControlTheme[3] = 0; ui12->uControlTheme[4] = 0; ui12->uControlTheme[5] = 0; ui12->uControlTheme[6] = 0; ui12->uControlTheme[7] = 0; ui12->uControlTheme[8] = 4; ui12->uControlTheme[9] = 0; ui12->uControlTheme[10] = 0; ui12->uControlTheme[11] = 0; ui12->uControlTheme[12] = 0; ui12->uControlTheme[13] = 0; ui12->uControlTheme[14] = 0; ui12->uControlTheme[15] = 0; ui12->uControlTheme[16] = 2; ui12->uControlTheme[17] = 0; ui12->uControlTheme[18] = 0; ui12->uControlTheme[19] = 0; ui12->uControlTheme[20] = 0; ui12->uControlTheme[21] = 0; ui12->uControlTheme[22] = 0; ui12->uControlTheme[23] = 0; ui12->uControlTheme[24] = 0; ui12->uControlTheme[25] = 0; ui12->uControlTheme[26] = 0; ui12->uControlTheme[27] = 0; ui12->uControlTheme[28] = 0; ui12->uControlTheme[29] = 0; ui12->uControlTheme[30] = 0; ui12->uControlTheme[31] = 0; 
	ui12->uFluxCapControl[0] = 0; ui12->uFluxCapControl[1] = 0; ui12->uFluxCapControl[2] = 0; ui12->uFluxCapControl[3] = 0; ui12->uFluxCapControl[4] = 0; ui12->uFluxCapControl[5] = 0; ui12->uFluxCapControl[6] = 0; ui12->uFluxCapControl[7] = 0; ui12->uFluxCapControl[8] = 0; ui12->uFluxCapControl[9] = 0; ui12->uFluxCapControl[10] = 0; ui12->uFluxCapControl[11] = 0; ui12->uFluxCapControl[12] = 0; ui12->uFluxCapControl[13] = 0; ui12->uFluxCapControl[14] = 0; ui12->uFluxCapControl[15] = 0; ui12->uFluxCapControl[16] = 0; ui12->uFluxCapControl[17] = 0; ui12->uFluxCapControl[18] = 0; ui12->uFluxCapControl[19] = 0; ui12->uFluxCapControl[20] = 0; ui12->uFluxCapControl[21] = 0; ui12->uFluxCapControl[22] = 0; ui12->uFluxCapControl[23] = 0; ui12->uFluxCapControl[24] = 0; ui12->uFluxCapControl[25] = 0; ui12->uFluxCapControl[26] = 0; ui12->uFluxCapControl[27] = 0; ui12->uFluxCapControl[28] = 0; ui12->uFluxCapControl[29] = 0; ui12->uFluxCapControl[30] = 0; ui12->uFluxCapControl[31] = 0; ui12->uFluxCapControl[32] = 0; ui12->uFluxCapControl[33] = 0; ui12->uFluxCapControl[34] = 0; ui12->uFluxCapControl[35] = 0; ui12->uFluxCapControl[36] = 0; ui12->uFluxCapControl[37] = 0; ui12->uFluxCapControl[38] = 0; ui12->uFluxCapControl[39] = 0; ui12->uFluxCapControl[40] = 0; ui12->uFluxCapControl[41] = 0; ui12->uFluxCapControl[42] = 0; ui12->uFluxCapControl[43] = 0; ui12->uFluxCapControl[44] = 0; ui12->uFluxCapControl[45] = 0; ui12->uFluxCapControl[46] = 0; ui12->uFluxCapControl[47] = 0; ui12->uFluxCapControl[48] = 0; ui12->uFluxCapControl[49] = 0; ui12->uFluxCapControl[50] = 0; ui12->uFluxCapControl[51] = 0; ui12->uFluxCapControl[52] = 0; ui12->uFluxCapControl[53] = 0; ui12->uFluxCapControl[54] = 0; ui12->uFluxCapControl[55] = 0; ui12->uFluxCapControl[56] = 0; ui12->uFluxCapControl[57] = 0; ui12->uFluxCapControl[58] = 0; ui12->uFluxCapControl[59] = 0; ui12->uFluxCapControl[60] = 0; ui12->uFluxCapControl[61] = 0; ui12->uFluxCapControl[62] = 0; ui12->uFluxCapControl[63] = 0; 
	ui12->fFluxCapData[0] = 0.000000; ui12->fFluxCapData[1] = 0.000000; ui12->fFluxCapData[2] = 0.000000; ui12->fFluxCapData[3] = 0.000000; ui12->fFluxCapData[4] = 0.000000; ui12->fFluxCapData[5] = 0.000000; ui12->fFluxCapData[6] = 0.000000; ui12->fFluxCapData[7] = 0.000000; ui12->fFluxCapData[8] = 0.000000; ui12->fFluxCapData[9] = 0.000000; ui12->fFluxCapData[10] = 0.000000; ui12->fFluxCapData[11] = 0.000000; ui12->fFluxCapData[12] = 0.000000; ui12->fFluxCapData[13] = 0.000000; ui12->fFluxCapData[14] = 0.000000; ui12->fFluxCapData[15] = 0.000000; ui12->fFluxCapData[16] = 0.000000; ui12->fFluxCapData[17] = 0.000000; ui12->fFluxCapData[18] = 0.000000; ui12->fFluxCapData[19] = 0.000000; ui12->fFluxCapData[20] = 0.000000; ui12->fFluxCapData[21] = 0.000000; ui12->fFluxCapData[22] = 0.000000; ui12->fFluxCapData[23] = 0.000000; ui12->fFluxCapData[24] = 0.000000; ui12->fFluxCapData[25] = 0.000000; ui12->fFluxCapData[26] = 0.000000; ui12->fFluxCapData[27] = 0.000000; ui12->fFluxCapData[28] = 0.000000; ui12->fFluxCapData[29] = 0.000000; ui12->fFluxCapData[30] = 0.000000; ui12->fFluxCapData[31] = 0.000000; ui12->fFluxCapData[32] = 0.000000; ui12->fFluxCapData[33] = 0.000000; ui12->fFluxCapData[34] = 0.000000; ui12->fFluxCapData[35] = 0.000000; ui12->fFluxCapData[36] = 0.000000; ui12->fFluxCapData[37] = 0.000000; ui12->fFluxCapData[38] = 0.000000; ui12->fFluxCapData[39] = 0.000000; ui12->fFluxCapData[40] = 0.000000; ui12->fFluxCapData[41] = 0.000000; ui12->fFluxCapData[42] = 0.000000; ui12->fFluxCapData[43] = 0.000000; ui12->fFluxCapData[44] = 0.000000; ui12->fFluxCapData[45] = 0.000000; ui12->fFluxCapData[46] = 0.000000; ui12->fFluxCapData[47] = 0.000000; ui12->fFluxCapData[48] = 0.000000; ui12->fFluxCapData[49] = 0.000000; ui12->fFluxCapData[50] = 0.000000; ui12->fFluxCapData[51] = 0.000000; ui12->fFluxCapData[52] = 0.000000; ui12->fFluxCapData[53] = 0.000000; ui12->fFluxCapData[54] = 0.000000; ui12->fFluxCapData[55] = 0.000000; ui12->fFluxCapData[56] = 0.000000; ui12->fFluxCapData[57] = 0.000000; ui12->fFluxCapData[58] = 0.000000; ui12->fFluxCapData[59] = 0.000000; ui12->fFluxCapData[60] = 0.000000; ui12->fFluxCapData[61] = 0.000000; ui12->fFluxCapData[62] = 0.000000; ui12->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui12);
	delete ui12;


	m_f_HPF = 1000.000000;
	CUICtrl* ui13 = new CUICtrl;
	ui13->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui13->uControlId = 23;
	ui13->bLogSlider = false;
	ui13->bExpSlider = false;
	ui13->fUserDisplayDataLoLimit = 100.000000;
	ui13->fUserDisplayDataHiLimit = 18000.000000;
	ui13->uUserDataType = floatData;
	ui13->fInitUserIntValue = 0;
	ui13->fInitUserFloatValue = 1000.000000;
	ui13->fInitUserDoubleValue = 0;
	ui13->fInitUserUINTValue = 0;
	ui13->m_pUserCookedIntData = NULL;
	ui13->m_pUserCookedFloatData = &m_f_HPF;
	ui13->m_pUserCookedDoubleData = NULL;
	ui13->m_pUserCookedUINTData = NULL;
	ui13->cControlUnits = "Hz";
	ui13->cVariableName = "m_f_HPF";
	ui13->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui13->dPresetData[0] = 1000.000000;ui13->dPresetData[1] = 0.000000;ui13->dPresetData[2] = 0.000000;ui13->dPresetData[3] = 0.000000;ui13->dPresetData[4] = 0.000000;ui13->dPresetData[5] = 0.000000;ui13->dPresetData[6] = 0.000000;ui13->dPresetData[7] = 0.000000;ui13->dPresetData[8] = 0.000000;ui13->dPresetData[9] = 0.000000;ui13->dPresetData[10] = 0.000000;ui13->dPresetData[11] = 0.000000;ui13->dPresetData[12] = 0.000000;ui13->dPresetData[13] = 0.000000;ui13->dPresetData[14] = 0.000000;ui13->dPresetData[15] = 0.000000;
	ui13->cControlName = "Delay HPF";
	ui13->bOwnerControl = false;
	ui13->bMIDIControl = false;
	ui13->uMIDIControlCommand = 176;
	ui13->uMIDIControlName = 3;
	ui13->uMIDIControlChannel = 0;
	ui13->nGUIRow = nIndexer++;
	ui13->nGUIColumn = -1;
	ui13->bEnableParamSmoothing = false;
	ui13->fSmoothingTimeInMs = 100.00;
	ui13->uControlTheme[0] = 0; ui13->uControlTheme[1] = 0; ui13->uControlTheme[2] = 0; ui13->uControlTheme[3] = 0; ui13->uControlTheme[4] = 0; ui13->uControlTheme[5] = 0; ui13->uControlTheme[6] = 0; ui13->uControlTheme[7] = 0; ui13->uControlTheme[8] = 4; ui13->uControlTheme[9] = 0; ui13->uControlTheme[10] = 0; ui13->uControlTheme[11] = 0; ui13->uControlTheme[12] = 0; ui13->uControlTheme[13] = 0; ui13->uControlTheme[14] = 0; ui13->uControlTheme[15] = 0; ui13->uControlTheme[16] = 2; ui13->uControlTheme[17] = 0; ui13->uControlTheme[18] = 0; ui13->uControlTheme[19] = 0; ui13->uControlTheme[20] = 0; ui13->uControlTheme[21] = 0; ui13->uControlTheme[22] = 0; ui13->uControlTheme[23] = 0; ui13->uControlTheme[24] = 0; ui13->uControlTheme[25] = 0; ui13->uControlTheme[26] = 0; ui13->uControlTheme[27] = 0; ui13->uControlTheme[28] = 0; ui13->uControlTheme[29] = 0; ui13->uControlTheme[30] = 0; ui13->uControlTheme[31] = 0; 
	ui13->uFluxCapControl[0] = 0; ui13->uFluxCapControl[1] = 0; ui13->uFluxCapControl[2] = 0; ui13->uFluxCapControl[3] = 0; ui13->uFluxCapControl[4] = 0; ui13->uFluxCapControl[5] = 0; ui13->uFluxCapControl[6] = 0; ui13->uFluxCapControl[7] = 0; ui13->uFluxCapControl[8] = 0; ui13->uFluxCapControl[9] = 0; ui13->uFluxCapControl[10] = 0; ui13->uFluxCapControl[11] = 0; ui13->uFluxCapControl[12] = 0; ui13->uFluxCapControl[13] = 0; ui13->uFluxCapControl[14] = 0; ui13->uFluxCapControl[15] = 0; ui13->uFluxCapControl[16] = 0; ui13->uFluxCapControl[17] = 0; ui13->uFluxCapControl[18] = 0; ui13->uFluxCapControl[19] = 0; ui13->uFluxCapControl[20] = 0; ui13->uFluxCapControl[21] = 0; ui13->uFluxCapControl[22] = 0; ui13->uFluxCapControl[23] = 0; ui13->uFluxCapControl[24] = 0; ui13->uFluxCapControl[25] = 0; ui13->uFluxCapControl[26] = 0; ui13->uFluxCapControl[27] = 0; ui13->uFluxCapControl[28] = 0; ui13->uFluxCapControl[29] = 0; ui13->uFluxCapControl[30] = 0; ui13->uFluxCapControl[31] = 0; ui13->uFluxCapControl[32] = 0; ui13->uFluxCapControl[33] = 0; ui13->uFluxCapControl[34] = 0; ui13->uFluxCapControl[35] = 0; ui13->uFluxCapControl[36] = 0; ui13->uFluxCapControl[37] = 0; ui13->uFluxCapControl[38] = 0; ui13->uFluxCapControl[39] = 0; ui13->uFluxCapControl[40] = 0; ui13->uFluxCapControl[41] = 0; ui13->uFluxCapControl[42] = 0; ui13->uFluxCapControl[43] = 0; ui13->uFluxCapControl[44] = 0; ui13->uFluxCapControl[45] = 0; ui13->uFluxCapControl[46] = 0; ui13->uFluxCapControl[47] = 0; ui13->uFluxCapControl[48] = 0; ui13->uFluxCapControl[49] = 0; ui13->uFluxCapControl[50] = 0; ui13->uFluxCapControl[51] = 0; ui13->uFluxCapControl[52] = 0; ui13->uFluxCapControl[53] = 0; ui13->uFluxCapControl[54] = 0; ui13->uFluxCapControl[55] = 0; ui13->uFluxCapControl[56] = 0; ui13->uFluxCapControl[57] = 0; ui13->uFluxCapControl[58] = 0; ui13->uFluxCapControl[59] = 0; ui13->uFluxCapControl[60] = 0; ui13->uFluxCapControl[61] = 0; ui13->uFluxCapControl[62] = 0; ui13->uFluxCapControl[63] = 0; 
	ui13->fFluxCapData[0] = 0.000000; ui13->fFluxCapData[1] = 0.000000; ui13->fFluxCapData[2] = 0.000000; ui13->fFluxCapData[3] = 0.000000; ui13->fFluxCapData[4] = 0.000000; ui13->fFluxCapData[5] = 0.000000; ui13->fFluxCapData[6] = 0.000000; ui13->fFluxCapData[7] = 0.000000; ui13->fFluxCapData[8] = 0.000000; ui13->fFluxCapData[9] = 0.000000; ui13->fFluxCapData[10] = 0.000000; ui13->fFluxCapData[11] = 0.000000; ui13->fFluxCapData[12] = 0.000000; ui13->fFluxCapData[13] = 0.000000; ui13->fFluxCapData[14] = 0.000000; ui13->fFluxCapData[15] = 0.000000; ui13->fFluxCapData[16] = 0.000000; ui13->fFluxCapData[17] = 0.000000; ui13->fFluxCapData[18] = 0.000000; ui13->fFluxCapData[19] = 0.000000; ui13->fFluxCapData[20] = 0.000000; ui13->fFluxCapData[21] = 0.000000; ui13->fFluxCapData[22] = 0.000000; ui13->fFluxCapData[23] = 0.000000; ui13->fFluxCapData[24] = 0.000000; ui13->fFluxCapData[25] = 0.000000; ui13->fFluxCapData[26] = 0.000000; ui13->fFluxCapData[27] = 0.000000; ui13->fFluxCapData[28] = 0.000000; ui13->fFluxCapData[29] = 0.000000; ui13->fFluxCapData[30] = 0.000000; ui13->fFluxCapData[31] = 0.000000; ui13->fFluxCapData[32] = 0.000000; ui13->fFluxCapData[33] = 0.000000; ui13->fFluxCapData[34] = 0.000000; ui13->fFluxCapData[35] = 0.000000; ui13->fFluxCapData[36] = 0.000000; ui13->fFluxCapData[37] = 0.000000; ui13->fFluxCapData[38] = 0.000000; ui13->fFluxCapData[39] = 0.000000; ui13->fFluxCapData[40] = 0.000000; ui13->fFluxCapData[41] = 0.000000; ui13->fFluxCapData[42] = 0.000000; ui13->fFluxCapData[43] = 0.000000; ui13->fFluxCapData[44] = 0.000000; ui13->fFluxCapData[45] = 0.000000; ui13->fFluxCapData[46] = 0.000000; ui13->fFluxCapData[47] = 0.000000; ui13->fFluxCapData[48] = 0.000000; ui13->fFluxCapData[49] = 0.000000; ui13->fFluxCapData[50] = 0.000000; ui13->fFluxCapData[51] = 0.000000; ui13->fFluxCapData[52] = 0.000000; ui13->fFluxCapData[53] = 0.000000; ui13->fFluxCapData[54] = 0.000000; ui13->fFluxCapData[55] = 0.000000; ui13->fFluxCapData[56] = 0.000000; ui13->fFluxCapData[57] = 0.000000; ui13->fFluxCapData[58] = 0.000000; ui13->fFluxCapData[59] = 0.000000; ui13->fFluxCapData[60] = 0.000000; ui13->fFluxCapData[61] = 0.000000; ui13->fFluxCapData[62] = 0.000000; ui13->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui13);
	delete ui13;


	m_fRate = 0.020000;
	CUICtrl* ui14 = new CUICtrl;
	ui14->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui14->uControlId = 25;
	ui14->bLogSlider = false;
	ui14->bExpSlider = true;
	ui14->fUserDisplayDataLoLimit = 0.020000;
	ui14->fUserDisplayDataHiLimit = 20.000000;
	ui14->uUserDataType = floatData;
	ui14->fInitUserIntValue = 0;
	ui14->fInitUserFloatValue = 0.020000;
	ui14->fInitUserDoubleValue = 0;
	ui14->fInitUserUINTValue = 0;
	ui14->m_pUserCookedIntData = NULL;
	ui14->m_pUserCookedFloatData = &m_fRate;
	ui14->m_pUserCookedDoubleData = NULL;
	ui14->m_pUserCookedUINTData = NULL;
	ui14->cControlUnits = "Hz";
	ui14->cVariableName = "m_fRate";
	ui14->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui14->dPresetData[0] = 0.020000;ui14->dPresetData[1] = 0.000000;ui14->dPresetData[2] = 0.000000;ui14->dPresetData[3] = 0.000000;ui14->dPresetData[4] = 0.000000;ui14->dPresetData[5] = 0.000000;ui14->dPresetData[6] = 0.000000;ui14->dPresetData[7] = 0.000000;ui14->dPresetData[8] = 0.000000;ui14->dPresetData[9] = 0.000000;ui14->dPresetData[10] = 0.000000;ui14->dPresetData[11] = 0.000000;ui14->dPresetData[12] = 0.000000;ui14->dPresetData[13] = 0.000000;ui14->dPresetData[14] = 0.000000;ui14->dPresetData[15] = 0.000000;
	ui14->cControlName = "Rate";
	ui14->bOwnerControl = false;
	ui14->bMIDIControl = false;
	ui14->uMIDIControlCommand = 176;
	ui14->uMIDIControlName = 3;
	ui14->uMIDIControlChannel = 0;
	ui14->nGUIRow = nIndexer++;
	ui14->nGUIColumn = -1;
	ui14->bEnableParamSmoothing = true;
	ui14->fSmoothingTimeInMs = 100.00;
	ui14->uControlTheme[0] = 0; ui14->uControlTheme[1] = 0; ui14->uControlTheme[2] = 0; ui14->uControlTheme[3] = 0; ui14->uControlTheme[4] = 0; ui14->uControlTheme[5] = 0; ui14->uControlTheme[6] = 0; ui14->uControlTheme[7] = 0; ui14->uControlTheme[8] = 12; ui14->uControlTheme[9] = 0; ui14->uControlTheme[10] = 0; ui14->uControlTheme[11] = 0; ui14->uControlTheme[12] = 0; ui14->uControlTheme[13] = 0; ui14->uControlTheme[14] = 0; ui14->uControlTheme[15] = 0; ui14->uControlTheme[16] = 2; ui14->uControlTheme[17] = 0; ui14->uControlTheme[18] = 0; ui14->uControlTheme[19] = 0; ui14->uControlTheme[20] = 0; ui14->uControlTheme[21] = 0; ui14->uControlTheme[22] = 0; ui14->uControlTheme[23] = 0; ui14->uControlTheme[24] = 0; ui14->uControlTheme[25] = 0; ui14->uControlTheme[26] = 0; ui14->uControlTheme[27] = 0; ui14->uControlTheme[28] = 0; ui14->uControlTheme[29] = 0; ui14->uControlTheme[30] = 0; ui14->uControlTheme[31] = 0; 
	ui14->uFluxCapControl[0] = 0; ui14->uFluxCapControl[1] = 0; ui14->uFluxCapControl[2] = 0; ui14->uFluxCapControl[3] = 0; ui14->uFluxCapControl[4] = 0; ui14->uFluxCapControl[5] = 0; ui14->uFluxCapControl[6] = 0; ui14->uFluxCapControl[7] = 0; ui14->uFluxCapControl[8] = 0; ui14->uFluxCapControl[9] = 0; ui14->uFluxCapControl[10] = 0; ui14->uFluxCapControl[11] = 0; ui14->uFluxCapControl[12] = 0; ui14->uFluxCapControl[13] = 0; ui14->uFluxCapControl[14] = 0; ui14->uFluxCapControl[15] = 0; ui14->uFluxCapControl[16] = 0; ui14->uFluxCapControl[17] = 0; ui14->uFluxCapControl[18] = 0; ui14->uFluxCapControl[19] = 0; ui14->uFluxCapControl[20] = 0; ui14->uFluxCapControl[21] = 0; ui14->uFluxCapControl[22] = 0; ui14->uFluxCapControl[23] = 0; ui14->uFluxCapControl[24] = 0; ui14->uFluxCapControl[25] = 0; ui14->uFluxCapControl[26] = 0; ui14->uFluxCapControl[27] = 0; ui14->uFluxCapControl[28] = 0; ui14->uFluxCapControl[29] = 0; ui14->uFluxCapControl[30] = 0; ui14->uFluxCapControl[31] = 0; ui14->uFluxCapControl[32] = 0; ui14->uFluxCapControl[33] = 0; ui14->uFluxCapControl[34] = 0; ui14->uFluxCapControl[35] = 0; ui14->uFluxCapControl[36] = 0; ui14->uFluxCapControl[37] = 0; ui14->uFluxCapControl[38] = 0; ui14->uFluxCapControl[39] = 0; ui14->uFluxCapControl[40] = 0; ui14->uFluxCapControl[41] = 0; ui14->uFluxCapControl[42] = 0; ui14->uFluxCapControl[43] = 0; ui14->uFluxCapControl[44] = 0; ui14->uFluxCapControl[45] = 0; ui14->uFluxCapControl[46] = 0; ui14->uFluxCapControl[47] = 0; ui14->uFluxCapControl[48] = 0; ui14->uFluxCapControl[49] = 0; ui14->uFluxCapControl[50] = 0; ui14->uFluxCapControl[51] = 0; ui14->uFluxCapControl[52] = 0; ui14->uFluxCapControl[53] = 0; ui14->uFluxCapControl[54] = 0; ui14->uFluxCapControl[55] = 0; ui14->uFluxCapControl[56] = 0; ui14->uFluxCapControl[57] = 0; ui14->uFluxCapControl[58] = 0; ui14->uFluxCapControl[59] = 0; ui14->uFluxCapControl[60] = 0; ui14->uFluxCapControl[61] = 0; ui14->uFluxCapControl[62] = 0; ui14->uFluxCapControl[63] = 0; 
	ui14->fFluxCapData[0] = 0.000000; ui14->fFluxCapData[1] = 0.000000; ui14->fFluxCapData[2] = 0.000000; ui14->fFluxCapData[3] = 0.000000; ui14->fFluxCapData[4] = 0.000000; ui14->fFluxCapData[5] = 0.000000; ui14->fFluxCapData[6] = 0.000000; ui14->fFluxCapData[7] = 0.000000; ui14->fFluxCapData[8] = 0.000000; ui14->fFluxCapData[9] = 0.000000; ui14->fFluxCapData[10] = 0.000000; ui14->fFluxCapData[11] = 0.000000; ui14->fFluxCapData[12] = 0.000000; ui14->fFluxCapData[13] = 0.000000; ui14->fFluxCapData[14] = 0.000000; ui14->fFluxCapData[15] = 0.000000; ui14->fFluxCapData[16] = 0.000000; ui14->fFluxCapData[17] = 0.000000; ui14->fFluxCapData[18] = 0.000000; ui14->fFluxCapData[19] = 0.000000; ui14->fFluxCapData[20] = 0.000000; ui14->fFluxCapData[21] = 0.000000; ui14->fFluxCapData[22] = 0.000000; ui14->fFluxCapData[23] = 0.000000; ui14->fFluxCapData[24] = 0.000000; ui14->fFluxCapData[25] = 0.000000; ui14->fFluxCapData[26] = 0.000000; ui14->fFluxCapData[27] = 0.000000; ui14->fFluxCapData[28] = 0.000000; ui14->fFluxCapData[29] = 0.000000; ui14->fFluxCapData[30] = 0.000000; ui14->fFluxCapData[31] = 0.000000; ui14->fFluxCapData[32] = 0.000000; ui14->fFluxCapData[33] = 0.000000; ui14->fFluxCapData[34] = 0.000000; ui14->fFluxCapData[35] = 0.000000; ui14->fFluxCapData[36] = 0.000000; ui14->fFluxCapData[37] = 0.000000; ui14->fFluxCapData[38] = 0.000000; ui14->fFluxCapData[39] = 0.000000; ui14->fFluxCapData[40] = 0.000000; ui14->fFluxCapData[41] = 0.000000; ui14->fFluxCapData[42] = 0.000000; ui14->fFluxCapData[43] = 0.000000; ui14->fFluxCapData[44] = 0.000000; ui14->fFluxCapData[45] = 0.000000; ui14->fFluxCapData[46] = 0.000000; ui14->fFluxCapData[47] = 0.000000; ui14->fFluxCapData[48] = 0.000000; ui14->fFluxCapData[49] = 0.000000; ui14->fFluxCapData[50] = 0.000000; ui14->fFluxCapData[51] = 0.000000; ui14->fFluxCapData[52] = 0.000000; ui14->fFluxCapData[53] = 0.000000; ui14->fFluxCapData[54] = 0.000000; ui14->fFluxCapData[55] = 0.000000; ui14->fFluxCapData[56] = 0.000000; ui14->fFluxCapData[57] = 0.000000; ui14->fFluxCapData[58] = 0.000000; ui14->fFluxCapData[59] = 0.000000; ui14->fFluxCapData[60] = 0.000000; ui14->fFluxCapData[61] = 0.000000; ui14->fFluxCapData[62] = 0.000000; ui14->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui14);
	delete ui14;


	m_fDepth_pct = 50.000000;
	CUICtrl* ui15 = new CUICtrl;
	ui15->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui15->uControlId = 26;
	ui15->bLogSlider = false;
	ui15->bExpSlider = false;
	ui15->fUserDisplayDataLoLimit = 0.000000;
	ui15->fUserDisplayDataHiLimit = 100.000000;
	ui15->uUserDataType = floatData;
	ui15->fInitUserIntValue = 0;
	ui15->fInitUserFloatValue = 50.000000;
	ui15->fInitUserDoubleValue = 0;
	ui15->fInitUserUINTValue = 0;
	ui15->m_pUserCookedIntData = NULL;
	ui15->m_pUserCookedFloatData = &m_fDepth_pct;
	ui15->m_pUserCookedDoubleData = NULL;
	ui15->m_pUserCookedUINTData = NULL;
	ui15->cControlUnits = "%";
	ui15->cVariableName = "m_fDepth_pct";
	ui15->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui15->dPresetData[0] = 50.000000;ui15->dPresetData[1] = 0.000000;ui15->dPresetData[2] = 0.000000;ui15->dPresetData[3] = 0.000000;ui15->dPresetData[4] = 0.000000;ui15->dPresetData[5] = 0.000000;ui15->dPresetData[6] = 0.000000;ui15->dPresetData[7] = 0.000000;ui15->dPresetData[8] = 0.000000;ui15->dPresetData[9] = 0.000000;ui15->dPresetData[10] = 0.000000;ui15->dPresetData[11] = 0.000000;ui15->dPresetData[12] = 0.000000;ui15->dPresetData[13] = 0.000000;ui15->dPresetData[14] = 0.000000;ui15->dPresetData[15] = 0.000000;
	ui15->cControlName = "Depth";
	ui15->bOwnerControl = false;
	ui15->bMIDIControl = false;
	ui15->uMIDIControlCommand = 176;
	ui15->uMIDIControlName = 3;
	ui15->uMIDIControlChannel = 0;
	ui15->nGUIRow = nIndexer++;
	ui15->nGUIColumn = -1;
	ui15->bEnableParamSmoothing = false;
	ui15->fSmoothingTimeInMs = 100.00;
	ui15->uControlTheme[0] = 0; ui15->uControlTheme[1] = 0; ui15->uControlTheme[2] = 0; ui15->uControlTheme[3] = 0; ui15->uControlTheme[4] = 0; ui15->uControlTheme[5] = 0; ui15->uControlTheme[6] = 0; ui15->uControlTheme[7] = 0; ui15->uControlTheme[8] = 12; ui15->uControlTheme[9] = 0; ui15->uControlTheme[10] = 0; ui15->uControlTheme[11] = 0; ui15->uControlTheme[12] = 0; ui15->uControlTheme[13] = 0; ui15->uControlTheme[14] = 0; ui15->uControlTheme[15] = 0; ui15->uControlTheme[16] = 2; ui15->uControlTheme[17] = 0; ui15->uControlTheme[18] = 0; ui15->uControlTheme[19] = 0; ui15->uControlTheme[20] = 0; ui15->uControlTheme[21] = 0; ui15->uControlTheme[22] = 0; ui15->uControlTheme[23] = 0; ui15->uControlTheme[24] = 0; ui15->uControlTheme[25] = 0; ui15->uControlTheme[26] = 0; ui15->uControlTheme[27] = 0; ui15->uControlTheme[28] = 0; ui15->uControlTheme[29] = 0; ui15->uControlTheme[30] = 0; ui15->uControlTheme[31] = 0; 
	ui15->uFluxCapControl[0] = 0; ui15->uFluxCapControl[1] = 0; ui15->uFluxCapControl[2] = 0; ui15->uFluxCapControl[3] = 0; ui15->uFluxCapControl[4] = 0; ui15->uFluxCapControl[5] = 0; ui15->uFluxCapControl[6] = 0; ui15->uFluxCapControl[7] = 0; ui15->uFluxCapControl[8] = 0; ui15->uFluxCapControl[9] = 0; ui15->uFluxCapControl[10] = 0; ui15->uFluxCapControl[11] = 0; ui15->uFluxCapControl[12] = 0; ui15->uFluxCapControl[13] = 0; ui15->uFluxCapControl[14] = 0; ui15->uFluxCapControl[15] = 0; ui15->uFluxCapControl[16] = 0; ui15->uFluxCapControl[17] = 0; ui15->uFluxCapControl[18] = 0; ui15->uFluxCapControl[19] = 0; ui15->uFluxCapControl[20] = 0; ui15->uFluxCapControl[21] = 0; ui15->uFluxCapControl[22] = 0; ui15->uFluxCapControl[23] = 0; ui15->uFluxCapControl[24] = 0; ui15->uFluxCapControl[25] = 0; ui15->uFluxCapControl[26] = 0; ui15->uFluxCapControl[27] = 0; ui15->uFluxCapControl[28] = 0; ui15->uFluxCapControl[29] = 0; ui15->uFluxCapControl[30] = 0; ui15->uFluxCapControl[31] = 0; ui15->uFluxCapControl[32] = 0; ui15->uFluxCapControl[33] = 0; ui15->uFluxCapControl[34] = 0; ui15->uFluxCapControl[35] = 0; ui15->uFluxCapControl[36] = 0; ui15->uFluxCapControl[37] = 0; ui15->uFluxCapControl[38] = 0; ui15->uFluxCapControl[39] = 0; ui15->uFluxCapControl[40] = 0; ui15->uFluxCapControl[41] = 0; ui15->uFluxCapControl[42] = 0; ui15->uFluxCapControl[43] = 0; ui15->uFluxCapControl[44] = 0; ui15->uFluxCapControl[45] = 0; ui15->uFluxCapControl[46] = 0; ui15->uFluxCapControl[47] = 0; ui15->uFluxCapControl[48] = 0; ui15->uFluxCapControl[49] = 0; ui15->uFluxCapControl[50] = 0; ui15->uFluxCapControl[51] = 0; ui15->uFluxCapControl[52] = 0; ui15->uFluxCapControl[53] = 0; ui15->uFluxCapControl[54] = 0; ui15->uFluxCapControl[55] = 0; ui15->uFluxCapControl[56] = 0; ui15->uFluxCapControl[57] = 0; ui15->uFluxCapControl[58] = 0; ui15->uFluxCapControl[59] = 0; ui15->uFluxCapControl[60] = 0; ui15->uFluxCapControl[61] = 0; ui15->uFluxCapControl[62] = 0; ui15->uFluxCapControl[63] = 0; 
	ui15->fFluxCapData[0] = 0.000000; ui15->fFluxCapData[1] = 0.000000; ui15->fFluxCapData[2] = 0.000000; ui15->fFluxCapData[3] = 0.000000; ui15->fFluxCapData[4] = 0.000000; ui15->fFluxCapData[5] = 0.000000; ui15->fFluxCapData[6] = 0.000000; ui15->fFluxCapData[7] = 0.000000; ui15->fFluxCapData[8] = 0.000000; ui15->fFluxCapData[9] = 0.000000; ui15->fFluxCapData[10] = 0.000000; ui15->fFluxCapData[11] = 0.000000; ui15->fFluxCapData[12] = 0.000000; ui15->fFluxCapData[13] = 0.000000; ui15->fFluxCapData[14] = 0.000000; ui15->fFluxCapData[15] = 0.000000; ui15->fFluxCapData[16] = 0.000000; ui15->fFluxCapData[17] = 0.000000; ui15->fFluxCapData[18] = 0.000000; ui15->fFluxCapData[19] = 0.000000; ui15->fFluxCapData[20] = 0.000000; ui15->fFluxCapData[21] = 0.000000; ui15->fFluxCapData[22] = 0.000000; ui15->fFluxCapData[23] = 0.000000; ui15->fFluxCapData[24] = 0.000000; ui15->fFluxCapData[25] = 0.000000; ui15->fFluxCapData[26] = 0.000000; ui15->fFluxCapData[27] = 0.000000; ui15->fFluxCapData[28] = 0.000000; ui15->fFluxCapData[29] = 0.000000; ui15->fFluxCapData[30] = 0.000000; ui15->fFluxCapData[31] = 0.000000; ui15->fFluxCapData[32] = 0.000000; ui15->fFluxCapData[33] = 0.000000; ui15->fFluxCapData[34] = 0.000000; ui15->fFluxCapData[35] = 0.000000; ui15->fFluxCapData[36] = 0.000000; ui15->fFluxCapData[37] = 0.000000; ui15->fFluxCapData[38] = 0.000000; ui15->fFluxCapData[39] = 0.000000; ui15->fFluxCapData[40] = 0.000000; ui15->fFluxCapData[41] = 0.000000; ui15->fFluxCapData[42] = 0.000000; ui15->fFluxCapData[43] = 0.000000; ui15->fFluxCapData[44] = 0.000000; ui15->fFluxCapData[45] = 0.000000; ui15->fFluxCapData[46] = 0.000000; ui15->fFluxCapData[47] = 0.000000; ui15->fFluxCapData[48] = 0.000000; ui15->fFluxCapData[49] = 0.000000; ui15->fFluxCapData[50] = 0.000000; ui15->fFluxCapData[51] = 0.000000; ui15->fFluxCapData[52] = 0.000000; ui15->fFluxCapData[53] = 0.000000; ui15->fFluxCapData[54] = 0.000000; ui15->fFluxCapData[55] = 0.000000; ui15->fFluxCapData[56] = 0.000000; ui15->fFluxCapData[57] = 0.000000; ui15->fFluxCapData[58] = 0.000000; ui15->fFluxCapData[59] = 0.000000; ui15->fFluxCapData[60] = 0.000000; ui15->fFluxCapData[61] = 0.000000; ui15->fFluxCapData[62] = 0.000000; ui15->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui15);
	delete ui15;


	m_fResonance_pct = 50.000000;
	CUICtrl* ui16 = new CUICtrl;
	ui16->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui16->uControlId = 27;
	ui16->bLogSlider = false;
	ui16->bExpSlider = false;
	ui16->fUserDisplayDataLoLimit = 0.000000;
	ui16->fUserDisplayDataHiLimit = 100.000000;
	ui16->uUserDataType = floatData;
	ui16->fInitUserIntValue = 0;
	ui16->fInitUserFloatValue = 50.000000;
	ui16->fInitUserDoubleValue = 0;
	ui16->fInitUserUINTValue = 0;
	ui16->m_pUserCookedIntData = NULL;
	ui16->m_pUserCookedFloatData = &m_fResonance_pct;
	ui16->m_pUserCookedDoubleData = NULL;
	ui16->m_pUserCookedUINTData = NULL;
	ui16->cControlUnits = "%";
	ui16->cVariableName = "m_fResonance_pct";
	ui16->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui16->dPresetData[0] = 50.000000;ui16->dPresetData[1] = 0.000000;ui16->dPresetData[2] = 0.000000;ui16->dPresetData[3] = 0.000000;ui16->dPresetData[4] = 0.000000;ui16->dPresetData[5] = 0.000000;ui16->dPresetData[6] = 0.000000;ui16->dPresetData[7] = 0.000000;ui16->dPresetData[8] = 0.000000;ui16->dPresetData[9] = 0.000000;ui16->dPresetData[10] = 0.000000;ui16->dPresetData[11] = 0.000000;ui16->dPresetData[12] = 0.000000;ui16->dPresetData[13] = 0.000000;ui16->dPresetData[14] = 0.000000;ui16->dPresetData[15] = 0.000000;
	ui16->cControlName = "Resonance";
	ui16->bOwnerControl = false;
	ui16->bMIDIControl = false;
	ui16->uMIDIControlCommand = 176;
	ui16->uMIDIControlName = 3;
	ui16->uMIDIControlChannel = 0;
	ui16->nGUIRow = nIndexer++;
	ui16->nGUIColumn = -1;
	ui16->bEnableParamSmoothing = false;
	ui16->fSmoothingTimeInMs = 100.00;
	ui16->uControlTheme[0] = 0; ui16->uControlTheme[1] = 0; ui16->uControlTheme[2] = 0; ui16->uControlTheme[3] = 0; ui16->uControlTheme[4] = 0; ui16->uControlTheme[5] = 0; ui16->uControlTheme[6] = 0; ui16->uControlTheme[7] = 0; ui16->uControlTheme[8] = 12; ui16->uControlTheme[9] = 0; ui16->uControlTheme[10] = 0; ui16->uControlTheme[11] = 0; ui16->uControlTheme[12] = 0; ui16->uControlTheme[13] = 0; ui16->uControlTheme[14] = 0; ui16->uControlTheme[15] = 0; ui16->uControlTheme[16] = 2; ui16->uControlTheme[17] = 0; ui16->uControlTheme[18] = 0; ui16->uControlTheme[19] = 0; ui16->uControlTheme[20] = 0; ui16->uControlTheme[21] = 0; ui16->uControlTheme[22] = 0; ui16->uControlTheme[23] = 0; ui16->uControlTheme[24] = 0; ui16->uControlTheme[25] = 0; ui16->uControlTheme[26] = 0; ui16->uControlTheme[27] = 0; ui16->uControlTheme[28] = 0; ui16->uControlTheme[29] = 0; ui16->uControlTheme[30] = 0; ui16->uControlTheme[31] = 0; 
	ui16->uFluxCapControl[0] = 0; ui16->uFluxCapControl[1] = 0; ui16->uFluxCapControl[2] = 0; ui16->uFluxCapControl[3] = 0; ui16->uFluxCapControl[4] = 0; ui16->uFluxCapControl[5] = 0; ui16->uFluxCapControl[6] = 0; ui16->uFluxCapControl[7] = 0; ui16->uFluxCapControl[8] = 0; ui16->uFluxCapControl[9] = 0; ui16->uFluxCapControl[10] = 0; ui16->uFluxCapControl[11] = 0; ui16->uFluxCapControl[12] = 0; ui16->uFluxCapControl[13] = 0; ui16->uFluxCapControl[14] = 0; ui16->uFluxCapControl[15] = 0; ui16->uFluxCapControl[16] = 0; ui16->uFluxCapControl[17] = 0; ui16->uFluxCapControl[18] = 0; ui16->uFluxCapControl[19] = 0; ui16->uFluxCapControl[20] = 0; ui16->uFluxCapControl[21] = 0; ui16->uFluxCapControl[22] = 0; ui16->uFluxCapControl[23] = 0; ui16->uFluxCapControl[24] = 0; ui16->uFluxCapControl[25] = 0; ui16->uFluxCapControl[26] = 0; ui16->uFluxCapControl[27] = 0; ui16->uFluxCapControl[28] = 0; ui16->uFluxCapControl[29] = 0; ui16->uFluxCapControl[30] = 0; ui16->uFluxCapControl[31] = 0; ui16->uFluxCapControl[32] = 0; ui16->uFluxCapControl[33] = 0; ui16->uFluxCapControl[34] = 0; ui16->uFluxCapControl[35] = 0; ui16->uFluxCapControl[36] = 0; ui16->uFluxCapControl[37] = 0; ui16->uFluxCapControl[38] = 0; ui16->uFluxCapControl[39] = 0; ui16->uFluxCapControl[40] = 0; ui16->uFluxCapControl[41] = 0; ui16->uFluxCapControl[42] = 0; ui16->uFluxCapControl[43] = 0; ui16->uFluxCapControl[44] = 0; ui16->uFluxCapControl[45] = 0; ui16->uFluxCapControl[46] = 0; ui16->uFluxCapControl[47] = 0; ui16->uFluxCapControl[48] = 0; ui16->uFluxCapControl[49] = 0; ui16->uFluxCapControl[50] = 0; ui16->uFluxCapControl[51] = 0; ui16->uFluxCapControl[52] = 0; ui16->uFluxCapControl[53] = 0; ui16->uFluxCapControl[54] = 0; ui16->uFluxCapControl[55] = 0; ui16->uFluxCapControl[56] = 0; ui16->uFluxCapControl[57] = 0; ui16->uFluxCapControl[58] = 0; ui16->uFluxCapControl[59] = 0; ui16->uFluxCapControl[60] = 0; ui16->uFluxCapControl[61] = 0; ui16->uFluxCapControl[62] = 0; ui16->uFluxCapControl[63] = 0; 
	ui16->fFluxCapData[0] = 0.000000; ui16->fFluxCapData[1] = 0.000000; ui16->fFluxCapData[2] = 0.000000; ui16->fFluxCapData[3] = 0.000000; ui16->fFluxCapData[4] = 0.000000; ui16->fFluxCapData[5] = 0.000000; ui16->fFluxCapData[6] = 0.000000; ui16->fFluxCapData[7] = 0.000000; ui16->fFluxCapData[8] = 0.000000; ui16->fFluxCapData[9] = 0.000000; ui16->fFluxCapData[10] = 0.000000; ui16->fFluxCapData[11] = 0.000000; ui16->fFluxCapData[12] = 0.000000; ui16->fFluxCapData[13] = 0.000000; ui16->fFluxCapData[14] = 0.000000; ui16->fFluxCapData[15] = 0.000000; ui16->fFluxCapData[16] = 0.000000; ui16->fFluxCapData[17] = 0.000000; ui16->fFluxCapData[18] = 0.000000; ui16->fFluxCapData[19] = 0.000000; ui16->fFluxCapData[20] = 0.000000; ui16->fFluxCapData[21] = 0.000000; ui16->fFluxCapData[22] = 0.000000; ui16->fFluxCapData[23] = 0.000000; ui16->fFluxCapData[24] = 0.000000; ui16->fFluxCapData[25] = 0.000000; ui16->fFluxCapData[26] = 0.000000; ui16->fFluxCapData[27] = 0.000000; ui16->fFluxCapData[28] = 0.000000; ui16->fFluxCapData[29] = 0.000000; ui16->fFluxCapData[30] = 0.000000; ui16->fFluxCapData[31] = 0.000000; ui16->fFluxCapData[32] = 0.000000; ui16->fFluxCapData[33] = 0.000000; ui16->fFluxCapData[34] = 0.000000; ui16->fFluxCapData[35] = 0.000000; ui16->fFluxCapData[36] = 0.000000; ui16->fFluxCapData[37] = 0.000000; ui16->fFluxCapData[38] = 0.000000; ui16->fFluxCapData[39] = 0.000000; ui16->fFluxCapData[40] = 0.000000; ui16->fFluxCapData[41] = 0.000000; ui16->fFluxCapData[42] = 0.000000; ui16->fFluxCapData[43] = 0.000000; ui16->fFluxCapData[44] = 0.000000; ui16->fFluxCapData[45] = 0.000000; ui16->fFluxCapData[46] = 0.000000; ui16->fFluxCapData[47] = 0.000000; ui16->fFluxCapData[48] = 0.000000; ui16->fFluxCapData[49] = 0.000000; ui16->fFluxCapData[50] = 0.000000; ui16->fFluxCapData[51] = 0.000000; ui16->fFluxCapData[52] = 0.000000; ui16->fFluxCapData[53] = 0.000000; ui16->fFluxCapData[54] = 0.000000; ui16->fFluxCapData[55] = 0.000000; ui16->fFluxCapData[56] = 0.000000; ui16->fFluxCapData[57] = 0.000000; ui16->fFluxCapData[58] = 0.000000; ui16->fFluxCapData[59] = 0.000000; ui16->fFluxCapData[60] = 0.000000; ui16->fFluxCapData[61] = 0.000000; ui16->fFluxCapData[62] = 0.000000; ui16->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui16);
	delete ui16;


	m_fWet_pct_modDelay = 50.000000;
	CUICtrl* ui17 = new CUICtrl;
	ui17->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui17->uControlId = 28;
	ui17->bLogSlider = false;
	ui17->bExpSlider = false;
	ui17->fUserDisplayDataLoLimit = 0.000000;
	ui17->fUserDisplayDataHiLimit = 100.000000;
	ui17->uUserDataType = floatData;
	ui17->fInitUserIntValue = 0;
	ui17->fInitUserFloatValue = 50.000000;
	ui17->fInitUserDoubleValue = 0;
	ui17->fInitUserUINTValue = 0;
	ui17->m_pUserCookedIntData = NULL;
	ui17->m_pUserCookedFloatData = &m_fWet_pct_modDelay;
	ui17->m_pUserCookedDoubleData = NULL;
	ui17->m_pUserCookedUINTData = NULL;
	ui17->cControlUnits = "%";
	ui17->cVariableName = "m_fWet_pct_modDelay";
	ui17->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui17->dPresetData[0] = 50.000000;ui17->dPresetData[1] = 0.000000;ui17->dPresetData[2] = 0.000000;ui17->dPresetData[3] = 0.000000;ui17->dPresetData[4] = 0.000000;ui17->dPresetData[5] = 0.000000;ui17->dPresetData[6] = 0.000000;ui17->dPresetData[7] = 0.000000;ui17->dPresetData[8] = 0.000000;ui17->dPresetData[9] = 0.000000;ui17->dPresetData[10] = 0.000000;ui17->dPresetData[11] = 0.000000;ui17->dPresetData[12] = 0.000000;ui17->dPresetData[13] = 0.000000;ui17->dPresetData[14] = 0.000000;ui17->dPresetData[15] = 0.000000;
	ui17->cControlName = "Wet";
	ui17->bOwnerControl = false;
	ui17->bMIDIControl = false;
	ui17->uMIDIControlCommand = 176;
	ui17->uMIDIControlName = 3;
	ui17->uMIDIControlChannel = 0;
	ui17->nGUIRow = nIndexer++;
	ui17->nGUIColumn = -1;
	ui17->bEnableParamSmoothing = true;
	ui17->fSmoothingTimeInMs = 100.00;
	ui17->uControlTheme[0] = 0; ui17->uControlTheme[1] = 0; ui17->uControlTheme[2] = 0; ui17->uControlTheme[3] = 0; ui17->uControlTheme[4] = 0; ui17->uControlTheme[5] = 0; ui17->uControlTheme[6] = 0; ui17->uControlTheme[7] = 0; ui17->uControlTheme[8] = 12; ui17->uControlTheme[9] = 0; ui17->uControlTheme[10] = 0; ui17->uControlTheme[11] = 0; ui17->uControlTheme[12] = 0; ui17->uControlTheme[13] = 0; ui17->uControlTheme[14] = 0; ui17->uControlTheme[15] = 0; ui17->uControlTheme[16] = 2; ui17->uControlTheme[17] = 0; ui17->uControlTheme[18] = 0; ui17->uControlTheme[19] = 0; ui17->uControlTheme[20] = 0; ui17->uControlTheme[21] = 0; ui17->uControlTheme[22] = 0; ui17->uControlTheme[23] = 0; ui17->uControlTheme[24] = 0; ui17->uControlTheme[25] = 0; ui17->uControlTheme[26] = 0; ui17->uControlTheme[27] = 0; ui17->uControlTheme[28] = 0; ui17->uControlTheme[29] = 0; ui17->uControlTheme[30] = 0; ui17->uControlTheme[31] = 0; 
	ui17->uFluxCapControl[0] = 0; ui17->uFluxCapControl[1] = 0; ui17->uFluxCapControl[2] = 0; ui17->uFluxCapControl[3] = 0; ui17->uFluxCapControl[4] = 0; ui17->uFluxCapControl[5] = 0; ui17->uFluxCapControl[6] = 0; ui17->uFluxCapControl[7] = 0; ui17->uFluxCapControl[8] = 0; ui17->uFluxCapControl[9] = 0; ui17->uFluxCapControl[10] = 0; ui17->uFluxCapControl[11] = 0; ui17->uFluxCapControl[12] = 0; ui17->uFluxCapControl[13] = 0; ui17->uFluxCapControl[14] = 0; ui17->uFluxCapControl[15] = 0; ui17->uFluxCapControl[16] = 0; ui17->uFluxCapControl[17] = 0; ui17->uFluxCapControl[18] = 0; ui17->uFluxCapControl[19] = 0; ui17->uFluxCapControl[20] = 0; ui17->uFluxCapControl[21] = 0; ui17->uFluxCapControl[22] = 0; ui17->uFluxCapControl[23] = 0; ui17->uFluxCapControl[24] = 0; ui17->uFluxCapControl[25] = 0; ui17->uFluxCapControl[26] = 0; ui17->uFluxCapControl[27] = 0; ui17->uFluxCapControl[28] = 0; ui17->uFluxCapControl[29] = 0; ui17->uFluxCapControl[30] = 0; ui17->uFluxCapControl[31] = 0; ui17->uFluxCapControl[32] = 0; ui17->uFluxCapControl[33] = 0; ui17->uFluxCapControl[34] = 0; ui17->uFluxCapControl[35] = 0; ui17->uFluxCapControl[36] = 0; ui17->uFluxCapControl[37] = 0; ui17->uFluxCapControl[38] = 0; ui17->uFluxCapControl[39] = 0; ui17->uFluxCapControl[40] = 0; ui17->uFluxCapControl[41] = 0; ui17->uFluxCapControl[42] = 0; ui17->uFluxCapControl[43] = 0; ui17->uFluxCapControl[44] = 0; ui17->uFluxCapControl[45] = 0; ui17->uFluxCapControl[46] = 0; ui17->uFluxCapControl[47] = 0; ui17->uFluxCapControl[48] = 0; ui17->uFluxCapControl[49] = 0; ui17->uFluxCapControl[50] = 0; ui17->uFluxCapControl[51] = 0; ui17->uFluxCapControl[52] = 0; ui17->uFluxCapControl[53] = 0; ui17->uFluxCapControl[54] = 0; ui17->uFluxCapControl[55] = 0; ui17->uFluxCapControl[56] = 0; ui17->uFluxCapControl[57] = 0; ui17->uFluxCapControl[58] = 0; ui17->uFluxCapControl[59] = 0; ui17->uFluxCapControl[60] = 0; ui17->uFluxCapControl[61] = 0; ui17->uFluxCapControl[62] = 0; ui17->uFluxCapControl[63] = 0; 
	ui17->fFluxCapData[0] = 0.000000; ui17->fFluxCapData[1] = 0.000000; ui17->fFluxCapData[2] = 0.000000; ui17->fFluxCapData[3] = 0.000000; ui17->fFluxCapData[4] = 0.000000; ui17->fFluxCapData[5] = 0.000000; ui17->fFluxCapData[6] = 0.000000; ui17->fFluxCapData[7] = 0.000000; ui17->fFluxCapData[8] = 0.000000; ui17->fFluxCapData[9] = 0.000000; ui17->fFluxCapData[10] = 0.000000; ui17->fFluxCapData[11] = 0.000000; ui17->fFluxCapData[12] = 0.000000; ui17->fFluxCapData[13] = 0.000000; ui17->fFluxCapData[14] = 0.000000; ui17->fFluxCapData[15] = 0.000000; ui17->fFluxCapData[16] = 0.000000; ui17->fFluxCapData[17] = 0.000000; ui17->fFluxCapData[18] = 0.000000; ui17->fFluxCapData[19] = 0.000000; ui17->fFluxCapData[20] = 0.000000; ui17->fFluxCapData[21] = 0.000000; ui17->fFluxCapData[22] = 0.000000; ui17->fFluxCapData[23] = 0.000000; ui17->fFluxCapData[24] = 0.000000; ui17->fFluxCapData[25] = 0.000000; ui17->fFluxCapData[26] = 0.000000; ui17->fFluxCapData[27] = 0.000000; ui17->fFluxCapData[28] = 0.000000; ui17->fFluxCapData[29] = 0.000000; ui17->fFluxCapData[30] = 0.000000; ui17->fFluxCapData[31] = 0.000000; ui17->fFluxCapData[32] = 0.000000; ui17->fFluxCapData[33] = 0.000000; ui17->fFluxCapData[34] = 0.000000; ui17->fFluxCapData[35] = 0.000000; ui17->fFluxCapData[36] = 0.000000; ui17->fFluxCapData[37] = 0.000000; ui17->fFluxCapData[38] = 0.000000; ui17->fFluxCapData[39] = 0.000000; ui17->fFluxCapData[40] = 0.000000; ui17->fFluxCapData[41] = 0.000000; ui17->fFluxCapData[42] = 0.000000; ui17->fFluxCapData[43] = 0.000000; ui17->fFluxCapData[44] = 0.000000; ui17->fFluxCapData[45] = 0.000000; ui17->fFluxCapData[46] = 0.000000; ui17->fFluxCapData[47] = 0.000000; ui17->fFluxCapData[48] = 0.000000; ui17->fFluxCapData[49] = 0.000000; ui17->fFluxCapData[50] = 0.000000; ui17->fFluxCapData[51] = 0.000000; ui17->fFluxCapData[52] = 0.000000; ui17->fFluxCapData[53] = 0.000000; ui17->fFluxCapData[54] = 0.000000; ui17->fFluxCapData[55] = 0.000000; ui17->fFluxCapData[56] = 0.000000; ui17->fFluxCapData[57] = 0.000000; ui17->fFluxCapData[58] = 0.000000; ui17->fFluxCapData[59] = 0.000000; ui17->fFluxCapData[60] = 0.000000; ui17->fFluxCapData[61] = 0.000000; ui17->fFluxCapData[62] = 0.000000; ui17->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui17);
	delete ui17;


	m_f_Fc_Hz = 1000.000000;
	CUICtrl* ui18 = new CUICtrl;
	ui18->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui18->uControlId = 29;
	ui18->bLogSlider = false;
	ui18->bExpSlider = false;
	ui18->fUserDisplayDataLoLimit = 100.000000;
	ui18->fUserDisplayDataHiLimit = 18000.000000;
	ui18->uUserDataType = floatData;
	ui18->fInitUserIntValue = 0;
	ui18->fInitUserFloatValue = 1000.000000;
	ui18->fInitUserDoubleValue = 0;
	ui18->fInitUserUINTValue = 0;
	ui18->m_pUserCookedIntData = NULL;
	ui18->m_pUserCookedFloatData = &m_f_Fc_Hz;
	ui18->m_pUserCookedDoubleData = NULL;
	ui18->m_pUserCookedUINTData = NULL;
	ui18->cControlUnits = "Hz";
	ui18->cVariableName = "m_f_Fc_Hz";
	ui18->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui18->dPresetData[0] = 1000.000000;ui18->dPresetData[1] = 0.000000;ui18->dPresetData[2] = 0.000000;ui18->dPresetData[3] = 0.000000;ui18->dPresetData[4] = 0.000000;ui18->dPresetData[5] = 0.000000;ui18->dPresetData[6] = 0.000000;ui18->dPresetData[7] = 0.000000;ui18->dPresetData[8] = 0.000000;ui18->dPresetData[9] = 0.000000;ui18->dPresetData[10] = 0.000000;ui18->dPresetData[11] = 0.000000;ui18->dPresetData[12] = 0.000000;ui18->dPresetData[13] = 0.000000;ui18->dPresetData[14] = 0.000000;ui18->dPresetData[15] = 0.000000;
	ui18->cControlName = "Bass Cutoff";
	ui18->bOwnerControl = false;
	ui18->bMIDIControl = false;
	ui18->uMIDIControlCommand = 176;
	ui18->uMIDIControlName = 3;
	ui18->uMIDIControlChannel = 0;
	ui18->nGUIRow = nIndexer++;
	ui18->nGUIColumn = -1;
	ui18->bEnableParamSmoothing = false;
	ui18->fSmoothingTimeInMs = 100.00;
	ui18->uControlTheme[0] = 0; ui18->uControlTheme[1] = 0; ui18->uControlTheme[2] = 0; ui18->uControlTheme[3] = 0; ui18->uControlTheme[4] = 0; ui18->uControlTheme[5] = 0; ui18->uControlTheme[6] = 0; ui18->uControlTheme[7] = 0; ui18->uControlTheme[8] = 10; ui18->uControlTheme[9] = 0; ui18->uControlTheme[10] = 0; ui18->uControlTheme[11] = 0; ui18->uControlTheme[12] = 0; ui18->uControlTheme[13] = 0; ui18->uControlTheme[14] = 0; ui18->uControlTheme[15] = 0; ui18->uControlTheme[16] = 2; ui18->uControlTheme[17] = 0; ui18->uControlTheme[18] = 0; ui18->uControlTheme[19] = 0; ui18->uControlTheme[20] = 0; ui18->uControlTheme[21] = 0; ui18->uControlTheme[22] = 0; ui18->uControlTheme[23] = 0; ui18->uControlTheme[24] = 0; ui18->uControlTheme[25] = 0; ui18->uControlTheme[26] = 0; ui18->uControlTheme[27] = 0; ui18->uControlTheme[28] = 0; ui18->uControlTheme[29] = 0; ui18->uControlTheme[30] = 0; ui18->uControlTheme[31] = 0; 
	ui18->uFluxCapControl[0] = 0; ui18->uFluxCapControl[1] = 0; ui18->uFluxCapControl[2] = 0; ui18->uFluxCapControl[3] = 0; ui18->uFluxCapControl[4] = 0; ui18->uFluxCapControl[5] = 0; ui18->uFluxCapControl[6] = 0; ui18->uFluxCapControl[7] = 0; ui18->uFluxCapControl[8] = 0; ui18->uFluxCapControl[9] = 0; ui18->uFluxCapControl[10] = 0; ui18->uFluxCapControl[11] = 0; ui18->uFluxCapControl[12] = 0; ui18->uFluxCapControl[13] = 0; ui18->uFluxCapControl[14] = 0; ui18->uFluxCapControl[15] = 0; ui18->uFluxCapControl[16] = 0; ui18->uFluxCapControl[17] = 0; ui18->uFluxCapControl[18] = 0; ui18->uFluxCapControl[19] = 0; ui18->uFluxCapControl[20] = 0; ui18->uFluxCapControl[21] = 0; ui18->uFluxCapControl[22] = 0; ui18->uFluxCapControl[23] = 0; ui18->uFluxCapControl[24] = 0; ui18->uFluxCapControl[25] = 0; ui18->uFluxCapControl[26] = 0; ui18->uFluxCapControl[27] = 0; ui18->uFluxCapControl[28] = 0; ui18->uFluxCapControl[29] = 0; ui18->uFluxCapControl[30] = 0; ui18->uFluxCapControl[31] = 0; ui18->uFluxCapControl[32] = 0; ui18->uFluxCapControl[33] = 0; ui18->uFluxCapControl[34] = 0; ui18->uFluxCapControl[35] = 0; ui18->uFluxCapControl[36] = 0; ui18->uFluxCapControl[37] = 0; ui18->uFluxCapControl[38] = 0; ui18->uFluxCapControl[39] = 0; ui18->uFluxCapControl[40] = 0; ui18->uFluxCapControl[41] = 0; ui18->uFluxCapControl[42] = 0; ui18->uFluxCapControl[43] = 0; ui18->uFluxCapControl[44] = 0; ui18->uFluxCapControl[45] = 0; ui18->uFluxCapControl[46] = 0; ui18->uFluxCapControl[47] = 0; ui18->uFluxCapControl[48] = 0; ui18->uFluxCapControl[49] = 0; ui18->uFluxCapControl[50] = 0; ui18->uFluxCapControl[51] = 0; ui18->uFluxCapControl[52] = 0; ui18->uFluxCapControl[53] = 0; ui18->uFluxCapControl[54] = 0; ui18->uFluxCapControl[55] = 0; ui18->uFluxCapControl[56] = 0; ui18->uFluxCapControl[57] = 0; ui18->uFluxCapControl[58] = 0; ui18->uFluxCapControl[59] = 0; ui18->uFluxCapControl[60] = 0; ui18->uFluxCapControl[61] = 0; ui18->uFluxCapControl[62] = 0; ui18->uFluxCapControl[63] = 0; 
	ui18->fFluxCapData[0] = 0.000000; ui18->fFluxCapData[1] = 0.000000; ui18->fFluxCapData[2] = 0.000000; ui18->fFluxCapData[3] = 0.000000; ui18->fFluxCapData[4] = 0.000000; ui18->fFluxCapData[5] = 0.000000; ui18->fFluxCapData[6] = 0.000000; ui18->fFluxCapData[7] = 0.000000; ui18->fFluxCapData[8] = 0.000000; ui18->fFluxCapData[9] = 0.000000; ui18->fFluxCapData[10] = 0.000000; ui18->fFluxCapData[11] = 0.000000; ui18->fFluxCapData[12] = 0.000000; ui18->fFluxCapData[13] = 0.000000; ui18->fFluxCapData[14] = 0.000000; ui18->fFluxCapData[15] = 0.000000; ui18->fFluxCapData[16] = 0.000000; ui18->fFluxCapData[17] = 0.000000; ui18->fFluxCapData[18] = 0.000000; ui18->fFluxCapData[19] = 0.000000; ui18->fFluxCapData[20] = 0.000000; ui18->fFluxCapData[21] = 0.000000; ui18->fFluxCapData[22] = 0.000000; ui18->fFluxCapData[23] = 0.000000; ui18->fFluxCapData[24] = 0.000000; ui18->fFluxCapData[25] = 0.000000; ui18->fFluxCapData[26] = 0.000000; ui18->fFluxCapData[27] = 0.000000; ui18->fFluxCapData[28] = 0.000000; ui18->fFluxCapData[29] = 0.000000; ui18->fFluxCapData[30] = 0.000000; ui18->fFluxCapData[31] = 0.000000; ui18->fFluxCapData[32] = 0.000000; ui18->fFluxCapData[33] = 0.000000; ui18->fFluxCapData[34] = 0.000000; ui18->fFluxCapData[35] = 0.000000; ui18->fFluxCapData[36] = 0.000000; ui18->fFluxCapData[37] = 0.000000; ui18->fFluxCapData[38] = 0.000000; ui18->fFluxCapData[39] = 0.000000; ui18->fFluxCapData[40] = 0.000000; ui18->fFluxCapData[41] = 0.000000; ui18->fFluxCapData[42] = 0.000000; ui18->fFluxCapData[43] = 0.000000; ui18->fFluxCapData[44] = 0.000000; ui18->fFluxCapData[45] = 0.000000; ui18->fFluxCapData[46] = 0.000000; ui18->fFluxCapData[47] = 0.000000; ui18->fFluxCapData[48] = 0.000000; ui18->fFluxCapData[49] = 0.000000; ui18->fFluxCapData[50] = 0.000000; ui18->fFluxCapData[51] = 0.000000; ui18->fFluxCapData[52] = 0.000000; ui18->fFluxCapData[53] = 0.000000; ui18->fFluxCapData[54] = 0.000000; ui18->fFluxCapData[55] = 0.000000; ui18->fFluxCapData[56] = 0.000000; ui18->fFluxCapData[57] = 0.000000; ui18->fFluxCapData[58] = 0.000000; ui18->fFluxCapData[59] = 0.000000; ui18->fFluxCapData[60] = 0.000000; ui18->fFluxCapData[61] = 0.000000; ui18->fFluxCapData[62] = 0.000000; ui18->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui18);
	delete ui18;


	m_fWet_pct_reverb = 20.000000;
	CUICtrl* ui19 = new CUICtrl;
	ui19->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui19->uControlId = 100;
	ui19->bLogSlider = false;
	ui19->bExpSlider = false;
	ui19->fUserDisplayDataLoLimit = 0.000000;
	ui19->fUserDisplayDataHiLimit = 100.000000;
	ui19->uUserDataType = floatData;
	ui19->fInitUserIntValue = 0;
	ui19->fInitUserFloatValue = 20.000000;
	ui19->fInitUserDoubleValue = 0;
	ui19->fInitUserUINTValue = 0;
	ui19->m_pUserCookedIntData = NULL;
	ui19->m_pUserCookedFloatData = &m_fWet_pct_reverb;
	ui19->m_pUserCookedDoubleData = NULL;
	ui19->m_pUserCookedUINTData = NULL;
	ui19->cControlUnits = "%";
	ui19->cVariableName = "m_fWet_pct_reverb";
	ui19->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui19->dPresetData[0] = 20.000000;ui19->dPresetData[1] = 0.000000;ui19->dPresetData[2] = 0.000000;ui19->dPresetData[3] = 0.000000;ui19->dPresetData[4] = 0.000000;ui19->dPresetData[5] = 0.000000;ui19->dPresetData[6] = 0.000000;ui19->dPresetData[7] = 0.000000;ui19->dPresetData[8] = 0.000000;ui19->dPresetData[9] = 0.000000;ui19->dPresetData[10] = 0.000000;ui19->dPresetData[11] = 0.000000;ui19->dPresetData[12] = 0.000000;ui19->dPresetData[13] = 0.000000;ui19->dPresetData[14] = 0.000000;ui19->dPresetData[15] = 0.000000;
	ui19->cControlName = "Wet/Dry";
	ui19->bOwnerControl = false;
	ui19->bMIDIControl = false;
	ui19->uMIDIControlCommand = 176;
	ui19->uMIDIControlName = 3;
	ui19->uMIDIControlChannel = 0;
	ui19->nGUIRow = nIndexer++;
	ui19->nGUIColumn = -1;
	ui19->bEnableParamSmoothing = false;
	ui19->fSmoothingTimeInMs = 100.00;
	ui19->uControlTheme[0] = 0; ui19->uControlTheme[1] = 0; ui19->uControlTheme[2] = 0; ui19->uControlTheme[3] = 0; ui19->uControlTheme[4] = 0; ui19->uControlTheme[5] = 0; ui19->uControlTheme[6] = 0; ui19->uControlTheme[7] = 0; ui19->uControlTheme[8] = 6; ui19->uControlTheme[9] = 0; ui19->uControlTheme[10] = 0; ui19->uControlTheme[11] = 0; ui19->uControlTheme[12] = 0; ui19->uControlTheme[13] = 0; ui19->uControlTheme[14] = 0; ui19->uControlTheme[15] = 0; ui19->uControlTheme[16] = 2; ui19->uControlTheme[17] = 0; ui19->uControlTheme[18] = 0; ui19->uControlTheme[19] = 0; ui19->uControlTheme[20] = 0; ui19->uControlTheme[21] = 0; ui19->uControlTheme[22] = 0; ui19->uControlTheme[23] = 0; ui19->uControlTheme[24] = 0; ui19->uControlTheme[25] = 0; ui19->uControlTheme[26] = 0; ui19->uControlTheme[27] = 0; ui19->uControlTheme[28] = 0; ui19->uControlTheme[29] = 0; ui19->uControlTheme[30] = 0; ui19->uControlTheme[31] = 0; 
	ui19->uFluxCapControl[0] = 0; ui19->uFluxCapControl[1] = 0; ui19->uFluxCapControl[2] = 0; ui19->uFluxCapControl[3] = 0; ui19->uFluxCapControl[4] = 0; ui19->uFluxCapControl[5] = 0; ui19->uFluxCapControl[6] = 0; ui19->uFluxCapControl[7] = 0; ui19->uFluxCapControl[8] = 0; ui19->uFluxCapControl[9] = 0; ui19->uFluxCapControl[10] = 0; ui19->uFluxCapControl[11] = 0; ui19->uFluxCapControl[12] = 0; ui19->uFluxCapControl[13] = 0; ui19->uFluxCapControl[14] = 0; ui19->uFluxCapControl[15] = 0; ui19->uFluxCapControl[16] = 0; ui19->uFluxCapControl[17] = 0; ui19->uFluxCapControl[18] = 0; ui19->uFluxCapControl[19] = 0; ui19->uFluxCapControl[20] = 0; ui19->uFluxCapControl[21] = 0; ui19->uFluxCapControl[22] = 0; ui19->uFluxCapControl[23] = 0; ui19->uFluxCapControl[24] = 0; ui19->uFluxCapControl[25] = 0; ui19->uFluxCapControl[26] = 0; ui19->uFluxCapControl[27] = 0; ui19->uFluxCapControl[28] = 0; ui19->uFluxCapControl[29] = 0; ui19->uFluxCapControl[30] = 0; ui19->uFluxCapControl[31] = 0; ui19->uFluxCapControl[32] = 0; ui19->uFluxCapControl[33] = 0; ui19->uFluxCapControl[34] = 0; ui19->uFluxCapControl[35] = 0; ui19->uFluxCapControl[36] = 0; ui19->uFluxCapControl[37] = 0; ui19->uFluxCapControl[38] = 0; ui19->uFluxCapControl[39] = 0; ui19->uFluxCapControl[40] = 0; ui19->uFluxCapControl[41] = 0; ui19->uFluxCapControl[42] = 0; ui19->uFluxCapControl[43] = 0; ui19->uFluxCapControl[44] = 0; ui19->uFluxCapControl[45] = 0; ui19->uFluxCapControl[46] = 0; ui19->uFluxCapControl[47] = 0; ui19->uFluxCapControl[48] = 0; ui19->uFluxCapControl[49] = 0; ui19->uFluxCapControl[50] = 0; ui19->uFluxCapControl[51] = 0; ui19->uFluxCapControl[52] = 0; ui19->uFluxCapControl[53] = 0; ui19->uFluxCapControl[54] = 0; ui19->uFluxCapControl[55] = 0; ui19->uFluxCapControl[56] = 0; ui19->uFluxCapControl[57] = 0; ui19->uFluxCapControl[58] = 0; ui19->uFluxCapControl[59] = 0; ui19->uFluxCapControl[60] = 0; ui19->uFluxCapControl[61] = 0; ui19->uFluxCapControl[62] = 0; ui19->uFluxCapControl[63] = 0; 
	ui19->fFluxCapData[0] = 0.000000; ui19->fFluxCapData[1] = 0.000000; ui19->fFluxCapData[2] = 0.000000; ui19->fFluxCapData[3] = 0.000000; ui19->fFluxCapData[4] = 0.000000; ui19->fFluxCapData[5] = 0.000000; ui19->fFluxCapData[6] = 0.000000; ui19->fFluxCapData[7] = 0.000000; ui19->fFluxCapData[8] = 0.000000; ui19->fFluxCapData[9] = 0.000000; ui19->fFluxCapData[10] = 0.000000; ui19->fFluxCapData[11] = 0.000000; ui19->fFluxCapData[12] = 0.000000; ui19->fFluxCapData[13] = 0.000000; ui19->fFluxCapData[14] = 0.000000; ui19->fFluxCapData[15] = 0.000000; ui19->fFluxCapData[16] = 0.000000; ui19->fFluxCapData[17] = 0.000000; ui19->fFluxCapData[18] = 0.000000; ui19->fFluxCapData[19] = 0.000000; ui19->fFluxCapData[20] = 0.000000; ui19->fFluxCapData[21] = 0.000000; ui19->fFluxCapData[22] = 0.000000; ui19->fFluxCapData[23] = 0.000000; ui19->fFluxCapData[24] = 0.000000; ui19->fFluxCapData[25] = 0.000000; ui19->fFluxCapData[26] = 0.000000; ui19->fFluxCapData[27] = 0.000000; ui19->fFluxCapData[28] = 0.000000; ui19->fFluxCapData[29] = 0.000000; ui19->fFluxCapData[30] = 0.000000; ui19->fFluxCapData[31] = 0.000000; ui19->fFluxCapData[32] = 0.000000; ui19->fFluxCapData[33] = 0.000000; ui19->fFluxCapData[34] = 0.000000; ui19->fFluxCapData[35] = 0.000000; ui19->fFluxCapData[36] = 0.000000; ui19->fFluxCapData[37] = 0.000000; ui19->fFluxCapData[38] = 0.000000; ui19->fFluxCapData[39] = 0.000000; ui19->fFluxCapData[40] = 0.000000; ui19->fFluxCapData[41] = 0.000000; ui19->fFluxCapData[42] = 0.000000; ui19->fFluxCapData[43] = 0.000000; ui19->fFluxCapData[44] = 0.000000; ui19->fFluxCapData[45] = 0.000000; ui19->fFluxCapData[46] = 0.000000; ui19->fFluxCapData[47] = 0.000000; ui19->fFluxCapData[48] = 0.000000; ui19->fFluxCapData[49] = 0.000000; ui19->fFluxCapData[50] = 0.000000; ui19->fFluxCapData[51] = 0.000000; ui19->fFluxCapData[52] = 0.000000; ui19->fFluxCapData[53] = 0.000000; ui19->fFluxCapData[54] = 0.000000; ui19->fFluxCapData[55] = 0.000000; ui19->fFluxCapData[56] = 0.000000; ui19->fFluxCapData[57] = 0.000000; ui19->fFluxCapData[58] = 0.000000; ui19->fFluxCapData[59] = 0.000000; ui19->fFluxCapData[60] = 0.000000; ui19->fFluxCapData[61] = 0.000000; ui19->fFluxCapData[62] = 0.000000; ui19->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui19);
	delete ui19;


	m_f_APF1_Delay_mSec = 53.500000;
	CUICtrl* ui20 = new CUICtrl;
	ui20->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui20->uControlId = 101;
	ui20->bLogSlider = false;
	ui20->bExpSlider = false;
	ui20->fUserDisplayDataLoLimit = 0.000000;
	ui20->fUserDisplayDataHiLimit = 100.000000;
	ui20->uUserDataType = floatData;
	ui20->fInitUserIntValue = 0;
	ui20->fInitUserFloatValue = 53.500000;
	ui20->fInitUserDoubleValue = 0;
	ui20->fInitUserUINTValue = 0;
	ui20->m_pUserCookedIntData = NULL;
	ui20->m_pUserCookedFloatData = &m_f_APF1_Delay_mSec;
	ui20->m_pUserCookedDoubleData = NULL;
	ui20->m_pUserCookedUINTData = NULL;
	ui20->cControlUnits = "mSec";
	ui20->cVariableName = "m_f_APF1_Delay_mSec";
	ui20->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui20->dPresetData[0] = 53.500004;ui20->dPresetData[1] = 0.000000;ui20->dPresetData[2] = 0.000000;ui20->dPresetData[3] = 0.000000;ui20->dPresetData[4] = 0.000000;ui20->dPresetData[5] = 0.000000;ui20->dPresetData[6] = 0.000000;ui20->dPresetData[7] = 0.000000;ui20->dPresetData[8] = 0.000000;ui20->dPresetData[9] = 0.000000;ui20->dPresetData[10] = 0.000000;ui20->dPresetData[11] = 0.000000;ui20->dPresetData[12] = 0.000000;ui20->dPresetData[13] = 0.000000;ui20->dPresetData[14] = 0.000000;ui20->dPresetData[15] = 0.000000;
	ui20->cControlName = "APF1";
	ui20->bOwnerControl = false;
	ui20->bMIDIControl = false;
	ui20->uMIDIControlCommand = 176;
	ui20->uMIDIControlName = 3;
	ui20->uMIDIControlChannel = 0;
	ui20->nGUIRow = nIndexer++;
	ui20->nGUIColumn = -1;
	ui20->bEnableParamSmoothing = false;
	ui20->fSmoothingTimeInMs = 100.00;
	ui20->uControlTheme[0] = 0; ui20->uControlTheme[1] = 0; ui20->uControlTheme[2] = 0; ui20->uControlTheme[3] = 0; ui20->uControlTheme[4] = 0; ui20->uControlTheme[5] = 0; ui20->uControlTheme[6] = 0; ui20->uControlTheme[7] = 0; ui20->uControlTheme[8] = 8; ui20->uControlTheme[9] = 0; ui20->uControlTheme[10] = 0; ui20->uControlTheme[11] = 0; ui20->uControlTheme[12] = 0; ui20->uControlTheme[13] = 0; ui20->uControlTheme[14] = 0; ui20->uControlTheme[15] = 0; ui20->uControlTheme[16] = 2; ui20->uControlTheme[17] = 0; ui20->uControlTheme[18] = 0; ui20->uControlTheme[19] = 0; ui20->uControlTheme[20] = 0; ui20->uControlTheme[21] = 0; ui20->uControlTheme[22] = 0; ui20->uControlTheme[23] = 0; ui20->uControlTheme[24] = 0; ui20->uControlTheme[25] = 0; ui20->uControlTheme[26] = 0; ui20->uControlTheme[27] = 0; ui20->uControlTheme[28] = 0; ui20->uControlTheme[29] = 0; ui20->uControlTheme[30] = 0; ui20->uControlTheme[31] = 0; 
	ui20->uFluxCapControl[0] = 0; ui20->uFluxCapControl[1] = 0; ui20->uFluxCapControl[2] = 0; ui20->uFluxCapControl[3] = 0; ui20->uFluxCapControl[4] = 0; ui20->uFluxCapControl[5] = 0; ui20->uFluxCapControl[6] = 0; ui20->uFluxCapControl[7] = 0; ui20->uFluxCapControl[8] = 0; ui20->uFluxCapControl[9] = 0; ui20->uFluxCapControl[10] = 0; ui20->uFluxCapControl[11] = 0; ui20->uFluxCapControl[12] = 0; ui20->uFluxCapControl[13] = 0; ui20->uFluxCapControl[14] = 0; ui20->uFluxCapControl[15] = 0; ui20->uFluxCapControl[16] = 0; ui20->uFluxCapControl[17] = 0; ui20->uFluxCapControl[18] = 0; ui20->uFluxCapControl[19] = 0; ui20->uFluxCapControl[20] = 0; ui20->uFluxCapControl[21] = 0; ui20->uFluxCapControl[22] = 0; ui20->uFluxCapControl[23] = 0; ui20->uFluxCapControl[24] = 0; ui20->uFluxCapControl[25] = 0; ui20->uFluxCapControl[26] = 0; ui20->uFluxCapControl[27] = 0; ui20->uFluxCapControl[28] = 0; ui20->uFluxCapControl[29] = 0; ui20->uFluxCapControl[30] = 0; ui20->uFluxCapControl[31] = 0; ui20->uFluxCapControl[32] = 0; ui20->uFluxCapControl[33] = 0; ui20->uFluxCapControl[34] = 0; ui20->uFluxCapControl[35] = 0; ui20->uFluxCapControl[36] = 0; ui20->uFluxCapControl[37] = 0; ui20->uFluxCapControl[38] = 0; ui20->uFluxCapControl[39] = 0; ui20->uFluxCapControl[40] = 0; ui20->uFluxCapControl[41] = 0; ui20->uFluxCapControl[42] = 0; ui20->uFluxCapControl[43] = 0; ui20->uFluxCapControl[44] = 0; ui20->uFluxCapControl[45] = 0; ui20->uFluxCapControl[46] = 0; ui20->uFluxCapControl[47] = 0; ui20->uFluxCapControl[48] = 0; ui20->uFluxCapControl[49] = 0; ui20->uFluxCapControl[50] = 0; ui20->uFluxCapControl[51] = 0; ui20->uFluxCapControl[52] = 0; ui20->uFluxCapControl[53] = 0; ui20->uFluxCapControl[54] = 0; ui20->uFluxCapControl[55] = 0; ui20->uFluxCapControl[56] = 0; ui20->uFluxCapControl[57] = 0; ui20->uFluxCapControl[58] = 0; ui20->uFluxCapControl[59] = 0; ui20->uFluxCapControl[60] = 0; ui20->uFluxCapControl[61] = 0; ui20->uFluxCapControl[62] = 0; ui20->uFluxCapControl[63] = 0; 
	ui20->fFluxCapData[0] = 0.000000; ui20->fFluxCapData[1] = 0.000000; ui20->fFluxCapData[2] = 0.000000; ui20->fFluxCapData[3] = 0.000000; ui20->fFluxCapData[4] = 0.000000; ui20->fFluxCapData[5] = 0.000000; ui20->fFluxCapData[6] = 0.000000; ui20->fFluxCapData[7] = 0.000000; ui20->fFluxCapData[8] = 0.000000; ui20->fFluxCapData[9] = 0.000000; ui20->fFluxCapData[10] = 0.000000; ui20->fFluxCapData[11] = 0.000000; ui20->fFluxCapData[12] = 0.000000; ui20->fFluxCapData[13] = 0.000000; ui20->fFluxCapData[14] = 0.000000; ui20->fFluxCapData[15] = 0.000000; ui20->fFluxCapData[16] = 0.000000; ui20->fFluxCapData[17] = 0.000000; ui20->fFluxCapData[18] = 0.000000; ui20->fFluxCapData[19] = 0.000000; ui20->fFluxCapData[20] = 0.000000; ui20->fFluxCapData[21] = 0.000000; ui20->fFluxCapData[22] = 0.000000; ui20->fFluxCapData[23] = 0.000000; ui20->fFluxCapData[24] = 0.000000; ui20->fFluxCapData[25] = 0.000000; ui20->fFluxCapData[26] = 0.000000; ui20->fFluxCapData[27] = 0.000000; ui20->fFluxCapData[28] = 0.000000; ui20->fFluxCapData[29] = 0.000000; ui20->fFluxCapData[30] = 0.000000; ui20->fFluxCapData[31] = 0.000000; ui20->fFluxCapData[32] = 0.000000; ui20->fFluxCapData[33] = 0.000000; ui20->fFluxCapData[34] = 0.000000; ui20->fFluxCapData[35] = 0.000000; ui20->fFluxCapData[36] = 0.000000; ui20->fFluxCapData[37] = 0.000000; ui20->fFluxCapData[38] = 0.000000; ui20->fFluxCapData[39] = 0.000000; ui20->fFluxCapData[40] = 0.000000; ui20->fFluxCapData[41] = 0.000000; ui20->fFluxCapData[42] = 0.000000; ui20->fFluxCapData[43] = 0.000000; ui20->fFluxCapData[44] = 0.000000; ui20->fFluxCapData[45] = 0.000000; ui20->fFluxCapData[46] = 0.000000; ui20->fFluxCapData[47] = 0.000000; ui20->fFluxCapData[48] = 0.000000; ui20->fFluxCapData[49] = 0.000000; ui20->fFluxCapData[50] = 0.000000; ui20->fFluxCapData[51] = 0.000000; ui20->fFluxCapData[52] = 0.000000; ui20->fFluxCapData[53] = 0.000000; ui20->fFluxCapData[54] = 0.000000; ui20->fFluxCapData[55] = 0.000000; ui20->fFluxCapData[56] = 0.000000; ui20->fFluxCapData[57] = 0.000000; ui20->fFluxCapData[58] = 0.000000; ui20->fFluxCapData[59] = 0.000000; ui20->fFluxCapData[60] = 0.000000; ui20->fFluxCapData[61] = 0.000000; ui20->fFluxCapData[62] = 0.000000; ui20->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui20);
	delete ui20;


	m_f_APF2_Delay_mSec = 33.500000;
	CUICtrl* ui21 = new CUICtrl;
	ui21->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui21->uControlId = 102;
	ui21->bLogSlider = false;
	ui21->bExpSlider = false;
	ui21->fUserDisplayDataLoLimit = 0.000000;
	ui21->fUserDisplayDataHiLimit = 100.000000;
	ui21->uUserDataType = floatData;
	ui21->fInitUserIntValue = 0;
	ui21->fInitUserFloatValue = 33.500000;
	ui21->fInitUserDoubleValue = 0;
	ui21->fInitUserUINTValue = 0;
	ui21->m_pUserCookedIntData = NULL;
	ui21->m_pUserCookedFloatData = &m_f_APF2_Delay_mSec;
	ui21->m_pUserCookedDoubleData = NULL;
	ui21->m_pUserCookedUINTData = NULL;
	ui21->cControlUnits = "mSec";
	ui21->cVariableName = "m_f_APF2_Delay_mSec";
	ui21->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui21->dPresetData[0] = 33.500000;ui21->dPresetData[1] = 0.000000;ui21->dPresetData[2] = 0.000000;ui21->dPresetData[3] = 0.000000;ui21->dPresetData[4] = 0.000000;ui21->dPresetData[5] = 0.000000;ui21->dPresetData[6] = 0.000000;ui21->dPresetData[7] = 0.000000;ui21->dPresetData[8] = 0.000000;ui21->dPresetData[9] = 0.000000;ui21->dPresetData[10] = 0.000000;ui21->dPresetData[11] = 0.000000;ui21->dPresetData[12] = 0.000000;ui21->dPresetData[13] = 0.000000;ui21->dPresetData[14] = 0.000000;ui21->dPresetData[15] = 0.000000;
	ui21->cControlName = "APF2";
	ui21->bOwnerControl = false;
	ui21->bMIDIControl = false;
	ui21->uMIDIControlCommand = 176;
	ui21->uMIDIControlName = 3;
	ui21->uMIDIControlChannel = 0;
	ui21->nGUIRow = nIndexer++;
	ui21->nGUIColumn = -1;
	ui21->bEnableParamSmoothing = false;
	ui21->fSmoothingTimeInMs = 100.00;
	ui21->uControlTheme[0] = 0; ui21->uControlTheme[1] = 0; ui21->uControlTheme[2] = 0; ui21->uControlTheme[3] = 0; ui21->uControlTheme[4] = 0; ui21->uControlTheme[5] = 0; ui21->uControlTheme[6] = 0; ui21->uControlTheme[7] = 0; ui21->uControlTheme[8] = 8; ui21->uControlTheme[9] = 0; ui21->uControlTheme[10] = 0; ui21->uControlTheme[11] = 0; ui21->uControlTheme[12] = 0; ui21->uControlTheme[13] = 0; ui21->uControlTheme[14] = 0; ui21->uControlTheme[15] = 0; ui21->uControlTheme[16] = 2; ui21->uControlTheme[17] = 0; ui21->uControlTheme[18] = 0; ui21->uControlTheme[19] = 0; ui21->uControlTheme[20] = 0; ui21->uControlTheme[21] = 0; ui21->uControlTheme[22] = 0; ui21->uControlTheme[23] = 0; ui21->uControlTheme[24] = 0; ui21->uControlTheme[25] = 0; ui21->uControlTheme[26] = 0; ui21->uControlTheme[27] = 0; ui21->uControlTheme[28] = 0; ui21->uControlTheme[29] = 0; ui21->uControlTheme[30] = 0; ui21->uControlTheme[31] = 0; 
	ui21->uFluxCapControl[0] = 0; ui21->uFluxCapControl[1] = 0; ui21->uFluxCapControl[2] = 0; ui21->uFluxCapControl[3] = 0; ui21->uFluxCapControl[4] = 0; ui21->uFluxCapControl[5] = 0; ui21->uFluxCapControl[6] = 0; ui21->uFluxCapControl[7] = 0; ui21->uFluxCapControl[8] = 0; ui21->uFluxCapControl[9] = 0; ui21->uFluxCapControl[10] = 0; ui21->uFluxCapControl[11] = 0; ui21->uFluxCapControl[12] = 0; ui21->uFluxCapControl[13] = 0; ui21->uFluxCapControl[14] = 0; ui21->uFluxCapControl[15] = 0; ui21->uFluxCapControl[16] = 0; ui21->uFluxCapControl[17] = 0; ui21->uFluxCapControl[18] = 0; ui21->uFluxCapControl[19] = 0; ui21->uFluxCapControl[20] = 0; ui21->uFluxCapControl[21] = 0; ui21->uFluxCapControl[22] = 0; ui21->uFluxCapControl[23] = 0; ui21->uFluxCapControl[24] = 0; ui21->uFluxCapControl[25] = 0; ui21->uFluxCapControl[26] = 0; ui21->uFluxCapControl[27] = 0; ui21->uFluxCapControl[28] = 0; ui21->uFluxCapControl[29] = 0; ui21->uFluxCapControl[30] = 0; ui21->uFluxCapControl[31] = 0; ui21->uFluxCapControl[32] = 0; ui21->uFluxCapControl[33] = 0; ui21->uFluxCapControl[34] = 0; ui21->uFluxCapControl[35] = 0; ui21->uFluxCapControl[36] = 0; ui21->uFluxCapControl[37] = 0; ui21->uFluxCapControl[38] = 0; ui21->uFluxCapControl[39] = 0; ui21->uFluxCapControl[40] = 0; ui21->uFluxCapControl[41] = 0; ui21->uFluxCapControl[42] = 0; ui21->uFluxCapControl[43] = 0; ui21->uFluxCapControl[44] = 0; ui21->uFluxCapControl[45] = 0; ui21->uFluxCapControl[46] = 0; ui21->uFluxCapControl[47] = 0; ui21->uFluxCapControl[48] = 0; ui21->uFluxCapControl[49] = 0; ui21->uFluxCapControl[50] = 0; ui21->uFluxCapControl[51] = 0; ui21->uFluxCapControl[52] = 0; ui21->uFluxCapControl[53] = 0; ui21->uFluxCapControl[54] = 0; ui21->uFluxCapControl[55] = 0; ui21->uFluxCapControl[56] = 0; ui21->uFluxCapControl[57] = 0; ui21->uFluxCapControl[58] = 0; ui21->uFluxCapControl[59] = 0; ui21->uFluxCapControl[60] = 0; ui21->uFluxCapControl[61] = 0; ui21->uFluxCapControl[62] = 0; ui21->uFluxCapControl[63] = 0; 
	ui21->fFluxCapData[0] = 0.000000; ui21->fFluxCapData[1] = 0.000000; ui21->fFluxCapData[2] = 0.000000; ui21->fFluxCapData[3] = 0.000000; ui21->fFluxCapData[4] = 0.000000; ui21->fFluxCapData[5] = 0.000000; ui21->fFluxCapData[6] = 0.000000; ui21->fFluxCapData[7] = 0.000000; ui21->fFluxCapData[8] = 0.000000; ui21->fFluxCapData[9] = 0.000000; ui21->fFluxCapData[10] = 0.000000; ui21->fFluxCapData[11] = 0.000000; ui21->fFluxCapData[12] = 0.000000; ui21->fFluxCapData[13] = 0.000000; ui21->fFluxCapData[14] = 0.000000; ui21->fFluxCapData[15] = 0.000000; ui21->fFluxCapData[16] = 0.000000; ui21->fFluxCapData[17] = 0.000000; ui21->fFluxCapData[18] = 0.000000; ui21->fFluxCapData[19] = 0.000000; ui21->fFluxCapData[20] = 0.000000; ui21->fFluxCapData[21] = 0.000000; ui21->fFluxCapData[22] = 0.000000; ui21->fFluxCapData[23] = 0.000000; ui21->fFluxCapData[24] = 0.000000; ui21->fFluxCapData[25] = 0.000000; ui21->fFluxCapData[26] = 0.000000; ui21->fFluxCapData[27] = 0.000000; ui21->fFluxCapData[28] = 0.000000; ui21->fFluxCapData[29] = 0.000000; ui21->fFluxCapData[30] = 0.000000; ui21->fFluxCapData[31] = 0.000000; ui21->fFluxCapData[32] = 0.000000; ui21->fFluxCapData[33] = 0.000000; ui21->fFluxCapData[34] = 0.000000; ui21->fFluxCapData[35] = 0.000000; ui21->fFluxCapData[36] = 0.000000; ui21->fFluxCapData[37] = 0.000000; ui21->fFluxCapData[38] = 0.000000; ui21->fFluxCapData[39] = 0.000000; ui21->fFluxCapData[40] = 0.000000; ui21->fFluxCapData[41] = 0.000000; ui21->fFluxCapData[42] = 0.000000; ui21->fFluxCapData[43] = 0.000000; ui21->fFluxCapData[44] = 0.000000; ui21->fFluxCapData[45] = 0.000000; ui21->fFluxCapData[46] = 0.000000; ui21->fFluxCapData[47] = 0.000000; ui21->fFluxCapData[48] = 0.000000; ui21->fFluxCapData[49] = 0.000000; ui21->fFluxCapData[50] = 0.000000; ui21->fFluxCapData[51] = 0.000000; ui21->fFluxCapData[52] = 0.000000; ui21->fFluxCapData[53] = 0.000000; ui21->fFluxCapData[54] = 0.000000; ui21->fFluxCapData[55] = 0.000000; ui21->fFluxCapData[56] = 0.000000; ui21->fFluxCapData[57] = 0.000000; ui21->fFluxCapData[58] = 0.000000; ui21->fFluxCapData[59] = 0.000000; ui21->fFluxCapData[60] = 0.000000; ui21->fFluxCapData[61] = 0.000000; ui21->fFluxCapData[62] = 0.000000; ui21->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui21);
	delete ui21;


	m_f_APF3_Delay_mSec = 32.000000;
	CUICtrl* ui22 = new CUICtrl;
	ui22->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui22->uControlId = 103;
	ui22->bLogSlider = false;
	ui22->bExpSlider = false;
	ui22->fUserDisplayDataLoLimit = 0.000000;
	ui22->fUserDisplayDataHiLimit = 100.000000;
	ui22->uUserDataType = floatData;
	ui22->fInitUserIntValue = 0;
	ui22->fInitUserFloatValue = 32.000000;
	ui22->fInitUserDoubleValue = 0;
	ui22->fInitUserUINTValue = 0;
	ui22->m_pUserCookedIntData = NULL;
	ui22->m_pUserCookedFloatData = &m_f_APF3_Delay_mSec;
	ui22->m_pUserCookedDoubleData = NULL;
	ui22->m_pUserCookedUINTData = NULL;
	ui22->cControlUnits = "mSec";
	ui22->cVariableName = "m_f_APF3_Delay_mSec";
	ui22->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui22->dPresetData[0] = 32.000000;ui22->dPresetData[1] = 0.000000;ui22->dPresetData[2] = 0.000000;ui22->dPresetData[3] = 0.000000;ui22->dPresetData[4] = 0.000000;ui22->dPresetData[5] = 0.000000;ui22->dPresetData[6] = 0.000000;ui22->dPresetData[7] = 0.000000;ui22->dPresetData[8] = 0.000000;ui22->dPresetData[9] = 0.000000;ui22->dPresetData[10] = 0.000000;ui22->dPresetData[11] = 0.000000;ui22->dPresetData[12] = 0.000000;ui22->dPresetData[13] = 0.000000;ui22->dPresetData[14] = 0.000000;ui22->dPresetData[15] = 0.000000;
	ui22->cControlName = "APF3";
	ui22->bOwnerControl = false;
	ui22->bMIDIControl = false;
	ui22->uMIDIControlCommand = 176;
	ui22->uMIDIControlName = 3;
	ui22->uMIDIControlChannel = 0;
	ui22->nGUIRow = nIndexer++;
	ui22->nGUIColumn = -1;
	ui22->bEnableParamSmoothing = false;
	ui22->fSmoothingTimeInMs = 100.00;
	ui22->uControlTheme[0] = 0; ui22->uControlTheme[1] = 0; ui22->uControlTheme[2] = 0; ui22->uControlTheme[3] = 0; ui22->uControlTheme[4] = 0; ui22->uControlTheme[5] = 0; ui22->uControlTheme[6] = 0; ui22->uControlTheme[7] = 0; ui22->uControlTheme[8] = 8; ui22->uControlTheme[9] = 0; ui22->uControlTheme[10] = 0; ui22->uControlTheme[11] = 0; ui22->uControlTheme[12] = 0; ui22->uControlTheme[13] = 0; ui22->uControlTheme[14] = 0; ui22->uControlTheme[15] = 0; ui22->uControlTheme[16] = 2; ui22->uControlTheme[17] = 0; ui22->uControlTheme[18] = 0; ui22->uControlTheme[19] = 0; ui22->uControlTheme[20] = 0; ui22->uControlTheme[21] = 0; ui22->uControlTheme[22] = 0; ui22->uControlTheme[23] = 0; ui22->uControlTheme[24] = 0; ui22->uControlTheme[25] = 0; ui22->uControlTheme[26] = 0; ui22->uControlTheme[27] = 0; ui22->uControlTheme[28] = 0; ui22->uControlTheme[29] = 0; ui22->uControlTheme[30] = 0; ui22->uControlTheme[31] = 0; 
	ui22->uFluxCapControl[0] = 0; ui22->uFluxCapControl[1] = 0; ui22->uFluxCapControl[2] = 0; ui22->uFluxCapControl[3] = 0; ui22->uFluxCapControl[4] = 0; ui22->uFluxCapControl[5] = 0; ui22->uFluxCapControl[6] = 0; ui22->uFluxCapControl[7] = 0; ui22->uFluxCapControl[8] = 0; ui22->uFluxCapControl[9] = 0; ui22->uFluxCapControl[10] = 0; ui22->uFluxCapControl[11] = 0; ui22->uFluxCapControl[12] = 0; ui22->uFluxCapControl[13] = 0; ui22->uFluxCapControl[14] = 0; ui22->uFluxCapControl[15] = 0; ui22->uFluxCapControl[16] = 0; ui22->uFluxCapControl[17] = 0; ui22->uFluxCapControl[18] = 0; ui22->uFluxCapControl[19] = 0; ui22->uFluxCapControl[20] = 0; ui22->uFluxCapControl[21] = 0; ui22->uFluxCapControl[22] = 0; ui22->uFluxCapControl[23] = 0; ui22->uFluxCapControl[24] = 0; ui22->uFluxCapControl[25] = 0; ui22->uFluxCapControl[26] = 0; ui22->uFluxCapControl[27] = 0; ui22->uFluxCapControl[28] = 0; ui22->uFluxCapControl[29] = 0; ui22->uFluxCapControl[30] = 0; ui22->uFluxCapControl[31] = 0; ui22->uFluxCapControl[32] = 0; ui22->uFluxCapControl[33] = 0; ui22->uFluxCapControl[34] = 0; ui22->uFluxCapControl[35] = 0; ui22->uFluxCapControl[36] = 0; ui22->uFluxCapControl[37] = 0; ui22->uFluxCapControl[38] = 0; ui22->uFluxCapControl[39] = 0; ui22->uFluxCapControl[40] = 0; ui22->uFluxCapControl[41] = 0; ui22->uFluxCapControl[42] = 0; ui22->uFluxCapControl[43] = 0; ui22->uFluxCapControl[44] = 0; ui22->uFluxCapControl[45] = 0; ui22->uFluxCapControl[46] = 0; ui22->uFluxCapControl[47] = 0; ui22->uFluxCapControl[48] = 0; ui22->uFluxCapControl[49] = 0; ui22->uFluxCapControl[50] = 0; ui22->uFluxCapControl[51] = 0; ui22->uFluxCapControl[52] = 0; ui22->uFluxCapControl[53] = 0; ui22->uFluxCapControl[54] = 0; ui22->uFluxCapControl[55] = 0; ui22->uFluxCapControl[56] = 0; ui22->uFluxCapControl[57] = 0; ui22->uFluxCapControl[58] = 0; ui22->uFluxCapControl[59] = 0; ui22->uFluxCapControl[60] = 0; ui22->uFluxCapControl[61] = 0; ui22->uFluxCapControl[62] = 0; ui22->uFluxCapControl[63] = 0; 
	ui22->fFluxCapData[0] = 0.000000; ui22->fFluxCapData[1] = 0.000000; ui22->fFluxCapData[2] = 0.000000; ui22->fFluxCapData[3] = 0.000000; ui22->fFluxCapData[4] = 0.000000; ui22->fFluxCapData[5] = 0.000000; ui22->fFluxCapData[6] = 0.000000; ui22->fFluxCapData[7] = 0.000000; ui22->fFluxCapData[8] = 0.000000; ui22->fFluxCapData[9] = 0.000000; ui22->fFluxCapData[10] = 0.000000; ui22->fFluxCapData[11] = 0.000000; ui22->fFluxCapData[12] = 0.000000; ui22->fFluxCapData[13] = 0.000000; ui22->fFluxCapData[14] = 0.000000; ui22->fFluxCapData[15] = 0.000000; ui22->fFluxCapData[16] = 0.000000; ui22->fFluxCapData[17] = 0.000000; ui22->fFluxCapData[18] = 0.000000; ui22->fFluxCapData[19] = 0.000000; ui22->fFluxCapData[20] = 0.000000; ui22->fFluxCapData[21] = 0.000000; ui22->fFluxCapData[22] = 0.000000; ui22->fFluxCapData[23] = 0.000000; ui22->fFluxCapData[24] = 0.000000; ui22->fFluxCapData[25] = 0.000000; ui22->fFluxCapData[26] = 0.000000; ui22->fFluxCapData[27] = 0.000000; ui22->fFluxCapData[28] = 0.000000; ui22->fFluxCapData[29] = 0.000000; ui22->fFluxCapData[30] = 0.000000; ui22->fFluxCapData[31] = 0.000000; ui22->fFluxCapData[32] = 0.000000; ui22->fFluxCapData[33] = 0.000000; ui22->fFluxCapData[34] = 0.000000; ui22->fFluxCapData[35] = 0.000000; ui22->fFluxCapData[36] = 0.000000; ui22->fFluxCapData[37] = 0.000000; ui22->fFluxCapData[38] = 0.000000; ui22->fFluxCapData[39] = 0.000000; ui22->fFluxCapData[40] = 0.000000; ui22->fFluxCapData[41] = 0.000000; ui22->fFluxCapData[42] = 0.000000; ui22->fFluxCapData[43] = 0.000000; ui22->fFluxCapData[44] = 0.000000; ui22->fFluxCapData[45] = 0.000000; ui22->fFluxCapData[46] = 0.000000; ui22->fFluxCapData[47] = 0.000000; ui22->fFluxCapData[48] = 0.000000; ui22->fFluxCapData[49] = 0.000000; ui22->fFluxCapData[50] = 0.000000; ui22->fFluxCapData[51] = 0.000000; ui22->fFluxCapData[52] = 0.000000; ui22->fFluxCapData[53] = 0.000000; ui22->fFluxCapData[54] = 0.000000; ui22->fFluxCapData[55] = 0.000000; ui22->fFluxCapData[56] = 0.000000; ui22->fFluxCapData[57] = 0.000000; ui22->fFluxCapData[58] = 0.000000; ui22->fFluxCapData[59] = 0.000000; ui22->fFluxCapData[60] = 0.000000; ui22->fFluxCapData[61] = 0.000000; ui22->fFluxCapData[62] = 0.000000; ui22->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui22);
	delete ui22;


	m_f_APF4_Delay_mSec = 21.000000;
	CUICtrl* ui23 = new CUICtrl;
	ui23->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui23->uControlId = 104;
	ui23->bLogSlider = false;
	ui23->bExpSlider = false;
	ui23->fUserDisplayDataLoLimit = 0.000000;
	ui23->fUserDisplayDataHiLimit = 100.000000;
	ui23->uUserDataType = floatData;
	ui23->fInitUserIntValue = 0;
	ui23->fInitUserFloatValue = 21.000000;
	ui23->fInitUserDoubleValue = 0;
	ui23->fInitUserUINTValue = 0;
	ui23->m_pUserCookedIntData = NULL;
	ui23->m_pUserCookedFloatData = &m_f_APF4_Delay_mSec;
	ui23->m_pUserCookedDoubleData = NULL;
	ui23->m_pUserCookedUINTData = NULL;
	ui23->cControlUnits = "mSec";
	ui23->cVariableName = "m_f_APF4_Delay_mSec";
	ui23->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui23->dPresetData[0] = 21.000000;ui23->dPresetData[1] = 0.000000;ui23->dPresetData[2] = 0.000000;ui23->dPresetData[3] = 0.000000;ui23->dPresetData[4] = 0.000000;ui23->dPresetData[5] = 0.000000;ui23->dPresetData[6] = 0.000000;ui23->dPresetData[7] = 0.000000;ui23->dPresetData[8] = 0.000000;ui23->dPresetData[9] = 0.000000;ui23->dPresetData[10] = 0.000000;ui23->dPresetData[11] = 0.000000;ui23->dPresetData[12] = 0.000000;ui23->dPresetData[13] = 0.000000;ui23->dPresetData[14] = 0.000000;ui23->dPresetData[15] = 0.000000;
	ui23->cControlName = "APF4";
	ui23->bOwnerControl = false;
	ui23->bMIDIControl = false;
	ui23->uMIDIControlCommand = 176;
	ui23->uMIDIControlName = 3;
	ui23->uMIDIControlChannel = 0;
	ui23->nGUIRow = nIndexer++;
	ui23->nGUIColumn = -1;
	ui23->bEnableParamSmoothing = false;
	ui23->fSmoothingTimeInMs = 100.00;
	ui23->uControlTheme[0] = 0; ui23->uControlTheme[1] = 0; ui23->uControlTheme[2] = 0; ui23->uControlTheme[3] = 0; ui23->uControlTheme[4] = 0; ui23->uControlTheme[5] = 0; ui23->uControlTheme[6] = 0; ui23->uControlTheme[7] = 0; ui23->uControlTheme[8] = 8; ui23->uControlTheme[9] = 0; ui23->uControlTheme[10] = 0; ui23->uControlTheme[11] = 0; ui23->uControlTheme[12] = 0; ui23->uControlTheme[13] = 0; ui23->uControlTheme[14] = 0; ui23->uControlTheme[15] = 0; ui23->uControlTheme[16] = 2; ui23->uControlTheme[17] = 0; ui23->uControlTheme[18] = 0; ui23->uControlTheme[19] = 0; ui23->uControlTheme[20] = 0; ui23->uControlTheme[21] = 0; ui23->uControlTheme[22] = 0; ui23->uControlTheme[23] = 0; ui23->uControlTheme[24] = 0; ui23->uControlTheme[25] = 0; ui23->uControlTheme[26] = 0; ui23->uControlTheme[27] = 0; ui23->uControlTheme[28] = 0; ui23->uControlTheme[29] = 0; ui23->uControlTheme[30] = 0; ui23->uControlTheme[31] = 0; 
	ui23->uFluxCapControl[0] = 0; ui23->uFluxCapControl[1] = 0; ui23->uFluxCapControl[2] = 0; ui23->uFluxCapControl[3] = 0; ui23->uFluxCapControl[4] = 0; ui23->uFluxCapControl[5] = 0; ui23->uFluxCapControl[6] = 0; ui23->uFluxCapControl[7] = 0; ui23->uFluxCapControl[8] = 0; ui23->uFluxCapControl[9] = 0; ui23->uFluxCapControl[10] = 0; ui23->uFluxCapControl[11] = 0; ui23->uFluxCapControl[12] = 0; ui23->uFluxCapControl[13] = 0; ui23->uFluxCapControl[14] = 0; ui23->uFluxCapControl[15] = 0; ui23->uFluxCapControl[16] = 0; ui23->uFluxCapControl[17] = 0; ui23->uFluxCapControl[18] = 0; ui23->uFluxCapControl[19] = 0; ui23->uFluxCapControl[20] = 0; ui23->uFluxCapControl[21] = 0; ui23->uFluxCapControl[22] = 0; ui23->uFluxCapControl[23] = 0; ui23->uFluxCapControl[24] = 0; ui23->uFluxCapControl[25] = 0; ui23->uFluxCapControl[26] = 0; ui23->uFluxCapControl[27] = 0; ui23->uFluxCapControl[28] = 0; ui23->uFluxCapControl[29] = 0; ui23->uFluxCapControl[30] = 0; ui23->uFluxCapControl[31] = 0; ui23->uFluxCapControl[32] = 0; ui23->uFluxCapControl[33] = 0; ui23->uFluxCapControl[34] = 0; ui23->uFluxCapControl[35] = 0; ui23->uFluxCapControl[36] = 0; ui23->uFluxCapControl[37] = 0; ui23->uFluxCapControl[38] = 0; ui23->uFluxCapControl[39] = 0; ui23->uFluxCapControl[40] = 0; ui23->uFluxCapControl[41] = 0; ui23->uFluxCapControl[42] = 0; ui23->uFluxCapControl[43] = 0; ui23->uFluxCapControl[44] = 0; ui23->uFluxCapControl[45] = 0; ui23->uFluxCapControl[46] = 0; ui23->uFluxCapControl[47] = 0; ui23->uFluxCapControl[48] = 0; ui23->uFluxCapControl[49] = 0; ui23->uFluxCapControl[50] = 0; ui23->uFluxCapControl[51] = 0; ui23->uFluxCapControl[52] = 0; ui23->uFluxCapControl[53] = 0; ui23->uFluxCapControl[54] = 0; ui23->uFluxCapControl[55] = 0; ui23->uFluxCapControl[56] = 0; ui23->uFluxCapControl[57] = 0; ui23->uFluxCapControl[58] = 0; ui23->uFluxCapControl[59] = 0; ui23->uFluxCapControl[60] = 0; ui23->uFluxCapControl[61] = 0; ui23->uFluxCapControl[62] = 0; ui23->uFluxCapControl[63] = 0; 
	ui23->fFluxCapData[0] = 0.000000; ui23->fFluxCapData[1] = 0.000000; ui23->fFluxCapData[2] = 0.000000; ui23->fFluxCapData[3] = 0.000000; ui23->fFluxCapData[4] = 0.000000; ui23->fFluxCapData[5] = 0.000000; ui23->fFluxCapData[6] = 0.000000; ui23->fFluxCapData[7] = 0.000000; ui23->fFluxCapData[8] = 0.000000; ui23->fFluxCapData[9] = 0.000000; ui23->fFluxCapData[10] = 0.000000; ui23->fFluxCapData[11] = 0.000000; ui23->fFluxCapData[12] = 0.000000; ui23->fFluxCapData[13] = 0.000000; ui23->fFluxCapData[14] = 0.000000; ui23->fFluxCapData[15] = 0.000000; ui23->fFluxCapData[16] = 0.000000; ui23->fFluxCapData[17] = 0.000000; ui23->fFluxCapData[18] = 0.000000; ui23->fFluxCapData[19] = 0.000000; ui23->fFluxCapData[20] = 0.000000; ui23->fFluxCapData[21] = 0.000000; ui23->fFluxCapData[22] = 0.000000; ui23->fFluxCapData[23] = 0.000000; ui23->fFluxCapData[24] = 0.000000; ui23->fFluxCapData[25] = 0.000000; ui23->fFluxCapData[26] = 0.000000; ui23->fFluxCapData[27] = 0.000000; ui23->fFluxCapData[28] = 0.000000; ui23->fFluxCapData[29] = 0.000000; ui23->fFluxCapData[30] = 0.000000; ui23->fFluxCapData[31] = 0.000000; ui23->fFluxCapData[32] = 0.000000; ui23->fFluxCapData[33] = 0.000000; ui23->fFluxCapData[34] = 0.000000; ui23->fFluxCapData[35] = 0.000000; ui23->fFluxCapData[36] = 0.000000; ui23->fFluxCapData[37] = 0.000000; ui23->fFluxCapData[38] = 0.000000; ui23->fFluxCapData[39] = 0.000000; ui23->fFluxCapData[40] = 0.000000; ui23->fFluxCapData[41] = 0.000000; ui23->fFluxCapData[42] = 0.000000; ui23->fFluxCapData[43] = 0.000000; ui23->fFluxCapData[44] = 0.000000; ui23->fFluxCapData[45] = 0.000000; ui23->fFluxCapData[46] = 0.000000; ui23->fFluxCapData[47] = 0.000000; ui23->fFluxCapData[48] = 0.000000; ui23->fFluxCapData[49] = 0.000000; ui23->fFluxCapData[50] = 0.000000; ui23->fFluxCapData[51] = 0.000000; ui23->fFluxCapData[52] = 0.000000; ui23->fFluxCapData[53] = 0.000000; ui23->fFluxCapData[54] = 0.000000; ui23->fFluxCapData[55] = 0.000000; ui23->fFluxCapData[56] = 0.000000; ui23->fFluxCapData[57] = 0.000000; ui23->fFluxCapData[58] = 0.000000; ui23->fFluxCapData[59] = 0.000000; ui23->fFluxCapData[60] = 0.000000; ui23->fFluxCapData[61] = 0.000000; ui23->fFluxCapData[62] = 0.000000; ui23->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui23);
	delete ui23;


	m_f_APF5_Delay_mSec = 13.000000;
	CUICtrl* ui24 = new CUICtrl;
	ui24->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui24->uControlId = 105;
	ui24->bLogSlider = false;
	ui24->bExpSlider = false;
	ui24->fUserDisplayDataLoLimit = 0.000000;
	ui24->fUserDisplayDataHiLimit = 100.000000;
	ui24->uUserDataType = floatData;
	ui24->fInitUserIntValue = 0;
	ui24->fInitUserFloatValue = 13.000000;
	ui24->fInitUserDoubleValue = 0;
	ui24->fInitUserUINTValue = 0;
	ui24->m_pUserCookedIntData = NULL;
	ui24->m_pUserCookedFloatData = &m_f_APF5_Delay_mSec;
	ui24->m_pUserCookedDoubleData = NULL;
	ui24->m_pUserCookedUINTData = NULL;
	ui24->cControlUnits = "mSec";
	ui24->cVariableName = "m_f_APF5_Delay_mSec";
	ui24->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui24->dPresetData[0] = 13.000000;ui24->dPresetData[1] = 0.000000;ui24->dPresetData[2] = 0.000000;ui24->dPresetData[3] = 0.000000;ui24->dPresetData[4] = 0.000000;ui24->dPresetData[5] = 0.000000;ui24->dPresetData[6] = 0.000000;ui24->dPresetData[7] = 0.000000;ui24->dPresetData[8] = 0.000000;ui24->dPresetData[9] = 0.000000;ui24->dPresetData[10] = 0.000000;ui24->dPresetData[11] = 0.000000;ui24->dPresetData[12] = 0.000000;ui24->dPresetData[13] = 0.000000;ui24->dPresetData[14] = 0.000000;ui24->dPresetData[15] = 0.000000;
	ui24->cControlName = "APF5";
	ui24->bOwnerControl = false;
	ui24->bMIDIControl = false;
	ui24->uMIDIControlCommand = 176;
	ui24->uMIDIControlName = 3;
	ui24->uMIDIControlChannel = 0;
	ui24->nGUIRow = nIndexer++;
	ui24->nGUIColumn = -1;
	ui24->bEnableParamSmoothing = false;
	ui24->fSmoothingTimeInMs = 100.00;
	ui24->uControlTheme[0] = 0; ui24->uControlTheme[1] = 0; ui24->uControlTheme[2] = 0; ui24->uControlTheme[3] = 0; ui24->uControlTheme[4] = 0; ui24->uControlTheme[5] = 0; ui24->uControlTheme[6] = 0; ui24->uControlTheme[7] = 0; ui24->uControlTheme[8] = 8; ui24->uControlTheme[9] = 0; ui24->uControlTheme[10] = 0; ui24->uControlTheme[11] = 0; ui24->uControlTheme[12] = 0; ui24->uControlTheme[13] = 0; ui24->uControlTheme[14] = 0; ui24->uControlTheme[15] = 0; ui24->uControlTheme[16] = 2; ui24->uControlTheme[17] = 0; ui24->uControlTheme[18] = 0; ui24->uControlTheme[19] = 0; ui24->uControlTheme[20] = 0; ui24->uControlTheme[21] = 0; ui24->uControlTheme[22] = 0; ui24->uControlTheme[23] = 0; ui24->uControlTheme[24] = 0; ui24->uControlTheme[25] = 0; ui24->uControlTheme[26] = 0; ui24->uControlTheme[27] = 0; ui24->uControlTheme[28] = 0; ui24->uControlTheme[29] = 0; ui24->uControlTheme[30] = 0; ui24->uControlTheme[31] = 0; 
	ui24->uFluxCapControl[0] = 0; ui24->uFluxCapControl[1] = 0; ui24->uFluxCapControl[2] = 0; ui24->uFluxCapControl[3] = 0; ui24->uFluxCapControl[4] = 0; ui24->uFluxCapControl[5] = 0; ui24->uFluxCapControl[6] = 0; ui24->uFluxCapControl[7] = 0; ui24->uFluxCapControl[8] = 0; ui24->uFluxCapControl[9] = 0; ui24->uFluxCapControl[10] = 0; ui24->uFluxCapControl[11] = 0; ui24->uFluxCapControl[12] = 0; ui24->uFluxCapControl[13] = 0; ui24->uFluxCapControl[14] = 0; ui24->uFluxCapControl[15] = 0; ui24->uFluxCapControl[16] = 0; ui24->uFluxCapControl[17] = 0; ui24->uFluxCapControl[18] = 0; ui24->uFluxCapControl[19] = 0; ui24->uFluxCapControl[20] = 0; ui24->uFluxCapControl[21] = 0; ui24->uFluxCapControl[22] = 0; ui24->uFluxCapControl[23] = 0; ui24->uFluxCapControl[24] = 0; ui24->uFluxCapControl[25] = 0; ui24->uFluxCapControl[26] = 0; ui24->uFluxCapControl[27] = 0; ui24->uFluxCapControl[28] = 0; ui24->uFluxCapControl[29] = 0; ui24->uFluxCapControl[30] = 0; ui24->uFluxCapControl[31] = 0; ui24->uFluxCapControl[32] = 0; ui24->uFluxCapControl[33] = 0; ui24->uFluxCapControl[34] = 0; ui24->uFluxCapControl[35] = 0; ui24->uFluxCapControl[36] = 0; ui24->uFluxCapControl[37] = 0; ui24->uFluxCapControl[38] = 0; ui24->uFluxCapControl[39] = 0; ui24->uFluxCapControl[40] = 0; ui24->uFluxCapControl[41] = 0; ui24->uFluxCapControl[42] = 0; ui24->uFluxCapControl[43] = 0; ui24->uFluxCapControl[44] = 0; ui24->uFluxCapControl[45] = 0; ui24->uFluxCapControl[46] = 0; ui24->uFluxCapControl[47] = 0; ui24->uFluxCapControl[48] = 0; ui24->uFluxCapControl[49] = 0; ui24->uFluxCapControl[50] = 0; ui24->uFluxCapControl[51] = 0; ui24->uFluxCapControl[52] = 0; ui24->uFluxCapControl[53] = 0; ui24->uFluxCapControl[54] = 0; ui24->uFluxCapControl[55] = 0; ui24->uFluxCapControl[56] = 0; ui24->uFluxCapControl[57] = 0; ui24->uFluxCapControl[58] = 0; ui24->uFluxCapControl[59] = 0; ui24->uFluxCapControl[60] = 0; ui24->uFluxCapControl[61] = 0; ui24->uFluxCapControl[62] = 0; ui24->uFluxCapControl[63] = 0; 
	ui24->fFluxCapData[0] = 0.000000; ui24->fFluxCapData[1] = 0.000000; ui24->fFluxCapData[2] = 0.000000; ui24->fFluxCapData[3] = 0.000000; ui24->fFluxCapData[4] = 0.000000; ui24->fFluxCapData[5] = 0.000000; ui24->fFluxCapData[6] = 0.000000; ui24->fFluxCapData[7] = 0.000000; ui24->fFluxCapData[8] = 0.000000; ui24->fFluxCapData[9] = 0.000000; ui24->fFluxCapData[10] = 0.000000; ui24->fFluxCapData[11] = 0.000000; ui24->fFluxCapData[12] = 0.000000; ui24->fFluxCapData[13] = 0.000000; ui24->fFluxCapData[14] = 0.000000; ui24->fFluxCapData[15] = 0.000000; ui24->fFluxCapData[16] = 0.000000; ui24->fFluxCapData[17] = 0.000000; ui24->fFluxCapData[18] = 0.000000; ui24->fFluxCapData[19] = 0.000000; ui24->fFluxCapData[20] = 0.000000; ui24->fFluxCapData[21] = 0.000000; ui24->fFluxCapData[22] = 0.000000; ui24->fFluxCapData[23] = 0.000000; ui24->fFluxCapData[24] = 0.000000; ui24->fFluxCapData[25] = 0.000000; ui24->fFluxCapData[26] = 0.000000; ui24->fFluxCapData[27] = 0.000000; ui24->fFluxCapData[28] = 0.000000; ui24->fFluxCapData[29] = 0.000000; ui24->fFluxCapData[30] = 0.000000; ui24->fFluxCapData[31] = 0.000000; ui24->fFluxCapData[32] = 0.000000; ui24->fFluxCapData[33] = 0.000000; ui24->fFluxCapData[34] = 0.000000; ui24->fFluxCapData[35] = 0.000000; ui24->fFluxCapData[36] = 0.000000; ui24->fFluxCapData[37] = 0.000000; ui24->fFluxCapData[38] = 0.000000; ui24->fFluxCapData[39] = 0.000000; ui24->fFluxCapData[40] = 0.000000; ui24->fFluxCapData[41] = 0.000000; ui24->fFluxCapData[42] = 0.000000; ui24->fFluxCapData[43] = 0.000000; ui24->fFluxCapData[44] = 0.000000; ui24->fFluxCapData[45] = 0.000000; ui24->fFluxCapData[46] = 0.000000; ui24->fFluxCapData[47] = 0.000000; ui24->fFluxCapData[48] = 0.000000; ui24->fFluxCapData[49] = 0.000000; ui24->fFluxCapData[50] = 0.000000; ui24->fFluxCapData[51] = 0.000000; ui24->fFluxCapData[52] = 0.000000; ui24->fFluxCapData[53] = 0.000000; ui24->fFluxCapData[54] = 0.000000; ui24->fFluxCapData[55] = 0.000000; ui24->fFluxCapData[56] = 0.000000; ui24->fFluxCapData[57] = 0.000000; ui24->fFluxCapData[58] = 0.000000; ui24->fFluxCapData[59] = 0.000000; ui24->fFluxCapData[60] = 0.000000; ui24->fFluxCapData[61] = 0.000000; ui24->fFluxCapData[62] = 0.000000; ui24->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui24);
	delete ui24;


	m_f_APF6_Delay_mSec = 35.000000;
	CUICtrl* ui25 = new CUICtrl;
	ui25->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui25->uControlId = 106;
	ui25->bLogSlider = false;
	ui25->bExpSlider = false;
	ui25->fUserDisplayDataLoLimit = 0.000000;
	ui25->fUserDisplayDataHiLimit = 100.000000;
	ui25->uUserDataType = floatData;
	ui25->fInitUserIntValue = 0;
	ui25->fInitUserFloatValue = 35.000000;
	ui25->fInitUserDoubleValue = 0;
	ui25->fInitUserUINTValue = 0;
	ui25->m_pUserCookedIntData = NULL;
	ui25->m_pUserCookedFloatData = &m_f_APF6_Delay_mSec;
	ui25->m_pUserCookedDoubleData = NULL;
	ui25->m_pUserCookedUINTData = NULL;
	ui25->cControlUnits = "mSec";
	ui25->cVariableName = "m_f_APF6_Delay_mSec";
	ui25->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui25->dPresetData[0] = 35.000000;ui25->dPresetData[1] = 0.000000;ui25->dPresetData[2] = 0.000000;ui25->dPresetData[3] = 0.000000;ui25->dPresetData[4] = 0.000000;ui25->dPresetData[5] = 0.000000;ui25->dPresetData[6] = 0.000000;ui25->dPresetData[7] = 0.000000;ui25->dPresetData[8] = 0.000000;ui25->dPresetData[9] = 0.000000;ui25->dPresetData[10] = 0.000000;ui25->dPresetData[11] = 0.000000;ui25->dPresetData[12] = 0.000000;ui25->dPresetData[13] = 0.000000;ui25->dPresetData[14] = 0.000000;ui25->dPresetData[15] = 0.000000;
	ui25->cControlName = "APF6";
	ui25->bOwnerControl = false;
	ui25->bMIDIControl = false;
	ui25->uMIDIControlCommand = 176;
	ui25->uMIDIControlName = 3;
	ui25->uMIDIControlChannel = 0;
	ui25->nGUIRow = nIndexer++;
	ui25->nGUIColumn = -1;
	ui25->bEnableParamSmoothing = false;
	ui25->fSmoothingTimeInMs = 100.00;
	ui25->uControlTheme[0] = 0; ui25->uControlTheme[1] = 0; ui25->uControlTheme[2] = 0; ui25->uControlTheme[3] = 0; ui25->uControlTheme[4] = 0; ui25->uControlTheme[5] = 0; ui25->uControlTheme[6] = 0; ui25->uControlTheme[7] = 0; ui25->uControlTheme[8] = 8; ui25->uControlTheme[9] = 0; ui25->uControlTheme[10] = 0; ui25->uControlTheme[11] = 0; ui25->uControlTheme[12] = 0; ui25->uControlTheme[13] = 0; ui25->uControlTheme[14] = 0; ui25->uControlTheme[15] = 0; ui25->uControlTheme[16] = 2; ui25->uControlTheme[17] = 0; ui25->uControlTheme[18] = 0; ui25->uControlTheme[19] = 0; ui25->uControlTheme[20] = 0; ui25->uControlTheme[21] = 0; ui25->uControlTheme[22] = 0; ui25->uControlTheme[23] = 0; ui25->uControlTheme[24] = 0; ui25->uControlTheme[25] = 0; ui25->uControlTheme[26] = 0; ui25->uControlTheme[27] = 0; ui25->uControlTheme[28] = 0; ui25->uControlTheme[29] = 0; ui25->uControlTheme[30] = 0; ui25->uControlTheme[31] = 0; 
	ui25->uFluxCapControl[0] = 0; ui25->uFluxCapControl[1] = 0; ui25->uFluxCapControl[2] = 0; ui25->uFluxCapControl[3] = 0; ui25->uFluxCapControl[4] = 0; ui25->uFluxCapControl[5] = 0; ui25->uFluxCapControl[6] = 0; ui25->uFluxCapControl[7] = 0; ui25->uFluxCapControl[8] = 0; ui25->uFluxCapControl[9] = 0; ui25->uFluxCapControl[10] = 0; ui25->uFluxCapControl[11] = 0; ui25->uFluxCapControl[12] = 0; ui25->uFluxCapControl[13] = 0; ui25->uFluxCapControl[14] = 0; ui25->uFluxCapControl[15] = 0; ui25->uFluxCapControl[16] = 0; ui25->uFluxCapControl[17] = 0; ui25->uFluxCapControl[18] = 0; ui25->uFluxCapControl[19] = 0; ui25->uFluxCapControl[20] = 0; ui25->uFluxCapControl[21] = 0; ui25->uFluxCapControl[22] = 0; ui25->uFluxCapControl[23] = 0; ui25->uFluxCapControl[24] = 0; ui25->uFluxCapControl[25] = 0; ui25->uFluxCapControl[26] = 0; ui25->uFluxCapControl[27] = 0; ui25->uFluxCapControl[28] = 0; ui25->uFluxCapControl[29] = 0; ui25->uFluxCapControl[30] = 0; ui25->uFluxCapControl[31] = 0; ui25->uFluxCapControl[32] = 0; ui25->uFluxCapControl[33] = 0; ui25->uFluxCapControl[34] = 0; ui25->uFluxCapControl[35] = 0; ui25->uFluxCapControl[36] = 0; ui25->uFluxCapControl[37] = 0; ui25->uFluxCapControl[38] = 0; ui25->uFluxCapControl[39] = 0; ui25->uFluxCapControl[40] = 0; ui25->uFluxCapControl[41] = 0; ui25->uFluxCapControl[42] = 0; ui25->uFluxCapControl[43] = 0; ui25->uFluxCapControl[44] = 0; ui25->uFluxCapControl[45] = 0; ui25->uFluxCapControl[46] = 0; ui25->uFluxCapControl[47] = 0; ui25->uFluxCapControl[48] = 0; ui25->uFluxCapControl[49] = 0; ui25->uFluxCapControl[50] = 0; ui25->uFluxCapControl[51] = 0; ui25->uFluxCapControl[52] = 0; ui25->uFluxCapControl[53] = 0; ui25->uFluxCapControl[54] = 0; ui25->uFluxCapControl[55] = 0; ui25->uFluxCapControl[56] = 0; ui25->uFluxCapControl[57] = 0; ui25->uFluxCapControl[58] = 0; ui25->uFluxCapControl[59] = 0; ui25->uFluxCapControl[60] = 0; ui25->uFluxCapControl[61] = 0; ui25->uFluxCapControl[62] = 0; ui25->uFluxCapControl[63] = 0; 
	ui25->fFluxCapData[0] = 0.000000; ui25->fFluxCapData[1] = 0.000000; ui25->fFluxCapData[2] = 0.000000; ui25->fFluxCapData[3] = 0.000000; ui25->fFluxCapData[4] = 0.000000; ui25->fFluxCapData[5] = 0.000000; ui25->fFluxCapData[6] = 0.000000; ui25->fFluxCapData[7] = 0.000000; ui25->fFluxCapData[8] = 0.000000; ui25->fFluxCapData[9] = 0.000000; ui25->fFluxCapData[10] = 0.000000; ui25->fFluxCapData[11] = 0.000000; ui25->fFluxCapData[12] = 0.000000; ui25->fFluxCapData[13] = 0.000000; ui25->fFluxCapData[14] = 0.000000; ui25->fFluxCapData[15] = 0.000000; ui25->fFluxCapData[16] = 0.000000; ui25->fFluxCapData[17] = 0.000000; ui25->fFluxCapData[18] = 0.000000; ui25->fFluxCapData[19] = 0.000000; ui25->fFluxCapData[20] = 0.000000; ui25->fFluxCapData[21] = 0.000000; ui25->fFluxCapData[22] = 0.000000; ui25->fFluxCapData[23] = 0.000000; ui25->fFluxCapData[24] = 0.000000; ui25->fFluxCapData[25] = 0.000000; ui25->fFluxCapData[26] = 0.000000; ui25->fFluxCapData[27] = 0.000000; ui25->fFluxCapData[28] = 0.000000; ui25->fFluxCapData[29] = 0.000000; ui25->fFluxCapData[30] = 0.000000; ui25->fFluxCapData[31] = 0.000000; ui25->fFluxCapData[32] = 0.000000; ui25->fFluxCapData[33] = 0.000000; ui25->fFluxCapData[34] = 0.000000; ui25->fFluxCapData[35] = 0.000000; ui25->fFluxCapData[36] = 0.000000; ui25->fFluxCapData[37] = 0.000000; ui25->fFluxCapData[38] = 0.000000; ui25->fFluxCapData[39] = 0.000000; ui25->fFluxCapData[40] = 0.000000; ui25->fFluxCapData[41] = 0.000000; ui25->fFluxCapData[42] = 0.000000; ui25->fFluxCapData[43] = 0.000000; ui25->fFluxCapData[44] = 0.000000; ui25->fFluxCapData[45] = 0.000000; ui25->fFluxCapData[46] = 0.000000; ui25->fFluxCapData[47] = 0.000000; ui25->fFluxCapData[48] = 0.000000; ui25->fFluxCapData[49] = 0.000000; ui25->fFluxCapData[50] = 0.000000; ui25->fFluxCapData[51] = 0.000000; ui25->fFluxCapData[52] = 0.000000; ui25->fFluxCapData[53] = 0.000000; ui25->fFluxCapData[54] = 0.000000; ui25->fFluxCapData[55] = 0.000000; ui25->fFluxCapData[56] = 0.000000; ui25->fFluxCapData[57] = 0.000000; ui25->fFluxCapData[58] = 0.000000; ui25->fFluxCapData[59] = 0.000000; ui25->fFluxCapData[60] = 0.000000; ui25->fFluxCapData[61] = 0.000000; ui25->fFluxCapData[62] = 0.000000; ui25->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui25);
	delete ui25;


	m_f_APF7_Delay_mSec = 9.000000;
	CUICtrl* ui26 = new CUICtrl;
	ui26->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui26->uControlId = 107;
	ui26->bLogSlider = false;
	ui26->bExpSlider = false;
	ui26->fUserDisplayDataLoLimit = 0.000000;
	ui26->fUserDisplayDataHiLimit = 100.000000;
	ui26->uUserDataType = floatData;
	ui26->fInitUserIntValue = 0;
	ui26->fInitUserFloatValue = 9.000000;
	ui26->fInitUserDoubleValue = 0;
	ui26->fInitUserUINTValue = 0;
	ui26->m_pUserCookedIntData = NULL;
	ui26->m_pUserCookedFloatData = &m_f_APF7_Delay_mSec;
	ui26->m_pUserCookedDoubleData = NULL;
	ui26->m_pUserCookedUINTData = NULL;
	ui26->cControlUnits = "mSec";
	ui26->cVariableName = "m_f_APF7_Delay_mSec";
	ui26->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui26->dPresetData[0] = 9.000000;ui26->dPresetData[1] = 0.000000;ui26->dPresetData[2] = 0.000000;ui26->dPresetData[3] = 0.000000;ui26->dPresetData[4] = 0.000000;ui26->dPresetData[5] = 0.000000;ui26->dPresetData[6] = 0.000000;ui26->dPresetData[7] = 0.000000;ui26->dPresetData[8] = 0.000000;ui26->dPresetData[9] = 0.000000;ui26->dPresetData[10] = 0.000000;ui26->dPresetData[11] = 0.000000;ui26->dPresetData[12] = 0.000000;ui26->dPresetData[13] = 0.000000;ui26->dPresetData[14] = 0.000000;ui26->dPresetData[15] = 0.000000;
	ui26->cControlName = "APF7";
	ui26->bOwnerControl = false;
	ui26->bMIDIControl = false;
	ui26->uMIDIControlCommand = 176;
	ui26->uMIDIControlName = 3;
	ui26->uMIDIControlChannel = 0;
	ui26->nGUIRow = nIndexer++;
	ui26->nGUIColumn = -1;
	ui26->bEnableParamSmoothing = false;
	ui26->fSmoothingTimeInMs = 100.00;
	ui26->uControlTheme[0] = 0; ui26->uControlTheme[1] = 0; ui26->uControlTheme[2] = 0; ui26->uControlTheme[3] = 0; ui26->uControlTheme[4] = 0; ui26->uControlTheme[5] = 0; ui26->uControlTheme[6] = 0; ui26->uControlTheme[7] = 0; ui26->uControlTheme[8] = 8; ui26->uControlTheme[9] = 0; ui26->uControlTheme[10] = 0; ui26->uControlTheme[11] = 0; ui26->uControlTheme[12] = 0; ui26->uControlTheme[13] = 0; ui26->uControlTheme[14] = 0; ui26->uControlTheme[15] = 0; ui26->uControlTheme[16] = 2; ui26->uControlTheme[17] = 0; ui26->uControlTheme[18] = 0; ui26->uControlTheme[19] = 0; ui26->uControlTheme[20] = 0; ui26->uControlTheme[21] = 0; ui26->uControlTheme[22] = 0; ui26->uControlTheme[23] = 0; ui26->uControlTheme[24] = 0; ui26->uControlTheme[25] = 0; ui26->uControlTheme[26] = 0; ui26->uControlTheme[27] = 0; ui26->uControlTheme[28] = 0; ui26->uControlTheme[29] = 0; ui26->uControlTheme[30] = 0; ui26->uControlTheme[31] = 0; 
	ui26->uFluxCapControl[0] = 0; ui26->uFluxCapControl[1] = 0; ui26->uFluxCapControl[2] = 0; ui26->uFluxCapControl[3] = 0; ui26->uFluxCapControl[4] = 0; ui26->uFluxCapControl[5] = 0; ui26->uFluxCapControl[6] = 0; ui26->uFluxCapControl[7] = 0; ui26->uFluxCapControl[8] = 0; ui26->uFluxCapControl[9] = 0; ui26->uFluxCapControl[10] = 0; ui26->uFluxCapControl[11] = 0; ui26->uFluxCapControl[12] = 0; ui26->uFluxCapControl[13] = 0; ui26->uFluxCapControl[14] = 0; ui26->uFluxCapControl[15] = 0; ui26->uFluxCapControl[16] = 0; ui26->uFluxCapControl[17] = 0; ui26->uFluxCapControl[18] = 0; ui26->uFluxCapControl[19] = 0; ui26->uFluxCapControl[20] = 0; ui26->uFluxCapControl[21] = 0; ui26->uFluxCapControl[22] = 0; ui26->uFluxCapControl[23] = 0; ui26->uFluxCapControl[24] = 0; ui26->uFluxCapControl[25] = 0; ui26->uFluxCapControl[26] = 0; ui26->uFluxCapControl[27] = 0; ui26->uFluxCapControl[28] = 0; ui26->uFluxCapControl[29] = 0; ui26->uFluxCapControl[30] = 0; ui26->uFluxCapControl[31] = 0; ui26->uFluxCapControl[32] = 0; ui26->uFluxCapControl[33] = 0; ui26->uFluxCapControl[34] = 0; ui26->uFluxCapControl[35] = 0; ui26->uFluxCapControl[36] = 0; ui26->uFluxCapControl[37] = 0; ui26->uFluxCapControl[38] = 0; ui26->uFluxCapControl[39] = 0; ui26->uFluxCapControl[40] = 0; ui26->uFluxCapControl[41] = 0; ui26->uFluxCapControl[42] = 0; ui26->uFluxCapControl[43] = 0; ui26->uFluxCapControl[44] = 0; ui26->uFluxCapControl[45] = 0; ui26->uFluxCapControl[46] = 0; ui26->uFluxCapControl[47] = 0; ui26->uFluxCapControl[48] = 0; ui26->uFluxCapControl[49] = 0; ui26->uFluxCapControl[50] = 0; ui26->uFluxCapControl[51] = 0; ui26->uFluxCapControl[52] = 0; ui26->uFluxCapControl[53] = 0; ui26->uFluxCapControl[54] = 0; ui26->uFluxCapControl[55] = 0; ui26->uFluxCapControl[56] = 0; ui26->uFluxCapControl[57] = 0; ui26->uFluxCapControl[58] = 0; ui26->uFluxCapControl[59] = 0; ui26->uFluxCapControl[60] = 0; ui26->uFluxCapControl[61] = 0; ui26->uFluxCapControl[62] = 0; ui26->uFluxCapControl[63] = 0; 
	ui26->fFluxCapData[0] = 0.000000; ui26->fFluxCapData[1] = 0.000000; ui26->fFluxCapData[2] = 0.000000; ui26->fFluxCapData[3] = 0.000000; ui26->fFluxCapData[4] = 0.000000; ui26->fFluxCapData[5] = 0.000000; ui26->fFluxCapData[6] = 0.000000; ui26->fFluxCapData[7] = 0.000000; ui26->fFluxCapData[8] = 0.000000; ui26->fFluxCapData[9] = 0.000000; ui26->fFluxCapData[10] = 0.000000; ui26->fFluxCapData[11] = 0.000000; ui26->fFluxCapData[12] = 0.000000; ui26->fFluxCapData[13] = 0.000000; ui26->fFluxCapData[14] = 0.000000; ui26->fFluxCapData[15] = 0.000000; ui26->fFluxCapData[16] = 0.000000; ui26->fFluxCapData[17] = 0.000000; ui26->fFluxCapData[18] = 0.000000; ui26->fFluxCapData[19] = 0.000000; ui26->fFluxCapData[20] = 0.000000; ui26->fFluxCapData[21] = 0.000000; ui26->fFluxCapData[22] = 0.000000; ui26->fFluxCapData[23] = 0.000000; ui26->fFluxCapData[24] = 0.000000; ui26->fFluxCapData[25] = 0.000000; ui26->fFluxCapData[26] = 0.000000; ui26->fFluxCapData[27] = 0.000000; ui26->fFluxCapData[28] = 0.000000; ui26->fFluxCapData[29] = 0.000000; ui26->fFluxCapData[30] = 0.000000; ui26->fFluxCapData[31] = 0.000000; ui26->fFluxCapData[32] = 0.000000; ui26->fFluxCapData[33] = 0.000000; ui26->fFluxCapData[34] = 0.000000; ui26->fFluxCapData[35] = 0.000000; ui26->fFluxCapData[36] = 0.000000; ui26->fFluxCapData[37] = 0.000000; ui26->fFluxCapData[38] = 0.000000; ui26->fFluxCapData[39] = 0.000000; ui26->fFluxCapData[40] = 0.000000; ui26->fFluxCapData[41] = 0.000000; ui26->fFluxCapData[42] = 0.000000; ui26->fFluxCapData[43] = 0.000000; ui26->fFluxCapData[44] = 0.000000; ui26->fFluxCapData[45] = 0.000000; ui26->fFluxCapData[46] = 0.000000; ui26->fFluxCapData[47] = 0.000000; ui26->fFluxCapData[48] = 0.000000; ui26->fFluxCapData[49] = 0.000000; ui26->fFluxCapData[50] = 0.000000; ui26->fFluxCapData[51] = 0.000000; ui26->fFluxCapData[52] = 0.000000; ui26->fFluxCapData[53] = 0.000000; ui26->fFluxCapData[54] = 0.000000; ui26->fFluxCapData[55] = 0.000000; ui26->fFluxCapData[56] = 0.000000; ui26->fFluxCapData[57] = 0.000000; ui26->fFluxCapData[58] = 0.000000; ui26->fFluxCapData[59] = 0.000000; ui26->fFluxCapData[60] = 0.000000; ui26->fFluxCapData[61] = 0.000000; ui26->fFluxCapData[62] = 0.000000; ui26->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui26);
	delete ui26;


	m_f_APF8_Delay_mSec = 25.500000;
	CUICtrl* ui27 = new CUICtrl;
	ui27->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui27->uControlId = 108;
	ui27->bLogSlider = false;
	ui27->bExpSlider = false;
	ui27->fUserDisplayDataLoLimit = 0.000000;
	ui27->fUserDisplayDataHiLimit = 100.000000;
	ui27->uUserDataType = floatData;
	ui27->fInitUserIntValue = 0;
	ui27->fInitUserFloatValue = 25.500000;
	ui27->fInitUserDoubleValue = 0;
	ui27->fInitUserUINTValue = 0;
	ui27->m_pUserCookedIntData = NULL;
	ui27->m_pUserCookedFloatData = &m_f_APF8_Delay_mSec;
	ui27->m_pUserCookedDoubleData = NULL;
	ui27->m_pUserCookedUINTData = NULL;
	ui27->cControlUnits = "mSec";
	ui27->cVariableName = "m_f_APF8_Delay_mSec";
	ui27->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui27->dPresetData[0] = 25.500000;ui27->dPresetData[1] = 0.000000;ui27->dPresetData[2] = 0.000000;ui27->dPresetData[3] = 0.000000;ui27->dPresetData[4] = 0.000000;ui27->dPresetData[5] = 0.000000;ui27->dPresetData[6] = 0.000000;ui27->dPresetData[7] = 0.000000;ui27->dPresetData[8] = 0.000000;ui27->dPresetData[9] = 0.000000;ui27->dPresetData[10] = 0.000000;ui27->dPresetData[11] = 0.000000;ui27->dPresetData[12] = 0.000000;ui27->dPresetData[13] = 0.000000;ui27->dPresetData[14] = 0.000000;ui27->dPresetData[15] = 0.000000;
	ui27->cControlName = "APF8";
	ui27->bOwnerControl = false;
	ui27->bMIDIControl = false;
	ui27->uMIDIControlCommand = 176;
	ui27->uMIDIControlName = 3;
	ui27->uMIDIControlChannel = 0;
	ui27->nGUIRow = nIndexer++;
	ui27->nGUIColumn = -1;
	ui27->bEnableParamSmoothing = false;
	ui27->fSmoothingTimeInMs = 100.00;
	ui27->uControlTheme[0] = 0; ui27->uControlTheme[1] = 0; ui27->uControlTheme[2] = 0; ui27->uControlTheme[3] = 0; ui27->uControlTheme[4] = 0; ui27->uControlTheme[5] = 0; ui27->uControlTheme[6] = 0; ui27->uControlTheme[7] = 0; ui27->uControlTheme[8] = 8; ui27->uControlTheme[9] = 0; ui27->uControlTheme[10] = 0; ui27->uControlTheme[11] = 0; ui27->uControlTheme[12] = 0; ui27->uControlTheme[13] = 0; ui27->uControlTheme[14] = 0; ui27->uControlTheme[15] = 0; ui27->uControlTheme[16] = 2; ui27->uControlTheme[17] = 0; ui27->uControlTheme[18] = 0; ui27->uControlTheme[19] = 0; ui27->uControlTheme[20] = 0; ui27->uControlTheme[21] = 0; ui27->uControlTheme[22] = 0; ui27->uControlTheme[23] = 0; ui27->uControlTheme[24] = 0; ui27->uControlTheme[25] = 0; ui27->uControlTheme[26] = 0; ui27->uControlTheme[27] = 0; ui27->uControlTheme[28] = 0; ui27->uControlTheme[29] = 0; ui27->uControlTheme[30] = 0; ui27->uControlTheme[31] = 0; 
	ui27->uFluxCapControl[0] = 0; ui27->uFluxCapControl[1] = 0; ui27->uFluxCapControl[2] = 0; ui27->uFluxCapControl[3] = 0; ui27->uFluxCapControl[4] = 0; ui27->uFluxCapControl[5] = 0; ui27->uFluxCapControl[6] = 0; ui27->uFluxCapControl[7] = 0; ui27->uFluxCapControl[8] = 0; ui27->uFluxCapControl[9] = 0; ui27->uFluxCapControl[10] = 0; ui27->uFluxCapControl[11] = 0; ui27->uFluxCapControl[12] = 0; ui27->uFluxCapControl[13] = 0; ui27->uFluxCapControl[14] = 0; ui27->uFluxCapControl[15] = 0; ui27->uFluxCapControl[16] = 0; ui27->uFluxCapControl[17] = 0; ui27->uFluxCapControl[18] = 0; ui27->uFluxCapControl[19] = 0; ui27->uFluxCapControl[20] = 0; ui27->uFluxCapControl[21] = 0; ui27->uFluxCapControl[22] = 0; ui27->uFluxCapControl[23] = 0; ui27->uFluxCapControl[24] = 0; ui27->uFluxCapControl[25] = 0; ui27->uFluxCapControl[26] = 0; ui27->uFluxCapControl[27] = 0; ui27->uFluxCapControl[28] = 0; ui27->uFluxCapControl[29] = 0; ui27->uFluxCapControl[30] = 0; ui27->uFluxCapControl[31] = 0; ui27->uFluxCapControl[32] = 0; ui27->uFluxCapControl[33] = 0; ui27->uFluxCapControl[34] = 0; ui27->uFluxCapControl[35] = 0; ui27->uFluxCapControl[36] = 0; ui27->uFluxCapControl[37] = 0; ui27->uFluxCapControl[38] = 0; ui27->uFluxCapControl[39] = 0; ui27->uFluxCapControl[40] = 0; ui27->uFluxCapControl[41] = 0; ui27->uFluxCapControl[42] = 0; ui27->uFluxCapControl[43] = 0; ui27->uFluxCapControl[44] = 0; ui27->uFluxCapControl[45] = 0; ui27->uFluxCapControl[46] = 0; ui27->uFluxCapControl[47] = 0; ui27->uFluxCapControl[48] = 0; ui27->uFluxCapControl[49] = 0; ui27->uFluxCapControl[50] = 0; ui27->uFluxCapControl[51] = 0; ui27->uFluxCapControl[52] = 0; ui27->uFluxCapControl[53] = 0; ui27->uFluxCapControl[54] = 0; ui27->uFluxCapControl[55] = 0; ui27->uFluxCapControl[56] = 0; ui27->uFluxCapControl[57] = 0; ui27->uFluxCapControl[58] = 0; ui27->uFluxCapControl[59] = 0; ui27->uFluxCapControl[60] = 0; ui27->uFluxCapControl[61] = 0; ui27->uFluxCapControl[62] = 0; ui27->uFluxCapControl[63] = 0; 
	ui27->fFluxCapData[0] = 0.000000; ui27->fFluxCapData[1] = 0.000000; ui27->fFluxCapData[2] = 0.000000; ui27->fFluxCapData[3] = 0.000000; ui27->fFluxCapData[4] = 0.000000; ui27->fFluxCapData[5] = 0.000000; ui27->fFluxCapData[6] = 0.000000; ui27->fFluxCapData[7] = 0.000000; ui27->fFluxCapData[8] = 0.000000; ui27->fFluxCapData[9] = 0.000000; ui27->fFluxCapData[10] = 0.000000; ui27->fFluxCapData[11] = 0.000000; ui27->fFluxCapData[12] = 0.000000; ui27->fFluxCapData[13] = 0.000000; ui27->fFluxCapData[14] = 0.000000; ui27->fFluxCapData[15] = 0.000000; ui27->fFluxCapData[16] = 0.000000; ui27->fFluxCapData[17] = 0.000000; ui27->fFluxCapData[18] = 0.000000; ui27->fFluxCapData[19] = 0.000000; ui27->fFluxCapData[20] = 0.000000; ui27->fFluxCapData[21] = 0.000000; ui27->fFluxCapData[22] = 0.000000; ui27->fFluxCapData[23] = 0.000000; ui27->fFluxCapData[24] = 0.000000; ui27->fFluxCapData[25] = 0.000000; ui27->fFluxCapData[26] = 0.000000; ui27->fFluxCapData[27] = 0.000000; ui27->fFluxCapData[28] = 0.000000; ui27->fFluxCapData[29] = 0.000000; ui27->fFluxCapData[30] = 0.000000; ui27->fFluxCapData[31] = 0.000000; ui27->fFluxCapData[32] = 0.000000; ui27->fFluxCapData[33] = 0.000000; ui27->fFluxCapData[34] = 0.000000; ui27->fFluxCapData[35] = 0.000000; ui27->fFluxCapData[36] = 0.000000; ui27->fFluxCapData[37] = 0.000000; ui27->fFluxCapData[38] = 0.000000; ui27->fFluxCapData[39] = 0.000000; ui27->fFluxCapData[40] = 0.000000; ui27->fFluxCapData[41] = 0.000000; ui27->fFluxCapData[42] = 0.000000; ui27->fFluxCapData[43] = 0.000000; ui27->fFluxCapData[44] = 0.000000; ui27->fFluxCapData[45] = 0.000000; ui27->fFluxCapData[46] = 0.000000; ui27->fFluxCapData[47] = 0.000000; ui27->fFluxCapData[48] = 0.000000; ui27->fFluxCapData[49] = 0.000000; ui27->fFluxCapData[50] = 0.000000; ui27->fFluxCapData[51] = 0.000000; ui27->fFluxCapData[52] = 0.000000; ui27->fFluxCapData[53] = 0.000000; ui27->fFluxCapData[54] = 0.000000; ui27->fFluxCapData[55] = 0.000000; ui27->fFluxCapData[56] = 0.000000; ui27->fFluxCapData[57] = 0.000000; ui27->fFluxCapData[58] = 0.000000; ui27->fFluxCapData[59] = 0.000000; ui27->fFluxCapData[60] = 0.000000; ui27->fFluxCapData[61] = 0.000000; ui27->fFluxCapData[62] = 0.000000; ui27->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui27);
	delete ui27;


	m_fDelay1_mSec = 9.000000;
	CUICtrl* ui28 = new CUICtrl;
	ui28->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui28->uControlId = 111;
	ui28->bLogSlider = false;
	ui28->bExpSlider = false;
	ui28->fUserDisplayDataLoLimit = 0.000000;
	ui28->fUserDisplayDataHiLimit = 100.000000;
	ui28->uUserDataType = floatData;
	ui28->fInitUserIntValue = 0;
	ui28->fInitUserFloatValue = 9.000000;
	ui28->fInitUserDoubleValue = 0;
	ui28->fInitUserUINTValue = 0;
	ui28->m_pUserCookedIntData = NULL;
	ui28->m_pUserCookedFloatData = &m_fDelay1_mSec;
	ui28->m_pUserCookedDoubleData = NULL;
	ui28->m_pUserCookedUINTData = NULL;
	ui28->cControlUnits = "mSec";
	ui28->cVariableName = "m_fDelay1_mSec";
	ui28->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui28->dPresetData[0] = 9.000000;ui28->dPresetData[1] = 0.000000;ui28->dPresetData[2] = 0.000000;ui28->dPresetData[3] = 0.000000;ui28->dPresetData[4] = 0.000000;ui28->dPresetData[5] = 0.000000;ui28->dPresetData[6] = 0.000000;ui28->dPresetData[7] = 0.000000;ui28->dPresetData[8] = 0.000000;ui28->dPresetData[9] = 0.000000;ui28->dPresetData[10] = 0.000000;ui28->dPresetData[11] = 0.000000;ui28->dPresetData[12] = 0.000000;ui28->dPresetData[13] = 0.000000;ui28->dPresetData[14] = 0.000000;ui28->dPresetData[15] = 0.000000;
	ui28->cControlName = "Delay 1";
	ui28->bOwnerControl = false;
	ui28->bMIDIControl = false;
	ui28->uMIDIControlCommand = 176;
	ui28->uMIDIControlName = 3;
	ui28->uMIDIControlChannel = 0;
	ui28->nGUIRow = nIndexer++;
	ui28->nGUIColumn = -1;
	ui28->bEnableParamSmoothing = false;
	ui28->fSmoothingTimeInMs = 100.00;
	ui28->uControlTheme[0] = 0; ui28->uControlTheme[1] = 0; ui28->uControlTheme[2] = 0; ui28->uControlTheme[3] = 0; ui28->uControlTheme[4] = 0; ui28->uControlTheme[5] = 0; ui28->uControlTheme[6] = 0; ui28->uControlTheme[7] = 0; ui28->uControlTheme[8] = 10; ui28->uControlTheme[9] = 0; ui28->uControlTheme[10] = 0; ui28->uControlTheme[11] = 0; ui28->uControlTheme[12] = 0; ui28->uControlTheme[13] = 0; ui28->uControlTheme[14] = 0; ui28->uControlTheme[15] = 0; ui28->uControlTheme[16] = 2; ui28->uControlTheme[17] = 0; ui28->uControlTheme[18] = 0; ui28->uControlTheme[19] = 0; ui28->uControlTheme[20] = 0; ui28->uControlTheme[21] = 0; ui28->uControlTheme[22] = 0; ui28->uControlTheme[23] = 0; ui28->uControlTheme[24] = 0; ui28->uControlTheme[25] = 0; ui28->uControlTheme[26] = 0; ui28->uControlTheme[27] = 0; ui28->uControlTheme[28] = 0; ui28->uControlTheme[29] = 0; ui28->uControlTheme[30] = 0; ui28->uControlTheme[31] = 0; 
	ui28->uFluxCapControl[0] = 0; ui28->uFluxCapControl[1] = 0; ui28->uFluxCapControl[2] = 0; ui28->uFluxCapControl[3] = 0; ui28->uFluxCapControl[4] = 0; ui28->uFluxCapControl[5] = 0; ui28->uFluxCapControl[6] = 0; ui28->uFluxCapControl[7] = 0; ui28->uFluxCapControl[8] = 0; ui28->uFluxCapControl[9] = 0; ui28->uFluxCapControl[10] = 0; ui28->uFluxCapControl[11] = 0; ui28->uFluxCapControl[12] = 0; ui28->uFluxCapControl[13] = 0; ui28->uFluxCapControl[14] = 0; ui28->uFluxCapControl[15] = 0; ui28->uFluxCapControl[16] = 0; ui28->uFluxCapControl[17] = 0; ui28->uFluxCapControl[18] = 0; ui28->uFluxCapControl[19] = 0; ui28->uFluxCapControl[20] = 0; ui28->uFluxCapControl[21] = 0; ui28->uFluxCapControl[22] = 0; ui28->uFluxCapControl[23] = 0; ui28->uFluxCapControl[24] = 0; ui28->uFluxCapControl[25] = 0; ui28->uFluxCapControl[26] = 0; ui28->uFluxCapControl[27] = 0; ui28->uFluxCapControl[28] = 0; ui28->uFluxCapControl[29] = 0; ui28->uFluxCapControl[30] = 0; ui28->uFluxCapControl[31] = 0; ui28->uFluxCapControl[32] = 0; ui28->uFluxCapControl[33] = 0; ui28->uFluxCapControl[34] = 0; ui28->uFluxCapControl[35] = 0; ui28->uFluxCapControl[36] = 0; ui28->uFluxCapControl[37] = 0; ui28->uFluxCapControl[38] = 0; ui28->uFluxCapControl[39] = 0; ui28->uFluxCapControl[40] = 0; ui28->uFluxCapControl[41] = 0; ui28->uFluxCapControl[42] = 0; ui28->uFluxCapControl[43] = 0; ui28->uFluxCapControl[44] = 0; ui28->uFluxCapControl[45] = 0; ui28->uFluxCapControl[46] = 0; ui28->uFluxCapControl[47] = 0; ui28->uFluxCapControl[48] = 0; ui28->uFluxCapControl[49] = 0; ui28->uFluxCapControl[50] = 0; ui28->uFluxCapControl[51] = 0; ui28->uFluxCapControl[52] = 0; ui28->uFluxCapControl[53] = 0; ui28->uFluxCapControl[54] = 0; ui28->uFluxCapControl[55] = 0; ui28->uFluxCapControl[56] = 0; ui28->uFluxCapControl[57] = 0; ui28->uFluxCapControl[58] = 0; ui28->uFluxCapControl[59] = 0; ui28->uFluxCapControl[60] = 0; ui28->uFluxCapControl[61] = 0; ui28->uFluxCapControl[62] = 0; ui28->uFluxCapControl[63] = 0; 
	ui28->fFluxCapData[0] = 0.000000; ui28->fFluxCapData[1] = 0.000000; ui28->fFluxCapData[2] = 0.000000; ui28->fFluxCapData[3] = 0.000000; ui28->fFluxCapData[4] = 0.000000; ui28->fFluxCapData[5] = 0.000000; ui28->fFluxCapData[6] = 0.000000; ui28->fFluxCapData[7] = 0.000000; ui28->fFluxCapData[8] = 0.000000; ui28->fFluxCapData[9] = 0.000000; ui28->fFluxCapData[10] = 0.000000; ui28->fFluxCapData[11] = 0.000000; ui28->fFluxCapData[12] = 0.000000; ui28->fFluxCapData[13] = 0.000000; ui28->fFluxCapData[14] = 0.000000; ui28->fFluxCapData[15] = 0.000000; ui28->fFluxCapData[16] = 0.000000; ui28->fFluxCapData[17] = 0.000000; ui28->fFluxCapData[18] = 0.000000; ui28->fFluxCapData[19] = 0.000000; ui28->fFluxCapData[20] = 0.000000; ui28->fFluxCapData[21] = 0.000000; ui28->fFluxCapData[22] = 0.000000; ui28->fFluxCapData[23] = 0.000000; ui28->fFluxCapData[24] = 0.000000; ui28->fFluxCapData[25] = 0.000000; ui28->fFluxCapData[26] = 0.000000; ui28->fFluxCapData[27] = 0.000000; ui28->fFluxCapData[28] = 0.000000; ui28->fFluxCapData[29] = 0.000000; ui28->fFluxCapData[30] = 0.000000; ui28->fFluxCapData[31] = 0.000000; ui28->fFluxCapData[32] = 0.000000; ui28->fFluxCapData[33] = 0.000000; ui28->fFluxCapData[34] = 0.000000; ui28->fFluxCapData[35] = 0.000000; ui28->fFluxCapData[36] = 0.000000; ui28->fFluxCapData[37] = 0.000000; ui28->fFluxCapData[38] = 0.000000; ui28->fFluxCapData[39] = 0.000000; ui28->fFluxCapData[40] = 0.000000; ui28->fFluxCapData[41] = 0.000000; ui28->fFluxCapData[42] = 0.000000; ui28->fFluxCapData[43] = 0.000000; ui28->fFluxCapData[44] = 0.000000; ui28->fFluxCapData[45] = 0.000000; ui28->fFluxCapData[46] = 0.000000; ui28->fFluxCapData[47] = 0.000000; ui28->fFluxCapData[48] = 0.000000; ui28->fFluxCapData[49] = 0.000000; ui28->fFluxCapData[50] = 0.000000; ui28->fFluxCapData[51] = 0.000000; ui28->fFluxCapData[52] = 0.000000; ui28->fFluxCapData[53] = 0.000000; ui28->fFluxCapData[54] = 0.000000; ui28->fFluxCapData[55] = 0.000000; ui28->fFluxCapData[56] = 0.000000; ui28->fFluxCapData[57] = 0.000000; ui28->fFluxCapData[58] = 0.000000; ui28->fFluxCapData[59] = 0.000000; ui28->fFluxCapData[60] = 0.000000; ui28->fFluxCapData[61] = 0.000000; ui28->fFluxCapData[62] = 0.000000; ui28->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui28);
	delete ui28;


	m_fDelay2_mSec = 11.000000;
	CUICtrl* ui29 = new CUICtrl;
	ui29->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui29->uControlId = 112;
	ui29->bLogSlider = false;
	ui29->bExpSlider = false;
	ui29->fUserDisplayDataLoLimit = 0.000000;
	ui29->fUserDisplayDataHiLimit = 100.000000;
	ui29->uUserDataType = floatData;
	ui29->fInitUserIntValue = 0;
	ui29->fInitUserFloatValue = 11.000000;
	ui29->fInitUserDoubleValue = 0;
	ui29->fInitUserUINTValue = 0;
	ui29->m_pUserCookedIntData = NULL;
	ui29->m_pUserCookedFloatData = &m_fDelay2_mSec;
	ui29->m_pUserCookedDoubleData = NULL;
	ui29->m_pUserCookedUINTData = NULL;
	ui29->cControlUnits = "mSec";
	ui29->cVariableName = "m_fDelay2_mSec";
	ui29->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui29->dPresetData[0] = 11.000000;ui29->dPresetData[1] = 0.000000;ui29->dPresetData[2] = 0.000000;ui29->dPresetData[3] = 0.000000;ui29->dPresetData[4] = 0.000000;ui29->dPresetData[5] = 0.000000;ui29->dPresetData[6] = 0.000000;ui29->dPresetData[7] = 0.000000;ui29->dPresetData[8] = 0.000000;ui29->dPresetData[9] = 0.000000;ui29->dPresetData[10] = 0.000000;ui29->dPresetData[11] = 0.000000;ui29->dPresetData[12] = 0.000000;ui29->dPresetData[13] = 0.000000;ui29->dPresetData[14] = 0.000000;ui29->dPresetData[15] = 0.000000;
	ui29->cControlName = "Delay 2";
	ui29->bOwnerControl = false;
	ui29->bMIDIControl = false;
	ui29->uMIDIControlCommand = 176;
	ui29->uMIDIControlName = 3;
	ui29->uMIDIControlChannel = 0;
	ui29->nGUIRow = nIndexer++;
	ui29->nGUIColumn = -1;
	ui29->bEnableParamSmoothing = false;
	ui29->fSmoothingTimeInMs = 100.00;
	ui29->uControlTheme[0] = 0; ui29->uControlTheme[1] = 0; ui29->uControlTheme[2] = 0; ui29->uControlTheme[3] = 0; ui29->uControlTheme[4] = 0; ui29->uControlTheme[5] = 0; ui29->uControlTheme[6] = 0; ui29->uControlTheme[7] = 0; ui29->uControlTheme[8] = 10; ui29->uControlTheme[9] = 0; ui29->uControlTheme[10] = 0; ui29->uControlTheme[11] = 0; ui29->uControlTheme[12] = 0; ui29->uControlTheme[13] = 0; ui29->uControlTheme[14] = 0; ui29->uControlTheme[15] = 0; ui29->uControlTheme[16] = 2; ui29->uControlTheme[17] = 0; ui29->uControlTheme[18] = 0; ui29->uControlTheme[19] = 0; ui29->uControlTheme[20] = 0; ui29->uControlTheme[21] = 0; ui29->uControlTheme[22] = 0; ui29->uControlTheme[23] = 0; ui29->uControlTheme[24] = 0; ui29->uControlTheme[25] = 0; ui29->uControlTheme[26] = 0; ui29->uControlTheme[27] = 0; ui29->uControlTheme[28] = 0; ui29->uControlTheme[29] = 0; ui29->uControlTheme[30] = 0; ui29->uControlTheme[31] = 0; 
	ui29->uFluxCapControl[0] = 0; ui29->uFluxCapControl[1] = 0; ui29->uFluxCapControl[2] = 0; ui29->uFluxCapControl[3] = 0; ui29->uFluxCapControl[4] = 0; ui29->uFluxCapControl[5] = 0; ui29->uFluxCapControl[6] = 0; ui29->uFluxCapControl[7] = 0; ui29->uFluxCapControl[8] = 0; ui29->uFluxCapControl[9] = 0; ui29->uFluxCapControl[10] = 0; ui29->uFluxCapControl[11] = 0; ui29->uFluxCapControl[12] = 0; ui29->uFluxCapControl[13] = 0; ui29->uFluxCapControl[14] = 0; ui29->uFluxCapControl[15] = 0; ui29->uFluxCapControl[16] = 0; ui29->uFluxCapControl[17] = 0; ui29->uFluxCapControl[18] = 0; ui29->uFluxCapControl[19] = 0; ui29->uFluxCapControl[20] = 0; ui29->uFluxCapControl[21] = 0; ui29->uFluxCapControl[22] = 0; ui29->uFluxCapControl[23] = 0; ui29->uFluxCapControl[24] = 0; ui29->uFluxCapControl[25] = 0; ui29->uFluxCapControl[26] = 0; ui29->uFluxCapControl[27] = 0; ui29->uFluxCapControl[28] = 0; ui29->uFluxCapControl[29] = 0; ui29->uFluxCapControl[30] = 0; ui29->uFluxCapControl[31] = 0; ui29->uFluxCapControl[32] = 0; ui29->uFluxCapControl[33] = 0; ui29->uFluxCapControl[34] = 0; ui29->uFluxCapControl[35] = 0; ui29->uFluxCapControl[36] = 0; ui29->uFluxCapControl[37] = 0; ui29->uFluxCapControl[38] = 0; ui29->uFluxCapControl[39] = 0; ui29->uFluxCapControl[40] = 0; ui29->uFluxCapControl[41] = 0; ui29->uFluxCapControl[42] = 0; ui29->uFluxCapControl[43] = 0; ui29->uFluxCapControl[44] = 0; ui29->uFluxCapControl[45] = 0; ui29->uFluxCapControl[46] = 0; ui29->uFluxCapControl[47] = 0; ui29->uFluxCapControl[48] = 0; ui29->uFluxCapControl[49] = 0; ui29->uFluxCapControl[50] = 0; ui29->uFluxCapControl[51] = 0; ui29->uFluxCapControl[52] = 0; ui29->uFluxCapControl[53] = 0; ui29->uFluxCapControl[54] = 0; ui29->uFluxCapControl[55] = 0; ui29->uFluxCapControl[56] = 0; ui29->uFluxCapControl[57] = 0; ui29->uFluxCapControl[58] = 0; ui29->uFluxCapControl[59] = 0; ui29->uFluxCapControl[60] = 0; ui29->uFluxCapControl[61] = 0; ui29->uFluxCapControl[62] = 0; ui29->uFluxCapControl[63] = 0; 
	ui29->fFluxCapData[0] = 0.000000; ui29->fFluxCapData[1] = 0.000000; ui29->fFluxCapData[2] = 0.000000; ui29->fFluxCapData[3] = 0.000000; ui29->fFluxCapData[4] = 0.000000; ui29->fFluxCapData[5] = 0.000000; ui29->fFluxCapData[6] = 0.000000; ui29->fFluxCapData[7] = 0.000000; ui29->fFluxCapData[8] = 0.000000; ui29->fFluxCapData[9] = 0.000000; ui29->fFluxCapData[10] = 0.000000; ui29->fFluxCapData[11] = 0.000000; ui29->fFluxCapData[12] = 0.000000; ui29->fFluxCapData[13] = 0.000000; ui29->fFluxCapData[14] = 0.000000; ui29->fFluxCapData[15] = 0.000000; ui29->fFluxCapData[16] = 0.000000; ui29->fFluxCapData[17] = 0.000000; ui29->fFluxCapData[18] = 0.000000; ui29->fFluxCapData[19] = 0.000000; ui29->fFluxCapData[20] = 0.000000; ui29->fFluxCapData[21] = 0.000000; ui29->fFluxCapData[22] = 0.000000; ui29->fFluxCapData[23] = 0.000000; ui29->fFluxCapData[24] = 0.000000; ui29->fFluxCapData[25] = 0.000000; ui29->fFluxCapData[26] = 0.000000; ui29->fFluxCapData[27] = 0.000000; ui29->fFluxCapData[28] = 0.000000; ui29->fFluxCapData[29] = 0.000000; ui29->fFluxCapData[30] = 0.000000; ui29->fFluxCapData[31] = 0.000000; ui29->fFluxCapData[32] = 0.000000; ui29->fFluxCapData[33] = 0.000000; ui29->fFluxCapData[34] = 0.000000; ui29->fFluxCapData[35] = 0.000000; ui29->fFluxCapData[36] = 0.000000; ui29->fFluxCapData[37] = 0.000000; ui29->fFluxCapData[38] = 0.000000; ui29->fFluxCapData[39] = 0.000000; ui29->fFluxCapData[40] = 0.000000; ui29->fFluxCapData[41] = 0.000000; ui29->fFluxCapData[42] = 0.000000; ui29->fFluxCapData[43] = 0.000000; ui29->fFluxCapData[44] = 0.000000; ui29->fFluxCapData[45] = 0.000000; ui29->fFluxCapData[46] = 0.000000; ui29->fFluxCapData[47] = 0.000000; ui29->fFluxCapData[48] = 0.000000; ui29->fFluxCapData[49] = 0.000000; ui29->fFluxCapData[50] = 0.000000; ui29->fFluxCapData[51] = 0.000000; ui29->fFluxCapData[52] = 0.000000; ui29->fFluxCapData[53] = 0.000000; ui29->fFluxCapData[54] = 0.000000; ui29->fFluxCapData[55] = 0.000000; ui29->fFluxCapData[56] = 0.000000; ui29->fFluxCapData[57] = 0.000000; ui29->fFluxCapData[58] = 0.000000; ui29->fFluxCapData[59] = 0.000000; ui29->fFluxCapData[60] = 0.000000; ui29->fFluxCapData[61] = 0.000000; ui29->fFluxCapData[62] = 0.000000; ui29->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui29);
	delete ui29;


	m_fDelay3_mSec = 18.000000;
	CUICtrl* ui30 = new CUICtrl;
	ui30->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui30->uControlId = 113;
	ui30->bLogSlider = false;
	ui30->bExpSlider = false;
	ui30->fUserDisplayDataLoLimit = 0.000000;
	ui30->fUserDisplayDataHiLimit = 100.000000;
	ui30->uUserDataType = floatData;
	ui30->fInitUserIntValue = 0;
	ui30->fInitUserFloatValue = 18.000000;
	ui30->fInitUserDoubleValue = 0;
	ui30->fInitUserUINTValue = 0;
	ui30->m_pUserCookedIntData = NULL;
	ui30->m_pUserCookedFloatData = &m_fDelay3_mSec;
	ui30->m_pUserCookedDoubleData = NULL;
	ui30->m_pUserCookedUINTData = NULL;
	ui30->cControlUnits = "mSec";
	ui30->cVariableName = "m_fDelay3_mSec";
	ui30->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui30->dPresetData[0] = 18.000000;ui30->dPresetData[1] = 0.000000;ui30->dPresetData[2] = 0.000000;ui30->dPresetData[3] = 0.000000;ui30->dPresetData[4] = 0.000000;ui30->dPresetData[5] = 0.000000;ui30->dPresetData[6] = 0.000000;ui30->dPresetData[7] = 0.000000;ui30->dPresetData[8] = 0.000000;ui30->dPresetData[9] = 0.000000;ui30->dPresetData[10] = 0.000000;ui30->dPresetData[11] = 0.000000;ui30->dPresetData[12] = 0.000000;ui30->dPresetData[13] = 0.000000;ui30->dPresetData[14] = 0.000000;ui30->dPresetData[15] = 0.000000;
	ui30->cControlName = "Delay 3";
	ui30->bOwnerControl = false;
	ui30->bMIDIControl = false;
	ui30->uMIDIControlCommand = 176;
	ui30->uMIDIControlName = 3;
	ui30->uMIDIControlChannel = 0;
	ui30->nGUIRow = nIndexer++;
	ui30->nGUIColumn = -1;
	ui30->bEnableParamSmoothing = false;
	ui30->fSmoothingTimeInMs = 100.00;
	ui30->uControlTheme[0] = 0; ui30->uControlTheme[1] = 0; ui30->uControlTheme[2] = 0; ui30->uControlTheme[3] = 0; ui30->uControlTheme[4] = 0; ui30->uControlTheme[5] = 0; ui30->uControlTheme[6] = 0; ui30->uControlTheme[7] = 0; ui30->uControlTheme[8] = 10; ui30->uControlTheme[9] = 0; ui30->uControlTheme[10] = 0; ui30->uControlTheme[11] = 0; ui30->uControlTheme[12] = 0; ui30->uControlTheme[13] = 0; ui30->uControlTheme[14] = 0; ui30->uControlTheme[15] = 0; ui30->uControlTheme[16] = 2; ui30->uControlTheme[17] = 0; ui30->uControlTheme[18] = 0; ui30->uControlTheme[19] = 0; ui30->uControlTheme[20] = 0; ui30->uControlTheme[21] = 0; ui30->uControlTheme[22] = 0; ui30->uControlTheme[23] = 0; ui30->uControlTheme[24] = 0; ui30->uControlTheme[25] = 0; ui30->uControlTheme[26] = 0; ui30->uControlTheme[27] = 0; ui30->uControlTheme[28] = 0; ui30->uControlTheme[29] = 0; ui30->uControlTheme[30] = 0; ui30->uControlTheme[31] = 0; 
	ui30->uFluxCapControl[0] = 0; ui30->uFluxCapControl[1] = 0; ui30->uFluxCapControl[2] = 0; ui30->uFluxCapControl[3] = 0; ui30->uFluxCapControl[4] = 0; ui30->uFluxCapControl[5] = 0; ui30->uFluxCapControl[6] = 0; ui30->uFluxCapControl[7] = 0; ui30->uFluxCapControl[8] = 0; ui30->uFluxCapControl[9] = 0; ui30->uFluxCapControl[10] = 0; ui30->uFluxCapControl[11] = 0; ui30->uFluxCapControl[12] = 0; ui30->uFluxCapControl[13] = 0; ui30->uFluxCapControl[14] = 0; ui30->uFluxCapControl[15] = 0; ui30->uFluxCapControl[16] = 0; ui30->uFluxCapControl[17] = 0; ui30->uFluxCapControl[18] = 0; ui30->uFluxCapControl[19] = 0; ui30->uFluxCapControl[20] = 0; ui30->uFluxCapControl[21] = 0; ui30->uFluxCapControl[22] = 0; ui30->uFluxCapControl[23] = 0; ui30->uFluxCapControl[24] = 0; ui30->uFluxCapControl[25] = 0; ui30->uFluxCapControl[26] = 0; ui30->uFluxCapControl[27] = 0; ui30->uFluxCapControl[28] = 0; ui30->uFluxCapControl[29] = 0; ui30->uFluxCapControl[30] = 0; ui30->uFluxCapControl[31] = 0; ui30->uFluxCapControl[32] = 0; ui30->uFluxCapControl[33] = 0; ui30->uFluxCapControl[34] = 0; ui30->uFluxCapControl[35] = 0; ui30->uFluxCapControl[36] = 0; ui30->uFluxCapControl[37] = 0; ui30->uFluxCapControl[38] = 0; ui30->uFluxCapControl[39] = 0; ui30->uFluxCapControl[40] = 0; ui30->uFluxCapControl[41] = 0; ui30->uFluxCapControl[42] = 0; ui30->uFluxCapControl[43] = 0; ui30->uFluxCapControl[44] = 0; ui30->uFluxCapControl[45] = 0; ui30->uFluxCapControl[46] = 0; ui30->uFluxCapControl[47] = 0; ui30->uFluxCapControl[48] = 0; ui30->uFluxCapControl[49] = 0; ui30->uFluxCapControl[50] = 0; ui30->uFluxCapControl[51] = 0; ui30->uFluxCapControl[52] = 0; ui30->uFluxCapControl[53] = 0; ui30->uFluxCapControl[54] = 0; ui30->uFluxCapControl[55] = 0; ui30->uFluxCapControl[56] = 0; ui30->uFluxCapControl[57] = 0; ui30->uFluxCapControl[58] = 0; ui30->uFluxCapControl[59] = 0; ui30->uFluxCapControl[60] = 0; ui30->uFluxCapControl[61] = 0; ui30->uFluxCapControl[62] = 0; ui30->uFluxCapControl[63] = 0; 
	ui30->fFluxCapData[0] = 0.000000; ui30->fFluxCapData[1] = 0.000000; ui30->fFluxCapData[2] = 0.000000; ui30->fFluxCapData[3] = 0.000000; ui30->fFluxCapData[4] = 0.000000; ui30->fFluxCapData[5] = 0.000000; ui30->fFluxCapData[6] = 0.000000; ui30->fFluxCapData[7] = 0.000000; ui30->fFluxCapData[8] = 0.000000; ui30->fFluxCapData[9] = 0.000000; ui30->fFluxCapData[10] = 0.000000; ui30->fFluxCapData[11] = 0.000000; ui30->fFluxCapData[12] = 0.000000; ui30->fFluxCapData[13] = 0.000000; ui30->fFluxCapData[14] = 0.000000; ui30->fFluxCapData[15] = 0.000000; ui30->fFluxCapData[16] = 0.000000; ui30->fFluxCapData[17] = 0.000000; ui30->fFluxCapData[18] = 0.000000; ui30->fFluxCapData[19] = 0.000000; ui30->fFluxCapData[20] = 0.000000; ui30->fFluxCapData[21] = 0.000000; ui30->fFluxCapData[22] = 0.000000; ui30->fFluxCapData[23] = 0.000000; ui30->fFluxCapData[24] = 0.000000; ui30->fFluxCapData[25] = 0.000000; ui30->fFluxCapData[26] = 0.000000; ui30->fFluxCapData[27] = 0.000000; ui30->fFluxCapData[28] = 0.000000; ui30->fFluxCapData[29] = 0.000000; ui30->fFluxCapData[30] = 0.000000; ui30->fFluxCapData[31] = 0.000000; ui30->fFluxCapData[32] = 0.000000; ui30->fFluxCapData[33] = 0.000000; ui30->fFluxCapData[34] = 0.000000; ui30->fFluxCapData[35] = 0.000000; ui30->fFluxCapData[36] = 0.000000; ui30->fFluxCapData[37] = 0.000000; ui30->fFluxCapData[38] = 0.000000; ui30->fFluxCapData[39] = 0.000000; ui30->fFluxCapData[40] = 0.000000; ui30->fFluxCapData[41] = 0.000000; ui30->fFluxCapData[42] = 0.000000; ui30->fFluxCapData[43] = 0.000000; ui30->fFluxCapData[44] = 0.000000; ui30->fFluxCapData[45] = 0.000000; ui30->fFluxCapData[46] = 0.000000; ui30->fFluxCapData[47] = 0.000000; ui30->fFluxCapData[48] = 0.000000; ui30->fFluxCapData[49] = 0.000000; ui30->fFluxCapData[50] = 0.000000; ui30->fFluxCapData[51] = 0.000000; ui30->fFluxCapData[52] = 0.000000; ui30->fFluxCapData[53] = 0.000000; ui30->fFluxCapData[54] = 0.000000; ui30->fFluxCapData[55] = 0.000000; ui30->fFluxCapData[56] = 0.000000; ui30->fFluxCapData[57] = 0.000000; ui30->fFluxCapData[58] = 0.000000; ui30->fFluxCapData[59] = 0.000000; ui30->fFluxCapData[60] = 0.000000; ui30->fFluxCapData[61] = 0.000000; ui30->fFluxCapData[62] = 0.000000; ui30->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui30);
	delete ui30;


	m_fDelay4_mSec = 33.000000;
	CUICtrl* ui31 = new CUICtrl;
	ui31->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui31->uControlId = 114;
	ui31->bLogSlider = false;
	ui31->bExpSlider = false;
	ui31->fUserDisplayDataLoLimit = 0.000000;
	ui31->fUserDisplayDataHiLimit = 100.000000;
	ui31->uUserDataType = floatData;
	ui31->fInitUserIntValue = 0;
	ui31->fInitUserFloatValue = 33.000000;
	ui31->fInitUserDoubleValue = 0;
	ui31->fInitUserUINTValue = 0;
	ui31->m_pUserCookedIntData = NULL;
	ui31->m_pUserCookedFloatData = &m_fDelay4_mSec;
	ui31->m_pUserCookedDoubleData = NULL;
	ui31->m_pUserCookedUINTData = NULL;
	ui31->cControlUnits = "mSec";
	ui31->cVariableName = "m_fDelay4_mSec";
	ui31->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui31->dPresetData[0] = 33.000000;ui31->dPresetData[1] = 0.000000;ui31->dPresetData[2] = 0.000000;ui31->dPresetData[3] = 0.000000;ui31->dPresetData[4] = 0.000000;ui31->dPresetData[5] = 0.000000;ui31->dPresetData[6] = 0.000000;ui31->dPresetData[7] = 0.000000;ui31->dPresetData[8] = 0.000000;ui31->dPresetData[9] = 0.000000;ui31->dPresetData[10] = 0.000000;ui31->dPresetData[11] = 0.000000;ui31->dPresetData[12] = 0.000000;ui31->dPresetData[13] = 0.000000;ui31->dPresetData[14] = 0.000000;ui31->dPresetData[15] = 0.000000;
	ui31->cControlName = "Delay 4";
	ui31->bOwnerControl = false;
	ui31->bMIDIControl = false;
	ui31->uMIDIControlCommand = 176;
	ui31->uMIDIControlName = 3;
	ui31->uMIDIControlChannel = 0;
	ui31->nGUIRow = nIndexer++;
	ui31->nGUIColumn = -1;
	ui31->bEnableParamSmoothing = false;
	ui31->fSmoothingTimeInMs = 100.00;
	ui31->uControlTheme[0] = 0; ui31->uControlTheme[1] = 0; ui31->uControlTheme[2] = 0; ui31->uControlTheme[3] = 0; ui31->uControlTheme[4] = 0; ui31->uControlTheme[5] = 0; ui31->uControlTheme[6] = 0; ui31->uControlTheme[7] = 0; ui31->uControlTheme[8] = 10; ui31->uControlTheme[9] = 0; ui31->uControlTheme[10] = 0; ui31->uControlTheme[11] = 0; ui31->uControlTheme[12] = 0; ui31->uControlTheme[13] = 0; ui31->uControlTheme[14] = 0; ui31->uControlTheme[15] = 0; ui31->uControlTheme[16] = 2; ui31->uControlTheme[17] = 0; ui31->uControlTheme[18] = 0; ui31->uControlTheme[19] = 0; ui31->uControlTheme[20] = 0; ui31->uControlTheme[21] = 0; ui31->uControlTheme[22] = 0; ui31->uControlTheme[23] = 0; ui31->uControlTheme[24] = 0; ui31->uControlTheme[25] = 0; ui31->uControlTheme[26] = 0; ui31->uControlTheme[27] = 0; ui31->uControlTheme[28] = 0; ui31->uControlTheme[29] = 0; ui31->uControlTheme[30] = 0; ui31->uControlTheme[31] = 0; 
	ui31->uFluxCapControl[0] = 0; ui31->uFluxCapControl[1] = 0; ui31->uFluxCapControl[2] = 0; ui31->uFluxCapControl[3] = 0; ui31->uFluxCapControl[4] = 0; ui31->uFluxCapControl[5] = 0; ui31->uFluxCapControl[6] = 0; ui31->uFluxCapControl[7] = 0; ui31->uFluxCapControl[8] = 0; ui31->uFluxCapControl[9] = 0; ui31->uFluxCapControl[10] = 0; ui31->uFluxCapControl[11] = 0; ui31->uFluxCapControl[12] = 0; ui31->uFluxCapControl[13] = 0; ui31->uFluxCapControl[14] = 0; ui31->uFluxCapControl[15] = 0; ui31->uFluxCapControl[16] = 0; ui31->uFluxCapControl[17] = 0; ui31->uFluxCapControl[18] = 0; ui31->uFluxCapControl[19] = 0; ui31->uFluxCapControl[20] = 0; ui31->uFluxCapControl[21] = 0; ui31->uFluxCapControl[22] = 0; ui31->uFluxCapControl[23] = 0; ui31->uFluxCapControl[24] = 0; ui31->uFluxCapControl[25] = 0; ui31->uFluxCapControl[26] = 0; ui31->uFluxCapControl[27] = 0; ui31->uFluxCapControl[28] = 0; ui31->uFluxCapControl[29] = 0; ui31->uFluxCapControl[30] = 0; ui31->uFluxCapControl[31] = 0; ui31->uFluxCapControl[32] = 0; ui31->uFluxCapControl[33] = 0; ui31->uFluxCapControl[34] = 0; ui31->uFluxCapControl[35] = 0; ui31->uFluxCapControl[36] = 0; ui31->uFluxCapControl[37] = 0; ui31->uFluxCapControl[38] = 0; ui31->uFluxCapControl[39] = 0; ui31->uFluxCapControl[40] = 0; ui31->uFluxCapControl[41] = 0; ui31->uFluxCapControl[42] = 0; ui31->uFluxCapControl[43] = 0; ui31->uFluxCapControl[44] = 0; ui31->uFluxCapControl[45] = 0; ui31->uFluxCapControl[46] = 0; ui31->uFluxCapControl[47] = 0; ui31->uFluxCapControl[48] = 0; ui31->uFluxCapControl[49] = 0; ui31->uFluxCapControl[50] = 0; ui31->uFluxCapControl[51] = 0; ui31->uFluxCapControl[52] = 0; ui31->uFluxCapControl[53] = 0; ui31->uFluxCapControl[54] = 0; ui31->uFluxCapControl[55] = 0; ui31->uFluxCapControl[56] = 0; ui31->uFluxCapControl[57] = 0; ui31->uFluxCapControl[58] = 0; ui31->uFluxCapControl[59] = 0; ui31->uFluxCapControl[60] = 0; ui31->uFluxCapControl[61] = 0; ui31->uFluxCapControl[62] = 0; ui31->uFluxCapControl[63] = 0; 
	ui31->fFluxCapData[0] = 0.000000; ui31->fFluxCapData[1] = 0.000000; ui31->fFluxCapData[2] = 0.000000; ui31->fFluxCapData[3] = 0.000000; ui31->fFluxCapData[4] = 0.000000; ui31->fFluxCapData[5] = 0.000000; ui31->fFluxCapData[6] = 0.000000; ui31->fFluxCapData[7] = 0.000000; ui31->fFluxCapData[8] = 0.000000; ui31->fFluxCapData[9] = 0.000000; ui31->fFluxCapData[10] = 0.000000; ui31->fFluxCapData[11] = 0.000000; ui31->fFluxCapData[12] = 0.000000; ui31->fFluxCapData[13] = 0.000000; ui31->fFluxCapData[14] = 0.000000; ui31->fFluxCapData[15] = 0.000000; ui31->fFluxCapData[16] = 0.000000; ui31->fFluxCapData[17] = 0.000000; ui31->fFluxCapData[18] = 0.000000; ui31->fFluxCapData[19] = 0.000000; ui31->fFluxCapData[20] = 0.000000; ui31->fFluxCapData[21] = 0.000000; ui31->fFluxCapData[22] = 0.000000; ui31->fFluxCapData[23] = 0.000000; ui31->fFluxCapData[24] = 0.000000; ui31->fFluxCapData[25] = 0.000000; ui31->fFluxCapData[26] = 0.000000; ui31->fFluxCapData[27] = 0.000000; ui31->fFluxCapData[28] = 0.000000; ui31->fFluxCapData[29] = 0.000000; ui31->fFluxCapData[30] = 0.000000; ui31->fFluxCapData[31] = 0.000000; ui31->fFluxCapData[32] = 0.000000; ui31->fFluxCapData[33] = 0.000000; ui31->fFluxCapData[34] = 0.000000; ui31->fFluxCapData[35] = 0.000000; ui31->fFluxCapData[36] = 0.000000; ui31->fFluxCapData[37] = 0.000000; ui31->fFluxCapData[38] = 0.000000; ui31->fFluxCapData[39] = 0.000000; ui31->fFluxCapData[40] = 0.000000; ui31->fFluxCapData[41] = 0.000000; ui31->fFluxCapData[42] = 0.000000; ui31->fFluxCapData[43] = 0.000000; ui31->fFluxCapData[44] = 0.000000; ui31->fFluxCapData[45] = 0.000000; ui31->fFluxCapData[46] = 0.000000; ui31->fFluxCapData[47] = 0.000000; ui31->fFluxCapData[48] = 0.000000; ui31->fFluxCapData[49] = 0.000000; ui31->fFluxCapData[50] = 0.000000; ui31->fFluxCapData[51] = 0.000000; ui31->fFluxCapData[52] = 0.000000; ui31->fFluxCapData[53] = 0.000000; ui31->fFluxCapData[54] = 0.000000; ui31->fFluxCapData[55] = 0.000000; ui31->fFluxCapData[56] = 0.000000; ui31->fFluxCapData[57] = 0.000000; ui31->fFluxCapData[58] = 0.000000; ui31->fFluxCapData[59] = 0.000000; ui31->fFluxCapData[60] = 0.000000; ui31->fFluxCapData[61] = 0.000000; ui31->fFluxCapData[62] = 0.000000; ui31->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui31);
	delete ui31;


	m_fKrt_1 = 0.500000;
	CUICtrl* ui32 = new CUICtrl;
	ui32->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui32->uControlId = 116;
	ui32->bLogSlider = false;
	ui32->bExpSlider = false;
	ui32->fUserDisplayDataLoLimit = 0.010000;
	ui32->fUserDisplayDataHiLimit = 0.990000;
	ui32->uUserDataType = floatData;
	ui32->fInitUserIntValue = 0;
	ui32->fInitUserFloatValue = 0.500000;
	ui32->fInitUserDoubleValue = 0;
	ui32->fInitUserUINTValue = 0;
	ui32->m_pUserCookedIntData = NULL;
	ui32->m_pUserCookedFloatData = &m_fKrt_1;
	ui32->m_pUserCookedDoubleData = NULL;
	ui32->m_pUserCookedUINTData = NULL;
	ui32->cControlUnits = "Units";
	ui32->cVariableName = "m_fKrt_1";
	ui32->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui32->dPresetData[0] = 0.500000;ui32->dPresetData[1] = 0.000000;ui32->dPresetData[2] = 0.000000;ui32->dPresetData[3] = 0.000000;ui32->dPresetData[4] = 0.000000;ui32->dPresetData[5] = 0.000000;ui32->dPresetData[6] = 0.000000;ui32->dPresetData[7] = 0.000000;ui32->dPresetData[8] = 0.000000;ui32->dPresetData[9] = 0.000000;ui32->dPresetData[10] = 0.000000;ui32->dPresetData[11] = 0.000000;ui32->dPresetData[12] = 0.000000;ui32->dPresetData[13] = 0.000000;ui32->dPresetData[14] = 0.000000;ui32->dPresetData[15] = 0.000000;
	ui32->cControlName = "Krt";
	ui32->bOwnerControl = false;
	ui32->bMIDIControl = false;
	ui32->uMIDIControlCommand = 176;
	ui32->uMIDIControlName = 3;
	ui32->uMIDIControlChannel = 0;
	ui32->nGUIRow = nIndexer++;
	ui32->nGUIColumn = -1;
	ui32->bEnableParamSmoothing = false;
	ui32->fSmoothingTimeInMs = 100.00;
	ui32->uControlTheme[0] = 0; ui32->uControlTheme[1] = 0; ui32->uControlTheme[2] = 0; ui32->uControlTheme[3] = 0; ui32->uControlTheme[4] = 0; ui32->uControlTheme[5] = 0; ui32->uControlTheme[6] = 0; ui32->uControlTheme[7] = 0; ui32->uControlTheme[8] = 14; ui32->uControlTheme[9] = 0; ui32->uControlTheme[10] = 0; ui32->uControlTheme[11] = 0; ui32->uControlTheme[12] = 0; ui32->uControlTheme[13] = 0; ui32->uControlTheme[14] = 0; ui32->uControlTheme[15] = 0; ui32->uControlTheme[16] = 2; ui32->uControlTheme[17] = 0; ui32->uControlTheme[18] = 0; ui32->uControlTheme[19] = 0; ui32->uControlTheme[20] = 0; ui32->uControlTheme[21] = 0; ui32->uControlTheme[22] = 0; ui32->uControlTheme[23] = 0; ui32->uControlTheme[24] = 0; ui32->uControlTheme[25] = 0; ui32->uControlTheme[26] = 0; ui32->uControlTheme[27] = 0; ui32->uControlTheme[28] = 0; ui32->uControlTheme[29] = 0; ui32->uControlTheme[30] = 0; ui32->uControlTheme[31] = 0; 
	ui32->uFluxCapControl[0] = 0; ui32->uFluxCapControl[1] = 0; ui32->uFluxCapControl[2] = 0; ui32->uFluxCapControl[3] = 0; ui32->uFluxCapControl[4] = 0; ui32->uFluxCapControl[5] = 0; ui32->uFluxCapControl[6] = 0; ui32->uFluxCapControl[7] = 0; ui32->uFluxCapControl[8] = 0; ui32->uFluxCapControl[9] = 0; ui32->uFluxCapControl[10] = 0; ui32->uFluxCapControl[11] = 0; ui32->uFluxCapControl[12] = 0; ui32->uFluxCapControl[13] = 0; ui32->uFluxCapControl[14] = 0; ui32->uFluxCapControl[15] = 0; ui32->uFluxCapControl[16] = 0; ui32->uFluxCapControl[17] = 0; ui32->uFluxCapControl[18] = 0; ui32->uFluxCapControl[19] = 0; ui32->uFluxCapControl[20] = 0; ui32->uFluxCapControl[21] = 0; ui32->uFluxCapControl[22] = 0; ui32->uFluxCapControl[23] = 0; ui32->uFluxCapControl[24] = 0; ui32->uFluxCapControl[25] = 0; ui32->uFluxCapControl[26] = 0; ui32->uFluxCapControl[27] = 0; ui32->uFluxCapControl[28] = 0; ui32->uFluxCapControl[29] = 0; ui32->uFluxCapControl[30] = 0; ui32->uFluxCapControl[31] = 0; ui32->uFluxCapControl[32] = 0; ui32->uFluxCapControl[33] = 0; ui32->uFluxCapControl[34] = 0; ui32->uFluxCapControl[35] = 0; ui32->uFluxCapControl[36] = 0; ui32->uFluxCapControl[37] = 0; ui32->uFluxCapControl[38] = 0; ui32->uFluxCapControl[39] = 0; ui32->uFluxCapControl[40] = 0; ui32->uFluxCapControl[41] = 0; ui32->uFluxCapControl[42] = 0; ui32->uFluxCapControl[43] = 0; ui32->uFluxCapControl[44] = 0; ui32->uFluxCapControl[45] = 0; ui32->uFluxCapControl[46] = 0; ui32->uFluxCapControl[47] = 0; ui32->uFluxCapControl[48] = 0; ui32->uFluxCapControl[49] = 0; ui32->uFluxCapControl[50] = 0; ui32->uFluxCapControl[51] = 0; ui32->uFluxCapControl[52] = 0; ui32->uFluxCapControl[53] = 0; ui32->uFluxCapControl[54] = 0; ui32->uFluxCapControl[55] = 0; ui32->uFluxCapControl[56] = 0; ui32->uFluxCapControl[57] = 0; ui32->uFluxCapControl[58] = 0; ui32->uFluxCapControl[59] = 0; ui32->uFluxCapControl[60] = 0; ui32->uFluxCapControl[61] = 0; ui32->uFluxCapControl[62] = 0; ui32->uFluxCapControl[63] = 0; 
	ui32->fFluxCapData[0] = 0.000000; ui32->fFluxCapData[1] = 0.000000; ui32->fFluxCapData[2] = 0.000000; ui32->fFluxCapData[3] = 0.000000; ui32->fFluxCapData[4] = 0.000000; ui32->fFluxCapData[5] = 0.000000; ui32->fFluxCapData[6] = 0.000000; ui32->fFluxCapData[7] = 0.000000; ui32->fFluxCapData[8] = 0.000000; ui32->fFluxCapData[9] = 0.000000; ui32->fFluxCapData[10] = 0.000000; ui32->fFluxCapData[11] = 0.000000; ui32->fFluxCapData[12] = 0.000000; ui32->fFluxCapData[13] = 0.000000; ui32->fFluxCapData[14] = 0.000000; ui32->fFluxCapData[15] = 0.000000; ui32->fFluxCapData[16] = 0.000000; ui32->fFluxCapData[17] = 0.000000; ui32->fFluxCapData[18] = 0.000000; ui32->fFluxCapData[19] = 0.000000; ui32->fFluxCapData[20] = 0.000000; ui32->fFluxCapData[21] = 0.000000; ui32->fFluxCapData[22] = 0.000000; ui32->fFluxCapData[23] = 0.000000; ui32->fFluxCapData[24] = 0.000000; ui32->fFluxCapData[25] = 0.000000; ui32->fFluxCapData[26] = 0.000000; ui32->fFluxCapData[27] = 0.000000; ui32->fFluxCapData[28] = 0.000000; ui32->fFluxCapData[29] = 0.000000; ui32->fFluxCapData[30] = 0.000000; ui32->fFluxCapData[31] = 0.000000; ui32->fFluxCapData[32] = 0.000000; ui32->fFluxCapData[33] = 0.000000; ui32->fFluxCapData[34] = 0.000000; ui32->fFluxCapData[35] = 0.000000; ui32->fFluxCapData[36] = 0.000000; ui32->fFluxCapData[37] = 0.000000; ui32->fFluxCapData[38] = 0.000000; ui32->fFluxCapData[39] = 0.000000; ui32->fFluxCapData[40] = 0.000000; ui32->fFluxCapData[41] = 0.000000; ui32->fFluxCapData[42] = 0.000000; ui32->fFluxCapData[43] = 0.000000; ui32->fFluxCapData[44] = 0.000000; ui32->fFluxCapData[45] = 0.000000; ui32->fFluxCapData[46] = 0.000000; ui32->fFluxCapData[47] = 0.000000; ui32->fFluxCapData[48] = 0.000000; ui32->fFluxCapData[49] = 0.000000; ui32->fFluxCapData[50] = 0.000000; ui32->fFluxCapData[51] = 0.000000; ui32->fFluxCapData[52] = 0.000000; ui32->fFluxCapData[53] = 0.000000; ui32->fFluxCapData[54] = 0.000000; ui32->fFluxCapData[55] = 0.000000; ui32->fFluxCapData[56] = 0.000000; ui32->fFluxCapData[57] = 0.000000; ui32->fFluxCapData[58] = 0.000000; ui32->fFluxCapData[59] = 0.000000; ui32->fFluxCapData[60] = 0.000000; ui32->fFluxCapData[61] = 0.000000; ui32->fFluxCapData[62] = 0.000000; ui32->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui32);
	delete ui32;


	m_fLPF_g = 0.500000;
	CUICtrl* ui33 = new CUICtrl;
	ui33->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui33->uControlId = 117;
	ui33->bLogSlider = false;
	ui33->bExpSlider = false;
	ui33->fUserDisplayDataLoLimit = 0.000000;
	ui33->fUserDisplayDataHiLimit = 1.000000;
	ui33->uUserDataType = floatData;
	ui33->fInitUserIntValue = 0;
	ui33->fInitUserFloatValue = 0.500000;
	ui33->fInitUserDoubleValue = 0;
	ui33->fInitUserUINTValue = 0;
	ui33->m_pUserCookedIntData = NULL;
	ui33->m_pUserCookedFloatData = &m_fLPF_g;
	ui33->m_pUserCookedDoubleData = NULL;
	ui33->m_pUserCookedUINTData = NULL;
	ui33->cControlUnits = "";
	ui33->cVariableName = "m_fLPF_g";
	ui33->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui33->dPresetData[0] = 0.500000;ui33->dPresetData[1] = 0.000000;ui33->dPresetData[2] = 0.000000;ui33->dPresetData[3] = 0.000000;ui33->dPresetData[4] = 0.000000;ui33->dPresetData[5] = 0.000000;ui33->dPresetData[6] = 0.000000;ui33->dPresetData[7] = 0.000000;ui33->dPresetData[8] = 0.000000;ui33->dPresetData[9] = 0.000000;ui33->dPresetData[10] = 0.000000;ui33->dPresetData[11] = 0.000000;ui33->dPresetData[12] = 0.000000;ui33->dPresetData[13] = 0.000000;ui33->dPresetData[14] = 0.000000;ui33->dPresetData[15] = 0.000000;
	ui33->cControlName = "Damping";
	ui33->bOwnerControl = false;
	ui33->bMIDIControl = false;
	ui33->uMIDIControlCommand = 176;
	ui33->uMIDIControlName = 3;
	ui33->uMIDIControlChannel = 0;
	ui33->nGUIRow = nIndexer++;
	ui33->nGUIColumn = -1;
	ui33->bEnableParamSmoothing = false;
	ui33->fSmoothingTimeInMs = 100.00;
	ui33->uControlTheme[0] = 0; ui33->uControlTheme[1] = 0; ui33->uControlTheme[2] = 0; ui33->uControlTheme[3] = 0; ui33->uControlTheme[4] = 0; ui33->uControlTheme[5] = 0; ui33->uControlTheme[6] = 0; ui33->uControlTheme[7] = 0; ui33->uControlTheme[8] = 14; ui33->uControlTheme[9] = 0; ui33->uControlTheme[10] = 0; ui33->uControlTheme[11] = 0; ui33->uControlTheme[12] = 0; ui33->uControlTheme[13] = 0; ui33->uControlTheme[14] = 0; ui33->uControlTheme[15] = 0; ui33->uControlTheme[16] = 2; ui33->uControlTheme[17] = 0; ui33->uControlTheme[18] = 0; ui33->uControlTheme[19] = 0; ui33->uControlTheme[20] = 0; ui33->uControlTheme[21] = 0; ui33->uControlTheme[22] = 0; ui33->uControlTheme[23] = 0; ui33->uControlTheme[24] = 0; ui33->uControlTheme[25] = 0; ui33->uControlTheme[26] = 0; ui33->uControlTheme[27] = 0; ui33->uControlTheme[28] = 0; ui33->uControlTheme[29] = 0; ui33->uControlTheme[30] = 0; ui33->uControlTheme[31] = 0; 
	ui33->uFluxCapControl[0] = 0; ui33->uFluxCapControl[1] = 0; ui33->uFluxCapControl[2] = 0; ui33->uFluxCapControl[3] = 0; ui33->uFluxCapControl[4] = 0; ui33->uFluxCapControl[5] = 0; ui33->uFluxCapControl[6] = 0; ui33->uFluxCapControl[7] = 0; ui33->uFluxCapControl[8] = 0; ui33->uFluxCapControl[9] = 0; ui33->uFluxCapControl[10] = 0; ui33->uFluxCapControl[11] = 0; ui33->uFluxCapControl[12] = 0; ui33->uFluxCapControl[13] = 0; ui33->uFluxCapControl[14] = 0; ui33->uFluxCapControl[15] = 0; ui33->uFluxCapControl[16] = 0; ui33->uFluxCapControl[17] = 0; ui33->uFluxCapControl[18] = 0; ui33->uFluxCapControl[19] = 0; ui33->uFluxCapControl[20] = 0; ui33->uFluxCapControl[21] = 0; ui33->uFluxCapControl[22] = 0; ui33->uFluxCapControl[23] = 0; ui33->uFluxCapControl[24] = 0; ui33->uFluxCapControl[25] = 0; ui33->uFluxCapControl[26] = 0; ui33->uFluxCapControl[27] = 0; ui33->uFluxCapControl[28] = 0; ui33->uFluxCapControl[29] = 0; ui33->uFluxCapControl[30] = 0; ui33->uFluxCapControl[31] = 0; ui33->uFluxCapControl[32] = 0; ui33->uFluxCapControl[33] = 0; ui33->uFluxCapControl[34] = 0; ui33->uFluxCapControl[35] = 0; ui33->uFluxCapControl[36] = 0; ui33->uFluxCapControl[37] = 0; ui33->uFluxCapControl[38] = 0; ui33->uFluxCapControl[39] = 0; ui33->uFluxCapControl[40] = 0; ui33->uFluxCapControl[41] = 0; ui33->uFluxCapControl[42] = 0; ui33->uFluxCapControl[43] = 0; ui33->uFluxCapControl[44] = 0; ui33->uFluxCapControl[45] = 0; ui33->uFluxCapControl[46] = 0; ui33->uFluxCapControl[47] = 0; ui33->uFluxCapControl[48] = 0; ui33->uFluxCapControl[49] = 0; ui33->uFluxCapControl[50] = 0; ui33->uFluxCapControl[51] = 0; ui33->uFluxCapControl[52] = 0; ui33->uFluxCapControl[53] = 0; ui33->uFluxCapControl[54] = 0; ui33->uFluxCapControl[55] = 0; ui33->uFluxCapControl[56] = 0; ui33->uFluxCapControl[57] = 0; ui33->uFluxCapControl[58] = 0; ui33->uFluxCapControl[59] = 0; ui33->uFluxCapControl[60] = 0; ui33->uFluxCapControl[61] = 0; ui33->uFluxCapControl[62] = 0; ui33->uFluxCapControl[63] = 0; 
	ui33->fFluxCapData[0] = 0.000000; ui33->fFluxCapData[1] = 0.000000; ui33->fFluxCapData[2] = 0.000000; ui33->fFluxCapData[3] = 0.000000; ui33->fFluxCapData[4] = 0.000000; ui33->fFluxCapData[5] = 0.000000; ui33->fFluxCapData[6] = 0.000000; ui33->fFluxCapData[7] = 0.000000; ui33->fFluxCapData[8] = 0.000000; ui33->fFluxCapData[9] = 0.000000; ui33->fFluxCapData[10] = 0.000000; ui33->fFluxCapData[11] = 0.000000; ui33->fFluxCapData[12] = 0.000000; ui33->fFluxCapData[13] = 0.000000; ui33->fFluxCapData[14] = 0.000000; ui33->fFluxCapData[15] = 0.000000; ui33->fFluxCapData[16] = 0.000000; ui33->fFluxCapData[17] = 0.000000; ui33->fFluxCapData[18] = 0.000000; ui33->fFluxCapData[19] = 0.000000; ui33->fFluxCapData[20] = 0.000000; ui33->fFluxCapData[21] = 0.000000; ui33->fFluxCapData[22] = 0.000000; ui33->fFluxCapData[23] = 0.000000; ui33->fFluxCapData[24] = 0.000000; ui33->fFluxCapData[25] = 0.000000; ui33->fFluxCapData[26] = 0.000000; ui33->fFluxCapData[27] = 0.000000; ui33->fFluxCapData[28] = 0.000000; ui33->fFluxCapData[29] = 0.000000; ui33->fFluxCapData[30] = 0.000000; ui33->fFluxCapData[31] = 0.000000; ui33->fFluxCapData[32] = 0.000000; ui33->fFluxCapData[33] = 0.000000; ui33->fFluxCapData[34] = 0.000000; ui33->fFluxCapData[35] = 0.000000; ui33->fFluxCapData[36] = 0.000000; ui33->fFluxCapData[37] = 0.000000; ui33->fFluxCapData[38] = 0.000000; ui33->fFluxCapData[39] = 0.000000; ui33->fFluxCapData[40] = 0.000000; ui33->fFluxCapData[41] = 0.000000; ui33->fFluxCapData[42] = 0.000000; ui33->fFluxCapData[43] = 0.000000; ui33->fFluxCapData[44] = 0.000000; ui33->fFluxCapData[45] = 0.000000; ui33->fFluxCapData[46] = 0.000000; ui33->fFluxCapData[47] = 0.000000; ui33->fFluxCapData[48] = 0.000000; ui33->fFluxCapData[49] = 0.000000; ui33->fFluxCapData[50] = 0.000000; ui33->fFluxCapData[51] = 0.000000; ui33->fFluxCapData[52] = 0.000000; ui33->fFluxCapData[53] = 0.000000; ui33->fFluxCapData[54] = 0.000000; ui33->fFluxCapData[55] = 0.000000; ui33->fFluxCapData[56] = 0.000000; ui33->fFluxCapData[57] = 0.000000; ui33->fFluxCapData[58] = 0.000000; ui33->fFluxCapData[59] = 0.000000; ui33->fFluxCapData[60] = 0.000000; ui33->fFluxCapData[61] = 0.000000; ui33->fFluxCapData[62] = 0.000000; ui33->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui33);
	delete ui33;


	m_fModRate = 5.000000;
	CUICtrl* ui34 = new CUICtrl;
	ui34->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui34->uControlId = 130;
	ui34->bLogSlider = false;
	ui34->bExpSlider = false;
	ui34->fUserDisplayDataLoLimit = 0.000000;
	ui34->fUserDisplayDataHiLimit = 25.000000;
	ui34->uUserDataType = floatData;
	ui34->fInitUserIntValue = 0;
	ui34->fInitUserFloatValue = 5.000000;
	ui34->fInitUserDoubleValue = 0;
	ui34->fInitUserUINTValue = 0;
	ui34->m_pUserCookedIntData = NULL;
	ui34->m_pUserCookedFloatData = &m_fModRate;
	ui34->m_pUserCookedDoubleData = NULL;
	ui34->m_pUserCookedUINTData = NULL;
	ui34->cControlUnits = "Hz";
	ui34->cVariableName = "m_fModRate";
	ui34->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui34->dPresetData[0] = 5.000000;ui34->dPresetData[1] = 0.000000;ui34->dPresetData[2] = 0.000000;ui34->dPresetData[3] = 0.000000;ui34->dPresetData[4] = 0.000000;ui34->dPresetData[5] = 0.000000;ui34->dPresetData[6] = 0.000000;ui34->dPresetData[7] = 0.000000;ui34->dPresetData[8] = 0.000000;ui34->dPresetData[9] = 0.000000;ui34->dPresetData[10] = 0.000000;ui34->dPresetData[11] = 0.000000;ui34->dPresetData[12] = 0.000000;ui34->dPresetData[13] = 0.000000;ui34->dPresetData[14] = 0.000000;ui34->dPresetData[15] = 0.000000;
	ui34->cControlName = "Tremolo Rate";
	ui34->bOwnerControl = false;
	ui34->bMIDIControl = false;
	ui34->uMIDIControlCommand = 176;
	ui34->uMIDIControlName = 3;
	ui34->uMIDIControlChannel = 0;
	ui34->nGUIRow = nIndexer++;
	ui34->nGUIColumn = -1;
	ui34->bEnableParamSmoothing = false;
	ui34->fSmoothingTimeInMs = 100.00;
	ui34->uControlTheme[0] = 0; ui34->uControlTheme[1] = 0; ui34->uControlTheme[2] = 0; ui34->uControlTheme[3] = 0; ui34->uControlTheme[4] = 0; ui34->uControlTheme[5] = 0; ui34->uControlTheme[6] = 0; ui34->uControlTheme[7] = 0; ui34->uControlTheme[8] = 12; ui34->uControlTheme[9] = 0; ui34->uControlTheme[10] = 0; ui34->uControlTheme[11] = 0; ui34->uControlTheme[12] = 0; ui34->uControlTheme[13] = 0; ui34->uControlTheme[14] = 0; ui34->uControlTheme[15] = 0; ui34->uControlTheme[16] = 2; ui34->uControlTheme[17] = 0; ui34->uControlTheme[18] = 0; ui34->uControlTheme[19] = 0; ui34->uControlTheme[20] = 0; ui34->uControlTheme[21] = 0; ui34->uControlTheme[22] = 0; ui34->uControlTheme[23] = 0; ui34->uControlTheme[24] = 0; ui34->uControlTheme[25] = 0; ui34->uControlTheme[26] = 0; ui34->uControlTheme[27] = 0; ui34->uControlTheme[28] = 0; ui34->uControlTheme[29] = 0; ui34->uControlTheme[30] = 0; ui34->uControlTheme[31] = 0; 
	ui34->uFluxCapControl[0] = 0; ui34->uFluxCapControl[1] = 0; ui34->uFluxCapControl[2] = 0; ui34->uFluxCapControl[3] = 0; ui34->uFluxCapControl[4] = 0; ui34->uFluxCapControl[5] = 0; ui34->uFluxCapControl[6] = 0; ui34->uFluxCapControl[7] = 0; ui34->uFluxCapControl[8] = 0; ui34->uFluxCapControl[9] = 0; ui34->uFluxCapControl[10] = 0; ui34->uFluxCapControl[11] = 0; ui34->uFluxCapControl[12] = 0; ui34->uFluxCapControl[13] = 0; ui34->uFluxCapControl[14] = 0; ui34->uFluxCapControl[15] = 0; ui34->uFluxCapControl[16] = 0; ui34->uFluxCapControl[17] = 0; ui34->uFluxCapControl[18] = 0; ui34->uFluxCapControl[19] = 0; ui34->uFluxCapControl[20] = 0; ui34->uFluxCapControl[21] = 0; ui34->uFluxCapControl[22] = 0; ui34->uFluxCapControl[23] = 0; ui34->uFluxCapControl[24] = 0; ui34->uFluxCapControl[25] = 0; ui34->uFluxCapControl[26] = 0; ui34->uFluxCapControl[27] = 0; ui34->uFluxCapControl[28] = 0; ui34->uFluxCapControl[29] = 0; ui34->uFluxCapControl[30] = 0; ui34->uFluxCapControl[31] = 0; ui34->uFluxCapControl[32] = 0; ui34->uFluxCapControl[33] = 0; ui34->uFluxCapControl[34] = 0; ui34->uFluxCapControl[35] = 0; ui34->uFluxCapControl[36] = 0; ui34->uFluxCapControl[37] = 0; ui34->uFluxCapControl[38] = 0; ui34->uFluxCapControl[39] = 0; ui34->uFluxCapControl[40] = 0; ui34->uFluxCapControl[41] = 0; ui34->uFluxCapControl[42] = 0; ui34->uFluxCapControl[43] = 0; ui34->uFluxCapControl[44] = 0; ui34->uFluxCapControl[45] = 0; ui34->uFluxCapControl[46] = 0; ui34->uFluxCapControl[47] = 0; ui34->uFluxCapControl[48] = 0; ui34->uFluxCapControl[49] = 0; ui34->uFluxCapControl[50] = 0; ui34->uFluxCapControl[51] = 0; ui34->uFluxCapControl[52] = 0; ui34->uFluxCapControl[53] = 0; ui34->uFluxCapControl[54] = 0; ui34->uFluxCapControl[55] = 0; ui34->uFluxCapControl[56] = 0; ui34->uFluxCapControl[57] = 0; ui34->uFluxCapControl[58] = 0; ui34->uFluxCapControl[59] = 0; ui34->uFluxCapControl[60] = 0; ui34->uFluxCapControl[61] = 0; ui34->uFluxCapControl[62] = 0; ui34->uFluxCapControl[63] = 0; 
	ui34->fFluxCapData[0] = 0.000000; ui34->fFluxCapData[1] = 0.000000; ui34->fFluxCapData[2] = 0.000000; ui34->fFluxCapData[3] = 0.000000; ui34->fFluxCapData[4] = 0.000000; ui34->fFluxCapData[5] = 0.000000; ui34->fFluxCapData[6] = 0.000000; ui34->fFluxCapData[7] = 0.000000; ui34->fFluxCapData[8] = 0.000000; ui34->fFluxCapData[9] = 0.000000; ui34->fFluxCapData[10] = 0.000000; ui34->fFluxCapData[11] = 0.000000; ui34->fFluxCapData[12] = 0.000000; ui34->fFluxCapData[13] = 0.000000; ui34->fFluxCapData[14] = 0.000000; ui34->fFluxCapData[15] = 0.000000; ui34->fFluxCapData[16] = 0.000000; ui34->fFluxCapData[17] = 0.000000; ui34->fFluxCapData[18] = 0.000000; ui34->fFluxCapData[19] = 0.000000; ui34->fFluxCapData[20] = 0.000000; ui34->fFluxCapData[21] = 0.000000; ui34->fFluxCapData[22] = 0.000000; ui34->fFluxCapData[23] = 0.000000; ui34->fFluxCapData[24] = 0.000000; ui34->fFluxCapData[25] = 0.000000; ui34->fFluxCapData[26] = 0.000000; ui34->fFluxCapData[27] = 0.000000; ui34->fFluxCapData[28] = 0.000000; ui34->fFluxCapData[29] = 0.000000; ui34->fFluxCapData[30] = 0.000000; ui34->fFluxCapData[31] = 0.000000; ui34->fFluxCapData[32] = 0.000000; ui34->fFluxCapData[33] = 0.000000; ui34->fFluxCapData[34] = 0.000000; ui34->fFluxCapData[35] = 0.000000; ui34->fFluxCapData[36] = 0.000000; ui34->fFluxCapData[37] = 0.000000; ui34->fFluxCapData[38] = 0.000000; ui34->fFluxCapData[39] = 0.000000; ui34->fFluxCapData[40] = 0.000000; ui34->fFluxCapData[41] = 0.000000; ui34->fFluxCapData[42] = 0.000000; ui34->fFluxCapData[43] = 0.000000; ui34->fFluxCapData[44] = 0.000000; ui34->fFluxCapData[45] = 0.000000; ui34->fFluxCapData[46] = 0.000000; ui34->fFluxCapData[47] = 0.000000; ui34->fFluxCapData[48] = 0.000000; ui34->fFluxCapData[49] = 0.000000; ui34->fFluxCapData[50] = 0.000000; ui34->fFluxCapData[51] = 0.000000; ui34->fFluxCapData[52] = 0.000000; ui34->fFluxCapData[53] = 0.000000; ui34->fFluxCapData[54] = 0.000000; ui34->fFluxCapData[55] = 0.000000; ui34->fFluxCapData[56] = 0.000000; ui34->fFluxCapData[57] = 0.000000; ui34->fFluxCapData[58] = 0.000000; ui34->fFluxCapData[59] = 0.000000; ui34->fFluxCapData[60] = 0.000000; ui34->fFluxCapData[61] = 0.000000; ui34->fFluxCapData[62] = 0.000000; ui34->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui34);
	delete ui34;


	m_fModDepth = 50.000000;
	CUICtrl* ui35 = new CUICtrl;
	ui35->uControlType = FILTER_CONTROL_CONTINUOUSLY_VARIABLE;
	ui35->uControlId = 131;
	ui35->bLogSlider = false;
	ui35->bExpSlider = false;
	ui35->fUserDisplayDataLoLimit = 0.000000;
	ui35->fUserDisplayDataHiLimit = 100.000000;
	ui35->uUserDataType = floatData;
	ui35->fInitUserIntValue = 0;
	ui35->fInitUserFloatValue = 50.000000;
	ui35->fInitUserDoubleValue = 0;
	ui35->fInitUserUINTValue = 0;
	ui35->m_pUserCookedIntData = NULL;
	ui35->m_pUserCookedFloatData = &m_fModDepth;
	ui35->m_pUserCookedDoubleData = NULL;
	ui35->m_pUserCookedUINTData = NULL;
	ui35->cControlUnits = "%";
	ui35->cVariableName = "m_fModDepth";
	ui35->cEnumeratedList = "SEL1,SEL2,SEL3";
	ui35->dPresetData[0] = 50.000000;ui35->dPresetData[1] = 0.000000;ui35->dPresetData[2] = 0.000000;ui35->dPresetData[3] = 0.000000;ui35->dPresetData[4] = 0.000000;ui35->dPresetData[5] = 0.000000;ui35->dPresetData[6] = 0.000000;ui35->dPresetData[7] = 0.000000;ui35->dPresetData[8] = 0.000000;ui35->dPresetData[9] = 0.000000;ui35->dPresetData[10] = 0.000000;ui35->dPresetData[11] = 0.000000;ui35->dPresetData[12] = 0.000000;ui35->dPresetData[13] = 0.000000;ui35->dPresetData[14] = 0.000000;ui35->dPresetData[15] = 0.000000;
	ui35->cControlName = "Tremolo Depth";
	ui35->bOwnerControl = false;
	ui35->bMIDIControl = false;
	ui35->uMIDIControlCommand = 176;
	ui35->uMIDIControlName = 3;
	ui35->uMIDIControlChannel = 0;
	ui35->nGUIRow = nIndexer++;
	ui35->nGUIColumn = -1;
	ui35->bEnableParamSmoothing = false;
	ui35->fSmoothingTimeInMs = 100.00;
	ui35->uControlTheme[0] = 0; ui35->uControlTheme[1] = 0; ui35->uControlTheme[2] = 0; ui35->uControlTheme[3] = 0; ui35->uControlTheme[4] = 0; ui35->uControlTheme[5] = 0; ui35->uControlTheme[6] = 0; ui35->uControlTheme[7] = 0; ui35->uControlTheme[8] = 12; ui35->uControlTheme[9] = 0; ui35->uControlTheme[10] = 0; ui35->uControlTheme[11] = 0; ui35->uControlTheme[12] = 0; ui35->uControlTheme[13] = 0; ui35->uControlTheme[14] = 0; ui35->uControlTheme[15] = 0; ui35->uControlTheme[16] = 2; ui35->uControlTheme[17] = 0; ui35->uControlTheme[18] = 0; ui35->uControlTheme[19] = 0; ui35->uControlTheme[20] = 0; ui35->uControlTheme[21] = 0; ui35->uControlTheme[22] = 0; ui35->uControlTheme[23] = 0; ui35->uControlTheme[24] = 0; ui35->uControlTheme[25] = 0; ui35->uControlTheme[26] = 0; ui35->uControlTheme[27] = 0; ui35->uControlTheme[28] = 0; ui35->uControlTheme[29] = 0; ui35->uControlTheme[30] = 0; ui35->uControlTheme[31] = 0; 
	ui35->uFluxCapControl[0] = 0; ui35->uFluxCapControl[1] = 0; ui35->uFluxCapControl[2] = 0; ui35->uFluxCapControl[3] = 0; ui35->uFluxCapControl[4] = 0; ui35->uFluxCapControl[5] = 0; ui35->uFluxCapControl[6] = 0; ui35->uFluxCapControl[7] = 0; ui35->uFluxCapControl[8] = 0; ui35->uFluxCapControl[9] = 0; ui35->uFluxCapControl[10] = 0; ui35->uFluxCapControl[11] = 0; ui35->uFluxCapControl[12] = 0; ui35->uFluxCapControl[13] = 0; ui35->uFluxCapControl[14] = 0; ui35->uFluxCapControl[15] = 0; ui35->uFluxCapControl[16] = 0; ui35->uFluxCapControl[17] = 0; ui35->uFluxCapControl[18] = 0; ui35->uFluxCapControl[19] = 0; ui35->uFluxCapControl[20] = 0; ui35->uFluxCapControl[21] = 0; ui35->uFluxCapControl[22] = 0; ui35->uFluxCapControl[23] = 0; ui35->uFluxCapControl[24] = 0; ui35->uFluxCapControl[25] = 0; ui35->uFluxCapControl[26] = 0; ui35->uFluxCapControl[27] = 0; ui35->uFluxCapControl[28] = 0; ui35->uFluxCapControl[29] = 0; ui35->uFluxCapControl[30] = 0; ui35->uFluxCapControl[31] = 0; ui35->uFluxCapControl[32] = 0; ui35->uFluxCapControl[33] = 0; ui35->uFluxCapControl[34] = 0; ui35->uFluxCapControl[35] = 0; ui35->uFluxCapControl[36] = 0; ui35->uFluxCapControl[37] = 0; ui35->uFluxCapControl[38] = 0; ui35->uFluxCapControl[39] = 0; ui35->uFluxCapControl[40] = 0; ui35->uFluxCapControl[41] = 0; ui35->uFluxCapControl[42] = 0; ui35->uFluxCapControl[43] = 0; ui35->uFluxCapControl[44] = 0; ui35->uFluxCapControl[45] = 0; ui35->uFluxCapControl[46] = 0; ui35->uFluxCapControl[47] = 0; ui35->uFluxCapControl[48] = 0; ui35->uFluxCapControl[49] = 0; ui35->uFluxCapControl[50] = 0; ui35->uFluxCapControl[51] = 0; ui35->uFluxCapControl[52] = 0; ui35->uFluxCapControl[53] = 0; ui35->uFluxCapControl[54] = 0; ui35->uFluxCapControl[55] = 0; ui35->uFluxCapControl[56] = 0; ui35->uFluxCapControl[57] = 0; ui35->uFluxCapControl[58] = 0; ui35->uFluxCapControl[59] = 0; ui35->uFluxCapControl[60] = 0; ui35->uFluxCapControl[61] = 0; ui35->uFluxCapControl[62] = 0; ui35->uFluxCapControl[63] = 0; 
	ui35->fFluxCapData[0] = 0.000000; ui35->fFluxCapData[1] = 0.000000; ui35->fFluxCapData[2] = 0.000000; ui35->fFluxCapData[3] = 0.000000; ui35->fFluxCapData[4] = 0.000000; ui35->fFluxCapData[5] = 0.000000; ui35->fFluxCapData[6] = 0.000000; ui35->fFluxCapData[7] = 0.000000; ui35->fFluxCapData[8] = 0.000000; ui35->fFluxCapData[9] = 0.000000; ui35->fFluxCapData[10] = 0.000000; ui35->fFluxCapData[11] = 0.000000; ui35->fFluxCapData[12] = 0.000000; ui35->fFluxCapData[13] = 0.000000; ui35->fFluxCapData[14] = 0.000000; ui35->fFluxCapData[15] = 0.000000; ui35->fFluxCapData[16] = 0.000000; ui35->fFluxCapData[17] = 0.000000; ui35->fFluxCapData[18] = 0.000000; ui35->fFluxCapData[19] = 0.000000; ui35->fFluxCapData[20] = 0.000000; ui35->fFluxCapData[21] = 0.000000; ui35->fFluxCapData[22] = 0.000000; ui35->fFluxCapData[23] = 0.000000; ui35->fFluxCapData[24] = 0.000000; ui35->fFluxCapData[25] = 0.000000; ui35->fFluxCapData[26] = 0.000000; ui35->fFluxCapData[27] = 0.000000; ui35->fFluxCapData[28] = 0.000000; ui35->fFluxCapData[29] = 0.000000; ui35->fFluxCapData[30] = 0.000000; ui35->fFluxCapData[31] = 0.000000; ui35->fFluxCapData[32] = 0.000000; ui35->fFluxCapData[33] = 0.000000; ui35->fFluxCapData[34] = 0.000000; ui35->fFluxCapData[35] = 0.000000; ui35->fFluxCapData[36] = 0.000000; ui35->fFluxCapData[37] = 0.000000; ui35->fFluxCapData[38] = 0.000000; ui35->fFluxCapData[39] = 0.000000; ui35->fFluxCapData[40] = 0.000000; ui35->fFluxCapData[41] = 0.000000; ui35->fFluxCapData[42] = 0.000000; ui35->fFluxCapData[43] = 0.000000; ui35->fFluxCapData[44] = 0.000000; ui35->fFluxCapData[45] = 0.000000; ui35->fFluxCapData[46] = 0.000000; ui35->fFluxCapData[47] = 0.000000; ui35->fFluxCapData[48] = 0.000000; ui35->fFluxCapData[49] = 0.000000; ui35->fFluxCapData[50] = 0.000000; ui35->fFluxCapData[51] = 0.000000; ui35->fFluxCapData[52] = 0.000000; ui35->fFluxCapData[53] = 0.000000; ui35->fFluxCapData[54] = 0.000000; ui35->fFluxCapData[55] = 0.000000; ui35->fFluxCapData[56] = 0.000000; ui35->fFluxCapData[57] = 0.000000; ui35->fFluxCapData[58] = 0.000000; ui35->fFluxCapData[59] = 0.000000; ui35->fFluxCapData[60] = 0.000000; ui35->fFluxCapData[61] = 0.000000; ui35->fFluxCapData[62] = 0.000000; ui35->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui35);
	delete ui35;


	m_uFilterStructure = 0;
	CUICtrl* ui36 = new CUICtrl;
	ui36->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui36->uControlId = 43;
	ui36->bLogSlider = false;
	ui36->bExpSlider = false;
	ui36->fUserDisplayDataLoLimit = 0.000000;
	ui36->fUserDisplayDataHiLimit = 3.000000;
	ui36->uUserDataType = UINTData;
	ui36->fInitUserIntValue = 0;
	ui36->fInitUserFloatValue = 0;
	ui36->fInitUserDoubleValue = 0;
	ui36->fInitUserUINTValue = 0.000000;
	ui36->m_pUserCookedIntData = NULL;
	ui36->m_pUserCookedFloatData = NULL;
	ui36->m_pUserCookedDoubleData = NULL;
	ui36->m_pUserCookedUINTData = &m_uFilterStructure;
	ui36->cControlUnits = "";
	ui36->cVariableName = "m_uFilterStructure";
	ui36->cEnumeratedList = "DF,TDF,CF,TCF";
	ui36->dPresetData[0] = -0.000000;ui36->dPresetData[1] = 0.000000;ui36->dPresetData[2] = 0.000000;ui36->dPresetData[3] = 0.000000;ui36->dPresetData[4] = 0.000000;ui36->dPresetData[5] = 0.000000;ui36->dPresetData[6] = 0.000000;ui36->dPresetData[7] = 0.000000;ui36->dPresetData[8] = 0.000000;ui36->dPresetData[9] = 0.000000;ui36->dPresetData[10] = 0.000000;ui36->dPresetData[11] = 0.000000;ui36->dPresetData[12] = 0.000000;ui36->dPresetData[13] = 0.000000;ui36->dPresetData[14] = 0.000000;ui36->dPresetData[15] = 0.000000;
	ui36->cControlName = "Filter Structure";
	ui36->bOwnerControl = false;
	ui36->bMIDIControl = false;
	ui36->uMIDIControlCommand = 176;
	ui36->uMIDIControlName = 3;
	ui36->uMIDIControlChannel = 0;
	ui36->nGUIRow = nIndexer++;
	ui36->nGUIColumn = -1;
	ui36->bEnableParamSmoothing = false;
	ui36->fSmoothingTimeInMs = 100.0;
	ui36->uControlTheme[0] = 0; ui36->uControlTheme[1] = 0; ui36->uControlTheme[2] = 0; ui36->uControlTheme[3] = 8355711; ui36->uControlTheme[4] = 139; ui36->uControlTheme[5] = 0; ui36->uControlTheme[6] = 0; ui36->uControlTheme[7] = 0; ui36->uControlTheme[8] = 0; ui36->uControlTheme[9] = 0; ui36->uControlTheme[10] = 0; ui36->uControlTheme[11] = 0; ui36->uControlTheme[12] = 0; ui36->uControlTheme[13] = 0; ui36->uControlTheme[14] = 0; ui36->uControlTheme[15] = 0; ui36->uControlTheme[16] = 0; ui36->uControlTheme[17] = 0; ui36->uControlTheme[18] = 0; ui36->uControlTheme[19] = 0; ui36->uControlTheme[20] = 0; ui36->uControlTheme[21] = 0; ui36->uControlTheme[22] = 0; ui36->uControlTheme[23] = 0; ui36->uControlTheme[24] = 0; ui36->uControlTheme[25] = 0; ui36->uControlTheme[26] = 0; ui36->uControlTheme[27] = 0; ui36->uControlTheme[28] = 0; ui36->uControlTheme[29] = 0; ui36->uControlTheme[30] = 0; ui36->uControlTheme[31] = 0; 
	ui36->uFluxCapControl[0] = 0; ui36->uFluxCapControl[1] = 0; ui36->uFluxCapControl[2] = 0; ui36->uFluxCapControl[3] = 0; ui36->uFluxCapControl[4] = 0; ui36->uFluxCapControl[5] = 0; ui36->uFluxCapControl[6] = 0; ui36->uFluxCapControl[7] = 0; ui36->uFluxCapControl[8] = 0; ui36->uFluxCapControl[9] = 0; ui36->uFluxCapControl[10] = 0; ui36->uFluxCapControl[11] = 0; ui36->uFluxCapControl[12] = 0; ui36->uFluxCapControl[13] = 0; ui36->uFluxCapControl[14] = 0; ui36->uFluxCapControl[15] = 0; ui36->uFluxCapControl[16] = 0; ui36->uFluxCapControl[17] = 0; ui36->uFluxCapControl[18] = 0; ui36->uFluxCapControl[19] = 0; ui36->uFluxCapControl[20] = 0; ui36->uFluxCapControl[21] = 0; ui36->uFluxCapControl[22] = 0; ui36->uFluxCapControl[23] = 0; ui36->uFluxCapControl[24] = 0; ui36->uFluxCapControl[25] = 0; ui36->uFluxCapControl[26] = 0; ui36->uFluxCapControl[27] = 0; ui36->uFluxCapControl[28] = 0; ui36->uFluxCapControl[29] = 0; ui36->uFluxCapControl[30] = 0; ui36->uFluxCapControl[31] = 0; ui36->uFluxCapControl[32] = 0; ui36->uFluxCapControl[33] = 0; ui36->uFluxCapControl[34] = 0; ui36->uFluxCapControl[35] = 0; ui36->uFluxCapControl[36] = 0; ui36->uFluxCapControl[37] = 0; ui36->uFluxCapControl[38] = 0; ui36->uFluxCapControl[39] = 0; ui36->uFluxCapControl[40] = 0; ui36->uFluxCapControl[41] = 0; ui36->uFluxCapControl[42] = 0; ui36->uFluxCapControl[43] = 0; ui36->uFluxCapControl[44] = 0; ui36->uFluxCapControl[45] = 0; ui36->uFluxCapControl[46] = 0; ui36->uFluxCapControl[47] = 0; ui36->uFluxCapControl[48] = 0; ui36->uFluxCapControl[49] = 0; ui36->uFluxCapControl[50] = 0; ui36->uFluxCapControl[51] = 0; ui36->uFluxCapControl[52] = 0; ui36->uFluxCapControl[53] = 0; ui36->uFluxCapControl[54] = 0; ui36->uFluxCapControl[55] = 0; ui36->uFluxCapControl[56] = 0; ui36->uFluxCapControl[57] = 0; ui36->uFluxCapControl[58] = 0; ui36->uFluxCapControl[59] = 0; ui36->uFluxCapControl[60] = 0; ui36->uFluxCapControl[61] = 0; ui36->uFluxCapControl[62] = 0; ui36->uFluxCapControl[63] = 0; 
	ui36->fFluxCapData[0] = 0.000000; ui36->fFluxCapData[1] = 0.000000; ui36->fFluxCapData[2] = 0.000000; ui36->fFluxCapData[3] = 0.000000; ui36->fFluxCapData[4] = 0.000000; ui36->fFluxCapData[5] = 0.000000; ui36->fFluxCapData[6] = 0.000000; ui36->fFluxCapData[7] = 0.000000; ui36->fFluxCapData[8] = 0.000000; ui36->fFluxCapData[9] = 0.000000; ui36->fFluxCapData[10] = 0.000000; ui36->fFluxCapData[11] = 0.000000; ui36->fFluxCapData[12] = 0.000000; ui36->fFluxCapData[13] = 0.000000; ui36->fFluxCapData[14] = 0.000000; ui36->fFluxCapData[15] = 0.000000; ui36->fFluxCapData[16] = 0.000000; ui36->fFluxCapData[17] = 0.000000; ui36->fFluxCapData[18] = 0.000000; ui36->fFluxCapData[19] = 0.000000; ui36->fFluxCapData[20] = 0.000000; ui36->fFluxCapData[21] = 0.000000; ui36->fFluxCapData[22] = 0.000000; ui36->fFluxCapData[23] = 0.000000; ui36->fFluxCapData[24] = 0.000000; ui36->fFluxCapData[25] = 0.000000; ui36->fFluxCapData[26] = 0.000000; ui36->fFluxCapData[27] = 0.000000; ui36->fFluxCapData[28] = 0.000000; ui36->fFluxCapData[29] = 0.000000; ui36->fFluxCapData[30] = 0.000000; ui36->fFluxCapData[31] = 0.000000; ui36->fFluxCapData[32] = 0.000000; ui36->fFluxCapData[33] = 0.000000; ui36->fFluxCapData[34] = 0.000000; ui36->fFluxCapData[35] = 0.000000; ui36->fFluxCapData[36] = 0.000000; ui36->fFluxCapData[37] = 0.000000; ui36->fFluxCapData[38] = 0.000000; ui36->fFluxCapData[39] = 0.000000; ui36->fFluxCapData[40] = 0.000000; ui36->fFluxCapData[41] = 0.000000; ui36->fFluxCapData[42] = 0.000000; ui36->fFluxCapData[43] = 0.000000; ui36->fFluxCapData[44] = 0.000000; ui36->fFluxCapData[45] = 0.000000; ui36->fFluxCapData[46] = 0.000000; ui36->fFluxCapData[47] = 0.000000; ui36->fFluxCapData[48] = 0.000000; ui36->fFluxCapData[49] = 0.000000; ui36->fFluxCapData[50] = 0.000000; ui36->fFluxCapData[51] = 0.000000; ui36->fFluxCapData[52] = 0.000000; ui36->fFluxCapData[53] = 0.000000; ui36->fFluxCapData[54] = 0.000000; ui36->fFluxCapData[55] = 0.000000; ui36->fFluxCapData[56] = 0.000000; ui36->fFluxCapData[57] = 0.000000; ui36->fFluxCapData[58] = 0.000000; ui36->fFluxCapData[59] = 0.000000; ui36->fFluxCapData[60] = 0.000000; ui36->fFluxCapData[61] = 0.000000; ui36->fFluxCapData[62] = 0.000000; ui36->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui36);
	delete ui36;


	m_f_GC = 0;
	CUICtrl* ui37 = new CUICtrl;
	ui37->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui37->uControlId = 45;
	ui37->bLogSlider = false;
	ui37->bExpSlider = false;
	ui37->fUserDisplayDataLoLimit = 0.000000;
	ui37->fUserDisplayDataHiLimit = 1.000000;
	ui37->uUserDataType = UINTData;
	ui37->fInitUserIntValue = 0;
	ui37->fInitUserFloatValue = 0;
	ui37->fInitUserDoubleValue = 0;
	ui37->fInitUserUINTValue = 0.000000;
	ui37->m_pUserCookedIntData = NULL;
	ui37->m_pUserCookedFloatData = NULL;
	ui37->m_pUserCookedDoubleData = NULL;
	ui37->m_pUserCookedUINTData = &m_f_GC;
	ui37->cControlUnits = "";
	ui37->cVariableName = "m_f_GC";
	ui37->cEnumeratedList = "SWITCH_OFF,SWITCH_ON";
	ui37->dPresetData[0] = -0.000000;ui37->dPresetData[1] = 0.000000;ui37->dPresetData[2] = 0.000000;ui37->dPresetData[3] = 0.000000;ui37->dPresetData[4] = 0.000000;ui37->dPresetData[5] = 0.000000;ui37->dPresetData[6] = 0.000000;ui37->dPresetData[7] = 0.000000;ui37->dPresetData[8] = 0.000000;ui37->dPresetData[9] = 0.000000;ui37->dPresetData[10] = 0.000000;ui37->dPresetData[11] = 0.000000;ui37->dPresetData[12] = 0.000000;ui37->dPresetData[13] = 0.000000;ui37->dPresetData[14] = 0.000000;ui37->dPresetData[15] = 0.000000;
	ui37->cControlName = "Gain Comp";
	ui37->bOwnerControl = false;
	ui37->bMIDIControl = false;
	ui37->uMIDIControlCommand = 176;
	ui37->uMIDIControlName = 3;
	ui37->uMIDIControlChannel = 0;
	ui37->nGUIRow = nIndexer++;
	ui37->nGUIColumn = -1;
	ui37->bEnableParamSmoothing = false;
	ui37->fSmoothingTimeInMs = 100.0;
	ui37->uControlTheme[0] = 0; ui37->uControlTheme[1] = 0; ui37->uControlTheme[2] = 0; ui37->uControlTheme[3] = 8355711; ui37->uControlTheme[4] = 139; ui37->uControlTheme[5] = 0; ui37->uControlTheme[6] = 0; ui37->uControlTheme[7] = 0; ui37->uControlTheme[8] = 0; ui37->uControlTheme[9] = 0; ui37->uControlTheme[10] = 0; ui37->uControlTheme[11] = 0; ui37->uControlTheme[12] = 0; ui37->uControlTheme[13] = 0; ui37->uControlTheme[14] = 0; ui37->uControlTheme[15] = 0; ui37->uControlTheme[16] = 0; ui37->uControlTheme[17] = 0; ui37->uControlTheme[18] = 0; ui37->uControlTheme[19] = 0; ui37->uControlTheme[20] = 0; ui37->uControlTheme[21] = 0; ui37->uControlTheme[22] = 0; ui37->uControlTheme[23] = 0; ui37->uControlTheme[24] = 0; ui37->uControlTheme[25] = 0; ui37->uControlTheme[26] = 0; ui37->uControlTheme[27] = 0; ui37->uControlTheme[28] = 0; ui37->uControlTheme[29] = 0; ui37->uControlTheme[30] = 0; ui37->uControlTheme[31] = 0; 
	ui37->uFluxCapControl[0] = 0; ui37->uFluxCapControl[1] = 0; ui37->uFluxCapControl[2] = 0; ui37->uFluxCapControl[3] = 0; ui37->uFluxCapControl[4] = 0; ui37->uFluxCapControl[5] = 0; ui37->uFluxCapControl[6] = 0; ui37->uFluxCapControl[7] = 0; ui37->uFluxCapControl[8] = 0; ui37->uFluxCapControl[9] = 0; ui37->uFluxCapControl[10] = 0; ui37->uFluxCapControl[11] = 0; ui37->uFluxCapControl[12] = 0; ui37->uFluxCapControl[13] = 0; ui37->uFluxCapControl[14] = 0; ui37->uFluxCapControl[15] = 0; ui37->uFluxCapControl[16] = 0; ui37->uFluxCapControl[17] = 0; ui37->uFluxCapControl[18] = 0; ui37->uFluxCapControl[19] = 0; ui37->uFluxCapControl[20] = 0; ui37->uFluxCapControl[21] = 0; ui37->uFluxCapControl[22] = 0; ui37->uFluxCapControl[23] = 0; ui37->uFluxCapControl[24] = 0; ui37->uFluxCapControl[25] = 0; ui37->uFluxCapControl[26] = 0; ui37->uFluxCapControl[27] = 0; ui37->uFluxCapControl[28] = 0; ui37->uFluxCapControl[29] = 0; ui37->uFluxCapControl[30] = 0; ui37->uFluxCapControl[31] = 0; ui37->uFluxCapControl[32] = 0; ui37->uFluxCapControl[33] = 0; ui37->uFluxCapControl[34] = 0; ui37->uFluxCapControl[35] = 0; ui37->uFluxCapControl[36] = 0; ui37->uFluxCapControl[37] = 0; ui37->uFluxCapControl[38] = 0; ui37->uFluxCapControl[39] = 0; ui37->uFluxCapControl[40] = 0; ui37->uFluxCapControl[41] = 0; ui37->uFluxCapControl[42] = 0; ui37->uFluxCapControl[43] = 0; ui37->uFluxCapControl[44] = 0; ui37->uFluxCapControl[45] = 0; ui37->uFluxCapControl[46] = 0; ui37->uFluxCapControl[47] = 0; ui37->uFluxCapControl[48] = 0; ui37->uFluxCapControl[49] = 0; ui37->uFluxCapControl[50] = 0; ui37->uFluxCapControl[51] = 0; ui37->uFluxCapControl[52] = 0; ui37->uFluxCapControl[53] = 0; ui37->uFluxCapControl[54] = 0; ui37->uFluxCapControl[55] = 0; ui37->uFluxCapControl[56] = 0; ui37->uFluxCapControl[57] = 0; ui37->uFluxCapControl[58] = 0; ui37->uFluxCapControl[59] = 0; ui37->uFluxCapControl[60] = 0; ui37->uFluxCapControl[61] = 0; ui37->uFluxCapControl[62] = 0; ui37->uFluxCapControl[63] = 0; 
	ui37->fFluxCapData[0] = 0.000000; ui37->fFluxCapData[1] = 0.000000; ui37->fFluxCapData[2] = 0.000000; ui37->fFluxCapData[3] = 0.000000; ui37->fFluxCapData[4] = 0.000000; ui37->fFluxCapData[5] = 0.000000; ui37->fFluxCapData[6] = 0.000000; ui37->fFluxCapData[7] = 0.000000; ui37->fFluxCapData[8] = 0.000000; ui37->fFluxCapData[9] = 0.000000; ui37->fFluxCapData[10] = 0.000000; ui37->fFluxCapData[11] = 0.000000; ui37->fFluxCapData[12] = 0.000000; ui37->fFluxCapData[13] = 0.000000; ui37->fFluxCapData[14] = 0.000000; ui37->fFluxCapData[15] = 0.000000; ui37->fFluxCapData[16] = 0.000000; ui37->fFluxCapData[17] = 0.000000; ui37->fFluxCapData[18] = 0.000000; ui37->fFluxCapData[19] = 0.000000; ui37->fFluxCapData[20] = 0.000000; ui37->fFluxCapData[21] = 0.000000; ui37->fFluxCapData[22] = 0.000000; ui37->fFluxCapData[23] = 0.000000; ui37->fFluxCapData[24] = 0.000000; ui37->fFluxCapData[25] = 0.000000; ui37->fFluxCapData[26] = 0.000000; ui37->fFluxCapData[27] = 0.000000; ui37->fFluxCapData[28] = 0.000000; ui37->fFluxCapData[29] = 0.000000; ui37->fFluxCapData[30] = 0.000000; ui37->fFluxCapData[31] = 0.000000; ui37->fFluxCapData[32] = 0.000000; ui37->fFluxCapData[33] = 0.000000; ui37->fFluxCapData[34] = 0.000000; ui37->fFluxCapData[35] = 0.000000; ui37->fFluxCapData[36] = 0.000000; ui37->fFluxCapData[37] = 0.000000; ui37->fFluxCapData[38] = 0.000000; ui37->fFluxCapData[39] = 0.000000; ui37->fFluxCapData[40] = 0.000000; ui37->fFluxCapData[41] = 0.000000; ui37->fFluxCapData[42] = 0.000000; ui37->fFluxCapData[43] = 0.000000; ui37->fFluxCapData[44] = 0.000000; ui37->fFluxCapData[45] = 0.000000; ui37->fFluxCapData[46] = 0.000000; ui37->fFluxCapData[47] = 0.000000; ui37->fFluxCapData[48] = 0.000000; ui37->fFluxCapData[49] = 0.000000; ui37->fFluxCapData[50] = 0.000000; ui37->fFluxCapData[51] = 0.000000; ui37->fFluxCapData[52] = 0.000000; ui37->fFluxCapData[53] = 0.000000; ui37->fFluxCapData[54] = 0.000000; ui37->fFluxCapData[55] = 0.000000; ui37->fFluxCapData[56] = 0.000000; ui37->fFluxCapData[57] = 0.000000; ui37->fFluxCapData[58] = 0.000000; ui37->fFluxCapData[59] = 0.000000; ui37->fFluxCapData[60] = 0.000000; ui37->fFluxCapData[61] = 0.000000; ui37->fFluxCapData[62] = 0.000000; ui37->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui37);
	delete ui37;


	m_uCoupleQ = 0;
	CUICtrl* ui38 = new CUICtrl;
	ui38->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui38->uControlId = 46;
	ui38->bLogSlider = false;
	ui38->bExpSlider = false;
	ui38->fUserDisplayDataLoLimit = 0.000000;
	ui38->fUserDisplayDataHiLimit = 1.000000;
	ui38->uUserDataType = UINTData;
	ui38->fInitUserIntValue = 0;
	ui38->fInitUserFloatValue = 0;
	ui38->fInitUserDoubleValue = 0;
	ui38->fInitUserUINTValue = 0.000000;
	ui38->m_pUserCookedIntData = NULL;
	ui38->m_pUserCookedFloatData = NULL;
	ui38->m_pUserCookedDoubleData = NULL;
	ui38->m_pUserCookedUINTData = &m_uCoupleQ;
	ui38->cControlUnits = "";
	ui38->cVariableName = "m_uCoupleQ";
	ui38->cEnumeratedList = "SWITCH_OFF,SWITCH_ON";
	ui38->dPresetData[0] = -0.000000;ui38->dPresetData[1] = 0.000000;ui38->dPresetData[2] = 0.000000;ui38->dPresetData[3] = 0.000000;ui38->dPresetData[4] = 0.000000;ui38->dPresetData[5] = 0.000000;ui38->dPresetData[6] = 0.000000;ui38->dPresetData[7] = 0.000000;ui38->dPresetData[8] = 0.000000;ui38->dPresetData[9] = 0.000000;ui38->dPresetData[10] = 0.000000;ui38->dPresetData[11] = 0.000000;ui38->dPresetData[12] = 0.000000;ui38->dPresetData[13] = 0.000000;ui38->dPresetData[14] = 0.000000;ui38->dPresetData[15] = 0.000000;
	ui38->cControlName = "Couple Q";
	ui38->bOwnerControl = false;
	ui38->bMIDIControl = false;
	ui38->uMIDIControlCommand = 176;
	ui38->uMIDIControlName = 3;
	ui38->uMIDIControlChannel = 0;
	ui38->nGUIRow = nIndexer++;
	ui38->nGUIColumn = -1;
	ui38->bEnableParamSmoothing = false;
	ui38->fSmoothingTimeInMs = 100.0;
	ui38->uControlTheme[0] = 0; ui38->uControlTheme[1] = 0; ui38->uControlTheme[2] = 0; ui38->uControlTheme[3] = 8355711; ui38->uControlTheme[4] = 139; ui38->uControlTheme[5] = 0; ui38->uControlTheme[6] = 0; ui38->uControlTheme[7] = 0; ui38->uControlTheme[8] = 0; ui38->uControlTheme[9] = 0; ui38->uControlTheme[10] = 0; ui38->uControlTheme[11] = 0; ui38->uControlTheme[12] = 0; ui38->uControlTheme[13] = 0; ui38->uControlTheme[14] = 0; ui38->uControlTheme[15] = 0; ui38->uControlTheme[16] = 0; ui38->uControlTheme[17] = 0; ui38->uControlTheme[18] = 0; ui38->uControlTheme[19] = 0; ui38->uControlTheme[20] = 0; ui38->uControlTheme[21] = 0; ui38->uControlTheme[22] = 0; ui38->uControlTheme[23] = 0; ui38->uControlTheme[24] = 0; ui38->uControlTheme[25] = 0; ui38->uControlTheme[26] = 0; ui38->uControlTheme[27] = 0; ui38->uControlTheme[28] = 0; ui38->uControlTheme[29] = 0; ui38->uControlTheme[30] = 0; ui38->uControlTheme[31] = 0; 
	ui38->uFluxCapControl[0] = 0; ui38->uFluxCapControl[1] = 0; ui38->uFluxCapControl[2] = 0; ui38->uFluxCapControl[3] = 0; ui38->uFluxCapControl[4] = 0; ui38->uFluxCapControl[5] = 0; ui38->uFluxCapControl[6] = 0; ui38->uFluxCapControl[7] = 0; ui38->uFluxCapControl[8] = 0; ui38->uFluxCapControl[9] = 0; ui38->uFluxCapControl[10] = 0; ui38->uFluxCapControl[11] = 0; ui38->uFluxCapControl[12] = 0; ui38->uFluxCapControl[13] = 0; ui38->uFluxCapControl[14] = 0; ui38->uFluxCapControl[15] = 0; ui38->uFluxCapControl[16] = 0; ui38->uFluxCapControl[17] = 0; ui38->uFluxCapControl[18] = 0; ui38->uFluxCapControl[19] = 0; ui38->uFluxCapControl[20] = 0; ui38->uFluxCapControl[21] = 0; ui38->uFluxCapControl[22] = 0; ui38->uFluxCapControl[23] = 0; ui38->uFluxCapControl[24] = 0; ui38->uFluxCapControl[25] = 0; ui38->uFluxCapControl[26] = 0; ui38->uFluxCapControl[27] = 0; ui38->uFluxCapControl[28] = 0; ui38->uFluxCapControl[29] = 0; ui38->uFluxCapControl[30] = 0; ui38->uFluxCapControl[31] = 0; ui38->uFluxCapControl[32] = 0; ui38->uFluxCapControl[33] = 0; ui38->uFluxCapControl[34] = 0; ui38->uFluxCapControl[35] = 0; ui38->uFluxCapControl[36] = 0; ui38->uFluxCapControl[37] = 0; ui38->uFluxCapControl[38] = 0; ui38->uFluxCapControl[39] = 0; ui38->uFluxCapControl[40] = 0; ui38->uFluxCapControl[41] = 0; ui38->uFluxCapControl[42] = 0; ui38->uFluxCapControl[43] = 0; ui38->uFluxCapControl[44] = 0; ui38->uFluxCapControl[45] = 0; ui38->uFluxCapControl[46] = 0; ui38->uFluxCapControl[47] = 0; ui38->uFluxCapControl[48] = 0; ui38->uFluxCapControl[49] = 0; ui38->uFluxCapControl[50] = 0; ui38->uFluxCapControl[51] = 0; ui38->uFluxCapControl[52] = 0; ui38->uFluxCapControl[53] = 0; ui38->uFluxCapControl[54] = 0; ui38->uFluxCapControl[55] = 0; ui38->uFluxCapControl[56] = 0; ui38->uFluxCapControl[57] = 0; ui38->uFluxCapControl[58] = 0; ui38->uFluxCapControl[59] = 0; ui38->uFluxCapControl[60] = 0; ui38->uFluxCapControl[61] = 0; ui38->uFluxCapControl[62] = 0; ui38->uFluxCapControl[63] = 0; 
	ui38->fFluxCapData[0] = 0.000000; ui38->fFluxCapData[1] = 0.000000; ui38->fFluxCapData[2] = 0.000000; ui38->fFluxCapData[3] = 0.000000; ui38->fFluxCapData[4] = 0.000000; ui38->fFluxCapData[5] = 0.000000; ui38->fFluxCapData[6] = 0.000000; ui38->fFluxCapData[7] = 0.000000; ui38->fFluxCapData[8] = 0.000000; ui38->fFluxCapData[9] = 0.000000; ui38->fFluxCapData[10] = 0.000000; ui38->fFluxCapData[11] = 0.000000; ui38->fFluxCapData[12] = 0.000000; ui38->fFluxCapData[13] = 0.000000; ui38->fFluxCapData[14] = 0.000000; ui38->fFluxCapData[15] = 0.000000; ui38->fFluxCapData[16] = 0.000000; ui38->fFluxCapData[17] = 0.000000; ui38->fFluxCapData[18] = 0.000000; ui38->fFluxCapData[19] = 0.000000; ui38->fFluxCapData[20] = 0.000000; ui38->fFluxCapData[21] = 0.000000; ui38->fFluxCapData[22] = 0.000000; ui38->fFluxCapData[23] = 0.000000; ui38->fFluxCapData[24] = 0.000000; ui38->fFluxCapData[25] = 0.000000; ui38->fFluxCapData[26] = 0.000000; ui38->fFluxCapData[27] = 0.000000; ui38->fFluxCapData[28] = 0.000000; ui38->fFluxCapData[29] = 0.000000; ui38->fFluxCapData[30] = 0.000000; ui38->fFluxCapData[31] = 0.000000; ui38->fFluxCapData[32] = 0.000000; ui38->fFluxCapData[33] = 0.000000; ui38->fFluxCapData[34] = 0.000000; ui38->fFluxCapData[35] = 0.000000; ui38->fFluxCapData[36] = 0.000000; ui38->fFluxCapData[37] = 0.000000; ui38->fFluxCapData[38] = 0.000000; ui38->fFluxCapData[39] = 0.000000; ui38->fFluxCapData[40] = 0.000000; ui38->fFluxCapData[41] = 0.000000; ui38->fFluxCapData[42] = 0.000000; ui38->fFluxCapData[43] = 0.000000; ui38->fFluxCapData[44] = 0.000000; ui38->fFluxCapData[45] = 0.000000; ui38->fFluxCapData[46] = 0.000000; ui38->fFluxCapData[47] = 0.000000; ui38->fFluxCapData[48] = 0.000000; ui38->fFluxCapData[49] = 0.000000; ui38->fFluxCapData[50] = 0.000000; ui38->fFluxCapData[51] = 0.000000; ui38->fFluxCapData[52] = 0.000000; ui38->fFluxCapData[53] = 0.000000; ui38->fFluxCapData[54] = 0.000000; ui38->fFluxCapData[55] = 0.000000; ui38->fFluxCapData[56] = 0.000000; ui38->fFluxCapData[57] = 0.000000; ui38->fFluxCapData[58] = 0.000000; ui38->fFluxCapData[59] = 0.000000; ui38->fFluxCapData[60] = 0.000000; ui38->fFluxCapData[61] = 0.000000; ui38->fFluxCapData[62] = 0.000000; ui38->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui38);
	delete ui38;


	m_uDelayType_2 = 0;
	CUICtrl* ui39 = new CUICtrl;
	ui39->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui39->uControlId = 44;
	ui39->bLogSlider = false;
	ui39->bExpSlider = false;
	ui39->fUserDisplayDataLoLimit = 0.000000;
	ui39->fUserDisplayDataHiLimit = 2.000000;
	ui39->uUserDataType = UINTData;
	ui39->fInitUserIntValue = 0;
	ui39->fInitUserFloatValue = 0;
	ui39->fInitUserDoubleValue = 0;
	ui39->fInitUserUINTValue = 0.000000;
	ui39->m_pUserCookedIntData = NULL;
	ui39->m_pUserCookedFloatData = NULL;
	ui39->m_pUserCookedDoubleData = NULL;
	ui39->m_pUserCookedUINTData = &m_uDelayType_2;
	ui39->cControlUnits = "";
	ui39->cVariableName = "m_uDelayType_2";
	ui39->cEnumeratedList = "delay,flanger,basschorus";
	ui39->dPresetData[0] = -0.000000;ui39->dPresetData[1] = 0.000000;ui39->dPresetData[2] = 0.000000;ui39->dPresetData[3] = 0.000000;ui39->dPresetData[4] = 0.000000;ui39->dPresetData[5] = 0.000000;ui39->dPresetData[6] = 0.000000;ui39->dPresetData[7] = 0.000000;ui39->dPresetData[8] = 0.000000;ui39->dPresetData[9] = 0.000000;ui39->dPresetData[10] = 0.000000;ui39->dPresetData[11] = 0.000000;ui39->dPresetData[12] = 0.000000;ui39->dPresetData[13] = 0.000000;ui39->dPresetData[14] = 0.000000;ui39->dPresetData[15] = 0.000000;
	ui39->cControlName = "Delay Type";
	ui39->bOwnerControl = false;
	ui39->bMIDIControl = false;
	ui39->uMIDIControlCommand = 176;
	ui39->uMIDIControlName = 3;
	ui39->uMIDIControlChannel = 0;
	ui39->nGUIRow = nIndexer++;
	ui39->nGUIColumn = -1;
	ui39->bEnableParamSmoothing = false;
	ui39->fSmoothingTimeInMs = 100.0;
	ui39->uControlTheme[0] = 0; ui39->uControlTheme[1] = 0; ui39->uControlTheme[2] = 0; ui39->uControlTheme[3] = 8355711; ui39->uControlTheme[4] = 139; ui39->uControlTheme[5] = 0; ui39->uControlTheme[6] = 0; ui39->uControlTheme[7] = 0; ui39->uControlTheme[8] = 0; ui39->uControlTheme[9] = 0; ui39->uControlTheme[10] = 0; ui39->uControlTheme[11] = 0; ui39->uControlTheme[12] = 0; ui39->uControlTheme[13] = 0; ui39->uControlTheme[14] = 0; ui39->uControlTheme[15] = 0; ui39->uControlTheme[16] = 0; ui39->uControlTheme[17] = 0; ui39->uControlTheme[18] = 0; ui39->uControlTheme[19] = 0; ui39->uControlTheme[20] = 0; ui39->uControlTheme[21] = 0; ui39->uControlTheme[22] = 0; ui39->uControlTheme[23] = 0; ui39->uControlTheme[24] = 0; ui39->uControlTheme[25] = 0; ui39->uControlTheme[26] = 0; ui39->uControlTheme[27] = 0; ui39->uControlTheme[28] = 0; ui39->uControlTheme[29] = 0; ui39->uControlTheme[30] = 0; ui39->uControlTheme[31] = 0; 
	ui39->uFluxCapControl[0] = 0; ui39->uFluxCapControl[1] = 0; ui39->uFluxCapControl[2] = 0; ui39->uFluxCapControl[3] = 0; ui39->uFluxCapControl[4] = 0; ui39->uFluxCapControl[5] = 0; ui39->uFluxCapControl[6] = 0; ui39->uFluxCapControl[7] = 0; ui39->uFluxCapControl[8] = 0; ui39->uFluxCapControl[9] = 0; ui39->uFluxCapControl[10] = 0; ui39->uFluxCapControl[11] = 0; ui39->uFluxCapControl[12] = 0; ui39->uFluxCapControl[13] = 0; ui39->uFluxCapControl[14] = 0; ui39->uFluxCapControl[15] = 0; ui39->uFluxCapControl[16] = 0; ui39->uFluxCapControl[17] = 0; ui39->uFluxCapControl[18] = 0; ui39->uFluxCapControl[19] = 0; ui39->uFluxCapControl[20] = 0; ui39->uFluxCapControl[21] = 0; ui39->uFluxCapControl[22] = 0; ui39->uFluxCapControl[23] = 0; ui39->uFluxCapControl[24] = 0; ui39->uFluxCapControl[25] = 0; ui39->uFluxCapControl[26] = 0; ui39->uFluxCapControl[27] = 0; ui39->uFluxCapControl[28] = 0; ui39->uFluxCapControl[29] = 0; ui39->uFluxCapControl[30] = 0; ui39->uFluxCapControl[31] = 0; ui39->uFluxCapControl[32] = 0; ui39->uFluxCapControl[33] = 0; ui39->uFluxCapControl[34] = 0; ui39->uFluxCapControl[35] = 0; ui39->uFluxCapControl[36] = 0; ui39->uFluxCapControl[37] = 0; ui39->uFluxCapControl[38] = 0; ui39->uFluxCapControl[39] = 0; ui39->uFluxCapControl[40] = 0; ui39->uFluxCapControl[41] = 0; ui39->uFluxCapControl[42] = 0; ui39->uFluxCapControl[43] = 0; ui39->uFluxCapControl[44] = 0; ui39->uFluxCapControl[45] = 0; ui39->uFluxCapControl[46] = 0; ui39->uFluxCapControl[47] = 0; ui39->uFluxCapControl[48] = 0; ui39->uFluxCapControl[49] = 0; ui39->uFluxCapControl[50] = 0; ui39->uFluxCapControl[51] = 0; ui39->uFluxCapControl[52] = 0; ui39->uFluxCapControl[53] = 0; ui39->uFluxCapControl[54] = 0; ui39->uFluxCapControl[55] = 0; ui39->uFluxCapControl[56] = 0; ui39->uFluxCapControl[57] = 0; ui39->uFluxCapControl[58] = 0; ui39->uFluxCapControl[59] = 0; ui39->uFluxCapControl[60] = 0; ui39->uFluxCapControl[61] = 0; ui39->uFluxCapControl[62] = 0; ui39->uFluxCapControl[63] = 0; 
	ui39->fFluxCapData[0] = 0.000000; ui39->fFluxCapData[1] = 0.000000; ui39->fFluxCapData[2] = 0.000000; ui39->fFluxCapData[3] = 0.000000; ui39->fFluxCapData[4] = 0.000000; ui39->fFluxCapData[5] = 0.000000; ui39->fFluxCapData[6] = 0.000000; ui39->fFluxCapData[7] = 0.000000; ui39->fFluxCapData[8] = 0.000000; ui39->fFluxCapData[9] = 0.000000; ui39->fFluxCapData[10] = 0.000000; ui39->fFluxCapData[11] = 0.000000; ui39->fFluxCapData[12] = 0.000000; ui39->fFluxCapData[13] = 0.000000; ui39->fFluxCapData[14] = 0.000000; ui39->fFluxCapData[15] = 0.000000; ui39->fFluxCapData[16] = 0.000000; ui39->fFluxCapData[17] = 0.000000; ui39->fFluxCapData[18] = 0.000000; ui39->fFluxCapData[19] = 0.000000; ui39->fFluxCapData[20] = 0.000000; ui39->fFluxCapData[21] = 0.000000; ui39->fFluxCapData[22] = 0.000000; ui39->fFluxCapData[23] = 0.000000; ui39->fFluxCapData[24] = 0.000000; ui39->fFluxCapData[25] = 0.000000; ui39->fFluxCapData[26] = 0.000000; ui39->fFluxCapData[27] = 0.000000; ui39->fFluxCapData[28] = 0.000000; ui39->fFluxCapData[29] = 0.000000; ui39->fFluxCapData[30] = 0.000000; ui39->fFluxCapData[31] = 0.000000; ui39->fFluxCapData[32] = 0.000000; ui39->fFluxCapData[33] = 0.000000; ui39->fFluxCapData[34] = 0.000000; ui39->fFluxCapData[35] = 0.000000; ui39->fFluxCapData[36] = 0.000000; ui39->fFluxCapData[37] = 0.000000; ui39->fFluxCapData[38] = 0.000000; ui39->fFluxCapData[39] = 0.000000; ui39->fFluxCapData[40] = 0.000000; ui39->fFluxCapData[41] = 0.000000; ui39->fFluxCapData[42] = 0.000000; ui39->fFluxCapData[43] = 0.000000; ui39->fFluxCapData[44] = 0.000000; ui39->fFluxCapData[45] = 0.000000; ui39->fFluxCapData[46] = 0.000000; ui39->fFluxCapData[47] = 0.000000; ui39->fFluxCapData[48] = 0.000000; ui39->fFluxCapData[49] = 0.000000; ui39->fFluxCapData[50] = 0.000000; ui39->fFluxCapData[51] = 0.000000; ui39->fFluxCapData[52] = 0.000000; ui39->fFluxCapData[53] = 0.000000; ui39->fFluxCapData[54] = 0.000000; ui39->fFluxCapData[55] = 0.000000; ui39->fFluxCapData[56] = 0.000000; ui39->fFluxCapData[57] = 0.000000; ui39->fFluxCapData[58] = 0.000000; ui39->fFluxCapData[59] = 0.000000; ui39->fFluxCapData[60] = 0.000000; ui39->fFluxCapData[61] = 0.000000; ui39->fFluxCapData[62] = 0.000000; ui39->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui39);
	delete ui39;


	m_uDirection = 0;
	CUICtrl* ui40 = new CUICtrl;
	ui40->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui40->uControlId = 42;
	ui40->bLogSlider = false;
	ui40->bExpSlider = false;
	ui40->fUserDisplayDataLoLimit = 0.000000;
	ui40->fUserDisplayDataHiLimit = 1.000000;
	ui40->uUserDataType = UINTData;
	ui40->fInitUserIntValue = 0;
	ui40->fInitUserFloatValue = 0;
	ui40->fInitUserDoubleValue = 0;
	ui40->fInitUserUINTValue = 0.000000;
	ui40->m_pUserCookedIntData = NULL;
	ui40->m_pUserCookedFloatData = NULL;
	ui40->m_pUserCookedDoubleData = NULL;
	ui40->m_pUserCookedUINTData = &m_uDirection;
	ui40->cControlUnits = "";
	ui40->cVariableName = "m_uDirection";
	ui40->cEnumeratedList = "UP,DOWN";
	ui40->dPresetData[0] = -0.000000;ui40->dPresetData[1] = 0.000000;ui40->dPresetData[2] = 0.000000;ui40->dPresetData[3] = 0.000000;ui40->dPresetData[4] = 0.000000;ui40->dPresetData[5] = 0.000000;ui40->dPresetData[6] = 0.000000;ui40->dPresetData[7] = 0.000000;ui40->dPresetData[8] = 0.000000;ui40->dPresetData[9] = 0.000000;ui40->dPresetData[10] = 0.000000;ui40->dPresetData[11] = 0.000000;ui40->dPresetData[12] = 0.000000;ui40->dPresetData[13] = 0.000000;ui40->dPresetData[14] = 0.000000;ui40->dPresetData[15] = 0.000000;
	ui40->cControlName = "Direction";
	ui40->bOwnerControl = false;
	ui40->bMIDIControl = false;
	ui40->uMIDIControlCommand = 176;
	ui40->uMIDIControlName = 3;
	ui40->uMIDIControlChannel = 0;
	ui40->nGUIRow = nIndexer++;
	ui40->nGUIColumn = -1;
	ui40->bEnableParamSmoothing = false;
	ui40->fSmoothingTimeInMs = 100.0;
	ui40->uControlTheme[0] = 0; ui40->uControlTheme[1] = 0; ui40->uControlTheme[2] = 0; ui40->uControlTheme[3] = 8355711; ui40->uControlTheme[4] = 139; ui40->uControlTheme[5] = 0; ui40->uControlTheme[6] = 0; ui40->uControlTheme[7] = 0; ui40->uControlTheme[8] = 0; ui40->uControlTheme[9] = 0; ui40->uControlTheme[10] = 0; ui40->uControlTheme[11] = 0; ui40->uControlTheme[12] = 0; ui40->uControlTheme[13] = 0; ui40->uControlTheme[14] = 0; ui40->uControlTheme[15] = 0; ui40->uControlTheme[16] = 0; ui40->uControlTheme[17] = 0; ui40->uControlTheme[18] = 0; ui40->uControlTheme[19] = 0; ui40->uControlTheme[20] = 0; ui40->uControlTheme[21] = 0; ui40->uControlTheme[22] = 0; ui40->uControlTheme[23] = 0; ui40->uControlTheme[24] = 0; ui40->uControlTheme[25] = 0; ui40->uControlTheme[26] = 0; ui40->uControlTheme[27] = 0; ui40->uControlTheme[28] = 0; ui40->uControlTheme[29] = 0; ui40->uControlTheme[30] = 0; ui40->uControlTheme[31] = 0; 
	ui40->uFluxCapControl[0] = 0; ui40->uFluxCapControl[1] = 0; ui40->uFluxCapControl[2] = 0; ui40->uFluxCapControl[3] = 0; ui40->uFluxCapControl[4] = 0; ui40->uFluxCapControl[5] = 0; ui40->uFluxCapControl[6] = 0; ui40->uFluxCapControl[7] = 0; ui40->uFluxCapControl[8] = 0; ui40->uFluxCapControl[9] = 0; ui40->uFluxCapControl[10] = 0; ui40->uFluxCapControl[11] = 0; ui40->uFluxCapControl[12] = 0; ui40->uFluxCapControl[13] = 0; ui40->uFluxCapControl[14] = 0; ui40->uFluxCapControl[15] = 0; ui40->uFluxCapControl[16] = 0; ui40->uFluxCapControl[17] = 0; ui40->uFluxCapControl[18] = 0; ui40->uFluxCapControl[19] = 0; ui40->uFluxCapControl[20] = 0; ui40->uFluxCapControl[21] = 0; ui40->uFluxCapControl[22] = 0; ui40->uFluxCapControl[23] = 0; ui40->uFluxCapControl[24] = 0; ui40->uFluxCapControl[25] = 0; ui40->uFluxCapControl[26] = 0; ui40->uFluxCapControl[27] = 0; ui40->uFluxCapControl[28] = 0; ui40->uFluxCapControl[29] = 0; ui40->uFluxCapControl[30] = 0; ui40->uFluxCapControl[31] = 0; ui40->uFluxCapControl[32] = 0; ui40->uFluxCapControl[33] = 0; ui40->uFluxCapControl[34] = 0; ui40->uFluxCapControl[35] = 0; ui40->uFluxCapControl[36] = 0; ui40->uFluxCapControl[37] = 0; ui40->uFluxCapControl[38] = 0; ui40->uFluxCapControl[39] = 0; ui40->uFluxCapControl[40] = 0; ui40->uFluxCapControl[41] = 0; ui40->uFluxCapControl[42] = 0; ui40->uFluxCapControl[43] = 0; ui40->uFluxCapControl[44] = 0; ui40->uFluxCapControl[45] = 0; ui40->uFluxCapControl[46] = 0; ui40->uFluxCapControl[47] = 0; ui40->uFluxCapControl[48] = 0; ui40->uFluxCapControl[49] = 0; ui40->uFluxCapControl[50] = 0; ui40->uFluxCapControl[51] = 0; ui40->uFluxCapControl[52] = 0; ui40->uFluxCapControl[53] = 0; ui40->uFluxCapControl[54] = 0; ui40->uFluxCapControl[55] = 0; ui40->uFluxCapControl[56] = 0; ui40->uFluxCapControl[57] = 0; ui40->uFluxCapControl[58] = 0; ui40->uFluxCapControl[59] = 0; ui40->uFluxCapControl[60] = 0; ui40->uFluxCapControl[61] = 0; ui40->uFluxCapControl[62] = 0; ui40->uFluxCapControl[63] = 0; 
	ui40->fFluxCapData[0] = 0.000000; ui40->fFluxCapData[1] = 0.000000; ui40->fFluxCapData[2] = 0.000000; ui40->fFluxCapData[3] = 0.000000; ui40->fFluxCapData[4] = 0.000000; ui40->fFluxCapData[5] = 0.000000; ui40->fFluxCapData[6] = 0.000000; ui40->fFluxCapData[7] = 0.000000; ui40->fFluxCapData[8] = 0.000000; ui40->fFluxCapData[9] = 0.000000; ui40->fFluxCapData[10] = 0.000000; ui40->fFluxCapData[11] = 0.000000; ui40->fFluxCapData[12] = 0.000000; ui40->fFluxCapData[13] = 0.000000; ui40->fFluxCapData[14] = 0.000000; ui40->fFluxCapData[15] = 0.000000; ui40->fFluxCapData[16] = 0.000000; ui40->fFluxCapData[17] = 0.000000; ui40->fFluxCapData[18] = 0.000000; ui40->fFluxCapData[19] = 0.000000; ui40->fFluxCapData[20] = 0.000000; ui40->fFluxCapData[21] = 0.000000; ui40->fFluxCapData[22] = 0.000000; ui40->fFluxCapData[23] = 0.000000; ui40->fFluxCapData[24] = 0.000000; ui40->fFluxCapData[25] = 0.000000; ui40->fFluxCapData[26] = 0.000000; ui40->fFluxCapData[27] = 0.000000; ui40->fFluxCapData[28] = 0.000000; ui40->fFluxCapData[29] = 0.000000; ui40->fFluxCapData[30] = 0.000000; ui40->fFluxCapData[31] = 0.000000; ui40->fFluxCapData[32] = 0.000000; ui40->fFluxCapData[33] = 0.000000; ui40->fFluxCapData[34] = 0.000000; ui40->fFluxCapData[35] = 0.000000; ui40->fFluxCapData[36] = 0.000000; ui40->fFluxCapData[37] = 0.000000; ui40->fFluxCapData[38] = 0.000000; ui40->fFluxCapData[39] = 0.000000; ui40->fFluxCapData[40] = 0.000000; ui40->fFluxCapData[41] = 0.000000; ui40->fFluxCapData[42] = 0.000000; ui40->fFluxCapData[43] = 0.000000; ui40->fFluxCapData[44] = 0.000000; ui40->fFluxCapData[45] = 0.000000; ui40->fFluxCapData[46] = 0.000000; ui40->fFluxCapData[47] = 0.000000; ui40->fFluxCapData[48] = 0.000000; ui40->fFluxCapData[49] = 0.000000; ui40->fFluxCapData[50] = 0.000000; ui40->fFluxCapData[51] = 0.000000; ui40->fFluxCapData[52] = 0.000000; ui40->fFluxCapData[53] = 0.000000; ui40->fFluxCapData[54] = 0.000000; ui40->fFluxCapData[55] = 0.000000; ui40->fFluxCapData[56] = 0.000000; ui40->fFluxCapData[57] = 0.000000; ui40->fFluxCapData[58] = 0.000000; ui40->fFluxCapData[59] = 0.000000; ui40->fFluxCapData[60] = 0.000000; ui40->fFluxCapData[61] = 0.000000; ui40->fFluxCapData[62] = 0.000000; ui40->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui40);
	delete ui40;


	m_uPluginType = 0;
	CUICtrl* ui41 = new CUICtrl;
	ui41->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui41->uControlId = 41;
	ui41->bLogSlider = false;
	ui41->bExpSlider = false;
	ui41->fUserDisplayDataLoLimit = 0.000000;
	ui41->fUserDisplayDataHiLimit = 3.000000;
	ui41->uUserDataType = UINTData;
	ui41->fInitUserIntValue = 0;
	ui41->fInitUserFloatValue = 0;
	ui41->fInitUserDoubleValue = 0;
	ui41->fInitUserUINTValue = 0.000000;
	ui41->m_pUserCookedIntData = NULL;
	ui41->m_pUserCookedFloatData = NULL;
	ui41->m_pUserCookedDoubleData = NULL;
	ui41->m_pUserCookedUINTData = &m_uPluginType;
	ui41->cControlUnits = "";
	ui41->cVariableName = "m_uPluginType";
	ui41->cEnumeratedList = "Envelope,Delay,Reverb,Trem";
	ui41->dPresetData[0] = -0.000000;ui41->dPresetData[1] = 0.000000;ui41->dPresetData[2] = 0.000000;ui41->dPresetData[3] = 0.000000;ui41->dPresetData[4] = 0.000000;ui41->dPresetData[5] = 0.000000;ui41->dPresetData[6] = 0.000000;ui41->dPresetData[7] = 0.000000;ui41->dPresetData[8] = 0.000000;ui41->dPresetData[9] = 0.000000;ui41->dPresetData[10] = 0.000000;ui41->dPresetData[11] = 0.000000;ui41->dPresetData[12] = 0.000000;ui41->dPresetData[13] = 0.000000;ui41->dPresetData[14] = 0.000000;ui41->dPresetData[15] = 0.000000;
	ui41->cControlName = "Plugin Type";
	ui41->bOwnerControl = false;
	ui41->bMIDIControl = false;
	ui41->uMIDIControlCommand = 176;
	ui41->uMIDIControlName = 3;
	ui41->uMIDIControlChannel = 0;
	ui41->nGUIRow = nIndexer++;
	ui41->nGUIColumn = -1;
	ui41->bEnableParamSmoothing = false;
	ui41->fSmoothingTimeInMs = 100.0;
	ui41->uControlTheme[0] = 0; ui41->uControlTheme[1] = 0; ui41->uControlTheme[2] = 0; ui41->uControlTheme[3] = 8355711; ui41->uControlTheme[4] = 139; ui41->uControlTheme[5] = 0; ui41->uControlTheme[6] = 0; ui41->uControlTheme[7] = 0; ui41->uControlTheme[8] = 0; ui41->uControlTheme[9] = 0; ui41->uControlTheme[10] = 0; ui41->uControlTheme[11] = 0; ui41->uControlTheme[12] = 0; ui41->uControlTheme[13] = 0; ui41->uControlTheme[14] = 0; ui41->uControlTheme[15] = 0; ui41->uControlTheme[16] = 0; ui41->uControlTheme[17] = 0; ui41->uControlTheme[18] = 0; ui41->uControlTheme[19] = 0; ui41->uControlTheme[20] = 0; ui41->uControlTheme[21] = 0; ui41->uControlTheme[22] = 0; ui41->uControlTheme[23] = 0; ui41->uControlTheme[24] = 0; ui41->uControlTheme[25] = 0; ui41->uControlTheme[26] = 0; ui41->uControlTheme[27] = 0; ui41->uControlTheme[28] = 0; ui41->uControlTheme[29] = 0; ui41->uControlTheme[30] = 0; ui41->uControlTheme[31] = 0; 
	ui41->uFluxCapControl[0] = 0; ui41->uFluxCapControl[1] = 0; ui41->uFluxCapControl[2] = 0; ui41->uFluxCapControl[3] = 0; ui41->uFluxCapControl[4] = 0; ui41->uFluxCapControl[5] = 0; ui41->uFluxCapControl[6] = 0; ui41->uFluxCapControl[7] = 0; ui41->uFluxCapControl[8] = 0; ui41->uFluxCapControl[9] = 0; ui41->uFluxCapControl[10] = 0; ui41->uFluxCapControl[11] = 0; ui41->uFluxCapControl[12] = 0; ui41->uFluxCapControl[13] = 0; ui41->uFluxCapControl[14] = 0; ui41->uFluxCapControl[15] = 0; ui41->uFluxCapControl[16] = 0; ui41->uFluxCapControl[17] = 0; ui41->uFluxCapControl[18] = 0; ui41->uFluxCapControl[19] = 0; ui41->uFluxCapControl[20] = 0; ui41->uFluxCapControl[21] = 0; ui41->uFluxCapControl[22] = 0; ui41->uFluxCapControl[23] = 0; ui41->uFluxCapControl[24] = 0; ui41->uFluxCapControl[25] = 0; ui41->uFluxCapControl[26] = 0; ui41->uFluxCapControl[27] = 0; ui41->uFluxCapControl[28] = 0; ui41->uFluxCapControl[29] = 0; ui41->uFluxCapControl[30] = 0; ui41->uFluxCapControl[31] = 0; ui41->uFluxCapControl[32] = 0; ui41->uFluxCapControl[33] = 0; ui41->uFluxCapControl[34] = 0; ui41->uFluxCapControl[35] = 0; ui41->uFluxCapControl[36] = 0; ui41->uFluxCapControl[37] = 0; ui41->uFluxCapControl[38] = 0; ui41->uFluxCapControl[39] = 0; ui41->uFluxCapControl[40] = 0; ui41->uFluxCapControl[41] = 0; ui41->uFluxCapControl[42] = 0; ui41->uFluxCapControl[43] = 0; ui41->uFluxCapControl[44] = 0; ui41->uFluxCapControl[45] = 0; ui41->uFluxCapControl[46] = 0; ui41->uFluxCapControl[47] = 0; ui41->uFluxCapControl[48] = 0; ui41->uFluxCapControl[49] = 0; ui41->uFluxCapControl[50] = 0; ui41->uFluxCapControl[51] = 0; ui41->uFluxCapControl[52] = 0; ui41->uFluxCapControl[53] = 0; ui41->uFluxCapControl[54] = 0; ui41->uFluxCapControl[55] = 0; ui41->uFluxCapControl[56] = 0; ui41->uFluxCapControl[57] = 0; ui41->uFluxCapControl[58] = 0; ui41->uFluxCapControl[59] = 0; ui41->uFluxCapControl[60] = 0; ui41->uFluxCapControl[61] = 0; ui41->uFluxCapControl[62] = 0; ui41->uFluxCapControl[63] = 0; 
	ui41->fFluxCapData[0] = 0.000000; ui41->fFluxCapData[1] = 0.000000; ui41->fFluxCapData[2] = 0.000000; ui41->fFluxCapData[3] = 0.000000; ui41->fFluxCapData[4] = 0.000000; ui41->fFluxCapData[5] = 0.000000; ui41->fFluxCapData[6] = 0.000000; ui41->fFluxCapData[7] = 0.000000; ui41->fFluxCapData[8] = 0.000000; ui41->fFluxCapData[9] = 0.000000; ui41->fFluxCapData[10] = 0.000000; ui41->fFluxCapData[11] = 0.000000; ui41->fFluxCapData[12] = 0.000000; ui41->fFluxCapData[13] = 0.000000; ui41->fFluxCapData[14] = 0.000000; ui41->fFluxCapData[15] = 0.000000; ui41->fFluxCapData[16] = 0.000000; ui41->fFluxCapData[17] = 0.000000; ui41->fFluxCapData[18] = 0.000000; ui41->fFluxCapData[19] = 0.000000; ui41->fFluxCapData[20] = 0.000000; ui41->fFluxCapData[21] = 0.000000; ui41->fFluxCapData[22] = 0.000000; ui41->fFluxCapData[23] = 0.000000; ui41->fFluxCapData[24] = 0.000000; ui41->fFluxCapData[25] = 0.000000; ui41->fFluxCapData[26] = 0.000000; ui41->fFluxCapData[27] = 0.000000; ui41->fFluxCapData[28] = 0.000000; ui41->fFluxCapData[29] = 0.000000; ui41->fFluxCapData[30] = 0.000000; ui41->fFluxCapData[31] = 0.000000; ui41->fFluxCapData[32] = 0.000000; ui41->fFluxCapData[33] = 0.000000; ui41->fFluxCapData[34] = 0.000000; ui41->fFluxCapData[35] = 0.000000; ui41->fFluxCapData[36] = 0.000000; ui41->fFluxCapData[37] = 0.000000; ui41->fFluxCapData[38] = 0.000000; ui41->fFluxCapData[39] = 0.000000; ui41->fFluxCapData[40] = 0.000000; ui41->fFluxCapData[41] = 0.000000; ui41->fFluxCapData[42] = 0.000000; ui41->fFluxCapData[43] = 0.000000; ui41->fFluxCapData[44] = 0.000000; ui41->fFluxCapData[45] = 0.000000; ui41->fFluxCapData[46] = 0.000000; ui41->fFluxCapData[47] = 0.000000; ui41->fFluxCapData[48] = 0.000000; ui41->fFluxCapData[49] = 0.000000; ui41->fFluxCapData[50] = 0.000000; ui41->fFluxCapData[51] = 0.000000; ui41->fFluxCapData[52] = 0.000000; ui41->fFluxCapData[53] = 0.000000; ui41->fFluxCapData[54] = 0.000000; ui41->fFluxCapData[55] = 0.000000; ui41->fFluxCapData[56] = 0.000000; ui41->fFluxCapData[57] = 0.000000; ui41->fFluxCapData[58] = 0.000000; ui41->fFluxCapData[59] = 0.000000; ui41->fFluxCapData[60] = 0.000000; ui41->fFluxCapData[61] = 0.000000; ui41->fFluxCapData[62] = 0.000000; ui41->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui41);
	delete ui41;


	m_uLFOType = 0;
	CUICtrl* ui42 = new CUICtrl;
	ui42->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui42->uControlId = 2048;
	ui42->bLogSlider = false;
	ui42->bExpSlider = false;
	ui42->fUserDisplayDataLoLimit = 0.000000;
	ui42->fUserDisplayDataHiLimit = 3.000000;
	ui42->uUserDataType = UINTData;
	ui42->fInitUserIntValue = 0;
	ui42->fInitUserFloatValue = 0;
	ui42->fInitUserDoubleValue = 0;
	ui42->fInitUserUINTValue = 0.000000;
	ui42->m_pUserCookedIntData = NULL;
	ui42->m_pUserCookedFloatData = NULL;
	ui42->m_pUserCookedDoubleData = NULL;
	ui42->m_pUserCookedUINTData = &m_uLFOType;
	ui42->cControlUnits = "";
	ui42->cVariableName = "m_uLFOType";
	ui42->cEnumeratedList = "TRI,SINE,SQUARE,SAW";
	ui42->dPresetData[0] = -0.000000;ui42->dPresetData[1] = 0.000000;ui42->dPresetData[2] = 0.000000;ui42->dPresetData[3] = 0.000000;ui42->dPresetData[4] = 0.000000;ui42->dPresetData[5] = 0.000000;ui42->dPresetData[6] = 0.000000;ui42->dPresetData[7] = 0.000000;ui42->dPresetData[8] = 0.000000;ui42->dPresetData[9] = 0.000000;ui42->dPresetData[10] = 0.000000;ui42->dPresetData[11] = 0.000000;ui42->dPresetData[12] = 0.000000;ui42->dPresetData[13] = 0.000000;ui42->dPresetData[14] = 0.000000;ui42->dPresetData[15] = 0.000000;
	ui42->cControlName = "LFO";
	ui42->bOwnerControl = false;
	ui42->bMIDIControl = false;
	ui42->uMIDIControlCommand = 176;
	ui42->uMIDIControlName = 3;
	ui42->uMIDIControlChannel = 0;
	ui42->nGUIRow = nIndexer++;
	ui42->nGUIColumn = -1;
	ui42->bEnableParamSmoothing = false;
	ui42->fSmoothingTimeInMs = 100.0;
	ui42->uControlTheme[0] = 0; ui42->uControlTheme[1] = 0; ui42->uControlTheme[2] = 0; ui42->uControlTheme[3] = 8355711; ui42->uControlTheme[4] = 139; ui42->uControlTheme[5] = 0; ui42->uControlTheme[6] = 0; ui42->uControlTheme[7] = 0; ui42->uControlTheme[8] = 0; ui42->uControlTheme[9] = 0; ui42->uControlTheme[10] = 0; ui42->uControlTheme[11] = 0; ui42->uControlTheme[12] = 0; ui42->uControlTheme[13] = 0; ui42->uControlTheme[14] = 0; ui42->uControlTheme[15] = 0; ui42->uControlTheme[16] = 0; ui42->uControlTheme[17] = 0; ui42->uControlTheme[18] = 0; ui42->uControlTheme[19] = 0; ui42->uControlTheme[20] = 0; ui42->uControlTheme[21] = 0; ui42->uControlTheme[22] = 0; ui42->uControlTheme[23] = 0; ui42->uControlTheme[24] = 0; ui42->uControlTheme[25] = 0; ui42->uControlTheme[26] = 0; ui42->uControlTheme[27] = 0; ui42->uControlTheme[28] = 0; ui42->uControlTheme[29] = 0; ui42->uControlTheme[30] = 0; ui42->uControlTheme[31] = 0; 
	ui42->uFluxCapControl[0] = 0; ui42->uFluxCapControl[1] = 0; ui42->uFluxCapControl[2] = 0; ui42->uFluxCapControl[3] = 0; ui42->uFluxCapControl[4] = 0; ui42->uFluxCapControl[5] = 0; ui42->uFluxCapControl[6] = 0; ui42->uFluxCapControl[7] = 0; ui42->uFluxCapControl[8] = 0; ui42->uFluxCapControl[9] = 0; ui42->uFluxCapControl[10] = 0; ui42->uFluxCapControl[11] = 0; ui42->uFluxCapControl[12] = 0; ui42->uFluxCapControl[13] = 0; ui42->uFluxCapControl[14] = 0; ui42->uFluxCapControl[15] = 0; ui42->uFluxCapControl[16] = 0; ui42->uFluxCapControl[17] = 0; ui42->uFluxCapControl[18] = 0; ui42->uFluxCapControl[19] = 0; ui42->uFluxCapControl[20] = 0; ui42->uFluxCapControl[21] = 0; ui42->uFluxCapControl[22] = 0; ui42->uFluxCapControl[23] = 0; ui42->uFluxCapControl[24] = 0; ui42->uFluxCapControl[25] = 0; ui42->uFluxCapControl[26] = 0; ui42->uFluxCapControl[27] = 0; ui42->uFluxCapControl[28] = 0; ui42->uFluxCapControl[29] = 0; ui42->uFluxCapControl[30] = 0; ui42->uFluxCapControl[31] = 0; ui42->uFluxCapControl[32] = 0; ui42->uFluxCapControl[33] = 0; ui42->uFluxCapControl[34] = 0; ui42->uFluxCapControl[35] = 0; ui42->uFluxCapControl[36] = 0; ui42->uFluxCapControl[37] = 0; ui42->uFluxCapControl[38] = 0; ui42->uFluxCapControl[39] = 0; ui42->uFluxCapControl[40] = 0; ui42->uFluxCapControl[41] = 0; ui42->uFluxCapControl[42] = 0; ui42->uFluxCapControl[43] = 0; ui42->uFluxCapControl[44] = 0; ui42->uFluxCapControl[45] = 0; ui42->uFluxCapControl[46] = 0; ui42->uFluxCapControl[47] = 0; ui42->uFluxCapControl[48] = 0; ui42->uFluxCapControl[49] = 0; ui42->uFluxCapControl[50] = 0; ui42->uFluxCapControl[51] = 0; ui42->uFluxCapControl[52] = 0; ui42->uFluxCapControl[53] = 0; ui42->uFluxCapControl[54] = 0; ui42->uFluxCapControl[55] = 0; ui42->uFluxCapControl[56] = 0; ui42->uFluxCapControl[57] = 0; ui42->uFluxCapControl[58] = 0; ui42->uFluxCapControl[59] = 0; ui42->uFluxCapControl[60] = 0; ui42->uFluxCapControl[61] = 0; ui42->uFluxCapControl[62] = 0; ui42->uFluxCapControl[63] = 0; 
	ui42->fFluxCapData[0] = 0.000000; ui42->fFluxCapData[1] = 0.000000; ui42->fFluxCapData[2] = 0.000000; ui42->fFluxCapData[3] = 0.000000; ui42->fFluxCapData[4] = 0.000000; ui42->fFluxCapData[5] = 0.000000; ui42->fFluxCapData[6] = 0.000000; ui42->fFluxCapData[7] = 0.000000; ui42->fFluxCapData[8] = 0.000000; ui42->fFluxCapData[9] = 0.000000; ui42->fFluxCapData[10] = 0.000000; ui42->fFluxCapData[11] = 0.000000; ui42->fFluxCapData[12] = 0.000000; ui42->fFluxCapData[13] = 0.000000; ui42->fFluxCapData[14] = 0.000000; ui42->fFluxCapData[15] = 0.000000; ui42->fFluxCapData[16] = 0.000000; ui42->fFluxCapData[17] = 0.000000; ui42->fFluxCapData[18] = 0.000000; ui42->fFluxCapData[19] = 0.000000; ui42->fFluxCapData[20] = 0.000000; ui42->fFluxCapData[21] = 0.000000; ui42->fFluxCapData[22] = 0.000000; ui42->fFluxCapData[23] = 0.000000; ui42->fFluxCapData[24] = 0.000000; ui42->fFluxCapData[25] = 0.000000; ui42->fFluxCapData[26] = 0.000000; ui42->fFluxCapData[27] = 0.000000; ui42->fFluxCapData[28] = 0.000000; ui42->fFluxCapData[29] = 0.000000; ui42->fFluxCapData[30] = 0.000000; ui42->fFluxCapData[31] = 0.000000; ui42->fFluxCapData[32] = 0.000000; ui42->fFluxCapData[33] = 0.000000; ui42->fFluxCapData[34] = 0.000000; ui42->fFluxCapData[35] = 0.000000; ui42->fFluxCapData[36] = 0.000000; ui42->fFluxCapData[37] = 0.000000; ui42->fFluxCapData[38] = 0.000000; ui42->fFluxCapData[39] = 0.000000; ui42->fFluxCapData[40] = 0.000000; ui42->fFluxCapData[41] = 0.000000; ui42->fFluxCapData[42] = 0.000000; ui42->fFluxCapData[43] = 0.000000; ui42->fFluxCapData[44] = 0.000000; ui42->fFluxCapData[45] = 0.000000; ui42->fFluxCapData[46] = 0.000000; ui42->fFluxCapData[47] = 0.000000; ui42->fFluxCapData[48] = 0.000000; ui42->fFluxCapData[49] = 0.000000; ui42->fFluxCapData[50] = 0.000000; ui42->fFluxCapData[51] = 0.000000; ui42->fFluxCapData[52] = 0.000000; ui42->fFluxCapData[53] = 0.000000; ui42->fFluxCapData[54] = 0.000000; ui42->fFluxCapData[55] = 0.000000; ui42->fFluxCapData[56] = 0.000000; ui42->fFluxCapData[57] = 0.000000; ui42->fFluxCapData[58] = 0.000000; ui42->fFluxCapData[59] = 0.000000; ui42->fFluxCapData[60] = 0.000000; ui42->fFluxCapData[61] = 0.000000; ui42->fFluxCapData[62] = 0.000000; ui42->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui42);
	delete ui42;


	m_uMode = 0;
	CUICtrl* ui43 = new CUICtrl;
	ui43->uControlType = FILTER_CONTROL_RADIO_SWITCH_VARIABLE;
	ui43->uControlId = 2049;
	ui43->bLogSlider = false;
	ui43->bExpSlider = false;
	ui43->fUserDisplayDataLoLimit = 0.000000;
	ui43->fUserDisplayDataHiLimit = 1.000000;
	ui43->uUserDataType = UINTData;
	ui43->fInitUserIntValue = 0;
	ui43->fInitUserFloatValue = 0;
	ui43->fInitUserDoubleValue = 0;
	ui43->fInitUserUINTValue = 0.000000;
	ui43->m_pUserCookedIntData = NULL;
	ui43->m_pUserCookedFloatData = NULL;
	ui43->m_pUserCookedDoubleData = NULL;
	ui43->m_pUserCookedUINTData = &m_uMode;
	ui43->cControlUnits = "";
	ui43->cVariableName = "m_uMode";
	ui43->cEnumeratedList = "Tremolo,Panner";
	ui43->dPresetData[0] = -0.000000;ui43->dPresetData[1] = 0.000000;ui43->dPresetData[2] = 0.000000;ui43->dPresetData[3] = 0.000000;ui43->dPresetData[4] = 0.000000;ui43->dPresetData[5] = 0.000000;ui43->dPresetData[6] = 0.000000;ui43->dPresetData[7] = 0.000000;ui43->dPresetData[8] = 0.000000;ui43->dPresetData[9] = 0.000000;ui43->dPresetData[10] = 0.000000;ui43->dPresetData[11] = 0.000000;ui43->dPresetData[12] = 0.000000;ui43->dPresetData[13] = 0.000000;ui43->dPresetData[14] = 0.000000;ui43->dPresetData[15] = 0.000000;
	ui43->cControlName = "Tremolo Mode";
	ui43->bOwnerControl = false;
	ui43->bMIDIControl = false;
	ui43->uMIDIControlCommand = 176;
	ui43->uMIDIControlName = 3;
	ui43->uMIDIControlChannel = 0;
	ui43->nGUIRow = nIndexer++;
	ui43->nGUIColumn = -1;
	ui43->bEnableParamSmoothing = false;
	ui43->fSmoothingTimeInMs = 100.0;
	ui43->uControlTheme[0] = 0; ui43->uControlTheme[1] = 0; ui43->uControlTheme[2] = 0; ui43->uControlTheme[3] = 8355711; ui43->uControlTheme[4] = 139; ui43->uControlTheme[5] = 0; ui43->uControlTheme[6] = 0; ui43->uControlTheme[7] = 0; ui43->uControlTheme[8] = 0; ui43->uControlTheme[9] = 0; ui43->uControlTheme[10] = 0; ui43->uControlTheme[11] = 0; ui43->uControlTheme[12] = 0; ui43->uControlTheme[13] = 0; ui43->uControlTheme[14] = 0; ui43->uControlTheme[15] = 0; ui43->uControlTheme[16] = 0; ui43->uControlTheme[17] = 0; ui43->uControlTheme[18] = 0; ui43->uControlTheme[19] = 0; ui43->uControlTheme[20] = 0; ui43->uControlTheme[21] = 0; ui43->uControlTheme[22] = 0; ui43->uControlTheme[23] = 0; ui43->uControlTheme[24] = 0; ui43->uControlTheme[25] = 0; ui43->uControlTheme[26] = 0; ui43->uControlTheme[27] = 0; ui43->uControlTheme[28] = 0; ui43->uControlTheme[29] = 0; ui43->uControlTheme[30] = 0; ui43->uControlTheme[31] = 0; 
	ui43->uFluxCapControl[0] = 0; ui43->uFluxCapControl[1] = 0; ui43->uFluxCapControl[2] = 0; ui43->uFluxCapControl[3] = 0; ui43->uFluxCapControl[4] = 0; ui43->uFluxCapControl[5] = 0; ui43->uFluxCapControl[6] = 0; ui43->uFluxCapControl[7] = 0; ui43->uFluxCapControl[8] = 0; ui43->uFluxCapControl[9] = 0; ui43->uFluxCapControl[10] = 0; ui43->uFluxCapControl[11] = 0; ui43->uFluxCapControl[12] = 0; ui43->uFluxCapControl[13] = 0; ui43->uFluxCapControl[14] = 0; ui43->uFluxCapControl[15] = 0; ui43->uFluxCapControl[16] = 0; ui43->uFluxCapControl[17] = 0; ui43->uFluxCapControl[18] = 0; ui43->uFluxCapControl[19] = 0; ui43->uFluxCapControl[20] = 0; ui43->uFluxCapControl[21] = 0; ui43->uFluxCapControl[22] = 0; ui43->uFluxCapControl[23] = 0; ui43->uFluxCapControl[24] = 0; ui43->uFluxCapControl[25] = 0; ui43->uFluxCapControl[26] = 0; ui43->uFluxCapControl[27] = 0; ui43->uFluxCapControl[28] = 0; ui43->uFluxCapControl[29] = 0; ui43->uFluxCapControl[30] = 0; ui43->uFluxCapControl[31] = 0; ui43->uFluxCapControl[32] = 0; ui43->uFluxCapControl[33] = 0; ui43->uFluxCapControl[34] = 0; ui43->uFluxCapControl[35] = 0; ui43->uFluxCapControl[36] = 0; ui43->uFluxCapControl[37] = 0; ui43->uFluxCapControl[38] = 0; ui43->uFluxCapControl[39] = 0; ui43->uFluxCapControl[40] = 0; ui43->uFluxCapControl[41] = 0; ui43->uFluxCapControl[42] = 0; ui43->uFluxCapControl[43] = 0; ui43->uFluxCapControl[44] = 0; ui43->uFluxCapControl[45] = 0; ui43->uFluxCapControl[46] = 0; ui43->uFluxCapControl[47] = 0; ui43->uFluxCapControl[48] = 0; ui43->uFluxCapControl[49] = 0; ui43->uFluxCapControl[50] = 0; ui43->uFluxCapControl[51] = 0; ui43->uFluxCapControl[52] = 0; ui43->uFluxCapControl[53] = 0; ui43->uFluxCapControl[54] = 0; ui43->uFluxCapControl[55] = 0; ui43->uFluxCapControl[56] = 0; ui43->uFluxCapControl[57] = 0; ui43->uFluxCapControl[58] = 0; ui43->uFluxCapControl[59] = 0; ui43->uFluxCapControl[60] = 0; ui43->uFluxCapControl[61] = 0; ui43->uFluxCapControl[62] = 0; ui43->uFluxCapControl[63] = 0; 
	ui43->fFluxCapData[0] = 0.000000; ui43->fFluxCapData[1] = 0.000000; ui43->fFluxCapData[2] = 0.000000; ui43->fFluxCapData[3] = 0.000000; ui43->fFluxCapData[4] = 0.000000; ui43->fFluxCapData[5] = 0.000000; ui43->fFluxCapData[6] = 0.000000; ui43->fFluxCapData[7] = 0.000000; ui43->fFluxCapData[8] = 0.000000; ui43->fFluxCapData[9] = 0.000000; ui43->fFluxCapData[10] = 0.000000; ui43->fFluxCapData[11] = 0.000000; ui43->fFluxCapData[12] = 0.000000; ui43->fFluxCapData[13] = 0.000000; ui43->fFluxCapData[14] = 0.000000; ui43->fFluxCapData[15] = 0.000000; ui43->fFluxCapData[16] = 0.000000; ui43->fFluxCapData[17] = 0.000000; ui43->fFluxCapData[18] = 0.000000; ui43->fFluxCapData[19] = 0.000000; ui43->fFluxCapData[20] = 0.000000; ui43->fFluxCapData[21] = 0.000000; ui43->fFluxCapData[22] = 0.000000; ui43->fFluxCapData[23] = 0.000000; ui43->fFluxCapData[24] = 0.000000; ui43->fFluxCapData[25] = 0.000000; ui43->fFluxCapData[26] = 0.000000; ui43->fFluxCapData[27] = 0.000000; ui43->fFluxCapData[28] = 0.000000; ui43->fFluxCapData[29] = 0.000000; ui43->fFluxCapData[30] = 0.000000; ui43->fFluxCapData[31] = 0.000000; ui43->fFluxCapData[32] = 0.000000; ui43->fFluxCapData[33] = 0.000000; ui43->fFluxCapData[34] = 0.000000; ui43->fFluxCapData[35] = 0.000000; ui43->fFluxCapData[36] = 0.000000; ui43->fFluxCapData[37] = 0.000000; ui43->fFluxCapData[38] = 0.000000; ui43->fFluxCapData[39] = 0.000000; ui43->fFluxCapData[40] = 0.000000; ui43->fFluxCapData[41] = 0.000000; ui43->fFluxCapData[42] = 0.000000; ui43->fFluxCapData[43] = 0.000000; ui43->fFluxCapData[44] = 0.000000; ui43->fFluxCapData[45] = 0.000000; ui43->fFluxCapData[46] = 0.000000; ui43->fFluxCapData[47] = 0.000000; ui43->fFluxCapData[48] = 0.000000; ui43->fFluxCapData[49] = 0.000000; ui43->fFluxCapData[50] = 0.000000; ui43->fFluxCapData[51] = 0.000000; ui43->fFluxCapData[52] = 0.000000; ui43->fFluxCapData[53] = 0.000000; ui43->fFluxCapData[54] = 0.000000; ui43->fFluxCapData[55] = 0.000000; ui43->fFluxCapData[56] = 0.000000; ui43->fFluxCapData[57] = 0.000000; ui43->fFluxCapData[58] = 0.000000; ui43->fFluxCapData[59] = 0.000000; ui43->fFluxCapData[60] = 0.000000; ui43->fFluxCapData[61] = 0.000000; ui43->fFluxCapData[62] = 0.000000; ui43->fFluxCapData[63] = 0.000000; 
	m_UIControlList.append(*ui43);
	delete ui43;


	m_uX_TrackPadIndex = -1;
	m_uY_TrackPadIndex = -1;

	m_AssignButton1Name = "B1";
	m_AssignButton2Name = "B2";
	m_AssignButton3Name = "B3";

	m_bLatchingAssignButton1 = false;
	m_bLatchingAssignButton2 = false;
	m_bLatchingAssignButton3 = false;

	m_nGUIType = -1;
	m_nGUIThemeID = -1;
	m_bUseCustomVSTGUI = false;

	m_uControlTheme[0] = 0; m_uControlTheme[1] = 0; m_uControlTheme[2] = 0; m_uControlTheme[3] = 0; m_uControlTheme[4] = 0; m_uControlTheme[5] = 0; m_uControlTheme[6] = 0; m_uControlTheme[7] = 0; m_uControlTheme[8] = 0; m_uControlTheme[9] = 0; m_uControlTheme[10] = 0; m_uControlTheme[11] = 0; m_uControlTheme[12] = 0; m_uControlTheme[13] = 0; m_uControlTheme[14] = 0; m_uControlTheme[15] = 0; m_uControlTheme[16] = 0; m_uControlTheme[17] = 0; m_uControlTheme[18] = 0; m_uControlTheme[19] = 0; m_uControlTheme[20] = 0; m_uControlTheme[21] = 0; m_uControlTheme[22] = 0; m_uControlTheme[23] = 0; m_uControlTheme[24] = 0; m_uControlTheme[25] = 0; m_uControlTheme[26] = 0; m_uControlTheme[27] = 0; m_uControlTheme[28] = 0; m_uControlTheme[29] = 0; m_uControlTheme[30] = 0; m_uControlTheme[31] = 0; m_uControlTheme[32] = 0; m_uControlTheme[33] = 0; m_uControlTheme[34] = 0; m_uControlTheme[35] = 0; m_uControlTheme[36] = 0; m_uControlTheme[37] = 0; m_uControlTheme[38] = 0; m_uControlTheme[39] = 0; m_uControlTheme[40] = 0; m_uControlTheme[41] = 0; m_uControlTheme[42] = 0; m_uControlTheme[43] = 0; m_uControlTheme[44] = 0; m_uControlTheme[45] = 0; m_uControlTheme[46] = 0; m_uControlTheme[47] = 0; m_uControlTheme[48] = 0; m_uControlTheme[49] = 0; m_uControlTheme[50] = 0; m_uControlTheme[51] = 0; m_uControlTheme[52] = 0; m_uControlTheme[53] = 0; m_uControlTheme[54] = 0; m_uControlTheme[55] = 0; m_uControlTheme[56] = 0; m_uControlTheme[57] = 0; m_uControlTheme[58] = 0; m_uControlTheme[59] = 0; m_uControlTheme[60] = 0; m_uControlTheme[61] = 0; m_uControlTheme[62] = 0; m_uControlTheme[63] = 0; 

	m_uPlugInEx[0] = 6813; m_uPlugInEx[1] = 430; m_uPlugInEx[2] = 2; m_uPlugInEx[3] = 0; m_uPlugInEx[4] = 0; m_uPlugInEx[5] = 0; m_uPlugInEx[6] = 1; m_uPlugInEx[7] = 0; m_uPlugInEx[8] = 0; m_uPlugInEx[9] = 0; m_uPlugInEx[10] = 0; m_uPlugInEx[11] = 0; m_uPlugInEx[12] = 0; m_uPlugInEx[13] = 0; m_uPlugInEx[14] = 0; m_uPlugInEx[15] = 0; m_uPlugInEx[16] = 0; m_uPlugInEx[17] = 0; m_uPlugInEx[18] = 0; m_uPlugInEx[19] = 0; m_uPlugInEx[20] = 0; m_uPlugInEx[21] = 0; m_uPlugInEx[22] = 0; m_uPlugInEx[23] = 0; m_uPlugInEx[24] = 0; m_uPlugInEx[25] = 0; m_uPlugInEx[26] = 0; m_uPlugInEx[27] = 0; m_uPlugInEx[28] = 0; m_uPlugInEx[29] = 0; m_uPlugInEx[30] = 0; m_uPlugInEx[31] = 0; m_uPlugInEx[32] = 0; m_uPlugInEx[33] = 0; m_uPlugInEx[34] = 0; m_uPlugInEx[35] = 0; m_uPlugInEx[36] = 0; m_uPlugInEx[37] = 0; m_uPlugInEx[38] = 0; m_uPlugInEx[39] = 0; m_uPlugInEx[40] = 0; m_uPlugInEx[41] = 0; m_uPlugInEx[42] = 0; m_uPlugInEx[43] = 0; m_uPlugInEx[44] = 0; m_uPlugInEx[45] = 0; m_uPlugInEx[46] = 0; m_uPlugInEx[47] = 0; m_uPlugInEx[48] = 0; m_uPlugInEx[49] = 0; m_uPlugInEx[50] = 0; m_uPlugInEx[51] = 0; m_uPlugInEx[52] = 0; m_uPlugInEx[53] = 0; m_uPlugInEx[54] = 0; m_uPlugInEx[55] = 0; m_uPlugInEx[56] = 0; m_uPlugInEx[57] = 0; m_uPlugInEx[58] = 0; m_uPlugInEx[59] = 0; m_uPlugInEx[60] = 0; m_uPlugInEx[61] = 0; m_uPlugInEx[62] = 0; m_uPlugInEx[63] = 0; 
	m_fPlugInEx[0] = 0.000000; m_fPlugInEx[1] = 500.000000; m_fPlugInEx[2] = 0.000000; m_fPlugInEx[3] = 0.000000; m_fPlugInEx[4] = 0.000000; m_fPlugInEx[5] = 0.000000; m_fPlugInEx[6] = 0.000000; m_fPlugInEx[7] = 0.000000; m_fPlugInEx[8] = 0.000000; m_fPlugInEx[9] = 0.000000; m_fPlugInEx[10] = 0.000000; m_fPlugInEx[11] = 0.000000; m_fPlugInEx[12] = 0.000000; m_fPlugInEx[13] = 0.000000; m_fPlugInEx[14] = 0.000000; m_fPlugInEx[15] = 0.000000; m_fPlugInEx[16] = 0.000000; m_fPlugInEx[17] = 0.000000; m_fPlugInEx[18] = 0.000000; m_fPlugInEx[19] = 0.000000; m_fPlugInEx[20] = 0.000000; m_fPlugInEx[21] = 0.000000; m_fPlugInEx[22] = 0.000000; m_fPlugInEx[23] = 0.000000; m_fPlugInEx[24] = 0.000000; m_fPlugInEx[25] = 0.000000; m_fPlugInEx[26] = 0.000000; m_fPlugInEx[27] = 0.000000; m_fPlugInEx[28] = 0.000000; m_fPlugInEx[29] = 0.000000; m_fPlugInEx[30] = 0.000000; m_fPlugInEx[31] = 0.000000; m_fPlugInEx[32] = 0.000000; m_fPlugInEx[33] = 0.000000; m_fPlugInEx[34] = 0.000000; m_fPlugInEx[35] = 0.000000; m_fPlugInEx[36] = 0.000000; m_fPlugInEx[37] = 0.000000; m_fPlugInEx[38] = 0.000000; m_fPlugInEx[39] = 0.000000; m_fPlugInEx[40] = 0.000000; m_fPlugInEx[41] = 0.000000; m_fPlugInEx[42] = 0.000000; m_fPlugInEx[43] = 0.000000; m_fPlugInEx[44] = 0.000000; m_fPlugInEx[45] = 0.000000; m_fPlugInEx[46] = 0.000000; m_fPlugInEx[47] = 0.000000; m_fPlugInEx[48] = 0.000000; m_fPlugInEx[49] = 0.000000; m_fPlugInEx[50] = 0.000000; m_fPlugInEx[51] = 0.000000; m_fPlugInEx[52] = 0.000000; m_fPlugInEx[53] = 0.000000; m_fPlugInEx[54] = 0.000000; m_fPlugInEx[55] = 0.000000; m_fPlugInEx[56] = 0.000000; m_fPlugInEx[57] = 0.000000; m_fPlugInEx[58] = 0.000000; m_fPlugInEx[59] = 0.000000; m_fPlugInEx[60] = 0.000000; m_fPlugInEx[61] = 0.000000; m_fPlugInEx[62] = 0.000000; m_fPlugInEx[63] = 0.000000; 

	m_TextLabels[0] = ""; m_TextLabels[1] = ""; m_TextLabels[2] = ""; m_TextLabels[3] = ""; m_TextLabels[4] = ""; m_TextLabels[5] = ""; m_TextLabels[6] = ""; m_TextLabels[7] = ""; m_TextLabels[8] = ""; m_TextLabels[9] = ""; m_TextLabels[10] = ""; m_TextLabels[11] = ""; m_TextLabels[12] = ""; m_TextLabels[13] = ""; m_TextLabels[14] = ""; m_TextLabels[15] = ""; m_TextLabels[16] = ""; m_TextLabels[17] = ""; m_TextLabels[18] = ""; m_TextLabels[19] = ""; m_TextLabels[20] = ""; m_TextLabels[21] = ""; m_TextLabels[22] = ""; m_TextLabels[23] = ""; m_TextLabels[24] = ""; m_TextLabels[25] = ""; m_TextLabels[26] = ""; m_TextLabels[27] = ""; m_TextLabels[28] = ""; m_TextLabels[29] = ""; m_TextLabels[30] = ""; m_TextLabels[31] = ""; m_TextLabels[32] = ""; m_TextLabels[33] = ""; m_TextLabels[34] = ""; m_TextLabels[35] = ""; m_TextLabels[36] = ""; m_TextLabels[37] = ""; m_TextLabels[38] = ""; m_TextLabels[39] = ""; m_TextLabels[40] = ""; m_TextLabels[41] = ""; m_TextLabels[42] = ""; m_TextLabels[43] = ""; m_TextLabels[44] = ""; m_TextLabels[45] = ""; m_TextLabels[46] = ""; m_TextLabels[47] = ""; m_TextLabels[48] = ""; m_TextLabels[49] = ""; m_TextLabels[50] = ""; m_TextLabels[51] = ""; m_TextLabels[52] = ""; m_TextLabels[53] = ""; m_TextLabels[54] = ""; m_TextLabels[55] = ""; m_TextLabels[56] = ""; m_TextLabels[57] = ""; m_TextLabels[58] = ""; m_TextLabels[59] = ""; m_TextLabels[60] = ""; m_TextLabels[61] = ""; m_TextLabels[62] = ""; m_TextLabels[63] = ""; 

	m_uLabelCX[0] = 0; m_uLabelCX[1] = 0; m_uLabelCX[2] = 0; m_uLabelCX[3] = 0; m_uLabelCX[4] = 0; m_uLabelCX[5] = 0; m_uLabelCX[6] = 0; m_uLabelCX[7] = 0; m_uLabelCX[8] = 0; m_uLabelCX[9] = 0; m_uLabelCX[10] = 0; m_uLabelCX[11] = 0; m_uLabelCX[12] = 0; m_uLabelCX[13] = 0; m_uLabelCX[14] = 0; m_uLabelCX[15] = 0; m_uLabelCX[16] = 0; m_uLabelCX[17] = 0; m_uLabelCX[18] = 0; m_uLabelCX[19] = 0; m_uLabelCX[20] = 0; m_uLabelCX[21] = 0; m_uLabelCX[22] = 0; m_uLabelCX[23] = 0; m_uLabelCX[24] = 0; m_uLabelCX[25] = 0; m_uLabelCX[26] = 0; m_uLabelCX[27] = 0; m_uLabelCX[28] = 0; m_uLabelCX[29] = 0; m_uLabelCX[30] = 0; m_uLabelCX[31] = 0; m_uLabelCX[32] = 0; m_uLabelCX[33] = 0; m_uLabelCX[34] = 0; m_uLabelCX[35] = 0; m_uLabelCX[36] = 0; m_uLabelCX[37] = 0; m_uLabelCX[38] = 0; m_uLabelCX[39] = 0; m_uLabelCX[40] = 0; m_uLabelCX[41] = 0; m_uLabelCX[42] = 0; m_uLabelCX[43] = 0; m_uLabelCX[44] = 0; m_uLabelCX[45] = 0; m_uLabelCX[46] = 0; m_uLabelCX[47] = 0; m_uLabelCX[48] = 0; m_uLabelCX[49] = 0; m_uLabelCX[50] = 0; m_uLabelCX[51] = 0; m_uLabelCX[52] = 0; m_uLabelCX[53] = 0; m_uLabelCX[54] = 0; m_uLabelCX[55] = 0; m_uLabelCX[56] = 0; m_uLabelCX[57] = 0; m_uLabelCX[58] = 0; m_uLabelCX[59] = 0; m_uLabelCX[60] = 0; m_uLabelCX[61] = 0; m_uLabelCX[62] = 0; m_uLabelCX[63] = 0; 
	m_uLabelCY[0] = 0; m_uLabelCY[1] = 0; m_uLabelCY[2] = 0; m_uLabelCY[3] = 0; m_uLabelCY[4] = 0; m_uLabelCY[5] = 0; m_uLabelCY[6] = 0; m_uLabelCY[7] = 0; m_uLabelCY[8] = 0; m_uLabelCY[9] = 0; m_uLabelCY[10] = 0; m_uLabelCY[11] = 0; m_uLabelCY[12] = 0; m_uLabelCY[13] = 0; m_uLabelCY[14] = 0; m_uLabelCY[15] = 0; m_uLabelCY[16] = 0; m_uLabelCY[17] = 0; m_uLabelCY[18] = 0; m_uLabelCY[19] = 0; m_uLabelCY[20] = 0; m_uLabelCY[21] = 0; m_uLabelCY[22] = 0; m_uLabelCY[23] = 0; m_uLabelCY[24] = 0; m_uLabelCY[25] = 0; m_uLabelCY[26] = 0; m_uLabelCY[27] = 0; m_uLabelCY[28] = 0; m_uLabelCY[29] = 0; m_uLabelCY[30] = 0; m_uLabelCY[31] = 0; m_uLabelCY[32] = 0; m_uLabelCY[33] = 0; m_uLabelCY[34] = 0; m_uLabelCY[35] = 0; m_uLabelCY[36] = 0; m_uLabelCY[37] = 0; m_uLabelCY[38] = 0; m_uLabelCY[39] = 0; m_uLabelCY[40] = 0; m_uLabelCY[41] = 0; m_uLabelCY[42] = 0; m_uLabelCY[43] = 0; m_uLabelCY[44] = 0; m_uLabelCY[45] = 0; m_uLabelCY[46] = 0; m_uLabelCY[47] = 0; m_uLabelCY[48] = 0; m_uLabelCY[49] = 0; m_uLabelCY[50] = 0; m_uLabelCY[51] = 0; m_uLabelCY[52] = 0; m_uLabelCY[53] = 0; m_uLabelCY[54] = 0; m_uLabelCY[55] = 0; m_uLabelCY[56] = 0; m_uLabelCY[57] = 0; m_uLabelCY[58] = 0; m_uLabelCY[59] = 0; m_uLabelCY[60] = 0; m_uLabelCY[61] = 0; m_uLabelCY[62] = 0; m_uLabelCY[63] = 0; 

	m_pVectorJSProgram[JS_PROG_INDEX(0,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(0,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(0,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(0,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(0,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(0,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(0,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(1,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(2,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(3,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(4,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(5,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(6,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(7,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(8,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(9,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(10,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(11,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(12,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(13,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(14,6)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,0)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,1)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,2)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,3)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,4)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,5)] = 0.0000;
	m_pVectorJSProgram[JS_PROG_INDEX(15,6)] = 0.0000;


	m_JS_XCtrl.cControlName = "MIDI JS X";
	m_JS_XCtrl.uControlId = 32773;
	m_JS_XCtrl.uUserDataType = floatData;
	m_JS_XCtrl.bMIDIControl = false;
	m_JS_XCtrl.uMIDIControlCommand = 176;
	m_JS_XCtrl.uMIDIControlName = 16;
	m_JS_XCtrl.uMIDIControlChannel = 0;
	m_JS_XCtrl.fJoystickValue = 0.0;
	m_JS_XCtrl.bKorgVectorJoystickOrientation = true;
	m_JS_XCtrl.nGUIRow = nIndexer++;
	m_JS_XCtrl.bEnableParamSmoothing = false;
	m_JS_XCtrl.fSmoothingTimeInMs = 100.00;
	m_JS_XCtrl.dPresetData[0] = 0.000000;m_JS_XCtrl.dPresetData[1] = 0.000000;m_JS_XCtrl.dPresetData[2] = 0.000000;m_JS_XCtrl.dPresetData[3] = 0.000000;m_JS_XCtrl.dPresetData[4] = 0.000000;m_JS_XCtrl.dPresetData[5] = 0.000000;m_JS_XCtrl.dPresetData[6] = 0.000000;m_JS_XCtrl.dPresetData[7] = 0.000000;m_JS_XCtrl.dPresetData[8] = 0.000000;m_JS_XCtrl.dPresetData[9] = 0.000000;m_JS_XCtrl.dPresetData[10] = 0.000000;m_JS_XCtrl.dPresetData[11] = 0.000000;m_JS_XCtrl.dPresetData[12] = 0.000000;m_JS_XCtrl.dPresetData[13] = 0.000000;m_JS_XCtrl.dPresetData[14] = 0.000000;m_JS_XCtrl.dPresetData[15] = 0.000000;

	m_JS_YCtrl.cControlName = "MIDI JS Y";
	m_JS_YCtrl.uControlId = 32774;
	m_JS_XCtrl.uUserDataType = floatData;
	m_JS_YCtrl.bMIDIControl = false;
	m_JS_YCtrl.uMIDIControlCommand = 176;
	m_JS_YCtrl.uMIDIControlName = 17;
	m_JS_YCtrl.uMIDIControlChannel = 0;
	m_JS_YCtrl.fJoystickValue = 0.0;
	m_JS_YCtrl.bKorgVectorJoystickOrientation = true;
	m_JS_YCtrl.nGUIRow = nIndexer++;
	m_JS_YCtrl.bEnableParamSmoothing = false;
	m_JS_YCtrl.fSmoothingTimeInMs = 100.00;
	m_JS_YCtrl.dPresetData[0] = 0.000000;m_JS_YCtrl.dPresetData[1] = 0.000000;m_JS_YCtrl.dPresetData[2] = 0.000000;m_JS_YCtrl.dPresetData[3] = 0.000000;m_JS_YCtrl.dPresetData[4] = 0.000000;m_JS_YCtrl.dPresetData[5] = 0.000000;m_JS_YCtrl.dPresetData[6] = 0.000000;m_JS_YCtrl.dPresetData[7] = 0.000000;m_JS_YCtrl.dPresetData[8] = 0.000000;m_JS_YCtrl.dPresetData[9] = 0.000000;m_JS_YCtrl.dPresetData[10] = 0.000000;m_JS_YCtrl.dPresetData[11] = 0.000000;m_JS_YCtrl.dPresetData[12] = 0.000000;m_JS_YCtrl.dPresetData[13] = 0.000000;m_JS_YCtrl.dPresetData[14] = 0.000000;m_JS_YCtrl.dPresetData[15] = 0.000000;

	float* pJSProg = NULL;
	m_PresetNames[0] = "Factory Preset";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[0] = pJSProg;

	m_PresetNames[1] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[1] = pJSProg;

	m_PresetNames[2] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[2] = pJSProg;

	m_PresetNames[3] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[3] = pJSProg;

	m_PresetNames[4] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[4] = pJSProg;

	m_PresetNames[5] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[5] = pJSProg;

	m_PresetNames[6] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[6] = pJSProg;

	m_PresetNames[7] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[7] = pJSProg;

	m_PresetNames[8] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[8] = pJSProg;

	m_PresetNames[9] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[9] = pJSProg;

	m_PresetNames[10] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[10] = pJSProg;

	m_PresetNames[11] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[11] = pJSProg;

	m_PresetNames[12] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[12] = pJSProg;

	m_PresetNames[13] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[13] = pJSProg;

	m_PresetNames[14] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[14] = pJSProg;

	m_PresetNames[15] = "";
	pJSProg = new float[MAX_JS_PROGRAM_STEPS*MAX_JS_PROGRAM_STEP_VARS];
	pJSProg[JS_PROG_INDEX(0,0)] = 0.000000;pJSProg[JS_PROG_INDEX(0,1)] = 0.000000;pJSProg[JS_PROG_INDEX(0,2)] = 0.000000;pJSProg[JS_PROG_INDEX(0,3)] = 0.000000;pJSProg[JS_PROG_INDEX(0,4)] = 0.000000;pJSProg[JS_PROG_INDEX(0,5)] = 0.000000;pJSProg[JS_PROG_INDEX(0,6)] = 0.000000;pJSProg[JS_PROG_INDEX(1,0)] = 0.000000;pJSProg[JS_PROG_INDEX(1,1)] = 0.000000;pJSProg[JS_PROG_INDEX(1,2)] = 0.000000;pJSProg[JS_PROG_INDEX(1,3)] = 0.000000;pJSProg[JS_PROG_INDEX(1,4)] = 0.000000;pJSProg[JS_PROG_INDEX(1,5)] = 0.000000;pJSProg[JS_PROG_INDEX(1,6)] = 0.000000;pJSProg[JS_PROG_INDEX(2,0)] = 0.000000;pJSProg[JS_PROG_INDEX(2,1)] = 0.000000;pJSProg[JS_PROG_INDEX(2,2)] = 0.000000;pJSProg[JS_PROG_INDEX(2,3)] = 0.000000;pJSProg[JS_PROG_INDEX(2,4)] = 0.000000;pJSProg[JS_PROG_INDEX(2,5)] = 0.000000;pJSProg[JS_PROG_INDEX(2,6)] = 0.000000;pJSProg[JS_PROG_INDEX(3,0)] = 0.000000;pJSProg[JS_PROG_INDEX(3,1)] = 0.000000;pJSProg[JS_PROG_INDEX(3,2)] = 0.000000;pJSProg[JS_PROG_INDEX(3,3)] = 0.000000;pJSProg[JS_PROG_INDEX(3,4)] = 0.000000;pJSProg[JS_PROG_INDEX(3,5)] = 0.000000;pJSProg[JS_PROG_INDEX(3,6)] = 0.000000;pJSProg[JS_PROG_INDEX(4,0)] = 0.000000;pJSProg[JS_PROG_INDEX(4,1)] = 0.000000;pJSProg[JS_PROG_INDEX(4,2)] = 0.000000;pJSProg[JS_PROG_INDEX(4,3)] = 0.000000;pJSProg[JS_PROG_INDEX(4,4)] = 0.000000;pJSProg[JS_PROG_INDEX(4,5)] = 0.000000;pJSProg[JS_PROG_INDEX(4,6)] = 0.000000;pJSProg[JS_PROG_INDEX(5,0)] = 0.000000;pJSProg[JS_PROG_INDEX(5,1)] = 0.000000;pJSProg[JS_PROG_INDEX(5,2)] = 0.000000;pJSProg[JS_PROG_INDEX(5,3)] = 0.000000;pJSProg[JS_PROG_INDEX(5,4)] = 0.000000;pJSProg[JS_PROG_INDEX(5,5)] = 0.000000;pJSProg[JS_PROG_INDEX(5,6)] = 0.000000;pJSProg[JS_PROG_INDEX(6,0)] = 0.000000;pJSProg[JS_PROG_INDEX(6,1)] = 0.000000;pJSProg[JS_PROG_INDEX(6,2)] = 0.000000;pJSProg[JS_PROG_INDEX(6,3)] = 0.000000;pJSProg[JS_PROG_INDEX(6,4)] = 0.000000;pJSProg[JS_PROG_INDEX(6,5)] = 0.000000;pJSProg[JS_PROG_INDEX(6,6)] = 0.000000;pJSProg[JS_PROG_INDEX(7,0)] = 0.000000;pJSProg[JS_PROG_INDEX(7,1)] = 0.000000;pJSProg[JS_PROG_INDEX(7,2)] = 0.000000;pJSProg[JS_PROG_INDEX(7,3)] = 0.000000;pJSProg[JS_PROG_INDEX(7,4)] = 0.000000;pJSProg[JS_PROG_INDEX(7,5)] = 0.000000;pJSProg[JS_PROG_INDEX(7,6)] = 0.000000;pJSProg[JS_PROG_INDEX(8,0)] = 0.000000;pJSProg[JS_PROG_INDEX(8,1)] = 0.000000;pJSProg[JS_PROG_INDEX(8,2)] = 0.000000;pJSProg[JS_PROG_INDEX(8,3)] = 0.000000;pJSProg[JS_PROG_INDEX(8,4)] = 0.000000;pJSProg[JS_PROG_INDEX(8,5)] = 0.000000;pJSProg[JS_PROG_INDEX(8,6)] = 0.000000;pJSProg[JS_PROG_INDEX(9,0)] = 0.000000;pJSProg[JS_PROG_INDEX(9,1)] = 0.000000;pJSProg[JS_PROG_INDEX(9,2)] = 0.000000;pJSProg[JS_PROG_INDEX(9,3)] = 0.000000;pJSProg[JS_PROG_INDEX(9,4)] = 0.000000;pJSProg[JS_PROG_INDEX(9,5)] = 0.000000;pJSProg[JS_PROG_INDEX(9,6)] = 0.000000;pJSProg[JS_PROG_INDEX(10,0)] = 0.000000;pJSProg[JS_PROG_INDEX(10,1)] = 0.000000;pJSProg[JS_PROG_INDEX(10,2)] = 0.000000;pJSProg[JS_PROG_INDEX(10,3)] = 0.000000;pJSProg[JS_PROG_INDEX(10,4)] = 0.000000;pJSProg[JS_PROG_INDEX(10,5)] = 0.000000;pJSProg[JS_PROG_INDEX(10,6)] = 0.000000;pJSProg[JS_PROG_INDEX(11,0)] = 0.000000;pJSProg[JS_PROG_INDEX(11,1)] = 0.000000;pJSProg[JS_PROG_INDEX(11,2)] = 0.000000;pJSProg[JS_PROG_INDEX(11,3)] = 0.000000;pJSProg[JS_PROG_INDEX(11,4)] = 0.000000;pJSProg[JS_PROG_INDEX(11,5)] = 0.000000;pJSProg[JS_PROG_INDEX(11,6)] = 0.000000;pJSProg[JS_PROG_INDEX(12,0)] = 0.000000;pJSProg[JS_PROG_INDEX(12,1)] = 0.000000;pJSProg[JS_PROG_INDEX(12,2)] = 0.000000;pJSProg[JS_PROG_INDEX(12,3)] = 0.000000;pJSProg[JS_PROG_INDEX(12,4)] = 0.000000;pJSProg[JS_PROG_INDEX(12,5)] = 0.000000;pJSProg[JS_PROG_INDEX(12,6)] = 0.000000;pJSProg[JS_PROG_INDEX(13,0)] = 0.000000;pJSProg[JS_PROG_INDEX(13,1)] = 0.000000;pJSProg[JS_PROG_INDEX(13,2)] = 0.000000;pJSProg[JS_PROG_INDEX(13,3)] = 0.000000;pJSProg[JS_PROG_INDEX(13,4)] = 0.000000;pJSProg[JS_PROG_INDEX(13,5)] = 0.000000;pJSProg[JS_PROG_INDEX(13,6)] = 0.000000;pJSProg[JS_PROG_INDEX(14,0)] = 0.000000;pJSProg[JS_PROG_INDEX(14,1)] = 0.000000;pJSProg[JS_PROG_INDEX(14,2)] = 0.000000;pJSProg[JS_PROG_INDEX(14,3)] = 0.000000;pJSProg[JS_PROG_INDEX(14,4)] = 0.000000;pJSProg[JS_PROG_INDEX(14,5)] = 0.000000;pJSProg[JS_PROG_INDEX(14,6)] = 0.000000;pJSProg[JS_PROG_INDEX(15,0)] = 0.000000;pJSProg[JS_PROG_INDEX(15,1)] = 0.000000;pJSProg[JS_PROG_INDEX(15,2)] = 0.000000;pJSProg[JS_PROG_INDEX(15,3)] = 0.000000;pJSProg[JS_PROG_INDEX(15,4)] = 0.000000;pJSProg[JS_PROG_INDEX(15,5)] = 0.000000;pJSProg[JS_PROG_INDEX(15,6)] = 0.000000;
	m_PresetJSPrograms[15] = pJSProg;

	// --- v6.8 thread-safe additions
	int nNumParams = m_UIControlList.count() + numAddtlParams;// numAddtlParams = 2 params from VJStick
	m_uControlListCount = m_UIControlList.count();

	// --- create fast lookup table of controls
	if(m_ppControlTable) delete [] m_ppControlTable;
	m_ppControlTable = new CUICtrl*[nNumParams];

	// --- create outbound GUI Parameters -----------------------------------------------
	if(m_pOutGUIParameters) delete [] m_pOutGUIParameters;
	m_pOutGUIParameters = new GUI_PARAMETER[nNumParams];
	memset(m_pOutGUIParameters, 0, sizeof(GUI_PARAMETER)*nNumParams);
	for(int i=0; i<m_UIControlList.count(); i++)
	{
		// --- save the CUICtrl* for fast lookups
		CUICtrl* pUICtrl = m_UIControlList.getAt(i);
		if(!pUICtrl) continue; // will never happen
		m_ppControlTable[i] = pUICtrl;

		// --- setup the outbound GUI Parameters
		m_pOutGUIParameters[i].fActualValue = 0.0;
		m_pOutGUIParameters[i].fNormalizedValue = 0.0;
		m_pOutGUIParameters[i].uControlId = pUICtrl->uControlId;
		m_pOutGUIParameters[i].nControlIndex = i;
		m_pOutGUIParameters[i].uSampleOffset = 0;
		m_pOutGUIParameters[i].bKorgVectorJoystickOrientation = false;
	}


	// **--0xEDA5--**
// -------------------------------------------------------------------------------------- //

	return true;

}
