#pragma once
#include "pluginconstants.h"

// base class


// --- envelope detector with built-in Parameteric EQ module
class CEnvelopeDetectorEMJ : public CEnvelopeDetector
{
public:
	CEnvelopeDetectorEMJ();
	virtual ~CEnvelopeDetectorEMJ();

	// --- overrides -- COMMENT THIS STUFF IN CLASS!
	//
	// --- prepareForPlay()
	void prepareForPlay()
	{
		// --- flush biquad
		m_PEQ.flushDelays();

		// --- set initial coefficients
		calculatePEQCoeffs();

		// --- base class does its thing
		CEnvelopeDetector::prepareForPlay();
	}

	// --- detect()
	float detect(float fInput)
	{
		float filteredInput = m_PEQ.doBiQuad(fInput);

		// --- base class does its thing
		return CEnvelopeDetector::detect(filteredInput);
	}

	// --- interface functions
	//
	// --- setPEQParameters()
	void setPEQParameters(float fc, float Q, float boostCut_dB)
	{
		m_fPEQ_Fc = fc;
		m_fPEQ_Q = Q;
		m_fPEQ_Gain_dB = boostCut_dB;

		calculatePEQCoeffs();
	}

	// --- you might want to add your own discrete interface functions here...

	// --- the coefficient updater
	void calculatePEQCoeffs();

protected:
	// --- the biquad - using regular old biquad because this EQ will not be modulated
	CBiQuad m_PEQ;

	// --- the parameters
	float m_fPEQ_Fc;
	float m_fPEQ_Q;
	float m_fPEQ_Gain_dB;

};

