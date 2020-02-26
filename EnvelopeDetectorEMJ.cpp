#include "EnvelopeDetectorEMJ.h"


// --- ADD YOUR OWN COMMENTS!
CEnvelopeDetectorEMJ::CEnvelopeDetectorEMJ()
{
	m_fPEQ_Fc = 1000.f;
	m_fPEQ_Q = 0.707f;
	m_fPEQ_Gain_dB = 0.0;
}


CEnvelopeDetectorEMJ::~CEnvelopeDetectorEMJ()
{
}

void CEnvelopeDetectorEMJ::calculatePEQCoeffs()
{
	float K = tanf(pi*m_fPEQ_Fc / (float)m_fSampleRate);

	float Vo = pow(10.0, m_fPEQ_Gain_dB / 20.0);

	bool bBoost = m_fPEQ_Gain_dB >= 0 ? true : false;

	float d0 = 1.0 + (1.0 / m_fPEQ_Q)*K + K*K;
	float e0 = 1.0 + (1.0 / (Vo*m_fPEQ_Q))*K + K*K;

	float alpha = 1.0 + (Vo / m_fPEQ_Q)*K + K*K;
	float beta = 2.0*(K*K - 1.0);
	float gamma = 1.0 - (Vo / m_fPEQ_Q)*K + K*K;
	float delta = 1.0 - (1.0 / m_fPEQ_Q)*K + K*K;
	float eta = 1.0 - (1.0 / (Vo*m_fPEQ_Q))*K + K*K;

	float a0 = bBoost ? alpha / d0 : d0 / e0;
	float a1 = bBoost ? beta / d0 : beta / e0;
	float a2 = bBoost ? gamma / d0 : delta / e0;

	float b1 = bBoost ? beta / d0 : beta / e0;
	float b2 = bBoost ? delta / d0 : eta / e0;

	// --- update coeffs
	m_PEQ.m_f_a0 = a0;
	m_PEQ.m_f_a1 = a1;
	m_PEQ.m_f_a2 = a2;
	m_PEQ.m_f_b1 = b1;
	m_PEQ.m_f_b2 = b2;
}
