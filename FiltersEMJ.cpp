#include "FiltersEMJ.h"
#include "pluginconstants.h"



CFiltersEMJ::CFiltersEMJ()
{
	m_f_c0 = 1.0;
	m_f_d0 = 0.0;
}


float CFiltersEMJ::doBiQuad(float f_xn)
{
	/*
	if (m_uFilterType == LPF)
	return CFilters::doBiQuad(f_xn);
	*/
	/*
	if (m_uFilterType == HSF)
	{

	// use same terms as in book:
	float theta_c = 2.0*pi*fCutoffFreq / (float)m_nSampleRate;
	float mu = pow(10.0, m_f_peakGain / 20.0);

	// beta
	float fBeta = ((1.0 + mu) / 4.0);

	// delta
	float fDelta = fBeta * tanf(theta_c / 2.0);

	// gamma
	float fGamma = ((1.0 - fDelta) / (1.0 + fDelta));

	// alpha
	float fAlpha = ((1.0 + fGamma) / 2.0);

	// left channel
	m_HSF.m_f_a0 = fAlpha;
	m_HSF.m_f_a1 = -((1.0 + fGamma) / 2.0);
	m_HSF.m_f_a2 = 0.0;
	m_HSF.m_f_b1 = -fGamma;
	m_HSF.m_f_b2 = 0.0;
	m_HSF.m_f_c0 = mu - 1.0;
	m_HSF.m_f_d0 = 1.0;

	}  */
	return true;
}

CFiltersEMJ::~CFiltersEMJ()
{
}
