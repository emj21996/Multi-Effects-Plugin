#include "BiQuadEMJ.h"



CBiQuadEMJ::CBiQuadEMJ()
{
	UINT m_uFilterStructure = DF;
}


CBiQuadEMJ::~CBiQuadEMJ()
{
}





float CBiQuadEMJ::doBiQuad(float f_xn)
{
	// Direct Form, included in the base class, so its a simple function
	if (m_uFilterStructure == DF)
		return CBiQuad::doBiQuad(f_xn);


	// Transpose Direct Form
	if (m_uFilterStructure == TDF)
	{
		float wn = f_xn + m_f_Xz_1;
		float yn = m_f_a0*wn + m_f_Yz_1;

		// underflow check
		if (yn > 0.0 && yn < FLT_MIN_PLUS) yn = 0;
		if (yn < 0.0 && yn > FLT_MIN_MINUS) yn = 0;

		// X Delays
		m_f_Xz_1 = m_f_Xz_2 - m_f_b1*wn;
		m_f_Xz_2 = -m_f_b2*wn;

		// Y delays
		m_f_Yz_1 = m_f_Yz_2 + m_f_a1*wn;
		m_f_Yz_2 = m_f_a2*wn;

		return yn;
	}


	// Canonical Form
	if (m_uFilterStructure == CF)
	{
		float wn = f_xn - (m_f_b1*m_f_Xz_1) - (m_f_b2*m_f_Xz_2);
		float yn = (m_f_a0*wn) + (m_f_a1*m_f_Xz_1) + (m_f_a2*m_f_Xz_2);

		// underflow check
		if (yn > 0.0 && yn < FLT_MIN_PLUS) yn = 0;
		if (yn < 0.0 && yn > FLT_MIN_MINUS) yn = 0;

		// update states
		m_f_Xz_2 = m_f_Xz_1;
		m_f_Xz_1 = wn;

		return yn;
	}

	// Transpose Canonical Form
	if (m_uFilterStructure == TCF)
	{

		float yn = m_f_a0*f_xn + m_f_Xz_1;
		float s1 = m_f_Xz_1 + m_f_a1 - m_f_b1;
		float s2 = m_f_a2 - m_f_b2;


		// underflow check
		if (yn > 0.0 && yn < FLT_MIN_PLUS) yn = 0;
		if (yn < 0.0 && yn > FLT_MIN_MINUS) yn = 0;

		// update states
		m_f_Xz_1 = m_f_Xz_2 + m_f_a1*f_xn - m_f_b1*yn;
		m_f_Xz_2 = m_f_a2*f_xn - m_f_b2*yn;

		return yn;
	}

	// modified biQuad for High Shelving
	/*
	if (m_uFilterStructure == modBiQuad)
	{
	float wn = m_f_a0*f_xn + m_f_a1*m_f_Xz_1 + m_f_a2*m_f_Xz_2 - m_f_b1*m_f_Yz_1 - m_f_b2*m_f_Yz_2;
	float yn = wn + m_f_c0*wn + m_f_d0*wn;     // modified feedback loop

	// underflow check
	if (yn > 0.0 && yn < FLT_MIN_PLUS) yn = 0;
	if (yn < 0.0 && yn > FLT_MIN_MINUS) yn = 0;

	// shuffle delays
	// Y delays
	m_f_Yz_2 = m_f_Yz_1;
	m_f_Yz_1 = wn;

	// X delays
	m_f_Xz_2 = m_f_Xz_1;
	m_f_Xz_1 = f_xn;

	return  yn;
	} */


	return true;
}
