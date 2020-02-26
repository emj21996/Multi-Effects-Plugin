#pragma once
#include "pluginconstants.h"



class CFiltersEMJ :
	public CBiQuad
{
	int m_nSampleRate;
public:
	CFiltersEMJ();
	virtual ~CFiltersEMJ();

	// SET SAMPLE RATE
	void setSampleRate(int nSampleRate) { m_nSampleRate = nSampleRate; }

	float m_f_c0;
	float m_f_d0;

	// DOES FILTER
	float doBiQuad(float f_xn);

	// filters cookbook

	// flush delays 

protected:
	UINT m_uFilterType;
};

