#pragma once
#include "pluginconstants.h"

//enum { directForm, canonicalForm, transposeDirectForm, transposeCanonical, modBiQuad };
enum { DF, TDF, CF, TCF };
enum { LPF, HSF };

// directForm is already done in the base class

class CBiQuadEMJ :
	public CBiQuad
{
public:
	CBiQuadEMJ();
	virtual ~CBiQuadEMJ();

	void setFilterStructure(UINT u) { m_uFilterStructure = u; }
	void setFilterType(UINT v) { m_uFilterType = v; }

	float doBiQuad(float f_xn);

protected:
	UINT m_uFilterStructure;
	UINT m_uFilterType;

};

