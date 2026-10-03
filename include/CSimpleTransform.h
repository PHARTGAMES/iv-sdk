#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CSIMPLETRANSFORM_H
#define IVSDK_HEADER_CSIMPLETRANSFORM_H
class CSimpleTransform
{
public:
	CVector m_vPosition;												// 000-00C
	float m_fHeading;													// 00C-010
};

VALIDATE_SIZE(CSimpleTransform, 0x10);

#endif // IVSDK_HEADER_CSIMPLETRANSFORM_H
