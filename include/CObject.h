#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_COBJECT_H
#define IVSDK_HEADER_COBJECT_H
class CPhysical;

class CObject : public CPhysical
{
public:
	uint8_t pad[0x40];													// 210-250
	float m_fScale;														// 250-254 set by SET_OBJECT_SCALE but unused
	uint8_t pad2[0xCC];													// 254-320
};
VALIDATE_SIZE(CObject, 0x320);
VALIDATE_OFFSET(CObject, m_fScale, 0x250);
#endif // IVSDK_HEADER_COBJECT_H
