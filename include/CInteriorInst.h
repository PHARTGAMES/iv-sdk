#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CINTERIORINST_H
#define IVSDK_HEADER_CINTERIORINST_H
class CInteriorInst : public CBuilding
{
public:
	uint8_t pad[0xF0];										// 070-160
};

VALIDATE_SIZE(CInteriorInst, 0x160);

#endif // IVSDK_HEADER_CINTERIORINST_H
