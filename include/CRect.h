#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CRECT_H
#define IVSDK_HEADER_CRECT_H
class CRect
{
public:
	float left;				// 00-04
	float bottom;			// 04-08
	float right;			// 08-0C
	float top;				// 0C-10
};
VALIDATE_SIZE(CRect, 0x10);
#endif // IVSDK_HEADER_CRECT_H
