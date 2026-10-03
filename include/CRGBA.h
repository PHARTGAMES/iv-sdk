#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CRGBA_H
#define IVSDK_HEADER_CRGBA_H
struct CRGBA
{
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t a;
};

struct CRGBFloat
{
	float red;
	float green;
	float blue;
};
#endif // IVSDK_HEADER_CRGBA_H
