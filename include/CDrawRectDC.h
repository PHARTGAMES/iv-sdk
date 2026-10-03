#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CDRAWRECTDC_H
#define IVSDK_HEADER_CDRAWRECTDC_H
class CDrawRectDC : public CBaseDC
{
public:
	uint8_t pad[0x14];						// 08-1C
	// coords are by pixel
	// also works in main menu
	CDrawRectDC(CRect* rect, CRGBA color)
	{
		((void(__thiscall*)(CDrawRectDC*, CRect*, CRGBA))(AddressSetter::Get(0x3C1110, 0x461780)))(this, rect, color);
	}
};
VALIDATE_SIZE(CDrawRectDC, 0x1C);
#endif // IVSDK_HEADER_CDRAWRECTDC_H
