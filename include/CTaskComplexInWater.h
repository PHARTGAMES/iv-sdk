#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTASKCOMPLEXINWATER_H
#define IVSDK_HEADER_CTASKCOMPLEXINWATER_H
class CTaskComplexInWater : public CTaskComplex
{
public:
	CTaskComplexInWater(uint32_t unk, uint32_t unk2, bool bUnk)
	{
		((void(__thiscall*)(CTaskComplexInWater*, uint32_t, uint32_t, bool))(AddressSetter::Get(0x61EC00, 0x762950)))(this, unk, unk2, bUnk);
	}
};
#endif // IVSDK_HEADER_CTASKCOMPLEXINWATER_H
