#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTASKCOMPLEXCLIMBLADDER_H
#define IVSDK_HEADER_CTASKCOMPLEXCLIMBLADDER_H
class CTaskComplexClimbLadder : public CTaskComplex
{
public:
	CTaskComplexClimbLadder(CObject* ladder, int32_t type, uint32_t unk0)
	{
		((void(__thiscall*)(CTaskComplexClimbLadder*, CObject*, int32_t, uint32_t))(AddressSetter::Get(0x8AD9D0, 0x8756F0)))(this, ladder, type, unk0);
	}
};
#endif // IVSDK_HEADER_CTASKCOMPLEXCLIMBLADDER_H
