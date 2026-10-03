#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTIMECYCLE_H
#define IVSDK_HEADER_CTIMECYCLE_H
class CTimeCycle
{
public:
	static inline char*& sTimecycleModifierPath = AddressSetter::GetRef<char*>(0x48A651, 0x63AD31); // platform:/data

	static void Initialise()
	{
		return ((void(__cdecl*)())(AddressSetter::Get(0x48AD70, 0x63B450)))();
	}
	static void InitModifiers()
	{
		return ((void(__cdecl*)())(AddressSetter::Get(0x48A650, 0x63AD30)))();
	}
};
#endif // IVSDK_HEADER_CTIMECYCLE_H
