#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CPICKUPS_H
#define IVSDK_HEADER_CPICKUPS_H
class CPickups
{
public:
	static void DoPickUpEffects()
	{
		return ((void(__cdecl*)())(AddressSetter::Get(0x534280, 0x589100)))();
	}
};
#endif // IVSDK_HEADER_CPICKUPS_H
