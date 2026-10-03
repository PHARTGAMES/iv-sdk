#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CCUTSCENEMGR_H
#define IVSDK_HEADER_CCUTSCENEMGR_H
class CCutsceneMgr
{
public:
	static inline uint32_t& ms_running = AddressSetter::GetRef<uint32_t>(0xDE45F0, 0xD515D0);

	static bool IsRunning()
	{
		return ((bool(__cdecl*)())(AddressSetter::Get(0x45F210, 0x3F02B0)))();
	}
};
#endif // IVSDK_HEADER_CCUTSCENEMGR_H
