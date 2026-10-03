#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CREPLAY_H
#define IVSDK_HEADER_CREPLAY_H
class CReplay
{
public:
	static inline auto& Mode = AddressSetter::GetRef<uint32_t>(0xCD0170, 0xD2EAC0);
};
#endif // IVSDK_HEADER_CREPLAY_H
