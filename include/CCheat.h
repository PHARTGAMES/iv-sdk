#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CCHEAT_H
#define IVSDK_HEADER_CCHEAT_H
class CCheat
{
public:
	static inline auto& m_bHasPlayerCheated = AddressSetter::GetRef<bool>(0x11E3688, 0xF13B38);
};
#endif // IVSDK_HEADER_CCHEAT_H
