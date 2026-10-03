#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CDSTREAM_H
#define IVSDK_HEADER_CDSTREAM_H
inline void CdStreamAddImage(char* sPath, uint8_t unk1, int32_t unkNeg1)
{
	((void(__cdecl*)(char*, uint8_t, int32_t))(AddressSetter::Get(0x497730, 0x622BE0)))(sPath, unk1, unkNeg1);
}
#endif // IVSDK_HEADER_CDSTREAM_H
