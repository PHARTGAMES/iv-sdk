#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_RAGE_H
#define IVSDK_HEADER_RAGE_H
// any non-class rage functions should go here
namespace rage
{
	extern HWND& g_pHWND;
	extern IDirect3DDevice9*& g_pDirect3DDevice;
	static uint32_t atStringHash(const char* sString, uint32_t* nExistingHash = nullptr)
	{
		return ((uint32_t(__cdecl*)(const char*, uint32_t*))(AddressSetter::Get(0x1B1C30, 0x5CF50)))(sString, nExistingHash);
	}
}
#endif // IVSDK_HEADER_RAGE_H
