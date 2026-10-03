#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CSHADERLIB_H
#define IVSDK_HEADER_CSHADERLIB_H
class CShaderLib
{
public:
	// default, unlit, deferred, etc.
	static int GetTechniqueID(char* name)
	{
		((int(__cdecl*)(char*))(AddressSetter::Get(0x2340C0, 0x185D0)))(name);
	}
};
#endif // IVSDK_HEADER_CSHADERLIB_H
