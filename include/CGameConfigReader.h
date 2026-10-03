#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CGAMECONFIGREADER_H
#define IVSDK_HEADER_CGAMECONFIGREADER_H
class CGameConfigReader
{
public:

	void LoadFile(char* fileName)
	{
		((void(__thiscall*)(CGameConfigReader*, char*))(AddressSetter::Get(0x4D5C10, 0x6CA0D0)))(this, fileName);
	}
};
extern CGameConfigReader*& GameConfigReader;
#endif // IVSDK_HEADER_CGAMECONFIGREADER_H
