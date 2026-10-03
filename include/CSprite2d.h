#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CSPRITE2D_H
#define IVSDK_HEADER_CSPRITE2D_H
class CSprite2d
{
public:
	rage::grcTexturePC* m_pTexture = nullptr;

	void SetTexture(char* sName)
	{
		((void(__thiscall*)(CSprite2d*, char*))(AddressSetter::Get(0x4534A0, 0x45DF40)))(this, sName);
	}
	void Delete()
	{
		((void(__thiscall*)(CSprite2d*))(AddressSetter::Get(0x4523E0, 0x45CE80)))(this);
	}
};
VALIDATE_SIZE(CSprite2d, 0x4);
#endif // IVSDK_HEADER_CSPRITE2D_H
