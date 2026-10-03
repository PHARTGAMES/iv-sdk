#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CCUSTOMSHADEREFFECTBASE_H
#define IVSDK_HEADER_CCUSTOMSHADEREFFECTBASE_H
class CCustomShaderEffectBase
{
public:
	void Update(CEntity* attachedEntity)
	{
		((void(__thiscall*)(CCustomShaderEffectBase*, CEntity*))(*(void***)this)[3])(this, attachedEntity);
	}
};

#endif // IVSDK_HEADER_CCUSTOMSHADEREFFECTBASE_H
