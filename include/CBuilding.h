#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CBUILDING_H
#define IVSDK_HEADER_CBUILDING_H
class CBuilding : public CEntity
{
public:

	void ReplaceWithNewModel(int32_t index)
	{
		return ((void(__thiscall*)(CBuilding*, int32_t))(AddressSetter::Get(0x71B430, 0x4DDD00)))(this, index);
	}
};

VALIDATE_SIZE(CBuilding, 0x70);

#endif // IVSDK_HEADER_CBUILDING_H
