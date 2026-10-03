#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTASKSIMPLESIDEWAYSDIVE_H
#define IVSDK_HEADER_CTASKSIMPLESIDEWAYSDIVE_H
class CTaskSimpleSidewaysDive : public CTaskSimple
{
public:
	CTaskSimpleSidewaysDive(bool bDirection)
	{
		((void(__thiscall*)(CTaskSimpleSidewaysDive*, bool))(AddressSetter::Get(0xEDBC0, 0x302F30)))(this, bDirection);
	}
};
#endif // IVSDK_HEADER_CTASKSIMPLESIDEWAYSDIVE_H
