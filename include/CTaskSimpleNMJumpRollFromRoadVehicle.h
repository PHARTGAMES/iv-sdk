#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTASKSIMPLENMJUMPROLLFROMROADVEHICLE_H
#define IVSDK_HEADER_CTASKSIMPLENMJUMPROLLFROMROADVEHICLE_H
class CTaskSimpleNMJumpRollFromRoadVehicle : public CTaskSimple
{
public:
	CTaskSimpleNMJumpRollFromRoadVehicle(uint32_t time, uint32_t time2)
	{
		((void(__thiscall*)(CTaskSimpleNMJumpRollFromRoadVehicle*, uint32_t, uint32_t))(AddressSetter::Get(0x85CCB0, 0x7D81B0)))(this, time, time2);
	}
};
#endif // IVSDK_HEADER_CTASKSIMPLENMJUMPROLLFROMROADVEHICLE_H
