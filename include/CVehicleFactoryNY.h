#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CVEHICLEFACTORYNY_H
#define IVSDK_HEADER_CVEHICLEFACTORYNY_H
class CVehicleFactory
{
public:

};

class CVehicleFactoryNY : CVehicleFactory
{
public:

	CVehicle* CreateVehicle(int32_t model, int32_t createdBy, CMatrix* mat, bool bNetwork)
	{
		return ((CVehicle * (__thiscall*)(CVehicleFactoryNY*, int32_t, int32_t, CMatrix*, bool))(*(void***)this)[1])(this, model, createdBy, mat, bNetwork);
	}
};

extern CVehicleFactoryNY*& VehicleFactory;
#endif // IVSDK_HEADER_CVEHICLEFACTORYNY_H
