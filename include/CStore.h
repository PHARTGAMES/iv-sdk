#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CSTORE_H
#define IVSDK_HEADER_CSTORE_H
template<typename T>
class CStore
{
public:
	uint32_t m_maxItems;				// 00-04
	uint32_t m_nextItem;				// 04-08
	T* m_storeArray;					// 08-0C
};

extern CStore<CVehicleModelInfo>& ms_vehicleModelStore;
extern CStore<CPedModelInfo>& ms_pedModelStore;
#endif // IVSDK_HEADER_CSTORE_H
