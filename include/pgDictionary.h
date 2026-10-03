#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_PGDICTIONARY_H
#define IVSDK_HEADER_PGDICTIONARY_H
namespace rage
{
	template<typename T>
	class pgDictionary
	{
	public:
		T* getEntryByKey(uint32_t nHash)
		{
			return ((T * (__thiscall*)(pgDictionary*, uint32_t))(AddressSetter::Get(0x24B60, 0x3A490)))(this, nHash);
		}
	};
}
#endif // IVSDK_HEADER_PGDICTIONARY_H
