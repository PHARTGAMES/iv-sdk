#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTASK_H
#define IVSDK_HEADER_CTASK_H
class CTask
{
public:
	void* operator new(size_t nSize)
	{
		return CPools::ms_pTaskPool->New();
	}
	void operator delete(void*, size_t, uint32_t) {};
};

class CTaskComplex : public CTask
{
public:

};

class CTaskSimple : public CTask
{
public:

};
#endif // IVSDK_HEADER_CTASK_H
