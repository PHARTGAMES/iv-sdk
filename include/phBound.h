#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_PHBOUND_H
#define IVSDK_HEADER_PHBOUND_H
namespace rage
{
	class phBound
	{
	public:
		uint8_t pad[0x80];				// 00-80

		// +0x50 off vft is set mass for cars but seems to be userpurge, todo
	};
	VALIDATE_SIZE(phBound, 0x80);

	class phBoundComposite : public phBound
	{
	public:
		uint8_t pad[0x20];				// 80-A0
	};
	VALIDATE_SIZE(phBoundComposite, 0xA0);
};
#endif // IVSDK_HEADER_PHBOUND_H
