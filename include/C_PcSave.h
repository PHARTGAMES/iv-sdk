#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_C_PCSAVE_H
#define IVSDK_HEADER_C_PCSAVE_H
class C_PcSave
{
public:

	// slot -1 is autosave
	static int SaveSlot(int32_t SlotNum)
	{
		return ((int(__cdecl*)(int32_t))(AddressSetter::Get(0x42E0A0, 0x459560)))(SlotNum);
	}
	static int LoadSlot(int32_t SlotNum)
	{
		return ((int(__cdecl*)(int32_t))(AddressSetter::Get(0x42E130, 0x4595F0)))(SlotNum);
	}
};
#endif // IVSDK_HEADER_C_PCSAVE_H
