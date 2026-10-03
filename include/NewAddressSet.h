#pragma once
#include "IVSDKCore.h"
namespace AddressSetter
{
    extern uint32_t gBaseAddress;
    extern bool bAddressesRead;
    uint32_t GetVersionFromEXE();
    void DetermineVersion();
    void Init();
    uint32_t Get(uint32_t addr1070, uint32_t addr1080);
	// note that the base address is added here and 0x400000 is not subtracted, so rebase your .idb to 0x0 or subtract it yourself
	template<typename T> T& GetRef(uint32_t addr1070, uint32_t addr1080)
	{
		if (!bAddressesRead)
		{
			Init();
		}
		if (plugin::gameVer == plugin::VERSION_1070) return *reinterpret_cast<T*>(gBaseAddress + addr1070);
		if (plugin::gameVer == plugin::VERSION_1080) return *reinterpret_cast<T*>(gBaseAddress + addr1080);
		return *static_cast<T*>(nullptr);
	}

}
