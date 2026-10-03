#include "IVSDK.h"
#include "Hooks.h"
#include "injector/injector.hpp"
#include <array>

namespace plugin
{
    namespace
    {
        struct HookPatch
        {
            uintptr_t address = 0;
            unsigned char original[5] = {};
            unsigned char installed[5] = {};
        };
        std::array<HookPatch, 9> patches;
        size_t patchCount = 0;
        bool initialized = false;

        uintptr_t DoHook(uintptr_t address, void(*function)())
        {
            if (!address) return 0;
            auto& patch = patches[patchCount++];
            patch.address = address;
            injector::ReadMemoryRaw(address, patch.original, sizeof(patch.original), true);
            uintptr_t previous = (uintptr_t)injector::MakeCALL(address, function);
            injector::ReadMemoryRaw(address, patch.installed, sizeof(patch.installed), true);
            return previous;
        }
	void InitHooks()
	{
		processScriptsEvent::returnAddress = DoHook(AddressSetter::Get(0x21601, 0x95141), processScriptsEvent::MainHook);
		gameLoadEvent::returnAddress = DoHook(AddressSetter::Get(0x4ADB38, 0x770748), gameLoadEvent::MainHook);
		gameLoadPriorityEvent::returnAddress = DoHook(AddressSetter::Get(0x4ADA9D, 0x7706AD), gameLoadPriorityEvent::MainHook);
		drawingEvent::returnAddress = DoHook(AddressSetter::Get(0x46AFA8, 0x60E1C8), drawingEvent::MainHook);
		processAutomobileEvent::callAddress = DoHook(AddressSetter::Get(0x7FE9C6, 0x652C26), processAutomobileEvent::MainHook);
		processPadEvent::callAddress = DoHook(AddressSetter::Get(0x3C4002, 0x46A802), processPadEvent::MainHook);
		processCameraEvent::returnAddress = DoHook(AddressSetter::Get(0x52C4C2, 0x694232), processCameraEvent::MainHook);
		mountDeviceEvent::returnAddress = DoHook(AddressSetter::Get(0x3B2E27, 0x456C27), mountDeviceEvent::MainHook);
		ingameStartupEvent::returnAddress = DoHook(AddressSetter::Get(0x20379, 0x93F09), ingameStartupEvent::MainHook);
	}
    }

    bool Init()
    {
        if (initialized) return true;
        if (!AddressSetter::bAddressesRead) AddressSetter::Init();
        if (gameVer == VERSION_NONE) return false;
        InitHooks();
        initialized = true;
        return true;
    }

    bool IsInitialized() { return initialized; }

    void Deinit()
    {
        if (!initialized) return;
        // Other plugins may have chained a hook after ours. In that case keep
        // our code loaded and leave the chain intact; unloading is unsupported.
        while (patchCount)
        {
            const auto& patch = patches[--patchCount];
            unsigned char current[5];
            injector::ReadMemoryRaw(patch.address, current, sizeof(current), true);
            if (std::memcmp(current, patch.installed, sizeof(current)) == 0)
            {
                injector::WriteMemoryRaw(patch.address,
                    const_cast<unsigned char*>(patch.original), sizeof(patch.original), true);
                FlushInstructionCache(GetCurrentProcess(),
                    reinterpret_cast<void*>(patch.address), sizeof(patch.original));
            }
        }
        processScriptsEvent::funcPtrs.clear();
        gameLoadPriorityEvent::funcPtrs.clear();
        gameLoadEvent::funcPtrs.clear();
        ingameStartupEvent::funcPtrs.clear();
        mountDeviceEvent::funcPtrs.clear();
        drawingEvent::funcPtrs.clear();
        processCameraEvent::funcPtrs.clear();
        processAutomobileEvent::funcPtrs.clear();
        processPadEvent::funcPtrs.clear();
        initialized = false;
    }
}
