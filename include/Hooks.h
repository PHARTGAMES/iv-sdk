#pragma once
#include "IVSDK.h"
namespace plugin
{
    namespace processScriptsEvent
    {
        extern uint8_t threadDummy[256];
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace gameLoadPriorityEvent
    {
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace gameLoadEvent
    {
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace ingameStartupEvent
    {
        extern uint8_t threadDummy[256];
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace mountDeviceEvent
    {
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace drawingEvent
    {
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace processCameraEvent
    {
        extern uintptr_t returnAddress;
        extern std::list<void(*)()> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)());
    }
    namespace processAutomobileEvent
    {
        extern CVehicle* thisParam;
        extern uintptr_t callAddress;
        extern std::list<void(*)(CVehicle*)> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)(CVehicle*));
    }
    namespace processPadEvent
    {
        extern CPad* thisParam;
        extern uintptr_t callAddress;
        extern std::list<void(*)(CPad*)> funcPtrs;
        void Run();
        void MainHook();
        void Add(void(*funcPtr)(CPad*));
    }
    namespace Overrides
    {
        void GetTexture(CSprite2d(__stdcall* funcPtr)(char*));
    }
}
