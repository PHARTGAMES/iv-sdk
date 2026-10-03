#pragma once
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <list>
#include <exception>
//#ifndef NOMINMAX
//#define NOMINMAX
//#endif
#include <windows.h>
#include <d3d9.h>

#define VALIDATE_SIZE(struc, size) static_assert(sizeof(struc) == size, "Invalid structure size of " #struc)
#define VALIDATE_OFFSET(struc, member, offset) \
    static_assert(offsetof(struc, member) == offset, "The offset of " #member " in " #struc " is not " #offset "...")

namespace plugin
{
    enum eGameVersion { VERSION_NONE, VERSION_1070, VERSION_1080 };
    extern eGameVersion gameVer;
    // Call once during plugin startup, before registering callbacks.
    bool Init();
    // Restores SDK CALL hooks. Caller must ensure no hook/callback is executing.
    void Deinit();
    bool IsInitialized();
    // Legacy application callbacks: defined/called by the application, not the SDK.
    void gameStartupEvent();
    void gameShutdownEvent();
}
