#include "IVSDK.h"
#include "Addresses.h"

namespace plugin { eGameVersion gameVer = VERSION_NONE; }
namespace Addresses { uint32_t nProcessScriptsEventRet = 0; uint32_t nGameLoadEventRet = 0; }

CVector& LastUpdateCoors = AddressSetter::GetRef<CVector>(0x12932B0, 0xF59B00);
CCamera& TheCamera = AddressSetter::GetRef<CCamera>(0xB21A6C, 0xB488E8);
CVehicleFactoryNY*& VehicleFactory = AddressSetter::GetRef<CVehicleFactoryNY*>(0x11F5514, 0xE52DE8);
const char** RadarBlipSpriteFilenames = (const char**)AddressSetter::Get(0xC844F8, 0xC91690);
CWeaponInfo* aWeaponInfo = (CWeaponInfo*)AddressSetter::Get(0x1140A20, 0xE4A600);
CDraw& Scene = AddressSetter::GetRef<CDraw>(0xCF47E0, 0xDF8280);
audRadioAudioEntity& RadioAudioEntity = AddressSetter::GetRef<audRadioAudioEntity>(0xDA3700, 0xD71F48);
CText& TheText = AddressSetter::GetRef<CText>(0xCF4CE8, 0xDFB4C8);
CGameConfigReader*& GameConfigReader = AddressSetter::GetRef<CGameConfigReader*>(0x15AB8E0, 0x15CE578);
CPad* Pads = (CPad*)AddressSetter::Get(0xCFB818, 0xDD8EA8);
bool& gbIplsNeededAtPosn = AddressSetter::GetRef<bool>(0x128FFA0, 0xF6E470);
CVector& gvecIplsNeededAtPosn = AddressSetter::GetRef<CVector>(0xB3BE50, 0xB49190);
CPedFactoryNY*& PedFactory = AddressSetter::GetRef<CPedFactoryNY*>(0x11E35A0, 0xE52DE0);
audEngine& AudioEngine = AddressSetter::GetRef<audEngine>(0x1316CA0, 0xCF1970);
CStore<CVehicleModelInfo>& ms_vehicleModelStore = AddressSetter::GetRef<CStore<CVehicleModelInfo>>(0xB2C14C, 0xB3E8E8);
CStore<CPedModelInfo>& ms_pedModelStore = AddressSetter::GetRef<CStore<CPedModelInfo>>(0xB2C158, 0xB3E8F4);
rage::SkyDome*& TheSkyDome = AddressSetter::GetRef<rage::SkyDome*>(0x130B040, 0x13366A8);

namespace rage
{
    HWND& g_pHWND = AddressSetter::GetRef<HWND>(0x1449DDC, 0x1352060);
    IDirect3DDevice9*& g_pDirect3DDevice = AddressSetter::GetRef<IDirect3DDevice9*>(0x148AB48, 0x1345630);
    grcTextureFactoryPC*& TextureFactory = AddressSetter::GetRef<grcTextureFactoryPC*>(0x14A8630, 0x14CAD4C);
}
