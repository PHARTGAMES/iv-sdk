#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CRENDERPHASE_H
#define IVSDK_HEADER_CRENDERPHASE_H
// todo

// +0x24 off vftable might be type
class CRenderPhase;

// all of these inherit CRenderPhase
class CRenderPhasePreRenderViewport;
class CRenderPhaseTreeImposters;
class CRenderPhaseHeight;
class CRenderPhaseCloudGeneration;
class CRenderPhaseRainUpdate;
class CRenderPhaseSetDefaultRenderState;
class CRenderPhaseCascadeShadows;
class CRenderPhaseWarpShadow;
class CRenderPhaseMirrorReflection;
class CRenderPhaseWaterReflection;
class CRenderPhaseWaterSurface;
class CRenderPhaseReflection;
class CRenderPhaseInteriorReflection;
class CRenderPhaseDeferredLighting_SceneToGBuffer;
class CRenderPhaseDeferredLighting_LightsToScreen;
class CRenderPhaseDrawScene;
class CRenderPhasePostRenderViewport;
class CRenderPhaseRadar;
#endif // IVSDK_HEADER_CRENDERPHASE_H
