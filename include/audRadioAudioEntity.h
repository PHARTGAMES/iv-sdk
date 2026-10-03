#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_AUDRADIOAUDIOENTITY_H
#define IVSDK_HEADER_AUDRADIOAUDIOENTITY_H
class audRadioAudioEntity
{
public:
	uint8_t pad[0x78];								// 00-78
	uint32_t m_nCurrentRadioStation;				// 78-7C
};
VALIDATE_OFFSET(audRadioAudioEntity, m_nCurrentRadioStation, 0x78);

extern audRadioAudioEntity& RadioAudioEntity;
#endif // IVSDK_HEADER_AUDRADIOAUDIOENTITY_H
