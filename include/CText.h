#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CTEXT_H
#define IVSDK_HEADER_CTEXT_H
class CText
{
public:
	const wchar_t* Get(const char* Ident)
	{
		return ((const wchar_t*(__thiscall*)(CText*, const char*))(AddressSetter::Get(0x3B54C0, 0x4A4000)))(this, Ident);
	}
};
extern CText& TheText;
#endif // IVSDK_HEADER_CTEXT_H
