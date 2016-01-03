
{$L hm_i386}

Procedure ConvertX86(hci : PHermesConverterInterface); CDecl; External name ExternalAsmPrefix+'ConvertX86';
Procedure ConvertX86Stretch(hci : PHermesConverterInterface); CDecl; External name ExternalAsmPrefix+'ConvertX86Stretch';
Procedure ClearX86_32(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearX86_32';
Procedure ClearX86_24(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearX86_24';
Procedure ClearX86_16(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearX86_16';
Procedure ClearX86_8(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearX86_8';

Function Hermes_X86_CPU : Integer; CDecl; External name ExternalAsmPrefix+'Hermes_X86_CPU';

Procedure ConvertX86p32_32BGR888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_32BGR888';
Procedure ConvertX86p32_32RGBA888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_32RGBA888';
Procedure ConvertX86p32_32BGRA888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_32BGRA888';
Procedure ConvertX86p32_24RGB888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_24RGB888';
Procedure ConvertX86p32_24BGR888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_24BGR888';
Procedure ConvertX86p32_16RGB565(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_16RGB565';
Procedure ConvertX86p32_16BGR565(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_16BGR565';
Procedure ConvertX86p32_16RGB555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_16RGB555';
Procedure ConvertX86p32_16BGR555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_16BGR555';
Procedure ConvertX86p32_8RGB332(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_8RGB332';

Procedure ConvertX86p32_16RGB565_S(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p32_16RGB565_S';

Procedure ConvertX86p16_32RGB888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_32RGB888';
Procedure ConvertX86p16_32BGR888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_32BGR888';
Procedure ConvertX86p16_32RGBA888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_32RGBA888';
Procedure ConvertX86p16_32BGRA888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_32BGRA888';
Procedure ConvertX86p16_24RGB888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_24RGB888';
Procedure ConvertX86p16_24BGR888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_24BGR888';
Procedure ConvertX86p16_16BGR565(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_16BGR565';
Procedure ConvertX86p16_16RGB555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_16RGB555';
Procedure ConvertX86p16_16BGR555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_16BGR555';
Procedure ConvertX86p16_8RGB332(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86p16_8RGB332';

Procedure CopyX86p_4byte(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'CopyX86p_4byte';
Procedure CopyX86p_3byte(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'CopyX86p_3byte';
Procedure CopyX86p_2byte(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'CopyX86p_2byte';
Procedure CopyX86p_1byte(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'CopyX86p_1byte';

Procedure ConvertX86pI8_32(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86pI8_32';
Procedure ConvertX86pI8_24(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86pI8_24';
Procedure ConvertX86pI8_16(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertX86pI8_16';
