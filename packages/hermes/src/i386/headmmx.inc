
{$L mmx_clr}
{$L mmx_main}
{$L mmxp2_32}
{$L mmxp_32}

Procedure ConvertMMX(hci : PHermesConverterInterface); CDecl; External name ExternalAsmPrefix+'ConvertMMX';

Procedure ClearMMX_32(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearMMX_32';
Procedure ClearMMX_24(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearMMX_24';
Procedure ClearMMX_16(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearMMX_16';
Procedure ClearMMX_8(hci : PHermesClearInterface); CDecl; External name ExternalAsmPrefix+'ClearMMX_8';

Procedure ConvertMMXpII32_24RGB888(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertMMXpII32_24RGB888';
Procedure ConvertMMXpII32_16RGB565(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertMMXpII32_16RGB565';
Procedure ConvertMMXpII32_16BGR565(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertMMXpII32_16BGR565';
Procedure ConvertMMXpII32_16RGB555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertMMXpII32_16RGB555';
Procedure ConvertMMXpII32_16BGR555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertMMXpII32_16BGR555';

Procedure ConvertMMXp32_16RGB555(CONVERT_PARAMETERS); CDecl; External name ExternalAsmPrefix+'ConvertMMXp32_16RGB555';
