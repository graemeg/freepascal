{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2001-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:

    First version of this unit.
    13 Mar 2001.

    Updated to use fpc 1.0.7
    Added array of const and some
    overlay functions.
    08 Jan 2003

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    12 Jan 2003.

    Changed cardinal > longword.
    Changed startcode for unit.
    11 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$mode objfpc}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

UNIT IDENTIFY;

INTERFACE
USES Exec, utility;

{
** $VER: identify.h 13.0 (11.09.2001)
**
** identify.library definitions
**
** (C) Copyright 1996-2000 Richard Koerber
** All Rights Reserved.
}

CONST  _IDTAGS      = $CD450000;

{ --------------------------------------------------------------------------}
{ Generic library informations }

CONST  IDENTIFYVERSION = 13;
       IDENTIFYNAME    : PChar = 'identify.library';

TYPE   pIdentifyBase = ^tIdentifyBase;
       TIdentifyBase  = Record
          ifyb_LibNode  :tLibrary;
       End;

CONST  IDENTIFYBUFLEN = 50;  { default buffer length }

{ --------------------------------------------------------------------------}
{ Expansion() tags }

       IDTAG_ConfigDev   = _IDTAGS+$00; { "p_ConfigDev" ConfigDev }
                                        { structure to be evaluated }
       IDTAG_ManufID     = _IDTAGS+$01; { UWORD manufacturer ID if no }
                                        { ConfigDev is available }
       IDTAG_ProdID      = _IDTAGS+$02; { UBYTE product ID if no }
                                        { ConfigDev is available }
       IDTAG_StrLength   = _IDTAGS+$03; { UWORD of maximum buffer length, }
                                        { including termination. Default }
                                        { is 50. }
       IDTAG_ManufStr    = _IDTAGS+$04; { STRPTR of manufacturer name }
                                        { puffer, or NULL }
       IDTAG_ProdStr     = _IDTAGS+$05; { STRPTR of product name }
                                        { puffer, or NULL }
       IDTAG_ClassStr    = _IDTAGS+$06; { STRPTR of product class }
                                        { puffer, or NULL }
       IDTAG_DeadStr     = _IDTAGS+$07; { STRPTR deadend or recoverable alert? }
       IDTAG_SubsysStr   = _IDTAGS+$08; { STRPTR alert subsystem }
       IDTAG_GeneralStr  = _IDTAGS+$09; { STRPTR alert general cause }
       IDTAG_SpecStr     = _IDTAGS+$0A; { STRPTR alert specific cause }
       IDTAG_FuncNameStr = _IDTAGS+$0B; { STRPTR function name }
       IDTAG_Expansion   = _IDTAGS+$0C; { "pp_ConfigDev" ConfigDev for a }
                                        { complete expansion check. Init }
                                        { the variable with NULL and pass }
                                        { a pointer to it using this tag. }
       IDTAG_Secondary   = _IDTAGS+$0D; { BOOL warn for secondary expansion }
                                        { boards (defaults to FALSE) }
       IDTAG_ClassID     = _IDTAGS+$0E; { ULONG * class ID of the provided }
                                        { expansion board (see below) [V8] }
       IDTAG_Localize    = _IDTAGS+$0F; { BOOL return localized strings }
                                        { (defaults to TRUE) [V8] }
       IDTAG_NULL4NA     = _IDTAGS+$10; { BOOL return NULL for not available, }
                                        { else return a string (defaults to }
                                        { FALSE) [V8] }

{ --------------------------------------------------------------------------}
{ Hardware description types }

       IDHW_SYSTEM      = 0;   { System (Amiga,DraCo,...)            [V2] }
       IDHW_CPU         = 1;   { CPU (68000,68010,...,68060) }
       IDHW_FPU         = 2;   { FPU (---,68881,68882,68040,68060) }
       IDHW_MMU         = 3;   { MMU (---,68852,68030,68040,68060) }
       IDHW_OSVER       = 4;   { OS Version (Vx.x) }
       IDHW_EXECVER     = 5;   { Exec Version (Vx.x) }
       IDHW_WBVER       = 6;   { Workbench Version (---,Vx.x) }
       IDHW_ROMSIZE     = 7;   { OS ROM Size (xKB, xMB) }
       IDHW_CHIPSET     = 8;   { Chipset (OCS,ECS,AGA,DraCo) }
       IDHW_GFXSYS      = 9;   { Graphic OS (AmigaOS, CyberGraphX, ...) }
       IDHW_CHIPRAM     = 10;  { Chip RAM (xKB, xMB, xGB) }
       IDHW_FASTRAM     = 11;  { Fast RAM (xKB, xMB, xGB) }
       IDHW_RAM         = 12;  { Total RAM (xKB, xMB, xGB) }
       IDHW_SETPATCHVER = 13;  { SetPatch Version (---,Vx.x)         [V4] }
       IDHW_AUDIOSYS    = 14;  { Audio OS (AmigaOS, AHI, ...)        [V5] }
       IDHW_OSNR        = 15;  { AmigaOS (2.04, 3.1, ...) }
       IDHW_VMMCHIPRAM  = 16;  { VMM Chip RAM ( *KB, *MB, *GB) }
       IDHW_VMMFASTRAM  = 17;  { VMM Fast RAM ( *KB, *MB, *GB) }
       IDHW_VMMRAM      = 18;  { VMM Total RAM ( *KB, *MB, *GB) }
       IDHW_PLNCHIPRAM  = 19;  { Plain Chip RAM ( *KB, *MB, *GB) }
       IDHW_PLNFASTRAM  = 20;  { Plain Fast RAM ( *KB, *MB, *GB) }
       IDHW_PLNRAM      = 21;  { Plain Total RAM (*KB, *MB, *GB) }
       IDHW_VBR         = 22;  { Vector Base Register                [V6] }
       IDHW_LASTALERT   = 23;  { Last Alert code }
       IDHW_VBLANKFREQ  = 24;  { VBlank Frequency }
       IDHW_POWERFREQ   = 25;  { Power Frequency }
       IDHW_ECLOCK      = 26;  { EClock }
       IDHW_SLOWRAM     = 27;  { Plain Slow RAM ( *KB, *MB, *GB) }
       IDHW_GARY        = 28;  { Gary (---,Normal,...) }
       IDHW_RAMSEY      = 29;  { RAMSEY (---,D,F) }
       IDHW_BATTCLOCK   = 30;  { Battery Backed Up Clock (---,Found) }
       IDHW_CHUNKYPLANAR = 31; { Chunky To Planar Hardware (---,Found) [V7] }
       IDHW_POWERPC     = 32;  { PowerPC present? (---,Found) }
       IDHW_PPCCLOCK    = 33;  { PowerPC clock (unit MHz) }
       IDHW_CPUREV      = 34;  { CPU revision                          [V8] }
       IDHW_CPUCLOCK    = 35;  { CPU clock (unit MHz) }
       IDHW_FPUCLOCK    = 36;  { FPU clock (unit MHz) }
       IDHW_RAMACCESS   = 37;  { Main board RAM access time (unit ns) }
       IDHW_RAMWIDTH    = 38;  { Main board RAM width (bit) }
       IDHW_RAMCAS      = 39;  { Main board RAM CAS mode }
       IDHW_RAMBANDWIDTH = 40; { Main board RAM bandwidth }
       IDHW_TCPIP       = 41;  { TCP/IP stack                          [V9] }
       IDHW_PPCOS       = 42;  { PowerPC OS }
       IDHW_AGNUS       = 43;  { Agnus chip revision }
       IDHW_AGNUSMODE   = 44;  { Agnus chip mode }
       IDHW_DENISE      = 45;  { Denise chip version                   [V10] }
       IDHW_DENISEREV   = 46;  { Denise chip revision }
       IDHW_BOINGBAG    = 47;  { BoingBag number                       [V12] }
       IDHW_NUMBEROF    = 48;  { Number of types, PRIVATE! }

{ --------------------------------------------------------------------------}
{ IDHW_SYSTEM numerical result codes }

       IDSYS_AMIGA1000  = 0;    { Amiga 1000 }
       IDSYS_AMIGAOCS   = 1;    { OCS Amiga 500/2000 }
       IDSYS_AMIGAECS   = 2;    { ECS Amiga 500/2000 }
       IDSYS_AMIGA500   = 3;    { Amiga 500 }
       IDSYS_AMIGA2000  = 4;    { Amiga 2000 }
       IDSYS_AMIGA3000  = 5;    { Amiga 3000 }
       IDSYS_CDTV       = 6;    { CDTV }
       IDSYS_AMIGA600   = 7;    { Amiga 600 }
       IDSYS_CD32       = 8;    { CD32 }
       IDSYS_AMIGA1200  = 9;    { Amiga 1200 }
       IDSYS_AMIGA4000  = 10;   { Amiga 4000 }
       IDSYS_DRACO      = 11;   { DraCo }
       IDSYS_UAE        = 12;   { Ubiquitous Amiga Emulator }
       IDSYS_AMIGA4000T = 13;   { Amiga 4000 Tower }

{ --------------------------------------------------------------------------}
{ IDHW_CPU numerical result codes }

       IDCPU_68000     = 0;     { 68000 }
       IDCPU_68010     = 1;     { 68010 }
       IDCPU_68020     = 2;     { 68020 }
       IDCPU_68030     = 3;     { 68030 }
       IDCPU_68EC030   = 4;     { 68EC030 (without MMU) }
       IDCPU_68040     = 5;     { 68040 }
       IDCPU_68LC040   = 6;     { 68LC040 (without FPU) }
       IDCPU_68060     = 7;     { 68060 }
       IDCPU_68LC060   = 8;     { 68LC060 (without FPU) }

{ --------------------------------------------------------------------------}
{ IDHW_FPU numerical result codes }

       IDFPU_NONE      = 0;     { no FPU }
       IDFPU_68881     = 1;     { 68881 }
       IDFPU_68882     = 2;     { 68882 }
       IDFPU_68040     = 3;     { 68040 }
       IDFPU_68060     = 4;     { 68060 }

{ --------------------------------------------------------------------------}
{ IDHW_MMU numerical result codes }

       IDMMU_NONE      = 0;     { no MMU }
       IDMMU_68851     = 1;     { 68851 }
       IDMMU_68030     = 2;     { 68030 }
       IDMMU_68040     = 3;     { 68040 }
       IDMMU_68060     = 4;     { 68060 }

{ --------------------------------------------------------------------------}
{ IDHW_OSNR numerical result codes }

       IDOS_UNKNOWN    = 0;     { <V36 or >V40 }
       IDOS_2_0        = 1;     { OS2.0  (V36) }
       IDOS_2_04       = 2;     { OS2.04 (V37) }
       IDOS_2_1        = 3;     { OS2.1  (V38) }
       IDOS_3_0        = 4;     { OS3.0  (V39) }
       IDOS_3_1        = 5;     { OS3.1  (V40) }
       IDOS_3_5        = 6;     { OS3.5  (V44) }
       IDOS_3_9        = 7;     { OS3.9  (V45) }

{ --------------------------------------------------------------------------}
{ IDHW_CHIPSET numerical result codes }

       IDCS_OCS        = 0;     { OCS }
       IDCS_ECS        = 1;     { ECS }
       IDCS_AGA        = 2;     { AGA }
       IDCS_ALTAIS     = 3;     { DraCo Altais }
       IDCS_NECS       = 4;     { Nearly ECS (no HR-Agnus or HR-Denise) }

{ --------------------------------------------------------------------------}
{ IDHW_GFXSYS numerical result codes }

       IDGOS_AMIGAOS   = 0;     { Plain AmigaOS }
       IDGOS_EGS       = 1;     { EGS }
       IDGOS_RETINA    = 2;     { Retina }
       IDGOS_GRAFFITI  = 3;     { Graffiti }
       IDGOS_TIGA      = 4;     { TIGA }
       IDGOS_PROBENCH  = 5;     { Merlin ProBench }
       IDGOS_PICASSO   = 6;     { Picasso }
       IDGOS_CGX       = 7;     { CyberGraphX }
       IDGOS_CGX3      = 8;     { CyberGraphX 3D }
       IDGOS_PICASSO96 = 9;     { Picasso96 }
       IDGOS_CGX4      = 10;    { CyberGraphX V4 }

{ --------------------------------------------------------------------------}
{ IDHW_AUDIOSYS numerical result codes }

       IDAOS_AMIGAOS   = 0;     { Plain AmigaOS }
       IDAOS_MAESTIX   = 1;     { MaestroPro driver }
       IDAOS_TOCCATA   = 2;     { Toccata driver }
       IDAOS_PRELUDE   = 3;     { Prelude driver }
       IDAOS_AHI       = 4;     { AHI }
       IDAOS_MACROAUDIO = 5;    { MacroAudio }

{ --------------------------------------------------------------------------}
{ IDHW_GARY numerical result codes }

       IDGRY_NONE      = 0;     { No Gary available }
       IDGRY_NORMAL    = 1;     { Normal Gary }
       IDGRY_ENHANCED  = 2;     { Enhanced Gary }

{ --------------------------------------------------------------------------}
{ IDHW_RAMSEY numerical result codes }

       IDRSY_NONE      = 0;     { No RamSey available }
       IDRSY_REVD      = 1;     { RamSey Revision D }
       IDRSY_REVF      = 2;     { RamSey Revision F }

{ --------------------------------------------------------------------------}
{ IDHW_POWERPC numerical result codes }

       IDPPC_NONE      = 0;     {  No PowerPC implemented }
       IDPPC_OTHER     = 1;     {  Another PowerPC }
       IDPPC_602       = 2;     {  602 }
       IDPPC_603       = 3;     {  603 }
       IDPPC_603E      = 4;     {  603e }
       IDPPC_603P      = 5;     {  603p }
       IDPPC_604       = 6;     {  604 }
       IDPPC_604E      = 7;     {  604e }
       IDPPC_620       = 8;     {  620 }

{ --------------------------------------------------------------------------}
{ IDHW_RAMCAS numerical result codes }

       IDCAS_NONE      = 0;     {  Not available }
       IDCAS_NORMAL    = 1;     {  Normal access }
       IDCAS_DOUBLE    = 2;     {  Double access }

{ --------------------------------------------------------------------------}
{ IDHW_TCPIP numerical result codes }

       IDTCP_NONE      = 0;     {  Not available }
       IDTCP_AMITCP    = 1;     {  AmiTCP }
       IDTCP_MIAMI     = 2;     {  Miami }
       IDTCP_TERMITE   = 3;     {  TermiteTCP }
       IDTCP_GENESIS   = 4;     {  GENESiS }
       IDTCP_MIAMIDX   = 5;     {  MiamiDx }

{ --------------------------------------------------------------------------}
{ IDHW_PPCOS numerical result codes }

       IDPOS_NONE      = 0;     {  None }
       IDPOS_POWERUP   = 1;     {  PowerUP (Phase 5) }
       IDPOS_WARPOS    = 2;     {  WarpOS (Haage&Partner) }

{ --------------------------------------------------------------------------}
{ IDHW_AGNUS numerical result codes }

       IDAG_NONE       = 0;     {  None }
       IDAG_UNKNOWN    = 1;     {  Unknown Agnus }
       IDAG_8361       = 2;     {  Agnus 8361 }
       IDAG_8367       = 3;     {  Agnus 8367 }
       IDAG_8370       = 4;     {  Agnus 8370 }
       IDAG_8371       = 5;     {  Agnus 8371 }
       IDAG_8372_R4    = 6;     {  Agnus 8372 Rev. 1-4 }
       IDAG_8372_R5    = 7;     {  Agnus 8372 Rev. 5 }
       IDAG_8374_R2    = 8;     {  Alice 8374 Rev. 1-2 }
       IDAG_8374_R3    = 9;     {  Alice 8374 Rev. 3-4 }

{ --------------------------------------------------------------------------}
{ IDHW_DENISE numerical result codes }

       IDDN_NONE       = 0;     {  None }
       IDDN_UNKNOWN    = 1;     {  Unknown Denise (new model?! :-) }
       IDDN_8362       = 2;     {  Denise 8362 }
       IDDN_8373       = 3;     {  Denise 8373 }
       IDDN_8364       = 4;     {  Lisa 8364 }

{ --------------------------------------------------------------------------}
{ IDHW_AGNUSMODE numerical result codes }

       IDAM_NONE       = 0;     {  None }
       IDAM_NTSC       = 1;     {  NTSC Agnus }
       IDAM_PAL        = 2;     {  PAL Agnus }

{ --------------------------------------------------------------------------}
{ IDTAG_ClassID numerical result codes }

       IDCID_UNKNOWN     = 0;   {  unknown functionality }
       IDCID_MISC        = 1;   {  miscellaneous expansion }
       IDCID_TURBO       = 2;   {  accelerator }
       IDCID_TURBORAM    = 3;   {  accelerator RAM }
       IDCID_TURBOHD     = 4;   {  accelerator + some HD controller }
       IDCID_TURBOIDE    = 5;   {  accelerator + AT-IDE controller }
       IDCID_TURBOSCSIHD = 6;   {  accelerator + SCSI host adapter }
       IDCID_NET         = 7;   {  some network }
       IDCID_ARCNET      = 8;   {  ArcNet }
       IDCID_ETHERNET    = 9;   {  Ethernet }
       IDCID_AUDIO       = 10;  {  audio }
       IDCID_VIDEO       = 11;  {  video (video digitizer etc) }
       IDCID_MULTIMEDIA  = 12;  {  multimedia (audio, video, etc) }
       IDCID_DSP         = 13;  {  multi purpose DSP }
       IDCID_BUSBRIDGE   = 14;  {  ISA bus bridge }
       IDCID_BRIDGE      = 15;  {  PC bridge }
       IDCID_EMULATOR    = 16;  {  some Emulator hardware }
       IDCID_MACEMU      = 17;  {  Macintosh emulator hardware }
       IDCID_ATARIEMU    = 18;  {  Atari emulator hardware }
       IDCID_GFX         = 19;  {  graphics (register) }
       IDCID_GFXRAM      = 20;  {  graphics (video RAM) }
       IDCID_HD          = 21;  {  some HD controller }
       IDCID_HDRAM       = 22;  {  some HD controller + RAM }
       IDCID_IDEHD       = 23;  {  IDE controller }
       IDCID_IDEHDRAM    = 24;  {  IDE controller + RAM }
       IDCID_SCSIHD      = 25;  {  SCSI host adapter }
       IDCID_RAMSCSIHD   = 26;  {  SCSI host adapter + RAM }
       IDCID_CDROM       = 27;  {  CD-ROM interface }
       IDCID_MODEM       = 28;  {  internal modem }
       IDCID_ISDN        = 29;  {  internal ISDN terminal adapter }
       IDCID_MULTIIO     = 30;  {  multi I/O (serial + parallel) }
       IDCID_SERIAL      = 31;  {  multi serial (no parallel) }
       IDCID_PARALLEL    = 32;  {  multi parallel (no serial) }
       IDCID_SCANIF      = 33;  {  scanner interface }
       IDCID_TABLET      = 34;  {  drawing tablet interface }
       IDCID_RAM         = 35;  {  plain RAM expansion }
       IDCID_FLASHROM    = 36;  {  Flash ROM }
       IDCID_SCSIIDE     = 37;  {  combined SCSI/IDE controller }
       IDCID_RAMFPU      = 38;  {  RAM expansion + FPU }
       IDCID_GUESS       = 39;  {  name was unknown but guessed }
       IDCID_KICKSTART   = 40;  {  KickStart }
       IDCID_RAM32       = 41;  {  32bit RAM expansion }
       IDCID_TURBOANDRAM = 42;  {  accelerator + RAM expansion }
       IDCID_ACCESS      = 43;  {  access control }
       IDCID_INTERFACE   = 44;  {  some interface }
       IDCID_MFMHD       = 45;  {  MFM HD controller }
       IDCID_FLOPPY      = 46;  {  floppy disk controller }
        { Be prepared to get other numbers as well. In this case you }
        { should assume IDCID_MISC. }

{ --------------------------------------------------------------------------}
{ Error codes }

{ Positive error codes are DOS errors! }
       IDERR_OKAY = 0;       { No error }
       IDERR_NOLENGTH = -1;  { Buffer length is 0 ?? }
       IDERR_BADID = -2;     { Missing or bad board ID }
       IDERR_NOMEM = -3;     { Not enough memory }
       IDERR_NOFD = -4;      { No fitting FD file found }
       IDERR_OFFSET = -5;    { Function offset not found }
       IDERR_DONE = -6;      { Done with traversing (not an error) }
       IDERR_SECONDARY = -7; { Secondary expansion board (not an error) }

{ --------------------------------------------------------------------------}
{ That's all... }




VAR IdentifyBase : pLibrary;

FUNCTION IdExpansion(TagList : pTagItem) : LONGINT;
FUNCTION IdHardware(Type_ : Ulong; TagList : pTagItem) : pCHAR;
FUNCTION IdAlert(ID : Ulong; TagList : pTagItem) : LONGINT;
FUNCTION IdFunction(LibName : pCHAR; Offset : LONGINT; TagList : pTagItem) : LONGINT;
FUNCTION IdHardwareNum(Type_ : Ulong; TagList : pTagItem) : Ulong;
PROCEDURE IdHardwareUpdate;
FUNCTION IdFormatString(String_ : pCHAR; Buffer : pCHAR; Length : Ulong; Tags : pTagItem) : Ulong;
FUNCTION IdEstimateFormatSize(String_ : pCHAR; Tags : pTagItem) : Ulong;

{
     This is functions and procedures with array of const.
     For use with fpc 1.0 and above.
}

FUNCTION IdExpansionTags(const TagList : Array Of Const) : LONGINT;
FUNCTION IdHardwareTags(Type_ : longword; const TagList : Array Of Const) : pCHAR;
FUNCTION IdAlertTags(ID : longword; const TagList : Array Of Const) : LONGINT;
FUNCTION IdFunctionTags(LibName : pCHAR; Offset : LONGINT; const TagList : Array Of Const) : LONGINT;
FUNCTION IdHardwareNumTags(Type_ : longword; const TagList : Array Of Const) : longword;
FUNCTION IdFormatStringTags(String_ : pCHAR; Buffer : pCHAR; Length : longword; const Tags : Array Of Const) : longword;
FUNCTION IdEstimateFormatSizeTags(String_ : pCHAR; const Tags : Array Of Const) : longword;

{
     Overlay functions
}

FUNCTION IdFunction(LibName : string; Offset : LONGINT; TagList : pTagItem) : LONGINT;
FUNCTION IdFormatString(String_ : string; Buffer : pCHAR; Length : Ulong; Tags : pTagItem) : Ulong;
FUNCTION IdEstimateFormatSize(String_ : string; Tags : pTagItem) : Ulong;
FUNCTION IdFunctionTags(LibName : string; Offset : LONGINT; const TagList : Array Of Const) : LONGINT;
FUNCTION IdFormatStringTags(String_ : string; Buffer : pCHAR; Length : longword; const Tags : Array Of Const) : longword;
FUNCTION IdEstimateFormatSizeTags(String_ : string; const Tags : Array Of Const) : longword;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitIDENTIFYLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    IDENTIFYIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray,pastoc;

FUNCTION IdExpansion(TagList : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L TagList,A0
        MOVEA.L IdentifyBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IdHardware(Type_ : Ulong; TagList : pTagItem) : pCHAR;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Type_,D0
        MOVEA.L TagList,A0
        MOVEA.L IdentifyBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IdAlert(ID : Ulong; TagList : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  ID,D0
        MOVEA.L TagList,A0
        MOVEA.L IdentifyBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IdFunction(LibName : pCHAR; Offset : LONGINT; TagList : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LibName,A0
        MOVE.L  Offset,D0
        MOVEA.L TagList,A1
        MOVEA.L IdentifyBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IdHardwareNum(Type_ : Ulong; TagList : pTagItem) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Type_,D0
        MOVEA.L TagList,A0
        MOVEA.L IdentifyBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE IdHardwareUpdate;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L IdentifyBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION IdFormatString(String_ : pCHAR; Buffer : pCHAR; Length : Ulong; Tags : pTagItem) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L String_,A0
        MOVEA.L Buffer,A1
        MOVE.L  Length,D0
        MOVEA.L Tags,A2
        MOVEA.L IdentifyBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IdEstimateFormatSize(String_ : pCHAR; Tags : pTagItem) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L String_,A0
        MOVEA.L Tags,A1
        MOVEA.L IdentifyBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION IdExpansionTags(const TagList : Array Of Const) : LONGINT;
begin
    IdExpansionTags := IdExpansion(readintags(TagList));
end;

FUNCTION IdHardwareTags(Type_ : longword; const TagList : Array Of Const) : pCHAR;
begin
    IdHardwareTags := IdHardware(Type_ , readintags(TagList));
end;

FUNCTION IdAlertTags(ID : longword; const TagList : Array Of Const) : LONGINT;
begin
    IdAlertTags := IdAlert(ID , readintags(TagList));
end;

FUNCTION IdFunctionTags(LibName : pCHAR; Offset : LONGINT; const TagList : Array Of Const) : LONGINT;
begin
    IdFunctionTags := IdFunction(LibName , Offset , readintags(TagList));
end;

FUNCTION IdHardwareNumTags(Type_ : longword; const TagList : Array Of Const) : longword;
begin
    IdHardwareNumTags := IdHardwareNum(Type_ , readintags(TagList));
end;

FUNCTION IdFormatStringTags(String_ : pCHAR; Buffer : pCHAR; Length : longword; const Tags : Array Of Const) : longword;
begin
    IdFormatStringTags := IdFormatString(String_ , Buffer , Length , readintags(Tags));
end;

FUNCTION IdEstimateFormatSizeTags(String_ : pCHAR; const Tags : Array Of Const) : longword;
begin
    IdEstimateFormatSizeTags := IdEstimateFormatSize(String_ , readintags(Tags));
end;

{
     Overlay functions
}

FUNCTION IdFunction(LibName : string; Offset : LONGINT; TagList : pTagItem) : LONGINT;
begin
    IdFunction := IdFunction(pas2c(LibName),Offset,TagList);
end;

FUNCTION IdFormatString(String_ : string; Buffer : pCHAR; Length : Ulong; Tags : pTagItem) : Ulong;
begin
    IdFormatString := IdFormatString(pas2c(String_),Buffer,Length,Tags);
end;

FUNCTION IdEstimateFormatSize(String_ : string; Tags : pTagItem) : Ulong;
begin
    IdEstimateFormatSize := IdEstimateFormatSize(pas2c(String_),Tags);
end;

FUNCTION IdFunctionTags(LibName : string; Offset : LONGINT; const TagList : Array Of Const) : LONGINT;
begin
    IdFunctionTags := IdFunction(pas2c(LibName),Offset,readintags(TagList));
end;

FUNCTION IdFormatStringTags(String_ : string; Buffer : pCHAR; Length : longword; const Tags : Array Of Const) : longword;
begin
    IdFormatStringTags := IdFormatString(pas2c(String_),Buffer,Length,readintags(Tags));
end;

FUNCTION IdEstimateFormatSizeTags(String_ : string; const Tags : Array Of Const) : longword;
begin
    IdEstimateFormatSizeTags := IdEstimateFormatSize(pas2c(String_),readintags(Tags));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of identify.library}
  {$Info don't forget to use InitIDENTIFYLibrary in the beginning of your program}

var
    identify_exit : Pointer;

procedure CloseidentifyLibrary;
begin
    ExitProc := identify_exit;
    if IdentifyBase <> nil then begin
        CloseLibrary(IdentifyBase);
        IdentifyBase := nil;
    end;
end;

procedure InitIDENTIFYLibrary;
begin
    IdentifyBase := nil;
    IdentifyBase := OpenLibrary(IDENTIFYNAME,LIBVERSION);
    if IdentifyBase <> nil then begin
        identify_exit := ExitProc;
        ExitProc := @CloseidentifyLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open identify.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    IDENTIFYIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of identify.library}

var
    identify_exit : Pointer;

procedure CloseidentifyLibrary;
begin
    ExitProc := identify_exit;
    if IdentifyBase <> nil then begin
        CloseLibrary(IdentifyBase);
        IdentifyBase := nil;
    end;
end;

begin
    IdentifyBase := nil;
    IdentifyBase := OpenLibrary(IDENTIFYNAME,LIBVERSION);
    if IdentifyBase <> nil then begin
        identify_exit := ExitProc;
        ExitProc := @CloseidentifyLibrary;
        IDENTIFYIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open identify.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    IDENTIFYIsCompiledHow := 3;
   {$Warning No autoopening of identify.library compiled}
   {$Warning Make sure you open identify.library yourself}
{$endif dont_use_openlib}


END. (* UNIT IDENTIFY *)



