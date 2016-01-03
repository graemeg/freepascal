<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}

=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: memory.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

{$ifdef NDS_INTERFACE}
// WAIT_CR: Wait State Control Register
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
{$ifdef ARM9}
  REG_EXMEMCNT : pcuint16 = pointer($04000204);
{$else}
  REG_EXMEMSTAT: pcuint16 = pointer($04000204);
{$endif}
  ARM7_MAIN_RAM_PRIORITY  = (1 shl 15);
  ARM7_OWNS_CARD          = (1 shl 11);
  ARM7_OWNS_ROM           = (1 shl 7);


// Protection register (write-once sadly)
{$ifdef ARM7}
  PROTECTION    : pcuint32 = pointer($04000308);
{$endif ARM7}


  ALLRAM        : pcuint8  = pointer($00000000);
  
  MAINRAM8      : pcuint8  = pointer($02000000);
  MAINRAM16     : pcuint16 = pointer($02000000);
  MAINRAM32     : pcuint32 = pointer($02000000);

// fixme: shared RAM

// GBA_BUS is volatile, while GBAROM is not
  GBA_BUS       : pcuint16 = pointer($08000000);
  GBAROM        : pcuint16 = pointer($08000000);

  SRAM          : pcuint8  = pointer($0A000000);

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

{$ifdef ARM9}
  PALETTE             : pcuint16 = pointer($05000000);
  PALETTE_SUB         : pcuint16 = pointer($05000400);

  BG_PALETTE          : pcuint16 = pointer($05000000);
  BG_PALETTE_SUB      : pcuint16 = pointer($05000400);

  SPRITE_PALETTE      : pcuint16 = pointer($05000200);
  SPRITE_PALETTE_SUB  : pcuint16 = pointer($05000600);

  BG_GFX			         : pcuint16 = pointer($06000000);
  BG_GFX_SUB		       : pcuint16 = pointer($06200000);
  SPRITE_GFX			     : pcuint16 = pointer($06400000);
  SPRITE_GFX_SUB		   : pcuint16 = pointer($06600000);

  VRAM_0        : pcuint16 = pointer($06000000);
  VRAM          : pcuint16 = pointer($06800000);
  VRAM_A        : pcuint16 = pointer($06800000);
  VRAM_B        : pcuint16 = pointer($06820000);
  VRAM_C        : pcuint16 = pointer($06840000);
  VRAM_D        : pcuint16 = pointer($06860000);
  VRAM_E        : pcuint16 = pointer($06880000);
  VRAM_F        : pcuint16 = pointer($06890000);
  VRAM_G        : pcuint16 = pointer($06894000);
  VRAM_H        : pcuint16 = pointer($06898000);
  VRAM_I        : pcuint16 = pointer($068A0000);

  OAM           : pcuint16 = pointer($07000000);
  OAM_SUB       : pcuint16 = pointer($07000400);
{$endif ARM9}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef ARM7}
  VRAM          : pcuint16 = pointer($06000000);
{$endif ARM7}

type
  sGBAHeader = packed record
    entryPoint: cuint32;
    logo: array [0..155] of cuint8;
    title: array [0..11] of cchar;
    gamecode: array [0..3] of cchar;
    makercode: cuint16;
    is96h: cuint8;
    unitcode: cuint8;
    devicecode: cuint8;
    unused: array [0..6] of cuint8;
    version: cuint8;
    complement: cuint8;
    checksum: cuint16;
  end; 
  TGBAHeader = sGBAHeader;
  PGBAHeader = ^sGBAHeader;

const
  GBA_HEADER : pGBAHeader = pointer($08000000);

type
  sNDSHeader = packed record
    gameTitle: array [0..11] of cchar;
    gameCode: array [0..3] of cchar;
    makercode: array [0..1] of cchar;
    unitCode: cuint8;
    deviceType: cuint8;           // type of device in the game card
    deviceSize: cuint8;           // device capacity (1<<n Mbit)
    reserved1: array [0..8] of cuint8;
    romversion: cuint8;
    flags: cuint8;                // auto-boot flag
    
<<<<<<< HEAD
<<<<<<< HEAD
    arm9romOffset: cuint32;
=======
    arm9romSource: cuint32;
>>>>>>> graemeg/fixes_2_2
=======
    arm9romSource: cuint32;
>>>>>>> origin/fixes_2_2
    arm9executeAddress: cuint32;
    arm9destination: cuint32;
    arm9binarySize: cuint32;
    
<<<<<<< HEAD
<<<<<<< HEAD
    arm7romOffset: cuint32;
=======
    arm7romSource: cuint32;
>>>>>>> graemeg/fixes_2_2
=======
    arm7romSource: cuint32;
>>>>>>> origin/fixes_2_2
    arm7executeAddress: cuint32;
    arm7destination: cuint32;
    arm7binarySize: cuint32;
    
<<<<<<< HEAD
<<<<<<< HEAD
    filenameOffset: cuint32;
    filenameSize: cuint32;
    fatOffset: cuint32;
=======
    filenameSource: cuint32;
    filenameSize: cuint32;
    fatSource: cuint32;
>>>>>>> graemeg/fixes_2_2
=======
    filenameSource: cuint32;
    filenameSize: cuint32;
    fatSource: cuint32;
>>>>>>> origin/fixes_2_2
    fatSize: cuint32;
    
    arm9overlaySource: cuint32;
    arm9overlaySize: cuint32;
    arm7overlaySource: cuint32;
    arm7overlaySize: cuint32;
    
    cardControl13: cuint32;  // used in modes 1 and 3
    cardControlBF: cuint32;  // used in mode 2
    bannerOffset: cuint32;
    
    secureCRC16: cuint16;
    
    readTimeout: cuint16;
    
    unknownRAM1: cuint32;
    unknownRAM2: cuint32;
    
    bfPrime1: cuint32;
    bfPrime2: cuint32;
    romSize: cuint32;
    
    headerSize: cuint32;
    zeros88: array [0..13] of cuint32;
    gbaLogo: array [0..155] of cuint8;
    logoCRC16: cuint16;
    headerCRC16: cuint16;
    
    debugRomSource: cuint32;
    debugRomSize: cuint32;
    debugRomDestination: cuint32;
    offset_0x16C: cuint32;
    
    zero: array [0..143] of cuint8;
  end;
  tNDSHeader = sNDSHeader;
  pNDSHeader = ^tNDSHeader;
  
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  __NDSHeader : pNDSHeader = pointer($02FFFE00);
=======
  NDSHeader : pNDSHeader = pointer($027FFE00);
>>>>>>> graemeg/fixes_2_2
=======
  NDSHeader : pNDSHeader = pointer($027FFE00);
>>>>>>> origin/fixes_2_2
=======
  __NDSHeader : pNDSHeader = pointer($02FFFE00);
>>>>>>> origin/fixes_2.4


type
  sNDSBanner = packed record
    version: cuint16;
    crc: cuint16;
    reserved: array [0..27] of cuint8;
    icon: array [0..511] of cuint8;
    palette: array [0..15] of cuint16;
    titles: array [0..5, 0..127] of cuint16;
  end;
  tNDSBanner = sNDSBanner;
  pNDSBanner = ^tNDSBanner;
{$endif NDS_INTERFACE}


{$ifdef ARM9}
{$ifdef NDS_INTERFACE}
const
  BUS_OWNER_ARM9 = true;
  BUS_OWNER_ARM7 = false;
<<<<<<< HEAD
<<<<<<< HEAD
  
procedure sysSetCartOwner(arm9: cbool); inline; 
procedure sysSetCardOwner(arm9: cbool); inline;
procedure sysSetBusOwners(arm9rom, arm9card: cbool); inline;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
// Changes only the gba rom bus ownership
procedure sysSetCartOwner(arm9: cbool); inline; 
var
  i: cint;
begin
  if arm9 then
    i := 0
  else
    i := ARM7_OWNS_ROM;
  REG_EXMEMCNT^ := (REG_EXMEMCNT^ and not ARM7_OWNS_ROM) or (i);
end;

// Changes only the nds card bus ownership
procedure sysSetCardOwner(arm9: cbool); inline;
var
  i: cint;
begin
  if arm9 then
    i := 0
  else
    i := ARM7_OWNS_ROM;
  REG_EXMEMCNT^ := (REG_EXMEMCNT^ and not ARM7_OWNS_CARD) or (i);
end;

// Changes all bus ownerships
procedure sysSetBusOwners(arm9rom, arm9card: cbool); inline;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  val1, val2: cuint;
begin
  val1 := 0;
  val2 := 0;
  if not arm9card then
    val1 := ARM7_OWNS_CARD; 
  
  if not arm9rom then
    val2 := ARM7_OWNS_ROM ;
    
  REG_EXMEMCNT^ := (REG_EXMEMCNT^ and not(ARM7_OWNS_CARD or ARM7_OWNS_ROM)) or val1 or val2;
end;


{$endif NDS_IMPLEMENTATION}
{$endif ARM9}
=======
=======
>>>>>>> origin/fixes_2_2
  pattern: cuint16;
  a9r, a9c: cint;
=======
  val1, val2: cuint;
>>>>>>> origin/fixes_2.4
begin
  val1 := 0;
  val2 := 0;
  if not arm9card then
    val1 := ARM7_OWNS_CARD; 
  
  if not arm9rom then
    val2 := ARM7_OWNS_ROM ;
    
  REG_EXMEMCNT^ := (REG_EXMEMCNT^ and not(ARM7_OWNS_CARD or ARM7_OWNS_ROM)) or val1 or val2;
end;


{$endif NDS_IMPLEMENTATION}
{$endif ARM9}

{$ifdef NDS_INTERFACE}
{$ifdef ARM9}
procedure sysSetCartOwner(arm9: cbool); inline; 
procedure sysSetCardOwner(arm9: cbool); inline;
procedure sysSetBusOwners(arm9rom, arm9card: cbool); inline;
{$endif ARM9}
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
