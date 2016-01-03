(*
  $Id: fat.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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
<<<<<<< HEAD
<<<<<<< HEAD
function fatInit(cacheSize: cuint32; setAsDefaultDevice: cbool): cbool; cdecl; external;
function fatInitDefault(): cbool; cdecl; external;
function fatMountSimple (const name: pcchar; const ainterface: PDISC_INTERFACE): cbool; cdecl; external;
function fatMount (const name: pcchar; const ainterface: PDISC_INTERFACE; startSector, cacheSize, SectorsPerPage: cuint32): cbool; cdecl; external;
function fatUnmount(const name: pcchar): cbool; cdecl; external;

// Get Volume Label
procedure fatGetVolumeLabel(const name: pcchar; labl: pcchar); cdecl; external;

// File attributes
const
  ATTR_ARCHIVE    = $20;  // Archive
  ATTR_DIRECTORY  = $10;  // Directory
  ATTR_VOLUME     = $08;  // Volume
  ATTR_SYSTEM     = $04;  // System
  ATTR_HIDDEN     = $02;  // Hidden
  ATTR_READONLY   = $01;  // Read only


// Methods to modify DOS File Attributes

function FAT_getAttr(const _file: pcchar): cint; cdecl; external;
function FAT_setAttr(const _file: pcchar; attr: cint): cint; cdecl; external;

{$define LIBFAT_FEOS_MULTICWD}
=======
=======
>>>>>>> origin/fixes_2_2

const
  FEATURE_MEDIUM_CANREAD		= $00000001;
  FEATURE_MEDIUM_CANWRITE		= $00000002;
  FEATURE_SLOT_GBA			= $00000010;
  FEATURE_SLOT_NDS			= $00000020;

type
  FN_MEDIUM_STARTUP = function: cbool;
  FN_MEDIUM_ISINSERTED = function: cbool;
  FN_MEDIUM_READSECTORS = function(sector, numSectors: cuint32; buffer: pointer): cbool;
  FN_MEDIUM_WRITESECTORS = function(sector, numSectors: cuint32; const buffer: pointer): cbool;
  FN_MEDIUM_CLEARSTATUS = function: cbool;
  FN_MEDIUM_SHUTDOWN = function: cbool;

  IO_INTERFACE_STRUCT = record
    ioType: cuint32;
    features: cuint32;
    fn_startup: FN_MEDIUM_STARTUP;
    fn_isInserted: FN_MEDIUM_ISINSERTED;
    fn_readSectors: FN_MEDIUM_READSECTORS;
    fn_writeSectors: FN_MEDIUM_WRITESECTORS;
    fn_clearStatus: FN_MEDIUM_CLEARSTATUS;
    fn_shutdown: FN_MEDIUM_SHUTDOWN;
  end;

  IO_INTERFACE = IO_INTERFACE_STRUCT;
  TIO_INTERFACE_STRUCT = IO_INTERFACE_STRUCT;
  PIO_INTERFACE_STRUCT = ^IO_INTERFACE_STRUCT;

  PARTITION_INTERFACE = (
    PI_DEFAULT = 0,
    PI_SLOT_1, 
    PI_SLOT_2, 
    PI_CUSTOM
  );


function fatInit(cacheSize: cuint32; setAsDefaultDevice: cbool): cbool; cdecl; external;
function fatInitDefault(): cbool; cdecl; external;
function fatMountNormalInterface(partitionNumber: PARTITION_INTERFACE; cacheSize: cuint32): cbool; cdecl; external;
function fatMountCustomInterface(const device: PIO_INTERFACE_STRUCT; cacheSize: cuint32): cbool; cdecl; external;
function fatUnmount(partitionNumber: PARTITION_INTERFACE): cbool; cdecl; external;
function fatUnsafeUnmount(partitionNumber: PARTITION_INTERFACE): cbool; cdecl; external;
function fatSetDefaultInterface(partitionNumber: PARTITION_INTERFACE): cbool; cdecl; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$endif NDS_INTERFACE}

