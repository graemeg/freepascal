unit fat;
{$mode objfpc} 
{$J+}
{$INLINE ON}
{$MACRO ON}
{$ASSERTIONS ON}

interface

uses
  ctypes, gctypes, gccore;

function fatInit(cacheSize: cuint32; setAsDefaultDevice: cbool): cbool; cdecl; external;
function fatInitDefault: cbool; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
type
  PDISC_INTERFACE = ^DISC_INTERFACE;
>>>>>>> graemeg/cpstrnew
=======
type
  PDISC_INTERFACE = ^DISC_INTERFACE;
>>>>>>> graemeg/cpstrnew
=======
type
  PDISC_INTERFACE = ^DISC_INTERFACE;
>>>>>>> graemeg/cpstrnew
=======
type
  PDISC_INTERFACE = ^DISC_INTERFACE;
>>>>>>> origin/cpstrnew
function fatMountSimple(name_: pcchar; interface_: PDISC_INTERFACE): cbool; cdecl; external;

function fatMount(name_: pcchar; interface_: PDISC_INTERFACE; startSector: sec_t; cacheSize, SectorsPerPage: cuint32): cbool; cdecl; external;
procedure fatUnmount(name_: pcchar); cdecl; external;
procedure fatGetVolumeLabel(name_, label_: pcchar); cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// File attributes
const
  ATTR_ARCHIVE = $20;         // Archive
  ATTR_DIRECTORY = $10;       // Directory
  ATTR_VOLUME = $08;          // Volume
  ATTR_SYSTEM = $04;          // System
  ATTR_HIDDEN = $02;          // Hidden
  ATTR_READONLY = $01;        // Read only

(*
Methods to modify DOS File Attributes
*)
function FAT_getAttr(const _file: pcchar): cint; cdecl; external;
function FAT_setAttr(const _file: pcchar; attr: cint): cint; cdecl; external;



=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
implementation

initialization
{$linklib fat}
end.
