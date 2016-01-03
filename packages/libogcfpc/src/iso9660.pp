unit iso9660;
{$mode objfpc} 
{$J+}
{$INLINE ON}
{$MACRO ON}
{$ASSERTIONS ON}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ctypes, gctypes, gccore;
=======
  ctypes, gctypes;
>>>>>>> graemeg/cpstrnew
=======
  ctypes, gctypes;
>>>>>>> graemeg/cpstrnew
=======
  ctypes, gctypes;
>>>>>>> graemeg/cpstrnew
=======
  ctypes, gctypes;
>>>>>>> origin/cpstrnew

const
  ISO_MAXPATHLEN = 128;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ISO9660_Mount(const name: pcchar; const disc_interface: PDISC_INTERFACE): cbool; cdecl; external;
function ISO9660_Unmount(const name: pcchar): cbool; cdecl; external;
function ISO9660_GetVolumeLabel(const name: pcchar): pcchar; cdecl; external;
=======
function ISO9660_Mount: cbool; cdecl; external;
function ISO9660_Unmount: cbool; cdecl; external;
function ISO9660_LastAccess: cuint64; cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
function ISO9660_Mount: cbool; cdecl; external;
function ISO9660_Unmount: cbool; cdecl; external;
function ISO9660_LastAccess: cuint64; cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
function ISO9660_Mount: cbool; cdecl; external;
function ISO9660_Unmount: cbool; cdecl; external;
function ISO9660_LastAccess: cuint64; cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
function ISO9660_Mount: cbool; cdecl; external;
function ISO9660_Unmount: cbool; cdecl; external;
function ISO9660_LastAccess: cuint64; cdecl; external;
>>>>>>> origin/cpstrnew

implementation

initialization

end.
