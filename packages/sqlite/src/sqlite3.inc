<<<<<<< HEAD
<<<<<<< HEAD
{$mode objfpc}{$h+}
=======
{$mode objfpc}
>>>>>>> graemeg/fixes_2_2
=======
{$mode objfpc}
>>>>>>> origin/fixes_2_2

{$ifdef BSD}
  {$linklib c}
  {$linklib pthread}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
{$packrecords C}

{.$DEFINE SQLITE_OBSOLETE}

interface

uses
  ctypes,
=======
interface

uses
>>>>>>> graemeg/fixes_2_2
=======
interface

uses
>>>>>>> origin/fixes_2_2
{$ifdef LOAD_DYNAMICALLY}
  SysUtils, DynLibs;
{$else}
  DynLibs;
<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef darwin}
{$linklib sqlite3}
{$endif}
{$endif}

const
{$IFDEF WINDOWS}
  Sqlite3Lib = 'sqlite3.dll';
{$else}
  Sqlite3Lib = 'libsqlite3.'+sharedsuffix;
{$endif}

{$IFDEF LOAD_DYNAMICALLY}
  {$DEFINE D}
{$ELSE}
  {$DEFINE S}
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
  Header converted from Sqlite version 3.7.9
  SOURCE_ID = '2011-11-01 00:52:41 c7c6050ef060877ebe77b41d959e9df13f8c9b5e'
}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(*
** 2001 September 15
**
** The author disclaims copyright to this source code.  In place of
** a legal notice, here is a blessing:
**
**    May you do good and not evil.
**    May you find forgiveness for yourself and forgive others.
**    May you share freely, never taking more than you give.
**
*************************************************************************
** This header file defines the interface that the SQLite library
** presents to client programs.  If a C-function, structure, datatype,
** or constant definition does not appear in this file, then it is
** not a published API of SQLite, is subject to change without
** notice, and should not be referenced by programs that use SQLite.
**
** Some of the definitions that are in this file are marked as
** "experimental".  Experimental interfaces are normally new
** features recently added to SQLite.  We do not anticipate changes 
** to experimental interfaces but reserve to make minor changes if
** experience from use "in the wild" suggest such changes are prudent.
**
** The official C-language API documentation for SQLite is derived
** from comments in this file.  This file is the authoritative source
** on how SQLite interfaces are suppose to operate.
**
** The name of this file under configuration management is "sqlite.h.in".
** The makefile makes some minor changes to this file (such as inserting
** the version number) and changes its name to "sqlite3.h" as
** part of the build process.
**
** @(#) $Id: sqlite.h.in,v 1.312 2008/05/12 12:39:56 drh Exp $
*)

(*
** CAPI3REF: Compile-Time Library Version Numbers {F10010}
**
** The SQLITE_VERSION and SQLITE_VERSION_NUMBER #defines in
** the sqlite3.h file specify the version of SQLite with which
** that header file is associated.
**
** The "version" of SQLite is a string of the form "X.Y.Z".
** The phrase "alpha" or "beta" might be appended after the Z.
** The X value is major version number always 3 in SQLite3.
** The X value only changes when  backwards compatibility is
** broken and we intend to never break
** backwards compatibility.  The Y value is the minor version
** number and only changes when
** there are major feature enhancements that are forwards compatible
** but not backwards compatible.  The Z value is release number
** and is incremented with
** each release but resets back to 0 when Y is incremented.
**
** See also: [sqlite3_libversion()] and [sqlite3_libversion_number()].
**
** INVARIANTS:
**
** {F10011} The SQLITE_VERSION #define in the sqlite3.h header file
**          evaluates to a string literal that is the SQLite version
**          with which the header file is associated.
**
** {F10014} The SQLITE_VERSION_NUMBER #define resolves to an integer
**          with the value  (X*1000000 + Y*1000 + Z) where X, Y, and
**          Z are the major version, minor version, and release number.
*)
const
  SQLITE_VERSION        = '3.5.9';
  SQLITE_VERSION_NUMBER = 3005009;

(*
** CAPI3REF: Run-Time Library Version Numbers {F10020}
** KEYWORDS: sqlite3_version
**
** These features provide the same information as the [SQLITE_VERSION]
** and [SQLITE_VERSION_NUMBER] #defines in the header, but are associated
** with the library instead of the header file.  Cautious programmers might
** include a check in their application to verify that 
** sqlite3_libversion_number() always returns the value 
** [SQLITE_VERSION_NUMBER].
**
** The sqlite3_libversion() function returns the same information as is
** in the sqlite3_version[] string constant.  The function is provided
** for use in DLLs since DLL users usually do not have direct access to string
** constants within the DLL.
**
** INVARIANTS:
**
** {F10021} The [sqlite3_libversion_number()] interface returns an integer
**          equal to [SQLITE_VERSION_NUMBER]. 
**
** {F10022} The [sqlite3_version] string constant contains the text of the
**          [SQLITE_VERSION] string. 
**
** {F10023} The [sqlite3_libversion()] function returns
**          a pointer to the [sqlite3_version] string constant.
*)
>>>>>>> graemeg/cpstrnew
//SQLITE_EXTERN const char sqlite3_version[];
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_libversion{$IFDEF D}: function{$ENDIF}(): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_libversion_number{$IFDEF D}: function{$ENDIF}(): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
function sqlite3_version(): pansichar;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_threadsafe{$IFDEF D}: function{$ENDIF}(): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  ppsqlite3 = ^psqlite3;
  psqlite3 = ^_sqlite3;
  _sqlite3 = record end;

  
type
  sqlite3_destructor_type = procedure(user: pointer); cdecl;

const
  SQLITE_STATIC      = sqlite3_destructor_type(nil);
  SQLITE_TRANSIENT   = pointer(-1);//sqlite3_destructor_type(-1);
  

type
  psqlite_int64 = ^sqlite_int64;
  sqlite_int64 = Int64;

  psqlite_uint64 = ^sqlite_uint64;
  sqlite_uint64 = QWord;

  psqlite3_int64 = ^sqlite3_int64;
  sqlite3_int64 = sqlite_int64;

  psqlite3_uint64 = ^sqlite3_uint64;
  sqlite3_uint64 = sqlite_uint64;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_close{$IFDEF D}: function{$ENDIF}(ref: psqlite3): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  sqlite3_callback = function(user: pointer; cols: cint; values, name: ppansichar): cint; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_exec{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;                              (* An open database *)
  sql: pansichar;                            (* SQL to be evaluted *)
  cb: sqlite3_callback;                      (* Callback function *)
  user: pointer;                             (* 1st argument to callback *)
  errmsg: ppansichar                         (* Error msg written here *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

const
  SQLITE_OK         =  0;   (* Successful result *)
(* beginning-of-error-codes *)
  SQLITE_ERROR      =  1;   (* SQL error or missing database *)
  SQLITE_INTERNAL   =  2;   (* Internal logic error in SQLite *)
  SQLITE_PERM       =  3;   (* Access permission denied *)
  SQLITE_ABORT      =  4;   (* Callback routine requested an abort *)
  SQLITE_BUSY       =  5;   (* The database file is locked *)
  SQLITE_LOCKED     =  6;   (* A table in the database is locked *)
  SQLITE_NOMEM      =  7;   (* A malloc() failed *)
  SQLITE_READONLY   =  8;   (* Attempt to write a readonly database *)
  SQLITE_INTERRUPT  =  9;   (* Operation terminated by sqlite3_interrupt()*)
  SQLITE_IOERR      = 10;   (* Some kind of disk I/O error occurred *)
  SQLITE_CORRUPT    = 11;   (* The database disk image is malformed *)
  SQLITE_NOTFOUND   = 12;   (* NOT USED. Table or record not found *)
  SQLITE_FULL       = 13;   (* Insertion failed because database is full *)
  SQLITE_CANTOPEN   = 14;   (* Unable to open the database file *)
  SQLITE_PROTOCOL   = 15;   (* NOT USED. Database lock protocol error *)
  SQLITE_EMPTY      = 16;   (* Database is empty *)
  SQLITE_SCHEMA     = 17;   (* The database schema changed *)
  SQLITE_TOOBIG     = 18;   (* String or BLOB exceeds size limit *)
  SQLITE_CONSTRAINT = 19;   (* Abort due to constraint violation *)
  SQLITE_MISMATCH   = 20;   (* Data type mismatch *)
  SQLITE_MISUSE     = 21;   (* Library used incorrectly *)
  SQLITE_NOLFS      = 22;   (* Uses OS features not supported on host *)
  SQLITE_AUTH       = 23;   (* Authorization denied *)
  SQLITE_FORMAT     = 24;   (* Auxiliary database format error *)
  SQLITE_RANGE      = 25;   (* 2nd parameter to sqlite3_bind out of range *)
  SQLITE_NOTADB     = 26;   (* File opened that is not a database file *)
  SQLITE_ROW        = 100;  (* sqlite3_step() has another row ready *)
  SQLITE_DONE       = 101;  (* sqlite3_step() has finished executing *)

  SQLITE_IOERR_READ          = (SQLITE_IOERR or (1 shl 8));
  SQLITE_IOERR_SHORT_READ    = (SQLITE_IOERR or (2 shl 8));
  SQLITE_IOERR_WRITE         = (SQLITE_IOERR or (3 shl 8));
  SQLITE_IOERR_FSYNC         = (SQLITE_IOERR or (4 shl 8));
  SQLITE_IOERR_DIR_FSYNC     = (SQLITE_IOERR or (5 shl 8));
  SQLITE_IOERR_TRUNCATE      = (SQLITE_IOERR or (6 shl 8));
  SQLITE_IOERR_FSTAT         = (SQLITE_IOERR or (7 shl 8));
  SQLITE_IOERR_UNLOCK        = (SQLITE_IOERR or (8 shl 8));
  SQLITE_IOERR_RDLOCK        = (SQLITE_IOERR or (9 shl 8));
  SQLITE_IOERR_DELETE        = (SQLITE_IOERR or (10 shl 8));
  SQLITE_IOERR_BLOCKED       = (SQLITE_IOERR or (11 shl 8));
  SQLITE_IOERR_NOMEM         = (SQLITE_IOERR or (12 shl 8));
  SQLITE_IOERR_ACCESS            = (SQLITE_IOERR or (13 shl 8));
  SQLITE_IOERR_CHECKRESERVEDLOCK = (SQLITE_IOERR or (14 shl 8));
  SQLITE_IOERR_LOCK              = (SQLITE_IOERR or (15 shl 8));
  SQLITE_IOERR_CLOSE             = (SQLITE_IOERR or (16 shl 8));
  SQLITE_IOERR_DIR_CLOSE         = (SQLITE_IOERR or (17 shl 8));
  SQLITE_IOERR_SHMOPEN           = (SQLITE_IOERR or (18 shl 8));
  SQLITE_IOERR_SHMSIZE           = (SQLITE_IOERR or (19 shl 8));
  SQLITE_IOERR_SHMLOCK           = (SQLITE_IOERR or (20 shl 8));
  SQLITE_IOERR_SHMMAP            = (SQLITE_IOERR or (21 shl 8));
  SQLITE_IOERR_SEEK              = (SQLITE_IOERR or (22 shl 8));
  SQLITE_LOCKED_SHAREDCACHE      = (SQLITE_LOCKED or  (1 shl 8));
  SQLITE_BUSY_RECOVERY           = (SQLITE_BUSY   or  (1 shl 8));
  SQLITE_CANTOPEN_NOTEMPDIR      = (SQLITE_CANTOPEN or (1 shl 8));
  SQLITE_CORRUPT_VTAB            = (SQLITE_CORRUPT or (1 shl 8));
  SQLITE_READONLY_RECOVERY       = (SQLITE_READONLY or (1 shl 8));
  SQLITE_READONLY_CANTLOCK       = (SQLITE_READONLY or (2 shl 8));

  SQLITE_OPEN_READONLY         = $00000001;
  SQLITE_OPEN_READWRITE        = $00000002;
  SQLITE_OPEN_CREATE           = $00000004;
  SQLITE_OPEN_DELETEONCLOSE    = $00000008;
  SQLITE_OPEN_EXCLUSIVE        = $00000010;
  SQLITE_OPEN_MAIN_DB          = $00000100;
  SQLITE_OPEN_TEMP_DB          = $00000200;
  SQLITE_OPEN_TRANSIENT_DB     = $00000400;
  SQLITE_OPEN_MAIN_JOURNAL     = $00000800;
  SQLITE_OPEN_TEMP_JOURNAL     = $00001000;
  SQLITE_OPEN_SUBJOURNAL       = $00002000;
  SQLITE_OPEN_MASTER_JOURNAL   = $00004000;
  SQLITE_OPEN_NOMUTEX          = $00008000;
  SQLITE_OPEN_FULLMUTEX        = $00010000;
  SQLITE_OPEN_SHAREDCACHE      = $00020000;
  SQLITE_OPEN_PRIVATECACHE     = $00040000;
  SQLITE_OPEN_WAL              = $00080000;


  SQLITE_IOCAP_ATOMIC          = $00000001;
  SQLITE_IOCAP_ATOMIC512       = $00000002;
  SQLITE_IOCAP_ATOMIC1K        = $00000004;
  SQLITE_IOCAP_ATOMIC2K        = $00000008;
  SQLITE_IOCAP_ATOMIC4K        = $00000010;
  SQLITE_IOCAP_ATOMIC8K        = $00000020;
  SQLITE_IOCAP_ATOMIC16K       = $00000040;
  SQLITE_IOCAP_ATOMIC32K       = $00000080;
  SQLITE_IOCAP_ATOMIC64K       = $00000100;
  SQLITE_IOCAP_SAFE_APPEND     = $00000200;
  SQLITE_IOCAP_SEQUENTIAL      = $00000400;
  SQLITE_IOCAP_UNDELETABLE_WHEN_OPEN  = $00000800;
  
  SQLITE_LOCK_NONE          = 0;
  SQLITE_LOCK_SHARED        = 1;
  SQLITE_LOCK_RESERVED      = 2;
  SQLITE_LOCK_PENDING       = 3;
  SQLITE_LOCK_EXCLUSIVE     = 4;

  SQLITE_SYNC_NORMAL        = $00002;
  SQLITE_SYNC_FULL          = $00003;
  SQLITE_SYNC_DATAONLY      = $00010;


type
  psqlite3_io_methods = ^sqlite3_io_methods;

  psqlite3_file = ^sqlite3_file;
  sqlite3_file = record
    pMethods: psqlite3_io_methods;  (* Methods for an open file *)
  end;

  sqlite3_io_methods = record
     iVersion          : cint;
     Close             : function(f: psqlite3_file): cint; stdcall;
     Read              : function(f: psqlite3_file; addr: pointer; iAmt: cint; iOfst: sqlite3_int64): cint; stdcall;
     Write             : function(f: psqlite3_file; size: sqlite3_int64): cint; stdcall;
     Truncate          : function(f: psqlite3_file; size: sqlite3_int64): cint; stdcall;
     Sync              : function(f: psqlite3_file; flags: cint): cint; stdcall;
     FileSize          : function(f: psqlite3_file; pSize: psqlite3_int64): cint; stdcall;
     Lock              : function(f: psqlite3_file; flags: cint): cint; stdcall;
     Unlock            : function(f: psqlite3_file; flags: cint): cint; stdcall;
     CheckReservedLock : function(f: psqlite3_file): cint; stdcall;
     FileControl       : function(f: psqlite3_file; op: cint; pArg: pointer): cint; stdcall;
     SectorSize        : function(f: psqlite3_file): cint; stdcall;
     DeviceCharacteristics: function(f: psqlite3_file): cint; stdcall;
     xShmMap : function(f : psqlite3_file; iPg: cint; pgsz: cint; volatile : pointer) : cint;stdcall;
     xShmLock : function(f : psqlite3_file; offset: cint; n : cint; flags : cint) : cint; stdcall;
     xShmBarrier  : procedure (f : psqlite3_file); stdcall;
     xShmUnmap : function(f : psqlite3_file; deleteFlag : cint) : cint; stdcall;
          (* Additional methods may be added in future releases *)
  end;

const
  SQLITE_FCNTL_LOCKSTATE        = 1;
  SQLITE_GET_LOCKPROXYFILE      = 2;
  SQLITE_SET_LOCKPROXYFILE      = 3;
  SQLITE_LAST_ERRNO             = 4;
  SQLITE_FCNTL_SIZE_HINT        = 5;
  SQLITE_FCNTL_CHUNK_SIZE       = 6;
  SQLITE_FCNTL_FILE_POINTER     = 7;
  SQLITE_FCNTL_SYNC_OMITTED     = 8;
  SQLITE_FCNTL_WIN32_AV_RETRY   = 9;
  SQLITE_FCNTL_PERSIST_WAL      = 10;
  SQLITE_FCNTL_OVERWRITE        = 11;

type
  psqlite3_mutex = ^sqlite3_mutex;
  sqlite3_mutex = record end;

type
  psqlite3_vfs = ^sqlite3_vfs;
  sqlite3_vfs = record
    iVersion     : cint;          (* Structure version number *)
    szOsFile     : cint;          (* Size of subclassed sqlite3_file *)
    mxPathname   : cint;          (* Maximum file pathname length *)
    pNext        : psqlite3_vfs;  (* Next registered VFS *)
    zName        : pansichar;     (* Name of this virtual file system *)
    pAppData     : ppointer;      (* Pointer to application-specific *)
    Open         : function(vfs: psqlite3_vfs; zName: pansichar; f: psqlite3_file; flags: cint; pOutFlags: pcint): cint; cdecl;
    Delete       : function(vfs: psqlite3_vfs; zName: pansichar; syncDir: cint): cint; cdecl;
    Access       : function(vfs: psqlite3_vfs; zName: pansichar; flags: cint): cint; cdecl;
    FullPathname : function(vfs: psqlite3_vfs; zName: pansichar; nOut: cint; zOut: pansichar): cint; cdecl;
    DlOpen       : function(vfs: psqlite3_vfs; zFilename: pansichar): pointer; cdecl;
    DlError      : procedure(vfs: psqlite3_vfs; nByte: cint; zErrMsg: pansichar); cdecl;
    DlSym        : function(vfs: psqlite3_vfs; addr: pointer; zSymbol: pansichar): pointer; cdecl;
    DlClose      : procedure(vfs: psqlite3_vfs; addr: pointer); cdecl;
    Randomness   : function(vfs: psqlite3_vfs; nByte: cint; zOut: pansichar): cint; cdecl;
    Sleep        : function(vfs: psqlite3_vfs; microseconds: cint): cint; cdecl;    
    CurrentTime  : function(vfs: psqlite3_vfs; time: pcdouble): cint; cdecl;
    GetLastError : function(vfs: psqlite3_vfs; code: cint; msg: pansichar): cint; cdecl;
    CurrentTimeInt64 : function(vfs: psqlite3_vfs; time: psqlite3_int64): cint; cdecl; 	
    xSetSystemCall : function(vfs: psqlite3_vfs; zName: pansichar; sqlite3_syscall_ptr : pointer) : cint; cdecl;
    xGetSystemCall : function(vfs: psqlite3_vfs; zName: pansichar) : pointer; cdecl;
    xNextSystemCall : function(vfs: psqlite3_vfs; zName: pansichar) : pansichar; cdecl;
  end;

const
  SQLITE_ACCESS_EXISTS    = 0;
  SQLITE_ACCESS_READWRITE = 1;
  SQLITE_ACCESS_READ      = 2;
  SQLITE_SHM_UNLOCK       = 1;
  SQLITE_SHM_LOCK         = 2;
  SQLITE_SHM_SHARED       = 4;
  SQLITE_SHM_EXCLUSIVE    = 8;
  SQLITE_SHM_NLOCK        = 8;

{$IFDEF S}
  function sqlite3_initialize : cint;cdecl; external Sqlite3Lib;
  function sqlite3_shutdown : cint;cdecl; external Sqlite3Lib;
  function sqlite3_os_init : cint;cdecl; external Sqlite3Lib;
  function sqlite3_os_end : cint;cdecl; external Sqlite3Lib;
{$endif}  
{$ifdef D}
Var
  sqlite3_initialize,
  sqlite3_shutdown,
  sqlite3_os_init,
  sqlite3_os_end : Function : cint; cdecl;
{$endif}

{$IFDEF S}
 function sqlite3_config (a : cint): cint;cdecl;varargs; external Sqlite3Lib;
{$ENDIF}
{$IFDEF D}
var 
  sqlite3_config : function (a : cint): cint;cdecl;varargs;
{$ENDIF}

{$IFDEF S}
 function sqlite3_db_config (f : psqlite3; op : cint):cint;cdecl;varargs;external Sqlite3Lib;
{$ENDIF}
{$IFDEF D}
var 
 sqlite3_db_config : function (f : psqlite3; op : cint): cint;cdecl;varargs;
{$ENDIF}

Type
sqlite3_mem_methods = record
  xMalloc : function(size : cint) : pointer;cdecl;
  xFree : procedure(p : pointer); cdecl;
  xRealloc : function(p : pointer;size : cint) : pointer;cdecl;
  xSize : function(p : pointer) : cint; cdecl;
  xRoundup : function(size : cint) : cint; cdecl;
  xInit : function(): cint; cdecl;
  xShutdown : procedure(p : pointer); cdecl;
  pAppData: pointer;
end;
Tsqlite3_mem_methods = sqlite3_mem_methods;

Const
  SQLITE_CONFIG_SINGLETHREAD = 1;
  SQLITE_CONFIG_MULTITHREAD  = 2;
  SQLITE_CONFIG_SERIALIZED   = 3;
  SQLITE_CONFIG_MALLOC       = 4;
  SQLITE_CONFIG_GETMALLOC    = 5;
  SQLITE_CONFIG_SCRATCH      = 6;
  SQLITE_CONFIG_PAGECACHE    = 7;
  SQLITE_CONFIG_HEAP         = 8;
  SQLITE_CONFIG_MEMSTATUS    = 9;
  SQLITE_CONFIG_MUTEX        = 10;
  SQLITE_CONFIG_GETMUTEX     = 11;
  SQLITE_CONFIG_LOOKASIDE    = 13;
  SQLITE_CONFIG_PCACHE       = 14;
  SQLITE_CONFIG_GETPCACHE    = 15;
  SQLITE_CONFIG_LOG          = 16;
  SQLITE_CONFIG_URI          = 17;

  SQLITE_DBCONFIG_LOOKASIDE      = 1001;
  SQLITE_DBCONFIG_ENABLE_FKEY    = 1002;
  SQLITE_DBCONFIG_ENABLE_TRIGGER = 1003;

  
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_extended_result_codes{$IFDEF D}: function{$ENDIF}(db: psqlite3; onoff: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_last_insert_rowid{$IFDEF D}: function{$ENDIF}(db: psqlite3): sqlite3_int64; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_changes{$IFDEF D}: function{$ENDIF}(db: psqlite3): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_total_changes{$IFDEF D}: function{$ENDIF}(db: psqlite3): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_interrupt{$IFDEF D}: procedure{$ENDIF}(db: psqlite3); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_complete{$IFDEF D}: function{$ENDIF}(sql: pansichar): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_complete16{$IFDEF D}: function{$ENDIF}(sql: pansichar): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  busyhandler_callback = function(user: pointer; cnt: cint): cint; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_busy_handler{$IFDEF D}: function{$ENDIF}(db: psqlite3; cb: busyhandler_callback; user: pointer): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_busy_timeout{$IFDEF D}: function{$ENDIF}(db: psqlite3; ms: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_get_table{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;          (* An open database *)
  sql: pansichar;        (* SQL to be evaluated *)
  pResult: pppansichar;  (* Results of the query *)
  nrow: pcint;           (* Number of result rows written here *)
  ncolumn: pcint;        (* Number of result columns written here *)
  errmsg: ppansichar     (* Error msg written here *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_free_table{$IFDEF D}: procedure{$ENDIF}(result: ppansichar); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
//char *sqlite3_mprintf(const char*,...);
//char *sqlite3_vmprintf(const char*, va_list);
//char *sqlite3_snprintf(int,char*,const char*, ...);

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_malloc{$IFDEF D}: function{$ENDIF}(size: cint): pointer;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_realloc{$IFDEF D}: function{$ENDIF}(ptr: pointer; size: cint): pointer;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_free{$IFDEF D}: procedure{$ENDIF}(ptr: pointer);cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_memory_used{$IFDEF D}: function{$ENDIF}(): sqlite3_int64; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_memory_highwater{$IFDEF D}: function{$ENDIF}(resetFlag: cint): sqlite3_int64; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_randomness{$IFDEF D}: procedure{$ENDIF}(N: cint; P: pointer); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  xAuth = function(pUserData: pointer; code: cint; s1, s2, s3, s4: pansichar): cint; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_set_authorizer{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  cb: xAuth;
  pUserData: pointer
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

const
  SQLITE_DENY   = 1;   (* Abort the SQL statement with an error *)
  SQLITE_IGNORE = 2;   (* Don't allow access, but don't generate an error *)

const
  SQLITE_CREATE_INDEX         =  1;   (* Index Name      Table Name      *)
  SQLITE_CREATE_TABLE         =  2;   (* Table Name      NULL            *)
  SQLITE_CREATE_TEMP_INDEX    =  3;   (* Index Name      Table Name      *)
  SQLITE_CREATE_TEMP_TABLE    =  4;   (* Table Name      NULL            *)
  SQLITE_CREATE_TEMP_TRIGGER  =  5;   (* Trigger Name    Table Name      *)
  SQLITE_CREATE_TEMP_VIEW     =  6;   (* View Name       NULL            *)
  SQLITE_CREATE_TRIGGER       =  7;   (* Trigger Name    Table Name      *)
  SQLITE_CREATE_VIEW          =  8;   (* View Name       NULL            *)
  SQLITE_DELETE               =  9;   (* Table Name      NULL            *)
  SQLITE_DROP_INDEX           = 10;   (* Index Name      Table Name      *)
  SQLITE_DROP_TABLE           = 11;   (* Table Name      NULL            *)
  SQLITE_DROP_TEMP_INDEX      = 12;   (* Index Name      Table Name      *)
  SQLITE_DROP_TEMP_TABLE      = 13;   (* Table Name      NULL            *)
  SQLITE_DROP_TEMP_TRIGGER    = 14;   (* Trigger Name    Table Name      *)
  SQLITE_DROP_TEMP_VIEW       = 15;   (* View Name       NULL            *)
  SQLITE_DROP_TRIGGER         = 16;   (* Trigger Name    Table Name      *)
  SQLITE_DROP_VIEW            = 17;   (* View Name       NULL            *)
  SQLITE_INSERT               = 18;   (* Table Name      NULL            *)
  SQLITE_PRAGMA               = 19;   (* Pragma Name     1st arg or NULL *)
  SQLITE_READ                 = 20;   (* Table Name      Column Name     *)
  SQLITE_SELECT               = 21;   (* NULL            NULL            *)
  SQLITE_TRANSACTION          = 22;   (* NULL            NULL            *)
  SQLITE_UPDATE               = 23;   (* Table Name      Column Name     *)
  SQLITE_ATTACH               = 24;   (* Filename        NULL            *)
  SQLITE_DETACH               = 25;   (* Database Name   NULL            *)
  SQLITE_ALTER_TABLE          = 26;   (* Database Name   Table Name      *)
  SQLITE_REINDEX              = 27;   (* Index Name      NULL            *)
  SQLITE_ANALYZE              = 28;   (* Table Name      NULL            *)
  SQLITE_CREATE_VTABLE        = 29;   (* Table Name      Module Name     *)
  SQLITE_DROP_VTABLE          = 30;   (* Table Name      Module Name     *)
  SQLITE_FUNCTION             = 31;   (* Function Name   NULL            *)
  SQLITE_SAVEPOINT            = 32;   (* Operation       Savepoint Name  *)
  SQLITE_COPY                 =  0;   (* No longer used *)


type
  xTrace = procedure(user: pointer; s: pansichar); cdecl;
  xProfile = procedure(user: pointer; s: pansichar; i: sqlite3_uint64); cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_trace{$IFDEF D}: function{$ENDIF}(db: psqlite3; cb: xTrace; user: pointer): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_profile{$IFDEF D}: function{$ENDIF}(db: psqlite3; cb: xProfile; user: pointer): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  progress_callback = function(user: pointer): cint; cdecl;

{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_progress_handler{$IFDEF D}: procedure{$ENDIF}(db: psqlite3; i: cint; cb: progress_callback; user: pointer); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_open{$IFDEF D}: function{$ENDIF}(
  filename: pansichar;    (* Database filename (UTF-8) *)
  ppDb: ppsqlite3         (* OUT: SQLite db handle *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_open16{$IFDEF D}: function{$ENDIF}(
  filename: pwidechar;    (* Database filename (UTF-16) *)
  ppDb: ppsqlite3         (* OUT: SQLite db handle *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_open_v2{$IFDEF D}: function{$ENDIF}(
  filename: pansichar;    (* Database filename (UTF-8) *)
  ppDb: ppsqlite3;        (* OUT: SQLite db handle *)
  flags: cint;            (* Flags *)
  zVfs: pansichar         (* Name of VFS module to use *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}
function sqlite3_uri_parameter(zFilename : pansichar; zParam :pansichar) : pansichar;cdecl;external Sqlite3Lib;
{$ENDIF}
{$IFDEF D}
var
 sqlite3_uri_parameter : function(zFilename : pansichar; zParam :pansichar) : pansichar;cdecl;
{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_errcode{$IFDEF D}: function{$ENDIF}(db: psqlite3): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_extended_errcode{$IFDEF D}: function{$ENDIF}(db: psqlite3): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_errmsg{$IFDEF D}: function{$ENDIF}(db: psqlite3): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_errmsg16{$IFDEF D}: function{$ENDIF}(db: psqlite3): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  ppsqlite3_stmt = ^psqlite3_stmt;
  psqlite3_stmt = ^sqlite3_stmt;
  sqlite3_stmt = record end;


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_limit{$IFDEF D}: function{$ENDIF}(db: psqlite3; id: cint; newVal: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


const
  SQLITE_LIMIT_LENGTH                    = 0;
  SQLITE_LIMIT_SQL_LENGTH                = 1;
  SQLITE_LIMIT_COLUMN                    = 2;
  SQLITE_LIMIT_EXPR_DEPTH                = 3;
  SQLITE_LIMIT_COMPOUND_SELECT           = 4;
  SQLITE_LIMIT_VDBE_OP                   = 5;
  SQLITE_LIMIT_FUNCTION_ARG              = 6;
  SQLITE_LIMIT_ATTACHED                  = 7;
  SQLITE_LIMIT_LIKE_PATTERN_LENGTH       = 8;
  SQLITE_LIMIT_VARIABLE_NUMBER           = 9;
  SQLITE_LIMIT_TRIGGER_DEPTH             = 10;
   

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_prepare{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;             (* Database handle *)
  zSql: pansichar;          (* SQL statement, UTF-8 encoded *)
  nByte: cint;              (* Maximum length of zSql in bytes. *)
  ppStmt: ppsqlite3_stmt;   (* OUT: Statement handle *)
  pzTail: ppansichar        (* OUT: Pointer to unused portion of zSql *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_prepare_v2{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;             (* Database handle *)
  zSql: pansichar;          (* SQL statement, UTF-8 encoded *)
  nByte: cint;              (* Maximum length of zSql in bytes. *)
  ppStmt: ppsqlite3_stmt;   (* OUT: Statement handle *)
  pzTail: ppansichar        (* OUT: Pointer to unused portion of zSql *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_prepare16{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;             (* Database handle *)
  zSql: pwidechar;          (* SQL statement, UTF-16 encoded *)
  nByte: cint;              (* Maximum length of zSql in bytes. *)
  ppStmt: ppsqlite3_stmt;   (* OUT: Statement handle *)
  pzTail: ppwidechar        (* OUT: Pointer to unused portion of zSql *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_prepare16_v2{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;             (* Database handle *)
  zSql: pwidechar;          (* SQL statement, UTF-16 encoded *)
  nByte: cint;              (* Maximum length of zSql in bytes. *)
  ppStmt: ppsqlite3_stmt;   (* OUT: Statement handle *)
  pzTail: ppwidechar        (* OUT: Pointer to unused portion of zSql *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_sql{$IFDEF D}: function{$ENDIF}(pStmt: psqlite3_stmt): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


type
  ppsqlite3_value = ^psqlite3_value;
  psqlite3_value = ^sqlite3_value;
  sqlite3_value = record end;

  psqlite3_context = ^sqlite3_context;
  sqlite3_context = record end;


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_blob{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: pointer; L: cint; D: sqlite3_destructor_type): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_double{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: cdouble): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_int{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_int64{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: sqlite3_int64): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_null{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_text{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: pansichar; L: cint; D: sqlite3_destructor_type): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_text16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: pwidechar; L: cint; D: sqlite3_destructor_type): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_value{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: psqlite3_value): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_zeroblob{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint; V: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_parameter_count{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_parameter_name{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_bind_parameter_index{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; zName: pansichar): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_clear_bindings{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_count{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_name{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_name16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_database_name{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_database_name16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_table_name{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_table_name16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_origin_name{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_origin_name16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_decltype{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_decltype16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; N: cint): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_step{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_data_count{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

const
  SQLITE_INTEGER  = 1;
  SQLITE_FLOAT    = 2;
  SQLITE_BLOB     = 4;
  SQLITE_NULL     = 5;
  SQLITE_TEXT     = 3;
  SQLITE3_TEXT    = 3;


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_blob{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_bytes{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_bytes16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_double{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): cdouble; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_int{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_int64{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): sqlite3_int64; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_text{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_text16{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_type{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_column_value{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; iCol: cint): psqlite3_value; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_finalize{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_reset{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  xFunc = procedure(ctx: psqlite3_context; N: cint; V: ppsqlite3_value); cdecl;
  xStep = procedure(ctx: psqlite3_context; N: cint; V: ppsqlite3_value); cdecl;
  xFinal = procedure(ctx: psqlite3_context); cdecl;
  xDestroy = sqlite3_destructor_type;
  
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_function{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zFunctionName: pansichar;
  nArg: cint;
  eTextRep: cint;
  pApp: pointer;
  funccb: xFunc;
  stepcb: xStep;
  finalcb: xFinal
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_function16{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zFunctionName: pwidechar;
  nArg: cint;
  eTextRep: cint;
  pApp: pointer;
  funccb: xFunc;
  stepcb: xStep;
  finalcb: xFinal
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_function_v2{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zFunctionName: pansichar;
  nArg: cint;
  eTextRep: cint;
  pApp: pointer;
  funccb: xFunc;
  stepcb: xStep;
  finalcb: xFinal;
  destroycb : xDestroy
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

const
  SQLITE_UTF8           = 1;
  SQLITE_UTF16LE        = 2;
  SQLITE_UTF16BE        = 3;
  SQLITE_UTF16          = 4;    (* Use native byte order *)
  SQLITE_ANY            = 5;    (* sqlite3_create_function only *)
  SQLITE_UTF16_ALIGNED  = 8;    (* sqlite3_create_collation only *)

{$IFDEF SQLITE_OBSOLETE}
type
  memory_alarm_cb = function(user: pointer; i64: sqlite3_int64; i: cint): pointer; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_aggregate_count{$IFDEF D}: function{$ENDIF}(ctx: psqlite3_context): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_expired{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_transfer_bindings{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt; stmt2: psqlite3_stmt): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_global_recover{$IFDEF D}: function{$ENDIF}(): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_thread_cleanup{$IFDEF D}: procedure{$ENDIF}(); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_memory_alarm{$IFDEF D}: function{$ENDIF}(cb: memory_alarm_cb; user: pointer; i64: sqlite3_int64): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_blob{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_bytes{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_bytes16{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_double{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): cdouble; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_int{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_int64{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): sqlite3_int64; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_text{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): pansichar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_text16{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_text16le{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_text16be{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): pwidechar; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_type{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_value_numeric_type{$IFDEF D}: function{$ENDIF}(val: psqlite3_value): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_aggregate_context{$IFDEF D}: function{$ENDIF}(ctx: psqlite3_context; nBytes: cint): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_user_data{$IFDEF D}: function{$ENDIF}(ctx: psqlite3_context): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_context_db_handle{$IFDEF D}: function{$ENDIF}(ctx: psqlite3_context): psqlite3; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


type
  set_auxdata_cb = function(p: pointer): pointer; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_get_auxdata{$IFDEF D}: function{$ENDIF}(ctx: psqlite3_context; N: cint): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_set_auxdata{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; N: cint; P: pointer; cb: set_auxdata_cb); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_blob{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pointer; N: cint; D: sqlite3_destructor_type); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_double{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: cdouble); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_error{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pansichar; N: cint); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_error16{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pwidechar; N: cint); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_error_toobig{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_error_nomem{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_error_code{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: cint); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_int{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: cint); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_int64{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: sqlite3_int64); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_null{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_text{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pansichar; N: cint; D: sqlite3_destructor_type); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_text16{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pwidechar; N: cint; D: sqlite3_destructor_type); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_text16le{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pwidechar; N: cint; D: sqlite3_destructor_type); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_text16be{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: pwidechar; N: cint; D: sqlite3_destructor_type); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_value{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: psqlite3_value); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_result_zeroblob{$IFDEF D}: procedure{$ENDIF}(ctx: psqlite3_context; V: cint); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  xCompare = function(user: pointer; A: cint; B: pointer; C: cint; D: pointer): cint; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_collation{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zName: pansichar;
  eTextRep: cint;
  user: pointer;
  xcomparecb: xCompare
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_collation_v2{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zName: pansichar;
  eTextRep: cint;
  user: pointer;
  xcomparecb: xCompare;
  xdestroycb: xDestroy
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_collation16{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zName: pwidechar;
  eTextRep: cint;
  user: pointer;
  xcomparecb: xCompare
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


type
  collation_needed_cb = function(user: pointer; db: psqlite3; eTextRep: cint; s: pansichar): pointer; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_collation_needed{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  user: pointer;
  cb: collation_needed_cb
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_collation_needed16{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  user: pointer;
  cb: collation_needed_cb
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_key{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;                   (* Database to be rekeyed *)
  pKey: pointer; nKey: cint       (* The key *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_rekey{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;                   (* Database to be rekeyed *)
  pKey: pointer; nKey: cint       (* The new key *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_sleep{$IFDEF D}: function{$ENDIF}(M: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


{$ifndef win32}
var
  sqlite3_temp_directory: pansichar; cvar; external {Sqlite3Lib};
{$endif}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_get_autocommit{$IFDEF D}: function{$ENDIF}(db: psqlite3): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_db_handle{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt): psqlite3; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_next_stmt{$IFDEF D}: function{$ENDIF}(db: psqlite3;stmt: psqlite3_stmt):psqlite3_stmt;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF} 
 
type
  commit_callback = function(user: pointer): cint; cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_commit_hook{$IFDEF D}: function{$ENDIF}(db: psqlite3; cb: commit_callback; user: pointer): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_rollback_hook{$IFDEF D}: function{$ENDIF}(db: psqlite3; cb: sqlite3_destructor_type; user: pointer): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
type
  update_callback = procedure(user: pointer; event: cint; database, table: pansichar; rowid: sqlite3_int64); cdecl;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_update_hook{$IFDEF D}: function{$ENDIF}(db: psqlite3; cb: update_callback; user: pointer): pointer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_enable_shared_cache{$IFDEF D}: function{$ENDIF}(B: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_release_memory{$IFDEF D}: function{$ENDIF}(N: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_soft_heap_limit{$IFDEF D}: procedure{$ENDIF}(N: cint); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_soft_heap_limit64{$IFDEF D}: function{$ENDIF}(N: int64):int64;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
 
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_table_column_metadata{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;             (* Connection handle *)
  zDbName: pansichar;           (* Database name or NULL *)
  zTableName: pansichar;        (* Table name *)
  zColumnName: pansichar;       (* Column name *)
  pzDataType: ppansichar;       (* OUTPUT: Declared data type *)
  pzCollSeq: ppansichar;        (* OUTPUT: Collation sequence name *)
  pNotNull: pcint;          (* OUTPUT: True if NOT NULL constracint exists *)
  pPrimaryKey: pcint;       (* OUTPUT: True if column part of PK *)
  pAutoinc: pcint           (* OUTPUT: True if column is auto-increment *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_load_extension{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;          (* Load the extension cinto this database connection *)
  zFile: pansichar;          (* Name of the shared library containing extension *)
  zProc: pansichar;          (* Entry point.  Derived from zFile if 0 *)
  pzErrMsg: ppansichar       (* Put error message here if not 0 *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_enable_load_extension{$IFDEF D}: function{$ENDIF}(db: psqlite3; onoff: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_auto_extension{$IFDEF D}: function{$ENDIF}(xEntrypoint: pointer): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_reset_auto_extension{$IFDEF D}: procedure{$ENDIF}(); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


type
  psqlite3_module = ^sqlite3_module;
  sqlite3_module = record{
    iVersion     : cint;
   // xCreate      : function(db: psqlite3; pAux: pointer; argc: cint; ): cint; cdecl;
  cint (*xCreate)(db: psqlite3; void *pAux;
               cint argc; const char *const*argv;
               sqlite3_vtab **ppVTab; char**);
  cint (*xConnect)(db: psqlite3; void *pAux;
               cint argc; const char *const*argv;
               sqlite3_vtab **ppVTab; char**);
  cint (*xBestIndex)(sqlite3_vtab *pVTab; sqlite3_index_info*);
  cint (*xDisconnect)(sqlite3_vtab *pVTab);
  cint (*xDestroy)(sqlite3_vtab *pVTab);
  cint (*xOpen)(sqlite3_vtab *pVTab; sqlite3_vtab_cursor **ppCursor);
  cint (*xClose)(sqlite3_vtab_cursor*);
  cint (*xFilter)(sqlite3_vtab_cursor*; cint idxNum; const char *idxStr;
                cint argc; sqlite3_value **argv);
    xNext         : function(pVCurs: sqlite3_vtab_cursor): cint; cdecl;
    xEof          : function(pVCurs: sqlite3_vtab_cursor): cint; cdecl;
    xColumn       : function(pVCurs: sqlite3_vtab_cursor; ctx: psqlite3_context; i: cint): cint; cdecl;
    xRowid        : function(pVCurs: sqlite3_vtab_cursor; var pRowid: sqlite3_int64): cint; cdecl;
    xUpdate       : function(pVtab: psqlite3_vtab; var v: psqlite3_value; var p: sqlite3_int64): cint; cdecl;
    xBegin        : function(pVtab: psqlite3_vtab): cint; cdecl;
    xSync         : function(pVtab: psqlite3_vtab): cint; cdecl;
    xCommit       : function(pVtab: psqlite3_vtab): cint; cdecl;
    xRollback     : function(pVtab: psqlite3_vtab): cint; cdecl;
    xFindFunction : function(pVtab: psqlite3_vtab; nArg: cint; zName: pansichar; var pxFunc: xFunc; var ppArg: pointer): cint; cdecl;
    xRename       : function(pVtab: psqlite3_vtab; zNew: pansichar): cint; cdecl;
  }end;
{.$WARNING TODO}


type
  psqlite3_index_constracint = ^sqlite3_index_constracint;
  sqlite3_index_constracint = record
     iColumn: cint;              (* Column on left-hand side of constracint *)
     op: char;                   (* Constracint operator *)
     usable: char;               (* True if this constracint is usable *)
     iTermOffset: cint;          (* Used cinternally - xBestIndex should ignore *)
  end;

  psqlite3_index_orderby = ^sqlite3_index_orderby;
  sqlite3_index_orderby = record
     iColumn: cint;              (* Column number *)
     desc: char;                 (* True for DESC.  False for ASC. *)
  end;

  psqlite3_index_constracint_usage = ^sqlite3_index_constracint_usage;
  sqlite3_index_constracint_usage = record
    argvIndex: cint;             (* if >0; constracint is part of argv to xFilter *)
    omit: char;                  (* Do not code a test for this constracint *)
  end;

  psqlite3_index_info = ^sqlite3_index_info;
  sqlite3_index_info = record
  (* Inputs *)
    nConstracint: cint;         (* Number of entries in aConstracint *)
    aConstracint: psqlite3_index_constracint;
    nOrderBy: cint;             (* Number of terms in the ORDER BY clause *)
    aOrderBy: psqlite3_index_orderby;

  (* Outputs *)
    aConstracintUsage: psqlite3_index_constracint_usage;

    idxNum: cint;               (* Number used to identify the index *)
    idxStr: pansichar;          (* String; possibly obtained from sqlite3_malloc *)
    needToFreeIdxStr: cint;     (* Free idxStr using sqlite3_free() if true *)
    orderByConsumed: cint;      (* True if output is already ordered *)
    estimatedCost: cdouble;     (* Estimated cost of using this index *)
  end;

const
  SQLITE_INDEX_CONSTRAINT_EQ    = 2;
  SQLITE_INDEX_CONSTRAINT_GT    = 4;
  SQLITE_INDEX_CONSTRAINT_LE    = 8;
  SQLITE_INDEX_CONSTRAINT_LT    = 16;
  SQLITE_INDEX_CONSTRAINT_GE    = 32;
  SQLITE_INDEX_CONSTRAINT_MATCH = 64;


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_module{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;               (* SQLite connection to register module with *)
  zName: pansichar;           (* Name of the module *)
  module: psqlite3_module;    (* Methods for the module *)
  user: pointer               (* Client data for xCreate/xConnect *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_create_module_v2{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;               (* SQLite connection to register module with *)
  zName: pansichar;           (* Name of the module *)
  module: psqlite3_module;    (* Methods for the module *)
  user: pointer;              (* Client data for xCreate/xConnect *)
  xdestroycb: xDestroy        (* Module destructor function *)
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


type
  psqlite3_vtab = ^sqlite3_vtab;
  sqlite3_vtab = record
    pModule: psqlite3_module;        (* The module for this virtual table *)
    nRef: cint;                      (* Used cinternally *)
    zErrMsg: pansichar;              (* Error message from sqlite3_mprcintf() *)
  (* Virtual table implementations will typically add additional fields *)
  end;

type
  psqlite3_vtab_cursor = ^sqlite3_vtab_cursor;
  sqlite3_vtab_cursor = record
    pVtab: psqlite3_vtab;      (* Virtual table of this cursor *)
  (* Virtual table implementations will typically add additional fields *)
  end;

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_declare_vtab{$IFDEF D}: function{$ENDIF}(db: psqlite3; zCreateTable: pansichar): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_overload_function{$IFDEF D}: function{$ENDIF}(db: psqlite3; zFuncName: pansichar; nArg: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

type
  ppsqlite3_blob = ^psqlite3_blob;
  psqlite3_blob = ^sqlite3_blob;
  sqlite3_blob = record end;


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_blob_open{$IFDEF D}: function{$ENDIF}(
  db: psqlite3;
  zDb: pansichar;
  zTable: pansichar;
  zColumn: pansichar;
  iRow: sqlite3_int64;
  flags: cint;
  ppBlob: ppsqlite3_blob
): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}


{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_blob_reopen{$IFDEF D}: function{$ENDIF}(blob: psqlite3_blob;p:sqlite3_int64): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_blob_close{$IFDEF D}: function{$ENDIF}(blob: psqlite3_blob): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_blob_bytes{$IFDEF D}: function{$ENDIF}(blob: psqlite3_blob): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_blob_read{$IFDEF D}: function{$ENDIF}(blob: psqlite3_blob; Z: pointer; N: cint; iOffset: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_blob_write{$IFDEF D}: function{$ENDIF}(blob: psqlite3_blob; Z: pointer; N: cint; iOffset: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_vfs_find{$IFDEF D}: function{$ENDIF}(zVfsName: pansichar): psqlite3_vfs; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_vfs_register{$IFDEF D}: function{$ENDIF}(vfs: psqlite3_vfs; makeDflt: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_vfs_unregister{$IFDEF D}: function{$ENDIF}(vfs: psqlite3_vfs): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_mutex_alloc{$IFDEF D}: function{$ENDIF}(n: cint): psqlite3_mutex; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_mutex_free{$IFDEF D}: procedure{$ENDIF}(mtx: psqlite3_mutex); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_mutex_ente{$IFDEF D}: procedure{$ENDIF}(mtx: psqlite3_mutex); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_mutex_try{$IFDEF D}: function{$ENDIF}(mtx: psqlite3_mutex): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF}sqlite3_mutex_leave{$IFDEF D}: procedure{$ENDIF}(mtx: psqlite3_mutex); cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_mutex_held{$IFDEF D}: function{$ENDIF}(mtx: psqlite3_mutex): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_mutex_notheld{$IFDEF D}: function{$ENDIF}(mtx: psqlite3_mutex): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_db_mutex{$IFDEF D}: function{$ENDIF}(db: psqlite3): psqlite3_mutex; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

const
  SQLITE_MUTEX_FAST             = 0;
  SQLITE_MUTEX_RECURSIVE        = 1;
  SQLITE_MUTEX_STATIC_MASTER    = 2;
  SQLITE_MUTEX_STATIC_MEM       = 3;  (* sqlite3_malloc() *)
  SQLITE_MUTEX_STATIC_MEM2      = 4;  (* sqlite3_release_memory() *)
  SQLITE_MUTEX_STATIC_PRNG      = 5;  (* sqlite3_random() *)
  SQLITE_MUTEX_STATIC_LRU       = 6;  (* lru page list *)
  SQLITE_MUTEX_STATIC_LRU2      = 7;  (* lru page list *)

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_file_control{$IFDEF D}: function{$ENDIF}(db: psqlite3; zDbName: pansichar; op: cint; p: pointer): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_test_control{$IFDEF D}: function{$ENDIF}(op: cint; args: array of const): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_status{$IFDEF D}: function{$ENDIF}(op: cint; pcurrent:pcint; pHighwater: pcint; resetFlag: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_db_status{$IFDEF D}: function{$ENDIF}(db : psqlite3;op: cint; pcurrent:pcint; pHighwater: pcint; resetFlag: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF}sqlite3_stmt_status{$IFDEF D}: function{$ENDIF}(stmt: psqlite3_stmt;op: cint; pcurrent:pcint; pHighwater: pcint; resetFlag: cint): cint; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

const
  SQLITE_TESTCTRL_FAULT_CONFIG             = 1;
  SQLITE_TESTCTRL_FAULT_FAILURES           = 2;
  SQLITE_TESTCTRL_FAULT_BENIGN_FAILURES    = 3;
  SQLITE_TESTCTRL_FAULT_PENDING            = 4;
  SQLITE_TESTCTRL_PRNG_SAVE                = 5;
  SQLITE_TESTCTRL_PRNG_RESTORE             = 6;
  SQLITE_TESTCTRL_PRNG_RESET               = 7;
  SQLITE_TESTCTRL_BITVEC_TEST              = 8;


{Backup api}
type
  psqlite3backup = Pointer;

{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_backup_init{$IFDEF D}: function{$ENDIF}(pDest: psqlite3; const zDestName: pansichar; pSource: psqlite3; const zSourceName: pansichar): psqlite3backup; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_backup_step{$IFDEF D}: function{$ENDIF}(p: psqlite3backup; nPage: Integer): Integer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_backup_finish{$IFDEF D}: function{$ENDIF}(p: psqlite3backup): Integer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_backup_remaining{$IFDEF D}: function{$ENDIF}(p: psqlite3backup): Integer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_backup_pagecount{$IFDEF D}: function{$ENDIF}(p: psqlite3backup): Integer; cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

Type
  xNotifycb = procedure (Argp: pointer; narg : cint);cdecl;
  
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_unlock_notify{$IFDEF D}: function{$ENDIF}(pBlocked:psqlite3;xNotify: xNotifycb;arg:pointer):cint;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}procedure{$ELSE}var{$ENDIF} sqlite3_log{$IFDEF D}: procedure{$ENDIF}(iErrCode:cint;fmt : pansichar); cdecl;varargs;{$IFDEF S}external Sqlite3Lib;{$ENDIF}

Type
  wal_hook_cb = function (p : pointer; db :psqlite3; c : pansichar; d: cint): cint;cdecl;
 
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_wal_hook{$IFDEF D}: function{$ENDIF}(db:psqlite3;cb : wal_hook_cb; p: pointer): pointer;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF}
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_wal_autocheckpoint{$IFDEF D}: function{$ENDIF}(db:psqlite3;n : cint): cint;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF} 
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_wal_checkpoint{$IFDEF D}: function{$ENDIF}(db:psqlite3;zDB: pansichar): cint;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF} 
{$IFDEF S}function{$ELSE}var{$ENDIF} sqlite3_wal_checkpoint_v2{$IFDEF D}: function{$ENDIF}(db:psqlite3;zDB: pansichar;emode:cint;nLog:pcint;nCkpt:pcint): cint;cdecl;{$IFDEF S}external Sqlite3Lib;{$ENDIF} 
                             

{$IFDEF LOAD_DYNAMICALLY}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InitializeSqliteANSI(const LibraryName: AnsiString = ''): Integer; //needed as TLibraryLoadFunction
function InitializeSqlite(const LibraryName: UnicodeString = ''): Integer;
function TryInitializeSqlite(const LibraryName: Unicodestring = ''): Integer;
function ReleaseSqlite: Integer;
procedure ReleaseSqlite; //needed as TLibraryUnLoadFunction

function InitialiseSQLite: Integer; deprecated;
function InitialiseSQLite(const LibraryName: UnicodeString): Integer; deprecated;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function InitializeSqlite(const LibraryName: String = ''): Integer;
function TryInitializeSqlite(const LibraryName: string = ''): Integer;
function ReleaseSqlite: Integer;

function InitialiseSQLite: Integer; deprecated;
function InitialiseSQLite(const LibraryName: String): Integer; deprecated;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  SQLiteLibraryHandle: TLibHandle;
  SQLiteDefaultLibrary: String = Sqlite3Lib;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SQLiteLoadedLibrary: UnicodeString;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{$endif}

{
  Automatically converted by H2Pas 0.99.16 from sqlite3.h
  The following command line parameters were used:
    -D
    -c
    sqlite3.h

  Manual corrections made by Luiz Américo - 2005
}

{$PACKRECORDS C}

const
{$IFDEF WINDOWS}
  Sqlite3Lib = 'sqlite3.dll';
{$else}
  Sqlite3Lib = 'libsqlite3.'+sharedsuffix;
{$endif}

  SQLITE_INTEGER = 1;   
  SQLITE_FLOAT = 2;   
{ #define SQLITE_TEXT  3  // See below  }
  SQLITE_BLOB = 4;   
  SQLITE_NULL = 5;   
  SQLITE_TEXT = 3;   
  SQLITE3_TEXT = 3;   
  SQLITE_UTF8 = 1;       
  SQLITE_UTF16LE = 2;       
  SQLITE_UTF16BE = 3;       
{ Use native byte order  }
  SQLITE_UTF16 = 4;       
{ sqlite3_create_function only  }
  SQLITE_ANY = 5;  
   
   //sqlite_exec return values
  SQLITE_OK = 0;   
  SQLITE_ERROR = 1;{ SQL error or missing database  }
  SQLITE_INTERNAL = 2;{ An internal logic error in SQLite  }
  SQLITE_PERM = 3;   { Access permission denied  }
  SQLITE_ABORT = 4; { Callback routine requested an abort  }
  SQLITE_BUSY = 5;  { The database file is locked  }
  SQLITE_LOCKED = 6;{ A table in the database is locked  }
  SQLITE_NOMEM = 7; { A malloc() failed  }
  SQLITE_READONLY = 8;{ Attempt to write a readonly database  }
  SQLITE_INTERRUPT = 9;{ Operation terminated by sqlite3_interrupt() }
  SQLITE_IOERR = 10;   { Some kind of disk I/O error occurred  }
  SQLITE_CORRUPT = 11;   { The database disk image is malformed  }
  SQLITE_NOTFOUND = 12;   { (Internal Only) Table or record not found  }
  SQLITE_FULL = 13;   { Insertion failed because database is full  }
  SQLITE_CANTOPEN = 14;   { Unable to open the database file  }
  SQLITE_PROTOCOL = 15;   { Database lock protocol error  }
  SQLITE_EMPTY = 16;   { Database is empty  }
  SQLITE_SCHEMA = 17;   { The database schema changed  }
  SQLITE_TOOBIG = 18;   { Too much data for one row of a table  }
  SQLITE_CONSTRAINT = 19;   { Abort due to contraint violation  }
  SQLITE_MISMATCH = 20;   { Data type mismatch  }
  SQLITE_MISUSE = 21;   { Library used incorrectly  }
  SQLITE_NOLFS = 22;   { Uses OS features not supported on host  }
  SQLITE_AUTH = 23;   { Authorization denied  }
  SQLITE_FORMAT = 24;   { Auxiliary database format error  }
  SQLITE_RANGE = 25;   { 2nd parameter to sqlite3_bind out of range  }
  SQLITE_NOTADB = 26;   { File opened that is not a database file  }
  SQLITE_ROW = 100;   { sqlite3_step() has another row ready  }
  SQLITE_DONE = 101;   { sqlite3_step() has finished executing  }

   SQLITE_COPY = 0;   
   SQLITE_CREATE_INDEX = 1;   
   SQLITE_CREATE_TABLE = 2;   
   SQLITE_CREATE_TEMP_INDEX = 3;   
   SQLITE_CREATE_TEMP_TABLE = 4;   
   SQLITE_CREATE_TEMP_TRIGGER = 5;   
   SQLITE_CREATE_TEMP_VIEW = 6;   
   SQLITE_CREATE_TRIGGER = 7;   
   SQLITE_CREATE_VIEW = 8;   
   SQLITE_DELETE = 9;   
   SQLITE_DROP_INDEX = 10;   
   SQLITE_DROP_TABLE = 11;   
   SQLITE_DROP_TEMP_INDEX = 12;   
   SQLITE_DROP_TEMP_TABLE = 13;   
   SQLITE_DROP_TEMP_TRIGGER = 14;   
   SQLITE_DROP_TEMP_VIEW = 15;   
   SQLITE_DROP_TRIGGER = 16;   
   SQLITE_DROP_VIEW = 17;   
   SQLITE_INSERT = 18;   
   SQLITE_PRAGMA = 19;   
   SQLITE_READ = 20;   
   SQLITE_SELECT = 21;   
   SQLITE_TRANSACTION = 22;   
   SQLITE_UPDATE = 23;   
   SQLITE_ATTACH = 24;   
   SQLITE_DETACH = 25;   
   SQLITE_ALTER_TABLE = 26;   
   SQLITE_REINDEX = 27;   

  SQLITE_DENY = 1;   
  SQLITE_IGNORE = 2;   

// Original from sqlite3.h: 
//#define SQLITE_STATIC      ((void(*)(void *))0)
//#define SQLITE_TRANSIENT   ((void(*)(void *))-1)
Const
  SQLITE_STATIC    =  0;
  SQLITE_TRANSIENT =  -1;

type
  sqlite_int64 = int64;
  sqlite_uint64 = qword;
  PPPChar = ^PPChar;
  Psqlite3  = Pointer;
  PPSqlite3 = ^PSqlite3;
  Psqlite3_context  = Pointer;
  Psqlite3_stmt  = Pointer;
  PPsqlite3_stmt = ^Psqlite3_stmt;
  Psqlite3_value  = Pointer;
  PPsqlite3_value  = ^Psqlite3_value;

//Callback function types
//Notice that most functions were named using as prefix the function name that uses them,
//rather than describing their functions  

  sqlite3_callback = function (_para1:pointer; _para2:longint; _para3:PPchar; _para4:PPchar):longint;cdecl;
  busy_handler_func = function (_para1:pointer; _para2:longint):longint;cdecl;
  sqlite3_set_authorizer_func = function (_para1:pointer; _para2:longint; _para3:Pchar; _para4:Pchar; _para5:Pchar; _para6:Pchar):longint;cdecl;
  sqlite3_trace_func = procedure (_para1:pointer; _para2:Pchar);cdecl;
  sqlite3_progress_handler_func = function (_para1:pointer):longint;cdecl;
  sqlite3_commit_hook_func = function (_para1:pointer):longint;cdecl;
  bind_destructor_func = procedure (_para1:pointer);cdecl;
  create_function_step_func = procedure (_para1:Psqlite3_context; _para2:longint; _para3:PPsqlite3_value);cdecl;
  create_function_func_func = procedure (_para1:Psqlite3_context; _para2:longint; _para3:PPsqlite3_value);cdecl;
  create_function_final_func = procedure (_para1:Psqlite3_context);cdecl;
  sqlite3_set_auxdata_func = procedure (_para1:pointer);cdecl;
  sqlite3_result_func = procedure (_para1:pointer);cdecl;
  sqlite3_create_collation_func = function (_para1:pointer; _para2:longint; _para3:pointer; _para4:longint; _para5:pointer):longint;cdecl;
  sqlite3_collation_needed_func = procedure (_para1:pointer; _para2:Psqlite3; eTextRep:longint; _para4:Pchar);cdecl;

{$ifndef win32}
var
  //This is not working under windows. Any clues?
  sqlite3_temp_directory : Pchar;cvar; external;
{$endif}

{$IFNDEF LOAD_DYNAMICALLY}

function sqlite3_close(_para1:Psqlite3):longint;cdecl; external Sqlite3Lib name 'sqlite3_close';
function sqlite3_exec(_para1:Psqlite3; sql:Pchar; _para3:sqlite3_callback; _para4:pointer; errmsg:PPchar):longint;cdecl; external Sqlite3Lib name 'sqlite3_exec';
function sqlite3_last_insert_rowid(_para1:Psqlite3):sqlite_int64;cdecl; external Sqlite3Lib name 'sqlite3_last_insert_rowid';
function sqlite3_changes(_para1:Psqlite3):longint;cdecl; external Sqlite3Lib name 'sqlite3_changes';
function sqlite3_total_changes(_para1:Psqlite3):longint;cdecl; external Sqlite3Lib name 'sqlite3_total_changes';
procedure sqlite3_interrupt(_para1:Psqlite3);cdecl; external Sqlite3Lib name 'sqlite3_interrupt';
function sqlite3_complete(sql:Pchar):longint;cdecl; external Sqlite3Lib name 'sqlite3_complete';
function sqlite3_complete16(sql:pointer):longint;cdecl; external Sqlite3Lib name 'sqlite3_complete16';
function sqlite3_busy_handler(_para1:Psqlite3; _para2:busy_handler_func; _para3:pointer):longint;cdecl; external Sqlite3Lib name 'sqlite3_busy_handler';
function sqlite3_busy_timeout(_para1:Psqlite3; ms:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_busy_timeout';
function sqlite3_get_table(_para1:Psqlite3; sql:Pchar; resultp:PPPchar; nrow:Plongint; ncolumn:Plongint; errmsg:PPchar):longint;cdecl; external Sqlite3Lib name 'sqlite3_get_table';
procedure sqlite3_free_table(result:PPchar);cdecl; external Sqlite3Lib name 'sqlite3_free_table';
// Todo: see how translate sqlite3_mprintf, sqlite3_vmprintf, sqlite3_snprintf
// function sqlite3_mprintf(_para1:Pchar; args:array of const):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_mprintf';
function sqlite3_mprintf(_para1:Pchar):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_mprintf';
//function sqlite3_vmprintf(_para1:Pchar; _para2:va_list):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_vmprintf';
procedure sqlite3_free(z:Pchar);cdecl; external Sqlite3Lib name 'sqlite3_free';
//function sqlite3_snprintf(_para1:longint; _para2:Pchar; _para3:Pchar; args:array of const):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_snprintf';
function sqlite3_snprintf(_para1:longint; _para2:Pchar; _para3:Pchar):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_snprintf';
function sqlite3_set_authorizer(_para1:Psqlite3; xAuth:sqlite3_set_authorizer_func; pUserData:pointer):longint;cdecl; external Sqlite3Lib name 'sqlite3_set_authorizer';
function sqlite3_trace(_para1:Psqlite3; xTrace:sqlite3_trace_func; _para3:pointer):pointer;cdecl; external Sqlite3Lib name 'sqlite3_trace';
procedure sqlite3_progress_handler(_para1:Psqlite3; _para2:longint; _para3:sqlite3_progress_handler_func; _para4:pointer);cdecl; external Sqlite3Lib name 'sqlite3_progress_handler';
function sqlite3_commit_hook(_para1:Psqlite3; _para2:sqlite3_commit_hook_func; _para3:pointer):pointer;cdecl; external Sqlite3Lib name 'sqlite3_commit_hook';
function sqlite3_open(filename:Pchar; ppDb:PPsqlite3):longint;cdecl; external Sqlite3Lib name 'sqlite3_open';
function sqlite3_open16(filename:pointer; ppDb:PPsqlite3):longint;cdecl; external Sqlite3Lib name 'sqlite3_open16';
function sqlite3_errcode(db:Psqlite3):longint;cdecl; external Sqlite3Lib name 'sqlite3_errcode';
function sqlite3_errmsg(_para1:Psqlite3):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_errmsg';
function sqlite3_errmsg16(_para1:Psqlite3):pointer;cdecl; external Sqlite3Lib name 'sqlite3_errmsg16';
function sqlite3_prepare(db:Psqlite3; zSql:Pchar; nBytes:longint; ppStmt:PPsqlite3_stmt; pzTail:PPchar):longint;cdecl; external Sqlite3Lib name 'sqlite3_prepare';
function sqlite3_prepare16(db:Psqlite3; zSql:pointer; nBytes:longint; ppStmt:PPsqlite3_stmt; pzTail:Ppointer):longint;cdecl; external Sqlite3Lib name 'sqlite3_prepare16';
function sqlite3_bind_blob(_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; n:longint; _para5:bind_destructor_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_blob';
function sqlite3_bind_double(_para1:Psqlite3_stmt; _para2:longint; _para3:double):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_double';
function sqlite3_bind_int(_para1:Psqlite3_stmt; _para2:longint; _para3:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_int';
function sqlite3_bind_int64(_para1:Psqlite3_stmt; _para2:longint; _para3:sqlite_int64):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_int64';
function sqlite3_bind_null(_para1:Psqlite3_stmt; _para2:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_null';
function sqlite3_bind_text(_para1:Psqlite3_stmt; _para2:longint; _para3:Pchar; n:longint; _para5:bind_destructor_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_text';
function sqlite3_bind_text16(_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; _para4:longint; _para5:bind_destructor_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_text16';
//function sqlite3_bind_value(_para1:Psqlite3_stmt; _para2:longint; _para3:Psqlite3_value):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_value';
//These overloaded functions were introduced to allow the use of SQLITE_STATIC and SQLITE_TRANSIENT
//It's the c world man ;-)
function sqlite3_bind_blob(_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; n:longint; _para5:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_blob';
function sqlite3_bind_text(_para1:Psqlite3_stmt; _para2:longint; _para3:Pchar; n:longint; _para5:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_text';
function sqlite3_bind_text16(_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; _para4:longint; _para5:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_text16';

function sqlite3_bind_parameter_count(_para1:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_parameter_count';
function sqlite3_bind_parameter_name(_para1:Psqlite3_stmt; _para2:longint):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_bind_parameter_name';
function sqlite3_bind_parameter_index(_para1:Psqlite3_stmt; zName:Pchar):longint;cdecl; external Sqlite3Lib name 'sqlite3_bind_parameter_index';
//function sqlite3_clear_bindings(_para1:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_clear_bindings';
function sqlite3_column_count(pStmt:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_column_count';
function sqlite3_column_name(_para1:Psqlite3_stmt; _para2:longint):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_column_name';
function sqlite3_column_name16(_para1:Psqlite3_stmt; _para2:longint):pointer;cdecl; external Sqlite3Lib name 'sqlite3_column_name16';
function sqlite3_column_decltype(_para1:Psqlite3_stmt; i:longint):Pchar;cdecl; external Sqlite3Lib name 'sqlite3_column_decltype';
function sqlite3_column_decltype16(_para1:Psqlite3_stmt; _para2:longint):pointer;cdecl; external Sqlite3Lib name 'sqlite3_column_decltype16';
function sqlite3_step(_para1:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_step';
function sqlite3_data_count(pStmt:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_data_count';
function sqlite3_column_blob(_para1:Psqlite3_stmt; iCol:longint):pointer;cdecl; external Sqlite3Lib name 'sqlite3_column_blob';
function sqlite3_column_bytes(_para1:Psqlite3_stmt; iCol:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_column_bytes';
function sqlite3_column_bytes16(_para1:Psqlite3_stmt; iCol:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_column_bytes16';
function sqlite3_column_double(_para1:Psqlite3_stmt; iCol:longint):double;cdecl; external Sqlite3Lib name 'sqlite3_column_double';
function sqlite3_column_int(_para1:Psqlite3_stmt; iCol:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_column_int';
function sqlite3_column_int64(_para1:Psqlite3_stmt; iCol:longint):sqlite_int64;cdecl; external Sqlite3Lib name 'sqlite3_column_int64';
function sqlite3_column_text(_para1:Psqlite3_stmt; iCol:longint):PChar;cdecl; external Sqlite3Lib name 'sqlite3_column_text';
function sqlite3_column_text16(_para1:Psqlite3_stmt; iCol:longint):pointer;cdecl; external Sqlite3Lib name 'sqlite3_column_text16';
function sqlite3_column_type(_para1:Psqlite3_stmt; iCol:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_column_type';
function sqlite3_finalize(pStmt:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_finalize';
function sqlite3_reset(pStmt:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_reset';
function sqlite3_create_function(_para1:Psqlite3; zFunctionName:Pchar; nArg:longint; eTextRep:longint; _para5:pointer;        xFunc:create_function_func_func; xStep:create_function_step_func; xFinal:create_function_final_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_create_function';
function sqlite3_create_function16(_para1:Psqlite3; zFunctionName:pointer; nArg:longint; eTextRep:longint; _para5:pointer;            xFunc:create_function_func_func; xStep:create_function_step_func; xFinal:create_function_final_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_create_function16';
function sqlite3_aggregate_count(_para1:Psqlite3_context):longint;cdecl; external Sqlite3Lib name 'sqlite3_aggregate_count';
function sqlite3_value_blob(_para1:Psqlite3_value):pointer;cdecl; external Sqlite3Lib name 'sqlite3_value_blob';
function sqlite3_value_bytes(_para1:Psqlite3_value):longint;cdecl; external Sqlite3Lib name 'sqlite3_value_bytes';
function sqlite3_value_bytes16(_para1:Psqlite3_value):longint;cdecl; external Sqlite3Lib name 'sqlite3_value_bytes16';
function sqlite3_value_double(_para1:Psqlite3_value):double;cdecl; external Sqlite3Lib name 'sqlite3_value_double';
function sqlite3_value_int(_para1:Psqlite3_value):longint;cdecl; external Sqlite3Lib name 'sqlite3_value_int';
function sqlite3_value_int64(_para1:Psqlite3_value):sqlite_int64;cdecl; external Sqlite3Lib name 'sqlite3_value_int64';
function sqlite3_value_text(_para1:Psqlite3_value):PChar;cdecl; external Sqlite3Lib name 'sqlite3_value_text';
function sqlite3_value_text16(_para1:Psqlite3_value):pointer;cdecl; external Sqlite3Lib name 'sqlite3_value_text16';
function sqlite3_value_text16le(_para1:Psqlite3_value):pointer;cdecl; external Sqlite3Lib name 'sqlite3_value_text16le';
function sqlite3_value_text16be(_para1:Psqlite3_value):pointer;cdecl; external Sqlite3Lib name 'sqlite3_value_text16be';
function sqlite3_value_type(_para1:Psqlite3_value):longint;cdecl; external Sqlite3Lib name 'sqlite3_value_type';
function sqlite3_aggregate_context(_para1:Psqlite3_context; nBytes:longint):pointer;cdecl; external Sqlite3Lib name 'sqlite3_aggregate_context';
function sqlite3_user_data(_para1:Psqlite3_context):pointer;cdecl; external Sqlite3Lib name 'sqlite3_user_data';
function sqlite3_get_auxdata(_para1:Psqlite3_context; _para2:longint):pointer;cdecl; external Sqlite3Lib name 'sqlite3_get_auxdata';
procedure sqlite3_set_auxdata(_para1:Psqlite3_context; _para2:longint; _para3:pointer; _para4:sqlite3_set_auxdata_func);cdecl; external Sqlite3Lib name 'sqlite3_set_auxdata';
procedure sqlite3_result_blob(_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl; external Sqlite3Lib name 'sqlite3_result_blob';
procedure sqlite3_result_double(_para1:Psqlite3_context; _para2:double);cdecl; external Sqlite3Lib name 'sqlite3_result_double';
procedure sqlite3_result_error(_para1:Psqlite3_context; _para2:Pchar; _para3:longint);cdecl; external Sqlite3Lib name 'sqlite3_result_error';
procedure sqlite3_result_error16(_para1:Psqlite3_context; _para2:pointer; _para3:longint);cdecl; external Sqlite3Lib name 'sqlite3_result_error16';
procedure sqlite3_result_int(_para1:Psqlite3_context; _para2:longint);cdecl; external Sqlite3Lib name 'sqlite3_result_int';
procedure sqlite3_result_int64(_para1:Psqlite3_context; _para2:sqlite_int64);cdecl; external Sqlite3Lib name 'sqlite3_result_int64';
procedure sqlite3_result_null(_para1:Psqlite3_context);cdecl; external Sqlite3Lib name 'sqlite3_result_null';
procedure sqlite3_result_text(_para1:Psqlite3_context; _para2:Pchar; _para3:longint; _para4:sqlite3_result_func);cdecl; external Sqlite3Lib name 'sqlite3_result_text';
procedure sqlite3_result_text16(_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl; external Sqlite3Lib name 'sqlite3_result_text16';
procedure sqlite3_result_text16le(_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl; external Sqlite3Lib name 'sqlite3_result_text16le';
procedure sqlite3_result_text16be(_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl; external Sqlite3Lib name 'sqlite3_result_text16be';
procedure sqlite3_result_value(_para1:Psqlite3_context; _para2:Psqlite3_value);cdecl; external Sqlite3Lib name 'sqlite3_result_value';     
function sqlite3_create_collation(_para1:Psqlite3; zName:Pchar; eTextRep:longint; _para4:pointer; xCompare:sqlite3_create_collation_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_create_collation';
function sqlite3_create_collation16(_para1:Psqlite3; zName:Pchar; eTextRep:longint; _para4:pointer; xCompare:sqlite3_create_collation_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_create_collation16';
function sqlite3_collation_needed(_para1:Psqlite3; _para2:pointer; _para3:sqlite3_collation_needed_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_collation_needed';
function sqlite3_collation_needed16(_para1:Psqlite3; _para2:pointer; _para3:sqlite3_collation_needed_func):longint;cdecl; external Sqlite3Lib name 'sqlite3_collation_needed16';
function sqlite3_libversion:PChar;cdecl; external Sqlite3Lib name 'sqlite3_libversion';
//Alias for allowing better code portability (win32 is not working with external variables) 
function sqlite3_version:PChar;cdecl; external Sqlite3Lib name 'sqlite3_libversion';

// Not published functions
function sqlite3_libversion_number:longint;cdecl; external Sqlite3Lib name 'sqlite3_libversion_number';
//function sqlite3_key(db:Psqlite3; pKey:pointer; nKey:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_key';
//function sqlite3_rekey(db:Psqlite3; pKey:pointer; nKey:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_rekey';
//function sqlite3_sleep(_para1:longint):longint;cdecl; external Sqlite3Lib name 'sqlite3_sleep';
//function sqlite3_expired(_para1:Psqlite3_stmt):longint;cdecl; external Sqlite3Lib name 'sqlite3_expired';
//function sqlite3_global_recover:longint;cdecl; external Sqlite3Lib name 'sqlite3_global_recover';

{$ELSE LOAD_DYNAMICALLY}

Var
  sqlite3_close : function (_para1:Psqlite3):longint;cdecl;
  sqlite3_exec : function (_para1:Psqlite3; sql:Pchar; _para3:sqlite3_callback; _para4:pointer; errmsg:PPchar):longint;cdecl;
  sqlite3_last_insert_rowid : function (_para1:Psqlite3):sqlite_int64;cdecl;
  sqlite3_changes : function (_para1:Psqlite3):longint;cdecl;
  sqlite3_total_changes : function (_para1:Psqlite3):longint;cdecl;
  sqlite3_interrupt : procedure (_para1:Psqlite3);cdecl;
  sqlite3_complete : function (sql:Pchar):longint;cdecl;
  sqlite3_complete16 : function (sql:pointer):longint;cdecl;
  sqlite3_busy_handler : function (_para1:Psqlite3; _para2:busy_handler_func; _para3:pointer):longint;cdecl;
  sqlite3_busy_timeout : function (_para1:Psqlite3; ms:longint):longint;cdecl;
  sqlite3_get_table : function (_para1:Psqlite3; sql:Pchar; resultp:PPPchar; nrow:Plongint; ncolumn:Plongint; errmsg:PPchar):longint;cdecl;
  sqlite3_free_table : procedure (result:PPchar);cdecl;
// Todo: see how translate sqlite3_mprintf, sqlite3_vmprintf, sqlite3_snprintf
//   sqlite3_mprintf : function (_para1:Pchar; args:array of const):Pchar;cdecl;
  sqlite3_mprintf : function (_para1:Pchar):Pchar;cdecl;
//  sqlite3_vmprintf : function (_para1:Pchar; _para2:va_list):Pchar;cdecl;
  sqlite3_free : procedure (z:Pchar);cdecl;
//  sqlite3_snprintf : function (_para1:longint; _para2:Pchar; _para3:Pchar; args:array of const):Pchar;cdecl;
  sqlite3_snprintf : function (_para1:longint; _para2:Pchar; _para3:Pchar):Pchar;cdecl;
  sqlite3_set_authorizer : function (_para1:Psqlite3; xAuth:sqlite3_set_authorizer_func; pUserData:pointer):longint;cdecl;
  sqlite3_trace : function (_para1:Psqlite3; xTrace:sqlite3_trace_func; _para3:pointer):pointer;cdecl;
  sqlite3_progress_handler : procedure (_para1:Psqlite3; _para2:longint; _para3:sqlite3_progress_handler_func; _para4:pointer);cdecl;
  sqlite3_commit_hook : function (_para1:Psqlite3; _para2:sqlite3_commit_hook_func; _para3:pointer):pointer;cdecl;
  sqlite3_open : function (filename:Pchar; ppDb:PPsqlite3):longint;cdecl;
  sqlite3_open16 : function (filename:pointer; ppDb:PPsqlite3):longint;cdecl;
  sqlite3_errcode : function (db:Psqlite3):longint;cdecl;
  sqlite3_errmsg : function (_para1:Psqlite3):Pchar;cdecl;
  sqlite3_errmsg16 : function (_para1:Psqlite3):pointer;cdecl;
  sqlite3_prepare : function (db:Psqlite3; zSql:Pchar; nBytes:longint; ppStmt:PPsqlite3_stmt; pzTail:PPchar):longint;cdecl;
  sqlite3_prepare16 : function (db:Psqlite3; zSql:pointer; nBytes:longint; ppStmt:PPsqlite3_stmt; pzTail:Ppointer):longint;cdecl;
  sqlite3_bind_blob : function (_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; n:longint; _para5:bind_destructor_func):longint;cdecl;
  sqlite3_bind_double : function (_para1:Psqlite3_stmt; _para2:longint; _para3:double):longint;cdecl;
  sqlite3_bind_int : function (_para1:Psqlite3_stmt; _para2:longint; _para3:longint):longint;cdecl;
  sqlite3_bind_int64 : function (_para1:Psqlite3_stmt; _para2:longint; _para3:sqlite_int64):longint;cdecl;
  sqlite3_bind_null : function (_para1:Psqlite3_stmt; _para2:longint):longint;cdecl;
  sqlite3_bind_text : function (_para1:Psqlite3_stmt; _para2:longint; _para3:Pchar; n:longint; _para5:bind_destructor_func):longint;cdecl;
  sqlite3_bind_text16 : function (_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; _para4:longint; _para5:bind_destructor_func):longint;cdecl;
//  sqlite3_bind_value : function (_para1:Psqlite3_stmt; _para2:longint; _para3:Psqlite3_value):longint;cdecl;
//These overloaded functions were introduced to allow the use of SQLITE_STATIC and SQLITE_TRANSIENT
//It's the c world man ;-)
  sqlite3_bind_blob1 : function (_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; n:longint; _para5:longint):longint;cdecl;
  sqlite3_bind_text1 : function (_para1:Psqlite3_stmt; _para2:longint; _para3:Pchar; n:longint; _para5:longint):longint;cdecl;
  sqlite3_bind_text161 : function (_para1:Psqlite3_stmt; _para2:longint; _para3:pointer; _para4:longint; _para5:longint):longint;cdecl;

  sqlite3_bind_parameter_count : function (_para1:Psqlite3_stmt):longint;cdecl;
  sqlite3_bind_parameter_name : function (_para1:Psqlite3_stmt; _para2:longint):Pchar;cdecl;
  sqlite3_bind_parameter_index : function (_para1:Psqlite3_stmt; zName:Pchar):longint;cdecl;
//  sqlite3_clear_bindings : function (_para1:Psqlite3_stmt):longint;cdecl;
  sqlite3_column_count : function (pStmt:Psqlite3_stmt):longint;cdecl;
  sqlite3_column_name : function (_para1:Psqlite3_stmt; _para2:longint):Pchar;cdecl;
  sqlite3_column_name16 : function (_para1:Psqlite3_stmt; _para2:longint):pointer;cdecl;
  sqlite3_column_decltype : function (_para1:Psqlite3_stmt; i:longint):Pchar;cdecl;
  sqlite3_column_decltype16 : function (_para1:Psqlite3_stmt; _para2:longint):pointer;cdecl;
  sqlite3_step : function (_para1:Psqlite3_stmt):longint;cdecl;
  sqlite3_data_count : function (pStmt:Psqlite3_stmt):longint;cdecl;
  sqlite3_column_blob : function (_para1:Psqlite3_stmt; iCol:longint):pointer;cdecl;
  sqlite3_column_bytes : function (_para1:Psqlite3_stmt; iCol:longint):longint;cdecl;
  sqlite3_column_bytes16 : function (_para1:Psqlite3_stmt; iCol:longint):longint;cdecl;
  sqlite3_column_double : function (_para1:Psqlite3_stmt; iCol:longint):double;cdecl;
  sqlite3_column_int : function (_para1:Psqlite3_stmt; iCol:longint):longint;cdecl;
  sqlite3_column_int64 : function (_para1:Psqlite3_stmt; iCol:longint):sqlite_int64;cdecl;
  sqlite3_column_text : function (_para1:Psqlite3_stmt; iCol:longint):PChar;cdecl;
  sqlite3_column_text16 : function (_para1:Psqlite3_stmt; iCol:longint):pointer;cdecl;
  sqlite3_column_type : function (_para1:Psqlite3_stmt; iCol:longint):longint;cdecl;
  sqlite3_finalize : function (pStmt:Psqlite3_stmt):longint;cdecl;
  sqlite3_reset : function (pStmt:Psqlite3_stmt):longint;cdecl;
  sqlite3_create_function : function (_para1:Psqlite3; zFunctionName:Pchar; nArg:longint; eTextRep:longint; _para5:pointer;        xFunc:create_function_func_func; xStep:create_function_step_func; xFinal:create_function_final_func):longint;cdecl;
  sqlite3_create_function16 : function (_para1:Psqlite3; zFunctionName:pointer; nArg:longint; eTextRep:longint; _para5:pointer;            xFunc:create_function_func_func; xStep:create_function_step_func; xFinal:create_function_final_func):longint;cdecl;
  sqlite3_aggregate_count : function (_para1:Psqlite3_context):longint;cdecl;
  sqlite3_value_blob : function (_para1:Psqlite3_value):pointer;cdecl;
  sqlite3_value_bytes : function (_para1:Psqlite3_value):longint;cdecl;
  sqlite3_value_bytes16 : function (_para1:Psqlite3_value):longint;cdecl;
  sqlite3_value_double : function (_para1:Psqlite3_value):double;cdecl;
  sqlite3_value_int : function (_para1:Psqlite3_value):longint;cdecl;
  sqlite3_value_int64 : function (_para1:Psqlite3_value):sqlite_int64;cdecl;
  sqlite3_value_text : function (_para1:Psqlite3_value):PChar;cdecl;
  sqlite3_value_text16 : function (_para1:Psqlite3_value):pointer;cdecl;
  sqlite3_value_text16le : function (_para1:Psqlite3_value):pointer;cdecl;
  sqlite3_value_text16be : function (_para1:Psqlite3_value):pointer;cdecl;
  sqlite3_value_type : function (_para1:Psqlite3_value):longint;cdecl;
  sqlite3_aggregate_context : function (_para1:Psqlite3_context; nBytes:longint):pointer;cdecl;
  sqlite3_user_data : function (_para1:Psqlite3_context):pointer;cdecl;
  sqlite3_get_auxdata : function (_para1:Psqlite3_context; _para2:longint):pointer;cdecl;
  sqlite3_set_auxdata : procedure (_para1:Psqlite3_context; _para2:longint; _para3:pointer; _para4:sqlite3_set_auxdata_func);cdecl;
  sqlite3_result_blob : procedure (_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl;
  sqlite3_result_double : procedure (_para1:Psqlite3_context; _para2:double);cdecl;
  sqlite3_result_error : procedure (_para1:Psqlite3_context; _para2:Pchar; _para3:longint);cdecl;
  sqlite3_result_error16 : procedure (_para1:Psqlite3_context; _para2:pointer; _para3:longint);cdecl;
  sqlite3_result_int : procedure (_para1:Psqlite3_context; _para2:longint);cdecl;
  sqlite3_result_int64 : procedure (_para1:Psqlite3_context; _para2:sqlite_int64);cdecl;
  sqlite3_result_null : procedure (_para1:Psqlite3_context);cdecl;
  sqlite3_result_text : procedure (_para1:Psqlite3_context; _para2:Pchar; _para3:longint; _para4:sqlite3_result_func);cdecl;
  sqlite3_result_text16 : procedure (_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl;
  sqlite3_result_text16le : procedure (_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl;
  sqlite3_result_text16be : procedure (_para1:Psqlite3_context; _para2:pointer; _para3:longint; _para4:sqlite3_result_func);cdecl;
  sqlite3_result_value : procedure (_para1:Psqlite3_context; _para2:Psqlite3_value);cdecl;     
  sqlite3_create_collation : function (_para1:Psqlite3; zName:Pchar; eTextRep:longint; _para4:pointer; xCompare:sqlite3_create_collation_func):longint;cdecl;
  sqlite3_create_collation16 : function (_para1:Psqlite3; zName:Pchar; eTextRep:longint; _para4:pointer; xCompare:sqlite3_create_collation_func):longint;cdecl;
  sqlite3_collation_needed : function (_para1:Psqlite3; _para2:pointer; _para3:sqlite3_collation_needed_func):longint;cdecl;
  sqlite3_collation_needed16 : function (_para1:Psqlite3; _para2:pointer; _para3:sqlite3_collation_needed_func):longint;cdecl;
  sqlite3_libversion: function : PChar;cdecl;
//Alias for allowing better code portability (win32 is not working with external variables) 
  sqlite3_version: function: PChar;cdecl;

// Not published functions
  sqlite3_libversion_number : Function :longint;cdecl;
//  sqlite3_key : function (db:Psqlite3; pKey:pointer; nKey:longint):longint;cdecl;
//  sqlite3_rekey : function (db:Psqlite3; pKey:pointer; nKey:longint):longint;cdecl;
//  sqlite3_sleep : function (_para1:longint):longint;cdecl;
//  sqlite3_expired : function (_para1:Psqlite3_stmt):longint;cdecl;
//function sqlite3_global_recover:longint;cdecl;

Procedure InitialiseSQLite;
Procedure InitialiseSQLite(LibraryName : String);
Procedure ReleaseSQLite;

var 
  SQLiteLibraryHandle : TLibHandle;
  DefaultLibrary : String = Sqlite3Lib;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF LOAD_DYNAMICALLY}

implementation

<<<<<<< HEAD
<<<<<<< HEAD
function sqlite3_version(): pansichar;
begin
  Result := sqlite3_libversion();
end;

{$IFDEF LOAD_DYNAMICALLY}

resourcestring
  SErrLoadFailed     = 'Can not load SQLite client library "%s". Check your installation.';
  SErrAlreadyLoaded  = 'SQLite interface already initialized from library %s.';

procedure LoadAddresses(LibHandle: TLibHandle);
begin
  pointer(sqlite3_libversion) := GetProcedureAddress(LibHandle,'sqlite3_libversion');
  pointer(sqlite3_libversion_number) := GetProcedureAddress(LibHandle,'sqlite3_libversion_number');
  pointer(sqlite3_threadsafe) := GetProcedureAddress(LibHandle,'sqlite3_threadsafe');
  pointer(sqlite3_close) := GetProcedureAddress(LibHandle,'sqlite3_close');
  pointer(sqlite3_exec) := GetProcedureAddress(LibHandle,'sqlite3_exec');
  pointer(sqlite3_extended_result_codes) := GetProcedureAddress(LibHandle,'sqlite3_extended_result_codes');
  pointer(sqlite3_last_insert_rowid) := GetProcedureAddress(LibHandle,'sqlite3_last_insert_rowid');
  pointer(sqlite3_changes) := GetProcedureAddress(LibHandle,'sqlite3_changes');
  pointer(sqlite3_total_changes) := GetProcedureAddress(LibHandle,'sqlite3_total_changes');
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF LOAD_DYNAMICALLY}

ResourceString
  SErrLoadFailed     = 'Can not load SQLite client library "%s". Check your installation.';
  SErrAlreadyLoaded  = 'SQLIte interface already initialized from library %s.';

Procedure LoadAddresses(LibHandle : TLibHandle);

begin
  pointer(sqlite3_close) := GetProcedureAddress(LibHandle,'sqlite3_close');
  pointer(sqlite3_exec) := GetProcedureAddress(LibHandle,'sqlite3_exec');
  pointer(sqlite3_last_insert_rowid) := GetProcedureAddress(LibHandle,'sqlite3_last_insert_rowid');
  pointer(sqlite3_changes) := GetProcedureAddress(LibHandle,'sqlite3_changes');
  pointer(sqlite3_total_changes) := GetProcedureAddress(LibHandle,'sqlite3_total_changes');
  pointer(sqlite3_interrupt) := GetProcedureAddress(LibHandle,'sqlite3_interrupt');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_complete) := GetProcedureAddress(LibHandle,'sqlite3_complete');
  pointer(sqlite3_complete16) := GetProcedureAddress(LibHandle,'sqlite3_complete16');
  pointer(sqlite3_busy_handler) := GetProcedureAddress(LibHandle,'sqlite3_busy_handler');
  pointer(sqlite3_busy_timeout) := GetProcedureAddress(LibHandle,'sqlite3_busy_timeout');
  pointer(sqlite3_get_table) := GetProcedureAddress(LibHandle,'sqlite3_get_table');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_malloc) := GetProcedureAddress(LibHandle,'sqlite3_malloc');
  pointer(sqlite3_realloc) := GetProcedureAddress(LibHandle,'sqlite3_realloc');
  pointer(sqlite3_memory_used) := GetProcedureAddress(LibHandle,'sqlite3_memory_used');
  pointer(sqlite3_memory_highwater) := GetProcedureAddress(LibHandle,'sqlite3_memory_highwater');
  pointer(sqlite3_set_authorizer) := GetProcedureAddress(LibHandle,'sqlite3_set_authorizer');
  pointer(sqlite3_trace) := GetProcedureAddress(LibHandle,'sqlite3_trace');
  pointer(sqlite3_profile) := GetProcedureAddress(LibHandle,'sqlite3_profile');
  pointer(sqlite3_open) := GetProcedureAddress(LibHandle,'sqlite3_open');
  pointer(sqlite3_open16) := GetProcedureAddress(LibHandle,'sqlite3_open16');
  pointer(sqlite3_open_v2) := GetProcedureAddress(LibHandle,'sqlite3_open_v2');
  pointer(sqlite3_errcode) := GetProcedureAddress(LibHandle,'sqlite3_errcode');
  pointer(sqlite3_extended_errcode) := GetProcedureAddress(LibHandle,'sqlite3_extended_errcode');
  pointer(sqlite3_errmsg) := GetProcedureAddress(LibHandle,'sqlite3_errmsg');
  pointer(sqlite3_errmsg16) := GetProcedureAddress(LibHandle,'sqlite3_errmsg16');
  pointer(sqlite3_limit) := GetProcedureAddress(LibHandle,'sqlite3_limit');
  pointer(sqlite3_prepare) := GetProcedureAddress(LibHandle,'sqlite3_prepare');
  pointer(sqlite3_prepare_v2) := GetProcedureAddress(LibHandle,'sqlite3_prepare_v2');
  pointer(sqlite3_prepare16) := GetProcedureAddress(LibHandle,'sqlite3_prepare16');
  pointer(sqlite3_prepare16_v2) := GetProcedureAddress(LibHandle,'sqlite3_prepare16_v2');
  pointer(sqlite3_sql) := GetProcedureAddress(LibHandle,'sqlite3_sql');
=======
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_free_table) := GetProcedureAddress(LibHandle,'sqlite3_free_table');
// Todo: see how translate sqlite3_mprintf, sqlite3_vmprintf, sqlite3_snprintf
//   pointer(sqlite3_mprintf) := GetProcedureAddress(LibHandle,'sqlite3_mprintf');
  pointer(sqlite3_mprintf) := GetProcedureAddress(LibHandle,'sqlite3_mprintf');
//  pointer(sqlite3_vmprintf) := GetProcedureAddress(LibHandle,'sqlite3_vmprintf');
  pointer(sqlite3_free) := GetProcedureAddress(LibHandle,'sqlite3_free');
//  pointer(sqlite3_snprintf) := GetProcedureAddress(LibHandle,'sqlite3_snprintf');
  pointer(sqlite3_snprintf) := GetProcedureAddress(LibHandle,'sqlite3_snprintf');
  pointer(sqlite3_set_authorizer) := GetProcedureAddress(LibHandle,'sqlite3_set_authorizer');
  pointer(sqlite3_trace) := GetProcedureAddress(LibHandle,'sqlite3_trace');
  pointer(sqlite3_progress_handler) := GetProcedureAddress(LibHandle,'sqlite3_progress_handler');
  pointer(sqlite3_commit_hook) := GetProcedureAddress(LibHandle,'sqlite3_commit_hook');
  pointer(sqlite3_open) := GetProcedureAddress(LibHandle,'sqlite3_open');
  pointer(sqlite3_open16) := GetProcedureAddress(LibHandle,'sqlite3_open16');
  pointer(sqlite3_errcode) := GetProcedureAddress(LibHandle,'sqlite3_errcode');
  pointer(sqlite3_errmsg) := GetProcedureAddress(LibHandle,'sqlite3_errmsg');
  pointer(sqlite3_errmsg16) := GetProcedureAddress(LibHandle,'sqlite3_errmsg16');
  pointer(sqlite3_prepare) := GetProcedureAddress(LibHandle,'sqlite3_prepare');
  pointer(sqlite3_prepare16) := GetProcedureAddress(LibHandle,'sqlite3_prepare16');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_bind_blob) := GetProcedureAddress(LibHandle,'sqlite3_bind_blob');
  pointer(sqlite3_bind_double) := GetProcedureAddress(LibHandle,'sqlite3_bind_double');
  pointer(sqlite3_bind_int) := GetProcedureAddress(LibHandle,'sqlite3_bind_int');
  pointer(sqlite3_bind_int64) := GetProcedureAddress(LibHandle,'sqlite3_bind_int64');
  pointer(sqlite3_bind_null) := GetProcedureAddress(LibHandle,'sqlite3_bind_null');
  pointer(sqlite3_bind_text) := GetProcedureAddress(LibHandle,'sqlite3_bind_text');
  pointer(sqlite3_bind_text16) := GetProcedureAddress(LibHandle,'sqlite3_bind_text16');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_bind_value) := GetProcedureAddress(LibHandle,'sqlite3_bind_value');
  pointer(sqlite3_bind_zeroblob) := GetProcedureAddress(LibHandle,'sqlite3_bind_zeroblob');
  pointer(sqlite3_bind_parameter_count) := GetProcedureAddress(LibHandle,'sqlite3_bind_parameter_count');
  pointer(sqlite3_bind_parameter_name) := GetProcedureAddress(LibHandle,'sqlite3_bind_parameter_name');
  pointer(sqlite3_bind_parameter_index) := GetProcedureAddress(LibHandle,'sqlite3_bind_parameter_index');
  pointer(sqlite3_clear_bindings) := GetProcedureAddress(LibHandle,'sqlite3_clear_bindings');
  pointer(sqlite3_column_count) := GetProcedureAddress(LibHandle,'sqlite3_column_count');
  pointer(sqlite3_column_name) := GetProcedureAddress(LibHandle,'sqlite3_column_name');
  pointer(sqlite3_column_name16) := GetProcedureAddress(LibHandle,'sqlite3_column_name16');
  pointer(sqlite3_column_database_name) := GetProcedureAddress(LibHandle,'sqlite3_column_database_name');
  pointer(sqlite3_column_database_name16) := GetProcedureAddress(LibHandle,'sqlite3_column_database_name16');
  pointer(sqlite3_column_table_name) := GetProcedureAddress(LibHandle,'sqlite3_column_table_name');
  pointer(sqlite3_column_table_name16) := GetProcedureAddress(LibHandle,'sqlite3_column_table_name16');
  pointer(sqlite3_column_origin_name) := GetProcedureAddress(LibHandle,'sqlite3_column_origin_name');
  pointer(sqlite3_column_origin_name16) := GetProcedureAddress(LibHandle,'sqlite3_column_origin_name16');
=======
=======
>>>>>>> origin/fixes_2_2
//  pointer(sqlite3_bind_value) := GetProcedureAddress(LibHandle,'sqlite3_bind_value');
//These overloaded functions were introduced to allow the use of SQLITE_STATIC and SQLITE_TRANSIENT
//It's the c world man ;-)
  pointer(sqlite3_bind_blob1) := GetProcedureAddress(LibHandle,'sqlite3_bind_blob');
  pointer(sqlite3_bind_text1) := GetProcedureAddress(LibHandle,'sqlite3_bind_text');
  pointer(sqlite3_bind_text161) := GetProcedureAddress(LibHandle,'sqlite3_bind_text16');

  pointer(sqlite3_bind_parameter_count) := GetProcedureAddress(LibHandle,'sqlite3_bind_parameter_count');
  pointer(sqlite3_bind_parameter_name) := GetProcedureAddress(LibHandle,'sqlite3_bind_parameter_name');
  pointer(sqlite3_bind_parameter_index) := GetProcedureAddress(LibHandle,'sqlite3_bind_parameter_index');
//  pointer(sqlite3_clear_bindings) := GetProcedureAddress(LibHandle,'sqlite3_clear_bindings');
  pointer(sqlite3_column_count) := GetProcedureAddress(LibHandle,'sqlite3_column_count');
  pointer(sqlite3_column_name) := GetProcedureAddress(LibHandle,'sqlite3_column_name');
  pointer(sqlite3_column_name16) := GetProcedureAddress(LibHandle,'sqlite3_column_name16');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_column_decltype) := GetProcedureAddress(LibHandle,'sqlite3_column_decltype');
  pointer(sqlite3_column_decltype16) := GetProcedureAddress(LibHandle,'sqlite3_column_decltype16');
  pointer(sqlite3_step) := GetProcedureAddress(LibHandle,'sqlite3_step');
  pointer(sqlite3_data_count) := GetProcedureAddress(LibHandle,'sqlite3_data_count');
  pointer(sqlite3_column_blob) := GetProcedureAddress(LibHandle,'sqlite3_column_blob');
  pointer(sqlite3_column_bytes) := GetProcedureAddress(LibHandle,'sqlite3_column_bytes');
  pointer(sqlite3_column_bytes16) := GetProcedureAddress(LibHandle,'sqlite3_column_bytes16');
  pointer(sqlite3_column_double) := GetProcedureAddress(LibHandle,'sqlite3_column_double');
  pointer(sqlite3_column_int) := GetProcedureAddress(LibHandle,'sqlite3_column_int');
  pointer(sqlite3_column_int64) := GetProcedureAddress(LibHandle,'sqlite3_column_int64');
  pointer(sqlite3_column_text) := GetProcedureAddress(LibHandle,'sqlite3_column_text');
  pointer(sqlite3_column_text16) := GetProcedureAddress(LibHandle,'sqlite3_column_text16');
  pointer(sqlite3_column_type) := GetProcedureAddress(LibHandle,'sqlite3_column_type');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_column_value) := GetProcedureAddress(LibHandle,'sqlite3_column_value');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_finalize) := GetProcedureAddress(LibHandle,'sqlite3_finalize');
  pointer(sqlite3_reset) := GetProcedureAddress(LibHandle,'sqlite3_reset');
  pointer(sqlite3_create_function) := GetProcedureAddress(LibHandle,'sqlite3_create_function');
  pointer(sqlite3_create_function16) := GetProcedureAddress(LibHandle,'sqlite3_create_function16');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_create_function_v2) := GetProcedureAddress(LibHandle,'sqlite3_create_function_v2');
=======
  pointer(sqlite3_aggregate_count) := GetProcedureAddress(LibHandle,'sqlite3_aggregate_count');
>>>>>>> graemeg/fixes_2_2
=======
  pointer(sqlite3_aggregate_count) := GetProcedureAddress(LibHandle,'sqlite3_aggregate_count');
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_value_blob) := GetProcedureAddress(LibHandle,'sqlite3_value_blob');
  pointer(sqlite3_value_bytes) := GetProcedureAddress(LibHandle,'sqlite3_value_bytes');
  pointer(sqlite3_value_bytes16) := GetProcedureAddress(LibHandle,'sqlite3_value_bytes16');
  pointer(sqlite3_value_double) := GetProcedureAddress(LibHandle,'sqlite3_value_double');
  pointer(sqlite3_value_int) := GetProcedureAddress(LibHandle,'sqlite3_value_int');
  pointer(sqlite3_value_int64) := GetProcedureAddress(LibHandle,'sqlite3_value_int64');
  pointer(sqlite3_value_text) := GetProcedureAddress(LibHandle,'sqlite3_value_text');
  pointer(sqlite3_value_text16) := GetProcedureAddress(LibHandle,'sqlite3_value_text16');
  pointer(sqlite3_value_text16le) := GetProcedureAddress(LibHandle,'sqlite3_value_text16le');
  pointer(sqlite3_value_text16be) := GetProcedureAddress(LibHandle,'sqlite3_value_text16be');
  pointer(sqlite3_value_type) := GetProcedureAddress(LibHandle,'sqlite3_value_type');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_value_numeric_type) := GetProcedureAddress(LibHandle,'sqlite3_value_numeric_type');
  pointer(sqlite3_aggregate_context) := GetProcedureAddress(LibHandle,'sqlite3_aggregate_context');
  pointer(sqlite3_user_data) := GetProcedureAddress(LibHandle,'sqlite3_user_data');
  pointer(sqlite3_context_db_handle) := GetProcedureAddress(LibHandle,'sqlite3_context_db_handle');
  pointer(sqlite3_get_auxdata) := GetProcedureAddress(LibHandle,'sqlite3_get_auxdata');
  pointer(sqlite3_create_collation) := GetProcedureAddress(LibHandle,'sqlite3_create_collation');
  pointer(sqlite3_create_collation_v2) := GetProcedureAddress(LibHandle,'sqlite3_create_collation_v2');
  pointer(sqlite3_create_collation16) := GetProcedureAddress(LibHandle,'sqlite3_create_collation16');
  pointer(sqlite3_collation_needed) := GetProcedureAddress(LibHandle,'sqlite3_collation_needed');
  pointer(sqlite3_collation_needed16) := GetProcedureAddress(LibHandle,'sqlite3_collation_needed16');
  pointer(sqlite3_key) := GetProcedureAddress(LibHandle,'sqlite3_key');
  pointer(sqlite3_rekey) := GetProcedureAddress(LibHandle,'sqlite3_rekey');
  pointer(sqlite3_sleep) := GetProcedureAddress(LibHandle,'sqlite3_sleep');
  pointer(sqlite3_get_autocommit) := GetProcedureAddress(LibHandle,'sqlite3_get_autocommit');
  pointer(sqlite3_db_handle) := GetProcedureAddress(LibHandle,'sqlite3_db_handle');
  pointer(sqlite3_commit_hook) := GetProcedureAddress(LibHandle,'sqlite3_commit_hook');
  pointer(sqlite3_rollback_hook) := GetProcedureAddress(LibHandle,'sqlite3_rollback_hook');
  pointer(sqlite3_update_hook) := GetProcedureAddress(LibHandle,'sqlite3_update_hook');
  pointer(sqlite3_enable_shared_cache) := GetProcedureAddress(LibHandle,'sqlite3_enable_shared_cache');
  pointer(sqlite3_release_memory) := GetProcedureAddress(LibHandle,'sqlite3_release_memory');
  pointer(sqlite3_table_column_metadata) := GetProcedureAddress(LibHandle,'sqlite3_table_column_metadata');
  pointer(sqlite3_load_extension) := GetProcedureAddress(LibHandle,'sqlite3_load_extension');
  pointer(sqlite3_enable_load_extension) := GetProcedureAddress(LibHandle,'sqlite3_enable_load_extension');
  pointer(sqlite3_auto_extension) := GetProcedureAddress(LibHandle,'sqlite3_auto_extension');
  pointer(sqlite3_create_module) := GetProcedureAddress(LibHandle,'sqlite3_create_module');
  pointer(sqlite3_create_module_v2) := GetProcedureAddress(LibHandle,'sqlite3_create_module_v2');
  pointer(sqlite3_declare_vtab) := GetProcedureAddress(LibHandle,'sqlite3_declare_vtab');
  pointer(sqlite3_overload_function) := GetProcedureAddress(LibHandle,'sqlite3_overload_function');
  pointer(sqlite3_blob_open) := GetProcedureAddress(LibHandle,'sqlite3_blob_open');
  pointer(sqlite3_blob_reopen) := GetProcedureAddress(LibHandle,'sqlite3_blob_reopen');
  pointer(sqlite3_blob_close) := GetProcedureAddress(LibHandle,'sqlite3_blob_close');
  pointer(sqlite3_blob_bytes) := GetProcedureAddress(LibHandle,'sqlite3_blob_bytes');
  pointer(sqlite3_blob_read) := GetProcedureAddress(LibHandle,'sqlite3_blob_read');
  pointer(sqlite3_blob_write) := GetProcedureAddress(LibHandle,'sqlite3_blob_write');
  pointer(sqlite3_vfs_find) := GetProcedureAddress(LibHandle,'sqlite3_vfs_find');
  pointer(sqlite3_vfs_register) := GetProcedureAddress(LibHandle,'sqlite3_vfs_register');
  pointer(sqlite3_vfs_unregister) := GetProcedureAddress(LibHandle,'sqlite3_vfs_unregister');
  pointer(sqlite3_mutex_alloc) := GetProcedureAddress(LibHandle,'sqlite3_mutex_alloc');
  pointer(sqlite3_mutex_try) := GetProcedureAddress(LibHandle,'sqlite3_mutex_try');
  pointer(sqlite3_mutex_held) := GetProcedureAddress(LibHandle,'sqlite3_mutex_held');
  pointer(sqlite3_mutex_notheld) := GetProcedureAddress(LibHandle,'sqlite3_mutex_notheld');
  pointer(sqlite3_db_mutex) := GetProcedureAddress(LibHandle,'sqlite3_db_mutex');
  pointer(sqlite3_file_control) := GetProcedureAddress(LibHandle,'sqlite3_file_control');
  pointer(sqlite3_test_control) := GetProcedureAddress(LibHandle,'sqlite3_test_control');
  pointer(sqlite3_status) := GetProcedureAddress(LibHandle,'sqlite3_status');
  pointer(sqlite3_db_status) := GetProcedureAddress(LibHandle,'sqlite3_db_status');
  pointer(sqlite3_stmt_status) := GetProcedureAddress(LibHandle,'sqlite3_stmt_status');
  pointer(sqlite3_interrupt) := GetProcedureAddress(LibHandle,'sqlite3_interrupt');
  pointer(sqlite3_free_table) := GetProcedureAddress(LibHandle,'sqlite3_free_table');
  pointer(sqlite3_free) := GetProcedureAddress(LibHandle,'sqlite3_free');
  pointer(sqlite3_randomness) := GetProcedureAddress(LibHandle,'sqlite3_randomness');
  pointer(sqlite3_progress_handler) := GetProcedureAddress(LibHandle,'sqlite3_progress_handler');
=======
  pointer(sqlite3_aggregate_context) := GetProcedureAddress(LibHandle,'sqlite3_aggregate_context');
  pointer(sqlite3_user_data) := GetProcedureAddress(LibHandle,'sqlite3_user_data');
  pointer(sqlite3_get_auxdata) := GetProcedureAddress(LibHandle,'sqlite3_get_auxdata');
>>>>>>> graemeg/fixes_2_2
=======
  pointer(sqlite3_aggregate_context) := GetProcedureAddress(LibHandle,'sqlite3_aggregate_context');
  pointer(sqlite3_user_data) := GetProcedureAddress(LibHandle,'sqlite3_user_data');
  pointer(sqlite3_get_auxdata) := GetProcedureAddress(LibHandle,'sqlite3_get_auxdata');
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_set_auxdata) := GetProcedureAddress(LibHandle,'sqlite3_set_auxdata');
  pointer(sqlite3_result_blob) := GetProcedureAddress(LibHandle,'sqlite3_result_blob');
  pointer(sqlite3_result_double) := GetProcedureAddress(LibHandle,'sqlite3_result_double');
  pointer(sqlite3_result_error) := GetProcedureAddress(LibHandle,'sqlite3_result_error');
  pointer(sqlite3_result_error16) := GetProcedureAddress(LibHandle,'sqlite3_result_error16');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_result_error_toobig) := GetProcedureAddress(LibHandle,'sqlite3_result_error_toobig');
  pointer(sqlite3_result_error_nomem) := GetProcedureAddress(LibHandle,'sqlite3_result_error_nomem');
  pointer(sqlite3_result_error_code) := GetProcedureAddress(LibHandle,'sqlite3_result_error_code');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_result_int) := GetProcedureAddress(LibHandle,'sqlite3_result_int');
  pointer(sqlite3_result_int64) := GetProcedureAddress(LibHandle,'sqlite3_result_int64');
  pointer(sqlite3_result_null) := GetProcedureAddress(LibHandle,'sqlite3_result_null');
  pointer(sqlite3_result_text) := GetProcedureAddress(LibHandle,'sqlite3_result_text');
  pointer(sqlite3_result_text16) := GetProcedureAddress(LibHandle,'sqlite3_result_text16');
  pointer(sqlite3_result_text16le) := GetProcedureAddress(LibHandle,'sqlite3_result_text16le');
  pointer(sqlite3_result_text16be) := GetProcedureAddress(LibHandle,'sqlite3_result_text16be');
  pointer(sqlite3_result_value) := GetProcedureAddress(LibHandle,'sqlite3_result_value');
<<<<<<< HEAD
<<<<<<< HEAD
  pointer(sqlite3_result_zeroblob) := GetProcedureAddress(LibHandle,'sqlite3_result_zeroblob');
  pointer(sqlite3_soft_heap_limit) := GetProcedureAddress(LibHandle,'sqlite3_soft_heap_limit');
  pointer(sqlite3_soft_heap_limit64) := GetProcedureAddress(LibHandle,'sqlite3_soft_heap_limit64');
  pointer(sqlite3_reset_auto_extension) := GetProcedureAddress(LibHandle,'sqlite3_reset_auto_extension');
  pointer(sqlite3_mutex_free) := GetProcedureAddress(LibHandle,'sqlite3_mutex_free');
  pointer(sqlite3_mutex_ente) := GetProcedureAddress(LibHandle,'sqlite3_mutex_ente');
  pointer(sqlite3_mutex_leave) := GetProcedureAddress(LibHandle,'sqlite3_mutex_leave');
  pointer(sqlite3_backup_init) := GetProcedureAddress(LibHandle,'sqlite3_backup_init');
  pointer(sqlite3_backup_step) := GetProcedureAddress(LibHandle,'sqlite3_backup_step');
  pointer(sqlite3_backup_finish) := GetProcedureAddress(LibHandle,'sqlite3_backup_finish');
  pointer(sqlite3_backup_remaining) := GetProcedureAddress(LibHandle,'sqlite3_backup_remaining');
  pointer(sqlite3_backup_pagecount) := GetProcedureAddress(LibHandle,'sqlite3_backup_pagecount');
  pointer(sqlite3_unlock_notify) := GetProcedureAddress(LibHandle,'sqlite3_unlock_notify');
  pointer(sqlite3_log) := GetProcedureAddress(LibHandle,'sqlite3_log');
  pointer(sqlite3_wal_hook) := GetProcedureAddress(LibHandle,'sqlite3_wal_hook');
  pointer(sqlite3_wal_autocheckpoint) := GetProcedureAddress(LibHandle,'sqlite3_wal_autocheckpoint');
  pointer(sqlite3_wal_checkpoint) := GetProcedureAddress(LibHandle,'sqlite3_wal_checkpoint');
  pointer(sqlite3_wal_checkpoint_v2) := GetProcedureAddress(LibHandle,'sqlite3_wal_checkpoint_v2');
  
   
  pointer(sqlite3_initialize) := GetProcedureAddress(LibHandle,'sqlite3_initialize');
  pointer(sqlite3_shutdown) := GetProcedureAddress(LibHandle,'sqlite3_shutdown');
  pointer(sqlite3_os_init) := GetProcedureAddress(LibHandle,'sqlite3_os_init');
  pointer(sqlite3_os_end) := GetProcedureAddress(LibHandle,'sqlite3_os_end');
  pointer(sqlite3_config) := GetProcedureAddress(LibHandle,'sqlite3_config');
  pointer(sqlite3_db_config) := GetProcedureAddress(LibHandle,'sqlite3_db_config');
  pointer(sqlite3_uri_parameter) := GetProcedureAddress(LibHandle,'sqlite3_uri_parameter');

{$IFDEF SQLITE_OBSOLETE}
  pointer(sqlite3_aggregate_count) := GetProcedureAddress(LibHandle,'sqlite3_aggregate_count');
  pointer(sqlite3_expired) := GetProcedureAddress(LibHandle,'sqlite3_expired');
  pointer(sqlite3_transfer_bindings) := GetProcedureAddress(LibHandle,'sqlite3_transfer_bindings');
  pointer(sqlite3_global_recover) := GetProcedureAddress(LibHandle,'sqlite3_global_recover');
  pointer(sqlite3_memory_alarm) := GetProcedureAddress(LibHandle,'sqlite3_memory_alarm');
  pointer(sqlite3_thread_cleanup) := GetProcedureAddress(LibHandle,'sqlite3_thread_cleanup');
{$ENDIF}
end;

var
  RefCount: Integer;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TryInitializeSqlite(const LibraryName: UnicodeString): Integer;

Var
  N  : UnicodeString;
  
begin
  N:=LibraryName;
  if (N='') then
    N:=SQLiteDefaultLibrary;
  result:=InterlockedIncrement(RefCount);
  if result  = 1 then
  begin
    SQLiteLibraryHandle := LoadLibrary(N);
=======
=======
>>>>>>> graemeg/cpstrnew
function TryInitializeSqlite(const LibraryName: string): Integer;
begin
=======
function TryInitializeSqlite(const LibraryName: string): Integer;
begin
>>>>>>> graemeg/cpstrnew
=======
function TryInitializeSqlite(const LibraryName: string): Integer;
begin
>>>>>>> origin/cpstrnew
  result:=InterlockedIncrement(RefCount);
  if result  = 1 then
  begin
    SQLiteLibraryHandle := LoadLibrary(LibraryName);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    if (SQLiteLibraryHandle = NilHandle) then
    begin
      RefCount := 0;
      Exit(-1);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    SQLiteLoadedLibrary := N;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    LoadedLibrary := LibraryName;
>>>>>>> graemeg/cpstrnew
    LoadAddresses(SQLiteLibraryHandle);
  end;
end;

function InitialiseSQLite:integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  result:=InitializeSqlite(SQLiteDefaultLibrary);
end;

function  InitializeSQLiteANSI(const LibraryName: AnsiString):integer;
begin
  result:=InitializeSQLite(LibraryName);
end;

function  InitializeSQLite(const LibraryName: UnicodeString) :integer;
begin
  if (LibraryName<>'') and (SQLiteLoadedLibrary <> '') and (SQLiteLoadedLibrary <> LibraryName) then
    raise EInoutError.CreateFmt(SErrAlreadyLoaded,[SQLiteLoadedLibrary]);

  result:= TryInitializeSQLite(LibraryName);
  if result=-1 then
    if LibraryName='' then
      raise EInOutError.CreateFmt(SErrLoadFailed,[SQLiteDefaultLibrary])
    else
      raise EInOutError.CreateFmt(SErrLoadFailed,[LibraryName]);
end;

function  InitialiseSQLite(const LibraryName: UnicodeString):integer;
begin
  result:=InitializeSqlite(LibraryName);
end;

function  ReleaseSQLite:integer;
begin
  if InterlockedDecrement(RefCount) <= 0 then
  begin
    if SQLiteLibraryHandle <> NilHandle then
      UnloadLibrary(SQLiteLibraryHandle);
    SQLiteLibraryHandle := NilHandle;
    SQLiteLoadedLibrary := '';
    RefCount := 0;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
begin
=======
begin
>>>>>>> graemeg/cpstrnew
=======
begin
>>>>>>> origin/cpstrnew
  result:=InitializeSqlite(SQLiteDefaultLibrary);
end;

function  InitializeSQLite(const LibraryName: String) :integer;
begin
  if (LoadedLibrary <> '') and (LoadedLibrary <> LibraryName) then
    raise EInoutError.CreateFmt(SErrAlreadyLoaded,[LoadedLibrary]);

  result:= TryInitializeSQLIte(LibraryName);
  if result=-1 then
    raise EInOutError.CreateFmt(SErrLoadFailed,[LibraryName]);
>>>>>>> graemeg/cpstrnew
end;

function  InitialiseSQLite(const LibraryName: String):integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ReleaseSQLite;
end;

=======
  result:=InitializeSqlite(LibraryName);
end;

=======
  result:=InitializeSqlite(LibraryName);
end;

>>>>>>> graemeg/cpstrnew
=======
  result:=InitializeSqlite(LibraryName);
end;

>>>>>>> origin/cpstrnew
function  ReleaseSQLite:integer;
begin
  if InterlockedDecrement(RefCount) <= 0 then
  begin
    if SQLiteLibraryHandle <> NilHandle then
      UnloadLibrary(SQLiteLibraryHandle);
    SQLiteLibraryHandle := NilHandle;
    LoadedLibrary := '';
    RefCount := 0;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  result:=InitializeSqlite(LibraryName);
end;

function  ReleaseSQLite:integer;
begin
  if InterlockedDecrement(RefCount) <= 0 then
  begin
    if SQLiteLibraryHandle <> NilHandle then
      UnloadLibrary(SQLiteLibraryHandle);
    SQLiteLibraryHandle := NilHandle;
    LoadedLibrary := '';
    RefCount := 0;
  end;
end;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ENDIF}
=======
=======
>>>>>>> origin/fixes_2_2
  pointer(sqlite3_create_collation) := GetProcedureAddress(LibHandle,'sqlite3_create_collation');
  pointer(sqlite3_create_collation16) := GetProcedureAddress(LibHandle,'sqlite3_create_collation16');
  pointer(sqlite3_collation_needed) := GetProcedureAddress(LibHandle,'sqlite3_collation_needed');
  pointer(sqlite3_collation_needed16) := GetProcedureAddress(LibHandle,'sqlite3_collation_needed16');
  pointer(sqlite3_libversion):=GetProcedureAddress(LibHandle,'sqlite3_libversion');
//Alias for allowing better code portability (win32 is not working with external variables) 
  pointer(sqlite3_version):=GetProcedureAddress(LibHandle,'sqlite3_libversion');

// Not published functions
  pointer(sqlite3_libversion_number):=GetProcedureAddress(LibHandle,'sqlite3_libversion_number');
//  pointer(sqlite3_key) := GetProcedureAddress(LibHandle,'sqlite3_key');
//  pointer(sqlite3_rekey) := GetProcedureAddress(LibHandle,'sqlite3_rekey');
//  pointer(sqlite3_sleep) := GetProcedureAddress(LibHandle,'sqlite3_sleep');
//  pointer(sqlite3_expired) := GetProcedureAddress(LibHandle,'sqlite3_expired');
// function sqlite3_global_recover:longint;cdecl;
end;
        
var
  RefCount : integer;
  LoadedLibrary : String;
            

Function TryInitialiseSqlite(Const LibraryName : String) : Boolean;
            
begin
  Result:=False;
  if (RefCount=0) then
    begin
    SQLiteLibraryHandle:=LoadLibrary(LibraryName);
    Result:=(SQLiteLibraryHandle<>nilhandle);
    If not Result then
      Exit;
    inc(RefCount);
    LoadedLibrary:=LibraryName;
    LoadAddresses(SQLiteLibraryHandle);
    end
  else
    begin
    If (LoadedLibrary<>LibraryName) then
      Raise EInoutError.CreateFmt(SErrAlreadyLoaded,[LoadedLibrary]);
    inc(RefCount);
    Result:=True;
    end;
end;

Procedure InitialiseSQLite;

begin
  InitialiseSQLite(DefaultLibrary);
end;


Procedure InitialiseSQLite(LibraryName : String);

begin
  If Not TryInitialiseSQLIte(LibraryName) then
    Raise EInOutError.CreateFmt(SErrLoadFailed,[LibraryName]);
end;

Procedure ReleaseSQLITE;

begin
  if RefCount>1 then
    Dec(RefCount)
  else if UnloadLibrary(SQLITELibraryHandle) then
    begin
    Dec(RefCount);
    SQLITELibraryHandle := NilHandle;
    LoadedLibrary:='';
    end;
end;

{$ENDIF}

end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
