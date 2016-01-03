{$IFDEF OGC_INTERFACE}
const
  DVD_IDENTIFY = $12;
  DVD_READ_DISCID = $70;
  DVD_LOW_READ = $71;
  DVD_WAITFORCOVERCLOSE = $79;
  DVD_READ_PHYSICAL = $80;
  DVD_READ_COPYRIGHT = $81;
  DVD_READ_DISCKEY = $82;
  DVD_GETCOVER = $88;
  DVD_RESET = $8A;
  DVD_OPEN_PARTITION = $8B;
  DVD_CLOSE_PARTITION = $8C;
  DVD_READ_UNENCRYPTED = $8D;
  DVD_REPORTKEY = $A4;
  DVD_READ = $D0;
  DVD_READ_CONFIG = $D1;
  DVD_READ_BCA = $DA;
  DVD_GET_ERROR = $E0;
  DVD_SET_MOTOR = $E3;
  DVD_READY = $1;
  DVD_INIT = $2;
  DVD_UNKNOWN = $4;
  DVD_NO_DISC = $8;
  DVD_IOS_ERROR = $10;
  DVD_D0 = $20;
  DVD_A8 = $40;
  DVD_COVER_DISC_INSERTED = $02;
  LIBDI_MAX_RETRIES = 16;

const
  DEVICE_TYPE_WII_DVD = (( ord('W') shl 24 ) or ( ord('D') shl 16 ) or ( ord('V') shl 8 ) or ord('D') );

type
  DI_DriveID = packed record
    rev : cuint16;
    dev_code : cuint16;
    rel_date : cuint32;
  end;
  PDI_DriveID = ^DI_DriveID;


  di_callback = function(status, error: uint32_t): cint; cdecl;
  read_func = function(par0: pointer; par1, par2: uint32_t): cint; cdecl;
  read_func_async = function(par0: pointer; par1, par2: uint32_t;
    par3: ipccallback): cint; cdecl;


var
  di_fd : cint; external;
  __io_wiidvd : DISC_INTERFACE; external;


function DI_Init: cint; cdecl; external;

procedure DI_LoadDVDX(load: cbool); cdecl; external;

procedure DI_UseCache(use: cbool); cdecl; external;

procedure DI_SetInitCallback(cb: di_callback); cdecl; external;

procedure DI_Mount; cdecl; external;

procedure DI_Close; cdecl; external;

function DI_GetStatus: cint; cdecl; external;

function DI_Identify(id: PDI_DriveID): cint; cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function DI_CheckDVDSupport(): cint; cdecl; external;
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
function DI_ReadDiscID(id: pcuint64): cint; cdecl; external;
function DI_GetError(error: Pcuint32): cint; cdecl; external;
function DI_GetCoverRegister(status: Pcuint32): cint; cdecl; external;

function DI_Reset: cint; cdecl; external;

function DI_StopMotor: cint; cdecl; external;

function DI_Eject: cint; cdecl; external;

function DI_KillDrive: cint; cdecl; external;

function DI_ReadDVD(buf: pointer; len, lba: cuint32): cint; cdecl; external;

function DI_ReadDVDAsync(buf: pointer; len, lba: cuint32; ipc_cb: ipccallback)
  : cint; cdecl; external;

function DI_Read(buf: pointer; size, offset: cuint32): cint; cdecl; external;

function DI_UnencryptedRead(buf: pointer; size, offset: cuint32): cint; cdecl; external;

function DI_ReadDVDConfig(value: Pcuint32; flag: cuint32): cint; cdecl; external;

function DI_ReadDVDCopyright(copyright: Pcuint32): cint; cdecl; external;

function DI_ReadDVDDiscKey(buf: pointer): cint; cdecl; external;

function DI_ReadDVDPhysical(buf: pointer): cint; cdecl; external;

function DI_Read_BCA(buf: pointer): cint; cdecl; external;

function DI_ReportKey(keytype: cint; lba: cuint32; buf: pointer): cint; cdecl; external;

function DI_OpenPartition(offset: cuint32): cint; cdecl; external;

function DI_ClosePartition: cint; cdecl; external;
{$ENDIF}

