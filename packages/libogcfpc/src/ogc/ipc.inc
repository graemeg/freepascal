
{$IFDEF OGC_INTERFACE}
const
  IPC_HEAP = - 1;
  IPC_OPEN_NONE = 0;
  IPC_OPEN_READ = 1;
  IPC_OPEN_WRITE = 2;
  IPC_OPEN_RW = ( IPC_OPEN_READ or IPC_OPEN_WRITE );
  IPC_MAXPATH_LEN = 64;
  IPC_OK = 0;
  IPC_EINVAL = - 4;
  IPC_ENOHEAP = - 5;
  IPC_ENOENT = - 6;
  IPC_EQUEUEFULL = - 8;
  IPC_ENOMEM = - 22;
type
  _ioctlv = record
    data : pointer;
    len : cuint32;
  end;
  ioctlv = _ioctlv;


procedure __IPC_Reinitialize; cdecl; external;


type
  ipccallback = function(result: cint32; usrdata: pointer): cint32; cdecl;


function iosCreateHeap(size: cint32): cint32; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
function iosDestroyHeap(hid: cint32): cint32; cdecl; external;

>>>>>>> graemeg/cpstrnew
=======
function iosDestroyHeap(hid: cint32): cint32; cdecl; external;

>>>>>>> graemeg/cpstrnew
=======
function iosDestroyHeap(hid: cint32): cint32; cdecl; external;

>>>>>>> graemeg/cpstrnew
=======
function iosDestroyHeap(hid: cint32): cint32; cdecl; external;

>>>>>>> origin/cpstrnew
function iosAlloc(hid, size: cint32): pointer; cdecl; external;

procedure iosFree(hid: cint32; ptr: pointer); cdecl; external;


function IPC_GetBufferLo: pointer; cdecl; external;

function IPC_GetBufferHi: pointer; cdecl; external;

procedure IPC_SetBufferLo(bufferlo: pointer); cdecl; external;

procedure IPC_SetBufferHi(bufferhi: pointer); cdecl; external;


function IOS_Open(filepath: pcchar; mode: cuint32): cint32; cdecl; external;

function IOS_OpenAsync(filepath: pcchar; mode: cuint32; ipc_cb: ipccallback;
  usrdata: pointer): cint32; cdecl; external;


function IOS_Close(fd: cint32): cint32; cdecl; external;

function IOS_CloseAsync(fd: cint32; ipc_cb: ipccallback; usrdata: pointer)
  : cint32; cdecl; external;


function IOS_Seek(fd, where, whence: cint32): cint32; cdecl; external;

function IOS_SeekAsync(fd, where, whence: cint32; ipc_cb: ipccallback;
  usrdata: pointer): cint32; cdecl; external;

function IOS_Read(fd: cint32; buf: pointer; len: cint32): cint32; cdecl; external;

function IOS_ReadAsync(fd: cint32; buf: pointer; len: cint32;
  ipc_cb: ipccallback; usrdata: pointer): cint32; cdecl; external;

function IOS_Write(fd: cint32; buf: pointer; len: cint32): cint32; cdecl; external;

function IOS_WriteAsync(fd: cint32; buf: pointer; len: cint32;
  ipc_cb: ipccallback; usrdata: pointer): cint32; cdecl; external;


function IOS_Ioctl(fd, ioctl: cint32; buffer_in: pointer; len_in: cint32;
  buffer_io: pointer; len_io: cint32): cint32; cdecl; external;

function IOS_IoctlAsync(fd, ioctl: cint32; buffer_in: pointer; len_in: cint32;
  buffer_io: pointer; len_io: cint32; ipc_cb: ipccallback; usrdata: pointer)
  : cint32; cdecl; external;

type
  Pioctlv = ^ioctlv;
function IOS_Ioctlv(fd, ioctl, cnt_in, cnt_io: cint32; argv: Pioctlv): cint32; cdecl; external;

function IOS_IoctlvAsync(fd, ioctl, cnt_in, cnt_io: cint32; argv: Pioctlv;
  ipc_cb: ipccallback; usrdata: pointer): cint32; cdecl; external;


function IOS_IoctlvFormat(hId, fd, ioctl: cint32; format: pcchar; par4: array of const)
  : cint32; cdecl; external;

function IOS_IoctlvFormatAsync(hId, fd, ioctl: cint32; usr_cb: ipccallback;
  usr_data: pointer; format: pcchar; par6: array of const): cint32; cdecl; external;


function IOS_IoctlvReboot(fd, ioctl, cnt_in, cnt_io: cint32; argv: Pioctlv)
  : cint32; cdecl; external;

function IOS_IoctlvRebootBackground(fd, ioctl, cnt_in, cnt_io: cint32;
  argv: Pioctlv): cint32; cdecl; external;
{$ENDIF}

