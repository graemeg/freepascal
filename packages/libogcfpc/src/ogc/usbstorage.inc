{$ifdef OGC_INTERFACE}

{$ifdef HW_RVL}
const
  USBSTORAGE_OK           = 0;
  USBSTORAGE_ENOINTERFACE = -10000;
  USBSTORAGE_ESENSE       = -10001;
  USBSTORAGE_ESHORTWRITE  = -10002;
  USBSTORAGE_ESHORTREAD   = -10003;
  USBSTORAGE_ESIGNATURE   = -10004;
  USBSTORAGE_ETAG         = -10005;
  USBSTORAGE_ESTATUS      = -10006;
  USBSTORAGE_EDATARESIDUE = -10007;
  USBSTORAGE_ETIMEDOUT    = -10008;
  USBSTORAGE_EINIT        = -10009;
  USBSTORAGE_PROCESSING   = -10010;

type
  usbstorage_handle = record
    configuration: cuint8;
    interface_: cuint32;
    altInterface: cuint32;
    bInterfaceSubClass: cuint8;

    ep_in: cuint8;
    ep_out: cuint8;
    
    max_lun: cuint8;
    sector_size: pcuint32;
    
    usb_fd: cint32;
    
    lock: mutex_t;
    alarm: syswd_t;
    retval: cint32;
    
    tag: cuint32;
    suspended: cuint8;
    
    buffer: pcuint8;
  end;
  pusbstorage_handle = ^usbstorage_handle;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
  B_RAW_DEVICE_DATA_IN = $01;
  B_RAW_DEVICE_COMMAND = 0;

type 
  raw_device_command = record
    command: array [0..15] of cuint8;
    command_length: cuint8;
    flags: cuint8;
    scsi_status: cuint8;
    data: pointer;
    data_length: cint;
  end;
  praw_device_command = ^raw_device_command;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function USBStorage_Initialize(): cint32; cdecl; external;

function USBStorage_Open(dev: pusbstorage_handle; device_id: cint32; vid, pid: cuint16): cint32; cdecl; external;
function USBStorage_Close(dev: pusbstorage_handle): cint32; cdecl; external;
function USBStorage_Reset(dev: pusbstorage_handle): cint32; cdecl; external;

function USBStorage_GetMaxLUN(dev: pusbstorage_handle): cint32; cdecl; external;
function USBStorage_MountLUN(dev: pusbstorage_handle; lun: cuint8): cint32; cdecl; external;
function USBStorage_Suspend(dev: pusbstorage_handle): cint32; cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function USBStorage_IsDVD(): cint32; cdecl; external;
function USBStorage_ioctl(request: cint; args: array of const): cint; cdecl; external;
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
function USBStorage_ReadCapacity(dev: pusbstorage_handle; lun: cuint8; sector_size, n_sectors: pcuint32): cint32; cdecl; external;
function USBStorage_Read(dev: pusbstorage_handle; lun: cuint8; sector: cuint32; n_sectors: cuint16; buffer: pcuint8): cint32; cdecl; external;
function USBStorage_Write(dev: pusbstorage_handle; lun: cuint8; sector: cuint32; n_sectors: cuint16; const buffer: pcuint8): cint32; cdecl; external;
function USBStorage_StartStop(dev: pusbstorage_handle; lun, lo_ej, start, imm: cuint8): cint32; cdecl; external;

function DEVICE_TYPE_WII_USB: cint32; inline;

var
  __io_usbstorage: DISC_INTERFACE; cvar; external;

{$endif HW_RVL}

{$endif OGC_INTERFACE}

{$ifdef OGC_IMPLEMENTATION}
{$ifdef HW_RVL}
function DEVICE_TYPE_WII_USB: cint32; inline;
begin
  DEVICE_TYPE_WII_USB := ((ord('W') shl 24) or (ord('U') shl 16) or (ord('S') shl 8) or ord('B'));
end;
{$endif HW_RVL}
{$endif OGC_IMPLEMENTATION}
