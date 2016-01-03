{$ifdef OGC_INTERFACE}

{$ifdef HW_RVL}


const
  USB_MAXPATH = IPC_MAXPATH_LEN;
  USB_OK = 0;
  USB_FAILED = 1;
  USB_CLASS_HID = $03;
  USB_SUBCLASS_BOOT = $01;
  USB_PROTOCOL_KEYBOARD = $01;
  USB_PROTOCOL_MOUSE = $02;
  USB_REPTYPE_INPUT = $01;
  USB_REPTYPE_OUTPUT = $02;
  USB_REPTYPE_FEATURE = $03;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  USB_REQTYPE_GET = $A1;
  USB_REQTYPE_SET = $21;
>>>>>>> graemeg/cpstrnew
=======
  USB_REQTYPE_GET = $A1;
  USB_REQTYPE_SET = $21;
>>>>>>> graemeg/cpstrnew
=======
  USB_REQTYPE_GET = $A1;
  USB_REQTYPE_SET = $21;
>>>>>>> graemeg/cpstrnew
=======
  USB_REQTYPE_GET = $A1;
  USB_REQTYPE_SET = $21;
>>>>>>> origin/cpstrnew
  USB_DT_DEVICE = $01;
  USB_DT_CONFIG = $02;
  USB_DT_STRING = $03;
  USB_DT_INTERFACE = $04;
  USB_DT_ENDPOINT = $05;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  USB_DT_DEVICE_QUALIFIER      = $06;
  USB_DT_OTHER_SPEED_CONFIG    = $07;
  USB_DT_INTERFACE_POWER       = $08;
  USB_DT_OTG                   = $09;
  USB_DT_DEBUG                 = $10;
  USB_DT_INTERFACE_ASSOCIATION = $11;  
  USB_DT_HID = $21;
  USB_DT_REPORT = $22;
  USB_DT_PHYSICAL					 = $23;
  USB_DT_CLASS_SPECIFIC_INTERFACE = $24;
  USB_DT_CLASS_SPECIFIC_ENDPOINT  = $25;
  USB_DT_HUB                      = $29;
=======
  USB_DT_HID = $21;
  USB_DT_REPORT = $22;
>>>>>>> graemeg/cpstrnew
=======
  USB_DT_HID = $21;
  USB_DT_REPORT = $22;
>>>>>>> graemeg/cpstrnew
=======
  USB_DT_HID = $21;
  USB_DT_REPORT = $22;
>>>>>>> graemeg/cpstrnew
=======
  USB_DT_HID = $21;
  USB_DT_REPORT = $22;
>>>>>>> origin/cpstrnew

  USB_REQ_GETSTATUS = $00;
  USB_REQ_CLEARFEATURE = $01;
  USB_REQ_SETFEATURE = $03;
  USB_REQ_SETADDRESS = $05;
  USB_REQ_GETDESCRIPTOR = $06;
  USB_REQ_SETDESCRIPTOR = $07;
  USB_REQ_GETCONFIG = $08;
  USB_REQ_SETCONFIG = $09;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  USB_REQ_GETINTERFACE = $0A;
  USB_REQ_SETINTERFACE = $0B;
  USB_REQ_SYNCFRAME = $0C;
  USB_REQ_GETREPORT = $01;
  USB_REQ_GETIDLE = $02;  
  USB_REQ_GETPROTOCOL = $03;
  USB_REQ_SETREPORT = $09;
  USB_REQ_SETIDLE = $0A;
  USB_REQ_SETPROTOCOL = $0B;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  USB_REQ_GETINTERFACE = $0a;
  USB_REQ_SETINTERFACE = $0b;
  USB_REQ_SYNCFRAME = $0c;
  USB_REQ_GETPROTOCOL = $03;
  USB_REQ_SETPROTOCOL = $0B;
  USB_REQ_GETREPORT = $01;
  USB_REQ_SETREPORT = $09;
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

  USB_DT_DEVICE_SIZE = 18;
  USB_DT_CONFIG_SIZE = 9;
  USB_DT_INTERFACE_SIZE = 9;
  USB_DT_ENDPOINT_SIZE = 7;
  USB_DT_ENDPOINT_AUDIO_SIZE = 9;  (* Audio extension  *)
  USB_DT_HID_SIZE = 9;
  USB_DT_HUB_NONVAR_SIZE = 7;

  USB_CTRLTYPE_DIR_HOST2DEVICE = (0 shl 7);
  USB_CTRLTYPE_DIR_DEVICE2HOST = (1 shl 7);
  USB_CTRLTYPE_TYPE_STANDARD = (0 shl 5);
  USB_CTRLTYPE_TYPE_CLASS = (1 shl 5);
  USB_CTRLTYPE_TYPE_VENDOR = (2 shl 5);
  USB_CTRLTYPE_TYPE_RESERVED = (3 shl 5);
  USB_CTRLTYPE_REC_DEVICE = 0;
  USB_CTRLTYPE_REC_INTERFACE = 1;
  USB_CTRLTYPE_REC_ENDPOINT = 2;
  USB_CTRLTYPE_REC_OTHER = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  
  USB_REQTYPE_INTERFACE_GET		= (USB_CTRLTYPE_DIR_DEVICE2HOST or USB_CTRLTYPE_TYPE_CLASS or USB_CTRLTYPE_REC_INTERFACE);
  USB_REQTYPE_INTERFACE_SET		= (USB_CTRLTYPE_DIR_HOST2DEVICE or USB_CTRLTYPE_TYPE_CLASS or USB_CTRLTYPE_REC_INTERFACE);
  USB_REQTYPE_ENDPOINT_GET		= (USB_CTRLTYPE_DIR_DEVICE2HOST or USB_CTRLTYPE_TYPE_CLASS or USB_CTRLTYPE_REC_ENDPOINT);
  USB_REQTYPE_ENDPOINT_SET		= (USB_CTRLTYPE_DIR_HOST2DEVICE or USB_CTRLTYPE_TYPE_CLASS or USB_CTRLTYPE_REC_ENDPOINT);
  
  
  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  USB_FEATURE_ENDPOINT_HALT = 0;
  USB_ENDPOINT_INTERRUPT = $03;
  USB_ENDPOINT_IN = $80;
  USB_ENDPOINT_OUT = $00;
  USB_OH0_DEVICE_ID = $00000000;  // for completion

type
  _usbendpointdesc = packed record
    bLength : cuint8;
    bDescriptorType : cuint8;
    bEndpointAddress : cuint8;
    bmAttributes : cuint8;
    wMaxPacketSize : cuint16;
    bInterval : cuint8;
  end;
  usb_endpointdesc = _usbendpointdesc;
  pusb_endpointdesc = ^usb_endpointdesc;

  P_usbendpointdesc = ^_usbendpointdesc;
  _usbinterfacedesc = packed record
    bLength : cuint8;
    bDescriptorType : cuint8;
    bInterfaceNumber : cuint8;
    bAlternateSetting : cuint8;
    bNumEndpoints : cuint8;
    bInterfaceClass : cuint8;
    bInterfaceSubClass : cuint8;
    bInterfaceProtocol : cuint8;
    iInterface : cuint8;
    extra : pcuint8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    extra_size : cuint16;
=======
    extra_size : cuint8;
>>>>>>> graemeg/cpstrnew
=======
    extra_size : cuint8;
>>>>>>> graemeg/cpstrnew
=======
    extra_size : cuint8;
>>>>>>> graemeg/cpstrnew
=======
    extra_size : cuint8;
>>>>>>> origin/cpstrnew
    endpoints : P_usbendpointdesc;
  end;
  usb_interfacedesc = _usbinterfacedesc;

  P_usbinterfacedesc = ^_usbinterfacedesc;
  _usbconfdesc = packed record
    bLength : cuint8;
    bDescriptorType : cuint8;
    wTotalLength : cuint16;
    bNumInterfaces : cuint8;
    bConfigurationValue : cuint8;
    iConfiguration : cuint8;
    bmAttributes : cuint8;
    bMaxPower : cuint8;
    interfaces : P_usbinterfacedesc;
  end;
  usb_configurationdesc = _usbconfdesc;

  P_usbconfdesc = ^_usbconfdesc;
  _usbdevdesc = packed record
    bLength : cuint8;
    bDescriptorType : cuint8;
    bcdUSB : cuint16;
    bDeviceClass : cuint8;
    bDeviceSubClass : cuint8;
    bDeviceProtocol : cuint8;
    bMaxPacketSize0 : cuint8;
    idVendor : cuint16;
    idProduct : cuint16;
    bcdDevice : cuint16;
    iManufacturer : cuint8;
    iProduct : cuint8;
    iSerialNumber : cuint8;
    bNumConfigurations : cuint8;
    configurations : P_usbconfdesc;
  end;
  usb_devdesc = _usbdevdesc;
  pusb_devdesc = ^_usbdevdesc;


  _usbhiddesc = packed record
    bLength: cuint8;
    bDescriptorType: cuint8;
  	bcdHID: cuint16;
	  bCountryCode: cuint8;
	  bNumDescriptors: cuint8;
    //	struct {
    //		u8 bDescriptorType;
    //		u16 wDescriptorLength;
    //	} descr[1];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    descr: packed record
=======
    descr: record
>>>>>>> graemeg/cpstrnew
=======
    descr: record
>>>>>>> graemeg/cpstrnew
=======
    descr: record
>>>>>>> graemeg/cpstrnew
=======
    descr: record
>>>>>>> origin/cpstrnew
      bDescriptorType: cuint8;
      wDescriptorLength: cuint16;
    end;
  end;
  usb_hiddesc = _usbhiddesc;
  Pusb_hiddesc = ^usb_hiddesc;

type
  _usb_device_entry = packed record
    device_id : cint32;
    vid : cuint16;
    pid : cuint16;
    token : cuint32;
  end;
  usb_device_entry = _usb_device_entry;
  Pusb_device_entry = ^usb_device_entry;



type
  usbcallback = function(result: cint32; usrdata: pointer): cint32; cdecl;


function USB_Initialize: cint32; cdecl; external;

function USB_Deinitialize: cint32; cdecl; external;


function USB_OpenDevice(device_id: cint32; vid, pid: cuint16; fd: pcint32): cint32; cdecl; external;

function USB_CloseDevice(fd: pcint32): cint32; cdecl; external;

function USB_CloseDeviceAsync(fd: pcint32; cb: usbcallback; usrdata: pointer): cint32; cdecl; external;


function USB_GetDescriptors(fd: cint32; udd: Pusb_devdesc): cint32; cdecl; external;

procedure USB_FreeDescriptors(udd: Pusb_devdesc); cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function USB_GetGenericDescriptor(fd: cint32; _type, _index, _interface: cuint8; data: pointer; size: cuint32): cint32; cdecl; external;
function USB_GetHIDDescriptor(fd: cint32; _interface: cuint8; uhd: pusb_hiddesc; size: cuint32): cint32; cdecl; external;
=======
function USB_GetHIDDescriptor(fd: cint32; uhd: Pusb_hiddesc): cint32; cdecl; external;

>>>>>>> graemeg/cpstrnew
=======
function USB_GetHIDDescriptor(fd: cint32; uhd: Pusb_hiddesc): cint32; cdecl; external;

>>>>>>> graemeg/cpstrnew
=======
function USB_GetHIDDescriptor(fd: cint32; uhd: Pusb_hiddesc): cint32; cdecl; external;

>>>>>>> graemeg/cpstrnew
=======
function USB_GetHIDDescriptor(fd: cint32; uhd: Pusb_hiddesc): cint32; cdecl; external;

>>>>>>> origin/cpstrnew

function USB_GetDeviceDescription(fd: cint32; devdesc: Pusb_devdesc): cint32; cdecl; external;

function USB_DeviceRemovalNotifyAsync(fd: cint32; cb: usbcallback; userdata: pointer): cint32; cdecl; external;

function USB_DeviceChangeNotifyAsync(interface_class: cuint8; cb: usbcallback; userdata: pointer): cint32; cdecl; external;


function USB_SuspendDevice(fd: cint32): cint32; cdecl; external;

function USB_ResumeDevice(fd: cint32): cint32; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function USB_ReadIsoMsg(fd: cint32; bEndpoint, bPackets: cuint8; rpPacketSizes: pcuint16; rpData: pointer): cint32; cdecl; external;
function USB_ReadIsoMsgAsync(fd: cint32; bEndpoint, bPackets: cuint8; rpPacketSizes: pcuint16; rpData: pointer; cb: usbcallback; userdata: pointer): cint32; cdecl; external;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function USB_ReadIntrMsg(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer): cint32; cdecl; external;

function USB_ReadIntrMsgAsync(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer; cb: usbcallback; usrdata: pointer): cint32; cdecl; external;


function USB_ReadBlkMsg(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer): cint32; cdecl; external;

function USB_ReadBlkMsgAsync(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer; cb: usbcallback; usrdata: pointer): cint32; cdecl; external;


function USB_ReadCtrlMsg(fd: cint32; bmRequestType, bmRequest: cuint8;
  wValue, wIndex, wLength: cuint16; rpData: pointer): cint32; cdecl; external;

function USB_ReadCtrlMsgAsync(fd: cint32; bmRequestType, bmRequest: cuint8;
  wValue, wIndex, wLength: cuint16; rpData: pointer; cb: usbcallback;
  usrdata: pointer): cint32; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function USB_WriteIsoMsg(fd: cint32; bEndpoint, bPackets: cuint8; rpPacketSizes: pcuint16; rpData: pointer): cint32; cdecl; external;
function USB_WriteIsoMsgAsync(fd: cint32; bEndpoint, bPackets: cuint8; rpPacketSizes: pcuint16; rpData: pointer; cb: usbcallback; userdata: pointer): cint32; cdecl; external;


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function USB_WriteIntrMsg(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer): cint32; cdecl; external;

function USB_WriteIntrMsgAsync(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer; cb: usbcallback; usrdata: pointer): cint32; cdecl; external;


function USB_WriteBlkMsg(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer): cint32; cdecl; external;

function USB_WriteBlkMsgAsync(fd: cint32; bEndpoint: cuint8; wLength: cuint16;
  rpData: pointer; cb: usbcallback; usrdata: pointer): cint32; cdecl; external;


function USB_WriteCtrlMsg(fd: cint32; bmRequestType, bmRequest: cuint8;
  wValue, wIndex, wLength: cuint16; rpData: pointer): cint32; cdecl; external;

function USB_WriteCtrlMsgAsync(fd: cint32; bmRequestType, bmRequest: cuint8;
  wValue, wIndex, wLength: cuint16; rpData: pointer; cb: usbcallback;
  usrdata: pointer): cint32; cdecl; external;


function USB_GetConfiguration(fd: cint32; configuration: pcuint8): cint32; cdecl; external;

function USB_SetConfiguration(fd: cint32; configuration: cuint8): cint32; cdecl; external;

function USB_SetAlternativeInterface(fd: cint32;
  interface_, alternateSetting: cuint8): cint32; cdecl; external;

function USB_ClearHalt(fd: cint32; endpointAddress: cuint8): cint32; cdecl; external;

function USB_GetDeviceList(descr_buffer: Pusb_device_entry;
  num_descr, interface_class: cuint8; cnt_descr: pcuint8): cint32; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function USB_GetAsciiString(fd: cint32; bIndex: cuint8; wLangID, wLength: cuint16; rpData: pointer): cint32; cdecl; external;
=======
function USB_GetAsciiString(fd: cint32; wIndex, wLangID, wLength: cuint16; rpData: pointer): cint32; cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
function USB_GetAsciiString(fd: cint32; wIndex, wLangID, wLength: cuint16; rpData: pointer): cint32; cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
function USB_GetAsciiString(fd: cint32; wIndex, wLangID, wLength: cuint16; rpData: pointer): cint32; cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
function USB_GetAsciiString(fd: cint32; wIndex, wLangID, wLength: cuint16; rpData: pointer): cint32; cdecl; external;
>>>>>>> origin/cpstrnew


{$endif HW_RVL}

{$endif OGC_INTERFACE}
