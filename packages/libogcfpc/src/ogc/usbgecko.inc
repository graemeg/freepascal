
{$IFDEF OGC_INTERFACE}
procedure usb_flush(chn: cint32); cdecl; external;

function usb_isgeckoalive(chn: cint32): cint; cdecl; external;

function usb_recvbuffer(chn: cint32; buffer: pointer; size: cint): cint; cdecl; external;

function usb_sendbuffer(chn: cint32; buffer: pointer; size: cint): cint; cdecl; external;

function usb_recvbuffer_safe(chn: cint32; buffer: pointer; size: cint): cint; cdecl; external;

function usb_sendbuffer_safe(chn: cint32; buffer: pointer; size: cint): cint; cdecl; external;

function usb_recvbuffer_ex(chn: cint32; buffer: pointer; size, retries: cint)
  : cint; cdecl; external;

function usb_sendbuffer_ex(chn: cint32; buffer: pointer; size, retries: cint)
  : cint; cdecl; external;

function usb_recvbuffer_safe_ex(chn: cint32; buffer: pointer;
  size, retries: cint): cint; cdecl; external;

function usb_sendbuffer_safe_ex(chn: cint32; buffer: pointer;
  size, retries: cint): cint; cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  
  
function usb_flashread(chn: cint32; offset: cuint32; buffer: pointer; length: cint): cint; cdecl; external;
function usb_flashwrite(chn: cint32; offset: cuint32; const buffer: pointer; length: cint): cint; cdecl; external;
function usb_flashverify(chn: cint32): cint; cdecl; external;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ENDIF OGC_INTERFACE}

