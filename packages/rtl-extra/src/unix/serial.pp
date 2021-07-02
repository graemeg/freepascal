{ Unit for handling the serial interfaces for Linux and similar Unices.
  (c) 2000 Sebastian Guenther, sg@freepascal.org; modified MarkMLl 2012.
}

unit Serial;

{$MODE objfpc}
{$H+}
{$PACKRECORDS C}

interface

uses BaseUnix,termio,unix;

type

  TSerialHandle = LongInt;

  TParityType = (NoneParity, OddParity, EvenParity);

  TSerialFlags = set of (RtsCtsFlowControl);

  TSerialState = record
    LineState: LongWord;
    tios: termios;
  end;


{ Open the serial device with the given device name, for example:
    /dev/ttyS0, /dev/ttyS1... for normal serial ports
    /dev/ttyI0, /dev/ttyI1... for ISDN emulated serial ports
    other device names are possible; refer to your OS documentation.
  Returns "0" if device could not be found }
function SerOpen(const DeviceName: String): TSerialHandle;

{ Closes a serial device previously opened with SerOpen. }
procedure SerClose(Handle: TSerialHandle);

{ Flushes the data queues of the given serial device. DO NOT USE THIS:
  use either SerSync (non-blocking) or SerDrain (blocking). }
procedure SerFlush(Handle: TSerialHandle); deprecated;

{ Suggest to the kernel that buffered output data should be sent. This
  is unlikely to have a useful effect except possibly in the case of
  buggy ports that lose Tx interrupts, and is implemented as a preferred
  alternative to the deprecated SerFlush procedure. }
procedure SerSync(Handle: TSerialHandle);

{ Wait until all buffered output has been transmitted. It is the caller's
  responsibility to ensure that this won't block permanently due to an
  inappropriate handshake state. }
procedure SerDrain(Handle: TSerialHandle);

{ Discard all pending input. }
procedure SerFlushInput(Handle: TSerialHandle);

{ Discard all unsent output. }
procedure SerFlushOutput(Handle: TSerialHandle);

{ Reads a maximum of "Count" bytes of data into the specified buffer.
  Result: Number of bytes read. }
function SerRead(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;

{ Tries to write "Count" bytes from "Buffer".
  Result: Number of bytes written. }
function SerWrite(Handle: TSerialHandle; Const Buffer; Count: LongInt): LongInt;

procedure SerSetParams(Handle: TSerialHandle; BitsPerSec: LongInt;
  ByteSize: Integer; Parity: TParityType; StopBits: Integer;
  Flags: TSerialFlags);

{ Saves and restores the state of the serial device. }
function SerSaveState(Handle: TSerialHandle): TSerialState;
procedure SerRestoreState(Handle: TSerialHandle; State: TSerialState);

{ Getting and setting the line states directly. }
procedure SerSetDTR(Handle: TSerialHandle; State: Boolean);
procedure SerSetRTS(Handle: TSerialHandle; State: Boolean);
function SerGetCTS(Handle: TSerialHandle): Boolean;
function SerGetDSR(Handle: TSerialHandle): Boolean;
function SerGetCD(Handle: TSerialHandle): Boolean;
function SerGetRI(Handle: TSerialHandle): Boolean;

{ Set a line break state. If the requested time is greater than zero this is in
  mSec, in the case of unix this is likely to be rounded up to a few hundred
  mSec and to increase by a comparable increment; on unix if the time is less
  than or equal to zero its absolute value will be passed directly to the
  operating system with implementation-specific effect. If the third parameter
  is omitted or true there will be an implicit call of SerDrain() before and
  after the break.

  NOTE THAT on Linux, the only reliable mSec parameter is zero which results in
  a break of around 250 mSec. Might be completely ineffective on Solaris.
 }
procedure SerBreak(Handle: TSerialHandle; mSec: LongInt=0; sync: boolean= true); 

type    TSerialIdle= procedure(h: TSerialHandle);

{ Set this to a shim around Application.ProcessMessages if calling SerReadTimeout(),
  SerBreak() etc. from the main thread so that it doesn't lock up a Lazarus app. }
var     SerialIdle: TSerialIdle= nil;

{ This is similar to SerRead() but adds a mSec timeout. Note that this variant
  returns as soon as a single byte is available, or as dictated by the timeout. }
function SerReadTimeout(Handle: TSerialHandle; var Buffer; mSec: LongInt): LongInt;

{ This is similar to SerRead() but adds a mSec timeout. Note that this variant
  attempts to accumulate as many bytes as are available, but does not exceed
  the timeout. Set up a SerIdle callback if using this in a main thread in a
  Lazarus app. }
function SerReadTimeout(Handle: TSerialHandle; var Buffer: array of byte; count, mSec: LongInt): LongInt;


{ ************************************************************************** }

implementation


function SerOpen(const DeviceName: String): TSerialHandle;
begin
  Result := fpopen(DeviceName, O_RDWR or O_NOCTTY);
end;

procedure SerClose(Handle: TSerialHandle);
begin
  fpClose(Handle);
end;

procedure SerFlush(Handle: TSerialHandle); deprecated;
begin
  fpfsync(Handle);
end;

procedure SerSync(Handle: TSerialHandle);
begin
  fpfsync(Handle)
end;

procedure SerDrain(Handle: TSerialHandle);
begin
  tcdrain(Handle)
end;

procedure SerFlushInput(Handle: TSerialHandle);
begin
  tcflush(Handle, TCIFLUSH)
end;

procedure SerFlushOutput(Handle: TSerialHandle);
begin
  tcflush(Handle, TCOFLUSH)
end;

function SerRead(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;
begin
  Result := fpRead(Handle, Buffer, Count);
end;

function SerWrite(Handle: TSerialHandle; Const Buffer; Count: LongInt): LongInt;
begin
  Result := fpWrite(Handle, Buffer, Count);
end;

procedure SerSetParams(Handle: TSerialHandle; BitsPerSec: LongInt;
  ByteSize: Integer; Parity: TParityType; StopBits: Integer;
  Flags: TSerialFlags);
var
  tios: termios;
begin
  FillChar(tios, SizeOf(tios), #0);

  case BitsPerSec of
    50: tios.c_cflag := B50;
    75: tios.c_cflag := B75;
    110: tios.c_cflag := B110;
    134: tios.c_cflag := B134;
    150: tios.c_cflag := B150;
    200: tios.c_cflag := B200;
    300: tios.c_cflag := B300;
    600: tios.c_cflag := B600;
    1200: tios.c_cflag := B1200;
    1800: tios.c_cflag := B1800;
    2400: tios.c_cflag := B2400;
    4800: tios.c_cflag := B4800;
    19200: tios.c_cflag := B19200;
    38400: tios.c_cflag := B38400;
    57600: tios.c_cflag := B57600;
    115200: tios.c_cflag := B115200;
    230400: tios.c_cflag := B230400;
{$ifndef BSD}
    460800: tios.c_cflag := B460800;
{$endif}
{$if declared(B500000)}
    500000: tios.c_cflag := B500000;
{$endif declared(B500000)}
{$if declared(B576000)}
    576000: tios.c_cflag := B576000;
{$endif declared(B576000)}
{$if declared(B921600)}
    921600: tios.c_cflag := B921600;
{$endif declared(B921600)}
{$if declared(B1000000)}
    1000000: tios.c_cflag := B1000000;
{$endif declared(B1000000)}
{$if declared(B1152000)}
    1152000: tios.c_cflag := B1152000;
{$endif declared(B1152000)}
{$if declared(B1500000)}
    1500000: tios.c_cflag := B1500000;
{$endif declared(B1500000)}
{$if declared(B2000000)}
    2000000: tios.c_cflag := B2000000;
{$endif declared(B2000000)}
{$if declared(B2500000)}
    2500000: tios.c_cflag := B2500000;
{$endif declared(B2500000)}
{$if declared(B3000000)}
    3000000: tios.c_cflag := B3000000;
{$endif declared(B3000000)}
{$if declared(B3500000)}
    3500000: tios.c_cflag := B3500000;
{$endif declared(B3500000)}
{$if declared(B4000000)}
    4000000: tios.c_cflag := B4000000;
{$endif declared(B4000000)}
    else tios.c_cflag := B9600;
  end;
{$ifndef SOLARIS}
  tios.c_ispeed := tios.c_cflag;
  tios.c_ospeed := tios.c_ispeed;
{$endif}

  tios.c_cflag := tios.c_cflag or CREAD or CLOCAL;

  case ByteSize of
    5: tios.c_cflag := tios.c_cflag or CS5;
    6: tios.c_cflag := tios.c_cflag or CS6;
    7: tios.c_cflag := tios.c_cflag or CS7;
    else tios.c_cflag := tios.c_cflag or CS8;
  end;

  case Parity of
    OddParity: tios.c_cflag := tios.c_cflag or PARENB or PARODD;
    EvenParity: tios.c_cflag := tios.c_cflag or PARENB;
  end;

  if StopBits = 2 then
    tios.c_cflag := tios.c_cflag or CSTOPB;

  if RtsCtsFlowControl in Flags then
    tios.c_cflag := tios.c_cflag or CRTSCTS;

  tcflush(Handle, TCIOFLUSH);
  tcsetattr(Handle, TCSANOW, tios)
end;

function SerSaveState(Handle: TSerialHandle): TSerialState;
begin
  fpioctl(Handle, TIOCMGET, @Result.LineState);
//  fpioctl(Handle, TCGETS, @Result.tios);
  TcGetAttr(handle,result.tios);

end;

procedure SerRestoreState(Handle: TSerialHandle; State: TSerialState);
begin
//  fpioctl(Handle, TCSETS, @State.tios);
    TCSetAttr(handle,TCSANOW,State.tios);
    fpioctl(Handle, TIOCMSET, @State.LineState);
end;

procedure SerSetDTR(Handle: TSerialHandle; State: Boolean);
const
  DTR: Cardinal = TIOCM_DTR;
begin
  if State then
    fpioctl(Handle, TIOCMBIS, @DTR)
  else
    fpioctl(Handle, TIOCMBIC, @DTR);
end;

procedure SerSetRTS(Handle: TSerialHandle; State: Boolean);
const
  RTS: Cardinal = TIOCM_RTS;
begin
  if State then
    fpioctl(Handle, TIOCMBIS, @RTS)
  else
    fpioctl(Handle, TIOCMBIC, @RTS);
end;

function SerGetCTS(Handle: TSerialHandle): Boolean;
var
  Flags: Cardinal;
begin
  fpioctl(Handle, TIOCMGET, @Flags);
  Result := (Flags and TIOCM_CTS) <> 0;
end;

function SerGetDSR(Handle: TSerialHandle): Boolean;
var
  Flags: Cardinal;
begin
  fpioctl(Handle, TIOCMGET, @Flags);
  Result := (Flags and TIOCM_DSR) <> 0;
end;

function SerGetCD(Handle: TSerialHandle): Boolean;
var
  Flags: Cardinal;
begin
  fpioctl(Handle, TIOCMGET, @Flags);
  Result := (Flags and TIOCM_CD) <> 0
end;

function SerGetRI(Handle: TSerialHandle): Boolean;
var
  Flags: Cardinal;
begin
  fpioctl(Handle, TIOCMGET, @Flags);
  Result := (Flags and TIOCM_RI) <> 0;
end;

procedure SerBreak(Handle: TSerialHandle; mSec: LongInt= 0; sync: boolean= true);
begin
  if sync then
    tcdrain(Handle);
  if mSec <= 0 then
    tcsendbreak(Handle, Abs(mSec))
  else
    tcsendbreak(Handle, Trunc(mSec / 250));
  if sync then
    tcdrain(Handle)
end; 

function SerReadTimeout(Handle: TSerialHandle; var Buffer; mSec: LongInt): LongInt;

VAR     readSet: TFDSet;
        selectTimeout: TTimeVal;

begin
  fpFD_ZERO(readSet);
  fpFD_SET(Handle, readSet);
  selectTimeout.tv_sec := mSec div 1000;
  selectTimeout.tv_usec := (mSec mod 1000) * 1000;
  result := 0;
  if fpSelect(Handle + 1, @readSet, nil, nil, @selectTimeout) > 0 then
    result := fpRead(Handle, Buffer, 1)
end { SerReadTimeout } ;

{$ifdef LINUX}
  {$define SELECT_UPDATES_TIMEOUT}
{$endif}

{$ifdef SELECT_UPDATES_TIMEOUT}

function SerReadTimeout(Handle: TSerialHandle; var Buffer: array of byte; count, mSec: LongInt): LongInt;

VAR     readSet: TFDSet;
        selectTimeout: TTimeVal;

begin
  fpFD_ZERO(readSet);
  fpFD_SET(Handle, readSet);
  selectTimeout.tv_sec := mSec div 1000;
  selectTimeout.tv_usec := (mSec mod 1000) * 1000;
  result := 0;

// Note: this variant of fpSelect() is a thin wrapper around the kernel's syscall.
// In the case of Linux the syscall DOES update the timeout parameter.

  while fpSelect(Handle + 1, @readSet, nil, nil, @selectTimeout) > 0 do begin
    Inc(result,fpRead(Handle, Buffer[result], count - result));
    if result >= count then
      break;
    if Assigned(SerialIdle) then
      SerialIdle(Handle)
  end
end { SerReadTimeout } ;

{$else}

function SerReadTimeout(Handle: TSerialHandle; var Buffer: array of byte; count, mSec: LongInt): LongInt;

VAR     readSet: TFDSet;
        selectTimeout: TTimeVal;
        uSecOnEntry, uSecElapsed: QWord;

  function now64uSec: QWord;

  var   tv: timeval;

  begin
    fpgettimeofday(@tv, nil);
    result := tv.tv_sec * 1000000 + tv.tv_usec
  end { now64uSec } ;

begin
  fpFD_ZERO(readSet);
  fpFD_SET(Handle, readSet);
  selectTimeout.tv_sec := mSec div 1000;
  selectTimeout.tv_usec := (mSec mod 1000) * 1000;
  result := 0;
  uSecOnEntry := now64uSec;

// Note: this variant of fpSelect() is a thin wrapper around the kernel's syscall.
// In the case of Solaris the syscall DOES NOT update the timeout parameter.

  while fpSelect(Handle + 1, @readSet, nil, nil, @selectTimeout) > 0 do begin
    Inc(result,fpRead(Handle, Buffer[result], count - result));
    uSecElapsed := now64uSec - uSecOnEntry;
    if (result >= count) or (uSecElapsed >= mSec * 1000) then
      break;
    selectTimeout.tv_sec := (mSec * 1000 - uSecElapsed) div 1000000;
    selectTimeout.tv_usec := (mSec * 1000 - uSecElapsed) mod 1000000;
    if Assigned(SerialIdle) then
      SerialIdle(Handle)
  end
end { SerReadTimeout } ;

{$endif}


end.
