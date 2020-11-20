{ Unit for handling the serial interfaces for Linux and similar Unices.
  (c) 2000 Sebastian Guenther, sg@freepascal.org; modified MarkMLl 2012.
  Windows variant written with reference to Dejan Crnila's TComPort v1.01.
}

unit Serial;

{$MODE objfpc}
{$H+}

interface

uses Windows;

type

  TSerialHandle = THandle;

  TParityType = (NoneParity, OddParity, EvenParity);

  TSerialFlags = set of (RtsCtsFlowControl);

  TSerialState = TDCB;


{ Open the serial device with the given device name, for example:
    \COM1, \COM2 (strictly, \\.\COM1...) for normal serial ports.
    ISDN devices, serial port redirectors/virtualisers etc. normally
    implement names of this form, but refer to your OS documentation.
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
procedure SerBreak(Handle: TSerialHandle; mSec: LongInt= 250; sync: boolean= true);

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


function SerSetParamsPrivate(Handle: TSerialHandle; BitsPerSec: LongInt;
  ByteSize: Integer; Parity: TParityType; StopBits: Integer;
  Flags: TSerialFlags): boolean;

const   bufSize= 2048;

        dcb_Binary           = $00000001;
        dcb_Parity           = $00000002;
        dcb_OutxCtsFlow      = $00000004;
        dcb_OutxDsrFlow      = $00000008;
        dcb_DtrControl       = $00000030;
        dcb_DsrSensivity     = $00000040;
        dcb_TXContinueOnXOff = $00000080;
        dcb_OutX             = $00000100;
        dcb_InX              = $00000200;
        dcb_ErrorChar        = $00000400;
        dcb_Null             = $00000800;
        dcb_RtsControl       = $00003000;
        dcb_AbortOnError     = $00004000;

var     DCB: TDCB;
        Timeouts: TCommTimeouts;

begin
  result := true;
  FillChar(DCB, SizeOf(DCB), 0);
  DCB.DCBlength := SizeOf(DCB);
  DCB.XonChar := #17;
  DCB.XoffChar := #19;
  DCB.XonLim := bufSize div 4;
  DCB.XoffLim := 1;
  DCB.Flags := DCB.Flags or dcb_Binary;
  DCB.BaudRate := BitsPerSec;
  DCB.ByteSize := ByteSize;
  case Parity of
    OddParity:  DCB.Parity := Windows.ODDPARITY;
    EvenParity: DCB.Parity := Windows.EVENPARITY
  else
    DCB.Parity := Windows.NOPARITY
  end;
  if StopBits > 1 then
    DCB.StopBits := TWOSTOPBITS
  else
    DCB.StopBits := ONESTOPBIT;
  if RtsCtsFlowControl in Flags then
    DCB.Flags := DCB.Flags or dcb_OutxCtsFlow or
                (dcb_RtsControl and (RTS_CONTROL_HANDSHAKE shl 12));
  if not SetCommState(Handle, DCB) then
    result := false;
  if GetCommTimeouts(Handle, Timeouts) then begin
    Timeouts.ReadIntervalTimeout := MAXDWORD;
    Timeouts.ReadTotalTimeoutMultiplier := 0;
    Timeouts.ReadTotalTimeoutConstant := 0;
    Timeouts.WriteTotalTimeoutMultiplier := 0;
    Timeouts.WriteTotalTimeoutConstant := 30000;
    if not SetCommTimeouts(Handle, Timeouts) then
      result := false
  end else
    result := false;
  if not SetupComm(Handle, bufSize, bufSize) then
    result := false
end { SerSetParamsPrivate } ;


function SerOpen(const DeviceName: String): TSerialHandle;

var     securityAttributes: TSecurityAttributes;

begin
  securityAttributes.nLength := SizeOf(TSecurityAttributes);
  securityAttributes.lpSecurityDescriptor := nil;
  securityAttributes.bInheritHandle := true;
  result := CreateFile(
    PChar(DeviceName),
    GENERIC_READ or GENERIC_WRITE,
    0,
    @securityAttributes,
    OPEN_EXISTING,
    0,
    0);
  if result = INVALID_HANDLE_VALUE then
    result := 0
  else

(* Don't trust Windows's initial state. If the internal variant (returning a    *)
(* result) of SerSetParams() fails it indicates that part of the comms API is   *)
(* unavailable, assume that this is fatal because it will probably mess up      *)
(* things like read timeouts.                                                   *)

    if not SerSetParamsPrivate(result, 9600, 8, NoneParity, 1, []) then begin
      CloseHandle(result);
      result := 0
    end
end { SerOpen } ;


procedure SerClose(Handle: TSerialHandle);

begin
  CloseHandle(Handle);
end { SerClose } ;


procedure SerFlush(Handle: TSerialHandle); deprecated;

begin
  FlushFileBuffers(Handle);
end { SerFlush } ;


procedure SerSync(Handle: TSerialHandle);

begin
  FlushFileBuffers(Handle)
end { SerSync } ;


procedure SerDrain(Handle: TSerialHandle);

var     errors: DWORD;
        comStat: TComStat;

begin
  FlushFileBuffers(Handle);
  repeat
    if not ClearCommError(Handle, errors, @comStat) then
      break;
    if (ComStat.cbOutQue > 0) and Assigned(SerialIdle) then
      SerialIdle(Handle)
  until ComStat.cbOutQue = 0
end { SerDrain } ;


procedure SerFlushInput(Handle: TSerialHandle);

begin
  PurgeComm(Handle, PURGE_RXABORT + PURGE_RXCLEAR)
end { SerFlushInput } ;


procedure SerFlushOutput(Handle: TSerialHandle);

begin
  PurgeComm(Handle, PURGE_TXABORT + PURGE_TXCLEAR)
end { SerFlushOutput } ;


function SerRead(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;

var     BytesRead: DWORD;
        Timeouts: TCommTimeouts;

begin
  if GetCommTimeouts(Handle, Timeouts) then begin
    Timeouts.ReadIntervalTimeout := MAXDWORD;
    Timeouts.ReadTotalTimeoutConstant := 0;
    SetCommTimeouts(Handle, Timeouts)
  end;
  if not ReadFile(Handle, Buffer, Count, BytesRead, nil) then
    result := 0
  else
    result := BytesRead
end { SerRead } ;


function SerWrite(Handle: TSerialHandle; const Buffer; Count: LongInt): LongInt;

var     BytesWritten: DWORD;

begin
  if not WriteFile(Handle, Buffer, Count, BytesWritten, nil) then
    result := 0
  else
    result := BytesWritten
end { SerWrite } ;


procedure SerSetParams(Handle: TSerialHandle; BitsPerSec: LongInt;
  ByteSize: Integer; Parity: TParityType; StopBits: Integer;
  Flags: TSerialFlags);

begin
  if SerSetParamsPrivate(Handle, BitsPerSec, ByteSize, Parity, StopBits, Flags) then begin
  end
end { SerSetParams } ;


function SerSaveState(Handle: TSerialHandle): TSerialState;

begin
  GetCommState(Handle, result)
end { SerSaveState } ;


procedure SerRestoreState(Handle: TSerialHandle; State: TSerialState);

begin
  SetCommState(Handle, State)
end { SerRestoreState } ;


(* Mask out a 2-bit field and merge in a replacement.                   *)
//
procedure merge2Bits(var flag: dword; startBit: integer; value: dword);

var     mask0, mask1: dword;
        index: integer;

begin
  mask0:= $fffffffc;
  for index:= 1 TO startBit do
    mask0:= (mask0 shl 1) or $00000001;
  mask1:= $00000003 shl startBit;
  value:= value shl startBit;
  flag:= (flag and mask0) or (value and mask1)
end { merge2Bits } ;


procedure SerSetDTR(Handle: TSerialHandle; State: Boolean);

var     dcb: TDCB;

begin
  if GetCommState(Handle, dcb) then begin
    if State then
      merge2Bits(dcb.Flags, 4, DTR_CONTROL_ENABLE)
    else
      merge2Bits(dcb.Flags, 4, DTR_CONTROL_DISABLE);
    SetCommState(Handle, dcb)
  end
end { SerSetDTR } ;


procedure SerSetRTS(Handle: TSerialHandle; State: Boolean);

var     dcb: TDCB;

begin
  if GetCommState(Handle, dcb) then begin
    if State then
      merge2Bits(dcb.Flags, 12, RTS_CONTROL_ENABLE)
    else
      merge2Bits(dcb.Flags, 12, RTS_CONTROL_DISABLE);
    SetCommState(Handle, dcb)
  end
end { SerSetRTS } ;


function SerGetCTS(Handle: TSerialHandle): Boolean;

var     status: dword;

begin
  if GetCommModemStatus(Handle, status) then
    result := status and MS_CTS_ON <> 0
  else
    result := false
end { SerGetCTS } ;


function SerGetDSR(Handle: TSerialHandle): Boolean;

var     status: dword;

begin
  if GetCommModemStatus(Handle, status) then
    result := status and MS_DSR_ON <> 0
  else
    result := false
end { SerGetDSR } ;


function SerGetCD(Handle: TSerialHandle): Boolean;

var     status: dword;

begin
  if GetCommModemStatus(Handle, status) then
    result := status and MS_RLSD_ON <> 0
  else
    result := false
end { SerGetCD } ;


function SerGetRI(Handle: TSerialHandle): Boolean;

var     status: dword;

begin
  if GetCommModemStatus(Handle, status) then
    result := status and MS_RING_ON <> 0
  else
    result := false
end { SerGetRI } ;


procedure SerBreak(Handle: TSerialHandle; mSec: LongInt=250; sync: boolean= true);

const   quantum= 100;

begin
  if sync then
    SerDrain(Handle);
  SetCommBreak(Handle);
  repeat
    if mSec < quantum then begin
      Sleep(mSec);
      mSec := 0;
    end else begin
      Sleep(quantum);
      mSec := msec-quantum;
    end;
    if (mSec > 0) and Assigned(SerialIdle) then
      SerialIdle(Handle);
  until mSec <= 0;
  ClearCommBreak(Handle);
  if sync then
    SerDrain(handle)
end { SerBreak } ;


function SerReadTimeout(Handle: TSerialHandle; var Buffer; mSec: LongInt): LongInt;

var     BytesRead: DWORD;
        Timeouts: TCommTimeouts;

begin
  if GetCommTimeouts(Handle, Timeouts) then begin
    Timeouts.ReadIntervalTimeout := 0;
    Timeouts.ReadTotalTimeoutConstant := mSec;
    SetCommTimeouts(Handle, Timeouts)
  end;
  if not ReadFile(Handle, Buffer, 1, BytesRead, nil) then
    result := 0
  else
    result := BytesRead
end { SerReadTimeout } ;


function SerReadTimeout(Handle: TSerialHandle; var Buffer: array of byte; count, mSec: LongInt): LongInt;

var     BytesRead: DWORD;
        Timeouts: TCommTimeouts;

begin
  if GetCommTimeouts(Handle, Timeouts) then begin
    Timeouts.ReadIntervalTimeout := 0;
    Timeouts.ReadTotalTimeoutConstant := mSec;
    SetCommTimeouts(Handle, Timeouts)
  end;
  if not ReadFile(Handle, Buffer[0], count, BytesRead, nil) then
    result := 0
  else
    result := BytesRead
end { SerReadTimeout } ;


end.
