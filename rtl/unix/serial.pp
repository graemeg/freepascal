{ Unit for handling the serial interfaces for Linux and similar Unices.
  (c) 2000 Sebastian Guenther, sg@freepascal.org
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

{ Flushes the data queues of the given serial device. }
procedure SerFlush(Handle: TSerialHandle);

{ Reads a maximum of "Count" bytes of data into the specified buffer.
  Result: Number of bytes read. }
function SerRead(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;

{ Tries to write "Count" bytes from "Buffer".
  Result: Number of bytes written. }
function SerWrite(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;

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
function SerGetRI(Handle: TSerialHandle): Boolean;


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

procedure SerFlush(Handle: TSerialHandle);
begin
  fpfsync(Handle);
end;

function SerRead(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;
begin
  Result := fpRead(Handle, Buffer, Count);
end;

function SerWrite(Handle: TSerialHandle; var Buffer; Count: LongInt): LongInt;
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
    else tios.c_cflag := B9600;
  end;
  tios.c_ispeed := tios.c_cflag;
  tios.c_ospeed := tios.c_ispeed;

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

function SerGetRI(Handle: TSerialHandle): Boolean;
var
  Flags: Cardinal;
begin
  fpioctl(Handle, TIOCMGET, @Flags);
  Result := (Flags and TIOCM_RI) <> 0;
end;


end.
