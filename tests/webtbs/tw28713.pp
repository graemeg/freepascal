{ %OPT=-O3 }
// Compiled with option -O3 for Win32-I386

type
  PWordArray = ^TWordArray;
  TWordArray = array [0..1023]of Word;

  WordRec = packed record
{$ifdef FPC}
{$ifdef FPC_LITTLE_ENDIAN}
    LoByte,HiByte:Byte
{$endif}
{$ifdef FPC_BIG_ENDIAN}
    HiByte,LoByte:Byte
{$endif}
{$endif}
  end;

var
  Buffer:TWordArray;
  OldMousePos:LongInt = 0;
  ScreenBuffer:Pointer = @Buffer;

procedure Show(ScreenBuffer:Pointer);
begin
  WordRec(PWordArray(ScreenBuffer)^[OldMousePos]).HiByte:=(not
  WordRec(PWordArray(ScreenBuffer)^[OldMousePos]).HiByte)and $7F
  // he forgets to write the result into the array
end;

begin
  Buffer[0]:=$0000;
  Show(ScreenBuffer);
  if Buffer[0]<>$7F00 then
    halt(1);
  writeln('ok');
end.
