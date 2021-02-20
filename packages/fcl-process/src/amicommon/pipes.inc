{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt

    DOS/go32v2 specific part of pipe stream.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

uses
  exec, AmigaDos;

// No pipes under dos, sorry...

Function CreatePipeHandles (Var Inhandle,OutHandle : THandle; APipeBufferSize : Cardinal = 1024) : Boolean;
begin
  InHandle := THandle (UnusedHandle);
  OutHandle := THandle (UnusedHandle);
  Result := False;
end;


Function TInputPipeStream.GetNumBytesAvailable: DWord;
var
  fib: TFileInfoBlock;
begin
  Result := 0;
  if Boolean(ExamineFH(BPTR(Handle), @fib)) then
    Result := fib.fib_size;
end;

function TInputPipeStream.GetPosition: Int64;
begin
  Result:=FPos;
end;

procedure TInputPipeStream.InvalidSeek;
begin
  Raise EPipeSeek.Create (ENoSeekMsg);
end;

procedure PipeClose (const FHandle: THandle);
var
  Filename: array[0..255] of Char;
  DeleteIt: Boolean;
begin
  if (FHandle <> UnusedHandle) and (FHandle <> 0) then
  begin
   DeleteIt := NameFromFH(BPTR(FHandle), @(Filename[0]), 255);
   FileClose(FHandle);
   if DeleteIt then
     AmigaDos.dosDeleteFile(@(Filename[0]));
 end;
end;
