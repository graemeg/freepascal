{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt

    OS/2 specific part of pipe stream.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

uses
 DosCalls;

const
 PipeBufSize = 1024;

Function CreatePipeHandles (Var Inhandle,OutHandle : Longint) : Boolean;

begin
  CreatePipeHandles := DosCreatePipe (InHandle, OutHandle, PipeBufSize) = 0;
end;

Function TInputPipeStream.GetNumBytesAvailable: DWord;

begin
  // TODO: find out if this is possible in OS/2
  Result := 0;
end;

