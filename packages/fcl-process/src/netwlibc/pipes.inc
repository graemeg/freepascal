{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by Michael Van Canneyt

    Netware specific part of pipe stream.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

// Unsupported for the moment...

Function CreatePipeHandles (Var Inhandle,OutHandle : THandle) : Boolean;

begin
  Result := false;  {todo}
end;


Function TInputPipeStream.GetNumBytesAvailable: DWord;

begin
  Result := 0;
end;

