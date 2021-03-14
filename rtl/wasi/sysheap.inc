{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                              Heap Management
*****************************************************************************}

function SysOSAlloc(size: ptruint): pointer;
const
  page_size = 65536;
  err = high(longword);
var
  res: ptruint;
begin
  res:=fpc_wasm32_memory_grow((size + page_size - 1) div page_size);
  if res<>err then
    SysOSAlloc:=pointer(res*page_size)
  else
    SysOSAlloc:=nil;
end;

procedure SysOSFree(p: pointer; size: ptruint);
begin
end;
