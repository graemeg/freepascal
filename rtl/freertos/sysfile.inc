{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Free Pascal development team

    Low level file functions
    GBA does not have any drive, so no file handling is needed.
    Copyright (c) 2006 by Francesco Lombardi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************
                        Low level File Routines
               All these functions can set InOutRes on errors
****************************************************************************}

{ close a file from the handle value }
procedure do_close(handle : longint);
begin
end;


procedure do_erase(p : pchar; pchangeable: boolean);
begin
  if assigned (@rtl_do_erase) then
    rtl_do_erase(p);
end;


procedure do_rename(p1,p2 : pchar; p1changeable, p2changeable: boolean);
begin
  if assigned (@rtl_do_rename) then
    rtl_do_rename(p1, p2);
end;


function do_write(h: longint; addr: pointer; len: longint) : longint;
begin
  if assigned (rtl_do_write) then
    result := rtl_do_write(h, addr, len)
  else
    result := -1;
end;


function do_read(h: longint; addr: pointer; len: longint) : longint;
begin
  if assigned (rtl_do_read) then
    result := rtl_do_read(h, addr, len)
  else
    result := -1;
end;


function do_filepos(handle: longint) : longint;
begin
  if assigned (rtl_do_filepos) then
    result := rtl_do_filepos(handle)
  else
    result := -1;
end;


procedure do_seek(handle, pos: longint);
begin
  if assigned (rtl_do_seek) then
    rtl_do_seek(handle, pos);
end;


function do_seekend(handle: longint):longint;
begin
  if assigned (rtl_do_seekend) then
    result := rtl_do_seekend(handle)
  else
    result := -1;
end;


function do_filesize(handle : longint) : longint;
begin
  result := -1;
  if assigned (rtl_do_filesize) then
    result := rtl_do_filesize(handle);
end;


{ truncate at a given position }
procedure do_truncate(handle, pos: longint);
begin
  if assigned (rtl_do_truncate) then
    rtl_do_truncate(handle, pos);
end;


procedure do_open(var f;p:PFileTextRecChar;flags:longint; pchangeable: boolean);
begin
  if assigned (rtl_do_open) then
    rtl_do_open(f, p, flags);
end;


function do_isdevice(handle: THandle): boolean;
begin
  result := false;
  if assigned (rtl_do_isdevice) then
    result := rtl_do_isdevice(handle);
end;
