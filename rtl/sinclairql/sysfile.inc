{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Free Pascal development team

    Low level file functions for the Sinclair QL

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
  Error2InOutRes(io_close(handle));
end;

{ delete a file, given its name }
procedure do_erase(p : pchar; pchangeable: boolean);
begin
  Error2InOutRes(io_delet(p));
end;


procedure do_rename(p1,p2 : pchar; p1changeable, p2changeable: boolean);
var
  chanId: longint;
  res: longint;
begin
  { To rename a QL file, it must exist and be opened. For WIN/FLP this
    means open mode 0 (Q_OPEN) but for RAM this can be any of Q_OPEN, 
    Q_OPEN_NEW or Q_OPEN_OVER. }

  { Does the file exist? }
  chanId := io_open(p1, Q_OPEN_IN);
  if chanId < 0 then
    begin
      InOutRes:=2;    { File not found. }
      exit;
    end;

  { Close and reopen in correct mode. }
  io_close(chanId);

  chanId := io_open(p1, Q_OPEN);
  if chanId < 0 then
    begin
      Error2InOutRes(chanId);
      exit;
    end;

  { Now, finally, we can rename. }
  res := fs_rename(chanId,p2);

  { Close the file. Never errors out. }
  io_close(chanId);

  if res < 0 then
    Error2InOutRes(res);
end;


function do_write(h: longint; addr: pointer; len: longint) : longint;
var
  res: longint;
begin
  do_write:=0;
  res:=io_sstrg(h, -1, addr, len);
  if res < 0 then
    Error2InOutRes(res)
  else
    do_write:=res;
end;


function do_read(h: longint; addr: pointer; len: longint) : longint;
var
  res: longint;
begin
  do_read := 0;
  res := io_fline(h, -1, addr, len);
  if res = ERR_EF then
    res := 0;
  if res < 0 then
    Error2InOutRes(res)
  else
    do_read := res;
end;


function do_filepos(handle: longint): longint;
var
  res: longint;
  pos: longint;
begin
  do_filepos := 0;
  pos := 0;
  res := fs_posre(handle, pos);
  if res = ERR_EF then
    res := 0;
  if (res < 0) then
    Error2InOutRes(res)
  else
    do_filepos := pos;
end;


procedure do_seek(handle, pos: longint);
var
  res: longint;
begin
  res := fs_posab(handle, pos);
  if res = ERR_EF then
    res := 0;
  if (res < 0) then
    Error2InOutRes(res);
end;


{ The maximum length of a QL file is 2^31 - 64 bytes ($7FFFFFC0)
  so the maximum offset is that, minus 1. ($7fffffBF) }

const
  MAX_QL_FILE_LENGTH = $7FFFFFBF;

function do_seekend(handle: longint): longint;
var
  res: longint;
  pos: longint;
begin
  do_seekend:=-1;
  pos:=MAX_QL_FILE_LENGTH;
  res:=fs_posab(handle, pos);
  if res = ERR_EF then
    res := 0;
  if res < 0 then
    Error2InOutRes(res)
  else
    do_seekend := pos;
end;


function do_filesize(handle: longint): longint;
var
  res: longint;
  header: array [0..$39] of byte;
begin
  do_filesize := 0;
  res := fs_headr(handle, @header, $40);
  if res < 0 then
    Error2InOutRes(res)
  else
    do_filesize := plongint(@header[0])^;
end;


{ truncate at a given position }
procedure do_truncate(handle, pos: longint);
begin
  do_seek(handle, pos);
  fs_truncate(handle);
end;


procedure do_open(var f;p:pchar;flags:longint; pchangeable: boolean);
{
  filerec and textrec have both handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}
var
  res: longint;
  openMode: longint;
begin
  openMode:=Q_OPEN;

  { close first if opened }
  if ((flags and $10000)=0) then
   begin
     case filerec(f).mode of
       fmInput, fmOutput, fmInout:
         do_close(filerec(f).handle);
       fmClosed: ;
     else
       begin
         InOutRes:=102; {not assigned}
         exit;
       end;
     end;
   end;

  { reset file handle }
  filerec(f).handle:=UnusedHandle;

  { convert filemode to filerec modes }
  case (flags and 3) of
    0 : filerec(f).mode:=fmInput;
    1 : filerec(f).mode:=fmOutput;
    2 : filerec(f).mode:=fmInout;
  end;

  { empty name is special }
  if p[0]=#0 then begin
    case filerec(f).mode of
      fminput :
        filerec(f).handle:=StdInputHandle;
      fmappend,
      fmoutput : begin
        filerec(f).handle:=StdOutputHandle;
        filerec(f).mode:=fmOutput; {fool fmappend}
      end;
    end;
    exit;
  end;

  { rewrite (create a new file) }
  if (flags and $1000)<>0 then openMode:=Q_OPEN_OVER;

  res:=io_open(p,openMode);

  if res < 0 then
    begin
      Error2InOutRes(res);
      filerec(f).mode:=fmClosed;
      exit;
    end
  else
    filerec(f).handle:=res;

  { append mode }
  if ((Flags and $100)<>0) and
      (FileRec(F).Handle<>UnusedHandle) then begin
    do_seekend(filerec(f).handle);
    filerec(f).mode:=fmOutput; {fool fmappend}
  end;
end;


function do_isdevice(handle: thandle): boolean;
begin
  do_isdevice:=false;
end;
