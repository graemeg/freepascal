{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Free Pascal development team

    Low level file functions for Nintendo Wii
    Copyright (c) 2011 by Francesco Lombardi

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
procedure do_close(handle: THandle);
begin
  if FileIODevice.FileIO.DoClose <> nil then
    FileIODevice.FileIO.DoClose(handle);
  //_fclose (_PFILE(pointer(handle))^);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_erase(p: pchar; pchangeable: boolean);
=======
procedure do_erase(p: pchar);
>>>>>>> graemeg/cpstrnew
=======
procedure do_erase(p: pchar);
>>>>>>> graemeg/cpstrnew
=======
procedure do_erase(p: pchar);
>>>>>>> graemeg/cpstrnew
=======
procedure do_erase(p: pchar);
>>>>>>> origin/cpstrnew
begin
  if FileIODevice.FileIO.DoErase <> nil then
    FileIODevice.FileIO.DoErase(p);
//  _unlink(p);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_rename(p1, p2: pchar; p1changeable, p2changeable: boolean);
=======
procedure do_rename(p1, p2: pchar);
>>>>>>> graemeg/cpstrnew
=======
procedure do_rename(p1, p2: pchar);
>>>>>>> graemeg/cpstrnew
=======
procedure do_rename(p1, p2: pchar);
>>>>>>> graemeg/cpstrnew
=======
procedure do_rename(p1, p2: pchar);
>>>>>>> origin/cpstrnew
begin
//  _rename(p1, p2);
  if FileIODevice.FileIO.DoRename <> nil then
    FileIODevice.FileIO.DoRename(p1, p2);
end;

function do_write(h: THandle; addr: pointer; len: longint) : longint;
begin
//  result := _fwrite(addr, 1, len, _PFILE(pointer(h))^);
  if FileIODevice.FileIO.DoWrite <> nil then
    result :=  FileIODevice.FileIO.DoWrite(h, addr, len);
end;

function do_read(h: THandle; addr: pointer; len: longint) : longint;
begin
//  result := _fread(addr, 1, len, _PFILE(pointer(h))^);
  if FileIODevice.FileIO.DoRead <> nil then
    result := FileIODevice.FileIO.DoRead(h, addr, len);
end;

function do_filepos(handle: THandle): longint;
begin
//  result := _ftell(_PFILE(pointer(handle))^);
  if FileIODevice.FileIO.DoFilePos <> nil then
    result := FileIODevice.FileIO.DoFilePos(handle);
end;

procedure do_seek(handle: THandle; pos: longint);
begin
  //_fseek(_PFILE(pointer(handle))^, pos, SEEK_SET);
  if FileIODevice.FileIO.DoSeek <> nil then
    FileIODevice.FileIO.DoSeek(handle, pos);
end;

function do_seekend(handle: THandle): longint;
begin
//  result := _fseek(_PFILE(pointer(handle))^, 0, SEEK_END);
  if FileIODevice.FileIO.DoSeekend <> nil then
    result := FileIODevice.FileIO.DoSeekend(handle);
end;

function do_filesize(handle: THandle): longint;
begin
//  result := -1;
  if FileIODevice.FileIO.DoFilesize <> nil then
    result := FileIODevice.FileIO.DoFilesize(handle);
end;

{ truncate at a given position }
procedure do_truncate(handle: THandle; pos: longint);
begin
//  _ftruncate(_fileno(_PFILE(pointer(handle))^), pos);
  if FileIODevice.FileIO.DoTruncate <> nil then
    FileIODevice.FileIO.DoTruncate(handle, pos);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_open(var f; p: pchar; flags: longint; pchangeable: boolean);
=======
procedure do_open(var f; p: pchar; flags: longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f; p: pchar; flags: longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f; p: pchar; flags: longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f; p: pchar; flags: longint);
>>>>>>> origin/cpstrnew
begin
(*
{ close first if opened }
  if ((flags and $10000) = 0) then
   begin
     case FileRec(f).mode of
      fminput,fmoutput,fminout : Do_Close(FileRec(f).Handle);
      fmclosed : ;
     else
      begin
//        inoutres:=102; {not assigned}
        exit;
      end;
     end;
   end;
{ reset file Handle }
  FileRec(f).Handle:=UnusedHandle;

{ We do the conversion of filemodes here, concentrated on 1 place }
  case (flags and 3) of
   0 : begin
         oflags := 'rb'#0;
         filerec(f).mode := fminput;
       end;
   1 : begin
         if (flags and $1000)=$1000 then
           oflags := 'w+b' else
           oflags := 'wb';
         filerec(f).mode := fmoutput;
       end;
   2 : begin
         if (flags and $1000)=$1000 then
           oflags := 'w+' else
           oflags := 'r+';
         filerec(f).mode := fminout;
       end;
  end;
  {if (flags and $1000)=$1000 then
   oflags:=oflags or (O_CREAT or O_TRUNC)
  else
   if (flags and $100)=$100 then
    oflags:=oflags or (O_APPEND);}

{ empty name is special }
  if p[0]=#0 then
  begin
    case FileRec(f).mode of
      fminput: FileRec(f).Handle:=StdInputHandle;
      fminout, { this is set by rewrite }
      fmoutput: FileRec(f).Handle:=StdOutputHandle;
      fmappend:
      begin
        FileRec(f).Handle:=StdOutputHandle;
        FileRec(f).mode:=fmoutput; {fool fmappend}
      end;
    end;
    exit;
  end;

{ real open call }
  FileRec(f).Handle := longint(fopen(p, @oflags[1]));//_open(p,oflags,438);
  // errno does not seem to be set on succsess ??
{
  if FileRec(f).Handle = 0 then
    Errno2Inoutres
  else
    InOutRes := 0;
}
*)
//  FileRec(f).Handle := THandle (_fopen(p, @oflags[1]));
  if FileIODevice.FileIO.DoOpen <> nil then
    FileIODevice.FileIO.DoOpen(f, p, flags);
end;

function do_isdevice(handle: THandle): boolean;
begin
//  result :=  (_isatty(_fileno(_PFILE(pointer(handle))^)) > 0);
  if FileIODevice.FileIO.DoIsdevice <> nil then
    result := FileIODevice.FileIO.DoIsdevice(handle);
end;


