{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Low level file functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************

                          Low Level File Routines

****************************************************************************}

procedure do_close(h:thandle);
var
  RC: cardinal;
begin
{ Only three standard handles under real OS/2 }
  if h>2 then
   begin
    RC := DosClose (H);
    if RC <> 0 then
     begin
      InOutRes := longint (RC);
      OSErrorWatch (RC);
     end;
   end;
{$ifdef IODEBUG}
  writeln('do_close: handle=', H, ', InOutRes=', InOutRes);
{$endif}
end;

procedure do_erase(p:Pchar; pchangeable: boolean);
var
  oldp: pchar;
  RC: cardinal;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  oldp:=p;
  DoDirSeparators(p,pchangeable);
  RC := DosDelete (P);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;
  if p<>oldp then
    freemem(p);
=======
=======
>>>>>>> origin/fixes_2_2
  DoDirSeparators(p);
  inoutres:=DosDelete(p);
>>>>>>> graemeg/fixes_2_2
end;

procedure do_rename(p1,p2:Pchar; p1changeable, p2changeable: boolean);
var
  oldp1, oldp2 : pchar;
  RC: cardinal;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  oldp1:=p1;
  oldp2:=p2;
  DoDirSeparators(p1,p1changeable);
  DoDirSeparators(p2,p2changeable);
  RC := DosMove (p1, p2);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;
  if p1<>oldp1 then
    freemem(p1);
  if p2<>oldp2 then
    freemem(p2);
=======
=======
>>>>>>> origin/fixes_2_2
  DoDirSeparators(p1);
  DoDirSeparators(p2);
  inoutres:=DosMove(p1, p2);
>>>>>>> graemeg/fixes_2_2
end;

function do_read(h:thandle;addr:pointer;len:longint):longint;
Var
  T: cardinal;
  RC: cardinal;
begin
{$ifdef IODEBUG}
  write('do_read: handle=', h, ', addr=', ptrint(addr), ', length=', len);
{$endif}
  RC := DosRead(H, Addr, Len, T);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;
  do_read:= longint (T);
{$ifdef IODEBUG}
  writeln(', actual_len=', t, ', InOutRes=', InOutRes);
{$endif}
end;

function do_write(h:thandle;addr:pointer;len:longint) : longint;
Var
  T: cardinal;
  RC: cardinal;
begin
{$ifdef IODEBUG}
  write('do_write: handle=', h, ', addr=', ptrint(addr), ', length=', len);
{$endif}
  RC := DosWrite(H, Addr, Len, T);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;
  do_write:= longint (T);
{$ifdef IODEBUG}
  writeln(', actual_len=', t, ', InOutRes=', InOutRes);
{$endif}
end;

function Do_FilePos (Handle: THandle): int64;
var
  PosActual: int64;
  RC: cardinal;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  RC := Sys_DosSetFilePtrL (Handle, 0, 1, PosActual);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;
=======
  InOutRes := Sys_DosSetFilePtrL (Handle, 0, 1, PosActual);
>>>>>>> graemeg/fixes_2_2
=======
  InOutRes := Sys_DosSetFilePtrL (Handle, 0, 1, PosActual);
>>>>>>> origin/fixes_2_2
  Do_FilePos := PosActual;
{$ifdef IODEBUG}
  writeln('do_filepos: handle=', Handle, ', actual_pos=', PosActual, ', InOutRes=', InOutRes);
{$endif}
end;

procedure Do_Seek (Handle: THandle; Pos: int64);
var
  PosActual: int64;
  RC: cardinal;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  RC := Sys_DosSetFilePtrL(Handle, Pos, 0 {ZeroBased}, PosActual);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;
=======
  InOutRes:=Sys_DosSetFilePtrL(Handle, Pos, 0 {ZeroBased}, PosActual);
>>>>>>> graemeg/fixes_2_2
=======
  InOutRes:=Sys_DosSetFilePtrL(Handle, Pos, 0 {ZeroBased}, PosActual);
>>>>>>> origin/fixes_2_2
{$ifdef IODEBUG}
  writeln('do_seek: handle=', Handle, ', pos=', pos, ', actual_pos=', PosActual, ', InOutRes=', InOutRes);
{$endif}
end;

function Do_SeekEnd (Handle: THandle): int64;
var
  PosActual: int64;
  RC: cardinal;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  RC := Sys_DosSetFilePtrL (Handle, 0, 2 {EndBased}, PosActual);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
    Do_SeekEnd := -1;
   end
  else
   Do_SeekEnd := PosActual;
=======
=======
>>>>>>> origin/fixes_2_2
  InOutRes := Sys_DosSetFilePtrL (Handle, 0, 2 {EndBased}, PosActual);
  Do_SeekEnd := PosActual;
>>>>>>> graemeg/fixes_2_2
{$ifdef IODEBUG}
  writeln('do_seekend: handle=', Handle, ', actual_pos=', PosActual, ', InOutRes=', InOutRes);
{$endif}
end;

function Do_FileSize (Handle: THandle): int64;
var
  AktFilePos: int64;
begin
  AktFilePos := Do_FilePos (Handle);
  if InOutRes = 0 then
   begin
    Do_FileSize := Do_SeekEnd (Handle);
    Do_Seek (Handle, AktFilePos);
   end;
end;

procedure Do_Truncate (Handle: THandle; Pos: int64);
var
  RC: cardinal;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  RC := Sys_DosSetFileSizeL (Handle, Pos);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end
  else
   Do_SeekEnd (Handle);
=======
=======
>>>>>>> origin/fixes_2_2
  InOutRes := Sys_DosSetFileSizeL (Handle, Pos);
  Do_SeekEnd (Handle);
>>>>>>> graemeg/fixes_2_2
end;


const
    FileHandleCount: cardinal = 20;

function Increase_File_Handle_Count: boolean;
var L1: longint;
    L2: cardinal;
    RC: cardinal;
begin
  L1 := 10;
  RC := DosSetRelMaxFH (L1, L2);
  if RC <> 0 then
   begin
    Increase_File_Handle_Count := false;
    OSErrorWatch (RC);
   end
  else
   if L2 > FileHandleCount then
    begin
      FileHandleCount := L2;
      Increase_File_Handle_Count := true;
    end
   else
    Increase_File_Handle_Count := false;
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
  Action, Attrib, OpenFlags, FM: Cardinal;
  oldp : pchar;
  RC: cardinal;
begin
<<<<<<< HEAD
=======
  // convert unix slashes to normal slashes
  DoDirSeparators(p);

>>>>>>> graemeg/fixes_2_2
  // close first if opened
  if ((flags and $10000)=0) then
  begin
    case filerec(f).mode of
      fminput,fmoutput,fminout : Do_Close (FileRec (F).Handle);
      fmclosed:;
    else
      begin
        inoutres:=102; {not assigned}
        exit;
      end;
    end;
  end;

  // reset file handle
  filerec(f).handle := UnusedHandle;

  Attrib:=0;
  OpenFlags:=0;

  // convert filesharing
  FM := Flags and $FF and not (8);
(* DenyNone if sharing not specified. *)
  if FM and 112 = 0 then
    FM := FM or 64;
  // convert filemode to filerec modes and access mode
  case (FM and 3) of
    0: filerec(f).mode:=fminput;
    1: filerec(f).mode:=fmoutput;
    2: filerec(f).mode:=fminout;
  end;

  if (flags and $1000)<>0 then
    OpenFlags:=OpenFlags or 2 {doOverwrite} or 16 {doCreate} // Create/overwrite
  else
    OpenFlags:=OpenFlags or 1 {doOpen}; // Open existing

  // Handle Std I/O
  if p[0]=#0 then
  begin
    case FileRec(f).mode of
      fminput :
        FileRec(f).Handle:=StdInputHandle;
      fminout, // this is set by rewrite
      fmoutput :
        FileRec(f).Handle:=StdOutputHandle;
      fmappend :
        begin
          FileRec(f).Handle:=StdOutputHandle;
          FileRec(f).mode:=fmoutput; // fool fmappend
        end;
    end;
    exit;
  end;

  oldp:=p;
  // convert unix slashes to normal slashes
  DoDirSeparators(p,pchangeable);
  Attrib:=32 {faArchive};

<<<<<<< HEAD
<<<<<<< HEAD
  RC := Sys_DosOpenL(p, FileRec(F).Handle, Action, 0, Attrib, OpenFlags, FM, nil);
  if RC <> 0 then
   begin
    InOutRes := longint (RC);
    OSErrorWatch (RC);
   end;

  // If too many open files try to set more file handles and open again
  if (InOutRes = 4) then
   if Increase_File_Handle_Count then
    begin
     RC := Sys_DosOpenL(p, FileRec(F).Handle, Action, 0, Attrib, OpenFlags, FM, nil);
     if RC <> 0 then
      begin
       InOutRes := longint (RC);
       OSErrorWatch (RC);
      end;
    end;
  if RC <> 0 then
   FileRec(F).Handle:=UnusedHandle;
=======
=======
>>>>>>> origin/fixes_2_2
  InOutRes:=Sys_DosOpenL(p, FileRec(F).Handle, Action, 0, Attrib, OpenFlags, FM, nil);

  // If too many open files try to set more file handles and open again
  if (InOutRes = 4) then
    if Increase_File_Handle_Count then
      InOutRes:=Sys_DosOpenL(p, FileRec(F).Handle, Action, 0, Attrib, OpenFlags, FM, nil);

  If InOutRes<>0 then FileRec(F).Handle:=UnusedHandle;
>>>>>>> graemeg/fixes_2_2

  // If Handle created -> make some things
  if (FileRec(F).Handle <> UnusedHandle) then
  begin

    // Move to end of file for Append command
    if ((Flags and $100) <> 0) then
    begin
      if not (Do_IsDevice (FileRec (F).Handle)) then
       Do_SeekEnd (FileRec (F).Handle);
      FileRec(F).Mode := fmOutput;
    end;

  end
  else
    FileRec(f).mode:=fmclosed;

  if oldp<>p then
    freemem(p);

{$ifdef IODEBUG}
  writeln('do_open,', filerec(f).handle, ',', filerec(f).name, ',', filerec(f).mode, ', InOutRes=', InOutRes);
{$endif}
end;

function do_isdevice (Handle: THandle): boolean;
var
  HT, Attr: cardinal;
  RC: cardinal;
begin
  do_isdevice:=false;
  RC := DosQueryHType(Handle, HT, Attr);
  if RC <> 0 then
   begin
    OSErrorWatch (RC);
    Exit;
   end;
  if ht=1 then
   do_isdevice:=true;
end;
{$ASMMODE ATT}
