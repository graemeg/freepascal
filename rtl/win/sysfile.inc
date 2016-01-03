{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Low leve file functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                          Low Level File Routines
*****************************************************************************}

function do_isdevice(handle:thandle):boolean;
begin
  do_isdevice:=(handle = StdInputHandle) or (handle = StdOutputHandle) or (handle = StdErrorHandle);
end;


procedure do_close(h : thandle);
begin
  if do_isdevice(h) then
   exit;
  if CloseHandle(h)=0 then
    Errno2InOutRes(GetLastError);
end;


procedure do_erase(p: pwidechar; pchangeable: boolean);
var
  oldp: pwidechar;
  err: longword;
begin
<<<<<<< HEAD
<<<<<<< HEAD
   oldp:=p;
   DoDirSeparators(p,pchangeable);
   if DeleteFileW(p)=0 then
=======
=======
>>>>>>> origin/fixes_2_2
   DoDirSeparators(p);
   if DeleteFile(p)=0 then
>>>>>>> graemeg/fixes_2_2
    Begin
      err:=GetLastError;
      if err=5 then
       begin
<<<<<<< HEAD
<<<<<<< HEAD
         if ((GetFileAttributesW(p) and FILE_ATTRIBUTE_DIRECTORY)=FILE_ATTRIBUTE_DIRECTORY) then
          err:=2;
=======
=======
>>>>>>> origin/fixes_2_2
         if ((GetFileAttributes(p) and FILE_ATTRIBUTE_DIRECTORY)=FILE_ATTRIBUTE_DIRECTORY) then
          errno:=2;
>>>>>>> graemeg/fixes_2_2
       end;
      Errno2InoutRes(err);
    end;
   if p<>oldp then
     freemem(p);
end;


procedure do_rename(p1,p2: pwidechar; p1changeable, p2changeable: boolean);
var
  oldp1,oldp2: pwidechar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  oldp1:=p1;
  oldp2:=p2;
  DoDirSeparators(p1,p1changeable);
  DoDirSeparators(p2,p2changeable);
  if MoveFileW(p1,p2)=0 then
    Begin
      Errno2InoutRes(GetLastError);
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
  if MoveFile(p1,p2)=0 then
   Begin
      errno:=GetLastError;
      Errno2InoutRes;
   end;
>>>>>>> graemeg/fixes_2_2
end;


function do_write(h:thandle;addr:pointer;len : longint) : longint;
var
   size:longint;
{$ifndef WINCE}
   ConsoleMode : dword;
   CodePage : UInt;
   accept_smaller_size : boolean;
{$endif ndef WINCE}
begin
   if writefile(h,addr,len,size,nil)=0 then
    Begin
<<<<<<< HEAD
      Errno2InoutRes(GetLastError);
=======
      errno:=GetLastError;
      Errno2InoutRes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef WINCE}
    end
   else if (size<len) then
    Begin
      if GetConsoleMode (h, @ConsoleMode) then
       Begin
         accept_smaller_size:=false;
         { GetConsoleMode success means that we do have a
           console handle that might return less than
           LEN because a UTF-8 with length LEN input was
           transformed into a shorter string of size SIZE }
         CodePage:=GetConsoleOutputCP;
         Case CodePage of
           1200, {utf-16}
           1201, {unicodeFFFE}
           12000, {utf-32}
           12001, {utf-32BE}
           65000, {utf-7}
           65001: {utf-8}
             accept_smaller_size:=true;
         end;
         if accept_smaller_size then
           size:=len;
       end;

{$endif ndef WINCE}
    end;
   do_write:=size;
end;


function do_read(h:thandle;addr:pointer;len : longint) : longint;
var
  _result:longint;
  err: longword;
begin
  if readfile(h,addr,len,_result,nil)=0 then
    Begin
      err:=GetLastError;
      if err<>ERROR_BROKEN_PIPE then
        Errno2InoutRes(err);
    end;
  do_read:=_result;
end;


type
  tint64rec = record
    low, high: dword;
  end;

function do_filepos(handle : thandle) : Int64;
var
  rslt: tint64rec;
begin
  rslt.high := 0;
  rslt.low := SetFilePointer(handle, 0, @rslt.high, FILE_CURRENT);
  if (rslt.low = $FFFFFFFF) and (GetLastError <> 0) then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Errno2InoutRes(GetLastError);
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> origin/cpstrnew
  end;
  do_filepos := int64(rslt);
end;


procedure do_seek(handle: thandle; pos: Int64);
var
  posHigh: LongInt;
begin
  posHigh := tint64rec(pos).high;
  if (SetFilePointer(handle, pos, @posHigh, FILE_BEGIN)=-1) and
  { return value of -1 is valid unless GetLastError is non-zero }
    (GetLastError <> 0) then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Errno2InoutRes(GetLastError);
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> origin/cpstrnew
  end;
end;


function do_seekend(handle:thandle):Int64;
var
  rslt: tint64rec;
begin
  rslt.high := 0;
  rslt.low := SetFilePointer(handle, 0, @rslt.high, FILE_END);
  if (rslt.low = $FFFFFFFF) and (GetLastError <> 0) then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Errno2InoutRes(GetLastError);
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> graemeg/cpstrnew
=======
    errno := GetLastError;
    Errno2InoutRes;
>>>>>>> origin/cpstrnew
  end;
  do_seekend := int64(rslt);
end;


function do_filesize(handle : thandle) : Int64;
var
  aktfilepos : Int64;
begin
  aktfilepos:=do_filepos(handle);
  do_filesize:=do_seekend(handle);
  do_seek(handle,aktfilepos);
end;


procedure do_truncate (handle:thandle;pos:Int64);
begin
   do_seek(handle,pos);
   if not(SetEndOfFile(handle)) then
    begin
      Errno2InoutRes(GetLastError);
    end;
end;


procedure do_open(var f; p: pwidechar; flags: longint; pchangeable: boolean);
{
  filerec and textrec have both handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}
Const
  file_Share_Read   = $00000001;
  file_Share_Write  = $00000002;
  file_Share_Delete = $00000004;
Var
  shflags,
  oflags,cd : longint;
  security : TSecurityAttributes;
  oldp : pwidechar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
  DoDirSeparators(p);
>>>>>>> graemeg/fixes_2_2
=======
  DoDirSeparators(p);
>>>>>>> origin/fixes_2_2
  { close first if opened }
  if ((flags and $10000)=0) then
   begin
     case filerec(f).mode of
       fminput,fmoutput,fminout : Do_Close(filerec(f).handle);
       fmclosed : ;
     else
      begin
        {not assigned}
        inoutres:=102;
        exit;
      end;
     end;
   end;
  oldp:=p;
  DoDirSeparators(p,pchangeable);
  { reset file handle }
  filerec(f).handle:=UnusedHandle;
  { convert filesharing }
  shflags:=0;
  if ((filemode and fmshareExclusive) = fmshareExclusive) then
    { no sharing }
  else
    if ((filemode and $f0) = fmShareCompat) or ((filemode and fmshareDenyWrite) = fmshareDenyWrite) then
      shflags := file_Share_Read
  else
    if ((filemode and fmshareDenyRead) = fmshareDenyRead) then
      shflags := file_Share_Write
  else
    if ((filemode and fmshareDenyNone) = fmshareDenyNone) then
      shflags :=
{$ifdef WINCE}
        { WinCE does not know file_share_delete }
        file_Share_Read or file_Share_Write;
{$else WINCE}
        fmShareDenyNoneFlags;
{$endif WINCE}
  { convert filemode to filerec modes }
  case (flags and 3) of
   0 : begin
         filerec(f).mode:=fminput;
         oflags:=longint(GENERIC_READ);
       end;
   1 : begin
         filerec(f).mode:=fmoutput;
         oflags:=longint(GENERIC_WRITE);
       end;
   2 : begin
         filerec(f).mode:=fminout;
         oflags:=longint(GENERIC_WRITE or GENERIC_READ);
       end;
  end;
  { create it ? }
  if (flags and $1000)<>0 then
    cd:=CREATE_ALWAYS
  { or Append/Open ? }
  else
    cd:=OPEN_EXISTING;
  { empty name is special }
  if p[0]=#0 then
   begin
     case FileRec(f).mode of
       fminput :
         FileRec(f).Handle:=StdInputHandle;
       fminout, { this is set by rewrite }
       fmoutput :
         FileRec(f).Handle:=StdOutputHandle;
       fmappend :
         begin
           FileRec(f).Handle:=StdOutputHandle;
           FileRec(f).mode:=fmoutput; {fool fmappend}
         end;
     end;
     { no dirseparators can have been replaced in the empty string -> no need
       to check whether we have to free p }
     exit;
   end;
  security.nLength := Sizeof(TSecurityAttributes);
  security.bInheritHandle:=true;
  security.lpSecurityDescriptor:=nil;
  filerec(f).handle:=CreateFileW(p,oflags,shflags,@security,cd,FILE_ATTRIBUTE_NORMAL,0);

  { append mode }
  if ((flags and $100)<>0) and
     (filerec(f).handle<>0) and
     (filerec(f).handle<>UnusedHandle) then
   begin
     do_seekend(filerec(f).handle);
     filerec(f).mode:=fmoutput; {fool fmappend}
   end;

  { get errors }
  { handle -1 is returned sometimes !! (PM) }
  if (filerec(f).handle=0) or (filerec(f).handle=UnusedHandle) then
    begin
      Errno2InoutRes(GetLastError);
      FileRec(f).mode:=fmclosed;
    end;
  if oldp<>p then
    freemem(p);
end;
