{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2010 by Sven Barth

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
  do_isdevice := (handle = StdInputHandle) or
                 (handle = StdOutputHandle) or
                 (handle = StdErrorHandle);
end;


procedure do_close(h : thandle);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  res: LongInt;
begin
  if do_isdevice(h) then
    Exit;
  res:=NtClose(h);
  if res <> STATUS_SUCCESS then
    begin
      errno:=res;
      Errno2InOutRes;
    end;
end;


procedure do_erase(p : pwidechar; pchangeable: boolean);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  if do_isdevice(h) then
    Exit;
  NtClose(h);
end;


procedure do_erase(p : pchar);
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
var
  ntstr: TNtUnicodeString;
  objattr: TObjectAttributes;
  iostatus: TIOStatusBlock;
  h: THandle;
  disp: TFileDispositionInformation;
  res: LongInt;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldp: pwidechar;
begin
  InoutRes := 4;
  oldp:=p;
  DoDirSeparators(p,pchangeable);

  SysPWideCharToNtStr(ntstr, p, 0);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  InoutRes := 4;
  DoDirSeparators(p);

  SysPCharToNtStr(ntstr, p, 0);
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
  SysInitializeObjectAttributes(objattr, @ntstr, 0, 0, Nil);

  res := NtCreateFile(@h, NT_DELETE or NT_SYNCHRONIZE, @objattr, @iostatus, Nil,
            0, FILE_SHARE_READ or FILE_SHARE_WRITE or FILE_SHARE_DELETE,
            FILE_OPEN, FILE_NON_DIRECTORY_FILE or FILE_SYNCHRONOUS_IO_NONALERT,
            Nil, 0);

  if res >= 0 then begin
    disp.DeleteFile := True;

    res := NtSetInformationFile(h, @iostatus, @disp,
      SizeOf(TFileDispositionInformation), FileDispositionInformation);

    errno := res;

    NtClose(h);
  end else
  if res = STATUS_FILE_IS_A_DIRECTORY then
    errno := 2
  else
    errno := res;

  SysFreeNtStr(ntstr);
  Errno2InoutRes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if p<>oldp then
    freemem(p);
end;


procedure do_rename(p1,p2 : pwidechar; p1changeable, p2changeable: boolean);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


procedure do_rename(p1,p2 : pchar);
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
var
  h: THandle;
  objattr: TObjectAttributes;
  iostatus: TIOStatusBlock;
  dest, src: TNtUnicodeString;
  renameinfo: PFileRenameInformation;
  res: LongInt;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldp1, oldp2 : pwidechar;
begin
  oldp1:=p1;
  oldp2:=p2;

  { check whether the destination exists first }
  DoDirSeparators(p2,p2changeable);
  SysPWideCharToNtStr(dest, p2, 0);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  DoDirSeparators(p1);
  DoDirSeparators(p2);

  { check whether the destination exists first }
  SysPCharToNtStr(dest, p2, 0);
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
  SysInitializeObjectAttributes(objattr, @dest, 0, 0, Nil);

  res := NtCreateFile(@h, 0, @objattr, @iostatus, Nil, 0,
           FILE_SHARE_READ or FILE_SHARE_WRITE, FILE_OPEN,
           FILE_NON_DIRECTORY_FILE, Nil, 0);
  if res >= 0 then begin
    { destination already exists => error }
    NtClose(h);
    errno := 5;
    Errno2InoutRes;
  end else begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    DoDirSeparators(p1,p1changeable);
    SysPWideCharToNtStr(src, p1, 0);
=======
    SysPCharToNtStr(src, p1, 0);
>>>>>>> graemeg/cpstrnew
=======
    SysPCharToNtStr(src, p1, 0);
>>>>>>> graemeg/cpstrnew
=======
    SysPCharToNtStr(src, p1, 0);
>>>>>>> graemeg/cpstrnew
=======
    SysPCharToNtStr(src, p1, 0);
>>>>>>> origin/cpstrnew
    SysInitializeObjectAttributes(objattr, @src, 0, 0, Nil);

    res := NtCreateFile(@h, GENERIC_ALL or NT_SYNCHRONIZE or FILE_READ_ATTRIBUTES,
             @objattr, @iostatus, Nil, 0, FILE_SHARE_READ or FILE_SHARE_WRITE,
             FILE_OPEN, FILE_OPEN_FOR_BACKUP_INTENT or FILE_OPEN_REMOTE_INSTANCE
             or FILE_NON_DIRECTORY_FILE or FILE_SYNCHRONOUS_IO_NONALERT, Nil,
             0);

    if res >= 0 then begin
      renameinfo := GetMem(SizeOf(TFileRenameInformation) + dest.Length);
      with renameinfo^ do begin
        ReplaceIfExists := False;
        RootDirectory := 0;
        FileNameLength := dest.Length;
        Move(dest.Buffer^, renameinfo^.FileName, dest.Length);
      end;

      res := NtSetInformationFile(h, @iostatus, renameinfo,
               SizeOf(TFileRenameInformation) + dest.Length,
               FileRenameInformation);
      if res < 0 then begin
        { this could happen if src and destination reside on different drives,
          so we need to copy the file manually }
        {$message warning 'do_rename: Implement file copy!'}
        errno := res;
        Errno2InoutRes;
      end;

      NtClose(h);
    end else begin
      errno := res;
      Errno2InoutRes;
    end;

    SysFreeNtStr(src);
  end;

  SysFreeNtStr(dest);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if p1<>oldp1 then
    freemem(p1);
  if p2<>oldp2 then
    freemem(p2);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


function do_write(h:thandle;addr:pointer;len : longint) : longint;
var
  res: LongInt;
  iostatus: TIoStatusBlock;
begin
  res := NtWriteFile(h, 0, Nil, Nil, @iostatus, addr, len, Nil, Nil);

  if res = STATUS_PENDING then begin
    res := NtWaitForSingleObject(h, False, Nil);
    if res >= 0 then
      res := iostatus.Status;
  end;

  if res < 0 then begin
    errno := res;
    Errno2InoutRes;
    do_write := 0;
  end else
    do_write := LongInt(iostatus.Information);
end;


function do_read(h: thandle; addr: pointer; len: longint): longint;
var
  iostatus: TIOStatusBlock;
  res: LongInt;
begin
  res := NtReadFile(h, 0, Nil, Nil, @iostatus, addr, len, Nil, Nil);

  if res = STATUS_PENDING then begin
    res := NtWaitForSingleObject(h, False, Nil);
    if res >= 0 then
      res := iostatus.Status;
  end;

  if (res < 0) and (res <> STATUS_PIPE_BROKEN) then begin
    errno := res;
    Errno2InoutRes;
    do_read := 0;
  end else
  if res = STATUS_PIPE_BROKEN then
    do_read := 0
  else
    do_read := LongInt(iostatus.Information);
end;


function do_filepos(handle : thandle) : Int64;
var
  res: LongInt;
  iostatus: TIoStatusBlock;
  position: TFilePositionInformation;
begin
  res := NtQueryInformationFile(handle, @iostatus, @position,
           SizeOf(TFilePositionInformation), FilePositionInformation);

  if res < 0 then begin
    errno := res;
    Errno2InoutRes;
    do_filepos := 0;
  end else
    do_filepos := position.CurrentByteOffset.QuadPart;
end;


procedure do_seek(handle: thandle; pos: Int64);
var
  position: TFilePositionInformation;
  iostatus: TIoStatusBlock;
  res: LongInt;
begin
  position.CurrentByteOffset.QuadPart := pos;
  res := NtSetInformationFile(handle, @iostatus, @position,
           SizeOf(TFilePositionInformation), FilePositionInformation);
  if res < 0 then begin
    errno := res;
    Errno2InoutRes;
  end;
end;


function do_seekend(handle:thandle):Int64;
var
  res: LongInt;
  standard: TFileStandardInformation;
  position: TFilePositionInformation;
  iostatus: TIoStatusBlock;
begin
  do_seekend := 0;

  res := NtQueryInformationFile(handle, @iostatus, @standard,
           SizeOf(TFileStandardInformation), FileStandardInformation);
  if res >= 0 then begin
    position.CurrentByteOffset.QuadPart := standard.EndOfFile.QuadPart;
    res := NtSetInformationFile(handle, @iostatus, @position,
             SizeOf(TFilePositionInformation), FilePositionInformation);
    if res >= 0 then
      do_seekend := position.CurrentByteOffset.QuadPart;
  end;

  if res < 0 then begin
    errno := res;
    Errno2InoutRes;
  end;
end;


function do_filesize(handle : thandle) : Int64;
var
  res: LongInt;
  iostatus: TIoStatusBlock;
  standard: TFileStandardInformation;
begin
  res := NtQueryInformationFile(handle, @iostatus, @standard,
           SizeOf(TFileStandardInformation), FileStandardInformation);
  if res >= 0 then
    do_filesize := standard.EndOfFile.QuadPart
  else begin
    errno := res;
    Errno2InoutRes;
    do_filesize := 0;
  end;
end;


procedure do_truncate (handle:thandle;pos:Int64);
var
  endoffileinfo: TFileEndOfFileInformation;
  allocinfo: TFileAllocationInformation;
  iostatus: TIoStatusBlock;
  res: LongInt;
begin
  // based on ReactOS' SetEndOfFile
  endoffileinfo.EndOfFile.QuadPart := pos;
  res := NtSetInformationFile(handle, @iostatus, @endoffileinfo,
           SizeOf(TFileEndOfFileInformation), FileEndOfFileInformation);
  if res < 0 then begin
    errno := res;
    Errno2InoutRes;
  end else begin
    allocinfo.AllocationSize.QuadPart := pos;
    res := NtSetInformationFile(handle, @iostatus, @allocinfo,
             SizeOf(TFileAllocationInformation), FileAllocationInformation);
    if res < 0 then begin
      errno := res;
      Errno2InoutRes;
    end;
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_open(var f;p:pwidechar;flags:longint; pchangeable: boolean);
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> origin/cpstrnew
{
  filerec and textrec have both handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}
var
  shflags, cd, oflags: LongWord;
  objattr: TObjectAttributes;
  iostatus: TIoStatusBlock;
  ntstr: TNtUnicodeString;
  res: LongInt;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldp : pwidechar;
begin
=======
begin
  DoDirSeparators(p);
>>>>>>> graemeg/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> graemeg/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> graemeg/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> origin/cpstrnew
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
  { reset file handle }
  filerec(f).handle:=UnusedHandle;
  { convert filesharing }
  shflags := 0;
  if ((filemode and fmshareExclusive) = fmshareExclusive) then
    { no sharing }
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if ((filemode and $F0) = fmShareCompat) or ((filemode and fmshareDenyWrite) = fmshareDenyWrite) then
=======
    if (filemode = fmShareCompat) or ((filemode and fmshareDenyWrite) = fmshareDenyWrite) then
>>>>>>> graemeg/cpstrnew
=======
    if (filemode = fmShareCompat) or ((filemode and fmshareDenyWrite) = fmshareDenyWrite) then
>>>>>>> graemeg/cpstrnew
=======
    if (filemode = fmShareCompat) or ((filemode and fmshareDenyWrite) = fmshareDenyWrite) then
>>>>>>> graemeg/cpstrnew
=======
    if (filemode = fmShareCompat) or ((filemode and fmshareDenyWrite) = fmshareDenyWrite) then
>>>>>>> origin/cpstrnew
      shflags := FILE_SHARE_READ
  else
    if ((filemode and fmshareDenyRead) = fmshareDenyRead) then
      shflags := FILE_SHARE_WRITE
  else
    if ((filemode and fmshareDenyNone) = fmshareDenyNone) then
      shflags := FILE_SHARE_READ or FILE_SHARE_WRITE or FILE_SHARE_DELETE;
  { convert filemode to filerec modes }
  case (flags and 3) of
   0 : begin
         filerec(f).mode:=fminput;
         oflags := GENERIC_READ;
       end;
   1 : begin
         filerec(f).mode:=fmoutput;
         oflags := GENERIC_WRITE;
       end;
   2 : begin
         filerec(f).mode:=fminout;
         oflags := GENERIC_WRITE or GENERIC_READ;
       end;
  end;
  oflags := oflags or NT_SYNCHRONIZE or FILE_READ_ATTRIBUTES;
  { create it ? }
  if (flags and $1000) <> 0 then
    cd := FILE_OVERWRITE_IF
  { or Append/Open ? }
  else
    cd := FILE_OPEN;
  { empty name is special }
  { console i/o not supported yet }
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
     exit;
   end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldp:=p;
  DoDirSeparators(p,pchangeable);
  SysPWideCharToNtStr(ntstr, p, 0);
=======
  SysPCharToNtStr(ntstr, p, 0);
>>>>>>> graemeg/cpstrnew
=======
  SysPCharToNtStr(ntstr, p, 0);
>>>>>>> graemeg/cpstrnew
=======
  SysPCharToNtStr(ntstr, p, 0);
>>>>>>> graemeg/cpstrnew
=======
  SysPCharToNtStr(ntstr, p, 0);
>>>>>>> origin/cpstrnew

  SysInitializeObjectAttributes(objattr, @ntstr, OBJ_INHERIT, 0, Nil);

  res := NtCreateFile(@filerec(f).handle, oflags, @objattr, @iostatus, Nil,
           FILE_ATTRIBUTE_NORMAL, shflags, cd,
           FILE_NON_DIRECTORY_FILE or FILE_SYNCHRONOUS_IO_NONALERT, Nil, 0);

  SysFreeNtStr(ntstr);

  { append mode }
  if (flags and $100 <> 0) and (res >= 0) then begin
    do_seekend(filerec(f).handle);
    filerec(f).mode := fmoutput; {fool fmappend}
  end;

  { get errors }
  if res < 0 then begin
    errno := res;
    Errno2InoutRes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FileRec(f).mode:=fmclosed;
  end;
  if oldp<>p then
    freemem(p);
=======
  end;
>>>>>>> graemeg/cpstrnew
=======
  end;
>>>>>>> graemeg/cpstrnew
=======
  end;
>>>>>>> graemeg/cpstrnew
=======
  end;
>>>>>>> origin/cpstrnew
end;
