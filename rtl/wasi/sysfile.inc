{
    This file is part of the Free Pascal run time library.

    Main OS dependant body of the system unit, loosely modelled
    after POSIX.  *BSD version (Linux version is near identical)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

procedure Do_Close(Handle:thandle);
var
  res: __wasi_errno_t;
begin
  repeat
    res:=__wasi_fd_close(Handle);
  until (res=__WASI_ERRNO_SUCCESS) or (res<>__WASI_ERRNO_INTR);
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
end;

procedure Do_Erase(p: pchar; pchangeable: boolean);
var
  fd: __wasi_fd_t;
  pr: PChar;
  res: __wasi_errno_t;
begin
  if not ConvertToFdRelativePath(p,fd,pr) then
    exit;
  res:=__wasi_path_unlink_file(fd,pr,StrLen(pr));
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
  FreeMem(pr);
end;

procedure do_truncate (handle:thandle;fpos:int64);
var
  res: __wasi_errno_t;
begin
  res:=__wasi_fd_filestat_set_size(handle,fpos);
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
end;

procedure Do_Rename(p1,p2:pchar; p1changeable, p2changeable: boolean);
var
  fd1,fd2: __wasi_fd_t;
  pr1,pr2: PChar;
  res: __wasi_errno_t;
begin
  if not ConvertToFdRelativePath(p1,fd1,pr1) then
    exit;
  if not ConvertToFdRelativePath(p2,fd2,pr2) then
  begin
    FreeMem(pr1);
    exit;
  end;
  res:=__wasi_path_rename(fd1,pr1,StrLen(pr1),fd2,pr2,StrLen(pr2));
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
  FreeMem(pr1);
  FreeMem(pr2);
end;

function Do_Write(Handle:thandle;Addr:Pointer;Len:Longint):longint;
var
  our_iov: __wasi_ciovec_t;
  our_nwritten: longint;
  res: __wasi_errno_t;
begin
  repeat
    our_iov.buf := Addr;
    our_iov.buf_len := Len;
    res:=__wasi_fd_write(Handle, @our_iov, 1, @our_nwritten);
  until (res=__WASI_ERRNO_SUCCESS) or ((res<>__WASI_ERRNO_INTR) and (res<>__WASI_ERRNO_AGAIN));
  if res=__WASI_ERRNO_SUCCESS then
    begin
      Do_Write:=our_nwritten;
      InOutRes:=0;
    end
  else
    begin
      Do_Write:=0;
      InOutRes:=Errno2InoutRes(res);
    end;
end;

function Do_Read(Handle:thandle;Addr:Pointer;Len:Longint):Longint;
var
  our_iov: __wasi_iovec_t;
  our_nread: __wasi_size_t;
  res: __wasi_errno_t;
begin
  repeat
    our_iov.buf:=Addr;
    our_iov.buf_len:=Len;
    __wasi_fd_read(Handle,@our_iov,1,@our_nread);
  until (res=__WASI_ERRNO_SUCCESS) or ((res<>__WASI_ERRNO_INTR) and (res<>__WASI_ERRNO_AGAIN));
  if res=__WASI_ERRNO_SUCCESS then
    begin
      Do_Read:=our_nread;
      InOutRes:=0;
    end
  else
    begin
      Do_Read:=0;
      InOutRes:=Errno2InoutRes(res);
    end;
end;

function Do_FilePos(Handle: thandle):Int64;
var
  res: __wasi_errno_t;
  fpos:__wasi_filesize_t;
begin
  res:=__wasi_fd_tell(Handle,@fpos);
  if res=__WASI_ERRNO_SUCCESS then
  begin
    InOutRes:=0;
    Do_FilePos:=fpos;
  end
  else
  begin
    InOutRes:=Errno2InoutRes(res);
    Do_FilePos:=-1;
  end;
end;

procedure Do_Seek(Handle:thandle;Pos:Int64);
var
  res: __wasi_errno_t;
  newoffset: __wasi_filesize_t;
begin
  res:=__wasi_fd_seek(Handle,Pos,__WASI_WHENCE_SET,@newoffset);
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
end;

function Do_Seekend(Handle:thandle):Int64;
var
  res: __wasi_errno_t;
  newoffset: __wasi_filesize_t;
begin
  res:=__wasi_fd_seek(Handle,0,__WASI_WHENCE_END,@newoffset);
  if res=__WASI_ERRNO_SUCCESS then
  begin
    InOutRes:=0;
    Do_Seekend:=newoffset;
  end
  else
  begin
    InOutRes:=Errno2InoutRes(res);
    Do_Seekend:=-1;
  end;
end;

function Do_FileSize(Handle:thandle):Int64;
var
  res: __wasi_errno_t;
  buf: __wasi_filestat_t;
begin
  res:=__wasi_fd_filestat_get(Handle, @buf);
  if res=__WASI_ERRNO_SUCCESS then
  begin
    InOutRes:=0;
    Do_FileSize:=buf.size;
  end
  else
  begin
    InOutRes:=Errno2InoutRes(res);
    Do_FileSize:=0;
  end;
end;

procedure Do_Open(var f; p: pchar; flags: longint; pchangeable: boolean);
{
  FileRec and textrec have both Handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}
var
  oflags : __wasi_oflags_t = 0;
  fs_rights_base: __wasi_rights_t = 0;
  fdflags: __wasi_fdflags_t = 0;
  ourfd: __wasi_fd_t;
  res: __wasi_errno_t;
  pr: PChar;
  fd: __wasi_fd_t;
Begin
{ close first if opened }
  if ((flags and $10000)=0) then
   begin
     case FileRec(f).mode of
      fminput,fmoutput,fminout : Do_Close(FileRec(f).Handle);
      fmclosed : ;
     else
      begin
        inoutres:=102; {not assigned}
        exit;
      end;
     end;
   end;
{ reset file Handle }
  FileRec(f).Handle:=UnusedHandle;
{ We do the conversion of filemodes here, concentrated on 1 place }
  case (flags and 3) of
   0 : begin
         fs_rights_base :=__WASI_RIGHTS_FD_READ or __WASI_RIGHTS_FD_FILESTAT_GET or __WASI_RIGHTS_FD_SEEK or __WASI_RIGHTS_FD_TELL;
         FileRec(f).mode:=fminput;
       end;
   1 : begin
         fs_rights_base :=__WASI_RIGHTS_FD_WRITE or __WASI_RIGHTS_FD_FILESTAT_GET or __WASI_RIGHTS_FD_SEEK or __WASI_RIGHTS_FD_TELL;
         FileRec(f).mode:=fmoutput;
       end;
   2 : begin
         fs_rights_base :=__WASI_RIGHTS_FD_READ or __WASI_RIGHTS_FD_WRITE or __WASI_RIGHTS_FD_FILESTAT_GET or __WASI_RIGHTS_FD_SEEK or __WASI_RIGHTS_FD_TELL;
         FileRec(f).mode:=fminout;
       end;
  end;
  if (flags and $1000)=$1000 then
   oflags:=oflags or (__WASI_OFLAGS_CREAT or __WASI_OFLAGS_TRUNC)
  else
   if (flags and $100)=$100 then
    fdflags:=fdflags or __WASI_FDFLAGS_APPEND;
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
     exit;
   end;
  if not ConvertToFdRelativePath(p,fd,pr) then
    exit;
{ real open call }
  repeat
    res:=__wasi_path_open(fd,
                          0,
                          pr,
                          strlen(pr),
                          oflags,
                          fs_rights_base,
                          fs_rights_base,
                          fdflags,
                          @ourfd);
  until (res=__WASI_ERRNO_SUCCESS) or (res<>__WASI_ERRNO_INTR);
  {if (res=__WASI_ERRNO_ROFS) and ((OFlags and O_RDWR)<>0) then
   begin
     Oflags:=Oflags and not(O_RDWR);
     repeat
       FileRec(f).Handle:=Fpopen(p,oflags,MODE_OPEN);
     until (FileRec(f).Handle<>-1) or (geterrno<>ESysEINTR);
   end;}
  If res<>__WASI_ERRNO_SUCCESS Then
    begin
      FileRec(f).mode:=fmclosed;
      InOutRes:=Errno2InoutRes(res);
    end
  else
    begin
      FileRec(f).Handle:=ourfd;
      InOutRes:=0;
    end;
  FreeMem(pr);
end;
