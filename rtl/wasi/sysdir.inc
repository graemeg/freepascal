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


{*****************************************************************************
                           Directory Handling
*****************************************************************************}

procedure Do_MkDir(s: rawbytestring);
var
  fd: __wasi_fd_t;
  pr: PChar;
  res: __wasi_errno_t;
begin
  if not ConvertToFdRelativePath(PChar(s),fd,pr) then
    exit;
  res:=__wasi_path_create_directory(fd,pr,StrLen(pr));
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
  FreeMem(pr);
end;

procedure Do_RmDir(s: rawbytestring);
var
  fd: __wasi_fd_t;
  pr: PChar;
  res: __wasi_errno_t;
begin
  if not ConvertToFdRelativePath(PChar(s),fd,pr) then
    exit;
  res:=__wasi_path_remove_directory(fd,pr,StrLen(pr));
  if res=__WASI_ERRNO_SUCCESS then
    InOutRes:=0
  else
    InOutRes:=Errno2InoutRes(res);
  FreeMem(pr);
end;

procedure do_ChDir(s: rawbytestring);
begin
  DebugWriteLn('do_ChDir');
end;

procedure do_getdir(drivenr : byte;var dir : rawbytestring);
begin
  if drivenr=0 then
    drivenr:=current_drive;
  if (drivenr<=drives_count) and (current_dirs[drivenr]<>nil) then
    dir:=current_dirs[drivenr]
  else
    InoutRes:=15;
end;
