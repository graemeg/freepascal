{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2013 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function Errno2InoutRes(errno: __wasi_errno_t): Word;
begin
  case errno of
    __WASI_ERRNO_NFILE,
    __WASI_ERRNO_MFILE:
      Errno2InoutRes:=4;
    __WASI_ERRNO_NOENT:
      Errno2InoutRes:=2;
    __WASI_ERRNO_BADF:
      Errno2InoutRes:=6;
    __WASI_ERRNO_NOMEM,
    __WASI_ERRNO_FAULT:
      Errno2InoutRes:=217;
    __WASI_ERRNO_INVAL:
      Errno2InoutRes:=218;
    __WASI_ERRNO_PIPE,
    __WASI_ERRNO_INTR,
    __WASI_ERRNO_IO,
    __WASI_ERRNO_AGAIN,
    __WASI_ERRNO_NOSPC:
      Errno2InoutRes:=101;
    __WASI_ERRNO_NAMETOOLONG:
      Errno2InoutRes:=3;
    __WASI_ERRNO_ROFS,
    __WASI_ERRNO_EXIST,
    __WASI_ERRNO_NOTEMPTY,
    __WASI_ERRNO_ACCES:
      Errno2InoutRes:=5;
    __WASI_ERRNO_BUSY,
    __WASI_ERRNO_NOTDIR,        // busy, enotdir, mantis #25931
    __WASI_ERRNO_ISDIR:
      Errno2InoutRes:=5;
    else
      Errno2InoutRes:=errno;
  end;
end;

{*****************************************************************************
                          Low Level File Routines
*****************************************************************************}

function Do_IsDevice(Handle:THandle):boolean;
var
  ourfdstat: __wasi_fdstat_t;
begin
  __wasi_fd_fdstat_get(Handle,@ourfdstat);
  Do_IsDevice:=ourfdstat.fs_filetype in [__WASI_FILETYPE_BLOCK_DEVICE,__WASI_FILETYPE_CHARACTER_DEVICE];
end;
