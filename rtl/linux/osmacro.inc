{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    The "linux" posixy macro's that are used both in the Baseunix unit as the
    system unit. Not aliased via public names because I want these to be
    inlined as much as possible in the future.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function FpS_ISDIR(m : TMode): boolean;

begin
  FpS_ISDIR:=((m and S_IFMT) = S_IFDIR);
end;

function FpS_ISCHR(m : TMode): boolean;
begin
  FpS_ISCHR:=((m and S_IFMT) = S_IFCHR);
end;

function FpS_ISBLK(m : TMode): boolean;
begin
 FpS_ISBLK:=((m and S_IFMT) = S_IFBLK);
end;

function FpS_ISREG(m : TMode): boolean;
begin
 FpS_ISREG:=((m and S_IFMT) = S_IFREG);
end;

function FpS_ISFIFO(m : TMode): boolean;
begin
 FpS_ISFIFO:=((m and S_IFMT) = S_IFIFO);
end;

Function FPS_ISLNK(m:TMode):boolean;

begin
 FPS_ISLNK:=((m and S_IFMT) = S_IFLNK);
end;

Function FPS_ISSOCK(m:TMode):boolean;

begin
 FPS_ISSOCK:=((m and S_IFMT) = S_IFSOCK);
end;

function wifexited(status : cint): boolean;
begin
 wifexited:=(status AND $7f) =0;
end;

function wexitstatus(status : cint): cint;
begin
 wexitstatus:=(status and $FF00) shr 8;
end;

function wstopsig(status : cint): cint;
begin
 wstopsig:=(status and $FF00) shr 8;
end;

const wstopped=127;

function wifsignaled(status : cint): boolean;
begin
 wifsignaled:=((status and $FF)<>wstopped) and ((status and 127)<>0);
end;

function wtermsig(status : cint):cint;

begin
 wtermsig:=cint(status and 127);
end;


{$ifdef FPC_USE_LIBC}
function __fxstat(__ver,filedes : cint; var __stat: tstat): cint; cdecl; external clib name '__fxstat'+suffix64bit;
function __xstat(__ver : cint;FileName: PChar; var __stat: tstat): cint; cdecl; external clib name '__xstat'+suffix64bit;
function __lxstat(__ver : cint; FileName: PChar; var __stat: tstat): cint; cdecl; external clib name '__lxstat'+suffix64bit;
{
function __fxstat64(__ver : cint; FileDes: Integer; var __stat: tstat): cint; cdecl; external clib name '__fxstat64';
function __xstat64(__ver : cint; FileName: PChar; var __stat: tstat): cint; cdecl; external clib name '__xstat64';
function __lxstat64(__ver : cint; FileName: PChar; var __stat: tstat): cint; cdecl; external clib name '__lxstat64';
function __xmknod(__ver : cint; Pathname: PChar; Mode: mode_t; var Device: dev_t): cint; cdecl; external clib name '__xmknod';
}
{$define FPC_HAS_BASEUNIX_STAT}
function  FpFstat(fd : cint; var sb : stat): cint;inline;
  begin
    FpFstat:=__fxstat(_STAT_VER,fd,sb);
  end;


function  fpLstat(path:pchar;Info:pstat):cint;inline;
  begin
    fpLstat:=__lxstat(_STAT_VER,path,Info^);
  end;


function  FpStat(path: pchar; var buf : stat): cint;inline;
  begin
    FpStat:=__xstat(_STAT_VER,path,buf);
  end;

{$endif FPC_USE_LIBC}
