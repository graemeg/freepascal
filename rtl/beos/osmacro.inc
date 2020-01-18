{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    The *BSD POSIX macro's that are used both in the Baseunix unit as the
    system unit. Not aliased via public names because I want these to be
    inlined as much as possible in the future.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function FPS_ISDIR(m : TMode): boolean;

begin
 FPS_ISDIR:=((m and S_IFMT) = S_IFDIR);
end;

function FPS_ISCHR(m : TMode): boolean;
begin
 FPS_ISCHR:=((m and S_IFMT) = S_IFCHR);
end;

function FPS_ISBLK(m : TMode): boolean;
begin
 FPS_ISBLK:=((m and S_IFMT) = S_IFBLK);
end;

function FPS_ISREG(m : TMode): boolean;
begin
 FPS_ISREG:=((m and S_IFMT) = S_IFREG);
end;

function FPS_ISFIFO(m : TMode): boolean;
begin
 FPS_ISFIFO:=((m and S_IFMT) = S_IFIFO);
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
 wifexited:=(status AND (not $FF)) = 0;
end;

function wexitstatus(status : cint): cint;
begin
 wexitstatus:=status AND $FF;
end;

// const wstopped=127;

function wifsignaled(status : cint): boolean;
begin
 wifsignaled := ((status shr 8) AND $FF) <> 0;
end;

function wtermsig(status : cint):cint;
begin
 wtermsig:= ((status shr 8) AND $FF);
end;

function wstopsig(status : cint): cint;
begin
 wstopsig:=((status shr 16) AND $FF);
end;





