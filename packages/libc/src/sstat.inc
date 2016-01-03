{ ---------------------------------------------------------------------
    Macros from sys/stat.h
  ---------------------------------------------------------------------}

function __S_ISTYPE(mode,mask : __mode_t) : boolean;
begin
  __S_ISTYPE:=(mode and __S_IFMT) = mask;
end;

function S_ISDIR(mode : __mode_t) : boolean;
begin
   S_ISDIR:=__S_ISTYPE(mode,__S_IFDIR);
end;

function S_ISCHR(mode : __mode_t) : boolean;
begin
   S_ISCHR:=__S_ISTYPE(mode,__S_IFCHR);
end;

function S_ISBLK(mode : __mode_t) : boolean;
begin
   S_ISBLK:=__S_ISTYPE(mode,__S_IFBLK);
end;

function S_ISREG(mode : __mode_t) : boolean;
begin
   S_ISREG:=__S_ISTYPE(mode,__S_IFREG);
end;

function S_ISFIFO(mode : __mode_t) : boolean;
begin
   S_ISFIFO:=__S_ISTYPE(mode,__S_IFIFO);
end;

function S_ISLNK(mode : __mode_t) : boolean;
begin
   S_ISLNK:=__S_ISTYPE(mode,__S_IFLNK);
end;


function S_ISSOCK(mode : __mode_t) : boolean;
begin
   S_ISSOCK:=__S_ISTYPE(mode,__S_IFSOCK);
end;


function fstat(__fd:longint; __buf:Pstat):longint;
  begin
    __fxstat(_STAT_VER,__fd,__buf);
  end;


function lstat(__file:Pchar; __buf:Pstat):longint;
  begin
    __lxstat(_STAT_VER,__file,__buf);
  end;


function stat(__file:Pchar; __buf:Pstat):longint;
  begin
    __xstat(_STAT_VER,__file,__buf);
  end;


function fstat64(__fd:longint; __buf:Pstat64):longint;
  begin
    __fxstat64(_STAT_VER,__fd,__buf);
  end;


function lstat64(__file:Pchar; __buf:Pstat64):longint;
  begin
    __lxstat64(_STAT_VER,__file,__buf);
  end;


function stat64(__file:Pchar; __buf:Pstat64):longint;
  begin
    __xstat64(_STAT_VER,__file,__buf);
  end;


function stat(__file:Pchar; var __buf:_stat):longint;
  begin
    __xstat(_STAT_VER,__file,__buf);
  end;


function fstat(__fd:longint; var __buf:_stat):longint;
  begin
    __fxstat(_STAT_VER,__fd,__buf);
  end;


function stat64(__file:Pchar; var __buf: _stat64):longint;
  begin
    __xstat64(_STAT_VER,__file,__buf);
  end;


function fstat64(__fd:longint; var __buf: _stat64):longint;
  begin
    __fxstat64(_STAT_VER,__fd,__buf);
  end;


function lstat(__file:Pchar; var __buf:_stat):longint;
  begin
    __lxstat(_STAT_VER,__file,__buf);
  end;


function lstat64(__file:Pchar; var __buf:_stat64):longint;
  begin
    __lxstat64(_STAT_VER,__file,__buf);
  end;
