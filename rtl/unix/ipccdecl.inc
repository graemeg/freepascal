{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Marco van de Voort
    member of the Free Pascal development team

    CDecl calls for ipc unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


function cshmget(key:key_t;size:size_t;shmflg:cint):cint; cdecl; external;
Function cshmat (shmid:cint; shmaddr:pointer; shmflg:cint):pointer; cdecl;external;
Function cshmdt (shmaddr:pointer):cint;  cdecl; external;
Function cshmctl(shmid:cint; cmd:cint; buf: pshmid_ds): cint; cdecl; external;
Function csemget(key:Tkey; nsems:cint; semflg:cint): cint;  cdecl; external;
Function csemop(semid:cint; sops: psembuf; nsops: cuint): cint;  cdecl; external;
Function csemctl(semid:cint; semnum:cint; cmd:cint; var arg: tsemun): cint;  cdecl; external;
Function cmsgget(key: TKey; msgflg:cint):cint;  cdecl; external;
Function cmsgsnd(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgflg:cint): cint;  cdecl; external;
Function cmsgrcv(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgtyp:cint; msgflg:cint):cint;  cdecl; external;
Function cmsgctl(msqid:cint; cmd: cint; buf: PMSQid_ds): cint;  cdecl; external;

function cftok(path:Pchar; id:cint):key_t; cdecl; external;

function ftok(path:Pchar; id:cint):key_t;

var st:stat;

begin
  ftok:=cftok(path,id);
end;

function shmget(key:key_t;size:size_t;flag:cint):cint;
begin
  shmget:=cshmget(key, size, flag);
end;

Function shmat (shmid:cint; shmaddr:pointer; shmflg:cint):pointer;
begin
  shmat:=cshmat(shmid, shmaddr, shmflg);
end;

Function shmdt (shmaddr:pointer):cint;

begin
  shmdt:=cshmdt(shmaddr);
end;

Function shmctl(shmid:cint; cmd:cint; buf: pshmid_ds): cint;

begin
  shmctl:= cshmctl(shmid, cmd, buf);
end;

Function semget(key:Tkey; nsems:cint; semflg:cint): cint;
begin
  semget:=csemget(key, nsems, semflg);
end;

Function semop(semid:cint; sops: psembuf; nsops: cuint): cint;
begin
  semop:=csemop(semid, sops, nsops);
end;

Function semctl(semid:cint; semnum:cint; cmd:cint; var arg: tsemun): cint;
begin
  semctl:=csemctl(semid, semnum, cmd, arg);
end;

Function msgget(key: TKey; msgflg:cint):cint;
begin
  msgget:=cmsgget(key, msgflg);
end;

Function msgsnd(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgflg:cint): cint;
begin
  msgsnd:=cmsgsnd(msqid, msgp, msgsz, msgflg);
end;

Function msgrcv(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgtyp:cint; msgflg:cint):cint;
begin
  msgrcv:=cmsgrcv(msqid, msgp, msgsz, msgtyp, msgflg);
end;

Function msgctl(msqid:cint; cmd: cint; buf: PMSQid_ds): cint;
begin
 msgctl:=cmsgctl( msqid, cmd, buf);
end;

