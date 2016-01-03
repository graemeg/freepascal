{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Ipc body implemented using direct linux syscalls

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ***********************************************************************}


Function ftok (Path : pchar; ID : cint) : TKey;
Var Info : TStat;
begin
  If fpstat(path,info)<0 then
    ftok:=-1
  else
    begin
    ftok:= (info.st_ino and $FFFF) or ((info.st_dev and $ff) shl 16) or (byte(ID) shl 24)
    end;
end;


function shmget(key: Tkey; size:size_t; flag:cint):cint;
begin
  shmget:=do_syscall (syscall_nr_SHMGET,TSysParam(key),TSysParam(size),TSysParam(flag));
end;

function shmat (shmid:cint; shmaddr:pointer; shmflg:cint): pointer;
begin
  shmat:=pointer(do_syscall(syscall_nr_SHMAT,TSysParam(shmid),TSysParam(shmaddr),TSysParam(shmflg)));
end;

function shmdt (shmaddr:pointer): cint;
begin
  shmdt:=do_syscall(syscall_nr_SHMDT,TSysParam(shmaddr));
end;

function shmctl(shmid:cint; cmd:cint; buf: pshmid_ds): cint;
begin
 shmctl:=do_syscall(syscall_nr_SHMCTL,TSysParam(shmid),TSysParam(cmd),TSysParam(buf));
end;

function msgget(key:Tkey; msgflg:cint):cint;
begin
  msgget:=do_syscall(syscall_nr_MSGGET,TSysParam(key),TSysParam(msgflg));
end;

function msgsnd(msqid:cint; msgp: pmsgbuf; msgsz: size_t; msgflg:cint):cint;
begin
  msgsnd:=do_syscall(syscall_nr_MSGSND,TSysParam(msqid),TSysParam(msgp),TSysParam(msgsz),TSysParam(msgflg));
end;

function msgrcv(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgtyp:clong; msgflg:cint):cint;
begin
msgrcv:=do_syscall(syscall_nr_MSGRCV,TSysParam(msqid),TSysParam(msgp),TSysParam(msgsz),TSysParam(msgtyp),TSysParam(msgflg));
end;

Function msgctl(msqid:cint; cmd: cint; buf: PMSQid_ds): cint;
begin
  msgctl:=do_syscall(syscall_nr_MSGCTL,TSysParam(msqid),TSysParam(cmd),TSysParam(buf));
end;

Function semget(key:Tkey; nsems:cint; semflg:cint): cint;
begin
  semget:=do_syscall (syscall_nr_SEMGET,TSysParam(key),TSysParam(nsems),TSysParam(semflg));
end;

Function semop(semid:cint; sops: psembuf; nsops:cuint): cint;
begin
  semop:=do_syscall (syscall_nr_SEMOP,TSysParam(semid),TSysParam(sops),TSysParam(nsops));
end;

Function semctl(semid:cint; semnum:cint; cmd:cint; var arg: tsemun): cint;
begin
  semctl:=do_syscall(syscall_nr_SEMCTL,TSysParam(semid),TSysParam(semnum),TSysParam(cmd),TSysParam(arg));
end;

Function semtimedop(semid:cint; sops: psembuf; nsops: cuint; timeOut: ptimespec): cint;
begin
  semtimedop:=do_syscall( syscall_nr_SEMTIMEDOP,TSysParam(semid),TSysParam(sops),TSysParam(nsops),TSysParam(timeOut));
end;


