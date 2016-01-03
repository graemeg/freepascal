{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Linux IPC implemented with ipccall

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ***********************************************************************}
{ The following definitions come from linux/ipc.h }

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

{$ifdef linux_ipc32}
const
  ipc_api_select = 0;    // 16-bit gid/pid types and old structs
{$else}
const
  ipc_api_select = $100; // 32-bit gid/pid types and newer structs
{$endif}

Const
  CALL_SEMOP      = 1 +ipc_api_select;
  CALL_SEMGET     = 2 +ipc_api_select;
  CALL_SEMCTL     = 3 +ipc_api_select;
  CALL_SEMTIMEDOP = 4 +ipc_api_select;
  CALL_MSGSND     = 11+ipc_api_select;
  CALL_MSGRCV     = 12+ipc_api_select;
  CALL_MSGGET     = 13+ipc_api_select;
  CALL_MSGCTL     = 14+ipc_api_select;
  CALL_SHMAT      = 21+ipc_api_select;
  CALL_SHMDT      = 22+ipc_api_select;
  CALL_SHMGET     = 23+ipc_api_select;
  CALL_SHMCTL     = 24+ipc_api_select;

{ generic call that handles all IPC calls }

function ipccall(Call: cuint; First: cint; Second,Third : culong; P: pointer; Fifth: clong) : ptrint;
begin
 ipccall:=do_syscall(syscall_nr_ipc,TSysParam(call),TSysParam(first),TSysParam(second),TSysParam(third),TSysParam(P),TSysParam(Fifth));
end;

function shmget(key: Tkey; size:size_t; flag:cint):cint;
begin
  shmget:=ipccall (CALL_SHMGET,key,size,flag,nil,0);
end;

Function shmat (shmid:cint; shmaddr:pointer; shmflg:cint):pointer;
Var raddr : pchar;
    error : ptrint;
begin
  error:=ipccall(CALL_SHMAT,shmid,shmflg,cint(@raddr),shmaddr,0);
  If Error<0 then
    shmat:=pchar(error)
  else
    shmat:=raddr;
end;

function shmdt (shmaddr:pointer): cint;
begin
  shmdt:=ipccall(CALL_SHMDT,0,0,0,shmaddr,0);
end;

function shmctl(shmid:cint; cmd:cint; buf: pshmid_ds): cint;
begin
 shmctl:=ipccall(CALL_SHMCTL,shmid,cmd,0,buf,0);
end;

function msgget(key:Tkey; msgflg:cint):cint;
begin
  msgget:=ipccall(CALL_MSGGET,key,msgflg,0,Nil,0);
end;

function msgsnd(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgflg:cint):cint;
begin
  msgsnd:=ipccall(Call_MSGSND,msqid,msgsz,msgflg,msgp,0);
end;

function msgrcv(msqid:cint; msgp: PMSGBuf; msgsz: size_t; msgtyp:clong; msgflg:cint):cint;
Type
  TIPC_Kludge = Record
    msgp   : pmsgbuf;
    msgtyp : clong;
  end;
Var
   tmp : TIPC_Kludge;
begin
  tmp.msgp   := msgp;
  tmp.msgtyp := msgtyp;
  msgrcv:=ipccall(CALL_MSGRCV,msqid,msgsz,msgflg,@tmp,0);
end;

Function msgctl(msqid:cint; cmd: cint; buf: PMSQid_ds): cint;
begin
  msgctl:=ipccall(CALL_MSGCTL,msqid,cmd,0,buf,0);
end;

Function semget(key:Tkey; nsems:cint; semflg:cint): cint;
begin
  semget:=ipccall (CALL_SEMGET,key,nsems,semflg,Nil,0);
end;

Function semop(semid:cint; sops: psembuf; nsops:cuint): cint;
begin
  semop:=ipccall (CALL_SEMOP,semid,cint(nsops),0,Pointer(sops),0);
end;

Function semctl(semid:cint; semnum:cint; cmd:cint; var arg: tsemun): cint;
begin
  semctl:=ipccall(CALL_SEMCTL,semid,semnum,cmd,@arg,0);
end;

Function semtimedop(semid:cint; sops: psembuf; nsops: cuint; timeOut: ptimespec): cint;
begin
  semtimedop:=ipccall(CALL_SEMTIMEDOP,semid,culong(nsops),culong(0),Pointer(sops),clong(timeOut));
end;

