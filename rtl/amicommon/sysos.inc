{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{*****************************************************************************
                           AmigaOS structures
*****************************************************************************}

{$include execd.inc}
{$include timerd.inc}
{$include doslibd.inc}


{*****************************************************************************
                           AmigaOS functions
*****************************************************************************}

{$include execf.inc}
{$include doslibf.inc}

{$ifdef cpum68k}
{$if defined(amiga_v1_0_only) or defined(amiga_v1_2_only)}
{$include legacyexec.inc}
{$include legacydos.inc}
{$include legacyutil.inc}
{$endif}
{$endif}

{*****************************************************************************
                             CPU specific
*****************************************************************************}

{$ifdef cpum68k}
{$include m68kamiga.inc}
{$endif}

{*****************************************************************************
                    System Dependent Structures/Consts
*****************************************************************************}

{$packrecords default}

{$ifndef FPC_SYSTEM_HAS_STACKTOP}
{$define FPC_SYSTEM_HAS_STACKTOP}
function StackTop: pointer;
begin
  StackTop:=FindTask(nil)^.tc_SPUpper;
end;
{$endif FPC_SYSTEM_HAS_STACKTOP}

const
  CTRL_C           = 20;      { Error code on CTRL-C press }

{ Used for CTRL_C checking in I/O calls }
procedure checkCTRLC;
begin
  if BreakOn then begin
    if (SetSignal(0,0) And SIGBREAKF_CTRL_C)<>0 then begin
      { Clear CTRL-C signal }
      SetSignal(0,SIGBREAKF_CTRL_C);
      Halt(CTRL_C);
    end;
  end;
end;


{ Converts a AmigaOS dos.library error code to a TP compatible error code }
{ Based on 1.0.x Amiga RTL }
procedure dosError2InOut(errno: LongInt);
begin
  case errno of
    ERROR_BAD_NUMBER,
    ERROR_ACTION_NOT_KNOWN,
    ERROR_NOT_IMPLEMENTED : InOutRes := 1;

    ERROR_OBJECT_NOT_FOUND : InOutRes := 2;
    ERROR_DIR_NOT_FOUND :  InOutRes := 3;
    ERROR_DISK_WRITE_PROTECTED : InOutRes := 150;
    ERROR_OBJECT_WRONG_TYPE : InOutRes := 151;

    ERROR_OBJECT_EXISTS,
    ERROR_DELETE_PROTECTED,
    ERROR_WRITE_PROTECTED,
    ERROR_READ_PROTECTED,
    ERROR_OBJECT_IN_USE,
    ERROR_DIRECTORY_NOT_EMPTY : InOutRes := 5;

    ERROR_NO_MORE_ENTRIES : InOutRes := 18;
    ERROR_RENAME_ACROSS_DEVICES : InOutRes := 17;
    ERROR_DISK_FULL : InOutRes := 101;
    ERROR_INVALID_RESIDENT_LIBRARY : InoutRes := 153;
    ERROR_BAD_HUNK : InOutRes := 153;
    ERROR_NOT_A_DOS_DISK : InOutRes := 157;

    ERROR_NO_DISK,
    ERROR_DISK_NOT_VALIDATED,
    ERROR_DEVICE_NOT_MOUNTED : InOutRes := 152;

    ERROR_SEEK_ERROR : InOutRes := 156;

    ERROR_LOCK_COLLISION,
    ERROR_LOCK_TIMEOUT,
    ERROR_UNLOCK_ERROR,
    ERROR_INVALID_LOCK,
    ERROR_INVALID_COMPONENT_NAME,
    ERROR_BAD_STREAM_NAME,
    ERROR_FILE_NOT_OBJECT : InOutRes := 6;
   else
    InOutRes := errno;
  end;
end;


{ Converts an Unix-like path to Amiga-like path }
function PathConv(path: string): string; alias: 'PATHCONV'; [public];
var tmppos: longint;
begin
  { check for short paths }
  if length(path)<=2 then begin
    if (path='.') or (path='./') then path:='' else
    if path='..' then path:='/' else
    if path='*' then path:='#?';
  end else begin
    { convert parent directories }
    tmppos:=pos('../',path);
    while tmppos<>0 do begin
      { delete .. to have / as parent dir sign }
      delete(path,tmppos,2);
      tmppos:=pos('../',path);
    end;
    { convert current directories }
    tmppos:=pos('./',path);
    while tmppos<>0 do begin
      { delete ./ since we doesn't need to sign current directory }
      delete(path,tmppos,2);
      tmppos:=pos('./',path);
    end;
    { convert wildstar to #? }
    tmppos:=pos('*',path);
    while tmppos<>0 do begin
      delete(path,tmppos,1);
      insert('#?',path,tmppos);
      tmppos:=pos('*',path);
    end;
  end;
  PathConv:=path;
end;

{ Converts an Unix-like path to Amiga-like path }
function PathConv(const path: rawbytestring): rawbytestring; alias: 'PATHCONVRBS'; [public];
var tmppos: longint;
begin
  { check for short paths }
  if length(path)<=2 then begin
    if (path='.') or (path='./') then PathConv:='' else
    if path='..' then PathConv:='/' else
    if path='*' then PathConv:='#?'
    else PathConv:=path;
  end else begin
    { convert parent directories }
    PathConv:=path;
    tmppos:=pos('../',PathConv);
    while tmppos<>0 do begin
      { delete .. to have / as parent dir sign }
      delete(PathConv,tmppos,2);
      tmppos:=pos('../',PathConv);
    end;
    { convert current directories }
    tmppos:=pos('./',PathConv);
    while tmppos<>0 do begin
      { delete ./ since we doesn't need to sign current directory }
      delete(PathConv,tmppos,2);
      tmppos:=pos('./',PathConv);
    end;
    { convert wildstar to #? }
    tmppos:=pos('*',PathConv);
    while tmppos<>0 do begin
      delete(PathConv,tmppos,1);
      insert('#?',PathConv,tmppos);
      tmppos:=pos('*',PathConv);
    end;
  end;
end;

{ Thread Init/Exit Procedure support }
Type
  PThreadProcInfo = ^TThreadProcInfo;
  TThreadProcInfo = Record
    Next     : PThreadProcInfo;
    Proc     : TProcedure;
  End;

const
  threadInitProcList :PThreadProcInfo = nil;
  threadExitProcList :PThreadProcInfo = nil;

Procedure DoThreadProcChain(p: PThreadProcInfo);
Begin
  while p <> nil do
    begin
      p^.proc;
      p:=p^.next;
    end;
End;

Procedure AddThreadProc(var procList: PThreadProcInfo; Proc: TProcedure);
var
  P : PThreadProcInfo;
Begin
  New(P);
  P^.Next:=procList;
  P^.Proc:=Proc;
  procList:=P;
End;

Procedure CleanupThreadProcChain(var procList: PThreadProcInfo);
var
  P : PThreadProcInfo;
Begin
  while procList <> nil do
    begin
      p:=procList;
      procList:=procList^.next;
      dispose(p);
    end;
End;

Procedure AddThreadInitProc(Proc: TProcedure);
Begin
  AddThreadProc(threadInitProcList,Proc);
End;

Procedure AddThreadExitProc(Proc: TProcedure);
Begin
  AddThreadProc(threadExitProcList,Proc);
End;

Procedure DoThreadInitProcChain;
Begin
  DoThreadProcChain(threadInitProcList);
End;

Procedure DoThreadExitProcChain;
Begin
  DoThreadProcChain(threadExitProcList);
End;
