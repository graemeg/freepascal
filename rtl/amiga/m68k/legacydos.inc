{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 Karoly Balogh, Free Pascal Development team

    Amiga dos.library legacy (OS 1.x/2.x) support functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  This unit implements some missing functions of OS 1.x (and some OS 2.x)
  dos.library, so the legacy OS support can be implemented with minimal
  changes to the normal system unit and common Amiga-like code

  Please note that this code doesn't aim to be API feature complete, just
  functional enough for the RTL code.
}


function CreateNewProc(tags: PTagItem): PProcess;
begin
{$warning CreateNewProc unimplemented!}
  CreateNewProc:=nil;
end;

function NameFromLock(lock  : LongInt;
                      buffer: PChar;
                      len   : LongInt): LongBool;
var
  fib_area: array[1..sizeof(TFileInfoBlock) + sizeof(longint)] of byte;
  fib: pfileinfoblock;
  namelen: longint;
  blen: longint;
begin
  NameFromLock:=false;
  if len <= 0 then
    exit;

  if (lock = 0) and (len >= 5) then
    begin
      buffer:='SYS:';
      NameFromLock:=true;
      exit;
    end;

  fib:=align(@fib_area[1],sizeof(longint));
  buffer[0]:=#0;
  dec(len); // always preserve one byte for zero term
  blen:=0;
  repeat
    if Examine(lock,fib) <> 0 then
      begin
        namelen:=strlen(@fib^.fib_FileName[0]);
        if (namelen+1) > (len-blen) then
          exit;

        move(buffer[0],buffer[namelen+1],blen);
        move(fib^.fib_FileName[0],buffer[0],namelen);
        lock:=ParentDir(lock);
        if lock = 0 then
          buffer[namelen]:=':'
        else
          buffer[namelen]:='/';
        inc(blen,namelen+1);
        buffer[blen]:=#0;
      end
    else
      exit;
  until lock = 0;

  if buffer[blen-1]='/' then
    buffer[blen-1]:=#0;

  NameFromLock:=true;
end;

function NameFromFH(fh    : BPTR;
                    buffer: PChar;
                    len   : LongInt): LongBool;
begin
{$warning NameFromFH unimplemented!}
  NameFromFH:=false;
end;

function ExamineFH(fh : BPTR;
                   fib: PFileInfoBlock): LongBool;
begin
{$warning ExamineFH unimplemented!}
  ExamineFH:=false;
end;

function LockDosList(flags: Cardinal): PDosList;
begin
{$warning LockDosList unimplemented!}
  LockDosList:=nil;
end;

procedure UnLockDosList(flags: Cardinal);
begin
{$warning UnlockDosList unimplemented!}
end;

function NextDosEntry(dlist: PDosList;
                      flags: Cardinal): PDosList;
begin
{$warning NextDosEntry unimplemented!}
  NextDosEntry:=nil;
end;

function MatchFirst(pat   : PChar;
                    anchor: PAnchorPath): LongInt;
begin
{$warning MatchFirst unimplemented!}
  MatchFirst:=-1;
end;

function MatchNext(anchor: PAnchorPath): LongInt;
begin
{$warning MatchNext unimplemented!}
  MatchNext:=-1;
end;

procedure MatchEnd(anchor: PAnchorPath);
begin
{$warning MatchEnd unimplemented!}
end;

function SystemTagList(command: PChar;
                       tags   : PTagItem): LongInt;
begin
{$warning SystemTagList unimplemented!}
  SystemTagList:=-1;
end;

function GetVar(name  : PChar;
                buffer: PChar;
                size  : LongInt;
                flags : LongInt): LongInt;
begin
{$warning GetVar unimplemented!}
  GetVar:=-1;
end;

function SetFileDate(name: PChar;
                     date: PDateStamp): LongBool;
begin
{$warning SetFileDate unimplemented!}
  SetFileDate:=false;
end;

function SetFileSize(fh  : LongInt;
                     pos : LongInt;
                     mode: LongInt): LongInt;
begin
{$warning SetFileSize unimplemented!}
  SetFileSize:=-1;
end;


function GetProgramDir: LongInt;
begin
{$warning GetProgramDir unimplemented!}
  GetProgramDir:=0;
end;


function GetProgramName(buf: PChar;
                        len: LongInt): LongBool;
begin
{$warning GetProgramName unimplemented!}
  GetProgramName:=false;
end;


var
  __fpc_global_args: pchar; external name '__fpc_args';

function GetArgStr: PChar;
begin
  GetArgStr:=__fpc_global_args;
end;
