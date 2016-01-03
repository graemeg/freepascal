{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Free Pascal development team

    Low level directory functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                           Directory Handling
*****************************************************************************}
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var
  tmpStr : array[0..255] of char;
  tmpLock: LongInt;
begin
  checkCTRLC;
  if not assigned(s) or (len=0) or (InOutRes<>0) then exit;
  tmpStr:=PathConv(strpas(s))+#0;
  tmpLock:=dosCreateDir(@tmpStr);
  if tmpLock=0 then begin
    dosError2InOut(IoErr);
    exit;
  end;
  UnLock(tmpLock);
end;

Procedure RmDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
var
  tmpStr : array[0..255] of Char;
begin
  checkCTRLC;
  if not assigned(s) or (len=0) then exit;
  if (s='.') then InOutRes:=16;
  If (s='') or (InOutRes<>0) then exit;
  tmpStr:=PathConv(strpas(s))+#0;
  if not dosDeleteFile(@tmpStr) then
    dosError2InOut(IoErr);
end;

Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
var
  tmpStr : array[0..255] of Char;
  tmpLock: LongInt;
  FIB    : PFileInfoBlock;
begin
  checkCTRLC;
  if not assigned(s) or (len=0) or (InOutRes<>0) then exit;
  tmpStr:=PathConv(strpas(s))+#0;
  tmpLock:=0;

  { Changing the directory is a pretty complicated affair }
  {   1) Obtain a lock on the directory                   }
  {   2) CurrentDir the lock                              }
  tmpLock:=Lock(@tmpStr,SHARED_LOCK);
  if tmpLock=0 then begin
    dosError2InOut(IoErr);
    exit;
  end;

  FIB:=nil;
  new(FIB);

  if (Examine(tmpLock,FIB)=True) and (FIB^.fib_DirEntryType>0) then begin
    tmpLock:=CurrentDir(tmpLock);
    if MOS_OrigDir=0 then begin
      MOS_OrigDir:=tmpLock;
      tmpLock:=0;
    end;
  end else begin
    dosError2InOut(ERROR_DIR_NOT_FOUND);
  end;

  if tmpLock<>0 then Unlock(tmpLock);
  if assigned(FIB) then dispose(FIB);
end;

procedure GetDir (DriveNr: byte; var Dir: ShortString);
var tmpbuf: array[0..255] of char;
begin
  checkCTRLC;
  Dir:='';
  if not GetCurrentDirName(tmpbuf,256) then
    dosError2InOut(IoErr)
  else
    Dir:=strpas(tmpbuf);
end;
