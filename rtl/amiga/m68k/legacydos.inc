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


function CreateNewProc(tags: PTagItem): PProcess; public name '_fpc_amiga_createproc';
begin
{$warning CreateNewProc unimplemented!}
  CreateNewProc:=nil;
end;

function NameFromLock(lock  : LongInt;
                      buffer: PChar;
                      len   : LongInt): LongBool; public name '_fpc_amiga_namefromlock';
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
                    len   : LongInt): LongBool; public name '_fpc_amiga_namefromfh';
begin
{$warning NameFromFH unimplemented!}
  NameFromFH:=false;
end;

function ExamineFH(fh : BPTR;
                   fib: PFileInfoBlock): LongBool; public name '_fpc_amiga_examinefh';
begin
{$warning ExamineFH unimplemented!}
  ExamineFH:=false;
end;

function LockDosList(flags: Cardinal): PDosList; public name '_fpc_amiga_lockdoslist';
begin
{$warning LockDosList unimplemented!}
  LockDosList:=nil;
end;

procedure UnLockDosList(flags: Cardinal); public name '_fpc_amiga_unlockdoslist';
begin
{$warning UnlockDosList unimplemented!}
end;

function NextDosEntry(dlist: PDosList;
                      flags: Cardinal): PDosList; public name '_fpc_amiga_nextdosentry';
begin
{$warning NextDosEntry unimplemented!}
  NextDosEntry:=nil;
end;

// Very first dirty version of MatchFirst/Next/End)
//TODO: pattern detection, for now only simple "*" or "#?" or full name  (without patterns) is supported
function MatchFirst(pat   : PChar;
                    anchor: PAnchorPath): LongInt; public name '_fpc_amiga_matchfirst';
var
  p: PChar;
  len: LongInt;
  Path,FileN: AnsiString;
  LastSeparatorPos: Integer;
  i: Integer;
  DirLock: BPTR;
  ib: TFileInfoBlock;
  Res: LongInt;
  NChain: PAChain;
begin
  MatchFirst := -1;
  if not Assigned(Anchor) then
    Exit;
  // Search for last '/' or ':' and determine length
  Len := strlen(Pat);
  P := Pat;
  LastSeparatorPos := 0;
  for i := 1 to Len do
  begin
    if (P^ = '/') or (P^ = ':') then
    begin
      LastSeparatorPos := i;
    end;
    Inc(P);
  end;
  // copy Directory name
  SetLength(Path, LastSeparatorPos);
  Move(Pat^, Path[1], LastSeparatorPos);
  // copy filename
  SetLength(FileN, Len - LastSeparatorPos);
  P := Pat;
  Inc(P, LastSeparatorPos);
  Move(P^, FileN[1], Len - LastSeparatorPos);
  // searchpattern lowercase
  FileN := LowerCase(FileN);

  // if no path is given use the current working dir, or try to lock the dir
  if Path = '' then
    DirLock := CurrentDir(0)
  else
    DirLock := Lock(PChar(Path), ACCESS_READ);
  //
  // no dirlock found -> dir not found
  if DirLock = 0 then
  begin
    MatchFirst := -1;
    Exit;
  end;
  // examine the dir to get the fib for ExNext
  if Examine(DirLock, @ib) = 0 then
  begin
    MatchFirst := -1;
    Exit;
  end;
  // we search here directly what we need to find
  //  guess it's not meant that way but works
  repeat
    // get next dir entry
    Res := ExNext(DirLock, @ib);
    // nothing nore found -> exit
    if Res = 0 then
      break;
    // include some nifty pattern compare here? later maybe!
    if (FileN = '*') or (FileN = '#?') or (FileN = lowercase(AnsiString(ib.fib_FileName))) then
    begin
      // Match found
      // new chain
      NChain := AllocMem(SizeOf(TAChain));
      if Assigned(Anchor^.ap_First) then
      begin
        // put chain entry to the list
        Anchor^.ap_Last^.an_Child := NChain;
        NChain^.an_Parent := Anchor^.ap_Last;
        Anchor^.ap_Last := NChain;
      end
      else
      begin
        // first chain Entry
        Anchor^.ap_Last := NChain;
        Anchor^.ap_First := NChain;
        NChain^.an_Parent := Pointer(Anchor);
      end;
      // copy the fileinfoblock into the chain
      Move(ib, NChain^.an_Info, SizeOf(TFileInfoBlock));
    end;
  until Res = 0; // useless... we jump oput earlier
  //
  // if we found something
  if Assigned(Anchor^.ap_Last) then
  begin
    // set current to the first entry we found
    Anchor^.ap_Last := Anchor^.ap_First;
    // we only copy the file info block, rest is not needed for freepascal stuff
    Move(Anchor^.ap_First^.an_Info, Anchor^.ap_Info, SizeOf(TFileInfoBlock));
    // most importantly set the return code
    MatchFirst := 0;
  end;
  Unlock(DirLock);
end;

function MatchNext(anchor: PAnchorPath): LongInt; public name '_fpc_amiga_matchnext';
begin
  MatchNext := -1;
  if not Assigned(Anchor) then
    Exit;
  // was already last entry?
  if not Assigned(Anchor^.ap_Last) then
    Exit;
  // Get the next Chain Entry
  anchor^.ap_Last := anchor^.ap_Last^.an_Child;
  // check if next one is valid and copy the file infoblock, or just set the error code ;)
  if Assigned(anchor^.ap_Last) then
  begin
    Move(Anchor^.ap_Last^.an_Info, Anchor^.ap_Info, SizeOf(TFileInfoBlock));
    MatchNext := 0;
  end
  else
    MatchNext := ERROR_NO_MORE_ENTRIES;
end;

procedure MatchEnd(anchor: PAnchorPath); public name '_fpc_amiga_matchend';
var
  p, nextp: PAChain;
begin
  if Assigned(Anchor) then
  begin
    // destroy all the chain entries we created before
    p := Anchor^.ap_First;
    while Assigned(p) do
    begin
      Nextp := p^.an_Child;
      FreeMem(P);
      P := NextP;
    end;
    // reset the contents (is this needed?)
    Anchor^.ap_First := nil;
    Anchor^.ap_Last := nil;
  end;
end;

procedure NextTag(var Tag: PTagItem); inline;
begin
  if Tag^.ti_Tag = TAG_END then
    Exit;
  Inc(Tag);
  repeat
    case Tag^.ti_Tag of
      TAG_IGNORE: Inc(Tag);
      TAG_SKIP: Inc(Tag, Tag^.ti_Data);
      TAG_MORE: Tag := PTagItem(Tag^.ti_Data);
      else
        Break;
    end;
  until False;
end;

// we emulate that by the old execute command, should be enough for most cases
function SystemTagList(command: PChar;
                       tags   : PTagItem): LongInt; public name '_fpc_amiga_systemtaglist';
var
  I,O: BPTR; // in / ouput handles
  tag: PTagItem;
begin
  i := 0;
  O := 0;
  tag := Tags;
  if Assigned(tag) then
  begin
    repeat
      case Tag^.ti_Tag of
        SYS_Input: I := Tag^.ti_Data;
        SYS_Output: O := Tag^.ti_Data;
      end;
      NextTag(Tag);
    until tag^.ti_Tag = TAG_END;
  end;
  if Execute(command, I, O) then
    SystemTagList := 0
  else
    SystemTagList := -1;
end;

function GetVar(name  : PChar;
                buffer: PChar;
                size  : LongInt;
                flags : LongInt): LongInt; public name '_fpc_amiga_getvar';
begin
{$warning GetVar unimplemented!}
  GetVar:=-1;
end;

function SetFileDate(name: PChar;
                     date: PDateStamp): LongBool; public name '_fpc_amiga_setfiledate';
begin
{$warning SetFileDate unimplemented!}
  SetFileDate:=false;
end;

function SetFileSize(fh  : LongInt;
                     pos : LongInt;
                     mode: LongInt): LongInt; public name '_fpc_amiga_setfilesize';
begin
{$warning SetFileSize unimplemented!}
  SetFileSize:=-1;
end;

function GetProgramName(buf: PChar;
                        len: LongInt): LongBool; public name '_fpc_amiga_getprogramname';
var
  pr: PProcess;
  pn: PChar;
  pl: longint;
  pcli: PCommandLineInterface;
begin
  GetProgramName:=false;
  pl:=0;

  if len > 0 then
    begin
      pr:=PProcess(FindTask(nil));
      pcli:=PCommandLineInterface(pr^.pr_CLI shl 2);
      if (pcli <> nil) and (pcli^.cli_CommandName <> 0) then
        begin
          pn:=PChar(pcli^.cli_CommandName shl 2) + 1;
          pl:=Byte(pn[-1]);
          if pl > len-1 then
            pl:=len-1;
          move(pn[0],buf[0],pl);
          GetProgramName:=true;
        end;
      buf[pl]:=#0;
    end;
end;

function GetProgramDir: LongInt; public name '_fpc_amiga_getprogramdir';
var
  cmd: array[0..255] of char;
  prglock: LongInt;
begin
  { this is quite minimalistic and only covers the simplest cases }
  if GetProgramName(cmd,length(cmd)) then
    begin
      prglock:=Lock(cmd,SHARED_LOCK);
      GetProgramDir:=ParentDir(prglock);
      Unlock(prglock);
    end
  else
    GetProgramDir:=0;
end;

var
  __fpc_global_args: pchar; external name '__fpc_args';
  __fpc_global_arglen: dword; external name '__fpc_arglen';
  __fpc_args_buffer: pchar;

function GetArgStr: PChar; public name '_fpc_amiga_getargstr';
var
  len: dword;
begin
  { the string we get from pre-v2.0 OS is not empty
    or zero terminated on start, so we need to copy it
    to an alternate buffer, and zero terminate according
    to the length. This allocation will be freed on exit
    by the memory pool. }
  if __fpc_args_buffer = nil then
    begin
      len:=__fpc_global_arglen-1;
      __fpc_args_buffer:=SysAllocMem(len+1);
      if len > 0 then
        move(__fpc_global_args^,__fpc_args_buffer^,len);
      __fpc_args_buffer[len]:=#0;
    end;
  GetArgStr:=__fpc_args_buffer;
end;
