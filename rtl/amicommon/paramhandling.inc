{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by Marcus Sackrow,
    member of the Free Pascal development team.

    Parameter handling for Amiga-like systems

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function GetWBArgsNum: Integer;
var
  Startup: PWBStartup;
begin
  GetWBArgsNum := 0;
  Startup := nil;
  Startup := PWBStartup(AOS_wbMsg);
  if Startup <> nil then
  begin
    Result := Startup^.sm_NumArgs - 1;
  end;
end;

function GetWBArg(Idx: Integer): string;
var
  startup: PWBStartup;
  wbarg: PWBArgList;
  Path: array[0..254] of Char;
  strPath: string;
  Len: Integer;
begin
  GetWBArg := '';
  FillChar(Path[0],255,#0);
  Startup := PWBStartup(AOS_wbMsg);
  if Startup <> nil then
  begin
    //if (Idx >= 0) and (Idx < Startup^.sm_NumArgs) then
    begin
      wbarg := Startup^.sm_ArgList;
      if NameFromLock(wbarg^[Idx + 1].wa_Lock,@Path[0],255) then
      begin
        Len := 0;
        while (Path[Len] <> #0) and (Len < 254) do
          Inc(Len);
        if Len > 0 then
          if (Path[Len - 1] <> ':') and (Path[Len - 1] <> '/') then
            Path[Len] := '/';
        strPath := Path;
      end;
      Result := strPath + wbarg^[Idx + 1].wa_Name;
    end;
  end;
end;

{ Generates correct argument array on startup }
procedure GenerateArgs;
var
  ArgVLen: LongInt;

  procedure AllocArg(Idx, Len: LongInt);
  var
    i, OldArgVLen : LongInt;
  begin
    if Idx >= ArgVLen then
    begin
      OldArgVLen := ArgVLen;
      ArgVLen := (Idx + 8) and (not 7);
      SysReAllocMem(Argv, Argvlen * SizeOf(Pointer));
      for i := OldArgVLen to ArgVLen - 1 do
        ArgV[i]:=nil;
    end;
    ArgV[Idx] := SysAllocMem(Succ(Len));
  end;

var
  Count: Word;
  Start: Word;
  Ende: Word;
  LocalIndex: Word;
  i: Integer;
  P : PChar;
  Temp : AnsiString;
  InQuotes: boolean;
begin
  P := GetArgStr;
  ArgVLen := 0;

  { Set argv[0] }
  Temp := ParamStr(0);
  AllocArg(0, Length(Temp));
  Move(Temp[1], Argv[0]^, Length(Temp));
  Argv[0][Length(Temp)] := #0;

  { check if we're started from Workbench }
  if AOS_wbMsg <> nil then
  begin
    ArgC := GetWBArgsNum + 1;
    for i := 1 to ArgC - 1 do
    begin
      Temp := GetWBArg(i);
      AllocArg(i, Length(Temp));
      Move(Temp[1], Argv[i]^, Length(Temp));
      Argv[i][Length(Temp)] := #0;
    end;
    Exit;
  end;

  InQuotes := False;
  { Handle the other args }
  Count := 0;
  { first index is one }
  LocalIndex := 1;
  while (P[Count] <> #0) do
  begin
    while (p[count]=' ') or (p[count]=#9) or (p[count]=LineEnding) do
      Inc(count);
    if p[count] = '"' then
    begin
      inQuotes := True;
      Inc(Count);
    end;
    start := count;
    if inQuotes then
    begin
      while (p[count]<>#0) and (p[count]<>'"') and (p[count]<>LineEnding) do
      begin
        Inc(Count)
      end;
    end else
    begin
      while (p[count]<>#0) and (p[count]<>' ') and (p[count]<>#9) and (p[count]<>LineEnding) do
        inc(count);
    end;
    ende := count;
    if not inQuotes then
    begin
      while (p[start]=' ') and (Start < Ende) do
        Inc(Start)
    end;
    if (ende-start>0) then
    begin
      allocarg(localindex,ende-start);
      move(p[start],argv[localindex]^,ende-start);
      argv[localindex][ende-start]:=#0;
      if inQuotes and (argv[localindex][(ende-start) - 1] = '"') then
        argv[localindex][(ende-start)-1] := #0;
      inc(localindex);
    end;
    if inQuotes and (p[count] = '"') then
      Inc(Count);
    inQuotes := False;
  end;
  argc:=localindex;
end;

function GetProgDir: string;
var
  s1: string;
  alock: LongInt;
  counter: Byte;
begin
  GetProgDir := '';
  FillChar(s1, 255, #0);
  { GetLock of program directory }

  alock := GetProgramDir;
  if alock <> 0 then
  begin
    if NameFromLock(alock, @s1[1], 255) then
    begin
      Counter := 1;
      while (s1[Counter] <> #0) and (Counter <> 0) do
        Inc(Counter);
      s1[0] := char(Counter - 1);
      GetProgDir := s1;
    end;
  end;
end;

function GetProgramName: string;
{ Returns ONLY the program name }
var
  s1: string;
  Counter: Byte;
begin
  GetProgramName := '';
  FillChar(s1, 255, #0);
  if GetProgramName(@s1[1], 255) then
  begin
    { now check out and assign the length of the string }
    Counter := 1;
    while (s1[Counter] <> #0) and (Counter <> 0) do
      Inc(Counter);
    s1[0] := char(Counter - 1);

    { now remove any component path which should not be there }
    for Counter := Length(s1) downto 1 do
      if (s1[Counter] = '/') or (s1[Counter] = ':') then
        break;
    { readjust counterv to point to character }
    if Counter <> 1 then
      Inc(Counter);

    GetProgramName := Copy(s1, Counter, Length(s1));
  end;
end;


{*****************************************************************************
                             ParamStr
*****************************************************************************}

{ number of args }
function ParamCount: LongInt;
begin
  if AOS_wbMsg <> nil then
    ParamCount := GetWBArgsNum
  else
    ParamCount := argc - 1;
end;

{ argument number l }
function ParamStr(l: LongInt): string;
var
  s1: string;
begin
  ParamStr := '';
  if AOS_wbMsg <> nil then
  begin
    ParamStr := GetWBArg(l);
  end
  else
  begin
    if l = 0 then
    begin
      s1 := GetProgDir;
      if length(s1) > 0 then
      begin
        if s1[Length(s1)] = ':' then
          paramstr := s1 + GetProgramName
        else
          paramstr:=s1+'/'+GetProgramName;
      end
      else
        paramstr:=GetProgramName;
    end
    else
    begin
      if (l > 0) and (l + 1 <= argc) then
        ParamStr := StrPas(argv[l]);
    end;
 end;
end;
