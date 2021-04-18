{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    <What does this file>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ---------------------------------------------------------------------
    Environment variable auxiliary routines
  ---------------------------------------------------------------------}
Const
  FPC_EnvCount : Integer = -1;

Function FPCCountEnvVar(EP : PPChar) : integer;

begin
  If (FPC_EnvCount=-1) then
    begin
    FPC_EnvCount:=0;
    If (EP<>Nil) then
      While (EP^<>Nil) do
        begin
        Inc(FPC_EnvCount);
        Inc(EP);
        end;
    end;
  Result:=FPC_EnvCount;
end;

Function FPCGetEnvVarFromP(EP : PPChar; EnvVar : String) : String;

var
  hp         : ppchar;
  lenvvar,hs : string;
  eqpos      : longint;

begin
  lenvvar:=upcase(envvar);
  hp:=EP;
  Result:='';
  If (hp<>Nil) then
    while assigned(hp^) do
     begin
       hs:=strpas(hp^);
       eqpos:=pos('=',hs);
       if upcase(copy(hs,1,eqpos-1))=lenvvar then
        begin
          Result:=copy(hs,eqpos+1,length(hs)-eqpos);
          exit;
        end;
       inc(hp);
     end;
end;

Function FPCGetEnvStrFromP(EP : PPChar; Index : Integer) : String;

begin
  Result:='';
  while assigned(EP^) and (Index>1) do
    begin
      dec(Index);
      inc(EP);
    end;
  if Assigned(EP^) then
    Result:=EP^;
end;

{ these are extremely inefficient, but not much we can do about it because
  changing their result type based on the supported OS-interfaces will change
  program behaviour if they are passed as a parameter to overloaded routines }
{$ifndef SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}
Function GetEnvironmentVariable(Const EnvVar : AnsiString) : AnsiString;
begin
  result:=AnsiString(GetEnvironmentVariable(UnicodeString(EnvVar)));
end;
{$endif not SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{$ifndef SYSUTILS_HAS_UNICODESTR_ENVVAR_IMPL}
Function GetEnvironmentVariable(Const EnvVar : UnicodeString) : UnicodeString;
begin
  result:=UnicodeString(GetEnvironmentVariable(AnsiString(EnvVar)));
end;
{$endif not SYSUTILS_HAS_UNICODESTR_ENVVAR_IMPL}


{ ---------------------------------------------------------------------
    Application name
  ---------------------------------------------------------------------}

Function VendorName : String;

begin
  If Assigned(OnGetVendorName) then
    Result:=OnGetVendorName()
  else
    Result:='';
end;

Function ApplicationName : String;

begin
  If Assigned(OnGetApplicationName) then
    Result:=OnGetApplicationName()
  else
    Result:=ChangeFileExt(ExtractFileName(Paramstr(0)),'');
end;

{ ---------------------------------------------------------------------
    Default implementations for AppConfigDir implementation.
  ---------------------------------------------------------------------}

Function DGetAppConfigDir(Global : Boolean) : String;

begin
  Result:=ExcludeTrailingPathDelimiter(ExtractFilePath(ParamStr(0)));
end;

Function DGetAppConfigFile(Global : Boolean; SubDir : Boolean) : String;

begin
  Result:=IncludeTrailingPathDelimiter(GetAppConfigDir(Global));
  if SubDir then
    Result:=IncludeTrailingPathDelimiter(Result+'Config');
  Result:=Result+ApplicationName+ConfigExtension;
end;

Function GetAppConfigFile(Global : Boolean) : String;

begin
  Result:=GetAppConfigFile(Global,False);
end;

Function DGetUserDir : String;

begin
  Result:=ExtractFilePath(Paramstr(0));
end;


{ ---------------------------------------------------------------------
    Fallback implementations for AppConfigDir implementation.
  ---------------------------------------------------------------------}
{
 If a particular OS does it different:
 - set the HAS_OSCONFIG define before including sysutils.inc.
 - implement the functions.
 Default config assumes a DOS-like configuration.
}

{$ifndef HAS_OSCONFIG}
Function GetAppConfigDir(Global : Boolean) : String;

begin
  Result:=DGetAppConfigDir(Global);
end;

Function GetAppConfigFile(Global : Boolean; SubDir : Boolean) : String;

begin
  Result:=DGetAppConfigFile(Global,Subdir);
end;
{$endif}

{ ---------------------------------------------------------------------
    Fallback implementations for GetUserDir implementation.
  ---------------------------------------------------------------------}
{
 If a particular OS does it different:
 - set the HAVE_OSUSERDIR define before including sysutils.inc.
 - implement the function.
 Default makes it the application directory. Rationale is that the result
 will be used for config files, and it should exist. The application directory
 has this for sure.
}

{$ifndef HAS_OSUSERDIR}
Function GetUserDir : String;

begin
  Result:=DGetUserDir;
end;
{$endif}

{ ---------------------------------------------------------------------
  Get temporary directory name
  ---------------------------------------------------------------------}

{$ifndef HAS_TEMPDIR}
Function GetTempDir(Global : Boolean) : String;

begin
  If Assigned(OnGetTempDir) then
    Result:=OnGetTempDir(Global)
  else
    begin
    Result:=GetEnvironmentVariable('TEMP');
    If (Result='') Then
      Result:=GetEnvironmentVariable('TMP');
    end;
  if (Result<>'') then
    Result:=IncludeTrailingPathDelimiter(Result);
end;
{$endif}

Function GetTempDir : String;

begin
  Result:=GetTempDir(True);
end;

{ ---------------------------------------------------------------------
  Get temporary file name
  ---------------------------------------------------------------------}


{$ifndef HAS_TEMPFILE}
Function GetTempFileName(Const Dir,Prefix : String) : String;

Var
  I : Integer;
  Start : String;

begin
  If Assigned(OnGetTempFile) then
    Result:=OnGetTempFile(Dir,Prefix)
  else
    begin
    If (Dir='') then
      Start:=GetTempDir
    else
      Start:=IncludeTrailingPathDelimiter(Dir);
    If (Prefix='') then
      Start:=Start+'TMP'
    else
      Start:=Start+Prefix;
    I:=0;
    Repeat
      Result:=Format('%s%.5d.tmp',[Start,I]);
      Inc(I);
    Until not (FileExists(Result) or DirectoryExists(Result));
    end;
end;
{$endif}

Function GetTempFileName : String;

begin
  Result:=GetTempFileName('','');
end;

{$if not(defined(win32)) and not(defined(win64))}
Function GetTempFileName(Dir,Prefix: PChar; uUnique: DWORD; TempFileName: PChar):DWORD;

Var
  P,Buf : String;
  L : Integer;

begin
  P:=StrPas(Prefix);
  if (uUnique<>0) then
    P:=P+format('%.4x',[uUnique]);
  Buf:=GetTempFileName(StrPas(Dir),P);
  L:=Length(Buf);
  If (L>0) then
    Move(Buf[1],TempFileName^,L+1);
  if (uUnique<>0) then
    result:=uUnique
  else
    result:=1;
end;
{$endif}
