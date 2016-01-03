{$mode objfpc}
{$H+}
{
    This file is part of Free Pascal Build tools
    Copyright (c) 2005 by Michael Van Canneyt

    Create a configuration file

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program fpcmkcfg;

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  fpmkunit,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SysUtils,
  Classes,
{$ifdef unix}
  baseunix,
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  fpTemplate;
=======
  fpTemplate,
  process;
>>>>>>> graemeg/cpstrnew
=======
  fpTemplate,
  process;
>>>>>>> graemeg/cpstrnew
=======
  fpTemplate,
  process;
>>>>>>> graemeg/cpstrnew
=======
  fpTemplate,
  process;
>>>>>>> origin/cpstrnew

{
  The inc files must be built from a template with the data2inc
  command.
  data2inc -b -s fpc.cft fpccfg.inc DefaultConfig
  data2inc -b -s fpinc.ini fpini.inc fpini
  data2inc -b -s fpinc.cfg fpcfg.inc fpcfg
  data2inc -b -s fppkg.cfg fppkg.inc fppkg
  data2inc -b -s default.cft default.inc fppkg_default
}

{$i fpccfg.inc}
{$i fpini.inc}
{$i fpcfg.inc}
{$i fppkg.inc}
{$i default.inc}

Const
  BuildVersion={$I %FPCVERSION%};
  BuildTarget={$I %FPCTARGET%};
  BuildOSTarget={$I %FPCTARGETOS%};
{$ifdef unix}
  ExeExt = '';
{$else unix}
  ExeExt = '.exe';
{$endif unix}


Resourcestring
  SUsage00  = 'Usage: %s [options]';
  SUsage10  = 'Where options is one or more of';
  SUSage20  = '  -t filename   Template file name. Default is built-in';
  SUSage30  = '  -o filename   Set output file. Default is standard output.';
  SUsage40  = '  -d name=value define name=value pair.';
  SUsage50  = '  -h            show this help and exit.';
  SUsage60  = '  -u name       remove name from list of name/value pairs.';
//  SUsage70  = '  -l filename   read name/value pairs from filename';
  SUsage70  = '  -m            show builtin macros and exit.';
  SUsage80  = '  -b            show builtin template and exit.';
  SUsage84  = '  -s            skip the creation of a backup-file.';
  SUsage87  = '  -p            force directory creation.';
  SUsage90  = '  -v            be verbose.';
  Susage100 = '  -0            use built in fpc.cfg template (default)';
  Susage110 = '  -1            use built in fp.cfg template';
  Susage120 = '  -2            use built in fp.ini template';
  Susage130 = '  -3            use built in fppkg.cfg template';
  Susage140 = '  -4            use built in fppkg default compiler template';
  SErrUnknownOption   = 'Error: Unknown option.';
  SErrArgExpected     = 'Error: Option "%s" requires an argument.';
  SErrIncompletePair  = 'Error: Incomplete name-value pair "%s".';
  SErrNoSuchFile      = 'Error: File "%s" does not exist.';
  SErrNoSuchDirectory = 'Error: Directory of file "%s" does not exists. User -p to force creation.';
  SErrBackupFailed    = 'Error: Backup of file "%s" to "%s" failed.';
  SErrDelBackupFailed = 'Error: Delete of old backup file "%s" failed.';
  SErrCreateDirFailed = 'Error: Could not create the directory for file "%s".';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrDestDirectory   = 'Error: The output file "%s" is a directory.';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  SWarnIgnoringFile   = 'Warning: Ignoring non-existent file: ';
  SWarnIgnoringPair   = 'Warning: Ignoring wrong name/value pair: ';
  SWarngccNotFound    = 'Warning: Could not find gcc. Unable to determine the gcclib path.';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SWarnCouldNotExecute= 'Warning: Could not execute command ''%s''';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  SBackupCreated      = 'Saved old "%s" to "%s"';


Var
  Verbose : Boolean;
  SkipBackup : Boolean;
  CreateDir: Boolean;
  Cfg : TStringList;
  TemplateParser: TTemplateParser;
  TemplateFileName,
  OutputFileName : String;
  IDEBuildin : Integer;

function IsSuperUser:boolean;
begin
{$ifdef unix}
  result:=(fpGetUID=0);
{$else unix}
  result:=false;
{$endif unix}
end;


function GetDefaultLocalRepository: string;

begin
{$IFDEF Unix}
  result := '{UserDir}.fppkg'+PathDelim;
{$ELSE Unix}
  result := '{AppConfigDir}';
{$ENDIF Unix}
end;

function GetDefaultLocalBasepath: string;

begin
{$IFDEF Unix}
  result := '~/.fppkg'+PathDelim+'lib'+PathDelim+'fpc'+PathDelim+'$fpcversion';
{$ELSE Unix}
  result := '$LOCAL_APPDATA'+PathDelim+'FreePascal'+PathDelim+'fppkg';
{$ENDIF Unix}
end;

function GetDefaultCompilerConfigDir: string;

begin
{$IFDEF Unix}
  if IsSuperUser then
    result := '/etc/fppkg/'
  else
{$ENDIF}
  result := '{LocalRepository}config/';
end;

function GetDefaultNeedCrossBinutilsIfdef: string;

begin
  result := '';
  // On Darwin there is never a need for a crossbinutils prefix
  if SameText(BuildOSTarget,'Darwin') then
    result := '#IFNDEF ' + BuildOSTarget + LineEnding +
              '#DEFINE NEEDCROSSBINUTILS' + LineEnding +
              '#ENDIF'
  else if (BuildTarget = 'i386') or (BuildTarget = 'x86_64') then
    begin
    // Cross-binutils are not needed to compile for i386 on an x86_64 system
    result := '#IFNDEF CPUI386' + LineEnding +
              '#IFNDEF CPUAMD64' + LineEnding +
              '#DEFINE NEEDCROSSBINUTILS' + LineEnding +
              '#ENDIF' + LineEnding +
              '#ENDIF' + LineEnding +
              LineEnding +
              '#IFNDEF ' + BuildOSTarget + LineEnding +
              '#DEFINE NEEDCROSSBINUTILS' + LineEnding +
              '#ENDIF';
    end
  else
    result := '#DEFINE NEEDCROSSBINUTILS';
end;

function GetDefaultGCCDir: string;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  var
    OS: TOS;
    CPU: TCPU;
    s: string;

  procedure AddConditionalLinkerPath(const aCpuType: string; const ACPU: TCPU; var ConfigFileOption: string);
  var
    path: string;
    ErrS: string;
  begin
    path := GetDefaultLibGCCDir(ACPU, OS, ErrS);
    if ErrS<>'' then
      Writeln(StdErr, ErrS);
    if path <> '' then
      begin
      if ConfigFileOption<>'' then ConfigFileOption:=ConfigFileOption+LineEnding;
      ConfigFileOption := ConfigFileOption + '#ifdef ' + ACpuType + LineEnding + '-Fl' + Path + LineEnding + '#endif';
      end;
  end;

begin
  CPU := StringToCPU(BuildTarget);
  OS := StringToOS(BuildOSTarget);
  result := '';

  case OS of
    freebsd, openbsd, netbsd :
      result := '-Fl'+GetDefaultLibGCCDir(CPU, OS, S);
    linux :
       begin
       if CPU in [i386, x86_64] then
         begin
         AddConditionalLinkerPath('cpui386', i386, result);
         AddConditionalLinkerPath('cpux86_64', x86_64, result);
         end
       else if CPU in [powerpc, powerpc64] then
         begin
         AddConditionalLinkerPath('cpupowerpc', powerpc, result);
         AddConditionalLinkerPath('cpupowerpc64', powerpc64, result);
         end
       end;
    darwin :
       begin
       AddConditionalLinkerPath('cpui386', i386, result);
       AddConditionalLinkerPath('cpux86_64', x86_64, result);
       AddConditionalLinkerPath('cpupowerpc', powerpc, result);
       AddConditionalLinkerPath('cpupowerpc64', powerpc64, result);
       end
  end; {case}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var GccExecutable: string;

  function GetGccExecutable: string;
  begin
    if GccExecutable='' then
      begin
      GccExecutable := ExeSearch('gcc'+ExeExt,GetEnvironmentVariable('PATH'));
      if GccExecutable='' then
        begin
        Writeln(StdErr,SWarngccNotFound);
        GccExecutable:='-';
        end;
      end;
    if GccExecutable = '-' then
      result := ''
    else
      result := GccExecutable;
  end;

  function ExecuteProc(const CommandLine: string; ReadStdErr: boolean) : string;

  const BufSize=2048;

  var S: TProcess;
      buf: array[0..BufSize-1] of byte;
      count: integer;

  begin
    S:=TProcess.Create(Nil);
    try
      S.Commandline:=CommandLine;
      S.Options:=[poUsePipes,poWaitOnExit];
      S.execute;
      Count:=s.output.read(buf,BufSize);
      if (count=0) and ReadStdErr then
        Count:=s.Stderr.read(buf,BufSize);
      setlength(result,count);
      move(buf[0],result[1],count);
    finally
      S.Free;
    end;
  end;

  function Get4thWord(const AString: string): string;
  var p: pchar;
      spacecount: integer;
      StartWord: pchar;
  begin
    if length(AString)>6 then
      begin
      p := @AString[1];
      spacecount:=0;
      StartWord:=nil;
      while (not (p^ in [#0,#10,#13])) and ((p^<>' ') or (StartWord=nil)) do
        begin
        if p^=' ' then
          begin
          inc(spacecount);
          if spacecount=3 then StartWord:=p+1;
          end;
        inc(p);
        end;
      if StartWord<>nil then
        begin
        SetLength(result,p-StartWord);
        move(StartWord^,result[1],p-StartWord);
        end
      else
        result := '';
      end;
  end;

  function GetGccDirArch(const ACpuType, GCCParams: string) : string;
  var ExecResult: string;
      libgccFilename: string;
      gccDir: string;
  begin
    ExecResult:=ExecuteProc(GetGccExecutable+' -v '+GCCParams, True);
    libgccFilename:=Get4thWord(ExecResult);
    if libgccFilename='' then
      libgccFilename:=ExecuteProc(GetGccExecutable+' --print-libgcc-file-name '+GCCParams, False);
    gccDir := ExtractFileDir(libgccFilename);
    if gccDir='' then
      result := ''
    else if ACpuType = '' then
      result := '-Fl'+gccDir
    else
      result := '#ifdef ' + ACpuType + LineEnding + '-Fl' + gccDir + LineEnding + '#endif';
  end;

begin
  result := '';
  GccExecutable:='';
  if sametext(BuildOSTarget,'Freebsd') or sametext(BuildOSTarget,'Openbsd') then
    result := '-Fl/usr/local/lib'
  else if sametext(BuildOSTarget,'Netbsd') then
    result := '-Fl/usr/pkg/lib'
  else if sametext(BuildOSTarget,'Linux') then
    begin
    if (BuildTarget = 'i386') or (BuildTarget = 'x86_64') then
      result := GetGccDirArch('cpui386','-m32') + LineEnding +
                GetGccDirArch('cpux86_64','-m64')
    else if (BuildTarget = 'powerpc') or (BuildTarget = 'powerpc64') then
      result := GetGccDirArch('cpupowerpc','-m32') + LineEnding +
                GetGccDirArch('cpupowerpc64','-m64')
    end
  else if sametext(BuildOSTarget,'Darwin') then
    result := GetGccDirArch('cpupowerpc','-arch ppc') + LineEnding +
              GetGccDirArch('cpupowerpc64','-arch ppc64') + LineEnding +
              GetGccDirArch('cpui386','-arch i386') + LineEnding +
              GetGccDirArch('cpux86_64','-arch x86_64');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


procedure Init;

begin
  Verbose:=False;
  IDEBuildIn:=0;

  TemplateParser := TTemplateParser.Create;
  TemplateParser.StartDelimiter:='%';
  TemplateParser.EndDelimiter:='%';
  TemplateParser.Values['FPCVERSION'] := BuildVersion;
  TemplateParser.Values['FPCTARGET'] := BuildTarget;
  TemplateParser.Values['FPCTARGETOS'] := BuildOSTarget;
  TemplateParser.Values['FPCBIN'] := 'fpc';
  TemplateParser.Values['PWD'] := GetCurrentDir;
  TemplateParser.Values['BUILDDATE'] := DateToStr(Date);
  TemplateParser.Values['BUILDTIME'] := TimeToStr(Time);

  TemplateParser.Values['LOCALREPOSITORY'] := GetDefaultLocalRepository;
  TemplateParser.Values['LOCALBASEPATH'] := GetDefaultLocalBasepath;
  TemplateParser.Values['COMPILERCONFIGDIR'] := GetDefaultCompilerConfigDir;
  TemplateParser.Values['NEEDCROSSBINUTILSIFDEF'] := GetDefaultNeedCrossBinutilsIfdef;
  TemplateParser.Values['GCCLIBPATH'] := GetDefaultGCCDIR;

  Cfg:=TStringList.Create;
  Cfg.Text:=StrPas(Addr(DefaultConfig[0][1]));
end;

Procedure Done;

begin
  FreeAndNil(Cfg);
  FreeAndNil(TemplateParser);
end;

Procedure Usage;

begin
  Writeln(Format(SUsage00,[ExtractFileName(Paramstr(0))]));
  Writeln(SUsage10);
  Writeln(SUsage20);
  Writeln(SUsage30);
  Writeln(SUsage40);
  Writeln(SUsage50);
  Writeln(SUsage60);
  Writeln(SUsage70);
  Writeln(SUsage80);
  Writeln(SUsage84);
  Writeln(SUsage87);
  Writeln(SUsage90);
  Writeln(SUsage100);
  Writeln(SUsage110);
  Writeln(SUsage120);
  Writeln(SUsage130);
  Writeln(SUsage140);
  Halt(1);
end;

Procedure UnknownOption(Const S : String);

begin
  Writeln(SErrUnknownOption,S);
  Usage;
end;

Procedure ShowBuiltIn;

Var
  I : Integer;


begin
  For I:=0 to Cfg.Count-1 do
    Writeln(Cfg[I]);
end;


Procedure ShowBuiltInMacros;

Var
  I : Integer;

begin
  For I:=0 to TemplateParser.ValueCount-1 do
    Writeln(TemplateParser.NamesByIndex[I]+'='+TemplateParser.ValuesByIndex[I]);
end;


Procedure ProcessCommandline;

Var
  I : Integer;
  S : String;
  ShowBuiltinCommand : boolean;

  Function GetOptArg : String;

  begin
    If I=ParamCount then
      begin
      Writeln(StdErr,Format(SErrArgExpected,[S]));
      Halt(1);
      end;
    inc(I);
    Result:=ParamStr(I);
  end;

  procedure AddPair(const Value: String);
  var P: integer;
      N,V: String;
  begin
    P:=Pos('=',Value);
    If p=0 then
      begin
      Writeln(StdErr,Format(SErrIncompletePair,[Value]));
      Halt(1);
      end;
    V:=Value;
    N:=Copy(V,1,P-1);
    Delete(V,1,P);
    TemplateParser.Values[N] := V;
  end;

begin
  I:=1;
  ShowBuiltinCommand := False;
  SkipBackup := False;
  CreateDir := False;
  While( I<=ParamCount) do
    begin
    S:=Paramstr(i);
    If Length(S)<=1 then
      UnknownOption(S)
    else
      case S[2] of
        'v' : Verbose:=True;
        'h' : Usage;
        'b' : ShowBuiltinCommand := true;
        'm' : begin
              ShowBuiltinMacros;
              halt(0);
              end;
        't' : TemplateFileName:=GetOptArg;
        'd' : AddPair(GetOptArg);
        'u' : TemplateParser.Values[GetOptArg]:='';
        'o' : OutputFileName:=GetoptArg;
        's' : SkipBackup:=True;
        'p' : CreateDir:=True;
        '0' : IDEBuildin:=0;
        '1' : IDEBuildin:=1;
        '2' : IDEBuildin:=2;
        '3' : IDEBuildin:=3;
        '4' : IDEBuildin:=4;
      else
        UnknownOption(S);
      end;
    Inc(I);
    end;
  If (TemplateFileName<>'') then
    begin
    If Not FileExists(TemplateFileName) then
      begin
      Writeln(StdErr,Format(SErrNoSuchFile,[TemplateFileName]));
      Halt(1);
      end;
    Cfg.LoadFromFile(TemplateFileName);
    TemplateParser.Values['TEMPLATEFILE'] := TemplateFileName;
    end
  else
    begin
      case IDEBuildin of
        1:
           Cfg.Text:=StrPas(Addr(fpcfg[0][1]));
        2:
           Cfg.Text:=StrPas(Addr(fpini[0][1]));
        3:
           Cfg.Text:=StrPas(Addr(fppkg[0][1]));
        4:
           Cfg.Text:=StrPas(Addr(fppkg_default[0][1]));
      end;

    TemplateParser.Values['TEMPLATEFILE'] := 'builtin';
    end;
  if ShowBuiltinCommand then
    begin
    ShowBuiltIn;
    halt(0);
    end;
end;


Procedure CreateFile;

Var
  Fout : Text;
  S,BFN : String;
  I : Integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (OutputFileName<>'') and
     DirectoryExists(OutputFileName) then
    begin
      Writeln(StdErr,Format(SErrDestDirectory,[OutputFileName]));
      Halt(1);
    end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  If (OutputFileName<>'')
     and FileExists(OutputFileName)
     and not SkipBackup then
    begin
    BFN:=ChangeFileExt(OutputFileName,'.bak');
    If FileExists(BFN) and not DeleteFile(BFN) then
      begin
      Writeln(StdErr,Format(SErrDelBackupFailed,[BFN]));
      Halt(1);
      end;
    If not RenameFile(OutputFileName,BFN) then
      begin
      Writeln(StdErr,Format(SErrBackupFailed,[OutputFileName,BFN]));
      Halt(1);
      end
    else
      Writeln(Format(SBackupCreated,[ExtractFileName(OutputFileName),ExtractFileName(BFN)]));
    end;
  if (OutputFileName<>'') and not DirectoryExists(ExtractFilePath(OutputFileName)) then
    begin
    if CreateDir then
      begin
      if not ForceDirectories(ExtractFilePath(OutputFileName)) then
        begin
        Writeln(StdErr,Format(SErrCreateDirFailed,[OutputFileName]));
        Halt(1);
        end;
      end
    else
      begin
      Writeln(StdErr,Format(SErrNoSuchDirectory,[OutputFileName]));
      Halt(1);
      end;
    end;
  Assign(Fout,OutputFileName);
  Rewrite(FOut);
  Try
    For I:=0 to Cfg.Count-1 do
      begin
      S:=Cfg[i];
      S := TemplateParser.ParseString(S);
      Writeln(FOut,S);
      end;
  Finally
    Close(Fout);
  end;
end;

begin
  Init;
  Try
    ProcessCommandLine;
    CreateFile;
  Finally
    Done;
  end;
end.
