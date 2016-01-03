{
    This file is part of the Free Pascal Utilities
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit pkgoptions;

interface

<<<<<<< HEAD
<<<<<<< HEAD
uses Classes, Sysutils, Inifiles, fprepos, fpTemplate, pkgglobals;
=======
uses Classes, Sysutils, Inifiles, fprepos;
>>>>>>> graemeg/fixes_2_2
=======
uses Classes, Sysutils, Inifiles, fprepos;
>>>>>>> origin/fixes_2_2

Const
  UnitConfigFileName   = 'fpunits.conf';
  ManifestFileName     = 'manifest.xml';
  MirrorsFileName      = 'mirrors.xml';
  PackagesFileName     = 'packages.xml';
  VersionsFileName     = 'versions-%s.dat';
<<<<<<< HEAD
<<<<<<< HEAD
  CurrentConfigVersion = 4;
=======
  CurrentConfigVersion = 3;
>>>>>>> graemeg/fixes_2_2
=======
  CurrentConfigVersion = 3;
>>>>>>> origin/fixes_2_2

Type

  { TGlobalOptions }

  TGlobalOptions = Class(TPersistent)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    FConfigFilename: string;
    FSaveInifileChanges : Boolean;
=======
    FDirty : Boolean;
>>>>>>> graemeg/fixes_2_2
=======
    FDirty : Boolean;
>>>>>>> origin/fixes_2_2
    FConfigVersion : Integer;
    FRemoteMirrorsURL,
    FRemoteRepository,
    FLocalRepository,
    FCompilerConfigDir,
    FArchivesDir,
    FBuildDir,
    FDownloader,
    FDefaultCompilerConfig,
    FFPMakeCompilerConfig : String;
    // Parameter options
    FCompilerConfig : String;
    FAllowBroken,
    FInstallGlobal,
    FRecoveryMode   : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FOptionParser: TTemplateParser;
    FShowLocation: Boolean;
    FSkipConfigurationFiles: boolean;
    FSkipFixBrokenAfterInstall: boolean;
    FCustomFPMakeOptions : string;
    function  GetOptString(Index: integer): String;
    procedure SetOptString(Index: integer; const AValue: String);
    procedure UpdateLocalRepositoryOption;
  Public
    Constructor Create;
    destructor Destroy; override;
    Procedure InitGlobalDefaults;
    Procedure LoadGlobalFromFile(const AFileName : String);
    Procedure SaveGlobalToFile(const AFileName : String);
    procedure LogValues(ALogLevel: TLogLevel);
    // Is set when the inifile has an old version number (which is also the case when a new file is generated)
    Property SaveInifileChanges : Boolean Read FSaveInifileChanges;
=======
=======
>>>>>>> origin/fixes_2_2
    function  GetOptString(Index: integer): String;
    procedure SetOptString(Index: integer; const AValue: String);
  Public
    Constructor Create;
    Procedure InitGlobalDefaults;
    Procedure LoadGlobalFromFile(const AFileName : String);
    Procedure SaveGlobalToFile(const AFileName : String);
    procedure LogValues;
    Property Dirty : Boolean Read FDirty;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property ConfigVersion : Integer read FConfigVersion;
    function LocalPackagesFile:string;
    function LocalMirrorsFile:string;
    function LocalVersionsFile(const ACompilerConfig:String):string;
  Published
    Property RemoteMirrorsURL : String Index 0 Read GetOptString Write SetOptString;
    // 1 is unused
    Property RemoteRepository : String Index 2 Read GetOptString Write SetOptString;
    Property LocalRepository : String Index 3 Read GetOptString Write SetOptString;
    Property BuildDir : String Index 4 Read GetOptString Write SetOptString;
    Property ArchivesDir : String Index 5 Read GetOptString Write SetOptString;
    Property CompilerConfigDir : String Index 6 Read GetOptString Write SetOptString;
    Property DefaultCompilerConfig : String Index 8 Read GetOptString Write SetOptString;
    Property FPMakeCompilerConfig : String Index 9 Read GetOptString Write SetOptString;
    Property Downloader: String Index 10 Read GetOptString Write SetOptString;
<<<<<<< HEAD
<<<<<<< HEAD
    Property CustomFPMakeOptions: String Index 11 Read GetOptString Write SetOptString;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Parameters
    Property CompilerConfig : String Read FCompilerConfig Write FCompilerConfig;
    Property InstallGlobal : Boolean Read FInstallGlobal Write FInstallGlobal;
    Property RecoveryMode : Boolean Read FRecoveryMode Write FRecoveryMode;
    Property AllowBroken : Boolean Read FAllowBroken Write FAllowBroken;
<<<<<<< HEAD
<<<<<<< HEAD
    Property ShowLocation : Boolean Read FShowLocation Write FShowLocation;
    Property SkipConfigurationFiles: boolean read FSkipConfigurationFiles write FSkipConfigurationFiles;
    Property SkipFixBrokenAfterInstall: boolean read FSkipFixBrokenAfterInstall write FSkipFixBrokenAfterInstall;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;


  { TCompilerOptions }

  TCompilerOptions = Class(TPersistent)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    FConfigFilename: string;
    FSaveInifileChanges: Boolean;
=======
    FDirty: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
    FDirty: Boolean;
>>>>>>> origin/fixes_2_2
    FConfigVersion : Integer;
    FCompiler,
    FCompilerVersion,
    FLocalInstallDir,
<<<<<<< HEAD
<<<<<<< HEAD
    FGlobalInstallDir,
    FLocalPrefix,
    FGlobalPrefix: String;
    FCompilerCPU: TCPU;
    FCompilerOS: TOS;
    FOptionParser: TTemplateParser;
    FOptions: TStrings;
    function GetOptions: TStrings;
=======
    FGlobalInstallDir : String;
    FCompilerCPU: TCPU;
    FCompilerOS: TOS;
>>>>>>> graemeg/fixes_2_2
=======
    FGlobalInstallDir : String;
    FCompilerCPU: TCPU;
    FCompilerOS: TOS;
>>>>>>> origin/fixes_2_2
    function GetOptString(Index: integer): String;
    procedure SetOptString(Index: integer; const AValue: String);
    procedure SetCompilerCPU(const AValue: TCPU);
    procedure SetCompilerOS(const AValue: TOS);
  Public
    Constructor Create;
<<<<<<< HEAD
<<<<<<< HEAD
    Destructor Destroy; override;
    Procedure InitCompilerDefaults;
    Procedure LoadCompilerFromFile(const AFileName : String);
    Procedure SaveCompilerToFile(const AFileName : String);
    procedure LogValues(ALogLevel: TLogLevel; const ACfgName:string);
    procedure UpdateLocalRepositoryOption;
    procedure CheckCompilerValues;
    Function LocalUnitDir:string;
    Function GlobalUnitDir:string;
    Function HasOptions: boolean;
    // Is set when the inifile has an old version number (which is also the case when a new file is generated)
    Property SaveInifileChanges : Boolean Read FSaveInifileChanges;
    Property ConfigVersion : Integer read FConfigVersion;
=======
    Procedure InitCompilerDefaults;
    Procedure LoadCompilerFromFile(const AFileName : String);
    Procedure SaveCompilerToFile(const AFileName : String);
=======
    Procedure InitCompilerDefaults;
    Procedure LoadCompilerFromFile(const AFileName : String);
    Procedure SaveCompilerToFile(const AFileName : String);
>>>>>>> origin/fixes_2_2
    procedure LogValues(const ACfgName:string);
    Property Dirty : Boolean Read FDirty;
    Property ConfigVersion : Integer read FConfigVersion;
    Function LocalUnitDir:string;
    Function GlobalUnitDir:string;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Published
    Property Compiler : String Index 1 Read GetOptString Write SetOptString;
    Property CompilerTarget : String Index 2 Read GetOptString Write SetOptString;
    Property CompilerVersion : String Index 3 Read GetOptString Write SetOptString;
    Property GlobalInstallDir : String Index 4 Read GetOptString Write SetOptString;
    Property LocalInstallDir : String Index 5 Read GetOptString Write SetOptString;
<<<<<<< HEAD
<<<<<<< HEAD
    Property GlobalPrefix : String Index 6 Read GetOptString Write SetOptString;
    Property LocalPrefix : String Index 7 Read GetOptString Write SetOptString;
    Property Options : TStrings read GetOptions;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property CompilerOS : TOS Read FCompilerOS Write SetCompilerOS;
    Property CompilerCPU : TCPU Read FCompilerCPU Write SetCompilerCPU;
  end;

var
  GlobalOptions : TGlobalOptions;
  CompilerOptions : TCompilerOptions;
  FPMakeCompilerOptions : TCompilerOptions;


Implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  pkgglobals,
>>>>>>> graemeg/fixes_2_2
  pkgmessages;

Const
  DefaultMirrorsURL  = 'http://www.freepascal.org/repository/'+MirrorsFileName;
{$ifdef localrepository}
  DefaultRemoteRepository = 'file://'+{$I %HOME%}+'/repository/';
{$else}
  DefaultRemoteRepository = 'auto';
{$endif}

  // ini file keys
  SDefaults = 'Defaults';

  // All configs
  KeyConfigVersion         = 'ConfigVersion';

  // Global config
  KeyRemoteMirrorsURL = 'RemoteMirrors';
  KeyRemoteRepository      = 'RemoteRepository';
  KeyLocalRepository       = 'LocalRepository';
  KeyArchivesDir           = 'ArchivesDir';
  KeyBuildDir              = 'BuildDir';
  KeyCompilerConfigDir     = 'CompilerConfigDir';
  KeyCompilerConfig        = 'CompilerConfig';
  KeyFPMakeCompilerConfig  = 'FPMakeCompilerConfig';
  KeyDownloader            = 'Downloader';
<<<<<<< HEAD
<<<<<<< HEAD
  KeyCustomFPMakeOptions   = 'FPMakeOptions';

  // Compiler dependent config
  KeyGlobalPrefix          = 'GlobalPrefix';
  KeyLocalPrefix           = 'LocalPrefix';
=======

  // Compiler dependent config
>>>>>>> graemeg/fixes_2_2
=======

  // Compiler dependent config
>>>>>>> origin/fixes_2_2
  KeyGlobalInstallDir      = 'GlobalInstallDir';
  KeyLocalInstallDir       = 'LocalInstallDir';
  KeyCompiler              = 'Compiler' ;
  KeyCompilerOS            = 'OS';
  KeyCompilerCPU           = 'CPU';
  KeyCompilerVersion       = 'Version';


{*****************************************************************************
                           TGlobalOptions
*****************************************************************************}

constructor TGlobalOptions.Create;
begin
  FOptionParser := TTemplateParser.Create;
  FOptionParser.Values['AppConfigDir'] := GetAppConfigDir(false);
  FOptionParser.Values['UserDir'] := GetUserDir;
  InitGlobalDefaults;
end;

destructor TGlobalOptions.Destroy;
begin
  FOptionParser.Free;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TGlobalOptions.GetOptString(Index: integer): String;
begin
  Case Index of
    0 : Result:=FRemoteMirrorsURL;
    2 : Result:=FRemoteRepository;
<<<<<<< HEAD
    3 : Result:=FOptionParser.ParseString(FLocalRepository);
    4 : Result:=FOptionParser.ParseString(FBuildDir);
    5 : Result:=FOptionParser.ParseString(FArchivesDir);
    6 : Result:=FOptionParser.ParseString(FCompilerConfigDir);
    8 : Result:=FDefaultCompilerConfig;
    9 : Result:=FFPMakeCompilerConfig;
   10 : Result:=FDownloader;
   11 : Result:=FCustomFPMakeOptions;
=======
    3 : Result:=FLocalRepository;
    4 : Result:=FBuildDir;
    5 : Result:=FArchivesDir;
    6 : Result:=FCompilerConfigDir;
    8 : Result:=FDefaultCompilerConfig;
    9 : Result:=FFPMakeCompilerConfig;
   10 : Result:=FDownloader;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    else
      Error('Unknown option');
  end;
end;

procedure TGlobalOptions.SetOptString(Index: integer; const AValue: String);
begin
  If AValue=GetOptString(Index) then
    Exit;
  Case Index of
    1 : FRemoteMirrorsURL:=AValue;
    2 : FRemoteRepository:=AValue;
<<<<<<< HEAD
    3 : begin
          FLocalRepository:=AValue;
          UpdateLocalRepositoryOption;
        end;
=======
    3 : FLocalRepository:=AValue;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    4 : FBuildDir:=FixPath(AValue);
    5 : FArchivesDir:=FixPath(AValue);
    6 : FCompilerConfigDir:=FixPath(AValue);
    8 : FDefaultCompilerConfig:=AValue;
    9 : FFPMakeCompilerConfig:=AValue;
<<<<<<< HEAD
<<<<<<< HEAD
    10 : FDownloader:=AValue;
    11 : FCustomFPMakeOptions:=AValue;
=======
   10 : FDownloader:=AValue;
>>>>>>> graemeg/fixes_2_2
=======
   10 : FDownloader:=AValue;
>>>>>>> origin/fixes_2_2
    else
      Error('Unknown option');
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TGlobalOptions.UpdateLocalRepositoryOption;
begin
  FOptionParser.Values['LocalRepository'] := LocalRepository;
end;


function TGlobalOptions.LocalPackagesFile:string;
begin
  Result:=LocalRepository+PackagesFileName;
=======
function TGlobalOptions.LocalPackagesFile:string;
begin
  Result:=FLocalRepository+PackagesFileName;
>>>>>>> graemeg/fixes_2_2
end;


function TGlobalOptions.LocalMirrorsFile:string;
begin
<<<<<<< HEAD
  Result:=LocalRepository+MirrorsFileName;
=======
  Result:=FLocalRepository+MirrorsFileName;
>>>>>>> graemeg/fixes_2_2
end;


function TGlobalOptions.LocalVersionsFile(const ACompilerConfig:String):string;
begin
<<<<<<< HEAD
  Result:=LocalRepository+Format(VersionsFileName,[ACompilerConfig]);
=======
  Result:=FLocalRepository+Format(VersionsFileName,[ACompilerConfig]);
>>>>>>> graemeg/fixes_2_2
end;


Procedure TGlobalOptions.InitGlobalDefaults;
<<<<<<< HEAD
var
  i: Integer;
=======
>>>>>>> graemeg/fixes_2_2
=======
function TGlobalOptions.LocalPackagesFile:string;
begin
  Result:=FLocalRepository+PackagesFileName;
end;


function TGlobalOptions.LocalMirrorsFile:string;
begin
  Result:=FLocalRepository+MirrorsFileName;
end;


function TGlobalOptions.LocalVersionsFile(const ACompilerConfig:String):string;
begin
  Result:=FLocalRepository+Format(VersionsFileName,[ACompilerConfig]);
end;


Procedure TGlobalOptions.InitGlobalDefaults;
>>>>>>> origin/fixes_2_2
begin
  FConfigVersion:=CurrentConfigVersion;
  // Retrieve Local fppkg directory
{$ifdef unix}
  if IsSuperUser then
    begin
      if DirectoryExists('/usr/local/lib/fpc') then
        FLocalRepository:='/usr/local/lib/fpc/fppkg/'
      else
        FLocalRepository:='/usr/lib/fpc/fppkg/';
    end
  else
<<<<<<< HEAD
<<<<<<< HEAD
    FLocalRepository:='{UserDir}.fppkg/';
{$else}
  if IsSuperUser then
    FLocalRepository:=IncludeTrailingPathDelimiter(GetAppConfigDir(true))
  else
    FLocalRepository:='{AppConfigDir}';
=======
    FLocalRepository:=IncludeTrailingPathDelimiter(GetEnvironmentVariable('HOME'))+'.fppkg/';
{$else}
  FLocalRepository:=IncludeTrailingPathDelimiter(GetAppConfigDir(IsSuperUser));
>>>>>>> graemeg/fixes_2_2
=======
    FLocalRepository:=IncludeTrailingPathDelimiter(GetEnvironmentVariable('HOME'))+'.fppkg/';
{$else}
  FLocalRepository:=IncludeTrailingPathDelimiter(GetAppConfigDir(IsSuperUser));
>>>>>>> origin/fixes_2_2
{$endif}
  UpdateLocalRepositoryOption;
  // Directories
<<<<<<< HEAD
<<<<<<< HEAD
  FBuildDir:='{LocalRepository}build'+PathDelim;
  FArchivesDir:='{LocalRepository}archives'+PathDelim;
  FCompilerConfigDir:='{LocalRepository}config'+PathDelim;
=======
  FBuildDir:=FLocalRepository+'build'+PathDelim;
  FArchivesDir:=FLocalRepository+'archives'+PathDelim;
  FCompilerConfigDir:=FLocalRepository+'config'+PathDelim;
>>>>>>> graemeg/fixes_2_2
=======
  FBuildDir:=FLocalRepository+'build'+PathDelim;
  FArchivesDir:=FLocalRepository+'archives'+PathDelim;
  FCompilerConfigDir:=FLocalRepository+'config'+PathDelim;
>>>>>>> origin/fixes_2_2
  // Remote
  FRemoteMirrorsURL:=DefaultMirrorsURL;
  FRemoteRepository:=DefaultRemoteRepository;
  // Other config
  FDefaultCompilerConfig:='default';
  FFPMakeCompilerConfig:='default';
  // Downloader
{$if defined(unix) or defined(windows)}
  FDownloader:='lnet';
{$else}
  FDownloader:='base';
{$endif}
  // Parameter defaults
  FCompilerConfig:=FDefaultCompilerConfig;
  FInstallGlobal:=False;
  FRecoveryMode:=False;
  FAllowBroken:=False;
<<<<<<< HEAD
<<<<<<< HEAD

  SetLength(FPMKUnitDeps,FPMKUnitDepDefaultCount);
  for i := 0 to FPMKUnitDepDefaultCount-1 do
    FPMKUnitDeps[i]:=FPMKUnitDepsDefaults[i];
end;


procedure TGlobalOptions.LoadGlobalFromFile(const AFileName: String);
Var
  Ini : TMemIniFile;
begin
  Ini:=TMemIniFile.Create(AFileName);
  try
    FConfigFileName:=AFileName;
    With Ini do
      begin
        FConfigVersion:=ReadInteger(SDefaults,KeyConfigVersion,0);
        if (FConfigVersion<>CurrentConfigVersion) then
          begin
            Log(vlDebug,SLogUpgradingConfig,[AFileName]);
            FSaveInifileChanges:=true;
            if FConfigVersion<1 then
              begin
                FRemoteRepository:='auto';
              end;
            if FConfigVersion<3 then
              begin
                // Directories
                FBuildDir:=FLocalRepository+'build'+PathDelim;
                FArchivesDir:=FLocalRepository+'archives'+PathDelim;
                FCompilerConfigDir:=FLocalRepository+'config'+PathDelim;
              end;
            if (FConfigVersion>CurrentConfigVersion) then
              Error(SErrUnsupportedConfigVersion,[AFileName]);
          end;
        FRemoteMirrorsURL:=ReadString(SDefaults,KeyRemoteMirrorsURL,FRemoteMirrorsURL);
        FRemoteRepository:=ReadString(SDefaults,KeyRemoteRepository,FRemoteRepository);
        FLocalRepository:=ReadString(SDefaults,KeyLocalRepository,FLocalRepository);
        UpdateLocalRepositoryOption;
        FBuildDir:=FixPath(ReadString(SDefaults,KeyBuildDir,FBuildDir));
        FArchivesDir:=FixPath(ReadString(SDefaults,KeyArchivesDir,FArchivesDir));
        FCompilerConfigDir:=FixPath(ReadString(SDefaults,KeyCompilerConfigDir,FCompilerConfigDir));
        FDefaultCompilerConfig:=ReadString(SDefaults,KeyCompilerConfig,FDefaultCompilerConfig);
        FFPMakeCompilerConfig:=ReadString(SDefaults,KeyFPMakeCompilerConfig,FFPMakeCompilerConfig);
        FDownloader:=ReadString(SDefaults,KeyDownloader,FDownloader);
        FCustomFPMakeOptions:=ReadString(SDefaults,KeyCustomFPMakeOptions,FCustomFPMakeOptions);
      end;
  finally
    Ini.Free;
  end;
end;


procedure TGlobalOptions.SaveGlobalToFile(const AFileName: String);
Var
  Ini : TIniFile;
begin
  if FileExists(AFileName) then
    BackupFile(AFileName);
  Ini:=TIniFile.Create(AFileName);
  try
    With Ini do
      begin
        WriteInteger(SDefaults,KeyConfigVersion,CurrentConfigVersion);
        WriteString(SDefaults,KeyLocalRepository,FLocalRepository);
        WriteString(SDefaults,KeyBuildDir,FBuildDir);
        WriteString(SDefaults,KeyArchivesDir,FArchivesDir);
        WriteString(SDefaults,KeyCompilerConfigDir,FCompilerConfigDir);
        WriteString(SDefaults,KeyRemoteMirrorsURL,FRemoteMirrorsURL);
        WriteString(SDefaults,KeyRemoteRepository,FRemoteRepository);
        WriteString(SDefaults,KeyCompilerConfig,FDefaultCompilerConfig);
        WriteString(SDefaults,KeyFPMakeCompilerConfig,FFPMakeCompilerConfig);
        WriteString(SDefaults,KeyDownloader,FDownloader);
        FSaveInifileChanges:=False;
      end;
    Ini.UpdateFile;
  finally
    Ini.Free;
  end;
end;


procedure TGlobalOptions.LogValues(ALogLevel: TLogLevel);
begin
  Log(ALogLevel,SLogGlobalCfgHeader,[FConfigFilename]);
  Log(ALogLevel,SLogGlobalCfgRemoteMirrorsURL,[FRemoteMirrorsURL]);
  Log(ALogLevel,SLogGlobalCfgRemoteRepository,[FRemoteRepository]);
  Log(ALogLevel,SLogGlobalCfgLocalRepository,[FLocalRepository,LocalRepository]);
  Log(ALogLevel,SLogGlobalCfgBuildDir,[FBuildDir,BuildDir]);
  Log(ALogLevel,SLogGlobalCfgArchivesDir,[FArchivesDir,ArchivesDir]);
  Log(ALogLevel,SLogGlobalCfgCompilerConfigDir,[FCompilerConfigDir,CompilerConfigDir]);
  Log(ALogLevel,SLogGlobalCfgDefaultCompilerConfig,[FDefaultCompilerConfig]);
  Log(ALogLevel,SLogGlobalCfgFPMakeCompilerConfig,[FPMakeCompilerConfig]);
  Log(ALogLevel,SLogGlobalCfgDownloader,[FDownloader]);
end;


{*****************************************************************************
                           TCompilerOptions
*****************************************************************************}

constructor TCompilerOptions.Create;
begin
  FOptionParser := TTemplateParser.Create;
  FOptionParser.Values['AppConfigDir'] := GetAppConfigDir(false);
  FOptionParser.Values['UserDir'] := GetUserDir;
  {$ifdef unix}
  FLocalInstallDir:='{LocalPrefix}'+'lib'+PathDelim+'fpc'+PathDelim+'{CompilerVersion}'+PathDelim;
  FGlobalInstallDir:='{GlobalPrefix}'+'lib'+PathDelim+'fpc'+PathDelim+'{CompilerVersion}'+PathDelim;
  {$else unix}
  FLocalInstallDir:='{LocalPrefix}';
  FGlobalInstallDir:='{GlobalPrefix}';
  {$endif}
end;

destructor TCompilerOptions.Destroy;
begin
  FOptionParser.Free;
  if assigned(FOptions) then
    FreeAndNil(FOptions);
  inherited Destroy;
end;


function TCompilerOptions.GetOptString(Index: integer): String;
begin
  Case Index of
    1 : Result:=FCompiler;
    2 : Result:=MakeTargetString(CompilerCPU,CompilerOS);
    3 : Result:=FCompilerVersion;
    4 : Result:=FOptionParser.ParseString(FGlobalInstallDir);
    5 : Result:=FOptionParser.ParseString(FLocalInstallDir);
    6 : Result:=FixPath(FOptionParser.ParseString(FGlobalPrefix));
    7 : Result:=FixPath(FOptionParser.ParseString(FLocalPrefix));
    else
      Error('Unknown option');
  end;
end;

function TCompilerOptions.GetOptions: TStrings;
begin
  if not assigned(FOptions) then
    begin
      FOptions := TStringList.Create;
      FOptions.Delimiter:=' ';
    end;
  Result := FOptions;
end;


procedure TCompilerOptions.SetOptString(Index: integer; const AValue: String);
begin
  If AValue=GetOptString(Index) then
    Exit;
  Case Index of
    1 : FCompiler:=AValue;
    2 : StringToCPUOS(AValue,FCompilerCPU,FCompilerOS);
    3 : begin
          FCompilerVersion:=AValue;
          FOptionParser.Values['CompilerVersion'] := FCompilerVersion;
        end;
    4 : FGlobalInstallDir:=FixPath(AValue);
    5 : FLocalInstallDir:=FixPath(AValue);
    6 : begin
          FGlobalPrefix:=AValue;
          FOptionParser.Values['GlobalPrefix'] := GlobalPrefix;
        end;
    7 : begin
          FLocalPrefix:=AValue;
          FOptionParser.Values['LocalPrefix'] := LocalPrefix;
        end
    else
      Error('Unknown option');
  end;
end;


procedure TCompilerOptions.SetCompilerCPU(const AValue: TCPU);
begin
  if FCompilerCPU=AValue then
    exit;
  FCompilerCPU:=AValue;
end;


procedure TCompilerOptions.UpdateLocalRepositoryOption;
begin
  FOptionParser.Values['LocalRepository'] := GlobalOptions.LocalRepository;
end;

procedure TCompilerOptions.CheckCompilerValues;
var
  AVersion : string;
  ACpu     : TCpu;
  AOs      : TOS;
begin
  if Compiler='' then
    Exit;
  if (CompilerCPU=cpuNone) or
   (CompilerOS=osNone) or
   (CompilerVersion='') then
  begin
    GetCompilerInfo(Compiler,'-iVTPTO',AVersion,ACpu,AOs);
    if CompilerCPU=cpuNone then
      CompilerCPU := ACpu;
    if CompilerOS=osNone then
      CompilerOS:=AOs;
    if CompilerVersion='' then
      CompilerVersion:=AVersion;
=======
end;


=======
end;


>>>>>>> origin/fixes_2_2
procedure TGlobalOptions.LoadGlobalFromFile(const AFileName: String);
Var
  Ini : TMemIniFile;
begin
  try
    Ini:=TMemIniFile.Create(AFileName);
    With Ini do
      begin
        FConfigVersion:=ReadInteger(SDefaults,KeyConfigVersion,0);
        if (FConfigVersion<>CurrentConfigVersion) then
          begin
            Log(vlDebug,SLogUpgradingConfig,[AFileName]);
            FDirty:=true;
            if FConfigVersion<1 then
              begin
                FRemoteRepository:='auto';
              end;
            if FConfigVersion<3 then
              begin
                // Directories
                FBuildDir:=FLocalRepository+'build'+PathDelim;
                FArchivesDir:=FLocalRepository+'archives'+PathDelim;
                FCompilerConfigDir:=FLocalRepository+'config'+PathDelim;
              end;
            if (FConfigVersion>CurrentConfigVersion) then
              Error(SErrUnsupportedConfigVersion,[AFileName]);
          end;
        FRemoteMirrorsURL:=ReadString(SDefaults,KeyRemoteMirrorsURL,FRemoteMirrorsURL);
        FRemoteRepository:=ReadString(SDefaults,KeyRemoteRepository,FRemoteRepository);
        FLocalRepository:=ReadString(SDefaults,KeyLocalRepository,FLocalRepository);
        FBuildDir:=FixPath(ReadString(SDefaults,KeyBuildDir,FBuildDir));
        FArchivesDir:=FixPath(ReadString(SDefaults,KeyArchivesDir,FArchivesDir));
        FCompilerConfigDir:=FixPath(ReadString(SDefaults,KeyCompilerConfigDir,FCompilerConfigDir));
        FDefaultCompilerConfig:=ReadString(SDefaults,KeyCompilerConfig,FDefaultCompilerConfig);
        FFPMakeCompilerConfig:=ReadString(SDefaults,KeyFPMakeCompilerConfig,FFPMakeCompilerConfig);
        FDownloader:=ReadString(SDefaults,KeyDownloader,FDownloader);
      end;
  finally
    Ini.Free;
>>>>>>> graemeg/fixes_2_2
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TCompilerOptions.SetCompilerOS(const AValue: TOS);
begin
  if FCompilerOS=AValue then
    exit;
  FCompilerOS:=AValue;
end;


function TCompilerOptions.LocalUnitDir:string;
var ALocalInstallDir: string;
begin
  ALocalInstallDir:=LocalInstallDir;

  if ALocalInstallDir<>'' then
    result:=ALocalInstallDir+'units'+PathDelim+CompilerTarget+PathDelim
  else
    result:='';
end;


function TCompilerOptions.GlobalUnitDir:string;
var AGlobalInstallDir: string;
begin
  AGlobalInstallDir:=GlobalInstallDir;

  if AGlobalInstallDir<>'' then
    result:=AGlobalInstallDir+'units'+PathDelim+CompilerTarget+PathDelim
  else
    result:='';
end;


function TCompilerOptions.HasOptions: boolean;
begin
  result := assigned(FOptions);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TGlobalOptions.SaveGlobalToFile(const AFileName: String);
Var
  Ini : TIniFile;
begin
  if FileExists(AFileName) then
    BackupFile(AFileName);
  try
    Ini:=TIniFile.Create(AFileName);
    With Ini do
      begin
        WriteInteger(SDefaults,KeyConfigVersion,CurrentConfigVersion);
        WriteString(SDefaults,KeyBuildDir,FBuildDir);
        WriteString(SDefaults,KeyArchivesDir,FArchivesDir);
        WriteString(SDefaults,KeyCompilerConfigDir,FCompilerConfigDir);
        WriteString(SDefaults,KeyLocalRepository,FLocalRepository);
        WriteString(SDefaults,KeyRemoteMirrorsURL,FRemoteMirrorsURL);
        WriteString(SDefaults,KeyRemoteRepository,FRemoteRepository);
        WriteString(SDefaults,KeyCompilerConfig,FDefaultCompilerConfig);
        WriteString(SDefaults,KeyFPMakeCompilerConfig,FFPMakeCompilerConfig);
        WriteString(SDefaults,KeyDownloader,FDownloader);
        FDirty:=False;
      end;
    Ini.UpdateFile;
  finally
    Ini.Free;
  end;
end;


procedure TGlobalOptions.LogValues;
begin
  Log(vlDebug,SLogGlobalCfgHeader);
  Log(vlDebug,SLogGlobalCfgRemoteMirrorsURL,[FRemoteMirrorsURL]);
  Log(vlDebug,SLogGlobalCfgRemoteRepository,[FRemoteRepository]);
  Log(vlDebug,SLogGlobalCfgLocalRepository,[FLocalRepository]);
  Log(vlDebug,SLogGlobalCfgBuildDir,[FBuildDir]);
  Log(vlDebug,SLogGlobalCfgArchivesDir,[FArchivesDir]);
  Log(vlDebug,SLogGlobalCfgCompilerConfigDir,[FCompilerConfigDir]);
  Log(vlDebug,SLogGlobalCfgDefaultCompilerConfig,[FDefaultCompilerConfig]);
  Log(vlDebug,SLogGlobalCfgFPMakeCompilerConfig,[FFPMakeCompilerConfig]);
  Log(vlDebug,SLogGlobalCfgDownloader,[FDownloader]);
end;


{*****************************************************************************
                           TCompilerOptions
*****************************************************************************}

constructor TCompilerOptions.Create;
<<<<<<< HEAD
begin
end;


function TCompilerOptions.GetOptString(Index: integer): String;
begin
  Case Index of
    1 : Result:=FCompiler;
    2 : Result:=MakeTargetString(CompilerCPU,CompilerOS);
    3 : Result:=FCompilerVersion;
    4 : Result:=FGlobalInstallDir;
    5 : Result:=FLocalInstallDir;
    else
      Error('Unknown option');
  end;
end;

procedure TCompilerOptions.SetOptString(Index: integer; const AValue: String);
begin
  If AValue=GetOptString(Index) then
    Exit;
  Case Index of
    1 : FCompiler:=AValue;
    2 : StringToCPUOS(AValue,FCompilerCPU,FCompilerOS);
    3 : FCompilerVersion:=AValue;
    4 : FGlobalInstallDir:=FixPath(AValue);
    5 : FLocalInstallDir:=FixPath(AValue);
    else
      Error('Unknown option');
  end;
  FDirty:=True;
end;


procedure TCompilerOptions.SetCompilerCPU(const AValue: TCPU);
begin
  if FCompilerCPU=AValue then
    exit;
  FCompilerCPU:=AValue;
  FDirty:=True;
end;


procedure TCompilerOptions.SetCompilerOS(const AValue: TOS);
begin
  if FCompilerOS=AValue then
    exit;
  FCompilerOS:=AValue;
  FDirty:=True;
end;


function TCompilerOptions.LocalUnitDir:string;
begin
  if FLocalInstallDir<>'' then
    result:=FLocalInstallDir+'units'+PathDelim+CompilerTarget+PathDelim
  else
    result:='';
end;


=======
begin
end;


function TCompilerOptions.GetOptString(Index: integer): String;
begin
  Case Index of
    1 : Result:=FCompiler;
    2 : Result:=MakeTargetString(CompilerCPU,CompilerOS);
    3 : Result:=FCompilerVersion;
    4 : Result:=FGlobalInstallDir;
    5 : Result:=FLocalInstallDir;
    else
      Error('Unknown option');
  end;
end;

procedure TCompilerOptions.SetOptString(Index: integer; const AValue: String);
begin
  If AValue=GetOptString(Index) then
    Exit;
  Case Index of
    1 : FCompiler:=AValue;
    2 : StringToCPUOS(AValue,FCompilerCPU,FCompilerOS);
    3 : FCompilerVersion:=AValue;
    4 : FGlobalInstallDir:=FixPath(AValue);
    5 : FLocalInstallDir:=FixPath(AValue);
    else
      Error('Unknown option');
  end;
  FDirty:=True;
end;


procedure TCompilerOptions.SetCompilerCPU(const AValue: TCPU);
begin
  if FCompilerCPU=AValue then
    exit;
  FCompilerCPU:=AValue;
  FDirty:=True;
end;


procedure TCompilerOptions.SetCompilerOS(const AValue: TOS);
begin
  if FCompilerOS=AValue then
    exit;
  FCompilerOS:=AValue;
  FDirty:=True;
end;


function TCompilerOptions.LocalUnitDir:string;
begin
  if FLocalInstallDir<>'' then
    result:=FLocalInstallDir+'units'+PathDelim+CompilerTarget+PathDelim
  else
    result:='';
end;


>>>>>>> origin/fixes_2_2
function TCompilerOptions.GlobalUnitDir:string;
begin
  if FGlobalInstallDir<>'' then
    result:=FGlobalInstallDir+'units'+PathDelim+CompilerTarget+PathDelim
  else
    result:='';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TCompilerOptions.InitCompilerDefaults;
<<<<<<< HEAD
<<<<<<< HEAD
var
  ACompilerVersion: string;
  fpcdir: string;
begin
  FConfigVersion:=CurrentConfigVersion;
  if fcompiler = '' then
    FCompiler:=ExeSearch('fpc'+ExeExt,GetEnvironmentVariable('PATH'));
  if FCompiler='' then
    Raise EPackagerError.Create(SErrMissingFPC);
  // Detect compiler version/target from -i option
  GetCompilerInfo(FCompiler,'-iVTPTO',ACompilerVersion,FCompilerCPU,FCompilerOS);
  CompilerVersion := ACompilerVersion;
=======
=======
>>>>>>> origin/fixes_2_2

var
  infoSL : TStringList;
begin
  FConfigVersion:=CurrentConfigVersion;
  FCompiler:=ExeSearch('fpc'+ExeExt,GetEnvironmentVariable('PATH'));
  if FCompiler='' then
    Raise EPackagerError.Create(SErrMissingFPC);
  // Detect compiler version/target from -i option
  infosl:=TStringList.Create;
  infosl.Delimiter:=' ';
  infosl.DelimitedText:=GetCompilerInfo(FCompiler,'-iVTPTO');
  if infosl.Count<>3 then
    Raise EPackagerError.Create(SErrInvalidFPCInfo);
  FCompilerVersion:=infosl[0];
  FCompilerCPU:=StringToCPU(infosl[1]);
  FCompilerOS:=StringToOS(infosl[2]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // Temporary hack to workaround bug in fpc.exe that doesn't support spaces
  // We retrieve the real binary
  if FCompilerVersion='2.2.0' then
    FCompiler:=GetCompilerInfo(FCompiler,'-PB');
  Log(vlDebug,SLogDetectedCompiler,[FCompiler,FCompilerVersion,MakeTargetString(FCompilerCPU,FCompilerOS)]);
<<<<<<< HEAD
<<<<<<< HEAD

  // Use the same algorithm as the compiler, see options.pas
  // Except that the prefix is extracted and GlobalInstallDir is set using
  // that prefix
{$ifdef Unix}
  FGlobalPrefix:='/usr/local/';
  if not DirectoryExists(FGlobalPrefix+'lib/fpc/'+FCompilerVersion+'/') and
     DirectoryExists('/usr/lib/fpc/'+FCompilerVersion+'/') then
    FGlobalPrefix:='/usr/';
{$else unix}
  FGlobalPrefix:=ExtractFilePath(FCompiler)+'..'+PathDelim;
  if not(DirectoryExists(FGlobalPrefix+PathDelim+'units')) and
     not(DirectoryExists(FGlobalPrefix+PathDelim+'rtl')) then
    FGlobalPrefix:=FGlobalPrefix+'..'+PathDelim;
  FGlobalPrefix:=ExpandFileName(FGlobalPrefix);
{$endif unix}

  Log(vlDebug,SLogDetectedPrefix,['global',FGlobalPrefix]);
  // User writable install directory
  if not IsSuperUser then
    begin
      FLocalPrefix:= '{LocalRepository}';
      Log(vlDebug,SLogDetectedPrefix,['local',FLocalPrefix]);
    end;

  fpcdir:=FixPath(GetEnvironmentVariable('FPCDIR'));
  if fpcdir<>'' then
    begin
    {$ifndef Unix}
    fpcdir:=ExpandFileName(fpcdir);
    {$endif unix}
    Log(vlDebug,SLogFPCDirEnv,[fpcdir]);
    FGlobalInstallDir:=fpcdir;
=======
=======
>>>>>>> origin/fixes_2_2
  // Use the same algorithm as the compiler, see options.pas
{$ifdef Unix}
  FGlobalInstallDir:=FixPath(GetEnvironmentVariable('FPCDIR'));
  if FGlobalInstallDir='' then
    begin
      FGlobalInstallDir:='/usr/local/lib/fpc/'+FCompilerVersion+'/';
      if not DirectoryExists(FGlobalInstallDir) and
         DirectoryExists('/usr/lib/fpc/'+FCompilerVersion) then
        FGlobalInstallDir:='/usr/lib/fpc/'+FCompilerVersion+'/';
    end;
{$else unix}
  FGlobalInstallDir:=FixPath(GetEnvironmentVariable('FPCDIR'));
  if FGlobalInstallDir='' then
    begin
      FGlobalInstallDir:=ExtractFilePath(FCompiler)+'../';
      if not(DirectoryExists(FGlobalInstallDir+'/units')) and
         not(DirectoryExists(FGlobalInstallDir+'/rtl')) then
        FGlobalInstallDir:=FGlobalInstallDir+'../';
    end;
  FGlobalInstallDir:=ExpandFileName(FGlobalInstallDir);
{$endif unix}
  Log(vlDebug,SLogDetectedFPCDIR,['global',FGlobalInstallDir]);
  // User writable install directory
  if not IsSuperUser then
    begin
      FLocalInstallDir:=GlobalOptions.LocalRepository+'lib'+PathDelim+FCompilerVersion+PathDelim;
      Log(vlDebug,SLogDetectedFPCDIR,['local',FLocalInstallDir]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;


procedure TCompilerOptions.LoadCompilerFromFile(const AFileName: String);
Var
  Ini : TMemIniFile;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Ini:=TMemIniFile.Create(AFileName);
  try
    FConfigFilename:=AFileName;
=======
  try
    Ini:=TMemIniFile.Create(AFileName);
>>>>>>> graemeg/fixes_2_2
=======
  try
    Ini:=TMemIniFile.Create(AFileName);
>>>>>>> origin/fixes_2_2
    With Ini do
      begin
        FConfigVersion:=ReadInteger(SDefaults,KeyConfigVersion,0);
        if (FConfigVersion<>CurrentConfigVersion) then
          begin
            Log(vlDebug,SLogUpgradingConfig,[AFileName]);
<<<<<<< HEAD
<<<<<<< HEAD
            FSaveInifileChanges:=true;
            if (FConfigVersion>CurrentConfigVersion) then
              Error(SErrUnsupportedConfigVersion,[AFileName]);
          end;
        GlobalPrefix:=ReadString(SDefaults,KeyGlobalPrefix,FGlobalPrefix);
        LocalPrefix:=ReadString(SDefaults,KeyLocalPrefix,FLocalPrefix);
=======
=======
>>>>>>> origin/fixes_2_2
            FDirty:=true;
            if (FConfigVersion>CurrentConfigVersion) then
              Error(SErrUnsupportedConfigVersion,[AFileName]);
          end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        FGlobalInstallDir:=FixPath(ReadString(SDefaults,KeyGlobalInstallDir,FGlobalInstallDir));
        FLocalInstallDir:=FixPath(ReadString(SDefaults,KeyLocalInstallDir,FLocalInstallDir));
        FCompiler:=ReadString(SDefaults,KeyCompiler,FCompiler);
        FCompilerOS:=StringToOS(ReadString(SDefaults,KeyCompilerOS,OSToString(CompilerOS)));
        FCompilerCPU:=StringToCPU(ReadString(SDefaults,KeyCompilerCPU,CPUtoString(CompilerCPU)));
<<<<<<< HEAD
<<<<<<< HEAD
        CompilerVersion:=ReadString(SDefaults,KeyCompilerVersion,FCompilerVersion);
=======
        FCompilerVersion:=ReadString(SDefaults,KeyCompilerVersion,FCompilerVersion);
>>>>>>> graemeg/fixes_2_2
=======
        FCompilerVersion:=ReadString(SDefaults,KeyCompilerVersion,FCompilerVersion);
>>>>>>> origin/fixes_2_2
      end;
  finally
    Ini.Free;
  end;
end;


procedure TCompilerOptions.SaveCompilerToFile(const AFileName: String);
Var
  Ini : TIniFile;
begin
  if FileExists(AFileName) then
    BackupFile(AFileName);
<<<<<<< HEAD
<<<<<<< HEAD
  Ini:=TIniFile.Create(AFileName);
  try
    With Ini do
      begin
        WriteInteger(SDefaults,KeyConfigVersion,CurrentConfigVersion);
        WriteString(SDefaults,KeyGlobalPrefix,FGlobalPrefix);
        WriteString(SDefaults,KeyLocalPrefix,FLocalPrefix);
=======
  try
=======
  try
>>>>>>> origin/fixes_2_2
    Ini:=TIniFile.Create(AFileName);
    With Ini do
      begin
        WriteInteger(SDefaults,KeyConfigVersion,CurrentConfigVersion);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        WriteString(SDefaults,KeyGlobalInstallDir,FGlobalInstallDir);
        WriteString(SDefaults,KeyLocalInstallDir,FLocalInstallDir);
        WriteString(SDefaults,KeyCompiler,FCompiler);
        WriteString(SDefaults,KeyCompilerOS,OSToString(CompilerOS));
        WriteString(SDefaults,KeyCompilerCPU,CPUtoString(CompilerCPU));
        WriteString(SDefaults,KeyCompilerVersion,FCompilerVersion);
<<<<<<< HEAD
<<<<<<< HEAD
        FSaveInifileChanges:=False;
=======
        FDirty:=False;
>>>>>>> graemeg/fixes_2_2
=======
        FDirty:=False;
>>>>>>> origin/fixes_2_2
      end;
    Ini.UpdateFile;
  finally
    Ini.Free;
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TCompilerOptions.LogValues(ALogLevel: TLogLevel; const ACfgName:string);
begin
  Log(ALogLevel,SLogCompilerCfgHeader,[ACfgName,FConfigFilename]);
  Log(ALogLevel,SLogCompilerCfgCompiler,[FCompiler]);
  Log(ALogLevel,SLogCompilerCfgTarget,[MakeTargetString(CompilerCPU,CompilerOS)]);
  Log(ALogLevel,SLogCompilerCfgVersion,[FCompilerVersion]);
  Log(ALogLevel,SLogCompilerCfgGlobalPrefix,[FGlobalPrefix,GlobalPrefix]);
  Log(ALogLevel,SLogCompilerCfgLocalPrefix,[FLocalPrefix,LocalPrefix]);
  Log(ALogLevel,SLogCompilerCfgGlobalInstallDir,[FGlobalInstallDir,GlobalInstallDir]);
  Log(ALogLevel,SLogCompilerCfgLocalInstallDir,[FLocalInstallDir,LocalInstallDir]);
  Log(ALogLevel,SLogCompilerCfgOptions,[Options.DelimitedText]);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCompilerOptions.LogValues(const ACfgName:string);
begin
  Log(vlDebug,SLogCompilerCfgHeader,[ACfgName]);
  Log(vlDebug,SLogCompilerCfgCompiler,[FCompiler]);
  Log(vlDebug,SLogCompilerCfgTarget,[MakeTargetString(CompilerCPU,CompilerOS)]);
  Log(vlDebug,SLogCompilerCfgVersion,[FCompilerVersion]);
  Log(vlDebug,SLogCompilerCfgGlobalInstallDir,[FGlobalInstallDir]);
  Log(vlDebug,SLogCompilerCfgLocalInstallDir,[FLocalInstallDir]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


initialization
  GlobalOptions:=TGlobalOptions.Create;
  CompilerOptions:=TCompilerOptions.Create;
  FPMakeCompilerOptions:=TCompilerOptions.Create;
finalization
  FreeAndNil(GlobalOptions);
  FreeAndNil(CompilerOptions);
  FreeAndNil(FPMakeCompilerOptions);
end.
