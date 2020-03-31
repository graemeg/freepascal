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

// pkgglobals must be AFTER fpmkunit
uses Classes, Sysutils, Inifiles, StrUtils, fpTemplate, fpmkunit, pkgglobals, fgl;

Const
  UnitConfigFileName   = 'fpunits.cfg';
  ManifestFileName     = 'manifest.xml';
  MirrorsFileName      = 'mirrors.xml';
  PackagesFileName     = 'packages.xml';
  MinSupportedConfigVersion = 4;
  CurrentConfigVersion = 5;

Type
  TFPRepositoryType = (fprtUnknown, fprtInstalled, fprtAvailable);

  { TFppkgOptionSection }

  TCompilerOptions = class;
  TFppkgOptions = class;
  TFppkgOptionSection = class(TPersistent)
  private
    FOptionParser: TTemplateParser;
    FName: string;
  protected
    class function GetKey: string; virtual;
    procedure AddKeyValueToStrings(AStrings: TStrings; AKey, AValue: string); overload;
    procedure AddKeyValueToStrings(AStrings: TStrings; AKey: string; AValue: Integer); overload;
    property OptionParser: TTemplateParser read FOptionParser;
  public
    constructor Create(AnOptionParser: TTemplateParser); virtual;
    procedure AddKeyValue(const AKey, AValue: string); virtual;
    procedure SaveToStrings(AStrings: TStrings); virtual;
    procedure LogValues(ALogLevel: TLogLevel); virtual;
    function AllowDuplicate: Boolean; virtual;

    property Name: string read FName write FName;
  end;
  TFppkgOptionSectionList = specialize TFPGObjectList<TFppkgOptionSection>;

  { TFppkgGLobalOptionSection }

  TFppkgGlobalOptionSection = class(TFppkgOptionSection)
  private
    FCustomFPMakeOptions: string;

    FBuildDir: string;
    FCompilerConfigDir: string;
    FConfigVersion: integer;
    FCompilerConfig: string;
    FInstallRepository: string;
    FDownloader: string;
    FFPMakeCompilerConfig: string;
    FLocalRepository: string;
    FRemoteMirrorsURL: string;
    FRemoteRepository: string;
    FArchivesDir: string;
    function GetArchivesDir: string;
    function GetBuildDir: string;
    function GetCompilerConfigDir: string;
    function GetLocalRepository: string;
    procedure SetArchivesDir(AValue: string);
    procedure SetBuildDir(AValue: string);
    procedure SetCompilerConfigDir(AValue: string);
    procedure SetConfigVersion(AValue: integer);
    procedure SetCompilerConfig(AValue: string);
    procedure SetCustomFPMakeOptions(AValue: string);
    procedure SetDownloader(AValue: string);
    procedure SetFPMakeCompilerConfig(AValue: string);
    procedure SetLocalRepository(AValue: string);
    procedure SetRemoteMirrorsURL(AValue: string);
    procedure SetRemoteRepository(AValue: string);
  public
    constructor Create(AnOptionParser: TTemplateParser); override;
    destructor Destroy; override;
    procedure AddKeyValue(const AKey, AValue: string); override;
    procedure SaveToStrings(AStrings: TStrings); override;
    procedure LogValues(ALogLevel: TLogLevel); override;

    function LocalMirrorsFile:string;
    function LocalPackagesFile:string;

    property ConfigVersion: integer read FConfigVersion write SetConfigVersion;
    property LocalRepository: string read GetLocalRepository write SetLocalRepository;
    property BuildDir: string read GetBuildDir write SetBuildDir;
    property CompilerConfigDir: string read GetCompilerConfigDir write SetCompilerConfigDir;
    property ArchivesDir: string read GetArchivesDir write SetArchivesDir;
    property Downloader: string read FDownloader write SetDownloader;
    property CompilerConfig: string read FCompilerConfig write SetCompilerConfig;
    property FPMakeCompilerConfig: string read FFPMakeCompilerConfig write SetFPMakeCompilerConfig;
    property InstallRepository: string read FInstallRepository write FInstallRepository;
    property RemoteRepository: string read FRemoteRepository write SetRemoteRepository;
    property RemoteMirrorsURL: string read FRemoteMirrorsURL write SetRemoteMirrorsURL;
    Property CustomFPMakeOptions: string read FCustomFPMakeOptions Write SetCustomFPMakeOptions;
  end;

  { TFppkgCustomOptionSection }

  TFppkgCustomOptionSection = class(TFppkgOptionSection);

  { TFppkgRepositoryOptionSection }

  TFppkgRepositoryOptionSection = class(TFppkgOptionSection)
  private
    FDescription: string;
    FInstallRepositoryName: string;
    FPath: string;
    FPrefix: string;
    FRepositoryName: string;
    function GetPath: string;
    function GetPrefix: string;
    procedure SetDescription(AValue: string);
    procedure SetPrefix(AValue: string);
    procedure SetRepositoryName(AValue: string);
    procedure SetPath(AValue: string);
  protected
    procedure SaveToStrings(AStrings: TStrings); override;
    class function GetKey: string; override;
  public
    procedure AddKeyValue(const AKey, AValue: string); override;
    procedure LogValues(ALogLevel: TLogLevel); override;
    function AllowDuplicate: Boolean; override;
    function GetRepositoryType: TFPRepositoryType; virtual;

    property RepositoryName: string read FRepositoryName write SetRepositoryName;
    property Description: string read FDescription write SetDescription;
    property Path: string read GetPath write SetPath;
    property Prefix: string read GetPrefix write SetPrefix;
    property InstallRepositoryName: string read FInstallRepositoryName write FInstallRepositoryName;
  end;
  TFppkgRepositoryOptionSectionClass = class of TFppkgRepositoryOptionSection;

  { TFppkgIncludeFilesOptionSection }

  TFppkgIncludeFilesOptionSection = class(TFppkgOptionSection)
  private
    FOptions: TFppkgOptions;
    // Only used for logging
    FOptionCache: TStringList;
    FCurrentDir: String;

    procedure IncludeFile(AFileName: string);
    procedure IncludeFileMask(AFileNameMask: string);
  protected
    class function GetKey: string; override;
  public
    constructor Create(AnOptionParser: TTemplateParser; AnOptions: TFppkgOptions; ACurrentDir: string);
    destructor Destroy; override;
    procedure SaveToStrings(AStrings: TStrings); override;
    procedure AddKeyValue(const AKey, AValue: string); override;
    procedure LogValues(ALogLevel: TLogLevel); override;
    function AllowDuplicate: Boolean; override;
  end;

  { TFppkgCommandLineOptionSection }

  TFppkgCommandLineOptionSection = class(TFppkgOptionSection)
  private
    FAllowBroken: Boolean;
    FCompilerConfig: string;
    FInstallRepository: string;
    FRecoveryMode: Boolean;
    FShowLocation: Boolean;
    FSkipConfigurationFiles: Boolean;
    FSkipFixBrokenAfterInstall: Boolean;
  public
    constructor Create(AnOptionParser: TTemplateParser); override;
    property RecoveryMode: Boolean read FRecoveryMode write FRecoveryMode;
    property ShowLocation: Boolean read FShowLocation write FShowLocation;
    property CompilerConfig : string read FCompilerConfig write FCompilerConfig;
    property InstallRepository: string read FInstallRepository write FInstallRepository;
    property SkipConfigurationFiles: Boolean read FSkipConfigurationFiles write FSkipConfigurationFiles;
    property AllowBroken : Boolean read FAllowBroken write FAllowBroken;
    property SkipFixBrokenAfterInstall: Boolean read FSkipFixBrokenAfterInstall write FSkipFixBrokenAfterInstall;
  end;


  { TFppkgOptions }

  TFppkgOptions = class(TPersistent)
  private
    FOptionParser: TTemplateParser;
    FPreferGlobal: Boolean;
    FSectionList: TFppkgOptionSectionList;
    function GetCommandLineSection: TFppkgCommandLineOptionSection;
    function GetGlobalSection: TFppkgGLobalOptionSection;
    function GetSectionList: TFppkgOptionSectionList;
  public
    constructor Create();
    destructor Destroy; override;

    procedure LoadFromFile(const AFileName: string);
    procedure SaveToFile(const AFileName: string);
    function GetSectionByName(const SectionName: string): TFppkgOptionSection;
    procedure LogValues(ALogLevel: TLogLevel);

    procedure BindToCompilerOptions(ACompilerOptions: TCompilerOptions);
    procedure AddRepositoriesForCompilerSettings(ACompilerOptions: TCompilerOptions);
    function AddRepositoryOptionSection(ASectionClass: TFppkgRepositoryOptionSectionClass): TFppkgRepositoryOptionSection;
    function AddIncludeFilesOptionSection(AFileMask: string): TFppkgIncludeFilesOptionSection;
    property PreferGlobal : Boolean Read FPreferGlobal Write FPreferGLobal;
    property SectionList: TFppkgOptionSectionList read GetSectionList;
    property GlobalSection: TFppkgGLobalOptionSection read GetGlobalSection;
    property CommandLineSection: TFppkgCommandLineOptionSection read GetCommandLineSection;
  end;

  { TCompilerOptions }

  TCompilerOptions = Class(TPersistent)
  private
    FConfigFilename: string;
    FSaveInifileChanges: Boolean;
    FConfigVersion : Integer;
    FCompiler,
    FCompilerVersion,
    FLocalInstallDir,
    FGlobalInstallDir,
    FLocalPrefix,
    FGlobalPrefix: String;
    FCompilerCPU: TCPU;
    FCompilerOS: TOS;
    FOptionParser: TTemplateParser;
    FOptions: TStrings;
    function GetOptions: TStrings;
    function GetOptString(Index: integer): String;
    procedure SetOptString(Index: integer; const AValue: String);
    procedure SetCompilerCPU(const AValue: TCPU);
    procedure SetCompilerOS(const AValue: TOS);
  Public
    Constructor Create;
    Destructor Destroy; override;
    Procedure InitCompilerDefaults;
    Procedure LoadCompilerFromFile(const AFileName : String);
    function SaveCompilerToFile(const AFileName : String): Boolean;
    procedure LogValues(ALogLevel: TLogLevel; const ACfgName:string);
    procedure UpdateLocalRepositoryOption(FppkgOptions: TFppkgOptions);
    procedure CheckCompilerValues;
    Function HasOptions: boolean;
    // Is set when the inifile has an old version number (which is also the case when a new file is generated)
    Property SaveInifileChanges : Boolean Read FSaveInifileChanges;
    Property ConfigVersion : Integer read FConfigVersion;
  Published
    Property Compiler : String Index 1 Read GetOptString Write SetOptString;
    Property CompilerTarget : String Index 2 Read GetOptString Write SetOptString;
    Property CompilerVersion : String Index 3 Read GetOptString Write SetOptString;
    Property GlobalInstallDir : String Index 4 Read GetOptString Write SetOptString; deprecated;
    Property LocalInstallDir : String Index 5 Read GetOptString Write SetOptString; deprecated;
    Property GlobalPrefix : String Index 6 Read GetOptString Write SetOptString;
    Property LocalPrefix : String Index 7 Read GetOptString Write SetOptString;
    Property Options : TStrings read GetOptions;
    Property CompilerOS : TOS Read FCompilerOS Write SetCompilerOS;
    Property CompilerCPU : TCPU Read FCompilerCPU Write SetCompilerCPU;
  end;

Implementation

uses
  pkgUninstalledSrcsRepo,
  pkgPackagesStructure,
  pkgmessages;

Const
  DefaultMirrorsURL  = 'https://www.freepascal.org/repository/'+MirrorsFileName;
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
  KeyDeprGlobalSection     = 'Defaults';
  KeyGlobalSection         = 'Global';
  KeyRepositorySection     = 'Repository';
  KeyIncludeFilesSection   = 'IncludeFiles';
  KeyRemoteMirrorsURL      = 'RemoteMirrors';
  KeyRemoteRepository      = 'RemoteRepository';
  KeyLocalRepository       = 'LocalRepository';
  KeyArchivesDir           = 'ArchivesDir';
  KeyBuildDir              = 'BuildDir';
  KeyCompilerConfigDir     = 'CompilerConfigDir';
  KeyCompilerConfig        = 'CompilerConfig';
  KeyFPMakeCompilerConfig  = 'FPMakeCompilerConfig';
  KeyDownloader            = 'Downloader';
  KeyCustomFPMakeOptions   = 'FPMakeOptions';
  KeyInstallRepository     = 'InstallRepository';

  KeyRepositoryName        = 'Name';
  KeyRepositoryDescription = 'Description';
  KeyRepositoryPath        = 'Path';
  KeyRepositoryPrefix      = 'Prefix';
  KeyInstallRepositoryName = 'InstallRepository';

  KeyIncludeFile           = 'File';
  KeyIncludeFileMask       = 'FileMask';

  // Compiler dependent config
  KeyGlobalPrefix          = 'GlobalPrefix';
  KeyLocalPrefix           = 'LocalPrefix';
  KeyGlobalInstallDir      = 'GlobalInstallDir';
  KeyLocalInstallDir       = 'LocalInstallDir';
  KeyCompiler              = 'Compiler' ;
  KeyCompilerOS            = 'OS';
  KeyCompilerCPU           = 'CPU';
  KeyCompilerVersion       = 'Version';
  KeyCompilerOptions       = 'CompilerOptions';

{ TFppkgIncludeFilesOptionSection }

procedure TFppkgIncludeFilesOptionSection.IncludeFile(AFileName: string);
begin
  AFileName := FOptionParser.ParseString(AFileName);
  if FileExists(AFileName) then
    begin
      FOptions.LoadFromFile(AFileName);
    end
  else
    log(llWarning, SLogIncludeFileDoesNotExist, [AFileName]);
end;

procedure TFppkgIncludeFilesOptionSection.IncludeFileMask(AFileNameMask: string);
var
  FileDir: string;
  SR: TSearchRec;
begin
  AFileNameMask := FOptionParser.ParseString(AFileNameMask);
  FileDir := IncludeTrailingPathDelimiter(ExtractFileDir(AFileNameMask));

  if IsRelativePath(AFileNameMask) then
    FileDir := ConcatPaths([FCurrentDir, FileDir]);

  if DirectoryExists(FileDir) then
    begin
      if IsRelativePath(AFileNameMask) then
        AFileNameMask := ConcatPaths([FCurrentDir, AFileNameMask]);

      if FindFirst(AFileNameMask, faAnyFile-faDirectory, SR)=0 then
        begin
          repeat
            IncludeFile(FileDir+SR.Name);
          until FindNext(SR)<>0;
        end;
      FindClose(SR);
    end
  else
    log(llWarning, SLogIncludeFileMaskDoesNotExist, [FileDir, AFileNameMask]);
end;

class function TFppkgIncludeFilesOptionSection.GetKey: string;
begin
  Result := KeyIncludeFilesSection;
end;

procedure TFppkgIncludeFilesOptionSection.SaveToStrings(AStrings: TStrings);
begin
  AStrings.Add('['+KeyIncludeFilesSection+']');
  AddKeyValueToStrings(AStrings, KeyIncludeFileMask, FCurrentDir);
end;

constructor TFppkgIncludeFilesOptionSection.Create(AnOptionParser: TTemplateParser;
  AnOptions: TFppkgOptions; ACurrentDir: string);
begin
  inherited Create(AnOptionParser);
  FOptions := AnOptions;
  FCurrentDir := ACurrentDir;
  FOptionCache := TStringList.Create;
end;

destructor TFppkgIncludeFilesOptionSection.Destroy;
begin
  FOptionCache.Free;
  inherited Destroy;
end;

procedure TFppkgIncludeFilesOptionSection.AddKeyValue(const AKey, AValue: string);
begin
  if SameText(AKey,KeyIncludeFile) then
    begin
      FOptionCache.Append(SLogIncludeFile + '=' + AValue);
      IncludeFile(AValue);
    end
  else if SameText(AKey,KeyIncludeFileMask) then
    begin
      FOptionCache.Append(SLogIncludeFileMask + '=' + AValue);
      IncludeFileMask(AValue);
    end;
end;

procedure TFppkgIncludeFilesOptionSection.LogValues(ALogLevel: TLogLevel);
var
  i: Integer;
begin
  inherited LogValues(ALogLevel);
  for i := 0 to FOptionCache.Count -1 do
    begin
      log(ALogLevel, FOptionCache.Names[i], [FOptionCache.ValueFromIndex[i], FOptionParser.ParseString(FOptionCache.ValueFromIndex[i])]);
    end;
end;

function TFppkgIncludeFilesOptionSection.AllowDuplicate: Boolean;
begin
  Result := inherited AllowDuplicate;
end;

{ TFppkgRepositoryOptionSection }

procedure TFppkgRepositoryOptionSection.SetDescription(AValue: string);
begin
  if FDescription = AValue then Exit;
  FDescription := AValue;
end;

procedure TFppkgRepositoryOptionSection.SetPrefix(AValue: string);
begin
  if FPrefix = AValue then Exit;
  FPrefix := AValue;
end;

function TFppkgRepositoryOptionSection.GetPath: string;
begin
  Result := OptionParser.ParseString(FPath);
end;

function TFppkgRepositoryOptionSection.GetPrefix: string;
begin
  Result := OptionParser.ParseString(FPrefix);
end;

procedure TFppkgRepositoryOptionSection.SetRepositoryName(AValue: string);
begin
  if FRepositoryName = AValue then Exit;
  FRepositoryName := AValue;
end;

procedure TFppkgRepositoryOptionSection.SetPath(AValue: string);
begin
  if FPath = AValue then Exit;
  FPath := fpmkunit.FixPath(AValue, True);
end;

procedure TFppkgRepositoryOptionSection.SaveToStrings(AStrings: TStrings);
begin
  inherited SaveToStrings(AStrings);
  AddKeyValueToStrings(AStrings, KeyRepositoryName, RepositoryName);
  AddKeyValueToStrings(AStrings, KeyRepositoryDescription, Description);
  AddKeyValueToStrings(AStrings, KeyRepositoryPath, FPath);
  AddKeyValueToStrings(AStrings, KeyRepositoryPrefix, FPrefix);
  AddKeyValueToStrings(AStrings, KeyInstallRepositoryName, InstallRepositoryName);
end;

class function TFppkgRepositoryOptionSection.GetKey: string;
begin
  Result := KeyRepositorySection;
end;

procedure TFppkgRepositoryOptionSection.AddKeyValue(const AKey, AValue: string);
begin
  if SameText(AKey,KeyRepositoryName) then
    RepositoryName := AValue
  else if SameText(AKey,KeyRepositoryDescription) then
    Description := AValue
  else if SameText(AKey,KeyRepositoryPath) then
    Path := AValue
  else if SameText(AKey,KeyRepositoryPrefix) then
    Prefix := AValue
  else if SameText(AKey,KeyInstallRepositoryName) then
    InstallRepositoryName := AValue
end;

procedure TFppkgRepositoryOptionSection.LogValues(ALogLevel: TLogLevel);
begin
  inherited LogValues(ALogLevel);
  log(ALogLevel,SLogRepositoryName,[FRepositoryName]);
  log(ALogLevel,SLogRepositoryDescription,[FDescription]);
  log(ALogLevel,SLogRepositoryPath,[FPath,Path]);
  log(ALogLevel,SLogRepositoryPrefix,[FPrefix,Prefix]);
  log(ALogLevel,SLogInstallRepository,[FInstallRepositoryName]);
end;

function TFppkgRepositoryOptionSection.AllowDuplicate: Boolean;
begin
  Result := True;
end;

function TFppkgRepositoryOptionSection.GetRepositoryType: TFPRepositoryType;
begin
  result := fprtInstalled;
end;

{ TFppkgCommandLineOptionSection }

constructor TFppkgCommandLineOptionSection.Create(AnOptionParser: TTemplateParser);
begin
  inherited Create(AnOptionParser);
  // Parameter defaults
  FRecoveryMode:=False;
  FAllowBroken:=False;
end;

{ TFppkgOptionSection }

class function TFppkgOptionSection.GetKey: string;
begin
  Result := '';
end;

procedure TFppkgOptionSection.AddKeyValueToStrings(AStrings: TStrings; AKey, AValue: string);
begin
  if AValue<>'' then
    AStrings.Add(AKey+'='+AValue);
end;

procedure TFppkgOptionSection.AddKeyValueToStrings(AStrings: TStrings; AKey: string; AValue: Integer);
begin
  if AValue<>-1 then
    AStrings.Add(AKey+'='+IntToStr(AValue));
end;

constructor TFppkgOptionSection.Create(AnOptionParser: TTemplateParser);
begin
  FOptionParser:=AnOptionParser;
end;

procedure TFppkgOptionSection.AddKeyValue(const AKey, AValue: string);
begin
  // Do nothing
end;

procedure TFppkgOptionSection.SaveToStrings(AStrings: TStrings);
begin
  AStrings.Add('['+GetKey+']');
end;

procedure TFppkgOptionSection.LogValues(ALogLevel: TLogLevel);
begin
  log(ALogLevel,SLogCfgSectionHeader, [Trim(Name)]);
end;

function TFppkgOptionSection.AllowDuplicate: Boolean;
begin
  Result:=False;
end;

{*****************************************************************************
                      TFppkgGlobalOptionSection
*****************************************************************************}

procedure TFppkgGlobalOptionSection.SetBuildDir(AValue: string);
begin
  if FBuildDir = AValue then Exit;
  FBuildDir := fpmkunit.FixPath(AValue, True);
end;

function TFppkgGlobalOptionSection.GetCompilerConfigDir: string;
begin
  Result := FOptionParser.ParseString(FCompilerConfigDir);
end;

function TFppkgGlobalOptionSection.GetLocalRepository: string;
begin
  Result := FOptionParser.ParseString(FLocalRepository);
end;

procedure TFppkgGlobalOptionSection.SetArchivesDir(AValue: string);
begin
  if FArchivesDir = AValue then Exit;
  FArchivesDir := fpmkunit.FixPath(AValue, True);
end;

function TFppkgGlobalOptionSection.GetBuildDir: string;
begin
  Result := FOptionParser.ParseString(FBuildDir);
end;

function TFppkgGlobalOptionSection.GetArchivesDir: string;
begin
  Result := FOptionParser.ParseString(FArchivesDir);
end;

procedure TFppkgGlobalOptionSection.SetCompilerConfigDir(AValue: string);
begin
  if FCompilerConfigDir = AValue then Exit;
  FCompilerConfigDir := fpmkunit.FixPath(AValue, True);
end;

procedure TFppkgGlobalOptionSection.SetConfigVersion(AValue: integer);
begin
  if FConfigVersion = AValue then Exit;
  FConfigVersion := AValue;
end;

procedure TFppkgGlobalOptionSection.SetCompilerConfig(AValue: string);
begin
  if FCompilerConfig = AValue then Exit;
  FCompilerConfig := AValue;
end;

procedure TFppkgGlobalOptionSection.SetCustomFPMakeOptions(AValue: string);
begin
  if FCustomFPMakeOptions = AValue then Exit;
  FCustomFPMakeOptions := AValue;
end;

procedure TFppkgGlobalOptionSection.SetDownloader(AValue: string);
begin
  if FDownloader = AValue then Exit;
  FDownloader := AValue;
end;

procedure TFppkgGlobalOptionSection.SetFPMakeCompilerConfig(AValue: string);
begin
  if FFPMakeCompilerConfig = AValue then Exit;
  FFPMakeCompilerConfig := AValue;
end;

procedure TFppkgGlobalOptionSection.SetLocalRepository(AValue: string);
begin
  if FLocalRepository = AValue then Exit;
  FLocalRepository := AValue;
  FOptionParser.Values['LocalRepository'] := LocalRepository;
end;

procedure TFppkgGlobalOptionSection.SetRemoteMirrorsURL(AValue: string);
begin
  if FRemoteMirrorsURL = AValue then Exit;
  FRemoteMirrorsURL := AValue;
end;

procedure TFppkgGlobalOptionSection.SetRemoteRepository(AValue: string);
begin
  if FRemoteRepository = AValue then Exit;
  FRemoteRepository := AValue;
end;

constructor TFppkgGlobalOptionSection.Create(AnOptionParser: TTemplateParser);
begin
  Inherited Create(AnOptionParser);
  // Retrieve Local fppkg directory
{$ifdef unix}
  if IsSuperUser then
    begin
      if DirectoryExists('/usr/local/lib/fpc') then
        LocalRepository:='/usr/local/lib/fpc/fppkg/'
      else
        LocalRepository:='/usr/lib/fpc/fppkg/';
    end
  else
    LocalRepository:='{UserDir}.fppkg/';
{$else}
  if IsSuperUser then
    LocalRepository:=IncludeTrailingPathDelimiter(GetFppkgConfigDir(true))
  else
    LocalRepository:='{AppConfigDir}';
{$endif}

  ConfigVersion := CurrentConfigVersion;
  CompilerConfig := 'default';
  FPMakeCompilerConfig := 'default';
  RemoteRepository := DefaultRemoteRepository;
  FRemoteMirrorsURL:=DefaultMirrorsURL;

  // Directories
  BuildDir:='{LocalRepository}build'+PathDelim;
  ArchivesDir:='{LocalRepository}archives'+PathDelim;
  CompilerConfigDir:='{LocalRepository}config'+PathDelim;
{$if (defined(unix) and not defined(android)) or defined(windows)}
  Downloader:='FPC';
{$else}
  Downloader:='base';
{$endif}
end;

destructor TFppkgGlobalOptionSection.Destroy;
begin
  inherited Destroy;
end;

procedure TFppkgGlobalOptionSection.AddKeyValue(const AKey, AValue: string);
begin
  if SameText(AKey,KeyBuildDir) then
    BuildDir := AValue
  else if SameText(AKey,KeyDownloader) then
    Downloader := AValue
  else if SameText(AKey,KeyConfigVersion) then
    begin
      ConfigVersion := StrToIntDef(AValue,-1);
      if (FConfigVersion<>CurrentConfigVersion) then
        begin
          if (FConfigVersion<MinSupportedConfigVersion) or (FConfigVersion>CurrentConfigVersion) then
            Error(SErrUnsupportedConfigVersion);
          log(llWarning,SLogOldConfigFileFormat);
        end;
    end
  else if SameText(AKey,KeyCompilerConfig) then
    CompilerConfig := AValue
  else if SameText(AKey,KeyFPMakeCompilerConfig) then
    FPMakeCompilerConfig := AValue
  else if SameText(AKey,KeyCompilerConfigDir) then
    CompilerConfigDir := AValue
  else if SameText(AKey,KeyRemoteMirrorsURL) then
    RemoteMirrorsURL := AValue
  else if SameText(AKey,KeyRemoteRepository) then
    RemoteRepository := AValue
  else if SameText(AKey,KeyLocalRepository) then
    LocalRepository := AValue
  else if SameText(AKey,KeyArchivesDir) then
    ArchivesDir := AValue
  else if SameText(AKey,KeyInstallRepository) then
    InstallRepository := AValue
  else if SameText(AKey,KeyCustomFPMakeOptions) then
    CustomFPMakeOptions := AValue
end;

procedure TFppkgGlobalOptionSection.SaveToStrings(AStrings: TStrings);
begin
  AStrings.Add('['+KeyGlobalSection+']');
  AddKeyValueToStrings(AStrings, KeyConfigVersion, CurrentConfigVersion);
  AddKeyValueToStrings(AStrings, KeyBuildDir, BuildDir);
  AddKeyValueToStrings(AStrings, KeyDownloader, Downloader);
  AddKeyValueToStrings(AStrings, KeyCompilerConfig, CompilerConfig);
  AddKeyValueToStrings(AStrings, KeyFPMakeCompilerConfig, FPMakeCompilerConfig);
  AddKeyValueToStrings(AStrings, KeyCompilerConfigDir, CompilerConfigDir);
  AddKeyValueToStrings(AStrings, KeyRemoteMirrorsURL, RemoteMirrorsURL);
  AddKeyValueToStrings(AStrings, KeyRemoteRepository, RemoteRepository);
  AddKeyValueToStrings(AStrings, KeyLocalRepository, LocalRepository);
  AddKeyValueToStrings(AStrings, KeyArchivesDir, ArchivesDir);
  AddKeyValueToStrings(AStrings, KeyCustomFPMakeOptions, CustomFPMakeOptions);
end;

procedure TFppkgGlobalOptionSection.LogValues(ALogLevel: TLogLevel);
begin
  inherited LogValues(ALogLevel);
  log(ALogLevel,SLogGlobalCfgRemoteMirrorsURL,[FRemoteMirrorsURL]);
  log(ALogLevel,SLogGlobalCfgRemoteRepository,[FRemoteRepository]);
  log(ALogLevel,SLogGlobalCfgLocalRepository,[FLocalRepository,LocalRepository]);
  log(ALogLevel,SLogGlobalCfgBuildDir,[FBuildDir,BuildDir]);
  log(ALogLevel,SLogGlobalCfgArchivesDir,[FArchivesDir,ArchivesDir]);
  log(ALogLevel,SLogGlobalCfgCompilerConfigDir,[FCompilerConfigDir,CompilerConfigDir]);
  log(ALogLevel,SLogGlobalCfgDefaultCompilerConfig,[FCompilerConfig]);
  log(ALogLevel,SLogGlobalCfgFPMakeCompilerConfig,[FPMakeCompilerConfig]);
  log(ALogLevel,SLogGlobalCfgDownloader,[FDownloader]);
end;

function TFppkgGlobalOptionSection.LocalMirrorsFile: string;
begin
  Result:=LocalRepository+MirrorsFileName;
end;

function TFppkgGlobalOptionSection.LocalPackagesFile: string;
begin
  Result:=LocalRepository+PackagesFileName;
end;

{*****************************************************************************
                            TFppkgOptions
*****************************************************************************}

function TFppkgOptions.GetSectionList: TFppkgOptionSectionList;
begin
  Result := FSectionList;
end;

function TFppkgOptions.GetGlobalSection: TFppkgGLobalOptionSection;
begin
  Result := GetSectionByName(KeyGlobalSection) as TFppkgGlobalOptionSection;
  // Below version 5 the global-section was called 'Defaults'
  if not Assigned(Result) then
    Result := GetSectionByName(KeyDeprGlobalSection) as TFppkgGlobalOptionSection;

  if not Assigned(Result) then
    begin
      Result := TFppkgGlobalOptionSection.Create(FOptionParser);
      Result.Name := KeyGlobalSection;
      FSectionList.Add(Result);
    end;
end;

function TFppkgOptions.GetCommandLineSection: TFppkgCommandLineOptionSection;
begin
  Result := GetSectionByName(' Commandline ') as TFppkgCommandLineOptionSection;
  if not Assigned(Result) then
    begin
      Result := TFppkgCommandLineOptionSection.Create(FOptionParser);
      Result.Name := ' Commandline ';
      FSectionList.Add(Result);
    end;
end;

constructor TFppkgOptions.Create();
begin
  FOptionParser := TTemplateParser.Create;
  FOptionParser.Values['AppConfigDir'] := GetFppkgConfigDir(false);
  FOptionParser.Values['UserDir'] := GetUserDir;

  FSectionList := TFppkgOptionSectionList.Create;
end;

destructor TFppkgOptions.Destroy;
begin
  FSectionList.Free;
  FOptionParser.Free;
  inherited Destroy;
end;

procedure TFppkgOptions.LoadFromFile(const AFileName: string);
var
  IniFile: TStringList;
  CurrentSection: TFppkgOptionSection;
  s: String;
  i: Integer;
  j: SizeInt;
begin
  log(llInfo, SLogStartLoadingConfFile, [AFileName]);
  IniFile:=TStringList.Create;
  try
    Inifile.LoadFromFile(AFileName);
    for i := 0 to Inifile.Count-1 do
      begin
        s := Trim(IniFile[i]);
        if s = '' then
          Continue;
        if (Copy(s, 1, 1) = '[') and (Copy(s, length(s), 1) = ']') then
          begin
            s := Trim(Copy(s, 2, Length(s) - 2));
            CurrentSection := GetSectionByName(s);
            if not Assigned(CurrentSection) or CurrentSection.AllowDuplicate then
              begin
                if SameText(s, KeyGlobalSection) or SameText(s, KeyDeprGlobalSection) then
                  CurrentSection := GetGlobalSection
                else
                  begin
                    if SameText(s, TFppkgRepositoryOptionSection.GetKey) then
                      CurrentSection := TFppkgRepositoryOptionSection.Create(FOptionParser)
                    else if SameText(s, TFppkgUninstalledSourceRepositoryOptionSection.GetKey) then
                      CurrentSection := TFppkgUninstalledSourceRepositoryOptionSection.Create(FOptionParser)
                    else if SameText(s, TFppkgIncludeFilesOptionSection.GetKey) then
                      CurrentSection := TFppkgIncludeFilesOptionSection.Create(FOptionParser, Self, ExtractFileDir(AFileName))
                    else if SameText(s, TFppkgUninstalledRepositoryOptionSection.GetKey) then
                      CurrentSection := TFppkgUninstalledRepositoryOptionSection.Create(FOptionParser)
                    else
                      CurrentSection := TFppkgCustomOptionSection.Create(FOptionParser);
                    FSectionList.Add(CurrentSection);
                    CurrentSection.Name := s;
                  end;
              end
          end
        else if copy(s,1,1)<>';' then // comment
          begin
            // regular key
            j:=Pos('=', s);
            if j>0 then
              CurrentSection.AddKeyValue(Trim(Copy(s, 1,  j - 1)), Trim(Copy(s, j + 1, Length(s) - j)));
          end;
      end;
  finally
    Inifile.Free;
  end;
end;

procedure TFppkgOptions.SaveToFile(const AFileName: string);
var
  IniFile: TStringList;
  CurrentSection: TFppkgOptionSection;
  GSection: TFppkgGlobalOptionSection;
  i: Integer;
begin
  IniFile:=TStringList.Create;
  try
    GSection := GlobalSection;
    GSection.SaveToStrings(IniFile);

    for i := 0 to SectionList.Count-1 do
      begin
        CurrentSection := SectionList[i];
        if CurrentSection<>GSection then
          begin
            IniFile.Add('');
            CurrentSection.SaveToStrings(IniFile);
          end;
      end;

    Inifile.SaveToFile(AFileName);
  finally
    Inifile.Free;
  end;
end;

function TFppkgOptions.GetSectionByName(const SectionName: string): TFppkgOptionSection;
var
  i: Integer;
begin
  Result := nil;
  for i := 0 to SectionList.Count-1 do
    begin
      if SectionList[i].Name=SectionName then
        begin
          Result:=SectionList[i];
          Break;
        end;
    end;
end;

procedure TFppkgOptions.LogValues(ALogLevel: TLogLevel);
var
  i: Integer;
begin
  log(ALogLevel,SLogCfgHeader);
  for i := 0 to SectionList.Count-1 do
    begin
      SectionList[i].LogValues(ALogLevel);
    end;
end;

procedure TFppkgOptions.BindToCompilerOptions(ACompilerOptions: TCompilerOptions);
begin
  FOptionParser.Values['CompilerVersion'] := ACompilerOptions.CompilerVersion;
end;

procedure TFppkgOptions.AddRepositoriesForCompilerSettings(
  ACompilerOptions: TCompilerOptions);
var
  CurrentSection: TFppkgRepositoryOptionSection;
begin
  CurrentSection := TFppkgRepositoryOptionSection.Create(FOptionParser);
  CurrentSection.RepositoryName:='global';
  CurrentSection.Description:='global';
  CurrentSection.Path:=ACompilerOptions.GlobalInstallDir;
  CurrentSection.Prefix:=ACompilerOptions.GlobalPrefix;
  FSectionList.Add(CurrentSection);

  CurrentSection := TFppkgRepositoryOptionSection.Create(FOptionParser);
  CurrentSection.RepositoryName:='local';
  CurrentSection.Description:='local';
  CurrentSection.Path:=ACompilerOptions.LocalInstallDir;
  CurrentSection.Prefix:=ACompilerOptions.LocalPrefix;
  FSectionList.Add(CurrentSection);

  if CommandLineSection.InstallRepository='' then
    begin
      if IsSuperUser then
        CommandLineSection.InstallRepository:='global'
      else
        CommandLineSection.InstallRepository:='local';
    end;
end;

function TFppkgOptions.AddRepositoryOptionSection(ASectionClass: TFppkgRepositoryOptionSectionClass): TFppkgRepositoryOptionSection;
begin
  Result := ASectionClass.Create(FOptionParser);
  FSectionList.Add(Result);
end;

function TFppkgOptions.AddIncludeFilesOptionSection(AFileMask: string): TFppkgIncludeFilesOptionSection;
begin
  Result := TFppkgIncludeFilesOptionSection.Create(FOptionParser, Self, AFileMask);
  FSectionList.Add(Result);
end;

{*****************************************************************************
                           TCompilerOptions
*****************************************************************************}

constructor TCompilerOptions.Create;
begin
  FOptionParser := TTemplateParser.Create;
  FOptionParser.Values['AppConfigDir'] := GetFppkgConfigDir(false);
  FOptionParser.Values['UserDir'] := GetUserDir;
  FSaveInifileChanges := True;
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
    6 : Result:=fpmkunit.FixPath(FOptionParser.ParseString(FGlobalPrefix), True);
    7 : Result:=fpmkunit.FixPath(FOptionParser.ParseString(FLocalPrefix), True);
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
    4 : FGlobalInstallDir:=fpmkunit.FixPath(AValue, True);
    5 : FLocalInstallDir:=fpmkunit.FixPath(AValue, True);
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


procedure TCompilerOptions.UpdateLocalRepositoryOption(FppkgOptions: TFppkgOptions);
begin
  FOptionParser.Values['LocalRepository'] := FppkgOptions.GlobalSection.LocalRepository;
end;

procedure TCompilerOptions.CheckCompilerValues;
var
  AVersion : string;
  ACpu     : TCpu;
  AOs      : TOS;
begin
  if Compiler='' then
    Exit;
  // This is not the place to complain when the compiler does
  // not exist at all.
  if not FileExists(Compiler) then
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
  end;
end;


procedure TCompilerOptions.SetCompilerOS(const AValue: TOS);
begin
  if FCompilerOS=AValue then
    exit;
  FCompilerOS:=AValue;
end;

function TCompilerOptions.HasOptions: boolean;
begin
  result := assigned(FOptions);
end;


procedure TCompilerOptions.InitCompilerDefaults;
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
  // Temporary hack to workaround bug in fpc.exe that doesn't support spaces
  // We retrieve the real binary
  if FCompilerVersion='2.2.0' then
    FCompiler:=GetCompilerInfo(FCompiler,'-PB');
  log(llDebug,SLogDetectedCompiler,[FCompiler,FCompilerVersion,MakeTargetString(FCompilerCPU,FCompilerOS)]);

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

  log(llDebug,SLogDetectedPrefix,['global',FGlobalPrefix]);
  // User writable install directory
  if not IsSuperUser then
    begin
      FLocalPrefix:= '{LocalRepository}';
      log(llDebug,SLogDetectedPrefix,['local',FLocalPrefix]);
    end;

  fpcdir:=fpmkunit.FixPath(GetEnvironmentVariable('FPCDIR'), True);
  if fpcdir<>'' then
    begin
    {$ifndef Unix}
    fpcdir:=ExpandFileName(fpcdir);
    {$endif unix}
    log(llDebug,SLogFPCDirEnv,[fpcdir]);
    FGlobalInstallDir:=fpcdir;
    end;
end;


procedure TCompilerOptions.LoadCompilerFromFile(const AFileName: String);
Var
  Ini : TMemIniFile;
  S,sOptions : UTF8String;

begin
  Ini:=TMemIniFile.Create(AFileName);
  try
    FConfigFilename:=AFileName;
    With Ini do
      begin
        FConfigVersion:=ReadInteger(SDefaults,KeyConfigVersion,0);
        if (FConfigVersion<>CurrentConfigVersion) then
          begin
            log(llDebug,SLogUpgradingConfig,[AFileName]);
            FSaveInifileChanges:=true;
            if (FConfigVersion>CurrentConfigVersion) then
              Error(SErrUnsupportedConfigVersion,[AFileName]);
          end
        else
          begin
            FSaveInifileChanges:=False;
          end;
        GlobalPrefix:=ReadString(SDefaults,KeyGlobalPrefix,FGlobalPrefix);
        LocalPrefix:=ReadString(SDefaults,KeyLocalPrefix,FLocalPrefix);
        FGlobalInstallDir:=fpmkunit.FixPath(ReadString(SDefaults,KeyGlobalInstallDir,FGlobalInstallDir), True);
        FLocalInstallDir:=fpmkunit.FixPath(ReadString(SDefaults,KeyLocalInstallDir,FLocalInstallDir), True);
        FCompiler:=ReadString(SDefaults,KeyCompiler,FCompiler);
        FCompilerOS:=StringToOS(ReadString(SDefaults,KeyCompilerOS,OSToString(CompilerOS)));
        FCompilerCPU:=StringToCPU(ReadString(SDefaults,KeyCompilerCPU,CPUtoString(CompilerCPU)));
        sOptions:=ReadString(SDefaults,KeyCompilerOptions,'');
        if (sOptions<>'') then
          for S in SplitCommandline(sOptions) do
             self.Options.Add(S);
        CompilerVersion:=ReadString(SDefaults,KeyCompilerVersion,FCompilerVersion);
      end;
  finally
    Ini.Free;
  end;
end;


function TCompilerOptions.SaveCompilerToFile(const AFileName: String): Boolean;
Var
  Ini : TIniFile;
begin
  Result := False;
  try
    if FileExists(AFileName) then
      BackupFile(AFileName);
    Ini:=TIniFile.Create(AFileName);
    try
      With Ini do
        begin
          WriteInteger(SDefaults,KeyConfigVersion,CurrentConfigVersion);
          WriteString(SDefaults,KeyGlobalPrefix,FGlobalPrefix);
          WriteString(SDefaults,KeyLocalPrefix,FLocalPrefix);
          WriteString(SDefaults,KeyGlobalInstallDir,FGlobalInstallDir);
          WriteString(SDefaults,KeyLocalInstallDir,FLocalInstallDir);
          WriteString(SDefaults,KeyCompiler,FCompiler);
          WriteString(SDefaults,KeyCompilerOS,OSToString(CompilerOS));
          WriteString(SDefaults,KeyCompilerCPU,CPUtoString(CompilerCPU));
          WriteString(SDefaults,KeyCompilerVersion,FCompilerVersion);
          if HasOptions then
            WriteString(SDefaults,KeyCompilerOptions,Self.Options.Text);
          FSaveInifileChanges:=False;
        end;
      Ini.UpdateFile;
    finally
      Ini.Free;
    end;
    Result := True;
  except
    on E: Exception do
      log(llWarning, SWarnFailedToWriteCompConf, [AFileName, E.Message]);
  end;
end;


procedure TCompilerOptions.LogValues(ALogLevel: TLogLevel; const ACfgName:string);
begin
  log(ALogLevel,SLogCompilerCfgHeader,[ACfgName,FConfigFilename]);
  log(ALogLevel,SLogCompilerCfgCompiler,[FCompiler]);
  log(ALogLevel,SLogCompilerCfgTarget,[MakeTargetString(CompilerCPU,CompilerOS)]);
  log(ALogLevel,SLogCompilerCfgVersion,[FCompilerVersion]);
  log(ALogLevel,SLogCompilerCfgGlobalPrefix,[FGlobalPrefix,GlobalPrefix]);
  log(ALogLevel,SLogCompilerCfgLocalPrefix,[FLocalPrefix,LocalPrefix]);
  log(ALogLevel,SLogCompilerCfgGlobalInstallDir,[FGlobalInstallDir,GlobalInstallDir]);
  log(ALogLevel,SLogCompilerCfgLocalInstallDir,[FLocalInstallDir,LocalInstallDir]);
  log(ALogLevel,SLogCompilerCfgOptions,[Options.DelimitedText]);
end;

end.
