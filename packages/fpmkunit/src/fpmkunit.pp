{
    This file is part of the Free Pascal Makefile Package

    Implementation of fpmake classes and functions

    Copyright (c) 2007 by the freepascal team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit fpmkunit;

{$Mode objfpc}
{$H+}
{$inline on}
{$MODESWITCH TYPEHELPERS}

{ For target or cpu dependent dependencies also add an overload where you
  can pass only a set of cpus. This is disabled for now because it creates
  an error in the compiler with overload choosing }
{ define cpu_only_overloads}

Interface

{$IFDEF MORPHOS}
 {$DEFINE NO_UNIT_PROCESS}
 {$DEFINE NO_THREADING}
{$ENDIF}

{$IFDEF AMIGA}
 {$DEFINE NO_UNIT_PROCESS}
 {$DEFINE NO_THREADING}
{$ENDIF}

{$IFDEF AROS}
 {$DEFINE NO_UNIT_PROCESS}
 {$DEFINE NO_THREADING}
{$ENDIF}

{$IFDEF OS2}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF OS2}

{$IFDEF GO32V2}
 {$DEFINE NO_UNIT_PROCESS}
 {$DEFINE NO_THREADING}
{$ENDIF GO32V2}

{$IFDEF NDS}
 {$DEFINE NO_THREADING}
{$ENDIF NDS}

{$IFDEF NETBSD}
 { NetBSD pthreads are not yet working, try to use fpmake without threads }
  {$DEFINE NO_THREADING}
{$ENDIF NETBSD}

{$ifndef NO_UNIT_PROCESS}
  {$define HAS_UNIT_PROCESS}
{$endif NO_UNIT_PROCESS}

{$ifndef NO_UNIT_ZIPPER}
  {$define HAS_UNIT_ZIPPER}
{$endif NO_UNIT_ZIPPER}

{$ifndef NO_TAR_SUPPORT}
  {$define HAS_TAR_SUPPORT}
{$endif NO_TAR_SUPPORT}

{$ifdef unix}
  {$ifdef HAS_TAR_SUPPORT}
    {$define CREATE_TAR_FILE}
  {$endif HAS_TAR_SUPPORT}
{$endif unix}

uses
{$ifdef UNIX}
  BaseUnix,
{$endif UNIX}
{$ifndef NO_THREADING}
{$ifdef UNIX}
  cthreads,
{$endif UNIX}
{$ifdef WINDOWS}
  windows,
{$endif WINDOWS}
{$endif NO_THREADING}
  SysUtils, Classes
{$ifdef HAS_UNIT_PROCESS}
  ,process
{$endif HAS_UNIT_PROCESS}
{$ifdef HAS_TAR_SUPPORT}
  ,libtar
{$endif HAS_TAR_SUPPORT}
{$ifdef HAS_UNIT_ZIPPER}
  ,zipper, zstream
{$endif HAS_UNIT_ZIPPER}
  ;

Type
  TFileType = (ftSource,ftUnit,ftObject,ftResource,ftExecutable,ftStaticLibrary,
               ftSharedLibrary);
  TFileTypes = set of TFileType;

  // Please keep this order, see OSCPUSupported below
  TCpu=(cpuNone,
    i386,m68k,powerpc,sparc,x86_64,arm,powerpc64,avr,armeb,
    mips,mipsel,mips64,mips64el,jvm,i8086,aarch64,cpuwasm,sparc64,riscv32,riscv64,xtensa,z80
  );
  TCPUS = Set of TCPU;

  // Please keep this order, see OSCPUSupported below
  TOS=(osNone,
    linux,go32v2,win32,os2,freebsd,beos,netbsd,
    amiga,atari, solaris, qnx, netware, openbsd,wdosx,
    palmos,macosclassic,darwin,emx,watcom,morphos,netwlibc,
    win64,wince,gba,nds,embedded,symbian,haiku,iphonesim,
    aix,java,android,nativent,msdos,wii,aros,dragonfly,
    win16,wasm,freertos,zxspectrum,msxdos,ios,amstradcpc,sinclairql
  );
  TOSes = Set of TOS;

  TCompilerMode = (cmFPC,cmTP,cmObjFPC,cmDelphi,cmMacPas);
  TCompilerModes = Set of TCompilerMode;

  TInstallMOde = (imInstall, imUnInstall);

  TTargetType = (ttProgram,ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit,ttExampleProgram,ttFPDoc,ttSharedLibrary);
  TTargetTypes = set of TTargetType;

  TFPDocFormat = (ffHtml, ffHtm, ffXHtml, ffLaTex, ffXMLStruct, ffChm);
  TFPDocFormats = set of TFPDocFormat;

  TTargetState = (tsNeutral,tsConsidering,tsNoCompile,tsCompiled,tsInstalled,tsNotFound);
  TTargetStates = Set of TTargetState;

  TSourceType = (stDoc,stSrc,stExample,stTest);
  TSourceTypes = set of TSourceType;

  TVerboseLevel = (vlError,vlWarning,vlInfo,vldebug,vlCommand);
  TVerboseLevels = Set of TVerboseLevel;

  TCommandAt = (caBeforeCompile,caAfterCompile,
                caBeforeInstall,caAfterInstall,
                caBeforeArchive,caAfterArchive,
                caBeforeClean,caAfterClean,
                caBeforeDownload,caAfterDownload);

  TDependencyType = (depPackage,depImplicitPackage,depUnit,depInclude);
  TDependencyTypes = set of TDependencyType;

  TLogEvent = Procedure (Level : TVerboseLevel; Const Msg : String) of Object;
  TNotifyProcEvent = procedure(Sender: TObject);

  TRunMode = (rmCompile,rmBuild,rmInstall,rmBuildInstall,rmArchive,rmClean,rmDistClean,rmManifest,rmZipInstall,rmPkgList,rmUnInstall,rmInfo,rmDocProject);

  TBuildMode = (bmOneByOne, bmBuildUnit{, bmSkipImplicitUnits});
  TBuildModes = set of TBuildMode;
  TProcessPackageResult = (ppHandled, ppDelayed);
  TCheckDependencyResult = (cdAvailable, cdNotAvailable, cdNotYetAvailable);

Const
  // Aliases
  Amd64   = X86_64;
  PPC = PowerPC;
  PPC64 = PowerPC64;
  DOS = Go32v2;
  MacOSX = Darwin;

  AllOSes = [Low(TOS)..High(TOS)];
  AllCPUs = [Low(TCPU)..High(TCPU)];
  AllUnixOSes  = [Linux,FreeBSD,NetBSD,OpenBSD,Darwin,QNX,BeOS,Solaris,Haiku,iphonesim,ios,aix,Android,dragonfly];
  AllBSDOSes      = [FreeBSD,NetBSD,OpenBSD,Darwin,iphonesim,ios,dragonfly];
  AllWindowsOSes  = [Win32,Win64,WinCE];
  AllAmigaLikeOSes = [Amiga,MorphOS,AROS];
  AllLimit83fsOses = [go32v2,os2,emx,watcom,msdos,win16,atari];

  AllSmartLinkLibraryOSes = [Linux,msdos,win16,palmos]; // OSes that use .a library files for smart-linking
  AllImportLibraryOSes = AllWindowsOSes + [os2,emx,netwlibc,netware,watcom,go32v2,macosclassic,nativent,msdos,win16];

  { This table is kept OS,Cpu because it is easier to maintain (PFV) }
  OSCPUSupported : array[TOS,TCpu] of boolean = (
    { os          none   i386    m68k  ppc    sparc  x86_64 arm    ppc64  avr    armeb  mips   mipsel mips64 mips64el jvm    i8086 aarch64 wasm   sparc64 riscv32 riscv64  xtensa z80}
    { none }    ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { linux }   ( false, true,  true,  true,  true,  true,  true,  true,  false, true , true , true , true , true ,   false, false, true , false, true ,  true ,  true,    true , false),
    { go32v2 }  ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { win32 }   ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { os2 }     ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { freebsd } ( false, true,  true,  false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { beos }    ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { netbsd }  ( false, true,  true,  true,  true,  true,  true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { amiga }   ( false, false, true,  true,  false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { atari }   ( false, false, true,  false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { solaris } ( false, true,  false, false, true,  true,  false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { qnx }     ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { netware } ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { openbsd } ( false, true,  true,  false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { wdosx }   ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { palmos }  ( false, false, true,  false, false, false, true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
{ macosclassic }( false, false, true,  true,  false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { darwin }  ( false, true,  false, true,  false, true,  false,  true, false, false, false, false, false, false,   false, false, true , false, false,  false,  false,   false, false),
    { emx }     ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { watcom }  ( false, true,  false, false, false ,false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { morphos } ( false, false, false, true,  false ,false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { netwlibc }( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { win64   } ( false, false, false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, true,  false, false,  false,  false,   false, false),
    { wince    }( false, true,  false, false, false, false, true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { gba    }  ( false, false, false, false, false, false, true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { nds    }  ( false, false, false, false, false, false, true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { embedded }( false, true,  true,  true,  true,  true,  true,  true,  true,  true , false, true,  false, true,    false, true , false, false, false,  true,   true,    true , false),
    { symbian } ( false, true,  false, false, false, false, true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { haiku }   ( false, true,  false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { iphonesim}( false, true,  false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { aix    }  ( false, false, false, true,  false, false, false, true,  false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { java }    ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   true , false, false, false, false,  false,  false,   false, false),
    { android } ( false, true,  false, false, false, true,  true,  false, false, false, false, true,  false, true,    true , false, true,  false, false,  false,  false,   false, false),
    { nativent }( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { msdos }   ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, true , false, false, false,  false,  false,   false, false),
    { wii }     ( false, false, false, true , false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { aros }    ( false, true,  false, false, false, true,  true,  false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { dragonfly}( false, false, false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false),
    { win16 }   ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, true , false, false, false,  false,  false,   false, false),
    { wasm }    ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, true,  false,  false,  false,   false, false),
    { freertos }( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   true , false),
    {zxspectrum}( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, true ),
    { msxdos }  ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, true ),
    { ios }     ( false, false, false, false, false, false,  true, false, false, false, false, false, false, false,   false, false, true , false, false,  false,  false,   false, false),
    {amstradcpc}( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, true ),
    {sinclairql}( false, false, true,  false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false,  false,  false,   false, false)
  );

  // Useful
  UnitExt = '.ppu';
  PPUExt  = UnitExt;
  PasExt  = '.pas';
  PPExt   = '.pp';
  IncExt  = '.inc';
  ObjExt  = '.o';
  LTOExt  = '.bc';
  RstExt  = '.rst';
  RsjExt  = '.rsj';
  LibExt  = '.a';
  SharedLibExt = '.so';
  DyLibExt = '.dylib';
  DLLExt  = '.dll';
  AIXSharedLibExt = '.a';
  ExeExt  = '.exe';
  DbgExt  = '.dbg';
  ZipExt  = '.zip';
  FpmkExt = '.fpm';

  FPMakePPFile = 'fpmake.pp';
  ManifestFile = 'manifest.xml';
  DocProjectFileExt = '-docs.xml';
  PkgListFileBase = 'pkg-';
  PkgListFileExt = '.lst';

  DirNotFound = '<dirnotfound>';

  UnitTargets = [ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit];
  ProgramTargets = [ttProgram,ttExampleProgram,ttSharedLibrary];

  DefaultMessages = [vlError,vlWarning,vlCommand];
  AllMessages = [vlError,vlWarning,vlCommand,vlInfo];

Type
  TTargets = Class;
  { TNamedItem }

  TNamedItem = Class(TCollectionItem)
  private
    FName: String;
    procedure SetName(const AValue: String);virtual;
  Public
    property Name : String Read FName Write SetName;
  end;

  { TNamedCollection }

  TNamedCollection = Class(TCollection)
  private
    FUniqueNames: Boolean;
  private
    function GetItem(Index: Integer): TNamedItem;
    procedure SetItem(Index: Integer; AValue: TNamedItem);
  Public
    Function IndexOfName(const AName : String) : Integer;
    Function ItemByName(const AName : String) : TNamedItem;
    Property UniqueNames : Boolean Read FUniqueNames;
    property Items[Index: Integer]: TNamedItem read GetItem write SetItem;
  end;

  { TNamedItemList }

  TNamedItemList = Class(TFPList)
  private
    function GetNamedItem(Index : Integer): TNamedItem;
    procedure SetNamedItem(Index : Integer; const AValue: TNamedItem);
  public
    Function IndexOfName(const AName : String) : Integer;
    Function ItemByName(const ANAme : String) : TNamedItem;
    Property NamedItems[Index : Integer] : TNamedItem Read GetNamedItem Write SetNamedItem; default;
  end;

  { TCommand }
  TCommand = Class(TNamedItem)
  private
    FAfterCommand: TNotifyEvent;
    FBeforeCommand: TNotifyEvent;
    FCommand: String;
    FCommandAt: TCommandAt;
    FDestFile: String;
    FIgnoreResult: Boolean;
    FOptions: TStrings;
    FSourceFile: String;
    Function GetOptions : TStrings;
    Procedure SetOptions(Const Value : TStrings);
  Public
    Destructor Destroy; override;
    Function HaveOptions : Boolean;
    Function CmdLineOptions : String;
    Procedure ParseOptions(S : String);
    Property SourceFile : String Read FSourceFile Write FSourceFile;
    Property DestFile : String Read FDestFile Write FDestFile;
    Property Command : String Read FCommand Write FCommand;
    Property Options : TStrings Read GetOptions Write SetOptions;
    Property At : TCommandAt Read FCommandAt Write FCommandAt;
    Property IgnoreResult : Boolean Read FIgnoreResult Write FIgnoreResult;
    Property BeforeCommand : TNotifyEvent Read FBeforeCommand Write FBeforeCommand;
    Property AfterCommand : TNotifyEvent Read FAfterCommand Write FAfterCommand;
  end;

  { TCommands }

  { TCommandEnumerator }

  TCommandEnumerator = Class(TCollectionEnumerator)
  public
    function GetCurrent: TCommand;
    property Current: TCommand read GetCurrent;
  end;

  TCommands = Class(TNamedCollection)
  private
    FDefaultAt: TCommandAt;
    function GetCommand(const Dest : String): TCommand;
    function GetCommandItem(Index : Integer): TCommand;
    procedure SetCommandItem(Index : Integer; const AValue: TCommand);
  Public
    Function AddCommand(Const Cmd : String) : TCommand;
    Function AddCommand(Const Cmd,Options : String) : TCommand;
    Function AddCommand(Const Cmd,Options,Dest,Source : String) : TCommand;
    Function AddCommand(At : TCommandAt; Const Cmd : String) : TCommand;
    Function AddCommand(At : TCommandAt; Const Cmd,Options : String) : TCommand;
    Function AddCommand(At : TCommandAt; Const Cmd,Options, Dest,Source : String) : TCommand;
    Function GetEnumerator : TCommandEnumerator;
    Property CommandItems[Index : Integer] : TCommand Read GetCommandItem Write SetCommandItem;
    Property Commands[Dest : String] : TCommand Read GetCommand; default;
    Property DefaultAt : TCommandAt Read FDefaultAt Write FDefaultAt;
  end;

  { TFPVersion }

  TFPVersion = Class(TPersistent)
  private
    FMajor,
    FMinor,
    FMicro,
    FBuild    : Integer;
    function GetAsString: String;
    function GetEmpty: Boolean;
    procedure SetAsString(const AValue: String);
  Public
   Procedure Clear;
   Procedure Assign(Source : TPersistent); override;
   Function CompareVersion(AVersion : TFPVersion) : Integer;
   Function SameVersion(AVersion : TFPVersion) : Boolean;
   Property AsString : String Read GetAsString Write SetAsString;
   Property Empty : Boolean Read GetEmpty;
  Published
   Property Major : Integer Read FMajor Write FMajor;
   Property Minor : Integer Read FMinor Write FMinor;
   Property Micro : Integer Read FMicro Write FMicro;
   Property Build : Integer Read FBuild Write FBuild;
  end;

  { TConditionalString }
  TConditionalString = Class
  private
    FOSes   : TOSes;
    FCPUs   : TCPUs;
    FValue  : String;
  Public
    Constructor Create;virtual;
    Property Value : String Read FValue Write FValue;
    Property OSes  : TOSes Read FOSes Write FOSes;
    Property CPUs : TCPUs Read FCPUS Write FCPUs;
  end;

  TConditionalStringClass = class of TConditionalString;

  { TConditionalStrings }

  TConditionalStrings = Class(TFPList)
  private
    FCSClass : TConditionalStringClass;
    function GetConditionalString(Index : Integer): TConditionalString;
    procedure SetConditionalString(Index : Integer; const AValue: TConditionalString);
  Public
    Constructor Create(AClass:TConditionalStringClass);
    Function Add(Const Value : String) : TConditionalString;inline;
    Function Add(Const Value : String;const OSes:TOSes) : TConditionalString;inline;
{$ifdef cpu_only_overloads}
    Function Add(Const Value : String;const CPUs:TCPUs) : TConditionalString;inline;
{$endif cpu_only_overloads}
    Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TConditionalString;
    Property ConditionalStrings[Index : Integer] : TConditionalString Read GetConditionalString Write SetConditionalString; default;
  end;

  { TConditionalDestString }

  TConditionalDestString = Class(TConditionalString)
  private
    FDestPath: string;
  public
    property DestPath: string read FDestPath write FDestPath;
  end;

  TConditionalDestStringClass = class of TConditionalDestString;

  { TConditionalDestStrings }

  TConditionalDestStrings = Class(TConditionalStrings)
  private
    function GetConditionalString(Index : Integer): TConditionalDestString;
    procedure SetConditionalString(Index : Integer; AValue: TConditionalDestString);
  public
    Constructor Create(AClass:TConditionalDestStringClass);
    Function Add(Const Value : String; ADestPath: String) : TConditionalDestString;inline;
    Function Add(Const Value : String;const OSes:TOSes; ADestPath: String) : TConditionalDestString;inline;
{$ifdef cpu_only_overloads}
    Function Add(Const Value : String;const CPUs:TCPUs; ADestPath: String) : TConditionalDestString;inline;
{$endif cpu_only_overloads}
    Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TOSes; ADestPath: String) : TConditionalDestString;
    Property ConditionalStrings[Index : Integer] : TConditionalDestString Read GetConditionalString Write SetConditionalString; default;
  end;

  { TNotifyEventCollection }

  TNotifyEventAction = (neaBeforeCompile, neaAfterCompile, neaBeforeInstall, neaAfterInstall,
                        neaBeforeClean, neaAfterClean, neaBeforeArchive, neaAfterArchive,
                        neaBeforeManifest, neaAfterManifest, neaBeforePkgList, neaAfterPkgList,
                        neaBeforeUnInstall, neaAfterUnInstall,
                        neaBeforeCreateBuildEngine, neaAfterCreateBuildengine,
                        neaBeforeDocProject, neaAfterDocProject);

  TNotifyEventActionSet = set of TNotifyEventAction;

  TNotifyEventItem = class(TCollectionItem)
  private
    FOnAction: TNotifyEventAction;
    FOnEvent: TNotifyEvent;
    FOnProcEvent: TNotifyProcEvent;
  public
    property OnAction: TNotifyEventAction read FOnAction write FOnAction;
    property OnEvent: TNotifyEvent read FOnEvent write FOnEvent;
    property OnProcEvent: TNotifyProcEvent read FOnProcEvent write FOnProcEvent;
    procedure CallEvent(Sender: TObject);
  end;

  { TNotifyEventEnumerator }

  TNotifyEventEnumerator = Class(TCollectionEnumerator)
  public
    function GetCurrent: TNotifyEventItem;
    property Current: TNotifyEventItem read GetCurrent;
  end;


  TNotifyEventCollection = class(TCollection)
  private
    FSupportedActionSet: TNotifyEventActionSet;
  public
    constructor create(ASupportedActionSet: TNotifyEventActionSet);
    Function GetEnumerator : TNotifyEventEnumerator;
    procedure AppendEvent(AnAction: TNotifyEventAction; AnEvent: TNotifyEvent);
    procedure AppendProcEvent(AnACtion: TNotifyEventAction; AnProcEvent: TNotifyProcEvent);
    procedure CallEvents(AnAction: TNotifyEventAction; Sender: TObject);
  end;

  { TDictionary }

  TReplaceFunction = Function (Const AName,Args : String) : String of Object;

  TDictionary = Class(TComponent)
  private
    FList : TStringList;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy;override;
    Procedure AddVariable(const AName,Value : String);
    Procedure AddFunction(const AName : String; FReplacement : TReplaceFunction);
    Procedure RemoveItem(const AName : String);
    Function GetValue(AName : String) : String;
    Function GetValue(const AName,Args : String) : String; virtual;
    Function ReplaceStrings(Const ASource : String; Const MaxDepth: Integer = 10) : String; virtual;
    Function Substitute(Const Source : String; const Macros : Array of string) : String; virtual;
  end;

  { TPackageDictionary }

  TPackageDictionary = Class(TDictionary)
  private
    FMasterDictionary: TDictionary;
  Public
    Function GetValue(const AName,Args : String) : String; override;
    property MasterDictionary: TDictionary read FMasterDictionary write FMasterDictionary;
  end;


  { TDependency }
  TDependency = Class(TConditionalString)
  private
    FDependencyType : TDependencyType;
    // Package, Unit
    FTarget : TObject;
    FVersion : TFPVersion;
    FRequireChecksum : Cardinal;
    // Filenames, Includes
    FTargetFileName : String;
    Function GetVersion : string;
    Procedure SetVersion(const V : string);
  Public
    Constructor Create;override;
    Destructor Destroy;override;
    Property Target : TObject Read FTarget Write FTarget;
    Property DependencyType : TDependencyType Read FDependencyType;
    Property TargetFileName : String Read FTargetFileName Write FTargetFileName;
    Property Version : String Read GetVersion Write SetVersion;
    Property RequireChecksum : Cardinal Read FRequireChecksum Write FRequireChecksum;
  end;

  TResourceFile = Class(TConditionalString);

  { TPackageVariant }

  TPackage = Class;
  TPackageVariant = class(TNamedItem)
  private
    FOptions: TStrings;
    FTargets: TTargets;
    FIncludePath: TConditionalStrings;
    FSourcePath: TConditionalStrings;
  public
    constructor Create(ACollection: TCollection); override;
    destructor Destroy; override;
    property Options: TStrings read FOptions;
    Property IncludePath : TConditionalStrings Read FIncludePath;
    Property SourcePath : TConditionalStrings Read FSourcePath;
    property Targets: TTargets read FTargets;
  end;


  { TPackageVariantEnumerator }

  TPackageVariantEnumerator = Class(TCollectionEnumerator)
  public
    function GetCurrent: TPackageVariant;
    property Current: TPackageVariant read GetCurrent;
  end;

  { TPackageVariants }

  TPackageVariants = class(TNamedCollection)
  private
    FActivePackageVariantName: string;
    FAutoAddToPackage: boolean;
    FDefaultPackageVariantName: string;
    FIsInheritable: boolean;
    FMasterPackage: TPackage;
    FName: string;
    function GetActivePackageVariant: TPackageVariant;
    function GetDefaultPackageVariant: TPackageVariant;
    procedure SetActivePackageVariantName(AValue: string);
    procedure SetDefaultPackageVariantName(AValue: string);
  public
    function Add(AName: String): TPackageVariant; overload; virtual;
    Function GetEnumerator :  TPackageVariantEnumerator;
    property Name: string read FName write FName;
    property MasterPackage: TPackage read FMasterPackage;
    property DefaultPackageVariant: TPackageVariant read GetDefaultPackageVariant;
    property ActivePackageVariant: TPackageVariant read GetActivePackageVariant;
    property DefaultPackageVariantName: string read FDefaultPackageVariantName write SetDefaultPackageVariantName;
    property ActivePackageVariantName: string read FActivePackageVariantName write SetActivePackageVariantName;
    property IsInheritable: boolean read FIsInheritable;
    property AutoAddToPackage: boolean read FAutoAddToPackage;
  end;


  TDependencies = Class(TConditionalStrings)
    function GetDependency(Index : Integer): TDependency;
    procedure SetDependency(Index : Integer; const AValue: TDependency);
  Public
    Function Add(Const Value : String) : TDependency;inline;
    Function Add(Const Value : String;const OSes:TOSes) : TDependency;inline;
{$ifdef cpu_only_overloads}
    Function Add(Const Value : String;const CPUs:TCPUs) : TDependency;inline;
{$endif cpu_only_overloads}
    Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TDependency;
    Function AddUnit(Const Value : String) : TDependency;inline;
    Function AddUnit(Const Value : String;const OSes:TOSes) : TDependency;inline;
{$ifdef cpu_only_overloads}
    Function AddUnit(Const Value : String;const CPUs:TCPUs) : TDependency;inline;
{$endif cpu_only_overloads}
    Function AddUnit(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TDependency;
    Function AddInclude(Const Value : String) : TDependency;inline;
    Function AddInclude(Const Value : String;const OSes:TOSes) : TDependency;inline;
{$ifdef cpu_only_overloads}
    Function AddInclude(Const Value : String;const CPUs:TCPUs) : TDependency;inline;
{$endif cpu_only_overloads}
    Function AddInclude(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TDependency;
    Property Dependencies[Index : Integer] : TDependency Read GetDependency Write SetDependency; default;
  end;

  { TResourceFiles }

  TResourceFiles = Class(TConditionalStrings)
  public
    Procedure GetInstallFiles(AList : TStrings; const APrefixU, APrefixB : String; ACPU:TCPU; AOS : TOS); virtual;
  end;

  { TTarget }

  TTarget = Class(TNamedItem)
  private
    FInstall : Boolean;
    FAfterClean: TNotifyEvent;
    FAfterCompile: TNotifyEvent;
    FBeforeClean: TNotifyEvent;
    FBeforeCompile: TNotifyEvent;
    FCPUs: TCPUs;
    FIsFPMakePlugin: Boolean;
    FOSes: TOSes;
    FMode: TCompilerMode;
    FResourceStrings: Boolean;
    FObjectPath,
    FUnitPath,
    FIncludePath : TConditionalStrings;
    FDependencies : TDependencies;
    FResourceFiles : TResourceFiles;
    FCommands : TCommands;
    FDirectory: String;
    FExtension: String;
    FExeName : String;
    FTargetSourceFileName : String;
    FFileType: TFileType;
    FOptions: TStrings;
    FFPCTarget: String;
    FTargetState: TTargetState;
    FTargetType: TTargetType;
    FXML: string;
    function GetOptions: TStrings;
    procedure SetOptions(const AValue: TStrings);
  Protected
    Function GetSourceFileName : String; virtual;
    Function GetUnitFileName : String; virtual;
    function GetUnitLibFileName(AOS: TOS): String; virtual;
    Function GetObjectFileName : String; virtual;
    Function GetLTOFileName : String; virtual;
    Function GetBinFileBase: String;
    function GetRSTFileName : String; Virtual;
    function GetRSJFileName : String; Virtual;
    function GetImportLibFileName(AOS : TOS) : String; Virtual;
    Function GetProgramFileName(AOS : TOS) : String; Virtual;
    Function GetProgramDebugFileName(AOS : TOS) : String; Virtual;
    Function GetLibraryFileName(AOS : TOS) : String; Virtual;
    Function GetLibraryDebugFileName(AOS : TOS) : String; Virtual;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    procedure AssignTo(Dest: TPersistent); override;
    Function  GetOutputFileName (AOs : TOS) : String; Virtual;
    Function HaveOptions : Boolean;
    procedure SetName(const AValue: String);override;
    procedure SetExeName(const AValue: String);
    procedure SetXML(const AValue: string);
    Procedure GetCleanFiles(List : TStrings; const APrefixU, APrefixB : String; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetInstallFiles(List : TStrings; const APrefixU, APrefixB : String; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    Property Dependencies : TDependencies Read FDependencies;
    Property ResourceFiles: TResourceFiles read FResourceFiles;
    Property Commands : TCommands Read FCommands;
    Property State : TTargetState Read FTargetState;
    Property TargetType : TTargetType Read FTargetType Write FTargetType;
    Property OSes : TOSes Read FOSes Write FOSes;
    Property CPUs : TCPUs Read FCPUs Write FCPUs;
    Property Mode : TCompilerMode Read FMode Write FMode;
    Property Options : TStrings Read GetOptions Write SetOptions;
    Property SourceFileName: String Read GetSourceFileName ;
    Property UnitFileName : String Read GetUnitFileName;
    Property ObjectFileName : String Read GetObjectFileName;
    Property LTOFileName : String Read GetLTOFileName;
    Property RSTFileName : String Read GetRSTFileName;
    Property RSJFileName : String Read GetRSJFileName;
    Property FPCTarget : String Read FFPCTarget Write FFPCTarget;
    Property Extension : String Read FExtension Write FExtension;
    Property FileType : TFileType Read FFileType Write FFileType;
    Property Directory : String Read FDirectory Write FDirectory;
    Property ResourceStrings : Boolean Read FResourceStrings Write FResourceStrings;
    Property Install : Boolean Read FInstall Write FInstall;
    Property TargetSourceFileName: String Read FTargetSourceFileName;
    Property ObjectPath : TConditionalStrings Read FObjectPath;
    Property UnitPath : TConditionalStrings Read FUnitPath;
    Property IncludePath : TConditionalStrings Read FIncludePath;
    Property XML: string Read FXML Write SetXML;
    Property IsFPMakePlugin : Boolean read FIsFPMakePlugin write FIsFPMakePlugin;
    // Events.
    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FBeforeCompile;
    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAfterCompile;
    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBeforeClean;
    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterClean;
  end;


  { TTargetEnumerator }

  TTargetEnumerator = Class(TCollectionEnumerator)
  public
    function GetCurrent: TTarget;
    property Current: TTarget read GetCurrent;
  end;

  { TTargets }

  TTargets = Class(TNamedCollection)
  private
    function GetTargetItem(Index : Integer): TTarget;
    function GetTarget(const AName : String): TTarget;
    procedure SetTargetItem(Index : Integer; const AValue: TTarget);
  Public
    Function AddFPDoc(Const AUnitName, AXMLName : String) : TTarget;inline;
    Function AddUnit(Const AUnitName : String) : TTarget;inline;
    Function AddUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;inline;
{$ifdef cpu_only_overloads}
    Function AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;inline;
{$endif cpu_only_overloads}
    Function AddUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
    Function AddImplicitUnit(Const AUnitName : String;InstallUnit:boolean=true) : TTarget;inline;
    Function AddImplicitUnit(Const AUnitName : String;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;inline;
    Function AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;InstallUnit:boolean=true) : TTarget;inline;
    Function AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
    Function AddProgram(Const AProgramName : String) : TTarget;inline;
    Function AddProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;inline;
{$ifdef cpu_only_overloads}
    Function AddProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;inline;
{$endif cpu_only_overloads}
    Function AddProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
    Function AddLibrary(Const ALibraryName : String) : TTarget;inline;
    Function AddLibrary(Const ALibraryName : String;const OSes:TOSes) : TTarget;inline;
{$ifdef cpu_only_overloads}
    Function AddLibrary(Const ALibraryName : String;const CPUs:TCPUs) : TTarget;inline;
{$endif cpu_only_overloads}
    Function AddLibrary(Const ALibraryName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
    Function AddExampleUnit(Const AUnitName : String) : TTarget;inline;
    Function AddExampleUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;inline;
{$ifdef cpu_only_overloads}
    Function AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;inline;
{$endif cpu_only_overloads}
    Function AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
    Function AddExampleProgram(Const AProgramName : String) : TTarget;inline;
    Function AddExampleProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;inline;
{$ifdef cpu_only_overloads}
    Function AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;inline;
{$endif cpu_only_overloads}
    Function AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
    Function GetEnumerator : TTargetEnumerator;
    Property Targets[AName : String] : TTarget Read GetTarget; default;
    Property TargetItems[Index : Integer] : TTarget Read GetTargetItem Write SetTargetItem;
  end;

  { TSource }

  TSource = Class(TNamedItem)
  private
    FSourceType : TSourceType;
    FInstallSourcePath : string;
    function GetInstallSourcePath: string;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    Procedure GetInstallFiles(List : TStrings); virtual;
    property SourceType : TSourceType read FSourceType;
    property InstallSourcePath : string read GetInstallSourcePath;
  end;

  { TSources }

  { TSourceEnumerator }

  TSourceEnumerator = Class(TCollectionEnumerator)
  public
    function GetCurrent: TSource;
    property Current: TSource read GetCurrent;
  end;

  TSources = Class(TNamedCollection)
  private
    function GetSourceItem(Index : Integer): TSource;
    procedure SetSourceItem(Index : Integer; const AValue: TSource);
  public
    Function AddDoc(const AFiles : String) : TSource;
    Function AddDoc(const AFiles : String; AInstallSourcePath : String) : TSource;
    Function AddSrc(const AFiles : String) : TSource;
    Function AddExample(const AFiles : String) : TSource;
    Function AddExample(const AFiles : String; AInstallSourcePath : String) : TSource;
    Function AddTest(const AFiles : String) : TSource;
    Function GetEnumerator : TSourceEnumerator;
    procedure AddDocFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False; AInstallSourcePath : String = '');
    procedure AddSrcFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False);
    procedure AddExampleFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False; AInstallSourcePath : String = '');
    procedure AddTestFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False);
    Property SourceItems[Index : Integer] : TSource Read GetSourceItem Write SetSourceItem;default;
  end;

  { TPackage }

  TPackage = Class(TNamedItem)
  private
    FAfterArchive: TNotifyEvent;
    FAfterArchiveProc: TNotifyProcEvent;
    FAfterClean: TNotifyEvent;
    FAfterCleanProc: TNotifyProcEvent;
    FAfterCompile: TNotifyEvent;
    FAfterCompileProc: TNotifyProcEvent;
    FAfterInstall: TNotifyEvent;
    FAfterInstallProc: TNotifyProcEvent;
    FAfterManifest: TNotifyEvent;
    FAfterManifestProc: TNotifyProcEvent;
    FAfterPkgList: TNotifyEvent;
    FAfterPkgListProc: TNotifyProcEvent;
    FBeforeArchive: TNotifyEvent;
    FBeforeArchiveProc: TNotifyProcEvent;
    FBeforeClean: TNotifyEvent;
    FBeforeCleanProc: TNotifyProcEvent;
    FBeforeCompile: TNotifyEvent;
    FBeforeCompileProc: TNotifyProcEvent;
    FBeforeInstall: TNotifyEvent;
    FBeforeInstallProc: TNotifyProcEvent;
    FBeforeManifest: TNotifyEvent;
    FBeforeManifestProc: TNotifyProcEvent;
    FBeforePkgList: TNotifyEvent;
    FBeforePkgListProc: TNotifyProcEvent;
    FBuildMode: TBuildMode;
    FFlags: TStrings;
    FFPDocFormat: TFPDocFormats;
    FIsFPMakeAddIn: boolean;
    FSupportBuildModes: TBuildModes;
    FUnitPath,
    FObjectPath,
    FIncludePath,
    FSourcePath,
    FExamplePath,
    FTestPath,
    FCleanFiles   : TConditionalStrings;
    FInstallFiles : TConditionalDestStrings;
    FDependencies : TDependencies;
    FCPUs: TCPUs;
    FOSes: TOSes;
    FTargetState: TTargetState;
    FTargets: TTargets;
    FSources: TSources;
    FDirectory: String;
    FOptions: TStrings;
    FTransmitOptions: TStrings;
    FFileName: String;
    FShortName: String; { Used to generate the short 8.3 zip file name, must be 4 characters at most }
    FAuthor: String;
    FLicense: String;
    FHomepageURL: String;
    FDownloadURL: String;
    FVersion: TFPVersion;
    FEmail : String;
    FNeedLibC : Boolean;
    FCommands : TCommands;
    FDescriptionFile : String;
    FDescription : String;
    FInstalledChecksum : Cardinal;
    FUnitsOutputDir: String;
    FPackageUnitInstallDir: String;
    // Cached directory of installed packages
    FUnitDir : String;
    FUnitConfigFileName : String;
    // Used by buildunits
    FBUTargets: TTargets;
    FBUTarget: TTarget;
    // Used to identify if package is being processed by a thread
    FProcessing : boolean;
    // Dictionary
    FDictionary : TDictionary;
    // Is set when all sourcefiles are found
    FAllFilesResolved: boolean;
    FPackageVariants: TFPList;
{$ifndef NO_THREADING}
    FResolveDirsCS: TRTLCriticalSection;
{$endif}
    Function GetDescription : string;
    function GetDictionary: TDictionary;
    Function GetFileName : string;
    Function GetShortName : string;
    function GetOptions: TStrings;
    function GetTransmitOptions: TStrings;
    Function GetVersion : string;
    procedure SetOptions(const AValue: TStrings);
    procedure SetTransmitOptions(AValue: TStrings);
    Procedure SetVersion(const V : string);
  Protected
    procedure SetName(const AValue: String);override;
    procedure SaveUnitConfigToStringList(Const AStringList: TStrings;ACPU:TCPU;AOS:TOS); virtual;
    property Dictionary: TDictionary read GetDictionary;
    property PackageVariantsList: TFPList read FPackageVariants;
  Public
    constructor Create(ACollection: TCollection); override;
    destructor destroy; override;
    Function HaveOptions : Boolean;
    Function  GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):String;
    Function  GetUnitConfigOutputFilename(ACPU:TCPU; AOS : TOS):String;
    Procedure InheritPackageVariantsFromDependency(ADependencyPackage: TPackage);
    Function  GetPackageVariantsByName(AName: string): TPackageVariants;
    Procedure SetUnitsOutputDir(AValue: string);
    Function  GetPackageUnitInstallDir(ACPU:TCPU; AOS : TOS):String;
    Procedure SetPackageUnitInstallDir(AValue: string);
    Function  GetBinOutputDir(ACPU:TCPU; AOS : TOS) : String;
    Procedure GetCleanFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    procedure GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU:TCPU; AOS : TOS); virtual;
    procedure GetInstallSourceFiles(List: TStrings; SourceTypes : TSourceTypes; TargetTypes : TTargetTypes); virtual;
    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetArchiveSourceFiles(List : TStrings); virtual;
    Procedure GetManifest(Manifest : TStrings);
    Procedure ListPackage(PkgList : TStrings);
    Procedure AddPackageVariant(APackageVariant: TPackageVariants);
    procedure ApplyPackageVariantToCompilerOptions(ACompilerOptions: tstrings);
    procedure SetDefaultPackageVariant;
    procedure LoadUnitConfigFromFile(Const AFileName: String);
    procedure SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;AOS:TOS);
    procedure EnterResolveDirsCS;
    procedure LeaveResolveDirsCS;
    Property Version : String Read GetVersion Write SetVersion;
    Property FileName : String Read GetFileName Write FFileName;
    Property ShortName : String Read GetShortName Write FShortName;
    Property HomepageURL : String Read FHomepageURL Write FHomepageURL;
    Property DownloadURL : String Read FDownloadURL Write FDownloadURL;
    Property Email : String Read FEmail Write FEmail;
    Property Author : String Read FAuthor Write FAuthor;
    Property License : String Read FLicense Write FLicense;
    Property Directory : String Read FDirectory Write FDirectory;
    Property Description : String Read GetDescription Write FDescription;
    Property DescriptionFile : String Read FDescriptionFile Write FDescriptionFile;
    Property InstalledChecksum : Cardinal Read FInstalledChecksum Write FInstalledChecksum;
    Property IsFPMakeAddIn: boolean read FIsFPMakeAddIn write FIsFPMakeAddIn;
    Property SupportBuildModes: TBuildModes read FSupportBuildModes write FSupportBuildModes;
    Property BuildMode: TBuildMode read FBuildMode;
    Property Flags: TStrings read FFlags;
    Property PackageVersion: TFPVersion read FVersion;
    // Options which are passed to the compiler for packages which depend on
    // this package.
    Property TransmitOptions: TStrings Read GetTransmitOptions Write SetTransmitOptions;
    // Compiler options.
    Property OSes : TOSes Read FOSes Write FOSes;
    Property CPUs : TCPUs Read FCPUs Write FCPUs;
    Property NeedLibC : Boolean Read FNeedLibC Write FNeedLibC;
    Property Options: TStrings Read GetOptions Write SetOptions;
    Property UnitPath : TConditionalStrings Read FUnitPath;
    Property ObjectPath : TConditionalStrings Read FObjectPath;
    Property IncludePath : TConditionalStrings Read FIncludePath;
    Property SourcePath : TConditionalStrings Read FSourcePath;
    Property ExamplePath : TConditionalStrings Read FExamplePath;
    Property TestPath : TConditionalStrings Read FTestPath;
    Property FPDocFormat: TFPDocFormats read FFPDocFormat write FFPDocFormat;
    // Targets and dependencies
    Property InstallFiles : TConditionalDestStrings Read FInstallFiles;
    Property CleanFiles : TConditionalStrings Read FCleanFiles;
    Property Dependencies : TDependencies Read FDependencies;
    Property Commands : TCommands Read FCommands;
    Property State : TTargetState Read FTargetState Write FTargetState;
    Property Targets : TTargets Read FTargets;
    Property Sources : TSources Read FSources;
    Property UnitDir : String Read FUnitDir Write FUnitDir;
    Property UnitConfigFileName: String read FUnitConfigFileName write FUnitConfigFileName;
    // events
    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FBeforeCompile;
    Property BeforeCompileProc : TNotifyProcEvent Read FBeforeCompileProc write FBeforeCompileProc;
    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAfterCompile;
    Property AfterCompileProc : TNotifyProcEvent Read FAfterCompileProc Write FAfterCompileProc;
    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FBeforeInstall;
    Property BeforeInstallProc : TNotifyProcEvent Read FBeforeInstallProc Write FBeforeInstallProc;
    Property AfterInstall : TNotifyEvent Read FAfterInstall Write FAfterInstall;
    Property AfterInstallProc : TNotifyProcEvent Read FAfterInstallProc Write FAfterInstallProc;
    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBeforeClean;
    Property BeforeCleanProc : TNotifyProcEvent Read FBeforeCleanProc Write FBeforeCleanProc;
    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterClean;
    Property AfterCleanProc : TNotifyProcEvent Read FAfterCleanProc Write FAfterCleanProc;
    Property BeforeArchive : TNotifyEvent Read FBeforeArchive Write FBeforeArchive;
    Property BeforeArchiveProc : TNotifyProcEvent Read FBeforeArchiveProc Write FBeforeArchiveProc;
    Property AfterArchive : TNotifyEvent Read FAfterArchive Write FAfterArchive;
    Property AfterArchiveProc : TNotifyProcEvent Read FAfterArchiveProc Write FAfterArchiveProc;
    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write FBeforeManifest;
    Property BeforeManifestProc : TNotifyProcEvent Read FBeforeManifestProc Write FBeforeManifestProc;
    Property AfterManifest : TNotifyEvent Read FAfterManifest Write FAfterManifest;
    Property AfterManifestProc : TNotifyProcEvent Read FAfterManifestProc Write FAfterManifestProc;
    Property BeforePkgList : TNotifyEvent Read FBeforePkgList Write FBeforePkgList;
    Property BeforePkgListProc : TNotifyProcEvent Read FBeforePkgListProc Write FBeforePkgListProc;
    Property AfterPkgList : TNotifyEvent Read FAfterPkgList Write FAfterPkgList;
    Property AfterPkgListProc : TNotifyProcEvent Read FAfterPkgListProc Write FAfterPkgListProc;
  end;

  { TPackageEnumerator }

  TPackageEnumerator = Class(TCollectionEnumerator)
  public
    function GetCurrent: TPackage;
    property Current: TPackage read GetCurrent;
  end;

  { TPackages }

  TPackages = Class(TNamedCollection)
  private
    function GetPackage(const AName : String): TPackage;
    function GetPackageItem(AIndex : Integer): TPackage;
    procedure SetPackageItem(AIndex : Integer; const AValue: TPackage);
  Public
    Function AddPackage(Const AName : String) : TPackage;
    Function GetEnumerator : TPackageEnumerator;
    Property Packages[AName : String] : TPackage Read GetPackage ; Default;
    Property PackageItems[AIndex : Integer] : TPackage Read GetPackageItem Write SetPackageItem;
  end;

  { TCustomDefaults }

  TCustomDefaults = Class(TPersistent)
  Private
    FArchive: String;
    FBuildMode: TBuildMode;
    FCompiler: String;
    FCopy: String;
    FFPDocOptions: String;
    FFPDocOutputDir: String;
    FFPUnitSourcePath: String;
    FIgnoreInvalidOptions: Boolean;
    FInstallExamples: Boolean;
    FMkDir: String;
    FMove: String;
    FOptions: TStrings;
    FCPU: TCPU;
    FOS: TOS;
    FMode : TCompilerMode;
    FCompilerVersion : String;
    FPrefix: String;
    FBaseInstallDir,
    FUnitInstallDir,
    FUnitConfigFilesInstallDir,
    FBinInstallDir,
    FLibInstallDir,
    FDocInstallDir,
    FExamplesInstallDir : String;
    FSingleFPDocFile: Boolean;
    FSingleDocProjectFile: Boolean;
    FSearchPath: TStrings;
    FSkipCrossPrograms: boolean;
    FThreadsAmount: integer;
    FRemoveTree: String;
    FRemoveDir: String;
    FRemove: String;
    FTarget: String;
    FUnixPaths: Boolean;
    FNoFPCCfg: Boolean;
    FUseEnvironment: Boolean;
    FZipPrefix: String;
    FExplicitOSNone: Boolean;
    function SafeExpandFileName(const AFileName: string): string;
    function GetBuildCPU: TCpu;
    function GetBuildOS: TOS;
    function GetBuildString: String;
    function GetFPDocOutputDir: String;
    function GetFPUnitSourcePath: String;
    function GetLocalUnitDir: String;
    function GetGlobalUnitDir: String;
    function GetBaseInstallDir: String;
    function GetBinInstallDir: String;
    function GetLibInstallDir: String;
    function GetCompiler: String;
    function GetDocInstallDir: String;
    function GetExamplesInstallDir: String;
    function GetOptions: TStrings;
    function GetPrefix: String;
    function GetSearchPath: TStrings;
    function GetUnitInstallDir: String;
    function GetUnitConfigFilesInstallDir: String;
    procedure SetLocalUnitDir(const AValue: String);
    procedure SetGlobalUnitDir(const AValue: String);
    procedure IntSetBaseInstallDir(const AValue: String);
    procedure SetBaseInstallDir(const AValue: String);
    procedure SetCPU(const AValue: TCPU);
    procedure SetOptions(const AValue: TStrings);
    procedure SetOS(const AValue: TOS);
    procedure SetPrefix(const AValue: String);
    procedure SetSearchPath(AValue: TStrings);
    procedure SetTarget(const AValue: String);
    procedure SetUnitInstallDir(const AValue: String);
    procedure SetUnitConfigFilesInstallDir(const AValue: String);
    procedure SetZipPrefix(AValue: String);
  Protected
    procedure RecalcTarget;
    Function CmdLineOptions : String;
  Public
    Constructor Create;
    Destructor Destroy; override;
    Procedure InitDefaults;
    Function HaveOptions: Boolean;
    function IsBuildDifferentFromTarget: boolean;
    procedure CompilerDefaults; virtual;
    Procedure LocalInit(Const AFileName : String);
    Procedure LoadFromFile(Const AFileName : String);
    Procedure SaveToFile(Const AFileName : String);
    procedure SaveToStream(S : TStream);virtual;
    procedure LoadFromStream(S : TStream);virtual;
    // Compile Information
    Property Target : String Read FTarget Write SetTarget;
    Property OS : TOS Read FOS Write SetOS;
    Property CPU : TCPU Read FCPU Write SetCPU;
    Property CompilerVersion : String read FCompilerVersion;
    Property ExplicitOSNone: Boolean read FExplicitOSNone Write FExplicitOSNone;
    Property BuildString : String read GetBuildString;
    Property BuildOS : TOS read GetBuildOS;
    Property BuildCPU : TCpu read GetBuildCPU;
    Property Mode : TCompilerMode Read FMode Write FMode;
    Property UnixPaths : Boolean Read FUnixPaths Write FUnixPaths;
    Property Options : TStrings Read GetOptions Write SetOptions;    // Default compiler options.
    Property NoFPCCfg : Boolean Read FNoFPCCfg Write FNoFPCCfg;
    // When ThreadsAmount is specified, #threadsamount# worker-threads are
    // created. When such a worker-thread is ready all worker-threads are evaluated
    // to see if there are idle threads (there is always at least one such thread.)
    // To each idle thread a package is assigned which has to be compiled for the
    // current target and for which all dependencies are compiled earlier.
    // When no package is available the thread remains idle until another thread
    // has finished it's task. Compilation stops when all packages are compiled
    // or when an error occures.
    //
    // When ThreadsAmount is not specified (-1), all packages are compiled on by one.
    // Dependencies are compiled recursively. When a package is already compiled
    // (because some other package was depending on it) the package is skipped.
    // When the last package in the list is compiled, the compilation stops.
    Property ThreadsAmount : integer Read FThreadsAmount Write FThreadsAmount;
    // paths etc.
    Property LocalUnitDir : String Read GetLocalUnitDir Write SetLocalUnitDir;
    Property GlobalUnitDir : String Read GetGlobalUnitDir Write SetGlobalUnitDir;
    // The SearchPath contains a list of directories in which packages are
    // installed. Packages are searched for in order of this list.
    Property SearchPath: TStrings read GetSearchPath write SetSearchPath;
    Property Prefix : String Read GetPrefix Write SetPrefix;
    Property ZipPrefix : String Read FZipPrefix Write SetZipPrefix;
    Property BaseInstallDir : String Read GetBaseInstallDir Write SetBaseInstallDir;
    Property UnitInstallDir : String Read GetUnitInstallDir Write SetUnitInstallDir;
    Property UnitConfigFilesInstallDir : String Read GetUnitConfigFilesInstallDir Write SetUnitConfigFilesInstallDir;
    Property BinInstallDir : String Read GetBinInstallDir Write FBinInstallDir;
    Property LibInstallDir : String Read GetLibInstallDir Write FLibInstallDir;
    Property DocInstallDir : String Read GetDocInstallDir Write FDocInstallDir;
    Property ExamplesInstallDir : String Read GetExamplesInstallDir Write FExamplesInstallDir;
    Property FPDocOutputDir : String Read GetFPDocOutputDir Write FFPDocOutputDir;
    Property FPDocOptions : String Read FFPDocOptions Write FFPDocOptions;
    Property FPUnitSourcePath: String read GetFPUnitSourcePath Write FFPUnitSourcePath;

    // Command tools. If not set, internal commands  will be used.
    Property Compiler : String Read GetCompiler Write FCompiler; // Compiler. Defaults to fpc
    Property Copy : String Read FCopy Write FCopy;             // copy $(FILES) to $(DEST)
    Property Move : String Read FMove Write FMove;             // Move $(FILES) to $(DEST)
    Property Remove : String Read FRemove Write FRemove;       // Delete $(FILES)
    Property RemoveDir : String Read FRemoveDir Write FRemoveDir;       // Delete $(FILES)
    Property RemoveTree : String Read FRemoveTree Write FRemoveTree;       // removes $(DIRECTORY)
    Property MkDir : String Read FMkDir write FMkDir;          // Make $(DIRECTORY)
    Property Archive : String Read FArchive Write FArchive;    // zip $(ARCHIVE) $(FILESORDIRS)
    // Misc
    Property UseEnvironment : Boolean read FUseEnvironment write FUseEnvironment;
    Property IgnoreInvalidOptions: Boolean read FIgnoreInvalidOptions write FIgnoreInvalidOptions;
    Property BuildMode: TBuildMode read FBuildMode write FBuildMode;
    Property SingleFPDocFile : Boolean Read FSingleFPDocFile Write FSingleFPDocFile;
    // Installation optioms
    Property InstallExamples: Boolean read FInstallExamples write FInstallExamples;
    Property SkipCrossPrograms: boolean read FSkipCrossPrograms write FSkipCrossPrograms;
  end;

  { TBasicDefaults }

  TBasicDefaults = Class(TCustomDefaults)
  end;

  { TFPCDefaults }

  TFPCDefaults = Class(TCustomDefaults)
  public
    procedure CompilerDefaults; override;
  end;

  { TBuildEngine }
  TCopyFileProc = procedure(const APackage: TPackage; Const ASourceFileName, ADestFileName : String) of object;

  TBuildEngine = Class(TComponent)
  private
    // general variables
    FCompiler : String;
    FStartDir : String;
    FForceCompile : Boolean;
    FListMode : Boolean;
    FVerbose : boolean;
    FInteractive : boolean;
    FProgressMax : integer;
    FProgressCount : integer;
    FExternalPackages : TPackages;
    // Events
    FOnLog: TLogEvent;
    FNotifyEventCollection: TNotifyEventCollection;
    FOnCopyFile: TCopyFileProc;
    FOnFinishCopy: TNotifyEvent;

    FCachedlibcPath: string;
    FGeneralCriticalSection: TRTLCriticalSection;
{$ifdef HAS_UNIT_ZIPPER}
    FZipper: TZipper;
    FGZFileStream: TGZFileStream;
{$endif HAS_UNIT_ZIPPER}
{$ifdef HAS_TAR_SUPPORT}
    FTarWriter: TTarWriter;
{$endif HAS_TAR_SUPPORT}
    procedure AddFileToArchive(const APackage: TPackage; Const ASourceFileName, ADestFileName : String);
    procedure FinishArchive(Sender: TObject);
  Protected
    Procedure Error(const Msg : String);
    Procedure Error(const Fmt : String; const Args : Array of const);
    // Internal copy/delete/move/archive/mkdir files
    Function  SysDirectoryExists(const ADir:string):Boolean;
    Function  SysFileExists(const AFileName:string):Boolean;
    Procedure SysCopyFile(Const Src,Dest : String); virtual;
    Procedure SysMoveFile(Const Src,Dest : String); virtual;
    Procedure SysDeleteFile(Const AFileName : String); virtual;
    Procedure SysDeleteDirectory(Const ADirectoryName : String); virtual;
    Procedure SysDeleteTree(Const ADirectoryName : String); virtual;
    Procedure SysArchiveFiles(List : TStrings; Const AFileName : String); virtual;
    procedure LogIndent;
    procedure LogUnIndent;
    Procedure EnterDir(ADir : String);
    Function GetCompiler : String;
    Function InstallPackageFiles(APAckage : TPackage; tt : TTargetTypes; Const Dest : String; Const InstallMode: TInstallMode):Boolean;
    Procedure InstallUnitConfigFile(APAckage : TPackage; Const Dest : String);
    function GetUnitConfigFilesInstallDir(ABaseDir: string): String;

    Function InstallPackageSourceFiles(APAckage : TPackage; stt : TSourceTypes; ttt : TTargetTypes; Const Dest : String; Const InstallMode: TInstallMode):Boolean;
    Function FileNewer(const Src,Dest : String) : Boolean;
    Procedure LogSearchPath(APackage: TPackage;const ASearchPathName:string;Path:TConditionalStrings; ACPU:TCPU;AOS:TOS);
    Function FindFileInPath(APackage: TPackage; Path:TConditionalStrings; AFileName:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;

    procedure GetDirectoriesFromFilelist(const AFileList, ADirectoryList: TStringList);
    procedure AddPackageMacrosToDictionary(const APackage: TPackage; ADictionary: TDictionary);
    //package commands
    function  GetUnitDir(APackage:TPackage):String;
    procedure ResolvePackagePaths(APackage:TPackage);
    procedure AddDependencyPaths(L: TStrings; DependencyType: TDependencyType; ATarget: TTarget);
    procedure AddDependencyUnitPaths(L:TStrings;APackage: TPackage);
    procedure AddDependencyTransmittedOptions(Args: TStrings; APackage: TPackage);
    procedure GetDocProject(Proj: TStrings; P: TPackage; aIndent: string); virtual;

  Public
    Constructor Create(AOwner : TComponent); override;
    destructor Destroy;override;
    function AddPathPrefix(APackage: TPackage; APath: string): string;

    property Verbose : boolean read FVerbose write FVerbose;
    property Interactive : boolean read FInteractive write FInteractive;
    Procedure ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;DoChangeDir:boolean=true; WarnIfNotFound:boolean=true);
    Procedure ClearResolvedFileNames(APackage : TPackage);

    // Public Copy/delete/Move/Archive/Mkdir Commands.
    Procedure ExecuteCommand(const Cmd,Args : String; const Env: TStrings = nil; IgnoreError : Boolean = False); virtual;
    procedure CmdCopyFiles(List: TStrings; const DestDir: String; APackage: TPackage);
    Procedure CmdCreateDir(const DestDir : String);
    Procedure CmdMoveFiles(List : TStrings; Const DestDir : String);
    Procedure CmdDeleteFiles(List : TStrings);
    procedure CmdDeleteDestFiles(List: TStrings; const DestDir: String);
    Procedure CmdArchiveFiles(List : TStrings; Const ArchiveFile : String);
    Procedure CmdRenameFile(SourceName, DestName : String);
    Procedure CmdRemoveDirs(List: TStrings);
    Procedure CmdRemoveTrees(List: TStrings);
    Procedure ExecuteCommands(Commands : TCommands; At : TCommandAt; APackage: TPackage = nil);
    // Dependency commands
    Function  DependencyOK(ADependency : TDependency) : Boolean;
    // Target commands
    Function  GetCompilerCommand(APackage : TPackage; ATarget : TTarget; Env: TStrings) : String;
    Function  TargetOK(ATarget : TTarget; ACPU: TCPU; AOS: TOS) : Boolean;
    Function  TargetInstallOK(ATarget : TTarget;ACPU:TCPU; AOS : TOS) : Boolean;
    Function  NeedsCompile(APackage:TPackage; ATarget : TTarget) : Boolean;
    Procedure Compile(APackage:TPackage; ATarget : TTarget);  virtual;
    Procedure MaybeCompile(APackage:TPackage; ATarget: TTarget);
    Procedure CompileDependencies(APackage:TPackage; ATarget: TTarget);
    // Package commands
{    Function  GetPackageDir(APackage : TPackage; AbsolutePath : Boolean = False) : String;
    Function  GetUnitsOutputDir(APackage : TPackage; AbsolutePath : Boolean = False) : String;
    Function  GetBinOutputDir(APackage : TPackage; AbsolutePath : Boolean = False) : String;  }
    Function  PackageOK(APackage : TPackage) : Boolean; virtual;
    Procedure DoBeforeCompile(APackage : TPackage);virtual;
    Procedure DoAfterCompile(APackage : TPackage);virtual;
    Procedure DoBeforeInstall(APackage : TPackage);virtual;
    Procedure DoAfterInstall(APackage : TPackage);virtual;
    Procedure DoBeforeArchive(APackage : TPackage);virtual;
    Procedure DoAfterArchive(APackage : TPackage);virtual;
    Procedure DoBeforeClean(APackage : TPackage);virtual;
    Procedure DoAfterClean(APackage : TPackage);virtual;
    Function  NeedsCompile(APackage : TPackage) : Boolean; virtual;
    Procedure Compile(APackage : TPackage);
    Procedure MaybeCompile(APackage:TPackage);
    Function ReadyToCompile(APackage:TPackage) : Boolean;
    Procedure Install(APackage : TPackage; AnArchiveFiles: boolean);
    Procedure UnInstall(APackage : TPackage);
    Procedure Archive(APackage : TPackage);
    Procedure PkgList(PkgList: TStrings; APackage : TPackage);
    Procedure Clean(APackage : TPackage; AllTargets: boolean);
    Procedure Clean(APackage : TPackage; ACPU:TCPU; AOS : TOS);
    Procedure CompileDependencies(APackage : TPackage);
    function CheckDependencies(APackage : TPackage; ErrorOnFailure: boolean): TCheckDependencyResult;
    Function  CheckExternalPackage(Const APackageName, ForPackageName : String; ErrorOnFailure: boolean):TPackage;
    procedure CreateOutputDir(APackage: TPackage);
    // Packages commands
    Procedure Compile(Packages : TPackages);
    Procedure Install(Packages : TPackages);
    Procedure UnInstall(Packages : TPackages);
    Procedure ZipInstall(Packages : TPackages);
    Procedure Archive(Packages : TPackages);
    procedure Manifest(Packages: TPackages; Package: TPackage);
    procedure PkgList(Packages: TPackages);
    procedure FPDocProject(Packages: TPackages; SingleDocFile : Boolean);
    Procedure Clean(Packages : TPackages; AllTargets: boolean);

    Procedure Log(Level : TVerboseLevel; Msg : String);
    Procedure Log(Level : TVerboseLevel; Fmt : String; const Args : Array Of Const);

    Property ListMode : Boolean Read FListMode Write FListMode;
    Property ForceCompile : Boolean Read FForceCompile Write FForceCompile;
    Property ExternalPackages: TPackages Read FExternalPackages;
    Property StartDir: String Read FStartDir;

    // Events
    Property NotifyEventCollection: TNotifyEventCollection read FNotifyEventCollection;
    Property OnLog : TLogEvent Read FOnLog Write FOnlog;
  end;

  { TCustomInstaller }

  TCustomInstaller = Class(TComponent)
  private
    FBuildEngine: TBuildEngine;
    FPackages: TPackages;
    FRunMode: TRunMode;
    FListMode : Boolean;
    FInteractive : boolean;
    FLogLevels : TVerboseLevels;
    FFPMakeOptionsString: string;
    FPackageVariantSettings: TStrings;
    FPackageVariants: TFPList;
    FNotifyEventCollection: TNotifyEventCollection;
  Protected
    Procedure Log(Level : TVerboseLevel; Const Msg : String);
    Procedure CreatePackages; virtual;
    Procedure FreePackages; virtual;
    function GetPackages: TPackages; virtual;
    Procedure CheckPackages; virtual;
    Procedure CreateBuildEngine; virtual;
    Procedure Error(const Msg : String);
    Procedure Error(const Fmt : String; const Args : Array of const);
    Procedure AnalyzeOptions;
    Procedure Usage(const FMT : String; const Args : Array of const);
    Procedure Compile(Force : Boolean); virtual;
    Procedure Clean(AllTargets: boolean); virtual;
    Procedure Install(ForceBuild : Boolean); virtual;
    Procedure UnInstall; virtual;
    Procedure ZipInstall; virtual;
    Procedure Archive; virtual;
    Procedure Manifest; virtual;
    Procedure PkgList; virtual;
    Procedure FPDocProject; virtual;
    Procedure Info; virtual;
    procedure AddAutoPackageVariantsToPackage(APackage: TPackage); virtual;
  Public
    Constructor Create(AOwner : TComponent); virtual;
    Destructor destroy; override;
    Function AddPackage(Const AName : String) : TPackage;
    Function AddPackageVariant(AName: string; AIsInheritable: boolean; AutoAddToPackage: Boolean = false): TPackageVariants;
    Function Run : Boolean;
    Property FPMakeOptionsString: string read FFPMakeOptionsString;
    Property BuildEngine : TBuildEngine Read FBuildEngine;
    //files in package
    Property Packages : TPackages Read GetPackages;
    Property RunMode : TRunMode Read FRunMode;
    Property ListMode : Boolean Read FListMode;
    Property NotifyEventCollection : TNotifyEventCollection read FNotifyEventCollection;
  end;

  { TFPCInstaller }
  TFPCInstaller = class(TCustomInstaller)
  public
    Constructor Create(AOwner : TComponent); override;
  end;

  { TBasicInstaller }
  TBasicInstaller = class(TCustomInstaller)
    Constructor Create(AOwner : TComponent); override;
  end;

  { TValueItem }

  TValueItem = Class(TObject)
    FValue : String;
    Constructor Create(AValue : String);
  end;

  { TFunctionItem }

  TFunctionItem = Class(TObject)
    FFunc : TReplaceFunction;
    Constructor Create(AFunc : TReplaceFunction);
  end;

{$ifndef NO_THREADING}

  { TCompileWorkerThread }

  TCompileWorkerThread = class(TThread)
  private
    FBuildEngine: TBuildEngine;
    FCompilationOK: boolean;
    FDone: boolean;
    FErrorMessage: string;
    FNotifyMainThreadEvent: PRTLEvent;
    FNotifyStartTask: PRTLEvent;
    FPackage: TPackage;
  protected
    procedure execute; override;
    property Done: boolean read FDone;
    property APackage: TPackage read FPackage write FPackage;
    property CompilationOK: boolean read FCompilationOK;
    property NotifyStartTask: PRTLEvent read FNotifyStartTask;
    property ErrorMessage: string read FErrorMessage;
  public
    constructor Create(ABuildEngine: TBuildEngine; NotifyMainThreadEvent: PRTLEvent); virtual;
    destructor Destroy; override;
  end;

{$endif NO_THREADING}

  { TfpmPlugin }

  TfpmPlugin = class
  protected
    function GetName: string; virtual;
  public
    property Name: string read GetName;

    procedure BeforeResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; out AContinue: Boolean); virtual;
    procedure ResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; SearchDirectory: string; out AContinue: Boolean); virtual;
    procedure AfterResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; out AContinue: Boolean); virtual;
  end;
  TfpmPluginClass = class of TfpmPlugin;

  { TfpmPluginManager }

  TfpmPluginManager = class(TfpmPlugin)
  private
    FPlugins: array of TfpmPlugin;
  public
    destructor Destroy; override;
    procedure RegisterPlugin(APlugin: TfpmPluginClass);

    procedure BeforeResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; out AContinue: Boolean); override;
    procedure ResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; SearchPath: string; out AContinue: Boolean); override;
    procedure AfterResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; out AContinue: Boolean); override;
  end;

  { TfpmResolvePackagePathsPlugin }

  TfpmResolvePackagePathsPlugin = class(TfpmPlugin)
  private
    procedure ResolveUnitConfigFilenameForBasePath(ABuildEngine: TBuildEngine; APackage: TPackage; ABasePath: string;
      out AContinue: Boolean);
  public
    procedure BeforeResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; out AContinue: Boolean); override;
    procedure ResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage; SearchPath: string;
      out AContinue: Boolean); override;
  end;

  ECollectionError = Class(Exception);
  EDictionaryError = Class(Exception);
  EInstallerError = Class(Exception);

  TInstallerClass = Class of TCustomInstaller;
  TDictionaryClass = Class of TDictionary;
  TPackageDictionaryClass = Class of TPackageDictionary;

Type
  TArchiveEvent = Procedure (Const AFileName : String; List : TStrings) of Object;
  TArchiveProc = Procedure (Const AFileName : String; List : TStrings);

Var
  DictionaryClass : TDictionaryClass = TDictionary;
  PackageDictionaryClass : TPackageDictionaryClass = TPackageDictionary;
  OnArchiveFiles : TArchiveEvent = Nil;
  ArchiveFilesProc : TArchiveProc = Nil;

  Defaults : TCustomDefaults; // Set by installer.
  GlobalDictionary : TDictionary;


Function CurrentOS : String;
Function CurrentCPU : String;

Function Installer(InstallerClass: TInstallerClass) : TCustomInstaller; overload;
Function Installer : TCustomInstaller; overload;

Function OSToString(OS: TOS) : String;
Function OSesToString(OSes: TOSes) : String;
Function CPUToString(CPU: TCPU) : String;
Function CPUSToString(CPUS: TCPUS) : String;
Function StringToOS(const S : String) : TOS;
function IsDifferentFromBuild(ACpu: TCPU; AOs: TOs): boolean;
//Function StringToOSes(const S : String) : TOSes;
Function StringToCPU(const S : String) : TCPU;
Function StringToCPUS(const S : String) : TCPUS;
Function ModeToString(Mode: TCompilerMode) : String;
Function StringToMode(const S : String) : TCompilerMode;
Function MakeTargetString(CPU : TCPU;OS: TOS) : String;
Procedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TOS);
Function FixPath (const APath : String) : String; inline; deprecated 'Use the overload with AIsDir instead';
Function FixPath (const APath : String; AIsDir : Boolean) : String;
Function IsRelativePath(const APath : String) : boolean;
Procedure ChangeDir(const APath : String);
Procedure SplitCommand(Const Cmd : String; out Exe,Options : String);
Procedure AddCustomFpmakeCommandlineOption(const ACommandLineOption, HelpMessage : string);
Function GetCustomFpmakeCommandlineOptionValue(const ACommandLineOption : string) : string;
Function AddProgramExtension(const ExecutableName: string; AOS : TOS) : string;
Function AddLibraryExtension(const LibraryName: string; AOS : TOS) : string;
Function GetImportLibraryFilename(const UnitName: string; AOS : TOS) : string;

procedure SearchFiles(AFileName, ASearchPathPrefix: string; Recursive: boolean; var List: TStrings);
function GetDefaultLibGCCDir(CPU : TCPU;OS: TOS; var ErrorMessage: string): string;

function GetPluginManager: TfpmPluginManager;

Implementation

uses typinfo, rtlconsts;

const
{$ifdef CREATE_TAR_FILE}
  {$ifdef HAS_UNIT_ZIPPER}
  ArchiveExtension = '.tar.gz';
  {$else }
  ArchiveExtension = '.tar';
  {$endif HAS_UNIT_ZIPPER}
{$else CREATE_TAR_FILE}
  ArchiveExtension = '.zip';
{$endif CREATE_TAR_FILE}

var
  GPluginManager: TfpmPluginManager;


{----------------- from strutils ---------------------}

function FindPart(const HelpWilds, inputStr: string): Integer;
var
  i, J: Integer;
  Diff: Integer;
begin
  Result:=0;
  i:=Pos('?',HelpWilds);
  if (i=0) then
    Result:=Pos(HelpWilds, inputStr)
  else
    begin
    Diff:=Length(inputStr) - Length(HelpWilds);
    for i:=0 to Diff do
      begin
      for J:=1 to Length(HelpWilds) do
        if (inputStr[i + J] = HelpWilds[J]) or (HelpWilds[J] = '?') then
          begin
          if (J=Length(HelpWilds)) then
            begin
            Result:=i+1;
            Exit;
            end;
          end
        else
          Break;
      end;
    end;
end;

function isWild(inputStr, Wilds: string; ignoreCase: Boolean): Boolean;

 function SearchNext(var Wilds: string): Integer;

 begin
   Result:=Pos('*', Wilds);
   if Result>0 then
     Wilds:=Copy(Wilds,1,Result - 1);
 end;

var
  CWild, CinputWord: Integer; { counter for positions }
  i, LenHelpWilds: Integer;
  MaxinputWord, MaxWilds: Integer; { Length of inputStr and Wilds }
  HelpWilds: string;
begin
  if Wilds = inputStr then begin
    Result:=True;
    Exit;
  end;
  repeat { delete '**', because '**' = '*' }
    i:=Pos('**', Wilds);
    if i > 0 then
      Wilds:=Copy(Wilds, 1, i - 1) + '*' + Copy(Wilds, i + 2, Maxint);
  until i = 0;
  if Wilds = '*' then begin { for fast end, if Wilds only '*' }
    Result:=True;
    Exit;
  end;
  MaxinputWord:=Length(inputStr);
  MaxWilds:=Length(Wilds);
  if ignoreCase then begin { upcase all letters }
    inputStr:=AnsiUpperCase(inputStr);
    Wilds:=AnsiUpperCase(Wilds);
  end;
  if (MaxWilds = 0) or (MaxinputWord = 0) then begin
    Result:=False;
    Exit;
  end;
  CinputWord:=1;
  CWild:=1;
  Result:=True;
  repeat
    if inputStr[CinputWord] = Wilds[CWild] then begin { equal letters }
      { goto next letter }
      inc(CWild);
      inc(CinputWord);
      Continue;
    end;
    if Wilds[CWild] = '?' then begin { equal to '?' }
      { goto next letter }
      inc(CWild);
      inc(CinputWord);
      Continue;
    end;
    if Wilds[CWild] = '*' then begin { handling of '*' }
      HelpWilds:=Copy(Wilds, CWild + 1, MaxWilds);
      i:=SearchNext(HelpWilds);
      LenHelpWilds:=Length(HelpWilds);
      if i = 0 then begin
        { no '*' in the rest, compare the ends }
        if HelpWilds = '' then Exit; { '*' is the last letter }
        { check the rest for equal Length and no '?' }
        for i:=0 to LenHelpWilds - 1 do begin
          if (HelpWilds[LenHelpWilds - i] <> inputStr[MaxinputWord - i]) and
            (HelpWilds[LenHelpWilds - i]<> '?') then
          begin
            Result:=False;
            Exit;
          end;
        end;
        Exit;
      end;
      { handle all to the next '*' }
      inc(CWild, 1 + LenHelpWilds);
      i:=FindPart(HelpWilds, Copy(inputStr, CinputWord, Maxint));
      if i= 0 then begin
        Result:=False;
        Exit;
      end;
      CinputWord:=i + LenHelpWilds;
      Continue;
    end;
    Result:=False;
    Exit;
  until (CinputWord > MaxinputWord) or (CWild > MaxWilds);
  { no completed evaluation }
  if CinputWord <= MaxinputWord then Result:=False;
  if (CWild <= MaxWilds) and (Wilds[MaxWilds] <> '*') then Result:=False;
end;


type
  TUnsortedDuplicatesStringList = class(TStringList)
  public
    function Add(const S: string): Integer; override;
  end;

var
  CustomFpmakeCommandlineOptions: TStrings;
  CustomFpMakeCommandlineValues: TStrings;

{$ifdef NO_THREADING}
var
{$else NO_THREADING}
threadvar
{$endif NO_THREADING}
  GPathPrefix : string;
  GLogPrefix  : string;

ResourceString
  SErrInvalidCPU        = 'Invalid CPU name "%s"';
  SErrInvalidOS         = 'Invalid OS name "%s"';
  SErrInvalidMode       = 'Invalid compiler mode "%s"';
  SErrInvalidTarget     = 'Invalid compiler target "%s"';
  SErrNameExists        = 'Name "%s" already exists in the collection.';
  SErrNoSuchName        = 'Could not find item with name "%s" in the collection.';
  SErrInValidArgument   = 'Invalid command-line argument at position %d: %s';
  SErrNeedArgument      = 'Option at position %d (%s) needs an argument';
  SErrNoPackagesDefined = 'No action possible: No packages were defined.';
  SErrInstaller         = 'The installer encountered the following error:';
  SErrDepUnknownTarget  = 'Unknown target for unit "%s" in dependencies for %s in package %s';
  SErrExternalCommandFailed = 'External command "%s" failed with exit code %d. Console output:'+LineEnding+'%s';
  SErrExtCommandNotFound= 'External command "%s" not found';
  SErrCreatingDirectory = 'Failed to create directory "%s"';
  SErrDeletingFile      = 'Failed to delete file "%s"';
  SErrRemovingDirectory = 'Failed to remove directory "%s"';
  SErrMovingFile        = 'Failed to move file "%s" to "%s"';
  SErrCopyingFile       = 'Failed to copy file "%s" to "%s"';
  SErrChangeDirFailed   = 'Failed to enter directory "%s"';
  SErrInvalidArgumentToSubstitute = 'Invalid number of arguments to Substitute';
  SErrNoArchiveSupport  = 'This binary contains no archive support. Please recompile with archive support';
  SErrNoDictionaryItem  = 'No item called "%s" in the dictionary';
  SErrNoDictionaryValue = 'The item "%s" in the dictionary is not a value';
  SErrNoDictionaryFunc  = 'The item "%s" in the dictionary is not a function';
  SErrInvalidFPCInfo    = 'Compiler returns invalid information, check if fpc -iV works';
  SErrDependencyNotFound = 'Could not find unit directory for dependency package "%s" required for package "%s"';
  SErrAlreadyInitialized = 'Installer can only be initialized once';
  SErrInvalidState      = 'Invalid state for target %s';
  SErrCouldNotCompile   = 'Could not compile target %s from package %s';
  SErrUnsupportedBuildmode = 'Package does not support this buildmode';
  SErrPackVarNotExist   = 'There is no package variant with the name "%s"';
  SErrEventNotSupported = 'Unsupported event type';
  SErrorPkgNotInstalled = 'Package "%s" is not installed, can not uninstall.';
  SErrBuildUnitCompilation = 'Compilation of "%s" failed';
  SErrExpectPkgVariant  = 'The Package-variant on the command-line position %d (%s) should have at least one item';

  SWarnCircularTargetDependency = 'Warning: Circular dependency detected when compiling target %s with target %s';
  SWarnCircularPackageDependency = 'Warning: Circular dependency detected when compiling package %s with package %s';
  SWarnFailedToSetTime    = 'Warning: Failed to set timestamp on file "%s"';
  SWarnFailedToGetTime    = 'Warning: Failed to get timestamp from file "%s"';
  SWarnAttemptingToCompileNonNeutralTarget = 'Warning: Attempting to compile non-neutral target %s';
  SWarnSourceFileNotFound  = 'Warning: Source file "%s" from package %s not found for %s';
  SWarnIncludeFileNotFound = 'Warning: Include file "%s" from package %s not found for %s';
  SWarnDepUnitNotFound     = 'Warning: Dependency on unit %s is not supported for %s';
  SWarnTargetDependsOnPackage = 'Warning: Target %s of package %s depends on another package (%s). These kind of dependencies are not processed';
  SWarnDependOnOtherPlatformPackage = 'Warning: Package %s depends on package %s which is not available for the %s platform';
  SWarnStartCompilingPackage = 'Start compiling package %s for target %s.';
  SWarnCompilingPackagecompleteProgress = '[%3.0f%%] Compiled package %s';
  SWarnCompilingPackagecomplete = 'Compiled package %s';
  SWarnInstallationPackagecomplete = 'Installation package %s for target %s succeeded';
  SWarnCanNotGetAccessRights = 'Warning: Failed to copy access-rights from file %s';
  SWarnCanNotSetAccessRights = 'Warning: Failed to copy access-rights to file %s';
  SWarnCanNotGetFileAge = 'Warning: Failed to get FileAge for %s';
  SWarnExtCommandNotFound = 'Warning: External command "%s" not found but "%s" is older then "%s"';
  SWarnDuplicatePackage = 'Warning: Package %s is already added. Using the existing package';
  SWarngccNotFound        = 'Could not find libgcc';
  SWarngcclibpath         = 'Warning: Unable to determine the libgcc path.';
  SWarnNoFCLProcessSupport= 'No FCL-Process support';
  SWarnRetryRemDirectory     = 'Failed to remove directory "%s". Retry after a short delay';
  SWarnRetryDeleteFile       = 'Failed to remove file "%s". Retry after a short delay';
  SWarnCombinedPathAndUDir= 'Warning: Better do not combine the SearchPath and Global/Local-UnitDir parameters';
  SWarnRemovedNonEmptyDirectory = 'Warning: Removed non empty directory "%s"';

  SInfoPackageAlreadyProcessed = 'Package %s is already processed';
  SInfoSkipPackageTargetProgress = '[%3.0f%%] Skipped package %s which has been disabled for target %s';
  SInfoSkipPackageTarget = 'Skipped package %s which has been disabled for target %s';
  SInfoCompilingTarget    = 'Compiling target %s';
  SInfoExecutingCommand   = 'Executing command "%s %s"';
  SInfoCreatingOutputDir  = 'Creating output dir "%s"';
  SInfoInstallingPackage  = 'Installing package %s';
  SInfoUnInstallingPackage= 'Uninstalling package %s';
  SInfoArchivingPackage   = 'Archiving package %s in "%s"';
  SInfoCleaningPackage    = 'Cleaning package %s';
  SInfoCleanPackagecomplete = 'Clean of package %s completed';
  SInfoManifestPackage    = 'Creating manifest for package %s';
  SInfoPackageDocProject  = 'Creating fpdoc project file for package %s';
  SInfoPkgListPackage    = 'Adding package %s to the package list';
  SInfoCopyingFile        = 'Copying file "%s" to "%s"';
  SInfoDeletedFile        = 'Deleted file "%s"';
  SInfoRemovedDirectory   = 'Removed directory "%s"';
  SInfoSourceNewerDest    = 'Source file "%s" (%s) is newer than destination "%s" (%s).';
  SInfoDestDoesNotExist   = 'Destination file "%s" does not exist.';
  SInfoFallbackBuildmode  = 'Buildmode not supported by package, falling back to one by one unit compilation';
  SInfoFallbackBuildmodeBU= 'Buildmode not supported by package, falling back to compilation using a buildunit';
  SInfoDirectoryNoPackage = 'Found directory "%s" while searching for package "%s" which does not contain a package';
  SInfoSrcDirectoryNoPkg  = 'Found source-directory "%s" while searching for package "%s" but the package is not compiled or has the wrong name';

  SDbgComparingFileTimes    = 'Comparing file "%s" time "%s" to "%s" time "%s".';
  SDbgCompilingDependenciesOfTarget = 'Compiling dependencies of target %s';
  SDbgResolvingSourcesOfTarget = 'Resolving filenames of target %s for %s';
  SDbgResolvedSourceFile    = 'Resolved source file %s to "%s"';
  SDbgSourceAlreadyResolved = 'Source file of %s has been resolved earlier';
  SDbgResolvedIncludeFile   = 'Resolved include file %s to "%s"';
  SDbgOutputNotYetAvailable = 'Output file %s not available';
  SDbgDependencyOnUnit      = 'Dependency of %s on unit %s';
  SDbgDependencyUnitRecompiled = 'Dependent unit %s is being recompiled';
  SDbgMustCompile           = 'Must compile %s. (%s)';
  SDbgSkippingTargetWrongCPU = 'Skipping target %s, different CPU (%s)';
  SDbgSkippingTargetWrongOS  = 'Skipping target %s, different OS (%s)';
  SDbgTargetIsNotAUnitOrProgram = 'Skipping Target %s, not an unit or program';
  SDbgConsideringTarget     = 'Considering target %s';
  SDbgConsideringPackage    = 'Considering package %s';
  SDbgSearchExtDepPath      = 'Search path for external dependency %s';
  SDbgExternalDependency    = 'External dependency %s found in "%s"';
  SDbgBuildEngineArchiving  = 'Build engine archiving';
  SDbgBuildEngineGenerateManifests = 'Build engine generating manifests';
  SDbgBuildEngineGeneratePkgList = 'Build engine generating package list';
  SDbgBuildEngineCleaning   = 'Build engine cleaning';
  SDbgGenerating            = 'Generating "%s"';
  SDbgLoading               = 'Loading "%s"';
  SDbgFound                 = 'Found';
  SDbgNotFound              = 'Not Found';
  SDbgDirectoryExists       = 'Directory "%s" %s';
  SDbgFileExists            = 'File "%s" %s';
  SDbgArchivingFile         = 'Archiving "%s"';
  SDbgSearchPath            = 'Using %s path "%s"';
  SDbgEnterDir              = 'Entering directory "%s"';
  SDbgPackageChecksumChanged = 'Package %s depends on the %s package which has been modified';
  SDbgFileDoesNotExist      = 'File "%s" does not exist';
  SDbgDirectoryDoesNotExist = 'Directory "%s" does not exist';
  SDbgDirectoryNotEmpty     = 'Directory "%s" is not empty. Will not remove';
  SDbgGenerateBuildUnit     = 'Generate build-unit %s';
  SDbgBuildUnitFailure      = 'Generate build-unit %s failed';
  SDbgForcedCompile         = 'Forced compile';
  SDbgOutputDoesNotExist    = 'Output file does not exist';
  SDbgNewerSource           = 'Source file is newer then output file';
  SDbgNewerInclude          = 'The include file %s is newer then output file';
  SDbgDependencyRecompiled  = 'The unit %s where this unit depends on is recompiled';
  SDbgPackageDepRecompiled  = 'The package %s where this package depends on is recompiled';
  SDbgTargetHasToBeCompiled = 'At least one of the targets in the package has to be compiled.';
  SDbgDeletedFile           = 'Recursively deleted file "%s"';
  SDbgRemovedDirectory      = 'Recursively removed directory "%s"';
  SDbgUnregisteredResource  = 'Adding resource file "%s", which is not registered.';
  SDbgSearchingDir          = 'Searching dir  %s.';

  // Help messages for usage
  SValue              = 'Value';
  SHelpUsage          = 'Usage: %s command [options]';
  SHelpCommand        = 'Where command is one of the following:';
  SHelpCompile        = 'Compile all units in the package(s).';
  SHelpBuild          = 'Build all units in the package(s).';
  SHelpInstall        = 'Install all units in the package(s).';
  SHelpBuildInstall   = 'Build and install all units in the package(s).';
  SHelpUnInstall      = 'Uninstall the package(s).';
  SHelpClean          = 'Clean (remove) all generated files in the package(s) for current CPU-OS target.';
  SHelpDistclean      = 'Clean (remove) all generated files in the package(s) for all targets.';
  SHelpArchive        = 'Create archive (zip) with all units in the package(s).';
  SHelpHelp           = 'This message.';
  SHelpManifest       = 'Create a manifest suitable for import in repository.';
  SHelpPkgList        = 'Create list of all packages suitable for FPC installer.';
  SHelpFPDocProject   = 'Create fpdoc project file(s) for all packages';
  SHelpZipInstall     = 'Install all units in the package(s) into an archive.';
  SHelpCmdOptions     = 'Where options is one or more of the following:';
  SHelpCPU            = 'Compile for indicated CPU.';
  SHelpOS             = 'Compile for indicated OS';
  SHelpTarget         = 'Compile for indicated target';
  SHelpList           = 'list commands instead of actually executing them.';
  SHelpPrefix         = 'Use indicated prefix directory for all commands.';
  SHelpNoFPCCfg       = 'Compiler will not use fpc.cfg';
  SHelpBaseInstallDir = 'Use indicated directory as base install dir.';
  SHelpLocalUnitDir   = 'Use indicated directory as local (user) unit dir.';
  SHelpGlobalUnitDir  = 'Use indicated directory as global unit dir.';
  SHelpSearchPath     = 'Add search directory for packages.';
  SHelpUnitInstallDir = 'Use indicated directory to install units into.';
  SHelpCompiler       = 'Use indicated binary as compiler';
  SHelpConfig         = 'Use indicated config file when compiling.';
  SHelpOptions        = 'Pass extra options to the compiler.';
  SHelpVerbose        = 'Be verbose when working.';
  SHelpDebug          = 'Add debug information when working.';
  SHelpInteractive    = 'Allow to interact with child processes';
  SHelpInstExamples   = 'Install the example-sources.';
  SHelpSkipCrossProgs = 'Skip programs when cross-compiling/installing';
  SHelpIgnoreInvOpt   = 'Ignore further invalid options.';
  sHelpFpdocOutputDir = 'Use indicated directory as fpdoc output folder.';
  sHelpSingleFpdocFile = 'Create a single fpdoc project file for all projects';
  sHelpDocOptionsFile = 'Name=Value File with options for fpdoc project file';
  sHelpFPUnitSrcPath  = 'Sourcepath to replace in fpunits.cfg on installation.';
  sHelpThreads        = 'Enable the indicated amount of worker threads.';
  sHelpUseEnvironment = 'Use environment to pass options to compiler.';
  SHelpUseBuildUnit   = 'Compile package in Build-unit mode.';
  sHelpZipPrefix      = 'Use indicated prefix for generated archives.';
  sHelpPackageVariant1= 'To add a package-variant to all packages:';
  sHelpPackageVariant2= ' +[variantname]+=[variant1],<variant2>,...';
  sHelpPackageVariant3= 'To add a package-variant to all packages which other packages will inherit:';
  sHelpPackageVariant4= ' +[variantname]*=[variant1],<variant2>,...';
  sHelpPackageVariant5= 'To add specific options for one package-variant:';
  sHelpPackageVariant6= ' --options_[variantname]_[variant1]=Value';


Const
  // Keys for Defaults file. Do not localize.
  KeyCompiler = 'Compiler';
  KeyArchive  = 'Archive';
  KeyCopy     = 'Copy';
  KeyMkDir    = 'MkDir';
  KeyMove     = 'Move';
  KeyRemove   = 'Remove';
  KeyRemoveDir= 'RemoveDir';
  KeyRemoveTree= 'RemoveTree';
  KeyOptions  = 'Options';
  KeyCPU      = 'CPU';
  KeyOS       = 'OS';
  KeyMode     = 'Mode';
  KeyPrefix   = 'Prefix';
  KeyTarget   = 'Target';
  KeyNoFPCCfg = 'NoFPCCfg';
  KeyUseEnv   = 'UseEnv';
  KeyPluginUnits        = 'PluginUnits';
  KeyLocalUnitDir       = 'LocalUnitDir';
  KeyGlobalUnitDir      = 'GlobalUnitDir';
  KeyBaseInstallDir     = 'BaseInstallDir';
  KeyUnitInstallDir     = 'UnitInstallDir';
  KeyBinInstallDir      = 'BinInstallDir';
  KeyLibInstallDir      = 'LibInstallDir';
  KeyDocInstallDir      = 'DocInstallDir';
  KeyExamplesInstallDir = 'ExamplesInstallDir';
  KeyInstallExamples    = 'InstallExamples';
  KeySkipCrossProdrams  = 'SkipCrossPrograms';
  // Keys for unit config
  KeyName     = 'Name';
  KeyVersion  = 'Version';
  KeyChecksum = 'Checksum';
  KeyNeedLibC = 'NeedLibC';
  KeyDepends  = 'Depends';
  KeyFlags    = 'Flags';
  KeyTransmit = 'TransmitOptions';
  KeyAddIn    = 'FPMakeAddIn';
  KeySourcePath = 'SourcePath';
  KeyFPMakeOptions = 'FPMakeOptions';
  KeyPackageVar = 'PackageVariant_';

{****************************************************************************
                                Helpers
****************************************************************************}

{$ifdef HAS_UNIT_PROCESS}
function ExecuteFPC(Verbose, Interactive: boolean; const Path: string; const ComLine: string; const Env: TStrings; ConsoleOutput: TMemoryStream): integer;
var
  P: TProcess;
  BytesRead: longint;

  function ReadFromStream(const ReadFromStdErr: boolean): longint;

  const
    READ_BYTES = 2048;

  type
    TMessages = (mCompiling, mLinking);

  var
    //ifdef the MsgNum so it contains the correct message numbers for each compiler version.
    MsgNum : array [TMessages] of integer = (3104, 9015);

    n,available: longint;
    BuffPos: longint;
    sLine: string;
    ch: char;
    msg: TMessages;
    ipos: integer;
    snum: string;
  begin

    // try reading it
    if ReadFromStdErr then
      begin
        available:=P.Stderr.NumBytesAvailable;
        // make sure we have room
        if (bytesRead + Available > ConsoleOutput.Size) then
          ConsoleOutput.SetSize(BytesRead + Available);
        n := P.Stderr.Read((ConsoleOutput.Memory + BytesRead)^, available);
      end
    else
      begin
        available:=P.Output.NumBytesAvailable;
        // make sure we have room
        if (bytesRead + Available > ConsoleOutput.Size) then
          ConsoleOutput.SetSize(BytesRead + Available);
        n := P.Output.Read((ConsoleOutput.Memory + BytesRead)^, available);
      end;
    if n > 0 then
    begin
      Inc(BytesRead, n);

      sLine := '';
      BuffPos := ConsoleOutput.Position;

      //read lines from the stream
      repeat
        ConsoleOutput.Read(ch,1);

        if ch in [#10, #13] then
        begin
          if Interactive then
            System.Writeln(output)
          else if Verbose then
            installer.log(vlInfo,sLine)
          else
            begin
              for msg := Low(TMessages) to High(TMessages) do
              begin
                snum := Format('(%d)', [MsgNum[msg]]);
                ipos := Pos(snum, sLine);
                if ipos = 1 then
                  installer.log(vlCommand,'      '+ Copy(sLine, ipos + Length(snum), Length(sLine) - ipos - Length(snum) + 1));
              end;
            end;
          if (LineEnding=#13#10) and (ch=#13) and
             (ConsoleOutput.Position<BytesRead) then
            begin
              ConsoleOutput.Read(ch,1);
              if ch=#10 then
                sLine:=''
              else
                begin
                  if Interactive then
                    begin
                      System.Write(output,ch);
                    end
                  else
                    sLine:=ch;
                end;
            end
          else
            sLine := '';
          BuffPos := ConsoleOutput.Position;
        end
        else
        begin
          if Interactive then
            System.Write(output,ch)
          else
            sLine := sLine + ch;
        end;

      until ConsoleOutput.Position >= BytesRead;

      // keep partial lines, unlessin interactive mode
      if not Interactive then
        ConsoleOutput.Position := BuffPos
        // Flush for interactive mode
      else if n > 0 then
        System.Flush(output);
    end;

    Result := n;
  end;

begin
  result := -1;
  BytesRead := 0;
  P := TProcess.Create(nil);
  try
    P.CommandLine := Path + ' ' + ComLine;
    if assigned(Env) then
      P.Environment.Assign(Env);

    if Interactive then
      P.Options := [poUsePipes,poPassInput]
    else
      P.Options := [poUsePipes];

    P.Execute;
    while P.Running do
      begin
        // Only call ReadFromStream if Data from corresponding stream
        // is already available, otherwise, on  linux, the read call
        // is blocking, and thus it is not possible to be sure to handle
        // big data amounts bboth on output and stderr pipes. PM.
        if P.Output.NumBytesAvailable > 0 then
          ReadFromStream(false)
        else if P.StdErr.NumBytesAvailable > 0 then
          ReadFromStream(true)
        else
      // no data, wait 100 ms
          Sleep(100);
      end;

    // read last part
    repeat
    until ReadFromStream(false)=0;

    // read stderr
    // JvdS: Note that this way stderr is added to the end of the stream. But I
    // see no way showing the stderr output at the place it was actually written
    repeat
    until ReadFromStream(true)=0;
    ConsoleOutput.SetSize(BytesRead);

    result := P.ExitStatus;
  finally
    P.Free;
  end;
end;
{$endif HAS_UNIT_PROCESS}

function IsDirectoryEmpty(const directory : string) : boolean;
var
  searchRec: TSearchRec;
  SearchResult: longint;
begin
  result := true;
  SearchResult := FindFirst(IncludeTrailingPathDelimiter(directory)+AllFilesMask, faAnyFile+faSymLink, searchRec);
  try
    while SearchResult=0 do
      begin
        if (searchRec.Name<>'.') and (searchRec.Name<>'..') then
           begin
             result := false;
             break;
           end;
        SearchResult := FindNext(searchRec);
      end;
  finally
    sysutils.FindClose(searchRec);
  end;
end;

function ParsecompilerOutput(M: TMemoryStream; Verbose: boolean): string;
type
  TParseCompilerOutputState = (cosBeginOfLine, cosSearchColon, cosParseNumber, cosOther);

var
  presult: pchar;
  state: TParseCompilerOutputState;
  ch: char;
  eolchar: char;
begin
  m.Seek(0, soBeginning);
  setlength(Result,M.Size);

  if verbose then
    begin
      m.Read(Result[1],M.Size);
      Exit;
    end;

  presult := @Result[1];
  eolchar := RightStr(LineEnding,1)[1];
  m.Seek(0,soBeginning);
  state := cosBeginOfLine;
  while m.Position<m.Size do
    begin
      ch := char(m.ReadByte);
      case state of
        cosBeginOfLine:
          begin
            if ch='(' then
              state := cosParseNumber
            else if ch=' ' then
              begin
                presult^ := ch;
                inc(presult);
              end
            else
              begin
                presult^ := ch;
                inc(presult);
                state := cosSearchColon;
              end;
          end;
        cosParseNumber:
          begin
            if ch=')' then
              begin
              state := cosOther;
              // Omit the space behind the number
              ch := char(m.ReadByte);
              assert(ch=' ');
              end;
          end;
        cosOther:
          begin
            presult^ := ch;
            inc(presult);
            if ch=eolchar then
              state := cosBeginOfLine;
          end;
        cosSearchColon:
          begin
            presult^ := ch;
            inc(presult);
            if (ch=':') or (ch=eolchar) then
              state := cosBeginOfLine;
          end;
      end;
    end;
  setlength(Result,presult-@result[1]);
end;

Function QuoteXML(S : String) : string;

  Procedure W(Var J : Integer; Var R : String; T : String);
  Var
    I: integer;
  begin
    If J+Length(T)>Length(R) then
      SetLength(R,J+Length(T));
    For I:=1 to Length(t) do
      begin
      R[J]:=T[i];
      If I<Length(T) then
        Inc(J);
      end;
  end;

const
  QuotStr = '&quot;';
  AmpStr = '&amp;';
  ltStr = '&lt;';
  gtStr = '&gt;';
Var
  I,J : Integer;
begin
  SetLength(Result,Length(S));
  J:=0;
  For I:=1 to Length(S) do
    begin
    Inc(J);
    case S[i] of
      '"': W(j,Result,QuotStr);
      '&': W(J,Result,AmpStr);
      '<': W(J,Result,ltStr);
      '>': W(J,Result,gtStr);
      // Escape whitespace using CharRefs to be consistent with W3 spec X 3.3.3
       #9: w(J,Result,'&#x9;');
{      #10: wrtStr('&#xA;');
       #13: wrtStr('&#xD;');}
    else
      Result[J]:=S[i];
    end;
    If (J=Length(Result)) and (I<Length(S)) then
      SetLength(Result,J+Length(S)-I);
    end;
  If J<>Length(Result) then
    SetLength(Result,J);
end;


function maybequoted(const s:string):string;
const
  {$IF DEFINED(MSWINDOWS) OR DEFINED(AMIGA) OR DEFINED(MORPHOS) OR DEFINED(AROS)}
    FORBIDDEN_CHARS = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                       '{', '}', '''', '`', '~'];
  {$ELSE}
    FORBIDDEN_CHARS = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                       '{', '}', '''', ':', '\', '`', '~'];
  {$ENDIF}
var
  s1 : string;
  i  : integer;
  quoted : boolean;
begin
  quoted:=false;
  s1:='"';
  for i:=1 to length(s) do
   begin
     case s[i] of
       '"' :
         begin
           quoted:=true;
           s1:=s1+'\"';
         end;
       ' ',
       #128..#255 :
         begin
           quoted:=true;
           s1:=s1+s[i];
         end;
       else begin
         if s[i] in FORBIDDEN_CHARS then
           quoted:=True;
         s1:=s1+s[i];
       end;
     end;
   end;
  if quoted then
    maybequoted:=s1+'"'
  else
    maybequoted:=s;
end;


procedure ReadIniFile(Const AFileName: String;L:TStrings);
Var
  F : TFileStream;
  Line : String;
  I,P,PC : Integer;
begin
  // On some file systems and when using a large number of parallel make
  // processes, the lock from the creation of the ini file may not yet
  // have been released even though the file has been closed already
  F:=TFileStream.Create(AFileName,fmOpenRead or fmShareDenyNone);
  Try
    L.LoadFromStream(F);
    // Fix lines.
    For I:=L.Count-1 downto 0 do
      begin
        Line:=L[I];
        P:=Pos('=',Line);
        PC:=Pos(';',Line);  // Comment line.
        If (P=0) or ((PC<>0) and (PC<P)) then
          L.Delete(I)
        else
          L[i]:=Trim(System.Copy(Line,1,P-1)+'='+Trim(System.Copy(Line,P+1,Length(Line)-P)));
      end;
  Finally
    F.Free;
  end;
end;


// Callback for Sysutils getapplicationname.
Function GetFPMakeName : String;

begin
  Result:='fpmake';
end;


Function CurrentOS : String;

begin
  Result:=OSToString(Defaults.OS);
end;

Function CurrentCPU : String;

begin
  Result:=CPUToString(Defaults.CPU);
end;

Function OSToString(OS: TOS) : String;

begin
  Result:=LowerCase(GetenumName(TypeInfo(TOS),Ord(OS)));
end;

Function OSesToString(OSes: TOSes) : String;

var
  i: TOS;
  Tmp: String;
  First: boolean;

begin
  { can't use SetToString, there are more than 32 OSes }
  First:=true;
  Result:='';
  for i in OSes do
    begin
      if not First then
        Result:=Result+','
      else
        First:=false;
      Str(i,Tmp);
      Result:=Result+Tmp;
    end;
  Result:=LowerCase(Result);
end;

Function CPUToString(CPU: TCPU) : String;

begin
  Result:=LowerCase(GetenumName(TypeInfo(TCPU),Ord(CPU)));
end;

Function CPUSToString(CPUS: TCPUS) : String;

begin
  Result:=LowerCase(SetToString(PTypeInfo(TypeInfo(TCPUS)),Integer(CPUS),False));
end;

Function StringToOS(const S : String) : TOS;

Var
  I : Integer;

begin
  I:=GetEnumValue(TypeInfo(TOS),S);
  if (I=-1) then
    Raise EInstallerError.CreateFmt(SErrInvalidOS,[S]);
  Result:=TOS(I);
end;

(*
Function StringToOSes(const S : String) : TOSes;

begin
  Result:=TOSes(StringToSet(PTypeInfo(TypeInfo(TOSes)),S));
end;
*)

function IsDifferentFromBuild(ACpu: TCPU; AOs: TOs): boolean;
begin
  result := (AOs<>Defaults.BuildOS) or (ACpu<>Defaults.BuildCPU);
end;

Function StringToCPU(const S : String) : TCPU;

Var
  I : Integer;

begin
  I:=GetEnumValue(TypeInfo(TCPU),S);
  if (I=-1) then
    Raise EInstallerError.CreateFmt(SErrInvalidCPU,[S]);
  Result:=TCPU(I);
end;

Function StringToCPUS(const S : String) : TCPUS;

begin
  Result:=TCPUS(StringToSet(PTypeInfo(TypeInfo(TCPUS)),S));
end;

Function ModeToString(Mode: TCompilerMode) : String;

begin
  Result:=LowerCase(GetenumName(TypeInfo(TCompilerMode),Ord(Mode)));
  Delete(Result,1,2);
end;

Function StringToMode(const S : String) : TCompilerMode;

Var
  I : Integer;

begin
  I:=GetEnumValue(TypeInfo(TCompilerMode),S);
  if (I=-1) then
    Raise EInstallerError.CreateFmt(SErrInvalidMode,[S]);
  Result:=TCompilerMode(I);
end;


Function MakeTargetString(CPU : TCPU;OS: TOS;ALimit83: boolean) : String;

begin
  if ALimit83 then
    Result := OSToString(OS)
  else
    Result:=CPUToString(CPU)+'-'+OSToString(OS);
end;

Function MakeTargetString(CPU : TCPU;OS: TOS) : String;

begin
  Result := MakeTargetString (CPU, OS,
           (Defaults.BuildOS in AllLimit83fsOses) or (OS in AllLimit83fsOses));
end;

function MakeZipSuffix(CPU : TCPU;OS: TOS;ALimit83: boolean) : String;

begin
  case OS of
    go32v2: result := 'dos';
    watcom: result := 'wat';
    os2:    result := 'os2';
    emx:    result := 'emx';
    osNone:
      begin
        if ALimit83 then
          result := 'src'
        else
          result := '.source'
      end
  else
    result := '.' + MakeTargetString(CPU, OS, ALimit83);
  end;
end;

function MakeZipSuffix(CPU : TCPU;OS: TOS) : String;

begin
  Result := MakeZipSuffix (CPU, OS,
           (Defaults.BuildOS in AllLimit83fsOses) or (OS in AllLimit83fsOses));
end;

Procedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TOS);

Var
  P : integer;

begin
  P:=Pos('-',S);
  If (P=0) then
    Raise EInstallerError.CreateFmt(SErrInvalidTarget,[S]);
  CPU:=StringToCPU(Copy(S,1,P-1));
  OS:=StringToOs(Copy(S,P+1,Length(S)-P));
end;


Procedure ResolveDependencies(L : TDependencies; P : TNamedCollection);
Var
  I,J : Integer;
  C : TDependency;
begin
  If Assigned(L) then
    For I:=0 to L.Count-1 do
      begin
        C:=L[i];
        if C.DependencyType in [depPackage,depUnit] then
          begin
            J:=P.IndexOfName(C.Value);
            If J<>-1 then
              C.Target:=P.Items[J];
          end;
      end;
end;


function AddConditionalStrings(APackage: TPackage; Dest : TStrings; Src : TConditionalStrings;ACPU:TCPU;AOS:TOS; Const APrefix : String='') : Integer ;
Var
  I : Integer;
  C : TConditionalString;
  D : TPackageDictionary;
  S : String;
begin
  Result:=0;
  D := PackageDictionaryClass.Create(nil);
  D.MasterDictionary := APackage.Dictionary;
  try
    D.AddVariable('CPU',CPUToString(ACPU));
    D.AddVariable('OS',OSToString(AOS));
    For I:=0 to Src.Count-1 do
      begin
        C:=Src[I];
        if (ACPU in C.CPUs) and (AOS in C.OSes) then
          begin
            If (APrefix<>'') then
              S:=APrefix+C.Value
            else
              S:=C.Value;
            S := D.ReplaceStrings(s);
            if C is TConditionalDestString then
              begin
                // If a destination path is given, omit the path of the sourcefile
                if TConditionalDestString(c).DestPath='' then
                  Dest.values[S] :=  D.ReplaceStrings(IncludeTrailingPathDelimiter(TConditionalDestString(c).DestPath))+S
                else
                  Dest.values[S] :=  D.ReplaceStrings(IncludeTrailingPathDelimiter(TConditionalDestString(c).DestPath)+APrefix+ExtractFileName(C.Value));
              end
            else
              Dest.Add(S);
            Inc(Result);
          end;
      end;
  finally
    D.Free;
  end;
end;


function FileListToString(List : TStrings; const APrefix : String) : String;
Var
  I : integer;
  S : String;
begin
  Result:='';
  For I:=0 to List.Count-1 do
    begin
      If (I>0) then
        Result:=Result+' ';
      S:=APrefix+List[i];
      If (Pos(' ',S)<>0) then
        S:='"'+S+'"';
      Result:=Result+S;
    end;
end;


function FixPath (const APath : String) : String;
begin
  Result := FixPath(APath, False);
end;

function FixPath (const APath : String; AIsDir : Boolean) : String;
Var
  P : PChar;
begin
  Result:=APath;
  If (result<>'') then
    begin
      UniqueString(Result);
      P:=PChar(Result);
      While (P^<>#0) do
        begin
          If P^ in ['/','\'] then
            P^:=PathDelim;
          Inc(P);
        end;
    end;
  if AIsDir and (Result <> '') then
    Result := IncludeTrailingPathDelimiter(Result);
end;

function IsRelativePath(const APath: String): boolean;
begin
  if APath='' then
    result := true
{$ifdef unix}
  else if APath[1] in AllowDirectorySeparators then
    result := false
{$else}
  else if ExtractFileDrive(APath)<>'' then
    result := false
{$endif}
  else
    result := true;
end;

procedure ChangeDir(const APath : String);
begin
  if Not SetCurrentDir(APath) then
    Raise EInstallerError.CreateFmt(SErrChangeDirFailed,[APath]);
end;


procedure SearchFiles(AFileName, ASearchPathPrefix: string; Recursive: boolean; var List: TStrings);

  procedure AddRecursiveFiles(const SearchDir, FileMask: string; Recursive: boolean);
  var
    Info : TSearchRec;
  begin
    if assigned(Installer) then
      Installer.Log(VlDebug,Format(SDbgSearchingDir,[SearchDir]));
    if FindFirst(SearchDir+AllFilesMask,faAnyFile and faDirectory,Info)=0 then
    begin
      repeat
          if ((Info.Attr and faDirectory) = faDirectory) and (Info.Name <> '.') and (Info.Name <> '..') and (Recursive) then
            AddRecursiveFiles(SearchDir + Info.Name + PathDelim, FileMask, Recursive);
          if ((Info.Attr and faDirectory) <> faDirectory) and IsWild(Info.Name, FileMask, FileNameCaseSensitive) then
            List.Add(SearchDir + Info.Name);
      until FindNext(Info)<>0;
    end;
    sysutils.FindClose(Info);
  end;

var
  CurrDir,
  BasePath: string;
  i: integer;
begin
  if IsRelativePath(AFileName) and (ASearchPathPrefix<>'') then
    AFileName := IncludeTrailingPathDelimiter(ASearchPathPrefix) + AFileName;

  BasePath := ExtractFilePath(ExpandFileName(AFileName));

  AddRecursiveFiles(BasePath, ExtractFileName(AFileName), Recursive);

  CurrDir:=GetCurrentDir;
  for i := 0 to Pred(List.Count) do
    List[i] := ExtractRelativepath(IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(CurrDir)+ASearchPathPrefix), List[i]);
end;

Const
  WhiteSpace = [#9,#10,#13,' '];
  QuoteChars = ['''','"'];

procedure SplitCommand(const Cmd : String; out Exe, Options : String);

Var
  I : Integer;
  InQuote : Boolean;
  LastQuote : Char;
  S : String;

begin
  S:=Trim(Cmd);
  InQuote:=False;
  LastQuote:=#0;
  I:=1;
  While (I<=Length(S)) and (Inquote or not (S[I] in whiteSpace)) do
    begin
    If S[i] in QuoteChars then
      begin
      InQuote:=Not (S[i]=LastQuote);
      If InQuote then
         LastQuote:=S[i]
       else
         LastQuote:=#0;
      end;
    Inc(I);
    end;
  Exe:=Copy(S,1,I-1);
  Delete(S,1,I);
  Options:=Trim(S);
end;

procedure AddCustomFpmakeCommandlineOption(const ACommandLineOption, HelpMessage : string);
begin
  if not assigned(CustomFpmakeCommandlineOptions) then
    CustomFpmakeCommandlineOptions := TStringList.Create;
  CustomFpmakeCommandlineOptions.Values[ACommandLineOption]:=HelpMessage;
end;

function GetCustomFpmakeCommandlineOptionValue(const ACommandLineOption: string): string;
begin
  if not assigned(CustomFpMakeCommandlineValues) then
    result := ''
  else
    result := CustomFpMakeCommandlineValues.Values[ACommandLineOption];
end;

function AddProgramExtension(const ExecutableName: string; AOS : TOS): string;
begin
  if AOS in [Go32v2,Win32,Win64,Wince,OS2,EMX,Watcom] then
    Result:=ExecutableName+ExeExt
  else
    Result:=ExecutableName;
end;

function AddLibraryExtension(const LibraryName: string; AOS : TOS): string;
begin
  if AOS in [Go32v2,Win32,Win64,Wince,OS2,EMX,Watcom] then
    Result:=LibraryName+DLLExt
  else if aOS in [darwin,macosclassic,iphonesim,ios] then
    Result:=LibraryName+DyLibExt
  else if aOS = Aix then
    Result:=LibraryName+AIXSharedLibExt
  else
    Result:=LibraryName+SharedLibExt;
end;

function GetImportLibraryFilename(const UnitName: string; AOS: TOS): string;
begin
  if AOS in [go32v2,watcom] then
    Result := 'libimp'+UnitName
  else if AOS in [os2,emx] then
    Result := UnitName
  else if AOS in [netware,netwlibc,macosclassic] then
    Result := 'lib'+UnitName
  else
    Result := 'libimp'+UnitName;
  Result := Result + LibExt;
end;

Function OptionListToString(L : TStrings) : String;

var
  I : Integer;
  S : String;

begin
  Result:='';
  For I:=0 to L.Count-1 do
    begin
    If (Result<>'') then
      Result:=Result+' ';
    S:=L[I];
    If (Pos(' ',S)<>0) or (S='') then
      Result:= Result + '"'+S+'"'
    else
      Result:= Result + S;
    end;
end;

Function OptionsToStringList(S : String) : TStrings;

Var
  P : Integer;

begin
  Result:=Nil;
  If (S='') then
    Exit;
  Result:=TStringList.Create;
  Repeat
    P:=Pos(' ',S);
    If P=0 then
      P:=Length(S)+1;
    Result.Add(Copy(S,1,P-1));
    Delete(S,1,P);
    S:=Trim(S);
  Until Length(S)=0;
  If Result.Count=0 then
    FreeAndNil(Result);
end;


{$ifdef HAS_UNIT_PROCESS}
{ function GetCompilerInfo
  used both for gcc and Free Pascal compiler
  returns stdout output of Acompiler with AOptions parameters
  If ReadStdErr is True, return stderr output if stdout is empty
  If EmptyIfStdErr, return empty string if stderr output is not empty }
function GetCompilerInfo(const ACompiler,AOptions:string; ReadStdErr: boolean;EmptyIfStdErr : boolean):string;
const
  BufSize = 1024;
var
  S: TProcess;
  Buf: array [0..BufSize - 1] of char;
  ErrorBuf: array [0..BufSize - 1] of char;
  Count, ErrorCount: longint;
begin
  S:=TProcess.Create(Nil);
  S.Commandline:=ACompiler+' '+AOptions;
  S.Options:=[poUsePipes,poWaitOnExit];
  S.execute;
  Count:=s.output.read(buf,BufSize);
  if (count=0) and ReadStdErr then
    Count:=s.Stderr.read(buf,BufSize)
  else if EmptyIfStdErr then
    begin
      ErrorCount:=s.StdErr.read(ErrorBuf,BufSize);
      if (ErrorCount>0) then
        begin
          Result:='';
          S.Free;
          exit;
        end;
    end;
  S.Free;
  SetLength(Result,Count);
  Move(Buf,Result[1],Count);
end;
{$endif HAS_UNIT_PROCESS}

function GetDefaultLibGCCDir(CPU : TCPU;OS: TOS; var ErrorMessage: string): string;

  function Get4thWord(const AString: string): string;
  var p: pchar;
      spacecount: integer;
      StartWord: pchar;
  begin
    result:='';
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
      GccExecutable: string;
  begin
    result := '';
    GccExecutable := ExeSearch(AddProgramExtension('gcc', OS),Sysutils.GetEnvironmentVariable('PATH'));
    if FileExists(GccExecutable) then
      begin
{$ifdef HAS_UNIT_PROCESS}
      ExecResult:=GetCompilerInfo(GccExecutable,'-v '+GCCParams, True, True);
      libgccFilename:=Get4thWord(ExecResult);
      // Use IsRelativePath to check if the 4th word is an (absolute) path.
      // This depends on the language settings. In English the 4th word is
      // empty, if this particular gcc version does not return the libgcc-
      // filename on -v. But in other languages (e.g. Dutch) it may be another
      // word.
      if IsRelativePath(libgccFilename) then
        libgccFilename:='';
      if libgccFilename='' then
        libgccFilename:=GetCompilerInfo(GccExecutable,'--print-libgcc-file-name '+GCCParams, False, True);
      result := ExtractFileDir(libgccFilename);
{$else HAS_UNIT_PROCESS}
      ErrorMessage := SWarnNoFCLProcessSupport;
{$endif HAS_UNIT_PROCESS}
      end
    else
      ErrorMessage := SWarngccNotFound;
  end;

begin
  result := '';
  ErrorMessage:='';
  if OS in [freebsd, openbsd, dragonfly] then
    result := '/usr/local/lib'
  else if OS = netbsd then
    result := '/usr/pkg/lib'
  else if OS = linux then
    case CPU of
      i386:     result := GetGccDirArch('cpui386','-m32');
      x86_64:   result := GetGccDirArch('cpux86_64','-m64');
      powerpc:  result := GetGccDirArch('cpupowerpc','-m32');
      powerpc64:result := GetGccDirArch('cpupowerpc64','-m64');
      arm:      result := GetGccDirArch('cpuarm','-marm -march=armv2');
      aarch64:  result := GetGccDirArch('cpuaarch64','-march=aarch64 -mcmodel=large');
      m68k:     result := GetGccDirArch('cpum68k','-march=68020');
      mips:     result := GetGccDirArch('cpumips','-mips32 -EB -mabi=32');
      mipsel:   result := GetGccDirArch('cpumipsel','-mips32 -EL -mabi=32');
      riscv32:  result := GetGccDirArch('cpuriscv32','-march=rv32imafdc');
      riscv64:  result := GetGccDirArch('cpuriscv64','-march=rv64imafdc');
      sparc:    result := GetGccDirArch('cpusparc','-m32');
      sparc64:  result := GetGccDirArch('cpusparc64','-m64');
      xtensa:   result := GetGccDirArch('cpuxtensa','');
    end {case}
  else if OS = darwin then
    case CPU of
      i386:     result := GetGccDirArch('cpui386','-arch i386');
      x86_64:   result := GetGccDirArch('cpux86_64','-arch x86_64');
      powerpc:  result := GetGccDirArch('cpupowerpc','-arch ppc');
      powerpc64:result := GetGccDirArch('cpupowerpc64','-arch ppc64');
      { this target uses clang }
      aarch64:  result := ''
    end; {case}
end;

function GetPluginManager: TfpmPluginManager;
begin
  if not assigned(GPluginManager) then
    GPluginManager := TfpmPluginManager.Create;
  Result := GPluginManager;
end;

{ TPackageEnumerator }

function TPackageEnumerator.GetCurrent: TPackage;
begin
  Result:=TPackage(Inherited GetCurrent);
end;

{ TCommandEnumerator }

function TCommandEnumerator.GetCurrent: TCommand;
begin
  Result:= TCommand(Inherited GetCurrent);
end;

{ TNotifyEventEnumerator }

function TNotifyEventEnumerator.GetCurrent: TNotifyEventItem;
begin
  Result:= TNotifyEventItem( Inherited GetCurrent);
end;

{ TPackageVariantEnumerator }

function TPackageVariantEnumerator.GetCurrent: TPackageVariant;
begin
  Result:= TPackageVariant( Inherited GetCurrent);
end;

{ TTargetEnumerator }

function TTargetEnumerator.GetCurrent: TTarget;
begin
  Result:=TTarget(Inherited GetCurrent);
end;

{ TSourceEnumerator }

function TSourceEnumerator.GetCurrent: TSource;
begin
  Result:=TSource(Inherited GetCurrent);
end;


{ TResourceFiles }

procedure TResourceFiles.GetInstallFiles(AList: TStrings; const APrefixU, APrefixB: String; ACPU: TCPU; AOS: TOS);
Var
  I : Integer;
  R : TResourceFile;
begin
  For I:=0 to Count-1 do
    begin
    R:=Tobject(Items[I]) as TResourceFile;
    if (ACPU in R.CPUs) and (AOS in R.OSes) then
      AList.Add(ConcatPaths([APrefixU, R.Value]));
    end;
end;

{ TfpmResolvePackagePathsPlugin }

procedure TfpmResolvePackagePathsPlugin.ResolveUnitConfigFilenameForBasePath(
  ABuildEngine: TBuildEngine; APackage: TPackage; ABasePath: string;
  out AContinue: Boolean);
var
  IsPackageSourceLocation: boolean;
  ASubDir: string;
  AnUnitConfigFilename: string;
  PackageBaseDir: string;
begin
  if APackage.State=tsNotFound then
    // When the state is tsNotFound, the package is not part of this fpmake, and only the package-name is known.
    // In this case search for the package-name.
    // This is not right for packages where the package-name and directory name of the source-files are
    // not the same. We don't have a better option, though.
    ASubDir:=APackage.Name
  else
    ASubDir:=APackage.Directory;

  IsPackageSourceLocation:=ABuildEngine.SysFileExists(IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath)+ASubDir)+FPMakePPFile);
  if IsPackageSourceLocation then
    begin
      PackageBaseDir:=IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath)+ASubDir);
      AnUnitConfigFileName:=PackageBaseDir+APackage.GetUnitConfigOutputFilename(Defaults.CPU,Defaults.OS);
      PackageBaseDir:=IncludeTrailingPathDelimiter(PackageBaseDir+APackage.GetUnitsOutputDir(defaults.CPU, Defaults.OS));
    end
  else
    begin
      PackageBaseDir:=IncludeTrailingPathDelimiter(ABasePath);
      AnUnitConfigFileName:=IncludeTrailingPathDelimiter(ABuildEngine.GetUnitConfigFilesInstallDir(ABasePath))+APackage.Name+FpmkExt;
      PackageBaseDir:=IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath)+APackage.GetUnitsOutputDir(Defaults.CPU, Defaults.OS))+APackage.Name;
    end;

  if (PackageBaseDir<>'') and ABuildEngine.SysDirectoryExists(PackageBaseDir) then
    begin
      AnUnitConfigFilename:=APackage.Dictionary.ReplaceStrings(AnUnitConfigFilename);
      if ABuildEngine.SysFileExists(AnUnitConfigFilename) then
        APackage.UnitConfigFileName:=AnUnitConfigFilename
      else
        begin
          // To avoid that directories which do not contain installed packages
          // check that there is an unit-configfile, or Packages.fpc file. (The
          // latter to detect packages compiled using old style Makefile's, like
          // the rtl). These directories which are not packages may exist in
          // Lazarus-source-repositories.
          if not ABuildEngine.SysFileExists(ConcatPaths([PackageBaseDir, 'Package.fpc'])) then
            begin
              if IsPackageSourceLocation then
                begin
                // There is no way that I know of to confirm that a directory
                // contains the compiled sources of the rtl. So we need a
                // workaround. This test is skipped in case of the rtl.
                if APackage.Name<>'rtl' then
                  begin
                  Installer.Log(vlInfo,Format(SInfoSrcDirectoryNoPkg,[PackageBaseDir, APackage.Name]));
                  Exit;
                  end;
                end
              else
                begin
                Installer.Log(vlInfo,Format(SInfoDirectoryNoPackage,[PackageBaseDir, APackage.Name]));
                Exit;
                end;
            end;
        end;

      APackage.UnitDir:=PackageBaseDir;
      AContinue := False;

      if IsPackageSourceLocation then
        // Set the state to tsNoCompile and not tsCompiled. Because packages
        // in the tsCompiled state trigger a rebuild of packages that depend
        // on it.
        APackage.FTargetState:=tsNoCompile
      else if not (APackage.FTargetState in [tsCompiled, tsNoCompile]) then
        APackage.FTargetState:=tsInstalled;
    end
  else
    AContinue := True;
end;

procedure TfpmResolvePackagePathsPlugin.BeforeResolvePackagePath(ABuildEngine: TBuildEngine;
  APackage: TPackage; out AContinue: Boolean);
begin
  if (APackage.State in [tsCompiled, tsNoCompile, tsInstalled]) then
    ResolveUnitConfigFilenameForBasePath(ABuildEngine, APackage, ABuildEngine.StartDir, AContinue)
  else
    AContinue := True;
end;

procedure TfpmResolvePackagePathsPlugin.ResolvePackagePath(ABuildEngine: TBuildEngine;
  APackage: TPackage; SearchPath: string; out AContinue: Boolean);
begin
  ResolveUnitConfigFilenameForBasePath(ABuildEngine, APackage, SearchPath, AContinue)
end;

{ TfpmPlugin }

function TfpmPlugin.GetName: string;
begin
  Result := ClassName;
end;

procedure TfpmPlugin.BeforeResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage;
  out AContinue: Boolean);
begin
  AContinue := True;
end;

procedure TfpmPlugin.ResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage;
  SearchDirectory: string; out AContinue: Boolean);
begin
  AContinue := True;
end;

procedure TfpmPlugin.AfterResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage;
  out AContinue: Boolean);
begin
  AContinue := True;
end;

{ TfpmPluginManager }

destructor TfpmPluginManager.Destroy;
var
  i: Integer;
begin
  for i := 0 to High(FPlugins) do
    FPlugins[i].Free;
  inherited Destroy;
end;

procedure TfpmPluginManager.RegisterPlugin(APlugin: TfpmPluginClass);
begin
  SetLength(FPlugins, Length(FPlugins)+1);
  FPlugins[high(FPlugins)] := APlugin.Create;
end;

procedure TfpmPluginManager.BeforeResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage;
  out AContinue: Boolean);
var
  i: Integer;
begin
  for i := 0 to high(FPlugins) do
    begin
      FPlugins[i].BeforeResolvePackagePath(ABuildEngine, APackage, AContinue);
      if not AContinue then
        Exit;
    end;
end;

procedure TfpmPluginManager.ResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage;
  SearchPath: string; out AContinue: Boolean);
var
  i: Integer;
begin
  for i := 0 to high(FPlugins) do
    begin
      FPlugins[i].ResolvePackagePath(ABuildEngine, APackage, SearchPath, AContinue);
      if not AContinue then
        Exit;
    end;
end;

procedure TfpmPluginManager.AfterResolvePackagePath(ABuildEngine: TBuildEngine; APackage: TPackage;
  out AContinue: Boolean);
var
  i: Integer;
begin
  for i := 0 to high(FPlugins) do
    begin
      FPlugins[i].AfterResolvePackagePath(ABuildEngine, APackage, AContinue);
      if not AContinue then
        Exit;
    end;
end;

constructor TPackageVariant.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FTargets := TTargets.Create(TTarget);
  FOptions := TStringList.Create;
  FIncludePath:=TConditionalStrings.Create(TConditionalString);
  FSourcePath:=TConditionalStrings.Create(TConditionalString);
end;

destructor TPackageVariant.Destroy;
begin
  FOptions.Free;
  FTargets.Free;
  FIncludePath.Free;
  FSourcePath.Free;
  inherited Destroy;
end;

{ TPackageVariants }

procedure TPackageVariants.SetDefaultPackageVariantName(AValue: string);
begin
  if FDefaultPackageVariantName=AValue then Exit;
  if not assigned(ItemByName(avalue)) then
    raise exception.CreateFmt(SErrPackVarNotExist,[AValue]);
  FDefaultPackageVariantName:=AValue;
end;

function TPackageVariants.GetActivePackageVariant: TPackageVariant;
begin
  result := ItemByName(ActivePackageVariantName) as TPackageVariant;
end;

function TPackageVariants.GetDefaultPackageVariant: TPackageVariant;
begin
  result := ItemByName(DefaultPackageVariantName) as TPackageVariant;
end;

procedure TPackageVariants.SetActivePackageVariantName(AValue: string);
begin
  if FActivePackageVariantName=AValue then Exit;
  if not assigned(ItemByName(avalue)) then
    raise exception.CreateFmt(SErrPackVarNotExist,[AValue]);
  FActivePackageVariantName:=AValue;
end;

function TPackageVariants.Add(AName: String): TPackageVariant;
begin
  result := self.add as TPackageVariant;
  result.Name := AName;
  if FDefaultPackageVariantName='' then
    FDefaultPackageVariantName:=AName;
end;

function TPackageVariants.GetEnumerator: TPackageVariantEnumerator;
begin
  Result:= TPackageVariantEnumerator.Create(Self);
end;


{ TConditionalDestStrings }

function TConditionalDestStrings.GetConditionalString(Index : Integer): TConditionalDestString;
begin
  Result:=TConditionalDestString(Items[Index]);
end;

procedure TConditionalDestStrings.SetConditionalString(Index : Integer; AValue: TConditionalDestString);
begin
  Items[Index]:=AValue;
end;

constructor TConditionalDestStrings.Create(AClass: TConditionalDestStringClass);
begin
  inherited Create(AClass);
end;

function TConditionalDestStrings.Add(const Value: String; ADestPath: String): TConditionalDestString;
begin
  Result:=Add(Value, AllCPUs, AllOSes, ADestPath);
end;

function TConditionalDestStrings.Add(const Value: String; const OSes: TOSes; ADestPath: String): TConditionalDestString;
begin
  Result:=Add(Value, AllCPUs, OSes, ADestPath);
end;

{$ifdef cpu_only_overloads}
Function TConditionalDestStrings.Add(Const Value : String;const CPUs:TCPUs; ADestPath: String) : TConditionalDestString;inline;
begin
  Result:=Add(Value, CPUs, AllOSes, ADestPath);
end;
{$endif cpu_only_overloads}


function TConditionalDestStrings.Add(const Value: String; const CPUs: TCPUs; const OSes: TOSes; ADestPath: String): TConditionalDestString;
var
  ACondString: TConditionalDestString;
begin
  ACondString := inherited Add(Value,CPUs,OSes) as TConditionalDestString;
  ACondString.DestPath:=ADestPath;
  Result:=ACondString;
end;

{ TPackageDictionary }

function TPackageDictionary.GetValue(const AName, Args: String): String;
Var
  O : TObject;
  I : Integer;
begin
  I:=Flist.IndexOf(AName);
  If (I=-1) then
    begin
      if assigned(MasterDictionary) then
        result := MasterDictionary.GetValue(AName,Args)
      else
        result := GlobalDictionary.GetValue(AName,Args);
      Exit;
    end;
  O:=Flist.Objects[I];
  If O is TValueItem then
    Result:=TValueItem(O).FValue
  else
    Result:=TFunctionItem(O).FFunc(AName,Args);
end;

{$ifndef NO_THREADING}

{ TCompileWorkerThread }

constructor TCompileWorkerThread.Create(ABuildEngine: TBuildEngine; NotifyMainThreadEvent: PRTLEvent);
begin
  inherited Create(true);
  FNotifyStartTask := RTLEventCreate;
  FBuildEngine := ABuildEngine;
  FNotifyMainThreadEvent:=NotifyMainThreadEvent;
  Start;
end;

destructor TCompileWorkerThread.Destroy;
begin
  RTLeventdestroy(FNotifyStartTask);
  inherited Destroy;
end;

procedure TCompileWorkerThread.execute;
begin
  while not Terminated do
    begin
    { Make sure all of our results are committed before we set (F)Done to true.
      While RTLeventSetEvent implies a barrier, once the main thread is notified
      it will walk over all threads and look for those that have Done=true -> it
      can look at a thread between that thread setting FDone to true and it
      calling RTLEventSetEvent }
    WriteBarrier;
    FDone:=true;
    RTLeventSetEvent(FNotifyMainThreadEvent);
    RTLeventWaitFor(FNotifyStartTask,500);
    if not FDone then
      begin
      { synchronise with ReadWriteBarrier in mainthread for same reason as above }
      ReadWriteBarrier;
      FBuildEngine.log(vlInfo,'Compiling: '+APackage.Name);
      FCompilationOK:=false;
      try
        FBuildEngine.Compile(APackage);
        FCompilationOK:=true;
      except
        on E: Exception do
          FErrorMessage := E.Message;
      end;
      end;
    end;
end;

{$endif NO_THREADING}

{****************************************************************************
                           TUnsortedDuplicatesStringList
****************************************************************************}

function TUnsortedDuplicatesStringList.Add(const S: string): Integer;

begin
  result := IndexOf(S);
  If result > -1 then
    Case DUplicates of
      DupIgnore : Exit;
      DupError : Error(SDuplicateString,0)
    end;
  inherited Add(S);
end;

{****************************************************************************
                                TNamedItem
****************************************************************************}

procedure TNamedItem.SetName(const AValue: String);

begin
  if FName=AValue then exit;
  With TNamedCollection(Collection) do
    If UniqueNames then
      If (IndexOfName(AVAlue)<>-1) then
        Raise ECollectionError.CreateFmt(SErrNameExists,[AValue]);
  FName:=AValue;
end;


{****************************************************************************
                                TNamedCollection
****************************************************************************}

function TNamedCollection.GetItem(Index: Integer): TNamedItem;
begin
  result := TNamedItem(inherited getItem(index));
end;

procedure TNamedCollection.SetItem(Index: Integer; AValue: TNamedItem);
begin
  inherited SetItem(Index, AValue);
end;

function TNamedCollection.IndexOfName(const AName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(TNamedItem(Items[Result]).FName,AName)<>0) do
    Dec(Result);
end;

function TNamedCollection.ItemByName(const AName: String): TNamedItem;

Var
  I : Integer;

begin
  I:=IndexOfName(AName);
  If (I=-1) Then
    Raise ECollectionError.CreateFmt(SErrNoSuchName,[AName]);
  Result:=TNamedItem(Items[i]);
end;


{****************************************************************************
                             TNamedItemList
****************************************************************************}

function TNamedItemList.GetNamedItem(Index : Integer): TNamedItem;
begin
  Result:=TNamedItem(Items[Index]);
end;


procedure TNamedItemList.SetNamedItem(Index : Integer; const AValue: TNamedItem);
begin
  Items[Index]:=AValue;
end;


function TNamedItemList.IndexOfName(const AName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetNamedItem(Result).Name,AName)<>0) do
    Dec(Result);
end;


function TNamedItemList.ItemByName(const ANAme: String): TNamedItem;
Var
  I : Integer;
begin
  I:=IndexOfName(AName);
  If (I=-1) Then
    Raise ECollectionError.CreateFmt(SErrNoSuchName,[AName]);
  Result:=TNamedItem(Items[i]);
end;


{****************************************************************************
                                TTargets
****************************************************************************}

function TTargets.GetTargetItem(Index : Integer): TTarget;
begin
  Result:=TTarget(Items[Index]);
end;


function TTargets.GetTarget(const AName : String): TTarget;
begin
  Result:=TTarget(ItemByName(AName));
end;


procedure TTargets.SetTargetItem(Index : Integer; const AValue: TTarget);
begin
  Items[Index]:=AValue;
end;

function TTargets.AddFPDoc(const AUnitName, AXMLName: String): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.XML:=AXMLName;
  Result.TargetType:=ttFPDoc;
end;

function TTargets.AddUnit(const AUnitName: String): TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,AllOSes);
end;


function TTargets.AddUnit(const AUnitName: String; const OSes: TOSes): TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


function TTargets.AddUnit(const AUnitName: String; const CPUs: TCPUs;
  const OSes: TOSes): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.TargetType:=TTUnit;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
end;


function TTargets.AddImplicitUnit(const AUnitName: String; InstallUnit: boolean
  ): TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,AllOSes,InstallUnit);
end;


function TTargets.AddImplicitUnit(const AUnitName: String; const OSes: TOSes;
  InstallUnit: boolean): TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,OSes,InstallUnit);
end;


function TTargets.AddImplicitUnit(const AUnitName: String; const CPUs: TCPUs;
  InstallUnit: boolean): TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,CPUs,AllOSes,InstallUnit);
end;


function TTargets.AddImplicitUnit(const AUnitName: String; const CPUs: TCPUs;
  const OSes: TOSes; InstallUnit: boolean): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  if InstallUnit then
    Result.TargetType:=TTImplicitUnit
  else
    Result.TargetType:=TTCleanOnlyUnit;
end;


function TTargets.AddProgram(const AProgramName: String): TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,AllOSes);
end;


function TTargets.AddProgram(const AProgramName: String; const OSes: TOSes
  ): TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


function TTargets.AddProgram(const AProgramName: String; const CPUs: TCPUs;
  const OSes: TOSes): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttProgram;
end;


function TTargets.AddLibrary(const ALibraryName: String): TTarget;
begin
  Result:=AddLibrary(ALibraryName,AllCPUs,AllOSes);
end;


function TTargets.AddLibrary(const ALibraryName: String; const OSes: TOSes
  ): TTarget;
begin
  Result:=AddLibrary(ALibraryName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddLibrary(Const ALibraryName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddLibrary(ALibraryName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


function TTargets.AddLibrary(const ALibraryName: String; const CPUs: TCPUs;
  const OSes: TOSes): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=ALibraryName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttSharedLibrary;
end;


function TTargets.AddExampleUnit(const AUnitName: String): TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,AllOSes);
end;


function TTargets.AddExampleUnit(const AUnitName: String; const OSes: TOSes
  ): TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


function TTargets.AddExampleUnit(const AUnitName: String; const CPUs: TCPUs;
  const OSes: TOSes): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleUnit;
end;


function TTargets.AddExampleProgram(const AProgramName: String): TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,AllOSes);
end;


function TTargets.AddExampleProgram(const AProgramName: String;
  const OSes: TOSes): TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


function TTargets.AddExampleProgram(const AProgramName: String;
  const CPUs: TCPUs; const OSes: TOSes): TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleProgram;
end;

function TTargets.GetEnumerator: TTargetEnumerator;
begin
  Result:= TTargetEnumerator.Create(Self);
end;


{****************************************************************************
                                TSources
****************************************************************************}

function TSources.GetSourceItem(Index : Integer): TSource;
begin
  Result:=TSource(Items[Index]);
end;


procedure TSources.SetSourceItem(Index : Integer; const AValue: TSource);
begin
  Items[Index]:=AValue;
end;


function TSources.AddDoc (const AFiles : String) : TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FSourceType:=stDoc;
end;


function TSources.AddDoc(const AFiles: String; AInstallSourcePath: String): TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FInstallSourcePath:=AInstallSourcePath;
  Result.FSourceType:=stDoc;
end;


function TSources.AddSrc(const AFiles : String) : TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FSourceType:=stSrc;
end;


function TSources.AddExample(const AFiles : String) : TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FSourceType:=stExample;
end;

function TSources.AddExample(const AFiles: String; AInstallSourcePath: String): TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FInstallSourcePath:=AInstallSourcePath;
  Result.FSourceType:=stExample;
end;


function TSources.AddTest(const AFiles : String) : TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FSourceType:=stTest;
end;

function TSources.GetEnumerator: TSourceEnumerator;
begin
  Result:=TSourceEnumerator.Create(Self);
end;

procedure TSources.AddDocFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean; AInstallSourcePath: String);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, ASearchPathPrefix, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddDoc(List[i], AInstallSourcePath);
  List.Free;
end;


procedure TSources.AddSrcFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, ASearchPathPrefix, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddSrc(List[i]);
  List.Free;
end;


procedure TSources.AddExampleFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean; AInstallSourcePath: String);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, ASearchPathPrefix, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddExample(List[i], AInstallSourcePath);
  List.Free;
end;


procedure TSources.AddTestFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, ASearchPathPrefix, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddTest(List[i]);
  List.Free;
end;


{****************************************************************************
                             TPackage
****************************************************************************}

constructor TPackage.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FVersion:=TFPVersion.Create;
  FTargets:=TTargets.Create(TTarget);
  FSources:=TSources.Create(TSource);
  FDependencies:=TDependencies.Create(TDependency);
  FInstallFiles:=TConditionalDestStrings.Create(TConditionalDestString);
  FCleanFiles:=TConditionalStrings.Create(TConditionalString);
  FUnitPath:=TConditionalStrings.Create(TConditionalString);
  FObjectPath:=TConditionalStrings.Create(TConditionalString);
  FIncludePath:=TConditionalStrings.Create(TConditionalString);
  FSourcePath:=TConditionalStrings.Create(TConditionalString);
  FExamplePath:=TConditionalStrings.Create(TConditionalString);
  FTestPath:=TConditionalStrings.Create(TConditionalString);
  FCommands:=TCommands.Create(TCommand);
  FUnitsOutputDir:='units'+PathDelim+'$(target)'+PathDelim;
  FPackageVariants:=TFPList.Create;
  FCPUs:=AllCPUs;
  FOSes:=AllOSes;
  FInstalledChecksum:=$ffffffff;
  FFlags := TStringList.Create;
  // Implicit dependency on RTL
  FDependencies.Add('rtl');
  FSupportBuildModes:=[bmBuildUnit, bmOneByOne];
{$ifndef NO_THREADING}
  InitCriticalSection(FResolveDirsCS);
{$endif}
end;


destructor TPackage.destroy;
var
  i: integer;
begin
{$ifndef NO_THREADING}
  DoneCriticalSection(FResolveDirsCS);
{$endif}
  FreeAndNil(FDictionary);
  FreeAndNil(FDependencies);
  FreeAndNil(FInstallFiles);
  FreeAndNil(FCleanFiles);
  FreeAndNil(FIncludePath);
  FreeAndNil(FSourcePath);
  FreeAndNil(FExamplePath);
  FreeAndNil(FTestPath);
  FreeAndNil(FCommands);
  FreeAndNil(FObjectPath);
  FreeAndNil(FUnitPath);
  FreeAndNil(FSources);
  FreeAndNil(FTargets);
  FreeAndNil(FVersion);
  FreeAndNil(FOptions);
  FreeAndNil(FTransmitOptions);
  FreeAndNil(FFlags);
  FreeAndNil(FPackageVariants);
  inherited destroy;
end;

function TPackage.HaveOptions: Boolean;
begin
  Result:=(FOptions<>Nil);
end;


procedure TPackage.SetName(const AValue: String);
begin
  inherited SetName(AValue);
  // RTL should not have any dependencies
  if AValue='rtl' then
    FDependencies.Clear;
end;


Function TPackage.GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):String;
begin
  result:=FixPath(Dictionary.Substitute(FUnitsOutputDir,['CPU',CPUToString(ACPU),'OS',OSToString(AOS),'target',MakeTargetString(ACPU,AOS)]), False);
end;

function TPackage.GetUnitConfigOutputFilename(ACPU: TCPU; AOS: TOS): String;
begin
  result:=FixPath(Dictionary.Substitute(Name+'-$(target)'+FpmkExt,['CPU',CPUToString(ACPU),'OS',OSToString(AOS),'target',MakeTargetString(ACPU,AOS)]), False);
end;

procedure TPackage.InheritPackageVariantsFromDependency(ADependencyPackage: TPackage);
var
  i: integer;
  APackageVariants: TPackageVariants;
begin
  for i := 0 to ADependencyPackage.FPackageVariants.Count-1 do
    begin
      APackageVariants := TPackageVariants(ADependencyPackage.FPackageVariants[i]);
      if APackageVariants.IsInheritable then
        begin
        if not assigned(GetPackageVariantsByName(APackageVariants.Name)) then
          begin
          FPackageVariants.Add(APackageVariants);
          end;
        end;
    end;
end;

function TPackage.GetPackageVariantsByName(AName: string): TPackageVariants;
var
  i: Integer;
begin
  result := nil;
  for i := 0 to FPackageVariants.Count-1 do
    if SameText(TPackageVariants(FPackageVariants.Items[i]).Name, AName) then
      begin
      result := TPackageVariants(FPackageVariants.Items[i]);
      break;
      end;
end;

procedure TPackage.SetUnitsOutputDir(AValue: string);
begin
  if AValue<>'' then
    FUnitsOutputDir:=IncludeTrailingPathDelimiter(AValue)
  else
    FUnitsOutputDir:='';
end;

function TPackage.GetPackageUnitInstallDir(ACPU: TCPU; AOS: TOS): String;
begin
  result:=FixPath(Dictionary.Substitute(FPackageUnitInstallDir,['CPU',CPUToString(ACPU),'OS',OSToString(AOS),'target',MakeTargetString(ACPU,AOS)]), False);
end;

procedure TPackage.SetPackageUnitInstallDir(AValue: string);
begin
  if AValue<>'' then
    FPackageUnitInstallDir:=IncludeTrailingPathDelimiter(AValue)
  else
    FPackageUnitInstallDir:='';
end;


Function TPackage.GetBinOutputDir(ACPU:TCPU; AOS : TOS) : String;
begin
  Result:='bin'+PathDelim+MakeTargetString(ACPU,AOS);
end;


procedure TPackage.GetCleanFiles(List: TStrings; ACPU:TCPU; AOS : TOS);
Var
  OB,OU : String;
  I : Integer;
begin
  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(ACPU,AOS));
  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(ACPU,AOS));
  List.Add(GetUnitConfigOutputFilename(ACPU,AOS));
  List.Add(ManifestFile);
  AddConditionalStrings(Self, List,CleanFiles,ACPU,AOS);
  For I:=0 to FTargets.Count-1 do
    FTargets.TargetItems[I].GetCleanFiles(List, OU, OB, ACPU, AOS);
end;


procedure TPackage.GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU:TCPU; AOS : TOS);
Var
  OB,OU,OL : String;
  I : Integer;
  T : TTarget;
begin
  if Types=[] then
    AddConditionalStrings(Self, List,InstallFiles,ACPU,AOS)
  else
    begin
      OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaults.OS));
      OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
      For I:=0 to FTargets.Count-1 do
        begin
          T:=FTargets.TargetItems[I];
          if (T.TargetType in Types) and Installer.BuildEngine.TargetInstallOK(T, ACPU, AOS) then
            T.GetInstallFiles(List, OU, OB, ACPU, AOS);
        end;
    end;
end;


procedure TPackage.GetInstallSourceFiles(List: TStrings; SourceTypes : TSourceTypes; TargetTypes : TTargetTypes);
Var
  I : Integer;
  S : TSource;
  T : TTarget;
begin
  For I:=0 to FSources.Count-1 do
    begin
      S:=FSources.SourceItems[I];
      if (S.SourceType in SourceTypes) then
        S.GetInstallFiles(List);
    end;
  For I:=0 to FTargets.Count-1 do
    begin
      T:=FTargets.TargetItems[I];
      if (T.TargetType in TargetTypes) then
        T.GetArchiveFiles(List,Defaults.CPU,Defaults.OS);
    end;
end;


procedure TPackage.GetArchiveFiles(List: TStrings; ACPU:TCPU; AOS : TOS);
Var
  I : Integer;
begin
  // Targets only
  For I:=0 to FTargets.Count-1 do
    FTargets.TargetItems[I].GetArchiveFiles(List,ACPU,AOS);
end;

procedure TPackage.GetArchiveSourceFiles(List: TStrings);
var
  i : integer;
begin
  for i := 0 to Sources.Count-1 do
    List.Add(Sources[i].Name);
end;

Function TPackage.GetDescription : string;
Var
  FN : String;
  L : TStringList;
begin
  If (FDescription<>'') then
    Result:=FDescription
  else
    If (FDescriptionFile<>'') then
      begin
        // Always relative to binary name.
        FN:=ExtractFilePath(ParamStr(0));
        FN:=FN+FDescriptionFile;
        If FileExists(FN) then
          begin
            L:=TStringList.Create;
            Try
              L.LoadFromFile(FN);
              Result:=L.Text;
            Finally
              L.Free;
            end;
          end;
      end;
end;

function TPackage.GetDictionary: TDictionary;
begin
  if not assigned(FDictionary) then
    FDictionary:=PackageDictionaryClass.Create(Nil);
  result := FDictionary;
end;


Function TPackage.GetVersion : string;
begin
  result:=FVersion.AsString;
end;

procedure TPackage.SetOptions(const AValue: TStrings);
begin
  If (AValue=Nil) or (AValue.Count=0) then
    FreeAndNil(Foptions)
  else
    Options.Assign(AValue);
end;

procedure TPackage.SetTransmitOptions(AValue: TStrings);
begin
  If (AValue=Nil) or (AValue.Count=0) then
    FreeAndNil(FTransmitOptions)
  else
    TransmitOptions.Assign(AValue);
end;

Procedure TPackage.SetVersion(const V : string);
begin
  FVersion.AsString:=V;
end;


Function TPackage.GetFileName : string;
begin
  If (FFileName<>'') then
    Result:=FFileName
  else
    if not FVersion.Empty and not
      ((Defaults.OS in AllLimit83fsOses) or (Defaults.BuildOS in AllLimit83fsOses)) then
      Result := Name + '-' + FVersion.AsString
    else
      Result := ShortName;
end;


Function TPackage.GetShortName : string;
begin
  if FShortName<>'' then
    result := FShortName
  else
    result := Name;
end;


function TPackage.GetOptions: TStrings;
begin
  If (FOptions=Nil) then
    FOptions:=TStringList.Create;
  Result:=FOptions;
end;

function TPackage.GetTransmitOptions: TStrings;
begin
  If (FTransmitOptions=Nil) then
    FTransmitOptions:=TStringList.Create;
  Result:=FTransmitOptions;
end;

Procedure TPackage.GetManifest(Manifest : TStrings);

  procedure AddOSes(const AIndent:string;AOSes:TOSes);
  var
    IOS : TOS;
  begin
    if (AOSes=AllOSes) then
      exit;
    Manifest.Add(AIndent+'<oses>');
    for IOS:=low(TOSes) to high(TOSes) do
      if IOS in AOSes then
        Manifest.Add(Format(AIndent+' <os name="%s"/>',[OSToString(IOS)]));
    Manifest.Add(AIndent+'</oses>');
  end;

  procedure AddCPUs(const AIndent:string;ACPUs:TCPUs);
  var
    ICPU : TCPU;
  begin
    if (ACPUs=AllCPUs) then
      exit;
    Manifest.Add(AIndent+'<cpus>');
    for ICPU:=low(TCPUs) to high(TCPUs) do
      if ICPU in ACPUs then
        Manifest.Add(Format(AIndent+' <cpu name="%s"/>',[CPUToString(ICPU)]));
    Manifest.Add(AIndent+'</cpus>');
  end;

  function GetXMLVersionString(sMajor, sMinor, sMicro, sBuild: integer): string;
  begin
    Result := '<version';
    if sMajor <> -1 then
      Result := Result + ' major="' + IntToStr(sMajor) + '"';
    if sMinor <> -1 then
      Result := Result + ' minor="' + IntToStr(sMinor) + '"';
    if sMicro <> -1 then
      Result := Result + ' micro="' + IntToStr(sMicro) + '"';
    if sBuild <> -1 then
      Result := Result + ' build="' + IntToStr(sBuild) + '"';

    Result := Result + '/>';
  end;

Var
  S : String;
  i : Integer;
  D : TDependency;
begin
  With Manifest do
    begin
    Add(Format('<package name="%s">',[QuoteXml(Name)]));

    Add(' ' + GetXMLVersionString(FVersion.Major,FVersion.Minor,FVersion.Micro,FVersion.Build));
    AddOSes(' ',OSes);
    AddCPUs(' ',CPUs);
    Add(Format(' <filename>%s</filename>',[QuoteXml(FileName + MakeZipSuffix(cpuNone, osNone) + ZipExt)]));
    Add(Format(' <author>%s</author>',[QuoteXml(Author)]));
    Add(Format(' <license>%s</license>',[QuoteXml(License)]));
    if HomepageURL<>'' then
      Add(Format(' <homepageurl>%s</homepageurl>',[QuoteXml(HomepageURL)]));
    if DownloadURL<>'' then
      Add(Format(' <downloadurl>%s</downloadurl>',[QuoteXml(DownloadURL)]));
    Add(Format(' <email>%s</email>',[QuoteXMl(Email)]));
    S:=Description;
    If (S<>'') then
      Add(Format(' <description>%s</description>',[QuoteXML(S)]));
    If (Dependencies.Count>0) then
      begin
        Add(' <dependencies>');
        for I:=0 to Dependencies.Count-1 do
          begin
            D:=Dependencies[i];
            Add('  <dependency>');
            Add(Format('   <package packagename="%s"/>',[QuoteXML(D.Value)]));
            if not D.FVersion.Empty then
              Add('   ' + GetXMLVersionString(D.FVersion.Major,D.FVersion.Minor,D.FVersion.Micro,D.FVersion.Build));
            AddOSes('   ',D.OSes);
            AddCPUs('   ',D.CPUs);
            Add('  </dependency>');
          end;
        Add(' </dependencies>');
      end;
    Add('</package>');
    end;
end;

Procedure TPackage.ListPackage(PkgList : TStrings);

  function GetArchiveName (const APackage: TPackage; ALimit83: boolean): string;
  begin
{ Special hack to allow both long and short source files being recognized }
    if ALimit83 and (Defaults.ZipPrefix = 'units-') then
      result := 'u'
    else
      result := Defaults.ZipPrefix;
    if ALimit83 then
      result := result + APackage.ShortName
    else
      result := result + APackage.Name;
    result := result + MakeZipSuffix(Defaults.CPU, Defaults.OS, ALimit83);
  end;

Var
  S : String;
begin
{  if OSes = AllOSes then
    Exit;}
  if ((OSes = AllOSes) or (Defaults.OS in OSes)) and
         ((CPUs = AllCPUs) or (Defaults.CPU in CPUs)) or
                       (Defaults.OS = osNone) and (Defaults.CPU = cpuNone) then
    begin
      if Defaults.OS = osNone then
        PkgList.Add (Format ('# Source %d', [Succ (PkgList.Count div 2)]))
      else {if OSes <> AllOSes then}
        PkgList.Add (Format ('# ' + OSToString(Defaults.OS) + ' %d', [Succ (PkgList.Count div 2)]));
      S := 'package=' + GetArchiveName (Self, false) + ArchiveExtension;
      if ((ShortName <> Name) or (Defaults.ZipPrefix = 'units-')) and
             ((Defaults.OS in AllLimit83fsOSes) or (Defaults.OS = osNone)) then
        S := S + '[' + GetArchiveName (Self, true) + ArchiveExtension + ']';
      S := S + ',' + Description;
      PkgList.Add(S);
    end;
end;

procedure TBuildEngine.GetDocProject(Proj: TStrings; P : TPackage; aIndent: string);

  Procedure AddLn(S : String);

  begin
     Proj.Add(aIndent+S);
  end;

  Procedure AddLn(Fmt : String; Args : array of const);

  begin
    Proj.Add(aIndent+Fmt,Args);
  end;

Var
  T : TTarget;
  S, O, FN : String;
  SL : TStringList;
  L : TUnsortedDuplicatesStringList;
  I : Integer;
  iCPU : TCPU;
  iOS : TOS;

begin
  GPathPrefix:=P.Directory;
  AddPackageMacrosToDictionary(P,P.Dictionary);
  // First target OS
  ResolveFileNames(P,Defaults.CPU,Defaults.OS,False,True);
  // Then other OSes
  for ICPU:=Low(TCPU) to high(TCPU) do
    for IOS:=Low(TOS) to high(TOS) do
       if (IOS<>Defaults.OS) or (iCPU<>Defaults.CPU) then
         if OSCPUSupported[IOS,ICPU] then
            ResolveFileNames(P,ICPU,IOS,false);
  AddLn('<package name="%s" output="" content="%s.xct">',[quotexml(P.Name),quotexml(P.Name)]);
  Addln('  <units>');
  SL:=TStringList.Create;
  For T in P.Targets do
    if (T.TargetType in [ttUnit,ttImplicitUnit]) and (T.TargetSourceFileName<>'') then
      begin
      SL.Clear;
      // Writeln(T.Name,' -> ',T.TargetSourceFileName);
      FN:=AddPathPrefix(P,T.TargetSourceFileName);
      SL.Add('-d'+CPUToString(Defaults.CPU));
      SL.Add('-d'+OSToString(Defaults.OS));
      if Defaults.OS in AllUnixOSes then
        SL.Add('-dUNIX');
      SL.Add('-M'+ModeToString(T.Mode));
      // Include Path
      L:=TUnsortedDuplicatesStringList.Create;
      L.Duplicates:=dupIgnore;
      AddDependencyPaths(L,depInclude,T);
      AddConditionalStrings(P, L,P.IncludePath,Defaults.CPU,Defaults.OS);
      AddConditionalStrings(P, L,T.IncludePath,Defaults.CPU,Defaults.OS);
      for i:=0 to L.Count-1 do
        SL.Add('-Fi'+AddPathPrefix(P,L[i]));
      FreeAndNil(L);
      if P.HaveOptions Then
        SL.AddStrings(P.Options);
      if T.HaveOptions then
        SL.AddStrings(T.Options);
      O:='';
      for S in SL do
        O:=O+' '+MaybeQuoted(P.Dictionary.ReplaceStrings(S));
      Delete(O,1,1);
      AddLn('    <unit file="%s" options="%s"/>',[FN,QuoteXML(O)]);
      end;
  Addln('  </units>');
  AddLn('</package>');
end;

procedure TPackage.AddPackageVariant(APackageVariant: TPackageVariants);
begin
  if not assigned(APackageVariant.FMasterPackage) then
    APackageVariant.FMasterPackage := Self;
  FPackageVariants.Add(APackageVariant);
end;

procedure TPackage.ApplyPackageVariantToCompilerOptions(ACompilerOptions: tstrings);
var
  i: integer;
  PackageVariants: TPackageVariants;
begin
  for i := 0 to FPackageVariants.Count-1 do
    begin
    PackageVariants := TPackageVariants(FPackageVariants.Items[i]);
    ACompilerOptions.AddStrings(PackageVariants.ActivePackageVariant.Options);
    end;
end;

procedure TPackage.SetDefaultPackageVariant;
var
  i,j: integer;
  PackageVariants: TPackageVariants;
begin
  for i := 0 to FPackageVariants.Count-1 do
    begin
    PackageVariants := TPackageVariants(FPackageVariants.Items[i]);
    if Installer.FPackageVariantSettings.Values[PackageVariants.Name]<>'' then
      PackageVariants.ActivePackageVariantName:= Installer.FPackageVariantSettings.Values[PackageVariants.Name]
    else
      PackageVariants.ActivePackageVariantName:= PackageVariants.DefaultPackageVariantName;
    IncludePath.AddList(PackageVariants.ActivePackageVariant.IncludePath);
    SourcePath.AddList(PackageVariants.ActivePackageVariant.SourcePath);
    Dictionary.AddVariable(PackageVariants.Name,PackageVariants.ActivePackageVariantName);
    SetUnitsOutputDir(IncludeTrailingPathDelimiter(FUnitsOutputDir)+'$('+PackageVariants.name+')');
    SetPackageUnitInstallDir(FixPath(FPackageUnitInstallDir, True)+'$('+PackageVariants.Name+')');
    // Do not add targets f the package is inherited
    if PackageVariants.MasterPackage=Self then
      for j := 0 to PackageVariants.ActivePackageVariant.Targets.count -1 do
        targets.add.assign(PackageVariants.ActivePackageVariant.Targets.items[j]);
    end;
end;


procedure TPackage.LoadUnitConfigFromFile(Const AFileName: String);
var
  L,L2 : TStrings;
  VOS : TOS;
  VCPU : TCPU;
  i,k : Integer;
  DepChecksum : Cardinal;
  DepName : String;
  D : TDependency;
  PackageVariantsStr: string;
  PackageVarName: string;
  pv: TPackageVariants;
  AnIsInheritable: boolean;
begin
  L:=TStringList.Create;
  Try
    ReadIniFile(AFileName,L);
    With L do
      begin
        Version:=Values[KeyVersion];
        InstalledChecksum:=Cardinal(StrToInt64Def(Values[KeyChecksum],$ffffffff));
        VCPU:=StringToCPU(Values[KeyCPU]);
        VOS:=StringToOS(Values[KeyOS]);
        OSes:=[VOS];
        CPUs:=[VCPU];
        L2:=TStringList.Create;
        L2.CommaText:=Values[KeyDepends];
        for i:=0 to L2.Count-1 do
          begin
            DepName:=L2[i];
            k:=Pos('|',DepName);
            if k>0 then
              begin
                DepChecksum:=StrToInt(Copy(DepName,k+1,Length(DepName)-k));
                DepName:=Copy(DepName,1,k-1);
              end
            else
              DepChecksum:=$ffffffff;
            D:=Dependencies.Add(DepName,CPUs,OSes);
            D.RequireChecksum:=DepChecksum;
          end;
        FreeAndNil(L2);
        NeedLibC:=Upcase(Values[KeyNeedLibC])='Y';
        IsFPMakeAddIn:=Upcase(Values[KeyAddIn])='Y';
        Flags.DelimitedText:=Values[KeyFlags];
        if Values[KeyTransmit]<>'' then
          TransmitOptions.DelimitedText:=Values[KeyTransmit];
        i := 1;
        repeat
        PackageVariantsStr:=Values[KeyPackageVar+inttostr(i)];
        if PackageVariantsStr<>'' then
          begin
            k := pos(':',PackageVariantsStr);
            if k > 0 then
              begin
                PackageVarName:=copy(PackageVariantsStr,1,k-1);
                if PackageVarName[Length(PackageVarName)]='*' then
                  begin
                  SetLength(PackageVarName,Length(PackageVarName)-1);
                  AnIsInheritable:=true;
                  end
                else
                  AnIsInheritable:=false;
                PackageVariantsStr:=copy(PackageVariantsStr,k+1,length(PackageVariantsStr)-k);
                pv := Installer.AddPackageVariant(PackageVarName, AnIsInheritable);
                AddPackageVariant(pv);

                k := pos(',',PackageVariantsStr);
                while k>0 do
                  begin
                    PackageVarName:=copy(PackageVariantsStr,1,k-1);
                    PackageVariantsStr:=copy(PackageVariantsStr,k+1,length(PackageVariantsStr)-k);
                    pv.Add(PackageVarName);
                    k := pos(',',PackageVariantsStr);
                  end;
                pv.Add(PackageVariantsStr);
              end;
          end;
        inc(i);
        until PackageVariantsStr='';

      end;
  Finally
    L.Free;
  end;
end;

procedure TPackage.SaveUnitConfigToStringList(const AStringList: TStrings; ACPU: TCPU; AOS: TOS);
Var
  Deps : String;
  i,j : integer;
  D : TDependency;
  p : TPackage;
  PackageVariants : TPackageVariants;
  PackageVariantsStr: string;
  s: string;
begin
  with AStringList do
    begin
      Values[KeyName]:=Name;
      Values[KeyVersion]:=Version;
      // TODO Generate checksum based on PPUs
      InstalledChecksum:=DateTimeToFileDate(Now);
      Values[KeyChecksum]:=IntToStr(InstalledChecksum);
      Values[KeyCPU]:=CPUToString(ACPU);
      Values[KeyOS]:=OSToString(AOS);
      //Installer;
      Values[KeySourcePath]:=IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(Installer.BuildEngine.FStartDir)+Directory);
      Values[KeyFPMakeOptions]:=trim(Installer.FPMakeOptionsString);
      Deps:='';
      for i:=0 to Dependencies.Count-1 do
        begin
          D:=Dependencies[i];
          if (ACPU in D.CPUs) and (AOS in D.OSes) then
            begin
              if Deps<>'' then
                Deps:=Deps+',';
              Deps:=Deps+D.Value;
              P:=TPackage(D.Target);
              if assigned(P) and (P.InstalledChecksum<>$ffffffff) then
                Deps:=Deps+'|'+IntToStr(P.InstalledChecksum);
            end;
        end;
      Values[KeyDepends]:=Deps;
      if Flags.Count>0 then
        Values[KeyFlags]:=Flags.DelimitedText;
      if TransmitOptions.Count>0 then
        Values[KeyTransmit]:=TransmitOptions.DelimitedText;
      if NeedLibC then
        Values[KeyNeedLibC]:='Y'
      else
        Values[KeyNeedLibC]:='N';
      if IsFPMakeAddIn then
        Values[KeyAddIn]:='Y'
      else
        Values[KeyAddIn]:='N';

      s := '';
      for i := 0 to FTargets.Count-1 do
        begin
          if FTargets.TargetItems[i].IsFPMakePlugin then
            begin
              if s <> '' then
                s := s + ',';
              s := s + FTargets.TargetItems[i].Name;
            end;
        end;
      if s<>'' then
        Values[KeyPluginUnits]:=s;

      for i := 0 to FPackageVariants.Count-1 do
        begin
          PackageVariants := TPackageVariants(FPackageVariants.Items[i]);
          PackageVariantsStr:=PackageVariants.Name;
          if PackageVariants.IsInheritable then
            PackageVariantsStr:=PackageVariantsStr+'*';
          PackageVariantsStr := PackageVariantsStr +':'+PackageVariants.DefaultPackageVariantName;
          for j := 0 to PackageVariants.Count-1 do
            if not sametext(PackageVariants.Items[j].Name, PackageVariants.DefaultPackageVariantName) then
              PackageVariantsStr:=PackageVariantsStr+','+PackageVariants.Items[j].Name;
          values[KeyPackageVar+inttostr(i+1)] := PackageVariantsStr;
        end;
    end;
end;

procedure TPackage.SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;AOS:TOS);
Var
  F : TFileStream;
  L : TStringList;
begin
  F:=TFileStream.Create(AFileName,fmCreate or fmShareDenyNone);
  L:=TStringList.Create;
  try
    SaveUnitConfigToStringList(L,ACPU,AOS);
    L.SaveToStream(F);
  Finally
    L.Free;
    F.Free;
  end;
end;

procedure TPackage.EnterResolveDirsCS;
begin
{$ifndef NO_THREADING}
   EnterCriticalSection(FResolveDirsCS);
{$endif}
end;

procedure TPackage.LeaveResolveDirsCS;
begin
{$ifndef NO_THREADING}
   LeaveCriticalSection(FResolveDirsCS);
{$endif}
end;



{****************************************************************************
                              TPackages
****************************************************************************}

function TPackages.GetPackage(const AName : String): TPackage;
begin
  Result:=TPackage(ItemByName(AName))
end;

function TPackages.GetPackageItem(AIndex : Integer): TPackage;
begin
  Result:=TPackage(Items[AIndex]);
end;


procedure TPackages.SetPackageItem(AIndex : Integer; const AValue: TPackage);
begin
  Items[AIndex]:=AValue;
end;


function TPackages.AddPackage(const AName: String): TPackage;
var
  i: integer;
begin
  i := IndexOfName(AName);
  if i > -1 then
    begin
    result := PackageItems[i];
    Installer.Log(vlWarning,Format(SWarnDuplicatePackage,[AName]))
    end
  else
    begin
    Result:=Add as TPackage;
    Result.Name:=AName;
    end;
end;

function TPackages.GetEnumerator: TPackageEnumerator;
begin
  Result:= TPackageEnumerator.Create(Self);
end;


{****************************************************************************
                             TCustomDefaults
****************************************************************************}

procedure TCustomDefaults.SetCPU(const AValue: TCPU);
begin
  FCPU:=AValue;
  GlobalDictionary.AddVariable('CPU',CPUToString(FCPU));
  RecalcTarget;
end;

procedure TCustomDefaults.SetOptions(const AValue: TStrings);
begin
  If (AValue=Nil) or (AValue.Count=0) then
    FreeAndNil(Foptions)
  else
    Options.Assign(AValue)
end;


function TCustomDefaults.GetBaseInstallDir: String;
begin
  If (FBaseInstallDir<>'') then
    Result:=FBaseInstallDir
  else
    if UnixPaths then
      Result:=Prefix +'lib' + PathDelim + 'fpc' + PathDelim + FCompilerVersion + PathDelim
    else
      Result:=Prefix;
end;


function TCustomDefaults.GetBinInstallDir: String;
begin
  If (FBinInstallDir<>'') then
    Result:=FBinInstallDir
  else
    If UnixPaths then
      Result:=Prefix+'bin'
    else
      Result:=BaseInstallDir+'bin'+pathdelim+MakeTargetString(Defaults.cpu, Defaults.OS);
end;

function TCustomDefaults.GetLibInstallDir: String;
begin
  If (FLibInstallDir<>'') then
    Result:=FLibInstallDir
  else
    If UnixPaths then
      Result:=Prefix+'lib'
    else
      Result:=BaseInstallDir+'bin'+pathdelim+MakeTargetString(Defaults.cpu, Defaults.OS);
end;


function TCustomDefaults.GetCompiler: String;
begin
  If (FCompiler<>'') then
    Result:=FCompiler
  else
    Result:='fpc';
end;


function TCustomDefaults.GetDocInstallDir: String;
begin
  If (FDocInstallDir<>'') then
    Result:=FDocInstallDir
  else if (Defaults.BuildOS=freebsd) or (Defaults.BuildOS=dragonfly) then
    Result:=Prefix+'share'+PathDelim+'doc'+PathDelim+'fpc-$(CompilerVersion)'+PathDelim+'$(PackageName)'
  else If UnixPaths then
    Result:=Prefix+'share'+PathDelim+'doc'+PathDelim+'fpc-$(CompilerVersion)'+PathDelim+'$(PackageName)'
  else
    Result:=BaseInstallDir+'doc'+PathDelim+'$(PackageName)';
end;


function TCustomDefaults.GetExamplesInstallDir: String;
begin
  If (FExamplesInstallDir<>'') then
    Result:=FExamplesInstallDir
  else if (Defaults.BuildOS=freebsd) or (Defaults.BuildOS=dragonfly) then
    Result:=Prefix+'share'+PathDelim+'examples'+PathDelim+'fpc-$(CompilerVersion)'+PathDelim+'$(PackageName)'+PathDelim+'examples'
  else If UnixPaths then
    Result:=Prefix+'share'+PathDelim+'doc'+PathDelim+'fpc-$(CompilerVersion)'+PathDelim+'$(PackageName)'+PathDelim+'examples'
  else
    Result:=BaseInstallDir+'examples'+PathDelim+'$(PackageName)';
end;

function TCustomDefaults.GetOptions: TStrings;
begin
  If (FOptions=Nil) then
    FOptions:=TStringList.Create;
  Result:=FOptions;
end;


function TCustomDefaults.GetPrefix: String;
begin
  // Use ExpandFileName to support ~/ expansion
  if FPrefix<>'' then
    Result:=IncludeTrailingPathDelimiter(ExpandFileName(FPrefix))
  else
    Result:='';
end;


function TCustomDefaults.GetSearchPath: TStrings;
begin
  Result := FSearchPath;
end;


function TCustomDefaults.GetUnitInstallDir: String;
begin
  result := FUnitInstallDir;
end;


function TCustomDefaults.GetUnitConfigFilesInstallDir: String;
begin
  result := FUnitConfigFilesInstallDir;
end;


function TCustomDefaults.GetLocalUnitDir: String;
begin
  if FSearchPath.Count>0 then
    Result:=FSearchPath[0]
  else
    Result:='';
end;

function TCustomDefaults.GetFPDocOutputDir: String;
begin
  If (FFPDocOutputDir<>'') then
    Result:=FixPath(FFPDocOutputDir, True)
  else
    Result:=FixPath('.'+PathDelim+'docs', True);
end;

function TCustomDefaults.GetFPUnitSourcePath: String;
begin
  If (FFPUnitSourcePath='') or (FFPUnitSourcePath='0') then
    result := FFPUnitSourcePath
  else
    Result:=FixPath(FFPUnitSourcePath, True);
end;

function TCustomDefaults.SafeExpandFileName(const AFileName: string): string;
begin
  if AFileName<>'' then
    Result:=IncludeTrailingPathDelimiter(ExpandFileName(AFileName))
  else
    Result:='';
end;

function TCustomDefaults.GetBuildCPU: TCpu;
begin
  result := StringToCPU({$I %FPCTARGETCPU%});
end;

function TCustomDefaults.GetBuildOS: TOS;
begin
  result := StringToOS({$I %FPCTARGETOS%});
end;

function TCustomDefaults.GetBuildString: String;
begin
  result := MakeTargetString(BuildCPU, BuildOS);
end;

function TCustomDefaults.GetGlobalUnitDir: String;
begin
  if FSearchPath.Count>1 then
    Result:=FSearchPath[1]
  else
    Result:='';
end;


procedure TCustomDefaults.SetLocalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if FSearchPath.Count=0 then
    FSearchPath.Add(SafeExpandFileName(AValue))
  else
    FSearchPath[0]:=SafeExpandFileName(AValue);
end;


procedure TCustomDefaults.SetGlobalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if FSearchPath.Count<2 then
    begin
      if FSearchPath.Count<1 then
        FSearchPath.Add('');
      FSearchPath.Add(SafeExpandFileName(AValue));
    end
  else
    FSearchPath[1]:=SafeExpandFileName(AValue);
end;

procedure TCustomDefaults.IntSetBaseInstallDir(const AValue: String);
begin
  if AValue<>'' then
    FBaseInstallDir:=IncludeTrailingPathDelimiter(AValue)
  else
    FBaseInstallDir:='';
  GlobalDictionary.AddVariable('baseinstalldir',BaseInstallDir);
  GlobalDictionary.AddVariable('bininstalldir',BinInstallDir);
  GlobalDictionary.AddVariable('libinstalldir',LibInstallDir);
  BinInstallDir:='';
  LibInstallDir:='';
  ExamplesInstallDir:='';
end;


procedure TCustomDefaults.SetBaseInstallDir(const AValue: String);
begin
  // There must be a possibility to skip ExpandFileName. So that the files
  // can be written into an archive with a relative path.
  if AValue<>'' then
    // Use ExpandFileName to support ~/ expansion
    IntSetBaseInstallDir(ExpandFileName(AValue))
  else
    IntSetBaseInstallDir(AValue);
end;


procedure TCustomDefaults.SetOS(const AValue: TOS);
begin
  FOS:=AValue;
  GlobalDictionary.AddVariable('OS',OSToString(FOS));
  Recalctarget;
end;


procedure TCustomDefaults.SetPrefix(const AValue: String);
begin
  if FPrefix=AValue then exit;
  FPrefix:=AValue;
  GlobalDictionary.AddVariable('prefix',Prefix);
  GlobalDictionary.AddVariable('bininstalldir',BinInstallDir);
  GlobalDictionary.AddVariable('libinstalldir',LibInstallDir);
  BaseInstallDir:='';
end;

procedure TCustomDefaults.SetSearchPath(AValue: TStrings);
begin
  FSearchPath.Assign(AValue);
end;


procedure TCustomDefaults.SetTarget(const AValue: String);
Var
  P : Integer;
begin
  if FTarget<>AValue then
    begin
      P:=Pos('-',AValue);
      If (P<>0) then
        begin
          FOS:=StringToOS(System.Copy(Avalue,P+1,Length(AValue)-P));
          GlobalDictionary.AddVariable('OS',OSToString(FOS));
          FCPU:=StringToCPU(System.Copy(Avalue,1,P-1));
          GlobalDictionary.AddVariable('CPU',CPUToString(FCPU));
        end
      else
        FOS:=StringToOS(AValue);
      FTarget:=AValue;
      GlobalDictionary.AddVariable('target',Target);
    end;
end;

procedure TCustomDefaults.SetUnitInstallDir(const AValue: String);
begin
  if AValue<>'' then
    FUnitInstallDir:=AValue
  else
    FUnitInstallDir:='';
end;


procedure TCustomDefaults.SetUnitConfigFilesInstallDir(const AValue: String);
begin
  FUnitConfigFilesInstallDir:=AValue;
end;


procedure TCustomDefaults.SetZipPrefix(AValue: String);
begin
  if FZipPrefix=AValue then Exit;
  FZipPrefix:=AValue;
end;


procedure TCustomDefaults.RecalcTarget;
begin
  Ftarget:=MakeTargetString(FCPU,FOS);
  GlobalDictionary.AddVariable('target',Target);
end;

function TCustomDefaults.CmdLineOptions: String;
begin
  If Haveoptions then
    Result:=OptionListToString(FOptions);
end;


constructor TCustomDefaults.Create;
begin
  FSearchPath:=TStringList.Create;
  InitDefaults;
end;

destructor TCustomDefaults.Destroy;
begin
  FSearchPath.Free;
  inherited;
end;


procedure TCustomDefaults.InitDefaults;
begin
{$ifdef unix}
  UnixPaths:=True;
{$else}
  UnixPaths:=False;
{$endif}
  FNoFPCCfg:=False;
  FCPU:=cpuNone;
  FOS:=osNone;
  FUnitInstallDir:='$(baseinstalldir)units/$(target)/$(packagename)';
  FUnitConfigFilesInstallDir:='fpmkinst/$(target)';
  FBuildMode:=bmOneByOne;
  FThreadsAmount:=-1;
end;

function TCustomDefaults.HaveOptions: Boolean;
begin
  Result:=Assigned(FOptions);
end;

function TCustomDefaults.IsBuildDifferentFromTarget: boolean;
begin
  result := IsDifferentFromBuild(CPU,OS);
end;


procedure TCustomDefaults.LocalInit(const AFileName: String);
Var
  FN : String;
begin
  FN:=AFileName;
  If (FN='') then
    begin
    // Environment variable.
    FN:=SysUtils.GetEnvironmentVariable('FPMAKECFG');
    If (FN<>'') then
      If not FileExists(FN) then
        FN:='';
    // User config file fpmake.cfg
    If (FN='') then
      begin
      FN:=GetAppConfigFile(False);
      If Not FileExists(FN) then
        FN:='';
      end;
    // Global config file fpmake.cfg
    If (FN='') then
      begin
      FN:=GetAppConfigFile(True);
      If Not FileExists(FN) then
        FN:='';
      end;
    end;
  If (FN<>'') and FileExists(FN) then
    LoadFromFile(FN);
end;


procedure TCustomDefaults.CompilerDefaults;
{$ifdef HAS_UNIT_PROCESS}
var
  infoSL : TStringList;
{$endif HAS_UNIT_PROCESS}
begin
  if (CPU=cpuNone) or ((OS=osNone) and not ExplicitOSNone) or
                                                     (FCompilerVersion='') then
    begin
{$ifdef HAS_UNIT_PROCESS}
      // Detect compiler version/target from -i option
      infosl:=TStringList.Create;
      infosl.Delimiter:=' ';
      infosl.DelimitedText:=GetCompilerInfo(GetCompiler,'-iVTPTO', False, True);
      if infosl.Count<>3 then
        Raise EInstallerError.Create(SErrInvalidFPCInfo);
      if FCompilerVersion='' then
        FCompilerVersion:=infosl[0];
      if CPU=cpuNone then
        CPU:=StringToCPU(infosl[1]);
      if (OS=osNone) and not ExplicitOSNone then
        OS:=StringToOS(infosl[2]);
{$else HAS_UNIT_PROCESS}
      // Defaults taken from compiler used to build fpmake
      if CPU=cpuNone then
        CPU:=StringToCPU({$I %FPCTARGETCPU%});
      if (OS=osNone) and not ExplicitOSNone then
        OS:=StringToOS({$I %FPCTARGETOS%});
      if FCompilerVersion='' then
        FCompilerVersion:={$I %FPCVERSION%};
{$endif HAS_UNIT_PROCESS}
    end;
end;


procedure TCustomDefaults.LoadFromFile(const AFileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(AFileName,fmOpenRead or fmShareDenyNone);
  Try
    LoadFromStream(F);
  Finally
    F.Free;
  end;
end;


procedure TCustomDefaults.SaveToFile(const AFileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(AFileName,fmCreate or fmShareDenyNone);
  Try
    SaveToStream(F);
  Finally
    F.Free;
  end;
end;


procedure TCustomDefaults.SaveToStream(S : TStream);
Var
  L : TStringList;
begin
  L:=TStringList.Create;
  try
    With L do
      begin
      Values[KeyArchive]:=FArchive;
      Values[KeyCompiler]:=FCompiler;
      Values[KeyCopy]:=FCopy;
      Values[KeyMkDir]:=FMkDir;
      Values[KeyMove]:=FMove;
      Values[KeyOptions]:=CmdLineOptions;
      Values[KeyCPU]:=CPUToString(FCPU);
      Values[KeyOS]:=OSToString(FOS);
      Values[KeyMode]:=ModeToString(FMode);
      Values[KeyLocalUnitDir]:=LocalUnitDir;
      Values[KeyGlobalUnitDir]:=GlobalUnitDir;
      Values[KeyPrefix]:=FPrefix;
      Values[KeyBaseInstallDir]:=FBaseInstallDir;
      Values[KeyUnitInstallDir]:=FUnitInstallDir;
      Values[KeyBinInstallDir]:=FBinInstallDir;
      Values[KeyLibInstallDir]:=FLibInstallDir;
      Values[KeyDocInstallDir]:=FDocInstallDir;
      Values[KeyExamplesInstallDir]:=FExamplesInstallDir;
      Values[KeyRemove]:=FRemove;
      Values[KeyRemoveDir]:=FRemoveDir;
      Values[KeyRemoveTree]:=FRemoveTree;
      Values[KeyTarget]:=FTarget;
      if FNoFPCCfg then
        Values[KeyNoFPCCfg]:='Y';
      if FUseEnvironment then
        Values[KeyUseEnv]:='Y';
      if FInstallExamples then
          Values[KeyInstallExamples]:='Y';
      if FSkipCrossPrograms then
        Values[KeySkipCrossProdrams]:='Y';
      end;
    L.SaveToStream(S);
  Finally
    L.Free;
  end;
end;


procedure TCustomDefaults.LoadFromStream(S: TStream);
Var
  L : TStrings;
  Line : String;
  I,P,PC : Integer;
begin
  L:=TStringList.Create;
  Try
    L.LoadFromStream(S);
    // Fix lines.
    For I:=L.Count-1 downto 0 do
      begin
      Line:=L[I];
      P:=Pos('=',Line);
      PC:=Pos(';',Line);  // Comment line.
      If (P=0) or ((PC<>0) and (PC<P)) then
        L.Delete(I)
      else
        L[i]:=Trim(System.Copy(Line,1,P-1)+'='+Trim(System.Copy(Line,P+1,Length(Line)-P)));
      end;
    With L do
      begin
      FArchive:=Values[KeyArchive];
      FCompiler:=Values[KeyCompiler];
      FCopy:=Values[KeyCopy];
      FMkDir:=Values[KeyMkDir];
      FMove:=Values[KeyMove];
      FRemove:=Values[KeyRemove];
      FRemoveDir:=Values[KeyRemoveDir];
      FRemoveTree:=Values[KeyRemoveTree];
      Self.Options:=OptionsToStringList(Values[KeyOptions]);
      Line:=Values[KeyCPU];
      If (Line<>'') then
        FCPU:=StringToCPU(Line);
      Line:=Values[KeyOS];
      If (Line<>'') then
        FOS:=StringToOS(Line);
      Line:=Values[KeyMode];
      If (Line<>'') then
        FMode:=StringToMode(Line);
      FTarget:=Values[KeyTarget];
      LocalUnitDir:=Values[KeyLocalUnitDir];
      GlobalUnitDir:=Values[KeyGlobalUnitDir];
      FPrefix:=Values[KeyPrefix];
      FBaseInstallDir:=Values[KeyBaseInstallDir];
      FUnitInstallDir:=Values[KeyUnitInstallDir];
      FBinInstallDir:=Values[KeyBinInstallDir];
      FLibInstallDir:=Values[KeyLibInstallDir];
      FDocInstallDir:=Values[KeyDocInstallDir];
      FExamplesInstallDir:=Values[KeyExamplesInstallDir];
      FInstallExamples:=(Upcase(Values[KeyInstallExamples])='Y');
      FSkipCrossPrograms:=(Upcase(Values[KeySkipCrossProdrams])='Y');
      FNoFPCCfg:=(Upcase(Values[KeyNoFPCCfg])='Y');
      FUseEnvironment:=(Upcase(Values[KeyUseEnv])='Y');

      GlobalDictionary.AddVariable('target',Target);
      GlobalDictionary.AddVariable('baseinstalldir',BaseInstallDir);
      GlobalDictionary.AddVariable('prefix',Prefix);
      GlobalDictionary.AddVariable('bininstalldir',BinInstallDir);
      GlobalDictionary.AddVariable('libinstalldir',LibInstallDir);
      end;
  Finally
    L.Free;
  end;
end;


{****************************************************************************
                             TFPCDefaults
****************************************************************************}

procedure TFPCDefaults.CompilerDefaults;
var
  BD : String;
begin
  inherited CompilerDefaults;

  // Use the same algorithm as the compiler, see options.pas
{$ifdef Unix}
  BD:=FixPath(GetEnvironmentVariable('FPCDIR'), False);
  if BD='' then
    begin
      BD:='/usr/local/lib/fpc/'+FCompilerVersion;
      if not DirectoryExists(BD) and
         DirectoryExists('/usr/lib/fpc/'+FCompilerVersion) then
        BD:='/usr/lib/fpc/'+FCompilerVersion;
    end;
{$else unix}
  BD:=FixPath(SysUtils.GetEnvironmentVariable('FPCDIR'), False);
  if BD='' then
    begin
      BD:=ExtractFilePath(FCompiler)+'..';
      if not(DirectoryExists(BD+'/units')) and
         not(DirectoryExists(BD+'/rtl')) then
        BD:=FBaseInstallDir+'..';
    end;
{$endif unix}

  // Where to install by default
  if (FBaseInstallDir='') and (FPrefix='') then
    BaseInstallDir:=BD;
end;


{****************************************************************************
                            TCustomInstaller
****************************************************************************}

constructor TCustomInstaller.Create(AOwner: TComponent);
begin
  FPackageVariantSettings := TStringList.Create;
  FPackageVariants := TFPList.Create;
  GlobalDictionary:=DictionaryClass.Create(Nil);
  AnalyzeOptions;
  GlobalDictionary.AddVariable('baseinstalldir',Defaults.BaseInstallDir);
  GlobalDictionary.AddVariable('bininstalldir',Defaults.BinInstallDir);
  GlobalDictionary.AddVariable('libinstalldir',Defaults.LibInstallDir);
  GlobalDictionary.AddVariable('Target',Defaults.Target);
  GlobalDictionary.AddVariable('BuildString',Defaults.BuildString);
  GlobalDictionary.AddVariable('Prefix',Defaults.Prefix);
  GlobalDictionary.AddVariable('CompilerVersion',Defaults.CompilerVersion);
  FNotifyEventCollection := TNotifyEventCollection.create([neaBeforeCompile, neaAfterCompile, neaBeforeClean, neaAfterClean,
                                                           neaBeforeInstall, neaAfterInstall, neaBeforeArchive, neaAfterArchive,
                                                           neaBeforeManifest, neaAfterManifest, neaBeforePkgList, neaAfterPkgList,
                                                           neaBeforeUnInstall, neaAfterUnInstall,
                                                           neaBeforeCreateBuildEngine, neaAfterCreateBuildengine]);
  CreatePackages;
end;


destructor TCustomInstaller.Destroy;
var
  i: integer;
begin
  FreePackages;
  FreeAndNil(Defaults);
  FreeAndNil(GlobalDictionary);
  FreeAndNil(FPackageVariantSettings);
  for i := 0 to FPackageVariants.Count-1 do
    begin
    if TPackageVariants(FPackageVariants.Items[i]).Owner=Self then
      TPackageVariants(FPackageVariants.Items[i]).Free;
    end;
  FreeAndNil(FPackageVariants);
  FreeAndNil(FNotifyEventCollection);
  inherited destroy;
end;

function TCustomInstaller.GetPackages: TPackages;
begin
  result := FPackages;
end;

procedure TCustomInstaller.Log(Level: TVerboseLevel; Const Msg: String);
begin
  If Level in FLogLevels then
    begin
    Writeln(StdOut, Msg);
    Flush(StdOut);
    end;
end;


procedure TCustomInstaller.CreatePackages;
begin
  FPackages:=TPackages.Create(TPackage);
end;

procedure TCustomInstaller.FreePackages;
begin
  FreeAndNil(FPackages);
end;


procedure TCustomInstaller.CreateBuildEngine;
begin
  NotifyEventCollection.CallEvents(neaBeforeCreateBuildEngine, Self);
  FBuildEngine:=TBuildEngine.Create(Self);
//  FBuildEngine.Defaults:=Defaults;
  FBuildEngine.ListMode:=FListMode;
  FBuildEngine.FInteractive:=FInteractive;
  FBuildEngine.Verbose := (vlInfo in FLogLevels) or (vlDebug in FLogLevels);
  FBuildEngine.OnLog:=@Self.Log;
  NotifyEventCollection.CallEvents(neaAfterCreateBuildengine, Self);
end;


procedure TCustomInstaller.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TCustomInstaller.Error(const Fmt: String; const Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


Function TCustomInstaller.AddPackage(const AName: String) : TPackage;
begin
  result:=Packages.AddPackage(AName);
  AddAutoPackageVariantsToPackage(result);
end;

Function TCustomInstaller.AddPackageVariant(AName: string; AIsInheritable: boolean; AutoAddToPackage: Boolean): TPackageVariants;
begin
  result := TPackageVariants.Create(TPackageVariant);
  result.Name:=AName;
  result.FIsInheritable:=AIsInheritable;
  result.FAutoAddToPackage:=AutoAddToPackage;
  FPackageVariants.Add(result);
end;

procedure TCustomInstaller.AnalyzeOptions;

  Function CheckOption(Index : Integer;const Short,Long : String; AddToOptionString: boolean = false): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O='--'+long) or (copy(O,1,Length(Long)+3)=('--'+long+'='));
    if AddToOptionString and Result then
      FFPMakeOptionsString := FFPMakeOptionsString+' '+O;
  end;

  Function CheckBuildOptionSetValue(Index: Integer): boolean;
  var
    O : String;
    BuildModeName: string;
    P: integer;
    C: string;
    PV: TPackageVariants;
    SL: TStringList;
  begin
    O:=Paramstr(Index);
    result := O[1]='+';
    if result then
      begin
      P:=Pos('=',Paramstr(Index));
      If (P=0) then
        Error(SErrNeedArgument,[Index,O])
      else
        begin
        BuildModeName:=copy(o,2,P-2);
        Delete(O,1,P);

        c := RightStr(BuildModeName,1);
        if (c = '*') or (c = '+') then
          begin
          // Add a new package-variant
          BuildModeName := copy(BuildModeName, 1, length(BuildModeName) -1);
          PV := AddPackageVariant(BuildModeName, (C = '*'), True);
          SL := TStringList.Create;
          try
            SL.CommaText := O;
            if SL.Count=0 then
              Error(SErrExpectPkgVariant,[Index,ParamStr(Index)]);
            for P := 0 to SL.Count -1 do
              begin
              PV.Add(SL.ValueFromIndex[P]);
              end;
            FPackageVariantSettings.Values[BuildModeName] := SL.ValueFromIndex[0];
          finally
            SL.Free;
          end;
          end
        else
          begin
          // Set the value of the package-variant.
          FPackageVariantSettings.Values[BuildModeName] := O;
          end;
        end;
      end;
  end;

  Function CheckCustomOption(Index : Integer; out CustOptName: string): Boolean;
  var
    O : String;
    i : integer;
  begin
    result := false;
    CustOptName:='';
    O:=Paramstr(Index);
    if copy(O,1,2)<>'--' then
      Exit;
    i := pos('=',O);
    if i=0 then
      Exit;
    O:=copy(O,3,i-3);
    CustOptName:=O;
    Result:=CustomFpmakeCommandlineOptions.IndexOfName(O)>-1;
    if Result then FFPMakeOptionsString := FFPMakeOptionsString+' '+Paramstr(Index);
  end;


  Function CheckCommand(Index : Integer;const Short,Long : String): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O=long);
  end;

  Function OptionArg(Var Index : Integer; AddToOptionString: boolean = false) : String;
  Var
    P : Integer;
  begin
    if (Length(ParamStr(Index))>1) and (Paramstr(Index)[2]<>'-') then
      begin
      If Index<ParamCount then
        begin
        Inc(Index);
        Result:=Paramstr(Index);
        if AddToOptionString then
          FFPMakeOptionsString := FFPMakeOptionsString+' '+Result;
        end
      else
        Error(SErrNeedArgument,[Index,ParamStr(Index)]);
      end
    else If length(ParamStr(Index))>2 then
      begin
      P:=Pos('=',Paramstr(Index));
      If (P=0) then
        Error(SErrNeedArgument,[Index,ParamStr(Index)])
      else
        begin
        Result:=Paramstr(Index);
        Delete(Result,1,P);
        end;
      end;
  end;

  function SplitSpaces(var SplitString: string) : string;
  var i : integer;
  begin
    i := pos(' ',SplitString);
    if i > 0 then
      begin
        result := copy(SplitString,1,i-1);
        delete(SplitString,1,i);
      end
    else
      begin
        result := SplitString;
        SplitString:='';
      end;
  end;

  Function CheckPackageVariantOptionSetValue(Index: Integer): boolean;
  var
    O : String;
    OptionName: string;
    I,J: integer;
    P: Integer;
    PV: TPackageVariants;
  begin
    result := false;
    O:=Paramstr(Index);
    if copy(O,1,2)<>'--' then
      Exit;
    P:=Pos('=',Paramstr(Index));
    if P > 0 then
      begin
      OptionName:=copy(o,3,P-3);
      Delete(O,1,P);
      for I := 0 to FPackageVariants.Count -1 do
        begin
        PV := TObject(FPackageVariants.Items[I]) as TPackageVariants;
        for J := 0 to PV.Count-1 do
          begin
          if OptionName = 'options_'+PV.Name+'_'+PV.Items[J].Name then
            begin
            result := true;
            while O <> '' do
              (PV.Items[J] as TPackageVariant).Options.Add(SplitSpaces(O));
            end;
          end;
        end;
      end;
  end;

Var
  I : Integer;
  DefaultsFileName : string;
  OptString : string;
  CustOptName : string;
  UnitDirSet: Boolean;
  SearchPathSet: Boolean;
begin
  I:=0;
  FListMode:=False;
  UnitDirSet:=False;
  SearchPathSet:=False;
  FLogLevels:=DefaultMessages;
  While (I<ParamCount) do
    begin
    Inc(I);
    if CheckOption(I,'v','verbose') then
      FLogLevels:=AllMessages
    else if CheckOption(I,'I','interactive') then
      FInteractive:=true
    else if CheckOption(I,'d','debug') then
      FLogLevels:=AllMessages+[vlDebug]
    else if CheckCommand(I,'m','compile') then
      FRunMode:=rmCompile
    else if CheckCommand(I,'b','build') then
      FRunMode:=rmBuild
    else if CheckCommand(I,'i','install') then
      FRunMode:=rmInstall
    else if CheckCommand(I,'bi','buildinstall') then
      FRunMode:=rmBuildInstall
    else if CheckCommand(I,'zi','zipinstall') then
      FRunMode:=rmZipInstall
    else if CheckCommand(I,'c','clean') then
      FRunMode:=rmClean
    else if CheckCommand(I,'dc','distclean') then
      FRunMode:=rmDistClean
    else if CheckCommand(I,'a','archive') then
      FRunMode:=rmarchive
    else if CheckCommand(I,'M','manifest') then
      FRunMode:=rmManifest
    else if CheckCommand(I,'l','pkglist') then
      FRunMode:=rmPkgList
    else if CheckCommand(I,'u','uninstall') then
      FRunMode:=rmUnInstall
    else if CheckCommand(I,'in','info') then
      FRunMode:=rmInfo
    else if CheckCommand(I,'dp','fpdocproject') then
      FRunMode:=rmDocProject
    else if CheckOption(I,'h','help') then
      Usage('',[])
    else if Checkoption(I,'C','cpu') then
      Defaults.CPU:=StringToCPU(OptionArg(I))
    else if Checkoption(I,'O','os') then
      begin
        Defaults.OS:=StringToOS(OptionArg(I));
        Defaults.ExplicitOSNone := OptionArg(I) = OSToString(osNone);
      end
    else if Checkoption(I,'t','target') then
      Defaults.Target:=OptionArg(I)
    else if CheckOption(I,'lc','list-commands') then
      FListMode:=True
    else if Checkoption(I,'P','prefix') then
      Defaults.Prefix:=OptionArg(I)
    else if Checkoption(I,'n','nofpccfg') then
      Defaults.NoFPCCfg:=true
    else if Checkoption(I,'zp','zipprefix') then
      Defaults.ZipPrefix:=OptionArg(I)
{$ifdef HAS_UNIT_PROCESS}
    else if Checkoption(I,'e','useenv') then
      Defaults.UseEnvironment:=true
{$endif}
{$ifndef NO_THREADING}
    else if CheckOption(I,'T','threads') then
      Defaults.ThreadsAmount:=StrToIntDef(OptionArg(I),-1)
{$endif NO_THREADING}
    else if CheckOption(I,'B','baseinstalldir') then
      Defaults.BaseInstallDir:=OptionArg(I)
    else if CheckOption(I,'BI','bininstalldir') then
      Defaults.BinInstallDir:=OptionArg(I)
    else if CheckOption(I,'LI','libinstalldir') then
      Defaults.LibInstallDir:=OptionArg(I)
    else if CheckOption(I,'U','unitinstalldir') then
      Defaults.UnitInstallDir:=OptionArg(I)
    else if CheckOption(I,'UL','localunitdir') then
      begin
        UnitDirSet:=true;
        // Do not warn against the combination of SearchPath and
        // localunitdir when they are the same. (This is done to be able
        // to call both older and newer versions of fpmake)
        if SearchPathSet and (Defaults.LocalUnitDir<>OptionArg(I)) then
          Log(vlWarning,SWarnCombinedPathAndUDir);
        Defaults.LocalUnitDir:=OptionArg(I)
      end
    else if CheckOption(I,'UG','globalunitdir') then
      begin
        UnitDirSet:=true;
        // Do not warn against the combination of SearchPath and
        // localunitdir when they are the same. (This is done to be able
        // to call both older and newer versions of fpmake)
        if SearchPathSet and (Defaults.GlobalUnitDir<>OptionArg(I)) then
          Log(vlWarning,SWarnCombinedPathAndUDir);
        Defaults.GlobalUnitDir:=OptionArg(I)
      end
    else if CheckOption(I,'sp','searchpath') then
      begin
        SearchPathSet:=true;
        if UnitDirSet then
          Log(vlWarning,SWarnCombinedPathAndUDir);
        Defaults.SearchPath.Add(OptionArg(I, true));
      end
    else if CheckOption(I,'o','options', true) then
      begin
        OptString := OptionArg(I, true);
        while OptString <> '' do
          Defaults.Options.Add(SplitSpaces(OptString));
      end
    else if CheckOption(I,'r','compiler') then
      Defaults.Compiler:=OptionArg(I)
    else if CheckOption(I,'f','config') then
      DefaultsFileName:=OptionArg(I)
    else if CheckOption(I,'ie','installexamples') then
      Defaults.InstallExamples:=true
    else if CheckOption(I,'scp','skipcrossprograms') then
      Defaults.SkipCrossPrograms:=true
    else if CheckOption(I,'bu','buildunit') then
      Defaults.BuildMode:=bmBuildUnit
    else if CheckOption(I,'io','ignoreinvalidoption', true) then
      Defaults.IgnoreInvalidOptions:=true
    else if CheckOption(I,'df','doc-folder') then
      Defaults.FPDocOutputDir:=OptionArg(I)
    else if CheckCommand(I,'do','doc-options') then
      Defaults.FPDocOptions:=OptionArg(I)
    else if CheckCommand(I,'sd','single-docfile') then
      Defaults.SingleFPDocFile:=True
    else if CheckOption(I,'fsp','fpunitsrcpath') then
      Defaults.FPUnitSourcePath:=OptionArg(I)
    else if assigned(CustomFpmakeCommandlineOptions) and CheckCustomOption(I,CustOptName) then
      begin
      if not assigned(CustomFpMakeCommandlineValues) then
        CustomFpMakeCommandlineValues := TStringList.Create;
      CustomFpMakeCommandlineValues.Values[CustOptName]:=OptionArg(I, true)
      end
    else if (not CheckBuildOptionSetValue(I)) and (not CheckPackageVariantOptionSetValue(I))
      and (not Defaults.IgnoreInvalidOptions) then
      begin
      Usage(SErrInValidArgument,[I,ParamStr(I)]);
      end;
    end;
  If DefaultsFileName<>'' then
    Defaults.LocalInit(DefaultsFileName);
  Defaults.CompilerDefaults;
end;


procedure TCustomInstaller.Usage(const FMT: String; const Args: array of const);

  Procedure LogCmd(const LC,Msg : String);
  begin
    Log(vlInfo,Format(' %-12s %s',[LC,MSG]));
  end;

  Procedure LogOption(const C,LC,Msg : String);
  begin
    Log(vlInfo,Format(' -%s --%-16s %s',[C,LC,MSG]));
  end;

  Procedure LogArgOption(const C,LC,Msg : String);
  begin
    if trim(c)='' then
      Log(vlInfo,Format('    --%-20s %s',[LC+'='+SValue,MSG]))
    else
      Log(vlInfo,Format(' -%s --%-20s %s',[C,LC+'='+SValue,MSG]));
  end;

var
  i: Integer;
begin
  // Force the Usage to be displayed
  Include(FLogLevels,vlInfo);
  If (FMT<>'') then
    Log(vlInfo,Format(Fmt,Args));
  Log(vlInfo,Format(SHelpUsage,[Paramstr(0)]));
  Log(vlInfo,SHelpCommand);
  LogCmd('compile',SHelpCompile);
  LogCmd('build',SHelpBuild);
  LogCmd('install',SHelpInstall);
  LogCmd('buildinstall',SHelpBuildInstall);
  LogCmd('uninstall',SHelpUnInstall);
  LogCmd('clean',SHelpClean);
  LogCmd('distclean',SHelpDistclean);
  LogCmd('archive',SHelpArchive);
  LogCmd('manifest',SHelpManifest);
  LogCmd('zipinstall',SHelpZipInstall);
  LogCmd('pkglist',SHelpPkgList);
  LogCmd('fpdocproject',SHelpFPDocProject);
  Log(vlInfo,SHelpCmdOptions);
  LogOption('h','help',SHelpHelp);
  LogOption('l','list-commands',SHelpList);
  LogOption('n','nofpccfg',SHelpNoFPCCfg);
  LogOption('v','verbose',SHelpVerbose);
  LogOption('d','debug',SHelpDebug);
  LogOption('I','interactive',SHelpInteractive);
{$ifdef HAS_UNIT_PROCESS}
  LogOption('e', 'useenv', sHelpUseEnvironment);
{$endif}
  LogOption('ie','installexamples',SHelpInstExamples);
  LogOption('bu','buildunit',SHelpUseBuildUnit);
  LogOption('scp','skipcrossprograms',SHelpSkipCrossProgs);
  LogOption('io','ignoreinvalidoption',SHelpIgnoreInvOpt);
  LogArgOption('C','cpu',SHelpCPU);
  LogArgOption('O','os',SHelpOS);
  LogArgOption('t','target',SHelpTarget);
  LogArgOption('P','prefix',SHelpPrefix);
  LogArgOption('B','baseinstalldir',SHelpBaseInstalldir);
  LogArgOption('BI','bininstalldir',SHelpBaseInstalldir);
  LogArgOption('LI','libinstalldir',SHelpBaseInstalldir);
  LogArgOption('UL','localunitdir',SHelpLocalUnitdir);
  LogArgOption('UG','globalunitdir',SHelpGlobalUnitdir);
  LogArgOption('sp','searchpath',SHelpSearchPath);
  LogArgOption('U','unitinstalldir',SHelpUnitInstallDir);
  LogArgOption('r','compiler',SHelpCompiler);
  LogArgOption('f','config',SHelpConfig);
  LogArgOption('o','options',SHelpOptions);
  LogArgOption('df', 'doc-folder', sHelpFpdocOutputDir);
  LogArgOption('sd','single-docfile', sHelpSingleFpdocFile);
  LogArgOption('do','doc-options', sHelpDocOptionsFile);
  LogArgOption('fsp', 'fpunitsrcpath', sHelpFPUnitSrcPath);
  LogArgOption('zp', 'zipprefix', sHelpZipPrefix);
{$ifndef NO_THREADING}
  LogArgOption('T', 'threads', sHelpThreads);
{$endif NO_THREADING}
  if assigned(CustomFpmakeCommandlineOptions) then for i  := 0 to CustomFpmakeCommandlineOptions.Count-1 do
    LogArgOption(' ',CustomFpmakeCommandlineOptions.Names[i],CustomFpmakeCommandlineOptions.ValueFromIndex[i]);
  Log(vlInfo, sHelpPackageVariant1);
  Log(vlInfo, sHelpPackageVariant2);
  Log(vlInfo, sHelpPackageVariant3);
  Log(vlInfo, sHelpPackageVariant4);
  Log(vlInfo, sHelpPackageVariant5);
  Log(vlInfo, sHelpPackageVariant6);
  Log(vlInfo,'');

  If (FMT<>'') then
    halt(1)
  else
    halt(0);
end;

procedure TCustomInstaller.Info;
Var Cpu : TCpu;
    OS  : TOS;

begin
  Write('CPU_TARGET=');
  for cpu:=succ(low(cpu)) to high(tcpu) do // skip NONE
    begin
       write(cputostring(cpu));
       if cpu<>high(tcpu) then
       write(',');
    end;
  writeln;
  Write('OS_TARGET=');
  for OS:=succ(low(TOS)) to high(tOS) do // skip NONE
    begin
       write(OStostring(os));
       if os<>high(tos) then
       write(',');
    end;
  writeln;
end;

procedure TCustomInstaller.Compile(Force: Boolean);
begin
  FNotifyEventCollection.CallEvents(neaBeforeCompile, Self);
  FBuildEngine.ForceCompile:=Force;
  FBuildEngine.Compile(Packages);
  FNotifyEventCollection.CallEvents(neaAfterCompile, Self);
end;


procedure TCustomInstaller.Clean(AllTargets: boolean);
begin
  NotifyEventCollection.CallEvents(neaBeforeClean, Self);
  BuildEngine.Clean(Packages, AllTargets);
  NotifyEventCollection.CallEvents(neaAfterClean, Self);
end;


procedure TCustomInstaller.Install(ForceBuild : Boolean);
begin
  NotifyEventCollection.CallEvents(neaBeforeInstall, self);
  BuildEngine.ForceCompile := ForceBuild;
  BuildEngine.Install(Packages);
  NotifyEventCollection.CallEvents(neaAfterInstall, self);
end;

procedure TCustomInstaller.ZipInstall;
begin
  NotifyEventCollection.CallEvents(neaBeforeInstall, self);
  BuildEngine.ZipInstall(Packages);
  NotifyEventCollection.CallEvents(neaAfterInstall, self);
end;


procedure TCustomInstaller.UnInstall;
begin
  NotifyEventCollection.CallEvents(neaBeforeUnInstall, self);
  BuildEngine.UnInstall(Packages);
  NotifyEventCollection.CallEvents(neaAfterUnInstall, self);
end;


procedure TCustomInstaller.Archive;
begin
  NotifyEventCollection.CallEvents(neaBeforeArchive, self);
  BuildEngine.Archive(Packages);
  NotifyEventCollection.CallEvents(neaAfterArchive, self);
end;


procedure TCustomInstaller.Manifest;
begin
  NotifyEventCollection.CallEvents(neaBeforeManifest, self);
  BuildEngine.Manifest(Packages, nil);
  NotifyEventCollection.CallEvents(neaAfterManifest, self);
end;


procedure TCustomInstaller.PkgList;
begin
  NotifyEventCollection.CallEvents(neaBeforePkgList, self);
  BuildEngine.PkgList(Packages);
  NotifyEventCollection.CallEvents(neaAfterPkgList, self);
end;

procedure TCustomInstaller.FPDocProject;
begin
  NotifyEventCollection.CallEvents(neaBeforeDocProject, self);
  BuildEngine.FPDocProject(Packages,Defaults.SingleFPDocFile);
  NotifyEventCollection.CallEvents(neaAfterDocProject, self);
end;

procedure TCustomInstaller.AddAutoPackageVariantsToPackage(APackage: TPackage);
var
  i: Integer;
  PV: TPackageVariants;
begin
  for i := 0 to FPackageVariants.Count-1 do
    begin
    PV := TObject(FPackageVariants.Items[i]) as TPackageVariants;
    if PV.AutoAddToPackage then
      APackage.AddPackageVariant(PV);
    end;
end;

procedure TCustomInstaller.CheckPackages;
begin
  If (Packages.Count=0) then
    Error(SErrNoPackagesDefined);
  // Check for other obvious errors ?
end;


Function TCustomInstaller.Run : Boolean;
begin
  Result:=True;
  try
    CheckPackages;
    CreateBuildEngine;
    Case RunMode of
      rmCompile : Compile(False);
      rmBuild   : Compile(True);
      rmInstall : Install(False);
      rmBuildInstall: Install(True);
      rmZipInstall : ZipInstall;
      rmArchive : Archive;
      rmClean    : Clean(False);
      rmDistClean: Clean(True);
      rmManifest : Manifest;
      rmPkgList : PkgList;
      rmUnInstall : UnInstall;
      rmInfo      : Info;
      rmDocProject : FPDocProject;
    end;
  except
    On E : Exception do
      begin
      Log(vlError,SErrInstaller);
      Log(vlError,E.Message);
      Result:=False;
      end;
  end;
  // Force returning an exitcode to the shell
  if not Result then
    ExitCode:=1;
end;


{****************************************************************************
                                TFPCInstaller
****************************************************************************}

constructor TFPCInstaller.Create(AOwner: TComponent);
begin
  if assigned(Defaults) then
    Error(SErrAlreadyInitialized);
  Defaults:=TFPCDefaults.Create;
  inherited Create(AOwner);
end;


{****************************************************************************
                                 TBasicInstaller
****************************************************************************}

constructor TBasicInstaller.Create(AOwner: TComponent);
begin
  if assigned(Defaults) then
    Error(SErrAlreadyInitialized);
  Defaults:=TBasicDefaults.Create;
  inherited Create(AOwner);
end;


{****************************************************************************
                                 TBuildEngine
****************************************************************************}

constructor TBuildEngine.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  // Maybe this should be the current directory ?
  // Or have it as a command-line option.
  // Would allow to put all 'installers' in one dir and call them
  // With --start-dir=/path/to/sources.
  FStartDir:=includeTrailingPathDelimiter(GetCurrentDir);
  FExternalPackages:=TPackages.Create(TPackage);
  FNotifyEventCollection := TNotifyEventCollection.create([neaAfterCompile, neaBeforeCompile, neaAfterInstall, neaBeforeInstall,
                                                           neaAfterClean, neaBeforeClean, neaAfterArchive, neaBeforeArchive,
                                                           neaAfterManifest, neaBeforeManifest, neaAfterPkgList, neaBeforePkgList,
                                                           neaBeforeUnInstall, neaAfterUnInstall]);
{$ifndef NO_THREADING}
  InitCriticalSection(FGeneralCriticalSection);
{$endif NO_THREADING}
end;


destructor TBuildEngine.Destroy;
begin
  FreeAndNil(FExternalPackages);
  FreeAndNil(FNotifyEventCollection);

{$ifndef NO_THREADING}
  DoneCriticalsection(FGeneralCriticalSection);
{$endif NO_THREADING}

  inherited Destroy;
end;

procedure TBuildEngine.AddFileToArchive(const APackage: TPackage; const ASourceFileName, ADestFileName: String);

  function GetArchiveName: string;
  begin
    result := Defaults.ZipPrefix;
    if Defaults.BuildOS in AllLimit83fsOses then
      result := result + APackage.ShortName
    else
      result := result + APackage.Name;
    result := result + MakeZipSuffix(Defaults.CPU, Defaults.OS);
  end;

{$ifdef UNIX}
var
  FileStat: stat;
{$endif UNIX}
begin
{$ifdef CREATE_TAR_FILE}
  {$ifdef HAS_TAR_SUPPORT}
  if not assigned(FTarWriter) then
    begin
    {$ifdef HAS_UNIT_ZIPPER}
      FGZFileStream := TGZFileStream.create(GetArchiveName + ArchiveExtension, gzopenwrite);
      try
        FTarWriter := TTarWriter.Create(FGZFileStream);
      except
        FGZFileStream.Free;
      end;
    {$else}
    FTarWriter := TTarWriter.Create(GetArchiveName + ArchiveExtension);
    {$endif HAS_UNIT_ZIPPER}
    FTarWriter.Permissions := [tpReadByOwner, tpWriteByOwner, tpReadByGroup, tpReadByOther];
    FTarWriter.UserName := 'root';
    FTarWriter.GroupName := 'root';
    end;
{$ifdef unix}
  if (FpStat(ASourceFileName, FileStat) = 0) and (FileStat.st_mode and S_IXUSR = S_IXUSR) then
    begin
    FTarWriter.Permissions := FTarWriter.Permissions + [tpExecuteByGroup];
    FTarWriter.Permissions := FTarWriter.Permissions + [tpExecuteByOwner];
    FTarWriter.Permissions := FTarWriter.Permissions + [tpExecuteByOther];
    end
  else
    begin
    FTarWriter.Permissions := FTarWriter.Permissions - [tpExecuteByGroup];
    FTarWriter.Permissions := FTarWriter.Permissions - [tpExecuteByOwner];
    FTarWriter.Permissions := FTarWriter.Permissions - [tpExecuteByOther];
    end;
{$endif unix}
  FTarWriter.AddFile(ASourceFileName, ADestFileName);
  {$endif HAS_TAR_SUPPORT}
{$else CREATE_TAR_FILE}
  {$ifdef HAS_UNIT_ZIPPER}
  if not assigned(FZipper) then
    begin
      FZipper := TZipper.Create;
      FZipper.FileName := GetArchiveName + ArchiveExtension;
    end;

  FZipper.Entries.AddFileEntry(ASourceFileName, ADestFileName);
  {$endif HAS_UNIT_ZIPPER}
{$ENDIF CREATE_TAR_FILE}
end;

procedure TBuildEngine.FinishArchive(Sender: TObject);
begin
  {$ifdef HAS_TAR_SUPPORT}
  if assigned(FTarWriter) then
    begin
      FreeAndNil(FTarWriter);
      {$ifdef HAS_UNIT_ZIPPER}
      FGZFileStream.Free;
      {$endif HAS_UNIT_ZIPPER}
    end;
  {$endif HAS_TAR_SUPPORT}
  {$ifdef HAS_UNIT_ZIPPER}
  if assigned(FZipper) then
    begin
      try
        FZipper.ZipAllFiles;
        FZipper.Clear;
      finally
        FreeAndNil(FZipper);
      end;
    end;
  {$endif HAS_UNIT_ZIPPER}
end;


procedure TBuildEngine.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TBuildEngine.Error(const Fmt: String; const Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


procedure TBuildEngine.ExecuteCommand(const Cmd,Args : String; const Env: TStrings = nil; IgnoreError : Boolean = False);
Var
  E : Integer;
  cmdLine: string;
  ConsoleOutput: TMemoryStream;
  s: string;
begin
  Log(vlInfo,SInfoExecutingCommand,[Cmd,Args]);
  if ListMode then
    Log(vlCommand,'%s %s',[Cmd,Args])
  else
    begin
      // We should check cmd for spaces, and move all after first space to args.
      ConsoleOutput := TMemoryStream.Create;
      try
        if Interactive then
          Log(vlInfo,'Starting "%s" "%s" interactively',[Cmd,Args]);
        {$ifdef HAS_UNIT_PROCESS}
        E:=ExecuteFPC(Verbose, Interactive, cmd, args, env, ConsoleOutput);
        {$else}
        E:=ExecuteProcess(cmd,args);
        {$endif}
        If (E<>0) and (not IgnoreError) then
          begin
            if trim(Args)<>'' then
              cmdLine := cmd + ' ' + trim(args)
            else
              cmdline := cmd;
            s := ParsecompilerOutput(ConsoleOutput,Verbose);
            Error(SErrExternalCommandFailed,[cmdLine,E,s]);
          end;
      finally
        ConsoleOutput.Free;
      end;
    end;
end;


Function TBuildEngine.SysDirectoryExists(const ADir:string):Boolean;
begin
  result:=SysUtils.DirectoryExists(ADir);
  if result then
    Log(vlDebug,SDbgDirectoryExists,[ADir,SDbgFound])
  else
    Log(vlDebug,SDbgDirectoryExists,[ADir,SDbgNotFound]);
end;


Function TBuildEngine.SysFileExists(const AFileName:string):Boolean;
begin
  result:=SysUtils.FileExists(AFileName);
  if result then
    Log(vlDebug,SDbgFileExists,[AFileName,SDbgFound])
  else
    Log(vlDebug,SDbgFileExists,[AFileName,SDbgNotFound]);
end;


procedure TBuildEngine.SysCopyFile(Const Src,Dest : String);
Var
  D,S : String;
  Fin,FOut : TFileStream;
  Count : Int64;
  A : Integer;
{$ifdef UNIX}
  FileStat: stat;
{$endif UNIX}
begin
  Log(vlInfo,SInfoCopyingFile,[Src,Dest]);
  FIn:=TFileStream.Create(Src,fmopenRead or fmShareDenyNone);
  Try
    D:=IncludeTrailingPathDelimiter(Dest);
    If DirectoryExists(D) then
      S:=D+ExtractFileName(Src)
    else
      S:=Dest;
    FOut:=TFileStream.Create(S,fmCreate or fmShareDenyNone);
    Try
      Count:=Fout.CopyFrom(FIn,0);
      If (Count<>Fin.Size) then
        Error(SErrCopyingFile,[Src,S]);
    Finally
      FreeAndNil(Fout);
    end;
    A:=FileGetDate(FIn.Handle);
    If (A=-1) then
      log(vlWarning,SWarnFailedToGetTime,[Src])
    else
      if FileSetDate(S,A)<>0 then
        Log(vlWarning,SWarnFailedToSetTime,[S]);
{$ifdef UNIX}
    // Copy the file-access rights on Unix, especially the executable-bit
    if FpStat(Src,FileStat) <> 0 then
      Log(vlWarning,SWarnCanNotGetAccessRights,[Src])
    else
      if FpChmod(s,FileStat.st_mode) <> 0 then
        Log(vlWarning,SWarnCanNotSetAccessRights,[S]);
{$endif UNIX}
  finally
    FreeAndNil(Fin);
  end;
end;


procedure TBuildEngine.SysMoveFile(Const Src,Dest : String);
Var
  S : String;
begin
  If DirectoryExists(IncludeTrailingPathDelimiter(Dest)) then
    S:=IncludeTrailingPathDelimiter(Dest)+ExtractFileName(Src)
  else
    S:=Dest;
  If Not RenameFile(Src,S) then
    begin
      Try
        SysCopyFile(Src,S);
        SysDeleteFile(Src);
      Except
        On E : Exception Do
          Error(SErrMovingFile,[Src,S]);
      end;
    end;
end;


procedure TBuildEngine.SysDeleteFile(Const AFileName : String);
var retries : integer;
    res : boolean;
begin
  if not FileExists(AFileName) then
    Log(vldebug,SDbgFileDoesNotExist,[AFileName])
  else
    begin
      retries := 2;
      res := SysUtils.DeleteFile(AFileName);
      while not res and (retries>0) do
        begin
           log(vlWarning, SWarnRetryDeleteFile, [AFileName]);
           sleep(5000);
           dec(retries);
           res := SysUtils.DeleteFile(AFileName);
        end;
     if not res then
       Error(SErrDeletingFile,[AFileName])
     else
       Log(vlInfo,SInfoDeletedFile,[AFileName]);
   end;
end;

procedure TBuildEngine.SysDeleteDirectory(Const ADirectoryName: String);
begin
  if not DirectoryExists(ADirectoryName) then
    Log(vldebug,SDbgDirectoryDoesNotExist,[ADirectoryName])
  else if not IsDirectoryEmpty(ADirectoryName) then
    Log(vldebug,SDbgDirectoryNotEmpty,[ADirectoryName])
  else If Not RemoveDir(ADirectoryName) then
    Error(SErrRemovingDirectory,[ADirectoryName])
  else
    Log(vlInfo,SInfoRemovedDirectory,[ADirectoryName]);
end;


procedure TBuildEngine.SysDeleteTree(Const ADirectoryName: String);

  function IntRemoveTree(const ADirectoryName: String) : boolean;
{$ifdef MSWINDOWS}
  { pulling in shellapi with all it dependent units and packages makes things too
    complicated so just add the constants here }
  const
    FO_DELETE                = $0003;
    FOF_SILENT               = $0004;
    FOF_NOCONFIRMATION       = $0010;
{$endif MSWINDOWS}
  var
    retries: integer;
{$ifdef MSWINDOWS}
    SHFileOpStruct: TSHFileOpStruct;
    DirBuf: array[0..MAX_PATH+1] of TCHAR;
{$else MSWINDOWS}
    searchRec: TSearchRec;
    SearchResult: longint;
    s: string;
{$endif MSWINDOWS}

  begin
    result := true;
{$ifdef MSWINDOWS}
    retries:=2;
    try
      FillChar(SHFileOpStruct, Sizeof(SHFileOpStruct), 0);
      FillChar(DirBuf, Sizeof(DirBuf), 0);
      StrPCopy(DirBuf, ADirectoryName);
      with SHFileOpStruct do
      begin
        pFrom := @DirBuf;
        wFunc := FO_DELETE;
        fFlags := FOF_NOCONFIRMATION or FOF_SILENT;
      end;
      Result := SHFileOperation(SHFileOpStruct) = 0;
    except
      Result := False;
    end;
    while not result and (retries>0) do
      begin
        log(vlWarning, SWarnRetryRemDirectory, [ADirectoryName]);
        sleep(5000);
        dec(retries);
        result := SHFileOperation(SHFileOpStruct) = 0;;
      end;

{$else MSWINDOWS}
    SearchResult := FindFirst(IncludeTrailingPathDelimiter(ADirectoryName)+AllFilesMask, faAnyFile+faSymLink, searchRec);
    try
      while SearchResult=0 do
        begin
          if (searchRec.Name<>'.') and (searchRec.Name<>'..') then
             begin
               s := IncludeTrailingPathDelimiter(ADirectoryName)+searchRec.Name;
               if (searchRec.Attr and faDirectory)=faDirectory then
                 begin
                   if not IntRemoveTree(s) then
                     result := false;
                 end
               else if not DeleteFile(s) then
                 result := False
               else
                 log(vldebug, SDbgDeletedFile, [s]);
             end;
          SearchResult := FindNext(searchRec);
        end;
    finally
      FindClose(searchRec);
    end;

    // There were reports of RemoveDir failing due to locking-problems. To solve
    // these the RemoveDir is tried three times, with a delay of 5 seconds. See
    // bug 21868
    retries := 2;
    result := RemoveDir(ADirectoryName);
    while not result and (retries>0) do
      begin
        log(vlWarning, SWarnRetryRemDirectory, [ADirectoryName]);
        sleep(5000);
        dec(retries);
        result := RemoveDir(ADirectoryName);
      end;

{$endif WINDOWS}

    if result then
      log(vldebug, SDbgRemovedDirectory, [ADirectoryName]);
  end;

begin
  if not DirectoryExists(ADirectoryName) then
    Log(vldebug,SDbgDirectoryDoesNotExist,[ADirectoryName])
  else If Not IntRemoveTree(ADirectoryName) then
    Error(SErrRemovingDirectory,[ADirectoryName])
  else
    Log(vlInfo,SInfoRemovedDirectory,[ADirectoryName]);
end;


procedure TBuildEngine.SysArchiveFiles(List: TStrings;Const AFileName: String);
begin
  If Not (Assigned(OnArchivefiles) or Assigned(ArchiveFilesProc)) then
    Raise EInstallerError.Create(SErrNoArchiveSupport);
  If Assigned(ArchiveFilesProc) then
    ArchiveFilesProc(AFileName,List)
  else
    OnArchiveFiles(AFileName,List);
end;


procedure TBuildEngine.LogIndent;
begin
  GLogPrefix:=GLogPrefix+'  ';
end;


procedure TBuildEngine.LogUnIndent;
begin
  Delete(GLogPrefix,1,2);
end;


procedure TBuildEngine.Log(Level: TVerboseLevel; Msg: String);
begin
  If Assigned(FOnLog) then
    begin
      if Level in [vlInfo,vlDebug] then
        FOnLog(Level,GLogPrefix+Msg)
      else
        FOnLog(Level,Msg);
    end;
end;


procedure TBuildEngine.Log(Level: TVerboseLevel; Fmt: String;const Args: array of const);
begin
  Log(Level,Format(Fmt,Args));
end;


procedure TBuildEngine.EnterDir(ADir: String);
Var
  D : String;
begin
  D:=FStartDir;
  D:=D+ADir;
  Log(vlDebug,SDbgEnterDir,[D]);
  If Not SetCurrentDir(D) then
    Error(SErrChangeDirFailed,[D]);
end;


procedure TBuildEngine.CmdCopyFiles(List: TStrings; Const DestDir: String; APackage : TPackage);

Var
  Args : String;
  I : Integer;
  DestFileName : String;
begin
  // When the files should be written to an archive, add them
  if assigned(FOnCopyFile) then
    begin
      For I:=0 to List.Count-1 do
        if List.Names[i]<>'' then
          begin
            if IsRelativePath(list.ValueFromIndex[i]) then
              DestFileName:=DestDir+list.ValueFromIndex[i]
            else
              DestFileName:=list.ValueFromIndex[i];
            FOnCopyFile(APackage, AddPathPrefix(APackage, List.Names[i]), DestFileName);
          end
        else
          FOnCopyFile(APackage, AddPathPrefix(APackage, List[i]), DestDir+ExtractFileName(List[i]));
      Exit;
    end;

  // Copy the files to their new location on disk
  CmdCreateDir(DestDir);
  If (Defaults.Copy<>'') then
    begin
      Args:=FileListToString(List, IncludeTrailingPathDelimiter(GPathPrefix));
      Args:=Args+' '+DestDir;
      ExecuteCommand(Defaults.Copy,Args);
    end
  else
    For I:=0 to List.Count-1 do
      if List.Names[i]<>'' then
        begin
          if IsRelativePath(list.ValueFromIndex[i]) then
            DestFileName:=DestDir+list.ValueFromIndex[i]
          else
            DestFileName:=list.ValueFromIndex[i];
          CmdCreateDir(ExtractFilePath(DestFileName));
          SysCopyFile(AddPathPrefix(APackage, List.Names[i]),DestFileName)
        end
      else
        SysCopyFile(AddPathPrefix(APackage, List[i]), DestDir);
end;


procedure TBuildEngine.CmdCreateDir(const DestDir: String);
begin
  If (Defaults.MkDir<>'') then
    ExecuteCommand(Defaults.MkDir,DestDir)
  else
    If not ForceDirectories(DestDir) then
      Error(SErrCreatingDirectory,[DestDir]);
end;


procedure TBuildEngine.CmdMoveFiles(List: TStrings; Const DestDir: String);
Var
  Args : String;
  I : Integer;
begin
  CmdCreateDir(DestDir);
  If (Defaults.Move<>'') then
    begin
      Args:=FileListToString(List,'');
      Args:=Args+' '+DestDir;
      ExecuteCommand(Defaults.Move,Args);
    end
  else
    For I:=0 to List.Count-1 do
      SysMoveFile(List[i],DestDir);
end;


procedure TBuildEngine.CmdDeleteFiles(List: TStrings);
Var
  Args : String;
  I : Integer;
begin
  If (Defaults.Remove<>'') then
    begin
      Args:=FileListToString(List,'');
      ExecuteCommand(Defaults.Remove,Args);
    end
  else
    For I:=0 to List.Count-1 do
      SysDeleteFile(List[i]);
end;

procedure TBuildEngine.CmdDeleteDestFiles(List: TStrings; Const DestDir: String);

Var
  I : Integer;
  DeleteFileName : String;
begin
  // Delete files from their location on disk
  For I:=0 to List.Count-1 do
    begin
      if List.Names[i]<>'' then
        begin
          if IsRelativePath(list.ValueFromIndex[i]) then
            DeleteFileName:=DestDir+list.ValueFromIndex[i]
          else
            DeleteFileName:=list.ValueFromIndex[i];
        end
      else
        DeleteFileName:=DestDir+ExtractFileName(list[i]);
      SysDeleteFile(DeleteFileName);
    end;
end;


procedure TBuildEngine.CmdArchiveFiles(List: TStrings; Const ArchiveFile: String);
Var
  S,C,O : String;
begin
  If (Defaults.Archive='') then
    SysArchiveFiles(List,ArchiveFile)
  else
    begin
      S:=FileListToString(List,IncludeTrailingPathDelimiter(GPathPrefix));
      SplitCommand(Defaults.Archive,C,O);
      If (O='') then
        O:=ArchiveFile+' '+S
      else
        O:=GlobalDictionary.Substitute(O,['ARCHIVE',ArchiveFile,'FILESORDIRS']);
      ExecuteCommand(C,O);
    end;
end;

procedure TBuildEngine.CmdRenameFile(SourceName, DestName: String);
var
  Args: string;
begin
  If (Defaults.Move<>'') then
    begin
      Args:=SourceName;
      Args:=Args+' '+DestName;
      ExecuteCommand(Defaults.Move,Args);
    end
  else
    SysMoveFile(SourceName,DestName);
end;

procedure TBuildEngine.CmdRemoveDirs(List: TStrings);
Var
  Args : String;
  I : Integer;
begin
  If (Defaults.RemoveDir<>'') then
    begin
      Args:=FileListToString(List,'');
      ExecuteCommand(Defaults.RemoveDir,Args);
    end
  else
    For I:=0 to List.Count-1 do
      SysDeleteDirectory(List[i]);
end;

procedure TBuildEngine.CmdRemoveTrees(List: TStrings);
Var
  Args : String;
  I : Integer;
begin
  If (Defaults.RemoveTree<>'') then
    begin
      Args:=FileListToString(List,'');
      ExecuteCommand(Defaults.RemoveTree,Args);
    end
  else
    For I:=0 to List.Count-1 do
      SysDeleteTree(List[i]);
end;

Function TBuildEngine.FileNewer(const Src,Dest : String) : Boolean;

Var
  DS,DD : Longint;
  D1,D2 : TDateTime;

begin
  DS:=FileAge(Src);
  { Return false if file not found or not accessible }
  if DS=-1 then
    begin
      Log(vlWarning,SWarnCanNotGetFileAge,[Src]);
      Result:=false;
      exit;
    end;
  DD:=FileAge(Dest);
  D1:=FileDateToDateTime(DS);
  D2:=FileDateToDateTime(DD);
  Log(vlDebug,SDbgComparingFileTimes,[Src,DateTimeToStr(D1),Dest,DateTimeToStr(D2)]);
  Result:=D1>D2;
  If Result then
    Log(vlInfo,SInfoSourceNewerDest,[Src,DateTimeToStr(D1),Dest,DateTimeToStr(D2)]);
end;


procedure TBuildEngine.ExecuteCommands(Commands: TCommands; At: TCommandAt; APackage: TPackage);
Var
  C : TCommand;
  I : Integer;
  Cmd,O : String;
  E : Boolean;
  ADictionary: TDictionary;
  SourceFile, DestFile: string;
begin
  For I:=0 to Commands.Count-1 do
    begin
      C:=Commands.CommandItems[i];
      if (C.At=At) then
        begin
          E:=True;

          if assigned(APackage) then
            ADictionary := APackage.Dictionary
          else
            ADictionary := GlobalDictionary;
          SourceFile := ADictionary.ReplaceStrings(C.SourceFile);
          DestFile := ADictionary.ReplaceStrings(C.DestFile);
          if IsRelativePath(SourceFile) then
            SourceFile := AddPathPrefix(APackage,SourceFile);
          if IsRelativePath(DestFile) then
            DestFile := AddPathPrefix(APackage,DestFile);

          Cmd:=C.Command;
          If (ExtractFilePath(Cmd)='') then
            Cmd:=ExeSearch(Cmd,SysUtils.GetEnvironmentvariable('PATH'));

          If (SourceFile<>'') and (DestFile<>'')  then
            begin
              if not FileExists(DestFile) then
                Log(vlInfo,SInfoDestDoesNotExist,[DestFile])
              else
                begin
                E:=FileNewer(SourceFile, DestFile);
                if E and (cmd = '') then
                  begin
                  log(vlWarning,SWarnExtCommandNotFound,[C.Command,DestFile,SourceFile]);
                  E := False;
                  end;
                end;
            end;
          If E then
            begin
            if Cmd = '' then
              error(SErrExtCommandNotFound,[C.Command]);

            If Assigned(C.BeforeCommand) then
              C.BeforeCommand(C);
            O:=ADictionary.Substitute(C.CmdLineOptions,['SOURCE',SourceFile,'DEST',DestFile]);

            Log(vlCommand,SInfoExecutingCommand,[Cmd,O]);
            ExecuteCommand(Cmd,O,nil,C.IgnoreResult);
            If Assigned(C.AfterCommand) then
              C.AfterCommand(C);
            end;
        end;
    end;
end;


procedure TBuildEngine.LogSearchPath(APackage: TPackage; const ASearchPathName: string; Path: TConditionalStrings; ACPU: TCPU; AOS: TOS);
var
  S : String;
  I : Integer;
  C : TConditionalString;
begin
  S:='';
  for i:=0 to Path.Count-1 do
    begin
      C:=Path[I];
      if (ACPU in C.CPUs) and (AOS in C.OSes) then
        begin
          if S<>'' then
            S:=S+PathSeparator;
          S:=S+APackage.Dictionary.ReplaceStrings(C.Value)
        end;
    end;
  if S<>'' then
    Log(vlDebug,SDbgSearchPath,[ASearchPathName,S]);
end;


Function TBuildEngine.FindFileInPath(APackage: TPackage; Path:TConditionalStrings; AFileName:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;
var
  I : Integer;
  C : TConditionalString;
begin
  Result:=false;
  for i:=0 to Path.Count-1 do
    begin
      C:=Path[I];
      if (ACPU in C.CPUs) and (AOS in C.OSes) then
        begin
          FoundPath:=IncludeTrailingPathDelimiter(APackage.Dictionary.ReplaceStrings(C.Value));
          if FileExists(AddPathPrefix(APackage,FoundPath+AFileName)) then
            begin
              result:=true;
              exit;
            end;
        end;
    end;
  FoundPath:='';
end;

procedure TBuildEngine.GetDirectoriesFromFilelist(const AFileList, ADirectoryList: TStringList);
var
  i: integer;
  s: string;
begin
  ADirectoryList.Sorted:=true;
  ADirectoryList.Duplicates:=dupIgnore;
  for i := 0 to AFileList.Count-1 do
    begin
      s := ExtractFileDir(AFileList.Strings[i]);
      if s <> '' then
        ADirectoryList.Add(s);
    end;
end;

procedure TBuildEngine.AddPackageMacrosToDictionary(const APackage: TPackage; ADictionary: TDictionary);
begin
  APackage.Dictionary.AddVariable('UNITSOUTPUTDIR',AddPathPrefix(APackage,APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS)));
  APackage.Dictionary.AddVariable('BINOUTPUTDIR',AddPathPrefix(APackage,APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS)));
  APackage.Dictionary.AddVariable('PACKAGEVERSION',APackage.Version);
  APackage.Dictionary.AddVariable('PACKAGEDIRECTORY',APackage.Directory);
  APackage.Dictionary.AddVariable('PackageName',APackage.Name);
end;

Procedure TBuildEngine.ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;DoChangeDir:boolean=true; WarnIfNotFound:boolean=true);

  procedure FindMainSource(T:TTarget);
  var
    SD,SF  : String;
  begin
    LogSearchPath(APackage,'package source',APackage.SourcePath,ACPU,AOS);
    SD:=APackage.Dictionary.ReplaceStrings(T.Directory);
    SF:=APackage.Dictionary.ReplaceStrings(T.SourceFileName);
    if SD='' then
      FindFileInPath(APackage,APackage.SourcePath,SF,SD,ACPU,AOS);
    if SD<>'' then
      SD:=IncludeTrailingPathDelimiter(SD);
    T.FTargetSourceFileName:=SD+SF;
    if FileExists(AddPathPrefix(APackage,T.TargetSourceFileName)) then
      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSourceFileName])
    else
      begin
        if WarnIfNotFound then
          Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APackage.Name,MakeTargetString(ACPU,AOS)]);
        APackage.FAllFilesResolved:=false;
        T.FTargetSourceFileName:='';
      end;
  end;

  procedure FindIncludeSources(T:TTarget);
  var
    SD,SF  : String;
    D : TDependency;
    j : integer;
  begin
    LogSearchPath(APackage,'target include',T.IncludePath,ACPU,AOS);
    LogSearchPath(APackage,'package include',APackage.IncludePath,ACPU,AOS);
    for j:=0 to T.Dependencies.Count-1 do
      begin
        D:=T.Dependencies[j];
        if (D.DependencyType=depInclude) then
          begin
            if D.TargetFileName<>'' then
              Log(vlDebug,SDbgSourceAlreadyResolved,[D.Value])
            else if (ACPU in D.CPUs) and (AOS in D.OSes) then
              begin
                if ExtractFilePath(D.Value)='' then
                  begin
                    SF:=APAckage.Dictionary.ReplaceStrings(D.Value);
                    SD:='';
                    // first check the target specific path
                    if not FindFileInPath(APackage, T.IncludePath,SF,SD,ACPU,AOS) then
                      FindFileInPath(APackage, APackage.IncludePath,SF,SD,ACPU,AOS);
                     if SD<>'' then
                       SD:=IncludeTrailingPathDelimiter(SD);
                     D.TargetFileName:=SD+SF;
                  end
                else
                  D.TargetFileName:=D.Value;
                if FileExists(AddPathPrefix(APackage,D.TargetFileName)) then
                  Log(vlDebug,SDbgResolvedIncludeFile,[D.Value,D.TargetFileName])
                else
                  begin
                    if WarnIfNotFound then
                      Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, APackage.Name, MakeTargetString(ACPU,AOS)]);
                    APackage.FAllFilesResolved:=false;
                    D.TargetFileName:='';
                  end;
              end;
          end;
      end;
  end;

  procedure FindExampleSource(T:TTarget);
  var
    SD,SF  : String;
  begin
    LogSearchPath(APackage,'package example',APackage.ExamplePath,ACPU,AOS);
    SD:=APackage.Dictionary.ReplaceStrings(T.Directory);
    SF:=APackage.Dictionary.ReplaceStrings(T.SourceFileName);
    if SD='' then
      FindFileInPath(APackage, APackage.ExamplePath,SF,SD,ACPU,AOS);
    if SD<>'' then
      SD:=IncludeTrailingPathDelimiter(SD);
    T.FTargetSourceFileName:=SD+SF;
    if FileExists(AddPathPrefix(APackage,T.TargetSourceFileName)) then
      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSourceFileName])
    else
      begin
        if WarnIfNotFound then
          Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName, APackage.Name, MakeTargetString(ACPU,AOS)]);
        T.FTargetSourceFileName:='';
        APackage.FAllFilesResolved:=false;
      end;
  end;

var
  T : TTarget;
  i : Integer;
begin
  if not((ACPU in APackage.CPUs) and (AOS in APackage.OSes)) then
    exit;
  if APackage.FAllFilesResolved then
    Exit;
  APackage.FAllFilesResolved:=true;
  try
    if DoChangeDir and (APackage.Directory<>'') then
      GPathPrefix := APackage.Directory;
    APackage.Dictionary.AddVariable('CPU',CPUToString(ACPU));
    APackage.Dictionary.AddVariable('OS',OSToString(AOS));
    For I:=0 to APackage.Targets.Count-1 do
      begin
        T:=APackage.FTargets.TargetItems[I];
        if (ACPU in T.CPUs) and (AOS in T.OSes) then
          begin
            // Debug information
            Log(vlDebug,SDbgResolvingSourcesOfTarget,[T.Name,MakeTargetString(ACPU,AOS)]);
            LogIndent;

            case T.TargetType of
              ttProgram,
              ttSharedLibrary,
              ttUnit,
              ttImplicitUnit :
                begin
                  if T.FTargetSourceFileName<>'' then
                    Log(vlDebug,SDbgSourceAlreadyResolved,[T.Name])
                  else
                    FindMainSource(T);
                  if T.Dependencies.Count>0 then
                    FindIncludeSources(T);
                end;
              ttExampleUnit,
              ttExampleProgram :
                begin
                  if T.FTargetSourceFileName<>'' then
                    Log(vlDebug,SDbgSourceAlreadyResolved,[T.Name])
                  else
                    FindExampleSource(T);
                end;
            end;

            LogUnIndent;
          end;
      end;
  finally
    If DoChangeDir and (APackage.Directory<>'') then
      GPathPrefix := '';
  end;
end;

procedure TBuildEngine.ClearResolvedFileNames(APackage: TPackage);

  procedure ClearResolvedFileNamesForDependencies(ADependencies: TDependencies);
  var
    I: Integer;
    D: TDependency;
  begin
    For I:=0 to ADependencies.Count-1 do
      begin
        D := ADependencies[I];
        D.TargetFileName:='';
      end;
  end;

var
  T : TTarget;
  I : Integer;
begin
  APackage.FAllFilesResolved:=false;
  ClearResolvedFileNamesForDependencies(APackage.Dependencies);
  For I:=0 to APackage.Targets.Count-1 do
    begin
      T:=APackage.FTargets.TargetItems[I];
      T.FTargetSourceFileName:='';
      ClearResolvedFileNamesForDependencies(T.Dependencies);
    end;
end;


procedure TBuildEngine.ResolvePackagePaths(APackage:TPackage);

var
  i: Integer;
  Continue: Boolean;
begin
{$ifndef NO_THREADING}
  APackage.EnterResolveDirsCS;
  try
{$endif}
    if APackage.UnitDir='' then
      begin
        Log(vldebug, SDbgSearchExtDepPath, [APackage.Name]);
        GetPluginManager.BeforeResolvePackagePath(Self, APackage, Continue);
        if Continue then
          begin
          for I := 0 to Defaults.SearchPath.Count-1 do
            begin
              if Defaults.SearchPath[i]<>'' then
                GetPluginManager.ResolvePackagePath(Self, APackage, Defaults.SearchPath[i], Continue);
              if not Continue then
                Break
            end;

          if Continue then
            GetPluginManager.AfterResolvePackagePath(Self, APackage, Continue);
          end;

        if APackage.UnitDir = '' then
          APackage.UnitDir := DirNotFound
      end;
{$ifndef NO_THREADING}
  finally
    APackage.LeaveResolveDirsCS;
  end;
{$endif}
end;


function TBuildEngine.GetUnitDir(APackage:TPackage):String;
begin
  ResolvePackagePaths(APackage);
  // Special error marker to prevent searches in case of error
  if APackage.UnitDir=DirNotFound then
    Result:=''
  else
    Result:=APackage.UnitDir;
end;


procedure TBuildEngine.AddDependencyPaths(L: TStrings; DependencyType: TDependencyType; ATarget: TTarget);
Var
  I : Integer;
  D : TDependency;
  SD : String;
begin
  For I:=0 to ATarget.Dependencies.Count-1 do
    begin
      D:=ATarget.Dependencies[i];
      if (D.DependencyType=DependencyType) and
         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
        begin
          SD:=ExcludeTrailingPathDelimiter(ExtractFilePath(D.TargetFileName));
          if SD<>'' then
            L.Add(SD);
        end;
    end;
end;


procedure TBuildEngine.AddDependencyUnitPaths(L:TStrings;APackage: TPackage);
Var
  I : Integer;
  P : TPackage;
  D : TDependency;
  S : String;
begin
  For I:=0 to APackage.Dependencies.Count-1 do
    begin
      D:=APackage.Dependencies[i];
      if (D.DependencyType=depPackage) and
         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
        begin
          P:=TPackage(D.Target);
          If Assigned(P) then
            begin
              // Already processed?
              S:=GetUnitDir(P);
              if L.IndexOf(S)=-1 then
                begin
                  // Add this package and then dependencies
                  L.Add(S);
                  AddDependencyUnitPaths(L,P);
                end;
            end;
        end;
    end;
end;

{$ifdef NO_THREADING}
var
{$else NO_THREADING}
threadvar
{$endif NO_THREADING}
  GHandledRecursiveDependencies: TStrings;

procedure TBuildEngine.AddDependencyTransmittedOptions(Args: TStrings; APackage: TPackage);
Var
  I, J : Integer;
  P : TPackage;
  D : TDependency;
  S : String;
  IsRootLevel: Boolean;
begin
  if not Assigned(GHandledRecursiveDependencies) then
    begin
      GHandledRecursiveDependencies := TStringList.Create;
      IsRootLevel := True;
    end
  else
    IsRootLevel := False;

  try
    For I:=0 to APackage.Dependencies.Count-1 do
      begin
        D:=APackage.Dependencies[i];
        if (D.DependencyType=depPackage) and
           (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
          begin
            P:=TPackage(D.Target);
            If Assigned(P) then
              begin
                // Already processed?
                if GHandledRecursiveDependencies.IndexOf(P.Name)=-1 then
                  begin
                    GHandledRecursiveDependencies.Add(P.Name);
                    AddDependencyTransmittedOptions(Args,P);
                    Args.AddStrings(P.TransmitOptions);
                  end;
              end;
          end;
      end;
  finally
    if IsRootLevel then
      FreeAndNil(GHandledRecursiveDependencies);
  end;
end;

function TBuildEngine.AddPathPrefix(APackage: TPackage; APath: string): string;
begin
  if IsRelativePath(APath) and (GPathPrefix<>'') then
    result := IncludeTrailingPathDelimiter(GPathPrefix) + APath
  else
    result := APath;
end;


Function TBuildEngine.GetCompilerCommand(APackage : TPackage; ATarget : TTarget; Env: TStrings) : String;
Var
  L : TUnsortedDuplicatesStringList;
  Args : TStringList;
  s : string;
  ErrS: string;
  i : Integer;
begin
  if ATarget.TargetSourceFileName = '' then
    Error(SErrCouldNotCompile,[ATarget.Name, APackage.Name]);

  Args:=TStringList.Create;
  Args.Duplicates:=dupIgnore;

  Result := '';

  //compiler configuration
  if Defaults.NoFPCCfg then
    Args.Add('-n');

  // Target OS
  Args.Add('-T'+OSToString(Defaults.OS));

  // Target CPU.
  // This setting is only applicable when 'fpc' is used as compiler-executable.
  if ExtractFileName(GetCompiler) = 'fpc' then
    Args.Add('-P'+CPUToString(Defaults.CPU));

  // Compile mode
  If ATarget.Mode<>cmFPC then
    Args.Add('-M'+ModeToString(ATarget.Mode))
  else If Defaults.Mode<>cmFPC then
    Args.Add('-M'+ModeToString(Defaults.Mode));
  // Output file paths
  If ATarget.TargetType in ProgramTargets then
    Args.Add('-FE'+AddPathPrefix(APackage,APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS)));
  Args.Add('-FU'+AddPathPrefix(APackage,APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS)));
  // Object Path
  L:=TUnsortedDuplicatesStringList.Create;
  L.Duplicates:=dupIgnore;
  AddConditionalStrings(APackage, L,APackage.ObjectPath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(APackage, L,ATarget.ObjectPath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fo'+AddPathPrefix(APackage,L[i]));
  FreeAndNil(L);
  // Unit Dirs
  L:=TUnsortedDuplicatesStringList.Create;
  L.Duplicates:=dupIgnore;
  AddDependencyUnitPaths(L,APackage);
  AddDependencyPaths(L,depUnit,ATarget);
  AddConditionalStrings(APackage, L,APackage.UnitPath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(APackage, L,ATarget.UnitPath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fu'+AddPathPrefix(APackage,L[i]));
  FreeAndNil(L);
  // Include Path
  L:=TUnsortedDuplicatesStringList.Create;
  L.Duplicates:=dupIgnore;
  AddDependencyPaths(L,depInclude,ATarget);
  AddConditionalStrings(APackage, L,APackage.IncludePath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(APackage, L,ATarget.IncludePath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fi'+AddPathPrefix(APackage,L[i]));
  FreeAndNil(L);

  // libc-linker path (always for Linux, since required for LLVM and SEH; this does not
  // force the linking of anything by itself, but just adds a search directory)
  if APackage.NeedLibC or
     (Defaults.OS=linux) then
    begin
      if FCachedlibcPath='' then
        begin
          s:=GetDefaultLibGCCDir(Defaults.CPU, Defaults.OS,ErrS);
          if s='' then
            Log(vlWarning, SWarngcclibpath +' '+ErrS)
          else
            begin
{$ifndef NO_THREADING}
              EnterCriticalsection(FGeneralCriticalSection);
              { prevent FCachedlibcPath getting freed by thread 2 while thread 1 is
                concatenating it to -Fl below }
              try
                if volatile(FCachedlibcPath)='' then
                  begin
{$endif NO_THREADING}
                    FCachedlibcPath:=s;
{$ifndef NO_THREADING}
                  end;
              finally
                LeaveCriticalsection(FGeneralCriticalSection);
              end;
{$endif NO_THREADING}
            end;
        end
      else
        { make sure we don't access the contents of the string before they've been
          synchronised from the thread that wrote them; the critical section there
          acts as a read/write barrier }
        ReadBarrier;
{$ifdef NO_THREADING}
      Args.Add('-Fl'+FCachedlibcPath);
{$ELSE}      
      Args.Add('-Fl'+volatile(FCachedlibcPath));
{$ENDIF}      
    end;

  // Custom options which are added by dependencies
  AddDependencyTransmittedOptions(Args, APackage);

  // Custom Options
  If (Defaults.HaveOptions) then
    Args.AddStrings(Defaults.Options);

  APackage.ApplyPackageVariantToCompilerOptions(Args);

  If (APackage.HaveOptions) then
    Args.AddStrings(APackage.Options);
  If (ATarget.HaveOptions) then
    Args.AddStrings(ATarget.Options);

  {$ifdef HAS_UNIT_PROCESS}
  // Force the compiler-output to be easy parseable
  if not Verbose then
    args.Add('-viq');
  {$endif}

  // Convert to string
  Result:='';
  for i:=0 to Args.Count-1 do
    Result:=Result+' '+maybequoted(APackage.Dictionary.ReplaceStrings(Args[i]));
  Delete(result,1,1);

  if Defaults.UseEnvironment and assigned(Env) then
    begin
      env.Values['FPCEXTCMD'] := Result;
      result := '!FPCEXTCMD';
      // Make sure that this process' environment variables are passed to the
      // compiler's environment
      for i := 0 to GetEnvironmentVariableCount-1 do
        env.Add(GetEnvironmentString(i));
    end;

  // Add Filename to compile
  result := result + ' ' + AddPathPrefix(APackage,ATarget.TargetSourceFileName);

  Args.Free;
end;


Function TBuildEngine.GetCompiler : String;
Var
  S : String;
begin
  // Cache in FCompiler for speed.
  If (FCompiler='') then
    begin
    FCompiler:=Defaults.Compiler;
    If (ExtractFilePath(FCompiler)='') then
      begin
      S:=ExeSearch(FCompiler,SysUtils.GetEnvironmentVariable('PATH'));
      If (S<>'') then
         FCompiler:=S;
      end;
    end;
  Result:=FCompiler;
end;


procedure TBuildEngine.CreateOutputDir(APackage: TPackage);
Var
  D : String;
  i: integer;
begin
  //create a units directory
  D:=AddPathPrefix(APackage,APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  If not SysDirectoryExists(D) then
    begin
      Log(vlInfo,SInfoCreatingOutputDir,[D]);
      CmdCreateDir(D);
    end;

  //also create a bin directory for programtargets
  For i := 0 to Pred(APackage.Targets.Count) do
    begin
      if APackage.Targets.TargetItems[i].TargetType in (ProgramTargets-[ttExampleProgram]) then
        begin
          D:=AddPathPrefix(APackage,APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS));
          If not SysDirectoryExists(D) then
            begin
              Log(vlInfo,SInfoCreatingOutputDir,[D]);
              CmdCreateDir(D);
            end;
          //do not continue loop, directory is made anyway
          break;
        end;
    end;
end;


Function TBuildEngine.DependencyOK(ADependency : TDependency) : Boolean;
begin
  Result:=(Defaults.CPU in ADependency.CPUs) and (Defaults.OS in ADependency.OSes);
end;

function TBuildEngine.TargetOK(ATarget: TTarget; ACPU: TCPU; AOS: TOS): Boolean;
begin
  if Defaults.SkipCrossPrograms and
     (ATarget.TargetType in ProgramTargets) and
     IsDifferentFromBuild(ACPU, AOS) then
    result := False
  else
    Result:=(ACPU in ATarget.CPUs) and (AOS in ATarget.OSes);
end;

function TBuildEngine.TargetInstallOK(ATarget: TTarget; ACPU: TCPU; AOS: TOS): Boolean;
begin
  result := TargetOK(ATarget, ACPU, AOS) and ATarget.Install;
end;


Function TBuildEngine.PackageOK(APackage : TPackage) : Boolean;
begin
  Result:=(Defaults.CPU in APackage.CPUs) and (Defaults.OS in APackage.OSes);
end;


procedure TBuildEngine.DoBeforeCompile(APackage: TPackage);
begin
  ExecuteCommands(APackage.Commands,caBeforeCompile);
  If Assigned(APackage.BeforeCompile) then
    APackage.BeforeCompile(APackage);
  If Assigned(APackage.BeforeCompileProc) then
    APackage.BeforeCompileProc(APackage);
  // It could be that files that weren't found before are available now.
  ResolveFileNames(APackage,Defaults.CPU,Defaults.OS,false,true);
end;


procedure TBuildEngine.DoAfterCompile(APackage: TPackage);
begin
  If Assigned(APackage.AfterCompile) then
    APackage.AfterCompile(APackage);
  If Assigned(APackage.AfterCompileProc) then
    APackage.AfterCompileProc(APackage);
  ExecuteCommands(APackage.Commands,caAfterCompile);
end;


Function TBuildEngine.NeedsCompile(APackage:TPackage;ATarget: TTarget): Boolean;
Var
  I : Integer;
  D : TDependency;
  T : TTarget;
  OD,OFN,TFN : String;
  CompileReason: String;
begin
  Result:=False;

  // Forced recompile?
  if FForceCompile then
    begin
    Result:=true;
    CompileReason:=SDbgForcedCompile;
    end;

  // For now examples are not compiled at all
  if ATarget.TargetType in [ttExampleUnit, ttExampleProgram] then
    Exit;

  // Files which should not be compiled on this target can not trigger a compile.
  if not TargetOK(ATarget, Defaults.CPU, Defaults.OS) then
    Exit;

  // Check output file
  if not result then
    begin
      if ATarget.TargetType in ProgramTargets then
        OD:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS)
      else
        OD:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
      If (OD<>'') then
        OD:=IncludeTrailingPathDelimiter(OD);
      OFN:=AddPathPrefix(APackage, OD+ATarget.GetOutPutFileName(Defaults.OS));
      Result:=Not FileExists(OFN);
      if Result then
        begin
        CompileReason:=SDbgOutputDoesNotExist;
        Log(vlDebug,SDbgOutputNotYetAvailable,[OFN]);
        end;
    end;

  // Check main source, only if the TargetSourceFileName is found
  If not Result and (ATarget.TargetSourceFileName<>'') then
    begin
      TFN := AddPathPrefix(APackage,ATarget.TargetSourceFileName);
      if FileExists(TFN) then
        Result:=FileNewer(TFN,OFN);
      if Result then
        CompileReason:=SDbgNewerSource;
    end;

  // Check unit and include dependencies
  If not Result then
    begin
      ResolveDependencies(ATarget.Dependencies,ATarget.Collection as TTargets);
      I:=0;
      for i:=0 to ATarget.Dependencies.Count-1 do
        begin
          D:=ATarget.Dependencies[i];
          if (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
            begin
              case D.DependencyType of
                depUnit :
                  begin
                    T:=TTarget(D.Target);
                    If (T=Nil) then
                      Error(SErrDepUnknownTarget,[D.Value, ATarget.Name, APackage.Name]);
                    // If a dependent package is compiled we always need to recompile
                    Log(vldebug, SDbgDependencyOnUnit, [ATarget.Name,T.Name]);
                    Result:=(T.State=tsCompiled);
                    if Result then
                      begin
                      Log(vldebug, SDbgDependencyUnitRecompiled, [T.Name]);
                      CompileReason:=Format(SDbgDependencyRecompiled,[T.Name]);
                      end;
                  end;
                depInclude :
                  begin
                    if D.TargetFileName<>'' then
                      begin
                        TFN:=AddPathPrefix(APackage,D.TargetFileName);
                        Result:=FileNewer(TFN,OFN);
                        if result then
                          CompileReason:=Format(SDbgNewerInclude,[D.TargetFileName]);
                      end;
                  end;
                depPackage :
                  begin
                    log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APackage.Name, d.Value]);
                  end;
              end;
              if result then
                break;
            end;
        end;
    end;

  if result then
    Log(vlDebug,SDbgMustCompile,[ATarget.Name, CompileReason]);
end;


procedure TBuildEngine.Compile(APackage: TPackage; ATarget: TTarget);
Var
  S : String;
  Env : TStrings;
begin
  Log(vlInfo,SInfoCompilingTarget,[ATarget.Name]);
  LogIndent;
  ExecuteCommands(ATarget.Commands,caBeforeCompile);
  If Assigned(ATarget.BeforeCompile) then
    ATarget.BeforeCompile(ATarget);

  if (APackage.BuildMode=bmBuildUnit) and not (ATarget.TargetType in [ttProgram,ttSharedLibrary,ttExampleProgram]) then
    begin
      APackage.FBUTarget.Dependencies.AddUnit(ATarget.Name).FTargetFileName:=ATarget.TargetSourceFileName;
    end
  else
    begin
      if Defaults.UseEnvironment then
        begin
          Env := TStringList.Create;
          try
            S:=GetCompilerCommand(APackage,ATarget,Env);
            ExecuteCommand(GetCompiler,S,Env);
          finally
            Env.Free;
          end;
        end
      else
        begin
          S:=GetCompilerCommand(APackage,ATarget,nil);
          ExecuteCommand(GetCompiler,S,nil);
        end;
      If Assigned(ATarget.AfterCompile) then
        ATarget.AfterCompile(ATarget);
      ExecuteCommands(ATarget.Commands,caAfterCompile);
    end;
  LogUnIndent;
end;


procedure TBuildEngine.CompileDependencies(APackage:TPackage; ATarget: TTarget);
Var
  I : Integer;
  T : TTarget;
  D : TDependency;
begin
  Log(vlDebug, Format(SDbgCompilingDependenciesOfTarget, [ATarget.Name]));
  LogIndent;
  For I:=0 to ATarget.Dependencies.Count-1 do
    begin
      D:=ATarget.Dependencies[i];
      if (D.DependencyType=depPackage) then
        log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APackage.Name, d.Value])
      else if (D.DependencyType=depUnit) and
         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
        begin
          T:=TTarget(D.Target);
          if Assigned(T) and (T<>ATarget) then
            begin
              if TargetOK(T, Defaults.CPU, Defaults.OS) then
                begin
                  // We don't need to compile implicit units, they are only
                  // used for dependency checking
                  if (T.TargetType<>ttImplicitUnit) then
                    begin
                      case T.State of
                        tsNeutral :
                          MaybeCompile(APackage,T);
                        tsConsidering :
                          Log(vlWarning,SWarnCircularTargetDependency,[ATarget.Name,T.Name]);
                      end;
                    end;
                end
              else
                Log(vlWarning, Format(SWarnDepUnitNotFound, [T.Name, MakeTargetString(Defaults.CPU,Defaults.OS)]));
            end
          else
            Error(SErrDepUnknownTarget,[D.Value, ATarget.Name, APackage.Name]);
        end;
    end;
  LogUnIndent;
end;


procedure TBuildEngine.MaybeCompile(APackage: TPackage; ATarget: TTarget);
begin
  if ATarget.State<>tsNeutral then
    Error(SErrInvalidState,[ATarget.Name]);
  Log(vlDebug, Format(SDbgConsideringTarget, [ATarget.Name]));
  LogIndent;
  ATarget.FTargetState:=tsConsidering;
  ResolveDependencies(ATarget.Dependencies,ATarget.Collection as TTargets);
  CompileDependencies(APackage, ATarget);
  if NeedsCompile(APackage, ATarget) then
    begin
      Compile(APackage,ATarget);
      ATarget.FTargetState:=tsCompiled;
    end
  else
    ATarget.FTargetState:=tsNoCompile;
  LogUnIndent;
end;


function TBuildEngine.NeedsCompile(APackage: TPackage): Boolean;
Var
  I : Integer;
  P : TPackage;
  D : TDependency;
  CompileReason: string;
begin
  Result:=False;

  // Forced recompile?
  if FForceCompile then
    begin
    Result:=true;
    CompileReason:=SDbgForcedCompile;
    end;

  // Recompile because of Package Dependencies?
  if not Result then
    begin
       I:=0;
       For I:=0 to APackage.Dependencies.Count-1 do
         begin
           D:=APackage.Dependencies[i];
           if (D.DependencyType=depPackage) and
              (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
             begin
               P:=TPackage(D.Target);
               if Assigned(P) then
                 begin
                   Result:=(P.State=tsCompiled);
                   if Result then
                     begin
                     break;
                     CompileReason:=Format(SDbgPackageDepRecompiled,[P.Name]);
                     end;
                 end;
             end;
         end;
    end;

  // Recompile a Target of this package?
  If Not Result then
    begin
      GPathPrefix := APackage.Directory;
      try
        for i:=0 to APackage.Targets.Count-1 do
          begin
            Result:=NeedsCompile(APackage,APackage.Targets.TargetItems[i]);
            if Result then
              begin
              break;
              CompileReason:=Format(SDbgTargetHasToBeCompiled,[APackage.Targets.TargetItems[i].Name]);
              end;
          end;
      Finally
        GPathPrefix := '';
      end;
    end;

  if result then
    Log(vlDebug,SDbgMustCompile,[APackage.Name, CompileReason]);
end;


function TBuildEngine.CheckExternalPackage(Const APackageName, ForPackageName : String; ErrorOnFailure: boolean):TPackage;
var
  S : String;
  F : String;
  I : Integer;
begin
  // Already checked?
  I:=ExternalPackages.IndexOfName(APackageName);
  if I<>-1 then
    begin
      result:=ExternalPackages.PackageItems[I];
      exit;
    end;
  // Create new external package
  Result:=ExternalPackages.AddPackage(APackageName);
  Result.FTargetState:=tsNotFound;
  // Load unit config
  S:=GetUnitDir(Result);
  if S<>'' then
    begin
      Log(vldebug, SDbgExternalDependency, [APackageName,S]);
      // Load unit config if it exists
      F:=result.UnitConfigFileName;
      if (F<>'') then
        begin
          Log(vlDebug, Format(SDbgLoading, [F]));
          Result.LoadUnitConfigFromFile(F);
          result.SetDefaultPackageVariant;
          result.UnitDir:=ConcatPaths([result.UnitDir,Result.GetPackageUnitInstallDir(Defaults.CPU, Defaults.OS)]);
        end;
      // Check recursive implicit dependencies
      CompileDependencies(Result);
    end
  else if ErrorOnFailure then
    Error(SErrDependencyNotFound,[APackageName,ForPackageName]);
end;


procedure TBuildEngine.CompileDependencies(APackage: TPackage);
Var
  I : Integer;
  P : TPackage;
  D : TDependency;
begin
  For I:=0 to APackage.Dependencies.Count-1 do
    begin
      D:=APackage.Dependencies[i];
      if (D.DependencyType=depPackage) and
         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
        begin
          P:=TPackage(D.Target);
          If Assigned(P) then
            begin
              if (Defaults.CPU in P.CPUs) and (Defaults.OS in P.OSes) then
                begin
                  case P.State of
                    tsNeutral :
                      MaybeCompile(P);
                    tsConsidering :
                      Log(vlWarning,SWarnCircularPackageDependency,[APackage.Name,P.Name]);
                  end;
                end
              else
                Log(vlWarning,SWarnDependOnOtherPlatformPackage,[APackage.Name, D.Value, MakeTargetString(Defaults.CPU, Defaults.OS)]);
            end
          else
            begin
              D.Target:=CheckExternalPackage(D.Value, APackage.Name, true);
              P:=TPackage(D.Target);
            end;
          if (D.RequireChecksum<>$ffffffff) and (D.RequireChecksum<>0) and
             (P.InstalledChecksum<>$ffffffff) and
             (P.InstalledChecksum<>D.RequireChecksum) then
            Log(vlWarning,SDbgPackageChecksumChanged,[APackage.Name, P.Name]);
          APackage.InheritPackageVariantsFromDependency(P);
        end;
    end;
end;

function TBuildEngine.CheckDependencies(APackage: TPackage; ErrorOnFailure: boolean): TCheckDependencyResult;
Var
  I : Integer;
  P : TPackage;
  D : TDependency;
begin
  result := cdAvailable;
  For I:=0 to APackage.Dependencies.Count-1 do
    begin
      D:=APackage.Dependencies[i];
      if (D.DependencyType=depPackage) and
         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
        begin
          P:=TPackage(D.Target);
          If Assigned(P) then
            begin
              if (Defaults.CPU in P.CPUs) and (Defaults.OS in P.OSes) then
                begin
                  case P.State of
                    tsNeutral :
                      result := cdNotYetAvailable;
                    tsConsidering :
                      Log(vlWarning,SWarnCircularPackageDependency,[APackage.Name,P.Name]);
                  end;
                end
              else
                Log(vlWarning,SWarnDependOnOtherPlatformPackage,[APackage.Name, D.Value, MakeTargetString(Defaults.CPU, Defaults.OS)]);
            end
          else
            begin
              D.Target:=CheckExternalPackage(D.Value, APackage.Name, ErrorOnFailure);
              P:=TPackage(D.Target);
            end;
          if (D.RequireChecksum<>$ffffffff) and
             (P.InstalledChecksum<>$ffffffff) and
             (P.InstalledChecksum<>D.RequireChecksum) then
            Log(vldebug,SDbgPackageChecksumChanged,[APackage.Name, P.Name]);
          APackage.InheritPackageVariantsFromDependency(P);
        end;
    end;
end;


procedure TBuildEngine.Compile(APackage: TPackage);
Var
  T : TTarget;
  I : Integer;
  Cmd: string;
  cmdOpts: string;
  sFPDocFormat: string;
  IFPDocFormat: TFPDocFormat;
  d: integer;
  UC: string;
  dep: TDependency;
  RegenerateUnitconfigFile: boolean;
  BUName: string;

  procedure CompileBuildUnit;
  var
    I: Integer;
    T: TTarget;
    L: TStrings;
    F: Text;
    CompilationFailed: Boolean;

  begin
    if (APackage.FBUTarget.Dependencies.Count>0) then
      begin
        Log(vldebug, Format(SDbgGenerateBuildUnit, [APackage.FBUTarget.Name]));
        system.Assign(F,AddPathPrefix(APackage,APackage.FBUTarget.FTargetSourceFileName));
        Rewrite(F);
        writeln(F,'unit ' + APackage.FBUTarget.Name +';');
        writeln(F,'interface');
        writeln(F,'uses');
        for i := 0 to APackage.FBUTarget.Dependencies.Count-1 do
          begin
            if i<>0 then
              write(F,',');
            writeln(F,APackage.FBUTarget.Dependencies.Dependencies[i].Value);
          end;
        writeln(F,';');
        writeln(F,'implementation');
        writeln(F,'end.');

        system.close(F);

        APackage.FBuildMode:=bmOneByOne;
        Compilationfailed:=false;
        try
          try
            Compile(APackage,APackage.FBUTarget);
          except
            on E: Exception do
              begin
                Log(vlError,E.Message);
                Compilationfailed:=true;
              end;
          end;
        finally
          if CompilationFailed then
            begin
              Log(vlDebug,Format(SDbgBuildUnitFailure,[APackage.FBUTarget.FTargetSourceFileName]));
              // Raise failure exception again.
              Error(SErrBuildUnitCompilation,[APackage.FBUTarget.FTargetSourceFileName]);
            end
          else
            begin
            // Delete temporary build-unit files
            L := TStringList.Create;
            try
              APackage.FBUTarget.GetCleanFiles(L,IncludeTrailingPathDelimiter(AddPathPrefix(APackage,APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS))),'',Defaults.CPU,Defaults.OS);
              L.Add(AddPathPrefix(APackage,APackage.FBUTarget.SourceFileName));
              CmdDeleteFiles(L);
            finally
              L.Free;
            end;
          end;
        end;
      end;

    For I:=0 to APackage.Targets.Count-1 do
      begin
        T:=APackage.Targets.TargetItems[i];
        if (T.TargetType = ttUnit) and (TargetOK(T, Defaults.CPU, Defaults.OS)) then
          begin
            If Assigned(T.AfterCompile) then
              T.AfterCompile(T);
            ExecuteCommands(T.Commands,caAfterCompile);
          end
      end;
  end;

  procedure ProcessCompileTarget;
  begin
    if TargetOK(T, Defaults.CPU, Defaults.OS) then
      begin
        if T.State=tsNeutral then
          MaybeCompile(APackage,T);
        // If a target is compiled, re-generate the UnitConfigFile
        if T.FTargetState<>tsNoCompile then
          RegenerateUnitconfigFile:= True;
      end
    else
      begin
        if not(Defaults.CPU in T.CPUs) then
          Log(vldebug, Format(SDbgSkippingTargetWrongCPU, [T.Name, CPUsToString(T.CPUs)]));
        if not(Defaults.OS in T.OSes) then
          Log(vldebug, Format(SDbgSkippingTargetWrongOS, [T.Name, OSesToString(T.OSes)]));
      end;
  end;

begin
  cmdOpts := '';

  log(vlWarning,SWarnStartCompilingPackage,[APackage.Name, Defaults.Target]);

  case Defaults.BuildMode of
    bmOneByOne:  begin
                   if (bmOneByOne in APackage.SupportBuildModes) then
                     APackage.FBuildMode:=bmOneByOne
                   else if bmBuildUnit in APackage.SupportBuildModes then
                     begin
                       log(vlInfo,SInfoFallbackBuildmodeBU);
                       APackage.FBuildMode:=bmBuildUnit;
                     end
                   else
                     raise exception.create(SErrUnsupportedBuildmode);
                 end;
    bmBuildUnit: begin
                   // When bmBuildUnit is supported by the package use a buildunit.
                   // Unless there is only one target and bmOneByOne is also supported
                   if (bmBuildUnit in APackage.SupportBuildModes) and
                      not ((APackage.Targets.Count=1) and (bmOneByOne in APackage.SupportBuildModes)) then
                     APackage.FBuildMode:=bmBuildUnit
                   else if bmOneByOne in APackage.SupportBuildModes then
                     begin
                       log(vlInfo,SInfoFallbackBuildmode);
                       APackage.FBuildMode:=bmOneByOne
                     end
                   else
                     raise exception.create(SErrUnsupportedBuildmode);
                 end;
  end;

  GPathPrefix:=APackage.Directory;
  Try
    CreateOutputDir(APackage);
    AddPackageMacrosToDictionary(APackage, APackage.Dictionary);
    DoBeforeCompile(APackage);
    RegenerateUnitconfigFile:=False;
    if APackage.BuildMode=bmBuildUnit then
      begin
        APackage.FBUTargets := TTargets.Create(TTarget);
        if (Defaults.BuildOS in AllLimit83fsOses) or
           (Defaults.OS in AllLimit83fsOses) then
          BUName := 'BUnit.pp'
        else
          BUName := 'BuildUnit_'+StringReplace(APackage.Name,'-','_',[rfReplaceAll])+'.pp';
        APackage.FBUTarget := APackage.FBUTargets.AddUnit(BUName);
        APackage.FBUTarget.FTargetSourceFileName := APackage.FBUTarget.SourceFileName;
      end;
    For I:=0 to APackage.Targets.Count-1 do
      begin
        T:=APackage.Targets.TargetItems[i];
        case T.TargetType of
        ttUnit:
          begin
            ProcessCompileTarget;
          end;
        ttSharedLibrary,
        ttProgram:
          begin // do nothing, are compiled later
          end;
        ttFPDoc:
          begin
            for d := 0 to T.Dependencies.Count - 1 do
            begin
              dep := TDependency(T.Dependencies[d]);

              //add unit dependencies
              if dep.DependencyType = depUnit then
                cmdOpts := cmdOpts + ' --input=' + AddPathPrefix(APackage,dep.Value);
            end;

            //check if a documentation target is given
            cmdOpts := cmdOpts + ' --input=' + AddPathPrefix(APackage,T.Directory + T.Name + T.Extension) + ' --descr='+ T.XML;
          end
        else
          log(vldebug, SDbgTargetIsNotAUnitOrProgram,[T.Name]);
        end;
      end;

    if APackage.BuildMode=bmBuildUnit then
      CompileBuildUnit;

    FreeAndNil(APackage.FBUTarget);

    For I:=0 to APackage.Targets.Count-1 do
      begin
        T:=APackage.Targets.TargetItems[i];
        if T.TargetType in [ttProgram,ttSharedLibrary] then
          begin
            ProcessCompileTarget;
          end;
      end;

    if RegenerateUnitconfigFile then
      begin
        UC:=AddPathPrefix(APackage, APackage.GetUnitConfigOutputFilename(Defaults.CPU,Defaults.OS));
        Log(vlInfo, Format(SDbgGenerating, [UC]));
        APackage.SaveUnitConfigToFile(UC,Defaults.CPU,Defaults.OS);
      end;

    //compile documentation, because options were found
    if cmdOpts <> '' then
    begin
      //append package name
      cmdOpts := cmdOpts + ' --package=' + APackage.Name;

      for IFPDocFormat:=Low(TFPDocFormat) to High(TFPDocFormat) do
      begin
        if IFPDocFormat in APackage.FPDocFormat then
        begin
          //prepend output format
          case IFPDocFormat of
            ffHtml:      sFPDocFormat := '--format=html --output=' + AddPathPrefix(APackage,Defaults.FPDocOutputDir);
            ffHtm:       sFPDocFormat := '--format=htm --output=' + AddPathPrefix(APackage,Defaults.FPDocOutputDir);
            ffXHtml:     sFPDocFormat := '--format=xhtml --output=' + AddPathPrefix(APackage,Defaults.FPDocOutputDir);
            ffLaTex:     sFPDocFormat := '--format=latex --output=' + AddPathPrefix(APackage,Defaults.FPDocOutputDir) + APackage.Name + '.tex';
            ffXMLStruct: sFPDocFormat := '--format=xml-struct --output=' + AddPathPrefix(APackage,Defaults.FPDocOutputDir);
            ffChm:       sFPDocFormat := '--format=chm --output=' + AddPathPrefix(APackage,Defaults.FPDocOutputDir) + APackage.Name + '.chm';
          end;

          //execute fpdoc
          Cmd:=ExeSearch('fpdoc',SysUtils.GetEnvironmentvariable('PATH'));
          if Cmd = '' then Cmd := 'fpdoc';
          ExecuteProcess(Cmd, sFPDocFormat + cmdOpts);
        end;
      end;
    end;

    DoAfterCompile(APackage);
  Finally
    GPathPrefix:='';
  end;
  inc(FProgressCount);
  if FProgressMax>0 then
    log(vlWarning,SWarnCompilingPackagecompleteProgress,[(FProgressCount)/FProgressMax * 100, APackage.Name])
  else
    log(vlWarning,SWarnCompilingPackagecomplete,[APackage.Name]);
end;

procedure TBuildEngine.MaybeCompile(APackage: TPackage);
begin
  if ReadyToCompile(APackage) then
    begin
      Compile(APackage);
      APackage.FTargetState:=tsCompiled;
    end;
end;

function TBuildEngine.ReadyToCompile(APackage: TPackage) : Boolean;
begin
  result := False;
  if APackage.State in [tsCompiled, tsNoCompile] then
    begin
      Log(vlInfo,SInfoPackageAlreadyProcessed,[APackage.Name]);
      Exit;
    end;
  if APackage.State<>tsNeutral then
    Error(SErrInvalidState,[APackage.Name]);
  Log(vlDebug,SDbgConsideringPackage,[APackage.Name]);
  LogIndent;
  if Defaults.ThreadsAmount=-1 then
    APackage.FTargetState:=tsConsidering;
  ResolveDependencies(APackage.Dependencies,(APackage.Collection as TPackages));
  // When multiple threads are used, delay the compilation of the package when
  // there are unsolved dependencies. When no threads are used, compile all
  // dependencies.
  if Defaults.ThreadsAmount=-1 then
    CompileDependencies(APackage)
  else if CheckDependencies(APackage, true)=cdNotYetAvailable then
    begin
      log(vlInfo,'Delaying package '+apackage.name);
      result := False;
      Exit;
    end;
  APackage.SetDefaultPackageVariant;

  ResolveFileNames(APackage,Defaults.CPU,Defaults.OS,True,False);
  If NeedsCompile(APackage) then
    result := True
  else
    begin
      APackage.FTargetState:=tsNoCompile;
      inc(FProgressCount);
    end;
  LogUnIndent;
end;


Function TBuildEngine.InstallPackageFiles(APAckage : TPackage; tt : TTargetTypes; Const Dest : String; Const InstallMode: TInstallMode):Boolean;
Var
  List : TStringList;
begin
  Result:=False;
  List:=TStringList.Create;
  Try
    APackage.GetInstallFiles(List,tt,Defaults.CPU, Defaults.OS);
    if (List.Count>0) then
      begin
        Result:=True;
        case InstallMode of
          imInstall:   CmdCopyFiles(List,Dest,APackage);
          imUnInstall: CmdDeleteDestFiles(List,Dest);
        end;
      end;
  Finally
    List.Free;
  end;
end;

function TBuildEngine.GetUnitConfigFilesInstallDir(ABaseDir: string): String;
begin
  result := FixPath(ABaseDir)+Defaults.UnitConfigFilesInstallDir;
end;

procedure TBuildEngine.InstallUnitConfigFile(APAckage: TPackage; const Dest: String);
Var
  List : TStringList;
  ConfigFileName: String;
  ConfigFileContent: TStrings;
  Index: integer;
begin
  ConfigFileName:=APackage.GetUnitConfigOutputFilename(Defaults.CPU,Defaults.OS);
  List:=TStringList.Create;
  Try
    if Defaults.FPUnitSourcePath<>'' then
      begin
        ConfigFileContent := TStringList.Create;
        try
          ConfigFileContent.LoadFromFile(AddPathPrefix(APAckage, ConfigFileName));
          if Defaults.FPUnitSourcePath='0' then
            begin
              Index := ConfigFileContent.IndexOfName(KeySourcePath);
              if Index > -1 then
                ConfigFileContent.Delete(Index)
            end
          else
            ConfigFileContent.Values[KeySourcePath] := Defaults.FPUnitSourcePath;
          ConfigFileContent.SaveToFile(AddPathPrefix(APAckage, ConfigFileName));
        finally
          ConfigFileContent.Free;
        end;
      end;
    List.Values[ConfigFileName] := APAckage.Name + FpmkExt;
    CmdCopyFiles(List,Dest,APackage);
  Finally
    List.Free;
  end;
end;

function TBuildEngine.InstallPackageSourceFiles(APAckage : TPackage; stt : TSourceTypes; ttt : TTargetTypes; Const Dest : String; Const InstallMode: TInstallMode): Boolean;
Var
  List : TStringList;
begin
  Result:=False;
  List:=TStringList.Create;
  Try
    APackage.GetInstallSourceFiles(List,stt,ttt);
    if (List.Count>0) then
      begin
        Result:=True;
        case InstallMode of
          imInstall:   CmdCopyFiles(List,Dest,APackage);
          imUnInstall: CmdDeleteDestFiles(List,Dest);
        end;
      end;
  Finally
    List.Free;
  end;
end;


procedure TBuildEngine.DoBeforeInstall(APackage: TPackage);
begin
  ExecuteCommands(APackage.Commands,caBeforeInstall);
  If Assigned(APackage.BeforeInstall) then
    APackage.BeforeInstall(APackage);
  If Assigned(APackage.BeforeInstallProc) then
    APackage.BeforeInstallProc(APackage);
end;


procedure TBuildEngine.DoAfterInstall(APackage: TPackage);
begin
  If Assigned(APackage.AfterInstall) then
    APackage.AfterInstall(APackage);
  If Assigned(APackage.AfterInstallProc) then
    APackage.AfterInstallProc(APackage);
  ExecuteCommands(APackage.Commands,caAfterInstall);
end;


procedure TBuildEngine.Install(APackage: TPackage; AnArchiveFiles: boolean);
Var
  UC,D : String;
  B : Boolean;
begin
  If (Apackage.State<>tsCompiled) then
    MaybeCompile(APackage);
  Log(vlCommand,SInfoInstallingPackage,[APackage.Name]);
  if AnArchiveFiles then
    FinishArchive(APackage);
  try
    If (APackage.Directory<>'') then
      GPathPrefix := APackage.Directory;
    if AnArchiveFiles then
      begin
        FOnCopyFile:=@AddFileToArchive;
        FOnFinishCopy:=@FinishArchive;
      end;
    DoBeforeInstall(APackage);
    // units
    B:=false;
    AddPackageMacrosToDictionary(APackage, APackage.Dictionary);
    GlobalDictionary.AddVariable('unitinstalldir', FixPath(APackage.Dictionary.ReplaceStrings(Defaults.UnitInstallDir), False));
    GlobalDictionary.AddVariable('packageunitinstalldir',APackage.GetPackageUnitInstallDir(Defaults.CPU,Defaults.OS));

    D:=FixPath(Defaults.Prefix,true);
    // This is to install the TPackage.Installfiles, which are not related to any
    // target
    if InstallPackageFiles(APackage,[],D, imInstall) then
      B:=true;
    D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.UnitInstallDir), True)+ExcludeLeadingPathDelimiter(APackage.GetPackageUnitInstallDir(Defaults.CPU,Defaults.OS));
    if InstallPackageFiles(APackage,[ttUnit, ttImplicitUnit],D, imInstall) then
      B:=true;
    // By default do not install the examples. Maybe add an option for this later
    //if InstallPackageFiles(APAckage,ttExampleUnit,D) then
    //  B:=true;
    // Unit (dependency) configuration if there were units installed
    D:=FixPath(APackage.Dictionary.ReplaceStrings(GetUnitConfigFilesInstallDir(Defaults.BaseInstallDir)), True);
    if B then
      InstallUnitConfigFile(APackage,D);
    // Programs
    D:=IncludeTrailingPathDelimiter(Defaults.BinInstallDir);
    InstallPackageFiles(APAckage,[ttProgram],D, imInstall);
    // Shared libraries
    D:=IncludeTrailingPathDelimiter(Defaults.LibInstallDir);
    InstallPackageFiles(APAckage,[ttSharedLibrary],D, imInstall);
    //InstallPackageFiles(APAckage,ttExampleProgram,D);
    // Documentation
    D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.DocInstallDir), True);
    InstallPackageSourceFiles(APackage,[stDoc],[],D, imInstall);
    // Examples
    if Defaults.InstallExamples then
      begin
        D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.ExamplesInstallDir), True);
        InstallPackageSourceFiles(APackage,[stExample],[ttExampleProgram,ttExampleUnit],D, imInstall);
      end;
    // Done.
    APackage.FTargetState:=tsInstalled;
    DoAfterInstall(APackage);
    if AnArchiveFiles then
      begin
      FOnCopyFile:=nil;
      FOnFinishCopy:=nil;
      end;
  Finally
    If (APackage.Directory<>'') then
      GPathPrefix := '';
  end;
end;


procedure TBuildEngine.UnInstall(APackage: TPackage);
Var
  D : String;
begin
  CheckDependencies(APackage, False);
  ResolvePackagePaths(APackage);
  APackage.SetDefaultPackageVariant;
  If (Apackage.State<>tsInstalled) then
    begin
    Error(SErrorPkgNotInstalled,[APackage.Name]);
    exit;
    end;
  Log(vlCommand,SInfoUnInstallingPackage,[APackage.Name]);

  //DoBeforeUnInstall(APackage);

  // units
  AddPackageMacrosToDictionary(APackage, APackage.Dictionary);
  GlobalDictionary.AddVariable('unitinstalldir', FixPath(APackage.Dictionary.ReplaceStrings(Defaults.UnitInstallDir), False));
  GlobalDictionary.AddVariable('packageunitinstalldir',APackage.GetPackageUnitInstallDir(Defaults.CPU,Defaults.OS));

  D:=FixPath(Defaults.Prefix,true);
  // This is to uninstall the TPackage.Installfiles, which are not related to any
  // target
  InstallPackageFiles(APackage,[],D,imUnInstall);
  D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.UnitInstallDir), True)+APackage.GetPackageUnitInstallDir(Defaults.CPU,Defaults.OS);
  InstallPackageFiles(APackage,[ttUnit, ttImplicitUnit],D, imUnInstall);
  SysDeleteDirectory(D);

  // Unit (dependency) configuration
  D:=FixPath(APackage.Dictionary.ReplaceStrings(GetUnitConfigFilesInstallDir(Defaults.BaseInstallDir)), True);
  SysDeleteFile(D+APackage.Name+FpmkExt);

  // Programs
  D:=IncludeTrailingPathDelimiter(Defaults.BinInstallDir);
  InstallPackageFiles(APAckage,[ttProgram],D, imUnInstall);
  SysDeleteDirectory(D);
  // Libraries
  D:=IncludeTrailingPathDelimiter(Defaults.LibInstallDir);
  InstallPackageFiles(APAckage,[ttSharedLibrary],D, imUnInstall);
  SysDeleteDirectory(D);
  // Documentation
  D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.DocInstallDir), True);
  InstallPackageSourceFiles(APackage,[stDoc],[],D, imUnInstall);
  SysDeleteDirectory(D);
  // Examples
  if Defaults.InstallExamples then
    begin
      D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.ExamplesInstallDir), True);
      InstallPackageSourceFiles(APackage,[stExample],[ttExampleProgram,ttExampleUnit],D, imUnInstall);
      SysDeleteDirectory(D);
    end;
  // Done.
  APackage.FTargetState:=tsNeutral;

  //DoAfterUnInstall(APackage);
end;


procedure TBuildEngine.DoBeforeArchive(APackage: TPackage);
begin
  ExecuteCommands(APackage.Commands,caBeforeArchive);
  If Assigned(APackage.BeforeArchive) then
    APackage.BeforeArchive(APackage);
  If Assigned(APackage.BeforeArchiveProc) then
    APackage.BeforeArchiveProc(APackage);
end;


procedure TBuildEngine.DoAfterArchive(APackage: TPackage);
begin
  If Assigned(APackage.AfterArchive) then
    APackage.AfterArchive(APackage);
  If Assigned(APackage.AfterArchiveProc) then
    APackage.AfterArchiveProc(APackage);
  ExecuteCommands(APackage.Commands,caAfterArchive);
end;


procedure TBuildEngine.Archive(APackage: TPackage);
Var
  L : TStringList;
  A : String;
  i: integer;
  ICPU : TCPU;
  IOS  : TOS;
{$ifdef HAS_UNIT_ZIPPER}
  ZipFile: TZipper;
{$endif HAS_UNIT_ZIPPER}
begin
  A:=Defaults.ZipPrefix + APackage.FileName + MakeZipSuffix(cpuNone, osNone) + ZipExt;
  Log(vlInfo,SInfoArchivingPackage,[APackage.Name,A]);
  try
    If (APackage.Directory<>'') then
      GPathPrefix := APackage.Directory;
    DoBeforeArchive(Apackage);
    AddPackageMacrosToDictionary(APackage, APackage.Dictionary);
    L:=TStringList.Create;
    L.Sorted:=true;
    L.Duplicates:=dupIgnore;
    Try
      // Add fpmake.pp & manifest.xml always
      L.Add(FPMakePPFile);
      L.Add(ManifestFile);
      //get all files from all targets
      for ICPU:=Low(TCPU) to high(TCPU) do
        for IOS:=Low(TOS) to high(TOS) do
          if OSCPUSupported[IOS,ICPU] then
            begin
              // Make sure that the package is resolved for each target
              ClearResolvedFileNames(APackage);
              ResolveFileNames(APackage,ICPU,IOS,false);
              APackage.GetArchiveFiles(L, ICPU, IOS);
            end;
      //from sources
      APackage.GetArchiveSourceFiles(L);

      //show all files
      for i := 0 to L.Count-1 do
        Log(vlDebug, Format(SDbgArchivingFile, [L[i]]));

{$ifdef HAS_UNIT_ZIPPER}
      if not Assigned(ArchiveFilesProc) then
        begin
          ZipFile := TZipper.Create;
          try
            ZipFile.FileName:=A;
            A := APackage.Dictionary.ReplaceStrings(Defaults.FPrefix);
            if A <> '' then
              A:=IncludeTrailingPathDelimiter(A);
            for i := 0 to L.Count-1 do
              begin
                ZipFile.Entries.AddFileEntry(AddPathPrefix(APackage, L[i]), A+L[i]);
              end;
            ZipFile.ZipAllFiles;
          finally
            ZipFile.Free;
          end;
        end
      else
{$endif HAS_UNIT_ZIPPER}
        CmdArchiveFiles(L,A);
    Finally
      L.Free;
    end;
    DoAfterArchive(Apackage);
  Finally
    If (APackage.Directory<>'') then
      GPathPrefix := '';
  end;
end;


procedure TBuildEngine.DoBeforeClean(APackage: TPackage);
begin
  ExecuteCommands(APackage.Commands,caBeforeClean);
  If Assigned(APackage.BeforeClean) then
    APackage.BeforeClean(APackage);
  If Assigned(APackage.BeforeCleanProc) then
    APackage.BeforeCleanProc(APackage);
end;


procedure TBuildEngine.DoAfterClean(APackage: TPackage);
begin
  If Assigned(APackage.AfterClean) then
    APackage.AfterClean(APackage);
  If Assigned(APackage.AfterInstallProc) then
    APackage.AfterCleanProc(APackage);
  ExecuteCommands(APackage.Commands,caAfterClean);
end;


procedure TBuildEngine.Clean(APackage: TPackage; AllTargets: boolean);
var
  ACPU: TCpu;
  AOS: TOS;
  DirectoryList : TStringList;
begin
  if not AllTargets and (not(Defaults.OS in APackage.OSes) or
     not (Defaults.CPU in APackage.CPUs))  then
    exit;
  Log(vlInfo,SInfoCleaningPackage,[APackage.Name]);
  try
    If (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    // Check for inherited options (packagevariants) from other packages
    if (Defaults.OS in APackage.OSes) and (Defaults.CPU in APackage.CPUs) then
      begin
        ResolveDependencies(APackage.Dependencies, (APackage.Collection as TPackages));
        CheckDependencies(APackage, False);
        APackage.SetDefaultPackageVariant;
      end;
    DoBeforeClean(Apackage);
    AddPackageMacrosToDictionary(APackage, APackage.Dictionary);
    if AllTargets then
      begin
        // Remove the unit-and bin-directories completely. This is safer in case of files
        // being renamed and such. See also bug 19655
        DirectoryList := TStringList.Create;
        try
          for ACPU:=low(TCpu) to high(TCpu) do if ACPU<>cpuNone then
            for AOS:=low(TOS) to high(TOS) do if AOS<>osNone then
              begin
                if OSCPUSupported[AOS,ACPU] and (AOS in APackage.OSes) and
                   (ACPU in APackage.CPUs) then
                  begin
                    // First perform a normal clean, to be sure that all files
                    // which are not in the units- or bin-dir are cleaned. (like
                    // the .fpm file)
                    Clean(APackage, ACPU, AOS);
                    DirectoryList.Add(ExtractFileDir(APackage.GetUnitsOutputDir(ACPU,AOS)));
                    DirectoryList.Add(ExtractFileDir(APackage.GetBinOutputDir(ACPU,AOS)));
                  end;
              end;
          CmdRemoveTrees(DirectoryList);
        finally
          DirectoryList.Free;
        end;
      end
    else
      Clean(APackage, Defaults.CPU, Defaults.OS);
    DoAfterClean(Apackage);
  Finally
    log(vlInfo, SInfoCleanPackagecomplete, [APackage.Name]);
    If (APackage.Directory<>'') then
      EnterDir('');
  end;
end;

procedure TBuildEngine.Clean(APackage: TPackage; ACPU: TCPU; AOS: TOS);
Var
  List,List2 : TStringList;
  DirectoryList : TStringList;
  RemainingList : TStrings;
  i : longint;
begin
  List:=TUnsortedDuplicatesStringList.Create;
  List.Duplicates:=DupIgnore;
  try
    List.Add(APackage.GetUnitConfigOutputFilename(ACPU,AOS));
    APackage.GetCleanFiles(List,ACPU,AOS);
    if (List.Count>0) then
      begin
      CmdDeleteFiles(List);
      DirectoryList:=TUnsortedDuplicatesStringList.Create;
      DirectoryList.Duplicates:=DupIgnore;
      try
        GetDirectoriesFromFilelist(List,DirectoryList);
        CmdRemoveDirs(DirectoryList);

        DirectoryList.Clear;
        if DirectoryExists(APackage.GetBinOutputDir(ACPU,AOS)) then
          DirectoryList.Add(APackage.GetBinOutputDir(ACPU,AOS));
        if DirectoryExists(APackage.GetUnitsOutputDir(ACPU,AOS)) then
          DirectoryList.Add(APackage.GetUnitsOutputDir(ACPU,AOS));
        CmdRemoveDirs(DirectoryList);

        DirectoryList.Clear;

        { force directory removal for units and bin dir if it ends with /$fpc_target }
        if DirectoryExists(APackage.GetBinOutputDir(ACPU,AOS)) and
           (MakeTargetString(ACPU,AOS)=ExtractFileName(ExcludeTrailingPathDelimiter(APackage.GetBinOutputDir(ACPU,AOS)))) then
          begin
            Installer.Log(vlWarning,Format(SWarnRemovedNonEmptyDirectory,[APackage.Directory+APackage.GetBinOutputDir(ACPU,AOS)]));
            DirectoryList.Add(APackage.GetBinOutputDir(ACPU,AOS));
            RemainingList := TStringList.Create;
            List2:=TStringList.Create;
            SearchFiles(AllFilesMask, APackage.GetBinOutputDir(ACPU,AOS), true, RemainingList);
            for i:=0 to RemainingList.Count-1 do
              begin
                if ExtractFileExt(Remaininglist[i])=PPUExt then
                  Installer.log(vlDebug,format('File %s still present, add corresponding entry to fpmake',[RemainingList[i]]))
                else
                  Installer.log(vlDebug,format('File %s still present',[RemainingList[i]]));
                List2.Add(IncludeTrailingPathDelimiter(APackage.GetUnitsOutputDir(ACPU,AOS))+Remaininglist[i]);
              end;
            CmdDeleteFiles(List2);
            List2.Free;
            RemainingList.Free;
            CmdRemoveTrees(DirectoryList);
            DirectoryList.Clear;
          end;
        if DirectoryExists(APackage.GetUnitsOutputDir(ACPU,AOS)) and
           (MakeTargetString(ACPU,AOS)=ExtractFileName(ExcludeTrailingPathDelimiter(APackage.GetUnitsOutputDir(ACPU,AOS)))) then
          begin
            Installer.Log(vlWarning,Format(SWarnRemovedNonEmptyDirectory,[APackage.Directory+APackage.GetUnitsOutputDir(ACPU,AOS)]));
            DirectoryList.Add(APackage.GetUnitsOutputDir(ACPU,AOS));
            RemainingList := TStringList.Create;
            List2:=TStringList.Create;
            SearchFiles(AllFilesMask, APackage.GetUnitsOutputDir(ACPU,AOS), true, RemainingList);
            for i:=0 to RemainingList.Count-1 do
              begin
                if ExtractFileExt(Remaininglist[i])=PPUExt then
                  Installer.log(vlDebug,format('File %s still present, add corresponding entry to fpmake',[RemainingList[i]]))
                else
                  Installer.log(vlDebug,format('File %s still present',[RemainingList[i]]));
                List2.Add(IncludeTrailingPathDelimiter(APackage.GetUnitsOutputDir(ACPU,AOS))+RemainingList[i]);
              end;
            CmdDeleteFiles(List2);
            List2.free;
            RemainingList.Free;
            CmdRemoveTrees(DirectoryList);
            DirectoryList.Clear;
          end;
        { Also remove units/ or bin/ directory if empty }
        if IsDirectoryEmpty(ExtractFileDir(ExcludeTrailingPathDelimiter(APackage.GetBinOutputDir(ACPU,AOS)))) then
          DirectoryList.Add(ExtractFileDir(ExcludeTrailingPathDelimiter(APackage.GetBinOutputDir(ACPU,AOS))));
        if IsDirectoryEmpty(ExtractFileDir(ExcludeTrailingPathDelimiter(APackage.GetUnitsOutputDir(ACPU,AOS)))) then
          DirectoryList.Add(ExtractFileDir(ExcludeTrailingPathDelimiter(APackage.GetUnitsOutputDir(ACPU,AOS))));
        CmdRemoveDirs(DirectoryList);
      finally
        DirectoryList.Free;
      end;
      end;
  Finally
    List.Free;
  end;
end;


Procedure TBuildEngine.PkgList(PkgList: TStrings; APackage : TPackage);
begin
  Log(vlInfo, Format(SInfoPkgListPackage,[APackage.Name]));
  APackage.ListPackage(PkgList);
end;


procedure TBuildEngine.Compile(Packages: TPackages);

  function IsReadyToCompile(APackage:TPackage): boolean;
  begin
    result := False;
    if not APackage.FProcessing and (APackage.State=tsNeutral) then
      begin
        if PackageOK(APackage) then
          result := ReadyToCompile(APackage)
        else
          begin
            inc(FProgressCount);
            log(vlInfo,SInfoSkipPackageTargetProgress,[(FProgressCount)/FProgressMax * 100, APackage.Name, Defaults.Target]);
            APackage.FTargetState:=tsNoCompile;
          end;
      end;
  end;

Var
  I : integer;
{$ifndef NO_THREADING}
  Thr, ThreadCount : Integer;
  Finished : boolean;
  ErrorState: boolean;
  ErrorMessage: string;
  NotifyThreadWaiting : PRTLEvent;
  Threads : array of TCompileWorkerThread;
{$endif NO_THREADING}
  P : TPackage;

{$ifndef NO_THREADING}
  procedure ProcessThreadResult(ATHread: TCompileWorkerThread);
  var
    StartI: integer;
    CompilePackage: TPackage;
    PackageAvailable: boolean;
  begin
    if AThread.Done then
      begin
        { synchronise with the WriteBarrier in the thread (-> ReadBarrier), and prevent
          any writes we do here afterwards to be reordered before that (so the compile
          thread won't see these writes either -> also WriteBarrier) }
        ReadWriteBarrier;
        if assigned(AThread.APackage) then
          begin
            // The thread has completed compiling the package
            if AThread.CompilationOK then
              AThread.APackage.FTargetState:=tsCompiled
            else // A problem occurred, stop the compilation
              begin
              ErrorState:=true;
              ErrorMessage:='Error inside worker thread for package '+Athread.APackage.Name+': '+AThread.ErrorMessage;
              Finished:=true;
              end;
            AThread.APackage := nil;
          end;
        StartI := I;

        CompilePackage := nil;
        PackageAvailable:=false;
        repeat
        if IsReadyToCompile(Packages.PackageItems[i]) then
          CompilePackage := Packages.PackageItems[i];
        if not (Packages.PackageItems[i].State in [tsCompiled, tsNoCompile]) then
          PackageAvailable:=true;
        inc(I);
        if I=packages.Count then
          i := 0;
        until Assigned(CompilePackage) or (I=StartI);
        if Assigned(CompilePackage) then
          begin
          // Instruct thread to compile package
          AThread.APackage := CompilePackage;
          AThread.APackage.FProcessing := true;
          { Commit changes before setting FDone to false, because the threads
            only wait for an event 500ms at a time and hence way wake up
            and see that FDone=false before the event is sent and the changes
            are all committed by the event code }
          WriteBarrier;
          AThread.FDone:=False;
          RTLeventSetEvent(AThread.NotifyStartTask);
          end;
        if not PackageAvailable then
          Finished := True;
      end;
  end;

{$endif NO_THREADING}

begin
  NotifyEventCollection.CallEvents(neaBeforeCompile, Self);
  FProgressMax:=Packages.Count;
  FProgressCount:=0;

  if Defaults.ThreadsAmount<0 then
    begin
      // Do not use any threading to compile the packages
      For I:=0 to Packages.Count-1 do
        begin
          P:=Packages.PackageItems[i];
          If PackageOK(P) then
            MaybeCompile(P)
          else
            begin
            inc(FProgressCount);
            log(vlInfo,SInfoSkipPackageTargetProgress,[(FProgressCount)/FProgressMax * 100, P.Name, Defaults.Target]);
            end;
        end;
    end
  else
    begin
{$ifndef NO_THREADING}
      // Use worker-threads to compile the packages
      ErrorState := False;
      Finished := False;
      I := 0;
      ThreadCount:=0;
      // This event is set by the worker-threads to notify the main/this thread
      // that a package finished it's task.
      NotifyThreadWaiting := RTLEventCreate;
      SetLength(Threads,Defaults.ThreadsAmount);
      try 
        // Create all worker-threads
        try
          for Thr:=0 to Defaults.ThreadsAmount-1 do
            begin
              Threads[Thr] := TCompileWorkerThread.Create(self,NotifyThreadWaiting);
              if assigned(Threads[Thr]) then
                inc(ThreadCount);
            end;
        except
          on E: Exception do
            begin
              ErrorMessage := E.Message;
              ErrorState:=true;
            end;
        end;
        try
          // When a thread notifies this thread that it is ready, loop on all
          // threads to check their state and if possible assign a new package
          // to them to compile.
          while not Finished do
            begin
              RTLeventWaitFor(NotifyThreadWaiting);
              for Thr:=0 to Defaults.ThreadsAmount-1 do
                if assigned(Threads[Thr]) and not Finished then
                  ProcessThreadResult(Threads[Thr]);
            end;
        except
          on E: Exception do
            begin
              if not ErrorState then
                ErrorMessage := E.Message;
              ErrorState:=true;
            end;
        end;
        try
          // Compilation finished or aborted. Wait for all threads to end.
          for thr:=0 to Defaults.ThreadsAmount-1 do
            if assigned(Threads[Thr]) then
              begin
                Threads[Thr].Terminate;
                RTLeventSetEvent(Threads[Thr].NotifyStartTask);
                Threads[Thr].WaitFor;
              end;
        except
          on E: Exception do
            begin
              if not ErrorState then
                ErrorMessage := E.Message;
              ErrorState:=true;
            end;
        end;
      finally
        RTLeventdestroy(NotifyThreadWaiting);
        for thr:=0 to Defaults.ThreadsAmount-1 do
          if assigned(Threads[Thr]) then
            begin
              Threads[Thr].Free;
              dec(ThreadCount);
            end;
      end;
    if ErrorState then
      raise Exception.Create(ErrorMessage);
{$endif NO_THREADING}
    end;
  NotifyEventCollection.CallEvents(neaAfterCompile, Self);
end;


procedure TBuildEngine.Install(Packages: TPackages);
Var
  I : Integer;
  P : TPackage;
begin
  NotifyEventCollection.CallEvents(neaBeforeInstall, Self);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      If PackageOK(P) then
        begin
          Install(P, False);
          log(vlWarning, SWarnInstallationPackagecomplete, [P.Name, Defaults.Target]);
        end
      else
        log(vlInfo,SInfoSkipPackageTarget,[P.Name, Defaults.Target]);
    end;
  NotifyEventCollection.CallEvents(neaAfterInstall, Self);
end;

procedure TBuildEngine.ZipInstall(Packages: TPackages);

var
  I : Integer;
  P : TPackage;

begin
  NotifyEventCollection.CallEvents(neaBeforeInstall, Self);

  if Defaults.UnixPaths then
    Defaults.IntSetBaseInstallDir('lib/fpc/' + Defaults.FCompilerVersion+ '/')
  else
    Defaults.IntSetBaseInstallDir('');

  try
    For I:=0 to Packages.Count-1 do
      begin
        P:=Packages.PackageItems[i];
        If PackageOK(P) then
          begin
            Install(P, True);
            log(vlWarning, SWarnInstallationPackagecomplete, [P.Name, Defaults.Target]);
          end
        else
          log(vlInfo,SInfoSkipPackageTarget,[P.Name, Defaults.Target]);
      end;
  finally
    FinishArchive(P);
  end;

  NotifyEventCollection.CallEvents(neaAfterInstall, Self);
end;


procedure TBuildEngine.UnInstall(Packages: TPackages);
Var
  I : Integer;
  P : TPackage;
begin
  NotifyEventCollection.CallEvents(neaBeforeUnInstall, Self);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      UnInstall(P);
    end;
  NotifyEventCollection.CallEvents(neaAfterUnInstall, Self);
end;


procedure TBuildEngine.Archive(Packages: TPackages);
Var
  I : Integer;
  P : TPackage;
begin
  NotifyEventCollection.CallEvents(neaBeforeArchive, Self);
  Log(vlDebug, SDbgBuildEngineArchiving);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      // Force generation of manifest.xml, this is required for the repository
      Manifest(nil, P);
      Archive(P);
    end;
  NotifyEventCollection.CallEvents(neaAfterArchive, Self);
end;


procedure TBuildEngine.Manifest(Packages: TPackages; Package: TPackage);
Var
  L : TStrings;
  I : Integer;
  P : TPackage;
  FN: string;
begin
  NotifyEventCollection.CallEvents(neaBeforeManifest, Self);
  Log(vlDebug, SDbgBuildEngineGenerateManifests);

  L:=TStringList.Create;
  Try
    Log(vlDebug, Format(SDbgGenerating, [ManifestFile]));
    L.Add('<?xml version="1.0"?>');
    L.Add('<packages>');
    if assigned(Packages) then
      begin
        For I:=0 to Packages.Count-1 do
          begin
            P:=Packages.PackageItems[i];
            Log(vlInfo, Format(SInfoManifestPackage,[P.Name]));
            P.GetManifest(L);
          end
      end;
    if assigned(Package) then
      begin
        Log(vlInfo, Format(SInfoManifestPackage,[Package.Name]));
        Package.GetManifest(L);
      end;
    L.Add('</packages>');
    FN := ManifestFile;
    if assigned(Package) then
      FN := FixPath(Package.Directory, True)+FN;
    L.SaveToFile(FN);
  Finally
    L.Free;
  end;

  NotifyEventCollection.CallEvents(neaAfterManifest, Self);
end;


procedure TBuildEngine.PkgList(Packages: TPackages);
Var
  I : Integer;
  P : TPackage;
  L : TStrings;
  PKGL : String;
begin
  L:=TStringList.Create;
  NotifyEventCollection.CallEvents(neaBeforePkgList, Self);
  Log(vlDebug, SDbgBuildEngineGeneratePkgList);
{ Consider only the target OS, because the installer would be run there }
  if Defaults.OS in AllLimit83fsOSes then
    PKGL := PkgListFileBase + OSToString (Defaults.OS) + PkgListFileExt
  else if Defaults.OS = osNone then
    PKGL := PkgListFileBase + 'src' + PkgListFileExt
  else
    PKGL := PkgListFileBase + CPUToString (Defaults.CPU) + '-' +
                                     OSToString (Defaults.OS) + PkgListFileExt;

  Try
    Log(vlDebug, Format(SDbgGenerating, [PKGL]));

    For I:=0 to Packages.Count-1 do
      begin
        P:=Packages.PackageItems[i];
        PkgList(L, P);
      end;

    L.SaveToFile(PKGL);
  Finally
    L.Free;
  end;

  NotifyEventCollection.CallEvents(neaAfterPkgList, Self);
end;

procedure TBuildEngine.FPDocProject(Packages: TPackages; SingleDocFile: Boolean);

  Procedure AddHeader(L,Opts : Tstrings);

  Var
    I : Integer;
    N,V : String;

  begin
    L.Add('<?xml version="1.0" encoding="utf-8"?>');
    L.Add('<docproject>');
    L.Add('  <options>');
    L.Add('    <option name="ostarget" value="'+QuoteXML(CPUToString(Defaults.CPU))+'"/>');
    L.Add('    <option name="cputarget" value="'+QuoteXML(OSToString(Defaults.OS))+'"/>');
    L.Add('    <option name="parse-impl" value="false"/>');
    L.Add('    <option name="dont-trim" value="false"/>');
    if assigned(Opts) then
      begin
      For I:=0 to Opts.Count-1 do
        begin
        Opts.GetNameValue(I,N,V);
        L.Add('    <option name="%s" value="%s"/>',[QuoteXML(N),QuoteXML(V)]);
        end;
      end;
    L.Add('  </options>');
    L.Add('  <packages>');
  end;

  Procedure AddFooter(L : Tstrings);
  begin
    L.Add('  </packages>');
    L.Add('</docproject>');
  end;

Var
  L,LOpts : TStringList;
  FN : String;
  P : TPackage;

begin
  LOpts:=Nil;
  L:=TStringList.Create;
  Try
    if Defaults.FPDocOptions<>'' then
       begin
       LOpts:=TStringList.Create;
       LOpts.LoadFromFile(Defaults.FPDocOptions);
       end;
    if SingleDocFile then
      begin
      FN:='fpmake'+DocProjectFileExt;
      if Defaults.FPDocOutputDir<>'' then
        FN:=IncludeTrailingPathDelimiter(Defaults.FPDocOutputDir)+FN;
      Log(vlDebug, Format(SDbgGenerating, [FN]));
      AddHeader(L,Lopts);
      Log(vlInfo, Format(SInfoPackageDocProject,['<all>']));
      For P in Packages do
        GetDocProject(L,P,'    ');
      AddFooter(L);
      L.SaveToFile(FN);
      end
   else
     For P in Packages do
       begin
       L.Clear;
       FN:=P.Name+DocProjectFileExt;
       if Defaults.FPDocOutputDir<>'' then
         FN:=IncludeTrailingPathDelimiter(Defaults.FPDocOutputDir)+FN;
       Log(vlDebug, Format(SDbgGenerating, [FN]));
       AddHeader(L,Lopts);
       Log(vlInfo, Format(SInfoPackageDocProject,[P.Name]));
       GetDocProject(L,P,'    ');
       AddFooter(L);
       L.SaveToFile(FN);
       end;
  Finally
    L.Free;
    Lopts.Free;
  end;
end;

procedure TBuildEngine.Clean(Packages: TPackages; AllTargets: boolean);
Var
  I : Integer;
  P : TPackage;
begin
  NotifyEventCollection.CallEvents(neaBeforeClean, Self);
  Log(vldebug, SDbgBuildEngineCleaning);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      If AllTargets or PackageOK(P) then
        Clean(P, AllTargets);
    end;
  NotifyEventCollection.CallEvents(neaAfterClean, Self);
end;

{****************************************************************************
                               TFPVersion
****************************************************************************}

function TFPVersion.GetAsString: String;
begin
  if Empty then
    Result:='<none>'
  else
  begin
    Result := '';
    if Major <> -1 then
      Result := Result + IntToStr(Major);
    if Minor <> -1 then
      Result := Result + '.' + IntToStr(Minor);
    if Micro <> -1 then
      Result := Result + '.' + IntToStr(Micro);
    if Build <> -1 then
      Result := Result + '-'  + IntToStr(Build);
  end;
end;

function TFPVersion.GetEmpty: Boolean;
begin
  Result:=(Major=0) and (Minor=0) and (Micro=0) and (Build=0);
end;

procedure TFPVersion.SetAsString(const AValue: String);

  Function NextDigit(sep : Char; var V : string) : integer;
  Var
    P : Integer;
  begin
    P:=Pos(Sep,V);
    If (P=0) then
      P:=Length(V)+1;
    Result:=StrToIntDef(Copy(V,1,P-1),-1);
    If Result<>-1 then
      Delete(V,1,P);
  end;

Var
  V : String;
begin
  Clear;
  // Special support for empty version string
  if (AValue='') or (AValue='<none>') then
    exit;
  V:=AValue;
  Major:=NextDigit('.',V);
  Minor:=NextDigit('.',V);
  Micro:=NextDigit('-',V);
  Build:=NextDigit(#0,V);
end;

procedure TFPVersion.Clear;
begin
  Micro:=-1;
  Major:=-1;
  Minor:=-1;
  Build:=-1;
end;

procedure TFPVersion.Assign(Source: TPersistent);
Var
  V : TFPVersion;
begin
  if Source is TFPVersion then
    begin
    V:=Source as TFPVersion;
    Major:=V.Major;
    Minor:=V.Minor;
    Micro:=V.Micro;
    Build:=V.Build;
    end
  else
    inherited Assign(Source);
end;

function TFPVersion.CompareVersion(AVersion: TFPVersion): Integer;
begin
  Result:=Major-AVersion.Major;
  If (Result=0) then
    begin
      Result:=Minor-AVersion.Minor;
      if (Result=0) then
        begin
          Result:=Micro-AVersion.Micro;
          If (Result=0) then
            Result:=Build-AVersion.Build;
        end;
    end;
end;

function TFPVersion.SameVersion(AVersion: TFPVersion): Boolean;
begin
  Result:=CompareVersion(AVersion)=0;
end;


{****************************************************************************
                                 TTarget
****************************************************************************}

constructor TTarget.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FInstall:=True;
  FCPUs:=AllCPUs;
  FOSes:=AllOSes;
  FUnitPath:=TConditionalStrings.Create(TConditionalString);
  FIncludePath:=TConditionalStrings.Create(TConditionalString);
  FObjectPath:=TConditionalStrings.Create(TConditionalString);
  FDependencies:=TDependencies.Create(TDependency);
  FResourceFiles:=TResourceFiles.Create(TResourceFile);
  FCommands:=TCommands.Create(TCommand);
end;


destructor TTarget.Destroy;
begin
  FreeAndNil(FUnitPath);
  FreeAndNil(FObjectPath);
  FreeAndNil(FIncludePath);
  FreeAndNil(FDependencies);
  FreeAndNil(FResourceFiles);
  FreeAndNil(FCommands);
  FreeAndNil(Foptions);
  inherited Destroy;
end;

procedure TTarget.AssignTo(Dest: TPersistent);
var
  DestTarget: TTarget;
begin
  if Dest is TTarget then
    begin
    DestTarget := TTarget(Dest);
    DestTarget.Dependencies.Assign(Dependencies);
    DestTarget.Commands.Assign(Commands);
    DestTarget.FTargetState := FTargetState;
    DestTarget.TargetType := TargetType;
    DestTarget.CPUs := CPUs;
    DestTarget.OSes := OSes;
    DestTarget.Mode := Mode;
    DestTarget.Options := Options;
    DestTarget.Name :=  Name;
    DestTarget.Extension:= Extension;
    DestTarget.FPCTarget := FPCTarget;
    DestTarget.FileType := FileType;
    DestTarget.Directory := Directory;
    DestTarget.ResourceStrings := ResourceStrings;
    DestTarget.ResourceFiles.Assign(ResourceFiles);
    DestTarget.Install := Install;
    DestTarget.FTargetSourceFileName := fTargetSourceFileName;
    DestTarget.ObjectPath.Assign(ObjectPath);
    DestTarget.UnitPath.Assign(UnitPath);
    DestTarget.IncludePath.Assign(IncludePath);
    DestTarget.FXML := FXML;
    DestTarget.AfterCompile := AfterCompile;
    DestTarget.BeforeCompile := BeforeCompile;
    DestTarget.BeforeClean := BeforeCompile;
    DestTarget.AfterClean := AfterClean;
    end
  else
    inherited AssignTo(Dest);
end;

function TTarget.GetOptions: TStrings;
begin
  If Foptions=Nil then
    FOptions:=TStringList.Create;
  Result:=FOptions;
end;

function TTarget.GetImportLibFileName(AOS : TOS) : String;
begin
  result := GetImportLibraryFilename(Name,AOS);
end;

function TTarget.GetUnitLibFileName(AOS : TOS): String;
begin
  if AOS in [atari,netwlibc,go32v2,watcom,wdosx,msdos,win16] then
    Result := Name+LibExt
  else if AOS in [java] then
    Result:=Name+'.jar'
  else if AOS in [macosclassic] then
    Result:=Name+'Lib'
  else
    Result:='libp'+Name+LibExt;
end;

procedure TTarget.SetOptions(const AValue: TStrings);
begin
  If (AValue=Nil) or (AValue.Count=0) then
    FreeAndNil(FOptions)
  else
    Options.Assign(AValue);
end;

function TTarget.GetSourceFileName: String;
begin
  Result:=Name+FExtension;
end;


function TTarget.GetUnitFileName: String;
begin
  Result:=Name+UnitExt;
end;


function TTarget.GetObjectFileName: String;
begin
  Result:=Name+ObjExt;
end;


function TTarget.GetLTOFileName: String;
begin
  Result:=Name+LTOExt;
end;


function TTarget.GetRSTFileName: String;
begin
  Result:=Name+RSText;
end;


function TTarget.GetRSJFileName: String;
begin
  Result:=Name+RSJext;
end;


function TTarget.GetBinFileBase: String;
begin
  if FExeName <> '' then
    Result := FExeName
  else
    Result:=Name;
end;


function TTarget.GetProgramFileName(AOS : TOS): String;
begin
    result := AddProgramExtension(GetBinFileBase, AOS);
end;


function TTarget.GetProgramDebugFileName(AOS: TOS): String;
begin
  result := GetBinFileBase + DbgExt;
end;

function TTarget.GetLibraryFileName(AOS : TOS): String;
begin
  result := AddLibraryExtension(GetBinFileBase, AOS);
  if aOS in AllUnixOSes then
    Result:='lib'+Result;
end;


function TTarget.GetLibraryDebugFileName(AOS: TOS): String;
begin
  result := GetBinFileBase + DbgExt;
end;


function TTarget.GetOutputFileName(AOs: TOS): String;
begin
  if TargetType in UnitTargets then
    Result:=GetUnitFileName
  else if TargetType=ttSharedLibrary then
    Result:=GetLibraryFileName(AOs)
  else
    Result:=GetProgramFileName(AOs);
end;

function TTarget.HaveOptions: Boolean;
begin
  Result:=(FOptions<>Nil);
end;


procedure TTarget.SetName(const AValue: String);
Var
  D,N,E : String;
begin
  N:=FixPath(AValue, False);
  D:=ExtractFilePath(N);
  E:=ExtractFileExt(N);
  N:=ExtractFileName(N);
  inherited SetName(Copy(N,1,Length(N)-Length(E)));
  FExtension:=E;
  FDirectory:=D;
end;

procedure TTarget.SetExeName(const AValue: String);
Var
  N,E : String;
begin
  N:=FixPath(AValue, False);
  E:=ExtractFileExt(N);
  N:=ExtractFileName(N);
  FExeName:=Copy(N,1,Length(N)-Length(E));
  { Use exact AValue for -o option }
  Options.Add('-o'+AValue);
end;

procedure TTarget.SetXML(const AValue: string);
begin
  FXML:=FixPath(AValue, False);
end;

procedure TTarget.GetCleanFiles(List: TStrings; const APrefixU, APrefixB : String; ACPU: TCPU; AOS : TOS);
begin
  If not(ACPU in CPUs) or not(AOS in OSes) then
    exit;
  List.Add(APrefixU + ObjectFileName);
  List.Add(APrefixU + LTOFileName);
  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit, ttCleanOnlyUnit]) then
    begin
      List.Add(APrefixU + UnitFileName);
      if (AOS in AllSmartLinkLibraryOSes) and FileExists(APrefixU + GetUnitLibFileName(AOS)) then
        List.Add(APrefixU + GetUnitLibFileName(AOS));
      if (AOS in AllImportLibraryOSes) and FileExists(APrefixU + GetImportLibFilename(AOS)) then
        List.Add(APrefixU + GetImportLibFilename(AOS));
    end
  else If (TargetType in [ttProgram,ttExampleProgram]) then
    begin
      List.Add(APrefixB + GetProgramFileName(AOS));
      if FileExists(APrefixB + GetProgramDebugFileName(AOS)) then
        List.Add(APrefixB + GetProgramDebugFileName(AOS));
      if (AOS in AllImportLibraryOSes) and FileExists(APrefixU + GetImportLibFilename(AOS)) then
        List.Add(APrefixU + GetImportLibFilename(AOS));
    end
  else If (TargetType in [ttSharedLibrary]) then
    begin
      List.Add(APrefixB + GetLibraryFileName(AOS));
      if FileExists(APrefixB + GetLibraryDebugFileName(AOS)) then
        List.Add(APrefixB + GetLibraryDebugFileName(AOS));
    end;
  If ResourceStrings then
    begin
      // choose between 2 possible resource files
      if FileExists(APrefixU + RSJFileName) then
        List.Add(APrefixU + RSJFileName)
      else
        List.Add(APrefixU + RSTFileName);
    end
  else
    begin
      if FileExists(APrefixU + RSJFileName) then
        begin
          Installer.Log(VlDebug,Format(SDbgUnregisteredResource,[APrefixU + RSJFileName]));
          List.Add(APrefixU + RSJFileName);
        end
      else if FileExists(APrefixU + RSTFileName) then
        begin
          Installer.Log(VlDebug,Format(SDbgUnregisteredResource,[APrefixU + RSTFileName]));
          List.Add(APrefixU + RSTFileName);
        end;
     end;
  // Maybe add later ?  AddConditionalStrings(List,CleanFiles);
end;


procedure TTarget.GetInstallFiles(List: TStrings; const APrefixU, APrefixB: String; ACPU: TCPU; AOS : TOS);
var
  UnitsDir : string;
begin
  UnitsDir := Installer.BuildEngine.AddPathPrefix(nil, APrefixU);
  If Not (TargetType in [ttProgram,ttSharedLibrary,ttExampleProgram]) then
    begin
      // The compiler does not create an objectfile for all programs.
      if FileExists(UnitsDir + ObjectFileName) then
        List.Add(APrefixU + ObjectFileName);
      if FileExists(UnitsDir + LTOFileName) then
        List.Add(APrefixU + LTOFileName);
    end;
  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit]) then
    begin
      List.Add(APrefixU + UnitFileName);
      if (AOS in AllSmartLinkLibraryOSes) and FileExists(UnitsDir + GetUnitLibFileName(AOS)) then
        List.Add(APrefixU + GetUnitLibFileName(AOS));
      if (AOS in AllImportLibraryOSes) and FileExists(UnitsDir + GetImportLibFilename(AOS)) then
        List.Add(APrefixU + GetImportLibFilename(AOS));
    end
  else If (TargetType in [ttProgram,ttExampleProgram]) then
    List.Add(APrefixB + GetProgramFileName(AOS))
  else If (TargetType in [ttSharedLibrary]) then
    List.Add(APrefixB + GetLibraryFileName(AOS));
  If ResourceStrings then
    begin
      // choose between 2 possible resource files
      if FileExists(UnitsDir + RSJFileName) then
        List.Add(APrefixU + RSJFileName)
      else
        List.Add(APrefixU + RSTFileName);
    end
  else
    begin
      if FileExists(UnitsDir + RSJFileName) then
        begin
          Installer.Log(VlDebug,Format(SDbgUNregisteredResource,[APrefixU + RSJFileName]));
          List.Add(APrefixU + RSJFileName);
        end
      else if FileExists(UnitsDir + RSTFileName) then
        begin
          Installer.Log(VlDebug,Format(SDbgUNregisteredResource,[APrefixU + RSTFileName]));
          List.Add(APrefixU + RSTFileName);
        end;
     end;
  FResourceFiles.GetInstallFiles(List, APrefixU, APrefixB, ACPU, AOS);
end;


procedure TTarget.GetArchiveFiles(List: TStrings; ACPU: TCPU; AOS : TOS);
var
  i : integer;
  D : TDependency;
begin
  If not(ACPU in CPUs) or not(AOS in OSes) then
    exit;
  // Main source
  if TargetSourceFileName<>'' then
    List.Add(TargetSourceFileName);
  // Includes
  for i:=0 to Dependencies.Count-1 do
    begin
      D:=Dependencies[i];
      if (D.DependencyType=depInclude) and
         (D.TargetFileName<>'') then
        List.Add(D.TargetFileName);
    end;
  // FPDoc files
  if XML <> '' then
  begin
    List.Add(Directory + Name + Extension);
    List.Add(XML);
  end;
end;


{****************************************************************************
                                 TSource
****************************************************************************}

function TSource.GetInstallSourcePath: string;
begin
  if FInstallSourcePath<>'' then
    result := FInstallSourcePath
  else if SourceType=stExample then
    result := 'examples'
  else
    result := '';
end;

constructor TSource.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
end;


destructor TSource.Destroy;
begin
  inherited Destroy;
end;

procedure TSource.GetInstallFiles(List: TStrings);
begin
  if InstallSourcePath<>'' then
    list.Values[name] := (IncludeTrailingPathDelimiter(InstallSourcePath)+ExtractFileName(Name))
  else
    list.add(Name);
end;


{****************************************************************************
                                 TCommands
****************************************************************************}

function TCommands.GetCommand(const Dest : String): TCommand;
begin
  Result:=TCommand(ItemByName(Dest));
end;

function TCommands.GetCommandItem(Index : Integer): TCommand;
begin
  Result:=TCommand(Items[Index]);
end;


procedure TCommands.SetCommandItem(Index : Integer; const AValue: TCommand);
begin
  Items[Index]:=AValue;
end;


function TCommands.AddCommand(const Cmd: String): TCommand;
begin
  Result:=AddCommand(fdefaultAt,Cmd,'','','');
end;


function TCommands.AddCommand(const Cmd, Options: String): TCommand;
begin
  Result:=AddCommand(fdefaultAt,Cmd,Options,'','');
end;


function TCommands.AddCommand(const Cmd, Options, Dest, Source: String ): TCommand;
begin
  Result:=AddCommand(fdefaultAt,Cmd,options,Dest,Source);
end;


function TCommands.AddCommand(At: TCommandAt; const Cmd: String): TCommand;
begin
  Result:=AddCommand(At,Cmd,'','','');
end;


function TCommands.AddCommand(At: TCommandAt; const Cmd, Options: String  ): TCommand;
begin
  Result:=AddCommand(At,Cmd,Options,'','');
end;


function TCommands.AddCommand(At: TCommandAt; const Cmd, Options, Dest, Source: String): TCommand;
begin
  Result:=Add as TCommand;
  Result.Command:=Cmd;
  If (Options<>'') then
    Result.ParseOptions(Options);
  Result.At:=At;
  Result.SourceFile:=Source;
  Result.DestFile:=Dest;
end;

function TCommands.GetEnumerator: TCommandEnumerator;
begin
  Result:=TCommandEnumerator.Create(Self);
end;


{****************************************************************************
                           TConditionalString
****************************************************************************}

Constructor TConditionalString.Create;
begin
  inherited Create;
end;


{****************************************************************************
                           TConditionalStrings
****************************************************************************}

Constructor TConditionalStrings.Create(AClass:TConditionalStringClass);
begin
  inherited Create;
  FCSClass:=AClass;
end;


function TConditionalStrings.GetConditionalString(Index : Integer): TConditionalString;
begin
  Result:=TConditionalString(Items[Index]);
end;


procedure TConditionalStrings.SetConditionalString(Index : Integer; const AValue: TConditionalString);
begin
  Items[Index]:=AValue;
end;


Function TConditionalStrings.Add(Const Value : String) : TConditionalString;
begin
  result:=Add(Value,AllCPUs,AllOSes);
end;


{$ifdef cpu_only_overloads}
Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPUs) : TConditionalString;
begin
  result:=Add(Value,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TConditionalStrings.Add(Const Value : String;const OSes:TOSes) : TConditionalString;
begin
  result:=Add(Value,AllCPUs,OSes);
end;


Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TConditionalString;
begin
  Result:=FCSClass.Create;
  Result.Value:=Value;
  Result.OSes:=OSes;
  Result.CPUs:=CPUs;
  inherited Add(Result);
end;


{****************************************************************************
                                TDependency
****************************************************************************}

Constructor TDependency.Create;
begin
  inherited Create;
  FVersion:=TFPVersion.Create;
end;


Destructor TDependency.Destroy;
begin
  FreeAndNil(FVersion);
end;


Function TDependency.GetVersion : string;
begin
  result:=FVersion.AsString;
end;


Procedure TDependency.SetVersion(const V : string);
begin
  FVersion.AsString:=V;
end;


{****************************************************************************
                                TDependencies
****************************************************************************}

function TDependencies.GetDependency(Index : Integer): TDependency;
begin
  Result:=TDependency(Items[Index]);
end;


procedure TDependencies.SetDependency(Index : Integer; const AValue: TDependency);
begin
  Items[Index]:=AValue;
end;


Function TDependencies.Add(Const Value : String) : TDependency;
begin
  result:=Add(Value,AllCPUs,AllOSes);
end;


{$ifdef cpu_only_overloads}
Function TDependencies.Add(Const Value : String;const CPUs:TCPUs) : TDependency;
begin
  result:=Add(Value,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TDependencies.Add(Const Value : String;const OSes:TOSes) : TDependency;
begin
  result:=Add(Value,AllCPUs,OSes);
end;


Function TDependencies.Add(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TDependency;
begin
  Result:=inherited Add(Value,CPUs,OSes) as TDependency;
  Result.Target:=nil;
  Result.FDependencyType:=depPackage;
end;


Function TDependencies.AddUnit(Const Value : String) : TDependency;
begin
  result:=AddUnit(Value,AllCPUs,AllOSes);
end;


{$ifdef cpu_only_overloads}
Function TDependencies.AddUnit(Const Value : String;const CPUs:TCPUs) : TDependency;
begin
  result:=AddUnit(Value,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TDependencies.AddUnit(Const Value : String;const OSes:TOSes) : TDependency;
begin
  result:=AddUnit(Value,AllCPUs,OSes);
end;


Function TDependencies.AddUnit(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TDependency;
begin
  Result:=inherited Add(Value,CPUs,OSes) as TDependency;
  Result.Target:=nil;
  Result.FDependencyType:=depUnit;
end;


Function TDependencies.AddInclude(Const Value : String) : TDependency;
begin
  result:=AddInclude(Value,AllCPUs,AllOSes);
end;


{$ifdef cpu_only_overloads}
Function TDependencies.AddInclude(Const Value : String;const CPUs:TCPUs) : TDependency;
begin
  result:=AddInclude(Value,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TDependencies.AddInclude(Const Value : String;const OSes:TOSes) : TDependency;
begin
  result:=AddInclude(Value,AllCPUs,OSes);
end;


Function TDependencies.AddInclude(Const Value : String;const CPUs:TCPUs;const OSes:TOSes) : TDependency;
Var
  N : String;
begin
  N:=FixPath(Value, False);
  if ExtractFileExt(N)='' then
    N:=ChangeFileExt(N,IncExt);
  Result:=inherited Add(N,CPUs,OSes) as TDependency;
  Result.FDependencyType:=depInclude;
end;


{****************************************************************************
                               TValueItem
****************************************************************************}

constructor TValueItem.Create(AValue: String);
begin
  FValue:=AValue;
end;


{****************************************************************************
                              TFunctionItem
****************************************************************************}

constructor TFunctionItem.Create(AFunc: TReplaceFunction);
begin
  FFunc:=AFunc;
end;

{****************************************************************************
                           TNotifyEventItem
****************************************************************************}

procedure TNotifyEventItem.CallEvent(Sender: TObject);
begin
  if assigned(OnEvent) then
    OnEvent(Sender);
  if assigned(OnProcEvent) then
    OnProcEvent(sender);
end;

{****************************************************************************
                           TNotifyEventCollection
****************************************************************************}

constructor TNotifyEventCollection.create(ASupportedActionSet: TNotifyEventActionSet);
begin
  FSupportedActionSet:=ASupportedActionSet;
  inherited create(TNotifyEventItem);
end;

function TNotifyEventCollection.GetEnumerator: TNotifyEventEnumerator;
begin
  Result:= TNotifyEventEnumerator.Create(Self);
end;

procedure TNotifyEventCollection.AppendEvent(AnAction: TNotifyEventAction; AnEvent: TNotifyEvent);
var
  item: TNotifyEventItem;
begin
  if not (AnAction in FSupportedActionSet) then
    raise Exception.Create(SErrEventNotSupported);
  item := TNotifyEventItem(add);
  item.OnEvent:=AnEvent;
  item.OnAction:=AnAction;
end;

procedure TNotifyEventCollection.AppendProcEvent(AnACtion: TNotifyEventAction;
  AnProcEvent: TNotifyProcEvent);
var
  item: TNotifyEventItem;
begin
  if not (AnAction in FSupportedActionSet) then
    raise Exception.Create(SErrEventNotSupported);
  item := TNotifyEventItem(add);
  item.OnProcEvent:=AnProcEvent;
  item.OnAction:=AnAction;
end;

procedure TNotifyEventCollection.CallEvents(AnAction: TNotifyEventAction; Sender: TObject);
var
  i: integer;
  item: TNotifyEventItem;
begin
  for i := 0 to Count-1 do
    begin
      item := TNotifyEventItem(Items[i]);
      if item.OnAction=AnAction then
        item.CallEvent(Sender);
    end;
end;

{****************************************************************************
                                 TDictionary
****************************************************************************}

constructor TDictionary.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FList:=TStringList.Create;
  FList.Sorted:=True;
  FList.Duplicates:=dupError;
end;


destructor TDictionary.Destroy;
Var
  I : Integer;
begin
  For I:=0 to Flist.Count-1 do
    FList.Objects[i].Free;
  FreeAndNil(FList);
  inherited Destroy;
end;


procedure TDictionary.AddVariable(const AName, Value: String);
Var
  I : Integer;
begin
  I:=Flist.IndexOf(AName);
  If I=-1 then
    I:=FList.Add(Aname)
  else
    Flist.Objects[i].Free;
  Flist.Objects[i]:=TValueItem.Create(Value);
end;


procedure TDictionary.AddFunction(const AName: String; FReplacement: TReplaceFunction);
Var
  I : Integer;
begin
  I:=Flist.IndexOf(AName);
  If I=-1 then
    I:=Flist.Add(AName)
  else
    Flist.Objects[i].Free;
  Flist.Objects[i]:=TFunctionItem.Create(FReplacement);
end;


procedure TDictionary.RemoveItem(const AName: String);
Var
  I : Integer;
begin
  I:=Flist.IndexOf(AName);
  If (I<>-1) then
    begin
    FList.Objects[i].Free;
    FList.Delete(I);
    end;
end;


function TDictionary.GetValue(AName: String): String;
begin
  Result:=GetValue(AName,'');
end;


function TDictionary.GetValue(const AName,Args: String): String;
Var
  O : TObject;
  I : Integer;
begin
  I:=Flist.IndexOf(AName);
  If (I=-1) then
    Raise EDictionaryError.CreateFmt(SErrNoDictionaryItem,[AName]);
  O:=Flist.Objects[I];
  If O is TValueItem then
    Result:=TValueItem(O).FValue
  else
    Result:=TFunctionItem(O).FFunc(AName,Args);
end;


function TDictionary.ReplaceStrings(Const ASource: String; Const MaxDepth: Integer = 10): String;
Var
  S,FN,FV : String;
  P: Integer;
begin
  Result:='';
  S:=ASource;
  P:=Pos('$(',S);
  While (P<>0) do
    begin
      Result:=Result+Copy(S,1,P-1);
      Delete(S,1,P+1);
      P:=Pos(')',S);
      FN:=Copy(S,1,P-1);
      Delete(S,1,P);
      P:=Pos(' ',FN);
      If (P<>0) then // function arguments ?
        begin
        FV:=FN;
        FN:=Copy(FN,1,P);
        System.Delete(FV,1,P);
        end
      else
        FV:='';
      if MaxDepth > 0 then
        Result:=Result+ReplaceStrings(GetValue(FN,FV), MaxDepth-1)
      else
        Result:=Result+GetValue(FN,FV);
      P:=Pos('$(',S);
    end;
  Result:=Result+S;
end;


Function TDictionary.Substitute(Const Source : String; const Macros : Array of string) : String;
Var
  I : Integer;
begin
  I:=0;
  While I<High(Macros) do
    begin
      AddVariable(Macros[i],Macros[I+1]);
      Inc(I,2);
    end;
  Result:=ReplaceStrings(Source);
  I:=0;
  While I<High(Macros) do
    begin
      RemoveItem(Macros[i]);
      Inc(I,2);
    end;
end;

{****************************************************************************
                                 Default Instances
****************************************************************************}

var
  DefInstaller : TCustomInstaller;

Function Installer(InstallerClass: TInstallerClass): TCustomInstaller;
begin
  If Not Assigned(DefInstaller) then
    begin
      try
        DefInstaller:=InstallerClass.Create(Nil);
      except
        On E : Exception do
          begin
            if IsConsole then
              begin
                WriteLn(SErrInstaller);
                WriteLn(E.Message);
                halt(1);
              end
            else
              raise;
          end;
      end;
    end;
  Result:=DefInstaller;
end;


Function Installer: TCustomInstaller;
begin
  Result := Installer(TFPCInstaller);
end;



{ TCommand }

function TCommand.GetOptions: TStrings;
begin
  If (FOptions=Nil) then
    FOptions:=TStringList.Create;
  Result:=FOptions;
end;

procedure TCommand.SetOptions(const Value: TStrings);
begin
  If (Value=Nil) or (Value.Count=0) then
    FreeAndNil(FOptions)
  else
    Options.Assign(Value);
end;

destructor TCommand.Destroy;
begin
  FreeAndNil(FOptions);
  inherited Destroy;
end;

function TCommand.HaveOptions: Boolean;
begin
  Result:=(FOptions<>Nil);
end;


function TCommand.CmdLineOptions: String;
begin
  If HaveOptions then
    Result:=OptionListToString(Options);
end;

procedure TCommand.ParseOptions(S: String);

begin
  Options:=OptionsToStringList(S);
end;

Initialization
  OnGetApplicationName:=@GetFPMakeName;
  CustomFpmakeCommandlineOptions:=nil;
  CustomFpMakeCommandlineValues:=nil;

  GetPluginManager.RegisterPlugin(TfpmResolvePackagePathsPlugin);

Finalization
  FreeAndNil(CustomFpMakeCommandlineValues);
  FreeAndNil(CustomFpmakeCommandlineOptions);
  FreeAndNil(DefInstaller);
  FreeAndNil(GlobalDictionary);
  FreeAndNil(Defaults);
  FreeAndNil(GPluginManager);
end.


