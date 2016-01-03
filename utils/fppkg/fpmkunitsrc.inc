{$ifdef Delphi}
<<<<<<< HEAD
<<<<<<< HEAD
const fpmkunitsrc : array[0..638] of string[240]=(
{$else Delphi}
const fpmkunitsrc : array[0..638,1..240] of char=(
=======
const fpmkunitsrc : array[0..599] of string[240]=(
{$else Delphi}
const fpmkunitsrc : array[0..599,1..240] of char=(
>>>>>>> graemeg/fixes_2_2
=======
const fpmkunitsrc : array[0..599] of string[240]=(
{$else Delphi}
const fpmkunitsrc : array[0..599,1..240] of char=(
>>>>>>> origin/fixes_2_2
{$endif Delphi}
  '{'#010+
  '    This file is part of the Free Pascal Makefile Package'#010+
  #010+
  '    Implementation of fpmake classes and functions'#010+
  #010+
  '    Copyright (c) 2007 by the freepascal team'#010+
  #010+
  '    See the file COPYING.FPC, included in this distribution,'#010+
  '    for details abo','ut the copyright.'#010+
  #010+
  '    This program is distributed in the hope that it will be useful,'#010+
  '    but WITHOUT ANY WARRANTY; without even the implied warranty of'#010+
  '    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.'#010+
  #010+
  ' ***************************','****************************************'+
  '***}'#010+
  #010+
  'unit fpmkunit;'#010+
  #010+
  '{$Mode objfpc}'#010+
  '{$H+}'#010+
  '{$inline on}'#010+
  #010+
  '{ For target or cpu dependent dependencies add also an overload were y'+
  'ou'#010+
  '  can pass only a set of cpus. This is disabled for now because it cre',
  'ates'#010+
  '  an error in the compiler with overload choosing }'#010+
  '{ define cpu_only_overloads}'#010+
  #010+
  'Interface'#010+
  #010+
  '{$ifndef NO_UNIT_PROCESS}'#010+
  '  {$define HAS_UNIT_PROCESS}'#010+
  '{$endif NO_UNIT_PROCESS}'#010+
  #010+
  '{$ifndef NO_UNIT_ZIPPER}'#010+
  '  {$define HAS_UNIT_ZIPPER}'#010+
  '{$endif ','NO_UNIT_ZIPPER}'#010+
  #010+
  'uses'#010+
  '  SysUtils, Classes, StrUtils'#010+
  '{$ifdef HAS_UNIT_PROCESS}'#010+
  '  ,process'#010+
  '{$endif HAS_UNIT_PROCESS}'#010+
  '{$ifdef HAS_UNIT_ZIPPER}'#010+
  '  ,zipper'#010+
  '{$endif HAS_UNIT_ZIPPER}'#010+
  '  ;'#010+
  #010+
  'Type'#010+
  '  TFileType = (ftSource,ftUnit,ftObject,ftResource,ftEx','ecutable,ftSt'+
  'aticLibrary,'#010+
  '               ftSharedLibrary);'#010+
  '  TFileTypes = set of TFileType;'#010+
  #010+
  '  // Please keep this order, see OSCPUSupported below'#010+
  '  TCpu=(cpuNone,'#010+
  '    i386,m68k,powerpc,sparc,x86_64,arm,powerpc64,avr,armeb'#010+
  '  );'#010+
  '  TCPUS = Se','t of TCPU;'#010+
  #010+
  '  // Please keep this order, see OSCPUSupported below'#010+
  '  TOS=(osNone,'#010+
  '    linux,go32v2,win32,os2,freebsd,beos,netbsd,'#010+
  '    amiga,atari, solaris, qnx, netware, openbsd,wdosx,'#010+
  '    palmos,macos,darwin,emx,watcom,morphos,netwlibc,'#010+
  '   ',' win64,wince,gba,nds,embedded,symbian'#010+
  '  );'#010+
  '  TOSes = Set of TOS;'#010+
  #010+
  '  TCompilerMode = (cmFPC,cmTP,cmObjFPC,cmDelphi,cmMacPas);'#010+
  '  TCompilerModes = Set of TCompilerMode;'#010+
  #010+
  '  TTargetType = (ttProgram,ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExa'+
  'mpl','eUnit,ttExampleProgram);'#010+
  '  TTargetTypes = set of TTargetType;'#010+
  #010+
  '  TTargetState = (tsNeutral,tsConsidering,tsNoCompile,tsCompiled,tsIns'+
  'talled,tsNotFound);'#010+
  '  TTargetStates = Set of TTargetState;'#010+
  #010+
  '  TSourceType = (stDoc,stSrc,stExample,stTest);',#010+
  '  TSourceTypes = set of TSourceType;'#010+
  #010+
  '  TVerboseLevel = (vlError,vlWarning,vlInfo,vldebug,vlCommand);'#010+
  '  TVerboseLevels = Set of TVerboseLevel;'#010+
  #010+
  '  TCommandAt = (caBeforeCompile,caAfterCompile,'#010+
  '                caBeforeInstall,caAfterInstall,',#010+
  '                caBeforeArchive,caAfterArchive,'#010+
  '                caBeforeClean,caAfterClean,'#010+
  '                caBeforeDownload,caAfterDownload);'#010+
  #010+
  '  TDependencyType = (depPackage,depImplicitPackage,depUnit,depInclude)'+
  ';'#010+
  '  TDependencyTypes = se','t of TDependencyType;'#010+
  #010+
  '  TLogEvent = Procedure (Level : TVerboseLevel; Const Msg : String) of'+
  ' Object;'#010+
  #010+
  '  TRunMode = (rmCompile,rmBuild,rmInstall,rmArchive,rmClean,rmManifest'+
  ');'#010+
  #010+
  'Const'#010+
  '  // Aliases'#010+
  '  Amd64   = X86_64;'#010+
  '  PPC = PowerPC;'#010+
  '  PPC64',' = PowerPC64;'#010+
  '  DOS = Go32v2;'#010+
  '  MacOSX = Darwin;'#010+
  #010+
  '  AllOSes = [Low(TOS)..High(TOS)];'#010+
  '  AllCPUs = [Low(TCPU)..High(TCPU)];'#010+
  '  AllUnixOSes  = [Linux,FreeBSD,NetBSD,OpenBSD,Darwin,QNX,BeOS,Solaris'+
  '];'#010+
  '  AllBSDOSes      = [FreeBSD,NetBSD,OpenBSD,D','arwin];'#010+
  '  AllWindowsOSes  = [Win32,Win64,WinCE];'#010+
  #010+
  '  { This table is kept OS,Cpu because it is easier to maintain (PFV) }'+
  #010+
  '  OSCPUSupported : array[TOS,TCpu] of boolean = ('#010+
  '    { os          none   i386    m68k  ppc    sparc  x86_64 arm    pp',
  'c64  avr    armeb}'#010+
  '    { none }    ( false, false, false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { linux }   ( false, true,  true,  true,  true,  true,  true,  tru'+
  'e,  false, true ),'#010+
  '    { go32v2 }  ( false, true,  false, fals','e, false, false, false, f'+
  'alse, false, false),'#010+
  '    { win32 }   ( false, true,  false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { os2 }     ( false, true,  false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { freebsd } ','( false, true,  true,  false, false, true,  false, f'+
  'alse, false, false),'#010+
  '    { beos }    ( false, true,  false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { netbsd }  ( false, true,  true,  true,  true,  false, false, fal'+
  'se, fals','e, false),'#010+
  '    { amiga }   ( false, false, true,  true,  false, false, false, fal'+
  'se, false, false),'#010+
  '    { atari }   ( false, false, true,  false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { solaris } ( false, true,  false, false, true,','  false, false, f'+
  'alse, false, false),'#010+
  '    { qnx }     ( false, true,  false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { netware } ( false, true,  false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { openbsd } ( false,',' true,  true,  false, false, false, false, f'+
  'alse, false, false),'#010+
  '    { wdosx }   ( false, true,  false, false, false, false, false, fal'+
  'se, false, false),'#010+
  '    { palmos }  ( false, false, true,  false, false, false, true,  fal'+
  'se, false, false','),'#010+
  '    { macos }   ( false, false, false, true,  false, false, false, fal'+
  'se, false, false),'#010+
  '    { darwin }  ( false, true,  false, true,  false, true,  true,  tru'+
  'e,  false, false),'#010+
  '    { emx }     ( false, true,  false, false, false, false,',' false, f'+
  'alse, false, false),'#010+
  '    { watcom }  ( false, true,  false, false, false ,false, false, fal'+
  'se, false, false),'#010+
  '    { morphos } ( false, false, false, true,  false ,false, false, fal'+
  'se, false, false),'#010+
  '    { netwlibc }( false, true,  ','false, false, false, false, false, f'+
  'alse, false, false),'#010+
  '    { win64   } ( false, false, false, false, false, true,  false, fal'+
  'se, false, false),'#010+
  '    { wince    }( false, true,  false, false, false, false, true,  fal'+
  'se, false, false),'#010+
  '    {',' gba    }  ( false, false, false, false, false, false, true,  f'+
  'alse, false, false),'#010+
  '    { nds    }  ( false, false, false, false, false, false, true,  fal'+
  'se, false, false),'#010+
  '    { embedded }( false, true,  true,  true,  true,  true,  true,  ','t'+
  'rue,  true,  true ),'#010+
  '    { symbian } ( false, true,  false, false, false, false, true,  fal'+
  'se, false, false)'#010+
  '  );'#010+
  #010+
  '  // Useful'#010+
  '  UnitExt = '#039'.ppu'#039';'#010+
  '  PPUExt  = UnitExt;'#010+
  '  PasExt  = '#039'.pas'#039';'#010+
  '  PPExt   = '#039'.pp'#039';'#010+
  '  IncExt  = '#039'.inc'#039';'#010+
  '  ObjExt  = ',#039'.o'#039';'#010+
  '  RstExt  = '#039'.rst'#039';'#010+
  '  LibExt  = '#039'.a'#039';'#010+
  '  SharedLibExt = '#039'.so'#039';'#010+
  '  DLLExt  = '#039'.dll'#039';'#010+
  '  ExeExt  = '#039'.exe'#039';'#010+
  '  ZipExt  = '#039'.zip'#039';'#010+
  #010+
  '  FPMakePPFile = '#039'fpmake.pp'#039';'#010+
  '  ManifestFile = '#039'manifest.xml'#039';'#010+
  '  UnitConfigFile = '#039'fpunits.conf'#039';'#010+
  #010+
  '  DirNotFoun','d = '#039'<dirnotfound>'#039';'#010+
  #010+
  '  UnitTargets = [ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit];'+
  #010+
  '  ProgramTargets = [ttProgram,ttExampleProgram];'#010+
  #010+
  '  DefaultMessages = [vlError,vlWarning,vlCommand];'#010+
  '  AllMessages = [vlError,vlWarning,vlCommand,v','lInfo];'#010+
  #010+
  #010+
  'Type'#010+
  '  { TNamedItem }'#010+
  #010+
  '  TNamedItem = Class(TCollectionItem)'#010+
  '  private'#010+
  '    FName: String;'#010+
  '    procedure SetName(const AValue: String);virtual;'#010+
  '  Public'#010+
  '    property Name : String Read FName Write SetName;'#010+
  '  end;'#010+
  #010+
  '  { TNamedCollect','ion }'#010+
  #010+
  '  TNamedCollection = Class(TCollection)'#010+
  '  private'#010+
  '    FUniqueNames: Boolean;'#010+
  '  Public'#010+
  '    Function IndexOfName(const AName : String) : Integer;'#010+
  '    Function ItemByName(const AName : String) : TNamedItem;'#010+
  '    Property UniqueNames : Bo','olean Read FUniqueNames;'#010+
  '  end;'#010+
  #010+
  '  { TNamedItemList }'#010+
  #010+
  '  TNamedItemList = Class(TFPList)'#010+
  '  private'#010+
  '    function GetNamedItem(Index : Integer): TNamedItem;'#010+
  '    procedure SetNamedItem(Index : Integer; const AValue: TNamedItem);'+
  #010+
  '  public'#010+
  '    F','unction IndexOfName(const AName : String) : Integer;'#010+
  '    Function ItemByName(const ANAme : String) : TNamedItem;'#010+
  '    Property NamedItems[Index : Integer] : TNamedItem Read GetNamedIte'+
  'm Write SetNamedItem; default;'#010+
  '  end;'#010+
  #010+
  '  { TCommand }'#010+
  '  T','Command = Class(TNamedItem)'#010+
  '  private'#010+
  '    FAfterCommand: TNotifyEvent;'#010+
  '    FBeforeCommand: TNotifyEvent;'#010+
  '    FCommand: String;'#010+
  '    FCommandAt: TCommandAt;'#010+
  '    FDestFile: String;'#010+
  '    FIgnoreResult: Boolean;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FOptions: TStrings;'#010+
  '    FSourc','eFile: String;'#010+
  '    Function GetOptions : TStrings;'#010+
  '    Procedure SetOptions(Const Value : TStrings);'#010+
  '  Public'#010+
  '    Destructor Destroy; override;'#010+
  '    Function HaveOptions : Boolean;'#010+
  '    Function CmdLineOptions : String;'#010+
  '    Procedure ParseOpt','ions(S : String);'#010+
  '    Property SourceFile : String Read FSourceFile Write FSourceFile;'#010+
  '    Property DestFile : String Read FDestFile Write FDestFile;'#010+
  '    Property Command : String Read FCommand Write FCommand;'#010+
  '    Property Options : TString','s Read GetOptions Write SetOptions;'#010+
  '    Property At : TCommandAt Read FCommandAt Write FCommandAt;'#010+
  '    Property IgnoreResult : Boolean Read FIgnoreResult Write FIgnoreRe'+
  'sult;'#010+
  '    Property BeforeCommand : TNotifyEvent Read FBeforeCommand Wri','te '+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FOptions: String;'#010+
  '    FSourceF','ile: String;'#010+
  '  Public'#010+
  '    Property SourceFile : String Read FSourceFile Write FSourceFile;'#010+
  '    Property DestFile : String Read FDestFile Write FDestFile;'#010+
  '    Property Command : String Read FCommand Write FCommand;'#010+
  '    Property Options : Str','ing Read FOptions Write FOptions;'#010+
  '    Property At : TCommandAt Read FCommandAt Write FCommandAt;'#010+
  '    Property IgnoreResult : Boolean Read FIgnoreResult Write FIgnoreRe'+
  'sult;'#010+
  '    Property BeforeCommand : TNotifyEvent Read FBeforeCommand Write',' '+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'FBeforeCommand;'#010+
  '    Property AfterCommand : TNotifyEvent Read FAfterCommand Write FAft'+
  'erCommand;'#010+
  '  end;'#010+
  #010+
  '  { TCommands }'#010+
  #010+
  '  TCommands = Class(TNamedCollection)'#010+
  '  private'#010+
  '    FDefaultAt: TCommandAt;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    function GetCommand(const Dest : St','ring): TCommand;'#010+
=======
  '    function GetCommand(const Dest : Stri','ng): TCommand;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    function GetCommand(const Dest : Stri','ng): TCommand;'#010+
>>>>>>> origin/fixes_2_2
  '    function GetCommandItem(Index : Integer): TCommand;'#010+
  '    procedure SetCommandItem(Index : Integer; const AValue: TCommand);'+
  #010+
  '  Public'#010+
  '    Function AddCommand(Const Cmd : String) : TCommand;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function AddCommand(Const C','md,Options : String) : TCommand;'#010+
=======
  '    Function AddCommand(Const Cmd',',Options : String) : TCommand;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function AddCommand(Const Cmd',',Options : String) : TCommand;'#010+
>>>>>>> origin/fixes_2_2
  '    Function AddCommand(Const Cmd,Options,Dest,Source : String) : TCom'+
  'mand;'#010+
  '    Function AddCommand(At : TCommandAt; Const Cmd : String) : TComman'+
  'd;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function AddCommand(At : TCommandAt; Const Cmd,Options',' : String)'+
=======
  '    Function AddCommand(At : TCommandAt; Const Cmd,Options :',' String)'+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function AddCommand(At : TCommandAt; Const Cmd,Options :',' String)'+
>>>>>>> origin/fixes_2_2
  ' : TCommand;'#010+
  '    Function AddCommand(At : TCommandAt; Const Cmd,Options, Dest,Sourc'+
  'e : String) : TCommand;'#010+
  '    Property CommandItems[Index : Integer] : TCommand Read GetCommandI'+
  'tem Write SetCommandItem;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property Commands[Dest ',': String] : TCommand Read GetCommand; def'+
=======
  '    Property Commands[Dest : ','String] : TCommand Read GetCommand; def'+
>>>>>>> graemeg/fixes_2_2
=======
  '    Property Commands[Dest : ','String] : TCommand Read GetCommand; def'+
>>>>>>> origin/fixes_2_2
  'ault;'#010+
  '    Property DefaultAt : TCommandAt Read FDefaultAt Write FDefaultAt;'#010+
  '  end;'#010+
  #010+
  '  { TFPVersion }'#010+
  #010+
  '  TFPVersion = Class(TPersistent)'#010+
  '  private'#010+
  '    FMajor,'#010+
  '    FMinor,'#010+
  '    FMicro,'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FBuild    : W','ord;'#010+
=======
  '    FBuild    : Wor','d;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FBuild    : Wor','d;'#010+
>>>>>>> origin/fixes_2_2
  '    function GetAsString: String;'#010+
  '    function GetEmpty: Boolean;'#010+
  '    procedure SetAsString(const AValue: String);'#010+
  '  Public'#010+
  '   Procedure Clear;'#010+
  '   Procedure Assign(Source : TPersistent); override;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '   Function CompareVersion(AVersion : ','TFPVersion) : Integer;'#010+
=======
  '   Function CompareVersion(AVersion : TF','PVersion) : Integer;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '   Function CompareVersion(AVersion : TF','PVersion) : Integer;'#010+
>>>>>>> origin/fixes_2_2
  '   Function SameVersion(AVersion : TFPVersion) : Boolean;'#010+
  '   Property AsString : String Read GetAsString Write SetAsString;'#010+
  '   Property Empty : Boolean Read GetEmpty;'#010+
  '  Published'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '   Property Major : Word Read FMajor W','rite FMajor;'#010+
=======
  '   Property Major : Word Read FMajor Wri','te FMajor;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '   Property Major : Word Read FMajor Wri','te FMajor;'#010+
>>>>>>> origin/fixes_2_2
  '   Property Minor : Word Read FMinor Write FMinor;'#010+
  '   Property Micro : Word Read FMicro Write FMicro;'#010+
  '   Property Build : Word Read FBuild Write FBuild;'#010+
  '  end;'#010+
  #010+
  '  { TConditionalString }'#010+
  '  TConditionalString = Class'#010+
  '  private'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ','  FOSes   : TOSes;'#010+
=======
  '    ','FOSes   : TOSes;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    ','FOSes   : TOSes;'#010+
>>>>>>> origin/fixes_2_2
  '    FCPUs   : TCPUs;'#010+
  '    FValue  : String;'#010+
  '  Public'#010+
  '    Constructor Create;virtual;'#010+
  '    Property Value : String Read FValue Write FValue;'#010+
  '    Property OSes  : TOSes Read FOSes Write FOSes;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property CPUs : TCPUs Read F','CPUS Write FCPUs;'#010+
=======
  '    Property CPUs : TCPUs Read FCP','US Write FCPUs;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Property CPUs : TCPUs Read FCP','US Write FCPUs;'#010+
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  TConditionalStringClass = class of TConditionalString;'#010+
  #010+
  '  { TConditionalStrings }'#010+
  #010+
  '  TConditionalStrings = Class(TFPList)'#010+
  '  private'#010+
  '    FCSClass : TConditionalStringClass;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    function GetConditionalString(Index ',': Integer): TConditionalStri'+
=======
  '    function GetConditionalString(Index : ','Integer): TConditionalStri'+
>>>>>>> graemeg/fixes_2_2
=======
  '    function GetConditionalString(Index : ','Integer): TConditionalStri'+
>>>>>>> origin/fixes_2_2
  'ng;'#010+
  '    procedure SetConditionalString(Index : Integer; const AValue: TCon'+
  'ditionalString);'#010+
  '  Public'#010+
  '    Constructor Create(AClass:TConditionalStringClass);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function Add(Const Value : String) : TConditionalStr','ing;inline;'#010+
=======
  '    Function Add(Const Value : String) : TConditionalStrin','g;inline;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function Add(Const Value : String) : TConditionalStrin','g;inline;'#010+
>>>>>>> origin/fixes_2_2
  '    Function Add(Const Value : String;const OSes:TOSes) : TConditional'+
  'String;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function Add(Const Value : String;const CPUs:TCPUs) : TConditional'+
  'String;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ','  Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TO'+
=======
  '    ','Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TO'+
>>>>>>> graemeg/fixes_2_2
=======
  '    ','Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TO'+
>>>>>>> origin/fixes_2_2
  'Ses) : TConditionalString;'#010+
  '    Property ConditionalStrings[Index : Integer] : TConditionalString '+
  'Read GetConditionalString Write SetConditionalString; default;'#010+
  '  end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  { TD','ependency }'#010+
=======
  '  { TDep','endency }'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  { TDep','endency }'#010+
>>>>>>> origin/fixes_2_2
  '  TDependency = Class(TConditionalString)'#010+
  '  private'#010+
  '    FDependencyType : TDependencyType;'#010+
  '    // Package, Unit'#010+
  '    FTarget : TObject;'#010+
  '    FVersion : TFPVersion;'#010+
  '    FRequireChecksum : Cardinal;'#010+
  '    // Filenames, Includes'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FT','argetFileName : String;'#010+
=======
  '    FTar','getFileName : String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FTar','getFileName : String;'#010+
>>>>>>> origin/fixes_2_2
  '    Function GetVersion : string;'#010+
  '    Procedure SetVersion(const V : string);'#010+
  '  Public'#010+
  '    Constructor Create;override;'#010+
  '    Destructor Destroy;override;'#010+
  '    Property Target : TObject Read FTarget Write FTarget;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    P','roperty DependencyType : TDependencyType Read FDependencyType;'#010+
  '    Property TargetFileName : String Read FTargetFileName Write FTarge'+
  'tFileName;'#010+
  '    Property Version : String Read GetVersion Write SetVersion;'#010+
  '    Property RequireChecksum : C','ardinal Read FRequireChecksum Write '+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Pro','perty DependencyType : TDependencyType Read FDependencyType;'#010+
  '    Property TargetFileName : String Read FTargetFileName Write FTarge'+
  'tFileName;'#010+
  '    Property Version : String Read GetVersion Write SetVersion;'#010+
  '    Property RequireChecksum : Car','dinal Read FRequireChecksum Write '+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'FRequireChecksum;'#010+
  '  end;'#010+
  #010+
  '  TDependencies = Class(TConditionalStrings)'#010+
  '    function GetDependency(Index : Integer): TDependency;'#010+
  '    procedure SetDependency(Index : Integer; const AValue: TDependency'+
  ');'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ','Public'#010+
=======
  '  Pu','blic'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Pu','blic'#010+
>>>>>>> origin/fixes_2_2
  '    Function Add(Const Value : String) : TDependency;inline;'#010+
  '    Function Add(Const Value : String;const OSes:TOSes) : TDependency;'+
  'inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function Add(Const Value : String;const CPUs:TCPUs) : TDepende','nc'+
  'y;inline;'#010+
=======
  '    Function Add(Const Value : String;const CPUs:TCPUs) : TDependenc','y'+
  ';inline;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function Add(Const Value : String;const CPUs:TCPUs) : TDependenc','y'+
  ';inline;'#010+
>>>>>>> origin/fixes_2_2
  '{$endif cpu_only_overloads}'#010+
  '    Function Add(Const Value : String;const CPUs:TCPUs;const OSes:TOSe'+
  's) : TDependency;'#010+
  '    Function AddUnit(Const Value : String) : TDependency;inline;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function AddUnit(Const Value : String;const',' OSes:TOSes) : TDepen'+
=======
  '    Function AddUnit(Const Value : String;const O','Ses:TOSes) : TDepen'+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function AddUnit(Const Value : String;const O','Ses:TOSes) : TDepen'+
>>>>>>> origin/fixes_2_2
  'dency;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddUnit(Const Value : String;const CPUs:TCPUs) : TDepende'+
  'ncy;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function AddUnit(Const Value : String;const CPUs:TCPUs;const OS','e'+
  's:TOSes) : TDependency;'#010+
=======
  '    Function AddUnit(Const Value : String;const CPUs:TCPUs;const OSes',
  ':TOSes) : TDependency;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function AddUnit(Const Value : String;const CPUs:TCPUs;const OSes',
  ':TOSes) : TDependency;'#010+
>>>>>>> origin/fixes_2_2
  '    Function AddInclude(Const Value : String) : TDependency;inline;'#010+
  '    Function AddInclude(Const Value : String;const OSes:TOSes) : TDepe'+
  'ndency;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Function AddInclude(Const Valu','e : String;const CPUs:TCPUs) : TDe'+
=======
  '    Function AddInclude(Const Value ',': String;const CPUs:TCPUs) : TDe'+
>>>>>>> graemeg/fixes_2_2
=======
  '    Function AddInclude(Const Value ',': String;const CPUs:TCPUs) : TDe'+
>>>>>>> origin/fixes_2_2
  'pendency;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
  '    Function AddInclude(Const Value : String;const CPUs:TCPUs;const OS'+
  'es:TOSes) : TDependency;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property Dependencies[Index : Integer] : TDependency Read GetD','ep'+
  'endency Write SetDependency; default;'#010+
=======
  '    Property Dependencies[Index : Integer] : TDependency Read GetDep','e'+
  'ndency Write SetDependency; default;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Property Dependencies[Index : Integer] : TDependency Read GetDep','e'+
  'ndency Write SetDependency; default;'#010+
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TTarget }'#010+
  #010+
  '  TTarget = Class(TNamedItem)'#010+
  '  private'#010+
  '    FInstall : Boolean;'#010+
  '    FAfterClean: TNotifyEvent;'#010+
  '    FAfterCompile: TNotifyEvent;'#010+
  '    FBeforeClean: TNotifyEvent;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FBeforeCompil','e: TNotifyEvent;'#010+
=======
  '    FBeforeCompile:',' TNotifyEvent;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FBeforeCompile:',' TNotifyEvent;'#010+
>>>>>>> origin/fixes_2_2
  '    FCPUs: TCPUs;'#010+
  '    FOSes: TOSes;'#010+
  '    FMode: TCompilerMode;'#010+
  '    FResourceStrings: Boolean;'#010+
  '    FObjectPath,'#010+
  '    FUnitPath,'#010+
  '    FIncludePath : TConditionalStrings;'#010+
  '    FDependencies : TDependencies;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FCommands : TComman','ds;'#010+
=======
  '    FCommands : TCommands',';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FCommands : TCommands',';'#010+
>>>>>>> origin/fixes_2_2
  '    FDirectory: String;'#010+
  '    FExtension: String;'#010+
  '    FTargetSourceFileName : String;'#010+
  '    FFileType: TFileType;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FOptions: TStrings;'#010+
  '    FFPCTarget: String;'#010+
  '    FTargetState: TTargetState;'#010+
  '    FTargetType: TTargetType;'#010+
  '    function Get','Options: TStrings;'#010+
  '    procedure SetOptions(const AValue: TStrings);'#010+
  '  Protected'#010+
  '    Function GetSourceFileName : String; virtual;'#010+
  '    Function GetUnitFileName : String; virtual;'#010+
  '    Function GetObjectFileName : String; virtual;'#010+
  '    Functio','n GetRSTFileName : String; Virtual;'#010+
  '    Function GetProgramFileName(AOS : TOS) : String; Virtual;'#010+
  '  Public'#010+
  '    Constructor Create(ACollection : TCollection); override;'#010+
  '    Destructor Destroy; override;'#010+
  '    Function  GetOutputFileName (AOs :',' TOS) : String; Virtual;'#010+
  '    Function HaveOptions : Boolean;'#010+
  '    procedure SetName(const AValue: String);override;'#010+
  '    Procedure GetCleanFiles(List : TStrings; const APrefixU, APrefixB '+
  ': String; ACPU:TCPU; AOS : TOS); virtual;'#010+
  '    Procedure',' GetInstallFiles(List : TStrings; const APrefixU, APref'+
  'ixB: String; ACPU:TCPU; AOS : TOS); virtual;'#010+
  '    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); '+
  'virtual;'#010+
  '    Property Dependencies : TDependencies Read FDependencies;',#010+
  '    Property Commands : TCommands Read FCommands;'#010+
  '    Property State : TTargetState Read FTargetState;'#010+
  '    Property TargetType : TTargetType Read FTargetType Write FTargetTy'+
  'pe;'#010+
  '    Property OSes : TOSes Read FOSes Write FOSes;'#010+
  '    Property',' CPUs : TCPUs Read FCPUs Write FCPUs;'#010+
  '    Property Mode : TCompilerMode Read FMode Write FMode;'#010+
  '    Property Options : TStrings Read GetOptions Write SetOptions;'#010+
  '    Property SourceFileName: String Read GetSourceFileName ;'#010+
  '    Property Unit','FileName : String Read GetUnitFileName;'#010+
  '    Property ObjectFileName : String Read GetObjectFileName;'#010+
  '    Property RSTFileName : String Read GetRSTFileName;'#010+
  '    Property FPCTarget : String Read FFPCTarget Write FFPCTarget;'#010+
  '    Property Exten','sion : String Read FExtension Write FExtension;'#010+
  '    Property FileType : TFileType Read FFileType Write FFileType;'#010+
  '    Property Directory : String Read FDirectory Write FDirectory;'#010+
  '    Property ResourceStrings : Boolean Read FResourceStrings',' Write F'+
  'ResourceStrings;'#010+
  '    Property Install : Boolean Read FInstall Write FInstall;'#010+
  '    Property TargetSourceFileName: String Read FTargetSourceFileName;'#010+
  '    Property ObjectPath : TConditionalStrings Read FObjectPath;'#010+
  '    Property UnitPat','h : TConditionalStrings Read FUnitPath;'#010+
  '    Property IncludePath : TConditionalStrings Read FIncludePath;'#010+
  '    // Events.'#010+
  '    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FB'+
  'eforeCompile;'#010+
  '    Property AfterCompile : TNotify','Event Read FAfterCompile Write FA'+
  'fterCompile;'#010+
  '    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBefor'+
  'eClean;'#010+
  '    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterCl'+
  'ean;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FOptions: String;'#010+
  '    FFPCTarget: String;'#010+
  '    FTargetState: TTargetState;'#010+
  '    FTargetType: TTargetType;'#010+
  '  Protected'#010+
  '    Func','tion GetSourceFileName : String; virtual;'#010+
  '    Function GetUnitFileName : String; virtual;'#010+
  '    Function GetObjectFileName : String; virtual;'#010+
  '    Function GetRSTFileName : String; Virtual;'#010+
  '    Function GetProgramFileName(AOS : TOS) : String; ','Virtual;'#010+
  '  Public'#010+
  '    Constructor Create(ACollection : TCollection); override;'#010+
  '    Destructor Destroy; override;'#010+
  '    Function  GetOutputFileName (AOs : TOS) : String; Virtual;'#010+
  '    procedure SetName(const AValue: String);override;'#010+
  '    Proced','ure GetCleanFiles(List : TStrings; const APrefixU, APrefix'+
  'B : String; ACPU:TCPU; AOS : TOS); virtual;'#010+
  '    Procedure GetInstallFiles(List : TStrings; const APrefixU, APrefix'+
  'B: String; ACPU:TCPU; AOS : TOS); virtual;'#010+
  '    Procedure GetArchiveF','iles(List : TStrings; ACPU:TCPU; AOS : TOS)'+
  '; virtual;'#010+
  '    Property Dependencies : TDependencies Read FDependencies;'#010+
  '    Property Commands : TCommands Read FCommands;'#010+
  '    Property State : TTargetState Read FTargetState;'#010+
  '    Property TargetTy','pe : TTargetType Read FTargetType Write FTarget'+
  'Type;'#010+
  '    Property OSes : TOSes Read FOSes Write FOSes;'#010+
  '    Property CPUs : TCPUs Read FCPUs Write FCPUs;'#010+
  '    Property Mode : TCompilerMode Read FMode Write FMode;'#010+
  '    Property Options : String',' Read FOptions Write Foptions;'#010+
  '    Property SourceFileName: String Read GetSourceFileName ;'#010+
  '    Property UnitFileName : String Read GetUnitFileName;'#010+
  '    Property ObjectFileName : String Read GetObjectFileName;'#010+
  '    Property RSTFileName : Str','ing Read GetRSTFileName;'#010+
  '    Property FPCTarget : String Read FFPCTarget Write FFPCTarget;'#010+
  '    Property Extension : String Read FExtension Write FExtension;'#010+
  '    Property FileType : TFileType Read FFileType Write FFileType;'#010+
  '    Property Dire','ctory : String Read FDirectory Write FDirectory;'#010+
  '    Property ResourceStrings : Boolean Read FResourceStrings Write FRe'+
  'sourceStrings;'#010+
  '    Property Install : Boolean Read FInstall Write FInstall;'#010+
  '    Property TargetSourceFileName: String Rea','d FTargetSourceFileName'+
  ';'#010+
  '    Property ObjectPath : TConditionalStrings Read FObjectPath;'#010+
  '    Property UnitPath : TConditionalStrings Read FUnitPath;'#010+
  '    Property IncludePath : TConditionalStrings Read FIncludePath;'#010+
  '    // Events.'#010+
  '    Proper','ty BeforeCompile : TNotifyEvent Read FBeforeCompile Write '+
  'FBeforeCompile;'#010+
  '    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAft'+
  'erCompile;'#010+
  '    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBefor'+
  'eClean;'#010+
  '    Pro','perty AfterClean : TNotifyEvent Read FAfterClean Write FAfter'+
  'Clean;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TTargets }'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  TTargets = Clas','s(TNamedCollection)'#010+
  '  private'#010+
  '    function GetTargetItem(Index : Integer): TTarget;'#010+
  '    function GetTarget(const AName : String): TTarget;'#010+
  '    procedure SetTargetItem(Index : Integer; const AValue: TTarget);'#010+
  '  Public'#010+
  '    Function AddUnit(Co','nst AUnitName : String) : TTarget;inline;'#010+
  '    Function AddUnit(Const AUnitName : String;const OSes:TOSes) : TTar'+
  'get;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTar'+
  'get;inline;'#010+
  '{$endi','f cpu_only_overloads}'#010+
  '    Function AddUnit(Const AUnitName : String;const CPUs:TCPUs;const O'+
  'Ses:TOSes) : TTarget;'#010+
  '    Function AddImplicitUnit(Const AUnitName : String;InstallUnit:bool'+
  'ean=true) : TTarget;inline;'#010+
  '    Function AddImplicitUnit','(Const AUnitName : String;const OSes:TOS'+
  'es;InstallUnit:boolean=true) : TTarget;inline;'#010+
  '    Function AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs'+
  ';InstallUnit:boolean=true) : TTarget;inline;'#010+
  '    Function AddImplicitUnit(Const AUn','itName : String;const CPUs:TCP'+
  'Us;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;'#010+
  '    Function AddProgram(Const AProgramName : String) : TTarget;inline;'+
  #010+
  '    Function AddProgram(Const AProgramName : String;const OSes:TOSes) '+
  ': TTarget;in','line;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddProgram(Const AProgramName : String;const CPUs:TCPUs) '+
  ': TTarget;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
  '    Function AddProgram(Const AProgramName : String;const CPUs:TCPUs;c'+
  'onst OSes:TOSes) : T','Target;'#010+
  '    Function AddExampleUnit(Const AUnitName : String) : TTarget;inline'+
  ';'#010+
  '    Function AddExampleUnit(Const AUnitName : String;const OSes:TOSes)'+
  ' : TTarget;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddExampleUnit(Const AUnitName',' : String;const CPUs:TCPU'+
  's) : TTarget;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
  '    Function AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs;'+
  'const OSes:TOSes) : TTarget;'#010+
  '    Function AddExampleProgram(Const AProgramName : String) : TTarg','e'+
  't;inline;'#010+
  '    Function AddExampleProgram(Const AProgramName : String;const OSes:'+
  'TOSes) : TTarget;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddExampleProgram(Const AProgramName : String;const CPUs:'+
  'TCPUs) : TTarget;inline;'#010+
  '{$endif cpu','_only_overloads}'#010+
  '    Function AddExampleProgram(Const AProgramName : String;const CPUs:'+
  'TCPUs;const OSes:TOSes) : TTarget;'#010+
  '    Property Targets[AName : String] : TTarget Read GetTarget; default'+
  ';'#010+
  '    Property TargetItems[Index : Integer] : TT','arget Read GetTargetIt'+
  'em Write SetTargetItem;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  TTargets = Class(TNamedCollection)'#010+
  '  private'#010+
  '    function GetTargetItem(Index : Integer): TTarget;'#010+
  '    function GetTarget(const AName : String): T','Target;'#010+
  '    procedure SetTargetItem(Index : Integer; const AValue: TTarget);'#010+
  '  Public'#010+
  '    Function AddUnit(Const AUnitName : String) : TTarget;inline;'#010+
  '    Function AddUnit(Const AUnitName : String;const OSes:TOSes) : TTar'+
  'get;inline;'#010+
  '{$ifdef',' cpu_only_overloads}'#010+
  '    Function AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTar'+
  'get;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
  '    Function AddUnit(Const AUnitName : String;const CPUs:TCPUs;const O'+
  'Ses:TOSes) : TTarget;'#010+
  '    Function AddI','mplicitUnit(Const AUnitName : String;InstallUnit:bo'+
  'olean=true) : TTarget;inline;'#010+
  '    Function AddImplicitUnit(Const AUnitName : String;const OSes:TOSes'+
  ';InstallUnit:boolean=true) : TTarget;inline;'#010+
  '    Function AddImplicitUnit(Const AUnitName',' : String;const CPUs:TCP'+
  'Us;InstallUnit:boolean=true) : TTarget;inline;'#010+
  '    Function AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs'+
  ';const OSes:TOSes;InstallUnit:boolean=true) : TTarget;'#010+
  '    Function AddProgram(Const AProgramName :',' String) : TTarget;inlin'+
  'e;'#010+
  '    Function AddProgram(Const AProgramName : String;const OSes:TOSes) '+
  ': TTarget;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddProgram(Const AProgramName : String;const CPUs:TCPUs) '+
  ': TTarget;inline;'#010+
  '{$endif c','pu_only_overloads}'#010+
  '    Function AddProgram(Const AProgramName : String;const CPUs:TCPUs;c'+
  'onst OSes:TOSes) : TTarget;'#010+
  '    Function AddExampleUnit(Const AUnitName : String) : TTarget;inline'+
  ';'#010+
  '    Function AddExampleUnit(Const AUnitName : Strin','g;const OSes:TOSe'+
  's) : TTarget;inline;'#010+
  '{$ifdef cpu_only_overloads}'#010+
  '    Function AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs)'+
  ' : TTarget;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
  '    Function AddExampleUnit(Const AUnitName : String;cons','t CPUs:TCPU'+
  's;const OSes:TOSes) : TTarget;'#010+
  '    Function AddExampleProgram(Const AProgramName : String) : TTarget;'+
  'inline;'#010+
  '    Function AddExampleProgram(Const AProgramName : String;const OSes:'+
  'TOSes) : TTarget;inline;'#010+
  '{$ifdef cpu_only_overload','s}'#010+
  '    Function AddExampleProgram(Const AProgramName : String;const CPUs:'+
  'TCPUs) : TTarget;inline;'#010+
  '{$endif cpu_only_overloads}'#010+
  '    Function AddExampleProgram(Const AProgramName : String;const CPUs:'+
  'TCPUs;const OSes:TOSes) : TTarget;'#010+
  '    Prope','rty Targets[AName : String] : TTarget Read GetTarget; defau'+
  'lt;'#010+
  '    Property TargetItems[Index : Integer] : TTarget Read GetTargetItem'+
  ' Write SetTargetItem;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TSource }'#010+
  #010+
  '  TSource = Class(TNamedItem)'#010+
  '  private'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FSourceType : TSourceType;'#010+
  '    FInstallSourcePath : string;'#010+
  '    function GetInstallSourcePath: string;'#010+
  '  Public'#010+
  '    Constructor',' Create(ACollection : TCollection); override;'#010+
  '    Destructor Destroy; override;'#010+
  '    Procedure GetInstallFiles(List : TStrings); virtual;'#010+
  '    property SourceType : TSourceType read FSourceType;'#010+
  '    property InstallSourcePath : string read Ge','tInstallSourcePath;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FSourceType : TSou','rceType;'#010+
  '  Public'#010+
  '    Constructor Create(ACollection : TCollection); override;'#010+
  '    Destructor Destroy; override;'#010+
  '    property SourceType : TSourceType read FSourceType;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TSources }'#010+
  #010+
  '  TSources = Class(TNamedCollection)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  private'#010+
  '    function GetSourceItem(Index : Integer): TSource;'#010+
  '    procedure SetSourceItem(Index : Integer; const AValue: TSource);'#010+
  '  public'#010+
  '    Function AddD','oc(const AFiles : String) : TSource;'#010+
  '    Function AddDoc(const AFiles : String; AInstallSourcePath : String'+
  ') : TSource;'#010+
  '    Function AddSrc(const AFiles : String) : TSource;'#010+
  '    Function AddExample(const AFiles : String) : TSource;'#010+
  '    Func','tion AddExample(const AFiles : String; AInstallSourcePath : '+
  'String) : TSource;'#010+
  '    Function AddTest(const AFiles : String) : TSource;'#010+
  '    procedure AddDocFiles(const AFileMask: string; Recursive: boolean '+
  '= False; AInstallSourcePath : String',' = '#039#039');'#010+
  '    procedure AddSrcFiles(const AFileMask: string; Recursive: boolean '+
  '= False);'#010+
  '    procedure AddExampleFiles(const AFileMask: string; Recursive: bool'+
  'ean = False; AInstallSourcePath : String = '#039#039');'#010+
  '    procedure AddTestFiles(const A','FileMask: string; Recursive: boole'+
  'an = False);'#010+
  '    Property SourceItems[Index : Integer] : TSource Read GetSourceItem'+
  ' Write SetSourceItem;default;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  private'#010,
  '    function GetSourceItem(Index : Integer): TSource;'#010+
  '    procedure SetSourceItem(Index : Integer; const AValue: TSource);'#010+
  '  public'#010+
  '    Function AddDoc(const AFiles : String) : TSource;'#010+
  '    Function AddSrc(const AFiles : String) : TSource;'#010,
  '    Function AddExample(const AFiles : String) : TSource;'#010+
  '    Function AddTest(const AFiles : String) : TSource;'#010+
  '    procedure AddDocFiles(const AFileMask: string; Recursive: boolean '+
  '= False);'#010+
  '    procedure AddSrcFiles(const AFileMask: stri','ng; Recursive: boolea'+
  'n = False);'#010+
  '    procedure AddExampleFiles(const AFileMask: string; Recursive: bool'+
  'ean = False);'#010+
  '    procedure AddTestFiles(const AFileMask: string; Recursive: boolean'+
  ' = False);'#010+
  '    Property SourceItems[Index : Integer] ',': TSource Read GetSourceIt'+
  'em Write SetSourceItem;default;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TPackage }'#010+
  #010+
  '  TPackage = Class(TNamedItem)'#010+
  '  private'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FAfterArchive: TNotifyEv','ent;'#010+
  '    FAfterClean: TNotifyEvent;'#010+
  '    FAfterCompile: TNotifyEvent;'#010+
  '    FAfterInstall: TNotifyEvent;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FAfterArchive: TNotifyEvent;'#010+
  '    FAfterClean: TNotifyEvent;'#010+
  '    FAfterCompile: TNotifyEvent;'#010+
  '    FAfterInstall: T','NotifyEvent;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    FAfterManifest: TNotifyEvent;'#010+
  '    FBeforeArchive: TNotifyEvent;'#010+
  '    FBeforeClean: TNotifyEvent;'#010+
  '    FBeforeCompile: TNotifyEvent;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    ','FBeforeInstall: TNotifyEvent;'#010+
  '    FBeforeManifest: TNotifyEvent;'#010+
  '    FUnitPath,'#010+
  '    FObjectPath,'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FBeforeInstall: TNotifyEvent;'#010+
  '    FBeforeManifest: TNotifyEvent;'#010+
  '    FUnitPath,'#010+
  '    FObje','ctPath,'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    FIncludePath,'#010+
  '    FSourcePath,'#010+
  '    FExamplePath,'#010+
  '    FTestPath,'#010+
  '    FCleanFiles,'#010+
  '    FInstallFiles : TConditionalStrings;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FDependencies',' : TDependencies;'#010+
  '    FCPUs: TCPUs;'#010+
  '    FOSes: TOSes;'#010+
  '    FTargetState: TTargetState;'#010+
  '    FTargets: TTargets;'#010+
  '    FSources: TSources;'#010+
  '    FDirectory: String;'#010+
  '    FOptions: TStrings;'#010+
  '    FFileName: String;'#010+
  '    FAuthor: String;'#010+
  '    FLicense: ','String;'#010+
  '    FHomepageURL: String;'#010+
  '    FDownloadURL: String;'#010+
  '    FVersion: TFPVersion;'#010+
  '    FEmail : String;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FDependencies : TDependencies;'#010+
  '    FCPUs: TCPUs;'#010+
  '    FOSes: TOSes;'#010+
  '    FTargetState: TTargetState;'#010+
  '   ',' FTargets: TTargets;'#010+
  '    FSources: TSources;'#010+
  '    FDirectory: String;'#010+
  '    FOptions: String;'#010+
  '    FFileName: String;'#010+
  '    FAuthor: String;'#010+
  '    FLicense: String;'#010+
  '    FHomepageURL: String;'#010+
  '    FDownloadURL: String;'#010+
  '    FVersion: TFPVersion;'#010+
  '    F','Email : String;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    FNeedLibC : Boolean;'#010+
  '    FCommands : TCommands;'#010+
  '    FDescriptionFile : String;'#010+
  '    FDescription : String;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FInstalledChecksum ',': Cardinal;'#010+
  '    // Cached directory of installed packages'#010+
  '    FUnitDir : String;'#010+
  '    Function GetDescription : string;'#010+
  '    Function GetFileName : string;'#010+
  '    function GetOptions: TStrings;'#010+
  '    Function GetVersion : string;'#010+
  '    procedure Set','Options(const AValue: TStrings);'#010+
  '    Procedure SetVersion(const V : string);'#010+
  '  Protected'#010+
  '    procedure SetName(const AValue: String);override;'#010+
  '    procedure LoadUnitConfigFromFile(Const AFileName: String);'#010+
  '    procedure SaveUnitConfigToFile','(Const AFileName: String;ACPU:TCPU'+
  ';AOS:TOS);'#010+
  '  Public'#010+
  '    constructor Create(ACollection: TCollection); override;'#010+
  '    destructor destroy; override;'#010+
  '    Function HaveOptions : Boolean;'#010+
  '    Function  GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):St','ring;'#010+
  '    Function  GetBinOutputDir(ACPU:TCPU; AOS : TOS) : String;'#010+
  '    Procedure GetCleanFiles(List : TStrings; ACPU:TCPU; AOS : TOS); vi'+
  'rtual;'#010+
  '    procedure GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU'+
  ':TCPU; AOS : TOS);'#010+
  '    pr','ocedure GetInstallSourceFiles(List: TStrings;Types : TSourceTy'+
  'pes);'#010+
  '    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); '+
  'virtual;'#010+
  '    Procedure GetManifest(Manifest : TStrings);'#010+
  '    Property Version : String Read GetVersion',' Write SetVersion;'#010+
  '    Property FileName : String Read GetFileName Write FFileName;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FInstalledChecksum : Cardinal;'#010+
  '    // Cached directory of installed packages'#010+
  '    FUnitDir : String;'#010+
  '    Functi','on GetDescription : string;'#010+
  '    Function GetFileName : string;'#010+
  '    Function GetVersion : string;'#010+
  '    Procedure SetVersion(const V : string);'#010+
  '  Protected'#010+
  '    procedure SetName(const AValue: String);override;'#010+
  '    procedure LoadUnitConfigFromF','ile(Const AFileName: String);'#010+
  '    procedure SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;A'+
  'OS:TOS);'#010+
  '  Public'#010+
  '    constructor Create(ACollection: TCollection); override;'#010+
  '    destructor destroy; override;'#010+
  '    Function  GetUnitsOutput','Dir(ACPU:TCPU; AOS : TOS):String;'#010+
  '    Function  GetBinOutputDir(ACPU:TCPU; AOS : TOS) : String;'#010+
  '    Procedure GetCleanFiles(List : TStrings; ACPU:TCPU; AOS : TOS); vi'+
  'rtual;'#010+
  '    procedure GetInstallFiles(List: TStrings;Types : TTargetTypes;A','C'+
  'PU:TCPU; AOS : TOS);'#010+
  '    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); '+
  'virtual;'#010+
  '    Procedure GetManifest(Manifest : TStrings);'#010+
  '    Property Version : String Read GetVersion Write SetVersion;'#010+
  '    Property FileName : Str','ing Read GetFileName Write FFileName;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Property HomepageURL : String Read FHomepageURL Write FHomepageURL'+
  ';'#010+
  '    Property DownloadURL : String Read FDownloadURL Write FDownloadURL'+
  ';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property',' Email : String Read FEmail Write FEmail;'#010+
  '    Property Author : String Read FAuthor Write FAuthor;'#010+
  '    Property License : String Read FLicense Write FLicense;'#010+
  '    Property Directory : String Read FDirectory Write FDirectory;'#010+
  '    Property De','scription : String Read GetDescription Write FDescrip'+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Property Email : String Read FEmail Write FEmail;'#010+
  '    ','Property Author : String Read FAuthor Write FAuthor;'#010+
  '    Property License : String Read FLicense Write FLicense;'#010+
  '    Property Directory : String Read FDirectory Write FDirectory;'#010+
  '    Property Description : String Read GetDescription Write F','Descrip'+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'tion;'#010+
  '    Property DescriptionFile : String Read FDescriptionFile Write FDes'+
  'criptionFile;'#010+
  '    Property InstalledChecksum : Cardinal Read FInstalledChecksum Writ'+
  'e FInstalledChecksum;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    /','/ Compiler options.'#010+
  '    Property OSes : TOSes Read FOSes Write FOSes;'#010+
  '    Property CPUs : TCPUs Read FCPUs Write FCPUs;'#010+
  '    Property NeedLibC : Boolean Read FNeedLibC Write FNeedLibC;'#010+
  '    Property Options: TStrings Read GetOptions Write Set','Options;'#010+
  '    Property UnitPath : TConditionalStrings Read FUnitPath;'#010+
  '    Property ObjectPath : TConditionalStrings Read FObjectPath;'#010+
  '    Property IncludePath : TConditionalStrings Read FIncludePath;'#010+
  '    Property SourcePath : TConditionalStr','ings Read FSourcePath;'#010+
  '    Property ExamplePath : TConditionalStrings Read FExamplePath;'#010+
  '    Property TestPath : TConditionalStrings Read FTestPath;'#010+
  '    // Targets and dependencies'#010+
  '    Property InstallFiles : TConditionalStrings Read FInsta','llFiles;'#010+
  '    Property CleanFiles : TConditionalStrings Read FCleanFiles;'#010+
  '    Property Dependencies : TDependencies Read FDependencies;'#010+
  '    Property Commands : TCommands Read FCommands;'#010+
  '    Property State : TTargetState Read FTargetState;'#010+
  '  ','  Property Targets : TTargets Read FTargets;'#010+
  '    Property Sources : TSources Read FSources;'#010+
  '    Property UnitDir : String Read FUnitDir Write FUnitDir;'#010+
  '    // events'#010+
  '    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FB'+
  'efor','eCompile;'#010+
  '    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAft'+
  'erCompile;'#010+
  '    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FB'+
  'eforeInstall;'#010+
  '    Property AfterInstall : TNotifyEvent Read FAfterInstall Write',' FA'+
  'fterInstall;'#010+
  '    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBefor'+
  'eClean;'#010+
  '    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterCl'+
  'ean;'#010+
  '    Property BeforeArchive : TNotifyEvent Read FBeforeArchive Write FB'+
  'e','foreArchive;'#010+
  '    Property AfterArchive : TNotifyEvent Read FAfterArchive Write FAft'+
  'erArchive;'#010+
  '    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write '+
  'FBeforeManifest;'#010+
  '    Property AfterManifest : TNotifyEvent Read FAfterManife','st Write '+
  'FAfterManifest;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    // Compiler options.'#010+
  '    Property OSes : TOSes ','Read FOSes Write FOSes;'#010+
  '    Property CPUs : TCPUs Read FCPUs Write FCPUs;'#010+
  '    Property NeedLibC : Boolean Read FNeedLibC Write FNeedLibC;'#010+
  '    Property Options: String Read FOptions Write FOptions;'#010+
  '    Property UnitPath : TConditionalStrings',' Read FUnitPath;'#010+
  '    Property ObjectPath : TConditionalStrings Read FObjectPath;'#010+
  '    Property IncludePath : TConditionalStrings Read FIncludePath;'#010+
  '    Property SourcePath : TConditionalStrings Read FSourcePath;'#010+
  '    Property ExamplePath : TC','onditionalStrings Read FExamplePath;'#010+
  '    Property TestPath : TConditionalStrings Read FTestPath;'#010+
  '    // Targets and dependencies'#010+
  '    Property InstallFiles : TConditionalStrings Read FInstallFiles;'#010+
  '    Property CleanFiles : TConditionalStrin','gs Read FCleanFiles;'#010+
  '    Property Dependencies : TDependencies Read FDependencies;'#010+
  '    Property Commands : TCommands Read FCommands;'#010+
  '    Property State : TTargetState Read FTargetState;'#010+
  '    Property Targets : TTargets Read FTargets;'#010+
  '    Pro','perty Sources : TSources Read FSources;'#010+
  '    Property UnitDir : String Read FUnitDir Write FUnitDir;'#010+
  '    // events'#010+
  '    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FB'+
  'eforeCompile;'#010+
  '    Property AfterCompile : TNotifyEvent R','ead FAfterCompile Write FA'+
  'fterCompile;'#010+
  '    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FB'+
  'eforeInstall;'#010+
  '    Property AfterInstall : TNotifyEvent Read FAfterInstall Write FAft'+
  'erInstall;'#010+
  '    Property BeforeClean : TNotifyEv','ent Read FBeforeClean Write FBef'+
  'oreClean;'#010+
  '    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterCl'+
  'ean;'#010+
  '    Property BeforeArchive : TNotifyEvent Read FBeforeArchive Write FB'+
  'eforeArchive;'#010+
  '    Property AfterArchive : TNotifyEven','t Read FAfterArchive Write FA'+
  'fterArchive;'#010+
  '    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write '+
  'FBeforeManifest;'#010+
  '    Property AfterManifest : TNotifyEvent Read FAfterManifest Write FA'+
  'fterManifest;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TPackages }'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  TPackages = Class(TNamedCollection)'#010+
  '  private'#010+
  '    function GetPackage(const AName : String): TPackage;'#010+
  '    function GetPackageItem(AIndex : Integer): TPackage;'#010+
  '    procedure SetPackageItem','(AIndex : Integer; const AValue: TPackag'+
  'e);'#010+
  '  Public'#010+
  '    Function AddPackage(Const AName : String) : TPackage;'#010+
  '    Property Packages[AName : String] : TPackage Read GetPackage ; Def'+
  'ault;'#010+
  '    Property PackageItems[AIndex : Integer] : TPackag','e Read GetPacka'+
  'geItem Write SetPackageItem;'#010+
  '  end;'#010+
  #010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  ','TPackages = Class(TNamedCollection)'#010+
  '  private'#010+
  '    function GetPackage(const AName : String): TPackage;'#010+
  '    function GetPackageItem(AIndex : Integer): TPackage;'#010+
  '    procedure SetPackageItem(AIndex : Integer; const AValue: TPackage)'+
  ';'#010+
  '  Public',#010+
  '    Function AddPackage(Const AName : String) : TPackage;'#010+
  '    Property Packages[AName : String] : TPackage Read GetPackage ; Def'+
  'ault;'#010+
  '    Property PackageItems[AIndex : Integer] : TPackage Read GetPackage'+
  'Item Write SetPackageItem;'#010+
  '  end;'#010+
  #010,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  { TCustomDefaults }'#010+
  #010+
  '  TCustomDefaults = Class(TPersistent)'#010+
  '  Private'#010+
  '    FArchive: String;'#010+
  '    FCompiler: String;'#010+
  '    FCopy: String;'#010+
  '    FMkDir: String;'#010+
  '    FMove: String;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FOptions:',' TStrings;'#010+
  '    FCPU: TCPU;'#010+
  '    FOS: TOS;'#010+
  '    FMode : TCompilerMode;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FOptions: String;'#010+
  '    FCPU: TCPU;'#010+
  '    FOS: TOS;'#010+
  '    FMode : T','CompilerMode;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    FCompilerVersion : String;'#010+
  '    FPrefix: String;'#010+
  '    FLocalUnitDir,'#010+
  '    FGlobalUnitDir,'#010+
  '    FBaseInstallDir,'#010+
  '    FUnitInstallDir,'#010+
  '    FBinInstallDir,'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FDocInstallDir,',#010+
  '    FExamplesInstallDir : String;'#010+
  '    FRemove: String;'#010+
=======
  '    FDocInstallDir,'#010+
  '    FExamplesInstallDir : String;'#010+
  '    FRemove: String',';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FDocInstallDir,'#010+
  '    FExamplesInstallDir : String;'#010+
  '    FRemove: String',';'#010+
>>>>>>> origin/fixes_2_2
  '    FTarget: String;'#010+
  '    FUnixPaths: Boolean;'#010+
  '    FNoFPCCfg: Boolean;'#010+
  '    function GetLocalUnitDir: String;'#010+
  '    function GetGlobalUnitDir: String;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    function GetBaseInstallDir: Strin','g;'#010+
  '    function GetBinInstallDir: String;'#010+
  '    function GetCompiler: String;'#010+
  '    function GetDocInstallDir: String;'#010+
  '    function GetExamplesInstallDir: String;'#010+
  '    function GetOptions: TStrings;'#010+
  '    function GetUnitInstallDir: String;'#010+
  '    pr','ocedure SetLocalUnitDir(const AValue: String);'#010+
  '    procedure SetGlobalUnitDir(const AValue: String);'#010+
  '    procedure SetBaseInstallDir(const AValue: String);'#010+
  '    procedure SetCPU(const AValue: TCPU);'#010+
  '    procedure SetOptions(const AValue: TSt','rings);'#010+
  '    procedure SetOS(const AValue: TOS);'#010+
  '    procedure SetPrefix(const AValue: String);'#010+
  '    procedure SetTarget(const AValue: String);'#010+
  '  Protected'#010+
  '    procedure RecalcTarget;'#010+
  '    Function CmdLineOptions : String;'#010+
  '  Public'#010+
  '    Constru','ctor Create;'#010+
  '    Procedure InitDefaults;'#010+
  '    Function HaveOptions: Boolean;'#010+
  '    procedure CompilerDefaults; virtual;'#010+
  '    Procedure LocalInit(Const AFileName : String);'#010+
  '    Procedure LoadFromFile(Const AFileName : String);'#010+
  '    Procedure Save','ToFile(Const AFileName : String);'#010+
  '    procedure SaveToStream(S : TStream);virtual;'#010+
  '    procedure LoadFromStream(S : TStream);virtual;'#010+
  '    // Compile Information'#010+
  '    Property Target : String Read FTarget Write SetTarget;'#010+
  '    Property OS : TO','S Read FOS Write SetOS;'#010+
  '    Property CPU : TCPU Read FCPU Write SetCPU;'#010+
  '    Property Mode : TCompilerMode Read FMode Write FMode;'#010+
  '    Property UnixPaths : Boolean Read FUnixPaths Write FUnixPaths;'#010+
  '    Property Options : TStrings Read GetOpt','ions Write SetOptions;   '+
  ' // Default compiler options.'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    function GetBaseInstallDir: String;'#010+
  '    function GetBinInstallDir: String;'#010+
  '    function',' GetCompiler: String;'#010+
  '    function GetDocInstallDir: String;'#010+
  '    function GetExamplesInstallDir: String;'#010+
  '    function GetUnitInstallDir: String;'#010+
  '    procedure SetLocalUnitDir(const AValue: String);'#010+
  '    procedure SetGlobalUnitDir(const AValu','e: String);'#010+
  '    procedure SetBaseInstallDir(const AValue: String);'#010+
  '    procedure SetCPU(const AValue: TCPU);'#010+
  '    procedure SetOS(const AValue: TOS);'#010+
  '    procedure SetPrefix(const AValue: String);'#010+
  '    procedure SetTarget(const AValue: String',');'#010+
  '  Protected'#010+
  '    procedure RecalcTarget;'#010+
  '  Public'#010+
  '    Constructor Create;'#010+
  '    Procedure InitDefaults;'#010+
  '    procedure CompilerDefaults; virtual;'#010+
  '    Procedure LocalInit(Const AFileName : String);'#010+
  '    Procedure LoadFromFile(Const AFileName :',' String);'#010+
  '    Procedure SaveToFile(Const AFileName : String);'#010+
  '    procedure SaveToStream(S : TStream);virtual;'#010+
  '    procedure LoadFromStream(S : TStream);virtual;'#010+
  '    // Compile Information'#010+
  '    Property Target : String Read FTarget Write Set','Target;'#010+
  '    Property OS : TOS Read FOS Write SetOS;'#010+
  '    Property CPU : TCPU Read FCPU Write SetCPU;'#010+
  '    Property Mode : TCompilerMode Read FMode Write FMode;'#010+
  '    Property UnixPaths : Boolean Read FUnixPaths Write FUnixPaths;'#010+
  '    Property Op','tions : String Read FOptions Write FOptions;    // De'+
  'fault compiler options.'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Property NoFPCCfg : Boolean Read FNoFPCCfg Write FNoFPCCfg;'#010+
  '    // paths etc.'#010+
  '    Property LocalUnitDir : String Read GetLocalUnitDir Write SetLocal'+
  'UnitDir;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property GlobalUnitD','ir : String Read GetGlobalUnitDir Write SetG'+
=======
  '  ','  Property GlobalUnitDir : String Read GetGlobalUnitDir Write SetG'+
>>>>>>> graemeg/fixes_2_2
=======
  '  ','  Property GlobalUnitDir : String Read GetGlobalUnitDir Write SetG'+
>>>>>>> origin/fixes_2_2
  'lobalUnitDir;'#010+
  '    Property Prefix : String Read FPrefix Write SetPrefix;'#010+
  '    Property BaseInstallDir : String Read GetBaseInstallDir Write SetB'+
  'aseInstallDir;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property UnitInstallDir : String R','ead GetUnitInstallDir Write FU'+
=======
  '    Property Uni','tInstallDir : String Read GetUnitInstallDir Write FU'+
>>>>>>> graemeg/fixes_2_2
=======
  '    Property Uni','tInstallDir : String Read GetUnitInstallDir Write FU'+
>>>>>>> origin/fixes_2_2
  'nitInstallDir;'#010+
  '    Property BinInstallDir : String Read GetBinInstallDir Write FBinIn'+
  'stallDir;'#010+
  '    Property DocInstallDir : String Read GetDocInstallDir Write FDocIn'+
  'stallDir;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Property ExamplesInstallDir : S','tring Read GetExamplesInstallDir '+
  'Write FExamplesInstallDir;'#010+
  '    // Command tools. If not set, internal commands  will be used.'#010+
  '    Property Compiler : String Read GetCompiler Write FCompiler; // Co'+
  'mpiler. Defaults to fpc'#010+
  '    Property Copy :',' String Read FCopy Write FCopy;             // co'+
  'py $(FILES) to $(DEST)'#010+
  '    Property Move : String Read FMove Write FMove;             // Move'+
  ' $(FILES) to $(DEST)'#010+
  '    Property Remove : String Read FRemove Write FRemove;       // Dele'+
  'te $(FI','LES)'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Property ','ExamplesInstallDir : String Read GetExamplesInstallDir '+
  'Write FExamplesInstallDir;'#010+
  '    // Command tools. If not set, internal commands  will be used.'#010+
  '    Property Compiler : String Read GetCompiler Write FCompiler; // Co'+
  'mpiler. Defaults to f','pc'#010+
  '    Property Copy : String Read FCopy Write FCopy;             // copy'+
  ' $(FILES) to $(DEST)'#010+
  '    Property Move : String Read FMove Write FMove;             // Move'+
  ' $(FILES) to $(DEST)'#010+
  '    Property Remove : String Read FRemove Write FRemove',';       // De'+
  'lete $(FILES)'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Property MkDir : String Read FMkDir write FMkDir;          // Make'+
  ' $(DIRECTORY)'#010+
  '    Property Archive : String Read FArchive Write FArchive;    // zip '+
  '$(ARCHIVE) $(FILESORDIRS)'#010+
  '  end;'#010+
  #010+
  '  { TBasicDefaults }'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  TBasicDefaults = Class(','TCustomDefaults)'#010+
=======
  '  T','BasicDefaults = Class(TCustomDefaults)'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  T','BasicDefaults = Class(TCustomDefaults)'#010+
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TFPCDefaults }'#010+
  #010+
  '  TFPCDefaults = Class(TCustomDefaults)'#010+
  '  public'#010+
  '    procedure CompilerDefaults; override;'#010+
  '  end;'#010+
  #010+
  '  { TBuildEngine }'#010+
  #010+
  '  TBuildEngine = Class(TComponent)'#010+
  '  private'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    // general variables'#010+
  '    FC','ompiler : String;'#010+
=======
  '    // ge','neral variables'#010+
  '    FCompiler : String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    // ge','neral variables'#010+
  '    FCompiler : String;'#010+
>>>>>>> origin/fixes_2_2
  '    FStartDir : String;'#010+
  '    FForceCompile : Boolean;'#010+
  '    FListMode : Boolean;'#010+
  '{$ifdef HAS_UNIT_ZIPPER}'#010+
  '    FZipFile: TZipper;'#010+
  '{$endif HAS_UNIT_ZIPPER}'#010+
  '    FExternalPackages : TPackages;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    // Logging'#010+
  '    FLogPrefix : Stri','ng;'#010+
=======
  '    // Logging',#010+
  '    FLogPrefix : String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    // Logging',#010+
  '    FLogPrefix : String;'#010+
>>>>>>> origin/fixes_2_2
  '    // Events'#010+
  '    FOnLog: TLogEvent;'#010+
  '    FAfterArchive: TNotifyEvent;'#010+
  '    FAfterClean: TNotifyEvent;'#010+
  '    FAfterCompile: TNotifyEvent;'#010+
  '    FAfterInstall: TNotifyEvent;'#010+
  '    FAfterManifest: TNotifyEvent;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FBeforeArchive: TNotifyEvent;'#010+
  ' ','   FBeforeClean: TNotifyEvent;'#010+
=======
  '    FBeforeAr','chive: TNotifyEvent;'#010+
  '    FBeforeClean: TNotifyEvent;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FBeforeAr','chive: TNotifyEvent;'#010+
  '    FBeforeClean: TNotifyEvent;'#010+
>>>>>>> origin/fixes_2_2
  '    FBeforeCompile: TNotifyEvent;'#010+
  '    FBeforeInstall: TNotifyEvent;'#010+
  '    FBeforeManifest: TNotifyEvent;'#010+
  '  Protected'#010+
  '    Procedure Error(const Msg : String);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Procedure Error(const Fmt : String; const Args : ','Array of const)'+
=======
  '    Procedure Error(const Fmt :',' String; const Args : Array of const)'+
>>>>>>> graemeg/fixes_2_2
=======
  '    Procedure Error(const Fmt :',' String; const Args : Array of const)'+
>>>>>>> origin/fixes_2_2
  ';'#010+
  '    // Internal copy/delete/move/archive/mkdir files'#010+
  '    Function  SysDirectoryExists(const ADir:string):Boolean;'#010+
  '    Function  SysFileExists(const AFileName:string):Boolean;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Procedure SysCopyFile(Const Src,Dest : Strin','g); virtual;'#010+
  '    Procedure SysMoveFile(Const Src,Dest : String); virtual;'#010+
  '    Procedure SysDeleteFile(Const AFileName : String); virtual;'#010+
  '    Procedure SysArchiveFiles(List : TStrings; Const AFileName : Strin'+
  'g); virtual;'#010+
  '    procedure LogIn','dent;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Procedure SysCopyFile(','Const Src,Dest : String); virtual;'#010+
  '    Procedure SysMoveFile(Const Src,Dest : String); virtual;'#010+
  '    Procedure SysDeleteFile(Const AFileName : String); virtual;'#010+
  '    Procedure SysArchiveFiles(List : TStrings; Const AFileName : Strin'+
  'g); virtua','l;'#010+
  '    procedure LogIndent;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    procedure LogUnIndent;'#010+
  '    Procedure Log(Level : TVerboseLevel; Const Msg : String);'#010+
  '    Procedure Log(Level : TVerboseLevel; Const Fmt : String; const Arg'+
  's : Array Of Const);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Procedure EnterDir(ADir : String);'#010+
  '    Function Ge','tCompiler : String;'#010+
  '    Function InstallPackageFiles(APAckage : TPackage; tt : TTargetType'+
  '; Const Dest : String):Boolean;'#010+
  '    Function InstallPackageSourceFiles(APAckage : TPackage; tt : TSour'+
  'ceType; Const Dest : String):Boolean;'#010+
  '    Functi','on FileNewer(const Src,Dest : String) : Boolean;'#010+
  '    Procedure LogSearchPath(const ASearchPathName:string;Path:TConditi'+
  'onalStrings; ACPU:TCPU;AOS:TOS);'#010+
  '    Function FindFileInPath(Path:TConditionalStrings; AFileName:String'+
  '; var FoundPath:St','ring;ACPU:TCPU;AOS:TOS):Boolean;'#010+
  #010+
  '    //package commands'#010+
  '    Procedure ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;'+
  'DoChangeDir:boolean=true);'#010+
  '    function  GetUnitDir(APackage:TPackage):String;'#010+
  '    procedure AddDependencyInclude','Paths(L:TStrings;ATarget: TTarget)'+
  ';'#010+
  '    procedure AddDependencyUnitPaths(L:TStrings;APackage: TPackage);'#010+
  '  Public'#010+
  '    Constructor Create(AOwner : TComponent); override;'#010+
  '    destructor Destroy;override;'#010+
  '    // Public Copy/delete/Move/Archive','/Mkdir Commands.'#010+
  '    Procedure ExecuteCommand(const Cmd,Args : String; IgnoreError : Bo'+
  'olean = False); virtual;'#010+
  '    Procedure CmdCopyFiles(List : TStrings; Const DestDir : String);'#010+
  '    Procedure CmdCreateDir(const DestDir : String);'#010+
  '    Pro','cedure CmdMoveFiles(List : TStrings; Const DestDir : String);'+
  #010+
  '    Procedure CmdDeleteFiles(List : TStrings);'#010+
  '    Procedure CmdArchiveFiles(List : TStrings; Const ArchiveFile : Str'+
  'ing);'#010+
  '    Procedure ExecuteCommands(Commands : TCommands; At ',': TCommandAt)'+
  ';'#010+
  '    // Dependency commands'#010+
  '    Function  DependencyOK(ADependency : TDependency) : Boolean;'#010+
  '    // Target commands'#010+
  '    Function  GetCompilerCommand(APackage : TPackage; ATarget : TTarge'+
  't) : String;'#010+
  '    Function  TargetOK(ATar','get : TTarget) : Boolean;'#010+
  '    Function  NeedsCompile(APackage:TPackage; ATarget : TTarget) : Boo'+
  'lean;'#010+
  '    Procedure Compile(APackage:TPackage; ATarget : TTarget);  virtual;'+
  #010+
  '    Procedure MaybeCompile(APackage:TPackage; ATarget: TTarget);'#010+
  '  ','  Procedure CompileDependencies(APackage:TPackage; ATarget: TTarge'+
  't);'#010+
  '    // Package commands'#010+
  '{    Function  GetPackageDir(APackage : TPackage; AbsolutePath : Boole'+
  'an = False) : String;'#010+
  '    Function  GetUnitsOutputDir(APackage : TPackage; A','bsolutePath : '+
  'Boolean = False) : String;'#010+
  '    Function  GetBinOutputDir(APackage : TPackage; AbsolutePath : Bool'+
  'ean = False) : String;  }'#010+
  '    Function  PackageOK(APackage : TPackage) : Boolean; virtual;'#010+
  '    Procedure DoBeforeCompile(APackage',' : TPackage);virtual;'#010+
  '    Procedure DoAfterCompile(APackage : TPackage);virtual;'#010+
  '    Procedure DoBeforeInstall(APackage : TPackage);virtual;'#010+
  '    Procedure DoAfterInstall(APackage : TPackage);virtual;'#010+
  '    Procedure DoBeforeArchive(APackage :',' TPackage);virtual;'#010+
  '    Procedure DoAfterArchive(APackage : TPackage);virtual;'#010+
  '    Procedure DoBeforeClean(APackage : TPackage);virtual;'#010+
  '    Procedure DoAfterClean(APackage : TPackage);virtual;'#010+
  '    Function  NeedsCompile(APackage : TPackage',') : Boolean; virtual;'#010+
  '    Procedure Compile(APackage : TPackage);'#010+
  '    Procedure MaybeCompile(APackage:TPackage);'#010+
  '    Procedure Install(APackage : TPackage);'#010+
  '    Procedure Archive(APackage : TPackage);'#010+
  '    Procedure Manifest(APackage : TPack','age);'#010+
  '    Procedure Clean(APackage : TPackage);'#010+
  '    Procedure CompileDependencies(APackage : TPackage);'#010+
  '    Function  CheckExternalPackage(Const APackageName : String):TPacka'+
  'ge;'#010+
  '    procedure CreateOutputDir(APackage: TPackage);'#010+
  '    // Pack','ages commands'#010+
  '    Procedure Compile(Packages : TPackages);'#010+
  '    Procedure Install(Packages : TPackages);'#010+
  '    Procedure Archive(Packages : TPackages);'#010+
  '    procedure Manifest(Packages: TPackages);'#010+
  '    Procedure Clean(Packages : TPackages);'#010+
  '   ',' Property ListMode : Boolean Read FListMode Write FListMode;'#010+
  '    Property ForceCompile : Boolean Read FForceCompile Write FForceCom'+
  'pile;'#010+
  '    Property ExternalPackages: TPackages Read FExternalPackages;'#010+
  '    // Events'#010+
  '    Property BeforeCompi','le : TNotifyEvent Read FBeforeCompile Write '+
  'FBeforeCompile;'#010+
  '    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAft'+
  'erCompile;'#010+
  '    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FB'+
  'eforeInstall;'#010+
  '    Property Af','terInstall : TNotifyEvent Read FAfterInstall Write FA'+
  'fterInstall;'#010+
  '    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBefor'+
  'eClean;'#010+
  '    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterCl'+
  'ean;'#010+
  '    Property BeforeAr','chive : TNotifyEvent Read FBeforeArchive Write '+
  'FBeforeArchive;'#010+
  '    Property AfterArchive : TNotifyEvent Read FAfterArchive Write FAft'+
  'erArchive;'#010+
  '    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write '+
  'FBeforeManifest;'#010+
  '    Prope','rty AfterManifest : TNotifyEvent Read FAfterManifest Write '+
  'FAfterManifest;'#010+
  '    Property OnLog : TLogEvent Read FOnLog Write FOnlog;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Procedure EnterDir(ADir : St','ring);'#010+
  '    Function GetCompiler : String;'#010+
  '    Function InstallPackageFiles(APAckage : TPackage; tt : TTargetType'+
  '; Const Dest : String):Boolean;'#010+
  '    Function FileNewer(const Src,Dest : String) : Boolean;'#010+
  '    Procedure LogSearchPath(const ASe','archPathName:string;Path:TCondi'+
  'tionalStrings; ACPU:TCPU;AOS:TOS);'#010+
  '    Function FindFileInPath(Path:TConditionalStrings; AFileName:String'+
  '; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;'#010+
  #010+
  '    //package commands'#010+
  '    Procedure ResolveFileName','s(APackage : TPackage; ACPU:TCPU;AOS:TO'+
  'S;DoChangeDir:boolean=true);'#010+
  '    function  GetUnitDir(APackage:TPackage):String;'#010+
  '    procedure AddDependencyIncludePaths(L:TStrings;ATarget: TTarget);'#010+
  '    procedure AddDependencyUnitPaths(L:TStrings;AP','ackage: TPackage);'+
  #010+
  '  Public'#010+
  '    Constructor Create(AOwner : TComponent); override;'#010+
  '    destructor Destroy;override;'#010+
  '    // Public Copy/delete/Move/Archive/Mkdir Commands.'#010+
  '    Procedure ExecuteCommand(const Cmd,Args : String; IgnoreError : B',
  'oolean = False); virtual;'#010+
  '    Procedure CmdCopyFiles(List : TStrings; Const DestDir : String);'#010+
  '    Procedure CmdCreateDir(const DestDir : String);'#010+
  '    Procedure CmdMoveFiles(List : TStrings; Const DestDir : String);'#010+
  '    Procedure CmdDeleteF','iles(List : TStrings);'#010+
  '    Procedure CmdArchiveFiles(List : TStrings; Const ArchiveFile : Str'+
  'ing);'#010+
  '    Procedure ExecuteCommands(Commands : TCommands; At : TCommandAt);'#010+
  '    // Dependency commands'#010+
  '    Function  DependencyOK(ADependency : TDe','pendency) : Boolean;'#010+
  '    // Target commands'#010+
  '    Function  GetCompilerCommand(APackage : TPackage; ATarget : TTarge'+
  't) : String;'#010+
  '    Function  TargetOK(ATarget : TTarget) : Boolean;'#010+
  '    Function  NeedsCompile(APackage:TPackage; ATarget : TTar','get) : B'+
  'oolean;'#010+
  '    Procedure Compile(APackage:TPackage; ATarget : TTarget);  virtual;'+
  #010+
  '    Procedure MaybeCompile(APackage:TPackage; ATarget: TTarget);'#010+
  '    Procedure CompileDependencies(APackage:TPackage; ATarget: TTarget)'+
  ';'#010+
  '    // Package c','ommands'#010+
  '{    Function  GetPackageDir(APackage : TPackage; AbsolutePath : Boole'+
  'an = False) : String;'#010+
  '    Function  GetUnitsOutputDir(APackage : TPackage; AbsolutePath : Bo'+
  'olean = False) : String;'#010+
  '    Function  GetBinOutputDir(APackage : TPac','kage; AbsolutePath : Bo'+
  'olean = False) : String;  }'#010+
  '    Function  PackageOK(APackage : TPackage) : Boolean; virtual;'#010+
  '    Procedure DoBeforeCompile(APackage : TPackage);virtual;'#010+
  '    Procedure DoAfterCompile(APackage : TPackage);virtual;'#010+
  '    P','rocedure DoBeforeInstall(APackage : TPackage);virtual;'#010+
  '    Procedure DoAfterInstall(APackage : TPackage);virtual;'#010+
  '    Procedure DoBeforeArchive(APackage : TPackage);virtual;'#010+
  '    Procedure DoAfterArchive(APackage : TPackage);virtual;'#010+
  '    Pro','cedure DoBeforeClean(APackage : TPackage);virtual;'#010+
  '    Procedure DoAfterClean(APackage : TPackage);virtual;'#010+
  '    Function  NeedsCompile(APackage : TPackage) : Boolean; virtual;'#010+
  '    Procedure Compile(APackage : TPackage);'#010+
  '    Procedure MaybeC','ompile(APackage:TPackage);'#010+
  '    Procedure Install(APackage : TPackage);'#010+
  '    Procedure Archive(APackage : TPackage);'#010+
  '    Procedure Manifest(APackage : TPackage);'#010+
  '    Procedure Clean(APackage : TPackage);'#010+
  '    Procedure CompileDependencies(APac','kage : TPackage);'#010+
  '    Function  CheckExternalPackage(Const APackageName : String):TPacka'+
  'ge;'#010+
  '    procedure CreateOutputDir(APackage: TPackage);'#010+
  '    // Packages commands'#010+
  '    Procedure Compile(Packages : TPackages);'#010+
  '    Procedure Install(Packa','ges : TPackages);'#010+
  '    Procedure Archive(Packages : TPackages);'#010+
  '    procedure Manifest(Packages: TPackages);'#010+
  '    Procedure Clean(Packages : TPackages);'#010+
  '    Property ListMode : Boolean Read FListMode Write FListMode;'#010+
  '    Property ForceCompile',' : Boolean Read FForceCompile Write FForceC'+
  'ompile;'#010+
  '    Property ExternalPackages: TPackages Read FExternalPackages;'#010+
  '    // Events'#010+
  '    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FB'+
  'eforeCompile;'#010+
  '    Property AfterCompile ',': TNotifyEvent Read FAfterCompile Write FA'+
  'fterCompile;'#010+
  '    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FB'+
  'eforeInstall;'#010+
  '    Property AfterInstall : TNotifyEvent Read FAfterInstall Write FAft'+
  'erInstall;'#010+
  '    Property BeforeC','lean : TNotifyEvent Read FBeforeClean Write FBef'+
  'oreClean;'#010+
  '    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterCl'+
  'ean;'#010+
  '    Property BeforeArchive : TNotifyEvent Read FBeforeArchive Write FB'+
  'eforeArchive;'#010+
  '    Property AfterArchi','ve : TNotifyEvent Read FAfterArchive Write FA'+
  'fterArchive;'#010+
  '    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write '+
  'FBeforeManifest;'#010+
  '    Property AfterManifest : TNotifyEvent Read FAfterManifest Write FA'+
  'fterManifest;'#010+
  '    Propert','y OnLog : TLogEvent Read FOnLog Write FOnlog;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TCustomInstaller }'#010+
  #010+
  '  TCustomInstaller = Class(TComponent)'#010+
  '  private'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FBuildEngine: TBuildEng','ine;'#010+
  '    FPackages: TPackages;'#010+
  '    FRunMode: TRunMode;'#010+
  '    FListMode : Boolean;'#010+
  '    FLogLevels : TVerboseLevels;'#010+
  '  Protected'#010+
  '    Procedure Log(Level : TVerboseLevel; Const Msg : String);'#010+
  '    Procedure CreatePackages; virtual;'#010+
  '    Procedure ','CheckPackages; virtual;'#010+
  '    Procedure CreateBuildEngine; virtual;'#010+
  '    Procedure Error(const Msg : String);'#010+
  '    Procedure Error(const Fmt : String; Args : Array of const);'#010+
  '    Procedure AnalyzeOptions;'#010+
  '    Procedure Usage(const FMT : String;',' Args : Array of const);'#010+
  '    Procedure Compile(Force : Boolean); virtual;'#010+
  '    Procedure Clean; virtual;'#010+
  '    Procedure Install; virtual;'#010+
  '    Procedure Archive; virtual;'#010+
  '    Procedure Manifest; virtual;'#010+
  '    Property BuildEngine : TBuildEngine',' Read FBuildEngine;'#010+
  '  Public'#010+
  '    Constructor Create(AOwner : TComponent); virtual;'#010+
  '    Destructor destroy; override;'#010+
  '    Function AddPackage(Const AName : String) : TPackage;'#010+
  '    Function Run : Boolean;'#010+
  '    //files in package'#010+
  '    Property P','ackages : TPackages Read FPackages;'#010+
  '    Property RunMode : TRunMode Read FRunMode;'#010+
  '    Property ListMode : Boolean Read FListMode;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FBuildEngine: TBuildEngine;'#010+
  '    FPackages: TPackages;'#010+
  '    FRunMode: TRunMode;'#010+
  '    FListMode : Boolean;'#010+
  '    FL','ogLevels : TVerboseLevels;'#010+
  '  Protected'#010+
  '    Procedure Log(Level : TVerboseLevel; Const Msg : String);'#010+
  '    Procedure CreatePackages; virtual;'#010+
  '    Procedure CheckPackages; virtual;'#010+
  '    Procedure CreateBuildEngine; virtual;'#010+
  '    Procedure Error(','const Msg : String);'#010+
  '    Procedure Error(const Fmt : String; Args : Array of const);'#010+
  '    Procedure AnalyzeOptions;'#010+
  '    Procedure Usage(const FMT : String; Args : Array of const);'#010+
  '    Procedure Compile(Force : Boolean); virtual;'#010+
  '    Procedur','e Clean; virtual;'#010+
  '    Procedure Install; virtual;'#010+
  '    Procedure Archive; virtual;'#010+
  '    Procedure Manifest; virtual;'#010+
  '    Property BuildEngine : TBuildEngine Read FBuildEngine;'#010+
  '  Public'#010+
  '    Constructor Create(AOwner : TComponent); virtual;'#010+
  '   ',' Destructor destroy; override;'#010+
  '    Function AddPackage(Const AName : String) : TPackage;'#010+
  '    Function Run : Boolean;'#010+
  '    //files in package'#010+
  '    Property Packages : TPackages Read FPackages;'#010+
  '    Property RunMode : TRunMode Read FRunMode;'#010+
  '   ',' Property ListMode : Boolean Read FListMode;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TFPCInstaller }'#010+
  '  TFPCInstaller = class(TCustomInstaller)'#010+
  '  public'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Constructor Create(AOwner ',': TComponent); override;'#010+
  '  end;'#010+
  #010+
  '  { TBasicInstaller }'#010+
  '  TBasicInstaller = class(TCustomInstaller)'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Constructor Create(AOwner : TComponent); override;'#010+
  '  end;'#010+
  #010+
  '  { TBasicInstaller }'#010+
  '  TBasicInstaller = class(TCust','omInstaller)'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Constructor Create(AOwner : TComponent); override;'#010+
  '  end;'#010+
  #010+
  '  TReplaceFunction = Function (Const AName,Args : String) : String of '+
<<<<<<< HEAD
<<<<<<< HEAD
  'Object;'#010,
=======
  'Object;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Object;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '  { TValueItem }'#010+
  #010+
  '  TValueItem = Class(TObject)'#010+
  '    FValue : String;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Constructor Create(AValue : String);'#010+
=======
  '    Constructor ','Create(AValue : String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Constructor ','Create(AValue : String);'#010+
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TFunctionItem }'#010+
  #010+
  '  TFunctionItem = Class(TObject)'#010+
  '    FFunc : TReplaceFunction;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Constructor Create(AFunc : TRepla','ceFunction);'#010+
=======
  '    Constructor Create(AFunc : TReplaceFunction);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Constructor Create(AFunc : TReplaceFunction);'#010+
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  { TDictionary }'#010+
  #010+
  '  TDictionary = Class(TComponent)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FList : TStringList;'#010+
  '  Public'#010+
  '    Constructor Create(AOwner : TComponent); override;'#010+
  '    Destructor Destroy;override;'#010+
  '    Procedure AddVariable(Const AName,Value ',': String);'#010+
  '    Procedure AddFunction(Const AName : String; FReplacement : TReplac'+
  'eFunction);'#010+
  '    Procedure RemoveItem(Const AName : String);'#010+
  '    Function GetValue(Const AName : String) : String;'#010+
  '    Function GetValue(Const AName,Args : Stri','ng) : String; virtual;'#010+
  '    Function ReplaceStrings(Const ASource : String) : String; virtual;'+
  #010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FList : ','TStringList;'#010+
  '  Public'#010+
  '    Constructor Create(AOwner : TComponent); override;'#010+
  '    Destructor Destroy;override;'#010+
  '    Procedure AddVariable(Const AName,Value : String);'#010+
  '    Procedure AddFunction(Const AName : String; FReplacement : TReplac'+
  'eFunc','tion);'#010+
  '    Procedure RemoveItem(Const AName : String);'#010+
  '    Function GetValue(Const AName : String) : String;'#010+
  '    Function GetValue(Const AName,Args : String) : String; virtual;'#010+
  '    Function ReplaceStrings(Const ASource : String) : String; v','irtua'+
  'l;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  ECollectionError = Class(Exception);'#010+
  '  EDictionaryError = Class(Exception);'#010+
  '  EInstallerError = Class(Exception);'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  TInstallerClass = C','lass of TCustomInstaller;'#010+
  '  TDictionaryClass = Class of TDictionary;'#010+
  #010+
  'Type'#010+
  '  TArchiveEvent = Procedure (Const AFileName : String; List : TStrings'+
  ') of Object;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  TInstallerClass = Class of TCustomInstaller;'#010+
  '  TDictionaryClass = Class of TDictionary;'#010+
  #010+
  'Type'#010+
  '  TArchiveE','vent = Procedure (Const AFileName : String; List : TStrin'+
  'gs) of Object;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  TArchiveProc = Procedure (Const AFileName : String; List : TStrings)'+
  ';'#010+
  #010+
  'Var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Dic','tionaryClass : TDictionaryClass = TDictionary;'#010+
  '  OnArchiveFiles : TArchiveEvent = Nil;'#010+
=======
  '  DictionaryClass : TDictionaryClass = TDictionary;'#010+
  '  OnArchiveFiles : TArchiveEvent = Nil;',#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  DictionaryClass : TDictionaryClass = TDictionary;'#010+
  '  OnArchiveFiles : TArchiveEvent = Nil;',#010+
>>>>>>> origin/fixes_2_2
  '  ArchiveFilesProc : TArchiveProc = Nil;'#010+
  #010+
  '  Defaults : TCustomDefaults; // Set by installer.'#010+
  '  Dictionary : TDictionary;'#010+
  #010+
  #010+
  'Function CurrentOS : String;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'F','unction CurrentCPU : String;'#010+
  #010+
  'Function Installer(InstallerClass: TInstallerClass) : TCustomInstaller'+
  '; overload;'#010+
  'Function Installer : TCustomInstaller; overload;'#010+
  #010+
  'Function OSToString(OS: TOS) : String;'#010+
  'Function OSesToString(OSes: TOSes) : St','ring;'#010+
  'Function CPUToString(CPU: TCPU) : String;'#010+
  'Function CPUSToString(CPUS: TCPUS) : String;'#010+
  'Function StringToOS(const S : String) : TOS;'#010+
  'Function OSesToString(const S : String) : TOSes;'#010+
  'Function StringToCPU(const S : String) : TCPU;'#010+
  'Functi','on StringToCPUS(const S : String) : TCPUS;'#010+
  'Function ModeToString(Mode: TCompilerMode) : String;'#010+
  'Function StringToMode(const S : String) : TCompilerMode;'#010+
  'Function MakeTargetString(CPU : TCPU;OS: TOS) : String;'#010+
  'Procedure StringToCPUOS(const S',' : String; Var CPU : TCPU; Var OS: TO'+
  'S);'#010+
  'Function FixPath (const APath : String) : String;'#010+
  'Procedure ChangeDir(const APath : String);'#010+
  'Function Substitute(Const Source : String; Macros : Array of string) :'+
  ' String;'#010+
  'Procedure SplitCommand(Cons','t Cmd : String; Var Exe,Options : String)'+
  ';'#010+
  #010+
  'Implementation'#010+
  #010+
  'uses typinfo, rtlconsts;'#010+
  #010+
  'type'#010+
  '  TUnsortedDuplicatesStringList = class(TStringList)'#010+
  '  public'#010+
  '    function Add(const S: string): Integer; override;'#010+
  '  end;'#010+
  #010+
  'ResourceString'#010+
  '  SErrInva','lidCPU        = '#039'Invalid CPU name "%s"'#039';'#010+
  '  SErrInvalidOS         = '#039'Invalid OS name "%s"'#039';'#010+
  '  SErrInvalidMode       = '#039'Invalid compiler mode "%s"'#039';'#010+
  '  SErrInvalidTarget     = '#039'Invalid compiler target "%s"'#039';'#010+
  '  SErrNameExists        = '#039'Name "%s','" already exists in the colle'+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function CurrentCPU : String;'#010+
  #010+
  'Function Installer(InstallerClass: TInstallerClass) : TC','ustomInstall'+
  'er; overload;'#010+
  'Function Installer : TCustomInstaller; overload;'#010+
  #010+
  'Function OSToString(OS: TOS) : String;'#010+
  'Function OSesToString(OSes: TOSes) : String;'#010+
  'Function CPUToString(CPU: TCPU) : String;'#010+
  'Function CPUSToString(CPUS: TCPUS) : S','tring;'#010+
  'Function StringToOS(const S : String) : TOS;'#010+
  'Function OSesToString(const S : String) : TOSes;'#010+
  'Function StringToCPU(const S : String) : TCPU;'#010+
  'Function StringToCPUS(const S : String) : TCPUS;'#010+
  'Function ModeToString(Mode: TCompilerMode) ',': String;'#010+
  'Function StringToMode(const S : String) : TCompilerMode;'#010+
  'Function MakeTargetString(CPU : TCPU;OS: TOS) : String;'#010+
  'Procedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TOS)'+
  ';'#010+
  'Function FixPath (const APath : String) : Str','ing;'#010+
  'Procedure ChangeDir(const APath : String);'#010+
  'Function Substitute(Const Source : String; Macros : Array of string) :'+
  ' String;'#010+
  'Procedure SplitCommand(Const Cmd : String; Var Exe,Options : String);'#010+
  #010+
  'Implementation'#010+
  #010+
  'uses typinfo;'#010+
  #010+
  'ResourceStr','ing'#010+
  '  SErrInvalidCPU        = '#039'Invalid CPU name "%s"'#039';'#010+
  '  SErrInvalidOS         = '#039'Invalid OS name "%s"'#039';'#010+
  '  SErrInvalidMode       = '#039'Invalid compiler mode "%s"'#039';'#010+
  '  SErrInvalidTarget     = '#039'Invalid compiler target "%s"'#039';'#010+
  '  SErrNameExists     ','   = '#039'Name "%s" already exists in the colle'+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'ction.'#039';'#010+
  '  SErrNoSuchName        = '#039'Could not find item with name "%s" in t'+
  'he collection.'#039';'#010+
  '  SErrInValidArgument   = '#039'Invalid command-line argument at positi'+
  'on %d: %s'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SErrNeedArgument      = '#039'Option at posi','tion %d (%s) needs an a'+
  'rgument'#039';'#010+
=======
  '  SErrNeedArgument      = '#039,'Option at position %d (%s) needs an ar'+
  'gument'#039';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  SErrNeedArgument      = '#039,'Option at position %d (%s) needs an ar'+
  'gument'#039';'#010+
>>>>>>> origin/fixes_2_2
  '  SErrNoPackagesDefined = '#039'No action possible: No packages were de'+
  'fined.'#039';'#010+
  '  SErrInstaller         = '#039'The installer encountered the following'+
  ' error:'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SErrDepUnknownTarget  = '#039'Unknown target for unit "%s" ','in depen'+
  'dencies for %s in package %s'#039';'#010+
=======
  '  SErrDepUnknownTarget  = '#039'Unknown target ','in dependencies for %s'+
  ': %s'#039';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  SErrDepUnknownTarget  = '#039'Unknown target ','in dependencies for %s'+
  ': %s'#039';'#010+
>>>>>>> origin/fixes_2_2
  '  SErrExternalCommandFailed = '#039'External command "%s" failed with e'+
  'xit code %d'#039';'#010+
  '  SErrCreatingDirectory = '#039'Failed to create directory "%s"'#039';'#010+
  '  SErrDeletingFile      = '#039'Failed to delete file "%s"'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SE','rrMovingFile        = '#039'Failed to move file "%s" to "%s"'#039';'+
  #010+
  '  SErrCopyingFile       = '#039'Failed to copy file "%s" to "%s"'#039';'#010+
  '  SErrChangeDirFailed   = '#039'Failed to enter directory "%s"'#039';'#010+
  '  SErrInvalidArgumentToSubstitute = '#039'Invalid number of argum','ents'+
  ' to Substitute'#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  SErrMovingFi','le        = '#039'Failed to move file "%s" to "%s"'#039';'+
  #010+
  '  SErrCopyingFile       = '#039'Failed to copy file "%s" to "%s"'#039';'#010+
  '  SErrChangeDirFailed   = '#039'Failed to enter directory "%s"'#039';'#010+
  '  SErrInvalidArgumentToSubstitute = '#039'Invalid number of arguments t'+
  'o Su','bstitute'#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  SErrNoArchiveSupport  = '#039'This binary contains no archive support'+
  '. Please recompile with archive support'#039';'#010+
  '  SErrNoDictionaryItem  = '#039'No item called "%s" in the dictionary'#039+
  ';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SErrNoDictionaryValue = '#039'The item "%s" in ','the dictionary is no'+
=======
  '  SErrNoDictionaryValue = '#039'The item "%s" in the dictio','nary is no'+
>>>>>>> graemeg/fixes_2_2
=======
  '  SErrNoDictionaryValue = '#039'The item "%s" in the dictio','nary is no'+
>>>>>>> origin/fixes_2_2
  't a value'#039';'#010+
  '  SErrNoDictionaryFunc  = '#039'The item "%s" in the dictionary is not '+
  'a function'#039';'#010+
  '  SErrInvalidFPCInfo    = '#039'Compiler returns invalid information, c'+
  'heck if fpc -iV works'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SErrDependencyNotFound = '#039'Could not f','ind unit directory for de'+
=======
  '  SErrDependencyNotFound = '#039'Could not find unit d','irectory for de'+
>>>>>>> graemeg/fixes_2_2
=======
  '  SErrDependencyNotFound = '#039'Could not find unit d','irectory for de'+
>>>>>>> origin/fixes_2_2
  'pendency package "%s"'#039';'#010+
  '  SErrAlreadyInitialized = '#039'Installer can only be initialized once'+
  #039';'#010+
  '  SErrInvalidState      = '#039'Invalid state for target %s'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SErrCouldNotCompile   = '#039'Could not compile target %s from packa',
  'ge %s'#039';'#010+
  #010+
  '  SWarnCircularTargetDependency = '#039'Warning: Circular dependency de'+
  'tected when compiling target %s with target %s'#039';'#010+
  '  SWarnCircularPackageDependency = '#039'Warning: Circular dependency d'+
  'etected when compiling package %s with package %s'#039,';'#010+
  '  SWarnFailedToSetTime    = '#039'Warning: Failed to set timestamp on f'+
  'ile "%s"'#039';'#010+
  '  SWarnFailedToGetTime    = '#039'Warning: Failed to get timestamp from'+
  ' file "%s"'#039';'#010+
  '  SWarnAttemptingToCompileNonNeutralTarget = '#039'Warning: Attempting '+
  'to compile non-n','eutral target %s'#039';'#010+
  '  SWarnSourceFileNotFound  = '#039'Warning: Source file "%s" from packa'+
  'ge %s not found for %s'#039';'#010+
  '  SWarnIncludeFileNotFound = '#039'Warning: Include file "%s" from pack'+
  'age %s not found for %s'#039';'#010+
  '  SWarnDepUnitNotFound     = '#039'Warning:',' Dependency on unit %s is '+
  'not supported for %s'#039';'#010+
  '  SWarnTargetDependsOnPackage = '#039'Warning: Target %s of package %s '+
  'depends on another package (%s). These kind of dependencies are not pr'+
  'ocessed'#039';'#010+
  '  SWarnDependOnOtherPlatformPackage = '#039'Warnin','g: Package %s depen'+
  'ds on package %s which is not available for the %s platform'#039';'#010+
  '  SWarnDone               = '#039'Done'#039';'#010+
  #010+
  '  SInfoCompilingPackage   = '#039'Compiling package %s'#039';'#010+
  '  SInfoPackageAlreadyProcessed = '#039'Package %s is already processed'#039+
  ';'#010+
  '  ','SInfoCompilingTarget    = '#039'Compiling target %s'#039';'#010+
  '  SInfoExecutingCommand   = '#039'Executing command "%s %s"'#039';'#010+
  '  SInfoCreatingOutputDir  = '#039'Creating output dir "%s"'#039';'#010+
  '  SInfoInstallingPackage  = '#039'Installing package %s'#039';'#010+
  '  SInfoArchivingPackage  ',' = '#039'Archiving package %s in "%s"'#039';'#010+
  '  SInfoCleaningPackage    = '#039'Cleaning package %s'#039';'#010+
  '  SInfoManifestPackage    = '#039'Creating manifest for package %s'#039';'+
  #010+
  '  SInfoCopyingFile        = '#039'Copying file "%s" to "%s"'#039';'#010+
  '  SInfoSourceNewerDest    = '#039'Sour','ce file "%s" (%s) is newer than'+
  ' destination "%s" (%s).'#039';'#010+
  #010+
  '  SDbgComparingFileTimes    = '#039'Comparing file "%s" time "%s" to "%'+
  's" time "%s".'#039';'#010+
  '  SDbgCompilingDependenciesOfTarget = '#039'Compiling dependencies of t'+
  'arget %s'#039';'#010+
  '  SDbgResolvingSourcesO','fTarget = '#039'Resolving filenames of target '+
  '%s for %s'#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  #010+
  '  SWarnCircularTargetDependency = '#039'Warning: Circular dependency de'+
  'tected w','hen compiling target %s with target %s'#039';'#010+
  '  SWarnCircularPackageDependency = '#039'Warning: Circular dependency d'+
  'etected when compiling package %s with package %s'#039';'#010+
  '  SWarnFailedToSetTime    = '#039'Warning: Failed to set timestamp on f'+
  'ile "%s"'#039';'#010+
  '  SW','arnFailedToGetTime    = '#039'Warning: Failed to get timestamp fr'+
  'om file "%s"'#039';'#010+
  '  SWarnFileDoesNotExist   = '#039'Warning: File "%s" does not exist'#039+
  ';'#010+
  '  SWarnAttemptingToCompileNonNeutralTarget = '#039'Warning: Attempting '+
  'to compile non-neutral target %s'#039';',#010+
  '  SWarnSourceFileNotFound  = '#039'Warning: Source file "%s" not found '+
  'for %s'#039';'#010+
  '  SWarnIncludeFileNotFound = '#039'Warning: Include file "%s" not found'+
  ' for %s'#039';'#010+
  '  SWarnDepUnitNotFound     = '#039'Warning: Dependency on unit %s is no'+
  't supported for %s'#039';'#010+
  #010,
  '  SInfoCompilingPackage   = '#039'Compiling package %s'#039';'#010+
  '  SInfoCompilingTarget    = '#039'Compiling target %s'#039';'#010+
  '  SInfoExecutingCommand   = '#039'Executing command "%s %s"'#039';'#010+
  '  SInfoCreatingOutputDir  = '#039'Creating output dir "%s"'#039';'#010+
  '  SInfoInstallingPackage','  = '#039'Installing package %s'#039';'#010+
  '  SInfoArchivingPackage   = '#039'Archiving package %s in "%s"'#039';'#010+
  '  SInfoCleaningPackage    = '#039'Cleaning package %s'#039';'#010+
  '  SInfoManifestPackage    = '#039'Creating manifest for package %s'#039';'+
  #010+
  '  SInfoCopyingFile        = '#039'Copying',' file "%s" to "%s"'#039';'#010+
  '  SInfoSourceNewerDest    = '#039'Source file "%s" (%s) is newer than d'+
  'estination "%s" (%s).'#039';'#010+
  #010+
  '  SDbgComparingFileTimes    = '#039'Comparing file "%s" time "%s" to "%'+
  's" time "%s".'#039';'#010+
  '  SDbgCompilingDependenciesOfTarget = '#039'Compili','ng dependencies of'+
  ' target %s'#039';'#010+
  '  SDbgResolvingSourcesOfTarget = '#039'Resolving filenames of target %s'+
  ' for %s'#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  SDbgResolvedSourceFile    = '#039'Resolved source file %s to "%s"'#039+
  ';'#010+
  '  SDbgResolvedIncludeFile   = '#039'Resolved include file %s to "%s"'#039+
  ';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SDbgOutputNotYetAvailable = '#039'Output file %s not availa','ble'#039';'+
=======
  '  ','SDbgOutputNotYetAvailable = '#039'Output file %s not available'#039';'+
>>>>>>> graemeg/fixes_2_2
=======
  '  ','SDbgOutputNotYetAvailable = '#039'Output file %s not available'#039';'+
>>>>>>> origin/fixes_2_2
  #010+
  '  SDbgDependencyOnUnit      = '#039'Dependency of %s on unit %s'#039';'#010+
  '  SDbgDependencyUnitRecompiled = '#039'Dependent unit %s is being recom'+
  'piled'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SDbgMustCompile           = '#039'Must compile %s'#039';'#010+
  '  SDbgSkippingTargetWrongCPU = '#039'Skipping target %s',', different CP'+
  'U (%s)'#039';'#010+
=======
  '  SDbgMustCompile           = '#039'Must compile %','s'#039';'#010+
  '  SDbgSkippingTargetWrongCPU = '#039'Skipping target %s, different CPU '+
  '(%s)'#039';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  SDbgMustCompile           = '#039'Must compile %','s'#039';'#010+
  '  SDbgSkippingTargetWrongCPU = '#039'Skipping target %s, different CPU '+
  '(%s)'#039';'#010+
>>>>>>> origin/fixes_2_2
  '  SDbgSkippingTargetWrongOS  = '#039'Skipping target %s, different OS ('+
  '%s)'#039';'#010+
  '  SDbgTargetIsNotAUnitOrProgram = '#039'Skipping Target %s, not an unit'+
  ' or program'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SDbgConsideringTarget     = '#039'Considering target %s'#039';'#010+
  '  SDbgCons','ideringPackage    = '#039'Considering package %s'#039';'#010+
  '  SDbgExternalDependency    = '#039'External dependency %s found in "%s'+
  '"'#039';'#010+
  '  SDbgBuildEngineArchiving  = '#039'Build engine archiving'#039';'#010+
  '  SDbgBuildEngineGenerateManifests = '#039'Build engine generating mani'+
  'fe','sts'#039';'#010+
  '  SDbgBuildEngineCleaning   = '#039'Build engine cleaning'#039';'#010+
  '  SDbgGenerating            = '#039'Generating "%s"'#039';'#010+
  '  SDbgLoading               = '#039'Loading "%s"'#039';'#010+
  '  SDbgFound                 = '#039'Found'#039';'#010+
  '  SDbgNotFound              = '#039'Not Found'#039';'#010+
  '  ','SDbgDirectoryExists       = '#039'Directory "%s" %s'#039';'#010+
  '  SDbgFileExists            = '#039'File "%s" %s'#039';'#010+
  '  SDbgArchivingFile         = '#039'Archiving "%s"'#039';'#010+
  '  SDbgSearchPath            = '#039'Using %s path "%s"'#039';'#010+
  '  SDbgEnterDir              = '#039'Entering direc','tory "%s"'#039';'#010+
  '  SDbgPackageChecksumChanged = '#039'Dependent package %s is modified'#039+
  ';'#010+
  '  SDbgFileDoesNotExist      = '#039'File "%s" does not exist'#039';'#010+
  #010+
  '  // Help messages for usage'#010+
  '  SValue              = '#039'Value'#039';'#010+
  '  SHelpUsage          = '#039'Usage: %s comma','nd [options]'#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  SDbgConsi','deringTarget     = '#039'Considering target %s'#039';'#010+
  '  SDbgConsideringPackage    = '#039'Considering package %s'#039';'#010+
  '  SDbgExternalDependency    = '#039'External dependency %s found in "%s'+
  '"'#039';'#010+
  '  SDbgBuildEngineArchiving  = '#039'Build engine archiving'#039';'#010+
  '  SDbgBuildEng','ineGenerateManifests = '#039'Build engine generating ma'+
  'nifests'#039';'#010+
  '  SDbgBuildEngineCleaning   = '#039'Build engine cleaning'#039';'#010+
  '  SDbgGenerating            = '#039'Generating "%s"'#039';'#010+
  '  SDbgLoading               = '#039'Loading "%s"'#039';'#010+
  '  SDbgFound                 = ',#039'Found'#039';'#010+
  '  SDbgNotFound              = '#039'Not Found'#039';'#010+
  '  SDbgDirectoryExists       = '#039'Directory "%s" %s'#039';'#010+
  '  SDbgFileExists            = '#039'File "%s" %s'#039';'#010+
  '  SDbgArchivingFile         = '#039'Archiving "%s"'#039';'#010+
  '  SDbgSearchPath            = '#039'Using %s pat','h "%s"'#039';'#010+
  '  SDbgEnterDir              = '#039'Entering directory "%s"'#039';'#010+
  '  SDbgPackageChecksumChanged = '#039'Dependent package %s is modified'#039+
  ';'#010+
  #010+
  '  // Help messages for usage'#010+
  '  SValue              = '#039'Value'#039';'#010+
  '  SHelpUsage          = '#039'Usage: %s command [','options]'#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  SHelpCommand        = '#039'Where command is one of the following:'#039+
  ';'#010+
  '  SHelpCompile        = '#039'Compile all units in the package(s).'#039';'+
  #010+
  '  SHelpBuild          = '#039'Build all units in the package(s).'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SHelpInstall        = '#039'Install ','all units in the package(s).'#039+
=======
  '  SHelpInstall        = '#039'Install all ','units in the package(s).'#039+
>>>>>>> graemeg/fixes_2_2
=======
  '  SHelpInstall        = '#039'Install all ','units in the package(s).'#039+
>>>>>>> origin/fixes_2_2
  ';'#010+
  '  SHelpClean          = '#039'Clean (remove) all units in the package(s'+
  ').'#039';'#010+
  '  SHelpArchive        = '#039'Create archive (zip) with all units in th'+
  'e package(s).'#039';'#010+
  '  SHelpHelp           = '#039'This message.'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SHelpManifest','       = '#039'Create a manifest suitable for import i'+
=======
  '  SHelpManifest    ','   = '#039'Create a manifest suitable for import i'+
>>>>>>> graemeg/fixes_2_2
=======
  '  SHelpManifest    ','   = '#039'Create a manifest suitable for import i'+
>>>>>>> origin/fixes_2_2
  'n repository.'#039';'#010+
  '  SHelpCmdOptions     = '#039'Where options is one or more of the follo'+
  'wing:'#039';'#010+
  '  SHelpCPU            = '#039'Compile for indicated CPU.'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SHelpOS             = '#039'Compile for indicated ','OS'#039';'#010+
=======
  '  SHelpOS             = '#039'Compile for indicated OS'#039';',#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  SHelpOS             = '#039'Compile for indicated OS'#039';',#010+
>>>>>>> origin/fixes_2_2
  '  SHelpTarget         = '#039'Compile for indicated target'#039';'#010+
  '  SHelpList           = '#039'list commands instead of actually executi'+
  'ng them.'#039';'#010+
  '  SHelpPrefix         = '#039'Use indicated prefix directory for all co'+
  'mmands.'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SHelpNoFPCCfg       = '#039'C','ompiler will not use fpc.cfg'#039';'#010+
=======
  '  SHelpNoFPCCfg       = '#039'Compi','ler will not use fpc.cfg'#039';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  SHelpNoFPCCfg       = '#039'Compi','ler will not use fpc.cfg'#039';'#010+
>>>>>>> origin/fixes_2_2
  '  SHelpBaseInstallDir = '#039'Use indicated directory as base install d'+
  'ir.'#039';'#010+
  '  SHelpLocalUnitDir   = '#039'Use indicated directory as local (user) u'+
  'nit dir.'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SHelpGlobalUnitDir  = '#039'Use indicated directory as global u','nit '+
  'dir.'#039';'#010+
  '  SHelpCompiler       = '#039'Use indicated binary as compiler'#039';'#010+
  '  SHelpConfig         = '#039'Use indicated config file when compiling.'+
  #039';'#010+
  '  SHelpOptions        = '#039'Pass extra options to the compiler.'#039';'+
  #010+
  '  SHelpVerbose        = '#039'Be verbose w','hen working.'#039';'#010+
  #010+
  #010+
  'Const'#010+
  '  // Keys for Defaults file. Do not localize.'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  SHelpGlobalUnitDir  = '#039'Use indicated directory as global unit ','d'+
  'ir.'#039';'#010+
  '  SHelpCompiler       = '#039'Use indicated binary as compiler'#039';'#010+
  '  SHelpConfig         = '#039'Use indicated config file when compiling.'+
  #039';'#010+
  '  SHelpVerbose        = '#039'Be verbose when working.'#039';'#010+
  #010+
  #010+
  'Const'#010+
  '  // Keys for Defaults file. Do not localize','.'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  KeyCompiler = '#039'Compiler'#039';'#010+
  '  KeyArchive  = '#039'Archive'#039';'#010+
  '  KeyCopy     = '#039'Copy'#039';'#010+
  '  KeyMkDir    = '#039'MkDir'#039';'#010+
  '  KeyMove     = '#039'Move'#039';'#010+
  '  KeyRemove   = '#039'Remove'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  KeyOptions  = '#039,'Options'#039';'#010+
  '  KeyCPU      = '#039'CPU'#039';'#010+
  '  KeyOS       = '#039'OS'#039';'#010+
  '  KeyMode     = '#039'Mode'#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  KeyOptions  = '#039'Options'#039';'#010+
  '  KeyCPU      = '#039'CPU'#039';'#010+
  '  KeyOS       = '#039'OS'#039';'#010+
  '  KeyMode   ','  = '#039'Mode'#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  KeyPrefix   = '#039'Prefix'#039';'#010+
  '  KeyTarget   = '#039'Target'#039';'#010+
  '  KeyNoFPCCfg = '#039'NoFPCCfg'#039';'#010+
  '  KeyLocalUnitDir       = '#039'LocalUnitDir'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  KeyGlobalUnitDir      = '#039'GlobalUnitDi','r'#039';'#010+
  '  KeyBaseInstallDir     = '#039'BaseInstallDir'#039';'#010+
  '  KeyUnitInstallDir     = '#039'UnitInstallDir'#039';'#010+
  '  KeyBinInstallDir      = '#039'BinInstallDir'#039';'#010+
  '  KeyDocInstallDir      = '#039'DocInstallDir'#039';'#010+
  '  KeyExamplesInstallDir = '#039'ExamplesInstallDir'#039';'#010+
  '  // Keys for ','unit config'#010+
  '  KeyName     = '#039'Name'#039';'#010+
  '  KeyVersion  = '#039'Version'#039';'#010+
  '  KeyChecksum = '#039'Checksum'#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  KeyGlobalUnitDir      = '#039'GlobalUnitDir'#039';'#010+
  '  KeyBaseInstallDir     = '#039'BaseInstallDir'#039';'#010+
  '  KeyUnitInstallDir','     = '#039'UnitInstallDir'#039';'#010+
  '  KeyBinInstallDir      = '#039'BinInstallDir'#039';'#010+
  '  KeyDocInstallDir      = '#039'DocInstallDir'#039';'#010+
  '  KeyExamplesInstallDir = '#039'ExamplesInstallDir'#039';'#010+
  '  // Keys for unit config'#010+
  '  KeyName     = '#039'Name'#039';'#010+
  '  KeyVersion  = '#039'Version'#039';'#010+
  '  Ke','yChecksum = '#039'Checksum'#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  KeyNeedLibC = '#039'NeedLibC'#039';'#010+
  '  KeyDepends  = '#039'Depends'#039';'#010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '               ','                 Helpers'#010+
  '**********************************************************************'+
  '******}'#010+
=======
  '                                Helpers'#010+
  '******************************************','**************************'+
  '********}'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '                                Helpers'#010+
  '******************************************','**************************'+
  '********}'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'Function QuoteXML(S : String) : string;'#010+
  #010+
  '  Procedure W(Var J : Integer; Var R : String; T : String);'#010+
  '  Var'#010+
  '    I: integer;'#010+
  '  begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    ','If J+Length(T)>Length(R) then'#010+
  '      SetLength(R,J+Length(T));'#010+
  '    For I:=1 to Length(t) do'#010+
=======
  '    If J+Length(T)>Length(R) then'#010+
  '      SetLength(R,J+Length(T));'#010+
  '    F','or I:=1 to Length(t) do'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    If J+Length(T)>Length(R) then'#010+
  '      SetLength(R,J+Length(T));'#010+
  '    F','or I:=1 to Length(t) do'#010+
>>>>>>> origin/fixes_2_2
  '      begin'#010+
  '      R[J]:=T[i];'#010+
  '      If I<Length(T) then'#010+
  '        Inc(J);'#010+
  '      end;'#010+
  '  end;'#010+
  #010+
  'const'#010+
  '  QuotStr = '#039'&quot;'#039';'#010+
  '  AmpStr = '#039'&amp;'#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ltStr = ',#039'&lt;'#039';'#010+
=======
  '  ltStr = '#039'&lt;'#039';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  ltStr = '#039'&lt;'#039';'#010+
>>>>>>> origin/fixes_2_2
  '  gtStr = '#039'&gt;'#039';'#010+
  'Var'#010+
  '  I,J : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SetLength(Result,Length(S));'#010+
=======
  '  SetLength(Re','sult,Length(S));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  SetLength(Re','sult,Length(S));'#010+
>>>>>>> origin/fixes_2_2
  '  J:=0;'#010+
  '  For I:=1 to Length(S) do'#010+
  '    begin'#010+
  '    Inc(J);'#010+
  '    case S[i] of'#010+
  '      '#039'"'#039': W(j,Result,QuotStr);'#010+
  '      '#039'&'#039': W(J,Result,AmpStr);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      '#039'<'#039': W(J,Resu','lt,ltStr);'#010+
  '      '#039'>'#039': W(J,Result,gtStr);'#010+
  '      // Escape whitespace using CharRefs to be consistent with W3 spe'+
  'c X 3.3.3'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      '#039'<'#039': W(J,Result,ltStr);'#010+
  '      '#039'>'#039': W(J,Result,gtStr);'#010+
  '      // Escape whitespace',' using CharRefs to be consistent with W3 s'+
  'pec X 3.3.3'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '       #9: w(J,Result,'#039'&#x9;'#039');'#010+
  '{      #10: wrtStr('#039'&#xA;'#039');'#010+
  '       #13: wrtStr('#039'&#xD;'#039');}'#010+
  '    else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Result[J]:=S[','i];'#010+
  '    end;'#010+
  '    If (J=Length(Result)) and (I<Length(S)) then'#010+
  '      SetLength(Result,J+Length(S)-I);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      Result[J]:=S[i];'#010+
  '    end;'#010+
  '    If (J=Length(Result)) and (I<Length(S)) then'#010+
  '     ',' SetLength(Result,J+Length(S)-I);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  '  If J<>Length(Result) then'#010+
  '    SetLength(Result,J);'#010+
  'end;'#010+
  #010+
  #010+
  'function maybequoted(const s:string):string;'#010+
  'const'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  {$IFDEF MSWINDOWS','}'#010+
  '    FORBIDDEN_CHARS = ['#039'!'#039', '#039'@'#039', '#039'#'#039', '#039'$'#039+
  ', '#039'%'#039', '#039'^'#039', '#039'&'#039', '#039'*'#039', '#039'('#039', '#039+
=======
  '  {$IFDEF MSWINDOWS}'#010+
  '    FORBIDDEN_CHARS = ['#039'!'#039', '#039'@'#039', '#039'#'#039', '#039'$'#039+
  ', '#039'%'#039', '#039'^'#039', '#039'&'#039', '#039'*'#039', '#039'(',#039', '#039+
>>>>>>> graemeg/fixes_2_2
=======
  '  {$IFDEF MSWINDOWS}'#010+
  '    FORBIDDEN_CHARS = ['#039'!'#039', '#039'@'#039', '#039'#'#039', '#039'$'#039+
  ', '#039'%'#039', '#039'^'#039', '#039'&'#039', '#039'*'#039', '#039'(',#039', '#039+
>>>>>>> origin/fixes_2_2
  ')'#039','#010+
  '                       '#039'{'#039', '#039'}'#039', '#039#039#039#039', '#039'`'+
  #039', '#039'~'#039'];'#010+
  '  {$ELSE}'#010+
  '    FORBIDDEN_CHARS = ['#039'!'#039', '#039'@'#039', '#039'#'#039', '#039'$'#039+
  ', '#039'%'#039', '#039'^'#039', '#039'&'#039', '#039'*'#039', '#039'('#039', '#039+
  ')'#039','#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                       '#039'{'#039', '#039'}'#039',',' '#039#039#039#039', '#039+
  ':'#039', '#039'\'#039', '#039'`'#039', '#039'~'#039'];'#010+
  '  {$ENDIF}'#010+
  'var'#010+
  '  s1 : string;'#010+
  '  i  : integer;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '                       '#039'{'#039', '#039'}'#039', '#039#039#039#039', '#039':'+
  #039', '#039'\'#039', '#039'`'#039', '#039'~'#039'];'#010+
  '  {$ENDIF}'#010+
  'var'#010+
  '  s1 : string;'#010+
  '  i  : in','teger;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  quoted : boolean;'#010+
  'begin'#010+
  '  quoted:=false;'#010+
  '  s1:='#039'"'#039';'#010+
  '  for i:=1 to length(s) do'#010+
  '   begin'#010+
  '     case s[i] of'#010+
  '       '#039'"'#039' :'#010+
  '         begin'#010+
  '           quoted:=true;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '     ','      s1:=s1+'#039'\"'#039';'#010+
  '         end;'#010+
  '       '#039' '#039','#010+
  '       #128..#255 :'#010+
  '         begin'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '           s1:=s1+'#039'\"'#039';'#010+
  '         end;'#010+
  '       '#039' '#039','#010+
  '       #128..#255 :'#010+
  '  ','       begin'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '           quoted:=true;'#010+
  '           s1:=s1+s[i];'#010+
  '         end;'#010+
  '       else begin'#010+
  '         if s[i] in FORBIDDEN_CHARS then'#010+
  '           quoted:=True;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '         s1:=','s1+s[i];'#010+
=======
  '         s1:=s1+s[i];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '         s1:=s1+s[i];'#010+
>>>>>>> origin/fixes_2_2
  '       end;'#010+
  '     end;'#010+
  '   end;'#010+
  '  if quoted then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    maybequoted:=s1+'#039'"'#039#010+
=======
  '    maybequ','oted:=s1+'#039'"'#039#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    maybequ','oted:=s1+'#039'"'#039#010+
>>>>>>> origin/fixes_2_2
  '  else'#010+
  '    maybequoted:=s;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure ReadIniFile(Const AFileName: String;L:TStrings);'#010+
  'Var'#010+
  '  F : TFileStream;'#010+
  '  Line : String;'#010+
  '  I,P,PC : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ',' F:=TFileStream.Create(AFileName,fmOpenRead);'#010+
  '  Try'#010+
  '    L.LoadFromStream(F);'#010+
=======
  '  F:=TFileStream.Create(AFileName,fmOpenRead);'#010+
  '  Try'#010+
  '    L.LoadFromS','tream(F);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  F:=TFileStream.Create(AFileName,fmOpenRead);'#010+
  '  Try'#010+
  '    L.LoadFromS','tream(F);'#010+
>>>>>>> origin/fixes_2_2
  '    // Fix lines.'#010+
  '    For I:=L.Count-1 downto 0 do'#010+
  '      begin'#010+
  '        Line:=L[I];'#010+
  '        P:=Pos('#039'='#039',Line);'#010+
  '        PC:=Pos('#039';'#039',Line);  // Comment line.'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        I','f (P=0) or ((PC<>0) and (PC<P)) then'#010+
  '          L.Delete(I)'#010+
  '        else'#010+
=======
  '        If (P=0) or ((PC<>0) and (PC<P)) then'#010+
  '          L.Delete(I)'#010+
  '        ','else'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        If (P=0) or ((PC<>0) and (PC<P)) then'#010+
  '          L.Delete(I)'#010+
  '        ','else'#010+
>>>>>>> origin/fixes_2_2
  '          L[i]:=Trim(System.Copy(Line,1,P-1)+'#039'='#039'+Trim(System.C'+
  'opy(Line,P+1,Length(Line)-P)));'#010+
  '      end;'#010+
  '  Finally'#010+
  '    F.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '// Callback for Sysutils ge','tapplicationname.'#010+
  'Function GetFPMakeName : String;'#010+
  #010+
  'begin'#010+
  '  Result:='#039'fpmake'#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '// Callback for Sysutils getapplicationname.'#010+
  'Function GetFPMakeName : String;'#010+
  #010+
  'begin'#010+
  '  Result:','='#039'fpmake'#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function CurrentOS : String;'#010+
  #010+
  'begin'#010+
  '  Result:=OSToString(Defaults.OS);'#010+
  'end;'#010+
  #010+
  'Function CurrentCPU : String;'#010+
  #010+
  'begin'#010+
  '  Result:=CPUToString(Defaults.CPU);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'end;',#010+
=======
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  'Function OSToString(OS: TOS) : String;'#010+
  #010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=LowerCase(GetenumName(TypeInfo(TOS),Ord(OS)));'#010+
=======
  '  Result:=LowerCase','(GetenumName(TypeInfo(TOS),Ord(OS)));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result:=LowerCase','(GetenumName(TypeInfo(TOS),Ord(OS)));'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  'Function OSesToString(OSes: TOSes) : String;'#010+
  #010+
  'begin'#010+
  '  Result:=LowerCase(SetToString(PtypeInfo(TypeInfo(TOSes)),Integer(OSe'+
<<<<<<< HEAD
<<<<<<< HEAD
  's),Fals','e));'#010+
=======
  's),False));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  's),False));'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  'Function CPUToString(CPU: TCPU) : String;'#010+
  #010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=LowerCase(GetenumName(TypeInfo(TCPU),Ord(CPU)));'#010+
=======
  '  Resul','t:=LowerCase(GetenumName(TypeInfo(TCPU),Ord(CPU)));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Resul','t:=LowerCase(GetenumName(TypeInfo(TCPU),Ord(CPU)));'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  'Function CPUSToString(CPUS: TCPUS) : String;'#010+
  #010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=LowerCase(SetToString(PTypeInfo(TypeInfo(TCPUS)),Inte','ger(C'+
  'PUS),False));'#010+
  'end;'#010+
  #010+
  'Function StringToOS(const S : String) : TOS;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result:=LowerCase(SetToString(PTypeInfo(TypeInfo(TCPUS)),Integer(CPU'+
  'S),False));'#010+
  'end;'#010+
  #010+
  'Function StringToOS(const S : String) : TO','S;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  #010+
  'begin'#010+
  '  I:=GetEnumValue(TypeInfo(TOS),S);'#010+
  '  if (I=-1) then'#010+
  '    Raise EInstallerError.CreateFmt(SErrInvalidOS,[S]);'#010+
  '  Result:=TOS(I);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function ','OSesToString(const S : String) : TOSes;'#010+
  #010+
  'begin'#010+
  '  Result:=TOSes(StringToSet(PTypeInfo(TypeInfo(TOSes)),S));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function OSesToString(const S : String) : TOSes;'#010+
  #010+
  'begin'#010+
  '  Result:=TOSes(Stri','ngToSet(PTypeInfo(TypeInfo(TOSes)),S));'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  'Function StringToCPU(const S : String) : TCPU;'#010+
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  #010+
  'begin'#010+
  '  I:=GetEnumValue(TypeInfo(TCPU),S);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if (I=-1) then',#010+
  '    Raise EInstallerError.CreateFmt(SErrInvalidCPU,[S]);'#010+
  '  Result:=TCPU(I);'#010+
=======
  '  if (I=-1) then'#010+
  '    Raise EInstallerError.CreateFmt(SErrInvalidCPU,[S]);'#010+
  '  Result:','=TCPU(I);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  if (I=-1) then'#010+
  '    Raise EInstallerError.CreateFmt(SErrInvalidCPU,[S]);'#010+
  '  Result:','=TCPU(I);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  'Function StringToCPUS(const S : String) : TCPUS;'#010+
  #010+
  'begin'#010+
  '  Result:=TCPUS(StringToSet(PTypeInfo(TypeInfo(TCPUS)),S));'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function ModeToString(Mode: TCompil','erMode) : String;'#010+
  #010+
  'begin'#010+
  '  Result:=LowerCase(GetenumName(TypeInfo(TCompilerMode),Ord(Mode)));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function ModeToString(Mode: TCompilerMode) : String;'#010+
  #010+
  'begin'#010+
  '  Result:=LowerCase(GetenumName(TypeInfo(T','CompilerMode),Ord(Mode)));'+
  #010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Delete(Result,1,2);'#010+
  'end;'#010+
  #010+
  'Function StringToMode(const S : String) : TCompilerMode;'#010+
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  #010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  I:=GetEnumValue(TypeInfo(TCompil','erMode),S);'#010+
  '  if (I=-1) then'#010+
  '    Raise EInstallerError.CreateFmt(SErrInvalidMode,[S]);'#010+
=======
  '  I:=GetEnumValue(TypeInfo(TCompilerMode),S);'#010+
  '  if (I=-1) then'#010+
  '    Raise EInstallerError.CreateFmt(SE','rrInvalidMode,[S]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  I:=GetEnumValue(TypeInfo(TCompilerMode),S);'#010+
  '  if (I=-1) then'#010+
  '    Raise EInstallerError.CreateFmt(SE','rrInvalidMode,[S]);'#010+
>>>>>>> origin/fixes_2_2
  '  Result:=TCompilerMode(I);'#010+
  'end;'#010+
  #010+
  #010+
  'Function MakeTargetString(CPU : TCPU;OS: TOS) : String;'#010+
  #010+
  'begin'#010+
  '  Result:=CPUToString(CPU)+'#039'-'#039'+OSToString(OS);'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Pr','ocedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TO'+
  'S);'#010+
=======
  'Procedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TOS',
  ');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Procedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TOS',
  ');'#010+
>>>>>>> origin/fixes_2_2
  #010+
  'Var'#010+
  '  P : integer;'#010+
  #010+
  'begin'#010+
  '  P:=Pos('#039'-'#039',S);'#010+
  '  If (P=0) then'#010+
<<<<<<< HEAD
  '    Raise EInstallerError.CreateFmt(SErrInvalidTarget,[S]);'#010+
  '  CPU:=StringToCPU(Copy(S,1,P-1));'#010+
<<<<<<< HEAD
  '  OS:=StringToO','s(Copy(S,P+1,Length(S)-P));'#010+
  'end;'#010+
  #010+
  #010+
  'Procedure ResolveDependencies(L : TDependencies; P : TNamedCollection)'+
  ';'#010+
=======
=======
  '    Raise EInstallerError.CreateFmt(SErrInvalidTarget,[S]);'#010+
  '  CPU:=StringToCPU(Copy(S,1,P-1));'#010+
>>>>>>> origin/fixes_2_2
  '  OS:=StringToOs(Copy(S,P+1,Length(S)-P));'#010+
  'end;'#010+
  #010+
  #010+
  'Procedure ResolveDependencies(L ',': TDependencies; P : TNamedCollectio'+
  'n);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I,J : Integer;'#010+
  '  C : TDependency;'#010+
  'begin'#010+
  '  If Assigned(L) then'#010+
  '    For I:=0 to L.Count-1 do'#010+
  '      begin'#010+
  '        C:=L[i];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '       ',' if C.DependencyType in [depPackage,depUnit] then'#010+
  '          begin'#010+
  '            J:=P.IndexOfName(C.Value);'#010+
=======
  '        if C.DependencyType in [depPackage,depUnit] then'#010+
  '          begin'#010+
  ' ','           J:=P.IndexOfName(C.Value);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        if C.DependencyType in [depPackage,depUnit] then'#010+
  '          begin'#010+
  ' ','           J:=P.IndexOfName(C.Value);'#010+
>>>>>>> origin/fixes_2_2
  '            If J<>-1 then'#010+
  '              C.Target:=P.Items[J];'#010+
  '          end;'#010+
  '      end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function AddConditionalStrings(Dest : TS','trings; Src : TConditionalSt'+
  'rings;ACPU:TCPU;AOS:TOS; Const APrefix : String='#039#039') : Integer ;'#010+
=======
  'function AddConditionalStrings(Dest : TStrings; Src : TConditionalStri'+
  'ngs;ACPU:TCPU;AOS:TOS; Const APrefix ',': String='#039#039') : Integer ;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function AddConditionalStrings(Dest : TStrings; Src : TConditionalStri'+
  'ngs;ACPU:TCPU;AOS:TOS; Const APrefix ',': String='#039#039') : Integer ;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  '  C : TConditionalString;'#010+
  '  S : String;'#010+
  'begin'#010+
  '  Result:=0;'#010+
  '  Dictionary.AddVariable('#039'CPU'#039',CPUToString(ACPU));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Dictionary.AddVar','iable('#039'OS'#039',OSToString(AOS));'#010+
  '  For I:=0 to Src.Count-1 do'#010+
  '    begin'#010+
=======
  '  Dictionary.AddVariable('#039'OS'#039',OSToString(AOS));'#010+
  '  For I:=0 to Src.Count-1 do'#010+
  '    begin',#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Dictionary.AddVariable('#039'OS'#039',OSToString(AOS));'#010+
  '  For I:=0 to Src.Count-1 do'#010+
  '    begin',#010+
>>>>>>> origin/fixes_2_2
  '      C:=Src[I];'#010+
  '      if (ACPU in C.CPUs) and (AOS in C.OSes) then'#010+
  '        begin'#010+
  '          If (APrefix<>'#039#039') then'#010+
  '            S:=APrefix+C.Value'#010+
  '          else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            ','S:=C.Value;'#010+
  '          Dest.Add(Dictionary.ReplaceStrings(S));'#010+
  '          Inc(Result);'#010+
=======
  '            S:=C.Value;'#010+
  '          Dest.Add(Dictionary.ReplaceStrings(S));'#010+
  '     ','     Inc(Result);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '            S:=C.Value;'#010+
  '          Dest.Add(Dictionary.ReplaceStrings(S));'#010+
  '     ','     Inc(Result);'#010+
>>>>>>> origin/fixes_2_2
  '        end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'function FileListToString(List : TStrings; const APrefix : String) : S'+
  'tring;'#010+
  'Var'#010+
  '  I : integer;'#010+
  '  S : String;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=',#039#039';'#010+
  '  For I:=0 to List.Count-1 do'#010+
  '    begin'#010+
  '      If (I>0) then'#010+
  '        Result:=Result+'#039' '#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result:='#039#039';'#010+
  '  For I:=0 to List.Count-1 do'#010+
  '    begin'#010+
  '      If (I>0) then'#010+
  '   ','     Result:=Result+'#039' '#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      S:=APrefix+List[i];'#010+
  '      If (Pos('#039' '#039',S)<>0) then'#010+
  '        S:='#039'"'#039'+S+'#039'"'#039';'#010+
  '      Result:=Result+S;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function FixPath (const APath ',': String) : String;'#010+
=======
  'function FixPath (const APath : String) : String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function FixPath (const APath : String) : String;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  P : PChar;'#010+
  'begin'#010+
  '  Result:=APath;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If (result<>'#039#039') then'#010+
=======
  '  If (r','esult<>'#039#039') then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If (r','esult<>'#039#039') then'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      P:=PChar(Result);'#010+
  '      While (P^<>#0) do'#010+
  '        begin'#010+
  '          If P^ in ['#039'/'#039','#039'\'#039'] then'#010+
  '            P^:=PathDelim;'#010+
  '          Inc(P);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      ','  end;'#010+
=======
  '        end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        end;'#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure ChangeDir(const APath : String);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begin'#010+
=======
  'b','egin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'b','egin'#010+
>>>>>>> origin/fixes_2_2
  '  if Not SetCurrentDir(APath) then'#010+
  '    Raise EInstallerError.CreateFmt(SErrChangeDirFailed,[APath]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure SearchFiles(const AFileName: string; Recursive: bo','olean; v'+
  'ar List: TStrings);'#010+
  #010+
  '  procedure AddRecursiveFiles(const SearchDir, FileMask: string; Recur'+
  'sive: boolean);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure SearchFiles(const AFileName: string; Recursive: boolean; var'+
  ' List: TStrings);'#010+
  #010+
  '  procedure AddRecursiveFiles(const Se','archDir, FileMask: string; Rec'+
  'ursive: boolean);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  const'#010+
  '    IgnoreCase = {$ifdef UNIX}False{$else}True{$endif};'#010+
  '  var'#010+
  '    Info : TSearchRec;'#010+
  '  begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    if FindFirst(SearchD','ir+'#039'*'#039',faAnyFile and faDirectory,Inf'+
  'o)=0 then'#010+
  '    begin'#010+
  '      repeat'#010+
  '          if ((Info.Attr and faDirectory) = faDirectory) and (Info.Nam'+
  'e <> '#039'.'#039') and (Info.Name <> '#039'..'#039') and (Recursive) then'+
  #010+
  '            AddRecursiveFiles(SearchDir + Info','.Name + PathDelim, Fil'+
  'eMask, Recursive);'#010+
  '          if ((Info.Attr and faDirectory) <> faDirectory) and IsWild(I'+
  'nfo.Name, FileMask, IgnoreCase) then'#010+
  '            List.Add(SearchDir + Info.Name);'#010+
  '      until FindNext(Info)<>0;'#010+
  '    end;'#010+
  '    Fin','dClose(Info);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    if FindFirst(SearchDir+'#039'*'#039',faAnyFile and faDirectory,Info)'+
  '=0 then'#010+
  '    begin'#010+
  '      repea','t'#010+
  '          if ((Info.Attr and faDirectory) = faDirectory) and (Info.Nam'+
  'e <> '#039'.'#039') and (Info.Name <> '#039'..'#039') and (Recursive) then'+
  #010+
  '            AddRecursiveFiles(SearchDir + Info.Name + PathDelim, FileM'+
  'ask, Recursive);'#010+
  '          if ((Info.Attr a','nd faDirectory) <> faDirectory) and IsWild'+
  '(Info.Name, FileMask, IgnoreCase) then'#010+
  '            List.Add(SearchDir + Info.Name);'#010+
  '      until FindNext(Info)<>0;'#010+
  '    end;'#010+
  '    FindClose(Info);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  'var'#010+
  '  CurrDir,'#010+
  '  BasePath: string;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  i: integer;'#010+
=======
  '  i: integ','er;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  i: integ','er;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  BasePath := ExtractFilePath(ExpandFileName(AFileName));'#010+
  '  AddRecursiveFiles(BasePath, ExtractFileName(AFileName), Recursive);'#010+
  #010+
  '  CurrDir:=GetCurrentDir;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  for i ',':= 0 to Pred(List.Count) do'#010+
  '    List[i] := ExtractRelativepath(IncludeTrailingPathDelimiter(CurrDi'+
  'r), List[i]);'#010+
  'end;'#010+
  #010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  for i := 0 to Pred(List.Count) do'#010+
  '    List[i] := ExtractRelativepath(Incl','udeTrailingPathDelimiter(Curr'+
  'Dir), List[i]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure SplitCommand(const Cmd : String; var Exe, Options : String);'+
  #010+
  #010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'Const'#010+
  '  WhiteSpace = [#9,#10,#13,'#039' '#039'];'#010+
  '  QuoteChars = ['#039#039#039#039','#039'"'#039'];'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure SplitCommand(const Cmd : String; var Exe, Opt','ions : String'+
  ');'#010+
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  '  InQuote : Boolean;'#010+
  '  LastQuote : Char;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  '  InQuote : Boolean;'#010+
  '  LastQuo','te : Char;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  S : String;'#010+
  #010+
  'begin'#010+
  '  S:=Trim(Cmd);'#010+
  '  InQuote:=False;'#010+
  '  LastQuote:=#0;'#010+
  '  I:=1;'#010+
  '  While (I<=Length(S)) and (Inquote or not (S[I] in whiteSpace)) do'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    I','f S[i] in QuoteChars then'#010+
  '      begin'#010+
  '      InQuote:=Not (S[i]=LastQuote);'#010+
=======
  '    If S[i] in QuoteChars then'#010+
  '      begin'#010+
  '      InQuote:=Not (S[i]=Las','tQuote);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    If S[i] in QuoteChars then'#010+
  '      begin'#010+
  '      InQuote:=Not (S[i]=Las','tQuote);'#010+
>>>>>>> origin/fixes_2_2
  '      If InQuote then'#010+
  '         LastQuote:=S[i]'#010+
  '       else'#010+
  '         LastQuote:=#0;'#010+
  '      end;'#010+
  '    Inc(I);'#010+
  '    end;'#010+
  '  Exe:=Copy(S,1,I-1);'#010+
  '  Delete(S,1,I);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Options:=','Trim(S);'#010+
  'end;'#010+
  #010+
  'Function OptionListToString(L : TStrings) : String;'#010+
  #010+
  'var'#010+
  '  I : Integer;'#010+
  '  S : String;'#010+
  #010+
  'begin'#010+
  '  Result:='#039#039';'#010+
  '  For I:=0 to L.Count-1 do'#010+
  '    begin'#010+
  '    If (Result<>'#039#039') then'#010+
  '      Result:=Result+'#039' '#039';'#010+
  '    S:=L[I];'#010+
  '    If (Pos('#039' '#039',S',')<>0) or (S='#039#039') then'#010+
  '     Result:='#039'"'#039'+S+'#039'"'#039';'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  'Function OptionsToStringList(S : String) : TStrings;'#010+
  #010+
  'Var'#010+
  '  P : Integer;'#010+
  #010+
  'begin'#010+
  '  Result:=Nil;'#010+
  '  If (S='#039#039') then'#010+
  '    Exit;'#010+
  '  Result:=TStringList.Create;'#010+
  '  Repeat'#010+
  '    P:=Pos('#039' '#039',S);'#010,
  '    If P=0 then'#010+
  '      P:=Length(S)+1;'#010+
  '    Result.Add(Copy(S,1,P-1));'#010+
  '    Delete(S,1,P);'#010+
  '    S:=Trim(S);'#010+
  '  Until Length(S)=0;'#010+
  '  If Result.Count=0 then'#010+
  '    FreeAndNil(Result);'#010+
=======
  '  Options:=Trim(S);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Options:=Trim(S);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{$ifdef HAS_UNIT_PROCESS}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function GetCompilerInfo(const AC','ompiler,AOptions:string):string;'#010+
=======
  'function GetCompilerInfo','(const ACompiler,AOptions:string):string;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function GetCompilerInfo','(const ACompiler,AOptions:string):string;'#010+
>>>>>>> origin/fixes_2_2
  'const'#010+
  '  BufSize = 1024;'#010+
  'var'#010+
  '  S: TProcess;'#010+
  '  Buf: array [0..BufSize - 1] of char;'#010+
  '  Count: longint;'#010+
  'begin'#010+
  '  S:=TProcess.Create(Nil);'#010+
  '  S.Commandline:=ACompiler+'#039' '#039'+AOptions;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  S.Options:=[poUsePipes];'#010+
  '  S.ex','ecute;'#010+
=======
  '  S.Options:=[poUsePipes','];'#010+
  '  S.execute;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  S.Options:=[poUsePipes','];'#010+
  '  S.execute;'#010+
>>>>>>> origin/fixes_2_2
  '  Count:=s.output.read(buf,BufSize);'#010+
  '  S.Free;'#010+
  '  SetLength(Result,Count);'#010+
  '  Move(Buf,Result[1],Count);'#010+
  'end;'#010+
  '{$endif HAS_UNIT_PROCESS}'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                   ','        TUnsortedDuplicatesStringList'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'function TUnsortedDuplicatesStringList.Add(const S: string): Integer;'#010+
  #010+
  'begin'#010+
  '  result := IndexOf(S);'#010+
  '  If result > -1 then'#010,
  '    Case DUplicates of'#010+
  '      DupIgnore : Exit;'#010+
  '      DupError : Error(SDuplicateString,0)'#010+
  '    end;'#010+
  '  inherited Add(S);'#010+
  'end;'#010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                                TName','dItem'#010+
=======
  '          ','                      TNamedItem'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '          ','                      TNamedItem'#010+
>>>>>>> origin/fixes_2_2
  '**********************************************************************'+
  '******}'#010+
  #010+
  'procedure TNamedItem.SetName(const AValue: String);'#010+
  #010+
  'begin'#010+
  '  if FName=AValue then exit;'#010+
  '  With TNamedCollection(Collection) do'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    If UniqueNames then'#010+
  '    ','  If (IndexOfName(AVAlue)<>-1) then'#010+
=======
  ' ','   If UniqueNames then'#010+
  '      If (IndexOfName(AVAlue)<>-1) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ','   If UniqueNames then'#010+
  '      If (IndexOfName(AVAlue)<>-1) then'#010+
>>>>>>> origin/fixes_2_2
  '        Raise ECollectionError.CreateFmt(SErrNameExists,[AValue]);'#010+
  '  FName:=AValue;'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                                TNa','medCollection'#010+
=======
  '        ','                        TNamedCollection'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        ','                        TNamedCollection'#010+
>>>>>>> origin/fixes_2_2
  '**********************************************************************'+
  '******}'#010+
  #010+
  'function TNamedCollection.IndexOfName(const AName: String): Integer;'#010+
  #010+
  'begin'#010+
  '  Result:=Count-1;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  While (Result>=0) and (CompareText(TNamedItem(Item','s[Result]).FName'+
=======
  '  While (Result>=0) and (','CompareText(TNamedItem(Items[Result]).FName'+
>>>>>>> graemeg/fixes_2_2
=======
  '  While (Result>=0) and (','CompareText(TNamedItem(Items[Result]).FName'+
>>>>>>> origin/fixes_2_2
  ',AName)<>0) do'#010+
  '    Dec(Result);'#010+
  'end;'#010+
  #010+
  'function TNamedCollection.ItemByName(const AName: String): TNamedItem;'+
  #010+
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  #010+
  'begin'#010+
  '  I:=IndexOfName(AName);'#010+
  '  If (I=-1) Then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Raise ECollectionError.CreateFmt(SErrNoSuc','hName,[AName]);'#010+
=======
  '    Raise ECollecti','onError.CreateFmt(SErrNoSuchName,[AName]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Raise ECollecti','onError.CreateFmt(SErrNoSuchName,[AName]);'#010+
>>>>>>> origin/fixes_2_2
  '  Result:=TNamedItem(Items[i]);'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                             TNamedItemList'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '***************************************************************','*****'+
=======
  '************************************','********************************'+
>>>>>>> graemeg/fixes_2_2
=======
  '************************************','********************************'+
>>>>>>> origin/fixes_2_2
  '********}'#010+
  #010+
  'function TNamedItemList.GetNamedItem(Index : Integer): TNamedItem;'#010+
  'begin'#010+
  '  Result:=TNamedItem(Items[Index]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TNamedItemList.SetNamedItem(Index : Integer; const AValue: T'+
<<<<<<< HEAD
<<<<<<< HEAD
  'NamedItem);'#010+
  'begin'#010+
  '  Items[Index]:=AVal','ue;'#010+
=======
  'NamedItem);',#010+
  'begin'#010+
  '  Items[Index]:=AValue;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'NamedItem);',#010+
  'begin'#010+
  '  Items[Index]:=AValue;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'function TNamedItemList.IndexOfName(const AName: String): Integer;'#010+
  'begin'#010+
  '  Result:=Count-1;'#010+
  '  While (Result>=0) and (CompareText(GetNamedItem(Result).Name,AName)<'+
  '>0) do'#010+
  '    Dec(Result);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TNamedItemList.ItemByName(c','onst ANAme: String): TNamedItem;'+
=======
  'function ','TNamedItemList.ItemByName(const ANAme: String): TNamedItem;'+
>>>>>>> graemeg/fixes_2_2
=======
  'function ','TNamedItemList.ItemByName(const ANAme: String): TNamedItem;'+
>>>>>>> origin/fixes_2_2
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  'begin'#010+
  '  I:=IndexOfName(AName);'#010+
  '  If (I=-1) Then'#010+
  '    Raise ECollectionError.CreateFmt(SErrNoSuchName,[AName]);'#010+
  '  Result:=TNamedItem(Items[i]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{**************************************','*****************************'+
=======
  '{***********','********************************************************'+
>>>>>>> graemeg/fixes_2_2
=======
  '{***********','********************************************************'+
>>>>>>> origin/fixes_2_2
  '*********'#010+
  '                                TTargets'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TTargets.GetTargetItem(Index : Integer): TTarget;'#010+
  'begin'#010+
  '  Result:=TTarge','t(Items[Index]);'#010+
=======
  'function TTargets.GetTargetItem(Index : Integer): TTar','get;'#010+
  'begin'#010+
  '  Result:=TTarget(Items[Index]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TTargets.GetTargetItem(Index : Integer): TTar','get;'#010+
  'begin'#010+
  '  Result:=TTarget(Items[Index]);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'function TTargets.GetTarget(const AName : String): TTarget;'#010+
  'begin'#010+
  '  Result:=TTarget(ItemByName(AName));'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TTargets.SetTargetItem(Index : Integer; const AValue: TTarge'+
  't);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begin'#010+
  '  Items[Index]:=AValue;'#010+
  'e','nd;'#010+
=======
  'begi','n'#010+
  '  Items[Index]:=AValue;'#010+
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begi','n'#010+
  '  Items[Index]:=AValue;'#010+
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'Function TTargets.AddUnit(Const AUnitName : String) : TTarget;'#010+
  'begin'#010+
  '  Result:=AddUnit(AUnitName,AllCPUs,AllOSes);'#010+
  'end;'#010+
  #010+
  #010+
  'Function TTargets.AddUnit(Const AUnitName : String;const OSes:TOSes) :'+
  ' TTarget;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begin'#010+
  '  Result:=AddUnit(AUnitNam','e,AllCPUs,OSes);'#010+
=======
  'begin',#010+
  '  Result:=AddUnit(AUnitName,AllCPUs,OSes);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begin',#010+
  '  Result:=AddUnit(AUnitName,AllCPUs,OSes);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{$ifdef cpu_only_overloads}'#010+
  'Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs) :'+
  ' TTarget;'#010+
  'begin'#010+
  '  Result:=AddUnit(AUnitName,CPUs,AllOSes);'#010+
  'end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
  'Function TTargets.AddUni','t(Const AUnitName : String;const CPUs:TCPUs;'+
  'const OSes:TOSes) : TTarget;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{$endif cpu_only_overloads}',#010+
  #010+
  #010+
  'Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs;co'+
  'nst OSes:TOSes) : TTarget;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=Add as TTarget;'#010+
  '  Result.Name:=AUnitName;'#010+
  '  Result.TargetType:=TTUnit;'#010+
  '  Result.CPUs:=CPUs;'#010+
  '  Result.OSes:=OSes;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddImplicitUn','it(Const AUnitName : String;InstallUn'+
=======
  'Func','tion TTargets.AddImplicitUnit(Const AUnitName : String;InstallUn'+
>>>>>>> graemeg/fixes_2_2
=======
  'Func','tion TTargets.AddImplicitUnit(Const AUnitName : String;InstallUn'+
>>>>>>> origin/fixes_2_2
  'it:boolean=true) : TTarget;'#010+
  'begin'#010+
  '  Result:=AddImplicitUnit(AUnitName,AllCPUs,AllOSes,InstallUnit);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddImplicitUnit(Const AUnitName : String;const OSes:'+
  'TOSes;InstallUnit:boolean=','true) : TTarget;'#010+
=======
  'Function TTargets.AddImplicitUnit(Const AUnitName : String;const OSes',
  ':TOSes;InstallUnit:boolean=true) : TTarget;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TTargets.AddImplicitUnit(Const AUnitName : String;const OSes',
  ':TOSes;InstallUnit:boolean=true) : TTarget;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=AddImplicitUnit(AUnitName,AllCPUs,OSes,InstallUnit);'#010+
  'end;'#010+
  #010+
  #010+
  'Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:'+
  'TCPUs;InstallUnit:boolean=true) : TTarget;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=AddImplicitUnit(AU','nitName,CPUs,AllOSes,InstallUnit);'#010+
=======
  ' ',' Result:=AddImplicitUnit(AUnitName,CPUs,AllOSes,InstallUnit);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ',' Result:=AddImplicitUnit(AUnitName,CPUs,AllOSes,InstallUnit);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:'+
  'TCPUs;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;'#010+
  'begin'#010+
  '  Result:=Add as TTarget;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result.Name:=AUnitName;'#010+
  '  Result.C','PUs:=CPUs;'#010+
=======
  '  Result.','Name:=AUnitName;'#010+
  '  Result.CPUs:=CPUs;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result.','Name:=AUnitName;'#010+
  '  Result.CPUs:=CPUs;'#010+
>>>>>>> origin/fixes_2_2
  '  Result.OSes:=OSes;'#010+
  '  if InstallUnit then'#010+
  '    Result.TargetType:=TTImplicitUnit'#010+
  '  else'#010+
  '    Result.TargetType:=TTCleanOnlyUnit;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddProgram(Const AProgramName : String) : TTarget;'#010+
  'begin'#010+
  '  Result:=AddProgra','m(AProgramName,AllCPUs,AllOSes);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TTargets.AddProgram(Const AProgramName : String) : TTarget',';'+
  #010+
  'begin'#010+
  '  Result:=AddProgram(AProgramName,AllCPUs,AllOSes);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TTargets.AddProgram(Const AProgramName : String;const OSes:TO'+
  'Ses) : TTarget;'#010+
  'begin'#010+
  '  Result:=AddProgram(AProgramName,AllCPUs,OSes);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$ifdef cpu_only_overloads}'#010+
  'Function TTargets.AddPro','gram(Const AProgramName : String;const CPUs:'+
  'TCPUs) : TTarget;'#010+
=======
  '{$ifdef cpu_only_overload','s}'#010+
  'Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TC'+
  'PUs) : TTarget;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '{$ifdef cpu_only_overload','s}'#010+
  'Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TC'+
  'PUs) : TTarget;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=AddProgram(AProgramName,CPUs,AllOSes);'#010+
  'end;'#010+
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TC'+
  'PUs;const OSes:TOS','es) : TTarget;'#010+
=======
  'Function TTargets.AddProgram(Const AProgramName : String;cons','t CPUs:'+
  'TCPUs;const OSes:TOSes) : TTarget;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TTargets.AddProgram(Const AProgramName : String;cons','t CPUs:'+
  'TCPUs;const OSes:TOSes) : TTarget;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=Add as TTarget;'#010+
  '  Result.Name:=AProgramName;'#010+
  '  Result.CPUs:=CPUs;'#010+
  '  Result.OSes:=OSes;'#010+
  '  Result.TargetType:=ttProgram;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddExampleUnit(Const AUnitName : String) : TTarget;'#010+
  'begin'#010+
  '  Resul','t:=AddExampleUnit(AUnitName,AllCPUs,AllOSes);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TTargets.AddExampleUnit(Const AUnitName : Strin','g) : TTarget'+
  ';'#010+
  'begin'#010+
  '  Result:=AddExampleUnit(AUnitName,AllCPUs,AllOSes);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TTargets.AddExampleUnit(Const AUnitName : String;const OSes:T'+
  'OSes) : TTarget;'#010+
  'begin'#010+
  '  Result:=AddExampleUnit(AUnitName,AllCPUs,OSes);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$ifdef cpu_only_overloads}'#010+
  'Function ','TTargets.AddExampleUnit(Const AUnitName : String;const CPUs'+
  ':TCPUs) : TTarget;'#010+
=======
  '{$ifdef cp','u_only_overloads}'#010+
  'Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:T'+
  'CPUs) : TTarget;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '{$ifdef cp','u_only_overloads}'#010+
  'Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:T'+
  'CPUs) : TTarget;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=AddExampleUnit(AUnitName,CPUs,AllOSes);'#010+
  'end;'#010+
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:T'+
  'C','PUs;const OSes:TOSes) : TTarget;'#010+
=======
  'Function TTargets.AddExampleUnit(Const AUnit','Name : String;const CPUs'+
  ':TCPUs;const OSes:TOSes) : TTarget;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TTargets.AddExampleUnit(Const AUnit','Name : String;const CPUs'+
  ':TCPUs;const OSes:TOSes) : TTarget;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=Add as TTarget;'#010+
  '  Result.Name:=AUnitName;'#010+
  '  Result.CPUs:=CPUs;'#010+
  '  Result.OSes:=OSes;'#010+
  '  Result.TargetType:=ttExampleUnit;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TTargets.AddExampleProgram(Const AProgramName : String)',' : T'+
=======
  'Function TTargets.AddExampleProgram(C','onst AProgramName : String) : T'+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TTargets.AddExampleProgram(C','onst AProgramName : String) : T'+
>>>>>>> origin/fixes_2_2
  'Target;'#010+
  'begin'#010+
  '  Result:=AddExampleProgram(AProgramName,AllCPUs,AllOSes);'#010+
  'end;'#010+
  #010+
  #010+
  'Function TTargets.AddExampleProgram(Const AProgramName : String;const '+
  'OSes:TOSes) : TTarget;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=AddExampleProgram(AProgramName,AllCPUs,OSes);'#010+
  'e','nd;'#010+
=======
  '  Result:=AddExampleProgram(AP','rogramName,AllCPUs,OSes);'#010+
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result:=AddExampleProgram(AP','rogramName,AllCPUs,OSes);'#010+
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  '{$ifdef cpu_only_overloads}'#010+
  'Function TTargets.AddExampleProgram(Const AProgramName : String;const '+
  'CPUs:TCPUs) : TTarget;'#010+
  'begin'#010+
  '  Result:=AddExampleProgram(AProgramName,CPUs,AllOSes);'#010+
  'end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
  'Function TTarget','s.AddExampleProgram(Const AProgramName : String;cons'+
  't CPUs:TCPUs;const OSes:TOSes) : TTarget;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{$endif cpu_only_ov','erloads}'#010+
  #010+
  #010+
  'Function TTargets.AddExampleProgram(Const AProgramName : String;const '+
  'CPUs:TCPUs;const OSes:TOSes) : TTarget;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=Add as TTarget;'#010+
  '  Result.Name:=AProgramName;'#010+
  '  Result.CPUs:=CPUs;'#010+
  '  Result.OSes:=OSes;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result.TargetType:=ttExampleProgram;'#010+
  'end;',#010+
=======
  '  Result.TargetT','ype:=ttExampleProgram;'#010+
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result.TargetT','ype:=ttExampleProgram;'#010+
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                                TSources'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TSources.GetSourceItem(Index :',' Integer): TSource;'#010+
=======
  'function TSo','urces.GetSourceItem(Index : Integer): TSource;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TSo','urces.GetSourceItem(Index : Integer): TSource;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=TSource(Items[Index]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TSources.SetSourceItem(Index : Integer; const AValue: TSourc'+
  'e);'#010+
  'begin'#010+
  '  Items[Index]:=AValue;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TSources.AddDoc (const AFiles : String) : TSource;'#010+
  'beg','in'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
  '  Result.FSourceType:=stDoc;'#010+
  'end;'#010+
  #010+
  #010+
  'function TSources.AddDoc(const AFiles: String; AInstallSourcePath: Str'+
  'ing): TSource;'#010+
  'begin'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
  '  Result.FIn','stallSourcePath:=AInstallSourcePath;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'function TSources.AddDoc (const AFil','es : String) : TSource;'#010+
  'begin'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Result.FSourceType:=stDoc;'#010+
  'end;'#010+
  #010+
  #010+
  'function TSources.AddSrc(const AFiles : String) : TSource;'#010+
  'begin'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result.FSourceType:=stSrc;'#010+
  'end;'#010+
  #010+
  #010+
  'function TSources','.AddExample(const AFiles : String) : TSource;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result.FS','ourceType:=stSrc;'#010+
  'end;'#010+
  #010+
  #010+
  'function TSources.AddExample(const AFiles : String) : TSource;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
  '  Result.FSourceType:=stExample;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TSources.AddExample(const AFiles: String; AInstallSourcePath:'+
  ' String): TSource;'#010+
  'begin'#010+
  '  Res','ult:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
  '  Result.FInstallSourcePath:=AInstallSourcePath;'#010+
  '  Result.FSourceType:=stExample;'#010+
  'end;'#010+
  #010+
  #010+
  'function TSources.AddTest(const AFiles : String) : TSource;'#010+
  'begin'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=','AFiles;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  #010+
  'function TSources.AddTest(const AFiles : String) : TSourc','e;'#010+
  'begin'#010+
  '  Result:=Add as TSource;'#010+
  '  Result.Name:=AFiles;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Result.FSourceType:=stTest;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TSources.AddDocFiles(const AFileMask: string; Recursive: boo'+
<<<<<<< HEAD
<<<<<<< HEAD
  'lean; AInstallSourcePath : String = '#039#039');'#010+
=======
  'lean);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'lean);'#010+
>>>>>>> origin/fixes_2_2
  'var'#010+
  '  List : TStrings;'#010+
  '  i: integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  List := TStringList.Create;'#010+
  '  SearchFil','es(AFileMask, Recursive, List);'#010+
  '  for i:= 0 to Pred(List.Count) do'#010+
  '    AddDoc(List[i], AInstallSourcePath);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  List := TStringList.Cre','ate;'#010+
  '  SearchFiles(AFileMask, Recursive, List);'#010+
  '  for i:= 0 to Pred(List.Count) do'#010+
  '    AddDoc(List[i]);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  List.Free;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TSources.AddSrcFiles(const AFileMask: string; Recursive: boo'+
  'lean);'#010+
  'var'#010+
  '  List : TStrings;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  i: integer',';'#010+
  'begin'#010+
=======
  '  i: integer;'#010+
  'be','gin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  i: integer;'#010+
  'be','gin'#010+
>>>>>>> origin/fixes_2_2
  '  List := TStringList.Create;'#010+
  '  SearchFiles(AFileMask, Recursive, List);'#010+
  '  for i:= 0 to Pred(List.Count) do'#010+
  '    AddSrc(List[i]);'#010+
  '  List.Free;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TSources.AddExampleFiles(const AFileMask: string; Recursive:'+
<<<<<<< HEAD
<<<<<<< HEAD
  ' boolean; AIn','stallSourcePath : String = '#039#039');'#010+
  'var'#010+
  '  List : TStrings;'#010+
=======
  ' boolean);'#010+
  'var'#010+
  '  ','List : TStrings;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' boolean);'#010+
  'var'#010+
  '  ','List : TStrings;'#010+
>>>>>>> origin/fixes_2_2
  '  i: integer;'#010+
  'begin'#010+
  '  List := TStringList.Create;'#010+
  '  SearchFiles(AFileMask, Recursive, List);'#010+
  '  for i:= 0 to Pred(List.Count) do'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    AddExample(List[i], AInstallSourcePath);'#010+
  '  List.Free;',#010+
  'end;'#010+
  #010+
  #010+
  'procedure TSources.AddTestFiles(const AFileMask: string; Recursive: bo'+
  'olean);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    AddExample(List[i]);'#010+
  '  List.Free;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TSources.AddTestFiles(const AFileMask: s','tring; Recursive: '+
  'boolean);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'var'#010+
  '  List : TStrings;'#010+
  '  i: integer;'#010+
  'begin'#010+
  '  List := TStringList.Create;'#010+
  '  SearchFiles(AFileMask, Recursive, List);'#010+
  '  for i:= 0 to Pred(List.Count) do'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '   ',' AddTest(List[i]);'#010+
=======
  '    AddTest(List[i]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    AddTest(List[i]);'#010+
>>>>>>> origin/fixes_2_2
  '  List.Free;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{*********************************************************************'+
  '*******'#010+
=======
  '{******************','*************************************************'+
  '*********'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '{******************','*************************************************'+
  '*********'#010+
>>>>>>> origin/fixes_2_2
  '                             TPackage'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'constr','uctor TPackage.Create(ACollection: TCollection);'#010+
  'begin'#010+
  '  inherited Create(ACollection);'#010+
  '  FVersion:=TFPVersion.Create;'#010+
  '  FTargets:=TTargets.Create(TTarget);'#010+
  '  FSources:=TSources.Create(TSource);'#010+
  '  FDependencies:=TDependencies.Create(TDepend','ency);'#010+
  '  FInstallFiles:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FCleanFiles:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FUnitPath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FObjectPath:=TConditionalStrings.Create(TC','onditionalString);'#010+
  '  FIncludePath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FSourcePath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FExamplePath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FTestPath:=TConditionalStrin','gs.Create(TConditionalString);'#010+
  '  FCommands:=TCommands.Create(TCommand);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'constructor TPackage.Create(ACollection: TCollection);'#010+
  'begin'#010+
  '  i','nherited Create(ACollection);'#010+
  '  FVersion:=TFPVersion.Create;'#010+
  '  FTargets:=TTargets.Create(TTarget);'#010+
  '  FSources:=TSources.Create(TSource);'#010+
  '  FDependencies:=TDependencies.Create(TDependency);'#010+
  '  FInstallFiles:=TConditionalStrings.Create(TCondit','ionalString);'#010+
  '  FCleanFiles:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FUnitPath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FObjectPath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FIncludePath:=TConditionalStrings.Cre','ate(TConditionalString);'#010+
  '  FSourcePath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FExamplePath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FTestPath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FCommands:=TCommands.Crea','te(TCommand);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  FCPUs:=AllCPUs;'#010+
  '  FOSes:=AllOSes;'#010+
  '  FInstalledChecksum:=$ffffffff;'#010+
  '  // Implicit dependency on RTL'#010+
  '  FDependencies.Add('#039'rtl'#039');'#010+
  'end;'#010+
  #010+
  #010+
  'destructor TPackage.destroy;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'beg','in'#010+
  '  FreeAndNil(FDependencies);'#010+
  '  FreeAndNil(FInstallFiles);'#010+
=======
  'begin'#010+
  '  FreeAndNil(FDependencies);'#010+
  '  FreeAndNil(FInstallFiles',');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begin'#010+
  '  FreeAndNil(FDependencies);'#010+
  '  FreeAndNil(FInstallFiles',');'#010+
>>>>>>> origin/fixes_2_2
  '  FreeAndNil(FCleanFiles);'#010+
  '  FreeAndNil(FIncludePath);'#010+
  '  FreeAndNil(FSourcePath);'#010+
  '  FreeAndNil(FExamplePath);'#010+
  '  FreeAndNil(FTestPath);'#010+
  '  FreeAndNil(FObjectPath);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FreeAndNil(FUni','tPath);'#010+
  '  FreeAndNil(FSources);'#010+
  '  FreeAndNil(FTargets);'#010+
  '  FreeAndNil(FVersion);'#010+
  '  FreeAndNil(FOptions);'#010+
  '  inherited destroy;'#010+
  'end;'#010+
  #010+
  'function TPackage.HaveOptions: Boolean;'#010+
  'begin'#010+
  '  Result:=(FOptions<>Nil);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.SetName(co','nst AValue: String);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  FreeAndNil(FUnitPath);'#010+
  '  FreeAndNil(FSources);'#010+
  '  FreeAndNil(FTargets);'#010+
  '  ','FreeAndNil(FVersion);'#010+
  '  inherited destroy;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.SetName(const AValue: String);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  inherited SetName(AValue);'#010+
  '  // RTL should not have any dependencies'#010+
  '  if AValue='#039'rtl'#039' then'#010+
  '    FDependencies.Clear;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TPackage.GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):String;'#010+
  'begin'#010+
  '  Result:='#039'unit','s'#039'+PathDelim+MakeTargetString(ACPU,AOS);'#010+
=======
  'Function',' TPackage.GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):String;'#010+
  'begin'#010+
  '  Result:='#039'units'#039'+PathDelim+MakeTargetString(ACPU,AOS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function',' TPackage.GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):String;'#010+
  'begin'#010+
  '  Result:='#039'units'#039'+PathDelim+MakeTargetString(ACPU,AOS);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TPackage.GetBinOutputDir(ACPU:TCPU; AOS : TOS) : String;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:='#039'bin'#039'+PathDelim+MakeTargetString(ACPU,AOS);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.GetCleanFiles(List: TStrings; ACPU:TCPU;',' AOS : TO'+
  'S);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result:='#039'bin'#039'+PathDelim+MakeTargetStrin','g(ACPU,AOS);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.GetCleanFiles(List: TStrings; ACPU:TCPU; AOS : TOS)'+
  ';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  OB,OU : String;'#010+
  '  I : Integer;'#010+
  'begin'#010+
  '  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaul'+
  'ts.OS));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defa'+
  'ults.OS));'#010+
  '  AddConditionalStrings(','List,CleanFiles,ACPU,AOS);'#010+
=======
  '  OU:=IncludeTrailingPathD','elimiter(GetUnitsOutputDir(Defaults.CPU,De'+
  'faults.OS));'#010+
  '  AddConditionalStrings(List,CleanFiles,ACPU,AOS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  OU:=IncludeTrailingPathD','elimiter(GetUnitsOutputDir(Defaults.CPU,De'+
  'faults.OS));'#010+
  '  AddConditionalStrings(List,CleanFiles,ACPU,AOS);'#010+
>>>>>>> origin/fixes_2_2
  '  For I:=0 to FTargets.Count-1 do'#010+
  '    FTargets.TargetItems[I].GetCleanFiles(List, OU, OB, ACPU, AOS);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TPackage.GetInstallFiles(List: TStrings;Types : TTargetTypes'+
  ';ACPU:TCPU; AOS : TOS);'#010+
  'Var'#010+
  '  OB,O','U : String;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TPackage.GetIns','tallFiles(List: TStrings;Types : TTargetTyp'+
  'es;ACPU:TCPU; AOS : TOS);'#010+
  'Var'#010+
  '  OB,OU : String;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  I : Integer;'#010+
  '  T : TTarget;'#010+
  'begin'#010+
  '  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaul'+
  'ts.OS));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defa'+
  'ults.OS));'#010+
  '  AddConditionalStrings(List,Ins','tallFiles,ACPU,AOS);'#010+
=======
  '  OU:=IncludeTrailingPathDelimiter','(GetUnitsOutputDir(Defaults.CPU,De'+
  'faults.OS));'#010+
  '  AddConditionalStrings(List,InstallFiles,ACPU,AOS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  OU:=IncludeTrailingPathDelimiter','(GetUnitsOutputDir(Defaults.CPU,De'+
  'faults.OS));'#010+
  '  AddConditionalStrings(List,InstallFiles,ACPU,AOS);'#010+
>>>>>>> origin/fixes_2_2
  '  For I:=0 to FTargets.Count-1 do'#010+
  '    begin'#010+
  '      T:=FTargets.TargetItems[I];'#010+
  '      if (T.TargetType in Types) and (T.Install) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        T.GetInstallFiles(List, OU, OB, ACPU, AOS);'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.','GetInstallSourceFiles(List: TStrings; Types: TSou'+
  'rceTypes);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  S : TSource;'#010+
  'begin'#010+
  '  For I:=0 to FSources.Count-1 do'#010+
  '    begin'#010+
  '      S:=FSources.SourceItems[I];'#010+
  '      if (S.SourceType in Types) then'#010+
  '        S.GetInstallFile','s(List);'#010+
=======
  '        ','T.GetInstallFiles(List, OU, OB, ACPU, AOS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        ','T.GetInstallFiles(List, OU, OB, ACPU, AOS);'#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.GetArchiveFiles(List: TStrings; ACPU:TCPU; AOS : TO'+
  'S);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  'begin'#010+
  '  // Targets only'#010+
  '  For I:=0 to FTargets.Count-1 do'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FTargets.TargetItems[I].GetArchiveFiles(List,ACPU,AOS);'#010+
  'end;',#010+
=======
  '    FTargets.TargetItems[I].G','etArchiveFiles(List,ACPU,AOS);'#010+
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FTargets.TargetItems[I].G','etArchiveFiles(List,ACPU,AOS);'#010+
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'Function TPackage.GetDescription : string;'#010+
  'Var'#010+
  '  FN : String;'#010+
  '  L : TStringList;'#010+
  'begin'#010+
  '  If (FDescription<>'#039#039') then'#010+
  '    Result:=FDescription'#010+
  '  else'#010+
  '    If (FDescriptionFile<>'#039#039') then'#010+
  '      begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        // Always relative to binary name.',#010+
=======
  '       ',' // Always relative to binary name.'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '       ',' // Always relative to binary name.'#010+
>>>>>>> origin/fixes_2_2
  '        FN:=ExtractFilePath(ParamStr(0));'#010+
  '        FN:=FN+FDescriptionFile;'#010+
  '        If FileExists(FN) then'#010+
  '          begin'#010+
  '            L:=TStringList.Create;'#010+
  '            Try'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '              L.LoadFromFile(FN);'#010+
  '              Result:=L.Text;'#010+
  '  ','          Finally'#010+
=======
  '              L.LoadFromFile(FN',');'#010+
  '              Result:=L.Text;'#010+
  '            Finally'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '              L.LoadFromFile(FN',');'#010+
  '              Result:=L.Text;'#010+
  '            Finally'#010+
>>>>>>> origin/fixes_2_2
  '              L.Free;'#010+
  '            end;'#010+
  '          end;'#010+
  '      end;'#010+
  'end;'#010+
  #010+
  #010+
  'Function TPackage.GetVersion : string;'#010+
  'begin'#010+
  '  result:=FVersion.AsString;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TPackage.SetOptions(const AValue: TStrings);'#010+
  'begin'#010+
  '  If (AVa','lue=Nil) or (AValue.Count=0) then'#010+
  '    FreeAndNil(Foptions)'#010+
  '  else'#010+
  '    Options.Assign(AValue);'#010+
  'end;'#010+
  #010+
  #010+
  'Procedure TPackage.SetVersion(const V : string);'#010+
=======
  #010+
  'Procedure TPackage.SetVersion(cons','t V : string);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  #010+
  'Procedure TPackage.SetVersion(cons','t V : string);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FVersion.AsString:=V;'#010+
  'end;'#010+
  #010+
  #010+
  'Function TPackage.GetFileName : string;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If (F','FileName<>'#039#039') then'#010+
=======
  '  If (FFileName<>'#039#039') then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If (FFileName<>'#039#039') then'#010+
>>>>>>> origin/fixes_2_2
  '    Result:=FFileName'#010+
  '  else'#010+
  '    if not FVersion.Empty then'#010+
  '      Result := Name + '#039'-'#039' + FVersion.AsString'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    else'#010+
  '      Result := Name;'#010+
  'end;'#010+
  #010+
  'function TPackage.GetOptions: TStrings;'#010+
  'begin'#010+
  '  If (FOptions=Nil) then'#010+
  '    FO','ptions:=TStringList.Create;'#010+
  '  Result:=FOptions;'#010+
  'end;'#010+
  #010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    else'#010,
  '      Result := Name;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'Procedure TPackage.GetManifest(Manifest : TStrings);'#010+
  #010+
  '  procedure AddOSes(const AIndent:string;AOSes:TOSes);'#010+
  '  var'#010+
  '    IOS : TOS;'#010+
  '  begin'#010+
  '    if (AOSes=AllOSes) then'#010+
  '      exit;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Man','ifest.Add(AIndent+'#039'<oses>'#039');'#010+
=======
  '    Manifest.Add(AIndent+'#039'<oses>'#039,');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Manifest.Add(AIndent+'#039'<oses>'#039,');'#010+
>>>>>>> origin/fixes_2_2
  '    for IOS:=low(TOSes) to high(TOSes) do'#010+
  '      if IOS in AOSes then'#010+
  '        Manifest.Add(Format(AIndent+'#039' <os name="%s"/>'#039',[OSToSt'+
  'ring(IOS)]));'#010+
  '    Manifest.Add(AIndent+'#039'</oses>'#039');'#010+
  '  end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  procedure AddCPUs(c','onst AIndent:string;ACPUs:TCPUs);'#010+
=======
  '  procedure AddCPUs(const AIndent:string;ACPUs:','TCPUs);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  procedure AddCPUs(const AIndent:string;ACPUs:','TCPUs);'#010+
>>>>>>> origin/fixes_2_2
  '  var'#010+
  '    ICPU : TCPU;'#010+
  '  begin'#010+
  '    if (ACPUs=AllCPUs) then'#010+
  '      exit;'#010+
  '    Manifest.Add(AIndent+'#039'<cpus>'#039');'#010+
  '    for ICPU:=low(TCPUs) to high(TCPUs) do'#010+
  '      if ICPU in ACPUs then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        Manifest.Add(Format(','AIndent+'#039' <cpu name="%s"/>'#039',[CPU'+
  'ToString(ICPU)]));'#010+
=======
  '        Manifest.Add(Format(AIndent+'#039' <cpu name="%s"/>',#039',[CPUTo'+
  'String(ICPU)]));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        Manifest.Add(Format(AIndent+'#039' <cpu name="%s"/>',#039',[CPUTo'+
  'String(ICPU)]));'#010+
>>>>>>> origin/fixes_2_2
  '    Manifest.Add(AIndent+'#039'</cpus>'#039');'#010+
  '  end;'#010+
  #010+
  'Var'#010+
  '  S : String;'#010+
  '  i : Integer;'#010+
  '  D : TDependency;'#010+
  'begin'#010+
  '  With Manifest do'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Add(Format('#039'<package name="%s">'#039',[QuoteXml(Name)]));'#010,
  '    Add(Format('#039' <version major="%d" minor="%d" micro="%d" build="'+
  '%d"/>'#039',[FVersion.Major,FVersion.Minor,FVersion.Micro,FVersion.Buil'+
  'd]));'#010+
  '    AddOSes('#039' '#039',OSes);'#010+
  '    AddCPUs('#039' '#039',CPUs);'#010+
  '    Add(Format('#039' <filename>%s</filename>'#039',[QuoteXml(File','Name'+
  ' + ZipExt)]));'#010+
  '    Add(Format('#039' <author>%s</author>'#039',[QuoteXml(Author)]));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Add(Format('#039'<package name="%s">'#039',[QuoteXml(Name)]));'#010+
  '    Add(Format('#039' <version ','major="%d" minor="%d" micro="%d" build'+
  '="%d"/>'#039',[FVersion.Major,FVersion.Minor,FVersion.Micro,FVersion.Bu'+
  'ild]));'#010+
  '    AddOSes('#039' '#039',OSes);'#010+
  '    AddCPUs('#039' '#039',CPUs);'#010+
  '    Add(Format('#039' <filename>%s</filename>'#039',[QuoteXml(FileName +'+
  ' ZipExt)]));'#010+
  '    Add','(Format('#039' <author>%s</author>'#039',[QuoteXml(Author)]));'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Add(Format('#039' <license>%s</license>'#039',[QuoteXml(License)]));'+
  #010+
  '    if HomepageURL<>'#039#039' then'#010+
  '      Add(Format('#039' <homepageurl>%s</homepageurl>'#039',[QuoteXml(Ho'+
<<<<<<< HEAD
<<<<<<< HEAD
  'mepageUR','L)]));'#010+
  '    if DownloadURL<>'#039#039' then'#010+
=======
  'mepageURL)]));'#010+
  '    if DownloadURL<','>'#039#039' then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'mepageURL)]));'#010+
  '    if DownloadURL<','>'#039#039' then'#010+
>>>>>>> origin/fixes_2_2
  '      Add(Format('#039' <downloadurl>%s</downloadurl>'#039',[QuoteXml(Do'+
  'wnloadURL)]));'#010+
  '    Add(Format('#039' <email>%s</email>'#039',[QuoteXMl(Email)]));'#010+
  '    S:=Description;'#010+
  '    If (S<>'#039#039') then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Add(Format('#039' <description','>%s</description>'#039',[QuoteXML('+
=======
  '      Add(Format('#039' <description>%s</description>'#039',[QuoteX','ML('+
>>>>>>> graemeg/fixes_2_2
=======
  '      Add(Format('#039' <description>%s</description>'#039',[QuoteX','ML('+
>>>>>>> origin/fixes_2_2
  'S)]));'#010+
  '    If (Dependencies.Count>0) then'#010+
  '      begin'#010+
  '        Add('#039' <dependencies>'#039');'#010+
  '        for I:=0 to Dependencies.Count-1 do'#010+
  '          begin'#010+
  '            D:=Dependencies[i];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            Add('#039'  <dependency>'#039')',';'#010+
  '            Add(Format('#039'   <package packagename="%s"/>'#039',[Quote'+
  'XML(D.Value)]));'#010+
  '            if not D.FVersion.Empty then'#010+
  '              Add(Format('#039'   <version major="%d" minor="%d" micro='+
  '"%d" build="%d"/>'#039',[D.FVersion.Major,D.FVersion.Mino','r,D.FVersion'+
  '.Micro,D.FVersion.Build]));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '            Add('#039'  <dependency>'#039');'#010+
  '            Add(Format('#039,'   <package packagename="%s"/>'#039',[Quot'+
  'eXML(D.Value)]));'#010+
  '            if not D.FVersion.Empty then'#010+
  '              Add(Format('#039'   <version major="%d" minor="%d" micro='+
  '"%d" build="%d"/>'#039',[D.FVersion.Major,D.FVersion.Minor,D.FVersion.M'+
  'icro,D.FVers','ion.Build]));'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '            AddOSes('#039'   '#039',D.OSes);'#010+
  '            AddCPUs('#039'   '#039',D.CPUs);'#010+
  '            Add('#039'  </dependency>'#039');'#010+
  '          end;'#010+
  '        Add('#039' </dependencies>'#039');'#010+
  '      end;'#010+
  '    Add('#039'</package>'#039');'#010+
  '    end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'end',';'#010+
  #010+
  #010+
  'procedure TPackage.LoadUnitConfigFromFile(Const AFileName: String);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TPackage.Loa','dUnitConfigFromFile(Const AFileName: String);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'var'#010+
  '  L,L2 : TStrings;'#010+
  '  VOS : TOS;'#010+
  '  VCPU : TCPU;'#010+
  '  i,k : Integer;'#010+
  '  DepChecksum : Cardinal;'#010+
  '  DepName : String;'#010+
  '  D : TDependency;'#010+
  'begin'#010+
  '  L:=TStringList.Create;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Tr','y'#010+
  '    ReadIniFile(AFileName,L);'#010+
=======
  '  Try'#010+
  '    ReadIniFile(AFileNam','e,L);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Try'#010+
  '    ReadIniFile(AFileNam','e,L);'#010+
>>>>>>> origin/fixes_2_2
  '    With L do'#010+
  '      begin'#010+
  '        Version:=Values[KeyVersion];'#010+
  '        InstalledChecksum:=Cardinal(StrToInt64Def(Values[KeyChecksum],'+
  '$ffffffff));'#010+
  '        VCPU:=StringToCPU(Values[KeyCPU]);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        VOS:=String','ToOS(Values[KeyOS]);'#010+
  '        OSes:=[VOS];'#010+
=======
  '        VOS:=StringToOS(Values[KeyOS]);'#010+
  '     ','   OSes:=[VOS];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        VOS:=StringToOS(Values[KeyOS]);'#010+
  '     ','   OSes:=[VOS];'#010+
>>>>>>> origin/fixes_2_2
  '        CPUs:=[VCPU];'#010+
  '        L2:=TStringList.Create;'#010+
  '        L2.CommaText:=Values[KeyDepends];'#010+
  '        for i:=0 to L2.Count-1 do'#010+
  '          begin'#010+
  '            DepName:=L2[i];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            k:=Pos('#039'|'#039',D','epName);'#010+
  '            if k>0 then'#010+
=======
  '            k:=Pos('#039'|'#039',DepName);'#010+
  '            if k>','0 then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '            k:=Pos('#039'|'#039',DepName);'#010+
  '            if k>','0 then'#010+
>>>>>>> origin/fixes_2_2
  '              begin'#010+
  '                DepChecksum:=StrToInt(Copy(DepName,k+1,Length(DepName)'+
  '-k));'#010+
  '                DepName:=Copy(DepName,1,k-1);'#010+
  '              end'#010+
  '            else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '              DepChecksum:=$ff','ffffff;'#010+
  '            D:=Dependencies.Add(DepName,CPUs,OSes);'#010+
=======
  '              DepChecksum:=$ffffffff;'#010+
  '            D:=Dep','endencies.Add(DepName,CPUs,OSes);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '              DepChecksum:=$ffffffff;'#010+
  '            D:=Dep','endencies.Add(DepName,CPUs,OSes);'#010+
>>>>>>> origin/fixes_2_2
  '            D.RequireChecksum:=DepChecksum;'#010+
  '          end;'#010+
  '        FreeAndNil(L2);'#010+
  '        NeedLibC:=Upcase(Values[KeyNeedLibC])='#039'Y'#039';'#010+
  '      end;'#010+
  '  Finally'#010+
  '    L.Free;'#010+
  '  end;'#010+
  'end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  #010,
  #010+
  'procedure TPackage.SaveUnitConfigToFile(Const AFileName: String;ACPU:T'+
  'CPU;AOS:TOS);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'procedure TPackage.SaveUn','itConfigToFile(Const AFileName: String;ACPU'+
  ':TCPU;AOS:TOS);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  F : TFileStream;'#010+
  '  L : TStringList;'#010+
  '  Deps : String;'#010+
  '  i : integer;'#010+
  '  D : TDependency;'#010+
  '  p : TPackage;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  F:=TFileStream.Create(AFileName,fmCrea','te);'#010+
  '  L:=TStringList.Create;'#010+
=======
  '  F:=TFileStream.Create(AFileName,fmCreate);'#010+
  '  L:=TStringList.Crea','te;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  F:=TFileStream.Create(AFileName,fmCreate);'#010+
  '  L:=TStringList.Crea','te;'#010+
>>>>>>> origin/fixes_2_2
  '  try'#010+
  '    With L do'#010+
  '      begin'#010+
  '        Values[KeyName]:=Name;'#010+
  '        Values[KeyVersion]:=Version;'#010+
  '        // TODO Generate checksum based on PPUs'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        Values[KeyChecksum]:=IntToStr(DateTimeToFileDate(Now))',';'#010+
  '        Values[KeyCPU]:=CPUToString(ACPU);'#010+
=======
  '        Values[KeyChecksum]:=IntToStr(DateTimeToFileDate(Now));'#010+
  '        Values[KeyCPU]:=','CPUToString(ACPU);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        Values[KeyChecksum]:=IntToStr(DateTimeToFileDate(Now));'#010+
  '        Values[KeyCPU]:=','CPUToString(ACPU);'#010+
>>>>>>> origin/fixes_2_2
  '        Values[KeyOS]:=OSToString(AOS);'#010+
  '        Deps:='#039#039';'#010+
  '        for i:=0 to Dependencies.Count-1 do'#010+
  '          begin'#010+
  '            D:=Dependencies[i];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            if (ACPU in D.CPUs) and (AOS in D','.OSes) then'#010+
  '              begin'#010+
=======
  '            if (ACPU in D.CPUs) and (AOS in D.OSes) then'#010+
  '              ','begin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '            if (ACPU in D.CPUs) and (AOS in D.OSes) then'#010+
  '              ','begin'#010+
>>>>>>> origin/fixes_2_2
  '                if Deps<>'#039#039' then'#010+
  '                  Deps:=Deps+'#039','#039';'#010+
  '                Deps:=Deps+D.Value;'#010+
  '                P:=TPackage(D.Target);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                if assigned(P) and (P.InstalledChecksum<>$ffffffff',') '+
  'then'#010+
  '                  Deps:=Deps+'#039'|'#039'+IntToStr(P.InstalledChecksum)'+
  ';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '                if assigned(P) and (P.InstalledChecksum<>$ffffffff) th'+
  'en'#010+
  '                  D','eps:=Deps+'#039'|'#039'+IntToStr(P.InstalledChecksu'+
  'm);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '              end;'#010+
  '          end;'#010+
  '        Values[KeyDepends]:=Deps;'#010+
  '        if NeedLibC then'#010+
  '          Values[KeyNeedLibC]:='#039'Y'#039#010+
  '        else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          Values[KeyNeedLibC',']:='#039'N'#039';'#010+
  '      end;'#010+
  '    L.SaveToStream(F);'#010+
=======
  '          Values[KeyNeedLibC]:='#039'N'#039';'#010+
  '      end;'#010+
  '    L.S','aveToStream(F);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '          Values[KeyNeedLibC]:='#039'N'#039';'#010+
  '      end;'#010+
  '    L.S','aveToStream(F);'#010+
>>>>>>> origin/fixes_2_2
  '  Finally'#010+
  '    L.Free;'#010+
  '    F.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                              TPackages'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '*******************************','*************************************'+
=======
  '*********************************************************','***********'+
>>>>>>> graemeg/fixes_2_2
=======
  '*********************************************************','***********'+
>>>>>>> origin/fixes_2_2
  '********}'#010+
  #010+
  'function TPackages.GetPackage(const AName : String): TPackage;'#010+
  'begin'#010+
  '  Result:=TPackage(ItemByName(AName))'#010+
  'end;'#010+
  #010+
  'function TPackages.GetPackageItem(AIndex : Integer): TPackage;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=','TPackage(Items[AIndex]);'#010+
  'end;'#010+
=======
  '  Result:=TPackage(Items[AIndex]);'#010+
  'e','nd;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result:=TPackage(Items[AIndex]);'#010+
  'e','nd;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'procedure TPackages.SetPackageItem(AIndex : Integer; const AValue: TPa'+
  'ckage);'#010+
  'begin'#010+
  '  Items[AIndex]:=AValue;'#010+
  'end;'#010+
  #010+
  #010+
  'function TPackages.AddPackage(const AName: String): TPackage;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=Add as TPackag','e;'#010+
  '  Result.Name:=AName;'#010+
  'end;'#010+
=======
  '  Result:=Add as TPackage;'#010+
  '  Result.Name:=AName;'#010+
  'e','nd;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result:=Add as TPackage;'#010+
  '  Result.Name:=AName;'#010+
  'e','nd;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                             TCustomDefaults'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'proced','ure TCustomDefaults.SetCPU(const AValue: TCPU);'#010+
=======
  'procedure TCustomDefaults.SetCPU','(const AValue: TCPU);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TCustomDefaults.SetCPU','(const AValue: TCPU);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FCPU:=AValue;'#010+
  '  RecalcTarget;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCustomDefaults.SetOptions(const AValue: TStrings);'#010+
  'begin'#010+
  '  If (AValue=Nil) or (AValue.Count=0) then'#010+
  '    FreeAndNil(Foptions)'#010+
  '  else'#010+
  '    O','ptions.Assign(AValue)'#010+
  'end;'#010+
  #010+
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'function TCustomDefaults.GetBaseInstallDir: String;'#010+
  'begin'#010+
  '  If (FBaseInstallDir<>'#039#039') then'#010+
  '    Result:=FBaseInstallDir'#010+
  '  else'#010+
  '    if UnixPaths then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Result:=Prefix +'#039'lib'#039' + PathDelim + '#039'fpc'#039' + Path'+
  'Delim + FC','ompilerVersion + PathDelim'#010+
=======
  '      Result:=Prefix +'#039'lib',#039' + PathDelim + '#039'fpc'#039' + Path'+
  'Delim'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Result:=Prefix +'#039'lib',#039' + PathDelim + '#039'fpc'#039' + Path'+
  'Delim'#010+
>>>>>>> origin/fixes_2_2
  '    else'#010+
  '      Result:=Prefix;'#010+
  'end;'#010+
  #010+
  #010+
  'function TCustomDefaults.GetBinInstallDir: String;'#010+
  'begin'#010+
  '  If (FBinInstallDir<>'#039#039') then'#010+
  '    Result:=FBinInstallDir'#010+
  '  else'#010+
  '    If UnixPaths then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Result:=Prefix+'#039'bin'#039#010+
  '    ','else'#010+
=======
  '      Result:=BaseInstal','lDir+'#039'bin'#039#010+
  '    else'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Result:=BaseInstal','lDir+'#039'bin'#039#010+
  '    else'#010+
>>>>>>> origin/fixes_2_2
  '      Result:=BaseInstallDir+'#039'bin'#039';'#010+
  'end;'#010+
  #010+
  #010+
  'function TCustomDefaults.GetCompiler: String;'#010+
  'begin'#010+
  '  If (FCompiler<>'#039#039') then'#010+
  '    Result:=FCompiler'#010+
  '  else'#010+
  '    Result:='#039'fpc'#039';'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TCustomDefaults.GetDocInstallDir: String;'#010+
  'begin'#010+
  '  ','If (FDocInstallDir<>'#039#039') then'#010+
  '    Result:=FDocInstallDir'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'function TCustomDefaults.GetDocInstallDir: S','tring;'#010+
  'begin'#010+
  '  If (FBinInstallDir<>'#039#039') then'#010+
  '    Result:=FBinInstallDir'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  else'#010+
  '    If UnixPaths then'#010+
  '      Result:=Prefix+'#039'share'#039'+PathDelim+'#039'doc'#039#010+
  '    else'#010+
  '      Result:=BaseInstallDir+'#039'docs'#039';'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TCustomDefaults.GetExamplesInstallDir: String;'#010+
  'b','egin'#010+
=======
  'function TCustomDefaults.GetExamplesInstal','lDir: String;'#010+
  'begin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TCustomDefaults.GetExamplesInstal','lDir: String;'#010+
  'begin'#010+
>>>>>>> origin/fixes_2_2
  '  If (FExamplesInstallDir<>'#039#039') then'#010+
  '    Result:=FExamplesInstallDir'#010+
  '  else'#010+
  '    If UnixPaths then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Result:=Prefix+'#039'share'#039'+PathDelim+'#039'doc'#039#010+
  '    else'#010+
  '      Result:=BaseInstallDir+'#039'examples'#039';'#010+
  'end;'#010+
  #010+
  'function TCustomDefaults.GetOptions: ','TStrings;'#010+
  'begin'#010+
  '  If (FOptions=Nil) then'#010+
  '    FOptions:=TStringList.Create;'#010+
  '  Result:=FOptions;'#010+
  'end;'#010+
  #010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      Result:=Prefix+'#039'share'#039'+PathDelim+'#039'docs'#039'+PathDeli'+
  'm+'#039'examples'#039#010+
  '    else'#010+
  '      Result:=BaseInstallDir+'#039'examples'#039';'#010+
  'end;'#010+
  #010,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'function TCustomDefaults.GetUnitInstallDir: String;'#010+
  'begin'#010+
  '  If (FUnitInstallDir<>'#039#039') then'#010+
  '    Result:=FUnitInstallDir'#010+
  '  else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Result:=B','aseInstallDir+'#039'units'#039'+PathDelim+Target;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    If UnixPaths then'#010+
  '      Result:=BaseInstallDir+'#039'units'#039'+PathDelim+Target'#010+
  '    else'#010+
  '      Result:=BaseInstallDir+',#039'units'#039'+PathDelim+Target;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'function TCustomDefaults.GetLocalUnitDir: String;'#010+
  'begin'#010+
  '  Result:=FLocalUnitDir;'#010+
  'end;'#010+
  #010+
  #010+
  'function TCustomDefaults.GetGlobalUnitDir: String;'#010+
  'begin'#010+
  '  If (FGlobalUnitDir<>'#039#039') then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Result:=FGlob','alUnitDir'#010+
  '  else'#010+
=======
  '    Result:=FGlobalUnitDir'#010+
  '  el','se'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Result:=FGlobalUnitDir'#010+
  '  el','se'#010+
>>>>>>> origin/fixes_2_2
  '    Result:=UnitInstallDir;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.SetLocalUnitDir(const AValue: String);'#010+
  'begin'#010+
  '  // Use ExpandFileName to support ~/ expansion'#010+
  '  if AValue<>'#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FLocalUnitDir:=IncludeTrailingPathDelimiter(','ExpandFileName(AValu'+
=======
  '    FLocalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName','(AValu'+
>>>>>>> graemeg/fixes_2_2
=======
  '    FLocalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName','(AValu'+
>>>>>>> origin/fixes_2_2
  'e))'#010+
  '  else'#010+
  '    FLocalUnitDir:='#039#039';'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.SetGlobalUnitDir(const AValue: String);'#010+
  'begin'#010+
  '  // Use ExpandFileName to support ~/ expansion'#010+
  '  if AValue<>'#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FGlobalUnitDir:=IncludeTrailingPat','hDelimiter(ExpandFileName(AVal'+
=======
  '    FGlobalUnitDir:=IncludeTrailingPathDelimiter(Exp','andFileName(AVal'+
>>>>>>> graemeg/fixes_2_2
=======
  '    FGlobalUnitDir:=IncludeTrailingPathDelimiter(Exp','andFileName(AVal'+
>>>>>>> origin/fixes_2_2
  'ue))'#010+
  '  else'#010+
  '    FGlobalUnitDir:='#039#039';'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.SetBaseInstallDir(const AValue: String);'#010+
  'begin'#010+
  '  // Use ExpandFileName to support ~/ expansion'#010+
  '  if AValue<>'#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FBaseInstallDir:=Incl','udeTrailingPathDelimiter(ExpandFileName(AVa'+
=======
  '    FBaseInstallDir:=IncludeTrailingPat','hDelimiter(ExpandFileName(AVa'+
>>>>>>> graemeg/fixes_2_2
=======
  '    FBaseInstallDir:=IncludeTrailingPat','hDelimiter(ExpandFileName(AVa'+
>>>>>>> origin/fixes_2_2
  'lue))'#010+
  '  else'#010+
  '    FBaseInstallDir:='#039#039';'#010+
  '  UnitInstallDir:='#039#039';'#010+
  '  BinInstallDir:='#039#039';'#010+
  '  ExamplesInstallDir:='#039#039';'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.SetOS(const AValue: TOS);'#010+
  'begin'#010+
  '  FOS:=AValue;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Recalcta','rget;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.SetPrefix(const AValue: String);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Recalctarget;'#010+
  'end;'#010+
  #010+
  #010+
  'p','rocedure TCustomDefaults.SetPrefix(const AValue: String);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  if FPrefix=AValue then exit;'#010+
  '  FPrefix:=IncludeTrailingPathDelimiter(AValue);'#010+
  '  BaseInstallDir:='#039#039';'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCustomDefaults.SetTarget(const AValue: Str','ing);'#010+
  'Var'#010+
  '  P : Integer;'#010+
=======
  'procedure TCustomDefaults.SetTarget(const AValue: String);'#010+
  'Var'#010+
  '  P ',': Integer;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TCustomDefaults.SetTarget(const AValue: String);'#010+
  'Var'#010+
  '  P ',': Integer;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  if FTarget<>AValue then'#010+
  '    begin'#010+
  '      P:=Pos('#039'-'#039',AValue);'#010+
  '      If (P<>0) then'#010+
  '        begin'#010+
  '          FOS:=StringToOS(System.Copy(Avalue,P+1,Length(AValue)-P));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          FCPU:=StringToCPU(System.Copy(Ava','lue,1,P-1));'#010+
  '        end'#010+
=======
  '          FCPU:=StringToCPU(System.Copy(Avalue,1,P-1));'#010+
  ' ','       end'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '          FCPU:=StringToCPU(System.Copy(Avalue,1,P-1));'#010+
  ' ','       end'#010+
>>>>>>> origin/fixes_2_2
  '      else'#010+
  '        FOS:=StringToOS(AValue);'#010+
  '      FTarget:=AValue;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.RecalcTarget;'#010+
  'begin'#010+
  '  Ftarget:=CPUToString(FCPU)+'#039'-'#039'+OStoString(FOS);'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TCustomDefaults.CmdLi','neOptions: String;'#010+
  'begin'#010+
  '  If Haveoptions then'#010+
  '    Result:=OptionListToString(FOptions);'#010+
  'end;'#010+
  #010+
  #010+
  'constructor TCustomDefaults.Create;'#010+
  'begin'#010+
  '  InitDefaults;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  #010+
  'constructor TCustomDefaults.Create;'#010+
  'begin'#010+
  ' ',' InitDefaults;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.InitDefaults;'#010+
  'begin'#010+
  '{$ifdef unix}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  UnixPaths:=True;'#010,
=======
  '  UnixPaths:=True;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  UnixPaths:=True;'#010+
>>>>>>> origin/fixes_2_2
  '{$else}'#010+
  '  UnixPaths:=False;'#010+
  '{$endif}'#010+
  '  FNoFPCCfg:=False;'#010+
  '  FCPU:=cpuNone;'#010+
  '  FOS:=osNone;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TCustomDefaults.HaveOptions: Boolean;'#010+
  'begin'#010+
  '  Result:=Assigned(FOptions);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.LocalInit(Const AFileName : S','tring);'#010+
=======
  #010+
  'procedure TCustomDefaults.LocalInit(Const A','FileName : String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  #010+
  'procedure TCustomDefaults.LocalInit(Const A','FileName : String);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  FN : String;'#010+
  'begin'#010+
  '  FN:=AFileName;'#010+
  '  If (FN='#039#039') then'#010+
  '    begin'#010+
  '    // Environment variable.'#010+
  '    FN:=GetEnvironmentVariable('#039'FPMAKECFG'#039');'#010+
  '    If (FN<>'#039#039') then'#010+
  '      If not FileExists(FN) then'#010+
  '        FN:='#039#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    // User config',' file fpmake.cfg'#010+
=======
  '    //',' User config file fpmake.cfg'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    //',' User config file fpmake.cfg'#010+
>>>>>>> origin/fixes_2_2
  '    If (FN='#039#039') then'#010+
  '      begin'#010+
  '      FN:=GetAppConfigFile(False);'#010+
  '      If Not FileExists(FN) then'#010+
  '        FN:='#039#039';'#010+
  '      end;'#010+
  '    // Global config file fpmake.cfg'#010+
  '    If (FN='#039#039') then'#010+
  '      begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      FN:=GetAppConfigFile(','True);'#010+
=======
  '      FN:=GetAp','pConfigFile(True);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      FN:=GetAp','pConfigFile(True);'#010+
>>>>>>> origin/fixes_2_2
  '      If Not FileExists(FN) then'#010+
  '        FN:='#039#039';'#010+
  '      end;'#010+
  '    end;'#010+
  '  If (FN<>'#039#039') and FileExists(FN) then'#010+
  '    LoadFromFile(FN);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.CompilerDefaults;'#010+
  '{$ifdef HAS_UNIT_PROCESS}'#010+
  'var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  infoSL : TStringList;',#010+
=======
  '  infoSL : ','TStringList;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  infoSL : ','TStringList;'#010+
>>>>>>> origin/fixes_2_2
  '{$endif HAS_UNIT_PROCESS}'#010+
  'begin'#010+
  '  if (CPU=cpuNone) or (OS=osNone) or (FCompilerVersion='#039#039') then'#010+
  '    begin'#010+
  '{$ifdef HAS_UNIT_PROCESS}'#010+
  '      // Detect compiler version/target from -i option'#010+
  '      infosl:=TStringList.Create;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      infosl.Delim','iter:='#039' '#039';'#010+
=======
  '      ','infosl.Delimiter:='#039' '#039';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      ','infosl.Delimiter:='#039' '#039';'#010+
>>>>>>> origin/fixes_2_2
  '      infosl.DelimitedText:=GetCompilerInfo(GetCompiler,'#039'-iVTPTO'#039+
  ');'#010+
  '      if infosl.Count<>3 then'#010+
  '        Raise EInstallerError.Create(SErrInvalidFPCInfo);'#010+
  '      if FCompilerVersion='#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        FCompilerVersion:=infosl[0];'#010+
  '  ','    if CPU=cpuNone then'#010+
=======
  '        FCompilerVersion:=i','nfosl[0];'#010+
  '      if CPU=cpuNone then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        FCompilerVersion:=i','nfosl[0];'#010+
  '      if CPU=cpuNone then'#010+
>>>>>>> origin/fixes_2_2
  '        CPU:=StringToCPU(infosl[1]);'#010+
  '      if OS=osNone then'#010+
  '        OS:=StringToOS(infosl[2]);'#010+
  '{$else HAS_UNIT_PROCESS}'#010+
  '      // Defaults taken from compiler used to build fpmake'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      if CPU=cpuNone then'#010+
  '        CP','U:=StringToCPU({$I %FPCTARGETCPU%});'#010+
=======
  '      if CPU=cpuNone the','n'#010+
  '        CPU:=StringToCPU({$I %FPCTARGETCPU%});'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      if CPU=cpuNone the','n'#010+
  '        CPU:=StringToCPU({$I %FPCTARGETCPU%});'#010+
>>>>>>> origin/fixes_2_2
  '      if OS=osNone then'#010+
  '        OS:=StringToOS({$I %FPCTARGETOS%});'#010+
  '      if FCompilerVersion='#039#039' then'#010+
  '        FCompilerVersion:={$I %FPCVERSION%};'#010+
  '{$endif HAS_UNIT_PROCESS}'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCus','tomDefaults.LoadFromFile(Const AFileName: String);'#010+
=======
  'pr','ocedure TCustomDefaults.LoadFromFile(Const AFileName: String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'pr','ocedure TCustomDefaults.LoadFromFile(Const AFileName: String);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  F : TFileStream;'#010+
  'begin'#010+
  '  F:=TFileStream.Create(AFileName,fmOpenRead);'#010+
  '  Try'#010+
  '    LoadFromStream(F);'#010+
  '  Finally'#010+
  '    F.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCustomDefaults.SaveToFile(Const AFile','Name: String);'#010+
=======
  'procedure TCustomDefaults.SaveToFile','(Const AFileName: String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TCustomDefaults.SaveToFile','(Const AFileName: String);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  F : TFileStream;'#010+
  'begin'#010+
  '  F:=TFileStream.Create(AFileName,fmCreate);'#010+
  '  Try'#010+
  '    SaveToStream(F);'#010+
  '  Finally'#010+
  '    F.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomDefaults.SaveToStream(S : TStream);'#010+
  'Var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  L : TStringList;'#010+
  'begin'#010+
  '  L:=T','StringList.Create;'#010+
=======
  '  L : TStringList;'#010,
  'begin'#010+
  '  L:=TStringList.Create;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  L : TStringList;'#010,
  'begin'#010+
  '  L:=TStringList.Create;'#010+
>>>>>>> origin/fixes_2_2
  '  try'#010+
  '    With L do'#010+
  '      begin'#010+
  '      Values[KeyArchive]:=FArchive;'#010+
  '      Values[KeyCompiler]:=FCompiler;'#010+
  '      Values[KeyCopy]:=FCopy;'#010+
  '      Values[KeyMkDir]:=FMkDir;'#010+
  '      Values[KeyMove]:=FMove;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Values[KeyOptions',']:=CmdLineOptions;'#010+
=======
  '      Value','s[KeyOptions]:=FOptions;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Value','s[KeyOptions]:=FOptions;'#010+
>>>>>>> origin/fixes_2_2
  '      Values[KeyCPU]:=CPUToString(FCPU);'#010+
  '      Values[KeyOS]:=OSToString(FOS);'#010+
  '      Values[KeyMode]:=ModeToString(FMode);'#010+
  '      Values[KeyLocalUnitDir]:=FLocalUnitDir;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Values[KeyGlobalUnitDir]:=FGlobalUnitDir;'#010+
  '    ','  Values[KeyPrefix]:=FPrefix;'#010+
=======
  '      Values[KeyGlobalUnitDir]:=FGlobalUnitDir',';'#010+
  '      Values[KeyPrefix]:=FPrefix;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Values[KeyGlobalUnitDir]:=FGlobalUnitDir',';'#010+
  '      Values[KeyPrefix]:=FPrefix;'#010+
>>>>>>> origin/fixes_2_2
  '      Values[KeyBaseInstallDir]:=FBaseInstallDir;'#010+
  '      Values[KeyUnitInstallDir]:=FUnitInstallDir;'#010+
  '      Values[KeyBinInstallDir]:=FBinInstallDir;'#010+
  '      Values[KeyDocInstallDir]:=FDocInstallDir;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Values[K','eyExamplesInstallDir]:=FExamplesInstallDir;'#010+
=======
  '      Va','lues[KeyExamplesInstallDir]:=FExamplesInstallDir;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Va','lues[KeyExamplesInstallDir]:=FExamplesInstallDir;'#010+
>>>>>>> origin/fixes_2_2
  '      Values[KeyRemove]:=FRemove;'#010+
  '      Values[KeyTarget]:=FTarget;'#010+
  '      if FNoFPCCfg then'#010+
  '        Values[KeyNoFPCCfg]:='#039'Y'#039';'#010+
  '      end;'#010+
  '    L.SaveToStream(S);'#010+
  '  Finally'#010+
  '    L.Free;'#010+
  '  end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'end;'#010+
  #010+
  #010,
=======
  'e','nd;'#010+
  #010+
  #010+
>>>>>>> graemeg/fixes_2_2
=======
  'e','nd;'#010+
  #010+
  #010+
>>>>>>> origin/fixes_2_2
  'procedure TCustomDefaults.LoadFromStream(S: TStream);'#010+
  'Var'#010+
  '  L : TStrings;'#010+
  '  Line : String;'#010+
  '  I,P,PC : Integer;'#010+
  'begin'#010+
  '  L:=TStringList.Create;'#010+
  '  Try'#010+
  '    L.LoadFromStream(S);'#010+
  '    // Fix lines.'#010+
  '    For I:=L.Count-1 downto 0 do'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      begin'#010+
  '    ','  Line:=L[I];'#010+
=======
  '      begi','n'#010+
  '      Line:=L[I];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      begi','n'#010+
  '      Line:=L[I];'#010+
>>>>>>> origin/fixes_2_2
  '      P:=Pos('#039'='#039',Line);'#010+
  '      PC:=Pos('#039';'#039',Line);  // Comment line.'#010+
  '      If (P=0) or ((PC<>0) and (PC<P)) then'#010+
  '        L.Delete(I)'#010+
  '      else'#010+
  '        L[i]:=Trim(System.Copy(Line,1,P-1)+'#039'='#039'+Trim(System.Cop'+
<<<<<<< HEAD
<<<<<<< HEAD
  'y(Line,P+1,Length(Line',')-P)));'#010+
=======
  'y(Line,P+1,Lengt','h(Line)-P)));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'y(Line,P+1,Lengt','h(Line)-P)));'#010+
>>>>>>> origin/fixes_2_2
  '      end;'#010+
  '    With L do'#010+
  '      begin'#010+
  '      FArchive:=Values[KeyArchive];'#010+
  '      FCompiler:=Values[KeyCompiler];'#010+
  '      FCopy:=Values[KeyCopy];'#010+
  '      FMkDir:=Values[KeyMkDir];'#010+
  '      FMove:=Values[KeyMove];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      FRemove:=Values[KeyRemo','ve];'#010+
  '      Options:=OptionsToStringList(Values[KeyOptions]);'#010+
=======
  '      FRemove:=Values[K','eyRemove];'#010+
  '      FOptions:=Values[KeyOptions];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      FRemove:=Values[K','eyRemove];'#010+
  '      FOptions:=Values[KeyOptions];'#010+
>>>>>>> origin/fixes_2_2
  '      Line:=Values[KeyCPU];'#010+
  '      If (Line<>'#039#039') then'#010+
  '        FCPU:=StringToCPU(Line);'#010+
  '      Line:=Values[KeyOS];'#010+
  '      If (Line<>'#039#039') then'#010+
  '        FOS:=StringToOS(Line);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Line',':=Values[KeyMode];'#010+
=======
  '      Line:=Values[KeyMo','de];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Line:=Values[KeyMo','de];'#010+
>>>>>>> origin/fixes_2_2
  '      If (Line<>'#039#039') then'#010+
  '        FMode:=StringToMode(Line);'#010+
  '      FTarget:=Values[KeyTarget];'#010+
  '      FLocalUnitDir:=Values[KeyLocalUnitDir];'#010+
  '      FGlobalUnitDir:=Values[KeyGlobalUnitDir];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      FPrefix:=Values[KeyPrefix];',#010+
  '      FBaseInstallDir:=Values[KeyBaseInstallDir];'#010+
  '      FUnitInstallDir:=Values[KeyUnitInstallDir];'#010+
  '      FBinInstallDir:=Values[KeyBinInstallDir];'#010+
  '      FDocInstallDir:=Values[KeyDocInstallDir];'#010+
  '      FExamplesInstallDir:=Values[KeyExampl','esInstallDir];'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      FPrefix:=Values[KeyPrefix];'#010+
  '      FBaseIn','stallDir:=Values[KeyBaseInstallDir];'#010+
  '      FUnitInstallDir:=Values[KeyUnitInstallDir];'#010+
  '      FBinInstallDir:=Values[KeyBinInstallDir];'#010+
  '      FDocInstallDir:=Values[KeyDocInstallDir];'#010+
  '      FExamplesInstallDir:=Values[KeyExamplesInstallDir];',#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      FNoFPCCfg:=(Upcase(Values[KeyNoFPCCfg])='#039'Y'#039');'#010+
  '      end;'#010+
  '  Finally'#010+
  '    L.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                             TFPCDefaults'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '******','**************************************************************'+
=======
  '********************','************************************************'+
>>>>>>> graemeg/fixes_2_2
=======
  '********************','************************************************'+
>>>>>>> origin/fixes_2_2
  '********}'#010+
  #010+
  'procedure TFPCDefaults.CompilerDefaults;'#010+
  'var'#010+
  '  BD : String;'#010+
  'begin'#010+
  '  inherited CompilerDefaults;'#010+
  #010+
  '  // Use the same algorithm as the compiler, see options.pas'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$ifdef U','nix}'#010+
  '  BD:=FixPath(GetEnvironmentVariable('#039'FPCDIR'#039'));'#010+
=======
  '{$ifdef Unix}'#010+
  '  BD:=Fix','Path(GetEnvironmentVariable('#039'FPCDIR'#039'));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '{$ifdef Unix}'#010+
  '  BD:=Fix','Path(GetEnvironmentVariable('#039'FPCDIR'#039'));'#010+
>>>>>>> origin/fixes_2_2
  '  if BD='#039#039' then'#010+
  '    begin'#010+
  '      BD:='#039'/usr/local/lib/fpc/'#039'+FCompilerVersion;'#010+
  '      if not DirectoryExists(BD) and'#010+
  '         DirectoryExists('#039'/usr/lib/fpc/'#039'+FCompilerVersion) the'+
  'n'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        B','D:='#039'/usr/lib/fpc/'#039'+FCompilerVersion;'#010+
=======
  '        BD:='#039'/usr/lib/f','pc/'#039'+FCompilerVersion;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        BD:='#039'/usr/lib/f','pc/'#039'+FCompilerVersion;'#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  '{$else unix}'#010+
  '  BD:=FixPath(GetEnvironmentVariable('#039'FPCDIR'#039'));'#010+
  '  if BD='#039#039' then'#010+
  '    begin'#010+
  '      BD:=ExtractFilePath(FCompiler)+'#039'..'#039';'#010+
  '      if not(DirectoryExists(BD+'#039'/units'#039')) and'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '         not(Dir','ectoryExists(BD+'#039'/rtl'#039')) then'#010+
=======
  '         not(DirectoryExists(B','D+'#039'/rtl'#039')) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '         not(DirectoryExists(B','D+'#039'/rtl'#039')) then'#010+
>>>>>>> origin/fixes_2_2
  '        BD:=FBaseInstallDir+'#039'..'#039';'#010+
  '    end;'#010+
  '{$endif unix}'#010+
  #010+
  '  // Where to install by default'#010+
  '  if (FBaseInstallDir='#039#039') and (FPrefix='#039#039') then'#010+
  '    BaseInstallDir:=BD;'#010+
  #010+
  '  // Where to find the units by default'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if (','FGlobalUnitDir='#039#039') then'#010+
=======
  '  if (FGlobalUnitDir','='#039#039') then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  if (FGlobalUnitDir','='#039#039') then'#010+
>>>>>>> origin/fixes_2_2
  '    GlobalUnitDir:=IncludeTrailingPathDelimiter(BD)+'#039'units'#039'+Pa'+
  'thDelim+Target;'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                            TCustomInstaller'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '********','************************************************************'+
=======
  '**********************','**********************************************'+
>>>>>>> graemeg/fixes_2_2
=======
  '**********************','**********************************************'+
>>>>>>> origin/fixes_2_2
  '********}'#010+
  #010+
  'constructor TCustomInstaller.Create(AOwner: TComponent);'#010+
  'begin'#010+
  '  Dictionary:=DictionaryClass.Create(Nil);'#010+
  '  AnalyzeOptions;'#010+
  '  CreatePackages;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'destructor TCustomIn','staller.Destroy;'#010+
=======
  'destructor TCustomInstaller.Destro','y;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'destructor TCustomInstaller.Destro','y;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FreeAndNil(Defaults);'#010+
  '  FreeAndNil(Dictionary);'#010+
  '  inherited destroy;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Log(Level: TVerboseLevel; const Msg: String'+
  ');'#010+
  'begin'#010+
  '  If Level in FLogLevels then'#010+
  '    Writeln(StdOut,Msg);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'end;'#010,
  #010+
  #010+
  'procedure TCustomInstaller.CreatePackages;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TC','ustomInstaller.CreatePackages;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FPAckages:=TPackages.Create(TPackage);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.CreateBuildEngine;'#010+
  'begin'#010+
  '  FBuildEngine:=TBuildEngine.Create(Self);'#010+
  '//  FBuildEngine.Defaults:=Defaults;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FBuildE','ngine.ListMode:=FListMode;'#010+
=======
  '  FBuildEngine.ListMode',':=FListMode;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  FBuildEngine.ListMode',':=FListMode;'#010+
>>>>>>> origin/fixes_2_2
  '  FBuildEngine.OnLog:=@Self.Log;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Error(const Msg: String);'#010+
  'begin'#010+
  '  Raise EInstallerError.Create(Msg);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCustomInstaller.Error(const Fmt: String; Args: array of con',
  'st);'#010+
  'begin'#010+
  '  Raise EInstallerError.CreateFmt(Fmt,Args);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TCustomInstaller.Error(const Fmt: String; Args: array of con'+
  'st);'#010+
  'begin'#010+
  '  R','aise EInstallerError.CreateFmt(Fmt,Args);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TCustomInstaller.AddPackage(const AName: String) : TPackage;'#010+
  'begin'#010+
  '  result:=FPackages.AddPackage(AName);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.AnalyzeOptions;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Function ','CheckOption(Index : Integer;const Short,Long : String): B'+
=======
  '  Function CheckOption(In','dex : Integer;const Short,Long : String): B'+
>>>>>>> graemeg/fixes_2_2
=======
  '  Function CheckOption(In','dex : Integer;const Short,Long : String): B'+
>>>>>>> origin/fixes_2_2
  'oolean;'#010+
  '  var'#010+
  '    O : String;'#010+
  '  begin'#010+
  '    O:=Paramstr(Index);'#010+
  '    Result:=(O='#039'-'#039'+short) or (O='#039'--'#039'+long) or (copy(O,'+
  '1,Length(Long)+3)=('#039'--'#039'+long+'#039'='#039'));'#010+
  '  end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Function CheckCommand','(Index : Integer;const Short,Long : String): '+
=======
  '  Function CheckCommand(Index : Integ','er;const Short,Long : String): '+
>>>>>>> graemeg/fixes_2_2
=======
  '  Function CheckCommand(Index : Integ','er;const Short,Long : String): '+
>>>>>>> origin/fixes_2_2
  'Boolean;'#010+
  '  var'#010+
  '    O : String;'#010+
  '  begin'#010+
  '    O:=Paramstr(Index);'#010+
  '    Result:=(O='#039'-'#039'+short) or (O=long);'#010+
  '  end;'#010+
  #010+
  '  Function OptionArg(Var Index : Integer) : String;'#010+
  '  Var'#010+
  '    P : Integer;'#010+
  '  begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ','  if (Length(ParamStr(Index))>1) and (Paramstr(Index)[2]<>'#039'-'#039+
=======
  '    if (Length(P','aramStr(Index))>1) and (Paramstr(Index)[2]<>'#039'-'#039+
>>>>>>> graemeg/fixes_2_2
=======
  '    if (Length(P','aramStr(Index))>1) and (Paramstr(Index)[2]<>'#039'-'#039+
>>>>>>> origin/fixes_2_2
  ') then'#010+
  '      begin'#010+
  '      If Index<ParamCount then'#010+
  '        begin'#010+
  '        Inc(Index);'#010+
  '        Result:=Paramstr(Index);'#010+
  '        end'#010+
  '      else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        Error(SErrNeedArgument,[Index,P','aramStr(Index)]);'#010+
=======
  '        Error(SErrNeedArgument,[Index,ParamStr(Index)',']);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        Error(SErrNeedArgument,[Index,ParamStr(Index)',']);'#010+
>>>>>>> origin/fixes_2_2
  '      end'#010+
  '    else If length(ParamStr(Index))>2 then'#010+
  '      begin'#010+
  '      P:=Pos('#039'='#039',Paramstr(Index));'#010+
  '      If (P=0) then'#010+
  '        Error(SErrNeedArgument,[Index,ParamStr(Index)])'#010+
  '      else'#010+
  '        begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        Result:=Param','str(Index);'#010+
  '        Delete(Result,1,P);'#010+
=======
  '        Result:=Paramstr(Index);'#010+
  '  ','      Delete(Result,1,P);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        Result:=Paramstr(Index);'#010+
  '  ','      Delete(Result,1,P);'#010+
>>>>>>> origin/fixes_2_2
  '        end;'#010+
  '      end;'#010+
  '  end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  function SplitSpaces(var SplitString: string) : string;'#010+
  '  var i : integer;'#010+
  '  begin'#010+
  '    i := pos('#039' '#039',SplitString);'#010+
  '    if i > 0 then'#010+
  '      begin'#010+
  '        result := copy','(SplitString,1,i-1);'#010+
  '        delete(SplitString,1,i);'#010+
  '      end'#010+
  '    else'#010+
  '      begin'#010+
  '        result := SplitString;'#010+
  '        SplitString:='#039#039';'#010+
  '      end;'#010+
  '  end;'#010+
  #010+
  'Var'#010+
  '  I : Integer;'#010+
  '  DefaultsFileName : string;'#010+
  '  OptString : string;'#010+
  'begin'#010+
  '  I:','=0;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  '  DefaultsFileName : string;'#010+
  'begin'#010+
  '  I:=0;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  FListMode:=False;'#010+
  '  FLogLevels:=DefaultMessages;'#010+
  '  While (I<ParamCount) do'#010+
  '    begin'#010+
  '    Inc(I);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    if CheckOption(I,'#039'v'#039','#039'verbose'#039') then'#010+
  '      FLogLevels:=AllMessages'#010+
  '    else if CheckOption(I,'#039'd'#039','#039'debug'#039') then'#010+
  '      FLogLevels:=AllM','essages+[vlDebug]'#010+
  '    else if CheckCommand(I,'#039'm'#039','#039'compile'#039') then'#010+
  '      FRunMode:=rmCompile'#010+
  '    else if CheckCommand(I,'#039'b'#039','#039'build'#039') then'#010+
  '      FRunMode:=rmBuild'#010+
  '    else if CheckCommand(I,'#039'i'#039','#039'install'#039') then'#010+
  '      FRunMode:=rmInstall'#010+
  '    els','e if CheckCommand(I,'#039'c'#039','#039'clean'#039') then'#010+
  '      FRunMode:=rmClean'#010+
  '    else if CheckCommand(I,'#039'a'#039','#039'archive'#039') then'#010+
  '      FRunMode:=rmarchive'#010+
  '    else if CheckCommand(I,'#039'M'#039','#039'manifest'#039') then'#010+
  '      FRunMode:=rmManifest'#010+
  '    else if CheckOption(I,'#039'h'#039',',#039'help'#039') then'#010+
  '      Usage('#039#039',[])'#010+
  '    else if Checkoption(I,'#039'C'#039','#039'cpu'#039') then'#010+
  '      Defaults.CPU:=StringToCPU(OptionArg(I))'#010+
  '    else if Checkoption(I,'#039'O'#039','#039'os'#039') then'#010+
  '      Defaults.OS:=StringToOS(OptionArg(I))'#010+
  '    else if Checkoption(I,'#039't'#039','#039'targ','et'#039') then'#010+
  '      Defaults.Target:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'l'#039','#039'list-commands'#039') then'#010+
  '      FListMode:=True'#010+
  '    else if Checkoption(I,'#039'P'#039','#039'prefix'#039') then'#010+
  '      Defaults.Prefix:=OptionArg(I)'#010+
  '    else if Checkoption(I,'#039'n'#039','#039'nofpccfg',#039') then'#010+
  '      Defaults.NoFPCCfg:=true'#010+
  '    else if CheckOption(I,'#039'B'#039','#039'baseinstalldir'#039') then'#010+
  '      Defaults.BaseInstallDir:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'UL'#039','#039'localunitdir'#039') then'#010+
  '      Defaults.LocalUnitDir:=OptionArg(I)'#010+
  '    else if',' CheckOption(I,'#039'UG'#039','#039'globalunitdir'#039') then'+
  #010+
  '      Defaults.GlobalUnitDir:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'o'#039','#039'options'#039') then'#010+
  '      begin'#010+
  '        OptString := OptionArg(I);'#010+
  '        while OptString <> '#039#039' do'#010+
  '          Defaults.Options.Ad','d(SplitSpaces(OptString));'#010+
  '      end'#010+
  '    else if CheckOption(I,'#039'r'#039','#039'compiler'#039') then'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    if CheckOption(I,',#039'v'#039','#039'verbose'#039') then'#010+
  '      FLogLevels:=AllMessages'#010+
  '    else if CheckOption(I,'#039'd'#039','#039'debug'#039') then'#010+
  '      FLogLevels:=AllMessages+[vlDebug]'#010+
  '    else if CheckCommand(I,'#039'm'#039','#039'compile'#039') then'#010+
  '      FRunMode:=rmCompile'#010+
  '    else if CheckCommand(I,'#039'b'#039','#039'b','uild'#039') then'#010+
  '      FRunMode:=rmBuild'#010+
  '    else if CheckCommand(I,'#039'i'#039','#039'install'#039') then'#010+
  '      FRunMode:=rmInstall'#010+
  '    else if CheckCommand(I,'#039'c'#039','#039'clean'#039') then'#010+
  '      FRunMode:=rmClean'#010+
  '    else if CheckCommand(I,'#039'a'#039','#039'archive'#039') then'#010+
  '      FRunMode:','=rmarchive'#010+
  '    else if CheckCommand(I,'#039'M'#039','#039'manifest'#039') then'#010+
  '      FRunMode:=rmManifest'#010+
  '    else if CheckOption(I,'#039'h'#039','#039'help'#039') then'#010+
  '      Usage('#039#039',[])'#010+
  '    else if Checkoption(I,'#039'C'#039','#039'cpu'#039') then'#010+
  '      Defaults.CPU:=StringToCPU(OptionArg(I))'#010+
  '    ','else if Checkoption(I,'#039'O'#039','#039'os'#039') then'#010+
  '      Defaults.OS:=StringToOS(OptionArg(I))'#010+
  '    else if Checkoption(I,'#039't'#039','#039'target'#039') then'#010+
  '      Defaults.Target:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'l'#039','#039'list-commands'#039') then'#010+
  '      FListMode:=True'#010+
  '    ','else if Checkoption(I,'#039'P'#039','#039'prefix'#039') then'#010+
  '      Defaults.Prefix:=OptionArg(I)'#010+
  '    else if Checkoption(I,'#039'n'#039','#039'nofpccfg'#039') then'#010+
  '      Defaults.NoFPCCfg:=true'#010+
  '    else if CheckOption(I,'#039'B'#039','#039'baseinstalldir'#039') then'#010+
  '      Defaults.BaseInstallDir:=Op','tionArg(I)'#010+
  '    else if CheckOption(I,'#039'UL'#039','#039'localunitdir'#039') then'#010+
  '      Defaults.LocalUnitDir:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'UG'#039','#039'globalunitdir'#039') then'#010+
  '      Defaults.GlobalUnitDir:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'r'#039','#039'compiler',#039') then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      Defaults.Compiler:=OptionArg(I)'#010+
  '    else if CheckOption(I,'#039'f'#039','#039'config'#039') then'#010+
  '      DefaultsFileName:=OptionArg(I)'#010+
  '    else'#010+
  '      begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Usage(SEr','rInValidArgument,[I,ParamStr(I)]);'#010+
  '      end;'#010+
  '    end;'#010+
  '  If DefaultsFileName<>'#039#039' then'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      Usage(SErrInValidArgument,[I,ParamStr(I)]);'#010+
  '      end;'#010+
  '    end;'#010+
  '  If DefaultsFileName','<>'#039#039' then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Defaults.LocalInit(DefaultsFileName);'#010+
  '  Defaults.CompilerDefaults;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Usage(const FMT: String; Args: array of con'+
  'st);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  #010,
  '  Procedure LogCmd(const LC,Msg : String);'#010+
  '  begin'#010+
  '    Log(vlInfo,Format('#039' %-12s %s'#039',[LC,MSG]));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  #010+
  '  Procedure LogCmd(const LC,Msg : String);'#010+
  '  begin'#010+
  '    Log(vlInfo,Format('#039' %','-12s %s'#039',[LC,MSG]));'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  '  Procedure LogOption(const C,LC,Msg : String);'#010+
  '  begin'#010+
  '    Log(vlInfo,Format('#039' -%s --%-16s %s'#039',[C,LC,MSG]));'#010+
  '  end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Procedure LogAr','gOption(const C,LC,Msg : String);'#010+
  '  begin'#010+
  '    Log(vlInfo,Format('#039' -%s --%-20s %s'#039',[C,LC+'#039'='#039'+SVal'+
  'ue,MSG]));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Procedure LogArgOption(const C,LC,Msg : String);'#010+
  '  begin'#010+
  '    Log(vlInfo,Format('#039' -%s --%-20','s %s'#039',[C,LC+'#039'='#039'+SV'+
  'alue,MSG]));'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  #010+
  'begin'#010+
  '  // Force the Usage to be displayed'#010+
  '  Include(FLogLevels,vlInfo);'#010+
  '  If (FMT<>'#039#039') then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Log(vlInfo,Format(Fmt,Args))',';'#010+
  '  Log(vlInfo,Format(SHelpUsage,[Paramstr(0)]));'#010+
  '  Log(vlInfo,SHelpCommand);'#010+
=======
  '    Log(vlInfo,Format(Fmt,Args));'#010+
  '  Log(vlInfo,Format(SHelpUsage,[Paramstr(0)]));'#010+
  '  Log(vlInfo,SHelpCommand)',';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Log(vlInfo,Format(Fmt,Args));'#010+
  '  Log(vlInfo,Format(SHelpUsage,[Paramstr(0)]));'#010+
  '  Log(vlInfo,SHelpCommand)',';'#010+
>>>>>>> origin/fixes_2_2
  '  LogCmd('#039'compile'#039',SHelpCompile);'#010+
  '  LogCmd('#039'build'#039',SHelpBuild);'#010+
  '  LogCmd('#039'install'#039',SHelpInstall);'#010+
  '  LogCmd('#039'clean'#039',SHelpClean);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  LogCmd('#039'archive'#039',SHelpArchive);'#010,
  '  LogCmd('#039'manifest'#039',SHelpManifest);'#010+
  '  Log(vlInfo,SHelpCmdOptions);'#010+
  '  LogOption('#039'h'#039','#039'help'#039',SHelpHelp);'#010+
  '  LogOption('#039'l'#039','#039'list-commands'#039',SHelpList);'#010+
  '  LogOption('#039'n'#039','#039'nofpccfg'#039',SHelpNoFPCCfg);'#010+
  '  LogOption('#039'v'#039','#039'verbose'#039',SHelpVerbose);'#010+
  '  LogArgOp','tion('#039'C'#039','#039'cpu'#039',SHelpCPU);'#010+
  '  LogArgOption('#039'O'#039','#039'os'#039',SHelpOS);'#010+
  '  LogArgOption('#039't'#039','#039'target'#039',SHelpTarget);'#010+
  '  LogArgOption('#039'P'#039','#039'prefix'#039',SHelpPrefix);'#010+
  '  LogArgOption('#039'B'#039','#039'baseinstalldir'#039',SHelpBaseInstalldi'+
  'r);'#010+
  '  LogArgOption('#039'UL'#039','#039'localunitdir'#039',SHe','lpLocalUnitdir'+
  ');'#010+
  '  LogArgOption('#039'UG'#039','#039'globalunitdir'#039',SHelpGlobalUnitdir'+
  ');'#010+
  '  LogArgOption('#039'r'#039','#039'compiler'#039',SHelpCompiler);'#010+
  '  LogArgOption('#039'f'#039','#039'config'#039',SHelpConfig);'#010+
  '  LogArgOption('#039'o'#039','#039'options'#039',SHelpOptions);'#010+
  '  Log(vlInfo,'#039#039');'#010+
  '  If (FMT<>'#039#039') t','hen'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  LogCmd('#039'archive'#039',SHelpArchive);'#010+
  '  LogCmd('#039'manifest'#039',SHelpManifest);'#010+
  '  Log(vlInfo,SHelpCmdOptions);'#010+
  '  LogOpti','on('#039'h'#039','#039'help'#039',SHelpHelp);'#010+
  '  LogOption('#039'l'#039','#039'list-commands'#039',SHelpList);'#010+
  '  LogOption('#039'n'#039','#039'nofpccfg'#039',SHelpNoFPCCfg);'#010+
  '  LogOption('#039'v'#039','#039'verbose'#039',SHelpVerbose);'#010+
  '  LogArgOption('#039'C'#039','#039'cpu'#039',SHelpCPU);'#010+
  '  LogArgOption('#039'O'#039','#039'os'#039',SHelpOS);'#010+
  '  LogArgOption('#039,'t'#039','#039'target'#039',SHelpTarget);'#010+
  '  LogArgOption('#039'P'#039','#039'prefix'#039',SHelpPrefix);'#010+
  '  LogArgOption('#039'B'#039','#039'baseinstalldir'#039',SHelpBaseInstalldi'+
  'r);'#010+
  '  LogArgOption('#039'UL'#039','#039'localunitdir'#039',SHelpLocalUnitdir);'+
  #010+
  '  LogArgOption('#039'UG'#039','#039'globalunitdir'#039',SHelpGlobalUnitdir'+
  ');'#010+
  '  ','LogArgOption('#039'r'#039','#039'compiler'#039',SHelpCompiler);'#010+
  '  LogArgOption('#039'f'#039','#039'config'#039',SHelpConfig);'#010+
  '  Log(vlInfo,'#039#039');'#010+
  '  If (FMT<>'#039#039') then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    halt(1)'#010+
  '  else'#010+
  '    halt(0);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Compile(Force: Boolean);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FBuildEngine.ForceCompile:=Force;'#010+
  '  FBuildEngine.Compile(FPackages);'#010+
  '  Log(vlWarning,SWarnDone);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Clean;',#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  FBuildEngine.Forc','eCompile:=Force;'#010+
  '  FBuildEngine.Compile(FPackages);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Clean;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  BuildEngine.Clean(FPackages);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Install;'#010+
  'begin'#010+
  '  BuildEngine.Install(FPackages);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCustomInstaller.Archive;'#010+
  'begin'#010+
  '  // Force generation of manifest.xml, this is required for the repos',
  'itory'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TCustomIn','staller.Archive;'#010+
  'begin'#010+
  '  // Force generation of manifest.xml, this is required for the reposi'+
  'tory'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  BuildEngine.Manifest(FPackages);'#010+
  '  BuildEngine.Archive(FPackages);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.Manifest;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  BuildEngine.Manifest(FPackages);'#010+
=======
  '  BuildEngine.Manifest(','FPackages);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  BuildEngine.Manifest(','FPackages);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TCustomInstaller.CheckPackages;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If (FPackages.Count=0) ','then'#010+
=======
  '  If (FPackages.Count=0) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If (FPackages.Count=0) then'#010+
>>>>>>> origin/fixes_2_2
  '    Error(SErrNoPackagesDefined);'#010+
  '  // Check for other obvious errors ?'#010+
  'end;'#010+
  #010+
  #010+
  'Function TCustomInstaller.Run : Boolean;'#010+
  'begin'#010+
  '  Result:=True;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  try'#010+
  '    CheckPackages;'#010+
  '    CreateBuildEngine;'#010+
  '    Case RunMode of'#010+
  '      rmCompile : Compile','(False);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  ' ',' try'#010+
  '    CheckPackages;'#010+
  '    CreateBuildEngine;'#010+
  '    Case RunMode of'#010+
  '      rmCompile : Compile(False);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      rmBuild   : Compile(True);'#010+
  '      rmInstall : Install;'#010+
  '      rmArchive : Archive;'#010+
  '      rmClean    : Clean;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      rmManifest : Manifest;'#010+
=======
  '      rmManifest : Manifes','t;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      rmManifest : Manifes','t;'#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  '  except'#010+
  '    On E : Exception do'#010+
  '      begin'#010+
  '      Log(vlError,SErrInstaller);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ','     Log(vlError,E.Message);'#010+
=======
  '      Log(vlError,E.Message);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Log(vlError,E.Message);'#010+
>>>>>>> origin/fixes_2_2
  '      Result:=False;'#010+
  '      end;'#010+
  '  end;'#010+
  '  // Force returning an exitcode to the shell'#010+
  '  if not Result then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    ExitCode:=1;'#010+
=======
  '    ExitCode:','=1;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    ExitCode:','=1;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '   ','                             TFPCInstaller'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TFPCInstaller.Create(AOwner: TComponent);'#010+
  'begin'#010+
  '  if assigned(Defaults) then'#010+
  '    Error(SErrAlreadyInitiali','zed);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '                                TFPCInstaller'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TFPCInstaller.','Create(AOwner: TComponent);'#010+
  'begin'#010+
  '  if assigned(Defaults) then'#010+
  '    Error(SErrAlreadyInitialized);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Defaults:=TFPCDefaults.Create;'#010+
  '  inherited Create(AOwner);'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
<<<<<<< HEAD
<<<<<<< HEAD
  '*******'#010+
  '                                 TBasicInstaller'#010+
  '***************************************','*****************************'+
  '********}'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '****','***'#010+
  '                                 TBasicInstaller'#010+
  '**********************************************************************'+
  '******}'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'constructor TBasicInstaller.Create(AOwner: TComponent);'#010+
  'begin'#010+
  '  if assigned(Defaults) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Error(SErrAlreadyInitialized);'#010+
=======
  '    Error(SErrAlr','eadyInitialized);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Error(SErrAlr','eadyInitialized);'#010+
>>>>>>> origin/fixes_2_2
  '  Defaults:=TBasicDefaults.Create;'#010+
  '  inherited Create(AOwner);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{***','****************************************************************'+
  '*********'#010+
  '                                 TBuildEngine'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TBuildEngine.Create(AOwner: T','Component);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{*********************************************************************'+
  '*******'#010+
  '                                 TBuildEngine'#010+
  '****************************','****************************************'+
  '********}'#010+
  #010+
  'constructor TBuildEngine.Create(AOwner: TComponent);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  inherited Create(AOwner);'#010+
  '  // Maybe this should be the current directory ?'#010+
  '  // Or have it as a command-line option.'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  // Would allow to put all '#039'installers'#039' in one dir and call t'+
  'hem'#010+
  '  // With --start-dir=/path/to/sourc','es.'#010+
=======
  '  // Would',' allow to put all '#039'installers'#039' in one dir and call'+
  ' them'#010+
  '  // With --start-dir=/path/to/sources.'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  // Would',' allow to put all '#039'installers'#039' in one dir and call'+
  ' them'#010+
  '  // With --start-dir=/path/to/sources.'#010+
>>>>>>> origin/fixes_2_2
  '  FStartDir:=includeTrailingPathDelimiter(GetCurrentDir);'#010+
  '  FExternalPackages:=TPackages.Create(TPackage);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'destructor TBuildEngine.Destroy;'#010+
=======
  'destructor TBuildEngine.Destro','y;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'destructor TBuildEngine.Destro','y;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FreeAndNil(FExternalPackages);'#010+
  '  inherited Destroy;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine','.Error(const Msg: String);'#010+
=======
  'procedure TBuildEngine.Error(const Msg: String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.Error(const Msg: String);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Raise EInstallerError.Create(Msg);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Error(const Fmt: String; const Args: array of c'+
<<<<<<< HEAD
<<<<<<< HEAD
  'onst);'#010+
=======
  'o','nst);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'o','nst);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Raise EInstallerError.CreateFmt(Fmt,Args);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.Execu','teCommand(const Cmd,Args : String; Ignor'+
  'eError : Boolean = False);'#010+
=======
  'procedure TBuildEngine.ExecuteCommand(const Cmd,Args : String; IgnoreE'+
  'rror : Boolean = False);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.ExecuteCommand(const Cmd,Args : String; IgnoreE'+
  'rror : Boolean = False);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  E : Integer;'#010+
  'begin'#010+
  '  Log(vlInfo,SInfoExecutingCommand,[Cmd,Args]);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if ListMode then'#010+
  '    Log(vlCommand,'#039'%s %s'#039',[Cmd,Args])'#010+
  '  else'#010+
  '    begin'#010+
  '      // We should check c','md for spaces, and move all after first sp'+
  'ace to args.'#010+
  '      E:=ExecuteProcess(cmd,args);'#010+
  '      If (E<>0) and (not IgnoreError) then'#010+
  '        Error(SErrExternalCommandFailed,[Cmd,E]);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  if Lis','tMode then'#010+
  '    Log(vlCommand,'#039'%s %s'#039',[Cmd,Args])'#010+
  '  else'#010+
  '    begin'#010+
  '      // We should check cmd for spaces, and move all after first spac'+
  'e to args.'#010+
  '      E:=ExecuteProcess(cmd,args);'#010+
  '      If (E<>0) and (not IgnoreError) then'#010+
  '        Error(S','ErrExternalCommandFailed,[Cmd,E]);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TBuildEngine.SysDirectoryExists(','const ADir:string):Boolean;'+
  #010+
=======
  'Function TBuildEngine.SysDirectoryExists(const ADir:string):Boolean;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TBuildEngine.SysDirectoryExists(const ADir:string):Boolean;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  result:=SysUtils.DirectoryExists(ADir);'#010+
  '  if result then'#010+
  '    Log(vlDebug,SDbgDirectoryExists,[ADir,SDbgFound])'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  else'#010+
=======
  ' ',' else'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ',' else'#010+
>>>>>>> origin/fixes_2_2
  '    Log(vlDebug,SDbgDirectoryExists,[ADir,SDbgNotFound]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TBuildEngine','.SysFileExists(const AFileName:string):Boolean;'+
  #010+
  'begin'#010+
  '  result:=SysUtils.FileExists(AFileName);'#010+
  '  if result then'#010+
  '    Log(vlDebug,SDbgFileExists,[AFileName,SDbgFound])'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TBuildEngine.SysFileExists(const AFileName:string):Boolean;'#010+
  'begin'#010+
  '  result:=SysUtils.FileExists(AFileName);'#010+
  '  if result then'#010+
  '    Log(vlDebug,SDbgFileExists,[AFi','leName,SDbgFound])'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  else'#010+
  '    Log(vlDebug,SDbgFileExists,[AFileName,SDbgNotFound]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'p','rocedure TBuildEngine.SysCopyFile(Const Src,Dest : String);'#010+
=======
  'procedure TBuildEngine.SysCopyFile(Const Src,Dest : String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.SysCopyFile(Const Src,Dest : String);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  D,S : String;'#010+
  '  Fin,FOut : TFileStream;'#010+
  '  Count : Int64;'#010+
  '  A : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Log(vlInfo,SInfoCopyingFile,[Src,Dest]);'#010+
  '  FIn:=TFileStream.Create(Src,fmopenRead);'#010+
  '  Try'#010+
  '    ','D:=IncludeTrailingPathDelimiter(Dest);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Lo','g(vlInfo,SInfoCopyingFile,[Src,Dest]);'#010+
  '  FIn:=TFileStream.Create(Src,fmopenRead);'#010+
  '  Try'#010+
  '    D:=IncludeTrailingPathDelimiter(Dest);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    If DirectoryExists(D) then'#010+
  '      S:=D+ExtractFileName(Src)'#010+
  '    else'#010+
  '      S:=Dest;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FOut:=TFileStream.Create(S,fmCreate);'#010+
  '    Try'#010+
  '      Count:=Fout.CopyFrom(FIn,0);'#010+
  '      If (Count<>Fin.Size) th','en'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    FOut:=TFileStream.','Create(S,fmCreate);'#010+
  '    Try'#010+
  '      Count:=Fout.CopyFrom(FIn,0);'#010+
  '      If (Count<>Fin.Size) then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '        Error(SErrCopyingFile,[Src,S]);'#010+
  '    Finally'#010+
  '      FreeAndNil(Fout);'#010+
  '    end;'#010+
  '    A:=FileGetDate(FIn.Handle);'#010+
  '    If (A=-1) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      log(vlWarning,SWarnFailedToGetTime,[Src])'#010+
  '    else'#010+
  '      if FileSetDate(S,A)<>0 then'#010+
  '        Lo','g(vlWarning,SWarnFailedToSetTime,[S]);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      log','(vlWarning,SWarnFailedToGetTime,[Src])'#010+
  '    else'#010+
  '      if FileSetDate(S,A)<>0 then'#010+
  '        Log(vlWarning,SWarnFailedToSetTime,[S]);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  finally'#010+
  '    FreeAndNil(Fin);'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.SysMoveFile(Const Src,Dest : String);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Var'#010+
  '  S : String;'#010+
  'begin'#010+
  '  If DirectoryExists(IncludeTrailingPathDelimiter(Dest)) then'#010+
  '    S:=In','cludeTrailingPathDelimiter(Dest)+ExtractFileName(Src)'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Var',#010+
  '  S : String;'#010+
  'begin'#010+
  '  If DirectoryExists(IncludeTrailingPathDelimiter(Dest)) then'#010+
  '    S:=IncludeTrailingPathDelimiter(Dest)+ExtractFileName(Src)'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  else'#010+
  '    S:=Dest;'#010+
  '  If Not RenameFile(Src,S) then'#010+
  '    begin'#010+
  '      Try'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        SysCopyFile(Src,S);'#010+
  '        SysDeleteFile(Src);'#010+
  '      Except'#010+
  '        On E : Exception Do'#010+
  '          Error(S','ErrMovingFile,[Src,S]);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '        SysCopyFile(Sr','c,S);'#010+
  '        SysDeleteFile(Src);'#010+
  '      Except'#010+
  '        On E : Exception Do'#010+
  '          Error(SErrMovingFile,[Src,S]);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.SysDeleteFile(Const AFileName : String);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if not FileExists(AFileName) then'#010+
  '    Log(vldebug,SDbgFileDoesNotExist,[AFileName])'#010+
  '  else If Not DeleteFile(AFileNam','e) then'#010+
=======
  '  if not FileExists(AFileNa','me) then'#010+
  '    Log(vlWarning,SWarnFileDoesNotExist,[AFileName])'#010+
  '  else If Not DeleteFile(AFileName) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  if not FileExists(AFileNa','me) then'#010+
  '    Log(vlWarning,SWarnFileDoesNotExist,[AFileName])'#010+
  '  else If Not DeleteFile(AFileName) then'#010+
>>>>>>> origin/fixes_2_2
  '    Error(SErrDeletingFile,[AFileName]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.SysArchiveFiles(List: TStrings;Const AFileName:'+
  ' String);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If Not (Assigned(OnArchivefiles) or Assigned(ArchiveFilesProc)) then'+
  #010+
  '    Raise EInstallerError.C','reate(SErrNoArchiveSupport);'#010+
  '  If Assigned(ArchiveFilesProc) then'#010+
  '    ArchiveFilesProc(AFileName,List)'#010+
  '  else'#010+
  '    OnArchiveFiles(AFileName,List);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  I','f Not (Assigned(OnArchivefiles) or Assigned(ArchiveFilesProc)) th'+
  'en'#010+
  '    Raise EInstallerError.Create(SErrNoArchiveSupport);'#010+
  '  If Assigned(ArchiveFilesProc) then'#010+
  '    ArchiveFilesProc(AFileName,List)'#010+
  '  else'#010+
  '    OnArchiveFiles(AFileName,List);',#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.LogIndent;'#010+
  'begin'#010+
  '  FLogPrefix:=FLogPrefix+'#039'  '#039';'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure',' TBuildEngine.LogUnIndent;'#010+
=======
  'procedure TBuildEngine.LogUnIndent;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.LogUnIndent;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Delete(FLogPrefix,1,2);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Log(Level: TVerboseLevel; const Msg: String);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If Assigned(FOnLog) then'#010+
  '    begin'#010+
  '      if Level in [vlInfo,vlDebug] then'#010+
  '        FOnLog(Level,F','LogPrefix+Msg)'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  If',' Assigned(FOnLog) then'#010+
  '    begin'#010+
  '      if Level in [vlInfo,vlDebug] then'#010+
  '        FOnLog(Level,FLogPrefix+Msg)'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      else'#010+
  '        FOnLog(Level,Msg);'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Log(Level: TVerboseLevel; const Fmt: String;con'+
<<<<<<< HEAD
<<<<<<< HEAD
  'st Args: array of const);'#010+
=======
  'st Arg','s: array of const);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'st Arg','s: array of const);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Log(Level,Format(Fmt,Args));'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.EnterDir','(ADir: String);'#010+
=======
  'procedure TBuildEngine.EnterDir(ADir: String);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.EnterDir(ADir: String);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  D : String;'#010+
  'begin'#010+
  '  D:=FStartDir;'#010+
  '  D:=D+ADir;'#010+
  '  Log(vlDebug,SDbgEnterDir,[D]);'#010+
  '  If Not SetCurrentDir(D) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Error(SErrChangeDirFailed,[D]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CmdCopyFiles(List: TStrings; Const DestDi','r: S'+
  'tring);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Error(SE','rrChangeDirFailed,[D]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CmdCopyFiles(List: TStrings; Const DestDir: Str'+
  'ing);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'Var'#010+
  '  Args : String;'#010+
  '  I : Integer;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  DestFileName : String;'#010+
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'begin'#010+
  '  CmdCreateDir(DestDir);'#010+
  '  If (Defaults.Copy<>'#039#039') then'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Args:=FileListToString(List,'#039#039');'#010+
  '      Args:=Args+'#039' '#039'+DestDir;'#010+
  '      ExecuteCommand(Defa','ults.Copy,Args);'#010+
  '    end'#010+
  '  else'#010+
  '    For I:=0 to List.Count-1 do'#010+
  '      if List.Names[i]<>'#039#039' then'#010+
  '        begin'#010+
  '          DestFileName:=DestDir+list.ValueFromIndex[i];'#010+
  '          CmdCreateDir(ExtractFilePath(DestFileName));'#010+
  '          SysCopyFi','le(List.names[i],DestFileName)'#010+
  '        end'#010+
  '      else'#010+
  '        SysCopyFile(List[i],DestDir);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      Args:=FileListToSt','ring(List,'#039#039');'#010+
  '      Args:=Args+'#039' '#039'+DestDir;'#010+
  '      ExecuteCommand(Defaults.Copy,Args);'#010+
  '    end'#010+
  '  else'#010+
  '    For I:=0 to List.Count-1 do'#010+
  '      SysCopyFile(List[i],DestDir);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CmdCreateDir(const DestDir: String);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begin'#010+
  '  If (Defaults.MkDir<>'#039#039') then'#010+
  '    ExecuteCommand(Defaults.MkDir,DestDir)'#010+
  ' ',' else'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'beg','in'#010+
  '  If (Defaults.MkDir<>'#039#039') then'#010+
  '    ExecuteCommand(Defaults.MkDir,DestDir)'#010+
  '  else'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    If not ForceDirectories(DestDir) then'#010+
  '      Error(SErrCreatingDirectory,[DestDir]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.CmdMoveFiles(List: TStrings; Const DestDir: Str'+
  'ing);'#010+
=======
  'procedure TBuildEngine.CmdMoveFiles(List: TStrings; Const Des','tDir: S'+
  'tring);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.CmdMoveFiles(List: TStrings; Const Des','tDir: S'+
  'tring);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  Args : String;'#010+
  '  I : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  CmdCreateDir(DestDi','r);'#010+
=======
  '  CmdCreateDir(DestDir);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  CmdCreateDir(DestDir);'#010+
>>>>>>> origin/fixes_2_2
  '  If (Defaults.Move<>'#039#039') then'#010+
  '    begin'#010+
  '      Args:=FileListToString(List,'#039#039');'#010+
  '      Args:=Args+'#039' '#039'+DestDir;'#010+
  '      ExecuteCommand(Defaults.Move,Args);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    end'#010+
  '  else'#010+
  '    For I:=0 to List.Count-1 do'#010+
  '      SysMoveFile(List[i],DestDir);'#010+
  'en','d;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    end',#010+
  '  else'#010+
  '    For I:=0 to List.Count-1 do'#010+
  '      SysMoveFile(List[i],DestDir);'#010+
  'end;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'procedure TBuildEngine.CmdDeleteFiles(List: TStrings);'#010+
  'Var'#010+
  '  Args : String;'#010+
  '  I : Integer;'#010+
  'begin'#010+
  '  If (Defaults.Remove<>'#039#039') then'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Args:=FileListToString(List,'#039#039');'#010+
  '      ExecuteCommand(Defaults.Remove,Args);'#010+
  '    end'#010+
  '  els','e'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      Args:=FileLi','stToString(List,'#039#039');'#010+
  '      ExecuteCommand(Defaults.Remove,Args);'#010+
  '    end'#010+
  '  else'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    For I:=0 to List.Count-1 do'#010+
  '      SysDeleteFile(List[i]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CmdArchiveFiles(List: TStrings; Const ArchiveFi'+
  'le: String);'#010+
  'Var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  S,C,O : String;'#010+
  'begin'#010+
  '  If (Defaults.Archive='#039#039') then'#010+
  '    SysArchiveFiles(List,A','rchiveFile)'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  S,','C,O : String;'#010+
  'begin'#010+
  '  If (Defaults.Archive='#039#039') then'#010+
  '    SysArchiveFiles(List,ArchiveFile)'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  else'#010+
  '    begin'#010+
  '      S:=FileListToString(List,'#039#039');'#010+
  '      SplitCommand(Defaults.Archive,C,O);'#010+
  '      If (O='#039#039') then'#010+
  '        O:=ArchiveFile+'#039' '#039'+S'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      else'#010+
  '        O:=Substitute(O,['#039'ARCHIVE'#039',ArchiveFile,'#039'FILESORDIR'+
  'S'#039']);'#010+
  '      Ex','ecuteCommand(C,O);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '     ',' else'#010+
  '        O:=Substitute(O,['#039'ARCHIVE'#039',ArchiveFile,'#039'FILESORDIR'+
  'S'#039']);'#010+
  '      ExecuteCommand(C,O);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  'end;'#010+
  #010+
  'Function TBuildEngine.FileNewer(const Src,Dest : String) : Boolean;'#010+
  #010+
  'Var'#010+
  '  DS,DD : Longint;'#010+
  '  D1,D2 : TDateTime;'#010+
  #010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  DS:=FileAge(Src);'#010+
  '  DD:=FileAge(Dest);'#010+
  '  D1:=FileDateToDateTime(DS);'#010+
  '  D2:=FileDateT','oDateTime(DD);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  DS:=Fi','leAge(Src);'#010+
  '  DD:=FileAge(Dest);'#010+
  '  D1:=FileDateToDateTime(DS);'#010+
  '  D2:=FileDateToDateTime(DD);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Log(vlDebug,SDbgComparingFileTimes,[Src,DateTimeToStr(D1),Dest,DateT'+
  'imeToStr(D2)]);'#010+
  '  Result:=D1>=D2;'#010+
  '  If Result then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Log(vlInfo,SInfoSourceNewerDest,[Src,DateTimeToStr(D1),Dest,DateTi'+
  'meToStr(D2)]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TB','uildEngine.ExecuteCommands(Commands: TCommands; At: TCom'+
  'mandAt);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Log(vlInfo,SInfoSource','NewerDest,[Src,DateTimeToStr(D1),Dest,Date'+
  'TimeToStr(D2)]);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.ExecuteCommands(Commands: TCommands; At: TComma'+
  'ndAt);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  C : TCommand;'#010+
  '  I : Integer;'#010+
  '  Cmd,O : String;'#010+
  '  E : Boolean;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  For I:=0 to Commands.Count-1 do'#010+
  '    begin'#010+
  '      C:=Commands.CommandItems[i];'#010+
  '      if (C.At=At) the','n'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  For I:=0 to Commands.','Count-1 do'#010+
  '    begin'#010+
  '      C:=Commands.CommandItems[i];'#010+
  '      if (C.At=At) then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '        begin'#010+
  '          E:=True;'#010+
  '          If (C.SourceFile<>'#039#039') and (C.DestFile<>'#039#039')  then'+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            E:=FileNewer(C.SourceFile,IncludeTrailingPathDelimiter(Dic'+
  'tionary.GetValue('#039'OUTPUTDIR'#039'))+C.DestFile);'#010+
  '          If E then'#010+
  '            ','begin'#010+
  '            If Assigned(C.BeforeCommand) then'#010+
  '              C.BeforeCommand(C);'#010+
  '            O:=Substitute(C.CmdLineOptions,['#039'SOURCE'#039',C.SourceF'+
  'ile,'#039'DEST'#039',C.DestFile]);'#010+
  '            Cmd:=C.Command;'#010+
  '            If (ExtractFilePath(Cmd)='#039#039,') then'#010+
  '              Cmd:=ExeSearch(Cmd,GetEnvironmentvariable('#039'PATH'#039')'+
  ');'#010+
  '            ExecuteCommand(Cmd,O,C.IgnoreResult);'#010+
  '            If Assigned(C.AfterCommand) then'#010+
  '              C.AfterCommand(C);'#010+
  '            end;'#010+
  '        end;'#010+
  '    end;'#010,
=======
=======
>>>>>>> origin/fixes_2_2
  '            E:=FileNewer(C.SourceFile,IncludeTrailingPathDelimiter(D','i'+
  'ctionary.GetValue('#039'OUTPUTDIR'#039'))+C.DestFile);'#010+
  '          If E then'#010+
  '            begin'#010+
  '            If Assigned(C.BeforeCommand) then'#010+
  '              C.BeforeCommand(C);'#010+
  '            O:=Substitute(C.Options,['#039'SOURCE'#039',C.SourceFile,'#039+
  'DEST'#039',C.DestFile',']);'#010+
  '            Cmd:=C.Command;'#010+
  '            If (ExtractFilePath(Cmd)='#039#039') then'#010+
  '              Cmd:=ExeSearch(Cmd,GetEnvironmentvariable('#039'PATH'#039+
  '));'#010+
  '            ExecuteCommand(Cmd,O,C.IgnoreResult);'#010+
  '            If Assigned(C.AfterCommand) then'#010,
  '              C.AfterCommand(C);'#010+
  '            end;'#010+
  '        end;'#010+
  '    end;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Procedure TBuildEngine.LogSearchPath(const ASearchPathName:string;Path'+
  ':TConditionalStrings; ACPU:TCPU;AOS:TOS);'#010+
  'var'#010+
  '  S : String;'#010+
  '  I : Integer;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  C : TConditionalString;'#010+
=======
  '  C : TCondition','alString;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  C : TCondition','alString;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  S:='#039#039';'#010+
  '  for i:=0 to Path.Count-1 do'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      C',':=Path[I];'#010+
=======
  '      C:=Path[I];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      C:=Path[I];'#010+
>>>>>>> origin/fixes_2_2
  '      if (ACPU in C.CPUs) and (AOS in C.OSes) then'#010+
  '        begin'#010+
  '          if S<>'#039#039' then'#010+
  '            S:=S+PathSeparator;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          S:=S+Dictionary.ReplaceStrings(C.Value)'#010+
  '        end;'#010+
  '    end;'#010+
  '  if S<>'#039#039' then'#010+
  '    Log(vlDebug,SDbg','SearchPath,[ASearchPathName,S]);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '          S:=S+Dictionary.ReplaceStr','ings(C.Value)'#010+
  '        end;'#010+
  '    end;'#010+
  '  if S<>'#039#039' then'#010+
  '    Log(vlDebug,SDbgSearchPath,[ASearchPathName,S]);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TBuildEngine.FindFileInPath(Path:TConditionalStrings; AFileNa'+
<<<<<<< HEAD
<<<<<<< HEAD
  'me:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;'#010+
=======
  'me:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean',';'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'me:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean',';'#010+
>>>>>>> origin/fixes_2_2
  'var'#010+
  '  I : Integer;'#010+
  '  C : TConditionalString;'#010+
  'begin'#010+
  '  Result:=false;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ','for i:=0 to Path.Count-1 do'#010+
=======
  '  for i:=0 to Path.Count-1 do'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  for i:=0 to Path.Count-1 do'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      C:=Path[I];'#010+
  '      if (ACPU in C.CPUs) and (AOS in C.OSes) then'#010+
  '        begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          FoundPath:=IncludeTrailingPathDelimiter(Dictionary.ReplaceSt'+
  'rings(C.Value));'#010+
  '          if FileExists(FoundPat','h+AFileName) then'#010+
=======
  '          FoundPath:=IncludeTrailingPathDelimit','er(Dictionary.Replace'+
  'Strings(C.Value));'#010+
  '          if FileExists(FoundPath+AFileName) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '          FoundPath:=IncludeTrailingPathDelimit','er(Dictionary.Replace'+
  'Strings(C.Value));'#010+
  '          if FileExists(FoundPath+AFileName) then'#010+
>>>>>>> origin/fixes_2_2
  '            begin'#010+
  '              result:=true;'#010+
  '              exit;'#010+
  '            end;'#010+
  '        end;'#010+
  '    end;'#010+
  '  FoundPath:='#039#039';'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Procedure TBuildEngine.ResolveFileNames(APackage : TPackage; ACPU:TCPU'+
  ';AOS:TOS;DoChangeDir:bo','olean=true);'#010+
=======
  'Procedure TBuildEngin','e.ResolveFileNames(APackage : TPackage; ACPU:TC'+
  'PU;AOS:TOS;DoChangeDir:boolean=true);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Procedure TBuildEngin','e.ResolveFileNames(APackage : TPackage; ACPU:TC'+
  'PU;AOS:TOS;DoChangeDir:boolean=true);'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '  procedure FindMainSource(T:TTarget);'#010+
  '  var'#010+
  '    SD,SF  : String;'#010+
  '  begin'#010+
  '    LogSearchPath('#039'package source'#039',APackage.SourcePath,ACPU,AO'+
  'S);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    SD:=Dictionary.ReplaceStrings(T.Directory);'#010+
  '    SF:=Dictionary.ReplaceStrings(T.So','urceFileName);'#010+
=======
  '    SD:=Dictio','nary.ReplaceStrings(T.Directory);'#010+
  '    SF:=Dictionary.ReplaceStrings(T.SourceFileName);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    SD:=Dictio','nary.ReplaceStrings(T.Directory);'#010+
  '    SF:=Dictionary.ReplaceStrings(T.SourceFileName);'#010+
>>>>>>> origin/fixes_2_2
  '    if SD='#039#039' then'#010+
  '      FindFileInPath(APackage.SourcePath,SF,SD,ACPU,AOS);'#010+
  '    if SD<>'#039#039' then'#010+
  '      SD:=IncludeTrailingPathDelimiter(SD);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    T.FTargetSourceFileName:=SD+SF;'#010+
  '    if FileExists(T.TargetSourceFileName) then'#010+
  '   ','   Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetS'+
  'ourceFileName])'#010+
  '    else'#010+
  '      begin'#010+
  '        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APacka'+
  'ge.Name,MakeTargetString(ACPU,AOS)]);'#010+
  '        T.FTargetSourceFileNam','e:='#039#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    T.FTargetS','ourceFileName:=SD+SF;'#010+
  '    if FileExists(T.TargetSourceFileName) then'#010+
  '      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSou'+
  'rceFileName])'#010+
  '    else'#010+
  '      begin'#010+
  '        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,Ma','ke'+
  'TargetString(ACPU,AOS)]);'#010+
  '        T.FTargetSourceFileName:='#039#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      end;'#010+
  '  end;'#010+
  #010+
  '  procedure FindIncludeSources(T:TTarget);'#010+
  '  var'#010+
  '    SD,SF  : String;'#010+
  '    D : TDependency;'#010+
  '    j : integer;'#010+
  '  begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    LogSearchPath('#039'target include'#039',T.IncludePath,ACPU,AOS);'#010+
  '    LogSearchPath('#039'package include'#039',AP','ackage.IncludePath,ACP'+
  'U,AOS);'#010+
=======
  '    LogSearchPath('#039'target include'#039',T.Inc','ludePath,ACPU,AOS);'#010+
  '    LogSearchPath('#039'package include'#039',APackage.IncludePath,ACPU,'+
  'AOS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    LogSearchPath('#039'target include'#039',T.Inc','ludePath,ACPU,AOS);'#010+
  '    LogSearchPath('#039'package include'#039',APackage.IncludePath,ACPU,'+
  'AOS);'#010+
>>>>>>> origin/fixes_2_2
  '    for j:=0 to T.Dependencies.Count-1 do'#010+
  '      begin'#010+
  '        D:=T.Dependencies[j];'#010+
  '        if (D.DependencyType=depInclude) then'#010+
  '          begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            D.TargetFileName:='#039#039';'#010+
  '            if (ACPU in D.CPUs',') and (AOS in D.OSes) then'#010+
  '              begin'#010+
  '                if ExtractFilePath(D.Value)='#039#039' then'#010+
  '                  begin'#010+
  '                    SF:=Dictionary.ReplaceStrings(D.Value);'#010+
  '                    SD:='#039#039';'#010+
  '                    // first ','check the target specific path'#010+
  '                    if not FindFileInPath(T.IncludePath,SF,SD,ACPU,AOS'+
  ') then'#010+
  '                      FindFileInPath(APackage.IncludePath,SF,SD,ACPU,A'+
  'OS);'#010+
  '                     if SD<>'#039#039' then'#010+
  '                     ','  SD:=IncludeTrailingPathDelimiter(SD);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      ','      D.TargetFileName:='#039#039';'#010+
  '            if (ACPU in D.CPUs) and (AOS in D.OSes) then'#010+
  '              begin'#010+
  '                if ExtractFilePath(D.Value)='#039#039' then'#010+
  '                  begin'#010+
  '                    SF:=Dictionary.ReplaceStrings(D.Value);',#010+
  '                    SD:='#039#039';'#010+
  '                    // first check the target specific path'#010+
  '                    if not FindFileInPath(T.IncludePath,SF,SD,ACPU,AOS'+
  ') then'#010+
  '                      FindFileInPath(APackage.IncludePath,SF,SD,ACPU,A'+
  'OS);',#010+
  '                     if SD<>'#039#039' then'#010+
  '                       SD:=IncludeTrailingPathDelimiter(SD);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '                     D.TargetFileName:=SD+SF;'#010+
  '                  end'#010+
  '                else'#010+
  '                  D.TargetFileName:=D.Value;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                if FileExists(D.TargetFileName) then'#010+
  '             ','     Log(vlDebug,SDbgResolvedIncludeFile,[D.Value,D.Tar'+
  'getFileName])'#010+
  '                else'#010+
  '                  begin'#010+
  '                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, A'+
  'Package.Name, MakeTargetString(ACPU,AOS)]);'#010+
  '            ','        D.TargetFileName:='#039#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '        ','        if FileExists(D.TargetFileName) then'#010+
  '                  Log(vlDebug,SDbgResolvedIncludeFile,[D.Value,D.Targe'+
  'tFileName])'#010+
  '                else'#010+
  '                  begin'#010+
  '                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value,','M'+
  'akeTargetString(ACPU,AOS)]);'#010+
  '                    D.TargetFileName:='#039#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '                  end;'#010+
  '              end;'#010+
  '          end;'#010+
  '      end;'#010+
  '  end;'#010+
  #010+
  '  procedure FindExampleSource(T:TTarget);'#010+
  '  var'#010+
  '    SD,SF  : String;'#010+
  '  begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    LogSearchPath('#039'package example'#039',APackage.ExamplePath,','ACP'+
=======
  '    LogSearchPa','th('#039'package example'#039',APackage.ExamplePath,ACP'+
>>>>>>> graemeg/fixes_2_2
=======
  '    LogSearchPa','th('#039'package example'#039',APackage.ExamplePath,ACP'+
>>>>>>> origin/fixes_2_2
  'U,AOS);'#010+
  '    SD:=Dictionary.ReplaceStrings(T.Directory);'#010+
  '    SF:=Dictionary.ReplaceStrings(T.SourceFileName);'#010+
  '    if SD='#039#039' then'#010+
  '      FindFileInPath(APackage.ExamplePath,SF,SD,ACPU,AOS);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    if SD<>'#039#039' then'#010+
  '      SD:=IncludeTrailingPathDel','imiter(SD);'#010+
=======
  '    if SD','<>'#039#039' then'#010+
  '      SD:=IncludeTrailingPathDelimiter(SD);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    if SD','<>'#039#039' then'#010+
  '      SD:=IncludeTrailingPathDelimiter(SD);'#010+
>>>>>>> origin/fixes_2_2
  '    T.FTargetSourceFileName:=SD+SF;'#010+
  '    if FileExists(T.TargetSourceFileName) then'#010+
  '      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSou'+
  'rceFileName])'#010+
  '    else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      begin'#010+
  '        Log(vlWarning,SWarnSourceFileNot','Found,[T.SourceFileName, APa'+
  'ckage.Name, MakeTargetString(ACPU,AOS)]);'#010+
=======
  '      begi','n'#010+
  '        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,MakeTa'+
  'rgetString(ACPU,AOS)]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      begi','n'#010+
  '        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,MakeTa'+
  'rgetString(ACPU,AOS)]);'#010+
>>>>>>> origin/fixes_2_2
  '        T.FTargetSourceFileName:='#039#039';'#010+
  '      end;'#010+
  '  end;'#010+
  #010+
  'var'#010+
  '  T : TTarget;'#010+
  '  i : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if not((ACPU in APackage.CPUs) and (AOS in APackage.OSes)) then'#010+
  '    exit',';'#010+
=======
  '  if not((ACPU in APackage.CPUs) and (AOS in APa','ckage.OSes)) then'#010+
  '    exit;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  if not((ACPU in APackage.CPUs) and (AOS in APa','ckage.OSes)) then'#010+
  '    exit;'#010+
>>>>>>> origin/fixes_2_2
  '  try'#010+
  '    if DoChangeDir and (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir(APackage.Directory);'#010+
  '    Dictionary.AddVariable('#039'CPU'#039',CPUToString(ACPU));'#010+
  '    Dictionary.AddVariable('#039'OS'#039',OSToString(AOS));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    For I:=0 to APackage.Targets.Count-1',' do'#010+
=======
  '    For I:=0 t','o APackage.Targets.Count-1 do'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    For I:=0 t','o APackage.Targets.Count-1 do'#010+
>>>>>>> origin/fixes_2_2
  '      begin'#010+
  '        T:=APackage.FTargets.TargetItems[I];'#010+
  '        if (ACPU in T.CPUs) and (AOS in T.OSes) then'#010+
  '          begin'#010+
  '            // Debug information'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            Log(vlDebug,SDbgResolvingSourcesOfTarget,[T.Name,MakeTarge'+
  'tString','(ACPU,AOS)]);'#010+
=======
  '            Log(vlDebug,SDbgResolvingSourcesOfTarge','t,[T.Name,MakeTar'+
  'getString(ACPU,AOS)]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '            Log(vlDebug,SDbgResolvingSourcesOfTarge','t,[T.Name,MakeTar'+
  'getString(ACPU,AOS)]);'#010+
>>>>>>> origin/fixes_2_2
  '            LogIndent;'#010+
  #010+
  '            case T.TargetType of'#010+
  '              ttProgram,'#010+
  '              ttUnit,'#010+
  '              ttImplicitUnit :'#010+
  '                begin'#010+
  '                  FindMainSource(T);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                  if T.Dependenc','ies.Count>0 then'#010+
=======
  '      ','            if T.Dependencies.Count>0 then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      ','            if T.Dependencies.Count>0 then'#010+
>>>>>>> origin/fixes_2_2
  '                    FindIncludeSources(T);'#010+
  '                end;'#010+
  '              ttExampleUnit,'#010+
  '              ttExampleProgram :'#010+
  '                begin'#010+
  '                  FindExampleSource(T);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                end;'#010+
  '            en','d;'#010+
=======
  '         ','       end;'#010+
  '            end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '         ','       end;'#010+
  '            end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '            LogUnIndent;'#010+
  '          end;'#010+
  '      end;'#010+
  '  finally'#010+
  '    If DoChangeDir and (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir('#039#039');'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'function TBuildEngine.GetUnitDir(APackage:TPackage):String;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begin'#010+
  '  // Retrieve Full dir','ectory name where to find the units.'#010+
=======
  'be','gin'#010+
  '  // Retrieve Full directory name where to find the units.'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'be','gin'#010+
  '  // Retrieve Full directory name where to find the units.'#010+
>>>>>>> origin/fixes_2_2
  '  // The search order is:'#010+
  '  //  - Package in this fpmake.pp'#010+
  '  //  - LocalUnitDir'#010+
  '  //  - GlobalUnitDir'#010+
  '  if (APackage.UnitDir='#039#039') and'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '     (APackage.State in [tsCompiled, tsNoCompile]) then'#010+
  '    begin'#010+
  '   ','   APackage.UnitDir:=IncludeTrailingPathDelimiter(FStartDir)+Incl'+
  'udeTrailingPathDelimiter(APackage.Directory)+APackage.GetUnitsOutputDi'+
  'r(Defaults.CPU,Defaults.OS);'#010+
  '    end;'#010+
  '  if (APackage.UnitDir='#039#039') and'#010+
  '     (Defaults.LocalUnitDir<>'#039#039') the','n'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '     (APackage.State=tsCompiled) then'#010+
  '    b','egin'#010+
  '      APackage.UnitDir:=IncludeTrailingPathDelimiter(FStartDir)+Includ'+
  'eTrailingPathDelimiter(APackage.Directory)+APackage.GetUnitsOutputDir('+
  'Defaults.CPU,Defaults.OS);'#010+
  '    end;'#010+
  '  if (APackage.UnitDir='#039#039') and'#010+
  '     (Defaults.LocalUnitDir<','>'#039#039') then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.LocalUni'+
  'tDir)+APackage.Name;'#010+
  '      if not SysDirectoryExists(APackage.UnitDir) then'#010+
  '        APackage.UnitDir:='#039#039';'#010+
  '    end;'#010+
  '  if APackage.UnitDir='#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    begin'#010+
  '   ','   APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.Global'+
  'UnitDir)+APackage.Name;'#010+
  '      if not SysDirectoryExists(APackage.UnitDir) then'#010+
  '        APackage.UnitDir:=DirNotFound;'#010+
  '    end;'#010+
  '  // Special error marker to prevent searches in',' case of error'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    b','egin'#010+
  '      APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.GlobalUn'+
  'itDir)+APackage.Name;'#010+
  '      if not SysDirectoryExists(APackage.UnitDir) then'#010+
  '        APackage.UnitDir:=DirNotFound;'#010+
  '    end;'#010+
  '  // Special error marker to prevent sea','rches in case of error'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  if APackage.UnitDir=DirNotFound then'#010+
  '    Result:='#039#039#010+
  '  else'#010+
  '    Result:=APackage.UnitDir;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.AddDependencyIncludePaths(L:TStrings;ATarget: T'+
  'Target);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  D : TDependency;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  SD : Str','ing;'#010+
=======
  '  ','SD : String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  ','SD : String;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  For I:=0 to ATarget.Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=ATarget.Dependencies[i];'#010+
  '      if (D.DependencyType=depInclude) and'#010+
  '         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then'#010+
  '        begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          SD:=E','xcludeTrailingPathDelimiter(ExtractFilePath(D.TargetF'+
=======
  '       ','   SD:=ExcludeTrailingPathDelimiter(ExtractFilePath(D.TargetF'+
>>>>>>> graemeg/fixes_2_2
=======
  '       ','   SD:=ExcludeTrailingPathDelimiter(ExtractFilePath(D.TargetF'+
>>>>>>> origin/fixes_2_2
  'ileName));'#010+
  '          if SD<>'#039#039' then'#010+
  '            L.Add(SD);'#010+
  '        end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.AddDependencyUnitPaths(L:TStrings;APackage: TPa'+
  'ckage);'#010+
  'Var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  I : Integer;'#010+
  '  ','P : TPackage;'#010+
=======
  '  I : Int','eger;'#010+
  '  P : TPackage;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  I : Int','eger;'#010+
  '  P : TPackage;'#010+
>>>>>>> origin/fixes_2_2
  '  D : TDependency;'#010+
  '  S : String;'#010+
  'begin'#010+
  '  For I:=0 to APackage.Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=APackage.Dependencies[i];'#010+
  '      if (D.DependencyType=depPackage) and'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '         (Defaults.CPU in D.CPUs) and (Defaults.OS i','n D.OSes) then'#010+
=======
  '         (Defaults.CPU in D.CPUs) and (Defau','lts.OS in D.OSes) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '         (Defaults.CPU in D.CPUs) and (Defau','lts.OS in D.OSes) then'#010+
>>>>>>> origin/fixes_2_2
  '        begin'#010+
  '          P:=TPackage(D.Target);'#010+
  '          If Assigned(P) then'#010+
  '            begin'#010+
  '              // Already processed?'#010+
  '              S:=GetUnitDir(P);'#010+
  '              if L.IndexOf(S)=-1 then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                begin'#010+
  '  ','                // Add this package and then dependencies'#010+
=======
  '                ','begin'#010+
  '                  // Add this package and then dependencies'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '                ','begin'#010+
  '                  // Add this package and then dependencies'#010+
>>>>>>> origin/fixes_2_2
  '                  L.Add(S);'#010+
  '                  AddDependencyUnitPaths(L,P);'#010+
  '                end;'#010+
  '            end;'#010+
  '        end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TBuildEngine.GetCompilerCommand','(APackage : TPackage; ATarge'+
  't : TTarget) : String;'#010+
  'Var'#010+
  '  L : TUnsortedDuplicatesStringList;'#010+
  '  Args : TStringList;'#010+
  '  i : Integer;'#010+
  'begin'#010+
  '  if ATarget.TargetSourceFileName = '#039#039' then'#010+
  '    Error(SErrCouldNotCompile,[ATarget.Name, APackage.Name]);',#010+
  #010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TBuildEngine.GetCompile','rCommand(APackage : TPackage; ATarge'+
  't : TTarget) : String;'#010+
  'Var'#010+
  '  L,Args : TStringList;'#010+
  '  i : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Args:=TStringList.Create;'#010+
  '  Args.Duplicates:=dupIgnore;'#010+
  #010+
  '  Result := '#039#039';'#010+
  #010+
  '  //compiler configuration'#010+
  '  if Defaults.NoFPCCfg then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Args.Add('#039'-n'#039');'#010+
=======
  ' ','   Args.Add('#039'-n'#039');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ','   Args.Add('#039'-n'#039');'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '  // Target OS'#010+
  '  Args.Add('#039'-T'#039'+OSToString(Defaults.OS));'#010+
  #010+
  '  // Compile mode'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If ATarg','et.Mode<>cmFPC then'#010+
  '    Args.Add('#039'-M'#039'+ModeToString(ATarget.Mode))'#010+
  '  else If Defaults.Mode<>cmFPC then'#010+
  '    Args.Add('#039'-M'#039'+ModeToString(Defaults.Mode));'#010+
  '  // Output file paths'#010+
  '  If ATarget.TargetType in ProgramTargets then'#010+
  '    Args.Add('#039'-FE'#039'+A','Package.GetBinOutputDir(Defaults.CPU,Def'+
  'aults.OS));'#010+
  '  Args.Add('#039'-FU'#039'+APackage.GetUnitsOutputDir(Defaults.CPU,Defau'+
  'lts.OS));'#010+
  '  // Object Path'#010+
  '  L:=TUnsortedDuplicatesStringList.Create;'#010+
  '  L.Duplicates:=dupIgnore;'#010+
  '  AddConditionalStrings(L,APa','ckage.ObjectPath,Defaults.CPU,Defaults.'+
  'OS);'#010+
  '  AddConditionalStrings(L,ATarget.ObjectPath,Defaults.CPU,Defaults.OS)'+
  ';'#010+
  '  for i:=0 to L.Count-1 do'#010+
  '    Args.Add('#039'-Fo'#039'+L[i]);'#010+
  '  FreeAndNil(L);'#010+
  '  // Unit Dirs'#010+
  '  L:=TUnsortedDuplicatesStringList.Crea','te;'#010+
  '  L.Duplicates:=dupIgnore;'#010+
  '  AddDependencyUnitPaths(L,APackage);'#010+
  '  AddConditionalStrings(L,APackage.UnitPath,Defaults.CPU,Defaults.OS);'+
  #010+
  '  AddConditionalStrings(L,ATarget.UnitPath,Defaults.CPU,Defaults.OS);'#010+
  '  for i:=0 to L.Count-1 do'#010+
  '   ',' Args.Add('#039'-Fu'#039'+L[i]);'#010+
  '  FreeAndNil(L);'#010+
  '  // Include Path'#010+
  '  L:=TUnsortedDuplicatesStringList.Create;'#010+
  '  L.Duplicates:=dupIgnore;'#010+
  '  AddDependencyIncludePaths(L,ATarget);'#010+
  '  AddConditionalStrings(L,APackage.IncludePath,Defaults.CPU,Defaults.O'+
  'S)',';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  If ATarget.Mode<>cmFPC then'#010+
  '    Args.Add('#039'-M'#039'+ModeToString(ATarget.Mode))'#010+
  '  else If Defaults.Mode<>cmFPC then'#010+
  '    Args.Add('#039'-M'#039'+ModeToString(D','efaults.Mode));'#010+
  '  // Output file paths'#010+
  '  If ATarget.TargetType in ProgramTargets then'#010+
  '    Args.Add('#039'-FE'#039'+APackage.GetBinOutputDir(Defaults.CPU,Defau'+
  'lts.OS));'#010+
  '  Args.Add('#039'-FU'#039'+APackage.GetUnitsOutputDir(Defaults.CPU,Defau'+
  'lts.OS));'#010+
  '  // Objec','t Path'#010+
  '  L:=TStringList.Create;'#010+
  '  L.Sorted:=true;'#010+
  '  L.Duplicates:=dupIgnore;'#010+
  '  AddConditionalStrings(L,APackage.ObjectPath,Defaults.CPU,Defaults.OS'+
  ');'#010+
  '  AddConditionalStrings(L,ATarget.ObjectPath,Defaults.CPU,Defaults.OS)'+
  ';'#010+
  '  for i:=0 to L.Co','unt-1 do'#010+
  '    Args.Add('#039'-Fo'#039'+L[i]);'#010+
  '  FreeAndNil(L);'#010+
  '  // Unit Dirs'#010+
  '  L:=TStringList.Create;'#010+
  '  L.Sorted:=true;'#010+
  '  L.Duplicates:=dupIgnore;'#010+
  '  AddDependencyUnitPaths(L,APackage);'#010+
  '  AddConditionalStrings(L,APackage.UnitPath,Defaults.CPU,Defaults','.OS'+
  ');'#010+
  '  AddConditionalStrings(L,ATarget.UnitPath,Defaults.CPU,Defaults.OS);'#010+
  '  for i:=0 to L.Count-1 do'#010+
  '    Args.Add('#039'-Fu'#039'+L[i]);'#010+
  '  FreeAndNil(L);'#010+
  '  // Include Path'#010+
  '  L:=TStringList.Create;'#010+
  '  L.Sorted:=true;'#010+
  '  L.Duplicates:=dupIgnore;'#010+
  '  AddD','ependencyIncludePaths(L,ATarget);'#010+
  '  AddConditionalStrings(L,APackage.IncludePath,Defaults.CPU,Defaults.O'+
  'S);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  AddConditionalStrings(L,ATarget.IncludePath,Defaults.CPU,Defaults.OS'+
  ');'#010+
  '  for i:=0 to L.Count-1 do'#010+
  '    Args.Add('#039'-Fi'#039'+L[i]);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FreeAndNil(L);'#010+
  '  // Custom Options'#010+
  '  If (Defaults.HaveOptions) then'#010+
  '    Args.AddStrings(Defaults.Options);'#010+
  '  I','f (APackage.HaveOptions) then'#010+
  '    Args.AddStrings(APackage.Options);'#010+
  '  If (ATarget.HaveOptions) then'#010+
  '    Args.AddStrings(ATarget.Options);'#010+
  '  // Add Filename to compile'#010+
  '  Args.Add(ATarget.TargetSourceFileName);'#010+
  '  // Convert to string'#010+
  '  Resul','t:='#039#039';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Free','AndNil(L);'#010+
  '  // Custom Options'#010+
  '  If (Defaults.Options<>'#039#039') then'#010+
  '    Args.Add(Defaults.Options);'#010+
  '  If (APackage.Options<>'#039#039') then'#010+
  '    Args.Add(APackage.Options);'#010+
  '  If (ATarget.Options<>'#039#039') then'#010+
  '    Args.Add(ATarget.Options);'#010+
  '  // Add Filenam','e to compile'#010+
  '  Args.Add(ATarget.TargetSourceFileName);'#010+
  '  // Convert to string'#010+
  '  Result:='#039#039';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  for i:=0 to Args.Count-1 do'#010+
  '    Result:=Result+'#039' '#039'+maybequoted(Args[i]);'#010+
  '  Delete(result,1,1);'#010+
  '  Args.Free;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TBuildEngine.GetCompiler : String;'#010+
=======
  'Function TBuildEngine.GetCompil','er : String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TBuildEngine.GetCompil','er : String;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  S : String;'#010+
  'begin'#010+
  '  // Cache in FCompiler for speed.'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If (FCompil','er='#039#039') then'#010+
=======
  '  If (FCompiler='#039#039') then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If (FCompiler='#039#039') then'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '    FCompiler:=Defaults.Compiler;'#010+
  '    If (ExtractFilePath(FCompiler)='#039#039') then'#010+
  '      begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      S:=ExeSearch(FCompiler,GetEnvironmentVariable('#039'PATH'#039'));'#010+
=======
  '      S:=ExeSearch(FCompiler,GetEnvironmen','tVariable('#039'PATH'#039')'+
  ');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      S:=ExeSearch(FCompiler,GetEnvironmen','tVariable('#039'PATH'#039')'+
  ');'#010+
>>>>>>> origin/fixes_2_2
  '      If (S<>'#039#039') then'#010+
  '         FCompiler:=S;'#010+
  '      end;'#010+
  '    end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ',' Result:=FCompiler;'#010+
=======
  '  Result:=FCompiler;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result:=FCompiler;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CreateOutputDir(APackage: TPackage);'#010+
  'Var'#010+
  '  D : String;'#010+
  '  i: integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  //create a units directory'#010+
  '  D:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '  If not SysDirectoryExists','(D) then'#010+
=======
  '  //create a units directory'#010,
  '  D:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '  If not SysDirectoryExists(D) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  //create a units directory'#010,
  '  D:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '  If not SysDirectoryExists(D) then'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      Log(vlInfo,SInfoCreatingOutputDir,[D]);'#010+
  '      CmdCreateDir(D);'#010+
  '    end;'#010+
  #010+
  '  //also create a bin directory for programtargets'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  For i := 0 to Pred(APackage.Targets.Count) do'#010+
  '    begin'#010+
  '      if APackage.Targets.TargetI','tems[i].TargetType in ProgramTarget'+
  's then'#010+
  '        begin'#010+
  '          D:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '          If not SysDirectoryExists(D) then'#010+
  '            begin'#010+
  '              Log(vlInfo,SInfoCreatingOutputDir,[D]);'#010+
  '   ','           CmdCreateDir(D);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  For',' i := 0 to Pred(APackage.Targets.Count) do'#010+
  '    begin'#010+
  '      if APackage.Targets.TargetItems[i].TargetType in ProgramTargets '+
  'then'#010+
  '        begin'#010+
  '          D:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '          If not SysDirectoryExis','ts(D) then'#010+
  '            begin'#010+
  '              Log(vlInfo,SInfoCreatingOutputDir,[D]);'#010+
  '              CmdCreateDir(D);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '            end;'#010+
  '          //do not continue loop, directory is made anyway'#010+
  '          break;'#010+
  '        end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TBuildEngine.DependencyOK(ADependency : TDependency) : Boolea'+
  'n;'#010+
  'begin'#010+
  '  Result:=(','Defaults.CPU in ADependency.CPUs) and (Defaults.OS in ADe'+
  'pendency.OSes);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Func','tion TBuildEngine.DependencyOK(ADependency : TDependency) : Bool'+
  'ean;'#010+
  'begin'#010+
  '  Result:=(Defaults.CPU in ADependency.CPUs) and (Defaults.OS in ADepe'+
  'ndency.OSes);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TBuildEngine.TargetOK(ATarget : TTarget) : Boolean;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=(Defaults.CPU in ATarget.CPUs) and (Defaults.OS in ATarget.O'+
  'Ses);'#010+
  'end;'#010+
  #010+
  #010+
  'Function T','BuildEngine.PackageOK(APackage : TPackage) : Boolean;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Resul','t:=(Defaults.CPU in ATarget.CPUs) and (Defaults.OS in ATarget'+
  '.OSes);'#010+
  'end;'#010+
  #010+
  #010+
  'Function TBuildEngine.PackageOK(APackage : TPackage) : Boolean;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=(Defaults.CPU in APackage.CPUs) and (Defaults.OS in APackage'+
  '.OSes);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.DoBeforeCompile(APackage: TPackage);'#010+
  'begin'#010+
  '  ExecuteCommands(APackage.Co','mmands,caBeforeCompile);'#010+
=======
  'procedure',' TBuildEngine.DoBeforeCompile(APackage: TPackage);'#010+
  'begin'#010+
  '  ExecuteCommands(APackage.Commands,caBeforeCompile);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure',' TBuildEngine.DoBeforeCompile(APackage: TPackage);'#010+
  'begin'#010+
  '  ExecuteCommands(APackage.Commands,caBeforeCompile);'#010+
>>>>>>> origin/fixes_2_2
  '  If Assigned(APackage.BeforeCompile) then'#010+
  '    APackage.BeforeCompile(APackage);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.DoAfterCompile(APackage: TPackage);'#010+
  'begin'#010+
  '  If Assigned(APackage.AfterCompile) then'#010+
  '    APackage.AfterCo','mpile(APackage);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TBuildEngine.DoAfterCompile(APa','ckage: TPackage);'#010+
  'begin'#010+
  '  If Assigned(APackage.AfterCompile) then'#010+
  '    APackage.AfterCompile(APackage);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  ExecuteCommands(APackage.Commands,caAfterCompile);'#010+
  'end;'#010+
  #010+
  #010+
  'Function TBuildEngine.NeedsCompile(APackage:TPackage;ATarget: TTarget)'+
<<<<<<< HEAD
<<<<<<< HEAD
  ': Boolean;'#010+
=======
  ': Boole','an;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ': Boole','an;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  '  D : TDependency;'#010+
  '  T : TTarget;'#010+
  '  OD,OFN : String;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Re','sult:=False;'#010+
=======
  '  Result:=False;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Result:=False;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '  // Forced recompile?'#010+
  '  if FForceCompile then'#010+
  '    Result:=true;'#010+
  #010+
  '  // Check output file'#010+
  '  if not result then'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      if ATarget.TargetType in ProgramTargets then'#010+
  '        OD:=APackage.GetBinOutputDir(Defaults.CPU,Defau','lts.OS)'#010+
  '      else'#010+
  '        OD:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '      If (OD<>'#039#039') then'#010+
  '        OD:=IncludeTrailingPathDelimiter(OD);'#010+
  '      OFN:=OD+ATarget.GetOutPutFileName(Defaults.OS);'#010+
  '      Result:=Not FileExists(OFN',');'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      if ATarget.Tar','getType in ProgramTargets then'#010+
  '        OD:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS)'#010+
  '      else'#010+
  '        OD:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);'#010+
  '      If (OD<>'#039#039') then'#010+
  '        OD:=IncludeTrailingPathDelimiter(OD);'#010,
  '      OFN:=OD+ATarget.GetOutPutFileName(Defaults.OS);'#010+
  '      Result:=Not FileExists(OFN);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '      if Result then'#010+
  '        Log(vlDebug,SDbgOutputNotYetAvailable,[OFN]);'#010+
  '    end;'#010+
  #010+
  '  // Check main source'#010+
  '  If not Result then'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      if FileExists(ATarget.TargetSourceFileName) then'#010+
  '        Result:=FileNewer(ATarget.TargetSou','rceFileName,OFN)'#010+
=======
  '      if Fil','eExists(ATarget.TargetSourceFileName) then'#010+
  '        Result:=FileNewer(ATarget.TargetSourceFileName,OFN)'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      if Fil','eExists(ATarget.TargetSourceFileName) then'#010+
  '        Result:=FileNewer(ATarget.TargetSourceFileName,OFN)'#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  #010+
  '  // Check unit and include dependencies'#010+
  '  If not Result then'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      ResolveDependencies(ATarget.Dependencies,ATarget.Collection as T'+
  'Targets);'#010+
  '      I:=0;'#010+
  '      for i:=0 to ATarget.Dependencies.Count-1 do',#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '      ResolveDependencies(ATarget.Dependencies,ATarget.','Collection as'+
  ' TTargets);'#010+
  '      I:=0;'#010+
  '      for i:=0 to ATarget.Dependencies.Count-1 do'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '        begin'#010+
  '          D:=ATarget.Dependencies[i];'#010+
  '          if (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then'+
  #010+
  '            begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '              case D.DependencyType of'#010+
  '                depUnit :'#010+
  '                  begin'#010+
  '         ','           T:=TTarget(D.Target);'#010+
  '                    If (T=Nil) then'#010+
  '                      Error(SErrDepUnknownTarget,[D.Value, ATarget.Nam'+
  'e, APackage.Name]);'#010+
  '                    // If a dependent package is compiled we always ne'+
  'ed to recom','pile'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '            ','  case D.DependencyType of'#010+
  '                depUnit :'#010+
  '                  begin'#010+
  '                    T:=TTarget(D.Target);'#010+
  '                    If (T=Nil) then'#010+
  '                      Error(SErrDepUnknownTarget,[ATarget.Name,D.Value'+
  ']);'#010+
  '           ','         // If a dependent package is compiled we always '+
  'need to recompile'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '                    Log(vldebug, SDbgDependencyOnUnit, [ATarget.Name,T'+
  '.Name]);'#010+
  '                    Result:=(T.State=tsCompiled);'#010+
  '                    if Result then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                      Log(vldebug, SDbgDependencyUnitRecompiled, [T.Na'+
  'm','e]);'#010+
=======
  ' ','                     Log(vldebug, SDbgDependencyUnitRecompiled, [T.'+
  'Name]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ','                     Log(vldebug, SDbgDependencyUnitRecompiled, [T.'+
  'Name]);'#010+
>>>>>>> origin/fixes_2_2
  '                  end;'#010+
  '                depInclude :'#010+
  '                  begin'#010+
  '                    if D.TargetFileName<>'#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                      Result:=FileNewer(D.TargetFileName,OFN)'#010+
  '                  end;'#010+
  '                depPackag','e :'#010+
  '                  begin'#010+
  '                    log(vlWarning,SWarnTargetDependsOnPackage,[ATarget'+
  '.Name, APackage.Name, d.Value]);'#010+
=======
  '                      Result:=FileNewer(','D.TargetFileName,OFN)'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '                      Result:=FileNewer(','D.TargetFileName,OFN)'#010+
>>>>>>> origin/fixes_2_2
  '                  end;'#010+
  '              end;'#010+
  '              if result then'#010+
  '                break;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            end',';'#010+
=======
  '            end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '            end;'#010+
>>>>>>> origin/fixes_2_2
  '        end;'#010+
  '    end;'#010+
  #010+
  '  if result then'#010+
  '    Log(vlDebug,SDbgMustCompile,[ATarget.Name]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.Compile(APackage: TPackage; ATarget: TTarget);'#010+
  'Var'#010+
  '  S : String;'#010+
  'begin'#010+
  '  Log(vlInfo,SInfoCompilingTarget,[ATarget.Name]',');'#010+
  '  LogIndent;'#010+
  '  ExecuteCommands(ATarget.Commands,caBeforeCompile);'#010+
  '  If Assigned(ATarget.BeforeCompile) then'#010+
  '    ATarget.BeforeCompile(ATarget);'#010+
  '  S:=GetCompilerCommand(APackage,ATarget);'#010+
  '  ExecuteCommand(GetCompiler,S);'#010+
  '  If Assigned(ATa','rget.AfterCompile) then'#010+
  '    ATarget.AfterCompile(ATarget);'#010+
  '  ExecuteCommands(ATarget.Commands,caAfterCompile);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure T','BuildEngine.Compile(APackage: TPackage; ATarget: TTarget)'+
  ';'#010+
  'Var'#010+
  '  S : String;'#010+
  'begin'#010+
  '  Log(vlInfo,SInfoCompilingTarget,[ATarget.Name]);'#010+
  '  LogIndent;'#010+
  '  ExecuteCommands(ATarget.Commands,caBeforeCompile);'#010+
  '  If Assigned(ATarget.BeforeCompile) the','n'#010+
  '    ATarget.BeforeCompile(ATarget);'#010+
  '  S:=GetCompilerCommand(APackage,ATarget);'#010+
  '  ExecuteCommand(GetCompiler,S);'#010+
  '  If Assigned(ATarget.AfterCompile) then'#010+
  '    ATarget.AfterCompile(ATarget);'#010+
  '  ExecuteCommands(ATarget.Commands,caAfterCompile)',';'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  LogUnIndent;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CompileDependencies(APackage:TPackage; ATarget:'+
  ' TTarget);'#010+
  'Var'#010+
  '  I : Integer;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  T : T','Target;'#010+
=======
  '  T : TTarget;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  T : TTarget;'#010+
>>>>>>> origin/fixes_2_2
  '  D : TDependency;'#010+
  'begin'#010+
  '  Log(vlDebug, Format(SDbgCompilingDependenciesOfTarget, [ATarget.Name'+
  ']));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  LogIndent;'#010+
  '  For I:=0 to ATarget.Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=ATarget.Dependencies[i];'#010+
  '      if (D.DependencyType=de','pPackage) then'#010+
  '        log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APack'+
  'age.Name, d.Value])'#010+
  '      else if (D.DependencyType=depUnit) and'#010+
  '         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then'#010+
  '        begin'#010+
  '         ',' T:=TTarget(D.Target);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  ' ',' LogIndent;'#010+
  '  For I:=0 to ATarget.Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=ATarget.Dependencies[i];'#010+
  '      if (D.DependencyType=depUnit) and'#010+
  '         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then'#010+
  '        begin'#010+
  '          T:=TTar','get(D.Target);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '          if Assigned(T) and (T<>ATarget) then'#010+
  '            begin'#010+
  '              if TargetOK(T) then'#010+
  '                begin'#010+
  '                  // We don'#039't need to compile implicit units, they'+
  ' are only'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                  //',' used for dependency checking'#010+
=======
  '                  // used fo','r dependency checking'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '                  // used fo','r dependency checking'#010+
>>>>>>> origin/fixes_2_2
  '                  if (T.TargetType<>ttImplicitUnit) then'#010+
  '                    begin'#010+
  '                      case T.State of'#010+
  '                        tsNeutral :'#010+
  '                          MaybeCompile(APackage,T);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ','                       tsConsidering :'#010+
=======
  '         ','               tsConsidering :'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '         ','               tsConsidering :'#010+
>>>>>>> origin/fixes_2_2
  '                          Log(vlWarning,SWarnCircularTargetDependency,'+
  '[ATarget.Name,T.Name]);'#010+
  '                      end;'#010+
  '                    end;'#010+
  '                end'#010+
  '              else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                ','Log(vlWarning, Format(SWarnDepUnitNotFound, [T.Name,'+
  ' MakeTargetString(Defaults.CPU,Defaults.OS)]));'#010+
  '            end'#010+
  '          else'#010+
  '            Error(SErrDepUnknownTarget,[D.Value, ATarget.Name, APackag'+
  'e.Name]);'#010+
  '        end;'#010+
  '    end;'#010+
  '  LogUn','Indent;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.MaybeCompile(APackage: TPackage; ATarget: TTarg'+
  'et);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '                Log(vlWa','rning, Format(SWarnDepUnitNotFound, [T.Name,'+
  ' MakeTargetString(Defaults.CPU,Defaults.OS)]));'#010+
  '            end'#010+
  '          else'#010+
  '            Error(SErrDepUnknownTarget,[ATarget.Name,D.Value]);'#010+
  '        end;'#010+
  '    end;'#010+
  '  LogUnIndent;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure',' TBuildEngine.MaybeCompile(APackage: TPackage; ATarget: TTa'+
  'rget);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  if ATarget.State<>tsNeutral then'#010+
  '    Error(SErrInvalidState,[ATarget.Name]);'#010+
  '  Log(vlDebug, Format(SDbgConsideringTarget, [ATarget.Name]));'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ','LogIndent;'#010+
  '  ATarget.FTargetState:=tsConsidering;'#010+
=======
  '  LogIndent;'#010+
  '  ATarget.FTa','rgetState:=tsConsidering;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  LogIndent;'#010+
  '  ATarget.FTa','rgetState:=tsConsidering;'#010+
>>>>>>> origin/fixes_2_2
  '  ResolveDependencies(ATarget.Dependencies,ATarget.Collection as TTarg'+
  'ets);'#010+
  '  CompileDependencies(APackage, ATarget);'#010+
  '  if NeedsCompile(APackage, ATarget) then'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Compile(APacka','ge,ATarget);'#010+
  '      ATarget.FTargetState:=tsCompiled;'#010+
=======
  '      Compile(APackage,ATarget);'#010+
  '      ATarg','et.FTargetState:=tsCompiled;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Compile(APackage,ATarget);'#010+
  '      ATarg','et.FTargetState:=tsCompiled;'#010+
>>>>>>> origin/fixes_2_2
  '    end'#010+
  '  else'#010+
  '    ATarget.FTargetState:=tsNoCompile;'#010+
  '  LogUnIndent;'#010+
  'end;'#010+
  #010+
  #010+
  'function TBuildEngine.NeedsCompile(APackage: TPackage): Boolean;'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  D : TDepe','ndency;'#010+
  'begin'#010+
  '  Result:=False;'#010+
=======
  '  D : TDependency;'#010+
  'begin'#010+
  '  Result:=','False;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  D : TDependency;'#010+
  'begin'#010+
  '  Result:=','False;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '  // Forced recompile?'#010+
  '  if FForceCompile then'#010+
  '    Result:=true;'#010+
  #010+
  '  // Recompile because of Package Dependencies?'#010+
  '  if not Result then'#010+
  '    begin'#010+
  '       I:=0;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '       For I:=0 to APackage.Dependencies.Count-1 d','o'#010+
  '         begin'#010+
  '           D:=APackage.Dependencies[i];'#010+
=======
  '       For I:=0 to APackage.Dependencies.Count-1 do'#010+
  '         begin'#010+
  '       ','    D:=APackage.Dependencies[i];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '       For I:=0 to APackage.Dependencies.Count-1 do'#010+
  '         begin'#010+
  '       ','    D:=APackage.Dependencies[i];'#010+
>>>>>>> origin/fixes_2_2
  '           if (D.DependencyType=depPackage) and'#010+
  '              (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) the'+
  'n'#010+
  '             begin'#010+
  '               P:=TPackage(D.Target);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      ','         if Assigned(P) then'#010+
=======
  '               if Assigned(P) ','then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '               if Assigned(P) ','then'#010+
>>>>>>> origin/fixes_2_2
  '                 begin'#010+
  '                   Result:=(P.State=tsCompiled);'#010+
  '                   if Result then'#010+
  '                     break;'#010+
  '                 end;'#010+
  '             end;'#010+
  '         end;'#010+
  '    end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  // Recompil','e a Target of this package?'#010+
=======
  '  // Recompile a Target of this packa','ge?'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  // Recompile a Target of this packa','ge?'#010+
>>>>>>> origin/fixes_2_2
  '  If Not Result then'#010+
  '    begin'#010+
  '      try'#010+
  '        If (APackage.Directory<>'#039#039') then'#010+
  '          EnterDir(APackage.Directory);'#010+
  '        for i:=0 to APackage.Targets.Count-1 do'#010+
  '          begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '            Result:=NeedsC','ompile(APackage,APackage.Targets.TargetIte'+
=======
  '            Result:=NeedsCompile(APackage,APackage','.Targets.TargetIte'+
>>>>>>> graemeg/fixes_2_2
=======
  '            Result:=NeedsCompile(APackage,APackage','.Targets.TargetIte'+
>>>>>>> origin/fixes_2_2
  'ms[i]);'#010+
  '            if Result then'#010+
  '              break;'#010+
  '          end;'#010+
  '      Finally'#010+
  '        If (APackage.Directory<>'#039#039') then'#010+
  '          EnterDir('#039#039');'#010+
  '      end;'#010+
  '    end;'#010+
  #010+
  '  if result then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Log(vl','Debug,SDbgMustCompile,[APackage.Name]);'#010+
=======
  '    Log(vlDebug,SDbgMustCompile,[A','Package.Name]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Log(vlDebug,SDbgMustCompile,[A','Package.Name]);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'function TBuildEngine.CheckExternalPackage(Const APackageName : String'+
  '):TPackage;'#010+
  'var'#010+
  '  S : String;'#010+
  '  I : Integer;'#010+
  'begin'#010+
  '  // Already checked?'#010+
  '  I:=ExternalPackages.IndexOfName(APackageName);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ',' if I<>-1 then'#010+
  '    begin'#010+
=======
  '  if I<>-1 then'#010+
  '    begin',#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  if I<>-1 then'#010+
  '    begin',#010+
>>>>>>> origin/fixes_2_2
  '      result:=ExternalPackages.PackageItems[I];'#010+
  '      exit;'#010+
  '    end;'#010+
  '  // Create new external package'#010+
  '  Result:=ExternalPackages.AddPackage(APackageName);'#010+
  '  Result.FTargetState:=tsNotFound;'#010+
  '  // Load unit config'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  S',':=GetUnitDir(Result);'#010+
  '  if S<>'#039#039' then'#010+
=======
  '  S:=GetUnitDir(Result);'#010+
  '  ','if S<>'#039#039' then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  S:=GetUnitDir(Result);'#010+
  '  ','if S<>'#039#039' then'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      Log(vldebug, SDbgExternalDependency, [APackageName,S]);'#010+
  '      Result.FTargetState:=tsInstalled;'#010+
  '      // Load unit config if it exists'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      S:=IncludeTrailingPathDelimiter(S)+UnitConfig','File;'#010+
  '      if FileExists(S) then'#010+
=======
  '      S:=IncludeTrailingPathDelimiter(S)+UnitConfigFile;'#010+
  '      if FileExist','s(S) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      S:=IncludeTrailingPathDelimiter(S)+UnitConfigFile;'#010+
  '      if FileExist','s(S) then'#010+
>>>>>>> origin/fixes_2_2
  '        begin'#010+
  '          Log(vlDebug, Format(SDbgLoading, [S]));'#010+
  '          Result.LoadUnitConfigFromFile(S);'#010+
  '        end;'#010+
  '      // Check recursive implicit dependencies'#010+
  '      CompileDependencies(Result);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '   ',' end'#010+
  '  else'#010+
  '    Error(SErrDependencyNotFound,[APackageName]);'#010+
=======
  '    end'#010+
  '  else'#010+
  '    Error(SE','rrDependencyNotFound,[APackageName]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    end'#010+
  '  else'#010+
  '    Error(SE','rrDependencyNotFound,[APackageName]);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.CompileDependencies(APackage: TPackage);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
  '  D : TDependency;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  For I:=0 to APackage.Dependencies.Count-1 do'#010,
  '    begin'#010+
  '      D:=APackage.Dependencies[i];'#010+
=======
  '  For I:=0 to APackage.Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=APack','age.Dependencies[i];'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  For I:=0 to APackage.Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=APack','age.Dependencies[i];'#010+
>>>>>>> origin/fixes_2_2
  '      if (D.DependencyType=depPackage) and'#010+
  '         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then'#010+
  '        begin'#010+
  '          P:=TPackage(D.Target);'#010+
  '          If Assigned(P) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        ','    begin'#010+
  '              if (Defaults.CPU in P.CPUs) and (Defaults.OS in P.OSes) '+
  'then'#010+
  '                begin'#010+
  '                  case P.State of'#010+
  '                    tsNeutral :'#010+
  '                      MaybeCompile(P);'#010+
  '                    tsConsid','ering :'#010+
  '                      Log(vlWarning,SWarnCircularPackageDependency,[AP'+
  'ackage.Name,P.Name]);'#010+
  '                  end;'#010+
  '                end'#010+
  '              else'#010+
  '                Log(vlWarning,SWarnDependOnOtherPlatformPackage,[APack'+
  'age.Name',', D.Value, MakeTargetString(Defaults.CPU, Defaults.OS)]);'#010+
  '            end'#010+
  '          else'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '            begin'#010+
  '              ','case P.State of'#010+
  '                tsNeutral :'#010+
  '                  MaybeCompile(P);'#010+
  '                tsConsidering :'#010+
  '                  Log(vlWarning,SWarnCircularPackageDependency,[APacka'+
  'ge.Name,P.Name]);'#010+
  '              end;'#010+
  '            end'#010+
  '      ','    else'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '            begin'#010+
  '              D.Target:=CheckExternalPackage(D.Value);'#010+
  '              P:=TPackage(D.Target);'#010+
  '            end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          if (D.RequireC','hecksum<>$ffffffff) and'#010+
  '             (P.InstalledChecksum<>$ffffffff) and'#010+
  '             (P.InstalledChecksum<>D.RequireChecksum) then'#010+
=======
  '          if (D.RequireChecksum<>$ffffffff) and'#010+
  '             (P.InstalledChecksum<>$ffffffff) and'#010+
  '      ','       (P.InstalledChecksum<>D.RequireChecksum) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '          if (D.RequireChecksum<>$ffffffff) and'#010+
  '             (P.InstalledChecksum<>$ffffffff) and'#010+
  '      ','       (P.InstalledChecksum<>D.RequireChecksum) then'#010+
>>>>>>> origin/fixes_2_2
  '            Log(vlDebug,SDbgPackageChecksumChanged,[P.Name]);'#010+
  '        end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuild','Engine.Compile(APackage: TPackage);'#010+
=======
  'procedure TBuildEngine.Compile(APackage: TPackage);'#010+
>>>>>>> graemeg/fixes_2_2
  'Var'#010+
  '  T : TTarget;'#010+
  '  I : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
  '  Try'#010+
  '    Log(vlInfo,SInfoCompilingPackage,[APackage.Name]);'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir(APackage.Directory);'#010+
  '    CreateOutputDir(APackage);',#010+
  '    Dictionary.AddVariable('#039'UNITSOUTPUTDIR'#039',APackage.GetUnitsO'+
  'utputDir(Defaults.CPU,Defaults.OS));'#010+
  '    Dictionary.AddVariable('#039'BINOUTPUTDIR'#039',APackage.GetBinOutpu'+
  'tDir(Defaults.CPU,Defaults.OS));'#010+
  '    DoBeforeCompile(APackage);'#010+
  '    For I:=0 t','o APackage.Targets.Count-1 do'#010+
  '      begin'#010+
  '        T:=APackage.Targets.TargetItems[i];'#010+
=======
=======
  'procedure TBuildEngine.Compile(APackage: TPackage);'#010+
  'Var'#010+
  '  T : TTarget;'#010+
  '  I : Integer;'#010+
  'begin'#010+
>>>>>>> origin/fixes_2_2
  '  Tr','y'#010+
  '    Log(vlInfo,SInfoCompilingPackage,[APackage.Name]);'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir(APackage.Directory);'#010+
  '    CreateOutputDir(APackage);'#010+
  '    Dictionary.AddVariable('#039'UNITSOUTPUTDIR'#039',APackage.GetUnitsO'+
  'utputDir(Defaults','.CPU,Defaults.OS));'#010+
  '    Dictionary.AddVariable('#039'BINOUTPUTDIR'#039',APackage.GetBinOutpu'+
  'tDir(Defaults.CPU,Defaults.OS));'#010+
  '    DoBeforeCompile(APackage);'#010+
  '    For I:=0 to APackage.Targets.Count-1 do'#010+
  '      begin'#010+
  '        T:=APackage.Targets.TargetItem','s[i];'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '        if (T.TargetType in [ttUnit,ttProgram]) then'#010+
  '          begin'#010+
  '            if TargetOK(T) then'#010+
  '              begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                if T.State=tsNeut','ral then'#010+
  '                  MaybeCompile(APackage,T);'#010+
  '              end'#010+
  '            else'#010+
  '              begin'#010+
  '                if not(Defaults.CPU in T.CPUs) then'#010+
  '                  Log(vldebug, Format(SDbgSkippingTargetWrongCPU, [T.N'+
  'ame, CPUsT','oString(T.CPUs)]));'#010+
  '                if not(Defaults.OS in T.OSes) then'#010+
  '                  Log(vldebug, Format(SDbgSkippingTargetWrongOS, [T.Na'+
  'me, OSesToString(T.OSes)]));'#010+
  '              end;'#010+
  '          end'#010+
  '        else'#010+
  '          log(vldebug, S','DbgTargetIsNotAUnitOrProgram,[T.Name]);'#010+
  '      end;'#010+
  '    DoAfterCompile(APackage);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '                if T.State=tsNeutral then'#010+
  '                  MaybeCompile(APackage,T);'#010+
  '              end'#010+
  '         ','   else'#010+
  '              begin'#010+
  '                if not(Defaults.CPU in T.CPUs) then'#010+
  '                  Log(vldebug, Format(SDbgSkippingTargetWrongCPU, [T.N'+
  'ame, CPUsToString(T.CPUs)]));'#010+
  '                if not(Defaults.OS in T.OSes) then'#010+
  '         ','         Log(vldebug, Format(SDbgSkippingTargetWrongOS, [T.'+
  'Name, OSesToString(T.OSes)]));'#010+
  '              end;'#010+
  '          end'#010+
  '        else'#010+
  '          log(vldebug, SDbgTargetIsNotAUnitOrProgram,[T.Name]);'#010+
  '      end;'#010+
  '    DoAfterCompile(APackage);',#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Finally'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir('#039#039');'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.MaybeCompile(APackage: TPackage);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if APackage.S','tate in [tsCompiled, tsNoCompile] then'#010+
  '    begin'#010+
  '      Log(vlInfo,SInfoPackageAlreadyProcessed,[APackage.Name]);'#010+
  '      Exit;'#010+
  '    end;'#010+
  '  if APackage.State<>tsNeutral then'#010+
  '    Error(SErrInvalidState,[APackage.Name]);'#010+
  '  Log(vlDebug,SDbgConside','ringPackage,[APackage.Name]);'#010+
=======
  '  if APackage.State<>tsNeutral then'#010+
  '    Error(SErrInvalidState,[APackage.Name]);'#010+
  '  Log(vlDebug,','SDbgConsideringPackage,[APackage.Name]);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  if APackage.State<>tsNeutral then'#010+
  '    Error(SErrInvalidState,[APackage.Name]);'#010+
  '  Log(vlDebug,','SDbgConsideringPackage,[APackage.Name]);'#010+
>>>>>>> origin/fixes_2_2
  '  LogIndent;'#010+
  '  APackage.FTargetState:=tsConsidering;'#010+
  '  ResolveDependencies(APackage.Dependencies,(APackage.Collection as TP'+
  'ackages));'#010+
  '  CompileDependencies(APackage);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  ResolveFileNames(APackage,Defaults.CPU,De','faults.OS);'#010+
=======
  '  ResolveFileNames(APackage,Defa','ults.CPU,Defaults.OS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  ResolveFileNames(APackage,Defa','ults.CPU,Defaults.OS);'#010+
>>>>>>> origin/fixes_2_2
  '  If NeedsCompile(APackage) then'#010+
  '    begin'#010+
  '      Compile(APackage);'#010+
  '      APackage.FTargetState:=tsCompiled;'#010+
  '    end'#010+
  '  else'#010+
  '    APackage.FTargetState:=tsNoCompile;'#010+
  '  LogUnIndent;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TBuildEngine.InstallPackageFiles(','APAckage : TPackage; tt : '+
=======
  'Function TBuildEngine.InstallPa','ckageFiles(APAckage : TPackage; tt : '+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TBuildEngine.InstallPa','ckageFiles(APAckage : TPackage; tt : '+
>>>>>>> origin/fixes_2_2
  'TTargetType; Const Dest : String):Boolean;'#010+
  'Var'#010+
  '  List : TStringList;'#010+
  'begin'#010+
  '  Result:=False;'#010+
  '  List:=TStringList.Create;'#010+
  '  Try'#010+
  '    APackage.GetInstallFiles(List,[tt],Defaults.CPU, Defaults.OS);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    if (List.Count>0)',' then'#010+
=======
  '    if (Li','st.Count>0) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    if (Li','st.Count>0) then'#010+
>>>>>>> origin/fixes_2_2
  '      begin'#010+
  '        Result:=True;'#010+
  '        CmdCopyFiles(List,Dest);'#010+
  '      end;'#010+
  '  Finally'#010+
  '    List.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TBuildEngine.InstallPackageSourceFiles(APAckage: TPackage; tt'+
  ': TSourceType; const Dest: String): Boolean;'#010+
  'Var',#010+
  '  List : TStringList;'#010+
  'begin'#010+
  '  Result:=False;'#010+
  '  List:=TStringList.Create;'#010+
  '  Try'#010+
  '    APackage.GetInstallSourceFiles(List,[tt]);'#010+
  '    if (List.Count>0) then'#010+
  '      begin'#010+
  '        Result:=True;'#010+
  '        CmdCopyFiles(List,Dest);'#010+
  '      end;'#010+
  '  Finall','y'#010+
  '    List.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.DoBeforeInstall(APackage: TPackage);'#010+
  'begin'#010+
  '  ExecuteCommands(APackage.Commands,caBeforeInstall);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  #010+
  'procedure TBuildEngine.DoBeforeInstall(APackage: TPackage);'#010+
  'begin'#010+
  '  ExecuteCommands(APackage.Commands,caBe','foreInstall);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  If Assigned(APackage.BeforeInstall) then'#010+
  '    APackage.BeforeInstall(APackage);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'p','rocedure TBuildEngine.DoAfterInstall(APackage: TPackage);'#010+
  'begin'#010+
  '  If Assigned(APackage.AfterInstall) then'#010+
  '    APackage.AfterInstall(APackage);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TBuildEngine.DoAfterInstall(APackage: TPackage);'#010+
  'begin'#010+
  '  If Assigned(APackage.AfterInstall) then'#010+
  '    APackage.AfterInstall(APack','age);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  ExecuteCommands(APackage.Commands,caAfterInstall);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.Install(APacka','ge: TPackage);'#010+
=======
  'procedure TBuildEngine.Install(APackage: TPackage);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.Install(APackage: TPackage);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  UC,D : String;'#010+
  '  B : Boolean;'#010+
  'begin'#010+
  '  If (Apackage.State<>tsCompiled) then'#010+
  '    MaybeCompile(APackage);'#010+
  '  try'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Log(vlInfo,SInfoInstallingPackage,[APackage.Name]);'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir(AP','ackage.Directory);'#010+
  '    DoBeforeInstall(APackage);'#010+
  '    // units'#010+
  '    B:=false;'#010+
  '    D:=IncludeTrailingPathDelimiter(Defaults.UnitInstallDir)+APackage.'+
  'Name;'#010+
  '    if InstallPackageFiles(APAckage,ttUnit,D) then'#010+
  '      B:=true;'#010+
  '    if InstallPackage','Files(APAckage,ttImplicitUnit,D) then'#010+
  '      B:=true;'#010+
  '    // By default do not install the examples. Maybe add an option for'+
  ' this later'#010+
  '    //if InstallPackageFiles(APAckage,ttExampleUnit,D) then'#010+
  '    //  B:=true;'#010+
  '    // Unit (dependency) con','figuration if there were units installed'+
  #010+
  '    if B then'#010+
  '      begin'#010+
  '        UC:=IncludeTrailingPathDelimiter(D)+UnitConfigFile;'#010+
  '        Log(vlInfo, Format(SDbgGenerating, [UC]));'#010+
  '        APackage.SaveUnitConfigToFile(UC,Defaults.CPU,Defaults','.OS);'#010+
  '      end;'#010+
  '    // Programs'#010+
  '    D:=IncludeTrailingPathDelimiter(Defaults.BinInstallDir);'#010+
  '    InstallPackageFiles(APAckage,ttProgram,D);'#010+
  '    //InstallPackageFiles(APAckage,ttExampleProgram,D);'#010+
  '    // Documentation'#010+
  '    D:=IncludeTrailing','PathDelimiter(Defaults.DocInstallDir)+'#039'fpc'+
  '-'#039'+APackage.FileName+PathDelim;'#010+
  '    InstallPackageSourceFiles(APackage,stDoc,D);'#010+
  '    // Examples'#010+
  '    D:=IncludeTrailingPathDelimiter(Defaults.ExamplesInstallDir)+'#039'f'+
  'pc-'#039'+APackage.FileName+PathDelim;'#010,
  '    InstallPackageSourceFiles(APackage,stExample,D);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    Log','(vlInfo,SInfoInstallingPackage,[APackage.Name]);'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir(APackage.Directory);'#010+
  '    DoBeforeInstall(APackage);'#010+
  '    // units'#010+
  '    B:=false;'#010+
  '    D:=IncludeTrailingPathDelimiter(Defaults.UnitInstallDir)','+APackag'+
  'e.Name;'#010+
  '    if InstallPackageFiles(APAckage,ttUnit,D) then'#010+
  '      B:=true;'#010+
  '    if InstallPackageFiles(APAckage,ttImplicitUnit,D) then'#010+
  '      B:=true;'#010+
  '    // Unit (dependency) configuration if there were units installed'#010+
  '    if B then'#010+
  ' ','     begin'#010+
  '        UC:=IncludeTrailingPathDelimiter(D)+UnitConfigFile;'#010+
  '        Log(vlInfo, Format(SDbgGenerating, [UC]));'#010+
  '        APackage.SaveUnitConfigToFile(UC,Defaults.CPU,Defaults.OS);'#010+
  '      end;'#010+
  '    // Programs'#010+
  '    D:=IncludeTrailingP','athDelimiter(Defaults.BinInstallDir);'#010+
  '    InstallPackageFiles(APAckage,ttProgram,D);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    // Done.'#010+
  '    APackage.FTargetState:=tsInstalled;'#010+
  '    DoAfterInstall(APackage);'#010+
  '  Finally'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir('#039#039');'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.','DoBeforeArchive(APackage: TPackage);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  end',';'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.DoBeforeArchive(APackage: TPackage);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  ExecuteCommands(APackage.Commands,caBeforeArchive);'#010+
  '  If Assigned(APackage.BeforeArchive) then'#010+
  '    APackage.BeforeArchive(APackage);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.DoAfterArchive(APackage: TPackag','e);'#010+
=======
  'procedure TBuildEngine.','DoAfterArchive(APackage: TPackage);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBuildEngine.','DoAfterArchive(APackage: TPackage);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  If Assigned(APackage.AfterArchive) then'#010+
  '    APackage.AfterArchive(APackage);'#010+
  '  ExecuteCommands(APackage.Commands,caAfterArchive);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Archive(APackage: TPackage);'#010+
  'Var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  L : TStringList;'#010+
  '  A : String;'#010+
  '  ','i: integer;'#010+
=======
  '  L',' : TStringList;'#010+
  '  A : String;'#010+
  '  i: integer;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  L',' : TStringList;'#010+
  '  A : String;'#010+
  '  i: integer;'#010+
>>>>>>> origin/fixes_2_2
  '  ICPU : TCPU;'#010+
  '  IOS  : TOS;'#010+
  'begin'#010+
  '  A:=FStartDir+ APackage.FileName + ZipExt;'#010+
  '  Log(vlInfo,SInfoArchivingPackage,[APackage.Name,A]);'#010+
  '  try'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      EnterDir(APackage.Directory);'#010+
  '    DoBeforeArc','hive(Apackage);'#010+
=======
  '      EnterDir(APac','kage.Directory);'#010+
  '    DoBeforeArchive(Apackage);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      EnterDir(APac','kage.Directory);'#010+
  '    DoBeforeArchive(Apackage);'#010+
>>>>>>> origin/fixes_2_2
  '    L:=TStringList.Create;'#010+
  '    L.Sorted:=true;'#010+
  '    L.Duplicates:=dupIgnore;'#010+
  '    Try'#010+
  '      // Add fpmake.pp & manifest.xml always'#010+
  '      L.Add(FPMakePPFile);'#010+
  '      L.Add(ManifestFile);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      //get all files from all targets'#010+
  '  ','    for ICPU:=Low(TCPU) to high(TCPU) do'#010+
=======
  '      //g','et all files from all targets'#010+
  '      for ICPU:=Low(TCPU) to high(TCPU) do'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      //g','et all files from all targets'#010+
  '      for ICPU:=Low(TCPU) to high(TCPU) do'#010+
>>>>>>> origin/fixes_2_2
  '        for IOS:=Low(TOS) to high(TOS) do'#010+
  '          if OSCPUSupported[IOS,ICPU] then'#010+
  '            begin'#010+
  '              ResolveFileNames(APackage,ICPU,IOS,false);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '              APackage.GetArchiveFiles(','L, ICPU, IOS);'#010+
=======
  '       ','       APackage.GetArchiveFiles(L, ICPU, IOS);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '       ','       APackage.GetArchiveFiles(L, ICPU, IOS);'#010+
>>>>>>> origin/fixes_2_2
  '            end;'#010+
  '      //from sources'#010+
  '      for i := 0 to APackage.Sources.Count-1 do'#010+
  '        L.Add(APackage.Sources[i].Name);'#010+
  #010+
  '      //show all files'#010+
  '      for i := 0 to L.Count-1 do'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        Log(vlDebug, Format(SDbgArchiving','File, [L[i]]));'#010+
=======
  '        L','og(vlDebug, Format(SDbgArchivingFile, [L[i]]));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '        L','og(vlDebug, Format(SDbgArchivingFile, [L[i]]));'#010+
>>>>>>> origin/fixes_2_2
  #010+
  '{$ifdef HAS_UNIT_ZIPPER}'#010+
  '      if not Assigned(ArchiveFilesProc) then'#010+
  '        begin'#010+
  '          FZipFile := TZipper.Create;'#010+
  '          FZipFile.ZipFiles(A, L);'#010+
  '        end'#010+
  '      else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$endif HAS_UNIT_ZIPPER}'#010+
  '        CmdArchive','Files(L,A);'#010+
=======
  '{$endif HAS','_UNIT_ZIPPER}'#010+
  '        CmdArchiveFiles(L,A);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '{$endif HAS','_UNIT_ZIPPER}'#010+
  '        CmdArchiveFiles(L,A);'#010+
>>>>>>> origin/fixes_2_2
  '    Finally'#010+
  '      L.Free;'#010+
  #010+
  '{$ifdef HAS_UNIT_ZIPPER}'#010+
  '      if not Assigned(ArchiveFilesProc) then'#010+
  '        FreeAndNil(FZipFile);'#010+
  '{$endif HAS_UNIT_ZIPPER}'#010+
  '    end;'#010+
  '    DoAfterArchive(Apackage);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Finally'#010+
  '    If (APackage.Directory<','>'#039#039') then'#010+
=======
  '  Fin','ally'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Fin','ally'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
>>>>>>> origin/fixes_2_2
  '      EnterDir('#039#039');'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.DoBeforeClean(APackage: TPackage);'#010+
  'begin'#010+
  '  ExecuteCommands(APackage.Commands,caBeforeClean);'#010+
  '  If Assigned(APackage.BeforeClean) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    APackage.BeforeClean(APackage);'#010+
  'end',';'#010+
=======
  '    APa','ckage.BeforeClean(APackage);'#010+
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    APa','ckage.BeforeClean(APackage);'#010+
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'procedure TBuildEngine.DoAfterClean(APackage: TPackage);'#010+
  'begin'#010+
  '  If Assigned(APackage.AfterClean) then'#010+
  '    APackage.AfterClean(APackage);'#010+
  '  ExecuteCommands(APackage.Commands,caAfterClean);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.Clean(APackage: T','Package);'#010+
=======
  'procedur','e TBuildEngine.Clean(APackage: TPackage);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedur','e TBuildEngine.Clean(APackage: TPackage);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  List : TStringList;'#010+
  'begin'#010+
  '  Log(vlInfo,SInfoCleaningPackage,[APackage.Name]);'#010+
  '  try'#010+
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      EnterDir(APackage.Directory);'#010+
  '    DoBeforeClean(Apackage);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    List:=TStringList.Create;'#010+
  '    try'#010,
=======
  '    Li','st:=TStringList.Create;'#010+
  '    try'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Li','st:=TStringList.Create;'#010+
  '    try'#010+
>>>>>>> origin/fixes_2_2
  '      APackage.GetCleanFiles(List,Defaults.CPU,Defaults.OS);'#010+
  '      if (List.Count>0) then'#010+
  '        CmdDeleteFiles(List);'#010+
  '    Finally'#010+
  '      List.Free;'#010+
  '    end;'#010+
  '    DoAfterClean(Apackage);'#010+
  '  Finally'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    If (APackage.Directory<>'#039#039') then'#010+
  '      E','nterDir('#039#039');'#010+
=======
  '    If (APac','kage.Directory<>'#039#039') then'#010+
  '      EnterDir('#039#039');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    If (APac','kage.Directory<>'#039#039') then'#010+
  '      EnterDir('#039#039');'#010+
>>>>>>> origin/fixes_2_2
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'Procedure TBuildEngine.Manifest(APackage : TPackage);'#010+
  'Var'#010+
  '  L : TStrings;'#010+
  '  PD,'#010+
  '  MF : String;'#010+
  'begin'#010+
  '  L:=TStringList.Create;'#010+
  '  Try'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Log(vlInfo, Format(SInfoManifestPackage,[APackage.Name]));'#010+
  '    PD:=APackage.D','irectory;'#010+
=======
  '    Log(vlInfo, Format(SInfoManifestPackage,[APac','kage.Name]));'#010+
  '    PD:=APackage.Directory;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Log(vlInfo, Format(SInfoManifestPackage,[APac','kage.Name]));'#010+
  '    PD:=APackage.Directory;'#010+
>>>>>>> origin/fixes_2_2
  '    if PD<>'#039#039' then'#010+
  '      PD:=IncludeTrailingPathDelimiter(PD);'#010+
  '    MF:=PD+ManifestFile;'#010+
  '    Log(vlDebug, Format(SDbgGenerating, [MF]));'#010+
  '    L.Add('#039'<?xml version="1.0"?>'#039');'#010+
  '    L.Add('#039'<packages>'#039');'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    APackage.GetManifest(L);'#010+
  '    ','L.Add('#039'</packages>'#039');'#010+
=======
  ' ','   APackage.GetManifest(L);'#010+
  '    L.Add('#039'</packages>'#039');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ','   APackage.GetManifest(L);'#010+
  '    L.Add('#039'</packages>'#039');'#010+
>>>>>>> origin/fixes_2_2
  '    L.SaveToFile(MF);'#010+
  '  Finally'#010+
  '    L.Free;'#010+
  '  end;'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Compile(Packages: TPackages);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If Assigned(BeforeCompile) then'#010+
  '    BeforeCompile(Self);'#010+
  '  For I',':=0 to Packages.Count-1 do'#010+
=======
  '  If Assigned(BeforeCompile) then'#010,
  '    BeforeCompile(Self);'#010+
  '  For I:=0 to Packages.Count-1 do'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If Assigned(BeforeCompile) then'#010,
  '    BeforeCompile(Self);'#010+
  '  For I:=0 to Packages.Count-1 do'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      P:=Packages.PackageItems[i];'#010+
  '      If PackageOK(P) then'#010+
  '        MaybeCompile(P);'#010+
  '    end;'#010+
  '  If Assigned(AfterCompile) then'#010+
  '    AfterCompile(Self);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.Install(Packages: TP','ackages);'#010+
=======
  'procedure T','BuildEngine.Install(Packages: TPackages);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure T','BuildEngine.Install(Packages: TPackages);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
  'begin'#010+
  '  If Assigned(BeforeInstall) then'#010+
  '    BeforeInstall(Self);'#010+
  '  For I:=0 to Packages.Count-1 do'#010+
  '    begin'#010+
  '      P:=Packages.PackageItems[i];'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      If PackageOK(P) then'#010+
  '        Install(P);'#010+
  '    ','end;'#010+
=======
  '      If PackageOK(','P) then'#010+
  '        Install(P);'#010+
  '    end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      If PackageOK(','P) then'#010+
  '        Install(P);'#010+
  '    end;'#010+
>>>>>>> origin/fixes_2_2
  '  If Assigned(AfterInstall) then'#010+
  '    AfterInstall(Self);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Archive(Packages: TPackages);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
  'begin'#010+
  '  If Assigned(BeforeArchive) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    BeforeArchive(Self);'#010+
  '  Log(vlDebug, SDb','gBuildEngineArchiving);'#010+
=======
  '    BeforeA','rchive(Self);'#010+
  '  Log(vlDebug, SDbgBuildEngineArchiving);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    BeforeA','rchive(Self);'#010+
  '  Log(vlDebug, SDbgBuildEngineArchiving);'#010+
>>>>>>> origin/fixes_2_2
  '  For I:=0 to Packages.Count-1 do'#010+
  '    begin'#010+
  '      P:=Packages.PackageItems[i];'#010+
  '      Archive(P);'#010+
  '    end;'#010+
  '  If Assigned(AfterArchive) then'#010+
  '    AfterArchive(Self);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TBuildEngine.Manifest(Packages: TPac','kages);'#010+
=======
  'procedure TBui','ldEngine.Manifest(Packages: TPackages);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TBui','ldEngine.Manifest(Packages: TPackages);'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
  'begin'#010+
  '  If Assigned(BeforeManifest) then'#010+
  '    BeforeManifest(Self);'#010+
  '  Log(vlDebug, SDbgBuildEngineGenerateManifests);'#010+
  '  For I:=0 to Packages.Count-1 do'#010+
  '    begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      P:=Packages.PackageItems[i];'#010+
  ' ','     Manifest(P);'#010+
=======
  '    ','  P:=Packages.PackageItems[i];'#010+
  '      Manifest(P);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    ','  P:=Packages.PackageItems[i];'#010+
  '      Manifest(P);'#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  '  If Assigned(AfterManifest) then'#010+
  '    AfterManifest(Self);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TBuildEngine.Clean(Packages: TPackages);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  '  P : TPackage;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If Assigned(BeforeClean) then'#010+
  '    BeforeClean(Self);'#010,
=======
  '  If Assigned(BeforeCle','an) then'#010+
  '    BeforeClean(Self);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If Assigned(BeforeCle','an) then'#010+
  '    BeforeClean(Self);'#010+
>>>>>>> origin/fixes_2_2
  '  Log(vldebug, SDbgBuildEngineCleaning);'#010+
  '  For I:=0 to Packages.Count-1 do'#010+
  '    begin'#010+
  '    P:=Packages.PackageItems[i];'#010+
  '    If PackageOK(P) then'#010+
  '      Clean(P);'#010+
  '    end;'#010+
  '  If Assigned(AfterClean) then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    AfterClean(Self);'#010+
  'end;'#010+
  #010+
  #010+
  '{***********','********************************************************'+
  '*********'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    After','Clean(Self);'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '                               TFPVersion'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TFPVersion.GetAsString: String;'#010+
  'begin'#010+
  '  if E','mpty then'#010+
=======
  'function TFPVersion.G','etAsString: String;'#010+
  'begin'#010+
  '  if Empty then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TFPVersion.G','etAsString: String;'#010+
  'begin'#010+
  '  if Empty then'#010+
>>>>>>> origin/fixes_2_2
  '    Result:='#039'<none>'#039#010+
  '  else'#010+
  '    Result:=Format('#039'%d.%d.%d-%d'#039',[Major,Minor,Micro,Build]);'#010+
  'end;'#010+
  #010+
  'function TFPVersion.GetEmpty: Boolean;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=(Major=0) and (Minor=0) and (Micro=0) and (Build=0);'#010+
  'end;'#010+
  #010+
  'procedure TFPVersion.','SetAsString(const AValue: String);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result:=(Major=0) and (Minor=0) and (Micro=0) and (Build','=0);'#010+
  'end;'#010+
  #010+
  'procedure TFPVersion.SetAsString(const AValue: String);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  '  Function NextDigit(sep : Char; var V : string) : integer;'#010+
  #010+
  '  Var'#010+
  '    P : Integer;'#010+
  #010+
  '  begin'#010+
  '    P:=Pos(Sep,V);'#010+
  '    If (P=0) then'#010+
  '      P:=Length(V)+1;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    Result:=StrToIntDef(Copy(V,1,P-1),-1);'#010+
  '    If Re','sult<>-1 then'#010+
=======
  '    Result:=StrToInt','Def(Copy(V,1,P-1),-1);'#010+
  '    If Result<>-1 then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    Result:=StrToInt','Def(Copy(V,1,P-1),-1);'#010+
  '    If Result<>-1 then'#010+
>>>>>>> origin/fixes_2_2
  '      Delete(V,1,P)'#010+
  '    else'#010+
  '      Result:=0;'#010+
  '  end;'#010+
  #010+
  'Var'#010+
  '  V : String;'#010+
  'begin'#010+
  '  Clear;'#010+
  '  // Special support for empty version string'#010+
  '  if (AValue='#039#039') or (AValue='#039'<none>'#039') then'#010+
  '    exit;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  V:=AValue;'#010+
  '  Major:=NextDigit('#039'.'#039',V);'#010,
=======
  '  V:=AVa','lue;'#010+
  '  Major:=NextDigit('#039'.'#039',V);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  V:=AVa','lue;'#010+
  '  Major:=NextDigit('#039'.'#039',V);'#010+
>>>>>>> origin/fixes_2_2
  '  Minor:=NextDigit('#039'.'#039',V);'#010+
  '  Micro:=NextDigit('#039'-'#039',V);'#010+
  '  Build:=NextDigit(#0,V);'#010+
  'end;'#010+
  #010+
  'procedure TFPVersion.Clear;'#010+
  'begin'#010+
  '  Micro:=0;'#010+
  '  Major:=0;'#010+
  '  Minor:=0;'#010+
  '  Build:=0;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TFPVersion.Assign(Source: TPersistent);'#010+
  #010+
  'Var'#010+
  '  V : TFPVe','rsion;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TFPVersion.Assign(Source',': TPersistent);'#010+
  #010+
  'Var'#010+
  '  V : TFPVersion;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'begin'#010+
  '  if Source is TFPVersion then'#010+
  '    begin'#010+
  '    V:=Source as TFPVersion;'#010+
  '    Major:=V.Major;'#010+
  '    Minor:=V.Minor;'#010+
  '    Micro:=V.Micro;'#010+
  '    Build:=V.Build;'#010+
  '    end'#010+
  '  else'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    inherited Assign(Source);'#010+
  'end;'#010+
  #010+
  'function TFPVersion.Compa','reVersion(AVersion: TFPVersion): Integer;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '    inherited Assign(Source);',#010+
  'end;'#010+
  #010+
  'function TFPVersion.CompareVersion(AVersion: TFPVersion): Integer;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=Major-AVersion.Major;'#010+
  '  If (Result=0) then'#010+
  '    begin'#010+
  '      Result:=Minor-AVersion.Minor;'#010+
  '      if (Result=0) then'#010+
  '        begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '          Result:=Micro-AVersion.Micro;'#010+
  '          If (','Result=0) then'#010+
=======
  '          Result:=Micr','o-AVersion.Micro;'#010+
  '          If (Result=0) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '          Result:=Micr','o-AVersion.Micro;'#010+
  '          If (Result=0) then'#010+
>>>>>>> origin/fixes_2_2
  '            Result:=Build-AVersion.Build;'#010+
  '        end;'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  'function TFPVersion.SameVersion(AVersion: TFPVersion): Boolean;'#010+
  'begin'#010+
  '  Result:=CompareVersion(AVersion)=0;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{***************************************','****************************'+
=======
  '{*******','************************************************************'+
>>>>>>> graemeg/fixes_2_2
=======
  '{*******','************************************************************'+
>>>>>>> origin/fixes_2_2
  '*********'#010+
  '                                 TTarget'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'constructor TTarget.Create(ACollection: TCollection);'#010+
  'begin'#010+
  '  inherited Create(ACo','llection);'#010+
=======
  'constructor TTarget.Create(ACollection: TCollectio','n);'#010+
  'begin'#010+
  '  inherited Create(ACollection);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'constructor TTarget.Create(ACollection: TCollectio','n);'#010+
  'begin'#010+
  '  inherited Create(ACollection);'#010+
>>>>>>> origin/fixes_2_2
  '  FInstall:=True;'#010+
  '  FCPUs:=AllCPUs;'#010+
  '  FOSes:=AllOSes;'#010+
  '  FUnitPath:=TConditionalStrings.Create(TConditionalString);'#010+
  '  FIncludePath:=TConditionalStrings.Create(TConditionalString);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FObjectPath:=TConditionalStrings.Create(TConditi','onalString);'#010+
=======
  '  FObjectPath:=TCo','nditionalStrings.Create(TConditionalString);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  FObjectPath:=TCo','nditionalStrings.Create(TConditionalString);'#010+
>>>>>>> origin/fixes_2_2
  '  FDependencies:=TDependencies.Create(TDependency);'#010+
  '  FCommands:=TCOmmands.Create(TCommand);'#010+
  'end;'#010+
  #010+
  #010+
  'destructor TTarget.Destroy;'#010+
  'begin'#010+
  '  FreeAndNil(FUnitPath);'#010+
  '  FreeAndNil(FObjectPath);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FreeAndNil(FIncludePath);'#010+
  '  FreeAndNil(','FDependencies);'#010+
  '  FreeAndNil(FCommands);'#010+
  '  FreeAndNil(Foptions);'#010+
  '  inherited Destroy;'#010+
  'end;'#010+
  #010+
  'function TTarget.GetOptions: TStrings;'#010+
  'begin'#010+
  '  If Foptions=Nil then'#010+
  '    FOptions:=TStringList.Create;'#010+
  '  Result:=FOptions;'#010+
  'end;'#010+
  #010+
  'procedure TTarget.Se','tOptions(const AValue: TStrings);'#010+
  'begin'#010+
  '  If (AValue=Nil) or (AValue.Count=0) then'#010+
  '    FreeAndNil(FOptions)'#010+
  '  else'#010+
  '    Options.Assign(AValue);'#010+
  'end;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  FreeAnd','Nil(FIncludePath);'#010+
  '  FreeAndNil(FDependencies);'#010+
  '  FreeAndNil(FCommands);'#010+
  '  inherited Destroy;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'function TTarget.GetSourceFileName: String;'#010+
  'begin'#010+
  '  Result:=Name+FExtension;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'functio','n TTarget.GetUnitFileName: String;'#010+
  'begin'#010+
  '  Result:=Name+UnitExt;'#010+
=======
  'function TTarget.GetUnitFileName: String;'#010+
  'begin'#010+
  '  Resul','t:=Name+UnitExt;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TTarget.GetUnitFileName: String;'#010+
  'begin'#010+
  '  Resul','t:=Name+UnitExt;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'function TTarget.GetObjectFileName: String;'#010+
  'begin'#010+
  '  Result:=Name+ObjExt;'#010+
  'end;'#010+
  #010+
  #010+
  'function TTarget.GetRSTFileName: String;'#010+
  'begin'#010+
  '  Result:=Name+RSText;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TT','arget.GetProgramFileName(AOS : TOS): String;'#010+
  'begin'#010+
=======
  'function TTarget.GetProgramFileName(AOS : TOS): String;'#010+
  'beg','in'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TTarget.GetProgramFileName(AOS : TOS): String;'#010+
  'beg','in'#010+
>>>>>>> origin/fixes_2_2
  '  if AOS in [Go32v2,Win32,Win64,OS2] then'#010+
  '    Result:=Name+ExeExt'#010+
  '  else'#010+
  '    Result:=Name;'#010+
  'end;'#010+
  #010+
  #010+
  'function TTarget.GetOutputFileName(AOs: TOS): String;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  if TargetType in UnitTargets ','then'#010+
  '    Result:=GetUnitFileName'#010+
  '  else'#010+
  '    Result:=GetProgramFileName(AOs);'#010+
  'end;'#010+
  #010+
  'function TTarget.HaveOptions: Boolean;'#010+
  'begin'#010+
  '  Result:=(FOptions<>Nil);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  if TargetType in UnitTargets then'#010+
  '    Result:=GetUnitFileName'#010+
  '  else'#010+
  '    Resu','lt:=GetProgramFileName(AOs);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TTarget.SetName(const AValue: String);'#010+
  'Var'#010+
  '  D,N,E : String;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ',' N:=FixPath(AValue);'#010+
  '  D:=ExtractFilePath(N);'#010+
  '  E:=ExtractFileExt(N);'#010+
  '  N:=ExtractFileName(N);'#010+
  '  inherited SetName(Copy(N,1,Length(N)-Length(E)));'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  N:=FixPath(AValue);'#010+
  '  D:=ExtractFilePath(N);'#010+
  '  E:=ExtractFileExt(N);'#010+
  '  N:=ExtractFileName(N);'#010+
  '  inherited SetName(Copy(N,1,Le','ngth(N)-Length(E)));'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  FExtension:=E;'#010+
  '  FDirectory:=D;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TTarget.GetCleanFiles(List: TStrings; cons','t APrefixU, APre'+
  'fixB : String; ACPU: TCPU; AOS : TOS);'#010+
  'begin'#010+
  '  If not(ACPU in CPUs) or not(AOS in OSes) then'#010+
  '    exit;'#010+
  '  List.Add(APrefixU + ObjectFileName);'#010+
  '  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit, ttCleanOnlyU'+
  'nit]) then'#010+
  ' ','   List.Add(APrefixU + UnitFileName)'#010+
  '  else If (TargetType in [ttProgram,ttExampleProgram]) then'#010+
  '    List.Add(APrefixB + GetProgramFileName(AOS));'#010+
  '  If ResourceStrings then'#010+
  '    List.Add(APrefixU + RSTFileName);'#010+
  '  // Maybe add later ?  AddCo','nditionalStrings(List,CleanFiles);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'procedure TTarget.GetCleanFiles(List: TStrings; const APrefixU, APrefi'+
  'xB : String; ACPU: TCPU; AOS : TOS);'#010+
  'begin'#010+
  '  If not(ACPU in CPUs) or not(AOS in OSes) then'#010+
  '    exit;'#010+
  '  List.','Add(APrefixU + ObjectFileName);'#010+
  '  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit, ttCleanOnlyU'+
  'nit]) then'#010+
  '    List.Add(APrefixU + UnitFileName)'#010+
  '  else If (TargetType in [ttProgram,ttExampleProgram]) then'#010+
  '    List.Add(APrefixB + GetPr','ogramFileName(AOS));'#010+
  '  If ResourceStrings then'#010+
  '    List.Add(APrefixU + RSTFileName);'#010+
  '  // Maybe add later ?  AddConditionalStrings(List,CleanFiles);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TTarget.GetInstallFiles(List: TStrings; const APrefixU, APre'+
<<<<<<< HEAD
<<<<<<< HEAD
  'fixB: String; ACPU: TCPU; AOS : TOS);'#010+
  'begin'#010+
  '  If not(ACPU in CPUs) or not(AOS in OSes) then'#010+
  '    exit;'#010+
  '  If Not (TargetType in [t','tProgram,ttExampleProgram]) then'#010+
  '    List.Add(APrefixU + ObjectFileName);'#010+
  '  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit]) then'#010+
  '    List.Add(APrefixU + UnitFileName)'#010+
  '  else If (TargetType in [ttProgram,ttExampleProgram]) then'#010+
  '    ','List.Add(APrefixB + GetProgramFileName(AOS));'#010+
  '  If ResourceStrings then'#010+
  '    List.Add(APrefixU + RSTFileName);'#010+
  '  // Maybe add later ?  AddConditionalStrings(List,InstallFiles);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TTarget.GetArchiveFiles(List: TStrings; ACPU: T','CPU; AOS : '+
  'TOS);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'fixB: String; ','ACPU: TCPU; AOS : TOS);'#010+
  'begin'#010+
  '  If not(ACPU in CPUs) or not(AOS in OSes) then'#010+
  '    exit;'#010+
  '  If Not (TargetType in [ttProgram,ttExampleProgram]) then'#010+
  '    List.Add(APrefixU + ObjectFileName);'#010+
  '  If (TargetType in [ttUnit,ttImplicitUnit,ttExample','Unit]) then'#010+
  '    List.Add(APrefixU + UnitFileName)'#010+
  '  else If (TargetType in [ttProgram,ttExampleProgram]) then'#010+
  '    List.Add(APrefixB + GetProgramFileName(AOS));'#010+
  '  If ResourceStrings then'#010+
  '    List.Add(APrefixU + RSTFileName);'#010+
  '  // Maybe add l','ater ?  AddConditionalStrings(List,InstallFiles);'#010+
  'end;'#010+
  #010+
  #010+
  'procedure TTarget.GetArchiveFiles(List: TStrings; ACPU: TCPU; AOS : TO'+
  'S);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'var'#010+
  '  i : integer;'#010+
  '  D : TDependency;'#010+
  'begin'#010+
  '  If not(ACPU in CPUs) or not(AOS in OSes) then'#010+
  '    exit;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  // Main source'#010+
  '  if TargetSourceFileName<>'#039#039' then'#010+
  '    List.Add(TargetSourceFileName);'#010+
  '  // Includes'#010+
  '  for i:=0 to Depen','dencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=Dependencies[i];'#010+
  '      if (D.DependencyType=depInclude) and'#010+
  '         (D.TargetFileName<>'#039#039') then'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  // Ma','in source'#010+
  '  if TargetSourceFileName<>'#039#039' then'#010+
  '    List.Add(TargetSourceFileName);'#010+
  '  // Includes'#010+
  '  for i:=0 to Dependencies.Count-1 do'#010+
  '    begin'#010+
  '      D:=Dependencies[i];'#010+
  '      if (D.DependencyType=depInclude) and'#010+
  '         (D.TargetFileName<>',#039#039') then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '        List.Add(D.TargetFileName);'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{****************************************************','***************'+
  '*********'#010+
  '                                 TSource'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'function TSource.GetInstallSourcePath: string;'#010+
  'begin'#010+
  '  if FInstallSourcePath<>'#039#039' then'#010+
  '    resul','t := FInstallSourcePath'#010+
  '  else if SourceType=stExample then'#010+
  '    result := '#039'examples'#039#010+
  '  else'#010+
  '    result := '#039#039';'#010+
  'end;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{*********************************************************************'+
  '*******'#010+
  '                                 TSource'#010+
  '************************************************************','********'+
  '********}'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  'constructor TSource.Create(ACollection: TCollection);'#010+
  'begin'#010+
  '  inherited Create(ACollection);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'destructor TSource.Destr','oy;'#010+
=======
  'destructor TSource.Destroy;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'destructor TSource.Destroy;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  inherited Destroy;'#010+
  'end;'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TSource.GetInstallFiles(List: TStrings);'#010+
  'begin'#010+
  '  if InstallSourcePath<>'#039#039' then'#010+
  '    list.Values[name] := (IncludeTrailingPathDelimiter(InstallSourcePa'+
  'th)+ExtractFileName(Name))'#010+
  '  else'#010+
  '    list.a','dd(Name);'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
=======
  #010+
  '{**********************************************************','*********'+
  '*********'#010+
>>>>>>> graemeg/fixes_2_2
=======
  #010+
  '{**********************************************************','*********'+
  '*********'#010+
>>>>>>> origin/fixes_2_2
  '                                 TCommands'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TCommands.GetC','ommand(const Dest : String): TCommand;'#010+
  'begin'#010+
  '  Result:=TCommand(ItemByName(Dest));'#010+
=======
  'function TCommands.GetCommand(const Dest : String): TCommand;'#010+
  'begin'#010+
  '  Result:=TCommand(ItemByName(D','est));'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TCommands.GetCommand(const Dest : String): TCommand;'#010+
  'begin'#010+
  '  Result:=TCommand(ItemByName(D','est));'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  'function TCommands.GetCommandItem(Index : Integer): TCommand;'#010+
  'begin'#010+
  '  Result:=TCommand(Items[Index]);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'procedure TCommands.SetCommandItem(Index :',' Integer; const AValue: TC'+
  'ommand);'#010+
=======
  'procedure TCommands.SetCommandItem(Index : Integer; const AValue: TCom'+
  'mand);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'procedure TCommands.SetCommandItem(Index : Integer; const AValue: TCom'+
  'mand);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Items[Index]:=AValue;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TCommands.AddCommand(const Cmd: String) : TCommand;'#010+
=======
  'Func','tion TCommands.AddCommand(const Cmd: String) : TCommand;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Func','tion TCommands.AddCommand(const Cmd: String) : TCommand;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=AddCommand(fdefaultAt,Cmd,'#039#039','#039#039','#039#039');'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TCommands.AddCommand(const Cmd, Option','s: String): TCommand;'+
  #010+
  'begin'#010+
  '  Result:=AddCommand(fdefaultAt,Cmd,Options,'#039#039','#039#039');'#010+
=======
  'function TCommands.AddCommand(const Cmd, Options: String): TCommand;'#010+
  'begin'#010+
  '  Result:=AddCommand(fdefaultAt,Cmd,Options,'#039#039','#039,#039');'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TCommands.AddCommand(const Cmd, Options: String): TCommand;'#010+
  'begin'#010+
  '  Result:=AddCommand(fdefaultAt,Cmd,Options,'#039#039','#039,#039');'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'function TCommands.AddCommand(const Cmd, Options, Dest, Source: String'+
  ' ): TCommand;'#010+
  'begin'#010+
  '  Result:=AddCommand(fdefaultAt,Cmd,options,Dest,Source);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'end;'#010,
  #010+
  #010+
  'Function TCommands.AddCommand(At: TCommandAt; const Cmd: String) : TCo'+
  'mmand;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TCommands.AddCommand(At: TCommandAt; const Cmd: String) : TCo'+
  'mman','d;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=AddCommand(At,Cmd,'#039#039','#039#039','#039#039');'#010+
  'end;'#010+
  #010+
  #010+
  'function TCommands.AddCommand(At: TCommandAt; const Cmd, Options: Stri'+
  'ng  ): TCommand;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=AddCom','mand(At,Cmd,Options,'#039#039','#039#039');'#010+
  'end;'#010+
  #010+
  #010+
  'function TCommands.AddCommand(At: TCommandAt; const Cmd, Options, Dest'+
  ', Source: String): TCommand;'#010+
  'begin'#010+
  '  Result:=Add as TCommand;'#010+
  '  Result.Command:=Cmd;'#010+
  '  If (Options<>'#039#039') then'#010+
  '    Result.ParseOptions(Op','tions);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result:=AddCommand(At,Cmd,Options,'#039#039','#039#039');'#010+
  'end;'#010+
  #010+
  #010+
  'function TCommands.AddCommand(At: TComman','dAt; const Cmd, Options, De'+
  'st, Source: String): TCommand;'#010+
  'begin'#010+
  '  Result:=Add as TCommand;'#010+
  '  Result.Command:=Cmd;'#010+
  '  Result.Options:=Options;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Result.At:=At;'#010+
  '  Result.SourceFile:=Source;'#010+
  '  Result.DestFile:=Dest;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{*********************************************************************'+
  '*******'#010+
  '                           TConditionalString'#010+
  '******************************','**************************************'+
  '********}'#010+
  #010+
  'Constructor TConditionalString.Create;'#010+
  'begin'#010+
  '  inherited Create;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{********************','***********************************************'+
  '*********'#010+
  '                           TConditionalString'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'Constructor TConditionalString.Create;'#010+
  'begin'#010+
  '  inherited C','reate;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                           TConditionalSt','rings'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'Constructor TConditionalStrings.Create(AClass:TConditionalStringClass)'+
  ';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '                           TConditionalStrings'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'Constructor TCondition','alStrings.Create(AClass:TConditionalStringClas'+
  's);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  inherited Create;'#010+
  '  FCSClass:=AClass;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TConditionalStrings.G','etConditionalString(Index : Integer): '+
  'TConditionalString;'#010+
  'begin'#010+
  '  Result:=TConditionalString(Items[Index]);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'function TConditionalStrings.GetConditionalString(Index : Integer): TC'+
  'onditionalString;'#010+
  'begin'#010+
  '  Result:=TConditionalString(Items[Index]);',#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TConditionalStrings.SetConditionalString(Index : Integer; co'+
  'nst AValue: TConditionalString);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Items[Index]:=','AValue;'#010+
=======
  '  Items[Index]:=AValue;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Items[Index]:=AValue;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TConditionalStrings.Add(Const Value : String) : TConditionalS'+
  'tring;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  result:=Add(Value,AllCPUs,AllOSes);'#010+
=======
  '  result:','=Add(Value,AllCPUs,AllOSes);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  result:','=Add(Value,AllCPUs,AllOSes);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{$ifdef cpu_only_overloads}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPU',
  's) : TConditionalString;'#010+
=======
  'Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPUs'+
  ') : TConditionalString;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPUs'+
  ') : TConditionalString;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  result:=Add(Value,CPUs,AllOSes);'#010+
  'end;'#010+
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TConditionalStrings.Add(Const Value : String;const OSes:TOSes'+
  ') : TConditionalString;'#010+
  'begin'#010+
  '  result:=Add(Value,AllCPUs,OSes);'#010+
  'end;',#010+
  #010+
  #010+
  'Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPUs'+
  ';const OSes:TOSes) : TConditionalString;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Functi','on TConditionalStrings.Add(Const Value : String;const OSes:TOS'+
  'es) : TConditionalString;'#010+
  'begin'#010+
  '  result:=Add(Value,AllCPUs,OSes);'#010+
  'end;'#010+
  #010+
  #010+
  'Function TConditionalStrings.Add(Const Value : String;const CPUs:TCPUs'+
  ';const OSes:TOSes) : TConditionalS','tring;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result:=FCSClass.Create;'#010+
  '  Result.Value:=Value;'#010+
  '  Result.OSes:=OSes;'#010+
  '  Result.CPUs:=CPUs;'#010+
  '  inherited Add(Result);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'end',';'#010+
=======
  'end;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'end;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                                TDependency'#010+
=======
  '                         ','       TDependency'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '                         ','       TDependency'#010+
>>>>>>> origin/fixes_2_2
  '**********************************************************************'+
  '******}'#010+
  #010+
  'Constructor TDependency.Create;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'beg','in'#010+
=======
  'begin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begin'#010+
>>>>>>> origin/fixes_2_2
  '  inherited Create;'#010+
  '  FVersion:=TFPVersion.Create;'#010+
  'end;'#010+
  #010+
  #010+
  'Destructor TDependency.Destroy;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  FreeAndNil(FVersion);'#010+
=======
  '  FreeAn','dNil(FVersion);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  FreeAn','dNil(FVersion);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TDependency.GetVersion : string;'#010+
  'begin'#010+
  '  result:=FVersion.AsString;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Procedure TDependency.SetV','ersion(const V : string);'#010+
=======
  'Procedure TDependency.SetVersion(const V : string);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Procedure TDependency.SetVersion(const V : string);'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FVersion.AsString:=V;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{*********************************************************************'+
  '*******'#010+
  '                                TDependencies'#010+
  '*****************************************************','***************'+
  '********}'#010+
  #010+
  'function TDependencies.GetDependency(Index : Integer): TDependency;'#010+
  'begin'#010+
  '  Result:=TDependency(Items[Index]);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{*******************************************','************************'+
  '*********'#010+
  '                                TDependencies'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'function TDependencies.GetDependency(Index : Integer): TDependency;'#010+
  'begin'#010+
  '  Resul','t:=TDependency(Items[Index]);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TDependencies.SetDependency(Index : Integer; const AValue: T'+
  'Dependency);'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Items','[Index]:=AValue;'#010+
=======
  '  Items[Index]:=AValue;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Items[Index]:=AValue;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TDependencies.Add(Const Value : String) : TDependency;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  result:=Add(Value,AllCPUs,AllOSes);'#010+
=======
  '  result:=Add','(Value,AllCPUs,AllOSes);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  result:=Add','(Value,AllCPUs,AllOSes);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{$ifdef cpu_only_overloads}'#010+
  'Function TDependencies.Add(Const Value : String;const CPUs:TCPUs) : TD'+
<<<<<<< HEAD
<<<<<<< HEAD
  'epe','ndency;'#010+
=======
  'ependency;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'ependency;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  result:=Add(Value,CPUs,AllOSes);'#010+
  'end;'#010+
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TDependencies.Add(Const Value : String;const OSes:TOSes) : TD'+
  'ependency;'#010+
=======
  'Function TDependencies.','Add(Const Value : String;const OSes:TOSes) : '+
  'TDependency;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TDependencies.','Add(Const Value : String;const OSes:TOSes) : '+
  'TDependency;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  result:=Add(Value,AllCPUs,OSes);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TDependencies.Add(','Const Value : String;const CPUs:TCPUs;con'+
  'st OSes:TOSes) : TDependency;'#010+
  'begin'#010+
  '  Result:=inherited Add(Value,CPUs,OSes) as TDependency;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TDependencies.Add(Const Value : String;const CPUs:TCPUs;const'+
  ' OSes:TOSes) : TDependency;'#010+
  'begin'#010+
  '  Result:=inherited Add(Value,','CPUs,OSes) as TDependency;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Result.Target:=nil;'#010+
  '  Result.FDependencyType:=depPackage;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TDependencies.AddUnit(Const Va','lue : String) : TDependency;'#010+
=======
  'Function TDependencies.AddUnit(Const Value : String) : TDependency;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function TDependencies.AddUnit(Const Value : String) : TDependency;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  result:=AddUnit(Value,AllCPUs,AllOSes);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{$ifdef cpu_only_overloads}'#010+
  'Function TDependencies.AddUnit(Const Value : String;const CPUs:TCPUs) '+
  ': TDependency;'#010+
  'begin'#010+
  '  result:=AddUnit(Value,CPUs,AllOSes)',';'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{$ifdef cpu_only_overlo','ads}'#010+
  'Function TDependencies.AddUnit(Const Value : String;const CPUs:TCPUs) '+
  ': TDependency;'#010+
  'begin'#010+
  '  result:=AddUnit(Value,CPUs,AllOSes);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  '{$endif cpu_only_overloads}'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TDependencies.AddUnit(Const Value : String;const OSes:TOSes) '+
=======
  'Function TDependencies.AddUnit(Const Value : String;const OSes:TOSes) ',
>>>>>>> graemeg/fixes_2_2
=======
  'Function TDependencies.AddUnit(Const Value : String;const OSes:TOSes) ',
>>>>>>> origin/fixes_2_2
  ': TDependency;'#010+
  'begin'#010+
  '  result:=AddUnit(Value,AllCPUs,OSes);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TDependencies.AddUnit(Const Value : String;const CPUs:TCP','Us'+
  ';const OSes:TOSes) : TDependency;'#010+
  'begin'#010+
  '  Result:=inherited Add(Value,CPUs,OSes) as TDependency;'#010+
  '  Result.Target:=nil;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TDependencies.AddUnit(Const Value : String;const CPUs:TCPUs;c'+
  'onst OSes:TOSes) : TDependency;'#010+
  'begin'#010+
  '  Result:=inherited Add(Value,CPUs,OSes) as TDependency;'#010+
  '  Result','.Target:=nil;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  Result.FDependencyType:=depUnit;'#010+
  'end;'#010+
  #010+
  #010+
  'Function TDependencies.AddInclude(Const Value : String) : TDependency;'+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begin'#010,
=======
  'begin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begin'#010+
>>>>>>> origin/fixes_2_2
  '  result:=AddInclude(Value,AllCPUs,AllOSes);'#010+
  'end;'#010+
  #010+
  #010+
  '{$ifdef cpu_only_overloads}'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function TDependencies.AddInclude(Const Value : String;const CPUs:TCPU'+
  's) : TDependency;'#010+
  'begin'#010+
  '  result:=AddInclude(Value,CPUs,AllOSes);'#010+
  'end;'#010+
  '{$endif cpu_only_ov','erloads}'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  'Function TDependencies.AddI','nclude(Const Value : String;const CPUs:TC'+
  'PUs) : TDependency;'#010+
  'begin'#010+
  '  result:=AddInclude(Value,CPUs,AllOSes);'#010+
  'end;'#010+
  '{$endif cpu_only_overloads}'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  #010+
  #010+
  'Function TDependencies.AddInclude(Const Value : String;const OSes:TOSe'+
  's) : TDependency;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  result:=AddInclude(Value,AllCPUs,OSes);'#010+
=======
  '  ','result:=AddInclude(Value,AllCPUs,OSes);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  ','result:=AddInclude(Value,AllCPUs,OSes);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'Function TDependencies.AddInclude(Const Value : String;const CPUs:TCPU'+
<<<<<<< HEAD
<<<<<<< HEAD
  's;const OSes:TOS','es) : TDependency;'#010+
=======
  's;const OSes:TOSes) : TDependency;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  's;const OSes:TOSes) : TDependency;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  N : String;'#010+
  'begin'#010+
  '  N:=FixPath(Value);'#010+
  '  if ExtractFileExt(N)='#039#039' then'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    ChangeFileExt(N,IncExt);'#010+
=======
  '    ChangeFi','leExt(N,IncExt);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    ChangeFi','leExt(N,IncExt);'#010+
>>>>>>> origin/fixes_2_2
  '  Result:=inherited Add(N,CPUs,OSes) as TDependency;'#010+
  '  Result.FDependencyType:=depInclude;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{*****************','**************************************************'+
  '*********'#010+
  '                               TValueItem'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TValueItem.Create(AValue: String);'#010+
  'begin'#010+
  '  FVal','ue:=AValue;'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{*********************************************************************'+
  '*******'#010+
  '                               TValueItem'#010+
  '*****','***************************************************************'+
  '********}'#010+
  #010+
  'constructor TValueItem.Create(AValue: String);'#010+
  'begin'#010+
  '  FValue:=AValue;'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '                              TFunctionItem'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TFunctio','nItem.Create(AFunc: TReplaceFunction);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '          ','                    TFunctionItem'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TFunctionItem.Create(AFunc: TReplaceFunction);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  FFunc:=AFunc;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  '{*********************************************************************'+
  '*******'#010+
  '                                 TDictionary'#010+
  '*************************************************','*******************'+
  '********}'#010+
  #010+
  'constructor TDictionary.Create(AOwner: TComponent);'#010+
  'begin'#010+
  '  inherited Create(AOwner);'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '{**************************************','*****************************'+
  '*********'#010+
  '                                 TDictionary'#010+
  '**********************************************************************'+
  '******}'#010+
  #010+
  'constructor TDictionary.Create(AOwner: TComponent);'#010+
  'begin'#010+
  '  inherited Create(','AOwner);'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '  FList:=TStringList.Create;'#010+
  '  FList.Sorted:=True;'#010+
  '  FList.Duplicates:=dupError;'#010+
  'end;'#010+
  #010+
  #010+
  'destructor TDictionary.Destroy;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Var'#010,
=======
  'Var'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Var'#010+
>>>>>>> origin/fixes_2_2
  '  I : Integer;'#010+
  'begin'#010+
  '  For I:=0 to Flist.Count-1 do'#010+
  '    FList.Objects[i].Free;'#010+
  '  FreeAndNil(FList);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  inherited Destroy;'#010+
=======
  '  inher','ited Destroy;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  inher','ited Destroy;'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TDictionary.AddVariable(const AName, Value: String);'#010+
  'Var'#010+
  '  I : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  I:=Flist.IndexOf(AName',');'#010+
=======
  '  I:=Flist.IndexOf(AName);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  I:=Flist.IndexOf(AName);'#010+
>>>>>>> origin/fixes_2_2
  '  If I=-1 then'#010+
  '    I:=FList.Add(Aname)'#010+
  '  else'#010+
  '    Flist.Objects[i].Free;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Flist.Objects[i]:=TValueItem.Create(Value);'#010+
=======
  '  Flist.Objects[i]:=TValueItem.','Create(Value);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Flist.Objects[i]:=TValueItem.','Create(Value);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TDictionary.AddFunction(const AName: String; FReplacement: T'+
  'ReplaceFunction);'#010+
  'Var'#010+
  '  I : Integer;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'begi','n'#010+
=======
  'begin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begin'#010+
>>>>>>> origin/fixes_2_2
  '  I:=Flist.IndexOf(AName);'#010+
  '  If I=-1 then'#010+
  '    I:=Flist.Add(AName)'#010+
  '  else'#010+
  '    Flist.Objects[i].Free;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Flist.Objects[i]:=TFunctionItem.Create(FReplacement);'#010+
=======
  '  Fli','st.Objects[i]:=TFunctionItem.Create(FReplacement);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Fli','st.Objects[i]:=TFunctionItem.Create(FReplacement);'#010+
>>>>>>> origin/fixes_2_2
  'end;'#010+
  #010+
  #010+
  'procedure TDictionary.RemoveItem(const AName: String);'#010+
  'Var'#010+
  '  I : Integer;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'b','egin'#010+
=======
  'begin'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'begin'#010+
>>>>>>> origin/fixes_2_2
  '  I:=Flist.IndexOf(AName);'#010+
  '  If (I<>-1) then'#010+
  '    begin'#010+
  '    FList.Objects[i].Free;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '    FList.Delete(I);'#010+
=======
  '    FList.Delete(I);',#010+
>>>>>>> graemeg/fixes_2_2
=======
  '    FList.Delete(I);',#010+
>>>>>>> origin/fixes_2_2
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  'function TDictionary.GetValue(const AName: String): String;'#010+
  'begin'#010+
  '  Result:=GetValue(AName,'#039#039');'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'function TDic','tionary.GetValue(const AName,Args: String): String;'#010+
=======
  'function TDictionary.GetValue(const AName,Args: String): String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'function TDictionary.GetValue(const AName,Args: String): String;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  O : TObject;'#010+
  '  I : Integer;'#010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  I:=Flist.IndexOf(AName);'#010+
  '  If (I=-1) then'#010+
  '    Raise EDictionaryError.CreateFmt(SErrNoDictionaryItem,[AName]);'#010+
  '  O:=Flist.Objects[I];'#010+
  '  If O is TVa','lueItem then'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  I:=Flist.Inde','xOf(AName);'#010+
  '  If (I=-1) then'#010+
  '    Raise EDictionaryError.CreateFmt(SErrNoDictionaryItem,[AName]);'#010+
  '  O:=Flist.Objects[I];'#010+
  '  If O is TValueItem then'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '    Result:=TValueItem(O).FValue'#010+
  '  else'#010+
  '    Result:=TFunctionItem(O).FFunc(AName,Args);'#010+
  'end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  #010+
=======
  #010,
>>>>>>> graemeg/fixes_2_2
=======
  #010,
>>>>>>> origin/fixes_2_2
  #010+
  'function TDictionary.ReplaceStrings(Const ASource: String): String;'#010+
  'Var'#010+
  '  S,FN,FV : String;'#010+
  '  P: Integer;'#010+
  'begin'#010+
  '  Result:='#039#039';'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  S:=A','Source;'#010+
=======
  '  S:=ASource;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  S:=ASource;'#010+
>>>>>>> origin/fixes_2_2
  '  P:=Pos('#039'$('#039',S);'#010+
  '  While (P<>0) do'#010+
  '    begin'#010+
  '      Result:=Result+Copy(S,1,P-1);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      Delete(S,1,P+1);'#010+
=======
  '      Delete(S,1,','P+1);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      Delete(S,1,','P+1);'#010+
>>>>>>> origin/fixes_2_2
  '      P:=Pos('#039')'#039',S);'#010+
  '      FN:=Copy(S,1,P-1);'#010+
  '      Delete(S,1,P);'#010+
  '      P:=Pos('#039' '#039',FN);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      If (P<>0) then // function argum','ents ?'#010+
=======
  '      If (P<>0) then // function arguments ?'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '      If (P<>0) then // function arguments ?'#010+
>>>>>>> origin/fixes_2_2
  '        begin'#010+
  '        FV:=FN;'#010+
  '        FN:=Copy(FN,1,P);'#010+
  '        System.Delete(FV,1,P);'#010+
  '        end'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '      else'#010+
=======
  ' ','     else'#010+
>>>>>>> graemeg/fixes_2_2
=======
  ' ','     else'#010+
>>>>>>> origin/fixes_2_2
  '        FV:='#039#039';'#010+
  '      Result:=Result+GetValue(FN,FV);'#010+
  '      P:=Pos('#039'$('#039',S);'#010+
  '    end;'#010+
  '  Result:=Result+S;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function Su','bstitute(Const Source : String; Macros : Array of string)'+
  ' : String;'#010+
=======
  'Function Substitute(Const Source : String; Macros : Array of string) :'+
  ' String;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function Substitute(Const Source : String; Macros : Array of string) :'+
  ' String;'#010+
>>>>>>> origin/fixes_2_2
  'Var'#010+
  '  I : Integer;'#010+
  'begin'#010+
  '  I:=0;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  While I<High(Macros) do'#010+
=======
  '  Whil','e I<High(Macros) do'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  Whil','e I<High(Macros) do'#010+
>>>>>>> origin/fixes_2_2
  '    begin'#010+
  '      Dictionary.AddVariable(Macros[i],Macros[I+1]);'#010+
  '      Inc(I,2);'#010+
  '    end;'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  Result:=Dictionary.Repl','aceStrings(Source);'#010+
  '  While I<High(Macros) do'#010+
  '    begin'#010+
  '      Dictionary.RemoveItem(Macros[i]);'#010+
  '      Inc(I,2);'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                                 De','fault Instances'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '  Result:=Dictionary.ReplaceStrings(Source);'#010+
  '  While I<High(Macros) do'#010+
  '    begin'#010+
  '      Dictionary.RemoveItem(Macros[i]);'#010+
  '      Inc(I',',2);'#010+
  '    end;'#010+
  'end;'#010+
  #010+
  #010+
  '{*********************************************************************'+
  '*******'#010+
  '                                 Default Instances'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '**********************************************************************'+
  '******}'#010+
  #010+
  'var'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  DefInstaller : TCustomInstaller;'#010+
=======
  '  DefIn','staller : TCustomInstaller;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  DefIn','staller : TCustomInstaller;'#010+
>>>>>>> origin/fixes_2_2
  #010+
  'Function Installer(InstallerClass: TInstallerClass): TCustomInstaller;'+
  #010+
  'begin'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '  If Not Assigned(DefInstall','er) then'#010+
=======
  '  If Not Assigned(DefInstaller) then'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  If Not Assigned(DefInstaller) then'#010+
>>>>>>> origin/fixes_2_2
  '    DefInstaller:=InstallerClass.Create(Nil);'#010+
  '  Result:=DefInstaller;'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Function Installer: TCustomInstaller;'#010+
=======
  'Function Installer: ','TCustomInstaller;'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Function Installer: ','TCustomInstaller;'#010+
>>>>>>> origin/fixes_2_2
  'begin'#010+
  '  Result := Installer(TFPCInstaller);'#010+
  'end;'#010+
  #010+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  #010+
  '{ TCommand }'#010+
  #010+
  'function TCommand.GetOptions: TStrings;'#010+
  'begin'#010+
  '  If',' (FOptions=Nil) then'#010+
  '    FOptions:=TStringList.Create;'#010+
  '  Result:=FOptions;'#010+
  'end;'#010+
  #010+
  'procedure TCommand.SetOptions(const Value: TStrings);'#010+
  'begin'#010+
  '  If (Value=Nil) or (Value.Count=0) then'#010+
  '    FreeAndNil(FOptions)'#010+
  '  else'#010+
  '    Options.Assign(Value);',#010+
  'end;'#010+
  #010+
  'destructor TCommand.Destroy;'#010+
  'begin'#010+
  '  FreeAndNil(FOptions);'#010+
  '  inherited Destroy;'#010+
  'end;'#010+
  #010+
  'function TCommand.HaveOptions: Boolean;'#010+
  'begin'#010+
  '  Result:=(FOptions<>Nil);'#010+
  'end;'#010+
  #010+
  #010+
  'function TCommand.CmdLineOptions: String;'#010+
  'begin'#010+
  '  If HaveOptions th','en'#010+
  '    Result:=OptionListToString(Options);'#010+
  'end;'#010+
  #010+
  'procedure TCommand.ParseOptions(S: String);'#010+
  #010+
  'begin'#010+
  '  Options:=OptionsToStringList(S);'#010+
  'end;'#010+
  #010+
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'Initialization'#010+
  '  OnGetApplicationName:=@GetFPMakeName;'#010+
  #010+
  'Finalization'#010+
  '  FreeAndNil(DefInstaller);'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  ' ',' FreeAndNil(Dictionary);'#010+
=======
  '  FreeAndNil(Dictionary);'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '  FreeAndNil(Dictionary);'#010+
>>>>>>> origin/fixes_2_2
  '  FreeAndNil(Defaults);'#010+
  'end.'#010
);
