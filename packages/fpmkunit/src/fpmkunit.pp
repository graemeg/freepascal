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

<<<<<<< HEAD
<<<<<<< HEAD
{ For target or cpu dependent dependencies also add an overload where you
=======
{ For target or cpu dependent dependencies add also an overload were you
>>>>>>> graemeg/fixes_2_2
=======
{ For target or cpu dependent dependencies add also an overload were you
>>>>>>> origin/fixes_2_2
  can pass only a set of cpus. This is disabled for now because it creates
  an error in the compiler with overload choosing }
{ define cpu_only_overloads}

Interface

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$IFDEF OS2}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF OS2}

{$IFDEF GO32V2}
 {$DEFINE NO_UNIT_PROCESS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 {$DEFINE NO_THREADING}
{$ENDIF GO32V2}

{$IFDEF NDS}
 {$DEFINE NO_THREADING}
{$ENDIF NDS}

{$IFDEF NETBSD}
 { NetBSD pthreads are not yet working, try to use fpmake without threads }
  {$DEFINE NO_THREADING}
{$ENDIF NETBSD}

=======
{$ENDIF GO32V2}

>>>>>>> graemeg/cpstrnew
=======
{$ENDIF GO32V2}

>>>>>>> graemeg/cpstrnew
=======
{$ENDIF GO32V2}

>>>>>>> graemeg/cpstrnew
=======
{$ENDIF GO32V2}

>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef NO_UNIT_PROCESS}
  {$define HAS_UNIT_PROCESS}
{$endif NO_UNIT_PROCESS}

{$ifndef NO_UNIT_ZIPPER}
  {$define HAS_UNIT_ZIPPER}
{$endif NO_UNIT_ZIPPER}

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
uses
  SysUtils, Classes, StrUtils
{$ifdef HAS_UNIT_PROCESS}
  ,process
{$endif HAS_UNIT_PROCESS}
{$ifdef HAS_UNIT_ZIPPER}
  ,zipper
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif HAS_UNIT_ZIPPER}
  ;

Type
  TFileType = (ftSource,ftUnit,ftObject,ftResource,ftExecutable,ftStaticLibrary,
               ftSharedLibrary);
  TFileTypes = set of TFileType;

  // Please keep this order, see OSCPUSupported below
  TCpu=(cpuNone,
<<<<<<< HEAD
<<<<<<< HEAD
    i386,m68k,powerpc,sparc,x86_64,arm,powerpc64,avr,armeb,
    mips,mipsel,jvm,i8086,aarch64
=======
    i386,m68k,powerpc,sparc,x86_64,arm,powerpc64,avr,armeb
>>>>>>> graemeg/fixes_2_2
=======
    i386,m68k,powerpc,sparc,x86_64,arm,powerpc64,avr,armeb
>>>>>>> origin/fixes_2_2
  );
  TCPUS = Set of TCPU;

  // Please keep this order, see OSCPUSupported below
  TOS=(osNone,
    linux,go32v2,win32,os2,freebsd,beos,netbsd,
    amiga,atari, solaris, qnx, netware, openbsd,wdosx,
    palmos,macos,darwin,emx,watcom,morphos,netwlibc,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    win64,wince,gba,nds,embedded,symbian,haiku,iphonesim,
    aix,java,android,nativent,msdos,wii,aros,dragonfly,
    win16
=======
    win64,wince,gba,nds,embedded,symbian,haiku
>>>>>>> graemeg/cpstrnew
=======
    win64,wince,gba,nds,embedded,symbian,haiku
>>>>>>> graemeg/cpstrnew
=======
    win64,wince,gba,nds,embedded,symbian,haiku
>>>>>>> graemeg/cpstrnew
=======
    win64,wince,gba,nds,embedded,symbian,haiku
>>>>>>> origin/cpstrnew
=======
    win64,wince,gba,nds,embedded,symbian
>>>>>>> graemeg/fixes_2_2
=======
    win64,wince,gba,nds,embedded,symbian
>>>>>>> origin/fixes_2_2
  );
  TOSes = Set of TOS;

  TCompilerMode = (cmFPC,cmTP,cmObjFPC,cmDelphi,cmMacPas);
  TCompilerModes = Set of TCompilerMode;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TInstallMOde = (imInstall, imUnInstall);

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TTargetType = (ttProgram,ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit,ttExampleProgram,ttFPDoc);
  TTargetTypes = set of TTargetType;

  TFPDocFormat = (ffHtml, ffHtm, ffXHtml, ffLaTex, ffXMLStruct, ffChm);
  TFPDocFormats = set of TFPDocFormat;

=======
  TTargetType = (ttProgram,ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit,ttExampleProgram);
  TTargetTypes = set of TTargetType;

>>>>>>> graemeg/fixes_2_2
=======
  TTargetType = (ttProgram,ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit,ttExampleProgram);
  TTargetTypes = set of TTargetType;

>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  TNotifyProcEvent = procedure(Sender: TObject);

  TRunMode = (rmCompile,rmBuild,rmInstall,rmArchive,rmClean,rmDistClean,rmManifest,rmZipInstall,rmPkgList,rmUnInstall);

  TBuildMode = (bmOneByOne, bmBuildUnit{, bmSkipImplicitUnits});
  TBuildModes = set of TBuildMode;
  TProcessPackageResult = (ppHandled, ppDelayed);
  TCheckDependencyResult = (cdAvailable, cdNotAvailable, cdNotYetAvailable);
=======

  TRunMode = (rmCompile,rmBuild,rmInstall,rmArchive,rmClean,rmManifest);
>>>>>>> graemeg/fixes_2_2
=======

  TRunMode = (rmCompile,rmBuild,rmInstall,rmArchive,rmClean,rmManifest);
>>>>>>> origin/fixes_2_2

Const
  // Aliases
  Amd64   = X86_64;
  PPC = PowerPC;
  PPC64 = PowerPC64;
  DOS = Go32v2;
  MacOSX = Darwin;

  AllOSes = [Low(TOS)..High(TOS)];
  AllCPUs = [Low(TCPU)..High(TCPU)];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AllUnixOSes  = [Linux,FreeBSD,NetBSD,OpenBSD,Darwin,QNX,BeOS,Solaris,Haiku,iphonesim,aix,Android,dragonfly];
  AllBSDOSes      = [FreeBSD,NetBSD,OpenBSD,Darwin,iphonesim,dragonfly];
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AllUnixOSes  = [Linux,FreeBSD,NetBSD,OpenBSD,Darwin,QNX,BeOS,Solaris,Haiku];
  AllBSDOSes      = [FreeBSD,NetBSD,OpenBSD,Darwin];
>>>>>>> graemeg/cpstrnew
  AllWindowsOSes  = [Win32,Win64,WinCE];
  AllAmigaLikeOSes = [Amiga,MorphOS,AROS];
  AllLimit83fsOses = [go32v2,os2,emx,watcom,msdos,win16];

  AllSmartLinkLibraryOSes = [Linux,msdos,amiga,morphos,aros,win16]; // OSes that use .a library files for smart-linking
  AllImportLibraryOSes = AllWindowsOSes + [os2,emx,netwlibc,netware,watcom,go32v2,macos,nativent,msdos,win16];

  { This table is kept OS,Cpu because it is easier to maintain (PFV) }
  OSCPUSupported : array[TOS,TCpu] of boolean = (
<<<<<<< HEAD
    { os          none   i386    m68k  ppc    sparc  x86_64 arm    ppc64  avr    armeb  mips   mipsel jvm    i8086  aarch64 }
    { none }    ( false, false, false, false, false, false, false, false, false, false, false, false, false, false, false),
    { linux }   ( false, true,  true,  true,  true,  true,  true,  true,  false, true , true , true , false, false, true ),
    { go32v2 }  ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { win32 }   ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { os2 }     ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { freebsd } ( false, true,  true,  false, false, true,  false, false, false, false, false, false, false, false, false),
    { beos }    ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { netbsd }  ( false, true,  true,  true,  true,  true,  false, false, false, false, false, false, false, false, false),
    { amiga }   ( false, false, true,  true,  false, false, false, false, false, false, false, false, false, false, false),
    { atari }   ( false, false, true,  false, false, false, false, false, false, false, false, false, false, false, false),
    { solaris } ( false, true,  false, false, true,  true,  false, false, false, false, false, false, false, false, false),
    { qnx }     ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { netware } ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { openbsd } ( false, true,  true,  false, false, true,  false, false, false, false, false, false, false, false, false),
    { wdosx }   ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { palmos }  ( false, false, true,  false, false, false, true,  false, false, false, false, false, false, false, false),
    { macos }   ( false, false, false, true,  false, false, false, false, false, false, false, false, false, false, false),
    { darwin }  ( false, true,  false, true,  false, true,  true,  true,  false, false, false, false, false, false, true ),
    { emx }     ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { watcom }  ( false, true,  false, false, false ,false, false, false, false, false, false, false, false, false, false),
    { morphos } ( false, false, false, true,  false ,false, false, false, false, false, false, false, false, false, false),
    { netwlibc }( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { win64   } ( false, false, false, false, false, true,  false, false, false, false, false, false, false, false, false),
    { wince    }( false, true,  false, false, false, false, true,  false, false, false, false, false, false, false, false),
    { gba    }  ( false, false, false, false, false, false, true,  false, false, false, false, false, false, false, false),
    { nds    }  ( false, false, false, false, false, false, true,  false, false, false, false, false, false, false, false),
    { embedded }( false, true,  true,  true,  true,  true,  true,  true,  true,  true , false, false, false, false, false),
    { symbian } ( false, true,  false, false, false, false, true,  false, false, false, false, false, false, false, false),
    { haiku }   ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { iphonesim}( false, true,  false, false, false, true,  false, false, false, false, false, false, false, false, false),
    { aix    }  ( false, false, false, true,  false, false, false, true,  false, false, false, false, false, false, false),
    { java }    ( false, false, false, false, false, false, false, false, false, false, false, false, true , false, false),
    { android } ( false, true,  false, false, false, false, true,  false, false, false, false, true,  true , false, false),
    { nativent }( false, true,  false, false, false, false, false, false, false, false, false, false, false, false, false),
    { msdos }   ( false, false, false, false, false, false, false, false, false, false, false, false, false, true , false),
    { wii }     ( false, false, false, true , false, false, false, false, false, false, false, false, false, false, false),
    { aros }    ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false, false),
    { dragonfly}( false, false, false, false, false, true,  false, false, false, false, false, false, false, false, false),
    { win16 }   ( false, false, false, false, false, false, false, false, false, false, false, false, false, true , false)
=======
=======
=======
>>>>>>> origin/fixes_2_2
  AllUnixOSes  = [Linux,FreeBSD,NetBSD,OpenBSD,Darwin,QNX,BeOS,Solaris];
  AllBSDOSes      = [FreeBSD,NetBSD,OpenBSD,Darwin];
  AllWindowsOSes  = [Win32,Win64,WinCE];

  { This table is kept OS,Cpu because it is easier to maintain (PFV) }
  OSCPUSupported : array[TOS,TCpu] of boolean = (
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { os          none   i386    m68k  ppc    sparc  x86_64 arm    ppc64  avr    armeb}
    { none }    ( false, false, false, false, false, false, false, false, false, false),
    { linux }   ( false, true,  true,  true,  true,  true,  true,  true,  false, true ),
    { go32v2 }  ( false, true,  false, false, false, false, false, false, false, false),
    { win32 }   ( false, true,  false, false, false, false, false, false, false, false),
    { os2 }     ( false, true,  false, false, false, false, false, false, false, false),
    { freebsd } ( false, true,  true,  false, false, true,  false, false, false, false),
    { beos }    ( false, true,  false, false, false, false, false, false, false, false),
    { netbsd }  ( false, true,  true,  true,  true,  false, false, false, false, false),
    { amiga }   ( false, false, true,  true,  false, false, false, false, false, false),
    { atari }   ( false, false, true,  false, false, false, false, false, false, false),
    { solaris } ( false, true,  false, false, true,  false, false, false, false, false),
    { qnx }     ( false, true,  false, false, false, false, false, false, false, false),
    { netware } ( false, true,  false, false, false, false, false, false, false, false),
    { openbsd } ( false, true,  true,  false, false, false, false, false, false, false),
    { wdosx }   ( false, true,  false, false, false, false, false, false, false, false),
    { palmos }  ( false, false, true,  false, false, false, true,  false, false, false),
    { macos }   ( false, false, false, true,  false, false, false, false, false, false),
    { darwin }  ( false, true,  false, true,  false, true,  true,  true,  false, false),
    { emx }     ( false, true,  false, false, false, false, false, false, false, false),
    { watcom }  ( false, true,  false, false, false ,false, false, false, false, false),
    { morphos } ( false, false, false, true,  false ,false, false, false, false, false),
    { netwlibc }( false, true,  false, false, false, false, false, false, false, false),
    { win64   } ( false, false, false, false, false, true,  false, false, false, false),
    { wince    }( false, true,  false, false, false, false, true,  false, false, false),
    { gba    }  ( false, false, false, false, false, false, true,  false, false, false),
    { nds    }  ( false, false, false, false, false, false, true,  false, false, false),
    { embedded }( false, true,  true,  true,  true,  true,  true,  true,  true,  true ),
<<<<<<< HEAD
<<<<<<< HEAD
    { symbian } ( false, true,  false, false, false, false, true,  false, false, false),
    { haiku }   ( false, true,  false, false, false, false, false, false, false, false) 
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
=======
    { symbian } ( false, true,  false, false, false, false, true,  false, false, false)
>>>>>>> graemeg/fixes_2_2
=======
    { symbian } ( false, true,  false, false, false, false, true,  false, false, false)
>>>>>>> origin/fixes_2_2
  );

  // Useful
  UnitExt = '.ppu';
  PPUExt  = UnitExt;
  PasExt  = '.pas';
  PPExt   = '.pp';
  IncExt  = '.inc';
  ObjExt  = '.o';
  RstExt  = '.rst';
<<<<<<< HEAD
<<<<<<< HEAD
  RsjExt  = '.rsj';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  LibExt  = '.a';
  SharedLibExt = '.so';
  DLLExt  = '.dll';
  ExeExt  = '.exe';
<<<<<<< HEAD
<<<<<<< HEAD
  DbgExt  = '.dbg';
  ZipExt  = '.zip';
  FpmkExt = '.fpm';

  FPMakePPFile = 'fpmake.pp';
  ManifestFile = 'manifest.xml';
  PkgListFileBase = 'pkg-';
  PkgListFileExt = '.lst';
=======
=======
>>>>>>> origin/fixes_2_2
  ZipExt  = '.zip';

  FPMakePPFile = 'fpmake.pp';
  ManifestFile = 'manifest.xml';
  UnitConfigFile = 'fpunits.conf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  DirNotFound = '<dirnotfound>';

  UnitTargets = [ttUnit,ttImplicitUnit,ttCleanOnlyUnit,ttExampleUnit];
  ProgramTargets = [ttProgram,ttExampleProgram];

  DefaultMessages = [vlError,vlWarning,vlCommand];
  AllMessages = [vlError,vlWarning,vlCommand,vlInfo];

<<<<<<< HEAD
<<<<<<< HEAD
Type
  TTargets = Class;
=======

Type
>>>>>>> graemeg/fixes_2_2
=======

Type
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  private
    function GetItem(Index: Integer): TNamedItem;
    procedure SetItem(Index: Integer; AValue: TNamedItem);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    Function IndexOfName(const AName : String) : Integer;
    Function ItemByName(const AName : String) : TNamedItem;
    Property UniqueNames : Boolean Read FUniqueNames;
<<<<<<< HEAD
<<<<<<< HEAD
    property Items[Index: Integer]: TNamedItem read GetItem write SetItem;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
    FOptions: String;
    FSourceFile: String;
  Public
    Property SourceFile : String Read FSourceFile Write FSourceFile;
    Property DestFile : String Read FDestFile Write FDestFile;
    Property Command : String Read FCommand Write FCommand;
    Property Options : String Read FOptions Write FOptions;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property At : TCommandAt Read FCommandAt Write FCommandAt;
    Property IgnoreResult : Boolean Read FIgnoreResult Write FIgnoreResult;
    Property BeforeCommand : TNotifyEvent Read FBeforeCommand Write FBeforeCommand;
    Property AfterCommand : TNotifyEvent Read FAfterCommand Write FAfterCommand;
  end;

  { TCommands }

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
<<<<<<< HEAD
<<<<<<< HEAD
    FBuild    : Integer;
=======
    FBuild    : Word;
>>>>>>> graemeg/fixes_2_2
=======
    FBuild    : Word;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
   Property Major : Integer Read FMajor Write FMajor;
   Property Minor : Integer Read FMinor Write FMinor;
   Property Micro : Integer Read FMicro Write FMicro;
   Property Build : Integer Read FBuild Write FBuild;
=======
=======
>>>>>>> origin/fixes_2_2
   Property Major : Word Read FMajor Write FMajor;
   Property Minor : Word Read FMinor Write FMinor;
   Property Micro : Word Read FMicro Write FMicro;
   Property Build : Word Read FBuild Write FBuild;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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
                        neaBeforeCreateBuildEngine, neaAfterCreateBuildengine);

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

  TNotifyEventCollection = class(TCollection)
  private
    FSupportedActionSet: TNotifyEventActionSet;
  public
    constructor create(ASupportedActionSet: TNotifyEventActionSet);
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
    Function Substitute(Const Source : String; Macros : Array of string) : String; virtual;
  end;

  { TPackageDictionary }

  TPackageDictionary = Class(TDictionary)
  private
    FMasterDictionary: TDictionary;
  Public
    Function GetValue(const AName,Args : String) : String; override;
    property MasterDictionary: TDictionary read FMasterDictionary write FMasterDictionary;
  end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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

  { TPackageVariants }

  TPackageVariants = class(TNamedCollection)
  private
    FActivePackageVariantName: string;
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
    property Name: string read FName write FName;
    property MasterPackage: TPackage read FMasterPackage;
    property DefaultPackageVariant: TPackageVariant read GetDefaultPackageVariant;
    property ActivePackageVariant: TPackageVariant read GetActivePackageVariant;
    property DefaultPackageVariantName: string read FDefaultPackageVariantName write SetDefaultPackageVariantName;
    property ActivePackageVariantName: string read FActivePackageVariantName write SetActivePackageVariantName;
    property IsInheritable: boolean read FIsInheritable;
  end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

  { TTarget }

  TTarget = Class(TNamedItem)
  private
    FInstall : Boolean;
    FAfterClean: TNotifyEvent;
    FAfterCompile: TNotifyEvent;
    FBeforeClean: TNotifyEvent;
    FBeforeCompile: TNotifyEvent;
    FCPUs: TCPUs;
    FOSes: TOSes;
    FMode: TCompilerMode;
    FResourceStrings: Boolean;
    FObjectPath,
    FUnitPath,
    FIncludePath : TConditionalStrings;
    FDependencies : TDependencies;
    FCommands : TCommands;
    FDirectory: String;
    FExtension: String;
    FTargetSourceFileName : String;
    FFileType: TFileType;
<<<<<<< HEAD
<<<<<<< HEAD
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
    function GetRSTFileName : String; Virtual;
    function GetRSJFileName : String; Virtual;
    function GetImportLibFileName(AOS : TOS) : String; Virtual;
    Function GetProgramFileName(AOS : TOS) : String; Virtual;
    Function GetProgramDebugFileName(AOS : TOS) : String; Virtual;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    procedure AssignTo(Dest: TPersistent); override;
    Function  GetOutputFileName (AOs : TOS) : String; Virtual;
    Function HaveOptions : Boolean;
    procedure SetName(const AValue: String);override;
    procedure SetXML(const AValue: string);
=======
=======
>>>>>>> origin/fixes_2_2
    FOptions: String;
    FFPCTarget: String;
    FTargetState: TTargetState;
    FTargetType: TTargetType;
  Protected
    Function GetSourceFileName : String; virtual;
    Function GetUnitFileName : String; virtual;
    Function GetObjectFileName : String; virtual;
    Function GetRSTFileName : String; Virtual;
    Function GetProgramFileName(AOS : TOS) : String; Virtual;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    Function  GetOutputFileName (AOs : TOS) : String; Virtual;
    procedure SetName(const AValue: String);override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Procedure GetCleanFiles(List : TStrings; const APrefixU, APrefixB : String; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetInstallFiles(List : TStrings; const APrefixU, APrefixB: String; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    Property Dependencies : TDependencies Read FDependencies;
    Property Commands : TCommands Read FCommands;
    Property State : TTargetState Read FTargetState;
    Property TargetType : TTargetType Read FTargetType Write FTargetType;
    Property OSes : TOSes Read FOSes Write FOSes;
    Property CPUs : TCPUs Read FCPUs Write FCPUs;
    Property Mode : TCompilerMode Read FMode Write FMode;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Options : TStrings Read GetOptions Write SetOptions;
=======
    Property Options : String Read FOptions Write Foptions;
>>>>>>> graemeg/fixes_2_2
=======
    Property Options : String Read FOptions Write Foptions;
>>>>>>> origin/fixes_2_2
    Property SourceFileName: String Read GetSourceFileName ;
    Property UnitFileName : String Read GetUnitFileName;
    Property ObjectFileName : String Read GetObjectFileName;
    Property RSTFileName : String Read GetRSTFileName;
<<<<<<< HEAD
<<<<<<< HEAD
    Property RSJFileName : String Read GetRSJFileName;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    Property XML: string Read FXML Write SetXML;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Events.
    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FBeforeCompile;
    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAfterCompile;
    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBeforeClean;
    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterClean;
  end;

  { TTargets }

  TTargets = Class(TNamedCollection)
  private
    function GetTargetItem(Index : Integer): TTarget;
    function GetTarget(const AName : String): TTarget;
    procedure SetTargetItem(Index : Integer; const AValue: TTarget);
  Public
<<<<<<< HEAD
<<<<<<< HEAD
    Function AddFPDoc(Const AUnitName, AXMLName : String) : TTarget;inline;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
    Property Targets[AName : String] : TTarget Read GetTarget; default;
    Property TargetItems[Index : Integer] : TTarget Read GetTargetItem Write SetTargetItem;
  end;

  { TSource }

  TSource = Class(TNamedItem)
  private
    FSourceType : TSourceType;
<<<<<<< HEAD
<<<<<<< HEAD
    FInstallSourcePath : string;
    function GetInstallSourcePath: string;
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    Procedure GetInstallFiles(List : TStrings); virtual;
    property SourceType : TSourceType read FSourceType;
    property InstallSourcePath : string read GetInstallSourcePath;
=======
=======
>>>>>>> origin/fixes_2_2
  Public
    Constructor Create(ACollection : TCollection); override;
    Destructor Destroy; override;
    property SourceType : TSourceType read FSourceType;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TSources }

  TSources = Class(TNamedCollection)
  private
    function GetSourceItem(Index : Integer): TSource;
    procedure SetSourceItem(Index : Integer; const AValue: TSource);
  public
    Function AddDoc(const AFiles : String) : TSource;
<<<<<<< HEAD
<<<<<<< HEAD
    Function AddDoc(const AFiles : String; AInstallSourcePath : String) : TSource;
    Function AddSrc(const AFiles : String) : TSource;
    Function AddExample(const AFiles : String) : TSource;
    Function AddExample(const AFiles : String; AInstallSourcePath : String) : TSource;
    Function AddTest(const AFiles : String) : TSource;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure AddDocFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False; AInstallSourcePath : String = '');
    procedure AddSrcFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False);
    procedure AddExampleFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False; AInstallSourcePath : String = '');
    procedure AddTestFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean = False);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure AddDocFiles(const AFileMask: string; Recursive: boolean = False; AInstallSourcePath : String = '');
    procedure AddSrcFiles(const AFileMask: string; Recursive: boolean = False);
    procedure AddExampleFiles(const AFileMask: string; Recursive: boolean = False; AInstallSourcePath : String = '');
    procedure AddTestFiles(const AFileMask: string; Recursive: boolean = False);
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    Function AddSrc(const AFiles : String) : TSource;
    Function AddExample(const AFiles : String) : TSource;
    Function AddTest(const AFiles : String) : TSource;
    procedure AddDocFiles(const AFileMask: string; Recursive: boolean = False);
    procedure AddSrcFiles(const AFileMask: string; Recursive: boolean = False);
    procedure AddExampleFiles(const AFileMask: string; Recursive: boolean = False);
    procedure AddTestFiles(const AFileMask: string; Recursive: boolean = False);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property SourceItems[Index : Integer] : TSource Read GetSourceItem Write SetSourceItem;default;
  end;

  { TPackage }

  TPackage = Class(TNamedItem)
  private
    FAfterArchive: TNotifyEvent;
<<<<<<< HEAD
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FBeforeManifestProc: TNotifyProcEvent;
    FBeforePkgList: TNotifyEvent;
    FBeforePkgListProc: TNotifyProcEvent;
    FBuildMode: TBuildMode;
    FFlags: TStrings;
    FFPDocFormat: TFPDocFormats;
    FIsFPMakeAddIn: boolean;
    FSupportBuildModes: TBuildModes;
=======
    FFPDocFormat: TFPDocFormats;
    FIsFPMakeAddIn: boolean;
>>>>>>> graemeg/cpstrnew
=======
    FFPDocFormat: TFPDocFormats;
    FIsFPMakeAddIn: boolean;
>>>>>>> graemeg/cpstrnew
=======
    FFPDocFormat: TFPDocFormats;
    FIsFPMakeAddIn: boolean;
>>>>>>> graemeg/cpstrnew
=======
    FFPDocFormat: TFPDocFormats;
    FIsFPMakeAddIn: boolean;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    FAfterClean: TNotifyEvent;
    FAfterCompile: TNotifyEvent;
    FAfterInstall: TNotifyEvent;
    FAfterManifest: TNotifyEvent;
    FBeforeArchive: TNotifyEvent;
    FBeforeClean: TNotifyEvent;
    FBeforeCompile: TNotifyEvent;
    FBeforeInstall: TNotifyEvent;
    FBeforeManifest: TNotifyEvent;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FUnitPath,
    FObjectPath,
    FIncludePath,
    FSourcePath,
    FExamplePath,
    FTestPath,
<<<<<<< HEAD
<<<<<<< HEAD
    FCleanFiles   : TConditionalStrings;
    FInstallFiles : TConditionalDestStrings;
=======
    FCleanFiles,
    FInstallFiles : TConditionalStrings;
>>>>>>> graemeg/fixes_2_2
=======
    FCleanFiles,
    FInstallFiles : TConditionalStrings;
>>>>>>> origin/fixes_2_2
    FDependencies : TDependencies;
    FCPUs: TCPUs;
    FOSes: TOSes;
    FTargetState: TTargetState;
    FTargets: TTargets;
    FSources: TSources;
    FDirectory: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FOptions: TStrings;
    FFileName: String;
    FShortName: String;
=======
    FOptions: String;
    FFileName: String;
>>>>>>> graemeg/fixes_2_2
=======
    FOptions: String;
    FFileName: String;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
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
    Function GetDescription : string;
    function GetDictionary: TDictionary;
    Function GetFileName : string;
    Function GetShortName : string;
    function GetOptions: TStrings;
    Function GetVersion : string;
    procedure SetOptions(const AValue: TStrings);
    Procedure SetVersion(const V : string);
  Protected
    procedure SetName(const AValue: String);override;
    procedure SaveUnitConfigToStringList(Const AStringList: TStrings;ACPU:TCPU;AOS:TOS); virtual;
    property Dictionary: TDictionary read GetDictionary;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetInstallSourceFiles(List: TStrings; SourceTypes : TSourceTypes; TargetTypes : TTargetTypes); virtual;
=======
    procedure GetInstallSourceFiles(List: TStrings;Types : TSourceTypes); virtual;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetInstallSourceFiles(List: TStrings;Types : TSourceTypes); virtual;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetInstallSourceFiles(List: TStrings;Types : TSourceTypes); virtual;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetInstallSourceFiles(List: TStrings;Types : TSourceTypes); virtual;
>>>>>>> origin/cpstrnew
    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetArchiveSourceFiles(List : TStrings); virtual;
    Procedure GetManifest(Manifest : TStrings);
    Procedure ListPackage(PkgList : TStrings);
    Procedure AddPackageVariant(APackageVariant: TPackageVariants);
    procedure ApplyPackageVariantToCompilerOptions(ACompilerOptions: tstrings);
    procedure SetDefaultPackageVariant;
    procedure LoadUnitConfigFromFile(Const AFileName: String);
    procedure SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;AOS:TOS);
    Property Version : String Read GetVersion Write SetVersion;
    Property FileName : String Read GetFileName Write FFileName;
    Property ShortName : String Read GetShortName Write FShortName;
=======
=======
>>>>>>> origin/fixes_2_2
    // Cached directory of installed packages
    FUnitDir : String;
    Function GetDescription : string;
    Function GetFileName : string;
    Function GetVersion : string;
    Procedure SetVersion(const V : string);
  Protected
    procedure SetName(const AValue: String);override;
    procedure LoadUnitConfigFromFile(Const AFileName: String);
    procedure SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;AOS:TOS);
  Public
    constructor Create(ACollection: TCollection); override;
    destructor destroy; override;
    Function  GetUnitsOutputDir(ACPU:TCPU; AOS : TOS):String;
    Function  GetBinOutputDir(ACPU:TCPU; AOS : TOS) : String;
    Procedure GetCleanFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    procedure GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU:TCPU; AOS : TOS);
    Procedure GetArchiveFiles(List : TStrings; ACPU:TCPU; AOS : TOS); virtual;
    Procedure GetManifest(Manifest : TStrings);
    Property Version : String Read GetVersion Write SetVersion;
    Property FileName : String Read GetFileName Write FFileName;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property HomepageURL : String Read FHomepageURL Write FHomepageURL;
    Property DownloadURL : String Read FDownloadURL Write FDownloadURL;
    Property Email : String Read FEmail Write FEmail;
    Property Author : String Read FAuthor Write FAuthor;
    Property License : String Read FLicense Write FLicense;
    Property Directory : String Read FDirectory Write FDirectory;
    Property Description : String Read GetDescription Write FDescription;
    Property DescriptionFile : String Read FDescriptionFile Write FDescriptionFile;
    Property InstalledChecksum : Cardinal Read FInstalledChecksum Write FInstalledChecksum;
<<<<<<< HEAD
<<<<<<< HEAD
    Property IsFPMakeAddIn: boolean read FIsFPMakeAddIn write FIsFPMakeAddIn;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property SupportBuildModes: TBuildModes read FSupportBuildModes write FSupportBuildModes;
    Property BuildMode: TBuildMode read FBuildMode;
    Property Flags: TStrings read FFlags;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Compiler options.
    Property OSes : TOSes Read FOSes Write FOSes;
    Property CPUs : TCPUs Read FCPUs Write FCPUs;
    Property NeedLibC : Boolean Read FNeedLibC Write FNeedLibC;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Options: TStrings Read GetOptions Write SetOptions;
=======
    Property Options: String Read FOptions Write FOptions;
>>>>>>> graemeg/fixes_2_2
=======
    Property Options: String Read FOptions Write FOptions;
>>>>>>> origin/fixes_2_2
    Property UnitPath : TConditionalStrings Read FUnitPath;
    Property ObjectPath : TConditionalStrings Read FObjectPath;
    Property IncludePath : TConditionalStrings Read FIncludePath;
    Property SourcePath : TConditionalStrings Read FSourcePath;
    Property ExamplePath : TConditionalStrings Read FExamplePath;
    Property TestPath : TConditionalStrings Read FTestPath;
<<<<<<< HEAD
<<<<<<< HEAD
    Property FPDocFormat: TFPDocFormats read FFPDocFormat write FFPDocFormat;
    // Targets and dependencies
    Property InstallFiles : TConditionalDestStrings Read FInstallFiles;
=======
    // Targets and dependencies
    Property InstallFiles : TConditionalStrings Read FInstallFiles;
>>>>>>> graemeg/fixes_2_2
=======
    // Targets and dependencies
    Property InstallFiles : TConditionalStrings Read FInstallFiles;
>>>>>>> origin/fixes_2_2
    Property CleanFiles : TConditionalStrings Read FCleanFiles;
    Property Dependencies : TDependencies Read FDependencies;
    Property Commands : TCommands Read FCommands;
    Property State : TTargetState Read FTargetState;
    Property Targets : TTargets Read FTargets;
    Property Sources : TSources Read FSources;
    Property UnitDir : String Read FUnitDir Write FUnitDir;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
    // events
    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FBeforeCompile;
    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAfterCompile;
    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FBeforeInstall;
    Property AfterInstall : TNotifyEvent Read FAfterInstall Write FAfterInstall;
    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBeforeClean;
    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterClean;
    Property BeforeArchive : TNotifyEvent Read FBeforeArchive Write FBeforeArchive;
    Property AfterArchive : TNotifyEvent Read FAfterArchive Write FAfterArchive;
    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write FBeforeManifest;
    Property AfterManifest : TNotifyEvent Read FAfterManifest Write FAfterManifest;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TPackages }

  TPackages = Class(TNamedCollection)
  private
    function GetPackage(const AName : String): TPackage;
    function GetPackageItem(AIndex : Integer): TPackage;
    procedure SetPackageItem(AIndex : Integer; const AValue: TPackage);
  Public
    Function AddPackage(Const AName : String) : TPackage;
    Property Packages[AName : String] : TPackage Read GetPackage ; Default;
    Property PackageItems[AIndex : Integer] : TPackage Read GetPackageItem Write SetPackageItem;
  end;

  { TCustomDefaults }

  TCustomDefaults = Class(TPersistent)
  Private
    FArchive: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FBuildMode: TBuildMode;
    FCompiler: String;
    FCopy: String;
    FFPDocOutputDir: String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FFPUnitSourcePath: String;
    FIgnoreInvalidOptions: Boolean;
    FInstallExamples: Boolean;
=======
    FIgnoreInvalidOptions: Boolean;
>>>>>>> graemeg/cpstrnew
=======
    FIgnoreInvalidOptions: Boolean;
>>>>>>> graemeg/cpstrnew
=======
    FIgnoreInvalidOptions: Boolean;
>>>>>>> graemeg/cpstrnew
=======
    FIgnoreInvalidOptions: Boolean;
>>>>>>> origin/cpstrnew
    FMkDir: String;
    FMove: String;
    FOptions: TStrings;
=======
=======
>>>>>>> origin/fixes_2_2
    FCompiler: String;
    FCopy: String;
    FMkDir: String;
    FMove: String;
    FOptions: String;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FCPU: TCPU;
    FOS: TOS;
    FMode : TCompilerMode;
    FCompilerVersion : String;
    FPrefix: String;
    FLocalUnitDir,
    FGlobalUnitDir,
    FBaseInstallDir,
    FUnitInstallDir,
<<<<<<< HEAD
<<<<<<< HEAD
    FUnitConfigFilesInstallDir,
    FBinInstallDir,
    FDocInstallDir,
    FExamplesInstallDir : String;
    FSkipCrossPrograms: boolean;
    FThreadsAmount: integer;
    FRemoveTree: String;
    FRemoveDir: String;
=======
    FBinInstallDir,
    FDocInstallDir,
    FExamplesInstallDir : String;
>>>>>>> graemeg/fixes_2_2
=======
    FBinInstallDir,
    FDocInstallDir,
    FExamplesInstallDir : String;
>>>>>>> origin/fixes_2_2
    FRemove: String;
    FTarget: String;
    FUnixPaths: Boolean;
    FNoFPCCfg: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FUseEnvironment: Boolean;
    FZipPrefix: String;
    FExplicitOSNone: Boolean;
    function GetBuildCPU: TCpu;
    function GetBuildOS: TOS;
    function GetBuildString: String;
    function GetFPDocOutputDir: String;
    function GetFPUnitSourcePath: String;
=======
    function GetFPDocOutputDir: String;
>>>>>>> graemeg/cpstrnew
=======
    function GetFPDocOutputDir: String;
>>>>>>> graemeg/cpstrnew
=======
    function GetFPDocOutputDir: String;
>>>>>>> graemeg/cpstrnew
=======
    function GetFPDocOutputDir: String;
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function GetLocalUnitDir: String;
    function GetGlobalUnitDir: String;
    function GetBaseInstallDir: String;
    function GetBinInstallDir: String;
    function GetCompiler: String;
    function GetDocInstallDir: String;
    function GetExamplesInstallDir: String;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetOptions: TStrings;
    function GetPrefix: String;
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
    procedure SetTarget(const AValue: String);
    procedure SetUnitInstallDir(const AValue: String);
    procedure SetUnitConfigFilesInstallDir(const AValue: String);
    procedure SetZipPrefix(AValue: String);
  Protected
    procedure RecalcTarget;
    Function CmdLineOptions : String;
  Public
    Constructor Create;
    Procedure InitDefaults;
    Function HaveOptions: Boolean;
    function IsBuildDifferentFromTarget: boolean;
=======
    function GetUnitInstallDir: String;
    procedure SetLocalUnitDir(const AValue: String);
    procedure SetGlobalUnitDir(const AValue: String);
    procedure SetBaseInstallDir(const AValue: String);
    procedure SetCPU(const AValue: TCPU);
    procedure SetOS(const AValue: TOS);
    procedure SetPrefix(const AValue: String);
    procedure SetTarget(const AValue: String);
  Protected
    procedure RecalcTarget;
  Public
    Constructor Create;
    Procedure InitDefaults;
>>>>>>> graemeg/fixes_2_2
=======
    function GetUnitInstallDir: String;
    procedure SetLocalUnitDir(const AValue: String);
    procedure SetGlobalUnitDir(const AValue: String);
    procedure SetBaseInstallDir(const AValue: String);
    procedure SetCPU(const AValue: TCPU);
    procedure SetOS(const AValue: TOS);
    procedure SetPrefix(const AValue: String);
    procedure SetTarget(const AValue: String);
  Protected
    procedure RecalcTarget;
  Public
    Constructor Create;
    Procedure InitDefaults;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
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
    Property Prefix : String Read GetPrefix Write SetPrefix;
    Property ZipPrefix : String Read FZipPrefix Write SetZipPrefix;
    Property BaseInstallDir : String Read GetBaseInstallDir Write SetBaseInstallDir;
    Property UnitInstallDir : String Read GetUnitInstallDir Write SetUnitInstallDir;
    Property UnitConfigFilesInstallDir : String Read GetUnitConfigFilesInstallDir Write SetUnitConfigFilesInstallDir;
    Property BinInstallDir : String Read GetBinInstallDir Write FBinInstallDir;
    Property DocInstallDir : String Read GetDocInstallDir Write FDocInstallDir;
    Property ExamplesInstallDir : String Read GetExamplesInstallDir Write FExamplesInstallDir;
    Property FPDocOutputDir : String Read GetFPDocOutputDir Write FFPDocOutputDir;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property FPUnitSourcePath: String read GetFPUnitSourcePath Write FFPUnitSourcePath;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    Property Mode : TCompilerMode Read FMode Write FMode;
    Property UnixPaths : Boolean Read FUnixPaths Write FUnixPaths;
    Property Options : String Read FOptions Write FOptions;    // Default compiler options.
    Property NoFPCCfg : Boolean Read FNoFPCCfg Write FNoFPCCfg;
    // paths etc.
    Property LocalUnitDir : String Read GetLocalUnitDir Write SetLocalUnitDir;
    Property GlobalUnitDir : String Read GetGlobalUnitDir Write SetGlobalUnitDir;
    Property Prefix : String Read FPrefix Write SetPrefix;
    Property BaseInstallDir : String Read GetBaseInstallDir Write SetBaseInstallDir;
    Property UnitInstallDir : String Read GetUnitInstallDir Write FUnitInstallDir;
    Property BinInstallDir : String Read GetBinInstallDir Write FBinInstallDir;
    Property DocInstallDir : String Read GetDocInstallDir Write FDocInstallDir;
    Property ExamplesInstallDir : String Read GetExamplesInstallDir Write FExamplesInstallDir;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Command tools. If not set, internal commands  will be used.
    Property Compiler : String Read GetCompiler Write FCompiler; // Compiler. Defaults to fpc
    Property Copy : String Read FCopy Write FCopy;             // copy $(FILES) to $(DEST)
    Property Move : String Read FMove Write FMove;             // Move $(FILES) to $(DEST)
    Property Remove : String Read FRemove Write FRemove;       // Delete $(FILES)
<<<<<<< HEAD
<<<<<<< HEAD
    Property RemoveDir : String Read FRemoveDir Write FRemoveDir;       // Delete $(FILES)
    Property RemoveTree : String Read FRemoveTree Write FRemoveTree;       // removes $(DIRECTORY)
    Property MkDir : String Read FMkDir write FMkDir;          // Make $(DIRECTORY)
    Property Archive : String Read FArchive Write FArchive;    // zip $(ARCHIVE) $(FILESORDIRS)
    // Misc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property UseEnvironment : Boolean read FUseEnvironment write FUseEnvironment;
    Property IgnoreInvalidOptions: Boolean read FIgnoreInvalidOptions write FIgnoreInvalidOptions;
    Property BuildMode: TBuildMode read FBuildMode write FBuildMode;
    // Installation optioms
    Property InstallExamples: Boolean read FInstallExamples write FInstallExamples;
    Property SkipCrossPrograms: boolean read FSkipCrossPrograms write FSkipCrossPrograms;
=======
    Property IgnoreInvalidOptions: Boolean read FIgnoreInvalidOptions write FIgnoreInvalidOptions;
>>>>>>> graemeg/cpstrnew
=======
    Property IgnoreInvalidOptions: Boolean read FIgnoreInvalidOptions write FIgnoreInvalidOptions;
>>>>>>> graemeg/cpstrnew
=======
    Property IgnoreInvalidOptions: Boolean read FIgnoreInvalidOptions write FIgnoreInvalidOptions;
>>>>>>> graemeg/cpstrnew
=======
    Property IgnoreInvalidOptions: Boolean read FIgnoreInvalidOptions write FIgnoreInvalidOptions;
>>>>>>> origin/cpstrnew
=======
    Property MkDir : String Read FMkDir write FMkDir;          // Make $(DIRECTORY)
    Property Archive : String Read FArchive Write FArchive;    // zip $(ARCHIVE) $(FILESORDIRS)
>>>>>>> graemeg/fixes_2_2
=======
    Property MkDir : String Read FMkDir write FMkDir;          // Make $(DIRECTORY)
    Property Archive : String Read FArchive Write FArchive;    // zip $(ARCHIVE) $(FILESORDIRS)
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  TCopyFileProc = procedure(const APackage: TPackage; Const ASourceFileName, ADestFileName : String) of object;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  TBuildEngine = Class(TComponent)
  private
    // general variables
    FCompiler : String;
    FStartDir : String;
    FForceCompile : Boolean;
    FListMode : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FVerbose : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FProgressMax : integer;
    FProgressCount : integer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef HAS_UNIT_ZIPPER}
    FZipFile: TZipper;
{$endif HAS_UNIT_ZIPPER}
>>>>>>> graemeg/cpstrnew
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
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef HAS_UNIT_ZIPPER}
    FZipFile: TZipper;
{$endif HAS_UNIT_ZIPPER}
    FExternalPackages : TPackages;
    // Logging
    FLogPrefix : String;
    // Events
    FOnLog: TLogEvent;
    FAfterArchive: TNotifyEvent;
    FAfterClean: TNotifyEvent;
    FAfterCompile: TNotifyEvent;
    FAfterInstall: TNotifyEvent;
    FAfterManifest: TNotifyEvent;
    FBeforeArchive: TNotifyEvent;
    FBeforeClean: TNotifyEvent;
    FBeforeCompile: TNotifyEvent;
    FBeforeInstall: TNotifyEvent;
    FBeforeManifest: TNotifyEvent;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Protected
    Procedure Error(const Msg : String);
    Procedure Error(const Fmt : String; const Args : Array of const);
    // Internal copy/delete/move/archive/mkdir files
    Function  SysDirectoryExists(const ADir:string):Boolean;
    Function  SysFileExists(const AFileName:string):Boolean;
    Procedure SysCopyFile(Const Src,Dest : String); virtual;
    Procedure SysMoveFile(Const Src,Dest : String); virtual;
    Procedure SysDeleteFile(Const AFileName : String); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure SysDeleteDirectory(Const ADirectoryName : String); virtual;
    Procedure SysDeleteTree(Const ADirectoryName : String); virtual;
    Procedure SysArchiveFiles(List : TStrings; Const AFileName : String); virtual;
    procedure LogIndent;
    procedure LogUnIndent;
    Procedure EnterDir(ADir : String);
    Function GetCompiler : String;
<<<<<<< HEAD
    Function InstallPackageFiles(APAckage : TPackage; tt : TTargetTypes; Const Dest : String; Const InstallMode: TInstallMode):Boolean;
    Procedure InstallUnitConfigFile(APAckage : TPackage; Const Dest : String);
    function GetUnitConfigFilesInstallDir(ABaseDir: string): String;

    Function InstallPackageSourceFiles(APAckage : TPackage; stt : TSourceTypes; ttt : TTargetTypes; Const Dest : String; Const InstallMode: TInstallMode):Boolean;
=======
    Function InstallPackageFiles(APAckage : TPackage; tt : TTargetType; Const Dest : String):Boolean;
    Function InstallPackageSourceFiles(APAckage : TPackage; tt : TSourceType; Const Dest : String):Boolean;
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
    Function FileNewer(const Src,Dest : String) : Boolean;
    Procedure LogSearchPath(APackage: TPackage;const ASearchPathName:string;Path:TConditionalStrings; ACPU:TCPU;AOS:TOS);
    Function FindFileInPath(APackage: TPackage; Path:TConditionalStrings; AFileName:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;

    procedure GetDirectoriesFromFilelist(const AFileList, ADirectoryList: TStringList);
    procedure AddPackageMacrosToDictionary(const APackage: TPackage; ADictionary: TDictionary);
    //package commands
    function  GetUnitDir(APackage:TPackage):String;
    procedure ResolvePackagePaths(APackage:TPackage);
    procedure AddDependencyPaths(L: TStrings; DependencyType: TDependencyType; ATarget: TTarget);
=======
=======
>>>>>>> origin/fixes_2_2
    Procedure SysArchiveFiles(List : TStrings; Const AFileName : String); virtual;
    procedure LogIndent;
    procedure LogUnIndent;
    Procedure Log(Level : TVerboseLevel; Const Msg : String);
    Procedure Log(Level : TVerboseLevel; Const Fmt : String; const Args : Array Of Const);
    Procedure EnterDir(ADir : String);
    Function GetCompiler : String;
    Function InstallPackageFiles(APAckage : TPackage; tt : TTargetType; Const Dest : String):Boolean;
    Function FileNewer(const Src,Dest : String) : Boolean;
    Procedure LogSearchPath(const ASearchPathName:string;Path:TConditionalStrings; ACPU:TCPU;AOS:TOS);
    Function FindFileInPath(Path:TConditionalStrings; AFileName:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;

    //package commands
    Procedure ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;DoChangeDir:boolean=true);
    function  GetUnitDir(APackage:TPackage):String;
    procedure AddDependencyIncludePaths(L:TStrings;ATarget: TTarget);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure AddDependencyUnitPaths(L:TStrings;APackage: TPackage);
  Public
    Constructor Create(AOwner : TComponent); override;
    destructor Destroy;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function AddPathPrefix(APackage: TPackage; APath: string): string;

    property Verbose : boolean read FVerbose write FVerbose;
    Procedure ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;DoChangeDir:boolean=true; WarnIfNotFound:boolean=true);
    Procedure ClearResolvedFileNames(APackage : TPackage);
=======

    property Verbose : boolean read FVerbose write FVerbose;
>>>>>>> graemeg/cpstrnew
=======

    property Verbose : boolean read FVerbose write FVerbose;
>>>>>>> graemeg/cpstrnew
=======

    property Verbose : boolean read FVerbose write FVerbose;
>>>>>>> graemeg/cpstrnew
=======

    property Verbose : boolean read FVerbose write FVerbose;
>>>>>>> origin/cpstrnew

    // Public Copy/delete/Move/Archive/Mkdir Commands.
    Procedure ExecuteCommand(const Cmd,Args : String; const Env: TStrings = nil; IgnoreError : Boolean = False); virtual;
    procedure CmdCopyFiles(List: TStrings; const DestDir: String; APackage: TPackage);
    Procedure CmdCreateDir(const DestDir : String);
    Procedure CmdMoveFiles(List : TStrings; Const DestDir : String);
    Procedure CmdDeleteFiles(List : TStrings);
    procedure CmdDeleteDestFiles(List: TStrings; const DestDir: String);
    Procedure CmdArchiveFiles(List : TStrings; Const ArchiveFile : String);
    Procedure CmdRenameFile(SourceName, DestName : String);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure CmdRemoveDirs(List: TStrings);
    Procedure CmdRemoveTrees(List: TStrings);
    Procedure ExecuteCommands(Commands : TCommands; At : TCommandAt; APackage: TPackage = nil);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Procedure ExecuteCommands(Commands : TCommands; At : TCommandAt);
>>>>>>> graemeg/cpstrnew
    // Dependency commands
    Function  DependencyOK(ADependency : TDependency) : Boolean;
    // Target commands
    Function  GetCompilerCommand(APackage : TPackage; ATarget : TTarget; Env: TStrings) : String;
    Function  TargetOK(ATarget : TTarget; ACPU: TCPU; AOS: TOS) : Boolean;
    Function  TargetInstallOK(ATarget : TTarget;ACPU:TCPU; AOS : TOS) : Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    // Public Copy/delete/Move/Archive/Mkdir Commands.
    Procedure ExecuteCommand(const Cmd,Args : String; IgnoreError : Boolean = False); virtual;
    Procedure CmdCopyFiles(List : TStrings; Const DestDir : String);
    Procedure CmdCreateDir(const DestDir : String);
    Procedure CmdMoveFiles(List : TStrings; Const DestDir : String);
    Procedure CmdDeleteFiles(List : TStrings);
    Procedure CmdArchiveFiles(List : TStrings; Const ArchiveFile : String);
    Procedure ExecuteCommands(Commands : TCommands; At : TCommandAt);
    // Dependency commands
    Function  DependencyOK(ADependency : TDependency) : Boolean;
    // Target commands
    Function  GetCompilerCommand(APackage : TPackage; ATarget : TTarget) : String;
    Function  TargetOK(ATarget : TTarget) : Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    Function ReadyToCompile(APackage:TPackage) : Boolean;
    Procedure Install(APackage : TPackage; AnArchiveFiles: boolean);
    Procedure UnInstall(APackage : TPackage);
    Procedure Archive(APackage : TPackage);
    Procedure PkgList(PkgList: TStrings; APackage : TPackage);
    Procedure Clean(APackage : TPackage; AllTargets: boolean);
    Procedure Clean(APackage : TPackage; ACPU:TCPU; AOS : TOS);
    Procedure CompileDependencies(APackage : TPackage);
    function CheckDependencies(APackage : TPackage; ErrorOnFailure: boolean): TCheckDependencyResult;
    Function  CheckExternalPackage(Const APackageName : String; ErrorOnFailure: boolean):TPackage;
=======
=======
>>>>>>> origin/fixes_2_2
    Procedure Install(APackage : TPackage);
    Procedure Archive(APackage : TPackage);
    Procedure Manifest(APackage : TPackage);
    Procedure Clean(APackage : TPackage);
    Procedure CompileDependencies(APackage : TPackage);
    Function  CheckExternalPackage(Const APackageName : String):TPackage;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure CreateOutputDir(APackage: TPackage);
    // Packages commands
    Procedure Compile(Packages : TPackages);
    Procedure Install(Packages : TPackages);
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure UnInstall(Packages : TPackages);
    Procedure ZipInstall(Packages : TPackages);
    Procedure Archive(Packages : TPackages);
    procedure Manifest(Packages: TPackages; Package: TPackage);
    procedure PkgList(Packages: TPackages);
    Procedure Clean(Packages : TPackages; AllTargets: boolean);

    Procedure Log(Level : TVerboseLevel; Msg : String);
    Procedure Log(Level : TVerboseLevel; Fmt : String; const Args : Array Of Const);

    Property ListMode : Boolean Read FListMode Write FListMode;
    Property ForceCompile : Boolean Read FForceCompile Write FForceCompile;
    Property ExternalPackages: TPackages Read FExternalPackages;
    Property StartDir: String Read FStartDir;
    // Events
    Property NotifyEventCollection: TNotifyEventCollection read FNotifyEventCollection;
=======
=======
>>>>>>> origin/fixes_2_2
    Procedure Archive(Packages : TPackages);
    procedure Manifest(Packages: TPackages);
    Procedure Clean(Packages : TPackages);
    Property ListMode : Boolean Read FListMode Write FListMode;
    Property ForceCompile : Boolean Read FForceCompile Write FForceCompile;
    Property ExternalPackages: TPackages Read FExternalPackages;
    // Events
    Property BeforeCompile : TNotifyEvent Read FBeforeCompile Write FBeforeCompile;
    Property AfterCompile : TNotifyEvent Read FAfterCompile Write FAfterCompile;
    Property BeforeInstall : TNotifyEvent Read FBeforeInstall Write FBeforeInstall;
    Property AfterInstall : TNotifyEvent Read FAfterInstall Write FAfterInstall;
    Property BeforeClean : TNotifyEvent Read FBeforeClean Write FBeforeClean;
    Property AfterClean : TNotifyEvent Read FAfterClean Write FAfterClean;
    Property BeforeArchive : TNotifyEvent Read FBeforeArchive Write FBeforeArchive;
    Property AfterArchive : TNotifyEvent Read FAfterArchive Write FAfterArchive;
    Property BeforeManifest : TNotifyEvent Read FBeforeManifest Write FBeforeManifest;
    Property AfterManifest : TNotifyEvent Read FAfterManifest Write FAfterManifest;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property OnLog : TLogEvent Read FOnLog Write FOnlog;
  end;

  { TCustomInstaller }

  TCustomInstaller = Class(TComponent)
  private
    FBuildEngine: TBuildEngine;
    FPackages: TPackages;
    FRunMode: TRunMode;
    FListMode : Boolean;
    FLogLevels : TVerboseLevels;
<<<<<<< HEAD
<<<<<<< HEAD
    FFPMakeOptionsString: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FPackageVariantSettings: TStrings;
    FPackageVariants: TFPList;
    FNotifyEventCollection: TNotifyEventCollection;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Protected
    Procedure Log(Level : TVerboseLevel; Const Msg : String);
    Procedure CreatePackages; virtual;
    Procedure FreePackages; virtual;
    function GetPackages: TPackages; virtual;
=======
  Protected
    Procedure Log(Level : TVerboseLevel; Const Msg : String);
    Procedure CreatePackages; virtual;
>>>>>>> graemeg/fixes_2_2
=======
  Protected
    Procedure Log(Level : TVerboseLevel; Const Msg : String);
    Procedure CreatePackages; virtual;
>>>>>>> origin/fixes_2_2
    Procedure CheckPackages; virtual;
    Procedure CreateBuildEngine; virtual;
    Procedure Error(const Msg : String);
    Procedure Error(const Fmt : String; Args : Array of const);
    Procedure AnalyzeOptions;
    Procedure Usage(const FMT : String; Args : Array of const);
    Procedure Compile(Force : Boolean); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Clean(AllTargets: boolean); virtual;
    Procedure Install; virtual;
    Procedure UnInstall; virtual;
    Procedure ZipInstall; virtual;
    Procedure Archive; virtual;
    Procedure Manifest; virtual;
    Procedure PkgList; virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    Procedure Clean; virtual;
    Procedure Install; virtual;
    Procedure Archive; virtual;
    Procedure Manifest; virtual;
    Property BuildEngine : TBuildEngine Read FBuildEngine;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    Constructor Create(AOwner : TComponent); virtual;
    Destructor destroy; override;
    Function AddPackage(Const AName : String) : TPackage;
<<<<<<< HEAD
<<<<<<< HEAD
    Function  AddPackageVariant(AName: string; AIsInheritable: boolean): TPackageVariants;
    Function Run : Boolean;
    Property FPMakeOptionsString: string read FFPMakeOptionsString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property BuildEngine : TBuildEngine Read FBuildEngine;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    //files in package
    Property Packages : TPackages Read GetPackages;
    Property RunMode : TRunMode Read FRunMode;
    Property ListMode : Boolean Read FListMode;
    Property NotifyEventCollection : TNotifyEventCollection read FNotifyEventCollection;
=======
=======
>>>>>>> origin/fixes_2_2
    Function Run : Boolean;
    //files in package
    Property Packages : TPackages Read FPackages;
    Property RunMode : TRunMode Read FRunMode;
    Property ListMode : Boolean Read FListMode;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
=======
  TReplaceFunction = Function (Const AName,Args : String) : String of Object;

>>>>>>> graemeg/fixes_2_2
=======
  TReplaceFunction = Function (Const AName,Args : String) : String of Object;

>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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

=======
=======
>>>>>>> origin/fixes_2_2
  { TDictionary }

  TDictionary = Class(TComponent)
    FList : TStringList;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy;override;
    Procedure AddVariable(Const AName,Value : String);
    Procedure AddFunction(Const AName : String; FReplacement : TReplaceFunction);
    Procedure RemoveItem(Const AName : String);
    Function GetValue(Const AName : String) : String;
    Function GetValue(Const AName,Args : String) : String; virtual;
    Function ReplaceStrings(Const ASource : String) : String; virtual;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ECollectionError = Class(Exception);
  EDictionaryError = Class(Exception);
  EInstallerError = Class(Exception);

  TInstallerClass = Class of TCustomInstaller;
  TDictionaryClass = Class of TDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
  TPackageDictionaryClass = Class of TPackageDictionary;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Type
  TArchiveEvent = Procedure (Const AFileName : String; List : TStrings) of Object;
  TArchiveProc = Procedure (Const AFileName : String; List : TStrings);

Var
  DictionaryClass : TDictionaryClass = TDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
  PackageDictionaryClass : TPackageDictionaryClass = TPackageDictionary;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  OnArchiveFiles : TArchiveEvent = Nil;
  ArchiveFilesProc : TArchiveProc = Nil;

  Defaults : TCustomDefaults; // Set by installer.
<<<<<<< HEAD
<<<<<<< HEAD
  GlobalDictionary : TDictionary;
=======
  Dictionary : TDictionary;
>>>>>>> graemeg/fixes_2_2
=======
  Dictionary : TDictionary;
>>>>>>> origin/fixes_2_2


Function CurrentOS : String;
Function CurrentCPU : String;

Function Installer(InstallerClass: TInstallerClass) : TCustomInstaller; overload;
Function Installer : TCustomInstaller; overload;

Function OSToString(OS: TOS) : String;
Function OSesToString(OSes: TOSes) : String;
Function CPUToString(CPU: TCPU) : String;
Function CPUSToString(CPUS: TCPUS) : String;
Function StringToOS(const S : String) : TOS;
<<<<<<< HEAD
<<<<<<< HEAD
function IsDifferentFromBuild(ACpu: TCPU; AOs: TOs): boolean;
//Function StringToOSes(const S : String) : TOSes;
=======
Function OSesToString(const S : String) : TOSes;
>>>>>>> graemeg/fixes_2_2
=======
Function OSesToString(const S : String) : TOSes;
>>>>>>> origin/fixes_2_2
Function StringToCPU(const S : String) : TCPU;
Function StringToCPUS(const S : String) : TCPUS;
Function ModeToString(Mode: TCompilerMode) : String;
Function StringToMode(const S : String) : TCompilerMode;
Function MakeTargetString(CPU : TCPU;OS: TOS) : String;
Procedure StringToCPUOS(const S : String; Var CPU : TCPU; Var OS: TOS);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function FixPath (const APath : String) : String; inline; deprecated 'Use the overload with AIsDir instead';
Function FixPath (const APath : String; AIsDir : Boolean) : String;
=======
Function FixPath (const APath : String) : String;
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
Function IsRelativePath(const APath : String) : boolean;
Procedure ChangeDir(const APath : String);
Procedure SplitCommand(Const Cmd : String; Var Exe,Options : String);
Procedure AddCustomFpmakeCommandlineOption(const ACommandLineOption, HelpMessage : string);
Function GetCustomFpmakeCommandlineOptionValue(const ACommandLineOption : string) : string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function AddProgramExtension(const ExecutableName: string; AOS : TOS) : string;
Function GetImportLibraryFilename(const UnitName: string; AOS : TOS) : string;

procedure SearchFiles(AFileName, ASearchPathPrefix: string; Recursive: boolean; var List: TStrings);
function GetDefaultLibGCCDir(CPU : TCPU;OS: TOS; var ErrorMessage: string): string;
=======

procedure SearchFiles(const AFileName: string; Recursive: boolean; var List: TStrings);
>>>>>>> graemeg/cpstrnew
=======

procedure SearchFiles(const AFileName: string; Recursive: boolean; var List: TStrings);
>>>>>>> graemeg/cpstrnew
=======

procedure SearchFiles(const AFileName: string; Recursive: boolean; var List: TStrings);
>>>>>>> graemeg/cpstrnew
=======

procedure SearchFiles(const AFileName: string; Recursive: boolean; var List: TStrings);
>>>>>>> origin/cpstrnew

Implementation

uses typinfo, rtlconsts;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
  TUnsortedDuplicatesStringList = class(TStringList)
  public
    function Add(const S: string): Integer; override;
  end;

var
  CustomFpmakeCommandlineOptions: TStrings;
  CustomFpMakeCommandlineValues: TStrings;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef NO_THREADING}
var
{$else NO_THREADING}
threadvar
{$endif NO_THREADING}
  GPathPrefix : string;
  GLogPrefix  : string;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
Function FixPath (const APath : String) : String;
Procedure ChangeDir(const APath : String);
Function Substitute(Const Source : String; Macros : Array of string) : String;
Procedure SplitCommand(Const Cmd : String; Var Exe,Options : String);

Implementation

uses typinfo;
>>>>>>> graemeg/fixes_2_2
=======
Function FixPath (const APath : String) : String;
Procedure ChangeDir(const APath : String);
Function Substitute(Const Source : String; Macros : Array of string) : String;
Procedure SplitCommand(Const Cmd : String; Var Exe,Options : String);

Implementation

uses typinfo;
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
  SErrDepUnknownTarget  = 'Unknown target for unit "%s" in dependencies for %s in package %s';
  SErrExternalCommandFailed = 'External command "%s" failed with exit code %d. Console output:'+LineEnding+'%s';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrExtCommandNotFound= 'External command "%s" not found';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SErrCreatingDirectory = 'Failed to create directory "%s"';
  SErrDeletingFile      = 'Failed to delete file "%s"';
  SErrRemovingDirectory = 'Failed to remove directory "%s"';
=======
=======
>>>>>>> origin/fixes_2_2
  SErrDepUnknownTarget  = 'Unknown target in dependencies for %s: %s';
  SErrExternalCommandFailed = 'External command "%s" failed with exit code %d';
  SErrCreatingDirectory = 'Failed to create directory "%s"';
  SErrDeletingFile      = 'Failed to delete file "%s"';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SErrMovingFile        = 'Failed to move file "%s" to "%s"';
  SErrCopyingFile       = 'Failed to copy file "%s" to "%s"';
  SErrChangeDirFailed   = 'Failed to enter directory "%s"';
  SErrInvalidArgumentToSubstitute = 'Invalid number of arguments to Substitute';
  SErrNoArchiveSupport  = 'This binary contains no archive support. Please recompile with archive support';
  SErrNoDictionaryItem  = 'No item called "%s" in the dictionary';
  SErrNoDictionaryValue = 'The item "%s" in the dictionary is not a value';
  SErrNoDictionaryFunc  = 'The item "%s" in the dictionary is not a function';
  SErrInvalidFPCInfo    = 'Compiler returns invalid information, check if fpc -iV works';
  SErrDependencyNotFound = 'Could not find unit directory for dependency package "%s"';
  SErrAlreadyInitialized = 'Installer can only be initialized once';
  SErrInvalidState      = 'Invalid state for target %s';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrCouldNotCompile   = 'Could not compile target %s from package %s';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrUnsupportedBuildmode = 'Package does not support this buildmode';
  SErrPackVarNotExist   = 'There is no package variant with the name "%s"';
  SErrEventNotSupported = 'Unsupported event type';
  SErrorPkgNotInstalled = 'Package "%s" is not installed, can not uninstall.';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  SWarnCircularTargetDependency = 'Warning: Circular dependency detected when compiling target %s with target %s';
  SWarnCircularPackageDependency = 'Warning: Circular dependency detected when compiling package %s with package %s';
  SWarnFailedToSetTime    = 'Warning: Failed to set timestamp on file "%s"';
  SWarnFailedToGetTime    = 'Warning: Failed to get timestamp from file "%s"';
<<<<<<< HEAD
<<<<<<< HEAD
  SWarnAttemptingToCompileNonNeutralTarget = 'Warning: Attempting to compile non-neutral target %s';
  SWarnSourceFileNotFound  = 'Warning: Source file "%s" from package %s not found for %s';
  SWarnIncludeFileNotFound = 'Warning: Include file "%s" from package %s not found for %s';
  SWarnDepUnitNotFound     = 'Warning: Dependency on unit %s is not supported for %s';
  SWarnTargetDependsOnPackage = 'Warning: Target %s of package %s depends on another package (%s). These kind of dependencies are not processed';
  SWarnDependOnOtherPlatformPackage = 'Warning: Package %s depends on package %s which is not available for the %s platform';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SWarnStartCompilingPackage = 'Start compiling package %s for target %s.';
  SWarnCompilingPackagecompleteProgress = '[%3.0f%%] Compiled package %s';
  SWarnCompilingPackagecomplete = 'Compiled package %s';
  SWarnSkipPackageTargetProgress = '[%3.0f%%] Skipped package %s which has been disabled for target %s';
  SWarnSkipPackageTarget = 'Skipped package %s which has been disabled for target %s';
  SWarnInstallationPackagecomplete = 'Installation package %s for target %s succeeded';
  SWarnCleanPackagecomplete = 'Clean of package %s completed';
  SWarnCanNotGetAccessRights = 'Warning: Failed to copy access-rights from file %s';
  SWarnCanNotSetAccessRights = 'Warning: Failed to copy access-rights to file %s';
  SWarnCanNotGetFileAge = 'Warning: Failed to get FileAge for %s';
  SWarnExtCommandNotFound = 'Warning: External command "%s" not found but "%s" is older then "%s"';
  SWarnDuplicatePackage = 'Warning: Package %s is already added. Using the existing package';
  SWarngccNotFound        = 'Could not find libgcc';
  SWarngcclibpath         = 'Warning: Unable to determine the libgcc path.';
  SWarnNoFCLProcessSupport= 'No FCL-Process support';
  SWarnRetryRemDirectory     = 'Failed to remove directory "%s". Retry after a short delay';

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SWarnStartBuildingPackage = 'Start building package %s for target %s.';
  SWarnBuildingPackagecomplete = '[%3.0f%%] Built target %s';
  SWarnInstallationPackagecomplete = 'Installation package %s for target %s succeeded';
  SWarnCleanPackagecomplete = 'Clean of package %s completed';

  SInfoCompilingPackage   = 'Compiling package %s';
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
  SInfoPackageAlreadyProcessed = 'Package %s is already processed';
=======
=======
>>>>>>> origin/fixes_2_2
  SWarnFileDoesNotExist   = 'Warning: File "%s" does not exist';
  SWarnAttemptingToCompileNonNeutralTarget = 'Warning: Attempting to compile non-neutral target %s';
  SWarnSourceFileNotFound  = 'Warning: Source file "%s" not found for %s';
  SWarnIncludeFileNotFound = 'Warning: Include file "%s" not found for %s';
  SWarnDepUnitNotFound     = 'Warning: Dependency on unit %s is not supported for %s';

  SInfoCompilingPackage   = 'Compiling package %s';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SInfoCompilingTarget    = 'Compiling target %s';
  SInfoExecutingCommand   = 'Executing command "%s %s"';
  SInfoCreatingOutputDir  = 'Creating output dir "%s"';
  SInfoInstallingPackage  = 'Installing package %s';
<<<<<<< HEAD
<<<<<<< HEAD
  SInfoUnInstallingPackage= 'Uninstalling package %s';
  SInfoArchivingPackage   = 'Archiving package %s in "%s"';
  SInfoCleaningPackage    = 'Cleaning package %s';
  SInfoManifestPackage    = 'Creating manifest for package %s';
  SInfoPkgListPackage    = 'Adding package %s to the package list';
  SInfoCopyingFile        = 'Copying file "%s" to "%s"';
  SInfoDeletedFile        = 'Deleted file "%s"';
  SInfoRemovedDirectory   = 'Removed directory "%s"';
  SInfoSourceNewerDest    = 'Source file "%s" (%s) is newer than destination "%s" (%s).';
  SInfoDestDoesNotExist   = 'Destination file "%s" does not exist.';
  SInfoFallbackBuildmode  = 'Buildmode not supported by package, falling back to one by one unit compilation';
  SInfoFallbackBuildmodeBU= 'Buildmode not supported by package, falling back to compilation using a buildunit';
=======
=======
>>>>>>> origin/fixes_2_2
  SInfoArchivingPackage   = 'Archiving package %s in "%s"';
  SInfoCleaningPackage    = 'Cleaning package %s';
  SInfoManifestPackage    = 'Creating manifest for package %s';
  SInfoCopyingFile        = 'Copying file "%s" to "%s"';
  SInfoSourceNewerDest    = 'Source file "%s" (%s) is newer than destination "%s" (%s).';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  SDbgComparingFileTimes    = 'Comparing file "%s" time "%s" to "%s" time "%s".';
  SDbgCompilingDependenciesOfTarget = 'Compiling dependencies of target %s';
  SDbgResolvingSourcesOfTarget = 'Resolving filenames of target %s for %s';
  SDbgResolvedSourceFile    = 'Resolved source file %s to "%s"';
<<<<<<< HEAD
<<<<<<< HEAD
  SDbgSourceAlreadyResolved = 'Source file of %s has been resolved earlier';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SDbgResolvedIncludeFile   = 'Resolved include file %s to "%s"';
  SDbgOutputNotYetAvailable = 'Output file %s not available';
  SDbgDependencyOnUnit      = 'Dependency of %s on unit %s';
  SDbgDependencyUnitRecompiled = 'Dependent unit %s is being recompiled';
<<<<<<< HEAD
<<<<<<< HEAD
  SDbgMustCompile           = 'Must compile %s. (%s)';
=======
  SDbgMustCompile           = 'Must compile %s';
>>>>>>> graemeg/fixes_2_2
=======
  SDbgMustCompile           = 'Must compile %s';
>>>>>>> origin/fixes_2_2
  SDbgSkippingTargetWrongCPU = 'Skipping target %s, different CPU (%s)';
  SDbgSkippingTargetWrongOS  = 'Skipping target %s, different OS (%s)';
  SDbgTargetIsNotAUnitOrProgram = 'Skipping Target %s, not an unit or program';
  SDbgConsideringTarget     = 'Considering target %s';
  SDbgConsideringPackage    = 'Considering package %s';
  SDbgExternalDependency    = 'External dependency %s found in "%s"';
  SDbgBuildEngineArchiving  = 'Build engine archiving';
  SDbgBuildEngineGenerateManifests = 'Build engine generating manifests';
<<<<<<< HEAD
<<<<<<< HEAD
  SDbgBuildEngineGeneratePkgList = 'Build engine generating package list';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
  SDbgPackageChecksumChanged = 'Dependent package %s is modified';
<<<<<<< HEAD
<<<<<<< HEAD
  SDbgFileDoesNotExist      = 'File "%s" does not exist';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SDbgDirectoryDoesNotExist = 'Directory "%s" does not exist';
  SDbgDirectoryNotEmpty     = 'Directory "%s" is not empty. Will not remove';
  SDbgGenerateBuildUnit     = 'Generate build-unit %s';
  SDbgForcedCompile         = 'Forced compile';
  SDbgOutputDoesNotExist    = 'Output file does not exist';
  SDbgNewerSource           = 'Source file is newer then output file';
  SDbgNewerInclude          = 'The include file %s is newer then output file';
  SDbgDependencyRecompiled  = 'The unit %s where this unit depends on is recompiled';
  SDbgPackageDepRecompiled  = 'The package %s where this package depends on is recompiled';
  SDbgTargetHasToBeCompiled = 'At least one of the targets in the package has to be compiled.';
  SDbgDeletedFile           = 'Recursively deleted file "%s"';
  SDbgRemovedDirectory      = 'Recursively removed directory "%s"';

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  // Help messages for usage
  SValue              = 'Value';
  SHelpUsage          = 'Usage: %s command [options]';
  SHelpCommand        = 'Where command is one of the following:';
  SHelpCompile        = 'Compile all units in the package(s).';
  SHelpBuild          = 'Build all units in the package(s).';
  SHelpInstall        = 'Install all units in the package(s).';
<<<<<<< HEAD
<<<<<<< HEAD
  SHelpUnInstall      = 'Uninstall the package(s).';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SHelpClean          = 'Clean (remove) all units in the package(s).';
  SHelpArchive        = 'Create archive (zip) with all units in the package(s).';
  SHelpHelp           = 'This message.';
  SHelpManifest       = 'Create a manifest suitable for import in repository.';
<<<<<<< HEAD
<<<<<<< HEAD
  SHelpPkgList        = 'Create list of all packages suitable for FPC installer.';
  SHelpZipInstall     = 'Install all units in the package(s) into an archive.';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  SHelpUnitInstallDir = 'Use indicated directory to install units into.';
  SHelpCompiler       = 'Use indicated binary as compiler';
  SHelpConfig         = 'Use indicated config file when compiling.';
  SHelpOptions        = 'Pass extra options to the compiler.';
  SHelpVerbose        = 'Be verbose when working.';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SHelpInstExamples   = 'Install the example-sources.';
  SHelpSkipCrossProgs = 'Skip programs when cross-compiling/installing';
  SHelpIgnoreInvOpt   = 'Ignore further invalid options.';
  sHelpFpdocOutputDir = 'Use indicated directory as fpdoc output folder.';
  sHelpFPUnitSrcPath  = 'Sourcepath to replace in fpunits.cfg on installation.';
  sHelpThreads        = 'Enable the indicated amount of worker threads.';
  sHelpUseEnvironment = 'Use environment to pass options to compiler.';
  SHelpUseBuildUnit   = 'Compile package in Build-unit mode.';
  sHelpZipPrefix      = 'Use indicated prefix for generated archives.';
=======
  SHelpIgnoreInvOpt   = 'Ignore further invalid options.';
  sHelpFpdocOutputDir = 'Use indicated directory as fpdoc output folder.';
>>>>>>> graemeg/cpstrnew
=======
  SHelpIgnoreInvOpt   = 'Ignore further invalid options.';
  sHelpFpdocOutputDir = 'Use indicated directory as fpdoc output folder.';
>>>>>>> graemeg/cpstrnew
=======
  SHelpIgnoreInvOpt   = 'Ignore further invalid options.';
  sHelpFpdocOutputDir = 'Use indicated directory as fpdoc output folder.';
>>>>>>> graemeg/cpstrnew
=======
  SHelpIgnoreInvOpt   = 'Ignore further invalid options.';
  sHelpFpdocOutputDir = 'Use indicated directory as fpdoc output folder.';
>>>>>>> origin/cpstrnew
=======
  SHelpCompiler       = 'Use indicated binary as compiler';
  SHelpConfig         = 'Use indicated config file when compiling.';
  SHelpVerbose        = 'Be verbose when working.';
>>>>>>> graemeg/fixes_2_2
=======
  SHelpCompiler       = 'Use indicated binary as compiler';
  SHelpConfig         = 'Use indicated config file when compiling.';
  SHelpVerbose        = 'Be verbose when working.';
>>>>>>> origin/fixes_2_2


Const
  // Keys for Defaults file. Do not localize.
  KeyCompiler = 'Compiler';
  KeyArchive  = 'Archive';
  KeyCopy     = 'Copy';
  KeyMkDir    = 'MkDir';
  KeyMove     = 'Move';
  KeyRemove   = 'Remove';
<<<<<<< HEAD
<<<<<<< HEAD
  KeyRemoveDir= 'RemoveDir';
  KeyRemoveTree= 'RemoveTree';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  KeyOptions  = 'Options';
  KeyCPU      = 'CPU';
  KeyOS       = 'OS';
  KeyMode     = 'Mode';
  KeyPrefix   = 'Prefix';
  KeyTarget   = 'Target';
  KeyNoFPCCfg = 'NoFPCCfg';
<<<<<<< HEAD
<<<<<<< HEAD
  KeyUseEnv   = 'UseEnv';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  KeyLocalUnitDir       = 'LocalUnitDir';
  KeyGlobalUnitDir      = 'GlobalUnitDir';
  KeyBaseInstallDir     = 'BaseInstallDir';
  KeyUnitInstallDir     = 'UnitInstallDir';
  KeyBinInstallDir      = 'BinInstallDir';
  KeyDocInstallDir      = 'DocInstallDir';
  KeyExamplesInstallDir = 'ExamplesInstallDir';
<<<<<<< HEAD
<<<<<<< HEAD
  KeyInstallExamples    = 'InstallExamples';
  KeySkipCrossProdrams  = 'SkipCrossPrograms';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // Keys for unit config
  KeyName     = 'Name';
  KeyVersion  = 'Version';
  KeyChecksum = 'Checksum';
  KeyNeedLibC = 'NeedLibC';
  KeyDepends  = 'Depends';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  KeyFlags    = 'Flags';
  KeyAddIn    = 'FPMakeAddIn';
  KeySourcePath = 'SourcePath';
  KeyFPMakeOptions = 'FPMakeOptions';
  KeyPackageVar = 'PackageVariant_';
=======
  KeyAddIn    = 'FPMakeAddIn';
  KeySourcePath = 'SourcePath';
  KeyFPMakeOptions = 'FPMakeOptions';
>>>>>>> graemeg/cpstrnew
=======
  KeyAddIn    = 'FPMakeAddIn';
  KeySourcePath = 'SourcePath';
  KeyFPMakeOptions = 'FPMakeOptions';
>>>>>>> graemeg/cpstrnew
=======
  KeyAddIn    = 'FPMakeAddIn';
  KeySourcePath = 'SourcePath';
  KeyFPMakeOptions = 'FPMakeOptions';
>>>>>>> graemeg/cpstrnew
=======
  KeyAddIn    = 'FPMakeAddIn';
  KeySourcePath = 'SourcePath';
  KeyFPMakeOptions = 'FPMakeOptions';
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{****************************************************************************
                                Helpers
****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef HAS_UNIT_PROCESS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ExecuteFPC(Verbose: boolean; const Path: string; const ComLine: string; const Env: TStrings; ConsoleOutput: TMemoryStream): integer;
var
  P: TProcess;
  BytesRead: longint;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ExecuteFPC(Verbose: boolean; const Path: string; const ComLine: string; ConsoleOutput: TMemoryStream): integer;
var
  P: TProcess;
  BytesRead: longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======

  function ReadFromStream: longint;

  const
    READ_BYTES = 2048;

  type
    TMessages = (mCompiling, mLinking);

  var
    //ifdef the MsgNum so it contains the correct message numbers for each compiler version.
    MsgNum : array [TMessages] of integer = (3104, 9015);

    n: longint;
    BuffPos: longint;
    sLine: string;
    ch: char;
    msg: TMessages;
    ipos: integer;
    snum: string;
  begin
    // make sure we have room
    ConsoleOutput.SetSize(BytesRead + READ_BYTES);

    // try reading it
    n := P.Output.Read((ConsoleOutput.Memory + BytesRead)^, READ_BYTES);
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
          if Verbose then
            writeln(sLine)
          else
            begin
              for msg := Low(TMessages) to High(TMessages) do
              begin
                snum := Format('(%d)', [MsgNum[msg]]);
                ipos := Pos(snum, sLine);
                if ipos = 1 then
                  writeln('      ', Copy(sLine, ipos + Length(snum), Length(sLine) - ipos - Length(snum) + 1));
              end;
            end;

          sLine := '';
          BuffPos := ConsoleOutput.Position;
        end
        else
          sLine := sLine + ch;

      until ConsoleOutput.Position >= BytesRead;

      ConsoleOutput.Position := BuffPos;
    end
    else
    begin
      // no data, wait 100 ms
      Sleep(100);
    end;

    Result := n;
  end;

begin
  result := -1;
  BytesRead := 0;
  P := TProcess.Create(nil);
  try
    if Verbose then
      P.CommandLine := Path + ' ' + ComLine
    else
      P.CommandLine := Path + ' -viq ' + ComLine;

    P.Options := [poUsePipes];

    P.Execute;
    while P.Running do
      ReadFromStream;

    // read last part
    repeat
    until ReadFromStream = 0;
    ConsoleOutput.SetSize(BytesRead);

    result := P.ExitStatus;
  finally
    P.Free;
  end;
end;
{$endif HAS_UNIT_PROCESS}

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
>>>>>>> origin/cpstrnew

  function ReadFromStream: longint;

  const
    READ_BYTES = 2048;

  type
    TMessages = (mCompiling, mLinking);

  var
    //ifdef the MsgNum so it contains the correct message numbers for each compiler version.
    MsgNum : array [TMessages] of integer = (3104, 9015);

    n: longint;
    BuffPos: longint;
    sLine: string;
    ch: char;
    msg: TMessages;
    ipos: integer;
    snum: string;
  begin
    // make sure we have room
    ConsoleOutput.SetSize(BytesRead + READ_BYTES);

    // try reading it
    n := P.Output.Read((ConsoleOutput.Memory + BytesRead)^, READ_BYTES);
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
          if Verbose then
            writeln(sLine)
          else
            begin
              for msg := Low(TMessages) to High(TMessages) do
              begin
                snum := Format('(%d)', [MsgNum[msg]]);
                ipos := Pos(snum, sLine);
                if ipos = 1 then
                  writeln('      ', Copy(sLine, ipos + Length(snum), Length(sLine) - ipos - Length(snum) + 1));
              end;
            end;

          sLine := '';
          BuffPos := ConsoleOutput.Position;
        end
        else
          sLine := sLine + ch;

      until ConsoleOutput.Position >= BytesRead;

      ConsoleOutput.Position := BuffPos;
    end
    else
    begin
      // no data, wait 100 ms
      Sleep(100);
    end;

    Result := n;
  end;

begin
  result := -1;
  BytesRead := 0;
  P := TProcess.Create(nil);
  try
    if Verbose then
      P.CommandLine := Path + ' ' + ComLine
    else
      P.CommandLine := Path + ' -viq ' + ComLine;

    P.Options := [poUsePipes];

    P.Execute;
    while P.Running do
      ReadFromStream;

    // read last part
    repeat
    until ReadFromStream = 0;
    ConsoleOutput.SetSize(BytesRead);

    result := P.ExitStatus;
  finally
    P.Free;
  end;
end;
{$endif HAS_UNIT_PROCESS}

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
>>>>>>> graemeg/cpstrnew

  function ReadFromStream: longint;

  const
    READ_BYTES = 2048;

  type
    TMessages = (mCompiling, mLinking);

  var
    //ifdef the MsgNum so it contains the correct message numbers for each compiler version.
    MsgNum : array [TMessages] of integer = (3104, 9015);

    n: longint;
    BuffPos: longint;
    sLine: string;
    ch: char;
    msg: TMessages;
    ipos: integer;
    snum: string;
  begin
    // make sure we have room
    ConsoleOutput.SetSize(BytesRead + READ_BYTES);

    // try reading it
    n := P.Output.Read((ConsoleOutput.Memory + BytesRead)^, READ_BYTES);
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
          if Verbose then
            writeln(sLine)
          else
            begin
              for msg := Low(TMessages) to High(TMessages) do
              begin
                snum := Format('(%d)', [MsgNum[msg]]);
                ipos := Pos(snum, sLine);
                if ipos = 1 then
                  writeln('      ', Copy(sLine, ipos + Length(snum), Length(sLine) - ipos - Length(snum) + 1));
              end;
            end;

          sLine := '';
          BuffPos := ConsoleOutput.Position;
        end
        else
          sLine := sLine + ch;

      until ConsoleOutput.Position >= BytesRead;

      ConsoleOutput.Position := BuffPos;
    end
    else
    begin
      // no data, wait 100 ms
      Sleep(100);
    end;

    Result := n;
  end;

begin
  result := -1;
  BytesRead := 0;
  P := TProcess.Create(nil);
  try
    if Verbose then
      P.CommandLine := Path + ' ' + ComLine
    else
      P.CommandLine := Path + ' -viq ' + ComLine;

    P.Options := [poUsePipes];

    P.Execute;
    while P.Running do
      ReadFromStream;

    // read last part
    repeat
    until ReadFromStream = 0;
    ConsoleOutput.SetSize(BytesRead);

    result := P.ExitStatus;
  finally
    P.Free;
  end;
end;
{$endif HAS_UNIT_PROCESS}

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
>>>>>>> graemeg/cpstrnew

  function ReadFromStream: longint;

  const
    READ_BYTES = 2048;

  type
    TMessages = (mCompiling, mLinking);

  var
    //ifdef the MsgNum so it contains the correct message numbers for each compiler version.
    MsgNum : array [TMessages] of integer = (3104, 9015);

    n: longint;
    BuffPos: longint;
    sLine: string;
    ch: char;
    msg: TMessages;
    ipos: integer;
    snum: string;
  begin
    // make sure we have room
    ConsoleOutput.SetSize(BytesRead + READ_BYTES);

    // try reading it
    n := P.Output.Read((ConsoleOutput.Memory + BytesRead)^, READ_BYTES);
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
          if Verbose then
            writeln(sLine)
          else
            begin
              for msg := Low(TMessages) to High(TMessages) do
              begin
                snum := Format('(%d)', [MsgNum[msg]]);
                ipos := Pos(snum, sLine);
                if ipos = 1 then
                  writeln('      ', Copy(sLine, ipos + Length(snum), Length(sLine) - ipos - Length(snum) + 1));
              end;
            end;

          sLine := '';
          BuffPos := ConsoleOutput.Position;
        end
        else
          sLine := sLine + ch;

      until ConsoleOutput.Position >= BytesRead;

      ConsoleOutput.Position := BuffPos;
    end
    else
    begin
      // no data, wait 100 ms
      Sleep(100);
    end;

    Result := n;
  end;

begin
  result := -1;
  BytesRead := 0;
  P := TProcess.Create(nil);
  try
    if Verbose then
      P.CommandLine := Path + ' ' + ComLine
    else
      P.CommandLine := Path + ' -viq ' + ComLine;

    P.Options := [poUsePipes];

    P.Execute;
    while P.Running do
      ReadFromStream;

    // read last part
    repeat
    until ReadFromStream = 0;
    ConsoleOutput.SetSize(BytesRead);

    result := P.ExitStatus;
  finally
    P.Free;
  end;
end;
{$endif HAS_UNIT_PROCESS}

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
>>>>>>> graemeg/cpstrnew

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
          if Verbose then
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
                sLine:=ch;
            end
          else
            sLine := '';
          BuffPos := ConsoleOutput.Position;
        end
        else
          sLine := sLine + ch;

      until ConsoleOutput.Position >= BytesRead;

      ConsoleOutput.Position := BuffPos;
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

procedure SplitCommand(const Cmd : String; var Exe, Options : String);

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AddProgramExtension(const ExecutableName: string; AOS : TOS): string;
begin
  if AOS in [Go32v2,Win32,Win64,Wince,OS2,EMX,Watcom] then
    Result:=ExecutableName+ExeExt
  else
    Result:=ExecutableName;
end;

function GetImportLibraryFilename(const UnitName: string; AOS: TOS): string;
begin
  if AOS in [go32v2,watcom] then
    Result := 'libimp'+UnitName
  else if AOS in [os2,emx] then
    Result := UnitName
  else if AOS in [netware,netwlibc,macos] then
    Result := 'lib'+UnitName
  else
    Result := 'libimp'+UnitName;
  Result := Result + LibExt;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
function GetCompilerInfo(const ACompiler,AOptions:string; ReadStdErr: boolean):string;
const
  BufSize = 1024;
var
  S: TProcess;
  Buf: array [0..BufSize - 1] of char;
  Count: longint;
begin
  S:=TProcess.Create(Nil);
  S.Commandline:=ACompiler+' '+AOptions;
  S.Options:=[poUsePipes,poWaitOnExit];
  S.execute;
  Count:=s.output.read(buf,BufSize);
  if (count=0) and ReadStdErr then
    Count:=s.Stderr.read(buf,BufSize);
  S.Free;
  SetLength(Result,Count);
  Move(Buf,Result[1],Count);
end;
{$endif HAS_UNIT_PROCESS}

function GetDefaultLibGCCDir(CPU : TCPU;OS: TOS; var ErrorMessage: string): string;

<<<<<<< HEAD
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
      ExecResult:=GetCompilerInfo(GccExecutable,'-v '+GCCParams, True);
      libgccFilename:=Get4thWord(ExecResult);
      // Use IsRelativePath to check if the 4th word is an (absolute) path.
      // This depends on the language settings. In English the 4th word is
      // empty, if this particular gcc version does not return the libgcc-
      // filename on -v. But in other languages (e.g. Dutch) it may be another
      // word.
      if IsRelativePath(libgccFilename) then
        libgccFilename:='';
      if libgccFilename='' then
        libgccFilename:=GetCompilerInfo(GccExecutable,'--print-libgcc-file-name '+GCCParams, False);
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
      aarch64:  result := GetGccDirArch('cpuaarch64','');
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
  inherited Create(false);
  FNotifyStartTask := RTLEventCreate;
  FBuildEngine := ABuildEngine;
  FNotifyMainThreadEvent:=NotifyMainThreadEvent;
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
    FDone:=true;
    RTLeventSetEvent(FNotifyMainThreadEvent);
    RTLeventWaitFor(FNotifyStartTask,500);
    if not FDone then
      begin
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

<<<<<<< HEAD
{$endif NO_THREADING}
=======
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

Function OptionListToString(L : TStrings) : String;
>>>>>>> origin/cpstrnew

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

=======
>>>>>>> graemeg/cpstrnew
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

Function TTargets.AddUnit(Const AUnitName : String) : TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,AllOSes);
end;


Function TTargets.AddUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.TargetType:=TTUnit;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,AllOSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,OSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,CPUs,AllOSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
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


Function TTargets.AddProgram(Const AProgramName : String) : TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,AllOSes);
end;


Function TTargets.AddProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttProgram;
end;


Function TTargets.AddExampleUnit(Const AUnitName : String) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,AllOSes);
end;


Function TTargets.AddExampleUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleUnit;
end;


Function TTargets.AddExampleProgram(Const AProgramName : String) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,AllOSes);
end;


Function TTargets.AddExampleProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleProgram;
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

<<<<<<< HEAD
procedure TSources.AddDocFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean; AInstallSourcePath: String);
=======

procedure TSources.AddDocFiles(const AFileMask: string; Recursive: boolean; AInstallSourcePath : String = '');
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TSources.AddExampleFiles(const AFileMask, ASearchPathPrefix: string; Recursive: boolean; AInstallSourcePath: String);
=======
procedure TSources.AddExampleFiles(const AFileMask: string; Recursive: boolean; AInstallSourcePath : String = '');
>>>>>>> graemeg/cpstrnew
=======
procedure TSources.AddExampleFiles(const AFileMask: string; Recursive: boolean; AInstallSourcePath : String = '');
>>>>>>> graemeg/cpstrnew
=======
procedure TSources.AddExampleFiles(const AFileMask: string; Recursive: boolean; AInstallSourcePath : String = '');
>>>>>>> graemeg/cpstrnew
=======
procedure TSources.AddExampleFiles(const AFileMask: string; Recursive: boolean; AInstallSourcePath : String = '');
>>>>>>> origin/cpstrnew
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
end;


destructor TPackage.destroy;
var
  i: integer;
begin
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
  List.Add(GetUnitConfigOutputFilename(Defaults.CPU,Defaults.OS));
  AddConditionalStrings(Self, List,CleanFiles,ACPU,AOS);
  For I:=0 to FTargets.Count-1 do
    FTargets.TargetItems[I].GetCleanFiles(List, OU, OB, ACPU, AOS);
end;


procedure TPackage.GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU:TCPU; AOS : TOS);
Var
  OB,OU : String;
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


procedure TPackage.GetInstallSourceFiles(List: TStrings; Types: TSourceTypes);
Var
  I : Integer;
  S : TSource;
begin
  For I:=0 to FSources.Count-1 do
    begin
      S:=FSources.SourceItems[I];
      if (S.SourceType in Types) then
        S.GetInstallFiles(List);
    end;
end;


procedure TPackage.GetInstallSourceFiles(List: TStrings; Types: TSourceTypes);
Var
  I : Integer;
  S : TSource;
begin
  For I:=0 to FSources.Count-1 do
    begin
      S:=FSources.SourceItems[I];
      if (S.SourceType in Types) then
        S.GetInstallFiles(List);
    end;
end;


procedure TPackage.GetInstallSourceFiles(List: TStrings; Types: TSourceTypes);
Var
  I : Integer;
  S : TSource;
begin
  For I:=0 to FSources.Count-1 do
    begin
      S:=FSources.SourceItems[I];
      if (S.SourceType in Types) then
        S.GetInstallFiles(List);
    end;
end;


procedure TPackage.GetInstallSourceFiles(List: TStrings; Types: TSourceTypes);
Var
  I : Integer;
  S : TSource;
begin
  For I:=0 to FSources.Count-1 do
    begin
      S:=FSources.SourceItems[I];
      if (S.SourceType in Types) then
        S.GetInstallFiles(List);
=======

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
  {$IFDEF MSWINDOWS}
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
  F:=TFileStream.Create(AFileName,fmOpenRead);
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

begin
  Result:=LowerCase(SetToString(PtypeInfo(TypeInfo(TOSes)),Integer(OSes),False));
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


Function OSesToString(const S : String) : TOSes;

begin
  Result:=TOSes(StringToSet(PTypeInfo(TypeInfo(TOSes)),S));
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


Function MakeTargetString(CPU : TCPU;OS: TOS) : String;

begin
  Result:=CPUToString(CPU)+'-'+OSToString(OS);
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


function AddConditionalStrings(Dest : TStrings; Src : TConditionalStrings;ACPU:TCPU;AOS:TOS; Const APrefix : String='') : Integer ;
Var
  I : Integer;
  C : TConditionalString;
  S : String;
begin
  Result:=0;
  Dictionary.AddVariable('CPU',CPUToString(ACPU));
  Dictionary.AddVariable('OS',OSToString(AOS));
  For I:=0 to Src.Count-1 do
    begin
      C:=Src[I];
      if (ACPU in C.CPUs) and (AOS in C.OSes) then
        begin
          If (APrefix<>'') then
            S:=APrefix+C.Value
          else
            S:=C.Value;
          Dest.Add(Dictionary.ReplaceStrings(S));
          Inc(Result);
        end;
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
Var
  P : PChar;
begin
  Result:=APath;
  If (result<>'') then
    begin
      P:=PChar(Result);
      While (P^<>#0) do
        begin
          If P^ in ['/','\'] then
            P^:=PathDelim;
          Inc(P);
        end;
    end;
end;


procedure ChangeDir(const APath : String);
begin
  if Not SetCurrentDir(APath) then
    Raise EInstallerError.CreateFmt(SErrChangeDirFailed,[APath]);
end;


procedure SearchFiles(const AFileName: string; Recursive: boolean; var List: TStrings);

  procedure AddRecursiveFiles(const SearchDir, FileMask: string; Recursive: boolean);
  const
    IgnoreCase = {$ifdef UNIX}False{$else}True{$endif};
  var
    Info : TSearchRec;
  begin
    if FindFirst(SearchDir+'*',faAnyFile and faDirectory,Info)=0 then
    begin
      repeat
          if ((Info.Attr and faDirectory) = faDirectory) and (Info.Name <> '.') and (Info.Name <> '..') and (Recursive) then
            AddRecursiveFiles(SearchDir + Info.Name + PathDelim, FileMask, Recursive);
          if ((Info.Attr and faDirectory) <> faDirectory) and IsWild(Info.Name, FileMask, IgnoreCase) then
            List.Add(SearchDir + Info.Name);
      until FindNext(Info)<>0;
    end;
    FindClose(Info);
  end;

var
  CurrDir,
  BasePath: string;
  i: integer;
begin
  BasePath := ExtractFilePath(ExpandFileName(AFileName));
  AddRecursiveFiles(BasePath, ExtractFileName(AFileName), Recursive);

  CurrDir:=GetCurrentDir;
  for i := 0 to Pred(List.Count) do
    List[i] := ExtractRelativepath(IncludeTrailingPathDelimiter(CurrDir), List[i]);
end;


procedure SplitCommand(const Cmd : String; var Exe, Options : String);

Const
  WhiteSpace = [#9,#10,#13,' '];
  QuoteChars = ['''','"'];

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


{$ifdef HAS_UNIT_PROCESS}
function GetCompilerInfo(const ACompiler,AOptions:string):string;
const
  BufSize = 1024;
var
  S: TProcess;
  Buf: array [0..BufSize - 1] of char;
  Count: longint;
begin
  S:=TProcess.Create(Nil);
  S.Commandline:=ACompiler+' '+AOptions;
  S.Options:=[poUsePipes];
  S.execute;
  Count:=s.output.read(buf,BufSize);
  S.Free;
  SetLength(Result,Count);
  Move(Buf,Result[1],Count);
end;
{$endif HAS_UNIT_PROCESS}


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


Function TTargets.AddUnit(Const AUnitName : String) : TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,AllOSes);
end;


Function TTargets.AddUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.TargetType:=TTUnit;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,AllOSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,OSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,CPUs,AllOSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
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


Function TTargets.AddProgram(Const AProgramName : String) : TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,AllOSes);
end;


Function TTargets.AddProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttProgram;
end;


Function TTargets.AddExampleUnit(Const AUnitName : String) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,AllOSes);
end;


Function TTargets.AddExampleUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleUnit;
end;


Function TTargets.AddExampleProgram(Const AProgramName : String) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,AllOSes);
end;


Function TTargets.AddExampleProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleProgram;
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


function TSources.AddTest(const AFiles : String) : TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FSourceType:=stTest;
end;


procedure TSources.AddDocFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddDoc(List[i]);
  List.Free;
end;


procedure TSources.AddSrcFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddSrc(List[i]);
  List.Free;
end;


procedure TSources.AddExampleFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddExample(List[i]);
  List.Free;
end;


procedure TSources.AddTestFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
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
  FInstallFiles:=TConditionalStrings.Create(TConditionalString);
  FCleanFiles:=TConditionalStrings.Create(TConditionalString);
  FUnitPath:=TConditionalStrings.Create(TConditionalString);
  FObjectPath:=TConditionalStrings.Create(TConditionalString);
  FIncludePath:=TConditionalStrings.Create(TConditionalString);
  FSourcePath:=TConditionalStrings.Create(TConditionalString);
  FExamplePath:=TConditionalStrings.Create(TConditionalString);
  FTestPath:=TConditionalStrings.Create(TConditionalString);
  FCommands:=TCommands.Create(TCommand);
  FCPUs:=AllCPUs;
  FOSes:=AllOSes;
  FInstalledChecksum:=$ffffffff;
  // Implicit dependency on RTL
  FDependencies.Add('rtl');
end;


destructor TPackage.destroy;
begin
  FreeAndNil(FDependencies);
  FreeAndNil(FInstallFiles);
  FreeAndNil(FCleanFiles);
  FreeAndNil(FIncludePath);
  FreeAndNil(FSourcePath);
  FreeAndNil(FExamplePath);
  FreeAndNil(FTestPath);
  FreeAndNil(FObjectPath);
  FreeAndNil(FUnitPath);
  FreeAndNil(FSources);
  FreeAndNil(FTargets);
  FreeAndNil(FVersion);
  inherited destroy;
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
  Result:='units'+PathDelim+MakeTargetString(ACPU,AOS);
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
  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaults.OS));
  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  AddConditionalStrings(List,CleanFiles,ACPU,AOS);
  For I:=0 to FTargets.Count-1 do
    FTargets.TargetItems[I].GetCleanFiles(List, OU, OB, ACPU, AOS);
end;


procedure TPackage.GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU:TCPU; AOS : TOS);
Var
  OB,OU : String;
  I : Integer;
  T : TTarget;
begin
  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaults.OS));
  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  AddConditionalStrings(List,InstallFiles,ACPU,AOS);
  For I:=0 to FTargets.Count-1 do
    begin
      T:=FTargets.TargetItems[I];
      if (T.TargetType in Types) and (T.Install) then
        T.GetInstallFiles(List, OU, OB, ACPU, AOS);
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


Function TPackage.GetVersion : string;
begin
  result:=FVersion.AsString;
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
    if not FVersion.Empty then
      Result := Name + '-' + FVersion.AsString
    else
      Result := Name;
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

Var
  S : String;
  i : Integer;
  D : TDependency;
begin
  With Manifest do
    begin
    Add(Format('<package name="%s">',[QuoteXml(Name)]));
    Add(Format(' <version major="%d" minor="%d" micro="%d" build="%d"/>',[FVersion.Major,FVersion.Minor,FVersion.Micro,FVersion.Build]));
    AddOSes(' ',OSes);
    AddCPUs(' ',CPUs);
    Add(Format(' <filename>%s</filename>',[QuoteXml(FileName + ZipExt)]));
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
              Add(Format('   <version major="%d" minor="%d" micro="%d" build="%d"/>',[D.FVersion.Major,D.FVersion.Minor,D.FVersion.Micro,D.FVersion.Build]));
            AddOSes('   ',D.OSes);
            AddCPUs('   ',D.CPUs);
            Add('  </dependency>');
          end;
        Add(' </dependencies>');
      end;
    Add('</package>');
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
      end;
  Finally
    L.Free;
  end;
end;


procedure TPackage.SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;AOS:TOS);
Var
  F : TFileStream;
  L : TStringList;
  Deps : String;
  i : integer;
  D : TDependency;
  p : TPackage;
begin
  F:=TFileStream.Create(AFileName,fmCreate);
  L:=TStringList.Create;
  try
    With L do
      begin
        Values[KeyName]:=Name;
        Values[KeyVersion]:=Version;
        // TODO Generate checksum based on PPUs
        Values[KeyChecksum]:=IntToStr(DateTimeToFileDate(Now));
        Values[KeyCPU]:=CPUToString(ACPU);
        Values[KeyOS]:=OSToString(AOS);
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
        if NeedLibC then
          Values[KeyNeedLibC]:='Y'
        else
          Values[KeyNeedLibC]:='N';
      end;
    L.SaveToStream(F);
  Finally
    L.Free;
    F.Free;
  end;
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
begin
  Result:=Add as TPackage;
  Result.Name:=AName;
end;


{****************************************************************************
                             TCustomDefaults
****************************************************************************}

procedure TCustomDefaults.SetCPU(const AValue: TCPU);
begin
  FCPU:=AValue;
  RecalcTarget;
end;


function TCustomDefaults.GetBaseInstallDir: String;
begin
  If (FBaseInstallDir<>'') then
    Result:=FBaseInstallDir
  else
    if UnixPaths then
      Result:=Prefix +'lib' + PathDelim + 'fpc' + PathDelim
    else
      Result:=Prefix;
end;


function TCustomDefaults.GetBinInstallDir: String;
begin
  If (FBinInstallDir<>'') then
    Result:=FBinInstallDir
  else
    If UnixPaths then
      Result:=BaseInstallDir+'bin'
    else
      Result:=BaseInstallDir+'bin';
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
  If (FBinInstallDir<>'') then
    Result:=FBinInstallDir
  else
    If UnixPaths then
      Result:=Prefix+'share'+PathDelim+'doc'
    else
      Result:=BaseInstallDir+'docs';
end;


function TCustomDefaults.GetExamplesInstallDir: String;
begin
  If (FExamplesInstallDir<>'') then
    Result:=FExamplesInstallDir
  else
    If UnixPaths then
      Result:=Prefix+'share'+PathDelim+'docs'+PathDelim+'examples'
    else
      Result:=BaseInstallDir+'examples';
end;


function TCustomDefaults.GetUnitInstallDir: String;
begin
  If (FUnitInstallDir<>'') then
    Result:=FUnitInstallDir
  else
    If UnixPaths then
      Result:=BaseInstallDir+'units'+PathDelim+Target
    else
      Result:=BaseInstallDir+'units'+PathDelim+Target;
end;


function TCustomDefaults.GetLocalUnitDir: String;
begin
  Result:=FLocalUnitDir;
end;


function TCustomDefaults.GetGlobalUnitDir: String;
begin
  If (FGlobalUnitDir<>'') then
    Result:=FGlobalUnitDir
  else
    Result:=UnitInstallDir;
end;


procedure TCustomDefaults.SetLocalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FLocalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FLocalUnitDir:='';
end;


procedure TCustomDefaults.SetGlobalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FGlobalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FGlobalUnitDir:='';
end;


procedure TCustomDefaults.SetBaseInstallDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FBaseInstallDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FBaseInstallDir:='';
  UnitInstallDir:='';
  BinInstallDir:='';
  ExamplesInstallDir:='';
end;


procedure TCustomDefaults.SetOS(const AValue: TOS);
begin
  FOS:=AValue;
  Recalctarget;
end;


procedure TCustomDefaults.SetPrefix(const AValue: String);
begin
  if FPrefix=AValue then exit;
  FPrefix:=IncludeTrailingPathDelimiter(AValue);
  BaseInstallDir:='';
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
          FCPU:=StringToCPU(System.Copy(Avalue,1,P-1));
        end
      else
        FOS:=StringToOS(AValue);
      FTarget:=AValue;
    end;
end;


procedure TCustomDefaults.RecalcTarget;
begin
  Ftarget:=CPUToString(FCPU)+'-'+OStoString(FOS);
end;


constructor TCustomDefaults.Create;
begin
  InitDefaults;
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
end;


procedure TCustomDefaults.LocalInit(Const AFileName : String);
Var
  FN : String;
begin
  FN:=AFileName;
  If (FN='') then
    begin
    // Environment variable.
    FN:=GetEnvironmentVariable('FPMAKECFG');
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
  if (CPU=cpuNone) or (OS=osNone) or (FCompilerVersion='') then
    begin
{$ifdef HAS_UNIT_PROCESS}
      // Detect compiler version/target from -i option
      infosl:=TStringList.Create;
      infosl.Delimiter:=' ';
      infosl.DelimitedText:=GetCompilerInfo(GetCompiler,'-iVTPTO');
      if infosl.Count<>3 then
        Raise EInstallerError.Create(SErrInvalidFPCInfo);
      if FCompilerVersion='' then
        FCompilerVersion:=infosl[0];
      if CPU=cpuNone then
        CPU:=StringToCPU(infosl[1]);
      if OS=osNone then
        OS:=StringToOS(infosl[2]);
{$else HAS_UNIT_PROCESS}
      // Defaults taken from compiler used to build fpmake
      if CPU=cpuNone then
        CPU:=StringToCPU({$I %FPCTARGETCPU%});
      if OS=osNone then
        OS:=StringToOS({$I %FPCTARGETOS%});
      if FCompilerVersion='' then
        FCompilerVersion:={$I %FPCVERSION%};
{$endif HAS_UNIT_PROCESS}
    end;
end;


procedure TCustomDefaults.LoadFromFile(Const AFileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(AFileName,fmOpenRead);
  Try
    LoadFromStream(F);
  Finally
    F.Free;
  end;
end;


procedure TCustomDefaults.SaveToFile(Const AFileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(AFileName,fmCreate);
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
      Values[KeyOptions]:=FOptions;
      Values[KeyCPU]:=CPUToString(FCPU);
      Values[KeyOS]:=OSToString(FOS);
      Values[KeyMode]:=ModeToString(FMode);
      Values[KeyLocalUnitDir]:=FLocalUnitDir;
      Values[KeyGlobalUnitDir]:=FGlobalUnitDir;
      Values[KeyPrefix]:=FPrefix;
      Values[KeyBaseInstallDir]:=FBaseInstallDir;
      Values[KeyUnitInstallDir]:=FUnitInstallDir;
      Values[KeyBinInstallDir]:=FBinInstallDir;
      Values[KeyDocInstallDir]:=FDocInstallDir;
      Values[KeyExamplesInstallDir]:=FExamplesInstallDir;
      Values[KeyRemove]:=FRemove;
      Values[KeyTarget]:=FTarget;
      if FNoFPCCfg then
        Values[KeyNoFPCCfg]:='Y';
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
      FOptions:=Values[KeyOptions];
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
      FLocalUnitDir:=Values[KeyLocalUnitDir];
      FGlobalUnitDir:=Values[KeyGlobalUnitDir];
      FPrefix:=Values[KeyPrefix];
      FBaseInstallDir:=Values[KeyBaseInstallDir];
      FUnitInstallDir:=Values[KeyUnitInstallDir];
      FBinInstallDir:=Values[KeyBinInstallDir];
      FDocInstallDir:=Values[KeyDocInstallDir];
      FExamplesInstallDir:=Values[KeyExamplesInstallDir];
      FNoFPCCfg:=(Upcase(Values[KeyNoFPCCfg])='Y');
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
  BD:=FixPath(GetEnvironmentVariable('FPCDIR'));
  if BD='' then
    begin
      BD:='/usr/local/lib/fpc/'+FCompilerVersion;
      if not DirectoryExists(BD) and
         DirectoryExists('/usr/lib/fpc/'+FCompilerVersion) then
        BD:='/usr/lib/fpc/'+FCompilerVersion;
    end;
{$else unix}
  BD:=FixPath(GetEnvironmentVariable('FPCDIR'));
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

  // Where to find the units by default
  if (FGlobalUnitDir='') then
    GlobalUnitDir:=IncludeTrailingPathDelimiter(BD)+'units'+PathDelim+Target;
end;


{****************************************************************************
                            TCustomInstaller
****************************************************************************}

constructor TCustomInstaller.Create(AOwner: TComponent);
begin
  Dictionary:=DictionaryClass.Create(Nil);
  AnalyzeOptions;
  CreatePackages;
end;


destructor TCustomInstaller.Destroy;
begin
  FreeAndNil(Defaults);
  FreeAndNil(Dictionary);
  inherited destroy;
end;


procedure TCustomInstaller.Log(Level: TVerboseLevel; const Msg: String);
begin
  If Level in FLogLevels then
    Writeln(StdOut,Msg);
end;


procedure TCustomInstaller.CreatePackages;
begin
  FPAckages:=TPackages.Create(TPackage);
end;


procedure TCustomInstaller.CreateBuildEngine;
begin
  FBuildEngine:=TBuildEngine.Create(Self);
//  FBuildEngine.Defaults:=Defaults;
  FBuildEngine.ListMode:=FListMode;
  FBuildEngine.OnLog:=@Self.Log;
end;


procedure TCustomInstaller.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TCustomInstaller.Error(const Fmt: String; Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


Function TCustomInstaller.AddPackage(const AName: String) : TPackage;
begin
  result:=FPackages.AddPackage(AName);
end;


procedure TCustomInstaller.AnalyzeOptions;

  Function CheckOption(Index : Integer;const Short,Long : String): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O='--'+long) or (copy(O,1,Length(Long)+3)=('--'+long+'='));
  end;

  Function CheckCommand(Index : Integer;const Short,Long : String): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O=long);
  end;

  Function OptionArg(Var Index : Integer) : String;
  Var
    P : Integer;
  begin
    if (Length(ParamStr(Index))>1) and (Paramstr(Index)[2]<>'-') then
      begin
      If Index<ParamCount then
        begin
        Inc(Index);
        Result:=Paramstr(Index);
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

Var
  I : Integer;
  DefaultsFileName : string;
begin
  I:=0;
  FListMode:=False;
  FLogLevels:=DefaultMessages;
  While (I<ParamCount) do
    begin
    Inc(I);
    if CheckOption(I,'v','verbose') then
      FLogLevels:=AllMessages
    else if CheckOption(I,'d','debug') then
      FLogLevels:=AllMessages+[vlDebug]
    else if CheckCommand(I,'m','compile') then
      FRunMode:=rmCompile
    else if CheckCommand(I,'b','build') then
      FRunMode:=rmBuild
    else if CheckCommand(I,'i','install') then
      FRunMode:=rmInstall
    else if CheckCommand(I,'c','clean') then
      FRunMode:=rmClean
    else if CheckCommand(I,'a','archive') then
      FRunMode:=rmarchive
    else if CheckCommand(I,'M','manifest') then
      FRunMode:=rmManifest
    else if CheckOption(I,'h','help') then
      Usage('',[])
    else if Checkoption(I,'C','cpu') then
      Defaults.CPU:=StringToCPU(OptionArg(I))
    else if Checkoption(I,'O','os') then
      Defaults.OS:=StringToOS(OptionArg(I))
    else if Checkoption(I,'t','target') then
      Defaults.Target:=OptionArg(I)
    else if CheckOption(I,'l','list-commands') then
      FListMode:=True
    else if Checkoption(I,'P','prefix') then
      Defaults.Prefix:=OptionArg(I)
    else if Checkoption(I,'n','nofpccfg') then
      Defaults.NoFPCCfg:=true
    else if CheckOption(I,'B','baseinstalldir') then
      Defaults.BaseInstallDir:=OptionArg(I)
    else if CheckOption(I,'UL','localunitdir') then
      Defaults.LocalUnitDir:=OptionArg(I)
    else if CheckOption(I,'UG','globalunitdir') then
      Defaults.GlobalUnitDir:=OptionArg(I)
    else if CheckOption(I,'r','compiler') then
      Defaults.Compiler:=OptionArg(I)
    else if CheckOption(I,'f','config') then
      DefaultsFileName:=OptionArg(I)
    else
      begin
      Usage(SErrInValidArgument,[I,ParamStr(I)]);
      end;
    end;
  If DefaultsFileName<>'' then
    Defaults.LocalInit(DefaultsFileName);
  Defaults.CompilerDefaults;
end;


procedure TCustomInstaller.Usage(const FMT: String; Args: array of const);

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
    Log(vlInfo,Format(' -%s --%-20s %s',[C,LC+'='+SValue,MSG]));
  end;

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
  LogCmd('clean',SHelpClean);
  LogCmd('archive',SHelpArchive);
  LogCmd('manifest',SHelpManifest);
  Log(vlInfo,SHelpCmdOptions);
  LogOption('h','help',SHelpHelp);
  LogOption('l','list-commands',SHelpList);
  LogOption('n','nofpccfg',SHelpNoFPCCfg);
  LogOption('v','verbose',SHelpVerbose);
  LogArgOption('C','cpu',SHelpCPU);
  LogArgOption('O','os',SHelpOS);
  LogArgOption('t','target',SHelpTarget);
  LogArgOption('P','prefix',SHelpPrefix);
  LogArgOption('B','baseinstalldir',SHelpBaseInstalldir);
  LogArgOption('UL','localunitdir',SHelpLocalUnitdir);
  LogArgOption('UG','globalunitdir',SHelpGlobalUnitdir);
  LogArgOption('r','compiler',SHelpCompiler);
  LogArgOption('f','config',SHelpConfig);
  Log(vlInfo,'');
  If (FMT<>'') then
    halt(1)
  else
    halt(0);
end;


procedure TCustomInstaller.Compile(Force: Boolean);
begin
  FBuildEngine.ForceCompile:=Force;
  FBuildEngine.Compile(FPackages);
end;


procedure TCustomInstaller.Clean;
begin
  BuildEngine.Clean(FPackages);
end;


procedure TCustomInstaller.Install;
begin
  BuildEngine.Install(FPackages);
end;


procedure TCustomInstaller.Archive;
begin
  // Force generation of manifest.xml, this is required for the repository
  BuildEngine.Manifest(FPackages);
  BuildEngine.Archive(FPackages);
end;


procedure TCustomInstaller.Manifest;
begin
  BuildEngine.Manifest(FPackages);
end;


procedure TCustomInstaller.CheckPackages;
begin
  If (FPackages.Count=0) then
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
      rmInstall : Install;
      rmArchive : Archive;
      rmClean    : Clean;
      rmManifest : Manifest;
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
end;


destructor TBuildEngine.Destroy;
begin
  FreeAndNil(FExternalPackages);
  inherited Destroy;
end;


procedure TBuildEngine.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TBuildEngine.Error(const Fmt: String; const Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


procedure TBuildEngine.ExecuteCommand(const Cmd,Args : String; IgnoreError : Boolean = False);
Var
  E : Integer;
begin
  Log(vlInfo,SInfoExecutingCommand,[Cmd,Args]);
  if ListMode then
    Log(vlCommand,'%s %s',[Cmd,Args])
  else
    begin
      // We should check cmd for spaces, and move all after first space to args.
      E:=ExecuteProcess(cmd,args);
      If (E<>0) and (not IgnoreError) then
        Error(SErrExternalCommandFailed,[Cmd,E]);
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
begin
  Log(vlInfo,SInfoCopyingFile,[Src,Dest]);
  FIn:=TFileStream.Create(Src,fmopenRead);
  Try
    D:=IncludeTrailingPathDelimiter(Dest);
    If DirectoryExists(D) then
      S:=D+ExtractFileName(Src)
    else
      S:=Dest;
    FOut:=TFileStream.Create(S,fmCreate);
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
begin
  if not FileExists(AFileName) then
    Log(vlWarning,SWarnFileDoesNotExist,[AFileName])
  else If Not DeleteFile(AFileName) then
    Error(SErrDeletingFile,[AFileName]);
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
  FLogPrefix:=FLogPrefix+'  ';
end;


procedure TBuildEngine.LogUnIndent;
begin
  Delete(FLogPrefix,1,2);
end;


procedure TBuildEngine.Log(Level: TVerboseLevel; const Msg: String);
begin
  If Assigned(FOnLog) then
    begin
      if Level in [vlInfo,vlDebug] then
        FOnLog(Level,FLogPrefix+Msg)
      else
        FOnLog(Level,Msg);
    end;
end;


procedure TBuildEngine.Log(Level: TVerboseLevel; const Fmt: String;const Args: array of const);
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


procedure TBuildEngine.CmdCopyFiles(List: TStrings; Const DestDir: String);

Var
  Args : String;
  I : Integer;

begin
  CmdCreateDir(DestDir);
  If (Defaults.Copy<>'') then
    begin
      Args:=FileListToString(List,'');
      Args:=Args+' '+DestDir;
      ExecuteCommand(Defaults.Copy,Args);
    end
  else
    For I:=0 to List.Count-1 do
      SysCopyFile(List[i],DestDir);
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


procedure TBuildEngine.CmdArchiveFiles(List: TStrings; Const ArchiveFile: String);
Var
  S,C,O : String;
begin
  If (Defaults.Archive='') then
    SysArchiveFiles(List,ArchiveFile)
  else
    begin
      S:=FileListToString(List,'');
      SplitCommand(Defaults.Archive,C,O);
      If (O='') then
        O:=ArchiveFile+' '+S
      else
        O:=Substitute(O,['ARCHIVE',ArchiveFile,'FILESORDIRS']);
      ExecuteCommand(C,O);
    end;
end;

Function TBuildEngine.FileNewer(const Src,Dest : String) : Boolean;

Var
  DS,DD : Longint;
  D1,D2 : TDateTime;

begin
  DS:=FileAge(Src);
  DD:=FileAge(Dest);
  D1:=FileDateToDateTime(DS);
  D2:=FileDateToDateTime(DD);
  Log(vlDebug,SDbgComparingFileTimes,[Src,DateTimeToStr(D1),Dest,DateTimeToStr(D2)]);
  Result:=D1>=D2;
  If Result then
    Log(vlInfo,SInfoSourceNewerDest,[Src,DateTimeToStr(D1),Dest,DateTimeToStr(D2)]);
end;


procedure TBuildEngine.ExecuteCommands(Commands: TCommands; At: TCommandAt);
Var
  C : TCommand;
  I : Integer;
  Cmd,O : String;
  E : Boolean;
begin
  For I:=0 to Commands.Count-1 do
    begin
      C:=Commands.CommandItems[i];
      if (C.At=At) then
        begin
          E:=True;
          If (C.SourceFile<>'') and (C.DestFile<>'')  then
            E:=FileNewer(C.SourceFile,IncludeTrailingPathDelimiter(Dictionary.GetValue('OUTPUTDIR'))+C.DestFile);
          If E then
            begin
            If Assigned(C.BeforeCommand) then
              C.BeforeCommand(C);
            O:=Substitute(C.Options,['SOURCE',C.SourceFile,'DEST',C.DestFile]);
            Cmd:=C.Command;
            If (ExtractFilePath(Cmd)='') then
              Cmd:=ExeSearch(Cmd,GetEnvironmentvariable('PATH'));
            ExecuteCommand(Cmd,O,C.IgnoreResult);
            If Assigned(C.AfterCommand) then
              C.AfterCommand(C);
            end;
        end;
>>>>>>> graemeg/fixes_2_2
    end;
end;


<<<<<<< HEAD
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
    Add(Format(' <filename>%s</filename>',[QuoteXml(FileName + ZipExt)]));
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
    SetPackageUnitInstallDir(IncludeTrailingPathDelimiter(FPackageUnitInstallDir)+'$('+PackageVariants.Name+')');
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        Flags.DelimitedText:=Values[KeyFlags];

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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
      if NeedLibC then
        Values[KeyNeedLibC]:='Y'
      else
        Values[KeyNeedLibC]:='N';
      if IsFPMakeAddIn then
        Values[KeyAddIn]:='Y'
      else
        Values[KeyAddIn]:='N';
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
<<<<<<< HEAD
    SaveUnitConfigToStringList(L,ACPU,AOS);
=======
    With L do
      begin
        Values[KeyName]:=Name;
        Values[KeyVersion]:=Version;
        // TODO Generate checksum based on PPUs
        Values[KeyChecksum]:=IntToStr(DateTimeToFileDate(Now));
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
        if NeedLibC then
          Values[KeyNeedLibC]:='Y'
        else
          Values[KeyNeedLibC]:='N';
        if IsFPMakeAddIn then
          Values[KeyAddIn]:='Y'
        else
          Values[KeyAddIn]:='N';
      end;
>>>>>>> graemeg/cpstrnew
    L.SaveToStream(F);
  Finally
    L.Free;
    F.Free;
  end;
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
=======
Procedure TBuildEngine.LogSearchPath(const ASearchPathName:string;Path:TConditionalStrings; ACPU:TCPU;AOS:TOS);
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
          S:=S+Dictionary.ReplaceStrings(C.Value)
        end;
    end;
  if S<>'' then
    Log(vlDebug,SDbgSearchPath,[ASearchPathName,S]);
end;


Function TBuildEngine.FindFileInPath(Path:TConditionalStrings; AFileName:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;
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
          FoundPath:=IncludeTrailingPathDelimiter(Dictionary.ReplaceStrings(C.Value));
          if FileExists(FoundPath+AFileName) then
            begin
              result:=true;
              exit;
            end;
        end;
    end;
  FoundPath:='';
end;


Procedure TBuildEngine.ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;DoChangeDir:boolean=true);

  procedure FindMainSource(T:TTarget);
  var
    SD,SF  : String;
  begin
    LogSearchPath('package source',APackage.SourcePath,ACPU,AOS);
    SD:=Dictionary.ReplaceStrings(T.Directory);
    SF:=Dictionary.ReplaceStrings(T.SourceFileName);
    if SD='' then
      FindFileInPath(APackage.SourcePath,SF,SD,ACPU,AOS);
    if SD<>'' then
      SD:=IncludeTrailingPathDelimiter(SD);
    T.FTargetSourceFileName:=SD+SF;
    if FileExists(T.TargetSourceFileName) then
      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSourceFileName])
    else
      begin
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,MakeTargetString(ACPU,AOS)]);
        T.FTargetSourceFileName:='';
      end;
  end;

  procedure FindIncludeSources(T:TTarget);
  var
    SD,SF  : String;
    D : TDependency;
    j : integer;
  begin
    LogSearchPath('target include',T.IncludePath,ACPU,AOS);
    LogSearchPath('package include',APackage.IncludePath,ACPU,AOS);
    for j:=0 to T.Dependencies.Count-1 do
      begin
        D:=T.Dependencies[j];
        if (D.DependencyType=depInclude) then
          begin
            D.TargetFileName:='';
            if (ACPU in D.CPUs) and (AOS in D.OSes) then
              begin
                if ExtractFilePath(D.Value)='' then
                  begin
                    SF:=Dictionary.ReplaceStrings(D.Value);
                    SD:='';
                    // first check the target specific path
                    if not FindFileInPath(T.IncludePath,SF,SD,ACPU,AOS) then
                      FindFileInPath(APackage.IncludePath,SF,SD,ACPU,AOS);
                     if SD<>'' then
                       SD:=IncludeTrailingPathDelimiter(SD);
                     D.TargetFileName:=SD+SF;
                  end
                else
                  D.TargetFileName:=D.Value;
                if FileExists(D.TargetFileName) then
                  Log(vlDebug,SDbgResolvedIncludeFile,[D.Value,D.TargetFileName])
                else
                  begin
                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value,MakeTargetString(ACPU,AOS)]);
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
    LogSearchPath('package example',APackage.ExamplePath,ACPU,AOS);
    SD:=Dictionary.ReplaceStrings(T.Directory);
    SF:=Dictionary.ReplaceStrings(T.SourceFileName);
    if SD='' then
      FindFileInPath(APackage.ExamplePath,SF,SD,ACPU,AOS);
    if SD<>'' then
      SD:=IncludeTrailingPathDelimiter(SD);
    T.FTargetSourceFileName:=SD+SF;
    if FileExists(T.TargetSourceFileName) then
      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSourceFileName])
    else
      begin
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,MakeTargetString(ACPU,AOS)]);
        T.FTargetSourceFileName:='';
      end;
  end;

var
  T : TTarget;
  i : Integer;
begin
  if not((ACPU in APackage.CPUs) and (AOS in APackage.OSes)) then
    exit;
  try
    if DoChangeDir and (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    Dictionary.AddVariable('CPU',CPUToString(ACPU));
    Dictionary.AddVariable('OS',OSToString(AOS));
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
              ttUnit,
              ttImplicitUnit :
                begin
                  FindMainSource(T);
                  if T.Dependencies.Count>0 then
                    FindIncludeSources(T);
                end;
              ttExampleUnit,
              ttExampleProgram :
                begin
                  FindExampleSource(T);
                end;
            end;

            LogUnIndent;
          end;
      end;
  finally
    If DoChangeDir and (APackage.Directory<>'') then
      EnterDir('');
  end;
end;


function TBuildEngine.GetUnitDir(APackage:TPackage):String;
begin
  // Retrieve Full directory name where to find the units.
  // The search order is:
  //  - Package in this fpmake.pp
  //  - LocalUnitDir
  //  - GlobalUnitDir
  if (APackage.UnitDir='') and
     (APackage.State=tsCompiled) then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(FStartDir)+IncludeTrailingPathDelimiter(APackage.Directory)+APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
    end;
  if (APackage.UnitDir='') and
     (Defaults.LocalUnitDir<>'') then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.LocalUnitDir)+APackage.Name;
      if not SysDirectoryExists(APackage.UnitDir) then
        APackage.UnitDir:='';
    end;
  if APackage.UnitDir='' then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.GlobalUnitDir)+APackage.Name;
      if not SysDirectoryExists(APackage.UnitDir) then
        APackage.UnitDir:=DirNotFound;
    end;
  // Special error marker to prevent searches in case of error
  if APackage.UnitDir=DirNotFound then
    Result:=''
  else
    Result:=APackage.UnitDir;
end;


procedure TBuildEngine.AddDependencyIncludePaths(L:TStrings;ATarget: TTarget);
Var
  I : Integer;
  D : TDependency;
  SD : String;
begin
  For I:=0 to ATarget.Dependencies.Count-1 do
    begin
      D:=ATarget.Dependencies[i];
      if (D.DependencyType=depInclude) and
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


Function TBuildEngine.GetCompilerCommand(APackage : TPackage; ATarget : TTarget) : String;
Var
  L,Args : TStringList;
  i : Integer;
begin
  Args:=TStringList.Create;
  Args.Duplicates:=dupIgnore;

  Result := '';

  //compiler configuration
  if Defaults.NoFPCCfg then
    Args.Add('-n');

  // Target OS
  Args.Add('-T'+OSToString(Defaults.OS));

  // Compile mode
  If ATarget.Mode<>cmFPC then
    Args.Add('-M'+ModeToString(ATarget.Mode))
  else If Defaults.Mode<>cmFPC then
    Args.Add('-M'+ModeToString(Defaults.Mode));
  // Output file paths
  If ATarget.TargetType in ProgramTargets then
    Args.Add('-FE'+APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS));
  Args.Add('-FU'+APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  // Object Path
  L:=TStringList.Create;
  L.Sorted:=true;
  L.Duplicates:=dupIgnore;
  AddConditionalStrings(L,APackage.ObjectPath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(L,ATarget.ObjectPath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fo'+L[i]);
  FreeAndNil(L);
  // Unit Dirs
  L:=TStringList.Create;
  L.Sorted:=true;
  L.Duplicates:=dupIgnore;
  AddDependencyUnitPaths(L,APackage);
  AddConditionalStrings(L,APackage.UnitPath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(L,ATarget.UnitPath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fu'+L[i]);
  FreeAndNil(L);
  // Include Path
  L:=TStringList.Create;
  L.Sorted:=true;
  L.Duplicates:=dupIgnore;
  AddDependencyIncludePaths(L,ATarget);
  AddConditionalStrings(L,APackage.IncludePath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(L,ATarget.IncludePath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fi'+L[i]);
  FreeAndNil(L);
  // Custom Options
  If (Defaults.Options<>'') then
    Args.Add(Defaults.Options);
  If (APackage.Options<>'') then
    Args.Add(APackage.Options);
  If (ATarget.Options<>'') then
    Args.Add(ATarget.Options);
  // Add Filename to compile
  Args.Add(ATarget.TargetSourceFileName);
  // Convert to string
  Result:='';
  for i:=0 to Args.Count-1 do
    Result:=Result+' '+maybequoted(Args[i]);
  Delete(result,1,1);
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
      S:=ExeSearch(FCompiler,GetEnvironmentVariable('PATH'));
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
  D:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
  If not SysDirectoryExists(D) then
    begin
      Log(vlInfo,SInfoCreatingOutputDir,[D]);
      CmdCreateDir(D);
    end;

  //also create a bin directory for programtargets
  For i := 0 to Pred(APackage.Targets.Count) do
    begin
      if APackage.Targets.TargetItems[i].TargetType in ProgramTargets then
        begin
          D:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS);
          If not SysDirectoryExists(D) then
            begin
              Log(vlInfo,SInfoCreatingOutputDir,[D]);
              CmdCreateDir(D);
            end;
          //do not continue loop, directory is made anyway
          break;
        end;
>>>>>>> graemeg/fixes_2_2
    end;
end;


<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  else if (Defaults.BuildOS=freebsd) or (Defaults.BuildOS=dragonfly) then
    Result:=Prefix+'share'+PathDelim+'doc'+PathDelim+'fpc-$(CompilerVersion)'+PathDelim+'$(PackageName)'
  else If UnixPaths then
    Result:=Prefix+'share'+PathDelim+'doc'+PathDelim+'fpc-$(CompilerVersion)'+PathDelim+'$(PackageName)'
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  else
    Result:=BaseInstallDir+'docs'+PathDelim+'$(PackageName)';
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
<<<<<<< HEAD
    Result:=BaseInstallDir+'examples'+PathDelim+'$(PackageName)';
=======
    If UnixPaths then
      Result:=Prefix+'share'+PathDelim+'doc'
    else
      Result:=BaseInstallDir+'examples';
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Result:='';
end;


function TCustomDefaults.GetUnitInstallDir: String;
begin
  result := FUnitInstallDir;
end;


function TCustomDefaults.GetUnitConfigFilesInstallDir: String;
begin
  result := FUnitConfigFilesInstallDir;
=======
    Result:=BaseInstallDir+'units'+PathDelim+Target;
>>>>>>> graemeg/cpstrnew
=======
    Result:=BaseInstallDir+'units'+PathDelim+Target;
>>>>>>> graemeg/cpstrnew
=======
    Result:=BaseInstallDir+'units'+PathDelim+Target;
>>>>>>> graemeg/cpstrnew
=======
    Result:=BaseInstallDir+'units'+PathDelim+Target;
>>>>>>> origin/cpstrnew
end;


function TCustomDefaults.GetLocalUnitDir: String;
begin
  Result:=FLocalUnitDir;
end;

function TCustomDefaults.GetFPDocOutputDir: String;
begin
  If (FFPDocOutputDir<>'') then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Result:=IncludeTrailingPathDelimiter(FixPath(FFPDocOutputDir))
  else
    Result:=IncludeTrailingPathDelimiter(FixPath('.'+PathDelim+'docs'));
end;

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

function TCustomDefaults.GetGlobalUnitDir: String;
begin
  If (FGlobalUnitDir<>'') then
    Result:=FGlobalUnitDir
  else
    Result:=UnitInstallDir;
end;


procedure TCustomDefaults.SetLocalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FLocalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FLocalUnitDir:='';
end;


procedure TCustomDefaults.SetGlobalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FGlobalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FGlobalUnitDir:='';
end;

procedure TCustomDefaults.IntSetBaseInstallDir(const AValue: String);
begin
  if AValue<>'' then
    FBaseInstallDir:=IncludeTrailingPathDelimiter(AValue)
  else
    FBaseInstallDir:='';
  GlobalDictionary.AddVariable('baseinstalldir',BaseInstallDir);
  GlobalDictionary.AddVariable('bininstalldir',BinInstallDir);
  BinInstallDir:='';
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
  BaseInstallDir:='';
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
  InitDefaults;
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


procedure TCustomDefaults.LocalInit(Const AFileName : String);
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
      infosl.DelimitedText:=GetCompilerInfo(GetCompiler,'-iVTPTO', False);
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


procedure TCustomDefaults.LoadFromFile(Const AFileName: String);
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


procedure TCustomDefaults.SaveToFile(Const AFileName: String);
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
      Values[KeyLocalUnitDir]:=FLocalUnitDir;
      Values[KeyGlobalUnitDir]:=FGlobalUnitDir;
      Values[KeyPrefix]:=FPrefix;
      Values[KeyBaseInstallDir]:=FBaseInstallDir;
      Values[KeyUnitInstallDir]:=FUnitInstallDir;
      Values[KeyBinInstallDir]:=FBinInstallDir;
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
      Options:=OptionsToStringList(Values[KeyOptions]);
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
      FLocalUnitDir:=Values[KeyLocalUnitDir];
      FGlobalUnitDir:=Values[KeyGlobalUnitDir];
      FPrefix:=Values[KeyPrefix];
      FBaseInstallDir:=Values[KeyBaseInstallDir];
      FUnitInstallDir:=Values[KeyUnitInstallDir];
      FBinInstallDir:=Values[KeyBinInstallDir];
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

  // Where to find the units by default
  if (FGlobalUnitDir='') then
    GlobalUnitDir:=IncludeTrailingPathDelimiter(BD)+'units'+PathDelim+Target;
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
  FBuildEngine.Verbose := (FLogLevels = AllMessages);
  FBuildEngine.OnLog:=@Self.Log;
  NotifyEventCollection.CallEvents(neaAfterCreateBuildengine, Self);
end;


procedure TCustomInstaller.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TCustomInstaller.Error(const Fmt: String; Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


Function TCustomInstaller.AddPackage(const AName: String) : TPackage;
begin
  result:=Packages.AddPackage(AName);
end;

function TCustomInstaller.AddPackageVariant(AName: string; AIsInheritable: boolean): TPackageVariants;
begin
  result := TPackageVariants.Create(TPackageVariant);
  result.Name:=AName;
  result.FIsInheritable:=AIsInheritable;
  FPackageVariants.Add(result);
end;

procedure TCustomInstaller.AnalyzeOptions;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Function CheckOption(Index : Integer;const Short,Long : String; AddToOptionString: boolean = false): Boolean;
=======
  Function CheckOption(Index : Integer;const Short,Long : String; AddToOptionString: boolean = true): Boolean;
>>>>>>> graemeg/cpstrnew
=======
  Function CheckOption(Index : Integer;const Short,Long : String; AddToOptionString: boolean = true): Boolean;
>>>>>>> graemeg/cpstrnew
=======
  Function CheckOption(Index : Integer;const Short,Long : String; AddToOptionString: boolean = true): Boolean;
>>>>>>> graemeg/cpstrnew
=======
  Function CheckOption(Index : Integer;const Short,Long : String; AddToOptionString: boolean = true): Boolean;
>>>>>>> origin/cpstrnew
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O='--'+long) or (copy(O,1,Length(Long)+3)=('--'+long+'='));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if AddToOptionString and Result then
      FFPMakeOptionsString := FFPMakeOptionsString+' '+O;
  end;

  Function CheckBuildOptionSetValue(Index: Integer): boolean;
  var
    O : String;
    BuildModeName: string;
    P: integer;
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
        FPackageVariantSettings.Values[BuildModeName] := O;
        end;
      end;
=======
    if AddToOptionString and Result then FFPMakeOptionsString := FFPMakeOptionsString+' '+O;
>>>>>>> graemeg/cpstrnew
=======
    if AddToOptionString and Result then FFPMakeOptionsString := FFPMakeOptionsString+' '+O;
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
  end;

=======
=======
>>>>>>> graemeg/cpstrnew
    if AddToOptionString and Result then FFPMakeOptionsString := FFPMakeOptionsString+' '+O;
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

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  end;

>>>>>>> origin/cpstrnew

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

Var
  I : Integer;
  DefaultsFileName : string;
  OptString : string;
  CustOptName : string;
begin
  I:=0;
  FListMode:=False;
  FLogLevels:=DefaultMessages;
  While (I<ParamCount) do
    begin
    Inc(I);
    if CheckOption(I,'v','verbose',false) then
      FLogLevels:=AllMessages
    else if CheckOption(I,'d','debug',false) then
      FLogLevels:=AllMessages+[vlDebug]
    else if CheckCommand(I,'m','compile') then
      FRunMode:=rmCompile
    else if CheckCommand(I,'b','build') then
      FRunMode:=rmBuild
    else if CheckCommand(I,'i','install') then
      FRunMode:=rmInstall
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
    else if CheckOption(I,'l','list-commands') then
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
    else if CheckOption(I,'B','bininstalldir') then
      Defaults.BinInstallDir:=OptionArg(I)
    else if CheckOption(I,'U','unitinstalldir') then
      Defaults.UnitInstallDir:=OptionArg(I)
    else if CheckOption(I,'UL','localunitdir') then
      Defaults.LocalUnitDir:=OptionArg(I)
    else if CheckOption(I,'UG','globalunitdir') then
      Defaults.GlobalUnitDir:=OptionArg(I)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    else if CheckOption(I,'o','options', true) then
      begin
        OptString := OptionArg(I, true);
=======
    else if CheckOption(I,'o','options') then
      begin
        OptString := OptionArg(I);
>>>>>>> graemeg/cpstrnew
=======
    else if CheckOption(I,'o','options') then
      begin
        OptString := OptionArg(I);
>>>>>>> graemeg/cpstrnew
=======
    else if CheckOption(I,'o','options') then
      begin
        OptString := OptionArg(I);
>>>>>>> graemeg/cpstrnew
=======
    else if CheckOption(I,'o','options') then
      begin
        OptString := OptionArg(I);
>>>>>>> origin/cpstrnew
        while OptString <> '' do
          Defaults.Options.Add(SplitSpaces(OptString));
      end
    else if CheckOption(I,'r','compiler') then
      Defaults.Compiler:=OptionArg(I)
    else if CheckOption(I,'f','config') then
      DefaultsFileName:=OptionArg(I)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    else if CheckOption(I,'ie','installexamples') then
      Defaults.InstallExamples:=true
    else if CheckOption(I,'sp','skipcrossprograms') then
      Defaults.SkipCrossPrograms:=true
    else if CheckOption(I,'bu','buildunit') then
      Defaults.BuildMode:=bmBuildUnit
    else if CheckOption(I,'io','ignoreinvalidoption', true) then
      Defaults.IgnoreInvalidOptions:=true
    else if CheckOption(I,'d','doc-folder') then
      Defaults.FPDocOutputDir:=OptionArg(I)
    else if CheckOption(I,'fsp','fpunitsrcpath') then
      Defaults.FPUnitSourcePath:=OptionArg(I)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    else if CheckOption(I,'io','ignoreinvalidoption') then
      Defaults.IgnoreInvalidOptions:=true
    else if CheckOption(I,'d','doc-folder') then
      Defaults.FPDocOutputDir:=OptionArg(I)
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
    else if assigned(CustomFpmakeCommandlineOptions) and CheckCustomOption(I,CustOptName) then
      begin
      if not assigned(CustomFpMakeCommandlineValues) then
        CustomFpMakeCommandlineValues := TStringList.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      CustomFpMakeCommandlineValues.Values[CustOptName]:=OptionArg(I, true)
      end
    else if (not CheckBuildOptionSetValue(I)) and (not Defaults.IgnoreInvalidOptions) then
=======
      CustomFpMakeCommandlineValues.Values[CustOptName]:=OptionArg(I)
      end
    else if not Defaults.IgnoreInvalidOptions then
>>>>>>> graemeg/cpstrnew
=======
      CustomFpMakeCommandlineValues.Values[CustOptName]:=OptionArg(I)
      end
    else if not Defaults.IgnoreInvalidOptions then
>>>>>>> graemeg/cpstrnew
=======
      CustomFpMakeCommandlineValues.Values[CustOptName]:=OptionArg(I)
      end
    else if not Defaults.IgnoreInvalidOptions then
>>>>>>> graemeg/cpstrnew
=======
      CustomFpMakeCommandlineValues.Values[CustOptName]:=OptionArg(I)
      end
    else if not Defaults.IgnoreInvalidOptions then
>>>>>>> origin/cpstrnew
      begin
      Usage(SErrInValidArgument,[I,ParamStr(I)]);
      end;
    end;
  If DefaultsFileName<>'' then
    Defaults.LocalInit(DefaultsFileName);
  Defaults.CompilerDefaults;
end;


procedure TCustomInstaller.Usage(const FMT: String; Args: array of const);

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
  LogCmd('uninstall',SHelpUnInstall);
  LogCmd('clean',SHelpClean);
  LogCmd('archive',SHelpArchive);
  LogCmd('manifest',SHelpManifest);
  LogCmd('zipinstall',SHelpZipInstall);
  LogCmd('pkglist',SHelpPkgList);
  Log(vlInfo,SHelpCmdOptions);
  LogOption('h','help',SHelpHelp);
  LogOption('l','list-commands',SHelpList);
  LogOption('n','nofpccfg',SHelpNoFPCCfg);
  LogOption('v','verbose',SHelpVerbose);
{$ifdef HAS_UNIT_PROCESS}
  LogOption('e', 'useenv', sHelpUseEnvironment);
{$endif}
  LogOption('ie','installexamples',SHelpInstExamples);
  LogOption('bu','buildunit',SHelpUseBuildUnit);
  LogOption('sp','skipcrossprograms',SHelpSkipCrossProgs);
  LogOption('io','ignoreinvalidoption',SHelpIgnoreInvOpt);
  LogArgOption('C','cpu',SHelpCPU);
  LogArgOption('O','os',SHelpOS);
  LogArgOption('t','target',SHelpTarget);
  LogArgOption('P','prefix',SHelpPrefix);
  LogArgOption('B','baseinstalldir',SHelpBaseInstalldir);
  LogArgOption('UL','localunitdir',SHelpLocalUnitdir);
  LogArgOption('UG','globalunitdir',SHelpGlobalUnitdir);
  LogArgOption('U','unitinstalldir',SHelpUnitInstallDir);
  LogArgOption('r','compiler',SHelpCompiler);
  LogArgOption('f','config',SHelpConfig);
  LogArgOption('o','options',SHelpOptions);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  LogArgOption('d', 'doc-folder', sHelpFpdocOutputDir);
  LogArgOption('fsp', 'fpunitsrcpath', sHelpFPUnitSrcPath);
  LogArgOption('zp', 'zipprefix', sHelpZipPrefix);
{$ifndef NO_THREADING}
  LogArgOption('T', 'threads', sHelpThreads);
{$endif NO_THREADING}
=======
  LogArgOption('io','ignoreinvalidoption',SHelpIgnoreInvOpt);
  LogArgOption('d', 'doc-folder', sHelpFpdocOutputDir);
>>>>>>> graemeg/cpstrnew
=======
  LogArgOption('io','ignoreinvalidoption',SHelpIgnoreInvOpt);
  LogArgOption('d', 'doc-folder', sHelpFpdocOutputDir);
>>>>>>> graemeg/cpstrnew
=======
  LogArgOption('io','ignoreinvalidoption',SHelpIgnoreInvOpt);
  LogArgOption('d', 'doc-folder', sHelpFpdocOutputDir);
>>>>>>> graemeg/cpstrnew
=======
  LogArgOption('io','ignoreinvalidoption',SHelpIgnoreInvOpt);
  LogArgOption('d', 'doc-folder', sHelpFpdocOutputDir);
>>>>>>> origin/cpstrnew
  if assigned(CustomFpmakeCommandlineOptions) then for i  := 0 to CustomFpmakeCommandlineOptions.Count-1 do
    LogArgOption(' ',CustomFpmakeCommandlineOptions.Names[i],CustomFpmakeCommandlineOptions.ValueFromIndex[i]);
  Log(vlInfo,'');
  If (FMT<>'') then
    halt(1)
  else
    halt(0);
end;


procedure TCustomInstaller.Compile(Force: Boolean);
begin
  FNotifyEventCollection.CallEvents(neaBeforeCompile, Self);
  FBuildEngine.ForceCompile:=Force;
  FBuildEngine.Compile(Packages);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FNotifyEventCollection.CallEvents(neaAfterCompile, Self);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;


procedure TCustomInstaller.Clean(AllTargets: boolean);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeClean, Self);
  BuildEngine.Clean(Packages, AllTargets);
  NotifyEventCollection.CallEvents(neaAfterClean, Self);
=======
  BuildEngine.Clean(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Clean(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Clean(Packages);
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


procedure TCustomInstaller.Install;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeInstall, self);
  BuildEngine.Install(Packages);
  NotifyEventCollection.CallEvents(neaAfterInstall, self);
end;

procedure TCustomInstaller.ZipInstall;
begin
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeInstall, self);
  BuildEngine.ZipInstall(Packages);
  NotifyEventCollection.CallEvents(neaAfterInstall, self);
=======
  BuildEngine.Clean(Packages);
>>>>>>> origin/cpstrnew
end;


procedure TCustomInstaller.UnInstall;
begin
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeUnInstall, self);
  BuildEngine.UnInstall(Packages);
  NotifyEventCollection.CallEvents(neaAfterUnInstall, self);
=======
  BuildEngine.Install(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Install(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Install(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Install(Packages);
>>>>>>> origin/cpstrnew
end;


procedure TCustomInstaller.Archive;
begin
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeArchive, self);
  BuildEngine.Archive(Packages);
  NotifyEventCollection.CallEvents(neaAfterArchive, self);
=======
  // Force generation of manifest.xml, this is required for the repository
  BuildEngine.Manifest(Packages);
  BuildEngine.Archive(Packages);
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


procedure TCustomInstaller.Manifest;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeManifest, self);
  BuildEngine.Manifest(Packages, nil);
  NotifyEventCollection.CallEvents(neaAfterManifest, self);
end;


procedure TCustomInstaller.PkgList;
begin
  NotifyEventCollection.CallEvents(neaBeforePkgList, self);
  BuildEngine.PkgList(Packages);
  NotifyEventCollection.CallEvents(neaAfterPkgList, self);
=======
  BuildEngine.Manifest(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Manifest(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Manifest(Packages);
>>>>>>> graemeg/cpstrnew
=======
  BuildEngine.Manifest(Packages);
>>>>>>> origin/cpstrnew
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
      rmInstall : Install;
      rmZipInstall : ZipInstall;
      rmArchive : Archive;
      rmClean    : Clean(False);
      rmDistClean: Clean(True);
      rmManifest : Manifest;
      rmPkgList : PkgList;
      rmUnInstall : UnInstall;
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
        {$ifdef HAS_UNIT_PROCESS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        E:=ExecuteFPC(Verbose, cmd, args, env, ConsoleOutput);
=======
        E:=ExecuteFPC(Verbose, cmd, args, ConsoleOutput);
>>>>>>> graemeg/cpstrnew
=======
        E:=ExecuteFPC(Verbose, cmd, args, ConsoleOutput);
>>>>>>> graemeg/cpstrnew
=======
        E:=ExecuteFPC(Verbose, cmd, args, ConsoleOutput);
>>>>>>> graemeg/cpstrnew
=======
        E:=ExecuteFPC(Verbose, cmd, args, ConsoleOutput);
>>>>>>> origin/cpstrnew
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
begin
  if not FileExists(AFileName) then
    Log(vldebug,SDbgFileDoesNotExist,[AFileName])
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  else If Not SysUtils.DeleteFile(AFileName) then
    Error(SErrDeletingFile,[AFileName])
  else
    Log(vlInfo,SInfoDeletedFile,[AFileName]);
=======
  else If Not DeleteFile(AFileName) then
    Error(SErrDeletingFile,[AFileName]);
>>>>>>> graemeg/cpstrnew
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
    i: integer;
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
    i := 2;
    result := RemoveDir(ADirectoryName);
{$endif WINDOWS}

    while not result and (i>0) do
      begin
        log(vlWarning, SWarnRetryRemDirectory, [ADirectoryName]);
        sleep(5000);
        dec(i);
        result := RemoveDir(ADirectoryName);
      end;

    if result then
      log(vldebug, SDbgRemovedDirectory, [ADirectoryName]);
  end;

begin
<<<<<<< HEAD
  if not DirectoryExists(ADirectoryName) then
    Log(vldebug,SDbgDirectoryDoesNotExist,[ADirectoryName])
  else If Not IntRemoveTree(ADirectoryName) then
    Error(SErrRemovingDirectory,[ADirectoryName])
  else
    Log(vlInfo,SInfoRemovedDirectory,[ADirectoryName]);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
  if not FileExists(AFileName) then
    Log(vldebug,SDbgFileDoesNotExist,[AFileName])
>>>>>>> origin/cpstrnew
  else If Not DeleteFile(AFileName) then
    Error(SErrDeletingFile,[AFileName]);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

>>>>>>> graemeg/cpstrnew
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
=======
>>>>>>> origin/cpstrnew

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          SysCopyFile(AddPathPrefix(APackage, List.Names[i]),DestFileName)
        end
      else
        SysCopyFile(AddPathPrefix(APackage, List[i]), DestDir);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          SysCopyFile(List.names[i],DestFileName)
        end
      else
        SysCopyFile(List[i],DestDir);
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if WarnIfNotFound then
          Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APackage.Name,MakeTargetString(ACPU,AOS)]);
        APackage.FAllFilesResolved:=false;
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APackage.Name,MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APackage.Name,MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APackage.Name,MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,APackage.Name,MakeTargetString(ACPU,AOS)]);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    if WarnIfNotFound then
                      Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, APackage.Name, MakeTargetString(ACPU,AOS)]);
                    APackage.FAllFilesResolved:=false;
=======
                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if WarnIfNotFound then
          Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName, APackage.Name, MakeTargetString(ACPU,AOS)]);
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> graemeg/cpstrnew
=======
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName, APackage.Name, MakeTargetString(ACPU,AOS)]);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
  APackage.FAllFilesResolved:=false;
  ClearResolvedFileNamesForDependencies(APackage.Dependencies);
  For I:=0 to APackage.Targets.Count-1 do
=======
  // Retrieve Full directory name where to find the units.
  // The search order is:
  //  - Package in this fpmake.pp
  //  - LocalUnitDir
  //  - GlobalUnitDir
  if (APackage.UnitDir='') and
     (APackage.State in [tsCompiled, tsNoCompile]) then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(FStartDir)+IncludeTrailingPathDelimiter(APackage.Directory)+APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
    end;
  if (APackage.UnitDir='') and
     (Defaults.LocalUnitDir<>'') then
>>>>>>> graemeg/cpstrnew
    begin
      T:=APackage.FTargets.TargetItems[I];
      T.FTargetSourceFileName:='';
      ClearResolvedFileNamesForDependencies(T.Dependencies);
    end;
end;


procedure TBuildEngine.ResolvePackagePaths(APackage:TPackage);

  procedure ResolveUnitConfigFilenameForBasePath(ABasePath: string);
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

    IsPackageSourceLocation:=FileExists(IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath)+ASubDir)+FPMakePPFile);
    if IsPackageSourceLocation then
      begin
        PackageBaseDir:=IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath)+ASubDir);
        AnUnitConfigFileName:=PackageBaseDir+APackage.GetUnitConfigOutputFilename(Defaults.CPU,Defaults.OS);
        PackageBaseDir:=IncludeTrailingPathDelimiter(PackageBaseDir+APackage.GetUnitsOutputDir(defaults.CPU, Defaults.OS));
      end
    else
      begin
        PackageBaseDir:=IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath));
        AnUnitConfigFileName:=IncludeTrailingPathDelimiter(GetUnitConfigFilesInstallDir(ABasePath))+APackage.Name+FpmkExt;
        PackageBaseDir:=IncludeTrailingPathDelimiter(IncludeTrailingPathDelimiter(ABasePath)+APackage.GetUnitsOutputDir(Defaults.CPU, Defaults.OS))+APackage.Name;
      end;

    if (PackageBaseDir<>'') and SysDirectoryExists(PackageBaseDir) then
      begin
        APackage.UnitDir:=PackageBaseDir;
        if IsPackageSourceLocation then
          // Set the state to tsNoCompile and not tsCompiled. Because packages
          // in the tsCompiled state trigger a rebuild of packages that depend
          // on it.
          APackage.FTargetState:=tsNoCompile
        else if not (APackage.FTargetState in [tsCompiled, tsNoCompile]) then
          APackage.FTargetState:=tsInstalled;
        AnUnitConfigFilename:=APackage.Dictionary.ReplaceStrings(AnUnitConfigFilename);
        if FileExists(AnUnitConfigFilename) then
          APackage.UnitConfigFileName:=AnUnitConfigFilename;
      end;
  end;

begin
  if APackage.UnitDir='' then
    begin
      // Retrieve Full directory name where to find the units.
      // The search order is:
      //  - Package in this fpmake.pp
      //  - LocalUnitDir
      //  - GlobalUnitDir
      if (APackage.State in [tsCompiled, tsNoCompile, tsInstalled]) then
        ResolveUnitConfigFilenameForBasePath(FStartDir);
      if (APackage.UnitDir='') and
         (Defaults.LocalUnitDir<>'') then
        ResolveUnitConfigFilenameForBasePath(Defaults.LocalUnitDir);
      if (APackage.UnitDir='') and
         (Defaults.GlobalUnitDir<>'') then
        ResolveUnitConfigFilenameForBasePath(Defaults.GlobalUnitDir);

      if (APackage.UnitDir='') then
        APackage.UnitDir:=DirNotFound;
    end;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  s : string;
  ErrS: string;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

  // libc-linker path
  if APackage.NeedLibC then
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
        try
{$endif NO_THREADING}
          FCachedlibcPath:=s;
{$ifndef NO_THREADING}
        finally
          LeaveCriticalsection(FGeneralCriticalSection);
        end;
{$endif NO_THREADING}
        end;
      end;

    Args.Add('-Fl'+FCachedlibcPath);
    end;

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
=======

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
  {$IFDEF MSWINDOWS}
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
  F:=TFileStream.Create(AFileName,fmOpenRead);
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

begin
  Result:=LowerCase(SetToString(PtypeInfo(TypeInfo(TOSes)),Integer(OSes),False));
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


Function OSesToString(const S : String) : TOSes;

begin
  Result:=TOSes(StringToSet(PTypeInfo(TypeInfo(TOSes)),S));
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


Function MakeTargetString(CPU : TCPU;OS: TOS) : String;

begin
  Result:=CPUToString(CPU)+'-'+OSToString(OS);
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


function AddConditionalStrings(Dest : TStrings; Src : TConditionalStrings;ACPU:TCPU;AOS:TOS; Const APrefix : String='') : Integer ;
Var
  I : Integer;
  C : TConditionalString;
  S : String;
begin
  Result:=0;
  Dictionary.AddVariable('CPU',CPUToString(ACPU));
  Dictionary.AddVariable('OS',OSToString(AOS));
  For I:=0 to Src.Count-1 do
    begin
      C:=Src[I];
      if (ACPU in C.CPUs) and (AOS in C.OSes) then
        begin
          If (APrefix<>'') then
            S:=APrefix+C.Value
          else
            S:=C.Value;
          Dest.Add(Dictionary.ReplaceStrings(S));
          Inc(Result);
        end;
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
Var
  P : PChar;
begin
  Result:=APath;
  If (result<>'') then
    begin
      P:=PChar(Result);
      While (P^<>#0) do
        begin
          If P^ in ['/','\'] then
            P^:=PathDelim;
          Inc(P);
        end;
    end;
end;


procedure ChangeDir(const APath : String);
begin
  if Not SetCurrentDir(APath) then
    Raise EInstallerError.CreateFmt(SErrChangeDirFailed,[APath]);
end;


procedure SearchFiles(const AFileName: string; Recursive: boolean; var List: TStrings);

  procedure AddRecursiveFiles(const SearchDir, FileMask: string; Recursive: boolean);
  const
    IgnoreCase = {$ifdef UNIX}False{$else}True{$endif};
  var
    Info : TSearchRec;
  begin
    if FindFirst(SearchDir+'*',faAnyFile and faDirectory,Info)=0 then
    begin
      repeat
          if ((Info.Attr and faDirectory) = faDirectory) and (Info.Name <> '.') and (Info.Name <> '..') and (Recursive) then
            AddRecursiveFiles(SearchDir + Info.Name + PathDelim, FileMask, Recursive);
          if ((Info.Attr and faDirectory) <> faDirectory) and IsWild(Info.Name, FileMask, IgnoreCase) then
            List.Add(SearchDir + Info.Name);
      until FindNext(Info)<>0;
    end;
    FindClose(Info);
  end;

var
  CurrDir,
  BasePath: string;
  i: integer;
begin
  BasePath := ExtractFilePath(ExpandFileName(AFileName));
  AddRecursiveFiles(BasePath, ExtractFileName(AFileName), Recursive);

  CurrDir:=GetCurrentDir;
  for i := 0 to Pred(List.Count) do
    List[i] := ExtractRelativepath(IncludeTrailingPathDelimiter(CurrDir), List[i]);
end;


procedure SplitCommand(const Cmd : String; var Exe, Options : String);

Const
  WhiteSpace = [#9,#10,#13,' '];
  QuoteChars = ['''','"'];

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


{$ifdef HAS_UNIT_PROCESS}
function GetCompilerInfo(const ACompiler,AOptions:string):string;
const
  BufSize = 1024;
var
  S: TProcess;
  Buf: array [0..BufSize - 1] of char;
  Count: longint;
begin
  S:=TProcess.Create(Nil);
  S.Commandline:=ACompiler+' '+AOptions;
  S.Options:=[poUsePipes];
  S.execute;
  Count:=s.output.read(buf,BufSize);
  S.Free;
  SetLength(Result,Count);
  Move(Buf,Result[1],Count);
end;
{$endif HAS_UNIT_PROCESS}


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


Function TTargets.AddUnit(Const AUnitName : String) : TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,AllOSes);
end;


Function TTargets.AddUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.TargetType:=TTUnit;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,AllOSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,AllCPUs,OSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;InstallUnit:boolean=true) : TTarget;
begin
  Result:=AddImplicitUnit(AUnitName,CPUs,AllOSes,InstallUnit);
end;


Function TTargets.AddImplicitUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes;InstallUnit:boolean=true) : TTarget;
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


Function TTargets.AddProgram(Const AProgramName : String) : TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,AllOSes);
end;


Function TTargets.AddProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttProgram;
end;


Function TTargets.AddExampleUnit(Const AUnitName : String) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,AllOSes);
end;


Function TTargets.AddExampleUnit(Const AUnitName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleUnit(AUnitName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddExampleUnit(Const AUnitName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AUnitName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleUnit;
end;


Function TTargets.AddExampleProgram(Const AProgramName : String) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,AllOSes);
end;


Function TTargets.AddExampleProgram(Const AProgramName : String;const OSes:TOSes) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,AllCPUs,OSes);
end;


{$ifdef cpu_only_overloads}
Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs) : TTarget;
begin
  Result:=AddExampleProgram(AProgramName,CPUs,AllOSes);
end;
{$endif cpu_only_overloads}


Function TTargets.AddExampleProgram(Const AProgramName : String;const CPUs:TCPUs;const OSes:TOSes) : TTarget;
begin
  Result:=Add as TTarget;
  Result.Name:=AProgramName;
  Result.CPUs:=CPUs;
  Result.OSes:=OSes;
  Result.TargetType:=ttExampleProgram;
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


function TSources.AddTest(const AFiles : String) : TSource;
begin
  Result:=Add as TSource;
  Result.Name:=AFiles;
  Result.FSourceType:=stTest;
end;


procedure TSources.AddDocFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddDoc(List[i]);
  List.Free;
end;


procedure TSources.AddSrcFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddSrc(List[i]);
  List.Free;
end;


procedure TSources.AddExampleFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
  for i:= 0 to Pred(List.Count) do
    AddExample(List[i]);
  List.Free;
end;


procedure TSources.AddTestFiles(const AFileMask: string; Recursive: boolean);
var
  List : TStrings;
  i: integer;
begin
  List := TStringList.Create;
  SearchFiles(AFileMask, Recursive, List);
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
  FInstallFiles:=TConditionalStrings.Create(TConditionalString);
  FCleanFiles:=TConditionalStrings.Create(TConditionalString);
  FUnitPath:=TConditionalStrings.Create(TConditionalString);
  FObjectPath:=TConditionalStrings.Create(TConditionalString);
  FIncludePath:=TConditionalStrings.Create(TConditionalString);
  FSourcePath:=TConditionalStrings.Create(TConditionalString);
  FExamplePath:=TConditionalStrings.Create(TConditionalString);
  FTestPath:=TConditionalStrings.Create(TConditionalString);
  FCommands:=TCommands.Create(TCommand);
  FCPUs:=AllCPUs;
  FOSes:=AllOSes;
  FInstalledChecksum:=$ffffffff;
  // Implicit dependency on RTL
  FDependencies.Add('rtl');
end;


destructor TPackage.destroy;
begin
  FreeAndNil(FDependencies);
  FreeAndNil(FInstallFiles);
  FreeAndNil(FCleanFiles);
  FreeAndNil(FIncludePath);
  FreeAndNil(FSourcePath);
  FreeAndNil(FExamplePath);
  FreeAndNil(FTestPath);
  FreeAndNil(FObjectPath);
  FreeAndNil(FUnitPath);
  FreeAndNil(FSources);
  FreeAndNil(FTargets);
  FreeAndNil(FVersion);
  inherited destroy;
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
  Result:='units'+PathDelim+MakeTargetString(ACPU,AOS);
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
  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaults.OS));
  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  AddConditionalStrings(List,CleanFiles,ACPU,AOS);
  For I:=0 to FTargets.Count-1 do
    FTargets.TargetItems[I].GetCleanFiles(List, OU, OB, ACPU, AOS);
end;


procedure TPackage.GetInstallFiles(List: TStrings;Types : TTargetTypes;ACPU:TCPU; AOS : TOS);
Var
  OB,OU : String;
  I : Integer;
  T : TTarget;
begin
  OB:=IncludeTrailingPathDelimiter(GetBinOutputDir(Defaults.CPU,Defaults.OS));
  OU:=IncludeTrailingPathDelimiter(GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  AddConditionalStrings(List,InstallFiles,ACPU,AOS);
  For I:=0 to FTargets.Count-1 do
    begin
      T:=FTargets.TargetItems[I];
      if (T.TargetType in Types) and (T.Install) then
        T.GetInstallFiles(List, OU, OB, ACPU, AOS);
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


Function TPackage.GetVersion : string;
begin
  result:=FVersion.AsString;
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
    if not FVersion.Empty then
      Result := Name + '-' + FVersion.AsString
    else
      Result := Name;
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

Var
  S : String;
  i : Integer;
  D : TDependency;
begin
  With Manifest do
    begin
    Add(Format('<package name="%s">',[QuoteXml(Name)]));
    Add(Format(' <version major="%d" minor="%d" micro="%d" build="%d"/>',[FVersion.Major,FVersion.Minor,FVersion.Micro,FVersion.Build]));
    AddOSes(' ',OSes);
    AddCPUs(' ',CPUs);
    Add(Format(' <filename>%s</filename>',[QuoteXml(FileName + ZipExt)]));
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
              Add(Format('   <version major="%d" minor="%d" micro="%d" build="%d"/>',[D.FVersion.Major,D.FVersion.Minor,D.FVersion.Micro,D.FVersion.Build]));
            AddOSes('   ',D.OSes);
            AddCPUs('   ',D.CPUs);
            Add('  </dependency>');
          end;
        Add(' </dependencies>');
      end;
    Add('</package>');
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
      end;
  Finally
    L.Free;
  end;
end;


procedure TPackage.SaveUnitConfigToFile(Const AFileName: String;ACPU:TCPU;AOS:TOS);
Var
  F : TFileStream;
  L : TStringList;
  Deps : String;
  i : integer;
  D : TDependency;
  p : TPackage;
begin
  F:=TFileStream.Create(AFileName,fmCreate);
  L:=TStringList.Create;
  try
    With L do
      begin
        Values[KeyName]:=Name;
        Values[KeyVersion]:=Version;
        // TODO Generate checksum based on PPUs
        Values[KeyChecksum]:=IntToStr(DateTimeToFileDate(Now));
        Values[KeyCPU]:=CPUToString(ACPU);
        Values[KeyOS]:=OSToString(AOS);
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
        if NeedLibC then
          Values[KeyNeedLibC]:='Y'
        else
          Values[KeyNeedLibC]:='N';
      end;
    L.SaveToStream(F);
  Finally
    L.Free;
    F.Free;
  end;
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
begin
  Result:=Add as TPackage;
  Result.Name:=AName;
end;


{****************************************************************************
                             TCustomDefaults
****************************************************************************}

procedure TCustomDefaults.SetCPU(const AValue: TCPU);
begin
  FCPU:=AValue;
  RecalcTarget;
end;


function TCustomDefaults.GetBaseInstallDir: String;
begin
  If (FBaseInstallDir<>'') then
    Result:=FBaseInstallDir
  else
    if UnixPaths then
      Result:=Prefix +'lib' + PathDelim + 'fpc' + PathDelim
    else
      Result:=Prefix;
end;


function TCustomDefaults.GetBinInstallDir: String;
begin
  If (FBinInstallDir<>'') then
    Result:=FBinInstallDir
  else
    If UnixPaths then
      Result:=BaseInstallDir+'bin'
    else
      Result:=BaseInstallDir+'bin';
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
  If (FBinInstallDir<>'') then
    Result:=FBinInstallDir
  else
    If UnixPaths then
      Result:=Prefix+'share'+PathDelim+'doc'
    else
      Result:=BaseInstallDir+'docs';
end;


function TCustomDefaults.GetExamplesInstallDir: String;
begin
  If (FExamplesInstallDir<>'') then
    Result:=FExamplesInstallDir
  else
    If UnixPaths then
      Result:=Prefix+'share'+PathDelim+'docs'+PathDelim+'examples'
    else
      Result:=BaseInstallDir+'examples';
end;


function TCustomDefaults.GetUnitInstallDir: String;
begin
  If (FUnitInstallDir<>'') then
    Result:=FUnitInstallDir
  else
    If UnixPaths then
      Result:=BaseInstallDir+'units'+PathDelim+Target
    else
      Result:=BaseInstallDir+'units'+PathDelim+Target;
end;


function TCustomDefaults.GetLocalUnitDir: String;
begin
  Result:=FLocalUnitDir;
end;


function TCustomDefaults.GetGlobalUnitDir: String;
begin
  If (FGlobalUnitDir<>'') then
    Result:=FGlobalUnitDir
  else
    Result:=UnitInstallDir;
end;


procedure TCustomDefaults.SetLocalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FLocalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FLocalUnitDir:='';
end;


procedure TCustomDefaults.SetGlobalUnitDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FGlobalUnitDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FGlobalUnitDir:='';
end;


procedure TCustomDefaults.SetBaseInstallDir(const AValue: String);
begin
  // Use ExpandFileName to support ~/ expansion
  if AValue<>'' then
    FBaseInstallDir:=IncludeTrailingPathDelimiter(ExpandFileName(AValue))
  else
    FBaseInstallDir:='';
  UnitInstallDir:='';
  BinInstallDir:='';
  ExamplesInstallDir:='';
end;


procedure TCustomDefaults.SetOS(const AValue: TOS);
begin
  FOS:=AValue;
  Recalctarget;
end;


procedure TCustomDefaults.SetPrefix(const AValue: String);
begin
  if FPrefix=AValue then exit;
  FPrefix:=IncludeTrailingPathDelimiter(AValue);
  BaseInstallDir:='';
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
          FCPU:=StringToCPU(System.Copy(Avalue,1,P-1));
        end
      else
        FOS:=StringToOS(AValue);
      FTarget:=AValue;
    end;
end;


procedure TCustomDefaults.RecalcTarget;
begin
  Ftarget:=CPUToString(FCPU)+'-'+OStoString(FOS);
end;


constructor TCustomDefaults.Create;
begin
  InitDefaults;
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
end;


procedure TCustomDefaults.LocalInit(Const AFileName : String);
Var
  FN : String;
begin
  FN:=AFileName;
  If (FN='') then
    begin
    // Environment variable.
    FN:=GetEnvironmentVariable('FPMAKECFG');
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
  if (CPU=cpuNone) or (OS=osNone) or (FCompilerVersion='') then
    begin
{$ifdef HAS_UNIT_PROCESS}
      // Detect compiler version/target from -i option
      infosl:=TStringList.Create;
      infosl.Delimiter:=' ';
      infosl.DelimitedText:=GetCompilerInfo(GetCompiler,'-iVTPTO');
      if infosl.Count<>3 then
        Raise EInstallerError.Create(SErrInvalidFPCInfo);
      if FCompilerVersion='' then
        FCompilerVersion:=infosl[0];
      if CPU=cpuNone then
        CPU:=StringToCPU(infosl[1]);
      if OS=osNone then
        OS:=StringToOS(infosl[2]);
{$else HAS_UNIT_PROCESS}
      // Defaults taken from compiler used to build fpmake
      if CPU=cpuNone then
        CPU:=StringToCPU({$I %FPCTARGETCPU%});
      if OS=osNone then
        OS:=StringToOS({$I %FPCTARGETOS%});
      if FCompilerVersion='' then
        FCompilerVersion:={$I %FPCVERSION%};
{$endif HAS_UNIT_PROCESS}
    end;
end;


procedure TCustomDefaults.LoadFromFile(Const AFileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(AFileName,fmOpenRead);
  Try
    LoadFromStream(F);
  Finally
    F.Free;
  end;
end;


procedure TCustomDefaults.SaveToFile(Const AFileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(AFileName,fmCreate);
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
      Values[KeyOptions]:=FOptions;
      Values[KeyCPU]:=CPUToString(FCPU);
      Values[KeyOS]:=OSToString(FOS);
      Values[KeyMode]:=ModeToString(FMode);
      Values[KeyLocalUnitDir]:=FLocalUnitDir;
      Values[KeyGlobalUnitDir]:=FGlobalUnitDir;
      Values[KeyPrefix]:=FPrefix;
      Values[KeyBaseInstallDir]:=FBaseInstallDir;
      Values[KeyUnitInstallDir]:=FUnitInstallDir;
      Values[KeyBinInstallDir]:=FBinInstallDir;
      Values[KeyDocInstallDir]:=FDocInstallDir;
      Values[KeyExamplesInstallDir]:=FExamplesInstallDir;
      Values[KeyRemove]:=FRemove;
      Values[KeyTarget]:=FTarget;
      if FNoFPCCfg then
        Values[KeyNoFPCCfg]:='Y';
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
      FOptions:=Values[KeyOptions];
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
      FLocalUnitDir:=Values[KeyLocalUnitDir];
      FGlobalUnitDir:=Values[KeyGlobalUnitDir];
      FPrefix:=Values[KeyPrefix];
      FBaseInstallDir:=Values[KeyBaseInstallDir];
      FUnitInstallDir:=Values[KeyUnitInstallDir];
      FBinInstallDir:=Values[KeyBinInstallDir];
      FDocInstallDir:=Values[KeyDocInstallDir];
      FExamplesInstallDir:=Values[KeyExamplesInstallDir];
      FNoFPCCfg:=(Upcase(Values[KeyNoFPCCfg])='Y');
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
  BD:=FixPath(GetEnvironmentVariable('FPCDIR'));
  if BD='' then
    begin
      BD:='/usr/local/lib/fpc/'+FCompilerVersion;
      if not DirectoryExists(BD) and
         DirectoryExists('/usr/lib/fpc/'+FCompilerVersion) then
        BD:='/usr/lib/fpc/'+FCompilerVersion;
    end;
{$else unix}
  BD:=FixPath(GetEnvironmentVariable('FPCDIR'));
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

  // Where to find the units by default
  if (FGlobalUnitDir='') then
    GlobalUnitDir:=IncludeTrailingPathDelimiter(BD)+'units'+PathDelim+Target;
end;


{****************************************************************************
                            TCustomInstaller
****************************************************************************}

constructor TCustomInstaller.Create(AOwner: TComponent);
begin
  Dictionary:=DictionaryClass.Create(Nil);
  AnalyzeOptions;
  CreatePackages;
end;


destructor TCustomInstaller.Destroy;
begin
  FreeAndNil(Defaults);
  FreeAndNil(Dictionary);
  inherited destroy;
end;


procedure TCustomInstaller.Log(Level: TVerboseLevel; const Msg: String);
begin
  If Level in FLogLevels then
    Writeln(StdOut,Msg);
end;


procedure TCustomInstaller.CreatePackages;
begin
  FPAckages:=TPackages.Create(TPackage);
end;


procedure TCustomInstaller.CreateBuildEngine;
begin
  FBuildEngine:=TBuildEngine.Create(Self);
//  FBuildEngine.Defaults:=Defaults;
  FBuildEngine.ListMode:=FListMode;
  FBuildEngine.OnLog:=@Self.Log;
end;


procedure TCustomInstaller.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TCustomInstaller.Error(const Fmt: String; Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


Function TCustomInstaller.AddPackage(const AName: String) : TPackage;
begin
  result:=FPackages.AddPackage(AName);
end;


procedure TCustomInstaller.AnalyzeOptions;

  Function CheckOption(Index : Integer;const Short,Long : String): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O='--'+long) or (copy(O,1,Length(Long)+3)=('--'+long+'='));
  end;

  Function CheckCommand(Index : Integer;const Short,Long : String): Boolean;
  var
    O : String;
  begin
    O:=Paramstr(Index);
    Result:=(O='-'+short) or (O=long);
  end;

  Function OptionArg(Var Index : Integer) : String;
  Var
    P : Integer;
  begin
    if (Length(ParamStr(Index))>1) and (Paramstr(Index)[2]<>'-') then
      begin
      If Index<ParamCount then
        begin
        Inc(Index);
        Result:=Paramstr(Index);
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

Var
  I : Integer;
  DefaultsFileName : string;
begin
  I:=0;
  FListMode:=False;
  FLogLevels:=DefaultMessages;
  While (I<ParamCount) do
    begin
    Inc(I);
    if CheckOption(I,'v','verbose') then
      FLogLevels:=AllMessages
    else if CheckOption(I,'d','debug') then
      FLogLevels:=AllMessages+[vlDebug]
    else if CheckCommand(I,'m','compile') then
      FRunMode:=rmCompile
    else if CheckCommand(I,'b','build') then
      FRunMode:=rmBuild
    else if CheckCommand(I,'i','install') then
      FRunMode:=rmInstall
    else if CheckCommand(I,'c','clean') then
      FRunMode:=rmClean
    else if CheckCommand(I,'a','archive') then
      FRunMode:=rmarchive
    else if CheckCommand(I,'M','manifest') then
      FRunMode:=rmManifest
    else if CheckOption(I,'h','help') then
      Usage('',[])
    else if Checkoption(I,'C','cpu') then
      Defaults.CPU:=StringToCPU(OptionArg(I))
    else if Checkoption(I,'O','os') then
      Defaults.OS:=StringToOS(OptionArg(I))
    else if Checkoption(I,'t','target') then
      Defaults.Target:=OptionArg(I)
    else if CheckOption(I,'l','list-commands') then
      FListMode:=True
    else if Checkoption(I,'P','prefix') then
      Defaults.Prefix:=OptionArg(I)
    else if Checkoption(I,'n','nofpccfg') then
      Defaults.NoFPCCfg:=true
    else if CheckOption(I,'B','baseinstalldir') then
      Defaults.BaseInstallDir:=OptionArg(I)
    else if CheckOption(I,'UL','localunitdir') then
      Defaults.LocalUnitDir:=OptionArg(I)
    else if CheckOption(I,'UG','globalunitdir') then
      Defaults.GlobalUnitDir:=OptionArg(I)
    else if CheckOption(I,'r','compiler') then
      Defaults.Compiler:=OptionArg(I)
    else if CheckOption(I,'f','config') then
      DefaultsFileName:=OptionArg(I)
    else
      begin
      Usage(SErrInValidArgument,[I,ParamStr(I)]);
      end;
    end;
  If DefaultsFileName<>'' then
    Defaults.LocalInit(DefaultsFileName);
  Defaults.CompilerDefaults;
end;


procedure TCustomInstaller.Usage(const FMT: String; Args: array of const);

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
    Log(vlInfo,Format(' -%s --%-20s %s',[C,LC+'='+SValue,MSG]));
  end;

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
  LogCmd('clean',SHelpClean);
  LogCmd('archive',SHelpArchive);
  LogCmd('manifest',SHelpManifest);
  Log(vlInfo,SHelpCmdOptions);
  LogOption('h','help',SHelpHelp);
  LogOption('l','list-commands',SHelpList);
  LogOption('n','nofpccfg',SHelpNoFPCCfg);
  LogOption('v','verbose',SHelpVerbose);
  LogArgOption('C','cpu',SHelpCPU);
  LogArgOption('O','os',SHelpOS);
  LogArgOption('t','target',SHelpTarget);
  LogArgOption('P','prefix',SHelpPrefix);
  LogArgOption('B','baseinstalldir',SHelpBaseInstalldir);
  LogArgOption('UL','localunitdir',SHelpLocalUnitdir);
  LogArgOption('UG','globalunitdir',SHelpGlobalUnitdir);
  LogArgOption('r','compiler',SHelpCompiler);
  LogArgOption('f','config',SHelpConfig);
  Log(vlInfo,'');
  If (FMT<>'') then
    halt(1)
  else
    halt(0);
end;


procedure TCustomInstaller.Compile(Force: Boolean);
begin
  FBuildEngine.ForceCompile:=Force;
  FBuildEngine.Compile(FPackages);
end;


procedure TCustomInstaller.Clean;
begin
  BuildEngine.Clean(FPackages);
end;


procedure TCustomInstaller.Install;
begin
  BuildEngine.Install(FPackages);
end;


procedure TCustomInstaller.Archive;
begin
  // Force generation of manifest.xml, this is required for the repository
  BuildEngine.Manifest(FPackages);
  BuildEngine.Archive(FPackages);
end;


procedure TCustomInstaller.Manifest;
begin
  BuildEngine.Manifest(FPackages);
end;


procedure TCustomInstaller.CheckPackages;
begin
  If (FPackages.Count=0) then
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
      rmInstall : Install;
      rmArchive : Archive;
      rmClean    : Clean;
      rmManifest : Manifest;
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
end;


destructor TBuildEngine.Destroy;
begin
  FreeAndNil(FExternalPackages);
  inherited Destroy;
end;


procedure TBuildEngine.Error(const Msg: String);
begin
  Raise EInstallerError.Create(Msg);
end;


procedure TBuildEngine.Error(const Fmt: String; const Args: array of const);
begin
  Raise EInstallerError.CreateFmt(Fmt,Args);
end;


procedure TBuildEngine.ExecuteCommand(const Cmd,Args : String; IgnoreError : Boolean = False);
Var
  E : Integer;
begin
  Log(vlInfo,SInfoExecutingCommand,[Cmd,Args]);
  if ListMode then
    Log(vlCommand,'%s %s',[Cmd,Args])
  else
    begin
      // We should check cmd for spaces, and move all after first space to args.
      E:=ExecuteProcess(cmd,args);
      If (E<>0) and (not IgnoreError) then
        Error(SErrExternalCommandFailed,[Cmd,E]);
>>>>>>> origin/fixes_2_2
    end;
end;


<<<<<<< HEAD
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
=======
Function TBuildEngine.DependencyOK(ADependency : TDependency) : Boolean;
begin
  Result:=(Defaults.CPU in ADependency.CPUs) and (Defaults.OS in ADependency.OSes);
end;


Function TBuildEngine.TargetOK(ATarget : TTarget) : Boolean;
begin
  Result:=(Defaults.CPU in ATarget.CPUs) and (Defaults.OS in ATarget.OSes);
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
>>>>>>> graemeg/fixes_2_2
end;


procedure TBuildEngine.DoAfterCompile(APackage: TPackage);
begin
  If Assigned(APackage.AfterCompile) then
    APackage.AfterCompile(APackage);
<<<<<<< HEAD
  If Assigned(APackage.AfterCompileProc) then
    APackage.AfterCompileProc(APackage);
=======
>>>>>>> graemeg/fixes_2_2
  ExecuteCommands(APackage.Commands,caAfterCompile);
end;


Function TBuildEngine.NeedsCompile(APackage:TPackage;ATarget: TTarget): Boolean;
Var
  I : Integer;
  D : TDependency;
  T : TTarget;
<<<<<<< HEAD
  OD,OFN,TFN : String;
  CompileReason: String;
=======
  OD,OFN : String;
>>>>>>> graemeg/fixes_2_2
begin
  Result:=False;

  // Forced recompile?
  if FForceCompile then
<<<<<<< HEAD
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
=======
    Result:=true;
>>>>>>> graemeg/fixes_2_2

  // Check output file
  if not result then
    begin
      if ATarget.TargetType in ProgramTargets then
        OD:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS)
      else
        OD:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
      If (OD<>'') then
        OD:=IncludeTrailingPathDelimiter(OD);
<<<<<<< HEAD
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
=======
      OFN:=OD+ATarget.GetOutPutFileName(Defaults.OS);
      Result:=Not FileExists(OFN);
      if Result then
        Log(vlDebug,SDbgOutputNotYetAvailable,[OFN]);
    end;

  // Check main source
  If not Result then
    begin
      if FileExists(ATarget.TargetSourceFileName) then
        Result:=FileNewer(ATarget.TargetSourceFileName,OFN)
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
                      Error(SErrDepUnknownTarget,[D.Value, ATarget.Name, APackage.Name]);
=======
                      Error(SErrDepUnknownTarget,[ATarget.Name,D.Value]);
>>>>>>> graemeg/fixes_2_2
                    // If a dependent package is compiled we always need to recompile
                    Log(vldebug, SDbgDependencyOnUnit, [ATarget.Name,T.Name]);
                    Result:=(T.State=tsCompiled);
                    if Result then
<<<<<<< HEAD
                      begin
                      Log(vldebug, SDbgDependencyUnitRecompiled, [T.Name]);
                      CompileReason:=Format(SDbgDependencyRecompiled,[T.Name]);
                      end;
=======
                      Log(vldebug, SDbgDependencyUnitRecompiled, [T.Name]);
>>>>>>> graemeg/fixes_2_2
                  end;
                depInclude :
                  begin
                    if D.TargetFileName<>'' then
<<<<<<< HEAD
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
                depPackage :
                  begin
                    log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APackage.Name, d.Value]);
                  end;
                depPackage :
                  begin
                    log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APackage.Name, d.Value]);
                  end;
                depPackage :
                  begin
                    log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APackage.Name, d.Value]);
                  end;
                depPackage :
                  begin
                    log(vlWarning,SWarnTargetDependsOnPackage,[ATarget.Name, APackage.Name, d.Value]);
=======
                      Result:=FileNewer(D.TargetFileName,OFN)
>>>>>>> graemeg/fixes_2_2
                  end;
              end;
              if result then
                break;
            end;
        end;
    end;

  if result then
<<<<<<< HEAD
    Log(vlDebug,SDbgMustCompile,[ATarget.Name, CompileReason]);
=======
    Log(vlDebug,SDbgMustCompile,[ATarget.Name]);
>>>>>>> graemeg/fixes_2_2
end;


procedure TBuildEngine.Compile(APackage: TPackage; ATarget: TTarget);
Var
  S : String;
<<<<<<< HEAD
  Env : TStrings;
=======
>>>>>>> graemeg/fixes_2_2
begin
  Log(vlInfo,SInfoCompilingTarget,[ATarget.Name]);
  LogIndent;
  ExecuteCommands(ATarget.Commands,caBeforeCompile);
  If Assigned(ATarget.BeforeCompile) then
    ATarget.BeforeCompile(ATarget);
<<<<<<< HEAD

  if (APackage.BuildMode=bmBuildUnit) and not (ATarget.TargetType in [ttProgram,ttExampleProgram]) then
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
          S:=GetCompilerCommand(APackage,ATarget,Env);
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


function TBuildEngine.CheckExternalPackage(Const APackageName : String; ErrorOnFailure: boolean):TPackage;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      F:=result.UnitConfigFileName;
      if (F<>'') then
        begin
          Log(vlDebug, Format(SDbgLoading, [F]));
          Result.LoadUnitConfigFromFile(F);
          result.SetDefaultPackageVariant;
          result.UnitDir:=result.UnitDir+Result.GetPackageUnitInstallDir(Defaults.CPU, Defaults.OS);
=======
      F:=IncludeTrailingPathDelimiter(S)+UnitConfigFile;
      if FileExists(F) then
        begin
          Log(vlDebug, Format(SDbgLoading, [F]));
          Result.LoadUnitConfigFromFile(F);
        end
      else if FileExists(IncludeTrailingPathDelimiter(S)+FPMakePPFile) then
        begin
=======
=======
>>>>>>> graemeg/cpstrnew
      F:=IncludeTrailingPathDelimiter(S)+UnitConfigFile;
      if FileExists(F) then
        begin
          Log(vlDebug, Format(SDbgLoading, [F]));
          Result.LoadUnitConfigFromFile(F);
        end
      else if FileExists(IncludeTrailingPathDelimiter(S)+FPMakePPFile) then
        begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
      F:=IncludeTrailingPathDelimiter(S)+UnitConfigFile;
      if FileExists(F) then
        begin
          Log(vlDebug, Format(SDbgLoading, [F]));
          Result.LoadUnitConfigFromFile(F);
        end
      else if FileExists(IncludeTrailingPathDelimiter(S)+FPMakePPFile) then
        begin
>>>>>>> origin/cpstrnew
          // The package is not installed, but the source-path is given.
          // It is an external package so it is impossible to compile it, so
          // assume that it has been compiled earlier.
          F := IncludeTrailingPathDelimiter(Result.UnitDir) + Result.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
          // If the unit-directory does not exist, you know for sure that
          // the package is not compiled
          if DirectoryExists(F) then
            begin
              Result.UnitDir := F;
              Result.FTargetState:=tsCompiled;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
        end;
      // Check recursive implicit dependencies
      CompileDependencies(Result);
    end
  else if ErrorOnFailure then
    Error(SErrDependencyNotFound,[APackageName]);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            end
          else
            begin
              D.Target:=CheckExternalPackage(D.Value, true);
              P:=TPackage(D.Target);
            end;
          if (D.RequireChecksum<>$ffffffff) and
             (P.InstalledChecksum<>$ffffffff) and
             (P.InstalledChecksum<>D.RequireChecksum) then
            Log(vlDebug,SDbgPackageChecksumChanged,[P.Name]);
          APackage.InheritPackageVariantsFromDependency(P);
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
                      result := cdNotYetAvailable;
=======
                      MaybeCompile(P);
>>>>>>> origin/cpstrnew
                    tsConsidering :
                      Log(vlWarning,SWarnCircularPackageDependency,[APackage.Name,P.Name]);
                  end;
                end
              else
                Log(vlWarning,SWarnDependOnOtherPlatformPackage,[APackage.Name, D.Value, MakeTargetString(Defaults.CPU, Defaults.OS)]);
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            end
          else
            begin
              D.Target:=CheckExternalPackage(D.Value, ErrorOnFailure);
              P:=TPackage(D.Target);
            end;
          if (D.RequireChecksum<>$ffffffff) and
             (P.InstalledChecksum<>$ffffffff) and
             (P.InstalledChecksum<>D.RequireChecksum) then
            Log(vlDebug,SDbgPackageChecksumChanged,[P.Name]);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
        try
          Compile(APackage,APackage.FBUTarget);
        finally
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  dep: TDependency;
begin
  cmdOpts := '';

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  dep: TDependency;
begin
  cmdOpts := '';

>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        ttUnit:
          begin
            ProcessCompileTarget;
          end;
        ttProgram:
          begin // do nothing, are compiled later
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

        ttUnit,ttProgram:
          begin
            if TargetOK(T) then
              begin
                if T.State=tsNeutral then
                  MaybeCompile(APackage,T);
              end
            else
              begin
                if not(Defaults.CPU in T.CPUs) then
                  Log(vldebug, Format(SDbgSkippingTargetWrongCPU, [T.Name, CPUsToString(T.CPUs)]));
                if not(Defaults.OS in T.OSes) then
                  Log(vldebug, Format(SDbgSkippingTargetWrongOS, [T.Name, OSesToString(T.OSes)]));
              end;
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
        ttFPDoc:
          begin
            for d := 0 to T.Dependencies.Count - 1 do
            begin
              dep := TDependency(T.Dependencies[d]);

              //add unit dependencies
              if dep.DependencyType = depUnit then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                cmdOpts := cmdOpts + ' --input=' + AddPathPrefix(APackage,dep.Value);
            end;

            //check if a documentation target is given
            cmdOpts := cmdOpts + ' --input=' + AddPathPrefix(APackage,T.Directory + T.Name + T.Extension) + ' --descr='+ T.XML;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                cmdOpts := cmdOpts + ' --input=' + dep.Value;
            end;

            //check if a documentation target is given
            cmdOpts := cmdOpts + ' --input=' + T.Directory + T.Name + T.Extension + ' --descr='+ T.XML;
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
          end
        else
          log(vldebug, SDbgTargetIsNotAUnitOrProgram,[T.Name]);
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      end;

    if APackage.BuildMode=bmBuildUnit then
      CompileBuildUnit;

    FreeAndNil(APackage.FBUTarget);

    For I:=0 to APackage.Targets.Count-1 do
      begin
        T:=APackage.Targets.TargetItems[i];
        if T.TargetType=ttProgram then
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
=======
=======
>>>>>>> origin/cpstrnew
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
            ffHtml:      sFPDocFormat := '--format=html --output=' + Defaults.FPDocOutputDir;
            ffHtm:       sFPDocFormat := '--format=htm --output=' + Defaults.FPDocOutputDir;
            ffXHtml:     sFPDocFormat := '--format=xhtml --output=' + Defaults.FPDocOutputDir;
            ffLaTex:     sFPDocFormat := '--format=latex --output=' + Defaults.FPDocOutputDir + APackage.Name + '.tex';
            ffXMLStruct: sFPDocFormat := '--format=xml-struct --output=' + Defaults.FPDocOutputDir;
            ffChm:       sFPDocFormat := '--format=chm --output=' + Defaults.FPDocOutputDir + APackage.Name + '.chm';
          end;

          //execute fpdoc
          Cmd:=ExeSearch('fpdoc',GetEnvironmentvariable('PATH'));
          if Cmd = '' then Cmd := 'fpdoc';
          ExecuteProcess(Cmd, sFPDocFormat + cmdOpts);
        end;
      end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
            ffHtml:      sFPDocFormat := '--format=html --output=' + Defaults.FPDocOutputDir;
            ffHtm:       sFPDocFormat := '--format=htm --output=' + Defaults.FPDocOutputDir;
            ffXHtml:     sFPDocFormat := '--format=xhtml --output=' + Defaults.FPDocOutputDir;
            ffLaTex:     sFPDocFormat := '--format=latex --output=' + Defaults.FPDocOutputDir + APackage.Name + '.tex';
            ffXMLStruct: sFPDocFormat := '--format=xml-struct --output=' + Defaults.FPDocOutputDir;
            ffChm:       sFPDocFormat := '--format=chm --output=' + Defaults.FPDocOutputDir + APackage.Name + '.chm';
          end;

          //execute fpdoc
          Cmd:=ExeSearch('fpdoc',GetEnvironmentvariable('PATH'));
          if Cmd = '' then Cmd := 'fpdoc';
          ExecuteProcess(Cmd, sFPDocFormat + cmdOpts);
        end;
      end;
>>>>>>> graemeg/cpstrnew
=======
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
            ffHtml:      sFPDocFormat := '--format=html --output=' + Defaults.FPDocOutputDir;
            ffHtm:       sFPDocFormat := '--format=htm --output=' + Defaults.FPDocOutputDir;
            ffXHtml:     sFPDocFormat := '--format=xhtml --output=' + Defaults.FPDocOutputDir;
            ffLaTex:     sFPDocFormat := '--format=latex --output=' + Defaults.FPDocOutputDir + APackage.Name + '.tex';
            ffXMLStruct: sFPDocFormat := '--format=xml-struct --output=' + Defaults.FPDocOutputDir;
            ffChm:       sFPDocFormat := '--format=chm --output=' + Defaults.FPDocOutputDir + APackage.Name + '.chm';
          end;

          //execute fpdoc
          Cmd:=ExeSearch('fpdoc',GetEnvironmentvariable('PATH'));
          if Cmd = '' then Cmd := 'fpdoc';
          ExecuteProcess(Cmd, sFPDocFormat + cmdOpts);
        end;
      end;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if ReadyToCompile(APackage) then
    begin
      Compile(APackage);
      APackage.FTargetState:=tsCompiled;
    end;
end;

function TBuildEngine.ReadyToCompile(APackage: TPackage) : Boolean;
begin
  result := False;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

function TBuildEngine.InstallPackageSourceFiles(APAckage: TPackage; tt: TSourceType; const Dest: String): Boolean;
Var
  List : TStringList;
begin
  Result:=False;
  List:=TStringList.Create;
  Try
    APackage.GetInstallSourceFiles(List,[tt]);
    if (List.Count>0) then
      begin
        Result:=True;
        CmdCopyFiles(List,Dest);
      end;
  Finally
    List.Free;
  end;
end;

function TBuildEngine.InstallPackageSourceFiles(APAckage: TPackage; tt: TSourceType; const Dest: String): Boolean;
Var
  List : TStringList;
begin
  Result:=False;
  List:=TStringList.Create;
  Try
    APackage.GetInstallSourceFiles(List,[tt]);
    if (List.Count>0) then
      begin
        Result:=True;
        CmdCopyFiles(List,Dest);
      end;
  Finally
    List.Free;
  end;
end;

function TBuildEngine.InstallPackageSourceFiles(APAckage: TPackage; tt: TSourceType; const Dest: String): Boolean;
Var
  List : TStringList;
begin
  Result:=False;
  List:=TStringList.Create;
  Try
    APackage.GetInstallSourceFiles(List,[tt]);
    if (List.Count>0) then
      begin
        Result:=True;
        CmdCopyFiles(List,Dest);
      end;
  Finally
    List.Free;
  end;
end;

function TBuildEngine.InstallPackageSourceFiles(APAckage: TPackage; tt: TSourceType; const Dest: String): Boolean;
=======
=======
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
begin
  Log(vlInfo,SInfoCopyingFile,[Src,Dest]);
  FIn:=TFileStream.Create(Src,fmopenRead);
  Try
    D:=IncludeTrailingPathDelimiter(Dest);
    If DirectoryExists(D) then
      S:=D+ExtractFileName(Src)
    else
      S:=Dest;
    FOut:=TFileStream.Create(S,fmCreate);
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
begin
  if not FileExists(AFileName) then
    Log(vlWarning,SWarnFileDoesNotExist,[AFileName])
  else If Not DeleteFile(AFileName) then
    Error(SErrDeletingFile,[AFileName]);
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
  FLogPrefix:=FLogPrefix+'  ';
end;


procedure TBuildEngine.LogUnIndent;
begin
  Delete(FLogPrefix,1,2);
end;


procedure TBuildEngine.Log(Level: TVerboseLevel; const Msg: String);
begin
  If Assigned(FOnLog) then
    begin
      if Level in [vlInfo,vlDebug] then
        FOnLog(Level,FLogPrefix+Msg)
      else
        FOnLog(Level,Msg);
    end;
end;


procedure TBuildEngine.Log(Level: TVerboseLevel; const Fmt: String;const Args: array of const);
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


procedure TBuildEngine.CmdCopyFiles(List: TStrings; Const DestDir: String);

Var
  Args : String;
  I : Integer;

begin
  CmdCreateDir(DestDir);
  If (Defaults.Copy<>'') then
    begin
      Args:=FileListToString(List,'');
      Args:=Args+' '+DestDir;
      ExecuteCommand(Defaults.Copy,Args);
    end
  else
    For I:=0 to List.Count-1 do
      SysCopyFile(List[i],DestDir);
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


procedure TBuildEngine.CmdArchiveFiles(List: TStrings; Const ArchiveFile: String);
Var
  S,C,O : String;
begin
  If (Defaults.Archive='') then
    SysArchiveFiles(List,ArchiveFile)
  else
    begin
      S:=FileListToString(List,'');
      SplitCommand(Defaults.Archive,C,O);
      If (O='') then
        O:=ArchiveFile+' '+S
      else
        O:=Substitute(O,['ARCHIVE',ArchiveFile,'FILESORDIRS']);
      ExecuteCommand(C,O);
    end;
end;

Function TBuildEngine.FileNewer(const Src,Dest : String) : Boolean;

Var
  DS,DD : Longint;
  D1,D2 : TDateTime;

begin
  DS:=FileAge(Src);
  DD:=FileAge(Dest);
  D1:=FileDateToDateTime(DS);
  D2:=FileDateToDateTime(DD);
  Log(vlDebug,SDbgComparingFileTimes,[Src,DateTimeToStr(D1),Dest,DateTimeToStr(D2)]);
  Result:=D1>=D2;
  If Result then
    Log(vlInfo,SInfoSourceNewerDest,[Src,DateTimeToStr(D1),Dest,DateTimeToStr(D2)]);
end;


procedure TBuildEngine.ExecuteCommands(Commands: TCommands; At: TCommandAt);
Var
  C : TCommand;
  I : Integer;
  Cmd,O : String;
  E : Boolean;
begin
  For I:=0 to Commands.Count-1 do
    begin
      C:=Commands.CommandItems[i];
      if (C.At=At) then
        begin
          E:=True;
          If (C.SourceFile<>'') and (C.DestFile<>'')  then
            E:=FileNewer(C.SourceFile,IncludeTrailingPathDelimiter(Dictionary.GetValue('OUTPUTDIR'))+C.DestFile);
          If E then
            begin
            If Assigned(C.BeforeCommand) then
              C.BeforeCommand(C);
            O:=Substitute(C.Options,['SOURCE',C.SourceFile,'DEST',C.DestFile]);
            Cmd:=C.Command;
            If (ExtractFilePath(Cmd)='') then
              Cmd:=ExeSearch(Cmd,GetEnvironmentvariable('PATH'));
            ExecuteCommand(Cmd,O,C.IgnoreResult);
            If Assigned(C.AfterCommand) then
              C.AfterCommand(C);
            end;
        end;
    end;
end;


Procedure TBuildEngine.LogSearchPath(const ASearchPathName:string;Path:TConditionalStrings; ACPU:TCPU;AOS:TOS);
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
          S:=S+Dictionary.ReplaceStrings(C.Value)
        end;
    end;
  if S<>'' then
    Log(vlDebug,SDbgSearchPath,[ASearchPathName,S]);
end;


Function TBuildEngine.FindFileInPath(Path:TConditionalStrings; AFileName:String; var FoundPath:String;ACPU:TCPU;AOS:TOS):Boolean;
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
          FoundPath:=IncludeTrailingPathDelimiter(Dictionary.ReplaceStrings(C.Value));
          if FileExists(FoundPath+AFileName) then
            begin
              result:=true;
              exit;
            end;
        end;
    end;
  FoundPath:='';
end;


Procedure TBuildEngine.ResolveFileNames(APackage : TPackage; ACPU:TCPU;AOS:TOS;DoChangeDir:boolean=true);

  procedure FindMainSource(T:TTarget);
  var
    SD,SF  : String;
  begin
    LogSearchPath('package source',APackage.SourcePath,ACPU,AOS);
    SD:=Dictionary.ReplaceStrings(T.Directory);
    SF:=Dictionary.ReplaceStrings(T.SourceFileName);
    if SD='' then
      FindFileInPath(APackage.SourcePath,SF,SD,ACPU,AOS);
    if SD<>'' then
      SD:=IncludeTrailingPathDelimiter(SD);
    T.FTargetSourceFileName:=SD+SF;
    if FileExists(T.TargetSourceFileName) then
      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSourceFileName])
    else
      begin
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,MakeTargetString(ACPU,AOS)]);
        T.FTargetSourceFileName:='';
      end;
  end;

  procedure FindIncludeSources(T:TTarget);
  var
    SD,SF  : String;
    D : TDependency;
    j : integer;
  begin
    LogSearchPath('target include',T.IncludePath,ACPU,AOS);
    LogSearchPath('package include',APackage.IncludePath,ACPU,AOS);
    for j:=0 to T.Dependencies.Count-1 do
      begin
        D:=T.Dependencies[j];
        if (D.DependencyType=depInclude) then
          begin
            D.TargetFileName:='';
            if (ACPU in D.CPUs) and (AOS in D.OSes) then
              begin
                if ExtractFilePath(D.Value)='' then
                  begin
                    SF:=Dictionary.ReplaceStrings(D.Value);
                    SD:='';
                    // first check the target specific path
                    if not FindFileInPath(T.IncludePath,SF,SD,ACPU,AOS) then
                      FindFileInPath(APackage.IncludePath,SF,SD,ACPU,AOS);
                     if SD<>'' then
                       SD:=IncludeTrailingPathDelimiter(SD);
                     D.TargetFileName:=SD+SF;
                  end
                else
                  D.TargetFileName:=D.Value;
                if FileExists(D.TargetFileName) then
                  Log(vlDebug,SDbgResolvedIncludeFile,[D.Value,D.TargetFileName])
                else
                  begin
                    Log(vlWarning,SWarnIncludeFileNotFound,[D.Value,MakeTargetString(ACPU,AOS)]);
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
    LogSearchPath('package example',APackage.ExamplePath,ACPU,AOS);
    SD:=Dictionary.ReplaceStrings(T.Directory);
    SF:=Dictionary.ReplaceStrings(T.SourceFileName);
    if SD='' then
      FindFileInPath(APackage.ExamplePath,SF,SD,ACPU,AOS);
    if SD<>'' then
      SD:=IncludeTrailingPathDelimiter(SD);
    T.FTargetSourceFileName:=SD+SF;
    if FileExists(T.TargetSourceFileName) then
      Log(vlDebug,SDbgResolvedSourceFile,[T.SourceFileName,T.TargetSourceFileName])
    else
      begin
        Log(vlWarning,SWarnSourceFileNotFound,[T.SourceFileName,MakeTargetString(ACPU,AOS)]);
        T.FTargetSourceFileName:='';
      end;
  end;

var
  T : TTarget;
  i : Integer;
begin
  if not((ACPU in APackage.CPUs) and (AOS in APackage.OSes)) then
    exit;
  try
    if DoChangeDir and (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    Dictionary.AddVariable('CPU',CPUToString(ACPU));
    Dictionary.AddVariable('OS',OSToString(AOS));
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
              ttUnit,
              ttImplicitUnit :
                begin
                  FindMainSource(T);
                  if T.Dependencies.Count>0 then
                    FindIncludeSources(T);
                end;
              ttExampleUnit,
              ttExampleProgram :
                begin
                  FindExampleSource(T);
                end;
            end;

            LogUnIndent;
          end;
      end;
  finally
    If DoChangeDir and (APackage.Directory<>'') then
      EnterDir('');
  end;
end;


function TBuildEngine.GetUnitDir(APackage:TPackage):String;
begin
  // Retrieve Full directory name where to find the units.
  // The search order is:
  //  - Package in this fpmake.pp
  //  - LocalUnitDir
  //  - GlobalUnitDir
  if (APackage.UnitDir='') and
     (APackage.State=tsCompiled) then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(FStartDir)+IncludeTrailingPathDelimiter(APackage.Directory)+APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
    end;
  if (APackage.UnitDir='') and
     (Defaults.LocalUnitDir<>'') then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.LocalUnitDir)+APackage.Name;
      if not SysDirectoryExists(APackage.UnitDir) then
        APackage.UnitDir:='';
    end;
  if APackage.UnitDir='' then
    begin
      APackage.UnitDir:=IncludeTrailingPathDelimiter(Defaults.GlobalUnitDir)+APackage.Name;
      if not SysDirectoryExists(APackage.UnitDir) then
        APackage.UnitDir:=DirNotFound;
    end;
  // Special error marker to prevent searches in case of error
  if APackage.UnitDir=DirNotFound then
    Result:=''
  else
    Result:=APackage.UnitDir;
end;


procedure TBuildEngine.AddDependencyIncludePaths(L:TStrings;ATarget: TTarget);
Var
  I : Integer;
  D : TDependency;
  SD : String;
begin
  For I:=0 to ATarget.Dependencies.Count-1 do
    begin
      D:=ATarget.Dependencies[i];
      if (D.DependencyType=depInclude) and
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


Function TBuildEngine.GetCompilerCommand(APackage : TPackage; ATarget : TTarget) : String;
Var
  L,Args : TStringList;
  i : Integer;
begin
  Args:=TStringList.Create;
  Args.Duplicates:=dupIgnore;

  Result := '';

  //compiler configuration
  if Defaults.NoFPCCfg then
    Args.Add('-n');

  // Target OS
  Args.Add('-T'+OSToString(Defaults.OS));

  // Compile mode
  If ATarget.Mode<>cmFPC then
    Args.Add('-M'+ModeToString(ATarget.Mode))
  else If Defaults.Mode<>cmFPC then
    Args.Add('-M'+ModeToString(Defaults.Mode));
  // Output file paths
  If ATarget.TargetType in ProgramTargets then
    Args.Add('-FE'+APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS));
  Args.Add('-FU'+APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
  // Object Path
  L:=TStringList.Create;
  L.Sorted:=true;
  L.Duplicates:=dupIgnore;
  AddConditionalStrings(L,APackage.ObjectPath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(L,ATarget.ObjectPath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fo'+L[i]);
  FreeAndNil(L);
  // Unit Dirs
  L:=TStringList.Create;
  L.Sorted:=true;
  L.Duplicates:=dupIgnore;
  AddDependencyUnitPaths(L,APackage);
  AddConditionalStrings(L,APackage.UnitPath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(L,ATarget.UnitPath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fu'+L[i]);
  FreeAndNil(L);
  // Include Path
  L:=TStringList.Create;
  L.Sorted:=true;
  L.Duplicates:=dupIgnore;
  AddDependencyIncludePaths(L,ATarget);
  AddConditionalStrings(L,APackage.IncludePath,Defaults.CPU,Defaults.OS);
  AddConditionalStrings(L,ATarget.IncludePath,Defaults.CPU,Defaults.OS);
  for i:=0 to L.Count-1 do
    Args.Add('-Fi'+L[i]);
  FreeAndNil(L);
  // Custom Options
  If (Defaults.Options<>'') then
    Args.Add(Defaults.Options);
  If (APackage.Options<>'') then
    Args.Add(APackage.Options);
  If (ATarget.Options<>'') then
    Args.Add(ATarget.Options);
  // Add Filename to compile
  Args.Add(ATarget.TargetSourceFileName);
  // Convert to string
  Result:='';
  for i:=0 to Args.Count-1 do
    Result:=Result+' '+maybequoted(Args[i]);
  Delete(result,1,1);
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
      S:=ExeSearch(FCompiler,GetEnvironmentVariable('PATH'));
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
  D:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
  If not SysDirectoryExists(D) then
    begin
      Log(vlInfo,SInfoCreatingOutputDir,[D]);
      CmdCreateDir(D);
    end;

  //also create a bin directory for programtargets
  For i := 0 to Pred(APackage.Targets.Count) do
    begin
      if APackage.Targets.TargetItems[i].TargetType in ProgramTargets then
        begin
          D:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS);
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


Function TBuildEngine.TargetOK(ATarget : TTarget) : Boolean;
begin
  Result:=(Defaults.CPU in ATarget.CPUs) and (Defaults.OS in ATarget.OSes);
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
end;


procedure TBuildEngine.DoAfterCompile(APackage: TPackage);
begin
  If Assigned(APackage.AfterCompile) then
    APackage.AfterCompile(APackage);
  ExecuteCommands(APackage.Commands,caAfterCompile);
end;


Function TBuildEngine.NeedsCompile(APackage:TPackage;ATarget: TTarget): Boolean;
Var
  I : Integer;
  D : TDependency;
  T : TTarget;
  OD,OFN : String;
begin
  Result:=False;

  // Forced recompile?
  if FForceCompile then
    Result:=true;

  // Check output file
  if not result then
    begin
      if ATarget.TargetType in ProgramTargets then
        OD:=APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS)
      else
        OD:=APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS);
      If (OD<>'') then
        OD:=IncludeTrailingPathDelimiter(OD);
      OFN:=OD+ATarget.GetOutPutFileName(Defaults.OS);
      Result:=Not FileExists(OFN);
      if Result then
        Log(vlDebug,SDbgOutputNotYetAvailable,[OFN]);
    end;

  // Check main source
  If not Result then
    begin
      if FileExists(ATarget.TargetSourceFileName) then
        Result:=FileNewer(ATarget.TargetSourceFileName,OFN)
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
                      Error(SErrDepUnknownTarget,[ATarget.Name,D.Value]);
                    // If a dependent package is compiled we always need to recompile
                    Log(vldebug, SDbgDependencyOnUnit, [ATarget.Name,T.Name]);
                    Result:=(T.State=tsCompiled);
                    if Result then
                      Log(vldebug, SDbgDependencyUnitRecompiled, [T.Name]);
                  end;
                depInclude :
                  begin
                    if D.TargetFileName<>'' then
                      Result:=FileNewer(D.TargetFileName,OFN)
                  end;
              end;
              if result then
                break;
            end;
        end;
    end;

  if result then
    Log(vlDebug,SDbgMustCompile,[ATarget.Name]);
end;


procedure TBuildEngine.Compile(APackage: TPackage; ATarget: TTarget);
Var
  S : String;
begin
  Log(vlInfo,SInfoCompilingTarget,[ATarget.Name]);
  LogIndent;
  ExecuteCommands(ATarget.Commands,caBeforeCompile);
  If Assigned(ATarget.BeforeCompile) then
    ATarget.BeforeCompile(ATarget);
>>>>>>> origin/fixes_2_2
  S:=GetCompilerCommand(APackage,ATarget);
  ExecuteCommand(GetCompiler,S);
  If Assigned(ATarget.AfterCompile) then
    ATarget.AfterCompile(ATarget);
  ExecuteCommands(ATarget.Commands,caAfterCompile);
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
      if (D.DependencyType=depUnit) and
         (Defaults.CPU in D.CPUs) and (Defaults.OS in D.OSes) then
        begin
          T:=TTarget(D.Target);
          if Assigned(T) and (T<>ATarget) then
            begin
              if TargetOK(T) then
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
            Error(SErrDepUnknownTarget,[ATarget.Name,D.Value]);
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
begin
  Result:=False;

  // Forced recompile?
  if FForceCompile then
    Result:=true;

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
                     break;
                 end;
             end;
         end;
    end;

  // Recompile a Target of this package?
  If Not Result then
    begin
      try
        If (APackage.Directory<>'') then
          EnterDir(APackage.Directory);
        for i:=0 to APackage.Targets.Count-1 do
          begin
            Result:=NeedsCompile(APackage,APackage.Targets.TargetItems[i]);
            if Result then
              break;
          end;
      Finally
        If (APackage.Directory<>'') then
          EnterDir('');
      end;
    end;

  if result then
    Log(vlDebug,SDbgMustCompile,[APackage.Name]);
end;


function TBuildEngine.CheckExternalPackage(Const APackageName : String):TPackage;
var
  S : String;
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
      Result.FTargetState:=tsInstalled;
      // Load unit config if it exists
      S:=IncludeTrailingPathDelimiter(S)+UnitConfigFile;
      if FileExists(S) then
        begin
          Log(vlDebug, Format(SDbgLoading, [S]));
          Result.LoadUnitConfigFromFile(S);
        end;
      // Check recursive implicit dependencies
      CompileDependencies(Result);
    end
  else
    Error(SErrDependencyNotFound,[APackageName]);
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
              case P.State of
                tsNeutral :
                  MaybeCompile(P);
                tsConsidering :
                  Log(vlWarning,SWarnCircularPackageDependency,[APackage.Name,P.Name]);
              end;
            end
          else
            begin
              D.Target:=CheckExternalPackage(D.Value);
              P:=TPackage(D.Target);
            end;
          if (D.RequireChecksum<>$ffffffff) and
             (P.InstalledChecksum<>$ffffffff) and
             (P.InstalledChecksum<>D.RequireChecksum) then
            Log(vlDebug,SDbgPackageChecksumChanged,[P.Name]);
        end;
    end;
end;


procedure TBuildEngine.Compile(APackage: TPackage);
Var
  T : TTarget;
  I : Integer;
begin
  Try
    Log(vlInfo,SInfoCompilingPackage,[APackage.Name]);
    If (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    CreateOutputDir(APackage);
    Dictionary.AddVariable('UNITSOUTPUTDIR',APackage.GetUnitsOutputDir(Defaults.CPU,Defaults.OS));
    Dictionary.AddVariable('BINOUTPUTDIR',APackage.GetBinOutputDir(Defaults.CPU,Defaults.OS));
    DoBeforeCompile(APackage);
    For I:=0 to APackage.Targets.Count-1 do
      begin
        T:=APackage.Targets.TargetItems[i];
        if (T.TargetType in [ttUnit,ttProgram]) then
          begin
            if TargetOK(T) then
              begin
                if T.State=tsNeutral then
                  MaybeCompile(APackage,T);
              end
            else
              begin
                if not(Defaults.CPU in T.CPUs) then
                  Log(vldebug, Format(SDbgSkippingTargetWrongCPU, [T.Name, CPUsToString(T.CPUs)]));
                if not(Defaults.OS in T.OSes) then
                  Log(vldebug, Format(SDbgSkippingTargetWrongOS, [T.Name, OSesToString(T.OSes)]));
              end;
          end
        else
          log(vldebug, SDbgTargetIsNotAUnitOrProgram,[T.Name]);
      end;
    DoAfterCompile(APackage);
  Finally
    If (APackage.Directory<>'') then
      EnterDir('');
  end;
end;


procedure TBuildEngine.MaybeCompile(APackage: TPackage);
begin
  if APackage.State<>tsNeutral then
    Error(SErrInvalidState,[APackage.Name]);
  Log(vlDebug,SDbgConsideringPackage,[APackage.Name]);
  LogIndent;
  APackage.FTargetState:=tsConsidering;
  ResolveDependencies(APackage.Dependencies,(APackage.Collection as TPackages));
  CompileDependencies(APackage);
  ResolveFileNames(APackage,Defaults.CPU,Defaults.OS);
  If NeedsCompile(APackage) then
    begin
      Compile(APackage);
      APackage.FTargetState:=tsCompiled;
    end
  else
    APackage.FTargetState:=tsNoCompile;
  LogUnIndent;
end;


Function TBuildEngine.InstallPackageFiles(APAckage : TPackage; tt : TTargetType; Const Dest : String):Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Var
  List : TStringList;
begin
  Result:=False;
  List:=TStringList.Create;
  Try
<<<<<<< HEAD
<<<<<<< HEAD
    APackage.GetInstallSourceFiles(List,[tt]);
=======
    APackage.GetInstallFiles(List,[tt],Defaults.CPU, Defaults.OS);
>>>>>>> graemeg/fixes_2_2
=======
    APackage.GetInstallFiles(List,[tt],Defaults.CPU, Defaults.OS);
>>>>>>> origin/fixes_2_2
    if (List.Count>0) then
      begin
        Result:=True;
        CmdCopyFiles(List,Dest);
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
<<<<<<< HEAD
<<<<<<< HEAD
  If Assigned(APackage.BeforeInstallProc) then
    APackage.BeforeInstallProc(APackage);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TBuildEngine.DoAfterInstall(APackage: TPackage);
begin
  If Assigned(APackage.AfterInstall) then
    APackage.AfterInstall(APackage);
<<<<<<< HEAD
<<<<<<< HEAD
  If Assigned(APackage.AfterInstallProc) then
    APackage.AfterInstallProc(APackage);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ExecuteCommands(APackage.Commands,caAfterInstall);
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TBuildEngine.Install(APackage: TPackage; AnArchiveFiles: boolean);
=======
procedure TBuildEngine.Install(APackage: TPackage);
>>>>>>> graemeg/fixes_2_2
=======
procedure TBuildEngine.Install(APackage: TPackage);
>>>>>>> origin/fixes_2_2
Var
  UC,D : String;
  B : Boolean;
begin
  If (Apackage.State<>tsCompiled) then
    MaybeCompile(APackage);
<<<<<<< HEAD
<<<<<<< HEAD
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
    D:=FixPath(APackage.Dictionary.ReplaceStrings(Defaults.UnitInstallDir), True)+APackage.GetPackageUnitInstallDir(Defaults.CPU,Defaults.OS);
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
<<<<<<< HEAD
    InstallPackageFiles(APAckage,[ttProgram],D, imInstall);
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
=======
    InstallPackageFiles(APAckage,ttProgram,D);
    //InstallPackageFiles(APAckage,ttExampleProgram,D);
    // Documentation
    D:=IncludeTrailingPathDelimiter(Defaults.DocInstallDir)+'fpc-'+APackage.FileName+PathDelim;
    InstallPackageSourceFiles(APackage,stDoc,D);
    // Examples
    D:=IncludeTrailingPathDelimiter(Defaults.ExamplesInstallDir)+'fpc-'+APackage.FileName+PathDelim;
    InstallPackageSourceFiles(APackage,stExample,D);
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
=======
=======
>>>>>>> origin/fixes_2_2
  try
    Log(vlInfo,SInfoInstallingPackage,[APackage.Name]);
    If (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    DoBeforeInstall(APackage);
    // units
    B:=false;
    D:=IncludeTrailingPathDelimiter(Defaults.UnitInstallDir)+APackage.Name;
    if InstallPackageFiles(APAckage,ttUnit,D) then
      B:=true;
    if InstallPackageFiles(APAckage,ttImplicitUnit,D) then
      B:=true;
    // Unit (dependency) configuration if there were units installed
    if B then
      begin
        UC:=IncludeTrailingPathDelimiter(D)+UnitConfigFile;
        Log(vlInfo, Format(SDbgGenerating, [UC]));
        APackage.SaveUnitConfigToFile(UC,Defaults.CPU,Defaults.OS);
      end;
    // Programs
    D:=IncludeTrailingPathDelimiter(Defaults.BinInstallDir);
    InstallPackageFiles(APAckage,ttProgram,D);
    // Done.
    APackage.FTargetState:=tsInstalled;
    DoAfterInstall(APackage);
  Finally
    If (APackage.Directory<>'') then
      EnterDir('');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
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


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TBuildEngine.DoBeforeArchive(APackage: TPackage);
begin
  ExecuteCommands(APackage.Commands,caBeforeArchive);
  If Assigned(APackage.BeforeArchive) then
    APackage.BeforeArchive(APackage);
<<<<<<< HEAD
<<<<<<< HEAD
  If Assigned(APackage.BeforeArchiveProc) then
    APackage.BeforeArchiveProc(APackage);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TBuildEngine.DoAfterArchive(APackage: TPackage);
begin
  If Assigned(APackage.AfterArchive) then
    APackage.AfterArchive(APackage);
<<<<<<< HEAD
<<<<<<< HEAD
  If Assigned(APackage.AfterArchiveProc) then
    APackage.AfterArchiveProc(APackage);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ExecuteCommands(APackage.Commands,caAfterArchive);
end;


procedure TBuildEngine.Archive(APackage: TPackage);
Var
  L : TStringList;
  A : String;
  i: integer;
  ICPU : TCPU;
  IOS  : TOS;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
begin
  A:=FStartDir+ APackage.FileName + ZipExt;
  Log(vlInfo,SInfoArchivingPackage,[APackage.Name,A]);
  try
    If (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    DoBeforeArchive(Apackage);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
              // Make sure that the package is resolved for each target
              ClearResolvedFileNames(APackage);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              ResolveFileNames(APackage,ICPU,IOS,false);
              APackage.GetArchiveFiles(L, ICPU, IOS);
            end;
      //from sources
<<<<<<< HEAD
<<<<<<< HEAD
      APackage.GetArchiveSourceFiles(L);
=======
      for i := 0 to APackage.Sources.Count-1 do
        L.Add(APackage.Sources[i].Name);
>>>>>>> graemeg/fixes_2_2
=======
      for i := 0 to APackage.Sources.Count-1 do
        L.Add(APackage.Sources[i].Name);
>>>>>>> origin/fixes_2_2

      //show all files
      for i := 0 to L.Count-1 do
        Log(vlDebug, Format(SDbgArchivingFile, [L[i]]));

{$ifdef HAS_UNIT_ZIPPER}
      if not Assigned(ArchiveFilesProc) then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
          FZipFile := TZipper.Create;
          FZipFile.ZipFiles(A, L);
>>>>>>> graemeg/fixes_2_2
=======
          FZipFile := TZipper.Create;
          FZipFile.ZipFiles(A, L);
>>>>>>> origin/fixes_2_2
        end
      else
{$endif HAS_UNIT_ZIPPER}
        CmdArchiveFiles(L,A);
    Finally
      L.Free;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

{$ifdef HAS_UNIT_ZIPPER}
      if not Assigned(ArchiveFilesProc) then
        FreeAndNil(FZipFile);
{$endif HAS_UNIT_ZIPPER}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
    DoAfterArchive(Apackage);
  Finally
    If (APackage.Directory<>'') then
<<<<<<< HEAD
<<<<<<< HEAD
      GPathPrefix := '';
=======
      EnterDir('');
>>>>>>> graemeg/fixes_2_2
=======
      EnterDir('');
>>>>>>> origin/fixes_2_2
  end;
end;


procedure TBuildEngine.DoBeforeClean(APackage: TPackage);
begin
<<<<<<< HEAD
  ExecuteCommands(APackage.Commands,caBeforeClean);
  If Assigned(APackage.BeforeClean) then
    APackage.BeforeClean(APackage);
<<<<<<< HEAD
  If Assigned(APackage.BeforeCleanProc) then
    APackage.BeforeCleanProc(APackage);
=======
>>>>>>> graemeg/fixes_2_2
end;


procedure TBuildEngine.DoAfterClean(APackage: TPackage);
begin
  If Assigned(APackage.AfterClean) then
    APackage.AfterClean(APackage);
<<<<<<< HEAD
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
  Log(vlInfo,SInfoCleaningPackage,[APackage.Name]);
  try
    If (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    // Check for inherited options (packagevariants) from other packages
    ResolveDependencies(APackage.Dependencies, (APackage.Collection as TPackages));
    CheckDependencies(APackage, False);
    APackage.SetDefaultPackageVariant;
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
                if OSCPUSupported[AOS,ACPU] then
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
    If (APackage.Directory<>'') then
      EnterDir('');
  end;
end;

procedure TBuildEngine.Clean(APackage: TPackage; ACPU: TCPU; AOS: TOS);
Var
  List : TStringList;
  DirectoryList : TStringList;
begin
  List:=TStringList.Create;
  try
    List.Add(APackage.GetUnitConfigOutputFilename(ACPU,AOS));
    APackage.GetCleanFiles(List,ACPU,AOS);
    if (List.Count>0) then
      begin
      CmdDeleteFiles(List);
      DirectoryList := TStringList.Create;
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
        if DirectoryExists(ExtractFileDir(APackage.GetBinOutputDir(ACPU,AOS))) then
          DirectoryList.Add(ExtractFileDir(APackage.GetBinOutputDir(ACPU,AOS)));
        if DirectoryExists(ExtractFileDir(APackage.GetUnitsOutputDir(ACPU,AOS))) then
          DirectoryList.Add(ExtractFileDir(APackage.GetUnitsOutputDir(ACPU,AOS)));
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
            log(vlWarning,SWarnSkipPackageTargetProgress,[(FProgressCount)/FProgressMax * 100, APackage.Name, Defaults.Target]);
            APackage.FTargetState:=tsNoCompile;
          end;
      end;
  end;

Var
  I : integer;
{$ifndef NO_THREADING}
  Thr : Integer;
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
        if assigned(AThread.APackage) then
          begin
            // The thread has completed compiling the package
            if AThread.CompilationOK then
              AThread.APackage.FTargetState:=tsCompiled
            else // A problem occured, stop the compilation
              begin
              ErrorState:=true;
              ErrorMessage:=AThread.ErrorMessage;
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
            log(vlWarning,SWarnSkipPackageTargetProgress,[(FProgressCount)/FProgressMax * 100, P.Name, Defaults.Target]);
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
      // This event is set by the worker-threads to notify the main/this thread
      // that a package finished it's task.
      NotifyThreadWaiting := RTLEventCreate;
      SetLength(Threads,Defaults.ThreadsAmount);
      // Create all worker-threads
      for Thr:=0 to Defaults.ThreadsAmount-1 do
        Threads[Thr] := TCompileWorkerThread.Create(self,NotifyThreadWaiting);
      try
        // When a thread notifies this thread that it is ready, loop on all
        // threads to check their state and if possible assign a new package
        // to them to compile.
        while not Finished do
          begin
            RTLeventWaitFor(NotifyThreadWaiting);
            for Thr:=0 to Defaults.ThreadsAmount-1 do if not Finished then
              ProcessThreadResult(Threads[Thr]);
          end;
        // Compilation finished or aborted. Wait for all threads to end.
        for thr:=0 to Defaults.ThreadsAmount-1 do
          begin
            Threads[Thr].Terminate;
            RTLeventSetEvent(Threads[Thr].NotifyStartTask);
            Threads[Thr].WaitFor;
          end;
      finally
        RTLeventdestroy(NotifyThreadWaiting);
        for thr:=0 to Defaults.ThreadsAmount-1 do
          Threads[Thr].Free;
      end;
    if ErrorState then
      raise Exception.Create(ErrorMessage);
{$endif NO_THREADING}
    end;
  NotifyEventCollection.CallEvents(neaAfterCompile, Self);
end;


procedure TBuildEngine.Install(Packages: TPackages);
=======
=======
  ExecuteCommands(APackage.Commands,caBeforeClean);
  If Assigned(APackage.BeforeClean) then
    APackage.BeforeClean(APackage);
end;


procedure TBuildEngine.DoAfterClean(APackage: TPackage);
begin
  If Assigned(APackage.AfterClean) then
    APackage.AfterClean(APackage);
>>>>>>> origin/fixes_2_2
  ExecuteCommands(APackage.Commands,caAfterClean);
end;


procedure TBuildEngine.Clean(APackage: TPackage);
Var
  List : TStringList;
begin
  Log(vlInfo,SInfoCleaningPackage,[APackage.Name]);
  try
    If (APackage.Directory<>'') then
      EnterDir(APackage.Directory);
    DoBeforeClean(Apackage);
    List:=TStringList.Create;
    try
      APackage.GetCleanFiles(List,Defaults.CPU,Defaults.OS);
      if (List.Count>0) then
        CmdDeleteFiles(List);
    Finally
      List.Free;
    end;
    DoAfterClean(Apackage);
  Finally
    If (APackage.Directory<>'') then
      EnterDir('');
  end;
end;


Procedure TBuildEngine.Manifest(APackage : TPackage);
Var
  L : TStrings;
  PD,
  MF : String;
begin
  L:=TStringList.Create;
  Try
    Log(vlInfo, Format(SInfoManifestPackage,[APackage.Name]));
    PD:=APackage.Directory;
    if PD<>'' then
      PD:=IncludeTrailingPathDelimiter(PD);
    MF:=PD+ManifestFile;
    Log(vlDebug, Format(SDbgGenerating, [MF]));
    L.Add('<?xml version="1.0"?>');
    L.Add('<packages>');
    APackage.GetManifest(L);
    L.Add('</packages>');
    L.SaveToFile(MF);
  Finally
    L.Free;
  end;
end;


procedure TBuildEngine.Compile(Packages: TPackages);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Var
  I : Integer;
  P : TPackage;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeInstall, Self);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      log(vlWarning,SWarnStartBuildingPackage,[P.Name, Defaults.Target]);
      If PackageOK(P) then
<<<<<<< HEAD
        begin
          Install(P, False);
          log(vlWarning, SWarnInstallationPackagecomplete, [P.Name, Defaults.Target]);
        end
      else
        log(vlWarning,SWarnSkipPackageTarget,[P.Name, Defaults.Target]);
=======
        MaybeCompile(P);

      //show compile progress
      log(vlWarning,SWarnBuildingPackagecomplete,[(I + 1)/Packages.Count * 100, P.Name]);
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
          log(vlWarning,SWarnSkipPackageTarget,[P.Name, Defaults.Target]);
      end;
  finally
    FinishArchive(P);
  end;

  NotifyEventCollection.CallEvents(neaAfterInstall, Self);
end;


procedure TBuildEngine.UnInstall(Packages: TPackages);
=======
=======
>>>>>>> origin/fixes_2_2
  If Assigned(BeforeCompile) then
    BeforeCompile(Self);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      If PackageOK(P) then
        MaybeCompile(P);
    end;
  If Assigned(AfterCompile) then
    AfterCompile(Self);
end;


procedure TBuildEngine.Install(Packages: TPackages);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Var
  I : Integer;
  P : TPackage;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeUnInstall, Self);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
<<<<<<< HEAD
      UnInstall(P);
=======
      If PackageOK(P) then
        Install(P);
      log(vlWarning, SWarnInstallationPackagecomplete, [P.Name, Defaults.Target]);
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
  NotifyEventCollection.CallEvents(neaAfterUnInstall, Self);
=======
=======
>>>>>>> origin/fixes_2_2
  If Assigned(BeforeInstall) then
    BeforeInstall(Self);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      If PackageOK(P) then
        Install(P);
    end;
  If Assigned(AfterInstall) then
    AfterInstall(Self);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure TBuildEngine.Archive(Packages: TPackages);
Var
  I : Integer;
  P : TPackage;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeArchive, Self);
=======
  If Assigned(BeforeArchive) then
    BeforeArchive(Self);
>>>>>>> graemeg/fixes_2_2
=======
  If Assigned(BeforeArchive) then
    BeforeArchive(Self);
>>>>>>> origin/fixes_2_2
  Log(vlDebug, SDbgBuildEngineArchiving);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
<<<<<<< HEAD
<<<<<<< HEAD
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

procedure TBuildEngine.Clean(Packages: TPackages; AllTargets: boolean);
=======
=======
>>>>>>> origin/fixes_2_2
      Archive(P);
    end;
  If Assigned(AfterArchive) then
    AfterArchive(Self);
end;


procedure TBuildEngine.Manifest(Packages: TPackages);
Var
  I : Integer;
  P : TPackage;
begin
  If Assigned(BeforeManifest) then
    BeforeManifest(Self);
  Log(vlDebug, SDbgBuildEngineGenerateManifests);
  For I:=0 to Packages.Count-1 do
    begin
      P:=Packages.PackageItems[i];
      Manifest(P);
    end;
  If Assigned(AfterManifest) then
    AfterManifest(Self);
end;


procedure TBuildEngine.Clean(Packages: TPackages);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Var
  I : Integer;
  P : TPackage;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  NotifyEventCollection.CallEvents(neaBeforeClean, Self);
  Log(vldebug, SDbgBuildEngineCleaning);
  For I:=0 to Packages.Count-1 do
    begin
<<<<<<< HEAD
      P:=Packages.PackageItems[i];
      If AllTargets or PackageOK(P) then
        Clean(P, AllTargets);
      log(vlWarning, SWarnCleanPackagecomplete, [P.Name]);
=======
    P:=Packages.PackageItems[i];
    If PackageOK(P) then
      Clean(P);
    log(vlWarning, SWarnCleanPackagecomplete, [P.Name]);
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
  NotifyEventCollection.CallEvents(neaAfterClean, Self);
end;

=======
=======
>>>>>>> origin/fixes_2_2
  If Assigned(BeforeClean) then
    BeforeClean(Self);
  Log(vldebug, SDbgBuildEngineCleaning);
  For I:=0 to Packages.Count-1 do
    begin
    P:=Packages.PackageItems[i];
    If PackageOK(P) then
      Clean(P);
    end;
  If Assigned(AfterClean) then
    AfterClean(Self);
end;


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{****************************************************************************
                               TFPVersion
****************************************************************************}

function TFPVersion.GetAsString: String;
begin
  if Empty then
    Result:='<none>'
  else
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    Result:=Format('%d.%d.%d-%d',[Major,Minor,Micro,Build]);
>>>>>>> graemeg/fixes_2_2
=======
    Result:=Format('%d.%d.%d-%d',[Major,Minor,Micro,Build]);
>>>>>>> origin/fixes_2_2
end;

function TFPVersion.GetEmpty: Boolean;
begin
  Result:=(Major=0) and (Minor=0) and (Micro=0) and (Build=0);
end;

procedure TFPVersion.SetAsString(const AValue: String);

  Function NextDigit(sep : Char; var V : string) : integer;
<<<<<<< HEAD
<<<<<<< HEAD
  Var
    P : Integer;
=======
=======
>>>>>>> origin/fixes_2_2

  Var
    P : Integer;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  begin
    P:=Pos(Sep,V);
    If (P=0) then
      P:=Length(V)+1;
    Result:=StrToIntDef(Copy(V,1,P-1),-1);
    If Result<>-1 then
<<<<<<< HEAD
<<<<<<< HEAD
      Delete(V,1,P);
=======
      Delete(V,1,P)
    else
      Result:=0;
>>>>>>> graemeg/fixes_2_2
=======
      Delete(V,1,P)
    else
      Result:=0;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  Micro:=-1;
  Major:=-1;
  Minor:=-1;
  Build:=-1;
end;

procedure TFPVersion.Assign(Source: TPersistent);
Var
  V : TFPVersion;
=======
=======
>>>>>>> origin/fixes_2_2
  Micro:=0;
  Major:=0;
  Minor:=0;
  Build:=0;
end;

procedure TFPVersion.Assign(Source: TPersistent);

Var
  V : TFPVersion;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  FCommands:=TCommands.Create(TCommand);
=======
  FCommands:=TCOmmands.Create(TCommand);
>>>>>>> graemeg/fixes_2_2
=======
  FCommands:=TCOmmands.Create(TCommand);
>>>>>>> origin/fixes_2_2
end;


destructor TTarget.Destroy;
begin
  FreeAndNil(FUnitPath);
  FreeAndNil(FObjectPath);
  FreeAndNil(FIncludePath);
  FreeAndNil(FDependencies);
  FreeAndNil(FCommands);
<<<<<<< HEAD
<<<<<<< HEAD
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
  else if AOS in [macos] then
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
=======
  inherited Destroy;
end;

>>>>>>> graemeg/fixes_2_2
=======
  inherited Destroy;
end;

>>>>>>> origin/fixes_2_2

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


function TTarget.GetRSTFileName: String;
begin
  Result:=Name+RSText;
end;


<<<<<<< HEAD
<<<<<<< HEAD
function TTarget.GetRSJFileName: String;
begin
  Result:=Name+RSJext;
end;


function TTarget.GetProgramFileName(AOS : TOS): String;
begin
  result := AddProgramExtension(Name, AOS);
end;


function TTarget.GetProgramDebugFileName(AOS: TOS): String;
begin
  result := Name + DbgExt;
=======
=======
>>>>>>> origin/fixes_2_2
function TTarget.GetProgramFileName(AOS : TOS): String;
begin
  if AOS in [Go32v2,Win32,Win64,OS2] then
    Result:=Name+ExeExt
  else
    Result:=Name;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


function TTarget.GetOutputFileName(AOs: TOS): String;
begin
  if TargetType in UnitTargets then
    Result:=GetUnitFileName
  else
    Result:=GetProgramFileName(AOs);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TTarget.HaveOptions: Boolean;
begin
  Result:=(FOptions<>Nil);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

procedure TTarget.SetName(const AValue: String);
Var
  D,N,E : String;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  N:=FixPath(AValue, False);
=======
  N:=FixPath(AValue);
>>>>>>> graemeg/fixes_2_2
=======
  N:=FixPath(AValue);
>>>>>>> origin/fixes_2_2
  D:=ExtractFilePath(N);
  E:=ExtractFileExt(N);
  N:=ExtractFileName(N);
  inherited SetName(Copy(N,1,Length(N)-Length(E)));
  FExtension:=E;
  FDirectory:=D;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TTarget.SetXML(const AValue: string);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FXML:=FixPath(AValue, False);
=======
  FXML:=FixPath(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FXML:=FixPath(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FXML:=FixPath(AValue);
>>>>>>> graemeg/cpstrnew
=======
  FXML:=FixPath(AValue);
>>>>>>> origin/cpstrnew
end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

procedure TTarget.GetCleanFiles(List: TStrings; const APrefixU, APrefixB : String; ACPU: TCPU; AOS : TOS);
begin
  If not(ACPU in CPUs) or not(AOS in OSes) then
    exit;
  List.Add(APrefixU + ObjectFileName);
  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit, ttCleanOnlyUnit]) then
<<<<<<< HEAD
<<<<<<< HEAD
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
    end;
  If ResourceStrings then
    begin
      // choose between 2 possible resource files
      if FileExists(APrefixU + RSJFileName) then
        List.Add(APrefixU + RSJFileName)
      else
        List.Add(APrefixU + RSTFileName);
    end;
=======
=======
>>>>>>> origin/fixes_2_2
    List.Add(APrefixU + UnitFileName)
  else If (TargetType in [ttProgram,ttExampleProgram]) then
    List.Add(APrefixB + GetProgramFileName(AOS));
  If ResourceStrings then
    List.Add(APrefixU + RSTFileName);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // Maybe add later ?  AddConditionalStrings(List,CleanFiles);
end;


procedure TTarget.GetInstallFiles(List: TStrings; const APrefixU, APrefixB: String; ACPU: TCPU; AOS : TOS);
<<<<<<< HEAD
<<<<<<< HEAD
var
  UnitsDir : string;
begin
  UnitsDir := Installer.BuildEngine.AddPathPrefix(nil, APrefixU);
  If Not (TargetType in [ttProgram,ttExampleProgram]) and FileExists(UnitsDir + ObjectFileName) then
    // The compiler does not create an objectfile for all programs.
    List.Add(APrefixU + ObjectFileName);
  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit]) then
    begin
      List.Add(APrefixU + UnitFileName);
      if (AOS in AllSmartLinkLibraryOSes) and FileExists(UnitsDir + GetUnitLibFileName(AOS)) then
        List.Add(APrefixU + GetUnitLibFileName(AOS));
      if (AOS in AllImportLibraryOSes) and FileExists(UnitsDir + GetImportLibFilename(AOS)) then
        List.Add(APrefixU + GetImportLibFilename(AOS));
    end
  else If (TargetType in [ttProgram,ttExampleProgram]) then
    List.Add(APrefixB + GetProgramFileName(AOS));
  If ResourceStrings then
    begin
      // choose between 2 possible resource files
      if FileExists(UnitsDir + RSJFileName) then
        List.Add(APrefixU + RSJFileName)
      else
        List.Add(APrefixU + RSTFileName);
    end;
=======
=======
>>>>>>> origin/fixes_2_2
begin
  If not(ACPU in CPUs) or not(AOS in OSes) then
    exit;
  If Not (TargetType in [ttProgram,ttExampleProgram]) then
    List.Add(APrefixU + ObjectFileName);
  If (TargetType in [ttUnit,ttImplicitUnit,ttExampleUnit]) then
    List.Add(APrefixU + UnitFileName)
  else If (TargetType in [ttProgram,ttExampleProgram]) then
    List.Add(APrefixB + GetProgramFileName(AOS));
  If ResourceStrings then
    List.Add(APrefixU + RSTFileName);
  // Maybe add later ?  AddConditionalStrings(List,InstallFiles);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  // FPDoc files
  if XML <> '' then
  begin
    List.Add(Directory + Name + Extension);
    List.Add(XML);
  end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{****************************************************************************
                                 TSource
****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
function TSource.GetInstallSourcePath: string;
begin
  if FInstallSourcePath<>'' then
    result := FInstallSourcePath
  else if SourceType=stExample then
    result := 'examples'
  else
    result := '';
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
constructor TSource.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
end;


destructor TSource.Destroy;
begin
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TSource.GetInstallFiles(List: TStrings);
begin
  if InstallSourcePath<>'' then
    list.Values[name] := (IncludeTrailingPathDelimiter(InstallSourcePath)+ExtractFileName(Name))
  else
    list.add(Name);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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


Function TCommands.AddCommand(const Cmd: String) : TCommand;
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


Function TCommands.AddCommand(At: TCommandAt; const Cmd: String) : TCommand;
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
<<<<<<< HEAD
<<<<<<< HEAD
  If (Options<>'') then
    Result.ParseOptions(Options);
=======
  Result.Options:=Options;
>>>>>>> graemeg/fixes_2_2
=======
  Result.Options:=Options;
>>>>>>> origin/fixes_2_2
  Result.At:=At;
  Result.SourceFile:=Source;
  Result.DestFile:=Dest;
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
<<<<<<< HEAD
<<<<<<< HEAD
  N:=FixPath(Value, False);
=======
  N:=FixPath(Value);
>>>>>>> graemeg/fixes_2_2
=======
  N:=FixPath(Value);
>>>>>>> origin/fixes_2_2
  if ExtractFileExt(N)='' then
    ChangeFileExt(N,IncExt);
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

<<<<<<< HEAD
<<<<<<< HEAD
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

procedure TNotifyEventCollection.AppendProcEvent(AnAction: TNotifyEventAction; AnProcEvent: TNotifyProcEvent);
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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


<<<<<<< HEAD
<<<<<<< HEAD
function TDictionary.GetValue(AName: String): String;
=======
function TDictionary.GetValue(const AName: String): String;
>>>>>>> graemeg/fixes_2_2
=======
function TDictionary.GetValue(const AName: String): String;
>>>>>>> origin/fixes_2_2
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


<<<<<<< HEAD
<<<<<<< HEAD
function TDictionary.ReplaceStrings(Const ASource: String; Const MaxDepth: Integer = 10): String;
=======
function TDictionary.ReplaceStrings(Const ASource: String): String;
>>>>>>> graemeg/fixes_2_2
=======
function TDictionary.ReplaceStrings(Const ASource: String): String;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
      if MaxDepth > 0 then
        Result:=Result+ReplaceStrings(GetValue(FN,FV), MaxDepth-1)
      else
        Result:=Result+GetValue(FN,FV);
=======
      Result:=Result+GetValue(FN,FV);
>>>>>>> graemeg/fixes_2_2
=======
      Result:=Result+GetValue(FN,FV);
>>>>>>> origin/fixes_2_2
      P:=Pos('$(',S);
    end;
  Result:=Result+S;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TDictionary.Substitute(Const Source : String; Macros : Array of string) : String;
=======
Function Substitute(Const Source : String; Macros : Array of string) : String;
>>>>>>> graemeg/fixes_2_2
=======
Function Substitute(Const Source : String; Macros : Array of string) : String;
>>>>>>> origin/fixes_2_2
Var
  I : Integer;
begin
  I:=0;
  While I<High(Macros) do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      AddVariable(Macros[i],Macros[I+1]);
      Inc(I,2);
    end;
  Result:=ReplaceStrings(Source);
  While I<High(Macros) do
    begin
      RemoveItem(Macros[i]);
=======
=======
>>>>>>> origin/fixes_2_2
      Dictionary.AddVariable(Macros[i],Macros[I+1]);
      Inc(I,2);
    end;
  Result:=Dictionary.ReplaceStrings(Source);
  While I<High(Macros) do
    begin
      Dictionary.RemoveItem(Macros[i]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Inc(I,2);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
{****************************************************************************
                                 Default Instances
****************************************************************************}

var
  DefInstaller : TCustomInstaller;

Function Installer(InstallerClass: TInstallerClass): TCustomInstaller;
begin
  If Not Assigned(DefInstaller) then
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    DefInstaller:=InstallerClass.Create(Nil);
>>>>>>> graemeg/fixes_2_2
=======
    DefInstaller:=InstallerClass.Create(Nil);
>>>>>>> origin/fixes_2_2
  Result:=DefInstaller;
end;


Function Installer: TCustomInstaller;
begin
  Result := Installer(TFPCInstaller);
end;


<<<<<<< HEAD
<<<<<<< HEAD

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

Finalization
  FreeAndNil(CustomFpMakeCommandlineValues);
  FreeAndNil(CustomFpmakeCommandlineOptions);
  FreeAndNil(DefInstaller);
  FreeAndNil(GlobalDictionary);
  FreeAndNil(Defaults);
end.

=======
=======
>>>>>>> origin/fixes_2_2
Initialization
  OnGetApplicationName:=@GetFPMakeName;

Finalization
  FreeAndNil(DefInstaller);
  FreeAndNil(Dictionary);
  FreeAndNil(Defaults);
end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
