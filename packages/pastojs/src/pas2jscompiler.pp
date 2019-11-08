{ Author: Mattias Gaertner  2019  mattias@freepascal.org

Abstract:
  TPas2jsCompiler is the wheel boss of the pas2js compiler.
  It can be used in a command line program or compiled into an application.

  TPas2jsCompiler does not have understanding of the file system.
  DO NOT ADD filesystem related calls to this unit.
  The file system is abstracted out in TPas2JSFS (unit pas2jsfs)
  Add high-level calls to TPas2JSFS instead or  create virtual methods that can be overridden.

  FileSystem specific things should go in Pas2JSFileCache and Pas2JSFSCompiler.

Compiler-ToDos:
  Warn if -Ju and -Fu intersect
  -Fa<x>[,y] (for a program) load units <x> and [y] before uses is parsed
  Add Windows macros, see InitMacros.
}
unit Pas2jsCompiler;

{$mode objfpc}{$H+}

{$I pas2js_defines.inc}

{$IF defined(VerboseUnitQueue) or defined(VerbosePCUFiler)}
{$DEFINE ReallyVerbose}
{$ENDIF}

interface

uses
  {$IFDEF Pas2js}
  JS,
  {$ELSE}
  RtlConsts,
  {$ENDIF}
  // !! No filesystem units here.
  Classes, SysUtils, contnrs,
  jsbase, jstree, jswriter, JSSrcMap, fpjson,
  PScanner, PParser, PasTree, PasResolver, PasResolveEval, PasUseAnalyzer,
  pas2jsresstrfile, pas2jsresources, pas2jshtmlresources, pas2jsjsresources,
  FPPas2Js, FPPJsSrcMap, Pas2jsLogger, Pas2jsFS, Pas2jsPParser, Pas2jsUseAnalyzer;

const
  VersionMajor = 1;
  VersionMinor = 5;
  VersionRelease = 1;
  VersionExtra = '';
  DefaultConfigFile = 'pas2js.cfg';

//------------------------------------------------------------------------------
// Messages
const
  nOptionIsEnabled = 101; sOptionIsEnabled = 'Option "%s" is %s';
  nSyntaxModeIs = 102; sSyntaxModeIs = 'Syntax mode is %s';
  // was: nMacroDefined = 103
  // 104 in unit Pas2JSFS
  // 105 in unit Pas2JSFS
  nNameValue = 106; sNameValue = '%s: %s';
  nReadingOptionsFromFile = 107; sReadingOptionsFromFile = 'Reading options from file %s';
  nEndOfReadingConfigFile = 108; sEndOfReadingConfigFile = 'End of reading config file %s';
  nInterpretingFileOption = 109; sInterpretingFileOption = 'interpreting file option %s';
  nSourceFileNotFound = 110; sSourceFileNotFound = 'source file not found %s';
  nFileIsFolder = 111; sFileIsFolder = 'expected file, but found directory %s';
  nConfigFileSearch = 112; sConfigFileSearch = 'Configfile search: %s';
  nHandlingOption = 113; sHandlingOption = 'handling option %s';
  nQuickHandlingOption = 114; sQuickHandlingOption = 'quick handling option %s';
  nOutputDirectoryNotFound = 115; sOutputDirectoryNotFound = 'output directory not found: %s';
  nUnableToWriteFile = 116; sUnableToWriteFile = 'Unable to write file %s';
  nWritingFile = 117; sWritingFile = 'Writing file %s ...';
  nCompilationAborted = 118; sCompilationAborted = 'Compilation aborted';
  nCfgDirective = 119; sCfgDirective = 'cfg directive %s: %s';
  nUnitCycle = 120; sUnitCycle = 'Unit cycle found %s';
  nOptionForbidsCompile = 121; sOptionForbidsCompile = 'Option -Ju forbids to compile unit "%s"';
  nUnitNeedsCompileDueToUsedUnit = 122; sUnitsNeedCompileDueToUsedUnit = 'Unit "%s" needs compile due to used unit "%s"';
  nUnitNeedsCompileDueToOption = 123; sUnitsNeedCompileDueToOption = 'Unit "%s" needs compile due to option "%s"';
  nUnitNeedsCompileJSMissing = 124; sUnitsNeedCompileJSMissing = 'Unit "%s" needs compile, js file missing "%s"';
  nUnitNeedsCompilePasHasChanged = 125; sUnitsNeedCompilePasHasChanged = 'Unit "%s" needs compile, Pascal file has changed, js is %s';
  nParsingFile = 126; sParsingFile = 'Parsing %s ...';
  nCompilingFile = 127; sCompilingFile = 'Compiling %s ...';
  nExpectedButFound = 128; sExpectedButFound = 'Illegal unit name: Expected "%s", but found "%s"';
  nLinesInFilesCompiled = 129; sLinesInFilesCompiled = '%s lines in %s files compiled, %s sec%s';
  nTargetPlatformIs = 130; sTargetPlatformIs = 'Target platform is %s';
  nTargetProcessorIs = 131; sTargetProcessorIs = 'Target processor is %s';
  nMessageEncodingIs = 132; sMessageEncodingIs = 'Message encoding is %s';
  nUnableToTranslatePathToDir = 133; sUnableToTranslatePathToDir = 'Unable to translate path %s to directory %s';
  nSrcMapSourceRootIs = 134; sSrcMapSourceRootIs = 'source map "sourceRoot" is %s';
  nSrcMapBaseDirIs = 135; sSrcMapBaseDirIs = 'source map "local base directory" is %s';
  nUnitFileNotFound = 136; sUnitFileNotFound = 'unit file not found %s';
  nClassInterfaceStyleIs = 137; sClassInterfaceStyleIs = 'Class interface style is %s';
  nHandlingEnvOpts = 138; sHandlingEnvOpts = 'handling environment options %s';
  nPostProcessorInfoX = 139; sPostProcessorInfoX = 'Post processor: %s';
  nPostProcessorRunX = 140; sPostProcessorRunX = 'Run post processor: %s';
  nPostProcessorFailX = 141; sPostProcessorFailX = 'Post processor failed: %s';
  nPostProcessorWarnX = 142; sPostProcessorWarnX = 'Post processor: %s';
  nPostProcessorFinished = 143; sPostProcessorFinished = 'Post processor finished';
  nRTLIdentifierChanged = 144; sRTLIdentifierChanged = 'RTL identifier %s changed from %s to %s';
  nSkipNoConstResourcestring = 145; sSkipNoConstResourcestring = 'Resource string %s is not a constant, not adding to resourcestrings file.';
  // Note: error numbers 201+ are used by Pas2jsFileCache

//------------------------------------------------------------------------------
// Options
type
  TP2jsCompilerOption = (
    coSkipDefaultConfigs,
    coBuildAll,
    // verbosity
    coShowLogo,
    coShowErrors,
    coShowWarnings,
    coShowNotes,
    coShowHints,
    coShowInfos,
    coShowLineNumbers,
    coShowTriedUsedFiles,
    coShowConditionals,
    coShowUsedTools,
    coShowDebug,
    coShowMessageNumbers, // not in "show all"
    // checks
    coOverflowChecks,
    coRangeChecks,
    coObjectChecks,
    coAssertions,
    // features
    coAllowCAssignments,
    coAllowMacros,
    // output
    coLowerCase,
    coUseStrict,
    coWriteDebugLog,
    coWriteMsgToStdErr,
    coPrecompile, // create precompile file
    // optimizations
    coEnumValuesAsNumbers,
    coKeepNotUsedPrivates,
    coKeepNotUsedDeclarationsWPO,
    // source map
    coSourceMapCreate,
    coSourceMapInclude,
    coSourceMapFilenamesAbsolute,
    coSourceMapXSSIHeader
    );
  TP2jsCompilerOptions = set of TP2jsCompilerOption;
  TP2jsOptimization = coEnumValuesAsNumbers..coKeepNotUsedDeclarationsWPO;
  TP2jsRTLVersionCheck = (
    rvcNone,
    rvcMain,
    rvcSystem,
    rvcUnit
    );
  TP2JSResourceStringFile = (rsfNone,rsfUnit,rsfProgram);
  TResourceMode = (rmNone,rmHTML,rmJS);

const
  DefaultP2jsCompilerOptions = [coShowErrors,coSourceMapXSSIHeader,coUseStrict];
  DefaultP2JSResourceStringFile = rsfProgram;
  DefaultP2jsRTLVersionCheck = rvcNone;
  DefaultResourceMode = rmHTML;

  coShowAll = [coShowErrors..coShowDebug];
  coO1Enable = [coEnumValuesAsNumbers];
  coO1Disable = [coKeepNotUsedPrivates,coKeepNotUsedDeclarationsWPO];


  p2jscoCaption: array[TP2jsCompilerOption] of string = (
    // only used by experts or programs parsing the pas2js output, no need for resourcestrings
    'Skip default configs',
    'Build all',
    'Show logo',
    'Show errors',
    'Show warnings',
    'Show notes',
    'Show hints',
    'Show infos',
    'Show line numbers',
    'Show tried/used files',
    'Show conditionals',
    'Show used tools',
    'Show debug',
    'Show message numbers',
    'Overflow checking',
    'Range checking',
    'Method call checking',
    'Assertions',
    'Allow C assignments',
    'Allow macros',
    'Lowercase identifiers',
    'Use strict',
    'Write pas2jsdebug.log',
    'Write messages to StdErr',
    'Create precompiled units',
    'Enum values as numbers',
    'Keep not used private declarations',
    'Keep not used declarations (WPO)',
    'Create source map',
    'Include Pascal sources in source map',
    'Do not shorten filenames in source map',
    'Prepend XSSI protection )]} to source map'
    );

//------------------------------------------------------------------------------
// $mode and $modeswitches
type
  TP2jsMode = (
    p2jmObjFPC,
    p2jmDelphi
    );
  TP2jsModes = set of TP2jsMode;
const
  p2jscModeNames: array[TP2jsMode] of string = (
    'ObjFPC',
    'Delphi'
    );
  p2jsMode_SwitchSets: array[TP2jsMode] of TModeSwitches = (
    OBJFPCModeSwitches*msAllPas2jsModeSwitches+msAllPas2jsModeSwitchesReadOnly,
    DelphiModeSwitches*msAllPas2jsModeSwitches+msAllPas2jsModeSwitchesReadOnly
    );

//------------------------------------------------------------------------------
// param macros
type
  EPas2jsMacro = class(Exception);

  TOnSubstituteMacro = function(Sender: TObject; var Params: string; Lvl: integer): boolean of object;

  { TPas2jsMacro }

  TPas2jsMacro = class
  public
    Name: string;
    Description: string;
    Value: string;
    CanHaveParams: boolean;
    OnSubstitute: TOnSubstituteMacro;
  end;

  { TPas2jsMacroEngine }

  TPas2jsMacroEngine = class
  private
    fMacros: TObjectList; // list of TPas2jsMacro
    FMaxLevel: integer;
    function GetMacros(Index: integer): TPas2jsMacro;
  public
    constructor Create;
    destructor Destroy; override;
    function Count: integer;
    function AddValue(const aName, aDescription, aValue: string): TPas2jsMacro;
    function AddFunction(const aName, aDescription: string;
      const OnSubstitute: TOnSubstituteMacro; CanHaveParams: boolean): TPas2jsMacro;
    function IndexOf(const aName: string): integer;
    procedure Delete(Index: integer);
    function FindMacro(const aName: string): TPas2jsMacro;
    procedure Substitute(var s: string; Sender: TObject = nil; Lvl: integer = 0);
    property Macros[Index: integer]: TPas2jsMacro read GetMacros; default;
    property MaxLevel: integer read FMaxLevel write FMaxLevel;
  end;

//------------------------------------------------------------------------------
// Module file
type
  ECompilerTerminate = class(Exception);

  TPas2jsCompiler = class;
  TPas2JSCompilerFile = Class;

  TUsedBySection = (
    ubMainSection,
    ubImplSection
    );

  TPas2jsReaderState = (
    prsNone,
    prsReading,
    prsWaitingForUsedUnits,
    prsCanContinue,
    prsFinished,
    prsError
    );

  { TPCUSupport }

  TPCUSupport = Class(TObject)
  private
    FFile: TPas2JSCompilerFile;
  Protected
    procedure RaiseInternalError(id: TMaxPrecInt; Msg: string);
    Procedure SetPasModule(aModule: TPasModule);
    Procedure SetReaderState(aReaderState: TPas2JSReaderState);
    Procedure SetPCUFileName(Const FN: String);
  public
    constructor Create(aCompilerFile: TPas2JSCompilerFile);
    function HandleException(E: Exception): Boolean; virtual; abstract;
    procedure CreatePCUReader; virtual; abstract;
    function HasReader: Boolean; virtual; abstract;
    function ReadContinue: Boolean; virtual; abstract;
    function ReadCanContinue: Boolean; virtual; abstract;
    function FindPCU(const UseUnitName: string): string; virtual; abstract;
    procedure SetInitialCompileFlags; virtual; abstract;
    procedure WritePCU; virtual; abstract;
    procedure ReadUnit; virtual; abstract;
    property MyFile: TPas2JSCompilerFile Read FFile;
  end;

  { TFindUnitInfo }

  TFindUnitInfo = Record
    FileName: String;
    UnitName: String;
    isPCU: Boolean;
    isForeign: Boolean;
  end;

  { TLoadUnitInfo }

  TLoadUnitInfo = Record
    UseFilename, // pas or pcu filename, see IsPCU
    UseUnitname,
    InFilename: String; // can be ''
    NameExpr, InFileExpr: TPasExpr; // can be nil
    UseIsForeign: boolean;
    IsPCU: Boolean;
  end;

  { TPas2JSCompilerSupport }

  TPas2JSCompilerSupport = Class
  private
    FCompiler: TPas2JSCompiler;
  Public
    Constructor Create(aCompiler: TPas2JSCompiler); virtual;
    Property Compiler: TPas2JSCompiler read FCompiler;
  end;

  { TPas2jsCompilerFile }

  TPas2jsCompilerFile = class(TPas2JSCompilerSupport)
  private
    FConverter: TPasToJSConverter;
    FFileResolver: TPas2jsFSResolver;
    FIsForeign: boolean;
    FIsMainFile: boolean;
    FJSFilename: string;
    FJSModule: TJSElement;
    FLog: TPas2jsLogger;
    FNeedBuild: Boolean;
    FParser: TPas2jsPasParser;
    FPasFileName: String;
    FPasModule: TPasModule;
    FPasResolver: TPas2jsCompilerResolver;
    FPasUnitName: string;
    FPCUFilename: string;
    FPCUSupport: TPCUSupport;
    FReaderState: TPas2jsReaderState;
    FResourceHandler: TPas2jsResourceHandler;
    FScanner: TPas2jsPasScanner;
    FShowDebug: boolean;
    FUnitFilename: string;
    FUseAnalyzer: TPas2JSAnalyzer;
    FUsedBy: array[TUsedBySection] of TFPList; // list of TPas2jsCompilerFile
    function GetUsedBy(Section: TUsedBySection; Index: integer): TPas2jsCompilerFile;
    function GetUsedByCount(Section: TUsedBySection): integer;
    procedure HandleResources(Sender: TObject; const aFileName: String; aOptions: TStrings);
    function OnConverterIsElementUsed(Sender: TObject; El: TPasElement): boolean;
    function OnConverterIsTypeInfoUsed(Sender: TObject; El: TPasElement): boolean;
    procedure OnPasResolverLog(Sender: TObject; const Msg: String);
    procedure OnParserLog(Sender: TObject; const Msg: String);
    procedure OnScannerLog(Sender: TObject; const Msg: String);
    procedure OnUseAnalyzerMessage(Sender: TObject; Msg: TPAMessage);
    procedure HandleEParserError(E: EParserError);
    procedure HandleEPasResolve(E: EPasResolve);
    procedure HandleEPas2JS(E: EPas2JS);
    procedure HandleUnknownException(E: Exception);
    procedure HandleException(E: Exception);
    {$IFDEF Pas2js}
    procedure HandleJSException(Msg: string; E: jsvalue);
    {$ENDIF}
    procedure DoLogMsgAtEl(MsgType: TMessageType; const Msg: string;
      MsgNumber: integer; El: TPasElement);
    procedure RaiseInternalError(id: TMaxPrecInt; Msg: string);
    procedure ReaderFinished;
  public
    constructor Create(aCompiler: TPas2jsCompiler;
      const aPasFilename, aPCUFilename: string); reintroduce;
    destructor Destroy; override;
    Function CreatePCUSupport: TPCUSupport; virtual;
    function GetInitialModeSwitches: TModeSwitches;
    function IsUnitReadFromPCU: Boolean;
    function GetInitialBoolSwitches: TBoolSwitches;
    function GetInitialConverterOptions: TPasToJsConverterOptions;
    procedure CreateScannerAndParser(aFileResolver: TPas2jsFSResolver);
    procedure CreateConverter;
    function OnResolverFindModule(const UseUnitName, InFilename: String; NameExpr,
      InFileExpr: TPasExpr): TPasModule;
    procedure OnResolverCheckSrcName(const Element: TPasElement);
    procedure OpenFile(aFilename: string);// beware: this changes FileResolver.BaseDirectory
    procedure ReadUnit;
    function ReadContinue: boolean; // true=finished
    function ReaderState: TPas2jsReaderState;
    procedure CreateJS;
    procedure EmitModuleHints;
    function GetPasFirstSection: TPasSection;
    function GetPasImplSection: TPasSection;
    function GetPasMainUsesClause: TPasUsesClause;
    function GetPasImplUsesClause: TPasUsesClause;
    function GetCurPasModule: TPasModule;
    function GetModuleName: string;
    class function GetFile(aModule: TPasModule): TPas2jsCompilerFile;
  public
    Property ResourceHandler : TPas2jsResourceHandler Read FResourceHandler Write FResourceHandler;
    property PasFileName: String Read FPasFileName;
    property PasUnitName: string read FPasUnitName write FPasUnitName;// unit name in source, initialized from UnitFilename
    property Converter: TPasToJSConverter read FConverter;
    property FileResolver: TPas2jsFSResolver read FFileResolver;
    property IsForeign: boolean read FIsForeign write FIsForeign;// true = do not build
    property IsMainFile: boolean read FIsMainFile write FIsMainFile;
    property JSFilename: string read FJSFilename write FJSFilename;
    property JSModule: TJSElement read FJSModule;
    property Log: TPas2jsLogger read FLog;
    property NeedBuild: Boolean read FNeedBuild write FNeedBuild;
    property Parser: TPas2jsPasParser read FParser;
    property PascalResolver: TPas2jsCompilerResolver read FPasResolver;
    property PasModule: TPasModule read FPasModule;
    property PCUFilename: string read FPCUFilename;
    property PCUSupport: TPCUSupport Read FPCUSupport;
    property Scanner: TPas2jsPasScanner read FScanner;
    property ShowDebug: boolean read FShowDebug write FShowDebug;
    property UnitFilename: string read FUnitFilename;
    property UseAnalyzer: TPas2JSAnalyzer read FUseAnalyzer; // unit analysis
    property UsedByCount[Section: TUsedBySection]: integer read GetUsedByCount;
    property UsedBy[Section: TUsedBySection; Index: integer]: TPas2jsCompilerFile read GetUsedBy;
  end;

  { TPas2JSCompilerSupport }

  TPas2JSPostProcessorSupport = Class(TPas2JSCompilerSupport)
  Public
    Procedure WriteUsedTools; virtual; abstract;
    Procedure Clear; virtual; abstract;
    Procedure AddPostProcessor(Const Cmd: String); virtual; abstract;
    Procedure CallPostProcessors(Const JSFileName: String; aWriter: TPas2JSMapper); virtual; abstract;
  end;

  { TPas2JSConfigSupport }

  TPas2JSConfigSupport = Class(TPas2JSCompilerSupport)
  private
    FConditionEval: TCondDirectiveEvaluator;
    FCurrentCfgFilename: string;
    FCurrentCfgLineNumber: integer;
  Protected
    procedure CfgSyntaxError(const Msg: string);
    function ConditionEvalVariable(Sender: TCondDirectiveEvaluator; aName: String; out Value: string): boolean;
    procedure ConditionEvalLog(Sender: TCondDirectiveEvaluator;  Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
    property ConditionEvaluator: TCondDirectiveEvaluator read FConditionEval;
    property CurrentCfgFilename: string read FCurrentCfgFilename;
    property CurrentCfgLineNumber: integer read FCurrentCfgLineNumber;
  Protected
    // These must be overridden in descendents
    function FindDefaultConfig: String; virtual; abstract;
    function GetReader(aFileName: string): TSourceLineReader; virtual; abstract;
  Public
    constructor Create(aCompiler: TPas2jsCompiler); override;
    destructor Destroy; override;
    procedure LoadDefaultConfig;
    procedure LoadConfig(Const aFileName: String);virtual;
    property Compiler:  TPas2jsCompiler Read FCompiler;
  end;

  { TPas2jsCompiler }

  TPas2jsCompiler = class
  private
    FAllJSIntoMainJS: Boolean;
    FCompilerExe: string;
    FConfigSupport: TPas2JSConfigSupport;
    FConverterGlobals: TPasToJSConverterGlobals;
    FDefines: TStrings; // Objects can be TMacroDef
    FFiles: TPasAnalyzerKeySet; // set of TPas2jsCompilerFile, key is UnitFilename
    FFS: TPas2jsFS;
    FHasShownEncoding: boolean;
    FHasShownLogo: boolean;
    FInsertFilenames: TStringList;
    FInterfaceType: TPasClassInterfaceType;
    FLog: TPas2jsLogger;
    FMainFile: TPas2jsCompilerFile;
    FMainJSFile: String;
    FMainJSFileIsResolved: Boolean;
    FMainJSFileResolved: String;
    FMainSrcFile: String;
    FMode: TP2jsMode;
    FNamespaces: TStringList;
    FNamespacesFromCmdLine: integer;
    FOptions: TP2jsCompilerOptions;
    FOwnsFS: boolean;
    FParamMacros: TPas2jsMacroEngine;
    FPostProcessorSupport: TPas2JSPostProcessorSupport;
    FPrecompileGUID: TGUID;
    FReadingModules: TFPList; // list of TPas2jsCompilerFile ordered by uses sections
    FRTLVersionCheck: TP2jsRTLVersionCheck;
    FSrcMapBaseDir: string;
    FSrcMapSourceRoot: string;
    FUnits: TPasAnalyzerKeySet; // set of TPas2jsCompilerFile, key is PasUnitName
    FWPOAnalyzer: TPas2JSAnalyzer;
    FResourceMode : TResourceMode;
    FResources : TPas2JSResourceHandler;
    FResourceStrings : TResourceStringsFile;
    FResourceStringFile :  TP2JSResourceStringFile;
    procedure AddInsertJSFilename(const aFilename: string);
    Procedure AddNamespaces(const Paths: string; FromCmdLine: boolean);
    procedure AddUnitResourceStrings(aFile: TPas2jsCompilerFile);
    function CreateFileWriter(aFile: TPas2jsCompilerFile; const aFilename: string): TPas2JSMapper;
    procedure EmitJavaScript(aFile: TPas2jsCompilerFile; aFileWriter: TPas2JSMapper);
    function GetDefaultNamespace: String;
    function GetFileCount: integer;
    function GetResolvedMainJSFile: string;
    function GetShowDebug: boolean;
    function GetShowFullPaths: boolean;
    function GetShowLogo: Boolean; inline;
    function GetShowTriedUsedFiles: boolean;
    function GetShowUsedTools: boolean; inline;
    function GetSkipDefaultConfig: Boolean; inline;
    function GetSrcMapEnable: boolean;
    function GetSrcMapInclude: boolean;
    function GetSrcMapFilenamesAbsolute: boolean;
    function GetSrcMapXSSIHeader: boolean;
    function GetTargetPlatform: TPasToJsPlatform;
    function GetTargetProcessor: TPasToJsProcessor;
    function GetWriteDebugLog: boolean;
    function GetWriteMsgToStdErr: boolean;
    function HandleOptionOptimization(C: Char; aValue: String): Boolean;
    function IndexOfInsertJSFilename(const aFilename: string): integer;
    procedure InsertCustomJSFiles(aWriter: TPas2JSMapper);
    function LoadUsedUnit(Info: TLoadUnitInfo; Context: TPas2jsCompilerFile): TPas2jsCompilerFile;
    function OnMacroCfgDir(Sender: TObject; var Params: string; Lvl: integer): boolean;
    procedure RemoveInsertJSFilename(const aFilename: string);
    function ResolvedMainJSFile: string;
    procedure SetAllJSIntoMainJS(AValue: Boolean);
    procedure SetConverterGlobals(const AValue: TPasToJSConverterGlobals);
    procedure SetCompilerExe(AValue: string);
    procedure SetFS(AValue: TPas2jsFS);
    procedure SetMode(AValue: TP2jsMode);
    procedure SetOptions(AValue: TP2jsCompilerOptions);
    procedure SetShowDebug(AValue: boolean);
    procedure SetShowFullPaths(AValue: boolean);
    procedure SetShowLogo(AValue: Boolean);
    procedure SetShowTriedUsedFiles(AValue: boolean);
    procedure SetShowUsedTools(AValue: boolean);
    procedure SetSkipDefaultConfig(AValue: Boolean);
    procedure SetSrcMapBaseDir(const AValue: string);
    procedure SetSrcMapEnable(const AValue: boolean);
    procedure SetSrcMapInclude(const AValue: boolean);
    procedure SetSrcMapFilenamesAbsolute(const AValue: boolean);
    procedure SetSrcMapXSSIHeader(const AValue: boolean);
    procedure SetTargetPlatform(const AValue: TPasToJsPlatform);
    procedure SetTargetProcessor(const AValue: TPasToJsProcessor);
    procedure SetWriteDebugLog(const AValue: boolean);
    procedure SetWriteMsgToStdErr(const AValue: boolean);
    procedure WriteJSToFile(const MapFileName: string; aFileWriter: TPas2JSMapper);
    procedure WriteResourceStrings(aFileName: String);
    procedure WriteResources(aFileName: String);
    procedure WriteSrcMap(const MapFileName: string; aFileWriter: TPas2JSMapper);
  private
    procedure AddDefinesForTargetPlatform;
    procedure AddDefinesForTargetProcessor;
    procedure AddReadingModule(aFile: TPas2jsCompilerFile);
    procedure RemoveReadingModule(aFile: TPas2jsCompilerFile);
    procedure RegisterMessages;
  private
    // params, cfg files
    FCurParam: string;
    FResourceOutputFile: String;
    procedure LoadConfig(CfgFilename: string);
    procedure ReadEnvironment;
    procedure ReadParam(Param: string; Quick, FromCmdLine: boolean);
    procedure ReadSingleLetterOptions(const Param: string; p: integer;
      const Allowed: string; out Enabled, Disabled: string);
    procedure ReadCodeGenerationFlags(Param: String; p: integer);
    procedure ReadSyntaxFlags(Param: String; p: integer);
    procedure ReadVerbosityFlags(Param: String; p: integer);
  protected
    // Create various other classes. Virtual so they can be overridden in descendents
    function CreateJSMapper: TPas2JSMapper;virtual;
    function CreateJSWriter(aFileWriter: TPas2JSMapper): TJSWriter; virtual;
    function CreateLog: TPas2jsLogger; virtual;
    function CreateMacroEngine: TPas2jsMacroEngine;virtual;
    function CreateSrcMap(const aFileName: String): TPas2JSSrcMap; virtual;
    function CreateOptimizer: TPas2JSAnalyzer;
    // These are mandatory !
    function CreateSetOfCompilerFiles(keyType: TKeyCompareType): TPasAnalyzerKeySet; virtual; abstract;
    function CreateFS: TPas2JSFS; virtual; abstract;
    function FormatPath(Const aPath: String): String;
    function FullFormatPath(Const aPath: String): String;
    procedure WritePrecompiledFormats; virtual;
    procedure WriteHelpLine(S: String);
    // Override these for PCU format
    function CreateCompilerFile(const PasFileName, PCUFilename: String): TPas2jsCompilerFile; virtual;
    // Command-line option handling
    procedure HandleOptionPCUFormat(aValue: String); virtual;
    function HandleOptionPaths(C: Char; aValue: String; FromCmdLine: Boolean): Boolean; virtual;
    function HandleOptionJ(C: Char; aValue: String; Quick,FromCmdLine: Boolean): Boolean; virtual;
    procedure HandleOptionConfigFile(aPos: Integer; const aFileName: string); virtual;
    procedure HandleOptionInfo(aValue: string);
    // DoWriteJSFile: return false to use the default write function.
    function DoWriteJSFile(const DestFilename: String; aWriter: TPas2JSMapper): Boolean; virtual;
    procedure Compile(StartTime: TDateTime);
    procedure ProcessQueue;
    function MarkNeedBuilding(aFile: TPas2jsCompilerFile;
      Checked: TPasAnalyzerKeySet { set of TPas2jsCompilerFile, key is UnitFilename };
      var SrcFileCount: integer): boolean;
    procedure OptimizeProgram(aFile: TPas2jsCompilerFile); virtual;
    procedure CreateJavaScript(aFile: TPas2jsCompilerFile;
      Checked: TPasAnalyzerKeySet { set of TPas2jsCompilerFile, key is UnitFilename });
    procedure FinishSrcMap(SrcMap: TPas2JSSrcMap); virtual;
    // WriteSingleJSFile does not recurse
    procedure WriteSingleJSFile(aFile: TPas2jsCompilerFile; CombinedFileWriter: TPas2JSMapper);
    // WriteJSFiles recurses uses clause
    procedure WriteJSFiles(aFile: TPas2jsCompilerFile;
      CombinedFileWriter: TPas2JSMapper;
      Checked: TPasAnalyzerKeySet { set of TPas2jsCompilerFile, key is UnitFilename });
    procedure InitParamMacros;virtual;
    procedure ClearDefines;
    procedure RaiseInternalError(id: TMaxPrecInt; Msg: string);
    {$IFDEF Pas2js}
    procedure HandleJSException(Msg: string; E: jsvalue; TerminateInternal: boolean = true);
    {$ENDIF}
    function GetExitCode: Longint; virtual;
    procedure SetExitCode(Value: Longint); virtual;
    Procedure SetWorkingDir(const aDir: String); virtual;
    Procedure CreateResourceSupport; virtual;

  public
    constructor Create; virtual;
    destructor Destroy; override;
    procedure Reset; virtual;
    procedure ParamFatal(Msg: string);
    procedure Run(
      aCompilerExe: string; // needed for default config and help
      aWorkingDir: string;
      ParamList: TStrings;
      DoReset: boolean = true);
    procedure Terminate(TheExitCode: integer);

    class function GetVersion(ShortVersion: boolean): string;
    procedure WriteHelp;
    procedure WriteLogo;
    procedure WriteEncoding;
    procedure WriteVersionLine;
    procedure WriteOptions;
    procedure WriteDefines;
    procedure WriteUsedTools;
    procedure WriteFoldersAndSearchPaths;
    procedure WriteInfo;
    function GetShownMsgTypes: TMessageTypes;

    procedure AddDefine(const aName: String);
    procedure AddDefine(const aName, Value: String);
    procedure RemoveDefine(const aName: String);
    function IsDefined(const aName: String): boolean;
    procedure SetOption(Flag: TP2jsCompilerOption; Enable: boolean);

    function GetUnitInfo(const UseUnitName, InFileName, ModuleDir: String;
      PCUSupport: TPCUSupport): TFindUnitInfo;
    function FindFileWithUnitFilename(UnitFilename: string): TPas2jsCompilerFile;
    procedure LoadModuleFile(UnitFilename, UseUnitName: string;
      out aFile: TPas2jsCompilerFile; isPCU: Boolean);
    Function FindUnitJSFileName(aFileName: String): String;
    function FindLoadedUnit(const TheUnitName: string): TPas2jsCompilerFile;
    procedure AddUsedUnit(aFile: TPas2jsCompilerFile);

    function ExpandFileName(const Filename: string): string;
  public
    property CompilerExe: string read FCompilerExe write SetCompilerExe;
    property DefaultNamespace: String read GetDefaultNamespace;
    property Defines: TStrings read FDefines;
    property FS: TPas2jsFS read FFS write SetFS;
    property OwnsFS: boolean read FOwnsFS write FOwnsFS;
    property FileCount: integer read GetFileCount;
    property InterfaceType: TPasClassInterfaceType read FInterfaceType write FInterfaceType;
    property Log: TPas2jsLogger read FLog;
    property MainFile: TPas2jsCompilerFile read FMainFile;
    property Mode: TP2jsMode read FMode write SetMode;
    property Options: TP2jsCompilerOptions read FOptions write SetOptions;
    property ConverterGlobals: TPasToJSConverterGlobals read FConverterGlobals write SetConverterGlobals;
    property ParamMacros: TPas2jsMacroEngine read FParamMacros;
    property PrecompileGUID: TGUID read FPrecompileGUID write FPrecompileGUID;
    property RTLVersionCheck: TP2jsRTLVersionCheck read FRTLVersionCheck write FRTLVersionCheck;
    property SrcMapEnable: boolean read GetSrcMapEnable write SetSrcMapEnable;
    property SrcMapSourceRoot: string read FSrcMapSourceRoot write FSrcMapSourceRoot;
    property SrcMapInclude: boolean read GetSrcMapInclude write SetSrcMapInclude;
    property SrcMapXSSIHeader: boolean read GetSrcMapXSSIHeader write SetSrcMapXSSIHeader;
    property SrcMapFilenamesAbsolute: boolean read GetSrcMapFilenamesAbsolute write SetSrcMapFilenamesAbsolute;
    property ShowDebug: boolean read GetShowDebug write SetShowDebug;
    property ShowFullPaths: boolean read GetShowFullPaths write SetShowFullPaths;
    property ShowLogo: Boolean read GetShowLogo write SetShowLogo;
    property ShowTriedUsedFiles: boolean read GetShowTriedUsedFiles write SetShowTriedUsedFiles;
    property ShowUsedTools: boolean read GetShowUsedTools write SetShowUsedTools;
    property SkipDefaultConfig: Boolean read GetSkipDefaultConfig write SetSkipDefaultConfig;
    property TargetPlatform: TPasToJsPlatform read GetTargetPlatform write SetTargetPlatform;
    property TargetProcessor: TPasToJsProcessor read GetTargetProcessor write SetTargetProcessor;
    property WPOAnalyzer: TPas2JSAnalyzer read FWPOAnalyzer; // Whole Program Optimization
    property WriteDebugLog: boolean read GetWriteDebugLog write SetWriteDebugLog;
    property WriteMsgToStdErr: boolean read GetWriteMsgToStdErr write SetWriteMsgToStdErr;
    property AllJSIntoMainJS: Boolean Read FAllJSIntoMainJS Write SetAllJSIntoMainJS;
    property ExitCode: longint read GetExitCode write SetExitCode;
    property InsertFilenames: TStringList read FInsertFilenames;
    property MainJSFile: String Read FMainJSFile Write FMainJSFile;
    property MainSrcFile: String Read FMainSrcFile Write FMainSrcFile;
    property SrcMapBaseDir: string read FSrcMapBaseDir write SetSrcMapBaseDir; // includes trailing pathdelim
    property Namespaces: TStringList read FNamespaces;
    property NamespacesFromCmdLine: integer read FNamespacesFromCmdLine;
    Property ResourceMode : TResourceMode Read FResourceMode Write FResourceMode;
    Property ResourceOutputFile : String Read FResourceOutputFile Write FResourceOutputFile;
    // can be set optionally, will be freed by compiler
    property ConfigSupport: TPas2JSConfigSupport Read FConfigSupport Write FConfigSupport;
    property PostProcessorSupport: TPas2JSPostProcessorSupport Read FPostProcessorSupport Write FPostProcessorSupport;
  end;

function GetCompiledDate: string;
function GetCompiledVersion: string;
function GetCompiledTargetOS: string;
function GetCompiledTargetCPU: string;

implementation
// !! No filesystem units here.

uses pas2jsutils;


function GetCompiledDate: string;
begin
  Result:={$I %Date%};
end;

function GetCompiledVersion: string;
begin
  Result:={$I %FPCVERSION%};
end;

function GetCompiledTargetOS: string;
begin
  Result:=lowerCase({$I %FPCTARGETOS%});
end;

function GetCompiledTargetCPU: string;
begin
  Result:=lowerCase({$I %FPCTARGETCPU%});
end;

{ TPas2JSCompilerSupport }

constructor TPas2JSCompilerSupport.Create(aCompiler: TPas2JSCompiler);
begin
  FCompiler:=aCompiler;
end;

{ TPas2JSConfigSupport }

constructor TPas2JSConfigSupport.Create(aCompiler: TPas2jsCompiler);
begin
  Inherited Create(aCompiler);
  FConditionEval:=TCondDirectiveEvaluator.Create;
  FConditionEval.OnLog:=@ConditionEvalLog;
  FConditionEval.OnEvalVariable:=@ConditionEvalVariable;
end;

destructor TPas2JSConfigSupport.Destroy;
begin
  FreeAndNil(FConditionEval);
  inherited Destroy;
end;

{ TPCUSupport }

procedure TPCUSupport.RaiseInternalError(id: TMaxPrecInt; Msg: string);
begin
  MyFile.RaiseInternalError(id,msg);
end;

procedure TPCUSupport.SetPasModule(aModule: TPasModule);
begin
  MyFile.FPasModule:=aModule;
end;

procedure TPCUSupport.SetReaderState(aReaderState: TPas2JSReaderState);
begin
  MyFile.FReaderState:=aReaderState;
end;

procedure TPCUSupport.SetPCUFileName(const FN: String);
begin
  FFile.FPCUFilename:=FN;
end;

constructor TPCUSupport.Create(aCompilerFile: TPas2JSCompilerFile);
begin
  FFile:=aCompilerFile;
end;

{ TPas2jsMacroEngine }

function TPas2jsMacroEngine.GetMacros(Index: integer): TPas2jsMacro;
begin
  Result:=TPas2jsMacro(fMacros[Index]);
end;

constructor TPas2jsMacroEngine.Create;
begin
  fMacros:=TObjectList.Create(true);
  FMaxLevel:=10;
end;

destructor TPas2jsMacroEngine.Destroy;
begin
  FreeAndNil(fMacros);
  inherited Destroy;
end;

function TPas2jsMacroEngine.Count: integer;
begin
  Result:=fMacros.Count;
end;

function TPas2jsMacroEngine.AddValue(const aName, aDescription, aValue: string
  ): TPas2jsMacro;
begin
  if not IsValidIdent(aName) then
    raise EPas2jsMacro.Create('invalid macro name "'+aName+'"');
  if IndexOf(aName)>=0 then
    raise EPas2jsMacro.Create('duplicate macro name "'+aName+'"');
  Result:=TPas2jsMacro.Create;
  Result.Name:=aName;
  Result.Description:=aDescription;
  Result.Value:=aValue;
  fMacros.Add(Result);
end;

function TPas2jsMacroEngine.AddFunction(const aName, aDescription: string;
  const OnSubstitute: TOnSubstituteMacro; CanHaveParams: boolean): TPas2jsMacro;
begin
  if not IsValidIdent(aName) then
    raise EPas2jsMacro.Create('invalid macro name "'+aName+'"');
  if IndexOf(aName)>=0 then
    raise EPas2jsMacro.Create('duplicate macro name "'+aName+'"');
  Result:=TPas2jsMacro.Create;
  Result.Name:=aName;
  Result.Description:=aDescription;
  Result.CanHaveParams:=CanHaveParams;
  Result.OnSubstitute:=OnSubstitute;
  fMacros.Add(Result);
end;

function TPas2jsMacroEngine.IndexOf(const aName: string): integer;
var
  i: Integer;
begin
  for i:=0 to Count-1 do
    if CompareText(Macros[i].Name,aName)=0 then
      exit(i);
  Result:=-1;
end;

procedure TPas2jsMacroEngine.Delete(Index: integer);
begin
  fMacros.Delete(Index);
end;

function TPas2jsMacroEngine.FindMacro(const aName: string): TPas2jsMacro;
var
  i: Integer;
begin
  i:=IndexOf(aName);
  if i>=0 then
    Result:=Macros[i]
  else
    Result:=nil;
end;

procedure TPas2jsMacroEngine.Substitute(var s: string; Sender: TObject;
  Lvl: integer);
// Rules:
//   $macro or $macro$
// if Macro.OnSubstitute is set then optional brackets are allowed: $macro(params)
var
  p, StartP, BracketLvl, ParamStartP: Integer;
  MacroName, NewValue: String;
  Macro: TPas2jsMacro;
begin
  if Lvl>=MaxLevel then
    raise EPas2jsMacro.Create('macro cycle detected: "'+s+'"');
  p:=1;
  while p<length(s) do begin
    if (s[p]='$') and (s[p+1] in ['_','a'..'z','A'..'Z']) then
    begin
      StartP:=p;
      inc(p,2);
      while (p<=length(s)) and (s[p] in ['_','a'..'z','A'..'Z','0'..'9']) do
        inc(p);
      MacroName:=copy(s,StartP+1,p-StartP-1);
      Macro:=FindMacro(MacroName);
      if Macro=nil then
        raise EPas2jsMacro.Create('macro not found "'+MacroName+'" in "'+s+'"');
      NewValue:='';
      if Macro.CanHaveParams and (p<=length(s)) and (s[p]='(') then
      begin
        // read NewValue
        inc(p);
        ParamStartP:=p;
        BracketLvl:=1;
        repeat
          if p>length(s) then
            raise EPas2jsMacro.Create('missing closing bracket ) in "'+s+'"');
          case s[p] of
          '(': inc(BracketLvl);
          ')':
            if BracketLvl=1 then
            begin
              NewValue:=copy(s,ParamStartP,p-ParamStartP);
              break;
            end else begin
              dec(BracketLvl);
            end;
          end;
        until false;
      end else if (p<=length(s)) and (s[p]='$') then
        inc(p);
      if Assigned(Macro.OnSubstitute) then
      begin
        if not Macro.OnSubstitute(Sender,NewValue,Lvl+1) then
          raise EPas2jsMacro.Create('macro "'+MacroName+'" failed in "'+s+'"');
      end else
        NewValue:=Macro.Value;
      s:=LeftStr(s,StartP-1)+NewValue+copy(s,p,length(s));
      p:=StartP;
    end;
    inc(p);
  end;
end;

{ TPas2jsCompilerFile }

constructor TPas2jsCompilerFile.Create(aCompiler: TPas2jsCompiler;
  const aPasFilename, aPCUFilename: string);
var
  ub: TUsedBySection;
begin
  inherited Create(aCompiler);
  FPasFileName:=aPasFilename;
  FPCUFilename:=aPCUFilename;
  if FPasFileName<>'' then
    FUnitFilename:=FPasFileName
  else
    FUnitFilename:=FPCUFilename;
  FLog:=Compiler.Log;

  FPasResolver:=TPas2jsCompilerResolver.Create;
  FPasResolver.Owner:=Self;
  FPasResolver.OnFindModule:=@OnResolverFindModule;
  FPasResolver.OnCheckSrcName:=@OnResolverCheckSrcName;
  FPasResolver.OnLog:=@OnPasResolverLog;
  FPasResolver.Log:=Log;
  FPasResolver.AddObjFPCBuiltInIdentifiers(btAllJSBaseTypes,bfAllJSBaseProcs);
  FIsMainFile:=Compiler.FS.SameFileName(Compiler.MainSrcFile,PasFilename);
  for ub in TUsedBySection do
    FUsedBy[ub]:=TFPList.Create;

  FUseAnalyzer:=TPas2JSAnalyzer.Create;
  FUseAnalyzer.OnMessage:=@OnUseAnalyzerMessage;
  FUseAnalyzer.Resolver:=FPasResolver;

  FPCUSupport:=CreatePCUSupport;
end;

destructor TPas2jsCompilerFile.Destroy;
var
  ub: TUsedBySection;
begin
  FreeAndNil(FPCUSupport);
  FreeAndNil(FUseAnalyzer);
  for ub in TUsedBySection do
    FreeAndNil(FUsedBy[ub]);
  FreeAndNil(FJSModule);
  FreeAndNil(FConverter);
  FreeAndNil(FParser);
  FreeAndNil(FScanner);
  FreeAndNil(FFileResolver);
  FreeAndNil(FPasResolver);
  if FPasModule<>nil then
    FPasModule.ReleaseUsedUnits;
  ReleaseAndNil(TPasElement(FPasModule){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
  inherited Destroy;
end;

function TPas2jsCompilerFile.CreatePCUSupport: TPCUSupport;
begin
  Result:=Nil;
end;

function TPas2jsCompilerFile.GetInitialModeSwitches: TModeSwitches;
begin
  Result:=p2jsMode_SwitchSets[Compiler.Mode];
end;

function TPas2jsCompilerFile.GetInitialBoolSwitches: TBoolSwitches;
var
  bs: TBoolSwitches;
begin
  bs:=[bsLongStrings,bsWriteableConst];
  if coAllowMacros in Compiler.Options then
    Include(bs,bsMacro);
  if coOverflowChecks in Compiler.Options then
    Include(bs,bsOverflowChecks);
  if coRangeChecks in Compiler.Options then
    Include(bs,bsRangeChecks);
  if coObjectChecks in Compiler.Options then
    Include(bs,bsObjectChecks);
  if coAssertions in Compiler.Options then
    Include(bs,bsAssertions);
  if coShowHints in Compiler.Options then
    Include(bs,bsHints);
  if coShowNotes in Compiler.Options then
    Include(bs,bsNotes);
  if coShowWarnings in Compiler.Options then
    Include(bs,bsWarnings);
  Result:=bs;
end;

function TPas2jsCompilerFile.
  GetInitialConverterOptions: TPasToJsConverterOptions;
begin
  Result:=DefaultPasToJSOptions;

  if coUseStrict in Compiler.Options then
    Include(Result,fppas2js.coUseStrict)
  else
    Exclude(Result,fppas2js.coUseStrict);

  if coEnumValuesAsNumbers in Compiler.Options then
    Include(Result,fppas2js.coEnumNumbers);

  if coLowerCase in Compiler.Options then
    Include(Result,fppas2js.coLowerCase)
  else
    Exclude(Result,fppas2js.coLowerCase);

  case Compiler.RTLVersionCheck of
    rvcNone: ;
    rvcMain: Include(Result,fppas2js.coRTLVersionCheckMain);
    rvcSystem: Include(Result,fppas2js.coRTLVersionCheckSystem);
    rvcUnit: Include(Result,fppas2js.coRTLVersionCheckUnit);
  end;
end;

procedure TPas2jsCompilerFile.CreateScannerAndParser(aFileResolver: TPas2jsFSResolver);
var
  aUnitName: String;
  i: Integer;
  M: TMacroDef;
begin
  FFileResolver:=aFileResolver;
  // scanner
  if FScanner<>nil then
    RaiseInternalError(20180707193258,UnitFilename);
  FScanner := TPas2jsPasScanner.Create(FileResolver);
  Scanner.LogEvents:=PascalResolver.ScannerLogEvents;
  Scanner.OnLog:=@OnScannerLog;
  Scanner.OnFormatPath:=@Compiler.FormatPath;
  Scanner.RegisterResourceHandler('*',@HandleResources);

  // create parser (Note: this sets some scanner options to defaults)
  FParser := TPas2jsPasParser.Create(Scanner, FileResolver, PascalResolver);

  // set options
  Scanner.Options:=Scanner.Options+[po_StopOnErrorDirective];
  Scanner.AllowedModeSwitches:=msAllPas2jsModeSwitches;
  Scanner.ReadOnlyModeSwitches:=msAllPas2jsModeSwitchesReadOnly;
  Scanner.CurrentModeSwitches:=GetInitialModeSwitches;
  Scanner.AllowedBoolSwitches:=bsAllPas2jsBoolSwitches;
  Scanner.ReadOnlyBoolSwitches:=bsAllPas2jsBoolSwitchesReadOnly;
  Scanner.CurrentBoolSwitches:=GetInitialBoolSwitches;
  Scanner.AllowedValueSwitches:=vsAllPas2jsValueSwitches;
  Scanner.ReadOnlyValueSwitches:=vsAllPas2jsValueSwitchesReadOnly;
  Scanner.CurrentValueSwitch[vsInterfaces]:=InterfaceTypeNames[Compiler.InterfaceType];
  if coAllowCAssignments in Compiler.Options then
    Scanner.Options:=Scanner.Options+[po_cassignments];
  if Compiler.Mode=p2jmDelphi then
    Scanner.Options:=Scanner.Options+[po_delphi];
  // Note: some Scanner.Options are set by TPasResolver
  for i:=0 to Compiler.Defines.Count-1 do
    begin
    M:=TMacroDef(Compiler.Defines.Objects[i]);
    if M=nil then
      Scanner.AddDefine(Compiler.Defines[i])
    else
      Scanner.AddMacro(M.Name,M.Value);
    end;
  Scanner.CompilerVersion:=Compiler.GetVersion(true);
  Scanner.TargetPlatform:=Compiler.TargetPlatform;
  Scanner.TargetProcessor:=Compiler.TargetProcessor;
  Scanner.Resolver:=PascalResolver;

  // parser
  Parser.LogEvents:=PascalResolver.ParserLogEvents;
  Parser.OnLog:=@OnParserLog;
  Parser.Log:=Log;
  PascalResolver.P2JParser:=Parser;

  if not IsMainFile then
  begin
    aUnitName:=ExtractFilenameOnly(UnitFilename);
    if CompareText(aUnitName,'system')=0 then
      Parser.ImplicitUses.Clear;
  end;
end;

procedure TPas2jsCompilerFile.CreateConverter;
begin
  if FConverter<>nil then exit;
  FConverter:=TPasToJSConverter.Create;
  FConverter.Globals:=Compiler.ConverterGlobals;
  FConverter.Options:=GetInitialConverterOptions;
end;

procedure TPas2jsCompilerFile.OnResolverCheckSrcName(const Element: TPasElement);
var
  SrcName, ExpectedSrcName: String;
begin
  //writeln('TPas2jsCompilerFile.OnPasTreeCheckSrcName ',UnitFilename,' Name=',Element.Name,' IsMainFile=',IsMainFile);
  if (Element.ClassType=TPasUnitModule) or (Element.ClassType=TPasModule) then
  begin
    SrcName:=Element.Name;
    if IsMainFile then
    begin
      // main source is an unit
      if PasUnitName='' then
      begin
        {$IFDEF VerboseSetPasUnitName}
        writeln('TPas2jsCompilerFile.OnPasTreeCheckSrcName ',UnitFilename,' Name=',Element.Name,' IsMainFile=',IsMainFile);
        {$ENDIF}
        PasUnitName:=SrcName;
        Compiler.AddUsedUnit(Self);
      end;
    end else begin
      // an unit name must fit its filename
      ExpectedSrcName:=ExtractFilenameOnly(UnitFilename);
      if CompareText(SrcName,ExpectedSrcName)=0 then
        exit; // ok
      Parser.RaiseParserError(nExpectedButFound,[ExpectedSrcName,SrcName]);
    end;
  end;
end;

function TPas2jsCompilerFile.GetUsedBy(Section: TUsedBySection; Index: integer
  ): TPas2jsCompilerFile;
begin
  Result:=TPas2jsCompilerFile(FUsedBy[Section][Index]);
end;

function TPas2jsCompilerFile.GetUsedByCount(Section: TUsedBySection): integer;
begin
  Result:=FUsedBy[Section].Count;
end;

procedure TPas2jsCompilerFile.HandleResources(Sender: TObject; const aFileName: String; aOptions: TStrings);
begin
  // maybe emit message ?
  FResourceHandler.StartUnit(PasModule.Name);
  FResourceHandler.HandleResource(aFileName,aOptions);
end;

function TPas2jsCompilerFile.OnConverterIsElementUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  if (Compiler.WPOAnalyzer<>nil)
      and not (coKeepNotUsedDeclarationsWPO in Compiler.Options) then
    Result:=Compiler.WPOAnalyzer.IsUsed(El)
  else if not (coKeepNotUsedPrivates in Compiler.Options) then
    Result:=UseAnalyzer.IsUsed(El)
  else
    Result:=true;
  if Sender=nil then ;
end;

function TPas2jsCompilerFile.OnConverterIsTypeInfoUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  if (Compiler.WPOAnalyzer<>nil)
      and not (coKeepNotUsedDeclarationsWPO in Compiler.Options) then
    Result:=Compiler.WPOAnalyzer.IsTypeInfoUsed(El)
  else if not (coKeepNotUsedPrivates in Compiler.Options) then
    Result:=UseAnalyzer.IsTypeInfoUsed(El)
  else
    Result:=true;
  if Sender=nil then ;
end;

procedure TPas2jsCompilerFile.OnPasResolverLog(Sender: TObject; const Msg: String);
var
  aResolver: TPasResolver;
begin
  if Msg='' then ; // ignore standard formatted message
  aResolver:=TPasResolver(Sender);
  DoLogMsgAtEl(aResolver.LastMsgType,aResolver.LastMsg,aResolver.LastMsgNumber,
          aResolver.LastElement);
  if Sender=nil then ;
end;

procedure TPas2jsCompilerFile.OnParserLog(Sender: TObject; const Msg: String);
var
  aParser: TPasParser;
  aScanner: TPascalScanner;
begin
  if Msg='' then ; // ignore standard formatted message
  aParser:=TPasParser(Sender);
  aScanner:=aParser.Scanner;
  Log.Log(aParser.LastMsgType,aParser.LastMsg,aParser.LastMsgNumber,
          aScanner.CurFilename,aScanner.CurRow,aScanner.CurColumn);
  if Sender=nil then ;
end;

procedure TPas2jsCompilerFile.OnScannerLog(Sender: TObject; const Msg: String);
var
  aScanner: TPas2jsPasScanner;
begin
  if Msg='' then ; // ignore standard formatted message
  aScanner:=TPas2jsPasScanner(Sender);
  Log.Log(aScanner.LastMsgType,aScanner.LastMsg,aScanner.LastMsgNumber,
          aScanner.CurFilename,aScanner.CurRow,aScanner.CurColumn);
  if Sender=nil then ;
end;

procedure TPas2jsCompilerFile.OnUseAnalyzerMessage(Sender: TObject;
  Msg: TPAMessage);
begin
  Log.Log(Msg.MsgType,Msg.MsgText,Msg.MsgNumber,Msg.Filename,Msg.Row,Msg.Col);
  if Sender=nil then ;
end;

procedure TPas2jsCompilerFile.HandleEParserError(E: EParserError);
begin
  Log.Log(Parser.LastMsgType,Parser.LastMsg,Parser.LastMsgNumber,
          E.Filename,E.Row,E.Column);
  Compiler.Terminate(ExitCodeSyntaxError);
end;

procedure TPas2jsCompilerFile.HandleEPasResolve(E: EPasResolve);
var
  aFilename: String;
  aRow, aColumn: integer;
begin
  if E.PasElement<>nil then
  begin
    aFilename:=E.PasElement.SourceFilename;
    PascalResolver.UnmangleSourceLineNumber(E.PasElement.SourceLinenumber,aRow,aColumn);
  end else begin
    aFilename:=Scanner.CurFilename;
    aRow:=Scanner.CurRow;
    aColumn:=Scanner.CurColumn;
  end;
  Log.Log(E.MsgType,E.Message,E.MsgNumber,aFilename,aRow,aColumn);
  Compiler.Terminate(ExitCodeSyntaxError);
end;

procedure TPas2jsCompilerFile.HandleEPas2JS(E: EPas2JS);
var
  aFilename: String;
  aRow, aColumn: integer;
begin
  if E.PasElement<>nil then
  begin
    aFilename:=E.PasElement.SourceFilename;
    PascalResolver.UnmangleSourceLineNumber(E.PasElement.SourceLinenumber,aRow,aColumn);
    Log.Log(E.MsgType,E.Message,E.MsgNumber,aFilename,aRow,aColumn);
  end else begin
    Log.Log(E.MsgType,E.Message,E.MsgNumber);
  end;
  Compiler.Terminate(ExitCodeConverterError);
end;

procedure TPas2jsCompilerFile.HandleUnknownException(E: Exception);
begin
  if not (E is ECompilerTerminate) then
    Log.Log(mtFatal,'bug: uncaught '+E.ClassName+': '+E.Message,0); // must use on E:ECompilerTerminate do raise;
  Log.Log(mtFatal,E.ClassName+': '+E.Message,0);
  Compiler.Terminate(ExitCodeErrorInternal);
  // Note: a "raise E" is not allowed by caught exceptions, try..except will free it
end;

procedure TPas2jsCompilerFile.HandleException(E: Exception);
begin
  {$IFDEF ReallyVerbose}
  writeln('TPas2jsCompilerFile.HandleException ',E.ClassName,' ',E.Message);
  {$ENDIF}
  if ShowDebug then
    Log.LogExceptionBackTrace(E);
  if E is EScannerError then
  begin
    Log.Log(Scanner.LastMsgType,Scanner.LastMsg,Scanner.LastMsgNumber,
            Scanner.CurFilename,Scanner.CurRow,Scanner.CurColumn);
    Compiler.Terminate(ExitCodeSyntaxError);
  end else if E is EParserError then
    HandleEParserError(EParserError(E))
  else if E is EPasResolve then
    HandleEPasResolve(EPasResolve(E))
  else if E is EPas2JS then
    HandleEPas2JS(EPas2JS(E))
  else if E is EFileNotFoundError then
  begin
    if (E.Message<>'') or (Log.LastMsgType<>mtFatal) then
      Log.Log(mtFatal,E.Message);
    Compiler.Terminate(ExitCodeFileNotFound);
  end
  else if E is EPas2jsFS then
  begin
    Log.Log(mtFatal,E.Message);
    Compiler.Terminate(ExitCodeFileNotFound);
  end
  else if Assigned(PCUSupport) and PCUSupport.HandleException(E) then
  else
    HandleUnknownException(E);
end;

{$IFDEF Pas2js}
procedure TPas2jsCompilerFile.HandleJSException(Msg: string; E: jsvalue);
begin
  Compiler.HandleJSException(Msg,E,true);
end;
{$ENDIF}

procedure TPas2jsCompilerFile.DoLogMsgAtEl(MsgType: TMessageType;
  const Msg: string; MsgNumber: integer; El: TPasElement);
var
  Line, Col: integer;
  Filename: String;
begin
  if (El<>nil) then
  begin
    Filename:=El.SourceFilename;
    TPasResolver.UnmangleSourceLineNumber(El.SourceLinenumber,Line,Col);
  end else begin
    Filename:='';
    Line:=0;
    Col:=0;
  end;
  Log.Log(MsgType,Msg,MsgNumber,Filename,Line,Col);
end;

procedure TPas2jsCompilerFile.RaiseInternalError(id: TMaxPrecInt; Msg: string);
begin
  Compiler.RaiseInternalError(id,Msg);
end;

function TPas2jsCompilerFile.IsUnitReadFromPCU: Boolean;
begin
  Result:=Assigned(PCUSupport) and PCUSupport.HasReader;
end;

procedure TPas2jsCompilerFile.ReaderFinished;

begin
  FReaderState:=prsFinished;
  try
    Compiler.RemoveReadingModule(Self);

    if coWriteDebugLog in Compiler.Options then
    begin
      Log.DebugLogWriteLn('Pas-Module:');
      Log.DebugLogWriteLn(PasModule.GetDeclaration(true));
    end;

    if Assigned(PCUSupport) and not PCUSupport.HasReader then
      UseAnalyzer.Options:=UseAnalyzer.Options+[paoImplReferences];

    {$IFDEF VerboseUnitQueue}
    writeln('TPas2jsCompilerFile.ReaderFinished analyzing ',UnitFilename,' ...');
    {$ENDIF}
    UseAnalyzer.AnalyzeModule(FPasModule);
    {$IFDEF ReallyVerbose}
    writeln('TPas2jsCompilerFile.ReaderFinished analyzed ',UnitFilename,' ScopeModule=',GetObjName(UseAnalyzer.ScopeModule));
    {$ENDIF}
    if Assigned(PCUSupport) and Not PCUSupport.HasReader
        and (coPrecompile in Compiler.Options) then
      PCUSupport.WritePCU;
  except
    on E: ECompilerTerminate do
      raise;
    on E: Exception do
      HandleException(E);
    {$IFDEF pas2js}
    else
      HandleJSException('[20181031190529] TPas2jsCompilerFile.ReaderFinished File="'+UnitFilename+'"',
                        JSExceptValue);
    {$ENDIF}
  end;
end;

procedure TPas2jsCompilerFile.OpenFile(aFilename: string);
begin
  FPasFilename:=aFilename;
  try
    Scanner.OpenFile(PasFilename);
  except
    on E: ECompilerTerminate do
      raise;
    on E: Exception do
      HandleException(E);
    {$IFDEF pas2js}
    else HandleJSException('[20181031190536] TPas2jsCompilerFile.OpenFile "'+aFilename+'"',JSExceptValue);
    {$ENDIF}
  end;
end;

procedure TPas2jsCompilerFile.ReadUnit;
begin
  if ShowDebug then
    Log.LogMsg(nParsingFile,[QuoteStr(UnitFilename)]);
  if FPasModule<>nil then
    Compiler.RaiseInternalError(20180305190321,UnitFilename);
  FReaderState:=prsReading;
  try
    {$IFDEF VerboseUnitQueue}
    writeln('TPas2jsCompilerFile.ReadUnit ',UnitFilename,' START');
    {$ENDIF}
    Compiler.AddReadingModule(Self);
    PascalResolver.InterfaceOnly:=IsForeign;

    if IsUnitReadFromPCU then
      PCUSupport.ReadUnit
    else
    begin
      if IsMainFile then
        Parser.ParseMain(FPasModule)
      else
        Parser.ParseSubModule(FPasModule);
      if Parser.CurModule=nil then
        ReaderFinished
      else
        FReaderState:=prsWaitingForUsedUnits;
    end;
    {$IFDEF VerboseUnitQueue}
    writeln('TPas2jsCompilerFile.ReadUnit ',UnitFilename,' ReaderState=',ReaderState);
    {$ENDIF}
  except
    on E: ECompilerTerminate do
      raise;
    on E: Exception do
      HandleException(E);
    {$IFDEF pas2js}
    else
      HandleJSException('[20181031190541] TPas2jsCompilerFile.ReadUnit File="'+UnitFilename+'"',
                        JSExceptValue);
    {$ENDIF}
  end;
  if FReaderState=prsReading then
    FReaderState:=prsError;
  if (PasModule<>nil) and (PasModule.CustomData=nil) then
    PasModule.CustomData:=Self;
end;

function TPas2jsCompilerFile.ReadContinue: boolean;
begin
  Result:=true;
  if ShowDebug then
    Log.LogPlain(['Debug: Continue reading unit "',UnitFilename,'"...']);
  if FPasModule=nil then
    Compiler.RaiseInternalError(20180305190338,UnitFilename);
  FReaderState:=prsReading;
  try
    {$IFDEF VerboseUnitQueue}
    writeln('TPas2jsCompilerFile.ReadContinue ',UnitFilename);
    {$ENDIF}
    if Assigned(PCUSupport) and PCUSupport.HasReader then
      Result:=PCUSupport.ReadContinue
    else
      begin
      Parser.ParseContinue;
      Result:=Parser.CurModule=nil;
      end;
    {$IFDEF VerboseUnitQueue}
    writeln('TPas2jsCompilerFile.ReadContinue ',UnitFilename,' finished=',Result);
    {$ENDIF}
    if Result then
      ReaderFinished
    else
      FReaderState:=prsWaitingForUsedUnits;
  except
    on E: ECompilerTerminate do
      raise;
    on E: Exception do
      HandleException(E);
    {$IFDEF pas2js}
    else
      HandleJSException('[20181031190545] TPas2jsCompilerFile.ReadContinue File="'+UnitFilename+'"',
                        JSExceptValue);
    {$ENDIF}
  end;
  if FReaderState=prsReading then
    FReaderState:=prsError;
end;

function TPas2jsCompilerFile.ReaderState: TPas2jsReaderState;
var
  Section: TPasSection;
begin
  Result:=FReaderState;
  if Result=prsWaitingForUsedUnits then
  begin
    if Assigned(PCUSupport) and PCUSupport.HasReader then
    begin
      If PCUSupport.ReadCanContinue then
        Result:=prsCanContinue;
    end
    else
    begin
      if Parser.CanParseContinue(Section) then
        Result:=prsCanContinue;
    end;
  end;
end;

procedure TPas2jsCompilerFile.CreateJS;
begin
  //writeln('TPas2jsCompilerFile.CreateJS START ',UnitFilename,' JS=',GetObjName(FJSModule));
  try
    // convert
    CreateConverter;
    Converter.OnIsElementUsed:=@OnConverterIsElementUsed;
    Converter.OnIsTypeInfoUsed:=@OnConverterIsTypeInfoUsed;
    FJSModule:=Converter.ConvertPasElement(PasModule,PascalResolver);
    if FResourceHandler.Outputmode=romJS then
      FJSModule:=FResourceHandler.WriteJS(PasUnitName,FJSModule);
  except
    on E: ECompilerTerminate do
      raise;
    on E: Exception do
      HandleException(E);
    {$IFDEF pas2js}
    else
      HandleJSException('[20181031190549] TPas2jsCompilerFile.CreateJS File="'+UnitFilename+'"',
                        JSExceptValue);
    {$ENDIF}
  end;
  //writeln('TPas2jsCompilerFile.CreateJS END ',UnitFilename,' JS=',GetObjName(FJSModule));
end;

procedure TPas2jsCompilerFile.EmitModuleHints;
begin
  try
    // show hints only for units with sources
    if (PCUSupport=nil) or not PCUSupport.HasReader then
      begin
      //writeln('TPas2jsCompilerFile.EmitModuleHints ',UnitFilename);
      UseAnalyzer.EmitModuleHints(PasModule);
      end;
  except
    on E: ECompilerTerminate do
      raise;
    on E: Exception do
      HandleException(E);
    {$IFDEF pas2js}
    else
      HandleJSException('[20190226183324] TPas2jsCompilerFile.EmitModuleHints File="'+UnitFilename+'"',
                        JSExceptValue);
    {$ENDIF}
  end;
end;

function TPas2jsCompilerFile.GetPasFirstSection: TPasSection;
var
  aModule: TPasModule;
begin
  aModule:=GetCurPasModule;
  if aModule=nil then exit;
  if aModule.ClassType=TPasProgram then
    Result:=TPasProgram(aModule).ProgramSection
  else if aModule.ClassType=TPasLibrary then
    Result:=TPasLibrary(aModule).LibrarySection
  else
    Result:=aModule.InterfaceSection;
end;

function TPas2jsCompilerFile.GetPasImplSection: TPasSection;
var
  aModule: TPasModule;
begin
  Result:=nil;
  aModule:=GetCurPasModule;
  if aModule=nil then exit;
  Result:=aModule.ImplementationSection;
end;

function TPas2jsCompilerFile.GetPasMainUsesClause: TPasUsesClause;
var
  aModule: TPasModule;
  IntfSection: TInterfaceSection;
  PrgSection: TProgramSection;
  LibSection: TLibrarySection;
begin
  Result:=nil;
  aModule:=GetCurPasModule;
  if aModule=nil then exit;
  if aModule.ClassType=TPasModule then
  begin
    IntfSection:=TPasModule(aModule).InterfaceSection;
    if IntfSection<>nil then
      Result:=IntfSection.UsesClause;
  end else if aModule.ClassType=TPasProgram then
  begin
    PrgSection:=TPasProgram(aModule).ProgramSection;
    if PrgSection<>nil then
      Result:=PrgSection.UsesClause;
  end else if aModule.ClassType=TPasLibrary then
  begin
    LibSection:=TPasLibrary(aModule).LibrarySection;
    if LibSection<>nil then
      Result:=LibSection.UsesClause;
  end;
end;

function TPas2jsCompilerFile.GetPasImplUsesClause: TPasUsesClause;
var
  aModule: TPasModule;
begin
  Result:=nil;
  aModule:=GetCurPasModule;
  if aModule=nil then exit;
  if aModule.ImplementationSection<>nil then
    Result:=aModule.ImplementationSection.UsesClause;
end;

function TPas2jsCompilerFile.GetCurPasModule: TPasModule;
begin
  if PasModule<>nil then
    Result:=PasModule
  else if (PascalResolver<>nil) and (PascalResolver.RootElement<>nil) then
    Result:=PascalResolver.RootElement
  else if Parser<>nil then
    Result:=Parser.CurModule
  else
    Result:=nil;
end;

function TPas2jsCompilerFile.GetModuleName: string;
var
  aModule: TPasModule;
begin
  aModule:=GetCurPasModule;
  if aModule<>nil then
    Result:=aModule.Name
  else
    Result:='';
  if Result='' then
    Result:=ExtractFilenameOnly(UnitFilename);
end;

class function TPas2jsCompilerFile.GetFile(aModule: TPasModule
  ): TPas2jsCompilerFile;
var
  Scope: TPasModuleScope;
  Resolver: TPas2jsCompilerResolver;
begin
  Result:=nil;
  if (aModule=nil) or (aModule.CustomData=nil) then exit;
  if aModule.CustomData is TPas2jsCompilerFile then
    Result:=TPas2jsCompilerFile(aModule.CustomData)
  else if aModule.CustomData is TPasModuleScope then
  begin
    Scope:=TPasModuleScope(aModule.CustomData);
    Resolver:=NoNil(Scope.Owner) as TPas2jsCompilerResolver;
    Result:=Resolver.Owner as TPas2jsCompilerFile;
  end;
end;

function TPas2jsCompilerFile.OnResolverFindModule(const UseUnitName,
  InFilename: String; NameExpr, InFileExpr: TPasExpr): TPasModule;

var
  aFile: TPas2jsCompilerFile;
  UnitInfo: TFindUnitInfo;
  LoadInfo: TLoadUnitInfo;
  ModuleDir: String;
begin
  Result:=nil;
  aFile:=Nil;
  // check duplicate identifier or unit cycle
  if CompareText(ExtractFilenameOnly(UnitFilename),UseUnitname)=0 then
    Parser.RaiseParserError(nUnitCycle,[UseUnitname]);

  ModuleDir:=ExtractFilePath(PasFileName);
  UnitInfo:=Compiler.GetUnitInfo(UseUnitName,InFileName,ModuleDir,PCUSupport);
  if UnitInfo.FileName<>'' then
    begin
    LoadInfo.UseFilename:=UnitInfo.FileName;
    LoadInfo.UseUnitname:=UnitInfo.UnitName;
    LoadInfo.NameExpr:=NameExpr;
    LoadInfo.IsPCU:=UnitInfo.isPCU;
    if UnitInfo.isPCU then
      begin
      LoadInfo.InFilename:='';
      LoadInfo.InFileExpr:=Nil;
      LoadInfo.UseIsForeign:=False;
      end
    else
      begin
      LoadInfo.InFilename:=InFileName;
      LoadInfo.InFileExpr:=InFileExpr;
      LoadInfo.UseIsForeign:=UnitInfo.isForeign;
      end;
    aFile:=Compiler.LoadUsedUnit(LoadInfo,Self);
    end;
  if aFile<>nil then
    Result:=aFile.PasModule;
  // if Result=nil resolver will give a nice error position, so don't do it here
end;

{ TPas2JSConfigSupport }

procedure TPas2JSConfigSupport.CfgSyntaxError(const Msg: string);
begin
  Compiler.Log.Log(mtError,Msg,0,CurrentCfgFilename,CurrentCfgLineNumber,0);
  Compiler.Terminate(ExitCodeErrorInConfig);
end;

procedure TPas2JSConfigSupport.LoadConfig(Const aFileName: String);
type
  TSkip = (
    skipNone,
    skipIf,
    skipElse
  );
const
  IdentChars = ['a'..'z','A'..'Z','_','0'..'9'];
var
  Line: String;
  l, p, StartP: integer;

  function GetWord: String;
  begin
    StartP:=p;
    while (p<=l) and ((Line[p] in IdentChars) or (Line[p]>#127)) do inc(p);
    Result:=copy(Line,StartP,p-StartP);
    while (p<=l) and (Line[p] in [' ',#9]) do inc(p);
  end;

  procedure DebugCfgDirective(const s: string);
  begin
    Compiler.Log.LogMsg(nCfgDirective,[QuoteStr(Line),s],CurrentCfgFilename,CurrentCfgLineNumber,1,false);
  end;

var
  OldCfgFilename, Directive, aName, Expr: String;
  aFile: TSourceLineReader;
  IfLvl, SkipLvl, OldCfgLineNumber: Integer;
  Skip: TSkip;
begin
  if Compiler.ShowDebug or Compiler.ShowTriedUsedFiles then
    Compiler.Log.LogMsgIgnoreFilter(nReadingOptionsFromFile,[QuoteStr(aFilename)]);
  IfLvl:=0;
  SkipLvl:=0;
  Skip:=skipNone;
  aFile:=nil;
  try
    OldCfgFilename:=FCurrentCfgFilename;
    FCurrentCfgFilename:=aFilename;
    OldCfgLineNumber:=FCurrentCfgLineNumber;
    aFile:=GetReader(aFileName);
    while not aFile.IsEOF do begin
      Line:=aFile.ReadLine;
      FCurrentCfgLineNumber:=aFile.LineNumber;
      if Compiler.ShowDebug then
        Compiler.Log.LogMsgIgnoreFilter(nInterpretingFileOption,[QuoteStr(Line)]);
      if Line='' then continue;
      l:=length(Line);
      p:=1;
      while (p<=l) and (Line[p] in [' ',#9]) do inc(p);
      if p>l then continue; // empty line

      if (p<=l) and (Line[p]='#') then
      begin
        // cfg directive
        inc(p);
        if (p>l) or (Line[p] in [#0,#9,' ','-']) then continue; // comment
        Directive:=lowercase(GetWord);
        case Directive of
        'ifdef','ifndef':
          begin
            inc(IfLvl);
            if Skip=skipNone then
            begin
              aName:=GetWord;
              if Compiler.IsDefined(aName)=(Directive='ifdef') then
              begin
                // execute block
                if Compiler.ShowDebug then
                  DebugCfgDirective('true -> execute');
              end else begin
                // skip block
                if Compiler.ShowDebug then
                  DebugCfgDirective('false -> skip');
                SkipLvl:=IfLvl;
                Skip:=skipIf;
              end;
            end;
          end;
        'if':
          begin
            inc(IfLvl);
            if Skip=skipNone then
            begin
              Expr:=copy(Line,p,length(Line));
              if ConditionEvaluator.Eval(Expr) then
              begin
                // execute block
                if Compiler.ShowDebug then
                  DebugCfgDirective('true -> execute');
              end else begin
                // skip block
                if Compiler.ShowDebug then
                  DebugCfgDirective('false -> skip');
                SkipLvl:=IfLvl;
                Skip:=skipIf;
              end;
            end;
          end;
        'else':
          begin
            if IfLvl=0 then
              CfgSyntaxError('"'+Directive+'" without #ifdef');
            if (Skip=skipElse) and (IfLvl=SkipLvl) then
              CfgSyntaxError('"there was already an #else');
            if (Skip=skipIf) and (IfLvl=SkipLvl) then
            begin
              // if-block was skipped -> execute else block
              if Compiler.ShowDebug then
                DebugCfgDirective('execute');
              SkipLvl:=0;
              Skip:=skipNone;
            end else if Skip=skipNone then
            begin
              // if-block was executed -> skip else block
              if Compiler.ShowDebug then
                DebugCfgDirective('skip');
              Skip:=skipElse;
              SkipLvl:=IfLvl;
            end;
          end;
        'elseif':
          begin
            if IfLvl=0 then
              CfgSyntaxError('"'+Directive+'" without #ifdef');
            if (Skip=skipIf) and (IfLvl=SkipLvl) then
            begin
              // if-block was skipped -> try this elseif
              Expr:=copy(Line,p,length(Line));
              if ConditionEvaluator.Eval(Expr) then
              begin
                // execute elseif block
                if Compiler.ShowDebug then
                  DebugCfgDirective('true -> execute');
                SkipLvl:=0;
                Skip:=skipNone;
              end else begin
                // skip elseif block
                if Compiler.ShowDebug then
                  DebugCfgDirective('false -> skip');
              end;
            end else if Skip=skipNone then
            begin
              // if-block was executed -> skip without test
              if Compiler.ShowDebug then
                DebugCfgDirective('no test -> skip');
              Skip:=skipIf;
            end;
          end;
        'endif':
          begin
            if IfLvl=0 then
              CfgSyntaxError('"'+Directive+'" without #ifdef');
            dec(IfLvl);
            if IfLvl<SkipLvl then
            begin
              // end block
              if Compiler.ShowDebug then
                DebugCfgDirective('end block');
              SkipLvl:=0;
              Skip:=skipNone;
            end;
          end;
        'error':
          Compiler.ParamFatal('user defined: '+copy(Line,p,length(Line)))
        else
          if Skip=skipNone then
            CfgSyntaxError('unknown directive "#'+Directive+'"')
          else
            DebugCfgDirective('skipping unknown directive');
        end;
      end else if Skip=skipNone then
      begin
        // option line
        Line:=copy(Line,p,length(Line));
        Compiler.ReadParam(Line,false,false);
      end;
    end;
  finally
    FCurrentCfgFilename:=OldCfgFilename;
    FCurrentCfgLineNumber:=OldCfgLineNumber;
    aFile.Free;
  end;
  if Compiler.ShowDebug or Compiler.ShowTriedUsedFiles then
    Compiler.Log.LogMsgIgnoreFilter(nEndOfReadingConfigFile,[QuoteStr(aFilename)]);
end;

procedure TPas2JSConfigSupport.LoadDefaultConfig;
var
  aFileName: string;

begin
  aFileName:=FindDefaultConfig;
  if aFileName<>'' then
    LoadConfig(aFilename);
end;

procedure TPas2JSConfigSupport.ConditionEvalLog(Sender: TCondDirectiveEvaluator;
  Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
begin
  CfgSyntaxError(SafeFormat(Sender.MsgPattern,Args));
end;

function TPas2JSConfigSupport.ConditionEvalVariable(Sender: TCondDirectiveEvaluator;
  aName: String; out Value: string): boolean;
var
  i: Integer;
  M: TMacroDef;
  ms: TModeSwitch;
begin
  // check defines
  i:=Compiler.Defines.IndexOf(aName);
  if i>=0 then
  begin
    M:=TMacroDef(Compiler.Defines.Objects[i]);
    if M=nil then
      Value:=CondDirectiveBool[true]
    else
      Value:=M.Value;
    exit(true);
  end;

  // check modeswitches
  ms:=StrToModeSwitch(aName);
  if (ms<>msNone) and (ms in p2jsMode_SwitchSets[Compiler.Mode]) then
  begin
    Value:=CondDirectiveBool[true];
    exit(true);
  end;

  if Sender=nil then ;
  Result:=false;
end;

{ TPas2jsCompiler }

procedure TPas2jsCompiler.SetFS(AValue: TPas2jsFS);
begin
  if FFS=AValue then Exit;
  FOwnsFS:=false;
  FFS:=AValue;
end;

function TPas2jsCompiler.GetFileCount: integer;
begin
  Result:=FFiles.Count;
end;

function TPas2jsCompiler.GetDefaultNamespace: String;
var
  C: TClass;
begin
  Result:='';
  if FMainFile=nil then exit;
  if FMainFile.PasModule=nil then exit;
  C:=FMainFile.PasModule.ClassType;
  if (C=TPasProgram) or (C=TPasLibrary) or (C=TPasPackage) then
    Result:=FMainFile.PascalResolver.DefaultNameSpace;
end;

procedure TPas2jsCompiler.Compile(StartTime: TDateTime);
var
  Checked: TPasAnalyzerKeySet;
  CombinedFileWriter: TPas2JSMapper;
  SrcFileCount: integer;
  Seconds: TDateTime;
  ok: Boolean;
begin
  if FMainFile<>nil then
    RaiseInternalError(20170504192137,'');
  Checked:=nil;
  CombinedFileWriter:=nil;
  SrcFileCount:=0;

  CreateGUID(FPrecompileGUID);

  ok:=false;
  try
    // load main Pascal file
    LoadModuleFile(MainSrcFile,'',FMainFile,False);
    if MainFile=nil then exit;
    // parse and load Pascal files recursively
    FMainFile.ReadUnit;
    ProcessQueue;

    // whole program optimization
    if MainFile.PasModule is TPasProgram then
      OptimizeProgram(MainFile);

    // check what files need building
    Checked:=CreateSetOfCompilerFiles(kcFilename);
    MarkNeedBuilding(MainFile,Checked,SrcFileCount);
    SrcFileCount:=Checked.Count;// all modules, including skipped modules
    FreeAndNil(Checked);

    // convert all Pascal to JavaScript
    Checked:=CreateSetOfCompilerFiles(kcFilename);
    CreateJavaScript(MainFile,Checked);
    FreeAndNil(Checked);

    // write .js files
    Checked:=CreateSetOfCompilerFiles(kcFilename);
    WriteJSFiles(MainFile,Nil,Checked);
    FreeAndNil(Checked);

    // write success message
    if ExitCode=0 then
    begin
      Seconds:=(Now-StartTime)*86400;
      Log.LogMsgIgnoreFilter(nLinesInFilesCompiled,
             [IntToStr(FS.ReadLineCounter),IntToStr(SrcFileCount),
              FormatFloat('0.0',Seconds),'s']);
      ok:=true;
    end;
  finally
    Checked.Free;
    if not Ok then
      Log.LogMsgIgnoreFilter(nCompilationAborted,[]);
    CombinedFileWriter.Free;
  end;
end;

procedure TPas2jsCompiler.ProcessQueue;
var
  i: Integer;
  aFile: TPas2jsCompilerFile;
  Found: Boolean;
  Section: TPasSection;
begin
  // parse til exception or all modules have finished
  repeat
    {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
    writeln('TPas2jsCompiler.ProcessQueue FParsingModules.Count=',FReadingModules.Count);
    {$ENDIF}
    Found:=false;
    for i:=FReadingModules.Count-1 downto 0 do
      begin
      aFile:=TPas2jsCompilerFile(FReadingModules[i]);
      if aFile.ReaderState<>prsCanContinue then
        begin
        {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
        writeln('TPas2jsCompiler.ProcessQueue aFile=',aFile.UnitFilename,' NOT YET READY');
        {$ENDIF}
        if (not aFile.IsUnitReadFromPCU) and (aFile.Parser.CurModule=nil) then
          RaiseInternalError(20180306111410,'File='+aFile.UnitFilename+' Parser.CurModule=nil');
        continue;
        end;
      Found:=true;
      {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
      writeln('TPas2jsCompiler.ProcessQueue aFile=',aFile.UnitFilename);
      {$ENDIF}
      aFile.ReadContinue;
      if aFile.ReaderState=prsCanContinue then
      begin
        {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
        writeln('TPas2jsCompiler.ProcessQueue aFile=',aFile.UnitFilename,' ReadContinue buggy');
        {$ENDIF}
        RaiseInternalError(20180313130300,'File='+aFile.UnitFilename+' ReadContinue buggy');
      end;
      break;
      end;
  until not Found;
  {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
  writeln('TPas2jsCompiler.ProcessQueue END FParsingModules.Count=',FReadingModules.Count);
  {$ENDIF}

  // check consistency
  for i:=0 to FReadingModules.Count-1 do
    begin
    aFile:=TPas2jsCompilerFile(FReadingModules[i]);
    if aFile.PascalResolver=nil then
      RaiseInternalError(20180313124125,aFile.UnitFilename);
    if (Not aFile.IsUnitReadFromPCU) and (aFile.Parser.CurModule<>nil) then
      begin
      {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
      writeln('TPas2jsCompiler.ProcessQueue aFile=',aFile.UnitFilename,' was not finished');
      {$ENDIF}
      RaiseInternalError(20180305185342,aFile.UnitFilename);
      end;
    Section:=aFile.PascalResolver.GetLastSection;
    if Section=nil then
      RaiseInternalError(20180313124207,aFile.UnitFilename);
    if Section.PendingUsedIntf<>nil then
      RaiseInternalError(20180313124226,aFile.UnitFilename+' '+GetObjName(Section)+' PendingUsedIntf='+GetObjName(Section.PendingUsedIntf));
    end;
end;

function TPas2jsCompiler.MarkNeedBuilding(aFile: TPas2jsCompilerFile;
  Checked: TPasAnalyzerKeySet; var SrcFileCount: integer): boolean;

  procedure Mark(MsgNumber: integer;
    Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
  begin
    if aFile.NeedBuild then exit;
    aFile.NeedBuild:=true;
    inc(SrcFileCount);
    if ShowDebug or ShowTriedUsedFiles then
      Log.LogMsg(MsgNumber,Args,'',0,0,false);
  end;

  procedure CheckUsesClause(UsesClause: TPasUsesClause);
  var
    i: Integer;
    UsedFile: TPas2jsCompilerFile;
    aModule: TPasModule;
  begin
    if length(UsesClause)=0 then exit;
    for i:=0 to length(UsesClause)-1 do begin
      aModule:=UsesClause[i].Module as TPasModule;
      UsedFile:=TPas2jsCompilerFile.GetFile(aModule);
      if UsedFile=nil then
        RaiseInternalError(20171214121631,aModule.Name);
      if MarkNeedBuilding(UsedFile,Checked,SrcFileCount) then
      begin
        if not aFile.NeedBuild then
          Mark(nUnitNeedsCompileDueToUsedUnit,
                                  [aFile.GetModuleName,UsedFile.GetModuleName]);
      end;
    end;
  end;

begin
  Result:=false;
  //writeln('TPas2jsCompiler.MarkNeedBuilding ',aFile.UnitFilename);
  // check each file only once
  if Checked.FindItem(aFile)<>nil then
    exit(aFile.NeedBuild);
  Checked.Add(aFile);

  if AllJSIntoMainJS and (WPOAnalyzer<>nil)
  and not WPOAnalyzer.IsUsed(aFile.PasModule) then
  begin
    {$IFDEF REALLYVERBOSE}
    writeln('TPas2jsCompiler.MarkNeedBuilding module not used by WPO: ',aFile.UnitFilename);
    {$ENDIF}
    exit(false);
  end;

  // check dependencies
  //writeln('TPas2jsCompiler.MarkNeedBuilding CheckUsesClause ',aFile.UnitFilename,' MainUses');
  CheckUsesClause(aFile.GetPasMainUsesClause);
  //writeln('TPas2jsCompiler.MarkNeedBuilding CheckUsesClause ',aFile.UnitFilename,' ImplUses');
  CheckUsesClause(aFile.GetPasImplUsesClause);

  if (not aFile.NeedBuild) and (not aFile.IsForeign) then
  begin
    // this unit can be compiled
    if aFile.IsMainFile then
      Mark(nUnitNeedsCompileDueToOption,[aFile.GetModuleName,'<main source file>'])
    else if coBuildAll in Options then
      Mark(nUnitNeedsCompileDueToOption,[aFile.GetModuleName,'-B'])
    else if AllJSIntoMainJS then
      Mark(nUnitNeedsCompileDueToOption,[aFile.GetModuleName,'-Jc'])
    else if (aFile.JSFilename<>'') and (not FS.FileExists(aFile.JSFilename)) then
      Mark(nUnitNeedsCompileJSMissing,[aFile.GetModuleName,FormatPath(aFile.JSFilename)])
    else if (aFile.JSFilename<>'')
             and FS.File1IsNewer(aFile.UnitFilename,aFile.JSFilename) then
    begin
      Mark(nUnitNeedsCompilePasHasChanged,[aFile.GetModuleName,FullFormatPath(aFile.JSFilename)])
    end;
  end;

  if aFile.NeedBuild then
  begin
    // unit needs compile
    if aFile.IsForeign then
    begin
      // ... but is forbidden to compile
      Log.LogMsg(nOptionForbidsCompile,[aFile.GetModuleName]);
      Terminate(ExitCodeWriteError);
    end;
  end;

  Result:=aFile.NeedBuild;
end;

function TPas2jsCompiler.CreateOptimizer: TPas2JSAnalyzer;

begin
  Result:=TPas2JSAnalyzer.Create;
end;

procedure TPas2jsCompiler.OptimizeProgram(aFile: TPas2jsCompilerFile);
begin
  if not AllJSIntoMainJS then exit;
  if coKeepNotUsedDeclarationsWPO in Options then exit;
  if not (aFile.PasModule is TPasProgram) then exit;
  FWPOAnalyzer:=CreateOptimizer;
  FWPOAnalyzer.Resolver:=aFile.PascalResolver;
  FWPOAnalyzer.Options:=FWPOAnalyzer.Options+[paoOnlyExports];
  FWPOAnalyzer.AnalyzeWholeProgram(TPasProgram(aFile.PasModule));
end;

procedure TPas2jsCompiler.CreateJavaScript(aFile: TPas2jsCompilerFile;
  Checked: TPasAnalyzerKeySet);

  procedure CheckUsesClause(UsesClause: TPasUsesClause);
  var
    i: Integer;
    UsedFile: TPas2jsCompilerFile;
    aModule: TPasModule;
  begin
    if length(UsesClause)=0 then exit;
    for i:=0 to length(UsesClause)-1 do begin
      aModule:=UsesClause[i].Module as TPasModule;
      UsedFile:=TPas2jsCompilerFile.GetFile(aModule);
      if UsedFile=nil then
        RaiseInternalError(20171214121720,aModule.Name);
      CreateJavaScript(UsedFile,Checked);
    end;
  end;

begin
  //writeln('TPas2jsCompiler.CreateJavaScript ',aFile.UnitFilename,' JS=',GetObjName(aFile.JSModule),' Need=',aFile.NeedBuild);
  if aFile.JSModule<>nil then exit; // already created

  // check each file only once
  if Checked.ContainsItem(aFile) then exit;
  Checked.Add(aFile);

  // emit module hints
  aFile.EmitModuleHints;

  if not aFile.NeedBuild then exit;

  Log.LogMsg(nCompilingFile,[FullFormatPath(aFile.UnitFilename)],'',0,0,
    not (coShowLineNumbers in Options));

  // convert dependencies
  CheckUsesClause(aFile.GetPasMainUsesClause);
  CheckUsesClause(aFile.GetPasImplUsesClause);

  aFile.CreateJS;
end;

procedure TPas2jsCompiler.FinishSrcMap(SrcMap: TPas2JSSrcMap);
var
  LocalFilename, MapFilename, BaseDir: String;
  aFile: TPas2jsFile;
  i: Integer;
begin
  if SrcMapBaseDir<>'' then
    BaseDir:=SrcMapBaseDir
  else
    BaseDir:=ExtractFilePath(ExtractFilePath(SrcMap.LocalFilename));
  for i:=0 to SrcMap.SourceCount-1 do begin
    LocalFilename:=SrcMap.SourceFiles[i];
    if LocalFilename='' then continue;
    if SrcMapInclude and FS.FileExists(LocalFilename) then
    begin
      // include source in SrcMap
      aFile:=FS.LoadFile(LocalFilename);
      SrcMap.SourceContents[i]:=aFile.Source;
    end;
    // translate local file name
    MapFilename:=LocalFilename;
    if (BaseDir<>'') and not SrcMapFilenamesAbsolute then
    begin
      if not FS.TryCreateRelativePath(LocalFilename,BaseDir,true,false,MapFilename) then
      begin
        // e.g. file is on another partition
        if not SrcMapInclude then
        begin
          Log.Log(mtError,
            SafeFormat(sUnableToTranslatePathToDir,[QuoteStr(LocalFilename),QuoteStr(BaseDir)]),
                       nUnableToTranslatePathToDir);
          Terminate(ExitCodeConverterError);
        end;
        // the source is included, do not translate the filename
        MapFilename:=LocalFilename;
      end;
    end;
    if FilenameIsAbsolute(MapFilename)
        and SameText(SrcMapSourceRoot,'file://') then
      begin
      // Firefox needs the "file://" schema with every file
      MapFilename:='file://'+MapFilename;
      end;
    {$IFNDEF Unix}
    // use / as PathDelim
    if PathDelim<>'/' then
      MapFilename:=StringReplace(MapFilename,PathDelim,'/',[rfReplaceAll]);
    {$ENDIF}
    if LocalFilename<>MapFilename then
      SrcMap.SourceTranslatedFiles[i]:=MapFilename;
  end;
end;

function TPas2jsCompiler.DoWriteJSFile(const DestFilename: String;
  aWriter: TPas2JSMapper): Boolean;
begin
  Result:=False;
  if DestFilename='' then ;
  if aWriter=nil then ;
end;

function TPas2jsCompiler.CreateJSWriter(aFileWriter: TPas2JSMapper): TJSWriter;

begin
  Result:=TJSWriter.Create(aFileWriter);
end;

function TPas2jsCompiler.CreateJSMapper: TPas2JSMapper;

begin
  Result:=TPas2JSMapper.Create(4096);
end;

function TPas2jsCompiler.CreateSrcMap(const aFileName: String): TPas2JSSrcMap;

begin
  Result:=TPas2JSSrcMap.Create(aFileName);
end;

function TPas2jsCompiler.CreateFileWriter(aFile: TPas2jsCompilerFile;
  const aFilename: string): TPas2JSMapper;

var
  SrcMap: TPas2JSSrcMap;
  DestFileName : String;

begin
  DestFileName:=AFileName;
  if DestFileName='' then
    DestFileName:=aFile.JSFilename;
  Result:=CreateJSMapper;
  Result.DestFileName:=DestFileName;
  if SrcMapEnable then
  begin
    SrcMap:=CreateSrcMap(ExtractFilename(DestFilename));
    Result.SrcMap:=SrcMap;
    SrcMap.Release;// release the refcount from the Create
    SrcMap.SourceRoot:=SrcMapSourceRoot;
    SrcMap.LocalFilename:=aFile.JSFilename;
    if SrcMapXSSIHeader then
      SrcMap.Options:=SrcMap.Options+[smoSafetyHeader]
    else
      SrcMap.Options:=SrcMap.Options-[smoSafetyHeader];
    SrcMap.Options:=SrcMap.Options+[smoAllowSrcLine0];
  end;
end;


procedure TPas2jsCompiler.EmitJavaScript(aFile: TPas2jsCompilerFile;
  aFileWriter: TPas2JSMapper);

var
  aJSWriter: TJSWriter;
begin
  // write JavaScript
  aJSWriter:=CreateJSWriter(aFileWriter);
  try
    aJSWriter.Options:=DefaultJSWriterOptions;
    aJSWriter.IndentSize:=2;
    try
      aJSWriter.WriteJS(aFile.JSModule);
    except
      on E: Exception do begin
        if ShowDebug then
          Log.LogExceptionBackTrace(E);
        Log.LogPlain('[20180204193420] Error while creating JavaScript '+FullFormatPath(aFileWriter.DestFilename)+': '+E.Message);
        Terminate(ExitCodeErrorInternal);
      end
      {$IFDEF Pas2js}
      else HandleJSException('[20181031190520] TPas2jsCompiler.WriteJSFiles Error while creating JavaScript',JSExceptValue);
      {$ENDIF}
    end;
  Finally
    aJSWriter.Free;
  end;
end;


procedure TPas2jsCompiler.WriteJSToFile(const MapFileName: string;
  aFileWriter: TPas2JSMapper);

Var
  {$IFDEF Pas2js}
  buf: TJSArray;
  {$ELSE}
  buf: TMemoryStream;
  {$ENDIF}
  Src : String;

begin
  // write js
  try
    {$IFDEF Pas2js}
    buf:=TJSArray.new;
    {$ELSE}
    buf:=TMemoryStream.Create;
    {$ENDIF}
    try
      {$IFDEF FPC_HAS_CPSTRING}
      // UTF8-BOM
      if (Log.Encoding='') or (Log.Encoding='utf8') then
      begin
        Src:=String(UTF8BOM);
        buf.Write(Src[1],length(Src));
      end;
      {$ENDIF}
      // JS source
      {$IFDEF Pas2js}
      buf:=TJSArray(aFileWriter.Buffer).slice();
      {$ELSE}
      buf.Write(aFileWriter.Buffer^,aFileWriter.BufferLength);
      {$ENDIF}
      // source map comment
      if aFileWriter.SrcMap<>nil then
      begin
        Src:='//# sourceMappingURL='+ExtractFilename(MapFilename)+LineEnding;
        {$IFDEF Pas2js}
        buf.push(Src);
        {$ELSE}
        buf.Write(Src[1],length(Src));
        {$ENDIF}
      end;
      //SetLength(Src,buf.Position);
      //Move(buf.Memory^,Src[1],length(Src));
      //writeln('TPas2jsCompiler.WriteJSFiles ====',Src);
      //writeln('TPas2jsCompiler.WriteJSFiles =======================');
      {$IFDEF Pas2js}
      {$ELSE}
      buf.Position:=0;
      {$ENDIF}
      FS.SaveToFile(buf,aFileWriter.DestFilename);
    finally
      {$IFDEF Pas2js}
      buf:=nil;
      {$ELSE}
      buf.Free;
      {$ENDIF}
    end;
  except
    on E: Exception do begin
      if ShowDebug then
        Log.LogExceptionBackTrace(E);
      {$IFDEF FPC}
      if E.Message<>SafeFormat(SFCreateError,[aFileWriter.DestFileName]) then
      {$ENDIF}
        Log.LogPlain('Error: '+E.Message);
      Log.LogMsg(nUnableToWriteFile,[FullFormatPath(aFileWriter.DestFilename)]);
      Terminate(ExitCodeWriteError);
    end
    {$IFDEF Pas2js}
    else HandleJSException('[20181031190637] TPas2jsCompiler.WriteJSFiles',JSExceptValue,true);
    {$ENDIF}
  end;
end;

procedure TPas2jsCompiler.WriteSrcMap(const MapFileName: string;
  aFileWriter: TPas2JSMapper);

Var
  {$IFDEF Pas2js}
  buf: TJSArray;
  {$ELSE}
  buf: TMemoryStream;
  {$ENDIF}
begin
  Log.LogMsg(nWritingFile,[FullFormatPath(MapFilename)],'',0,0,
             not (coShowLineNumbers in Options));
  FinishSrcMap(aFileWriter.SrcMap);
  try
    {$IFDEF Pas2js}
    buf:=TJSArray.new;
    {$ELSE}
    buf:=TMemoryStream.Create;
    {$ENDIF}
    try
      // Note: No UTF-8 BOM in source map, Chrome 59 gives an error
      aFileWriter.SrcMap.SaveToStream(buf);
      {$IFDEF Pas2js}
      {$ELSE}
      buf.Position:=0;
      {$ENDIF}
      FS.SaveToFile(buf,MapFilename);
    finally
      {$IFDEF Pas2js}
      buf:=nil;
      {$ELSE}
      buf.Free;
      {$ENDIF}
    end;
  except
    on E: Exception do begin
      if ShowDebug then
        Log.LogExceptionBackTrace(E);
      {$IFDEF FPC}
      if E.Message<>SafeFormat(SFCreateError,[aFileWriter.DestFileName]) then
      {$ENDIF}
        Log.LogPlain('Error: '+E.Message);
      Log.LogMsg(nUnableToWriteFile,[FullFormatPath(MapFilename)]);
      Terminate(ExitCodeWriteError);
    end
    {$IFDEF Pas2js}
    else HandleJSException('[20181031190737] TPas2jsCompiler.WriteJSFiles',JSExceptValue);
    {$ENDIF}
  end;
end;

procedure TPas2jsCompiler.AddUnitResourceStrings(aFile : TPas2jsCompilerFile);

Var
  ResList : TFPList;

  Procedure AddToList(aList : TFPList);
  var
    I : integer;
  begin
    For I:=0 to aList.Count-1 do
      ResList.Add(aList[i]);
  end;

  Procedure AddUsedToList(aList : TFPList);
  var
    I : integer;

  begin
    For I:=0 to aList.Count-1 do
      if aFile.UseAnalyzer.IsUsed(TPasElement(aList[i])) then
        ResList.Add(aList[i]);
  end;

  Procedure CheckSection(aSection : TPasSection);

  begin
    if not (Assigned(aSection) and Assigned(aSection.ResStrings)) then
      exit;
    if FResourceStringFile=rsfProgram then
      AddUsedToList(aSection.ResStrings)
    else
      AddToList(aSection.ResStrings);
  end;

Var
  I : Integer;
  Res : TPasResString;
  aValue : TResEvalValue;

begin
  if FResourceStringFile=rsfUnit then
     FResourceStrings.Clear;
  ResList:=TFPList.Create;
  try
    // Program ?
    if aFile.pasModule is TPasProgram then
      CheckSection(TPasProgram(aFile.pasModule).ProgramSection)
    else if aFile.pasModule is TPasLibrary then // Library ?
      CheckSection(TPasLibrary(aFile.pasModule).LibrarySection)
    else
      begin
      // Interface
      CheckSection(aFile.PasModule.InterfaceSection);
      // Implementation
      CheckSection(aFile.PasModule.ImplementationSection);
      end;
    // Now add to file
    if ResList.Count>0 then
      begin
      FResourceStrings.StartUnit(aFile.GetModuleName);
      For I:=0 to ResList.Count-1 do
        begin
        Res:=TPasResString(ResList[i]);
        aValue:=aFile.PascalResolver.Eval(Res.Expr,[refConst],False);
        case aValue.Kind of
        {$IFDEF FPC_HAS_CPSTRING}
        revkString:
          FResourceStrings.AddString(Res.Name,TResEvalString(aValue).S);
        {$ENDIF}
        revkUnicodeString:
          FResourceStrings.AddString(Res.Name,TJSONStringType(TResEvalUTF16(aValue).S))
        else
          Log.Log(mtNote,sSkipNoConstResourcestring,nSkipNoConstResourcestring,aFile.PasFileName);
        end;
        ReleaseEvalValue(aValue);
        end;
      end;
  finally
    ResList.Free;
  end;
end;

procedure TPas2jsCompiler.WriteResourceStrings(aFileName : String);
Var
  {$IFDEF Pas2js}
  buf: TJSArray;
  {$ELSE}
  buf: TMemoryStream;
  {$ENDIF}
  S : TJSONStringType;
begin
  Log.LogMsg(nWritingFile,[FullFormatPath(aFilename)],'',0,0,False);
  try
    {$IFDEF Pas2js}
    buf:=TJSArray.new;
    {$ELSE}
    buf:=TMemoryStream.Create;
    {$ENDIF}
    try
      // Note: No UTF-8 BOM in source map, Chrome 59 gives an error
      S:=FResourceStrings.AsString;
      {$ifdef pas2js}
      buf.push(S);
      {$else}
      buf.Write(S[1],length(S));
      {$endif}
      FS.SaveToFile(buf,aFilename);
    finally
      {$IFDEF Pas2js}
      buf:=nil;
      {$ELSE}
      buf.Free;
      {$ENDIF}
    end;
  except
    on E: Exception do begin
      if ShowDebug then
        Log.LogExceptionBackTrace(E);
      {$IFDEF FPC}
      if E.Message<>SafeFormat(SFCreateError,[aFileName]) then
      {$ENDIF}
        Log.LogPlain('Error: '+E.Message);
      Log.LogMsg(nUnableToWriteFile,[FullFormatPath(aFilename)]);
      Terminate(ExitCodeWriteError);
    end
    {$IFDEF Pas2js}
    else HandleJSException('[20181031190737] TPas2jsCompiler.WriteJSFiles',JSExceptValue);
    {$ENDIF}
  end;

end;

procedure TPas2jsCompiler.WriteResources(aFileName: String);

Var
  {$IFDEF Pas2js}
  buf: TJSArray;
  {$ELSE}
  buf: TMemoryStream;
  {$ENDIF}
  S : TJSONStringType;
begin
    Log.LogMsg(nWritingFile,[FullFormatPath(aFilename)],'',0,0,False);
    try
      {$IFDEF Pas2js}
      buf:=TJSArray.new;
      {$ELSE}
      buf:=TMemoryStream.Create;
      {$ENDIF}
      try
        S:=FResources.AsString;
        {$ifdef pas2js}
        buf.push(S);
        {$else}
        buf.Write(S[1],length(S));
        {$endif}
        FS.SaveToFile(buf,aFilename);
      finally
        {$IFDEF Pas2js}
        buf:=nil;
        {$ELSE}
        buf.Free;
        {$ENDIF}
      end;
    except
      on E: Exception do begin
        if ShowDebug then
          Log.LogExceptionBackTrace(E);
        {$IFDEF FPC}
        if E.Message<>SafeFormat(SFCreateError,[aFileName]) then
        {$ENDIF}
          Log.LogPlain('Error: '+E.Message);
        Log.LogMsg(nUnableToWriteFile,[FullFormatPath(aFilename)]);
        Terminate(ExitCodeWriteError);
      end
      {$IFDEF Pas2js}
      else HandleJSException('[20181031190737] TPas2jsCompiler.WriteJSFiles',JSExceptValue);
      {$ENDIF}
    end;
end;

procedure TPas2jsCompiler.WriteSingleJSFile(aFile: TPas2jsCompilerFile; CombinedFileWriter: TPas2JSMapper);

  Procedure WriteToStandardOutput(aFileWriter : TPas2JSMapper);

  begin
  // write to stdout
    {$IFDEF HasStdErr}
    Log.WriteMsgToStdErr:=false;
    {$ENDIF}
    try
      Log.LogRaw(aFileWriter.AsString);
    finally
      {$IFDEF HasStdErr}
      Log.WriteMsgToStdErr:=coWriteMsgToStdErr in Options;
      {$ENDIF}
    end;
  end;

  Procedure CheckOutputDir(Const DestFileName : String);

  Var
    DestDir : String;

  begin
    // check output directory
    DestDir:=ChompPathDelim(ExtractFilePath(DestFilename));
    if (DestDir<>'') and not FS.DirectoryExists(DestDir) then
    begin
      Log.LogMsg(nOutputDirectoryNotFound,[FullFormatPath(DestDir)]);
      Terminate(ExitCodeFileNotFound);
    end;
    if FS.DirectoryExists(DestFilename) then
    begin
      Log.LogMsg(nFileIsFolder,[FullFormatPath(DestFilename)]);
      Terminate(ExitCodeWriteError);
    end;
  end;

Var
  aFileWriter : TPas2JSMapper;
  isSingleFile : Boolean;
  ResFileName,MapFilename : String;

begin
  aFileWriter:=CombinedFileWriter;
  try
    isSingleFile:=aFileWriter=nil;
    if isSingleFile then
      // create local writer for this file
      begin
      aFileWriter:=CreateFileWriter(aFile,'');
      if aFile.IsMainFile and Not AllJSIntoMainJS then
        begin
        InsertCustomJSFiles(aFileWriter);
        if FResources.OutputMode=romExtraJS then
          aFileWriter.WriteFile(FResources.AsString,GetResolvedMainJSFile);
        end;
      end;

    if FResourceStringFile<>rsfNone then
      AddUnitResourceStrings(aFile);
    FResources.DoneUnit(aFile.isMainFile);
    EmitJavaScript(aFile,aFileWriter);



    if aFile.IsMainFile and (TargetPlatform=PlatformNodeJS) then
      aFileWriter.WriteFile('rtl.run();'+LineEnding,aFile.UnitFilename);

    if isSingleFile or aFile.isMainFile then
      begin
      if Assigned(PostProcessorSupport) then
        PostProcessorSupport.CallPostProcessors(aFile.JSFilename,aFileWriter);

      // Give chance to descendants to write file
      if DoWriteJSFile(aFile.JSFilename,aFileWriter) then
        exit;// descendant has written -> finished

      if (aFile.JSFilename='') and (MainJSFile='.') then
        WriteToStandardOutput(aFileWriter);

      //writeln('TPas2jsCompiler.WriteJSFiles ',aFile.UnitFilename,' ',aFile.JSFilename);
      Log.LogMsg(nWritingFile,[FullFormatPath(aFileWriter.DestFilename)],'',0,0, not (coShowLineNumbers in Options));

      CheckOutputDir(aFileWriter.DestFileName);

      MapFilename:=aFileWriter.DestFilename+'.map';
      WriteJSToFile(MapFileName,aFileWriter);
      if (FResourceStringFile=rsfUnit) or (aFile.IsMainFile and (FResourceStringFile<>rsfNone)) then
        if FResourceStrings.StringsCount>0 then
          WriteResourceStrings(ChangeFileExt(aFileWriter.DestFileName,'.jrs'));
//      Writeln('IsSingleFile ',isSingleFile,' mainfile: ',aFile.IsMainFile,' filename: ', aFileWriter.DestFileName);
      if aFile.isMainFile and (FResources.OutputMode=romFile) and (FResources.ResourceCount>0) then
        begin
        ResFileName:=FResourceOutputFile;
        if ResFileName='' then
          // default is projectname-res.ext, to avoid projectname.html, used in web projects in Lazarus IDE
          ResFileName:=ChangeFileExt(aFileWriter.DestFileName,'-res'+FResources.OutputFileExtension);
        WriteResources(ResFileName);
        end;

      // write source map
      if aFileWriter.SrcMap<>nil then
        WriteSrcMap(MapFileName,aFileWriter);
      end;

  finally
    if isSingleFile then
      aFileWriter.Free;
  end;
end;


procedure TPas2jsCompiler.WriteJSFiles(aFile: TPas2jsCompilerFile; CombinedFileWriter: TPas2JSMapper; Checked: TPasAnalyzerKeySet);

  procedure CheckUsesClause(aFileWriter: TPas2JSMapper; UsesClause: TPasUsesClause);
  var
    i: Integer;
    UsedFile: TPas2jsCompilerFile;
    aModule: TPasModule;
  begin
    if length(UsesClause)=0 then exit;
    for i:=0 to length(UsesClause)-1 do begin
      aModule:=UsesClause[i].Module as TPasModule;
      UsedFile:=TPas2jsCompilerFile.GetFile(aModule);
      if UsedFile=nil then
        RaiseInternalError(20171214121720,aModule.Name);
      WriteJSFiles(UsedFile,aFileWriter,Checked);
    end;
  end;

Var
  aFileWriter : TPas2JSMapper;

begin
  // writeln('TPas2jsCompiler.WriteJSFiles START ',aFile.UnitFilename,' Need=',aFile.NeedBuild,' Checked=',Checked.ContainsItem(aFile),' JSModule=',GetObjName(aFile.JSModule));
  if (aFile.JSModule=nil) or (not aFile.NeedBuild) then exit;
  // check each file only once
  if Checked.ContainsItem(aFile) then exit;
  Checked.Add(aFile);


  aFileWriter:=CombinedFileWriter;
  if AllJSIntoMainJS and (aFileWriter=nil) then
    begin
    // create CombinedFileWriter
    aFileWriter:=CreateFileWriter(aFile,GetResolvedMainJSFile);
    InsertCustomJSFiles(aFileWriter);
    if FResources.OutputMode=romExtraJS then
      aFileWriter.WriteFile(FResources.AsString,GetResolvedMainJSFile);
    end;
  Try
    // convert dependencies
    CheckUsesClause(aFileWriter,aFile.GetPasMainUsesClause);
    CheckUsesClause(aFileWriter,aFile.GetPasImplUsesClause);

    // Write me...
    WriteSingleJSFile(aFile,aFileWriter);
  finally
    if aFileWriter<>CombinedFileWriter then
      aFileWriter.Free;
  end;
end;

procedure TPas2jsCompiler.InitParamMacros;
begin
  ParamMacros.AddValue('Pas2jsFullVersion','major.minor.release<extra>',GetVersion(false));
  ParamMacros.AddValue('Pas2jsVersion','major.minor.release',GetVersion(true));
  ParamMacros.AddFunction('CfgDir','Use within a config file. The directory of this config file',@OnMacroCfgDir,false);
  // Additionally, under windows the following special variables are recognized:

{ ToDo:
  LOCAL_APPDATA
      Usually the directory ”Local settings/Application Data” under the user’s home directory.
  APPDATA
      Usually the directory ”Application Data” under the user’s home directory.
  COMMON_APPDATA
      Usually the directory ”Application Data” under the ’All users’ directory.
  PERSONAL
      Usually the ”My documents” directory of the user.
  PROGRAM_FILES
      Usually ”program files” directory on the system drive
  PROGRAM_FILES_COMMON
      Usually the ”Common files” directory under the program files directory.
  PROFILE
      The user’s home directory.   }
end;

procedure TPas2jsCompiler.ClearDefines;
var
  i: Integer;
  M: TMacroDef;
begin
  for i:=0 to FDefines.Count-1 do
    begin
    M:=TMacroDef(FDefines.Objects[i]);
    M.Free;
    end;
  FDefines.Clear;
end;

procedure TPas2jsCompiler.RaiseInternalError(id: TMaxPrecInt; Msg: string);
begin
  Log.LogPlain('['+IntToStr(id)+'] '+Msg);
  raise Exception.Create(Msg);
end;

{$IFDEF Pas2js}
procedure TPas2jsCompiler.HandleJSException(Msg: string; E: jsvalue;
  TerminateInternal: boolean);
var
  obj: JS.TJSObject;
  Exc: Exception;
begin
  if isObject(E) then
  begin
    obj:=js.TJSObject(E);
    if isExt(obj,TJSError) then
    begin
      {AllowWriteln}
      if obj['stack'] then
        writeln(obj['stack']);
      {AllowWriteln-}
      Log.Log(mtFatal,Msg+': '+String(obj['message']));
    end else if isExt(obj,TObject) then
    begin
      if TObject(obj) is Exception then
      begin
        Exc:=Exception(TObject(obj));
        {$ifdef NodeJS}
        {AllowWriteln}
        if Exc.NodeJSError<>nil then
          writeln(Exc.NodeJSError.stack);
        {AllowWriteln-}
        {$endif}
        Log.Log(mtFatal,Msg+': ('+Exc.ClassName+') '+Exc.Message);
      end else begin
        Log.Log(mtFatal,Msg+': ('+TObject(obj).ClassName+')');
      end;
    end else
      Log.Log(mtFatal,Msg+': '+String(E));
  end else begin
    Log.Log(mtFatal,Msg+': '+String(E));
  end;
  if TerminateInternal then
    Terminate(ExitCodeErrorInternal);
end;
{$ENDIF}

function TPas2jsCompiler.GetExitCode: Longint;
begin
  Result:=System.ExitCode;
end;

procedure TPas2jsCompiler.SetExitCode(Value: Longint);
begin
  System.ExitCode:=Value;
end;

procedure TPas2jsCompiler.SetWorkingDir(const aDir: String);
begin
  // Do nothing
  if aDir='' then ;
end;

procedure TPas2jsCompiler.CreateResourceSupport;
begin
  Case FResourceMode of
    rmNone : FResources:=TNoResources.Create(FS);
    rmHTML : FResources:=THTMLResourceLinkHandler.Create(FS);
    rmJS : FResources:=TJSResourceHandler.Create(FS);
  end;
end;

procedure TPas2jsCompiler.Terminate(TheExitCode: integer);
begin
  ExitCode:=TheExitCode;
  if Log<>nil then Log.Flush;
  raise ECompilerTerminate.Create('');
end;

function TPas2jsCompiler.GetShowDebug: boolean;
begin
  Result:=coShowDebug in Options;
end;

function TPas2jsCompiler.GetShowFullPaths: boolean;
begin
  Result:=FS.ShowFullPaths;
end;

function TPas2jsCompiler.GetShowLogo: Boolean;
begin
  Result:=coShowLogo in FOptions;
end;

function TPas2jsCompiler.GetShowTriedUsedFiles: boolean;
begin
  Result:=coShowTriedUsedFiles in FOptions;
end;

function TPas2jsCompiler.GetShowUsedTools: boolean;
begin
  Result:=coShowUsedTools in Options;
end;

function TPas2jsCompiler.GetSkipDefaultConfig: Boolean;
begin
  Result:=coSkipDefaultConfigs in FOptions;
end;

function TPas2jsCompiler.GetSrcMapEnable: boolean;
begin
  Result:=coSourceMapCreate in FOptions;
end;

function TPas2jsCompiler.GetSrcMapInclude: boolean;
begin
  Result:=coSourceMapInclude in FOptions;
end;

function TPas2jsCompiler.GetSrcMapFilenamesAbsolute: boolean;
begin
  Result:=coSourceMapFilenamesAbsolute in FOptions;
end;

function TPas2jsCompiler.GetSrcMapXSSIHeader: boolean;
begin
  Result:=coSourceMapXSSIHeader in FOptions;
end;

function TPas2jsCompiler.GetTargetPlatform: TPasToJsPlatform;
begin
  Result:=FConverterGlobals.TargetPlatform;
end;

function TPas2jsCompiler.GetTargetProcessor: TPasToJsProcessor;
begin
  Result:=FConverterGlobals.TargetProcessor;
end;

function TPas2jsCompiler.GetWriteDebugLog: boolean;
begin
  Result:=coWriteDebugLog in FOptions;
end;

function TPas2jsCompiler.GetWriteMsgToStdErr: boolean;
begin
  Result:=coWriteMsgToStdErr in FOptions;
end;

procedure TPas2jsCompiler.SetCompilerExe(AValue: string);
begin
  if AValue<>'' then
    AValue:=ExpandFileName(AValue);
  if FCompilerExe=AValue then Exit;
  FCompilerExe:=AValue;
end;

procedure TPas2jsCompiler.SetMode(AValue: TP2jsMode);
begin
  if FMode=AValue then Exit;
  FMode:=AValue;
  case FMode of
    p2jmObjFPC: Options:=Options-[coAllowCAssignments];
    p2jmDelphi: Options:=Options-[coAllowCAssignments];
  end;
end;

procedure TPas2jsCompiler.SetOptions(AValue: TP2jsCompilerOptions);
begin
  if FOptions=AValue then Exit;
  FOptions:=AValue;
  Log.ShowMsgNumbers:=coShowMessageNumbers in FOptions;
  Log.ShowMsgTypes:=GetShownMsgTypes;
  FS.ShowTriedUsedFiles:=coShowTriedUsedFiles in FOptions;
end;

procedure TPas2jsCompiler.SetShowDebug(AValue: boolean);
begin
  if AValue then
    FOptions:=FOptions+[coShowNotes,coShowInfos,coShowDebug]
  else
    Exclude(FOptions,coShowNotes);
end;

procedure TPas2jsCompiler.SetShowFullPaths(AValue: boolean);
begin
  FS.ShowFullPaths:=AValue;
end;

procedure TPas2jsCompiler.SetShowLogo(AValue: Boolean);
begin
  SetOption(coShowLogo,AValue);
end;

procedure TPas2jsCompiler.SetShowTriedUsedFiles(AValue: boolean);
begin
  FS.ShowTriedUsedFiles:=AValue;
  SetOption(coShowTriedUsedFiles,AValue);
end;

procedure TPas2jsCompiler.SetShowUsedTools(AValue: boolean);
begin
  SetOption(coShowUsedTools,AValue);
end;

procedure TPas2jsCompiler.SetSkipDefaultConfig(AValue: Boolean);
begin
  SetOption(coSkipDefaultConfigs,AValue);
end;

procedure TPas2jsCompiler.SetSrcMapBaseDir(const AValue: string);
var
  NewValue: String;
begin
  NewValue:=FS.ExpandDirectory(AValue);
  if FSrcMapBaseDir=NewValue then Exit;
  FSrcMapBaseDir:=NewValue;
end;

procedure TPas2jsCompiler.SetSrcMapEnable(const AValue: boolean);
begin
  SetOption(coSourceMapCreate,AValue);
end;

procedure TPas2jsCompiler.SetSrcMapInclude(const AValue: boolean);
begin
  SetOption(coSourceMapInclude,AValue);
end;

procedure TPas2jsCompiler.SetSrcMapFilenamesAbsolute(const AValue: boolean);
begin
  SetOption(coSourceMapFilenamesAbsolute,AValue);
end;

procedure TPas2jsCompiler.SetSrcMapXSSIHeader(const AValue: boolean);
begin
  SetOption(coSourceMapXSSIHeader,AValue);
end;

procedure TPas2jsCompiler.SetTargetPlatform(const AValue: TPasToJsPlatform);
var
  OldPlatform: TPasToJsPlatform;
begin
  OldPlatform:=FConverterGlobals.TargetPlatform;
  if OldPlatform=AValue then Exit;
  RemoveDefine(PasToJsPlatformNames[OldPlatform]);
  FConverterGlobals.TargetPlatform:=AValue;
  if AValue=PlatformNodeJS then
    AllJSIntoMainJS:=true;
  AddDefinesForTargetPlatform;
end;

procedure TPas2jsCompiler.SetTargetProcessor(const AValue: TPasToJsProcessor);
var
  OldTargetProcessor: TPasToJsProcessor;
begin
  OldTargetProcessor:=FConverterGlobals.TargetProcessor;
  if OldTargetProcessor=AValue then Exit;
  RemoveDefine(PasToJsProcessorNames[OldTargetProcessor]);
  FConverterGlobals.TargetProcessor:=AValue;
  AddDefinesForTargetProcessor;
end;

procedure TPas2jsCompiler.SetWriteDebugLog(const AValue: boolean);
begin
  SetOption(coWriteDebugLog,AValue);
end;

procedure TPas2jsCompiler.SetWriteMsgToStdErr(const AValue: boolean);
begin
  SetOption(coWriteMsgToStdErr,AValue);
  {$IFDEF HasStdErr}
  Log.WriteMsgToStdErr:=AValue;
  {$ENDIF}
end;

procedure TPas2jsCompiler.AddDefinesForTargetPlatform;
begin
  AddDefine(PasToJsPlatformNames[TargetPlatform]);
  AddDefine('Pas2JSTargetOS',PasToJsPlatformNames[TargetPlatform]);
end;

procedure TPas2jsCompiler.AddDefinesForTargetProcessor;
begin
  AddDefine(PasToJsProcessorNames[TargetProcessor]);
  AddDefine('Pas2JSTargetCPU',PasToJsProcessorNames[TargetProcessor]);
  case TargetProcessor of
    ProcessorECMAScript5: AddDefine('ECMAScript', '5');
    ProcessorECMAScript6: AddDefine('ECMAScript', '6');
  end;
end;

procedure TPas2jsCompiler.AddReadingModule(aFile: TPas2jsCompilerFile);
begin
  if FReadingModules.IndexOf(aFile)>=0 then
    exit;
  FReadingModules.Add(aFile);
end;

procedure TPas2jsCompiler.RemoveReadingModule(aFile: TPas2jsCompilerFile);
begin
  FReadingModules.Remove(aFile);
end;

procedure TPas2jsCompiler.RegisterMessages;
var
  LastMsgNumber: integer;

  procedure r(MsgType: TMessageType; MsgNumber: integer; const MsgPattern: string);
  var
    s: String;
  begin
    if (LastMsgNumber>=0) and (MsgNumber<>LastMsgNumber+1) then
    begin
      if MsgNumber>LastMsgNumber+1 then
        s:='TPas2jsCompiler.RegisterMessages: gap in registered message numbers: '+IntToStr(LastMsgNumber+1)+' '+IntToStr(MsgNumber)
      else
        s:='TPas2jsCompiler.RegisterMessages: not ascending order in registered message numbers: Last='+IntToStr(LastMsgNumber)+' New='+IntToStr(MsgNumber);
      RaiseInternalError(20170504161422,s);
    end;
    Log.RegisterMsg(MsgType,MsgNumber,MsgPattern);
    LastMsgNumber:=MsgNumber;
  end;

begin
  LastMsgNumber:=-1;
  r(mtInfo,nOptionIsEnabled,sOptionIsEnabled);
  r(mtInfo,nSyntaxModeIs,sSyntaxModeIs);
  LastMsgNumber:=-1; // was nMacroDefined 103
  r(mtInfo,nUsingPath,sUsingPath);
  r(mtNote,nFolderNotFound,sFolderNotFound);
  r(mtInfo,nNameValue,sNameValue);
  r(mtInfo,nReadingOptionsFromFile,sReadingOptionsFromFile);
  r(mtInfo,nEndOfReadingConfigFile,sEndOfReadingConfigFile);
  r(mtDebug,nInterpretingFileOption,sInterpretingFileOption);
  r(mtFatal,nSourceFileNotFound,sSourceFileNotFound);
  r(mtFatal,nFileIsFolder,sFileIsFolder);
  r(mtInfo,nConfigFileSearch,sConfigFileSearch);
  r(mtDebug,nHandlingOption,sHandlingOption);
  r(mtDebug,nQuickHandlingOption,sQuickHandlingOption);
  r(mtFatal,nOutputDirectoryNotFound,sOutputDirectoryNotFound);
  r(mtError,nUnableToWriteFile,sUnableToWriteFile);
  r(mtInfo,nWritingFile,sWritingFile);
  r(mtFatal,nCompilationAborted,sCompilationAborted);
  r(mtDebug,nCfgDirective,sCfgDirective);
  r(mtError,nUnitCycle,sUnitCycle);
  r(mtError,nOptionForbidsCompile,sOptionForbidsCompile);
  r(mtInfo,nUnitNeedsCompileDueToUsedUnit,sUnitsNeedCompileDueToUsedUnit);
  r(mtInfo,nUnitNeedsCompileDueToOption,sUnitsNeedCompileDueToOption);
  r(mtInfo,nUnitNeedsCompileJSMissing,sUnitsNeedCompileJSMissing);
  r(mtInfo,nUnitNeedsCompilePasHasChanged,sUnitsNeedCompilePasHasChanged);
  r(mtInfo,nParsingFile,sParsingFile);
  r(mtInfo,nCompilingFile,sCompilingFile);
  r(mtError,nExpectedButFound,sExpectedButFound);
  r(mtInfo,nLinesInFilesCompiled,sLinesInFilesCompiled);
  r(mtInfo,nTargetPlatformIs,sTargetPlatformIs);
  r(mtInfo,nTargetProcessorIs,sTargetProcessorIs);
  r(mtInfo,nMessageEncodingIs,sMessageEncodingIs);
  r(mtError,nUnableToTranslatePathToDir,sUnableToTranslatePathToDir);
  r(mtInfo,nSrcMapSourceRootIs,sSrcMapSourceRootIs);
  r(mtInfo,nSrcMapBaseDirIs,sSrcMapBaseDirIs);
  r(mtFatal,nUnitFileNotFound,sUnitFileNotFound);
  r(mtInfo,nClassInterfaceStyleIs,sClassInterfaceStyleIs);
  r(mtInfo,nHandlingEnvOpts,sHandlingEnvOpts);
  r(mtInfo,nPostProcessorInfoX,sPostProcessorInfoX);
  r(mtInfo,nPostProcessorRunX,sPostProcessorRunX);
  r(mtError,nPostProcessorFailX,sPostProcessorFailX);
  r(mtWarning,nPostProcessorWarnX,sPostProcessorWarnX);
  r(mtInfo,nPostProcessorFinished,sPostProcessorFinished);
  r(mtInfo,nRTLIdentifierChanged,sRTLIdentifierChanged);
  Pas2jsPParser.RegisterMessages(Log);
end;

procedure TPas2jsCompiler.LoadConfig(CfgFilename: string);
begin
  ConfigSupport.LoadConfig(CfgFileName);
end;

procedure TPas2jsCompiler.ReadEnvironment;
var
  s: String;
  List: TStrings;
begin
  s:=GetEnvironmentVariable('PAS2JS_OPTS');
  if s='' then exit;
  if ShowDebug then
    Log.LogMsgIgnoreFilter(nHandlingEnvOpts,['PAS2JS_OPTS=['+s+']']);
  List:=TStringList.Create;
  try
    SplitCmdLineParams(s,List);
    for s in List do
      if s<>'' then
        ReadParam(s,false,false);
  finally
    List.Free;
  end;
end;

procedure TPas2jsCompiler.ParamFatal(Msg: string);
begin
  if FCurParam<>'' then
    Msg:='parameter '+FCurParam+': '+Msg;
  if Assigned(ConfigSupport) and  (ConfigSupport.CurrentCfgFilename<>'') then
    Log.Log(mtFatal,Msg,0,ConfigSupport.CurrentCfgFilename,ConfigSupport.CurrentCfgLineNumber,0)
  else
    Log.LogPlain(['Fatal: ',Msg]);
  Terminate(ExitCodeErrorInParams);
end;

procedure TPas2jsCompiler.HandleOptionPCUFormat(aValue: String);
begin
  ParamFatal('No support in this compiler for precompiled format '+aValue);
end;

function TPas2jsCompiler.HandleOptionPaths(C: Char; aValue: String;
  FromCmdLine: Boolean): Boolean;
Var
  ErrorMsg: String;
begin
  Result:=True;
  case c of
    'N': AddNamespaces(aValue,FromCmdLine);
    'r': Log.Log(mtNote,'-Fr not yet implemented');
    'e': Log.OutputFilename:=aValue;
  else
    ErrorMsg:=FS.HandleOptionPaths(C,aValue,FromCmdLine);
    if ErrorMsg<>'' then
      ParamFatal(ErrorMsg);
  end;
end;

function TPas2jsCompiler.HandleOptionOptimization(C: Char; aValue: String): Boolean;
Var
  Enable: Boolean;
begin
  Result:=True;
  case C of
  '-': Options:=Options-coO1Enable+coO1Disable;
  '1': Options:=Options+coO1Enable-coO1Disable;
  'o':
    begin
    if aValue='' then
      ParamFatal('missing -Oo option');
    Enable:=true;
    c:=aValue[length(aValue)];
    if c in ['+','-'] then
    begin
      Enable:=c='+';
      Delete(aValue,length(aValue),1);
    end;
    Case LowerCase(avalue) of
     'enumnumbers': SetOption(coEnumValuesAsNumbers,Enable);
     'emovenotusedprivates': SetOption(coKeepNotUsedPrivates,not Enable);
     'removenotuseddeclarations': SetOption(coKeepNotUsedDeclarationsWPO,not Enable)
    else
      Result:=False;
    end;
    end;
  else
    Result:=False;
  end;

end;

function TPas2jsCompiler.HandleOptionJ(C: Char; aValue: String;
  Quick, FromCmdLine: Boolean): Boolean;

Var
  S, ErrorMsg, aName: String;
  i: Integer;
  enable: Boolean;
  pbi: TPas2JSBuiltInName;

begin
  Result:=True;
  case c of
  'c': // -Jc concatenate
    begin
      if aValue='' then
        AllJSIntoMainJS:=true
      else if (AValue='-') then
        AllJSIntoMainJS:=false
      else
        ParamFatal('invalid value (-Jc) "'+aValue+'"');
    end;
  'e': // -Je<encoding>
    begin
      S:=NormalizeEncoding(aValue);
      case S of
      {$IFDEF FPC_HAS_CPSTRING}
      'console','system',
      {$ENDIF}
      'utf8', 'json':
        if Log.Encoding<>S then begin
          Log.Encoding:=S;
          if FHasShownEncoding then begin
            FHasShownEncoding:=false;
            WriteEncoding;
          end;
        end;
      else ParamFatal('invalid encoding (-Je) "'+aValue+'"');
      end;
    end;
  'i': // -Ji<js-file>
    if aValue='' then
      ParamFatal('missing insertion file "'+aValue+'"')
    else if not Quick then
    begin
      if aValue='' then
        Result:=false
      else if aValue[length(aValue)]='-' then
      begin
        Delete(aValue,length(aValue),1);
        if aValue='' then
          Result:=False
        else
          RemoveInsertJSFilename(aValue);
      end else
        AddInsertJSFilename(aValue);
    end;
  'l': // -Jl
    SetOption(coLowercase,aValue<>'-');
  'm': // -Jm source map options
    if aValue='' then
      SrcMapEnable:=true
    else if aValue[1]='-' then
    begin
      if aValue<>'-' then
        Result:=False
      else
        SrcMapEnable:=false;
    end else
    begin
      case aValue of
      'include':
        SrcMapInclude:=true;
      'include-':
        SrcMapInclude:=false;
      'absolute':
        SrcMapFilenamesAbsolute:=true;
      'absolute-':
        SrcMapFilenamesAbsolute:=false;
      'xssiheader':
        SrcMapXSSIHeader:=true;
      'xssiheader-':
        SrcMapXSSIHeader:=false;
      else
        begin
        i:=Pos('=',aValue);
        if i<1 then
          ParamFatal('unknown -Jm parameter "'+aValue+'"')
        else
          begin
          S:=LeftStr(aValue,i-1);
          Delete(aValue,1,i);
          Case s of
            'sourceroot': SrcMapSourceRoot:=aValue;
            'basedir': SrcMapBaseDir:=aValue;
          else
            ParamFatal('unknown -Jm parameter "'+s+'"')
          end;
          end;
        end;
      end;
      // enable source maps when setting any -Jm<x> option
      SrcMapEnable:=true;
    end;
  'o': // -Jo<flag>
    begin
      S:=aValue;
      if aValue='' then
        ParamFatal('missing value of -Jo option');
      if SameText(LeftStr(S,4),'rtl-') then
      begin
        // -Jortl-<name>=<value>   set rtl identifier
        i:=5;
        while (i<=length(S)) and (S[i] in ['a'..'z','A'..'Z','0'..'9','_']) do
          inc(i);
        if (i>length(S)) or (S[i]<>'=') then
          ParamFatal('expected -Jortl-name=value');
        aName:='pbi'+copy(S,5,i-5);
        S:=copy(S,i+1,255);
        val(aName,pbi,i);
        if i<>0 then
          ParamFatal('unknown rtl identifier "'+aName+'"');
        if IsValidJSIdentifier(TJSString(ConverterGlobals.BuiltInNames[pbi]))
            and not IsValidJSIdentifier(TJSString(S)) then
          ParamFatal('JavaScript identifier expected');
        if not Quick then
          ConverterGlobals.BuiltInNames[pbi]:=S;
      end else begin
        Enable:=true;
        c:=S[length(S)];
        if c in ['+','-'] then
        begin
          Enable:=c='+';
          Delete(S,length(S),1);
        end;
        Case lowercase(S) of
          'searchlikefpc': FS.SearchLikeFPC:=Enable;
          'usestrict': SetOption(coUseStrict,Enable);
          'checkversion=main': RTLVersionCheck:=rvcMain;
          'checkversion=system': RTLVersionCheck:=rvcSystem;
          'checkversion=unit': RTLVersionCheck:=rvcUnit;
        else
          Result:=False;
        end;
      end;
    end;
  'p': // -Jp<...>
    begin
    if not Assigned(PostProcessorSupport) then
      ParamFatal('-Jp: No postprocessor support available');
    Result:=copy(aValue,1,3)='cmd';
    if Result then
      begin
      delete(aValue,1,3);
      if not Quick then
        PostProcessorSupport.AddPostProcessor(aValue);
      end;
    end;
  'r': // -Jr<...>
    begin
    S:=aValue;
    if aValue='' then
      ParamFatal('missing value for -Jr option')
    else if (S='none') then
      FResourceStringFile:=rsfNone
    else if (S='unit') then
      FResourceStringFile:=rsfunit
    else if (S='program') then
      FResourceStringFile:=rsfProgram
    else
      ParamFatal('invalid resource string file format (-Jr) "'+aValue+'"');
    end;
  'R': // -JR<...>
    begin
    I:=Pos('=',aValue);
    if I=0 then
      I:=Length(aValue)+1;
    S:=Copy(aValue,1,I-1);
    if S='' then
      ParamFatal('missing value for -JR option')
    else if (S='none') then
      FResourceMode:=rmNone
    else if (S='js') then
      FResourceMode:= rmJS
    else if (S='html') then
      begin
      FResourceMode:=rmHTML;
      S:=Copy(aValue,I+1,Length(aValue)-I);
      FResourceOutputFile:=S;
      if (FResourceOutputFile<>'') and (ExtractFileExt(FResourceOutputFile)='') then
        FResourceOutputFile:=FResourceOutputFile+'.html';
      end;
    end;
  'u': // -Ju<foreign path>
    if not Quick then
      begin
      ErrorMsg:=FS.AddForeignUnitPath(aValue,FromCmdLine);
      if ErrorMsg<>'' then
        ParamFatal('invalid foreign unit path (-Ju) "'+ErrorMsg+'"');
      end;
  'U': // -JU...
    HandleOptionPCUFormat(aValue);
  else
    Result:=False;
  end;
end;

procedure TPas2jsCompiler.HandleOptionConfigFile(aPos: Integer; const aFileName: string);

Var
  FN: String;

begin
  // load extra config file
  if aFilename='' then
    ParamFatal('invalid config file at param position '+IntToStr(aPos));
  FN:=ExpandFileName(aFilename);
  if not FS.FileExists(FN) then
    ParamFatal('config file not found: "'+aFileName+'"');
  LoadConfig(FN);
end;

procedure TPas2jsCompiler.HandleOptionInfo(aValue: string);

Var
  InfoMsg: String;

  procedure AppendInfo(Add: string);
  begin
    if InfoMsg<>'' then
      InfoMsg:=InfoMsg+' ';
    InfoMsg:=InfoMsg+Add;
  end;

Var
  P,L: integer;
  C,c2: Char;
  pr: TPasToJsProcessor;
  pl: TPasToJsPlatform;
  s: string;
  pbi: TPas2JSBuiltInName;
begin
  // write information and halt
  InfoMsg:='';
  if aValue='' then
    begin
    WriteInfo;
    Terminate(0);
    exit;
    end;
  P:=1;
  L:=Length(aValue);
  while p<=l do
  begin
    C:=aValue[P];
    case C of
    'D': // wite compiler date
      AppendInfo(GetCompiledDate);
    'V': // write short version
      AppendInfo(GetVersion(true));
    'W': // write long version
      AppendInfo(GetVersion(false));
    'S':
      begin
        inc(p);
        if p>l then
          ParamFatal('missing info option after S in "'+aValue+'".');
        C2:=aValue[p];
        case C2 of
        'O': // write source OS
          AppendInfo(GetCompiledTargetOS);
        'P': // write source processor
          AppendInfo(GetCompiledTargetCPU);
        else
          ParamFatal('unknown info option S"'+C2+'" in "'+aValue+'".');
        end;
      end;
    'T':
      begin
        inc(p);
        if p>l then
          ParamFatal('missing info option after T in "'+aValue+'".');
        C2:=aValue[p];
        case C2 of
        'O': // write target platform
          AppendInfo(PasToJsPlatformNames[TargetPlatform]);
        'P': // write target processor
          AppendInfo(PasToJsProcessorNames[TargetProcessor]);
        else
          ParamFatal('unknown info option S"'+C2+'" in "'+aValue+'".');
        end;
      end;
    'c':
      // write list of supported JS processors
      for pr in TPasToJsProcessor do
        Log.LogPlain(PasToJsProcessorNames[pr]);
    'o':
      begin
        // write list of optimizations
        Log.LogPlain('EnumNumbers');
        Log.LogPlain('RemoveNotUsedPrivates');
        Log.LogPlain('RemoveNotUsedDeclarations');
      end;
    't':
      // write list of supported targets
      for pl in TPasToJsPlatform do
        Log.LogPlain(PasToJsPlatformNames[pl]);
    'J':
      // write list of RTL identifiers
      begin
        Log.LogPlain('-JoRTL-<x> identifiers:');
        for pbi in TPas2JSBuiltInName do
        begin
          str(pbi,s);
          Delete(s,1,3);
          Log.LogPlain('-JoRTL-'+s+'='+Pas2JSBuiltInNames[pbi]);
        end;
      end
    else
      ParamFatal('unknown info option "'+C+'" in "'+aValue+'".');
    end;
    inc(p);
  end;
  if InfoMsg<>'' then
    Log.LogPlain(InfoMsg);
end;

procedure TPas2jsCompiler.ReadParam(Param: string; Quick, FromCmdLine: boolean);

  procedure UnknownParam;
  begin
    ParamFatal('unknown parameter "'+Param+'". Use -h for help.');
  end;

var
  EnabledFlags, DisabledFlags, Identifier, aValue: string;
  p, l, i: Integer;
  c: Char;
  aProc: TPasToJsProcessor;
  aPlatform: TPasToJsPlatform;

begin
  //writeln('TPas2jsCompiler.ReadParam ',Param,' ',Quick,' ',FromCmdLine);
  if ShowDebug then
    if Quick then
      Log.LogMsgIgnoreFilter(nQuickHandlingOption,[QuoteStr(Param)])
    else
      Log.LogMsgIgnoreFilter(nHandlingOption,[QuoteStr(Param)]);
  if Param='' then exit;
  FCurParam:=Param;
  ParamMacros.Substitute(Param,Self);
  if Param='' then exit;

  if Quick and ((Param='-h') or (Param='-?') or (Param='--help')) then
  begin
    WriteHelp;
    Terminate(0);
  end;

  l:=length(Param);
  p:=1;
  case Param[p] of
  '-':
    begin
      inc(p);
      if p>l then
        UnknownParam;
      aValue:=Copy(Param,P+1,Length(Param));
      case Param[p] of
      'i':
        begin
        HandleOptionInfo(aValue);
        Terminate(0);
        end;
      'B','l','n':
        begin
          ReadSingleLetterOptions(Param,p,'Bln',EnabledFlags,DisabledFlags);
          for i:=1 to length(EnabledFlags) do begin
            case EnabledFlags[i] of
            'B': Options:=Options+[coBuildAll];
            'l': ShowLogo:=true;
            'n': SkipDefaultConfig:=true;
            end;
          end;
          for i:=1 to length(DisabledFlags) do begin
            case DisabledFlags[i] of
            'B': Options:=Options-[coBuildAll];
            'l': ShowLogo:=false;
            'n': SkipDefaultConfig:=false;
            end;
          end;
        end;
      'C': // code generation
        ReadCodeGenerationFlags(aValue,1);
      'd': // define
        if not Quick then
        begin
          Identifier:=aValue;
          i:=Pos(':=',Identifier);
          if i>0 then
          begin
            aValue:=copy(Identifier,i+2,length(Identifier));
            Identifier:=LeftStr(Identifier,i-1);
            if not IsValidIdent(Identifier) then
              ParamFatal('invalid define name (-d): "'+Param+'"');
            AddDefine(Identifier,aValue);
          end else begin
            if not IsValidIdent(Identifier) then
              ParamFatal('invalid define (-d): "'+Param+'"');
            AddDefine(Identifier);
          end;
        end;
      'F': // folders and search paths
        begin
          if aValue='' then
            UnknownParam;
          c:=aValue[1];
          Delete(aValue,1,1);
          if not HandleOptionPaths(c,aValue,fromCmdLine) then
            UnknownParam;
        end;
      'I': // include path, same as -Fi
        if not Quick then
        begin
          if not HandleOptionPaths('i',aValue,fromCmdLine) then
            UnknownParam;
        end;
      'J': // extra pas2js options
        begin
          if aValue='' then
            UnknownParam;
          c:=aValue[1];
          Delete(aValue,1,1);
          if not HandleOptionJ(c,aValue,Quick,FromCmdLine) then
            UnknownParam;
        end;
      'M': // syntax mode
        begin
          case lowerCase(aValue) of
            'delphi': Mode:=p2jmDelphi;
            'objfpc': Mode:=p2jmObjFPC;
          else
            ParamFatal('invalid syntax mode  (-M) "'+aValue+'"');
          end;
        end;
      'N':
        begin
          if aValue='' then
            UnknownParam;
          case aValue[1] of
          'S':
            begin
            Log.Log(mtWarning,'obsolete option -NS, use -FN instead');
            Delete(aValue,1,1);
            HandleOptionPaths('N',aValue,FromCmdLine);
            end;
          else UnknownParam;
          end;
        end;
      'o': // output file, main JavaScript file
        begin
          if aValue='' then
            ParamFatal('invalid empty output file (-o)')
          else if aValue='..' then
            ParamFatal('invalid output file (-o) "'+aValue+'"')
          else if aValue='.' then
            // ok, stdout
          else
            aValue:=ExpandFileName(aValue);
          MainJSFile:=aValue;
        end;
      'O': // optimizations
        begin
        if aValue='' then
          UnknownParam;
        C:=aValue[1];
        Delete(aValue,1,1);
        if not HandleOptionOptimization(C,aValue) then
          UnknownParam;
        end;
      'P': // target processor
        begin
        for aProc in TPasToJsProcessor do
          if SameText(aValue,PasToJsProcessorNames[aProc]) then
            begin
            TargetProcessor:=aProc;
            aValue:='';
            break;
            end;
        if aValue<>'' then
          ParamFatal('invalid target processor (-P) "'+aValue+'"');
        end;
      'S': // Syntax
        begin
          inc(p);
          if (p<=l) and (Param[p]='I') then
            begin
            Identifier:=copy(Param,p,length(Param));
            if SameText(Identifier,'com') then
              InterfaceType:=citCom
            else if SameText(Identifier,'corba') then
              InterfaceType:=citCorba
            else
              ParamFatal('invalid interface style (-SI) "'+Identifier+'"');
            end
          else
            ReadSyntaxFlags(Param,p);
        end;
      'T': // target platform
        begin
        inc(p);
        Identifier:=copy(Param,p,length(Param));
        for aPlatform in TPasToJsPlatform do
          if SameText(Identifier,PasToJsPlatformNames[aPlatform]) then
            begin
            TargetPlatform:=aPlatform;
            Identifier:='';
            break;
            end;
        if Identifier<>'' then
          ParamFatal('invalid target platform (-T) "'+Identifier+'"');
        end;
      'u': // undefine
        if not Quick then
        begin
          if not IsValidIdent(aValue) then
            ParamFatal('invalid undefine (-u): "'+aValue+'"');
          RemoveDefine(aValue);
        end;
      'v': // verbose
        begin
          inc(p);
          ReadVerbosityFlags(Param,p);
        end;
      else
        UnknownParam;
      end;
    end;
  '@':
    if not Quick then
      HandleOptionConfigFile(i,copy(Param,2,length(Param)));
  else
    // filename
    if (not Quick) then
    begin
      if not FromCmdLine then
        ConfigSupport.CfgSyntaxError('invalid parameter');
      if MainSrcFile<>'' then
        ParamFatal('Only one Pascal file is supported, but got "'+MainSrcFile+'" and "'+Param+'".');
      MainSrcFile:=ExpandFileName(Param);
    end;
  end;
end;

procedure TPas2jsCompiler.ReadSingleLetterOptions(const Param: string;
  p: integer; const Allowed: string; out Enabled, Disabled: string);
// e.g. 'B' 'lB' 'l-' 'l+B-'
var
  Letter: Char;
  i, l: Integer;
begin
  l:=length(Param);
  if p>l then
    ParamFatal('Invalid option "'+Param+'"');
  Enabled:='';
  Disabled:='';
  while p<=l do
  begin
    Letter:=Param[p];
    if Letter='-' then
      ParamFatal('Invalid option "'+Param+'"');
    if Pos(Letter,Allowed)<1 then
      ParamFatal('unknown option "'+Param+'". Use -h for help.');
    inc(p);
    if (p<=l) and (Param[p]='-') then
    begin
      // disable
      if Pos(Letter,Disabled)<1 then Disabled+=Letter;
      i:=Pos(Letter,Enabled);
      if i>0 then Delete(Enabled,i,1);
      inc(p);
    end else begin
      // enable
      if Pos(Letter,Enabled)<1 then Enabled+=Letter;
      i:=Pos(Letter,Disabled);
      if i>0 then Delete(Disabled,i,1);
      if (p<=l) and (Param[p]='+') then inc(p);
    end;
  end;
end;

procedure TPas2jsCompiler.ReadCodeGenerationFlags(Param: String; p: integer);
var
  Enabled, Disabled: string;
  i: Integer;
begin
  ReadSingleLetterOptions(Param,p,'orR',Enabled,Disabled);
  for i:=1 to length(Enabled) do begin
    case Enabled[i] of
    'o': Options:=Options+[coOverflowChecks];
    'r': Options:=Options+[coRangeChecks];
    'R': Options:=Options+[coObjectChecks];
    end;
  end;
  for i:=1 to length(Disabled) do begin
    case Disabled[i] of
    'o': Options:=Options-[coOverflowChecks];
    'r': Options:=Options-[coRangeChecks];
    'R': Options:=Options-[coObjectChecks];
    end;
  end;
end;

procedure TPas2jsCompiler.ReadSyntaxFlags(Param: String; p: integer);
var
  Enabled, Disabled: string;
  i: Integer;
begin
  ReadSingleLetterOptions(Param,p,'2acdm',Enabled,Disabled);
  for i:=1 to length(Enabled) do begin
    case Enabled[i] of
    '2': Mode:=p2jmObjFPC;
    'a': Options:=Options+[coAssertions];
    'c': Options:=Options+[coAllowCAssignments];
    'd': Mode:=p2jmDelphi;
    'm': Options:=Options+[coAllowMacros];
    end;
  end;
  for i:=1 to length(Disabled) do begin
    case Disabled[i] of
    '2': ;
    'a': Options:=Options-[coAssertions];
    'c': Options:=Options-[coAllowCAssignments];
    'd': ;
    'm': Options:=Options-[coAllowMacros];
    end;
  end;
end;

procedure TPas2jsCompiler.ReadVerbosityFlags(Param: String; p: integer);
var
  Enabled, Disabled: string;
  i, l: Integer;
begin
  l:=length(Param);
  if p>l then exit;

  if Param[p]='m' then
  begin
    // read m-flags
    repeat
      inc(p);
      if (p>l) or not (Param[p] in ['0'..'9']) then
        ParamFatal('missing number in "'+Param+'"');
      i:=0;
      while (p<=l) and (Param[p] in ['0'..'9']) do
      begin
        i:=i*10+ord(Param[p])-ord('0');
        if i>99999 then
          ParamFatal('Invalid -vm parameter in "'+Param+'"');
        inc(p);
      end;
      if (p<=l) and (Param[p]='-') then
      begin
        inc(p);
        Log.MsgNumberDisabled[i]:=false;
      end else
        Log.MsgNumberDisabled[i]:=true;
      if p>l then break;
      if Param[p]<>',' then
        ParamFatal('Invalid option "'+Param+'"');
    until false;
    exit;
  end;

  // read other flags
  ReadSingleLetterOptions(Param,p,'ewnhila0bctdqxvz',Enabled,Disabled);
  for i:=1 to length(Enabled) do begin
    case Enabled[i] of
    'e': Options:=Options+[coShowErrors];
    'w': Options:=Options+[coShowWarnings];
    'n': Options:=Options+[coShowNotes];
    'h': Options:=Options+[coShowHints];
    'i': Options:=Options+[coShowInfos];
    'l': Options:=Options+[coShowLineNumbers];
    'a': Options:=Options+coShowAll;
    '0': Options:=Options-coShowAll+[coShowErrors];
    'b': ShowFullPaths:=true;
    'c': Options:=Options+[coShowConditionals,coShowInfos];
    't': ShowTriedUsedFiles:=true;
    'd': ShowDebug:=true;
    'q': Options:=Options+[coShowMessageNumbers];
    'x': Options:=Options+[coShowUsedTools];
    'v': Options:=Options+[coWriteDebugLog];
    'z': WriteMsgToStdErr:=true;
    end;
  end;
  for i:=1 to length(Disabled) do begin
    case Disabled[i] of
    'e': Options:=Options-[coShowErrors];
    'w': Options:=Options-[coShowWarnings];
    'n': Options:=Options-[coShowNotes];
    'h': Options:=Options-[coShowHints];
    'i': Options:=Options-[coShowInfos];
    'l': Options:=Options-[coShowLineNumbers];
    'a': ;
    '0': ;
    'b': ShowFullPaths:=false;
    'c': Options:=Options-[coShowConditionals];
    't': ShowTriedUsedFiles:=false;
    'd': ShowDebug:=false;
    'q': Options:=Options-[coShowMessageNumbers];
    'x': Options:=Options-[coShowUsedTools];
    'v': Options:=Options+[coWriteDebugLog];
    'z': WriteMsgToStdErr:=false;
    end;
  end;
end;

procedure TPas2jsCompiler.SetAllJSIntoMainJS(AValue: Boolean);
begin
  if FAllJSIntoMainJS=AValue then Exit;
  if aValue then
    FMainJSFileIsResolved:=False;
  FAllJSIntoMainJS:=AValue;
end;

procedure TPas2jsCompiler.SetConverterGlobals(
  const AValue: TPasToJSConverterGlobals);
begin
  if AValue=FConverterGlobals then exit;
  if (FConverterGlobals<>nil) and (FConverterGlobals.Owner=Self) then
    FreeAndNil(FConverterGlobals);
  FConverterGlobals:=AValue;
end;

function TPas2jsCompiler.FormatPath(const aPath: String): String;
begin
  Result:=FS.FormatPath(aPath);
end;

function TPas2jsCompiler.FullFormatPath(const aPath: String): String;
begin
  Result:=QuoteStr(FormatPath(aPath));
end;

function TPas2jsCompiler.CreateMacroEngine: TPas2jsMacroEngine;

begin
  Result:=TPas2jsMacroEngine.Create;
end;

function TPas2jsCompiler.CreateLog: TPas2jsLogger;

begin
  Result:=TPas2jsLogger.Create;
end;

constructor TPas2jsCompiler.Create;

begin
  FOptions:=DefaultP2jsCompilerOptions;
  FConverterGlobals:=TPasToJSConverterGlobals.Create(Self);
  FNamespaces:=TStringList.Create;
  FDefines:=TStringList.Create;
  FInsertFilenames:=TStringList.Create;
  FLog:=CreateLog;
  FLog.OnFormatPath:=@FormatPath;
  FParamMacros:=CreateMacroEngine;
  RegisterMessages;
  FS:=CreateFS;
  FOwnsFS:=true;

  // Done by Reset: TStringList(FDefines).Sorted:=True;
  // Done by Reset: TStringList(FDefines).Duplicates:=dupError;
  //FConditionEval.OnEvalFunction:=@ConditionEvalFunction;

  FFiles:=CreateSetOfCompilerFiles(kcFilename);
  FUnits:=CreateSetOfCompilerFiles(kcUnitName);
  FResourceMode:=DefaultResourceMode;
  FResourceStrings:=TResourceStringsFile.Create;
  FReadingModules:=TFPList.Create;
  InitParamMacros;
  Reset;
end;

destructor TPas2jsCompiler.Destroy;

  procedure FreeStuff;
  begin
    FreeAndNil(FResourceStrings);
    FreeAndNil(FNamespaces);
    FreeAndNil(FWPOAnalyzer);
    FreeAndNil(FInsertFilenames);

    FMainFile:=nil;
    FreeAndNil(FUnits);
    FreeAndNil(FReadingModules);
    FFiles.FreeItems;
    FreeAndNil(FFiles);

    FreeAndNil(FPostProcessorSupport);
    FreeAndNil(FConfigSupport);
    ConverterGlobals:=nil;

    ClearDefines;
    FreeAndNil(FDefines);

    FLog.OnFormatPath:=nil;
    if FOwnsFS then
      FreeAndNil(FFS)
    else
      FFS:=nil;

    FreeAndNil(FParamMacros);
  end;

begin
  if ShowDebug then
    try
      FreeStuff;
    except
      on E: Exception do
      begin
        Log.LogExceptionBackTrace(E);
      end
      {$IFDEF Pas2js}
      else HandleJSException('[20181031190818] TPas2jsCompiler.Destroy',JSExceptValue);
      {$ENDIF}
    end
  else
    FreeStuff;

  FreeAndNil(FLog);
  inherited Destroy;
end;

function TPas2jsCompiler.OnMacroCfgDir(Sender: TObject; var Params: string;
  Lvl: integer): boolean;
begin
  if Lvl=0 then ;
  if Sender=nil then ;
  Params:=ExtractFilePath(ConfigSupport.CurrentCfgFilename);
  Result:=true;
end;

procedure TPas2jsCompiler.AddDefine(const aName: String);
begin
  if FDefines.IndexOf(aName)>=0 then exit;
  FDefines.Add(aName);
end;

procedure TPas2jsCompiler.AddDefine(const aName, Value: String);
var
  Index: Integer;
  M: TMacroDef;
begin
  Index:=FDefines.IndexOf(aName);
  If (Index<0) then
    FDefines.AddObject(aName,TMacroDef.Create(aName,Value))
  else begin
    M:=TMacroDef(FDefines.Objects[Index]);
    if M=nil then
      FDefines.Objects[Index]:=TMacroDef.Create(aName,Value)
    else
      M.Value:=Value;
  end;
end;

procedure TPas2jsCompiler.RemoveDefine(const aName: String);
var
  i: Integer;
  M: TMacroDef;
begin
  i:=FDefines.IndexOf(aName);
  if (i<>-1) then
  begin
    M:=TMacroDef(FDefines.Objects[i]);
    M.Free;
    FDefines.Delete(i);
  end;
end;

function TPas2jsCompiler.IsDefined(const aName: String): boolean;
begin
  Result:=FDefines.IndexOf(aName)>=0;
end;

class function TPas2jsCompiler.GetVersion(ShortVersion: boolean): string;
begin
  Result:=IntToStr(VersionMajor)+'.'+IntToStr(VersionMinor)+'.'+IntToStr(VersionRelease);
  if not ShortVersion then
    Result+=VersionExtra;
end;

procedure TPas2jsCompiler.WritePrecompiledFormats;
begin
  WriteHelpLine('   -JU: This pas2js does not support PCU files');
end;

procedure TPas2jsCompiler.AddNamespaces(const Paths: string;
  FromCmdLine: boolean);

// cmd line paths are added in front of the cfg paths
// cmd line paths are added in order, cfg paths are added in reverse order
// multi paths separated by semicolon are added in order
// duplicates are removed
var
  Added: Integer;

  function Add(aPath: string): boolean;
  var
    Remove: Boolean;
    i: Integer;
  begin
    Remove:=false;
    // search duplicate
    if aPath[length(aPath)]='-' then
    begin
      Delete(aPath,length(aPath),1);
      Remove:=true;
    end;
    if not IsValidIdent(aPath,true,true) then
      exit(False);
    i:=Namespaces.Count-1;
    while (i>=0) and (CompareText(aPath,NameSpaces[i])<>0) do dec(i);

    if Remove then
    begin
      // remove
      if i>=0 then
      begin
        NameSpaces.Delete(i);
        if NamespacesFromCmdLine>i then dec(FNamespacesFromCmdLine);
      end;
      exit(true);
    end;

    if FromCmdLine then
    begin
      // from cmdline: append in order to the cmdline params, in front of cfg params
      if i>=0 then
      begin
        if i<=NamespacesFromCmdLine then exit(true);
        NameSpaces.Delete(i);
      end;
      NameSpaces.Insert(NamespacesFromCmdLine,aPath);
      inc(FNamespacesFromCmdLine);
    end else begin
      // from cfg: append in reverse order to the cfg params, behind cmdline params
      if i>=0 then
      begin
        if i<=FNamespacesFromCmdLine+Added then exit(true);
        NameSpaces.Delete(i);
      end;
      NameSpaces.Insert(FNamespacesFromCmdLine+Added,aPath);
      inc(Added);
    end;
    Result:=true;
  end;

var
  aPath: String;
  p: integer;

begin
  p:=1;
  Added:=0;
  while p<=length(Paths) do
    begin
    aPath:=GetNextDelimitedItem(Paths,';',p);
    if aPath='' then
      continue;
    if not Add(aPath) then
      exit;
    end;
end;

procedure TPas2jsCompiler.Reset;
begin
  FreeAndNil(FWPOAnalyzer);
  FPrecompileGUID:=default(TGUID);
  FNamespaces.Clear;
  FNamespacesFromCmdLine:=0;
  FMainFile:=nil;
  FUnits.Clear;
  FReadingModules.Clear;
  FFiles.FreeItems;
  FInsertFilenames.Clear;
  if Assigned(FPostProcessorSupport) then
    FPostProcessorSupport.Clear;
  FCompilerExe:='';
  FSrcMapBaseDir:='';
  FMainSrcFile:='';
  FOptions:=DefaultP2jsCompilerOptions;
  FRTLVersionCheck:=DefaultP2jsRTLVersionCheck;
  FMode:=p2jmObjFPC;
  FConverterGlobals.Reset;
  FConverterGlobals.RTLVersion:=(VersionMajor*100+VersionMinor)*100+VersionRelease;
  FConverterGlobals.TargetPlatform:=PlatformBrowser;
  FConverterGlobals.TargetProcessor:=ProcessorECMAScript5;
  FMainJSFileIsResolved:=False;
  Log.Reset;
  Log.ShowMsgTypes:=GetShownMsgTypes;

  ClearDefines;
  TStringList(FDefines).Sorted:=True;
  {$IFDEF FPC}
  TStringList(FDefines).Duplicates:=dupError;
  {$ENDIF}

  AddDefine('PAS2JS');
  AddDefine('PAS2JS_FULLVERSION',IntToStr((VersionMajor*100+VersionMinor)*100+VersionRelease));
  AddDefinesForTargetPlatform;
  AddDefinesForTargetProcessor;
  // add FPC compatibility flags
  AddDefine('FPC_HAS_FEATURE_CLASSES');
  AddDefine('FPC_HAS_FEATURE_INIT');
  AddDefine('FPC_HAS_FEATURE_DYNARRAYS');
  AddDefine('FPC_HAS_FEATURE_EXCEPTIONS');
  AddDefine('FPC_HAS_FEATURE_EXITCODE');
  AddDefine('FPC_HAS_FEATURE_INITFINAL');
  AddDefine('FPC_HAS_FEATURE_RTTI');
  AddDefine('FPC_HAS_FEATURE_SUPPORT');
  AddDefine('FPC_HAS_FEATURE_UNICODESTRINGS');
  AddDefine('FPC_HAS_FEATURE_WIDESTRINGS');
  AddDefine('FPC_HAS_TYPE_DOUBLE');
  AddDefine('FPC_HAS_UNICODESTRING');
  AddDefine('FPC_UNICODESTRINGS');
  AddDefine('FPC_WIDESTRING_EQUAL_UNICODESTRING');
  AddDefine('STR_CONCAT_PROCS');
  AddDefine('UNICODE');

  FHasShownLogo:=false;
  FHasShownEncoding:=false;
  FFS.Reset;
end;

procedure TPas2jsCompiler.Run(aCompilerExe: string; aWorkingDir: string;
  ParamList: TStrings; DoReset: boolean);
var
  i: Integer;
  StartTime: TDateTime;
begin
  StartTime:=Now;

  if DoReset then Reset;
  if FileCount>0 then
    RaiseInternalError(20170504161340,'internal error: TPas2jsCompiler.Run FileCount>0');

  try
    // set working directory, needed by all relative filenames
    SetWorkingDir(aWorkingDir);

    CompilerExe:=aCompilerExe; // maybe needed to find the default config

    // quick check command line params
    for i:=0 to ParamList.Count-1 do
      ReadParam(ParamList[i],true,true);
    if WriteDebugLog then
      Log.OpenDebugLog;
    if ShowLogo then
      WriteLogo;

    // read default config
    if Assigned(ConfigSupport) and not SkipDefaultConfig then
      ConfigSupport.LoadDefaultConfig;

    // read env PAS2JS_OPTS
    ReadEnvironment;

    // read command line parameters
    for i:=0 to ParamList.Count-1 do
      ReadParam(ParamList[i],false,true);

    // At this point we know what kind of resources we must use
    CreateResourceSupport;

    // now we know, if the logo can be displayed
    if ShowLogo then
      WriteLogo;

    // show debug info
    if ShowDebug then
    begin
      WriteOptions;
      WriteDefines;
    end;
    if ShowDebug or ShowUsedTools then
      WriteUsedTools;
    if ShowDebug or ShowTriedUsedFiles then
      WriteFoldersAndSearchPaths;

    if MainSrcFile='' then
      ParamFatal('No source file name in command line');
    if not FS.FileExists(MainSrcFile) then
      ParamFatal('Pascal file not found: "'+MainSrcFile+'"');

    // compile
    Compile(StartTime);
  except
    on E: ECompilerTerminate do
    begin
    end;
    on E: Exception do begin
      if ShowDebug then
        Log.LogExceptionBackTrace(E);
      raise; // reraise unexpected exception
    end else begin
      if ShowDebug then
        Log.LogExceptionBackTrace(nil);
      {$IFDEF Pas2js}
      HandleJSException('[20181031190933] TPas2jsCompiler.Run',JSExceptValue,false);
      {$ENDIF}
      raise; // reraise unexpected exception
    end;
  end;
end;

procedure TPas2jsCompiler.WriteHelpLine(S: String);


const
  MaxLineLen = 78;
  Indent = 12;


var
  l, p, LastCharStart, WordBreak: integer;
  Len: integer;
  CodePointCount: Integer;

  procedure InitLine;
  begin
    l:=length(s);
    p:=1;
    LastCharStart:=p;
    WordBreak:=0;
    CodePointCount:=0;
  end;

begin
  if length(s)<=MaxLineLen then
  begin
    Log.LogRaw(s);
    exit;
  end;
  InitLine;
  while p<=l do
  begin
    case s[p] of
    'a'..'z','A'..'Z','0'..'9','_','-','.',',','"','''','`',
    #128..high(char) :
      begin
      LastCharStart:=p;
      {$IFDEF FPC_HAS_CPSTRING}
      Len:=UTF8CharacterStrictLength(@s[p]);
      if Len=0 then Len:=1;
      inc(p,Len);
      {$ELSE}
      if (p<l) and (s[p] in [#$DC00..#$DFFF]) then
        inc(p,2)
      else
        inc(p,1);
      {$ENDIF}
      end;
    else
      LastCharStart:=p;
      WordBreak:=p;
      inc(p);
    end;
    inc(CodePointCount);
    if CodePointCount>=MaxLineLen then
    begin
      if (WordBreak=0)
          or (WordBreak<MaxLineLen div {$IFDEF FPC_HAS_CPSTRING}3{$ELSE}2{$ENDIF}) then
        WordBreak:=LastCharStart;
      Len:=WordBreak-1;
      Log.LogRaw(LeftStr(s,Len));
      Delete(s,1,len);
      s:=StringOfChar(' ',Indent)+Trim(s);
      InitLine;
    end;
  end;
  Log.LogRaw(s);
end;

procedure TPas2jsCompiler.WriteHelp;

  procedure w(s: string); inline;

  begin
    WriteHelpLine(S);
  end;

var
  i: Integer;
  ParamMacro: TPas2jsMacro;
begin
  WriteLogo;
  Log.LogLn;
  if CompilerExe<>'' then
  begin
    w('Usage: '+CompilerExe+' <your.pas>');
  end else begin
    w('Usage: pas2js <your.pas>');
  end;
  Log.LogLn;
  w('Options:');
  w('Put + after a boolean switch option to enable it, - to disable it');
  w('  @<x>   : Read compiler options from file <x> in addition to the default '+DefaultConfigFile);
  w('  -B     : Rebuild all');
  w('  -d<x>  : Defines the symbol <x>. Optional: -d<x>:=<value>');
  w('  -i<x>  : Write information and halt. <x> is a combination of the following:');
  w('    -iD  : Write compiler date');
  w('    -iSO : Write compiler OS');
  w('    -iSP : Write compiler host processor');
  w('    -iTO : Write target platform');
  w('    -iTP : Write target processor');
  w('    -iV  : Write short compiler version');
  w('    -iW  : Write full compiler version');
  w('    -ic  : Write list of supported JS processors usable by -P<x>');
  w('    -io  : Write list of supported optimizations usable by -Oo<x>');
  w('    -it  : Write list of supported targets usable by -T<x>');
  w('    -iJ  : Write list of supported JavaScript identifiers -JoRTL-<x>');
  w('  -C<x>  : Code generation options. <x> is a combination of the following letters:');
  // -C3        Turn on ieee error checking for constants
  w('    o    : Overflow checking of integer operations');
  // -CO        Check for possible overflow of integer operations
  w('    r    : Range checking');
  w('    R    : Object checks. Verify method calls and object type casts.');
  w('  -F...   Set file names and paths:');
  w('   -Fe<x>: Redirect output to file <x>. UTF-8 encoded.');
  w('   -FE<x>: Set main output path to <x>');
  w('   -Fi<x>: Add <x> to include paths');
  w('   -FN<x>: add <x> to namespaces. Namespaces with trailing - are removed.');
  w('            Delphi calls this flag "unit scope names".');
  //w('   -Fr<x>: Load error message file <x>');
  w('   -Fu<x>: Add <x> to unit paths');
  w('   -FU<x>: Set unit output path to <x>');
  w('  -I<x>  : Add <x> to include paths, same as -Fi');
  w('  -J...  Extra options of pas2js');
  w('   -Jc   : Write all JavaScript concatenated into the output file');
  w('   -Je<x>: Encode messages as <x>.');
  w('     -Jeconsole: Console codepage. This is the default.');
  w('     -Jesystem : System codepage. On non Windows console and system are the same.');
  w('     -Jeutf-8  : Unicode UTF-8. Default when using -Fe.');
  w('     -JeJSON   : Output compiler messages as JSON. Logo etc are outputted as-is.');
  w('   -Ji<x>: Insert JS file <x> into main JS file. E.g. -Jirtl.js. Can be given multiple times. To remove a file name append a minus, e.g. -Jirtl.js-.');
  w('   -Jl   : lower case identifiers');
  w('   -Jm   : generate source maps');
  w('     -Jmsourceroot=<x>: use x as "sourceRoot", prefix URL for source file names.');
  w('     -Jmbasedir=<x>: write source file names relative to directory x, default is map file folder.');
  w('     -Jminclude: include Pascal sources in source map.');
  w('     -Jmabsolute: store absolute filenames, not relative.');
  w('     -Jmxssiheader: start source map with XSSI protection )]}'', default.');
  w('     -Jm-: disable generating source maps');
  w('   -Jo<x>: Enable or disable extra option. The x is case insensitive:');
  w('     -JoSearchLikeFPC: search source files like FPC, default: search case insensitive.');
  w('     -JoUseStrict: add "use strict" to modules, default.');
  w('     -JoCheckVersion-: do not add rtl version check, default.');
  w('     -JoCheckVersion=main: insert rtl version check into main.');
  w('     -JoCheckVersion=system: insert rtl version check into system unit init.');
  w('     -JoCheckVersion=unit: insert rtl version check into every unit init.');
  w('     -JoRTL-<y>=<z>: set RTL identifier y to value z. See -iJ.');
  w('   -Jr<x> Control writing of resource string file');
  w('     -Jrnone: Do not write resource string file');
  w('     -Jrunit: Write resource string file per unit with all resource strings');
  w('     -Jrprogram: Write resource string file per program with all used resource strings in program');
  w('   -Jr<x> Control writing of linked resources');
  w('     -JRnone: Do not write resources');
  w('     -JRjs: Write resources in Javascript structure');
  w('     -JRhtml[=filename] : Write resources as preload links in HTML file (default is projectfile-res.html)');
  w('   -Jpcmd<command>: Run postprocessor. For each generated js execute command passing the js as stdin and read the new js from stdout. This option can be added multiple times to call several postprocessors in succession.');
  w('   -Ju<x>: Add <x> to foreign unit paths. Foreign units are not compiled.');
  WritePrecompiledFormats;
  w('  -l     : Write logo');
  w('  -MDelphi: Delphi 7 compatibility mode');
  w('  -MObjFPC: FPC''s Object Pascal compatibility mode (default)');
  w('  -NS<x> : obsolete: add <x> to namespaces. Same as -FN<x>');
  w('  -n     : Do not read the default config files');
  w('  -o<x>  : Change main JavaScript file to <x>, "." means stdout');
  w('  -O<x>  : Optimizations:');
  w('    -O-  : Disable optimizations');
  w('    -O1  : Level 1 optimizations (quick and debugger friendly)');
  //w('    -O2  : Level 2 optimizations (Level 1 + not debugger friendly)');
  w('    -Oo<x>: Enable or disable optimization. The x is case insensitive:');
  w('      -OoEnumNumbers[-]: write enum value as number instead of name. Default in -O1.');
  w('      -OoRemoveNotUsedPrivates[-]: Default is enabled');
  w('      -OoRemoveNotUsedDeclarations[-]: Default enabled for programs with -Jc');
  w('  -P<x>  : Set target processor. Case insensitive:');
  w('    -Pecmascript5: default');
  w('    -Pecmascript6');
  w('  -S<x>  : Syntax options. <x> is a combination of the following letters:');
  w('    a    : Turn on assertions');
  w('    c    : Support operators like C (*=,+=,/= and -=)');
  w('    d    : Same as -Mdelphi');
  w('    m    : Enables macro replacements');
  w('    2    : Same as -Mobjfpc (default)');
  w('  -SI<x>  : Set interface style to <x>');
  w('    -SIcom  : COM, reference counted interface (default)');
  w('    -SIcorba: CORBA interface');
  w('  -T<x>  : Set target platform');
  w('    -Tbrowser: default');
  w('    -Tnodejs : add pas.run(), includes -Jc');
  w('  -u<x>  : Undefines the symbol <x>');
  w('  -v<x>  : Be verbose. <x> is a combination of the following letters:');
  w('    e    : Show errors (default)');
  w('    w    : Show warnings');
  w('    n    : Show notes');
  w('    h    : Show hints');
  w('    i    : Show info');
  w('    l    : Show line numbers, needs -vi');
  w('    a    : Show everything');
  w('    0    : Show nothing (except errors)');
  w('    b    : Show file names with full path');
  w('    c    : Show conditionals');
  w('    t    : Show tried/used files');
  w('    d    : Show debug notes and info, enables -vni');
  w('    q    : Show message numbers');
  w('    x    : Show used tools');
  w('    v    : Write pas2jsdebug.log with lots of debugging info');
  w('    z    : Write messages to stderr, -o. still uses stdout.');
  w('  -vm<x>,<y>: Do not show messages numbered <x> and <y>.');
  w('  -?     : Show this help');
  w('  -h     : Show this help');
  Log.LogLn;
  w('Environment variable PAS2JS_OPTS is parsed after default config and before command line parameters.');
  Log.LogLn;
  w('Macros: Format is $Name, $Name$ or $Name()');
  for i:=0 to ParamMacros.Count-1 do begin
    ParamMacro:=ParamMacros[i];
    Log.LogRaw(['  $',ParamMacro.Name,BoolToStr(ParamMacro.CanHaveParams,'()',''),': ',ParamMacro.Description]);
  end;
end;

procedure TPas2jsCompiler.WriteLogo;
begin
  if FHasShownLogo then exit;
  FHasShownLogo:=true;
  WriteVersionLine;
  Log.LogPlain('Copyright (c) 2019 Free Pascal team.');
  if coShowInfos in Options then
    WriteEncoding;
end;

procedure TPas2jsCompiler.WriteEncoding;
begin
  if FHasShownEncoding then exit;
  FHasShownEncoding:=true;
  Log.LogMsgIgnoreFilter(nMessageEncodingIs,[Log.GetEncodingCaption]);
end;

procedure TPas2jsCompiler.WriteVersionLine;
var
  s: String;
begin
  s:='Pas2JS Compiler version '+GetVersion(false);
  s:=s+' ['+{$i %Date%}+'] for '+{$i %FPCTargetOS%}+' '+{$i %FPCTargetCPU%};
  Log.LogPlain(s);
  if coShowInfos in Options then
    WriteEncoding;
end;

procedure TPas2jsCompiler.WriteOptions;
var
  co: TP2jsCompilerOption;
  fco: TP2jsFSOption;
begin
  // message encoding
  WriteEncoding;
  // target platform
  Log.LogMsgIgnoreFilter(nTargetPlatformIs,[PasToJsPlatformNames[TargetPlatform]]);
  Log.LogMsgIgnoreFilter(nTargetProcessorIs,[PasToJsProcessorNames[TargetProcessor]]);
  // default syntax mode
  Log.LogMsgIgnoreFilter(nSyntaxModeIs,[p2jscModeNames[Mode]]);
  Log.LogMsgIgnoreFilter(nClassInterfaceStyleIs,[InterfaceTypeNames[InterfaceType]]);
  // boolean options
  for co in TP2jsCompilerOption do
    Log.LogMsgIgnoreFilter(nOptionIsEnabled,
      [p2jscoCaption[co],BoolToStr(co in Options,'enabled','disabled')]);
  for fco in TP2jsFSOption do
    Log.LogMsgIgnoreFilter(nOptionIsEnabled,
      [p2jsfcoCaption[fco],BoolToStr(fco in FS.Options,'enabled','disabled')]);

  // source map options
  if SrcMapEnable then
  begin
    Log.LogMsgIgnoreFilter(nSrcMapSourceRootIs,[QuoteStr(SrcMapSourceRoot)]);
    Log.LogMsgIgnoreFilter(nSrcMapBaseDirIs,[QuoteStr(SrcMapBaseDir)]);
  end;
end;

procedure TPas2jsCompiler.WriteDefines;
var
  i: Integer;
  S: String;
  M: TMacroDef;
  pbi: TPas2JSBuiltInName;
begin
  for i:=0 to Defines.Count-1 do
    begin
    S:=Defines[i];
    M:=TMacroDef(Defines.Objects[i]);
    if M<>nil then
      Log.Log(mtInfo,SafeFormat(SLogMacroXSetToY,[S,QuoteStr(M.Value)]),nLogMacroXSetToY,'',0,0,false)
    else
      Log.Log(mtInfo,SafeFormat(SLogMacroDefined,[S]),nLogMacroDefined,'',0,0,false)
    end;
  for pbi in TPas2JSBuiltInName do
    if Pas2JSBuiltInNames[pbi]<>ConverterGlobals.BuiltInNames[pbi] then
    begin
      WriteStr(S,pbi);
      S:=copy(S,4,255);
      Log.LogMsgIgnoreFilter(nRTLIdentifierChanged,[QuoteStr(S),
        QuoteStr(Pas2JSBuiltInNames[pbi]),QuoteStr(ConverterGlobals.BuiltInNames[pbi])]);
    end;
end;

procedure TPas2jsCompiler.WriteUsedTools;
begin
  if Assigned(FPostProcessorSupport) then
    FPostProcessorSupport.WriteUsedTools;
end;

procedure TPas2jsCompiler.WriteFoldersAndSearchPaths;
var
  I: integer;
begin
  Log.LogMsgIgnoreFilter(nNameValue,['Compiler exe',QuoteStr(CompilerExe)]);
  FS.WriteFoldersAndSearchPaths;
  for i:=0 to Namespaces.Count-1 do
    Log.LogMsgIgnoreFilter(nUsingPath,['Unit scope',Namespaces[i]]);
  Log.LogMsgIgnoreFilter(nNameValue,['Output file',QuoteStr(MainJSFile)]);
end;

procedure TPas2jsCompiler.WriteInfo;
begin
  WriteVersionLine;
  Log.LogLn;
  Log.LogPlain('Compiler date      : '+GetCompiledDate);
  Log.LogPlain('Compiler CPU target: '+GetCompiledTargetCPU);
  Log.LogLn;
  Log.LogPlain('Supported targets (targets marked with ''{*}'' are under development):');
  Log.LogPlain(['  ',PasToJsPlatformNames[PlatformBrowser],': webbrowser']);
  Log.LogPlain(['  ',PasToJsPlatformNames[PlatformNodeJS],': Node.js']);
  Log.LogLn;
  Log.LogPlain('Supported CPU instruction sets:');
  Log.LogPlain('  ECMAScript5, ECMAScript6');
  Log.LogLn;
  Log.LogPlain('Recognized compiler and RTL features:');
  Log.LogPlain('  RTTI,CLASSES,EXCEPTIONS,EXITCODE,RANDOM,DYNARRAYS,COMMANDARGS,');
  Log.LogPlain('  UNICODESTRINGS');
  Log.LogLn;
  Log.LogPlain('Supported Optimizations:');
  Log.LogPlain('  EnumNumbers');
  Log.LogPlain('  RemoveNotUsedPrivates');
  Log.LogLn;
  Log.LogPlain('Supported Whole Program Optimizations:');
  Log.LogPlain('  RemoveNotUsedDeclarations');
  Log.LogLn;
  Log.LogPlain('This program comes under the Library GNU General Public License');
  Log.LogPlain('For more information read COPYING.FPC, included in this distribution');
  Log.LogLn;
  Log.LogPlain('Please report bugs in our bug tracker on:');
  Log.LogPlain('                 http://bugs.freepascal.org');
  Log.LogLn;
  Log.LogPlain('More information may be found on our WWW pages (including directions');
  Log.LogPlain('for mailing lists useful for asking questions or discussing potential');
  Log.LogPlain('new features, etc.):');
  Log.LogPlain('                 http://www.freepascal.org');
end;

function TPas2jsCompiler.GetShownMsgTypes: TMessageTypes;
begin
  Result:=[mtFatal];
  if coShowErrors in FOptions then Include(Result,mtError);
  if coShowWarnings in FOptions then Include(Result,mtWarning);
  if coShowNotes in FOptions then Include(Result,mtNote);
  if coShowHints in FOptions then Include(Result,mtHint);
  if coShowInfos in FOptions then Include(Result,mtInfo);
  if coShowDebug in FOptions then Include(Result,mtDebug);
end;


procedure TPas2jsCompiler.SetOption(Flag: TP2jsCompilerOption; Enable: boolean);
begin
  if Enable then
    Options:=Options+[Flag]
  else
    Options:=Options-[Flag];
end;

function TPas2jsCompiler.FindFileWithUnitFilename(UnitFilename: string): TPas2jsCompilerFile;
begin
  if UnitFilename='' then exit(nil);
  Result:=TPas2jsCompilerFile(FFiles.FindKey(Pointer(UnitFilename)));
end;

function TPas2jsCompiler.CreateCompilerFile(const PasFileName,
  PCUFilename: String): TPas2jsCompilerFile;
begin
  Result:=TPas2jsCompilerFile.Create(Self,PasFileName,PCUFilename);
end;

procedure TPas2jsCompiler.LoadModuleFile(UnitFilename, UseUnitName: string; out
  aFile: TPas2jsCompilerFile; isPCU: Boolean);
// Creates aFile and opens the file, ready for parsing
// Note: aFile must be an out parameter and not a function result, so it is
//       already set while running
var
  aPasTree: TPas2jsCompilerResolver;
  ExpUnitFilename: String;
begin
  aFile:=nil;
  Log.LogMsg(nParsingFile,[FormatPath(UnitFilename)],'',0,0,not (coShowLineNumbers in Options));

  ExpUnitFilename:=UnitFilename;
  if ExpUnitFilename<>'' then
    ExpUnitFilename:=ExpandFileName(ExpUnitFilename);
  aFile:=FindFileWithUnitFilename(ExpUnitFilename);
  if aFile<>nil then exit;

  if (ExpUnitFilename='') or not FS.FileExists(ExpUnitFilename) then
  begin
    if isPCU then
      Log.LogMsg(nUnitFileNotFound,[QuoteStr(UnitFilename)])
    else
      Log.LogMsg(nSourceFileNotFound,[QuoteStr(UnitFilename)]);
    Terminate(ExitCodeFileNotFound);
  end;

  if FS.DirectoryExists(ExpUnitFilename) then
  begin
    Log.LogMsg(nFileIsFolder,[QuoteStr(UnitFilename)]);
    Terminate(ExitCodeFileNotFound);
  end;

  if isPCU then
    aFile:=CreateCompilerFile('',ExpUnitFilename)
  else
    aFile:=CreateCompilerFile(ExpUnitFilename,'');
  if UseUnitName<>'' then
    begin
    {$IFDEF VerboseSetPasUnitName}
    writeln('TPas2jsCompiler.LoadModuleFile File="',aFile.UnitFilename,'" UseUnit="',UseUnitName,'"');
    {$ENDIF}
    if CompareText(ExtractFilenameOnly(UnitFilename),UseUnitName)=0 then
      aFile.PasUnitName:=UseUnitName // e.g. when searching Unit1, found UNIT1.pas, use Unit1
    else
      aFile.PasUnitName:=ExtractFilenameOnly(UnitFilename);
    end;
  FFiles.Add(aFile);
  // do not add here aFile to FUnits
  aFile.ShowDebug:=ShowDebug;
  if aFile.IsMainFile then
    aFile.JSFilename:=GetResolvedMainJSFile;

  // pastree (engine)
  aPasTree:=aFile.PascalResolver;
  if coShowLineNumbers in Options then
    aPasTree.ScannerLogEvents:=aPasTree.ScannerLogEvents+[sleLineNumber];
  if coShowConditionals in Options then
    aPasTree.ScannerLogEvents:=aPasTree.ScannerLogEvents+[sleConditionals];
  if [coShowLineNumbers,coShowInfos,coShowDebug]*Options<>[] then
    aPasTree.ParserLogEvents:=aPasTree.ParserLogEvents+[pleInterface,pleImplementation];

  // scanner
  aFile.ResourceHandler:=FResources;;
  aFile.CreateScannerAndParser(FS.CreateResolver);

  if ShowDebug then
    Log.LogPlain(['Debug: Opening file "',UnitFilename,'"...']);
  if IsPCU then
    begin
    aFile.FileResolver.BaseDirectory:=ExtractFilePath(UnitFilename);
    aFile.PCUSupport.CreatePCUReader;
    end
  else
    begin
    // open file (beware: this changes FileResolver.BaseDirectory)
    aFile.OpenFile(UnitFilename);
    end;
end;

function TPas2jsCompiler.FindUnitJSFileName(aFileName: String): String;
begin
  if AllJSIntoMainJS then
    Result:=GetResolvedMainJSFile
  else
    Result:=FS.FindUnitJSFileName(aFilename);
end;

function TPas2jsCompiler.FindLoadedUnit(const TheUnitName: string
  ): TPas2jsCompilerFile;
begin
  if not IsValidIdent(TheUnitName,true) then exit(nil);
  Result:=TPas2jsCompilerFile(FUnits.FindKey(Pointer(TheUnitName)));
end;

procedure TPas2jsCompiler.AddUsedUnit(aFile: TPas2jsCompilerFile);
var
  OldFile: TPas2jsCompilerFile;
begin
  if aFile.PasUnitName='' then
    RaiseInternalError(20170504161347,'missing PasUnitName "'+aFile.UnitFilename+'"');
  OldFile:=FindLoadedUnit(aFile.PasUnitName);
  if OldFile<>nil then
  begin
    if OldFile<>aFile then
      RaiseInternalError(20170504161354,'duplicate unit "'+OldFile.PasUnitName+'" "'+aFile.UnitFilename+'" "'+OldFile.UnitFilename+'"');
  end else begin
    FUnits.Add(aFile);
  end;
end;

function TPas2jsCompiler.ExpandFileName(const Filename: string): string;

begin
  Result:=FS.ExpandFileName(Filename);
end;

procedure TPas2jsCompiler.InsertCustomJSFiles(aWriter: TPas2JSMapper);
var
  i: Integer;
  Filename: String;
  FileResolver: TPas2jsFSResolver;
  aFile: TPas2jsFile;
begin
  if InsertFilenames.Count=0 then exit;
  FileResolver:=FS.CreateResolver;
  try
    for i:=0 to InsertFilenames.Count-1 do begin
      Filename:=FS.FindCustomJSFileName(InsertFilenames[i]);
      if Filename='' then
      begin
        Log.LogMsg(nCustomJSFileNotFound,[InsertFilenames[i]]);
        raise EFileNotFoundError.Create('');
      end;
      aFile:=FS.LoadFile(Filename);
      if aFile.Source='' then continue;
      aWriter.WriteFile(aFile.Source,Filename);
    end
  finally
    FileResolver.Free;
  end;
end;

function TPas2jsCompiler.IndexOfInsertJSFilename(const aFilename: string
  ): integer;
var
  i: Integer;
begin
  for i:=0 to FInsertFilenames.Count-1 do
    if FS.SameFileName(aFilename,InsertFilenames[i]) then
      exit(i);
  Result:=-1;
end;

procedure TPas2jsCompiler.AddInsertJSFilename(const aFilename: string);
begin
  if IndexOfInsertJSFilename(aFilename)<0 then
    InsertFilenames.Add(aFilename);
end;

procedure TPas2jsCompiler.RemoveInsertJSFilename(const aFilename: string);
var
  i: Integer;
begin
  i:=IndexOfInsertJSFilename(aFilename);
  if i>=0 then
    InsertFilenames.Delete(i);
end;

function TPas2jsCompiler.GetResolvedMainJSFile: string;

begin
  if not FMainJSFileIsResolved then
  begin
    FMainJSFileResolved:=ResolvedMainJSFile;
    FMainJSFileIsResolved:=True;
  end;
  Result:=FMainJSFileResolved;
end;

function TPas2jsCompiler.GetUnitInfo(const UseUnitName, InFileName,
  ModuleDir: String; PCUSupport: TPCUSupport): TFindUnitInfo;

var
  FoundPasFilename, FoundPasUnitName: string;
  FoundPasIsForeign: Boolean;
  FoundPCUFilename, FoundPCUUnitName: string;

  procedure TryUnitName(const TestUnitName: string);
  var
    aFile: TPas2jsCompilerFile;
  begin
    if FoundPasFilename='' then
    begin
      // search loaded units
      aFile:=FindLoadedUnit(TestUnitName);
      if aFile<>nil then
      begin
        if aFile.PasFilename<>'' then
        begin
          FoundPasFilename:=aFile.PasFilename;
          FoundPasUnitName:=TestUnitName;
        end else if Assigned(PCUSupport) and (aFile.PCUFilename<>'')
            and (FoundPCUFilename='') then
        begin
          FoundPCUFilename:=aFile.PCUFilename;
          FoundPCUUnitName:=TestUnitName;
        end;
      end else begin
        // search pas in unit path
        FoundPasFilename:=FS.FindUnitFileName(TestUnitName,'',ModuleDir,FoundPasIsForeign);
        if FoundPasFilename<>'' then
          FoundPasUnitName:=TestUnitName;
      end;
    end;
    if Assigned(PCUSupport) and (FoundPCUFilename='')
      and (FoundPasFilename='') // for now: search pcu only if there is no pas
      then
    begin
      FoundPCUFilename:=PCUSupport.FindPCU(TestUnitName);
      if FoundPCUFilename<>'' then
        FoundPCUUnitName:=TestUnitName;
    end;
  end;

var
  aNameSpace, DefNameSpace: String;
  i: Integer;

begin
  //writeln('TPas2jsCompiler.GetUnitInfo ',UseUnitName,' in=',InFileName,' ',GetObjName(PCUSupport));
  Result:=Default(TFindUnitInfo);
  FoundPasFilename:='';
  FoundPasIsForeign:=false;
  FoundPasUnitName:='';
  FoundPCUFilename:='';
  FoundPCUUnitName:='';

  if InFilename='' then
  begin
    if Pos('.',UseUnitname)<1 then
    begin
      // generic unit name -> search with namespaces
      // first the default program namespace
      DefNameSpace:=GetDefaultNamespace;
      if DefNameSpace<>'' then
        TryUnitName(DefNameSpace+'.'+UseUnitname);

      if (FoundPasFilename='') or (FoundPCUFilename='') then
      begin
        // then the cmdline namespaces
        for i:=0 to Namespaces.Count-1 do
        begin
          aNameSpace:=Namespaces[i];
          if aNameSpace='' then continue;
          if SameText(aNameSpace,DefNameSpace) then continue;
          TryUnitName(aNameSpace+'.'+UseUnitname);
        end;
      end;
    end;

    if (FoundPasFilename='') or (FoundPCUFilename='') then
    begin
      // search unitname
      TryUnitName(UseUnitname);
    end;
  end else begin
    // search Pascal file with InFilename
    FoundPasFilename:=FS.FindUnitFileName(UseUnitname,InFilename,ModuleDir,FoundPasIsForeign);
    if FoundPasFilename='' then
      exit; // an in-filename unit source is missing -> stop
    FoundPasUnitName:=ExtractFilenameOnly(InFilename);

    // Note: at the moment if there is a source do not search for pcu
    // Eventually search for both, load pcu and if that fails unload pcu and load source
    if (FoundPasFilename='') and Assigned(PCUSupport) and (FoundPCUFilename='') then
    begin
      // no pas file -> search pcu
      FoundPCUFilename:=PCUSupport.FindPCU(UseUnitName);
      if FoundPCUFilename<>'' then
        FoundPCUUnitName:=UseUnitName;
    end;
  end;

  if (FoundPasFilename='') and (FoundPCUFilename<>'') then
  begin
    Result.FileName:=FoundPCUFilename;
    Result.UnitName:=FoundPCUUnitName;
    Result.isPCU:=True;
    Result.isForeign:=False;
  end else if (FoundPasFileName<>'') then
  begin
    Result.FileName:=FoundPasFilename;
    Result.UnitName:=FoundPasUnitName;
    Result.isPCU:=False;
    Result.isForeign:=FoundPasIsForeign;
  end;
end;

function TPas2jsCompiler.LoadUsedUnit(Info: TLoadUnitInfo;
  Context: TPas2jsCompilerFile): TPas2jsCompilerFile;

  function FindCycle(aFile, SearchFor: TPas2jsCompilerFile;
    var Cycle: TFPList): boolean;
  // Note: when traversing, add every search file to Cycle, to avoid running in circles.
  // When a cycle is detected, clear the Cycle list and build the cycle path
  var
    i: Integer;
    aParent: TPas2jsCompilerFile;
  begin
    Cycle.Add(aFile);
    for i:=0 to aFile.UsedByCount[ubMainSection]-1 do begin
      aParent:=aFile.UsedBy[ubMainSection,i];
      if aParent=SearchFor then
      begin
        // unit cycle found
        Cycle.Clear;
        Cycle.Add(aParent);
        Cycle.Add(aFile);
        exit(true);
      end;
      if Cycle.IndexOf(aParent)>=0 then
        continue;// already searched
      if FindCycle(aParent,SearchFor,Cycle) then
      begin
        Cycle.Add(aFile);
        exit(true);
      end;
    end;
    Result:=false;
  end;

var
  aFile: TPas2jsCompilerFile;

  procedure CheckCycle;
  var
    i: Integer;
    Cycle: TFPList;
    CyclePath: String;
  begin
    if Context.PasModule.ImplementationSection=nil then
    begin
      // main uses section (e.g. interface or program, not implementation)
      // -> check for cycles

      aFile.FUsedBy[ubMainSection].Add(Context);

      Cycle:=TFPList.Create;
      try
        if FindCycle(aFile,aFile,Cycle) then
        begin
          CyclePath:='';
          for i:=0 to Cycle.Count-1 do begin
            if i>0 then CyclePath+=',';
            CyclePath+=TPas2jsCompilerFile(Cycle[i]).GetModuleName;
          end;
          Context.PascalResolver.RaiseMsg(20180223141537,nUnitCycle,sUnitCycle,[CyclePath],Info.NameExpr);
        end;
      finally
        Cycle.Free;
      end;
    end else begin
      // implementation uses section
      aFile.FUsedBy[ubImplSection].Add(Context);
    end;
  end;

var
  UseJSFilename: String;
  OtherFile: TPas2jsCompilerFile;
begin
  Result:=nil;

  aFile:=FindFileWithUnitFilename(Info.UseFilename);

  if aFile<>nil then
  begin
    // known unit
    if (aFile.PasUnitName<>'') and (CompareText(aFile.PasUnitName,Info.UseUnitname)<>0) then
    begin
      Log.LogPlain(['Debug: TPas2jsPasTree.FindUnit unitname MISMATCH aFile.PasUnitname="',aFile.PasUnitName,'"',
         ' Self=',Context.FileResolver.FS.FormatPath(Context.UnitFilename),
         ' Uses=',Info.UseUnitname,
         ' IsForeign=',Context.IsForeign]);
      RaiseInternalError(20170504161412,'TPas2jsPasTree.FindUnit unit name mismatch');
    end;
    CheckCycle;
  end else begin
    // new unit

    if Info.InFilename<>'' then
    begin
      aFile:=FindLoadedUnit(Info.UseUnitname);
      if aFile<>nil then
      begin
        {$IF defined(VerbosePasResolver) or defined(VerbosePas2JS)}
        writeln('TPas2jsCompilerFile.FindUnit in-file unit name duplicate: New=',Info.UseFilename,' Old=',aFile.UnitFilename);
        {$ENDIF}
        Context.PascalResolver.RaiseMsg(20180223141323,nDuplicateFileFound,sDuplicateFileFound,
          [Info.UseFilename,aFile.UnitFilename],Info.InFileExpr);
      end;
    end;

    UseJSFilename:='';
    if (not Context.IsForeign) then
      UseJSFilename:=FindUnitJSFileName(Info.UseFilename);
    //  Log.LogPlain(['Debug: TPas2jsPasTree.FindUnit Self=',FileResolver.Cache.FormatPath(UnitFilename),
    //    ' Uses=',ActualUnitname,' Found="',FileResolver.Cache.FormatPath(UseFilename),'"',
    //    ' IsForeign=',IsForeign,' JSFile="',FileResolver.Cache.FormatPath(useJSFilename),'"']);
    // load Pascal or PCU file
    LoadModuleFile(Info.UseFilename,Info.UseUnitname,aFile,Info.IsPCU);

    // consistency checks
    if aFile.PasUnitName<>Info.UseUnitname then
      RaiseInternalError(20170922143329,'aFile.PasUnitName='+aFile.PasUnitName+' UseUnitname='+Info.UseUnitname);
    if Info.isPCU then
    begin
      if Not FS.SameFileName(aFile.PCUFilename,Info.UseFilename) then
        RaiseInternalError(20180312122331,'aFile.PCUFilename='+aFile.PCUFilename+' UseFilename='+Info.UseFilename);
    end else
    begin
      if Not FS.SameFileName(aFile.UnitFilename,Info.UseFilename) then
        RaiseInternalError(20170922143330,'aFile.UnitFilename='+aFile.UnitFilename+' UseFilename='+Info.UseFilename);
    end;

    if aFile=Context then
    begin
      // unit uses itself -> cycle
      Context.Parser.RaiseParserError(nUnitCycle,[Info.UseUnitname]);
    end;

    // add file to trees
    AddUsedUnit(aFile);
    // consistency checks
    OtherFile:=FindLoadedUnit(Info.UseUnitname);
    if aFile<>OtherFile then
    begin
      if OtherFile=nil then
        RaiseInternalError(20170922143405,'UseUnitname='+Info.UseUnitname)
      else
        RaiseInternalError(20170922143511,'UseUnitname='+Info.UseUnitname+' Found='+OtherFile.PasUnitName);
    end;
    OtherFile:=FindFileWithUnitFilename(Info.UseFilename);
    if aFile<>OtherFile then
    begin
      if OtherFile=nil then
        RaiseInternalError(20180224094625,'UseFilename='+Info.UseFilename)
      else
        RaiseInternalError(20180224094627,'UseFilename='+Info.UseFilename+' Found='+OtherFile.UnitFilename);
    end;

    CheckCycle;

    aFile.JSFilename:=UseJSFilename;
    aFile.IsForeign:=Info.UseIsForeign;

    // read
    aFile.ReadUnit;
    // beware: the parser may not yet have finished
  end;

  Result:=aFile;
end;

function TPas2jsCompiler.ResolvedMainJSFile: string;
Var
  OP,UP: String;

begin
  OP:=FS.MainOutputPath;
  UP:=FS.UnitOutputPath;
  if MainJSFile='.' then
    Result:=''
  else begin
    Result:=MainJSFile;
    if Result<>'' then
    begin
      // has option -o
      if ExtractFilePath(Result)='' then
      begin
        // -o<FileWithoutPath>
        if OP<>'' then
          Result:=OP+Result
        else if UP<>'' then
          Result:=UP+Result;
      end;
    end else begin
      // no option -o
      Result:=ChangeFileExt(MainSrcFile,'.js');
      if OP<>'' then
      begin
        // option -FE and no -o => put into MainOutputPath
        Result:=OP+ExtractFilename(Result)
      end else if UP<>'' then
      begin
        // option -FU and no -o and no -FE => put into UnitOutputPath
        Result:=UP+ExtractFilename(Result)
      end else begin
        // no -FU, no -FE and no -o => put into source directory
      end;
    end;
  end;
end;

end.

