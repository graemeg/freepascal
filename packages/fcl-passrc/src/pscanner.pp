{
    This file is part of the Free Component Library

    Pascal source lexical scanner
    Copyright (c) 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit PScanner;

{$mode objfpc}
{$h+}

{$ifdef fpc}
  {$define UsePChar}
  {$define UseAnsiStrings}
  {$define HasStreams}
  {$IF FPC_FULLVERSION<30101}
    {$define EmulateArrayInsert}
  {$endif}
  {$define HasFS}
{$endif}

{$IFDEF NODEJS}
  {$define HasFS}
{$ENDIF}

interface

uses
  {$ifdef pas2js}
  js,
  {$IFDEF NODEJS}
  NodeJSFS,
  {$ENDIF}
  Types,
  {$endif}
  SysUtils, Classes;

// message numbers
const
  nErrInvalidCharacter = 1001;
  nErrOpenString = 1002;
  nErrIncludeFileNotFound = 1003;
  nErrIfXXXNestingLimitReached = 1004;
  nErrInvalidPPElse = 1005;
  nErrInvalidPPEndif = 1006;
  nLogOpeningFile = 1007;
  nLogLineNumber = 1008; // same as FPC
  nLogIFDefAccepted = 1009;
  nLogIFDefRejected = 1010;
  nLogIFNDefAccepted = 1011;
  nLogIFNDefRejected = 1012;
  nLogIFAccepted = 1013;
  nLogIFRejected = 1014;
  nLogIFOptAccepted = 1015;
  nLogIFOptRejected = 1016;
  nLogELSEIFAccepted = 1017;
  nLogELSEIFRejected = 1018;
  nErrInvalidMode = 1019;
  nErrInvalidModeSwitch = 1020;
  nErrXExpectedButYFound = 1021;
  nErrRangeCheck = 1022;
  nErrDivByZero = 1023;
  nErrOperandAndOperatorMismatch = 1024;
  nUserDefined = 1025;
  nLogMacroDefined = 1026; // FPC=3101
  nLogMacroUnDefined = 1027; // FPC=3102
  nWarnIllegalCompilerDirectiveX = 1028;
  nIllegalStateForWarnDirective = 1027;
  nErrIncludeLimitReached = 1028;
  nMisplacedGlobalCompilerSwitch = 1029;
  nLogMacroXSetToY = 1030;
  nInvalidDispatchFieldName = 1031;
  nErrWrongSwitchToggle = 1032;
  nNoResourceSupport = 1033;
  nResourceFileNotFound = 1034;

// resourcestring patterns of messages
resourcestring
  SErrInvalidCharacter = 'Invalid character ''%s''';
  SErrOpenString = 'string exceeds end of line';
  SErrIncludeFileNotFound = 'Could not find include file ''%s''';
  SErrResourceFileNotFound = 'Could not find resource file ''%s''';
  SErrIfXXXNestingLimitReached = 'Nesting of $IFxxx too deep';
  SErrInvalidPPElse = '$ELSE without matching $IFxxx';
  SErrInvalidPPEndif = '$ENDIF without matching $IFxxx';
  SLogOpeningFile = 'Opening source file "%s".';
  SLogLineNumber = 'Reading line %d.';
  SLogIFDefAccepted = 'IFDEF %s found, accepting.';
  SLogIFDefRejected = 'IFDEF %s found, rejecting.';
  SLogIFNDefAccepted = 'IFNDEF %s found, accepting.';
  SLogIFNDefRejected = 'IFNDEF %s found, rejecting.';
  SLogIFAccepted = 'IF %s found, accepting.';
  SLogIFRejected = 'IF %s found, rejecting.';
  SLogIFOptAccepted = 'IFOpt %s found, accepting.';
  SLogIFOptRejected = 'IFOpt %s found, rejecting.';
  SLogELSEIFAccepted = 'ELSEIF %s found, accepting.';
  SLogELSEIFRejected = 'ELSEIF %s found, rejecting.';
  SErrInvalidMode = 'Invalid mode: "%s"';
  SErrInvalidModeSwitch = 'Invalid mode switch: "%s"';
  SErrXExpectedButYFound = '"%s" expected, but "%s" found';
  SErrRangeCheck = 'range check failed';
  SErrDivByZero = 'division by zero';
  SErrOperandAndOperatorMismatch = 'operand and operator mismatch';
  SUserDefined = 'User defined: "%s"';
  SLogMacroDefined = 'Macro defined: %s';
  SLogMacroUnDefined = 'Macro undefined: %s';
  SWarnIllegalCompilerDirectiveX = 'Illegal compiler directive "%s"';
  SIllegalStateForWarnDirective = 'Illegal state "%s" for $WARN directive';
  SErrIncludeLimitReached = 'Include file limit reached';
  SMisplacedGlobalCompilerSwitch = 'Misplaced global compiler switch, ignored';
  SLogMacroXSetToY = 'Macro %s set to %s';
  SInvalidDispatchFieldName = 'Invalid Dispatch field name';
  SErrWrongSwitchToggle = 'Wrong switch toggle, use ON/OFF or +/-';
  SNoResourceSupport = 'No support for resources of type "%s"';

type
  TMessageType = (
    mtFatal,
    mtError,
    mtWarning,
    mtNote,
    mtHint,
    mtInfo,
    mtDebug
    );
  TMessageTypes = set of TMessageType;

  TMessageArgs = array of string;

  TToken = (
    tkEOF,
    tkWhitespace,
    tkComment,
    tkIdentifier,
    tkString,
    tkNumber,
    tkChar, // ^A .. ^Z
    // Simple (one-character) tokens
    tkBraceOpen,             // '('
    tkBraceClose,            // ')'
    tkMul,                   // '*'
    tkPlus,                  // '+'
    tkComma,                 // ','
    tkMinus,                 // '-'
    tkDot,                   // '.'
    tkDivision,              // '/'
    tkColon,                 // ':'
    tkSemicolon,             // ';'
    tkLessThan,              // '<'
    tkEqual,                 // '='
    tkGreaterThan,           // '>'
    tkAt,                    // '@'
    tkSquaredBraceOpen,      // '['
    tkSquaredBraceClose,     // ']'
    tkCaret,                 // '^'
    tkBackslash,             // '\'
    // Two-character tokens
    tkDotDot,                // '..'
    tkAssign,                // ':='
    tkNotEqual,              // '<>'
    tkLessEqualThan,         // '<='
    tkGreaterEqualThan,      // '>='
    tkPower,                 // '**'
    tkSymmetricalDifference, // '><'
    tkAssignPlus,            // +=
    tkAssignMinus,           // -=
    tkAssignMul,             // *=
    tkAssignDivision,        // /=
    tkAtAt,                  // @@
    // Reserved words
    tkabsolute,
    tkand,
    tkarray,
    tkas,
    tkasm,
    tkbegin,
    tkbitpacked,
    tkcase,
    tkclass,
    tkconst,
    tkconstref,
    tkconstructor,
    tkdestructor,
    tkdispinterface,
    tkdiv,
    tkdo,
    tkdownto,
    tkelse,
    tkend,
    tkexcept,
    tkexports,
    tkfalse,
    tkfile,
    tkfinalization,
    tkfinally,
    tkfor,
    tkfunction,
    tkgeneric,
    tkgoto,
    tkif,
    tkimplementation,
    tkin,
    tkinherited,
    tkinitialization,
    tkinline,
    tkinterface,
    tkis,
    tklabel,
    tklibrary,
    tkmod,
    tknil,
    tknot,
    tkobject,
    tkof,
    tkoperator,
    tkor,
    tkpacked,
    tkprocedure,
    tkprogram,
    tkproperty,
    tkraise,
    tkrecord,
    tkrepeat,
    tkResourceString,
    tkself,
    tkset,
    tkshl,
    tkshr,
    tkspecialize,
//    tkstring,
    tkthen,
    tkthreadvar,
    tkto,
    tktrue,
    tktry,
    tktype,
    tkunit,
    tkuntil,
    tkuses,
    tkvar,
    tkwhile,
    tkwith,
    tkxor,
    tkLineEnding,
    tkTab
    );
  TTokens = set of TToken;

  TModeSwitch = (
    msNone,
    { generic }
    msFpc, msObjfpc, msDelphi, msDelphiUnicode, msTP7, msMac, msIso, msExtpas, msGPC,
    { more specific }
    msClass,               { delphi class model }
    msObjpas,              { load objpas unit }
    msResult,              { result in functions }
    msStringPchar,         { pchar 2 string conversion }
    msCVarSupport,         { cvar variable directive }
    msNestedComment,       { nested comments }
    msTPProcVar,           { tp style procvars (no @ needed) }
    msMacProcVar,          { macpas style procvars }
    msRepeatForward,       { repeating forward declarations is needed }
    msPointer2Procedure,   { allows the assignement of pointers to
                             procedure variables                     }
    msAutoDeref,           { does auto dereferencing of struct. vars }
    msInitFinal,           { initialization/finalization for units }
    msDefaultAnsistring,   { ansistring turned on by default }
    msOut,                 { support the calling convention OUT }
    msDefaultPara,         { support default parameters }
    msHintDirective,       { support hint directives }
    msDuplicateNames,      { allow locals/paras to have duplicate names of globals }
    msProperty,            { allow properties }
    msDefaultInline,       { allow inline proc directive }
    msExcept,              { allow exception-related keywords }
    msObjectiveC1,         { support interfacing with Objective-C (1.0) }
    msObjectiveC2,         { support interfacing with Objective-C (2.0) }
    msNestedProcVars,      { support nested procedural variables }
    msNonLocalGoto,        { support non local gotos (like iso pascal) }
    msAdvancedRecords,     { advanced record syntax with visibility sections, methods and properties }
    msISOLikeUnaryMinus,   { unary minus like in iso pascal: same precedence level as binary minus/plus }
    msSystemCodePage,      { use system codepage as compiler codepage by default, emit ansistrings with system codepage }
    msFinalFields,         { allows declaring fields as "final", which means they must be initialised
                             in the (class) constructor and are constant from then on (same as final
                             fields in Java) }
    msDefaultUnicodestring, { makes the default string type in $h+ mode unicodestring rather than
                               ansistring; similarly, char becomes unicodechar rather than ansichar }
    msTypeHelpers,         { allows the declaration of "type helper" (non-Delphi) or "record helper"
                             (Delphi) for primitive types }
    msCBlocks,             { 'cblocks', support for http://en.wikipedia.org/wiki/Blocks_(C_language_extension) }
    msISOLikeIO,           { I/O as it required by an ISO compatible compiler }
    msISOLikeProgramsPara, { program parameters as it required by an ISO compatible compiler }
    msISOLikeMod,          { mod operation as it is required by an iso compatible compiler }
    msArrayOperators,      { use Delphi compatible array operators instead of custom ones ("+") }
    msExternalClass,       { Allow external class definitions }
    msPrefixedAttributes,  { Allow attributes, disable proc modifier [] }
    msOmitRTTI,            { treat class section 'published' as 'public' and typeinfo does not work on symbols declared with this switch }
    msMultiHelpers,        { off=only one helper per type, on=all }
    msImplicitFunctionSpec { implicit function specialization }
    );
  TModeSwitches = Set of TModeSwitch;

  // switches, that can be 'on' or 'off'
  TBoolSwitch = (
    bsNone,
    bsAlign,          // A   align fields
    bsBoolEval,       // B   complete boolean evaluation
    bsAssertions,     // C   generate code for assertions
    bsDebugInfo,      // D   generate debuginfo (debug lines), OR: $description 'text'
    bsExtension,      // E   output file extension
                      // F
    bsImportedData,   // G
    bsLongStrings,    // H   String=AnsiString
    bsIOChecks,       // I   generate EInOutError
    bsWriteableConst, // J   writable typed const
                      // K
    bsLocalSymbols,   // L   generate local symbol information (debug, requires $D+)
    bsTypeInfo,       // M   allow published members OR $M minstacksize,maxstacksize
                      // N
    bsOptimization,   // O   enable safe optimizations (-O1)
    bsOpenStrings,    // P   deprecated Delphi directive
    bsOverflowChecks, // Q   or $OV
    bsRangeChecks,    // R
                      // S
    bsTypedAddress,   // T   enabled: @variable gives typed pointer, otherwise untyped pointer
    bsSafeDivide,     // U
    bsVarStringChecks,// V   strict shortstring checking, e.g. cannot pass shortstring[3] to shortstring
    bsStackframes,    // W   always generate stackframes (debugging)
    bsExtendedSyntax, // X   deprecated Delphi directive
    bsReferenceInfo,  // Y   store for each identifier the declaration location
                      // Z
    bsHints,
    bsNotes,
    bsWarnings,
    bsMacro,
    bsScopedEnums,
    bsObjectChecks,   // check methods 'Self' and object type casts
    bsPointerMath,    // pointer arithmetic
    bsGoto       // support label and goto, set by {$goto on|off}
    );
  TBoolSwitches = set of TBoolSwitch;
const
  LetterToBoolSwitch: array['A'..'Z'] of TBoolSwitch = (
    bsAlign,          // A
    bsBoolEval,       // B
    bsAssertions,     // C
    bsDebugInfo,      // D or $description
    bsExtension,      // E
    bsNone,           // F
    bsImportedData,   // G
    bsLongStrings,    // H
    bsIOChecks,       // I or $include
    bsWriteableConst, // J
    bsNone,           // K
    bsLocalSymbols,   // L
    bsTypeInfo,       // M or $M minstacksize,maxstacksize
    bsNone,           // N
    bsOptimization,   // O
    bsOpenStrings,    // P
    bsOverflowChecks, // Q
    bsRangeChecks,    // R or $resource
    bsNone,           // S
    bsTypedAddress,   // T
    bsSafeDivide,     // U
    bsVarStringChecks,// V
    bsStackframes,    // W
    bsExtendedSyntax, // X
    bsReferenceInfo,  // Y
    bsNone            // Z
    );

  bsAll = [low(TBoolSwitch)..high(TBoolSwitch)];
  bsFPCMode: TBoolSwitches = [bsPointerMath,bsWriteableConst];
  bsObjFPCMode: TBoolSwitches = [bsPointerMath,bsWriteableConst];
  bsDelphiMode: TBoolSwitches = [bsWriteableConst,bsGoto];
  bsDelphiUnicodeMode: TBoolSwitches = [bsWriteableConst,bsGoto];
  bsMacPasMode: TBoolSwitches = [bsPointerMath,bsWriteableConst];

type
  TValueSwitch = (
    vsInterfaces,
    vsDispatchField,
    vsDispatchStrField
    );
  TValueSwitches = set of TValueSwitch;
  TValueSwitchArray = array[TValueSwitch] of string;
const
  vsAllValueSwitches = [low(TValueSwitch)..high(TValueSwitch)];
  DefaultValueSwitches: array[TValueSwitch] of string = (
     'com', // vsInterfaces
     'Msg', // vsDispatchField
     'MsgStr' // vsDispatchStrField
     );
  DefaultMaxIncludeStackDepth = 20;

type
  TWarnMsgState = (
    wmsDefault,
    wmsOn,
    wmsOff,
    wmsError
  );

type
  TTokenOption = (toForceCaret,toOperatorToken);
  TTokenOptions = Set of TTokenOption;


  { TMacroDef }

  TMacroDef = Class(TObject)
  Private
    FName: String;
    FValue: String;
  Public
    Constructor Create(Const AName,AValue : String);
    Property Name  : String Read FName;
    Property Value : String Read FValue Write FValue;
  end;

  { TLineReader }

  TLineReader = class
  Private
    FFilename: string;
  public
    constructor Create(const AFilename: string); virtual;
    function IsEOF: Boolean; virtual; abstract;
    function ReadLine: string; virtual; abstract;
    property Filename: string read FFilename;
  end;

  { TFileLineReader }

  TFileLineReader = class(TLineReader)
  private
    {$ifdef pas2js}
    {$else}
    FTextFile: Text;
    FFileOpened: Boolean;
    FBuffer : Array[0..4096-1] of byte;
    {$endif}
  public
    constructor Create(const AFilename: string); override;
    destructor Destroy; override;
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
  end;

  { TStreamLineReader }

  TStreamLineReader = class(TLineReader)
  private
    FContent: String;
    FPos : Integer;
  public
    {$ifdef HasStreams}
    Procedure InitFromStream(AStream : TStream);
    {$endif}
    Procedure InitFromString(const s: string);
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
  end;

  { TFileStreamLineReader }

  TFileStreamLineReader = class(TStreamLineReader)
  Public
    constructor Create(const AFilename: string); override;
  end;

  { TStringStreamLineReader }

  TStringStreamLineReader = class(TStreamLineReader)
  Public
    constructor Create(const AFilename: string; Const ASource: String); reintroduce;
  end;

  { TMacroReader }

  TMacroReader = Class(TStringStreamLineReader)
  private
    FCurCol: Integer;
    FCurRow: Integer;
  Public
    Property CurCol : Integer Read FCurCol Write FCurCol;
    Property CurRow : Integer Read FCurRow Write FCurRow;
  end;

  { TBaseFileResolver }

  TBaseFileResolver = class
  private
    FBaseDirectory: string;
    FResourcePaths,
    FIncludePaths: TStringList;
    FStrictFileCase : Boolean;
  Protected
    function FindIncludeFileName(const aFilename: string): String; virtual; abstract;
    procedure SetBaseDirectory(AValue: string); virtual;
    procedure SetStrictFileCase(AValue: Boolean); virtual;
    Property IncludePaths: TStringList Read FIncludePaths;
    Property ResourcePaths: TStringList Read FResourcePaths;
  public
    constructor Create; virtual;
    destructor Destroy; override;
    procedure AddIncludePath(const APath: string); virtual;
    procedure AddResourcePath(const APath: string); virtual;
    function FindResourceFileName(const AName: string): String; virtual; abstract;
    function FindSourceFile(const AName: string): TLineReader; virtual; abstract;
    function FindIncludeFile(const AName: string): TLineReader; virtual; abstract;
    Property StrictFileCase : Boolean Read FStrictFileCase Write SetStrictFileCase;
    property BaseDirectory: string read FBaseDirectory write SetBaseDirectory;
  end;
  TBaseFileResolverClass = Class of TBaseFileResolver;

{$IFDEF HASFS}
  { TFileResolver }

  TFileResolver = class(TBaseFileResolver)
  private
    {$ifdef HasStreams}
    FUseStreams: Boolean;
    {$endif}
  Protected
    function SearchLowUpCase(FN: string): string;
    Function FindIncludeFileName(const AName: string): String; override;
    Function CreateFileReader(Const AFileName : String) : TLineReader; virtual;
  Public
    function FindResourceFileName(const AFileName: string): String; override;
    function FindSourceFile(const AName: string): TLineReader; override;
    function FindIncludeFile(const AName: string): TLineReader; override;
    {$ifdef HasStreams}
    Property UseStreams : Boolean Read FUseStreams Write FUseStreams;
    {$endif}
  end;
{$ENDIF}

  {$ifdef fpc}
  { TStreamResolver }

  TStreamResolver = class(TBaseFileResolver)
  Private
    FOwnsStreams: Boolean;
    FStreams : TStringList;
    function FindStream(const AName: string; ScanIncludes: Boolean): TStream;
    function FindStreamReader(const AName: string; ScanIncludes: Boolean): TLineReader;
    procedure SetOwnsStreams(AValue: Boolean);
  Protected
    function FindIncludeFileName(const aFilename: string): String; override;
  Public
    constructor Create; override;
    destructor Destroy; override;
    Procedure Clear;
    function FindResourceFileName(const AFileName: string): String; override;
    Procedure AddStream(Const AName : String; AStream : TStream);
    function FindSourceFile(const AName: string): TLineReader; override;
    function FindIncludeFile(const AName: string): TLineReader; override;
    Property OwnsStreams : Boolean Read FOwnsStreams write SetOwnsStreams;
    Property Streams: TStringList read FStreams;
  end;
  {$endif}

const
  CondDirectiveBool: array[boolean] of string = (
    '0', // false
    '1'  // true  Note: True is <>'0'
    );
type
  TMaxPrecInt = {$ifdef fpc}int64{$else}NativeInt{$endif};
  TMaxFloat = {$ifdef fpc}extended{$else}double{$endif};

  TCondDirectiveEvaluator = class;

  TCEEvalVarEvent = function(Sender: TCondDirectiveEvaluator; Name: String; out Value: string): boolean of object;
  TCEEvalFunctionEvent = function(Sender: TCondDirectiveEvaluator; Name, Param: String; out Value: string): boolean of object;
  TCELogEvent = procedure(Sender: TCondDirectiveEvaluator; Args : Array of {$ifdef pas2js}jsvalue{$else}const{$endif}) of object;

  { TCondDirectiveEvaluator - evaluate $IF expression }

  TCondDirectiveEvaluator = class
  private
    FOnEvalFunction: TCEEvalFunctionEvent;
    FOnEvalVariable: TCEEvalVarEvent;
    FOnLog: TCELogEvent;
  protected
    type
      TPrecedenceLevel = (
        ceplFirst, // tkNot
        ceplSecond, // *, /, div, mod, and, shl, shr
        ceplThird, // +, -, or, xor
        ceplFourth // =, <>, <, >, <=, >=
        );
      TStackItem = record
        Level: TPrecedenceLevel;
        Operathor: TToken;
        Operand: String;
        OperandPos: integer;
      end;
  protected
    {$ifdef UsePChar}
    FTokenStart: PChar;
    FTokenEnd: PChar;
    {$else}
    FTokenStart: integer; // position in Expression
    FTokenEnd: integer; // position in Expression
    {$endif}
    FToken: TToken;
    FStack: array of TStackItem;
    FStackTop: integer;
    function IsFalse(const Value: String): boolean; inline;
    function IsTrue(const Value: String): boolean; inline;
    function IsInteger(const Value: String; out i: TMaxPrecInt): boolean;
    function IsExtended(const Value: String; out e: TMaxFloat): boolean;
    procedure NextToken;
    procedure Log(aMsgType: TMessageType; aMsgNumber: integer;
      const aMsgFmt: String; const Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif}; MsgPos: integer = 0);
    procedure LogXExpectedButTokenFound(const X: String; ErrorPos: integer = 0);
    procedure ReadOperand(Skip: boolean = false); // unary operators plus one operand
    procedure ReadExpression; // binary operators
    procedure ResolveStack(MinStackLvl: integer; Level: TPrecedenceLevel;
      NewOperator: TToken);
    function GetTokenString: String;
    function GetStringLiteralValue: String; // read value of tkString
    procedure Push(const AnOperand: String; OperandPosition: integer);
  public
    Expression: String;
    MsgPos: integer;
    MsgNumber: integer;
    MsgType: TMessageType;
    MsgPattern: String; // Format parameter
    constructor Create;
    destructor Destroy; override;
    function Eval(const Expr: string): boolean;
    property OnEvalVariable: TCEEvalVarEvent read FOnEvalVariable write FOnEvalVariable;
    property OnEvalFunction: TCEEvalFunctionEvent read FOnEvalFunction write FOnEvalFunction;
    property OnLog: TCELogEvent read FOnLog write FOnLog;
  end;

  EScannerError       = class(Exception);
  EFileNotFoundError  = class(Exception);

  TPascalScannerPPSkipMode = (ppSkipNone, ppSkipIfBranch, ppSkipElseBranch, ppSkipAll);

  TPOption = (
    po_delphi,               // DEPRECATED since fpc 3.1.1: Delphi mode: forbid nested comments
    po_KeepScannerError,     // default: catch EScannerError and raise an EParserError instead
    po_CAssignments,         // allow C-operators += -= *= /=
    po_ResolveStandardTypes, // search for 'longint', 'string', etc., do not use dummies, TPasResolver sets this to use its declarations
    po_AsmWhole,             // store whole text between asm..end in TPasImplAsmStatement.Tokens
    po_NoOverloadedProcs,    // do not create TPasOverloadedProc for procs with same name
    po_KeepClassForward,     // disabled: delete class fowards when there is a class declaration
    po_ArrayRangeExpr,       // enable: create TPasArrayType.IndexRange, disable: create TPasArrayType.Ranges
    po_SelfToken,            // Self is a token. For backward compatibility.
    po_CheckModeSwitches,    // error on unknown modeswitch with an error
    po_CheckCondFunction,    // error on unknown function in conditional expression, default: return '0'
    po_StopOnErrorDirective, // error on user $Error, $message error|fatal
    po_ExtConstWithoutExpr,  // allow typed const without expression in external class and with external modifier
    po_StopOnUnitInterface   // parse only a unit name and stop at interface keyword
    );
  TPOptions = set of TPOption;

type
  TPasSourcePos = Record
    FileName: String;
    Row, Column: Cardinal;
  end;
const
  DefPasSourcePos: TPasSourcePos = (Filename:''; Row:0; Column:0);

type
  { TPascalScanner }

  TPScannerLogHandler = Procedure (Sender : TObject; Const Msg : String) of object;
  TPScannerLogEvent = (sleFile,sleLineNumber,sleConditionals,sleDirective);
  TPScannerLogEvents = Set of TPScannerLogEvent;
  TPScannerDirectiveEvent = procedure(Sender: TObject; Directive, Param: String;
    var Handled: boolean) of object;
  TPScannerFormatPathEvent = function(const aPath: string): string of object;
  TPScannerWarnEvent = procedure(Sender: TObject; Identifier: string; State: TWarnMsgState; var Handled: boolean) of object;
  TPScannerModeDirective = procedure(Sender: TObject; NewMode: TModeSwitch; Before: boolean; var Handled: boolean) of object;

  // aFileName: full filename (search is already done) aOptions: list of name:value pairs.
  TResourceHandler = Procedure (Sender : TObject; const aFileName : String; aOptions : TStrings) of object;

  TPasScannerTokenPos = {$ifdef UsePChar}PChar{$else}integer{$endif};

  TPascalScanner = class
  private
    type
      TResourceHandlerRecord = record
        Ext : String;
        Handler : TResourceHandler;
      end;
      TWarnMsgNumberState = record
        Number: integer;
        State: TWarnMsgState;
      end;
      TWarnMsgNumberStateArr = array of TWarnMsgNumberState;
  private
    FAllowedBoolSwitches: TBoolSwitches;
    FAllowedModeSwitches: TModeSwitches;
    FAllowedValueSwitches: TValueSwitches;
    FConditionEval: TCondDirectiveEvaluator;
    FCurrentBoolSwitches: TBoolSwitches;
    FCurrentModeSwitches: TModeSwitches;
    FCurrentValueSwitches: TValueSwitchArray;
    FCurTokenPos: TPasSourcePos;
    FLastMsg: string;
    FLastMsgArgs: TMessageArgs;
    FLastMsgNumber: integer;
    FLastMsgPattern: string;
    FLastMsgType: TMessageType;
    FFileResolver: TBaseFileResolver;
    FCurSourceFile: TLineReader;
    FCurFilename: string;
    FCurRow: Integer;
    FCurColumnOffset: integer;
    FCurToken: TToken;
    FCurTokenString: string;
    FCurLine: string;
    FMaxIncludeStackDepth: integer;
    FModuleRow: Integer;
    FMacros: TStrings; // Objects are TMacroDef
    FDefines: TStrings;
    FNonTokens: TTokens;
    FOnDirective: TPScannerDirectiveEvent;
    FOnEvalFunction: TCEEvalFunctionEvent;
    FOnEvalVariable: TCEEvalVarEvent;
    FOnFormatPath: TPScannerFormatPathEvent;
    FOnModeChanged: TPScannerModeDirective;
    FOnWarnDirective: TPScannerWarnEvent;
    FOptions: TPOptions;
    FLogEvents: TPScannerLogEvents;
    FOnLog: TPScannerLogHandler;
    FPreviousToken: TToken;
    FReadOnlyBoolSwitches: TBoolSwitches;
    FReadOnlyModeSwitches: TModeSwitches;
    FReadOnlyValueSwitches: TValueSwitches;
    FSkipComments: Boolean;
    FSkipGlobalSwitches: boolean;
    FSkipWhiteSpace: Boolean;
    FTokenOptions: TTokenOptions;
    FTokenPos: TPasScannerTokenPos; // position in FCurLine }
    FIncludeStack: TFPList;
    FFiles: TStrings;
    FWarnMsgStates: TWarnMsgNumberStateArr;
    FResourceHandlers : Array of TResourceHandlerRecord;

    // Preprocessor $IFxxx skipping data
    PPSkipMode: TPascalScannerPPSkipMode;
    PPIsSkipping: Boolean;
    PPSkipStackIndex: Integer;
    PPSkipModeStack: array[0..255] of TPascalScannerPPSkipMode;
    PPIsSkippingStack: array[0..255] of Boolean;
    function GetCurColumn: Integer;
    function GetCurrentValueSwitch(V: TValueSwitch): string;
    function GetForceCaret: Boolean;
    function GetMacrosOn: boolean;
    function IndexOfWarnMsgState(Number: integer; InsertPos: boolean): integer;
    function OnCondEvalFunction(Sender: TCondDirectiveEvaluator; Name,
      Param: String; out Value: string): boolean;
    procedure OnCondEvalLog(Sender: TCondDirectiveEvaluator;
      Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
    function OnCondEvalVar(Sender: TCondDirectiveEvaluator; Name: String; out
      Value: string): boolean;
    procedure SetAllowedBoolSwitches(const AValue: TBoolSwitches);
    procedure SetAllowedModeSwitches(const AValue: TModeSwitches);
    procedure SetAllowedValueSwitches(const AValue: TValueSwitches);
    procedure SetMacrosOn(const AValue: boolean);
    procedure SetOptions(AValue: TPOptions);
    procedure SetReadOnlyBoolSwitches(const AValue: TBoolSwitches);
    procedure SetReadOnlyModeSwitches(const AValue: TModeSwitches);
    procedure SetReadOnlyValueSwitches(const AValue: TValueSwitches);
  protected
    // extension without initial dot (.)
    Function IndexOfResourceHandler(Const aExt : string) : Integer;
    Function FindResourceHandler(Const aExt : string) : TResourceHandler;
    function ReadIdentifier(const AParam: string): string;
    function FetchLine: boolean;
    procedure AddFile(aFilename: string); virtual;
    function GetMacroName(const Param: String): String;
    procedure SetCurMsg(MsgType: TMessageType; MsgNumber: integer; Const Fmt : String; Args : Array of {$ifdef pas2js}jsvalue{$else}const{$endif});
    Procedure DoLog(MsgType: TMessageType; MsgNumber: integer; Const Msg : String; SkipSourceInfo : Boolean = False);overload;
    Procedure DoLog(MsgType: TMessageType; MsgNumber: integer; Const Fmt : String; Args : Array of {$ifdef pas2js}jsvalue{$else}const{$endif};SkipSourceInfo : Boolean = False);overload;
    procedure Error(MsgNumber: integer; const Msg: string);overload;
    procedure Error(MsgNumber: integer; const Fmt: string; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});overload;
    procedure PushSkipMode;
    function HandleDirective(const ADirectiveText: String): TToken; virtual;
    function HandleLetterDirective(Letter: char; Enable: boolean): TToken; virtual;
    procedure HandleBoolDirective(bs: TBoolSwitch; const Param: String); virtual;
    procedure DoHandleDirective(Sender: TObject; Directive, Param: String;
      var Handled: boolean); virtual;
    procedure HandleIFDEF(const AParam: String);
    procedure HandleIFNDEF(const AParam: String);
    procedure HandleIFOPT(const AParam: String);
    procedure HandleIF(const AParam: String);
    procedure HandleELSEIF(const AParam: String);
    procedure HandleELSE(const AParam: String);
    procedure HandleENDIF(const AParam: String);
    procedure HandleDefine(Param: String); virtual;
    procedure HandleDispatchField(Param: String; vs: TValueSwitch); virtual;
    procedure HandleError(Param: String); virtual;
    procedure HandleMessageDirective(Param: String); virtual;
    procedure HandleIncludeFile(Param: String); virtual;
    procedure HandleResource(Param : string); virtual;

    procedure HandleUnDefine(Param: String); virtual;

    function HandleInclude(const Param: String): TToken; virtual;
    procedure HandleMode(const Param: String); virtual;
    procedure HandleModeSwitch(const Param: String); virtual;
    function HandleMacro(AIndex: integer): TToken; virtual;
    procedure HandleInterfaces(const Param: String); virtual;
    procedure HandleWarn(Param: String); virtual;
    procedure HandleWarnIdentifier(Identifier, Value: String); virtual;
    procedure PushStackItem; virtual;
    function DoFetchTextToken: TToken;
    function DoFetchToken: TToken;
    procedure ClearFiles;
    Procedure ClearMacros;
    Procedure SetCurToken(const AValue: TToken);
    Procedure SetCurTokenString(const AValue: string);
    procedure SetCurrentBoolSwitches(const AValue: TBoolSwitches); virtual;
    procedure SetCurrentModeSwitches(AValue: TModeSwitches); virtual;
    procedure SetCurrentValueSwitch(V: TValueSwitch; const AValue: string);
    procedure SetWarnMsgState(Number: integer; State: TWarnMsgState); virtual;
    function GetWarnMsgState(Number: integer): TWarnMsgState; virtual;
    function LogEvent(E : TPScannerLogEvent) : Boolean; inline;
    property TokenPos: TPasScannerTokenPos read FTokenPos write FTokenPos;
  public
    constructor Create(AFileResolver: TBaseFileResolver);
    destructor Destroy; override;
    // extension without initial dot  (.), case insensitive
    Procedure RegisterResourceHandler(aExtension : String; aHandler : TResourceHandler); overload;
    Procedure RegisterResourceHandler(aExtensions : Array of String; aHandler : TResourceHandler); overload;
    procedure OpenFile(AFilename: string);
    procedure FinishedModule; virtual; // called by parser after end.
    function FormatPath(const aFilename: string): string; virtual;
    procedure SetNonToken(aToken : TToken);
    procedure UnsetNonToken(aToken : TToken);
    procedure SetTokenOption(aOption : TTokenoption);
    procedure UnSetTokenOption(aOption : TTokenoption);
    function CheckToken(aToken : TToken; const ATokenString : String) : TToken;
    function FetchToken: TToken;
    function ReadNonPascalTillEndToken(StopAtLineEnd: boolean): TToken; virtual;
    function AddDefine(const aName: String; Quiet: boolean = false): boolean;
    function RemoveDefine(const aName: String; Quiet: boolean = false): boolean;
    function UnDefine(const aName: String; Quiet: boolean = false): boolean; // check defines and macros
    function IsDefined(const aName: String): boolean; // check defines and macros
    function IfOpt(Letter: Char): boolean;
    function AddMacro(const aName, aValue: String; Quiet: boolean = false): boolean;
    function RemoveMacro(const aName: String; Quiet: boolean = false): boolean;
    procedure SetCompilerMode(S : String);
    function CurSourcePos: TPasSourcePos;
    function SetForceCaret(AValue : Boolean) : Boolean; // returns old state
    function IgnoreMsgType(MsgType: TMessageType): boolean; virtual;
    property FileResolver: TBaseFileResolver read FFileResolver;
    property Files: TStrings read FFiles;
    property CurSourceFile: TLineReader read FCurSourceFile;
    property CurFilename: string read FCurFilename;
    property CurLine: string read FCurLine;
    property CurRow: Integer read FCurRow;
    property CurColumn: Integer read GetCurColumn;
    property CurToken: TToken read FCurToken;
    property CurTokenString: string read FCurTokenString;
    property CurTokenPos: TPasSourcePos read FCurTokenPos;
    property PreviousToken : TToken Read FPreviousToken;
    property ModuleRow: Integer read FModuleRow;
    property NonTokens : TTokens Read FNonTokens;
    Property TokenOptions : TTokenOptions Read FTokenOptions Write FTokenOptions;
    property Defines: TStrings read FDefines;
    property Macros: TStrings read FMacros;
    property MacrosOn: boolean read GetMacrosOn write SetMacrosOn;
    property OnDirective: TPScannerDirectiveEvent read FOnDirective write FOnDirective;
    property AllowedModeSwitches: TModeSwitches read FAllowedModeSwitches Write SetAllowedModeSwitches;
    property ReadOnlyModeSwitches: TModeSwitches read FReadOnlyModeSwitches Write SetReadOnlyModeSwitches;// always set, cannot be disabled
    property CurrentModeSwitches: TModeSwitches read FCurrentModeSwitches Write SetCurrentModeSwitches;
    property AllowedBoolSwitches: TBoolSwitches read FAllowedBoolSwitches Write SetAllowedBoolSwitches;
    property ReadOnlyBoolSwitches: TBoolSwitches read FReadOnlyBoolSwitches Write SetReadOnlyBoolSwitches;// cannot be changed by code
    property CurrentBoolSwitches: TBoolSwitches read FCurrentBoolSwitches Write SetCurrentBoolSwitches;
    property AllowedValueSwitches: TValueSwitches read FAllowedValueSwitches Write SetAllowedValueSwitches;
    property ReadOnlyValueSwitches: TValueSwitches read FReadOnlyValueSwitches Write SetReadOnlyValueSwitches;// cannot be changed by code
    property CurrentValueSwitch[V: TValueSwitch]: string read GetCurrentValueSwitch Write SetCurrentValueSwitch;
    property WarnMsgState[Number: integer]: TWarnMsgState read GetWarnMsgState write SetWarnMsgState;
    property Options : TPOptions read FOptions write SetOptions;
    property SkipWhiteSpace : Boolean Read FSkipWhiteSpace Write FSkipWhiteSpace;
    property SkipComments : Boolean Read FSkipComments Write FSkipComments;
    property SkipGlobalSwitches: Boolean read FSkipGlobalSwitches write FSkipGlobalSwitches;
    property MaxIncludeStackDepth: integer read FMaxIncludeStackDepth write FMaxIncludeStackDepth default DefaultMaxIncludeStackDepth;
    property ForceCaret : Boolean read GetForceCaret;

    property LogEvents : TPScannerLogEvents read FLogEvents write FLogEvents;
    property OnLog : TPScannerLogHandler read FOnLog write FOnLog;
    property OnFormatPath: TPScannerFormatPathEvent read FOnFormatPath write FOnFormatPath;
    property ConditionEval: TCondDirectiveEvaluator read FConditionEval;
    property OnEvalVariable: TCEEvalVarEvent read FOnEvalVariable write FOnEvalVariable;
    property OnEvalFunction: TCEEvalFunctionEvent read FOnEvalFunction write FOnEvalFunction;
    property OnWarnDirective: TPScannerWarnEvent read FOnWarnDirective write FOnWarnDirective;
    property OnModeChanged: TPScannerModeDirective read FOnModeChanged write FOnModeChanged; // set by TPasParser

    property LastMsg: string read FLastMsg write FLastMsg;
    property LastMsgNumber: integer read FLastMsgNumber write FLastMsgNumber;
    property LastMsgType: TMessageType read FLastMsgType write FLastMsgType;
    property LastMsgPattern: string read FLastMsgPattern write FLastMsgPattern;
    property LastMsgArgs: TMessageArgs read FLastMsgArgs write FLastMsgArgs;
  end;

const
  TokenInfos: array[TToken] of string = (
    'EOF',
    'Whitespace',
    'Comment',
    'Identifier',
    'string',
    'Number',
    'Character',
    '(',
    ')',
    '*',
    '+',
    ',',
    '-',
    '.',
    '/',
    ':',
    ';',
    '<',
    '=',
    '>',
    '@',
    '[',
    ']',
    '^',
    '\',
    '..',
    ':=',
    '<>',
    '<=',
    '>=',
    '**',
    '><',
    '+=',
    '-=',
    '*=',
    '/=',
    '@@',
    // Reserved words
    'absolute',
    'and',
    'array',
    'as',
    'asm',
    'begin',
    'bitpacked',
    'case',
    'class',
    'const',
    'constref',
    'constructor',
    'destructor',
    'dispinterface',
    'div',
    'do',
    'downto',
    'else',
    'end',
    'except',
    'exports',
    'false',
    'file',
    'finalization',
    'finally',
    'for',
    'function',
    'generic',
    'goto',
    'if',
    'implementation',
    'in',
    'inherited',
    'initialization',
    'inline',
    'interface',
    'is',
    'label',
    'library',
    'mod',
    'nil',
    'not',
    'object',
    'of',
    'operator',
    'or',
    'packed',
    'procedure',
    'program',
    'property',
    'raise',
    'record',
    'repeat',
    'resourcestring',
    'self',
    'set',
    'shl',
    'shr',
    'specialize',
//    'string',
    'then',
    'threadvar',
    'to',
    'true',
    'try',
    'type',
    'unit',
    'until',
    'uses',
    'var',
    'while',
    'with',
    'xor',
    'LineEnding',
    'Tab'
  );

  SModeSwitchNames : array[TModeSwitch] of string =
  ( '', // msNone
    '', // Fpc,
    '', // Objfpc,
    '', // Delphi,
    '', // DelphiUnicode,
    '', // TP7,
    '', // Mac,
    '', // Iso,
    '', // Extpas,
    '', // GPC,
    { more specific }
    'CLASS',
    'OBJPAS',
    'RESULT',
    'PCHARTOSTRING',
    'CVAR',
    'NESTEDCOMMENTS',
    'CLASSICPROCVARS',
    'MACPROCVARS',
    'REPEATFORWARD',
    'POINTERTOPROCVAR',
    'AUTODEREF',
    'INITFINAL',
    'ANSISTRINGS',
    'OUT',
    'DEFAULTPARAMETERS',
    'HINTDIRECTIVE',
    'DUPLICATELOCALS',
    'PROPERTIES',
    'ALLOWINLINE',
    'EXCEPTIONS',
    'OBJECTIVEC1',
    'OBJECTIVEC2',
    'NESTEDPROCVARS',
    'NONLOCALGOTO',
    'ADVANCEDRECORDS',
    'ISOUNARYMINUS',
    'SYSTEMCODEPAGE',
    'FINALFIELDS',
    'UNICODESTRINGS',
    'TYPEHELPERS',
    'CBLOCKS',
    'ISOIO',
    'ISOPROGRAMPARAS',
    'ISOMOD',
    'ARRAYOPERATORS',
    'EXTERNALCLASS',
    'PREFIXEDATTRIBUTES',
    'OMITRTTI',
    'MULTIHELPERS',
    'IMPLICITFUNCTIONSPECIALIZATION'
    );

  LetterSwitchNames: array['A'..'Z'] of string=(
     'ALIGN'          // A   align fields
    ,'BOOLEVAL'       // B   complete boolean evaluation
    ,'ASSERTIONS'     // C   generate code for assertions
    ,'DEBUGINFO'      // D   generate debuginfo (debug lines), OR: $description 'text'
    ,'EXTENSION'      // E   output file extension
    ,''               // F
    ,'IMPORTEDDATA'   // G
    ,'LONGSTRINGS'    // H   String=AnsiString
    ,'IOCHECKS'       // I   generate EInOutError
    ,'WRITEABLECONST' // J   writable typed const
    ,''               // K
    ,'LOCALSYMBOLS'   // L   generate local symbol information (debug, requires $D+)
    ,'TYPEINFO'       // M   allow published members OR $M minstacksize,maxstacksize
    ,''               // N
    ,'OPTIMIZATION'   // O   enable safe optimizations (-O1)
    ,'OPENSTRINGS'    // P   deprecated Delphi directive
    ,'OVERFLOWCHECKS' // Q
    ,'RANGECHECKS'    // R   OR resource
    ,''               // S
    ,'TYPEDADDRESS'   // T   enabled: @variable gives typed pointer, otherwise untyped pointer
    ,'SAFEDIVIDE'     // U
    ,'VARSTRINGCHECKS'// V   strict shortstring checking, e.g. cannot pass shortstring[3] to shortstring
    ,'STACKFRAMES'    // W   always generate stackframes (debugging)
    ,'EXTENDEDSYNTAX' // X   deprecated Delphi directive
    ,'REFERENCEINFO'  // Y   store for each identifier the declaration location
    ,''               // Z
   );

  BoolSwitchNames: array[TBoolSwitch] of string = (
    // letter directives
    'None',
    'Align',
    'BoolEval',
    'Assertions',
    'DebugInfo',
    'Extension',
    'ImportedData',
    'LongStrings',
    'IOChecks',
    'WriteableConst',
    'LocalSymbols',
    'TypeInfo',
    'Optimization',
    'OpenStrings',
    'OverflowChecks',
    'RangeChecks',
    'TypedAddress',
    'SafeDivide',
    'VarStringChecks',
    'Stackframes',
    'ExtendedSyntax',
    'ReferenceInfo',
    // other bool directives
    'Hints',
    'Notes',
    'Warnings',
    'Macro',
    'ScopedEnums',
    'ObjectChecks',
    'PointerMath',
    'Goto'
    );

  ValueSwitchNames: array[TValueSwitch] of string = (
    'Interfaces', // vsInterfaces
    'DispatchField', // vsDispatchField
    'DispatchStrField' // vsDispatchStrField
    );

const
  AllLanguageModes = [msFPC,msObjFPC,msDelphi,msTP7,msMac,msISO,msExtPas];

const
  MessageTypeNames : Array[TMessageType] of string = (
    'Fatal','Error','Warning','Note','Hint','Info','Debug'
  );

const
  // all mode switches supported by FPC
  msAllModeSwitches = [low(TModeSwitch)..High(TModeSwitch)];

  DelphiModeSwitches = [msDelphi,msClass,msObjpas,msResult,msStringPchar,
     msPointer2Procedure,msAutoDeref,msTPProcVar,msInitFinal,msDefaultAnsistring,
     msOut,msDefaultPara,msDuplicateNames,msHintDirective,
     msProperty,msDefaultInline,msExcept,msAdvancedRecords,msTypeHelpers,
     msPrefixedAttributes,msArrayOperators,msImplicitFunctionSpec
     ];

  DelphiUnicodeModeSwitches = delphimodeswitches + [msSystemCodePage,msDefaultUnicodestring];

  // mode switches of $mode FPC, don't confuse with msAllModeSwitches
  FPCModeSwitches = [msFpc,msStringPchar,msNestedComment,msRepeatForward,
    msCVarSupport,msInitFinal,msHintDirective,msProperty,msDefaultInline];
  //FPCBoolSwitches bsObjectChecks

  OBJFPCModeSwitches =  [msObjfpc,msClass,msObjpas,msResult,msStringPchar,msNestedComment,
    msRepeatForward,msCVarSupport,msInitFinal,msOut,msDefaultPara,msHintDirective,
    msProperty,msDefaultInline,msExcept];

  TPModeSwitches = [msTP7,msTPProcVar,msDuplicateNames];

  GPCModeSwitches = [msGPC,msTPProcVar];

  MacModeSwitches = [msMac,msCVarSupport,msMacProcVar,msNestedProcVars,
    msNonLocalGoto,msISOLikeUnaryMinus,msDefaultInline];

  ISOModeSwitches =  [msIso,msTPProcVar,msDuplicateNames,msNestedProcVars,
    msNonLocalGoto,msISOLikeUnaryMinus,msISOLikeIO,msISOLikeProgramsPara,
    msISOLikeMod];

  ExtPasModeSwitches = [msExtpas,msTPProcVar,msDuplicateNames,msNestedProcVars,
    msNonLocalGoto,msISOLikeUnaryMinus,msISOLikeIO,msISOLikeProgramsPara,
    msISOLikeMod];

function StrToModeSwitch(aName: String): TModeSwitch;
function ModeSwitchesToStr(Switches: TModeSwitches): string;
function BoolSwitchesToStr(Switches: TBoolSwitches): string;

function FilenameIsAbsolute(const TheFilename: string):boolean;
function FilenameIsWinAbsolute(const TheFilename: string): boolean;
function FilenameIsUnixAbsolute(const TheFilename: string): boolean;
function IsNamedToken(Const AToken : String; Out T : TToken) : Boolean;
Function ExtractFilenameOnly(Const AFileName : String) : String;

procedure CreateMsgArgs(var MsgArgs: TMessageArgs; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
function SafeFormat(const Fmt: string; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif}): string;

implementation

const
  IdentChars = ['0'..'9', 'A'..'Z', 'a'..'z','_'];
  Digits = ['0'..'9'];
  Letters = ['a'..'z','A'..'Z'];
  HexDigits = ['0'..'9','a'..'f','A'..'F'];
Var
  SortedTokens : array of TToken;
  LowerCaseTokens  : Array[ttoken] of String;

Function ExtractFilenameOnly(Const AFileName : String) : String;

begin
  Result:=ChangeFileExt(ExtractFileName(aFileName),'');
end;


Procedure SortTokenInfo;

Var
  tk: tToken;
  I,J,K, l: integer;

begin
  for tk:=Low(TToken) to High(ttoken) do
    LowerCaseTokens[tk]:=LowerCase(TokenInfos[tk]);
  SetLength(SortedTokens,Ord(tkXor)-Ord(tkAbsolute)+1);
  I:=0;
  for tk := tkAbsolute to tkXOR do
    begin
    SortedTokens[i]:=tk;
    Inc(i);
    end;
  l:=Length(SortedTokens)-1;
  k:=l shr 1;
  while (k>0) do
    begin
    for i:=0 to l-k do
      begin
      j:=i;
      while (J>=0) and (LowerCaseTokens[SortedTokens[J]]>LowerCaseTokens[SortedTokens[J+K]]) do
        begin
        tk:=SortedTokens[J];
        SortedTokens[J]:=SortedTokens[J+K];
        SortedTokens[J+K]:=tk;
        if (J>K) then
          Dec(J,K)
        else
          J := 0
        end;
      end;
      K:=K shr 1;
    end;
end;

function IndexOfToken(Const AToken : string) : Integer;

var
  B,T,M : Integer;
  N : String;
begin
  B:=0;
  T:=Length(SortedTokens)-1;
  while (B<=T) do
    begin
    M:=(B+T) div 2;
    N:=LowerCaseTokens[SortedTokens[M]];
    if (AToken<N) then
      T:=M-1
    else if (AToken=N) then
      Exit(M)
    else
      B:=M+1;
    end;
  Result:=-1;
end;

function IsNamedToken(Const AToken : String; Out T : TToken) : Boolean;

Var
  I : Integer;

begin
  if (Length(SortedTokens)=0) then
    SortTokenInfo;
  I:=IndexOfToken(LowerCase(AToken));
  Result:=I<>-1;
  If Result then
    T:=SortedTokens[I];
end;

procedure CreateMsgArgs(var MsgArgs: TMessageArgs; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
var
  i: Integer;
  {$ifdef pas2js}
  v: jsvalue;
  {$endif}
begin
  SetLength(MsgArgs, High(Args)-Low(Args)+1);
  for i:=Low(Args) to High(Args) do
    {$ifdef pas2js}
    begin
    v:=Args[i];
    if isBoolean(v) then
      MsgArgs[i] := BoolToStr(Boolean(v))
    else if isString(v) then
      MsgArgs[i] := String(v)
    else if isNumber(v) then
      begin
      if IsInteger(v) then
        MsgArgs[i] := str(NativeInt(v))
      else
        MsgArgs[i] := str(double(v));
      end
    else
      MsgArgs[i]:='';
    end;
    {$else}
    case Args[i].VType of
      vtInteger:      MsgArgs[i] := IntToStr(Args[i].VInteger);
      vtBoolean:      MsgArgs[i] := BoolToStr(Args[i].VBoolean);
      vtChar:         MsgArgs[i] := Args[i].VChar;
      {$ifndef FPUNONE}
      vtExtended:     ; //  Args[i].VExtended^;
      {$ENDIF}
      vtString:       MsgArgs[i] := Args[i].VString^;
      vtPointer:      ; //  Args[i].VPointer;
      vtPChar:        MsgArgs[i] := Args[i].VPChar;
      vtObject:       ; //  Args[i].VObject;
      vtClass:        ; //  Args[i].VClass;
      vtWideChar:     MsgArgs[i] := AnsiString(Args[i].VWideChar);
      vtPWideChar:    MsgArgs[i] := Args[i].VPWideChar;
      vtAnsiString:   MsgArgs[i] := AnsiString(Args[i].VAnsiString);
      vtCurrency:     ; //  Args[i].VCurrency^);
      vtVariant:      ; //  Args[i].VVariant^);
      vtInterface:    ; //  Args[i].VInterface^);
      vtWidestring:   MsgArgs[i] := AnsiString(WideString(Args[i].VWideString));
      vtInt64:        MsgArgs[i] := IntToStr(Args[i].VInt64^);
      vtQWord:        MsgArgs[i] := IntToStr(Args[i].VQWord^);
      vtUnicodeString:MsgArgs[i] := AnsiString(UnicodeString(Args[i].VUnicodeString));
    end;
    {$endif}
end;

function SafeFormat(const Fmt: string; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif}): string;
var
  MsgArgs: TMessageArgs;
  i: Integer;
begin
  try
    Result:=Format(Fmt,Args);
  except
    Result:='';
    MsgArgs:=nil;
    CreateMsgArgs(MsgArgs,Args);
    for i:=0 to length(MsgArgs)-1 do
      begin
      if i>0 then
        Result:=Result+',';
      Result:=Result+MsgArgs[i];
      end;
    Result:='{'+Fmt+'}['+Result+']';
  end;
end;

type
  TIncludeStackItem = class
    SourceFile: TLineReader;
    Filename: string;
    Token: TToken;
    TokenString: string;
    Line: string;
    Row: Integer;
    ColumnOffset: integer;
    TokenPos: {$ifdef UsePChar}PChar;{$else}integer; { position in Line }{$endif}
  end;

function StrToModeSwitch(aName: String): TModeSwitch;
var
  ms: TModeSwitch;
begin
  aName:=UpperCase(aName);
  if aName='' then exit(msNone);
  for ms in TModeSwitch do
    if SModeSwitchNames[ms]=aName then exit(ms);
  Result:=msNone;
end;

function ModeSwitchesToStr(Switches: TModeSwitches): string;
var
  ms: TModeSwitch;
begin
  Result:='';
  for ms in Switches do
    Result:=Result+SModeSwitchNames[ms]+',';
  Result:='['+LeftStr(Result,length(Result)-1)+']';
end;

function BoolSwitchesToStr(Switches: TBoolSwitches): string;
var
  bs: TBoolSwitch;
begin
  Result:='';
  for bs in Switches do
    Result:=Result+BoolSwitchNames[bs]+',';
  Result:='['+LeftStr(Result,length(Result)-1)+']';
end;

function FilenameIsAbsolute(const TheFilename: string):boolean;
begin
  {$IFDEF WINDOWS}
  // windows
  Result:=FilenameIsWinAbsolute(TheFilename);
  {$ELSE}
  // unix
  Result:=FilenameIsUnixAbsolute(TheFilename);
  {$ENDIF}
end;

function FilenameIsWinAbsolute(const TheFilename: string): boolean;
begin
  Result:=((length(TheFilename)>=2) and (TheFilename[1] in ['A'..'Z','a'..'z'])
           and (TheFilename[2]=':'))
     or ((length(TheFilename)>=2)
         and (TheFilename[1]='\') and (TheFilename[2]='\'));
end;

function FilenameIsUnixAbsolute(const TheFilename: string): boolean;
begin
  Result:=(TheFilename<>'') and (TheFilename[1]='/');
end;

{ TCondDirectiveEvaluator }

// inline
function TCondDirectiveEvaluator.IsFalse(const Value: String): boolean;
begin
  Result:=Value=CondDirectiveBool[false];
end;

// inline
function TCondDirectiveEvaluator.IsTrue(const Value: String): boolean;
begin
  Result:=Value<>CondDirectiveBool[false];
end;

function TCondDirectiveEvaluator.IsInteger(const Value: String; out i: TMaxPrecInt
  ): boolean;
var
  Code: integer;
begin
  val(Value,i,Code);
  Result:=Code=0;
end;

function TCondDirectiveEvaluator.IsExtended(const Value: String; out e: TMaxFloat
  ): boolean;
var
  Code: integer;
begin
  val(Value,e,Code);
  Result:=Code=0;
end;

procedure TCondDirectiveEvaluator.NextToken;
const
  IdentChars = ['a'..'z','A'..'Z','_','0'..'9'];

  {$ifdef UsePChar}
  function IsIdentifier(a,b: PChar): boolean;
  var
    ac: Char;
  begin
    repeat
      ac:=a^;
      if (ac in IdentChars) and (upcase(ac)=upcase(b^)) then
        begin
        inc(a);
        inc(b);
        end
      else
        begin
        Result:=(not (ac in IdentChars)) and (not (b^ in IdentChars));
        exit;
        end;
    until false;
  end;
  {$endif}

  function ReadIdentifier: TToken;
  begin
    Result:=tkIdentifier;
    {$ifdef UsePChar}
    case FTokenEnd-FTokenStart of
    2:
      if IsIdentifier(FTokenStart,'or') then
        Result:=tkor;
    3:
      if IsIdentifier(FTokenStart,'not') then
        Result:=tknot
      else if IsIdentifier(FTokenStart,'and') then
        Result:=tkand
      else if IsIdentifier(FTokenStart,'xor') then
        Result:=tkxor
      else if IsIdentifier(FTokenStart,'shl') then
        Result:=tkshl
      else if IsIdentifier(FTokenStart,'shr') then
        Result:=tkshr
      else if IsIdentifier(FTokenStart,'mod') then
        Result:=tkmod
      else if IsIdentifier(FTokenStart,'div') then
        Result:=tkdiv;
    end;
    {$else}
    case lowercase(copy(Expression,FTokenStart,FTokenEnd-FTokenStart)) of
    'or': Result:=tkor;
    'not': Result:=tknot;
    'and': Result:=tkand;
    'xor': Result:=tkxor;
    'shl': Result:=tkshl;
    'shr': Result:=tkshr;
    'mod': Result:=tkmod;
    'div': Result:=tkdiv;
    end;
    {$endif}
  end;

{$ifndef UsePChar}
const
  AllSpaces = [#9,#10,#13,' '];
  Digits = ['0'..'9'];
  HexDigits = ['0'..'9'];
var
  l: integer;
  Src: String;
{$endif}
begin
  FTokenStart:=FTokenEnd;

  // skip white space
  {$ifdef UsePChar}
  repeat
    case FTokenStart^ of
      #0:
      if FTokenStart-PChar(Expression)>=length(Expression) then
        begin
        FToken:=tkEOF;
        FTokenEnd:=FTokenStart;
        exit;
        end
      else
        inc(FTokenStart);
      #9,#10,#13,' ':
        inc(FTokenStart);
      else break;
    end;
  until false;
  {$else}
  Src:=Expression;
  l:=length(Src);
  while (FTokenStart<=l) and (Src[FTokenStart] in AllSpaces) do
    inc(FTokenStart);
  if FTokenStart>l then
    begin
    FToken:=tkEOF;
    FTokenEnd:=FTokenStart;
    exit;
    end;
  {$endif}

  // read token
  FTokenEnd:=FTokenStart;
  case {$ifdef UsePChar}FTokenEnd^{$else}Src[FTokenEnd]{$endif} of
  'a'..'z','A'..'Z','_':
    begin
    inc(FTokenEnd);
    {$ifdef UsePChar}
    while FTokenEnd^ in IdentChars do inc(FTokenEnd);
    {$else}
    while (FTokenEnd<=l) and (Src[FTokenEnd] in IdentChars) do inc(FTokenEnd);
    {$endif}
    FToken:=ReadIdentifier;
    end;
  '0'..'9':
    begin
    FToken:=tkNumber;
    // examples: 1, 1.2, 1.2E3, 1E-2
    inc(FTokenEnd);
    {$ifdef UsePChar}
    while FTokenEnd^ in Digits do inc(FTokenEnd);
    if (FTokenEnd^='.') and (FTokenEnd[1]<>'.') then
      begin
      inc(FTokenEnd);
      while FTokenEnd^ in Digits do inc(FTokenEnd);
      end;
    if FTokenEnd^ in ['e','E'] then
      begin
      inc(FTokenEnd);
      if FTokenEnd^ in ['-','+'] then inc(FTokenEnd);
      while FTokenEnd^ in Digits do inc(FTokenEnd);
      end;
    {$else}
    while (FTokenEnd<=l) and (Src[FTokenEnd] in Digits) do inc(FTokenEnd);
    if (FTokenEnd<=l) and (Src[FTokenEnd]='.')
        and ((FTokenEnd=l) or (Src[FTokenEnd+1]<>'.')) then
      begin
      inc(FTokenEnd);
      while (FTokenEnd<=l) and (Src[FTokenEnd] in Digits) do inc(FTokenEnd);
      end;
    if (FTokenEnd<=l) and (Src[FTokenEnd] in ['e','E']) then
      begin
      inc(FTokenEnd);
      if (FTokenEnd<=l) and (Src[FTokenEnd] in ['-','+']) then inc(FTokenEnd);
      while (FTokenEnd<=l) and (Src[FTokenEnd] in Digits) do inc(FTokenEnd);
      end;
    {$endif}
    end;
  '$':
    begin
    FToken:=tkNumber;
    {$ifdef UsePChar}
    while FTokenEnd^ in HexDigits do inc(FTokenEnd);
    {$else}
    while (FTokenEnd<=l) and (Src[FTokenEnd] in HexDigits) do inc(FTokenEnd);
    {$endif}
    end;
  '%':
    begin
    FToken:=tkNumber;
    {$ifdef UsePChar}
    while FTokenEnd^ in ['0','1'] do inc(FTokenEnd);
    {$else}
    while (FTokenEnd<=l) and (Src[FTokenEnd] in ['0','1']) do inc(FTokenEnd);
    {$endif}
    end;
  '(':
    begin
    FToken:=tkBraceOpen;
    inc(FTokenEnd);
    end;
  ')':
    begin
    FToken:=tkBraceClose;
    inc(FTokenEnd);
    end;
  '=':
    begin
    FToken:=tkEqual;
    inc(FTokenEnd);
    end;
  '<':
    begin
    inc(FTokenEnd);
    case {$ifdef UsePChar}FTokenEnd^{$else}copy(Src,FTokenEnd,1){$endif} of
    '=':
      begin
      FToken:=tkLessEqualThan;
      inc(FTokenEnd);
      end;
    '<':
      begin
      FToken:=tkshl;
      inc(FTokenEnd);
      end;
    '>':
      begin
      FToken:=tkNotEqual;
      inc(FTokenEnd);
      end;
    else
      FToken:=tkLessThan;
    end;
    end;
  '>':
    begin
    inc(FTokenEnd);
    case {$ifdef UsePChar}FTokenEnd^{$else}copy(Src,FTokenEnd,1){$endif} of
    '=':
      begin
      FToken:=tkGreaterEqualThan;
      inc(FTokenEnd);
      end;
    '>':
      begin
      FToken:=tkshr;
      inc(FTokenEnd);
      end;
    else
      FToken:=tkGreaterThan;
    end;
    end;
  '+':
    begin
    FToken:=tkPlus;
    inc(FTokenEnd);
    end;
  '-':
    begin
    FToken:=tkMinus;
    inc(FTokenEnd);
    end;
  '*':
    begin
    FToken:=tkMul;
    inc(FTokenEnd);
    end;
  '/':
    begin
    FToken:=tkDivision;
    inc(FTokenEnd);
    end;
  '''':
    begin
    FToken:=tkString;
    repeat
      inc(FTokenEnd);
      {$ifdef UsePChar}
      if FTokenEnd^='''' then
        begin
        inc(FTokenEnd);
        if FTokenEnd^<>'''' then break;
        end
      else if FTokenEnd^ in [#0,#10,#13] then
        Log(mtError,nErrOpenString,SErrOpenString,[]);
      {$else}
      if FTokenEnd>l then
        Log(mtError,nErrOpenString,SErrOpenString,[]);
      case Src[FTokenEnd] of
      '''':
        begin
        inc(FTokenEnd);
        if (FTokenEnd>l) or (Src[FTokenEnd]<>'''') then break;
        end;
      #10,#13:
        Log(mtError,nErrOpenString,SErrOpenString,[]);
      end;
      {$endif}
    until false;
    end
  else
    FToken:=tkEOF;
  end;
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.NextToken END Token[',FTokenStart-PChar(Expression)+1,']="',GetTokenString,'" ',FToken);
  {$ENDIF}
end;

procedure TCondDirectiveEvaluator.Log(aMsgType: TMessageType;
  aMsgNumber: integer; const aMsgFmt: String;
  const Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
  MsgPos: integer);
begin
  if MsgPos<1 then
    MsgPos:=FTokenEnd{$ifdef UsePChar}-PChar(Expression)+1{$endif};
  MsgType:=aMsgType;
  MsgNumber:=aMsgNumber;
  MsgPattern:=aMsgFmt;
  if Assigned(OnLog) then
    begin
    OnLog(Self,Args);
    if not (aMsgType in [mtError,mtFatal]) then exit;
    end;
  raise EScannerError.CreateFmt(MsgPattern+' at '+IntToStr(MsgPos),Args);
end;

procedure TCondDirectiveEvaluator.LogXExpectedButTokenFound(const X: String;
  ErrorPos: integer);
begin
  Log(mtError,nErrXExpectedButYFound,SErrXExpectedButYFound,
      [X,TokenInfos[FToken]],ErrorPos);
end;

procedure TCondDirectiveEvaluator.ReadOperand(Skip: boolean);
{ Read operand and put it on the stack
  Examples:
   Variable
   not Variable
   not not undefined Variable
   defined(Variable)
   !Variable
   unicodestring
   123
   $45
   'Abc'
   (expression)
}
var
  i: TMaxPrecInt;
  e: extended;
  S, aName, Param: String;
  Code: integer;
  NameStartP: {$ifdef UsePChar}PChar{$else}integer{$endif};
  p, Lvl: integer;
begin
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.ReadOperand START Token[',FTokenStart-PChar(Expression)+1,']="',GetTokenString,'" ',FToken,BoolToStr(Skip,' SKIP',''));
  {$ENDIF}
  case FToken of
    tknot:
      begin
      // boolean not
      NextToken;
      ReadOperand(Skip);
      if not Skip then
        FStack[FStackTop].Operand:=CondDirectiveBool[IsFalse(FStack[FStackTop].Operand)];
      end;
    tkMinus:
      begin
      // unary minus
      NextToken;
      ReadOperand(Skip);
      if not Skip then
        begin
        i:=StrToInt64Def(FStack[FStackTop].Operand,0);
        FStack[FStackTop].Operand:=IntToStr(-i);
        end;
      end;
    tkPlus:
      begin
      // unary plus
      NextToken;
      ReadOperand(Skip);
      if not Skip then
        begin
        i:=StrToInt64Def(FStack[FStackTop].Operand,0);
        FStack[FStackTop].Operand:=IntToStr(i);
        end;
      end;
    tkNumber:
      begin
      // number: convert to decimal
      if not Skip then
        begin
        S:=GetTokenString;
        val(S,i,Code);
        if Code=0 then
          begin
          // integer
          Push(IntToStr(i),FTokenStart{$ifdef UsePChar}-PChar(Expression)+1{$endif});
          end
        else
          begin
          val(S,e,Code);
          if Code>0 then
            Log(mtError,nErrRangeCheck,sErrRangeCheck,[]);
          if e=0 then ;
          // float
          Push(S,FTokenStart{$ifdef UsePChar}-PChar(Expression)+1{$endif});
          end;
        end;
      NextToken;
      end;
    tkString:
      begin
      // string literal
      if not Skip then
        Push(GetStringLiteralValue,FTokenStart{$ifdef UsePChar}-PChar(Expression)+1{$endif});
      NextToken;
      end;
    tkIdentifier:
      if Skip then
        begin
        NextToken;
        if FToken=tkBraceOpen then
          begin
          // only one parameter is supported
          NextToken;
          if FToken=tkIdentifier then
            NextToken;
          if FToken<>tkBraceClose then
            LogXExpectedButTokenFound(')');
          NextToken;
          end;
        end
      else
        begin
        aName:=GetTokenString;
        p:=FTokenStart{$ifdef UsePChar}-PChar(Expression)+1{$endif};
        NextToken;
        if FToken=tkBraceOpen then
          begin
          // function
          NameStartP:=FTokenStart;
          NextToken;
          // only one parameter is supported
          Param:='';
          if FToken=tkIdentifier then
            begin
            Param:=GetTokenString;
            NextToken;
            end;
          if FToken<>tkBraceClose then
            LogXExpectedButTokenFound(')');
          if not OnEvalFunction(Self,aName,Param,S) then
            begin
            FTokenStart:=NameStartP;
            FTokenEnd:=FTokenStart+length(aName);
            LogXExpectedButTokenFound('function');
            end;
          Push(S,p);
          NextToken;
          end
        else
          begin
          // variable
          if OnEvalVariable(Self,aName,S) then
            Push(S,p)
          else
            begin
            // variable does not exist -> evaluates to false
            Push(CondDirectiveBool[false],p);
            end;
          end;
        end;
    tkBraceOpen:
      begin
      NextToken;
      if Skip then
        begin
        Lvl:=1;
        repeat
          case FToken of
          tkEOF:
            LogXExpectedButTokenFound(')');
          tkBraceOpen: inc(Lvl);
          tkBraceClose:
            begin
            dec(Lvl);
            if Lvl=0 then break;
            end;
          end;
          NextToken;
        until false;
        end
      else
        begin
        ReadExpression;
        if FToken<>tkBraceClose then
          LogXExpectedButTokenFound(')');
        end;
      NextToken;
      end;
  else
    LogXExpectedButTokenFound('identifier');
  end;
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.ReadOperand END Top=',FStackTop,' Value="',FStack[FStackTop].Operand,'" Token[',FTokenStart-PChar(Expression)+1,']="',GetTokenString,'" ',FToken);
  {$ENDIF}
end;

procedure TCondDirectiveEvaluator.ReadExpression;
// read operand operator operand ... til tkEOF or tkBraceClose
var
  OldStackTop: Integer;

  procedure ReadBinary(Level: TPrecedenceLevel; NewOperator: TToken);
  begin
    ResolveStack(OldStackTop,Level,NewOperator);
    NextToken;
    ReadOperand;
  end;

begin
  OldStackTop:=FStackTop;
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.ReadExpression START Top=',FStackTop,' Token[',FTokenStart-PChar(Expression)+1,']="',GetTokenString,'" ',FToken);
  {$ENDIF}
  ReadOperand;
  repeat
    {$IFDEF VerbosePasDirectiveEval}
    writeln('TCondDirectiveEvaluator.ReadExpression NEXT Top=',FStackTop,' Token[',FTokenStart-PChar(Expression)+1,']="',GetTokenString,'" ',FToken);
    {$ENDIF}
    case FToken of
    tkEOF,tkBraceClose:
      begin
      ResolveStack(OldStackTop,high(TPrecedenceLevel),tkEOF);
      exit;
      end;
    tkand:
      begin
      ResolveStack(OldStackTop,ceplSecond,tkand);
      NextToken;
      if (FStackTop=OldStackTop+1) and IsFalse(FStack[FStackTop].Operand) then
        begin
        // false and ...
        // -> skip all "and"
        repeat
          ReadOperand(true);
          if FToken<>tkand then break;
          NextToken;
        until false;
        FStack[FStackTop].Operathor:=tkEOF;
        end
      else
        ReadOperand;
      end;
    tkMul,tkDivision,tkdiv,tkmod,tkshl,tkshr:
      ReadBinary(ceplSecond,FToken);
    tkor:
      begin
      ResolveStack(OldStackTop,ceplThird,tkor);
      NextToken;
      if (FStackTop=OldStackTop+1) and IsTrue(FStack[FStackTop].Operand) then
        begin
        // true or ...
        // -> skip all "and" and "or"
        repeat
          ReadOperand(true);
          if not (FToken in [tkand,tkor]) then break;
          NextToken;
        until false;
        FStack[FStackTop].Operathor:=tkEOF;
        end
      else
        ReadOperand;
      end;
    tkPlus,tkMinus,tkxor:
      ReadBinary(ceplThird,FToken);
    tkEqual,tkNotEqual,tkLessThan,tkGreaterThan,tkLessEqualThan,tkGreaterEqualThan:
      ReadBinary(ceplFourth,FToken);
    else
      LogXExpectedButTokenFound('operator');
    end;
  until false;
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.ReadExpression END Top=',FStackTop,' Value="',FStack[FStackTop].Operand,'" Token[',FTokenStart-PChar(Expression)+1,']=',GetTokenString,' ',FToken);
  {$ENDIF}
end;

procedure TCondDirectiveEvaluator.ResolveStack(MinStackLvl: integer;
  Level: TPrecedenceLevel; NewOperator: TToken);
var
  A, B, R: String;
  Op: TToken;
  AInt, BInt: TMaxPrecInt;
  AFloat, BFloat: extended;
  BPos: Integer;
begin
  // resolve all higher or equal level operations
  // Note: the stack top contains operand B
  //       the stack second contains operand A and the operator between A and B

  //writeln('TCondDirectiveEvaluator.ResolveStack FStackTop=',FStackTop,' MinStackLvl=',MinStackLvl);
  //if FStackTop>MinStackLvl+1 then
  //  writeln('  FStack[FStackTop-1].Level=',FStack[FStackTop-1].Level,' Level=',Level);
  while (FStackTop>MinStackLvl+1) and (FStack[FStackTop-1].Level<=Level) do
    begin
    // pop last operand and operator from stack
    B:=FStack[FStackTop].Operand;
    BPos:=FStack[FStackTop].OperandPos;
    dec(FStackTop);
    Op:=FStack[FStackTop].Operathor;
    A:=FStack[FStackTop].Operand;
    {$IFDEF VerbosePasDirectiveEval}
    writeln('  ResolveStack Top=',FStackTop,' A="',A,'" ',Op,' B="',B,'"');
    {$ENDIF}
    {$IFOPT R+}{$DEFINE RangeChecking}{$ENDIF}
    {$R+}
    try
      case Op of
      tkand: // boolean and
        R:=CondDirectiveBool[IsTrue(A) and IsTrue(B)];
      tkor: // boolean or
        R:=CondDirectiveBool[IsTrue(A) or IsTrue(B)];
      tkxor: // boolean xor
        R:=CondDirectiveBool[IsTrue(A) xor IsTrue(B)];
      tkMul, tkdiv, tkmod, tkshl, tkshr, tkPlus, tkMinus:
        if IsInteger(A,AInt) then
          begin
          if IsInteger(B,BInt) then
            case Op of
              tkMul: R:=IntToStr(AInt*BInt);
              tkdiv: R:=IntToStr(AInt div BInt);
              tkmod: R:=IntToStr(AInt mod BInt);
              tkshl: R:=IntToStr(AInt shl BInt);
              tkshr: R:=IntToStr(AInt shr BInt);
              tkPlus: R:=IntToStr(AInt+BInt);
              tkMinus: R:=IntToStr(AInt-BInt);
            end
          else if IsExtended(B,BFloat) then
            case Op of
              tkMul: R:=FloatToStr(Extended(AInt)*BFloat);
              tkPlus: R:=FloatToStr(Extended(AInt)+BFloat);
              tkMinus: R:=FloatToStr(Extended(AInt)-BFloat);
            else
              LogXExpectedButTokenFound('integer',BPos);
            end
          else
            LogXExpectedButTokenFound('integer',BPos);
          end
        else if IsExtended(A,AFloat) then
          begin
          if IsExtended(B,BFloat) then
            case Op of
              tkMul: R:=FloatToStr(AFloat*BFloat);
              tkPlus: R:=FloatToStr(AFloat+BFloat);
              tkMinus: R:=FloatToStr(AFloat-BFloat);
            else
              LogXExpectedButTokenFound('float',BPos);
            end
          else
            LogXExpectedButTokenFound('float',BPos);
          end
        else
          Log(mtError,nErrOperandAndOperatorMismatch,sErrOperandAndOperatorMismatch,[]);
      tkDivision:
        if IsExtended(A,AFloat) then
          begin
          if IsExtended(B,BFloat) then
            R:=FloatToStr(AFloat/BFloat)
          else
            LogXExpectedButTokenFound('float',BPos);
          end
        else
          Log(mtError,nErrOperandAndOperatorMismatch,sErrOperandAndOperatorMismatch,[]);
      tkEqual,
      tkNotEqual,
      tkLessThan,tkGreaterThan,
      tkLessEqualThan,tkGreaterEqualThan:
        begin
        if IsInteger(A,AInt) and IsInteger(B,BInt) then
          case Op of
          tkEqual: R:=CondDirectiveBool[AInt=BInt];
          tkNotEqual: R:=CondDirectiveBool[AInt<>BInt];
          tkLessThan: R:=CondDirectiveBool[AInt<BInt];
          tkGreaterThan: R:=CondDirectiveBool[AInt>BInt];
          tkLessEqualThan: R:=CondDirectiveBool[AInt<=BInt];
          tkGreaterEqualThan: R:=CondDirectiveBool[AInt>=BInt];
          end
        else if IsExtended(A,AFloat) and IsExtended(B,BFloat) then
          case Op of
          tkEqual: R:=CondDirectiveBool[AFloat=BFloat];
          tkNotEqual: R:=CondDirectiveBool[AFloat<>BFloat];
          tkLessThan: R:=CondDirectiveBool[AFloat<BFloat];
          tkGreaterThan: R:=CondDirectiveBool[AFloat>BFloat];
          tkLessEqualThan: R:=CondDirectiveBool[AFloat<=BFloat];
          tkGreaterEqualThan: R:=CondDirectiveBool[AFloat>=BFloat];
          end
        else
          case Op of
          tkEqual: R:=CondDirectiveBool[A=B];
          tkNotEqual: R:=CondDirectiveBool[A<>B];
          tkLessThan: R:=CondDirectiveBool[A<B];
          tkGreaterThan: R:=CondDirectiveBool[A>B];
          tkLessEqualThan: R:=CondDirectiveBool[A<=B];
          tkGreaterEqualThan: R:=CondDirectiveBool[A>=B];
          end;
        end;
      else
        Log(mtError,nErrOperandAndOperatorMismatch,sErrOperandAndOperatorMismatch,[]);
      end;
    except
      on E: EDivByZero do
        Log(mtError,nErrDivByZero,sErrDivByZero,[]);
      on E: EZeroDivide do
        Log(mtError,nErrDivByZero,sErrDivByZero,[]);
      on E: EMathError do
        Log(mtError,nErrRangeCheck,sErrRangeCheck+' '+E.Message,[]);
      on E: EInterror do
        Log(mtError,nErrRangeCheck,sErrRangeCheck+' '+E.Message,[]);
    end;
    {$IFNDEF RangeChecking}{$R-}{$UNDEF RangeChecking}{$ENDIF}
    {$IFDEF VerbosePasDirectiveEval}
    writeln('  ResolveStack Top=',FStackTop,' A="',A,'" ',Op,' B="',B,'" = "',R,'"');
    {$ENDIF}
    FStack[FStackTop].Operand:=R;
    FStack[FStackTop].OperandPos:=BPos;
    end;
  FStack[FStackTop].Operathor:=NewOperator;
  FStack[FStackTop].Level:=Level;
end;

function TCondDirectiveEvaluator.GetTokenString: String;
begin
  Result:=copy(Expression,FTokenStart{$ifdef UsePChar}-PChar(Expression)+1{$endif},
               FTokenEnd-FTokenStart);
end;

function TCondDirectiveEvaluator.GetStringLiteralValue: String;
var
  {$ifdef UsePChar}
  p, StartP: PChar;
  {$else}
  Src: string;
  p, l, StartP: Integer;
  {$endif}
begin
  Result:='';
  p:=FTokenStart;
  {$ifdef UsePChar}
  repeat
    case p^ of
    '''':
      begin
      inc(p);
      StartP:=p;
      repeat
        case p^ of
        #0: Log(mtError,nErrInvalidCharacter,SErrInvalidCharacter,['#0']);
        '''': break;
        else inc(p);
        end;
      until false;
      if p>StartP then
        Result:=Result+copy(Expression,StartP-PChar(Expression)+1,p-StartP);
      inc(p);
      end;
    else
      Log(mtError,nErrInvalidCharacter,SErrInvalidCharacter,['#0']);
    end;
  until false;
  {$else}
  Src:=Expression;
  l:=length(Src);
  repeat
    if (p>l) or (Src[p]<>'''') then
      Log(mtError,nErrInvalidCharacter,SErrInvalidCharacter,['#0'])
    else
      begin
      inc(p);
      StartP:=p;
      repeat
        if p>l then
          Log(mtError,nErrInvalidCharacter,SErrInvalidCharacter,['#0'])
        else if Src[p]='''' then
          break
        else
          inc(p);
      until false;
      if p>StartP then
        Result:=Result+copy(Expression,StartP,p-StartP);
      inc(p);
      end;
  until false;
  {$endif}
end;

procedure TCondDirectiveEvaluator.Push(const AnOperand: String;
  OperandPosition: integer);
begin
  inc(FStackTop);
  if FStackTop>=length(FStack) then
    SetLength(FStack,length(FStack)*2+4);
  with FStack[FStackTop] do
    begin
    Operand:=AnOperand;
    OperandPos:=OperandPosition;
    Operathor:=tkEOF;
    Level:=ceplFourth;
    end;
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.Push Top=',FStackTop,' Operand="',AnOperand,'" Pos=',OperandPosition);
  {$ENDIF}
end;

constructor TCondDirectiveEvaluator.Create;
begin

end;

destructor TCondDirectiveEvaluator.Destroy;
begin
  inherited Destroy;
end;

function TCondDirectiveEvaluator.Eval(const Expr: string): boolean;
begin
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TCondDirectiveEvaluator.Eval Expr="',Expr,'"');
  {$ENDIF}
  Expression:=Expr;
  MsgType:=mtInfo;
  MsgNumber:=0;
  MsgPattern:='';
  if Expr='' then exit(false);
  FTokenStart:={$ifdef UsePChar}PChar(Expr){$else}1{$endif};
  FTokenEnd:=FTokenStart;
  FStackTop:=-1;
  NextToken;
  ReadExpression;
  Result:=IsTrue(FStack[0].Operand);
end;

{ TMacroDef }

constructor TMacroDef.Create(const AName, AValue: String);
begin
  FName:=AName;
  FValue:=AValue;
end;

{ TLineReader }

constructor TLineReader.Create(const AFilename: string);
begin
  FFileName:=AFileName;
end;

{ ---------------------------------------------------------------------
  TFileLineReader
  ---------------------------------------------------------------------}

constructor TFileLineReader.Create(const AFilename: string);

begin
  inherited Create(AFileName);
  {$ifdef pas2js}
  raise Exception.Create('ToDo TFileLineReader.Create');
  {$else}
  Assign(FTextFile, AFilename);
  Reset(FTextFile);
  SetTextBuf(FTextFile,FBuffer,SizeOf(FBuffer));
  FFileOpened := true;
  {$endif}
end;

destructor TFileLineReader.Destroy;
begin
  {$ifdef pas2js}
  // ToDo
  {$else}
  if FFileOpened then
    Close(FTextFile);
  {$endif}
  inherited Destroy;
end;

function TFileLineReader.IsEOF: Boolean;
begin
  {$ifdef pas2js}
  Result:=true;// ToDo
  {$else}
  Result := EOF(FTextFile);
  {$endif}
end;

function TFileLineReader.ReadLine: string;
begin
  {$ifdef pas2js}
  Result:='';// ToDo
  {$else}
  ReadLn(FTextFile, Result);
  {$endif}
end;

{ TStreamLineReader }

{$ifdef HasStreams}
Procedure TStreamLineReader.InitFromStream(AStream : TStream);

begin
  SetLength(FContent,AStream.Size);
  if FContent<>'' then
    AStream.Read(FContent[1],length(FContent));
  FPos:=0;
end;
{$endif}

procedure TStreamLineReader.InitFromString(const s: string);
begin
  FContent:=s;
  FPos:=0;
end;

function TStreamLineReader.IsEOF: Boolean;
begin
  Result:=FPos>=Length(FContent);
end;

function TStreamLineReader.ReadLine: string;

Var
  LPos : Integer;
  EOL : Boolean;

begin
  If isEOF then
    exit('');
  LPos:=FPos+1;
  Repeat
    Inc(FPos);
    EOL:=(FContent[FPos] in [#10,#13]);
  until isEOF or EOL;
  If EOL then
    Result:=Copy(FContent,LPos,FPos-LPos)
  else
    Result:=Copy(FContent,LPos,FPos-LPos+1);
  If (not isEOF) and (FContent[FPos]=#13) and (FContent[FPos+1]=#10) then
    inc(FPos);
end;

{ TFileStreamLineReader }

constructor TFileStreamLineReader.Create(const AFilename: string);
{$ifdef HasStreams}
Var
  S : TFileStream;
{$endif}
begin
  inherited Create(AFilename);
  {$ifdef HasStreams}
  S:=TFileStream.Create(AFileName,fmOpenRead or fmShareDenyWrite);
  try
     InitFromStream(S);
  finally
    S.Free;
  end;
  {$else}
  raise Exception.Create('TFileStreamLineReader.Create');
  {$endif}
end;

{ TStringStreamLineReader }

constructor TStringStreamLineReader.Create(const AFilename: string; const ASource: String);
begin
  inherited Create(AFilename);
  InitFromString(ASource);
end;

{ ---------------------------------------------------------------------
  TBaseFileResolver
  ---------------------------------------------------------------------}

procedure TBaseFileResolver.SetBaseDirectory(AValue: string);
begin
  if FBaseDirectory=AValue then Exit;
  FBaseDirectory:=AValue;
end;

procedure TBaseFileResolver.SetStrictFileCase(AValue: Boolean);
begin
  if FStrictFileCase=AValue then Exit;
  FStrictFileCase:=AValue;
end;


constructor TBaseFileResolver.Create;
begin
  inherited Create;
  FIncludePaths := TStringList.Create;
  FResourcePaths := TStringList.Create;
end;

destructor TBaseFileResolver.Destroy;
begin
  FResourcePaths.Free;
  FIncludePaths.Free;
  inherited Destroy;
end;

procedure TBaseFileResolver.AddIncludePath(const APath: string);

Var
  FP : String;

begin
  if (APath='') then
    FIncludePaths.Add('./')
  else
    begin
{$IFDEF HASFS}
    FP:=IncludeTrailingPathDelimiter(ExpandFileName(APath));
{$ELSE}
    FP:=APath;
{$ENDIF}
    FIncludePaths.Add(FP);
    end;
end;

procedure TBaseFileResolver.AddResourcePath(const APath: string);
Var
  FP : String;

begin
  if (APath='') then
    FResourcePaths.Add('./')
  else
    begin
{$IFDEF HASFS}
    FP:=IncludeTrailingPathDelimiter(ExpandFileName(APath));
{$ELSE}
    FP:=APath;
{$ENDIF}
    FResourcePaths.Add(FP);
    end;
end;


{$IFDEF HASFS}

{ ---------------------------------------------------------------------
  TFileResolver
  ---------------------------------------------------------------------}


function TFileResolver.SearchLowUpCase(FN: string): string;

var
  Dir: String;

begin
  If FileExists(FN) then
    Result:=FN
  else if StrictFileCase then
    Result:=''
  else
    begin
    Dir:=ExtractFilePath(FN);
    FN:=ExtractFileName(FN);
    Result:=Dir+LowerCase(FN);
    If FileExists(Result) then exit;
    Result:=Dir+uppercase(Fn);
    If FileExists(Result) then exit;
    Result:='';
    end;
end;

function TFileResolver.FindIncludeFileName(const AName: string): String;


  Function FindInPath(FN : String) : String;

  var
    I : integer;

  begin
    Result:='';
    I:=0;
    While (Result='') and (I<FIncludePaths.Count) do
      begin
      Result:=SearchLowUpCase(FIncludePaths[i]+FN);
      Inc(I);
      end;
    // search in BaseDirectory
    if (Result='') and (BaseDirectory<>'') then
      Result:=SearchLowUpCase(BaseDirectory+FN);
  end;

var
  FN : string;

begin
  Result := '';
  // convert pathdelims to system
  FN:=SetDirSeparators(AName);
  If FilenameIsAbsolute(FN) then
    begin
    Result := SearchLowUpCase(FN);
    if (Result='') and (ExtractFileExt(FN)='') then
      begin
      Result:=SearchLowUpCase(FN+'.inc');
      if Result='' then
        begin
        Result:=SearchLowUpCase(FN+'.pp');
        if Result='' then
          Result:=SearchLowUpCase(FN+'.pas');
        end;
      end;
    end
  else
    begin
    // file name is relative
    // search in include path
    Result:=FindInPath(FN);
    // No extension, try default extensions
    if (Result='') and (ExtractFileExt(FN)='') then
      begin
      Result:=FindInPath(FN+'.inc');
      if Result='' then
        begin
        Result:=FindInPath(FN+'.pp');
        if Result='' then
          Result:=FindInPath(FN+'.pas');
        end;
      end;
    end;
end;

function TFileResolver.CreateFileReader(const AFileName: String): TLineReader;
begin
  {$ifdef HasStreams}
  If UseStreams then
    Result:=TFileStreamLineReader.Create(AFileName)
  else
  {$endif}
    Result:=TFileLineReader.Create(AFileName);
end;

function TFileResolver.FindResourceFileName(const AFileName: string): String;

  Function FindInPath(FN : String) : String;

  var
    I : integer;

  begin
    Result:='';
    I:=0;
    While (Result='') and (I<FResourcePaths.Count) do
      begin
      Result:=SearchLowUpCase(FResourcePaths[i]+FN);
      Inc(I);
      end;
    // search in BaseDirectory
    if (Result='') and (BaseDirectory<>'') then
      Result:=SearchLowUpCase(BaseDirectory+FN);
  end;

var
  FN : string;

begin
  Result := '';
  // convert pathdelims to system
  FN:=SetDirSeparators(AFileName);
  If FilenameIsAbsolute(FN) then
    begin
    Result := SearchLowUpCase(FN);
    end
  else
    begin
    // file name is relative
    // search in include path
    Result:=FindInPath(FN);
    end;
end;

function TFileResolver.FindSourceFile(const AName: string): TLineReader;
begin
  Result := nil;
  if not FileExists(AName) then
    Raise EFileNotFoundError.create(AName)
  else
    try
      Result := CreateFileReader(AName)
    except
      Result := nil;
    end;
end;

function TFileResolver.FindIncludeFile(const AName: string): TLineReader;

Var
  FN : String;

begin
  Result:=Nil;
  FN:=FindIncludeFileName(AName);
  If (FN<>'') then
    try
      Result := TFileLineReader.Create(FN);
    except
      Result:=Nil;
    end;
end;
{$ENDIF}

{$ifdef fpc}
{ TStreamResolver }

procedure TStreamResolver.SetOwnsStreams(AValue: Boolean);
begin
  if FOwnsStreams=AValue then Exit;
  FOwnsStreams:=AValue;
end;

function TStreamResolver.FindIncludeFileName(const aFilename: string): String;
begin
  raise EFileNotFoundError.Create('TStreamResolver.FindIncludeFileName not supported '+aFilename);
  Result:='';
end;

function TStreamResolver.FindResourceFileName(const AFileName: string): String;
begin
  raise EFileNotFoundError.Create('TStreamResolver.FindResourceFileName not supported '+aFileName);
  Result:='';
end;

constructor TStreamResolver.Create;
begin
  Inherited;
  FStreams:=TStringList.Create;
  FStreams.Sorted:=True;
  FStreams.Duplicates:=dupError;
end;

destructor TStreamResolver.Destroy;
begin
  Clear;
  FreeAndNil(FStreams);
  inherited Destroy;
end;

procedure TStreamResolver.Clear;

Var
  I : integer;
begin
  if OwnsStreams then
    begin
    For I:=0 to FStreams.Count-1 do
      Fstreams.Objects[i].Free;
    end;
  FStreams.Clear;
end;

procedure TStreamResolver.AddStream(const AName: String; AStream: TStream);
begin
  FStreams.AddObject(AName,AStream);
end;

function TStreamResolver.FindStream(const AName: string; ScanIncludes : Boolean) : TStream;

Var
  I,J : Integer;
  FN : String;
begin
  Result:=Nil;
  I:=FStreams.IndexOf(AName);
  If (I=-1) and ScanIncludes then
    begin
    J:=0;
    While (I=-1) and (J<IncludePaths.Count-1) do
      begin
      FN:=IncludeTrailingPathDelimiter(IncludePaths[i])+AName;
      I:=FStreams.IndexOf(FN);
      Inc(J);
      end;
    end;
  If (I<>-1) then
    Result:=FStreams.Objects[i] as TStream;
end;

function TStreamResolver.FindStreamReader(const AName: string; ScanIncludes : Boolean) : TLineReader;

Var
  S : TStream;
  SL : TStreamLineReader;

begin
  Result:=Nil;
  S:=FindStream(AName,ScanIncludes);
  If (S<>Nil) then
    begin
    S.Position:=0;
    SL:=TStreamLineReader.Create(AName);
    try
      SL.InitFromStream(S);
      Result:=SL;
    except
      FreeAndNil(SL);
      Raise;
    end;
    end;
end;

function TStreamResolver.FindSourceFile(const AName: string): TLineReader;

begin
  Result:=FindStreamReader(AName,False);
end;

function TStreamResolver.FindIncludeFile(const AName: string): TLineReader;
begin
  Result:=FindStreamReader(AName,True);
end;
{$endif}

{ ---------------------------------------------------------------------
  TPascalScanner
  ---------------------------------------------------------------------}

constructor TPascalScanner.Create(AFileResolver: TBaseFileResolver);

  Function CS : TStringList;

  begin
    Result:=TStringList.Create;
    Result.Sorted:=True;
    Result.Duplicates:=dupError;
  end;

var
  vs: TValueSwitch;
begin
  inherited Create;
  FFileResolver := AFileResolver;
  FFiles:=TStringList.Create;
  FIncludeStack := TFPList.Create;
  FDefines := CS;
  FMacros:=CS;
  FMaxIncludeStackDepth:=DefaultMaxIncludeStackDepth;

  FCurrentModeSwitches:=FPCModeSwitches;
  FAllowedModeSwitches:=msAllModeSwitches;
  FCurrentBoolSwitches:=bsFPCMode;
  FAllowedBoolSwitches:=bsAll;
  FAllowedValueSwitches:=vsAllValueSwitches;
  for vs in TValueSwitch do
    FCurrentValueSwitches[vs]:=DefaultValueSwitches[vs];

  FConditionEval:=TCondDirectiveEvaluator.Create;
  FConditionEval.OnLog:=@OnCondEvalLog;
  FConditionEval.OnEvalVariable:=@OnCondEvalVar;
  FConditionEval.OnEvalFunction:=@OnCondEvalFunction;
end;

destructor TPascalScanner.Destroy;
begin
  FreeAndNil(FConditionEval);
  ClearMacros;
  FreeAndNil(FMacros);
  FreeAndNil(FDefines);
  ClearFiles;
  FreeAndNil(FFiles);
  FreeAndNil(FIncludeStack);
  inherited Destroy;
end;

procedure TPascalScanner.RegisterResourceHandler(aExtension: String; aHandler: TResourceHandler);

Var
  Idx: Integer;

begin
  if (aExtension='') then
    exit;
  if (aExtension[1]='.') then
    aExtension:=copy(aExtension,2,Length(aExtension)-1);
  Idx:=IndexOfResourceHandler(lowerCase(aExtension));
  if Idx=-1 then
    begin
    Idx:=Length(FResourceHandlers);
    SetLength(FResourceHandlers,Idx+1);
    FResourceHandlers[Idx].Ext:=LowerCase(aExtension);
    end;
  FResourceHandlers[Idx].handler:=aHandler;
end;

procedure TPascalScanner.RegisterResourceHandler(aExtensions: array of String; aHandler: TResourceHandler);

Var
  S : String;

begin
  For S in aExtensions do
    RegisterResourceHandler(S,aHandler);
end;

procedure TPascalScanner.ClearFiles;

begin
  // Dont' free the first element, because it is CurSourceFile
  while FIncludeStack.Count > 1 do
    begin
    TBaseFileResolver(FIncludeStack[1]).{$ifdef pas2js}Destroy{$else}Free{$endif};
    FIncludeStack.Delete(1);
    end;
  FIncludeStack.Clear;
  FreeAndNil(FCurSourceFile);
  FFiles.Clear;
  FModuleRow:=0;
end;

procedure TPascalScanner.ClearMacros;

Var
  I : Integer;

begin
  For I:=0 to FMacros.Count-1 do
    FMacros.Objects[i].{$ifdef pas2js}Destroy{$else}Free{$endif};
  FMacros.Clear;
end;

procedure TPascalScanner.SetCurToken(const AValue: TToken);
begin
  FCurToken:=AValue;
end;

procedure TPascalScanner.SetCurTokenString(const AValue: string);
begin
  FCurTokenString:=AValue;
end;

procedure TPascalScanner.OpenFile(AFilename: string);
begin
  Clearfiles;
  FCurSourceFile := FileResolver.FindSourceFile(AFilename);
  FCurFilename := AFilename;
  AddFile(FCurFilename);
  {$IFDEF HASFS}
  FileResolver.BaseDirectory := IncludeTrailingPathDelimiter(ExtractFilePath(FCurFilename));
  {$ENDIF}
  if LogEvent(sleFile) then
    DoLog(mtInfo,nLogOpeningFile,SLogOpeningFile,[FormatPath(AFileName)],True);
end;

procedure TPascalScanner.FinishedModule;
begin
  if (sleLineNumber in LogEvents)
      and (not CurSourceFile.IsEOF)
      and ((FCurRow Mod 100) > 0) then
    DoLog(mtInfo,nLogLineNumber,SLogLineNumber,[CurRow],True);
end;

function TPascalScanner.FormatPath(const aFilename: string): string;
begin
  if Assigned(OnFormatPath) then
    Result:=OnFormatPath(aFilename)
  else
    Result:=aFilename;
end;

procedure TPascalScanner.SetNonToken(aToken: TToken);
begin
  Include(FNonTokens,aToken);
end;

procedure TPascalScanner.UnsetNonToken(aToken: TToken);
begin
  Exclude(FNonTokens,aToken);
end;

procedure TPascalScanner.SetTokenOption(aOption: TTokenoption);
begin
  Include(FTokenOptions,aOption);
end;

procedure TPascalScanner.UnSetTokenOption(aOption: TTokenoption);
begin
  Exclude(FTokenOptions,aOption);
end;

function TPascalScanner.CheckToken(aToken: TToken; const ATokenString: String): TToken;
begin
  Result:=atoken;
  if (aToken=tkIdentifier) and (CompareText(aTokenString,'operator')=0) then
    if (toOperatorToken in TokenOptions) then
      Result:=tkoperator;
end;

function TPascalScanner.FetchToken: TToken;
var
  IncludeStackItem: TIncludeStackItem;
begin
  FPreviousToken:=FCurToken;
  while true do
  begin
    Result := DoFetchToken;
    Case FCurToken of
    tkEOF:
      begin
      if FIncludeStack.Count > 0 then
        begin
        IncludeStackItem :=
          TIncludeStackItem(FIncludeStack[FIncludeStack.Count - 1]);
        FIncludeStack.Delete(FIncludeStack.Count - 1);
        CurSourceFile.{$ifdef pas2js}Destroy{$else}Free{$endif};
        FCurSourceFile := IncludeStackItem.SourceFile;
        FCurFilename := IncludeStackItem.Filename;
        FCurToken := IncludeStackItem.Token;
        FCurTokenString := IncludeStackItem.TokenString;
        FCurLine := IncludeStackItem.Line;
        FCurRow := IncludeStackItem.Row;
        FCurColumnOffset := IncludeStackItem.ColumnOffset;
        FTokenPos := IncludeStackItem.TokenPos;
        IncludeStackItem.Free;
        Result := FCurToken;
        end
      else
        break;
      end;
    tkWhiteSpace,
    tkLineEnding:
      if not (FSkipWhiteSpace or PPIsSkipping) then
        Break;
    tkComment:
      if not (FSkipComments or PPIsSkipping) then
        Break;
    tkSelf:
      begin
      if Not (po_selftoken in Options) then
        begin
        FCurToken:=tkIdentifier;
        Result:=FCurToken;
        end;
      if not (FSkipComments or PPIsSkipping) then
        Break;
      end;
    tkOperator:
      begin
      if Not (toOperatorToken in FTokenOptions) then
        begin
        FCurToken:=tkIdentifier;
        Result:=FCurToken;
        end;
      if not (FSkipComments or PPIsSkipping) then
        Break;
      end;

    else
      if not PPIsSkipping then
        break;
    end; // Case
  end;
//  Writeln(Result, '(',CurTokenString,')');
end;

function TPascalScanner.ReadNonPascalTillEndToken(StopAtLineEnd: boolean
  ): TToken;
var
  StartPos: {$ifdef UsePChar}PChar{$else}integer{$endif};
  {$ifndef UsePChar}
  var
    s: string;
    l: integer;
  {$endif}

  Procedure Add;
  var
    AddLen: PtrInt;
    {$ifdef UsePChar}
    OldLen: Integer;
    {$endif}
  begin
    AddLen:=FTokenPos-StartPos;
    if AddLen=0 then
      FCurTokenString:=''
    else
      begin
      {$ifdef UsePChar}
      OldLen:=length(FCurTokenString);
      SetLength(FCurTokenString,OldLen+AddLen);
      Move(StartPos^,PChar(PChar(FCurTokenString)+OldLen)^,AddLen);
      {$else}
      FCurTokenString:=FCurTokenString+copy(FCurLine,StartPos,AddLen);
      {$endif}
      StartPos:=FTokenPos;
      end;
  end;

  function DoEndOfLine: boolean;
  begin
    Add;
    if StopAtLineEnd then
      begin
      ReadNonPascalTillEndToken := tkLineEnding;
      FCurToken := tkLineEnding;
      FetchLine;
      exit(true);
      end;
    if not FetchLine then
      begin
      ReadNonPascalTillEndToken := tkEOF;
      FCurToken := tkEOF;
      exit(true);
      end;
    {$ifndef UsePChar}
    s:=FCurLine;
    l:=length(s);
    {$endif}
    StartPos:=FTokenPos;
    Result:=false;
  end;

begin
  FCurTokenString := '';
  StartPos:=FTokenPos;
  {$ifndef UsePChar}
  s:=FCurLine;
  l:=length(s);
  {$endif}
  repeat
    {$ifndef UsePChar}
    if FTokenPos>l then
      if DoEndOfLine then exit;
    {$endif}
    case {$ifdef UsePChar}FTokenPos^{$else}s[FTokenPos]{$endif} of
      {$ifdef UsePChar}
      #0: // end of line
        if DoEndOfLine then exit;
      {$endif}
      '''':
        begin
        // Notes:
        // 1. Eventually there should be a mechanism to override parsing non-pascal
        // 2. By default skip Pascal string literals, as this is more intuitive
        //    in IDEs with Pascal highlighters
        inc(FTokenPos);
        repeat
          {$ifndef UsePChar}
          if FTokenPos>l then
            Error(nErrOpenString,SErrOpenString);
          {$endif}
          case {$ifdef UsePChar}FTokenPos^{$else}s[FTokenPos]{$endif} of
          {$ifdef UsePChar}
          #0: Error(nErrOpenString,SErrOpenString);
          {$endif}
          '''':
            begin
            inc(FTokenPos);
            break;
            end;
          #10,#13:
            begin
            // string literal missing closing apostroph
            break;
            end
          else
            inc(FTokenPos);
          end;
        until false;
        end;
      '/':
        begin
        inc(FTokenPos);
        if {$ifdef UsePChar}FTokenPos^='/'{$else}(FTokenPos<=l) and (s[FTokenPos]='/'){$endif} then
          begin
          // skip Delphi comment //, see Note above
          repeat
            inc(FTokenPos);
          until {$ifdef UsePChar}FTokenPos^ in [#0,#10,#13]{$else}(FTokenPos>l) or (s[FTokenPos] in [#10,#13]){$endif};
          end;
        end;
      '0'..'9', 'A'..'Z', 'a'..'z','_':
        begin
        // number or identifier
        if {$ifdef UsePChar}
            (FTokenPos[0] in ['e','E'])
            and (FTokenPos[1] in ['n','N'])
            and (FTokenPos[2] in ['d','D'])
            and not (FTokenPos[3] in IdentChars)
            {$else}
            (TJSString(copy(s,FTokenPos,3)).toLowerCase='end')
            and ((FTokenPos+3>l) or not (s[FTokenPos+3] in IdentChars))
            {$endif}
            then
          begin
          // 'end' found
          Add;
          if FCurTokenString<>'' then
            begin
            // return characters in front of 'end'
            Result:=tkWhitespace;
            FCurToken:=Result;
            exit;
            end;
          // return 'end'
          Result := tkend;
          {$ifdef UsePChar}
          SetLength(FCurTokenString, 3);
          Move(FTokenPos^, FCurTokenString[1], 3);
          {$else}
          FCurTokenString:=copy(s,FTokenPos,3);
          {$endif}
          inc(FTokenPos,3);
          FCurToken := Result;
          exit;
          end
        else
          begin
          // skip identifier
          while {$ifdef UsePChar}FTokenPos[0] in IdentChars{$else}(FTokenPos<=l) and (s[FTokenPos] in IdentChars){$endif} do
            inc(FTokenPos);
          end;
        end;
      else
        inc(FTokenPos);
    end;
  until false;
end;

procedure TPascalScanner.Error(MsgNumber: integer; const Msg: string);
begin
  SetCurMsg(mtError,MsgNumber,Msg,[]);
  raise EScannerError.CreateFmt('%s(%d,%d) Error: %s',
    [FormatPath(CurFilename),CurRow,CurColumn,FLastMsg]);
end;

procedure TPascalScanner.Error(MsgNumber: integer; const Fmt: string;
  Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
begin
  SetCurMsg(mtError,MsgNumber,Fmt,Args);
  raise EScannerError.CreateFmt('%s(%d,%d) Error: %s',
    [FormatPath(CurFilename),CurRow,CurColumn,FLastMsg]);
end;

function TPascalScanner.DoFetchTextToken:TToken;
var
  OldLength     : Integer;
  TokenStart    : {$ifdef UsePChar}PChar{$else}integer{$endif};
  SectionLength : Integer;
  {$ifndef UsePChar}
  s: String;
  l: integer;
  {$endif}
begin
  Result:=tkEOF;
  OldLength:=0;
  FCurTokenString := '';
  {$ifndef UsePChar}
  s:=FCurLine;
  l:=length(s);
  {$endif}

  repeat
    {$ifndef UsePChar}
    if FTokenPos>l then break;
    {$endif}
    case {$ifdef UsePChar}FTokenPos[0]{$else}s[FTokenPos]{$endif} of
      '^' :
        begin
        TokenStart := FTokenPos;
        Inc(FTokenPos);
        if {$ifdef UsePChar}FTokenPos[0] in Letters{$else}(FTokenPos<l) and (s[FTokenPos] in Letters){$endif} then
          Inc(FTokenPos);
        if Result=tkEOF then Result := tkChar else Result:=tkString;
        end;
      '#':
        begin
        TokenStart := FTokenPos;
        Inc(FTokenPos);
        if {$ifdef UsePChar}FTokenPos[0]='$'{$else}(FTokenPos<l) and (s[FTokenPos]='$'){$endif} then
        begin
          Inc(FTokenPos);
          repeat
            Inc(FTokenPos);
          until {$ifdef UsePChar}not (FTokenPos[0] in HexDigits){$else}(FTokenPos>l) or not (s[FTokenPos] in HexDigits){$endif};
        end else
          repeat
            Inc(FTokenPos);
          until {$ifdef UsePChar}not (FTokenPos[0] in Digits){$else}(FTokenPos>l) or not (s[FTokenPos] in Digits){$endif};
        if Result=tkEOF then Result := tkChar else Result:=tkString;
        end;
      '''':
        begin
          TokenStart := FTokenPos;
          Inc(FTokenPos);

          while true do
          begin
            if {$ifdef UsePChar}FTokenPos[0] = ''''{$else}(FTokenPos<=l) and (s[FTokenPos]=''''){$endif} then
              if {$ifdef UsePChar}FTokenPos[1] = ''''{$else}(FTokenPos<l) and (s[FTokenPos+1]=''''){$endif} then
                Inc(FTokenPos)
              else
                break;

            if {$ifdef UsePChar}FTokenPos[0] = #0{$else}FTokenPos>l{$endif} then
              Error(nErrOpenString,SErrOpenString);

            Inc(FTokenPos);
          end;
          Inc(FTokenPos);
          if ((FTokenPos - TokenStart)=3) then // 'z'
            Result := tkChar
          else
            Result := tkString;
        end;
    else
      Break;
    end;
    SectionLength := FTokenPos - TokenStart;
    {$ifdef UsePChar}
    SetLength(FCurTokenString, OldLength + SectionLength);
    if SectionLength > 0 then
      Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
    {$else}
    FCurTokenString:=FCurTokenString+copy(FCurLine,TokenStart,SectionLength);
    {$endif}
    Inc(OldLength, SectionLength);
  until false;
end;

procedure TPascalScanner.PushStackItem;

Var
  SI: TIncludeStackItem;

begin
  if FIncludeStack.Count>=MaxIncludeStackDepth then
    Error(nErrIncludeLimitReached,SErrIncludeLimitReached);
  SI := TIncludeStackItem.Create;
  SI.SourceFile := CurSourceFile;
  SI.Filename := CurFilename;
  SI.Token := CurToken;
  SI.TokenString := CurTokenString;
  SI.Line := CurLine;
  SI.Row := CurRow;
  SI.ColumnOffset := FCurColumnOffset;
  SI.TokenPos := FTokenPos;
  FIncludeStack.Add(SI);
  FTokenPos:={$ifdef UsePChar}Nil{$else}-1{$endif};
  FCurRow := 0;
  FCurColumnOffset := 1;
end;

procedure TPascalScanner.HandleIncludeFile(Param: String);

var
  NewSourceFile: TLineReader;
begin
  if Length(Param)>1 then
    begin
    if (Param[1]='''') then
      begin
      if Param[length(Param)]<>'''' then
        Error(nErrOpenString,SErrOpenString,[]);
      Param:=copy(Param,2,length(Param)-2);
      end;
    end;
  NewSourceFile := FileResolver.FindIncludeFile(Param);
  if not Assigned(NewSourceFile) then
    Error(nErrIncludeFileNotFound, SErrIncludeFileNotFound, [Param]);

  PushStackItem;
  FCurSourceFile:=NewSourceFile;
  FCurFilename := Param;
  if FCurSourceFile is TFileLineReader then
    FCurFilename := TFileLineReader(FCurSourceFile).Filename; // nicer error messages
  AddFile(FCurFilename);
  If LogEvent(sleFile) then
    DoLog(mtInfo,nLogOpeningFile,SLogOpeningFile,[FormatPath(FCurFileName)],True);
end;

procedure TPascalScanner.HandleResource(Param: string);

Var
  Ext,aFullFileName,aFilename,aOptions : String;
  P: Integer;
  H : TResourceHandler;
  OptList : TStrings;

begin
  aFilename:='';
  aOptions:='';
  P:=Pos(';',Param);
  If P=0 then
    aFileName:=Trim(Param)
  else
    begin
    aFileName:=Trim(Copy(Param,1,P-1));
    aOptions:=Copy(Param,P+1,Length(Param)-P);
    end;
  Ext:=ExtractFileExt(aFileName);
  // Construct & find filename
  If (ChangeFileExt(aFileName,'')='*') then
    aFileName:=ChangeFileExt(ExtractFileName(CurFilename),Ext);
  aFullFileName:=FileResolver.FindResourceFileName(aFileName);
  if aFullFileName='' then
    Error(nResourceFileNotFound,SErrResourceFileNotFound,[aFileName]);
  // Check if we can find a handler.
  if Ext<>'' then
    Ext:=Copy(Ext,2,Length(Ext)-1);
  H:=FindResourceHandler(LowerCase(Ext));
  if (H=Nil) then
    H:=FindResourceHandler('*');
  if (H=Nil) then
    Error(nNoResourceSupport,SNoResourceSupport,[Ext]);
  // Let the handler take care of the rest.
  OptList:=TStringList.Create;
  try
    OptList.NameValueSeparator:=':';
    OptList.Delimiter:=';';
    OptList.StrictDelimiter:=True;
    OptList.DelimitedText:=aOptions;
    H(Self,aFullFileName,OptList);
  finally
    OptList.Free;
  end;
end;

function TPascalScanner.HandleMacro(AIndex : integer) : TToken;

Var
  M : TMacroDef;
  ML : TMacroReader;
  OldRow, OldCol: Integer;

begin
  OldRow:=CurRow;
  OldCol:=CurColumn;
  PushStackItem;
  M:=FMacros.Objects[AIndex] as TMacroDef;
  ML:=TMacroReader.Create(FCurFileName,M.Value);
  ML.CurRow:=OldRow;
  ML.CurCol:=OldCol-length(M.Name);
  FCurSourceFile:=ML;
  Result:=DoFetchToken;
//  Writeln(Result,Curtoken);
end;

procedure TPascalScanner.HandleInterfaces(const Param: String);
var
  s, NewValue: String;
  p: SizeInt;
begin
  if not (vsInterfaces in AllowedValueSwitches) then
    Error(nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,['interfaces']);
  s:=Uppercase(Param);
  p:=Pos(' ',s);
  if p>0 then
    s:=LeftStr(s,p-1);
  case s of
  'COM','DEFAULT': NewValue:='COM';
  'CORBA': NewValue:='CORBA';
  else
    Error(nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,['interfaces '+s]);
    exit;
  end;
  if SameText(NewValue,CurrentValueSwitch[vsInterfaces]) then exit;
  if vsInterfaces in ReadOnlyValueSwitches then
    begin
    Error(nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,['interfaces']);
    exit;
    end;
  CurrentValueSwitch[vsInterfaces]:=NewValue;
end;

procedure TPascalScanner.HandleWarn(Param: String);
// $warn identifier on|off|default|error
var
  p, StartPos: Integer;
  Identifier, Value: String;
begin
  p:=1;
  while (p<=length(Param)) and (Param[p] in [' ',#9]) do inc(p);
  StartPos:=p;
  while (p<=length(Param)) and (Param[p] in ['a'..'z','A'..'Z','0'..'9','_']) do inc(p);
  Identifier:=copy(Param,StartPos,p-StartPos);
  while (p<=length(Param)) and (Param[p] in [' ',#9]) do inc(p);
  StartPos:=p;
  while (p<=length(Param)) and (Param[p] in ['a'..'z','A'..'Z','_']) do inc(p);
  Value:=copy(Param,StartPos,p-StartPos);
  HandleWarnIdentifier(Identifier,Value);
end;

procedure TPascalScanner.HandleWarnIdentifier(Identifier,
  Value: String);
var
  Number: LongInt;
  State: TWarnMsgState;
  Handled: Boolean;
begin
  if Identifier='' then
    Error(nIllegalStateForWarnDirective,SIllegalStateForWarnDirective,['']);
  if Value='' then
    begin
    DoLog(mtWarning,nIllegalStateForWarnDirective,SIllegalStateForWarnDirective,['']);
    exit;
    end;
  case lowercase(Value) of
  'on': State:=wmsOn;
  'off': State:=wmsOff;
  'default': State:=wmsDefault;
  'error': State:=wmsError;
  else
    DoLog(mtWarning,nIllegalStateForWarnDirective,SIllegalStateForWarnDirective,[Value]);
    exit;
  end;

  if Assigned(OnWarnDirective) then
    begin
    Handled:=false;
    OnWarnDirective(Self,Identifier,State,Handled);
    if Handled then
      exit;
    end;

  if Identifier[1] in ['0'..'9'] then
    begin
    // fpc number
    Number:=StrToIntDef(Identifier,-1);
    if Number<0 then
      begin
      DoLog(mtWarning,nIllegalStateForWarnDirective,SIllegalStateForWarnDirective,[Identifier]);
      exit;
      end;
    SetWarnMsgState(Number,State);
    end;
end;

procedure TPascalScanner.HandleDefine(Param: String);

Var
  Index : Integer;
  MName,MValue : String;

begin
  Param := UpperCase(Param);
  Index:=Pos(':=',Param);
  If (Index=0) then
    AddDefine(GetMacroName(Param))
  else
    begin
    MValue:=Trim(Param);
    MName:=Trim(Copy(MValue,1,Index-1));
    Delete(MValue,1,Index+1);
    AddMacro(MName,MValue);
    end;
end;

procedure TPascalScanner.HandleDispatchField(Param: String; vs: TValueSwitch);
var
  NewValue: String;
begin
  if not (vs in AllowedValueSwitches) then
    Error(nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,[ValueSwitchNames[vs]]);
  NewValue:=ReadIdentifier(Param);
  if NewValue='-' then
    NewValue:=''
  else if not IsValidIdent(NewValue,false) then
    DoLog(mtWarning,nInvalidDispatchFieldName,SInvalidDispatchFieldName,[]);
  if SameText(NewValue,CurrentValueSwitch[vs]) then exit;
  if vs in ReadOnlyValueSwitches then
    begin
    Error(nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,[ValueSwitchNames[vs]]);
    exit;
    end;
  CurrentValueSwitch[vs]:=NewValue;
end;

procedure TPascalScanner.HandleError(Param: String);
begin
  if po_StopOnErrorDirective in Options then
    Error(nUserDefined, SUserDefined,[Param])
  else
    DoLog(mtWarning,nUserDefined,SUserDefined+' error',[Param]);
end;

procedure TPascalScanner.HandleMessageDirective(Param: String);
var
  p: Integer;
  Kind: String;
  MsgType: TMessageType;
begin
  if Param='' then exit;
  p:=1;
  while (p<=length(Param)) and (Param[p] in ['a'..'z','A'..'Z']) do inc(p);
  Kind:=LeftStr(Param,p-1);
  MsgType:=mtHint;
  case UpperCase(Kind) of
  'HINT': MsgType:=mtHint;
  'NOTE': MsgType:=mtNote;
  'WARN': MsgType:=mtWarning;
  'ERROR': MsgType:=mtError;
  'FATAL': MsgType:=mtFatal;
  else
    // $Message 'hint text'
    p:=1;
  end;
  while (p<=length(Param)) and (Param[p] in [' ',#9]) do inc(p);
  Delete(Param,1,p-1);
  if MsgType in [mtFatal,mtError] then
    HandleError(Param)
  else
    DoLog(MsgType,nUserDefined,SUserDefined,[Param]);
end;

procedure TPascalScanner.HandleUnDefine(Param: String);
begin
  UnDefine(GetMacroName(Param));
end;

function TPascalScanner.HandleInclude(const Param: String): TToken;

begin
  Result:=tkComment;
  if (Param<>'') and (Param[1]='%') then
    begin
    FCurTokenString:=''''+Param+'''';
    FCurToken:=tkString;
    Result:=FCurToken;
    end
  else
    HandleIncludeFile(Param);
end;

procedure TPascalScanner.HandleMode(const Param: String);

  procedure SetMode(const LangMode: TModeSwitch;
    const NewModeSwitches: TModeSwitches; IsDelphi: boolean;
    const AddBoolSwitches: TBoolSwitches = [];
    const RemoveBoolSwitches: TBoolSwitches = []
    );
  var
    Handled: Boolean;
  begin
    if not (LangMode in AllowedModeSwitches) then
      Error(nErrInvalidMode,SErrInvalidMode,[Param]);
    Handled:=false;
    if Assigned(OnModeChanged) then
      OnModeChanged(Self,LangMode,true,Handled);
    if not Handled then
      begin
      CurrentModeSwitches:=(NewModeSwitches+ReadOnlyModeSwitches)*AllowedModeSwitches;
      CurrentBoolSwitches:=CurrentBoolSwitches+(AddBoolSwitches*AllowedBoolSwitches)
        -(RemoveBoolSwitches*AllowedBoolSwitches);
      if IsDelphi then
        FOptions:=FOptions+[po_delphi]
      else
        FOptions:=FOptions-[po_delphi];
      end;
    Handled:=false;
    if Assigned(OnModeChanged) then
      OnModeChanged(Self,LangMode,false,Handled);
  end;

Var
  P : String;
begin
  if SkipGlobalSwitches then
    begin
    DoLog(mtWarning,nMisplacedGlobalCompilerSwitch,SMisplacedGlobalCompilerSwitch,[]);
    exit;
    end;
  P:=UpperCase(Param);
  Case P of
  'FPC','DEFAULT':
    SetMode(msFpc,FPCModeSwitches,false,bsFPCMode);
  'OBJFPC':
    begin
    SetMode(msObjfpc,OBJFPCModeSwitches,true,bsObjFPCMode);
    UnsetNonToken(tkgeneric);
    UnsetNonToken(tkspecialize);
    end;
  'DELPHI':
    begin
    SetMode(msDelphi,DelphiModeSwitches,true,bsDelphiMode,[bsPointerMath]);
    SetNonToken(tkgeneric);
    SetNonToken(tkspecialize);
    end;
  'DELPHIUNICODE':
    begin
    SetMode(msDelphiUnicode,DelphiUnicodeModeSwitches,true,bsDelphiUnicodeMode,[bsPointerMath]);
    SetNonToken(tkgeneric);
    SetNonToken(tkspecialize);
    end;
  'TP':
    SetMode(msTP7,TPModeSwitches,false);
  'MACPAS':
    SetMode(msMac,MacModeSwitches,false,bsMacPasMode);
  'ISO':
    SetMode(msIso,ISOModeSwitches,false);
  'EXTENDED':
    SetMode(msExtpas,ExtPasModeSwitches,false);
  'GPC':
    SetMode(msGPC,GPCModeSwitches,false);
  else
    Error(nErrInvalidMode,SErrInvalidMode,[Param])
  end;
end;

procedure TPascalScanner.HandleModeSwitch(const Param: String);
// $modeswitch param
// name, name-, name+, name off, name on, name- comment, name on comment
Var
  MS : TModeSwitch;
  MSN,PM : String;
  p : Integer;
  Enable: Boolean;

begin
  PM:=Param;
  p:=1;
  while (p<=length(PM)) and (PM[p] in ['a'..'z','A'..'Z','_','0'..'9']) do
    inc(p);
  MSN:=LeftStr(PM,p-1);
  Delete(PM,1,p-1);
  MS:=StrToModeSwitch(MSN);
  if (MS=msNone) or not (MS in AllowedModeSwitches) then
    begin
    if po_CheckModeSwitches in Options then
      Error(nErrInvalidModeSwitch,SErrInvalidModeSwitch,[MSN])
    else
      DoLog(mtWarning,nErrInvalidModeSwitch,SErrInvalidModeSwitch,[MSN]);
    exit; // ignore
    end;
  if PM='' then
    Enable:=true
  else
    case PM[1] of
    '+','-':
      begin
      Enable:=PM[1]='+';
      p:=2;
      if (p<=length(PM)) and not (PM[p] in [' ',#9]) then
        Error(nErrWrongSwitchToggle,SErrWrongSwitchToggle,[]);
      end;
    ' ',#9:
      begin
      PM:=TrimLeft(PM);
      if PM<>'' then
        begin
        p:=1;
        while (p<=length(PM)) and (PM[p] in ['A'..'Z']) do inc(p);
        if (p<=length(PM)) and not (PM[p] in [' ',#9]) then
          Error(nErrWrongSwitchToggle,SErrWrongSwitchToggle,[]);
        PM:=LeftStr(PM,p-1);
        if PM='ON' then
          Enable:=true
        else if PM='OFF' then
          Enable:=false
        else
          Error(nErrWrongSwitchToggle,SErrWrongSwitchToggle,[]);
        end;
      end;
    else
      Error(nErrWrongSwitchToggle,SErrWrongSwitchToggle,[]);
    end;

  if MS in CurrentModeSwitches=Enable then
    exit; // no change
  if MS in ReadOnlyModeSwitches then
    begin
    DoLog(mtWarning,nErrInvalidModeSwitch,SErrInvalidModeSwitch,[MSN]);
    exit;
    end;
  if Enable then
    CurrentModeSwitches:=CurrentModeSwitches+[MS]
  else
    CurrentModeSwitches:=CurrentModeSwitches-[MS];
end;

procedure TPascalScanner.PushSkipMode;

begin
  if PPSkipStackIndex = High(PPSkipModeStack) then
    Error(nErrIfXXXNestingLimitReached,SErrIfXXXNestingLimitReached);
  PPSkipModeStack[PPSkipStackIndex] := PPSkipMode;
  PPIsSkippingStack[PPSkipStackIndex] := PPIsSkipping;
  Inc(PPSkipStackIndex);
end;

procedure TPascalScanner.HandleIFDEF(const AParam: String);
var
  aName: String;
begin
  PushSkipMode;
  if PPIsSkipping then
    PPSkipMode := ppSkipAll
  else
    begin
    aName:=ReadIdentifier(AParam);
    if IsDefined(aName) then
      PPSkipMode := ppSkipElseBranch
    else
      begin
      PPSkipMode := ppSkipIfBranch;
      PPIsSkipping := true;
      end;
    If LogEvent(sleConditionals) then
      if PPSkipMode=ppSkipElseBranch then
        DoLog(mtInfo,nLogIFDefAccepted,sLogIFDefAccepted,[aName])
      else
        DoLog(mtInfo,nLogIFDefRejected,sLogIFDefRejected,[aName]);
    end;
end;

procedure TPascalScanner.HandleIFNDEF(const AParam: String);
var
  aName: String;
begin
  PushSkipMode;
  if PPIsSkipping then
    PPSkipMode := ppSkipAll
  else
    begin
    aName:=ReadIdentifier(AParam);
    if IsDefined(aName) then
      begin
      PPSkipMode := ppSkipIfBranch;
      PPIsSkipping := true;
      end
    else
      PPSkipMode := ppSkipElseBranch;
    If LogEvent(sleConditionals) then
      if PPSkipMode=ppSkipElseBranch then
        DoLog(mtInfo,nLogIFNDefAccepted,sLogIFNDefAccepted,[aName])
      else
        DoLog(mtInfo,nLogIFNDefRejected,sLogIFNDefRejected,[aName]);
    end;
end;

procedure TPascalScanner.HandleIFOPT(const AParam: String);

begin
  PushSkipMode;
  if PPIsSkipping then
    PPSkipMode := ppSkipAll
  else
    begin
    if (length(AParam)<>2) or not (AParam[1] in ['a'..'z','A'..'Z'])
        or not (AParam[2] in ['+','-']) then
      Error(nErrXExpectedButYFound,sErrXExpectedButYFound,['letter[+|-]',AParam]);
    if IfOpt(AParam[1])=(AParam[2]='+') then
      PPSkipMode := ppSkipElseBranch
    else
      begin
      PPSkipMode := ppSkipIfBranch;
      PPIsSkipping := true;
      end;
    If LogEvent(sleConditionals) then
      if PPSkipMode=ppSkipElseBranch then
        DoLog(mtInfo,nLogIFOptAccepted,sLogIFOptAccepted,[AParam])
      else
        DoLog(mtInfo,nLogIFOptRejected,sLogIFOptRejected,[AParam]);
    end;
end;

procedure TPascalScanner.HandleIF(const AParam: String);

begin
  PushSkipMode;
  if PPIsSkipping then
    PPSkipMode := ppSkipAll
  else
    begin
    if ConditionEval.Eval(AParam) then
      PPSkipMode := ppSkipElseBranch
    else
      begin
      PPSkipMode := ppSkipIfBranch;
      PPIsSkipping := true;
      end;
    If LogEvent(sleConditionals) then
      if PPSkipMode=ppSkipElseBranch then
        DoLog(mtInfo,nLogIFAccepted,sLogIFAccepted,[AParam])
      else
        DoLog(mtInfo,nLogIFRejected,sLogIFRejected,[AParam]);
    end;
end;

procedure TPascalScanner.HandleELSEIF(const AParam: String);
begin
  if PPSkipStackIndex = 0 then
    Error(nErrInvalidPPElse,sErrInvalidPPElse);
  if PPSkipMode = ppSkipIfBranch then
    begin
    if ConditionEval.Eval(AParam) then
      begin
      PPSkipMode := ppSkipElseBranch;
      PPIsSkipping := false;
      end
    else
      PPIsSkipping := true;
    If LogEvent(sleConditionals) then
      if PPSkipMode=ppSkipElseBranch then
        DoLog(mtInfo,nLogELSEIFAccepted,sLogELSEIFAccepted,[AParam])
      else
        DoLog(mtInfo,nLogELSEIFRejected,sLogELSEIFRejected,[AParam]);
    end
  else if PPSkipMode=ppSkipElseBranch then
    begin
    PPIsSkipping := true;
    end;
end;

procedure TPascalScanner.HandleELSE(const AParam: String);

begin
  if AParam='' then;
  if PPSkipStackIndex = 0 then
    Error(nErrInvalidPPElse,sErrInvalidPPElse);
  if PPSkipMode = ppSkipIfBranch then
    PPIsSkipping := false
  else if PPSkipMode = ppSkipElseBranch then
    PPIsSkipping := true;
end;


procedure TPascalScanner.HandleENDIF(const AParam: String);

begin
  if AParam='' then;
  if PPSkipStackIndex = 0 then
    Error(nErrInvalidPPEndif,sErrInvalidPPEndif);
  Dec(PPSkipStackIndex);
  PPSkipMode := PPSkipModeStack[PPSkipStackIndex];
  PPIsSkipping := PPIsSkippingStack[PPSkipStackIndex];
end;

function TPascalScanner.HandleDirective(const ADirectiveText: String): TToken;

Var
  Directive,Param : String;
  P : Integer;
  Handled: Boolean;

  procedure DoBoolDirective(bs: TBoolSwitch);
  begin
    if bs in AllowedBoolSwitches then
      begin
      Handled:=true;
      HandleBoolDirective(bs,Param);
      end
    else
      Handled:=false;
  end;

begin
  Result:=tkComment;
  P:=Pos(' ',ADirectiveText);
  If P=0 then
    P:=Length(ADirectiveText)+1;
  Directive:=Copy(ADirectiveText,2,P-2); // 1 is $
  Param:=ADirectiveText;
  Delete(Param,1,P);
  {$IFDEF VerbosePasDirectiveEval}
  Writeln('TPascalScanner.HandleDirective.Directive: "',Directive,'", Param : "',Param,'"');
  {$ENDIF}

  Case UpperCase(Directive) of
  'IFDEF':
     HandleIFDEF(Param);
  'IFNDEF':
     HandleIFNDEF(Param);
  'IFOPT':
     HandleIFOPT(Param);
  'IF':
     HandleIF(Param);
  'ELSEIF':
     HandleELSEIF(Param);
  'ELSE':
     HandleELSE(Param);
  'ENDIF':
    HandleENDIF(Param);
  'IFEND':
    HandleENDIF(Param);
  else
    if PPIsSkipping then exit;

    Handled:=false;
    if (length(Directive)=2)
        and (Directive[1] in ['a'..'z','A'..'Z'])
        and (Directive[2] in ['-','+']) then
      begin
      Handled:=true;
      Result:=HandleLetterDirective(Directive[1],Directive[2]='+');
      end;

    if not Handled then
      begin
      Handled:=true;
      Case UpperCase(Directive) of
        'ASSERTIONS':
          DoBoolDirective(bsAssertions);
        'DEFINE':
          HandleDefine(Param);
        'GOTO':
          DoBoolDirective(bsGoto);
        'DIRECTIVEFIELD':
          HandleDispatchField(Param,vsDispatchField);
        'DIRECTIVESTRFIELD':
          HandleDispatchField(Param,vsDispatchStrField);
        'ERROR':
          HandleError(Param);
        'HINT':
          DoLog(mtHint,nUserDefined,SUserDefined,[Param]);
        'HINTS':
          DoBoolDirective(bsHints);
        'I','INCLUDE':
          Result:=HandleInclude(Param);
        'INTERFACES':
          HandleInterfaces(Param);
        'LONGSTRINGS':
          DoBoolDirective(bsLongStrings);
        'MACRO':
          DoBoolDirective(bsMacro);
        'MESSAGE':
          HandleMessageDirective(Param);
        'MODE':
          HandleMode(Param);
        'MODESWITCH':
          HandleModeSwitch(Param);
        'NOTE':
          DoLog(mtNote,nUserDefined,SUserDefined,[Param]);
        'NOTES':
          DoBoolDirective(bsNotes);
        'OBJECTCHECKS':
          DoBoolDirective(bsObjectChecks);
        'OVERFLOWCHECKS','OV':
          DoBoolDirective(bsOverflowChecks);
        'POINTERMATH':
          DoBoolDirective(bsPointerMath);
        'R' :
          HandleResource(Param);
        'RANGECHECKS':
          DoBoolDirective(bsRangeChecks);
        'SCOPEDENUMS':
          DoBoolDirective(bsScopedEnums);
        'TYPEDADDRESS':
          DoBoolDirective(bsTypedAddress);
        'TYPEINFO':
          DoBoolDirective(bsTypeInfo);
        'UNDEF':
          HandleUnDefine(Param);
        'WARN':
          HandleWarn(Param);
        'WARNING':
          DoLog(mtWarning,nUserDefined,SUserDefined,[Param]);
        'WARNINGS':
          DoBoolDirective(bsWarnings);
        'WRITEABLECONST':
          DoBoolDirective(bsWriteableConst);
      else
        Handled:=false;
      end;
      end;

    DoHandleDirective(Self,Directive,Param,Handled);
    if (not Handled) then
      if LogEvent(sleDirective) then
        DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,
          [Directive]);
  end;
end;

function TPascalScanner.HandleLetterDirective(Letter: char; Enable: boolean): TToken;
var
  bs: TBoolSwitch;
begin
  Result:=tkComment;
  Letter:=upcase(Letter);
  bs:=LetterToBoolSwitch[Letter];
  if bs=bsNone then
    DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,
      [Letter]);
  if not (bs in AllowedBoolSwitches) then
    begin
    DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,
      [Letter]);
    end;
  if (bs in FCurrentBoolSwitches)<>Enable then
    begin
    if bs in FReadOnlyBoolSwitches then
      begin
      DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,
        [Letter+BoolToStr(Enable,'+','-')]);
      exit;
      end;
    if Enable then
      begin
      AddDefine(LetterSwitchNames[Letter]);
      Include(FCurrentBoolSwitches,bs);
      end
    else
      begin
      UnDefine(LetterSwitchNames[Letter]);
      Exclude(FCurrentBoolSwitches,bs);
      end;
    end;
end;

procedure TPascalScanner.HandleBoolDirective(bs: TBoolSwitch;
  const Param: String);
var
  NewValue: Boolean;
begin
  if CompareText(Param,'on')=0 then
    NewValue:=true
  else if CompareText(Param,'off')=0 then
    NewValue:=false
  else
    Error(nErrXExpectedButYFound,SErrXExpectedButYFound,['on',Param]);
  if (bs in CurrentBoolSwitches)=NewValue then exit;
  if bs in ReadOnlyBoolSwitches then
    DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,
      [BoolSwitchNames[bs]])
  else if NewValue then
    CurrentBoolSwitches:=CurrentBoolSwitches+[bs]
  else
    CurrentBoolSwitches:=CurrentBoolSwitches-[bs];
end;

procedure TPascalScanner.DoHandleDirective(Sender: TObject; Directive,
  Param: String; var Handled: boolean);
begin
  if Assigned(OnDirective) then
    OnDirective(Self,Directive,Param,Handled);
end;

function TPascalScanner.DoFetchToken: TToken;
var
  TokenStart: {$ifdef UsePChar}PChar{$else}integer{$endif};
  i: TToken;
  SectionLength, NestingLevel, Index: Integer;
  {$ifdef UsePChar}
  OldLength: integer;
  {$else}
  s: string;
  l: integer;
  {$endif}

  procedure FetchCurTokenString; inline;
  begin
    {$ifdef UsePChar}
    SetLength(FCurTokenString, SectionLength);
    if SectionLength > 0 then
      Move(TokenStart^, FCurTokenString[1], SectionLength);
    {$else}
    FCurTokenString:=copy(FCurLine,TokenStart,SectionLength);
    {$endif}
  end;

  function FetchLocalLine: boolean; inline;
  begin
    Result:=FetchLine;
    {$ifndef UsePChar}
    if not Result then exit;
    s:=FCurLine;
    l:=length(s);
    {$endif}
  end;

begin
  Result:=tkLineEnding;
  if FTokenPos {$ifdef UsePChar}= nil{$else}<1{$endif} then
    if not FetchLine then
      begin
      Result := tkEOF;
      FCurToken := Result;
      exit;
      end;
  FCurTokenString := '';
  FCurTokenPos.FileName:=CurFilename;
  FCurTokenPos.Row:=CurRow;
  FCurTokenPos.Column:=CurColumn;
  {$ifndef UsePChar}
  s:=FCurLine;
  l:=length(s);
  if FTokenPos>l then
    begin
    FetchLine;
    Result := tkLineEnding;
    FCurToken := Result;
    exit;
    end;
  {$endif}
  case {$ifdef UsePChar}FTokenPos[0]{$else}s[FTokenPos]{$endif} of
    {$ifdef UsePChar}
    #0:         // Empty line
      begin
      FetchLine;
      Result := tkLineEnding;
      end;
    {$endif}
    ' ':
      begin
      Result := tkWhitespace;
      repeat
        Inc(FTokenPos);
        if {$ifdef UsePChar}FTokenPos[0] = #0{$else}FTokenPos>l{$endif} then
          if not FetchLocalLine then
            begin
            FCurToken := Result;
            exit;
            end;
      until not ({$ifdef UsePChar}FTokenPos[0]{$else}s[FTokenPos]{$endif}=' ');
      end;
    #9:
      begin
      Result := tkTab;
      repeat
        Inc(FTokenPos);
        if {$ifdef UsePChar}FTokenPos[0] = #0{$else}FTokenPos>l{$endif} then
          if not FetchLocalLine then
            begin
            FCurToken := Result;
            exit;
            end;
      until not ({$ifdef UsePChar}FTokenPos[0]{$else}s[FTokenPos]{$endif}=#9);
      end;
    '#', '''':
      Result:=DoFetchTextToken;
    '&':
      begin
      TokenStart := FTokenPos;
      repeat
        Inc(FTokenPos);
      until {$ifdef UsePChar}not (FTokenPos[0] in ['0'..'7']){$else}(FTokenPos>l) or not (s[FTokenPos] in ['0'..'7']){$endif};
      SectionLength := FTokenPos - TokenStart;
      if (SectionLength=1)
          and ({$ifdef UsePChar}FTokenPos^{$else}s[FTokenPos]{$endif} in IdentChars) then
        begin
        // &Keyword
        DoFetchToken();
        Result:=tkIdentifier;
        end
      else
        begin
        FetchCurTokenString;
        Result := tkNumber;
        end;
      end;
    '$':
      begin
      TokenStart := FTokenPos;
      repeat
        Inc(FTokenPos);
      until {$ifdef UsePChar}not (FTokenPos[0] in HexDigits){$else}(FTokenPos>l) or not (s[FTokenPos] in HexDigits){$endif};
      SectionLength := FTokenPos - TokenStart;
      FetchCurTokenString;
      Result := tkNumber;
      end;
    '%':
      begin
      TokenStart := FTokenPos;
      repeat
        Inc(FTokenPos);
      until {$ifdef UsePChar}not (FTokenPos[0] in ['0','1']){$else}(FTokenPos>l) or not (s[FTokenPos] in ['0','1']){$endif};
      SectionLength := FTokenPos - TokenStart;
      FetchCurTokenString;
      Result := tkNumber;
      end;
    '(':
      begin
      Inc(FTokenPos);
      if {$ifdef UsePChar}FTokenPos[0] <> '*'{$else}(FTokenPos>l) or (s[FTokenPos]<>'*'){$endif} then
        Result := tkBraceOpen
      else
        begin
        // Old-style multi-line comment
        Inc(FTokenPos);
        TokenStart := FTokenPos;
        FCurTokenString := '';
        {$ifdef UsePChar}
        OldLength := 0;
        {$endif}
        NestingLevel:=0;
        repeat
          if {$ifdef UsePChar}FTokenPos[0] = #0{$else}FTokenPos>l{$endif} then
            begin
            SectionLength:=FTokenPos - TokenStart;
            {$ifdef UsePChar}
            SetLength(FCurTokenString, OldLength + SectionLength+1); // +1 for #10
            if SectionLength > 0 then
              Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
            Inc(OldLength, SectionLength+1);
            FCurTokenString[OldLength] := #10;
            {$else}
            FCurTokenString:=FCurTokenString+copy(FCurLine,TokenStart,SectionLength)+#10;
            {$endif}
            if not FetchLocalLine then
              begin
              Result := tkEOF;
              FCurToken := Result;
              exit;
              end;
            TokenStart:=FTokenPos;
            end
          else if {$ifdef UsePChar}(FTokenPos[0] = '*') and (FTokenPos[1] = ')')
              {$else}(FTokenPos<l) and (s[FTokenPos]='*') and (s[FTokenPos+1]=')'){$endif}
            then begin
            dec(NestingLevel);
            if NestingLevel<0 then
              break;
            inc(FTokenPos,2);
            end
          else if (msNestedComment in CurrentModeSwitches)
              and {$ifdef UsePChar}(FTokenPos[0] = '(') and (FTokenPos[1] = '*')
              {$else}(FTokenPos<l) and (s[FTokenPos]='(') and (s[FTokenPos+1]='*'){$endif}
            then begin
            inc(FTokenPos,2);
            Inc(NestingLevel);
            end
          else
            Inc(FTokenPos);
        until false;
        SectionLength := FTokenPos - TokenStart;
        {$ifdef UsePChar}
        SetLength(FCurTokenString, OldLength + SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
        {$else}
        FCurTokenString:=FCurTokenString+copy(FCurLine,TokenStart,SectionLength);
        {$endif}
        Inc(FTokenPos, 2);
        Result := tkComment;
        if Copy(CurTokenString,1,1)='$' then
          Result := HandleDirective(CurTokenString);
        end;
      end;
    ')':
      begin
      Inc(FTokenPos);
      Result := tkBraceClose;
      end;
    '*':
      begin
      Result:=tkMul;
      Inc(FTokenPos);
      if {$ifdef UsePChar}FTokenPos[0]='*'{$else}(FTokenPos<=l) and (s[FTokenPos]='*'){$endif} then
        begin
        Inc(FTokenPos);
        Result := tkPower;
        end
      else if (po_CAssignments in options) then
        begin
        if {$ifdef UsePChar}FTokenPos[0]='='{$else}(FTokenPos<=l) and (s[FTokenPos]='='){$endif} then
          begin
          Inc(FTokenPos);
          Result:=tkAssignMul;
          end;
        end;
      end;
    '+':
      begin
      Result:=tkPlus;
      Inc(FTokenPos);
      if (po_CAssignments in options) then
        begin
        if {$ifdef UsePChar}FTokenPos[0]='='{$else}(FTokenPos<=l) and (s[FTokenPos]='='){$endif} then
          begin
          Inc(FTokenPos);
          Result:=tkAssignPlus;
          end;
        end
      end;
    ',':
      begin
      Inc(FTokenPos);
      Result := tkComma;
      end;
    '-':
      begin
      Result := tkMinus;
      Inc(FTokenPos);
      if (po_CAssignments in options) then
        begin
        if {$ifdef UsePChar}FTokenPos[0]='='{$else}(FTokenPos<=l) and (s[FTokenPos]='='){$endif} then
          begin
          Inc(FTokenPos);
          Result:=tkAssignMinus;
          end;
        end
      end;
    '.':
      begin
      Inc(FTokenPos);
      if {$ifdef UsePChar}FTokenPos[0]='.'{$else}(FTokenPos<=l) and (s[FTokenPos]='.'){$endif} then
        begin
        Inc(FTokenPos);
        Result := tkDotDot;
        end
      else
        Result := tkDot;
      end;
    '/':
      begin
      Result := tkDivision;
      Inc(FTokenPos);
      if {$ifdef UsePChar}FTokenPos[0]='/'{$else}(FTokenPos<=l) and (s[FTokenPos]='/'){$endif} then
        begin
        // Single-line comment
        Inc(FTokenPos);
        TokenStart := FTokenPos;
        FCurTokenString := '';
        while {$ifdef UsePChar}FTokenPos[0] <> #0{$else}(FTokenPos<=l) and (s[FTokenPos]<>#0){$endif} do
          Inc(FTokenPos);
        SectionLength := FTokenPos - TokenStart;
        FetchCurTokenString;
        // Handle macro which is //
        if FCurSourceFile is TMacroReader then
          begin
          // exhaust till eof of macro stream
          Repeat
            I:=Fetchtoken;
          until (i<>tkLineEnding);
          FetchLocalLine;
          end;
        Result := tkComment;
        end
      else if (po_CAssignments in options) then
        begin
        if {$ifdef UsePChar}FTokenPos[0]='='{$else}(FTokenPos<=l) and (s[FTokenPos]='='){$endif} then
          begin
          Inc(FTokenPos);
          Result:=tkAssignDivision;
          end;
        end
      end;
    '0'..'9':
      begin
      // 1, 12, 1.2, 1.2E3, 1.E2, 1E2, 1.2E-3, 1E+2
      // beware of 1..2
      TokenStart := FTokenPos;
      repeat
        Inc(FTokenPos);
      until {$ifdef UsePChar}not (FTokenPos[0] in Digits){$else}(FTokenPos>l) or not (s[FTokenPos] in Digits){$endif};
      if {$ifdef UsePChar}(FTokenPos[0]='.') and (FTokenPos[1]<>'.'){$else}
          (FTokenPos<=l) and (s[FTokenPos]='.') and ((FTokenPos=l) or (s[FTokenPos+1]<>'.')){$endif}then
        begin
        inc(FTokenPos);
        while {$ifdef UsePChar}FTokenPos[0] in Digits{$else}(FTokenPos<=l) and (s[FTokenPos] in Digits){$endif} do
          Inc(FTokenPos);
        end;
      if {$ifdef UsePChar}FTokenPos[0] in ['e', 'E']{$else}(FTokenPos<=l) and (s[FTokenPos] in ['e', 'E']){$endif} then
      begin
        Inc(FTokenPos);
        if {$ifdef UsePChar}FTokenPos[0] in ['-','+']{$else}(FTokenPos<=l) and (s[FTokenPos] in ['-','+']){$endif} then
          inc(FTokenPos);
        while {$ifdef UsePChar}FTokenPos[0] in Digits{$else}(FTokenPos<=l) and (s[FTokenPos] in Digits){$endif} do
          Inc(FTokenPos);
      end;
      SectionLength := FTokenPos - TokenStart;
      FetchCurTokenString;
      Result := tkNumber;
      end;
    ':':
      begin
      Inc(FTokenPos);
      if {$ifdef UsePChar}FTokenPos[0]='='{$else}(FTokenPos<=l) and (s[FTokenPos]='='){$endif} then
        begin
        Inc(FTokenPos);
        Result := tkAssign;
        end
      else
        Result := tkColon;
      end;
    ';':
      begin
      Inc(FTokenPos);
      Result := tkSemicolon;
      end;
    '<':
      begin
      Inc(FTokenPos);
      {$ifndef UsePChar}
      if FTokenPos>l then
        Result := tkLessThan
      else
      {$endif}
      case {$ifdef UsePChar}FTokenPos^{$else}s[FTokenPos]{$endif} of
      '>':
        begin
        Inc(FTokenPos);
        Result := tkNotEqual;
        end;
      '=':
        begin
        Inc(FTokenPos);
        Result := tkLessEqualThan;
        end;
      '<':
        begin
        Inc(FTokenPos);
        Result := tkshl;
        end;
      else
        Result := tkLessThan;
      end;
      end;
    '=':
      begin
      Inc(FTokenPos);
      Result := tkEqual;
      end;
    '>':
      begin
      Inc(FTokenPos);
      {$ifndef UsePChar}
      if FTokenPos>l then
        Result := tkGreaterThan
      else
      {$endif}
      case {$ifdef UsePChar}FTokenPos^{$else}s[FTokenPos]{$endif} of
      '=':
        begin
        Inc(FTokenPos);
        Result := tkGreaterEqualThan;
        end;
      '<':
        begin
        Inc(FTokenPos);
        Result := tkSymmetricalDifference;
        end;
      '>':
        begin
        Inc(FTokenPos);
        Result := tkshr;
        end;
      else
        Result := tkGreaterThan;
      end;
      end;
    '@':
      begin
      Inc(FTokenPos);
      Result := tkAt;
      if {$ifdef UsePChar}FTokenPos^='@'{$else}(FTokenPos<=l) and (s[FTokenPos]='@'){$endif} then
        begin
        Inc(FTokenPos);
        Result:=tkAtAt;
        end;
      end;
    '[':
      begin
      Inc(FTokenPos);
      Result := tkSquaredBraceOpen;
      end;
    ']':
      begin
      Inc(FTokenPos);
      Result := tkSquaredBraceClose;
      end;
    '^':
      begin
      if ForceCaret or PPisSkipping or
         (PreviousToken in [tkeof,tkTab,tkLineEnding,tkComment,tkIdentifier,
                   tkNil,tkOperator,tkBraceClose,tkSquaredBraceClose,tkCaret,
                   tkWhitespace]) then
        begin
        Inc(FTokenPos);
        Result := tkCaret;
        end
      else
        Result:=DoFetchTextToken;
      end;
    '\':
      begin
      Inc(FTokenPos);
      Result := tkBackslash;
      end;
    '{':        // Multi-line comment
      begin
      Inc(FTokenPos);
      TokenStart := FTokenPos;
      FCurTokenString := '';
      {$ifdef UsePChar}
      OldLength := 0;
      {$endif}
      NestingLevel := 0;
      repeat
        if {$ifdef UsePChar}FTokenPos[0] = #0{$else}FTokenPos>l{$endif} then
          begin
          SectionLength := FTokenPos - TokenStart;
          {$ifdef UsePChar}
          SetLength(FCurTokenString, OldLength + SectionLength+1); // +1 for the #10
          if SectionLength > 0 then
            Move(TokenStart^, FCurTokenString[OldLength + 1],SectionLength);
          Inc(OldLength, SectionLength+1);
          FCurTokenString[OldLength] := #10;
          {$else}
          FCurTokenString:=FCurTokenString+copy(FCurLine,TokenStart,SectionLength)+#10;
          {$endif}
          if not FetchLocalLine then
          begin
            Result := tkEOF;
            FCurToken := Result;
            exit;
          end;
          TokenStart := FTokenPos;
          end
        else if {$ifdef UsePChar}(FTokenPos[0] = '}'){$else}(s[FTokenPos]='}'){$endif} then
          begin
          Dec(NestingLevel);
          if NestingLevel<0 then
            break;
          Inc(FTokenPos);
          end
        else if {$ifdef UsePChar}(FTokenPos[0] = '{'){$else}(s[FTokenPos]='{'){$endif}
            and (msNestedComment in CurrentModeSwitches) then
          begin
          inc(FTokenPos);
          Inc(NestingLevel);
          end
        else
          Inc(FTokenPos);
      until false;
      SectionLength := FTokenPos - TokenStart;
      {$ifdef UsePChar}
      SetLength(FCurTokenString, OldLength + SectionLength);
      if SectionLength > 0 then
        Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
      {$else}
      FCurTokenString:=FCurTokenString+copy(s,TokenStart,SectionLength);
      {$endif}
      Inc(FTokenPos);
      Result := tkComment;
      if (Copy(CurTokenString,1,1)='$') then
        Result:=HandleDirective(CurTokenString);
      end;
    'A'..'Z', 'a'..'z', '_':
      begin
      TokenStart := FTokenPos;
      repeat
        Inc(FTokenPos);
      until {$ifdef UsePChar}not (FTokenPos[0] in IdentChars){$else}(FTokenPos>l) or not (s[FTokenPos] in IdentChars){$endif};
      SectionLength := FTokenPos - TokenStart;
      FetchCurTokenString;
      Result:=tkIdentifier;
      for i:=tkAbsolute to tkXor do
        begin
        if (CompareText(CurTokenString, TokenInfos[i])=0) then
          begin
          Result:=I;
          break;
          end;
        end;
      if (Result<>tkIdentifier) and (Result in FNonTokens) then
        Result:=tkIdentifier;
      FCurToken := Result;
      if MacrosOn then
        begin
        Index:=FMacros.IndexOf(CurTokenString);
        if Index>=0 then
          Result:=HandleMacro(Index);
        end;
      end;
  else
    if PPIsSkipping then
      Inc(FTokenPos)
    else
      Error(nErrInvalidCharacter, SErrInvalidCharacter,
        [{$ifdef UsePChar}FTokenPos[0]{$else}s[FTokenPos]{$endif}]);
  end;

  FCurToken := Result;
end;

function TPascalScanner.LogEvent(E: TPScannerLogEvent): Boolean;
begin
  Result:=E in FLogEvents;
end;

function TPascalScanner.GetCurColumn: Integer;
begin
  If {$ifdef UsePChar}(FTokenPos<>Nil){$else}FTokenPos>0{$endif} then
    Result := FTokenPos {$ifdef UsePChar}- PChar(CurLine){$else}-1{$endif} + FCurColumnOffset
  else
    Result := FCurColumnOffset;
end;

function TPascalScanner.GetCurrentValueSwitch(V: TValueSwitch): string;
begin
  Result:=FCurrentValueSwitches[V];
end;

function TPascalScanner.GetForceCaret: Boolean;
begin
  Result:=toForceCaret in FTokenOptions;
end;

function TPascalScanner.GetMacrosOn: boolean;
begin
  Result:=bsMacro in FCurrentBoolSwitches;
end;

function TPascalScanner.IndexOfWarnMsgState(Number: integer; InsertPos: boolean
  ): integer;
var
  l, r, m, CurNumber: Integer;
begin
  l:=0;
  r:=length(FWarnMsgStates)-1;
  m:=0;
  while l<=r do
    begin
    m:=(l+r) div 2;
    CurNumber:=FWarnMsgStates[m].Number;
    if Number>CurNumber then
      l:=m+1
    else if Number<CurNumber then
      r:=m-1
    else
      exit(m);
    end;
  if not InsertPos then
    exit(-1);
  if length(FWarnMsgStates)=0 then
    exit(0);
  if (m<length(FWarnMsgStates)) and (FWarnMsgStates[m].Number<=Number) then
    inc(m);
  Result:=m;
end;

function TPascalScanner.OnCondEvalFunction(Sender: TCondDirectiveEvaluator;
  Name, Param: String; out Value: string): boolean;
begin
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TPascalScanner.OnCondEvalFunction Func="',Name,'" Param="',Param,'"');
  {$ENDIF}
  if CompareText(Name,'defined')=0 then
    begin
    if not IsValidIdent(Param) then
      Sender.Log(mtError,nErrXExpectedButYFound,SErrXExpectedButYFound,
        ['identifier',Param]);
    Value:=CondDirectiveBool[IsDefined(Param)];
    exit(true);
    end
  else if CompareText(Name,'undefined')=0 then
    begin
    if not IsValidIdent(Param) then
      Sender.Log(mtError,nErrXExpectedButYFound,SErrXExpectedButYFound,
        ['identifier',Param]);
    Value:=CondDirectiveBool[not IsDefined(Param)];
    exit(true);
    end
  else if CompareText(Name,'option')=0 then
    begin
    if (length(Param)<>1) or not (Param[1] in ['a'..'z','A'..'Z']) then
      Sender.Log(mtError,nErrXExpectedButYFound,SErrXExpectedButYFound,
        ['letter',Param]);
    Value:=CondDirectiveBool[IfOpt(Param[1])];
    exit(true);
    end;
  // last check user hook
  if Assigned(OnEvalFunction) then
    begin
    Result:=OnEvalFunction(Sender,Name,Param,Value);
    if not (po_CheckCondFunction in Options) then
      begin
      Value:='0';
      Result:=true;
      end;
    exit;
    end;
  if (po_CheckCondFunction in Options) then
    begin
    Value:='';
    Result:=false;
    end
  else
    begin
    Value:='0';
    Result:=true;
    end;
end;

procedure TPascalScanner.OnCondEvalLog(Sender: TCondDirectiveEvaluator;
  Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
begin
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TPascalScanner.OnCondEvalLog "',Sender.MsgPattern,'"');
  {$ENDIF}
  // ToDo: move CurLine/CurRow to Sender.MsgPos
  if Sender.MsgType<=mtError then
    begin
    SetCurMsg(Sender.MsgType,Sender.MsgNumber,Sender.MsgPattern,Args);
    raise EScannerError.Create(FLastMsg);
    end
  else
    DoLog(Sender.MsgType,Sender.MsgNumber,Sender.MsgPattern,Args,true);
end;

function TPascalScanner.OnCondEvalVar(Sender: TCondDirectiveEvaluator;
  Name: String; out Value: string): boolean;
var
  i: Integer;
  M: TMacroDef;
begin
  {$IFDEF VerbosePasDirectiveEval}
  writeln('TPascalScanner.OnCondEvalVar "',Name,'"');
  {$ENDIF}
  // first check defines
  if FDefines.IndexOf(Name)>=0 then
    begin
    Value:='1';
    exit(true);
    end;
  // then check macros
  i:=FMacros.IndexOf(Name);
  if i>=0 then
    begin
    M:=FMacros.Objects[i] as TMacroDef;
    Value:=M.Value;
    exit(true);
    end;
  // last check user hook
  if Assigned(OnEvalVariable) then
    begin
    Result:=OnEvalVariable(Sender,Name,Value);
    exit;
    end;
  Value:='';
  Result:=false;
end;

procedure TPascalScanner.SetAllowedBoolSwitches(const AValue: TBoolSwitches);
begin
  if FAllowedBoolSwitches=AValue then Exit;
  FAllowedBoolSwitches:=AValue;
end;

procedure TPascalScanner.SetAllowedModeSwitches(const AValue: TModeSwitches);
begin
  if FAllowedModeSwitches=AValue then Exit;
  FAllowedModeSwitches:=AValue;
  CurrentModeSwitches:=FCurrentModeSwitches*AllowedModeSwitches;
end;

procedure TPascalScanner.SetAllowedValueSwitches(const AValue: TValueSwitches);
begin
  if FAllowedValueSwitches=AValue then Exit;
  FAllowedValueSwitches:=AValue;
end;

procedure TPascalScanner.SetCurrentBoolSwitches(const AValue: TBoolSwitches);
var
  OldBS, Removed, Added: TBoolSwitches;
begin
  if FCurrentBoolSwitches=AValue then Exit;
  OldBS:=FCurrentBoolSwitches;
  FCurrentBoolSwitches:=AValue;
  Removed:=OldBS-FCurrentBoolSwitches;
  Added:=FCurrentBoolSwitches-OldBS;
  if bsGoto in Added then
    begin
    UnsetNonToken(tklabel);
    UnsetNonToken(tkgoto);
    end;
  if bsGoto in Removed then
    begin
    SetNonToken(tklabel);
    SetNonToken(tkgoto);
    end;
end;

procedure TPascalScanner.SetCurrentModeSwitches(AValue: TModeSwitches);
var
  Old, AddedMS, RemovedMS: TModeSwitches;
begin
  AValue:=AValue*AllowedModeSwitches;
  if FCurrentModeSwitches=AValue then Exit;
  Old:=FCurrentModeSwitches;
  FCurrentModeSwitches:=AValue;
  AddedMS:=FCurrentModeSwitches-Old;
  RemovedMS:=Old-FCurrentModeSwitches;
  if msDefaultUnicodestring in AddedMS then
    begin
    AddDefine('UNICODE');
    AddDefine('FPC_UNICODESTRINGS');
    end
  else if msDefaultUnicodestring in RemovedMS then
    begin
    UnDefine('UNICODE');
    UnDefine('FPC_UNICODESTRINGS');
    end;
  if msDefaultAnsistring in AddedMS then
    begin
    AddDefine(LetterSwitchNames['H'],true);
    Include(FCurrentBoolSwitches,bsLongStrings);
    end
  else if msDefaultAnsistring in RemovedMS then
    begin
    UnDefine(LetterSwitchNames['H'],true);
    Exclude(FCurrentBoolSwitches,bsLongStrings);
    end;
end;

procedure TPascalScanner.SetCurrentValueSwitch(V: TValueSwitch;
  const AValue: string);
begin
  if not (V in AllowedValueSwitches) then exit;
  if FCurrentValueSwitches[V]=AValue then exit;
  FCurrentValueSwitches[V]:=AValue;
end;

procedure TPascalScanner.SetWarnMsgState(Number: integer; State: TWarnMsgState);

  {$IFDEF EmulateArrayInsert}
  procedure Delete(var A: TWarnMsgNumberStateArr; Index, Count: integer); overload;
  var
    i: Integer;
  begin
    if Index<0 then
      Error(nErrDivByZero,'[20180627142123]');
    if Index+Count>length(A) then
      Error(nErrDivByZero,'[20180627142127]');
    for i:=Index+Count to length(A)-1 do
      A[i-Count]:=A[i];
    SetLength(A,length(A)-Count);
  end;

  procedure Insert(Item: TWarnMsgNumberState; var A: TWarnMsgNumberStateArr; Index: integer); overload;
  var
    i: Integer;
  begin
    if Index<0 then
      Error(nErrDivByZero,'[20180627142133]');
    if Index>length(A) then
      Error(nErrDivByZero,'[20180627142137]');
    SetLength(A,length(A)+1);
    for i:=length(A)-1 downto Index+1 do
      A[i]:=A[i-1];
    A[Index]:=Item;
  end;
  {$ENDIF}

var
  i: Integer;
  Item: TWarnMsgNumberState;
begin
  i:=IndexOfWarnMsgState(Number,true);
  if (i<length(FWarnMsgStates)) and (FWarnMsgStates[i].Number=Number) then
    begin
    // already exists
    if State=wmsDefault then
      Delete(FWarnMsgStates,i,1)
    else
      FWarnMsgStates[i].State:=State;
    end
  else if State<>wmsDefault then
    begin
    // new state
    Item.Number:=Number;
    Item.State:=State;
    Insert(Item,FWarnMsgStates,i);
    end;
end;

function TPascalScanner.GetWarnMsgState(Number: integer): TWarnMsgState;
var
  i: Integer;
begin
  i:=IndexOfWarnMsgState(Number,false);
  if i<0 then
    Result:=wmsDefault
  else
    Result:=FWarnMsgStates[i].State;
end;

procedure TPascalScanner.SetMacrosOn(const AValue: boolean);
begin
  if AValue then
    Include(FCurrentBoolSwitches,bsMacro)
  else
    Exclude(FCurrentBoolSwitches,bsMacro);
end;

procedure TPascalScanner.DoLog(MsgType: TMessageType; MsgNumber: integer;
  const Msg: String; SkipSourceInfo: Boolean);
begin
  DoLog(MsgType,MsgNumber,Msg,[],SkipSourceInfo);
end;

procedure TPascalScanner.DoLog(MsgType: TMessageType; MsgNumber: integer;
  const Fmt: String; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
  SkipSourceInfo: Boolean);

Var
  Msg : String;

begin
  if IgnoreMsgType(MsgType) then exit;
  SetCurMsg(MsgType,MsgNumber,Fmt,Args);
  If Assigned(FOnLog) then
    begin
    Msg:=MessageTypeNames[MsgType]+': ';
    if SkipSourceInfo then
      Msg:=Msg+FLastMsg
    else
      Msg:=Msg+Format('%s(%d,%d) : %s',[FormatPath(FCurFileName),CurRow,CurColumn,FLastMsg]);
    FOnLog(Self,Msg);
    end;
end;

procedure TPascalScanner.SetOptions(AValue: TPOptions);

Var
  isModeSwitch : Boolean;

begin
  if FOptions=AValue then Exit;
  // Change of mode ?
  IsModeSwitch:=(po_delphi in Avalue) <> (po_delphi in FOptions);
  FOptions:=AValue;
  if isModeSwitch then
    if (po_delphi in FOptions) then
      CurrentModeSwitches:=DelphiModeSwitches
    else
      CurrentModeSwitches:=FPCModeSwitches
end;

procedure TPascalScanner.SetReadOnlyBoolSwitches(const AValue: TBoolSwitches);
begin
  if FReadOnlyBoolSwitches=AValue then Exit;
  FReadOnlyBoolSwitches:=AValue;
end;

procedure TPascalScanner.SetReadOnlyModeSwitches(const AValue: TModeSwitches);
begin
  if FReadOnlyModeSwitches=AValue then Exit;
  FReadOnlyModeSwitches:=AValue;
  FAllowedModeSwitches:=FAllowedModeSwitches+FReadOnlyModeSwitches;
  FCurrentModeSwitches:=FCurrentModeSwitches+FReadOnlyModeSwitches;
end;

procedure TPascalScanner.SetReadOnlyValueSwitches(const AValue: TValueSwitches);
begin
  if FReadOnlyValueSwitches=AValue then Exit;
  FReadOnlyValueSwitches:=AValue;
end;

function TPascalScanner.IndexOfResourceHandler(const aExt: string): Integer;

begin
  Result:=Length(FResourceHandlers)-1;
  While (Result>=0) and (FResourceHandlers[Result].Ext<>aExt) do
    Dec(Result);
end;

function TPascalScanner.FindResourceHandler(const aExt: string): TResourceHandler;

Var
  Idx : Integer;

begin
  Idx:=IndexOfResourceHandler(aExt);
  if Idx=-1 then
    Result:=Nil
  else
    Result:=FResourceHandlers[Idx].handler;
end;

function TPascalScanner.ReadIdentifier(const AParam: string): string;
var
  p, l: Integer;
begin
  p:=1;
  l:=length(AParam);
  while (p<=l) and (AParam[p] in IdentChars) do inc(p);
  Result:=LeftStr(AParam,p-1);
end;

function TPascalScanner.FetchLine: boolean;
begin
  if CurSourceFile.IsEOF then
  begin
    if {$ifdef UsePChar}FTokenPos<>nil{$else}FTokenPos>0{$endif} then
      begin
      FCurLine := '';
      FTokenPos := {$ifdef UsePChar}nil{$else}-1{$endif};
      inc(FCurRow); // set CurRow to last line+1
      inc(FModuleRow);
      FCurColumnOffset:=1;
      end;
    Result := false;
  end else
  begin
    FCurLine := CurSourceFile.ReadLine;
    FTokenPos := {$ifdef UsePChar}PChar(CurLine){$else}1{$endif};
    Result := true;
    {$ifdef UseAnsiStrings}
    if (FCurRow = 0)
    and (Length(CurLine) >= 3)
    and (FTokenPos[0] = #$EF)
    and (FTokenPos[1] = #$BB)
    and (FTokenPos[2] = #$BF) then
      // ignore UTF-8 Byte Order Mark
      inc(FTokenPos, 3);
    {$endif}
    Inc(FCurRow);
    inc(FModuleRow);
    FCurColumnOffset:=1;
    if (FCurSourceFile is TMacroReader) and (FCurRow=1) then
    begin
      FCurRow:=TMacroReader(FCurSourceFile).CurRow;
      FCurColumnOffset:=TMacroReader(FCurSourceFile).CurCol;
    end;
    if LogEvent(sleLineNumber)
        and (((FCurRow Mod 100) = 0)
          or CurSourceFile.IsEOF) then
      DoLog(mtInfo,nLogLineNumber,SLogLineNumber,[FCurRow],True); // log last line
  end;
end;

procedure TPascalScanner.AddFile(aFilename: string);
var
  i: Integer;
begin
  for i:=0 to FFiles.Count-1 do
    if FFiles[i]=aFilename then exit;
  FFiles.Add(aFilename);
end;

function TPascalScanner.GetMacroName(const Param: String): String;
var
  p: Integer;
begin
  Result:=Trim(Param);
  p:=1;
  while (p<=length(Result)) and (Result[p] in ['a'..'z','A'..'Z','0'..'9','_']) do
    inc(p);
  SetLength(Result,p-1);
end;

procedure TPascalScanner.SetCurMsg(MsgType: TMessageType; MsgNumber: integer;
  const Fmt: String; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
begin
  FLastMsgType := MsgType;
  FLastMsgNumber := MsgNumber;
  FLastMsgPattern := Fmt;
  FLastMsg := SafeFormat(Fmt,Args);
  CreateMsgArgs(FLastMsgArgs,Args);
end;

function TPascalScanner.AddDefine(const aName: String; Quiet: boolean): boolean;

begin
  If FDefines.IndexOf(aName)>=0 then exit(false);
  Result:=true;
  FDefines.Add(aName);
  if (not Quiet) and LogEvent(sleConditionals) then
    DoLog(mtInfo,nLogMacroDefined,sLogMacroDefined,[aName])
end;

function TPascalScanner.RemoveDefine(const aName: String; Quiet: boolean
  ): boolean;

Var
  I : Integer;

begin
  I:=FDefines.IndexOf(aName);
  if (I<0) then exit(false);
  Result:=true;
  FDefines.Delete(I);
  if (not Quiet) and LogEvent(sleConditionals) then
    DoLog(mtInfo,nLogMacroUnDefined,sLogMacroUnDefined,[aName])
end;

function TPascalScanner.UnDefine(const aName: String; Quiet: boolean): boolean;
begin
  // Important: always call both, do not use OR
  Result:=RemoveDefine(aName,Quiet);
  if RemoveMacro(aName,Quiet) then Result:=true;
end;

function TPascalScanner.IsDefined(const aName: String): boolean;
begin
  Result:=(FDefines.IndexOf(aName)>=0) or (FMacros.IndexOf(aName)>=0);
end;

function TPascalScanner.IfOpt(Letter: Char): boolean;
begin
  Letter:=upcase(Letter);
  Result:=(Letter in ['A'..'Z']) and (LetterSwitchNames[Letter]<>'')
    and IsDefined(LetterSwitchNames[Letter]);
end;

function TPascalScanner.AddMacro(const aName, aValue: String; Quiet: boolean
  ): boolean;
var
  Index: Integer;
begin
  Index:=FMacros.IndexOf(aName);
  If (Index=-1) then
    FMacros.AddObject(aName,TMacroDef.Create(aName,aValue))
  else
    begin
    if TMacroDef(FMacros.Objects[Index]).Value=aValue then exit(false);
    TMacroDef(FMacros.Objects[Index]).Value:=aValue;
    end;
  Result:=true;
  if (not Quiet) and LogEvent(sleConditionals) then
    DoLog(mtInfo,nLogMacroXSetToY,SLogMacroXSetToY,[aName,aValue])
end;

function TPascalScanner.RemoveMacro(const aName: String; Quiet: boolean
  ): boolean;
var
  Index: Integer;
begin
  Index:=FMacros.IndexOf(aName);
  if Index<0 then exit(false);
  Result:=true;
  TMacroDef(FMacros.Objects[Index]).{$ifdef pas2js}Destroy{$else}Free{$endif};
  FMacros.Delete(Index);
  if (not Quiet) and LogEvent(sleConditionals) then
    DoLog(mtInfo,nLogMacroUnDefined,sLogMacroUnDefined,[aName])
end;

procedure TPascalScanner.SetCompilerMode(S: String);
begin
  HandleMode(S);
end;

function TPascalScanner.CurSourcePos: TPasSourcePos;
begin
  Result.FileName:=CurFilename;
  Result.Row:=CurRow;
  Result.Column:=CurColumn;
end;

function TPascalScanner.SetForceCaret(AValue: Boolean): Boolean;

begin
  Result:=toForceCaret in FTokenOptions;
  if aValue then
    Include(FTokenOptions,toForceCaret)
  else
    Exclude(FTokenOptions,toForceCaret)
end;

function TPascalScanner.IgnoreMsgType(MsgType: TMessageType): boolean;
begin
  case MsgType of
    mtWarning: if not (bsWarnings in FCurrentBoolSwitches) then exit(true);
    mtNote: if not (bsNotes in FCurrentBoolSwitches) then exit(true);
    mtHint: if not (bsHints in FCurrentBoolSwitches) then exit(true);
  end;
  Result:=false;
end;

end.
