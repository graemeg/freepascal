{
    This file is part of the Free Component Library

    Pascal source parser
    Copyright (c) 2000-2005 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit PParser;

{$i fcl-passrc.inc}

interface

uses
  {$ifdef NODEJS}
  Node.FS,
  {$endif}
  SysUtils, Classes, Types, PasTree, PScanner;

// message numbers
const
  nErrNoSourceGiven = 2001;
  nErrMultipleSourceFiles = 2002;
  nParserError = 2003;
  nParserErrorAtToken = 2004;
  nParserUngetTokenError = 2005;
  nParserExpectTokenError = 2006;
  nParserForwardNotInterface = 2007;
  nParserExpectVisibility = 2008;
  nParserStrangeVisibility = 2009;
  nParserExpectToken2Error = 2010;
  nParserExpectedCommaRBracket = 2011;
  nParserExpectedCommaSemicolon = 2012;
  nParserExpectedAssignIn = 2013;
  nParserExpectedCommaColon = 2014;
  nErrUnknownOperatorType = 2015;
  nParserOnlyOneArgumentCanHaveDefault = 2016;
  nParserExpectedLBracketColon = 2017;
  nParserExpectedSemiColonEnd = 2018;
  nParserExpectedConstVarID = 2019;
  nParserExpectedNested = 2020;
  nParserExpectedColonID = 2021;
  nParserSyntaxError = 2022;
  nParserTypeSyntaxError = 2023;
  nParserArrayTypeSyntaxError = 2024;
  nParserExpectedIdentifier = 2026;
  nParserNotAProcToken = 2026;
  nRangeExpressionExpected = 2027;
  nParserExpectCase = 2028;
  nParserGenericFunctionNeedsGenericKeyword = 2029;
  nLogStartImplementation = 2030;
  nLogStartInterface = 2031;
  nParserNoConstructorAllowed = 2032;
  nParserNoFieldsAllowed = 2033;
  nParserInvalidRecordVisibility = 2034;
  nErrRecordConstantsNotAllowed = 2035;
  nErrRecordMethodsNotAllowed = 2036;
  nErrRecordPropertiesNotAllowed = 2037;
  nErrRecordTypesNotAllowed = 2038;
  nParserTypeNotAllowedHere = 2039;
  nParserNotAnOperand = 2040;
  nParserArrayPropertiesCannotHaveDefaultValue = 2041;
  nParserDefaultPropertyMustBeArray = 2042;
  nParserUnknownProcedureType = 2043;
  nParserGenericArray1Element = 2044;
  nParserTypeParamsNotAllowedOnType = 2045;
  nParserDuplicateIdentifier = 2046;
  nParserDefaultParameterRequiredFor = 2047;
  nParserOnlyOneVariableCanBeInitialized = 2048;
  nParserExpectedTypeButGot = 2049;
  nParserPropertyArgumentsCanNotHaveDefaultValues = 2050;
  nParserExpectedExternalClassName = 2051;
  nParserNoConstRangeAllowed = 2052;
  nErrRecordVariablesNotAllowed = 2053;
  nParserResourcestringsMustBeGlobal = 2054;
  nParserOnlyOneVariableCanBeAbsolute = 2055;
  nParserXNotAllowedInY = 2056;
  nFileSystemsNotSupported = 2057;
  nInvalidMessageType = 2058;

// resourcestring patterns of messages
resourcestring
  SErrNoSourceGiven = 'No source file specified';
  SErrMultipleSourceFiles = 'Please specify only one source file';
  SParserError = 'Error';
  SParserErrorAtToken = '%s at token "%s" in file %s at line %d column %d';
  SParserUngetTokenError = 'Internal error: Cannot unget more tokens, history buffer is full';
  SParserExpectTokenError = 'Expected "%s"';
  SParserForwardNotInterface = 'The use of a FORWARD procedure modifier is not allowed in the interface';
  SParserExpectVisibility = 'Expected visibility specifier';
  SParserStrangeVisibility = 'Strange strict visibility encountered : "%s"';
  SParserExpectToken2Error = 'Expected "%s" or "%s"';
  SParserExpectedCommaRBracket = 'Expected "," or ")"';
  SParserExpectedCommaSemicolon = 'Expected "," or ";"';
  SParserExpectedAssignIn = 'Expected := or in';
  SParserExpectedCommaColon = 'Expected "," or ":"';
  SErrUnknownOperatorType = 'Unknown operator type: %s';
  SParserOnlyOneArgumentCanHaveDefault = 'A default value can only be assigned to 1 parameter';
  SParserExpectedLBracketColon = 'Expected "(" or ":"';
  SParserExpectedSemiColonEnd = 'Expected ";" or "End"';
  SParserExpectedConstVarID = 'Expected "const", "var" or identifier';
  SParserExpectedNested = 'Expected nested keyword';
  SParserExpectedColonID = 'Expected ":" or identifier';
  SParserSyntaxError = 'Syntax error';
  SParserTypeSyntaxError = 'Syntax error in type';
  SParserArrayTypeSyntaxError = 'Syntax error in array type';
  SParserExpectedIdentifier = 'Identifier expected';
  SParserNotAProcToken = 'Not a procedure or function token';
  SRangeExpressionExpected = 'Range expression expected';
  SParserExpectCase = 'Case label expression expected';
  SParserGenericFunctionNeedsGenericKeyword = 'Generic function needs keyword generic';
  SLogStartImplementation = 'Start parsing implementation section.';
  SLogStartInterface = 'Start parsing interface section';
  SParserNoConstructorAllowed = 'Constructors or Destructors are not allowed in Interfaces or Records';
  SParserNoFieldsAllowedInX = 'Fields are not allowed in %s';
  SParserInvalidRecordVisibility = 'Records can only have public and (strict) private as visibility specifiers';
  SErrRecordConstantsNotAllowed = 'Record constants not allowed at this location';
  SErrRecordVariablesNotAllowed = 'Record variables not allowed at this location';
  SErrRecordMethodsNotAllowed = 'Record methods not allowed at this location';
  SErrRecordPropertiesNotAllowed = 'Record properties not allowed at this location';
  SErrRecordTypesNotAllowed = 'Record types not allowed at this location';
  SParserTypeNotAllowedHere = 'Type "%s" not allowed here';
  SParserNotAnOperand = 'Not an operand: (%d : %s)';
  SParserArrayPropertiesCannotHaveDefaultValue = 'Array properties cannot have default value';
  SParserDefaultPropertyMustBeArray = 'The default property must be an array property';
  SParserUnknownProcedureType = 'Unknown procedure type "%d"';
  SParserGenericArray1Element = 'Generic arrays can have only 1 template element';
  SParserTypeParamsNotAllowedOnType = 'Type parameters not allowed on this type';
  SParserDuplicateIdentifier = 'Duplicate identifier "%s"';
  SParserDefaultParameterRequiredFor = 'Default parameter required for "%s"';
  SParserOnlyOneVariableCanBeInitialized = 'Only one variable can be initialized';
  SParserExpectedTypeButGot = 'Expected type, but got %s';
  SParserPropertyArgumentsCanNotHaveDefaultValues = 'Property arguments can not have default values';
  SParserExpectedExternalClassName = 'Expected external class name';
  SParserNoConstRangeAllowed = 'Const ranges are not allowed';
  SParserResourcestringsMustBeGlobal = 'Resourcestrings can be only static or global';
  SParserOnlyOneVariableCanBeAbsolute = 'Only one variable can be absolute';
  SParserXNotAllowedInY = '%s is not allowed in %s';
  SErrFileSystemNotSupported = 'No support for filesystems enabled';
  SErrInvalidMessageType = 'Invalid message type: string or integer expression expected';

type
  TPasScopeType = (
    stModule,  // e.g. unit, program, library
    stUsesClause,
    stTypeSection,
    stTypeDef, // e.g. a TPasType
    stResourceString, // e.g. TPasResString
    stProcedure, // also method, procedure, constructor, destructor, ...
    stProcedureHeader,
    stSpecializeType, // calls BeginScope to resolve c in a<b>.c
    stWithExpr, // calls BeginScope after parsing every WITH-expression
    stExceptOnExpr,
    stExceptOnStatement,
    stForLoopHeader,
    stDeclaration, // e.g. a TPasProperty, TPasVariable, TPasArgument, ...
    stAncestors, // the list of ancestors and interfaces of a class
    stInitialFinalization
    );
  TPasScopeTypes = set of TPasScopeType;

  TPasParserLogHandler = Procedure (Sender : TObject; Const Msg : String) of object;
  TPParserLogEvent = (pleInterface,pleImplementation);
  TPParserLogEvents = set of TPParserLogEvent;
  TPasParser = Class;

  { TPasTreeContainer }

  TPasTreeContainer = class
  private
    FCurrentParser: TPasParser;
    FNeedComments: Boolean;
    FOnLog: TPasParserLogHandler;
    FPParserLogEvents: TPParserLogEvents;
    FScannerLogEvents: TPScannerLogEvents;
  protected
    FPackage: TPasPackage;
    FInterfaceOnly : Boolean;
    procedure SetCurrentParser(AValue: TPasParser); virtual;
  public
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; const ASourceFilename: String;
      ASourceLinenumber: Integer): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;overload;
      virtual; abstract;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASrcPos: TPasSourcePos; TypeParams: TFPList = nil): TPasElement; overload;
      virtual;
    function CreateFunctionType(const AName, AResultName: String; AParent: TPasElement;
      UseParentAsResultParent: Boolean; const ASrcPos: TPasSourcePos; TypeParams: TFPList = nil): TPasFunctionType;
    function FindElement(const AName: String): TPasElement; virtual; abstract;
    function FindElementFor(const AName: String; AParent: TPasElement; TypeParamCount: integer): TPasElement; virtual;
    procedure BeginScope(ScopeType: TPasScopeType; El: TPasElement); virtual;
    procedure FinishScope(ScopeType: TPasScopeType; El: TPasElement); virtual;
    procedure FinishTypeAlias(var aType: TPasType); virtual;
    function FindModule(const AName: String): TPasModule; virtual;
    function FindModule(const AName: String; NameExpr, InFileExpr: TPasExpr): TPasModule; virtual;
    function CheckPendingUsedInterface(Section: TPasSection): boolean; virtual; // true if changed
    function NeedArrayValues(El: TPasElement): boolean; virtual;
    function GetDefaultClassVisibility(AClass: TPasClassType): TPasMemberVisibility; virtual;
    procedure ModeChanged(Sender: TObject; NewMode: TModeSwitch;
      Before: boolean; var Handled: boolean); virtual;
    property Package: TPasPackage read FPackage;
    property InterfaceOnly : Boolean Read FInterfaceOnly Write FInterFaceOnly;
    property ScannerLogEvents : TPScannerLogEvents Read FScannerLogEvents Write FScannerLogEvents;
    property ParserLogEvents : TPParserLogEvents Read FPParserLogEvents Write FPParserLogEvents;
    property OnLog : TPasParserLogHandler Read FOnLog Write FOnLog;
    property CurrentParser : TPasParser Read FCurrentParser Write SetCurrentParser;
    property NeedComments : Boolean Read FNeedComments Write FNeedComments;
  end;

  EParserError = class(Exception)
  private
    FFilename: String;
    FRow, FColumn: Integer;
  public
    constructor Create(const AReason, AFilename: String;
      ARow, AColumn: Integer); reintroduce;
    property Filename: String read FFilename;
    property Row: Integer read FRow;
    property Column: Integer read FColumn;
  end;

  TExprKind = (ek_Normal, ek_PropertyIndex);
  TIndentAction = (iaNone,iaIndent,iaUndent);

  { TPasParser }

  TPasParser = class
  private
    const FTokenRingSize = 32;
    type
      TTokenRec = record
        Token: TToken;
        AsString: String;
        Comments: TStrings;
        SourcePos: TPasSourcePos;
        TokenPos: TPasSourcePos;
      end;
      PTokenRec = ^TTokenRec;
  private
    FCurModule: TPasModule;
    FFileResolver: TBaseFileResolver;
    FImplicitUses: TStrings;
    FLastMsg: string;
    FLastMsgArgs: TMessageArgs;
    FLastMsgNumber: integer;
    FLastMsgPattern: string;
    FLastMsgType: TMessageType;
    FLogEvents: TPParserLogEvents;
    FOnLog: TPasParserLogHandler;
    FOptions: TPOptions;
    FScanner: TPascalScanner;
    FEngine: TPasTreeContainer;
    FCurToken: TToken;
    FCurTokenString: String;
    FSavedComments : String;
    // UngetToken support:
    FTokenRing: array[0..FTokenRingSize-1] of TTokenRec;
    FTokenRingCur: Integer; // index of current token in FTokenBuffer
    FTokenRingStart: Integer; // first valid ring index in FTokenBuffer, if FTokenRingStart=FTokenRingEnd the ring is empty
    FTokenRingEnd: Integer; // first invalid ring index in FTokenBuffer
    {$ifdef VerbosePasParser}
    FDumpIndent : String;
    procedure DumpCurToken(Const Msg : String; IndentAction : TIndentAction = iaNone);
    {$endif}
    function CheckOverloadList(AList: TFPList; AName: String; out OldMember: TPasElement): TPasOverloadedProc;
    function DoCheckHint(Element: TPasElement): Boolean;
    function GetCurrentModeSwitches: TModeSwitches;
    Procedure SetCurrentModeSwitches(AValue: TModeSwitches);
    function GetVariableModifiers(Parent: TPasElement;
      Out VarMods: TVariableModifiers; Out LibName, ExportName: TPasExpr;
      const AllowedMods: TVariableModifiers): string;
    function GetVariableValueAndLocation(Parent : TPasElement; Out Value: TPasExpr; Out AbsoluteExpr: TPasExpr; Out Location: String): Boolean;
    procedure HandleProcedureModifier(Parent: TPasElement; pm : TProcedureModifier);
    procedure HandleProcedureTypeModifier(ProcType: TPasProcedureType; ptm : TProcTypeModifier);
    procedure ParseMembersLocalConsts(AType: TPasMembersType; AVisibility: TPasMemberVisibility);
    procedure ParseMembersLocalTypes(AType: TPasMembersType; AVisibility: TPasMemberVisibility);
    procedure ParseVarList(Parent: TPasElement; VarList: TFPList; AVisibility: TPasMemberVisibility; Full: Boolean);
    procedure SetOptions(AValue: TPOptions);
    procedure OnScannerModeChanged(Sender: TObject; NewMode: TModeSwitch;
      Before: boolean; var Handled: boolean);
  protected
    Function SaveComments : String;
    Function SaveComments(Const AValue : String) : String;
    function LogEvent(E : TPParserLogEvent) : Boolean; inline;
    Procedure DoLog(MsgType: TMessageType; MsgNumber: integer; Const Msg : String; SkipSourceInfo : Boolean = False);overload;
    Procedure DoLog(MsgType: TMessageType; MsgNumber: integer; Const Fmt : String; Args : Array of {$ifdef pas2js}jsvalue{$else}const{$endif};SkipSourceInfo : Boolean = False);overload;
    function GetProcTypeFromToken(tk: TToken; IsClass: Boolean=False ): TProcType;
    procedure ParseAsmBlock(AsmBlock: TPasImplAsmStatement); virtual;
    procedure ParseRecordMembers(ARec: TPasRecordType; AEndToken: TToken; AllowMethods : Boolean);
    procedure ParseRecordVariantParts(ARec: TPasRecordType; AEndToken: TToken);
    function GetProcedureClass(ProcType : TProcType): TPTreeElement;
    procedure ParseClassFields(AType: TPasClassType; const AVisibility: TPasMemberVisibility; IsClassField : Boolean);
    procedure ParseClassMembers(AType: TPasClassType);
    procedure ProcessMethod(AType: TPasClassType; IsClass : Boolean; AVisibility : TPasMemberVisibility; MustBeGeneric: boolean);
    procedure ReadGenericArguments(List: TFPList; Parent: TPasElement);
    procedure ReadSpecializeArguments(Parent: TPasElement; Params: TFPList);
    function ReadDottedIdentifier(Parent: TPasElement; out Expr: TPasExpr; NeedAsString: boolean): String;
    function CheckProcedureArgs(Parent: TPasElement;
      Args: TFPList; // list of TPasArgument
      ProcType: TProcType): boolean;
    function CheckVisibility(S: String; var AVisibility: TPasMemberVisibility; IsObjCProtocol : Boolean = False): Boolean;
    procedure ParseExc(MsgNumber: integer; const Msg: String);
    procedure ParseExc(MsgNumber: integer; const Fmt: String; Args : Array of {$ifdef pas2js}jsvalue{$else}const{$endif});
    procedure ParseExcExpectedIdentifier;
    procedure ParseExcSyntaxError;
    procedure ParseExcTokenError(const Arg: string);
    procedure ParseExcTypeParamsNotAllowed;
    procedure ParseExcExpectedAorB(const A, B: string);
    function OpLevel(t: TToken): Integer;
    Function TokenToExprOp (AToken : TToken) : TExprOpCode;
    function CreateElement(AClass: TPTreeElement; const AName: String; AParent: TPasElement): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String; AParent: TPasElement; const ASrcPos: TPasSourcePos): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility; const ASrcPos: TPasSourcePos; TypeParams: TFPList = nil): TPasElement;overload;
    function CreatePrimitiveExpr(AParent: TPasElement; AKind: TPasExprKind; const AValue: String): TPrimitiveExpr;
    function CreateBoolConstExpr(AParent: TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean): TBoolConstExpr;
    function CreateBinaryExpr(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode): TBinaryExpr; overload;
    function CreateBinaryExpr(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode; const ASrcPos: TPasSourcePos): TBinaryExpr; overload;
    procedure AddToBinaryExprChain(var ChainFirst: TPasExpr;
      Element: TPasExpr; AOpCode: TExprOpCode; const ASrcPos: TPasSourcePos);
    {$IFDEF VerbosePasParser}
    procedure WriteBinaryExprChain(Prefix: string; First, Last: TPasExpr);
    {$ENDIF}
    function CreateUnaryExpr(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode): TUnaryExpr; overload;
    function CreateUnaryExpr(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode; const ASrcPos: TPasSourcePos): TUnaryExpr; overload;
    function CreateArrayValues(AParent : TPasElement): TArrayValues;
    function CreateFunctionType(const AName, AResultName: String; AParent: TPasElement;
             UseParentAsResultParent: Boolean; const NamePos: TPasSourcePos; TypeParams: TFPList = nil): TPasFunctionType;
    function CreateInheritedExpr(AParent : TPasElement): TInheritedExpr;
    function CreateSelfExpr(AParent : TPasElement): TSelfExpr;
    function CreateNilExpr(AParent : TPasElement): TNilExpr;
    function CreateRecordValues(AParent : TPasElement): TRecordValues;
    Function IsCurTokenHint(out AHint : TPasMemberHint) : Boolean; overload;
    Function IsCurTokenHint: Boolean; overload;
    Function TokenIsCallingConvention(const S: String; out CC : TCallingConvention) : Boolean; virtual;
    Function TokenIsProcedureModifier(Parent: TPasElement; const S: String; Out PM : TProcedureModifier): Boolean; virtual;
    Function TokenIsAnonymousProcedureModifier(Parent: TPasElement; S: String; Out PM: TProcedureModifier): Boolean; virtual;
    Function TokenIsProcedureTypeModifier(Parent : TPasElement; const S : String; Out PTM : TProcTypeModifier) : Boolean; virtual;
    Function CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;
    function IsAnonymousProcAllowed(El: TPasElement): boolean; virtual;
    function ParseParams(AParent : TPasElement; ParamsKind: TPasExprKind; AllowFormatting : Boolean = False): TParamsExpr;
    function ParseExprOperand(AParent : TPasElement): TPasExpr;
    function ParseExpIdent(AParent : TPasElement): TPasExpr; deprecated 'use ParseExprOperand instead'; // since fpc 3.3.1
    procedure DoParseClassType(AType: TPasClassType);
    Function DoParseClassExternalHeader(AObjKind: TPasObjKind;
      out AExternalNameSpace, AExternalName: string) : Boolean;
    procedure DoParseArrayType(ArrType: TPasArrayType);
    function DoParseExpression(AParent: TPaselement;InitExpr: TPasExpr=nil; AllowEqual : Boolean = True): TPasExpr;
    function DoParseConstValueExpression(AParent: TPasElement): TPasExpr;
    function CheckPackMode: TPackMode;
    function AddUseUnit(ASection: TPasSection; const NamePos: TPasSourcePos;
      AUnitName : string; NameExpr: TPasExpr; InFileExpr: TPrimitiveExpr): TPasUsesUnit;
    procedure CheckImplicitUsedUnits(ASection: TPasSection);
    procedure FinishedModule; virtual;
    // Overload handling
    procedure AddProcOrFunction(Decs: TPasDeclarations; AProc: TPasProcedure);
    function  CheckIfOverloaded(AParent: TPasElement; const AName: String): TPasElement;
  public
    constructor Create(AScanner: TPascalScanner; AFileResolver: TBaseFileResolver;  AEngine: TPasTreeContainer);
    Destructor Destroy; override;
    procedure SetLastMsg(MsgType: TMessageType; MsgNumber: integer; Const Fmt : String; Args : Array of {$ifdef pas2js}jsvalue{$else}const{$endif});
    // General parsing routines
    function CurTokenName: String;
    function CurTokenText: String;
    Function CurComments : TStrings;
    function CurTokenPos: TPasSourcePos;
    function CurSourcePos: TPasSourcePos;
    function HasToken: boolean;
    Function SavedComments : String;
    procedure NextToken; // read next non whitespace, non space
    procedure ChangeToken(tk: TToken);
    procedure UngetToken;
    procedure CheckToken(tk: TToken);
    procedure CheckTokens(tk: TTokens);
    procedure ExpectToken(tk: TToken);
    procedure ExpectTokens(tk:  TTokens);
    function GetPrevToken: TToken;
    function ExpectIdentifier: String;
    Function CurTokenIsIdentifier(Const S : String) : Boolean;
    // Expression parsing
    function isEndOfExp(AllowEqual : Boolean = False; CheckHints : Boolean = True): Boolean;
    function ExprToText(Expr: TPasExpr): String;
    function ArrayExprToText(Expr: TPasExprArray): String;
    // Type declarations
    function ResolveTypeReference(Name: string; Parent: TPasElement; ParamCnt: integer = 0): TPasType;
    function ParseVarType(Parent : TPasElement = Nil): TPasType;
    function ParseTypeDecl(Parent: TPasElement): TPasType;
    function ParseGenericTypeDecl(Parent: TPasElement; AddToParent: boolean): TPasGenericType;
    function ParseType(Parent: TPasElement; const NamePos: TPasSourcePos; const TypeName: String = ''; Full: Boolean = false): TPasType;
    function ParseReferenceToProcedureType(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String): TPasProcedureType;
    function ParseProcedureType(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String; const PT: TProcType): TPasProcedureType;
    function ParseStringType(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String): TPasAliasType;
    function ParseSimpleType(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String; IsFull : Boolean = False): TPasType;
    function ParseAliasType(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String): TPasType;
    function ParseTypeReference(Parent: TPasElement; NeedExpr: boolean; out Expr: TPasExpr): TPasType;
    function ParseSpecializeType(Parent: TPasElement; Const NamePos: TPasSourcePos; const TypeName, GenName: string; var GenNameExpr: TPasExpr): TPasSpecializeType;
    function ParsePointerType(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String): TPasPointerType;
    Function ParseArrayType(Parent : TPasElement; Const NamePos: TPasSourcePos; Const TypeName : String; PackMode : TPackMode) : TPasArrayType;
    Function ParseFileType(Parent : TPasElement; Const NamePos: TPasSourcePos; Const TypeName  : String) : TPasFileType;
    Function ParseRecordDecl(Parent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName : string; const Packmode : TPackMode = pmNone) : TPasRecordType;
    function ParseEnumType(Parent: TPasElement; Const NamePos: TPasSourcePos; const TypeName: String): TPasEnumType;
    function ParseSetType(Parent: TPasElement; Const NamePos: TPasSourcePos; const TypeName: String; AIsPacked : Boolean = False): TPasSetType;
    Function ParseClassDecl(Parent: TPasElement; Const NamePos: TPasSourcePos; Const AClassName: String; AObjKind: TPasObjKind; PackMode : TPackMode= pmNone): TPasType;
    Function ParseProperty(Parent : TPasElement; Const AName : String; AVisibility : TPasMemberVisibility; IsClassField: boolean) : TPasProperty;
    function ParseRangeType(AParent: TPasElement; Const NamePos: TPasSourcePos; Const TypeName: String; Full: Boolean = True): TPasRangeType;
    procedure ParseExportDecl(Parent: TPasElement; List: TFPList);
    // Constant declarations
    function ParseConstDecl(Parent: TPasElement): TPasConst;
    function ParseResourcestringDecl(Parent: TPasElement): TPasResString;
    function ParseAttributes(Parent: TPasElement; Add: boolean): TPasAttributes;
    // Variable handling. This includes parts of records
    procedure ParseVarDecl(Parent: TPasElement; List: TFPList);
    procedure ParseInlineVarDecl(Parent: TPasElement; List: TFPList;  AVisibility : TPasMemberVisibility  = visDefault; ClosingBrace: Boolean = False);
    // Main scope parsing
    procedure ParseMain(var Module: TPasModule);
    procedure ParseUnit(var Module: TPasModule);
    function GetLastSection: TPasSection; virtual;
    function CanParseContinue(out Section: TPasSection): boolean; virtual;
    procedure ParseContinue; virtual;
    procedure ParseProgram(var Module: TPasModule; SkipHeader : Boolean = False);
    procedure ParseLibrary(var Module: TPasModule);
    procedure ParseOptionalUsesList(ASection: TPasSection);
    procedure ParseUsesList(ASection: TPasSection);
    procedure ParseInterface;
    procedure ParseImplementation;
    procedure ParseInitialization;
    procedure ParseFinalization;
    procedure ParseDeclarations(Declarations: TPasDeclarations);
    procedure ParseStatement(Parent: TPasImplBlock; out NewImplElement: TPasImplElement);
    procedure ParseAdhocExpression(out NewExprElement: TPasExpr);
    procedure ParseLabels(AParent: TPasElement);
    procedure ParseProcBeginBlock(Parent: TProcedureBody);
    procedure ParseProcAsmBlock(Parent: TProcedureBody);
    // Function/Procedure declaration
    function ParseProcedureOrFunctionDecl(Parent: TPasElement;
      ProcType: TProcType; MustBeGeneric: boolean;
      AVisibility: TPasMemberVisibility = VisDefault): TPasProcedure;
    procedure ParseArgList(Parent: TPasElement;
      Args: TFPList; // list of TPasArgument
      EndToken: TToken);
    procedure ParseProcedureOrFunction(Parent: TPasElement;
      Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);
    procedure ParseProcedureBody(Parent: TPasElement);
    function ParseMethodResolution(Parent: TPasElement): TPasMethodResolution;
    // Properties for external access
    property FileResolver: TBaseFileResolver read FFileResolver;
    property Scanner: TPascalScanner read FScanner;
    property Engine: TPasTreeContainer read FEngine;
    property CurToken: TToken read FCurToken;
    property CurTokenString: String read FCurTokenString;
    property Options : TPOptions Read FOptions Write SetOptions;
    property CurrentModeswitches : TModeSwitches Read GetCurrentModeSwitches Write SetCurrentModeSwitches;
    property CurModule : TPasModule Read FCurModule;
    property LogEvents : TPParserLogEvents Read FLogEvents Write FLogEvents;
    property OnLog : TPasParserLogHandler Read FOnLog Write FOnLog;
    property ImplicitUses: TStrings read FImplicitUses;
    property LastMsg: string read FLastMsg write FLastMsg;
    property LastMsgNumber: integer read FLastMsgNumber write FLastMsgNumber;
    property LastMsgType: TMessageType read FLastMsgType write FLastMsgType;
    property LastMsgPattern: string read FLastMsgPattern write FLastMsgPattern;
    property LastMsgArgs: TMessageArgs read FLastMsgArgs write FLastMsgArgs;
  end;

Type
  TParseSourceOption = (
    {$ifdef HasStreams}
    poUseStreams,
    {$endif}
    poSkipDefaultDefs);
  TParseSourceOptions = set of TParseSourceOption;

Var
  DefaultFileResolverClass : TBaseFileResolverClass = Nil;

{$ifdef HasStreams}
function ParseSource(AEngine: TPasTreeContainer;
                     const FPCCommandLine, OSTarget, CPUTarget: String;
                     UseStreams  : Boolean): TPasModule; deprecated 'use version with options';
{$endif}
function ParseSource(AEngine: TPasTreeContainer;
                     const FPCCommandLine, OSTarget, CPUTarget: String): TPasModule; deprecated 'use version with split command line';
function ParseSource(AEngine: TPasTreeContainer;
                     const FPCCommandLine, OSTarget, CPUTarget: String;
                     Options : TParseSourceOptions): TPasModule; deprecated 'use version with split command line';
function ParseSource(AEngine: TPasTreeContainer;
                     const FPCCommandLine : Array of String;
                     OSTarget, CPUTarget: String;
                     Options : TParseSourceOptions): TPasModule;

Function IsHintToken(T : String; Out AHint : TPasMemberHint) : boolean;
Function IsProcModifier(S : String; Out PM : TProcedureModifier) : Boolean;
Function IsCallingConvention(S : String; out CC : TCallingConvention) : Boolean;
Function TokenToAssignKind( tk : TToken) : TAssignKind;

implementation

{$IF FPC_FULLVERSION>=30301}
uses strutils;
{$ENDIF}

const
  WhitespaceTokensToIgnore = [tkWhitespace, tkComment, tkLineEnding, tkTab];

type
  TDeclType = (declNone, declConst, declResourcestring, declType,
               declVar, declThreadvar, declProperty, declExports);

{$IF FPC_FULLVERSION<30301}
Function SplitCommandLine(S: String) : TStringDynArray;

  Function GetNextWord : String;

  Const
    WhiteSpace = [' ',#9,#10,#13];
    Literals = ['"',''''];

  Var
    Wstart,wend : Integer;
    InLiteral : Boolean;
    LastLiteral : Char;

    Procedure AppendToResult;

    begin
      Result:=Result+Copy(S,WStart,WEnd-WStart);
      WStart:=Wend+1;
    end;

  begin
    Result:='';
    WStart:=1;
    While (WStart<=Length(S)) and charinset(S[WStart],WhiteSpace) do
      Inc(WStart);
    WEnd:=WStart;
    InLiteral:=False;
    LastLiteral:=#0;
    While (Wend<=Length(S)) and (Not charinset(S[Wend],WhiteSpace) or InLiteral) do
      begin
      if charinset(S[Wend],Literals) then
        If InLiteral then
          begin
          InLiteral:=Not (S[Wend]=LastLiteral);
          if not InLiteral then
            AppendToResult;
          end
        else
          begin
          InLiteral:=True;
          LastLiteral:=S[Wend];
          AppendToResult;
          end;
       inc(wend);
       end;
     AppendToResult;
     While (WEnd<=Length(S)) and (S[Wend] in WhiteSpace) do
       inc(Wend);
     Delete(S,1,WEnd-1);
  end;

Var
  W : String;
  len : Integer;

begin
  Len:=0;
  Result:=Default(TStringDynArray);
  SetLength(Result,(Length(S) div 2)+1);
  While Length(S)>0 do
    begin
    W:=GetNextWord;
    If (W<>'') then
      begin
      Result[Len]:=W;
      Inc(Len);
      end;
    end;
  SetLength(Result,Len);
end;
{$ENDIF}

Function IsHintToken(T : String; Out AHint : TPasMemberHint) : boolean;

Const
   MemberHintTokens : Array[TPasMemberHint] of string =
     ('deprecated','library','platform','experimental','unimplemented');
Var
  I : TPasMemberHint;

begin
  t:=LowerCase(t);
  Result:=False;
  For I:=Low(TPasMemberHint) to High(TPasMemberHint) do
    begin
    result:=(t=MemberHintTokens[i]);
    if Result then
      begin
      aHint:=I;
      exit;
      end;
    end;
end;


Function IsCallingConvention(S : String; out CC : TCallingConvention) : Boolean;

Var
  CCNames : Array[TCallingConvention] of String
         = ('','register','pascal','cdecl','stdcall','oldfpccall','safecall','syscall',
           'mwpascal', 'hardfloat','sysv_abi_default','sysv_abi_cdecl',
           'ms_abi_default','ms_abi_cdecl','vectorcall');
Var
  C : TCallingConvention;

begin
  S:=Lowercase(s);
  Result:=False;
  for C:=Low(TCallingConvention) to High(TCallingConvention) do
    begin
    Result:=(CCNames[c]<>'') and (s=CCnames[c]);
    If Result then
      begin
      CC:=C;
      exit;
      end;
    end;
end;

Function IsProcModifier(S : String; Out PM : TProcedureModifier) : Boolean;

Var
  P : TProcedureModifier;

begin
  S:=LowerCase(S);
  Result:=False;
  For P:=Low(TProcedureModifier) to High(TProcedureModifier) do
    begin
    Result:=s=ModifierNames[P];
    If Result then
      begin
      PM:=P;
      exit;
      end;
    end;
end;

Function TokenToAssignKind( tk : TToken) : TAssignKind;

begin
  case tk of
    tkAssign         : Result:=akDefault;
    tkAssignPlus     : Result:=akAdd;
    tkAssignMinus    : Result:=akMinus;
    tkAssignMul      : Result:=akMul;
    tkAssignDivision : Result:=akDivision;
  else
    Raise Exception.CreateFmt('Not an assignment token : %s',[TokenInfos[tk]]);
  end;
end;

function ParseSource(AEngine: TPasTreeContainer;
  const FPCCommandLine, OSTarget, CPUTarget: String): TPasModule;
var
  FPCParams: TStringDynArray;
begin
  FPCParams:=SplitCommandLine(FPCCommandLine);
  Result:=ParseSource(AEngine, FPCParams, OSTarget, CPUTarget,[]);
end;

{$ifdef HasStreams}
function ParseSource(AEngine: TPasTreeContainer;
  const FPCCommandLine, OSTarget, CPUTarget: String; UseStreams : Boolean): TPasModule;
var
  FPCParams: TStringDynArray;
begin
  FPCParams:=SplitCommandLine(FPCCommandLine);
  if UseStreams then
    Result:=ParseSource(AEngine,FPCParams, OSTarget, CPUTarget,[poUseStreams])
  else
    Result:=ParseSource(AEngine,FPCParams, OSTarget, CPUTarget,[]);
end;
{$endif}

function ParseSource(AEngine: TPasTreeContainer;
  const FPCCommandLine, OSTarget, CPUTarget: String;
  Options : TParseSourceOptions): TPasModule;

Var
  Args : TStringArray;

begin
  Args:=SplitCommandLine(FPCCommandLine);
  Result:=ParseSource(aEngine,Args,OSTarget,CPUTarget,Options);

end;

function ParseSource(AEngine: TPasTreeContainer;
                     const FPCCommandLine : Array of String;
                     OSTarget, CPUTarget: String;
                     Options : TParseSourceOptions): TPasModule;

var
  FileResolver: TBaseFileResolver;
  Parser: TPasParser;
  Filename: String;
  Scanner: TPascalScanner;

  procedure ProcessCmdLinePart(S : String);
  var
    l,Len: Integer;

  begin
    if (S='') then
      exit;
    Len:=Length(S);
    if (s[1] = '-') and (len>1) then
    begin
      case s[2] of
        'd': // -d define
          Scanner.AddDefine(UpperCase(Copy(s, 3, Len)));
        'u': // -u undefine
          Scanner.RemoveDefine(UpperCase(Copy(s, 3, Len)));
        'F': // -F
          if (len>2) and (s[3] = 'i') then // -Fi include path
            FileResolver.AddIncludePath(Copy(s, 4, Len));
        'I': // -I include path
          FileResolver.AddIncludePath(Copy(s, 3, Len));
        'S': // -S mode
          if  (len>2) then
            begin
            l:=3;
            While L<=Len do
              begin
              case S[l] of
                'c' : Scanner.Options:=Scanner.Options+[po_cassignments];
                'd' : Scanner.SetCompilerMode('DELPHI');
                '2' : Scanner.SetCompilerMode('OBJFPC');
                'h' : ; // do nothing
              end;
              inc(l);
              end;
            end;
        'M' :
           begin
           delete(S,1,2);
           Scanner.SetCompilerMode(S);
           end;
      end;
    end else
      if Filename <> '' then
        raise ENotSupportedException.Create(SErrMultipleSourceFiles)
      else
        Filename := s;
  end;

var
  S: String;

begin
  if DefaultFileResolverClass=Nil then
    raise ENotImplemented.Create(SErrFileSystemNotSupported);
  Result := nil;
  FileResolver := nil;
  Scanner := nil;
  Parser := nil;
  try
    FileResolver := DefaultFileResolverClass.Create;
    {$ifdef HasStreams}
    if FileResolver is TFileResolver then
      TFileResolver(FileResolver).UseStreams:=poUseStreams in Options;
    {$endif}
    Scanner := TPascalScanner.Create(FileResolver);
    Scanner.LogEvents:=AEngine.ScannerLogEvents;
    Scanner.OnLog:=AEngine.Onlog;
    if not (poSkipDefaultDefs in Options) then
      begin
      Scanner.AddDefine('FPK');
      Scanner.AddDefine('FPC');
      // TargetOS
      s := UpperCase(OSTarget);
      Scanner.AddDefine(s);
      Case s of
        'LINUX' : Scanner.AddDefine('UNIX');
        'FREEBSD' :
          begin
          Scanner.AddDefine('BSD');
          Scanner.AddDefine('UNIX');
          end;
        'NETBSD' :
          begin
          Scanner.AddDefine('BSD');
          Scanner.AddDefine('UNIX');
          end;
        'SUNOS' :
          begin
          Scanner.AddDefine('SOLARIS');
          Scanner.AddDefine('UNIX');
          end;
        'GO32V2' : Scanner.AddDefine('DPMI');
        'BEOS' : Scanner.AddDefine('UNIX');
        'QNX' : Scanner.AddDefine('UNIX');
        'AROS' : Scanner.AddDefine('HASAMIGA');
        'MORPHOS' : Scanner.AddDefine('HASAMIGA');
        'AMIGA' : Scanner.AddDefine('HASAMIGA');
      end;
      // TargetCPU
      s := UpperCase(CPUTarget);
      Scanner.AddDefine('CPU'+s);
      if (s='X86_64') then
        Scanner.AddDefine('CPU64')
      else
        Scanner.AddDefine('CPU32');
      end;
    Parser := TPasParser.Create(Scanner, FileResolver, AEngine);
    if (poSkipDefaultDefs in Options) then
      Parser.ImplicitUses.Clear;
    Filename := '';
    Parser.LogEvents:=AEngine.ParserLogEvents;
    Parser.OnLog:=AEngine.Onlog;

    For S in FPCCommandLine do
      ProcessCmdLinePart(S);
    if Filename = '' then
      raise Exception.Create(SErrNoSourceGiven);
{$IFDEF HASFS}
    FileResolver.AddIncludePath(ExtractFilePath(FileName));
{$ENDIF}
    Scanner.OpenFile(Filename);
    Parser.ParseMain(Result);
  finally
    Parser.Free;
    Scanner.Free;
    FileResolver.Free;
  end;
end;

{ ---------------------------------------------------------------------
  TPasTreeContainer
  ---------------------------------------------------------------------}

procedure TPasTreeContainer.SetCurrentParser(AValue: TPasParser);
begin
  if FCurrentParser=AValue then Exit;
  FCurrentParser:=AValue;
end;

function TPasTreeContainer.CreateElement(AClass: TPTreeElement;
  const AName: String; AParent: TPasElement; const ASourceFilename: String;
  ASourceLinenumber: Integer): TPasElement;
begin
  Result := CreateElement(AClass, AName, AParent, visDefault, ASourceFilename,
    ASourceLinenumber);
end;

function TPasTreeContainer.CreateElement(AClass: TPTreeElement;
  const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASrcPos: TPasSourcePos; TypeParams: TFPList): TPasElement;
begin
  Result := CreateElement(AClass, AName, AParent, AVisibility, ASrcPos.FileName,
    ASrcPos.Row);
  if TypeParams=nil then ;
end;

function TPasTreeContainer.CreateFunctionType(const AName, AResultName: String;
  AParent: TPasElement; UseParentAsResultParent: Boolean;
  const ASrcPos: TPasSourcePos; TypeParams: TFPList): TPasFunctionType;
var
  ResultParent: TPasElement;
begin
  Result := TPasFunctionType(CreateElement(TPasFunctionType, AName, AParent,
    visDefault, ASrcPos, TypeParams));

  if UseParentAsResultParent then
    ResultParent := AParent
  else
    ResultParent := Result;

  TPasFunctionType(Result).ResultEl :=
    TPasResultElement(CreateElement(TPasResultElement, AResultName, ResultParent,
    visDefault, ASrcPos, TypeParams));
end;

function TPasTreeContainer.FindElementFor(const AName: String;
  AParent: TPasElement; TypeParamCount: integer): TPasElement;
begin
  Result:=FindElement(AName);
  if AParent=nil then ;
  if TypeParamCount=0 then ;
end;

procedure TPasTreeContainer.BeginScope(ScopeType: TPasScopeType; El: TPasElement
  );
begin
  if ScopeType=stModule then ; // avoid compiler warning
  if El=nil then ;
end;

procedure TPasTreeContainer.FinishScope(ScopeType: TPasScopeType;
  El: TPasElement);
begin
  if ScopeType=stModule then ; // avoid compiler warning
  if Assigned(El) and (CurrentParser<>nil) then
    El.SourceEndLinenumber := CurrentParser.CurSourcePos.Row;
end;

procedure TPasTreeContainer.FinishTypeAlias(var aType: TPasType);
begin
  if aType=nil then ;
end;

function TPasTreeContainer.FindModule(const AName: String): TPasModule;
begin
  if AName='' then ;  // avoid compiler warning
  Result := nil;
end;

function TPasTreeContainer.FindModule(const AName: String; NameExpr,
  InFileExpr: TPasExpr): TPasModule;
begin
  Result:=FindModule(AName);
  if NameExpr=nil then ;
  if InFileExpr=nil then ;
end;

function TPasTreeContainer.CheckPendingUsedInterface(Section: TPasSection
  ): boolean;
begin
  if Section=nil then ;  // avoid compiler warning
  Result:=false;
end;

function TPasTreeContainer.NeedArrayValues(El: TPasElement): boolean;
begin
  Result:=false;
  if El=nil then ;  // avoid compiler warning
end;

function TPasTreeContainer.GetDefaultClassVisibility(AClass: TPasClassType
  ): TPasMemberVisibility;
begin
  Result:=visDefault; 
  if AClass=nil then ;  // avoid compiler warning
end;

procedure TPasTreeContainer.ModeChanged(Sender: TObject; NewMode: TModeSwitch;
  Before: boolean; var Handled: boolean);
begin
  if Sender=nil then ;
  if NewMode=msDelphi then ;
  if Before then ;
  if Handled then ;
end;

{ ---------------------------------------------------------------------
  EParserError
  ---------------------------------------------------------------------}

constructor EParserError.Create(const AReason, AFilename: String;
  ARow, AColumn: Integer);
begin
  inherited Create(AReason);
  FFilename := AFilename;
  FRow := ARow;
  FColumn := AColumn;
end;

{ ---------------------------------------------------------------------
  TPasParser
  ---------------------------------------------------------------------}

procedure TPasParser.ParseExc(MsgNumber: integer; const Msg: String);
begin
  ParseExc(MsgNumber,Msg,[]);
end;

procedure TPasParser.ParseExc(MsgNumber: integer; const Fmt: String;
  Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
var
  p: TPasSourcePos;
begin
  {$IFDEF VerbosePasParser}
  writeln('TPasParser.ParseExc Token="',CurTokenText,'"');
  //writeln('TPasParser.ParseExc ',Scanner.CurColumn,' ',Scanner.CurSourcePos.Column,' ',Scanner.CurTokenPos.Column,' ',Scanner.CurSourceFile.Filename);
  {$ENDIF}
  SetLastMsg(mtError,MsgNumber,Fmt,Args);
  p:=Scanner.CurTokenPos;
  if p.FileName='' then
    p:=Scanner.CurSourcePos;
  if p.Row=0 then
    begin
    p.Row:=1;
    p.Column:=1;
    end;
  raise EParserError.Create(SafeFormat(SParserErrorAtToken,
    [FLastMsg, CurTokenName, p.FileName, p.Row, p.Column])
    {$ifdef addlocation}+' ('+IntToStr(p.Row)+' '+IntToStr(p.Column)+')'{$endif},
    p.FileName, p.Row, p.Column);
end;

procedure TPasParser.ParseExcExpectedIdentifier;
begin
  ParseExc(nParserExpectedIdentifier,SParserExpectedIdentifier);
end;

procedure TPasParser.ParseExcSyntaxError;
begin
  ParseExc(nParserSyntaxError,SParserSyntaxError);
end;

procedure TPasParser.ParseExcTokenError(const Arg: string);
begin
  ParseExc(nParserExpectTokenError,SParserExpectTokenError,[Arg]);
end;

procedure TPasParser.ParseExcTypeParamsNotAllowed;
begin
  ParseExc(nParserTypeParamsNotAllowedOnType,sParserTypeParamsNotAllowedOnType,[]);
end;

procedure TPasParser.ParseExcExpectedAorB(const A, B: string);
begin
  ParseExc(nParserExpectToken2Error,SParserExpectToken2Error,[A,B]);
end;

constructor TPasParser.Create(AScanner: TPascalScanner;
  AFileResolver: TBaseFileResolver; AEngine: TPasTreeContainer);
begin
  inherited Create;
  FScanner := AScanner;
  if FScanner.OnModeChanged=nil then
    FScanner.OnModeChanged:=@OnScannerModeChanged;
  FFileResolver := AFileResolver;
  FTokenRingCur:=High(FTokenRing);
  FEngine := AEngine;
  if Assigned(FEngine) then
    begin
    FEngine.CurrentParser:=Self;
    If FEngine.NeedComments then
      FScanner.SkipComments:=Not FEngine.NeedComments;
    end;
  FImplicitUses := TStringList.Create;
  FImplicitUses.Add('System'); // system always implicitely first.
end;

destructor TPasParser.Destroy;
var
  i: Integer;
begin
  if FScanner.OnModeChanged=@OnScannerModeChanged then
    FScanner.OnModeChanged:=nil;
  if Assigned(FEngine) then
    begin
    FEngine.CurrentParser:=Nil;
    FEngine:=nil;
    end;
  FreeAndNil(FImplicitUses);
  for i:=low(FTokenRing) to high(FTokenRing) do
    FreeAndNil(FTokenRing[i].Comments);
  inherited Destroy;
end;

function TPasParser.CurTokenName: String;
begin
  if CurToken = tkIdentifier then
    Result := 'Identifier ' + FCurTokenString
  else
    Result := TokenInfos[CurToken];
end;

function TPasParser.CurTokenText: String;
begin
  case CurToken of
    tkIdentifier, tkString, tkNumber, tkChar:
      Result := FCurTokenString;
    else
      Result := TokenInfos[CurToken];
  end;
end;

function TPasParser.CurComments: TStrings;
begin
  if FTokenRingStart=FTokenRingEnd then
    Result:=nil
  else
    Result:=FTokenRing[FTokenRingCur].Comments;
end;

function TPasParser.CurTokenPos: TPasSourcePos;
begin
  if HasToken then
    Result:=FTokenRing[FTokenRingCur].TokenPos
  else if Scanner<>nil then
    Result:=Scanner.CurTokenPos
  else
    Result:=Default(TPasSourcePos);
end;

function TPasParser.CurSourcePos: TPasSourcePos;
begin
  if HasToken then
    Result:=FTokenRing[FTokenRingCur].SourcePos
  else if Scanner<>nil then
    Result:=Scanner.CurSourcePos
  else
    Result:=Default(TPasSourcePos);
end;

function TPasParser.HasToken: boolean;
begin
  if FTokenRingStart<FTokenRingEnd then
    Result:=(FTokenRingCur>=FTokenRingStart) and (FTokenRingCur<FTokenRingEnd)
  else
    Result:=(FTokenRingCur>=FTokenRingStart) or (FTokenRingCur<FTokenRingEnd);
end;

function TPasParser.SavedComments: String;
begin
  Result:=FSavedComments;
end;

procedure TPasParser.NextToken;

Var
  P: PTokenRec;
begin
  FTokenRingCur:=(FTokenRingCur+1) mod FTokenRingSize;
  P:=@FTokenRing[FTokenRingCur];
  if FTokenRingCur <> FTokenRingEnd then
    begin
    // Get token from buffer
    //writeln('TPasParser.NextToken REUSE Start=',FTokenRingStart,' Cur=',FTokenRingCur,' End=',FTokenRingEnd,' Cur=',CurTokenString);
    FCurToken := Scanner.CheckToken(P^.Token,P^.AsString);
    FCurTokenString := P^.AsString;
    end
  else
    begin
    // Fetch new token
    //writeln('TPasParser.NextToken FETCH Start=',FTokenRingStart,' Cur=',FTokenRingCur,' End=',FTokenRingEnd,' Cur=',CurTokenString);
    FTokenRingEnd:=(FTokenRingEnd+1) mod FTokenRingSize;
    if FTokenRingStart=FTokenRingEnd then
      FTokenRingStart:=(FTokenRingStart+1) mod FTokenRingSize;
    try
      if p^.Comments=nil then
        p^.Comments:=TStringList.Create
      else
        p^.Comments.Clear;
      repeat
        FCurToken := Scanner.FetchToken;
        if FCurToken=tkComment then
          p^.Comments.Add(Scanner.CurTokenString);
      until not (FCurToken in WhitespaceTokensToIgnore);
    except
      on e: EScannerError do
        begin
        if po_KeepScannerError in Options then
          raise
        else
          begin
          FLastMsgType := mtError;
          FLastMsgNumber := Scanner.LastMsgNumber;
          FLastMsgPattern := Scanner.LastMsgPattern;
          FLastMsg := Scanner.LastMsg;
          FLastMsgArgs := Scanner.LastMsgArgs;
          raise EParserError.Create(e.Message,
            Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
          end;
        end;
    end;
    p^.Token:=FCurToken;
    FCurTokenString := Scanner.CurTokenString;
    p^.AsString:=FCurTokenString;
    p^.SourcePos:=Scanner.CurSourcePos;
    p^.TokenPos:=Scanner.CurTokenPos;
    end;
  //writeln('TPasParser.NextToken END Start=',FTokenRingStart,' Cur=',FTokenRingCur,' End=',FTokenRingEnd,' Cur=',CurTokenString);
end;

procedure TPasParser.ChangeToken(tk: TToken);
var
  Cur, Last: PTokenRec;
  IsLast: Boolean;

  Procedure DoChange(tk1,tk2 : TToken);

    begin
      // change last token '>>' into two '>'
      Cur:=@FTokenRing[FTokenRingCur];
      Cur^.Token:=tk2;
      Cur^.AsString:=TokenInfos[tk2];
      Last:=@FTokenRing[FTokenRingEnd];
      Last^.Token:=tk2;
      Last^.AsString:=TokenInfos[tk2];
      if Last^.Comments<>nil then
        Last^.Comments.Clear;
      Last^.SourcePos:=Cur^.SourcePos;
      dec(Cur^.SourcePos.Column);
      Last^.TokenPos:=Cur^.TokenPos;
      inc(Last^.TokenPos.Column);
      FTokenRingEnd:=(FTokenRingEnd+1) mod FTokenRingSize;
      if FTokenRingStart=FTokenRingEnd then
        FTokenRingStart:=(FTokenRingStart+1) mod FTokenRingSize;
      FCurToken:=tk1;
      FCurTokenString:=TokenInfos[tk1];
    end;

begin
  //writeln('TPasParser.ChangeToken FTokenBufferSize=',FTokenRingStart,' FTokenBufferIndex=',FTokenRingCur);
  IsLast:=((FTokenRingCur+1) mod FTokenRingSize)=FTokenRingEnd;
  if (CurToken=tkGreaterEqualThan) and (tk=tkGreaterThan) and IsLast then
    begin
    DoChange(tkGreaterThan,tkEqual);
    end
  else if (CurToken=tkshr) and (tk=tkGreaterThan) and IsLast then
    begin
    DoChange(tkGreaterThan,tkGreaterThan);
    end
  else
    CheckToken(tk);
end;

procedure TPasParser.UngetToken;

var
  P: PTokenRec;
begin
  //writeln('TPasParser.UngetToken START Start=',FTokenRingStart,' Cur=',FTokenRingCur,' End=',FTokenRingEnd,' Cur=',CurTokenString);
  if FTokenRingStart = FTokenRingEnd then
    ParseExc(nParserUngetTokenError,SParserUngetTokenError);
  if FTokenRingCur>0 then
    dec(FTokenRingCur)
  else
    FTokenRingCur:=High(FTokenRing);
  P:=@FTokenRing[FTokenRingCur];
  FCurToken := P^.Token;
  FCurTokenString := P^.AsString;
  //writeln('TPasParser.UngetToken END Start=',FTokenRingStart,' Cur=',FTokenRingCur,' End=',FTokenRingEnd,' Cur=',CurTokenString);
end;

procedure TPasParser.CheckToken(tk: TToken);
begin
  if (CurToken<>tk) then
    begin
    {$IFDEF VerbosePasParser}
    writeln('TPasParser.ParseExcTokenError String="',CurTokenString,'" Text="',CurTokenText,'" CurToken=',CurToken,' tk=',tk);
    {$ENDIF}
    ParseExcTokenError(TokenInfos[tk]);
    end;
end;

procedure TPasParser.CheckTokens(tk: TTokens);

Var
  S : String;
  T : TToken;
begin
  if not (CurToken in tk) then
    begin
    {$IFDEF VerbosePasParser}
    writeln('TPasParser.ParseExcTokenError String="',CurTokenString,'" Text="',CurTokenText,'" CurToken=',CurToken);
    {$ENDIF}
    S:='';
    For T in TToken do
      if t in tk then
        begin
        if (S<>'') then
          S:=S+' or ';
        S:=S+TokenInfos[t];
        end;
    ParseExcTokenError(S);
    end;
end;


procedure TPasParser.ExpectToken(tk: TToken);
begin
  NextToken;
  CheckToken(tk);
end;

procedure TPasParser.ExpectTokens(tk: TTokens);
begin
  NextToken;
  CheckTokens(tk);
end;

function TPasParser.GetPrevToken: TToken;
var
  i: Integer;
  P: PTokenRec;
begin
  if FTokenRingStart = FTokenRingEnd then
    Result:=tkEOF;
  i:=FTokenRingCur;
  if i>0 then
    dec(i)
  else
    i:=High(FTokenRing);
  P:=@FTokenRing[i];
  Result := P^.Token;
end;

function TPasParser.ExpectIdentifier: String;
begin
  ExpectToken(tkIdentifier);
  Result := CurTokenString;
end;

function TPasParser.CurTokenIsIdentifier(const S: String): Boolean;
begin
  Result:=(Curtoken=tkIdentifier) and (CompareText(S,CurtokenText)=0);
end;

function TPasParser.IsCurTokenHint(out AHint: TPasMemberHint): Boolean;
begin
  Result:=CurToken=tklibrary;
  if Result then
    AHint:=hLibrary
  else if (CurToken=tkIdentifier) then
    Result:=IsHintToken(CurTokenString,ahint);
end;

function TPasParser.IsCurTokenHint: Boolean;
var
  dummy : TPasMemberHint;
begin
  Result:=IsCurTokenHint(dummy);
end;

function TPasParser.TokenIsCallingConvention(const S: String; out
  CC: TCallingConvention): Boolean;
begin
  Result:=IsCallingConvention(S,CC);
end;

function TPasParser.TokenIsProcedureModifier(Parent: TPasElement;
  const S: String; out PM: TProcedureModifier): Boolean;
begin
  Result:=IsProcModifier(S,PM);
  if not Result then exit;
  While (Parent<>Nil) do
    begin
    if Parent is TPasClassType then
      begin
      if PM in [pmPublic,pmForward] then exit(false);
      if TPasClassType(Parent).ObjKind in [okInterface,okDispInterface] then
        if not (PM in [pmOverload, pmMessage, pmDispId,pmNoReturn,pmFar,pmFinal]) then
          exit(false);
      exit;
      end
    else if Parent is TPasRecordType then
      begin
      if not (PM in [pmOverload,
                     pmInline, pmAssembler,
                     pmExternal,
                     pmNoReturn, pmFar, pmFinal]) then exit(false);
      exit;
      end;
    Parent:=Parent.Parent;
    end;
end;

function TPasParser.TokenIsAnonymousProcedureModifier(Parent: TPasElement;
  S: String; out PM: TProcedureModifier): Boolean;
begin
  Result:=IsProcModifier(S,PM);
  if not Result then exit;
  case PM of
  pmAssembler: Result:=true;
  else
    Result:=false;
  end;
  if Parent=nil then ;
end;

function TPasParser.TokenIsProcedureTypeModifier(Parent: TPasElement;
  const S: String; out PTM: TProcTypeModifier): Boolean;
begin
  if CompareText(S,ProcTypeModifiers[ptmVarargs])=0 then
    begin
    Result:=true;
    PTM:=ptmVarargs;
    end
  else if CompareText(S,ProcTypeModifiers[ptmFar])=0 then
    begin
    Result:=true;
    PTM:=ptmFar;
    end
  else if CompareText(S,ProcTypeModifiers[ptmStatic])=0 then
    begin
    Result:=true;
    PTM:=ptmStatic;
    end
  else if CompareText(S,ProcTypeModifiers[ptmCblock])=0 then
    begin
    Result:=true;
    PTM:=ptmCblock;
    end
  else if (CompareText(S,ProcTypeModifiers[ptmAsync])=0) and (po_AsyncProcs in Options) then
    begin
    Result:=true;
    PTM:=ptmAsync;
    end
  else
   Result:=false;
  if Parent=nil then;
end;

function TPasParser.CheckHint(Element: TPasElement; ExpectSemiColon: Boolean
  ): TPasMemberHints;

Var
  Found : Boolean;
  h : TPasMemberHint;

begin
  Result:=[];
  Repeat
    NextToken;
    Found:=IsCurTokenHint(h);
    If Found then
      begin
      Include(Result,h);
      if (h=hDeprecated) then
        begin
        NextToken;
        if (Curtoken<>tkString) then
          UnGetToken
        else if assigned(Element) then
          Element.HintMessage:=CurTokenString;
        end;
      end;
  Until Not Found;
  UngetToken;
  If Assigned(Element) then
    Element.Hints:=Result;
  if ExpectSemiColon then
    ExpectToken(tkSemiColon);
end;

function TPasParser.IsAnonymousProcAllowed(El: TPasElement): boolean;
begin
  while El is TPasExpr do
    El:=El.Parent;
  Result:=El is TPasImplBlock; // only in statements
end;

function TPasParser.CheckPackMode: TPackMode;

begin
  NextToken;
  Case CurToken of
    tkPacked    : Result:=pmPacked;
    tkbitpacked : Result:=pmBitPacked;
  else
    result:=pmNone;
  end;
  if (Result<>pmNone) then
     begin
     NextToken;
     if Not (CurToken in [tkArray, tkRecord, tkObject, tkClass, tkObjCClass, tkSet]) then
       ParseExcTokenError('SET, ARRAY, RECORD, OBJECT or CLASS');
     end;
end;

Function IsSimpleTypeToken(Var AName : String) : Boolean;

Const
   SimpleTypeCount = 15;
   SimpleTypeNames : Array[1..SimpleTypeCount] of string =
     ('byte','boolean','char','integer','int64','longint','longword','double',
      'shortint','smallint','string','word','qword','cardinal','widechar');
   SimpleTypeCaseNames : Array[1..SimpleTypeCount] of string =
     ('Byte','Boolean','Char','Integer','Int64','LongInt','LongWord','Double',
     'ShortInt','SmallInt','String','Word','QWord','Cardinal','WideChar');

Var
  S : String;
  I : Integer;

begin
  S:=LowerCase(AName);
  I:=SimpleTypeCount;
  While (I>0) and (s<>SimpleTypeNames[i]) do
    Dec(I);
  Result:=(I>0);
  if Result Then
    AName:=SimpleTypeCaseNames[I];
end;

function TPasParser.ParseStringType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String): TPasAliasType;

Var
  LengthAsText : String;
  ok: Boolean;
  Params: TParamsExpr;
  LengthExpr: TPasExpr;

begin
  Result := TPasAliasType(CreateElement(TPasAliasType, TypeName, Parent, NamePos));
  ok:=false;
  try
    If (Result.Name='') then
      Result.Name:='string';
    Result.Expr:=CreatePrimitiveExpr(Result,pekIdent,TypeName);
    NextToken;
    LengthAsText:='';
    if CurToken=tkSquaredBraceOpen then
      begin
      Params:=TParamsExpr(CreateElement(TParamsExpr,'',Result));
      Params.Value:=Result.Expr;
      Params.Value.Parent:=Params;
      Result.Expr:=Params;
      LengthAsText:='';
      NextToken;
      LengthExpr:=DoParseExpression(Params,nil,false);
      Params.AddParam(LengthExpr);
      CheckToken(tkSquaredBraceClose);
      LengthAsText:=ExprToText(LengthExpr);
      end
    else
      UngetToken;
    Result.DestType:=TPasStringType(CreateElement(TPasStringType,'string',Result));
    TPasStringType(Result.DestType).LengthExpr:=LengthAsText;
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseSimpleType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String; IsFull: Boolean
  ): TPasType;

Type
  TSimpleTypeKind = (stkAlias,stkString,stkRange);

Var
  Ref: TPasType;
  K : TSimpleTypeKind;
  Name : String;
  Expr: TPasExpr;
  ok, MustBeSpecialize: Boolean;

begin
  Result:=nil;
  if CurToken=tkspecialize then
    begin
    MustBeSpecialize:=true;
    ExpectIdentifier;
    end
  else
    MustBeSpecialize:=false;
  Name := CurTokenString;
  Expr:=nil;
  Ref:=nil;
  ok:=false;
  try
    if IsFull then
      Name:=ReadDottedIdentifier(Parent,Expr,true)
    else
      begin
      NextToken;
      while CurToken=tkDot do
        begin
        ExpectIdentifier;
        Name := Name+'.'+CurTokenString;
        NextToken;
        end;
      end;

    if MustBeSpecialize and (CurToken<>tkLessThan) then
      ParseExcTokenError('<');

    // Current token is first token after identifier.
    if IsFull and (CurToken=tkSemicolon) or isCurTokenHint then // Type A = B;
      begin
      K:=stkAlias;
      UnGetToken;
      end
    else if IsFull and (CurToken=tkSquaredBraceOpen) then
      begin
      if LowerCase(Name)='string' then // Type A = String[12]; shortstring
        K:=stkString
      else
        ParseExcSyntaxError;
      UnGetToken;
      end
    else if (CurToken = tkLessThan)
        and (MustBeSpecialize or (msDelphi in CurrentModeswitches)) then // A = B<t>;
      begin
      Result:=ParseSpecializeType(Parent,NamePos,TypeName,Name,Expr);
      ok:=true;
      exit;
      end
    else if (CurToken in [tkBraceOpen,tkDotDot]) then // A: B..C;
      begin
      K:=stkRange;
      UnGetToken;
      end
    else
      begin
      if IsFull then
        ParseExcTokenError(';');
      K:=stkAlias;
      if (not (po_resolvestandardtypes in Options)) and (LowerCase(Name)='string') then
        K:=stkString;
      UnGetToken;
      end;

    Case K of
      stkString:
        begin
        ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
        Result:=ParseStringType(Parent,NamePos,TypeName);
        end;
      stkRange:
        begin
        ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
        UnGetToken; // move to '='
        Result:=ParseRangeType(Parent,NamePos,TypeName,False);
        end;
      stkAlias:
        begin
        Ref:=ResolveTypeReference(Name,Parent);
        if IsFull then
          begin
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName, Parent, NamePos));
          TPasAliasType(Result).DestType:=Ref;
          Ref:=nil;
          TPasAliasType(Result).Expr:=Expr;
          Expr.Parent:=Result;
          Expr:=nil;
          if TypeName<>'' then
            Engine.FinishScope(stTypeDef,Result);
          end
        else
          Result:=Ref;
        end;
    end;
    ok:=true;
  finally
    if not ok then
      begin
      if Result<>nil then
        Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if Expr<>nil then
        Expr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if Ref<>nil then
        Ref.Release{$IFDEF CheckPasTreeRefCount}('ResolveTypeReference'){$ENDIF};
      end
  end;
end;

// On entry, we're on the TYPE token
function TPasParser.ParseAliasType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String): TPasType;
var
  ok: Boolean;
begin
  Result := TPasTypeAliasType(CreateElement(TPasTypeAliasType, TypeName, Parent, NamePos));
  ok:=false;
  try
    TPasTypeAliasType(Result).DestType := ParseType(Result,NamePos,'');
    Engine.FinishTypeAlias(Result);
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseTypeReference(Parent: TPasElement; NeedExpr: boolean;
  out Expr: TPasExpr): TPasType;
// returns either
// a) TPasSpecializeType, Expr=nil
// b) TPasUnresolvedTypeRef, Expr<>nil
// c) TPasType, Expr<>nil
// After parsing CurToken is behind last reference token, e.g. ;
var
  Name: String;
  IsSpecialize, ok: Boolean;
  NamePos: TPasSourcePos;
begin
  Result:=nil;
  Expr:=nil;
  ok:=false;
  try
    NamePos:=CurSourcePos;
    if CurToken=tkspecialize then
      begin
      IsSpecialize:=true;
      NextToken;
      end
    else
      IsSpecialize:=false;
    // read dotted identifier
    CheckToken(tkIdentifier);
    Name:=ReadDottedIdentifier(Parent,Expr,true);

    if CurToken=tkLessThan then
      begin
      // specialize
      if IsSpecialize or (msDelphi in CurrentModeswitches) then
        begin
        Result:=ParseSpecializeType(Parent,NamePos,'',Name,Expr);
        NextToken;
        end
      else
        CheckToken(tkend);
      end
    else if IsSpecialize then
      CheckToken(tkLessThan)
    else
      begin
      // simple type reference
      Result:=ResolveTypeReference(Name,Parent);
      end;
    ok:=true;
  finally
    if not ok then
      begin
      if Result<>nil then
        Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
      end
    else if (not NeedExpr) and (Expr<>nil) then
      ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
  end;
end;

function TPasParser.ParseSpecializeType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName, GenName: string;
  var GenNameExpr: TPasExpr): TPasSpecializeType;
// after parsing CurToken is at >
var
  ST: TPasSpecializeType;
begin
  Result:=nil;
  if CurToken<>tkLessThan then
    ParseExcTokenError('[20190801112729]');
  ST:=TPasSpecializeType(CreateElement(TPasSpecializeType,TypeName,Parent,NamePos));
  try
    if GenNameExpr<>nil then
      begin
      ST.Expr:=GenNameExpr;
      GenNameExpr.Parent:=ST;
      GenNameExpr:=nil; // ownership transferred to ST
      end;
    // read nested specialize arguments
    ReadSpecializeArguments(ST,ST.Params);
    if CurToken<>tkGreaterThan then
      ParseExcTokenError('[20190801113005]');
    // Important: resolve type reference AFTER args, because arg count is needed
    ST.DestType:=ResolveTypeReference(GenName,ST,ST.Params.Count);

    // Check for cascaded specialize A<B>.C or A<B>.C<D>
    NextToken;
    if CurToken<>tkDot then
      UnGetToken
    else
      begin
      NextToken;
      Engine.BeginScope(stSpecializeType,ST);
      ST.SubType:=ParseSimpleType(ST,CurSourcePos,GenName,False);
      Engine.FinishScope(stSpecializeType,ST);
      end;

    Engine.FinishScope(stTypeDef,ST);
    Result:=ST;
  finally
    if Result=nil then
      ST.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParsePointerType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String): TPasPointerType;

var
  ok: Boolean;
  Name: String;
begin
  Result := TPasPointerType(CreateElement(TPasPointerType, TypeName, Parent, NamePos));
  ok:=false;
  Try
    // only allowed: ^dottedidentifer
    // forbidden: ^^identifier, ^array of word, ^A<B>
    ExpectTokens([tkIdentifier,tkFile]);
    Name:=CurTokenString;
    repeat
      NextToken;
      if CurToken=tkDot then
        begin
        ExpectIdentifier;
        Name := Name+'.'+CurTokenString;
        end
      else
        break;
    until false;
    if CurToken=tkLessThan then
      begin
      Repeat
        NextToken; // We should do something with this.
      Until CurToken=tkGreaterThan;
      end
    else
      UngetToken;
    Result.DestType:=ResolveTypeReference(Name,Result);
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseEnumType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String): TPasEnumType;

Var
  EnumValue: TPasEnumValue;
  ok: Boolean;

begin
  Result := TPasEnumType(CreateElement(TPasEnumType, TypeName, Parent, NamePos));
  ok:=false;
  try
    while True do
      begin
      NextToken;
      SaveComments;
      EnumValue := TPasEnumValue(CreateElement(TPasEnumValue, CurTokenString, Result));
      Result.Values.Add(EnumValue);
      NextToken;
      if CurToken = tkBraceClose then
        break
      else if CurToken in [tkEqual,tkAssign] then
        begin
        NextToken;
        EnumValue.Value:=DoParseExpression(Result);
       // UngetToken;
        if CurToken = tkBraceClose then
          Break
        else if not (CurToken=tkComma) then
          ParseExc(nParserExpectedCommaRBracket,SParserExpectedCommaRBracket);
        end
      else if not (CurToken=tkComma) then
        ParseExc(nParserExpectedCommaRBracket,SParserExpectedCommaRBracket)
      end;
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseSetType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String; AIsPacked : Boolean = False): TPasSetType;

var
  ok: Boolean;
begin
  Result := TPasSetType(CreateElement(TPasSetType, TypeName, Parent, NamePos));
  Result.IsPacked:=AIsPacked;
  ok:=false;
  try
    ExpectToken(tkOf);
    Result.EnumType := ParseType(Result,CurSourcePos);
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String; Full: Boolean
  ): TPasType;

Type
  TLocalClassType = (lctClass,lctObjcClass,lctObjcCategory,lctHelper);

Const
  // These types are allowed only when full type declarations
  FullTypeTokens = [tkGeneric,{tkSpecialize,}tkClass,tkObjCClass,tkInterface,tkObjcProtocol,tkDispInterface,tkType];
  // Parsing of these types already takes care of hints
  NoHintTokens = [tkProcedure,tkFunction];
  InterfaceKindTypes : Array[Boolean] of TPasObjKind = (okInterface,okObjcProtocol);
  ClassKindTypes : Array[TLocalClassType] of TPasObjKind = (okClass,okObjCClass,okObjcCategory,okClassHelper);


var
  PM: TPackMode;
  CH, ok, isHelper : Boolean;
  lClassType : TLocalClassType;

begin
  Result := nil;
  // NextToken and check pack mode
  Pm:=CheckPackMode;
  if Full then
    CH:=Not (CurToken in NoHintTokens)
  else
    begin
    CH:=False;
    if (CurToken in FullTypeTokens) then
      ParseExc(nParserTypeNotAllowedHere,SParserTypeNotAllowedHere,[CurtokenText]);
    end;
  ok:=false;
  Try
    case CurToken of
      // types only allowed when full
      tkObject: Result := ParseClassDecl(Parent, NamePos, TypeName, okObject,PM);
      tkDispInterface:
        Result := ParseClassDecl(Parent, NamePos, TypeName, okDispInterface,PM);
      tkObjcProtocol,
      tkInterface:
        begin
        Result := ParseClassDecl(Parent, NamePos, TypeName, InterfaceKindTypes[(CurToken=tkObjcProtocol)],PM);
        end;
      tkSpecialize:
        Result:=ParseSimpleType(Parent,CurSourcePos,TypeName);
      tkObjCClass,
      tkobjccategory,
      tkClass:
        begin
        If (CurToken=tkObjCClass) then
          lClassType:=lctObjcClass
        else if (CurToken=tkobjccategory) then
          lClassType:=lctObjcCategory
        else
          begin
          lClassType:=lctClass;
          NextToken;
          if CurTokenIsIdentifier('Helper') then
            begin
            // class helper: atype end;
            // class helper for atype end;
            NextToken;
            if CurToken in [tkfor,tkBraceOpen] then
              lClassType:=lctHelper;
            UnGetToken;
            end;
          UngetToken;
          end;
        Result:=ParseClassDecl(Parent,NamePos,TypeName,ClassKindTypes[lClasstype], PM);
        end;
      tkType:
        begin
        isHelper:=false;
        if msTypeHelpers in Scanner.CurrentModeSwitches then
          begin
          NextToken;
          if CurTokenIsIdentifier('helper') then
            begin
            // atype = type helper;
            // atype = type helper for atype end;
            NextToken;
            isHelper:=CurToken in [tkfor,tkBraceOpen];
            UnGetToken;
            end;
          UnGetToken;
          end;
        if isHelper then
          Result:=ParseClassDecl(Parent,NamePos,TypeName,okTypeHelper,PM)
        else
          Result:=ParseAliasType(Parent,NamePos,TypeName);
        end;
      // Always allowed
      tkIdentifier:
        begin
        // Bug 31709: PReference = ^Reference;
        // Checked in Delphi: ^Reference to procedure; is not allowed !!
        if CurTokenIsIdentifier('reference') and Not (Parent is TPasPointerType) then
          begin
          CH:=False;
          Result:=ParseReferencetoProcedureType(Parent,NamePos,TypeName)
          end
        else
          Result:=ParseSimpleType(Parent,NamePos,TypeName,Full);
        end;
      tkCaret: Result:=ParsePointerType(Parent,NamePos,TypeName);
      tkFile: Result:=ParseFileType(Parent,NamePos,TypeName);
      tkArray: Result:=ParseArrayType(Parent,NamePos,TypeName,pm);
      tkBraceOpen: Result:=ParseEnumType(Parent,NamePos,TypeName);
      tkSet: Result:=ParseSetType(Parent,NamePos,TypeName,pm=pmPacked);
      tkProcedure: Result:=ParseProcedureType(Parent,NamePos,TypeName,ptProcedure);
      tkFunction: Result:=ParseProcedureType(Parent,NamePos,TypeName,ptFunction);
      tkRecord:
        begin
        NextToken;
        isHelper:=false;
        if CurTokenIsIdentifier('Helper') then
          begin
          // record helper: atype end;
          // record helper for atype end;
          NextToken;
          isHelper:=CurToken in [tkfor,tkBraceOpen];
          UnGetToken;
          end;
        UngetToken;
        if isHelper then
          Result:=ParseClassDecl(Parent,NamePos,TypeName,okRecordHelper,PM)
        else
          Result:=ParseRecordDecl(Parent,NamePos,TypeName,PM);
        end;
      tkNumber,tkMinus,tkChar:
        begin
        UngetToken;
        Result:=ParseRangeType(Parent,NamePos,TypeName,Full);
        end;
    else
      ParseExcExpectedIdentifier;
    end;
    if CH then
      CheckHint(Result,True);
    ok:=true;
  finally
    if not ok then
      if Result<>nil then
        begin
        Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
        Result:=nil;
        end;
  end;
end;

function TPasParser.ParseReferenceToProcedureType(Parent: TPasElement; const NamePos: TPasSourcePos; const TypeName: String
  ): TPasProcedureType;
begin
  if not CurTokenIsIdentifier('reference') then
    ParseExcTokenError('reference');
  ExpectToken(tkTo);
  NextToken;
  Case CurToken of
   tkprocedure : Result:=ParseProcedureType(Parent,NamePos,TypeName,ptProcedure);
   tkfunction : Result:=ParseProcedureType(Parent,NamePos,TypeName,ptFunction);
  else
    result:=Nil; // Fool compiler
    ParseExcTokenError('procedure or function');
  end;
  Result.IsReferenceTo:=True;
end;

function TPasParser.ParseVarType(Parent : TPasElement = Nil): TPasType;
var
  NamePos: TPasSourcePos;
begin
  NextToken;
  case CurToken of
    tkProcedure:
      begin
        Result := TPasProcedureType(CreateElement(TPasProcedureType, '', Parent));
        ParseProcedureOrFunction(Result, TPasProcedureType(Result), ptProcedure, True);
        if CurToken = tkSemicolon then
          UngetToken;        // Unget semicolon
      end;
    tkFunction:
      begin
        Result := CreateFunctionType('', 'Result', Parent, False, CurSourcePos);
        ParseProcedureOrFunction(Result, TPasFunctionType(Result), ptFunction, True);
        if CurToken = tkSemicolon then
          UngetToken;        // Unget semicolon
      end;
  else
    NamePos:=CurSourcePos;
    UngetToken;
    Result := ParseType(Parent,NamePos);
  end;
end;

function TPasParser.ParseArrayType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String; PackMode: TPackMode
  ): TPasArrayType;
Var
  ok: Boolean;
begin
  Result := TPasArrayType(CreateElement(TPasArrayType, TypeName, Parent, NamePos));
  ok:=false;
  try
    Result.PackMode:=PackMode;
    DoParseArrayType(Result);
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      begin
      Result.Parent:=nil;
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
  end;
end;

function TPasParser.ParseFileType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String): TPasFileType;
begin
  Result:=TPasFileType(CreateElement(TPasFileType, TypeName, Parent, NamePos));
  NextToken;
  If CurToken=tkOf then
    Result.ElType := ParseType(Result,CurSourcePos)
  else
   UngetToken;
end;

function TPasParser.isEndOfExp(AllowEqual : Boolean = False; CheckHints : Boolean = True):Boolean;
const
  EndExprToken = [
    tkEOF, tkBraceClose, tkSquaredBraceClose, tkSemicolon, tkComma, tkColon,
    tkdo, tkdownto, tkelse, tkend, tkof, tkthen, tkto, tkotherwise
  ];
begin
  if (CurToken in EndExprToken) or (CheckHints and IsCurTokenHint) then
    exit(true);
  if AllowEqual and (CurToken=tkEqual) then
    exit(true);
  Result:=false;
end;

function TPasParser.ExprToText(Expr: TPasExpr): String;
var
  C: TClass;
begin
  Result:='';
  C:=Expr.ClassType;
  if C=TPrimitiveExpr then
    Result:=TPrimitiveExpr(Expr).Value
  else if C=TSelfExpr then
    Result:='self'
  else if C=TBoolConstExpr then
    Result:=BoolToStr(TBoolConstExpr(Expr).Value,'true','false')
  else if C=TNilExpr then
    Result:='nil'
  else if C=TInheritedExpr then
    Result:='inherited'
  else if C=TUnaryExpr then
    Result:=OpcodeStrings[TUnaryExpr(Expr).OpCode]+ExprToText(TUnaryExpr(Expr).Operand)
  else if C=TBinaryExpr then
    begin
    Result:=ExprToText(TBinaryExpr(Expr).left);
    if OpcodeStrings[TBinaryExpr(Expr).OpCode]<>'' then
      Result:=Result+OpcodeStrings[TBinaryExpr(Expr).OpCode]
    else
      Result:=Result+' ';
    Result:=Result+ExprToText(TBinaryExpr(Expr).right)
    end
  else if C=TParamsExpr then
    begin
    case TParamsExpr(Expr).Kind of
      pekArrayParams: Result:=ExprToText(TParamsExpr(Expr).Value)
        +'['+ArrayExprToText(TParamsExpr(Expr).Params)+']';
      pekFuncParams: Result:=ExprToText(TParamsExpr(Expr).Value)
        +'('+ArrayExprToText(TParamsExpr(Expr).Params)+')';
      pekSet: Result:='['+ArrayExprToText(TParamsExpr(Expr).Params)+']';
      else ParseExc(nErrUnknownOperatorType,SErrUnknownOperatorType,[ExprKindNames[TParamsExpr(Expr).Kind]]);
    end;
    end
  else
    ParseExc(nErrUnknownOperatorType,SErrUnknownOperatorType,['TPasParser.ExprToText: '+Expr.ClassName]);
end;

function TPasParser.ArrayExprToText(Expr: TPasExprArray): String;
var
  i: Integer;
begin
  Result:='';
  for i:=0 to length(Expr)-1 do
    begin
    if i>0 then
      Result:=Result+',';
    Result:=Result+ExprToText(Expr[i]);
    end;
end;

function TPasParser.ResolveTypeReference(Name: string; Parent: TPasElement;
  ParamCnt: integer): TPasType;
var
  SS: Boolean;
  Ref: TPasElement;
begin
  Ref:=Nil;
  SS:=(not (po_ResolveStandardTypes in FOptions)) and isSimpleTypeToken(Name);
  if not SS then
    begin
    Ref:=Engine.FindElementFor(Name,Parent,ParamCnt);
    if Ref=nil then
      begin
      {$IFDEF VerbosePasResolver}
      {AllowWriteln}
      if po_resolvestandardtypes in FOptions then
        begin
        writeln('ERROR: TPasParser.ResolveTypeReference: resolver failed to raise an error');
        ParseExcExpectedIdentifier;
        end;
      {AllowWriteln-}
      {$ENDIF}
      end
    else if not (Ref is TPasType) then
      ParseExc(nParserExpectedTypeButGot,SParserExpectedTypeButGot,[Ref.ElementTypeName]);
    end;
  if (Ref=Nil) then
    Result:=TPasUnresolvedTypeRef(CreateElement(TPasUnresolvedTypeRef,Name,Parent))
  else
    begin
    Ref.AddRef{$IFDEF CheckPasTreeRefCount}('ResolveTypeReference'){$ENDIF};
    Result:=TPasType(Ref);
    end;
end;

function TPasParser.ParseParams(AParent: TPasElement; ParamsKind: TPasExprKind;
  AllowFormatting: Boolean = False): TParamsExpr;
var
  Params  : TParamsExpr;
  Expr    : TPasExpr;
  PClose  : TToken;

begin
  Result:=nil;
  if ParamsKind in [pekArrayParams, pekSet] then
    begin
    if CurToken<>tkSquaredBraceOpen then
      ParseExc(nParserExpectTokenError,SParserExpectTokenError,['[']);
    PClose:=tkSquaredBraceClose;
    end
  else
    begin
    if CurToken<>tkBraceOpen then
      ParseExc(nParserExpectTokenError,SParserExpectTokenError,['(']);
    PClose:=tkBraceClose;
    end;

  Params:=TParamsExpr(CreateElement(TParamsExpr,'',AParent,CurTokenPos));
  try
    Params.Kind:=ParamsKind;
    NextToken;
    if not isEndOfExp(false,false) then
      begin
      repeat
        Expr:=DoParseExpression(Params);
        if not Assigned(Expr) then
          ParseExcSyntaxError;
        Params.AddParam(Expr);
        if (CurToken=tkColon) then
          if Not AllowFormatting then
            ParseExc(nParserExpectTokenError,SParserExpectTokenError,[','])
          else
            begin
            NextToken;
            Expr.format1:=DoParseExpression(Expr);
            if (CurToken=tkColon) then
              begin
              NextToken;
              Expr.format2:=DoParseExpression(Expr);
              end;
            end;
        if not (CurToken in [tkComma, PClose]) then
          ParseExc(nParserExpectTokenError,SParserExpectTokenError,[',']);

        if CurToken = tkComma then
          begin
          NextToken;
          if CurToken = PClose then
            begin
            //ErrorExpected(parser, 'identifier');
            ParseExcSyntaxError;
            end;
          end;
      until CurToken=PClose;
      end;
    NextToken;
    Result:=Params;
  finally
    if Result=nil then
      Params.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.TokenToExprOp(AToken: TToken): TExprOpCode;

begin
  Case AToken of
    tkMul                   : Result:=eopMultiply;
    tkPlus                  : Result:=eopAdd;
    tkMinus                 : Result:=eopSubtract;
    tkDivision              : Result:=eopDivide;
    tkLessThan              : Result:=eopLessThan;
    tkEqual                 : Result:=eopEqual;
    tkGreaterThan           : Result:=eopGreaterThan;
    tkAt                    : Result:=eopAddress;
    tkAtAt                  : Result:=eopMemAddress;
    tkNotEqual              : Result:=eopNotEqual;
    tkLessEqualThan         : Result:=eopLessthanEqual;
    tkGreaterEqualThan      : Result:=eopGreaterThanEqual;
    tkPower                 : Result:=eopPower;
    tkSymmetricalDifference : Result:=eopSymmetricalDifference;
    tkIs                    : Result:=eopIs;
    tkAs                    : Result:=eopAs;
    tkSHR                   : Result:=eopSHR;
    tkSHL                   : Result:=eopSHL;
    tkAnd                   : Result:=eopAnd;
    tkOr                    : Result:=eopOR;
    tkXor                   : Result:=eopXOR;
    tkMod                   : Result:=eopMod;
    tkDiv                   : Result:=eopDiv;
    tkNot                   : Result:=eopNot;
    tkIn                    : Result:=eopIn;
    tkDot                   : Result:=eopSubIdent;
    tkCaret                 : Result:=eopDeref;
  else
    result:=eopAdd; // Fool compiler
    ParseExc(nParserNotAnOperand,SParserNotAnOperand,[AToken,TokenInfos[AToken]]);
  end;
end;

function TPasParser.ParseExprOperand(AParent: TPasElement): TPasExpr;
type
  TAllow = (aCannot, aCan, aMust);

  Function IsWriteOrStr(P : TPasExpr) : boolean;

  Var
    N : String;
  begin
    Result:=P is TPrimitiveExpr;
    if Result then
      begin
      N:=LowerCase(TPrimitiveExpr(P).Value);
      // We should actually resolve this to system.NNN
      Result:=(N='write') or (N='str') or (N='writeln') or (N='writestr');
      end;
  end;

  function IsSpecialize: boolean;
  var
    LookAhead, i: Integer;

    function Next: boolean;
    begin
      if LookAhead=FTokenRingSize then exit(false);
      NextToken;
      inc(LookAhead);
      Result:=true;
    end;

  begin
    Result:=false;
    LookAhead:=0;
    CheckToken(tkLessThan);
    try
      Next;
      if not (CurToken in [tkIdentifier,tkself]) then exit;
      while Next do
        case CurToken of
        tkDot:
          begin
          if not Next then exit;
          if not (CurToken in [tkIdentifier,tkself,tktrue,tkfalse]) then exit;
          end;
        tkComma:
          begin
          if not Next then exit;
          if not (CurToken in [tkIdentifier,tkself]) then exit;
          end;
        tkLessThan:
          begin
          // e.g. A<B<
          // not a valid comparison, could be a specialization -> good enough
          exit(true);
          end;
        tkGreaterThan:
          begin
          // e.g. A<B>
          exit(true);
          end;
        else
          exit;
        end;
    finally
      for i:=1 to LookAhead do
        UngetToken;
    end;
  end;

var
  Last, Func, Expr: TPasExpr;
  Params: TParamsExpr;
  Bin: TBinaryExpr;
  ok: Boolean;
  CanSpecialize: TAllow;
  aName: String;
  ISE: TInlineSpecializeExpr;
  SrcPos, ScrPos: TPasSourcePos;
  ProcType: TProcType;
  ProcExpr: TProcedureExpr;
begin
  Result:=nil;
  CanSpecialize:=aCannot;
  aName:='';
  case CurToken of
    tkString: Last:=CreatePrimitiveExpr(AParent,pekString,CurTokenString);
    tkChar:   Last:=CreatePrimitiveExpr(AParent,pekString,CurTokenText);
    tkNumber: Last:=CreatePrimitiveExpr(AParent,pekNumber,CurTokenString);
    tkIdentifier:
      begin
      if msDelphi in CurrentModeswitches then
        CanSpecialize:=aCan
      else
        CanSpecialize:=aCannot;
      aName:=CurTokenText;
      if (CompareText(aName,'self')=0) and not (tkself in Scanner.NonTokens) then
        Last:=CreateSelfExpr(AParent)
      else
        Last:=CreatePrimitiveExpr(AParent,pekIdent,aName);
      end;
    tkspecialize:
      begin
      CanSpecialize:=aMust;
      ExpectToken(tkIdentifier);
      aName:=CurTokenText;
      Last:=CreatePrimitiveExpr(AParent,pekIdent,aName);
      end;
    tkfalse, tktrue:    Last:=CreateBoolConstExpr(AParent,pekBoolConst, CurToken=tktrue);
    tknil:              Last:=CreateNilExpr(AParent);
    tkSquaredBraceOpen:
      begin
      Last:=ParseParams(AParent,pekSet);
      UngetToken;
      end;
    tkinherited:
      begin
      //inherited; inherited function
      Last:=CreateInheritedExpr(AParent);
      NextToken;
      if (CurToken=tkIdentifier) then
        begin
        SrcPos:=CurTokenPos;
        Bin:=CreateBinaryExpr(AParent,Last,ParseExprOperand(AParent),eopNone,SrcPos);
        if not Assigned(Bin.right) then
          begin
          Bin.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
          ParseExcExpectedIdentifier;
          end;
        Result:=Bin;
        exit;
        end;
      UngetToken;
      end;
    tkself:
      begin
      CanSpecialize:=aCan;
      aName:=CurTokenText;
      Last:=CreateSelfExpr(AParent);
      end;
    tkprocedure,tkfunction:
      begin
      if not IsAnonymousProcAllowed(AParent) then
        ParseExcExpectedIdentifier;
      if CurToken=tkprocedure then
        ProcType:=ptAnonymousProcedure
      else
        ProcType:=ptAnonymousFunction;
      try
        ProcExpr:=TProcedureExpr(CreateElement(TProcedureExpr,'',AParent,visPublic));
        ProcExpr.Proc:=TPasAnonymousProcedure(ParseProcedureOrFunctionDecl(ProcExpr,ProcType,false));
        Engine.FinishScope(stProcedure,ProcExpr.Proc);
        Result:=ProcExpr;
      finally
        if Result=nil then
          ProcExpr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
      exit; // do not allow postfix operators . ^. [] ()
      end;
    tkCaret:
      begin
      // Why is this still needed?
      // ^A..^_ characters
      NextToken;
      if not (length(CurTokenText)=1) or not (CurTokenText[1] in ['A'..'_']) then
        begin
        UngetToken;
        ParseExcExpectedIdentifier;
        end;
      Last:=CreatePrimitiveExpr(AParent,pekString, '^'+CurTokenText);
      end;
    tkBraceOpen:
      begin
      NextToken;
      Last:=DoParseExpression(AParent);
      if not Assigned(Last) then
        ParseExcSyntaxError;
      if (CurToken<>tkBraceClose) then
        begin
        Last.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
        CheckToken(tkBraceClose);
        end;
      end
  else
    ParseExcExpectedIdentifier;
  end;

  Result:=Last;
  ok:=false;
  ISE:=nil;
  try
    NextToken;
    Func:=Last;
    repeat
      case CurToken of
      tkDot:
        begin
        ScrPos:=CurTokenPos;
        NextToken;
        if CurToken=tkspecialize then
          begin
          // Obj.specialize ...
          if CanSpecialize=aMust then
            CheckToken(tkLessThan);
          CanSpecialize:=aMust;
          NextToken;
          end
        else if msDelphi in CurrentModeswitches then
          CanSpecialize:=aCan
        else
          CanSpecialize:=aCannot;
        if CurToken in [tkIdentifier,tktrue,tkfalse,tkself] then // true and false are sub identifiers as well
          begin
          aName:=aName+'.'+CurTokenString;
          Expr:=CreatePrimitiveExpr(AParent,pekIdent,CurTokenString);
          AddToBinaryExprChain(Result,Expr,eopSubIdent,ScrPos);
          Func:=Expr;
          NextToken;
          end
        else
          begin
          UngetToken;
          ParseExcExpectedIdentifier;
          end;
        end;
      tkBraceOpen,tkSquaredBraceOpen:
        begin
        if CurToken=tkBraceOpen then
          Params:=ParseParams(AParent,pekFuncParams,IsWriteOrStr(Func))
        else
          Params:=ParseParams(AParent,pekArrayParams);
        if not Assigned(Params) then Exit;
        Params.Value:=Result;
        Result.Parent:=Params;
        Result:=Params;
        CanSpecialize:=aCannot;
        Func:=nil;
        end;
      tkCaret:
        begin
        Result:=CreateUnaryExpr(AParent,Result,TokenToExprOp(CurToken));
        NextToken;
        CanSpecialize:=aCannot;
        Func:=nil;
        end;
      tkLessThan:
        begin
        SrcPos:=CurTokenPos;
        if CanSpecialize=aCannot then
          break
        else if (CanSpecialize=aCan) and not IsSpecialize then
          break
        else
          begin
          // an inline specialization (e.g. A<B,C>  or  something.A<B>)
          // check expression in front is an identifier
          Expr:=Result;
          if Expr.Kind=pekBinary then
            begin
            Bin:=TBinaryExpr(Expr);
            if Bin.OpCode<>eopSubIdent then
              ParseExcSyntaxError;
            Expr:=Bin.right;
            end
          else
            Bin:=nil;
          if Expr.Kind<>pekIdent then
            ParseExcSyntaxError;

          // read specialized params
          if Bin<>nil then
            ISE:=TInlineSpecializeExpr(CreateElement(TInlineSpecializeExpr,'',Bin,SrcPos))
          else
            ISE:=TInlineSpecializeExpr(CreateElement(TInlineSpecializeExpr,'',AParent,SrcPos));
          ReadSpecializeArguments(ISE,ISE.Params);

          // A<B>  or  something.A<B>
          ISE.NameExpr:=Expr;
          Expr.Parent:=ISE;
          if Bin<>nil then
            begin
            // something.A<B>
            Bin.Right:=ISE;
            end
          else
            begin
            // A<B>
            Result:=ISE;
            end;
          ISE:=nil;
          CanSpecialize:=aCannot;
          NextToken;
          end;
        Func:=nil;
        end
      else
        break;
      end;
    until false;
    ok:=true;
  finally
    if not ok then
      begin
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      ISE.Free;
      end;
  end;
end;

function TPasParser.ParseExpIdent(AParent: TPasElement): TPasExpr;
begin
  Result:=ParseExprOperand(AParent);
end;

function TPasParser.OpLevel(t: TToken): Integer;
begin
  case t of
  //  tkDot:
  //    Result:=5;
    tknot,tkAt,tkAtAt:
      Result:=4;
    tkMul, tkDivision, tkdiv, tkmod, tkand, tkShl,tkShr, tkas, tkPower, tkis:
      // Note that "is" has same precedence as "and" in Delphi and fpc, even though
      // some docs say otherwise. e.g. "Obj is TObj and aBool"
      Result:=3;
    tkPlus, tkMinus, tkor, tkxor:
      Result:=2;
    tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan, tkGreaterThan, tkGreaterEqualThan, tkin:
      Result:=1;
  else
    Result:=0;
  end;
end;

function TPasParser.DoParseExpression(AParent: TPaselement; InitExpr: TPasExpr;
  AllowEqual: Boolean): TPasExpr;
type
  TOpStackItem = record
    Token: TToken;
    SrcPos: TPasSourcePos;
  end;

var
  ExpStack  : TFPList; // list of TPasExpr
  OpStack   : array of TOpStackItem;
  OpStackTop: integer;
  PrefixCnt : Integer;
  x         : TPasExpr;
  i         : Integer;
  TempOp    : TToken;
  NotBinary : Boolean;

const
  PrefixSym = [tkPlus, tkMinus, tknot, tkAt, tkAtAt]; // + - not @ @@
  BinaryOP  = [tkMul, tkDivision, tkdiv, tkmod,  tkDotDot,
               tkand, tkShl,tkShr, tkas, tkPower,
               tkPlus, tkMinus, tkor, tkxor, tkSymmetricalDifference,
               tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan,
               tkGreaterThan, tkGreaterEqualThan, tkin, tkis];

  function PopExp: TPasExpr; inline;
  begin
    if ExpStack.Count>0 then begin
      Result:=TPasExpr(ExpStack[ExpStack.Count-1]);
      ExpStack.Delete(ExpStack.Count-1);
    end else
      Result:=nil;
  end;

  procedure PushOper(Token: TToken);
  begin
    inc(OpStackTop);
    if OpStackTop=length(OpStack) then
      SetLength(OpStack,length(OpStack)*2+4);
    OpStack[OpStackTop].Token:=Token;
    OpStack[OpStackTop].SrcPos:=CurTokenPos;
  end;

  function PeekOper: TToken; inline;
  begin
    if OpStackTop>=0 then Result:=OpStack[OpStackTop].Token
    else Result:=tkEOF;
  end;

  function PopOper(out SrcPos: TPasSourcePos): TToken;
  begin
    Result:=PeekOper;
    if Result=tkEOF then
      SrcPos:=DefPasSourcePos
    else
      begin
      SrcPos:=OpStack[OpStackTop].SrcPos;
      dec(OpStackTop);
      end;
  end;

  procedure PopAndPushOperator;
  var
    t       : TToken;
    xright  : TPasExpr;
    xleft   : TPasExpr;
    bin     : TBinaryExpr;
    SrcPos: TPasSourcePos;
  begin
    t:=PopOper(SrcPos);
    xright:=PopExp;
    xleft:=PopExp;
    if t=tkDotDot then
      begin
      bin:=CreateBinaryExpr(AParent,xleft,xright,eopNone,SrcPos);
      bin.Kind:=pekRange;
      end
    else
      bin:=CreateBinaryExpr(AParent,xleft,xright,TokenToExprOp(t),SrcPos);
    ExpStack.Add(bin);
  end;

Var
  AllowedBinaryOps : Set of TToken;
  SrcPos: TPasSourcePos;
begin
  AllowedBinaryOps:=BinaryOP;
  if Not AllowEqual then
    Exclude(AllowedBinaryOps,tkEqual);
  {$ifdef VerbosePasParser}
  //DumpCurToken('Entry',iaIndent);
  {$endif}
  Result:=nil;
  ExpStack := TFPList.Create;
  SetLength(OpStack,4);
  OpStackTop:=-1;
  try
    repeat
      NotBinary:=True;
      PrefixCnt:=0;
      if not Assigned(InitExpr) then
        begin

        // parse prefix operators
        while CurToken in PrefixSym do
          begin
          PushOper(CurToken);
          inc(PrefixCnt);
          NextToken;
          end;
        // parse operand
        x:=ParseExprOperand(AParent);
        if not Assigned(x) then
          ParseExcSyntaxError;
        ExpStack.Add(x);
        // apply prefixes
        for i:=1 to PrefixCnt do
          begin
          TempOp:=PopOper(SrcPos);
          x:=PopExp;
          if (TempOp=tkMinus) and (x.Kind=pekRange) then
            begin
            TBinaryExpr(x).Left:=CreateUnaryExpr(x, TBinaryExpr(x).left,
                                                 eopSubtract, SrcPos);
            ExpStack.Add(x);
            end
          else
            ExpStack.Add(CreateUnaryExpr(AParent, x, TokenToExprOp(TempOp), SrcPos));
          end;
        end
      else
        begin
        // the first part of the expression has been parsed externally.
        // this is used by Constant Expression parser (CEP) parsing only,
        // whenever it makes a false assuming on constant expression type.
        // i.e: SI_PAD_SIZE = ((128/sizeof(longint)) - 3);
        //
        // CEP assumes that it's array or record, because the expression
        // starts with "(". After the first part is parsed, the CEP meets "-"
        // that assures, it's not an array expression. The CEP should give the
        // first part back to the expression parser, to get the correct
        // token tree according to the operations priority.
        //
        // quite ugly. type information is required for CEP to work clean
        ExpStack.Add(InitExpr);
        InitExpr:=nil;
        end;
      if (CurToken in AllowedBinaryOPs) then
        begin
        // process operators of higher precedence than next operator
        NotBinary:=False;
        TempOp:=PeekOper;
        while (OpStackTop>=0) and (OpLevel(TempOp)>=OpLevel(CurToken)) do begin
          PopAndPushOperator;
          TempOp:=PeekOper;
        end;
        PushOper(CurToken);
        NextToken;
        end;
       //Writeln('Bin ',NotBinary ,' or EOE ',isEndOfExp, ' Ex ',Assigned(x),' stack ',ExpStack.Count);
    until NotBinary or isEndOfExp(AllowEqual, NotBinary);

    if not NotBinary then ParseExcExpectedIdentifier;

    while OpStackTop>=0 do PopAndPushOperator;

    // only 1 expression should be left on the OpStack
    if ExpStack.Count<>1 then
      ParseExcSyntaxError;
    Result:=TPasExpr(ExpStack[0]);
    Result.Parent:=AParent;

  finally
    {$ifdef VerbosePasParser}
    if Not Assigned(Result) then
      DumpCurToken('Exiting (no result)',iaUndent)
    else
      DumpCurtoken('Exiting (Result: "'+Result.GetDeclaration(true)+'") ',iaUndent);
    {$endif}
    if not Assigned(Result) then begin
      // expression error!
      for i:=0 to ExpStack.Count-1 do
        TPasExpr(ExpStack[i]).Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
    end;
    SetLength(OpStack,0);
    ExpStack.Free;
  end;
end;

function GetExprIdent(p: TPasExpr): String;
begin
  Result:='';
  if not Assigned(p) then exit;
  if (p.ClassType=TPrimitiveExpr) and (p.Kind=pekIdent) then
    Result:=TPrimitiveExpr(p).Value
  else if (p.ClassType=TSelfExpr) then
    Result:='Self';
end;

function TPasParser.DoParseConstValueExpression(AParent: TPasElement): TPasExpr;
// sets CurToken to token behind expression

  function lastfield:boolean;

  begin
    Result:=CurToken<>tkSemicolon;
    if not Result then
     begin
       NextToken;
       if CurToken=tkBraceClose then
         Result:=true
       else
         UngetToken;
     end;
  end;

  procedure ReadArrayValues(x : TPasExpr);
  var
    a: TArrayValues;
  begin
    Result:=nil;
    a:=nil;
    try
      a:=CreateArrayValues(AParent);
      if x<>nil then
        begin
        a.AddValues(x);
        x:=nil;
        end;
      repeat
        NextToken;
        a.AddValues(DoParseConstValueExpression(a));
      until CurToken<>tkComma;
      Result:=a;
    finally
      if Result=nil then
        begin
        a.Free;
        x.Free;
        end;
    end;
  end;

var
  x , v: TPasExpr;
  n : String;
  r : TRecordValues;
begin
  if CurToken <> tkBraceOpen then
    Result:=DoParseExpression(AParent)
  else begin
    Result:=nil;
    if Engine.NeedArrayValues(AParent) then
      ReadArrayValues(nil)
    else
      begin
      NextToken;
      x:=DoParseConstValueExpression(AParent);
      case CurToken of
        tkComma: // array of values (a,b,c);
          ReadArrayValues(x);

        tkColon: // record field (a:xxx;b:yyy;c:zzz);
          begin
          if not (x is TPrimitiveExpr) then
            CheckToken(tkBraceClose);
          r:=nil;
          try
            n:=GetExprIdent(x);
            r:=CreateRecordValues(AParent);
            NextToken;
            v:=DoParseConstValueExpression(r);
            r.AddField(TPrimitiveExpr(x), v);
            x:=nil;
            if not lastfield then
              repeat
                n:=ExpectIdentifier;
                x:=CreatePrimitiveExpr(r,pekIdent,n);
                ExpectToken(tkColon);
                NextToken;
                v:=DoParseConstValueExpression(AParent);
                r.AddField(TPrimitiveExpr(x), v);
                x:=nil;
              until lastfield; // CurToken<>tkSemicolon;
            Result:=r;
          finally
            if Result=nil then
              begin
              r.Free;
              x.Free;
              end;
          end;
          end;
      else
        // Binary expression!  ((128 div sizeof(longint)) - 3);
        Result:=DoParseExpression(AParent,x);
        if CurToken<>tkBraceClose then
          begin
          ReleaseAndNil(TPasElement(Result){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
          ParseExc(nParserExpectedCommaRBracket,SParserExpectedCommaRBracket);
          end;
        NextToken;
        if CurToken <> tkSemicolon then // the continue of expression
          Result:=DoParseExpression(AParent,Result);
        Exit;
      end;
      end;
    if CurToken<>tkBraceClose then
      begin
      ReleaseAndNil(TPasElement(Result){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
      ParseExc(nParserExpectedCommaRBracket,SParserExpectedCommaRBracket);
      end;
    NextToken;
  end;
end;

function TPasParser.CheckOverloadList(AList: TFPList; AName: String; out
  OldMember: TPasElement): TPasOverloadedProc;

Var
  I : Integer;

begin
  Result:=Nil;
  I:=0;
  While (Result=Nil) and (I<AList.Count) do
    begin
    OldMember:=TPasElement(AList[i]);
    if CompareText(OldMember.Name, AName) = 0 then
      begin
      if OldMember is TPasOverloadedProc then
        Result:=TPasOverloadedProc(OldMember)
      else
        begin
        Result:=TPasOverloadedProc(CreateElement(TPasOverloadedProc, AName, OldMember.Parent));
        OldMember.Parent:=Result;
        Result.Visibility:=OldMember.Visibility;
        Result.Overloads.Add(OldMember);
        Result.SourceFilename:=OldMember.SourceFilename;
        Result.SourceLinenumber:=OldMember.SourceLinenumber;
        Result.DocComment:=Oldmember.DocComment;
        AList[i] := Result;
        end;
      end;
    Inc(I);
    end;
  If Result=Nil then
    OldMember:=Nil;
end;

procedure TPasParser.AddProcOrFunction(Decs: TPasDeclarations;
  AProc: TPasProcedure);
var
  I : Integer;
  OldMember: TPasElement;
  OverloadedProc: TPasOverloadedProc;
begin
  OldMember:=nil;
  With Decs do
    begin
    if not (po_nooverloadedprocs in Options) then
      OverloadedProc:=CheckOverloadList(Functions,AProc.Name,OldMember)
    else
      OverloadedProc:=nil;
    If (OverloadedProc<>Nil) then
      begin
      OverLoadedProc.Overloads.Add(AProc);
      if (OldMember<>OverloadedProc) then
        begin
        I:=Declarations.IndexOf(OldMember);
        If I<>-1 then
          Declarations[i]:=OverloadedProc;
        end;
      end
    else
      begin
      Declarations.Add(AProc);
      Functions.Add(AProc);
      end;
    end;
  Engine.FinishScope(stProcedure,AProc);
end;

// Return the parent of a function declaration. This is AParent,
// except when AParent is a class/record and the function is overloaded.
// Then the parent is the overload object.
function TPasParser.CheckIfOverloaded(AParent: TPasElement; const AName: String): TPasElement;
var
  Member: TPasElement;
  OverloadedProc: TPasOverloadedProc;

begin
  Result:=AParent;
  If (not (po_nooverloadedprocs in Options)) and (AParent is TPasMembersType) then
    begin
    OverloadedProc:=CheckOverLoadList(TPasMembersType(AParent).Members,AName,Member);
    If (OverloadedProc<>Nil) then
      Result:=OverloadedProc;
    end;
end;

procedure TPasParser.ParseMain(var Module: TPasModule);
begin
  Module:=nil;
  NextToken;
  SaveComments;
  case CurToken of
    tkUnit:
      ParseUnit(Module);
    tkProgram:
      ParseProgram(Module);
    tkLibrary:
      ParseLibrary(Module);
    tkEOF:
      CheckToken(tkprogram);
  else
    UngetToken;
    ParseProgram(Module,True);
  end;
end;

// Starts after the "unit" token
procedure TPasParser.ParseUnit(var Module: TPasModule);
var
  AUnitName: String;
  StartPos: TPasSourcePos;
  HasFinished: Boolean;
begin
  StartPos:=CurTokenPos;
  Module := nil;
  AUnitName := ExpectIdentifier;
  NextToken;
  while CurToken = tkDot do
    begin
    ExpectIdentifier;
    AUnitName := AUnitName + '.' + CurTokenString;
    NextToken;
    end;
  UngetToken;
  Module := TPasModule(CreateElement(TPasModule, AUnitName, Engine.Package, StartPos));
  FCurModule:=Module;
  HasFinished:=true;
  try
    Scanner.CurModuleName:=AUnitName;
    if Assigned(Engine.Package) then
      begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
      Module.AddRef{$IFDEF CheckPasTreeRefCount}('TPasPackage.Modules'){$ENDIF};
      end;
    CheckHint(Module,True);
    ExpectToken(tkInterface);
    if po_StopOnUnitInterface in Options then
      begin
      HasFinished:=false;
      {$IFDEF VerbosePasResolver}
      writeln('TPasParser.ParseUnit pause parsing after unit name ',CurModule.Name);
      {$ENDIF}
      exit;
      end;
    ParseInterface;
    if (Module.InterfaceSection<>nil)
        and (Module.InterfaceSection.PendingUsedIntf<>nil) then
      begin
      HasFinished:=false;
      {$IFDEF VerbosePasResolver}
      writeln('TPasParser.ParseUnit pause parsing after interface uses list ',CurModule.Name);
      {$ENDIF}
      end;
    if (Module.ImplementationSection<>nil)
        and (Module.ImplementationSection.PendingUsedIntf<>nil) then
      begin
      HasFinished:=false;
      {$IFDEF VerbosePasResolver}
      writeln('TPasParser.ParseUnit pause parsing after implementation uses list ',CurModule.Name);
      {$ENDIF}
      end;
    if HasFinished then
      FinishedModule;
  finally
    if HasFinished then
      //begin
      //Module.Release{$IFDEF CheckPasTreeRefCount}('TPasPackage.Modules'){$ENDIF};
      FCurModule:=nil; // clear module if there is an error or finished parsing
      //end;
  end;
end;

function TPasParser.GetLastSection: TPasSection;
begin
  Result:=nil;
  if FCurModule=nil then
    exit; // parse completed
  if CurModule is TPasProgram then
    Result:=TPasProgram(CurModule).ProgramSection
  else if CurModule is TPasLibrary then
    Result:=TPasLibrary(CurModule).LibrarySection
  else if (CurModule.ClassType=TPasModule) or (CurModule is TPasUnitModule) then
    begin
    if CurModule.ImplementationSection<>nil then
      Result:=CurModule.ImplementationSection
    else
      Result:=CurModule.InterfaceSection; // might be nil
    end;
end;

function TPasParser.CanParseContinue(out Section: TPasSection): boolean;
begin
  Result:=false;
  Section:=nil;
  if FCurModule=nil then
    exit; // parse completed
  if (LastMsg<>'') and (LastMsgType<=mtError) then
    begin
    {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
    writeln('TPasParser.CanParseContinue ',CurModule.Name,' LastMsg="',LastMsgType,':',LastMsg,'"');
    {$ENDIF}
    exit;
    end;
  if (Scanner.LastMsg<>'') and (Scanner.LastMsgType<=mtError) then
    begin
    {$IF defined(VerbosePasResolver) or defined(VerboseUnitQueue)}
    writeln('TPasParser.CanParseContinue ',CurModule.Name,' Scanner.LastMsg="',Scanner.LastMsgType,':',Scanner.LastMsg,'"');
    {$ENDIF}
    exit;
    end;

  Section:=GetLastSection;
  if Section=nil then
    if (po_StopOnUnitInterface in Options)
        and ((CurModule is TPasUnitModule) or (CurModule.ClassType=TPasModule))
        and (CurModule.InterfaceSection=nil) then
      exit(true)
    else
      begin
      {$IFDEF VerboseUnitQueue}
      writeln('TPasParser.CanParseContinue ',CurModule.Name,' no LastSection');
      {$ENDIF}
      exit(false);
      end;
  Result:=Section.PendingUsedIntf=nil;
  {$IFDEF VerboseUnitQueue}
  writeln('TPasParser.CanParseContinue ',CurModule.Name,' Result=',Result,' ',Section.ElementTypeName);
  {$ENDIF}
end;

procedure TPasParser.ParseContinue;
// continue parsing after stopped due to pending uses
var
  Section: TPasSection;
  HasFinished: Boolean;
begin
  if CurModule=nil then
    ParseExcTokenError('TPasParser.ParseContinue missing module');
  {$IFDEF VerbosePasParser}
  writeln('TPasParser.ParseContinue ',CurModule.Name);
  {$ENDIF}
  if not CanParseContinue(Section) then
    ParseExcTokenError('TPasParser.ParseContinue missing section');
  HasFinished:=true;
  try
    if Section=nil then
      begin
      // continue after unit name
      ParseInterface;
      end
    else
      begin
      // continue after uses clause
      Engine.FinishScope(stUsesClause,Section);
      ParseDeclarations(Section);
      end;
    Section:=GetLastSection;
    if Section=nil then
      ParseExc(nErrNoSourceGiven,'[20180306112327]');
    if Section.PendingUsedIntf<>nil then
      HasFinished:=false;
    if HasFinished then
      FinishedModule;
  finally
    if HasFinished then
      FCurModule:=nil; // clear module if there is an error or finished parsing
  end;
end;

// Starts after the "program" token
procedure TPasParser.ParseProgram(var Module: TPasModule; SkipHeader : Boolean = False);
Var
  PP : TPasProgram;
  Section : TProgramSection;
  N : String;
  StartPos: TPasSourcePos;
  HasFinished: Boolean;
  {$IFDEF VerbosePasResolver}
  aSection: TPasSection;
  {$ENDIF}
begin
  StartPos:=CurTokenPos;
  if SkipHeader then
    N:=ChangeFileExt(Scanner.CurFilename,'')
  else
    begin
    N:=ExpectIdentifier;
    NextToken;
    while CurToken = tkDot do
      begin
      ExpectIdentifier;
      N := N + '.' + CurTokenString;
      NextToken;
      end;
    UngetToken;
    end;
  Module := nil;
  PP:=TPasProgram(CreateElement(TPasProgram, N, Engine.Package, StartPos));
  Module :=PP;
  HasFinished:=true;
  FCurModule:=Module;
  try
    Scanner.CurModuleName:=N;
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    if not SkipHeader then
      begin
      NextToken;
      If (CurToken=tkBraceOpen) then
        begin
        PP.InputFile:=ExpectIdentifier;
        NextToken;
        if Not (CurToken in [tkBraceClose,tkComma]) then
          ParseExc(nParserExpectedCommaRBracket,SParserExpectedCommaRBracket);
        If (CurToken=tkComma) then
          PP.OutPutFile:=ExpectIdentifier;
        ExpectToken(tkBraceClose);
        NextToken;
        end;
      if (CurToken<>tkSemicolon) then
        ParseExcTokenError(';');
      end;
    Section := TProgramSection(CreateElement(TProgramSection, '', CurModule));
    PP.ProgramSection := Section;
    ParseOptionalUsesList(Section);
    HasFinished:=Section.PendingUsedIntf=nil;
    if not HasFinished then
      begin
      {$IFDEF VerbosePasResolver}
      {AllowWriteln}
      writeln('TPasParser.ParseProgram pause parsing after uses list of "',CurModule.Name,'"');
      if CanParseContinue(aSection) then
        begin
        writeln('TPasParser.ParseProgram Section=',Section.ClassName,' Section.PendingUsedIntf=',Section.PendingUsedIntf<>nil);
        if aSection<>nil then
          writeln('TPasParser.ParseProgram aSection=',aSection.ClassName,' ',Section=aSection);
        ParseExc(nErrNoSourceGiven,'[20180305172432] ');
        end;
      {AllowWriteln-}
      {$ENDIF}
      exit;
      end;
    ParseDeclarations(Section);
    FinishedModule;
  finally
    if HasFinished then
      FCurModule:=nil; // clear module if there is an error or finished parsing
  end;
end;

// Starts after the "library" token
procedure TPasParser.ParseLibrary(var Module: TPasModule);
Var
  PP : TPasLibrary;
  Section : TLibrarySection;
  N: String;
  StartPos: TPasSourcePos;
  HasFinished: Boolean;

begin
  StartPos:=CurTokenPos;
  N:=ExpectIdentifier;
  NextToken;
  while CurToken = tkDot do
    begin
    ExpectIdentifier;
    N := N + '.' + CurTokenString;
    NextToken;
    end;
  UngetToken;
  Module := nil;
  PP:=TPasLibrary(CreateElement(TPasLibrary, N, Engine.Package, StartPos));
  Module :=PP;
  HasFinished:=true;
  FCurModule:=Module;
  try
    Scanner.CurModuleName:=N;
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    NextToken;
    if (CurToken<>tkSemicolon) then
        ParseExcTokenError(';');
    Section := TLibrarySection(CreateElement(TLibrarySection, '', CurModule));
    PP.LibrarySection := Section;
    ParseOptionalUsesList(Section);
    HasFinished:=Section.PendingUsedIntf=nil;
    if not HasFinished then
      exit;
    ParseDeclarations(Section);
    FinishedModule;
  finally
    if HasFinished then
      FCurModule:=nil; // clear module if there is an error or finished parsing
  end;
end;

procedure TPasParser.ParseOptionalUsesList(ASection: TPasSection);
// checks if next token is Uses keyword and reads the uses list
begin
  NextToken;
  CheckImplicitUsedUnits(ASection);
  if CurToken=tkuses then
    ParseUsesList(ASection)
  else
    UngetToken;
  Engine.CheckPendingUsedInterface(ASection);
  if ASection.PendingUsedIntf<>nil then
    exit;
  Engine.FinishScope(stUsesClause,ASection);
end;

// Starts after the "interface" token
procedure TPasParser.ParseInterface;
var
  Section: TInterfaceSection;
begin
  If LogEvent(pleInterface) then
    DoLog(mtInfo,nLogStartInterface,SLogStartInterface);
  Section := TInterfaceSection(CreateElement(TInterfaceSection, '', CurModule));
  CurModule.InterfaceSection := Section;
  ParseOptionalUsesList(Section);
  if Section.PendingUsedIntf<>nil then
    exit;
  ParseDeclarations(Section); // this also parses the Implementation section
end;

// Starts after the "implementation" token
procedure TPasParser.ParseImplementation;
var
  Section: TImplementationSection;
begin
  Section := TImplementationSection(CreateElement(TImplementationSection, '', CurModule));
  CurModule.ImplementationSection := Section;
  ParseOptionalUsesList(Section);
  if Section.PendingUsedIntf<>nil then
    exit;
  ParseDeclarations(Section);
end;

procedure TPasParser.ParseInitialization;
var
  Section: TInitializationSection;
  SubBlock: TPasImplElement;
begin
  Section := TInitializationSection(CreateElement(TInitializationSection, '', CurModule,CurTokenPos));
  CurModule.InitializationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      Engine.FinishScope(stInitialFinalization,Section);
      exit;
    end
    else if (CurToken=tkfinalization) then
    begin
      Engine.FinishScope(stInitialFinalization,Section);
      ParseFinalization;
      exit;
    end
    else if CurToken<>tkSemiColon then
    begin
      UngetToken;
      ParseStatement(Section,SubBlock);
      if SubBlock=nil then
        ExpectToken(tkend);
    end;
  until false;
end;

procedure TPasParser.ParseFinalization;
var
  Section: TFinalizationSection;
  SubBlock: TPasImplElement;
begin
  Section := TFinalizationSection(CreateElement(TFinalizationSection, '', CurModule));
  CurModule.FinalizationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      Engine.FinishScope(stInitialFinalization,Section);
      exit;
    end
    else if CurToken<>tkSemiColon then
    begin
      UngetToken;
      ParseStatement(Section,SubBlock);
      if SubBlock=nil then
        ExpectToken(tkend);
    end;
  until false;
end;

function TPasParser.GetProcTypeFromToken(tk: TToken; IsClass: Boolean
  ): TProcType;
begin
  Result:=ptProcedure;
  Case tk of
    tkProcedure :
      if IsClass then
        Result:=ptClassProcedure
      else
        Result:=ptProcedure;
    tkFunction:
      if IsClass then
        Result:=ptClassFunction
      else
        Result:=ptFunction;
    tkConstructor:
      if IsClass then
        Result:=ptClassConstructor
      else
        Result:=ptConstructor;
    tkDestructor:
      if IsClass then
        Result:=ptClassDestructor
      else
        Result:=ptDestructor;
    tkOperator:
      if IsClass then
        Result:=ptClassOperator
      else
        Result:=ptOperator;
  else
    ParseExc(nParserNotAProcToken,SParserNotAProcToken);
  end;
end;

procedure TPasParser.ParseDeclarations(Declarations: TPasDeclarations);
var
  HadTypeSection: boolean;
  CurBlock: TDeclType;

  procedure SetBlock(NewBlock: TDeclType);
  begin
    if CurBlock=NewBlock then exit;
    if CurBlock=declType then
      begin
      if msDelphi in CurrentModeswitches then
        // Delphi allows forward types only inside a type section
        Engine.FinishScope(stTypeSection,Declarations);
      end;
    if NewBlock=declType then
      HadTypeSection:=true
    else if (NewBlock=declNone) and HadTypeSection then
      begin
      HadTypeSection:=false;
      if not (msDelphi in CurrentModeswitches) then
        // ObjFPC allows forward types inside a whole section
        Engine.FinishScope(stTypeSection,Declarations);
      end;
    CurBlock:=NewBlock;
    Scanner.SetForceCaret(NewBlock=declType);
  end;

var
  ConstEl: TPasConst;
  ResStrEl: TPasResString;
  TypeEl: TPasType;
  ClassEl: TPasClassType;
  List: TFPList;
  i,j: Integer;
  ExpEl: TPasExportSymbol;
  PropEl : TPasProperty;
  PT : TProcType;
  ok, MustBeGeneric: Boolean;
  Proc: TPasProcedure;
  CurEl: TPasElement;
begin
  CurBlock := declNone;
  HadTypeSection:=false;
  while True do
  begin
    if CurBlock in [DeclNone,declConst,declType,declVar] then
      Scanner.SetTokenOption(toOperatorToken)
    else
      Scanner.UnSetTokenOption(toOperatorToken);
    NextToken;
    Scanner.SkipGlobalSwitches:=true;
  //  writeln('TPasParser.ParseDeclarations Token=',CurTokenString,' ',CurToken, ' ',scanner.CurFilename);
    case CurToken of
    tkend:
      begin
      If (CurModule is TPasProgram) and (CurModule.InitializationSection=Nil) then
        ParseExcTokenError('begin');
      ExpectToken(tkDot);
      break;
      end;
    tkimplementation:
      if (Declarations is TInterfaceSection) then
        begin
        If Not Engine.InterfaceOnly then
          begin
          If LogEvent(pleImplementation) then
            DoLog(mtInfo,nLogStartImplementation,SLogStartImplementation);
          SetBlock(declNone);
          ParseImplementation;
          end;
        break;
        end
      else
        ParseExcSyntaxError;
    tkinitialization:
      if (Declarations is TInterfaceSection)
      or ((Declarations is TImplementationSection) and not (Declarations is TProgramSection)) then
        begin
        SetBlock(declNone);
        ParseInitialization;
        break;
        end
      else
        ParseExcSyntaxError;
    tkfinalization:
      if (Declarations is TInterfaceSection)
      or ((Declarations is TImplementationSection) and not (Declarations is TProgramSection)) then
        begin
        SetBlock(declNone);
        ParseFinalization;
        break;
        end;
    tkUses:
      if Declarations.ClassType=TInterfaceSection then
        ParseExcTokenError(TokenInfos[tkimplementation])
      else if Declarations is TPasSection then
        ParseExcTokenError(TokenInfos[tkend])
      else
        ParseExcSyntaxError;
    tkConst:
      SetBlock(declConst);
    tkexports:
      SetBlock(declExports);
    tkResourcestring:
      if Declarations is TPasSection then
        SetBlock(declResourcestring)
      else
        begin
        {$IFDEF VerbosePasParser}
        writeln('TPasParser.ParseDeclarations ',Declarations.Parent.ClassName);
        {$ENDIF}
        ParseExc(nParserResourcestringsMustBeGlobal,SParserResourcestringsMustBeGlobal);
        end;
    tkType:
      SetBlock(declType);
    tkVar:
      SetBlock(declVar);
    tkThreadVar:
      SetBlock(declThreadVar);
    tkProperty:
      SetBlock(declProperty);
    tkProcedure, tkFunction, tkConstructor, tkDestructor, tkOperator:
      begin
      MustBeGeneric:=(not (msDelphi in CurrentModeswitches)) and (GetPrevToken=tkgeneric);
      SetBlock(declNone);
      SaveComments;
      pt:=GetProcTypeFromToken(CurToken);
      AddProcOrFunction(Declarations, ParseProcedureOrFunctionDecl(Declarations, pt, MustBeGeneric));
      end;
    tkClass:
      begin
      MustBeGeneric:=(not (msDelphi in CurrentModeswitches)) and (GetPrevToken=tkgeneric);
      SetBlock(declNone);
      SaveComments;
      NextToken;
      CheckTokens([tkprocedure,tkFunction,tkConstructor,tkDestructor,tkoperator]);
      pt:=GetProcTypeFromToken(CurToken,True);
      AddProcOrFunction(Declarations,ParseProcedureOrFunctionDecl(Declarations, pt, MustBeGeneric));
      end;
    tkAbsolute,
    tkIdentifier:
      begin
      Scanner.UnSetTokenOption(toOperatorToken);
      SaveComments;
      case CurBlock of
        declConst:
          begin
            ConstEl := ParseConstDecl(Declarations);
            Declarations.Declarations.Add(ConstEl);
            Declarations.Consts.Add(ConstEl);
            Engine.FinishScope(stDeclaration,ConstEl);
          end;
        declResourcestring:
          begin
            ResStrEl := ParseResourcestringDecl(Declarations);
            Declarations.Declarations.Add(ResStrEl);
            Declarations.ResStrings.Add(ResStrEl);
            Engine.FinishScope(stResourceString,ResStrEl);
          end;
        declType:
          begin
          TypeEl := ParseTypeDecl(Declarations);
          // Scanner.SetForceCaret(OldForceCaret); // It may have been switched off
          if Assigned(TypeEl) then        // !!!
            begin
            Declarations.Declarations.Add(TypeEl);
            {$IFDEF CheckPasTreeRefCount}if TypeEl.RefIds.IndexOf('CreateElement')>=0 then TypeEl.ChangeRefId('CreateElement','TPasDeclarations.Children');{$ENDIF}
            if (TypeEl.ClassType = TPasClassType)
                and (not (po_keepclassforward in Options)) then
            begin
              // Remove previous forward declarations, if necessary
              for i := 0 to Declarations.Classes.Count - 1 do
              begin
                ClassEl := TPasClassType(Declarations.Classes[i]);
                if CompareText(ClassEl.Name, TypeEl.Name) = 0 then
                begin
                  Declarations.Classes.Delete(i);
                  for j := 0 to Declarations.Declarations.Count - 1 do
                    if CompareText(TypeEl.Name,
                      TPasElement(Declarations.Declarations[j]).Name) = 0 then
                    begin
                      Declarations.Declarations.Delete(j);
                      break;
                    end;
                  ClassEl.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
                  break;
                end;
              end;
              // Add the new class to the class list
              Declarations.Classes.Add(TypeEl)
            end else
              Declarations.Types.Add(TypeEl);
            end;
          end;
        declExports:
          begin
          List := TFPList.Create;
          try
            ok:=false;
            try
              ParseExportDecl(Declarations, List);
              ok:=true;
            finally
              if not ok then
                for i := 0 to List.Count - 1 do
                  TPasExportSymbol(List[i]).Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
            end;
            for i := 0 to List.Count - 1 do
            begin
              ExpEl := TPasExportSymbol(List[i]);
              Declarations.Declarations.Add(ExpEl);
              {$IFDEF CheckPasTreeRefCount}ExpEl.ChangeRefId('CreateElement','TPasDeclarations.Children');{$ENDIF}
              Declarations.ExportSymbols.Add(ExpEl);
            end;
          finally
            List.Free;
          end;
          end;
        declVar, declThreadVar:
          begin
            List := TFPList.Create;
            try
              ParseVarDecl(Declarations, List);
              for i := 0 to List.Count - 1 do
              begin
                CurEl := TPasElement(List[i]);
                Declarations.Declarations.Add(CurEl);
                if CurEl.ClassType=TPasAttributes then
                  Declarations.Attributes.Add(CurEl)
                else
                  Declarations.Variables.Add(TPasVariable(CurEl));
                Engine.FinishScope(stDeclaration,CurEl);
              end;
              CheckToken(tkSemicolon);
            finally
              List.Free;
            end;
          end;
        declProperty:
          begin
          PropEl:=ParseProperty(Declarations,CurtokenString,visDefault,false);
          Declarations.Declarations.Add(PropEl);
          {$IFDEF CheckPasTreeRefCount}PropEl.ChangeRefId('CreateElement','TPasDeclarations.Children');{$ENDIF}
          Declarations.Properties.Add(PropEl);
          Engine.FinishScope(stDeclaration,PropEl);
          end;
      else
        ParseExcSyntaxError;
      end;
      end;
    tkGeneric:
      begin
      NextToken;
      if (CurToken in [tkclass,tkprocedure,tkfunction]) then
        begin
        if msDelphi in CurrentModeswitches then
          ParseExcSyntaxError; // inconsistency, tkGeneric should be in Scanner.NonTokens
        SetBlock(declNone);
        UngetToken;
        end;
      if CurBlock = declType then
        begin
        CheckToken(tkIdentifier);
        ParseGenericTypeDecl(Declarations,true);
        end
      else if CurBlock = declNone then
        begin
        if msDelphi in CurrentModeswitches then
          ParseExcSyntaxError; // inconsistency, tkGeneric should be in Scanner.NonTokens
        SetBlock(declNone);
        SaveComments;
        NextToken;
        case CurToken of
        tkclass:
          begin
          // generic class ...
          NextToken;
          if not (CurToken in [tkprocedure,tkfunction]) then
            ParseExcSyntaxError;
          // generic class procedure ...
          pt:=GetProcTypeFromToken(CurToken,true);
          AddProcOrFunction(Declarations, ParseProcedureOrFunctionDecl(Declarations, pt, true));
          end;
        tkprocedure,tkfunction:
          begin
          // generic procedure ...
          SetBlock(declNone);
          SaveComments;
          pt:=GetProcTypeFromToken(CurToken);
          AddProcOrFunction(Declarations, ParseProcedureOrFunctionDecl(Declarations, pt, true));
          end;
        else
          ParseExcSyntaxError;
        end;
        end
      else
        begin
        ParseExcSyntaxError;
        end;
      end;
    tkbegin:
      begin
      if Declarations is TProcedureBody then
        begin
        Proc:=Declarations.Parent as TPasProcedure;
        if pmAssembler in Proc.Modifiers then
          ParseExc(nParserExpectTokenError,SParserExpectTokenError,['asm']);
        SetBlock(declNone);
        ParseProcBeginBlock(TProcedureBody(Declarations));
        break;
        end
      else if (Declarations is TInterfaceSection)
          or (Declarations is TImplementationSection) then
        begin
        SetBlock(declNone);
        ParseInitialization;
        break;
        end
      else
        ParseExcSyntaxError;
      end;
    tkasm:
      begin
      if Declarations is TProcedureBody then
        begin
        Proc:=Declarations.Parent as TPasProcedure;
        // Assembler keyword is optional in Delphi mode (bug 31690)
        if not ((pmAssembler in Proc.Modifiers) or (msDelphi in CurrentModeswitches)) then
          ParseExc(nParserExpectTokenError,SParserExpectTokenError,['begin']);
        SetBlock(declNone);
        ParseProcAsmBlock(TProcedureBody(Declarations));
        break;
        end
      else
        ParseExcSyntaxError;
      end;
    tklabel:
      begin
      SetBlock(declNone);
      if not (Declarations is TInterfaceSection) then
        ParseLabels(Declarations);
      end;
    tkSquaredBraceOpen:
      if msPrefixedAttributes in CurrentModeSwitches then
        ParseAttributes(Declarations,true)
      else
        ParseExcSyntaxError;
    else
      ParseExcSyntaxError;
    end;
  end;
  SetBlock(declNone);
end;

function TPasParser.AddUseUnit(ASection: TPasSection;
  const NamePos: TPasSourcePos; AUnitName: string; NameExpr: TPasExpr;
  InFileExpr: TPrimitiveExpr): TPasUsesUnit;

  procedure CheckDuplicateInUsesList(UsesClause: TPasUsesClause);
  var
    i: Integer;
  begin
    if UsesClause=nil then exit;
    for i:=0 to length(UsesClause)-1 do
      if CompareText(AUnitName,UsesClause[i].Name)=0 then
        ParseExc(nParserDuplicateIdentifier,SParserDuplicateIdentifier,[AUnitName]);
  end;

var
  UnitRef: TPasElement;
  UsesUnit: TPasUsesUnit;
begin
  Result:=nil;
  UsesUnit:=nil;
  UnitRef:=nil;
  try
    {$IFDEF VerbosePasParser}
    writeln('TPasParser.AddUseUnit AUnitName=',AUnitName,' CurModule.Name=',CurModule.Name);
    {$ENDIF}
    if CompareText(AUnitName,CurModule.Name)=0 then
      begin
      if CompareText(AUnitName,'System')=0 then
        exit; // for compatibility ignore implicit use of system in system
      ParseExc(nParserDuplicateIdentifier,SParserDuplicateIdentifier,[AUnitName]);
      end;

    // Note: The alias (AUnitName) must be unique within a module.
    //       Using an unit module twice with different alias is allowed.
    CheckDuplicateInUsesList(ASection.UsesClause);
    if ASection.ClassType=TImplementationSection then
      CheckDuplicateInUsesList(CurModule.InterfaceSection.UsesClause);

    UnitRef := Engine.FindModule(AUnitName,NameExpr,InFileExpr);
    if Assigned(UnitRef) then
      UnitRef.AddRef{$IFDEF CheckPasTreeRefCount}('TPasUsesUnit.Module'){$ENDIF}
    else
      UnitRef := TPasUnresolvedUnitRef(CreateElement(TPasUnresolvedUnitRef,
        AUnitName, ASection, NamePos));

    UsesUnit:=TPasUsesUnit(CreateElement(TPasUsesUnit,AUnitName,ASection,NamePos));
    Result:=ASection.AddUnitToUsesList(AUnitName,NameExpr,InFileExpr,UnitRef,UsesUnit);
    if InFileExpr<>nil then
      begin
      if UnitRef is TPasModule then
        begin
        if TPasModule(UnitRef).Filename='' then
          TPasModule(UnitRef).Filename:=InFileExpr.Value;
        end
      else if UnitRef is TPasUnresolvedUnitRef then
        TPasUnresolvedUnitRef(UnitRef).FileName:=InFileExpr.Value;
      end;
  finally
    if Result=nil then
      begin
      if UsesUnit<>nil then
        UsesUnit.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if NameExpr<>nil then
        NameExpr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if InFileExpr<>nil then
        InFileExpr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if UnitRef<>nil then
        UnitRef.Release{$IFDEF CheckPasTreeRefCount}('FindModule'){$ENDIF};
      end;
  end;
end;

procedure TPasParser.CheckImplicitUsedUnits(ASection: TPasSection);
var
  i: Integer;
  NamePos: TPasSourcePos;
begin
  If not (ASection.ClassType=TImplementationSection) Then // interface,program,library,package
    begin
    // load implicit units, like 'System'
    NamePos:=CurSourcePos;
    for i:=0 to ImplicitUses.Count-1 do
      AddUseUnit(ASection,NamePos,ImplicitUses[i],nil,nil);
    end;
end;

procedure TPasParser.FinishedModule;
begin
  if Scanner<>nil then
    Scanner.FinishedModule;
  Engine.FinishScope(stModule,CurModule);
end;

// Starts after the "uses" token
procedure TPasParser.ParseUsesList(ASection: TPasSection);
var
  AUnitName, aName: String;
  NameExpr: TPasExpr;
  InFileExpr: TPrimitiveExpr;
  FreeExpr: Boolean;
  NamePos, SrcPos: TPasSourcePos;
  aModule: TPasModule;
begin
  Scanner.SkipGlobalSwitches:=true;
  NameExpr:=nil;
  InFileExpr:=nil;
  FreeExpr:=true;
  try
    Repeat
      FreeExpr:=true;
      AUnitName := ExpectIdentifier;
      NamePos:=CurSourcePos;
      NameExpr:=CreatePrimitiveExpr(ASection,pekString,AUnitName);
      NextToken;
      while CurToken = tkDot do
      begin
        SrcPos:=CurTokenPos;
        ExpectIdentifier;
        aName:=CurTokenString;
        AUnitName := AUnitName + '.' + aName;
        AddToBinaryExprChain(NameExpr,
              CreatePrimitiveExpr(ASection,pekString,aName),eopSubIdent,SrcPos);
        NextToken;
      end;
      if (CurToken=tkin) then
        begin
        if (msDelphi in CurrentModeswitches) then
          begin
          aModule:=ASection.GetModule;
          if (aModule<>nil)
              and ((aModule.ClassType=TPasModule) or (aModule is TPasUnitModule)) then
            CheckToken(tkSemicolon); // delphi does not allow in-filename in units
          end;
        ExpectToken(tkString);
        InFileExpr:=CreatePrimitiveExpr(ASection,pekString,CurTokenString);
        NextToken;
        end;
      FreeExpr:=false;
      AddUseUnit(ASection,NamePos,AUnitName,NameExpr,InFileExpr);
      InFileExpr:=nil;
      NameExpr:=nil;

      if Not (CurToken in [tkComma,tkSemicolon]) then
        ParseExc(nParserExpectedCommaSemicolon,SParserExpectedCommaSemicolon);
    Until (CurToken=tkSemicolon);
  finally
    if FreeExpr then
      begin
      ReleaseAndNil(TPasElement(NameExpr){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
      ReleaseAndNil(TPasElement(InFileExpr){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
      end;
  end;
end;

// Starts after the variable name
function TPasParser.ParseConstDecl(Parent: TPasElement): TPasConst;
var
  OldForceCaret,ok: Boolean;
begin
  SaveComments;
  Result := TPasConst(CreateElement(TPasConst, CurTokenString, Parent));
  if Parent is TPasMembersType then
    Include(Result.VarModifiers,vmClass);
  ok:=false;
  try
    NextToken;
    if CurToken = tkColon then
      begin
      if not (bsWriteableConst in Scanner.CurrentBoolSwitches) then
        Result.IsConst:=true;
      OldForceCaret:=Scanner.SetForceCaret(True);
      try
        Result.VarType := ParseType(Result,CurSourcePos);
        {$IFDEF CheckPasTreeRefCount}if Result.VarType.RefIds.IndexOf('CreateElement')>=0 then Result.VarType.ChangeRefId('CreateElement','TPasVariable.VarType'){$ENDIF};
      finally
        Scanner.SetForceCaret(OldForceCaret);
      end;
      end
    else
      begin
      UngetToken;
      Result.IsConst:=true;
      end;
    NextToken;
    if CurToken=tkEqual then
      begin
      NextToken;
      Result.Expr:=DoParseConstValueExpression(Result);
      if (Result.VarType=Nil) and (Result.Expr.Kind=pekRange) then
        ParseExc(nParserNoConstRangeAllowed,SParserNoConstRangeAllowed);
      end
    else if (Result.VarType<>nil)
        and (po_ExtConstWithoutExpr in Options) then
      begin
      if (Parent is TPasClassType)
          and TPasClassType(Parent).IsExternal
          and (TPasClassType(Parent).ObjKind=okClass) then
        // typed const without expression is allowed in external class
        Result.IsConst:=true
      else if CurToken=tkSemicolon then
        begin
        NextToken;
        if CurTokenIsIdentifier('external') then
          begin
          // typed external const without expression is allowed
          Result.IsConst:=true;
          Include(Result.VarModifiers,vmExternal);
          NextToken;
          if CurToken in [tkString,tkIdentifier] then
            begin
            // external LibraryName;
            // external LibraryName name ExportName;
            // external name ExportName;
            if not CurTokenIsIdentifier('name') then
              Result.LibraryName:=DoParseExpression(Result);
            if not CurTokenIsIdentifier('name') then
              ParseExcSyntaxError;
            NextToken;
            if not (CurToken in [tkChar,tkString,tkIdentifier]) then
              ParseExcTokenError(TokenInfos[tkString]);
            Result.ExportName:=DoParseExpression(Result);
            Result.IsConst:=true; // external const is readonly
            end
          else if CurToken=tkSemicolon then
            // external;
          else
            ParseExcSyntaxError;
          end
        else
          begin
          UngetToken;
          CheckToken(tkEqual);
          end;
        end
      else
        CheckToken(tkEqual);
      end
    else
      CheckToken(tkEqual);
    UngetToken;
    CheckHint(Result,not (Parent is TPasMembersType));
    ok:=true;
  finally
    if not ok then
      ReleaseAndNil(TPasElement(Result){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
  end;
end;

// Starts after the variable name
function TPasParser.ParseResourcestringDecl(Parent: TPasElement): TPasResString;
var
  ok: Boolean;
begin
  SaveComments;
  Result := TPasResString(CreateElement(TPasResString, CurTokenString, Parent));
  ok:=false;
  try
    ExpectToken(tkEqual);
    NextToken; // skip tkEqual
    Result.Expr:=DoParseConstValueExpression(Result);
    UngetToken;
    CheckHint(Result,True);
    ok:=true;
  finally
    if not ok then
      ReleaseAndNil(TPasElement(Result){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
  end;
end;

function TPasParser.ParseAttributes(Parent: TPasElement; Add: boolean
  ): TPasAttributes;
// returns with CurToken at tkSquaredBraceClose
var
  Expr, Arg: TPasExpr;
  Attributes: TPasAttributes;
  Params: TParamsExpr;
  Decls: TPasDeclarations;
begin
  Result:=nil;
  Attributes:=TPasAttributes(CreateElement(TPasAttributes,'',Parent));
  try
    repeat
      NextToken;
      // [name,name(param,param,...),...]
      Expr:=nil;
      ReadDottedIdentifier(Attributes,Expr,false);
      if CurToken=tkBraceOpen then
        begin
        Params:=TParamsExpr(CreateElement(TParamsExpr,'',Attributes));
        Params.Kind:=pekFuncParams;
        Attributes.AddCall(Params);
        Params.Value:=Expr;
        Expr.Parent:=Params;
        Expr:=nil;
        repeat
          NextToken;
          if CurToken=tkBraceClose then
            break;
          Arg:=DoParseConstValueExpression(Params);
          Params.AddParam(Arg);
        until CurToken<>tkComma;
        CheckToken(tkBraceClose);
        NextToken;
        end
      else
        begin
        Attributes.AddCall(Expr);
        Expr:=nil;
        end;
    until CurToken<>tkComma;
    CheckToken(tkSquaredBraceClose);
    Result:=Attributes;
    if Add then
      begin
      if Parent is TPasDeclarations then
        begin
        Decls:=TPasDeclarations(Parent);
        Decls.Declarations.Add(Result);
        Decls.Attributes.Add(Result);
        end
      else if Parent is TPasMembersType then
        TPasMembersType(Parent).Members.Add(Result)
      else
        ParseExcTokenError('[20190922193803]');
      Engine.FinishScope(stDeclaration,Result);
      end;
  finally
    if Result=nil then
      begin
      Attributes.Free;
      Expr.Free;
      end;
  end;
end;

{$warn 5043 off}
procedure TPasParser.ReadGenericArguments(List: TFPList; Parent: TPasElement);
Var
  N : String;
  T : TPasGenericTemplateType;
  Expr: TPasExpr;
  TypeEl: TPasType;
begin
  ExpectToken(tkLessThan);
  repeat
    N:=ExpectIdentifier;
    T:=TPasGenericTemplateType(CreateElement(TPasGenericTemplateType,N,Parent));
    List.Add(T);
    NextToken;
    if Curtoken = tkColon then
      repeat
        NextToken;
        // comma separated list of constraints: identifier, class, record, constructor
        case CurToken of
        tkclass,tkrecord,tkconstructor:
          begin
          if T.TypeConstraint='' then
            T.TypeConstraint:=CurTokenString;
          Expr:=CreatePrimitiveExpr(T,pekIdent,CurTokenText);
          T.AddConstraint(Expr);
          NextToken;
          end;
        tkIdentifier,tkspecialize:
          begin
          TypeEl:=ParseTypeReference(T,false,Expr);
          if T.TypeConstraint='' then
            T.TypeConstraint:=TypeEl.Name;
          if (Expr<>nil) and (Expr.Parent=T) then
            Expr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
          T.AddConstraint(TypeEl);
          end;
        else
          CheckToken(tkIdentifier);
        end;
      until CurToken<>tkComma;
    Engine.FinishScope(stTypeDef,T);
  until not (CurToken in [tkSemicolon,tkComma]);
  if Not (CurToken in [tkGreaterThan,tkGreaterEqualThan]) then
    ParseExcExpectedAorB(TokenInfos[tkComma], TokenInfos[tkGreaterThan])
  else if CurToken=tkGreaterEqualThan then
    begin
    ChangeToken(tkGreaterThan);
    end;
end;
{$warn 5043 on}

procedure TPasParser.ReadSpecializeArguments(Parent: TPasElement;
  Params: TFPList);
// after parsing CurToken is on tkGreaterThan
Var
  TypeEl: TPasType;
begin
  //writeln('START TPasParser.ReadSpecializeArguments ',CurTokenText,' ',CurTokenString);
  CheckToken(tkLessThan);
  repeat
    //writeln('ARG TPasParser.ReadSpecializeArguments ',CurTokenText,' ',CurTokenString);
    TypeEl:=ParseType(Parent,CurTokenPos,'');
    Params.Add(TypeEl);
    NextToken;
    if CurToken=tkComma then
      continue
    else if CurToken=tkshr then
      begin
      ChangeToken(tkGreaterThan);
      break;
      end
    else if CurToken=tkGreaterThan then
      break
    else
      ParseExcExpectedAorB(TokenInfos[tkComma], TokenInfos[tkGreaterThan]);
  until false;
end;

function TPasParser.ReadDottedIdentifier(Parent: TPasElement; out
  Expr: TPasExpr; NeedAsString: boolean): String;
var
  SrcPos: TPasSourcePos;
begin
  Expr:=nil;
  if NeedAsString then
    Result := CurTokenString
  else
    Result:='';
  CheckToken(tkIdentifier);
  Expr:=CreatePrimitiveExpr(Parent,pekIdent,CurTokenString);
  NextToken;
  while CurToken=tkDot do
    begin
    SrcPos:=CurTokenPos;
    ExpectIdentifier;
    if NeedAsString then
      Result := Result+'.'+CurTokenString;
    AddToBinaryExprChain(Expr,CreatePrimitiveExpr(Parent,pekIdent,CurTokenString),
                         eopSubIdent,SrcPos);
    NextToken;
    end;
end;

// Starts after the type name
function TPasParser.ParseRangeType(AParent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String; Full: Boolean
  ): TPasRangeType;

Var
  PE : TPasExpr;
  ok: Boolean;

begin
  Result := TPasRangeType(CreateElement(TPasRangeType, TypeName, AParent, NamePos));
  ok:=false;
  try
    if Full then
      begin
      If not (CurToken=tkEqual) then
        ParseExcTokenError(TokenInfos[tkEqual]);
      end;
    NextToken;
    PE:=DoParseExpression(Result,Nil,False);
    if not ((PE is TBinaryExpr) and (TBinaryExpr(PE).Kind=pekRange)) then
      begin
      PE.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      ParseExc(nRangeExpressionExpected,SRangeExpressionExpected);
      end;
    Result.RangeExpr:=TBinaryExpr(PE);
    UngetToken;
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

// Starts after Exports, on first identifier.
procedure TPasParser.ParseExportDecl(Parent: TPasElement; List: TFPList);
Var
  E : TPasExportSymbol;
  aName: String;
  NameExpr: TPasExpr;
begin
  try
    Repeat
      if List.Count>0 then
        ExpectIdentifier;
      aName:=ReadDottedIdentifier(Parent,NameExpr,true);
      E:=TPasExportSymbol(CreateElement(TPasExportSymbol,aName,Parent));
      if NameExpr.Kind=pekIdent then
        // simple identifier -> no need to store NameExpr
        NameExpr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF}
      else
        begin
        E.NameExpr:=NameExpr;
        NameExpr.Parent:=E;
        end;
      NameExpr:=nil;
      List.Add(E);
      if CurTokenIsIdentifier('INDEX') then
        begin
        NextToken;
        E.Exportindex:=DoParseExpression(E,Nil)
        end
      else if CurTokenIsIdentifier('NAME') then
        begin
        NextToken;
        E.ExportName:=DoParseExpression(E,Nil)
        end;
      if not (CurToken in [tkComma,tkSemicolon]) then
        ParseExc(nParserExpectedCommaSemicolon,SParserExpectedCommaSemicolon);
      Engine.FinishScope(stDeclaration,E);
    until (CurToken=tkSemicolon);
  finally
    if NameExpr<>nil then
      NameExpr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF}
  end;
end;

function TPasParser.ParseProcedureType(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: String; const PT: TProcType
  ): TPasProcedureType;

var
  ok: Boolean;
begin
  if PT in [ptFunction,ptClassFunction] then
    Result := CreateFunctionType(TypeName, 'Result', Parent, False, NamePos)
  else
    Result := TPasProcedureType(CreateElement(TPasProcedureType, TypeName, Parent, NamePos));
  ok:=false;
  try
    ParseProcedureOrFunction(Result, TPasProcedureType(Result), PT, True);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseTypeDecl(Parent: TPasElement): TPasType;
var
  TypeName: String;
  NamePos: TPasSourcePos;
  OldForceCaret , IsDelphiGenericType: Boolean;
begin
  OldForceCaret:=Scanner.SetForceCaret(True);
  try
    IsDelphiGenericType:=false;
    if (msDelphi in CurrentModeswitches) then
      begin
      NextToken;
      IsDelphiGenericType:=CurToken=tkLessThan;
      UngetToken;
      end;
    if IsDelphiGenericType then
      Result:=ParseGenericTypeDecl(Parent,false)
    else
      begin
      TypeName := CurTokenString;
      NamePos:=CurSourcePos;
      ExpectToken(tkEqual);
      Result:=ParseType(Parent,NamePos,TypeName,True);
      end;
  finally
    Scanner.SetForceCaret(OldForceCaret);
  end;
end;

function TPasParser.ParseGenericTypeDecl(Parent: TPasElement;
  AddToParent: boolean): TPasGenericType;

  procedure InitGenericType(NewEl: TPasGenericType; GenericTemplateTypes: TFPList);
  begin
    ParseGenericTypeDecl:=NewEl;
    if AddToParent then
      begin
      if Parent is TPasDeclarations then
        begin
        TPasDeclarations(Parent).Declarations.Add(NewEl);
        {$IFDEF CheckPasTreeRefCount}NewEl.ChangeRefId('CreateElement','TPasDeclarations.Children');{$ENDIF}
        end
      else if Parent is TPasMembersType then
        begin
        TPasMembersType(Parent).Members.Add(NewEl);
        {$IFDEF CheckPasTreeRefCount}NewEl.ChangeRefId('CreateElement','TPasMembersType.Members');{$ENDIF}
        end;
      end;
    if GenericTemplateTypes.Count>0 then
      begin
      // Note: TPasResolver sets GenericTemplateTypes already in CreateElement
      //       This is for other tools like fpdoc.
      NewEl.SetGenericTemplates(GenericTemplateTypes);
      end;
  end;

  procedure ParseProcType(const TypeName: string;
    const NamePos: TPasSourcePos; TypeParams: TFPList;
    IsReferenceTo: boolean);
  var
    ProcTypeEl: TPasProcedureType;
    ProcType: TProcType;
  begin
    ProcTypeEl:=Nil;
    ProcType:=ptProcedure;
    case CurToken of
    tkFunction:
      begin
      ProcTypeEl := CreateFunctionType(TypeName, 'Result', Parent, False,
                                       NamePos, TypeParams);
      ProcType:=ptFunction;
      end;
    tkprocedure:
      begin
      ProcTypeEl := TPasProcedureType(CreateElement(TPasProcedureType,
                          TypeName, Parent, visDefault, NamePos, TypeParams));
      ProcType:=ptProcedure;
      end;
    else
      ParseExcTokenError('procedure or function');
    end;
    ProcTypeEl.IsReferenceTo:=IsReferenceTo;
    if AddToParent and (Parent is TPasDeclarations) then
      TPasDeclarations(Parent).Functions.Add(ProcTypeEl);
    InitGenericType(ProcTypeEl,TypeParams);
    ParseProcedureOrFunction(ProcTypeEl, ProcTypeEl, ProcType, True);
  end;

var
  TypeName, AExternalNameSpace, AExternalName: String;
  NamePos: TPasSourcePos;
  TypeParams: TFPList;
  ClassEl: TPasClassType;
  RecordEl: TPasRecordType;
  ArrEl: TPasArrayType;
  i: Integer;
  AObjKind: TPasObjKind;
  ok: Boolean;
  GenTempl: TPasGenericTemplateType;
begin
  Result:=nil;
  ok := false;
  TypeName := CurTokenString;
  NamePos := CurSourcePos;
  TypeParams:=TFPList.Create;
  try
    ReadGenericArguments(TypeParams,Parent);
    ExpectToken(tkEqual);
    NextToken;
    Case CurToken of
      tkObject,
      tkClass,
      tkinterface:
        begin
        case CurToken of
        tkobject: AObjKind:=okObject;
        tkinterface: AObjKind:=okInterface;
        else AObjKind:=okClass;
        end;
        NextToken;
        if (AObjKind = okClass) and (CurToken = tkOf) then
          ParseExcExpectedIdentifier;
        DoParseClassExternalHeader(AObjKind,AExternalNameSpace,AExternalName);
        ClassEl := TPasClassType(CreateElement(TPasClassType,
          TypeName, Parent, visDefault, NamePos, TypeParams));
        ClassEl.ObjKind:=AObjKind;
        if AObjKind=okInterface then
          if SameText(Scanner.CurrentValueSwitch[vsInterfaces],'CORBA') then
            ClassEl.InterfaceType:=citCorba;
        if AddToParent and (Parent is TPasDeclarations) then
          TPasDeclarations(Parent).Classes.Add(ClassEl);
        ClassEl.IsExternal:=(AExternalName<>'');
        if AExternalName<>'' then
          ClassEl.ExternalName:={$ifdef pas2js}DeQuoteString{$else}AnsiDequotedStr{$endif}(AExternalName,'''');
        if AExternalNameSpace<>'' then
          ClassEl.ExternalNameSpace:={$ifdef pas2js}DeQuoteString{$else}AnsiDequotedStr{$endif}(AExternalNameSpace,'''');
        InitGenericType(ClassEl,TypeParams);
        DoParseClassType(ClassEl);
        CheckHint(ClassEl,True);
        Engine.FinishScope(stTypeDef,ClassEl);
        end;
     tkRecord:
       begin
       RecordEl := TPasRecordType(CreateElement(TPasRecordType,
         TypeName, Parent, visDefault, NamePos, TypeParams));
       if AddToParent and (Parent is TPasDeclarations) then
         TPasDeclarations(Parent).Classes.Add(RecordEl);
       InitGenericType(RecordEl,TypeParams);
       NextToken;
       ParseRecordMembers(RecordEl,tkend,
                        (msAdvancedRecords in Scanner.CurrentModeSwitches)
                        and not (Parent is TProcedureBody)
                        and (RecordEl.Name<>''));
       CheckHint(RecordEl,True);
       Engine.FinishScope(stTypeDef,RecordEl);
       end;
     tkArray:
       begin
       ArrEl := TPasArrayType(CreateElement(TPasArrayType,
         TypeName, Parent, visDefault, NamePos, TypeParams));
       if AddToParent and (Parent is TPasDeclarations) then
         TPasDeclarations(Parent).Types.Add(ArrEl);
       InitGenericType(ArrEl,TypeParams);
       DoParseArrayType(ArrEl);
       CheckHint(ArrEl,True);
       Engine.FinishScope(stTypeDef,ArrEl);
       end;
    tkprocedure,tkfunction:
      ParseProcType(TypeName,NamePos,TypeParams,false);
    tkIdentifier:
      if CurTokenIsIdentifier('reference') then
        begin
        NextToken;
        CheckToken(tkto);
        NextToken;
        ParseProcType(TypeName,NamePos,TypeParams,true);
        end
      else
        ParseExcTypeParamsNotAllowed;
    else
      ParseExcTypeParamsNotAllowed;
    end;
    ok:=true;
  finally
    if (not ok) and (Result<>nil) and not AddToParent then
      Result.Release({$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF});
    for i:=0 to TypeParams.Count-1 do
      begin
      GenTempl:=TPasGenericTemplateType(TypeParams[i]);
      GenTempl.Parent:=nil;
      GenTempl.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
    TypeParams.Free;
  end;
end;

function TPasParser.GetVariableValueAndLocation(Parent: TPasElement; out
  Value: TPasExpr; out AbsoluteExpr: TPasExpr; out Location: String): Boolean;

begin
  Value:=Nil;
  AbsoluteExpr:=Nil;
  Location:='';
  NextToken;
  Result:=CurToken=tkEqual;
  if Result then
    begin
    NextToken;
    Value := DoParseConstValueExpression(Parent);
    end;
  if (CurToken=tkAbsolute) then
    begin
    Result:=True;
    NextToken;
    if Curtoken=tkNumber then
      begin
      AbsoluteExpr:=CreatePrimitiveExpr(Parent,pekNumber,CurTokenString);
      Location:=CurTokenString
      end
    else
      begin
      Location:=ReadDottedIdentifier(Parent,AbsoluteExpr,true);
      UnGetToken;
      end
    end
  else
    UngetToken;
end;

function TPasParser.GetVariableModifiers(Parent: TPasElement; out
  VarMods: TVariableModifiers; out LibName, ExportName: TPasExpr;
  const AllowedMods: TVariableModifiers): string;

Var
  S : String;
  ExtMod: TVariableModifier;
begin
  Result := '';
  LibName := nil;
  ExportName := nil;
  VarMods := [];
  NextToken;
  If (vmCVar in AllowedMods) and CurTokenIsIdentifier('cvar') then
    begin
    Result:=';cvar';
    Include(VarMods,vmcvar);
    ExpectToken(tkSemicolon);
    NextToken;
    end;
  s:=LowerCase(CurTokenText);
  if (vmExternal in AllowedMods) and (s='external') then
    ExtMod:=vmExternal
  else if (vmPublic in AllowedMods) and (s='public') then
    ExtMod:=vmPublic
  else if (vmExport in AllowedMods) and (s='export') then
    ExtMod:=vmExport
  else
    begin
    UngetToken;
    exit;
    end;
  Include(VarMods,ExtMod);
  Result:=Result+';'+CurTokenText;

  NextToken;
  if not (CurToken in [tkString,tkIdentifier]) then
    begin
    if (CurToken=tkSemicolon) and (ExtMod in [vmExternal,vmPublic]) then
      exit;
    ParseExcSyntaxError;
    end;
  // export name exportname;
  // public;
  // public name exportname;
  // external;
  // external libname;
  // external libname name exportname;
  // external name exportname;
  if (ExtMod=vmExternal) and (CurToken in [tkString,tkIdentifier])
      and Not (CurTokenIsIdentifier('name')) then
    begin
    Result := Result + ' ' + CurTokenText;
    LibName:=DoParseExpression(Parent);
    end;
  if CurToken=tkSemiColon then
    exit;
  if not CurTokenIsIdentifier('name') then
    ParseExcSyntaxError;
  NextToken;
  if not (CurToken in [tkChar,tkString,tkIdentifier]) then
    ParseExcTokenError(TokenInfos[tkString]);
  Result := Result + ' ' + CurTokenText;
  ExportName:=DoParseExpression(Parent);
end;


// Full means that a full variable declaration is being parsed.
procedure TPasParser.ParseVarList(Parent: TPasElement; VarList: TFPList;
  AVisibility: TPasMemberVisibility; Full : Boolean);
// on Exception the VarList is restored, no need to Release the new elements

var
  i, OldListCount: Integer;
  Value , aLibName, aExpName, AbsoluteExpr: TPasExpr;
  VarType: TPasType;
  VarEl: TPasVariable;
  H : TPasMemberHints;
  VarMods, AllowedVarMods: TVariableModifiers;
  D,Mods,AbsoluteLocString: string;
  OldForceCaret,ok,ExternalStruct: Boolean;

begin
  Value:=Nil;
  aLibName:=nil;
  aExpName:=nil;
  AbsoluteExpr:=nil;
  AbsoluteLocString:='';
  OldListCount:=VarList.Count;
  ok:=false;
  try
    D:=SaveComments; // This means we support only one comment per 'list'.
    VarEl:=nil;
    while CurToken=tkSquaredBraceOpen do
      begin
      if msPrefixedAttributes in CurrentModeswitches then
        begin
        VarList.Add(ParseAttributes(Parent,false));
        NextToken;
        end
      else
        CheckToken(tkIdentifier);
      end;
    Repeat
      // create the TPasVariable here, so that SourceLineNumber is correct
      VarEl:=TPasVariable(CreateElement(TPasVariable,CurTokenString,Parent,
                                        AVisibility,CurTokenPos));
      VarList.Add(VarEl);
      NextToken;
      case CurToken of
      tkColon: break;
      tkComma: ExpectIdentifier;
      else
        ParseExc(nParserExpectedCommaColon,SParserExpectedCommaColon);
      end;
    Until (CurToken=tkColon);
    OldForceCaret:=Scanner.SetForceCaret(True);
    try
      VarType := ParseVarType(VarEl);
      {$IFDEF CheckPasTreeRefCount}if VarType.RefIds.IndexOf('CreateElement')>=0 then VarType.ChangeRefId('CreateElement','TPasVariable.VarType'){$ENDIF};
    finally
      Scanner.SetForceCaret(OldForceCaret);
    end;
    // read type
    for i := OldListCount to VarList.Count - 1 do
      begin
      VarEl:=TPasVariable(VarList[i]);
      // Writeln(VarEl.Name, AVisibility);
      VarEl.VarType := VarType;
      //VarType.Parent := VarEl; // this is wrong for references
      if (i>OldListCount) then
        VarType.AddRef{$IFDEF CheckPasTreeRefCount}('TPasVariable.VarType'){$ENDIF};
      end;

    H:=CheckHint(Nil,False);
    If Full then
      GetVariableValueAndLocation(VarEl,Value,AbsoluteExpr,AbsoluteLocString);
    if (VarList.Count>OldListCount+1) then
      begin
      // multiple variables
      if Value<>nil then
        ParseExc(nParserOnlyOneVariableCanBeInitialized,SParserOnlyOneVariableCanBeInitialized);
      if AbsoluteExpr<>nil then
        ParseExc(nParserOnlyOneVariableCanBeAbsolute,SParserOnlyOneVariableCanBeAbsolute);
      end;
    TPasVariable(VarList[OldListCount]).Expr:=Value;
    Value:=nil;

    // Note: external members are allowed for non external classes/records too
    ExternalStruct:=(msExternalClass in CurrentModeSwitches)
                    and (Parent is TPasMembersType);

    H:=H+CheckHint(Nil,False);
    if Full or ExternalStruct then
      begin
      NextToken;
      If Curtoken<>tkSemicolon then
        UnGetToken;
      VarEl:=TPasVariable(VarList[OldListCount]);
      AllowedVarMods:=[];
      if ExternalStruct then
        AllowedVarMods:=[vmExternal]
      else
        AllowedVarMods:=[vmCVar,vmExternal,vmPublic,vmExport];
      Mods:=GetVariableModifiers(VarEl,VarMods,aLibName,aExpName,AllowedVarMods);
      if (Mods='') and (CurToken<>tkSemicolon) then
        NextToken;
      end
    else
      begin
      NextToken;
      VarMods:=[];
      Mods:='';
      end;
    SaveComments(D);

    // connect
    for i := OldListCount to VarList.Count - 1 do
      begin
      VarEl:=TPasVariable(VarList[i]);
      // Writeln(VarEl.Name, AVisibility);
      // Procedure declaration eats the hints.
      if Assigned(VarType) and (VarType is TPasProcedureType) then
        VarEl.Hints:=VarType.Hints
      else
        VarEl.Hints:=H;
      VarEl.Modifiers:=Mods;
      VarEl.VarModifiers:=VarMods;
      VarEl.{%H-}AbsoluteLocation:=AbsoluteLocString;
      if AbsoluteExpr<>nil then
        begin
        VarEl.AbsoluteExpr:=AbsoluteExpr;
        AbsoluteExpr:=nil;
        end;
      if aLibName<>nil then
        begin
        VarEl.LibraryName:=aLibName;
        aLibName:=nil;
        end;
      if aExpName<>nil then
        begin
        VarEl.ExportName:=aExpName;
        aExpName:=nil;
        end;
      end;
    ok:=true;
  finally
    if not ok then
      begin
      if aLibName<>nil then aLibName.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if aExpName<>nil then aExpName.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if AbsoluteExpr<>nil then AbsoluteExpr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      if Value<>nil then Value.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      for i:=OldListCount to VarList.Count-1 do
        TPasElement(VarList[i]).Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      VarList.Count:=OldListCount;
      end;
  end;
end;

procedure TPasParser.SetOptions(AValue: TPOptions);
begin
  if FOptions=AValue then Exit;
  FOptions:=AValue;
  If Assigned(FScanner) then
    FScanner.Options:=AValue;
end;

procedure TPasParser.OnScannerModeChanged(Sender: TObject;
  NewMode: TModeSwitch; Before: boolean; var Handled: boolean);
begin
  Engine.ModeChanged(Self,NewMode,Before,Handled);
  if Sender=nil then ;
end;

function TPasParser.SaveComments: String;
begin
  if Engine.NeedComments then
    FSavedComments:=CurComments.Text; // Expensive, so don't do unless needed.
  Result:=FSavedComments;
end;

function TPasParser.SaveComments(const AValue: String): String;
begin
  FSavedComments:=AValue;
  Result:=FSavedComments;
end;

function TPasParser.LogEvent(E: TPParserLogEvent): Boolean;
begin
  Result:=E in FLogEvents;
end;

procedure TPasParser.SetLastMsg(MsgType: TMessageType; MsgNumber: integer;
  const Fmt: String; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif});
begin
  FLastMsgType := MsgType;
  FLastMsgNumber := MsgNumber;
  FLastMsgPattern := Fmt;
  FLastMsg := SafeFormat(Fmt,Args);
  CreateMsgArgs(FLastMsgArgs,Args);
end;

procedure TPasParser.DoLog(MsgType: TMessageType; MsgNumber: integer;
  const Msg: String; SkipSourceInfo: Boolean);
begin
  DoLog(MsgType,MsgNumber,Msg,[],SkipSourceInfo);
end;

procedure TPasParser.DoLog(MsgType: TMessageType; MsgNumber: integer;
  const Fmt: String; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
  SkipSourceInfo: Boolean);

Var
  Msg : String;

begin
  if (Scanner<>nil) and Scanner.IgnoreMsgType(MsgType) then
    exit;
  SetLastMsg(MsgType,MsgNumber,Fmt,Args);
  If Assigned(FOnLog) then
    begin
    Msg:=MessageTypeNames[MsgType]+': ';
    if SkipSourceInfo or not assigned(scanner) then
      Msg:=Msg+FLastMsg
    else
      Msg:=Msg+Format('%s(%d,%d) : %s',[Scanner.CurFilename,Scanner.CurRow,Scanner.CurColumn,FLastMsg]);
    FOnLog(Self,Msg);
    end;
end;

procedure TPasParser.ParseInlineVarDecl(Parent: TPasElement; List: TFPList;
  AVisibility: TPasMemberVisibility = VisDefault; ClosingBrace: Boolean = False);

Var
  tt : TTokens;
begin
  ParseVarList(Parent,List,AVisibility,False);
  tt:=[tkEnd,tkSemicolon];
  if ClosingBrace then
    Include(tt,tkBraceClose);
  if not (CurToken in tt) then
    ParseExc(nParserExpectedSemiColonEnd,SParserExpectedSemiColonEnd);
end;

// Starts after the variable name
procedure TPasParser.ParseVarDecl(Parent: TPasElement; List: TFPList);

begin
  ParseVarList(Parent,List,visDefault,True);
end;

// Starts after the opening bracket token
procedure TPasParser.ParseArgList(Parent: TPasElement; Args: TFPList; EndToken: TToken);

  Function GetParamName : string;

  begin
    if ([msDelphi,msDelphiUnicode,msObjfpc]* CurrentModeswitches)<>[] then
      Result := ExpectIdentifier
    else
      begin
      NextToken;
      if CurToken in [tkProperty,tkIdentifier,tkClass] then
        Result:=CurTokenString
      else
        ParseExcTokenError('identifier')
      end;
  end;
var
  OldForceCaret,IsUntyped, ok, LastHadDefaultValue: Boolean;
  Name : String;
  Value : TPasExpr;
  i, OldArgCount: Integer;
  Arg: TPasArgument;
  Access: TArgumentAccess;
  ArgType: TPasType;
begin
  LastHadDefaultValue := false;
  while True do
  begin
    OldArgCount:=Args.Count;
    Access := argDefault;
    IsUntyped := False;
    ArgType := nil;
    NextToken;
    if CurToken = tkDotDotDot then
    begin
      expectToken(endToken);
      Break;
    end else  if CurToken = tkConst then
    begin
      Access := argConst;
      Name := GetParamName;
    end else if CurToken = tkConstRef then
    begin
      Access := argConstref;
      Name := getParamName;
    end else if CurToken = tkVar then
    begin
      Access := ArgVar;
      Name:=GetParamName;
    end else if (CurToken = tkIdentifier) and (UpperCase(CurTokenString) = 'OUT') then
    begin
      if  ([msObjfpc, msDelphi, msDelphiUnicode, msOut] * CurrentModeswitches)<>[] then
        begin
        Access := ArgOut;
        Name := ExpectIdentifier
        end
      else
        Name := CurTokenString
    end else if (CurToken = tkproperty) or (CurToken=tkClass) then
      begin
      if ([msDelphi,msDelphiUnicode,msObjfpc]* CurrentModeswitches)<>[] then
        ParseExcTokenError('identifier')
      else
        Name := CurTokenString
    end else if CurToken = tkIdentifier then
      Name := CurTokenString
    else
      ParseExc(nParserExpectedConstVarID,SParserExpectedConstVarID);
    while True do
    begin
      Arg := TPasArgument(CreateElement(TPasArgument, Name, Parent));
      Arg.Access := Access;
      Args.Add(Arg);
      NextToken;
      if CurToken = tkColon then
        break
      else if ((CurToken = tkSemicolon) or (CurToken = tkBraceClose)) and
        (Access <> argDefault) then
      begin
        // found an untyped const or var argument
        UngetToken;
        IsUntyped := True;
        break
      end
      else if CurToken <> tkComma then
        ParseExc(nParserExpectedCommaColon,SParserExpectedCommaColon);
      NextToken;
      if CurToken = tkIdentifier then
        Name := CurTokenString
      else
        ParseExc(nParserExpectedConstVarID,SParserExpectedConstVarID);
    end;
    Value:=Nil;
    if not IsUntyped then
      begin
      Arg := TPasArgument(Args[OldArgCount]);
      ArgType:=Nil;
      ok:=false;
      oldForceCaret:=Scanner.SetForceCaret(True);
      try
        ArgType := ParseType(Arg,CurSourcePos);
        NextToken;
        if CurToken = tkEqual then
          begin
          if (Args.Count>OldArgCount+1) then
            begin
            ArgType.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
            ArgType:=nil;
            ParseExc(nParserOnlyOneArgumentCanHaveDefault,SParserOnlyOneArgumentCanHaveDefault);
            end;
          if Parent is TPasProperty then
            ParseExc(nParserPropertyArgumentsCanNotHaveDefaultValues,
              SParserPropertyArgumentsCanNotHaveDefaultValues);
          NextToken;
          Value := DoParseExpression(Arg,Nil);
          // After this, we're on ), which must be unget.
          LastHadDefaultValue:=true;
          end
        else if LastHadDefaultValue then
          ParseExc(nParserDefaultParameterRequiredFor,
            SParserDefaultParameterRequiredFor,[TPasArgument(Args[OldArgCount]).Name]);
        UngetToken;
        ok:=true;
      finally
        Scanner.SetForceCaret(oldForceCaret);
        if (not ok) and (ArgType<>nil) then
          ArgType.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
      end;

    for i := OldArgCount to Args.Count - 1 do
    begin
      Arg := TPasArgument(Args[i]);
      Arg.ArgType := ArgType;
      if Assigned(ArgType) then
        begin
        if (i > OldArgCount) then
          ArgType.AddRef{$IFDEF CheckPasTreeRefCount}('TPasArgument.ArgType'){$ENDIF};
        end;
      Arg.ValueExpr := Value;
      Value:=Nil; // Only the first gets a value. OK, since Var A,B : Integer = 1 is not allowed.
    end;

    for i := OldArgCount to Args.Count - 1 do
      Engine.FinishScope(stDeclaration,TPasArgument(Args[i]));

    NextToken;
    if (CurToken = tkIdentifier) and (LowerCase(CurTokenString) = 'location') then
      begin
        NextToken; // remove 'location'
        NextToken; // remove register
      end;
    if CurToken = EndToken then
      break;
    CheckToken(tkSemicolon);
  end;
end;


function TPasParser.CheckProcedureArgs(Parent: TPasElement; Args: TFPList;
  ProcType: TProcType): boolean;

begin
  NextToken;
  if CurToken=tkBraceOpen then
    begin
    Result:=true;
    NextToken;
    if (CurToken<>tkBraceClose) then
      begin
      UngetToken;
      ParseArgList(Parent, Args, tkBraceClose);
      end;
    end
  else
    begin
    Result:=false;
    case ProcType of
    ptOperator,ptClassOperator:
      ParseExc(nParserExpectedLBracketColon,SParserExpectedLBracketColon);
    ptAnonymousProcedure,ptAnonymousFunction:
      case CurToken of
      tkIdentifier, // e.g. procedure assembler
      tkbegin,tkvar,tkconst,tktype,tkprocedure,tkfunction,tkasm:
        UngetToken;
      tkColon:
        if ProcType=ptAnonymousFunction then
          UngetToken
        else
          ParseExcTokenError('begin');
      else
        ParseExcTokenError('begin');
      end;
    else
      case CurToken of
        tkSemicolon, // e.g. procedure;
        tkColon, // e.g. function: id
        tkof, // e.g. procedure of object
        tkis, // e.g. procedure is nested
        tkIdentifier: // e.g. procedure cdecl;
          UngetToken;
      else
        ParseExcTokenError(';');
      end;
    end;
    end;
end;

procedure TPasParser.HandleProcedureModifier(Parent: TPasElement; pm: TProcedureModifier);
// at end on last token of modifier, usually the semicolon
Var
  P : TPasProcedure;
  E : TPasExpr;

  procedure AddModifier;
  begin
    if pm in P.Modifiers then
      ParseExcSyntaxError;
    P.AddModifier(pm);
  end;

begin
  P:=TPasProcedure(Parent);
  if pm<>pmPublic then
    AddModifier;
  Case pm of
  pmExternal:
    begin
    NextToken;
    if CurToken in [tkString,tkIdentifier] then
      begin
      // external libname
      // external libname name XYZ
      // external name XYZ
      if Not CurTokenIsIdentifier('NAME') then
        begin
        E:=DoParseExpression(Parent);
        if Assigned(P) then
          P.LibraryExpr:=E;
        end;
      if CurTokenIsIdentifier('NAME') then
        begin
        NextToken;
        if not (CurToken in [tkChar,tkString,tkIdentifier]) then
          ParseExcTokenError(TokenInfos[tkString]);
        E:=DoParseExpression(Parent);
        if Assigned(P) then
          P.LibrarySymbolName:=E;
        end;
      if CurToken<>tkSemicolon then
        UngetToken;
      end
    else
      UngetToken;
    end;
  pmPublic:
    begin
    NextToken;
    If not CurTokenIsIdentifier('name') then
      begin
      if P.Parent is TPasMembersType then
        begin
        // public section starts
        UngetToken;
        UngetToken;
        exit;
        end;
      AddModifier;
      CheckToken(tkSemicolon);
      exit;
      end
    else
      begin
      AddModifier;
      NextToken;  // Should be "public name string".
      if not (CurToken in [tkString,tkIdentifier]) then
        ParseExcTokenError(TokenInfos[tkString]);
      E:=DoParseExpression(Parent);
      if Parent is TPasProcedure then
        TPasProcedure(Parent).PublicName:=E;
      CheckToken(tkSemicolon);
      end;
    end;
  pmForward:
    begin
    if (Parent.Parent is TInterfaceSection) then
       begin
       ParseExc(nParserForwardNotInterface,SParserForwardNotInterface);
       UngetToken;
       end;
    end;
  pmMessage:
    begin
    NextToken;
    E:=DoParseExpression(Parent);
    TPasProcedure(Parent).MessageExpr:=E;
    if E is TPrimitiveExpr then
      begin
      TPasProcedure(Parent).MessageName:=TPrimitiveExpr(E).Value;
      case E.Kind of
        pekNumber, pekUnary:
          TPasProcedure(Parent).Messagetype:=pmtInteger;
        pekString:
          TPasProcedure(Parent).Messagetype:=pmtString;
        pekIdent : ; // unknown at this time
      else
        ParseExc(nInvalidMessageType,SErrInvalidMessageType);
      end;
      end;
    if CurToken<>tkSemicolon then
      UngetToken;
    end;
  pmDispID:
    begin
    NextToken;
    TPasProcedure(Parent).DispIDExpr:=DoParseExpression(Parent);
    if CurToken<>tkSemicolon then
      UngetToken;
    end;
  else
    // Do nothing, satisfy compiler
  end; // Case
end;

procedure TPasParser.HandleProcedureTypeModifier(ProcType: TPasProcedureType;
  ptm: TProcTypeModifier);
var
  Expr: TPasExpr;
begin
  if ptm in ProcType.Modifiers then
    ParseExcSyntaxError;
  Include(ProcType.Modifiers,ptm);
  if ptm=ptmVarargs then
    begin
    NextToken;
    if CurToken<>tkof then
      begin
      UngetToken;
      exit;
      end;
    NextToken;
    Expr:=nil;
    try
      ProcType.VarArgsType:=ParseTypeReference(ProcType,false,Expr);
    finally
      if Expr<>nil then Expr.Release{$IFDEF CheckPasTreeRefCount}('20191029145019'){$ENDIF};
    end;
    end;
end;

// Next token is expected to be a "(", ";" or for a function ":". The caller
// will get the token after the final ";" as next token.

function TPasParser.DoCheckHint(Element : TPasElement): Boolean;

var
  ahint : TPasMemberHint;

begin
  Result:= IsCurTokenHint(ahint);
  if Result then  // deprecated,platform,experimental,library, unimplemented etc
    begin
    Element.Hints:=Element.Hints+[ahint];
    if aHint=hDeprecated then
      begin
      NextToken;
      if (CurToken<>tkString) then
        UngetToken
      else
        Element.HintMessage:=CurTokenString;
      end;
    end;
end;

procedure TPasParser.ParseProcedureOrFunction(Parent: TPasElement;
  Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);

  Function FindInSection(AName : String;ASection : TPasSection) : Boolean;

  Var
    I : integer;
    Cn,FN : String;
    CT : TPasClassType;

  begin
    I:=ASection.Functions.Count-1;
    While (I>=0) and (CompareText(TPasElement(ASection.Functions[I]).Name,AName)<>0) do
      Dec(I);
    Result:=I<>-1;
    I:=Pos('.',AName);
    if (Not Result) and (I>0) then
      begin
      CN:=Copy(AName,1,I-1);
      FN:=AName;
      Delete(FN,1,I);
      I:=ASection.Classes.Count-1;
      While Not Result and (I>=0) do
        begin
        CT:=TPasClassType(ASection.Classes[i]);
        if CompareText(CT.Name,CN)=0 then
          Result:=CT.FindMember(TPasFunction, FN)<>Nil;
        Dec(I);
        end;
      end;
  end;

  procedure ConsumeSemi;
  begin
    NextToken;
    if (CurToken <> tkSemicolon) and IsCurTokenHint then
      UngetToken;
  end;

Var
  Tok : String;
  CC : TCallingConvention;
  PM : TProcedureModifier;
  ResultEl: TPasResultElement;
  OK: Boolean;
  IsProcType: Boolean; // false = procedure, true = procedure type
  IsAnonymous: Boolean;
  OldForceCaret : Boolean;
  PTM: TProcTypeModifier;
  ModTokenCount: Integer;
  LastToken: TToken;

begin
  // Element must be non-nil. Removed all checks for not-nil.
  // If it is nil, the following fails anyway.
  CheckProcedureArgs(Element,Element.Args,ProcType);
  IsProcType:=not (Parent is TPasProcedure);
  IsAnonymous:=(not IsProcType) and (ProcType in [ptAnonymousProcedure,ptAnonymousFunction]);
  case ProcType of
    ptFunction,ptClassFunction,ptAnonymousFunction:
      begin
      NextToken;
      if CurToken = tkColon then
        begin
        ResultEl:=TPasFunctionType(Element).ResultEl;
        OldForceCaret:=Scanner.SetForceCaret(True);
        try
          ResultEl.ResultType := ParseType(ResultEl,CurSourcePos);
        finally
          Scanner.SetForceCaret(OldForceCaret);
        end;
        end
      // In Delphi mode, the signature in the implementation section can be
      // without result as it was declared
      // We actually check if the function exists in the interface section.
      else if (not IsAnonymous)
          and (msDelphi in CurrentModeswitches)
          and (Assigned(CurModule.ImplementationSection)
            or (CurModule is TPasProgram))
          then
        begin
        OK:=False;
        if Assigned(CurModule.InterfaceSection) then
          OK:=FindInSection(Parent.Name,CurModule.InterfaceSection)
        else if (CurModule is TPasProgram) and Assigned(TPasProgram(CurModule).ProgramSection) then
          OK:=FindInSection(Parent.Name,TPasProgram(CurModule).ProgramSection);
        if Not OK then
          CheckToken(tkColon)
        else
          begin
          CheckToken(tkSemiColon);
          UngetToken;
          end;
        end
      else
        begin
        // Raise error
        CheckToken(tkColon);
        end;
      end;
    ptOperator,ptClassOperator:
      begin
      NextToken;
      ResultEl:=TPasFunctionType(Element).ResultEl;
      if (CurToken=tkIdentifier) then
        begin
        ResultEl.Name := CurTokenName;
        ExpectToken(tkColon);
        ResultEl.ResultType := ParseType(ResultEl,CurSourcePos);
        end
      else if not ((Parent is TPasOperator) and (TPasOperator(Parent).OperatorType=otInitialize)) then
        // Initialize operator has no result
        begin
         if (CurToken=tkColon) then
            ResultEl.Name := 'Result'
          else
            ParseExc(nParserExpectedColonID,SParserExpectedColonID);
         ResultEl.ResultType := ParseType(ResultEl,CurSourcePos);
         end;
      end;
  else
    ResultEl:=Nil;
  end;
  if OfObjectPossible then
    begin
    NextToken;
    if (CurToken = tkOf) then
      begin
      ExpectToken(tkObject);
      Element.IsOfObject := True;
      end
    else if (CurToken = tkIs) then
      begin
      ExpectToken(tkIdentifier);
      if (lowerCase(CurTokenString)<>'nested') then
        ParseExc(nParserExpectedNested,SParserExpectedNested);
      Element.IsNested:=True;
      end
    else
      UnGetToken;
    end;
  ModTokenCount:=0;
  //writeln('TPasParser.ParseProcedureOrFunction IsProcType=',IsProcType,' IsAnonymous=',IsAnonymous);
  Repeat
    inc(ModTokenCount);
    //writeln('TPasParser.ParseProcedureOrFunction ',ModTokenCount,' ',CurToken,' ',CurTokenText);
    LastToken:=CurToken;
    NextToken;
    if (CurToken = tkEqual) and IsProcType and (ModTokenCount<=3) then
      begin
      // for example: const p: procedure = nil;
      UngetToken;
      Engine.FinishScope(stProcedureHeader,Element);
      exit;
      end;
    If CurToken=tkSemicolon then
      begin
      if IsAnonymous then
        CheckToken(tkbegin); // begin expected, but ; found
      // if LastToken=tkSemicolon then
      //  ParseExcSyntaxError;
      continue;
      end
    else if TokenIsCallingConvention(CurTokenString,cc) then
      begin
      Element.CallingConvention:=Cc;
      if cc = ccSysCall then
      begin
        // remove LibBase
        NextToken;
        if CurToken=tkSemiColon then
          UngetToken
        else
          // remove legacy or basesysv on MorphOS syscalls
          begin
          if (Pos('sysv',LowerCase(CurtokenText))>0) or CurTokenIsIdentifier('legacy') then
            NextToken; 
          // remove LibBase (Amiga, AROS, MorphOS)  or Interface (OS4)
          if CurTokenIsIdentifier('consoledevice') or
              ((Curtoken=tkIdentifier) and (Pos('base',LowerCase(CurtokenText)) > 0)) or 
              ((Curtoken=tkIdentifier) and (CurtokenText[1] = 'I')) then
            NextToken;
          end;
      end;
      if IsProcType then
        begin
        ExpectTokens([tkSemicolon,tkEqual]);
        if CurToken=tkEqual then
          UngetToken;
        end
      else if IsAnonymous then
      else
        ExpectTokens([tkSemicolon]);
      end
    else if IsAnonymous and TokenIsAnonymousProcedureModifier(Parent,CurTokenString,PM) then
      HandleProcedureModifier(Parent,PM)
    else if TokenIsProcedureTypeModifier(Parent,CurTokenString,PTM) then
      begin
      HandleProcedureTypeModifier(Element,PTM);
      // Backwards compatibility
      if (PTM=ptmFar) and (Parent is TPasProcedure) then
        (Parent as TPasProcedure).AddModifier(pmFar)
      end
    else if (not IsProcType) and (not IsAnonymous)
        and TokenIsProcedureModifier(Parent,CurTokenString,PM) then
      HandleProcedureModifier(Parent,PM)
    else if (CurToken=tklibrary) and not IsProcType and not IsAnonymous then
      // library is a token and a directive.
      begin
      Tok:=UpperCase(CurTokenString);
      NextToken;
      If (tok<>'NAME') then
        begin
        if hLibrary in Element.Hints then
          ParseExcSyntaxError;
        Element.Hints:=Element.Hints+[hLibrary];
        end
      else
        begin
        NextToken;  // Should be "export name astring".
        ExpectToken(tkSemicolon);
        end;
      end
    else if (not IsAnonymous) and DoCheckHint(Element) then
      // deprecated,platform,experimental,library, unimplemented etc
      ConsumeSemi
    else if (CurToken=tkIdentifier) and (not IsAnonymous)
        and (CompareText(CurTokenText,'alias')=0) then
      begin
      ExpectToken(tkColon);
      ExpectToken(tkString);
      if (Parent is TPasProcedure) then
        (Parent as TPasProcedure).AliasName:=CurTokenText;
      ExpectToken(tkSemicolon);
      end
    else if (CurToken = tkSquaredBraceOpen) then
      begin
      if msPrefixedAttributes in CurrentModeswitches then
        begin
        // [attribute]
        UngetToken;
        break;
        end
      else
        begin
        // ToDo: read FPC's [] modifiers, e.g. [public,alias:'']
        repeat
          NextToken;
          if CurToken in [tkSquaredBraceOpen,tkSemicolon] then
            CheckToken(tkSquaredBraceClose);
        until CurToken = tkSquaredBraceClose;
        ExpectToken(tkSemicolon);
        end;
      end
    else
      begin
      // not a modifier/hint/calling convention
      if LastToken=tkSemicolon then
        begin
        UngetToken;
        if IsAnonymous then
          ParseExcSyntaxError;
        break;
        end
      else if IsAnonymous then
        begin
        UngetToken;
        break;
        end
      else
        begin
        CheckToken(tkSemicolon);
        continue;
        end;
      end;
    // Writeln('Done: ',TokenInfos[Curtoken],' ',CurtokenString);
  Until false;
  if (ProcType in [ptOperator,ptClassOperator]) and (Parent is TPasOperator) then
    TPasOperator(Parent).CorrectName;
  Engine.FinishScope(stProcedureHeader,Element);
  if (not IsProcType)
  and (not TPasProcedure(Parent).IsForward)
  and (not TPasProcedure(Parent).IsExternal)
  and ((Parent.Parent is TImplementationSection)
     or (Parent.Parent is TProcedureBody)
     or IsAnonymous)
  then
    ParseProcedureBody(Parent);
end;

// starts after the semicolon
procedure TPasParser.ParseProcedureBody(Parent: TPasElement);

var
  Body: TProcedureBody;

begin
  Body := TProcedureBody(CreateElement(TProcedureBody, '', Parent));
  TPasProcedure(Parent).Body:=Body;
  ParseDeclarations(Body);
end;

function TPasParser.ParseMethodResolution(Parent: TPasElement
  ): TPasMethodResolution;
var
  ok: Boolean;
begin
  ok:=false;
  Result:=TPasMethodResolution(CreateElement(TPasMethodResolution,'',Parent));
  try
    if CurToken=tkfunction then
      Result.ProcClass:=TPasFunction
    else
      Result.ProcClass:=TPasProcedure;
    ExpectToken(tkIdentifier);
    Result.InterfaceName:=CreatePrimitiveExpr(Result,pekIdent,CurTokenString);
    ExpectToken(tkDot);
    ExpectToken(tkIdentifier);
    Result.InterfaceProc:=CreatePrimitiveExpr(Result,pekIdent,CurTokenString);
    ExpectToken(tkEqual);
    ExpectToken(tkIdentifier);
    Result.ImplementationProc:=CreatePrimitiveExpr(Result,pekIdent,CurTokenString);
    NextToken;
    if CurToken=tkSemicolon then
    else if CurToken=tkend then
      UngetToken
    else
      CheckToken(tkSemicolon);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

function TPasParser.ParseProperty(Parent: TPasElement; const AName: String;
  AVisibility: TPasMemberVisibility; IsClassField: boolean): TPasProperty;

  function GetAccessorName(aParent: TPasElement; out Expr: TPasExpr): String;
  var
    Params: TParamsExpr;
    Param: TPasExpr;
    SrcPos: TPasSourcePos;
  begin
    NextToken;
    // read ident.subident...
    Result:=ReadDottedIdentifier(aParent,Expr,true);

    // read optional array index
    if CurToken <> tkSquaredBraceOpen then
      UnGetToken
    else
      begin
      Result := Result + '[';
      Param:=Nil;
      Params:=TParamsExpr(CreateElement(TParamsExpr,'',aParent));
      Params.Kind:=pekArrayParams;
      Params.Value:=Expr;
      Expr.Parent:=Params;
      Expr:=Params;
      NextToken;
      case CurToken of
        tkChar:             Param:=CreatePrimitiveExpr(aParent,pekString, CurTokenText);
        tkNumber:           Param:=CreatePrimitiveExpr(aParent,pekNumber, CurTokenString);
        tkIdentifier:       Param:=CreatePrimitiveExpr(aParent,pekIdent, CurTokenText);
        tkfalse, tktrue:    Param:=CreateBoolConstExpr(aParent,pekBoolConst, CurToken=tktrue);
      else
        ParseExcExpectedIdentifier;
      end;
      Params.AddParam(Param);
      Result := Result + CurTokenString;
      ExpectToken(tkSquaredBraceClose);
      Result := Result + ']';
      end;
    repeat
      NextToken;
      if CurToken <> tkDot then
        begin
        UngetToken;
        break;
        end;
      SrcPos:=CurTokenPos;
      ExpectIdentifier;
      Result := Result + '.' + CurTokenString;
      AddToBinaryExprChain(Expr,CreatePrimitiveExpr(aParent,pekIdent,CurTokenString),
                           eopSubIdent,SrcPos);
    until false;
  end;

  procedure ParseImplements;
  var
    Identifier: String;
    Expr: TPasExpr;
    l: Integer;
  begin
    // comma list of identifiers
    repeat
      ExpectToken(tkIdentifier);
      l:=length(Result.Implements);
      Identifier:=ReadDottedIdentifier(Result,Expr,l=0);
      if l=0 then
        Result.ImplementsName := Identifier;
      SetLength(Result.Implements,l+1);
      Result.Implements[l]:=Expr;
    until CurToken<>tkComma;
  end;

  procedure ConsumeSemi;
  begin
    if (CurToken = tkSemicolon) then
      begin
      NextToken;
      if IsCurTokenHint then
        UngetToken;
      end;
  end;

var
  isArray , ok, IsClass: Boolean;
  ObjKind: TPasObjKind;
begin
  Result:=TPasProperty(CreateElement(TPasProperty,AName,Parent,AVisibility));
  if IsClassField then
    Include(Result.VarModifiers,vmClass);
  IsClass:=(Parent<>nil) and (Parent.ClassType=TPasClassType);
  if IsClass then
    ObjKind:=TPasClassType(Parent).ObjKind
  else
    ObjKind:=okClass;
  ok:=false;
  try
    NextToken;
    isArray:=CurToken=tkSquaredBraceOpen;
    if isArray then
      begin
      ParseArgList(Result, Result.Args, tkSquaredBraceClose);
      NextToken;
      end;
    if CurToken = tkColon then
      begin
      Result.VarType := ParseType(Result,CurSourcePos);
      {$IFDEF CheckPasTreeRefCount}if Result.VarType.RefIds.IndexOf('CreateElement')>=0 then Result.VarType.ChangeRefId('CreateElement','TPasVariable.VarType'){$ENDIF};
      NextToken;
      end
    else if not IsClass then
      ParseExcTokenError(':');
    if CurTokenIsIdentifier('INDEX') then
      begin
      NextToken;
      Result.IndexExpr := DoParseExpression(Result);
      end;
    if CurTokenIsIdentifier('READ') then
      begin
      Result.ReadAccessorName := GetAccessorName(Result,Result.ReadAccessor);
      NextToken;
      end;
    if CurTokenIsIdentifier('WRITE') then
      begin
      Result.WriteAccessorName := GetAccessorName(Result,Result.WriteAccessor);
      NextToken;
      end;
    if IsClass and (ObjKind=okDispInterface) then
      begin
      if CurTokenIsIdentifier('READONLY') then
        begin
        Result.DispIDReadOnly:=True;
        NextToken;
        end;
      if CurTokenIsIdentifier('DISPID') then
        begin
        NextToken;
        Result.DispIDExpr := DoParseExpression(Result,Nil);
        end;
      end;
    if IsClass and (ObjKind=okClass) and CurTokenIsIdentifier('IMPLEMENTS') then
      ParseImplements;
    if CurTokenIsIdentifier('STORED') then
      begin
      if not (ObjKind in [okClass]) then
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['STORED',ObjKindNames[ObjKind]]);
      NextToken;
      if CurToken = tkTrue then
        begin
        Result.StoredAccessorName := 'True';
        Result.StoredAccessor := CreateBoolConstExpr(Result,pekBoolConst,true);
        end
      else if CurToken = tkFalse then
        begin
        Result.StoredAccessorName := 'False';
        Result.StoredAccessor := CreateBoolConstExpr(Result,pekBoolConst,false);
        end
      else if CurToken = tkIdentifier then
        begin
        UngetToken;
        Result.StoredAccessorName := GetAccessorName(Result,Result.StoredAccessor);
        end
      else
        ParseExcSyntaxError;
      NextToken;
      end;
    if CurTokenIsIdentifier('DEFAULT') then
      begin
      if not (ObjKind in [okClass,okClassHelper]) then // FPC allows it in type helpers
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['DEFAULT',ObjKindNames[ObjKind]]);
      if isArray then
        ParseExc(nParserArrayPropertiesCannotHaveDefaultValue,SParserArrayPropertiesCannotHaveDefaultValue);
      NextToken;
      Result.DefaultExpr := DoParseExpression(Result);
  //      NextToken;
      end
    else if CurtokenIsIdentifier('NODEFAULT') then
      begin
      if not (ObjKind in [okClass]) then
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['NODEFAULT',ObjKindNames[ObjKind]]);
      Result.IsNodefault:=true;
      if Result.DefaultExpr<>nil then
        ParseExcSyntaxError;
      NextToken;
      end;
    // Here the property ends. There can still be a 'default'
    if CurToken = tkSemicolon then
      begin
      NextToken;
      if CurTokenIsIdentifier('DEFAULT') then
        begin
        if (Result.VarType<>Nil) and (not isArray) then
          ParseExc(nParserDefaultPropertyMustBeArray,SParserDefaultPropertyMustBeArray);
        NextToken;
        if CurToken = tkSemicolon then
          begin
          Result.IsDefault := True;
          NextToken;
          end
        end;
      // Handle hints
      while DoCheckHint(Result) do
        begin
        NextToken; // eat Hint token
        ConsumeSemi; // Now on hint token or semicolon
        end;
//      if Result.Hints=[] then
        UngetToken;
      end
    else if CurToken=tkend then
      // ok
    else
      CheckToken(tkSemicolon);
    ok:=true;
  finally
    if not ok then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

// Starts after the "begin" token
procedure TPasParser.ParseProcBeginBlock(Parent: TProcedureBody);
var
  BeginBlock: TPasImplBeginBlock;
  SubBlock: TPasImplElement;
  Proc: TPasProcedure;
begin
  BeginBlock := TPasImplBeginBlock(CreateElement(TPasImplBeginBlock, '', Parent));
  Parent.Body := BeginBlock;
  // these can be used in code for typecasts
  Scanner.SetNonToken(tkobjccategory);
  Scanner.SetNonToken(tkobjcprotocol);
  Scanner.SetNonToken(tkobjcclass);
  repeat
    NextToken;
//    writeln('TPasParser.ParseProcBeginBlock ',curtokenstring);
    if CurToken=tkend then
      break
    else if CurToken<>tkSemiColon then
    begin
      UngetToken;
      ParseStatement(BeginBlock,SubBlock);
      if SubBlock=nil then
        ExpectToken(tkend);
    end;
  until false;
  // A declaration can follow...
  Scanner.UnSetNonToken(tkobjccategory);
  Scanner.UnSetNonToken(tkobjcprotocol);
  Scanner.UnSetNonToken(tkobjcclass);
  Proc:=Parent.Parent as TPasProcedure;
  if Proc.GetProcTypeEnum in [ptAnonymousProcedure,ptAnonymousFunction] then
    NextToken
  else
    ExpectToken(tkSemicolon);
//  writeln('TPasParser.ParseProcBeginBlock ended ',curtokenstring);
end;

procedure TPasParser.ParseProcAsmBlock(Parent: TProcedureBody);
var
  AsmBlock: TPasImplAsmStatement;
begin
  AsmBlock:=TPasImplAsmStatement(CreateElement(TPasImplAsmStatement,'',Parent));
  Parent.Body:=AsmBlock;
  ParseAsmBlock(AsmBlock);
  NextToken;
  if not (Parent.Parent is TPasAnonymousProcedure) then
    CheckToken(tkSemicolon);
end;

procedure TPasParser.ParseAsmBlock(AsmBlock: TPasImplAsmStatement);

Var
  LastToken : TToken;
  p: PTokenRec;

  Function atEndOfAsm : Boolean;

  begin
    Result:=(CurToken=tkEnd) and not (LastToken in [tkAt,tkAtAt]);
  end;

begin
  if po_asmwhole in Options then
    begin
    FTokenRingCur:=0;
    FTokenRingStart:=0;
    FTokenRingEnd:=1;
    p:=@FTokenRing[0];
    p^.Comments.Clear;
    repeat
      Scanner.ReadNonPascalTillEndToken(true);
      case Scanner.CurToken of
      tkLineEnding,tkWhitespace:
        AsmBlock.Tokens.Add(Scanner.CurTokenString);
      tkend:
        begin
        p^.Token := tkend;
        p^.AsString := Scanner.CurTokenString;
        break;
        end
      else
        begin
        // missing end
        p^.Token := tkEOF;
        p^.AsString := '';
        break;
        end;
      end;
    until false;
    FCurToken := p^.Token;
    FCurTokenString := p^.AsString;
    CheckToken(tkend);
    end
  else
    begin
    LastToken:=tkEOF;
    NextToken;
    While Not atEndOfAsm do
      begin
      AsmBlock.Tokens.Add(CurTokenText);
      LastToken:=CurToken;
      NextToken;
      end;
    end;
  // Do not consume end. Current token will normally be end;
end;

// Next token is start of (compound) statement
// After parsing CurToken is on last token of statement
procedure TPasParser.ParseStatement(Parent: TPasImplBlock;
  out NewImplElement: TPasImplElement);
var
  CurBlock: TPasImplBlock;

  {$IFDEF VerbosePasParser}
  function i: string;
  var
    c: TPasElement;
  begin
    Result:='ParseImplCompoundStatement ';
    c:=CurBlock;
    while c<>nil do begin
      Result:=Result+'  ';
      c:=c.Parent;
    end;
  end;
  {$ENDIF}

  function CloseBlock: boolean; // true if parent reached
  var C: TPasImplBlockClass;
    NeedUnget: Boolean;
  begin
    C:=TPasImplBlockClass(CurBlock.ClassType);
    if C=TPasImplExceptOn then
      begin
      Engine.FinishScope(stExceptOnStatement,CurBlock);
      NeedUnget:=CurToken=tkSemicolon;
      if NeedUnget then
        NextToken;
      if (CurToken in [tkend,tkelse])
          or ((CurToken=tkIdentifier) and (lowercase(CurTokenString)='on')) then
        // ok
      else
        ParseExcExpectedAorB('end','on');
      if NeedUnget then
        UngetToken;
      end
    else if C=TPasImplWithDo then
      Engine.FinishScope(stWithExpr,CurBlock);
    CurBlock:=CurBlock.Parent as TPasImplBlock;
    Result:=CurBlock=Parent;
  end;

  function CloseStatement(CloseIfs: boolean): boolean; // true if parent reached
  begin
    if CurBlock=Parent then exit(true);
    while CurBlock.CloseOnSemicolon
        or (CloseIfs and (CurBlock is TPasImplIfElse)) do
      if CloseBlock then exit(true);
    Result:=false;
  end;

  procedure CreateBlock(NewBlock: TPasImplBlock);
  begin
    CurBlock.AddElement(NewBlock);
    CurBlock:=NewBlock;
    if NewImplElement=nil then NewImplElement:=CurBlock;
  end;

  procedure CheckStatementCanStart;
  var
    t: TToken;
  begin
    if (CurBlock.Elements.Count=0) then
      exit; // at start of block
    t:=GetPrevToken;
    if t in [tkSemicolon,tkColon,tkElse,tkotherwise] then
      exit;
    {$IFDEF VerbosePasParser}
    writeln('TPasParser.ParseStatement.CheckSemicolon Prev=',GetPrevToken,' Cur=',CurToken,' ',CurBlock.ClassName,' ',CurBlock.Elements.Count,' ',TObject(CurBlock.Elements[0]).ClassName);
    {$ENDIF}
    // last statement not complete -> semicolon is missing
    ParseExcTokenError('Semicolon');
  end;

var
  CmdElem: TPasImplElement;

  procedure AddStatement(El: TPasImplElement);
  begin
    CurBlock.AddElement(El);
    CmdElem:=El;
    UngetToken;
  end;

var
  SubBlock: TPasImplElement;
  Left, Right, Expr: TPasExpr;
  El : TPasImplElement;
  lt : TLoopType;
  SrcPos: TPasSourcePos;
  Name: String;
  TypeEl: TPasType;
  ImplRaise: TPasImplRaise;
  VarEl: TPasVariable;
  ImplExceptOn: TPasImplExceptOn;

begin
  NewImplElement:=nil;
  El:=nil;
  Left:=nil;
  try
    CurBlock := Parent;
    while True do
    begin
      NextToken;
      //WriteLn({$IFDEF VerbosePasParser}i,{$ENDIF}' Token=',CurTokenText,' CurBlock=',CurBlock.ClassName);
      case CurToken of
      tkasm:
        begin
        CheckStatementCanStart;
        El:=TPasImplElement(CreateElement(TPasImplAsmStatement,'',CurBlock,CurTokenPos));
        ParseAsmBlock(TPasImplAsmStatement(El));
        CurBlock.AddElement(El);
        El:=nil;
        if NewImplElement=nil then NewImplElement:=CurBlock;
        if CloseStatement(False) then
          break;
        end;
      tkbegin:
        begin
        CheckStatementCanStart;
        El:=TPasImplElement(CreateElement(TPasImplBeginBlock,'',CurBlock,CurTokenPos));
        CreateBlock(TPasImplBeginBlock(El));
        El:=nil;
        end;
      tkrepeat:
        begin
        CheckStatementCanStart;
        El:=TPasImplRepeatUntil(CreateElement(TPasImplRepeatUntil,'',CurBlock,CurTokenPos));
        CreateBlock(TPasImplRepeatUntil(El));
        El:=nil;
        end;
      tkIf:
        begin
        CheckStatementCanStart;
        SrcPos:=CurTokenPos;
        NextToken;
        Left:=DoParseExpression(CurBlock);
        UngetToken;
        El:=TPasImplIfElse(CreateElement(TPasImplIfElse,'',CurBlock,SrcPos));
        TPasImplIfElse(El).ConditionExpr:=Left;
        Left.Parent:=El;
        Left:=nil;
        //WriteLn(i,'IF Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(TPasImplIfElse(El));
        El:=nil;
        ExpectToken(tkthen);
        end;
      tkelse,tkotherwise:
        // ELSE can close multiple blocks, similar to semicolon
        repeat
          {$IFDEF VerbosePasParser}
          writeln('TPasParser.ParseStatement ELSE CurBlock=',CurBlock.ClassName);
          {$ENDIF}
          if CurBlock is TPasImplIfElse then
            begin
            if TPasImplIfElse(CurBlock).IfBranch=nil then
            begin
              // empty THEN statement  e.g. if condition then else
              El:=TPasImplCommand(CreateElement(TPasImplCommand,'', CurBlock,CurTokenPos));
              CurBlock.AddElement(El); // this sets TPasImplIfElse(CurBlock).IfBranch:=El
              El:=nil;
            end;
            if (CurToken=tkelse) and (TPasImplIfElse(CurBlock).ElseBranch=nil) then
              begin
                // Check if next token is an else too
                NextToken;
                if CurToken = tkElse then
                  begin
                    // empty ELSE statement without semicolon e.g. if condition then [...] else else
                    El:=TPasImplCommand(CreateElement(TPasImplCommand,'', CurBlock,CurTokenPos));
                    CurBlock.AddElement(El); // this sets TPasImplIfElse(CurBlock).IfBranch:=El
                    El:=nil;
                    CloseBlock;
                  end;
                UngetToken;
                break; // add next statement as ElseBranch
              end;
            end
          else if (CurBlock is TPasImplTryExcept) and (CurToken=tkelse) then
            begin
            // close TryExcept handler and open an TryExceptElse handler
            CloseBlock;
            El:=TPasImplTryExceptElse(CreateElement(TPasImplTryExceptElse,'',CurBlock,CurTokenPos));
            TPasImplTry(CurBlock).ElseBranch:=TPasImplTryExceptElse(El);
            CurBlock:=TPasImplTryExceptElse(El);
            El:=nil;
            break;
            end
          else if (CurBlock is TPasImplCaseStatement) then
            begin
            UngetToken;
            // Note: a TPasImplCaseStatement is parsed by a call of ParseStatement,
            //       so it must be the top level block
            if CurBlock<>Parent then
              CheckToken(tkSemicolon);
            exit;
            end
          else if (CurBlock is TPasImplWhileDo)
              or (CurBlock is TPasImplForLoop)
              or (CurBlock is TPasImplWithDo)
              or (CurBlock is TPasImplRaise)
              or (CurBlock is TPasImplExceptOn) then
            // simply close block
          else
            ParseExcSyntaxError;
          CloseBlock;
        until false;
      tkwhile:
        begin
          // while Condition do
          CheckStatementCanStart;
          SrcPos:=CurTokenPos;
          NextToken;
          Left:=DoParseExpression(CurBlock);
          UngetToken;
          //WriteLn(i,'WHILE Condition="',Condition,'" Token=',CurTokenText);
          El:=TPasImplWhileDo(CreateElement(TPasImplWhileDo,'',CurBlock,SrcPos));
          TPasImplWhileDo(El).ConditionExpr:=Left;
          Left.Parent:=El;
          Left:=nil;
          CreateBlock(TPasImplWhileDo(El));
          El:=nil;
          ExpectToken(tkdo);
        end;
      tkgoto:
        begin
        CheckStatementCanStart;
        NextToken;
        CurBlock.AddCommand('goto '+curtokenstring);
        // expecttoken(tkSemiColon);
        end;
      tkfor:
        begin
          // for VarName := StartValue to EndValue do
          // for VarName in Expression do
          CheckStatementCanStart;
          El:=TPasImplForLoop(CreateElement(TPasImplForLoop,'',CurBlock,CurTokenPos));
          ExpectIdentifier;
          Expr:=CreatePrimitiveExpr(El,pekIdent,CurTokenString);
          TPasImplForLoop(El).VariableName:=Expr;
          repeat
            NextToken;
            case CurToken of
              tkAssign:
                begin
                lt:=ltNormal;
                break;
                end;
              tkin:
                begin
                lt:=ltIn;
                break;
                end;
              tkDot:
                begin
                SrcPos:=CurTokenPos;
                ExpectIdentifier;
                AddToBinaryExprChain(Expr,
                  CreatePrimitiveExpr(El,pekIdent,CurTokenString), eopSubIdent,SrcPos);
                TPasImplForLoop(El).VariableName:=Expr;
                end;
            else
              ParseExc(nParserExpectedAssignIn,SParserExpectedAssignIn);
            end;
          until false;
          NextToken;
          TPasImplForLoop(El).StartExpr:=DoParseExpression(El);
          if (Lt=ltNormal) then
            begin
            if Not (CurToken in [tkTo,tkDownTo]) then
              ParseExcTokenError(TokenInfos[tkTo]);
            if CurToken=tkdownto then
              Lt:=ltDown;
            NextToken;
            TPasImplForLoop(El).EndExpr:=DoParseExpression(El);
            end;
          TPasImplForLoop(El).LoopType:=lt;
          if (CurToken<>tkDo) then
            ParseExcTokenError(TokenInfos[tkDo]);
          Engine.FinishScope(stForLoopHeader,El);
          CreateBlock(TPasImplForLoop(El));
          El:=nil;
          //WriteLn(i,'FOR "',VarName,'" := ',StartValue,' to ',EndValue,' Token=',CurTokenText);
        end;
      tkwith:
        begin
          // with Expr do
          // with Expr, Expr do
          CheckStatementCanStart;
          SrcPos:=CurTokenPos;
          NextToken;
          El:=TPasImplWithDo(CreateElement(TPasImplWithDo,'',CurBlock,SrcPos));
          Expr:=DoParseExpression(CurBlock);
          //writeln(i,'WITH Expr="',Expr,'" Token=',CurTokenText);
          TPasImplWithDo(El).AddExpression(Expr);
          Expr.Parent:=El;
          Engine.BeginScope(stWithExpr,Expr);
          CreateBlock(TPasImplWithDo(El));
          El:=nil;
          repeat
            if CurToken=tkdo then break;
            if CurToken<>tkComma then
              ParseExcTokenError(TokenInfos[tkdo]);
            NextToken;
            Expr:=DoParseExpression(CurBlock);
            //writeln(i,'WITH ...,Expr="',Expr,'" Token=',CurTokenText);
            TPasImplWithDo(CurBlock).AddExpression(Expr);
            Engine.BeginScope(stWithExpr,Expr);
          until false;
        end;
      tkcase:
        begin
          CheckStatementCanStart;
          SrcPos:=CurTokenPos;
          NextToken;
          Left:=DoParseExpression(CurBlock);
          UngetToken;
          //writeln(i,'CASE OF Expr="',Expr,'" Token=',CurTokenText);
          ExpectToken(tkof);
          El:=TPasImplCaseOf(CreateElement(TPasImplCaseOf,'',CurBlock,SrcPos));
          TPasImplCaseOf(El).CaseExpr:=Left;
          Left.Parent:=El;
          Left:=nil;
          CreateBlock(TPasImplCaseOf(El));
          El:=nil;
          repeat
            NextToken;
            //writeln(i,'CASE OF Token=',CurTokenText);
            case CurToken of
            tkend:
              begin
              if CurBlock.Elements.Count=0 then
                ParseExc(nParserExpectCase,SParserExpectCase);
              break; // end without else
              end;
            tkelse,tkotherwise:
              begin
                // create case-else block
                El:=TPasImplCaseElse(CreateElement(TPasImplCaseElse,'',CurBlock,CurTokenPos));
                TPasImplCaseOf(CurBlock).ElseBranch:=TPasImplCaseElse(El);
                CreateBlock(TPasImplCaseElse(El));
                El:=nil;
                break;
              end
            else
              // read case values
              repeat
                SrcPos:=CurTokenPos;
                Left:=DoParseExpression(CurBlock);
                //writeln(i,'CASE value="',Expr,'" Token=',CurTokenText);
                if CurBlock is TPasImplCaseStatement then
                  begin
                  TPasImplCaseStatement(CurBlock).AddExpression(Left);
                  Left:=nil;
                  end
                else
                  begin
                  El:=TPasImplCaseStatement(CreateElement(TPasImplCaseStatement,'',CurBlock,SrcPos));
                  TPasImplCaseStatement(El).AddExpression(Left);
                  Left:=nil;
                  CreateBlock(TPasImplCaseStatement(El));
                  El:=nil;
                  end;
                //writeln(i,'CASE after value Token=',CurTokenText);
                if (CurToken=tkComma) then
                  NextToken
                else if (CurToken<>tkColon) then
                  ParseExcTokenError(TokenInfos[tkComma]);
              until Curtoken=tkColon;
              // read statement
              ParseStatement(CurBlock,SubBlock);
              // CurToken is now at last token of case-statement
              CloseBlock;
              if CurToken<>tkSemicolon then
                NextToken;
              if (CurToken in [tkSemicolon,tkelse,tkend,tkotherwise]) then
                // ok
              else
                ParseExcTokenError(TokenInfos[tkSemicolon]);
              if CurToken<>tkSemicolon then
                UngetToken;
            end;
          until false;
          if CurToken=tkend then
          begin
            if CloseBlock then break;
            if CloseStatement(false) then break;
          end;
        end;
      tktry:
        begin
        CheckStatementCanStart;
        El:=TPasImplTry(CreateElement(TPasImplTry,'',CurBlock,CurTokenPos));
        CreateBlock(TPasImplTry(El));
        El:=nil;
        end;
      tkfinally:
        begin
          if CloseStatement(true) then
          begin
            UngetToken;
            break;
          end;
          if CurBlock is TPasImplTry then
          begin
            El:=TPasImplTryFinally(CreateElement(TPasImplTryFinally,'',CurBlock,CurTokenPos));
            TPasImplTry(CurBlock).FinallyExcept:=TPasImplTryFinally(El);
            CurBlock:=TPasImplTryFinally(El);
            El:=nil;
          end else
            ParseExcSyntaxError;
        end;
      tkexcept:
        begin
          if CloseStatement(true) then
          begin
            UngetToken;
            break;
          end;
          if CurBlock is TPasImplTry then
          begin
            //writeln(i,'EXCEPT');
            El:=TPasImplTryExcept(CreateElement(TPasImplTryExcept,'',CurBlock,CurTokenPos));
            TPasImplTry(CurBlock).FinallyExcept:=TPasImplTryExcept(El);
            CurBlock:=TPasImplTryExcept(El);
            El:=nil;
          end else
            ParseExcSyntaxError;
        end;
      tkraise:
        begin
        CheckStatementCanStart;
        ImplRaise:=TPasImplRaise(CreateElement(TPasImplRaise,'',CurBlock,CurTokenPos));
        CreateBlock(ImplRaise);
        NextToken;
        If Curtoken in [tkElse,tkEnd,tkSemicolon,tkotherwise] then
          UnGetToken
        else
          begin
          ImplRaise.ExceptObject:=DoParseExpression(ImplRaise);
          if (CurToken=tkIdentifier) and (Uppercase(CurtokenString)='AT') then
            begin
            NextToken;
            ImplRaise.ExceptAddr:=DoParseExpression(ImplRaise);
            end;
          If Curtoken in [tkElse,tkEnd,tkSemicolon,tkotherwise] then
            UngetToken
          end;
        end;
      tkend:
        begin
          // Note: ParseStatement should return with CurToken at last token of the statement
          if CloseStatement(true) then
          begin
            // there was none requiring an END
            UngetToken;
            break;
          end;
          // still a block left
          if CurBlock is TPasImplBeginBlock then
          begin
            // close at END
            if CloseBlock then break; // close end
            if CloseStatement(false) then break;
          end else if CurBlock is TPasImplCaseElse then
          begin
            if CloseBlock then break; // close else
            if CloseBlock then break; // close caseof
            if CloseStatement(false) then break;
          end else if CurBlock is TPasImplTryHandler then
          begin
            if CloseBlock then break; // close finally/except
            if CloseBlock then break; // close try
            if CloseStatement(false) then break;
          end else
            ParseExcSyntaxError;
        end;
      tkSemiColon:
        if CloseStatement(true) then break;
      tkFinalization:
        if CloseStatement(true) then
          begin
          UngetToken;
          break;
          end;
      tkuntil:
        begin
          if CloseStatement(true) then
          begin
            UngetToken;
            break;
          end;
          if CurBlock is TPasImplRepeatUntil then
          begin
            NextToken;
            Left:=DoParseExpression(CurBlock);
            UngetToken;
            TPasImplRepeatUntil(CurBlock).ConditionExpr:=Left;
            Left:=nil;
            //WriteLn(i,'UNTIL Condition="',Condition,'" Token=',CurTokenString);
            if CloseBlock then break;
          end else
            ParseExcSyntaxError;
        end;
      tkEOF:
        CheckToken(tkend);
      tkAt,tkAtAt,
      tkIdentifier,tkspecialize,
      tkNumber,tkString,tkfalse,tktrue,tkChar,
      tkBraceOpen,tkSquaredBraceOpen,
      tkMinus,tkPlus,tkinherited:
        begin
        // Do not check this here:
        //      if (CurToken=tkAt) and not (msDelphi in CurrentModeswitches) then
        //        ParseExc;
        CheckStatementCanStart;

        //writeln('TPasParser.ParseStatement ',CurToken,' ',CurTokenString);

        // On is usable as an identifier
        if lowerCase(CurTokenText)='on' then
          begin
            // in try except:
            // on E: Exception do
            // on Exception do
            if CurBlock is TPasImplTryExcept then
            begin
              SrcPos:=CurTokenPos;
              ExpectIdentifier;
              ImplExceptOn:=TPasImplExceptOn(CreateElement(TPasImplExceptOn,'',CurBlock,SrcPos));
              El:=ImplExceptOn;
              SrcPos:=CurSourcePos;
              Name:=CurTokenString;
              NextToken;
              //writeln('TPasParser.ParseStatement ',CurToken,' ',CurTokenString);
              //writeln('ON t=',Name,' Token=',CurTokenText);
              if CurToken=tkColon then
                begin
                // the first expression was the variable name
                NextToken;
                TypeEl:=ParseSimpleType(ImplExceptOn,SrcPos,'');
                ImplExceptOn.TypeEl:=TypeEl;
                VarEl:=TPasVariable(CreateElement(TPasVariable,Name,ImplExceptOn,SrcPos));
                ImplExceptOn.VarEl:=VarEl;
                VarEl.VarType:=TypeEl;
                TypeEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasVariable.VarType'){$ENDIF};
                if TypeEl.Parent=ImplExceptOn then
                  TypeEl.Parent:=VarEl;
                end
              else
                begin
                UngetToken;
                ImplExceptOn.TypeEl:=ParseSimpleType(ImplExceptOn,SrcPos,'');
                end;
              Engine.FinishScope(stExceptOnExpr,ImplExceptOn);
              CreateBlock(ImplExceptOn);
              El:=nil;
              ExpectToken(tkDo);
            end else
              ParseExcSyntaxError;
          end
        else
          begin
          SrcPos:=CurTokenPos;
          Left:=DoParseExpression(CurBlock);
          case CurToken of
            tkAssign,
            tkAssignPlus,
            tkAssignMinus,
            tkAssignMul,
            tkAssignDivision:
              begin
              // assign statement
              El:=TPasImplAssign(CreateElement(TPasImplAssign,'',CurBlock,SrcPos));
              TPasImplAssign(El).left:=Left;
              Left.Parent:=El;
              Left:=nil;
              TPasImplAssign(El).Kind:=TokenToAssignKind(CurToken);
              NextToken;
              Right:=DoParseExpression(CurBlock);
              TPasImplAssign(El).right:=Right;
              Right.Parent:=El;
              Right:=nil;
              AddStatement(El);
              El:=nil;
              end;
            tkColon:
              begin
              if not (bsGoto in Scanner.CurrentBoolSwitches) then
                ParseExcTokenError(TokenInfos[tkSemicolon])
              else if not (Left is TPrimitiveExpr) then
                ParseExcTokenError(TokenInfos[tkSemicolon]);
              // label mark. todo: check mark identifier in the list of labels
              El:=TPasImplLabelMark(CreateElement(TPasImplLabelMark,'', CurBlock,SrcPos));
              TPasImplLabelMark(El).LabelId:=TPrimitiveExpr(Left).Value;
              ReleaseAndNil(TPasElement(Left){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
              CurBlock.AddElement(El);
              CmdElem:=TPasImplLabelMark(El);
              El:=nil;
              end;
          else
            // simple statement (function call)
            El:=TPasImplSimple(CreateElement(TPasImplSimple,'',CurBlock,SrcPos));
            TPasImplSimple(El).Expr:=Left;
            Left.Parent:=El;
            Left:=nil;
            AddStatement(El);
            El:=nil;
          end;

          if not (CmdElem is TPasImplLabelMark) then
            if NewImplElement=nil then NewImplElement:=CmdElem;
          end;
        end;
      else
        ParseExcSyntaxError;
      end;
    end;
  finally
    if El<>nil then El.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
    if Left<>nil then Left.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

procedure TPasParser.ParseLabels(AParent: TPasElement);
var
  Labels: TPasLabels;
begin
  Labels:=TPasLabels(CreateElement(TPasLabels, '', AParent));
  repeat
    expectTokens([tkIdentifier,tkNumber]);
    Labels.Labels.Add(CurTokenString);
    NextToken;
    if not (CurToken in [tkSemicolon, tkComma]) then
      ParseExcTokenError(TokenInfos[tkSemicolon]);
  until CurToken=tkSemicolon;
end;

// Starts after the "procedure" or "function" token
function TPasParser.GetProcedureClass(ProcType: TProcType): TPTreeElement;

begin
  Result:=Nil;
  Case ProcType of
    ptFunction       : Result:=TPasFunction;
    ptClassFunction  : Result:=TPasClassFunction;
    ptClassProcedure : Result:=TPasClassProcedure;
    ptClassConstructor : Result:=TPasClassConstructor;
    ptClassDestructor  : Result:=TPasClassDestructor;
    ptProcedure      : Result:=TPasProcedure;
    ptConstructor    : Result:=TPasConstructor;
    ptDestructor     : Result:=TPasDestructor;
    ptOperator       : Result:=TPasOperator;
    ptClassOperator  : Result:=TPasClassOperator;
    ptAnonymousProcedure: Result:=TPasAnonymousProcedure;
    ptAnonymousFunction: Result:=TPasAnonymousFunction;
  else
    ParseExc(nParserUnknownProcedureType,SParserUnknownProcedureType,[Ord(ProcType)]);
  end;
end;

function TPasParser.ParseProcedureOrFunctionDecl(Parent: TPasElement;
  ProcType: TProcType; MustBeGeneric: boolean; AVisibility: TPasMemberVisibility
  ): TPasProcedure;
var
  NameParts: TProcedureNameParts;
  NamePos: TPasSourcePos;

  function ExpectProcName: string;
  { Simple procedure:
      Name
    Method implementation of non generic class:
      aClass.SubClass.Name
    ObjFPC generic procedure or method declaration:
      MustBeGeneric=true, Name<Templates>
    Delphi generic Method Declaration:
      MustBeGeneric=false, Name<Templates>
    ObjFPC Method implementation of generic class:
      aClass.SubClass.Name
    Delphi Method implementation of generic class:
      aClass<Templates>.SubClass<Templates>.Name
      aClass.SubClass<Templates>.Name<Templates>
  }
  Var
    L : TFPList;
    I , Cnt, p: Integer;
    CurName: String;
    Part: TProcedureNamePart;
  begin
    Result:=ExpectIdentifier;
    NamePos:=CurSourcePos;
    Cnt:=1;
    repeat
      NextToken;
      if CurToken=tkDot then
        begin
        if Parent is TImplementationSection then
          begin
          inc(Cnt);
          CurName:=ExpectIdentifier;
          NamePos:=CurSourcePos;
          Result:=Result+'.'+CurName;
          if NameParts<>nil then
            begin
            Part:=TProcedureNamePart.Create;
            NameParts.Add(Part);
            Part.Name:=CurName;
            end;
          end
        else
          ParseExcSyntaxError;
        end
      else if CurToken=tkLessThan then
        begin
        if (not MustBeGeneric) and not (msDelphi in CurrentModeswitches) then
          ParseExc(nParserGenericFunctionNeedsGenericKeyword,SParserGenericFunctionNeedsGenericKeyword);
        // generic templates
        if NameParts=nil then
          begin
          // initialize NameParts
          NameParts:=TProcedureNameParts.Create;
          i:=0;
          CurName:=Result;
          repeat
            Part:=TProcedureNamePart.Create;
            NameParts.Add(Part);
            p:=Pos('.',CurName);
            if p>0 then
              begin
              Part.Name:=LeftStr(CurName,p-1);
              System.Delete(CurName,1,p);
              end
            else
              begin
              Part.Name:=CurName;
              break;
              end;
            inc(i);
          until false;
          end
        else if TProcedureNamePart(NameParts[Cnt-1]).Templates<>nil then
          ParseExcSyntaxError;
        UnGetToken;
        L:=TFPList.Create;
        TProcedureNamePart(NameParts[Cnt-1]).Templates:=L;
        ReadGenericArguments(L,Parent);
        end
      else
        break;
    until false;
    if (NameParts=nil) and MustBeGeneric then
      CheckToken(tkLessThan);
    UngetToken;
  end;

var
  OperatorTypeName,Name: String;
  PC : TPTreeElement;
  Ot : TOperatorType;
  IsTokenBased , ok: Boolean;
  j, i: Integer;

begin
  OperatorTypeName:='';
  NameParts:=nil;
  Result:=nil;
  ok:=false;
  try
    case ProcType of
    ptOperator,ptClassOperator:
      begin
      if MustBeGeneric then
        ParseExcTokenError('procedure');
      NextToken;
      IsTokenBased:=CurToken<>tkIdentifier;
      if IsTokenBased then
        OT:=TPasOperator.TokenToOperatorType(CurTokenText)
      else
        begin
        OT:=TPasOperator.NameToOperatorType(CurTokenString);
        OperatorTypeName:=CurTokenString;
        // Case Class operator TMyRecord.+
        if (OT=otUnknown) then
          begin
          NextToken;
          if CurToken<>tkDot then
            ParseExc(nErrUnknownOperatorType,SErrUnknownOperatorType,[OperatorTypeName]);
          NextToken;
          IsTokenBased:=CurToken<>tkIdentifier;
          if IsTokenBased then
            OT:=TPasOperator.TokenToOperatorType(CurTokenText)
          else
            OT:=TPasOperator.NameToOperatorType(CurTokenString);
          end;
        end;
      if (ot=otUnknown) then
        ParseExc(nErrUnknownOperatorType,SErrUnknownOperatorType,[CurTokenString]);
      Name:=OperatorNames[Ot];
      if OperatorTypeName<>'' then
        Name:=OperatorTypeName+'.'+Name;
      NamePos:=CurTokenPos;
      end;
    ptAnonymousProcedure,ptAnonymousFunction:
      begin
      Name:='';
      if MustBeGeneric then
        ParseExcTokenError('generic'); // inconsistency
      NamePos:=CurTokenPos;
      end
    else
      Name:=ExpectProcName;
    end;
    PC:=GetProcedureClass(ProcType);
    if Name<>'' then
      Parent:=CheckIfOverLoaded(Parent,Name);
    Result := TPasProcedure(Engine.CreateElement(PC, Name, Parent, AVisibility,
                                                 NamePos, NameParts));
    if NameParts<>nil then
      begin
      if Result.NameParts=nil then
        // CreateElement has not used the NameParts -> do it now
        Result.SetNameParts(NameParts);
      // sanity check
      for i:=0 to Result.NameParts.Count-1 do
        with TProcedureNamePart(Result.NameParts[i]) do
          if Templates<>nil then
            for j:=0 to Templates.Count-1 do
              if TPasElement(Templates[j]).Parent<>Result then
                ParseExc(nParserError,SParserError+'[20190818131750] '+TPasElement(Templates[j]).Parent.Name+':'+TPasElement(Templates[j]).Parent.ClassName);
      if NameParts.Count>0 then
        ParseExc(nParserError,SParserError+'[20190818131909] "'+Name+'"');
      end;
    case ProcType of
    ptFunction, ptClassFunction, ptOperator, ptClassOperator, ptAnonymousFunction:
      begin
      Result.ProcType := CreateFunctionType('', 'Result', Result, False, CurTokenPos);
      if (ProcType in [ptOperator, ptClassOperator]) then
        begin
        TPasOperator(Result).TokenBased:=IsTokenBased;
        TPasOperator(Result).OperatorType:=OT;
        TPasOperator(Result).CorrectName;
        end;
      end;
    else
      Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '', Result));
    end;
    ParseProcedureOrFunction(Result, Result.ProcType, ProcType, False);
    Result.Hints:=Result.ProcType.Hints;
    Result.HintMessage:=Result.ProcType.HintMessage;
    // + is detected as 'positive', but is in fact Add if there are 2 arguments.
    if (ProcType in [ptOperator, ptClassOperator]) then
      With TPasOperator(Result) do
        begin
        if (OperatorType in [otPositive, otNegative]) then
          begin
          if (ProcType.Args.Count>1) then
            begin
            Case OperatorType of
              otPositive : OperatorType:=otPlus;
              otNegative : OperatorType:=otMinus;
            else
            end;
            Name:=OperatorNames[OperatorType];
            TPasOperator(Result).CorrectName;
            end;
          end;
        end;
    ok:=true;
  finally
    if NameParts<>nil then
      ReleaseProcNameParts(NameParts);
    if (not ok) and (Result<>nil) then
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  end;
end;

// Current token is the first token after tkOf
procedure TPasParser.ParseRecordVariantParts(ARec: TPasRecordType;
  AEndToken: TToken);

Var
  M : TPasRecordType;
  V : TPasVariant;
  Done : Boolean;

begin
  Repeat
    V:=TPasVariant(CreateElement(TPasVariant, '', ARec));
    ARec.Variants.Add(V);
    Repeat
      NextToken;
      V.Values.Add(DoParseExpression(ARec));
      if Not (CurToken in [tkComma,tkColon]) then
        ParseExc(nParserExpectedCommaColon,SParserExpectedCommaColon);
    Until (curToken=tkColon);
    ExpectToken(tkBraceOpen);
    NextToken;
    M:=TPasRecordType(CreateElement(TPasRecordType,'',V));
    V.Members:=M;
    ParseRecordMembers(M,tkBraceClose,False);
    // Current token is closing ), so we eat that
    NextToken;
    // If there is a semicolon, we eat that too.
    if CurToken=tkSemicolon then
      NextToken;
    // ParseExpression starts with a nexttoken.
    // So we need to determine the next token, and if it is an ending token, unget.
    Done:=CurToken=AEndToken;
    If not Done then
      Ungettoken;
  Until Done;
end;

{$ifdef VerbosePasParser}
procedure TPasParser.DumpCurToken(const Msg: String; IndentAction: TIndentAction
  );
begin
  {AllowWriteln}
  if IndentAction=iaUndent then
    FDumpIndent:=copy(FDumpIndent,1,Length(FDumpIndent)-2);
  Writeln(FDumpIndent,Msg,' : ',TokenInfos[CurToken],' "',CurTokenString,'", Position: ',Scanner.CurFilename,'(',Scanner.CurRow,',',SCanner.CurColumn,') : ',Scanner.CurLine);
  if IndentAction=iaIndent then
    FDumpIndent:=FDumpIndent+'  ';
  {$ifdef pas2js}
  // ToDo
  {$else}
  Flush(output);
  {$endif}
  {AllowWriteln-}
end;
{$endif}

function TPasParser.GetCurrentModeSwitches: TModeSwitches;
begin
  if Assigned(FScanner) then
    Result:=FScanner.CurrentModeSwitches
  else
    Result:=[msNone];
end;

procedure TPasParser.SetCurrentModeSwitches(AValue: TModeSwitches);
begin
  if Assigned(FScanner) then
    FScanner.CurrentModeSwitches:=AValue;
end;

// Starts on first token after Record or (. Ends on AEndToken
procedure TPasParser.ParseRecordMembers(ARec: TPasRecordType;
  AEndToken: TToken; AllowMethods: Boolean);
var
  isClass : Boolean;

  procedure EnableIsClass;
  begin
    isClass:=True;
    Scanner.SetTokenOption(toOperatorToken);
  end;

  procedure DisableIsClass;
  begin
    if not isClass then exit;
    isClass:=false;
    Scanner.UnSetTokenOption(toOperatorToken);
  end;

  Function CheckSection : Boolean;

  begin
    // Advanced records can have empty sections.
    { Use Case:
      Record
      type
      const
      var
      Case Integer of
      end;
    }
    NextToken;
    Result:=CurToken in [tkvar,tktype,tkConst,tkCase];
    if Not Result then
      UngetToken;
  end;

Var
  VariantName : String;
  v : TPasMemberVisibility;
  Proc: TPasProcedure;
  ProcType: TProcType;
  Prop : TPasProperty;
  NamePos: TPasSourcePos;
  OldCount, i: Integer;
  CurEl: TPasElement;
  LastToken: TToken;
  AllowVisibility: Boolean;
  IsGeneric : Boolean;

begin
  IsGeneric:=False;
  AllowVisibility:=msAdvancedRecords in CurrentModeswitches;
  if AllowVisibility then
    v:=visPublic
  else
    v:=visDefault;
  isClass:=False;
  LastToken:=tkrecord;
  while CurToken<>AEndToken do
    begin
    SaveComments;
    Case CurToken of
      tkType:
        begin
        DisableIsClass;
        if Not AllowMethods then
          ParseExc(nErrRecordTypesNotAllowed,SErrRecordTypesNotAllowed);
        if CheckSection then
          continue;
        ExpectToken(tkIdentifier);
        ParseMembersLocalTypes(ARec,v);
        end;
      tkConst:
        begin
        DisableIsClass;
        if Not AllowMethods then
          ParseExc(nErrRecordConstantsNotAllowed,SErrRecordConstantsNotAllowed);
        if CheckSection then
          continue;
        ExpectToken(tkIdentifier);
        ParseMembersLocalConsts(ARec,v);
        end;
      tkVar:
        begin
        if Not AllowMethods then
          ParseExc(nErrRecordVariablesNotAllowed,SErrRecordVariablesNotAllowed);
        if CheckSection then
          continue;
        ExpectToken(tkIdentifier);
        OldCount:=ARec.Members.Count;
        ParseInlineVarDecl(ARec, ARec.Members, v, AEndToken=tkBraceClose);
        for i:=OldCount to ARec.Members.Count-1 do
          begin
          CurEl:=TPasElement(ARec.Members[i]);
          if CurEl.ClassType=TPasAttributes then continue;
          if isClass then
            With TPasVariable(CurEl) do
              VarModifiers:=VarModifiers + [vmClass];
          Engine.FinishScope(stDeclaration,TPasVariable(CurEl));
          end;
        end;
      tkClass:
        begin
        if LastToken=tkclass then
          ParseExc(nParserTypeSyntaxError,SParserTypeSyntaxError);
        if Not AllowMethods then
          begin
          NextToken;
          case CurToken of
          tkConst: ParseExc(nErrRecordConstantsNotAllowed,SErrRecordConstantsNotAllowed);
          tkvar: ParseExc(nErrRecordVariablesNotAllowed,SErrRecordVariablesNotAllowed);
          else
            ParseExc(nErrRecordMethodsNotAllowed,SErrRecordMethodsNotAllowed);
          end;
          end;
        EnableIsClass;
        end;
      tkProperty:
        begin
        DisableIsClass;
        if Not AllowMethods then
          ParseExc(nErrRecordPropertiesNotAllowed,SErrRecordPropertiesNotAllowed);
        ExpectToken(tkIdentifier);
        Prop:=ParseProperty(ARec,CurtokenString,v,LastToken=tkclass);
        ARec.Members.Add(Prop);
        Engine.FinishScope(stDeclaration,Prop);
        end;
      tkOperator,
      tkProcedure,
      tkConstructor,
      tkFunction :
        begin
        DisableIsClass;
        if Not AllowMethods then
          ParseExc(nErrRecordMethodsNotAllowed,SErrRecordMethodsNotAllowed);
        ProcType:=GetProcTypeFromToken(CurToken,LastToken=tkclass);
        Proc:=ParseProcedureOrFunctionDecl(ARec,ProcType,IsGeneric,v);
        if Proc.Parent is TPasOverloadedProc then
          TPasOverloadedProc(Proc.Parent).Overloads.Add(Proc)
        else
          ARec.Members.Add(Proc);
        Engine.FinishScope(stProcedure,Proc);
        end;
      tkDestructor:
        ParseExc(nParserNoConstructorAllowed,SParserNoConstructorAllowed);
      tkGeneric, // Can count as field name
      tkabsolute,
      tkSelf, // Count as field name
      tkIdentifier :
        begin
        if (Curtoken=tkGeneric) and AllowVisibility then
          begin
          NextToken;
          if CurToken in [tkClass,tkOperator,tkFunction,tkProcedure] then
            begin
            IsGeneric:=True;
            Continue;
            end;
          UnGetToken;
          end;
        If AllowVisibility and CheckVisibility(CurTokenString,v) then
          begin
          if not (v in [visPrivate,visPublic,visStrictPrivate]) then
            ParseExc(nParserInvalidRecordVisibility,SParserInvalidRecordVisibility);
          NextToken;
          Continue;
          end;
        OldCount:=ARec.Members.Count;
        ParseInlineVarDecl(ARec, ARec.Members, v, AEndToken=tkBraceClose);
        for i:=OldCount to ARec.Members.Count-1 do
          begin
          CurEl:=TPasElement(ARec.Members[i]);
          if CurEl.ClassType=TPasAttributes then continue;
          if isClass then
            With TPasVariable(CurEl) do
              VarModifiers:=VarModifiers + [vmClass];
          Engine.FinishScope(stDeclaration,TPasVariable(CurEl));
          end;
        end;
      tkSquaredBraceOpen:
        if msPrefixedAttributes in CurrentModeswitches then
          ParseAttributes(ARec,true)
        else
          CheckToken(tkIdentifier);
      tkCase :
        begin
        DisableIsClass;
        ARec.Variants:=TFPList.Create;
        NextToken;
        VariantName:=CurTokenString;
        NamePos:=CurSourcePos;
        NextToken;
        If CurToken=tkColon then
          begin
          ARec.VariantEl:=TPasVariable(CreateElement(TPasVariable,VariantName,ARec,NamePos));
          TPasVariable(ARec.VariantEl).VarType:=ParseType(ARec,CurSourcePos);
          end
        else
          begin
          UnGetToken;
          UnGetToken;
          ARec.VariantEl:=ParseType(ARec,CurSourcePos);
          end;
        ExpectToken(tkOf);
        ParseRecordVariantParts(ARec,AEndToken);
        end;
    else
      ParseExc(nParserTypeSyntaxError,SParserTypeSyntaxError);
    end;
    if CurToken=AEndToken then
      break;
    LastToken:=CurToken;
    NextToken;
    if not IsClass then
      IsGeneric:=False;
    end;
end;

// Starts after the "record" token
function TPasParser.ParseRecordDecl(Parent: TPasElement;
  const NamePos: TPasSourcePos; const TypeName: string;
  const Packmode: TPackMode): TPasRecordType;

var
  ok: Boolean;
  allowadvanced : Boolean;

begin
  Result := TPasRecordType(CreateElement(TPasRecordType, TypeName, Parent, NamePos));
  ok:=false;
  try
    Result.PackMode:=PackMode;
    NextToken;
    allowAdvanced:=(msAdvancedRecords in Scanner.CurrentModeSwitches)
                   and not (Parent is TProcedureBody)
                   and (Result.Name<>'');
    ParseRecordMembers(Result,tkEnd,allowAdvanced);
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      begin
      Result.Parent:=nil; // clear references from members to Result
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
  end;
end;

Function IsVisibility(S : String;  var AVisibility :TPasMemberVisibility; IsObjCProtocol : Boolean) : Boolean;

Const
  VNames : array[TPasMemberVisibility] of string =
    ('', 'private', 'protected', 'public', 'published', 'automated', '', '','required','optional');
  VLast : Array[Boolean] of TPasMemberVisibility = (visAutomated,visOptional);

Var
  V : TPasMemberVisibility;

begin
  Result:=False;
  S:=lowerCase(S);
  For V :=Low(TPasMemberVisibility) to VLast[isObjCProtocol] do
    begin
    Result:=(VNames[V]<>'') and (S=VNames[V]);
    if Result then
      begin
      AVisibility := v;
      Exit;
      end;
    end;
end;

function TPasParser.CheckVisibility(S: String; var AVisibility: TPasMemberVisibility; IsObjCProtocol : Boolean = false): Boolean;

Var
  B : Boolean;

begin
  s := LowerCase(CurTokenString);
  B:=(S='strict');
  if B then
    begin
    NextToken;
    s:=LowerCase(CurTokenString);
    end;
  Result:=isVisibility(S,AVisibility,isObjCProtocol);
  if Result then
    begin
    if (AVisibility=visPublished) and (msOmitRTTI in Scanner.CurrentModeSwitches) then
      AVisibility:=visPublic;
    if B then
      case AVisibility of
        visPrivate   : AVisibility:=visStrictPrivate;
        visProtected : AVisibility:=visStrictProtected;
      else
        ParseExc(nParserStrangeVisibility,SParserStrangeVisibility,[S]);
      end
    end
  else if B then
    ParseExc(nParserExpectVisibility,SParserExpectVisibility);
end;

procedure TPasParser.ProcessMethod(AType: TPasClassType; IsClass: Boolean;
  AVisibility: TPasMemberVisibility; MustBeGeneric: boolean);

var
  Proc: TPasProcedure;
  ProcType: TProcType;
begin
  ProcType:=GetProcTypeFromToken(CurToken,isClass);
  Proc:=ParseProcedureOrFunctionDecl(AType,ProcType,MustBeGeneric,AVisibility);
  if Proc.Parent is TPasOverloadedProc then
    TPasOverloadedProc(Proc.Parent).Overloads.Add(Proc)
  else
    AType.Members.Add(Proc);
  Engine.FinishScope(stProcedure,Proc);
end;

procedure TPasParser.ParseClassFields(AType: TPasClassType;
  const AVisibility: TPasMemberVisibility; IsClassField: Boolean);

Var
  VarList: TFPList;
  Element: TPasElement;
  I : Integer;
  isStatic : Boolean;
  VarEl: TPasVariable;

begin
  VarList := TFPList.Create;
  try
    ParseInlineVarDecl(AType, VarList, AVisibility, False);
    if CurToken=tkSemicolon then
      begin
      NextToken;
      isStatic:=CurTokenIsIdentifier('static');
      if isStatic then
        ExpectToken(tkSemicolon)
      else
        UngetToken;
      end;
    for i := 0 to VarList.Count - 1 do
      begin
      Element := TPasElement(VarList[i]);
      Element.Visibility := AVisibility;
      AType.Members.Add(Element);
      if (Element is TPasVariable) then
        begin
        VarEl:=TPasVariable(Element);
        if IsClassField then
          Include(VarEl.VarModifiers,vmClass);
        if isStatic then
          Include(VarEl.VarModifiers,vmStatic);
        Engine.FinishScope(stDeclaration,VarEl);
        end;
      end;
  finally
    VarList.Free;
  end;
end;

procedure TPasParser.ParseMembersLocalTypes(AType: TPasMembersType;
  AVisibility: TPasMemberVisibility);

Var
  T : TPasType;
  Done : Boolean;
begin
  Done:=False;
  //Writeln('Parsing local types');
  while (CurToken=tkSquaredBraceOpen)
      and (msPrefixedAttributes in CurrentModeswitches) do
    begin
    ParseAttributes(AType,true);
    NextToken;
    end;
  Repeat
    T:=ParseTypeDecl(AType);
    T.Visibility:=AVisibility;
    AType.Members.Add(t);
    // Writeln(CurtokenString,' ',TokenInfos[Curtoken]);
    NextToken;
    case CurToken of
    tkgeneric:
      begin
      NextToken;
      if CurToken<>tkIdentifier then
        Done:=true;
      UngetToken;
      end;
    tkIdentifier:
      Done:=CheckVisibility(CurTokenString,AVisibility);
    tkSquaredBraceOpen:
      if msPrefixedAttributes in CurrentModeswitches then
        repeat
          ParseAttributes(AType,true);
          NextToken;
          Done:=false;
        until CurToken<>tkSquaredBraceOpen
      else
        Done:=true;
    else
      Done:=true;
    end;
    if Done then
      UngetToken;
  Until Done;
  Engine.FinishScope(stTypeSection,AType);
end;

procedure TPasParser.ParseMembersLocalConsts(AType: TPasMembersType;
  AVisibility: TPasMemberVisibility);

Var
  C : TPasConst;
  Done : Boolean;
begin
  // Writeln('Parsing local consts');
  while (CurToken=tkSquaredBraceOpen)
      and (msPrefixedAttributes in CurrentModeswitches) do
    begin
    ParseAttributes(AType,true);
    NextToken;
    end;
  Repeat
    C:=ParseConstDecl(AType);
    C.Visibility:=AVisibility;
    AType.Members.Add(C);
    Engine.FinishScope(stDeclaration,C);
    //Writeln('TPasParser.ParseMembersLocalConsts ',CurtokenString,' ',TokenInfos[CurToken]);
    NextToken;
    if CurToken<>tkSemicolon then
      exit;
    NextToken;
    case CurToken of
    tkIdentifier:
      Done:=CheckVisibility(CurTokenString,AVisibility);
    tkSquaredBraceOpen:
      if msPrefixedAttributes in CurrentModeswitches then
        repeat
          ParseAttributes(AType,true);
          NextToken;
          Done:=false;
        until CurToken<>tkSquaredBraceOpen
      else
        Done:=true;
    else
      Done:=true;
    end;
    if Done then
      UngetToken;
  Until Done;
end;

procedure TPasParser.ParseClassMembers(AType: TPasClassType);
Type
  TSectionType = (stNone,stConst,stType,stVar,stClassVar);
Var
  CurVisibility : TPasMemberVisibility;
  CurSection : TSectionType;
  haveClass: boolean; // true means last token was class keyword
  IsMethodResolution: Boolean;
  LastToken: TToken;
  PropEl: TPasProperty;
  MethodRes: TPasMethodResolution;
begin
  CurSection:=stNone;
  haveClass:=false;
  if Assigned(FEngine) then
    CurVisibility:=FEngine.GetDefaultClassVisibility(AType)
  else
    CurVisibility := visPublic;
  LastToken:=CurToken;
  while (CurToken<>tkEnd) do
    begin
    //writeln('TPasParser.ParseClassMembers LastToken=',LastToken,' CurToken=',CurToken,' haveClass=',haveClass,' CurSection=',CurSection);
    case CurToken of
    tkType:
      begin
      if haveClass then
        ParseExcExpectedAorB('Procedure','Function');
      case AType.ObjKind of
      okClass,okObject,
      okClassHelper,okRecordHelper,okTypeHelper: ;
      else
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['TYPE',ObjKindNames[AType.ObjKind]]);
      end;
      CurSection:=stType;
      NextToken;
      ParseMembersLocalTypes(AType,CurVisibility);
      CurSection:=stNone;
      end;
    tkConst:
      begin
      if haveClass then
        ParseExcExpectedAorB('Procedure','Var');
      case AType.ObjKind of
      okClass,okObject,
      okClassHelper,okRecordHelper,okTypeHelper: ;
      else
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['CONST',ObjKindNames[AType.ObjKind]]);
      end;
      CurSection:=stConst;
      NextToken;
      ParseMembersLocalConsts(AType,CurVisibility);
      CurSection:=stNone;
      end;
    tkVar:
      if not (CurSection in [stVar,stClassVar]) then
        begin
        if (AType.ObjKind in okWithFields)
        or (haveClass and (AType.ObjKind in okAllHelpers)) then
          // ok
        else
          ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['VAR',ObjKindNames[AType.ObjKind]]);
        if LastToken=tkClass then
          CurSection:=stClassVar
        else
          CurSection:=stVar;
        end;
    tkIdentifier:
      if CheckVisibility(CurTokenString,CurVisibility,(AType.ObjKind=okObjcProtocol)) then
        CurSection:=stNone
      else
        begin
        if haveClass then
          begin
          if LastToken=tkclass then
            ParseExcExpectedAorB('Procedure','Function');
          end
        else
          SaveComments;
        Case CurSection of
        stNone,
        stVar:
          begin
          if not (AType.ObjKind in okWithFields) then
            ParseExc(nParserNoFieldsAllowed,SParserNoFieldsAllowedInX,[ObjKindNames[AType.ObjKind]]);
          ParseClassFields(AType,CurVisibility,CurSection=stClassVar);
          if Curtoken=tkEnd then // case Ta = Class x : String end;
            UngetToken;
          HaveClass:=False;
          end;
        stClassVar:
          begin
          if not (AType.ObjKind in okWithClassFields) then
            ParseExc(nParserNoFieldsAllowed,SParserNoFieldsAllowedInX,[ObjKindNames[AType.ObjKind]]);
          ParseClassFields(AType,CurVisibility,CurSection=stClassVar);
          HaveClass:=False;
          end;
        else
          Raise Exception.Create('Internal error 201704251415');
        end;
        end;
    tkConstructor,tkDestructor:
      begin
      curSection:=stNone;
      if not haveClass then
        SaveComments;
      case AType.ObjKind of
      okObject,okClass: ;
      okClassHelper,okTypeHelper,okRecordHelper:
        begin
        if (CurToken=tkdestructor) and not haveClass then
          ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['destructor',ObjKindNames[AType.ObjKind]]);
        end;
      else
        if CurToken=tkconstructor then
          ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['constructor',ObjKindNames[AType.ObjKind]])
        else
          ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['destructor',ObjKindNames[AType.ObjKind]]);
      end;
      ProcessMethod(AType,HaveClass,CurVisibility,false);
      haveClass:=False;
      end;
    tkProcedure,tkFunction:
      begin
      curSection:=stNone;
      IsMethodResolution:=false;
      if not haveClass then
        begin
        SaveComments;
        if AType.ObjKind=okClass then
          begin
          NextToken;
          if CurToken=tkIdentifier then
            begin
            NextToken;
            IsMethodResolution:=CurToken=tkDot;
            UngetToken;
            end;
          UngetToken;
          end;
        end;
      if IsMethodResolution then
        begin
        MethodRes:=ParseMethodResolution(AType);
        AType.Members.Add(MethodRes);
        Engine.FinishScope(stDeclaration,MethodRes);
        end
      else
        ProcessMethod(AType,HaveClass,CurVisibility,false);
      haveClass:=False;
      end;
    tkgeneric:
      begin
      if msDelphi in CurrentModeswitches then
        ParseExcSyntaxError; // inconsistency, tkGeneric should be in Scanner.NonTokens
      if haveClass and (LastToken=tkclass) then
        ParseExcTokenError('Generic Class');
      case AType.ObjKind of
      okClass,okObject,
      okClassHelper,okRecordHelper,okTypeHelper: ;
      else
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['generic',ObjKindNames[AType.ObjKind]]);
      end;
      SaveComments;
      CurSection:=stNone;
      NextToken;
      if CurToken=tkclass then
        begin
        haveClass:=true;
        NextToken;
        end
      else
        haveClass:=false;
      if not (CurToken in [tkprocedure,tkfunction]) then
        ParseExcExpectedAorB('Procedure','Function');
      ProcessMethod(AType,HaveClass,CurVisibility,true);
      end;
    tkclass:
      begin
      case AType.ObjKind of
      okClass,okObject,
      okClassHelper,okRecordHelper,okTypeHelper, okObjCClass, okObjcCategory, okObjcProtocol : ;
      else
        ParseExc(nParserXNotAllowedInY,SParserXNotAllowedInY,['CLASS',ObjKindNames[AType.ObjKind]]);
      end;

      SaveComments;
      HaveClass:=True;
      curSection:=stNone;
      end;
    tkProperty:
      begin
      curSection:=stNone;
      if not haveClass then
        SaveComments;
      ExpectIdentifier;
      PropEl:=ParseProperty(AType,CurtokenString,CurVisibility,HaveClass);
      AType.Members.Add(PropEl);
      Engine.FinishScope(stDeclaration,PropEl);
      HaveClass:=False;
      end;
    tkSquaredBraceOpen:
      if msPrefixedAttributes in CurrentModeswitches then
        ParseAttributes(AType,true)
      else
        CheckToken(tkIdentifier);
    else
      CheckToken(tkIdentifier);
    end;
    LastToken:=CurToken;
    NextToken;
    end;
end;

procedure TPasParser.DoParseClassType(AType: TPasClassType);
var
  s: String;
  Expr: TPasExpr;
begin
  if (CurToken=tkIdentifier) and (AType.ObjKind=okClass) then
    begin
    s := LowerCase(CurTokenString);
    if (s = 'sealed') or (s = 'abstract') then
      begin
      AType.Modifiers.Add(s);
      NextToken;
      end;
    end;
  // Parse ancestor list
  AType.IsForward:=(CurToken=tkSemiColon);
  if (CurToken=tkBraceOpen) then
    begin
    // read ancestor and interfaces
    if (AType.ObjKind=okRecordHelper)
        and ([msTypeHelpers,msDelphi]*Scanner.CurrentModeSwitches=[msDelphi]) then
      // Delphi does not support ancestors in record helpers
      CheckToken(tkend);
    NextToken;
    AType.AncestorType := ParseTypeReference(AType,false,Expr);
    if AType.ObjKind in [okClass,okObjCClass,okObjcProtocol] then
      while CurToken=tkComma do
        begin
        NextToken;
        AType.Interfaces.Add(ParseTypeReference(AType,false,Expr));
        end;
    CheckToken(tkBraceClose);
    NextToken;
    AType.IsShortDefinition:=(CurToken=tkSemicolon);
    end;
  if (AType.ObjKind in okAllHelpers) then
    begin
    CheckToken(tkfor);
    NextToken;
    AType.HelperForType:=ParseTypeReference(AType,false,Expr);
    end;
  Engine.FinishScope(stAncestors,AType);
  if AType.IsShortDefinition or AType.IsForward then
    UngetToken
  else
    begin
    if (AType.ObjKind in [okInterface,okDispInterface]) and (CurToken = tkSquaredBraceOpen) then
      begin
      NextToken;
      AType.GUIDExpr:=DoParseExpression(AType);
      if (CurToken<>tkSquaredBraceClose) then
        ParseExcTokenError(TokenInfos[tkSquaredBraceClose]);
      NextToken;
      end;
    ParseClassMembers(AType);
    end;
end;

function TPasParser.DoParseClassExternalHeader(AObjKind: TPasObjKind; out AExternalNameSpace, AExternalName: string): Boolean;
begin
  Result:=False;
  if ((aObjKind in [okObjcCategory,okObjcClass,okObjcProtocol]) or
      ((AObjKind in [okClass,okInterface]) and (msExternalClass in CurrentModeswitches)))
      and CurTokenIsIdentifier('external') then
    begin
    Result:=True;
    NextToken;
    if CurToken<>tkString then
      UnGetToken
    else
      AExternalNameSpace:=CurTokenString;
    if (aObjKind in [okObjcCategory,okObjcClass]) then
      begin
      // Name is optional in objcclass/category/protocol
      NextToken;
      if CurToken=tkBraceOpen then
        exit;
      UnGetToken;
      end;
    ExpectIdentifier;
    If Not CurTokenIsIdentifier('Name')  then
      ParseExc(nParserExpectedExternalClassName,SParserExpectedExternalClassName);
    NextToken;
    if not (CurToken in [tkChar,tkString]) then
      CheckToken(tkString);
    AExternalName:=CurTokenString;
    NextToken;
    end
  else
    begin
    AExternalNameSpace:='';
    AExternalName:='';
    end;
end;

procedure TPasParser.DoParseArrayType(ArrType: TPasArrayType);
var
  S: String;
  RangeExpr: TPasExpr;
begin
  NextToken;
  S:='';
  case CurToken of
    tkSquaredBraceOpen:
      begin
      // static array
      if ArrType.Parent is TPasArgument then
        ParseExcTokenError('of');
      repeat
        NextToken;
        if po_arrayrangeexpr in Options then
          begin
          RangeExpr:=DoParseExpression(ArrType);
          ArrType.AddRange(RangeExpr);
          end
        else if CurToken<>tkSquaredBraceClose then
          S:=S+CurTokenText;
        if CurToken=tkSquaredBraceClose then
          break
        else if CurToken=tkComma then
          continue
        else if po_arrayrangeexpr in Options then
          ParseExcTokenError(']');
      until false;
      ArrType.IndexRange:=S;
      ExpectToken(tkOf);
      ArrType.ElType := ParseType(ArrType,CurSourcePos);
      end;
    tkOf:
      begin
      NextToken;
      if CurToken = tkConst then
        // array of const
        begin
        if not (ArrType.Parent is TPasArgument) then
          ParseExcExpectedIdentifier;
        end
      else
        begin
        if (CurToken=tkarray) and (ArrType.Parent is TPasArgument) then
          ParseExcExpectedIdentifier;
        UngetToken;
        ArrType.ElType := ParseType(ArrType,CurSourcePos);
        end;
      end
    else
      ParseExc(nParserArrayTypeSyntaxError,SParserArrayTypeSyntaxError);
  end;
  // TPasProcedureType parsing has eaten the semicolon;
  // We know it was a local definition if the array def (ArrType) is the parent
  if (ArrType.ElType is TPasProcedureType) and (ArrType.ElType.Parent=ArrType) then
    UnGetToken;
end;

function TPasParser.ParseClassDecl(Parent: TPasElement;
  const NamePos: TPasSourcePos; const AClassName: String;
  AObjKind: TPasObjKind; PackMode: TPackMode): TPasType;

Var
  isExternal,ok: Boolean;
  AExternalNameSpace,AExternalName : String;
  PCT:TPasClassType;

begin
  NextToken;
  if (AObjKind = okClass) and (CurToken = tkOf) then
    begin
    Result := TPasClassOfType(CreateElement(TPasClassOfType, AClassName,
      Parent, NamePos));
    ok:=false;
    try
      ExpectIdentifier;
      UngetToken;                // Only names are allowed as following type
      TPasClassOfType(Result).DestType := ParseType(Result,CurSourcePos);
      Engine.FinishScope(stTypeDef,Result);
      ok:=true;
    finally
      if not ok then
        Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
    end;
    exit;
    end;
  isExternal:=DoParseClassExternalHeader(AObjKind,AExternalNameSpace,AExternalName);
  if AObjKind in okAllHelpers then
    begin
    if not CurTokenIsIdentifier('Helper') then
      ParseExcSyntaxError;
    NextToken;
    end;
  PCT := TPasClassType(CreateElement(TPasClassType, AClassName,
    Parent, NamePos));
  Result:=PCT;
  ok:=false;
  try
    PCT.HelperForType:=nil;
    PCT.IsExternal:=IsExternal;
    if AExternalName<>'' then
      PCT.ExternalName:={$ifdef pas2js}DeQuoteString{$else}AnsiDequotedStr{$endif}(AExternalName,'''');
    if AExternalNameSpace<>'' then
      PCT.ExternalNameSpace:={$ifdef pas2js}DeQuoteString{$else}AnsiDequotedStr{$endif}(AExternalNameSpace,'''');
    PCT.ObjKind := AObjKind;
    PCT.PackMode:=PackMode;
    if AObjKind=okInterface then
      begin
      if SameText(Scanner.CurrentValueSwitch[vsInterfaces],'CORBA') then
        PCT.InterfaceType:=citCorba;
      end;
    DoParseClassType(PCT);
    Engine.FinishScope(stTypeDef,Result);
    ok:=true;
  finally
    if not ok then
      begin
      PCT.Parent:=nil; // clear references from members to PCT
      Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      end;
  end;
end;

function TPasParser.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement): TPasElement;
begin
  Result := Engine.CreateElement(AClass, AName, AParent, visDefault, CurSourcePos);
end;

function TPasParser.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; const ASrcPos: TPasSourcePos): TPasElement;
begin
  Result := Engine.CreateElement(AClass, AName, AParent, visDefault, ASrcPos);
end;

function TPasParser.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility): TPasElement;
begin
  Result := Engine.CreateElement(AClass, AName, AParent, AVisibility,
    CurSourcePos);
end;

function TPasParser.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASrcPos: TPasSourcePos; TypeParams: TFPList): TPasElement;
begin
  if (ASrcPos.Row=0) and (ASrcPos.FileName='') then
    Result := Engine.CreateElement(AClass, AName, AParent, AVisibility, CurSourcePos, TypeParams)
  else
    Result := Engine.CreateElement(AClass, AName, AParent, AVisibility, ASrcPos, TypeParams);
end;

function TPasParser.CreatePrimitiveExpr(AParent: TPasElement;
  AKind: TPasExprKind; const AValue: String): TPrimitiveExpr;
begin
  Result:=TPrimitiveExpr(CreateElement(TPrimitiveExpr,'',AParent,CurTokenPos));
  Result.Kind:=AKind;
  Result.Value:=AValue;
end;

function TPasParser.CreateBoolConstExpr(AParent: TPasElement;
  AKind: TPasExprKind; const ABoolValue: Boolean): TBoolConstExpr;
begin
  Result:=TBoolConstExpr(CreateElement(TBoolConstExpr,'',AParent,CurTokenPos));
  Result.Kind:=AKind;
  Result.Value:=ABoolValue;
end;

function TPasParser.CreateBinaryExpr(AParent: TPasElement; xleft,
  xright: TPasExpr; AOpCode: TExprOpCode): TBinaryExpr;
begin
  Result:=CreateBinaryExpr(AParent,xleft,xright,AOpCode,CurSourcePos);
end;

function TPasParser.CreateBinaryExpr(AParent: TPasElement; xleft,
  xright: TPasExpr; AOpCode: TExprOpCode; const ASrcPos: TPasSourcePos
  ): TBinaryExpr;
begin
  Result:=TBinaryExpr(CreateElement(TBinaryExpr,'',AParent,ASrcPos));
  Result.OpCode:=AOpCode;
  Result.Kind:=pekBinary;
  if xleft<>nil then
    begin
    Result.left:=xleft;
    xleft.Parent:=Result;
    end;
  if xright<>nil then
    begin
    Result.right:=xright;
    xright.Parent:=Result;
    end;
end;

procedure TPasParser.AddToBinaryExprChain(var ChainFirst: TPasExpr;
  Element: TPasExpr; AOpCode: TExprOpCode; const ASrcPos: TPasSourcePos);
begin
  if Element=nil then
    exit
  else if ChainFirst=nil then
    begin
    // empty chain => simply add element, no need to create TBinaryExpr
    ChainFirst:=Element;
    end
  else
    begin
    // create new binary, old becomes left, Element right
    ChainFirst:=CreateBinaryExpr(ChainFirst.Parent,ChainFirst,Element,AOpCode,ASrcPos);
    end;
end;

{$IFDEF VerbosePasParser}
{AllowWriteln}
procedure TPasParser.WriteBinaryExprChain(Prefix: string; First, Last: TPasExpr
  );
var
  i: Integer;
begin
  if First=nil then
    begin
    write(Prefix,'First=nil');
    if Last=nil then
      writeln('=Last')
    else
      begin
      writeln(', ERROR Last=',Last.ClassName);
      ParseExcSyntaxError;
      end;
    end
  else if Last=nil then
    begin
    writeln(Prefix,'ERROR Last=nil First=',First.ClassName);
    ParseExcSyntaxError;
    end
  else if First is TBinaryExpr then
    begin
    i:=0;
    while First is TBinaryExpr do
      begin
      writeln(Prefix,Space(i*2),'bin.left=',TBinaryExpr(First).left.ClassName);
      if First=Last then break;
      First:=TBinaryExpr(First).right;
      inc(i);
      end;
    if First<>Last then
      begin
      writeln(Prefix,Space(i*2),'ERROR Last is not last in chain');
      ParseExcSyntaxError;
      end;
    if not (Last is TBinaryExpr) then
      begin
      writeln(Prefix,Space(i*2),'ERROR Last is not TBinaryExpr: ',Last.ClassName);
      ParseExcSyntaxError;
      end;
    if TBinaryExpr(Last).right=nil then
      begin
      writeln(Prefix,Space(i*2),'ERROR Last.right=nil');
      ParseExcSyntaxError;
      end;
    writeln(Prefix,Space(i*2),'last.right=',TBinaryExpr(Last).right.ClassName);
    end
  else if First=Last then
    writeln(Prefix,'First=Last=',First.ClassName)
  else
    begin
    write(Prefix,'ERROR First=',First.ClassName);
    if Last<>nil then
      writeln(' Last=',Last.ClassName)
    else
      writeln(' Last=nil');
    end;
end;
{AllowWriteln-}
{$ENDIF}

function TPasParser.CreateUnaryExpr(AParent: TPasElement; AOperand: TPasExpr;
  AOpCode: TExprOpCode): TUnaryExpr;
begin
  Result:=CreateUnaryExpr(AParent,AOperand,AOpCode,CurTokenPos);
end;

function TPasParser.CreateUnaryExpr(AParent: TPasElement; AOperand: TPasExpr;
  AOpCode: TExprOpCode; const ASrcPos: TPasSourcePos): TUnaryExpr;
begin
  Result:=TUnaryExpr(CreateElement(TUnaryExpr,'',AParent,ASrcPos));
  Result.Kind:=pekUnary;
  Result.Operand:=AOperand;
  Result.Operand.Parent:=Result;
  Result.OpCode:=AOpCode;
end;

function TPasParser.CreateArrayValues(AParent: TPasElement): TArrayValues;
begin
  Result:=TArrayValues(CreateElement(TArrayValues,'',AParent));
  Result.Kind:=pekListOfExp;
end;

function TPasParser.CreateFunctionType(const AName, AResultName: String;
  AParent: TPasElement; UseParentAsResultParent: Boolean;
  const NamePos: TPasSourcePos; TypeParams: TFPList): TPasFunctionType;
begin
  Result:=Engine.CreateFunctionType(AName,AResultName,
                                    AParent,UseParentAsResultParent,
                                    NamePos,TypeParams);
end;

function TPasParser.CreateInheritedExpr(AParent: TPasElement): TInheritedExpr;
begin
  Result:=TInheritedExpr(CreateElement(TInheritedExpr,'',AParent,CurTokenPos));
  Result.Kind:=pekInherited;
end;

function TPasParser.CreateSelfExpr(AParent: TPasElement): TSelfExpr;
begin
  Result:=TSelfExpr(CreateElement(TSelfExpr,'Self',AParent,CurTokenPos));
  Result.Kind:=pekSelf;
end;

function TPasParser.CreateNilExpr(AParent: TPasElement): TNilExpr;
begin
  Result:=TNilExpr(CreateElement(TNilExpr,'nil',AParent,CurTokenPos));
  Result.Kind:=pekNil;
end;

function TPasParser.CreateRecordValues(AParent: TPasElement): TRecordValues;
begin
  Result:=TRecordValues(CreateElement(TRecordValues,'',AParent));
  Result.Kind:=pekListOfExp;
end;

procedure TPasParser.ParseAdhocExpression(out NewExprElement: TPasExpr);
begin
  NewExprElement := DoParseExpression(nil);
end;

initialization
{$IFDEF HASFS}
  DefaultFileResolverClass:=TFileResolver;
{$ENDIF}
end.
