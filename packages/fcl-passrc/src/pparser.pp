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

{$mode objfpc}
{$h+}

unit PParser;

interface

uses SysUtils, Classes, PasTree, PScanner;

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
  SParserExpectedLBracketSemicolon = 'Expected "(" or ";"';
  SParserExpectedColonSemicolon = 'Expected ":" or ";"';
  SParserExpectedSemiColonEnd = 'Expected ";" or "End"';
  SParserExpectedConstVarID = 'Expected "const", "var" or identifier';
  SParserExpectedNested = 'Expected nested keyword';
  SParserExpectedColonID = 'Expected ":" or identifier';
  SParserSyntaxError = 'Syntax error';
  SParserTypeSyntaxError = 'Syntax error in type';
  SParserArrayTypeSyntaxError = 'Syntax error in array type';
  SParserInterfaceTokenError = 'Invalid token in interface section of unit';
  SParserImplementationTokenError = 'Invalid token in implementation section of unit';
  SParserInvalidTypeDef = 'Invalid type definition';
  SParserExpectedIdentifier = 'Identifier expected';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SParserNotAProcToken = 'Not a procedure or function token';
  SRangeExpressionExpected = 'Range expression expected';
  SParserExpectCase = 'Case label expression expected';
  SParserHelperNotAllowed = 'Helper objects not allowed for "%s"';
  SLogStartImplementation = 'Start parsing implementation section.';
  SLogStartInterface = 'Start parsing interface section';
  SParsingUsedUnit = 'Parsing used unit "%s" with commandLine "%s"';
  SParserNoConstructorAllowed = 'Constructors or Destructors are not allowed in Interfaces or Record helpers';
  SParserNoFieldsAllowed = 'Fields are not allowed in Interfaces';
  SParserInvalidRecordVisibility = 'Records can only have public and (strict) private as visibility specifiers';
  SErrRecordConstantsNotAllowed = 'Record constants not allowed at this location.';
  SErrRecordMethodsNotAllowed = 'Record methods not allowed at this location.';
  SErrRecordPropertiesNotAllowed = 'Record properties not allowed at this location.';
  SErrRecordVisibilityNotAllowed = 'Record visibilities not allowed at this location.';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
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
  public
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; const ASourceFilename: String;
      ASourceLinenumber: Integer): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;overload;
      virtual; abstract;
    function CreateFunctionType(const AName, AResultName: String; AParent: TPasElement;
      UseParentAsResultParent: Boolean; const ASourceFilename: String;
      ASourceLinenumber: Integer): TPasFunctionType;
    function FindElement(const AName: String): TPasElement; virtual; abstract;
    function FindModule(const AName: String): TPasModule; virtual;
    property Package: TPasPackage read FPackage;
    property InterfaceOnly : Boolean Read FInterfaceOnly Write FInterFaceOnly;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property ScannerLogEvents : TPScannerLogEvents Read FScannerLogEvents Write FScannerLogEvents;
    Property ParserLogEvents : TPParserLogEvents Read FPParserLogEvents Write FPParserLogEvents;
    Property OnLog : TPasParserLogHandler Read FOnLog Write FOnLog;
    Property CurrentParser : TPasParser Read FCurrentParser;
    Property NeedComments : Boolean Read FNeedComments Write FNeedComments;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

  EParserError = class(Exception)
  private
    FFilename: String;
    FRow, FColumn: Integer;
  public
    constructor Create(const AReason, AFilename: String;
      ARow, AColumn: Integer);
    property Filename: String read FFilename;
    property Row: Integer read FRow;
    property Column: Integer read FColumn;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TProcType = (ptProcedure, ptFunction, ptOperator, ptClassOperator, ptConstructor, ptDestructor,
               ptClassProcedure, ptClassFunction, ptClassConstructor, ptClassDestructor);


  TExprKind = (ek_Normal, ek_PropertyIndex);
  TIndentAction = (iaNone,iaIndent,iaUndent);

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ParseSource(AEngine: TPasTreeContainer;
  const FPCCommandLine, OSTarget, CPUTarget: String): TPasModule;


implementation

uses Classes;

var
  IsIdentStart: array[char] of boolean;

<<<<<<< HEAD
<<<<<<< HEAD
const
  WhitespaceTokensToIgnore = [tkWhitespace, tkComment, tkLineEnding, tkTab];
=======
  TDeclType = (declNone, declConst, declResourcestring, declType, declVar, declThreadvar, declProperty);
>>>>>>> graemeg/fixes_2_2
=======
  TDeclType = (declNone, declConst, declResourcestring, declType, declVar, declThreadvar, declProperty);
>>>>>>> origin/fixes_2_2

type
  TDeclType = (declNone, declConst, declResourcestring, declType, declVar, declThreadvar, declProperty);

  TProcType = (ptProcedure, ptFunction, ptOperator, ptConstructor, ptDestructor,
               ptClassProcedure, ptClassFunction);

               
  TExprKind = (ek_Normal, ek_PropertyIndex);

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
  { TPasParser }

  TPasParser = class
  private
    FCurModule: TPasModule;
    FFileResolver: TBaseFileResolver;
    FLogEvents: TPParserLogEvents;
    FOnLog: TPasParserLogHandler;
    FOptions: TPOptions;
    FScanner: TPascalScanner;
    FEngine: TPasTreeContainer;
    FCurToken: TToken;
    FCurTokenString: String;
    FCurComments : TStrings;
    FSavedComments : String;
    // UngetToken support:
    FTokenBuffer: array[0..1] of TToken;
    FTokenStringBuffer: array[0..1] of String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FCommentsBuffer: array[0..1] of TStrings;
    FTokenBufferIndex: Integer; // current index in FTokenBuffer
    FTokenBufferSize: Integer; // maximum valid index in FTokenBuffer
    FDumpIndent : String;
    function CheckOverloadList(AList: TFPList; AName: String; out OldMember: TPasElement): TPasOverloadedProc;
    procedure DumpCurToken(Const Msg : String; IndentAction : TIndentAction = iaNone);
    function GetVariableModifiers(Out VarMods : TVariableModifiers; Out Libname,ExportName : string): string;
    function GetVariableValueAndLocation(Parent : TPasElement; Out Value : TPasExpr; Out Location: String): Boolean;
    procedure HandleProcedureModifier(Parent: TPasElement; pm : TProcedureModifier);
    procedure ParseAsmBlock(AsmBlock: TPasImplAsmStatement);
    procedure ParseClassLocalConsts(AType: TPasClassType; AVisibility: TPasMemberVisibility);
    procedure ParseClassLocalTypes(AType: TPasClassType; AVisibility: TPasMemberVisibility);
    procedure ParseVarList(Parent: TPasElement; VarList: TFPList; AVisibility: TPasMemberVisibility; Full: Boolean);
    procedure SetOptions(AValue: TPOptions);
  protected
    Function SaveComments : String;
    Function SaveComments(Const AValue : String) : String;
    function LogEvent(E : TPParserLogEvent) : Boolean; inline;
    Procedure DoLog(Const Msg : String; SkipSourceInfo : Boolean = False);overload;
    Procedure DoLog(Const Fmt : String; Args : Array of const;SkipSourceInfo : Boolean = False);overload;
    function GetProcTypeFromToken(tk: TToken; IsClass: Boolean=False ): TProcType;
    procedure ParseRecordFieldList(ARec: TPasRecordType; AEndToken: TToken; AllowMethods : Boolean);
    procedure ParseRecordVariantParts(ARec: TPasRecordType; AEndToken: TToken);
    function GetProcedureClass(ProcType : TProcType): TPTreeElement;
    procedure ParseClassFields(AType: TPasClassType; const AVisibility: TPasMemberVisibility; IsClassField : Boolean);
    procedure ParseClassMembers(AType: TPasClassType);
    procedure ProcessMethod(AType: TPasClassType; IsClass : Boolean; AVisibility : TPasMemberVisibility);
    procedure ReadGenericArguments(List : TFPList;Parent : TPasElement);
    function CheckProcedureArgs(Parent: TPasElement; Args: TFPList; Mandatory: Boolean): boolean;
    function CheckVisibility(S: String; var AVisibility: TPasMemberVisibility): Boolean;
    procedure ParseExc(const Msg: String);
    procedure ParseExc(const Fmt: String; Args : Array of const);
    function OpLevel(t: TToken): Integer;
    Function TokenToExprOp (AToken : TToken) : TExprOpCode;
    function CreateElement(AClass: TPTreeElement; const AName: String; AParent: TPasElement): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility): TPasElement;overload;
    function CreateFunctionType(const AName, AResultName: String; AParent: TPasElement;
             UseParentAsResultParent: Boolean): TPasFunctionType;
    Function IsCurTokenHint(out AHint : TPasMemberHint) : Boolean; overload;
    Function IsCurTokenHint: Boolean; overload;
    Function TokenIsCallingConvention(S : String; out CC : TCallingConvention) : Boolean; virtual;
    Function TokenIsProcedureModifier(Parent : TPasElement; S : String; Out Pm : TProcedureModifier) : Boolean; virtual;
    Function CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;
    function ParseParams(AParent : TPasElement;paramskind: TPasExprKind): TParamsExpr;
    function ParseExpIdent(AParent : TPasElement): TPasExpr;
    procedure DoParseClassType(AType: TPasClassType);
    function DoParseExpression(Aparent : TPaselement;InitExpr: TPasExpr=nil): TPasExpr;
    function DoParseConstValueExpression(AParent : TPasElement): TPasExpr;
    function CheckPackMode: TPackMode;
    // Overload handling
    procedure AddProcOrFunction(Decs: TPasDeclarations; AProc: TPasProcedure);
    function  CheckIfOverloaded(AParent: TPasElement; const AName: String): TPasElement;
  public
    constructor Create(AScanner: TPascalScanner; AFileResolver: TBaseFileResolver;  AEngine: TPasTreeContainer);
    Destructor Destroy; override;
    // General parsing routines
    function CurTokenName: String;
    function CurTokenText: String;
    Function CurComments : TStrings;
    Function SavedComments : String;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FTokenBufferIndex: Integer; // current index in FTokenBuffer
    FTokenBufferSize: Integer; // maximum valid index in FTokenBuffer
    procedure ParseExc(const Msg: String);
  protected
    function OpLevel(t: TToken): Integer;
    Function TokenToExprOp (AToken : TToken) : TExprOpCode;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement): TPasElement;overload;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility): TPasElement;overload;
<<<<<<< HEAD
<<<<<<< HEAD
    Function IsCurTokenHint(out AHint : TPasMemberHint) : Boolean; overload;
    Function IsCurTokenHint: Boolean; overload;
    Function CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;

    function ParseParams(AParent : TPasElement;paramskind: TPasExprKind): TParamsExpr;
    function ParseExpIdent(AParent : TPasElement): TPasExpr;
=======
    Function CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;
>>>>>>> graemeg/fixes_2_2
=======
    Function CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;
>>>>>>> origin/fixes_2_2
  public
    Options : set of TPOptions;
    CurModule: TPasModule;
    constructor Create(AScanner: TPascalScanner; AFileResolver: TFileResolver;
      AEngine: TPasTreeContainer);
    function CurTokenName: String;
    function CurTokenText: String;
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
    procedure NextToken; // read next non whitespace, non space
    procedure UngetToken;
    procedure CheckToken(tk: TToken);
    procedure ExpectToken(tk: TToken);
    function ExpectIdentifier: String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function CurTokenIsIdentifier(Const S : String) : Boolean;
    // Expression parsing
    function isEndOfExp: Boolean;
    // Type declarations
    function ParseComplexType(Parent : TPasElement = Nil): TPasType;
    function ParseTypeDecl(Parent: TPasElement): TPasType;
    function ParseType(Parent: TPasElement; Const TypeName : String = '';Full : Boolean = False): TPasType;
    function ParseProcedureType(Parent: TPasElement; const TypeName: String; const PT: TProcType): TPasProcedureType;
    function ParseStringType(Parent: TPasElement; const TypeName: String): TPasAliasType;
    function ParseSimpleType(Parent: TPasElement; Const TypeName: String; IsFull : Boolean = False): TPasType;
    function ParseAliasType(Parent: TPasElement; Const TypeName: String): TPasTypeAliasType;
    function ParsePointerType(Parent: TPasElement; Const TypeName: String): TPasPointerType;
    Function ParseArrayType(Parent : TPasElement; Const TypeName : String; PackMode : TPackMode) : TPasArrayType;
    Function ParseFileType(Parent : TPasElement; Const TypeName  : String) : TPasFileType;
    Function ParseRecordDecl(Parent: TPasElement; Const TypeName : string; const Packmode : TPackMode = pmNone) : TPasRecordType;
    function ParseEnumType(Parent: TPasElement; const TypeName: String): TPasEnumType;
    function ParseSetType(Parent: TPasElement; const TypeName: String ): TPasSetType;
    function ParseSpecializeType(Parent: TPasElement; Const TypeName: String): TPasClassType;
    Function ParseClassDecl(Parent: TPasElement; const AClassName: String;   AObjKind: TPasObjKind; PackMode : TPackMode= pmNone): TPasType;
    Function ParseProperty(Parent : TPasElement; Const AName : String; AVisibility : TPasMemberVisibility) : TPasProperty;
    function ParseRangeType(AParent: TPasElement; Const TypeName: String; Full : Boolean = True): TPasRangeType;
    procedure ParseExportDecl(Parent: TPasElement; List: TFPList);
    // Constant declarations
    function ParseConstDecl(Parent: TPasElement): TPasConst;
    function ParseResourcestringDecl(Parent: TPasElement): TPasResString;
    // Variable handling. This includes parts of records
    procedure ParseVarDecl(Parent: TPasElement; List: TFPList);
    procedure ParseInlineVarDecl(Parent: TPasElement; List: TFPList;  AVisibility : TPasMemberVisibility  = visDefault; ClosingBrace: Boolean = False);
    // Main scope parsing
    procedure ParseMain(var Module: TPasModule);
    procedure ParseUnit(var Module: TPasModule);
    procedure ParseProgram(var Module: TPasModule; SkipHeader : Boolean = False);
    procedure ParseLibrary(var Module: TPasModule);
    procedure ParseUsesList(ASection: TPasSection);
    procedure ParseInterface;
    procedure ParseImplementation;
    procedure ParseInitialization;
    procedure ParseFinalization;
    procedure ParseDeclarations(Declarations: TPasDeclarations);
    procedure ParseStatement(Parent: TPasImplBlock;  out NewImplElement: TPasImplElement);
    procedure ParseLabels(AParent: TPasElement);
    procedure ParseProcBeginBlock(Parent: TProcedureBody);
    // Function/Procedure declaration
    function  ParseProcedureOrFunctionDecl(Parent: TPasElement; ProcType: TProcType;AVisibility : TPasMemberVisibility = VisDefault): TPasProcedure;
    procedure ParseArgList(Parent: TPasElement; Args: TFPList; EndToken: TToken);
    procedure ParseProcedureOrFunctionHeader(Parent: TPasElement; Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);
    procedure ParseProcedureBody(Parent: TPasElement);
    // Properties for external access
    property FileResolver: TBaseFileResolver read FFileResolver;
=======

    function ParseType(Parent: TPasElement; Prefix : String): TPasType;overload;
    function ParseType(Parent: TPasElement): TPasType;overload;
    function ParseComplexType(Parent : TPasElement = Nil): TPasType;
=======
=======
>>>>>>> origin/fixes_2_2

    function ParseType(Parent: TPasElement; Prefix : String): TPasType;overload;
    function ParseType(Parent: TPasElement): TPasType;overload;
    function ParseComplexType: TPasType;
>>>>>>> graemeg/fixes_2_2
    procedure ParseArrayType(Element: TPasArrayType);
    procedure ParseFileType(Element: TPasFileType);
    function isEndOfExp: Boolean;
    function DoParseExpression(Aparent : TPaselement;InitExpr: TPasExpr=nil): TPasExpr;
    function DoParseConstValueExpression(AParent : TPasElement): TPasExpr;
    function ParseExpression(AParent : TPaselement; Kind: TExprKind=ek_Normal): String;
    function ParseCommand: String; // single, not compound command like begin..end
    procedure AddProcOrFunction(Declarations: TPasDeclarations; AProc: TPasProcedure);
    function CheckIfOverloaded(AOwner: TPasClassType;
      const AName: String): TPasElement;

    procedure ParseMain(var Module: TPasModule);
    procedure ParseUnit(var Module: TPasModule);
    procedure ParseProgram(var Module: TPasModule);
    procedure ParseInterface;
    procedure ParseImplementation;
    procedure ParseInitialization;
    procedure ParseFinalization;
    procedure ParseDeclarations(Declarations: TPasDeclarations);
    procedure ParseUsesList(ASection: TPasSection);
    function ParseConstDecl(Parent: TPasElement): TPasConst;
    function ParseResourcestringDecl(Parent: TPasElement): TPasResString;
    function ParseTypeDecl(Parent: TPasElement): TPasType;
    procedure ParseInlineVarDecl(Parent: TPasElement; VarList: TList);overload;
    procedure ParseInlineVarDecl(Parent: TPasElement; VarList: TList;
      AVisibility : TPasMemberVisibility; ClosingBrace: Boolean);overload;
    procedure ParseVarDecl(Parent: TPasElement; List: TList);
    procedure ParseArgList(Parent: TPasElement; Args: TList; EndToken: TToken);
    procedure ParseProcedureOrFunctionHeader(Parent: TPasElement;
      Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);
    procedure ParseProcedureBody(Parent: TPasElement);
    function ParseProcedureOrFunctionDecl(Parent: TPasElement;
      ProcType: TProcType): TPasProcedure;
    procedure ParseRecordDecl(Parent: TPasRecordType; IsNested: Boolean);   // !!!: Optimize this. We have 3x the same wrapper code around it.
    function ParseClassDecl(Parent: TPasElement; const AClassName: String;
      AObjKind: TPasObjKind): TPasType;
    procedure ParseProperty(Element:TPasElement);
    procedure ParseProcBeginBlock(Parent: TProcedureBody);
    procedure ParseStatement(Parent: TPasImplBlock;
                             out NewImplElement: TPasImplElement);
    procedure ParseLabels(AParent: TPasElement);

    property FileResolver: TFileResolver read FFileResolver;
>>>>>>> graemeg/cpstrnew
    property Scanner: TPascalScanner read FScanner;
    property Engine: TPasTreeContainer read FEngine;
    property CurToken: TToken read FCurToken;
    property CurTokenString: String read FCurTokenString;
    Property Options : TPOptions Read FOptions Write SetOptions;
    Property CurModule : TPasModule Read FCurModule;
    Property LogEvents : TPParserLogEvents Read FLogEvents Write FLogEvents;
    Property OnLog : TPasParserLogHandler Read FOnLog Write FOnLog;
  end;

function ParseSource(AEngine: TPasTreeContainer;
                     const FPCCommandLine, OSTarget, CPUTarget: String;
                     UseStreams  : Boolean = False): TPasModule;
Function IsHintToken(T : String; Out AHint : TPasMemberHint) : boolean;
Function IsModifier(S : String; Out Pm : TProcedureModifier) : Boolean;
Function IsCallingConvention(S : String; out CC : TCallingConvention) : Boolean;
Function TokenToAssignKind( tk : TToken) : TAssignKind;

implementation

const
  WhitespaceTokensToIgnore = [tkWhitespace, tkComment, tkLineEnding, tkTab];

type
  TDeclType = (declNone, declConst, declResourcestring, declType,
               declVar, declThreadvar, declProperty, declExports);

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
         = ('','register','pascal','cdecl','stdcall','oldfpccall','safecall','syscall');
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


Function IsModifier(S : String; Out Pm : TProcedureModifier) : Boolean;


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
  const FPCCommandLine, OSTarget, CPUTarget: String;
  UseStreams  : Boolean = False): TPasModule;
var
  FileResolver: TFileResolver;
  Parser: TPasParser;
  Start, CurPos: PChar;
  Filename: String;
  Scanner: TPascalScanner;

  procedure ProcessCmdLinePart;
  var
    l: Integer;
    s: String;
  begin
    l := CurPos - Start;
    SetLength(s, l);
    if l > 0 then
      Move(Start^, s[1], l)
    else
      exit;
    if (s[1] = '-') and (length(s)>1) then
    begin
      case s[2] of
        'd': // -d define
          Scanner.AddDefine(UpperCase(Copy(s, 3, Length(s))));
        'F': // -F
          if (length(s)>2) and (s[3] = 'i') then // -Fi include path
            FileResolver.AddIncludePath(Copy(s, 4, Length(s)));
        'I': // -I include path
          FileResolver.AddIncludePath(Copy(s, 3, Length(s)));
        'S': // -S mode
          if  (length(s)>2) then
            case S[3] of
              'c' : Scanner.Options:=Scanner.Options+[po_cassignments];
              'd','2' : Parser.Options:=Parser.Options+[po_delphi];
            end;
      end;
    end else
      if Filename <> '' then
        raise Exception.Create(SErrMultipleSourceFiles)
      else
        Filename := s;
  end;

<<<<<<< HEAD
var
  s: String;
begin
  Result := nil;
  FileResolver := nil;
  Scanner := nil;
  Parser := nil;
  try
    FileResolver := TFileResolver.Create;
    FileResolver.UseStreams:=UseStreams;
    Scanner := TPascalScanner.Create(FileResolver);
    Scanner.AddDefine('FPK');
    Scanner.AddDefine('FPC');
    SCanner.LogEvents:=AEngine.ScannerLogEvents;
    SCanner.OnLog:=AEngine.Onlog;

    // TargetOS
    s := UpperCase(OSTarget);
    Scanner.AddDefine(s);
    if s = 'LINUX' then
      Scanner.AddDefine('UNIX')
    else if s = 'FREEBSD' then
    begin
      Scanner.AddDefine('BSD');
      Scanner.AddDefine('UNIX');
    end else if s = 'NETBSD' then
    begin
      Scanner.AddDefine('BSD');
      Scanner.AddDefine('UNIX');
    end else if s = 'SUNOS' then
    begin
      Scanner.AddDefine('SOLARIS');
      Scanner.AddDefine('UNIX');
    end else if s = 'GO32V2' then
      Scanner.AddDefine('DPMI')
    else if s = 'BEOS' then
      Scanner.AddDefine('UNIX')
    else if s = 'QNX' then
      Scanner.AddDefine('UNIX')
    else if s = 'AROS' then
      Scanner.AddDefine('HASAMIGA')
    else if s = 'MORPHOS' then
      Scanner.AddDefine('HASAMIGA')
    else if s = 'AMIGA' then
      Scanner.AddDefine('HASAMIGA');

    // TargetCPU
    s := UpperCase(CPUTarget);
    Scanner.AddDefine('CPU'+s);
    if (s='x86_64') then
      Scanner.AddDefine('CPU64')
    else
      Scanner.AddDefine('CPU32');

    Parser := TPasParser.Create(Scanner, FileResolver, AEngine);
    Filename := '';
    Parser.LogEvents:=AEngine.ParserLogEvents;
    Parser.OnLog:=AEngine.Onlog;

    if FPCCommandLine<>'' then
      begin
        Start := @FPCCommandLine[1];
        CurPos := Start;
        while CurPos[0] <> #0 do
        begin
          if CurPos[0] = ' ' then
          begin
            ProcessCmdLinePart;
            Start := CurPos + 1;
          end;
          Inc(CurPos);
        end;
        ProcessCmdLinePart;
      end;

    if Filename = '' then
      raise Exception.Create(SErrNoSourceGiven);
    FileResolver.AddIncludePath(ExtractFilePath(FileName));
    Scanner.OpenFile(Filename);
    Parser.ParseMain(Result);
  finally
    Parser.Free;
    Scanner.Free;
    FileResolver.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ ---------------------------------------------------------------------
  TPasTreeContainer
  ---------------------------------------------------------------------}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TPasTreeContainer.CreateElement(AClass: TPTreeElement;
  const AName: String; AParent: TPasElement; const ASourceFilename: String;
  ASourceLinenumber: Integer): TPasElement;
begin
  Result := CreateElement(AClass, AName, AParent, visDefault, ASourceFilename,
    ASourceLinenumber);
end;

function TPasTreeContainer.CreateFunctionType(const AName, AResultName: String;
  AParent: TPasElement; UseParentAsResultParent: Boolean;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasFunctionType;
var
  ResultParent: TPasElement;
begin
  Result := TPasFunctionType(CreateElement(TPasFunctionType, AName, AParent,
    ASourceFilename, ASourceLinenumber));

  if UseParentAsResultParent then
    ResultParent := AParent
  else
    ResultParent := Result;

  TPasFunctionType(Result).ResultEl :=
    TPasResultElement(CreateElement(TPasResultElement, AResultName, ResultParent,
    ASourceFilename, ASourceLinenumber));
end;

function TPasTreeContainer.FindModule(const AName: String): TPasModule;
begin
  Result := nil;
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

procedure TPasParser.ParseExc(const Msg: String);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName, Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn])
    {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},
=======
  raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},
>>>>>>> graemeg/cpstrnew
=======
  raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},
>>>>>>> graemeg/cpstrnew
=======
  raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},
>>>>>>> graemeg/cpstrnew
=======
  raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},
>>>>>>> origin/cpstrnew
    Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
end;

procedure TPasParser.ParseExc(const Fmt: String; Args: array of const);
begin
  ParseExc(Format(Fmt,Args));
end;

constructor TPasParser.Create(AScanner: TPascalScanner;
  AFileResolver: TBaseFileResolver; AEngine: TPasTreeContainer);
begin
  inherited Create;
  FScanner := AScanner;
  FFileResolver := AFileResolver;
  FEngine := AEngine;
  FCommentsBuffer[0]:=TStringList.Create;
  FCommentsBuffer[1]:=TStringList.Create;
  if Assigned(FEngine) then
    begin
    FEngine.FCurrentParser:=Self;
    If FEngine.NeedComments then
      FScanner.SkipComments:=Not FEngine.NeedComments;
    end;
end;

destructor TPasParser.Destroy;
begin
  FreeAndNil(FCommentsBuffer[0]);
  FreeAndNil(FCommentsBuffer[1]);
  if Assigned(FEngine) then
    FEngine.FCurrentParser:=Nil;
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
  Result:=FCurComments;
end;

function TPasParser.SavedComments: String;
begin
  Result:=FSavedComments;
end;

procedure TPasParser.NextToken;

Var
  T : TStrings;
begin
  if FTokenBufferIndex < FTokenBufferSize then
  begin
    // Get token from buffer
    FCurToken := FTokenBuffer[FTokenBufferIndex];
    FCurTokenString := FTokenStringBuffer[FTokenBufferIndex];
    FCurComments:=FCommentsBuffer[FTokenBufferIndex];
    Inc(FTokenBufferIndex);
    //writeln('TPasParser.NextToken From Buf ',CurTokenText,' id=',FTokenBufferIndex);
  end else
  begin
    { We have to fetch a new token. But first check, wether there is space left
      in the token buffer.}
    if FTokenBufferSize = 2 then
      begin
      FTokenBuffer[0] := FTokenBuffer[1];
      FTokenStringBuffer[0] := FTokenStringBuffer[1];
      T:=FCommentsBuffer[0];
      FCommentsBuffer[0]:=FCommentsBuffer[1];
      FCommentsBuffer[1]:=T;
      Dec(FTokenBufferSize);
      Dec(FTokenBufferIndex);
      end;
    // Fetch new token
    try
      FCommentsBuffer[FTokenBufferSize].Clear;
      repeat
        FCurToken := Scanner.FetchToken;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if FCurToken=tkComment then
          FCommentsBuffer[FTokenBufferSize].Add(Scanner.CurTokenString);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      until not (FCurToken in WhitespaceTokensToIgnore);
    except
      on e: EScannerError do
        raise EParserError.Create(e.Message,
          Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
    end;
    FCurTokenString := Scanner.CurTokenString;
    FTokenBuffer[FTokenBufferSize] := FCurToken;
    FTokenStringBuffer[FTokenBufferSize] := FCurTokenString;
    FCurComments:=FCommentsBuffer[FTokenBufferSize];
    Inc(FTokenBufferSize);
    Inc(FTokenBufferIndex);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  //  writeln('TPasParser.NextToken New ',CurTokenText,' id=',FTokenBufferIndex,' comments = ',FCurComments.text);
=======
    //writeln('TPasParser.NextToken New ',CurTokenText,' id=',FTokenBufferIndex);
>>>>>>> graemeg/cpstrnew
=======
    //writeln('TPasParser.NextToken New ',CurTokenText,' id=',FTokenBufferIndex);
>>>>>>> graemeg/cpstrnew
=======
    //writeln('TPasParser.NextToken New ',CurTokenText,' id=',FTokenBufferIndex);
>>>>>>> graemeg/cpstrnew
=======
    //writeln('TPasParser.NextToken New ',CurTokenText,' id=',FTokenBufferIndex);
>>>>>>> origin/cpstrnew
  end;
end;

procedure TPasParser.UngetToken;

begin
  if FTokenBufferIndex = 0 then
    ParseExc(SParserUngetTokenError)
  else begin
    Dec(FTokenBufferIndex);
    if FTokenBufferIndex>0 then
    begin
      FCurToken := FTokenBuffer[FTokenBufferIndex-1];
      FCurTokenString := FTokenStringBuffer[FTokenBufferIndex-1];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FCurComments:=FCommentsBuffer[FTokenBufferIndex-1];
    end else begin
      FCurToken := tkWhitespace;
      FCurTokenString := '';
      FCurComments.Clear;
    end;
    //writeln('TPasParser.UngetToken ',CurTokenText,' id=',FTokenBufferIndex);
  end;
end;

procedure TPasParser.CheckToken(tk: TToken);
begin
  if (CurToken<>tk) then
    ParseExc(Format(SParserExpectTokenError, [TokenInfos[tk]]));
=======
    end else begin
      FCurToken := tkWhitespace;
      FCurTokenString := '';
    end;
    //writeln('TPasParser.UngetToken ',CurTokenText,' id=',FTokenBufferIndex);
  end;
>>>>>>> graemeg/cpstrnew
=======
    end else begin
      FCurToken := tkWhitespace;
      FCurTokenString := '';
    end;
    //writeln('TPasParser.UngetToken ',CurTokenText,' id=',FTokenBufferIndex);
  end;
>>>>>>> graemeg/cpstrnew
=======
    end else begin
      FCurToken := tkWhitespace;
      FCurTokenString := '';
    end;
    //writeln('TPasParser.UngetToken ',CurTokenText,' id=',FTokenBufferIndex);
  end;
>>>>>>> graemeg/cpstrnew
=======
    end else begin
      FCurToken := tkWhitespace;
      FCurTokenString := '';
    end;
    //writeln('TPasParser.UngetToken ',CurTokenText,' id=',FTokenBufferIndex);
  end;
>>>>>>> origin/cpstrnew
end;


procedure TPasParser.ExpectToken(tk: TToken);
begin
  NextToken;
  CheckToken(tk);
end;

function TPasParser.ExpectIdentifier: String;
begin
  ExpectToken(tkIdentifier);
  Result := CurTokenString;
end;

function TPasParser.CurTokenIsIdentifier(const S: String): Boolean;
begin
  Result:=(Curtoken=tkidentifier) and (CompareText(S,CurtokenText)=0);
end;

<<<<<<< HEAD

function TPasParser.IsCurTokenHint(out AHint: TPasMemberHint): Boolean;
begin
  Result:=CurToken=tklibrary;
  if Result then
    AHint:=hLibrary
  else if (CurToken=tkIdentifier) then
    Result:=IsHintToken(CurTokenString,ahint);
end;

<<<<<<< HEAD
function TPasParser.IsCurTokenHint: Boolean;
var
  dummy : TPasMemberHint;
begin
  Result:=IsCurTokenHint(dummy);
=======
Function TPasParser.IsCurTokenHint(out AHint : TPasMemberHint) : Boolean;
Var
  T : string;
begin
  if CurToken=tklibrary then
    begin
    AHint:=hLibrary;
    Result:=True;
    end
  else if CurToken=tkIdentifier then
    begin
      T:=LowerCase(CurTokenString);
      Result:=True;
      if (T='deprecated') then ahint:=hDeprecated
      else if (T='platform') then ahint:=hPlatform
      else if (T='experimental') then ahint:=hExperimental
      else if (T='unimplemented') then ahint:=hUnimplemented
      else Result:=False;
    end
  else
    Result:=False;
end;

Function TPasParser.IsCurTokenHint: Boolean;
var
  dummy : TPasMemberHint;
begin
  Result:=IsCurTokenHint(dummy);
end;


=======
>>>>>>> graemeg/fixes_2_2
Function TPasParser.CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;

Var
  Found : Boolean;
<<<<<<< HEAD
  h : TPasMemberHint;
  
=======

>>>>>>> graemeg/fixes_2_2
begin
  Result:=[];
  Repeat
    NextToken;
<<<<<<< HEAD
    Found:=IsCurTokenHint(h);
    If Found then
      Include(Result,h)
=======
    Found:=CompareText(CurTokenString,'deprecated')=0;
    If Found then
      Include(Result,hDeprecated)
    else
     begin
     Found:=CompareText(CurTokenString,'library')=0;
     if Found then
       Include(Result,hLibrary)
     else 
       begin
       Found:=CompareText(CurTokenString,'platform')=0;
       If Found then
         Include(Result,hPlatform);
       end;
     end;
>>>>>>> graemeg/fixes_2_2
  Until Not Found;
  UnGetToken;
  If Assigned(Element) then
    Element.Hints:=Result;
  if ExpectSemiColon then
<<<<<<< HEAD
    ExpectToken(tkSemiColon);
>>>>>>> origin/cpstrnew
end;
=======
    ExpectToken(tkSemiColon);  
end;

Function TPasParser.CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;

Var
  Found : Boolean;

begin
  Result:=[];
  Repeat
    NextToken;
    Found:=CompareText(CurTokenString,'deprecated')=0;
    If Found then
      Include(Result,hDeprecated)
    else
     begin
     Found:=CompareText(CurTokenString,'library')=0;
     if Found then
       Include(Result,hLibrary)
     else 
       begin
       Found:=CompareText(CurTokenString,'platform')=0;
       If Found then
         Include(Result,hPlatform);
       end;
     end;
  Until Not Found;
  UnGetToken;
  If Assigned(Element) then
    Element.Hints:=Result;
  if ExpectSemiColon then
    ExpectToken(tkSemiColon);  
end;

function TPasParser.ParseType(Parent: TPasElement; Prefix : String): TPasType;
>>>>>>> graemeg/fixes_2_2

function TPasParser.TokenIsCallingConvention(S: String; out
  CC: TCallingConvention): Boolean;
begin
  Result:=IsCallingConvention(S,CC);
end;

<<<<<<< HEAD
function TPasParser.TokenIsProcedureModifier(Parent: TPasElement; S: String;
  out Pm: TProcedureModifier): Boolean;
begin
  Result:=IsModifier(S,PM);
  if result and (pm in [pmPublic,pmForward]) then
    begin
    While (Parent<>Nil) and Not ((Parent is TPasClassType) or (Parent is TPasRecordType)) do
     Parent:=Parent.Parent;
    Result:=Not Assigned(Parent);
=======
  procedure ParseRange;
  begin
    Result := TPasRangeType(CreateElement(TPasRangeType, '', Parent));
    try
      TPasRangeType(Result).RangeStart := ParseExpression(Result);
      ExpectToken(tkDotDot);
      TPasRangeType(Result).RangeEnd := ParseExpression(Result);
    except
      Result.Free;
      raise;
>>>>>>> origin/cpstrnew
    end;
end;

<<<<<<< HEAD

function TPasParser.CheckHint(Element: TPasElement; ExpectSemiColon: Boolean
  ): TPasMemberHints;
=======
Function TPasParser.IsCurTokenHint(out AHint : TPasMemberHint) : Boolean;
Var
  T : string;
begin
  if CurToken=tklibrary then
    begin
    AHint:=hLibrary;
    Result:=True;
    end
  else if CurToken=tkIdentifier then
    begin
      T:=LowerCase(CurTokenString);
      Result:=True;
      if (T='deprecated') then ahint:=hDeprecated
      else if (T='platform') then ahint:=hPlatform
      else if (T='experimental') then ahint:=hExperimental
      else if (T='unimplemented') then ahint:=hUnimplemented
      else Result:=False;
    end
  else
    Result:=False;
end;

Function TPasParser.IsCurTokenHint: Boolean;
var
<<<<<<< HEAD
  dummy : TPasMemberHint;
=======
  Name, s: String;
  EnumValue: TPasEnumValue;
  Ref: TPasElement;
  HadPackedModifier : Boolean;           // 12/04/04 - Dave - Added
  IsBitPacked : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD

>>>>>>> origin/cpstrnew
begin
  Result:=IsCurTokenHint(dummy);
end;

<<<<<<< HEAD
=======
Function TPasParser.IsCurTokenHint(out AHint : TPasMemberHint) : Boolean;
Var
  T : string;
begin
  if CurToken=tklibrary then
    begin
    AHint:=hLibrary;
    Result:=True;
    end
  else if CurToken=tkIdentifier then
    begin
      T:=LowerCase(CurTokenString);
      Result:=True;
      if (T='deprecated') then ahint:=hDeprecated
      else if (T='platform') then ahint:=hPlatform
      else if (T='experimental') then ahint:=hExperimental
      else if (T='unimplemented') then ahint:=hUnimplemented
      else Result:=False;
    end
  else
    Result:=False;
end;

Function TPasParser.IsCurTokenHint: Boolean;
var
  dummy : TPasMemberHint;
begin
  Result:=IsCurTokenHint(dummy);
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
Function TPasParser.IsCurTokenHint(out AHint : TPasMemberHint) : Boolean;
Var
  T : string;
begin
  if CurToken=tklibrary then
    begin
    AHint:=hLibrary;
    Result:=True;
    end
  else if CurToken=tkIdentifier then
    begin
      T:=LowerCase(CurTokenString);
      Result:=True;
      if (T='deprecated') then ahint:=hDeprecated
      else if (T='platform') then ahint:=hPlatform
      else if (T='experimental') then ahint:=hExperimental
      else if (T='unimplemented') then ahint:=hUnimplemented
      else Result:=False;
    end
  else
    Result:=False;
end;

Function TPasParser.IsCurTokenHint: Boolean;
var
  dummy : TPasMemberHint;
begin
  Result:=IsCurTokenHint(dummy);
end;

>>>>>>> graemeg/cpstrnew

Function TPasParser.CheckHint(Element : TPasElement; ExpectSemiColon : Boolean) : TPasMemberHints;
>>>>>>> graemeg/cpstrnew

Var
  Found : Boolean;
  h : TPasMemberHint;
  
begin
  Result:=[];
  Repeat
    NextToken;
    Found:=IsCurTokenHint(h);
    If Found then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  
begin
=======
  
begin
>>>>>>> origin/fixes_2_2
  Result := nil;         // !!!: Remove in the future
  HadPackedModifier := False;     { Assume not present }
  NextToken;
  if CurToken in [tkPacked,tkbitpacked] then     { If PACKED modifier }
     begin                        { Handle PACKED modifier for all situations }
     IsBitPacked:=(CurToken=tkBitPacked);
     NextToken;                   { Move to next token for rest of parse }
     if CurToken in [tkArray, tkRecord, tkObject, tkClass] then  { If allowed }
       HadPackedModifier := True  { rememeber for later }
     else                         { otherwise, syntax error }
       ParseExc(Format(SParserExpectTokenError,['ARRAY, RECORD, OBJECT or CLASS']))
     end;
  case CurToken of
    tkIdentifier:
>>>>>>> graemeg/fixes_2_2
      begin
      Include(Result,h);
      if (h=hDeprecated) then
        begin
        NextToken;
        if (Curtoken<>tkString) then
          UnGetToken
        else
          Element.HintMessage:=CurTokenString;
        end;
      end;
=======
      Include(Result,h)
>>>>>>> graemeg/cpstrnew
=======
      Include(Result,h)
>>>>>>> graemeg/cpstrnew
=======
      Include(Result,h)
>>>>>>> graemeg/cpstrnew
  Until Not Found;
  UnGetToken;
  If Assigned(Element) then
    Element.Hints:=Result;
  if ExpectSemiColon then
    ExpectToken(tkSemiColon);
end;

<<<<<<< HEAD
function TPasParser.CheckPackMode: TPackMode;

=======
function TPasParser.ParseType(Parent: TPasElement; Prefix : String): TPasType;

  procedure ParseRange;
  begin
    Result := TPasRangeType(CreateElement(TPasRangeType, '', Parent));
    try
      TPasRangeType(Result).RangeStart := ParseExpression(Result);
      ExpectToken(tkDotDot);
      TPasRangeType(Result).RangeEnd := ParseExpression(Result);
    except
      Result.Free;
      raise;
    end;
  end;

var
  Name, s: String;
  EnumValue: TPasEnumValue;
  Ref: TPasElement;
  HadPackedModifier : Boolean;           // 12/04/04 - Dave - Added
  IsBitPacked : Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
     if Not (CurToken in [tkArray, tkRecord, tkObject, tkClass]) then
       ParseExc(Format(SParserExpectTokenError,['ARRAY, RECORD, OBJECT or CLASS']))
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

function TPasParser.ParseStringType(Parent: TPasElement; const TypeName: String
  ): TPasAliasType;

Var
  S : String;

begin
  Result := TPasAliasType(CreateElement(TPasAliasType, TypeName, Parent));
  try
    If (Result.Name='') then
      Result.Name:='string';
    NextToken;
    if CurToken=tkSquaredBraceOpen then
      begin
      S:='';
      NextToken;
      While Not (Curtoken in [tkSquaredBraceClose,tkEOF]) do
        begin
        S:=S+CurTokenString;
        NextToken;
        end;
      end
    else
      UngetToken;
    Result.DestType:=TPasStringType(CreateElement(TPasStringType,'string',Nil));
    TPasStringType(Result.DestType).LengthExpr:=S;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TPasParser.ParseSimpleType(Parent: TPasElement;
  const TypeName: String; IsFull: Boolean): TPasType;

Type
  TSimpleTypeKind = (stkAlias,stkString,stkRange);

Var
  Ref: TPasElement;
  K : TSimpleTypeKind;
  Name : String;
  SS : Boolean;
begin
  Name := CurTokenString;
  NextToken;
  while CurToken=tkDot do
    begin
    ExpectIdentifier;
    Name := Name+'.'+CurTokenString;
    NextToken;
    end;
  // Current token is first token after identifier.
  if IsFull then
    begin
    if (CurToken=tkSemicolon) or isCurTokenHint then // Type A = B;
      K:=stkAlias
    else if (CurToken=tkSquaredBraceOpen) then // Type A = String[12];
      K:=stkString
    else // Type A = A..B;
      K:=stkRange;
    UnGetToken;
    end
  else  if (CurToken=tkDotDot) then // Type A = B;
    begin
    K:=stkRange;
    UnGetToken;
    end
  else
    begin
    UnGetToken;
    K:=stkAlias;
    if (LowerCase(Name)='string') then
      K:=stkString;
    end;
  Case K of
    stkString:
      begin
      Result:=ParseStringType(Parent,TypeName);
      end;
<<<<<<< HEAD
    stkRange:
=======
    tkFile:
      begin
        Result := TPasFileType(CreateElement(TPasFileType, '', Parent));
        ParseFileType(TPasFileType(Result));
      end;
    tkArray:
>>>>>>> graemeg/cpstrnew
      begin
      UnGetToken;
      Result:=ParseRangeType(Parent,TypeName,False);
      end;
    stkAlias:
      begin
      Ref:=Nil;
      SS:=isSimpleTypeToken(Name);
      if not SS then
        Ref:=Engine.FindElement(Name);
      if (Ref=Nil) then
        Ref:=TPasUnresolvedTypeRef(CreateElement(TPasUnresolvedTypeRef,Name,Nil))
      else
        Ref.AddRef;
      if isFull then
        begin
<<<<<<< HEAD
        Result := TPasAliasType(CreateElement(TPasAliasType, TypeName, Parent));
        TPasAliasType(Result).DestType:=Ref as TPasType;
        end
      else
        Result:=Ref as TPasType
=======
          NextToken;
          EnumValue := TPasEnumValue(CreateElement(TPasEnumValue,
            CurTokenString, Result));
          TPasEnumType(Result).Values.Add(EnumValue);
          NextToken;
          if CurToken = tkBraceClose then
            break
          else if CurToken in [tkEqual,tkAssign] then
            begin
            EnumValue.AssignedValue:=ParseExpression(Result);
            NextToken;
            if CurToken = tkBraceClose then
              Break
            else if not (CurToken=tkComma) then
              ParseExc(SParserExpectedCommaRBracket);
            end
          else if not (CurToken=tkComma) then
            ParseExc(SParserExpectedCommaRBracket)
        end;
      end;
    tkSet:
      begin
        Result := TPasSetType(CreateElement(TPasSetType, '', Parent));
    try
          ExpectToken(tkOf);
          TPasSetType(Result).EnumType := ParseType(Result);
    except
      Result.Free;
      raise;
    end;
      end;
    tkRecord:
      begin
        Result := TPasRecordType(CreateElement(TPasRecordType, '', Parent));
        TPasRecordType(Result).IsPacked:=HadPackedModifier;
        If HadPackedModifier then
            TPasRecordType(Result).IsBitPacked:=IsBitPacked;
<<<<<<< HEAD
<<<<<<< HEAD
    try
=======
=======
>>>>>>> origin/fixes_2_2
	try
>>>>>>> graemeg/fixes_2_2
          ParseRecordDecl(TPasRecordType(Result), False);
    except
      Result.Free;
      raise;
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
  end;
end;

// On entry, we're on the TYPE token
function TPasParser.ParseAliasType(Parent: TPasElement; const TypeName: String
  ): TPasTypeAliasType;
begin
  Result := TPasTypeAliasType(CreateElement(TPasTypeAliasType, TypeName, Parent));
  try
    Result.DestType := ParseType(nil,'');
  except
    FreeAndNil(Result);
    raise;
  end;
end;

function TPasParser.ParsePointerType(Parent: TPasElement; const TypeName: String
  ): TPasPointerType;

begin
  Result := TPasPointerType(CreateElement(TPasPointerType, TypeName, Parent));
  Try
    TPasPointerType(Result).DestType := ParseType(nil);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TPasParser.ParseEnumType(Parent: TPasElement; const TypeName: String
  ): TPasEnumType;

Var
  EnumValue: TPasEnumValue;

begin
  Result := TPasEnumType(CreateElement(TPasEnumType, TypeName, Parent));
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
          ParseExc(SParserExpectedCommaRBracket);
        end
      else if not (CurToken=tkComma) then
        ParseExc(SParserExpectedCommaRBracket)
      end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TPasParser.ParseSetType(Parent: TPasElement; const TypeName: String
  ): TPasSetType;

begin
  Result := TPasSetType(CreateElement(TPasSetType, TypeName, Parent));
  try
    ExpectToken(tkOf);
    Result.EnumType := ParseType(Result,'',False);
  except
    Result.Free;
    raise;
  end;
end;

function TPasParser.ParseType(Parent: TPasElement; const TypeName: String;
  Full: Boolean): TPasType;

Const
  // These types are allowed only when full type declarations
  FullTypeTokens = [tkGeneric,{tkSpecialize,}tkClass,tkInterface,tkType];
  // Parsing of these types already takes care of hints
  NoHintTokens = [tkProcedure,tkFunction];
var
  PM : TPackMode;
  CH : Boolean; // Check hint ?
begin
  Result := nil;
  Pm:=CheckPackMode;
  if Full then
    CH:=Not (CurToken in NoHintTokens)
  else
    begin
    CH:=False;
    if (CurToken in FullTypeTokens) then
      ParseExc('Type '+CurtokenText+' not allowed here');
    end;
  Try
    case CurToken of
      // types only allowed when full
      tkObject: Result := ParseClassDecl(Parent, TypeName, okObject,PM);
      tkInterface: Result := ParseClassDecl(Parent, TypeName, okInterface);
      tkSpecialize: Result:=ParseSpecializeType(Parent,TypeName);
      tkClass: Result := ParseClassDecl(Parent, TypeName, okClass, PM);
      tkType: Result:=ParseAliasType(Parent,TypeName);
      // Always allowed
      tkIdentifier: Result:=ParseSimpleType(Parent,TypeName,Full);
      tkCaret: Result:=ParsePointerType(Parent,TypeName);
      tkFile: Result:=ParseFileType(Parent,TypeName);
      tkArray: Result:=ParseArrayType(Parent,TypeName,pm);
      tkBraceOpen: Result:=ParseEnumType(Parent,TypeName);
      tkSet: Result:=ParseSetType(Parent,TypeName);
      tkProcedure: Result:=ParseProcedureType(Parent,TypeName,ptProcedure);
      tkFunction: Result:=ParseProcedureType(Parent,TypeName,ptFunction);
      tkRecord:
        begin
        NextToken;
        if (Curtoken=tkHelper) then
          begin
          UnGetToken;
          Result:=ParseClassDecl(Parent,TypeName,okRecordHelper,PM);
          end
        else
          begin
          UnGetToken;
          Result := ParseRecordDecl(Parent,TypeName,PM);
          end;
        end;
    else
      UngetToken;
      Result:=ParseRangeType(Parent,TypeName,Full);
    end;
    if CH then
      CheckHint(Result,True);
  Except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TPasParser.ParseComplexType(Parent : TPasElement = Nil): TPasType;
begin
  NextToken;
  case CurToken of
    tkProcedure:
      begin
        Result := TPasProcedureType(CreateElement(TPasProcedureType, '', Parent));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        ParseProcedureOrFunctionHeader(Result, TPasProcedureType(Result), ptProcedure, True);
        if CurToken = tkSemicolon then
          UngetToken;        // Unget semicolon
      end;
    tkFunction:
      begin
        Result := CreateFunctionType('', 'Result', Parent, False);
        ParseProcedureOrFunctionHeader(Result, TPasFunctionType(Result), ptFunction, True);
        if CurToken = tkSemicolon then
          UngetToken;        // Unget semicolon
      end;
  else
    UngetToken;
    Result := ParseType(Parent);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ParseProcedureOrFunctionHeader(Result,
          TPasProcedureType(Result), ptProcedure, True);
        if CurToken = tkSemicolon then UngetToken;        // Unget semicolon
      end;
    tkFunction:
      begin
        Result := Engine.CreateFunctionType('', 'Result', Parent, False,
      Scanner.CurFilename, Scanner.CurRow);
        ParseProcedureOrFunctionHeader(Result,
          TPasFunctionType(Result), ptFunction, True);
        UngetToken;        // Unget semicolon
      end;
    else
    begin
      UngetToken;
      Result := ParseType(Parent);
      exit;
    end;
>>>>>>> graemeg/cpstrnew
  end;
end;

function TPasParser.ParseArrayType(Parent: TPasElement; const TypeName: String;
  PackMode: TPackMode): TPasArrayType;

Var
  S : String;

begin
  Result := TPasArrayType(CreateElement(TPasArrayType, TypeName, Parent));
  try
    Result.PackMode:=PackMode;
    NextToken;
    S:='';
    case CurToken of
      tkSquaredBraceOpen:
        begin
          repeat
            NextToken;
            if CurToken<>tkSquaredBraceClose then
              S:=S+CurTokenText;
          until CurToken = tkSquaredBraceClose;
          Result.IndexRange:=S;
          ExpectToken(tkOf);
          Result.ElType := ParseType(nil);
        end;
      tkOf:
        begin
          NextToken;
          if CurToken = tkConst then
          else
          begin
            UngetToken;
              Result.ElType := ParseType(nil);
          end
        end
      else
        ParseExc(SParserArrayTypeSyntaxError);
    end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TPasParser.ParseFileType(Parent: TPasElement; const TypeName: String
  ): TPasFileType;


begin
  Result:=TPasFileType(CreateElement(TPasFileType, TypeName, Parent));
  NextToken;
  If CurToken=tkOf then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Result.ElType := ParseType(nil)
=======
    Element.ElType := ParseType(nil)
>>>>>>> graemeg/cpstrnew
=======
    Element.ElType := ParseType(nil)
>>>>>>> graemeg/cpstrnew
=======
    Element.ElType := ParseType(nil)
>>>>>>> graemeg/cpstrnew
=======
    Element.ElType := ParseType(nil)
>>>>>>> origin/cpstrnew
  else 
   ungettoken;
end;

function TPasParser.isEndOfExp:Boolean;
const
  EndExprToken = [
    tkEOF, tkBraceClose, tkSquaredBraceClose, tkSemicolon, tkComma, tkColon,
    tkdo, tkdownto, tkelse, tkend, tkof, tkthen, tkto
  ];
begin
  Result:=(CurToken in EndExprToken) or IsCurTokenHint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

function TPasParser.ParseParams(AParent: TPasElement;paramskind: TPasExprKind): TParamsExpr;
var
  params  : TParamsExpr;
  p       : TPasExpr;
  PClose  : TToken;
begin
  Result:=nil;
  if paramskind in [pekArrayParams, pekSet] then begin
    if CurToken<>tkSquaredBraceOpen then Exit;
    PClose:=tkSquaredBraceClose;
  end else begin
    if CurToken<>tkBraceOpen then Exit;
    PClose:=tkBraceClose;
  end;

  params:=TParamsExpr.Create(AParent,paramskind);
  try
    NextToken;
    if not isEndOfExp then begin
      repeat
        p:=DoParseExpression(AParent);
        if not Assigned(p) then Exit; // bad param syntax
        params.AddParam(p);

        if not (CurToken in [tkComma, PClose]) then begin
          Exit;
        end;

        if CurToken = tkComma then begin
          NextToken;
          if CurToken = PClose then begin
            //ErrorExpected(parser, 'identifier');
            Exit;
          end;
        end;
      until CurToken=PClose;
    end;
    NextToken;
    Result:=params;
  finally
    if not Assigned(Result) then params.Free;
=======
end;

function TPasParser.ParseParams(AParent: TPasElement;paramskind: TPasExprKind): TParamsExpr;
=======
end;

function TPasParser.ParseParams(AParent: TPasElement;paramskind: TPasExprKind): TParamsExpr;
=======
=======
>>>>>>> origin/cpstrnew
end;

function TPasParser.ParseParams(AParent: TPasElement;paramskind: TPasExprKind): TParamsExpr;
var
  params  : TParamsExpr;
  p       : TPasExpr;
  PClose  : TToken;
begin
  Result:=nil;
  if paramskind in [pekArrayParams, pekSet] then begin
    if CurToken<>tkSquaredBraceOpen then Exit;
    PClose:=tkSquaredBraceClose;
  end else begin
    if CurToken<>tkBraceOpen then Exit;
    PClose:=tkBraceClose;
  end;

  params:=TParamsExpr.Create(AParent,paramskind);
  try
    NextToken;
    if not isEndOfExp then begin
      repeat
        p:=DoParseExpression(AParent);
        if not Assigned(p) then Exit; // bad param syntax
        params.AddParam(p);

        if not (CurToken in [tkComma, PClose]) then begin
          Exit;
        end;

        if CurToken = tkComma then begin
          NextToken;
          if CurToken = PClose then begin
            //ErrorExpected(parser, 'identifier');
            Exit;
          end;
        end;
      until CurToken=PClose;
    end;
    NextToken;
    Result:=params;
  finally
    if not Assigned(Result) then params.Free;
  end;
end;

Function TPasParser.TokenToExprOp (AToken : TToken) : TExprOpCode;

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
    ParseExc(format('Not an operand: (%d : %s)',[AToken,TokenInfos[AToken]]));
  end;
end;
 
function TPasParser.ParseExpIdent(AParent : TPasElement):TPasExpr;
var
  x       : TPasExpr;
  prm     : TParamsExpr;
  u       : TUnaryExpr;
  b       : TBinaryExpr;
  optk    : TToken;
begin
  Result:=nil;
  case CurToken of
    tkString:           x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenString);
    tkChar:             x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText);
    tkNumber:           x:=TPrimitiveExpr.Create(AParent,pekNumber, CurTokenString);
    tkIdentifier:       x:=TPrimitiveExpr.Create(AParent,pekIdent, CurTokenText);
    tkfalse, tktrue:    x:=TBoolConstExpr.Create(Aparent,pekBoolConst, CurToken=tktrue);
    tknil:              x:=TNilExpr.Create(Aparent);
    tkSquaredBraceOpen: x:=ParseParams(AParent,pekSet);
    tkinherited: begin
      //inherited; inherited function
      x:=TInheritedExpr.Create(AParent);
      NextToken;
      if (length(CurTokenText)>0) and (CurTokenText[1] in ['A'..'_']) then begin
        b:=TBinaryExpr.Create(AParent,x, DoParseExpression(AParent), eopNone);
        if not Assigned(b.right) then Exit; // error
        x:=b;
        UngetToken;
      end
       else UngetToken;
    end;
    tkself: begin
      x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText); //function(self);
      x:=TSelfExpr.Create(AParent);
      NextToken;
      if CurToken = tkDot then begin // self.Write(EscapeText(AText));
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then Exit; // error
        x:=b;
      end
       else UngetToken;
    end;
    tkAt: begin
      // P:=@function;
      NextToken;
      if (length(CurTokenText)=0) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '@'+CurTokenText);
    end;
    tkCaret: begin
      // ^A..^_ characters. See #16341
      NextToken;
      if not (length(CurTokenText)=1) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '^'+CurTokenText);
    end;
  else
    ParseExc(SParserExpectedIdentifier);
  end;

  if x.Kind<>pekSet then NextToken;

  try
    if x.Kind=pekIdent then begin
      while CurToken in [tkBraceOpen, tkSquaredBraceOpen, tkCaret] do
        case CurToken of
          tkBraceOpen: begin
            prm:=ParseParams(AParent,pekFuncParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkSquaredBraceOpen: begin
            prm:=ParseParams(AParent,pekArrayParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkCaret: begin
            u:=TUnaryExpr.Create(AParent,x, TokenToExprOp(CurToken));
            x:=u;
            NextToken;
          end;
        end;

      if CurToken in [tkDot, tkas] then begin
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then Exit; // error
        x:=b;
      end;
    end;

    if CurToken = tkDotDot then begin
      NextToken;
      b:=TBinaryExpr.CreateRange(AParent,x, DoParseExpression(AParent));
      if not Assigned(b.right) then Exit; // error
      x:=b;
    end;

    Result:=x;
  finally
    if not Assigned(Result) then x.Free;
  end;
end;

function TPasParser.OpLevel(t: TToken): Integer;
begin
  case t of
    tknot,tkAt:
      Result:=4;
    tkMul, tkDivision, tkdiv, tkmod, tkand, tkShl,tkShr, tkas, tkPower :
      Result:=3;
    tkPlus, tkMinus, tkor, tkxor:
      Result:=2;
    tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan, tkGreaterThan, tkGreaterEqualThan, tkin, tkis:
      Result:=1;
  else
    Result:=0;
  end;
end;

function TPasParser.DoParseExpression(Aparent : TPaselement;InitExpr: TPasExpr): TPasExpr;
var
  expstack  : TList;
  opstack   : TList;
  pcount    : Integer;
  x         : TPasExpr;
  i         : Integer;
  tempop    : TToken;
  NotBinary : Boolean;
  
const
  PrefixSym = [tkPlus, tkMinus, tknot, tkAt]; // + - not @
  BinaryOP  = [tkMul, tkDivision, tkdiv, tkmod,
               tkand, tkShl,tkShr, tkas, tkPower,
               tkPlus, tkMinus, tkor, tkxor, tkSymmetricalDifference,
               tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan,
               tkGreaterThan, tkGreaterEqualThan, tkin, tkis];

  function PopExp: TPasExpr; inline;
  begin
    if expstack.Count>0 then begin
      Result:=TPasExpr(expstack[expstack.Count-1]);
      expstack.Delete(expstack.Count-1);
    end else
      Result:=nil;
  end;

  procedure PushOper(token: TToken); inline;
  begin
    opstack.Add( Pointer(PtrInt(token)) );
  end;

  function PeekOper: TToken; inline;
  begin
    if opstack.Count>0 then Result:=TToken(PtrUInt(opstack[ opstack.Count-1]))
    else Result:=tkEOF
  end;

  function PopOper: TToken; inline;
  begin
    Result:=PeekOper;
    if Result<>tkEOF then opstack.Delete(opstack.Count-1);
  end;

  procedure PopAndPushOperator;
  var
    t       : TToken;
    xright  : TPasExpr;
    xleft   : TPasExpr;
  begin
    t:=PopOper;
    xright:=PopExp;
    xleft:=PopExp;
    expstack.Add(TBinaryExpr.Create(AParent,xleft, xright, TokenToExprOp(t)));
  end;

begin
  Result:=nil;
  expstack := TList.Create;
  opstack := TList.Create;
  try
    repeat
      NotBinary:=True;
      pcount:=0;

      if not Assigned(InitExpr) then
      begin
        // the first part of the expression has been parsed externally.
        // this is used by Constant Expresion parser (CEP) parsing only,
        // whenever it makes a false assuming on constant expression type.
        // i.e: SI_PAD_SIZE = ((128/sizeof(longint)) - 3);
        //
        // CEP assumes that it's array or record, because the expression
        // starts with "(". After the first part is parsed, the CEP meets "-"
        // that assures, it's not an array expression. The CEP should give the
        // first partback to the expression parser, to get the correct
        // token tree according to the operations priority.
        //
        // quite ugly. type information is required for CEP to work clean

        while CurToken in PrefixSym do begin
          PushOper(CurToken);
          inc(pcount);
          NextToken;
        end;

        if CurToken = tkBraceOpen then begin
          NextToken;
          x:=DoParseExpression(AParent);
          if CurToken<>tkBraceClose then Exit;
          NextToken;

          // for the expression like  (TObject(m)).Free;
          if CurToken = tkDot then begin
            NextToken;
            x:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(tkDot));
          end;

        end else begin
          x:=ParseExpIdent(AParent);
        end;

        if not Assigned(x) then Exit;
        expstack.Add(x);
        for i:=1 to pcount do begin
          tempop:=PopOper;
          expstack.Add( TUnaryExpr.Create(AParent, PopExp, TokenToExprOp(tempop) ));
        end;

      end else
      begin
        expstack.Add(InitExpr);
        InitExpr:=nil;
      end;

      if (CurToken in BinaryOP) then begin
        // Adjusting order of the operations
        NotBinary:=False;
        tempop:=PeekOper;
        while (opstack.Count>0) and (OpLevel(tempop)>=OpLevel(CurToken)) do begin
          PopAndPushOperator;
          tempop:=PeekOper;
        end;
        PushOper(CurToken);
        NextToken;
      end;

    until NotBinary or isEndOfExp;

    if not NotBinary then ParseExc(SParserExpectedIdentifier);

    while opstack.Count>0 do PopAndPushOperator;

    // only 1 expression should be on the stack, at the end of the correct expression
    if expstack.Count=1 then Result:=TPasExpr(expstack[0]);

  finally
    if not Assigned(Result) then begin
      // expression error!
      for i:=0 to expstack.Count-1 do
        TObject(expstack[i]).Free;
    end;
    opstack.Free;
    expstack.Free;
  end;
end;

function TPasParser.ParseExpression(Aparent : TPaselement;Kind: TExprKind): String;
var
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
  ls: String;
begin
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) then 
    begin
      if (CurToken in [tkComma, tkSemicolon,
        tkColon, tkDotDot, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept,
        tkof, tkbegin, tkdo, tkto, tkdownto, tkinitialization, tkfinalization])
      then
        break;
        
      if (Kind=ek_PropertyIndex) and (CurToken=tkIdentifier) then begin
        ls:=LowerCase(CurTokenText);
        if (ls='read') or (ls ='write') or (ls='default') or (ls='nodefault') or (ls='implements') then
          Break;
      end;
        
    end;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
<<<<<<< HEAD
=======
    if CurToken=tkString then
      begin
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
      end
    else
      Result := Result + CurTokenText;
  end;
  if Result='' then
    ParseExc(SParserSyntaxError);
  UngetToken;
end;

function GetExprIdent(p: TPasExpr): String;
begin
  if Assigned(p) and (p is TPrimitiveExpr) and (p.Kind=pekIdent) then
    Result:=TPrimitiveExpr(p).Value
  else
    Result:='';
end;

function TPasParser.DoParseConstValueExpression(Aparent : TPaselement): TPasExpr;
var
  x : TPasExpr;
  n : AnsiString;
  r : TRecordValues;
  a : TArrayValues;

function lastfield:boolean;

begin
  result:= CurToken<>tkSemicolon;
  if not result then
   begin
     nexttoken;
     if curtoken=tkbraceclose then
       result:=true
     else
       ungettoken;
   end; 
end;

begin
  if CurToken <> tkBraceOpen then
    Result:=DoParseExpression(AParent)
  else begin
    NextToken;
    x:=DoParseConstValueExpression(Aparent);
    case CurToken of
      tkComma: // array of values (a,b,c);
        begin
          a:=TArrayValues.Create(AParent);
          a.AddValues(x);
          repeat
            NextToken;
            x:=DoParseConstValueExpression(AParent);
            a.AddValues(x);
          until CurToken<>tkComma;
          Result:=a;
        end;

      tkColon: // record field (a:xxx;b:yyy;c:zzz);
        begin
          n:=GetExprIdent(x);
          x.Free;
          r:=TRecordValues.Create(AParent);
          NextToken;
          x:=DoParseConstValueExpression(AParent);
          r.AddField(n, x);
          if not lastfield then
            repeat
              n:=ExpectIdentifier;
              ExpectToken(tkColon);
              NextToken;
              x:=DoParseConstValueExpression(AParent);
              r.AddField(n, x)
            until lastfield; // CurToken<>tkSemicolon;
          Result:=r;
        end;
    else
      // Binary expression!  ((128 div sizeof(longint)) - 3);       ;
      Result:=DoParseExpression(AParent,x);
    end;
    if CurToken<>tkBraceClose then ParseExc(SParserExpectedCommaRBracket);
    NextToken;
  end;
end;

function TPasParser.ParseCommand: String;
var
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
begin
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) and (CurToken in [tkComma, tkSemicolon,
      tkColon, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept, tkof, tkdo,
      tkbegin, tkinitialization, tkfinalization]) then
      break;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
>>>>>>> origin/cpstrnew
    if CurToken=tkString then
      begin
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
      end
    else
      Result := Result + CurTokenText;
  end;
  if Result='' then
    ParseExc(SParserSyntaxError);
  UngetToken;
end;

<<<<<<< HEAD
function GetExprIdent(p: TPasExpr): String;
begin
  if Assigned(p) and (p is TPrimitiveExpr) and (p.Kind=pekIdent) then
    Result:=TPrimitiveExpr(p).Value
  else
    Result:='';
=======
procedure TPasParser.AddProcOrFunction(Declarations: TPasDeclarations;
  AProc: TPasProcedure);
var
  i: Integer;
  Member: TPasElement;
  OverloadedProc: TPasOverloadedProc;
begin
  for i := 0 to Declarations.Functions.Count - 1 do
  begin
    Member := TPasElement(Declarations.Functions[i]);
    if CompareText(Member.Name, AProc.Name) = 0 then
    begin
      if Member.ClassType = TPasOverloadedProc then
        TPasOverloadedProc(Member).Overloads.Add(AProc)
      else
      begin
        OverloadedProc := TPasOverloadedProc.Create(AProc.Name, Declarations);
        OverloadedProc.Overloads.Add(Member);
        OverloadedProc.Overloads.Add(AProc);
        Declarations.Functions[i] := OverloadedProc;
        Declarations.Declarations[Declarations.Declarations.IndexOf(Member)] :=
          OverloadedProc;
      end;
      exit;
    end;
  end;

  // Not overloaded, so just add the proc/function to the lists
  Declarations.Declarations.Add(AProc);
  Declarations.Functions.Add(AProc);
>>>>>>> origin/cpstrnew
end;

function TPasParser.DoParseConstValueExpression(Aparent : TPaselement): TPasExpr;
var
  x : TPasExpr;
  n : AnsiString;
  r : TRecordValues;
  a : TArrayValues;

function lastfield:boolean;

begin
  result:= CurToken<>tkSemicolon;
  if not result then
   begin
     nexttoken;
     if curtoken=tkbraceclose then
       result:=true
     else
       ungettoken;
   end; 
end;

begin
<<<<<<< HEAD
  if CurToken <> tkBraceOpen then
    Result:=DoParseExpression(AParent)
  else begin
    NextToken;
    x:=DoParseConstValueExpression(Aparent);
    case CurToken of
      tkComma: // array of values (a,b,c);
        begin
          a:=TArrayValues.Create(AParent);
          a.AddValues(x);
          repeat
            NextToken;
            x:=DoParseConstValueExpression(AParent);
            a.AddValues(x);
          until CurToken<>tkComma;
          Result:=a;
        end;

      tkColon: // record field (a:xxx;b:yyy;c:zzz);
        begin
          n:=GetExprIdent(x);
          x.Free;
          r:=TRecordValues.Create(AParent);
          NextToken;
          x:=DoParseConstValueExpression(AParent);
          r.AddField(n, x);
          if not lastfield then
            repeat
              n:=ExpectIdentifier;
              ExpectToken(tkColon);
              NextToken;
              x:=DoParseConstValueExpression(AParent);
              r.AddField(n, x)
            until lastfield; // CurToken<>tkSemicolon;
          Result:=r;
        end;
=======
  Module:=nil;
  NextToken;
  case CurToken of
    tkUnit: ParseUnit(Module);
    tkProgram: ParseProgram(Module);
>>>>>>> origin/cpstrnew
    else
      // Binary expression!  ((128 div sizeof(longint)) - 3);       ;
      Result:=DoParseExpression(AParent,x);
    end;
    if CurToken<>tkBraceClose then ParseExc(SParserExpectedCommaRBracket);
    NextToken;
  end;
end;

<<<<<<< HEAD
function TPasParser.ParseCommand: String;
var
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
begin
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) and (CurToken in [tkComma, tkSemicolon,
      tkColon, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept, tkof, tkdo,
      tkbegin, tkinitialization, tkfinalization]) then
      break;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
    if CurToken=tkString then
      begin
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
      end
    else
      Result := Result + CurTokenText;
=======
// Starts after the "unit" token
procedure TPasParser.ParseUnit(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasModule, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    CheckHint(Module,True);
//    ExpectToken(tkSemicolon);
    ExpectToken(tkInterface);
    ParseInterface;
  finally
    CurModule:=nil;
  end;
end;

// Starts after the "program" token
procedure TPasParser.ParseProgram(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasProgram, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    NextToken;
    ParseImplementation;
  finally
    CurModule:=nil;
  end;
end;

// Starts after the "interface" token
procedure TPasParser.ParseInterface;
var
  Section: TInterfaceSection;
begin
  Section := TInterfaceSection(CreateElement(TInterfaceSection, '', CurModule));
  CurModule.InterfaceSection := Section;
  ParseDeclarations(Section);
end;

// Starts after the "implementation" token
procedure TPasParser.ParseImplementation;
var
  Section: TImplementationSection;
begin
  Section := TImplementationSection(CreateElement(TImplementationSection, '', CurModule));
  CurModule.ImplementationSection := Section;
  ParseDeclarations(Section);
end;

procedure TPasParser.ParseInitialization;
var
  Section: TInitializationSection;
  SubBlock: TPasImplElement;
begin
  Section := TInitializationSection(CreateElement(TInitializationSection, '', CurModule));
  CurModule.InitializationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      exit;
    end
    else if (CurToken=tkfinalization) then
    begin
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
  UngetToken;
end;

procedure TPasParser.ParseDeclarations(Declarations: TPasDeclarations);
var
  CurBlock: TDeclType;
  ConstEl: TPasConst;
  ResStrEl: TPasResString;
  TypeEl: TPasType;
  ClassEl: TPasClassType;
  List: TList;
  i,j: Integer;
  VarEl: TPasVariable;
  PropEl : TPasProperty;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
begin
  CurBlock := declNone;
  while True do
  begin
    NextToken;
    //writeln('TPasParser.ParseSection Token=',Scanner.CurTokenString,' ',CurToken);
    case CurToken of
      tkend:
        begin
        ExpectToken(tkDot);
        break;
        end;
      tkimplementation:
        if (CurToken = tkImplementation) and (Declarations is TInterfaceSection) then
          begin
          If Not Engine.InterfaceOnly then
            ParseImplementation;
          break;
          end;
      tkinitialization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end;
      tkfinalization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseFinalization;
          break;
          end;
      tkUses:
        if Declarations is TPasSection then
          ParseUsesList(TPasSection(Declarations))
        else
          ParseExc(SParserSyntaxError);
      tkConst:
        CurBlock := declConst;
      tkResourcestring:
        CurBlock := declResourcestring;
      tkType:
        CurBlock := declType;
      tkVar:
        CurBlock := declVar;
      tkThreadVar:
        CurBlock := declThreadVar;
      tkProperty:
<<<<<<< HEAD
<<<<<<< HEAD
        CurBlock := declProperty;
=======
        CurBlock := declProperty;  
>>>>>>> graemeg/fixes_2_2
=======
        CurBlock := declProperty;  
>>>>>>> origin/fixes_2_2
      tkProcedure:
        begin
          AddProcOrFunction(Declarations,
                       ParseProcedureOrFunctionDecl(Declarations, ptProcedure));
          CurBlock := declNone;
        end;
      tkFunction:
        begin
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptFunction));
          CurBlock := declNone;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
      tkConstructor:
        begin
          AddProcOrFunction(Declarations,
                     ParseProcedureOrFunctionDecl(Declarations, ptConstructor));
          CurBlock := declNone;
        end;
      tkDestructor:
        begin
          AddProcOrFunction(Declarations,
                      ParseProcedureOrFunctionDecl(Declarations, ptDestructor));
          CurBlock := declNone;
        end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      tkOperator:
        begin
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptOperator));
          CurBlock := declNone;
        end;
      tkClass:
        begin
          NextToken;
          case CurToken of
            tkprocedure:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassProcedure));
                CurBlock := declNone;
              end;
            tkfunction:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassFunction));
                CurBlock := declNone;
              end;
          else
            ExpectToken(tkprocedure);
          end;
        end;
      tkIdentifier:
        begin
          case CurBlock of
            declConst:
              begin
                ConstEl := ParseConstDecl(Declarations);
                Declarations.Declarations.Add(ConstEl);
                Declarations.Consts.Add(ConstEl);
              end;
            declResourcestring:
              begin
                ResStrEl := ParseResourcestringDecl(Declarations);
                Declarations.Declarations.Add(ResStrEl);
                Declarations.ResStrings.Add(ResStrEl);
              end;
            declType:
              begin
                TypeEl := ParseTypeDecl(Declarations);
                if Assigned(TypeEl) then        // !!!
                begin
                  Declarations.Declarations.Add(TypeEl);
                  if TypeEl.ClassType = TPasClassType then
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
                        ClassEl.Release;
                        break;
                      end;
                    end;
                    // Add the new class to the class list
                    Declarations.Classes.Add(TypeEl)
                  end else
                    Declarations.Types.Add(TypeEl);
                end;
              end;
            declVar, declThreadVar:
              begin
                List := TList.Create;
                try
                  try
                    ParseVarDecl(Declarations, List);
                  except
                    for i := 0 to List.Count - 1 do
                      TPasVariable(List[i]).Release;
                    raise;
                  end;
                  for i := 0 to List.Count - 1 do
                  begin
                    VarEl := TPasVariable(List[i]);
                    Declarations.Declarations.Add(VarEl);
                    Declarations.Variables.Add(VarEl);
                  end;
                finally
                  List.Free;
                end;
              end;
            declProperty:
              begin
<<<<<<< HEAD
<<<<<<< HEAD
              PropEl:=TPasProperty(CreateElement(TPasProperty, CurTokenString, Declarations));
=======
              PropEl:=TPasProperty(CreateElement(TPasProperty, CurTokenString, Section));
>>>>>>> graemeg/fixes_2_2
=======
              PropEl:=TPasProperty(CreateElement(TPasProperty, CurTokenString, Section));
>>>>>>> origin/fixes_2_2
              Try
                ParseProperty(PropEl)
              except
                Propel.Free;
                Raise;
<<<<<<< HEAD
<<<<<<< HEAD
              end;
              Declarations.Declarations.Add(PropEl);
              Declarations.properties.add(PropEl);
=======
              end;    
              Section.Declarations.Add(PropEl);
              Section.properties.add(PropEl);
>>>>>>> graemeg/fixes_2_2
=======
              end;    
              Section.Declarations.Add(PropEl);
              Section.properties.add(PropEl);
>>>>>>> origin/fixes_2_2
              end;
          else
            ParseExc(SParserSyntaxError);
          end;
        end;
      tkbegin:
        begin
        if Declarations is TProcedureBody then
          begin
          ParseProcBeginBlock(TProcedureBody(Declarations));
          break;
          end
        else if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end
        else
          ParseExc(SParserSyntaxError);
        end;
      tklabel:
        begin
          if not (Declarations is TInterfaceSection) then
            ParseLabels(Declarations);
        end;
    else
      ParseExc(SParserSyntaxError);
    end;
>>>>>>> origin/cpstrnew
  end;
  UngetToken;
end;

<<<<<<< HEAD
procedure TPasParser.AddProcOrFunction(Declarations: TPasDeclarations;
  AProc: TPasProcedure);
>>>>>>> graemeg/cpstrnew
var
  params  : TParamsExpr;
  p       : TPasExpr;
  PClose  : TToken;
begin
<<<<<<< HEAD
  Result:=nil;
  if paramskind in [pekArrayParams, pekSet] then begin
    if CurToken<>tkSquaredBraceOpen then Exit;
    PClose:=tkSquaredBraceClose;
  end else begin
    if CurToken<>tkBraceOpen then Exit;
    PClose:=tkBraceClose;
  end;

  params:=TParamsExpr.Create(AParent,paramskind);
  try
    NextToken;
    if not isEndOfExp then begin
      repeat
        p:=DoParseExpression(AParent);
        if not Assigned(p) then Exit; // bad param syntax
        params.AddParam(p);
=======
// Starts after the "uses" token
procedure TPasParser.ParseUsesList(ASection: TPasSection);

function CheckUnit(AUnitName : string):TPasElement;
begin
    result := Engine.FindModule(AUnitName);  // should we resolve module here when "IN" filename is not known yet?
    if Assigned(result) then
      result.AddRef
    else
      Result := TPasType(CreateElement(TPasUnresolvedTypeRef, AUnitName,
        ASection));
    ASection.UsesList.Add(Result);
end;

var
  AUnitName: String;
  Element: TPasElement;
begin
  If not (Asection is TImplementationSection) Then // interface,program,library,package
    Element:=CheckUnit('System'); // system always implicitely first.    
  while True do
  begin
    AUnitName := ExpectIdentifier; 
    Element :=CheckUnit(AUnitName);
>>>>>>> origin/cpstrnew

        if not (CurToken in [tkComma, PClose]) then begin
          Exit;
        end;

        if CurToken = tkComma then begin
          NextToken;
          if CurToken = PClose then begin
            //ErrorExpected(parser, 'identifier');
            Exit;
          end;
        end;
      until CurToken=PClose;
    end;
    NextToken;
<<<<<<< HEAD
    Result:=params;
  finally
    if not Assigned(Result) then params.Free;
  end;
=======
  for i := 0 to Declarations.Functions.Count - 1 do
  begin
    Member := TPasElement(Declarations.Functions[i]);
    if CompareText(Member.Name, AProc.Name) = 0 then
    begin
      if Member.ClassType = TPasOverloadedProc then
        TPasOverloadedProc(Member).Overloads.Add(AProc)
      else
      begin
        OverloadedProc := TPasOverloadedProc.Create(AProc.Name, Declarations);
        OverloadedProc.Overloads.Add(Member);
        OverloadedProc.Overloads.Add(AProc);
        Declarations.Functions[i] := OverloadedProc;
        Declarations.Declarations[Declarations.Declarations.IndexOf(Member)] :=
          OverloadedProc;
      end;
      exit;
    end;
=======

    if CurToken = tkin then begin
      // todo: store unit's file name somewhere
      NextToken; // skip in
      ExpectToken(tkString); // skip unit's real file name
      if (Element is TPasModule) and (TPasmodule(Element).filename<>'')  then
        TPasModule(Element).FileName:=curtokenstring;
    end;

    if CurToken = tkSemicolon then
      break
    else if CurToken <> tkComma then
      ParseExc(SParserExpectedCommaSemicolon);
>>>>>>> origin/cpstrnew
  end;

  // Not overloaded, so just add the proc/function to the lists
  Declarations.Declarations.Add(AProc);
  Declarations.Functions.Add(AProc);
>>>>>>> graemeg/cpstrnew
end;

Function TPasParser.TokenToExprOp (AToken : TToken) : TExprOpCode;

<<<<<<< HEAD
begin
<<<<<<< HEAD
  Case AToken of
    tkMul                   : Result:=eopMultiply;
    tkPlus                  : Result:=eopAdd;
    tkMinus                 : Result:=eopSubtract;
    tkDivision              : Result:=eopDivide;
    tkLessThan              : Result:=eopLessThan;
    tkEqual                 : Result:=eopEqual;
    tkGreaterThan           : Result:=eopGreaterThan;
    tkAt                    : Result:=eopAddress;
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
    ParseExc(format('Not an operand: (%d : %s)',[AToken,TokenInfos[AToken]]));
=======
    ExpectToken(tkEqual);
    Result.Value := ParseExpression;
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
>>>>>>> origin/fixes_2_2
  end;
end;
 
function TPasParser.ParseExpIdent(AParent : TPasElement):TPasExpr;
var
  x       : TPasExpr;
  prm     : TParamsExpr;
  u       : TUnaryExpr;
  b       : TBinaryExpr;
  optk    : TToken;
begin
<<<<<<< HEAD
  Result:=nil;
  case CurToken of
    tkString:           x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenString);
    tkChar:             x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText);
    tkNumber:           x:=TPrimitiveExpr.Create(AParent,pekNumber, CurTokenString);
    tkIdentifier:       x:=TPrimitiveExpr.Create(AParent,pekIdent, CurTokenText);
    tkfalse, tktrue:    x:=TBoolConstExpr.Create(Aparent,pekBoolConst, CurToken=tktrue);
    tknil:              x:=TNilExpr.Create(Aparent);
    tkSquaredBraceOpen: x:=ParseParams(AParent,pekSet);
    tkinherited: begin
      //inherited; inherited function
      x:=TInheritedExpr.Create(AParent);
      NextToken;
      if (length(CurTokenText)>0) and (CurTokenText[1] in ['A'..'_']) then begin
        b:=TBinaryExpr.Create(AParent,x, DoParseExpression(AParent), eopNone);
        if not Assigned(b.right) then Exit; // error
        x:=b;
        UngetToken;
      end
       else UngetToken;
    end;
    tkself: begin
      x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText); //function(self);
      x:=TSelfExpr.Create(AParent);
      NextToken;
      if CurToken = tkDot then begin // self.Write(EscapeText(AText));
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then Exit; // error
        x:=b;
      end
       else UngetToken;
    end;
    tkAt: begin
      // P:=@function;
      NextToken;
      if (length(CurTokenText)=0) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '@'+CurTokenText);
    end;
    tkCaret: begin
      // ^A..^_ characters. See #16341
      NextToken;
      if not (length(CurTokenText)=1) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '^'+CurTokenText);
    end;
  else
    ParseExc(SParserExpectedIdentifier);
=======
  Result := TPasResString(CreateElement(TPasResString, CurTokenString, Parent));
  try
    ExpectToken(tkEqual);
    Result.Value := ParseExpression;
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
>>>>>>> origin/fixes_2_2
  end;

  if x.Kind<>pekSet then NextToken;

=======
  Result := TPasConst(CreateElement(TPasConst, CurTokenString, Parent));
>>>>>>> origin/cpstrnew
  try
    if x.Kind=pekIdent then begin
      while CurToken in [tkBraceOpen, tkSquaredBraceOpen, tkCaret] do
        case CurToken of
          tkBraceOpen: begin
            prm:=ParseParams(AParent,pekFuncParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkSquaredBraceOpen: begin
            prm:=ParseParams(AParent,pekArrayParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkCaret: begin
            u:=TUnaryExpr.Create(AParent,x, TokenToExprOp(CurToken));
            x:=u;
            NextToken;
          end;
        end;

<<<<<<< HEAD
      if CurToken in [tkDot, tkas] then begin
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then Exit; // error
        x:=b;
      end;
    end;

    if CurToken = tkDotDot then begin
      NextToken;
      b:=TBinaryExpr.CreateRange(AParent,x, DoParseExpression(AParent));
      if not Assigned(b.right) then Exit; // error
      x:=b;
    end;

    Result:=x;
  finally
    if not Assigned(Result) then x.Free;
=======
    ExpectToken(tkEqual);
<<<<<<< HEAD

    //skipping the expression as a value
    //Result.Value := ParseExpression;

    // using new expression parser!
    NextToken; // skip tkEqual
    Result.Expr:=DoParseConstValueExpression(Result);

    // must unget for the check to be peformed fine!
    UngetToken;

=======
    Result.Value := ParseExpression;
>>>>>>> graemeg/fixes_2_2
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
>>>>>>> origin/cpstrnew
  end;
end;

function TPasParser.OpLevel(t: TToken): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  case t of
    tknot,tkAt:
      Result:=4;
    tkMul, tkDivision, tkdiv, tkmod, tkand, tkShl,tkShr, tkas, tkPower :
      Result:=3;
    tkPlus, tkMinus, tkor, tkxor:
      Result:=2;
    tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan, tkGreaterThan, tkGreaterEqualThan, tkin, tkis:
      Result:=1;
  else
    Result:=0;
=======
  Result := TPasResString(CreateElement(TPasResString, CurTokenString, Parent));
  try
    ExpectToken(tkEqual);
<<<<<<< HEAD
    Result.Value := ParseExpression(Result);
=======
    Result.Value := ParseExpression;
>>>>>>> graemeg/fixes_2_2
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
>>>>>>> origin/cpstrnew
  end;
end;

function TPasParser.DoParseExpression(Aparent : TPaselement;InitExpr: TPasExpr): TPasExpr;
var
  expstack  : TList;
  opstack   : TList;
  pcount    : Integer;
  x         : TPasExpr;
  i         : Integer;
  tempop    : TToken;
  NotBinary : Boolean;
  
const
  PrefixSym = [tkPlus, tkMinus, tknot, tkAt]; // + - not @
  BinaryOP  = [tkMul, tkDivision, tkdiv, tkmod,
               tkand, tkShl,tkShr, tkas, tkPower,
               tkPlus, tkMinus, tkor, tkxor, tkSymmetricalDifference,
               tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan,
               tkGreaterThan, tkGreaterEqualThan, tkin, tkis];

  function PopExp: TPasExpr; inline;
  begin
<<<<<<< HEAD
    if expstack.Count>0 then begin
      Result:=TPasExpr(expstack[expstack.Count-1]);
      expstack.Delete(expstack.Count-1);
    end else
      Result:=nil;
  end;

  procedure PushOper(token: TToken); inline;
  begin
<<<<<<< HEAD
    opstack.Add( Pointer(PtrInt(token)) );
  end;

  function PeekOper: TToken; inline;
  begin
    if opstack.Count>0 then Result:=TToken(PtrUInt(opstack[ opstack.Count-1]))
    else Result:=tkEOF
  end;

  function PopOper: TToken; inline;
  begin
    Result:=PeekOper;
    if Result<>tkEOF then opstack.Delete(opstack.Count-1);
  end;

  procedure PopAndPushOperator;
  var
    t       : TToken;
    xright  : TPasExpr;
    xleft   : TPasExpr;
  begin
    t:=PopOper;
    xright:=PopExp;
    xleft:=PopExp;
    expstack.Add(TBinaryExpr.Create(AParent,xleft, xright, TokenToExprOp(t)));
  end;

begin
  Result:=nil;
  expstack := TList.Create;
  opstack := TList.Create;
  try
    repeat
      NotBinary:=True;
      pcount:=0;

      if not Assigned(InitExpr) then
=======
    Result := TPasRangeType(CreateElement(TPasRangeType, TypeName, Parent));
    try
      TPasRangeType(Result).RangeStart := ParseExpression(Result);
      ExpectToken(tkDotDot);
      TPasRangeType(Result).RangeEnd := ParseExpression(Result);
=======
    Result := TPasRangeType(CreateElement(TPasRangeType, TypeName, Parent));
    try
      TPasRangeType(Result).RangeStart := ParseExpression;
      ExpectToken(tkDotDot);
      TPasRangeType(Result).RangeEnd := ParseExpression;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      CheckHint(Result,True);
    except
      Result.Free;
      raise;
    end;
  end;

var
  EnumValue: TPasEnumValue;
  Prefix : String;
  HadPackedModifier : Boolean;           // 12/04/04 - Dave - Added
  IsBitPacked : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
  
=======
=======
>>>>>>> origin/fixes_2_2

>>>>>>> graemeg/fixes_2_2
begin
  TypeName := CurTokenString;
  ExpectToken(tkEqual);
  NextToken;
  HadPackedModifier := False;     { Assume not present }
  if CurToken in [tkPacked,tkbitpacked] then     { If PACKED modifier }
     begin                        { Handle PACKED modifier for all situations }
     IsBitPacked:=CurToken=tkbitpacked;
     NextToken;                   { Move to next token for rest of parse }
     if CurToken in [tkArray, tkRecord, tkObject, tkClass] then  { If allowed }
       HadPackedModifier := True  { rememeber for later }
     else                         { otherwise, syntax error }
       ParseExc(Format(SParserExpectTokenError,['ARRAY, RECORD, OBJECT or CLASS']))
     end;
  case CurToken of
    tkRecord:
      begin
        Result := TPasRecordType(CreateElement(TPasRecordType, TypeName,
          Parent));
        try
          ParseRecordDecl(TPasRecordType(Result), False);
<<<<<<< HEAD
<<<<<<< HEAD
      CheckHint(Result,True);
=======
	  CheckHint(Result,True);
>>>>>>> graemeg/fixes_2_2
=======
	  CheckHint(Result,True);
>>>>>>> origin/fixes_2_2
          TPasRecordType(Result).IsPacked := HadPackedModifier;
          If HadPackedModifier then
            TPasRecordType(Result).IsBitPacked:=IsBitPacked;
        except
          Result.Free;
          raise;
        end;
      end;
    tkObject:
      begin
        Result := ParseClassDecl(Parent, TypeName, okObject);
        TPasClassType(Result).IsPacked := HadPackedModifier;
      end;
    tkClass:
      begin
        Result := ParseClassDecl(Parent, TypeName, okClass);
        { could be TPasClassOfType }
        if result is TPasClassType then
          TPasClassType(Result).IsPacked := HadPackedModifier;
      end;
    tkInterface:
      Result := ParseClassDecl(Parent, TypeName, okInterface);
    tkCaret:
      begin
        Result := TPasPointerType(CreateElement(TPasPointerType, TypeName,
          Parent));
        try
          TPasPointerType(Result).DestType := ParseType(nil);
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
        end;
      end;
    tkIdentifier:
>>>>>>> origin/cpstrnew
      begin
        // the first part of the expression has been parsed externally.
        // this is used by Constant Expresion parser (CEP) parsing only,
        // whenever it makes a false assuming on constant expression type.
        // i.e: SI_PAD_SIZE = ((128/sizeof(longint)) - 3);
        //
        // CEP assumes that it's array or record, because the expression
        // starts with "(". After the first part is parsed, the CEP meets "-"
        // that assures, it's not an array expression. The CEP should give the
        // first partback to the expression parser, to get the correct
        // token tree according to the operations priority.
        //
        // quite ugly. type information is required for CEP to work clean

        while CurToken in PrefixSym do begin
          PushOper(CurToken);
          inc(pcount);
          NextToken;
<<<<<<< HEAD
=======
          end
        else
          Prefix:='';
        if (CurToken = tkSemicolon) or IsCurTokenHint then
        begin
          UngetToken;
          UngetToken;
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName,
            Parent));
          try
            TPasAliasType(Result).DestType := ParseType(nil,Prefix);
            CheckHint(Result,True);
          except
            Result.Free;
            raise;
          end;
        end else if CurToken = tkSquaredBraceOpen then
        begin
          // !!!: Check for string type and store string length somewhere
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName,
            Parent));
          try
            TPasAliasType(Result).DestType :=
              TPasUnresolvedTypeRef.Create(CurTokenString, Parent);
            ParseExpression(Parent);
            ExpectToken(tkSquaredBraceClose);
            CheckHint(Result,True);
          except
            Result.Free;
            raise;
          end;
        end
        else  
        begin
          UngetToken;
          UngetToken;
          ParseRange;
        end;
      end;
    tkFile:
      begin
        Result := TPasFileType(CreateElement(TPasFileType, TypeName, Parent));
        Try
          ParseFileType(TPasFileType(Result));
          CheckHint(Result,True);
        Except
          Result.free;
          Raise;
        end;
      end;
    tkArray:
      begin
        Result := TPasArrayType(CreateElement(TPasArrayType, TypeName, Parent));
        try
          ParseArrayType(TPasArrayType(Result));
          TPasArrayType(Result).IsPacked := HadPackedModifier;
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
        end;
      end;
    tkSet:
      begin
        Result := TPasSetType(CreateElement(TPasSetType, TypeName, Parent));
        try
          ExpectToken(tkOf);
          TPasSetType(Result).EnumType := ParseType(Result);
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
>>>>>>> origin/cpstrnew
        end;

        if CurToken = tkBraceOpen then begin
          NextToken;
          x:=DoParseExpression(AParent);
          if CurToken<>tkBraceClose then Exit;
          NextToken;

          // for the expression like  (TObject(m)).Free;
          if CurToken = tkDot then begin
            NextToken;
<<<<<<< HEAD
            x:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(tkDot));
=======
            if CurToken = tkBraceClose then
              break
            else if CurToken in [tkEqual,tkAssign] then
              begin
              EnumValue.AssignedValue:=ParseExpression(result);
              NextToken;
              if CurToken = tkBraceClose then
                Break
              else if not (CurToken=tkComma) then
                ParseExc(SParserExpectedCommaRBracket);
              end
            else if not (CurToken=tkComma) then
              ParseExc(SParserExpectedCommaRBracket)
>>>>>>> origin/cpstrnew
          end;
<<<<<<< HEAD
<<<<<<< HEAD

        end else begin
          x:=ParseExpIdent(AParent);
=======
=======
>>>>>>> origin/fixes_2_2
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
>>>>>>> graemeg/fixes_2_2
        end;

        if not Assigned(x) then Exit;
        expstack.Add(x);
        for i:=1 to pcount do begin
          tempop:=PopOper;
          expstack.Add( TUnaryExpr.Create(AParent, PopExp, TokenToExprOp(tempop) ));
        end;

      end else
      begin
        expstack.Add(InitExpr);
        InitExpr:=nil;
      end;
<<<<<<< HEAD

      if (CurToken in BinaryOP) then begin
        // Adjusting order of the operations
        NotBinary:=False;
        tempop:=PeekOper;
        while (opstack.Count>0) and (OpLevel(tempop)>=OpLevel(CurToken)) do begin
          PopAndPushOperator;
          tempop:=PeekOper;
=======
    tkType:
      begin
        Result := TPasTypeAliasType(CreateElement(TPasTypeAliasType, TypeName,
          Parent));
        try
          TPasTypeAliasType(Result).DestType := ParseType(nil);
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
>>>>>>> graemeg/fixes_2_2
        end;
        PushOper(CurToken);
        NextToken;
      end;

    until NotBinary or isEndOfExp;

    if not NotBinary then ParseExc(SParserExpectedIdentifier);

    while opstack.Count>0 do PopAndPushOperator;

    // only 1 expression should be on the stack, at the end of the correct expression
    if expstack.Count=1 then Result:=TPasExpr(expstack[0]);

  finally
    if not Assigned(Result) then begin
      // expression error!
      for i:=0 to expstack.Count-1 do
        TObject(expstack[i]).Free;
=======
  Module:=nil;
  NextToken;
  case CurToken of
    tkUnit: ParseUnit(Module);
    tkProgram: ParseProgram(Module);
    else
      ParseExc(Format(SParserExpectTokenError, ['unit']));
  end;
end;

// Starts after the "unit" token
procedure TPasParser.ParseUnit(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasModule, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    CheckHint(Module,True);
//    ExpectToken(tkSemicolon);
    ExpectToken(tkInterface);
    ParseInterface;
  finally
    CurModule:=nil;
  end;
end;

<<<<<<< HEAD
// Starts after the "program" token
procedure TPasParser.ParseProgram(var Module: TPasModule);
=======
procedure TPasParser.ParseInlineVarDecl(Parent: TPasElement; VarList: TList;
  AVisibility: TPasMemberVisibility; ClosingBrace: Boolean);
var
  VarNames: TStringList;
  i: Integer;
  VarType: TPasType;
  VarEl: TPasVariable;
  H : TPasMemberHints;
<<<<<<< HEAD
<<<<<<< HEAD

>>>>>>> origin/cpstrnew
=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasProgram, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    NextToken;
    ParseImplementation;
  finally
    CurModule:=nil;
  end;
end;

<<<<<<< HEAD
// Starts after the "interface" token
procedure TPasParser.ParseInterface;
var
  Section: TInterfaceSection;
begin
  Section := TInterfaceSection(CreateElement(TInterfaceSection, '', CurModule));
  CurModule.InterfaceSection := Section;
  ParseDeclarations(Section);
end;
=======
    VarType := ParseComplexType(Parent);
>>>>>>> origin/cpstrnew

<<<<<<< HEAD
// Starts after the "implementation" token
procedure TPasParser.ParseImplementation;
var
  Section: TImplementationSection;
begin
  Section := TImplementationSection(CreateElement(TImplementationSection, '', CurModule));
  CurModule.ImplementationSection := Section;
  ParseDeclarations(Section);
end;

procedure TPasParser.ParseInitialization;
var
  Section: TInitializationSection;
  SubBlock: TPasImplElement;
begin
  Section := TInitializationSection(CreateElement(TInitializationSection, '', CurModule));
  CurModule.InitializationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      exit;
    end
    else if (CurToken=tkfinalization) then
    begin
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
=======
    H:=CheckHint(Nil,False);
    NextToken;

    H:=CheckHint(Nil,False);
    NextToken;

    for i := 0 to VarNames.Count - 1 do
    begin
      VarEl := TPasVariable(CreateElement(TPasVariable, VarNames[i], Parent,
        AVisibility));
      VarEl.VarType := VarType;
      VarEl.Hints:=H;
      if i > 0 then
        VarType.AddRef;
      VarList.Add(VarEl);
    end;
    // Records may be terminated with end, no semicolon
    if (CurToken <> tkEnd) and (CurToken <> tkSemicolon) and not
      (ClosingBrace and (CurToken = tkBraceClose)) then
      ParseExc(SParserExpectedSemiColonEnd);
  finally
    VarNames.Free;
  end;
>>>>>>> graemeg/fixes_2_2
end;

procedure TPasParser.ParseFinalization;
var
<<<<<<< HEAD
  Section: TFinalizationSection;
  SubBlock: TPasImplElement;
begin
  Section := TFinalizationSection(CreateElement(TFinalizationSection, '', CurModule));
  CurModule.FinalizationSection := Section;
  repeat
=======
  i: Integer;
  VarType: TPasType;
  Value, S: String;
  M: string;
  H : TPasMemberHints;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  LastVar : String;
>>>>>>> graemeg/fixes_2_2
=======
  LastVar : String;
>>>>>>> origin/fixes_2_2
begin
  while True do
  begin
    LastVar:=CurTokenString;
    List.Add(CreateElement(TPasVariable, CurTokenString, Parent));
>>>>>>> origin/cpstrnew
    NextToken;
<<<<<<< HEAD
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
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
  UngetToken;
end;

procedure TPasParser.ParseDeclarations(Declarations: TPasDeclarations);
var
  CurBlock: TDeclType;
  ConstEl: TPasConst;
  ResStrEl: TPasResString;
  TypeEl: TPasType;
  ClassEl: TPasClassType;
  List: TList;
  i,j: Integer;
  VarEl: TPasVariable;
  PropEl : TPasProperty;
begin
  CurBlock := declNone;
=======
    if CurToken = tkColon then
      break
    else if CurToken <> tkComma then
      ParseExc(SParserExpectedCommaColon);
    ExpectIdentifier;
  end;
  VarType := ParseComplexType;
  for i := 0 to List.Count - 1 do
  begin
    TPasVariable(List[i]).VarType := VarType;
    if i > 0 then
      VarType.AddRef;
  end;
//  Writeln(LastVar,': Parsed complex type: ',CurtokenText);
  // NextToken;
  // Writeln(LastVar,': Parsed complex type, next: ',CurtokenText);
  If CurToken=tkEqual then
    begin
    NextToken;
    Value := ParseExpression;
  //  Writeln(LastVar,' Value :',Value);
    for i := 0 to List.Count - 1 do
      TPasVariable(List[i]).Value := Value;
//    Writeln(LastVar,': Parsed expression:',CurTokenText);
    end
{  else
    UngetToken};

  NextToken;
  if CurToken = tkAbsolute then
  begin
    ExpectIdentifier;
    S:=CurTokenText;
    NextToken;
    if CurToken=tkDot then
      begin
      ExpectIdentifier;
      S:=S+'.'+CurTokenText;
      end
    else
      UnGetToken;
    For I:=0 to List.Count-1 do
      TPasVariable(List[i]).AbsoluteLocation:=S;
  end else
    UngetToken;

  H:=CheckHint(Nil,True);
  If (H<>[]) then
    for i := 0 to List.Count - 1 do
      TPasVariable(List[i]).Hints:=H;
  M := '';
>>>>>>> graemeg/fixes_2_2
  while True do
  begin
    NextToken;
    //writeln('TPasParser.ParseSection Token=',Scanner.CurTokenString,' ',CurToken);
    case CurToken of
      tkend:
        begin
        ExpectToken(tkDot);
        break;
        end;
      tkimplementation:
        if (CurToken = tkImplementation) and (Declarations is TInterfaceSection) then
          begin
          If Not Engine.InterfaceOnly then
            ParseImplementation;
          break;
          end;
      tkinitialization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
<<<<<<< HEAD
          ParseInitialization;
          break;
          end;
      tkfinalization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseFinalization;
          break;
          end;
      tkUses:
        if Declarations is TPasSection then
          ParseUsesList(TPasSection(Declarations))
        else
=======
          M := M + ' name ';
          NextToken;
          if (CurToken = tkString) or (CurToken = tkIdentifier) then
            // !!!: Is this really correct for tkString?
            M := M + CurTokenText
          else
            ParseExc(SParserSyntaxError);
          H:=CheckHint(Nil,True);
          If (H<>[]) then
            for i := 0 to List.Count - 1 do
              TPasVariable(List[i]).Hints:=H;
          
          // ExpectToken(tkSemicolon);
          end
        else if CurToken <> tkSemicolon then
>>>>>>> graemeg/fixes_2_2
          ParseExc(SParserSyntaxError);
      tkConst:
        CurBlock := declConst;
      tkResourcestring:
        CurBlock := declResourcestring;
      tkType:
        CurBlock := declType;
      tkVar:
        CurBlock := declVar;
      tkThreadVar:
        CurBlock := declThreadVar;
      tkProperty:
        CurBlock := declProperty;
      tkProcedure:
        begin
          AddProcOrFunction(Declarations,
                       ParseProcedureOrFunctionDecl(Declarations, ptProcedure));
          CurBlock := declNone;
        end;
      tkFunction:
        begin
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptFunction));
          CurBlock := declNone;
        end;
      tkConstructor:
        begin
          AddProcOrFunction(Declarations,
                     ParseProcedureOrFunctionDecl(Declarations, ptConstructor));
          CurBlock := declNone;
        end;
      tkDestructor:
        begin
          AddProcOrFunction(Declarations,
                      ParseProcedureOrFunctionDecl(Declarations, ptDestructor));
          CurBlock := declNone;
        end;
      tkOperator:
        begin
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptOperator));
          CurBlock := declNone;
        end;
      tkClass:
        begin
          NextToken;
          case CurToken of
            tkprocedure:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassProcedure));
                CurBlock := declNone;
              end;
            tkfunction:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassFunction));
                CurBlock := declNone;
              end;
          else
            ExpectToken(tkprocedure);
          end;
        end;
      tkIdentifier:
        begin
          case CurBlock of
            declConst:
              begin
                ConstEl := ParseConstDecl(Declarations);
                Declarations.Declarations.Add(ConstEl);
                Declarations.Consts.Add(ConstEl);
              end;
            declResourcestring:
              begin
                ResStrEl := ParseResourcestringDecl(Declarations);
                Declarations.Declarations.Add(ResStrEl);
                Declarations.ResStrings.Add(ResStrEl);
              end;
            declType:
              begin
                TypeEl := ParseTypeDecl(Declarations);
                if Assigned(TypeEl) then        // !!!
                begin
                  Declarations.Declarations.Add(TypeEl);
                  if TypeEl.ClassType = TPasClassType then
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
                        ClassEl.Release;
                        break;
                      end;
                    end;
                    // Add the new class to the class list
                    Declarations.Classes.Add(TypeEl)
                  end else
                    Declarations.Types.Add(TypeEl);
                end;
              end;
            declVar, declThreadVar:
              begin
                List := TList.Create;
                try
                  try
                    ParseVarDecl(Declarations, List);
                  except
                    for i := 0 to List.Count - 1 do
                      TPasVariable(List[i]).Release;
                    raise;
                  end;
                  for i := 0 to List.Count - 1 do
                  begin
                    VarEl := TPasVariable(List[i]);
                    Declarations.Declarations.Add(VarEl);
                    Declarations.Variables.Add(VarEl);
                  end;
                finally
                  List.Free;
                end;
              end;
            declProperty:
              begin
              PropEl:=TPasProperty(CreateElement(TPasProperty, CurTokenString, Declarations));
              Try
                ParseProperty(PropEl)
              except
                Propel.Free;
                Raise;
              end;
              Declarations.Declarations.Add(PropEl);
              Declarations.properties.add(PropEl);
              end;
          else
            ParseExc(SParserSyntaxError);
          end;
        end;
      tkbegin:
        begin
        if Declarations is TProcedureBody then
          begin
          ParseProcBeginBlock(TProcedureBody(Declarations));
          break;
          end
        else if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end
        else
          ParseExc(SParserSyntaxError);
        end;
      tklabel:
        begin
          if not (Declarations is TInterfaceSection) then
            ParseLabels(Declarations);
        end;
    else
      ParseExc(SParserSyntaxError);
>>>>>>> graemeg/cpstrnew
    end;
    opstack.Free;
    expstack.Free;
  end;
end;

<<<<<<< HEAD
function TPasParser.ParseExpression(Aparent : TPaselement;Kind: TExprKind): String;
var
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
  ls: String;
begin
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) then 
    begin
      if (CurToken in [tkComma, tkSemicolon,
        tkColon, tkDotDot, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept,
        tkof, tkbegin, tkdo, tkto, tkdownto, tkinitialization, tkfinalization])
      then
        break;
        
      if (Kind=ek_PropertyIndex) and (CurToken=tkIdentifier) then begin
        ls:=LowerCase(CurTokenText);
        if (ls='read') or (ls ='write') or (ls='default') or (ls='nodefault') or (ls='implements') then
          Break;
      end;
        
    end;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
    if CurToken=tkString then
      begin
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
      end
    else
      Result := Result + CurTokenText;
  end;
  if Result='' then
    ParseExc(SParserSyntaxError);
  UngetToken;
end;
=======
// Starts after the "uses" token
procedure TPasParser.ParseUsesList(ASection: TPasSection);

function CheckUnit(AUnitName : string):TPasElement;
begin
    result := Engine.FindModule(AUnitName);  // should we resolve module here when "IN" filename is not known yet?
    if Assigned(result) then
      result.AddRef
    else
      Result := TPasType(CreateElement(TPasUnresolvedTypeRef, AUnitName,
        ASection));
    ASection.UsesList.Add(Result);
end;

var
  AUnitName: String;
  Element: TPasElement;
begin
  If not (Asection is TImplementationSection) Then // interface,program,library,package
    Element:=CheckUnit('System'); // system always implicitely first.    
  while True do
  begin
    AUnitName := ExpectIdentifier; 
    Element :=CheckUnit(AUnitName);
>>>>>>> graemeg/cpstrnew

function GetExprIdent(p: TPasExpr): String;
begin
  if Assigned(p) and (p is TPrimitiveExpr) and (p.Kind=pekIdent) then
    Result:=TPrimitiveExpr(p).Value
  else
    Result:='';
end;

function TPasParser.DoParseConstValueExpression(Aparent : TPaselement): TPasExpr;
var
  x : TPasExpr;
  n : AnsiString;
  r : TRecordValues;
  a : TArrayValues;

function lastfield:boolean;

begin
  result:= CurToken<>tkSemicolon;
  if not result then
   begin
     nexttoken;
     if curtoken=tkbraceclose then
       result:=true
     else
       ungettoken;
   end; 
end;

begin
  if CurToken <> tkBraceOpen then
    Result:=DoParseExpression(AParent)
  else begin
    NextToken;
    x:=DoParseConstValueExpression(Aparent);
    case CurToken of
      tkComma: // array of values (a,b,c);
        begin
          a:=TArrayValues.Create(AParent);
          a.AddValues(x);
          repeat
            NextToken;
            x:=DoParseConstValueExpression(AParent);
            a.AddValues(x);
          until CurToken<>tkComma;
          Result:=a;
        end;

      tkColon: // record field (a:xxx;b:yyy;c:zzz);
        begin
          n:=GetExprIdent(x);
          x.Free;
          r:=TRecordValues.Create(AParent);
          NextToken;
          x:=DoParseConstValueExpression(AParent);
          r.AddField(n, x);
          if not lastfield then
            repeat
              n:=ExpectIdentifier;
              ExpectToken(tkColon);
              NextToken;
              x:=DoParseConstValueExpression(AParent);
              r.AddField(n, x)
            until lastfield; // CurToken<>tkSemicolon;
          Result:=r;
        end;
    else
      // Binary expression!  ((128 div sizeof(longint)) - 3);       ;
      Result:=DoParseExpression(AParent,x);
    end;
    if CurToken<>tkBraceClose then ParseExc(SParserExpectedCommaRBracket);
    NextToken;
<<<<<<< HEAD
=======

    if CurToken = tkin then begin
      // todo: store unit's file name somewhere
      NextToken; // skip in
      ExpectToken(tkString); // skip unit's real file name
      if (Element is TPasModule) and (TPasmodule(Element).filename<>'')  then
        TPasModule(Element).FileName:=curtokenstring;
    end;

    if CurToken = tkSemicolon then
      break
    else if CurToken <> tkComma then
      ParseExc(SParserExpectedCommaSemicolon);
>>>>>>> graemeg/cpstrnew
  end;
end;

function TPasParser.ParseCommand: String;
var
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
begin
<<<<<<< HEAD
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) and (CurToken in [tkComma, tkSemicolon,
      tkColon, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept, tkof, tkdo,
      tkbegin, tkinitialization, tkfinalization]) then
      break;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
    if CurToken=tkString then
      begin
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
      end
    else
      Result := Result + CurTokenText;
  end;
  UngetToken;
end;

procedure TPasParser.AddProcOrFunction(Declarations: TPasDeclarations;
  AProc: TPasProcedure);
>>>>>>> graemeg/cpstrnew
var
  params  : TParamsExpr;
  p       : TPasExpr;
  PClose  : TToken;
begin
<<<<<<< HEAD
  Result:=nil;
  if paramskind in [pekArrayParams, pekSet] then begin
    if CurToken<>tkSquaredBraceOpen then Exit;
    PClose:=tkSquaredBraceClose;
  end else begin
    if CurToken<>tkBraceOpen then Exit;
    PClose:=tkBraceClose;
  end;

  params:=TParamsExpr.Create(AParent,paramskind);
=======
  Result := TPasConst(CreateElement(TPasConst, CurTokenString, Parent));
>>>>>>> graemeg/cpstrnew
  try
    NextToken;
    if not isEndOfExp then begin
      repeat
        p:=DoParseExpression(AParent);
        if not Assigned(p) then Exit; // bad param syntax
        params.AddParam(p);
=======
  for i := 0 to Declarations.Functions.Count - 1 do
  begin
    Member := TPasElement(Declarations.Functions[i]);
    if CompareText(Member.Name, AProc.Name) = 0 then
    begin
      if Member.ClassType = TPasOverloadedProc then
        TPasOverloadedProc(Member).Overloads.Add(AProc)
      else
      begin
        OverloadedProc := TPasOverloadedProc.Create(AProc.Name, Declarations);
        OverloadedProc.Overloads.Add(Member);
        OverloadedProc.Overloads.Add(AProc);
        Declarations.Functions[i] := OverloadedProc;
        Declarations.Declarations[Declarations.Declarations.IndexOf(Member)] :=
          OverloadedProc;
      end;
      exit;
    end;
  end;

<<<<<<< HEAD
  // Not overloaded, so just add the proc/function to the lists
  Declarations.Declarations.Add(AProc);
  Declarations.Functions.Add(AProc);
end;
>>>>>>> graemeg/cpstrnew

        if not (CurToken in [tkComma, PClose]) then begin
          Exit;
        end;

        if CurToken = tkComma then begin
          NextToken;
          if CurToken = PClose then begin
            //ErrorExpected(parser, 'identifier');
            Exit;
          end;
        end;
      until CurToken=PClose;
    end;
    NextToken;
    Result:=params;
  finally
    if not Assigned(Result) then params.Free;
=======
    ExpectToken(tkEqual);

    //skipping the expression as a value
    //Result.Value := ParseExpression;

    // using new expression parser!
    NextToken; // skip tkEqual
    Result.Expr:=DoParseConstValueExpression(Result);

    // must unget for the check to be peformed fine!
    UngetToken;

    CheckHint(Result,True);
  except
    Result.Free;
    raise;
>>>>>>> graemeg/cpstrnew
  end;
end;

Function TPasParser.TokenToExprOp (AToken : TToken) : TExprOpCode;

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
    ParseExc(format('Not an operand: (%d : %s)',[AToken,TokenInfos[AToken]]));
  end;
end;
 
function TPasParser.ParseExpIdent(AParent : TPasElement):TPasExpr;
var
  x       : TPasExpr;
  prm     : TParamsExpr;
  u       : TUnaryExpr;
  b       : TBinaryExpr;
  optk    : TToken;
begin
  Result:=nil;
  case CurToken of
    tkString:           x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenString);
    tkChar:             x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText);
    tkNumber:           x:=TPrimitiveExpr.Create(AParent,pekNumber, CurTokenString);
    tkIdentifier:       x:=TPrimitiveExpr.Create(AParent,pekIdent, CurTokenText);
    tkfalse, tktrue:    x:=TBoolConstExpr.Create(Aparent,pekBoolConst, CurToken=tktrue);
    tknil:              x:=TNilExpr.Create(Aparent);
    tkSquaredBraceOpen: x:=ParseParams(AParent,pekSet);
    tkinherited: begin
      //inherited; inherited function
      x:=TInheritedExpr.Create(AParent);
      NextToken;
      if (length(CurTokenText)>0) and (CurTokenText[1] in ['A'..'_']) then begin
        b:=TBinaryExpr.Create(AParent,x, DoParseExpression(AParent), eopNone);
        if not Assigned(b.right) then Exit; // error
        x:=b;
        UngetToken;
      end
       else UngetToken;
    end;
    tkself: begin
      x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText); //function(self);
      x:=TSelfExpr.Create(AParent);
      NextToken;
      if CurToken = tkDot then begin // self.Write(EscapeText(AText));
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then Exit; // error
        x:=b;
      end
       else UngetToken;
    end;
    tkAt: begin
      // P:=@function;
      NextToken;
      if (length(CurTokenText)=0) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '@'+CurTokenText);
    end;
    tkCaret: begin
      // ^A..^_ characters. See #16341
      NextToken;
      if not (length(CurTokenText)=1) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '^'+CurTokenText);
    end;
  else
    ParseExc(SParserExpectedIdentifier);
  end;

  if x.Kind<>pekSet then NextToken;

  try
<<<<<<< HEAD
    if x.Kind=pekIdent then begin
      while CurToken in [tkBraceOpen, tkSquaredBraceOpen, tkCaret] do
        case CurToken of
          tkBraceOpen: begin
            prm:=ParseParams(AParent,pekFuncParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkSquaredBraceOpen: begin
            prm:=ParseParams(AParent,pekArrayParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkCaret: begin
            u:=TUnaryExpr.Create(AParent,x, TokenToExprOp(CurToken));
            x:=u;
            NextToken;
          end;
        end;

      if CurToken in [tkDot, tkas] then begin
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then Exit; // error
        x:=b;
      end;
    end;

    if CurToken = tkDotDot then begin
      NextToken;
      b:=TBinaryExpr.CreateRange(AParent,x, DoParseExpression(AParent));
      if not Assigned(b.right) then Exit; // error
      x:=b;
    end;

    Result:=x;
  finally
    if not Assigned(Result) then x.Free;
=======
    ExpectToken(tkEqual);
    Result.Value := ParseExpression(Result);
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
>>>>>>> graemeg/cpstrnew
  end;
end;

function TPasParser.OpLevel(t: TToken): Integer;
begin
  case t of
    tknot,tkAt:
      Result:=4;
    tkMul, tkDivision, tkdiv, tkmod, tkand, tkShl,tkShr, tkas, tkPower :
      Result:=3;
    tkPlus, tkMinus, tkor, tkxor:
      Result:=2;
    tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan, tkGreaterThan, tkGreaterEqualThan, tkin, tkis:
      Result:=1;
  else
    Result:=0;
  end;
end;

function TPasParser.DoParseExpression(Aparent : TPaselement;InitExpr: TPasExpr): TPasExpr;
var
  expstack  : TList;
  opstack   : TList;
  pcount    : Integer;
  x         : TPasExpr;
  i         : Integer;
  tempop    : TToken;
  NotBinary : Boolean;
  
const
  PrefixSym = [tkPlus, tkMinus, tknot, tkAt]; // + - not @
  BinaryOP  = [tkMul, tkDivision, tkdiv, tkmod,
               tkand, tkShl,tkShr, tkas, tkPower,
               tkPlus, tkMinus, tkor, tkxor, tkSymmetricalDifference,
               tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan,
               tkGreaterThan, tkGreaterEqualThan, tkin, tkis];

  function PopExp: TPasExpr; inline;
  begin
    if expstack.Count>0 then begin
      Result:=TPasExpr(expstack[expstack.Count-1]);
      expstack.Delete(expstack.Count-1);
    end else
      Result:=nil;
  end;

  procedure PushOper(token: TToken); inline;
  begin
<<<<<<< HEAD
    opstack.Add( Pointer(PtrInt(token)) );
  end;

  function PeekOper: TToken; inline;
  begin
    if opstack.Count>0 then Result:=TToken(PtrUInt(opstack[ opstack.Count-1]))
    else Result:=tkEOF
  end;

  function PopOper: TToken; inline;
  begin
    Result:=PeekOper;
    if Result<>tkEOF then opstack.Delete(opstack.Count-1);
  end;

  procedure PopAndPushOperator;
  var
    t       : TToken;
    xright  : TPasExpr;
    xleft   : TPasExpr;
  begin
    t:=PopOper;
    xright:=PopExp;
    xleft:=PopExp;
    expstack.Add(TBinaryExpr.Create(AParent,xleft, xright, TokenToExprOp(t)));
  end;

=======
    Result := TPasRangeType(CreateElement(TPasRangeType, TypeName, Parent));
    try
      TPasRangeType(Result).RangeStart := ParseExpression(Result);
      ExpectToken(tkDotDot);
      TPasRangeType(Result).RangeEnd := ParseExpression(Result);
      CheckHint(Result,True);
    except
      Result.Free;
      raise;
    end;
  end;

var
  EnumValue: TPasEnumValue;
  Prefix : String;
  HadPackedModifier : Boolean;           // 12/04/04 - Dave - Added
  IsBitPacked : Boolean;
  
>>>>>>> graemeg/cpstrnew
begin
  Result:=nil;
  expstack := TList.Create;
  opstack := TList.Create;
  try
    repeat
      NotBinary:=True;
      pcount:=0;

      if not Assigned(InitExpr) then
      begin
<<<<<<< HEAD
        // the first part of the expression has been parsed externally.
        // this is used by Constant Expresion parser (CEP) parsing only,
        // whenever it makes a false assuming on constant expression type.
        // i.e: SI_PAD_SIZE = ((128/sizeof(longint)) - 3);
        //
        // CEP assumes that it's array or record, because the expression
        // starts with "(". After the first part is parsed, the CEP meets "-"
        // that assures, it's not an array expression. The CEP should give the
        // first partback to the expression parser, to get the correct
        // token tree according to the operations priority.
        //
        // quite ugly. type information is required for CEP to work clean

        while CurToken in PrefixSym do begin
          PushOper(CurToken);
          inc(pcount);
          NextToken;
=======
        Result := TPasRecordType(CreateElement(TPasRecordType, TypeName,
          Parent));
        try
          ParseRecordDecl(TPasRecordType(Result), False);
      CheckHint(Result,True);
          TPasRecordType(Result).IsPacked := HadPackedModifier;
          If HadPackedModifier then
            TPasRecordType(Result).IsBitPacked:=IsBitPacked;
        except
          Result.Free;
          raise;
>>>>>>> graemeg/cpstrnew
        end;

        if CurToken = tkBraceOpen then begin
          NextToken;
          x:=DoParseExpression(AParent);
          if CurToken<>tkBraceClose then Exit;
          NextToken;

          // for the expression like  (TObject(m)).Free;
          if CurToken = tkDot then begin
            NextToken;
            x:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(tkDot));
          end;

        end else begin
          x:=ParseExpIdent(AParent);
        end;

        if not Assigned(x) then Exit;
        expstack.Add(x);
        for i:=1 to pcount do begin
          tempop:=PopOper;
          expstack.Add( TUnaryExpr.Create(AParent, PopExp, TokenToExprOp(tempop) ));
        end;

      end else
      begin
        expstack.Add(InitExpr);
        InitExpr:=nil;
      end;

      if (CurToken in BinaryOP) then begin
        // Adjusting order of the operations
        NotBinary:=False;
        tempop:=PeekOper;
        while (opstack.Count>0) and (OpLevel(tempop)>=OpLevel(CurToken)) do begin
          PopAndPushOperator;
          tempop:=PeekOper;
        end;
        PushOper(CurToken);
        NextToken;
<<<<<<< HEAD
=======
        if CurToken = tkDot then
          begin
          ExpectIdentifier;
          NextToken;
          end
        else
          Prefix:='';
        if (CurToken = tkSemicolon) or IsCurTokenHint then
        begin
          UngetToken;
          UngetToken;
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName,
            Parent));
          try
            TPasAliasType(Result).DestType := ParseType(nil,Prefix);
            CheckHint(Result,True);
          except
            Result.Free;
            raise;
          end;
        end else if CurToken = tkSquaredBraceOpen then
        begin
          // !!!: Check for string type and store string length somewhere
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName,
            Parent));
          try
            TPasAliasType(Result).DestType :=
              TPasUnresolvedTypeRef.Create(CurTokenString, Parent);
            ParseExpression(Parent);
            ExpectToken(tkSquaredBraceClose);
            CheckHint(Result,True);
          except
            Result.Free;
            raise;
          end;
        end
        else  
        begin
          UngetToken;
          UngetToken;
          ParseRange;
        end;
      end;
    tkFile:
      begin
        Result := TPasFileType(CreateElement(TPasFileType, TypeName, Parent));
        Try
          ParseFileType(TPasFileType(Result));
          CheckHint(Result,True);
        Except
          Result.free;
          Raise;
        end;
      end;
    tkArray:
      begin
        Result := TPasArrayType(CreateElement(TPasArrayType, TypeName, Parent));
        try
          ParseArrayType(TPasArrayType(Result));
          TPasArrayType(Result).IsPacked := HadPackedModifier;
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
        end;
      end;
    tkSet:
      begin
        Result := TPasSetType(CreateElement(TPasSetType, TypeName, Parent));
        try
          ExpectToken(tkOf);
          TPasSetType(Result).EnumType := ParseType(Result);
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
        end;
      end;
    tkBraceOpen:
      begin
        Result := TPasEnumType(CreateElement(TPasEnumType, TypeName, Parent));
        try
          while True do
          begin
            NextToken;
            EnumValue := TPasEnumValue(CreateElement(TPasEnumValue,
              CurTokenString, Result));
            TPasEnumType(Result).Values.Add(EnumValue);
            NextToken;
            if CurToken = tkBraceClose then
              break
            else if CurToken in [tkEqual,tkAssign] then
              begin
              EnumValue.AssignedValue:=ParseExpression(result);
              NextToken;
              if CurToken = tkBraceClose then
                Break
              else if not (CurToken=tkComma) then
                ParseExc(SParserExpectedCommaRBracket);
              end
            else if not (CurToken=tkComma) then
              ParseExc(SParserExpectedCommaRBracket)
          end;
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
        end;
>>>>>>> graemeg/cpstrnew
      end;

    until NotBinary or isEndOfExp;

    if not NotBinary then ParseExc(SParserExpectedIdentifier);

    while opstack.Count>0 do PopAndPushOperator;

    // only 1 expression should be on the stack, at the end of the correct expression
    if expstack.Count=1 then Result:=TPasExpr(expstack[0]);

  finally
    if not Assigned(Result) then begin
      // expression error!
      for i:=0 to expstack.Count-1 do
        TObject(expstack[i]).Free;
    end;
    opstack.Free;
    expstack.Free;
  end;
end;

function TPasParser.ParseExpression(Aparent : TPaselement;Kind: TExprKind): String;
var
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
  ls: String;
begin
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) then 
    begin
      if (CurToken in [tkComma, tkSemicolon,
        tkColon, tkDotDot, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept,
        tkof, tkbegin, tkdo, tkto, tkdownto, tkinitialization, tkfinalization])
      then
        break;
        
      if (Kind=ek_PropertyIndex) and (CurToken=tkIdentifier) then begin
        ls:=LowerCase(CurTokenText);
        if (ls='read') or (ls ='write') or (ls='default') or (ls='nodefault') or (ls='implements') then
          Break;
      end;
        
    end;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
    if CurToken=tkString then
      begin
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
      end
    else
      Result := Result + CurTokenText;
  end;
  if Result='' then
    ParseExc(SParserSyntaxError);
  UngetToken;
end;

function GetExprIdent(p: TPasExpr): String;
begin
  if Assigned(p) and (p is TPrimitiveExpr) and (p.Kind=pekIdent) then
    Result:=TPrimitiveExpr(p).Value
  else
    Result:='';
end;

function TPasParser.DoParseConstValueExpression(Aparent : TPaselement): TPasExpr;
var
<<<<<<< HEAD
  x : TPasExpr;
  n : AnsiString;
  r : TRecordValues;
  a : TArrayValues;
=======
  VarNames: TStringList;
  i: Integer;
  VarType: TPasType;
  VarEl: TPasVariable;
  H : TPasMemberHints;

begin
  VarNames := TStringList.Create;
  try
    while True do
    begin
      VarNames.Add(CurTokenString);
      NextToken;
      if CurToken = tkColon then
        break
      else if CurToken <> tkComma then
        ParseExc(SParserExpectedCommaColon);
>>>>>>> graemeg/cpstrnew

function lastfield:boolean;

<<<<<<< HEAD
begin
  result:= CurToken<>tkSemicolon;
  if not result then
   begin
     nexttoken;
     if curtoken=tkbraceclose then
       result:=true
     else
       ungettoken;
   end; 
end;
=======
    VarType := ParseComplexType(Parent);
>>>>>>> graemeg/cpstrnew

begin
  if CurToken <> tkBraceOpen then
    Result:=DoParseExpression(AParent)
  else begin
    NextToken;
    x:=DoParseConstValueExpression(Aparent);
    case CurToken of
      tkComma: // array of values (a,b,c);
        begin
          a:=TArrayValues.Create(AParent);
          a.AddValues(x);
          repeat
            NextToken;
            x:=DoParseConstValueExpression(AParent);
            a.AddValues(x);
          until CurToken<>tkComma;
          Result:=a;
        end;

      tkColon: // record field (a:xxx;b:yyy;c:zzz);
        begin
          n:=GetExprIdent(x);
          x.Free;
          r:=TRecordValues.Create(AParent);
          NextToken;
          x:=DoParseConstValueExpression(AParent);
          r.AddField(n, x);
          if not lastfield then
            repeat
              n:=ExpectIdentifier;
              ExpectToken(tkColon);
              NextToken;
              x:=DoParseConstValueExpression(AParent);
              r.AddField(n, x)
            until lastfield; // CurToken<>tkSemicolon;
          Result:=r;
        end;
    else
      // Binary expression!  ((128 div sizeof(longint)) - 3);       ;
      Result:=DoParseExpression(AParent,x);
    end;
    if CurToken<>tkBraceClose then ParseExc(SParserExpectedCommaRBracket);
    NextToken;
  end;
end;

function TPasParser.ParseCommand: String;
var
<<<<<<< HEAD
  BracketLevel: Integer;
  LastTokenWasWord: Boolean;
=======
  i: Integer;
  VarType: TPasType;
  Value, S: String;
  M: string;
  H : TPasMemberHints;
>>>>>>> graemeg/cpstrnew
begin
  SetLength(Result, 0);
  BracketLevel := 0;
  LastTokenWasWord := false;
  while True do
  begin
<<<<<<< HEAD
    NextToken;
    { !!!: Does not detect when normal brackets and square brackets are mixed
      in a wrong way. }
    if CurToken in [tkBraceOpen, tkSquaredBraceOpen] then
      Inc(BracketLevel)
    else if CurToken in [tkBraceClose, tkSquaredBraceClose] then
    begin
      if BracketLevel = 0 then
        break;
      Dec(BracketLevel);
    end else if (BracketLevel = 0) and (CurToken in [tkComma, tkSemicolon,
      tkColon, tkthen, tkend, tkelse, tkuntil, tkfinally, tkexcept, tkof, tkdo,
      tkbegin, tkinitialization, tkfinalization]) then
      break;

    if (CurTokenString<>'') and IsIdentStart[CurTokenString[1]] then
=======
    List.Add(CreateElement(TPasVariable, CurTokenString, Parent));
    NextToken;
    if CurToken = tkColon then
      break
    else if CurToken <> tkComma then
      ParseExc(SParserExpectedCommaColon);
    ExpectIdentifier;
  end;
  VarType := ParseComplexType;
  for i := 0 to List.Count - 1 do
  begin
    TPasVariable(List[i]).VarType := VarType;
    if i > 0 then
      VarType.AddRef;
  end;
  //writeln('TPasParser.ParseVarDecl ',CurTokenText);
  NextToken;
  // Writeln(LastVar,': Parsed complex type: ',CurtokenText);
  // NextToken;
  // Writeln(LastVar,': Parsed complex type, next: ',CurtokenText);
  If CurToken=tkEqual then
    begin
    Value := ParseExpression(Parent);
    for i := 0 to List.Count - 1 do
      TPasVariable(List[i]).Value := Value;
    NextToken;
    end;

  if CurToken = tkAbsolute then
  begin
    ExpectIdentifier;
    S:=CurTokenText;
    NextToken;
    if CurToken=tkDot then
>>>>>>> graemeg/cpstrnew
      begin
      if LastTokenWasWord then
        Result := Result + ' ';
      LastTokenWasWord:=true;
      end
    else
      LastTokenWasWord:=false;
    if CurToken=tkString then
      begin
<<<<<<< HEAD
      If (Length(CurTokenText)>0) and (CurTokenText[1]=#0) then
        Raise Exception.Create('First char is null : "'+CurTokenText+'"');
      Result := Result + ''''+StringReplace(CurTokenText,'''','''''',[rfReplaceAll])+''''
=======
      s := UpperCase(CurTokenText);
      if s = 'CVAR' then
        begin
        M := M + '; cvar';
        ExpectToken(tkSemicolon);
        end
      else if (s = 'EXTERNAL') or (s = 'PUBLIC') or (s = 'EXPORT') then
        begin
        M := M + ';' + CurTokenText;
        if s = 'EXTERNAL' then
          begin
          NextToken;
          if ((CurToken = tkString) or (CurToken = tkIdentifier)) and (UpperCase(CurTokenText)<> 'NAME') then
            begin
            // !!!: Is this really correct for tkString?
            M := M + ' ' + CurTokenText;
            NextToken;
            end;
          end
        else
          NextToken;
        if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'NAME') then
          begin
          M := M + ' name ';
          NextToken;
          if (CurToken = tkString) or (CurToken = tkIdentifier) then
            // !!!: Is this really correct for tkString?
            M := M + CurTokenText
          else
            ParseExc(SParserSyntaxError);
          H:=CheckHint(Nil,True);
          If (H<>[]) then
            for i := 0 to List.Count - 1 do
              TPasVariable(List[i]).Hints:=H;

          // ExpectToken(tkSemicolon);
          end
        else if CurToken <> tkSemicolon then
          ParseExc(SParserSyntaxError);
      end else
      begin
        UngetToken;
        break;
>>>>>>> graemeg/cpstrnew
      end
    else
      Result := Result + CurTokenText;
>>>>>>> graemeg/cpstrnew
  end;
end;

<<<<<<< HEAD
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
    ParseExc(format('Not an operand: (%d : %s)',[AToken,TokenInfos[AToken]]));
  end;
end;
 
function TPasParser.ParseExpIdent(AParent : TPasElement):TPasExpr;
=======
procedure TPasParser.AddProcOrFunction(Declarations: TPasDeclarations;
  AProc: TPasProcedure);
>>>>>>> graemeg/cpstrnew
var
  x       : TPasExpr;
  prm     : TParamsExpr;
  u       : TUnaryExpr;
  b       : TBinaryExpr;
  optk    : TToken;
begin
<<<<<<< HEAD
  Result:=nil;
  case CurToken of
    tkString:           x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenString);
    tkChar:             x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText);
    tkNumber:           x:=TPrimitiveExpr.Create(AParent,pekNumber, CurTokenString);
    tkIdentifier:       x:=TPrimitiveExpr.Create(AParent,pekIdent, CurTokenText);
    tkfalse, tktrue:    x:=TBoolConstExpr.Create(Aparent,pekBoolConst, CurToken=tktrue);
    tknil:              x:=TNilExpr.Create(Aparent);
    tkSquaredBraceOpen: x:=ParseParams(AParent,pekSet);
    tkinherited:
      begin
      //inherited; inherited function
      x:=TInheritedExpr.Create(AParent);
      NextToken;
      if (CurToken=tkIdentifier) then
        begin
        b:=TBinaryExpr.Create(AParent,x, DoParseExpression(AParent), eopNone);
        if not Assigned(b.right) then
          begin
          B.Free;
          Exit; // error
          end;
        x:=b;
        UngetToken;
        end
      else
        UngetToken;
=======
  for i := 0 to Declarations.Functions.Count - 1 do
  begin
    Member := TPasElement(Declarations.Functions[i]);
    if CompareText(Member.Name, AProc.Name) = 0 then
    begin
      if Member.ClassType = TPasOverloadedProc then
        TPasOverloadedProc(Member).Overloads.Add(AProc)
      else
      begin
        OverloadedProc := TPasOverloadedProc.Create(AProc.Name, Declarations);
        OverloadedProc.Overloads.Add(Member);
        OverloadedProc.Overloads.Add(AProc);
        Declarations.Functions[i] := OverloadedProc;
        Declarations.Declarations[Declarations.Declarations.IndexOf(Member)] :=
          OverloadedProc;
>>>>>>> graemeg/cpstrnew
      end;
    tkself: begin
      //x:=TPrimitiveExpr.Create(AParent,pekString, CurTokenText); //function(self);
      x:=TSelfExpr.Create(AParent);
      NextToken;
      if CurToken = tkDot then
        begin // self.Write(EscapeText(AText));
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then
          begin
          B.Free;
          Exit; // error
          end;
         x:=b;
        end;
      UngetToken;
    end;
    tkAt: begin
      // P:=@function;
      NextToken;
      if (length(CurTokenText)=0) or not (CurTokenText[1] in ['A'..'_']) then begin
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '@'+CurTokenText);
    end;
    tkCaret: begin
      // ^A..^_ characters. See #16341
      NextToken;
<<<<<<< HEAD
      if not (length(CurTokenText)=1) or not (CurTokenText[1] in ['A'..'_']) then begin
=======
      if CurToken = tkEqual then
      begin
        Value := ParseExpression(Parent);
      end else
>>>>>>> graemeg/cpstrnew
        UngetToken;
        ParseExc(SParserExpectedIdentifier);
      end;
      x:=TPrimitiveExpr.Create(AParent,pekString, '^'+CurTokenText);
    end;
  else
    ParseExc(SParserExpectedIdentifier);
  end;

<<<<<<< HEAD
  if x.Kind<>pekSet then NextToken;
=======
  // Not overloaded, so just add the proc/function to the lists
  Declarations.Declarations.Add(AProc);
  Declarations.Functions.Add(AProc);
end;
>>>>>>> graemeg/cpstrnew

  try
    if x.Kind=pekIdent then begin
      while CurToken in [tkBraceOpen, tkSquaredBraceOpen, tkCaret] do
        case CurToken of
          tkBraceOpen: begin
            prm:=ParseParams(AParent,pekFuncParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkSquaredBraceOpen: begin
            prm:=ParseParams(AParent,pekArrayParams);
            if not Assigned(prm) then Exit;
            prm.Value:=x;
            x:=prm;
          end;
          tkCaret: begin
            u:=TUnaryExpr.Create(AParent,x, TokenToExprOp(CurToken));
            x:=u;
            NextToken;
          end;
        end;

      if CurToken in [tkDot, tkas] then begin
        optk:=CurToken;
        NextToken;
        b:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(optk));
        if not Assigned(b.right) then
          begin
          b.free;
          Exit; // error
          end;
        x:=b;
      end;
    end;

    Result:=x;
  finally
    if not Assigned(Result) then x.Free;
  end;
end;

<<<<<<< HEAD
function TPasParser.OpLevel(t: TToken): Integer;
begin
<<<<<<< HEAD
  case t of
  //  tkDot:
  //    Result:=5;
    tknot,tkAt:
      Result:=4;
    tkMul, tkDivision, tkdiv, tkmod, tkand, tkShl,tkShr, tkas, tkPower :
      Result:=3;
    tkPlus, tkMinus, tkor, tkxor:
      Result:=2;
    tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan, tkGreaterThan, tkGreaterEqualThan, tkin, tkis:
      Result:=1;
  else
    Result:=0;
=======
// Next token is expected to be a "(", ";" or for a function ":". The caller
// will get the token after the final ";" as next token.
procedure TPasParser.ParseProcedureOrFunctionHeader(Parent: TPasElement;
  Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);

procedure ConsumeSemi;
begin
  NextToken;
  if (CurToken <> tksemicolon) and IsCurTokenHint then
    ungettoken;
end;

Var
  Tok : String;
  i: Integer;
  Proc: TPasProcedure;
  ahint : TPasMemberHint;
begin
  NextToken;
  case ProcType of
    ptFunction,ptClassFunction:
      begin
        if CurToken = tkBraceOpen then
        begin
          NextToken;
          if (CurToken <> tkBraceClose) then
            begin
              UngetToken;
              ParseArgList(Parent, Element.Args, tkBraceClose);
            end;
          ExpectToken(tkColon);
        end else if CurToken <> tkColon then
          ParseExc(SParserExpectedLBracketColon);
        if Assigned(Element) then        // !!!
          TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
        else
          ParseType(nil);
      end;

    ptProcedure,ptConstructor,ptDestructor,ptClassProcedure:
      begin
        if CurToken = tkBraceOpen then
        begin
          NextToken;
          if (CurToken = tkBraceClose) then
          else
            begin
              UngetToken;
              ParseArgList(Element, Element.Args, tkBraceClose);
            end
        end else if (CurToken = tkSemicolon)
          or (OfObjectPossible and (CurToken in [tkOf,tkEqual]))
        then
          UngetToken
        else
          ParseExc(SParserExpectedLBracketSemicolon);
      end;
    ptOperator:
      begin
        ParseArgList(Element, Element.Args, tkBraceClose);
        NextToken;
        if (CurToken=tkIdentifier) then
        begin
          TPasFunctionType(Element).ResultEl.Name := CurTokenName;
          ExpectToken(tkColon);
        end
        else if (CurToken=tkColon) then
          TPasFunctionType(Element).ResultEl.Name := 'Result'
        else
          ParseExc(SParserExpectedColonID);
        if Assigned(Element) then        // !!!
          TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
        else
          ParseType(nil);
      end;
>>>>>>> graemeg/cpstrnew
  end;
end;

function TPasParser.DoParseExpression(Aparent : TPaselement;InitExpr: TPasExpr): TPasExpr;
var
  expstack  : TFPList;
  opstack   : TFPList;
  pcount    : Integer;
  x         : TPasExpr;
  i         : Integer;
  tempop    : TToken;
  NotBinary : Boolean;
  
const
  PrefixSym = [tkPlus, tkMinus, tknot, tkAt]; // + - not @
  BinaryOP  = [tkMul, tkDivision, tkdiv, tkmod,  tkDotDot,
               tkand, tkShl,tkShr, tkas, tkPower,
               tkPlus, tkMinus, tkor, tkxor, tkSymmetricalDifference,
               tkEqual, tkNotEqual, tkLessThan, tkLessEqualThan,
               tkGreaterThan, tkGreaterEqualThan, tkin, tkis];

  function PopExp: TPasExpr; inline;
  begin
    if expstack.Count>0 then begin
      Result:=TPasExpr(expstack[expstack.Count-1]);
      expstack.Delete(expstack.Count-1);
    end else
      Result:=nil;
  end;

  procedure PushOper(token: TToken); inline;
  begin
    opstack.Add( Pointer(PtrInt(token)) );
  end;

<<<<<<< HEAD
  function PeekOper: TToken; inline;
  begin
    if opstack.Count>0 then Result:=TToken(PtrUInt(opstack[ opstack.Count-1]))
    else Result:=tkEOF
  end;

  function PopOper: TToken; inline;
  begin
    Result:=PeekOper;
    if Result<>tkEOF then opstack.Delete(opstack.Count-1);
  end;

  procedure PopAndPushOperator;
  var
    t       : TToken;
    xright  : TPasExpr;
    xleft   : TPasExpr;
    bin     : TBinaryExpr;
=======
  Module:=nil;
  NextToken;
  case CurToken of
    tkUnit: ParseUnit(Module);
    tkProgram: ParseProgram(Module);
    else
      ParseExc(Format(SParserExpectTokenError, ['unit']));
  end;
end;

// Starts after the "unit" token
procedure TPasParser.ParseUnit(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasModule, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
=======
  ExpectToken(tkSemicolon);
  while True do
>>>>>>> graemeg/cpstrnew
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    CheckHint(Module,True);
//    ExpectToken(tkSemicolon);
    ExpectToken(tkInterface);
    ParseInterface;
  finally
    CurModule:=nil;
  end;
end;

// Starts after the "program" token
procedure TPasParser.ParseProgram(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasProgram, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    NextToken;
<<<<<<< HEAD
    ParseImplementation;
  finally
    CurModule:=nil;
  end;
end;

// Starts after the "interface" token
procedure TPasParser.ParseInterface;
var
  Section: TInterfaceSection;
begin
  Section := TInterfaceSection(CreateElement(TInterfaceSection, '', CurModule));
  CurModule.InterfaceSection := Section;
  ParseDeclarations(Section);
end;

// Starts after the "implementation" token
procedure TPasParser.ParseImplementation;
var
  Section: TImplementationSection;
begin
  Section := TImplementationSection(CreateElement(TImplementationSection, '', CurModule));
  CurModule.ImplementationSection := Section;
  ParseDeclarations(Section);
end;

procedure TPasParser.ParseInitialization;
var
  Section: TInitializationSection;
  SubBlock: TPasImplElement;
begin
  Section := TInitializationSection(CreateElement(TInitializationSection, '', CurModule));
  CurModule.InitializationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      exit;
    end
    else if (CurToken=tkfinalization) then
    begin
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
  UngetToken;
end;

procedure TPasParser.ParseDeclarations(Declarations: TPasDeclarations);
var
  CurBlock: TDeclType;
  ConstEl: TPasConst;
  ResStrEl: TPasResString;
  TypeEl: TPasType;
  ClassEl: TPasClassType;
  List: TList;
  i,j: Integer;
  VarEl: TPasVariable;
  PropEl : TPasProperty;
begin
  CurBlock := declNone;
  while True do
>>>>>>> graemeg/cpstrnew
  begin
    t:=PopOper;
    xright:=PopExp;
    xleft:=PopExp;
    if t=tkDotDot then
      bin := TBinaryExpr.CreateRange(AParent,xleft, xright)
    else
      bin := TBinaryExpr.Create(AParent,xleft, xright, TokenToExprOp(t));
    expstack.Add(bin);
  end;

begin
  //DumpCurToken('Entry',iaIndent);
  Result:=nil;
  expstack := TFPList.Create;
  opstack := TFPList.Create;
  try
    repeat
      NotBinary:=True;
      pcount:=0;
      if not Assigned(InitExpr) then
        begin
        // the first part of the expression has been parsed externally.
        // this is used by Constant Expresion parser (CEP) parsing only,
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

        while CurToken in PrefixSym do
          begin
          PushOper(CurToken);
          inc(pcount);
          NextToken;
          end;

        if (CurToken = tkBraceOpen) then
          begin
          NextToken;
          x:=DoParseExpression(AParent);
          if CurToken<>tkBraceClose then
            begin
            x.free;
            Exit;
            end;
          NextToken;
          //     DumpCurToken('Here 1');
               // for the expression like  (TObject(m)).Free;
               if (x<>Nil) and (CurToken=tkDot) then
                 begin
                 NextToken;
          //       DumpCurToken('Here 2');
                 x:=TBinaryExpr.Create(AParent,x, ParseExpIdent(AParent), TokenToExprOp(tkDot));
          //       DumpCurToken('Here 3');
                 end;

          end
        else
          begin
          x:=ParseExpIdent(AParent);
          end;
        if not Assigned(x) then
          Exit;
        expstack.Add(x);

        for i:=1 to pcount do
          begin
          tempop:=PopOper;
          x:=popexp;
          if (tempop=tkMinus) and (X.Kind=pekRange) then
            begin
            TBinaryExpr(x).Left:=TUnaryExpr.Create(x, TBinaryExpr(X).left, eopSubtract);
            expstack.Add(x);
            end
          else
            expstack.Add( TUnaryExpr.Create(AParent, x, TokenToExprOp(tempop) ));
          end;
        end
      else
        begin
        expstack.Add(InitExpr);
        InitExpr:=nil;
        end;
      if (CurToken in BinaryOP) then
        begin
        // Adjusting order of the operations
        NotBinary:=False;
        tempop:=PeekOper;
        while (opstack.Count>0) and (OpLevel(tempop)>=OpLevel(CurToken)) do begin
          PopAndPushOperator;
          tempop:=PeekOper;
        end;
        PushOper(CurToken);
        NextToken;
        end;
     // Writeln('Bin ',NotBinary ,' or EOE ',isEndOfExp, ' Ex ',Assigned(x),' stack ',ExpStack.Count);
    until NotBinary or isEndOfExp;

    if not NotBinary then ParseExc(SParserExpectedIdentifier);

    while opstack.Count>0 do PopAndPushOperator;

    // only 1 expression should be on the stack, at the end of the correct expression
    if expstack.Count=1 then Result:=TPasExpr(expstack[0]);

  finally
    {if Not Assigned(Result) then
      DumpCurToken('Exiting (no result)',iaUndent)
    else
      DumpCurtoken('Exiting (Result: "'+Result.GetDeclaration(true)+'") ',iaUndent);}
    if not Assigned(Result) then begin
      // expression error!
      for i:=0 to expstack.Count-1 do
        TObject(expstack[i]).Free;
    end;
    opstack.Free;
    expstack.Free;
  end;
end;


function GetExprIdent(p: TPasExpr): String;
begin
  if Assigned(p) and (p is TPrimitiveExpr) and (p.Kind=pekIdent) then
    Result:=TPrimitiveExpr(p).Value
  else
    Result:='';
end;

function TPasParser.DoParseConstValueExpression(AParent: TPasElement): TPasExpr;
var
  x : TPasExpr;
  n : AnsiString;
  r : TRecordValues;
  a : TArrayValues;

function lastfield:boolean;

begin
  result:= CurToken<>tkSemicolon;
  if not result then
   begin
     nexttoken;
     if curtoken=tkbraceclose then
       result:=true
     else
       ungettoken;
   end; 
end;

begin
  if CurToken <> tkBraceOpen then
    Result:=DoParseExpression(AParent)
  else begin
    NextToken;
<<<<<<< HEAD
    x:=DoParseConstValueExpression(Aparent);
    case CurToken of
      tkComma: // array of values (a,b,c);
        begin
          a:=TArrayValues.Create(AParent);
          a.AddValues(x);
          repeat
            NextToken;
            x:=DoParseConstValueExpression(AParent);
            a.AddValues(x);
          until CurToken<>tkComma;
          Result:=a;
=======
    //writeln('TPasParser.ParseSection Token=',Scanner.CurTokenString,' ',CurToken);
    case CurToken of
      tkend:
        begin
        ExpectToken(tkDot);
        break;
        end;
      tkimplementation:
        if (CurToken = tkImplementation) and (Declarations is TInterfaceSection) then
          begin
          If Not Engine.InterfaceOnly then
            ParseImplementation;
          break;
          end;
      tkinitialization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end;
      tkfinalization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseFinalization;
          break;
          end;
      tkUses:
        if Declarations is TPasSection then
          ParseUsesList(TPasSection(Declarations))
        else
          ParseExc(SParserSyntaxError);
      tkConst:
        CurBlock := declConst;
      tkResourcestring:
        CurBlock := declResourcestring;
      tkType:
        CurBlock := declType;
      tkVar:
        CurBlock := declVar;
      tkThreadVar:
        CurBlock := declThreadVar;
      tkProperty:
        CurBlock := declProperty;
      tkProcedure:
        begin
          AddProcOrFunction(Declarations,
                       ParseProcedureOrFunctionDecl(Declarations, ptProcedure));
          CurBlock := declNone;
>>>>>>> graemeg/cpstrnew
        end;

      tkColon: // record field (a:xxx;b:yyy;c:zzz);
        begin
<<<<<<< HEAD
          n:=GetExprIdent(x);
          x.Free;
          r:=TRecordValues.Create(AParent);
          NextToken;
          x:=DoParseConstValueExpression(AParent);
          r.AddField(n, x);
          if not lastfield then
            repeat
              n:=ExpectIdentifier;
              ExpectToken(tkColon);
              NextToken;
              x:=DoParseConstValueExpression(AParent);
              r.AddField(n, x)
            until lastfield; // CurToken<>tkSemicolon;
          Result:=r;
=======
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptFunction));
          CurBlock := declNone;
        end;
      tkConstructor:
        begin
          AddProcOrFunction(Declarations,
                     ParseProcedureOrFunctionDecl(Declarations, ptConstructor));
          CurBlock := declNone;
        end;
      tkDestructor:
        begin
          AddProcOrFunction(Declarations,
                      ParseProcedureOrFunctionDecl(Declarations, ptDestructor));
          CurBlock := declNone;
>>>>>>> graemeg/cpstrnew
        end;
=======
    if (CurToken = tkIdentifier) or (CurToken=tklibrary) then // library is a token and a directive.
      begin
      Tok:=UpperCase(CurTokenString);
      If (Tok='CDECL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccCDecl;
        ExpectToken(tkSemicolon);
        end
      else If (Tok='EXPORT') then
        begin
        TPasProcedure(Parent).AddModifier(pmExported);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='PASCAL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccPascal;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='STDCALL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccStdCall;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='OLDFPCCALL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccOldFPCCall;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='EXTDECL') then
        begin
        // extdecl is a common macro for external functions
        TPasProcedure(Parent).AddModifier(pmExternal);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='EXTERNAL') then
        begin
        TPasProcedure(Parent).AddModifier(pmExternal);
        NextToken;
        if CurToken in [tkString,tkIdentifier] then
        begin
          NextToken;
          Tok:=UpperCase(CurTokenString);
          if Tok='NAME' then
          begin
            NextToken;
            if not (CurToken in [tkString,tkIdentifier]) then
              ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkString]]));
          end;
        end else
          UngetToken;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='REGISTER') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccRegister;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='COMPILERPROC') then
        begin
        TPasProcedure(Parent).AddModifier(pmCompilerProc);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='VARARGS') then
        begin
        TPasProcedure(Parent).AddModifier(pmVarArgs);
        ExpectToken(tkSemicolon);
        end
      else if IsCurTokenHint(ahint) then  // deprecated,platform,experimental,library, unimplemented etc
        begin
        element.hints:=element.hints+[ahint];
        consumesemi;
        end
      else if (tok='OVERLOAD') then
        begin
        TPasProcedure(Parent).AddModifier(pmOverload);
        ExpectToken(tkSemicolon);
        end
      else if (tok='INLINE') then
        begin
        TPasProcedure(Parent).AddModifier(pmInline);
        ExpectToken(tkSemicolon);
        end
      else if (tok='ASSEMBLER') then
        begin
        TPasProcedure(Parent).AddModifier(pmAssembler);
        ExpectToken(tkSemicolon);
        end
      else if (tok = 'EXTERNAL') then
        repeat
          NextToken;
        until CurToken = tkSemicolon
      else if (tok = 'PUBLIC') then
        begin
          NextToken;
          { Should be token Name,
            if not we're in a class and the public section starts }
          If (Uppercase(CurTokenString)<>'NAME') then
            begin
            UngetToken;
            UngetToken;
            Break;
            end
          else
            begin
            NextToken;  // Should be export name string.
            ExpectToken(tkSemicolon);
            end;
        end
      else if (tok = 'FORWARD') then
        begin
          if (Parent.Parent is TInterfaceSection) then
            begin
            UngetToken;
            break;
            end;
          if Parent is TPasProcedure then
            TPasProcedure(Parent).AddModifier(pmForward);
          ExpectToken(tkSemicolon);
        end
      else
        begin
        UnGetToken;
        Break;
        end
      end
    else if (CurToken = tkInline) then
      begin
      if Parent is TPasProcedure then
        TPasProcedure(Parent).AddModifier(pmInline);
      ExpectToken(tkSemicolon);
      end
    else if (CurToken = tkSquaredBraceOpen) then
      begin
      repeat
        NextToken
      until CurToken = tkSquaredBraceClose;
      ExpectToken(tkSemicolon);
      end
>>>>>>> graemeg/cpstrnew
    else
      // Binary expression!  ((128 div sizeof(longint)) - 3);       ;
      Result:=DoParseExpression(AParent,x);
      if CurToken<>tkBraceClose then ParseExc(SParserExpectedCommaRBracket);
      NextToken;
      if CurToken <> tkSemicolon then // the continue of expresion
        Result:=DoParseExpression(AParent,Result);
      Exit;
    end;
<<<<<<< HEAD
    if CurToken<>tkBraceClose then ParseExc(SParserExpectedCommaRBracket);
    NextToken;
  end;
=======
  if (ProcType = ptOperator) and (Parent is TPasProcedure) then
  begin
    Proc:=TPasProcedure(Parent);
    Proc.Name := Proc.Name + '(';
    for i := 0 to Proc.ProcType.Args.Count - 1 do
    begin
      if i > 0 then
        Proc.Name := Proc.Name + ', ';
      Proc.Name := Proc.Name +
        TPasArgument(Proc.ProcType.Args[i]).ArgType.Name;
    end;
    Proc.Name := Proc.Name + '): ' +
      TPasFunctionType(Proc.ProcType).ResultEl.ResultType.Name;
  end;

  if (Parent is TPasProcedure)
  and (not TPasProcedure(Parent).IsForward)
  and (not TPasProcedure(Parent).IsExternal)
  and ((Parent.Parent is TImplementationSection)
     or (Parent.Parent is TProcedureBody))
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
>>>>>>> graemeg/cpstrnew
end;

function TPasParser.CheckOverloadList(AList: TFPList; AName: String; out
  OldMember: TPasElement): TPasOverloadedProc;

Var
  I : Integer;

<<<<<<< HEAD
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
=======
var
  isArray : Boolean;

  procedure MaybeReadFullyQualifiedIdentifier(Var r : String);

  begin
    while True do
      begin
      NextToken;
      if CurToken = tkDot then
        begin
        ExpectIdentifier;
        R:=R + '.' + CurTokenString;
        end
>>>>>>> graemeg/cpstrnew
      else
        begin
<<<<<<< HEAD
        Result:=TPasOverloadedProc.Create(AName, OldMember.Parent);
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

<<<<<<< HEAD
procedure TPasParser.AddProcOrFunction(Decs: TPasDeclarations;
  AProc: TPasProcedure);
var
  I : Integer;
  OldMember: TPasElement;
  OverloadedProc: TPasOverloadedProc;
begin
  With Decs do
    begin
    OverloadedProc:=CheckOverloadList(Functions,AProc.Name,OldMember);
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
end;

// Return the parent of a function declaration. This is APArent,
// except when AParent is a class, and the function is overloaded.
// Then the parent is the overload object.
function TPasParser.CheckIfOverloaded(AParent: TPasElement; const AName: String): TPasElement;
var
  Member: TPasElement;
  OverloadedProc: TPasOverloadedProc;
=======
  function GetAccessorName: String;
  begin
    ExpectIdentifier;
    Result := CurTokenString;
    MaybeReadFullyQualifiedIdentifier(Result);
    if CurToken = tkSquaredBraceOpen then begin
      Result := Result + '[';
      NextToken;
      if CurToken in [tkIdentifier, tkNumber] then begin
    Result := Result + CurTokenString;
      end;
      ExpectToken(tkSquaredBraceClose);
      Result := Result + ']';
    end else
      UngetToken;
    //MaybeReadFullyQualifiedIdentifier(Result);
    //writeln(Result);
  end;
>>>>>>> graemeg/cpstrnew

var
  us  : String; 
  h   : TPasMemberHint;
begin
<<<<<<< HEAD
  Result:=AParent;
  If AParent is TPasClassType then
    begin
    OverloadedProc:=CheckOverLoadList(TPasClassType(AParent).Members,AName,Member);
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
  else
    ungettoken;
    ParseProgram(Module,True);
  //    ParseExc(Format(SParserExpectTokenError, ['unit']));
=======
  isArray:=False;
  NextToken;
// if array prop then parse [ arg1:type1;... ]

  if CurToken = tkSquaredBraceOpen then begin
    isArray:=True;
  // !!!: Parse array properties correctly
    ParseArgList(Element, TPasProperty(Element).Args, tkSquaredBraceClose);
    NextToken;
>>>>>>> graemeg/cpstrnew
  end;
end;

<<<<<<< HEAD
// Starts after the "unit" token
procedure TPasParser.ParseUnit(var Module: TPasModule);
var
  AUnitName: String;
begin
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
  Module := TPasModule(CreateElement(TPasModule, AUnitName,
    Engine.Package));
  FCurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    CheckHint(Module,True);
//    ExpectToken(tkSemicolon);
    ExpectToken(tkInterface);
    If LogEvent(pleInterface) then
      DoLog(SLogStartInterface );
    ParseInterface;
  finally
    FCurModule:=nil;
  end;
end;

// Starts after the "program" token
procedure TPasParser.ParseProgram(var Module: TPasModule; SkipHeader : Boolean = False);

Var
  PP : TPasProgram;
  Section : TProgramSection;
  N : String;

begin
  if SkipHeader then
    N:=ChangeFileExt(Scanner.CurFilename,'')
  else
    N:=ExpectIdentifier;
  Module := nil;
  PP:=TPasProgram(CreateElement(TPasProgram, N, Engine.Package));
  Module :=PP;
  FCurModule:=Module;
  try
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
          ParseExc(SParserExpectedCommaRBracket);
        If (CurToken=tkComma) then
          PP.OutPutFile:=ExpectIdentifier;
        ExpectToken(tkBraceClose);
        NextToken;
        end;
      if (CurToken<>tkSemicolon) then
        ParseExc(Format(SParserExpectTokenError,[';']));
      end;
    Section := TProgramSection(CreateElement(TProgramSection, '', CurModule));
    PP.ProgramSection := Section;
    ParseDeclarations(Section);
  finally
    FCurModule:=nil;
  end;
end;

procedure TPasParser.ParseLibrary(var Module: TPasModule);
Var
  PP : TPasLibrary;
  Section : TLibrarySection;

begin
  Module := nil;
  PP:=TPasLibrary(CreateElement(TPasLibrary, ExpectIdentifier, Engine.Package));
  Module :=PP;
  FCurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    NextToken;
    if (CurToken<>tkSemicolon) then
        ParseExc(Format(SParserExpectTokenError,[';']));
    Section := TLibrarySection(CreateElement(TLibrarySection, '', CurModule));
    PP.LibrarySection := Section;
    ParseDeclarations(Section);
  finally
    FCurModule:=nil;
  end;
end;

// Starts after the "interface" token
procedure TPasParser.ParseInterface;
var
  Section: TInterfaceSection;
begin
  Section := TInterfaceSection(CreateElement(TInterfaceSection, '', CurModule));
  CurModule.InterfaceSection := Section;
  ParseDeclarations(Section);
end;

// Starts after the "implementation" token
procedure TPasParser.ParseImplementation;
var
  Section: TImplementationSection;
begin
  Section := TImplementationSection(CreateElement(TImplementationSection, '', CurModule));
  CurModule.ImplementationSection := Section;
  ParseDeclarations(Section);
end;

procedure TPasParser.ParseInitialization;
var
  Section: TInitializationSection;
  SubBlock: TPasImplElement;
begin
  Section := TInitializationSection(CreateElement(TInitializationSection, '', CurModule));
  CurModule.InitializationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      exit;
    end
    else if (CurToken=tkfinalization) then
    begin
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
  UngetToken;
end;

function TPasParser.GetProcTypeFromToken(tk: TToken; IsClass: Boolean
  ): TProcType;

begin
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
    ParseExc(SParserNotAProcToken);
  end;
end;

procedure TPasParser.ParseDeclarations(Declarations: TPasDeclarations);
var
  CurBlock: TDeclType;
  ConstEl: TPasConst;
  ResStrEl: TPasResString;
  TypeEl: TPasType;
  ClassEl: TPasClassType;
  List: TFPList;
  i,j: Integer;
  VarEl: TPasVariable;
  ExpEl: TPasExportSymbol;
  PropEl : TPasProperty;
  TypeName: String;
  PT : TProcType;

begin
  CurBlock := declNone;
  while True do
  begin
    NextToken;
  //  writeln('TPasParser.ParseSection Token=',CurTokenString,' ',CurToken, ' ',scanner.CurFilename);
    case CurToken of
      tkend:
        begin
        If (CurModule is TPasProgram) and (CurModule.InitializationSection=Nil) then
          ParseExc(Format(SParserExpectTokenError,['begin']));
        ExpectToken(tkDot);
        break;
        end;
      tkimplementation:
        if (Declarations is TInterfaceSection) then
          begin
          If Not Engine.InterfaceOnly then
            begin
            If LogEvent(pleImplementation) then
              DoLog(SLogStartImplementation);
            ParseImplementation;
            end;
          break;
          end;
      tkinitialization:
        if (Declarations is TInterfaceSection)
        or ((Declarations is TImplementationSection) and not (Declarations is TProgramSection)) then
          begin
          ParseInitialization;
          break;
          end;
      tkfinalization:
        if (Declarations is TInterfaceSection)
        or ((Declarations is TImplementationSection) and not (Declarations is TProgramSection)) then
          begin
          ParseFinalization;
          break;
          end;
      tkUses:
        if Declarations is TPasSection then
          ParseUsesList(TPasSection(Declarations))
        else
          ParseExc(SParserSyntaxError);
      tkConst:
        CurBlock := declConst;
      tkexports:
        CurBlock := declExports;
      tkResourcestring:
        CurBlock := declResourcestring;
      tkType:
        CurBlock := declType;
      tkVar:
        CurBlock := declVar;
      tkThreadVar:
        CurBlock := declThreadVar;
      tkProperty:
        CurBlock := declProperty;
      tkProcedure, tkFunction, tkConstructor, tkDestructor,tkOperator:
        begin
        SaveComments;
        pt:=GetProcTypeFromToken(CurToken);
        AddProcOrFunction(Declarations, ParseProcedureOrFunctionDecl(Declarations, pt));
        CurBlock := declNone;
        end;
      tkClass:
        begin
          SaveComments;
          NextToken;
          If CurToken in [tkprocedure,tkFunction,tkConstructor, tkDestructor] then
            begin
            pt:=GetProcTypeFromToken(CurToken,True);
            AddProcOrFunction(Declarations,ParseProcedureOrFunctionDecl(Declarations, pt));
            CurBlock := declNone;
            end
          else
            ExpectToken(tkprocedure);
        end;
      tkIdentifier:
        begin
          SaveComments;
=======
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptOperator));
          CurBlock := declNone;
        end;
      tkClass:
        begin
          NextToken;
          case CurToken of
            tkprocedure:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassProcedure));
                CurBlock := declNone;
              end;
            tkfunction:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassFunction));
                CurBlock := declNone;
              end;
          else
            ExpectToken(tkprocedure);
          end;
        end;
      tkIdentifier:
        begin
>>>>>>> graemeg/cpstrnew
          case CurBlock of
            declConst:
              begin
                ConstEl := ParseConstDecl(Declarations);
                Declarations.Declarations.Add(ConstEl);
                Declarations.Consts.Add(ConstEl);
              end;
            declResourcestring:
              begin
                ResStrEl := ParseResourcestringDecl(Declarations);
                Declarations.Declarations.Add(ResStrEl);
                Declarations.ResStrings.Add(ResStrEl);
              end;
            declType:
              begin
                TypeEl := ParseTypeDecl(Declarations);
                if Assigned(TypeEl) then        // !!!
                begin
                  Declarations.Declarations.Add(TypeEl);
                  if TypeEl.ClassType = TPasClassType then
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
                        ClassEl.Release;
                        break;
                      end;
                    end;
                    // Add the new class to the class list
                    Declarations.Classes.Add(TypeEl)
                  end else
                    Declarations.Types.Add(TypeEl);
                end;
              end;
<<<<<<< HEAD
            declExports:
              begin
              List := TFPList.Create;
              try
                try
                  ParseExportDecl(Declarations, List);
                except
                  for i := 0 to List.Count - 1 do
                    TPasExportSymbol(List[i]).Release;
                  raise;
                end;
                for i := 0 to List.Count - 1 do
                begin
                  ExpEl := TPasExportSymbol(List[i]);
                  Declarations.Declarations.Add(ExpEl);
                  Declarations.ExportSymbols.Add(ExpEl);
                end;
              finally
                List.Free;
              end;
              end;
            declVar, declThreadVar:
              begin
                List := TFPList.Create;
=======
            declVar, declThreadVar:
              begin
                List := TList.Create;
>>>>>>> graemeg/cpstrnew
                try
                  try
                    ParseVarDecl(Declarations, List);
                  except
                    for i := 0 to List.Count - 1 do
                      TPasVariable(List[i]).Release;
                    raise;
                  end;
                  for i := 0 to List.Count - 1 do
                  begin
                    VarEl := TPasVariable(List[i]);
                    Declarations.Declarations.Add(VarEl);
                    Declarations.Variables.Add(VarEl);
                  end;
                finally
                  List.Free;
                end;
              end;
            declProperty:
              begin
<<<<<<< HEAD
              PropEl:=ParseProperty(Declarations,CurtokenString,visDefault);
=======
              PropEl:=TPasProperty(CreateElement(TPasProperty, CurTokenString, Declarations));
              Try
                ParseProperty(PropEl)
              except
                Propel.Free;
                Raise;
              end;
>>>>>>> graemeg/cpstrnew
              Declarations.Declarations.Add(PropEl);
              Declarations.properties.add(PropEl);
              end;
          else
            ParseExc(SParserSyntaxError);
          end;
<<<<<<< HEAD
        end;
      tkGeneric:
        begin
          if CurBlock <> declType then
            ParseExc(SParserSyntaxError);
          TypeName := ExpectIdentifier;
          ClassEl := TPasClassType(Engine.CreateElement(TPasClassType,TypeName,Declarations, Scanner.CurFilename, Scanner.CurRow));
          ClassEl.ObjKind:=okGeneric;
          try
            ReadGenericArguments(ClassEl.GenericTemplateTypes,ClassEl);
          Except
            List.Free;
            Raise;
          end;
          ExpectToken(tkEqual);
          ExpectToken(tkClass);
          NextToken;
          DoParseClassType(ClassEl);
          Declarations.Declarations.Add(ClassEl);
          Declarations.Classes.Add(ClassEl);
          CheckHint(classel,True);
        end;
      tkbegin:
        begin
        if Declarations is TProcedureBody then
          begin
          ParseProcBeginBlock(TProcedureBody(Declarations));
          break;
          end
        else if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end
        else
          ParseExc(SParserSyntaxError);
        end;
      tklabel:
        begin
          if not (Declarations is TInterfaceSection) then
            ParseLabels(Declarations);
        end;
    else
      ParseExc(SParserSyntaxError);
    end;
  end;
end;

// Starts after the "uses" token
procedure TPasParser.ParseUsesList(ASection: TPasSection);

  function CheckUnit(AUnitName : string):TPasElement;
  begin
    result := Engine.FindModule(AUnitName);  // should we resolve module here when "IN" filename is not known yet?
    if Assigned(result) then
      result.AddRef
    else
      Result := TPasType(CreateElement(TPasUnresolvedUnitRef, AUnitName,
        ASection));
    ASection.UsesList.Add(Result);
  end;

var
  AUnitName: String;
  Element: TPasElement;
begin
  If not (Asection.ClassType=TImplementationSection) Then // interface,program,library,package
    Element:=CheckUnit('System'); // system always implicitely first.    
  Repeat
    AUnitName := ExpectIdentifier; 
    NextToken;
    while CurToken = tkDot do
    begin
      ExpectIdentifier;
      AUnitName := AUnitName + '.' + CurTokenString;
      NextToken;
    end;
    Element := CheckUnit(AUnitName);
    if (CurToken=tkin) then
      begin
      ExpectToken(tkString);
      if (Element is TPasModule) and (TPasmodule(Element).filename='')  then
        TPasModule(Element).FileName:=curtokenstring
      else if (Element is TPasUnresolvedUnitRef) then
        TPasUnresolvedUnitRef(Element).FileName:=curtokenstring;
      NextToken;
      end;

    if Not (CurToken in [tkComma,tkSemicolon]) then
      ParseExc(SParserExpectedCommaSemicolon);
  Until (CurToken=tkSemicolon);
end;

// Starts after the variable name
function TPasParser.ParseConstDecl(Parent: TPasElement): TPasConst;
begin
  SaveComments;
  Result := TPasConst(CreateElement(TPasConst, CurTokenString, Parent));
  try
    NextToken;
    if CurToken = tkColon then
      Result.VarType := ParseType(nil)
    else
      UngetToken;
    ExpectToken(tkEqual);
    NextToken;
    Result.Expr:=DoParseConstValueExpression(Result);
    UngetToken;
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
  end;
end;

// Starts after the variable name
function TPasParser.ParseResourcestringDecl(Parent: TPasElement): TPasResString;
begin
  SaveComments;
  Result := TPasResString(CreateElement(TPasResString, CurTokenString, Parent));
  try
    ExpectToken(tkEqual);
    NextToken; // skip tkEqual
    Result.Expr:=DoParseConstValueExpression(Result);
    UngetToken;
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
  end;
end;

procedure TPasParser.ReadGenericArguments(List : TFPList;Parent : TPasElement);

Var
  N : String;

begin
  ExpectToken(tkLessThan);
  repeat
    N:=ExpectIdentifier;
    List.Add(CreateElement(TPasGenericTemplateType,N,Parent));
    NextToken;
    if not (CurToken in [tkComma, tkGreaterThan]) then
      ParseExc(Format(SParserExpectToken2Error,
        [TokenInfos[tkComma], TokenInfos[tkGreaterThan]]));
  until CurToken = tkGreaterThan;
end;

// Starts after the type name
function TPasParser.ParseRangeType(AParent: TPasElement;
  const TypeName: String; Full: Boolean): TPasRangeType;

Var
  PE : TPasExpr;

begin
  Result := TPasRangeType(CreateElement(TPasRangeType, TypeName, AParent));
  try
    if Full then
      begin
      If not (CurToken=tkEqual) then
        ParseExc(Format(SParserExpectTokenError,[TokenInfos[tkEqual]]));
      end;
    NextToken;
    PE:=DoParseExpression(Result,Nil);
    if not ((PE is TBinaryExpr) and (TBinaryExpr(PE).Kind=pekRange)) then
      begin
      FreeAndNil(PE);
      ParseExc(SRangeExpressionExpected);
      end;
    Result.RangeExpr:=PE as TBinaryExpr;
    UngetToken;
  except
    FreeAndNil(Result);
    raise;
  end;
end;

// Starts after Exports, on first identifier.
procedure TPasParser.ParseExportDecl(Parent: TPasElement; List: TFPList);
Var
  E : TPasExportSymbol;
begin
  Repeat
    if List.Count<>0 then
      ExpectIdentifier;
    E:=TPasExportSymbol(CreateElement(TPasExportSymbol,CurtokenString,Parent));
    List.Add(E);
    NextToken;
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
      ParseExc(SParserExpectedCommaSemicolon);
  until (CurToken=tkSemicolon);
end;

function TPasParser.ParseSpecializeType(Parent: TPasElement;
  const TypeName: String): TPasClassType;

begin
  Result := TPasClassType(Engine.CreateElement(TPasClassType, TypeName, Parent, Scanner.CurFilename, Scanner.CurRow));
  try
    Result.ObjKind := okSpecialize;
    Result.AncestorType := ParseType(nil);
    Result.IsShortDefinition:=True;
    ReadGenericArguments(TPasClassType(Result).GenericTemplateTypes,Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TPasParser.ParseProcedureType(Parent: TPasElement;
  const TypeName: String; const PT: TProcType): TPasProcedureType;

begin
  if PT in [ptFunction,ptClassFunction] then
    Result := CreateFunctionType(TypeName, 'Result', Parent, False)
  else
    Result := TPasProcedureType(CreateElement(TPasProcedureType, TypeName, Parent));
  try
    ParseProcedureOrFunctionHeader(Result, TPasProcedureType(Result), PT, True);
  except
    FreeAndNil(Result);
    raise;
  end;
end;

function TPasParser.ParseTypeDecl(Parent: TPasElement): TPasType;

var
  TypeName: String;
begin
  TypeName := CurTokenString;
  ExpectToken(tkEqual);
  Result:=ParseType(Parent,TypeName,True);
end;

function TPasParser.GetVariableValueAndLocation(Parent: TPasElement; out
  Value: TPasExpr; out Location: String): Boolean;

begin
  Value:=Nil;
  NextToken;
  Result:=CurToken=tkEqual;
  if Result then
    begin
    NextToken;
    Value := DoParseConstValueExpression(Parent);
//    NextToken;
    end;
  if (CurToken=tkAbsolute) then
    begin
    Result:=True;
    ExpectIdentifier;
    Location:=CurTokenText;
    NextToken;
    if CurToken=tkDot then
      begin
      ExpectIdentifier;
      Location:=Location+'.'+CurTokenText;
      end
    else
      UnGetToken;
    end
  else
    UngetToken;
end;

function TPasParser.GetVariableModifiers(out VarMods: TVariableModifiers; out
  Libname, ExportName: string): string;

Var
  S : String;
begin
  Result := '';
  VarMods := [];
  NextToken;
  If CurTokenIsIdentifier('cvar') then
    begin
    Result:=';cvar';
    Include(VarMods,vmcvar);
    ExpectToken(tkSemicolon);
    NextToken;
    end;
  s:=LowerCase(CurTokenText);
  if Not ((s='external') or (s='public') or (s='export')) then
    UngetToken
  else
    begin
    if s='external' then
      Include(VarMods,vmexternal)
    else if (s='public') then
      Include(varMods,vmpublic)
    else if (s='export') then
      Include(varMods,vmexport);
    Result:=Result+';'+CurTokenText;
    NextToken;
    if (Curtoken<>tksemicolon) then
      begin
      if (s='external') then
        begin
        Include(VarMods,vmexternal);
        if (CurToken in [tkString,tkIdentifier])
            and Not (CurTokenIsIdentifier('name')) then
          begin
          Result := Result + ' ' + CurTokenText;
          LibName:=CurTokenText;
          NextToken;
          end;
        end;
      if CurTokenIsIdentifier('name') then
        begin
        Result := Result + ' name ';
        NextToken;
        if (CurToken in [tkString,tkIdentifier]) then
          Result := Result + CurTokenText
        else
          ParseExc(SParserSyntaxError);
        ExportName:=CurTokenText;
        NextToken;
        end
      else
        ParseExc(SParserSyntaxError);
      end;
    end;
end;


// Full means that a full variable declaration is being parsed.
procedure TPasParser.ParseVarList(Parent: TPasElement; VarList: TFPList; AVisibility: TPasMemberVisibility; Full : Boolean);

var
  VarNames: TStringList;
  i: Integer;
  Value : TPasExpr;
  VarType: TPasType;
  VarEl: TPasVariable;
  H : TPasMemberHints;
  varmods: TVariableModifiers;
  D,Mods,Loc,alibname,aexpname : string;

begin
  VarNames := TStringList.Create;
  try
    D:=SaveComments; // This means we support only one comment per 'list'.
    Repeat
      VarNames.Add(CurTokenString);
      NextToken;
      if Not (CurToken in [tkComma,tkColon]) then
        ParseExc(SParserExpectedCommaColon);
      if CurToken=tkComma then
        ExpectIdentifier;
    Until (CurToken=tkColon);
    If Full then
      VarType := ParseComplexType(Nil)
    else
      VarType := ParseComplexType(Parent);
    Value:=Nil;
    H:=CheckHint(Nil,False);
    If Full then
      GetVariableValueAndLocation(Parent,Value,Loc);
    H:=H+CheckHint(Nil,Full);
    if full then
      Mods:=GetVariableModifiers(varmods,alibname,aexpname)
    else
      NextToken;
    SaveComments(D);
    for i := 0 to VarNames.Count - 1 do
      begin
      // Writeln(VarNames[i], AVisibility);
      VarEl:=TPasVariable(CreateElement(TPasVariable,VarNames[i],Parent,AVisibility));
      VarEl.VarType := VarType;
      // Procedure declaration eats the hints.
      if Assigned(VarType) and (VarType is TPasprocedureType) then
        VarEl.Hints:=VarType.Hints
      else
        VarEl.Hints:=H;
      Varel.Modifiers:=Mods;
      Varel.VarModifiers:=VarMods;
      if (i=0) then
        VarEl.Expr:=Value;
      VarEl.AbsoluteLocation:=Loc;
      VarEl.LibraryName:=alibName;
      VarEl.ExportName:=aexpname;
      if (i>0) then
        VarType.AddRef;
      VarList.Add(VarEl);
      end;
  finally
    VarNames.Free;
  end;
end;

procedure TPasParser.SetOptions(AValue: TPOptions);
begin
  if FOptions=AValue then Exit;
  FOptions:=AValue;
  If Assigned(FScanner) then
    FScanner.Options:=AValue;
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

procedure TPasParser.DoLog(const Msg: String; SkipSourceInfo: Boolean);
begin
  If Assigned(FOnLog) then
    if SkipSourceInfo or not assigned(scanner) then
      FOnLog(Self,Msg)
    else
      FOnLog(Self,Format('%s(%d) : %s',[Scanner.CurFilename,SCanner.CurRow,Msg]));
end;

procedure TPasParser.DoLog(const Fmt: String; Args: array of const;
  SkipSourceInfo: Boolean);
begin
  DoLog(Format(Fmt,Args),SkipSourceInfo);
end;

procedure TPasParser.ParseInlineVarDecl(Parent: TPasElement; List: TFPList;
  AVisibility: TPasMemberVisibility = VisDefault; ClosingBrace: Boolean = False);

Var
  tt : TTokens;
begin
  ParseVarList(Parent,List,AVisibility,False);
  tt:=[tkEnd,tkSemicolon];
  if ClosingBrace then
   include(tt,tkBraceClose);
  if not (CurToken in tt) then
    ParseExc(SParserExpectedSemiColonEnd);
end;

// Starts after the variable name
procedure TPasParser.ParseVarDecl(Parent: TPasElement; List: TFPList);

begin
  ParseVarList(Parent,list,visDefault,True);
end;

// Starts after the opening bracket token
procedure TPasParser.ParseArgList(Parent: TPasElement; Args: TFPList; EndToken: TToken);
var
  ArgNames: TStringList;
  IsUntyped: Boolean;
  Name : String;
  Value : TPasExpr;
  i: Integer;
  Arg: TPasArgument;
  Access: TArgumentAccess;
  ArgType: TPasType;
begin
  ArgNames := TStringList.Create;
  try
    while True do
    begin
      ArgNames.Clear;
      Access := argDefault;
      IsUntyped := False;
      ArgType := nil;
      while True do
      begin
        NextToken;
        if CurToken = tkConst then
        begin
          Access := argConst;
          Name := ExpectIdentifier;
        end else if CurToken = tkConstRef then
        begin
          Access := argConstref;
          Name := ExpectIdentifier;
        end else if CurToken = tkVar then
        begin
          Access := ArgVar;
          Name := ExpectIdentifier;
        end else if (CurToken = tkIdentifier) and (UpperCase(CurTokenString) = 'OUT') then
        begin
          Access := ArgOut;
          Name := ExpectIdentifier;
        end else if CurToken = tkIdentifier then
          Name := CurTokenString
        else
          ParseExc(SParserExpectedConstVarID);
        ArgNames.Add(Name);
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
          ParseExc(SParserExpectedCommaColon);
      end;
      Value:=Nil;
      if not IsUntyped then
        begin
        ArgType := ParseType(nil);
        try
          NextToken;
          if CurToken = tkEqual then
            begin
            if (ArgNames.Count>1) then
              begin
              FreeAndNil(ArgType);
              ParseExc(SParserOnlyOneArgumentCanHaveDefault);
              end;
            NextToken;
            Value := DoParseExpression(Parent,Nil);
            // After this, we're on ), which must be unget.
            end;
          UngetToken;
        except
          FreeAndNil(ArgType);
          Raise;
        end;
        end;

      for i := 0 to ArgNames.Count - 1 do
      begin
        Arg := TPasArgument(CreateElement(TPasArgument, ArgNames[i], Parent));
        Arg.Access := Access;
        Arg.ArgType := ArgType;
        if (i > 0) and Assigned(ArgType) then
          ArgType.AddRef;
        Arg.ValueExpr := Value;
        Value:=Nil; // Only the first gets a value. OK, since Var A,B : Integer = 1 is not allowed.
        Args.Add(Arg);
      end;

      NextToken;
      if (CurToken = tkIdentifier) and (LowerCase(CurTokenString) = 'location') then
        begin
          NextToken; // remove 'location'
          NextToken; // remove register
        end;
      if CurToken = EndToken then
        break;
    end;
  finally
    ArgNames.Free;
  end;
end;


function TPasParser.CheckProcedureArgs(Parent: TPasElement; Args: TFPList;
  Mandatory: Boolean): boolean;

begin
  Module:=nil;
  NextToken;
<<<<<<< HEAD
  Result:=(Curtoken=tkbraceOpen);
  if not Result then
    begin
    if Mandatory then
      ParseExc(SParserExpectedLBracketColon)
=======
  case CurToken of
    tkUnit: ParseUnit(Module);
    tkProgram: ParseProgram(Module);
>>>>>>> graemeg/cpstrnew
    else
      UngetToken;
    end
  else
    begin
    NextToken;
    if (CurToken<>tkBraceClose) then
      begin
      UngetToken;
      ParseArgList(Parent, Args, tkBraceClose);
      end;
    end;
end;

<<<<<<< HEAD
procedure TPasParser.HandleProcedureModifier(Parent: TPasElement;pm : TProcedureModifier);

Var
  Tok : String;
  P : TPasProcedure;
  E : TPasExpr;

begin
  if parent is TPasProcedure then
    P:=TPasProcedure(Parent);
  if Assigned(P) then
    P.AddModifier(pm);
  if (pm=pmExternal) then
    begin
    NextToken;
    if CurToken in [tkString,tkIdentifier] then
      begin
      // extrenal libname
      // external libname name XYZ
      // external name XYZ
      Tok:=UpperCase(CurTokenString);
      if Not ((curtoken=tkIdentifier) and (Tok='NAME')) then
        begin
        E:=DoParseExpression(Parent);
        if Assigned(P) then
          P.LibraryExpr:=E;
=======
// Starts after the "unit" token
procedure TPasParser.ParseUnit(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasModule, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    CheckHint(Module,True);
//    ExpectToken(tkSemicolon);
    ExpectToken(tkInterface);
    ParseInterface;
  finally
    CurModule:=nil;
  end;
end;

// Starts after the "program" token
procedure TPasParser.ParseProgram(var Module: TPasModule);
begin
  Module := nil;
  Module := TPasModule(CreateElement(TPasProgram, ExpectIdentifier,
    Engine.Package));
  CurModule:=Module;
  try
    if Assigned(Engine.Package) then
    begin
      Module.PackageName := Engine.Package.Name;
      Engine.Package.Modules.Add(Module);
    end;
    NextToken;
    ParseImplementation;
  finally
    CurModule:=nil;
  end;
end;

// Starts after the "interface" token
procedure TPasParser.ParseInterface;
var
  Section: TInterfaceSection;
begin
  Section := TInterfaceSection(CreateElement(TInterfaceSection, '', CurModule));
  CurModule.InterfaceSection := Section;
  ParseDeclarations(Section);
end;

// Starts after the "implementation" token
procedure TPasParser.ParseImplementation;
var
  Section: TImplementationSection;
begin
  Section := TImplementationSection(CreateElement(TImplementationSection, '', CurModule));
  CurModule.ImplementationSection := Section;
  ParseDeclarations(Section);
end;

procedure TPasParser.ParseInitialization;
var
  Section: TInitializationSection;
  SubBlock: TPasImplElement;
begin
  Section := TInitializationSection(CreateElement(TInitializationSection, '', CurModule));
  CurModule.InitializationSection := Section;
  repeat
    NextToken;
    if (CurToken=tkend) then
    begin
      ExpectToken(tkDot);
      exit;
    end
    else if (CurToken=tkfinalization) then
    begin
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
  UngetToken;
end;

procedure TPasParser.ParseDeclarations(Declarations: TPasDeclarations);
var
  CurBlock: TDeclType;
  ConstEl: TPasConst;
  ResStrEl: TPasResString;
  TypeEl: TPasType;
  ClassEl: TPasClassType;
  List: TList;
  i,j: Integer;
  VarEl: TPasVariable;
  PropEl : TPasProperty;
begin
  CurBlock := declNone;
  while True do
  begin
    NextToken;
    //writeln('TPasParser.ParseSection Token=',Scanner.CurTokenString,' ',CurToken);
    case CurToken of
      tkend:
        begin
        ExpectToken(tkDot);
        break;
        end;
      tkimplementation:
        if (CurToken = tkImplementation) and (Declarations is TInterfaceSection) then
          begin
          If Not Engine.InterfaceOnly then
            ParseImplementation;
          break;
          end;
      tkinitialization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end;
      tkfinalization:
        if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseFinalization;
          break;
          end;
      tkUses:
        if Declarations is TPasSection then
          ParseUsesList(TPasSection(Declarations))
        else
          ParseExc(SParserSyntaxError);
      tkConst:
        CurBlock := declConst;
      tkResourcestring:
        CurBlock := declResourcestring;
      tkType:
        CurBlock := declType;
      tkVar:
        CurBlock := declVar;
      tkThreadVar:
        CurBlock := declThreadVar;
      tkProperty:
        CurBlock := declProperty;
      tkProcedure:
        begin
          AddProcOrFunction(Declarations,
                       ParseProcedureOrFunctionDecl(Declarations, ptProcedure));
          CurBlock := declNone;
>>>>>>> graemeg/cpstrnew
        end;
      if CurToken=tkSemicolon then
        UnGetToken
      else
        begin
<<<<<<< HEAD
        Tok:=UpperCase(CurTokenString);
        if ((curtoken=tkIdentifier) and (Tok='NAME')) then
          begin
          NextToken;
          if not (CurToken in [tkString,tkIdentifier]) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkString]]));
          E:=DoParseExpression(Parent);
          if Assigned(P) then
            P.LibrarySymbolName:=E;
          end;
        end;
      end
    else
      UngetToken;
    end
  else if (pm = pmPublic) then
    begin
    NextToken;
    { Should be token Name,
      if not we're in a class and the public section starts }
    If (Uppercase(CurTokenString)<>'NAME') then
      begin
      UngetToken;
      UngetToken;
      exit;
      end
    else
      begin
      NextToken;  // Should be export name string.
      if not (CurToken in [tkString,tkIdentifier]) then
        ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkString]]));
      E:=DoParseExpression(Parent);
      if parent is TPasProcedure then
        TPasProcedure(Parent).PublicName:=E;
      if (CurToken <> tkSemicolon) then
        ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
      end;
    end
  else if (pm=pmForward) then
    begin
    if (Parent.Parent is TInterfaceSection) then
       begin
       ParseExc(SParserForwardNotInterface);
       UngetToken;
       end;
    end
  else if (pm=pmMessage) then
    begin
    Repeat
      NextToken;
      If CurToken<>tkSemicolon then
        begin
        if parent is TPasProcedure then
          TPasProcedure(Parent).MessageName:=CurtokenString;
        If (CurToken=tkString) and (parent is TPasProcedure) then
          TPasProcedure(Parent).Messagetype:=pmtString;
        end;
    until CurToken = tkSemicolon;
    UngetToken;
=======
=======
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptFunction));
          CurBlock := declNone;
        end;
      tkConstructor:
        begin
          AddProcOrFunction(Declarations,
                     ParseProcedureOrFunctionDecl(Declarations, ptConstructor));
          CurBlock := declNone;
        end;
      tkDestructor:
        begin
          AddProcOrFunction(Declarations,
                      ParseProcedureOrFunctionDecl(Declarations, ptDestructor));
          CurBlock := declNone;
>>>>>>> graemeg/cpstrnew
        end;
      tkbegin:
        begin
<<<<<<< HEAD
        if Declarations is TProcedureBody then
          begin
          ParseProcBeginBlock(TProcedureBody(Declarations));
          break;
          end
        else if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end
        else
          ParseExc(SParserSyntaxError);
        end;
      tklabel:
        begin
          if not (Declarations is TInterfaceSection) then
            ParseLabels(Declarations);
=======
          AddProcOrFunction(Declarations,
                        ParseProcedureOrFunctionDecl(Declarations, ptOperator));
          CurBlock := declNone;
        end;
      tkClass:
        begin
          NextToken;
          case CurToken of
            tkprocedure:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassProcedure));
                CurBlock := declNone;
              end;
            tkfunction:
              begin
                AddProcOrFunction(Declarations,
                  ParseProcedureOrFunctionDecl(Declarations, ptClassFunction));
                CurBlock := declNone;
              end;
          else
            ExpectToken(tkprocedure);
          end;
        end;
      tkIdentifier:
        begin
          case CurBlock of
            declConst:
              begin
                ConstEl := ParseConstDecl(Declarations);
                Declarations.Declarations.Add(ConstEl);
                Declarations.Consts.Add(ConstEl);
              end;
            declResourcestring:
              begin
                ResStrEl := ParseResourcestringDecl(Declarations);
                Declarations.Declarations.Add(ResStrEl);
                Declarations.ResStrings.Add(ResStrEl);
              end;
            declType:
              begin
                TypeEl := ParseTypeDecl(Declarations);
                if Assigned(TypeEl) then        // !!!
                begin
                  Declarations.Declarations.Add(TypeEl);
                  if TypeEl.ClassType = TPasClassType then
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
                        ClassEl.Release;
                        break;
                      end;
                    end;
                    // Add the new class to the class list
                    Declarations.Classes.Add(TypeEl)
                  end else
                    Declarations.Types.Add(TypeEl);
                end;
              end;
            declVar, declThreadVar:
              begin
                List := TList.Create;
                try
                  try
                    ParseVarDecl(Declarations, List);
                  except
                    for i := 0 to List.Count - 1 do
                      TPasVariable(List[i]).Release;
                    raise;
                  end;
                  for i := 0 to List.Count - 1 do
                  begin
                    VarEl := TPasVariable(List[i]);
                    Declarations.Declarations.Add(VarEl);
                    Declarations.Variables.Add(VarEl);
                  end;
                finally
                  List.Free;
                end;
              end;
            declProperty:
              begin
              PropEl:=TPasProperty(CreateElement(TPasProperty, CurTokenString, Declarations));
              Try
                ParseProperty(PropEl)
              except
                Propel.Free;
                Raise;
              end;
              Declarations.Declarations.Add(PropEl);
              Declarations.properties.add(PropEl);
              end;
          else
            ParseExc(SParserSyntaxError);
          end;
>>>>>>> graemeg/cpstrnew
        end;
      tkbegin:
        begin
        if Declarations is TProcedureBody then
          begin
          ParseProcBeginBlock(TProcedureBody(Declarations));
          break;
          end
        else if (Declarations is TInterfaceSection)
        or (Declarations is TImplementationSection) then
          begin
          ParseInitialization;
          break;
          end
        else
          ParseExc(SParserSyntaxError);
        end;
      tklabel:
        begin
          if not (Declarations is TInterfaceSection) then
            ParseLabels(Declarations);
        end;
    else
      ParseExc(SParserSyntaxError);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    end;
end;

<<<<<<< HEAD
// Next token is expected to be a "(", ";" or for a function ":". The caller
// will get the token after the final ";" as next token.
procedure TPasParser.ParseProcedureOrFunctionHeader(Parent: TPasElement;
  Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);

  procedure ConsumeSemi;
  begin
    NextToken;
    if (CurToken <> tksemicolon) and IsCurTokenHint then
      ungettoken;
  end;

  function DoCheckHint : Boolean;

  var
    ahint : TPasMemberHint;
  begin
  Result:= IsCurTokenHint(ahint);
  if Result then  // deprecated,platform,experimental,library, unimplemented etc
    begin
    element.hints:=element.hints+[ahint];
    if aHint=hDeprecated then
      begin
      nextToken;
      if (CurToken<>tkString) then
        UnGetToken
      else
        element.HintMessage:=curtokenstring;
      end;
    end;
=======
// Starts after the "uses" token
procedure TPasParser.ParseUsesList(ASection: TPasSection);

function CheckUnit(AUnitName : string):TPasElement;
begin
    result := Engine.FindModule(AUnitName);  // should we resolve module here when "IN" filename is not known yet?
    if Assigned(result) then
      result.AddRef
    else
      Result := TPasType(CreateElement(TPasUnresolvedTypeRef, AUnitName,
        ASection));
    ASection.UsesList.Add(Result);
end;

var
  AUnitName: String;
  Element: TPasElement;
begin
  If not (Asection is TImplementationSection) Then // interface,program,library,package
    Element:=CheckUnit('System'); // system always implicitely first.    
  while True do
  begin
    AUnitName := ExpectIdentifier; 
    Element :=CheckUnit(AUnitName);

    NextToken;

    if CurToken = tkin then begin
      // todo: store unit's file name somewhere
      NextToken; // skip in
      ExpectToken(tkString); // skip unit's real file name
      if (Element is TPasModule) and (TPasmodule(Element).filename<>'')  then
        TPasModule(Element).FileName:=curtokenstring;
    end;

    if CurToken = tkSemicolon then
      break
    else if CurToken <> tkComma then
      ParseExc(SParserExpectedCommaSemicolon);
>>>>>>> graemeg/cpstrnew
  end;

<<<<<<< HEAD
Var
  Tok : String;
  CC : TCallingConvention;
  PM : TProcedureModifier;
  Done: Boolean;

begin
  // Element must be non-nil. Removed all checks for not-nil.
  // If it is nil, the following fails anyway.
  CheckProcedureArgs(Parent,Element.Args,ProcType in [ptOperator,ptClassOperator]);
  case ProcType of
    ptFunction,ptClassFunction:
      begin
      ExpectToken(tkColon);
      TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
      end;
    ptOperator,ptClassOperator:
      begin
      NextToken;
      if (CurToken=tkIdentifier) then
        begin
        TPasFunctionType(Element).ResultEl.Name := CurTokenName;
        ExpectToken(tkColon);
        end
      else
        if (CurToken=tkColon) then
          TPasFunctionType(Element).ResultEl.Name := 'Result'
        else
          ParseExc(SParserExpectedColonID);
        TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
      end;
  end;
  if OfObjectPossible then
    begin
=======
// Starts after the variable name
function TPasParser.ParseConstDecl(Parent: TPasElement): TPasConst;
begin
  Result := TPasConst(CreateElement(TPasConst, CurTokenString, Parent));
  try
>>>>>>> graemeg/cpstrnew
    NextToken;
    if (curToken =tkOf) then
      begin
      ExpectToken(tkObject);
      Element.IsOfObject := True;
      end 
    else if (curToken = tkIs) then
      begin
      expectToken(tkIdentifier);
      if (lowerCase(CurTokenString)<>'nested') then
        ParseExc(SParserExpectedNested);
      Element.isNested:=True;
      end
    else
      UnGetToken;  
    end;  
  NextToken;
  if CurToken = tkEqual then
    begin
    // for example: const p: procedure = nil;
    UngetToken;
    exit;
    end
  else
    UngetToken;
  Repeat
    NextToken;
    If TokenisCallingConvention(CurTokenString,cc) then
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
          if CurTokenIsIdentifier('legacy') or CurTokenIsIdentifier('BaseSysV') then
            NextToken;
          NextToken; // remove offset
          end;
      end;
      ExpectToken(tkSemicolon);
      end
    else if TokenIsProcedureModifier(Parent,CurTokenString,pm) then
      HandleProcedureModifier(Parent,Pm)
    else if (CurToken=tklibrary) then // library is a token and a directive.
      begin
      Tok:=UpperCase(CurTokenString);
      NextToken;
      If (tok<>'NAME') then
        Element.Hints:=Element.Hints+[hLibrary]
      else
        begin
        NextToken;  // Should be export name string.
        ExpectToken(tkSemicolon);
        end;
      end
    else if DoCheckHint then
      consumesemi
    else if (CurToken = tkSquaredBraceOpen) then
      begin
      repeat
        NextToken
      until CurToken = tkSquaredBraceClose;
      ExpectToken(tkSemicolon);
      end;
    Done:=(CurToken=tkSemiColon);
    if Done then
      begin
      NextToken;
      Done:=Not ((Curtoken=tkSquaredBraceOpen) or TokenIsProcedureModifier(Parent,CurtokenString,Pm) or IscurtokenHint() or TokenisCallingConvention(CurTokenString,cc));
//      DumpCurToken('Done '+IntToStr(Ord(Done)));
      UngetToken;
<<<<<<< HEAD
      end;
//    Writeln('Done: ',TokenInfos[Curtoken],' ',CurtokenString);
  Until Done;
  if DoCheckHint then  // deprecated,platform,experimental,library, unimplemented etc
    ConsumeSemi;
  if (ProcType in [ptOperator,ptClassOperator]) and (Parent is TPasOperator) then
    TPasOperator(Parent).CorrectName;
  if (Parent is TPasProcedure)
  and (not TPasProcedure(Parent).IsForward)
  and (not TPasProcedure(Parent).IsExternal)
  and ((Parent.Parent is TImplementationSection)
     or (Parent.Parent is TProcedureBody))
  then
    ParseProcedureBody(Parent);
=======

    ExpectToken(tkEqual);

    //skipping the expression as a value
    //Result.Value := ParseExpression;

    // using new expression parser!
    NextToken; // skip tkEqual
    Result.Expr:=DoParseConstValueExpression(Result);

    // must unget for the check to be peformed fine!
    UngetToken;

    CheckHint(Result,True);
  except
    Result.Free;
    raise;
  end;
>>>>>>> graemeg/cpstrnew
end;

// starts after the semicolon
procedure TPasParser.ParseProcedureBody(Parent: TPasElement);

var
  Body: TProcedureBody;

begin
<<<<<<< HEAD
  Body := TProcedureBody(CreateElement(TProcedureBody, '', Parent));
  TPasProcedure(Parent).Body:=Body;
  ParseDeclarations(Body);
=======
  Result := TPasResString(CreateElement(TPasResString, CurTokenString, Parent));
  try
    ExpectToken(tkEqual);
    Result.Value := ParseExpression(Result);
    CheckHint(Result,True);
  except
    Result.Free;
    raise;
  end;
>>>>>>> graemeg/cpstrnew
end;


function TPasParser.ParseProperty(Parent: TPasElement; const AName: String;
  AVisibility: TPasMemberVisibility): TPasProperty;

  procedure MaybeReadFullyQualifiedIdentifier(Var r : String);

  begin
<<<<<<< HEAD
    while True do
      begin
      NextToken;
      if CurToken = tkDot then
        begin
        ExpectIdentifier;
        R:=R + '.' + CurTokenString;
        end
      else
        break;
      end;
  end;

  function GetAccessorName: String;
  begin
    ExpectIdentifier;
    Result := CurTokenString;
    MaybeReadFullyQualifiedIdentifier(Result);
    if CurToken <> tkSquaredBraceOpen then
      UnGetToken
    else
      begin
      Result := Result + '[';
      NextToken;
      if CurToken in [tkIdentifier, tkNumber] then
        Result := Result + CurTokenString;
      ExpectToken(tkSquaredBraceClose);
      Result := Result + ']';
      end;
  end;

var
  isArray : Boolean;
  h   : TPasMemberHint;

begin
  Result:=TPasProperty(CreateElement(TPasProperty,AName,Parent,AVisibility));
  try
    NextToken;
    isArray:=CurToken=tkSquaredBraceOpen;
    if isArray then
=======
    Result := TPasRangeType(CreateElement(TPasRangeType, TypeName, Parent));
    try
      TPasRangeType(Result).RangeStart := ParseExpression(Result);
      ExpectToken(tkDotDot);
      TPasRangeType(Result).RangeEnd := ParseExpression(Result);
      CheckHint(Result,True);
    except
      Result.Free;
      raise;
    end;
  end;

var
  EnumValue: TPasEnumValue;
  Prefix : String;
  HadPackedModifier : Boolean;           // 12/04/04 - Dave - Added
  IsBitPacked : Boolean;
  
begin
  TypeName := CurTokenString;
  ExpectToken(tkEqual);
  NextToken;
  HadPackedModifier := False;     { Assume not present }
  if CurToken in [tkPacked,tkbitpacked] then     { If PACKED modifier }
     begin                        { Handle PACKED modifier for all situations }
     IsBitPacked:=CurToken=tkbitpacked;
     NextToken;                   { Move to next token for rest of parse }
     if CurToken in [tkArray, tkRecord, tkObject, tkClass] then  { If allowed }
       HadPackedModifier := True  { rememeber for later }
     else                         { otherwise, syntax error }
       ParseExc(Format(SParserExpectTokenError,['ARRAY, RECORD, OBJECT or CLASS']))
     end;
  case CurToken of
    tkRecord:
      begin
        Result := TPasRecordType(CreateElement(TPasRecordType, TypeName,
          Parent));
        try
          ParseRecordDecl(TPasRecordType(Result), False);
      CheckHint(Result,True);
          TPasRecordType(Result).IsPacked := HadPackedModifier;
          If HadPackedModifier then
            TPasRecordType(Result).IsBitPacked:=IsBitPacked;
        except
          Result.Free;
          raise;
        end;
      end;
    tkObject:
>>>>>>> graemeg/cpstrnew
      begin
      ParseArgList(Result, Result.Args, tkSquaredBraceClose);
      NextToken;
      end;
    if CurToken = tkColon then
      begin
      Result.VarType := ParseType(Result);
      NextToken;
      end;
    if CurTokenIsIdentifier('INDEX') then
      begin
      NextToken;
      Result.IndexExpr := DoParseExpression(Result);
      end;
    if CurTokenIsIdentifier('READ') then
      begin
<<<<<<< HEAD
      Result.ReadAccessorName := GetAccessorName;
      NextToken;
=======
        Prefix:=CurTokenString;
        NextToken;
        if CurToken = tkDot then
          begin
          ExpectIdentifier;
          NextToken;
          end
        else
          Prefix:='';
        if (CurToken = tkSemicolon) or IsCurTokenHint then
        begin
          UngetToken;
          UngetToken;
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName,
            Parent));
          try
            TPasAliasType(Result).DestType := ParseType(nil,Prefix);
            CheckHint(Result,True);
          except
            Result.Free;
            raise;
          end;
        end else if CurToken = tkSquaredBraceOpen then
        begin
          // !!!: Check for string type and store string length somewhere
          Result := TPasAliasType(CreateElement(TPasAliasType, TypeName,
            Parent));
          try
            TPasAliasType(Result).DestType :=
              TPasUnresolvedTypeRef.Create(CurTokenString, Parent);
            ParseExpression(Parent);
            ExpectToken(tkSquaredBraceClose);
            CheckHint(Result,True);
          except
            Result.Free;
            raise;
          end;
        end
        else  
        begin
          UngetToken;
          UngetToken;
          ParseRange;
        end;
>>>>>>> graemeg/cpstrnew
      end;
    if CurTokenIsIdentifier('WRITE') then
      begin
      Result.WriteAccessorName := GetAccessorName;
      NextToken;
      end;
    if CurTokenIsIdentifier('IMPLEMENTS') then
      begin
      Result.ImplementsName := GetAccessorName;
      NextToken;
      end;
    if CurTokenIsIdentifier('STORED') then
      begin
      NextToken;
      if CurToken = tkTrue then
        Result.StoredAccessorName := 'True'
      else if CurToken = tkFalse then
        Result.StoredAccessorName := 'False'
      else if CurToken = tkIdentifier then
        Result.StoredAccessorName := CurTokenString
      else
        ParseExc(SParserSyntaxError);
      NextToken;
      end;
    if CurTokenIsIdentifier('DEFAULT') then
      begin
<<<<<<< HEAD
      if isArray then
        ParseExc('Array properties cannot have default value');
      NextToken;
      Result.DefaultExpr := DoParseExpression(Result);
//      NextToken;
      end
    else if CurtokenIsIdentifier('NODEFAULT') then
=======
        Result := TPasEnumType(CreateElement(TPasEnumType, TypeName, Parent));
        try
          while True do
          begin
            NextToken;
            EnumValue := TPasEnumValue(CreateElement(TPasEnumValue,
              CurTokenString, Result));
            TPasEnumType(Result).Values.Add(EnumValue);
            NextToken;
            if CurToken = tkBraceClose then
              break
            else if CurToken in [tkEqual,tkAssign] then
              begin
              EnumValue.AssignedValue:=ParseExpression(result);
              NextToken;
              if CurToken = tkBraceClose then
                Break
              else if not (CurToken=tkComma) then
                ParseExc(SParserExpectedCommaRBracket);
              end
            else if not (CurToken=tkComma) then
              ParseExc(SParserExpectedCommaRBracket)
          end;
          CheckHint(Result,True);
        except
          Result.Free;
          raise;
        end;
      end;
    tkProcedure:
>>>>>>> graemeg/cpstrnew
      begin
      Result.IsNodefault:=true;
      NextToken;
      end;
    // Here the property ends. There can still be a 'default'
    if CurToken = tkSemicolon then
      NextToken;
    if CurTokenIsIdentifier('DEFAULT') then
      begin
      if (Result.VarType<>Nil) and (not isArray) then
        ParseExc('The default property must be an array property');
      NextToken;
      if CurToken = tkSemicolon then
        begin
        Result.IsDefault := True;
        NextToken;
        end
      end;
    // Handle hints
    while IsCurTokenHint(h) do
      begin
      Result.Hints:=Result.Hints+[h];
      NextToken;
      if CurToken=tkSemicolon then
        NextToken;
      end;
    UngetToken;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

// Starts after the "begin" token
procedure TPasParser.ParseProcBeginBlock(Parent: TProcedureBody);
var
<<<<<<< HEAD
  BeginBlock: TPasImplBeginBlock;
  SubBlock: TPasImplElement;
=======
  VarNames: TStringList;
  i: Integer;
  VarType: TPasType;
  VarEl: TPasVariable;
  H : TPasMemberHints;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin

  BeginBlock := TPasImplBeginBlock(CreateElement(TPasImplBeginBlock, '', Parent));
  Parent.Body := BeginBlock;
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
  ExpectToken(tkSemicolon);
//  writeln('TPasParser.ParseProcBeginBlock ended ',curtokenstring);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TPasParser.ParseAsmBlock(AsmBlock : TPasImplAsmStatement);
=======
=======
>>>>>>> graemeg/cpstrnew
    VarType := ParseComplexType(Parent);

    H:=CheckHint(Nil,False);
    NextToken;
>>>>>>> graemeg/cpstrnew

begin
  NextToken;
  While CurToken<>tkEnd do
    begin
    AsmBlock.Tokens.Add(CurTokenText);
    NextToken;
    end;
  // NextToken; // Eat end.
  // Do not consume end. Current token will normally be end;
end;

// Next token is start of (compound) statement
// After parsing CurToken is on last token of statement
procedure TPasParser.ParseStatement(Parent: TPasImplBlock;
  out NewImplElement: TPasImplElement);
var
<<<<<<< HEAD
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
=======
  i: Integer;
  VarType: TPasType;
  Value, S: String;
  M: string;
  H : TPasMemberHints;
begin
  while True do
  begin
    List.Add(CreateElement(TPasVariable, CurTokenString, Parent));
    NextToken;
    if CurToken = tkColon then
      break
    else if CurToken <> tkComma then
      ParseExc(SParserExpectedCommaColon);
    ExpectIdentifier;
>>>>>>> graemeg/cpstrnew
  end;
  {$ENDIF}

  function CloseBlock: boolean; // true if parent reached
  begin
    CurBlock:=CurBlock.Parent as TPasImplBlock;
    Result:=CurBlock=Parent;
  end;
<<<<<<< HEAD
<<<<<<< HEAD

  function CloseStatement(CloseIfs: boolean): boolean; // true if parent reached
=======
=======
>>>>>>> graemeg/cpstrnew
  //writeln('TPasParser.ParseVarDecl ',CurTokenText);
  NextToken;
  // Writeln(LastVar,': Parsed complex type: ',CurtokenText);
  // NextToken;
  // Writeln(LastVar,': Parsed complex type, next: ',CurtokenText);
  If CurToken=tkEqual then
    begin
    Value := ParseExpression(Parent);
    for i := 0 to List.Count - 1 do
      TPasVariable(List[i]).Value := Value;
    NextToken;
    end;

  if CurToken = tkAbsolute then
>>>>>>> graemeg/cpstrnew
  begin
    if CurBlock=Parent then exit(true);
    while CurBlock.CloseOnSemicolon
    or (CloseIfs and (CurBlock is TPasImplIfElse)) do
      if CloseBlock then exit(true);
    Result:=false;
  end;

  procedure CreateBlock(NewBlock: TPasImplBlock);
  begin
<<<<<<< HEAD
    CurBlock.AddElement(NewBlock);
    CurBlock:=NewBlock;
    if NewImplElement=nil then NewImplElement:=CurBlock;
  end;
=======
    NextToken;
    if CurToken = tkIdentifier then
      begin
      s := UpperCase(CurTokenText);
      if s = 'CVAR' then
        begin
        M := M + '; cvar';
        ExpectToken(tkSemicolon);
        end
      else if (s = 'EXTERNAL') or (s = 'PUBLIC') or (s = 'EXPORT') then
        begin
        M := M + ';' + CurTokenText;
        if s = 'EXTERNAL' then
          begin
          NextToken;
          if ((CurToken = tkString) or (CurToken = tkIdentifier)) and (UpperCase(CurTokenText)<> 'NAME') then
            begin
            // !!!: Is this really correct for tkString?
            M := M + ' ' + CurTokenText;
            NextToken;
            end;
          end
        else
          NextToken;
        if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'NAME') then
          begin
          M := M + ' name ';
          NextToken;
          if (CurToken = tkString) or (CurToken = tkIdentifier) then
            // !!!: Is this really correct for tkString?
            M := M + CurTokenText
          else
            ParseExc(SParserSyntaxError);
          H:=CheckHint(Nil,True);
          If (H<>[]) then
            for i := 0 to List.Count - 1 do
              TPasVariable(List[i]).Hints:=H;

          // ExpectToken(tkSemicolon);
          end
        else if CurToken <> tkSemicolon then
          ParseExc(SParserSyntaxError);
      end else
      begin
        UngetToken;
        break;
      end
    end else
    begin
      UngetToken;
      break;
    end;
  end; // while

  if M <> '' then
    for i := 0 to List.Count - 1 do
      TPasVariable(List[i]).Modifiers := M;
end;
>>>>>>> graemeg/cpstrnew

var
  VarName: String;
  SubBlock: TPasImplElement;
  CmdElem: TPasImplElement;
  left: TPasExpr;
  right: TPasExpr;
  el : TPasImplElement;
  ak : TAssignKind;
  lt : TLoopType;

begin
  NewImplElement:=nil;
  CurBlock := Parent;
  while True do
  begin
    NextToken;
    //WriteLn(i,'Token=',CurTokenText);
    case CurToken of
    tkasm :
      begin
      el:=TPasImplElement(CreateElement(TPasImplAsmStatement,'',CurBlock));
      ParseAsmBlock(TPasImplAsmStatement(el));
      CurBlock.AddElement(el);
      NewImplElement:=El;
      end;
    tkbegin:
      begin
      el:=TPasImplElement(CreateElement(TPasImplBeginBlock,'',CurBlock));
      CreateBlock(TPasImplBeginBlock(el));
      end;
    tkrepeat:
      begin
      el:=TPasImplRepeatUntil(CreateElement(TPasImplRepeatUntil,'',CurBlock));
      CreateBlock(TPasImplRepeatUntil(el));
      end;
    tkIf:
      begin
        NextToken;
        Left:=DoParseExpression(CurBlock);
        UNgettoken;
        el:=TPasImplIfElse(CreateElement(TPasImplIfElse,'',CurBlock));
        TPasImplIfElse(el).ConditionExpr:=Left;
        //WriteLn(i,'IF Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(TPasImplIfElse(el));
        ExpectToken(tkthen);
      end;
    tkelse:
      if (CurBlock is TPasImplIfElse) then
      begin
        if TPasImplIfElse(CurBlock).IfBranch=nil then
        begin
        el:=TPasImplCommand(CreateElement(TPasImplCommand,'', CurBlock));
        CurBlock.AddElement(el);
        end;
        if TPasImplIfElse(CurBlock).ElseBranch<>nil then
        begin
          // this and the following 3 may solve TPasImplIfElse.AddElement BUG
          // ifs without begin end
          // if .. then
          //  if .. then
          //   else
          // else
          CloseBlock;
          CloseStatement(false);
        end;
      end else if (CurBlock is TPasImplWhileDo) then
      begin
        //if .. then while .. do smt else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplRaise) then
      begin
<<<<<<< HEAD
        //if .. then Raise Exception else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplTryExcept) then
      begin
        CloseBlock;
        el:=TPasImplTryExceptElse(CreateElement(TPasImplTryExceptElse,'',CurBlock));
        TPasImplTry(CurBlock).ElseBranch:=TPasImplTryExceptElse(el);
        CurBlock:=TPasImplTryExceptElse(el);
      end else
        ParseExc(SParserSyntaxError);
    tkwhile:
      begin
<<<<<<< HEAD
        // while Condition do
        NextToken;
        left:=DoParseExpression(Parent);
        ungettoken;
        //WriteLn(i,'WHILE Condition="',Condition,'" Token=',CurTokenText);
        el:=TPasImplWhileDo(CreateElement(TPasImplWhileDo,'',CurBlock));
        TPasImplWhileDo(el).ConditionExpr:=left;
        CreateBlock(TPasImplWhileDo(el));
        ExpectToken(tkdo);
      end;
    tkgoto:
      begin
        nexttoken;
        curblock.AddCommand('goto '+curtokenstring);
        expecttoken(tkSemiColon);
      end;
    tkfor:
      begin
        // for VarName := StartValue to EndValue do
        // for VarName in Expression do
        ExpectIdentifier;
        VarName:=CurTokenString;
        NextToken;
        Left:=Nil;
        Right:=Nil;
        if Not (CurToken in [tkAssign,tkIn]) then
          ParseExc(SParserExpectedAssignIn);
        if (CurToken=tkAssign) then
          lt:=ltNormal
        else
          lt:=ltin;
        NextToken;
        Left:=DoParseExpression(Parent);
        Try
          if (Lt=ltNormal) then
=======
=======
>>>>>>> graemeg/cpstrnew
        Value := ParseExpression(Parent);
      end else
        UngetToken;
    end;

    for i := 0 to ArgNames.Count - 1 do
    begin
      Arg := TPasArgument(CreateElement(TPasArgument, ArgNames[i], Parent));
      Arg.Access := Access;
      Arg.ArgType := ArgType;
      if (i > 0) and Assigned(ArgType) then
        ArgType.AddRef;
      Arg.Value := Value;
      Args.Add(Arg);
    end;

    ArgNames.Free;
    NextToken;
    if CurToken = EndToken then
      break;
  end;
end;

// Next token is expected to be a "(", ";" or for a function ":". The caller
// will get the token after the final ";" as next token.
procedure TPasParser.ParseProcedureOrFunctionHeader(Parent: TPasElement;
  Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);

procedure ConsumeSemi;
begin
  NextToken;
  if (CurToken <> tksemicolon) and IsCurTokenHint then
    ungettoken;
end;

Var
  Tok : String;
  i: Integer;
  Proc: TPasProcedure;
  ahint : TPasMemberHint;
begin
  NextToken;
  case ProcType of
    ptFunction,ptClassFunction:
      begin
        if CurToken = tkBraceOpen then
        begin
          NextToken;
          if (CurToken <> tkBraceClose) then
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
            begin
            if Not (CurToken in [tkTo,tkDownTo]) then
              ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkTo]]));
            if CurToken=tkdownto then
              Lt:=ltDown;
            NextToken;
            Right:=DoParseExpression(Parent);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          if (CurToken<>tkDo) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkDo]]));
        except
          FreeAndNil(Left);
          FreeAndNil(Right);
          Raise;
        end;
        el:=TPasImplForLoop(CreateElement(TPasImplForLoop,'',CurBlock));
        TPasImplForLoop(el).VariableName:=VarName;
        TPasImplForLoop(el).StartExpr:=Left;
        TPasImplForLoop(el).EndExpr:=Right;
        TPasImplForLoop(el).LoopType:=lt;
        CreateBlock(TPasImplForLoop(el));
        //WriteLn(i,'FOR "',VarName,'" := ',StartValue,' to ',EndValue,' Token=',CurTokenText);
      end;
    tkwith:
      begin
        // with Expr do
        // with Expr, Expr do
        NextToken;
        Left:=DoParseExpression(Parent);
        //writeln(i,'WITH Expr="',Expr,'" Token=',CurTokenText);
        el:=TPasImplWithDo(CreateElement(TPasImplWithDo,'',CurBlock));
        TPasImplWithDo(el).AddExpression(Left);
        CreateBlock(TPasImplWithDo(el));
        repeat
          if CurToken=tkdo then break;
          if CurToken<>tkComma then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkdo]]));
=======
=======
>>>>>>> graemeg/cpstrnew
          ExpectToken(tkColon);
        end else if CurToken <> tkColon then
          ParseExc(SParserExpectedLBracketColon);
        if Assigned(Element) then        // !!!
          TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
        else
          ParseType(nil);
      end;

    ptProcedure,ptConstructor,ptDestructor,ptClassProcedure:
      begin
        if CurToken = tkBraceOpen then
        begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
          NextToken;
          Left:=DoParseExpression(Parent);
          //writeln(i,'WITH ...,Expr="',Expr,'" Token=',CurTokenText);
          TPasImplWithDo(CurBlock).AddExpression(Left);
        until false;
      end;
    tkcase:
      begin
        NextToken;
        Left:=DoParseExpression(Parent);
        UngetToken;
        //writeln(i,'CASE OF Expr="',Expr,'" Token=',CurTokenText);
        ExpectToken(tkof);
        el:=TPasImplCaseOf(CreateElement(TPasImplCaseOf,'',CurBlock));
        TPasImplCaseOf(el).CaseExpr:=Left;
        CreateBlock(TPasImplCaseOf(el));
        repeat
          NextToken;
          //writeln(i,'CASE OF Token=',CurTokenText);
          case CurToken of
          tkend:
            begin
            if CurBlock.Elements.Count=0 then
              ParseExc(SParserExpectCase);
            break; // end without else
            end;
          tkelse:
            begin
              // create case-else block
              el:=TPasImplCaseElse(CreateElement(TPasImplCaseElse,'',CurBlock));
              TPasImplCaseOf(CurBlock).ElseBranch:=TPasImplCaseElse(el);
              CreateBlock(TPasImplCaseElse(el));
              break;
            end
<<<<<<< HEAD
<<<<<<< HEAD
          else
            // read case values
            if (curToken=tkIdentifier) and (LowerCase(CurtokenString)='otherwise') then
              begin
              // create case-else block
              el:=TPasImplCaseElse(CreateElement(TPasImplCaseElse,'',CurBlock));
              TPasImplCaseOf(CurBlock).ElseBranch:=TPasImplCaseElse(el);
              CreateBlock(TPasImplCaseElse(el));
              break;
              end
            else
              repeat
                Left:=DoParseExpression(Parent);
                //writeln(i,'CASE value="',Expr,'" Token=',CurTokenText);
                if CurBlock is TPasImplCaseStatement then
                  TPasImplCaseStatement(CurBlock).Expressions.Add(Left)
                else
                  begin
                  el:=TPasImplCaseStatement(CreateElement(TPasImplCaseStatement,'',CurBlock));
                  TPasImplCaseStatement(el).AddExpression(Left);
                  CurBlock.AddElement(el);
                  CurBlock:=TPasImplCaseStatement(el);
                  end;
                //writeln(i,'CASE after value Token=',CurTokenText);
                if (CurToken=tkComma) then
                  NextToken
                else if (CurToken<>tkColon) then
                  ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkComma]]))
              until Curtoken=tkColon;
            // read statement
            ParseStatement(CurBlock,SubBlock);
            CloseBlock;
            if CurToken<>tkSemicolon then
            begin
              NextToken;
              if not (CurToken in [tkSemicolon,tkelse,tkend]) then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
              if CurToken<>tkSemicolon then
                UngetToken;
            end;
          end;
        until false;
        if CurToken=tkend then
        begin
          if CloseBlock then break;
          if CloseStatement(false) then break;
        end;
=======
=======
>>>>>>> graemeg/cpstrnew
        end else if (CurToken = tkSemicolon)
          or (OfObjectPossible and (CurToken in [tkOf,tkEqual]))
        then
          UngetToken
        else
          ParseExc(SParserExpectedLBracketSemicolon);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
      end;
    tktry:
      begin
<<<<<<< HEAD
      el:=TPasImplTry(CreateElement(TPasImplTry,'',Curblock));
      CreateBlock(TPasImplTry(el));
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
          el:=TPasImplTryFinally(CreateElement(TPasImplTryFinally,'',Curblock));
          TPasImplTry(CurBlock).FinallyExcept:=TPasImplTryFinally(el);
          CurBlock:=TPasImplTryFinally(el);
        end else
          ParseExc(SParserSyntaxError);
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
          el:=TPasImplTryExcept(CreateElement(TPasImplTryExcept,'',CurBlock));
          TPasImplTry(CurBlock).FinallyExcept:=TPasImplTryExcept(el);
          CurBlock:=TPasImplTryExcept(el);
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkon:
      begin
        // in try except:
        // on E: Exception do
        // on Exception do
        if CurBlock is TPasImplTryExcept then
        begin
          NextToken;
          Left:=Nil;
          Right:=DoParseExpression(Parent);
          //writeln(i,'ON t=',TypeName,' Token=',CurTokenText);
  //        NextToken;
          if CurToken=tkColon then
            begin
            NextToken;
            Left:=Right;
            Right:=DoParseExpression(Parent);
            //writeln(i,'ON v=',VarName,' t=',TypeName,' Token=',CurTokenText);
            end;
//          else
          UngetToken;
          el:=TPasImplExceptOn(CreateElement(TPasImplExceptOn,'',CurBlock));
          TPasImplExceptOn(el).VarExpr:=Left;
          TPasImplExceptOn(el).TypeExpr:=Right;
          CurBlock.AddElement(el);
          CurBlock:=TPasImplExceptOn(el);
          ExpectToken(tkDo);
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkraise:
      begin
      el:=TPasImplRaise(CreateElement(TPasImplRaise,'',CurBlock));
      CreateBlock(TPasImplRaise(el));
      NextToken;
      If Curtoken=tkSemicolon then
        UnGetToken
      else
        begin
        TPasImplRaise(el).ExceptObject:=DoParseExpression(el);
        if (CurToken=tkIdentifier) and (Uppercase(CurtokenString)='AT') then
          begin
          NextToken;
          TPasImplRaise(el).ExceptAddr:=DoParseExpression(el);
          end;
        if Curtoken in [tkSemicolon,tkEnd] then
          UngetToken
        end;
      end;
    tkend:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplBeginBlock then
        begin
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
          ParseExc(SParserSyntaxError);
      end;
    tkSemiColon:
      if CloseStatement(true) then break;
    tkuntil:
      begin
<<<<<<< HEAD
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplRepeatUntil then
        begin
          NextToken;
          Left:=DoParseExpression(Parent);
          UngetToken;
          TPasImplRepeatUntil(CurBlock).ConditionExpr:=Left;
          //WriteLn(i,'UNTIL Condition="',Condition,'" Token=',CurTokenString);
          if CloseBlock then break;
        end else
          ParseExc(SParserSyntaxError);
      end;
=======
=======
>>>>>>> graemeg/cpstrnew
        ParseArgList(Element, Element.Args, tkBraceClose);
        NextToken;
        if (CurToken=tkIdentifier) then
        begin
          TPasFunctionType(Element).ResultEl.Name := CurTokenName;
          ExpectToken(tkColon);
        end
        else if (CurToken=tkColon) then
          TPasFunctionType(Element).ResultEl.Name := 'Result'
        else
          ParseExc(SParserExpectedColonID);
        if Assigned(Element) then        // !!!
          TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
        else
          ParseType(nil);
      end;
  end;

  NextToken;
  if OfObjectPossible and (CurToken = tkOf) then
  begin
    ExpectToken(tkObject);
    Element.IsOfObject := True;
  end else
    UngetToken;

  NextToken;
  if CurToken = tkEqual then
  begin
    // for example: const p: procedure = nil;
    UngetToken;
    exit;
  end else
    UngetToken;

  ExpectToken(tkSemicolon);
  while True do
    begin
    // CheckHint(Element,False);
    NextToken;
    if (CurToken = tkIdentifier) or (CurToken=tklibrary) then // library is a token and a directive.
      begin
      Tok:=UpperCase(CurTokenString);
      If (Tok='CDECL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccCDecl;
        ExpectToken(tkSemicolon);
        end
      else If (Tok='EXPORT') then
        begin
        TPasProcedure(Parent).AddModifier(pmExported);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='PASCAL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccPascal;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='STDCALL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccStdCall;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='OLDFPCCALL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccOldFPCCall;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='EXTDECL') then
        begin
        // extdecl is a common macro for external functions
        TPasProcedure(Parent).AddModifier(pmExternal);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='EXTERNAL') then
        begin
        TPasProcedure(Parent).AddModifier(pmExternal);
        NextToken;
        if CurToken in [tkString,tkIdentifier] then
        begin
          NextToken;
          Tok:=UpperCase(CurTokenString);
          if Tok='NAME' then
          begin
            NextToken;
            if not (CurToken in [tkString,tkIdentifier]) then
              ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkString]]));
          end;
        end else
          UngetToken;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='REGISTER') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccRegister;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='COMPILERPROC') then
        begin
        TPasProcedure(Parent).AddModifier(pmCompilerProc);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='VARARGS') then
        begin
        TPasProcedure(Parent).AddModifier(pmVarArgs);
        ExpectToken(tkSemicolon);
        end
      else if IsCurTokenHint(ahint) then  // deprecated,platform,experimental,library, unimplemented etc
        begin
        element.hints:=element.hints+[ahint];
        consumesemi;
        end
      else if (tok='OVERLOAD') then
        begin
        TPasProcedure(Parent).AddModifier(pmOverload);
        ExpectToken(tkSemicolon);
        end
      else if (tok='INLINE') then
        begin
        TPasProcedure(Parent).AddModifier(pmInline);
        ExpectToken(tkSemicolon);
        end
      else if (tok='ASSEMBLER') then
        begin
        TPasProcedure(Parent).AddModifier(pmAssembler);
        ExpectToken(tkSemicolon);
        end
      else if (tok = 'EXTERNAL') then
        repeat
          NextToken;
        until CurToken = tkSemicolon
      else if (tok = 'PUBLIC') then
        begin
          NextToken;
          { Should be token Name,
            if not we're in a class and the public section starts }
          If (Uppercase(CurTokenString)<>'NAME') then
            begin
            UngetToken;
            UngetToken;
            Break;
            end
          else
            begin
            NextToken;  // Should be export name string.
            ExpectToken(tkSemicolon);
            end;
        end
      else if (tok = 'FORWARD') then
        begin
          if (Parent.Parent is TInterfaceSection) then
            begin
            UngetToken;
            break;
            end;
          if Parent is TPasProcedure then
            TPasProcedure(Parent).AddModifier(pmForward);
          ExpectToken(tkSemicolon);
        end
      else
        begin
        UnGetToken;
        Break;
        end
      end
    else if (CurToken = tkInline) then
      begin
      if Parent is TPasProcedure then
        TPasProcedure(Parent).AddModifier(pmInline);
      ExpectToken(tkSemicolon);
      end
    else if (CurToken = tkSquaredBraceOpen) then
      begin
      repeat
        NextToken
      until CurToken = tkSquaredBraceClose;
      ExpectToken(tkSemicolon);
      end
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    else
      left:=DoParseExpression(nil);
      case CurToken of
        tkAssign,
        tkAssignPlus,
        tkAssignMinus,
        tkAssignMul,
        tkAssignDivision:
        begin
          // assign statement
          Ak:=TokenToAssignKind(CurToken);
          NextToken;
          right:=DoParseExpression(nil); // this may solve TPasImplWhileDo.AddElement BUG
          el:=TPasImplAssign(CreateElement(TPasImplAssign,'',CurBlock));
          TPasImplAssign(el).left:=Left;
          TPasImplAssign(el).right:=Right;
          TPasImplAssign(el).Kind:=ak;
          CurBlock.AddElement(el);
          CmdElem:=TPasImplAssign(el);
          UngetToken;
        end;
        tkColon:
        begin
          if not (left is TPrimitiveExpr) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
          // label mark. todo: check mark identifier in the list of labels
          el:=TPasImplLabelMark(CreateElement(TPasImplLabelMark,'', CurBlock));
          TPasImplLabelMark(el).LabelId:=TPrimitiveExpr(left).Value;
          CurBlock.AddElement(el);
          CmdElem:=TPasImplLabelMark(el);
          left.Free;
        end;
      else
        // simple statement (function call)
        el:=TPasImplSimple(CreateElement(TPasImplSimple,'',CurBlock));
        TPasImplSimple(el).expr:=Left;
        CurBlock.AddElement(el);
        CmdElem:=TPasImplSimple(el);
        UngetToken;
      end;

      if not (CmdElem is TPasImplLabelMark) then
        if NewImplElement=nil then NewImplElement:=CmdElem;
    end;
<<<<<<< HEAD
  end;
=======
  if (ProcType = ptOperator) and (Parent is TPasProcedure) then
  begin
    Proc:=TPasProcedure(Parent);
    Proc.Name := Proc.Name + '(';
    for i := 0 to Proc.ProcType.Args.Count - 1 do
    begin
      if i > 0 then
        Proc.Name := Proc.Name + ', ';
      Proc.Name := Proc.Name +
        TPasArgument(Proc.ProcType.Args[i]).ArgType.Name;
    end;
<<<<<<< HEAD
=======
  if (ProcType = ptOperator) and (Parent is TPasProcedure) then
  begin
    Proc:=TPasProcedure(Parent);
    Proc.Name := Proc.Name + '(';
    for i := 0 to Proc.ProcType.Args.Count - 1 do
    begin
      if i > 0 then
        Proc.Name := Proc.Name + ', ';
      Proc.Name := Proc.Name +
        TPasArgument(Proc.ProcType.Args[i]).ArgType.Name;
    end;
>>>>>>> graemeg/cpstrnew
    Proc.Name := Proc.Name + '): ' +
      TPasFunctionType(Proc.ProcType).ResultEl.ResultType.Name;
  end;

  if (Parent is TPasProcedure)
  and (not TPasProcedure(Parent).IsForward)
  and (not TPasProcedure(Parent).IsExternal)
  and ((Parent.Parent is TImplementationSection)
     or (Parent.Parent is TProcedureBody))
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;

procedure TPasParser.ParseLabels(AParent: TPasElement);
var
  Labels: TPasLabels;
begin
  Labels:=TPasLabels(CreateElement(TPasLabels, '', AParent));
  repeat
    Labels.Labels.Add(ExpectIdentifier);
    NextToken;
    if not (CurToken in [tkSemicolon, tkComma]) then
      ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
  until CurToken=tkSemicolon;
=======
>>>>>>> graemeg/cpstrnew
end;

// Starts after the "procedure" or "function" token
function TPasParser.GetProcedureClass(ProcType: TProcType): TPTreeElement;

<<<<<<< HEAD
begin
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
  else
    ParseExc('Unknown procedure Type '+intToStr(Ord(ProcType)));
=======
var
  isArray : Boolean;

var
  isArray : Boolean;

  procedure MaybeReadFullyQualifiedIdentifier(Var r : String);

  begin
    while True do
      begin
      NextToken;
      if CurToken = tkDot then
        begin
        ExpectIdentifier;
        R:=R + '.' + CurTokenString;
        end
      else
        break;
      end;
>>>>>>> graemeg/cpstrnew
  end;
end;

function TPasParser.ParseProcedureOrFunctionDecl(Parent: TPasElement; ProcType: TProcType;AVisibility : TPasMemberVisibility = VisDefault): TPasProcedure;

  function ExpectProcName: string;
  begin
    Result:=ExpectIdentifier;
    //writeln('ExpectProcName ',Parent.Classname);
    if Parent is TImplementationSection then
    begin
      NextToken;
<<<<<<< HEAD
      if CurToken=tkDot then
      begin
        Result:=Result+'.'+ExpectIdentifier;
      end else
        UngetToken;
    end;
  end;

var
  Name: String;
  PC : TPTreeElement;
  Ot : TOperatorType;
  IsTokenBased : Boolean;

begin
  If (Not (ProcType in [ptOperator,ptClassOperator])) then
    Name:=ExpectProcName
  else
    begin
=======
      if CurToken in [tkIdentifier, tkNumber] then begin
    Result := Result + CurTokenString;
      end;
      ExpectToken(tkSquaredBraceClose);
      Result := Result + ']';
    end else
      UngetToken;
    //MaybeReadFullyQualifiedIdentifier(Result);
    //writeln(Result);
  end;

var
  us  : String; 
  h   : TPasMemberHint;
begin
  isArray:=False;
  NextToken;
// if array prop then parse [ arg1:type1;... ]

  if CurToken = tkSquaredBraceOpen then begin
    isArray:=True;
  // !!!: Parse array properties correctly
    ParseArgList(Element, TPasProperty(Element).Args, tkSquaredBraceClose);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
    NextToken;
    IsTokenBased:=Curtoken<>tkIdentifier;
    if IsTokenBased then
      OT:=TPasOperator.TokenToOperatorType(CurTokenText)
    else
      OT:=TPasOperator.NameToOperatorType(CurTokenString);
    if (ot=otUnknown) then
      ParseExc(SErrUnknownOperatorType,[CurTokenString]);
    Name:=OperatorNames[Ot];
    end;
  PC:=GetProcedureClass(ProcType);
  Parent:=CheckIfOverLoaded(Parent,Name);
  Result:=TPasProcedure(CreateElement(PC,Name,Parent,AVisibility));
  try
    if Not (ProcType in [ptFunction, ptClassFunction, ptOperator, ptClassOperator]) then
      Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '', Result))
    else
      begin
      Result.ProcType := CreateFunctionType('', 'Result', Result, True);
      if (ProcType in [ptOperator, ptClassOperator]) then
        begin
        TPasOperator(Result).TokenBased:=IsTokenBased;
        TPasOperator(Result).OperatorType:=OT;
        TPasOperator(Result).CorrectName;
        end;
      end;
    ParseProcedureOrFunctionHeader(Result, Result.ProcType, ProcType, False);
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
            end;
            Name:=OperatorNames[OperatorType];
            TPasOperator(Result).CorrectName;
            end;
          end;
        end;
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

<<<<<<< HEAD
// Current token is the first token after tkOf
procedure TPasParser.ParseRecordVariantParts(ARec: TPasRecordType;
  AEndToken: TToken);
=======
    NextToken;
  end;
>>>>>>> graemeg/cpstrnew

Var
  M : TPasRecordType;
  V : TPasVariant;
  Done : Boolean;
=======
=======
>>>>>>> graemeg/cpstrnew
  if CurToken = tkColon then begin
// if ":prop_data_type" if supplied then read it
  // read property type
    TPasProperty(Element).VarType := ParseType(Element);
    NextToken;
<<<<<<< HEAD
  end;

  if CurToken <> tkSemicolon then begin
//  if indexed prop then read the index value
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'INDEX') then begin
//    read 'index' access modifier
      TPasProperty(Element).IndexValue := ParseExpression(Element,ek_PropertyIndex);
    end else
//    not indexed prop will be recheck for another token
      UngetToken;
>>>>>>> graemeg/cpstrnew

begin
  Repeat
    V:=TPasVariant(CreateElement(TPasVariant, '', ARec));
    ARec.Variants.Add(V);
    Repeat
      NextToken;
      V.Values.Add(DoParseExpression(ARec));
      if Not (CurToken in [tkComma,tkColon]) then
        ParseExc(SParserExpectedCommaColon);
    Until (curToken=tkColon);
    ExpectToken(tkBraceOpen);
    NextToken;
<<<<<<< HEAD
    M:=TPasRecordType(CreateElement(TPasRecordType,'',V));
    V.Members:=M;
    ParseRecordFieldList(M,tkBraceClose,False);
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

procedure TPasParser.DumpCurToken(const Msg: String; IndentAction: TIndentAction
  );
begin
  if IndentAction=iaUndent then
    FDumpIndent:=copy(FDumpIndent,1,Length(FDumpIndent)-2);
  Writeln(FDumpIndent,Msg,' : ',TokenInfos[CurToken],' "',CurTokenString,'", Position: ',Scanner.CurFilename,'(',Scanner.CurRow,',',SCanner.CurColumn,') : ',Scanner.CurLine);
  if IndentAction=iaIndent then
    FDumpIndent:=FDumpIndent+'  ';
  Flush(output);
end;

// Starts on first token after Record or (. Ends on AEndToken
procedure TPasParser.ParseRecordFieldList(ARec: TPasRecordType;
  AEndToken: TToken; AllowMethods: Boolean);

Var
  VN : String;
  v : TPasmemberVisibility;
  Proc: TPasProcedure;
  ProcType: TProcType;
  Prop : TPasProperty;
  Cons : TPasConst;
  isClass : Boolean;

begin
  v:=visDefault;
  isClass:=False;
  while CurToken<>AEndToken do
    begin
    SaveComments;
    Case CurToken of
      tkConst:
        begin
        if Not AllowMethods then
          ParseExc(SErrRecordConstantsNotAllowed);
        ExpectToken(tkIdentifier);
        Cons:=ParseConstDecl(ARec);
        Cons.Visibility:=v;
        ARec.members.Add(Cons);
        end;
      tkClass:
        begin
        if Not AllowMethods then
          ParseExc(SErrRecordMethodsNotAllowed);
        if isClass then
          ParseExc(SParserTypeSyntaxError);
        isClass:=True;
        end;
      tkProperty:
        begin
        if Not AllowMethods then
          ParseExc(SErrRecordPropertiesNotAllowed);
        ExpectToken(tkIdentifier);
        Prop:=ParseProperty(ARec,CurtokenString,v);
        Prop.isClass:=isClass;
        Arec.Members.Add(Prop);
        end;
      tkOperator,
      tkProcedure,
      tkFunction :
        begin
        if Not AllowMethods then
          ParseExc(SErrRecordMethodsNotAllowed);
        ProcType:=GetProcTypeFromtoken(CurToken,isClass);
        Proc:=ParseProcedureOrFunctionDecl(ARec,ProcType,v);
        if Proc.Parent is TPasOverloadedProc then
          TPasOverloadedProc(Proc.Parent).Overloads.Add(Proc)
        else
          ARec.Members.Add(Proc);
        end;
      tkIdentifier :
        begin
//        If (po_delphi in Scanner.Options) then
          if CheckVisibility(CurtokenString,v) then
            begin
            If not (po_delphi in Scanner.Options) then
              ParseExc(SErrRecordVisibilityNotAllowed);
            if not (v in [visPrivate,visPublic,visStrictPrivate]) then
              ParseExc(SParserInvalidRecordVisibility);
            NextToken;
            Continue;
            end;
        ParseInlineVarDecl(ARec, ARec.Members, v, AEndToken=tkBraceClose);
        end;
      tkCase :
        begin
        ARec.Variants:=TFPList.Create;
        NextToken;
        VN:=CurTokenString;
        NextToken;
        If CurToken=tkColon then
          ARec.VariantName:=VN
        else
          begin
          UnGetToken;
          UnGetToken;
          end;
        ARec.VariantType:=ParseType(ARec);
        ExpectToken(tkOf);
        ParseRecordVariantParts(ARec,AEndToken);
        end;
    else
      ParseExc(SParserTypeSyntaxError);
    end;
    If CurToken<>tkClass then
      isClass:=False;
    if CurToken<>AEndToken then
      NextToken;
    end;
end;
=======
  end;

// if the accessors list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'read' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'READ') then
      TPasProperty(Element).ReadAccessorName := GetAccessorName
    else
//    not read accessor will be recheck for another token
      UngetToken;

    NextToken;
  end;

// if the accessors list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'write' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'WRITE') then
      TPasProperty(Element).WriteAccessorName := GetAccessorName
    else
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'IMPLEMENTS') then
      TPasProperty(Element).ImplementsName := GetAccessorName
    else
//    not write accessor will be recheck for another token
      UngetToken;

    NextToken;
=======
>>>>>>> graemeg/cpstrnew
  end;

// if the specifiers list is not finished
  if CurToken <> tkSemicolon then begin
<<<<<<< HEAD
    // read 'stored' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'STORED') then begin
      NextToken;
      if CurToken = tkTrue then
        TPasProperty(Element).StoredAccessorName := 'True'
      else if CurToken = tkFalse then
        TPasProperty(Element).StoredAccessorName := 'False'
      else if CurToken = tkIdentifier then
        TPasProperty(Element).StoredAccessorName := CurTokenString
      else
        ParseExc(SParserSyntaxError);
    end else
//    not stored accessor will be recheck for another token
      UngetToken;

    NextToken;
  end;

// if the specifiers list is not finished
  if (CurToken <> tkSemicolon) and (CurToken = tkIdentifier) then begin
    us:=UpperCase(CurTokenText);
    if (us = 'DEFAULT') then begin
      if isArray then ParseExc('Array properties cannot have default value');
//    read 'default' value modifier -> ParseExpression(DEFAULT <value>)
      TPasProperty(Element).DefaultValue := ParseExpression(Element);
      NextToken;
    end else if (us = 'NODEFAULT') then begin
//    read 'nodefault' modifier
      TPasProperty(Element).IsNodefault:=true;
    end else
//    not "default <value>" prop will be recheck for another token
      UngetToken;
<<<<<<< HEAD
    NextToken;
  end;
>>>>>>> graemeg/cpstrnew

// Starts after the "record" token
function TPasParser.ParseRecordDecl(Parent: TPasElement;
  const TypeName: string; const Packmode: TPackMode): TPasRecordType;

<<<<<<< HEAD
begin
    Result := TPasRecordType(CreateElement(TPasRecordType, TypeName, Parent));
    try
      Result.PackMode:=PackMode;
      NextToken;
      ParseRecordFieldList(Result,tkEnd,true);
    except
      FreeAndNil(Result);
      Raise;
    end;
end;

Function IsVisibility(S : String;  var AVisibility :TPasMemberVisibility) : Boolean;

Const
  VNames : array[TPasMemberVisibility] of string =
    ('', 'private', 'protected', 'public', 'published', 'automated', '', '');
Var
  V : TPasMemberVisibility;

begin
  Result:=False;
  S:=lowerCase(S);
  For V :=Low(TPasMemberVisibility) to High(TPasMemberVisibility) do
    begin
    Result:=(VNames[V]<>'') and (S=VNames[V]);
    if Result then
      begin
      AVisibility := v;
      Exit;
      end;
    end;
end;

function TPasParser.CheckVisibility(S: String;
  var AVisibility: TPasMemberVisibility): Boolean;

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
  Result:=isVisibility(S,AVisibility);
  if Result then
    begin
    if B then
      case AVisibility of
        visPrivate   : AVisibility:=visStrictPrivate;
        visProtected : AVisibility:=visStrictProtected;
      else
        ParseExc(Format(SParserStrangeVisibility,[S]));
      end
    end
  else if B then
    ParseExc(SParserExpectVisibility);
end;

procedure TPasParser.ProcessMethod(AType: TPasClassType; IsClass : Boolean; AVisibility : TPasMemberVisibility);

var
  Proc: TPasProcedure;
  ProcType: TProcType;
begin
  ProcType:=GetProcTypeFromtoken(CurToken,isClass);
  Proc:=ParseProcedureOrFunctionDecl(AType,ProcType,AVisibility);
  if Proc.Parent is TPasOverloadedProc then
    TPasOverloadedProc(Proc.Parent).Overloads.Add(Proc)
  else
    AType.Members.Add(Proc);
end;

procedure TPasParser.ParseClassFields(AType: TPasClassType;
  const AVisibility: TPasMemberVisibility; IsClassField: Boolean);

Var
  VarList: TFPList;
  Element: TPasElement;
  I : Integer;

begin
  VarList := TFPList.Create;
  try
    ParseInlineVarDecl(AType, VarList, AVisibility, False);
    for i := 0 to VarList.Count - 1 do
      begin
      Element := TPasElement(VarList[i]);
      Element.Visibility := AVisibility;
      if IsClassField and (Element is TPasVariable) then
        TPasVariable(Element).VarModifiers:=TPasVariable(Element).VarModifiers+[vmClass];
      AType.Members.Add(Element);
=======
  if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'DEFAULT') then begin
    if not isArray then ParseExc('The default property must be an array property');
//  what is after DEFAULT token at the end
=======
>>>>>>> graemeg/cpstrnew
    NextToken;
    if CurToken = tkSemicolon then begin
//    ";" then DEFAULT=prop
      TPasProperty(Element).IsDefault := True;
      NextToken;
    end
  end;
  
  while IsCurTokenHint(h) do begin
    Element.Hints:=Element.Hints+[h];
    NextToken;
    // there can be multiple hints, separated by the, i.e.:
    // property Prop: integer read FMyProp write FMyProp; platform; library deprecated;
    if CurToken=tkSemicolon then 
      NextToken;
  end;    

  // property parsing must finish at the LAST Semicolon of the property
  // since we're parsing "one-step" ahead of the semicolon. we must return one-step
  UngetToken; 
end;

// Starts after the "begin" token
procedure TPasParser.ParseProcBeginBlock(Parent: TProcedureBody);
var
  BeginBlock: TPasImplBeginBlock;
  SubBlock: TPasImplElement;
begin
  //writeln('TPasParser.ParseProcBeginBlock ');

  BeginBlock := TPasImplBeginBlock(CreateElement(TPasImplBeginBlock, '', Parent));
  Parent.Body := BeginBlock;
  repeat
    NextToken;
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
  ExpectToken(tkSemicolon);
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
<<<<<<< HEAD
  {$ENDIF}

  function CloseBlock: boolean; // true if parent reached
=======
  //writeln('TPasParser.ParseVarDecl ',CurTokenText);
  NextToken;
  // Writeln(LastVar,': Parsed complex type: ',CurtokenText);
  // NextToken;
  // Writeln(LastVar,': Parsed complex type, next: ',CurtokenText);
  If CurToken=tkEqual then
    begin
    Value := ParseExpression(Parent);
    for i := 0 to List.Count - 1 do
      TPasVariable(List[i]).Value := Value;
    NextToken;
    end;

  if CurToken = tkAbsolute then
>>>>>>> origin/cpstrnew
  begin
    CurBlock:=CurBlock.Parent as TPasImplBlock;
    Result:=CurBlock=Parent;
  end;

<<<<<<< HEAD
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
    CurBlock:=NewBlock;
    if NewImplElement=nil then NewImplElement:=CurBlock;
  end;

var
  Condition: String;
  StartValue: String;
  VarName: String;
  EndValue: String;
  Expr: String;
  SubBlock: TPasImplElement;
  CmdElem: TPasImplElement;
  TypeName: String;
  ForDownTo: Boolean;
  left: TPasExpr;
  right: TPasExpr;
begin
  NewImplElement:=nil;
  CurBlock := Parent;
  while True do
  begin
    NextToken;
    //WriteLn(i,'Token=',CurTokenText);
    case CurToken of
    tkbegin:
      CreateBlock(CurBlock.AddBeginBlock);
    tkrepeat:
      CreateBlock(CurBlock.AddRepeatUntil);
    tkIf:
      begin
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'IF Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddIfElse(Condition));
        ExpectToken(tkthen);
      end;
    tkelse:
      if (CurBlock is TPasImplIfElse) then
      begin
        if TPasImplIfElse(CurBlock).IfBranch=nil then
        begin
          // empty then => add dummy command
          CurBlock.AddCommand('');
        end;
        if TPasImplIfElse(CurBlock).ElseBranch<>nil then
        begin
          // this and the following 3 may solve TPasImplIfElse.AddElement BUG
          // ifs without begin end
          // if .. then
          //  if .. then
          //   else
          // else
          CloseBlock;
          CloseStatement(false);
        end;
      end else if (CurBlock is TPasImplWhileDo) then
      begin
        //if .. then while .. do smt else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplRaise) then
      begin
        //if .. then Raise Exception else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplTryExcept) then
      begin
        CloseBlock;
        CurBlock:=TPasImplTry(CurBlock).AddExceptElse;
      end else
        ParseExc(SParserSyntaxError);
    tkwhile:
      begin
        // while Condition do
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'WHILE Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWhileDo(Condition));
        ExpectToken(tkdo);
      end;
    tkgoto:
      begin
        nexttoken;
        curblock.AddCommand('goto '+curtokenstring);
        expecttoken(tkSemiColon);
      end;
    tkfor:
      begin
        // for VarName := StartValue to EndValue do
        ExpectIdentifier;
        VarName:=CurTokenString;
        ExpectToken(tkAssign);
        StartValue:=ParseExpression(Parent);
        //writeln(i,'FOR Start=',StartValue);
        NextToken;
        if CurToken=tkTo then
          ForDownTo:=false
        else if CurToken=tkdownto then
          ForDownTo:=true
        else
          ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkTo]]));
        EndValue:=ParseExpression(Parent);
        CreateBlock(CurBlock.AddForLoop(VarName,StartValue,EndValue,ForDownTo));
        //WriteLn(i,'FOR "',VarName,'" := ',StartValue,' to ',EndValue,' Token=',CurTokenText);
        ExpectToken(tkdo);
      end;
    tkwith:
      begin
        // with Expr do
        // with Expr, Expr do
        Expr:=ParseExpression(Parent);
        //writeln(i,'WITH Expr="',Expr,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWithDo(Expr));
        repeat
          NextToken;
          if CurToken=tkdo then break;
          if CurToken<>tkComma then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkdo]]));
          Expr:=ParseExpression(Parent);
          //writeln(i,'WITH ...,Expr="',Expr,'" Token=',CurTokenText);
          TPasImplWithDo(CurBlock).AddExpression(Expr);
        until false;
      end;
    tkcase:
      begin
        Expr:=ParseExpression(Parent);
        //writeln(i,'CASE OF Expr="',Expr,'" Token=',CurTokenText);
        ExpectToken(tkof);
        CreateBlock(CurBlock.AddCaseOf(Expr));
        repeat
          NextToken;
          //writeln(i,'CASE OF Token=',CurTokenText);
          case CurToken of
          tkend:
            break; // end without else
          tkelse:
            begin
              // create case-else block
              CurBlock:=TPasImplCaseOf(CurBlock).AddElse;
              break;
            end
          else
            UngetToken;
            // read case values
            repeat
              Expr:=ParseExpression(Parent);
              //writeln(i,'CASE value="',Expr,'" Token=',CurTokenText);
              NextToken;
              if CurToken=tkDotDot then
              begin
                Expr:=Expr+'..'+ParseExpression(Parent);
                NextToken;
              end;
              // do not miss '..'
              if CurBlock is TPasImplCaseStatement then
                TPasImplCaseStatement(CurBlock).Expressions.Add(Expr)
              else
                CurBlock:=TPasImplCaseOf(CurBlock).AddCase(Expr);
              //writeln(i,'CASE after value Token=',CurTokenText);
              if CurToken=tkColon then break;
              if CurToken<>tkComma then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkComma]]));
            until false;
            // read statement
            ParseStatement(CurBlock,SubBlock);
            CloseBlock;
            if CurToken<>tkSemicolon then
            begin
              NextToken;
              if not (CurToken in [tkSemicolon,tkelse,tkend]) then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
              if CurToken<>tkSemicolon then
                UngetToken;
            end;
          end;
        until false;
        if CurToken=tkend then
        begin
          if CloseBlock then break;
          if CloseStatement(false) then break;
        end;
      end;
    tktry:
      CreateBlock(CurBlock.AddTry);
    tkfinally:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplTry then
        begin
          CurBlock:=TPasImplTry(CurBlock).AddFinally;
        end else
          ParseExc(SParserSyntaxError);
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
          CurBlock:=TPasImplTry(CurBlock).AddExcept;
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkon:
      begin
        // in try except:
        // on E: Exception do
        // on Exception do
        if CurBlock is TPasImplTryExcept then
        begin
          VarName:='';
          TypeName:=ParseExpression(Parent);
          //writeln(i,'ON t=',TypeName,' Token=',CurTokenText);
          NextToken;
          if CurToken=tkColon then
          begin
<<<<<<< HEAD
            VarName:=TypeName;
            TypeName:=ParseExpression(Parent);
            //writeln(i,'ON v=',VarName,' t=',TypeName,' Token=',CurTokenText);
          end else
            UngetToken;
          CurBlock:=TPasImplTryExcept(CurBlock).AddExceptOn(VarName,TypeName);
          ExpectToken(tkDo);
        end else
=======
          M := M + ' name ';
          NextToken;
          if (CurToken = tkString) or (CurToken = tkIdentifier) then
            // !!!: Is this really correct for tkString?
            M := M + CurTokenText
          else
            ParseExc(SParserSyntaxError);
          H:=CheckHint(Nil,True);
          If (H<>[]) then
            for i := 0 to List.Count - 1 do
              TPasVariable(List[i]).Hints:=H;

          // ExpectToken(tkSemicolon);
          end
        else if CurToken <> tkSemicolon then
>>>>>>> origin/cpstrnew
          ParseExc(SParserSyntaxError);
      end;
    tkraise:
      CreateBlock(CurBlock.AddRaise);
    tkend:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplBeginBlock then
        begin
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
          ParseExc(SParserSyntaxError);
      end;
    tkSemiColon:
      if CloseStatement(true) then break;
    tkuntil:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplRepeatUntil then
        begin
          Condition:=ParseExpression(Parent);
          TPasImplRepeatUntil(CurBlock).Condition:=Condition;
          //WriteLn(i,'UNTIL Condition="',Condition,'" Token=',CurTokenString);
          if CloseBlock then break;
        end else
          ParseExc(SParserSyntaxError);
      end;
    else
      left:=DoParseExpression(nil);
      case CurToken of
        tkAssign:
        begin
          // assign statement
          NextToken;
          right:=DoParseExpression(nil); // this may solve TPasImplWhileDo.AddElement BUG
          CmdElem:=CurBlock.AddAssign(left, right);
          UngetToken;
        end;
        tkColon:
        begin
          if not (left is TPrimitiveExpr) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
          // label mark. todo: check mark identifier in the list of labels
          CmdElem:=CurBlock.AddLabelMark(TPrimitiveExpr(left).Value);
          left.Free;
        end;
      else
        // simple statement (function call)
        CmdElem:=CurBlock.AddSimple(left);
        UngetToken;
      end;
=======
//  if indexed prop then read the index value
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'INDEX') then begin
//    read 'index' access modifier
      TPasProperty(Element).IndexValue := ParseExpression(Element,ek_PropertyIndex);
    end else
//    not indexed prop will be recheck for another token
      UngetToken;
>>>>>>> graemeg/cpstrnew

      if not (CmdElem is TPasImplLabelMark) then
        if NewImplElement=nil then NewImplElement:=CmdElem;
    end;
  end;
<<<<<<< HEAD
end;

procedure TPasParser.ParseLabels(AParent: TPasElement);
var
  Labels: TPasLabels;
begin
  Labels:=TPasLabels(CreateElement(TPasLabels, '', AParent));
  repeat
    Labels.Labels.Add(ExpectIdentifier);
    NextToken;
    if not (CurToken in [tkSemicolon, tkComma]) then
      ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
  until CurToken=tkSemicolon;
end;

=======
  if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'DEFAULT') then begin
    if not isArray then ParseExc('The default property must be an array property');
//  what is after DEFAULT token at the end
=======

// if the accessors list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'read' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'READ') then
      TPasProperty(Element).ReadAccessorName := GetAccessorName
    else
//    not read accessor will be recheck for another token
      UngetToken;

    NextToken;
  end;

// if the accessors list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'write' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'WRITE') then
      TPasProperty(Element).WriteAccessorName := GetAccessorName
    else
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'IMPLEMENTS') then
      TPasProperty(Element).ImplementsName := GetAccessorName
    else
//    not write accessor will be recheck for another token
      UngetToken;

>>>>>>> graemeg/cpstrnew
    NextToken;
    if CurToken = tkSemicolon then begin
//    ";" then DEFAULT=prop
      TPasProperty(Element).IsDefault := True;
      NextToken;
    end
  end;
<<<<<<< HEAD
  
  while IsCurTokenHint(h) do begin
    Element.Hints:=Element.Hints+[h];
    NextToken;
    // there can be multiple hints, separated by the, i.e.:
    // property Prop: integer read FMyProp write FMyProp; platform; library deprecated;
    if CurToken=tkSemicolon then 
      NextToken;
  end;    

  // property parsing must finish at the LAST Semicolon of the property
  // since we're parsing "one-step" ahead of the semicolon. we must return one-step
  UngetToken; 
end;

// Starts after the "begin" token
procedure TPasParser.ParseProcBeginBlock(Parent: TProcedureBody);
var
  BeginBlock: TPasImplBeginBlock;
  SubBlock: TPasImplElement;
begin
  //writeln('TPasParser.ParseProcBeginBlock ');
=======

// if the specifiers list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'stored' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'STORED') then begin
      NextToken;
      if CurToken = tkTrue then
        TPasProperty(Element).StoredAccessorName := 'True'
      else if CurToken = tkFalse then
        TPasProperty(Element).StoredAccessorName := 'False'
      else if CurToken = tkIdentifier then
        TPasProperty(Element).StoredAccessorName := CurTokenString
      else
        ParseExc(SParserSyntaxError);
    end else
//    not stored accessor will be recheck for another token
      UngetToken;
>>>>>>> graemeg/cpstrnew

  BeginBlock := TPasImplBeginBlock(CreateElement(TPasImplBeginBlock, '', Parent));
  Parent.Body := BeginBlock;
  repeat
    NextToken;
<<<<<<< HEAD
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
  ExpectToken(tkSemicolon);
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
=======
  end;

// if the specifiers list is not finished
  if (CurToken <> tkSemicolon) and (CurToken = tkIdentifier) then begin
    us:=UpperCase(CurTokenText);
    if (us = 'DEFAULT') then begin
      if isArray then ParseExc('Array properties cannot have default value');
//    read 'default' value modifier -> ParseExpression(DEFAULT <value>)
      TPasProperty(Element).DefaultValue := ParseExpression(Element);
      NextToken;
    end else if (us = 'NODEFAULT') then begin
//    read 'nodefault' modifier
      TPasProperty(Element).IsNodefault:=true;
    end else
//    not "default <value>" prop will be recheck for another token
      UngetToken;
    NextToken;
>>>>>>> graemeg/cpstrnew
  end;
  {$ENDIF}

  function CloseBlock: boolean; // true if parent reached
  begin
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

<<<<<<< HEAD
  procedure CreateBlock(NewBlock: TPasImplBlock);
  begin
    CurBlock:=NewBlock;
    if NewImplElement=nil then NewImplElement:=CurBlock;
  end;

var
  Condition: String;
  StartValue: String;
  VarName: String;
  EndValue: String;
  Expr: String;
  SubBlock: TPasImplElement;
  CmdElem: TPasImplElement;
  TypeName: String;
  ForDownTo: Boolean;
  left: TPasExpr;
  right: TPasExpr;
begin
  NewImplElement:=nil;
  CurBlock := Parent;
  while True do
  begin
    NextToken;
    //WriteLn(i,'Token=',CurTokenText);
    case CurToken of
    tkbegin:
      CreateBlock(CurBlock.AddBeginBlock);
    tkrepeat:
      CreateBlock(CurBlock.AddRepeatUntil);
    tkIf:
      begin
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'IF Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddIfElse(Condition));
        ExpectToken(tkthen);
      end;
    tkelse:
      if (CurBlock is TPasImplIfElse) then
      begin
        if TPasImplIfElse(CurBlock).IfBranch=nil then
        begin
          // empty then => add dummy command
          CurBlock.AddCommand('');
        end;
        if TPasImplIfElse(CurBlock).ElseBranch<>nil then
        begin
          // this and the following 3 may solve TPasImplIfElse.AddElement BUG
          // ifs without begin end
          // if .. then
          //  if .. then
          //   else
          // else
          CloseBlock;
          CloseStatement(false);
        end;
      end else if (CurBlock is TPasImplWhileDo) then
      begin
        //if .. then while .. do smt else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplRaise) then
      begin
        //if .. then Raise Exception else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplTryExcept) then
      begin
<<<<<<< HEAD
        CloseBlock;
        CurBlock:=TPasImplTry(CurBlock).AddExceptElse;
      end else
        ParseExc(SParserSyntaxError);
    tkwhile:
      begin
        // while Condition do
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'WHILE Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWhileDo(Condition));
        ExpectToken(tkdo);
      end;
    tkgoto:
      begin
        nexttoken;
        curblock.AddCommand('goto '+curtokenstring);
        expecttoken(tkSemiColon);
=======
        Value := ParseExpression(Parent);
      end else
        UngetToken;
    end;

    for i := 0 to ArgNames.Count - 1 do
    begin
      Arg := TPasArgument(CreateElement(TPasArgument, ArgNames[i], Parent));
      Arg.Access := Access;
      Arg.ArgType := ArgType;
      if (i > 0) and Assigned(ArgType) then
        ArgType.AddRef;
      Arg.Value := Value;
      Args.Add(Arg);
    end;

    ArgNames.Free;
    NextToken;
    if CurToken = EndToken then
      break;
  end;
end;

// Next token is expected to be a "(", ";" or for a function ":". The caller
// will get the token after the final ";" as next token.
procedure TPasParser.ParseProcedureOrFunctionHeader(Parent: TPasElement;
  Element: TPasProcedureType; ProcType: TProcType; OfObjectPossible: Boolean);

procedure ConsumeSemi;
begin
  NextToken;
  if (CurToken <> tksemicolon) and IsCurTokenHint then
    ungettoken;
end;

Var
  Tok : String;
  i: Integer;
  Proc: TPasProcedure;
  ahint : TPasMemberHint;
begin
  NextToken;
  case ProcType of
    ptFunction,ptClassFunction:
      begin
        if CurToken = tkBraceOpen then
        begin
          NextToken;
          if (CurToken <> tkBraceClose) then
            begin
              UngetToken;
              ParseArgList(Parent, Element.Args, tkBraceClose);
            end;
          ExpectToken(tkColon);
        end else if CurToken <> tkColon then
          ParseExc(SParserExpectedLBracketColon);
        if Assigned(Element) then        // !!!
          TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
        else
          ParseType(nil);
      end;

    ptProcedure,ptConstructor,ptDestructor,ptClassProcedure:
      begin
        if CurToken = tkBraceOpen then
        begin
          NextToken;
          if (CurToken = tkBraceClose) then
          else
            begin
              UngetToken;
              ParseArgList(Element, Element.Args, tkBraceClose);
            end
        end else if (CurToken = tkSemicolon)
          or (OfObjectPossible and (CurToken in [tkOf,tkEqual]))
        then
          UngetToken
        else
          ParseExc(SParserExpectedLBracketSemicolon);
>>>>>>> origin/cpstrnew
      end;
    tkfor:
      begin
<<<<<<< HEAD
        // for VarName := StartValue to EndValue do
        ExpectIdentifier;
        VarName:=CurTokenString;
        ExpectToken(tkAssign);
        StartValue:=ParseExpression(Parent);
        //writeln(i,'FOR Start=',StartValue);
        NextToken;
        if CurToken=tkTo then
          ForDownTo:=false
        else if CurToken=tkdownto then
          ForDownTo:=true
        else
          ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkTo]]));
        EndValue:=ParseExpression(Parent);
        CreateBlock(CurBlock.AddForLoop(VarName,StartValue,EndValue,ForDownTo));
        //WriteLn(i,'FOR "',VarName,'" := ',StartValue,' to ',EndValue,' Token=',CurTokenText);
        ExpectToken(tkdo);
      end;
    tkwith:
      begin
        // with Expr do
        // with Expr, Expr do
        Expr:=ParseExpression(Parent);
        //writeln(i,'WITH Expr="',Expr,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWithDo(Expr));
        repeat
          NextToken;
          if CurToken=tkdo then break;
          if CurToken<>tkComma then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkdo]]));
          Expr:=ParseExpression(Parent);
          //writeln(i,'WITH ...,Expr="',Expr,'" Token=',CurTokenText);
          TPasImplWithDo(CurBlock).AddExpression(Expr);
        until false;
      end;
    tkcase:
      begin
        Expr:=ParseExpression(Parent);
        //writeln(i,'CASE OF Expr="',Expr,'" Token=',CurTokenText);
        ExpectToken(tkof);
        CreateBlock(CurBlock.AddCaseOf(Expr));
        repeat
          NextToken;
          //writeln(i,'CASE OF Token=',CurTokenText);
          case CurToken of
          tkend:
            break; // end without else
          tkelse:
=======
        ParseArgList(Element, Element.Args, tkBraceClose);
        NextToken;
        if (CurToken=tkIdentifier) then
        begin
          TPasFunctionType(Element).ResultEl.Name := CurTokenName;
          ExpectToken(tkColon);
        end
        else if (CurToken=tkColon) then
          TPasFunctionType(Element).ResultEl.Name := 'Result'
        else
          ParseExc(SParserExpectedColonID);
        if Assigned(Element) then        // !!!
          TPasFunctionType(Element).ResultEl.ResultType := ParseType(Parent)
        else
          ParseType(nil);
      end;
  end;

  NextToken;
  if OfObjectPossible and (CurToken = tkOf) then
  begin
    ExpectToken(tkObject);
    Element.IsOfObject := True;
  end else
    UngetToken;

  NextToken;
  if CurToken = tkEqual then
  begin
    // for example: const p: procedure = nil;
    UngetToken;
    exit;
  end else
    UngetToken;

  ExpectToken(tkSemicolon);
  while True do
    begin
    // CheckHint(Element,False);
    NextToken;
    if (CurToken = tkIdentifier) or (CurToken=tklibrary) then // library is a token and a directive.
      begin
      Tok:=UpperCase(CurTokenString);
      If (Tok='CDECL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccCDecl;
        ExpectToken(tkSemicolon);
        end
      else If (Tok='EXPORT') then
        begin
        TPasProcedure(Parent).AddModifier(pmExported);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='PASCAL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccPascal;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='STDCALL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccStdCall;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='OLDFPCCALL') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccOldFPCCall;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='EXTDECL') then
        begin
        // extdecl is a common macro for external functions
        TPasProcedure(Parent).AddModifier(pmExternal);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='EXTERNAL') then
        begin
        TPasProcedure(Parent).AddModifier(pmExternal);
        NextToken;
        if CurToken in [tkString,tkIdentifier] then
        begin
          NextToken;
          Tok:=UpperCase(CurTokenString);
          if Tok='NAME' then
          begin
            NextToken;
            if not (CurToken in [tkString,tkIdentifier]) then
              ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkString]]));
          end;
        end else
          UngetToken;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='REGISTER') then
        begin
        TPasProcedure(Parent).CallingConvention:=ccRegister;
        ExpectToken(tkSemicolon);
        end
      else if (Tok='COMPILERPROC') then
        begin
        TPasProcedure(Parent).AddModifier(pmCompilerProc);
        ExpectToken(tkSemicolon);
        end
      else if (Tok='VARARGS') then
        begin
        TPasProcedure(Parent).AddModifier(pmVarArgs);
        ExpectToken(tkSemicolon);
        end
      else if IsCurTokenHint(ahint) then  // deprecated,platform,experimental,library, unimplemented etc
        begin
        element.hints:=element.hints+[ahint];
        consumesemi;
        end
      else if (tok='OVERLOAD') then
        begin
        TPasProcedure(Parent).AddModifier(pmOverload);
        ExpectToken(tkSemicolon);
        end
      else if (tok='INLINE') then
        begin
        TPasProcedure(Parent).AddModifier(pmInline);
        ExpectToken(tkSemicolon);
        end
      else if (tok='ASSEMBLER') then
        begin
        TPasProcedure(Parent).AddModifier(pmAssembler);
        ExpectToken(tkSemicolon);
        end
      else if (tok = 'EXTERNAL') then
        repeat
          NextToken;
        until CurToken = tkSemicolon
      else if (tok = 'PUBLIC') then
        begin
          NextToken;
          { Should be token Name,
            if not we're in a class and the public section starts }
          If (Uppercase(CurTokenString)<>'NAME') then
>>>>>>> origin/cpstrnew
            begin
              // create case-else block
              CurBlock:=TPasImplCaseOf(CurBlock).AddElse;
              break;
            end
          else
<<<<<<< HEAD
            UngetToken;
            // read case values
            repeat
              Expr:=ParseExpression(Parent);
              //writeln(i,'CASE value="',Expr,'" Token=',CurTokenText);
              NextToken;
              if CurToken=tkDotDot then
              begin
                Expr:=Expr+'..'+ParseExpression(Parent);
                NextToken;
              end;
              // do not miss '..'
              if CurBlock is TPasImplCaseStatement then
                TPasImplCaseStatement(CurBlock).Expressions.Add(Expr)
              else
                CurBlock:=TPasImplCaseOf(CurBlock).AddCase(Expr);
              //writeln(i,'CASE after value Token=',CurTokenText);
              if CurToken=tkColon then break;
              if CurToken<>tkComma then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkComma]]));
            until false;
            // read statement
            ParseStatement(CurBlock,SubBlock);
            CloseBlock;
            if CurToken<>tkSemicolon then
            begin
              NextToken;
              if not (CurToken in [tkSemicolon,tkelse,tkend]) then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
              if CurToken<>tkSemicolon then
                UngetToken;
            end;
          end;
        until false;
        if CurToken=tkend then
        begin
          if CloseBlock then break;
          if CloseStatement(false) then break;
        end;
      end;
    tktry:
      CreateBlock(CurBlock.AddTry);
    tkfinally:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplTry then
        begin
          CurBlock:=TPasImplTry(CurBlock).AddFinally;
        end else
          ParseExc(SParserSyntaxError);
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
          CurBlock:=TPasImplTry(CurBlock).AddExcept;
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkon:
=======
            begin
            NextToken;  // Should be export name string.
            ExpectToken(tkSemicolon);
            end;
        end
      else if (tok = 'FORWARD') then
        begin
          if (Parent.Parent is TInterfaceSection) then
            begin
            UngetToken;
            break;
            end;
          if Parent is TPasProcedure then
            TPasProcedure(Parent).AddModifier(pmForward);
          ExpectToken(tkSemicolon);
        end
      else
        begin
        UnGetToken;
        Break;
        end
      end
    else if (CurToken = tkInline) then
      begin
      if Parent is TPasProcedure then
        TPasProcedure(Parent).AddModifier(pmInline);
      ExpectToken(tkSemicolon);
      end
    else if (CurToken = tkSquaredBraceOpen) then
      begin
      repeat
        NextToken
      until CurToken = tkSquaredBraceClose;
      ExpectToken(tkSemicolon);
      end
    else
>>>>>>> origin/cpstrnew
      begin
        // in try except:
        // on E: Exception do
        // on Exception do
        if CurBlock is TPasImplTryExcept then
        begin
          VarName:='';
          TypeName:=ParseExpression(Parent);
          //writeln(i,'ON t=',TypeName,' Token=',CurTokenText);
          NextToken;
          if CurToken=tkColon then
          begin
            VarName:=TypeName;
            TypeName:=ParseExpression(Parent);
            //writeln(i,'ON v=',VarName,' t=',TypeName,' Token=',CurTokenText);
          end else
            UngetToken;
          CurBlock:=TPasImplTryExcept(CurBlock).AddExceptOn(VarName,TypeName);
          ExpectToken(tkDo);
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkraise:
      CreateBlock(CurBlock.AddRaise);
    tkend:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplBeginBlock then
        begin
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
          ParseExc(SParserSyntaxError);
      end;
<<<<<<< HEAD
    tkSemiColon:
      if CloseStatement(true) then break;
    tkuntil:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplRepeatUntil then
        begin
          Condition:=ParseExpression(Parent);
          TPasImplRepeatUntil(CurBlock).Condition:=Condition;
          //WriteLn(i,'UNTIL Condition="',Condition,'" Token=',CurTokenString);
          if CloseBlock then break;
        end else
          ParseExc(SParserSyntaxError);
      end;
    else
      left:=DoParseExpression(nil);
      case CurToken of
        tkAssign:
        begin
          // assign statement
          NextToken;
          right:=DoParseExpression(nil); // this may solve TPasImplWhileDo.AddElement BUG
          CmdElem:=CurBlock.AddAssign(left, right);
          UngetToken;
        end;
        tkColon:
        begin
          if not (left is TPrimitiveExpr) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
          // label mark. todo: check mark identifier in the list of labels
          CmdElem:=CurBlock.AddLabelMark(TPrimitiveExpr(left).Value);
          left.Free;
        end;
=======
    end;
  if (ProcType = ptOperator) and (Parent is TPasProcedure) then
  begin
    Proc:=TPasProcedure(Parent);
    Proc.Name := Proc.Name + '(';
    for i := 0 to Proc.ProcType.Args.Count - 1 do
    begin
      if i > 0 then
        Proc.Name := Proc.Name + ', ';
      Proc.Name := Proc.Name +
        TPasArgument(Proc.ProcType.Args[i]).ArgType.Name;
    end;
    Proc.Name := Proc.Name + '): ' +
      TPasFunctionType(Proc.ProcType).ResultEl.ResultType.Name;
  end;

  if (Parent is TPasProcedure)
  and (not TPasProcedure(Parent).IsForward)
  and (not TPasProcedure(Parent).IsExternal)
  and ((Parent.Parent is TImplementationSection)
     or (Parent.Parent is TProcedureBody))
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


procedure TPasParser.ParseProperty(Element:TPasElement);

var
  isArray : Boolean;

  procedure MaybeReadFullyQualifiedIdentifier(Var r : String);

  begin
    while True do
      begin
      NextToken;
      if CurToken = tkDot then
        begin
        ExpectIdentifier;
        R:=R + '.' + CurTokenString;
        end
>>>>>>> origin/cpstrnew
      else
        // simple statement (function call)
        CmdElem:=CurBlock.AddSimple(left);
        UngetToken;
      end;

<<<<<<< HEAD
      if not (CmdElem is TPasImplLabelMark) then
        if NewImplElement=nil then NewImplElement:=CmdElem;
    end;
=======
  function GetAccessorName: String;
  begin
    ExpectIdentifier;
    Result := CurTokenString;
    MaybeReadFullyQualifiedIdentifier(Result);
    if CurToken = tkSquaredBraceOpen then begin
      Result := Result + '[';
      NextToken;
      if CurToken in [tkIdentifier, tkNumber] then begin
    Result := Result + CurTokenString;
      end;
      ExpectToken(tkSquaredBraceClose);
      Result := Result + ']';
    end else
      UngetToken;
    //MaybeReadFullyQualifiedIdentifier(Result);
    //writeln(Result);
>>>>>>> origin/cpstrnew
  end;
end;

<<<<<<< HEAD
procedure TPasParser.ParseLabels(AParent: TPasElement);
var
  Labels: TPasLabels;
begin
  Labels:=TPasLabels(CreateElement(TPasLabels, '', AParent));
  repeat
    Labels.Labels.Add(ExpectIdentifier);
    NextToken;
    if not (CurToken in [tkSemicolon, tkComma]) then
      ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
  until CurToken=tkSemicolon;
end;
=======
var
  us  : String; 
  h   : TPasMemberHint;
begin
  isArray:=False;
  NextToken;
// if array prop then parse [ arg1:type1;... ]

  if CurToken = tkSquaredBraceOpen then begin
    isArray:=True;
  // !!!: Parse array properties correctly
    ParseArgList(Element, TPasProperty(Element).Args, tkSquaredBraceClose);
    NextToken;
  end;

  if CurToken = tkColon then begin
// if ":prop_data_type" if supplied then read it
  // read property type
    TPasProperty(Element).VarType := ParseType(Element);
    NextToken;
  end;

  if CurToken <> tkSemicolon then begin
//  if indexed prop then read the index value
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'INDEX') then begin
//    read 'index' access modifier
      TPasProperty(Element).IndexValue := ParseExpression(Element,ek_PropertyIndex);
    end else
//    not indexed prop will be recheck for another token
      UngetToken;
>>>>>>> origin/cpstrnew

>>>>>>> graemeg/cpstrnew
=======
  if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'DEFAULT') then begin
    if not isArray then ParseExc('The default property must be an array property');
//  what is after DEFAULT token at the end
    NextToken;
    if CurToken = tkSemicolon then begin
//    ";" then DEFAULT=prop
      TPasProperty(Element).IsDefault := True;
      NextToken;
    end
  end;
<<<<<<< HEAD
  
  while IsCurTokenHint(h) do begin
    Element.Hints:=Element.Hints+[h];
    NextToken;
    // there can be multiple hints, separated by the, i.e.:
    // property Prop: integer read FMyProp write FMyProp; platform; library deprecated;
    if CurToken=tkSemicolon then 
      NextToken;
  end;    

  // property parsing must finish at the LAST Semicolon of the property
  // since we're parsing "one-step" ahead of the semicolon. we must return one-step
  UngetToken; 
end;
=======

// if the accessors list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'read' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'READ') then
      TPasProperty(Element).ReadAccessorName := GetAccessorName
    else
//    not read accessor will be recheck for another token
      UngetToken;

    NextToken;
  end;

// if the accessors list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'write' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'WRITE') then
      TPasProperty(Element).WriteAccessorName := GetAccessorName
    else
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'IMPLEMENTS') then
      TPasProperty(Element).ImplementsName := GetAccessorName
    else
//    not write accessor will be recheck for another token
      UngetToken;

    NextToken;
  end;

// if the specifiers list is not finished
  if CurToken <> tkSemicolon then begin
    // read 'stored' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'STORED') then begin
      NextToken;
      if CurToken = tkTrue then
        TPasProperty(Element).StoredAccessorName := 'True'
      else if CurToken = tkFalse then
        TPasProperty(Element).StoredAccessorName := 'False'
      else if CurToken = tkIdentifier then
        TPasProperty(Element).StoredAccessorName := CurTokenString
      else
        ParseExc(SParserSyntaxError);
    end else
//    not stored accessor will be recheck for another token
      UngetToken;
>>>>>>> origin/cpstrnew

// Starts after the "begin" token
procedure TPasParser.ParseProcBeginBlock(Parent: TProcedureBody);
var
  BeginBlock: TPasImplBeginBlock;
  SubBlock: TPasImplElement;
begin
  //writeln('TPasParser.ParseProcBeginBlock ');

  BeginBlock := TPasImplBeginBlock(CreateElement(TPasImplBeginBlock, '', Parent));
  Parent.Body := BeginBlock;
  repeat
    NextToken;
<<<<<<< HEAD
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
  ExpectToken(tkSemicolon);
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
=======
  end;

// if the specifiers list is not finished
  if (CurToken <> tkSemicolon) and (CurToken = tkIdentifier) then begin
    us:=UpperCase(CurTokenText);
    if (us = 'DEFAULT') then begin
      if isArray then ParseExc('Array properties cannot have default value');
//    read 'default' value modifier -> ParseExpression(DEFAULT <value>)
      TPasProperty(Element).DefaultValue := ParseExpression(Element);
      NextToken;
    end else if (us = 'NODEFAULT') then begin
//    read 'nodefault' modifier
      TPasProperty(Element).IsNodefault:=true;
    end else
//    not "default <value>" prop will be recheck for another token
      UngetToken;
    NextToken;
>>>>>>> origin/cpstrnew
  end;
  {$ENDIF}

  function CloseBlock: boolean; // true if parent reached
  begin
    CurBlock:=CurBlock.Parent as TPasImplBlock;
    Result:=CurBlock=Parent;
  end;

<<<<<<< HEAD
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
    CurBlock:=NewBlock;
    if NewImplElement=nil then NewImplElement:=CurBlock;
  end;

var
  Condition: String;
  StartValue: String;
  VarName: String;
  EndValue: String;
  Expr: String;
  SubBlock: TPasImplElement;
  CmdElem: TPasImplElement;
  TypeName: String;
  ForDownTo: Boolean;
  left: TPasExpr;
  right: TPasExpr;
begin
  NewImplElement:=nil;
  CurBlock := Parent;
  while True do
  begin
    NextToken;
    //WriteLn(i,'Token=',CurTokenText);
    case CurToken of
    tkbegin:
      CreateBlock(CurBlock.AddBeginBlock);
    tkrepeat:
      CreateBlock(CurBlock.AddRepeatUntil);
    tkIf:
      begin
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'IF Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddIfElse(Condition));
        ExpectToken(tkthen);
      end;
    tkelse:
      if (CurBlock is TPasImplIfElse) then
      begin
        if TPasImplIfElse(CurBlock).IfBranch=nil then
        begin
          // empty then => add dummy command
          CurBlock.AddCommand('');
        end;
        if TPasImplIfElse(CurBlock).ElseBranch<>nil then
        begin
          // this and the following 3 may solve TPasImplIfElse.AddElement BUG
          // ifs without begin end
          // if .. then
          //  if .. then
          //   else
          // else
          CloseBlock;
          CloseStatement(false);
        end;
      end else if (CurBlock is TPasImplWhileDo) then
      begin
        //if .. then while .. do smt else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplRaise) then
      begin
        //if .. then Raise Exception else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplTryExcept) then
      begin
        CloseBlock;
        CurBlock:=TPasImplTry(CurBlock).AddExceptElse;
      end else
        ParseExc(SParserSyntaxError);
    tkwhile:
      begin
        // while Condition do
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'WHILE Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWhileDo(Condition));
        ExpectToken(tkdo);
      end;
    tkgoto:
      begin
        nexttoken;
        curblock.AddCommand('goto '+curtokenstring);
        expecttoken(tkSemiColon);
      end;
    tkfor:
      begin
        // for VarName := StartValue to EndValue do
        ExpectIdentifier;
        VarName:=CurTokenString;
        ExpectToken(tkAssign);
        StartValue:=ParseExpression(Parent);
        //writeln(i,'FOR Start=',StartValue);
        NextToken;
        if CurToken=tkTo then
          ForDownTo:=false
        else if CurToken=tkdownto then
          ForDownTo:=true
        else
          ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkTo]]));
        EndValue:=ParseExpression(Parent);
        CreateBlock(CurBlock.AddForLoop(VarName,StartValue,EndValue,ForDownTo));
        //WriteLn(i,'FOR "',VarName,'" := ',StartValue,' to ',EndValue,' Token=',CurTokenText);
        ExpectToken(tkdo);
      end;
<<<<<<< HEAD
    tkwith:
=======
  end;

  NextToken;
  if OfObjectPossible and (CurToken = tkOf) then
  begin
    ExpectToken(tkObject);
    Element.IsOfObject := True;
  end else
    UngetToken;

  NextToken;
  if CurToken = tkEqual then
  begin
    // for example: const p: procedure = nil;
    UngetToken;
    exit;
  end else
    UngetToken;

  ExpectToken(tkSemicolon);

  while True do
    begin
    // CheckHint(Element,False);
    NextToken;
    if (CurToken = tkIdentifier) then
>>>>>>> graemeg/fixes_2_2
      begin
        // with Expr do
        // with Expr, Expr do
        Expr:=ParseExpression(Parent);
        //writeln(i,'WITH Expr="',Expr,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWithDo(Expr));
        repeat
          NextToken;
          if CurToken=tkdo then break;
          if CurToken<>tkComma then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkdo]]));
          Expr:=ParseExpression(Parent);
          //writeln(i,'WITH ...,Expr="',Expr,'" Token=',CurTokenText);
          TPasImplWithDo(CurBlock).AddExpression(Expr);
        until false;
      end;
    tkcase:
      begin
        Expr:=ParseExpression(Parent);
        //writeln(i,'CASE OF Expr="',Expr,'" Token=',CurTokenText);
        ExpectToken(tkof);
        CreateBlock(CurBlock.AddCaseOf(Expr));
        repeat
          NextToken;
          //writeln(i,'CASE OF Token=',CurTokenText);
          case CurToken of
          tkend:
            break; // end without else
          tkelse:
            begin
              // create case-else block
              CurBlock:=TPasImplCaseOf(CurBlock).AddElse;
              break;
            end
          else
            UngetToken;
            // read case values
            repeat
              Expr:=ParseExpression(Parent);
              //writeln(i,'CASE value="',Expr,'" Token=',CurTokenText);
              NextToken;
              if CurToken=tkDotDot then
              begin
                Expr:=Expr+'..'+ParseExpression(Parent);
                NextToken;
              end;
              // do not miss '..'
              if CurBlock is TPasImplCaseStatement then
                TPasImplCaseStatement(CurBlock).Expressions.Add(Expr)
              else
                CurBlock:=TPasImplCaseOf(CurBlock).AddCase(Expr);
              //writeln(i,'CASE after value Token=',CurTokenText);
              if CurToken=tkColon then break;
              if CurToken<>tkComma then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkComma]]));
            until false;
            // read statement
            ParseStatement(CurBlock,SubBlock);
            CloseBlock;
            if CurToken<>tkSemicolon then
            begin
              NextToken;
              if not (CurToken in [tkSemicolon,tkelse,tkend]) then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
              if CurToken<>tkSemicolon then
                UngetToken;
            end;
          end;
        until false;
        if CurToken=tkend then
        begin
          if CloseBlock then break;
          if CloseStatement(false) then break;
        end;
      end;
    tktry:
      CreateBlock(CurBlock.AddTry);
    tkfinally:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplTry then
        begin
          CurBlock:=TPasImplTry(CurBlock).AddFinally;
        end else
          ParseExc(SParserSyntaxError);
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
<<<<<<< HEAD
          //writeln(i,'EXCEPT');
          CurBlock:=TPasImplTry(CurBlock).AddExcept;
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkon:
      begin
        // in try except:
        // on E: Exception do
        // on Exception do
        if CurBlock is TPasImplTryExcept then
=======
{       El['calling-conv'] := 'deprecated';}
        element.hints:=element.hints+[hDeprecated];
        ExpectToken(tkSemicolon);
        end
      else if (tok='PLATFORM') then  
        begin
{       El['calling-conv'] := 'deprecated';}
        element.hints:=element.hints+[hPlatform];
        ExpectToken(tkSemicolon);
        end
      else if (tok='LIBRARY') then  
        begin
{       El['calling-conv'] := 'deprecated';}
<<<<<<< HEAD
=======
        element.hints:=element.hints+[hDeprecated];
        ExpectToken(tkSemicolon);
        end
      else if (tok='PLATFORM') then  
        begin
{       El['calling-conv'] := 'deprecated';}
        element.hints:=element.hints+[hPlatform];
        ExpectToken(tkSemicolon);
        end
      else if (tok='LIBRARY') then  
        begin
{       El['calling-conv'] := 'deprecated';}
>>>>>>> origin/fixes_2_2
        element.hints:=element.hints+[hLibrary];
        ExpectToken(tkSemicolon);
        end
      else if (tok='OVERLOAD') then
>>>>>>> graemeg/fixes_2_2
        begin
          VarName:='';
          TypeName:=ParseExpression(Parent);
          //writeln(i,'ON t=',TypeName,' Token=',CurTokenText);
          NextToken;
          if CurToken=tkColon then
          begin
            VarName:=TypeName;
            TypeName:=ParseExpression(Parent);
            //writeln(i,'ON v=',VarName,' t=',TypeName,' Token=',CurTokenText);
          end else
            UngetToken;
          CurBlock:=TPasImplTryExcept(CurBlock).AddExceptOn(VarName,TypeName);
          ExpectToken(tkDo);
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkraise:
      CreateBlock(CurBlock.AddRaise);
    tkend:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplBeginBlock then
        begin
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
          ParseExc(SParserSyntaxError);
      end;
    tkSemiColon:
      if CloseStatement(true) then break;
    tkuntil:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplRepeatUntil then
        begin
          Condition:=ParseExpression(Parent);
          TPasImplRepeatUntil(CurBlock).Condition:=Condition;
          //WriteLn(i,'UNTIL Condition="',Condition,'" Token=',CurTokenString);
          if CloseBlock then break;
        end else
          ParseExc(SParserSyntaxError);
      end;
<<<<<<< HEAD
    else
      left:=DoParseExpression(nil);
      case CurToken of
        tkAssign:
        begin
          // assign statement
          NextToken;
          right:=DoParseExpression(nil); // this may solve TPasImplWhileDo.AddElement BUG
          CmdElem:=CurBlock.AddAssign(left, right);
          UngetToken;
        end;
        tkColon:
        begin
          if not (left is TPrimitiveExpr) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
          // label mark. todo: check mark identifier in the list of labels
          CmdElem:=CurBlock.AddLabelMark(TPrimitiveExpr(left).Value);
          left.Free;
        end;
      else
        // simple statement (function call)
        CmdElem:=CurBlock.AddSimple(left);
        UngetToken;
      end;

      if not (CmdElem is TPasImplLabelMark) then
        if NewImplElement=nil then NewImplElement:=CmdElem;
    end;
=======
    end;
     
end;


procedure TPasParser.ParseProperty(Element:TPasElement);

  procedure MaybeReadFullyQualifiedIdentifier(Var r : String);
  
  begin
    while True do 
      begin
      NextToken;
      if CurToken = tkDot then 
        begin
        ExpectIdentifier;
        R:=R + '.' + CurTokenString;
        end 
      else
        break;
      end;
  end;

  function GetAccessorName: String;
  begin
    ExpectIdentifier;
    Result := CurTokenString;
    MaybeReadFullyQualifiedIdentifier(Result);
    if CurToken = tkSquaredBraceOpen then begin
      Result := Result + '[';
      NextToken;
      if CurToken in [tkIdentifier, tkNumber] then begin
	Result := Result + CurTokenString;
      end;
      ExpectToken(tkSquaredBraceClose);
      Result := Result + ']';
    end else 
      UngetToken;
    //MaybeReadFullyQualifiedIdentifier(Result);  
    //writeln(Result);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

procedure TPasParser.ParseLabels(AParent: TPasElement);
var
  Labels: TPasLabels;
begin
  Labels:=TPasLabels(CreateElement(TPasLabels, '', AParent));
  repeat
    Labels.Labels.Add(ExpectIdentifier);
    NextToken;
    if not (CurToken in [tkSemicolon, tkComma]) then
      ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
  until CurToken=tkSemicolon;
end;

>>>>>>> graemeg/cpstrnew
=======
  if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'DEFAULT') then begin
    if not isArray then ParseExc('The default property must be an array property');
//  what is after DEFAULT token at the end
    NextToken;
    if CurToken = tkSemicolon then begin
//    ";" then DEFAULT=prop
      TPasProperty(Element).IsDefault := True;
      NextToken;
    end
  end;
  
<<<<<<< HEAD
  while IsCurTokenHint(h) do begin
    Element.Hints:=Element.Hints+[h];
=======
// if the accessors list is not finished  
  if CurToken <> tkSemicolon then begin
    // read 'read' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'READ') then
      TPasProperty(Element).ReadAccessorName := GetAccessorName
    else
//    not read accessor will be recheck for another token
      UngetToken;
    
    NextToken;
  end;
  
// if the accessors list is not finished    
  if CurToken <> tkSemicolon then begin
    // read 'write' access modifier
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'WRITE') then
      TPasProperty(Element).WriteAccessorName := GetAccessorName
    else
    if (CurToken = tkIdentifier) and (UpperCase(CurTokenText) = 'IMPLEMENTS') then
      TPasProperty(Element).ImplementsName := GetAccessorName
    else
//    not write accessor will be recheck for another token
      UngetToken;
    
>>>>>>> graemeg/fixes_2_2
    NextToken;
    // there can be multiple hints, separated by the, i.e.:
    // property Prop: integer read FMyProp write FMyProp; platform; library deprecated;
    if CurToken=tkSemicolon then 
      NextToken;
  end;    

  // property parsing must finish at the LAST Semicolon of the property
  // since we're parsing "one-step" ahead of the semicolon. we must return one-step
  UngetToken; 
end;

// Starts after the "begin" token
procedure TPasParser.ParseProcBeginBlock(Parent: TProcedureBody);
var
  BeginBlock: TPasImplBeginBlock;
  SubBlock: TPasImplElement;
begin
  //writeln('TPasParser.ParseProcBeginBlock ');

  BeginBlock := TPasImplBeginBlock(CreateElement(TPasImplBeginBlock, '', Parent));
  Parent.Body := BeginBlock;
  repeat
    NextToken;
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
  ExpectToken(tkSemicolon);
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
  begin
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

<<<<<<< HEAD
  procedure CreateBlock(NewBlock: TPasImplBlock);
  begin
    CurBlock:=NewBlock;
    if NewImplElement=nil then NewImplElement:=CurBlock;
=======
//!!  there may be DEPRECATED token
    CheckHint(Element,False);
    NextToken;
    
>>>>>>> graemeg/fixes_2_2
  end;

var
  Condition: String;
  StartValue: String;
  VarName: String;
  EndValue: String;
  Expr: String;
  SubBlock: TPasImplElement;
  CmdElem: TPasImplElement;
  TypeName: String;
  ForDownTo: Boolean;
  left: TPasExpr;
  right: TPasExpr;
begin
  NewImplElement:=nil;
  CurBlock := Parent;
  while True do
  begin
    NextToken;
    //WriteLn(i,'Token=',CurTokenText);
    case CurToken of
    tkbegin:
      CreateBlock(CurBlock.AddBeginBlock);
    tkrepeat:
      CreateBlock(CurBlock.AddRepeatUntil);
    tkIf:
      begin
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'IF Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddIfElse(Condition));
        ExpectToken(tkthen);
      end;
    tkelse:
      if (CurBlock is TPasImplIfElse) then
      begin
        if TPasImplIfElse(CurBlock).IfBranch=nil then
        begin
          // empty then => add dummy command
          CurBlock.AddCommand('');
        end;
        if TPasImplIfElse(CurBlock).ElseBranch<>nil then
        begin
          // this and the following 3 may solve TPasImplIfElse.AddElement BUG
          // ifs without begin end
          // if .. then
          //  if .. then
          //   else
          // else
          CloseBlock;
          CloseStatement(false);
        end;
      end else if (CurBlock is TPasImplWhileDo) then
      begin
        //if .. then while .. do smt else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplRaise) then
      begin
        //if .. then Raise Exception else ..
        CloseBlock;
        UngetToken;
      end else if (CurBlock is TPasImplTryExcept) then
      begin
        CloseBlock;
        CurBlock:=TPasImplTry(CurBlock).AddExceptElse;
      end else
        ParseExc(SParserSyntaxError);
    tkwhile:
      begin
        // while Condition do
        Condition:=ParseExpression(Parent);
        //WriteLn(i,'WHILE Condition="',Condition,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWhileDo(Condition));
        ExpectToken(tkdo);
      end;
    tkgoto:
      begin
        nexttoken;
        curblock.AddCommand('goto '+curtokenstring);
        expecttoken(tkSemiColon);
      end;
    tkfor:
      begin
        // for VarName := StartValue to EndValue do
        ExpectIdentifier;
        VarName:=CurTokenString;
        ExpectToken(tkAssign);
        StartValue:=ParseExpression(Parent);
        //writeln(i,'FOR Start=',StartValue);
        NextToken;
        if CurToken=tkTo then
          ForDownTo:=false
        else if CurToken=tkdownto then
          ForDownTo:=true
        else
          ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkTo]]));
        EndValue:=ParseExpression(Parent);
        CreateBlock(CurBlock.AddForLoop(VarName,StartValue,EndValue,ForDownTo));
        //WriteLn(i,'FOR "',VarName,'" := ',StartValue,' to ',EndValue,' Token=',CurTokenText);
        ExpectToken(tkdo);
      end;
    tkwith:
      begin
        // with Expr do
        // with Expr, Expr do
        Expr:=ParseExpression(Parent);
        //writeln(i,'WITH Expr="',Expr,'" Token=',CurTokenText);
        CreateBlock(CurBlock.AddWithDo(Expr));
        repeat
          NextToken;
          if CurToken=tkdo then break;
          if CurToken<>tkComma then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkdo]]));
          Expr:=ParseExpression(Parent);
          //writeln(i,'WITH ...,Expr="',Expr,'" Token=',CurTokenText);
          TPasImplWithDo(CurBlock).AddExpression(Expr);
        until false;
      end;
    tkcase:
      begin
        Expr:=ParseExpression(Parent);
        //writeln(i,'CASE OF Expr="',Expr,'" Token=',CurTokenText);
        ExpectToken(tkof);
        CreateBlock(CurBlock.AddCaseOf(Expr));
        repeat
          NextToken;
          //writeln(i,'CASE OF Token=',CurTokenText);
          case CurToken of
          tkend:
            break; // end without else
          tkelse:
            begin
              // create case-else block
              CurBlock:=TPasImplCaseOf(CurBlock).AddElse;
              break;
            end
          else
            UngetToken;
            // read case values
            repeat
              Expr:=ParseExpression(Parent);
              //writeln(i,'CASE value="',Expr,'" Token=',CurTokenText);
              NextToken;
              if CurToken=tkDotDot then
              begin
                Expr:=Expr+'..'+ParseExpression(Parent);
                NextToken;
              end;
              // do not miss '..'
              if CurBlock is TPasImplCaseStatement then
                TPasImplCaseStatement(CurBlock).Expressions.Add(Expr)
              else
                CurBlock:=TPasImplCaseOf(CurBlock).AddCase(Expr);
              //writeln(i,'CASE after value Token=',CurTokenText);
              if CurToken=tkColon then break;
              if CurToken<>tkComma then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkComma]]));
            until false;
            // read statement
            ParseStatement(CurBlock,SubBlock);
            CloseBlock;
            if CurToken<>tkSemicolon then
            begin
              NextToken;
              if not (CurToken in [tkSemicolon,tkelse,tkend]) then
                ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
              if CurToken<>tkSemicolon then
                UngetToken;
            end;
          end;
        until false;
        if CurToken=tkend then
        begin
          if CloseBlock then break;
          if CloseStatement(false) then break;
        end;
      end;
    tktry:
      CreateBlock(CurBlock.AddTry);
    tkfinally:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplTry then
        begin
          CurBlock:=TPasImplTry(CurBlock).AddFinally;
        end else
          ParseExc(SParserSyntaxError);
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
          CurBlock:=TPasImplTry(CurBlock).AddExcept;
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkon:
      begin
        // in try except:
        // on E: Exception do
        // on Exception do
        if CurBlock is TPasImplTryExcept then
        begin
          VarName:='';
          TypeName:=ParseExpression(Parent);
          //writeln(i,'ON t=',TypeName,' Token=',CurTokenText);
          NextToken;
          if CurToken=tkColon then
          begin
            VarName:=TypeName;
            TypeName:=ParseExpression(Parent);
            //writeln(i,'ON v=',VarName,' t=',TypeName,' Token=',CurTokenText);
          end else
            UngetToken;
          CurBlock:=TPasImplTryExcept(CurBlock).AddExceptOn(VarName,TypeName);
          ExpectToken(tkDo);
        end else
          ParseExc(SParserSyntaxError);
      end;
    tkraise:
      CreateBlock(CurBlock.AddRaise);
    tkend:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplBeginBlock then
        begin
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
          ParseExc(SParserSyntaxError);
      end;
    tkSemiColon:
      if CloseStatement(true) then break;
    tkuntil:
      begin
        if CloseStatement(true) then
        begin
          UngetToken;
          break;
        end;
        if CurBlock is TPasImplRepeatUntil then
        begin
          Condition:=ParseExpression(Parent);
          TPasImplRepeatUntil(CurBlock).Condition:=Condition;
          //WriteLn(i,'UNTIL Condition="',Condition,'" Token=',CurTokenString);
          if CloseBlock then break;
        end else
          ParseExc(SParserSyntaxError);
      end;
    else
      left:=DoParseExpression(nil);
      case CurToken of
        tkAssign:
        begin
          // assign statement
          NextToken;
          right:=DoParseExpression(nil); // this may solve TPasImplWhileDo.AddElement BUG
          CmdElem:=CurBlock.AddAssign(left, right);
          UngetToken;
        end;
        tkColon:
        begin
          if not (left is TPrimitiveExpr) then
            ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
          // label mark. todo: check mark identifier in the list of labels
          CmdElem:=CurBlock.AddLabelMark(TPrimitiveExpr(left).Value);
          left.Free;
        end;
      else
        // simple statement (function call)
        CmdElem:=CurBlock.AddSimple(left);
        UngetToken;
      end;

      if not (CmdElem is TPasImplLabelMark) then
        if NewImplElement=nil then NewImplElement:=CmdElem;
    end;
  end;
end;

procedure TPasParser.ParseLabels(AParent: TPasElement);
var
  Labels: TPasLabels;
begin
  Labels:=TPasLabels(CreateElement(TPasLabels, '', AParent));
  repeat
    Labels.Labels.Add(ExpectIdentifier);
    NextToken;
    if not (CurToken in [tkSemicolon, tkComma]) then
      ParseExc(Format(SParserExpectTokenError, [TokenInfos[tkSemicolon]]));
  until CurToken=tkSemicolon;
end;

>>>>>>> origin/cpstrnew
// Starts after the "procedure" or "function" token
function TPasParser.ParseProcedureOrFunctionDecl(Parent: TPasElement;
  ProcType: TProcType): TPasProcedure;

  function ExpectProcName: string;
  begin
    Result:=ExpectIdentifier;
    //writeln('ExpectProcName ',Parent.Classname);
    if Parent is TImplementationSection then
    begin
      NextToken;
      if CurToken=tkDot then
      begin
        Result:=Result+'.'+ExpectIdentifier;
      end else
        UngetToken;
    end;
  end;

var
  Name: String;
begin
  case ProcType of
    ptFunction:
      begin
        Name := ExpectProcName;
        Result := TPasFunction(CreateElement(TPasFunction, Name, Parent));
        Result.ProcType := Engine.CreateFunctionType('', 'Result', Result, True,
                                           Scanner.CurFilename, Scanner.CurRow);
      end;
    ptClassFunction:
      begin
        Name := ExpectProcName;
        Result := TPasClassFunction(CreateElement(TPasClassFunction, Name, Parent));
        Result.ProcType := Engine.CreateFunctionType('', 'Result', Result, True,
                                           Scanner.CurFilename, Scanner.CurRow);
      end;
    ptProcedure:
      begin
        Name := ExpectProcName;
        Result := TPasProcedure(CreateElement(TPasProcedure, Name, Parent));
        Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Result));
      end;
    ptClassProcedure:
      begin
        Name := ExpectProcName;
        Result := TPasClassProcedure(CreateElement(TPasClassProcedure, Name, Parent));
        Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Result));
      end;
    ptOperator:
      begin
        NextToken;
        Name := 'operator ' + TokenInfos[CurToken];
        Result := TPasOperator(CreateElement(TPasOperator, Name, Parent));
        Result.ProcType := Engine.CreateFunctionType('', '__INVALID__', Result,
                                     True, Scanner.CurFilename, Scanner.CurRow);
      end;
    ptConstructor:
      begin
        Name := ExpectProcName;
        Result := TPasConstructor(CreateElement(TPasConstructor, Name, Parent));
        Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Result));
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew
      end;
    ptDestructor:
      begin
        Name := ExpectProcName;
        Result := TPasDestructor(CreateElement(TPasDestructor, Name, Parent));
        Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Result));
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
      end;
    ptDestructor:
      begin
        Name := ExpectProcName;
        Result := TPasDestructor(CreateElement(TPasDestructor, Name, Parent));
        Result.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Result));
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;
  finally
    VarList.Free;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TPasParser.ParseClassLocalTypes(AType: TPasClassType; AVisibility : TPasMemberVisibility);

Var
  T : TPasType;
  Done : Boolean;
begin
//  Writeln('Parsing local types');
  Repeat
    T:=ParseTypeDecl(AType);
    T.Visibility:=AVisibility;
    AType.Members.Add(t);
//    Writeln(CurtokenString,' ',TokenInfos[Curtoken]);
    NextToken;
    Done:=(Curtoken<>tkIdentifier) or CheckVisibility(CurtokenString,AVisibility);
    if Done then
      UngetToken;
  Until Done;
end;

procedure TPasParser.ParseClassLocalConsts(AType: TPasClassType; AVisibility : TPasMemberVisibility);

Var
  C : TPasConst;
  Done : Boolean;
begin
//  Writeln('Parsing local consts');
  Repeat
    C:=ParseConstDecl(AType);
    C.Visibility:=AVisibility;
    AType.Members.Add(C);
//    Writeln(CurtokenString,' ',TokenInfos[Curtoken]);
    NextToken;
    Done:=(Curtoken<>tkIdentifier) or CheckVisibility(CurtokenString,AVisibility);
    if Done then
      UngetToken;
  Until Done;
end;

procedure TPasParser.ParseClassMembers(AType: TPasClassType);

Var
  CurVisibility : TPasMemberVisibility;

begin
  CurVisibility := visDefault;
  while (CurToken<>tkEnd) do
    begin
    case CurToken of
      tkType:
        begin
        ExpectToken(tkIdentifier);
        SaveComments;
        ParseClassLocalTypes(AType,CurVisibility);
        end;
      tkConst:
        begin
        ExpectToken(tkIdentifier);
        SaveComments;
        ParseClassLocalConsts(AType,CurVisibility);
        end;
      tkVar,
      tkIdentifier:
        begin
        if (AType.ObjKind=okInterface) then
          ParseExc(SParserNoFieldsAllowed);
        if CurToken=tkVar then
          ExpectToken(tkIdentifier);
        SaveComments;
        if Not CheckVisibility(CurtokenString,CurVisibility) then
          ParseClassFields(AType,CurVisibility,false);
        end;
      tkProcedure,tkFunction,tkConstructor,tkDestructor:
        begin
        SaveComments;
        if (Curtoken in [tkConstructor,tkDestructor]) and (AType.ObjKind in [okInterface,okRecordHelper]) then
          ParseExc(SParserNoConstructorAllowed);
        ProcessMethod(AType,False,CurVisibility);
        end;
      tkclass:
        begin
         SaveComments;
         NextToken;
         if CurToken in [tkConstructor,tkDestructor,tkprocedure,tkFunction] then
           ProcessMethod(AType,True,CurVisibility)
         else if CurToken = tkVar then
           begin
           ExpectToken(tkIdentifier);
           ParseClassFields(AType,CurVisibility,true);
           end
         else if CurToken=tkProperty then
           begin
           ExpectToken(tkIdentifier);
           AType.Members.Add(ParseProperty(AType,CurtokenString,CurVisibility));
           end
         else
           ParseExc(SParserTypeSyntaxError)
        end;
      tkProperty:
        begin
        SaveComments;
        ExpectIdentifier;
        AType.Members.Add(ParseProperty(AType,CurtokenString,CurVisibility));
        end;
    end;
    NextToken;
    end;
=======
  //writeln('TPasParser.ParseProcedureOrFunctionDecl Name="',Name,'" Token=',CurTokenText);
=======
  //writeln('TPasParser.ParseProcedureOrFunctionDecl Name="',Name,'" Token=',CurTokenText);

=======
  //writeln('TPasParser.ParseProcedureOrFunctionDecl Name="',Name,'" Token=',CurTokenText);

>>>>>>> graemeg/cpstrnew
=======
  //writeln('TPasParser.ParseProcedureOrFunctionDecl Name="',Name,'" Token=',CurTokenText);

>>>>>>> origin/cpstrnew
  ParseProcedureOrFunctionHeader(Result, Result.ProcType, ProcType, False);
end;
>>>>>>> graemeg/cpstrnew

  ParseProcedureOrFunctionHeader(Result, Result.ProcType, ProcType, False);
>>>>>>> graemeg/cpstrnew
end;
procedure TPasParser.DoParseClassType(AType: TPasClassType);

var
  Element : TPasElement;
  s: String;

begin
  // nettism/new delphi features
  if (CurToken=tkIdentifier) and (Atype.ObjKind in [okClass,okGeneric]) then
    begin
    s := LowerCase(CurTokenString);
    if (s = 'sealed') or (s = 'abstract') then
      begin
      AType.Modifiers.Add(s);
      NextToken;
      end;
    end;
  // Parse ancestor list
  Atype.IsForward:=(CurToken=tkSemiColon);
  if (CurToken=tkBraceOpen) then
    begin
    AType.AncestorType := ParseType(nil);
    while True do
      begin
      NextToken;
      if CurToken = tkBraceClose then
        break;
      UngetToken;
      ExpectToken(tkComma);
      Element:=ParseType(Nil); // search interface.
      if assigned(element) then
        AType.Interfaces.add(element);
      end;
    NextToken;
    AType.IsShortDefinition:=(CurToken=tkSemicolon);
    end;
  if (AType.ObjKind in [okClassHelper,okRecordHelper]) then
    begin
    if (CurToken<>tkFor) then
      ParseExc(Format(SParserExpectTokenError,[TokenInfos[tkFor]]));
    AType.HelperForType:=ParseType(Nil);
    NextToken;
    end;
  if (AType.IsShortDefinition or AType.IsForward) then
    UngetToken
  else
    begin
    if (AType.ObjKind=okInterface) and (CurToken = tkSquaredBraceOpen) then
      begin
<<<<<<< HEAD
      NextToken;
      AType.GUIDExpr:=DoParseExpression(AType);
      if (CurToken<>tkSquaredBraceClose) then
        ParseExc(Format(SParserExpectTokenError,[TokenInfos[tkSquaredBraceClose]]));
      NextToken;
      end;
    ParseClassMembers(AType);
    end;
=======
        UngetToken;
    UngetToken;
      end;
      Parent.VariantType := ParseType(Parent);
      Parent.Variants := TList.Create;

      ExpectToken(tkOf);

      while True do
      begin
        Variant := TPasVariant(CreateElement(TPasVariant, '', Parent));
        Parent.Variants.Add(Variant);
        while True do
        begin
          Variant.Values.Add(ParseExpression(Parent));
          NextToken;
          if CurToken = tkColon then
            break
          else if CurToken <> tkComma then
            ParseExc(SParserExpectedCommaColon);
        end;
        ExpectToken(tkBraceOpen);
    Variant.Members := TPasRecordType(CreateElement(TPasRecordType, '',
      Variant));
    try
          ParseRecordDecl(Variant.Members, True);
    except
      Variant.Members.Free;
      raise;
    end;
    NextToken;
    if CurToken = tkSemicolon then
      NextToken;
    if (CurToken = tkEnd) or (CurToken = tkBraceClose) then
      break
    else
      UngetToken;
      end
    end else
      ParseInlineVarDecl(Parent, Parent.Members, visDefault, IsNested);
  end;
>>>>>>> graemeg/cpstrnew
end;

function TPasParser.ParseClassDecl(Parent: TPasElement;
  const AClassName: String; AObjKind: TPasObjKind; PackMode: TPackMode
  ): TPasType;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Var
  SourcefileName : string;
  SourceLineNumber : Integer;

<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  procedure ProcessMethod(ProcType: TProcType);
  var
    Owner: TPasElement;
    Proc: TPasProcedure;
    s,Name: String;
    pt: TProcType;
    HasReturnValue: Boolean;

  begin
    HasReturnValue:=false;
    ExpectIdentifier;
    Name := CurTokenString;
    Owner := CheckIfOverloaded(TPasClassType(Result), Name);
    case ProcType of
     ptFunction:
    begin
         Proc := TPasFunction(CreateElement(TPasFunction, Name, Owner,
        CurVisibility));
      Proc.ProcType := Engine.CreateFunctionType('', 'Result', Proc, True,
        Scanner.CurFilename, Scanner.CurRow);
        HasReturnValue:=true;
       end;
     ptClassFunction:
    begin
         Proc := TPasClassFunction(CreateElement(TPasClassFunction, Name, Owner));
         Proc.ProcType := Engine.CreateFunctionType('', 'Result', Proc, True,
          Scanner.CurFilename, Scanner.CurRow);
         HasReturnValue:=true;
       end;
     ptClassProcedure:
       begin
         Proc := TPasClassProcedure(CreateElement(TPasClassProcedure, Name, Owner));
         Proc.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Proc, CurVisibility));
       end;
     ptConstructor:
       begin
        Proc := TPasConstructor(CreateElement(TPasConstructor, Name,
          Owner, CurVisibility));
        Proc.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Proc, CurVisibility));
       end;
     ptDestructor:
       begin
        Proc := TPasDestructor(CreateElement(TPasDestructor, Name,
          Owner, CurVisibility));
        Proc.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
          Proc, CurVisibility));
       end;
      else
        Proc := TPasProcedure(CreateElement(TPasProcedure, Name,
          Owner, CurVisibility));
      Proc.ProcType := TPasProcedureType(CreateElement(TPasProcedureType, '',
        Proc, CurVisibility));
    end;
    
    if Owner.ClassType = TPasOverloadedProc then
      TPasOverloadedProc(Owner).Overloads.Add(Proc)
    else
      TPasClassType(Result).Members.Add(Proc);

    if HasReturnValue then
      pt := ptFunction
    else
      pt := ptProcedure;
    ParseProcedureOrFunctionHeader(Proc, Proc.ProcType, pt, False);

    while True do
    begin
      NextToken;
      if CurToken = tkIdentifier then
      begin
        s := UpperCase(CurTokenString);
        if s = 'VIRTUAL' then
          Proc.AddModifier(pmVirtual)
        else if s = 'DYNAMIC' then
          Proc.AddModifier(pmDynamic)
        else if s = 'ABSTRACT' then
          Proc.AddModifier(pmAbstract)
        else if s = 'OVERRIDE' then
          Proc.AddModifier(pmOverride)
        else if s = 'REINTRODUCE' then
          Proc.AddModifier(pmReintroduce)
        else if s = 'OVERLOAD' then
          Proc.AddModifier(pmOverload)
        else if s = 'STATIC' then
          Proc.AddModifier(pmStatic)
        else if s = 'MESSAGE' then begin
          Proc.AddModifier(pmMessage);
          repeat
            NextToken;
            If CurToken<>tkSemicolon then
              begin
              Proc.MessageName:=CurtokenString;
              If (CurToken=tkString) then
                Proc.Messagetype:=pmtString;
              end;
          until CurToken = tkSemicolon;
          UngetToken;
        end
    else if s = 'CDECL' then
      Proc.CallingConvention:=ccCDecl
    else if s = 'PASCAL' then
      Proc.CallingConvention:=ccPascal
        else if s = 'STDCALL' then
          Proc.CallingConvention:=ccStdCall
        else if s = 'OLDFPCCALL' then
          Proc.CallingConvention:=ccOldFPCCall
        else if s = 'EXTDECL' then
          Proc.AddModifier(pmExtdecl)
        else if s = 'DEPRECATED' then
         Proc.Hints:=Proc.Hints+[hDeprecated]
        else if s = 'EXPORT' then
          Proc.AddModifier(pmExported)
        else
        begin
          UngetToken;
          break;
        end;
        ExpectToken(tkSemicolon);
      end else
      begin
        UngetToken;
        break;
      end;
    end;
  end;

var
  s, SourceFilename: String;
  i, SourceLinenumber: Integer;
  VarList: TList;
  Element: TPasElement;
  isStrict: Boolean;
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
begin
  isStrict:=False;

  // Save current parsing position to get it correct in all cases
  SourceFilename := Scanner.CurFilename;
  SourceLinenumber := Scanner.CurRow;

  NextToken;

  if (AObjKind = okClass) and (CurToken = tkOf) then
    begin
    Result := TPasClassOfType(Engine.CreateElement(TPasClassOfType, AClassName,
      Parent, SourceFilename, SourceLinenumber));
    ExpectIdentifier;
    UngetToken;                // Only names are allowed as following type
    TPasClassOfType(Result).DestType := ParseType(Result);
    exit;
    end;
  if (CurToken = tkHelper) then
    begin
    if Not (AObjKind in [okClass,okRecordHelper]) then
      ParseExc(Format(SParserHelperNotAllowed,[ObjKindNames[AObjKind]]));
    if (AObjKind = okClass)  then
      AObjKind:=okClassHelper;
    NextToken;
    end;
  Result := TPasClassType(Engine.CreateElement(TPasClassType, AClassName,
    Parent, SourceFilename, SourceLinenumber));

  try
    TPasClassType(Result).ObjKind := AObjKind;
<<<<<<< HEAD
    TPasClassType(Result).PackMode:=PackMode;
    DoParseClassType(TPasClassType(Result));
=======

    // nettism/new delphi features
    if (CurToken = tkIdentifier) and (AObjKind = okClass) then begin
      s := LowerCase(CurTokenString);
      if (s = 'sealed') or (s = 'abstract') then begin
        TPasClassType(Result).Modifiers.Add(s);
        NextToken;
      end;
    end;

    // nettism/new delphi features
    if (CurToken = tkIdentifier) and (AObjKind = okClass) then begin
      s := LowerCase(CurTokenString);
      if (s = 'sealed') or (s = 'abstract') then begin
        TPasClassType(Result).Modifiers.Add(s);
        NextToken;
      end;
    end;

    // nettism/new delphi features
    if (CurToken = tkIdentifier) and (AObjKind = okClass) then begin
      s := LowerCase(CurTokenString);
      if (s = 'sealed') or (s = 'abstract') then begin
        TPasClassType(Result).Modifiers.Add(s);
        NextToken;
      end;
    end;

    // nettism/new delphi features
    if (CurToken = tkIdentifier) and (AObjKind = okClass) then begin
      s := LowerCase(CurTokenString);
      if (s = 'sealed') or (s = 'abstract') then begin
        TPasClassType(Result).Modifiers.Add(s);
        NextToken;
      end;
    end;

    // Parse ancestor list
    if CurToken = tkBraceOpen then
    begin
      TPasClassType(Result).AncestorType := ParseType(nil);
      {$ifdef Inheritancewarnings}
        s:=TPasClassType(Result).AncestorType.pathname;
        if pos('#',s)=0 then
          begin
            writeln('Note: ', TPasClassType(Result).pathname,'''s ancestor ',s, ' at ',sourcefilename,':',sourcelinenumber,' cannot be resolved fully');
          end;
      {$endif}
      while True do
      begin
        NextToken;
        if CurToken = tkBraceClose then
          break;
        UngetToken;
        ExpectToken(tkComma);
        //ExpectIdentifier;
        Element:=ParseType(Nil); // search interface.
        if assigned(element) then
          TPasClassType(Result).Interfaces.add(element);
        // !!!: Store interface name
      end;
      NextToken;
    end
    else
      TPasClassType(Result).isForward:=CurToken=tkSemicolon;
<<<<<<< HEAD
<<<<<<< HEAD
    if CurToken = tkSemicolon then
       TPasClassType(Result).IsShortDefinition:=true;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    if CurToken <> tkSemicolon then
    begin
      if ( AObjKind = okInterface ) and ( CurToken = tkSquaredBraceOpen ) then
      begin
        ExpectToken(tkString);
        TPasClassType(Result).InterfaceGUID := CurTokenString;
        ExpectToken(tkSquaredBraceClose);
<<<<<<< HEAD
<<<<<<< HEAD
      end;
=======
      end;    
>>>>>>> graemeg/fixes_2_2
=======
      end;    
>>>>>>> origin/fixes_2_2
      CurVisibility := visDefault;
      while CurToken <> tkEnd do
      begin
        case CurToken of
          tkIdentifier:
            begin
              s := LowerCase(CurTokenString);
              if s = 'strict' then
              begin
                isStrict:=True;
                NextToken;
                s := LowerCase(CurTokenString);
              end
              else
                isStrict:=False;

              if s = 'private' then
                CurVisibility := visPrivate
              else if s = 'protected' then
                CurVisibility := visProtected
              else if s = 'public' then
                CurVisibility := visPublic
              else if s = 'published' then
                CurVisibility := visPublished
              else if s = 'automated' then
                CurVisibility := visAutomated
              else
              begin
                VarList := TList.Create;
                try
                  ParseInlineVarDecl(Result, VarList, CurVisibility, False);
                  for i := 0 to VarList.Count - 1 do
                  begin
                    Element := TPasElement(VarList[i]);
                    Element.Visibility := CurVisibility;
                    TPasClassType(Result).Members.Add(Element);
                  end;
                finally
                  VarList.Free;
                end;
              end;
              if isStrict then
              begin
                case CurVisibility of
                  visPrivate   : CurVisibility:=visStrictPrivate;
                  visProtected : CurVisibility:=visStrictProtected;
                else
                  ParseExc('strange strict visiblity');
                end;
              end;

            end;
          tkProcedure:
            ProcessMethod(ptProcedure);
          tkFunction:
            ProcessMethod(ptFunction);
          tkConstructor:
            ProcessMethod(ptConstructor);
          tkDestructor:
            ProcessMethod(ptDestructor);
          tkclass:
            begin
             NextToken;
             if CurToken = tkprocedure then ProcessMethod(ptClassProcedure)
              else ProcessMethod(ptClassFunction);
            end;               
          tkProperty:
            begin
              ExpectIdentifier;
              Element := CreateElement(TPasProperty, CurTokenString, Result, CurVisibility);
              TPasClassType(Result).Members.Add(Element);
              ParseProperty(Element);
            end;
          tkVar: // vars (nettism/new delphi features)
            if AObjKind<>okClass then ExpectToken(tkSemicolon);
          //todo: class vars
        end; // end case
        NextToken;
      end;
      // Eat semicolon after class...end
      CheckHint(result,true);
//      ExpectToken(tkSemicolon);
    end;
>>>>>>> graemeg/cpstrnew
  except
    Result.Free;
    raise;
  end;
end;

function TPasParser.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement): TPasElement;
begin
  Result := Engine.CreateElement(AClass, AName, AParent,
    Scanner.CurFilename, Scanner.CurRow);
end;

function TPasParser.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility): TPasElement;
begin
  Result := Engine.CreateElement(AClass, AName, AParent, AVisibility,
    Scanner.CurFilename, Scanner.CurRow);
end;

<<<<<<< HEAD
function TPasParser.CreateFunctionType(const AName, AResultName: String;
  AParent: TPasElement; UseParentAsResultParent: Boolean): TPasFunctionType;
begin
  Result:=Engine.CreateFunctionType(AName,AResultName,
                                    AParent,UseParentAsResultParent,
                                    Scanner.CurFilename,Scanner.CurRow);
end;
=======

function ParseSource(AEngine: TPasTreeContainer;
  const FPCCommandLine, OSTarget, CPUTarget: String): TPasModule;
var
  FileResolver: TFileResolver;
  Parser: TPasParser;
  Start, CurPos: PChar;
  Filename: String;
  Scanner: TPascalScanner;

  procedure ProcessCmdLinePart;
  var
    l: Integer;
    s: String;
  begin
    l := CurPos - Start;
    SetLength(s, l);
    if l > 0 then
      Move(Start^, s[1], l)
    else
      exit;
    if s[1] = '-' then
    begin
      case s[2] of
        'd': // -d define
          Scanner.Defines.Append(UpperCase(Copy(s, 3, Length(s))));
        'F': // -F
          if s[3] = 'i' then // -Fi include path
            FileResolver.AddIncludePath(Copy(s, 4, Length(s)));
<<<<<<< HEAD
<<<<<<< HEAD
        'I': // -I include path
          FileResolver.AddIncludePath(Copy(s, 3, Length(s)));
        'S': // -S mode
=======
=======
>>>>>>> origin/fixes_2_2
        'I':
          FileResolver.AddIncludePath(Copy(s, 3, Length(s)));    
        'S':
>>>>>>> graemeg/fixes_2_2
          if s[3]='d' then
            begin // -Sd mode delphi
              include(Scanner.Options,po_delphi);
              include(Parser.Options,po_delphi);
            end;
      end;
    end else
      if Filename <> '' then
        raise Exception.Create(SErrMultipleSourceFiles)
      else
        Filename := s;
  end;

var
  s: String;
begin
  Result := nil;
  FileResolver := nil;
  Scanner := nil;
  Parser := nil;
  try
    FileResolver := TFileResolver.Create;
    Scanner := TPascalScanner.Create(FileResolver);
    Scanner.Defines.Append('FPK');
    Scanner.Defines.Append('FPC');

    // TargetOS
    s := UpperCase(OSTarget);
    Scanner.Defines.Append(s);
    if s = 'LINUX' then
      Scanner.Defines.Append('UNIX')
    else if s = 'FREEBSD' then
    begin
      Scanner.Defines.Append('BSD');
      Scanner.Defines.Append('UNIX');
    end else if s = 'NETBSD' then
    begin
      Scanner.Defines.Append('BSD');
      Scanner.Defines.Append('UNIX');
    end else if s = 'SUNOS' then
    begin
      Scanner.Defines.Append('SOLARIS');
      Scanner.Defines.Append('UNIX');
    end else if s = 'GO32V2' then
      Scanner.Defines.Append('DPMI')
    else if s = 'BEOS' then
      Scanner.Defines.Append('UNIX')
    else if s = 'QNX' then
      Scanner.Defines.Append('UNIX');

    // TargetCPU
    s := UpperCase(CPUTarget);
    Scanner.Defines.Append('CPU'+s);
    if (s='x86_64') then
      Scanner.Defines.Append('CPU64')
    else
      Scanner.Defines.Append('CPU32');

    Parser := TPasParser.Create(Scanner, FileResolver, AEngine);
    Filename := '';
>>>>>>> graemeg/cpstrnew



initialization

procedure DoInit;
var
  c: Char;
begin
  for c:=low(char) to high(char) do
  begin
    IsIdentStart[c]:=c in ['a'..'z','A'..'Z','_'];
  end;
end;

<<<<<<< HEAD
=======
procedure DoInit;
var
  c: Char;
begin
  for c:=low(char) to high(char) do
  begin
    IsIdentStart[c]:=c in ['a'..'z','A'..'Z','_'];
  end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
procedure DoInit;
var
  c: Char;
begin
  for c:=low(char) to high(char) do
  begin
    IsIdentStart[c]:=c in ['a'..'z','A'..'Z','_'];
  end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
procedure DoInit;
var
  c: Char;
begin
  for c:=low(char) to high(char) do
  begin
    IsIdentStart[c]:=c in ['a'..'z','A'..'Z','_'];
  end;
end;

>>>>>>> origin/cpstrnew
initialization
  DoInit;

end.
