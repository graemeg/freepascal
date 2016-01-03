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

{$mode objfpc}
{$h+}

unit PScanner;

interface

uses SysUtils, Classes;

resourcestring
  SErrInvalidCharacter = 'Invalid character ''%s''';
  SErrOpenString = 'string exceeds end of line';
  SErrIncludeFileNotFound = 'Could not find include file ''%s''';
  SErrIfXXXNestingLimitReached = 'Nesting of $IFxxx too deep';
  SErrInvalidPPElse = '$ELSE without matching $IFxxx';
  SErrInvalidPPEndif = '$ENDIF without matching $IFxxx';
  SLogOpeningFile = 'Opening source file "%s".';
  SLogLineNumber = 'Reading line %d.';
  SLogIFDefAccepted = 'IFDEF %s found, accepting.';
  SLogIFDefRejected = 'IFDEF %s found, rejecting.';
  SLogIFNDefAccepted = 'IFNDEF %s found, accepting.';
  SLogIFNDefRejected = 'IFNDEF %s found, rejecting.';
  SLogIFOPTIgnored = 'IFOPT %s found, ignoring (rejected).';
  SLogIFIgnored = 'IF %s found, ignoring (rejected).';

type

  TToken = (
    tkEOF,
    tkWhitespace,
    tkComment,
    tkIdentifier,
    tkString,
    tkNumber,
    tkChar,
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
    tkHelper,
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
    tkon,
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TTokens = set of TToken;

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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

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
    FFilename: string;
    FTextFile: Text;
    FileOpened: Boolean;
    FBuffer : Array[0..4096-1] of byte;
  public
    constructor Create(const AFilename: string); override;
    destructor Destroy; override;
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
    property Filename: string read FFilename;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;

<<<<<<< HEAD
  { TStreamLineReader }

  TStreamLineReader = class(TLineReader)
  private
    FContent: AnsiString;
    FPos : Integer;
  public
    Procedure InitFromStream(AStream : TStream);
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
    constructor Create( const AFilename: string; Const ASource: String);
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
=======
=======
  end;

>>>>>>> graemeg/cpstrnew
=======
  end;

>>>>>>> graemeg/cpstrnew
=======
  end;

>>>>>>> origin/cpstrnew
  { TFileResolver }

  TFileResolver = class
  private
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
    FBaseDirectory: string;
    FIncludePaths: TStringList;
    FStrictFileCase : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
  Protected
    procedure SetBaseDirectory(AValue: string); virtual;
    procedure SetStrictFileCase(AValue: Boolean); virtual;
    Function FindIncludeFileName(const AName: string): String;
    Property IncludePaths: TStringList Read FIncludePaths;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  public
    constructor Create; virtual;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddIncludePath(const APath: string); virtual;
    function FindSourceFile(const AName: string): TLineReader; virtual; abstract;
    function FindIncludeFile(const AName: string): TLineReader; virtual; abstract;
    Property StrictFileCase : Boolean Read FStrictFileCase Write SetStrictFileCase;
    property BaseDirectory: string read FBaseDirectory write SetBaseDirectory;
  end;

  { TFileResolver }

  TFileResolver = class(TBaseFileResolver)
  private
    FUseStreams: Boolean;
  Protected
    Function CreateFileReader(Const AFileName : String) : TLineReader; virtual;
  Public
    function FindSourceFile(const AName: string): TLineReader; override;
    function FindIncludeFile(const AName: string): TLineReader; override;
    Property UseStreams : Boolean Read FUseStreams Write FUseStreams;
  end;

  { TStreamResolver }

  TStreamResolver = class(TBaseFileResolver)
  Private
    FOwnsStreams: Boolean;
    FStreams : TStringList;
    function FindStream(const AName: string; ScanIncludes: Boolean): TStream;
    function FindStreamReader(const AName: string; ScanIncludes: Boolean): TLineReader;
    procedure SetOwnsStreams(AValue: Boolean);
  Public
    constructor Create; override;
    destructor Destroy; override;
    Procedure Clear;
    Procedure AddStream(Const AName : String; AStream : TStream);
    function FindSourceFile(const AName: string): TLineReader; override;
    function FindIncludeFile(const AName: string): TLineReader; override;
    Property OwnsStreams : Boolean Read FOwnsStreams write SetOwnsStreams;
=======
    procedure AddIncludePath(const APath: string);
    function FindSourceFile(const AName: string): TLineReader;
    function FindIncludeFile(const AName: string): TLineReader;
    Property StrictFileCase : Boolean Read FStrictFileCase Write FStrictFileCase;
<<<<<<< HEAD
<<<<<<< HEAD
    property BaseDirectory: string read FBaseDirectory write FBaseDirectory;
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
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  EScannerError       = class(Exception);
  EFileNotFoundError  = class(Exception);

  TPascalScannerPPSkipMode = (ppSkipNone, ppSkipIfBranch, ppSkipElseBranch, ppSkipAll);

  TPOption = (po_delphi,po_cassignments);
  TPOptions = set of TPOption;

  { TPascalScanner }

  TPScannerLogHandler = Procedure (Sender : TObject; Const Msg : String) of object;
  TPScannerLogEvent = (sleFile,sleLineNumber,sleConditionals);
  TPScannerLogEvents = Set of TPScannerLogEvent;

  { TPascalScanner }

  { TPascalScanner }

  { TPascalScanner }

  { TPascalScanner }

  TPascalScanner = class
  private
    FFileResolver: TBaseFileResolver;
    FCurSourceFile: TLineReader;
    FCurFilename: string;
    FCurRow: Integer;
    FCurToken: TToken;
    FCurTokenString: string;
    FCurLine: string;
    FMacros,
    FDefines: TStrings;
    FOptions: TPOptions;
    FLogEvents: TPScannerLogEvents;
    FOnLog: TPScannerLogHandler;
    FSkipComments: Boolean;
    FSkipWhiteSpace: Boolean;
    TokenStr: PChar;
    FIncludeStack: TFPList;

    // Preprocessor $IFxxx skipping data
    PPSkipMode: TPascalScannerPPSkipMode;
    PPIsSkipping: Boolean;
    PPSkipStackIndex: Integer;
    PPSkipModeStack: array[0..255] of TPascalScannerPPSkipMode;
    PPIsSkippingStack: array[0..255] of Boolean;

    function GetCurColumn: Integer;
    procedure SetOptions(AValue: TPOptions);
  protected
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure DoLog(Const Msg : String; SkipSourceInfo : Boolean = False);overload;
    Procedure DoLog(Const Fmt : String; Args : Array of const;SkipSourceInfo : Boolean = False);overload;
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Args: array of Const);overload;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure HandleDefine(Param: String); virtual;
    procedure HandleIncludeFile(Param: String); virtual;
    procedure HandleUnDefine(Param: String);virtual;
    function HandleMacro(AIndex: integer): TToken;virtual;
    procedure PushStackItem; virtual;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function DoFetchTextToken: TToken;
=======
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Args: array of Const);overload;
>>>>>>> graemeg/fixes_2_2
=======
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Args: array of Const);overload;
>>>>>>> origin/fixes_2_2
    function DoFetchToken: TToken;
    procedure ClearFiles;
    Procedure ClearMacros;
    Procedure SetCurTokenString(AValue : string);
    function LogEvent(E : TPScannerLogEvent) : Boolean; inline;
  public
    constructor Create(AFileResolver: TBaseFileResolver);
    destructor Destroy; override;
    procedure OpenFile(const AFilename: string);
    function FetchToken: TToken;
    Procedure AddDefine(S : String);
    Procedure RemoveDefine(S : String);

    property FileResolver: TBaseFileResolver read FFileResolver;
    property CurSourceFile: TLineReader read FCurSourceFile;
    property CurFilename: string read FCurFilename;
    Property SkipWhiteSpace : Boolean Read FSkipWhiteSpace Write FSkipWhiteSpace;
    Property SkipComments : Boolean Read FSkipComments Write FSkipComments;
    property CurLine: string read FCurLine;
    property CurRow: Integer read FCurRow;
    property CurColumn: Integer read GetCurColumn;

    property CurToken: TToken read FCurToken;
    property CurTokenString: string read FCurTokenString;

    property Defines: TStrings read FDefines;
    property Macros: TStrings read FMacros;
    Property Options : TPOptions Read FOptions Write SetOptions;
    Property LogEvents : TPScannerLogEvents Read FLogEvents Write FLogEvents;
    Property OnLog : TPScannerLogHandler Read FOnLog Write FOnLog;
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
    'helper',
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
    'on',
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

function FilenameIsAbsolute(const TheFilename: string):boolean;
function FilenameIsWinAbsolute(const TheFilename: string): boolean;
function FilenameIsUnixAbsolute(const TheFilename: string): boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function IsNamedToken(Const AToken : String; Out T : TToken) : Boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

implementation

Var
  SortedTokens : array of TToken;
  LowerCaseTokens  : Array[ttoken] of String;

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

type
  TIncludeStackItem = class
    SourceFile: TLineReader;
    Filename: string;
    Token: TToken;
    TokenString: string;
    Line: string;
    Row: Integer;
    TokenStr: PChar;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ TMacroDef }

constructor TMacroDef.Create(const AName, AValue: String);
begin
  FName:=AName;
  FValue:=AValue;
end;

{ TStreamResolver }

procedure TStreamResolver.SetOwnsStreams(AValue: Boolean);
begin
  if FOwnsStreams=AValue then Exit;
  FOwnsStreams:=AValue;
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
      I:=FStreams.INdexOf(FN);
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

{ TStringStreamLineReader }

constructor TStringStreamLineReader.Create(const AFilename: string;  const ASource: String);

Var
  S : TStringStream;

begin
  inherited Create(AFilename);
  S:=TStringStream.Create(ASource);
  try
     InitFromStream(S);
  finally
    S.Free;
  end;
end;

{ TFileStreamLineReader }

constructor TFileStreamLineReader.Create(const AFilename: string);

Var
  S : TFileStream;

begin
  inherited Create(AFilename);
  S:=TFileStream.Create(AFileName,fmOpenRead or fmShareDenyWrite);
  try
     InitFromStream(S);
  finally
    S.Free;
  end;
end;

{ TStreamLineReader }

Procedure TStreamLineReader.InitFromStream(AStream : TStream);

begin
  SetLength(FContent,AStream.Size);
  AStream.Read(FContent[1],AStream.Size);
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
    exit;
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

{ TLineReader }

constructor TLineReader.Create(const AFilename: string);
begin
  FFileName:=AFileName;
end;

{ ---------------------------------------------------------------------
  TFileLineReader
  ---------------------------------------------------------------------}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

constructor TFileLineReader.Create(const AFilename: string);

begin
<<<<<<< HEAD
  inherited Create(AFileName);
=======
  inherited Create;
  FFilename:=AFilename;
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
  Assign(FTextFile, AFilename);
  Reset(FTextFile);
  SetTextBuf(FTextFile,FBuffer,SizeOf(FBuffer));
  FileOpened := true;
end;

destructor TFileLineReader.Destroy;
begin
  if FileOpened then
    Close(FTextFile);
  inherited Destroy;
end;

function TFileLineReader.IsEOF: Boolean;
begin
  Result := EOF(FTextFile);
end;

function TFileLineReader.ReadLine: string;
begin
  ReadLn(FTextFile, Result);
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

function TBaseFileResolver.FindIncludeFileName(const AName: string): String;

  function SearchLowUpCase(FN: string): string;

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

var
  i: Integer;
  FN : string;

begin
  Result := '';
  // convert pathdelims to system
  FN:=SetDirSeparators(AName);
  If FilenameIsAbsolute(FN) then
    begin
    // Maybe this should also do a SearchLowUpCase ?
    if FileExists(FN) then
      Result := FN;
    end
  else
    begin
    // file name is relative
    // search in include path
    I:=0;
    While (Result='') and (I<FIncludePaths.Count) do
      begin
      Result:=SearchLowUpCase(FIncludePaths[i]+AName);
      Inc(I);
      end;
    // search in BaseDirectory
    if (Result='') and (BaseDirectory<>'') then
      Result:=SearchLowUpCase(BaseDirectory+AName);
    end;
end;

constructor TBaseFileResolver.Create;
begin
  inherited Create;
  FIncludePaths := TStringList.Create;
end;

destructor TBaseFileResolver.Destroy;
begin
  FIncludePaths.Free;
  inherited Destroy;
end;

procedure TBaseFileResolver.AddIncludePath(const APath: string);
begin
  if (APath='') then
    FIncludePaths.Add('./')
  else
    FIncludePaths.Add(IncludeTrailingPathDelimiter(ExpandFileName(APath)));
end;

{ ---------------------------------------------------------------------
  TFileResolver
  ---------------------------------------------------------------------}

function TFileResolver.CreateFileReader(const AFileName: String): TLineReader;
begin
  If UseStreams then
    Result:=TFileStreamLineReader.Create(AFileName)
  else
    Result:=TFileLineReader.Create(AFileName);
end;

function TFileResolver.FindSourceFile(const AName: string): TLineReader;
begin
  if not FileExists(AName) then
    Raise EFileNotFoundError.create(Aname)
  else
    try
      Result := CreateFileReader(AName)
    except
      Result := nil;
    end;
end;

function TFileResolver.FindIncludeFile(const AName: string): TLineReader;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Var
  FN : String;

begin
  Result:=Nil;
  FN:=FindIncludeFileName(ANAme);
  If (FN<>'') then
    try
      Result := TFileLineReader.Create(FN);
    except
      Result:=Nil;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  function SearchLowUpCase(FN: string): string;
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

var
  i: Integer;
  FN : string;

begin
  Result := nil;
  // convert pathdelims to system
  FN:=SetDirSeparators(AName);

  If FilenameIsAbsolute(FN) then
    begin
      if FileExists(FN) then
        Result := TFileLineReader.Create(FN);
    end
  else
    begin
    // file name is relative

    // search in include path
    I:=0;
    While (Result=Nil) and (I<FIncludePaths.Count) do
      begin
      Try
<<<<<<< HEAD
        FN:=SearchLowUpCase(FIncludePaths[i]+AName);
=======
        FN:=FIncludePaths[i]+AName;
        If not FileExists(FN) then
          If StrictFileCase then
            FN:=''
          else
            begin 
            fn:=LowerCase(FN);
            If not FileExists(Fn) then
              begin
              FN:=uppercase(Fn);
              If not FileExists(FN) then
                FN:='';
              end;    
            end;  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        If (FN<>'') then
          Result := TFileLineReader.Create(FN);
      except
        Result:=Nil;
      end;
      Inc(I);
      end;
    // search in BaseDirectory
    if BaseDirectory<>'' then
      begin
      FN:=SearchLowUpCase(BaseDirectory+AName);
	  try
      If (FN<>'') then   
        Result := TFileLineReader.Create(FN);
      except 
        Result:=nil;
        end;		
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

begin
  inherited Create;
  FFileResolver := AFileResolver;
  FIncludeStack := TFPList.Create;
  FDefines := CS;
  FMacros:=CS;
end;

destructor TPascalScanner.Destroy;
begin
  ClearMacros;
  FreeAndNil(FMacros);
  FreeAndNil(FDefines);
  ClearFiles;
  FIncludeStack.Free;
  inherited Destroy;
end;

procedure TPascalScanner.ClearFiles;

begin
  // Dont' free the first element, because it is CurSourceFile
  while FIncludeStack.Count > 1 do
    begin
    TFileResolver(FIncludeStack[1]).Free;
    FIncludeStack.Delete(1);
    end;
  FIncludeStack.Clear;
  FreeAndNil(FCurSourceFile);
end;

procedure TPascalScanner.ClearMacros;

Var
  I : Integer;

begin
  For I:=0 to FMacros.Count-1 do
      FMacros.Objects[i].Free;
  FMacros.Clear;
end;

procedure TPascalScanner.SetCurTokenString(AValue: string);
begin
  FCurtokenString:=AValue;
end;

procedure TPascalScanner.OpenFile(const AFilename: string);
begin
  Clearfiles;
  FCurSourceFile := FileResolver.FindSourceFile(AFilename);
  if LogEvent(sleFile) then
    DoLog(SLogOpeningFile,[AFileName],True);
  FCurFilename := AFilename;
  FileResolver.BaseDirectory := IncludeTrailingPathDelimiter(ExtractFilePath(AFilename));
end;

function TPascalScanner.FetchToken: TToken;
var
  IncludeStackItem: TIncludeStackItem;
begin
  while true do
  begin
    Result := DoFetchToken;
    Case FCurToken of
    tkEOF:
      begin
      if FIncludeStack.Count > 0 then
        begin
        CurSourceFile.Free;
        IncludeStackItem :=
          TIncludeStackItem(FIncludeStack[FIncludeStack.Count - 1]);
        FIncludeStack.Delete(FIncludeStack.Count - 1);
        FCurSourceFile := IncludeStackItem.SourceFile;
        FCurFilename := IncludeStackItem.Filename;
        FCurToken := IncludeStackItem.Token;
        FCurTokenString := IncludeStackItem.TokenString;
        FCurLine := IncludeStackItem.Line;
        FCurRow := IncludeStackItem.Row;
        TokenStr := IncludeStackItem.TokenStr;
        IncludeStackItem.Free;
        Result := FCurToken;
        end
      else
        break
      end;
    tkWhiteSpace,
    tkLineEnding:
      if not (FSkipWhiteSpace or PPIsSkipping) then
        Break;
    tkComment:
      if not (FSkipComments or PPIsSkipping) then
        Break;
    else
      if not PPIsSkipping then
        break;
    end; // Case
  end;
//  Writeln(Result, '(',CurTokenString,')');
end;

procedure TPascalScanner.Error(const Msg: string);
begin
  raise EScannerError.Create(Msg);
end;

procedure TPascalScanner.Error(const Msg: string; Args: array of Const);
begin
  raise EScannerError.CreateFmt(Msg, Args);
end;

function TPascalScanner.DoFetchTextToken:TToken;
var
  OldLength     : Integer;
  TokenStart    : PChar;
  SectionLength : Integer;
begin
  Result:=tkEOF;
  OldLength:=0;
  FCurTokenString := '';

  while TokenStr[0] in ['#', ''''] do
  begin
    case TokenStr[0] of
      '#':
        begin
          TokenStart := TokenStr;
          Inc(TokenStr);
          if TokenStr[0] = '$' then
          begin
            Inc(TokenStr);
            repeat
              Inc(TokenStr);
            until not (TokenStr[0] in ['0'..'9', 'A'..'F', 'a'..'f']);
          end else
            repeat
              Inc(TokenStr);
            until not (TokenStr[0] in ['0'..'9']);
          if Result=tkEOF then Result := tkChar else Result:=tkString;
        end;
      '''':
        begin
          TokenStart := TokenStr;
          Inc(TokenStr);

          while true do
          begin
            if TokenStr[0] = '''' then
              if TokenStr[1] = '''' then
                Inc(TokenStr)
              else
                break;

            if TokenStr[0] = #0 then
              Error(SErrOpenString);

            Inc(TokenStr);
          end;
          Inc(TokenStr);
          Result := tkString;
        end;
    else
      Break;
    end;
    SectionLength := TokenStr - TokenStart;
    SetLength(FCurTokenString, OldLength + SectionLength);
    if SectionLength > 0 then
      Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
    Inc(OldLength, SectionLength);
  end;

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TPascalScanner.PushStackItem;

Var
  SI: TIncludeStackItem;

begin
  SI := TIncludeStackItem.Create;
  SI.SourceFile := CurSourceFile;
  SI.Filename := CurFilename;
  SI.Token := CurToken;
  SI.TokenString := CurTokenString;
  SI.Line := CurLine;
  SI.Row := CurRow;
  SI.TokenStr := TokenStr;
  FIncludeStack.Add(SI);
  TokenStr:=Nil;
  FCurRow := 0;
end;

Procedure TPascalScanner.HandleIncludeFile(Param : String);

begin
  PushStackItem;
  if Length(Param)>1 then
    begin
      if (Param[1]=#39) and (Param[length(Param)]=#39) then
       param:=copy(param,2,length(param)-2);
    end;
  FCurSourceFile := FileResolver.FindIncludeFile(Param);
  if not Assigned(FCurSourceFile) then
    Error(SErrIncludeFileNotFound, [Param]);
  FCurFilename := Param;
  if FCurSourceFile is TFileLineReader then
    FCurFilename := TFileLineReader(FCurSourceFile).Filename; // nicer error messages
  If LogEvent(sleFile) then
    DoLog(SLogOpeningFile,[FCurFileName],True);
end;

function TPascalScanner.HandleMacro(AIndex : integer) : TToken;

Var
  M : TMacroDef;
  ML : TMacroReader;

begin
  PushStackItem;
  M:=FMacros.Objects[AIndex] as TMacroDef;
  ML:=TMacroReader.Create(FCurFileName,M.Value);
  ML.CurRow:=FCurRow;
  ML.CurCol:=CurColumn;
  FCurSourceFile:=ML;
  Result:=DofetchToken;
//  Writeln(Result,Curtoken);
end;

Procedure TPascalScanner.HandleDefine(Param : String);

Var
  Index : Integer;
  MN,MV : String;

begin
  Param := UpperCase(Param);
  Index:=Pos(':=',Param);
  If (Index=0) then
    AddDefine(Param)
  else
    begin
    MV:=Trim(Param);
    MN:=Trim(Copy(MV,1,Index-1));
    Delete(MV,1,Index+1);
    Index:=FMacros.IndexOf(MN);
    If (Index=-1) then
      FMacros.AddObject(MN,TMacroDef.Create(MN,MV))
    else
      TMacroDef(FMacros.Objects[index]).Value:=MV;
    end;
end;

Procedure TPascalScanner.HandleUnDefine(Param : String);

Var
  Index : integer;

begin
  Param := UpperCase(Param);
  Index:=FDefines.IndexOf(Param);
  If (Index>=0) then
    RemoveDefine(Param)
  else
    begin
    Index := FMacros.IndexOf(Param);
    If (Index>=0) then
      begin
      FMacros.Objects[Index].FRee;
      FMacros.Delete(Index);
      end;
    end;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TPascalScanner.DoFetchToken: TToken;

  function FetchLine: Boolean;
  begin
    if CurSourceFile.IsEOF then
    begin
      FCurLine := '';
      TokenStr := nil;
      Result := false;
    end else
    begin
      FCurLine := CurSourceFile.ReadLine;
      TokenStr := PChar(CurLine);
      Result := true;
      Inc(FCurRow);
      if LogEvent(sleLineNumber) and ((FCurRow Mod 100) = 0) then
        DoLog(SLogLineNumber,[FCurRow],True);
    end;
  end;

var
  TokenStart, CurPos: PChar;
  i: TToken;
  OldLength, SectionLength, NestingLevel, Index: Integer;
  Directive, Param : string;
begin
  if TokenStr = nil then
    if not FetchLine then
    begin
      Result := tkEOF;
      FCurToken := Result;
      exit;
    end;

  FCurTokenString := '';

  case TokenStr[0] of
    #0:         // Empty line
      begin
        FetchLine;
        Result := tkLineEnding;
      end;
    ' ':
      begin
        Result := tkWhitespace;
        repeat
          Inc(TokenStr);
          if TokenStr[0] = #0 then
            if not FetchLine then
            begin
              FCurToken := Result;
              exit;
            end;
        until not (TokenStr[0] in [' ']);
      end;
    #9:
      begin
        Result := tkTab;
        repeat
          Inc(TokenStr);
          if TokenStr[0] = #0 then
            if not FetchLine then
            begin
              FCurToken := Result;
              exit;
            end;
        until not (TokenStr[0] in [#9]);
      end;
    '#', '''':
      Result:=DoFetchTextToken;
    '&':
      begin
        TokenStart := TokenStr;
        repeat
          Inc(TokenStr);
        until not (TokenStr[0] in ['0'..'7']);
        SectionLength := TokenStr - TokenStart;
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
        Result := tkNumber;
      end;
    '$':
      begin
        TokenStart := TokenStr;
        repeat
          Inc(TokenStr);
        until not (TokenStr[0] in ['0'..'9', 'A'..'F', 'a'..'f']);
        SectionLength := TokenStr - TokenStart;
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
        Result := tkNumber;
      end;
    '%':
      begin
        TokenStart := TokenStr;
        repeat
          Inc(TokenStr);
        until not (TokenStr[0] in ['0','1']);
        SectionLength := TokenStr - TokenStart;
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
        Result := tkNumber;
      end;
    '(':
      begin
        Inc(TokenStr);
        if TokenStr[0] = '*' then
        begin
          // Old-style multi-line comment
          Inc(TokenStr);
          while (TokenStr[0] <> '*') or (TokenStr[1] <> ')') do
          begin
            if TokenStr[0] = #0 then
            begin
              if not FetchLine then
              begin
                Result := tkEOF;
                FCurToken := Result;
                exit;
              end;
            end else
              Inc(TokenStr);
          end;
          Inc(TokenStr, 2);
          Result := tkComment;
        end else
          Result := tkBraceOpen;
      end;
    ')':
      begin
        Inc(TokenStr);
        Result := tkBraceClose;
      end;
    '*':
      begin
        Result:=tkMul;
        Inc(TokenStr);
        if TokenStr[0] = '*' then
          begin
          Inc(TokenStr);
          Result := tkPower;
          end 
        else if (po_cassignments in options) then
          begin
          if TokenStr[0]='=' then
            begin
            Inc(TokenStr);
            Result:=tkAssignMul;
            end;
          end
      end;
    '+':
      begin
        Result:=tkPlus;
        Inc(TokenStr);
        if (po_cassignments in options) then
          begin
          if TokenStr[0]='=' then
            begin
            Inc(TokenStr);
            Result:=tkAssignPlus;
            end;
          end
      end;
    ',':
      begin
        Inc(TokenStr);
        Result := tkComma;
      end;
    '-':
      begin
        Result := tkMinus;
        Inc(TokenStr);
        if (po_cassignments in options) then
          begin
          if TokenStr[0]='=' then
            begin
            Inc(TokenStr);
            Result:=tkAssignMinus;
            end;
          end
      end;
    '.':
      begin
        Inc(TokenStr);
        if TokenStr[0] = '.' then
        begin
          Inc(TokenStr);
          Result := tkDotDot;
        end else
          Result := tkDot;
      end;
    '/':
      begin
        Result := tkDivision;
        Inc(TokenStr);
        if (TokenStr[0] = '/') then       // Single-line comment
          begin
          Inc(TokenStr);
          TokenStart := TokenStr;
          FCurTokenString := '';
          while TokenStr[0] <> #0 do
            Inc(TokenStr);
          SectionLength := TokenStr - TokenStart;
          SetLength(FCurTokenString, SectionLength);
          if SectionLength > 0 then
            Move(TokenStart^, FCurTokenString[1], SectionLength);
          Result := tkComment;
          end
        else if (po_cassignments in options) then
          begin
          if TokenStr[0]='=' then
            begin
            Inc(TokenStr);
            Result:=tkAssignDivision;
            end;
          end
      end;
    '0'..'9':
      begin
        TokenStart := TokenStr;
        while true do
        begin
          Inc(TokenStr);
          case TokenStr[0] of
            '.':
              begin
                if TokenStr[1] in ['0'..'9', 'e', 'E'] then
                begin
                  Inc(TokenStr);
                  repeat
                    Inc(TokenStr);
                  until not (TokenStr[0] in ['0'..'9', 'e', 'E']);
                end;
                break;
              end;
            '0'..'9': ;
            'e', 'E':
              begin
                Inc(TokenStr);
                if TokenStr[0] = '-'  then
                  Inc(TokenStr);
                while TokenStr[0] in ['0'..'9'] do
                  Inc(TokenStr);
                break;
              end;
            else
              break;
          end;
        end;
        SectionLength := TokenStr - TokenStart;
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
        Result := tkNumber;
      end;
    ':':
      begin
        Inc(TokenStr);
        if TokenStr[0] = '=' then
        begin
          Inc(TokenStr);
          Result := tkAssign;
        end else
          Result := tkColon;
      end;
    ';':
      begin
        Inc(TokenStr);
        Result := tkSemicolon;
      end;
    '<':
      begin
        Inc(TokenStr);
        if TokenStr[0] = '>' then
        begin
          Inc(TokenStr);
          Result := tkNotEqual;
        end else if TokenStr[0] = '=' then
        begin
          Inc(TokenStr);
          Result := tkLessEqualThan;
        end else
          Result := tkLessThan;
      end;
    '=':
      begin
        Inc(TokenStr);
        Result := tkEqual;
      end;
    '>':
      begin
        Inc(TokenStr);
        if TokenStr[0] = '=' then
        begin
          Inc(TokenStr);
          Result := tkGreaterEqualThan;
            end else if TokenStr[0] = '<' then
            begin
          Inc(TokenStr);
          Result := tkSymmetricalDifference;
        end else
          Result := tkGreaterThan;
      end;
    '@':
      begin
        Inc(TokenStr);
        Result := tkAt;
      end;
    '[':
      begin
        Inc(TokenStr);
        Result := tkSquaredBraceOpen;
      end;
    ']':
      begin
        Inc(TokenStr);
        Result := tkSquaredBraceClose;
      end;
    '^':
      begin
        Inc(TokenStr);
        Result := tkCaret;
      end;
    '\':
      begin
        Inc(TokenStr);
        Result := tkBackslash;
      end;
    '{':        // Multi-line comment
      begin
        Inc(TokenStr);
        TokenStart := TokenStr;
        FCurTokenString := '';
        OldLength := 0;
        NestingLevel := 0;
        while (TokenStr[0] <> '}') or (NestingLevel > 0) do
        begin
          if TokenStr[0] = #0 then
          begin
            SectionLength := TokenStr - TokenStart + 1;
            SetLength(FCurTokenString, OldLength + SectionLength);
            if SectionLength > 1 then
              Move(TokenStart^, FCurTokenString[OldLength + 1],
                SectionLength - 1);
            Inc(OldLength, SectionLength);
            FCurTokenString[OldLength] := #10;
            if not FetchLine then
            begin
              Result := tkEOF;
              FCurToken := Result;
              exit;
            end;
            TokenStart := TokenStr;
          end else
          begin
            if not(po_delphi in Options) and (TokenStr[0] = '{') then
              Inc(NestingLevel)
            else if TokenStr[0] = '}' then
              Dec(NestingLevel);
            Inc(TokenStr);
          end;
        end;
        SectionLength := TokenStr - TokenStart;
        SetLength(FCurTokenString, OldLength + SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
        Inc(TokenStr);
        Result := tkComment;
        //WriteLn('Kommentar: "', CurTokenString, '"');
        if (Length(CurTokenString) > 0) and (CurTokenString[1] = '$') then
        begin
          TokenStart := @CurTokenString[2];
          CurPos := TokenStart;
          while (CurPos[0] <> ' ') and (CurPos[0] <> #0) do
            Inc(CurPos);
          SectionLength := CurPos - TokenStart;
          SetLength(Directive, SectionLength);
          if SectionLength > 0 then
          begin
            Move(TokenStart^, Directive[1], SectionLength);
            Directive := UpperCase(Directive);
            if CurPos[0] <> #0 then
            begin
              TokenStart := CurPos + 1;
              CurPos := TokenStart;
              while CurPos[0] <> #0 do
                Inc(CurPos);
              SectionLength := CurPos - TokenStart;
              SetLength(Param, SectionLength);
              if SectionLength > 0 then
                Move(TokenStart^, Param[1], SectionLength);
            end else
              Param := '';
<<<<<<< HEAD
            if Not PPIsSkipping then
              begin
              if (Directive = 'I') or (Directive = 'INCLUDE') then
                begin
                if ((Param='') or (Param[1]<>'%')) then
                  HandleIncludeFile(param)
                else if Param[1]='%' then
                  begin
                  fcurtokenstring:='{$i '+param+'}';
                  fcurtoken:=tkstring;
                  result:=fcurtoken;
                  exit;
                  end
                end
              else if (Directive = 'DEFINE') then
                HandleDefine(Param)
              else if (Directive = 'UNDEF') then
                HandleUnDefine(Param)
=======
            // WriteLn('Direktive: "', Directive, '", Param: "', Param, '"');
            if (Directive = 'I') or (Directive = 'INCLUDE') then
            begin
              if (not PPIsSkipping) and ((Param='') or (Param[1]<>'%')) then
              begin
                IncludeStackItem := TIncludeStackItem.Create;
                IncludeStackItem.SourceFile := CurSourceFile;
                IncludeStackItem.Filename := CurFilename;
                IncludeStackItem.Token := CurToken;
                IncludeStackItem.TokenString := CurTokenString;
                IncludeStackItem.Line := CurLine;
                IncludeStackItem.Row := CurRow;
                IncludeStackItem.TokenStr := TokenStr;
                FIncludeStack.Add(IncludeStackItem);
                if Length(Param)>1 then
                  begin
                    if (Param[1]=#39) and (Param[length(Param)]=#39) then
                     param:=copy(param,2,length(param)-2);
                  end;
               
                FCurSourceFile := FileResolver.FindIncludeFile(Param);
                if not Assigned(CurSourceFile) then
                  Error(SErrIncludeFileNotFound, [Param]);
                FCurFilename := Param;
                if FCurSourceFile is TFileLineReader then
                  FCurFilename := TFileLineReader(FCurSourceFile).Filename; // nicer error messages
                FCurRow := 0;
              end
             else
              if Param[1]='%' then
                begin
                  fcurtokenstring:='{$i '+param+'}';
                  fcurtoken:=tkstring;  
                  result:=fcurtoken;
                  exit; 
                end;
            end else if Directive = 'DEFINE' then
            begin
              if not PPIsSkipping then
              begin
                Param := UpperCase(Param);
                if Defines.IndexOf(Param) < 0 then
                  Defines.Add(Param);
>>>>>>> graemeg/cpstrnew
              end;
            if (Directive = 'IFDEF') then
              begin
              if PPSkipStackIndex = High(PPSkipModeStack) then
                Error(SErrIfXXXNestingLimitReached);
              PPSkipModeStack[PPSkipStackIndex] := PPSkipMode;
              PPIsSkippingStack[PPSkipStackIndex] := PPIsSkipping;
              Inc(PPSkipStackIndex);
              if PPIsSkipping then
              begin
                PPSkipMode := ppSkipAll;
                PPIsSkipping := true;
              end else
              begin
                Param := UpperCase(Param);
                Index := Defines.IndexOf(Param);
                if Index < 0 then
                  Index := Macros.IndexOf(Param);
                if Index < 0 then
                begin
                  PPSkipMode := ppSkipIfBranch;
                  PPIsSkipping := true;
                end else
                  PPSkipMode := ppSkipElseBranch;
                If LogEvent(sleConditionals) then
                  if PPSkipMode=ppSkipElseBranch then
                    DoLog(SLogIFDefAccepted,[Param])
                  else
                    DoLog(SLogIFDefRejected,[Param])
              end;
            end else if Directive = 'IFNDEF' then
            begin
              if PPSkipStackIndex = High(PPSkipModeStack) then
                Error(SErrIfXXXNestingLimitReached);
              PPSkipModeStack[PPSkipStackIndex] := PPSkipMode;
              PPIsSkippingStack[PPSkipStackIndex] := PPIsSkipping;
              Inc(PPSkipStackIndex);
              if PPIsSkipping then
              begin
                PPSkipMode := ppSkipAll;
                PPIsSkipping := true;
              end else
              begin
                Param := UpperCase(Param);
                Index := Defines.IndexOf(Param);
                if Index >= 0 then
                begin
                  PPSkipMode := ppSkipIfBranch;
                  PPIsSkipping := true;
                end else
                  PPSkipMode := ppSkipElseBranch;
                If LogEvent(sleConditionals) then
                  if PPSkipMode=ppSkipElseBranch then
                    DoLog(SLogIFNDefAccepted,[Param])
                  else
                    DoLog(SLogIFNDefRejected,[Param])
              end;
            end else if Directive = 'IFOPT' then
            begin
              if PPSkipStackIndex = High(PPSkipModeStack) then
                Error(SErrIfXXXNestingLimitReached);
              PPSkipModeStack[PPSkipStackIndex] := PPSkipMode;
              PPIsSkippingStack[PPSkipStackIndex] := PPIsSkipping;
              Inc(PPSkipStackIndex);
              if PPIsSkipping then
              begin
                PPSkipMode := ppSkipAll;
                PPIsSkipping := true;
              end else
              begin
                { !!!: Currently, options are not supported, so they are just
                  assumed as not being set. }
                PPSkipMode := ppSkipIfBranch;
                PPIsSkipping := true;
              end;
              If LogEvent(sleConditionals) then
                DoLog(SLogIFOPTIgnored,[Uppercase(Param)])
            end else if Directive = 'IF' then
            begin
              if PPSkipStackIndex = High(PPSkipModeStack) then
                Error(SErrIfXXXNestingLimitReached);
              PPSkipModeStack[PPSkipStackIndex] := PPSkipMode;
              PPIsSkippingStack[PPSkipStackIndex] := PPIsSkipping;
              Inc(PPSkipStackIndex);
              if PPIsSkipping then
              begin
                PPSkipMode := ppSkipAll;
                PPIsSkipping := true;
              end else
              begin
                { !!!: Currently, expressions are not supported, so they are
                  just assumed as evaluating to false. }
                PPSkipMode := ppSkipIfBranch;
                PPIsSkipping := true;
              If LogEvent(sleConditionals) then
                 DoLog(SLogIFIgnored,[Uppercase(Param)])
              end;
            end else if Directive = 'ELSE' then
            begin
              if PPSkipStackIndex = 0 then
                Error(SErrInvalidPPElse);
              if PPSkipMode = ppSkipIfBranch then
                PPIsSkipping := false
              else if PPSkipMode = ppSkipElseBranch then
                PPIsSkipping := true;
            end else if ((Directive = 'ENDIF') or (Directive='IFEND')) then
            begin
              if PPSkipStackIndex = 0 then
                Error(SErrInvalidPPEndif);
              Dec(PPSkipStackIndex);
              PPSkipMode := PPSkipModeStack[PPSkipStackIndex];
              PPIsSkipping := PPIsSkippingStack[PPSkipStackIndex];
            end;
          end else
            Directive := '';
        end;
      end;
    'A'..'Z', 'a'..'z', '_':
      begin
        TokenStart := TokenStr;
        repeat
          Inc(TokenStr);
        until not (TokenStr[0] in ['A'..'Z', 'a'..'z', '0'..'9', '_']);
        SectionLength := TokenStr - TokenStart;
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
        for i := tkAbsolute to tkXOR do
          if CompareText(CurTokenString, TokenInfos[i]) = 0 then
          begin
            Result := i;
            FCurToken := Result;
            exit;
          end;
        Index:=FMacros.IndexOf(CurtokenString);
        if (Index=-1) then
          Result := tkIdentifier
        else
          Result:=HandleMacro(index);
      end;
<<<<<<< HEAD
<<<<<<< HEAD
  else
    if PPIsSkipping then
      Inc(TokenStr)
    else
=======
=======
>>>>>>> origin/fixes_2_2
  else 
    if PPIsSkipping then
      Inc(TokenStr)
    else  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Error(SErrInvalidCharacter, [TokenStr[0]]);
  end;

  FCurToken := Result;
end;

function TPascalScanner.LogEvent(E: TPScannerLogEvent): Boolean;
begin
  Result:=E in FLogEvents;
end;

function TPascalScanner.GetCurColumn: Integer;
begin
  If (TokenStr<>Nil) then
    Result := TokenStr - PChar(CurLine)
  else
    Result:=0;
end;

procedure TPascalScanner.DoLog(const Msg: String;SkipSourceInfo : Boolean = False);
begin
  If Assigned(FOnLog) then
    if SkipSourceInfo then
      FOnLog(Self,Msg)
    else
      FOnLog(Self,Format('%s(%d) : %s',[FCurFileName,FCurRow,Msg]));
end;

procedure TPascalScanner.DoLog(const Fmt: String; Args: array of const;SkipSourceInfo : Boolean = False);
begin
  DoLog(Format(Fmt,Args),SkipSourceInfo);
end;

procedure TPascalScanner.SetOptions(AValue: TPOptions);
begin
  if FOptions=AValue then Exit;
  FOptions:=AValue;
end;

Procedure TPascalScanner.AddDefine(S : String);

begin
  If FDefines.IndexOf(S)=-1 then
    FDefines.Add(S);
end;

Procedure TPascalScanner.RemoveDefine(S : String);

Var
  I : Integer;

begin
  I:=FDefines.IndexOf(S);
  if (I<>-1) then
    FDefines.Delete(I);
end;

end.
