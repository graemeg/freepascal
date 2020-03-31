{
    This file is part of the Free Component Library

    WEBIDL source lexical scanner
    Copyright (c) 2018 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}

unit webidlscanner;

interface

uses SysUtils, Classes;


type
  TWebIDLVersion = (v1,v2);

  TIDLToken = (
    tkEOF,
    tkUnknown ,
    tkComment,
    tkWhitespace,
    tkString,
    tkNumberInteger,
    tkNumberFloat,
    // Simple (one-character) tokens
    tkDot, // '.',
    tkSemiColon, // ';'
    tkComma,                 // ','
    tkColon,                 // ':'
    tkBracketOpen,           // '('
    tkBracketClose,          // ')'
    tkCurlyBraceOpen,        // '{'
    tkCurlyBraceClose,       // '}'
    tkSquaredBraceOpen,       // '['
    tkSquaredBraceClose,      // ']'
    tkLess, // '<'
    tkEqual, // '='
    tkLarger, // '>'
    tkQuestionmark, // '?'
    tkminus, // '-'
    tkIdentifier,            // Any  identifier
    tkTrue,
    tkFalse,
    tkNull,
    tkAny,
    tkAttribute,
    tkCallback,
    tkConst,
    tkDeleter,
    tkDictionary,
    tkEllipsis,
    tkEnum,
    tkGetter,
    tkImplements,
    tkInfinity,
    tkInherit,
    tkInterface,
    tkIterable,
    tkLegacyCaller,
    tkNan,
    tkNegInfinity,
    tkOptional,
    tkOr,
    tkPartial,
    tkReadOnly,
    tkRequired,
    tkSetter,
    tkStatic,
    tkStringifier,
    tkSerializer,
    tkTypedef,
    tkUnrestricted,
    tkPromise,
    tkFrozenArray,
    tkByteString,
    tkDOMString,
    tkUSVString,
    tkboolean,
    tkbyte,
    tkdouble,
    tkfloat,
    tklong,
    tkobject,
    tkoctet,
    tkunsigned,
    tkvoid,
    tkShort,
    tkSequence,
    tkStringToken,
    tkMixin,
    tkIncludes,
    tkMapLike,
    tkRecord,
    tkSetLike,
    tkOther,
    tkConstructor
    );
  TIDLTokens = Set of TIDLToken;
  EWebIDLScanner = class(EParserError);

Const
  V2Tokens = [tkMixin,tkIncludes,tkMapLike,tkRecord,tkSetLike,tkFrozenArray,tkConstructor];
  V1Tokens = [tkImplements];
  VersionNonTokens : Array[TWebIDLVersion] of TIDLTokens = (V2Tokens,V1Tokens);

Type

  { TWebIDLScanner }

  TWebIDLScanner = class
  private
    FSource : TStringList;
    FCurRow: Integer;
    FCurToken: TIDLToken;
    FCurTokenString: UTF8string;
    FCurLine: UTF8string;
    FVersion: TWebIDLVersion;
    TokenStr: PChar;
    function DetermineToken: TIDLToken;
    function DetermineToken2: TIDLToken;
    function FetchLine: Boolean;
    function GetCurColumn: Integer;
    function ReadComment: UTF8String;
    function ReadIdent: UTF8String;
    function ReadNumber(var S: UTF8String): TIDLToken;
  protected
    Function GetErrorPos : String;
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Const Args: array of Const);overload;
    function ReadString: UTF8String; virtual;
    function DoFetchToken: TIDLToken;
  public
    constructor Create(Source: TStream); overload;
    constructor Create(const Source: UTF8String); overload;
    constructor CreateFile(const aFileName: UTF8String);
    destructor Destroy; override;
    function FetchToken: TIDLToken;

    property CurLine: UTF8String read FCurLine;
    property CurRow: Integer read FCurRow;
    property CurColumn: Integer read GetCurColumn;

    property CurToken: TIDLToken read FCurToken;
    property CurTokenString: UTF8String read FCurTokenString;
    Property Version : TWebIDLVersion Read FVersion Write FVersion;
  end;

const
  TokenInfos: array[TIDLToken] of string = (
  '',
  '',
  '',
  '',
  '',
  '',
  '',
  // Simple (one-character) tokens
  '.',
  ';',
  ',',                 // ','
  ':',                 // ':'
  '(',           // '('
  ')',          // ')'
  '{',        // '{'
  '}',       // '}'
  '[',       // '['
  ']',      // ']'
  '<',
  '=',
  '>',
  '?',
  '-',
  '',            // Any  identifier
  'true',
  'false',
  'null',
  'any',
  'attribute',
  'callback',
  'const',
  'deleter',
  'dictionary',
  'ellipsis',
  'enum',
  'getter',
  'implements',
  'Infinity',
  'inherit',
  'interface',
  'iterable',
  'legacycaller',
  'NaN',
  '-Infinity',
  'optional',
  'or',
  'partial',
  'readonly',
  'required',
  'setter',
  'static',
  'stringifier',
  'serializer',
  'typedef',
  'unrestricted',
  'Promise',
  'FrozenArray',
  'ByteString',
  'DOMString',
  'USVString',
  'boolean',
  'byte',
  'double',
  'float',
  'long',
  'object',
  'octet',
  'unsigned',
  'void',
  'short',
  'sequence',
  'string',
  'mixin',
  'includes',
  'maplike',
  'record',
  'setlike',
  'other',
  'constructor'
  );

Function GetTokenName(aToken : TIDLToken) : String;
Function GetTokenNames(aTokenList : TIDLTokens) : String;

implementation

uses typinfo;

Resourcestring
  SErrUnknownTerminator = 'Unknown terminator: "%s"';
  SErrInvalidCharacter = 'Invalid character at line %d, pos %d: ''%s''';
  SUnterminatedComment = 'Unterminated comment at line %d, pos %d: ''%s''';
  SErrOpenString = 'string exceeds end of line';
  SErrInvalidEllipsis = 'Invalid ellipsis token';
  SErrUnknownToken = 'Unknown token, expected number or minus : "%s"';
//  SerrExpectedTokenButWasIdentifier = 'Invalid terminator: "%s"';

Function GetTokenName(aToken : TIDLToken) : String;

begin
  Result:=TokenInfos[aToken];
  if Result='' then
    begin
    Result:=GetEnumName(TypeInfo(TIDLToken),Ord(aToken));
    Delete(Result,1,2);
    end;
end;

Function GetTokenNames(aTokenList : TIDLTokens) : String;

Var
  T : TIDLToken;

begin
  Result:='';
  For T in aTokenList do
    begin
    if (Result<>'') then
      Result:=Result+',';
    Result:=Result+GetTokenName(T);
    end;
end;


constructor TWebIDLScanner.Create(Source: TStream);
begin
  FSource:=TStringList.Create;
  FSource.LoadFromStream(Source);
end;

constructor TWebIDLScanner.Create(const Source: UTF8String);
begin
  FSource:=TStringList.Create;
  FSource.Text:=Source;
end;

constructor TWebIDLScanner.CreateFile(const aFileName: UTF8String);
begin
  FSource:=TStringList.Create;
  FSource.LoadFromFile(aFileName);
end;

destructor TWebIDLScanner.Destroy;
begin
  FreeAndNil(FSource);
  Inherited;
end;


function TWebIDLScanner.FetchToken: TIDLToken;

begin
  Result:=DoFetchToken;
end;

procedure TWebIDLScanner.Error(const Msg: string);
begin
  raise EWebIDLScanner.Create(GetErrorPos+Msg);
end;

procedure TWebIDLScanner.Error(const Msg: string; const Args: array of const);
begin
  raise EWebIDLScanner.Create(GetErrorPos+Format(Msg, Args));
end;

function TWebIDLScanner.ReadString : UTF8String;

Var
  C : Char;
  I, OldLength, SectionLength: Integer;
  S : UTF8String;
  TokenStart: PChar;
begin
  C:=TokenStr[0];
  Inc(TokenStr);
  TokenStart := TokenStr;
  OldLength := 0;
  Result := '';
  while not (TokenStr[0] in [#0,C]) do
    begin
    if (TokenStr[0]='\') then
      begin
      // Save length
      SectionLength := TokenStr - TokenStart;
      Inc(TokenStr);
      // Read escaped token
      Case TokenStr[0] of
        '"' : S:='"';
        '''' : S:='''';
        't' : S:=#9;
        'b' : S:=#8;
        'n' : S:=#10;
        'r' : S:=#13;
        'f' : S:=#12;
        '\' : S:='\';
        '/' : S:='/';
        'u' : begin
              S:='0000';
              For I:=1 to 4 do
                begin
                Inc(TokenStr);
                Case TokenStr[0] of
                  '0'..'9','A'..'F','a'..'f' :
                    S[i]:=Upcase(TokenStr[0]);
                else
                  Error(SErrInvalidCharacter, [CurRow,CurColumn,TokenStr[0]]);
                end;
                end;
              // WideChar takes care of conversion...
              S:=Utf8Encode(WideString(WideChar(StrToInt('$'+S))))
              end;
        #0  : Error(SErrOpenString);
      else
        Error(SErrInvalidCharacter, [CurRow,CurColumn,TokenStr[0]]);
      end;
      SetLength(Result, OldLength + SectionLength+1+Length(S));
      if SectionLength > 0 then
        Move(TokenStart^, Result[OldLength + 1], SectionLength);
      Move(S[1],Result[OldLength + SectionLength+1],Length(S));
      Inc(OldLength, SectionLength+Length(S));
      // Next char
      // Inc(TokenStr);
      TokenStart := TokenStr+1;
      end;
    if TokenStr[0] = #0 then
      Error(SErrOpenString);
    Inc(TokenStr);
    end;
  if TokenStr[0] = #0 then
    Error(SErrOpenString);
  SectionLength := TokenStr - TokenStart;
  SetLength(Result, OldLength + SectionLength);
  if SectionLength > 0 then
    Move(TokenStart^, Result[OldLength + 1], SectionLength);
  Inc(TokenStr);
end;

function TWebIDLScanner.ReadIdent: UTF8String;

Var
  TokenStart : PChar;
  SectionLength : Integer;

begin
  Result:='';
  if TokenStr[0]='_' then
    Inc(TokenStr);
  if TokenStr[0]=#0 then
    Exit;
  TokenStart := TokenStr;
  repeat
    Inc(TokenStr);
  until not (TokenStr[0] in ['A'..'Z', 'a'..'z', '0'..'9', '_']);
  SectionLength := TokenStr - TokenStart;
  SetString(Result, TokenStart, SectionLength);
end;

function TWebIDLScanner.FetchLine: Boolean;

begin
  Result:=FCurRow<FSource.Count;
  if Result then
    begin
    FCurLine:=FSource[FCurRow];
    TokenStr:=PChar(FCurLine);
    Inc(FCurRow);
    end
  else
    begin
    FCurLine:='';
    TokenStr:=nil;
    end;
end;

function TWebIDLScanner.ReadNumber(var S : UTF8String) : TIDLToken;

Var
  TokenStart : PChar;
  SectionLength : Integer;
  isHex : Boolean;

begin
  isHex:=False;
  TokenStart := TokenStr;
  Result:=tkNumberInteger;
  while true do
    begin
    Inc(TokenStr);
    SectionLength := TokenStr - TokenStart;
    case TokenStr[0] of
    'x':
      begin
      isHex:=True;
      end;
    'I':
      begin
      repeat
        Inc(TokenStr);
      until not (TokenStr[0] in ['i','n','f','t','y']);
      Result:=tkNegInfinity; // We'll check at the end if the string is actually correct
      break;
      end;
    '.':
      begin
      Result:=tkNumberFloat;
      if TokenStr[1] in ['0'..'9', 'e', 'E'] then
        begin
        Inc(TokenStr);
        repeat
          Inc(TokenStr);
        until not (TokenStr[0] in ['0'..'9', 'e', 'E','-','+']);
        end;
        break;
      end;
    '0'..'9':
      begin
      end;
    'a'..'d','f',
    'A'..'D','F':
      begin
      if Not isHex then
        Error(SErrUnknownToken,[S]);
      end;
    'e', 'E':
      begin
      if not  IsHex then
        begin
        Inc(TokenStr);
        if TokenStr[0] in ['-','+']  then
          Inc(TokenStr);
        while TokenStr[0] in ['0'..'9'] do
          Inc(TokenStr);
        break;
        end;
      end;
    else
      if (SectionLength=1) and (TokenStart[0]='-') then
        result:=tkMinus;
      break;
    end;
    end;
  SectionLength := TokenStr - TokenStart;
  S:='';
  SetString(S, TokenStart, SectionLength);
  if (Result=tkNegInfinity) and (S<>'-Infinity') then
    Error(SErrUnknownToken,[S]);
  if (Result=tkMinus) and (S<>'-') then
    Error(SErrUnknownTerminator,[s]);
end;

function TWebIDLScanner.GetErrorPos: String;
begin
  Result:=Format('Scanner error at line %d, pos %d: ',[CurRow,CurColumn]);
end;

function TWebIDLScanner.ReadComment : UTF8String;

Var
  TokenStart : PChar;
  SectionLength : Integer;
  EOC,IsStar : Boolean;
  S : String;

begin
  Result:='';
  TokenStart:=TokenStr;
  Inc(TokenStr);
  Case Tokenstr[0] of
    '/' : begin
          SectionLength := Length(FCurLine)- (TokenStr - PChar(FCurLine));
          Inc(TokenStr);
          SetString(Result, TokenStr, SectionLength);
          Fetchline;
          end;
    '*' :
      begin
      IsStar:=False;
      Inc(TokenStr);
      TokenStart:=TokenStr;
      Repeat
        if (TokenStr[0]=#0) then
          begin
          SectionLength := (TokenStr - TokenStart);
          S:='';
          SetString(S, TokenStart, SectionLength);
          Result:=Result+S;
          if not fetchLine then
            Error(SUnterminatedComment, [CurRow,CurCOlumn,TokenStr[0]]);
          TokenStart:=TokenStr;
          end;
        IsStar:=TokenStr[0]='*';
        Inc(TokenStr);
        EOC:=(isStar and (TokenStr[0]='/'));
      Until EOC;
      if EOC then
        begin
        SectionLength := (TokenStr - TokenStart-1);
        S:='';
        SetString(S, TokenStart, SectionLength);
        Result:=Result+S;
        Inc(TokenStr);
        end;
      end;
  else
    Error(SErrInvalidCharacter, [CurRow,CurCOlumn,TokenStr[0]]);
  end;
end;

function TWebIDLScanner.DetermineToken : TIDLToken;

begin
  Result:=High(TIDLToken);
  While (Result<>tkIdentifier) and (TokenInfos[result]<>FCurTokenString) do
    Result:=Pred(Result);
  if Result in VersionNonTokens[Version] then
    Result:=tkIdentifier;
//  if Result=tkIdentifier then
//    Error(SErrExpectedTokenButWasIdentifier,[FCurTokenString]);
end;

function TWebIDLScanner.DetermineToken2 : TIDLToken;

Const
  InfTokens = [tkNan,tkInfinity,tkNegInfinity,tkByteString,tkUSVString,tkDOMString,tkPromise,tkFrozenArray];

begin
  For Result in InfTokens do
    if (TokenInfos[result]=FCurTokenString) then exit;
  Result:=tkIdentifier;
end;

function TWebIDLScanner.DoFetchToken: TIDLToken;

  Procedure SetSingleToken(tk : TIDLToken);

  begin
    FCurTokenString:=TokenStr[0];
    Inc(TokenStr);
    Result :=tk;
  end;

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
      if not FetchLine then
        Result:=tkEOF
      else
        Result := tkWhitespace;
      end;
    #9, ' ':
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
      until not (TokenStr[0] in [#9, ' ']);
      end;
    '"':
      begin
        FCurTokenString:=ReadString;
        Result := tkString;
      end;
    ',':
      begin
        Inc(TokenStr);
        Result := tkComma;
      end;
    '0'..'9','-':
      begin
      Result := ReadNumber(FCurTokenString);
      end;
    ':': SetSingleToken(tkColon);
    '(': SetSingleToken(tkBracketOpen);
    ')': SetSingleToken(tkBracketClose);
    '{': SetSingleToken(tkCurlyBraceOpen);
    '}': SetSingleToken(tkCurlyBraceClose);
    '[': SetSingleToken(tkSquaredBraceOpen);
    ']': SetSingleToken(tkSquaredBraceClose);
    '<': SetSingleToken(tkLess);
    '=': SetSingleToken(tkEqual);
    '>': SetSingleToken(tkLarger);
    '?' : SetSingleToken(tkQuestionmark);
    ';' : SetSingleToken(tkSemicolon);
    '.' :
       begin
       inc(TokenStr);
       if TokenStr[0]<>'.' then
         begin
         Dec(Tokenstr);// Setsingletoken advances
         SetSingleToken(tkDot);
         end
       else
         begin
         inc(TokenStr);
         if TokenStr[0]<>'.' then
           Error(SErrInvalidEllipsis);
         inc(TokenStr);
         FCurTokenString:='...';
         Result:=tkEllipsis;
         end;
       end;
    '/' :
      begin
      FCurTokenString:=ReadComment;
      Result:=tkComment;
      end;
    'a'..'z':
      begin
      FCurTokenString:=ReadIdent;
      Result:=DetermineToken;
      end;
    'A'..'Z','_':
      begin
      FCurTokenString:=ReadIdent;
      Result:=tkIdentifier;
      Result:=DetermineToken2;
      end;
  else
    Error(SErrInvalidCharacter, [CurRow,CurCOlumn,TokenStr[0]]);
  end;

  FCurToken := Result;
end;

function TWebIDLScanner.GetCurColumn: Integer;
begin
  Result := TokenStr - PChar(CurLine);
end;


end.
