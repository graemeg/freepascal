{
    This file is part of the Free Component Library

    JSON source lexical scanner
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}

unit jsonscanner;

interface

uses SysUtils, Classes;

resourcestring
<<<<<<< HEAD
<<<<<<< HEAD
  SErrInvalidCharacter = 'Invalid character at line %d, pos %d: ''%s''';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SUnterminatedComment = 'Unterminated comment at line %d, pos %d: ''%s''';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  SErrInvalidCharacter = 'Invalid character ''%s''';
>>>>>>> graemeg/fixes_2_2
=======
  SErrInvalidCharacter = 'Invalid character ''%s''';
>>>>>>> origin/fixes_2_2
  SErrOpenString = 'string exceeds end of line';

type

  TJSONToken = (
    tkEOF,
    tkWhitespace,
    tkString,
    tkNumber,
    tkTrue,
    tkFalse,
    tkNull,
    // Simple (one-character) tokens
    tkComma,                 // ','
    tkColon,                 // ':'
    tkCurlyBraceOpen,        // '{'
    tkCurlyBraceClose,       // '}'
    tkSquaredBraceOpen,       // '['
    tkSquaredBraceClose,      // ']'
<<<<<<< HEAD
<<<<<<< HEAD
    tkIdentifier,            // Any Javascript identifier
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tkComment,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    tkUnknown
    );

  EScannerError       = class(EParserError);

  TJSONOption = (joUTF8,joStrict,joComments);
  TJSONOptions = set of TJSONOption;

Const
  DefaultOptions = [joUTF8];

Type

  { TJSONScanner }

  { TJSONScanner }

  { TJSONScanner }

  { TJSONScanner }

  { TJSONScanner }

  TJSONScanner = class
  private
    FAllowComments: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    tkUnknown
    );

  EScannerError       = class(Exception);


  TJSONScanner = class
  private
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FSource : TStringList;
    FCurRow: Integer;
    FCurToken: TJSONToken;
    FCurTokenString: string;
    FCurLine: string;
<<<<<<< HEAD
<<<<<<< HEAD
    FStrict: Boolean;
    TokenStr: PChar;
    FOptions : TJSONOptions;
    function GetCurColumn: Integer;
    function GetO(AIndex: TJSONOption): Boolean;
    procedure SetO(AIndex: TJSONOption; AValue: Boolean);
  protected
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Const Args: array of Const);overload;
    function DoFetchToken: TJSONToken;
  public
<<<<<<< HEAD
    constructor Create(Source : TStream; AUseUTF8 : Boolean = True); overload; deprecated 'use options form instead';
    constructor Create(const Source : String; AUseUTF8 : Boolean = True); overload; deprecated  'use options form instead';
    constructor Create(Source: TStream; AOptions: TJSONOptions); overload;
    constructor Create(const Source: String; AOptions: TJSONOptions); overload;
=======
    constructor Create(Source : TStream); overload;
    constructor Create(const Source : String); overload;
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
=======
>>>>>>> origin/fixes_2_2
    TokenStr: PChar;
    function GetCurColumn: Integer;
  protected
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Args: array of Const);overload;
    function DoFetchToken: TJSONToken;
  public
    constructor Create(Source : TStream); overload;
    constructor Create(Source : String); overload;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    destructor Destroy; override;
    function FetchToken: TJSONToken;


    property CurLine: string read FCurLine;
    property CurRow: Integer read FCurRow;
    property CurColumn: Integer read GetCurColumn;

    property CurToken: TJSONToken read FCurToken;
    property CurTokenString: string read FCurTokenString;
<<<<<<< HEAD
<<<<<<< HEAD
    // Use strict JSON: " for strings, object members are strings, not identifiers
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property Strict : Boolean Index joStrict Read GetO Write SetO ; deprecated 'use options instead';
    // if set to TRUE, then strings will be converted to UTF8 ansistrings, not system codepage ansistrings.
    Property UseUTF8 : Boolean index joUTF8 Read GetO Write SetO; deprecated 'Use options instead';
    // Parsing options
    Property Options : TJSONOptions Read FOptions Write FOptions;
=======
    Property Strict : Boolean Read FStrict Write FStrict;
>>>>>>> graemeg/cpstrnew
=======
    Property Strict : Boolean Read FStrict Write FStrict;
>>>>>>> graemeg/cpstrnew
=======
    Property Strict : Boolean Read FStrict Write FStrict;
>>>>>>> graemeg/cpstrnew
=======
    Property Strict : Boolean Read FStrict Write FStrict;
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

const
  TokenInfos: array[TJSONToken] of string = (
    'EOF',
    'Whitespace',
    'String',
    'Number',
    'True',
    'False',
    'Null',
    ',',
    ':',
    '{',
    '}',
    '[',
    ']',
<<<<<<< HEAD
<<<<<<< HEAD
    'identifier',
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    'comment',
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
    ''
  );


implementation

<<<<<<< HEAD
<<<<<<< HEAD
constructor TJSONScanner.Create(Source : TStream; AUseUTF8 : Boolean = True);

Var
  O : TJSONOptions;

begin
  O:=DefaultOptions;
  if AUseUTF8 then
    Include(O,joUTF8)
  else
    Exclude(O,joUTF8);
  Create(Source,O);
end;

constructor TJSONScanner.Create(const Source : String; AUseUTF8 : Boolean = True);
Var
  O : TJSONOptions;

begin
  O:=DefaultOptions;
  if AUseUTF8 then
    Include(O,joUTF8)
  else
    Exclude(O,joUTF8);
  Create(Source,O);
end;

constructor TJSONScanner.Create(Source: TStream; AOptions: TJSONOptions);
begin
  FSource:=TStringList.Create;
  FSource.LoadFromStream(Source);
  FOptions:=AOptions;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TJSONScanner.Create(const Source: String; AOptions: TJSONOptions);
=======
constructor TJSONScanner.Create(const Source : String);
>>>>>>> graemeg/cpstrnew
=======
constructor TJSONScanner.Create(const Source : String);
>>>>>>> graemeg/cpstrnew
=======
constructor TJSONScanner.Create(const Source : String);
>>>>>>> graemeg/cpstrnew
=======
constructor TJSONScanner.Create(const Source : String);
>>>>>>> origin/cpstrnew
begin
  FSource:=TStringList.Create;
  FSource.Text:=Source;
  FOptions:=AOptions;
=======
=======
>>>>>>> origin/fixes_2_2
constructor TJSONScanner.Create(Source : TStream);

begin
  FSource:=TStringList.Create;
  FSource.LoadFromStream(Source);
end;

constructor TJSONScanner.Create(Source : String);
begin
  FSource:=TStringList.Create;
  FSource.Text:=Source;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TJSONScanner.Destroy;
begin
  FreeAndNil(FSource);
  Inherited;
end;


function TJSONScanner.FetchToken: TJSONToken;
  
begin
  Result:=DoFetchToken;
end;

procedure TJSONScanner.Error(const Msg: string);
begin
  raise EScannerError.Create(Msg);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONScanner.Error(const Msg: string; const Args: array of const);
=======
procedure TJSONScanner.Error(const Msg: string; Args: array of Const);
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONScanner.Error(const Msg: string; Args: array of Const);
>>>>>>> origin/fixes_2_2
begin
  raise EScannerError.CreateFmt(Msg, Args);
end;

function TJSONScanner.DoFetchToken: TJSONToken;

  function FetchLine: Boolean;
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

var
  TokenStart, CurPos: PChar;
  it : TJSONToken;
  I : Integer;
  OldLength, SectionLength, Index: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  C : char;
  S : String;
  IsStar,EOC: Boolean;

=======
  S : String;
  
>>>>>>> graemeg/fixes_2_2
=======
  S : String;
  
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    '"','''':
      begin
        C:=TokenStr[0];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        If (C='''') and (joStrict in Options) then
=======
        If (C='''') and Strict then
>>>>>>> graemeg/cpstrnew
=======
        If (C='''') and Strict then
>>>>>>> graemeg/cpstrnew
=======
        If (C='''') and Strict then
>>>>>>> graemeg/cpstrnew
=======
        If (C='''') and Strict then
>>>>>>> origin/cpstrnew
          Error(SErrInvalidCharacter, [CurRow,CurColumn,TokenStr[0]]);
=======
    '"':
      begin
>>>>>>> graemeg/fixes_2_2
=======
    '"':
      begin
>>>>>>> origin/fixes_2_2
        Inc(TokenStr);
        TokenStart := TokenStr;
        OldLength := 0;
        FCurTokenString := '';
<<<<<<< HEAD
<<<<<<< HEAD
        while not (TokenStr[0] in [#0,C]) do
=======

        while not (TokenStr[0] in [#0,'"']) do
>>>>>>> graemeg/fixes_2_2
=======

        while not (TokenStr[0] in [#0,'"']) do
>>>>>>> origin/fixes_2_2
          begin
          if (TokenStr[0]='\') then
            begin
            // Save length
            SectionLength := TokenStr - TokenStart;
            Inc(TokenStr);
            // Read escaped token
            Case TokenStr[0] of
              '"' : S:='"';
<<<<<<< HEAD
<<<<<<< HEAD
              '''' : S:='''';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
                        Error(SErrInvalidCharacter, [CurRow,CurColumn,TokenStr[0]]);
                      end;
                      end;
                    // WideChar takes care of conversion...  
                    if (joUTF8 in Options) then
                      S:=Utf8Encode(WideString(WideChar(StrToInt('$'+S))))
                    else
                      S:=WideChar(StrToInt('$'+S));  
                    end;
              #0  : Error(SErrOpenString);
            else
              Error(SErrInvalidCharacter, [CurRow,CurColumn,TokenStr[0]]);
=======
=======
>>>>>>> origin/fixes_2_2
                        Error(SErrInvalidCharacter, [TokenStr[0]]);
                      end;
                      end;
                    // Takes care of conversion...  
                    S:=WideChar(StrToInt('$'+S));  
                    end;
              #0  : Error(SErrOpenString);
            else
              Error(SErrInvalidCharacter, [TokenStr[0]]);  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            end;
            SetLength(FCurTokenString, OldLength + SectionLength+1+Length(S));
            if SectionLength > 0 then
              Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
            Move(S[1],FCurTokenString[OldLength + SectionLength+1],Length(S));
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
        SetLength(FCurTokenString, OldLength + SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[OldLength + 1], SectionLength);
        Inc(TokenStr);
        Result := tkString;
      end;
    ',':
      begin
        Inc(TokenStr);
        Result := tkComma;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
    '0'..'9','.','-':
=======
    '0'..'9','-':
>>>>>>> graemeg/fixes_2_2
=======
    '0'..'9','-':
>>>>>>> origin/fixes_2_2
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
                  until not (TokenStr[0] in ['0'..'9', 'e', 'E','-','+']);
                end;
                break;
              end;
            '0'..'9': ;
            'e', 'E':
              begin
                Inc(TokenStr);
                if TokenStr[0] in ['-','+']  then
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        SetString(FCurTokenString, TokenStart, SectionLength);
=======
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
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
        If (FCurTokenString[1]='.') then
          FCurTokenString:='0'+FCurTokenString;
=======
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
>>>>>>> graemeg/fixes_2_2
=======
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
>>>>>>> origin/fixes_2_2
        Result := tkNumber;
      end;
    ':':
      begin
        Inc(TokenStr);
        Result := tkColon;
      end;
    '{':
      begin
        Inc(TokenStr);
        Result := tkCurlyBraceOpen;
      end;
    '}':
      begin
        Inc(TokenStr);
        Result := tkCurlyBraceClose;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    '/' :
      begin
      if Not (joComments in Options) then
        Error(SErrInvalidCharacter, [CurRow,CurCOlumn,TokenStr[0]]);
      Inc(TokenStr);
      Case Tokenstr[0] of
        '/' : begin
              SectionLength := Length(FCurLine)- (TokenStr - PChar(FCurLine));
              SetString(FCurTokenString, TokenStart, SectionLength);
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
              SetString(S, TokenStart, SectionLength);

              FCurtokenString:=FCurtokenString+S;
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
            SetString(S, TokenStart, SectionLength);
            FCurtokenString:=FCurtokenString+S;
            Inc(TokenStr);
            end;
          end;
      else
        Error(SErrInvalidCharacter, [CurRow,CurCOlumn,TokenStr[0]]);
      end;
      Result:=tkComment;
      end;
    'a'..'z','A'..'Z','_':
=======
    'a'..'z','_':
>>>>>>> graemeg/cpstrnew
=======
    'a'..'z','_':
>>>>>>> graemeg/cpstrnew
=======
    'a'..'z','_':
>>>>>>> graemeg/cpstrnew
=======
    'a'..'z','_':
>>>>>>> origin/cpstrnew
=======
    'T','t','F','f','N','n' :
>>>>>>> graemeg/fixes_2_2
=======
    'T','t','F','f','N','n' :
>>>>>>> origin/fixes_2_2
      begin
        TokenStart := TokenStr;
        repeat
          Inc(TokenStr);
        until not (TokenStr[0] in ['A'..'Z', 'a'..'z', '0'..'9', '_']);
        SectionLength := TokenStr - TokenStart;
<<<<<<< HEAD
<<<<<<< HEAD
        SetString(FCurTokenString, TokenStart, SectionLength);
=======
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
>>>>>>> graemeg/fixes_2_2
=======
        SetLength(FCurTokenString, SectionLength);
        if SectionLength > 0 then
          Move(TokenStart^, FCurTokenString[1], SectionLength);
>>>>>>> origin/fixes_2_2
        for it := tkTrue to tkNull do
          if CompareText(CurTokenString, TokenInfos[it]) = 0 then
            begin
            Result := it;
            FCurToken := Result;
            exit;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (joStrict in Options) then
=======
        if Strict then
>>>>>>> graemeg/cpstrnew
=======
        if Strict then
>>>>>>> graemeg/cpstrnew
=======
        if Strict then
>>>>>>> graemeg/cpstrnew
=======
        if Strict then
>>>>>>> origin/cpstrnew
          Error(SErrInvalidCharacter, [CurRow,CurColumn,TokenStr[0]])
        else
          Result:=tkIdentifier;
      end;
  else
    Error(SErrInvalidCharacter, [CurRow,CurCOlumn,TokenStr[0]]);
=======
=======
>>>>>>> origin/fixes_2_2
        Error(SErrInvalidCharacter, [TokenStart[0]]);
      end;
  else
    Error(SErrInvalidCharacter, [TokenStr[0]]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  FCurToken := Result;
end;

function TJSONScanner.GetCurColumn: Integer;
begin
  Result := TokenStr - PChar(CurLine);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONScanner.GetO(AIndex: TJSONOption): Boolean;
begin
  Result:=AIndex in FOptions;
end;

procedure TJSONScanner.SetO(AIndex: TJSONOption; AValue: Boolean);
begin
  If AValue then
    Include(Foptions,AIndex)
  else
    Exclude(Foptions,AIndex)
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
