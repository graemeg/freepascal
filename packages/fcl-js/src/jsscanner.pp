{
    This file is part of the Free Component Library

    ECMAScript (JavaScript) source lexical scanner

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$h+}

unit JSScanner;

interface

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses SysUtils, Classes, jstoken;
=======
uses SysUtils, Classes;
>>>>>>> graemeg/cpstrnew
=======
uses SysUtils, Classes;
>>>>>>> graemeg/cpstrnew
=======
uses SysUtils, Classes;
>>>>>>> graemeg/cpstrnew
=======
uses SysUtils, Classes;
>>>>>>> origin/cpstrnew

resourcestring
  SErrInvalidCharacter = 'Invalid character ''%s''';
  SErrOpenString = 'string exceeds end of line';
  SErrIncludeFileNotFound = 'Could not find include file ''%s''';
  SErrIfXXXNestingLimitReached = 'Nesting of $IFxxx too deep';
  SErrInvalidPPElse = '$ELSE without matching $IFxxx';
  SErrInvalidPPEndif = '$ENDIF without matching $IFxxx';
  SInvalidHexadecimalNumber = 'Invalid decimal number';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrInvalidNonEqual = 'Syntax Error: != or !== expected';
  SErrInvalidRegularExpression = 'Syntax error in regular expression: / expected, got: %s';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SErrInvalidNonEqual = 'SyntaxError: != or !== expected';

type

   TJSToken = (tjsUnknown,
   // Specials
   tjsEOF,tjsWhiteSpace,tjsChar,tjsString, tjsIdentifier,tjsNumber, tjsComment,tjsREGEX, tjsRESERVED,
   tjsANDAND, tjsANDEQ,
   tjsBraceOpen,tjsBraceClose,tjsSQuaredBraceOpen,tjsSQuaredBraceClose,tjsCurlyBraceOpen,tjsCurlyBraceClose,
   tjsCOMMA,tjsCOLON,  tjsDOT,tjsSEMICOLON, tjsASSIGN,tjsGT,tjsLT, tjsConditional,
   tjsPLUS,tjsMINUS,tjsMUL,tjsDIV,tjsAnd,tjsOR, tjsInv, tjsMod, tjsXOR, tjsNot,
   tjsEQ,
   tjsGE,
   tjsLE, tjsLSHIFT, tjsLSHIFTEQ,
   tjsMINUSEQ, tjsMINUSMINUS, tjsMODEQ,tjsDIVEQ,tjsXOREq,
   tjsNE,
   tjsOREQ, tjsOROR,
   tjsPLUSEQ, tjsPLUSPLUS,
   tjsURSHIFT, tjsURSHIFTEQ,
   tjsRSHIFT, tjsRSHIFTEQ,
   tjsSEQ, tjsSNE, tjsMULEQ,
   { Reserved words start here. They must be last }
   tjsBREAK,tjsCASE, tjsCATCH, tjsCONTINUE,
   tjsDEFAULT, tjsDELETE, tjsDO,
   tjsELSE,
   tjsFalse, tjsFINALLY, tjsFOR, tjsFUNCTION,
   tjsIF, tjsIN, tjsINSTANCEOF,
   tjsNEW,tjsNULL,
   tjsRETURN,
   tjsSWITCH,
   tjsTHIS, tjsTHROW, tjsTrue, tjsTRY, tjsTYPEOF,
   tjsVAR, tjsVOID,
   tjsWHILE, tjsWITH
 );

const
  FirstKeyword = tjsBreak;
  LastKeyWord = tJSWith;

  TokenInfos: array[TJSToken] of string = ('unknown',
     // Specials
      'EOF','whitespace','Char','String', 'identifier','number','comment','regular expression', 'reserved word',
      '&&','&=',
      '(',')','[',']','{','}',
      ',',':','.',';','=','>','<','?',
      '+','-','*','/','&','|','~','%','^','!',
      '==',
      '>=',
      '<=', '<<', '<<=',
      '-=', '--', '%=', '/=','^=',
      '!=',
      '|=', '||',
      '+=', '++',
      '>>>', '>>>=',
      '>>', '>>=',
      '===', '!==', '*=',
      // Identifiers last
      'break','case','catch', 'continue',
   'default','delete', 'do',
   'else',
   'false','finally', 'for', 'function',
   'if', 'in', 'instanceof',
   'new','null',
   'return',
   'switch',
   'this', 'throw', 'true', 'try', 'typeof',
   'var', 'void',
   'while', 'with'
  );
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

Type
  TLineReader = class
  public
    function IsEOF: Boolean; virtual; abstract;
    function ReadLine: string; virtual; abstract;
  end;

  { TStreamLineReader }

  TStreamLineReader = class(TLineReader)
  private
    FStream : TStream;
    Buffer : Array[0..1024] of Byte;
    FBufPos,
    FBufLen : Integer;
    procedure FillBuffer;
  public
    Constructor Create(AStream : TStream);
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
  end;

  TFileLineReader = class(TLineReader)
  private
    FTextFile: Text;
    FileOpened: Boolean;
  public
    constructor Create(const AFilename: string);
    destructor Destroy; override;
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
  end;

  EJSScannerError       = class(Exception);

  { TJSScanner }

  TJSScanner = class
  private
    FReturnComments: Boolean;
    FReturnWhiteSpace: Boolean;
    FSourceFile: TLineReader;
    FSourceFilename: string;
    FCurRow: Integer;
    FCurToken: TJSToken;
    FCurTokenString: string;
    FCurLine: string;
    FDefines: TStrings;
    TokenStr: PChar;
    FWasEndOfLine : Boolean;
    FSourceStream : TStream;
    FOwnSourceFile : Boolean;
    function CommentDiv: TJSToken;
    function DoIdentifier : TJSToken;
    function DoMultiLineComment: TJSToken;
    function DoNumericLiteral: TJSToken;
    function DoSingleLineComment: TJSToken;
    function DoStringLiteral: TJSToken;
    function DoWhiteSpace: TJSToken;
    function FetchLine: Boolean;
    function GetCurColumn: Integer;
    function ReadUnicodeEscape: WideChar;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function ReadRegex : TJSToken;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  protected
    procedure Error(const Msg: string);overload;
    procedure Error(const Msg: string; Args: array of Const);overload;
  public
    constructor Create(ALineReader: TLineReader);
    constructor Create(AStream : TStream);
    destructor Destroy; override;
    procedure OpenFile(const AFilename: string);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function FetchRegexprToken: TJSToken;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Function FetchToken: TJSToken;
    Function IsEndOfLine : Boolean;
    Property WasEndOfLine : Boolean Read FWasEndOfLine;
    Property ReturnComments : Boolean Read FReturnComments Write FReturnComments;
    Property ReturnWhiteSpace : Boolean Read FReturnWhiteSpace Write FReturnWhiteSpace;
    property SourceFile: TLineReader read FSourceFile;
    property CurFilename: string read FSourceFilename;
    property CurLine: string read FCurLine;
    property CurRow: Integer read FCurRow;
    property CurColumn: Integer read GetCurColumn;
    property CurToken: TJSToken read FCurToken;
    property CurTokenString: string read FCurTokenString;
  end;


implementation


constructor TFileLineReader.Create(const AFilename: string);
begin
  inherited Create;
  Assign(FTextFile, AFilename);
  Reset(FTextFile);
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

constructor TJSScanner.Create(ALineReader: TLineReader);
begin
  inherited Create;
  FSourceFile := ALineReader;
end;

constructor TJSScanner.Create(AStream: TStream);
begin
  FSourceStream:=ASTream;
  FOwnSourceFile:=True;
  Create(TStreamLineReader.Create(AStream));
end;

destructor TJSScanner.Destroy;
begin
  If FOwnSourceFile then
    FSourceFile.Free;
  inherited Destroy;
end;

procedure TJSScanner.OpenFile(const AFilename: string);
begin
  FSourceFile := TFileLineReader.Create(AFilename);
  FSourceFilename := AFilename;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TJSScanner.FetchRegexprToken: TJSToken;
begin
  if (CurToken in [tjsDiv,tjsDivEq]) then
    Result:=ReadRegEx
  else
    Result:=CurToken
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure TJSScanner.Error(const Msg: string);
begin
  raise EJSScannerError.Create(Msg);
end;

procedure TJSScanner.Error(const Msg: string; Args: array of Const);
begin
  raise EJSScannerError.CreateFmt(Msg, Args);
end;

function TJSScanner.FetchLine: Boolean;
begin
  if FSourceFile.IsEOF then
  begin
    FCurLine := '';
    TokenStr := nil;
    Result := false;
  end else
  begin
    FCurLine := FSourceFile.ReadLine;
    TokenStr := PChar(CurLine);
    Result := true;
    Inc(FCurRow);
    FWasEndofLine:=True;
  end;
end;

function TJSScanner.DoWhiteSpace : TJSToken;

begin
  Result:=tjsWhitespace;
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

function TJSScanner.DoSingleLineComment : TJSToken;

Var
  TokenStart : PChar;
  Len : Integer;

begin
  Inc(TokenStr);
  TokenStart := TokenStr;
  while TokenStr[0] <> #0 do
     Inc(TokenStr);
  Len:=TokenStr-TokenStart;
  SetLength(FCurTokenString, Len);
  if (Len>0) then
    Move(TokenStart^,FCurTokenString[1],Len);
  Result := tjsComment;
end;

function TJSScanner.DoMultiLineComment : TJSToken;

Var
  TokenStart : PChar;
  Len,OLen : Integer;
  PrevToken : Char;

begin
  Inc(TokenStr);
  TokenStart := TokenStr;
  FCurTokenString := '';
  OLen:= 0;
  PrevToken:=#0;
  while Not ((TokenStr[0]='/') and (PrevToken='*')) do
    begin
    if (TokenStr[0]=#0) then
      begin
      Len:=TokenStr-TokenStart+1;
      SetLength(FCurTokenString,OLen+Len);
      if Len>1 then
        Move(TokenStart^,FCurTokenString[OLen+1],Len-1);
      Inc(OLen,Len);
      FCurTokenString[OLen]:=#10;
      if not FetchLine then
        begin
        Result := tjsEOF;
        FCurToken := Result;
        exit;
        end;
      TokenStart := TokenStr;
      PrevToken:=#0;
      end
    else
      begin
      PrevToken:=TokenStr[0];
      Inc(TokenStr);
      end;
    end;
  Len:=TokenStr-TokenStart-1; // -1 for *
  SetLength(FCurTokenString, Olen+Len);
  if (Len>0) then
    Move(TokenStart^, FCurTokenString[Olen + 1], Len);
  Inc(TokenStr);
  Result := tjsComment;
end;

function TJSScanner.CommentDiv : TJSToken;

begin
  FCurTokenString := '';
  Inc(TokenStr);
  if (TokenStr[0] = '/') then       // Single-line comment
    Result:=DoSingleLineComment
  else if (TokenStr[0]='*') then
    Result:=DoMultiLineComment
  else if (TokenStr[0] = '=') then       // Single-line comment
    begin
    Result:=tjsDiveQ;
    Inc(TokenStr)
    end
  else
    Result:=tjsDiv;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSScanner.ReadUnicodeEscape: WideChar;
=======
Function TJSScanner.ReadUnicodeEscape : WideChar;
>>>>>>> graemeg/cpstrnew
=======
Function TJSScanner.ReadUnicodeEscape : WideChar;
>>>>>>> graemeg/cpstrnew
=======
Function TJSScanner.ReadUnicodeEscape : WideChar;
>>>>>>> graemeg/cpstrnew
=======
Function TJSScanner.ReadUnicodeEscape : WideChar;
>>>>>>> origin/cpstrnew

Var
  S : String;
  I : Integer;

begin
  S:='0000';
  For I:=1 to 4 do
    begin
    Inc(TokenStr);
    Case TokenStr[0] of
      '0'..'9','A'..'F','a'..'f' :
        S[i]:=Upcase(TokenStr[0]);
    else
      Error(SErrInvalidCharacter, [TokenStr[0]]);
    end;
    end;
  // Takes care of conversion... This needs improvement !!
  Result:=WideChar(StrToInt('$'+S));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TJSScanner.ReadRegex: TJSToken;

Var
  CC : Boolean; // Character class
  Done : Boolean;
  CL,L : Integer;
  TokenStart : PChar;

begin
  if (CurToken<>tjsDivEq) then
    FCurTokenString := '/'
  else
    FCurTokenString := '/=';
  CL:=Length(FCurTokenString);
  Inc(TokenStr);
  TokenStart:=TokenStr;
  Done:=False;
  CC:=False;
  While Not Done do
    begin
    Case TokenStr[0] of
      #0 : Done:=True;
      '/' : Done:=Not CC;
      '\' : begin
            Inc(TokenStr);
            Done:=TokenStr=#0;
            end;
      '[' : CC:=True;
      ']' : CC:=False;
    end;
    if not Done then
      Inc(TokenStr);
    end;
  If (TokenStr[0]<>'/') then
    Error(SErrInvalidRegularExpression, [TokenStr[0]]);
  repeat
    Inc(TokenStr);
  until not (TokenStr[0] in ['A'..'Z', 'a'..'z', '0'..'9', '_','$']);
  L:=(TokenStr-TokenStart);
  SetLength(FCurTokenString,CL+L);
  Move(TokenStart^,FCurTokenString[CL+1],L);
  Result:=tjsRegEx;
end;

function TJSScanner.DoStringLiteral: TJSToken;
=======
Function TJSScanner.DoStringLiteral : TJSToken;
>>>>>>> graemeg/cpstrnew
=======
Function TJSScanner.DoStringLiteral : TJSToken;
>>>>>>> graemeg/cpstrnew
=======
Function TJSScanner.DoStringLiteral : TJSToken;
>>>>>>> graemeg/cpstrnew
=======
Function TJSScanner.DoStringLiteral : TJSToken;
>>>>>>> origin/cpstrnew

Var
  Delim : Char;
  TokenStart : PChar;
  Len,OLen,I : Integer;
  S : String;

begin
  Delim:=TokenStr[0];
  Inc(TokenStr);
  TokenStart := TokenStr;
  OLen := 0;
  FCurTokenString := '';
  while not (TokenStr[0] in [#0,Delim]) do
    begin
    if (TokenStr[0]='\') then
      begin
      // Save length
      Len := TokenStr - TokenStart;
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
              S:=ReadUniCodeEscape;
              end;
        #0  : Error(SErrOpenString);
      else
        Error(SErrInvalidCharacter, [TokenStr[0]]);
      end;
      SetLength(FCurTokenString, OLen + Len+1+Length(S));
      if Len > 0 then
        Move(TokenStart^, FCurTokenString[OLen + 1], Len);
      Move(S[1],FCurTokenString[OLen + Len+1],Length(S));
      Inc(OLen, Len+Length(S));
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
  Len := TokenStr - TokenStart;
  SetLength(FCurTokenString, OLen + Len);
  if Len > 0 then
    Move(TokenStart^, FCurTokenString[OLen+1], Len);
  Inc(TokenStr);
  Result := tjsString;
end;

function TJSScanner.DoNumericLiteral :TJSToken;

Var
  TokenStart : PChar;
  Len : Integer;

begin
  TokenStart := TokenStr;
  while true do
    begin
    Inc(TokenStr);
    case TokenStr[0] of
      'x':
        If (TokenStart[0]='0') and ((TokenStr-TokenStart)=1) then
          begin
          Inc(TokenStr);
          while Upcase(TokenStr[0]) in ['0'..'9','A'..'F'] do
            Inc(TokenStr);
          end
        else
          Error(SInvalidHexadecimalNumber);
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
  Len:=TokenStr-TokenStart;
  Setlength(FCurTokenString, Len);
  if (Len>0) then
  Move(TokenStart^,FCurTokenString[1],Len);
  Result := tjsNumber;
end;

function TJSScanner.DoIdentifier : TJSToken;

Var
  TokenStart:PChar;
  Len : Integer;
  I : TJSToken;

begin
  Result:=tjsIdentifier;
  TokenStart := TokenStr;
  repeat
    Inc(TokenStr);
    If (TokenStr[0]='\') and (TokenStr[1]='u') then
  until not (TokenStr[0] in ['A'..'Z', 'a'..'z', '0'..'9', '_','$']);
  Len:=(TokenStr-TokenStart);
  SetLength(FCurTokenString,Len);
  if Len > 0 then
    Move(TokenStart^,FCurTokenString[1],Len);
 // Check if this is a keyword or identifier
 // !!!: Optimize this!
  I:=FirstKeyword;
  While (Result=tjsIdentifier) and (I<=Lastkeyword) do
    begin
    if (CurTokenString=TokenInfos[i]) then
      begin
      Result := i;
      FCurToken := Result;
      exit;
      end;
    I:=Succ(I);
    end
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TJSScanner.FetchToken: TJSToken;
=======
function TJSScanner.FetchToken: TJSToken;
>>>>>>> graemeg/cpstrnew
=======
function TJSScanner.FetchToken: TJSToken;
>>>>>>> graemeg/cpstrnew
=======
function TJSScanner.FetchToken: TJSToken;
>>>>>>> graemeg/cpstrnew
=======
function TJSScanner.FetchToken: TJSToken;
>>>>>>> origin/cpstrnew


var
  TokenStart, CurPos: PChar;
  i: TJSToken;
  OldLength, SectionLength, NestingLevel, Index: Integer;

begin
  if not (FCurtoken in [tjsWhiteSpace,tjsComment]) then
    FWasEndOfLine:=False;
  Repeat
    if TokenStr = nil then
      begin
      if not FetchLine then
        begin
        Result := tjsEOF;
        FCurToken := Result;
        exit;
        end;
      end;
    CurPos:=TokenStr;
    FCurTokenString := '';
    case TokenStr[0] of
      #0:         // Empty line
        begin
        FetchLine;
        Result := tjsWhitespace;
        end;
      '/' :
         Result:=CommentDiv;
      #9, ' ':
         Result := DoWhiteSpace;
      '''','"':
         Result:=DoStringLiteral;
      '0'..'9':
         Result:=DoNumericLiteral;
     '&':
         begin
         Inc(TokenStr);
         If Tokenstr[0]='&' then
           begin
           Inc(TokenStr);
           Result := tjsAndAnd;
           end
         else If Tokenstr[0]='=' then
           begin
           Inc(TokenStr);
           Result := tjsAndEQ;
           end
         else
           Result := tjsAnd;
         end;
     '%':
         begin
         Inc(TokenStr);
         If Tokenstr[0]='=' then
           begin
           Inc(TokenStr);
           Result := tjsModEq;
           end
         else
           Result := tjsMod;
         end;
     '^':
         begin
         Inc(TokenStr);
         If (TokenStr[0]='=') then
           begin
           Result:=tjsXorEq;
           Inc(tokenStr)
           end
         else
           result:=tjsXOR;
         end;
     '|':
         begin
         Inc(TokenStr);
         If Tokenstr[0]='|' then
           begin
           Inc(TokenStr);
           Result := tjsOROR;
           end
         else If Tokenstr[0]='=' then
             begin
             Inc(TokenStr);
             Result := tjsOREQ;
             end
         else
           Result := tjsOR;
         end;
    '(':
      begin
      Inc(TokenStr);
      Result := tjsBraceOpen;
      end;
    ')':
      begin
      Inc(TokenStr);
      Result := tjsBraceClose;
      end;
    '*':
      begin
      Inc(TokenStr);
      If (TokenStr[0]='=') then
        begin
        Inc(TokenStr);
        Result := tjsMulEq;
        end
      else
        Result := tjsMul;
      end;
    '+':
      begin
      Inc(TokenStr);
      If (TokenStr[0]='=') then
        begin
        Inc(TokenStr);
        Result := tjsPlusEq;
        end
      else If (TokenStr[0]='+') then
        begin
        Inc(TokenStr);
        Result := tjsPlusPlus;
        end
      else
        Result := tjsPlus;
      end;
    ',':
      begin
        Inc(TokenStr);
        Result := tjsComma;
      end;
    '-':
      begin
      Inc(TokenStr);
      If (TokenStr[0]='=') then
        begin
        Inc(TokenStr);
        Result:=tjsMinusEq
        end
      else If (TokenStr[0]='-') then
        begin
        Inc(TokenStr);
        Result:=tjsMinusMinus
        end
      else if (TokenStr[0] in ['0'..'9']) then
        begin
        Result:=DoNumericLiteral;
        If (Result=tjsNumber) then
          FCurTokenString:='-'+FCurTokenString;
        end
      else
        Result := tjsMinus;
      end;
    '.':
      begin
      Inc(TokenStr);
      if (TokenStr[0] in ['0'..'9']) then
        begin
        Result:=DoNumericLiteral;
        If (Result=tjsNumber) then
          FCurTokenString:='0.'+FCurTokenString;
         end
      else
        Result := tjsDot;
      end;
    ':':
      begin
      Inc(TokenStr);
      Result := tjsColon;
      end;
    '?':
      begin
      Inc(TokenStr);
      Result := tjsConditional;
      end;
    ';':
      begin
      Inc(TokenStr);
      Result := tjsSemicolon;
      end;
    '<':
      begin
      Inc(TokenStr);
      if TokenStr[0] = '=' then
        begin
        Inc(TokenStr);
        Result := tjsLE;
        end
      else if TokenStr[0] = '<' then
        begin
        Inc(TokenStr);
        if (TokenStr[0] = '=') then
          begin
          Inc(TokenStr);
          Result := tjsLShiftEq;
          end
        else
          Result := tjsLShift;
        end
      else
        Result := tjsLT;
      end;
    '=':
      begin
      Inc(TokenStr);
      if (TokenStr[0]='=') then
        begin
        Inc(TokenStr);
        If (TokenStr[0]='=') then
          begin
          Inc(TokenStr);
          Result:=tjsSEQ;
          end
        else
          Result:=tjsEQ;
        end
      else
        Result := tjsAssign;
      end;
    '!':
      begin
      Inc(TokenStr);
      if (TokenStr[0]='=') then
        begin
        Inc(TokenStr);
        If (TokenStr[0]='=') then
          begin
          Inc(TokenStr);
          Result:=tjsSNE;
          end
        else
          Result:=tjsNE;
        end
      else
        Result:=tjsNot;// Error(SErrInvalidNonEqual);
      end;
    '~':
      begin
      Inc(TokenStr);
      Result:=tjsInv;
      end;
    '>':
      begin
      Inc(TokenStr);
      if TokenStr[0] = '=' then
	begin
        Inc(TokenStr);
        Result:=tjsGE;
        end
      else if TokenStr[0] = '>' then
  	begin
        Inc(TokenStr);
        if (TokenStr[0] = '>') then
       	  begin
          Inc(TokenStr);
          if (TokenStr[0] = '=') then
            begin
            Inc(TokenStr);
            Result:=tjsURSHIFTEQ;
            end
          else
            Result:=tjsURSHIFT;
          end
        else if (TokenStr[0] = '=') then
          begin
          Inc(TokenStr);
          Result:=tjsRSHIFTEq;
          end
        else
          Result:=tjsRSHIFT;
        end
      else
        Result := tjsGT;
      end;
    '[':
      begin
      Inc(TokenStr);
      Result := tJSSquaredBraceOpen;
      end;
    ']':
      begin
      Inc(TokenStr);
      Result := tJSSquaredBraceClose;
      end;
    '{':
      begin
      Inc(TokenStr);
      Result := tJSCurlyBraceOpen;
      end;
    '}':
      begin
      Inc(TokenStr);
      Result := tJSCurlyBraceClose;
      end;
   else
     Result:=DoIdentifier;
   end; // Case
  Until (Not (Result in [tjsComment,tjsWhitespace])) or
        ((Result=tjsComment) and ReturnComments) or
        ((Result=tjsWhiteSpace) and ReturnWhiteSpace);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TJSScanner.IsEndOfLine: Boolean;
=======
function TJSScanner.IsEndOfLine: Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSScanner.IsEndOfLine: Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSScanner.IsEndOfLine: Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSScanner.IsEndOfLine: Boolean;
>>>>>>> origin/cpstrnew
begin
  Result:=(TokenStr=Nil) or (TokenStr[0] in [#0,#10,#13]);
end;

function TJSScanner.GetCurColumn: Integer;
begin
  Result := TokenStr - PChar(CurLine);
end;

{ TStreamLineReader }

constructor TStreamLineReader.Create(AStream: TStream);
begin
  FStream:=AStream;
  FBufPos:=0;
  FBufLen:=0;
end;

function TStreamLineReader.IsEOF: Boolean;
begin
  Result:=(FBufPos>=FBufLen);
  If Result then
    begin
    FillBuffer;
    Result:=(FBufLen=0);
    end;
end;

procedure TStreamLineReader.FillBuffer;

begin
  FBufLen:=FStream.Read(Buffer,SizeOf(Buffer)-1);
  Buffer[FBufLen]:=0;
  FBufPos:=0;
end;

function TStreamLineReader.ReadLine: string;

Var
  FPos,OLen,Len: Integer;
  PRun : PByte;

begin
  FPos:=FBufPos;
  SetLength(Result,0);
  Repeat
    PRun:=@Buffer[FBufPos];
    While (FBufPos<FBufLen) and Not (PRun^ in [10,13]) do
      begin
      Inc(PRun);
      Inc(FBufPos);
      end;
    If (FBufPos=FBufLen) then
      begin
      Len:=FBufPos-FPos;
      If (Len>0) then
        begin
        Olen:=Length(Result);
        SetLength(Result,OLen+Len);
        Move(Buffer[FPos],Result[OLen+1],Len);
        end;
      FillBuffer;
      FPos:=FBufPos;
      end;
  until (FBufPos=FBufLen) or (PRun^ in [10,13]);
  Len:=FBufPos-FPos;
  If (Len>0) then
    begin
    Olen:=Length(Result);
    SetLength(Result,OLen+Len);
    Move(Buffer[FPos],Result[OLen+1],Len)
    end;
  If (PRun^ in [10,13]) and (FBufPos<FBufLen) then
    begin
    Inc(FBufPos);
    // Check #13#10
    If (PRun^=13) then
      begin
      If (FBufPos=FBufLen) then
        FillBuffer;
      If (FBufPos<FBufLen) and (Buffer[FBufpos]=10) then
        Inc(FBufPos);
      end;
    end;
end;

end.
