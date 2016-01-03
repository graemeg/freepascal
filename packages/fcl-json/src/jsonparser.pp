{
    This file is part of the Free Component Library

    JSON source parser
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit jsonparser;

interface

uses
  Classes, SysUtils, fpJSON, jsonscanner;
  
Type

  { TJSONParser }

  TJSONParser = Class(TObject)
  Private
    FScanner : TJSONScanner;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function GetO(AIndex: TJSONOption): Boolean;
    function GetOptions: TJSONOptions;
    function ParseNumber: TJSONNumber;
    procedure SetO(AIndex: TJSONOption; AValue: Boolean);
    procedure SetOptions(AValue: TJSONOptions);
=======
    FStrict: Boolean;
    function ParseNumber: TJSONNumber;
    procedure SetStrict(const AValue: Boolean);
>>>>>>> graemeg/cpstrnew
=======
    FStrict: Boolean;
    function ParseNumber: TJSONNumber;
    procedure SetStrict(const AValue: Boolean);
>>>>>>> graemeg/cpstrnew
=======
    FStrict: Boolean;
    function ParseNumber: TJSONNumber;
    procedure SetStrict(const AValue: Boolean);
>>>>>>> graemeg/cpstrnew
=======
    FStrict: Boolean;
    function ParseNumber: TJSONNumber;
    procedure SetStrict(const AValue: Boolean);
>>>>>>> origin/cpstrnew
  Protected
    procedure DoError(const Msg: String);
=======
    function ParseNumber: TJSONNumber;
  Protected
    procedure DoError(Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
    function ParseNumber: TJSONNumber;
  Protected
    procedure DoError(Msg: String);
>>>>>>> origin/fixes_2_2
    function DoParse(AtCurrent,AllowEOF: Boolean): TJSONData;
    function GetNextToken: TJSONToken;
    function CurrentTokenString: String;
    function CurrentToken: TJSONToken;
    function ParseArray: TJSONArray;
    function ParseObject: TJSONObject;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Scanner : TJSONScanner read FScanner;
  Public
    function Parse: TJSONData;
    Constructor Create(Source : TStream; AUseUTF8 : Boolean = True); overload;deprecated 'use options form instead';
    Constructor Create(Source : TJSONStringType; AUseUTF8 : Boolean = True); overload;deprecated 'use options form instead';
    constructor Create(Source: TStream; AOptions: TJSONOptions); overload;
    constructor Create(const Source: String; AOptions: TJSONOptions); overload;
    destructor Destroy();override;
    // Use strict JSON: " for strings, object members are strings, not identifiers
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property Strict : Boolean Index joStrict Read GetO Write SetO ; deprecated 'use options instead';
    // if set to TRUE, then strings will be converted to UTF8 ansistrings, not system codepage ansistrings.
    Property UseUTF8 : Boolean index joUTF8 Read GetO Write SetO; deprecated 'Use options instead';
    // Parsing options
    Property Options : TJSONOptions Read GetOptions Write SetOptions;
=======
    Property Strict : Boolean Read FStrict Write SetStrict;
>>>>>>> graemeg/cpstrnew
=======
    Property Strict : Boolean Read FStrict Write SetStrict;
>>>>>>> graemeg/cpstrnew
=======
    Property Strict : Boolean Read FStrict Write SetStrict;
>>>>>>> graemeg/cpstrnew
=======
    Property Strict : Boolean Read FStrict Write SetStrict;
>>>>>>> origin/cpstrnew
  end;
  
  EJSONParser = Class(EParserError);
=======
=======
>>>>>>> origin/fixes_2_2
  Public
    function Parse: TJSONData;
    Constructor Create(Source : TStream); overload;
    Constructor Create(Source : TJSONStringType); overload;
    destructor Destroy();override;
  end;
  
  EJSONScanner = Class(Exception);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  
implementation

Resourcestring
  SErrUnexpectedEOF   = 'Unexpected EOF encountered.';
  SErrUnexpectedToken = 'Unexpected token (%s) encountered.';
  SErrExpectedColon   = 'Expected colon (:), got token "%s".';
  SErrUnexpectedComma = 'Invalid comma encountered.';
  SErrEmptyElement = 'Empty element encountered.';
  SErrExpectedElementName    = 'Expected element name, got token "%s"';
  SExpectedCommaorBraceClose = 'Expected , or ], got token "%s".';
  SErrInvalidNumber          = 'Number is not an integer or real number: %s';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrNoScanner = 'No scanner. No source specified ?';
  
{ TJSONParser }

procedure DefJSONParserHandler(AStream: TStream; const AUseUTF8: Boolean; out
  Data: TJSONData);

Var
  P : TJSONParser;

begin
  Data:=Nil;
  P:=TJSONParser.Create(AStream,[joUTF8]);
  try
    Data:=P.Parse;
  finally
    P.Free;
  end;
end;

function TJSONParser.Parse: TJSONData;

begin
  if (FScanner=Nil) then
    DoError(SErrNoScanner);
=======
=======
>>>>>>> origin/fixes_2_2
  
{ TJSONParser }

Function TJSONParser.Parse : TJSONData;

begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result:=DoParse(False,True);
end;

{
  Consume next token and convert to JSON data structure.
  If AtCurrent is true, the current token is used. If false,
  a token is gotten from the scanner.
  If AllowEOF is false, encountering a tkEOF will result in an exception.
}

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONParser.CurrentToken: TJSONToken;
=======
Function TJSONParser.CurrentToken : TJSONToken;
>>>>>>> graemeg/fixes_2_2
=======
Function TJSONParser.CurrentToken : TJSONToken;
>>>>>>> origin/fixes_2_2

begin
  Result:=FScanner.CurToken;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONParser.CurrentTokenString: String;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  If CurrentToken in [tkString,tkIdentifier,tkNumber,tkComment] then
=======
  If CurrentToken in [tkString,tkIdentifier,tkNumber] then
>>>>>>> graemeg/cpstrnew
=======
  If CurrentToken in [tkString,tkIdentifier,tkNumber] then
>>>>>>> graemeg/cpstrnew
=======
  If CurrentToken in [tkString,tkIdentifier,tkNumber] then
>>>>>>> graemeg/cpstrnew
=======
  If CurrentToken in [tkString,tkIdentifier,tkNumber] then
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
Function TJSONParser.CurrentTokenString : String;

begin
  If CurrentToken in [tkString,tkNumber] then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Result:=FScanner.CurTokenString
  else
    Result:=TokenInfos[CurrentToken];
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONParser.DoParse(AtCurrent, AllowEOF: Boolean): TJSONData;
=======
Function TJSONParser.DoParse(AtCurrent,AllowEOF : Boolean) : TJSONData;
>>>>>>> graemeg/fixes_2_2
=======
Function TJSONParser.DoParse(AtCurrent,AllowEOF : Boolean) : TJSONData;
>>>>>>> origin/fixes_2_2

var
  T : TJSONToken;
  
begin
  Result:=nil;
  try
    If not AtCurrent then
      T:=GetNextToken
    else
      T:=FScanner.CurToken;
    Case T of
      tkEof : If Not AllowEof then
                DoError(SErrUnexpectedEOF);
<<<<<<< HEAD
<<<<<<< HEAD
      tkNull  : Result:=CreateJSON;
      tkTrue,
      tkFalse : Result:=CreateJSON(t=tkTrue);
      tkString : if joUTF8 in Options then
                   Result:=CreateJSON(UTF8Decode(CurrentTokenString))
                     else
                       Result:=CreateJSON(CurrentTokenString);
=======
=======
>>>>>>> origin/fixes_2_2
      tkNull  : Result:=TJSONNull.Create;
      tkTrue,
      tkFalse : Result:=TJSONBoolean.Create(t=tkTrue);
      tkString : Result:=TJSONString.Create(CurrentTokenString);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      tkCurlyBraceOpen : Result:=ParseObject;
      tkCurlyBraceClose : DoError(SErrUnexpectedToken);
      tkSQuaredBraceOpen : Result:=ParseArray;
      tkSQuaredBraceClose : DoError(SErrUnexpectedToken);
      tkNumber : Result:=ParseNumber;
      tkComma : DoError(SErrUnexpectedToken);
    end;
  except
<<<<<<< HEAD
<<<<<<< HEAD
    FreeAndNil(Result);
=======
    if assigned(Result) then
      FreeAndNil(Result);
>>>>>>> graemeg/fixes_2_2
=======
    if assigned(Result) then
      FreeAndNil(Result);
>>>>>>> origin/fixes_2_2
    Raise;
  end;
end;


// Creates the correct JSON number type, based on the current token.
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONParser.ParseNumber: TJSONNumber;

Var
  I : Integer;
  I64 : Int64;
  QW  : QWord;
=======
=======
>>>>>>> origin/fixes_2_2
Function TJSONParser.ParseNumber : TJSONNumber;

Var
  I : Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  F : TJSONFloat;
  S : String;

begin
  S:=CurrentTokenString;
  I:=0;
<<<<<<< HEAD
<<<<<<< HEAD
  if TryStrToQWord(S,QW) then
    begin
    if QW>qword(high(Int64)) then
      Result:=CreateJSON(QW)
    else
      if QW>MaxInt then
      begin
        I64 := QW;
        Result:=CreateJSON(I64);
      end
      else
      begin
        I := QW;
        Result:=CreateJSON(I);
      end
    end
  else
    begin
    If TryStrToInt64(S,I64) then
      if (I64>Maxint) or (I64<-MaxInt) then
        Result:=CreateJSON(I64)
      Else
        begin
        I:=I64;
        Result:=CreateJSON(I);
        end
    else
      begin
      I:=0;
      Val(S,F,I);
      If (I<>0) then
        DoError(SErrInvalidNumber);
      Result:=CreateJSON(F);
      end;
    end;

end;

function TJSONParser.GetO(AIndex: TJSONOption): Boolean;
begin
  Result:=AIndex in Options;
end;

<<<<<<< HEAD
function TJSONParser.GetOptions: TJSONOptions;
begin
  Result:=FScanner.Options
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONParser.SetO(AIndex: TJSONOption; AValue: Boolean);
begin
  if aValue then
    FScanner.Options:=FScanner.Options+[AINdex]
  else
    FScanner.Options:=FScanner.Options-[AINdex]
end;

procedure TJSONParser.SetOptions(AValue: TJSONOptions);
begin
  FScanner.Options:=AValue;
end;

<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TJSONParser.SetStrict(const AValue: Boolean);
begin
  if (FStrict=AValue) then
     exit;
  FStrict:=AValue;
  If Assigned(FScanner) then
    FScanner.Strict:=Fstrict;
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

// Current token is {, on exit current token is }
function TJSONParser.ParseObject: TJSONObject;
=======
=======
>>>>>>> origin/fixes_2_2
  If TryStrToInt(S,I) then
    Result:=TJSONIntegerNumber.Create(I)
  else
    begin
    I:=0;
    Val(S,F,I);
    If (I<>0) then
      DoError(SErrInvalidNumber);
    Result:=TJSONFloatNumber.Create(F);
    end;
end;

// Current token is {, on exit current token is }
Function TJSONParser.ParseObject : TJSONObject;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Var
  T : TJSONtoken;
  E : TJSONData;
  N : String;
  
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=CreateJSONObject([]);
=======
  Result:=TJSONObject.Create;
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TJSONObject.Create;
>>>>>>> origin/fixes_2_2
  Try
    T:=GetNextToken;
    While T<>tkCurlyBraceClose do
      begin
<<<<<<< HEAD
<<<<<<< HEAD
      If (T<>tkString) and (T<>tkIdentifier) then
=======
      If T<>tkString then
>>>>>>> graemeg/fixes_2_2
=======
      If T<>tkString then
>>>>>>> origin/fixes_2_2
        DoError(SErrExpectedElementName);
      N:=CurrentTokenString;
      T:=GetNextToken;
      If (T<>tkColon) then
        DoError(SErrExpectedColon);
      E:=DoParse(False,False);
      Result.Add(N,E);
      T:=GetNextToken;
      If Not (T in [tkComma,tkCurlyBraceClose]) then
        DoError(SExpectedCommaorBraceClose);
      If T=tkComma then
        T:=GetNextToken;
      end;
  Except
    FreeAndNil(Result);
    Raise;
  end;
end;

// Current token is [, on exit current token is ]
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONParser.ParseArray: TJSONArray;
=======
Function TJSONParser.ParseArray : TJSONArray;
>>>>>>> graemeg/fixes_2_2
=======
Function TJSONParser.ParseArray : TJSONArray;
>>>>>>> origin/fixes_2_2

Var
  T : TJSONtoken;
  E : TJSONData;
  LastComma : Boolean;
  
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=CreateJSONArray([]);
=======
  Result:=TJSONArray.Create;
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TJSONArray.Create;
>>>>>>> origin/fixes_2_2
  LastComma:=False;
  Try
    Repeat
      T:=GetNextToken;
      If (T<>tkSquaredBraceClose) then
        begin
        E:=DoParse(True,False);
        If (E<>Nil) then
          Result.Add(E)
        else if (Result.Count>0) then
          DoError(SErrEmptyElement);
        T:=GetNextToken;
        If Not (T in [tkComma,tkSquaredBraceClose]) then
          DoError(SExpectedCommaorBraceClose);
        LastComma:=(t=TkComma);
        end;
    Until (T=tkSquaredBraceClose);
    If LastComma then // Test for ,] case
      DoError(SErrUnExpectedToken);
  Except
    FreeAndNil(Result);
    Raise;
  end;
end;

// Get next token, discarding whitespace
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONParser.GetNextToken: TJSONToken;
=======
Function TJSONParser.GetNextToken : TJSONToken ;
>>>>>>> graemeg/fixes_2_2
=======
Function TJSONParser.GetNextToken : TJSONToken ;
>>>>>>> origin/fixes_2_2

begin
  Repeat
    Result:=FScanner.FetchToken;
<<<<<<< HEAD
<<<<<<< HEAD
  Until (Not (Result in [tkComment,tkWhiteSpace]));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONParser.DoError(const Msg: String);
=======
Procedure TJSONParser.DoError(const Msg : String);
>>>>>>> graemeg/cpstrnew
=======
Procedure TJSONParser.DoError(const Msg : String);
>>>>>>> graemeg/cpstrnew
=======
Procedure TJSONParser.DoError(const Msg : String);
>>>>>>> graemeg/cpstrnew
=======
Procedure TJSONParser.DoError(const Msg : String);
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  Until (Result<>tkWhiteSpace);
end;

Procedure TJSONParser.DoError(Msg : String);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Var
  S : String;

begin
  S:=Format(Msg,[CurrentTokenString]);
  S:=Format('Error at line %d, Pos %d:',[FScanner.CurRow,FSCanner.CurColumn])+S;
<<<<<<< HEAD
<<<<<<< HEAD
  Raise EJSONParser.Create(S);
end;

constructor TJSONParser.Create(Source: TStream; AUseUTF8 : Boolean = True);
begin
  Inherited Create;
  FScanner:=TJSONScanner.Create(Source,[joUTF8]);
  if AUseUTF8 then
   Options:=Options + [joUTF8];
end;

constructor TJSONParser.Create(Source: TJSONStringType; AUseUTF8 : Boolean = True);
begin
  Inherited Create;
  FScanner:=TJSONScanner.Create(Source,[joUTF8]);
  if AUseUTF8 then
   Options:=Options + [joUTF8];
end;

constructor TJSONParser.Create(Source: TStream; AOptions: TJSONOptions);
begin
  FScanner:=TJSONScanner.Create(Source,AOptions);
end;

constructor TJSONParser.Create(const Source: String; AOptions: TJSONOptions);
begin
  FScanner:=TJSONScanner.Create(Source,AOptions);
=======
=======
>>>>>>> origin/fixes_2_2
  Raise EJSONScanner.Create(S);
end;

constructor TJSONParser.Create(Source: TStream);
begin
  Inherited Create;
  FScanner:=TJSONScanner.Create(Source);
end;

constructor TJSONParser.Create(Source: TJSONStringType);
begin
  Inherited Create;
  FScanner:=TJSONScanner.Create(Source);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TJSONParser.Destroy();
begin
  FreeAndNil(FScanner);
  inherited Destroy();
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure InitJSONHandler;

begin
  if GetJSONParserHandler=Nil then
    SetJSONParserHandler(@DefJSONParserHandler);
end;

Procedure DoneJSONHandler;

begin
  if GetJSONParserHandler=@DefJSONParserHandler then
    SetJSONParserHandler(Nil);
end;

initialization
  InitJSONHandler;
finalization
  DoneJSONHandler;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

