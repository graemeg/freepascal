{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2007 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************}
{*                             TParser                                      *}
{****************************************************************************}

const
  ParseBufSize     = 4096;
  LastSpecialToken = 5;

  TokNames : array[0..LastSpecialToken] of string =
  (
    'EOF',
    'Symbol',
    'String',
    'Integer',
    'Float',
    'WideString'
  );

function TParser.GetTokenName(aTok: char): string;
begin
  if ord(aTok) <= LastSpecialToken then
    Result:=TokNames[ord(aTok)]
  else Result:=aTok;
end;

procedure TParser.LoadBuffer;
var
  BytesRead: integer;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  BytesRead := FStream.Read(FBuf^, ParseBufSize);
  FBuf[BytesRead] := #0;
  Inc(FDeltaPos, BytesRead);
  FPos := 0;
  FBufLen := BytesRead;
  FEofReached:=BytesRead = 0;
end;

procedure TParser.CheckLoadBuffer; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  if fBuf[fPos]=#0 then LoadBuffer;
end;

procedure TParser.ProcessChar; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  fLastTokenStr:=fLastTokenStr+fBuf[fPos];
  inc(fPos);
  CheckLoadBuffer;
end;

function TParser.IsNumber: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['0'..'9'];
end;

function TParser.IsHexNum: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['0'..'9','A'..'F','a'..'f'];
end;

function TParser.IsAlpha: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['_','A'..'Z','a'..'z'];
end;

function TParser.IsAlphaNum: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=IsAlpha or IsNumber;
end;

function TParser.GetHexValue(c: char): byte; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  case c of
    '0'..'9' : Result:=ord(c)-$30;
    'A'..'F' : Result:=ord(c)-$37; //-$41+$0A
    'a'..'f' : Result:=ord(c)-$57; //-$61+$0A
  end;
end;

function TParser.GetAlphaNum: string;
begin
  if not IsAlpha then
    ErrorFmt(SParExpected,[GetTokenName(toSymbol)]);
  Result:='';
  while IsAlphaNum do
  begin
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
end;

procedure TParser.HandleNewLine;
begin
  if fBuf[fPos]=#13 then //CR
  begin
    inc(fPos);
    CheckLoadBuffer;
  end;
  if fBuf[fPos]=#10 then
  begin
    inc(fPos); //CR LF or LF
    CheckLoadBuffer;
  end;
  inc(fSourceLine);
  fDeltaPos:=-(fPos-1);
end;

procedure TParser.SkipBOM;
var
  i : integer;
  bom : string[3];
  backup : integer;
begin
  i:=1;
  bom:='   ';
  backup:=fPos;
  while (fBuf[fPos] in [#$BB,#$BF,#$EF]) and (i<=3) do
  begin
    bom[i]:=fBuf[fPos];
    inc(fPos);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    CheckLoadBuffer;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    inc(i);
  end;
  if (bom<>(#$EF+#$BB+#$BF)) then
    fPos:=backup;
end;

procedure TParser.SkipSpaces;
begin
  while fBuf[fPos] in [' ',#9] do begin
    inc(fPos);
    CheckLoadBuffer;
  end;
end;

procedure TParser.SkipWhitespace;
begin
  while true do
  begin
    case fBuf[fPos] of
      ' ',#9  : SkipSpaces;
      #10,#13 : HandleNewLine
      else break;
    end;
  end;
end;

procedure TParser.HandleEof;
begin
  fToken:=toEOF;
  fLastTokenStr:='';
end;

procedure TParser.HandleAlphaNum;
begin
  fLastTokenStr:=GetAlphaNum;
  fToken:=toSymbol;
end;

procedure TParser.HandleNumber;
type
  floatPunct = (fpDot,fpE);
  floatPuncts = set of floatPunct;
var
  allowed : floatPuncts;
begin
  fLastTokenStr:='';
  while IsNumber do
    ProcessChar;
  fToken:=toInteger;
  if (fBuf[fPos] in ['.','e','E']) then
  begin
    fToken:=toFloat;
    allowed:=[fpDot,fpE];
    while (fBuf[fPos] in ['.','e','E','0'..'9']) do
    begin
      case fBuf[fPos] of
        '.'     : if fpDot in allowed then Exclude(allowed,fpDot) else break;
        'E','e' : if fpE in allowed then
                  begin
                    allowed:=[];
                    ProcessChar;
                    if (fBuf[fPos] in ['+','-']) then ProcessChar;
                    if not (fBuf[fPos] in ['0'..'9']) then
                      ErrorFmt(SParInvalidFloat,[fLastTokenStr+fBuf[fPos]]);
                  end
                  else break;
      end;
      ProcessChar;
    end;
  end;
  if (fBuf[fPos] in ['s','S','d','D','c','C']) then //single, date, currency
  begin
    fFloatType:=fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
    fToken:=toFloat;
  end
  else fFloatType:=#0;
end;

procedure TParser.HandleHexNumber;
var valid : boolean;
begin
  fLastTokenStr:='$';
  inc(fPos);
  CheckLoadBuffer;
  valid:=false;
  while IsHexNum do
  begin
    valid:=true;
    ProcessChar;
  end;
  if not valid then
    ErrorFmt(SParInvalidInteger,[fLastTokenStr]);
  fToken:=toInteger;
end;

function TParser.HandleQuotedString: string;
begin
  Result:='';
  inc(fPos);
  CheckLoadBuffer;
  while true do
  begin
    case fBuf[fPos] of
      #0     : ErrorStr(SParUnterminatedString);
      #13,#10 : ErrorStr(SParUnterminatedString);
      ''''   : begin
                 inc(fPos);
                 CheckLoadBuffer;
                 if fBuf[fPos]<>'''' then exit;
               end;
    end;
=======
begin
  BytesRead := FStream.Read(FBuf^, ParseBufSize);
  if BytesRead = 0 then
  begin
    FEofReached := True;
    Exit;
  end;
  FBuf[BytesRead] := #0;
  Inc(FDeltaPos, BytesRead);
  FPos := 0;
  FBufLen := BytesRead;
end;

procedure TParser.CheckLoadBuffer; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  if fBuf[fPos]=#0 then LoadBuffer;
end;

procedure TParser.ProcessChar; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  fLastTokenStr:=fLastTokenStr+fBuf[fPos];
  inc(fPos);
  CheckLoadBuffer;
end;

function TParser.IsNumber: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['0'..'9'];
end;

function TParser.IsHexNum: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['0'..'9','A'..'F','a'..'f'];
end;

function TParser.IsAlpha: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['_','A'..'Z','a'..'z'];
end;

function TParser.IsAlphaNum: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=IsAlpha or IsNumber;
end;

function TParser.GetHexValue(c: char): byte; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  case c of
    '0'..'9' : Result:=ord(c)-$30;
    'A'..'F' : Result:=ord(c)-$37; //-$41+$0A
    'a'..'f' : Result:=ord(c)-$57; //-$61+$0A
  end;
end;

function TParser.GetAlphaNum: string;
begin
  if not IsAlpha then
    ErrorFmt(SParExpected,[GetTokenName(toSymbol)]);
  Result:='';
  while IsAlphaNum do
  begin
>>>>>>> graemeg/fixes_2_2
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
end;

<<<<<<< HEAD
procedure TParser.HandleDecimalCharacter(var ascii: boolean; out
  WideChr: widechar; out StringChr: char);
var i : integer;
begin
  inc(fPos);
  CheckLoadBuffer;
  // read a word number
  i:=0;
  while IsNumber and (i<high(word)) do
  begin
    i:=i*10+ord(fBuf[fPos])-ord('0');
    inc(fPos);
    CheckLoadBuffer;
  end;
  if i>high(word) then i:=0;
  if i>127 then ascii:=false;
  WideChr:=widechar(word(i));
  if i<256 then
    StringChr:=chr(i)
  else
    StringChr:=#0;
end;

procedure TParser.HandleString;
var ascii : boolean;
  s: string;
  w: WideChar;
  c: char;
begin
  fLastTokenWStr:='';
  fLastTokenStr:='';
  ascii:=true;
  while true do
  begin
    case fBuf[fPos] of
      '''' :
        begin
          // avoid conversions,
          // On some systems conversion from ansistring to widestring and back
          // to ansistring does not give the original ansistring.
          // See bug http://bugs.freepascal.org/view.php?id=15841
          s:=HandleQuotedString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          fLastTokenWStr:=fLastTokenWStr+UnicodeString(s);
=======
          fLastTokenWStr:=fLastTokenWStr+s;
>>>>>>> graemeg/cpstrnew
=======
          fLastTokenWStr:=fLastTokenWStr+s;
>>>>>>> graemeg/cpstrnew
=======
          fLastTokenWStr:=fLastTokenWStr+s;
>>>>>>> graemeg/cpstrnew
=======
          fLastTokenWStr:=fLastTokenWStr+s;
>>>>>>> origin/cpstrnew
          fLastTokenStr:=fLastTokenStr+s;
        end;
      '#'  :
        begin
          HandleDecimalCharacter(ascii,w,c);
          fLastTokenWStr:=fLastTokenWStr+w;
          fLastTokenStr:=fLastTokenStr+c;
        end;
=======
=======
begin
  BytesRead := FStream.Read(FBuf^, ParseBufSize);
  if BytesRead = 0 then
  begin
    FEofReached := True;
    Exit;
  end;
  FBuf[BytesRead] := #0;
  Inc(FDeltaPos, BytesRead);
  FPos := 0;
  FBufLen := BytesRead;
end;

procedure TParser.CheckLoadBuffer; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  if fBuf[fPos]=#0 then LoadBuffer;
end;

procedure TParser.ProcessChar; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  fLastTokenStr:=fLastTokenStr+fBuf[fPos];
  inc(fPos);
  CheckLoadBuffer;
end;

function TParser.IsNumber: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['0'..'9'];
end;

function TParser.IsHexNum: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['0'..'9','A'..'F','a'..'f'];
end;

function TParser.IsAlpha: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=fBuf[fPos] in ['_','A'..'Z','a'..'z'];
end;

function TParser.IsAlphaNum: boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  Result:=IsAlpha or IsNumber;
end;

function TParser.GetHexValue(c: char): byte; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
begin
  case c of
    '0'..'9' : Result:=ord(c)-$30;
    'A'..'F' : Result:=ord(c)-$37; //-$41+$0A
    'a'..'f' : Result:=ord(c)-$57; //-$61+$0A
  end;
end;

function TParser.GetAlphaNum: string;
begin
  if not IsAlpha then
    ErrorFmt(SParExpected,[GetTokenName(toSymbol)]);
  Result:='';
  while IsAlphaNum do
  begin
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
end;

>>>>>>> origin/fixes_2_2
procedure TParser.HandleNewLine;
begin
  if fBuf[fPos]=#13 then //CR
  begin
    inc(fPos);
    CheckLoadBuffer;
    if fBuf[fPos]=#10 then inc(fPos); //CR LF
  end
  else inc(fPos); //LF
  inc(fSourceLine);
  fDeltaPos:=-(fPos-1);
<<<<<<< HEAD
end;

procedure TParser.SkipSpaces;
begin
  while fBuf[fPos] in [' ',#9] do
    inc(fPos);
end;

procedure TParser.SkipWhitespace;
begin
=======
end;

procedure TParser.SkipSpaces;
begin
  while fBuf[fPos] in [' ',#9] do
    inc(fPos);
end;

procedure TParser.SkipWhitespace;
begin
>>>>>>> origin/fixes_2_2
  while true do
  begin
    CheckLoadBuffer;
    case fBuf[fPos] of
      ' ',#9  : SkipSpaces;
      #10,#13 : HandleNewLine
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      else break;
    end;
  end;
  if ascii then
    fToken:=Classes.toString
  else
    fToken:=toWString;
end;

<<<<<<< HEAD
procedure TParser.HandleMinus;
begin
  inc(fPos);
  CheckLoadBuffer;
  if IsNumber then
  begin
    HandleNumber;
    fLastTokenStr:='-'+fLastTokenStr;
  end
  else
  begin
    fToken:='-';
    fLastTokenStr:=fToken;
  end;
end;

procedure TParser.HandleUnknown;
begin
  fToken:=fBuf[fPos];
  fLastTokenStr:=fToken;
  inc(fPos);
  CheckLoadBuffer;
end;

constructor TParser.Create(Stream: TStream);
begin
  fStream:=Stream;
  fBuf:=GetMem(ParseBufSize+1);
  fBufLen:=0;
  fPos:=0;
  fDeltaPos:=1;
  fSourceLine:=1;
  fEofReached:=false;
  fLastTokenStr:='';
  fLastTokenWStr:='';
  fFloatType:=#0;
  fToken:=#0;
  LoadBuffer;
  SkipBom;
  NextToken;
end;

destructor TParser.Destroy;
begin
  fStream.Position:=SourcePos;
  FreeMem(fBuf);
end;

procedure TParser.CheckToken(T: Char);
begin
  if fToken<>T then
    ErrorFmt(SParWrongTokenType,[GetTokenName(T),GetTokenName(fToken)]);
end;

procedure TParser.HandleEof;
begin
<<<<<<< HEAD
  CheckToken(toSymbol);
  if CompareText(fLastTokenStr,S)<>0 then
    ErrorFmt(SParWrongTokenSymbol,[s,fLastTokenStr]);
end;

procedure TParser.Error(const Ident: string);
=======
  fToken:=toEOF;
  fLastTokenStr:='';
end;

procedure TParser.HandleAlphaNum;
>>>>>>> graemeg/fixes_2_2
begin
  fLastTokenStr:=GetAlphaNum;
  fToken:=toSymbol;
end;

<<<<<<< HEAD
procedure TParser.ErrorFmt(const Ident: string; const Args: array of const);
begin
  ErrorStr(Format(Ident,Args));
end;

procedure TParser.ErrorStr(const Message: string);
begin
  raise EParserError.CreateFmt(Message+SParLocInfo,[SourceLine,fPos+fDeltaPos,SourcePos]);
end;

procedure TParser.HexToBinary(Stream: TStream);
var outbuf : array[0..ParseBufSize-1] of byte;
    b : byte;
    i : integer;
begin
  i:=0;
  SkipWhitespace;
  while IsHexNum do
  begin
    b:=(GetHexValue(fBuf[fPos]) shl 4);
    inc(fPos);
    CheckLoadBuffer;
    if not IsHexNum then
      Error(SParUnterminatedBinValue);
    b:=b or GetHexValue(fBuf[fPos]);
    inc(fPos);
    CheckLoadBuffer;
    outbuf[i]:=b;
    inc(i);
    if i>=ParseBufSize then
    begin
      Stream.WriteBuffer(outbuf[0],i);
      i:=0;
    end;
    SkipWhitespace;
  end;
  if i>0 then
    Stream.WriteBuffer(outbuf[0],i);
  NextToken;
end;

function TParser.NextToken: Char;

begin
  SkipWhiteSpace;
  if fEofReached then
    HandleEof
  else
    case fBuf[fPos] of
      '_','A'..'Z','a'..'z' : HandleAlphaNum;
      '$'                   : HandleHexNumber;
      '-'                   : HandleMinus;
      '0'..'9'              : HandleNumber;
      '''','#'              : HandleString
      else
        HandleUnknown;
    end;
  Result:=fToken;
end;

function TParser.SourcePos: Longint;
begin
  Result:=fStream.Position-fBufLen+fPos;
end;

function TParser.TokenComponentIdent: string;
begin
=======
procedure TParser.HandleNumber;
type
  floatPunct = (fpDot,fpE);
  floatPuncts = set of floatPunct;
var
  allowed : floatPuncts;
begin
  fLastTokenStr:='';
  while IsNumber do
    ProcessChar;
  fToken:=toInteger;
  if (fBuf[fPos] in ['.','e','E']) then
  begin
    fToken:=toFloat;
    allowed:=[fpDot,fpE];
    while (fBuf[fPos] in ['.','e','E','0'..'9']) do
    begin
      case fBuf[fPos] of
        '.'     : if fpDot in allowed then Exclude(allowed,fpDot) else break;
        'E','e' : if fpE in allowed then
                  begin
                    allowed:=[];
                    ProcessChar;
                    if (fBuf[fPos] in ['+','-']) then ProcessChar;
                    if not (fBuf[fPos] in ['0'..'9']) then
                      ErrorFmt(SParInvalidFloat,[fLastTokenStr+fBuf[fPos]]);
                  end
                  else break;
      end;
      ProcessChar;
    end;
  end;
  if (fBuf[fPos] in ['s','S','d','D','c','C']) then //single, date, currency
  begin
    fFloatType:=fBuf[fPos];
    inc(fPos);
    fToken:=toFloat;
  end
  else fFloatType:=#0;
end;

procedure TParser.HandleHexNumber;
var valid : boolean;
begin
  fLastTokenStr:='$';
  inc(fPos);
  CheckLoadBuffer;
  valid:=false;
  while IsHexNum do
  begin
    valid:=true;
    ProcessChar;
  end;
  if not valid then
    ErrorFmt(SParInvalidInteger,[fLastTokenStr]);
  fToken:=toInteger;
end;

function TParser.HandleQuotedString: string;
begin
  Result:='';
  inc(fPos);
  CheckLoadBuffer;
  while true do
  begin
    case fBuf[fPos] of
      #0     : ErrorStr(SParUnterminatedString);
      #13,#10 : ErrorStr(SParUnterminatedString);
      ''''   : begin
                 inc(fPos);
                 CheckLoadBuffer;
                 if fBuf[fPos]<>'''' then exit;
               end;
    end;
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
end;

function TParser.HandleDecimalString(var ascii : boolean): widestring;
var i : integer;
begin
  Result:='';
  inc(fPos);
  CheckLoadBuffer;
  while IsNumber do
  begin
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
  if not TryStrToInt(Result,i) then
  i:=0;
  if i>127 then ascii:=false;
  setlength(Result,1);
  Result[1]:=widechar(word(i));
end;

procedure TParser.HandleString;
var ascii : boolean;
begin
  fLastTokenWStr:='';
  ascii:=true;
  while true do
    case fBuf[fPos] of
      '''' : fLastTokenWStr:=fLastTokenWStr+HandleQuotedString;
      '#'  : fLastTokenWStr:=fLastTokenWStr+HandleDecimalString(ascii)
      else break;
    end;
  if ascii then
    fToken:=toString
  else
    fToken:=toWString;
  fLastTokenStr:=fLastTokenWStr;
end;

procedure TParser.HandleMinus;
begin
  inc(fPos);
  CheckLoadBuffer;
  if IsNumber then
  begin
    HandleNumber;
    fLastTokenStr:='-'+fLastTokenStr;
  end
  else
  begin
    fToken:='-';
    fLastTokenStr:=fToken;
  end;
end;

procedure TParser.HandleUnknown;
begin
  fToken:=fBuf[fPos];
  fLastTokenStr:=fToken;
  inc(fPos);
end;

constructor TParser.Create(Stream: TStream);
begin
  fStream:=Stream;
  fBuf:=GetMem(ParseBufSize+1);
  fBufLen:=0;
  fPos:=0;
  fDeltaPos:=1;
  fSourceLine:=1;
  fEofReached:=false;
  fLastTokenStr:='';
  fLastTokenWStr:='';
  fFloatType:=#0;
  fToken:=#0;
  LoadBuffer;
  NextToken;
end;

destructor TParser.Destroy;
begin
  fStream.Position:=SourcePos;
  FreeMem(fBuf);
end;

procedure TParser.CheckToken(T: Char);
begin
  if fToken<>T then
    ErrorFmt(SParWrongTokenType,[GetTokenName(T),GetTokenName(fToken)]);
end;

procedure TParser.CheckTokenSymbol(const S: string);
begin
  CheckToken(toSymbol);
  if CompareText(fLastTokenStr,S)<>0 then
    ErrorFmt(SParWrongTokenSymbol,[s,fLastTokenStr]);
end;

<<<<<<< HEAD
procedure TParser.Error(const Ident: string);
=======
procedure TParser.SkipBOM;
var
  i : integer;
  bom : string[3];
  backup : integer;
begin
  i:=1;
  bom:='   ';
  backup:=fPos;
  while (fBuf[fPos] in [#$BB,#$BF,#$EF]) and (i<=3) do
  begin
    bom[i]:=fBuf[fPos];
    inc(fPos);
    inc(i);
  end;
  if (bom<>(#$EF+#$BB+#$BF)) then
    fPos:=backup;
end;

procedure TParser.SkipSpaces;
>>>>>>> origin/fixes_2.4
begin
  ErrorStr(Ident);
end;

procedure TParser.ErrorFmt(const Ident: string; const Args: array of const);
begin
  ErrorStr(Format(Ident,Args));
end;

procedure TParser.ErrorStr(const Message: string);
begin
  raise EParserError.CreateFmt(Message+SParLocInfo,[SourceLine,fPos+fDeltaPos,SourcePos]);
end;

procedure TParser.HexToBinary(Stream: TStream);
var outbuf : array[0..ParseBufSize-1] of byte;
    b : byte;
    i : integer;
begin
  i:=0;
  SkipWhitespace;
  while IsHexNum do
  begin
    b:=(GetHexValue(fBuf[fPos]) shl 4);
    inc(fPos);
    CheckLoadBuffer;
    if not IsHexNum then
      Error(SParUnterminatedBinValue);
    b:=b or GetHexValue(fBuf[fPos]);
    inc(fPos);
    outbuf[i]:=b;
    inc(i);
    if i>=ParseBufSize then
    begin
      Stream.WriteBuffer(outbuf[0],i);
      i:=0;
    end;
    SkipWhitespace;
  end;
  if i>0 then
    Stream.WriteBuffer(outbuf[0],i);
  NextToken;
=======
procedure TParser.HandleEof;
begin
  fToken:=toEOF;
  fLastTokenStr:='';
end;

procedure TParser.HandleAlphaNum;
begin
  fLastTokenStr:=GetAlphaNum;
  fToken:=toSymbol;
end;

procedure TParser.HandleNumber;
type
  floatPunct = (fpDot,fpE);
  floatPuncts = set of floatPunct;
var
  allowed : floatPuncts;
begin
  fLastTokenStr:='';
  while IsNumber do
    ProcessChar;
  fToken:=toInteger;
  if (fBuf[fPos] in ['.','e','E']) then
  begin
    fToken:=toFloat;
    allowed:=[fpDot,fpE];
    while (fBuf[fPos] in ['.','e','E','0'..'9']) do
    begin
      case fBuf[fPos] of
        '.'     : if fpDot in allowed then Exclude(allowed,fpDot) else break;
        'E','e' : if fpE in allowed then
                  begin
                    allowed:=[];
                    ProcessChar;
                    if (fBuf[fPos] in ['+','-']) then ProcessChar;
                    if not (fBuf[fPos] in ['0'..'9']) then
                      ErrorFmt(SParInvalidFloat,[fLastTokenStr+fBuf[fPos]]);
                  end
                  else break;
      end;
      ProcessChar;
    end;
  end;
  if (fBuf[fPos] in ['s','S','d','D','c','C']) then //single, date, currency
  begin
    fFloatType:=fBuf[fPos];
    inc(fPos);
    fToken:=toFloat;
  end
  else fFloatType:=#0;
end;

procedure TParser.HandleHexNumber;
var valid : boolean;
begin
  fLastTokenStr:='$';
  inc(fPos);
  CheckLoadBuffer;
  valid:=false;
  while IsHexNum do
  begin
    valid:=true;
    ProcessChar;
  end;
  if not valid then
    ErrorFmt(SParInvalidInteger,[fLastTokenStr]);
  fToken:=toInteger;
end;

function TParser.HandleQuotedString: string;
begin
  Result:='';
  inc(fPos);
  CheckLoadBuffer;
  while true do
  begin
    case fBuf[fPos] of
      #0     : ErrorStr(SParUnterminatedString);
      #13,#10 : ErrorStr(SParUnterminatedString);
      ''''   : begin
                 inc(fPos);
                 CheckLoadBuffer;
                 if fBuf[fPos]<>'''' then exit;
               end;
    end;
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
end;

function TParser.HandleDecimalString(var ascii : boolean): widestring;
var i : integer;
begin
  Result:='';
  inc(fPos);
  CheckLoadBuffer;
  while IsNumber do
  begin
    Result:=Result+fBuf[fPos];
    inc(fPos);
    CheckLoadBuffer;
  end;
  if not TryStrToInt(Result,i) then
  i:=0;
  if i>127 then ascii:=false;
  setlength(Result,1);
  Result[1]:=widechar(word(i));
end;

procedure TParser.HandleString;
var ascii : boolean;
begin
  fLastTokenWStr:='';
  ascii:=true;
  while true do
    case fBuf[fPos] of
      '''' : fLastTokenWStr:=fLastTokenWStr+HandleQuotedString;
      '#'  : fLastTokenWStr:=fLastTokenWStr+HandleDecimalString(ascii)
      else break;
    end;
  if ascii then
    fToken:=toString
  else
    fToken:=toWString;
  fLastTokenStr:=fLastTokenWStr;
end;

procedure TParser.HandleMinus;
begin
  inc(fPos);
  CheckLoadBuffer;
  if IsNumber then
  begin
    HandleNumber;
    fLastTokenStr:='-'+fLastTokenStr;
  end
  else
  begin
    fToken:='-';
    fLastTokenStr:=fToken;
  end;
end;

procedure TParser.HandleUnknown;
begin
  fToken:=fBuf[fPos];
  fLastTokenStr:=fToken;
  inc(fPos);
end;

constructor TParser.Create(Stream: TStream);
begin
  fStream:=Stream;
  fBuf:=GetMem(ParseBufSize+1);
  fBufLen:=0;
  fPos:=0;
  fDeltaPos:=1;
  fSourceLine:=1;
  fEofReached:=false;
  fLastTokenStr:='';
  fLastTokenWStr:='';
  fFloatType:=#0;
  fToken:=#0;
  LoadBuffer;
  SkipBom;
  NextToken;
end;

destructor TParser.Destroy;
begin
  fStream.Position:=SourcePos;
  FreeMem(fBuf);
end;

procedure TParser.CheckToken(T: Char);
begin
  if fToken<>T then
    ErrorFmt(SParWrongTokenType,[GetTokenName(T),GetTokenName(fToken)]);
end;

procedure TParser.CheckTokenSymbol(const S: string);
begin
  CheckToken(toSymbol);
  if CompareText(fLastTokenStr,S)<>0 then
    ErrorFmt(SParWrongTokenSymbol,[s,fLastTokenStr]);
end;

procedure TParser.Error(const Ident: string);
begin
  ErrorStr(Ident);
end;

procedure TParser.ErrorFmt(const Ident: string; const Args: array of const);
begin
  ErrorStr(Format(Ident,Args));
end;

procedure TParser.ErrorStr(const Message: string);
begin
  raise EParserError.CreateFmt(Message+SParLocInfo,[SourceLine,fPos+fDeltaPos,SourcePos]);
end;

procedure TParser.HexToBinary(Stream: TStream);
var outbuf : array[0..ParseBufSize-1] of byte;
    b : byte;
    i : integer;
begin
  i:=0;
  SkipWhitespace;
  while IsHexNum do
  begin
    b:=(GetHexValue(fBuf[fPos]) shl 4);
    inc(fPos);
    CheckLoadBuffer;
    if not IsHexNum then
      Error(SParUnterminatedBinValue);
    b:=b or GetHexValue(fBuf[fPos]);
    inc(fPos);
    outbuf[i]:=b;
    inc(i);
    if i>=ParseBufSize then
    begin
      Stream.WriteBuffer(outbuf[0],i);
      i:=0;
    end;
    SkipWhitespace;
  end;
  if i>0 then
    Stream.WriteBuffer(outbuf[0],i);
  NextToken;
end;

function TParser.NextToken: Char;

begin
  SkipWhiteSpace;
  if fEofReached then
    HandleEof
  else
    case fBuf[fPos] of
      '_','A'..'Z','a'..'z' : HandleAlphaNum;
      '$'                   : HandleHexNumber;
      '-'                   : HandleMinus;
      '0'..'9'              : HandleNumber;
      '''','#'              : HandleString
      else
        HandleUnknown;
    end;
  Result:=fToken;
end;

function TParser.SourcePos: Longint;
begin
  Result:=fStream.Position-fBufLen+fPos;
end;

function TParser.TokenComponentIdent: string;
begin
  if fToken<>toSymbol then
    ErrorFmt(SParExpected,[GetTokenName(toSymbol)]);
  CheckLoadBuffer;
  while fBuf[fPos]='.' do
  begin
    ProcessChar;
    fLastTokenStr:=fLastTokenStr+GetAlphaNum;
  end;
  Result:=fLastTokenStr;
>>>>>>> origin/fixes_2_2
end;

function TParser.NextToken: Char;

begin
  SkipWhiteSpace;
  if fEofReached then
    HandleEof
  else
    case fBuf[fPos] of
      '_','A'..'Z','a'..'z' : HandleAlphaNum;
      '$'                   : HandleHexNumber;
      '-'                   : HandleMinus;
      '0'..'9'              : HandleNumber;
      '''','#'              : HandleString
      else
        HandleUnknown;
    end;
  Result:=fToken;
end;

function TParser.SourcePos: Longint;
begin
  Result:=fStream.Position-fBufLen+fPos;
end;

function TParser.TokenComponentIdent: string;
begin
>>>>>>> graemeg/fixes_2_2
  if fToken<>toSymbol then
    ErrorFmt(SParExpected,[GetTokenName(toSymbol)]);
  CheckLoadBuffer;
  while fBuf[fPos]='.' do
  begin
    ProcessChar;
    fLastTokenStr:=fLastTokenStr+GetAlphaNum;
  end;
  Result:=fLastTokenStr;
end;

{$ifndef FPUNONE}
Function TParser.TokenFloat: Extended;

var errcode : word;

begin
  Val(fLastTokenStr,Result,errcode);
  if errcode<>0 then
    ErrorFmt(SParInvalidFloat,[fLastTokenStr]);
end;
{$endif}

Function TParser.TokenInt: Int64;
begin
  if not TryStrToInt64(fLastTokenStr,Result) then
    Result:=Int64(StrToQWord(fLastTokenStr)); //second chance for malformed files
end;

function TParser.TokenString: string;
begin
  case fToken of
<<<<<<< HEAD
<<<<<<< HEAD
    toWString : Result:=string(fLastTokenWStr);
=======
    toWString : Result:=fLastTokenWStr;
>>>>>>> graemeg/fixes_2_2
=======
    toWString : Result:=fLastTokenWStr;
>>>>>>> origin/fixes_2_2
    toFloat : if fFloatType<>#0 then
                Result:=fLastTokenStr+fFloatType
              else Result:=fLastTokenStr
    else
      Result:=fLastTokenStr;
  end;
end;

function TParser.TokenWideString: WideString;
begin
  if fToken=toWString then
    Result:=fLastTokenWStr
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=WideString(fLastTokenStr);
=======
    Result:=fLastTokenStr;
>>>>>>> graemeg/fixes_2_2
=======
    Result:=fLastTokenStr;
>>>>>>> origin/fixes_2_2
end;

function TParser.TokenSymbolIs(const S: string): Boolean;
begin
  Result:=(fToken=toSymbol) and (CompareText(fLastTokenStr,S)=0);
end;

