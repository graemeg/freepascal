{
    Delphi/Kylix compatibility unit: String handling routines.

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
{$inline on}
unit strutils;

interface

uses
  SysUtils{, Types};

{ ---------------------------------------------------------------------
    Case insensitive search/replace
  ---------------------------------------------------------------------}

Function AnsiResemblesText(const AText, AOther: string): Boolean;
Function AnsiContainsText(const AText, ASubText: string): Boolean;
Function AnsiStartsText(const ASubText, AText: string): Boolean;inline;
Function AnsiEndsText(const ASubText, AText: string): Boolean;inline;
Function AnsiReplaceText(const AText, AFromText, AToText: string): string;inline;
Function AnsiMatchText(const AText: string; const AValues: array of string): Boolean;
Function AnsiIndexText(const AText: string; const AValues: array of string): Integer;

{ ---------------------------------------------------------------------
    Case sensitive search/replace
  ---------------------------------------------------------------------}

Function AnsiContainsStr(const AText, ASubText: string): Boolean;inline;
Function AnsiStartsStr(const ASubText, AText: string): Boolean;inline;
Function AnsiEndsStr(const ASubText, AText: string): Boolean;inline;
Function AnsiReplaceStr(const AText, AFromText, AToText: string): string;inline;
Function AnsiMatchStr(const AText: string; const AValues: array of string): Boolean;
Function AnsiIndexStr(const AText: string; const AValues: array of string): Integer;

{ ---------------------------------------------------------------------
    Miscellaneous
  ---------------------------------------------------------------------}

Function DupeString(const AText: string; ACount: Integer): string;
Function ReverseString(const AText: string): string;
Function AnsiReverseString(const AText: AnsiString): AnsiString;inline;
Function StuffString(const AText: string; AStart, ALength: Cardinal;  const ASubText: string): string;
Function RandomFrom(const AValues: array of string): string; overload;
Function IfThen(AValue: Boolean; const ATrue: string; const AFalse: string = ''): string; overload;

{ ---------------------------------------------------------------------
    VB emulations.
  ---------------------------------------------------------------------}

Function LeftStr(const AText: AnsiString; const ACount: Integer): AnsiString;inline;
Function RightStr(const AText: AnsiString; const ACount: Integer): AnsiString;
Function MidStr(const AText: AnsiString; const AStart, ACount: Integer): AnsiString;inline;
Function RightBStr(const AText: AnsiString; const AByteCount: Integer): AnsiString;inline;
Function MidBStr(const AText: AnsiString; const AByteStart, AByteCount: Integer): AnsiString;inline;
Function AnsiLeftStr(const AText: AnsiString; const ACount: Integer): AnsiString;inline;
Function AnsiRightStr(const AText: AnsiString; const ACount: Integer): AnsiString;inline;
Function AnsiMidStr(const AText: AnsiString; const AStart, ACount: Integer): AnsiString;inline;
Function LeftBStr(const AText: AnsiString; const AByteCount: Integer): AnsiString;inline;
Function LeftStr(const AText: WideString; const ACount: Integer): WideString;inline;
Function RightStr(const AText: WideString; const ACount: Integer): WideString;
Function MidStr(const AText: WideString; const AStart, ACount: Integer): WideString;inline;

{ ---------------------------------------------------------------------
    Extended search and replace
  ---------------------------------------------------------------------}

const
  { Default word delimiters are any character except the core alphanumerics. }
  WordDelimiters: set of Char = [#0..#255] - ['a'..'z','A'..'Z','1'..'9','0'];
  
resourcestring
  SErrAmountStrings        = 'Amount of search and replace strings don''t match';

type
  TStringSearchOption = (soDown, soMatchCase, soWholeWord);
  TStringSearchOptions = set of TStringSearchOption;
  TStringSeachOption = TStringSearchOption;

Function SearchBuf(Buf: PChar; BufLen: Integer; SelStart, SelLength: Integer; SearchString: String; Options: TStringSearchOptions): PChar;
Function SearchBuf(Buf: PChar; BufLen: Integer; SelStart, SelLength: Integer; SearchString: String): PChar;inline; // ; Options: TStringSearchOptions = [soDown]
Function PosEx(const SubStr, S: string; Offset: Cardinal): Integer;
Function PosEx(const SubStr, S: string): Integer;inline; // Offset: Cardinal = 1
Function PosEx(c:char; const S: string; Offset: Cardinal): Integer;
function StringsReplace(const S: string; OldPattern, NewPattern: array of string;  Flags: TReplaceFlags): string;

{ ---------------------------------------------------------------------
    Soundex Functions.
  ---------------------------------------------------------------------}

type
  TSoundexLength = 1..MaxInt;

Function Soundex(const AText: string; ALength: TSoundexLength): string;
Function Soundex(const AText: string): string;inline; // ; ALength: TSoundexLength = 4

type
  TSoundexIntLength = 1..8;

Function SoundexInt(const AText: string; ALength: TSoundexIntLength): Integer;
Function SoundexInt(const AText: string): Integer;inline; //; ALength: TSoundexIntLength = 4
Function DecodeSoundexInt(AValue: Integer): string;
Function SoundexWord(const AText: string): Word;
Function DecodeSoundexWord(AValue: Word): string;
Function SoundexSimilar(const AText, AOther: string; ALength: TSoundexLength): Boolean;inline;
Function SoundexSimilar(const AText, AOther: string): Boolean;inline; //; ALength: TSoundexLength = 4
Function SoundexCompare(const AText, AOther: string; ALength: TSoundexLength): Integer;inline;
Function SoundexCompare(const AText, AOther: string): Integer;inline; //; ALength: TSoundexLength = 4
Function SoundexProc(const AText, AOther: string): Boolean;

type
  TCompareTextProc = Function(const AText, AOther: string): Boolean;

Const
  AnsiResemblesProc: TCompareTextProc = @SoundexProc;

{ ---------------------------------------------------------------------
    Other functions, based on RxStrUtils.
  ---------------------------------------------------------------------}

function IsEmptyStr(const S: string; const EmptyChars: TSysCharSet): Boolean;
function DelSpace(const S: string): string;
function DelChars(const S: string; Chr: Char): string;
function DelSpace1(const S: string): string;
function Tab2Space(const S: string; Numb: Byte): string;
function NPos(const C: string; S: string; N: Integer): Integer;
Function RPosEX(C:char;const S : AnsiString;offs:cardinal):Integer; overload;
Function RPosex (Const Substr : AnsiString; Const Source : AnsiString;offs:cardinal) : Integer; overload;
Function RPos(c:char;const S : AnsiString):Integer; overload;
Function RPos (Const Substr : AnsiString; Const Source : AnsiString) : Integer; overload;
function AddChar(C: Char; const S: string; N: Integer): string;
function AddCharR(C: Char; const S: string; N: Integer): string;
function PadLeft(const S: string; N: Integer): string;inline;
function PadRight(const S: string; N: Integer): string;inline;
function PadCenter(const S: string; Len: Integer): string;
function Copy2Symb(const S: string; Symb: Char): string;
function Copy2SymbDel(var S: string; Symb: Char): string;
function Copy2Space(const S: string): string;inline;
function Copy2SpaceDel(var S: string): string;inline;
function AnsiProperCase(const S: string; const WordDelims: TSysCharSet): string;
function WordCount(const S: string; const WordDelims: TSysCharSet): Integer;
function WordPosition(const N: Integer; const S: string; const WordDelims: TSysCharSet): Integer;
function ExtractWord(N: Integer; const S: string;  const WordDelims: TSysCharSet): string;inline;
function ExtractWordPos(N: Integer; const S: string; const WordDelims: TSysCharSet; var Pos: Integer): string;
function ExtractDelimited(N: Integer; const S: string;  const Delims: TSysCharSet): string;
function ExtractSubstr(const S: string; var Pos: Integer;  const Delims: TSysCharSet): string;
function IsWordPresent(const W, S: string; const WordDelims: TSysCharSet): Boolean;
function FindPart(const HelpWilds, InputStr: string): Integer;
function IsWild(InputStr, Wilds: string; IgnoreCase: Boolean): Boolean;
function XorString(const Key, Src: ShortString): ShortString;
function XorEncode(const Key, Source: string): string;
function XorDecode(const Key, Source: string): string;
function GetCmdLineArg(const Switch: string; SwitchChars: TSysCharSet): string;
function Numb2USA(const S: string): string;
function Hex2Dec(const S: string): Longint;
function Dec2Numb(N: Longint; Len, Base: Byte): string;
function Numb2Dec(S: string; Base: Byte): Longint;
function IntToBin(Value: Longint; Digits, Spaces: Integer): string;
function IntToBin(Value: Longint; Digits: Integer): string;
function intToBin(Value: int64; Digits:integer): string;
function IntToRoman(Value: Longint): string;
function RomanToInt(const S: string): Longint;
procedure BinToHex(BinValue, HexValue: PChar; BinBufSize: Integer);
function HexToBin(HexValue, BinValue: PChar; BinBufSize: Integer): Integer;

const
  DigitChars = ['0'..'9'];
  Brackets = ['(',')','[',']','{','}'];
  StdWordDelims = [#0..' ',',','.',';','/','\',':','''','"','`'] + Brackets;
  StdSwitchChars = ['-','/'];

function PosSet (const c:TSysCharSet;const s : ansistring ):Integer;
function PosSet (const c:string;const s : ansistring ):Integer;
function PosSetEx (const c:TSysCharSet;const s : ansistring;count:Integer ):Integer;
function PosSetEx (const c:string;const s : ansistring;count:Integer ):Integer;

Procedure Removeleadingchars(VAR S : AnsiString; Const CSet:TSysCharset);
Procedure RemoveTrailingChars(VAR S : AnsiString;Const CSet:TSysCharset);
Procedure RemovePadChars(VAR S : AnsiString;Const CSet:TSysCharset);

function TrimLeftSet(const S: String;const CSet:TSysCharSet): String;
Function TrimRightSet(const S: String;const CSet:TSysCharSet): String;
function TrimSet(const S: String;const CSet:TSysCharSet): String;

implementation

{ ---------------------------------------------------------------------
   Possibly Exception raising functions
  ---------------------------------------------------------------------}


function Hex2Dec(const S: string): Longint;
var
  HexStr: string;
begin
  if Pos('$',S)=0 then
    HexStr:='$'+ S
  else
    HexStr:=S;
  Result:=StrToInt(HexStr);
end;

{
  We turn off implicit exceptions, since these routines are tested, and it 
  saves 20% codesize (and some speed) and don't throw exceptions, except maybe 
  heap related. If they don't, that is consider a bug.

  In the future, be wary with routines that use strtoint, floating point 
  and/or format() derivatives. And check every divisor for 0.
}

{$IMPLICITEXCEPTIONS OFF}

{ ---------------------------------------------------------------------
    Case insensitive search/replace
  ---------------------------------------------------------------------}
Function AnsiResemblesText(const AText, AOther: string): Boolean;

begin
  if Assigned(AnsiResemblesProc) then
    Result:=AnsiResemblesProc(AText,AOther)
  else
    Result:=False;
end;

Function AnsiContainsText(const AText, ASubText: string): Boolean;
begin
  AnsiContainsText:=AnsiPos(AnsiUppercase(ASubText),AnsiUppercase(AText))>0;
end;


Function AnsiStartsText(const ASubText, AText: string): Boolean;inline;
begin
  Result:=AnsiCompareText(Copy(AText,1,Length(AsubText)),ASubText)=0;
end;


Function AnsiEndsText(const ASubText, AText: string): Boolean;inline;
begin
 result:=AnsiCompareText(Copy(AText,Length(AText)-Length(ASubText)+1,Length(ASubText)),asubtext)=0;
end;


Function AnsiReplaceText(const AText, AFromText, AToText: string): string;inline;
begin
  Result := StringReplace(AText,AFromText,AToText,[rfReplaceAll,rfIgnoreCase]);
end;


Function AnsiMatchText(const AText: string; const AValues: array of string): Boolean;
begin
  Result:=(AnsiIndexText(AText,AValues)<>-1)
end;


Function AnsiIndexText(const AText: string; const AValues: array of string): Integer;

var i : longint;

begin
  result:=-1;
  if high(AValues)=-1 Then
    Exit;
  for i:=low(AValues) to High(Avalues) do
     if CompareText(avalues[i],atext)=0 Then
       exit(i);  // make sure it is the first val.
end;


{ ---------------------------------------------------------------------
    Case sensitive search/replace
  ---------------------------------------------------------------------}

Function AnsiContainsStr(const AText, ASubText: string): Boolean;inline;
begin
  Result := AnsiPos(ASubText,AText)>0;
end;


Function AnsiStartsStr(const ASubText, AText: string): Boolean;inline;
begin
  Result := AnsiPos(ASubText,AText)=1;
end;


Function AnsiEndsStr(const ASubText, AText: string): Boolean;inline;
begin
 Result := AnsiCompareStr(Copy(AText,length(AText)-length(ASubText)+1,length(ASubText)),ASubText)=0;
end;


Function AnsiReplaceStr(const AText, AFromText, AToText: string): string;inline;
begin
Result := StringReplace(AText,AFromText,AToText,[rfReplaceAll]);
end;


Function AnsiMatchStr(const AText: string; const AValues: array of string): Boolean;
begin
  Result:=AnsiIndexStr(AText,Avalues)<>-1;
end;


Function AnsiIndexStr(const AText: string; const AValues: array of string): Integer;
var
  i : longint;
begin
  result:=-1;
  if high(AValues)=-1 Then
    Exit;
  for i:=low(AValues) to High(Avalues) do
     if (avalues[i]=AText) Then
       exit(i);                                 // make sure it is the first val.
end;


{ ---------------------------------------------------------------------
    Playthingies
  ---------------------------------------------------------------------}

Function DupeString(const AText: string; ACount: Integer): string;

var i,l : integer;

begin
 result:='';
 if aCount>=0 then
   begin
     l:=length(atext);
     SetLength(result,aCount*l);
     for i:=0 to ACount-1 do
       move(atext[1],Result[l*i+1],l);
   end;
end;

Function ReverseString(const AText: string): string;

var
    i,j:longint;

begin
  setlength(result,length(atext));
  i:=1; j:=length(atext);
  while (i<=j) do
    begin
      result[i]:=atext[j-i+1];
      inc(i);
    end;
end;


Function AnsiReverseString(const AText: AnsiString): AnsiString;inline;

begin
  Result:=ReverseString(AText);
end;



Function StuffString(const AText: string; AStart, ALength: Cardinal;  const ASubText: string): string;

var i,j,k : SizeUInt;

begin
  j:=length(ASubText);
  i:=length(AText);
  if AStart>i then 
    aStart:=i+1;
  k:=i+1-AStart;
  if ALength> k then
    ALength:=k;
  SetLength(Result,i+j-ALength);
  move (AText[1],result[1],AStart-1);
  move (ASubText[1],result[AStart],j);
  move (AText[AStart+ALength], Result[AStart+j],i+1-AStart-ALength);
end;

Function RandomFrom(const AValues: array of string): string; overload;

begin
  if high(AValues)=-1 then exit('');
  result:=Avalues[random(High(AValues)+1)];
end;

Function IfThen(AValue: Boolean; const ATrue: string; const AFalse: string = ''): string; overload;

begin
  if avalue then
    result:=atrue
  else
    result:=afalse;
end;

{ ---------------------------------------------------------------------
    VB emulations.
  ---------------------------------------------------------------------}

Function LeftStr(const AText: AnsiString; const ACount: Integer): AnsiString;inline;

begin
  Result:=Copy(AText,1,ACount);
end;

Function RightStr(const AText: AnsiString; const ACount: Integer): AnsiString;

var j,l:integer;

begin
  l:=length(atext);
  j:=ACount;
  if j>l then j:=l;
  Result:=Copy(AText,l-j+1,j);
end;

Function MidStr(const AText: AnsiString; const AStart, ACount: Integer): AnsiString;inline;

begin
  if (ACount=0) or (AStart>length(atext)) then
    exit('');
  Result:=Copy(AText,AStart,ACount);
end;



Function LeftBStr(const AText: AnsiString; const AByteCount: Integer): AnsiString;inline;

begin
  Result:=LeftStr(AText,AByteCount);
end;


Function RightBStr(const AText: AnsiString; const AByteCount: Integer): AnsiString;inline;
begin
  Result:=RightStr(Atext,AByteCount);
end;


Function MidBStr(const AText: AnsiString; const AByteStart, AByteCount: Integer): AnsiString;inline;
begin
  Result:=MidStr(AText,AByteStart,AByteCount);
end;


Function AnsiLeftStr(const AText: AnsiString; const ACount: Integer): AnsiString;inline;
begin
  Result := copy(AText,1,ACount);
end;


Function AnsiRightStr(const AText: AnsiString; const ACount: Integer): AnsiString;inline;
begin
  Result := copy(AText,length(AText)-ACount+1,ACount);
end;


Function AnsiMidStr(const AText: AnsiString; const AStart, ACount: Integer): AnsiString;inline;
begin
  Result:=Copy(AText,AStart,ACount);
end;


Function LeftStr(const AText: WideString; const ACount: Integer): WideString;inline;
begin
  Result:=Copy(AText,1,ACount);
end;


Function RightStr(const AText: WideString; const ACount: Integer): WideString;
var
  j,l:integer;
begin
  l:=length(atext);
  j:=ACount;
  if j>l then j:=l;
  Result:=Copy(AText,l-j+1,j);
end;


Function MidStr(const AText: WideString; const AStart, ACount: Integer): WideString;inline;
begin
  Result:=Copy(AText,AStart,ACount);
end;


{ ---------------------------------------------------------------------
    Extended search and replace
  ---------------------------------------------------------------------}

type
  TEqualFunction = function (const a,b : char) : boolean;

function EqualWithCase (const a,b : char) : boolean;
begin
  result := (a = b);
end;

function EqualWithoutCase (const a,b : char) : boolean;
begin
  result := (lowerCase(a) = lowerCase(b));
end;

function IsWholeWord (bufstart, bufend, wordstart, wordend : pchar) : boolean;
begin
            // Check start
  result := ((wordstart = bufstart) or ((wordstart-1)^ in worddelimiters)) and
            // Check end
            ((wordend = bufend) or ((wordend+1)^ in worddelimiters));
end;

function SearchDown(buf,aStart,endchar:pchar; SearchString:string;
    Equals : TEqualFunction; WholeWords:boolean) : pchar;
var Found : boolean;
    s, c : pchar;
begin
  result := aStart;
  Found := false;
  while not Found and (result <= endchar) do
    begin
    // Search first letter
    while (result <= endchar) and not Equals(result^,SearchString[1]) do
      inc (result);
    // Check if following is searchstring
    c := result;
    s := @(Searchstring[1]);
    Found := true;
    while (c <= endchar) and (s^ <> #0) and Found do
      begin
      Found := Equals(c^, s^);
      inc (c);
      inc (s);
      end;
    if s^ <> #0 then
      Found := false;
    // Check if it is a word
    if Found and WholeWords then
      Found := IsWholeWord(buf,endchar,result,c-1);
    if not found then
      inc (result);
    end;
  if not Found then
    result := nil;
end;

function SearchUp(buf,aStart,endchar:pchar; SearchString:string;
    equals : TEqualFunction; WholeWords:boolean) : pchar;
var Found : boolean;
    s, c, l : pchar;
begin
  result := aStart;
  Found := false;
  l := @(SearchString[length(SearchString)]);
  while not Found and (result >= buf) do
    begin
    // Search last letter
    while (result >= buf) and not Equals(result^,l^) do
      dec (result);
    // Check if before is searchstring
    c := result;
    s := l;
    Found := true;
    while (c >= buf) and (s >= @SearchString[1]) and Found do
      begin
      Found := Equals(c^, s^);
      dec (c);
      dec (s);
      end;
    if (s >= @(SearchString[1])) then
      Found := false;
    // Check if it is a word
    if Found and WholeWords then
      Found := IsWholeWord(buf,endchar,c+1,result);
    if found then
      result := c+1
    else
      dec (result);
    end;
  if not Found then
    result := nil;
end;

//function SearchDown(buf,aStart,endchar:pchar; SearchString:string; equal : TEqualFunction; WholeWords:boolean) : pchar;
function SearchBuf(Buf: PChar;BufLen: Integer;SelStart: Integer;SelLength: Integer;
    SearchString: String;Options: TStringSearchOptions):PChar;
var
  equal : TEqualFunction;
begin
  SelStart := SelStart + SelLength;
  if (SearchString = '') or (SelStart > BufLen) or (SelStart < 0) then
    result := nil
  else
    begin
    if soMatchCase in Options then
      Equal := @EqualWithCase
    else
      Equal := @EqualWithoutCase;
    if soDown in Options then
      result := SearchDown(buf,buf+SelStart,Buf+(BufLen-1), SearchString, Equal, (soWholeWord in Options))
    else
      result := SearchUp(buf,buf+SelStart,Buf+(Buflen-1), SearchString, Equal, (soWholeWord in Options));
    end;
end;


Function SearchBuf(Buf: PChar; BufLen: Integer; SelStart, SelLength: Integer; SearchString: String): PChar;inline; // ; Options: TStringSearchOptions = [soDown]
begin
  Result:=SearchBuf(Buf,BufLen,SelStart,SelLength,SearchString,[soDown]);
end;

Function PosEx(const SubStr, S: string; Offset: Cardinal): Integer;

var
  i,MaxLen, SubLen : SizeInt;
  SubFirst: Char;
  pc : pchar;
begin
<<<<<<< HEAD
  PosEx:=0;
  SubLen := Length(SubStr);
  if (SubLen > 0) and (Offset > 0) and (Offset <= Cardinal(Length(S))) then
   begin
    MaxLen := Length(S)- SubLen;
    SubFirst := SubStr[1];
    i := indexbyte(S[Offset],Length(S) - Offset + 1, Byte(SubFirst));
    while (i >= 0) and ((i + sizeint(Offset) - 1) <= MaxLen) do
    begin
      pc := @S[i+SizeInt(Offset)];
      //we know now that pc^ = SubFirst, because indexbyte returned a value > -1
      if (CompareByte(Substr[1],pc^,SubLen) = 0) then
      begin
        PosEx := i + SizeInt(Offset);
        Exit;
      end;
      //point Offset to next char in S
      Offset := sizeuint(i) + Offset + 1;
      i := indexbyte(S[Offset],Length(S) - Offset + 1, Byte(SubFirst));
    end;
  end;
=======
  if (offset<1) or (offset>SizeUInt(length(s))) then exit(0);
  i:=strpos(@s[offset],@substr[1]);
  if i=nil then
    PosEx:=0
  else
    PosEx:=succ(i-pchar(pointer(s)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function PosEx(c:char; const S: string; Offset: Cardinal): Integer;

var
  Len : longint;
  p: SizeInt;
begin
  Len := length(S);
  if (Offset < 1) or (Offset > SizeUInt(Length(S))) then exit(0);
  Len := length(S);
  p := indexbyte(S[Offset],Len-offset+1,Byte(c));
  if (p < 0) then
    PosEx := 0
  else
    PosEx := p + sizeint(Offset);
end; 

Function PosEx(const SubStr, S: string): Integer;inline; // Offset: Cardinal = 1
begin
  posex:=posex(substr,s,1);
end;

function StringsReplace(const S: string; OldPattern, NewPattern: array of string;  Flags: TReplaceFlags): string;

var pc,pcc,lastpc : pchar;
    strcount      : integer;
    ResStr,
    CompStr       : string;
    Found         : Boolean;
    sc            : integer;

begin
  sc := length(OldPattern);
  if sc <> length(NewPattern) then
    raise exception.Create(SErrAmountStrings);

  dec(sc);

  if rfIgnoreCase in Flags then
    begin
    CompStr:=AnsiUpperCase(S);
    for strcount := 0 to sc do
      OldPattern[strcount] := AnsiUpperCase(OldPattern[strcount]);
    end
  else
    CompStr := s;

  ResStr := '';
  pc := @CompStr[1];
  pcc := @s[1];
  lastpc := pc+Length(S);

<<<<<<< HEAD
=======
function StringsReplace(const S: string; OldPattern, NewPattern: array of string;  Flags: TReplaceFlags): string;

var pc,pcc,lastpc : pchar;
    strcount      : integer;
    ResStr,
    CompStr       : string;
    Found         : Boolean;
    sc            : integer;

<<<<<<< HEAD
=======
function StringsReplace(const S: string; OldPattern, NewPattern: array of string;  Flags: TReplaceFlags): string;

var pc,pcc,lastpc : pchar;
    strcount      : integer;
    ResStr,
    CompStr       : string;
    Found         : Boolean;
    sc            : integer;

>>>>>>> origin/fixes_2_2
begin
  sc := length(OldPattern);
  if sc <> length(NewPattern) then
    raise exception.Create(SErrAmountStrings);

  dec(sc);

  if rfIgnoreCase in Flags then
    begin
    CompStr:=AnsiUpperCase(S);
    for strcount := 0 to sc do
      OldPattern[strcount] := AnsiUpperCase(OldPattern[strcount]);
    end
  else
    CompStr := s;

  ResStr := '';
  pc := @CompStr[1];
  pcc := @s[1];
  lastpc := pc+Length(S);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  while pc < lastpc do
    begin
    Found := False;
    for strcount := 0 to sc do
      begin
      if (length(OldPattern[strcount])>0) and
         (OldPattern[strcount][1]=pc^) and
         (Length(OldPattern[strcount]) <= (lastpc-pc)) and
         (CompareByte(OldPattern[strcount][1],pc^,Length(OldPattern[strcount]))=0) then
        begin
        ResStr := ResStr + NewPattern[strcount];
        pc := pc+Length(OldPattern[strcount]);
        pcc := pcc+Length(OldPattern[strcount]);
        Found := true;
        end
      end;
    if not found then
      begin
      ResStr := ResStr + pcc^;
      inc(pc);
      inc(pcc);
      end
    else if not (rfReplaceAll in Flags) then
      begin
      ResStr := ResStr + StrPas(pcc);
      break;
      end;
    end;
  Result := ResStr;
end;

{ ---------------------------------------------------------------------
    Soundex Functions.
  ---------------------------------------------------------------------}
Const
SScore : array[1..255] of Char =
     ('0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0', // 1..32
      '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0', // 33..64
      '0','1','2','3','0','1','2','i','0','2','2','4','5','5','0','1','2','6','2','3','0','1','i','2','i','2', // 64..90
      '0','0','0','0','0','0', // 91..95
      '0','1','2','3','0','1','2','i','0','2','2','4','5','5','0','1','2','6','2','3','0','1','i','2','i','2', // 96..122
      '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0', // 123..154
      '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0', // 155..186
      '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0', // 187..218
      '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0', // 219..250
      '0','0','0','0','0'); // 251..255



Function Soundex(const AText: string; ALength: TSoundexLength): string;

Var
  S,PS : Char;
  I,L : integer;

begin
  Result:='';
  PS:=#0;
  If Length(AText)>0 then
    begin
    Result:=Upcase(AText[1]);
    I:=2;
    L:=Length(AText);
    While (I<=L) and (Length(Result)<ALength) do
      begin
      S:=SScore[Ord(AText[i])];
      If Not (S in ['0','i',PS]) then
        Result:=Result+S;
      If (S<>'i') then
        PS:=S;
      Inc(I);
      end;
    end;
  L:=Length(Result);
  If (L<ALength) then
    Result:=Result+StringOfChar('0',Alength-L);
end;



Function Soundex(const AText: string): string;inline; // ; ALength: TSoundexLength = 4

begin
  Result:=Soundex(AText,4);
end;

Const
  Ord0 = Ord('0');
  OrdA = Ord('A');

Function SoundexInt(const AText: string; ALength: TSoundexIntLength): Integer;

var
  SE: string;
  I: Integer;

begin
  Result:=-1;
  SE:=Soundex(AText,ALength);
  If Length(SE)>0 then
    begin
    Result:=Ord(SE[1])-OrdA;
    if ALength > 1 then
      begin
      Result:=Result*26+(Ord(SE[2])-Ord0);
      for I:=3 to ALength do
        Result:=(Ord(SE[I])-Ord0)+Result*7;
      end;
    Result:=ALength+Result*9;
    end;
end;


Function SoundexInt(const AText: string): Integer;inline; //; ALength: TSoundexIntLength = 4
begin
  Result:=SoundexInt(AText,4);
end;


Function DecodeSoundexInt(AValue: Integer): string;

var
  I, Len: Integer;

begin
  Result := '';
  Len := AValue mod 9;
  AValue := AValue div 9;
  for I:=Len downto 3 do
    begin
    Result:=Chr(Ord0+(AValue mod 7))+Result;
    AValue:=AValue div 7;
    end;
  if Len>1 then
    begin
    Result:=Chr(Ord0+(AValue mod 26))+Result;
    AValue:=AValue div 26;
    end;
  Result:=Chr(OrdA+AValue)+Result;
end;


Function SoundexWord(const AText: string): Word;

Var
  S : String;

begin
  S:=SoundEx(Atext,4);
  Result:=Ord(S[1])-OrdA;
  Result:=Result*26+ord(S[2])-48;
  Result:=Result*7+ord(S[3])-48;
  Result:=Result*7+ord(S[4])-48;
end;


Function DecodeSoundexWord(AValue: Word): string;
begin
  Result := Chr(Ord0+ (AValue mod 7));
  AValue := AValue div 7;
  Result := Chr(Ord0+ (AValue mod 7)) + Result;
  AValue := AValue div 7;
  Result := IntToStr(AValue mod 26) + Result;
  AValue := AValue div 26;
  Result := Chr(OrdA+AValue) + Result;
end;


Function SoundexSimilar(const AText, AOther: string; ALength: TSoundexLength): Boolean;inline;
begin
  Result:=Soundex(AText,ALength)=Soundex(AOther,ALength);
end;


Function SoundexSimilar(const AText, AOther: string): Boolean;inline; //; ALength: TSoundexLength = 4
begin
  Result:=SoundexSimilar(AText,AOther,4);
end;


Function SoundexCompare(const AText, AOther: string; ALength: TSoundexLength): Integer;inline;
begin
  Result:=AnsiCompareStr(Soundex(AText,ALength),Soundex(AOther,ALength));
end;


Function SoundexCompare(const AText, AOther: string): Integer;inline; //; ALength: TSoundexLength = 4
begin
  Result:=SoundexCompare(AText,AOther,4);
end;


Function SoundexProc(const AText, AOther: string): Boolean;
begin
  Result:=SoundexSimilar(AText,AOther);
end;

{ ---------------------------------------------------------------------
    RxStrUtils-like functions.
  ---------------------------------------------------------------------}


function IsEmptyStr(const S: string; const EmptyChars: TSysCharSet): Boolean;

var
  i,l: Integer;

begin
  l:=Length(S);
  i:=1;
  Result:=True;
  while Result and (i<=l) do
    begin
    Result:=(S[i] in EmptyChars);
    Inc(i);
    end;
end;

function DelSpace(const S: String): string;

begin
  Result:=DelChars(S,' ');
end;

function DelChars(const S: string; Chr: Char): string;

var
  I,J: Integer;

begin
  Result:=S;
  I:=Length(Result);
  While I>0 do
    begin
    if Result[I]=Chr then
      begin
      J:=I-1;
      While (J>0) and (Result[J]=Chr) do
        Dec(j);
      Delete(Result,J+1,I-J);
      I:=J+1;
      end;
    dec(I);
    end;
end;

function DelSpace1(const S: string): string;

var
  i: Integer;

begin
  Result:=S;
  for i:=Length(Result) downto 2 do
    if (Result[i]=' ') and (Result[I-1]=' ') then
      Delete(Result,I,1);
end;

function Tab2Space(const S: string; Numb: Byte): string;

var
  I: Integer;

begin
  I:=1;
  Result:=S;
  while I <= Length(Result) do
    if Result[I]<>Chr(9) then
      inc(I)
    else
      begin
      Result[I]:=' ';
      If (Numb>1) then
        Insert(StringOfChar(' ',Numb-1),Result,I);
      Inc(I,Numb);
      end;
end;

function NPos(const C: string; S: string; N: Integer): Integer;

var
  i,p,k: Integer;

begin
  Result:=0;
  if N<1 then
    Exit;
  k:=0;
  i:=1;
  Repeat
    p:=pos(C,S);
    Inc(k,p);
    if p>0 then
      delete(S,1,p);
    Inc(i);
  Until (i>n) or (p=0);
  If (P>0) then
    Result:=K;
end;

function AddChar(C: Char; const S: string; N: Integer): string;

Var
  l : Integer;

begin
  Result:=S;
  l:=Length(Result);
  if l<N then
    Result:=StringOfChar(C,N-l)+Result;
end;

function AddCharR(C: Char; const S: string; N: Integer): string;

Var
  l : Integer;

begin
  Result:=S;
  l:=Length(Result);
  if l<N then
    Result:=Result+StringOfChar(C,N-l);
end;


function PadRight(const S: string; N: Integer): string;inline;
begin
  Result:=AddCharR(' ',S,N);
end;


function PadLeft(const S: string; N: Integer): string;inline;
begin
  Result:=AddChar(' ',S,N);
end;


function Copy2Symb(const S: string; Symb: Char): string;

var
  p: Integer;

begin
  p:=Pos(Symb,S);
  if p=0 then
    p:=Length(S)+1;
  Result:=Copy(S,1,p-1);
end;

function Copy2SymbDel(var S: string; Symb: Char): string;

var
  p: Integer;

begin
  p:=Pos(Symb,S);
  if p=0 then
    begin
      result:=s;
      s:='';
    end
  else
    begin	
      Result:=Copy(S,1,p-1);
      delete(s,1,p);		
    end;
end;

function Copy2Space(const S: string): string;inline;
begin
  Result:=Copy2Symb(S,' ');
end;

function Copy2SpaceDel(var S: string): string;inline;
begin
  Result:=Copy2SymbDel(S,' ');
end;

function AnsiProperCase(const S: string; const WordDelims: TSysCharSet): string;

var
//  l :  Integer;
  P,PE : PChar;

begin
  Result:=AnsiLowerCase(S);
  P:=PChar(pointer(Result));
  PE:=P+Length(Result);
  while (P<PE) do
    begin
    while (P<PE) and (P^ in WordDelims) do
      inc(P);
    if (P<PE) then
      P^:=UpCase(P^);
    while (P<PE) and not (P^ in WordDelims) do
      inc(P);
    end;
end;

function WordCount(const S: string; const WordDelims: TSysCharSet): Integer;

var
  P,PE : PChar;

begin
  Result:=0;
  P:=Pchar(pointer(S));
  PE:=P+Length(S);
  while (P<PE) do
    begin
    while (P<PE) and (P^ in WordDelims) do
      Inc(P);
    if (P<PE) then
      inc(Result);
    while (P<PE) and not (P^ in WordDelims) do
      inc(P);
    end;
end;

function WordPosition(const N: Integer; const S: string; const WordDelims: TSysCharSet): Integer;

var
  PS,P,PE : PChar;
  Count: Integer;

begin
  Result:=0;
  Count:=0;
  PS:=PChar(pointer(S));
  PE:=PS+Length(S);
  P:=PS;
  while (P<PE) and (Count<>N) do
    begin
    while (P<PE) and (P^ in WordDelims) do
      inc(P);
    if (P<PE) then
      inc(Count);
    if (Count<>N) then
      while (P<PE) and not (P^ in WordDelims) do
        inc(P)
    else
      Result:=(P-PS)+1;
    end;
end;


function ExtractWord(N: Integer; const S: string; const WordDelims: TSysCharSet): string;inline;
var
  i: Integer;
begin
  Result:=ExtractWordPos(N,S,WordDelims,i);
end;


function ExtractWordPos(N: Integer; const S: string; const WordDelims: TSysCharSet; var Pos: Integer): string;
var
  i,j,l: Integer;
begin
  j:=0;
  i:=WordPosition(N, S, WordDelims);
  Pos:=i;
  if (i<>0) then
    begin
    j:=i;
    l:=Length(S);
    while (j<=L) and not (S[j] in WordDelims) do
      inc(j);
    end;
  SetLength(Result,j-i);
  If ((j-i)>0) then
    Move(S[i],Result[1],j-i);
end;

function ExtractDelimited(N: Integer; const S: string; const Delims: TSysCharSet): string;
var
  w,i,l,len: Integer;
begin
  w:=0;
  i:=1;
  l:=0;
  len:=Length(S);
  SetLength(Result, 0);
  while (i<=len) and (w<>N) do
    begin
    if s[i] in Delims then
      inc(w)
    else
      begin
      if (N-1)=w then
        begin
        inc(l);
        SetLength(Result,l);
        Result[L]:=S[i];
        end;
      end;
    inc(i);
    end;
end;

function ExtractSubstr(const S: string; var Pos: Integer; const Delims: TSysCharSet): string;

var
  i,l: Integer;

begin
  i:=Pos;
  l:=Length(S);
  while (i<=l) and not (S[i] in Delims) do
    inc(i);
  Result:=Copy(S,Pos,i-Pos);
  while (i<=l) and (S[i] in Delims) do
    inc(i);
  Pos:=i;
end;

function isWordPresent(const W, S: string; const WordDelims: TSysCharSet): Boolean;

var
  i,Count : Integer;

begin
  Result:=False;
  Count:=WordCount(S, WordDelims);
  I:=1;
  While (Not Result) and (I<=Count) do
    begin
    Result:=ExtractWord(i,S,WordDelims)=W;
    Inc(i);
    end;
end;


function Numb2USA(const S: string): string;
var
  i, NA: Integer;
begin
  i:=Length(S);
  Result:=S;
  NA:=0;
  while (i > 0) do begin
    if ((Length(Result) - i + 1 - NA) mod 3 = 0) and (i <> 1) then
    begin
      insert(',', Result, i);
      inc(NA);
    end;
    Dec(i);
  end;
end;

function PadCenter(const S: string; Len: Integer): string;
begin
  if Length(S)<Len then
    begin
    Result:=StringOfChar(' ',(Len div 2) -(Length(S) div 2))+S;
    Result:=Result+StringOfChar(' ',Len-Length(Result));
    end
  else
    Result:=S;
end;


function Dec2Numb(N: Longint; Len, Base: Byte): string;

var
  C: Integer;
  Number: Longint;

begin
  if N=0 then
    Result:='0'
  else
    begin
    Number:=N;
    Result:='';
    while Number>0 do
      begin
      C:=Number mod Base;
      if C>9 then
        C:=C+55
      else
        C:=C+48;
      Result:=Chr(C)+Result;
      Number:=Number div Base;
      end;
    end;
  if (Result<>'') then
    Result:=AddChar('0',Result,Len);
end;

function Numb2Dec(S: string; Base: Byte): Longint;

var
  i, P: Longint;

begin
  i:=Length(S);
  Result:=0;
  S:=UpperCase(S);
  P:=1;
  while (i>=1) do
    begin
    if (S[i]>'@') then
      Result:=Result+(Ord(S[i])-55)*P
    else
      Result:=Result+(Ord(S[i])-48)*P;
    Dec(i);
    P:=P*Base;
    end;
end;

function RomanToint(const S: string): Longint;

const
  RomanChars  = ['C','D','I','L','M','V','X'];
  RomanValues : array['C'..'X'] of Word
              = (100,500,0,0,0,0,1,0,0,50,1000,0,0,0,0,0,0,0,0,5,0,10);

var
  index, Next: Char;
  i,l: Integer;
  Negative: Boolean;

begin
  Result:=0;
  i:=0;
  Negative:=(Length(S)>0) and (S[1]='-');
  if Negative then
    inc(i);
  l:=Length(S);
  while (i<l) do
    begin
    inc(i);
    index:=UpCase(S[i]);
    if index in RomanChars then
      begin
      if Succ(i)<=l then
        Next:=UpCase(S[i+1])
      else
        Next:=#0;
      if (Next in RomanChars) and (RomanValues[index]<RomanValues[Next]) then
        begin
        inc(Result, RomanValues[Next]);
        Dec(Result, RomanValues[index]);
        inc(i);
        end
      else
        inc(Result, RomanValues[index]);
      end
    else
      begin
      Result:=0;
      Exit;
      end;
    end;
  if Negative then
    Result:=-Result;
end;

function intToRoman(Value: Longint): string;

const
  Arabics : Array[1..13] of Integer
          = (1,4,5,9,10,40,50,90,100,400,500,900,1000);
  Romans  :  Array[1..13] of String
          = ('I','IV','V','IX','X','XL','L','XC','C','CD','D','CM','M');

var
  i: Integer;

begin
  Result:='';
  for i:=13 downto 1 do
    while (Value >= Arabics[i]) do
      begin
        Value:=Value-Arabics[i];
        Result:=Result+Romans[i];
      end;
end;

function intToBin(Value: Longint; Digits, Spaces: Integer): string;
var endpos : integer;
    p,p2:pchar;
    k: integer;
begin
  Result:='';
  if (Digits>32) then
    Digits:=32;
  if (spaces=0) then
   begin
     result:=inttobin(value,digits);
     exit;
   end;
  endpos:=digits+ (digits-1) div spaces;
  setlength(result,endpos);
  p:=@result[endpos];
  p2:=@result[1];
  k:=spaces;
  while (p>=p2) do
<<<<<<< HEAD
    begin
      if k=0 then
       begin
         p^:=' ';
         dec(p);
         k:=spaces;
       end;
      p^:=chr(48+(cardinal(value) and 1));
      value:=cardinal(value) shr 1;
      dec(p); 
      dec(k);
   end;
end;

function intToBin(Value: Longint; Digits:integer): string;
var p,p2 : pchar;
begin
  result:='';
  if digits<=0 then exit;
  setlength(result,digits);
  p:=pchar(pointer(@result[digits]));
  p2:=pchar(pointer(@result[1]));
  // typecasts because we want to keep intto* delphi compat and take an integer
  while (p>=p2) and (cardinal(value)>0) do     
    begin
       p^:=chr(48+(cardinal(value) and 1));
       value:=cardinal(value) shr 1;
       dec(p); 
<<<<<<< HEAD
=======
    begin
      if k=0 then
       begin
         p^:=' ';
         dec(p);
         k:=spaces;
       end;
      p^:=chr(48+(cardinal(value) and 1));
      value:=cardinal(value) shr 1;
      dec(p); 
      dec(k);
   end;
end;

function intToBin(Value: Longint; Digits:integer): string;
var p,p2 : pchar;
begin
  result:='';
  if digits<=0 then exit;
  setlength(result,digits);
  p:=pchar(pointer(@result[digits]));
  p2:=pchar(pointer(@result[1]));
  // typecasts because we want to keep intto* delphi compat and take an integer
  while (p>=p2) and (cardinal(value)>0) do     
    begin
       p^:=chr(48+(cardinal(value) and 1));
       value:=cardinal(value) shr 1;
       dec(p); 
>>>>>>> origin/fixes_2_2
    end;
  digits:=p-p2+1;
  if digits>0 then
    fillchar(result[1],digits,#48);
end;

function intToBin(Value: int64; Digits:integer): string;
var p,p2 : pchar;
begin
  result:='';
  if digits<=0 then exit;
  setlength(result,digits);
  p:=pchar(pointer(@result[digits]));
  p2:=pchar(pointer(@result[1]));
  // typecasts because we want to keep intto* delphi compat and take a signed val
  // and avoid warnings
  while (p>=p2) and (qword(value)>0) do     
    begin
       p^:=chr(48+(cardinal(value) and 1));
       value:=qword(value) shr 1;
       dec(p); 
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  digits:=p-p2+1;
  if digits>0 then
    fillchar(result[1],digits,#48);
<<<<<<< HEAD
<<<<<<< HEAD
=======
end;

function intToBin(Value: int64; Digits:integer): string;
var p,p2 : pchar;
begin
  result:='';
  if digits<=0 then exit;
  setlength(result,digits);
  p:=pchar(pointer(@result[digits]));
  p2:=pchar(pointer(@result[1]));
  // typecasts because we want to keep intto* delphi compat and take a signed val
  // and avoid warnings
  while (p>=p2) and (qword(value)>0) do     
    begin
       p^:=chr(48+(cardinal(value) and 1));
       value:=qword(value) shr 1;
       dec(p); 
    end;
  digits:=p-p2+1;
  if digits>0 then
    fillchar(result[1],digits,#48);
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


function FindPart(const HelpWilds, inputStr: string): Integer;
var
  i, J: Integer;
  Diff: Integer;
begin
  Result:=0;
  i:=Pos('?',HelpWilds);
  if (i=0) then
    Result:=Pos(HelpWilds, inputStr)
  else
    begin
    Diff:=Length(inputStr) - Length(HelpWilds);
    for i:=0 to Diff do
      begin
      for J:=1 to Length(HelpWilds) do
        if (inputStr[i + J] = HelpWilds[J]) or (HelpWilds[J] = '?') then
          begin
          if (J=Length(HelpWilds)) then
            begin
            Result:=i+1;
            Exit;
            end;
          end
        else
          Break;
      end;
    end;
end;

function isWild(inputStr, Wilds: string; ignoreCase: Boolean): Boolean;

 function SearchNext(var Wilds: string): Integer;

 begin
   Result:=Pos('*', Wilds);
   if Result>0 then
     Wilds:=Copy(Wilds,1,Result - 1);
 end;

var
  CWild, CinputWord: Integer; { counter for positions }
  i, LenHelpWilds: Integer;
  MaxinputWord, MaxWilds: Integer; { Length of inputStr and Wilds }
  HelpWilds: string;
begin
  if Wilds = inputStr then begin
    Result:=True;
    Exit;
  end;
  repeat { delete '**', because '**' = '*' }
    i:=Pos('**', Wilds);
    if i > 0 then
      Wilds:=Copy(Wilds, 1, i - 1) + '*' + Copy(Wilds, i + 2, Maxint);
  until i = 0;
  if Wilds = '*' then begin { for fast end, if Wilds only '*' }
    Result:=True;
    Exit;
  end;
  MaxinputWord:=Length(inputStr);
  MaxWilds:=Length(Wilds);
  if ignoreCase then begin { upcase all letters }
    inputStr:=AnsiUpperCase(inputStr);
    Wilds:=AnsiUpperCase(Wilds);
  end;
  if (MaxWilds = 0) or (MaxinputWord = 0) then begin
    Result:=False;
    Exit;
  end;
  CinputWord:=1;
  CWild:=1;
  Result:=True;
  repeat
    if inputStr[CinputWord] = Wilds[CWild] then begin { equal letters }
      { goto next letter }
      inc(CWild);
      inc(CinputWord);
      Continue;
    end;
    if Wilds[CWild] = '?' then begin { equal to '?' }
      { goto next letter }
      inc(CWild);
      inc(CinputWord);
      Continue;
    end;
    if Wilds[CWild] = '*' then begin { handling of '*' }
      HelpWilds:=Copy(Wilds, CWild + 1, MaxWilds);
      i:=SearchNext(HelpWilds);
      LenHelpWilds:=Length(HelpWilds);
      if i = 0 then begin
        { no '*' in the rest, compare the ends }
        if HelpWilds = '' then Exit; { '*' is the last letter }
        { check the rest for equal Length and no '?' }
        for i:=0 to LenHelpWilds - 1 do begin
          if (HelpWilds[LenHelpWilds - i] <> inputStr[MaxinputWord - i]) and
            (HelpWilds[LenHelpWilds - i]<> '?') then
          begin
            Result:=False;
            Exit;
          end;
        end;
        Exit;
      end;
      { handle all to the next '*' }
      inc(CWild, 1 + LenHelpWilds);
      i:=FindPart(HelpWilds, Copy(inputStr, CinputWord, Maxint));
      if i= 0 then begin
        Result:=False;
        Exit;
      end;
      CinputWord:=i + LenHelpWilds;
      Continue;
    end;
    Result:=False;
    Exit;
  until (CinputWord > MaxinputWord) or (CWild > MaxWilds);
  { no completed evaluation }
  if CinputWord <= MaxinputWord then Result:=False;
  if (CWild <= MaxWilds) and (Wilds[MaxWilds] <> '*') then Result:=False;
end;

function XorString(const Key, Src: ShortString): ShortString;
var
  i: Integer;
begin
  Result:=Src;
  if Length(Key) > 0 then
    for i:=1 to Length(Src) do
      Result[i]:=Chr(Byte(Key[1 + ((i - 1) mod Length(Key))]) xor Ord(Src[i]));
end;

function XorEncode(const Key, Source: string): string;

var
  i: Integer;
  C: Byte;

begin
  Result:='';
  for i:=1 to Length(Source) do
    begin
    if Length(Key) > 0 then
      C:=Byte(Key[1 + ((i - 1) mod Length(Key))]) xor Byte(Source[i])
    else
      C:=Byte(Source[i]);
    Result:=Result+AnsiLowerCase(intToHex(C, 2));
    end;
end;

function XorDecode(const Key, Source: string): string;
var
  i: Integer;
  C: Char;
begin
  Result:='';
  for i:=0 to Length(Source) div 2 - 1 do
    begin
    C:=Chr(StrTointDef('$' + Copy(Source, (i * 2) + 1, 2), Ord(' ')));
    if Length(Key) > 0 then
      C:=Chr(Byte(Key[1 + (i mod Length(Key))]) xor Byte(C));
    Result:=Result + C;
    end;
end;

function GetCmdLineArg(const Switch: string; SwitchChars: TSysCharSet): string;
var
  i: Integer;
  S: string;
begin
  i:=1;
  Result:='';
  while (Result='') and (i<=ParamCount) do
    begin
    S:=ParamStr(i);
    if (SwitchChars=[]) or ((S[1] in SwitchChars) and (Length(S) > 1)) and
       (AnsiCompareText(Copy(S,2,Length(S)-1),Switch)=0) then
      begin
      inc(i);
      if i<=ParamCount then
        Result:=ParamStr(i);
      end;
    inc(i);
    end;
end;

Function RPosEX(C:char;const S : AnsiString;offs:cardinal):Integer; overload;

var I   : SizeUInt;
    p,p2: pChar;

Begin
 I:=Length(S);
 If (I<>0) and (offs<=i) Then
   begin
     p:=@s[offs];
     p2:=@s[1];
     while (p2<=p) and (p^<>c) do dec(p);
     RPosEx:=(p-p2)+1;
   end
  else
    RPosEX:=0;
End;

Function RPos(c:char;const S : AnsiString):Integer; overload;

var I   : Integer;
    p,p2: pChar;

Begin
 I:=Length(S);
 If I<>0 Then
   begin
     p:=@s[i];
     p2:=@s[1];
     while (p2<=p) and (p^<>c) do dec(p);
     i:=p-p2+1;
   end;
  RPos:=i;
End;

Function RPos (Const Substr : AnsiString; Const Source : AnsiString) : Integer; overload;
var
  MaxLen,llen : Integer;
  c : char;
  pc,pc2 : pchar;
begin
  rPos:=0;
  llen:=Length(SubStr);
  maxlen:=length(source);
  if (llen>0) and (maxlen>0) and ( llen<=maxlen) then
   begin
 //    i:=maxlen;
     pc:=@source[maxlen];
     pc2:=@source[llen-1];
     c:=substr[llen];
     while pc>=pc2 do
      begin
        if (c=pc^) and
           (CompareChar(Substr[1],pchar(pc-llen+1)^,Length(SubStr))=0) then
         begin
           rPos:=pchar(pc-llen+1)-pchar(@source[1])+1;
           exit;
         end;
        dec(pc);
      end;
   end;
end;

Function RPosex (Const Substr : AnsiString; Const Source : AnsiString;offs:cardinal) : Integer; overload;
var
  MaxLen,llen : Integer;
  c : char;
  pc,pc2 : pchar;
begin
  rPosex:=0;
  llen:=Length(SubStr);
  maxlen:=length(source);
  if SizeInt(offs)<maxlen then maxlen:=offs;
  if (llen>0) and (maxlen>0) and ( llen<=maxlen)  then
   begin
//     i:=maxlen;
     pc:=@source[maxlen];
     pc2:=@source[llen-1];
     c:=substr[llen];
     while pc>=pc2 do
      begin
        if (c=pc^) and
           (CompareChar(Substr[1],pchar(pc-llen+1)^,Length(SubStr))=0) then
         begin
           rPosex:=pchar(pc-llen+1)-pchar(@source[1])+1;
           exit;
         end;
        dec(pc);
      end;
   end;
end;

// def from delphi.about.com:
procedure BinToHex(BinValue, HexValue: PChar; BinBufSize: Integer);

Const
  HexDigits='0123456789ABCDEF';
var
  i : longint;
begin
  for i:=0 to binbufsize-1 do
    begin
    HexValue[0]:=hexdigits[1+((ord(binvalue^) shr 4))];
    HexValue[1]:=hexdigits[1+((ord(binvalue^) and 15))];
    inc(hexvalue,2);
    inc(binvalue);
    end;
end;


function HexToBin(HexValue, BinValue: PChar; BinBufSize: Integer): Integer;
// more complex, have to accept more than bintohex
// A..F    1000001
// a..f    1100001
// 0..9     110000

var i,j,h,l : integer;

begin
  i:=binbufsize;
  while (i>0) do
    begin
    if hexvalue^ IN ['A'..'F','a'..'f'] then
      h:=((ord(hexvalue^)+9) and 15)
    else if hexvalue^ IN ['0'..'9'] then
      h:=((ord(hexvalue^)) and 15)
    else
      break;
    inc(hexvalue);
    if hexvalue^ IN ['A'..'F','a'..'f'] then
      l:=(ord(hexvalue^)+9) and 15
    else if hexvalue^ IN ['0'..'9'] then
      l:=(ord(hexvalue^)) and 15
    else
      break;
    j := l + (h shl 4);
    inc(hexvalue);
    binvalue^:=chr(j);
    inc(binvalue);
    dec(i);
    end;
  result:=binbufsize-i;
end;

function possetex (const c:TSysCharSet;const s : ansistring;count:Integer ):Integer;

var i,j:Integer;

begin
 if pchar(pointer(s))=nil then
  j:=0
 else
  begin
   i:=length(s);
   j:=count;
   if j>i then
    begin
     result:=0;
     exit;
    end;
   while (j<=i) and (not (s[j] in c)) do inc(j);
   if (j>i) then
    j:=0;                                         // not found.
  end;
 result:=j;
end;

function posset (const c:TSysCharSet;const s : ansistring ):Integer;

begin
  result:=possetex(c,s,1);
end;

function possetex (const c:string;const s : ansistring;count:Integer ):Integer;

var cset : TSysCharSet;
    i    : integer;
begin
  cset:=[];
  if length(c)>0 then
  for i:=1 to length(c) do
    include(cset,c[i]);
  result:=possetex(cset,s,count);
end;

function posset (const c:string;const s : ansistring ):Integer;

var cset : TSysCharSet;
    i    : integer;
begin
  cset:=[];
  if length(c)>0 then
    for i:=1 to length(c) do
      include(cset,c[i]);
  result:=possetex(cset,s,1);
end;


Procedure Removeleadingchars(VAR S : AnsiString; Const CSet:TSysCharset);

VAR I,J : Longint;

Begin
 I:=Length(S); 
 IF (I>0) Then
  Begin
   J:=1;
   While (J<=I) And (S[J] IN CSet) DO 
     INC(J);
   IF J>1 Then
    Delete(S,1,J-1);
   End;
End;


function TrimLeftSet(const S: String;const CSet:TSysCharSet): String;

begin
  result:=s;
  removeleadingchars(result,cset); 
end;

Procedure RemoveTrailingChars(VAR S : AnsiString;Const CSet:TSysCharset);

VAR I,J: LONGINT;

Begin
 I:=Length(S);
 IF (I>0) Then
  Begin
   J:=I;
   While (j>0) and (S[J] IN CSet) DO DEC(J);
   IF J<>I Then
    SetLength(S,J);
  End;
End;

Function TrimRightSet(const S: String;const CSet:TSysCharSet): String;

begin
  result:=s;
  RemoveTrailingchars(result,cset); 
end;

Procedure RemovePadChars(VAR S : AnsiString;Const CSet:TSysCharset);

VAR I,J,K: LONGINT;

Begin
 I:=Length(S);
 IF (I>0) Then
  Begin
   J:=I;
   While (j>0) and (S[J] IN CSet) DO DEC(J);
   if j=0 Then
     begin 
       s:='';
       exit;
     end;
   k:=1;
   While (k<=I) And (S[k] IN CSet) DO 
     INC(k);
   IF k>1 Then
     begin
       move(s[k],s[1],j-k+1);
       setlength(s,j-k+1);
     end
   else
     setlength(s,j);  
  End;
End;

function TrimSet(const S: String;const CSet:TSysCharSet): String;

begin
  result:=s;
  RemovePadChars(result,cset); 
end;


end.
