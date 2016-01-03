{
Author Mazen NEIFER
Licence LGPL
}
unit UTF8BIDI;

{$mode objfpc}{$H+}

interface

uses
  FreeBIDI;

type
  TUCS32Char = Cardinal;
  TUCS16Char = Word;
  TUTF8Char = String[4];
  TUTF8String = UTF8String;

{****************************Conversion routines*******************************}
{Converts an UCS 16/32 bits charcater to UTF8 character}
function UnicodeToUTF8(aChar:TUCS32Char):TUTF8Char;
{Converts a wide char UCS 16 bits chcarcter to UTF8 character}
function UnicodeToUTF8(aChar:WideChar):TUTF8Char;
{Converts a wide char UCS 16 bits string to UTF8 character}
function UnicodeToUTF8(const Src:TString):TUTF8String;
{Converts an UTF8 character to UCS 32 bits character}
function UTF8ToUCS32(const UTF8Char:TUTF8Char):TUCS32Char;
{Converts an UTF8 character to UCS 16 bits character}
function UTF8ToUCS16(const UTF8Char:TUTF8Char):TUCS16Char;
{Converts an UTF8 string to UCS 16 bits string}
function UTF8ToUnicode(const Src:TUTF8String):TString;
{Converts an UTF8 string to a double byte string}
function UTF8ToDoubleByteString(const UTF8Str:TUTF8String):String;
function UTF8ToDoubleByte(UTF8Str:PChar; Len:Cardinal; DBStr:PByte):Cardinal;
{****************************Logical aspects***********************************}
{Returns the number of logical characters}
function LLength(const UTF8Str:TUTF8String):Cardinal;
{Converts visual position to logical position}
function LPos(const UTF8Str:TUTF8String; vp:Integer; pDir:TDirection):Cardinal;
{Returns character at a given logical position according to paragraph direction}
function LCharOf(UTF8String:TUTF8String; lp:Integer):TUTF8Char;
{****************************Visual aspects************************************}
{Returns the number of visual characters}
function VLength(const Src:TUTF8String; pDir:TDirection):Cardinal;
{Converts a logical position to a visual position}
function VPos(const UTF8Str:TUTF8String; lp:Integer; pDir, cDir:TDirection):Cardinal;
{Returns character at a given visual position according to paragraph direction}
function VCharOf(UTF8Str:TUTF8String; vp:Integer; dir:TDirection):TUTF8Char;
{Inserts a string into another paying attention of RTL/LTR direction}
procedure VInsert(const Src:TUTF8String; var Dest:TUTF8String; vp:Integer; pDir:TDirection);
{Deletes a string into another paying attention of RTL/LTR direction}
procedure VDelete(var str:TUTF8String; vp, len:Integer; pDir:TDirection);
{****************************Helper routines***********************************}
{Returns direction of a character}
function DirectionOf(Character:TUTF8Char):TDirection;
{Returns contextual direction of caracter in a string}
function DirectionOf(UTF8String:TUTF8String; lp:Integer; pDir:TDirection):TDirection;
{Inserts a char as if it was typed using keyboard in the most user friendly way.
Returns the new cursor position after insersion depending on the new visual text}
function InsertChar(Src:TUTF8Char; var Dest:TUTF8String; vp:Integer; pDir:TDirection):Integer;
{Returns a table mapping each visual position to its logical position in an UTF8*
string}
function VisualToLogical(const UTF8String:TUTF8String; pDir:TDirection):TVisualToLogical;

implementation

function DumpStr(const s:TUTF8String):String;
var
  i:Integer;
begin
  Result := '';
  for i:= 1 to Length(s) do
    case s[i] of
      #0..#127:
         Result := Result + s[i];
    else
      Result := Result + '$' + HexStr(Ord(s[i]),2);
    end;
end;
function ComputeCharLength(p:PChar):Cardinal;
begin
  if ord(p^)<%11000000
  then
{regular single byte character (#0 is a normal char, this is UTF8Charascal ;)}
    Result:=1
  else if ((ord(p^) and %11100000) = %11000000)
  then
    if (ord(p[1]) and %11000000) = %10000000 then
      Result:=2
    else
      Result:=1
  else if ((ord(p^) and %11110000) = %11100000)
  then
    if ((ord(p[1]) and %11000000) = %10000000)
      and ((ord(p[2]) and %11000000) = %10000000)
    then
      Result:=3
    else
        Result:=1
  else if ((ord(p^) and %11111000) = %11110000)
  then
    if ((ord(p[1]) and %11000000) = %10000000)
    and ((ord(p[2]) and %11000000) = %10000000)
    and ((ord(p[3]) and %11000000) = %10000000)
    then
      Result:=4
    else
      Result:=1
  else
    Result:=1
end;

{****************************Conversion routines*******************************}
function UnicodeToUTF8(aChar:TUCS32Char):TUTF8Char;
begin
  case aChar of
    0..$7f:
      begin
        Result[1]:=char(aChar);
        SetLength(UnicodeToUTF8,1);
      end;
    $80..$7ff:
      begin
        Result[1]:=char($c0 or (aChar shr 6));
        Result[2]:=char($80 or (aChar and $3f));
        SetLength(UnicodeToUTF8,2);
      end;
    $800..$ffff:
      begin
        SetLength(Result,3);
        Result[1]:=char($e0 or (aChar shr 12));
        Result[2]:=char($80 or ((aChar shr 6) and $3f));
        Result[3]:=char($80 or (aChar and $3f));
      end;
    $10000..$1fffff:
      begin
        SetLength(UnicodeToUTF8,4);
        Result[1]:=char($f0 or (aChar shr 18));
        Result[2]:=char($80 or ((aChar shr 12) and $3f));
        Result[3]:=char($80 or ((aChar shr 6) and $3f));
        Result[4]:=char($80 or (aChar and $3f));
      end;
  else
    SetLength(UnicodeToUTF8, 0);
  end;
end;

function UnicodeToUTF8(aChar:WideChar):TUTF8Char;
var
  c:TUCS16Char absolute aChar;
begin
  case c of
    0..$7f:
      begin
        Result[1]:=char(c);
        SetLength(UnicodeToUTF8,1);
      end;
    $80..$7ff:
      begin
        Result[1]:=char($c0 or (c shr 6));
        Result[2]:=char($80 or (c and $3f));
        SetLength(UnicodeToUTF8,2);
      end;
  else
    SetLength(UnicodeToUTF8, 0);
  end;
end;

function UnicodeToUTF8(const Src:TString):TUTF8String;
var
  vp:Integer;
begin
  vp := 1;
  Result := '';
  for vp := 1 to Length(Src) do
    Result := Result + UnicodeToUTF8(Src[vp]);
end;

function UTF8ToUCS32(const UTF8Char:TUTF8Char):TUCS32Char;
begin
  case ComputeCharLength(@UTF8Char[1]) of
    1:{regular single byte character (#0 is a normal char, this is UTF8Charascal ;)}
      Result := ord(UTF8Char[1]);
    2:
      Result := ((ord(UTF8Char[1]) and %00011111) shl 6)
                or (ord(UTF8Char[2]) and %00111111);
    3:
      Result := ((ord(UTF8Char[1]) and %00011111) shl 12)
                or ((ord(UTF8Char[1]) and %00111111) shl 6)
                or (ord(UTF8Char[2]) and %00111111);
    4:
      Result := ((ord(UTF8Char[1]) and %00011111) shl 18)
                or ((ord(UTF8Char[2]) and %00111111) shl 12)
                or ((ord(UTF8Char[3]) and %00111111) shl 6)
                or (ord(UTF8Char[4]) and %00111111);
  else
    Result := $FFFFFFFF;
  end
end;

function UTF8ToUCS16(const UTF8Char:TUTF8Char):TUCS16Char;
begin
  case Length(UTF8Char) of
    1:{regular single byte character (#0 is a normal char, this is UTF8Charascal ;)}
      Result := ord(UTF8Char[1]);
    2:
      Result := ((ord(UTF8Char[1]) and %00011111) shl 6)
                or (ord(UTF8Char[2]) and %00111111);
  else
    Result := $FFFF;
  end;
end;


function UTF8ToUnicode(const Src:TUTF8String):TString;
var
  lp, vp:Integer;
  c:TUTF8Char;
begin
  lp := 1;
  vp := 0;
  SetLength(Result, Length(Src));
  while lp <= Length(Src) do
  begin
    vp := vp + 1;
    c := LCharOf(Src, lp);
    Result[vp] := WideChar(UTF8ToUCS16(c));
    lp := lp + Length(c);
  end;
  SetLength(Result, vp);
end;

function UTF8ToDoubleByteString(const UTF8Str: TUTF8String): string;
var
  Len: Integer;
begin
  Len:=VLength(UTF8Str, drLTR);
  SetLength(Result,Len*2);
  if Len=0 then exit;
  UTF8ToDoubleByte(PChar(UTF8Str),length(UTF8Str),PByte(Result));
end;

function UTF8ToDoubleByte(UTF8Str: PChar; Len:Cardinal; DBStr: PByte):Cardinal;
var
  SrcPos: PChar;
  CharLen: LongInt;
  DestPos: PByte;
  u: Cardinal;
begin
  SrcPos:=UTF8Str;
  DestPos:=DBStr;
  Result:=0;
  while Len>0 do begin
    u:=UTF8ToUCS32(SrcPos);
    DestPos^:=byte((u shr 8) and $ff);
    inc(DestPos);
    DestPos^:=byte(u and $ff);
    inc(DestPos);
    inc(SrcPos,CharLen);
    dec(Len,CharLen);
    inc(Result);
  end;
end;

{****************************Logical aspects***********************************}
function LLength(const UTF8Str:TUTF8String):Cardinal;
begin
  Result := Length(UTF8Str);
end;

function LPos(const UTF8Str:TUTF8String; vp:Integer; pDir:TDirection):Cardinal;
var
  v2l:TVisualToLogical;
  i:integer;
begin
  v2l := VisualToLogical(UTF8Str, pDir);
  if vp <= v2l[0]
  then
    Result := v2l[vp]
  else
    Result := Length(UTF8Str) + 1;
end;

function LCharOf(UTF8String:TUTF8String; lp:Integer):TUTF8Char;
begin
  if lp > Length(UTF8String)
  then
    Exit('');
  while(lp > 0) and ((Ord(UTF8String[lp]) and $F0) in [$80..$B0]) do
begin
    Dec(lp);
end;
  if lp = 0
  then
    Exit('');
  Move(UTF8String[lp], Result[1], SizeOf(TUTF8Char) - 1);
  SetLength(Result, ComputeCharLength(@Result[1]));
end;
{****************************Visual aspects************************************}
function VLength(const Src:TUTF8String; pDir:TDirection):Cardinal;
begin
  Result := FreeBIDI.VLength(UTF8ToUnicode(Src), pDir);
end;

function VPos(const UTF8Str:TUTF8String; lp:Integer; pDir, cDir:TDirection):Cardinal;
var
  v2l:TVisualToLogical;
  vp:Integer;
begin
  v2l := VisualToLogical(UTF8Str, pDir);
  for vp := 1 to v2l[0] do
  if lp = v2l[vp]
  then
    begin
      Exit(vp);
    end;
  Result := v2l[0];
end;

function VPos(UTF8Char:PChar; Len:integer; BytePos:integer):Cardinal;
begin
end;


function VCharOf(UTF8Str:TUTF8String; vp:Integer; dir:TDirection):TUTF8Char;
var
  CharLen: LongInt;
begin
  Result:=LCharOf(UTF8Str,LPos(UTF8Str, vp, dir));
end;

{****************************Helper routines***********************************}
function DirectionOf(Character:TUTF8Char):TDirection;
begin
  case Character[1] of
    #9,#32,
    '/',
    '{','}',
    '[',']',
    '(',')':
      Result := drNONE;
    #$D8,#$D9:
      Result := drRTL;
  else
    Result := drLTR;
  end;
end;

function DirectionOf(UTF8String:TUTF8String; lp:Integer; pDir:TDirection):TDirection;
var
  c:TUTF8Char;
  lDir,rDir:TDirection;
  p:Integer;
begin
  if(lp <= 0)
  then
    lp := 1;
{Seek for proper character direction}
  c := LCharOf(UTF8String, lp);
  lDir := DirectionOf(c);
{Seek for left character direction if it is neutral}
  p := lp;
  while(p > 1) and (lDir = drNONE)do
  begin
    c := LCharOf(UTF8String, p - 1);
    lDir := DirectionOf(c);
    p := p - Length(c);
  end;
{Seek for right character direction if it is neutral}
  p := lp;
  repeat
    c := LCharOf(UTF8String, p);
    rDir := DirectionOf(c);
    p := p + Length(c);
  until(p > Length(UTF8String)) or (rDir <> drNONE);
  if(lDir = rDir)
  then
    Result := rDir
  else
    Result := pDir;
end;

function VisualToLogical(const UTF8String:TUTF8String; pDir:TDirection):TVisualToLogical;
  procedure Insert(value:Byte; var v2l:TVisualToLogical; InsPos:Byte);
    var
      l:Byte;
    begin
      if v2l[0] < 255
      then
        Inc(InsPos);
      if InsPos > v2l[0]
      then
        InsPos := v2l[0];
      for l := v2l[0] downto InsPos do
        v2l[l] := v2l[l-1];
      v2l[InsPos] := Value;
    end;
var
  lp, vp : Integer;
  cDir,lDir:TDirection;
  Character:TUTF8Char;
i:Integer;
begin
  Result[0] := 0;
  lp := 1;
  vp := 1;
  lDir := drNONE;
  while lp <= Length(UTF8String) do
  begin
    Character := LCharOf(UTF8String, lp);
    cDir := DirectionOf(UTF8String, lp, pDir);
    Inc(Result[0]);
    case cDir of
      drRTL:
        begin
          lDir := drRTL;
        end;
      drLTR:
        begin
          lDir := drLTR;
          vp := Result[0];
        end;
    else
      vp := Result[0];
    end;
    Insert(lp, Result, vp);
    Inc(lp, Length(Character));
  end;
end;

function InsertChar(Src:TUTF8Char; var Dest:TUTF8String; vp:Integer; pDir:TDirection):Integer;
var
  temp:TString;
  c:TCharacter;
begin
  temp := UTF8ToUnicode(Dest);
  c := WideChar(UTF8ToUCS16(Src));
  Result := FreeBIDI.InsertChar(c, temp, vp, pDir);
  Dest := UnicodeToUTF8(temp);
end;

procedure VInsert(const Src:TUTF8String;var Dest:TUTF8String; vp:Integer; pDir:TDirection);
  function VStr(const Src:TUTF8String; pDir:TDirection):TUTF8String;
  var
    v2lSrc:TVisualToLogical;
    i:Integer;
  begin
    v2lSrc := VisualToLogical(Src,pDir);
    Result := '';
    for i := 1 to v2lSrc[0] do
      Result := Result + LCharOf(Src,v2lSrc[i]);
  end;
var
  vSrc,vDest:TUTF8String;
begin
  vSrc := VStr(Src,pDir);
  vDest := VStr(Dest,pDir);
  Insert(vSrc, vDest, vp);
  Dest := VStr(vDest, pDir);
end;

procedure VDelete(var str:TUTF8String; vp, len:Integer; pDir:TDirection);
var
  temp:TString;
begin
  temp := UTF8ToUnicode(str);
  FreeBIDI.VDelete(temp, vp, len, pDir);
  str := UnicodeToUTF8(temp);
end;

end.
