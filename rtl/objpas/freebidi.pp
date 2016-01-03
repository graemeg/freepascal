{
Author Mazen NEIFER
Licence LGPL
}
unit FreeBIDI;

{$mode objfpc}{$H+}

interface

type
  TCharacter = WideChar;
  TString = WideString;
  TDirection=(
    drNONE,
    drRTL,
    drLTR
  );
  TVisualToLogical = Array[Byte]Of Byte;
  TFontInfoPtr = Pointer;
  TCharWidthRoutine = function(Character:TCharacter;FontInfo:TFontInfoPtr):Integer;

var
  FontInfoPtr:TFontInfoPtr;
  CharWidth:TCharWidthRoutine;

{****************************Logical aspects***********************************}
{Returns the number of logical characters}
function LLength(const Src:TString):Cardinal;
{Converts visual position to logical position}
function LPos(const Src:TString; vp:Integer; pDir:TDirection):Cardinal;
{****************************Visual aspects************************************}
{Returns the number of visual characters}
function VLength(const Src:TString; pDir:TDirection):Cardinal;
{Converts a logical position to a visual position}
function VPos(const Src:TString; lp:Integer; pDir, cDir:TDirection):Cardinal;
function VPos(UTF8Char:PChar; Len:integer; BytePos:integer):Cardinal;
{Returns character at a given visual position according to paragraph direction}
function VCharOf(Src:TString; vp:Integer; dir:TDirection):TCharacter;
{Inserts a string into another paying attention of RTL/LTR direction}
procedure VInsert(const Src:TString; var Dest:TString; vp:Integer; pDir:TDirection);
{Deletes a string into another paying attention of RTL/LTR direction}
procedure VDelete(var str:TString; vp, len:Integer; pDir:TDirection);
{Resturns a sub string of source string}
//function VCopy(const Src:TString; vStart, vWidth:Integer):TString;
{Resturns the visual image of current string}
function VStr(const Src:TString; pDir:TDirection):TString;
{****************************Helper routines***********************************}
{Returns direction of a character}
function DirectionOf(Character:TCharacter):TDirection;
{Returns contextual direction of caracter in a string}
function DirectionOf(Src:TString; lp:Integer; pDir:TDirection):TDirection;
{Inserts a char as if it was typed using keyboard in the most user friendly way.
Returns the new cursor position after insersion depending on the new visual text}
function InsertChar(Src:TCharacter; var Dest:TString; vp:Integer; pDir:TDirection):Integer;
{Returns a table mapping each visual position to its logical position in an UTF8*
string}
function VisualToLogical(const Src:TString; pDir:TDirection):TVisualToLogical;

implementation

function DefaultCharWidth(Character:TCharacter; FontInfoPtr:TFontInfoPtr):Integer;
begin
  case Character of
    #9:
      Result := 8;
  else
    Result := 1;
  end;
end;
function DumpStr(const Src:TString):String;
var
  i:Integer;
begin
  Result := '';
  for i:= 1 to Length(Src) do
    case Src[i] of
      #0..#127:
         Result := Result + Src[i];
    else
      Result := Result + '$' + HexStr(Ord(Src[i]),4);
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

{****************************Logical aspects***********************************}
function LLength(const Src:TString):Cardinal;
begin
  Result := Length(Src);
end;

function LPos(const Src:TString; vp:Integer; pDir:TDirection):Cardinal;
var
  v2l:TVisualToLogical;
  i:integer;
begin
  v2l := VisualToLogical(Src, pDir);
  if vp <= v2l[0]
  then
    Result := v2l[vp]
  else
    Result := Length(Src) + 1;
end;

{****************************Visual aspects************************************}
function VLength(const Src:TString; pDir:TDirection):Cardinal;
var
  Count:Integer;
begin
  Result := 0;
  Count := Length(Src);
  while (Count > 0) do
  begin
    Result := Result + CharWidth(Src[Count], FontInfoPtr);
    Count := Count - 1;
  end;
end;

function VPos(const Src:TString; lp:Integer; pDir, cDir:TDirection):Cardinal;
var
  v2l:TVisualToLogical;
  vp:Integer;
begin
  v2l := VisualToLogical(Src, pDir);
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


function VCharOf(Src:TString; vp:Integer; dir:TDirection):TCharacter;
var
  CharLen: LongInt;
begin
  Result := Src[LPos(Src, vp, dir)];
end;

{****************************Helper routines***********************************}
function DirectionOf(Character:TCharacter):TDirection;
begin
  case Character of
    #9,#32,
    '/',
    '{','}',
    '[',']',
    '(',')':
      Result := drNONE;
    #$0590..#$05FF,      //Hebrew
    #$0600..#$06FF:      //Arabic
      Result := drRTL;
  else
    Result := drLTR;
  end;
end;

function DirectionOf(Src:TString; lp:Integer; pDir:TDirection):TDirection;
var
  c:TCharacter;
  lDir,rDir:TDirection;
  p:Integer;
begin
  if(lp <= 0)
  then
    lp := 1;
{Seek for proper character direction}
  c := Src[lp];
  lDir := DirectionOf(c);
{Seek for left character direction if it is neutral}
  p := lp;
  while(p > 1) and (lDir = drNONE)do
  begin
    c := Src[p - 1];
    lDir := DirectionOf(c);
    p := p - Length(c);
  end;
{Seek for right character direction if it is neutral}
  p := lp;
  repeat
    c := Src[p];
    rDir := DirectionOf(c);
    p := p + Length(c);
  until(p > Length(Src)) or (rDir <> drNONE);
  if(lDir = rDir)
  then
    Result := rDir
  else
    Result := pDir;
end;

function VisualToLogical(const Src:TString; pDir:TDirection):TVisualToLogical;
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
  Character:TCharacter;
i:Integer;
begin
  Result[0] := 0;
  lp := 1;
  vp := 1;
  lDir := drNONE;
  while lp <= Length(Src) do
  begin
    Character := Src[lp];
    cDir := DirectionOf(Src, lp, pDir);
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
    lp := lp + 1;
  end;
end;

function InsertChar(Src:TCharacter; var Dest:TString; vp:Integer; pDir:TDirection):Integer;
var
  vSrc,vDest:TString;
begin
  vSrc := VStr(Src,pDir);
  vDest := VStr(Dest,pDir);
  Insert(vSrc, vDest, vp);
  Dest := VStr(vDest, pDir);
  case DirectionOf(Src) of
  drRTL:
    Result := vp;
  drLTR:
    Result := vp + 1;
  else
    if(vp < Length(vDest)) and (DirectionOf(vDest[vp + 1]) = drRTL)
    then
      Result := vp
    else
      Result := vp + 1;
  end;
end;

procedure VInsert(const Src:TString;var Dest:TString; vp:Integer; pDir:TDirection);
var
  vSrc,vDest:TString;
begin
  vSrc := VStr(Src,pDir);
  vDest := VStr(Dest,pDir);
  Insert(vSrc, vDest, vp);
  Dest := VStr(vDest, pDir);
end;

procedure VDelete(var str:TString; vp, len:Integer; pDir:TDirection);
var
  v2l:TVisualToLogical;
  i:Integer;
begin
  v2l := VisualToLogical(str, pDir);
  for i := 1 to v2l[0] do
    if(v2l[i] >= vp) and (v2l[i] < vp + len)
    then
      Delete(str, v2l[i], 1);
end;

function VStr(const Src:TString; pDir:TDirection):TString;
var
  v2lSrc:TVisualToLogical;
  vp:Integer;
begin
  v2lSrc := VisualToLogical(Src,pDir);
  SetLength(Result, v2lSrc[0]);
  for vp := 1 to v2lSrc[0] do
    Result[vp] := Src[v2lSrc[vp]];
end;

initialization

  CharWidth := @DefaultCharWidth;

end.
