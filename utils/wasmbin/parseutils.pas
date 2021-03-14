{ This file is part of wasmbin - a collection of WebAssembly binary utils.

  Copyright (C) 2019, 2020 Dmitry Boyarintsev <skalogryz.lists@gmail.com>
  Copyright (C) 2020 by the Free Pascal development team

  This source is free software; you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  A copy of the GNU General Public License is available on the World Wide Web
  at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
  to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
  Boston, MA 02110-1335, USA.
}

unit parseutils;

{$ifdef fpc}{$mode delphi}{$h+}{$endif}

interface

uses
  Classes, SysUtils;

type
  TCharSet = set of Char;

const
  EoLnChars      = [#10,#13];
  SpaceChars     = [#32,#9];
  InvsChars      = [#0..#32];
  WhiteSpaceChars = SpaceChars;
  SpaceEolnChars = EoLnChars+SpaceChars;
  NumericChars   = ['0'..'9'];
  HexChars       = ['0'..'9','a'..'f','A'..'F'];
  SignChars      = ['+','-'];
  SignNumericChars = NumericChars + SignChars;
  AlphabetChars  = ['a'..'z','A'..'Z'];
  AlphaNumChars  = AlphabetChars+NumericChars;

function ScanWhileWithFirst(const s: AnsiString; var index: Integer; const first, body: TCharSet): AnsiString;
function ScanWhile(const s: AnsiString; var index: Integer; const ch: TCharSet): AnsiString;
function ScanTo(const s: AnsiString; var index: Integer; const ch: TCharSet): AnsiString;
function SkipToEoln(const s: AnsiString; var index: Integer): AnsiString;
function ScanToSubstr(const s: AnsiString; var index: Integer; const substr: string): AnsiString;

// returns #10, #13, #10#13 or #13#10, if s[index] is end-of-line sequence
// otherwise returns empty string
function EolnStr(const s: AnsiString; index: Integer): String;

function IsSubStr(const sbs, s: AnsiString; index: Integer): Boolean;

// todo: not used?
function SkipCommentBlock(const s: AnsiString; var index: Integer; const closecmt: AnsiString): AnsiString;

function SkipLine(const s: AnsiString; var index: Integer): AnsiString;

procedure OffsetToLinePos(const t: AnsiString; Offset: Integer; var P: TPoint);

procedure ParseCSSValues(const s: String; css: TStrings);
procedure GetCssAbsBoundsRect(Css: TStrings; var r: TRect);
function CssValInt(const s: String; Def: integer): Integer;

type
  TCNumberFormat = (nfError, nfInteger, nfHex, nfFloat, nfFloatHex);

// if buf contains "nan" or "inf" it's also recognized as float numbers
function ScanNumberC(const buf: string; var idx: Integer;
  var numberText: string): TCNumberFormat;

implementation

function CssValInt(const s: String; Def: integer): Integer;
var
  i : integer;
  n : String;
  err : Integer;
begin
  i:=1;
  n:=ScanWhile(s, i, ['+','-']+NumericChars);
  Val(n, Result, err);
  if err<>0 then Result:=Def;
end;

procedure GetCssAbsBoundsRect(Css: TStrings; var r: TRect);
begin
  r.Left:=CssValInt(Css.Values['LEFT'], 0);
  r.Top:=CssValInt(Css.Values['top'], 0);
  r.Right:=r.Left+CssValInt(Css.Values['width'], 0);
  r.Bottom:=r.Top+CssValInt(Css.Values['height'], 0);
end;

procedure ParseCSSValues(const s: String; css: TStrings);
var
  i : integer;
  n : String;
  v : String;
begin
  i:=1;
  if (s='') or not Assigned(css) then Exit;
  while (i<=length(s))  do begin
    ScanTo(s, i, AlphaNumChars);
    n:=ScanWhile(s, i, AlphaNumChars+['_']);
    ScanTo(s, i, [':']);
    inc(i);
    ScanWhile(s, i, SpaceEolnChars);
    v:=ScanTo(s, i, [';']);
    css.Values[n]:=v;
  end;
end;

function ScanWhile(const s: AnsiString; var index: Integer; const ch: TCharSet): AnsiString;
var
  i : Integer;
begin
  Result := '';
  if (index <= 0) or (index > length(s)) then Exit;
  for i := index to length(s) do
    if not (s[i] in ch) then begin
      if i = index then Result := ''
      else Result := Copy(s, index, i - index);
      index := i;
      Exit;
    end;
  Result := Copy(s, index, length(s) - index + 1);
  index := length(s) + 1;
end;

function ScanWhileWithFirst(const s: AnsiString; var index: Integer; const first, body: TCharSet): AnsiString;
var
  i : Integer;
begin
  Result := '';
  if (index <= 0) or (index > length(s)) then Exit;

  i:=index;
  if not (s[i] in first) then Exit;

  inc(i);
  while (i<=length(s)) and (s[i] in body) do inc(i);
  Result := Copy(s, index, i-index);
  index:=i;
end;

function ScanTo(const s: AnsiString; var index: Integer; const ch: TCharSet): AnsiString;
var
  i : Integer;
begin
  Result := '';
  if (index <= 0) or (index > length(s)) then Exit;
  for i := index to length(s) do
    if (s[i] in ch) then begin
      if i = index then Result := ''
      else Result := Copy(s, index, i - index);
      index := i;
      Exit;
    end;
  Result := Copy(s, index, length(s) - index + 1);
  index := length(s) + 1;
end;

function EolnStr(const s: AnsiString; index: Integer): String;
begin
  if (index<=0) or (index>length(s)) or (not (s[index] in EoLnChars)) then
    Result:=''
  else begin
    if (index<length(s)) and (s[index+1] in EolnChars) and (s[index]<>s[index+1]) then
      Result:=Copy(s, index, 2)
    else
      Result:=s[index];
  end;
end;

function SkipToEoln(const s: AnsiString; var index: Integer): AnsiString;
begin
  Result := ScanTo(s, index, EoLnChars);
end;

function IsSubStr(const sbs, s: AnsiString; index: Integer): Boolean;
var
  i : Integer;
  j : Integer;
begin
  Result := false;
  if (sbs = '') or (length(sbs) > length(s) - index) then Exit;
  j := index;
  for i := 1 to length(sbs) do begin
    if sbs[i] <> s[j] then Exit;
    inc(j);
  end;
  Result := true;
end;

function SkipCommentBlock(const s: AnsiString; var index: Integer; const closecmt: AnsiString): AnsiString;
begin
  Result := '';
  if closecmt = '' then begin
    index := length(s) + 1;
    Exit;
  end;
  while index <= length(s) do begin
    Result := Result + ScanTo(s, index, [closecmt[1]]+EoLnChars);
    //if (index<=length(s)) and (s in EoLnChars(

    if IsSubStr(closecmt, s, index) then begin
      inc(index, length(closecmt));
      Exit;
    end else begin
      Result := Result + s[index];
      inc(index);
    end;
  end;
end;

function SkipLine(const s: AnsiString; var index: Integer): AnsiString;
begin
  Result:=ScanTo(s, index, EoLnChars);
  if (index<length(s)) and (s[index+1] in EoLnChars) and (s[index]<>s[index+1]) then
    inc(index);
  inc(index);
end;

procedure OffsetToLinePos(const t: AnsiString; Offset: Integer; var P: TPoint);
var
  i,  le  : Integer;
begin
  i := 1;
  le := 0;
  P.X := 0;
  P.Y := 0;
  while i < Offset do begin
    Inc(P.Y);
    le := i;
    SkipLine(t, i);
  end;
  P.X := Offset - le + 1;
end;

function isSubStrMatch(const s: AnsiString; index: integer; const substr: string): Boolean;
var
  i : integer;
  j : integer;
begin
  j:=index;
  Result:=false;
  for i:=1 to length(substr) do begin
    if s[j]<>substr[i] then Exit;
    inc(j);
  end;
  Result:=true;
end;

function ScanToSubstr(const s: AnsiString; var index: Integer; const substr: string): AnsiString;
var
  i: integer;
begin
  if substr='' then begin
    Result:='';
    Exit;
  end;
  i:=index;
  while (index<=length(s)) do begin
    ScanTo(s, index, [substr[1]]);
    if isSubStrMatch(s, index, substr) then begin
      inc(index, length(substr));
      Break;
    end else
      inc(index);
  end;
  Result:=Copy(s, i, index-i);
end;

function ScanHexNumber(const buf: string; var idx: Integer; var numberText: string): TCNumberFormat;
var
  xp : char;
  s : string;
begin
  Result := nfError;
  if (idx=length(buf)) or (buf[idx]<>'0') or (buf[idx+1]<>'x') then Exit;
  inc(idx, 2);
  numberText := ScanWhile(buf, idx, HexChars);
  if numberText = '' then Exit;

  numberText := '0x'+numberText;
  if ((idx<=length(buf)) and (buf[idx] in ['.','p','P'])) then begin
    if buf[idx]='.' then begin
      s := ScanWhileWithFirst(buf, idx, ['.']+HexChars, HexChars);
      if s = '' then Exit; // should not be empty
      numberText := numberText + s;
    end;

    if buf[idx] in ['p','P'] then begin
      // hexal exponenta is numeric, not hexidemical
      xp := buf[idx];
      inc(idx);
      s := ScanWhileWithFirst(buf, idx, SignNumericChars, NumericChars);
      if s = '' then Exit;
      numberText := numberText + xp+s;
    end;
    Result := nfFloathex
  end else
    Result := nfHex;
end;

function ScanNumeric(const buf: string; var idx: integer; var numberText: string): TCNumberFormat;
var
  mnt : string;
  exp : string;
  xp  : char;
begin
  Result := nfError;
  numberText:=ScanWhile(buf, idx, NumericChars);

  if ((idx<=length(buf)) and (buf[idx] in ['.','e','E'])) then begin

    //  mantissa (or fractional part) can be empty
    mnt := ScanWhileWithFirst(buf, idx, ['.']+NumericChars, NumericChars);

    if (buf[idx] in ['e','E']) then begin
      xp:=buf[idx];
      inc(idx);
      exp := ScanWhileWithFirst(buf, idx, SignNumericChars, NumericChars);
      // exponent cannot be empty, if "e" is present
      if exp='' then Exit;
      exp := xp+exp;
    end else
      exp := '';

    numberText:=NumberText+mnt+exp;
    Result := nfFloat;
  end else if numberText<>'' then
    Result := nfInteger;
end;


function ScanNumberC(const buf: string; var idx: Integer; var numberText: string): TCNumberFormat;
var
  ch  : char;
  sub : string;
begin
  Result := nfError;

  if buf[idx] in SignChars then begin
    ch:=buf[idx];
    inc(idx);
  end else
    ch := #0;

  if (idx+2<=length(buf)) and ((buf[idx]='i') and (buf[idx+1]='n') and (buf[idx+2]='f')) then begin
    numberText:='inf';
    inc(idx, 3);
    Result := nfFloat;
  end else if (idx+2<=length(buf)) and ((buf[idx]='n') and (buf[idx+1]='a') and (buf[idx+2]='n')) then begin
    numberText:='nan';
    inc(idx, 3);
    if (idx < length(buf)) and (buf[idx]=':') then begin
      inc(idx);
      sub := '';
      if (ScanNumberC(buf, idx, sub) in [nfHex, nfInteger]) then
        numberText:=numberText+':'+sub
      else
        Exit; // error
    end;
    Result := nfFloat;
  end else if (idx<length(buf)) and (buf[idx]='0') and (buf[idx+1]='x') then begin
    Result := ScanHexNumber(buf, idx, numberText)
  end else
    Result := ScanNumeric(buf, idx, numberText);

  if Result = nfError then Exit;

  if (ch<>#0) then begin
    if (numberText = '') then Exit;
    numberText:=ch+numberText;
  end;
end;

end.

