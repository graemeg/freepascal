{ Utility routines for HTTP server component

<<<<<<< HEAD
<<<<<<< HEAD
  Copyright (C) 2006-2008 by Micha Nelissen
=======
  Copyright (C) 2006-2007 Micha Nelissen
>>>>>>> graemeg/fixes_2_2
=======
  Copyright (C) 2006-2007 Micha Nelissen
>>>>>>> origin/fixes_2_2

  This library is Free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is diStributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; withOut even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a Copy of the GNU Library General Public License
  along with This library; if not, Write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
  
  This license has been modified. See file LICENSE.ADDON for more information.
  Should you find these sources without a LICENSE File, please contact
  me at ales@chello.sk
}

unit lHTTPUtil;

{$mode objfpc}{$h+}
{$inline on}

interface

uses
  sysutils, 
  strutils;

const
  HTTPDateFormat: string = 'ddd, dd mmm yyyy hh:nn:ss';
  HTTPAllowedChars = ['A'..'Z','a'..'z', '*','@','.','_','-', 
      '0'..'9', '$','!','''','(',')'];

type
  PSearchRec = ^TSearchRec;

  function GMTToLocalTime(ADateTime: TDateTime): TDateTime;
  function LocalTimeToGMT(ADateTime: TDateTime): TDateTime;
  function TryHTTPDateStrToDateTime(ADateStr: pchar; var ADest: TDateTime): boolean;

  function SeparatePath(var InPath: string; out ExtraPath: string; const Mode:Longint;
    ASearchRec: PSearchRec = nil): boolean;
  function CheckPermission(const ADocument: pchar): boolean;
  function HTTPDecode(AStr: pchar): pchar;
  function HTTPEncode(const AStr: string): string;
  function HexToNum(AChar: char): byte;
  
<<<<<<< HEAD
<<<<<<< HEAD
  function DecomposeURL(const URL: string; out Host, URI: string; out Port: Word): Boolean;
=======
  procedure DecomposeURL(const URL: string; out Host, URI: string; out Port: Word);
>>>>>>> graemeg/fixes_2_2
=======
  procedure DecomposeURL(const URL: string; out Host, URI: string; out Port: Word);
>>>>>>> origin/fixes_2_2
  function ComposeURL(Host, URI: string; const Port: Word): string;

implementation

uses
  lCommon;

function GMTToLocalTime(ADateTime: TDateTime): TDateTime;
begin
  Result := ADateTime + (TZSeconds*1000/MSecsPerDay);
end;

function LocalTimeToGMT(ADateTime: TDateTime): TDateTime;
begin
  Result := ADateTime - (TZSeconds*1000/MSecsPerDay);
end;

function TryHTTPDateStrToDateTime(ADateStr: pchar; var ADest: TDateTime): boolean;
var
  lYear, lMonth, lDay: word;
  lTime: array[0..2] of word;
  I, lCode: integer;
begin
  if StrLen(ADateStr) < Length(HTTPDateFormat)+4 then exit(false);
  { skip redundant short day string }
  Inc(ADateStr, 5);
  { day }
  if ADateStr[2] = ' ' then
    ADateStr[2] := #0
  else 
    exit(false);
  Val(ADateStr, lDay, lCode);
  if lCode <> 0 then exit(false);
  Inc(ADateStr, 3);
  { month }
  lMonth := 1;
  repeat
    if CompareMem(ADateStr, @ShortMonthNames[lMonth][1], 3) then break;
    inc(lMonth);
    if lMonth = 13 then exit(false);
  until false;
  Inc(ADateStr, 4);
  { year }
  if ADateStr[4] = ' ' then
    ADateStr[4] := #0
  else
    exit(false);
  Val(ADateStr, lYear, lCode);
  if lCode <> 0 then exit(false);
  Inc(ADateStr, 5);
  { hour, minute, second }
  for I := 0 to 2 do
  begin
    ADateStr[2] := #0;
    Val(ADateStr, lTime[I], lCode);
    Inc(ADateStr, 3);
    if lCode <> 0 then exit(false);
  end;
  ADest := EncodeDate(lYear, lMonth, lDay) + EncodeTime(lTime[0], lTime[1], lTime[2], 0);
  Result := true;
end;

function SeparatePath(var InPath: string; out ExtraPath: string; const Mode:Longint; 
  ASearchRec: PSearchRec = nil): boolean;
var
  lFullPath: string;
  lPos: integer;
  lSearchRec: TSearchRec;
begin
  if ASearchRec = nil then
    ASearchRec := @lSearchRec;
  ExtraPath := '';
  if Length(InPath) <= 2 then exit(false);
  lFullPath := InPath;
  if InPath[Length(InPath)] = PathDelim then
    SetLength(InPath, Length(InPath)-1);
  repeat
    Result := SysUtils.FindFirst(InPath, Mode, ASearchRec^) = 0;
    SysUtils.FindClose(ASearchRec^);
    if Result then
    begin
      ExtraPath := Copy(lFullPath, Length(InPath)+1, Length(lFullPath)-Length(InPath));
      break;
    end;
    lPos := RPos(PathDelim, InPath);
    if lPos > 0 then
      SetLength(InPath, lPos-1)
    else
      break;
  until false;
end;

function HexToNum(AChar: char): byte;
begin
  if ('0' <= AChar) and (AChar <= '9') then
    Result := ord(AChar) - ord('0')
  else if ('A' <= AChar) and (AChar <= 'F') then
    Result := ord(AChar) - (ord('A') - 10)
  else if ('a' <= AChar) and (AChar <= 'f') then
    Result := ord(AChar) - (ord('a') - 10)
  else
    Result := 0;
end;

function HTTPDecode(AStr: pchar): pchar;
var
  lPos, lNext, lDest: pchar;
begin
  lDest := AStr;
  repeat
    lPos := AStr;
    while not (lPos^ in ['%', '+', #0]) do
      Inc(lPos);
    if (lPos[0]='%') and (lPos[1] <> #0) and (lPos[2] <> #0) then
    begin
      lPos^ := char((HexToNum(lPos[1]) shl 4) + HexToNum(lPos[2]));
      lNext := lPos+2;
    end else if lPos[0] = '+' then
    begin
      lPos^ := ' ';
      lNext := lPos+1;
    end else
      lNext := nil;
    Inc(lPos);
    if lDest <> AStr then
      Move(AStr^, lDest^, lPos-AStr);
    Inc(lDest, lPos-AStr);
    AStr := lNext;
  until lNext = nil;
  Result := lDest;
end;

function HTTPEncode(const AStr: string): string;
  { code from MvC's web }
var
  src, srcend, dest: pchar;
  hex: string[2];
  len: integer;
begin
  len := Length(AStr);
  SetLength(Result, len*3); // Worst case scenario
  if len = 0 then
    exit;
  dest := pchar(Result);
  src := pchar(AStr);
  srcend := src + len; 
  while src < srcend do
  begin 
    if src^ in HTTPAllowedChars then
      dest^ := src^
    else if src^ = ' ' then
      dest^ := '+'
    else begin
      dest^ := '%';
      inc(dest);
      hex := HexStr(Ord(src^),2);
      dest^ := hex[1];
      inc(dest);
      dest^ := hex[2];
    end;
    inc(dest);
    inc(src);
  end;
  SetLength(Result, dest - pchar(Result));
end;

function CheckPermission(const ADocument: pchar): boolean;
var
  lPos: pchar;
begin
  lPos := ADocument;
  repeat
    lPos := StrScan(lPos, '/');
    if lPos = nil then exit(true);
    if (lPos[1] = '.') and (lPos[2] = '.') and ((lPos[3] = '/') or (lPos[3] = #0)) then
      exit(false);
    inc(lPos);
  until false;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function DecomposeURL(const URL: string; out Host, URI: string; out Port: Word): Boolean;
var
  n: Integer;
  tmp: string;
begin
  Result := False;

  try
    tmp := Trim(URL);
    if Length(tmp) < 1 then // don't do empty
      Exit;

    Port := 80;
    if tmp[Length(tmp)] = '/' then // remove trailing /
      Delete(tmp, Length(tmp), 1);

    if Pos('https://', tmp) = 1 then begin // check for HTTPS
      Result := True;
      Port := 443;
      Delete(tmp, 1, 8); // delete the https part for parsing reasons
    end else if Pos('http://', tmp) = 1 then begin
      Delete(tmp, 1, 7); // delete the http part for parsing reasons
    end;

    n := Pos(':', tmp); // find if we have a port at the end
    if n > 0 then begin
      Port := StrToInt(Copy(tmp, n + 1, Length(tmp)));
      Delete(tmp, n, Length(tmp));
    end;

    n := Pos('/', tmp); // find if we have a uri section
    if n > 0 then begin
      URI := Copy(tmp, n, Length(tmp));
      Delete(tmp, n, Length(tmp));
    end;
    Host := tmp;
  except
    Host := 'error';
    URI := '';
    Port := 0;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
procedure DecomposeURL(const URL: string; out Host, URI: string; out Port: Word);
var
  index: Integer;
begin
  index := PosEx('/', URL, 8);
  Host := Copy(URL, 8, index-8);
  URI := Copy(URL, index, Length(URL)+1-index);

  index := Pos(':', Host);
  if index > 0 then begin
    Port := StrToIntDef(Copy(Host, index+1, Length(Host)-index), -1);

    SetLength(Host, index-1);
  end else
    Port := 80;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function ComposeURL(Host, URI: string; const Port: Word): string;
begin
  Host := Trim(Host);
<<<<<<< HEAD
<<<<<<< HEAD
  URI := StringReplace(Trim(URI), '%20', ' ', [rfReplaceAll]);
=======
  URI := Trim(URI);
>>>>>>> graemeg/fixes_2_2
=======
  URI := Trim(URI);
>>>>>>> origin/fixes_2_2

  if (Pos('http://', Host) <> 1)
  and (Pos('https://', Host) <> 1) then
    Host := 'http://' + Host;

  if URI[Length(URI)] = '/' then
    Delete(URI, Length(URI), 1);

  if  (Host[Length(Host)] = '/')
  and (URI[1] = '/') then
    Delete(Host, Length(Host), 1)
  else if (URI[1] <> '/')
  and     (Host[Length(Host)] <> '/') then
    Host := Host + '/';

  Result := Host + URI + ':' + IntToStr(Port);
end;


end.
