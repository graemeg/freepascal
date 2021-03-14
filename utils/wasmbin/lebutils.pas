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

unit lebutils;

{$mode objfpc}{$H+}

interface

uses
  SysUtils, Classes;

function ReadU(src: TStream): UInt64;
function ReadS(src: TStream; bits: Integer): Int64;

procedure WriteU(src: TStream; vl: UInt64; bits: integer; fixedSize: Boolean = false);
procedure WriteS(src: TStream; vl: Int64; bits: integer);
procedure WriteU64(src: TStream; vl: UInt64);
procedure WriteU32(src: TStream; vl: UInt32);
procedure WriteU16(src: TStream; vl: UInt16);
procedure WriteU8(src: TStream; vl: UInt8);

procedure WriteS64(src: TStream; vl: Int64);

implementation

function ReadU(src: TStream): UInt64;
var
  b : byte;
  sh : integer;
begin
  Result := 0;
  sh := 0;
  while true do begin
    b := src.ReadByte;
    Result := Result or ((b and $7f) shl sh);
    if (b and $80)>0 then inc(sh, 7)
    else begin
      break;
    end;
  end;
end;

function ReadS(src: TStream; bits: Integer): Int64;
var
  b  : byte;
  sh : Integer;
begin
  Result := 0;
  sh := 0;
  repeat
    b := src.ReadByte;
    Result := Result or ((b and $7F) shl sh);
    inc(sh, 7);
  until ((b and $80) = 0);

  // sign bit of byte is second high order bit (0x40)
  if (sh < bits) and ((b and $40) > 0) then
    // sign extend
    result := result or ( (not 0) shl sh);
end;

procedure WriteU(src: TStream; vl: UInt64; bits: integer; fixedSize: Boolean = false);
var
  b: byte;
begin
  if (bits < 0) then bits := sizeof(vl)*8;

  repeat
    b := (vl and $7f);
    vl := vl shr 7;

    if bits >0 then begin
      dec(bits,7);
      if bits<0 then bits := 0;
    end;

    if (vl <> 0) or (fixedSize and (bits > 0)) then
      b := b or $80;

    src.WriteByte(b);
  until ((vl=0) and not fixedSize) or (bits = 0)
end;

procedure WriteS(src: TStream; vl: Int64; bits: integer);
var
  more     : Boolean;
  b : byte;
begin
  more := true;

  if (bits < 0) then bits := sizeof(vl);

  while more do begin
    b := (vl and $7f);
    vl := SarInt64(vl, 7);

    { sign bit of byte is second high order bit (0x40) }
    if ((vl = 0) and (b and $40 = 0))
      or ((vl = -1) and (b and $40 <> 0))
    then
      more := false
    else
      b := b or $80;
    src.WriteByte(b);
  end;
end;

procedure WriteU32(src: TStream; vl: UInt32);
begin
  WriteU(src, vl, sizeof(vl)*8);
end;

procedure WriteU64(src: TStream; vl: UInt64);
begin
  WriteU(src, vl, sizeof(vl)*8);
end;

procedure WriteU16(src: TStream; vl: UInt16);
begin
  WriteU(src, vl, sizeof(vl)*8);
end;

procedure WriteU8(src: TStream; vl: UInt8);
begin
  WriteU(src, vl, sizeof(vl)*8);
end;

procedure WriteS64(src: TStream; vl: Int64);
begin
  WriteS(src, vl, sizeof(vl)*8);
end;

end.
