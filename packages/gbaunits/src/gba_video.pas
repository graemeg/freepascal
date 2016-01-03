(*
  gba_video.pas 18/06/2006 4.38.37
  ------------------------------------------------------------------------------
  This lib is a raw porting of libgba library for gba (you can find it at
  http://www.devkitpro.org).

  As this is a direct port from c, I'm pretty sure that something could not work
  as you expect. I am even more sure that this code could be written better, so
  if you think that I have made some mistakes or you have some better
  implemented functions, let me know [francky74 (at) gmail (dot) com]
  Enjoy!

  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)

  Copyright (C) 2006  Francesco Lombardi

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
  ------------------------------------------------------------------------------
*)
unit gba_video;
{$i def.inc}

interface

uses
  gba_types, gba_regs;

type
  LCDC_IRQ = (
    LCDC_VBL_FLAG   = (1 shl 0),
    LCDC_HBL_FLAG   = (1 shl 1),
    LCDC_VCNT_FLAG  = (1 shl 2),
    LCDC_VBL        = (1 shl 3),
    LCDC_HBL        = (1 shl 4),
    LCDC_VCNT       = (1 shl 5)
  );
  TLCDCIRQ = LCDC_IRQ;

  BG_CTRL_BITS = (
    BG_MOSAIC     = BIT6,
    BG_16_COLOR		=	(0 shl 7),
    BG_256_COLOR	=	BIT7,
    BG_WRAP			=	BIT13,
    BG_SIZE_0		=	(0 shl 14),
    BG_SIZE_1		=	(1 shl 14),
    BG_SIZE_2		=	(2 shl 14),
    BG_SIZE_3		=	(3 shl 14)
  );
  TBgCTRLBits = BG_CTRL_BITS;

  NAMETABLE  = array [0..31, 0..31] of word;
  TNameTable = NAMETABLE;
  PNameTable = ^TNameTable;

  MODE3_LINE = array [0..239] of word;
  TMODE3Line = MODE3_LINE;

  MODE5_LINE = array [0..159] of word;
  TMODE5Line = MODE5_LINE;


const
  MODE_0	=	0;
  MODE_1	=	1;
  MODE_2	=	2;
  MODE_3	=	3;
  MODE_4	=	4;
  MODE_5	=	5;

  BACKBUFFER	=	BIT4;
  OBJ_1D_MAP	=	BIT6;
  LCDC_OFF	=	BIT7;
  BG0_ON		=	BIT8;
  BG1_ON		=	BIT9;
  BG2_ON		=	BIT10;
  BG3_ON		=	BIT11;
  OBJ_ON		=	BIT12;
  WIN0_ON		=	BIT13;
  WIN1_ON		=	BIT14;
  OBJ_WIN_ON	=	BIT15;

  BG0_ENABLE		=	BG0_ON;
  BG1_ENABLE		=	BG1_ON;
  BG2_ENABLE		=	BG2_ON;
  BG3_ENABLE		=	BG3_ON;
  OBJ_ENABLE		=	OBJ_ON;
  WIN0_ENABLE		=	WIN0_ON;
  WIN1_ENABLE		=	WIN1_ON;
  OBJ_WIN_ENABLE	=	BG0_ON;

  BG_ALL_ON		=	BG0_ON or BG1_ON or BG2_ON or BG3_ON;
  BG_ALL_ENABLE	=	BG0_ON or BG1_ON or BG2_ON or BG3_ON;


const
  MAP : PNameTable = pointer($06000000);
  BG_WID_32 = BG_SIZE_0;
  BG_WID_64 = BG_SIZE_1;
  BG_HT_32  = BG_SIZE_0;
  BG_HT_64  = BG_SIZE_2;
//---------------------------------------------------------------------------------
// Symbolic names for the rot/scale map sizes
//---------------------------------------------------------------------------------
  ROTBG_SIZE_16  = BG_SIZE_0;
  ROTBG_SIZE_32  = BG_SIZE_1;
  ROTBG_SIZE_64  = BG_SIZE_2;
  ROTBG_SIZE_128 = BG_SIZE_3;
  MODE3_FB: ^TMODE3Line = pointer($06000000);
  MODE5_FB: ^TMODE5Line = pointer($06000000);
  MODE5_BB: ^TMODE5Line = pointer($0600A000);


function VCount(m: integer): dword; inline;
function BgSize(m: integer): integer; inline;
function CharBase(m: integer): integer; inline;
function CharBaseAdr(m: integer): integer; inline;
function MapBaseAdr(m: integer): integer; inline;
function ScreenBase(m: integer): integer; inline;
function TileBase(m: integer): integer; inline;
function TileBaseAdr(m: integer): integer; inline;
function BgPriority(m: integer): integer; inline;
function BgPalette(m: integer): integer; inline;
function PatRAM4(x, tn: integer): dword; inline;
function PatRAM8(x, tn: integer): dword; inline;
function SprVRAM(tn: integer): dword; inline;


procedure SetMode(mode: dword); inline;
procedure Wait(count: dword);
procedure WaitForVBlank(); inline;
procedure WaitForVDraw(); inline;
procedure VSync(); inline;
function Flip(): pword;

function RGB(const r, g, b: word): word;inline;
function RGB5(const r, g, b: word): word;inline;
function RGB8(const r, g, b: word): word;inline;

implementation

function VCount(m: integer): dword; inline;
begin
  VCount := m shl 8;
end;

function BgSize(m: integer): integer; inline;
begin
  BgSize := ((m shl 14));
end;

function CharBase(m: integer): integer; inline;
begin
  CharBase := ((m) shl 2);
end;

function CharBaseAdr(m: integer): integer; inline;
begin
  CharBaseAdr := ($6000000 + ((m) shl 14));
end;

function MapBaseAdr(m: integer): integer; inline;
begin
  MapBaseAdr := ($6000000 + ((m) shl 11));
end;

function ScreenBase(m: integer): integer; inline;
begin
  ScreenBase := ((m) shl 8);
end;

//alternate names for char and screen base
function TileBase(m: integer): integer; inline;
begin
  TileBase := ((m) shl 2);
end;

function TileBaseAdr(m: integer): integer; inline;
begin
  TileBaseAdr := ($6000000 + ((m) shl 14));
end;

function BgPriority(m: integer): integer; inline;
begin
  BgPriority := (m);
end;

function BgPalette(m: integer): integer; inline;
begin
  BgPalette := ((m) shl 12);
end;

(*---------------------------------------------------------------------------------
	CHAR_BASE_ADR() is the direct equivalent to old PATRAM(),
	giving the base address of a chr bank.
	These macros pinpoint the base address of a single tile.
---------------------------------------------------------------------------------*)
function PatRAM4(x, tn: integer): dword; inline;
begin
  PatRAM4 := (dword($6000000 or (((x) shl 14) + ((tn) shl 5)) ));
end;

function PatRAM8(x, tn: integer): dword; inline;
begin
  PatRAM8 := (dword($6000000 or (((x) shl 14) + ((tn) shl 6)) ));
end;

function SprVRAM(tn: integer): dword; inline;
begin
  SprVRAM := (dword($6000000 or $10000 or ((tn) shl 5)));
end;

procedure SetMode(mode: dword); inline;
begin
  REG_DISPCNT^ := (mode);
end;

procedure Wait(count: dword);
var
  vline: word;
begin
	vline := REG_VCOUNT^;
	dec(count);
	while (count > 0) do
	begin
		while(vline = REG_VCOUNT^) do;
		while(vline <> REG_VCOUNT^) do;
		dec(count);
	end;
end;

procedure WaitForVBlank(); inline;
begin
  while (REG_VCOUNT^ < 160) do
end;

procedure WaitForVDraw(); inline;
begin
 	while (REG_VCOUNT^ >= 160) do
end;

// wait till the _next_ vblank
// Note that for real work, using a VBlank interrupt is preferred
procedure VSync(); inline;
begin
	WaitForVDraw();	
  WaitForVBlank();
end;

function Flip(): pword;
begin
  VideoBuffer := pword(dword(VideoBuffer) or $0000A000);
  REG_DISPCNT^ := REG_DISPCNT^ or $0010;
  Flip := VideoBuffer;
end;

function RGB(const r, g, b: word): word;inline;
begin
  RGB := ((r) + (g shl 5) + (b shl 10));
end;

function RGB5(const r, g, b: word): word;inline;
begin
  RGB5 := ((r) or ((g) shl 5) or ((b) shl 10));
end;

function RGB8(const r, g, b: word): word;inline;
begin
  RGB8 := ( (((b) shr 3) shl 10) or (((g) shr 3) shl 5) or ((r) shr 3) );
end;

end.
