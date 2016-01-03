(*
  gba_bg.pas  18/06/2006 4.07.57
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

unit gba_bg;
{$i def.inc}

interface

uses
  gba_types;


const 
  DISP_HBLANK_PROCESS_ON = (0 shl 5);
  DISP_HBLANK_PROCESS_OFF = (1 shl 5);

  DISP_SPRITE_TYPE_1D = (1 shl 6);
  DISP_SPRITE_TYPE_2D = (0 shl 6);

  DISP_FORCE_BLANK  = (1 shl 7);

  DISP_BG0  = (1 shl 8);
  DISP_BG1  = (1 shl 9);
  DISP_BG2  = (1 shl 10);
  DISP_BG3  = (1 shl 11);
  DISP_OBJ  = (1 shl 12);

  DISP_WND0 = (1 shl 13);
  DISP_WND1 = (1 shl 14);
  DISP_OBJWND = (1 shl 15);

  DSTAT_BIT_VBLANK  = (1);
  DSTAT_BIT_HBLANK  = (1 shl 1);
  DSTAT_BIT_VCOUNT  = (1 shl 2);

  DSTAT_USE_VBLANK  = (1 shl 3);
  DSTAT_USE_HBLANK  = (1 shl 4);
  DSTAT_USE_VCOUNT  = (1 shl 5);

  BG_SIZEA_256_256  = 0;
  BG_SIZEA_512_256  = (1 shl 14);
  BG_SIZEA_256_512  = (2 shl 14);
  BG_SIZEA_512_512  = (3 shl 14);

  BG_SIZEB_128_128  = 0;
  BG_SIZEB_256_256  = (1 shl 14);
  BG_SIZEB_512_512  = (2 shl 14);
  BG_SIZEB_1024_1024= (2 shl 14);

  BG_OVERLAP  = (1 shl 13);

  BG_COLOR_16   = 0;
  BG_COLOR_256  = (1 shl 7);

  BG_MOZAIC_ON  = (1 shl 6);

  BG_MAP_YFLIP  = (1 shl 11);
  BG_MAP_XFLIP  = (1 shl 10);

  MEM_BG_PAL  : ^word = pointer($5000000);

function DispBgMode(x: dword): dword;
function DispSelectBuffer(x: dword): dword;
function BgMapTile(x: word): word;
function MemBgChar(x: word): pointer;
function MemBgMap(x: word): pointer;
function BgCharBase(x: dword): dword;
function BgPriority(x: dword): dword;
function BgMapPal(x: dword): dword;
function BgMapBase(x: dword): dword;
function DStatVCountLine(x: dword): dword;


implementation

function DispBgMode(x: dword): dword;
begin
  DispBgMode := x;
end;

function DispSelectBuffer(x: dword): dword;
begin
  DispSelectBuffer := (x shl 4);
end;

function BgMapTile(x: word): word;
begin
  BgMapTile := (x);
end;

function MemBgChar(x: word): pointer;
begin
  MemBgChar := pointer($6000000 + (x)*$4000);
end;

function MemBgMap(x: word): pointer;
begin
  MemBgMap :=  pointer($6000000 + (x)* $800);
end;

function BgCharBase(x: dword): dword;
begin
  BgCharBase := ((x) shl 2);
end;

function BgPriority(x: dword): dword;
begin
  BgPriority := x;
end;


function BgMapPal(x: dword): dword;
begin
  BgMapPal := ((x) shl 12);
end;

function BgMapBase(x: dword): dword;
begin
  BgMapBase := ((x) shl 8);
end;

function DStatVCountLine(x: dword): dword;
begin
  DStatVCountLine := ((x) shl 8);
end;


end.

