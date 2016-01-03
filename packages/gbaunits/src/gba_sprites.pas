(*
  gba_sprites.pas 18/06/2006 4.28.18
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

unit gba_sprites;
{$i def.inc}
interface

uses
  gba_types, gba_regs;


const
  OAM	: POBJATTR =	pointer($07000000);
  OBJ_BASE_ADR	=	pointer($06010000);
  BITMAP_OBJ_BASE_ADR	= pointer($06014000);

  OBJ_ROT_SCALE_ON  = (1 shl 8);
  OBJ_DISABLE = (1 shl 9);
  OBJ_DOUBLE  = (1 shl 9);

  OBJ_MOSAIC    = (1 shl 12);
  OBJ_256_COLOR = (1 shl 13);
  OBJ_16_COLOR  = (0 shl 13);

  OBJ_HFLIP = (1 shl 12);
  OBJ_VFLIP = (1 shl 13);



type
  SPRITE_SHAPES = (
    SQUARE,
    WIDE,
    TALL
  );
  TSpriteShapes = SPRITE_SHAPES;
  
  SPRITE_SIZECODE = (
    Sprite_8x8,
    Sprite_16x16,
    Sprite_32x32,
    Sprite_64x64,
    Sprite_16x8,
    Sprite_32x8,
    Sprite_32x16,
    Sprite_64x32,
    Sprite_8x16,
    Sprite_8x32,
    Sprite_16x32,
    Sprite_32x64
  );
  TSpriteSizeCode = SPRITE_SIZECODE;

function ObjY(m: integer): integer; inline;
function ObjMode(m: integer): integer; inline;
function ObjShape(m: integer): integer; inline;
function ObjX(m: integer): integer; inline;
function ObjRotScale(m: integer): integer; inline;
function ObjSize(m: integer): integer; inline;
function ObjChar(m: integer): integer; inline;
function ObjPriority(m: integer): integer; inline;
function ObjPalette(m: integer): integer; inline;

{$define ObjTranslucent := ObjMode(1)}
{$define ObjObjWindow := ObjMode(2)}
{$define ObjSquare := ObjShape(0)}
{$define ObjWide := ObjShape(1)}
{$define ObjTall := ObjShape(2)}


implementation

function ObjY(m: integer): integer; inline;
begin
  ObjY := ((m) and $00ff);
end;

function ObjMode(m: integer): integer; inline;
begin
  ObjMode := ((m) shl 10);
end;

function ObjShape(m: integer): integer; inline;
begin
  ObjShape := ((m) shl 14);
end;

function ObjX(m: integer): integer; inline;
begin
  ObjX := ((m) and $01ff);
end;

function ObjRotScale(m: integer): integer; inline;
begin
  ObjRotScale := ((m) shl 9);
end;

function ObjSize(m: integer): integer; inline;
begin
  ObjSize := ((m) shl 14);
end;

function ObjChar(m: integer): integer; inline;
begin
  ObjChar := ((m) and $03ff);
end;

function ObjPriority(m: integer): integer; inline;
begin
  ObjPriority := ((m) shl 10);
end;

function ObjPalette(m: integer): integer; inline;
begin
  ObjPalette := ((m) shl 12);
end;

end.

