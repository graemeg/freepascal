(*
  gba_blend.pas 18/06/2006 4.17.40
  ------------------------------------------------------------------------------
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
unit gba_blend;
{$i def.inc}

interface

uses
  gba_types;

const
  BLEND_TOP_BG0 = (1 shl 0); 
  BLEND_TOP_BG1 = (1 shl 1);
  BLEND_TOP_BG2 = (1 shl 2);
  BLEND_TOP_BG3 = (1 shl 3);
  BLEND_TOP_OBJ = (1 shl 4);
  BLEND_TOP_BD  = (1 shl 5);
  
  BLEND_LOW_BG0 = (1 shl 8); 
  BLEND_LOW_BG1 = (1 shl 9);
  BLEND_LOW_BG2 = (1 shl 10);
  BLEND_LOW_BG3 = (1 shl 11);
  BLEND_LOW_OBJ = (1 shl 12);
  BLEND_LOW_BD  = (1 shl 13);

  BLEND_MODE_OFF    = (0 shl 6);
  BLEND_MODE_ALPHA  = (1 shl 6);
  BLEND_MODE_LIGHT  = (2 shl 6);
  BLEND_MODE_DARK   = (3 shl 6);

function BlendLow(n: dword): dword; inline;
function BlendHigh(n: dword): dword; inline;
function BlendLevel(n: dword): dword; inline;
function BlendBalance(n: dword): dword; inline;
function BlendDepth(n: dword): dword; inline;

implementation

function BlendLow(n: dword): dword; inline;
begin
  BlendLow := (n shl 0);
end;

function BlendHigh(n: dword): dword; inline;
begin
  BlendHigh := (n shl 8);
end;

function BlendLevel(n: dword): dword; inline;
begin
  BlendLevel := (BlendLow(n) or BlendHigh(n));
end;

function BlendBalance(n: dword): dword; inline;
begin
  BlendBalance := (BlendLow(n) or BlendHigh(16-n));
end;

function BlendDepth(n: dword): dword; inline;
begin
  BlendDepth := (n shl 0);
end;


end.
