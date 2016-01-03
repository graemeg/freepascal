(*
  gba_fade.pas  18/06/2006 4.19.14
  ------------------------------------------------------------------------------
  Part of this file is a raw porting of libgba library for gba (you can find it 
  at http://www.devkitpro.org).
  
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
 
unit gba_fade;
{$i def.inc}
interface

uses 
  gba_types, gba_regs, gba_video, gba_core;

var
  CurrentPalette: array [0..511] of word;
  FadeTable: array [0..(512*3*2)-1] of smallint;
  
procedure GetCurrentPalette();
procedure SetPalette(Palette: pword);
procedure DoFade(FadeCount: dword);
procedure FadeToGrayScale(FrameCount: dword);
procedure FadeToPalette(const NewPalette: pword; FrameCount: dword);
procedure FadeToGray(gray: word; FrameCount: dword);

implementation

procedure GetCurrentPalette();
var
  i: dword;
  Src: ^word;
  Dest: ^word;
begin
  Src := BG_COLORS;
  Dest := CurrentPalette;
	for i := 0 to 511 do
	  Dest[i] := dword(Src[i]);
end;


procedure SetPalette(Palette: pword);
var
  i: dword;
  Src, Dest: ^word;
begin	
	Src := Palette;
	Dest := BG_COLORS;
	for i := 0 to 511 do
		Dest[i] := dword(Src[i]);
end;

procedure DoFade(FadeCount: dword);
var
  r, g, b: word;
	i, count, color: dword;
	src: ^smallint;
  dest: ^word;
begin
  for count := 0 to FadeCount - 1 do
  begin
    src := FadeTable;
    dest := CurrentPalette;

    i := 0;
    while i < 512 do
    begin
      r := Src[(i*6)+1];
      r := r + Src[(i*6)];
      Src[(i*6)+1] := r;
      
      g := Src[(i*6)+3];
      g := g + Src[(i*6)+2];
      Src[(i*6)+3] := g;
      
      b := Src[(i*6)+5];
      b := b + Src[(i*6)+4];
      Src[(i*6)+5] := b;
      
      color := (r shr 8) or ((g shr 8) shl 5) or ((b shr 8) shl 10);
      Dest[i] := color;
      inc(i);
    end;
    WaitForVBlank();
    SetPalette(CurrentPalette);
	end;
end;

procedure FadeToGray(gray: word; FrameCount: dword);
var
	index, r, g, b, color: dword;
 	src: ^word;
	table: ^smallint; 
begin
  GetCurrentPalette();
  src := CurrentPalette;
  
  for index :=0  to 511 do 
  begin
    color := src[index];
    r := (color and $1f) shl 8;
    g := ((color shr 5) and $1f) shl 8;
    b := ((color shr 10) and $1f) shl 8;
    
    FadeTable[(index*6)] := ((gray shl 8)-r) div FrameCount;
    FadeTable[(index*6)+1] := r;
    
    FadeTable[(index*6)+2] := ((gray shl 8)-g) div FrameCount;
    FadeTable[(index*6)+3] := g;
    
    FadeTable[(index*6)+4] := ((gray shl 8)-b) div FrameCount;
    FadeTable[(index*6)+5] := b;
  end;
  DoFade( FrameCount);
end;

procedure FadeToPalette(const NewPalette: pword; FrameCount: dword);
var
  index: dword;
  color: word;
  r1, r2, g1, g2, b1, b2: smallint;
  Src: ^word;
  Dest: ^word;
  Table: ^smallint;
begin
  GetCurrentPalette();
  Src := CurrentPalette;
  Dest := NewPalette;
  Table := FadeTable;
  
  for index := 0 to 511 do
  begin
    color := Src[index];
    r1 := (color and $1f) shl 8;
    g1 := ((color shr 5) and $1f) shl 8;
    b1 := ((color shr 10) and $1f) shl 8;
    
    color := Dest[index];
    r2 := (color and $1f) shl 8;
    g2 := ((color shr 5) and $1f) shl 8;
    b2 := ((color shr 10) and $1f) shl 8;
    
    Table[(index*6)] := (r2 - r1) div FrameCount;
    Table[(index*6)+1] := r1;
    Table[(index*6)+2] := (g2 - g1) div FrameCount;
    Table[(index*6)+3] := g1;
    Table[(index*6)+4] := (b2 - b1) div FrameCount;
    Table[(index*6)+5] := b1;
  end;
  DoFade(FrameCount);
end;

procedure FadeToGrayScale(FrameCount: dword);
var
  index: dword;
  gray: word;
  r,g,b: smallint;
  Src: ^word;
  Dest: ^word;
  Table: ^smallint;
  GrayScalePalette: array [0..511] of word; 
begin
  GetCurrentPalette();
  Src := CurrentPalette;
  Table := FadeTable;

  for index := 0 to 511 do
  begin
    r := (Src[index] and $1f);
    g := ((Src[index] shr 5) and $1f);
    b := ((Src[index] shr 10) and $1f);
    Gray := (r shr 2) + (r shr 4) + (g shr 1) + (g shr 4) + (b shr 3); 
    GrayScalePalette[index] := RGB(gray, gray, gray);
  end;
  FadeToPalette(GrayScalePalette, FrameCount);
end;

end.
