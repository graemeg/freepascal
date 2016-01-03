(*
  gba_window.pas  18/06/2006 4.40.07
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

unit gba_window;
{$i def.inc}
interface

uses 
  gba_types;

const 
  WIN_0_BG0 = (1 shl 0);
  WIN_0_BG1 = (1 shl 1);
  WIN_0_BG2 = (1 shl 2);
  WIN_0_BG3 = (1 shl 3);
  WIN_0_OBJ = (1 shl 4);
  WIN_0_SPE = (1 shl 5);
  WIN_1_BG0 = (1 shl 8);
  WIN_1_BG1 = (1 shl 9);
  WIN_1_BG2 = (1 shl 10);
  WIN_1_BG3 = (1 shl 11);
  WIN_1_OBJ = (1 shl 12);
  WIN_1_SPE = (1 shl 13);

function WinRight(x: dword): dword;
function WinLeft(x: dword): dword;
function WinDown(x: dword): dword;
function WinTop(x: dword): dword;


implementation

function WinRight(x: dword): dword;
begin
  WinRight := (x shl 0);
end;

function WinLeft(x: dword): dword;
begin
  WinLeft := (x shl 8);
end;

function WinDown(x: dword): dword;
begin
  WinDown := (x shl 0);
end;

function WinTop(x: dword): dword;
begin
  WinTop := (x shl 8);
end;

end.
