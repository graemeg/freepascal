(*
  gba_mosaic.pas  18/06/2006 4.22.03
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

unit gba_mosaic;
{$i def.inc}
interface

uses
  gba_types;

function MosaicBgH(x: dword): dword;
function MosaicBgV(x: dword): dword;
function MosaicObjH(x: dword): dword;
function MosaicObjV(x: dword): dword;

implementation 

function MosaicBgH(x: dword): dword;
begin
  MosaicBgH := (x shl 0);
end;

function MosaicBgV(x: dword): dword;
begin
  MosaicBgV := (x shl 4);
end;
  
function MosaicObjH(x: dword): dword;
begin
  MosaicObjH := (x shl 8);
end;

function MosaicObjV(x: dword): dword;
begin
  MosaicObjV := (x shl 12);
end;

end.
