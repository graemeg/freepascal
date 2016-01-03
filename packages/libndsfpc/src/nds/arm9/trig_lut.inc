<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
const
  DEGREES_IN_CIRCLE = (1 shl 15);

function fixedToInt(n, bits: cint): cint; inline;
function intToFixed(n, bits: cint): cint; inline;

function floatToFixed(n, bits: cint): cint; inline;
function fixedToFloat(n, bits: cint): cfloat; inline;

function floorFixed(n, bits: cint): cint; inline;

function degreesToAngle(degrees: cint): cint; inline;
function angleToDegrees(angle: cint): cint; inline;

function cosLerp(angle: cint16): cint16; cdecl; external;
function sinLerp(angle: cint16): cint16; cdecl; external;
function tanLerp(angle: cint16): cint32; cdecl; external;
function acosLerp(par: cint16): cint16; cdecl; external;
function asinLerp(par: cint16): cint16; cdecl; external;

{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}


function fixedToInt(n, bits: cint): cint; inline;
begin
  result := cint(n shr bits);
end;

function intToFixed(n, bits: cint): cint; inline;
begin
  result := cint(n shl bits);
end;

function floatToFixed(n, bits: cint): cint; inline;
begin
  result := cint(n * cfloat(1 shl bits));
end;

function fixedToFloat(n, bits: cint): cfloat; inline;
begin
  result := cfloat(n) / cfloat(1 shl bits);
end;

function floorFixed(n, bits: cint): cint; inline;
begin
  result := cint(n and (not ((1 shl bits) - 1)));
end;

function degreesToAngle(degrees: cint): cint; inline;
begin
  result := degrees * DEGREES_IN_CIRCLE div 360;
end;

function angleToDegrees(angle: cint): cint; inline;
begin
  result := angle * 360 div DEGREES_IN_CIRCLE;
end;

{$endif NDS_IMPLEMENTATION}
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: trig_lut.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

{$ifdef NDS_INTERFACE}
var
  COS_bin: array [0..0] of cshort; cvar; external;
  SIN_bin: array [0..0] of cshort; cvar; external;
  TAN_bin: array [0..0] of cshort; cvar; external;

{$define  _COS := COS_bin}
{$define  _SIN := SIN_bin}
{$define  _TAN := TAN_bin}
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
