<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef ARM7}
  {$error Touch screen is only available on the ARM7}
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: touch.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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

{$ifndef ARM7}
{$error Touch screen is only available on the ARM7}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif ARM7}

{$ifdef NDS_INTERFACE}
const
  SCREEN_WIDTH	= 256;
  SCREEN_HEIGHT	= 192;


  TSC_MEASURE_TEMP1    = $84;
  TSC_MEASURE_Y        = $90;
  TSC_MEASURE_BATTERY  = $A4;
  TSC_MEASURE_Z1       = $B4;
  TSC_MEASURE_Z2       = $C4;
  TSC_MEASURE_X        = $D0;
  TSC_MEASURE_AUX      = $E4;
  TSC_MEASURE_TEMP2    = $F4;


<<<<<<< HEAD
<<<<<<< HEAD
procedure touchReadXY(var touchPos: touchPosition); cdecl; external;

function touchRead(command: cuint32): cuint16; cdecl; external;
function touchReadTemperature(t1: pcint; t2: pcint): cuint32;  cdecl; external;

function touchPenDown(): cbool; cdecl; external;
=======
=======
>>>>>>> origin/fixes_2_2

function touchReadXY(): touchPosition; cdecl; external;

function touchRead(command: cuint32): cuint16; cdecl; external;
function touchReadTemperature(t1: pcint; t2: pcint): cuint32;  cdecl; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}
