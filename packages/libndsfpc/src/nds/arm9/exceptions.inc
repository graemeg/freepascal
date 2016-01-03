<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
const
  EXCEPTION_VECTOR : ^VoidFn = pointer($2FFFD9C);

var
  exceptionC: ^VoidFn; cvar; external;
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: exceptions.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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
const
  EXCEPTION_VECTOR : ^VoidFunctionPointer = pointer($2FFFD9C);

var
  exceptionC: ^VoidFunctionPointer; cvar; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  exceptionStack: cuint32; cvar; external;
  exceptionRegisters: pcint32; cvar; external;

procedure enterException(); cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
procedure setExceptionHandler(handler: VoidFn); cdecl; external;
=======
procedure setExceptionHandler(handler: VoidFunctionPointer); cdecl; external;
>>>>>>> graemeg/fixes_2_2
=======
procedure setExceptionHandler(handler: VoidFunctionPointer); cdecl; external;
>>>>>>> origin/fixes_2_2
procedure defaultExceptionHandler(); cdecl; external;

function getCPSR(): cuint32; cdecl; external;
{$endif NDS_INTERFACE}
