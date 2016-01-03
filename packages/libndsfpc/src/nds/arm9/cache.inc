<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
procedure IC_InvalidateAll(); cdecl; external;
procedure IC_InvalidateRange(const base: pointer; size: cuint32); cdecl; external;
procedure DC_FlushAll(); cdecl; external;
procedure DC_FlushRange(const base: pointer; size: cuint32); cdecl; external;
procedure DC_InvalidateAll(); cdecl; external;
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: cache.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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
procedure IC_InvalidateAll(); cdecl; external;

procedure IC_InvalidateRange(const base: pointer; size: cuint32); cdecl; external;

procedure DC_FlushAll(); cdecl; external;

procedure DC_FlushRange(const base: pointer; size: cuint32); cdecl; external;

procedure DC_InvalidateAll(); cdecl; external;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure DC_InvalidateRange(const base: pointer; size: cuint32); cdecl; external;
{$endif NDS_INTERFACE}
