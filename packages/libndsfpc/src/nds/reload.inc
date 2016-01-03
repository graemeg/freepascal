(*
  $Id: reload.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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
type
  COPYFUNC = function (const value: pcchar): pointer;
  QUERYFUNC = function(value: cuint32): pcuint32;
 
  LOADER_DATA = record
    VERSION: cuint32;   // contains version information about loader
    QUERY: QUERYFUNC;     // used to retreive extended information from loader
    ARM7FUNC: COPYFUNC;  // a pointer to the ARM7 load function
    ARM9FUNC: COPYFUNC;  // a pointer to the ARM9 load function
    PATH: pcchar;      // THIS VALUE IS SET FOR YOU, DONT TOUCH IT
    RESERVED: cuint32;  // reserved for future expansion
  end;
  TLOADER_DATA = LOADER_DATA;
  PLOADER_DATA = ^LOADER_DATA;
 
const
  LOADNDS : PLOADER_DATA = pointer($02800000-48);
  BOOT_NDS = pcchar(-1);
{$endif NDS_INTERFACE}

 
 
(*
dump this near the begining of your ARM7's main function
 
LOADNDS->PATH = 0;
 
 
inside of your ARM7's main loop or VBlank IRQ or what-ever, add this code:
 
if (LOADNDS->PATH != 0) {
  LOADNDS->ARM7FUNC(LOADNDS->PATH);
}
 
 
//////////////////////////////////////
 
in your ARM9 code, to return to the menu, call this function:
 
WAIT_CR &= ~0x8080;
LOADNDS->ARM9FUNC(BOOT_NDS);
 
*)
