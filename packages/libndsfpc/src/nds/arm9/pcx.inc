<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: pcx.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef NDS_INTERFACE}
type
  // short int = cint16???
  PCXHeader = packed record
    manufacturer: cchar;   //should be 0
    version: cchar;        //should be 5 
    encoding: cchar;       //should be 1
    bitsPerPixel: cchar; //should be 8
    xmin,ymin: cint16;      //coordinates for top left,bottom right
    xmax,ymax: cint16;     
    hres: cint16;           //resolution
    vres: cint16;
    palette16: array [0..47] of cchar;  //16 color palette if 16 color image
    reserved: cchar;       //ignore
    colorPlanes: cchar;   //ignore
    bytesPerLine: cint16; 
    paletteYype: cint16;   //should be 2
    filler: array [0..57] of cchar;     //ignore
  end;
  TPCXHeader = PCXHeader;
  PPCXHeader = ^PCXHeader;


<<<<<<< HEAD
<<<<<<< HEAD
function loadPCX(const pcx: pcuchar; image: PsImage): cint; cdecl; external;
=======
function loadPCX(pcx: pcuchar; image: PsImage): cint; cdecl; external;
>>>>>>> graemeg/fixes_2_2
=======
function loadPCX(pcx: pcuchar; image: PsImage): cint; cdecl; external;
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}
