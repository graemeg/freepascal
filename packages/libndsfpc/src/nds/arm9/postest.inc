<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: postest.inc 25 2007-12-10 21:06:46Z p4p3r0 $ 
  ------------------------------------------------------------------------------
  	Copyright (C) 2007
  		Michael Noland (joat)
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
   
  Copyright (C) 2007  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef NDS_IMPLEMENTATION}
function PosTestBusy(): cbool; inline;
begin
  PosTestBusy := (GFX_STATUS^ and (1 shl 0)) <> 0;
end;

procedure PosTest_Asynch(x, y, z: v16); inline;
begin
	GFX_POS_TEST^ := VERTEX_PACK(x, y);
	GFX_POS_TEST^ := z;
end;

procedure PosTest(x, y, z: v16); inline;
begin 
	PosTest_Asynch(x, y, z);
	while PosTestBusy() do;
end;

function PosTestWresult(): cint32; inline;
begin
  PosTestWresult := GFX_POS_RESULT[3];
end;

function PosTestXresult(): cint32; inline;
begin
  PosTestXresult := GFX_POS_RESULT[0];
end;

function PosTestYresult(): cint32; inline;
begin
	PosTestYresult := GFX_POS_RESULT[1];
end;

function PosTestZresult(): cint32; inline;
begin
  PosTestZresult := GFX_POS_RESULT[2];
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
function PosTestBusy(): cbool; inline;
procedure PosTest_Asynch(x, y, z: v16); inline;
procedure PosTest(x, y, z: v16); inline;
function PosTestWresult(): cint32; inline;
function PosTestXresult(): cint32; inline;
function PosTestYresult(): cint32; inline;
function PosTestZresult(): cint32; inline;
{$endif NDS_INTERFACE}
