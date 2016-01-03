(*
  $Id$
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
  DynamicArray = record 
    data: ppointer;  // pointer to array of void pointers
    cur_size: integer;    // currently allocated size of the array 
  end;
  TDynamicArray = DynamicArray;
  PDynamicArray = ^TDynamicArray;


function DynamicArrayInit(v: PDynamicArray; initialSize: integer): pointer; inline;
procedure DynamicArrayDelete(v: PDynamicArray); inline;
function DynamicArrayGet(v: PDynamicArray; index: integer): pointer; inline;
procedure DynamicArraySet(v: PDynamicArray; index: integer; item: pointer); inline;
{$endif NDS_INTERFACE}


{$ifdef NDS_IMPLEMENTATION}
function DynamicArrayInit(v: PDynamicArray; initialSize: integer): pointer; inline;
begin
	v^.cur_size := initialSize;
//  v^.data := malloc(sizeof(pointer) * initialSize);
	GetMem(v^.data, sizeof(pointer) * initialSize);
  result := v^.data;
end;

procedure DynamicArrayDelete(v: PDynamicArray); inline;
begin
  if (v^.data) <> nil then
    free(v^.data);
end;

function DynamicArrayGet(v: PDynamicArray; index: integer): pointer; inline;
begin
	if (index >= v^.cur_size) then
	begin
		result := nil;
		exit;
	end;
	result := v^.data + index;
end;

procedure DynamicArraySet(v: PDynamicArray; index: integer; item: pointer); inline;
var
  i: integer;
begin
	if (index >= v^.cur_size) then 
	begin
		v^.data := ReAllocMem(v^.data, sizeof(pointer) * v^.cur_size * 2);
		FillChar(pointer(integer(v^.data^^) + v^.cur_size)^, sizeof(pointer) * v^.cur_size, 0);
		v^.cur_size := v^.cur_size * 2;
	end;
	{
	i := integer(v^.data);
	i := i + index;
  pointer(i) := item;
  }
  v^.data[index] := item;
end;
{$endif NDS_IMPLEMENTATION}
