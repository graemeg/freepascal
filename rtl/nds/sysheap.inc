{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Free Pascal development team

    Low level memory functions

    Heap functions unit for Nintendo DS
    Copyright (c) 2006 by Francesco Lombardi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{*****************************************************************************
      OS Memory allocation / deallocation
 ****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
function sbrk2(size: longint): pointer; cdecl; external name 'sbrk';

function SysOSAlloc(size: ptruint): pointer;
begin
  result := sbrk2(size);
  if result = pointer(-1) then
    result := nil
end;

procedure SysOSFree(p: pointer; size: ptruint);
begin
  sbrk2(-size);
end;


(*
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  heap_start: longint; external name '_end';
  
function SysOSAlloc(size: ptruint): pointer;
begin
  result := @heap_start;
end;

{ $define HAS_SYSOSFREE}

procedure SysOSFree(p: pointer; size: ptruint);
begin
  
end;
<<<<<<< HEAD
<<<<<<< HEAD
*)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
