{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    Include file with set operations called by the compiler

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}

{$ifndef FPC_SYSTEM_HAS_FPC_SET_LOAD_SMALL}
{ Error No pascal version of FPC_SET_LOAD_SMALL}
 { THIS DEPENDS ON THE ENDIAN OF THE ARCHITECTURE!
   Not anymore PM}

function fpc_set_load_small(l: fpc_small_set): fpc_normal_set; [public,alias:'FPC_SET_LOAD_SMALL']; compilerproc;
 {
  load a normal set p from a smallset l
 }
 begin
   FillDWord(fpc_set_load_small,sizeof(fpc_set_load_small) div 4,0);
   move(l,fpc_set_load_small,sizeof(l));
 end;
{$endif FPC_SYSTEM_HAS_FPC_SET_LOAD_SMALL}


{$ifndef FPC_SYSTEM_HAS_FPC_SET_CREATE_ELEMENT}
function fpc_set_create_element(b : byte): fpc_normal_set;[public,alias:'FPC_SET_CREATE_ELEMENT']; compilerproc;
 {
  create a new set in p from an element b
 }
 begin
   FillDWord(fpc_set_create_element,SizeOf(fpc_set_create_element) div 4,0);
{$ifndef FPC_NEW_BIGENDIAN_SETS}
   fpc_set_create_element[b div 32] := 1 shl (b mod 32);
{$else}
   fpc_set_create_element[b] := 1;
{$endif}
 end;
{$endif FPC_SYSTEM_HAS_FPC_SET_CREATE_ELEMENT}

{$ifndef FPC_SYSTEM_HAS_FPC_SET_SET_BYTE}

 function fpc_set_set_byte(const source: fpc_normal_set; b : byte): fpc_normal_set; compilerproc;
 {
  add the element b to the set "source"
 }
  var
   c: longint;
  begin
    move(source,fpc_set_set_byte,sizeof(source));
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    c := fpc_set_set_byte[b div 32];
    c := (1 shl (b mod 32)) or c;
    fpc_set_set_byte[b div 32] := c;
{$else}
    fpc_set_set_byte[b] := 1;
{$endif}
  end;
{$endif FPC_SYSTEM_HAS_FPC_SET_SET_BYTE}


{$ifndef FPC_SYSTEM_HAS_FPC_SET_UNSET_BYTE}

function fpc_set_unset_byte(const source: fpc_normal_set; b : byte): fpc_normal_set; compilerproc;
 {
   suppresses the element b to the set pointed by p
   used for exclude(set,element)
 }
  var
   c: longint;
  begin
    move(source,fpc_set_unset_byte,sizeof(source));
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    c := fpc_set_unset_byte[b div 32];
    c := c and not (1 shl (b mod 32));
    fpc_set_unset_byte[b div 32] := c;
{$else}
    fpc_set_unset_byte[b] := 0;
{$endif}
  end;
{$endif FPC_SYSTEM_HAS_FPC_SET_UNSET_BYTE}
>>>>>>> graemeg/fixes_2_2

{****************************************************************************
                                 Var sets
 ****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_LOAD_SMALL}
{
  convert sets
}
{$ifdef FPC_SETBASE_USED}
procedure fpc_varset_load(const l;sourcesize : longint;var dest;size,srcminusdstbase : ptrint); compilerproc;
  var
    srcptr, dstptr: pointer;
  begin
    srcptr:=@l;
    dstptr:=@dest;
    { going from a higher base to a lower base, e.g.
      src: 001f0000, base=2,size=4 -> 0000001f0000 in base 0
      dstr in base = 1 (-> srcminusdstbase = 1) -> to
      00001f0000, base=1 -> need to prepend "srcminusdstbase" zero bytes
    }
    if (srcminusdstbase>0) then
      begin
<<<<<<< HEAD
        { fill the skipped part with 0 }
        fillchar(dstptr^,srcminusdstbase,0);
        inc(dstptr,srcminusdstbase);
        dec(size,srcminusdstbase);
      end
    else if (srcminusdstbase<0) then
      begin
<<<<<<< HEAD
        { inc/dec switched since srcminusdstbase < 0 }
        dec(srcptr,srcminusdstbase);
        inc(sourcesize,srcminusdstbase);
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}
        c := fpc_set_set_range[i div 32];
        c := (1 shl (i mod 32)) or c;
        fpc_set_set_range[i div 32] := c;
{$else}
        fpc_set_set_range[i] := 1;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
      end;

<<<<<<< HEAD
    if sourcesize>size then
      sourcesize:=size;
    move(srcptr^,dstptr^,sourcesize);
    { fill the  leftover (if any) with 0 }
    FillChar((dstptr+sourcesize)^,size-sourcesize,0);
  end;
=======
=======
      end;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_SET_SET_RANGE}


>>>>>>> origin/fixes_2_2
{$ifndef FPC_SYSTEM_HAS_FPC_SET_ADD_SETS}
 function fpc_set_add_sets(const set1,set2: fpc_normal_set): fpc_normal_set;[public,alias:'FPC_SET_ADD_SETS']; compilerproc;
 var
   src1: fpc_normal_set_long absolute set1;
   src2: fpc_normal_set_long absolute set2;
   dest: fpc_normal_set_long absolute fpc_set_add_sets;
 {
   adds set1 and set2 into set dest
 }
  var
    i: integer;
   begin
     for i:=0 to 7 do
       dest[i] := src1[i] or src2[i];
   end;
{$endif}
>>>>>>> graemeg/fixes_2_2

{$else FPC_SETBASE_USED}

<<<<<<< HEAD
=======
{$ifndef FPC_SYSTEM_HAS_FPC_SET_MUL_SETS}
 function fpc_set_mul_sets(const set1,set2: fpc_normal_set): fpc_normal_set;[public,alias:'FPC_SET_MUL_SETS']; compilerproc;
 var
   src1: fpc_normal_set_long absolute set1;
   src2: fpc_normal_set_long absolute set2;
   dest: fpc_normal_set_long absolute fpc_set_mul_sets;
 {
   multiplies (takes common elements of) set1 and set2 result put in dest
 }
   var
    i: integer;
   begin
     for i:=0 to 7 do
       dest[i] := src1[i] and src2[i];
   end;
{$endif}


{$ifndef FPC_SYSTEM_HAS_FPC_SET_SUB_SETS}
 function fpc_set_sub_sets(const set1,set2: fpc_normal_set): fpc_normal_set;[public,alias:'FPC_SET_SUB_SETS']; compilerproc;
 var
   src1: fpc_normal_set_long absolute set1;
   src2: fpc_normal_set_long absolute set2;
   dest: fpc_normal_set_long absolute fpc_set_sub_sets;
 {
  computes the diff from set1 to set2 result in dest
 }
   var
    i: integer;
   begin
     for i:=0 to 7 do
       dest[i] := src1[i] and not src2[i];
   end;
{$endif}


{$ifndef FPC_SYSTEM_HAS_FPC_SET_SYMDIF_SETS}
 function fpc_set_symdif_sets(const set1,set2: fpc_normal_set): fpc_normal_set;[public,alias:'FPC_SET_SYMDIF_SETS']; compilerproc;
 var
   src1: fpc_normal_set_long absolute set1;
   src2: fpc_normal_set_long absolute set2;
   dest: fpc_normal_set_long absolute fpc_set_symdif_sets;
 {
   computes the symetric diff from set1 to set2 result in dest
 }
   var
    i: integer;
   begin
     for i:=0 to 7 do
       dest[i] := src1[i] xor src2[i];
   end;
{$endif}

{$ifndef FPC_SYSTEM_HAS_FPC_SET_COMP_SETS}
 function fpc_set_comp_sets(const set1,set2 : fpc_normal_set):boolean;[public,alias:'FPC_SET_COMP_SETS'];compilerproc;
 {
  compares set1 and set2 zeroflag is set if they are equal
 }
   var
    i: integer;
    src1: fpc_normal_set_long absolute set1;
    src2: fpc_normal_set_long absolute set2;
   begin
     fpc_set_comp_sets:= false;
     for i:=0 to 7 do
       if src1[i] <> src2[i] then
         exit;
     fpc_set_comp_sets:= true;
   end;
{$endif}



{$ifndef FPC_SYSTEM_HAS_FPC_SET_CONTAINS_SET}
 function fpc_set_contains_sets(const set1,set2 : fpc_normal_set):boolean;[public,alias:'FPC_SET_CONTAINS_SETS'];compilerproc;
 {
  on exit, zero flag is set if set1 <= set2 (set2 contains set1)
 }
 var
  i : integer;
  src1: fpc_normal_set_long absolute set1;
  src2: fpc_normal_set_long absolute set2;
 begin
   fpc_set_contains_sets:= false;
   for i:=0 to 7 do
     if (src1[i] and not src2[i]) <> 0 then
       exit;
   fpc_set_contains_sets:= true;
 end;
{$endif}

{$endif ndef FPC_NEW_BIGENDIAN_SETS}

{****************************************************************************
                                 Var sets
 ****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_LOAD_SMALL}
{
  convert sets
}
{$ifdef FPC_SETBASE_USED}
procedure fpc_varset_load(const l;sourcesize : longint;var dest;size,srcminusdstbase : ptrint); compilerproc;
  var
    srcptr, dstptr: pointer;
  begin
    srcptr:=@l;
    dstptr:=@dest;
    { going from a higher base to a lower base, e.g.
      src: 001f0000, base=2,size=4 -> 0000001f0000 in base 0
      dstr in base = 1 (-> srcminusdstbase = 1) -> to
      00001f0000, base=1 -> need to prepend "srcminusdstbase" zero bytes
    }
    if (srcminusdstbase>0) then
      begin
        { fill the skipped part with 0 }
        fillchar(dstptr^,srcminusdstbase,0);
        inc(dstptr,srcminusdstbase);
        dec(size,srcminusdstbase);
      end
    else if (srcminusdstbase<0) then
      begin
        { inc/dec switched since srcminusdstbase < 0 }
        dec(srcptr,srcminusdstbase);
        inc(sourcesize,srcminusdstbase);
      end;

    if sourcesize>size then
      sourcesize:=size;
    move(srcptr^,dstptr^,sourcesize);
    { fill the  leftover (if any) with 0 }
    FillChar((dstptr+sourcesize)^,size-sourcesize,0);
  end;

{$else FPC_SETBASE_USED}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure fpc_varset_load(const l;sourcesize : longint;var dest;size : ptrint); compilerproc;
  begin
    if sourcesize>size then
      sourcesize:=size;
    move(l,plongint(@dest)^,sourcesize);
    FillChar((@dest+sourcesize)^,size-sourcesize,0);
  end;
{$endif FPC_SETBASE_USED}

{$endif ndef FPC_SYSTEM_HAS_FPC_SET_LOAD_SMALL}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_CREATE_ELEMENT}
{
  create a new set in p from an element b
}
procedure fpc_varset_create_element(b,size : ptrint; var data); compilerproc;
  type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> origin/cpstrnew
  begin
    FillChar(data,size,0);
    tbsetarray(data)[b]:=1;
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray = array[0..sizeof(sizeint)-1] of byte;
{$else}
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
{$endif}
  begin
    FillChar(data,size,0);
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray(data)[b div 8]:=1 shl (b mod 8);
{$else}
    tbsetarray(data)[b]:=1;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_CREATE_ELEMENT}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_SET_BYTE}
{
  add the element b to the set "source"
}
procedure fpc_varset_set(const source;var dest; b,size : ptrint); compilerproc;
  type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> origin/cpstrnew
  begin
    move(source,dest,size);
    tbsetarray(dest)[b]:=1;
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray = array[0..sizeof(sizeint)-1] of byte;
{$else}
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
{$endif}
  begin
    move(source,dest,size);
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray(dest)[b div 8]:=tbytearray(dest)[b div 8] or (1 shl (b mod 8));
{$else}
    tbsetarray(dest)[b]:=1;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_SET_BYTE}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_UNSET_BYTE}
{
   suppresses the element b to the set pointed by p
   used for exclude(set,element)
}
procedure fpc_varset_unset(const source;var dest; b,size : ptrint); compilerproc;
  type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> origin/cpstrnew
  begin
    move(source,dest,size);
    tbsetarray(dest)[b]:=0;
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray = array[0..sizeof(sizeint)-1] of byte;
{$else}
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
{$endif}
  begin
    move(source,dest,size);
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray(dest)[b div 8]:=tbytearray(dest)[b div 8] and not (1 shl (b mod 8));
{$else}
    tbsetarray(dest)[b]:=0;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_UNSET_BYTE}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_SET_RANGE}
{
  adds the range [l..h] to the set orgset
}
procedure fpc_varset_set_range(const orgset; var dest;l,h,size : ptrint); compilerproc;
  type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> graemeg/cpstrnew
=======
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}
    tbytearray = array[0..sizeof(sizeint)-1] of byte;
{$else}
    tbsetarray = bitpacked array[0..sizeof(sizeint)-1] of 0..1;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  var
    i : ptrint;
  begin
    move(orgset,dest,size);
    for i:=l to h do
<<<<<<< HEAD
<<<<<<< HEAD
       tbsetarray(dest)[i]:=1;
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_NEW_BIGENDIAN_SETS}
       tbytearray(dest)[i div 8]:=(1 shl (i mod 8)) or tbytearray(dest)[i div 8];
{$else}
       tbsetarray(dest)[i]:=1;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_SET_RANGE}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_ADD_SETS}
{
  adds set1 and set2 into set dest
}
procedure fpc_varset_add_sets(const set1,set2; var dest;size : ptrint); compilerproc;
  type
    tbytearray = array[0..high(sizeint)-1] of byte;
  var
    i : ptrint;
  begin
     for i:=0 to size-1 do
       tbytearray(dest)[i]:=tbytearray(set1)[i] or tbytearray(set2)[i];
   end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_ADD_SETS}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_MUL_SETS}
{
  multiplies (takes common elements of) set1 and set2 result put in dest
}
procedure fpc_varset_mul_sets(const set1,set2; var dest;size : ptrint); compilerproc;
  type
    tbytearray = array[0..high(sizeint)-1] of byte;
  var
    i : ptrint;
  begin
    for i:=0 to size-1 do
      tbytearray(dest)[i]:=tbytearray(set1)[i] and tbytearray(set2)[i];
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_MUL_SETS}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_SUB_SETS}
{
  computes the diff from set1 to set2 result in dest
}
procedure fpc_varset_sub_sets(const set1,set2; var dest;size : ptrint); compilerproc;
  type
    tbytearray = array[0..high(sizeint)-1] of byte;
  var
    i : ptrint;
  begin
     for i:=0 to size-1 do
       tbytearray(dest)[i]:=tbytearray(set1)[i] and not tbytearray(set2)[i];
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_SUB_SETS}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_SYMDIF_SETS}
{
   computes the symetric diff from set1 to set2 result in dest
}
procedure fpc_varset_symdif_sets(const set1,set2; var dest;size : ptrint); compilerproc;
  type
    tbytearray = array[0..high(sizeint)-1] of byte;
  var
    i : ptrint;
   begin
     for i:=0 to size-1 do
       tbytearray(dest)[i]:=tbytearray(set1)[i] xor tbytearray(set2)[i];
   end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_SYMDIF_SETS}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_COMP_SETS}
{
  compares set1 and set2 zeroflag is set if they are equal
}
function fpc_varset_comp_sets(const set1,set2;size : ptrint):boolean; compilerproc;
  type
    tbytearray = array[0..high(sizeint)-1] of byte;
  var
    i : ptrint;
  begin
    fpc_varset_comp_sets:= false;
    for i:=0 to size-1 do
      if tbytearray(set1)[i]<>tbytearray(set2)[i] then
        exit;
    fpc_varset_comp_sets:=true;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_COMP_SETS}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_CONTAINS_SET}
{
  on exit, zero flag is set if set1 <= set2 (set2 contains set1)
}
function fpc_varset_contains_sets(const set1,set2;size : ptrint):boolean; compilerproc;
  type
    tbytearray = array[0..high(sizeint)-1] of byte;
  var
    i : ptrint;
  begin
    fpc_varset_contains_sets:= false;
    for i:=0 to size-1 do
      if (tbytearray(set1)[i] and not tbytearray(set2)[i])<>0 then
        exit;
    fpc_varset_contains_sets:=true;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_CONTAINS_SET}
