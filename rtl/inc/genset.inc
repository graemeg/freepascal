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


{****************************************************************************
                                 Var sets
 ****************************************************************************}

const
  maxsetsize = 32;

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
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
  begin
    FillChar(data,size,0);
    tbsetarray(data)[b]:=1;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_CREATE_ELEMENT}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_SET_BYTE}
{
  add the element b to the set "source"
}
procedure fpc_varset_set(const source;var dest; b,size : ptrint); compilerproc;
  type
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
  begin
    move(source,dest,size);
    tbsetarray(dest)[b]:=1;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_SET_BYTE}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_UNSET_BYTE}
{
   suppresses the element b to the set pointed by p
   used for exclude(set,element)
}
procedure fpc_varset_unset(const source;var dest; b,size : ptrint); compilerproc;
  type
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
  begin
    move(source,dest,size);
    tbsetarray(dest)[b]:=0;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_UNSET_BYTE}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_SET_RANGE}
{
  adds the range [l..h] to the set orgset
}
procedure fpc_varset_set_range(const orgset; var dest;l,h,size : ptrint); compilerproc;
  type
    tbsetarray = bitpacked array[0..high(sizeint)-1] of 0..1;
  var
    i : ptrint;
  begin
    move(orgset,dest,size);
    for i:=l to h do
       tbsetarray(dest)[i]:=1;
  end;
{$endif ndef FPC_SYSTEM_HAS_FPC_VARSET_SET_RANGE}


{$ifndef FPC_SYSTEM_HAS_FPC_VARSET_ADD_SETS}
{
  adds set1 and set2 into set dest
}
procedure fpc_varset_add_sets(const set1,set2; var dest;size : ptrint); compilerproc;
  type
    tbytearray = array[0..maxsetsize-1] of byte;
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
    tbytearray = array[0..maxsetsize-1] of byte;
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
    tbytearray = array[0..maxsetsize-1] of byte;
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
    tbytearray = array[0..maxsetsize-1] of byte;
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
    tbytearray = array[0..maxsetsize-1] of byte;
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
    tbytearray = array[0..maxsetsize-1] of byte;
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
