{
    Copyright (c) 1998-2004 by Florian Klaempfl and David Zhang

    Generate MIPSEL assembler for in set/case nodes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit ncpuset;

{$i fpcdefs.inc}

interface

uses
  globtype,
  nset,
  ncgset;

type
  tcpucasenode = class(tcgcasenode)
  protected
    procedure optimizevalues(var max_linear_list: aint; var max_dist: aword); override;
    function has_jumptable: boolean; override;
    procedure genjumptable(hp: pcaselabel; min_, max_: aint); override;
  end;


implementation

uses
  globals,
  systems,
  constexp,
  cpubase,
  aasmbase, aasmtai, aasmcpu, aasmdata,
  cgbase, cgutils, cgobj,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  defutil,procinfo;
=======
  procinfo;
>>>>>>> graemeg/cpstrnew
=======
  procinfo;
>>>>>>> graemeg/cpstrnew
=======
  procinfo;
>>>>>>> graemeg/cpstrnew
=======
  procinfo;
>>>>>>> origin/cpstrnew

procedure tcpucasenode.optimizevalues(var max_linear_list: aint; var max_dist: aword);
begin
  { give the jump table a higher priority }
  max_dist := (max_dist * 3) div 2;
end;


function tcpucasenode.has_jumptable: boolean;
begin
  has_jumptable := True;
end;


procedure tcpucasenode.genjumptable(hp: pcaselabel; min_, max_: aint);
var
  table: tasmlabel;
  last:  TConstExprInt;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  indexreg, jmpreg: tregister;
  href:  treference;
  jumpsegment: TAsmlist;
  opcgsize: tcgsize;
  labeltyp: taiconst_type;
=======
  indexreg, jmpreg, basereg: tregister;
  href:  treference;
  jumpsegment: TAsmlist;
>>>>>>> graemeg/cpstrnew
=======
  indexreg, jmpreg, basereg: tregister;
  href:  treference;
  jumpsegment: TAsmlist;
>>>>>>> graemeg/cpstrnew
=======
  indexreg, jmpreg, basereg: tregister;
  href:  treference;
  jumpsegment: TAsmlist;
>>>>>>> graemeg/cpstrnew
=======
  indexreg, jmpreg, basereg: tregister;
  href:  treference;
  jumpsegment: TAsmlist;
>>>>>>> origin/cpstrnew

  procedure genitem(t: pcaselabel);
  var
    i: aint;
  begin
    if assigned(t^.less) then
      genitem(t^.less);
    { fill possible hole }
    for i := last.svalue+1 to t^._low.svalue-1 do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      jumpSegment.concat(Tai_const.Create_type_sym(labeltyp,elselabel));
    for i := t^._low.svalue to t^._high.svalue do
      jumpSegment.concat(Tai_const.Create_type_sym(labeltyp,blocklabel(t^.blockid)));
=======
      jumpSegment.concat(Tai_const.Create_sym(elselabel));
    for i := t^._low.svalue to t^._high.svalue do
      jumpSegment.concat(Tai_const.Create_sym(blocklabel(t^.blockid)));
>>>>>>> graemeg/cpstrnew
=======
      jumpSegment.concat(Tai_const.Create_sym(elselabel));
    for i := t^._low.svalue to t^._high.svalue do
      jumpSegment.concat(Tai_const.Create_sym(blocklabel(t^.blockid)));
>>>>>>> graemeg/cpstrnew
=======
      jumpSegment.concat(Tai_const.Create_sym(elselabel));
    for i := t^._low.svalue to t^._high.svalue do
      jumpSegment.concat(Tai_const.Create_sym(blocklabel(t^.blockid)));
>>>>>>> graemeg/cpstrnew
=======
      jumpSegment.concat(Tai_const.Create_sym(elselabel));
    for i := t^._low.svalue to t^._high.svalue do
      jumpSegment.concat(Tai_const.Create_sym(blocklabel(t^.blockid)));
>>>>>>> origin/cpstrnew
    last := t^._high;
    if assigned(t^.greater) then
      genitem(t^.greater);
  end;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  opcgsize:=def_cgsize(opsize);
  last:=min_;
  jumpsegment := current_procinfo.aktlocaldata;
  if not (jumptable_no_range) then
    begin
      { a <= x <= b <-> unsigned(x-a) <= (b-a) }
      cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,opcgsize,aint(min_),hregister);
      { case expr greater than max_ => goto elselabel }
      cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,opcgsize,OC_A,aint(max_)-aint(min_),hregister,elselabel);
      min_:=0;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  jumpsegment := current_procinfo.aktlocaldata;
  if not (jumptable_no_range) then
    begin
      { case expr less than min_ => goto elselabel }
      cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, jmp_lt, aint(min_), hregister, elselabel);
      { case expr greater than max_ => goto elselabel }
      cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, opsize, jmp_gt, aint(max_), hregister, elselabel);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;
  current_asmdata.getjumplabel(table);
  indexreg := cg.getaddressregister(current_asmdata.CurrAsmList);
  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_SHL, OS_ADDR, 2, hregister, indexreg);
  { create reference }
  reference_reset_symbol(href, table, 0, sizeof(aint));
  href.offset := (-aint(min_)) * 4;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  href.base:=indexreg;
  jmpreg := cg.getaddressregister(current_asmdata.CurrAsmList);
  cg.a_load_ref_reg(current_asmdata.CurrAsmList, OS_ADDR, OS_ADDR, href, jmpreg);

  if (cs_create_pic in current_settings.moduleswitches) then
    begin
      cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_ADD,OS_ADDR,NR_GP,jmpreg,jmpreg);
      labeltyp:=aitconst_gotoff_symbol;
    end
  else
    labeltyp:=aitconst_ptr;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  basereg     := cg.getaddressregister(current_asmdata.CurrAsmList);
  cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList, href, basereg);

  jmpreg := cg.getaddressregister(current_asmdata.CurrAsmList);

  reference_reset(href, sizeof(aint));
  href.index := indexreg;
  href.base  := basereg;
  cg.a_load_ref_reg(current_asmdata.CurrAsmList, OS_ADDR, OS_ADDR, href, jmpreg);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_JR, jmpreg));
  { Delay slot }
  current_asmdata.CurrAsmList.concat(taicpu.op_none(A_NOP));
  { generate jump table }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  new_section(jumpSegment,sec_rodata,current_procinfo.procdef.mangledname,sizeof(aint));
  jumpSegment.concat(Tai_label.Create(table));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if not(cs_opt_size in current_settings.optimizerswitches) then
    jumpSegment.concat(Tai_Align.Create_Op(4, 0));
  jumpSegment.concat(Tai_label.Create(table));
  last := min_;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  genitem(hp);
end;



begin
  ccasenode := tcpucasenode;
end.
