{
    Copyright (c) 2000-2009 by Florian Klaempfl and David Zhang

    Code generation for add nodes on the FVM32

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
unit ncpuadd;

{$i fpcdefs.inc}

interface

uses
  node, ncgadd, cpubase, aasmbase, cgbase;

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  { tmipsaddnode }

  tmipsaddnode = class(tcgaddnode)
  private
    procedure cmp64_lt(left_reg, right_reg: TRegister64;unsigned:boolean);
    procedure cmp64_le(left_reg, right_reg: TRegister64;unsigned:boolean);
    procedure second_generic_cmp32(unsigned: boolean);
    procedure second_mul64bit;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmipsaddnode = class(tcgaddnode)
  private
    function cmp64_lt(left_reg, right_reg: TRegister64): TRegister;
    function cmp64_le(left_reg, right_reg: TRegister64): TRegister;
    function cmp64_eq(left_reg, right_reg: TRegister64): TRegister;
    function cmp64_ne(left_reg, right_reg: TRegister64): TRegister;
    function cmp64_ltu(left_reg, right_reg: TRegister64): TRegister;
    function cmp64_leu(left_reg, right_reg: TRegister64): TRegister;

    function GetRes_register(unsigned: boolean; this_reg, left_reg, right_reg: TRegister): TRegister;
    function GetRes64_register(unsigned: boolean; {this_reg,} left_reg, right_reg: TRegister64): TRegister;
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
  protected
    procedure second_addfloat; override;
    procedure second_cmpfloat; override;
    procedure second_cmpboolean; override;
    procedure second_cmpsmallset; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure second_add64bit; override;
    procedure second_cmp64bit; override;
    procedure second_cmpordinal; override;
    procedure second_addordinal; override;
  public
    function use_generic_mul32to64: boolean; override;
    function use_generic_mul64bit: boolean; override;
=======
    procedure second_cmp64bit; override;
    procedure second_cmpordinal; override;
>>>>>>> graemeg/cpstrnew
=======
    procedure second_cmp64bit; override;
    procedure second_cmpordinal; override;
>>>>>>> graemeg/cpstrnew
=======
    procedure second_cmp64bit; override;
    procedure second_cmpordinal; override;
>>>>>>> graemeg/cpstrnew
=======
    procedure second_cmp64bit; override;
    procedure second_cmpordinal; override;
>>>>>>> origin/cpstrnew
  end;

implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  systems, globtype, globals,
=======
  systems,
>>>>>>> graemeg/cpstrnew
=======
  systems,
>>>>>>> graemeg/cpstrnew
=======
  systems,
>>>>>>> graemeg/cpstrnew
=======
  systems,
>>>>>>> origin/cpstrnew
  cutils, verbose,
  paramgr,
  aasmtai, aasmcpu, aasmdata,
  defutil,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  cpuinfo,
  {cgbase,} cgcpu, cgutils,
  cpupara,
  procinfo,
  symconst,symdef,
  ncon, nset, nadd,
  ncgutil, hlcgobj, cgobj;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {cgbase,} cgcpu, cgutils,
  cpupara,
  ncon, nset, nadd,
  ncgutil, cgobj;
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

{*****************************************************************************
                               tmipsaddnode
*****************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure tmipsaddnode.second_generic_cmp32(unsigned: boolean);
var
  cond: TOpCmp;
begin
  pass_left_right;
  force_reg_left_right(True, True);
  location_reset(location,LOC_FLAGS,OS_NO);

  cond:=cmpnode2topcmp(unsigned);
  if nf_swapped in flags then
    cond:=swap_opcmp(cond);

  location.resflags.cond:=cond;
  location.resflags.reg1:=left.location.register;
  location.resflags.use_const:=(right.location.loc=LOC_CONSTANT);
  if location.resflags.use_const then
    location.resflags.value:=right.location.value
  else
    location.resflags.reg2:=right.location.register;
end;


procedure tmipsaddnode.second_add64bit;
begin
  if (nodetype=muln) then
    second_mul64bit
  else
    inherited second_add64bit;
end;


const
  cmpops: array[boolean] of TOpCmp = (OC_LT,OC_B);

procedure tmipsaddnode.cmp64_lt(left_reg, right_reg: TRegister64;unsigned: boolean);
begin
  cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cmpops[unsigned],right_reg.reghi,left_reg.reghi,location.truelabel);
  cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.falselabel);
  cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_B,right_reg.reglo,left_reg.reglo,location.truelabel);
  cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
end;


procedure tmipsaddnode.cmp64_le(left_reg, right_reg: TRegister64;unsigned: boolean);
begin
  cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,cmpops[unsigned],left_reg.reghi,right_reg.reghi,location.falselabel);
  cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.truelabel);
  cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_B,left_reg.reglo,right_reg.reglo,location.falselabel);
  cg.a_jmp_always(current_asmdata.CurrAsmList,location.truelabel);
end;


procedure tmipsaddnode.second_cmp64bit;
var
  truelabel,
  falselabel: tasmlabel;
  unsigned: boolean;
  left_reg,right_reg: TRegister64;
begin
  current_asmdata.getjumplabel(truelabel);
  current_asmdata.getjumplabel(falselabel);
  location_reset_jump(location,truelabel,falselabel);

  pass_left_right;
  force_reg_left_right(true,true);

  unsigned:=not(is_signed(left.resultdef)) or
            not(is_signed(right.resultdef));

  left_reg:=left.location.register64;
  if (right.location.loc=LOC_CONSTANT) then
    begin
      if lo(right.location.value64)=0 then
        right_reg.reglo:=NR_R0
      else
        begin
          right_reg.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,lo(right.location.value64),right_reg.reglo);
        end;
      if hi(right.location.value64)=0 then
        right_reg.reghi:=NR_R0
      else
        begin
          right_reg.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,hi(right.location.value64),right_reg.reghi);
        end;
    end
  else
    right_reg:=right.location.register64;

  case NodeType of
    equaln:
      begin
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.falselabel);
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reglo,right_reg.reglo,location.falselabel);
        cg.a_jmp_always(current_asmdata.CurrAsmList,location.truelabel);
      end;
    unequaln:
      begin
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reghi,right_reg.reghi,location.truelabel);
        cg.a_cmp_reg_reg_label(current_asmdata.CurrAsmList,OS_INT,OC_NE,left_reg.reglo,right_reg.reglo,location.truelabel);
        cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
      end;
  else
    if nf_swapped in flags then
      case NodeType of
        ltn:
          cmp64_lt(right_reg, left_reg,unsigned);
        lten:
          cmp64_le(right_reg, left_reg,unsigned);
        gtn:
          cmp64_lt(left_reg, right_reg,unsigned);
        gten:
          cmp64_le(left_reg, right_reg,unsigned);
      end
    else
      case NodeType of
        ltn:
          cmp64_lt(left_reg, right_reg,unsigned);
        lten:
          cmp64_le(left_reg, right_reg,unsigned);
        gtn:
          cmp64_lt(right_reg, left_reg,unsigned);
        gten:
          cmp64_le(right_reg, left_reg,unsigned);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function tmipsaddnode.GetRes_register(unsigned: boolean; this_reg, left_reg, right_reg: TRegister): TRegister;
var
  tmp_asm_op: tasmop;
begin
  case NodeType of
    equaln:
      tmp_asm_op := A_SEQ;
    unequaln:
      tmp_asm_op := A_SNE;
    else
      if not (unsigned) then
      begin
        if nf_swapped in flags then
          case NodeType of
            ltn:
              tmp_asm_op := A_SGT;
            lten:
              tmp_asm_op := A_SGE;
            gtn:
              tmp_asm_op := A_SLT;
            gten:
              tmp_asm_op := A_SLE;
          end
        else
          case NodeType of
            ltn:
              tmp_asm_op := A_SLT;
            lten:
              tmp_asm_op := A_SLE;
            gtn:
              tmp_asm_op := A_SGT;
            gten:
              tmp_asm_op := A_SGE;
          end;
      end
      else
      begin
        if nf_swapped in Flags then
          case NodeType of
            ltn:
              tmp_asm_op := A_SGTU;
            lten:
              tmp_asm_op := A_SGEU;
            gtn:
              tmp_asm_op := A_SLTU;
            gten:
              tmp_asm_op := A_SLEU;
          end
        else
          case NodeType of
            ltn:
              tmp_asm_op := A_SLTU;
            lten:
              tmp_asm_op := A_SLEU;
            gtn:
              tmp_asm_op := A_SGTU;
            gten:
              tmp_asm_op := A_SGEU;
          end;
      end;
  end;
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(tmp_asm_op, this_reg, left_reg, right_reg));
  GetRes_register := this_reg;
end;

function tmipsaddnode.cmp64_eq(left_reg, right_reg: TRegister64): TRegister;
var
  lfcmp64_L4: tasmlabel;
begin

  current_asmdata.getjumplabel(lfcmp64_L4);

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 0));

  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, left_reg.reghi, right_reg.reghi, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_sym(A_BNE, left_reg.reglo, right_reg.reglo, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 1));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L4);
  cmp64_eq := NR_TCR10;
end;

function tmipsaddnode.cmp64_ne(left_reg, right_reg: TRegister64): TRegister;
var
  lfcmp64_L4: tasmlabel;
begin

  current_asmdata.getjumplabel(lfcmp64_L4);

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 1));

  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, left_reg.reghi, right_reg.reghi, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_sym(A_BNE, left_reg.reglo, right_reg.reglo, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 0));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L4);
  cmp64_ne := NR_TCR10;
end;

function tmipsaddnode.cmp64_lt(left_reg, right_reg: TRegister64): TRegister;
var
  lfcmp64_L4, lfcmp64_L5: tasmlabel;
begin
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 0));

  current_asmdata.getjumplabel(lfcmp64_L4);
  current_asmdata.getjumplabel(lfcmp64_L5);

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLT, NR_TCR11, left_reg.reghi, right_reg.reghi));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L5));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, left_reg.reghi, right_reg.reghi, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, NR_TCR11, left_reg.reglo, right_reg.reglo));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L5));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_B, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L5);
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 1));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L4);
  cmp64_lt := NR_TCR10;
end;

function tmipsaddnode.cmp64_le(left_reg, right_reg: TRegister64): TRegister;
var
  lfcmp64_L4, lfcmp64_L5: tasmlabel;
begin
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 0));

  current_asmdata.getjumplabel(lfcmp64_L4);
  current_asmdata.getjumplabel(lfcmp64_L5);

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLT, NR_TCR11, right_reg.reghi, left_reg.reghi));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, right_reg.reghi, left_reg.reghi, lfcmp64_L5));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, NR_TCR11, right_reg.reglo, left_reg.reglo));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L5);
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 1));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L4);
  cmp64_le := NR_TCR10;
end;

function tmipsaddnode.cmp64_ltu(left_reg, right_reg: TRegister64): TRegister;
var
  lfcmp64_L4, lfcmp64_L5: tasmlabel;
begin
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 0));

  current_asmdata.getjumplabel(lfcmp64_L4);
  current_asmdata.getjumplabel(lfcmp64_L5);

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, NR_TCR11, left_reg.reghi, right_reg.reghi));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L5));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, left_reg.reghi, right_reg.reghi, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, NR_TCR11, left_reg.reglo, right_reg.reglo));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L5));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_B, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L5);
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 1));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L4);
  cmp64_ltu := NR_TCR10;
end;

function tmipsaddnode.cmp64_leu(left_reg, right_reg: TRegister64): TRegister;
var
  lfcmp64_L4, lfcmp64_L5: tasmlabel;
begin
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 0));

  current_asmdata.getjumplabel(lfcmp64_L4);
  current_asmdata.getjumplabel(lfcmp64_L5);

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, NR_TCR11, right_reg.reghi, left_reg.reghi));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, right_reg.reghi, left_reg.reghi, lfcmp64_L5));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, NR_TCR11, right_reg.reglo, left_reg.reglo));
  current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BNE, NR_TCR11, NR_R0, lfcmp64_L4));
  current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L5);
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, NR_TCR10, 1));

  cg.a_label(current_asmdata.CurrAsmList, lfcmp64_L4);
  cmp64_leu := NR_TCR10;
end;

function tmipsaddnode.GetRes64_register(unsigned: boolean; //this_reg: TRegister;
                                                            left_reg, right_reg: TRegister64): TRegister;
var
  tmpreg: TRegister;
  lfcmp64_L4, lfcmp_L5: tasmlabel;
begin
  case NodeType of
    equaln:
    begin
      GetRes64_register := cmp64_eq(left_reg, right_reg);
    end;
    unequaln:
      GetRes64_register := cmp64_ne(left_reg, right_reg);
    else
      if not (unsigned) then
      begin
        if nf_swapped in flags then
          case NodeType of
            ltn:
              GetRes64_register := cmp64_lt(right_reg, left_reg);
            lten:
              GetRes64_register := cmp64_le(right_reg, left_reg);
            gtn:
              GetRes64_register := cmp64_lt(left_reg, right_reg);
            gten:
              GetRes64_register := cmp64_le(left_reg, right_reg);
          end
        else
          case NodeType of
            ltn:
              GetRes64_register := cmp64_lt(left_reg, right_reg);
            lten:
              GetRes64_register := cmp64_le(left_reg, right_reg);
            gtn:
              GetRes64_register := cmp64_lt(right_reg, left_reg);
            gten:
              GetRes64_register := cmp64_le(right_reg, left_reg);
          end;
      end
      else
      begin
        if nf_swapped in Flags then
          case NodeType of
            ltn:
              GetRes64_register := cmp64_ltu(right_reg, left_reg);
            lten:
              GetRes64_register := cmp64_leu(right_reg, left_reg);
            gtn:
              GetRes64_register := cmp64_ltu(left_reg, right_reg);
            gten:
              GetRes64_register := cmp64_leu(left_reg, right_reg);
          end
        else
          case NodeType of
            ltn:
              GetRes64_register := cmp64_ltu(left_reg, right_reg);
            lten:
              GetRes64_register := cmp64_leu(left_reg, right_reg);
            gtn:
              GetRes64_register := cmp64_ltu(right_reg, left_reg);
            gten:
              GetRes64_register := cmp64_leu(right_reg, left_reg);
          end;
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
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure tmipsaddnode.second_addfloat;
var
  op: TAsmOp;
begin
  pass_left_right;
  if (nf_swapped in flags) then
    swapleftright;

        { force fpureg as location, left right doesn't matter
          as both will be in a fpureg }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  hlcg.location_force_fpureg(current_asmdata.CurrAsmList, left.location, left.resultdef, True);
  hlcg.location_force_fpureg(current_asmdata.CurrAsmList, right.location, right.resultdef, True);

  location_reset(location, LOC_FPUREGISTER, def_cgsize(resultdef));
  location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, True);
  location_force_fpureg(current_asmdata.CurrAsmList, right.location, (left.location.loc <> LOC_CFPUREGISTER));

  location_reset(location, LOC_FPUREGISTER, def_cgsize(resultdef));
  if left.location.loc <> LOC_CFPUREGISTER then
    location.Register := left.location.Register
  else
    location.Register := right.location.Register;
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

  case nodetype of
    addn:
    begin
      if location.size = OS_F64 then
        op := A_ADD_D
      else
        op := A_ADD_S;
    end;
    muln:
    begin
      if location.size = OS_F64 then
        op := A_MUL_D
      else
        op := A_MUL_S;
    end;
    subn:
    begin
      if location.size = OS_F64 then
        op := A_SUB_D
      else
        op := A_SUB_S;
    end;
    slashn:
    begin
      if location.size = OS_F64 then
        op := A_DIV_D
      else
        op := A_DIV_S;
    end;
    else
      internalerror(200306014);
  end;
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,
    location.Register, left.location.Register, right.location.Register));

end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
  ops_cmpfloat: array[boolean,ltn..unequaln] of TAsmOp = (
  // ltn       lten      gtn       gten      equaln    unequaln
    (A_C_LT_S, A_C_LE_S, A_C_LT_S, A_C_LE_S, A_C_EQ_S, A_C_EQ_S),
    (A_C_LT_D, A_C_LE_D, A_C_LT_D, A_C_LE_D, A_C_EQ_D, A_C_EQ_D)
  );

procedure tmipsaddnode.second_cmpfloat;
var
  op: tasmop;
  lreg,rreg: tregister;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure tmipsaddnode.second_cmpfloat;
var
  op: tasmop;
  lfcmptrue, lfcmpfalse: tasmlabel;
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
begin
  pass_left_right;
  if nf_swapped in flags then
    swapleftright;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  hlcg.location_force_fpureg(current_asmdata.CurrAsmList, left.location, left.resultdef, True);
  hlcg.location_force_fpureg(current_asmdata.CurrAsmList, right.location, right.resultdef, True);
  location_reset(location, LOC_FLAGS, OS_NO);

  op:=ops_cmpfloat[left.location.size=OS_F64,nodetype];

  if (nodetype in [gtn,gten]) then
    begin
      lreg:=right.location.register;
      rreg:=left.location.register;
    end
  else
    begin
      lreg:=left.location.register;
      rreg:=right.location.register;
    end;

  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,lreg,rreg));
  location.resflags.reg1:=NR_FCC0;
  if (nodetype=unequaln) then
    location.resflags.cond:=OC_EQ
  else
    location.resflags.cond:=OC_NE;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { force fpureg as location, left right doesn't matter
    as both will be in a fpureg }
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, True);
  location_force_fpureg(current_asmdata.CurrAsmList, right.location, True);

  location_reset(location, LOC_REGISTER, OS_INT);
  location.Register := NR_TCR0;

  case NodeType of
    equaln:
    begin
      if left.location.size = OS_F64 then
        op := A_C_EQ_D
      else
        op := A_C_EQ_S;
      current_asmdata.getjumplabel(lfcmpfalse);
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_OR, location.Register {NR_TCR0}, NR_R0, NR_R0));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, left.location.Register, right.location.Register));
      current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_BC1F, lfcmpfalse)); //lfcmpfalse
      current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ORI, location.Register{NR_TCR0}, NR_R0, 1));
      cg.a_label(current_asmdata.CurrAsmList, lfcmpfalse);

    end;
    unequaln:
    begin
      if left.location.size = OS_F64 then
        op := A_C_EQ_D
      else
        op := A_C_EQ_S;
      current_asmdata.getjumplabel(lfcmpfalse);
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ORI, location.Register{NR_TCR0}, NR_R0, 1));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, left.location.Register, right.location.Register));
      current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_BC1F, lfcmpfalse));
      current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_OR, location.Register {NR_TCR0}, NR_R0, NR_R0));
      cg.a_label(current_asmdata.CurrAsmList, lfcmpfalse);
    end;
    ltn:
    begin
      if left.location.size = OS_F64 then
        op := A_C_LT_D
      else
        op := A_C_LT_S;
      current_asmdata.getjumplabel(lfcmptrue);
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ORI, location.Register{NR_TCR0}, NR_R0, 1));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, left.location.Register, right.location.Register));
      current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_BC1T, lfcmptrue));
      current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_OR, location.Register {NR_TCR0}, NR_R0, NR_R0));
      cg.a_label(current_asmdata.CurrAsmList, lfcmptrue);
    end;
    lten:
    begin
      if left.location.size = OS_F64 then
        op := A_C_LE_D
      else
        op := A_C_LE_S;
      current_asmdata.getjumplabel(lfcmptrue);
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ORI, location.Register{NR_TCR0}, NR_R0, 1));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, left.location.Register, right.location.Register));
      current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_BC1T, lfcmptrue));
      current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_OR, location.Register {NR_TCR0}, NR_R0, NR_R0));
      cg.a_label(current_asmdata.CurrAsmList, lfcmptrue);
    end;
    gtn:
    begin
      if left.location.size = OS_F64 then
        op := A_C_LT_D
      else
        op := A_C_LT_S;
      current_asmdata.getjumplabel(lfcmptrue);
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ORI, location.Register{NR_TCR0}, NR_R0, 1));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, right.location.Register, left.location.Register));
      current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_BC1T, lfcmptrue));
      current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_OR, location.Register {NR_TCR0}, NR_R0, NR_R0));
      cg.a_label(current_asmdata.CurrAsmList, lfcmptrue);
    end;
    gten:
    begin
      if left.location.size = OS_F64 then
        op := A_C_LE_D
      else
        op := A_C_LE_S;
      current_asmdata.getjumplabel(lfcmptrue);
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ORI, location.Register{NR_TCR0}, NR_R0, 1));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, right.location.Register, left.location.Register));
      current_asmdata.CurrAsmList.concat(Taicpu.op_sym(A_BC1T, lfcmptrue));
      current_asmdata.CurrAsmList.concat(TAiCpu.Op_none(A_NOP));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_OR, location.Register {NR_TCR0}, NR_R0, NR_R0));
      cg.a_label(current_asmdata.CurrAsmList, lfcmptrue);
    end;
  end; {case}
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


procedure tmipsaddnode.second_cmpboolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  second_generic_cmp32(true);
end;


procedure tmipsaddnode.second_cmpsmallset;
begin
  second_generic_cmp32(true);
end;


procedure tmipsaddnode.second_cmpordinal;
var
  unsigned: boolean;
begin
  unsigned := not (is_signed(left.resultdef)) or not (is_signed(right.resultdef));
  second_generic_cmp32(unsigned);
end;


const
  multops: array[boolean] of TAsmOp = (A_MULT, A_MULTU);

procedure tmipsaddnode.second_addordinal;
var
  unsigned: boolean;
begin
  unsigned:=not(is_signed(left.resultdef)) or
            not(is_signed(right.resultdef));
  if (nodetype=muln) and is_64bit(resultdef) then
    begin
      pass_left_right;
      force_reg_left_right(true,false);
      location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
      location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
      location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
      current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg(multops[unsigned],left.location.register,right.location.register));
      current_asmdata.CurrAsmList.Concat(taicpu.op_reg(A_MFLO,location.register64.reglo));
      current_asmdata.CurrAsmList.Concat(taicpu.op_reg(A_MFHI,location.register64.reghi));
    end
  else
    inherited second_addordinal;
end;

procedure tmipsaddnode.second_mul64bit;
var
  list: TAsmList;
  hreg1,hreg2,tmpreg: TRegister;
begin
  list:=current_asmdata.CurrAsmList;
  pass_left_right;
  location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
  hlcg.location_force_reg(list,left.location,left.resultdef,left.resultdef,true);
  { calculate 32-bit terms lo(right)*hi(left) and hi(left)*lo(right) }
  hreg1:=NR_NO;
  hreg2:=NR_NO;
  tmpreg:=NR_NO;
  if (right.location.loc=LOC_CONSTANT) then
    begin
      { Omit zero terms, if any }
      if hi(right.location.value64)<>0 then
        begin
          hreg2:=cg.getintregister(list,OS_INT);
          tmpreg:=cg.getintregister(list,OS_INT);
          cg.a_load_const_reg(list,OS_INT,longint(hi(right.location.value64)),tmpreg);
          list.concat(taicpu.op_reg_reg_reg(A_MUL,hreg2,tmpreg,left.location.register64.reglo));
        end;
      tmpreg:=NR_NO;
      if lo(right.location.value64)<>0 then
        begin
          hreg1:=cg.getintregister(list,OS_INT);
          tmpreg:=cg.getintregister(list,OS_INT);
          cg.a_load_const_reg(list,OS_INT,longint(lo(right.location.value64)),tmpreg);
          list.concat(taicpu.op_reg_reg_reg(A_MUL,hreg1,tmpreg,left.location.register64.reghi));
        end;
    end
  else
    begin
      hlcg.location_force_reg(list,right.location,right.resultdef,right.resultdef,true);
      tmpreg:=right.location.register64.reglo;
      hreg1:=cg.getintregister(list,OS_INT);
      hreg2:=cg.getintregister(list,OS_INT);
      list.concat(taicpu.op_reg_reg_reg(A_MUL,hreg1,right.location.register64.reglo,left.location.register64.reghi));
      list.concat(taicpu.op_reg_reg_reg(A_MUL,hreg2,right.location.register64.reghi,left.location.register64.reglo));
    end;

  { At this point, tmpreg is either lo(right) or NR_NO if lo(left)*lo(right) is zero }
  if (tmpreg=NR_NO) then
    begin
      if (hreg2<>NR_NO) and (hreg1<>NR_NO) then
        begin
          location.register64.reghi:=cg.getintregister(list,OS_INT);
          list.concat(taicpu.op_reg_reg_reg(A_ADDU,location.register64.reghi,hreg1,hreg2));
        end
      else if (hreg2<>NR_NO) then
        location.register64.reghi:=hreg2
      else if (hreg1<>NR_NO) then
        location.register64.reghi:=hreg1
      else
        InternalError(2014122701);
      location.register64.reglo:=NR_R0;
    end
  else
    begin
      list.concat(taicpu.op_reg_reg(A_MULTU,left.location.register64.reglo,tmpreg));
      location.register64.reghi:=cg.getintregister(list,OS_INT);
      location.register64.reglo:=cg.getintregister(list,OS_INT);
      current_asmdata.CurrAsmList.Concat(taicpu.op_reg(A_MFLO,location.register64.reglo));
      current_asmdata.CurrAsmList.Concat(taicpu.op_reg(A_MFHI,location.register64.reghi));
      if (hreg2<>NR_NO) then
        list.concat(taicpu.op_reg_reg_reg(A_ADDU,location.register64.reghi,location.register64.reghi,hreg2));
      if (hreg1<>NR_NO) then
        list.concat(taicpu.op_reg_reg_reg(A_ADDU,location.register64.reghi,location.register64.reghi,hreg1));
    end;
end;

function tmipsaddnode.use_generic_mul32to64: boolean;
begin
  result:=false;
end;

function tmipsaddnode.use_generic_mul64bit: boolean;
begin
  result:=(cs_check_overflow in current_settings.localswitches) or
    (not (CPUMIPS_HAS_ISA32R2 in cpu_capabilities[current_settings.cputype]));
end;


=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  tmp_right_reg: TRegister;
begin
  pass_left_right;
  force_reg_left_right(True, True);
  tmp_right_reg := NR_NO;
  if right.location.loc = LOC_CONSTANT then
  begin
    tmp_right_reg := cg.GetIntRegister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, tmp_right_reg, right.location.Value));
  end
  else
  begin
    tmp_right_reg := right.location.Register;
  end;

  location_reset(location, LOC_REGISTER, OS_INT);
  location.Register := GetRes_register(True, NR_TCR0, left.location.Register, tmp_right_reg);

end;


procedure tmipsaddnode.second_cmpsmallset;
var
  tmp_right_reg: TRegister;
begin
  pass_left_right;
  force_reg_left_right(True, True);

  tmp_right_reg := NR_NO;

  if right.location.loc = LOC_CONSTANT then
  begin
    tmp_right_reg := cg.GetIntRegister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, tmp_right_reg, right.location.Value));
  end
  else
  begin
    tmp_right_reg := right.location.Register;
  end;


  location_reset(location, LOC_REGISTER, OS_INT);
  location.Register := GetRes_register(True, NR_TCR0, left.location.Register, tmp_right_reg);
end;


procedure tmipsaddnode.second_cmp64bit;
var
         unsigned   : boolean;
  tmp_left_reg: TRegister;

begin
  pass_left_right;
  force_reg_left_right(false,false);

  unsigned:=not(is_signed(left.resultdef)) or
            not(is_signed(right.resultdef));

  location_reset(location, LOC_REGISTER, OS_INT);
  location.Register := GetRes64_register(unsigned, {NR_TCR0, }left.location.register64, right.location.register64); // NR_TCR0;
end;


procedure tmipsaddnode.second_cmpordinal;
var
  unsigned: boolean;
  tmp_right_reg: TRegister;
begin
  pass_left_right;
  force_reg_left_right(True, True);
  unsigned := not (is_signed(left.resultdef)) or not (is_signed(right.resultdef));

  tmp_right_reg := NR_NO;
  if right.location.loc = LOC_CONSTANT then
  begin
    tmp_right_reg := cg.GetIntRegister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LI, tmp_right_reg, right.location.Value));
  end
  else
  begin
    tmp_right_reg := right.location.Register;
  end;
  location_reset(location, LOC_REGISTER, OS_INT);
  location.Register := getres_register(unsigned, NR_TCR0, left.location.Register, tmp_right_reg);
end;

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
begin
  caddnode := tmipsaddnode;
end.
