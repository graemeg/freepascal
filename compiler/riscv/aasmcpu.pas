{
    Copyright (c) 1999-2002 by Jonas Maebe and Thomas Schatzl

    Contains the assembler object for the Risc-V 32 and Risc-V 64

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
unit aasmcpu;

{$i fpcdefs.inc}

interface

uses
  globtype,verbose,
  aasmbase,aasmtai,aasmdata,aasmsym,
  cpubase,cgbase,cgutils;

    const
      { "mov reg,reg" source operand number }
      O_MOV_SOURCE = 1;
      { "mov reg,reg" source operand number }
      O_MOV_DEST = 0;


    type

      { taicpu }

      taicpu = class(tai_cpu_abstract_sym)
         memoryordering: TMemoryOrdering;
         constructor op_none(op : tasmop);

         constructor op_reg(op : tasmop;_op1 : tregister);
         constructor op_const(op : tasmop;_op1 : aint);
         constructor op_ref(op : tasmop;_op1 : treference);

         constructor op_reg_reg(op : tasmop;_op1,_op2 : tregister);
         constructor op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
         constructor op_reg_const(op:tasmop; _op1: tregister; _op2: aint);
         constructor op_const_reg(op:tasmop; _op1: aint; _op2: tregister);

         constructor op_const_const(op : tasmop;_op1,_op2 : aint);

         constructor op_reg_reg_const_const(op: tasmop; _op1, _op2: tregister; _op3, _op4: aint);

         constructor op_reg_reg_roundingmode(op : tasmop;_op1,_op2 : tregister; _op3: TRoundingMode);
         constructor op_reg_reg_reg(op : tasmop;_op1,_op2,_op3 : tregister);
         constructor op_reg_reg_const(op : tasmop;_op1,_op2 : tregister; _op3: aint);
         constructor op_reg_reg_sym_ofs(op : tasmop;_op1,_op2 : tregister; _op3: tasmsymbol;_op3ofs: aint);
         constructor op_reg_reg_ref(op : tasmop;_op1,_op2 : tregister; const _op3: treference);
         constructor op_const_reg_reg(op : tasmop;_op1 : aint;_op2, _op3 : tregister);
         constructor op_const_reg_const(op : tasmop;_op1 : aint;_op2 : tregister;_op3 : aint);
         constructor op_const_const_const(op : tasmop;_op1 : aint;_op2 : aint;_op3 : aint);

         constructor op_reg_reg_reg_reg(op : tasmop;_op1,_op2,_op3,_op4 : tregister);
         constructor op_reg_bool_reg_reg(op : tasmop;_op1: tregister;_op2:boolean;_op3,_op4:tregister);
         constructor op_reg_bool_reg_const(op : tasmop;_op1: tregister;_op2:boolean;_op3:tregister;_op4: aint);

         constructor op_reg_reg_reg_const(op : tasmop; _op1, _op2, _op3 : tregister; _op4 : aint);
         constructor op_reg_reg_reg_const_const(op : tasmop;_op1,_op2,_op3 : tregister;_op4,_op5 : aint);
         constructor op_reg_reg_const_const_const(op : tasmop;_op1,_op2 : tregister;_op3,_op4,_op5 : aint);


         { this is for Jmp instructions }
         constructor op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);
         constructor op_const_const_sym(op : tasmop;_op1,_op2 : aint;_op3: tasmsymbol);


         constructor op_sym(op : tasmop;_op1 : tasmsymbol);
         constructor op_sym_ofs(op : tasmop;_op1 : tasmsymbol;_op1ofs:aint);
         constructor op_reg_sym(op : tasmop;_op1 : tregister;_op2:tasmsymbol);
         constructor op_reg_sym_ofs(op : tasmop;_op1 : tregister;_op2:tasmsymbol;_op2ofs : aint);
         constructor op_sym_ofs_ref(op : tasmop;_op1 : tasmsymbol;_op1ofs:aint;const _op2 : treference);

         procedure loadroundingmode(opidx:aint;_roundmode:TRoundingMode);
         procedure loadfenceflags(opidx:aint;_flags:TFenceFlags);
         procedure loadbool(opidx:aint;_b:boolean);

         function is_same_reg_move(regtype: Tregistertype):boolean; override;

         { register spilling code }
         function spilling_get_operation_type(opnr: longint): topertype;override;
         function spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;override;
      end;

      tai_align = class(tai_align_abstract)
        { nothing to add }
      end;

    procedure InitAsm;
    procedure DoneAsm;


    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
    function spilling_create_store(r:tregister; const ref:treference):Taicpu;

implementation

uses cutils, cclasses;

{*****************************************************************************
                                 taicpu Constructors
*****************************************************************************}

    procedure taicpu.loadbool(opidx:aint;_b:boolean);
      begin
        if opidx>=ops then
         ops:=opidx+1;
        with oper[opidx]^ do
         begin
           if typ=top_ref then
            dispose(ref);
           b:=_b;
           typ:=top_bool;
         end;
      end;


    constructor taicpu.op_none(op : tasmop);
      begin
         inherited create(op);
      end;


    constructor taicpu.op_reg(op : tasmop;_op1 : tregister);
      begin
         inherited create(op);
         ops:=1;
         loadreg(0,_op1);
      end;


    constructor taicpu.op_ref(op : tasmop;_op1 : treference);
      begin
         inherited create(op);
         ops:=1;
         loadref(0,_op1);
      end;


    constructor taicpu.op_const(op : tasmop;_op1 : aint);
      begin
         inherited create(op);
         ops:=1;
         loadconst(0,_op1);
      end;


    constructor taicpu.op_reg_reg(op : tasmop;_op1,_op2 : tregister);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadreg(1,_op2);
      end;

    constructor taicpu.op_reg_const(op:tasmop; _op1: tregister; _op2: aint);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadconst(1,_op2);
      end;

     constructor taicpu.op_const_reg(op:tasmop; _op1: aint; _op2: tregister);
      begin
         inherited create(op);
         ops:=2;
         loadconst(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_const_const(op : tasmop;_op1,_op2 : aint);
      begin
         inherited create(op);
         ops:=2;
         loadconst(0,_op1);
         loadconst(1,_op2);
      end;


    constructor taicpu.op_reg_reg_const_const(op: tasmop; _op1, _op2: tregister; _op3, _op4: aint);
      begin
        inherited create(op);
        ops := 4;
        loadreg(0, _op1);
        loadreg(1, _op2);
        loadconst(2, _op3);
        loadconst(3, _op4);
      end;


    constructor taicpu.op_reg_reg_roundingmode(op: tasmop; _op1, _op2: tregister; _op3: TRoundingMode);
      begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadroundingmode(2,_op3);
      end;


    constructor taicpu.op_reg_reg_reg(op : tasmop;_op1,_op2,_op3 : tregister);
      begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;

     constructor taicpu.op_reg_reg_const(op : tasmop;_op1,_op2 : tregister; _op3: aint);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadconst(2,_op3);
      end;

     constructor taicpu.op_reg_reg_sym_ofs(op : tasmop;_op1,_op2 : tregister; _op3: tasmsymbol;_op3ofs: aint);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadsymbol(2,_op3,_op3ofs);
      end;

     constructor taicpu.op_reg_reg_ref(op : tasmop;_op1,_op2 : tregister; const _op3: treference);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadref(2,_op3);
      end;

    constructor taicpu.op_const_reg_reg(op : tasmop;_op1 : aint;_op2, _op3 : tregister);
      begin
         inherited create(op);
         ops:=3;
         loadconst(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;

     constructor taicpu.op_const_reg_const(op : tasmop;_op1 : aint;_op2 : tregister;_op3 : aint);
      begin
         inherited create(op);
         ops:=3;
         loadconst(0,_op1);
         loadreg(1,_op2);
         loadconst(2,_op3);
      end;


     constructor taicpu.op_const_const_const(op : tasmop;_op1 : aint;_op2 : aint;_op3 : aint);
      begin
         inherited create(op);
         ops:=3;
         loadconst(0,_op1);
         loadconst(1,_op2);
         loadconst(2,_op3);
      end;


     constructor taicpu.op_reg_reg_reg_reg(op : tasmop;_op1,_op2,_op3,_op4 : tregister);
      begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
         loadreg(3,_op4);
      end;

     constructor taicpu.op_reg_bool_reg_reg(op : tasmop;_op1: tregister;_op2:boolean;_op3,_op4:tregister);
      begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadbool(1,_op2);
         loadreg(2,_op3);
         loadreg(3,_op4);
      end;

     constructor taicpu.op_reg_bool_reg_const(op : tasmop;_op1: tregister;_op2:boolean;_op3:tregister;_op4: aint);
      begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadbool(0,_op2);
         loadreg(0,_op3);
         loadconst(0,cardinal(_op4));
      end;

     constructor taicpu.op_reg_reg_reg_const(op : tasmop; _op1, _op2, _op3 : tregister; _op4 : aint);
      begin
        inherited create(op);
	ops := 4;
	loadreg(0, _op1);
	loadreg(1, _op2);
	loadreg(2, _op3);
	loadconst(3, cardinal(_op4));
      end;

     constructor taicpu.op_reg_reg_reg_const_const(op : tasmop;_op1,_op2,_op3 : tregister;_op4,_op5 : aint);
      begin
         inherited create(op);
         ops:=5;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
         loadconst(3,cardinal(_op4));
         loadconst(4,cardinal(_op5));
      end;

     constructor taicpu.op_reg_reg_const_const_const(op : tasmop;_op1,_op2 : tregister;_op3,_op4,_op5 : aint);
      begin
         inherited create(op);
         ops:=5;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadconst(2,_op3);
         loadconst(3,_op4);
         loadconst(4,_op5);
      end;

    constructor taicpu.op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);
      begin
         inherited create(op);
         condition:=cond;
         ops:=1;
         loadsymbol(0,_op1,0);
      end;

     constructor taicpu.op_const_const_sym(op : tasmop;_op1,_op2 : aint; _op3: tasmsymbol);
      begin
         inherited create(op);
         ops:=3;
         loadconst(0,_op1);
         loadconst(1,_op2);
         loadsymbol(2,_op3,0);
      end;


    constructor taicpu.op_sym(op : tasmop;_op1 : tasmsymbol);
      begin
         inherited create(op);
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym_ofs(op : tasmop;_op1 : tasmsymbol;_op1ofs:aint);
      begin
         inherited create(op);
         ops:=1;
         loadsymbol(0,_op1,_op1ofs);
      end;

    constructor taicpu.op_reg_sym(op: tasmop; _op1: tregister; _op2: tasmsymbol);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadsymbol(1,_op2,0);
      end;


     constructor taicpu.op_reg_sym_ofs(op : tasmop;_op1 : tregister;_op2:tasmsymbol;_op2ofs : aint);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadsymbol(1,_op2,_op2ofs);
      end;


    constructor taicpu.op_sym_ofs_ref(op : tasmop;_op1 : tasmsymbol;_op1ofs:aint;const _op2 : treference);
      begin
         inherited create(op);
         ops:=2;
         loadsymbol(0,_op1,_op1ofs);
         loadref(1,_op2);
      end;


    procedure taicpu.loadroundingmode(opidx: aint; _roundmode: TRoundingMode);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_roundingmode then
             clearop(opidx);
           roundingmode:=_roundmode;
           typ:=top_roundingmode;
         end;
      end;


    procedure taicpu.loadfenceflags(opidx: aint; _flags: TFenceFlags);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_fenceflags then
             clearop(opidx);
           fenceflags:=_flags;
           typ:=top_fenceflags;
         end;
      end;


{ ****************************** newra stuff *************************** }

    function taicpu.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        case regtype of
          R_INTREGISTER:
            result:=
               (opcode=A_ADDI) and
               (oper[0]^.reg=oper[1]^.reg) and
               (oper[2]^.val=0);
          R_FPUREGISTER:
            result:=
               (opcode in [A_FSGNJ_S,A_FSGNJ_D]) and
               (oper[0]^.reg=oper[1]^.reg) and
               (oper[0]^.reg=oper[2]^.reg);
         else
           result:=false;
        end;
      end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        result := operand_read;
        case opcode of
          // U type
          A_LUI,
          A_AUIPC:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;

          // UJ type
          A_JAL:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;

          // I type
          A_JALR,
          A_LB,A_LH,A_LW,A_LBU,A_LHU,
          A_ADDI,A_SLTI,A_SLTIU,
          A_XORI,A_ORI,A_ANDI,
          A_SLLI,A_SRLI,A_SRAI,

          A_FENCE,A_FENCE_I,
          A_ECALL,A_EBREAK,
          A_CSRRW,A_CSRRS,A_CSRRC,A_CSRRWI,A_CSRRSI,A_CSRRCI,

          A_FRCSR,A_FRRM,A_FRFLAGS,A_FSCSR,A_FSRM,
          A_FSFLAGS,A_FSRMI,A_FSFLAGSI:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;

          A_FLW,
          A_FLD:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;

          // SB type
          A_Bxx:
            result:=operand_read;

          // S type
          A_SB,A_SH,A_SW,

          A_FSW,
          A_FSD:
            result:=operand_read;

          // R type
          A_ADD,A_SUB,A_SLL,A_SLT,A_SLTU,
          A_XOR,A_OR,A_AND,A_SRL,A_SRA,

          A_MUL,A_MULH,A_MULHSU,A_MULHU,
          A_DIV,A_DIVU,A_REM,A_REMU,

          A_LR_W,A_SC_W,A_AMOSWAP_W,A_AMOADD_W,A_AMOXOR_W,A_AMOAND_W,
          A_AMOOR_W,A_AMOMIN_W,A_AMOMAX_W,A_AMOMINU_W,A_AMOMAXU_W,

          A_FADD_S,A_FSUB_S,A_FMUL_S,A_FDIV_S,
          A_FSQRT_S,A_FSGNJ_S,A_FSGNJN_S,A_FSGNJX_S,
          A_FMIN_S,A_FMAX_S,
          A_FMV_X_S,A_FEQ_S,A_FLT_S,A_FLE_S,A_FCLASS_S,
          A_FCVT_W_S,A_FCVT_WU_S,A_FCVT_S_W,A_FCVT_S_WU,
          A_FMV_S_X,

          A_FADD_D,A_FSUB_D,A_FMUL_D,A_FDIV_D,
          A_FSQRT_D,A_FSGNJ_D,A_FSGNJN_D,A_FSGNJX_D,
          A_FMIN_D,A_FMAX_D,
          A_FEQ_D,A_FLT_D,A_FLE_D,A_FCLASS_D,
          A_FCVT_D_S,A_FCVT_S_D,
          A_FCVT_W_D,A_FCVT_WU_D,A_FCVT_D_W,A_FCVT_D_WU:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;

          // R4 type
          A_FMADD_S,A_FMSUB_S,A_FNMSUB_S,A_FNMADD_S,

          A_FMADD_D,A_FMSUB_D,A_FNMSUB_D,A_FNMADD_D:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;
{$ifdef RISCV64}
          A_LR_D,A_SC_D,A_AMOSWAP_D,A_AMOADD_D,A_AMOXOR_D,A_AMOAND_D,
          A_AMOOR_D,A_AMOMIN_D,A_AMOMAX_D,A_AMOMINU_D,A_AMOMAXU_D,

          A_MULW,
          A_DIVW,A_DIVUW,A_REMW,A_REMUW,

          A_FCVT_L_S,A_FCVT_LU_S,
          A_FCVT_S_L,A_FCVT_S_LU,

          A_FCVT_L_D,A_FCVT_LU_D,A_FMV_X_D,
          A_FCVT_D_L,A_FCVT_D_LU,A_FMV_D_X,

          A_ADDIW,A_SLLIW,A_SRLIW,A_SRAIW,
          A_ADDW,A_SLLW,A_SRLW,A_SUBW,A_SRAW,
          A_LD,A_LWU:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;

          A_SD:
            result:=operand_read;
{$endif RISCV64}
          else
            ;
        end;
      end;


    function taicpu.spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;
      begin
        result := operand_read;
      end;


    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER:
{$ifdef cpu64bitalu}
            result:=taicpu.op_reg_ref(A_LD,r,ref);
{$else cpu64bitalu}
            result:=taicpu.op_reg_ref(A_LW,r,ref);
{$endif cpu64bitalu}
          R_FPUREGISTER:
            result:=taicpu.op_reg_ref(A_FLD,r,ref);
          else
            internalerror(2005123101);
        end;
      end;


    function spilling_create_store(r:tregister; const ref:treference):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER:
{$ifdef cpu64bitalu}
            result:=taicpu.op_reg_ref(A_SD,r,ref);
{$else cpu64bitalu}
            result:=taicpu.op_reg_ref(A_SW,r,ref);
{$endif cpu64bitalu}
          R_FPUREGISTER:
            result:=taicpu.op_reg_ref(A_FSD,r,ref);
          else
            internalerror(2005123102);
        end;
      end;


    procedure InitAsm;
      begin
      end;


    procedure DoneAsm;
      begin
      end;


begin
  cai_align:=tai_align;
  cai_cpu:=taicpu;
end.

