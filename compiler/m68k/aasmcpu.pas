{
    Copyright (c) 1998-2001 by Florian Klaempfl and Pierre Muller

    m68k family assembler instructions

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
  cclasses,aasmtai,aasmdata,aasmsym,
  aasmbase,globals,verbose,symtype,
  cpubase,cpuinfo,cgbase,cgutils;


const
  { "mov reg,reg" source operand number }
  O_MOV_SOURCE = 0;
  { "mov reg,reg" source operand number }
  O_MOV_DEST = 1;
type

  taicpu = class(tai_cpu_abstract_sym)
     opsize : topsize;

     procedure loadregset(opidx:longint; const dataregs,addrregs,fpuregs:tcpuregisterset);

     constructor op_none(op : tasmop);
     constructor op_none(op : tasmop;_size : topsize);

     constructor op_reg(op : tasmop;_size : topsize;_op1 : tregister);
     constructor op_const(op : tasmop;_size : topsize;_op1 : longint);
     constructor op_ref(op : tasmop;_size : topsize;_op1 : treference);

     constructor op_reg_reg(op : tasmop;_size : topsize;_op1,_op2 : tregister);
     constructor op_reg_ref(op : tasmop;_size : topsize;_op1 : tregister;_op2 : treference);
     constructor op_reg_const(op:tasmop; _size: topsize; _op1: tregister; _op2: longint);

     constructor op_const_reg(op : tasmop;_size : topsize;_op1 : longint;_op2 : tregister);
     constructor op_const_const(op : tasmop;_size : topsize;_op1,_op2 : longint);
     constructor op_const_ref(op : tasmop;_size : topsize;_op1 : longint;_op2 : treference);

     constructor op_ref_reg(op : tasmop;_size : topsize;_op1 : treference;_op2 : tregister);
     { this is only allowed if _op1 is an int value (_op1^.isintvalue=true) }
     constructor op_ref_ref(op : tasmop;_size : topsize;_op1,_op2 : treference);

     constructor op_reg_reg_reg(op : tasmop;_size : topsize;_op1,_op2,_op3 : tregister);
     constructor op_const_reg_reg(op : tasmop;_size : topsize;_op1 : longint;_op2 : tregister;_op3 : tregister);
     constructor op_const_ref_reg(op : tasmop;_size : topsize;_op1 : longint;_op2 : treference;_op3 : tregister);
     constructor op_reg_reg_ref(op : tasmop;_size : topsize;_op1,_op2 : tregister; _op3 : treference);
     constructor op_const_reg_ref(op : tasmop;_size : topsize;_op1 : longint;_op2 : tregister;_op3 : treference);

     constructor op_reg_regset(op: tasmop; _size : topsize; _op1: tregister;const _op2data,_op2addr,_op2fpu: tcpuregisterset);
     constructor op_regset_reg(op: tasmop; _size : topsize;const _op1data,_op1addr,_op1fpu: tcpuregisterset; _op2: tregister);

     constructor op_ref_regset(op: tasmop; _size : topsize; _op1: treference;const _op2data,_op2addr,_op2fpu: tcpuregisterset);
     constructor op_regset_ref(op: tasmop; _size : topsize;const _op1data,_op1addr,_op1fpu: tcpuregisterset; _op2: treference);

     { this is for Jmp instructions }
     constructor op_cond_sym(op : tasmop;cond:TAsmCond;_size : topsize;_op1 : tasmsymbol);

     constructor op_sym(op : tasmop;_size : topsize;_op1 : tasmsymbol);
     { for DBxx opcodes }
     constructor op_reg_sym(op: tasmop; _size : topsize; _op1: tregister; _op2 :tasmsymbol);
     constructor op_sym_ofs_reg(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;_op2 : tregister);

     constructor op_sym_ofs(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint);
     constructor op_sym_ofs_ref(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;const _op2 : treference);

     function is_same_reg_move(regtype: Tregistertype):boolean;override;
     function spilling_get_operation_type(opnr: longint): topertype;override;
     function spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;override;

  private
     procedure init(_size : topsize); { this need to be called by all constructor }
  end;


  tai_align = class(tai_align_abstract)
        { nothing to add }
  end;

  procedure InitAsm;
  procedure DoneAsm;

    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
    function spilling_create_store(r:tregister; const ref:treference):Taicpu;

  implementation

    uses
      globtype;


{*****************************************************************************
                                 Taicpu Constructors
*****************************************************************************}



    procedure taicpu.loadregset(opidx:longint; const dataregs,addrregs,fpuregs:tcpuregisterset);
      var
        i : byte;
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_regset then
             clearop(opidx);
           new(dataregset);
           new(addrregset);
           new(fpuregset);
           dataregset^:=dataregs;
           addrregset^:=addrregs;
           fpuregset^:=fpuregs;
           typ:=top_regset;
           for i:=RS_D0 to RS_D7 do
             begin
               if assigned(add_reg_instruction_hook) and (i in dataregset^) then
                 add_reg_instruction_hook(self,newreg(R_INTREGISTER,i,R_SUBWHOLE));
             end;
           for i:=RS_A0 to RS_SP do
             begin
               if assigned(add_reg_instruction_hook) and (i in addrregset^) then
                 add_reg_instruction_hook(self,newreg(R_ADDRESSREGISTER,i,R_SUBWHOLE));
             end;
           for i:=RS_FP0 to RS_FP7 do
             begin
               if assigned(add_reg_instruction_hook) and (i in fpuregset^) then
                 add_reg_instruction_hook(self,newreg(R_FPUREGISTER,i,R_SUBWHOLE));
             end;
         end;
      end;


    procedure taicpu.init(_size : topsize);
      begin
         typ:=ait_instruction;
         is_jmp:=false;
         opsize:=_size;
         ops:=0;
      end;


    constructor taicpu.op_none(op : tasmop);
      begin
         inherited create(op);
         init(S_NO);
      end;


    constructor taicpu.op_none(op : tasmop;_size : topsize);
      begin
         inherited create(op);
         init(_size);
      end;


    constructor taicpu.op_reg(op : tasmop;_size : topsize;_op1 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadreg(0,_op1);
      end;


    constructor taicpu.op_const(op : tasmop;_size : topsize;_op1 : longint);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadconst(0,aword(_op1));
      end;


    constructor taicpu.op_ref(op : tasmop;_size : topsize;_op1 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadref(0,_op1);
      end;


    constructor taicpu.op_reg_reg(op : tasmop;_size : topsize;_op1,_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_reg_const(op:tasmop; _size: topsize; _op1: tregister; _op2: longint);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadconst(1,aword(_op2));
      end;


    constructor taicpu.op_reg_ref(op : tasmop;_size : topsize;_op1 : tregister;_op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_const_reg(op : tasmop;_size : topsize;_op1 : longint;_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadconst(0,aword(_op1));
         loadreg(1,_op2);
      end;


    constructor taicpu.op_const_const(op : tasmop;_size : topsize;_op1,_op2 : longint);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadconst(0,aword(_op1));
         loadconst(1,aword(_op2));
      end;


    constructor taicpu.op_const_ref(op : tasmop;_size : topsize;_op1 : longint;_op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadconst(0,aword(_op1));
         loadref(1,_op2);
      end;


    constructor taicpu.op_ref_reg(op : tasmop;_size : topsize;_op1 : treference;_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadref(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_ref_ref(op : tasmop;_size : topsize;_op1,_op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadref(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_reg_reg_reg(op : tasmop;_size : topsize;_op1,_op2,_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;

    constructor taicpu.op_const_reg_reg(op : tasmop;_size : topsize;_op1 : longint;_op2 : tregister;_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadconst(0,aword(_op1));
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;

    constructor taicpu.op_reg_reg_ref(op : tasmop;_size : topsize;_op1,_op2 : tregister;_op3 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadref(2,_op3);
      end;


    constructor taicpu.op_const_ref_reg(op : tasmop;_size : topsize;_op1 : longint;_op2 : treference;_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadconst(0,aword(_op1));
         loadref(1,_op2);
         loadreg(2,_op3);
      end;


    constructor taicpu.op_const_reg_ref(op : tasmop;_size : topsize;_op1 : longint;_op2 : tregister;_op3 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadconst(0,aword(_op1));
         loadreg(1,_op2);
         loadref(2,_op3);
      end;


   constructor taicpu.op_ref_regset(op: tasmop; _size : topsize; _op1: treference;const _op2data,_op2addr,_op2fpu: tcpuregisterset);
     Begin
        inherited create(op);
        init(_size);
        ops:=2;
        loadref(0,_op1);
        loadregset(1,_op2data,_op2addr,_op2fpu);
     end;


   constructor taicpu.op_regset_ref(op: tasmop; _size : topsize;const _op1data,_op1addr,_op1fpu: tcpuregisterset; _op2: treference);
     Begin
        inherited create(op);
        init(_size);
        ops:=2;
        loadregset(0,_op1data,_op1addr,_op1fpu);
        loadref(1,_op2);
     End;



   constructor taicpu.op_reg_regset(op: tasmop; _size : topsize; _op1: tregister;const _op2data,_op2addr,_op2fpu: tcpuregisterset);
     Begin
        inherited create(op);
        init(_size);
        ops:=2;
        loadreg(0,_op1);
        loadregset(1,_op2data,_op2addr,_op2fpu);
     end;


   constructor taicpu.op_regset_reg(op: tasmop; _size : topsize;const _op1data,_op1addr,_op1fpu: tcpuregisterset; _op2: tregister);
     Begin
        inherited create(op);
        init(_size);
        ops:=2;
        loadregset(0,_op1data,_op1addr,_op1fpu);
        loadreg(1,_op2);
     End;


    constructor taicpu.op_sym(op : tasmop;_size : topsize;_op1 : tasmsymbol);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


     constructor taicpu.op_reg_sym(op: tasmop; _size : topsize; _op1: tregister; _op2 :tasmsymbol);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadsymbol(1,_op2,0);
      end;


    constructor taicpu.op_sym_ofs_ref(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;const _op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadsymbol(0,_op1,_op1ofs);
         loadref(1,_op2);
      end;


    constructor taicpu.op_sym_ofs(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadsymbol(0,_op1,_op1ofs);
      end;

    constructor taicpu.op_sym_ofs_reg(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         if ((op >= A_DBCC) and (op <= A_DBF))
          or ((op >= A_FDBEQ) and (op <= A_FDBNGLE)) then
           begin
             loadreg(0,_op2);
             loadsymbol(1,_op1,_op1ofs);
           end
          else
           begin
             loadsymbol(0,_op1,_op1ofs);
             loadreg(1,_op2);
           end;
      end;


    constructor taicpu.op_cond_sym(op : tasmop;cond:TAsmCond;_size : topsize;_op1 : tasmsymbol);
      begin
         inherited create(op);
         init(_size);
         condition:=cond;
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    function taicpu.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        result:=(((opcode=A_MOVE) or (opcode=A_EXG)) and
                 (regtype = R_INTREGISTER) and
                 (ops=2) and
                 (oper[0]^.typ=top_reg) and
                 (oper[1]^.typ=top_reg) and
                 (isregoverlap(oper[0]^.reg,oper[1]^.reg))
                ) or
                (((opcode=A_MOVE) or (opcode=A_EXG) or (opcode=A_MOVEA)) and
                 (regtype = R_ADDRESSREGISTER) and
                 (ops=2) and
                 (oper[0]^.typ=top_reg) and
                 (oper[1]^.typ=top_reg) and
                 (isregoverlap(oper[0]^.reg,oper[1]^.reg))
                ) or
                ((opcode=A_FMOVE) and
                 (regtype = R_FPUREGISTER) and
                 (ops=2) and
                 (oper[0]^.typ=top_reg) and
                 (oper[1]^.typ=top_reg) and
                 (oper[0]^.reg=oper[1]^.reg)
                );
      end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        result:=operand_read;

        case opcode of
<<<<<<< HEAD
          // CPU opcodes
          A_MOVE, A_MOVEQ, A_MOVEA, A_MVZ, A_MVS, A_MOV3Q, A_LEA,
          A_BSET, A_BCLR:
            if opnr=1 then
              result:=operand_write;
          A_ADD, A_ADDQ, A_ADDX, A_SUB, A_SUBQ, A_SUBX,
          A_AND, A_LSR, A_LSL, A_ASR, A_ASL, A_EOR, A_EORI, A_OR,
          A_ROL, A_ROR, A_ROXL, A_ROXR,
          A_MULS, A_MULU, A_DIVS, A_DIVU, A_DIVSL, A_DIVUL:
            if opnr=1 then
              result:=operand_readwrite;
          A_DBRA:
            if opnr=0 then
              result:=operand_readwrite;
          A_CLR, A_SXX, A_SEQ, A_SNE, A_SLT, A_SLE, A_SGT, A_SGE, A_SCS, A_SCC,
          A_SMI, A_SPL, A_SF, A_ST, A_SVS, A_SVC, A_SHI, A_SLS:
            result:=operand_write;
          A_NEG, A_NEGX, A_EXT, A_EXTB, A_NOT, A_SWAP:
            result:=operand_readwrite;
          A_TST,A_CMP,A_CMPI,A_BTST:
            begin end; { Do nothing, default operand_read is fine here. }

          // FPU opcodes
          A_FSXX, A_FSEQ, A_FSNE, A_FSLT, A_FSLE, A_FSGT, A_FSGE:
             result:=operand_write;
          A_FABS,A_FSQRT,A_FNEG,A_FSIN,A_FCOS:
             if ops = 1 then
               begin
                 if opnr = 0 then
                   result:=operand_readwrite;
               end
             else
               if opnr = 1 then
                 result:=operand_write;
          A_FMOVE:
             if opnr=1 then
               result:=operand_write;
          A_FADD, A_FSUB, A_FMUL, A_FDIV:
             if opnr=1 then
               result:=operand_readwrite;
          A_FCMP, A_FTST:
             begin end; { operand_read }

=======
          A_MOVE, A_MOVEQ, A_ADD, A_ADDQ, A_ADDX, A_SUB, A_SUBQ,
          A_AND, A_LSR, A_LSL, A_ASR, A_ASL, A_EOR, A_EORI, A_OR:
            if opnr=1 then begin
              result:=operand_write;
            end else begin
              result:=operand_read;
            end;
          A_TST,A_CMP,A_CMPI:
            result:=operand_read;
          A_CLR, A_SXX:
            result:=operand_write;
          A_NEG, A_EXT, A_EXTB, A_NOT:
            result:=operand_readwrite;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          else begin
            internalerror(2004040903);
          end;
        end;
      end;

    function taicpu.spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;
      begin
        result := operand_read;
        if (oper[opnr]^.ref^.base = reg) and
          (oper[opnr]^.ref^.direction <> dir_none) then
           result := operand_readwrite;
      end;

    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER :
            result:=taicpu.op_ref_reg(A_MOVE,S_L,ref,r);
          R_ADDRESSREGISTER :
            result:=taicpu.op_ref_reg(A_MOVE,S_L,ref,r);
          R_FPUREGISTER :
            // no need to handle sizes here
            result:=taicpu.op_ref_reg(A_FMOVE,S_FS,ref,r);
          else
            internalerror(200602011);
        end;
      end;


    function spilling_create_store(r:tregister; const ref:treference):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER :
            result:=taicpu.op_reg_ref(A_MOVE,S_L,r,ref);
          R_ADDRESSREGISTER :
            result:=taicpu.op_reg_ref(A_MOVE,S_L,r,ref);
          R_FPUREGISTER :
            // no need to handle sizes here
            result:=taicpu.op_reg_ref(A_FMOVE,S_FS,r,ref);
          else
            internalerror(200602012);
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
