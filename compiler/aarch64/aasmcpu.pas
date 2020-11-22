{
    Copyright (c) 2003-2012 by Florian Klaempfl and others

    Contains the assembler object for Aarch64

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
  cclasses,globtype,globals,verbose,
  aasmbase,aasmtai,aasmdata,aasmsym,
  ogbase,
  symtype,
  cpubase,cpuinfo,cgbase,cgutils;

    const
      { "mov reg,reg" source operand number }
      O_MOV_SOURCE = 1;
      { "mov reg,reg" source operand number }
      O_MOV_DEST = 0;

      { Operand types }
      OT_NONE      = $00000000;

      OT_BITS8     = $00000001;  { size, and other attributes, of the operand  }
      OT_BITS16    = $00000002;
      OT_BITS32    = $00000004;
      OT_BITS64    = $00000008;  { FPU only  }
      OT_BITS80    = $00000010;
      OT_FAR       = $00000020;  { this means 16:16 or 16:32, like in CALL/JMP }
      OT_NEAR      = $00000040;
      OT_SHORT     = $00000080;
      OT_BITSTINY  = $00000100;  { fpu constant }
      OT_BITSSHIFTER =
                     $00000200;

      OT_SIZE_MASK = $000003FF;  { all the size attributes  }
      OT_NON_SIZE  = longint(not OT_SIZE_MASK);

      OT_SIGNED    = $00000100;  { the operand need to be signed -128-127 }

      OT_TO        = $00000200;  { operand is followed by a colon  }
                                 { reverse effect in FADD, FSUB &c  }
      OT_COLON     = $00000400;

      OT_SHIFTEROP = $00000800;
      OT_REGISTER  = $00001000;
      OT_IMMEDIATE = $00002000;
      OT_REGLIST   = $00008000;
      OT_IMM8      = $00002001;
      OT_IMM24     = $00002002;
      OT_IMM32     = $00002004;
      OT_IMM64     = $00002008;
      OT_IMM80     = $00002010;
      OT_IMMTINY   = $00002100;
      OT_IMMSHIFTER= $00002200;
      OT_IMMEDIATE24 = OT_IMM24;
      OT_SHIFTIMM  = OT_SHIFTEROP or OT_IMMSHIFTER;
      OT_SHIFTIMMEDIATE = OT_SHIFTIMM;
      OT_IMMEDIATESHIFTER = OT_IMMSHIFTER;

      OT_IMMEDIATEFPU = OT_IMMTINY;

      OT_REGMEM    = $00200000;  { for r/m, ie EA, operands  }
      OT_REGNORM   = $00201000;  { 'normal' reg, qualifies as EA  }
      OT_REG8      = $00201001;
      OT_REG16     = $00201002;
      OT_REG32     = $00201004;
      OT_REG64     = $00201008;
      OT_VREG      = $00201010;  { vector register }
      OT_REGF      = $00201020;  { coproc register }
      OT_MEMORY    = $00204000;  { register number in 'basereg'  }
      OT_MEM8      = $00204001;
      OT_MEM16     = $00204002;
      OT_MEM32     = $00204004;
      OT_MEM64     = $00204008;
      OT_MEM80     = $00204010;
      { word/byte load/store }
      OT_AM2       = $00010000;
      { misc ld/st operations }
      OT_AM3       = $00020000;
      { multiple ld/st operations }
      OT_AM4       = $00040000;
      { co proc. ld/st operations }
      OT_AM5       = $00080000;
      OT_AMMASK    = $000f0000;
      { IT instruction }
      OT_CONDITION = $00100000;

      OT_MEMORYAM2 = OT_MEMORY or OT_AM2;
      OT_MEMORYAM3 = OT_MEMORY or OT_AM3;
      OT_MEMORYAM4 = OT_MEMORY or OT_AM4;
      OT_MEMORYAM5 = OT_MEMORY or OT_AM5;

      OT_FPUREG    = $01000000;  { floating point stack registers  }
      OT_REG_SMASK = $00070000;  { special register operands: these may be treated differently  }
                                 { a mask for the following  }

      OT_MEM_OFFS  = $00604000;  { special type of EA  }
                                 { simple [address] offset  }
      OT_ONENESS   = $00800000;  { special type of immediate operand  }
                                 { so UNITY == IMMEDIATE | ONENESS  }
      OT_UNITY     = $00802000;  { for shift/rotate instructions  }

      instabentries = {$i a64nop.inc}

      maxinfolen = 5;

      IF_NONE   = $00000000;

      IF_ARMMASK    = $000F0000;
      IF_ARM7       = $00070000;
      IF_FPMASK     = $00F00000;
      IF_FPA        = $00100000;

      { if the instruction can change in a second pass }
      IF_PASS2  = longint($80000000);

    type
      TInsTabCache=array[TasmOp] of longint;
      PInsTabCache=^TInsTabCache;

      tinsentry = record
        opcode  : tasmop;
        ops     : byte;
        optypes : array[0..3] of longint;
        code    : array[0..maxinfolen] of char;
        flags   : longint;
      end;

      pinsentry=^tinsentry;

    var
      InsTabCache : PInsTabCache;

    type
      taicpu = class(tai_cpu_abstract_sym)
         oppostfix : TOpPostfix;
         procedure loadshifterop(opidx:longint;const so:tshifterop);
         procedure loadconditioncode(opidx: longint; const c: tasmcond);
         procedure loadrealconst(opidx: longint; const _value: bestreal);
         procedure loadregset(opidx: longint; _basereg: tregister; _nregs: byte; _regsetindex: byte = 255);
         procedure loadindexedreg(opidx: longint; _indexedreg: tregister; _regindex: byte);

         constructor op_none(op : tasmop);

         constructor op_reg(op : tasmop;_op1 : tregister);
         constructor op_ref(op : tasmop;const _op1 : treference);
         constructor op_const(op : tasmop;_op1 : longint);

         constructor op_reg_reg(op : tasmop;_op1,_op2 : tregister);
         constructor op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
         constructor op_reg_cond(op: tasmop; _op1: tregister; _op2: tasmcond);
         constructor op_reg_const(op:tasmop; _op1: tregister; _op2: aint);
         constructor op_reg_const_shifterop(op : tasmop;_op1: tregister; _op2: aint;_op3 : tshifterop);
         constructor op_reg_realconst(op: tasmop; _op1: tregister; _op2: bestreal);

         constructor op_indexedreg_reg(op : tasmop;_op1: tregister; _op1index: byte; _op2 : tregister);
         constructor op_reg_indexedreg(op : tasmop;_op1: tregister; _op2 : tregister; _op2index: byte);

         constructor op_reg_reg_reg(op : tasmop;_op1,_op2,_op3 : tregister);
         constructor op_reg_reg_reg_reg(op : tasmop;_op1,_op2,_op3,_op4 : tregister);
         constructor op_reg_reg_const(op : tasmop;_op1,_op2 : tregister; _op3: aint);
         constructor op_reg_reg_const_const(op : tasmop;_op1,_op2 : tregister; _op3, _op4: aint);
         constructor op_reg_reg_const_shifterop(op : tasmop;_op1,_op2 : tregister; _op3: aint; const _op4 : tshifterop);
         constructor op_reg_reg_sym_ofs(op : tasmop;_op1,_op2 : tregister; _op3: tasmsymbol;_op3ofs: longint);
         constructor op_reg_reg_ref(op : tasmop;_op1,_op2 : tregister; const _op3: treference);
         constructor op_reg_reg_shifterop(op : tasmop;_op1,_op2 : tregister;_op3 : tshifterop);
         constructor op_reg_reg_reg_shifterop(op : tasmop;_op1,_op2,_op3 : tregister; const _op4 : tshifterop);
         constructor op_reg_reg_reg_cond(op : tasmop;_op1,_op2,_op3 : tregister; const _op4: tasmcond);

         constructor op_const_ref(op:tasmop; _op1: aint; _op2: treference);

         { this is for Jmp instructions }
         constructor op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);

         { ldN(r)/stN }
         constructor op_regset_reg_ref(op: tasmop; basereg: tregister; nregs: byte; const ref: treference);

         constructor op_sym(op : tasmop;_op1 : tasmsymbol);
         constructor op_sym_ofs(op : tasmop;_op1 : tasmsymbol;_op1ofs:longint);
         constructor op_reg_sym_ofs(op : tasmop;_op1 : tregister;_op2:tasmsymbol;_op2ofs : longint);
         constructor op_sym_ofs_ref(op : tasmop;_op1 : tasmsymbol;_op1ofs:longint;const _op2 : treference);

         function is_same_reg_move(regtype: Tregistertype):boolean; override;

         function spilling_get_operation_type(opnr: longint): topertype;override;
         function spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;override;

         { assembler }
      public
         { the next will reset all instructions that can change in pass 2 }
         procedure ResetPass1;override;
         procedure ResetPass2;override;
         function  CheckIfValid:boolean;
         function GetString:string;
         function  Pass1(objdata:TObjData):longint;override;
         procedure Pass2(objdata:TObjData);override;
      protected
         procedure ppuloadoper(ppufile:tcompilerppufile;var o:toper);override;
         procedure ppuwriteoper(ppufile:tcompilerppufile;const o:toper);override;
         procedure ppubuildderefimploper(var o:toper);override;
         procedure ppuderefoper(var o:toper);override;
      end;

      tai_align = class(tai_align_abstract)
        { nothing to add }
      end;

    type
      tsimplereftype =
         { valid reference }
        (sr_simple,
         { invalid reference, should not be generated by the code generator (but
           can be encountered via inline assembly, where it must be rejected) }
         sr_internal_illegal,
         { invalid reference, may be generated by the code generator and then
           must be simplified (also rejected in inline assembly) }
         sr_complex);

    function simple_ref_type(op: tasmop; size:tcgsize; oppostfix: toppostfix; const ref: treference): tsimplereftype;
    function can_be_shifter_operand(opc: tasmop; opnr: longint): boolean;
    function valid_shifter_operand(opc: tasmop; useszr, usessp, is64bit: boolean; sm: tshiftmode; shiftimm: longint): boolean;

    function spilling_create_load(const ref: treference; r: tregister): taicpu;
    function spilling_create_store(r: tregister; const ref: treference): taicpu;

    function setoppostfix(i : taicpu;pf : toppostfix) : taicpu;
    function setcondition(i : taicpu;c : tasmcond) : taicpu;

    { inserts pc relative symbols at places where they are reachable
      and transforms special instructions to valid instruction encodings }
    procedure finalizearmcode(list,listtoinsert : TAsmList);

    procedure InitAsm;
    procedure DoneAsm;


implementation

  uses
    cutils,rgobj,itcpugas,aoptcpu;


    procedure taicpu.loadshifterop(opidx:longint;const so:tshifterop);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
          begin
            if typ<>top_shifterop then
              begin
                clearop(opidx);
                new(shifterop);
              end;
            shifterop^:=so;
            typ:=top_shifterop;
          end;
      end;


    procedure taicpu.loadconditioncode(opidx: longint; const c: tasmcond);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
          begin
            if typ<>top_conditioncode then
              begin
                clearop(opidx);
              end;
            cc:=c;
            typ:=top_conditioncode;
          end;
      end;


    procedure taicpu.loadrealconst(opidx:longint;const _value:bestreal);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
          begin
            if typ<>top_realconst then
              clearop(opidx);
            val_real:=_value;
            typ:=top_realconst;
          end;
      end;


    procedure taicpu.loadregset(opidx: longint; _basereg: tregister; _nregs: byte; _regsetindex: byte = 255);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
          begin
            if typ<>top_regset then
              clearop(opidx);
            basereg:=_basereg;
            nregs:=_nregs;
            regsetindex:=_regsetindex;
            typ:=top_regset;
          end;
      end;


    procedure taicpu.loadindexedreg(opidx: longint; _indexedreg: tregister; _regindex: byte);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
          begin
            if typ<>top_indexedreg then
              clearop(opidx);
            indexedreg:=_indexedreg;
            regindex:=_regindex;
            typ:=top_indexedreg;
          end;
      end;


{*****************************************************************************
                                 taicpu Constructors
*****************************************************************************}

    constructor taicpu.op_none(op : tasmop);
      begin
         inherited create(op);
      end;


    { for pld }
    constructor taicpu.op_ref(op : tasmop;const _op1 : treference);
      begin
         inherited create(op);
         ops:=1;
         loadref(0,_op1);
      end;


    constructor taicpu.op_reg(op : tasmop;_op1 : tregister);
      begin
         inherited create(op);
         ops:=1;
         loadreg(0,_op1);
      end;


    constructor taicpu.op_const(op : tasmop;_op1 : longint);
      begin
         inherited create(op);
         ops:=1;
         loadconst(0,aint(_op1));
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
         loadconst(1,aint(_op2));
      end;


    constructor taicpu.op_reg_const_shifterop(op: tasmop; _op1: tregister; _op2: aint; _op3: tshifterop);
      begin
        inherited create(op);
        ops:=3;
        loadreg(0,_op1);
        loadconst(1,_op2);
        loadshifterop(2,_op3);
      end;


    constructor taicpu.op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_reg_cond(op: tasmop; _op1: tregister; _op2: tasmcond);
      begin
        inherited create(op);
        ops:=2;
        loadreg(0,_op1);
        loadconditioncode(1,_op2);
      end;


    constructor taicpu.op_reg_reg_reg(op : tasmop;_op1,_op2,_op3 : tregister);
      begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
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


    constructor taicpu.op_reg_realconst(op : tasmop; _op1 : tregister; _op2 : bestreal);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadrealconst(1,_op2);
      end;


    constructor taicpu.op_indexedreg_reg(op: tasmop; _op1: tregister; _op1index: byte; _op2: tregister);
      begin
        inherited create(op);
        ops:=2;
        loadindexedreg(0,_op1,_op1index);
        loadreg(1,_op2);
      end;


    constructor taicpu.op_reg_indexedreg(op: tasmop; _op1: tregister; _op2: tregister; _op2index: byte);
      begin
        inherited create(op);
        ops:=2;
        loadreg(0,_op1);
        loadindexedreg(1,_op2,_op2index);
      end;


     constructor taicpu.op_reg_reg_const(op : tasmop;_op1,_op2 : tregister; _op3: aint);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadconst(2,aint(_op3));
      end;


     constructor taicpu.op_reg_reg_const_const(op: tasmop; _op1, _op2: tregister; _op3, _op4: aint);
       begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadconst(2,aint(_op3));
         loadconst(3,aint(_op4));
       end;


     constructor taicpu.op_reg_reg_const_shifterop(op: tasmop; _op1, _op2: tregister; _op3: aint; const _op4: tshifterop);
       begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadconst(2,aint(_op3));
         loadshifterop(3,_op4);
       end;


     constructor taicpu.op_reg_reg_sym_ofs(op : tasmop;_op1,_op2 : tregister; _op3: tasmsymbol;_op3ofs: longint);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadsymbol(0,_op3,_op3ofs);
      end;


     constructor taicpu.op_reg_reg_ref(op : tasmop;_op1,_op2 : tregister; const _op3: treference);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadref(2,_op3);
      end;


     constructor taicpu.op_reg_reg_shifterop(op : tasmop;_op1,_op2 : tregister;_op3 : tshifterop);
      begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadshifterop(2,_op3);
      end;


     constructor taicpu.op_reg_reg_reg_shifterop(op : tasmop;_op1,_op2,_op3 : tregister; const _op4 : tshifterop);
      begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
         loadshifterop(3,_op4);
      end;

     constructor taicpu.op_reg_reg_reg_cond(op: tasmop; _op1, _op2, _op3: tregister; const _op4: tasmcond);
       begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
         loadconditioncode(3,_op4);
       end;


     constructor taicpu.op_const_ref(op : tasmop; _op1 : aint; _op2 : treference);
      begin
         inherited create(op);
         ops:=2;
         loadconst(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);
      begin
         inherited create(op);
         condition:=cond;
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_regset_reg_ref(op: tasmop; basereg: tregister; nregs: byte; const ref: treference);
      begin
        inherited create(op);
        ops:=2;
        loadregset(0,basereg,nregs);
        loadref(1, ref);
      end;


    constructor taicpu.op_sym(op : tasmop;_op1 : tasmsymbol);
      begin
         inherited create(op);
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym_ofs(op : tasmop;_op1 : tasmsymbol;_op1ofs:longint);
      begin
         inherited create(op);
         ops:=1;
         loadsymbol(0,_op1,_op1ofs);
      end;


     constructor taicpu.op_reg_sym_ofs(op : tasmop;_op1 : tregister;_op2:tasmsymbol;_op2ofs : longint);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadsymbol(1,_op2,_op2ofs);
      end;


    constructor taicpu.op_sym_ofs_ref(op : tasmop;_op1 : tasmsymbol;_op1ofs:longint;const _op2 : treference);
      begin
         inherited create(op);
         ops:=2;
         loadsymbol(0,_op1,_op1ofs);
         loadref(1,_op2);
      end;


    function taicpu.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        { allow the register allocator to remove unnecessary moves }
        result:=(
                  ((opcode=A_MOV) and (regtype = R_INTREGISTER)) or
                  ((opcode=A_FMOV) and (regtype = R_MMREGISTER))
                ) and
                (oppostfix in [PF_None]) and
                (condition=C_None) and
                (ops=2) and
                (oper[0]^.typ=top_reg) and
                (oper[1]^.typ=top_reg) and
                (oper[0]^.reg=oper[1]^.reg);
      end;


    function spilling_create_op(op: tasmop; const ref: treference; r: tregister): taicpu;
      const
        { invalid sizes for aarch64 are 0 }
        subreg2bytesize: array[TSubRegister] of byte =
          (0,0,0,0,4,8,0,0,0,4,8,0,0,0,0,0,0,0,0,0,0,0,0,8,16,0,16,16,16,16,16,16,16,16,16,16);
      var
        scalefactor: byte;
      begin
        scalefactor:=subreg2bytesize[getsubreg(r)];
        if scalefactor=0 then
          internalerror(2014120301);
        if (ref.offset>4095*scalefactor) or
           ((ref.offset>255) and
            ((ref.offset mod scalefactor)<>0)) or
           (ref.offset<-256) then
          internalerror(2014120302);
        case getregtype(r) of
          R_INTREGISTER,
          R_MMREGISTER:
            result:=taicpu.op_reg_ref(op,r,ref);
          else
            internalerror(2004010407);
        end;
      end;


    function is_valid_load_symbol(op: tasmop; oppostfix: toppostfix; const ref: treference): tsimplereftype;
      begin
        result:=sr_complex;
        if not assigned(ref.symboldata) and
           not(ref.refaddr in [addr_pic,addr_gotpageoffset,addr_gotpage,addr_pageoffset,addr_page]) then
          exit;
        { can't use pre-/post-indexed mode here (makes no sense either) }
        if ref.addressmode<>AM_OFFSET then
          exit;
        { "ldr literal" must be a 32/64 bit LDR and have a symbol }
        if (ref.refaddr=addr_pic) and
           (not (op in [A_LDR,A_B,A_BL]) or
            not(oppostfix in [PF_NONE,PF_W,PF_SW]) or
            (not assigned(ref.symbol) and
             not assigned(ref.symboldata))) then
          exit;
        { if this is a (got) page offset load, we must have a base register and a
          symbol (except if we have an ADD with a non-got page offset load) }
        if (ref.refaddr in [addr_gotpageoffset,addr_pageoffset]) and
           (
             (
               (
                 (op<>A_ADD) or
                 (ref.refaddr=addr_gotpageoffset)
               ) and
               (
                 not assigned(ref.symbol) or
                 (ref.base=NR_NO)
               )
             ) or
             (
               (
                 (op=A_ADD) and
                 (ref.refaddr=addr_pageoffset)
               ) and
               not assigned(ref.symbol) and
               (ref.base=NR_NO)
             ) or
            (ref.index<>NR_NO) or
            (ref.offset<>0)) then
          begin
            result:=sr_internal_illegal;
            exit;
          end;
        { cannot have base or index register (we generate these kind of
          references internally, they should never end up here with an
          extra base or offset) }
        if (ref.refaddr in [addr_gotpage,addr_page]) and
           (ref.base<>NR_NO) or
           (ref.index<>NR_NO) then
          begin
            result:=sr_internal_illegal;
            exit;
          end;
        result:=sr_simple;
      end;


    function simple_ref_type(op: tasmop; size:tcgsize; oppostfix: toppostfix; const ref: treference): tsimplereftype;
      var
        accesssize: longint;
      begin
        result:=sr_internal_illegal;
        { post-indexed is only allowed for vector and immediate loads/stores }
        if (ref.addressmode=AM_POSTINDEXED) and
           not((op = A_LD1) or (op = A_LD2) or (op = A_LD3) or (op = A_LD4) or
               (op = A_LD1R) or (op = A_LD2R) or (op = A_LD3R) or (op = A_LD4R) or
               (op = A_ST1) or (op = A_ST2) or (op = A_ST3) or (op = A_ST4)) and
           (not(op in [A_LDR,A_STR,A_LDP,A_STP]) or
            (ref.base=NR_NO) or
            (ref.index<>NR_NO)) then
          exit;

        { can only have a shift mode if we have an index }
        if (ref.index=NR_NO) and
           (ref.shiftmode<>SM_None) then
          exit;

        { the index can never be the stack pointer }
        if ref.index=NR_SP then
          exit;

        { no instruction supports an index without a base }
        if (ref.base=NR_NO) and
           (ref.index<>NR_NO) then
          begin
            result:=sr_complex;
            exit;
          end;

        { LDR literal or GOT entry: 32 or 64 bit, label }
        if assigned(ref.symboldata) or
           assigned(ref.symbol) then
          begin
            { we generate these kind of references internally; at least for now,
              they should never end up here with an extra base or offset or so }
            result:=is_valid_load_symbol(op,oppostfix,ref);
            exit;
          end;

        { any other reference cannot be gotpage/gotpageoffset/pic }
        if ref.refaddr in [addr_gotpage,addr_gotpageoffset,addr_page,addr_pageoffset,addr_pic] then
          exit;

        { base & index:
            * index cannot be the stack pointer
            * offset must be 0
            * can scale with the size of the access
            * can zero/sign extend 32 bit index register, and/or multiple by
              access size
            * no pre/post-indexing except for ldN(r)/stN
        }
        if (ref.base<>NR_NO) and
           (ref.index<>NR_NO) then
          begin
            case op of
              { this holds for both integer and fpu/vector loads }
              A_LDR,A_STR:
                begin
                  if ref.addressmode in [AM_PREINDEXED,AM_POSTINDEXED] then
                    exit;
                  if (ref.offset=0) and
                     (((ref.shiftmode=SM_None) and
                       (ref.shiftimm=0)) or
                      ((ref.shiftmode in [SM_LSL,SM_UXTW,SM_SXTW]) and
                       (ref.shiftimm=tcgsizep2size[size]))) then
                    result:=sr_simple
                  else
                    result:=sr_complex;
                end;
              A_LD1,A_LD2,A_LD3,A_LD4,
              A_LD1R,A_LD2R,A_LD3R,A_LD4R,
              A_ST1,A_ST2,A_ST3,A_ST4:
                begin
                  if ref.addressmode in [AM_PREINDEXED] then
                    exit;
                  if (ref.offset=0) and
                     (ref.addressmode=AM_POSTINDEXED) then
                    result:=sr_simple
                  else
                   result:=sr_complex;
                end;
              { these don't support base+index }
              A_LDUR,A_STUR,
              A_LDP,A_STP:
                begin
                  if ref.addressmode in [AM_PREINDEXED,AM_POSTINDEXED] then
                    exit;
                  result:=sr_complex;
                end
              else
                { nothing: result is already sr_internal_illegal };
            end;
            exit;
          end;

        { base + immediate offset. Variants:
            * LDR*/STR*:
              - pre- or post-indexed with signed 9 bit immediate
              - regular with unsiged scaled immediate (multiple of access
                size), in the range 0 to (12 bit * access_size)-1
            * LDP/STP
              - pre- or post-indexed with signed 9 bit immediate
              - regular with signed 9 bit immediate
            * LDUR*/STUR*:
              - regular with signed 9 bit immediate
            * ldN(r)/stN
              - 0 or with postindex
        }
        if ref.base<>NR_NO then
          begin
            accesssize:=1 shl tcgsizep2size[size];
            case op of
              A_LDR,A_STR:
                begin
                  if (ref.addressmode=AM_OFFSET) and
                     (ref.offset>=0) and
                     (ref.offset<(((1 shl 12)-1)*accesssize)) and
                     ((ref.offset mod accesssize)=0) then
                    result:=sr_simple
                  else if (ref.offset>=-256) and
                     (ref.offset<=255) then
                    begin
                      { non pre-/post-indexed regular loads/stores can only be
                        performed using LDUR/STUR }
                      if ref.addressmode in [AM_PREINDEXED,AM_POSTINDEXED] then
                        result:=sr_simple
                      else
                        result:=sr_complex
                    end
                  else
                    result:=sr_complex;
                end;
              A_LDP,A_LDNP,
              A_STP,A_STNP:
                begin
                  { only supported for 32/64 bit }
                  if not(oppostfix in [PF_W,PF_SW,PF_None]) then
                    exit;
                  { offset must be a multple of the access size }
                  if (ref.offset mod accesssize)<>0 then
                    exit;
                  { offset must fit in a signed 7 bit offset }
                  if (ref.offset>=-(1 shl (6+tcgsizep2size[size]))) and
                     (ref.offset<=(1 shl (6+tcgsizep2size[size]))-1) then
                    result:=sr_simple
                  else
                    result:=sr_complex;
                end;
              A_LDUR,A_STUR:
                begin
                  if ref.addressmode in [AM_PREINDEXED,AM_POSTINDEXED] then
                    exit;
                  if (ref.offset>=-256) and
                     (ref.offset<=255) then
                    result:=sr_simple
                  else
                    result:=sr_complex;
                end;
              A_LD1,A_LD2,A_LD3,A_LD4,
              A_LD1R,A_LD2R,A_LD3R,A_LD4R,
              A_ST1,A_ST2,A_ST3,A_ST4:
                begin
                  if ref.addressmode in [AM_PREINDEXED] then
                    exit;
                  if (ref.offset=0) or
                     ((ref.addressmode=AM_POSTINDEXED) and
                      { to check the validity of the offset, we'd have to analyse the regset argument }
                      (ref.offset>0)) then
                    result:=sr_simple
                  else
                    result:=sr_complex;
                end;
              A_LDAR,
              A_LDAXR,
              A_LDXR,
              A_LDXP,
              A_STLR,
              A_STLXR,
              A_STLXP,
              A_STXP,
              A_STXR:
                begin
                  if (ref.addressmode=AM_OFFSET) and
                     (ref.offset=0) then
                    result:=sr_simple;
                end
              else
                { nothing: result is already sr_internal_illegal };
            end;
            exit;
          end;
        { absolute addresses are not supported, have to load them first into
          a register }
        result:=sr_complex;
      end;


    function can_be_shifter_operand(opc: tasmop; opnr: longint): boolean;
      begin
        case opc of
          A_ADD,
          A_AND,
          A_EON,
          A_EOR,
          A_ORN,
          A_ORR,
          A_SUB:
            result:=opnr=3;
          A_BIC,
          A_CMN,
          A_CMP,
          A_MOVK,
          A_MOVZ,
          A_MOVN,
          A_MVN,
          A_NEG,
          A_TST:
            result:=opnr=2;
          else
            result:=false;
        end;
      end;


    function valid_shifter_operand(opc: tasmop; useszr, usessp, is64bit: boolean; sm: tshiftmode; shiftimm: longint): boolean;
      begin
        case opc of
          A_ADD,
          A_SUB,
          A_NEG,
          A_AND,
          A_TST,
          A_CMN,
          A_CMP:
            begin
              result:=false;
              if not useszr then
                result:=
                  (sm in shiftedregmodes) and
                  ((shiftimm in [0..31]) or
                   (is64bit and
                    (shiftimm in [32..63])));
              if not usessp then
                result:=
                  result or
                  ((sm in extendedregmodes) and
                   (shiftimm in [0..4]));
            end;
          A_BIC,
          A_EON,
          A_EOR,
          A_MVN,
          A_ORN,
          A_ORR:
            result:=
              (sm in shiftedregmodes) and
              (shiftimm in [0..31*(ord(is64bit)+1)+ord(is64bit)]);
          A_MOVK,
          A_MOVZ,
          A_MOVN:
            result:=
              (sm=SM_LSL) and
              ((shiftimm in [0,16]) or
               (is64bit and
                (shiftimm in [32,48])));
          else
            result:=false;
        end;
      end;


    function spilling_create_load(const ref: treference; r: tregister): taicpu;
      var
        op: tasmop;
      begin
        if (ref.index<>NR_NO) or
           (ref.offset<-256) or
           (ref.offset>255) then
          op:=A_LDR
        else
          op:=A_LDUR;
        result:=spilling_create_op(op,ref,r);
      end;


    function spilling_create_store(r: tregister; const ref: treference): taicpu;
      var
        op: tasmop;
      begin
        if (ref.index<>NR_NO) or
           (ref.offset<-256) or
           (ref.offset>255) then
          op:=A_STR
        else
          op:=A_STUR;
        result:=spilling_create_op(op,ref,r);
      end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        case opcode of
          A_B,A_BL,A_BR,A_BLR,
          A_CMN,A_CMP,
          A_CCMN,A_CCMP,
          A_TST,
          A_FCMP,A_FCMPE,
          A_CBZ,A_CBNZ,
          A_RET:
            result:=operand_read;
          A_STR,A_STUR:
            if opnr=0 then
              result:=operand_read
            else
              { check for pre/post indexed in spilling_get_operation_type_ref }
              result:=operand_read;
          A_STLXP,
          A_STLXR,
          A_STXP,
          A_STXR:
            if opnr=0 then
              result:=operand_write
            else
              result:=operand_read;
          A_STP:
            begin
              if opnr in [0,1] then
                result:=operand_read
              else
                { check for pre/post indexed in spilling_get_operation_type_ref }
                result:=operand_read;
            end;
           A_LDP,
           A_LDXP:
             begin
               if opnr in [0,1] then
                 result:=operand_write
               else
                 { check for pre/post indexed in spilling_get_operation_type_ref }
                 result:=operand_read;
             end;
{$ifdef EXTDEBUG}
           { play save to avoid hard to find bugs, better fail at compile time }
           A_ADD,
           A_ADRP,
           A_AND,
           A_ASR,
           A_BFI,
           A_BFXIL,
           A_CLZ,
           A_CSEL,
           A_CSET,
           A_CSETM,
           A_FABS,
           A_EON,
           A_EOR,
           A_FADD,
           A_FCVT,
           A_FDIV,
           A_FMADD,
           A_FMOV,
           A_FMSUB,
           A_FMUL,
           A_FNEG,
           A_FNMADD,
           A_FNMSUB,
           A_FRINTX,
           A_FSQRT,
           A_FSUB,
           A_ORR,
           A_LSL,
           A_LSLV,
           A_LSR,
           A_LSRV,
           A_MOV,
           A_MOVK,
           A_MOVN,
           A_MOVZ,
           A_MSUB,
           A_MUL,
           A_MVN,
           A_NEG,
           A_LDR,
           A_LDUR,
           A_RBIT,
           A_ROR,
           A_RORV,
           A_SBFX,
           A_SCVTF,
           A_FCVTZS,
           A_SDIV,
           A_SMULL,
           A_SUB,
           A_UBFIZ,
           A_UBFX,
           A_UCVTF,
           A_UDIV,
           A_UMULL:
             if opnr=0 then
               result:=operand_write
             else
               result:=operand_read;
           else
             Internalerror(2019090802);
{$else EXTDEBUG}
           else
             if opnr=0 then
               result:=operand_write
             else
               result:=operand_read;
{$endif EXTDEBUG}
        end;
      end;


    function taicpu.spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;
      begin
        result:=operand_read;
        if (oper[opnr]^.ref^.base = reg) and
          (oper[opnr]^.ref^.addressmode in [AM_PREINDEXED,AM_POSTINDEXED]) then
           result:=operand_readwrite;
      end;


    procedure BuildInsTabCache;
//      var
//        i : longint;
      begin
(*        new(instabcache);
        FillChar(instabcache^,sizeof(tinstabcache),$ff);
        i:=0;
        while (i<InsTabEntries) do
          begin
            if InsTabCache^[InsTab[i].Opcode]=-1 then
              InsTabCache^[InsTab[i].Opcode]:=i;
            inc(i);
          end; *)
      end;


    procedure InitAsm;
      begin
        if not assigned(instabcache) then
          BuildInsTabCache;
      end;


    procedure DoneAsm;
      begin
        if assigned(instabcache) then
          begin
            dispose(instabcache);
            instabcache:=nil;
          end;
      end;


    function setoppostfix(i : taicpu;pf : toppostfix) : taicpu;
      begin
        i.oppostfix:=pf;
        result:=i;
      end;


    function setcondition(i : taicpu;c : tasmcond) : taicpu;
      begin
        i.condition:=c;
        result:=i;
      end;


    Function SimpleGetNextInstruction(Current: tai; Var Next: tai): Boolean;
      Begin
        Current:=tai(Current.Next);
        While Assigned(Current) And (Current.typ In SkipInstr) Do
          Current:=tai(Current.Next);
        Next:=Current;
        If Assigned(Next) And Not(Next.typ In SkipInstr) Then
           Result:=True
          Else
            Begin
              Next:=Nil;
              Result:=False;
            End;
      End;


(*
    function armconstequal(hp1,hp2: tai): boolean;
      begin
        result:=false;
        if hp1.typ<>hp2.typ then
          exit;
        case hp1.typ of
          tai_const:
            result:=
              (tai_const(hp2).sym=tai_const(hp).sym) and
              (tai_const(hp2).value=tai_const(hp).value) and
              (tai(hp2.previous).typ=ait_label);
            tai_const:
              result:=
                (tai_const(hp2).sym=tai_const(hp).sym) and
                (tai_const(hp2).value=tai_const(hp).value) and
                (tai(hp2.previous).typ=ait_label);
        end;
      end;
*)

    procedure insertpcrelativedata(list,listtoinsert : TAsmList);
(*
      var
        curinspos,
        penalty,
        lastinspos,
        { increased for every data element > 4 bytes inserted }
        currentsize,
        extradataoffset,
        limit: longint;
        curop : longint;
        curtai : tai;
        curdatatai,hp,hp2 : tai;
        curdata : TAsmList;
        l : tasmlabel;
        doinsert,
        removeref : boolean;
*)
      begin
(*
        curdata:=TAsmList.create;
        lastinspos:=-1;
        curinspos:=0;
        extradataoffset:=0;
        limit:=1016;
        curtai:=tai(list.first);
        doinsert:=false;
        while assigned(curtai) do
          begin
            { instruction? }
            case curtai.typ of
              ait_instruction:
                begin
                  { walk through all operand of the instruction }
                  for curop:=0 to taicpu(curtai).ops-1 do
                    begin
                      { reference? }
                      if (taicpu(curtai).oper[curop]^.typ=top_ref) then
                        begin
                          { pc relative symbol? }
                          curdatatai:=tai(taicpu(curtai).oper[curop]^.ref^.symboldata);
                          if assigned(curdatatai) and
                            { move only if we're at the first reference of a label }
                            not(tai_label(curdatatai).moved) then
                            begin
                              tai_label(curdatatai).moved:=true;
                              { check if symbol already used. }
                              { if yes, reuse the symbol }
                              hp:=tai(curdatatai.next);
                              removeref:=false;
                              if assigned(hp) then
                                begin
                                  case hp.typ of
                                    ait_const:
                                      begin
                                        if (tai_const(hp).consttype=aitconst_64bit) then
                                          inc(extradataoffset);
                                      end;
                                    ait_realconst:
                                      begin
                                        inc(extradataoffset,((tai_realconst(hp).savesize-4+3) div 4));
                                      end;
                                  end;
                                  if (hp.typ=ait_const) then
                                    begin
                                      hp2:=tai(curdata.first);
                                      while assigned(hp2) do
                                        begin
    {                                      if armconstequal(hp2,hp) then }
                                          if (hp2.typ=ait_const) and (tai_const(hp2).sym=tai_const(hp).sym)
                                            and (tai_const(hp2).value=tai_const(hp).value) and (tai(hp2.previous).typ=ait_label)
                                          then
                                            begin
                                              with taicpu(curtai).oper[curop]^.ref^ do
                                                begin
                                                  symboldata:=hp2.previous;
                                                  symbol:=tai_label(hp2.previous).labsym;
                                                end;
                                              removeref:=true;
                                              break;
                                            end;
                                          hp2:=tai(hp2.next);
                                        end;
                                    end;
                                end;
                              { move or remove symbol reference }
                              repeat
                                hp:=tai(curdatatai.next);
                                listtoinsert.remove(curdatatai);
                                if removeref then
                                  curdatatai.free
                                else
                                  curdata.concat(curdatatai);
                                curdatatai:=hp;
                              until (curdatatai=nil) or (curdatatai.typ=ait_label);
                              if lastinspos=-1 then
                                lastinspos:=curinspos;
                            end;
                        end;
                    end;
                  inc(curinspos);
                end;
              ait_align:
                begin
                  { code is always 4 byte aligned, so we don't have to take care of .align 2 which would
                    requires also incrementing curinspos by 1 }
                  inc(curinspos,(tai_align(curtai).aligntype div 4));
                end;
              ait_const:
                begin
                  inc(curinspos);
                  if (tai_const(curtai).consttype=aitconst_64bit) then
                    inc(curinspos);
                end;
              ait_realconst:
                begin
                  inc(curinspos,(tai_realconst(hp).savesize+3) div 4);
                end;
            end;
            { special case for case jump tables }
            if SimpleGetNextInstruction(curtai,hp) and
              (tai(hp).typ=ait_instruction) and
              (taicpu(hp).opcode=A_LDR) and
              (taicpu(hp).oper[0]^.typ=top_reg) and
              (taicpu(hp).oper[0]^.reg=NR_PC) then
              begin
                penalty:=1;
                hp:=tai(hp.next);
                { skip register allocations and comments inserted by the optimizer }
                while assigned(hp) and (hp.typ in [ait_comment,ait_regalloc]) do
                  hp:=tai(hp.next);
                while assigned(hp) and (hp.typ=ait_const) do
                  begin
                    inc(penalty);
                    hp:=tai(hp.next);
                  end;
              end
            else
              penalty:=0;

            { FLD/FST VFP instructions have a limit of +/- 1024, not 4096 }
            if SimpleGetNextInstruction(curtai,hp) and
               (tai(hp).typ=ait_instruction) and
               ((taicpu(hp).opcode=A_FLDS) or
                (taicpu(hp).opcode=A_FLDD)) then
              limit:=254;

            { don't miss an insert }
            doinsert:=doinsert or
              (not(curdata.empty) and
               (curinspos-lastinspos+penalty+extradataoffset>limit));

            { split only at real instructions else the test below fails }
            if doinsert and (curtai.typ=ait_instruction) and
              (
                { don't split loads of pc to lr and the following move }
                not(
                    (taicpu(curtai).opcode=A_MOV) and
                    (taicpu(curtai).oper[0]^.typ=top_reg) and
                    (taicpu(curtai).oper[0]^.reg=NR_R14) and
                    (taicpu(curtai).oper[1]^.typ=top_reg) and
                    (taicpu(curtai).oper[1]^.reg=NR_PC)
                   )
              ) then
              begin
                lastinspos:=-1;
                extradataoffset:=0;
                limit:=1016;
                doinsert:=false;
                hp:=tai(curtai.next);
                current_asmdata.getjumplabel(l);
                curdata.insert(taicpu.op_sym(A_B,l));
                curdata.concat(tai_label.create(l));
                list.insertlistafter(curtai,curdata);
                curtai:=hp;
              end
            else
              curtai:=tai(curtai.next);
          end;
        list.concatlist(curdata);
        curdata.free;
*)
      end;


    procedure finalizearmcode(list, listtoinsert: TAsmList);
      begin
        insertpcrelativedata(list, listtoinsert);
      end;

(*
      Floating point instruction format information, taken from the linux kernel
      ARM Floating Point Instruction Classes
      | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
      |c o n d|1 1 0 P|U|u|W|L|   Rn  |v|  Fd |0|0|0|1|  o f f s e t  | CPDT
      |c o n d|1 1 0 P|U|w|W|L|   Rn  |x|  Fd |0|0|1|0|  o f f s e t  | CPDT (copro 2)
      | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
      |c o n d|1 1 1 0|a|b|c|d|e|  Fn |j|  Fd |0|0|0|1|f|g|h|0|i|  Fm | CPDO
      |c o n d|1 1 1 0|a|b|c|L|e|  Fn |   Rd  |0|0|0|1|f|g|h|1|i|  Fm | CPRT
      |c o n d|1 1 1 0|a|b|c|1|e|  Fn |1|1|1|1|0|0|0|1|f|g|h|1|i|  Fm | comparisons
      | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |

      CPDT            data transfer instructions
                      LDF, STF, LFM (copro 2), SFM (copro 2)

      CPDO            dyadic arithmetic instructions
                      ADF, MUF, SUF, RSF, DVF, RDF,
                      POW, RPW, RMF, FML, FDV, FRD, POL

      CPDO            monadic arithmetic instructions
                      MVF, MNF, ABS, RND, SQT, LOG, LGN, EXP,
                      SIN, COS, TAN, ASN, ACS, ATN, URD, NRM

      CPRT            joint arithmetic/data transfer instructions
                      FIX (arithmetic followed by load/store)
                      FLT (load/store followed by arithmetic)
                      CMF, CNF CMFE, CNFE (comparisons)
                      WFS, RFS (write/read floating point status register)
                      WFC, RFC (write/read floating point control register)

      cond            condition codes
      P               pre/post index bit: 0 = postindex, 1 = preindex
      U               up/down bit: 0 = stack grows down, 1 = stack grows up
      W               write back bit: 1 = update base register (Rn)
      L               load/store bit: 0 = store, 1 = load
      Rn              base register
      Rd              destination/source register
      Fd              floating point destination register
      Fn              floating point source register
      Fm              floating point source register or floating point constant

      uv              transfer length (TABLE 1)
      wx              register count (TABLE 2)
      abcd            arithmetic opcode (TABLES 3 & 4)
      ef              destination size (rounding precision) (TABLE 5)
      gh              rounding mode (TABLE 6)
      j               dyadic/monadic bit: 0 = dyadic, 1 = monadic
      i               constant bit: 1 = constant (TABLE 6)
      */

      /*
      TABLE 1
      +-------------------------+---+---+---------+---------+
      |  Precision              | u | v | FPSR.EP | length  |
      +-------------------------+---+---+---------+---------+
      | Single                  | 0 | 0 |    x    | 1 words |
      | Double                  | 1 | 1 |    x    | 2 words |
      | Extended                | 1 | 1 |    x    | 3 words |
      | Packed decimal          | 1 | 1 |    0    | 3 words |
      | Expanded packed decimal | 1 | 1 |    1    | 4 words |
      +-------------------------+---+---+---------+---------+
      Note: x = don't care
      */

      /*
      TABLE 2
      +---+---+---------------------------------+
      | w | x | Number of registers to transfer |
      +---+---+---------------------------------+
      | 0 | 1 |  1                              |
      | 1 | 0 |  2                              |
      | 1 | 1 |  3                              |
      | 0 | 0 |  4                              |
      +---+---+---------------------------------+
      */

      /*
      TABLE 3: Dyadic Floating Point Opcodes
      +---+---+---+---+----------+-----------------------+-----------------------+
      | a | b | c | d | Mnemonic | Description           | Operation             |
      +---+---+---+---+----------+-----------------------+-----------------------+
      | 0 | 0 | 0 | 0 | ADF      | Add                   | Fd := Fn + Fm         |
      | 0 | 0 | 0 | 1 | MUF      | Multiply              | Fd := Fn * Fm         |
      | 0 | 0 | 1 | 0 | SUF      | Subtract              | Fd := Fn - Fm         |
      | 0 | 0 | 1 | 1 | RSF      | Reverse subtract      | Fd := Fm - Fn         |
      | 0 | 1 | 0 | 0 | DVF      | Divide                | Fd := Fn / Fm         |
      | 0 | 1 | 0 | 1 | RDF      | Reverse divide        | Fd := Fm / Fn         |
      | 0 | 1 | 1 | 0 | POW      | Power                 | Fd := Fn ^ Fm         |
      | 0 | 1 | 1 | 1 | RPW      | Reverse power         | Fd := Fm ^ Fn         |
      | 1 | 0 | 0 | 0 | RMF      | Remainder             | Fd := IEEE rem(Fn/Fm) |
      | 1 | 0 | 0 | 1 | FML      | Fast Multiply         | Fd := Fn * Fm         |
      | 1 | 0 | 1 | 0 | FDV      | Fast Divide           | Fd := Fn / Fm         |
      | 1 | 0 | 1 | 1 | FRD      | Fast reverse divide   | Fd := Fm / Fn         |
      | 1 | 1 | 0 | 0 | POL      | Polar angle (ArcTan2) | Fd := arctan2(Fn,Fm)  |
      | 1 | 1 | 0 | 1 |          | undefined instruction | trap                  |
      | 1 | 1 | 1 | 0 |          | undefined instruction | trap                  |
      | 1 | 1 | 1 | 1 |          | undefined instruction | trap                  |
      +---+---+---+---+----------+-----------------------+-----------------------+
      Note: POW, RPW, POL are deprecated, and are available for backwards
            compatibility only.
      */

      /*
      TABLE 4: Monadic Floating Point Opcodes
      +---+---+---+---+----------+-----------------------+-----------------------+
      | a | b | c | d | Mnemonic | Description           | Operation             |
      +---+---+---+---+----------+-----------------------+-----------------------+
      | 0 | 0 | 0 | 0 | MVF      | Move                  | Fd := Fm              |
      | 0 | 0 | 0 | 1 | MNF      | Move negated          | Fd := - Fm            |
      | 0 | 0 | 1 | 0 | ABS      | Absolute value        | Fd := abs(Fm)         |
      | 0 | 0 | 1 | 1 | RND      | Round to integer      | Fd := int(Fm)         |
      | 0 | 1 | 0 | 0 | SQT      | Square root           | Fd := sqrt(Fm)        |
      | 0 | 1 | 0 | 1 | LOG      | Log base 10           | Fd := log10(Fm)       |
      | 0 | 1 | 1 | 0 | LGN      | Log base e            | Fd := ln(Fm)          |
      | 0 | 1 | 1 | 1 | EXP      | Exponent              | Fd := e ^ Fm          |
      | 1 | 0 | 0 | 0 | SIN      | Sine                  | Fd := sin(Fm)         |
      | 1 | 0 | 0 | 1 | COS      | Cosine                | Fd := cos(Fm)         |
      | 1 | 0 | 1 | 0 | TAN      | Tangent               | Fd := tan(Fm)         |
      | 1 | 0 | 1 | 1 | ASN      | Arc Sine              | Fd := arcsin(Fm)      |
      | 1 | 1 | 0 | 0 | ACS      | Arc Cosine            | Fd := arccos(Fm)      |
      | 1 | 1 | 0 | 1 | ATN      | Arc Tangent           | Fd := arctan(Fm)      |
      | 1 | 1 | 1 | 0 | URD      | Unnormalized round    | Fd := int(Fm)         |
      | 1 | 1 | 1 | 1 | NRM      | Normalize             | Fd := norm(Fm)        |
      +---+---+---+---+----------+-----------------------+-----------------------+
      Note: LOG, LGN, EXP, SIN, COS, TAN, ASN, ACS, ATN are deprecated, and are
            available for backwards compatibility only.
      */

      /*
      TABLE 5
      +-------------------------+---+---+
      |  Rounding Precision     | e | f |
      +-------------------------+---+---+
      | IEEE Single precision   | 0 | 0 |
      | IEEE Double precision   | 0 | 1 |
      | IEEE Extended precision | 1 | 0 |
      | undefined (trap)        | 1 | 1 |
      +-------------------------+---+---+
      */

      /*
      TABLE 5
      +---------------------------------+---+---+
      |  Rounding Mode                  | g | h |
      +---------------------------------+---+---+
      | Round to nearest (default)      | 0 | 0 |
      | Round toward plus infinity      | 0 | 1 |
      | Round toward negative infinity  | 1 | 0 |
      | Round toward zero               | 1 | 1 |
      +---------------------------------+---+---+
*)
    function taicpu.GetString:string;
      var
        i : longint;
        s : string;
        addsize : boolean;
      begin
        s:='['+gas_op2str[opcode];
        for i:=0 to ops-1 do
         begin
           with oper[i]^ do
             begin
               if i=0 then
                s:=s+' '
               else
                s:=s+',';
               { type }
               addsize:=false;
               if (ot and OT_VREG)=OT_VREG then
                s:=s+'vreg'
               else
                 if (ot and OT_FPUREG)=OT_FPUREG then
                  s:=s+'fpureg'
               else
                if (ot and OT_REGISTER)=OT_REGISTER then
                 begin
                   s:=s+'reg';
                   addsize:=true;
                 end
               else
                if (ot and OT_REGLIST)=OT_REGLIST then
                 begin
                   s:=s+'reglist';
                   addsize:=false;
                 end
               else
                if (ot and OT_IMMEDIATE)=OT_IMMEDIATE then
                 begin
                   s:=s+'imm';
                   addsize:=true;
                 end
               else
                if (ot and OT_MEMORY)=OT_MEMORY then
                 begin
                   s:=s+'mem';
                   addsize:=true;
                   if (ot and OT_AM2)<>0 then
                     s:=s+' am2 ';
                 end
               else
                 s:=s+'???';
               { size }
               if addsize then
                begin
                  if (ot and OT_BITS8)<>0 then
                    s:=s+'8'
                  else
                   if (ot and OT_BITS16)<>0 then
                    s:=s+'24'
                  else
                   if (ot and OT_BITS32)<>0 then
                    s:=s+'32'
                  else
                   if (ot and OT_BITSSHIFTER)<>0 then
                    s:=s+'shifter'
                  else
                    s:=s+'??';
                  { signed }
                  if (ot and OT_SIGNED)<>0 then
                   s:=s+'s';
                end;
             end;
         end;
        GetString:=s+']';
      end;


    procedure taicpu.ResetPass1;
      begin
        { we need to reset everything here, because the choosen insentry
          can be invalid for a new situation where the previously optimized
          insentry is not correct }
      end;


    procedure taicpu.ResetPass2;
      begin
        { we are here in a second pass, check if the instruction can be optimized }
      end;


    function taicpu.CheckIfValid:boolean;
      begin
        Result:=False; { unimplemented }
      end;


    function taicpu.Pass1(objdata:TObjData):longint;
      begin
        Pass1:=0;
      end;


    procedure taicpu.Pass2(objdata:TObjData);
      begin
        { error in pass1 ? }
        current_filepos:=fileinfo;
        { Generate the instruction }
        { GenCode(objdata); }
      end;


    procedure taicpu.ppuloadoper(ppufile:tcompilerppufile;var o:toper);
      begin
      end;


    procedure taicpu.ppuwriteoper(ppufile:tcompilerppufile;const o:toper);
      begin
      end;


    procedure taicpu.ppubuildderefimploper(var o:toper);
      begin
      end;


    procedure taicpu.ppuderefoper(var o:toper);
      begin
      end;


begin
  cai_align:=tai_align;
end.

