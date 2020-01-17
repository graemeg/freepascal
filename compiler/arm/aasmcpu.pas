{
    Copyright (c) 2003 by Florian Klaempfl

    Contains the assembler object for the ARM

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
  globtype,globals,verbose,
  aasmbase,aasmtai,aasmdata,aasmsym,
  ogbase,
  symtype,
  cpubase,cpuinfo,cgbase,cgutils,
  sysutils;

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
      OT_NON_SIZE  = $0FFFF800;
      OT_OPT_SIZE  = $F0000000;

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
      OT_IMMEDIATEZERO = $10002200;
      OT_IMMEDIATEMM     = $00002400;
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
      OT_REGLO     = $10201004;  { lower reg (r0-r7) }
      OT_REGSP     = $20201004;
      OT_REG64     = $00201008;
      OT_VREG      = $00201010;  { vector register }
      OT_REGF      = $00201020;  { coproc register }
      OT_REGS      = $00201040;  { special register with mask }
      OT_MEMORY    = $00204000;  { register number in 'basereg'  }
      OT_MEM8      = $00204001;
      OT_MEM16     = $00204002;
      OT_MEM32     = $00204004;
      OT_MEM64     = $00204008;
      OT_MEM80     = $00204010;
      { word/byte load/store }
      OT_AM2       = $00010000;
      { misc ld/st operations, thumb reg indexed }
      OT_AM3       = $00020000;
      { multiple ld/st operations or thumb imm indexed }
      OT_AM4       = $00040000;
      { co proc. ld/st operations or thumb sp+imm indexed }
      OT_AM5       = $00080000;
      { exclusive ld/st operations or thumb pc+imm indexed }
      OT_AM6       = $00100000;
      OT_AMMASK    = $001f0000;
      { IT instruction }
      OT_CONDITION = $00200000;
      OT_MODEFLAGS = $00400000;

      OT_MEMORYAM2 = OT_MEMORY or OT_AM2;
      OT_MEMORYAM3 = OT_MEMORY or OT_AM3;
      OT_MEMORYAM4 = OT_MEMORY or OT_AM4;
      OT_MEMORYAM5 = OT_MEMORY or OT_AM5;
      OT_MEMORYAM6 = OT_MEMORY or OT_AM6;

      OT_FPUREG    = $01000000;  { floating point stack registers  }
      OT_REG_SMASK = $00070000;  { special register operands: these may be treated differently  }
                                 { a mask for the following  }

      OT_MEM_OFFS  = $00604000;  { special type of EA  }
                                 { simple [address] offset  }
      OT_ONENESS   = $00800000;  { special type of immediate operand  }
                                 { so UNITY == IMMEDIATE | ONENESS  }
      OT_UNITY     = $00802000;  { for shift/rotate instructions  }

      instabentries = {$i armnop.inc}

      maxinfolen = 5;

      IF_NONE   = $00000000;

      IF_EXTENSIONS = $0000000F;

      IF_NEON       = $00000001;

      IF_ARMMASK    = $000F0000;
      IF_ARM32      = $00010000;
      IF_THUMB      = $00020000;
      IF_THUMB32    = $00040000;
      IF_WIDE       = $00080000;

      IF_ARMvMASK   = $0FF00000;
      IF_ARMv4      = $00100000;
      IF_ARMv4T     = $00200000;
      IF_ARMv5      = $00300000;
      IF_ARMv5T     = $00400000;
      IF_ARMv5TE    = $00500000;
      IF_ARMv5TEJ   = $00600000;
      IF_ARMv6      = $00700000;
      IF_ARMv6K     = $00800000;
      IF_ARMv6T2    = $00900000;
      IF_ARMv6Z     = $00A00000;
      IF_ARMv6M     = $00B00000;
      IF_ARMv7      = $00C00000;
      IF_ARMv7A     = $00D00000;
      IF_ARMv7R     = $00E00000;
      IF_ARMv7M     = $00F00000;
      IF_ARMv7EM    = $01000000;

      IF_FPMASK     = $F0000000;
      IF_FPA        = $10000000;
      IF_VFPv2      = $20000000;
      IF_VFPv3      = $40000000;
      IF_VFPv4      = $80000000;

      { if the instruction can change in a second pass }
      IF_PASS2  = longint($80000000);

    type
      TInsTabCache=array[TasmOp] of longint;
      PInsTabCache=^TInsTabCache;

      tinsentry = record
        opcode  : tasmop;
        ops     : byte;
        optypes : array[0..5] of longint;
        code    : array[0..maxinfolen] of char;
        flags   : longword;
      end;

      pinsentry=^tinsentry;

    const
      InsTab : array[0..instabentries-1] of TInsEntry={$i armtab.inc}

    var
      InsTabCache : PInsTabCache;

    type
      taicpu = class(tai_cpu_abstract_sym)
         oppostfix : TOpPostfix;
         wideformat : boolean;
         roundingmode : troundingmode;
         procedure loadshifterop(opidx:longint;const so:tshifterop);
         procedure loadregset(opidx:longint; regsetregtype: tregistertype; regsetsubregtype: tsubregister; const s:tcpuregisterset; ausermode: boolean=false);
         procedure loadconditioncode(opidx:longint;const acond:tasmcond);
         procedure loadmodeflags(opidx:longint;const flags:tcpumodeflags);
         procedure loadspecialreg(opidx:longint;const areg:tregister; const aflags:tspecialregflags);
         procedure loadrealconst(opidx:longint;const _value:bestreal);

         constructor op_none(op : tasmop);

         constructor op_reg(op : tasmop;_op1 : tregister);
         constructor op_ref(op : tasmop;const _op1 : treference);
         constructor op_const(op : tasmop;_op1 : longint);

         constructor op_reg_reg(op : tasmop;_op1,_op2 : tregister);
         constructor op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
         constructor op_reg_const(op:tasmop; _op1: tregister; _op2: aint);

         constructor op_regset(op:tasmop; regtype: tregistertype; subreg: tsubregister; _op1: tcpuregisterset);
         constructor op_ref_regset(op:tasmop; _op1: treference; regtype: tregistertype; subreg: tsubregister; _op2: tcpuregisterset);

         constructor op_reg_reg_reg(op : tasmop;_op1,_op2,_op3 : tregister);
         constructor op_reg_reg_const(op : tasmop;_op1,_op2 : tregister; _op3: aint);
         constructor op_reg_const_const(op : tasmop;_op1 : tregister; _op2,_op3: aint);
         constructor op_reg_reg_const_const(op : tasmop;_op1,_op2 : tregister; _op3,_op4: aint);
         constructor op_reg_reg_sym_ofs(op : tasmop;_op1,_op2 : tregister; _op3: tasmsymbol;_op3ofs: longint);
         constructor op_reg_reg_ref(op : tasmop;_op1,_op2 : tregister; const _op3: treference);
         constructor op_reg_reg_shifterop(op : tasmop;_op1,_op2 : tregister;_op3 : tshifterop);
         constructor op_reg_reg_reg_shifterop(op : tasmop;_op1,_op2,_op3 : tregister;_op4 : tshifterop);
         { SFM/LFM }
         constructor op_reg_const_ref(op : tasmop;_op1 : tregister;_op2 : aint;_op3 : treference);

         { ITxxx }
         constructor op_cond(op: tasmop; cond: tasmcond);

         { CPSxx }
         constructor op_modeflags(op: tasmop; flags: tcpumodeflags);
         constructor op_modeflags_const(op: tasmop; flags: tcpumodeflags; a: aint);

         { MSR }
         constructor op_specialreg_reg(op: tasmop; specialreg: tregister; specialregflags: tspecialregflags; _op2: tregister);

         { *M*LL }
         constructor op_reg_reg_reg_reg(op : tasmop;_op1,_op2,_op3,_op4 : tregister);

         constructor op_reg_realconst(op : tasmop;_op1: tregister;_op2: bestreal);

         { this is for Jmp instructions }
         constructor op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);

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
      private
         { pass1 info }
         inIT,
         lastinIT: boolean;
         { arm version info }
         fArmVMask,
         fArmMask  : longint;
         { next fields are filled in pass1, so pass2 is faster }
         inssize   : shortint;
         insoffset : longint;
         LastInsOffset : longint; { need to be public to be reset }
         insentry  : PInsEntry;
         procedure BuildArmMasks(objdata:TObjData);
         function  InsEnd:longint;
         procedure create_ot(objdata:TObjData);
         function  Matches(p:PInsEntry):longint;
         function  calcsize(p:PInsEntry):shortint;
         procedure gencode(objdata:TObjData);
         function  NeedAddrPrefix(opidx:byte):boolean;
         procedure Swapoperands;
         function  FindInsentry(objdata:TObjData):boolean;
      end;

      tai_align = class(tai_align_abstract)
        { nothing to add }
      end;

    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
    function spilling_create_store(r:tregister; const ref:treference):Taicpu;

    function setoppostfix(i : taicpu;pf : toppostfix) : taicpu;
    function setroundingmode(i : taicpu;rm : troundingmode) : taicpu;
    function setcondition(i : taicpu;c : tasmcond) : taicpu;

    { inserts pc relative symbols at places where they are reachable
      and transforms special instructions to valid instruction encodings }
    procedure finalizearmcode(list,listtoinsert : TAsmList);
    { inserts .pdata section and dummy function prolog needed for arm-wince exception handling }
    procedure InsertPData;

    procedure InitAsm;
    procedure DoneAsm;


implementation

  uses
    itcpugas,aoptcpu,
    systems,symdef;


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
            if assigned(add_reg_instruction_hook) then
              add_reg_instruction_hook(self,shifterop^.rs);
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


    procedure taicpu.loadregset(opidx:longint; regsetregtype: tregistertype; regsetsubregtype: tsubregister; const s:tcpuregisterset; ausermode: boolean);
      var
        i : byte;
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_regset then
             begin
               clearop(opidx);
               new(regset);
             end;
           regset^:=s;
           regtyp:=regsetregtype;
           subreg:=regsetsubregtype;
           usermode:=ausermode;
           typ:=top_regset;
           case regsetregtype of
             R_INTREGISTER:
               for i:=RS_R0 to RS_R15 do
                 begin
                   if assigned(add_reg_instruction_hook) and (i in regset^) then
                     add_reg_instruction_hook(self,newreg(R_INTREGISTER,i,regsetsubregtype));
                 end;
             R_MMREGISTER:
               { both RS_S0 and RS_D0 range from 0 to 31 }
               for i:=RS_D0 to RS_D31 do
                 begin
                   if assigned(add_reg_instruction_hook) and (i in regset^) then
                     add_reg_instruction_hook(self,newreg(R_MMREGISTER,i,regsetsubregtype));
                 end;
             else
               internalerror(2019050932);
           end;
         end;
      end;


    procedure taicpu.loadconditioncode(opidx:longint;const acond:tasmcond);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_conditioncode then
             clearop(opidx);
           cc:=acond;
           typ:=top_conditioncode;
         end;
      end;

    procedure taicpu.loadmodeflags(opidx: longint; const flags: tcpumodeflags);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_modeflags then
             clearop(opidx);
           modeflags:=flags;
           typ:=top_modeflags;
         end;
      end;

    procedure taicpu.loadspecialreg(opidx: longint; const areg: tregister; const aflags: tspecialregflags);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_specialreg then
             clearop(opidx);
           specialreg:=areg;
           specialflags:=aflags;
           typ:=top_specialreg;
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

    constructor taicpu.op_regset(op: tasmop; regtype: tregistertype; subreg: tsubregister; _op1: tcpuregisterset);
      begin
        inherited create(op);
        ops:=1;
        loadregset(0,regtype,subreg,_op1);
      end;


    constructor taicpu.op_ref_regset(op:tasmop; _op1: treference; regtype: tregistertype; subreg: tsubregister; _op2: tcpuregisterset);
      begin
         inherited create(op);
         ops:=2;
         loadref(0,_op1);
         loadregset(1,regtype,subreg,_op2);
      end;


    constructor taicpu.op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadref(1,_op2);
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


     constructor taicpu.op_reg_reg_const(op : tasmop;_op1,_op2 : tregister; _op3: aint);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadconst(2,aint(_op3));
      end;


     constructor taicpu.op_reg_const_const(op : tasmop;_op1 : tregister; _op2,_op3: aint);
       begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadconst(1,aint(_op2));
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


    constructor taicpu.op_reg_const_ref(op : tasmop;_op1 : tregister;_op2 : aint;_op3 : treference);
      begin
         inherited create(op);
         ops:=3;
         loadreg(0,_op1);
         loadconst(1,_op2);
         loadref(2,_op3);
      end;


    constructor taicpu.op_cond(op: tasmop; cond: tasmcond);
      begin
        inherited create(op);
        ops:=1;
        loadconditioncode(0, cond);
      end;

    constructor taicpu.op_modeflags(op: tasmop; flags: tcpumodeflags);
      begin
        inherited create(op);
        ops := 1;
        loadmodeflags(0,flags);
      end;

    constructor taicpu.op_modeflags_const(op: tasmop; flags: tcpumodeflags; a: aint);
      begin
        inherited create(op);
        ops := 2;
        loadmodeflags(0,flags);
        loadconst(1,a);
      end;

    constructor taicpu.op_specialreg_reg(op: tasmop; specialreg: tregister; specialregflags: tspecialregflags; _op2: tregister);
      begin
        inherited create(op);
        ops:=2;
        loadspecialreg(0,specialreg,specialregflags);
        loadreg(1,_op2);
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


     constructor taicpu.op_reg_reg_reg_shifterop(op : tasmop;_op1,_op2,_op3 : tregister;_op4 : tshifterop);
      begin
         inherited create(op);
         ops:=4;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
         loadshifterop(3,_op4);
      end;


    constructor taicpu.op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);
      begin
         inherited create(op);
         condition:=cond;
         ops:=1;
         loadsymbol(0,_op1,0);
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
                  ((opcode=A_MVF) and (regtype = R_FPUREGISTER)) or
                  ((opcode in [A_FCPYS, A_FCPYD]) and (regtype = R_MMREGISTER)) or
                  ((opcode in [A_VMOV]) and (regtype = R_MMREGISTER) and (oppostfix in [PF_F32,PF_F64]))
                ) and
                ((oppostfix in [PF_None,PF_D]) or (opcode = A_VMOV)) and
                (condition=C_None) and
                (ops=2) and
                (oper[0]^.typ=top_reg) and
                (oper[1]^.typ=top_reg) and
                (oper[0]^.reg=oper[1]^.reg);
      end;


    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER :
            result:=taicpu.op_reg_ref(A_LDR,r,ref);
          R_FPUREGISTER :
            { use lfm because we don't know the current internal format
              and avoid exceptions
            }
            result:=taicpu.op_reg_const_ref(A_LFM,r,1,ref);
          R_MMREGISTER :
            result:=taicpu.op_reg_ref(A_VLDR,r,ref);
          else
            internalerror(200401041);
        end;
      end;


    function spilling_create_store(r:tregister; const ref:treference):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER :
            result:=taicpu.op_reg_ref(A_STR,r,ref);
          R_FPUREGISTER :
            { use sfm because we don't know the current internal format
              and avoid exceptions
            }
            result:=taicpu.op_reg_const_ref(A_SFM,r,1,ref);
          R_MMREGISTER :
            result:=taicpu.op_reg_ref(A_VSTR,r,ref);
          else
            internalerror(200401041);
        end;
      end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        if GenerateThumbCode then
          case opcode of
            A_ADC,A_ADD,A_AND,A_BIC,
            A_EOR,A_CLZ,A_RBIT,
            A_LDR,A_LDRB,A_LDRBT,A_LDRH,A_LDRSB,
            A_LDRSH,A_LDRT,
            A_MOV,A_MVN,A_MLA,A_MUL,
            A_ORR,A_RSB,A_RSC,A_SBC,A_SUB,
            A_SWP,A_SWPB,
            A_LDF,A_FLT,A_FIX,
            A_ADF,A_DVF,A_FDV,A_FML,
            A_RFS,A_RFC,A_RDF,
            A_RMF,A_RPW,A_RSF,A_SUF,A_ABS,A_ACS,A_ASN,A_ATN,A_COS,
            A_EXP,A_LOG,A_LGN,A_MVF,A_MNF,A_FRD,A_MUF,A_POL,A_RND,A_SIN,A_SQT,A_TAN,
            A_LFM,
            A_FLDS,A_FLDD,
            A_FMRX,A_FMXR,A_FMSTAT,
            A_FMSR,A_FMRS,A_FMDRR,
            A_FCPYS,A_FCPYD,A_FCVTSD,A_FCVTDS,
            A_FABSS,A_FABSD,A_FSQRTS,A_FSQRTD,A_FMULS,A_FMULD,
            A_FADDS,A_FADDD,A_FSUBS,A_FSUBD,A_FDIVS,A_FDIVD,
            A_FMACS,A_FMACD,A_FMSCS,A_FMSCD,A_FNMACS,A_FNMACD,
            A_FNMSCS,A_FNMSCD,A_FNMULS,A_FNMULD,
            A_FMDHR,A_FMRDH,A_FMDLR,A_FMRDL,
            A_FNEGS,A_FNEGD,
            A_FSITOS,A_FSITOD,A_FTOSIS,A_FTOSID,
            A_FTOUIS,A_FTOUID,A_FUITOS,A_FUITOD,
            A_SXTB16,A_UXTB16,
            A_UXTB,A_UXTH,A_SXTB,A_SXTH,
            A_NEG,
            A_VABS,A_VADD,A_VCVT,A_VDIV,A_VLDR,A_VMOV,A_VMUL,A_VNEG,A_VSQRT,A_VSUB,
            A_MRS,A_MSR:
              if opnr=0 then
                result:=operand_readwrite
              else
                result:=operand_read;
            A_BKPT,A_B,A_BL,A_BLX,A_BX,
            A_CMN,A_CMP,A_TEQ,A_TST,
            A_CMF,A_CMFE,A_WFS,A_CNF,
            A_FCMPS,A_FCMPD,A_FCMPES,A_FCMPED,A_FCMPEZS,A_FCMPEZD,
            A_FCMPZS,A_FCMPZD,
            A_VCMP,A_VCMPE:
              result:=operand_read;
            A_SMLAL,A_UMLAL:
              if opnr in [0,1] then
                result:=operand_readwrite
              else
                result:=operand_read;
             A_SMULL,A_UMULL,
             A_FMRRD:
              if opnr in [0,1] then
                result:=operand_readwrite
              else
                result:=operand_read;
            A_STR,A_STRB,A_STRBT,
            A_STRH,A_STRT,A_STF,A_SFM,
            A_FSTS,A_FSTD,
            A_VSTR:
              { important is what happens with the involved registers }
              if opnr=0 then
                result := operand_read
              else
                { check for pre/post indexed }
                result := operand_read;
            //Thumb2
            A_LSL, A_LSR, A_ROR, A_ASR, A_SDIV, A_UDIV, A_MOVW, A_MOVT, A_MLS, A_BFI,
            A_SMMLA,A_SMMLS:
              if opnr in [0] then
                result:=operand_readwrite
              else
                result:=operand_read;
            A_BFC:
              if opnr in [0] then
                result:=operand_readwrite
              else
                result:=operand_read;
            A_LDREX:
              if opnr in [0] then
                result:=operand_readwrite
              else
                result:=operand_read;
            A_STREX:
              result:=operand_write;
            else
              internalerror(200403151);
          end
        else
          case opcode of
            A_ADC,A_ADD,A_AND,A_BIC,A_ORN,
            A_EOR,A_CLZ,A_RBIT,
            A_LDR,A_LDRB,A_LDRBT,A_LDRH,A_LDRSB,
            A_LDRSH,A_LDRT,
            A_MOV,A_MVN,A_MLA,A_MUL,
            A_ORR,A_RSB,A_RSC,A_SBC,A_SUB,
            A_SWP,A_SWPB,
            A_LDF,A_FLT,A_FIX,
            A_ADF,A_DVF,A_FDV,A_FML,
            A_RFS,A_RFC,A_RDF,
            A_RMF,A_RPW,A_RSF,A_SUF,A_ABS,A_ACS,A_ASN,A_ATN,A_COS,
            A_EXP,A_LOG,A_LGN,A_MVF,A_MNF,A_FRD,A_MUF,A_POL,A_RND,A_SIN,A_SQT,A_TAN,
            A_LFM,
            A_FLDS,A_FLDD,
            A_FMRX,A_FMXR,A_FMSTAT,
            A_FMSR,A_FMRS,A_FMDRR,
            A_FCPYS,A_FCPYD,A_FCVTSD,A_FCVTDS,
            A_FABSS,A_FABSD,A_FSQRTS,A_FSQRTD,A_FMULS,A_FMULD,
            A_FADDS,A_FADDD,A_FSUBS,A_FSUBD,A_FDIVS,A_FDIVD,
            A_FMACS,A_FMACD,A_FMSCS,A_FMSCD,A_FNMACS,A_FNMACD,
            A_FNMSCS,A_FNMSCD,A_FNMULS,A_FNMULD,
            A_FMDHR,A_FMRDH,A_FMDLR,A_FMRDL,
            A_FNEGS,A_FNEGD,
            A_FSITOS,A_FSITOD,A_FTOSIS,A_FTOSID,
            A_FTOUIS,A_FTOUID,A_FUITOS,A_FUITOD,
            A_SXTB16,A_UXTB16,
            A_UXTB,A_UXTH,A_SXTB,A_SXTH,
            A_NEG,
            A_VABS,A_VADD,A_VCVT,A_VDIV,A_VLDR,A_VMOV,A_VMUL,A_VNEG,A_VSQRT,A_VSUB,
            A_VEOR,
            A_MRS,A_MSR:
              if opnr=0 then
                result:=operand_write
              else
                result:=operand_read;
            A_BKPT,A_B,A_BL,A_BLX,A_BX,
            A_CMN,A_CMP,A_TEQ,A_TST,
            A_CMF,A_CMFE,A_WFS,A_CNF,
            A_FCMPS,A_FCMPD,A_FCMPES,A_FCMPED,A_FCMPEZS,A_FCMPEZD,
            A_FCMPZS,A_FCMPZD,
            A_VCMP,A_VCMPE:
              result:=operand_read;
            A_SMLAL,A_UMLAL:
              if opnr in [0,1] then
                result:=operand_readwrite
              else
                result:=operand_read;
             A_SMULL,A_UMULL,
             A_FMRRD:
              if opnr in [0,1] then
                result:=operand_write
              else
                result:=operand_read;
            A_STR,A_STRB,A_STRBT,
            A_STRH,A_STRT,A_STF,A_SFM,
            A_FSTS,A_FSTD,
            A_VSTR:
              { important is what happens with the involved registers }
              if opnr=0 then
                result := operand_read
              else
                { check for pre/post indexed }
                result := operand_read;
            //Thumb2
            A_LSL, A_LSR, A_ROR, A_ASR, A_SDIV, A_UDIV, A_MOVW, A_MOVT, A_MLS, A_BFI,
            A_QADD,
            A_PKHTB,A_PKHBT,
            A_SMMLA,A_SMMLS,A_SMUAD,A_SMUSD:
              if opnr in [0] then
                result:=operand_write
              else
                result:=operand_read;
            A_VFMA,A_VFMS,A_VFNMA,A_VFNMS,
            A_BFC:
              if opnr in [0] then
                result:=operand_readwrite
              else
                result:=operand_read;
            A_LDREX:
              if opnr in [0] then
                result:=operand_write
              else
                result:=operand_read;
            A_STREX:
              result:=operand_write;
            else
              internalerror(200403151);
          end;
      end;


    function taicpu.spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;
      begin
        result := operand_read;
        if (oper[opnr]^.ref^.base = reg) and
          (oper[opnr]^.ref^.addressmode in [AM_PREINDEXED,AM_POSTINDEXED]) then
           result := operand_readwrite;
      end;


    procedure BuildInsTabCache;
      var
        i : longint;
      begin
        new(instabcache);
        FillChar(instabcache^,sizeof(tinstabcache),$ff);
        i:=0;
        while (i<InsTabEntries) do
          begin
            if InsTabCache^[InsTab[i].Opcode]=-1 then
              InsTabCache^[InsTab[i].Opcode]:=i;
            inc(i);
          end;
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


    function setroundingmode(i : taicpu;rm : troundingmode) : taicpu;
      begin
        i.roundingmode:=rm;
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

      var
        limit: longint;

      { FLD/FST VFP instructions have a limit of +/- 1024, not 4096, this
        function checks the next count instructions if the limit must be
        decreased }
      procedure CheckLimit(hp : tai;count : integer);
        var
          i : Integer;
        begin
          for i:=1 to count do
            if SimpleGetNextInstruction(hp,hp) and
               (tai(hp).typ=ait_instruction) and
               ((taicpu(hp).opcode=A_FLDS) or
                (taicpu(hp).opcode=A_FLDD) or
                (taicpu(hp).opcode=A_VLDR) or
                (taicpu(hp).opcode=A_LDF) or
                (taicpu(hp).opcode=A_STF)) then
              limit:=254;
        end;

      function is_case_dispatch(hp: taicpu): boolean;
        begin
          result:=
            ((taicpu(hp).opcode in [A_ADD,A_LDR]) and
             not(GenerateThumbCode or GenerateThumb2Code) and
             (taicpu(hp).oper[0]^.typ=top_reg) and
             (taicpu(hp).oper[0]^.reg=NR_PC)) or
             ((taicpu(hp).opcode=A_MOV) and (GenerateThumbCode) and
              (taicpu(hp).oper[0]^.typ=top_reg) and
              (taicpu(hp).oper[0]^.reg=NR_PC)) or
             (taicpu(hp).opcode=A_TBH) or
             (taicpu(hp).opcode=A_TBB);
        end;

      var
        curinspos,
        penalty,
        lastinspos,
        { increased for every data element > 4 bytes inserted }
        extradataoffset,
        curop : longint;
        curtai,
        inserttai : tai;
        curdatatai,hp,hp2 : tai;
        curdata : TAsmList;
        l : tasmlabel;
        doinsert,
        removeref : boolean;
        multiplier : byte;
      begin
        curdata:=TAsmList.create;
        lastinspos:=-1;
        curinspos:=0;
        extradataoffset:=0;
        if GenerateThumbCode then
          begin
            multiplier:=2;
            limit:=504;
          end
        else
          begin
            limit:=1016;
            multiplier:=1;
          end;
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
                          if assigned(curdatatai) then
                            begin
                              { create a new copy of a data entry on arm thumb if the entry has been inserted already
                                before because arm thumb does not allow pc relative negative offsets }
                              if (GenerateThumbCode) and
                                tai_label(curdatatai).inserted then
                                begin
                                  current_asmdata.getjumplabel(l);
                                  hp:=tai_label.create(l);
                                  listtoinsert.Concat(hp);
                                  hp2:=tai(curdatatai.Next.GetCopy);
                                  hp2.Next:=nil;
                                  hp2.Previous:=nil;
                                  listtoinsert.Concat(hp2);
                                  taicpu(curtai).oper[curop]^.ref^.symboldata:=hp;
                                  taicpu(curtai).oper[curop]^.ref^.symbol:=l;
                                  curdatatai:=hp;
                                end;

                              { move only if we're at the first reference of a label }
                              if not(tai_label(curdatatai).moved) then
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
                                              inc(extradataoffset,multiplier);
                                          end;
                                        ait_realconst:
                                          begin
                                            inc(extradataoffset,multiplier*(((tai_realconst(hp).savesize-4)+3) div 4));
                                          end;
                                        else
                                          ;
                                      end;
                                      { check if the same constant has been already inserted into the currently handled list,
                                        if yes, reuse it }
                                      if (hp.typ=ait_const) then
                                        begin
                                          hp2:=tai(curdata.first);
                                          while assigned(hp2) do
                                            begin
                                              if (hp2.typ=ait_const) and (tai_const(hp2).sym=tai_const(hp).sym)
                                                and (tai_const(hp2).value=tai_const(hp).value) and (tai(hp2.previous).typ=ait_label) and
                                                { gottpoff and tlsgd symbols are PC relative, so we cannot reuse them }
                                                (not(tai_const(hp2).consttype in [aitconst_gottpoff,aitconst_tlsgd,aitconst_tlsdesc])) then
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
                    end;
                  inc(curinspos,multiplier);
                end;
              ait_align:
                begin
                  { code is always 4 byte aligned, so we don't have to take care of .align 2 which would
                    requires also incrementing curinspos by 1 }
                  inc(curinspos,(tai_align(curtai).aligntype div 4)*multiplier);
                end;
              ait_const:
                begin
                  inc(curinspos,multiplier);
                  if (tai_const(curtai).consttype=aitconst_64bit) then
                    inc(curinspos,multiplier);
                end;
              ait_realconst:
                begin
                  inc(curinspos,multiplier*((tai_realconst(hp).savesize+3) div 4));
                end;
              else
                ;
            end;
            { special case for case jump tables }
            penalty:=0;
            if SimpleGetNextInstruction(curtai,hp) and
              (tai(hp).typ=ait_instruction) then
              begin
                case taicpu(hp).opcode of
                  A_MOV,
                  A_LDR,
                  A_ADD,
                  A_TBH,
                  A_TBB:
                    { approximation if we hit a case jump table }
                    if is_case_dispatch(taicpu(hp)) then
                      begin
                        penalty:=multiplier;
                        hp:=tai(hp.next);
                        { skip register allocations and comments inserted by the optimizer as well as a label and align
                          as jump tables for thumb might have }
                        while assigned(hp) and (hp.typ in [ait_comment,ait_regalloc,ait_label,ait_align]) do
                          hp:=tai(hp.next);
                        while assigned(hp) and (hp.typ=ait_const) do
                          begin
                            inc(penalty,multiplier);
                            hp:=tai(hp.next);
                          end;
                      end;
                  A_IT:
                    begin
                      if GenerateThumb2Code then
                        penalty:=multiplier;
                        { check if the next instruction fits as well
                          or if we splitted after the it so split before }
                        CheckLimit(hp,1);
                    end;
                  A_ITE,
                  A_ITT:
                    begin
                      if GenerateThumb2Code then
                        penalty:=2*multiplier;
                        { check if the next two instructions fit as well
                          or if we splitted them so split before }
                        CheckLimit(hp,2);
                    end;
                  A_ITEE,
                  A_ITTE,
                  A_ITET,
                  A_ITTT:
                    begin
                      if GenerateThumb2Code then
                        penalty:=3*multiplier;
                        { check if the next three instructions fit as well
                          or if we splitted them so split before }
                        CheckLimit(hp,3);
                    end;
                  A_ITEEE,
                  A_ITTEE,
                  A_ITETE,
                  A_ITTTE,
                  A_ITEET,
                  A_ITTET,
                  A_ITETT,
                  A_ITTTT:
                    begin
                      if GenerateThumb2Code then
                        penalty:=4*multiplier;
                        { check if the next three instructions fit as well
                          or if we splitted them so split before }
                      CheckLimit(hp,4);
                    end;
                  else
                    ;
                end;
              end;

            CheckLimit(curtai,1);

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
              ) and
              (
                { do not insert data after a B instruction due to their limited range }
                not((GenerateThumbCode) and
                    (taicpu(curtai).opcode=A_B)
                   )
              ) then
              begin
                lastinspos:=-1;
                extradataoffset:=0;

                if GenerateThumbCode then
                  limit:=502
                else
                  limit:=1016;

                { if this is an add/tbh/tbb-based jumptable, go back to the
                  previous instruction, because inserting data between the
                  dispatch instruction and the table would mess up the
                  addresses }
                inserttai:=curtai;
                if is_case_dispatch(taicpu(inserttai)) and
                   ((taicpu(inserttai).opcode=A_ADD) or
                    (taicpu(inserttai).opcode=A_TBH) or
                    (taicpu(inserttai).opcode=A_TBB)) then
                  begin
                    repeat
                      inserttai:=tai(inserttai.previous);
                    until inserttai.typ=ait_instruction;
                    { if it's an add-based jump table, then also skip the
                      pc-relative load }
                    if taicpu(curtai).opcode=A_ADD then
                      repeat
                        inserttai:=tai(inserttai.previous);
                      until inserttai.typ=ait_instruction;
                  end
                else

                { on arm thumb, insert the data always after all labels etc. following an instruction so it
                  is prevent that a bxx yyy; bl xxx; yyyy: sequence gets separated ( we never insert on arm thumb after
                  bxx) and the distance of bxx gets too long }
                if GenerateThumbCode then
                  while assigned(tai(inserttai.Next)) and (tai(inserttai.Next).typ in SkipInstr+[ait_label]) do
                    inserttai:=tai(inserttai.next);

                doinsert:=false;
                current_asmdata.getjumplabel(l);

                { align jump in thumb .text section to 4 bytes }
                if not(curdata.empty) and (GenerateThumbCode) then
                  curdata.Insert(tai_align.Create(4));
                curdata.insert(taicpu.op_sym(A_B,l));
                curdata.concat(tai_label.create(l));

                { mark all labels as inserted, arm thumb
                  needs this, so data referencing an already inserted label can be
                  duplicated because arm thumb does not allow negative pc relative offset }
                hp2:=tai(curdata.first);
                while assigned(hp2) do
                  begin
                    if hp2.typ=ait_label then
                      tai_label(hp2).inserted:=true;
                    hp2:=tai(hp2.next);
                  end;

                { continue with the last inserted label because we use later
                  on SimpleGetNextInstruction, so if we used curtai.next (which
                  is then equal curdata.last.previous) we could over see one
                  instruction }
                hp:=tai(curdata.Last);
                list.insertlistafter(inserttai,curdata);
                curtai:=hp;
              end
            else
              curtai:=tai(curtai.next);
          end;
        { align jump in thumb .text section to 4 bytes }
        if not(curdata.empty) and (GenerateThumbCode or GenerateThumb2Code) then
          curdata.Insert(tai_align.Create(4));
        list.concatlist(curdata);
        curdata.free;
      end;


    procedure ensurethumb2encodings(list: TAsmList);
      var
        curtai: tai;
        op2reg: TRegister;
      begin
        { Do Thumb-2 16bit -> 32bit transformations }
        curtai:=tai(list.first);
        while assigned(curtai) do
          begin
            case curtai.typ of
              ait_instruction:
                begin
                  case taicpu(curtai).opcode of
                    A_ADD:
                      begin
                        { Set wide flag for ADD Rd,Rn,Rm where registers are over R7(high register set) }
                        if taicpu(curtai).ops = 3 then
                          begin
                            if taicpu(curtai).oper[2]^.typ in [top_reg,top_shifterop] then
                              begin
                                if taicpu(curtai).oper[2]^.typ = top_reg then
                                  op2reg := taicpu(curtai).oper[2]^.reg
                                else if taicpu(curtai).oper[2]^.shifterop^.rs <> NR_NO then
                                  op2reg := taicpu(curtai).oper[2]^.shifterop^.rs
                                else
                                  op2reg := NR_NO;

                                if op2reg <> NR_NO then
                                  begin
                                    if (taicpu(curtai).oper[0]^.reg >= NR_R8) or
                                       (taicpu(curtai).oper[1]^.reg >= NR_R8) or
                                       (op2reg >= NR_R8) then
                                      begin
                                        taicpu(curtai).wideformat:=true;

                                        { Handle special cases where register rules are violated by optimizer/user }
                                        { if d == 13 || (d == 15 && S == ‚Äò0‚Äô) || n == 15 || m IN [13,15] then UNPREDICTABLE; }

                                        { Transform ADD.W Rx, Ry, R13 into ADD.W Rx, R13, Ry }
                                        if (op2reg = NR_R13) and (taicpu(curtai).oper[2]^.typ = top_reg) then
                                          begin
                                            taicpu(curtai).oper[2]^.reg := taicpu(curtai).oper[1]^.reg;
                                            taicpu(curtai).oper[1]^.reg := op2reg;
                                          end;
                                      end;
                                  end;
                              end;
                          end;
                      end;
                    else;
                  end;
                end;
              else
                ;
            end;

            curtai:=tai(curtai.Next);
          end;
      end;


    procedure ensurethumbencodings(list: TAsmList);
      var
        curtai: tai;
      begin
        { Do Thumb 16bit transformations to form valid instruction forms }
        curtai:=tai(list.first);
        while assigned(curtai) do
          begin
            case curtai.typ of
              ait_instruction:
                begin
                  case taicpu(curtai).opcode of
                    A_STM:
                      begin
                        if (taicpu(curtai).ops=2) and
                           (taicpu(curtai).oper[0]^.typ=top_ref) and
                           (taicpu(curtai).oper[0]^.ref^.index=NR_STACK_POINTER_REG) and
                           (taicpu(curtai).oper[0]^.ref^.addressmode=AM_PREINDEXED) and
                           (taicpu(curtai).oppostfix in [PF_FD,PF_DB]) then
                          begin
                            taicpu(curtai).oppostfix:=PF_None;
                            taicpu(curtai).loadregset(0, taicpu(curtai).oper[1]^.regtyp, taicpu(curtai).oper[1]^.subreg, taicpu(curtai).oper[1]^.regset^);
                            taicpu(curtai).ops:=1;
                            taicpu(curtai).opcode:=A_PUSH;
                          end;
                      end;

                    A_LDM:
                      begin
                        if (taicpu(curtai).ops=2) and
                           (taicpu(curtai).oper[0]^.typ=top_ref) and
                           (taicpu(curtai).oper[0]^.ref^.index=NR_STACK_POINTER_REG) and
                           (taicpu(curtai).oper[0]^.ref^.addressmode=AM_PREINDEXED) and
                           (taicpu(curtai).oppostfix in [PF_FD,PF_IA]) then
                          begin
                            taicpu(curtai).oppostfix:=PF_None;
                            taicpu(curtai).loadregset(0, taicpu(curtai).oper[1]^.regtyp, taicpu(curtai).oper[1]^.subreg, taicpu(curtai).oper[1]^.regset^);
                            taicpu(curtai).ops:=1;
                            taicpu(curtai).opcode:=A_POP;
                          end;
                      end;

                    A_ADD,
                    A_AND,A_EOR,A_ORR,A_BIC,
                    A_LSL,A_LSR,A_ASR,A_ROR,
                    A_ADC,A_SBC:
                      begin
                        if (taicpu(curtai).ops = 3) and
                           (taicpu(curtai).oper[2]^.typ=top_reg) and
                           (taicpu(curtai).oper[0]^.reg=taicpu(curtai).oper[1]^.reg) and
                           (taicpu(curtai).oper[0]^.reg<>NR_STACK_POINTER_REG) then
                          begin
                            taicpu(curtai).oper[1]^.reg:=taicpu(curtai).oper[2]^.reg;
                            taicpu(curtai).ops:=2;
                          end;
                      end;
                    else
                      ;
                  end;
                end;
              else
                ;
            end;

            curtai:=tai(curtai.Next);
          end;
      end;


    function getMergedInstruction(FirstOp,LastOp:TAsmOp;InvertLast:boolean) : TAsmOp;
      const
        opTable: array[A_IT..A_ITTTT] of string =
          ('T','TE','TT','TEE','TTE','TET','TTT',
           'TEEE','TTEE','TETE','TTTE',
           'TEET','TTET','TETT','TTTT');
        invertedOpTable: array[A_IT..A_ITTTT] of string =
          ('E','ET','EE','ETT','EET','ETE','EEE',
           'ETTT','EETT','ETET','EEET',
           'ETTE','EETE','ETEE','EEEE');
      var
        resStr : string;
        i : TAsmOp;
      begin
        if InvertLast then
          resStr := opTable[FirstOp]+invertedOpTable[LastOp]
        else
          resStr := opTable[FirstOp]+opTable[LastOp];
        if length(resStr) > 4 then
          internalerror(2012100805);

        for i := low(opTable) to high(opTable) do
          if opTable[i] = resStr then
            exit(i);

        internalerror(2012100806);
      end;

    procedure foldITInstructions(list: TAsmList);
      var
        curtai,hp1 : tai;
        levels,i : LongInt;
      begin
        curtai:=tai(list.First);
        while assigned(curtai) do
          begin
            case curtai.typ of
              ait_instruction:
                begin
                  if IsIT(taicpu(curtai).opcode) then
                    begin
                      levels := GetITLevels(taicpu(curtai).opcode);
                      if levels < 4 then
                        begin
                          i:=levels;
                          hp1:=tai(curtai.Next);
                          while assigned(hp1) and
                            (i > 0) do
                            begin
                              if hp1.typ=ait_instruction then
                                begin
                                  dec(i);
                                  if (i = 0) and
                                    mustbelast(hp1) then
                                    begin
                                      hp1:=nil;
                                      break;
                                    end;
                                end;
                              hp1:=tai(hp1.Next);
                            end;

                          if assigned(hp1) then
                            begin
                              // We are pointing at the first instruction after the IT block
                              while assigned(hp1) and
                                (hp1.typ<>ait_instruction) do
                                  hp1:=tai(hp1.Next);

                              if assigned(hp1) and
                                (hp1.typ=ait_instruction) and
                                IsIT(taicpu(hp1).opcode) then
                                begin
                                  if (levels+GetITLevels(taicpu(hp1).opcode) <= 4) and
                                    ((taicpu(curtai).oper[0]^.cc=taicpu(hp1).oper[0]^.cc) or
                                     (taicpu(curtai).oper[0]^.cc=inverse_cond(taicpu(hp1).oper[0]^.cc))) then
                                    begin
                                      taicpu(curtai).opcode:=getMergedInstruction(taicpu(curtai).opcode,
                                                                                  taicpu(hp1).opcode,
                                                                                  taicpu(curtai).oper[0]^.cc=inverse_cond(taicpu(hp1).oper[0]^.cc));

                                      list.Remove(hp1);
                                      hp1.Free;
                                    end;
                                end;
                            end;
                        end;
                    end;
                end
              else
                ;
            end;

            curtai:=tai(curtai.Next);
          end;
      end;

    procedure fix_invalid_imms(list: TAsmList);
      var
        curtai: tai;
        sh: byte;
      begin
        curtai:=tai(list.First);
        while assigned(curtai) do
          begin
            case curtai.typ of
              ait_instruction:
                begin
                  if (taicpu(curtai).opcode in [A_AND,A_BIC]) and
                     (taicpu(curtai).ops=3) and
                     (taicpu(curtai).oper[2]^.typ=top_const) and
                     (not is_shifter_const(taicpu(curtai).oper[2]^.val,sh)) and
                     is_shifter_const((not taicpu(curtai).oper[2]^.val) and $FFFFFFFF,sh) then
                    begin
                      case taicpu(curtai).opcode of
                        A_AND: taicpu(curtai).opcode:=A_BIC;
                        A_BIC: taicpu(curtai).opcode:=A_AND;
                        else
                          internalerror(2019050931);
                      end;
                      taicpu(curtai).oper[2]^.val:=(not taicpu(curtai).oper[2]^.val) and $FFFFFFFF;
                    end
                  else if (taicpu(curtai).opcode in [A_SUB,A_ADD]) and
                     (taicpu(curtai).ops=3) and
                     (taicpu(curtai).oper[2]^.typ=top_const) and
                     (not is_shifter_const(taicpu(curtai).oper[2]^.val,sh)) and
                     is_shifter_const(-taicpu(curtai).oper[2]^.val,sh) then
                    begin
                      case taicpu(curtai).opcode of
                        A_ADD: taicpu(curtai).opcode:=A_SUB;
                        A_SUB: taicpu(curtai).opcode:=A_ADD;
                        else
                          internalerror(2019050930);
                      end;
                      taicpu(curtai).oper[2]^.val:=-taicpu(curtai).oper[2]^.val;
                    end;
                end;
              else
                ;
            end;

            curtai:=tai(curtai.Next);
          end;
      end;


    procedure gather_it_info(list: TAsmList);
      var
        curtai: tai;
        in_it: boolean;
        it_count: longint;
      begin
        in_it:=false;
        it_count:=0;

        curtai:=tai(list.First);
        while assigned(curtai) do
          begin
            case curtai.typ of
              ait_instruction:
                begin
                  case taicpu(curtai).opcode of
                    A_IT..A_ITTTT:
                      begin
                        if in_it then
                          Message1(asmw_e_invalid_opcode_and_operands, 'ITxx instruction is inside another ITxx instruction')
                        else
                          begin
                            in_it:=true;
                            it_count:=GetITLevels(taicpu(curtai).opcode);
                          end;
                      end;
                    else
                      begin
                        taicpu(curtai).inIT:=in_it;
                        taicpu(curtai).lastinIT:=in_it and (it_count=1);

                        if in_it then
                          begin
                            dec(it_count);
                            if it_count <= 0 then
                              in_it:=false;
                          end;
                      end;
                  end;
                end;
              else
                ;
            end;

            curtai:=tai(curtai.Next);
          end;
      end;


    { Expands pseudo instructions ( mov r1,r2,lsl #4 -> lsl r1,r2,#4) }
    procedure expand_instructions(list: TAsmList);
      var
        curtai: tai;
      begin
        curtai:=tai(list.First);
        while assigned(curtai) do
          begin
            case curtai.typ of
              ait_instruction:
                begin
                  case taicpu(curtai).opcode of
                    A_MOV:
                      begin
                        if (taicpu(curtai).ops=3) and
                           (taicpu(curtai).oper[2]^.typ=top_shifterop) then
                          begin
                            case taicpu(curtai).oper[2]^.shifterop^.shiftmode of
                              SM_NONE: ;
                              SM_LSL: taicpu(curtai).opcode:=A_LSL;
                              SM_LSR: taicpu(curtai).opcode:=A_LSR;
                              SM_ASR: taicpu(curtai).opcode:=A_ASR;
                              SM_ROR: taicpu(curtai).opcode:=A_ROR;
                              SM_RRX: taicpu(curtai).opcode:=A_RRX;
                            end;

                            if taicpu(curtai).oper[2]^.shifterop^.shiftmode=SM_RRX then
                              taicpu(curtai).ops:=2;

                            if taicpu(curtai).oper[2]^.shifterop^.rs=NR_NO then
                              taicpu(curtai).loadconst(2, taicpu(curtai).oper[2]^.shifterop^.shiftimm)
                            else
                              taicpu(curtai).loadreg(2, taicpu(curtai).oper[2]^.shifterop^.rs);
                          end;
                      end;
                    A_NEG:
                      begin
                        taicpu(curtai).opcode:=A_RSB;
                        taicpu(curtai).oppostfix:=PF_S; // NEG should always set flags (according to documentation NEG<c> = RSBS<c>)

                        if taicpu(curtai).ops=2 then
                          begin
                            taicpu(curtai).loadconst(2,0);
                            taicpu(curtai).ops:=3;
                          end
                        else
                          begin
                            taicpu(curtai).loadconst(1,0);
                            taicpu(curtai).ops:=2;
                          end;
                      end;
                    A_SWI:
                      begin
                        taicpu(curtai).opcode:=A_SVC;
                      end;
                    else
                      ;
                  end;
                end;
              else
                ;
            end;

            curtai:=tai(curtai.Next);
          end;
      end;


    procedure finalizearmcode(list, listtoinsert: TAsmList);
      begin
        { Don't expand pseudo instructions when using GAS, it breaks on some thumb instructions }
        if target_asm.id<>as_gas then
          expand_instructions(list);

        { Do Thumb-2 16bit -> 32bit transformations }
        if GenerateThumb2Code then
          begin
            ensurethumbencodings(list);
            ensurethumb2encodings(list);
            foldITInstructions(list);
          end
        else if GenerateThumbCode then
          ensurethumbencodings(list);

        gather_it_info(list);

        fix_invalid_imms(list);

        insertpcrelativedata(list, listtoinsert);
      end;

    procedure InsertPData;
      var
        prolog: TAsmList;
      begin
        prolog:=TAsmList.create;
        new_section(prolog,sec_code,'FPC_EH_PROLOG',sizeof(pint),secorder_begin);
        prolog.concat(Tai_const.Createname('_ARM_ExceptionHandler', 0));
        prolog.concat(Tai_const.Create_32bit(0));
        prolog.concat(Tai_symbol.Createname_global('FPC_EH_CODE_START',AT_METADATA,0,voidpointertype));
        { dummy function }
        prolog.concat(taicpu.op_reg_reg(A_MOV,NR_R15,NR_R14));
        current_asmdata.asmlists[al_start].insertList(prolog);
        prolog.Free;
        new_section(current_asmdata.asmlists[al_end],sec_pdata,'',sizeof(pint));
        current_asmdata.asmlists[al_end].concat(Tai_const.Createname('FPC_EH_CODE_START', 0));
        current_asmdata.asmlists[al_end].concat(Tai_const.Create_32bit(longint($ffffff01)));
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
                 if (ot and OT_REGS)=OT_REGS then
                  s:=s+'sreg'
               else
                 if (ot and OT_REGF)=OT_REGF then
                  s:=s+'creg'
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
                     s:=s+' am2 '
                   else if (ot and OT_AM6)<>0 then
                     s:=s+' am2 ';
                 end
               else
                 if (ot and OT_SHIFTEROP)=OT_SHIFTEROP then
                  begin
                    s:=s+'shifterop';
                    addsize:=false;
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
        InsEntry:=nil;
        InsSize:=0;
        LastInsOffset:=-1;
      end;


    procedure taicpu.ResetPass2;
      begin
        { we are here in a second pass, check if the instruction can be optimized }
        if assigned(InsEntry) and
           ((InsEntry^.flags and IF_PASS2)<>0) then
         begin
           InsEntry:=nil;
           InsSize:=0;
         end;
        LastInsOffset:=-1;
      end;


    function taicpu.CheckIfValid:boolean;
      begin
        Result:=False; { unimplemented }
      end;


    function taicpu.Pass1(objdata:TObjData):longint;
      var
        ldr2op : array[PF_B..PF_T] of tasmop = (
          A_LDRB,A_LDRSB,A_LDRBT,A_LDRH,A_LDRSH,A_LDRT);
        str2op : array[PF_B..PF_T] of tasmop = (
          A_STRB,A_None,A_STRBT,A_STRH,A_None,A_STRT);
      begin
        Pass1:=0;
        { Save the old offset and set the new offset }
        InsOffset:=ObjData.CurrObjSec.Size;
        { Error? }
        if (Insentry=nil) and (InsSize=-1) then
          exit;
        { set the file postion }
        current_filepos:=fileinfo;

        { tranlate LDR+postfix to complete opcode }
        if (opcode=A_LDR) and (oppostfix=PF_D) then
          begin
            opcode:=A_LDRD;
            oppostfix:=PF_None;
          end
        else if (opcode=A_LDR) and (oppostfix<>PF_None) then
          begin
            if (oppostfix in [low(ldr2op)..high(ldr2op)]) then
              opcode:=ldr2op[oppostfix]
            else
              internalerror(2005091001);
            if opcode=A_None then
              internalerror(2005091004);
            { postfix has been added to opcode }
            oppostfix:=PF_None;
          end
        else if (opcode=A_STR) and (oppostfix=PF_D) then
          begin
            opcode:=A_STRD;
            oppostfix:=PF_None;
          end
        else if (opcode=A_STR) and (oppostfix<>PF_None) then
          begin
            if (oppostfix in [low(str2op)..high(str2op)]) then
              opcode:=str2op[oppostfix]
            else
              internalerror(2005091002);
            if opcode=A_None then
              internalerror(2005091003);
            { postfix has been added to opcode }
            oppostfix:=PF_None;
          end;

        { Get InsEntry }
        if FindInsEntry(objdata) then
         begin
           InsSize:=4;

           if insentry^.code[0] in [#$60..#$6C] then
             InsSize:=2;

           LastInsOffset:=InsOffset;
           Pass1:=InsSize;
           exit;
         end;
        LastInsOffset:=-1;
      end;


    procedure taicpu.Pass2(objdata:TObjData);
      begin
        { error in pass1 ? }
        if insentry=nil then
         exit;
        current_filepos:=fileinfo;
        { Generate the instruction }
        GenCode(objdata);
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


    procedure taicpu.BuildArmMasks(objdata:TObjData);
      const
        Masks: array[tcputype] of longint =
          (
            IF_NONE,
            IF_ARMv4,
            IF_ARMv4,
            IF_ARMv4T or IF_ARMv4,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6 or IF_ARMv6K,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6 or IF_ARMv6K or IF_ARMv6T2,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6 or IF_ARMv6K or IF_ARMv6T2 or IF_ARMv6Z,
            IF_ARMv4T or IF_ARMv5T or IF_ARMv6M,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6 or IF_ARMv6K or IF_ARMv6T2 or IF_ARMv6Z or IF_ARMv7,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6 or IF_ARMv6K or IF_ARMv6T2 or IF_ARMv6Z or IF_ARMv7 or IF_ARMv7A,
            IF_ARMv4T or IF_ARMv4 or IF_ARMv5 or IF_ARMv5T or IF_ARMv5TE or IF_ARMv5TEJ or IF_armv6 or IF_ARMv6K or IF_ARMv6T2 or IF_ARMv6Z or IF_ARMv7 or IF_ARMv7A or IF_ARMv7R,
            IF_ARMv4T or IF_ARMv5T or IF_ARMv6T2 or IF_ARMv7M,
            IF_ARMv4T or IF_ARMv5T or IF_ARMv6T2 or IF_ARMv7M or IF_ARMv7EM
          );

        FPUMasks: array[tfputype] of longword =
          (
            { fpu_none       } IF_NONE,
            { fpu_soft       } IF_NONE,
            { fpu_libgcc     } IF_NONE,
            { fpu_fpa        } IF_FPA,
            { fpu_fpa10      } IF_FPA,
            { fpu_fpa11      } IF_FPA,
            { fpu_vfpv2      } IF_VFPv2,
            { fpu_vfpv3      } IF_VFPv2 or IF_VFPv3,
            { fpu_neon_vfpv3 } IF_VFPv2 or IF_VFPv3 or IF_NEON,
            { fpu_vfpv3_d16  } IF_VFPv2 or IF_VFPv3,
            { fpu_fpv4_s16   } IF_NONE,
            { fpu_vfpv4      } IF_VFPv2 or IF_VFPv3 or IF_VFPv4,
            { fpu_neon_vfpv4 } IF_VFPv2 or IF_VFPv3 or IF_VFPv4 or IF_NEON
          );
      begin
        fArmVMask:=Masks[current_settings.cputype] or FPUMasks[current_settings.fputype];

        if objdata.ThumbFunc then
        //if current_settings.instructionset=is_thumb then
          begin
            fArmMask:=IF_THUMB;
            if CPUARM_HAS_THUMB2 in cpu_capabilities[current_settings.cputype] then
             fArmMask:=fArmMask or IF_THUMB32;
          end
        else
          fArmMask:=IF_ARM32;
      end;


    function  taicpu.InsEnd:longint;
      begin
        Result:=0; { unimplemented }
      end;


    procedure taicpu.create_ot(objdata:TObjData);
      var
        i,l,relsize : longint;
        dummy : byte;
        currsym : TObjSymbol;
      begin
        if ops=0 then
         exit;
        { update oper[].ot field }
        for i:=0 to ops-1 do
         with oper[i]^ do
          begin
            case typ of
              top_regset:
                begin
                  ot:=OT_REGLIST;
                end;
              top_reg :
                begin
                  case getregtype(reg) of
                    R_INTREGISTER:
                      begin
                        ot:=OT_REG32 or OT_SHIFTEROP;
                        if getsupreg(reg)<8 then
                          ot:=ot or OT_REGLO
                        else if reg=NR_STACK_POINTER_REG then
                          ot:=ot or OT_REGSP;
                      end;
                    R_FPUREGISTER:
                      ot:=OT_FPUREG;
                    R_MMREGISTER:
                      ot:=OT_VREG;
                    R_SPECIALREGISTER:
                      ot:=OT_REGF;
                    else
                      internalerror(2005090901);
                  end;
                end;
              top_ref :
                begin
                  if ref^.refaddr=addr_no then
                    begin
                      { create ot field }
                      { we should get the size here dependend on the
                        instruction }
                      if (ot and OT_SIZE_MASK)=0 then
                        ot:=OT_MEMORY or OT_BITS32
                      else
                        ot:=OT_MEMORY or (ot and OT_SIZE_MASK);
                      if (ref^.base=NR_NO) and (ref^.index=NR_NO) then
                        ot:=ot or OT_MEM_OFFS;
                      { if we need to fix a reference, we do it here }

                      { pc relative addressing }
                      if (ref^.base=NR_NO) and
                        (ref^.index=NR_NO) and
                        (ref^.shiftmode=SM_None)
                        { at least we should check if the destination symbol
                          is in a text section }
                        { and
                        (ref^.symbol^.owner="text") } then
                        ref^.base:=NR_PC;

                      { determine possible address modes }
                      if GenerateThumbCode or
                         GenerateThumb2Code then
                        begin
                          if (ref^.addressmode<>AM_OFFSET) then
                            ot:=ot or OT_AM2
                          else if (ref^.base=NR_PC) then
                            ot:=ot or OT_AM6
                          else if (ref^.base=NR_STACK_POINTER_REG) then
                            ot:=ot or OT_AM5
                          else if ref^.index=NR_NO then
                            ot:=ot or OT_AM4
                          else
                            ot:=ot or OT_AM3;
                        end;

                      if (ref^.base<>NR_NO) and
                        (opcode in [A_LDREX,A_LDREXB,A_LDREXH,A_LDREXD,
                                    A_STREX,A_STREXB,A_STREXH,A_STREXD]) and
                        (
                          (ref^.addressmode=AM_OFFSET) and
                          (ref^.index=NR_NO) and
                          (ref^.shiftmode=SM_None) and
                          (ref^.offset=0)
                        ) then
                        ot:=ot or OT_AM6
                      else if (ref^.base<>NR_NO) and
                        (
                          (
                            (ref^.index=NR_NO) and
                            (ref^.shiftmode=SM_None) and
                            (ref^.offset>=-4097) and
                            (ref^.offset<=4097)
                          ) or
                          (
                            (ref^.shiftmode=SM_None) and
                            (ref^.offset=0)
                          ) or
                          (
                            (ref^.index<>NR_NO) and
                            (ref^.shiftmode<>SM_None) and
                            (ref^.shiftimm<=32) and
                            (ref^.offset=0)
                          )
                        ) then
                        ot:=ot or OT_AM2;

                      if (ref^.index<>NR_NO) and
                        (oppostfix in [PF_None,PF_IA,PF_IB,PF_DA,PF_DB,PF_FD,PF_FA,PF_ED,PF_EA,
                                       PF_IAD,PF_DBD,PF_FDD,PF_EAD,
                                       PF_IAS,PF_DBS,PF_FDS,PF_EAS,
                                       PF_IAX,PF_DBX,PF_FDX,PF_EAX]) and
                        (
                          (ref^.base=NR_NO) and
                          (ref^.shiftmode=SM_None) and
                          (ref^.offset=0)
                        ) then
                        ot:=ot or OT_AM4;

                    end
                  else
                    begin
                      l:=ref^.offset;
                      currsym:=ObjData.symbolref(ref^.symbol);
                      if assigned(currsym) then
                        inc(l,currsym.address);
                      relsize:=(InsOffset+2)-l;
                      if (relsize<-33554428) or (relsize>33554428) then
                       ot:=OT_IMM32
                      else
                       ot:=OT_IMM24;
                    end;
                end;
              top_local :
                begin
                  { we should get the size here dependend on the
                    instruction }
                  if (ot and OT_SIZE_MASK)=0 then
                    ot:=OT_MEMORY or OT_BITS32
                  else
                    ot:=OT_MEMORY or (ot and OT_SIZE_MASK);
                end;
              top_const :
                begin
                  ot:=OT_IMMEDIATE;
                  if (val=0) then
                    ot:=ot_immediatezero
                  else if is_shifter_const(val,dummy) then
                    ot:=OT_IMMSHIFTER
                  else if GenerateThumb2Code and is_thumb32_imm(val) then
                    ot:=OT_IMMSHIFTER
                  else
                    ot:=OT_IMM32
                end;
              top_none :
                begin
                  { generated when there was an error in the
                    assembler reader. It never happends when generating
                    assembler }
                end;
              top_shifterop:
                begin
                  ot:=OT_SHIFTEROP;
                end;
              top_conditioncode:
                begin
                  ot:=OT_CONDITION;
                end;
              top_specialreg:
                begin
                  ot:=OT_REGS;
                end;
              top_modeflags:
                begin
                  ot:=OT_MODEFLAGS;
                end;
              top_realconst:
                begin
                  ot:=OT_IMMEDIATEMM;
                end;
              else
                internalerror(2004022623);
            end;
          end;
      end;


    function taicpu.Matches(p:PInsEntry):longint;
      { * IF_SM stands for Size Match: any operand whose size is not
       * explicitly specified by the template is `really' intended to be
       * the same size as the first size-specified operand.
       * Non-specification is tolerated in the input instruction, but
       * _wrong_ specification is not.
       *
       * IF_SM2 invokes Size Match on only the first _two_ operands, for
       * three-operand instructions such as SHLD: it implies that the
       * first two operands must match in size, but that the third is
       * required to be _unspecified_.
       *
       * IF_SB invokes Size Byte: operands with unspecified size in the
       * template are really bytes, and so no non-byte specification in
       * the input instruction will be tolerated. IF_SW similarly invokes
       * Size Word, and IF_SD invokes Size Doubleword.
       *
       * (The default state if neither IF_SM nor IF_SM2 is specified is
       * that any operand with unspecified size in the template is
       * required to have unspecified size in the instruction too...)
      }
      var
        i{,j,asize,oprs} : longint;
        {siz : array[0..3] of longint;}
      begin
        Matches:=100;

        { Check the opcode and operands }
        if (p^.opcode<>opcode) or (p^.ops<>ops) then
         begin
           Matches:=0;
           exit;
         end;

        { check ARM instruction version }
        if (p^.flags and fArmVMask)=0 then
          begin
            Matches:=0;
            exit;
          end;

        { check ARM instruction type }
        if (p^.flags and fArmMask)=0 then
          begin
            Matches:=0;
            exit;
          end;

        { Check wideformat flag }
        if wideformat and ((p^.flags and IF_WIDE)=0) then
          begin
            matches:=0;
            exit;
          end;

        { Check that no spurious colons or TOs are present }
        for i:=0 to p^.ops-1 do
         if (oper[i]^.ot and (not p^.optypes[i]) and (OT_COLON or OT_TO))<>0 then
          begin
            Matches:=0;
            exit;
          end;

        { Check that the operand flags all match up }
        for i:=0 to p^.ops-1 do
         begin
           if ((p^.optypes[i] and (not oper[i]^.ot)) or
               ((p^.optypes[i] and OT_SIZE_MASK) and
                ((p^.optypes[i] xor oper[i]^.ot) and OT_SIZE_MASK)))<>0 then
            begin
              if ((p^.optypes[i] and (not oper[i]^.ot) and OT_NON_SIZE) or
                  (oper[i]^.ot and OT_SIZE_MASK))<>0 then
               begin
                 Matches:=0;
                 exit;
               end
              else if ((p^.optypes[i] and OT_OPT_SIZE)<>0) and
                       ((p^.optypes[i] and OT_OPT_SIZE)<>(oper[i]^.ot and OT_OPT_SIZE)) then
               begin
                 Matches:=0;
                 exit;
               end
              else
               Matches:=1;
            end;
         end;

      { check postfixes:
        the existance of a certain postfix requires a
        particular code }

        { update condition flags
          or floating point single }
      if (oppostfix=PF_S) and
        not(p^.code[0] in [#$04..#$0F,#$14..#$16,#$29,#$30,#$60..#$6B,#$80..#$82,#$A0..#$A2,#$44,#$94,#$42,#$92]) then
        begin
          Matches:=0;
          exit;
        end;

      { floating point size }
      if (oppostfix in [PF_D,PF_E,PF_P,PF_EP]) and
        not(p^.code[0] in [
          // FPA
          #$A0..#$A2,
          // old-school VFP
          #$42,#$92,
          // vldm/vstm
          #$44,#$94]) then
        begin
          Matches:=0;
          exit;
        end;

      { multiple load/store address modes }
      if (oppostfix in [PF_IA,PF_IB,PF_DA,PF_DB,PF_FD,PF_FA,PF_ED,PF_EA]) and
        not(p^.code[0] in [
          // ldr,str,ldrb,strb
          #$17,
          // stm,ldm
          #$26,#$69,#$8C,
          // vldm/vstm
          #$44,#$94
        ]) then
        begin
          Matches:=0;
          exit;
        end;

      { we shouldn't see any opsize prefixes here }
      if (oppostfix in [PF_B,PF_SB,PF_BT,PF_H,PF_SH,PF_T]) then
        begin
          Matches:=0;
          exit;
        end;

      if (roundingmode<>RM_None) and not(p^.code[0] in []) then
        begin
          Matches:=0;
          exit;
        end;

      { Check thumb flags }
      if p^.code[0] in [#$60..#$61] then
        begin
          if (p^.code[0]=#$60) and
             (GenerateThumb2Code and
              ((not inIT) and (oppostfix<>PF_S)) or
              (inIT and (condition=C_None))) then
            begin
              Matches:=0;
              exit;
            end
          else if (p^.code[0]=#$61) and
             (oppostfix=PF_S) then
            begin
              Matches:=0;
              exit;
            end;
        end
      else if p^.code[0]=#$62 then
        begin
          if (GenerateThumb2Code and
              (condition<>C_None) and
              (not inIT) and
              (not lastinIT)) then
            begin
              Matches:=0;
              exit;
            end;
        end
      else if p^.code[0]=#$63 then
        begin
          if inIT then
            begin
              Matches:=0;
              exit;
            end;
        end
      else if p^.code[0]=#$64 then
        begin
          if (opcode=A_MUL) then
            begin
              if (ops=3) and
                 ((oper[2]^.typ<>top_reg) or
                  (oper[0]^.reg<>oper[2]^.reg)) then
                begin
                  matches:=0;
                  exit;
                end;
            end;
        end
      else if p^.code[0]=#$6B then
        begin
          if inIT or
             (oppostfix<>PF_S) then
            begin
              Matches:=0;
              exit;
            end;
        end;

      { Check operand sizes }
        { as default an untyped size can get all the sizes, this is different
          from nasm, but else we need to do a lot checking which opcodes want
          size or not with the automatic size generation }
        (*
        asize:=longint($ffffffff);
        if (p^.flags and IF_SB)<>0 then
          asize:=OT_BITS8
        else if (p^.flags and IF_SW)<>0 then
          asize:=OT_BITS16
        else if (p^.flags and IF_SD)<>0 then
          asize:=OT_BITS32;
        if (p^.flags and IF_ARMASK)<>0 then
         begin
           siz[0]:=0;
           siz[1]:=0;
           siz[2]:=0;
           if (p^.flags and IF_AR0)<>0 then
            siz[0]:=asize
           else if (p^.flags and IF_AR1)<>0 then
            siz[1]:=asize
           else if (p^.flags and IF_AR2)<>0 then
            siz[2]:=asize;
         end
        else
         begin
         { we can leave because the size for all operands is forced to be
           the same
           but not if IF_SB IF_SW or IF_SD is set PM }
           if asize=-1 then
             exit;
           siz[0]:=asize;
           siz[1]:=asize;
           siz[2]:=asize;
         end;

        if (p^.flags and (IF_SM or IF_SM2))<>0 then
         begin
           if (p^.flags and IF_SM2)<>0 then
            oprs:=2
           else
            oprs:=p^.ops;
           for i:=0 to oprs-1 do
            if ((p^.optypes[i] and OT_SIZE_MASK) <> 0) then
             begin
               for j:=0 to oprs-1 do
                siz[j]:=p^.optypes[i] and OT_SIZE_MASK;
               break;
             end;
          end
         else
          oprs:=2;

        { Check operand sizes }
        for i:=0 to p^.ops-1 do
         begin
           if ((p^.optypes[i] and OT_SIZE_MASK)=0) and
              ((oper[i]^.ot and OT_SIZE_MASK and (not siz[i]))<>0) and
              { Immediates can always include smaller size }
              ((oper[i]^.ot and OT_IMMEDIATE)=0) and
               (((p^.optypes[i] and OT_SIZE_MASK) or siz[i])<(oper[i]^.ot and OT_SIZE_MASK)) then
            Matches:=2;
         end;
        *)
      end;


    function  taicpu.calcsize(p:PInsEntry):shortint;
      begin
        result:=4;
      end;


    function  taicpu.NeedAddrPrefix(opidx:byte):boolean;
      begin
        Result:=False; { unimplemented }
      end;


    procedure taicpu.Swapoperands;
      begin
      end;


    function taicpu.FindInsentry(objdata:TObjData):boolean;
      var
        i : longint;
      begin
        result:=false;
      { Things which may only be done once, not when a second pass is done to
        optimize }
        if (Insentry=nil) or ((InsEntry^.flags and IF_PASS2)<>0) then
         begin
           { create the .ot fields }
           create_ot(objdata);

           BuildArmMasks(objdata);
           { set the file postion }
           current_filepos:=fileinfo;
         end
        else
         begin
           { we've already an insentry so it's valid }
           result:=true;
           exit;
         end;
        { Lookup opcode in the table }
        InsSize:=-1;
        i:=instabcache^[opcode];
        if i=-1 then
         begin
           Message1(asmw_e_opcode_not_in_table,gas_op2str[opcode]);
           exit;
         end;
        insentry:=@instab[i];
        while (insentry^.opcode=opcode) do
         begin
           if matches(insentry)=100 then
             begin
               result:=true;
               exit;
             end;
           inc(i);
           insentry:=@instab[i];
         end;
        Message1(asmw_e_invalid_opcode_and_operands,GetString);
        { No instruction found, set insentry to nil and inssize to -1 }
        insentry:=nil;
        inssize:=-1;
      end;


    procedure taicpu.gencode(objdata:TObjData);
      const
        CondVal : array[TAsmCond] of byte=(
         $E, $0, $1, $2, $3, $4, $5, $6, $7, $8, $9, $A,
         $B, $C, $D, $E, 0);
      var
        bytes, rd, rm, rn, d, m, n : dword;
        bytelen : longint;
        dp_operation : boolean;
        i_field : byte;
        currsym : TObjSymbol;
        offset : longint;
        refoper : poper;
        msb : longint;
        r: byte;
        singlerec : tcompsinglerec;
        doublerec : tcompdoublerec;

      procedure setshifterop(op : byte);
        var
          r : byte;
          imm : dword;
          count : integer;
        begin
          case oper[op]^.typ of
            top_const:
              begin
                i_field:=1;
                if oper[op]^.val and $ff=oper[op]^.val then
                  bytes:=bytes or dword(oper[op]^.val)
                else
                  begin
                    { calc rotate and adjust imm }
                    count:=0;
                    r:=0;
                    imm:=dword(oper[op]^.val);
                    repeat
                      imm:=RolDWord(imm, 2);
                      inc(r);
                      inc(count);
                      if count > 32 then
                        begin
                          message1(asmw_e_invalid_opcode_and_operands, 'invalid shifter imm');
                          exit;
                        end;
                    until (imm and $ff)=imm;
                    bytes:=bytes or (r shl 8) or imm;
                  end;
              end;
            top_reg:
              begin
                i_field:=0;
                bytes:=bytes or getsupreg(oper[op]^.reg);

                { does a real shifter op follow? }
                if (op+1<opercnt) and (oper[op+1]^.typ=top_shifterop) then
                  with oper[op+1]^.shifterop^ do
                    begin
                      bytes:=bytes or ((shiftimm and $1F) shl 7);
                      if shiftmode<>SM_RRX then
                        bytes:=bytes or (ord(shiftmode) - ord(SM_LSL)) shl 5
                      else
                        bytes:=bytes or (3 shl 5);
                      if getregtype(rs) <> R_INVALIDREGISTER then
                        begin
                          bytes:=bytes or (1 shl 4);
                          bytes:=bytes or (getsupreg(rs) shl 8);
                        end
                    end;
              end;
          else
            internalerror(2005091103);
          end;
        end;

      function MakeRegList(reglist: tcpuregisterset): word;
        var
          i, w: integer;
        begin
          result:=0;
          w:=0;
          for i:=RS_R0 to RS_R15 do
            begin
              if i in reglist then
                result:=result or (1 shl w);
              inc(w);
            end;
        end;

      function getcoproc(reg: tregister): byte;
        begin
          if reg=NR_p15 then
            result:=15
          else
            begin
              Message1(asmw_e_invalid_opcode_and_operands,'Invalid coprocessor port');
              result:=0;
            end;
        end;

      function getcoprocreg(reg: tregister): byte;
        var
          tmpr: tregister;
        begin
          { FIXME: temp variable r is needed here to avoid Internal error 20060521 }
          {        while compiling the compiler. }
          tmpr:=NR_CR0;
          result:=getsupreg(reg)-getsupreg(tmpr);
        end;

      function getmmreg(reg: tregister): byte;
        begin
          case reg of
            NR_D0: result:=0;
            NR_D1: result:=1;
            NR_D2: result:=2;
            NR_D3: result:=3;
            NR_D4: result:=4;
            NR_D5: result:=5;
            NR_D6: result:=6;
            NR_D7: result:=7;
            NR_D8: result:=8;
            NR_D9: result:=9;
            NR_D10: result:=10;
            NR_D11: result:=11;
            NR_D12: result:=12;
            NR_D13: result:=13;
            NR_D14: result:=14;
            NR_D15: result:=15;
            NR_D16: result:=16;
            NR_D17: result:=17;
            NR_D18: result:=18;
            NR_D19: result:=19;
            NR_D20: result:=20;
            NR_D21: result:=21;
            NR_D22: result:=22;
            NR_D23: result:=23;
            NR_D24: result:=24;
            NR_D25: result:=25;
            NR_D26: result:=26;
            NR_D27: result:=27;
            NR_D28: result:=28;
            NR_D29: result:=29;
            NR_D30: result:=30;
            NR_D31: result:=31;

            NR_S0: result:=0;
            NR_S1: result:=1;
            NR_S2: result:=2;
            NR_S3: result:=3;
            NR_S4: result:=4;
            NR_S5: result:=5;
            NR_S6: result:=6;
            NR_S7: result:=7;
            NR_S8: result:=8;
            NR_S9: result:=9;
            NR_S10: result:=10;
            NR_S11: result:=11;
            NR_S12: result:=12;
            NR_S13: result:=13;
            NR_S14: result:=14;
            NR_S15: result:=15;
            NR_S16: result:=16;
            NR_S17: result:=17;
            NR_S18: result:=18;
            NR_S19: result:=19;
            NR_S20: result:=20;
            NR_S21: result:=21;
            NR_S22: result:=22;
            NR_S23: result:=23;
            NR_S24: result:=24;
            NR_S25: result:=25;
            NR_S26: result:=26;
            NR_S27: result:=27;
            NR_S28: result:=28;
            NR_S29: result:=29;
            NR_S30: result:=30;
            NR_S31: result:=31;
          else
            result:=0;
          end;
        end;

      procedure encodethumbimm(imm: longword);
        var
          imm12, tmp: tcgint;
          shift: integer;
          found: boolean;
        begin
          found:=true;
          if (imm and $FF) = imm then
            imm12:=imm
          else if ((imm shr 16)=(imm and $FFFF)) and
                  ((imm and $FF00FF00) = 0) then
            imm12:=(imm and $ff) or ($1 shl 8)
          else if ((imm shr 16)=(imm and $FFFF)) and
                  ((imm and $00FF00FF) = 0) then
            imm12:=((imm shr 8) and $ff) or ($2 shl 8)
          else if ((imm shr 16)=(imm and $FFFF)) and
                  (((imm shr 8) and $FF)=(imm and $FF)) then
            imm12:=(imm and $ff) or ($3 shl 8)
          else
            begin
              found:=false;
              imm12:=0;
              for shift:=1 to 31 do
                begin
                  tmp:=RolDWord(imm,shift);
                  if ((tmp and $FF)=tmp) and
                     ((tmp and $80)=$80) then
                    begin
                      imm12:=(tmp and $7F) or (shift shl 7);
                      found:=true;
                      break;
                    end;
                end;
            end;

          if found then
            begin
              bytes:=bytes or (imm12 and $FF);
              bytes:=bytes or (((imm12 shr 8) and $7) shl 12);
              bytes:=bytes or (((imm12 shr 11) and $1) shl 26);
            end
          else
            Message1(asmw_e_value_exceeds_bounds, IntToStr(imm));
        end;

      procedure setthumbshift(op: byte; is_sat: boolean = false);
        var
          shift,typ: byte;
        begin
          shift:=0;
          typ:=0;
          case oper[op]^.shifterop^.shiftmode of
            SM_None: ;
            SM_LSL: begin typ:=0; shift:=oper[op]^.shifterop^.shiftimm; end;
            SM_LSR: begin typ:=1; shift:=oper[op]^.shifterop^.shiftimm; if shift=32 then shift:=0; end;
            SM_ASR: begin typ:=2; shift:=oper[op]^.shifterop^.shiftimm; if shift=32 then shift:=0; end;
            SM_ROR: begin typ:=3; shift:=oper[op]^.shifterop^.shiftimm; if shift=0 then message(asmw_e_invalid_opcode_and_operands); end;
            SM_RRX: begin typ:=3; shift:=0; end;
          end;

          if is_sat then
            begin
              bytes:=bytes or ((typ and 1) shl 5);
              bytes:=bytes or ((typ shr 1) shl 21);
            end
          else
            bytes:=bytes or (typ shl 4);
          bytes:=bytes or (shift and $3) shl 6;
          bytes:=bytes or ((shift and $1C) shr 2) shl 12;
        end;

      begin
        bytes:=$0;
        bytelen:=4;
        i_field:=0;
        { evaluate and set condition code }
        bytes:=bytes or (CondVal[condition] shl 28);

        { condition code allowed? }

        { setup rest of the instruction }
        case insentry^.code[0] of
          #$01: // B/BL
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              { set offset }
              if oper[0]^.typ=top_const then
                bytes:=bytes or ((oper[0]^.val shr 2) and $ffffff)
              else
                begin
                  currsym:=objdata.symbolref(oper[0]^.ref^.symbol);

                  { tlscall is not relative so ignore the offset }
                  if oper[0]^.ref^.refaddr<>addr_tlscall then
                    bytes:=bytes or (((oper[0]^.ref^.offset-8) shr 2) and $ffffff);

                  if (opcode<>A_BL) or (condition<>C_None) then
                    objdata.writereloc(aint(bytes),4,currsym,RELOC_RELATIVE_24)
                  else
                    case oper[0]^.ref^.refaddr of
                      addr_pic:
                        objdata.writereloc(aint(bytes),4,currsym,RELOC_ARM_CALL);
                      addr_full:
                        objdata.writereloc(aint(bytes),4,currsym,RELOC_RELATIVE_CALL);
                      addr_tlscall:
                        objdata.writereloc(aint(bytes),4,currsym,RELOC_TLS_CALL);
                      else
                        Internalerror(2019092903);
                    end;
                  exit;
                end;
            end;
          #$02:
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              { set code }
              bytes:=bytes or (oper[0]^.val and $FFFFFF);
            end;
          #$03:
            begin // BLX/BX
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              bytes:=bytes or getsupreg(oper[0]^.reg);
            end;
          #$04..#$07: // SUB
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set destination }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              { set Rn }
              bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);
              { create shifter op }
              setshifterop(2);
              { set I field }
              bytes:=bytes or (i_field shl 25);
              { set S if necessary }
              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$08,#$0A,#$0B: // MOV
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set destination }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              { create shifter op }
              setshifterop(1);
              { set I field }
              bytes:=bytes or (i_field shl 25);
              { set S if necessary }
              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$0C,#$0E,#$0F: // CMP
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set destination }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);
              { create shifter op }
              setshifterop(1);
              { set I field }
              bytes:=bytes or (i_field shl 25);
              { always set S bit }
              bytes:=bytes or (1 shl 20);
            end;
          #$10: // MRS
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set destination }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);

              case oper[1]^.reg of
                NR_APSR,NR_CPSR:;
                NR_SPSR:
                  begin
                    bytes:=bytes or (1 shl 22);
                  end;
              else
                Message(asmw_e_invalid_opcode_and_operands);
              end;
            end;
          #$12,#$13: // MSR
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              { set destination }

              if oper[0]^.typ=top_specialreg then
                begin
                  if (oper[0]^.specialreg<>NR_CPSR) and
                     (oper[0]^.specialreg<>NR_SPSR) then
                    Message1(asmw_e_invalid_opcode_and_operands, '"Invalid special reg"');

                  if srC in oper[0]^.specialflags then
                    bytes:=bytes or (1 shl 16);
                  if srX in oper[0]^.specialflags then
                    bytes:=bytes or (1 shl 17);
                  if srS in oper[0]^.specialflags then
                    bytes:=bytes or (1 shl 18);
                  if srF in oper[0]^.specialflags then
                    bytes:=bytes or (1 shl 19);

                  { Set R bit }
                  if oper[0]^.specialreg=NR_SPSR then
                    bytes:=bytes or (1 shl 22);
                end
              else
                case oper[0]^.reg of
                  NR_APSR_nzcvq: bytes:=bytes or (2 shl 18);
                  NR_APSR_g: bytes:=bytes or (1 shl 18);
                  NR_APSR_nzcvqg: bytes:=bytes or (3 shl 18);
                else
                  Message1(asmw_e_invalid_opcode_and_operands, 'Invalid combination APSR bits used');
                end;

              setshifterop(1);
            end;
          #$14: // MUL/MLA r1,r2,r3
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]);
              { set regs }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 16;
              bytes:=bytes or getsupreg(oper[1]^.reg);
              bytes:=bytes or getsupreg(oper[2]^.reg) shl 8;

              if oppostfix in [PF_S] then
                bytes:=bytes or (1 shl 20);
            end;
          #$15: // MUL/MLA r1,r2,r3,r4
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]) shl 4;
              { set regs }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 16;
              bytes:=bytes or getsupreg(oper[1]^.reg);
              bytes:=bytes or getsupreg(oper[2]^.reg) shl 8;
              if ops>3 then
                bytes:=bytes or getsupreg(oper[3]^.reg) shl 12
              else
                bytes:=bytes or ord(insentry^.code[4]) shl 12;

              if oppostfix in [PF_R,PF_X] then
                bytes:=bytes or (1 shl 5);

              if oppostfix in [PF_S] then
                bytes:=bytes or (1 shl 20);
            end;
          #$16: // MULL r1,r2,r3,r4
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]) shl 4;
              { set regs }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;

              if (ops=3) and (opcode=A_PKHTB) then
                begin
                  bytes:=bytes or getsupreg(oper[1]^.reg);
                  bytes:=bytes or getsupreg(oper[2]^.reg) shl 16;
                end
              else
                begin
                  bytes:=bytes or getsupreg(oper[1]^.reg) shl 16;
                  bytes:=bytes or getsupreg(oper[2]^.reg);
                end;

              if ops=4 then
                begin
                  if oper[3]^.typ=top_shifterop then
                    begin
                      if opcode in [A_PKHBT,A_PKHTB] then
                        begin
                          if ((opcode=A_PKHTB) and
                              (oper[3]^.shifterop^.shiftmode <> SM_ASR)) or
                            ((opcode=A_PKHBT) and
                             (oper[3]^.shifterop^.shiftmode <> SM_LSL)) or
                            (oper[3]^.shifterop^.rs<>NR_NO) then
                            Message1(asmw_e_invalid_opcode_and_operands,GetString);

                          bytes:=bytes or ((oper[3]^.shifterop^.shiftimm and $1F) shl 7);
                        end
                      else
                        begin
                          if (oper[3]^.shifterop^.shiftmode<>sm_ror) or
                            (oper[3]^.shifterop^.rs<>NR_NO) or
                            (not (oper[3]^.shifterop^.shiftimm in [0,8,16,24])) then
                            Message1(asmw_e_invalid_opcode_and_operands,GetString);

                          bytes:=bytes or (((oper[3]^.shifterop^.shiftimm shr 3) and $3) shl 10);
                        end;
                    end
                  else
                    bytes:=bytes or getsupreg(oper[3]^.reg) shl 8;
                end;

              if PF_S=oppostfix then
                bytes:=bytes or (1 shl 20);
              if PF_X=oppostfix then
                bytes:=bytes or (1 shl 5);
            end;
          #$17: // LDR/STR
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
              if getregtype(oper[1]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[1]^.ref^.offset;
                  if offset>=0 then
                    { set U flag }
                    bytes:=bytes or (1 shl 23)
                  else
                    offset:=-offset;
                  bytes:=bytes or (offset and $FFF);
                end
              else
                begin
                  { set U flag }
                  if oper[1]^.ref^.signindex>=0 then
                    bytes:=bytes or (1 shl 23);
                  { set I flag }
                  bytes:=bytes or (1 shl 25);
                  bytes:=bytes or getsupreg(oper[1]^.ref^.index);
                  { set shift }
                  with oper[1]^.ref^ do
                    if shiftmode<>SM_None then
                      begin
                        bytes:=bytes or ((shiftimm and $1F) shl 7);
                        if shiftmode<>SM_RRX then
                          bytes:=bytes or (ord(shiftmode) - ord(SM_LSL)) shl 5
                        else
                          bytes:=bytes or (3 shl 5);
                      end
                end;
              { set W bit }
              if oper[1]^.ref^.addressmode=AM_PREINDEXED then
                bytes:=bytes or (1 shl 21);
              { set P bit if necessary }
              if oper[1]^.ref^.addressmode<>AM_POSTINDEXED then
                bytes:=bytes or (1 shl 24);
            end;
          #$18: // LDREX/STREX
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              if (ops=3) then
                begin
                  if opcode<>A_LDREXD then
                    bytes:=bytes or getsupreg(oper[1]^.reg);

                  bytes:=bytes or (getsupreg(oper[2]^.ref^.base) shl 16);
                end
              else if (ops=4) then // STREXD
                begin
                  if opcode<>A_LDREXD then
                    bytes:=bytes or getsupreg(oper[1]^.reg);

                  bytes:=bytes or (getsupreg(oper[3]^.ref^.base) shl 16);
                end
              else
                bytes:=bytes or (getsupreg(oper[1]^.ref^.base) shl 16);
            end;
          #$19: // LDRD/STRD
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;

              refoper:=oper[1];
              if ops=3 then
                refoper:=oper[2];

              bytes:=bytes or getsupreg(refoper^.ref^.base) shl 16;
              if getregtype(refoper^.ref^.index)=R_INVALIDREGISTER then
                begin
                  bytes:=bytes or (1 shl 22);
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(refoper^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+refoper^.ref^.offset;
                  if offset>=0 then
                    { set U flag }
                    bytes:=bytes or (1 shl 23)
                  else
                    offset:=-offset;
                  bytes:=bytes or (offset and $F);
                  bytes:=bytes or ((offset and $F0) shl 4);
                end
              else
                begin
                  { set U flag }
                  if refoper^.ref^.signindex>=0 then
                    bytes:=bytes or (1 shl 23);
                  bytes:=bytes or getsupreg(refoper^.ref^.index);
                end;
              { set W bit }
              if refoper^.ref^.addressmode=AM_PREINDEXED then
                bytes:=bytes or (1 shl 21);
              { set P bit if necessary }
              if refoper^.ref^.addressmode<>AM_POSTINDEXED then
                bytes:=bytes or (1 shl 24);
            end;
          #$1A: // QADD/QSUB
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]) shl 4;
              { set regs }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              bytes:=bytes or getsupreg(oper[1]^.reg) shl 0;
              bytes:=bytes or getsupreg(oper[2]^.reg) shl 16;
            end;
          #$1B:
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]) shl 4;
              { set regs }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              bytes:=bytes or getsupreg(oper[1]^.reg);
              if ops=3 then
                begin
                  if (oper[2]^.shifterop^.shiftmode<>sm_ror) or
                    (oper[2]^.shifterop^.rs<>NR_NO) or
                    (not (oper[2]^.shifterop^.shiftimm in [0,8,16,24])) then
                    Message1(asmw_e_invalid_opcode_and_operands,GetString);

                  bytes:=bytes or (((oper[2]^.shifterop^.shiftimm shr 3) and $3) shl 10);
                end;
            end;
          #$1C: // MCR/MRC
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]) shl 4;
              { set regs and operands }
              bytes:=bytes or getcoproc(oper[0]^.reg) shl 8;
              bytes:=bytes or ((oper[1]^.val and $7) shl 21);
              bytes:=bytes or getsupreg(oper[2]^.reg) shl 12;
              bytes:=bytes or getcoprocreg(oper[3]^.reg) shl 16;
              bytes:=bytes or getcoprocreg(oper[4]^.reg);
              if ops > 5 then
                bytes:=bytes or ((oper[5]^.val and $7) shl 5);
            end;
          #$1D: // MCRR/MRRC
            begin
              { set instruction code }
              bytes:=bytes or ord(insentry^.code[1]) shl 24;
              bytes:=bytes or ord(insentry^.code[2]) shl 16;
              bytes:=bytes or ord(insentry^.code[3]) shl 4;
              { set regs and operands }
              bytes:=bytes or getcoproc(oper[0]^.reg) shl 8;
              bytes:=bytes or ((oper[1]^.val and $7) shl 4);
              bytes:=bytes or getsupreg(oper[2]^.reg) shl 12;
              bytes:=bytes or getsupreg(oper[3]^.reg) shl 16;
              bytes:=bytes or getcoprocreg(oper[4]^.reg);
            end;
          #$1E: // LDRHT/STRHT
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;

              refoper:=oper[1];

              bytes:=bytes or getsupreg(refoper^.ref^.base) shl 16;
              if getregtype(refoper^.ref^.index)=R_INVALIDREGISTER then
                begin
                  bytes:=bytes or (1 shl 22);
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(refoper^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+refoper^.ref^.offset;

                  if offset>=0 then
                    { set U flag }
                    bytes:=bytes or (1 shl 23)
                  else
                    offset:=-offset;
                  bytes:=bytes or (offset and $F);
                  bytes:=bytes or ((offset and $F0) shl 4);
                end
              else
                begin
                  { set U flag }
                  if refoper^.ref^.signindex>=0 then
                    bytes:=bytes or (1 shl 23);
                  bytes:=bytes or getsupreg(refoper^.ref^.index);
                end;
            end;
          #$22: // LDRH/STRH
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 16);
              bytes:=bytes or ord(insentry^.code[2]);
              { src/dest register (Rd) }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              { base register (Rn) }
              bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
              if getregtype(oper[1]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  bytes:=bytes or (1 shl 22); // with immediate offset
                  offset:=oper[1]^.ref^.offset;
                  if offset>=0 then
                    { set U flag }
                    bytes:=bytes or (1 shl 23)
                  else
                    offset:=-offset;
                  bytes:=bytes or (offset and $F);
                  bytes:=bytes or ((offset and $F0) shl 4);
                end
              else
                begin
                  { set U flag }
                  if oper[1]^.ref^.signindex>=0 then
                    bytes:=bytes or (1 shl 23);
                  bytes:=bytes or getsupreg(oper[1]^.ref^.index);
                end;
              { set W bit }
              if oper[1]^.ref^.addressmode=AM_PREINDEXED then
                bytes:=bytes or (1 shl 21);
              { set P bit if necessary }
              if oper[1]^.ref^.addressmode<>AM_POSTINDEXED then
                bytes:=bytes or (1 shl 24);
            end;
          #$25: // PLD/PLI
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.ref^.base) shl 16;
              if getregtype(oper[0]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[0]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[0]^.ref^.offset;
                  if offset>=0 then
                    begin
                      { set U flag }
                      bytes:=bytes or (1 shl 23);
                      bytes:=bytes or offset
                    end
                  else
                    begin
                      offset:=-offset;
                      bytes:=bytes or offset
                    end;
                end
              else
                begin
                  bytes:=bytes or (1 shl 25);
                  { set U flag }
                  if oper[0]^.ref^.signindex>=0 then
                    bytes:=bytes or (1 shl 23);
                  bytes:=bytes or getsupreg(oper[0]^.ref^.index);
                  { set shift }
                  with oper[0]^.ref^ do
                    if shiftmode<>SM_None then
                      begin
                        bytes:=bytes or ((shiftimm and $1F) shl 7);
                        if shiftmode<>SM_RRX then
                          bytes:=bytes or (ord(shiftmode) - ord(SM_LSL)) shl 5
                        else
                          bytes:=bytes or (3 shl 5);
                      end
                end;
            end;
          #$26: // LDM/STM
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 20);

              if ops>1 then
                begin
                  if oper[0]^.typ=top_ref then
                    begin
                      { set W bit }
                      if oper[0]^.ref^.addressmode=AM_PREINDEXED then
                        bytes:=bytes or (1 shl 21);
                      { set Rn }
                      bytes:=bytes or (getsupreg(oper[0]^.ref^.index) shl 16);
                    end
                  else { typ=top_reg }
                    begin
                      { set Rn }
                      bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);
                    end;

                  if oper[1]^.usermode then
                    begin
                      if (oper[0]^.typ=top_ref) then
                        begin
                          if (opcode=A_LDM) and
                             (RS_PC in oper[1]^.regset^) then
                            begin
                              // Valid exception return
                            end
                          else
                            Message(asmw_e_invalid_opcode_and_operands);
                        end;

                      bytes:=bytes or (1 shl 22);
                    end;
                  { reglist }
                  bytes:=bytes or MakeRegList(oper[1]^.regset^);
                end
              else
                begin
                  { push/pop }
                  { Set W and Rn to SP }
                  if opcode=A_PUSH then
                    bytes:=bytes or (1 shl 21);
                  bytes:=bytes or ($D shl 16);
                  { reglist }
                  bytes:=bytes or MakeRegList(oper[0]^.regset^);
                end;
              { set P bit }
              if (opcode=A_LDM) and (oppostfix in [PF_ED,PF_EA,PF_IB,PF_DB])
              or (opcode=A_STM) and (oppostfix in [PF_FA,PF_FD,PF_IB,PF_DB])
              or (opcode=A_PUSH) then
                bytes:=bytes or (1 shl 24);
              { set U bit }
              if (opcode=A_LDM) and (oppostfix in [PF_None,PF_ED,PF_FD,PF_IB,PF_IA])
              or (opcode=A_STM) and (oppostfix in [PF_None,PF_FA,PF_EA,PF_IB,PF_IA])
              or (opcode=A_POP) then
                bytes:=bytes or (1 shl 23);
            end;
          #$27: // SWP/SWPB
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 20);
              bytes:=bytes or (ord(insentry^.code[2]) shl 4);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              bytes:=bytes or getsupreg(oper[1]^.reg);
              if ops=3 then
                bytes:=bytes or (getsupreg(oper[2]^.ref^.base) shl 16);
            end;
          #$28: // BX/BLX
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              { set offset }
              if oper[0]^.typ=top_const then
                bytes:=bytes or ((oper[0]^.val shr 2) and $ffffff)
              else
                begin
                  currsym:=objdata.symbolref(oper[0]^.ref^.symbol);
                  if (currsym.bind<>AB_LOCAL) and (currsym.objsection<>objdata.CurrObjSec) then
                    begin
                      bytes:=bytes or $fffffe; // TODO: Not sure this is right, but it matches the output of gas
                      objdata.writereloc(oper[0]^.ref^.offset,0,currsym,RELOC_RELATIVE_24_THUMB);
                    end
                  else
                    begin
                      offset:=((currsym.offset-insoffset-8) and $3fffffe);

                      { Turn BLX into BL if the destination isn't odd, could happen with recursion }
                      if not odd(offset shr 1) then
                        bytes:=(bytes and $EB000000) or $EB000000;

                      bytes:=bytes or ((offset shr 2) and $ffffff);
                      bytes:=bytes or ((offset shr 1) and $1) shl 24;
                    end;
                end;
            end;
          #$29: // SUB
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              { set S if necessary }
              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$2A:
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set opers }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              if opcode in [A_SSAT, A_SSAT16] then
                bytes:=bytes or (((oper[1]^.val-1) and $1F) shl 16)
              else
                bytes:=bytes or ((oper[1]^.val and $1F) shl 16);
              bytes:=bytes or getsupreg(oper[2]^.reg);

              if (ops>3) and
                (oper[3]^.typ=top_shifterop) and
                (oper[3]^.shifterop^.rs=NR_NO) then
                begin
                  bytes:=bytes or ((oper[3]^.shifterop^.shiftimm and $1F) shl 7);
                  if oper[3]^.shifterop^.shiftmode=SM_ASR then
                    bytes:=bytes or (1 shl 6)
                  else if oper[3]^.shifterop^.shiftmode<>SM_LSL then
                    Message1(asmw_e_invalid_opcode_and_operands,GetString);
                end;
            end;
          #$2B: // SETEND
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set endian specifier }
              bytes:=bytes or ((oper[0]^.val and 1) shl 9);
            end;
          #$2C: // MOVW
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set destination }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              { set imm }
              bytes:=bytes or (oper[1]^.val and $FFF);
              bytes:=bytes or ((oper[1]^.val and $F000) shl 4);
            end;
          #$2D: // BFX
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if ops=3 then
                begin
                  msb:=(oper[1]^.val+oper[2]^.val-1);

                  { set destination }
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
                  { set immediates }
                  bytes:=bytes or ((oper[1]^.val and $1F) shl 7);
                  bytes:=bytes or ((msb and $1F) shl 16);
                end
              else
                begin
                  if opcode in [A_BFC,A_BFI] then
                    msb:=(oper[2]^.val+oper[3]^.val-1)
                  else
                    msb:=oper[3]^.val-1;

                  { set destination }
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
                  bytes:=bytes or getsupreg(oper[1]^.reg);
                  { set immediates }
                  bytes:=bytes or ((oper[2]^.val and $1F) shl 7);
                  bytes:=bytes or ((msb and $1F) shl 16);
                end;
            end;
          #$2E: // Cache stuff
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set code }
              bytes:=bytes or (oper[0]^.val and $F);
            end;
          #$2F: // Nop
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
            end;
          #$30: // Shifts
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set destination }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              bytes:=bytes or getsupreg(oper[1]^.reg);
              if ops>2 then
                begin
                  { set shift }
                  if oper[2]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[2]^.reg) shl 8)
                  else
                    bytes:=bytes or ((oper[2]^.val and $1F) shl 7);
                end;
              { set S if necessary }
              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$31: // BKPT
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 0);
              { set imm }
              bytes:=bytes or (oper[0]^.val and $FFF0) shl 4;
              bytes:=bytes or (oper[0]^.val and $F);
            end;
          #$32: // CLZ/REV
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
              bytes:=bytes or getsupreg(oper[1]^.reg);
            end;
          #$33:
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);

              if oper[1]^.typ=top_ref then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[1]^.ref^.offset;
                  if offset>=0 then
                    begin
                      { set U flag }
                      bytes:=bytes or (1 shl 23);
                      bytes:=bytes or offset
                    end
                  else
                    begin
                      bytes:=bytes or (1 shl 22);
                      offset:=-offset;
                      bytes:=bytes or offset
                    end;
                end
              else
                begin
                  if is_shifter_const(oper[1]^.val,r) then
                    begin
                      setshifterop(1);
                      bytes:=bytes or (1 shl 23);
                    end
                  else
                    begin
                      bytes:=bytes or (1 shl 22);
                      oper[1]^.val:=-oper[1]^.val;
                      setshifterop(1);
                    end;
                end;
            end;
          #$40,#$90: // VMOV
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              { set regs }
              Rd:=0;
              Rn:=0;
              Rm:=0;

              case oppostfix of
                PF_None:
                  begin
                    if ops=4 then
                      begin
                        if (getregtype(oper[0]^.reg)=R_MMREGISTER) and
                           (getregtype(oper[2]^.reg)=R_INTREGISTER) then
                          begin
                            Rd:=getmmreg(oper[0]^.reg);
                            Rm:=getsupreg(oper[2]^.reg);
                            Rn:=getsupreg(oper[3]^.reg);
                          end
                        else if (getregtype(oper[0]^.reg)=R_INTREGISTER) and
                                (getregtype(oper[2]^.reg)=R_MMREGISTER) then
                          begin
                            Rm:=getsupreg(oper[0]^.reg);
                            Rn:=getsupreg(oper[1]^.reg);
                            Rd:=getmmreg(oper[2]^.reg);
                          end
                        else
                          message(asmw_e_invalid_opcode_and_operands);

                        bytes:=bytes or (((Rd and $1E) shr 1) shl 0);
                        bytes:=bytes or ((Rd and $1) shl 5);

                        bytes:=bytes or (Rm shl 12);
                        bytes:=bytes or (Rn shl 16);
                      end
                    else if ops=3 then
                      begin
                        if (getregtype(oper[0]^.reg)=R_MMREGISTER) and
                           (getregtype(oper[1]^.reg)=R_INTREGISTER) then
                          begin
                            Rd:=getmmreg(oper[0]^.reg);
                            Rm:=getsupreg(oper[1]^.reg);
                            Rn:=getsupreg(oper[2]^.reg);
                          end
                        else if (getregtype(oper[0]^.reg)=R_INTREGISTER) and
                                (getregtype(oper[2]^.reg)=R_MMREGISTER) then
                          begin
                            Rm:=getsupreg(oper[0]^.reg);
                            Rn:=getsupreg(oper[1]^.reg);
                            Rd:=getmmreg(oper[2]^.reg);
                          end
                        else
                          message(asmw_e_invalid_opcode_and_operands);

                        bytes:=bytes or ((Rd and $F) shl 0);
                        bytes:=bytes or ((Rd and $10) shl 1);

                        bytes:=bytes or (Rm shl 12);
                        bytes:=bytes or (Rn shl 16);
                      end
                    else if ops=2 then
                      begin
                        if (getregtype(oper[0]^.reg)=R_MMREGISTER) and
                           (getregtype(oper[1]^.reg)=R_INTREGISTER) then
                          begin
                            Rd:=getmmreg(oper[0]^.reg);
                            Rm:=getsupreg(oper[1]^.reg);
                          end
                        else if (getregtype(oper[0]^.reg)=R_INTREGISTER) and
                                (getregtype(oper[1]^.reg)=R_MMREGISTER) then
                          begin
                            Rm:=getsupreg(oper[0]^.reg);
                            Rd:=getmmreg(oper[1]^.reg);
                          end
                        else
                          message(asmw_e_invalid_opcode_and_operands);

                        bytes:=bytes or (((Rd and $1E) shr 1) shl 16);
                        bytes:=bytes or ((Rd and $1) shl 7);

                        bytes:=bytes or (Rm shl 12);
                      end;
                  end;
                PF_F32:
                  begin
                    if (getregtype(oper[0]^.reg)<>R_MMREGISTER) then
                      Message(asmw_e_invalid_opcode_and_operands);

                    case oper[1]^.typ of
                      top_realconst:
                        begin
                          if not(IsVFPFloatImmediate(s32real,oper[1]^.val_real)) then
                            Message(asmw_e_invalid_opcode_and_operands);
                          singlerec.value:=oper[1]^.val_real;
                          singlerec:=tcompsinglerec(NtoLE(DWord(singlerec)));

                          bytes:=bytes or ((singlerec.bytes[2] shr 3) and $f);
                          bytes:=bytes or (DWord((singlerec.bytes[2] shr 7) and $1) shl 16) or (DWord(singlerec.bytes[3] and $3) shl 17) or (DWord((singlerec.bytes[3] shr 7) and $1) shl 19);
                        end;
                      top_reg:
                        begin
                          if getregtype(oper[1]^.reg)<>R_MMREGISTER then
                            Message(asmw_e_invalid_opcode_and_operands);
                          Rm:=getmmreg(oper[1]^.reg);
                          bytes:=bytes or (((Rm and $1E) shr 1) shl 0);
                          bytes:=bytes or ((Rm and $1) shl 5);
                        end;
                      else
                        Message(asmw_e_invalid_opcode_and_operands);
                    end;
                    Rd:=getmmreg(oper[0]^.reg);

                    bytes:=bytes or (((Rd and $1E) shr 1) shl 12);
                    bytes:=bytes or ((Rd and $1) shl 22);

                  end;
                PF_F64:
                  begin
                    if (getregtype(oper[0]^.reg)<>R_MMREGISTER) then
                      Message(asmw_e_invalid_opcode_and_operands);

                    case oper[1]^.typ of
                      top_realconst:
                        begin
                          if not(IsVFPFloatImmediate(s64real,oper[1]^.val_real)) then
                            Message(asmw_e_invalid_opcode_and_operands);
                          doublerec.value:=oper[1]^.val_real;
                          doublerec:=tcompdoublerec(NtoLE(QWord(doublerec)));

                          //      32c:       eeb41b00        vmov.f64        d1, #64 ; 0x40

                          // 32c:       eeb61b00        vmov.f64        d1, #96 ; 0x60
                          bytes:=bytes or (doublerec.bytes[6] and $f);
                          bytes:=bytes or (DWord((doublerec.bytes[6] shr 4) and $7) shl 16) or (DWord((doublerec.bytes[7] shr 7) and $1) shl 19);
                        end;
                      top_reg:
                        begin
                          if getregtype(oper[1]^.reg)<>R_MMREGISTER then
                            Message(asmw_e_invalid_opcode_and_operands);
                          Rm:=getmmreg(oper[1]^.reg);
                          bytes:=bytes or (Rm and $F);
                          bytes:=bytes or ((Rm and $10) shl 1);
                        end;
                      else
                        Message(asmw_e_invalid_opcode_and_operands);
                    end;
                    Rd:=getmmreg(oper[0]^.reg);

                    bytes:=bytes or (1 shl 8);

                    bytes:=bytes or ((Rd and $F) shl 12);
                    bytes:=bytes or (((Rd and $10) shr 4) shl 22);
                  end;
                else
                  Message(asmw_e_invalid_opcode_and_operands);
              end;
            end;
          #$41,#$91: // VMRS/VMSR
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set regs }
              if (opcode=A_VMRS) or
                 (opcode=A_FMRX) then
                begin
                  case oper[1]^.reg of
                    NR_FPSID: Rn:=$0;
                    NR_FPSCR: Rn:=$1;
                    NR_MVFR1: Rn:=$6;
                    NR_MVFR0: Rn:=$7;
                    NR_FPEXC: Rn:=$8;
                  else
                    Rn:=0;
                    message(asmw_e_invalid_opcode_and_operands);
                  end;

                  bytes:=bytes or (Rn shl 16);

                  if oper[0]^.reg=NR_APSR_nzcv then
                    bytes:=bytes or ($F shl 12)
                  else
                    bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
                end
              else
                begin
                  case oper[0]^.reg of
                    NR_FPSID: Rn:=$0;
                    NR_FPSCR: Rn:=$1;
                    NR_FPEXC: Rn:=$8;
                  else
                    Rn:=0;
                    message(asmw_e_invalid_opcode_and_operands);
                  end;

                  bytes:=bytes or (Rn shl 16);

                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 12);
                end;
            end;
          #$42,#$92: // VMUL
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set regs }
              if ops=3 then
                begin
                  Rd:=getmmreg(oper[0]^.reg);
                  Rn:=getmmreg(oper[1]^.reg);
                  Rm:=getmmreg(oper[2]^.reg);
                end
              else if ops=1 then
                begin
                  Rd:=getmmreg(oper[0]^.reg);
                  Rn:=0;
                  Rm:=0;
                end
              else if oper[1]^.typ=top_const then
                begin
                  Rd:=getmmreg(oper[0]^.reg);
                  Rn:=0;
                  Rm:=0;
                end
              else
                begin
                  Rd:=getmmreg(oper[0]^.reg);
                  Rn:=0;
                  Rm:=getmmreg(oper[1]^.reg);
                end;

              if (oppostfix=PF_F32) or (insentry^.code[5]=#1) then
                begin
                  D:=rd and $1; Rd:=Rd shr 1;
                  N:=rn and $1; Rn:=Rn shr 1;
                  M:=rm and $1; Rm:=Rm shr 1;
                end
              else
                begin
                  D:=(rd shr 4) and $1; Rd:=Rd and $F;
                  N:=(rn shr 4) and $1; Rn:=Rn and $F;
                  M:=(rm shr 4) and $1; Rm:=Rm and $F;

                  bytes:=bytes or (1 shl 8);
                end;

              bytes:=bytes or (Rd shl 12);
              bytes:=bytes or (Rn shl 16);
              bytes:=bytes or (Rm shl 0);

              bytes:=bytes or (D shl 22);
              bytes:=bytes or (N shl 7);
              bytes:=bytes or (M shl 5);
            end;
          #$43,#$93: // VCVT
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set regs }
              Rd:=getmmreg(oper[0]^.reg);
              Rm:=getmmreg(oper[1]^.reg);

              if (ops=2) and
                 (oppostfix in [PF_F32F64,PF_F64F32]) then
                begin
                  if oppostfix=PF_F32F64 then
                    begin
                      bytes:=bytes or (1 shl 8);

                      D:=rd and $1; Rd:=Rd shr 1;
                      M:=(rm shr 4) and $1; Rm:=Rm and $F;
                    end
                  else
                    begin
                      D:=(rd shr 4) and $1; Rd:=Rd and $F;
                      M:=rm and $1; Rm:=Rm shr 1;
                    end;

                  bytes:=bytes and $FFF0FFFF;
                  bytes:=bytes or ($7 shl 16);

                  bytes:=bytes or (Rd shl 12);
                  bytes:=bytes or (Rm shl 0);

                  bytes:=bytes or (D shl 22);
                  bytes:=bytes or (M shl 5);
                end
              else if (ops=2) and
                      (oppostfix=PF_None) then
                begin
                  d:=0;
                  case getsubreg(oper[0]^.reg) of
                    R_SUBNONE:
                      rd:=getsupreg(oper[0]^.reg);
                    R_SUBFS:
                      begin
                        rd:=getmmreg(oper[0]^.reg);

                        d:=rd and 1;
                        rd:=rd shr 1;
                      end;
                    R_SUBFD:
                      begin
                        rd:=getmmreg(oper[0]^.reg);

                        d:=(rd shr 4) and 1;
                        rd:=rd and $F;
                      end;
                    else
                      internalerror(2019050929);
                  end;

                  m:=0;
                  case getsubreg(oper[1]^.reg) of
                    R_SUBNONE:
                      rm:=getsupreg(oper[1]^.reg);
                    R_SUBFS:
                      begin
                        rm:=getmmreg(oper[1]^.reg);

                        m:=rm and 1;
                        rm:=rm shr 1;
                      end;
                    R_SUBFD:
                      begin
                        rm:=getmmreg(oper[1]^.reg);

                        m:=(rm shr 4) and 1;
                        rm:=rm and $F;
                      end;
                    else
                      internalerror(2019050928);
                  end;

                  bytes:=bytes or (Rd shl 12);
                  bytes:=bytes or (Rm shl 0);

                  bytes:=bytes or (D shl 22);
                  bytes:=bytes or (M shl 5);
                end
              else if ops=2 then
                begin
                  case oppostfix of
                    PF_S32F64,
                    PF_U32F64,
                    PF_F64S32,
                    PF_F64U32:
                      bytes:=bytes or (1 shl 8);
                    else
                      ;
                  end;

                  if oppostfix in [PF_S32F32,PF_S32F64,PF_U32F32,PF_U32F64] then
                    begin
                      case oppostfix of
                        PF_S32F64,
                        PF_S32F32:
                          bytes:=bytes or (1 shl 16);
                        else
                          ;
                      end;

                      bytes:=bytes or (1 shl 18);

                      D:=rd and $1; Rd:=Rd shr 1;

                      if oppostfix in [PF_S32F64,PF_U32F64] then
                        begin
                          M:=(rm shr 4) and $1; Rm:=Rm and $F;
                        end
                      else
                        begin
                          M:=rm and $1; Rm:=Rm shr 1;
                        end;
                    end
                  else
                    begin
                      case oppostfix of
                        PF_F64S32,
                        PF_F32S32:
                          bytes:=bytes or (1 shl 7);
                        else
                          bytes:=bytes and $FFFFFF7F;
                      end;

                      M:=rm and $1; Rm:=Rm shr 1;

                      if oppostfix in [PF_F64S32,PF_F64U32] then
                        begin
                          D:=(rd shr 4) and $1; Rd:=Rd and $F;
                        end
                      else
                        begin
                          D:=rd and $1; Rd:=Rd shr 1;
                        end
                    end;

                  bytes:=bytes or (Rd shl 12);
                  bytes:=bytes or (Rm shl 0);

                  bytes:=bytes or (D shl 22);
                  bytes:=bytes or (M shl 5);
                end
              else
                begin
                  if rd<>rm then
                    message(asmw_e_invalid_opcode_and_operands);

                  case oppostfix of
                    PF_S32F32,PF_U32F32,
                    PF_F32S32,PF_F32U32,
                    PF_S32F64,PF_U32F64,
                    PF_F64S32,PF_F64U32:
                      begin
                        if not (oper[2]^.val in [1..32]) then
                          message1(asmw_e_invalid_opcode_and_operands, 'fbits not within 1-32');

                        bytes:=bytes or (1 shl 7);
                        rn:=32;
                      end;
                    PF_S16F64,PF_U16F64,
                    PF_F64S16,PF_F64U16,
                    PF_S16F32,PF_U16F32,
                    PF_F32S16,PF_F32U16:
                      begin
                        if not (oper[2]^.val in [0..16]) then
                          message1(asmw_e_invalid_opcode_and_operands, 'fbits not within 0-16');

                        rn:=16;
                      end;
                    else
                      Rn:=0;
                      message(asmw_e_invalid_opcode_and_operands);
                  end;

                  case oppostfix of
                    PF_S16F64,PF_U16F64,
                    PF_S32F64,PF_U32F64,
                    PF_F64S16,PF_F64U16,
                    PF_F64S32,PF_F64U32:
                      begin
                        bytes:=bytes or (1 shl 8);
                        D:=(rd shr 4) and $1; Rd:=Rd and $F;
                      end;
                    else
                      begin
                        D:=rd and $1; Rd:=Rd shr 1;
                      end;
                  end;

                  case oppostfix of
                    PF_U16F64,PF_U16F32,
                    PF_U32F32,PF_U32F64,
                    PF_F64U16,PF_F32U16,
                    PF_F32U32,PF_F64U32:
                      bytes:=bytes or (1 shl 16);
                    else
                      ;
                  end;

                  if oppostfix in [PF_S32F32,PF_S32F64,PF_U32F32,PF_U32F64,PF_S16F32,PF_S16F64,PF_U16F32,PF_U16F64] then
                    bytes:=bytes or (1 shl 18);

                  bytes:=bytes or (Rd shl 12);
                  bytes:=bytes or (D shl 22);

                  rn:=rn-oper[2]^.val;

                  bytes:=bytes or ((rn and $1) shl 5);
                  bytes:=bytes or ((rn and $1E) shr 1);
                end;
            end;
          #$44,#$94: // VLDM/VSTM/VPUSH/VPOP
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              { set regs }
              if ops=2 then
                begin
                  if oper[0]^.typ=top_ref then
                    begin
                      Rn:=getsupreg(oper[0]^.ref^.index);

                      if oper[0]^.ref^.addressmode<>AM_OFFSET then
                        begin
                          { set W }
                          bytes:=bytes or (1 shl 21);
                        end
                      else if oppostfix in [PF_DB,PF_DBS,PF_DBD,PF_DBX] then
                        message1(asmw_e_invalid_opcode_and_operands, 'Invalid postfix without writeback');
                    end
                  else
                    begin
                      Rn:=getsupreg(oper[0]^.reg);

                      if oppostfix in [PF_DB,PF_DBS,PF_DBD,PF_DBX] then
                        message1(asmw_e_invalid_opcode_and_operands, 'Invalid postfix without writeback');
                    end;

                  bytes:=bytes or (Rn shl 16);

                  { Set PU bits }
                  case oppostfix of
                    PF_None,
                    PF_IA,PF_IAS,PF_IAD,PF_IAX:
                      bytes:=bytes or (1 shl 23);
                    PF_DB,PF_DBS,PF_DBD,PF_DBX:
                      bytes:=bytes or (2 shl 23);
                    else
                      ;
                  end;

                  case oppostfix of
                    PF_IAX,PF_DBX,PF_FDX,PF_EAX:
                      begin
                        bytes:=bytes or (1 shl 8);
                        bytes:=bytes or (1 shl 0); // Offset is odd
                      end;
                    else
                      ;
                  end;

                  dp_operation:=(oper[1]^.subreg=R_SUBFD);
                  if oper[1]^.regset^=[] then
                    message1(asmw_e_invalid_opcode_and_operands, 'Regset cannot be empty');

                  rd:=0;
                  for r:=0 to 31 do
                    if r in oper[1]^.regset^ then
                      begin
                        rd:=r;
                        break;
                      end;

                  rn:=32-rd;
                  for r:=rd+1 to 31 do
                    if not(r in oper[1]^.regset^) then
                      begin
                        rn:=r-rd;
                        break;
                      end;

                  if dp_operation then
                    begin
                      bytes:=bytes or (1 shl 8);

                      bytes:=bytes or (rn*2);

                      bytes:=bytes or ((rd and $F) shl 12);
                      bytes:=bytes or (((rd and $10) shr 4) shl 22);
                    end
                  else
                    begin
                      bytes:=bytes or rn;

                      bytes:=bytes or ((rd and $1) shl 22);
                      bytes:=bytes or (((rd and $1E) shr 1) shl 12);
                    end;
                end
              else { VPUSH/VPOP }
                begin
                  dp_operation:=(oper[0]^.subreg=R_SUBFD);
                  if oper[0]^.regset^=[] then
                    message1(asmw_e_invalid_opcode_and_operands, 'Regset cannot be empty');

                  rd:=0;
                  for r:=0 to 31 do
                    if r in oper[0]^.regset^ then
                      begin
                        rd:=r;
                        break;
                      end;

                  rn:=32-rd;
                  for r:=rd+1 to 31 do
                    if not(r in oper[0]^.regset^) then
                      begin
                        rn:=r-rd;
                        break;
                      end;

                  if dp_operation then
                    begin
                      bytes:=bytes or (1 shl 8);

                      bytes:=bytes or (rn*2);

                      bytes:=bytes or ((rd and $F) shl 12);
                      bytes:=bytes or (((rd and $10) shr 4) shl 22);
                    end
                  else
                    begin
                      bytes:=bytes or rn;

                      bytes:=bytes or ((rd and $1) shl 22);
                      bytes:=bytes or (((rd and $1E) shr 1) shl 12);
                    end;
                end;
            end;
          #$45,#$95: // VLDR/VSTR
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              { set regs }
              rd:=getmmreg(oper[0]^.reg);

              if getsubreg(oper[0]^.reg)=R_SUBFD then
                begin
                  bytes:=bytes or (1 shl 8);

                  bytes:=bytes or ((rd and $F) shl 12);
                  bytes:=bytes or (((rd and $10) shr 4) shl 22);
                end
              else
                begin
                  bytes:=bytes or (((rd and $1E) shr 1) shl 12);
                  bytes:=bytes or ((rd and $1) shl 22);
                end;

              { set ref }
              bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
              if getregtype(oper[1]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[1]^.ref^.offset;

                  offset:=offset div 4;

                  if offset>=0 then
                    begin
                      { set U flag }
                      bytes:=bytes or (1 shl 23);
                      bytes:=bytes or offset
                    end
                  else
                    begin
                      offset:=-offset;
                      bytes:=bytes or offset
                    end;
                end
              else
                message(asmw_e_invalid_opcode_and_operands);
            end;
          #$46: { System instructions }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              { set regs }
              if (oper[0]^.typ=top_modeflags) then
                begin
                  if mfA in oper[0]^.modeflags then bytes:=bytes or (1 shl 8);
                  if mfI in oper[0]^.modeflags then bytes:=bytes or (1 shl 7);
                  if mfF in oper[0]^.modeflags then bytes:=bytes or (1 shl 6);
                end;

              if (ops=2) then
                bytes:=bytes or (oper[1]^.val and $1F)
              else if (ops=1) and
                      (oper[0]^.typ=top_const) then
                bytes:=bytes or (oper[0]^.val and $1F);
            end;
          #$60: { Thumb }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);
              { set regs }
              if ops=2 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 3);
                  if (oper[1]^.typ=top_reg) then
                    bytes:=bytes or ((getsupreg(oper[1]^.reg) and $7) shl 6)
                  else
                    bytes:=bytes or ((oper[1]^.val and $1F) shl 6);
                end
              else if ops=3 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 3);
                  if (oper[2]^.typ=top_reg) then
                    bytes:=bytes or ((getsupreg(oper[2]^.reg) and $7) shl 6)
                  else
                    bytes:=bytes or ((oper[2]^.val and $1F) shl 6);
                end
              else if ops=1 then
                begin
                  if oper[0]^.typ=top_const then
                    bytes:=bytes or (oper[0]^.val and $FF);
                end;
            end;
          #$61: { Thumb }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);
              { set regs }
              if ops=2 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                  bytes:=bytes or ((getsupreg(oper[0]^.reg) and $8) shr 3) shl 7;

                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 3);
                end
              else if ops=1 then
                begin
                  if oper[0]^.typ=top_const then
                    bytes:=bytes or (oper[0]^.val and $FF);
                end;
            end;
          #$62..#$63: { Thumb branches }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);

              if insentry^.code[0]=#$63 then
                bytes:=bytes or (CondVal[condition] shl 8);

              if oper[0]^.typ=top_const then
                begin
                  if insentry^.code[0]=#$63 then
                    bytes:=bytes or (((oper[0]^.val shr 1)-1) and $FF)
                  else
                    bytes:=bytes or (((oper[0]^.val shr 1)-1) and $3FF);
                end
              else if oper[0]^.typ=top_reg then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 3);
                end
              else if oper[0]^.typ=top_ref then
                begin
                  offset:=0;
                  currsym:=objdata.symbolref(oper[0]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[0]^.ref^.offset;

                  if insentry^.code[0]=#$63 then
                    bytes:=bytes or (((offset+4) shr 1) and $FF)
                  else
                    bytes:=bytes or (((offset+4) shr 1) and $7FF);
                end
            end;
          #$64: { Thumb: Special encodings }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);

              case opcode of
                A_SUB:
                  begin
                    if (ops=3) and
                       (oper[2]^.typ=top_const) then
                      bytes:=bytes or ((oper[2]^.val shr 2) and $7F)
                    else if (ops=2) and
                            (oper[1]^.typ=top_const) then
                      bytes:=bytes or ((oper[1]^.val shr 2) and $7F);
                  end;
                A_MUL:
                  if (ops in [2,3]) then
                    begin
                      bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                      bytes:=bytes or (getsupreg(oper[1]^.reg) shl 3);
                    end;
                A_ADD:
                  begin
                    if ops=2 then
                      begin
                        bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                        bytes:=bytes or (getsupreg(oper[1]^.reg) shl $3);
                      end
                    else if (oper[0]^.reg<>NR_STACK_POINTER_REG) and
                       (oper[2]^.typ=top_const) then
                      begin
                        bytes:=bytes or (getsupreg(oper[0]^.reg) and $7) shl 8;
                        bytes:=bytes or ((oper[2]^.val shr 2) and $7F);
                      end
                    else if (oper[0]^.reg<>NR_STACK_POINTER_REG) and
                       (oper[2]^.typ=top_reg) then
                      begin
                        bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                        bytes:=bytes or ((getsupreg(oper[0]^.reg) and $8) shr 3) shl 7;
                      end
                    else
                      begin
                        bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                        bytes:=bytes or ((oper[2]^.val shr 2) and $7F);
                      end;
                  end;
                else
                  internalerror(2019050926);
              end;
            end;
          #$65: { Thumb load/store }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
              bytes:=bytes or (getsupreg(oper[1]^.ref^.base) shl 3);
              bytes:=bytes or (getsupreg(oper[1]^.ref^.index) shl 6);
            end;
          #$66: { Thumb load/store }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
              bytes:=bytes or (getsupreg(oper[1]^.ref^.base) shl 3);

              { set offset }
              offset:=0;
              currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
              if assigned(currsym) then
                offset:=currsym.offset-(insoffset+4) and (not longword(3));

              offset:=(offset+oper[1]^.ref^.offset);

              bytes:=bytes or (((offset shr ord(insentry^.code[3])) and $1F) shl 6);
            end;
          #$67: { Thumb load/store }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);
              { set regs }
              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);

              if oper[1]^.typ=top_ref then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-(insoffset+4) and (not longword(3));

                  offset:=(offset+oper[1]^.ref^.offset);

                  bytes:=bytes or ((offset shr ord(insentry^.code[3])) and $FF);
                end
              else
                bytes:=bytes or ((oper[1]^.val shr ord(insentry^.code[3])) and $FF);
            end;
          #$68: { Thumb CB[N]Z }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              { set opers }
              bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);

              if oper[1]^.typ=top_ref then
                begin
                  offset:=0;
                  currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[1]^.ref^.offset;

                  offset:=offset div 2;
                end
              else
                offset:=oper[1]^.val div 2;

              bytes:=bytes or ((offset) and $1F) shl 3;
              bytes:=bytes or ((offset shr 5) and 1) shl 9;
            end;
          #$69: { Thumb: Push/Pop/Stm/Ldm }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);

              case opcode of
                A_PUSH:
                  begin
                    for r:=0 to 7 do
                      if r in oper[0]^.regset^ then
                        bytes:=bytes or (1 shl r);
                    if RS_R14 in oper[0]^.regset^ then
                      bytes:=bytes or (1 shl 8);
                  end;
                A_POP:
                  begin
                    for r:=0 to 7 do
                      if r in oper[0]^.regset^ then
                        bytes:=bytes or (1 shl r);
                    if RS_R15 in oper[0]^.regset^ then
                      bytes:=bytes or (1 shl 8);
                  end;
                A_STM:
                  begin
                    for r:=0 to 7 do
                      if r in oper[1]^.regset^ then
                        bytes:=bytes or (1 shl r);

                    if oper[0]^.typ=top_ref then
                      bytes:=bytes or (getsupreg(oper[0]^.ref^.index) shl 8)
                    else
                      bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  end;
                A_LDM:
                  begin
                    for r:=0 to 7 do
                      if r in oper[1]^.regset^ then
                        bytes:=bytes or (1 shl r);

                    if oper[0]^.typ=top_ref then
                      bytes:=bytes or (getsupreg(oper[0]^.ref^.index) shl 8)
                    else
                      bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  end;
                else
                  internalerror(2019050925);
              end;
            end;
          #$6A: { Thumb: IT }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or (ord(insentry^.code[2]) shl 0);

              bytes:=bytes or (CondVal[oper[0]^.cc] shl 4);

              i_field:=(bytes shr 4) and 1;
              i_field:=(i_field shl 1) or i_field;
              i_field:=(i_field shl 2) or i_field;

              bytes:=bytes or ((i_field and ord(insentry^.code[3])) xor (ord(insentry^.code[3]) shr 4));
            end;
          #$6B: { Thumb: Data processing (misc) }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);
              { set regs }
              if ops>=2 then
                begin
                  if oper[1]^.typ=top_const then
                    begin
                      bytes:=bytes or ((getsupreg(oper[0]^.reg) and $7) shl 8);
                      bytes:=bytes or (oper[1]^.val and $FF);
                    end
                  else if oper[1]^.typ=top_reg then
                    begin
                      bytes:=bytes or (getsupreg(oper[0]^.reg) and $7);
                      bytes:=bytes or (getsupreg(oper[1]^.reg) shl 3);
                    end;
                end
              else if ops=1 then
                begin
                  if oper[0]^.typ=top_const then
                    bytes:=bytes or (oper[0]^.val and $FF);
                end;
            end;
          #$6C: { Thumb: CPS }
            begin
              bytelen:=2;
              bytes:=0;

              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 8);
              bytes:=bytes or ord(insentry^.code[2]);

              if mfA in oper[0]^.modeflags then bytes:=bytes or (1 shl 2);
              if mfI in oper[0]^.modeflags then bytes:=bytes or (1 shl 1);
              if mfF in oper[0]^.modeflags then bytes:=bytes or (1 shl 0);
            end;
          #$80: { Thumb-2: Dataprocessing }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if ops=1 then
                begin
                  if oper[0]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16)
                  else if oper[0]^.typ=top_const then
                    bytes:=bytes or (oper[0]^.val and $F);
                end
              else if (ops=2) and
                 (opcode in [A_CMP,A_CMN,A_TEQ,A_TST]) then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);

                  if oper[1]^.typ=top_const then
                    encodethumbimm(oper[1]^.val)
                  else if oper[1]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);
                end
              else if (ops=3) and
                      (opcode in [A_CMP,A_CMN,A_TEQ,A_TST]) then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);

                  if oper[2]^.typ=top_shifterop then
                    setthumbshift(2)
                  else if oper[2]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[2]^.reg) shl 12);
                end
              else if (ops=2) and
                      (opcode in [A_REV,A_RBIT,A_REV16,A_REVSH,A_CLZ]) then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);
                end
              else if ops=2 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);

                  if oper[1]^.typ=top_const then
                    encodethumbimm(oper[1]^.val)
                  else if oper[1]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);
                end
              else if ops=3 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);

                  if oper[2]^.typ=top_const then
                    encodethumbimm(oper[2]^.val)
                  else if oper[2]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[2]^.reg) shl 0);
                end
              else if ops=4 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);
                  bytes:=bytes or (getsupreg(oper[2]^.reg) shl 0);

                  if oper[3]^.typ=top_shifterop then
                    setthumbshift(3)
                  else if oper[3]^.typ=top_reg then
                    bytes:=bytes or (getsupreg(oper[3]^.reg) shl 12);
                end;

              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20)
              else if oppostfix=PF_X then
                bytes:=bytes or (1 shl 4)
              else if oppostfix=PF_R then
                bytes:=bytes or (1 shl 4);
            end;
          #$81: { Thumb-2: Dataprocessing misc }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if ops=3 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);

                  if oper[2]^.typ=top_const then
                    begin
                      bytes:=bytes or (oper[2]^.val and $FF);
                      bytes:=bytes or ((oper[2]^.val and $700) shr 8) shl 12;
                      bytes:=bytes or ((oper[2]^.val and $800) shr 11) shl 26;
                    end;
                end
              else if ops=2 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);

                  offset:=0;
                  if oper[1]^.typ=top_const then
                    begin
                      offset:=oper[1]^.val;
                    end
                  else if oper[1]^.typ=top_ref then
                    begin
                      currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                      if assigned(currsym) then
                        offset:=currsym.offset-insoffset-8;
                      offset:=offset+oper[1]^.ref^.offset;

                      offset:=offset;
                    end;

                  bytes:=bytes or  (offset and $FF);
                  bytes:=bytes or ((offset and $700) shr 8) shl 12;
                  bytes:=bytes or ((offset and $800) shr 11) shl 26;
                  bytes:=bytes or ((offset and $F000) shr 12) shl 16;
                end;

              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$82: { Thumb-2: Shifts }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
              if oper[1]^.typ=top_reg then
                begin
                  offset:=2;
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);
                end
              else
                begin
                  offset:=1;
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 0);
                end;

              if oper[offset]^.typ=top_const then
                begin
                  bytes:=bytes or (oper[offset]^.val and $3) shl 6;
                  bytes:=bytes or (oper[offset]^.val and $1C) shl 10;
                end
              else if oper[offset]^.typ=top_reg then
                bytes:=bytes or (getsupreg(oper[offset]^.reg) shl 16);

              if (ops>=(offset+2)) and
                 (oper[offset+1]^.typ=top_const) then
                bytes:=bytes or (oper[offset+1]^.val and $1F);

              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$84: { Thumb-2: Shifts(width-1) }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
              if oper[1]^.typ=top_reg then
                begin
                  offset:=2;
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);
                end
              else
                offset:=1;

              if oper[offset]^.typ=top_const then
                begin
                  bytes:=bytes or (oper[offset]^.val and $3) shl 6;
                  bytes:=bytes or (oper[offset]^.val and $1C) shl 10;
                end;

              if (ops>=(offset+2)) and
                 (oper[offset+1]^.typ=top_const) then
                begin
                  if opcode in [A_BFI,A_BFC] then
                    i_field:=oper[offset+1]^.val+oper[offset]^.val-1
                  else
                    i_field:=oper[offset+1]^.val-1;

                  bytes:=bytes or (i_field and $1F);
                end;

              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20);
            end;
          #$83: { Thumb-2: Saturation }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
              bytes:=bytes or (oper[1]^.val and $1F);
              bytes:=bytes or (getsupreg(oper[2]^.reg) shl 16);

              if ops=4 then
                setthumbshift(3,true);
            end;
          #$85: { Thumb-2: Long multiplications }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if ops=4 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[2]^.reg) shl 16);
                  bytes:=bytes or (getsupreg(oper[3]^.reg) shl 0);
                end;

              if oppostfix=PF_S then
                bytes:=bytes or (1 shl 20)
              else if oppostfix=PF_X then
                bytes:=bytes or (1 shl 4);
            end;
          #$86: { Thumb-2: Extension ops }
            begin
              bytes:=0;
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if ops=2 then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);
                end
              else if ops=3 then
                begin
                  if oper[2]^.typ=top_shifterop then
                    begin
                      bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                      bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);
                      bytes:=bytes or ((oper[2]^.shifterop^.shiftimm shr 3) shl 4);
                    end
                  else
                    begin
                      bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                      bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);
                      bytes:=bytes or (getsupreg(oper[2]^.reg) shl 0);
                    end;
                end
              else if ops=4 then
                begin
                  if oper[3]^.typ=top_shifterop then
                    begin
                      bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);
                      bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);
                      bytes:=bytes or (getsupreg(oper[2]^.reg) shl 0);
                      bytes:=bytes or ((oper[3]^.shifterop^.shiftimm shr 3) shl 4);
                    end;
                end;
            end;
          #$87: { Thumb-2: PLD/PLI }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.ref^.base) shl 16;
              if getregtype(oper[0]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[0]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=offset+oper[0]^.ref^.offset;
                  if offset>=0 then
                    begin
                      { set U flag }
                      bytes:=bytes or (1 shl 23);
                      bytes:=bytes or (offset and $FFF);
                    end
                  else
                    begin
                      bytes:=bytes or ($3 shl 10);

                      offset:=-offset;
                      bytes:=bytes or (offset and $FF);
                    end;
                end
              else
                begin
                  bytes:=bytes or getsupreg(oper[0]^.ref^.index);
                  { set shift }
                  with oper[0]^.ref^ do
                    if shiftmode=SM_LSL then
                      bytes:=bytes or ((shiftimm and $1F) shl 4);
                end;
            end;
          #$88: { Thumb-2: LDR/STR }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or (ord(insentry^.code[4]) shl 0);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
              if getregtype(oper[1]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=(offset+oper[1]^.ref^.offset) shr ord(insentry^.code[5]);
                  if offset>=0 then
                    begin
                      if (offset>255) and
                         (not (opcode in [A_LDRT,A_LDRSBT,A_LDRSHT,A_LDRBT,A_LDRHT])) then
                        bytes:=bytes or (1 shl 23);

                      { set U flag }
                      if (oper[1]^.ref^.addressmode<>AM_OFFSET) then
                        begin
                          bytes:=bytes or (1 shl 9);
                          bytes:=bytes or (1 shl 11);
                        end;
                      bytes:=bytes or offset
                    end
                  else
                    begin
                      bytes:=bytes or (1 shl 11);

                      offset:=-offset;
                      bytes:=bytes or offset
                    end;
                end
              else
                begin
                  { set I flag }
                  bytes:=bytes or (1 shl 25);
                  bytes:=bytes or getsupreg(oper[1]^.ref^.index);
                  { set shift }
                  with oper[1]^.ref^ do
                    if shiftmode<>SM_None then
                      bytes:=bytes or ((shiftimm and $1F) shl 4);
                end;

              if not (opcode in [A_LDRT,A_LDRSBT,A_LDRSHT,A_LDRBT,A_LDRHT]) then
                begin
                  { set W bit }
                  if oper[1]^.ref^.addressmode<>AM_OFFSET then
                    bytes:=bytes or (1 shl 8);
                  { set P bit if necessary }
                  if oper[1]^.ref^.addressmode<>AM_POSTINDEXED then
                    bytes:=bytes or (1 shl 10);
                end;
            end;
          #$89: { Thumb-2: LDRD/STRD }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or (ord(insentry^.code[4]) shl 0);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              bytes:=bytes or getsupreg(oper[1]^.reg) shl 8;
              bytes:=bytes or getsupreg(oper[2]^.ref^.base) shl 16;
              if getregtype(oper[2]^.ref^.index)=R_INVALIDREGISTER then
                begin
                  { set offset }
                  offset:=0;
                  currsym:=objdata.symbolref(oper[2]^.ref^.symbol);
                  if assigned(currsym) then
                    offset:=currsym.offset-insoffset-8;
                  offset:=(offset+oper[2]^.ref^.offset) div 4;
                  if offset>=0 then
                    begin
                      { set U flag }
                      bytes:=bytes or (1 shl 23);
                      bytes:=bytes or offset
                    end
                  else
                    begin
                      offset:=-offset;
                      bytes:=bytes or offset
                    end;
                end
              else
                begin
                  message(asmw_e_invalid_opcode_and_operands);
                end;
              { set W bit }
              if oper[2]^.ref^.addressmode<>AM_OFFSET then
                bytes:=bytes or (1 shl 21);
              { set P bit if necessary }
              if oper[2]^.ref^.addressmode<>AM_POSTINDEXED then
                bytes:=bytes or (1 shl 24);
            end;
          #$8A: { Thumb-2: LDREX }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or (ord(insentry^.code[4]) shl 0);
              { set Rn and Rd }
              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;

              if (ops=2) and (opcode in [A_LDREX]) then
                begin
                  bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
                  if getregtype(oper[1]^.ref^.index)=R_INVALIDREGISTER then
                    begin
                      { set offset }
                      offset:=0;
                      currsym:=objdata.symbolref(oper[1]^.ref^.symbol);
                      if assigned(currsym) then
                        offset:=currsym.offset-insoffset-8;
                      offset:=(offset+oper[1]^.ref^.offset) div 4;
                      if offset>=0 then
                        begin
                          bytes:=bytes or offset
                        end
                      else
                        begin
                          message(asmw_e_invalid_opcode_and_operands);
                        end;
                    end
                  else
                    begin
                      message(asmw_e_invalid_opcode_and_operands);
                    end;
                end
              else if (ops=2) then
                begin
                  bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
                end
              else
                begin
                  bytes:=bytes or getsupreg(oper[1]^.reg) shl 8;
                  bytes:=bytes or getsupreg(oper[2]^.ref^.base) shl 16;
                end;
            end;
          #$8B: { Thumb-2: STREX }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or (ord(insentry^.code[4]) shl 0);
              { set Rn and Rd }
              if (ops=3) and (opcode in [A_STREX]) then
                begin
                  bytes:=bytes or getsupreg(oper[0]^.reg) shl 8;
                  bytes:=bytes or getsupreg(oper[1]^.reg) shl 12;
                  bytes:=bytes or getsupreg(oper[2]^.ref^.base) shl 16;
                  if getregtype(oper[2]^.ref^.index)=R_INVALIDREGISTER then
                    begin
                      { set offset }
                      offset:=0;
                      currsym:=objdata.symbolref(oper[2]^.ref^.symbol);
                      if assigned(currsym) then
                        offset:=currsym.offset-insoffset-8;
                      offset:=(offset+oper[2]^.ref^.offset) div 4;
                      if offset>=0 then
                        begin
                          bytes:=bytes or offset
                        end
                      else
                        begin
                          message(asmw_e_invalid_opcode_and_operands);
                        end;
                    end
                  else
                    begin
                      message(asmw_e_invalid_opcode_and_operands);
                    end;
                end
              else if (ops=3) then
                begin
                  bytes:=bytes or getsupreg(oper[0]^.reg) shl 0;
                  bytes:=bytes or getsupreg(oper[1]^.reg) shl 12;
                  bytes:=bytes or getsupreg(oper[2]^.ref^.base) shl 16;
                end
              else
                begin
                  bytes:=bytes or getsupreg(oper[0]^.reg) shl 0;
                  bytes:=bytes or getsupreg(oper[1]^.reg) shl 12;
                  bytes:=bytes or getsupreg(oper[2]^.reg) shl 8;
                  bytes:=bytes or getsupreg(oper[3]^.ref^.base) shl 16;
                end;
            end;
          #$8C: { Thumb-2: LDM/STM }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or (ord(insentry^.code[4]) shl 0);

              if oper[0]^.typ=top_reg then
                bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16)
              else
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.ref^.base) shl 16);
                  if oper[0]^.ref^.addressmode<>AM_OFFSET then
                    bytes:=bytes or (1 shl 21);
                end;

              for r:=0 to 15 do
                if r in oper[1]^.regset^ then
                  bytes:=bytes or (1 shl r);

              case oppostfix of
                PF_None,PF_IA,PF_FD: bytes:=bytes or ($1 shl 23);
                PF_DB,PF_EA: bytes:=bytes or ($2 shl 23);
              else
                message1(asmw_e_invalid_opcode_and_operands, '"Invalid Postfix"');
              end;
            end;
          #$8D: { Thumb-2: BL/BLX }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 8);
              { set offset }
              if oper[0]^.typ=top_const then
                offset:=(oper[0]^.val shr 1) and $FFFFFF
              else
                begin
                  currsym:=objdata.symbolref(oper[0]^.ref^.symbol);
                  if (currsym.bind<>AB_LOCAL) and (currsym.objsection<>objdata.CurrObjSec) then
                    begin
                      objdata.writereloc(oper[0]^.ref^.offset,0,currsym,RELOC_RELATIVE_24_THUMB);
                      offset:=$FFFFFE
                    end
                  else
                    offset:=((currsym.offset-insoffset-8) shr 1) and $FFFFFF;
                end;

              bytes:=bytes or ((offset shr 00) and $7FF) shl 0;
              bytes:=bytes or ((offset shr 11) and $3FF) shl 16;
              bytes:=bytes or (((offset shr 21) xor (offset shr 23) xor 1) and $1) shl 11;
              bytes:=bytes or (((offset shr 22) xor (offset shr 23) xor 1) and $1) shl 13;
              bytes:=bytes or ((offset shr 23) and $1) shl 26;
            end;
          #$8E: { Thumb-2: TBB/TBH }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);
              { set Rn and Rm }
              bytes:=bytes or getsupreg(oper[0]^.ref^.base) shl 16;

              if getregtype(oper[0]^.ref^.index)=R_INVALIDREGISTER then
                message(asmw_e_invalid_effective_address)
              else
                begin
                  bytes:=bytes or getsupreg(oper[0]^.ref^.index);

                  if (opcode=A_TBH) and
                     (oper[0]^.ref^.shiftmode<>SM_LSL) and
                     (oper[0]^.ref^.shiftimm<>1) then
                    message(asmw_e_invalid_effective_address);
                end;
            end;
          #$8F: { Thumb-2: CPSxx }
            begin
              { set opcode }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if (oper[0]^.typ=top_modeflags) then
                begin
                  if mfA in oper[0]^.modeflags then bytes:=bytes or (1 shl 7);
                  if mfI in oper[0]^.modeflags then bytes:=bytes or (1 shl 6);
                  if mfF in oper[0]^.modeflags then bytes:=bytes or (1 shl 5);
                end;

              if (ops=2) then
                bytes:=bytes or (oper[1]^.val and $1F)
              else if (ops=1) and
                      (oper[0]^.typ=top_const) then
                bytes:=bytes or (oper[0]^.val and $1F);
            end;
          #$96: { Thumb-2: MSR/MRS }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if opcode=A_MRS then
                begin
                  bytes:=bytes or (getsupreg(oper[0]^.reg) shl 8);

                  case oper[1]^.reg of
                    NR_MSP: bytes:=bytes or $08;
                    NR_PSP: bytes:=bytes or $09;

                    NR_IPSR: bytes:=bytes or $05;
                    NR_EPSR: bytes:=bytes or $06;
                    NR_APSR: bytes:=bytes or $00;

                    NR_PRIMASK: bytes:=bytes or $10;
                    NR_BASEPRI: bytes:=bytes or $11;
                    NR_BASEPRI_MAX: bytes:=bytes or $12;
                    NR_FAULTMASK: bytes:=bytes or $13;
                    NR_CONTROL: bytes:=bytes or $14;
                  else
                    Message(asmw_e_invalid_opcode_and_operands);
                  end;
                end
              else
                begin
                  bytes:=bytes or (getsupreg(oper[1]^.reg) shl 16);

                  case oper[0]^.reg of
                    NR_APSR,
                    NR_APSR_nzcvqg: bytes:=bytes or $C00;
                    NR_APSR_g: bytes:=bytes or $400;
                    NR_APSR_nzcvq: bytes:=bytes or $800;

                    NR_MSP: bytes:=bytes or $08;
                    NR_PSP: bytes:=bytes or $09;

                    NR_PRIMASK: bytes:=bytes or $10;
                    NR_BASEPRI: bytes:=bytes or $11;
                    NR_BASEPRI_MAX: bytes:=bytes or $12;

                    NR_FAULTMASK: bytes:=bytes or $13;
                    NR_CONTROL: bytes:=bytes or $14;
                  else
                    Message(asmw_e_invalid_opcode_and_operands);
                  end;
                end;
            end;
          #$A0: { FPA: CPDT(LDF/STF) }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or (ord(insentry^.code[3]) shl 8);
              bytes:=bytes or ord(insentry^.code[4]);

              if ops=2 then
                begin
                  bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;

                  bytes:=bytes or getsupreg(oper[1]^.ref^.base) shl 16;
                  bytes:=bytes or ((oper[1]^.ref^.offset shr 2) and $FF);
                  if oper[1]^.ref^.offset>=0 then
                    bytes:=bytes or (1 shl 23);

                  if oper[1]^.ref^.addressmode<>AM_OFFSET then
                    bytes:=bytes or (1 shl 21);
                  if oper[1]^.ref^.addressmode=AM_PREINDEXED then
                    bytes:=bytes or (1 shl 24);

                  case oppostfix of
                    PF_S: bytes:=bytes or (0 shl 22) or (0 shl 15);
                    PF_D: bytes:=bytes or (0 shl 22) or (1 shl 15);
                    PF_E: bytes:=bytes or (1 shl 22) or (0 shl 15);
                    PF_P: bytes:=bytes or (1 shl 22) or (1 shl 15);
                    PF_EP: ;
                    else
                      message1(asmw_e_invalid_opcode_and_operands, '"Invalid postfix"');
                  end;
                end
              else
                begin
                  bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;

                  case oper[1]^.val of
                    1: bytes:=bytes or (1 shl 15);
                    2: bytes:=bytes or (1 shl 22);
                    3: bytes:=bytes or (1 shl 22) or (1 shl 15);
                    4: ;
                  else
                    message1(asmw_e_invalid_opcode_and_operands, 'Invalid count for LFM/SFM');
                  end;

                  bytes:=bytes or getsupreg(oper[2]^.ref^.base) shl 16;
                  bytes:=bytes or ((oper[2]^.ref^.offset shr 2) and $FF);
                  if oper[2]^.ref^.offset>=0 then
                    bytes:=bytes or (1 shl 23);

                  if oper[2]^.ref^.addressmode<>AM_OFFSET then
                    bytes:=bytes or (1 shl 21);
                  if oper[2]^.ref^.addressmode=AM_PREINDEXED then
                    bytes:=bytes or (1 shl 24);
                end;
            end;
          #$A1: { FPA: CPDO }
            begin
              { set instruction code }
              bytes:=bytes or ($E shl 24);
              bytes:=bytes or (ord(insentry^.code[1]) shl 15);
              bytes:=bytes or ((ord(insentry^.code[2]) shr 1) shl 20);
              bytes:=bytes or (1 shl 8);

              bytes:=bytes or getsupreg(oper[0]^.reg) shl 12;
              if ops=2 then
                begin
                  if oper[1]^.typ=top_reg then
                    bytes:=bytes or getsupreg(oper[1]^.reg) shl 0
                  else
                    case oper[1]^.val of
                      0: bytes:=bytes or $8;
                      1: bytes:=bytes or $9;
                      2: bytes:=bytes or $A;
                      3: bytes:=bytes or $B;
                      4: bytes:=bytes or $C;
                      5: bytes:=bytes or $D;
                      //0.5: bytes:=bytes or $E;
                      10: bytes:=bytes or $F;
                    else
                      Message(asmw_e_invalid_opcode_and_operands);
                    end;
                end
              else
                begin
                  bytes:=bytes or getsupreg(oper[1]^.reg) shl 16;
                  if oper[2]^.typ=top_reg then
                    bytes:=bytes or getsupreg(oper[2]^.reg) shl 0
                  else
                    case oper[2]^.val of
                      0: bytes:=bytes or $8;
                      1: bytes:=bytes or $9;
                      2: bytes:=bytes or $A;
                      3: bytes:=bytes or $B;
                      4: bytes:=bytes or $C;
                      5: bytes:=bytes or $D;
                      //0.5: bytes:=bytes or $E;
                      10: bytes:=bytes or $F;
                    else
                      Message(asmw_e_invalid_opcode_and_operands);
                    end;
                end;

              case roundingmode of
                RM_NONE: ;
                RM_P: bytes:=bytes or (1 shl 5);
                RM_M: bytes:=bytes or (2 shl 5);
                RM_Z: bytes:=bytes or (3 shl 5);
              end;

              case oppostfix of
                PF_S: bytes:=bytes or (0 shl 19) or (0 shl 7);
                PF_D: bytes:=bytes or (0 shl 19) or (1 shl 7);
                PF_E: bytes:=bytes or (1 shl 19) or (0 shl 7);
              else
                message1(asmw_e_invalid_opcode_and_operands, 'Precision cannot be undefined');
              end;
            end;
          #$A2: { FPA: CPDO }
            begin
              { set instruction code }
              bytes:=bytes or (ord(insentry^.code[1]) shl 24);
              bytes:=bytes or (ord(insentry^.code[2]) shl 16);
              bytes:=bytes or ($11 shl 4);

              case opcode of
                A_FLT:
                  begin
                    bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);
                    bytes:=bytes or (getsupreg(oper[1]^.reg) shl 12);

                    case roundingmode of
                      RM_NONE: ;
                      RM_P: bytes:=bytes or (1 shl 5);
                      RM_M: bytes:=bytes or (2 shl 5);
                      RM_Z: bytes:=bytes or (3 shl 5);
                    end;

                    case oppostfix of
                      PF_S: bytes:=bytes or (0 shl 19) or (0 shl 7);
                      PF_D: bytes:=bytes or (0 shl 19) or (1 shl 7);
                      PF_E: bytes:=bytes or (1 shl 19) or (0 shl 7);
                    else
                      message1(asmw_e_invalid_opcode_and_operands, 'Precision cannot be undefined');
                    end;
                  end;
                A_FIX:
                  begin
                    bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
                    bytes:=bytes or (getsupreg(oper[1]^.reg) shl 0);

                    case roundingmode of
                      RM_NONE: ;
                      RM_P: bytes:=bytes or (1 shl 5);
                      RM_M: bytes:=bytes or (2 shl 5);
                      RM_Z: bytes:=bytes or (3 shl 5);
                    end;
                  end;
                A_WFS,A_RFS,A_WFC,A_RFC:
                  begin
                    bytes:=bytes or (getsupreg(oper[0]^.reg) shl 12);
                  end;
                A_CMF,A_CNF,A_CMFE,A_CNFE:
                  begin
                    bytes:=bytes or (getsupreg(oper[0]^.reg) shl 16);

                    if oper[1]^.typ=top_reg then
                      bytes:=bytes or getsupreg(oper[1]^.reg) shl 0
                    else
                      case oper[1]^.val of
                        0: bytes:=bytes or $8;
                        1: bytes:=bytes or $9;
                        2: bytes:=bytes or $A;
                        3: bytes:=bytes or $B;
                        4: bytes:=bytes or $C;
                        5: bytes:=bytes or $D;
                        //0.5: bytes:=bytes or $E;
                        10: bytes:=bytes or $F;
                      else
                        Message(asmw_e_invalid_opcode_and_operands);
                      end;
                  end;
                else
                  Message1(asmw_e_invalid_opcode_and_operands, '"Unsupported opcode"');
              end;
            end;
          #$fe: // No written data
            begin
              exit;
            end;
          #$ff:
            internalerror(2005091101);
          else
            begin
              writeln(ord(insentry^.code[0]), ' - ', opcode);
              internalerror(2005091102);
            end;
        end;

        { Todo: Decide whether the code above should take care of writing data in an order that makes senes }
        if (insentry^.code[0] in [#$80..#$96]) and (bytelen=4) then
          bytes:=((bytes shr 16) and $FFFF) or ((bytes and $FFFF) shl 16);

        { we're finished, write code }
        objdata.writebytes(bytes,bytelen);
      end;

begin
  cai_align:=tai_align;
end.

