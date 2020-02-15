{
    Copyright (c) 1999-2008 by Mazen Neifer and Florian Klaempfl

    Contains the assembler object for the AVR

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
  cclasses,
  globtype,globals,verbose,
  aasmbase,aasmtai,aasmdata,aasmsym,
  cgbase,cgutils,cpubase,cpuinfo,
  ogbase;

    const
      { "mov reg,reg" source operand number }
      O_MOV_SOURCE = 1;
      { "mov reg,reg" source operand number }
      O_MOV_DEST = 0;

      maxinfolen = 5;

    type
      tinsentry = record
        opcode  : tasmop;
        ops     : byte;
        optypes : array[0..3] of longint;
        code    : array[0..maxinfolen] of char;
        flags   : longint;
      end;

      pinsentry=^tinsentry;

      taicpu = class(tai_cpu_abstract_sym)
         constructor op_none(op : tasmop);

         constructor op_reg(op : tasmop;_op1 : tregister);
         constructor op_const(op : tasmop;_op1 : LongInt);
         constructor op_ref(op : tasmop;const _op1 : treference);

         constructor op_reg_reg(op : tasmop;_op1,_op2 : tregister);
         constructor op_reg_ref(op : tasmop;_op1 : tregister;const _op2 : treference);
         constructor op_reg_const(op:tasmop; _op1: tregister; _op2: LongInt);
         constructor op_const_reg(op:tasmop; _op1: LongInt; _op2: tregister);
         constructor op_ref_reg(op : tasmop;const _op1 : treference;_op2 : tregister);

         { this is for Jmp instructions }
         constructor op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);
         constructor op_sym(op : tasmop;_op1 : tasmsymbol);
         constructor op_sym_ofs(op : tasmop;_op1 : tasmsymbol;_op1ofs:longint);
         procedure loadbool(opidx:longint;_b:boolean);
         { register allocation }
         function is_same_reg_move(regtype: Tregistertype):boolean; override;

         { register spilling code }
         function spilling_get_operation_type(opnr: longint): topertype;override;

         { assembler }
      public
         { the next will reset all instructions that can change in pass 2 }
         procedure ResetPass1;override;
         procedure ResetPass2;override;
{         function  CheckIfValid:boolean;
         function GetString:string; }
         function  Pass1(objdata:TObjData):longint;override;
//         procedure Pass2(objdata:TObjData);override;
         function calcsize(p:PInsEntry):shortint;
      private
         { next fields are filled in pass1, so pass2 is faster }
         inssize   : shortint;
         insoffset : longint;
         insentry  : PInsEntry;
         LastInsOffset : longint; { need to be public to be reset }
         function  FindInsentry(objdata:TObjData):boolean;
      end;

      tai_align = class(tai_align_abstract)
        { nothing to add }
      end;

    procedure InitAsm;
    procedure DoneAsm;

    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
    function spilling_create_store(r:tregister; const ref:treference):Taicpu;

    function setcondition(i : taicpu;c : tasmcond) : taicpu;

    { replaces cond. branches by rjmp/jmp and the inverse cond. branch if needed
      and transforms special instructions to valid instruction encodings }
    function finalizeavrcode(list : TAsmList) : Boolean;

implementation

{*****************************************************************************
                                 taicpu Constructors
*****************************************************************************}

    procedure taicpu.loadbool(opidx:longint;_b:boolean);
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


    constructor taicpu.op_ref(op : tasmop;const _op1 : treference);
      begin
         inherited create(op);
         ops:=1;
         loadref(0,_op1);
      end;


    constructor taicpu.op_const(op : tasmop;_op1 : LongInt);
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

    constructor taicpu.op_reg_const(op:tasmop; _op1: tregister; _op2: LongInt);
      begin
         inherited create(op);
         ops:=2;
         loadreg(0,_op1);
         loadconst(1,aint(_op2));
      end;

     constructor taicpu.op_const_reg(op:tasmop; _op1: LongInt; _op2: tregister);
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


    constructor taicpu.op_ref_reg(op : tasmop;const _op1 : treference;_op2 : tregister);
      begin
         inherited create(op);
         ops:=2;
         loadref(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_cond_sym(op : tasmop;cond:TAsmCond;_op1 : tasmsymbol);
      begin
         inherited create(op);
         is_jmp:=op in jmp_instructions;
         condition:=cond;
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym(op : tasmop;_op1 : tasmsymbol);
      begin
         inherited create(op);
         is_jmp:=op in jmp_instructions;
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym_ofs(op : tasmop;_op1 : tasmsymbol;_op1ofs:longint);
      begin
         inherited create(op);
         ops:=1;
         loadsymbol(0,_op1,_op1ofs);
      end;


    function taicpu.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        result:=(
                 ((opcode in [A_MOV,A_MOVW]) and (regtype = R_INTREGISTER))
                ) and
                (ops=2) and
                (oper[0]^.typ=top_reg) and
                (oper[1]^.typ=top_reg) and
                (oper[0]^.reg=oper[1]^.reg);
      end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        result:=operand_read;
        case opcode of
          A_CLR,A_LDD,A_LD,A_LDI,A_LDS,
          A_MOV,A_MOVW,A_POP:
            if opnr=0 then
              result:=operand_write;
          A_CP,A_CPC,A_CPI,A_PUSH,A_SBRC,A_SBRS,A_ST,A_STD,A_STS,
          A_MUL,A_MULS,A_MULSU,A_FMUL,A_FMULS,A_FMULSU:
            ;
          else
            begin
              if opnr=0 then
                result:=operand_readwrite;
            end;
        end;
      end;


    function  taicpu.calcsize(p:PInsEntry):shortint;
      begin
        case opcode of
          A_CALL,
          A_JMP:
            result:=4;
          A_LDS:
            if (getsupreg(oper[0]^.reg)>=RS_R16) and (getsupreg(oper[0]^.reg)<=RS_R31) and
              (oper[1]^.val>=0) and (oper[1]^.val<=127) then
              result:=2
            else
              result:=4;
          A_STS:
            if (getsupreg(oper[1]^.reg)>=RS_R16) and (getsupreg(oper[1]^.reg)<=RS_R31) and
              (oper[0]^.val>=0) and (oper[0]^.val<=127) then
              result:=2
            else
              result:=4;
        else
          result:=2;
        end;
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
{
        if assigned(InsEntry) and
           ((InsEntry^.flags and IF_PASS2)<>0) then
         begin
           InsEntry:=nil;
           InsSize:=0;
         end;
}
        LastInsOffset:=-1;
      end;


    function taicpu.FindInsentry(objdata:TObjData):boolean;
      begin
        result:=false;
      end;


    function taicpu.Pass1(objdata:TObjData):longint;
      begin
        Pass1:=0;
        { Save the old offset and set the new offset }
        InsOffset:=ObjData.CurrObjSec.Size;
        InsSize:=calcsize(InsEntry);
        { Error? }
        if (Insentry=nil) and (InsSize=-1) then
          exit;
        { set the file postion }
        current_filepos:=fileinfo;

        { Get InsEntry }
        if FindInsEntry(objdata) then
         begin
           LastInsOffset:=InsOffset;
           Pass1:=InsSize;
           exit;
         end;
        LastInsOffset:=-1;
      end;


    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER :
            if ref.offset<>0 then
              result:=taicpu.op_reg_ref(A_LDD,r,ref)
            else
              result:=taicpu.op_reg_ref(A_LD,r,ref);
          R_ADDRESSREGISTER :
            if ref.offset<>0 then
              result:=taicpu.op_reg_ref(A_LDD,r,ref)
            else
              result:=taicpu.op_reg_ref(A_LD,r,ref);
          else
            internalerror(200401041);
        end;
      end;


    function spilling_create_store(r:tregister; const ref:treference):Taicpu;
      begin
        case getregtype(r) of
          R_INTREGISTER :
            if ref.offset<>0 then
              result:=taicpu.op_ref_reg(A_STD,ref,r)
            else
              result:=taicpu.op_ref_reg(A_ST,ref,r);
          R_ADDRESSREGISTER :
            if ref.offset<>0 then
              result:=taicpu.op_ref_reg(A_STD,ref,r)
            else
              result:=taicpu.op_ref_reg(A_ST,ref,r);
          else
            internalerror(200401041);
        end;
      end;


    procedure InitAsm;
      begin
      end;


    procedure DoneAsm;
      begin
      end;


    function setcondition(i : taicpu;c : tasmcond) : taicpu;
      begin
        i.condition:=c;
        result:=i;
      end;


    function finalizeavrcode(list : TAsmList) : Boolean;
      var
        CurrOffset : longint;
        curtai, firstinstruction: tai;
        again : boolean;
        l : tasmlabel;
        inasmblock : Boolean;

      procedure remove_instruction;
        var
          i: Integer;
          hp: tai;
        begin
          taicpu(firstinstruction).opcode:=A_SLEEP;
          for i:=0 to taicpu(firstinstruction).opercnt-1 do
            taicpu(firstinstruction).freeop(i);
          taicpu(firstinstruction).opercnt:=0;
          taicpu(firstinstruction).ops:=0;
          firstinstruction:=tai(firstinstruction.Next);
          while assigned(firstinstruction) do
            begin
              if firstinstruction.typ in [ait_symbol_end,ait_label] then
                firstinstruction:=tai(firstinstruction.Next)
              else
                begin
                  hp:=tai(firstinstruction.Next);
                  list.Remove(firstinstruction);
                  firstinstruction.free;
                  firstinstruction:=hp;
                end;
            end;
        end;

      begin
        again:=true;
        Result:=true;
        while again do
          begin
            again:=false;
            CurrOffset:=0;
            curtai:=tai(list.first);
            while assigned(curtai) do
              begin
                { instruction? }
                if not(curtai.typ in SkipInstr) then
                  case curtai.typ of
                    ait_instruction:
                      begin
                        taicpu(curtai).InsOffset:=CurrOffset;
                        inc(CurrOffset,taicpu(curtai).calcsize(nil));
                      end;
                    ait_align:
                      inc(CurrOffset,tai_align(curtai).aligntype);
                    ait_const:
                      inc(CurrOffset,tai_const(curtai).size);
                    ait_symbolpair,
                    ait_marker:
                      ;
                    ait_label:
                      begin
                        tai_label(curtai).labsym.offset:=CurrOffset;
                      end;
                    else
                      internalerror(2011082401);
                  end;
                curtai:=tai(curtai.next);
              end;

            curtai:=tai(list.first);
            inasmblock:=false;
            firstinstruction:=nil;
            while assigned(curtai) do
              begin
                case curtai.typ of
                  ait_instruction:
                    begin
                      if not(assigned(firstinstruction)) then
                        firstinstruction:=curtai;
                      case taicpu(curtai).opcode of
                        A_BRxx:
                          if (taicpu(curtai).oper[0]^.typ=top_ref) and ((taicpu(curtai).InsOffset-taicpu(curtai).oper[0]^.ref^.symbol.offset>64) or
                            (taicpu(curtai).InsOffset-taicpu(curtai).oper[0]^.ref^.symbol.offset<-63)) then
                            begin
                              if inasmblock then
                                Message(asmw_e_brxx_out_of_range)
                              else
                                begin
                                  current_asmdata.getjumplabel(l);
                                  list.insertafter(tai_label.create(l),curtai);
                                  if CPUAVR_HAS_JMP_CALL in cpu_capabilities[current_settings.cputype] then
                                    list.insertafter(taicpu.op_sym(A_JMP,taicpu(curtai).oper[0]^.ref^.symbol),curtai)
                                  else
                                    list.insertafter(taicpu.op_sym(A_RJMP,taicpu(curtai).oper[0]^.ref^.symbol),curtai);
                                  taicpu(curtai).oper[0]^.ref^.symbol:=l;
                                  taicpu(curtai).condition:=inverse_cond(taicpu(curtai).condition);
                                  again:=true;
                                end;
                            end;
                        A_JMP:
                          { replace JMP by RJMP? ...
                            ... but do not mess with asm block }
                          if not(inasmblock) and (taicpu(curtai).InsOffset-taicpu(curtai).oper[0]^.ref^.symbol.offset<=2048) and
                          (taicpu(curtai).InsOffset-taicpu(curtai).oper[0]^.ref^.symbol.offset>=-2047) and
                          { jmps to function go outside the currently considered scope, so do not mess with them.
                            Those are generated by the peephole optimizer from call/ret sequences }
                          not(taicpu(curtai).oper[0]^.ref^.symbol.typ=AT_FUNCTION) then
                          begin
                            taicpu(curtai).opcode:=A_RJMP;
                            again:=true;
                          end;
                        A_STS:
                          begin
                            if current_settings.cputype in [cpu_avrtiny, cpu_avr1] then
                              with taicpu(curtai).oper[0]^ do
                                if (ref^.base=NR_NO) and (ref^.index=NR_NO) and (ref^.symbol=nil) and (ref^.offset<$40) then
                                  begin
                                    taicpu(curtai).opcode:=A_OUT;
                                    taicpu(curtai).loadconst(0,ref^.offset);
                                  end
                                else if current_settings.cputype=cpu_avr1 then
                                  begin
                                    remove_instruction;
                                    result:=false;
                                  end;
                          end;
                        A_LDS:
                          begin
                            if current_settings.cputype in [cpu_avrtiny, cpu_avr1] then
                              with taicpu(curtai).oper[1]^ do
                                if (ref^.base=NR_NO) and (ref^.index=NR_NO) and (ref^.symbol=nil) and (ref^.offset<$40) then
                                  begin
                                    taicpu(curtai).opcode:=A_IN;
                                    taicpu(curtai).loadconst(1,ref^.offset)
                                  end
                                else if current_settings.cputype=cpu_avr1 then
                                  begin
                                    remove_instruction;
                                    result:=false;
                                  end;
                          end;
                        A_SBIW,
                        A_MULS,
                        A_ICALL,
                        A_IJMP,
                        A_STD,
                        A_LD,
                        A_LDD,
                        A_ST,
                        A_ROR,
                        A_POP,
                        A_PUSH:
                          begin
                            { certain cpu types do not support some instructions, so replace them }
                            if current_settings.cputype=cpu_avr1 then
                              begin
                                remove_instruction;
                                result:=false;
                              end;
                          end;
                      end;
                    end;
                  ait_marker:
                    case tai_marker(curtai).Kind of
                      mark_AsmBlockStart:
                        inasmblock:=true;
                      mark_AsmBlockEnd:
                        inasmblock:=false;
                    end;
                end;
                curtai:=tai(curtai.next);
              end;
          end;
      end;


begin
  cai_cpu:=taicpu;
  cai_align:=tai_align;
end.
