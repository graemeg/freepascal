{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit implements the common RiscV optimizer object

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

unit aoptcpurv;

interface

{$I fpcdefs.inc}

{$define DEBUG_AOPTCPU}

uses
  cpubase,
  globals, globtype,
  cgbase,
  aoptobj, aoptcpub, aopt,
  aasmtai, aasmcpu;

type

  TRVCpuAsmOptimizer = class(TAsmOptimizer)
    function InstructionLoadsFromReg(const reg: TRegister; const hp: tai): boolean; override;
    function RegLoadedWithNewValue(reg: tregister; hp: tai): boolean; override;
    function RegModifiedByInstruction(Reg: TRegister; p1: tai): boolean; override;
    Function GetNextInstructionUsingReg(Current: tai; Out Next: tai; reg: TRegister): Boolean;
    { outputs a debug message into the assembler file }
    procedure DebugMsg(const s: string; p: tai);

    function PeepHoleOptPass1Cpu(var p: tai): boolean; override;
  end;

implementation

  uses
    cutils;

  function MatchInstruction(const instr: tai; const op: TAsmOps; const AConditions: TAsmConds = []): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        (taicpu(instr).opcode in op) and
        ((AConditions=[]) or (taicpu(instr).condition in AConditions));
    end;


  function MatchInstruction(const instr: tai; const op: TAsmOp; const AConditions: TAsmConds = []): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        (taicpu(instr).opcode = op) and
        ((AConditions=[]) or (taicpu(instr).condition in AConditions));
    end;


  function MatchOperand(const oper1: TOper; const oper2: TOper): boolean; inline;
    begin
      result := oper1.typ = oper2.typ;

      if result then
        case oper1.typ of
          top_const:
            Result:=oper1.val = oper2.val;
          top_reg:
            Result:=oper1.reg = oper2.reg;
          {top_ref:
            Result:=RefsEqual(oper1.ref^, oper2.ref^);}
          else Result:=false;
        end
    end;


  function MatchOperand(const oper: TOper; const reg: TRegister): boolean; inline;
    begin
      result := (oper.typ = top_reg) and (oper.reg = reg);
    end;


{$ifdef DEBUG_AOPTCPU}
  procedure TRVCpuAsmOptimizer.DebugMsg(const s: string;p : tai);
    begin
      asml.insertbefore(tai_comment.Create(strpnew(s)), p);
    end;
{$else DEBUG_AOPTCPU}
  procedure TRVCpuAsmOptimizer.DebugMsg(const s: string;p : tai);inline;
    begin
    end;
{$endif DEBUG_AOPTCPU}


  function TRVCpuAsmOptimizer.InstructionLoadsFromReg(const reg: TRegister; const hp: tai): boolean;
    var
      p: taicpu;
      i: longint;
    begin
      result:=false;
      if not (assigned(hp) and (hp.typ=ait_instruction)) then
        exit;
      p:=taicpu(hp);

      i:=0;
      while(i<p.ops) do
        begin
          case p.oper[I]^.typ of
            top_reg:
              result:=(p.oper[I]^.reg=reg) and (p.spilling_get_operation_type(i)<>operand_write);
            top_ref:
              result:=
                (p.oper[I]^.ref^.base=reg);
            else
              ;
          end;
          if result then exit; {Bailout if we found something}
          Inc(I);
        end;
    end;


  function TRVCpuAsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
    begin
      result:=
        (hp.typ=ait_instruction) and
        (taicpu(hp).ops>1) and
        (taicpu(hp).oper[0]^.typ=top_reg) and
        (taicpu(hp).oper[0]^.reg=reg) and
        (taicpu(hp).spilling_get_operation_type(0)<>operand_read);
    end;


  function TRVCpuAsmOptimizer.RegModifiedByInstruction(Reg: TRegister; p1: tai): boolean;
    var
      i : Longint;
    begin
      result:=false;
      for i:=0 to taicpu(p1).ops-1 do
        case taicpu(p1).oper[i]^.typ of
          top_reg:
            if (taicpu(p1).oper[i]^.reg=Reg) and (taicpu(p1).spilling_get_operation_type(i) in [operand_write,operand_readwrite]) then
              exit(true);
          else
            ;
        end;
    end;


  function TRVCpuAsmOptimizer.GetNextInstructionUsingReg(Current: tai; out Next: tai; reg: TRegister): Boolean;
    begin
      Next:=Current;
      repeat
        Result:=GetNextInstruction(Next,Next);
      until not (Result) or
            not(cs_opt_level3 in current_settings.optimizerswitches) or
            (Next.typ<>ait_instruction) or
            RegInInstruction(reg,Next) or
            is_calljmp(taicpu(Next).opcode);
    end;


  function TRVCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;

    procedure RemoveInstr(var orig: tai; moveback: boolean = true);
      var
        n: tai;
      begin
        if moveback and (not GetLastInstruction(orig,n)) then
          GetNextInstruction(orig,n);

        AsmL.Remove(orig);
        orig.Free;

        orig:=n;
      end;

    var
      hp1: tai;
    begin
      result:=false;
      case p.typ of
        ait_instruction:
          begin
            case taicpu(p).opcode of
              A_ADDI:
                begin
                  {
                    Changes
                      addi x, y, #
                      addi/addiw z, x, #
                      dealloc x
                    To
                      addi z, y, #+#
                  }
                  if (taicpu(p).ops=3) and
                     (taicpu(p).oper[2]^.typ=top_const) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1,[A_ADDI{$ifdef riscv64},A_ADDIW{$endif}]) and
                     (taicpu(hp1).ops=3) and
                     MatchOperand(taicpu(p).oper[0]^,taicpu(hp1).oper[1]^) and
                     (taicpu(p).oper[2]^.typ=top_const) and
                     is_imm12(taicpu(p).oper[2]^.val+taicpu(hp1).oper[2]^.val) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadreg(1,taicpu(p).oper[1]^.reg);
                      taicpu(hp1).loadconst(2, taicpu(p).oper[2]^.val+taicpu(hp1).oper[2]^.val);

                      DebugMsg('Peephole AddiAddi2Addi performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end
                  {
                    Changes
                      addi x, z, (ref)
                      ld/sd y, 0(x)
                      dealloc x
                    To
                      ld/sd y, 0(ref)(x)
                  }
                  else if (taicpu(p).ops=3) and
                     (taicpu(p).oper[2]^.typ=top_ref) and
                     MatchOperand(taicpu(p).oper[0]^,taicpu(p).oper[1]^) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1, [A_LB,A_LBU,A_LH,A_LHU,A_LW,
                                             A_SB,A_SH,A_SW{$ifdef riscv64},A_LD,A_LWU,A_SD{$endif}]) and
                     (taicpu(hp1).ops=2) and
                     (taicpu(hp1).oper[1]^.typ=top_ref) and
                     (taicpu(hp1).oper[1]^.ref^.base=taicpu(p).oper[0]^.reg) and
                     (taicpu(hp1).oper[1]^.ref^.offset=0) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadref(1,taicpu(p).oper[2]^.ref^);
                      taicpu(hp1).oper[1]^.ref^.base:=taicpu(p).oper[1]^.reg;

                      DebugMsg('Peephole AddiMem2Mem performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end
                  {
                    Changes
                      addi x, z, #w
                      ld/sd y, 0(x)
                      dealloc x
                    To
                      ld/sd y, #w(z)
                  }
                  else if (taicpu(p).ops=3) and
                     (taicpu(p).oper[2]^.typ=top_const) and
                     //MatchOperand(taicpu(p).oper[0]^,taicpu(p).oper[1]^) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1, [A_LB,A_LBU,A_LH,A_LHU,A_LW,
                                             A_SB,A_SH,A_SW{$ifdef riscv64},A_LWU,A_LD,A_SD{$endif}]) and
                     (taicpu(hp1).ops=2) and
                     (taicpu(hp1).oper[1]^.typ=top_ref) and
                     (taicpu(hp1).oper[1]^.ref^.base=taicpu(p).oper[0]^.reg) and
                     (taicpu(hp1).oper[1]^.ref^.offset=0) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      //taicpu(hp1).loadconst(1,taicpu(p).oper[2]^.ref^);
                      taicpu(hp1).oper[1]^.ref^.offset:=taicpu(p).oper[2]^.val;
                      taicpu(hp1).oper[1]^.ref^.base:=taicpu(p).oper[1]^.reg;

                      DebugMsg('Peephole AddiMem2Mem performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end;
                end;
              A_SUB:
                begin
                  {
                    Turn
                      sub x,y,z
                      bgeu X0,x,...
                      dealloc x
                    Into
                      bne y,x,...
                  }
                  if (taicpu(p).ops=3) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1,A_Bxx,[C_GEU,C_EQ]) and
                     (taicpu(hp1).ops=3) and
                     MatchOperand(taicpu(hp1).oper[0]^,NR_X0) and
                     MatchOperand(taicpu(hp1).oper[1]^,taicpu(p).oper[0]^) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     (not RegModifiedBetween(taicpu(p).oper[2]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadreg(0,taicpu(p).oper[1]^.reg);
                      taicpu(hp1).loadreg(1,taicpu(p).oper[2]^.reg);
                      taicpu(hp1).condition:=C_EQ;

                      DebugMsg('Peephole SubBxx2Beq performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end;
                end;
              A_SLT,
              A_SLTU:
                begin
                  {
                    Turn
                      sltu x,X0,y
                      beq/bne x, X0, ...
                      dealloc x
                    Into
                      bltu/geu X0, y, ...
                  }
                  if (taicpu(p).ops=3) and
                     MatchOperand(taicpu(p).oper[1]^,NR_X0) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1,A_Bxx,[C_NE,C_EQ]) and
                     (taicpu(hp1).ops=3) and
                     MatchOperand(taicpu(hp1).oper[0]^,taicpu(p).oper[0]^) and
                     MatchOperand(taicpu(hp1).oper[1]^,NR_X0) and
                     (not RegModifiedBetween(taicpu(p).oper[2]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadreg(0,NR_X0);
                      taicpu(hp1).loadreg(1,taicpu(p).oper[2]^.reg);

                      if taicpu(p).opcode=A_SLTU then
                        begin
                          if taicpu(hp1).condition=C_NE then
                            taicpu(hp1).condition:=C_LTU
                          else
                            taicpu(hp1).condition:=C_GEU;
                        end
                      else
                        begin    
                          if taicpu(hp1).condition=C_NE then
                            taicpu(hp1).condition:=C_LT
                          else
                            taicpu(hp1).condition:=C_GE;
                        end;

                      DebugMsg('Peephole SltuB2B performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end
                  {
                    Turn
                      sltu x,y,z
                      beq/bne x, X0, ...
                      dealloc x
                    Into
                      bltu/geu y, z, ...
                  }
                  else if (taicpu(p).ops=3) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1,A_Bxx,[C_NE,C_EQ]) and
                     (taicpu(hp1).ops=3) and
                     MatchOperand(taicpu(hp1).oper[0]^,taicpu(p).oper[0]^) and
                     MatchOperand(taicpu(hp1).oper[1]^,NR_X0) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     (not RegModifiedBetween(taicpu(p).oper[2]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadreg(0,taicpu(p).oper[1]^.reg);
                      taicpu(hp1).loadreg(1,taicpu(p).oper[2]^.reg);

                      if taicpu(p).opcode=A_SLTU then
                        begin
                          if taicpu(hp1).condition=C_NE then
                            taicpu(hp1).condition:=C_LTU
                          else
                            taicpu(hp1).condition:=C_GEU;
                        end
                      else
                        begin
                          if taicpu(hp1).condition=C_NE then
                            taicpu(hp1).condition:=C_LT
                          else
                            taicpu(hp1).condition:=C_GE;
                        end;

                      DebugMsg('Peephole SltuB2B performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end;
                end;
              A_SLTIU:
                begin
                  {
                    Turn
                      sltiu x,y,1
                      beq/ne x,x0,...
                      dealloc x
                    Into
                      bne y,x0,...
                  }
                  if (taicpu(p).ops=3) and
                     (taicpu(p).oper[2]^.typ=top_const) and
                     (taicpu(p).oper[2]^.val=1) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     MatchInstruction(hp1,A_Bxx,[C_NE,C_EQ]) and
                     (taicpu(hp1).ops=3) and
                     MatchOperand(taicpu(hp1).oper[0]^,taicpu(p).oper[0]^) and
                     MatchOperand(taicpu(hp1).oper[1]^,NR_X0) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadreg(0,taicpu(p).oper[1]^.reg);
                      taicpu(hp1).condition:=inverse_cond(taicpu(hp1).condition);

                      DebugMsg('Peephole Sltiu0B2B performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end;
                end;
              A_SLTI:
                begin
                  {
                    Turn
                      slti x,y,0
                      beq/ne x,x0,...
                      dealloc x
                    Into
                      bge/lt y,x0,...
                  }
                  if (taicpu(p).ops=3) and
                     (taicpu(p).oper[2]^.typ=top_const) and
                     (taicpu(p).oper[2]^.val=0) and
                     GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
                     (hp1.typ=ait_instruction) and
                     (taicpu(hp1).opcode=A_Bxx) and
                     (taicpu(hp1).ops=3) and
                     (taicpu(hp1).oper[0]^.typ=top_reg) and
                     (taicpu(hp1).oper[0]^.reg=taicpu(p).oper[0]^.reg) and
                     (taicpu(hp1).oper[1]^.typ=top_reg) and
                     (taicpu(hp1).oper[1]^.reg=NR_X0) and
                     (taicpu(hp1).condition in [C_NE,C_EQ]) and
                     (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p,hp1)) and
                     RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
                    begin
                      taicpu(hp1).loadreg(0,taicpu(p).oper[1]^.reg);
                      taicpu(hp1).loadreg(1,NR_X0);

                      if taicpu(hp1).condition=C_NE then
                        taicpu(hp1).condition:=C_LT
                      else
                        taicpu(hp1).condition:=C_GE;

                      DebugMsg('Peephole Slti0B2B performed', hp1);

                      RemoveInstr(p);

                      result:=true;
                    end;
                end;
              else
                ;
            end;
          end;
        else
          ;
      end;
    end;

end.
