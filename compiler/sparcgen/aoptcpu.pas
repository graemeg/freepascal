{
    Copyright (c) 1998-2004 by Jonas Maebe

    This unit calls the optimization procedures to optimize the assembler
    code for sparc

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

unit aoptcpu;

{$i fpcdefs.inc}

{ $define DEBUG_AOPTCPU}

  Interface

    uses
      cgbase, cpubase, aoptobj, aoptcpub, aopt, aasmtai;

    Type
      TCpuAsmOptimizer = class(TAsmOptimizer)
        function GetNextInstructionUsingReg(Current: tai;
          var Next: tai; reg: TRegister): Boolean;
        function TryRemoveMov(var p: tai; opcode: TAsmOp): boolean;
        function PeepHoleOptPass1Cpu(var p: tai): boolean; override;
        function RegUsedAfterInstruction(reg: Tregister; p: tai;
          var AllUsedRegs: TAllUsedRegs): Boolean;
        function RegLoadedWithNewValue(reg : tregister; hp : tai) : boolean; override;
        function InstructionLoadsFromReg(const reg : TRegister; const hp : tai) : boolean; override;
        procedure DebugMsg(const s : string;p : tai);
      End;

  Implementation

  uses
    globtype,globals,aasmcpu;

  function MatchInstruction(const instr: tai; const op: TAsmOp): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        (taicpu(instr).opcode = op);
    end;


  function MatchOperand(const oper: TOper; reg: TRegister): boolean;
    begin
      result:=(oper.typ=top_reg) and (oper.reg=reg);
    end;


  function IsSameReg(this,next: taicpu): boolean;
    begin
      result:=(next.ops=3) and
        (next.oper[2]^.typ=top_reg) and
        (next.oper[0]^.typ=top_reg) and
        (next.oper[2]^.reg=next.oper[0]^.reg) and
        (next.oper[2]^.reg=this.oper[2]^.reg);
    end;


  function TCpuAsmOptimizer.InstructionLoadsFromReg(const reg: TRegister; const hp: tai): boolean;
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
              result:=(p.oper[I]^.reg=reg) and (I<2);
            top_ref:
              result:=
                (p.oper[I]^.ref^.base=reg) or
                (p.oper[I]^.ref^.index=reg);
            else
              ;
          end;
          if result then exit; {Bailout if we found something}
          Inc(I);
        end;
    end;


  function TCpuAsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
    var
      p: taicpu;
    begin
      result:=false;
      if not ((assigned(hp)) and (hp.typ=ait_instruction)) then
        exit;

      p:=taicpu(hp);
      case p.opcode of
        { These instructions do not write into a register at all }
        A_NOP,
        A_FCMPs,A_FCMPd,A_FCMPq,A_CMP,
        A_BA,A_Bxx,A_FBA,A_FBxx,
        A_STB,A_STH,A_ST,A_STF,A_STDF,
        A_STX:
          exit;
        else
          ;
      end;

      result:=(p.ops>0) and (p.oper[p.ops-1]^.typ=top_reg) and
        (p.oper[p.ops-1]^.reg=reg);
    end;


  function TCpuAsmOptimizer.GetNextInstructionUsingReg(Current: tai;
    var Next: tai; reg: TRegister): Boolean;
    begin
      Next:=Current;
      repeat
        Result:=GetNextInstruction(Next,Next);
      until {not(cs_opt_level3 in current_settings.optimizerswitches) or} not(Result) or (Next.typ<>ait_instruction) or (RegInInstruction(reg,Next)) or
        (is_calljmp(taicpu(Next).opcode));
      if result and (next.typ=ait_instruction) and is_calljmp(taicpu(next).opcode) then
        begin
          result:=false;
          next:=nil;
        end;
    end;


  function TCpuAsmOptimizer.RegUsedAfterInstruction(reg: Tregister; p: tai;
    var AllUsedRegs: TAllUsedRegs): Boolean;
    begin
      AllUsedRegs[getregtype(reg)].Update(tai(p.Next),true);
      RegUsedAfterInstruction :=
        AllUsedRegs[getregtype(reg)].IsUsed(reg) and
        not(regLoadedWithNewValue(reg,p)) and
        (
          not(GetNextInstruction(p,p)) or
          instructionLoadsFromReg(reg,p) or
          not(regLoadedWithNewValue(reg,p))
        );
    end;


{$ifdef DEBUG_AOPTCPU}
  procedure TCpuAsmOptimizer.DebugMsg(const s: string;p : tai);
    begin
      asml.insertbefore(tai_comment.Create(strpnew(s)), p);
    end;
{$else DEBUG_AOPTCPU}
  procedure TCpuAsmOptimizer.DebugMsg(const s: string;p : tai);inline;
    begin
    end;
{$endif DEBUG_AOPTCPU}


  function TCpuAsmOptimizer.TryRemoveMov(var p: tai; opcode: TAsmOp): boolean;
    var
      next,hp1: tai;
      alloc,dealloc: tai_regalloc;
    begin
      { Fold
          op      ...,%reg1
          ...
          opcode  %reg1,%reg2
          dealloc %reg1
        into
          op   ...,%reg2
        opcode may be A_MOV, A_FMOVs, A_FMOVd, etc.
      }
      result:=false;
      if (taicpu(p).ops=3) and
         { don't mess with instructions using %g0 for destination }
         (taicpu(p).oper[2]^.reg<>NR_G0) and
         GetNextInstructionUsingReg(p,next,taicpu(p).oper[2]^.reg) and
         MatchInstruction(next,opcode) and
         MatchOperand(taicpu(next).oper[0]^,taicpu(p).oper[2]^.reg) and
         { the destination register of mov cannot be used between p and next }
         (not RegUsedBetween(taicpu(next).oper[1]^.reg,p,next)) and
         { This is necessary so 'mov  %reg1,%y' is not folded. Compiler should
           probably generate A_WRY opcode for this, not A_MOV. }
         (getregtype(taicpu(next).oper[1]^.reg)<>R_SPECIALREGISTER) then
        begin
          dealloc:=FindRegDealloc(taicpu(p).oper[2]^.reg,tai(next.Next));
          if assigned(dealloc) then
            begin
              { taicpu(p).oper[2]^.reg is not used anymore, try to find its allocation
                and remove it if possible }
              GetLastInstruction(p,hp1);

              asml.Remove(dealloc);
              alloc:=FindRegAlloc(taicpu(p).oper[2]^.reg,tai(hp1.Next));
              if assigned(alloc) then
                begin
                  asml.Remove(alloc);
                  alloc.free;
                  dealloc.free;
                end
              else
                asml.InsertAfter(dealloc,p);

              { try to move the allocation of the target register }
              GetLastInstruction(next,hp1);
              alloc:=FindRegAlloc(taicpu(next).oper[1]^.reg,tai(hp1.Next));
              if assigned(alloc) then
                begin
                  asml.Remove(alloc);
                  asml.InsertBefore(alloc,p);
                  { adjust used regs }
                  IncludeRegInUsedRegs(taicpu(next).oper[1]^.reg,UsedRegs);
                end;

              { finally get rid of the mov }
              taicpu(p).loadreg(2,taicpu(next).oper[1]^.reg);
              DebugMsg('Peephole OpMov2Op done',p);
              asml.remove(next);
              next.free;
            end;
        end;
    end;


  function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    var
      next,next2: tai;
    begin
      result:=false;
      case p.typ of
        ait_instruction:
          begin
            case taicpu(p).opcode of
              A_SLL:
                begin
                  { if this is sign/zero extension... }
                  if (taicpu(p).oper[1]^.typ=top_const) and
                    GetNextInstruction(p,next) and
                    (MatchInstruction(next,A_SRL) or MatchInstruction(next,A_SRA)) and
                    IsSameReg(taicpu(p),taicpu(next)) and
                    (taicpu(next).oper[1]^.typ=top_const) and
                    (taicpu(next).oper[1]^.val=taicpu(p).oper[1]^.val) and
                    (taicpu(next).oper[1]^.val=16) and
                    { ...followed by 16-bit store (possibly with PIC simplification, etc. in between) }
                    GetNextInstructionUsingReg(next,next2,taicpu(p).oper[2]^.reg) and
                    MatchInstruction(next2,A_STH) and
                    (taicpu(next2).oper[0]^.typ=top_reg) and
                    (taicpu(next2).oper[0]^.reg=taicpu(p).oper[2]^.reg) and
                    { the initial register may not be reused }
                    (not RegUsedBetween(taicpu(p).oper[0]^.reg,next,next2)) then
                    begin
                      TransferUsedRegs(TmpUsedRegs);
                      UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                      UpdateUsedRegs(TmpUsedRegs, tai(next.next));
                      if not RegUsedAfterInstruction(taicpu(p).oper[2]^.reg,next2,TmpUsedRegs) then
                        begin
                          taicpu(next2).loadreg(0,taicpu(p).oper[0]^.reg);
                          DebugMsg('Peephole SLLSRxSTH2STH done',next2);
                          asml.remove(p);
                          asml.remove(next);
                          p.free;
                          next.free;
                          p:=next2;
                        end;
                    end
                  else
                    TryRemoveMov(p,A_MOV);
                end;

{$ifdef SPARC64}
              A_SLLX:
                begin
                  { if this is sign/zero extension... }
                  if (taicpu(p).oper[1]^.typ=top_const) and
                    GetNextInstruction(p,next) and
                    (MatchInstruction(next,A_SRLX) or MatchInstruction(next,A_SRAX)) and
                    IsSameReg(taicpu(p),taicpu(next)) and
                    (taicpu(next).oper[1]^.typ=top_const) and
                    (taicpu(next).oper[1]^.val=taicpu(p).oper[1]^.val) and
                    (taicpu(next).oper[1]^.val=32) and
                    { ...followed by 32-bit store (possibly with PIC simplification, etc. in between) }
                    GetNextInstructionUsingReg(next,next2,taicpu(p).oper[2]^.reg) and
                    MatchInstruction(next2,A_ST) and
                    (taicpu(next2).oper[0]^.typ=top_reg) and
                    (taicpu(next2).oper[0]^.reg=taicpu(p).oper[2]^.reg) and
                    { the initial register may not be reused }
                    (not RegUsedBetween(taicpu(p).oper[0]^.reg,next,next2)) then
                    begin
                      TransferUsedRegs(TmpUsedRegs);
                      UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                      UpdateUsedRegs(TmpUsedRegs, tai(next.next));
                      if not RegUsedAfterInstruction(taicpu(p).oper[2]^.reg,next2,TmpUsedRegs) then
                        begin
                          taicpu(next2).loadreg(0,taicpu(p).oper[0]^.reg);
                          DebugMsg('Peephole SLLXSRxXST2ST done',next2);
                          asml.remove(p);
                          asml.remove(next);
                          p.free;
                          next.free;
                          p:=next2;
                        end;
                    end
                  else
                    TryRemoveMov(p,A_MOV);
                end;
{$endif SPARC64}

              A_SRL:
                begin
                  { happens with a_load_const_ref(...,0), where %g0 is used instead of 0 }
                  { TODO: override a_load_reg_ref_unaligned and don't generate such shifts }
                  if (taicpu(p).oper[2]^.typ=top_reg) and
                    (taicpu(p).oper[2]^.reg=NR_G0) then
                    begin
                      next:=tai(p.next);
                      asml.remove(p);
                      p.free;
                      p:=next;
                    end
                  { kill zero extension after right shift (e.g. happens with "high(dword)")}
                  else if (taicpu(p).oper[1]^.typ=top_const) and
                    (taicpu(p).oper[1]^.val>=16) and
                    GetNextInstruction(p,next) and
                    MatchInstruction(next,A_SLL) and
                    GetNextInstruction(next,next2) and
                    MatchInstruction(next2,A_SRL) and
                    IsSameReg(taicpu(p),taicpu(next)) and
                    IsSameReg(taicpu(p),taicpu(next2)) and
                    (taicpu(next).oper[1]^.typ=top_const) and
                    (taicpu(next2).oper[1]^.typ=top_const) and
                    (taicpu(next).oper[1]^.val=taicpu(next2).oper[1]^.val) and
                    (taicpu(next).oper[1]^.val=16) then
                    begin
                      asml.remove(next);
                      asml.remove(next2);
                      next.free;
                      next2.free;
                    end
                  else
                    TryRemoveMov(p,A_MOV);
                end;

              A_AND:
                begin
                  { Remove sign extension after 'and' if bit 7 of const operand is clear }
                  if (taicpu(p).oper[1]^.typ=top_const) and
                    GetNextInstruction(p,next) and
                    MatchInstruction(next,A_SLL) and
                    GetNextInstruction(next,next2) and
                    MatchInstruction(next2,A_SRA) and
                    IsSameReg(taicpu(p),taicpu(next)) and
                    IsSameReg(taicpu(p),taicpu(next2)) and
                    (taicpu(next).oper[1]^.typ=top_const) and
                    (taicpu(next2).oper[1]^.typ=top_const) and
                    (taicpu(next).oper[1]^.val=taicpu(next2).oper[1]^.val) and
                    ({(
                      (taicpu(p).oper[2]^.val<=$7fff) and
                      (taicpu(next).oper[2]^.val=16)
                    ) or }(
                      (taicpu(p).oper[1]^.val<=$7f) and
                      (taicpu(next).oper[1]^.val=24)
                    )) then
                    begin
                      asml.remove(next);
                      asml.remove(next2);
                      next.free;
                      next2.free;
                    end
                  else if (taicpu(p).oper[1]^.typ=top_const) and
                    (taicpu(p).oper[1]^.val=255) and
                    GetNextInstruction(p,next) and
                    MatchInstruction(next,A_STB) and
                    (taicpu(next).oper[0]^.typ=top_reg) and
                    (taicpu(next).oper[0]^.reg=taicpu(p).oper[2]^.reg) then
                    begin
                      TransferUsedRegs(TmpUsedRegs);
                      UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                      if not RegUsedAfterInstruction(taicpu(p).oper[2]^.reg,next,TmpUsedRegs) then
                        begin
                          taicpu(next).loadreg(0,taicpu(p).oper[0]^.reg);
                          asml.remove(p);
                          p.free;
                          p:=next;
                        end;
                    end
                  else
                    TryRemoveMov(p,A_MOV);
                end;

              A_ADD,A_ADDcc,A_ADDX,
              A_SUB,A_SUBcc,A_SUBX,
              A_SRA,A_SRAX,A_MULX,
              A_ANDcc,A_OR,A_ORcc,A_XOR,A_XORcc:
                TryRemoveMov(p,A_MOV);

              A_FADDs, A_FSUBs, A_FMULs, A_FDIVs,
              A_FABSs, A_FNEGs, A_FSQRTs,
              A_FDTOs, A_FITOs, A_FQTOs:
                TryRemoveMov(p,A_FMOVs);

              A_FADDd, A_FSUBd, A_FMULd, A_FDIVd,
              A_FABSd, A_FNEGd, A_FSQRTd,
              A_FSTOd, A_FITOd, A_FQTOd:
                TryRemoveMov(p,A_FMOVd);
              else
                ;
            end;
          end;
        else
          ;
      end;
    end;

begin
  casmoptimizer:=TCpuAsmOptimizer;
end.
