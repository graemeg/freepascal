{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit implements the ARM64 optimizer object

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

Unit aoptcpu;

{$i fpcdefs.inc}

{ $define DEBUG_AOPTCPU}

Interface

    uses
      globtype, globals,
      cutils,
      cgbase, cpubase, aasmtai, aasmcpu,
      aopt, aoptcpub, aoptarm;

    Type
      TCpuAsmOptimizer = class(TARMAsmOptimizer)
        { uses the same constructor as TAopObj }
        function PeepHoleOptPass1Cpu(var p: tai): boolean; override;
        function PostPeepHoleOptsCpu(var p: tai): boolean; override;
        function RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;override;
        function InstructionLoadsFromReg(const reg: TRegister; const hp: tai): boolean;override;
        function GetNextInstructionUsingReg(Current : tai; out Next : tai; reg : TRegister) : Boolean;
        function LookForPostindexedPattern(p : taicpu) : boolean;
        procedure DebugMsg(const s : string; p : tai);
      private
        function OptPass1Shift(var p: tai): boolean;
        function OptPostCMP(var p: tai): boolean;
        function RemoveSuperfluousMove(const p: tai; movp: tai; const optimizer: string): boolean;
        function OptPass1Data(var p: tai): boolean;
      End;

Implementation

  uses
    aasmbase,
    aoptutils,
    cgutils,
    verbose;

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

  function CanBeCond(p : tai) : boolean;
    begin
      result:=(p.typ=ait_instruction) and (taicpu(p).condition=C_None);
    end;


  function RefsEqual(const r1, r2: treference): boolean;
    begin
      refsequal :=
        (r1.offset = r2.offset) and
        (r1.base = r2.base) and
        (r1.index = r2.index) and (r1.scalefactor = r2.scalefactor) and
        (r1.symbol=r2.symbol) and (r1.refaddr = r2.refaddr) and
        (r1.relsymbol = r2.relsymbol) and
        (r1.shiftimm = r2.shiftimm) and
        (r1.addressmode = r2.addressmode) and
        (r1.shiftmode = r2.shiftmode) and
        (r1.volatility=[]) and
        (r2.volatility=[]);
    end;


  function MatchInstruction(const instr: tai; const op: TAsmOps; const postfix: TOpPostfixes): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        ((op = []) or (taicpu(instr).opcode in op)) and
        ((postfix = []) or (taicpu(instr).oppostfix in postfix));
    end;


  function MatchInstruction(const instr: tai; const op: TAsmOp; const postfix: TOpPostfixes): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        (taicpu(instr).opcode = op) and
        ((postfix = []) or (taicpu(instr).oppostfix in postfix));
    end;


  function MatchOperand(const oper: TOper; const reg: TRegister): boolean; inline;
    begin
      result := (oper.typ = top_reg) and (oper.reg = reg);
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
          top_conditioncode:
            Result:=oper1.cc = oper2.cc;
          top_realconst:
            Result:=oper1.val_real = oper2.val_real;
          top_ref:
            Result:=RefsEqual(oper1.ref^, oper2.ref^);
          else Result:=false;
        end
    end;


  function TCpuAsmOptimizer.GetNextInstructionUsingReg(Current: tai;
    Out Next: tai; reg: TRegister): Boolean;
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


  function TCpuAsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
    var
      p: taicpu;
    begin
      p := taicpu(hp);
      Result := false;
      if not ((assigned(hp)) and (hp.typ = ait_instruction)) then
        exit;

      case p.opcode of
        { These operands do not write into a register at all }
        A_CMP, A_CMN, A_TST, A_B, A_BL, A_MSR, A_FCMP:
          exit;
        {Take care of post/preincremented store and loads, they will change their base register}
        A_STR, A_LDR:
          begin
            Result := false;
            { actually, this does not apply here because post-/preindexed does not mean that a register
              is loaded with a new value, it is only modified
              (taicpu(p).oper[1]^.typ=top_ref) and
              (taicpu(p).oper[1]^.ref^.addressmode in [AM_PREINDEXED,AM_POSTINDEXED]) and
              (taicpu(p).oper[1]^.ref^.base = reg);
            }
            { STR does not load into it's first register }
            if p.opcode = A_STR then
              exit;
          end;
        else
          ;
      end;

      if Result then
        exit;

      case p.oper[0]^.typ of
        top_reg:
          Result := (p.oper[0]^.reg = reg);
        top_ref:
          Result :=
            (taicpu(p).oper[0]^.ref^.addressmode in [AM_PREINDEXED,AM_POSTINDEXED]) and
            (taicpu(p).oper[0]^.ref^.base = reg);
        else
          ;
      end;
    end;


  function TCpuAsmOptimizer.InstructionLoadsFromReg(const reg: TRegister; const hp: tai): boolean;
    var
      p: taicpu;
      i: longint;
    begin
      instructionLoadsFromReg := false;
      if not (assigned(hp) and (hp.typ = ait_instruction)) then
        exit;
      p:=taicpu(hp);

      i:=1;

      { Start on oper[0]? }
      if taicpu(hp).spilling_get_operation_type(0) in [operand_read, operand_readwrite] then
        i:=0;

      while(i<p.ops) do
        begin
          case p.oper[I]^.typ of
            top_reg:
              Result := (p.oper[I]^.reg = reg);
            top_ref:
              Result :=
                (p.oper[I]^.ref^.base = reg) or
                (p.oper[I]^.ref^.index = reg);
            else
              ;
          end;
          { Bailout if we found something }
          if Result then
            exit;
          Inc(I);
        end;
    end;


  function TCpuAsmOptimizer.RemoveSuperfluousMove(const p: tai; movp: tai; const optimizer: string):boolean;
    var
      alloc,
      dealloc : tai_regalloc;
      hp1 : tai;
    begin
      Result:=false;
      if MatchInstruction(movp, A_MOV, [PF_None]) and
        (taicpu(p).ops>=3) and
        { We can't optimize if there is a shiftop }
        (taicpu(movp).ops=2) and
        MatchOperand(taicpu(movp).oper[1]^, taicpu(p).oper[0]^.reg) and
        { don't mess with moves to fp }
        (taicpu(movp).oper[0]^.reg<>NR_FP) and
        { the destination register of the mov might not be used beween p and movp }
        not(RegUsedBetween(taicpu(movp).oper[0]^.reg,p,movp)) and
        { Take care to only do this for instructions which REALLY load to the first register.
          Otherwise
            str reg0, [reg1]
            mov reg2, reg0
          will be optimized to
            str reg2, [reg1]
        }
        RegLoadedWithNewValue(taicpu(p).oper[0]^.reg, p) then
        begin
          dealloc:=FindRegDeAlloc(taicpu(p).oper[0]^.reg,tai(movp.Next));
          if assigned(dealloc) then
            begin
              DebugMsg('Peephole '+optimizer+' removed superfluous mov', movp);
              result:=true;

              { taicpu(p).oper[0]^.reg is not used anymore, try to find its allocation
                and remove it if possible }
              asml.Remove(dealloc);
              alloc:=FindRegAllocBackward(taicpu(p).oper[0]^.reg,tai(p.previous));
              if assigned(alloc) then
                begin
                  asml.Remove(alloc);
                  alloc.free;
                  dealloc.free;
                end
              else
                asml.InsertAfter(dealloc,p);

              { try to move the allocation of the target register }
              GetLastInstruction(movp,hp1);
              alloc:=FindRegAlloc(taicpu(movp).oper[0]^.reg,tai(hp1.Next));
              if assigned(alloc) then
                begin
                  asml.Remove(alloc);
                  asml.InsertBefore(alloc,p);
                  { adjust used regs }
                  IncludeRegInUsedRegs(taicpu(movp).oper[0]^.reg,UsedRegs);
                end;

              { finally get rid of the mov }
              taicpu(p).loadreg(0,taicpu(movp).oper[0]^.reg);
              { Remove preindexing and postindexing for LDR in some cases.
                For example:
                  ldr	reg2,[reg1, xxx]!
                  mov reg1,reg2
                must be translated to:
                  ldr	reg1,[reg1, xxx]

                Preindexing must be removed there, since the same register is used as the base and as the target.
                Such case is not allowed for ARM CPU and produces crash. }
              if (taicpu(p).opcode = A_LDR) and (taicpu(p).oper[1]^.typ = top_ref)
                and (taicpu(movp).oper[0]^.reg = taicpu(p).oper[1]^.ref^.base)
              then
                taicpu(p).oper[1]^.ref^.addressmode:=AM_OFFSET;
              asml.remove(movp);
              movp.free;
            end;
        end;
    end;


  {
    optimize
      ldr/str regX,[reg1]
      ...
      add/sub reg1,reg1,regY/const

      into

      ldr/str regX,[reg1], regY/const
  }
  function TCpuAsmOptimizer.LookForPostindexedPattern(p: taicpu) : boolean;
    var
      hp1 : tai;
    begin
      Result:=false;
      if (p.oper[1]^.typ = top_ref) and
        (p.oper[1]^.ref^.addressmode=AM_OFFSET) and
        (p.oper[1]^.ref^.index=NR_NO) and
        (p.oper[1]^.ref^.offset=0) and
        GetNextInstructionUsingReg(p, hp1, p.oper[1]^.ref^.base) and
        { we cannot check NR_DEFAULTFLAGS for modification yet so don't allow a condition }
        MatchInstruction(hp1, [A_ADD, A_SUB], [PF_None]) and
        (taicpu(hp1).oper[0]^.reg=p.oper[1]^.ref^.base) and
        (taicpu(hp1).oper[1]^.reg=p.oper[1]^.ref^.base) and
        (
         { valid offset? }
         (taicpu(hp1).oper[2]^.typ=top_const) and
         (taicpu(hp1).oper[2]^.val>=-256) and
         (abs(taicpu(hp1).oper[2]^.val)<256)
        ) and
        { don't apply the optimization if the base register is loaded }
        (getsupreg(p.oper[0]^.reg)<>getsupreg(p.oper[1]^.ref^.base)) and
        not(RegModifiedBetween(taicpu(hp1).oper[0]^.reg,p,hp1)) and
        not(RegModifiedBetween(taicpu(hp1).oper[2]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole Str/LdrAdd/Sub2Str/Ldr Postindex done', p);
          p.oper[1]^.ref^.addressmode:=AM_POSTINDEXED;
          if taicpu(hp1).opcode=A_ADD then
            p.oper[1]^.ref^.offset:=taicpu(hp1).oper[2]^.val
          else
            p.oper[1]^.ref^.offset:=-taicpu(hp1).oper[2]^.val;
          asml.Remove(hp1);
          hp1.Free;
          Result:=true;
        end;
    end;


  function TCpuAsmOptimizer.OptPass1Shift(var p : tai): boolean;
    var
      hp1,hp2: tai;
      I2, I: Integer;
      shifterop: tshifterop;
    begin
      Result:=false;
      { This folds shifterops into following instructions
        <shiftop> r0, r1, #imm
        <op> r2, r3, r0

        to

        <op> r2, r3, r1, <shiftop> #imm
      }
      { do not handle ROR yet, only part of the instructions below support ROR as shifter operand }
      if MatchInstruction(p,[A_LSL, A_LSR, A_ASR{, A_ROR}],[PF_None]) and
         MatchOpType(taicpu(p),top_reg,top_reg,top_const) and
         GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
         MatchInstruction(hp1, [A_ADD, A_AND, A_BIC, A_CMP, A_CMN,
                                A_EON, A_EOR, A_NEG, A_ORN, A_ORR,
                                A_SUB, A_TST], [PF_None]) and
         RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) and
         (taicpu(hp1).ops >= 2) and
         { Currently we can't fold into another shifterop }
         (taicpu(hp1).oper[taicpu(hp1).ops-1]^.typ = top_reg) and
         { SP does not work completely with shifted registers, as I didn't find the exact rules,
           we do not operate on SP }
         (taicpu(hp1).oper[0]^.reg<>NR_SP) and
         (taicpu(hp1).oper[1]^.reg<>NR_SP) and
         (taicpu(hp1).oper[taicpu(hp1).ops-1]^.reg<>NR_SP) and
         { reg1 might not be modified inbetween }
         not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) and
         (
           { Only ONE of the two src operands is allowed to match }
           MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[taicpu(hp1).ops-2]^) xor
           MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[taicpu(hp1).ops-1]^)
         ) and
         { for SUB, the last operand must match, there is no RSB on AArch64 }
         ((taicpu(hp1).opcode<>A_SUB) or
          MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[taicpu(hp1).ops-1]^)) then
        begin
          { for the two operand instructions, start also at the second operand as they are not always commutative
            (depends on the flags tested laster on) and thus the operands cannot swapped }
          I2:=1;
          for I:=I2 to taicpu(hp1).ops-1 do
            if MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[I]^.reg) then
              begin
                { If the parameter matched on the second op from the RIGHT
                  we have to switch the parameters, this will not happen for CMP
                  were we're only evaluating the most right parameter
                }
                shifterop_reset(shifterop);
                case taicpu(p).opcode of
                  A_LSL:
                    shifterop.shiftmode:=SM_LSL;
                  A_ROR:
                    shifterop.shiftmode:=SM_ROR;
                  A_LSR:
                    shifterop.shiftmode:=SM_LSR;
                  A_ASR:
                    shifterop.shiftmode:=SM_ASR;
                  else
                    InternalError(2019090401);
                end;
                shifterop.shiftimm:=taicpu(p).oper[2]^.val;

                if I <> taicpu(hp1).ops-1 then
                  begin
                    if taicpu(hp1).ops = 3 then
                      hp2:=taicpu.op_reg_reg_reg_shifterop(taicpu(hp1).opcode,
                           taicpu(hp1).oper[0]^.reg, taicpu(hp1).oper[2]^.reg,
                           taicpu(p).oper[1]^.reg, shifterop)
                    else
                      hp2:=taicpu.op_reg_reg_shifterop(taicpu(hp1).opcode,
                           taicpu(hp1).oper[0]^.reg, taicpu(p).oper[1]^.reg,
                           shifterop);
                  end
                else
                  if taicpu(hp1).ops = 3 then
                    hp2:=taicpu.op_reg_reg_reg_shifterop(taicpu(hp1).opcode,
                         taicpu(hp1).oper[0]^.reg, taicpu(hp1).oper[1]^.reg,
                         taicpu(p).oper[1]^.reg,shifterop)
                  else
                    hp2:=taicpu.op_reg_reg_shifterop(taicpu(hp1).opcode,
                         taicpu(hp1).oper[0]^.reg, taicpu(p).oper[1]^.reg,
                         shifterop);

                taicpu(hp2).fileinfo:=taicpu(hp1).fileinfo;
                asml.insertbefore(hp2, hp1);
                GetNextInstruction(p, hp2);
                asml.remove(p);
                asml.remove(hp1);
                p.free;
                hp1.free;
                p:=hp2;
                DebugMsg('Peephole FoldShiftProcess done', p);
                Result:=true;
                break;
              end;
        end
      else if MatchInstruction(p,[A_LSL, A_LSR, A_ASR,A_ROR],[PF_None]) and
        GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
        RemoveSuperfluousMove(p, hp1, 'ShiftMov2Shift') then
        Result:=true;
    end;


  function TCpuAsmOptimizer.OptPass1Data(var p : tai): boolean;
    var
      hp1: tai;
    begin
      result:=false;
      if GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
        RemoveSuperfluousMove(p, hp1, 'DataMov2Data') then
        Result:=true;
    end;


  function TCpuAsmOptimizer.OptPostCMP(var p : tai): boolean;
    var
     hp1,hp2: tai;
    begin
      Result:=false;
      if MatchOpType(taicpu(p),top_reg,top_const) and
        (taicpu(p).oper[1]^.val=0) and
        GetNextInstruction(p,hp1) and
        MatchInstruction(hp1,A_B,[PF_None]) and
        (taicpu(hp1).condition in [C_EQ,C_NE]) then
        begin
          case taicpu(hp1).condition of
            C_NE:
              hp2:=taicpu.op_reg_sym_ofs(A_CBNZ,taicpu(p).oper[0]^.reg,taicpu(hp1).oper[0]^.ref^.symbol,taicpu(hp1).oper[0]^.ref^.offset);
            C_EQ:
              hp2:=taicpu.op_reg_sym_ofs(A_CBZ,taicpu(p).oper[0]^.reg,taicpu(hp1).oper[0]^.ref^.symbol,taicpu(hp1).oper[0]^.ref^.offset);
            else
              Internalerror(2019090801);
          end;
          taicpu(hp2).fileinfo:=taicpu(hp1).fileinfo;
          asml.insertbefore(hp2, hp1);

          asml.remove(p);
          asml.remove(hp1);
          p.free;
          hp1.free;
          p:=hp2;
          DebugMsg('Peephole CMPB.E/NE2CBNZ/CBZ done', p);
          Result:=true;
        end;
    end;


  function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    begin
      result := false;
      if p.typ=ait_instruction then
        begin
          case taicpu(p).opcode of
            A_LDR:
              begin
                Result:=LookForPostindexedPattern(taicpu(p));
              end;
            A_STR:
              begin
                Result:=LookForPostindexedPattern(taicpu(p));
              end;
            A_LSR,
            A_ROR,
            A_ASR,
            A_LSL:
              Result:=OptPass1Shift(p);
            A_ADD,
            A_ADC,
            A_SUB,
            A_SBC,
            A_AND,
            A_BIC,
            A_EOR,
            A_ORR,
            A_MUL:
              Result:=OptPass1Data(p);
            else
              ;
          end;
        end;
    end;


  function TCpuAsmOptimizer.PostPeepHoleOptsCpu(var p: tai): boolean;
    begin
      result := false;
      if p.typ=ait_instruction then
        begin
          case taicpu(p).opcode of
            A_CMP:
              Result:=OptPostCMP(p);
            else
              ;
          end;
        end;
    end;

begin
  casmoptimizer:=TCpuAsmOptimizer;
End.

