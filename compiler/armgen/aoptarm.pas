{
    Copyright (c) 1998-2020 by Jonas Maebe and Florian Klaempfl, members of the Free Pascal
    Development Team

    This unit implements an ARM optimizer object used commonly for ARM and AAarch64

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

Unit aoptarm;

{$i fpcdefs.inc}

{ $define DEBUG_PREREGSCHEDULER}
{ $define DEBUG_AOPTCPU}

Interface

uses
  cgbase, cgutils, cpubase, aasmtai, aasmcpu,aopt, aoptobj;

Type
  { while ARM and AAarch64 look not very similar at a first glance,
    several optimizations can be shared between both }
  TARMAsmOptimizer = class(TAsmOptimizer)
    procedure DebugMsg(const s : string; p : tai);

    function RemoveSuperfluousMove(const p: tai; movp: tai; const optimizer: string): boolean;
    function RedundantMovProcess(var p: tai; var hp1: tai): boolean;
    function GetNextInstructionUsingReg(Current: tai; out Next: tai; reg: TRegister): Boolean;

    function OptPass1UXTB(var p: tai): Boolean;
    function OptPass1UXTH(var p: tai): Boolean;
    function OptPass1SXTB(var p: tai): Boolean;
    function OptPass1SXTH(var p: tai): Boolean;
    function OptPass1And(var p: tai): Boolean; virtual;
  End;

  function MatchInstruction(const instr: tai; const op: TCommonAsmOps; const cond: TAsmConds; const postfix: TOpPostfixes): boolean;
  function MatchInstruction(const instr: tai; const op: TAsmOp; const cond: TAsmConds; const postfix: TOpPostfixes): boolean;
{$ifdef AARCH64}
  function MatchInstruction(const instr: tai; const ops : array of TAsmOp; const postfix: TOpPostfixes): boolean;
{$endif AARCH64}
  function MatchInstruction(const instr: tai; const op: TAsmOp; const postfix: TOpPostfixes): boolean;

  function RefsEqual(const r1, r2: treference): boolean;

  function MatchOperand(const oper: TOper; const reg: TRegister): boolean; inline;
  function MatchOperand(const oper1: TOper; const oper2: TOper): boolean; inline;

Implementation

  uses
    cutils,verbose,globtype,globals,
    systems,
    cpuinfo,
    cgobj,procinfo,
    aasmbase,aasmdata;


{$ifdef DEBUG_AOPTCPU}
  procedure TARMAsmOptimizer.DebugMsg(const s: string;p : tai);
    begin
      asml.insertbefore(tai_comment.Create(strpnew(s)), p);
    end;
{$else DEBUG_AOPTCPU}
  procedure TARMAsmOptimizer.DebugMsg(const s: string;p : tai);inline;
    begin
    end;
{$endif DEBUG_AOPTCPU}

  function MatchInstruction(const instr: tai; const op: TCommonAsmOps; const cond: TAsmConds; const postfix: TOpPostfixes): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        ((op = []) or ((taicpu(instr).opcode<=LastCommonAsmOp) and (taicpu(instr).opcode in op))) and
        ((cond = []) or (taicpu(instr).condition in cond)) and
        ((postfix = []) or (taicpu(instr).oppostfix in postfix));
    end;


  function MatchInstruction(const instr: tai; const op: TAsmOp; const cond: TAsmConds; const postfix: TOpPostfixes): boolean;
    begin
      result :=
        (instr.typ = ait_instruction) and
        (taicpu(instr).opcode = op) and
        ((cond = []) or (taicpu(instr).condition in cond)) and
        ((postfix = []) or (taicpu(instr).oppostfix in postfix));
    end;


{$ifdef AARCH64}
  function MatchInstruction(const instr: tai; const ops : array of TAsmOp; const postfix: TOpPostfixes): boolean;
  var
    op : TAsmOp;
  begin
    result:=false;
    if instr.typ <> ait_instruction then
      exit;
    for op in ops do
      begin
        if (taicpu(instr).opcode = op) and
           ((postfix = []) or (taicpu(instr).oppostfix in postfix)) then
          begin
            result:=true;
            exit;
          end;
      end;
    end;
{$endif AARCH64}

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


  function RefsEqual(const r1, r2: treference): boolean;
    begin
      refsequal :=
        (r1.offset = r2.offset) and
        (r1.base = r2.base) and
        (r1.index = r2.index) and (r1.scalefactor = r2.scalefactor) and
        (r1.symbol=r2.symbol) and (r1.refaddr = r2.refaddr) and
        (r1.relsymbol = r2.relsymbol) and
{$ifdef ARM}
        (r1.signindex = r2.signindex) and
{$endif ARM}
        (r1.shiftimm = r2.shiftimm) and
        (r1.addressmode = r2.addressmode) and
        (r1.shiftmode = r2.shiftmode) and
        (r1.volatility=[]) and
        (r2.volatility=[]);
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


  function TARMAsmOptimizer.GetNextInstructionUsingReg(Current: tai;
    Out Next: tai; reg: TRegister): Boolean;
    var
      gniResult: Boolean;
    begin
      Next:=Current;
      Result := False;
      repeat

        gniResult:=GetNextInstruction(Next,Next);
        if gniResult and RegInInstruction(reg,Next) then
          { Found something }
          Exit(True);

      until not gniResult or
        not(cs_opt_level3 in current_settings.optimizerswitches) or
        (Next.typ<>ait_instruction) or
        is_calljmp(taicpu(Next).opcode)
{$ifdef ARM}
        or RegModifiedByInstruction(NR_PC,Next)
{$endif ARM}
        ;
    end;


  function TARMAsmOptimizer.RemoveSuperfluousMove(const p: tai; movp: tai; const optimizer: string):boolean;
    var
      alloc,
      dealloc : tai_regalloc;
      hp1 : tai;
    begin
      Result:=false;
      if MatchInstruction(movp, A_MOV, [taicpu(p).condition], [PF_None]) and
        { We can't optimize if there is a shiftop }
        (taicpu(movp).ops=2) and
        MatchOperand(taicpu(movp).oper[1]^, taicpu(p).oper[0]^.reg) and
        { don't mess with moves to fp }
        (taicpu(movp).oper[0]^.reg<>current_procinfo.framepointer) and
        { the destination register of the mov might not be used beween p and movp }
        not(RegUsedBetween(taicpu(movp).oper[0]^.reg,p,movp)) and
{$ifdef ARM}
        { PC should be changed only by moves }
        (taicpu(movp).oper[0]^.reg<>NR_PC) and
        { cb[n]z are thumb instructions which require specific registers, with no wide forms }
        (taicpu(p).opcode<>A_CBZ) and
        (taicpu(p).opcode<>A_CBNZ) and
        { There is a special requirement for MUL and MLA, oper[0] and oper[1] are not allowed to be the same }
        not (
          (taicpu(p).opcode in [A_MLA, A_MUL]) and
          (taicpu(p).oper[1]^.reg = taicpu(movp).oper[0]^.reg) and
          (current_settings.cputype < cpu_armv6)
        ) and
{$endif ARM}
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


  function TARMAsmOptimizer.RedundantMovProcess(var p: tai; var hp1: tai):boolean;
    var
      I: Integer;
      current_hp, next_hp: tai;
      LDRChange: Boolean;
    begin
      Result:=false;
      {
        change
        mov r1, r0
        add r1, r1, #1
        to
        add r1, r0, #1

        Todo: Make it work for mov+cmp too

        CAUTION! If this one is successful p might not be a mov instruction anymore!
      }
      if (taicpu(p).ops = 2) and
         (taicpu(p).oper[1]^.typ = top_reg) and
         (taicpu(p).oppostfix = PF_NONE) then
        begin

          if
            MatchInstruction(hp1, [A_ADD, A_ADC,
{$ifdef ARM}
                                   A_RSB, A_RSC,
{$endif ARM}
                                   A_SUB, A_SBC,
                                   A_AND, A_BIC, A_EOR, A_ORR, A_MOV, A_MVN],
                             [taicpu(p).condition], []) and
            { MOV and MVN might only have 2 ops }
            (taicpu(hp1).ops >= 2) and
            MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[0]^.reg) and
            (taicpu(hp1).oper[1]^.typ = top_reg) and
            (
              (taicpu(hp1).ops = 2) or
              (taicpu(hp1).oper[2]^.typ in [top_reg, top_const, top_shifterop])
            ) and
{$ifdef AARCH64}
            (taicpu(p).oper[1]^.reg<>NR_SP) and
{$endif AARCH64}
            not(RegUsedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
            begin
              { When we get here we still don't know if the registers match }
              for I:=1 to 2 do
                {
                  If the first loop was successful p will be replaced with hp1.
                  The checks will still be ok, because all required information
                  will also be in hp1 then.
                }
                if (taicpu(hp1).ops > I) and
                   MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[I]^.reg)
{$ifdef ARM}
                   { prevent certain combinations on thumb(2), this is only a safe approximation }
                   and (not(GenerateThumbCode or GenerateThumb2Code) or
                    ((getsupreg(taicpu(p).oper[1]^.reg)<>RS_R13) and
                     (getsupreg(taicpu(p).oper[1]^.reg)<>RS_R15)))
{$endif ARM}

                then
                  begin
                    DebugMsg('Peephole RedundantMovProcess done', hp1);
                    taicpu(hp1).oper[I]^.reg := taicpu(p).oper[1]^.reg;
                    if p<>hp1 then
                      begin
                        asml.remove(p);
                        p.free;
                        p:=hp1;
                        Result:=true;
                      end;
                  end;

              if Result then Exit;
            end
          { Change:                   Change:
              mov     r1, r0            mov     r1, r0
              ...                       ...
              ldr/str r2, [r1, etc.]    mov     r2, r1
            To:                       To:
              ldr/str r2, [r0, etc.]    mov     r2, r0
          }
          else if (taicpu(p).condition = C_None) and (taicpu(p).oper[1]^.typ = top_reg)
{$ifdef ARM}
            and not (getsupreg(taicpu(p).oper[0]^.reg) in [RS_PC, RS_R14, RS_STACK_POINTER_REG])
            and (getsupreg(taicpu(p).oper[1]^.reg) <> RS_PC)
            { Thumb does not support references with base and index one being SP }
            and (not(GenerateThumbCode) or (getsupreg(taicpu(p).oper[1]^.reg) <> RS_STACK_POINTER_REG))
{$endif ARM}
{$ifdef AARCH64}
            and (getsupreg(taicpu(p).oper[0]^.reg) <> RS_STACK_POINTER_REG)
{$endif AARCH64}
            then
            begin
              current_hp := p;
              TransferUsedRegs(TmpUsedRegs);

              { Search local instruction block }
              while GetNextInstruction(current_hp, next_hp) and (next_hp <> BlockEnd) and (next_hp.typ = ait_instruction) do
                begin
                  UpdateUsedRegs(TmpUsedRegs, tai(current_hp.Next));
                  LDRChange := False;

                  if (taicpu(next_hp).opcode in [A_LDR,A_STR]) and (taicpu(next_hp).ops = 2) then
                    begin

                      { Change the registers from r1 to r0 }
                      if (taicpu(next_hp).oper[1]^.ref^.base = taicpu(p).oper[0]^.reg) and
{$ifdef ARM}
                        { This optimisation conflicts with something and raises
                          an access violation - needs further investigation. [Kit] }
                        (taicpu(next_hp).opcode <> A_LDR) and
{$endif ARM}
                        { Don't mess around with the base register if the
                          reference is pre- or post-indexed }
                        (taicpu(next_hp).oper[1]^.ref^.addressmode = AM_OFFSET) then
                        begin
                          taicpu(next_hp).oper[1]^.ref^.base := taicpu(p).oper[1]^.reg;
                          LDRChange := True;
                        end;

                      if taicpu(next_hp).oper[1]^.ref^.index = taicpu(p).oper[0]^.reg then
                        begin
                          taicpu(next_hp).oper[1]^.ref^.index := taicpu(p).oper[1]^.reg;
                          LDRChange := True;
                        end;

                      if LDRChange then
                        DebugMsg('Peephole Optimization: ' + std_regname(taicpu(p).oper[0]^.reg) + ' = ' + std_regname(taicpu(p).oper[1]^.reg) + ' (MovLdr2Ldr 1)', next_hp);

                      { Drop out if we're dealing with pre-indexed references }
                      if (taicpu(next_hp).oper[1]^.ref^.addressmode = AM_PREINDEXED) and
                        (
                          RegInRef(taicpu(p).oper[0]^.reg, taicpu(next_hp).oper[1]^.ref^) or
                          RegInRef(taicpu(p).oper[1]^.reg, taicpu(next_hp).oper[1]^.ref^)
                        ) then
                        begin
                          { Remember to update register allocations }
                          if LDRChange then
                            AllocRegBetween(taicpu(p).oper[1]^.reg, p, next_hp, UsedRegs);

                          Break;
                        end;

                      { The register being stored can be potentially changed (as long as it's not the stack pointer) }
                      if (taicpu(next_hp).opcode = A_STR) and (getsupreg(taicpu(p).oper[1]^.reg) <> RS_STACK_POINTER_REG) and
                        MatchOperand(taicpu(next_hp).oper[0]^, taicpu(p).oper[0]^.reg) then
                        begin
                          DebugMsg('Peephole Optimization: ' + std_regname(taicpu(p).oper[0]^.reg) + ' = ' + std_regname(taicpu(p).oper[1]^.reg) + ' (MovLdr2Ldr 2)', next_hp);
                          taicpu(next_hp).oper[0]^.reg := taicpu(p).oper[1]^.reg;
                          LDRChange := True;
                        end;

                      if LDRChange and (getsupreg(taicpu(p).oper[1]^.reg) <> RS_STACK_POINTER_REG) then
                        begin
                          AllocRegBetween(taicpu(p).oper[1]^.reg, p, next_hp, UsedRegs);
                          if (taicpu(p).oppostfix = PF_None) and
                            (
                              (
                                (taicpu(next_hp).opcode = A_LDR) and
                                MatchOperand(taicpu(next_hp).oper[0]^, taicpu(p).oper[0]^.reg)
                              ) or
                              not RegUsedAfterInstruction(taicpu(p).oper[0]^.reg, next_hp, TmpUsedRegs)
                            ) and
                            { Double-check to see if the old registers were actually
                              changed (e.g. if the super registers matched, but not
                              the sizes, they won't be changed). }
                            (
                              (taicpu(next_hp).opcode = A_LDR) or
                              not RegInOp(taicpu(p).oper[0]^.reg, taicpu(next_hp).oper[0]^)
                            ) and
                            not RegInRef(taicpu(p).oper[0]^.reg, taicpu(next_hp).oper[1]^.ref^) then
                            begin
                              DebugMsg('Peephole Optimization: RedundantMovProcess 2a done', p);
                              RemoveCurrentP(p);
                              Result := True;
                              Exit;
                            end;
                        end;
                    end
                  else if (taicpu(next_hp).opcode = A_MOV) and (taicpu(next_hp).oppostfix = PF_None) and
                    (taicpu(next_hp).ops = 2) then
                    begin
                      if MatchOperand(taicpu(next_hp).oper[0]^, taicpu(p).oper[0]^.reg) then
                        begin
                          { Found another mov that writes entirely to the register }
                          if RegUsedBetween(taicpu(p).oper[0]^.reg, p, next_hp) then
                            begin
                              { Register was used beforehand }
                              if MatchOperand(taicpu(next_hp).oper[1]^, taicpu(p).oper[1]^.reg) then
                                begin
                                  { This MOV is exactly the same as the first one.
                                    Since none of the registers have changed value
                                    at this point, we can remove it. }
                                  DebugMsg('Peephole Optimization: RedundantMovProcess 3a done', next_hp);

                                  if (next_hp = hp1) then
                                    { Don't let hp1 become a dangling pointer }
                                    hp1 := nil;

                                  asml.Remove(next_hp);
                                  next_hp.Free;

                                  { We still have the original p, so we can continue optimising;
                                   if it was -O2 or below, this instruction appeared immediately
                                   after the first MOV, so we're technically not looking more
                                   than one instruction ahead after it's removed! [Kit] }
                                  Continue;
                                end
                              else
                                { Register changes value - drop out }
                                Break;
                            end;

                          { We can delete the first MOV (only if the second MOV is unconditional) }
{$ifdef ARM}
                          if (taicpu(p).oppostfix = PF_None) and
                            (taicpu(next_hp).condition = C_None) then
{$endif ARM}
                            begin
                              DebugMsg('Peephole Optimization: RedundantMovProcess 2b done', p);
                              RemoveCurrentP(p);
                              Result := True;
                            end;
                          Exit;
                        end
                      else if MatchOperand(taicpu(next_hp).oper[1]^, taicpu(p).oper[0]^.reg) then
                        begin
                          if MatchOperand(taicpu(next_hp).oper[0]^, taicpu(p).oper[1]^.reg)
                            { Be careful - if the entire register is not used, removing this
                              instruction will leave the unused part uninitialised }
{$ifdef AARCH64}
                            and (getsubreg(taicpu(p).oper[1]^.reg) = R_SUBQ)
{$endif AARCH64}
                            then
                            begin
                              { Instruction will become mov r1,r1 }
                              DebugMsg('Peephole Optimization: Mov2None 2 done', next_hp);

                              if (next_hp = hp1) then
                                { Don't let hp1 become a dangling pointer }
                                hp1 := nil;

                              asml.Remove(next_hp);
                              next_hp.Free;
                              Continue;
                            end;

                          { Change the old register (checking the first operand again
                            forces it to be left alone if the full register is not
                            used, lest mov w1,w1 gets optimised out by mistake. [Kit] }
{$ifdef AARCH64}
                          if not MatchOperand(taicpu(next_hp).oper[0]^, taicpu(p).oper[1]^.reg) then
{$endif AARCH64}
                            begin
                              DebugMsg('Peephole Optimization: ' + std_regname(taicpu(p).oper[0]^.reg) + ' = ' + std_regname(taicpu(p).oper[1]^.reg) + ' (MovMov2Mov 2)', next_hp);
                              taicpu(next_hp).oper[1]^.reg := taicpu(p).oper[1]^.reg;
                              AllocRegBetween(taicpu(p).oper[1]^.reg, p, next_hp, UsedRegs);

                              { If this was the only reference to the old register,
                                then we can remove the original MOV now }

                              if (taicpu(p).oppostfix = PF_None) and
                                { A bit of a hack - sometimes registers aren't tracked properly, so do not
                                  remove if the register was apparently not allocated when its value is
                                  first set at the MOV command (this is especially true for the stack
                                  register). [Kit] }
                                (getsupreg(taicpu(p).oper[1]^.reg) <> RS_STACK_POINTER_REG) and
                                RegInUsedRegs(taicpu(p).oper[0]^.reg, UsedRegs) and
                                not RegUsedAfterInstruction(taicpu(p).oper[0]^.reg, next_hp, TmpUsedRegs) then
                                begin
                                  DebugMsg('Peephole Optimization: RedundantMovProcess 2c done', p);
                                  RemoveCurrentP(p);
                                  Result := True;
                                  Exit;
                                end;
                            end;
                        end;
                    end;

                  { On low optimisation settions, don't search more than one instruction ahead }
                  if not(cs_opt_level3 in current_settings.optimizerswitches) or
                    { Stop at procedure calls and jumps }
                    is_calljmp(taicpu(next_hp).opcode) or
                    { If the read register has changed value, or the MOV
                      destination register has been used, drop out }
                    RegInInstruction(taicpu(p).oper[0]^.reg, next_hp) or
                    RegModifiedByInstruction(taicpu(p).oper[1]^.reg, next_hp) then
                    Break;

                  current_hp := next_hp;
                end;
            end;
        end;
    end;


  function TARMAsmOptimizer.OptPass1UXTB(var p : tai) : Boolean;
    var
      hp1, hp2: tai;
    begin
      Result:=false;
      {
        change
        uxtb reg2,reg1
        strb reg2,[...]
        dealloc reg2
        to
        strb reg1,[...]
      }
      if MatchInstruction(p, taicpu(p).opcode, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_STR, [C_None], [PF_B]) and
        assigned(FindRegDealloc(taicpu(p).oper[0]^.reg,tai(hp1.Next))) and
        { the reference in strb might not use reg2 }
        not(RegInRef(taicpu(p).oper[0]^.reg,taicpu(hp1).oper[1]^.ref^)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UxtbStrb2Strb done', p);
          taicpu(hp1).loadReg(0,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p,hp2);
          asml.remove(p);
          p.free;
          p:=hp2;
          result:=true;
        end
      {
        change
        uxtb reg2,reg1
        uxth reg3,reg2
        dealloc reg2
        to
        uxtb reg3,reg1
      }
      else if MatchInstruction(p, A_UXTB, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_UXTH, [C_None], [PF_None]) and
        (taicpu(hp1).ops = 2) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UxtbUxth2Uxtb done', p);
          AllocRegBetween(taicpu(hp1).oper[0]^.reg,p,hp1,UsedRegs);
          taicpu(p).loadReg(0,taicpu(hp1).oper[0]^.reg);
          asml.remove(hp1);
          hp1.free;
          result:=true;
        end
      {
        change
        uxtb reg2,reg1
        uxtb reg3,reg2
        dealloc reg2
        to
        uxtb reg3,reg1
      }
      else if MatchInstruction(p, A_UXTB, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_UXTB, [C_None], [PF_None]) and
        (taicpu(hp1).ops = 2) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UxtbUxtb2Uxtb done', p);
          AllocRegBetween(taicpu(hp1).oper[0]^.reg,p,hp1,UsedRegs);
          taicpu(p).loadReg(0,taicpu(hp1).oper[0]^.reg);
          asml.remove(hp1);
          hp1.free;
          result:=true;
        end
      {
        change
        uxtb reg2,reg1
        and reg3,reg2,#0x*FF
        dealloc reg2
        to
        uxtb reg3,reg1
      }
      else if MatchInstruction(p, A_UXTB, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_AND, [C_None], [PF_None]) and
        (taicpu(hp1).ops=3) and
        (taicpu(hp1).oper[2]^.typ=top_const) and
        ((taicpu(hp1).oper[2]^.val and $FF)=$FF) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UxtbAndImm2Uxtb done', p);
          taicpu(hp1).opcode:=A_UXTB;
          taicpu(hp1).ops:=2;
          taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p,hp2);
          asml.remove(p);
          p.free;
          p:=hp2;
          result:=true;
        end
      else if GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
        RemoveSuperfluousMove(p, hp1, 'UxtbMov2Data') then
        Result:=true;
    end;


  function TARMAsmOptimizer.OptPass1UXTH(var p : tai) : Boolean;
    var
      hp1: tai;
    begin
      Result:=false;
      {
        change
        uxth reg2,reg1
        strh reg2,[...]
        dealloc reg2
        to
        strh reg1,[...]
      }
      if MatchInstruction(p, taicpu(p).opcode, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_STR, [C_None], [PF_H]) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { the reference in strb might not use reg2 }
        not(RegInRef(taicpu(p).oper[0]^.reg,taicpu(hp1).oper[1]^.ref^)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UXTHStrh2Strh done', p);
          taicpu(hp1).loadReg(0,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p, hp1);
          asml.remove(p);
          p.free;
          p:=hp1;
          result:=true;
        end
      {
        change
        uxth reg2,reg1
        uxth reg3,reg2
        dealloc reg2
        to
        uxth reg3,reg1
      }
      else if MatchInstruction(p, A_UXTH, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_UXTH, [C_None], [PF_None]) and
        (taicpu(hp1).ops=2) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UxthUxth2Uxth done', p);
          AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp1,UsedRegs);
          taicpu(hp1).opcode:=A_UXTH;
          taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p, hp1);
          asml.remove(p);
          p.free;
          p:=hp1;
          result:=true;
        end
      {
        change
        uxth reg2,reg1
        and reg3,reg2,#65535
        dealloc reg2
        to
        uxth reg3,reg1
      }
      else if MatchInstruction(p, A_UXTH, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_AND, [C_None], [PF_None]) and
        (taicpu(hp1).ops=3) and
        (taicpu(hp1).oper[2]^.typ=top_const) and
        ((taicpu(hp1).oper[2]^.val and $FFFF)=$FFFF) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole UxthAndImm2Uxth done', p);
          taicpu(hp1).opcode:=A_UXTH;
          taicpu(hp1).ops:=2;
          taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p, hp1);
          asml.remove(p);
          p.free;
          p:=hp1;
          result:=true;
        end
      else if GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
           RemoveSuperfluousMove(p, hp1, 'UxthMov2Data') then
        Result:=true;
    end;


  function TARMAsmOptimizer.OptPass1SXTB(var p : tai) : Boolean;
    var
      hp1, hp2: tai;
    begin
      Result:=false;
      {
        change
        sxtb reg2,reg1
        strb reg2,[...]
        dealloc reg2
        to
        strb reg1,[...]
      }
      if MatchInstruction(p, taicpu(p).opcode, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_STR, [C_None], [PF_B]) and
        assigned(FindRegDealloc(taicpu(p).oper[0]^.reg,tai(hp1.Next))) and
        { the reference in strb might not use reg2 }
        not(RegInRef(taicpu(p).oper[0]^.reg,taicpu(hp1).oper[1]^.ref^)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SxtbStrb2Strb done', p);
          taicpu(hp1).loadReg(0,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p,hp2);
          asml.remove(p);
          p.free;
          p:=hp2;
          result:=true;
        end
      {
        change
        sxtb reg2,reg1
        sxth reg3,reg2
        dealloc reg2
        to
        sxtb reg3,reg1
      }
      else if MatchInstruction(p, A_SXTB, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_SXTH, [C_None], [PF_None]) and
        (taicpu(hp1).ops = 2) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SxtbSxth2Sxtb done', p);
          AllocRegBetween(taicpu(hp1).oper[0]^.reg,p,hp1,UsedRegs);
          taicpu(p).loadReg(0,taicpu(hp1).oper[0]^.reg);
          asml.remove(hp1);
          hp1.free;
          result:=true;
        end
      {
        change
        sxtb reg2,reg1
        sxtb reg3,reg2
        dealloc reg2
        to
        uxtb reg3,reg1
      }
      else if MatchInstruction(p, A_SXTB, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_SXTB, [C_None], [PF_None]) and
        (taicpu(hp1).ops = 2) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SxtbSxtb2Sxtb done', p);
          AllocRegBetween(taicpu(hp1).oper[0]^.reg,p,hp1,UsedRegs);
          taicpu(p).loadReg(0,taicpu(hp1).oper[0]^.reg);
          asml.remove(hp1);
          hp1.free;
          result:=true;
        end
      {
        change
        sxtb reg2,reg1
        and reg3,reg2,#0x*FF
        dealloc reg2
        to
        uxtb reg3,reg1
      }
      else if MatchInstruction(p, A_SXTB, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_AND, [C_None], [PF_None]) and
        (taicpu(hp1).ops=3) and
        (taicpu(hp1).oper[2]^.typ=top_const) and
        ((taicpu(hp1).oper[2]^.val and $FF)=$FF) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SxtbAndImm2Sxtb done', p);
          taicpu(hp1).opcode:=A_SXTB;
          taicpu(hp1).ops:=2;
          taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p,hp2);
          asml.remove(p);
          p.free;
          p:=hp2;
          result:=true;
        end
      else if GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
           RemoveSuperfluousMove(p, hp1, 'SxtbMov2Data') then
        Result:=true;
    end;


  function TARMAsmOptimizer.OptPass1SXTH(var p : tai) : Boolean;
    var
      hp1: tai;
    begin
      Result:=false;
      {
        change
        sxth reg2,reg1
        strh reg2,[...]
        dealloc reg2
        to
        strh reg1,[...]
      }
      if MatchInstruction(p, taicpu(p).opcode, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_STR, [C_None], [PF_H]) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { the reference in strb might not use reg2 }
        not(RegInRef(taicpu(p).oper[0]^.reg,taicpu(hp1).oper[1]^.ref^)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SXTHStrh2Strh done', p);
          taicpu(hp1).loadReg(0,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p, hp1);
          asml.remove(p);
          p.free;
          p:=hp1;
          result:=true;
        end
      {
        change
        sxth reg2,reg1
        sxth reg3,reg2
        dealloc reg2
        to
        sxth reg3,reg1
      }
      else if MatchInstruction(p, A_SXTH, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_SXTH, [C_None], [PF_None]) and
        (taicpu(hp1).ops=2) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SxthSxth2Sxth done', p);
          AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp1,UsedRegs);
          taicpu(hp1).opcode:=A_SXTH;
          taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p, hp1);
          asml.remove(p);
          p.free;
          p:=hp1;
          result:=true;
        end
      {
        change
        sxth reg2,reg1
        and reg3,reg2,#65535
        dealloc reg2
        to
        sxth reg3,reg1
      }
      else if MatchInstruction(p, A_SXTH, [C_None], [PF_None]) and
        (taicpu(p).ops=2) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1, A_AND, [C_None], [PF_None]) and
        (taicpu(hp1).ops=3) and
        (taicpu(hp1).oper[2]^.typ=top_const) and
        ((taicpu(hp1).oper[2]^.val and $FFFF)=$FFFF) and
        MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
        RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole SxthAndImm2Sxth done', p);
          taicpu(hp1).opcode:=A_SXTH;
          taicpu(hp1).ops:=2;
          taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
          GetNextInstruction(p, hp1);
          asml.remove(p);
          p.free;
          p:=hp1;
          result:=true;
        end
      else if GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
           RemoveSuperfluousMove(p, hp1, 'SxthMov2Data') then
        Result:=true;
    end;


  function TARMAsmOptimizer.OptPass1And(var p : tai) : Boolean;
    var
      hp1, hp2: tai;
      i: longint;
    begin
      Result:=false;
      {
        optimize
        and reg2,reg1,const1
        ...
      }
      if (taicpu(p).ops>2) and
         (taicpu(p).oper[1]^.typ = top_reg) and
         (taicpu(p).oper[2]^.typ = top_const) then
        begin
          {
            change
            and reg2,reg1,const1
            ...
            and reg3,reg2,const2
            to
            and reg3,reg1,(const1 and const2)
          }
          if GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
          MatchInstruction(hp1, A_AND, [taicpu(p).condition], [PF_None]) and
          RegEndOfLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
          MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
          (taicpu(hp1).oper[2]^.typ = top_const)
{$ifdef AARCH64}
          and ((((getsubreg(taicpu(p).oper[0]^.reg)=R_SUBQ) and is_shifter_const(taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val,OS_64)) or
               ((getsubreg(taicpu(p).oper[0]^.reg)=R_SUBL) and is_shifter_const(taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val,OS_32))
          ) or
          ((taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val)=0))
{$endif AARCH64}
          then
            begin
              if not(RegUsedBetween(taicpu(hp1).oper[0]^.reg,p,hp1)) then
                begin
                  DebugMsg('Peephole AndAnd2And done', p);
                  AllocRegBetween(taicpu(hp1).oper[0]^.reg,p,hp1,UsedRegs);
                  if (taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val)=0 then
                    begin
                      DebugMsg('Peephole AndAnd2Mov0 1 done', p);
                      taicpu(p).opcode:=A_MOV;
                      taicpu(p).ops:=2;
                      taicpu(p).loadConst(1,0);
                      taicpu(p).oppostfix:=taicpu(hp1).oppostfix;
                    end
                  else
                    begin
                      DebugMsg('Peephole AndAnd2And 1 done', p);
                      taicpu(p).loadConst(2,taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val);
                      taicpu(p).oppostfix:=taicpu(hp1).oppostfix;
                      taicpu(p).loadReg(0,taicpu(hp1).oper[0]^.reg);
                    end;
                  asml.remove(hp1);
                  hp1.free;
                  Result:=true;
                  exit;
                end
              else if not(RegUsedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
                begin
                  if (taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val)=0 then
                    begin
                      DebugMsg('Peephole AndAnd2Mov0 2 done', hp1);
                      taicpu(hp1).opcode:=A_MOV;
                      taicpu(hp1).loadConst(1,0);
                      taicpu(hp1).ops:=2;
                      taicpu(hp1).oppostfix:=taicpu(p).oppostfix;
                    end
                  else
                    begin
                      DebugMsg('Peephole AndAnd2And 2 done', hp1);
                      AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp1,UsedRegs);
                      taicpu(hp1).loadConst(2,taicpu(p).oper[2]^.val and taicpu(hp1).oper[2]^.val);
                      taicpu(hp1).oppostfix:=taicpu(p).oppostfix;
                      taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
                    end;
                  GetNextInstruction(p, hp1);
                  RemoveCurrentP(p);
                  p:=hp1;
                  Result:=true;
                  exit;
                end;
            end
          {
            change
            and reg2,reg1,$xxxxxxFF
            strb reg2,[...]
            dealloc reg2
            to
            strb reg1,[...]
          }
          else if ((taicpu(p).oper[2]^.val and $FF) = $FF) and
            MatchInstruction(p, A_AND, [C_None], [PF_None]) and
            GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
            MatchInstruction(hp1, A_STR, [C_None], [PF_B]) and
            assigned(FindRegDealloc(taicpu(p).oper[0]^.reg,tai(hp1.Next))) and
            { the reference in strb might not use reg2 }
            not(RegInRef(taicpu(p).oper[0]^.reg,taicpu(hp1).oper[1]^.ref^)) and
            { reg1 might not be modified inbetween }
            not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
            begin
              DebugMsg('Peephole AndStrb2Strb done', p);
{$ifdef AARCH64}
              taicpu(hp1).loadReg(0,newreg(R_INTREGISTER,getsupreg(taicpu(p).oper[1]^.reg),R_SUBD));
{$else AARCH64}
              taicpu(hp1).loadReg(0,taicpu(p).oper[1]^.reg);
{$endif AARCH64}
              AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp1,UsedRegs);
              RemoveCurrentP(p);
              result:=true;
              exit;
            end
          {
            change
            and reg2,reg1,255
            uxtb/uxth reg3,reg2
            dealloc reg2
            to
            and reg3,reg1,x
          }
          else if ((taicpu(p).oper[2]^.val and $ffffff00)=0) and
            MatchInstruction(p, A_AND, [C_None], [PF_None]) and
            GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
            MatchInstruction(hp1, [A_UXTB,A_UXTH], [C_None], [PF_None]) and
            (taicpu(hp1).ops = 2) and
            RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
            MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
            { reg1 might not be modified inbetween }
            not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
            begin
              DebugMsg('Peephole AndUxt2And done', p);
              taicpu(hp1).opcode:=A_AND;
              taicpu(hp1).ops:=3;
              taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
              taicpu(hp1).loadconst(2,taicpu(p).oper[2]^.val);
              GetNextInstruction(p,hp1);
              asml.remove(p);
              p.Free;
              p:=hp1;
              result:=true;
              exit;
            end
          else if ((taicpu(p).oper[2]^.val and $ffffff80)=0) and
            MatchInstruction(p, A_AND, [C_None], [PF_None]) and
            GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
            MatchInstruction(hp1, [A_SXTB,A_SXTH], [C_None], [PF_None]) and
            (taicpu(hp1).ops = 2) and
            RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) and
            MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
            { reg1 might not be modified inbetween }
            not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
            begin
              DebugMsg('Peephole AndSxt2And done', p);
              taicpu(hp1).opcode:=A_AND;
              taicpu(hp1).ops:=3;
              taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);
              taicpu(hp1).loadconst(2,taicpu(p).oper[2]^.val);
              GetNextInstruction(p,hp1);
              asml.remove(p);
              p.Free;
              p:=hp1;
              result:=true;
              exit;
            end
          {
            from
            and reg1,reg0,2^n-1
            mov reg2,reg1, lsl imm1
            (mov reg3,reg2, lsr/asr imm1)
            remove either the and or the lsl/xsr sequence if possible
          }

          else if (taicpu(p).oper[2]^.val < high(int64)) and 
	    cutils.ispowerof2(taicpu(p).oper[2]^.val+1,i) and
            GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
            MatchInstruction(hp1, A_MOV, [taicpu(p).condition], [PF_None]) and
            (taicpu(hp1).ops=3) and
            MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
            (taicpu(hp1).oper[2]^.typ = top_shifterop) and
{$ifdef ARM}
            (taicpu(hp1).oper[2]^.shifterop^.rs = NR_NO) and
{$endif ARM}
            (taicpu(hp1).oper[2]^.shifterop^.shiftmode=SM_LSL) and
            RegEndOfLife(taicpu(p).oper[0]^.reg,taicpu(hp1)) then
            begin
              {
                and reg1,reg0,2^n-1
                mov reg2,reg1, lsl imm1
                mov reg3,reg2, lsr/asr imm1
                =>
                and reg1,reg0,2^n-1
                if lsr and 2^n-1>=imm1 or asr and 2^n-1>imm1
              }
              if GetNextInstructionUsingReg(hp1,hp2,taicpu(p).oper[0]^.reg) and
                MatchInstruction(hp2, A_MOV, [taicpu(p).condition], [PF_None]) and
                (taicpu(hp2).ops=3) and
                MatchOperand(taicpu(hp2).oper[1]^, taicpu(hp1).oper[0]^.reg) and
                (taicpu(hp2).oper[2]^.typ = top_shifterop) and
{$ifdef ARM}
                (taicpu(hp2).oper[2]^.shifterop^.rs = NR_NO) and
{$endif ARM}
                (taicpu(hp2).oper[2]^.shifterop^.shiftmode in [SM_ASR,SM_LSR]) and
                (taicpu(hp1).oper[2]^.shifterop^.shiftimm=taicpu(hp2).oper[2]^.shifterop^.shiftimm) and
                RegEndOfLife(taicpu(hp1).oper[0]^.reg,taicpu(hp2)) and
                ((i<32-taicpu(hp1).oper[2]^.shifterop^.shiftimm) or
                ((i=32-taicpu(hp1).oper[2]^.shifterop^.shiftimm) and
                 (taicpu(hp2).oper[2]^.shifterop^.shiftmode=SM_LSR))) then
                begin
                  DebugMsg('Peephole AndLslXsr2And done', p);
                  taicpu(p).oper[0]^.reg:=taicpu(hp2).oper[0]^.reg;
                  asml.Remove(hp1);
                  asml.Remove(hp2);
                  hp1.free;
                  hp2.free;
                  result:=true;
                  exit;
                end
              {
                and reg1,reg0,2^n-1
                mov reg2,reg1, lsl imm1
                =>
                mov reg2,reg0, lsl imm1
                if imm1>i
              }
              else if (i>32-taicpu(hp1).oper[2]^.shifterop^.shiftimm) and
                      not(RegModifiedBetween(taicpu(p).oper[1]^.reg, p, hp1)) then
                begin
                  DebugMsg('Peephole AndLsl2Lsl done', p);
                  taicpu(hp1).oper[1]^.reg:=taicpu(p).oper[1]^.reg;
                  GetNextInstruction(p, hp1);
                  asml.Remove(p);
                  p.free;
                  p:=hp1;
                  result:=true;
                  exit;
                end
            end;
        end;
      {
        change
        and reg1, ...
        mov reg2, reg1
        to
        and reg2, ...
      }
      if GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
         (taicpu(p).ops>=3) and
         RemoveSuperfluousMove(p, hp1, 'DataMov2Data') then
        Result:=true;
    end;

end.

