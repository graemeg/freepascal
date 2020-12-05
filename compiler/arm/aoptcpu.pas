{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit implements the ARM optimizer object

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

{ $define DEBUG_PREREGSCHEDULER}
{ $define DEBUG_AOPTCPU}

Interface

uses
  cgbase, cgutils, cpubase, aasmtai,
  aasmcpu,
  aopt, aoptobj, aoptarm;

Type
  TCpuAsmOptimizer = class(TARMAsmOptimizer)
    { Can't be done in some cases due to the limited range of jumps }
    function CanDoJumpOpts: Boolean; override;

    { uses the same constructor as TAopObj }
    function PeepHoleOptPass1Cpu(var p: tai): boolean; override;
    procedure PeepHoleOptPass2;override;
    Function RegInInstruction(Reg: TRegister; p1: tai): Boolean;override;
    function RemoveSuperfluousVMov(const p : tai; movp : tai; const optimizer : string) : boolean;

    { gets the next tai object after current that contains info relevant
      to the optimizer in p1 which used the given register or does a
      change in program flow.
      If there is none, it returns false and
      sets p1 to nil                                                     }
    Function GetNextInstructionUsingRef(Current: tai; Out Next: tai; const ref: TReference; StopOnStore: Boolean = true): Boolean;

    { outputs a debug message into the assembler file }
    procedure DebugMsg(const s: string; p: tai);

    function InstructionLoadsFromReg(const reg : TRegister; const hp : tai) : boolean; override;

    function RegLoadedWithNewValue(reg : tregister; hp : tai) : boolean; override;
  protected
    function LookForPreindexedPattern(p: taicpu): boolean;
    function LookForPostindexedPattern(p: taicpu): boolean;


    { Individual optimisation routines }
    function OptPass1DataCheckMov(var p: tai): Boolean;
    function OptPass1ADDSUB(var p: tai): Boolean;
    function OptPass1And(var p: tai): Boolean; override; { There's optimisation code that's general for all ARM platforms }
    function OptPass1CMP(var p: tai): Boolean;
    function OptPass1LDR(var p: tai): Boolean;
    function OptPass1STM(var p: tai): Boolean;
    function OptPass1STR(var p: tai): Boolean;
    function OptPass1MOV(var p: tai): Boolean;
    function OptPass1MUL(var p: tai): Boolean;
    function OptPass1MVN(var p: tai): Boolean;
    function OptPass1VMov(var p: tai): Boolean;
    function OptPass1VOp(var p: tai): Boolean;
  End;

  TCpuPreRegallocScheduler = class(TAsmScheduler)
    function SchedulerPass1Cpu(var p: tai): boolean;override;
    procedure SwapRegLive(p, hp1: taicpu);
  end;

  TCpuThumb2AsmOptimizer = class(TCpuAsmOptimizer)
    { uses the same constructor as TAopObj }
    function PeepHoleOptPass1Cpu(var p: tai): boolean; override;
    procedure PeepHoleOptPass2;override;
    function PostPeepHoleOptsCpu(var p: tai): boolean; override;
  End;

  function MustBeLast(p : tai) : boolean;

Implementation

  uses
    cutils,verbose,globtype,globals,
    systems,
    cpuinfo,
    cgobj,procinfo,
    aasmbase,aasmdata;

{ Range check must be disabled explicitly as conversions between signed and unsigned
  32-bit values are done without explicit typecasts }
{$R-}

  function CanBeCond(p : tai) : boolean;
    begin
      result:=
        not(GenerateThumbCode) and
        (p.typ=ait_instruction) and
        (taicpu(p).condition=C_None) and
        ((taicpu(p).opcode<A_IT) or (taicpu(p).opcode>A_ITTTT)) and
        (taicpu(p).opcode<>A_CBZ) and
        (taicpu(p).opcode<>A_CBNZ) and
        (taicpu(p).opcode<>A_PLD) and
        (((taicpu(p).opcode<>A_BLX) and
          { BL may need to be converted into BLX by the linker -- could possibly
            be allowed in case it's to a local symbol of which we know that it
            uses the same instruction set as the current one }
          (taicpu(p).opcode<>A_BL)) or
         (taicpu(p).oper[0]^.typ=top_reg));
    end;


  function RemoveRedundantMove(const cmpp: tai; movp: tai; asml: TAsmList):Boolean;
    begin
      Result:=false;
      if (taicpu(movp).condition = C_EQ) and
         (taicpu(cmpp).oper[0]^.reg = taicpu(movp).oper[0]^.reg) and
         (taicpu(cmpp).oper[1]^.val = taicpu(movp).oper[1]^.val) then
      begin
        asml.insertafter(tai_comment.Create(strpnew('Peephole Optimization: CmpMovMov - Removed redundant moveq')), movp);
        asml.remove(movp);
        movp.free;
        Result:=true;
      end;
    end;


  function AlignedToQWord(const ref : treference) : boolean;
    begin
      { (safe) heuristics to ensure alignment }
      result:=(target_info.abi in [abi_eabi,abi_armeb,abi_eabihf]) and
      (((ref.offset>=0) and
        ((ref.offset mod 8)=0) and
        ((ref.base=NR_R13) or
         (ref.index=NR_R13))
        ) or
       ((ref.offset<=0) and
        { when using NR_R11, it has always a value of <qword align>+4 }
        ((abs(ref.offset+4) mod 8)=0) and
        (current_procinfo.framepointer=NR_R11) and
        ((ref.base=NR_R11) or
         (ref.index=NR_R11))
       )
      );
    end;


  function isValidConstLoadStoreOffset(const aoffset: longint; const pf: TOpPostfix) : boolean;
    begin
      if GenerateThumb2Code then
        result := (aoffset<4096) and (aoffset>-256)
      else
        result := ((pf in [PF_None,PF_B]) and
                   (abs(aoffset)<4096)) or
                  (abs(aoffset)<256);
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
      {For these instructions we have to start on oper[0]}
      if (p.opcode in [A_STR, A_LDM, A_STM, A_PLD,
                          A_CMP, A_CMN, A_TST, A_TEQ,
                          A_B, A_BL, A_BX, A_BLX,
                          A_SMLAL, A_UMLAL, A_VSTM, A_VLDM]) then i:=0;

      while(i<p.ops) do
        begin
          case p.oper[I]^.typ of
            top_reg:
              instructionLoadsFromReg := (p.oper[I]^.reg = reg) or
                { STRD }
                ((i=0) and (p.opcode=A_STR) and (p.oppostfix=PF_D) and (getsupreg(p.oper[0]^.reg)+1=getsupreg(reg)));
            top_regset:
              instructionLoadsFromReg := (getsupreg(reg) in p.oper[I]^.regset^);
            top_shifterop:
              instructionLoadsFromReg := p.oper[I]^.shifterop^.rs = reg;
            top_ref:
              instructionLoadsFromReg :=
                (p.oper[I]^.ref^.base = reg) or
                (p.oper[I]^.ref^.index = reg);
            else
              ;
          end;
          if (i=0) and (p.opcode in [A_LDM,A_VLDM]) then
            exit;

          if instructionLoadsFromReg then
            exit; {Bailout if we found something}
          Inc(I);
        end;
    end;


  function TCpuAsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
    var
      p: taicpu;
    begin
      Result := false;
      if not ((assigned(hp)) and (hp.typ = ait_instruction)) then
        exit;

      p := taicpu(hp);
      case p.opcode of
        { These operands do not write into a register at all }
        A_CMP, A_CMN, A_TST, A_TEQ, A_B, A_BL, A_BX, A_BLX, A_SWI, A_MSR, A_PLD,
        A_VCMP:
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
        A_VSTR:
          begin
            Result := false;
            exit;
          end;
        { These four are writing into the first 2 register, UMLAL and SMLAL will also read from them }
        A_UMLAL, A_UMULL, A_SMLAL, A_SMULL:
          Result :=
            (p.oper[1]^.typ = top_reg) and
            (p.oper[1]^.reg = reg);
        {Loads to oper2 from coprocessor}
        {
        MCR/MRC is currently not supported in FPC
        A_MRC:
          Result :=
            (p.oper[2]^.typ = top_reg) and
            (p.oper[2]^.reg = reg);
        }
        {Loads to all register in the registerset}
        A_LDM, A_VLDM:
          Result := (getsupreg(reg) in p.oper[1]^.regset^);
        A_POP:
          Result := (getsupreg(reg) in p.oper[0]^.regset^) or
                                   (reg=NR_STACK_POINTER_REG);
        else
          ;
      end;

      if Result then
        exit;

      case p.oper[0]^.typ of
        {This is the case}
        top_reg:
          Result := (p.oper[0]^.reg = reg) or
            { LDRD }
            (p.opcode=A_LDR) and (p.oppostfix=PF_D) and (getsupreg(p.oper[0]^.reg)+1=getsupreg(reg));
        {LDM/STM might write a new value to their index register}
        top_ref:
          Result :=
            (taicpu(p).oper[0]^.ref^.addressmode in [AM_PREINDEXED,AM_POSTINDEXED]) and
            (taicpu(p).oper[0]^.ref^.base = reg);
        else
          ;
      end;
    end;


  function TCpuAsmOptimizer.GetNextInstructionUsingRef(Current: tai;
    Out Next: tai; const ref: TReference; StopOnStore: Boolean = true): Boolean;
    begin
      Next:=Current;
      repeat
        Result:=GetNextInstruction(Next,Next);
        if Result and
           (Next.typ=ait_instruction) and
           (taicpu(Next).opcode in [A_LDR, A_STR]) and
           (
            ((taicpu(Next).ops = 2) and
             (taicpu(Next).oper[1]^.typ = top_ref) and
             RefsEqual(taicpu(Next).oper[1]^.ref^,ref)) or
            ((taicpu(Next).ops = 3) and { LDRD/STRD }
             (taicpu(Next).oper[2]^.typ = top_ref) and
             RefsEqual(taicpu(Next).oper[2]^.ref^,ref))
           ) then
            {We've found an instruction LDR or STR with the same reference}
            exit;
      until not(Result) or
            (Next.typ<>ait_instruction) or
            not(cs_opt_level3 in current_settings.optimizerswitches) or
            is_calljmp(taicpu(Next).opcode) or
            (StopOnStore and (taicpu(Next).opcode in [A_STR, A_STM])) or
            RegModifiedByInstruction(NR_PC,Next);
      Result:=false;
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


  function TCpuAsmOptimizer.CanDoJumpOpts: Boolean;
    begin
      { Cannot perform these jump optimisations if the ARM architecture has 16-bit thumb codes }
      Result := not (
        (current_settings.instructionset = is_thumb) and not (CPUARM_HAS_THUMB2 in cpu_capabilities[current_settings.cputype])
      );
    end;


  function TCpuAsmOptimizer.RemoveSuperfluousVMov(const p: tai; movp: tai; const optimizer: string):boolean;
    var
      alloc,
      dealloc : tai_regalloc;
      hp1 : tai;
    begin
      Result:=false;
      if ((MatchInstruction(movp, A_VMOV, [taicpu(p).condition], [taicpu(p).oppostfix]) and
           ((getregtype(taicpu(movp).oper[0]^.reg)=R_MMREGISTER) or (taicpu(p).opcode=A_VLDR))
          ) or
          (((taicpu(p).oppostfix in [PF_F64F32,PF_F64S16,PF_F64S32,PF_F64U16,PF_F64U32]) or (getsubreg(taicpu(p).oper[0]^.reg)=R_SUBFD)) and MatchInstruction(movp, A_VMOV, [taicpu(p).condition], [PF_F64])) or
          (((taicpu(p).oppostfix in [PF_F32F64,PF_F32S16,PF_F32S32,PF_F32U16,PF_F32U32]) or (getsubreg(taicpu(p).oper[0]^.reg)=R_SUBFS)) and MatchInstruction(movp, A_VMOV, [taicpu(p).condition], [PF_F32]))
         ) and
         (taicpu(movp).ops=2) and
         MatchOperand(taicpu(movp).oper[1]^, taicpu(p).oper[0]^.reg) and
         { the destination register of the mov might not be used beween p and movp }
         not(RegUsedBetween(taicpu(movp).oper[0]^.reg,p,movp)) and
         { Take care to only do this for instructions which REALLY load to the first register.
           Otherwise
             vstr reg0, [reg1]
             vmov reg2, reg0
           will be optimized to
             vstr reg2, [reg1]
         }
         regLoadedWithNewValue(taicpu(p).oper[0]^.reg, p) then
        begin
          dealloc:=FindRegDeAlloc(taicpu(p).oper[0]^.reg,tai(movp.Next));
          if assigned(dealloc) then
            begin
              DebugMsg('Peephole Optimization: '+optimizer+' removed superfluous vmov', movp);
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

              { change
                  vldr reg0,[reg1]
                  vmov reg2,reg0
                into
                  ldr reg2,[reg1]

                if reg2 is an int register
              }
              if (taicpu(p).opcode=A_VLDR) and (getregtype(taicpu(movp).oper[0]^.reg)=R_INTREGISTER) then
                taicpu(p).opcode:=A_LDR;

              { finally get rid of the mov }
              taicpu(p).loadreg(0,taicpu(movp).oper[0]^.reg);
              asml.remove(movp);
              movp.free;
            end;
        end;
    end;

  {
    optimize
      add/sub reg1,reg1,regY/const
      ...
      ldr/str regX,[reg1]

      into

      ldr/str regX,[reg1, regY/const]!
  }
  function TCpuAsmOptimizer.LookForPreindexedPattern(p: taicpu): boolean;
    var
      hp1: tai;
    begin
      if GenerateARMCode and
        (p.ops=3) and
        MatchOperand(p.oper[0]^, p.oper[1]^.reg) and
        GetNextInstructionUsingReg(p, hp1, p.oper[0]^.reg) and
        (not RegModifiedBetween(p.oper[0]^.reg, p, hp1)) and
        MatchInstruction(hp1, [A_LDR,A_STR], [C_None], [PF_None,PF_B,PF_H,PF_SH,PF_SB]) and
        (taicpu(hp1).oper[1]^.ref^.addressmode=AM_OFFSET) and
        (taicpu(hp1).oper[1]^.ref^.base=p.oper[0]^.reg) and
        (taicpu(hp1).oper[0]^.reg<>p.oper[0]^.reg) and
        (taicpu(hp1).oper[1]^.ref^.offset=0) and
        (taicpu(hp1).oper[1]^.ref^.index=NR_NO) and
        (((p.oper[2]^.typ=top_reg) and
          (not RegModifiedBetween(p.oper[2]^.reg, p, hp1))) or
         ((p.oper[2]^.typ=top_const) and
          ((abs(p.oper[2]^.val) < 256) or
           ((abs(p.oper[2]^.val) < 4096) and
            (taicpu(hp1).oppostfix in [PF_None,PF_B]))))) then
        begin
          taicpu(hp1).oper[1]^.ref^.addressmode:=AM_PREINDEXED;

          if p.oper[2]^.typ=top_reg then
            begin
              taicpu(hp1).oper[1]^.ref^.index:=p.oper[2]^.reg;
              if p.opcode=A_ADD then
                taicpu(hp1).oper[1]^.ref^.signindex:=1
              else
                taicpu(hp1).oper[1]^.ref^.signindex:=-1;
            end
          else
            begin
              if p.opcode=A_ADD then
                taicpu(hp1).oper[1]^.ref^.offset:=p.oper[2]^.val
              else
                taicpu(hp1).oper[1]^.ref^.offset:=-p.oper[2]^.val;
            end;

          result:=true;
        end
      else
        result:=false;
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
        MatchInstruction(hp1, [A_ADD, A_SUB], [C_None], [PF_None]) and
        (taicpu(hp1).oper[0]^.reg=p.oper[1]^.ref^.base) and
        (taicpu(hp1).oper[1]^.reg=p.oper[1]^.ref^.base) and
        (
         (taicpu(hp1).oper[2]^.typ=top_reg) or
         { valid offset? }
         ((taicpu(hp1).oper[2]^.typ=top_const) and
          ((abs(taicpu(hp1).oper[2]^.val)<256) or
           ((abs(taicpu(hp1).oper[2]^.val)<4096) and (p.oppostfix in [PF_None,PF_B]))
          )
         )
        ) and
        { don't apply the optimization if the base register is loaded }
        (p.oper[0]^.reg<>p.oper[1]^.ref^.base) and
        not(RegModifiedBetween(taicpu(hp1).oper[0]^.reg,p,hp1)) and
        { don't apply the optimization if the (new) index register is loaded }
        (p.oper[0]^.reg<>taicpu(hp1).oper[2]^.reg) and
        not(RegModifiedBetween(taicpu(hp1).oper[2]^.reg,p,hp1)) and
        GenerateARMCode then
        begin
          DebugMsg('Peephole Optimization: Str/LdrAdd/Sub2Str/Ldr Postindex done', p);
          p.oper[1]^.ref^.addressmode:=AM_POSTINDEXED;
          if taicpu(hp1).oper[2]^.typ=top_const then
            begin
              if taicpu(hp1).opcode=A_ADD then
                p.oper[1]^.ref^.offset:=taicpu(hp1).oper[2]^.val
              else
                p.oper[1]^.ref^.offset:=-taicpu(hp1).oper[2]^.val;
            end
          else
            begin
              p.oper[1]^.ref^.index:=taicpu(hp1).oper[2]^.reg;
              if taicpu(hp1).opcode=A_ADD then
                p.oper[1]^.ref^.signindex:=1
              else
                p.oper[1]^.ref^.signindex:=-1;
            end;
          asml.Remove(hp1);
          hp1.Free;
          Result:=true;
        end;
    end;


  function TCpuAsmOptimizer.OptPass1ADDSUB(var p: tai): Boolean;
    var
      hp1,hp2: tai;
      oldreg: tregister;
    begin
      Result := OptPass1DataCheckMov(p);

      {
        change
        add/sub reg2,reg1,const1
        str/ldr reg3,[reg2,const2]
        dealloc reg2
        to
        str/ldr reg3,[reg1,const2+/-const1]
      }
      if (not GenerateThumbCode) and
         (taicpu(p).ops>2) and
         (taicpu(p).oper[1]^.typ = top_reg) and
         (taicpu(p).oper[2]^.typ = top_const) then
        begin
          hp1:=p;
          while GetNextInstructionUsingReg(hp1, hp1, taicpu(p).oper[0]^.reg) and
            { we cannot check NR_DEFAULTFLAGS for modification yet so don't allow a condition }
            MatchInstruction(hp1, [A_LDR, A_STR], [C_None], []) and
            (taicpu(hp1).oper[1]^.typ = top_ref) and
            (taicpu(hp1).oper[1]^.ref^.base=taicpu(p).oper[0]^.reg) and
            { don't optimize if the register is stored/overwritten }
            (taicpu(hp1).oper[0]^.reg<>taicpu(p).oper[1]^.reg) and
            (taicpu(hp1).oper[1]^.ref^.index=NR_NO) and
            (taicpu(hp1).oper[1]^.ref^.addressmode=AM_OFFSET) and
            { new offset must be valid: either in the range of 8 or 12 bit, depend on the
              ldr postfix }
            (((taicpu(p).opcode=A_ADD) and
             isValidConstLoadStoreOffset(taicpu(hp1).oper[1]^.ref^.offset+taicpu(p).oper[2]^.val, taicpu(hp1).oppostfix)
             ) or
             ((taicpu(p).opcode=A_SUB) and
              isValidConstLoadStoreOffset(taicpu(hp1).oper[1]^.ref^.offset-taicpu(p).oper[2]^.val, taicpu(hp1).oppostfix)
             )
            ) do
            begin
              { neither reg1 nor reg2 might be changed inbetween }
              if RegModifiedBetween(taicpu(p).oper[0]^.reg,p,hp1) or
                RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1) then
                break;
              { reg2 must be either overwritten by the ldr or it is deallocated afterwards }
              if ((taicpu(hp1).opcode=A_LDR) and (taicpu(p).oper[0]^.reg=taicpu(hp1).oper[0]^.reg)) or
                assigned(FindRegDeAlloc(taicpu(p).oper[0]^.reg,tai(hp1.Next))) then
                begin
                  { remember last instruction }
                  hp2:=hp1;
                  DebugMsg('Peephole Optimization: Add/SubLdr2Ldr done', p);
                  hp1:=p;
                  { fix all ldr/str }
                  while GetNextInstructionUsingReg(hp1, hp1, taicpu(p).oper[0]^.reg) do
                    begin
                      taicpu(hp1).oper[1]^.ref^.base:=taicpu(p).oper[1]^.reg;
                      if taicpu(p).opcode=A_ADD then
                        inc(taicpu(hp1).oper[1]^.ref^.offset,taicpu(p).oper[2]^.val)
                      else
                        dec(taicpu(hp1).oper[1]^.ref^.offset,taicpu(p).oper[2]^.val);
                      if hp1=hp2 then
                        break;
                    end;
                  RemoveCurrentP(p);
                  result:=true;
                  Exit;
                end;
            end;
        end;

      if (taicpu(p).condition = C_None) and
        (taicpu(p).oppostfix = PF_None) and
        LookForPreindexedPattern(taicpu(p)) then
        begin
          DebugMsg('Peephole Optimization: Add/Sub to Preindexed done', p);
          RemoveCurrentP(p);
          Result:=true;
          Exit;
        end;
    end;


  function TCpuAsmOptimizer.OptPass1MUL(var p: tai): Boolean;
    var
      hp1,hp2: tai;
      oldreg: tregister;
    begin
      Result := OptPass1DataCheckMov(p);
      {
       Turn
       mul reg0, z,w
       sub/add x, y, reg0
       dealloc reg0

       into

       mls/mla x,z,w,y
       }
      if (taicpu(p).condition = C_None) and
        (taicpu(p).oppostfix = PF_None) and
        (taicpu(p).ops=3) and
        (taicpu(p).oper[0]^.typ = top_reg) and
        (taicpu(p).oper[1]^.typ = top_reg) and
        (taicpu(p).oper[2]^.typ = top_reg) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1,[A_ADD,A_SUB],[C_None],[PF_None]) and
        (not RegModifiedBetween(taicpu(p).oper[1]^.reg, p, hp1)) and
        (not RegModifiedBetween(taicpu(p).oper[2]^.reg, p, hp1)) and

        (((taicpu(hp1).opcode=A_ADD) and (current_settings.cputype>=cpu_armv4)) or
         ((taicpu(hp1).opcode=A_SUB) and (current_settings.cputype in [cpu_armv6t2,cpu_armv7,cpu_armv7a,cpu_armv7r,cpu_armv7m,cpu_armv7em]))) and

        // CPUs before ARMv6 don't recommend having the same Rd and Rm for MLA.
        // TODO: A workaround would be to swap Rm and Rs
        (not ((taicpu(hp1).opcode=A_ADD) and (current_settings.cputype<=cpu_armv6) and MatchOperand(taicpu(hp1).oper[0]^, taicpu(p).oper[1]^))) and

        (((taicpu(hp1).ops=3) and
          (taicpu(hp1).oper[2]^.typ=top_reg) and
          ((MatchOperand(taicpu(hp1).oper[2]^, taicpu(p).oper[0]^.reg) and
            (not RegModifiedBetween(taicpu(hp1).oper[1]^.reg, p, hp1))) or
           ((MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
             (taicpu(hp1).opcode=A_ADD) and
             (not RegModifiedBetween(taicpu(hp1).oper[2]^.reg, p, hp1)))))) or
         ((taicpu(hp1).ops=2) and
          (taicpu(hp1).oper[1]^.typ=top_reg) and
          MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg))) and
        (RegEndOfLife(taicpu(p).oper[0]^.reg,taicpu(hp1))) then
        begin
          if taicpu(hp1).opcode=A_ADD then
            begin
              taicpu(hp1).opcode:=A_MLA;

              if taicpu(hp1).ops=3 then
                begin
                  if MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^) then
                    oldreg:=taicpu(hp1).oper[2]^.reg
                  else
                    oldreg:=taicpu(hp1).oper[1]^.reg;
                end
              else
                oldreg:=taicpu(hp1).oper[0]^.reg;

              taicpu(hp1).loadreg(1,taicpu(p).oper[1]^.reg);
              taicpu(hp1).loadreg(2,taicpu(p).oper[2]^.reg);
              taicpu(hp1).loadreg(3,oldreg);

              DebugMsg('Peephole Optimization: MulAdd2MLA done', p);
            end
          else
            begin
              taicpu(hp1).opcode:=A_MLS;

              taicpu(hp1).loadreg(3,taicpu(hp1).oper[1]^.reg);

              if taicpu(hp1).ops=2 then
                taicpu(hp1).loadreg(1,taicpu(hp1).oper[0]^.reg)
              else
                taicpu(hp1).loadreg(1,taicpu(p).oper[2]^.reg);

              taicpu(hp1).loadreg(2,taicpu(p).oper[1]^.reg);

              DebugMsg('Peephole Optimization: MulSub2MLS done', p);
              AllocRegBetween(taicpu(hp1).oper[1]^.reg,p,hp1,UsedRegs);
              AllocRegBetween(taicpu(hp1).oper[2]^.reg,p,hp1,UsedRegs);
              AllocRegBetween(taicpu(hp1).oper[3]^.reg,p,hp1,UsedRegs);

            end;

          taicpu(hp1).ops:=4;
          RemoveCurrentP(p);
          Result := True;
          Exit;
        end
    end;


  function TCpuAsmOptimizer.OptPass1And(var p: tai): Boolean;
    begin
      Result := OptPass1DataCheckMov(p);
      Result := inherited OptPass1And(p) or Result;
    end;


  function TCpuAsmOptimizer.OptPass1DataCheckMov(var p: tai): Boolean;
    var
      hp1: tai;
    begin
      {
        change
        op  reg1, ...
        mov reg2, reg1
        to
        op  reg2, ...
      }
      Result := (taicpu(p).ops >= 3) and
        GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
        RemoveSuperfluousMove(p, hp1, 'DataMov2Data');
    end;


  function TCpuAsmOptimizer.OptPass1CMP(var p: tai): Boolean;
    var
      hp1, hp2, hp_last: tai;
      MovRem1, MovRem2: Boolean;
    begin
      Result := False;

      { These optimizations can be applied only to the currently enabled operations because
        the other operations do not update all flags and FPC does not track flag usage }
      if (taicpu(p).condition = C_None) and
        (taicpu(p).oper[1]^.typ = top_const) and
        GetNextInstruction(p, hp1) then
        begin
          {
            change
            cmp   reg,const1
            moveq reg,const1
            movne reg,const2
            to
            cmp   reg,const1
            movne reg,const2
          }
          if MatchInstruction(hp1, A_MOV, [C_EQ, C_NE], [PF_NONE]) and
            (taicpu(hp1).oper[1]^.typ = top_const) and
            GetNextInstruction(hp1, hp2) and
            MatchInstruction(hp2, A_MOV, [C_EQ, C_NE], [PF_NONE]) and
            (taicpu(hp2).oper[1]^.typ = top_const) then
            begin
              MovRem1 := RemoveRedundantMove(p, hp1, asml);
              MovRem2 := RemoveRedundantMove(p, hp2, asml);

              Result:= MovRem1 or MovRem2;

              { Make sure that hp1 is still the next instruction after p }
              if MovRem1 then
                if MovRem2 then
                  begin
                    if not GetNextInstruction(p, hp1) then
                      Exit;
                  end
                else
                  hp1 := hp2;
            end;

          {
            change
            <op> reg,x,y
            cmp reg,#0
            into
            <op>s reg,x,y
          }
          if (taicpu(p).oppostfix = PF_None) and
            (taicpu(p).oper[1]^.val = 0) and
            { be careful here, following instructions could use other flags
              however after a jump fpc never depends on the value of flags }
            { All above instructions set Z and N according to the following
              Z := result = 0;
              N := result[31];
              EQ = Z=1; NE = Z=0;
              MI = N=1; PL = N=0; }
            (MatchInstruction(hp1, A_B, [C_EQ,C_NE,C_MI,C_PL], []) or
            { mov is also possible, but only if there is no shifter operand, it could be an rxx,
              we are too lazy to check if it is rxx or something else }
            (MatchInstruction(hp1, A_MOV, [C_EQ,C_NE,C_MI,C_PL], []) and (taicpu(hp1).ops=2))) and
            GetLastInstruction(p, hp_last) and
            MatchInstruction(hp_last, [A_ADC,A_ADD,A_BIC,A_SUB,A_MUL,A_MVN,A_MOV,A_ORR,
              A_EOR,A_AND,A_RSB,A_RSC,A_SBC,A_MLA], [C_None], [PF_None]) and
            (
              { mlas is only allowed in arm mode }
              (taicpu(hp_last).opcode<>A_MLA) or
              (current_settings.instructionset<>is_thumb)
            ) and
            (taicpu(hp_last).oper[0]^.reg = taicpu(p).oper[0]^.reg) and
            assigned(FindRegDealloc(NR_DEFAULTFLAGS,tai(hp1.Next))) then
            begin
              DebugMsg('Peephole Optimization: OpCmp2OpS done', hp_last);

              taicpu(hp_last).oppostfix:=PF_S;

              { move flag allocation if possible }
              hp1:=FindRegAlloc(NR_DEFAULTFLAGS,tai(hp_last.Next));
              if assigned(hp1) then
                begin
                  asml.Remove(hp1);
                  asml.insertbefore(hp1, hp_last);
                end;

              RemoveCurrentP(p);
              Result:=true;
            end;
        end;
    end;


  function TCpuAsmOptimizer.OptPass1LDR(var p: tai): Boolean;
    var
      hp1: tai;
    begin
      Result := False;

      { change
        ldr reg1,ref
        ldr reg2,ref
        into ...
      }
      if (taicpu(p).oper[1]^.typ = top_ref) and
         (taicpu(p).oper[1]^.ref^.addressmode=AM_OFFSET) and
         GetNextInstruction(p,hp1) and
         { ldrd is not allowed here }
         MatchInstruction(hp1, A_LDR, [taicpu(p).condition, C_None], [taicpu(p).oppostfix,PF_None]-[PF_D]) then
        begin
          {
            ...
            ldr reg1,ref
            mov reg2,reg1
          }
          if (taicpu(p).oppostfix=taicpu(hp1).oppostfix) and
             RefsEqual(taicpu(p).oper[1]^.ref^,taicpu(hp1).oper[1]^.ref^) and
             (taicpu(p).oper[0]^.reg<>taicpu(hp1).oper[1]^.ref^.index) and
             (taicpu(p).oper[0]^.reg<>taicpu(hp1).oper[1]^.ref^.base) and
             (taicpu(hp1).oper[1]^.ref^.addressmode=AM_OFFSET) then
            begin
              if taicpu(hp1).oper[0]^.reg=taicpu(p).oper[0]^.reg then
                begin
                  DebugMsg('Peephole Optimization: LdrLdr2Ldr done', hp1);
                  asml.remove(hp1);
                  hp1.free;
                end
              else
                begin
                  DebugMsg('Peephole Optimization: LdrLdr2LdrMov done', hp1);
                  taicpu(hp1).opcode:=A_MOV;
                  taicpu(hp1).oppostfix:=PF_None;
                  taicpu(hp1).loadreg(1,taicpu(p).oper[0]^.reg);
                end;
              result := true;
            end
          {
             ...
             ldrd reg1,reg1+1,ref
          }
          else if (GenerateARMCode or GenerateThumb2Code) and
            (CPUARM_HAS_EDSP in cpu_capabilities[current_settings.cputype]) and
            { ldrd does not allow any postfixes ... }
            (taicpu(p).oppostfix=PF_None) and
            not(odd(getsupreg(taicpu(p).oper[0]^.reg))) and
            (getsupreg(taicpu(p).oper[0]^.reg)+1=getsupreg(taicpu(hp1).oper[0]^.reg)) and
            { ldr ensures that either base or index contain no register, else ldr wouldn't
              use an offset either
            }
            (taicpu(p).oper[1]^.ref^.base=taicpu(hp1).oper[1]^.ref^.base) and
            (taicpu(p).oper[1]^.ref^.index=taicpu(hp1).oper[1]^.ref^.index) and
            (taicpu(p).oper[1]^.ref^.offset+4=taicpu(hp1).oper[1]^.ref^.offset) and
            (abs(taicpu(p).oper[1]^.ref^.offset)<256) and
            AlignedToQWord(taicpu(p).oper[1]^.ref^) then
            begin
              DebugMsg('Peephole Optimization: LdrLdr2Ldrd done', p);
              taicpu(p).loadref(2,taicpu(p).oper[1]^.ref^);
              taicpu(p).loadreg(1, taicpu(hp1).oper[0]^.reg);
              taicpu(p).ops:=3;
              taicpu(p).oppostfix:=PF_D;
              asml.remove(hp1);
              hp1.free;
              result:=true;
            end;
        end;

      {
        Change

          ldrb dst1, [REF]
          and  dst2, dst1, #255

        into

          ldrb dst2, [ref]
      }
      if not(GenerateThumbCode) and
         (taicpu(p).oppostfix=PF_B) and
         GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
         MatchInstruction(hp1, A_AND, [taicpu(p).condition], [PF_NONE]) and
         (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[0]^.reg) and
         (taicpu(hp1).oper[2]^.typ = top_const) and
         (taicpu(hp1).oper[2]^.val = $FF) and
         not(RegUsedBetween(taicpu(hp1).oper[0]^.reg, p, hp1)) and
         RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hp1)) then
         begin
           DebugMsg('Peephole Optimization: LdrbAnd2Ldrb done', p);
           taicpu(p).oper[0]^.reg := taicpu(hp1).oper[0]^.reg;
           asml.remove(hp1);
           hp1.free;
           result:=true;
         end;
      Result:=LookForPostindexedPattern(taicpu(p)) or Result;
      { Remove superfluous mov after ldr
        changes
        ldr reg1, ref
        mov reg2, reg1
        to
        ldr reg2, ref

        conditions are:
          * no ldrd usage
          * reg1 must be released after mov
          * mov can not contain shifterops
          * ldr+mov have the same conditions
          * mov does not set flags
      }
      if (taicpu(p).oppostfix<>PF_D) and
         GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
         RemoveSuperfluousMove(p, hp1, 'LdrMov2Ldr') then
        Result:=true;
    end;


  function TCpuAsmOptimizer.OptPass1STM(var p: tai): Boolean;
    var
      hp1, hp2, hp3, hp4: tai;
    begin
      Result := False;

      {
        change
        stmfd	r13!,[r14]
        sub	r13,r13,#4
        bl	abc
        add	r13,r13,#4
        ldmfd	r13!,[r15]
        into
        b         abc
      }
      if not(ts_thumb_interworking in current_settings.targetswitches) and
        (taicpu(p).condition = C_None) and
        (taicpu(p).oppostfix = PF_FD) and
        (taicpu(p).oper[0]^.typ = top_ref) and
        (taicpu(p).oper[0]^.ref^.index=NR_STACK_POINTER_REG) and
        (taicpu(p).oper[0]^.ref^.base=NR_NO) and
        (taicpu(p).oper[0]^.ref^.offset=0) and
        (taicpu(p).oper[0]^.ref^.addressmode=AM_PREINDEXED) and
        (taicpu(p).oper[1]^.typ = top_regset) and
        (taicpu(p).oper[1]^.regset^ = [RS_R14]) and
        GetNextInstruction(p, hp1) and
        MatchInstruction(hp1, A_SUB, [C_None], [PF_NONE]) and
        (taicpu(hp1).oper[0]^.typ = top_reg) and
        (taicpu(hp1).oper[0]^.reg = NR_STACK_POINTER_REG) and
        MatchOperand(taicpu(hp1).oper[0]^,taicpu(hp1).oper[1]^) and
        (taicpu(hp1).oper[2]^.typ = top_const) and

        GetNextInstruction(hp1, hp2) and
        SkipEntryExitMarker(hp2, hp2) and

        MatchInstruction(hp2, [A_BL,A_BLX], [C_None], [PF_NONE]) and
        (taicpu(hp2).oper[0]^.typ = top_ref) and

        GetNextInstruction(hp2, hp3) and
        SkipEntryExitMarker(hp3, hp3) and
        MatchInstruction(hp3, A_ADD, [C_None], [PF_NONE]) and
        MatchOperand(taicpu(hp1).oper[0]^,taicpu(hp3).oper[0]^) and
        MatchOperand(taicpu(hp1).oper[0]^,taicpu(hp3).oper[1]^) and
        MatchOperand(taicpu(hp1).oper[2]^,taicpu(hp3).oper[2]^) and

        GetNextInstruction(hp3, hp4) and
        MatchInstruction(hp4, A_LDM, [C_None], [PF_FD]) and
        MatchOperand(taicpu(p).oper[0]^,taicpu(hp4).oper[0]^) and
        (taicpu(hp4).oper[1]^.typ = top_regset) and
        (taicpu(hp4).oper[1]^.regset^ = [RS_R15]) then
        begin
          asml.Remove(hp1);
          asml.Remove(hp3);
          asml.Remove(hp4);
          taicpu(hp2).opcode:=A_B;
          hp1.free;
          hp3.free;
          hp4.free;
          RemoveCurrentp(p, hp2);
          DebugMsg('Peephole Optimization: Bl2B done', p);
          Result := True;
        end;
    end;



  function TCpuAsmOptimizer.OptPass1STR(var p: tai): Boolean;
    var
      hp1: tai;
    begin
      Result := False;

      { Common conditions }
      if (taicpu(p).oper[1]^.typ = top_ref) and
        (taicpu(p).oper[1]^.ref^.addressmode=AM_OFFSET) and
        (taicpu(p).oppostfix=PF_None) then
        begin
          { change
            str reg1,ref
            ldr reg2,ref
            into
            str reg1,ref
            mov reg2,reg1
          }
          if (taicpu(p).condition=C_None) and
             GetNextInstructionUsingRef(p,hp1,taicpu(p).oper[1]^.ref^) and
             MatchInstruction(hp1, A_LDR, [taicpu(p).condition], [PF_None]) and
             (taicpu(hp1).oper[1]^.typ=top_ref) and
             (taicpu(hp1).oper[1]^.ref^.addressmode=AM_OFFSET) and
             not(RegModifiedBetween(taicpu(p).oper[0]^.reg, p, hp1)) and
             ((taicpu(hp1).oper[1]^.ref^.index=NR_NO) or not (RegModifiedBetween(taicpu(hp1).oper[1]^.ref^.index, p, hp1))) and
             ((taicpu(hp1).oper[1]^.ref^.base=NR_NO) or not (RegModifiedBetween(taicpu(hp1).oper[1]^.ref^.base, p, hp1))) then
            begin
              if taicpu(hp1).oper[0]^.reg=taicpu(p).oper[0]^.reg then
                begin
                  DebugMsg('Peephole Optimization: StrLdr2StrMov 1 done', hp1);
                  asml.remove(hp1);
                  hp1.free;
                end
              else
                begin
                  taicpu(hp1).opcode:=A_MOV;
                  taicpu(hp1).oppostfix:=PF_None;
                  taicpu(hp1).loadreg(1,taicpu(p).oper[0]^.reg);
                  DebugMsg('Peephole Optimization: StrLdr2StrMov 2 done', hp1);
                end;
              result := True;
            end
          { change
            str reg1,ref
            str reg2,ref
            into
            strd reg1,reg2,ref
          }
          else if (GenerateARMCode or GenerateThumb2Code) and
             (CPUARM_HAS_EDSP in cpu_capabilities[current_settings.cputype]) and
             not(odd(getsupreg(taicpu(p).oper[0]^.reg))) and
             (abs(taicpu(p).oper[1]^.ref^.offset)<256) and
             AlignedToQWord(taicpu(p).oper[1]^.ref^) and
             GetNextInstruction(p,hp1) and
             MatchInstruction(hp1, A_STR, [taicpu(p).condition, C_None], [PF_None]) and
            (getsupreg(taicpu(p).oper[0]^.reg)+1=getsupreg(taicpu(hp1).oper[0]^.reg)) and
            { str ensures that either base or index contain no register, else ldr wouldn't
              use an offset either
            }
            (taicpu(p).oper[1]^.ref^.base=taicpu(hp1).oper[1]^.ref^.base) and
            (taicpu(p).oper[1]^.ref^.index=taicpu(hp1).oper[1]^.ref^.index) and
            (taicpu(p).oper[1]^.ref^.offset+4=taicpu(hp1).oper[1]^.ref^.offset) then
            begin
              DebugMsg('Peephole Optimization: StrStr2Strd done', p);
              taicpu(p).oppostfix:=PF_D;
              taicpu(p).loadref(2,taicpu(p).oper[1]^.ref^);
              taicpu(p).loadreg(1, taicpu(hp1).oper[0]^.reg);
              taicpu(p).ops:=3;
              asml.remove(hp1);
              hp1.free;
              result:=true;
            end;
        end;

      Result:=LookForPostindexedPattern(taicpu(p)) or Result;
    end;


  function TCpuAsmOptimizer.OptPass1MOV(var p: tai): Boolean;
    var
      hp1, hpfar1, hp2, hp3: tai;
      i, i2: longint;
      tempop: tasmop;
      dealloc: tai_regalloc;
    begin
      Result := False;
      hp1 := nil;

      { fold
        mov reg1,reg0, shift imm1
        mov reg1,reg1, shift imm2
      }
      if (taicpu(p).ops=3) and
         (taicpu(p).oper[2]^.typ = top_shifterop) and
         (taicpu(p).oper[2]^.shifterop^.rs = NR_NO) and
         getnextinstruction(p,hp1) and
         MatchInstruction(hp1, A_MOV, [taicpu(p).condition], [PF_None]) and
         (taicpu(hp1).ops=3) and
         MatchOperand(taicpu(hp1).oper[0]^, taicpu(p).oper[0]^.reg) and
         MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) and
         (taicpu(hp1).oper[2]^.typ = top_shifterop) and
         (taicpu(hp1).oper[2]^.shifterop^.rs = NR_NO) then
        begin
          { fold
            mov reg1,reg0, lsl 16
            mov reg1,reg1, lsr 16
            strh reg1, ...
            dealloc reg1
            to
            strh reg1, ...
            dealloc reg1
          }
          if (taicpu(p).oper[2]^.shifterop^.shiftmode=SM_LSL) and
            (taicpu(p).oper[2]^.shifterop^.shiftimm=16) and
            (taicpu(hp1).oper[2]^.shifterop^.shiftmode in [SM_LSR,SM_ASR]) and
            (taicpu(hp1).oper[2]^.shifterop^.shiftimm=16) and
            getnextinstruction(hp1,hp2) and
            MatchInstruction(hp2, A_STR, [taicpu(p).condition], [PF_H]) and
            MatchOperand(taicpu(hp2).oper[0]^, taicpu(p).oper[0]^.reg) then
            begin
              TransferUsedRegs(TmpUsedRegs);
              UpdateUsedRegs(TmpUsedRegs, tai(p.next));
              UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));
              if not(RegUsedAfterInstruction(taicpu(p).oper[0]^.reg,hp2,TmpUsedRegs)) then
                begin
                  DebugMsg('Peephole Optimization: removed superfluous 16 Bit zero extension', hp1);
                  taicpu(hp2).loadreg(0,taicpu(p).oper[1]^.reg);
                  asml.remove(hp1);
                  hp1.free;

                  RemoveCurrentP(p, hp2);
                  Result:=true;
                  Exit;
                end;
            end
          { fold
            mov reg1,reg0, shift imm1
            mov reg1,reg1, shift imm2
            to
            mov reg1,reg0, shift imm1+imm2
          }
          else if (taicpu(p).oper[2]^.shifterop^.shiftmode=taicpu(hp1).oper[2]^.shifterop^.shiftmode) or
            { asr makes no use after a lsr, the asr can be foled into the lsr }
             ((taicpu(p).oper[2]^.shifterop^.shiftmode=SM_LSR) and (taicpu(hp1).oper[2]^.shifterop^.shiftmode=SM_ASR) ) then
            begin
              inc(taicpu(p).oper[2]^.shifterop^.shiftimm,taicpu(hp1).oper[2]^.shifterop^.shiftimm);
              { avoid overflows }
              if taicpu(p).oper[2]^.shifterop^.shiftimm>31 then
                case taicpu(p).oper[2]^.shifterop^.shiftmode of
                  SM_ROR:
                    taicpu(p).oper[2]^.shifterop^.shiftimm:=taicpu(p).oper[2]^.shifterop^.shiftimm and 31;
                  SM_ASR:
                    taicpu(p).oper[2]^.shifterop^.shiftimm:=31;
                  SM_LSR,
                  SM_LSL:
                    begin
                      hp2:=taicpu.op_reg_const(A_MOV,taicpu(p).oper[0]^.reg,0);
                      InsertLLItem(p.previous, p.next, hp2);
                      p.free;
                      p:=hp2;
                    end;
                  else
                    internalerror(2008072803);
                end;
              DebugMsg('Peephole Optimization: ShiftShift2Shift 1 done', p);
              asml.remove(hp1);
              hp1.free;
              hp1 := nil;
              result := true;
            end
          { fold
            mov reg1,reg0, shift imm1
            mov reg1,reg1, shift imm2
            mov reg1,reg1, shift imm3 ...
            mov reg2,reg1, shift imm3 ...
          }
          else if GetNextInstructionUsingReg(hp1,hp2, taicpu(hp1).oper[0]^.reg) and
            MatchInstruction(hp2, A_MOV, [taicpu(p).condition], [PF_None]) and
            (taicpu(hp2).ops=3) and
            MatchOperand(taicpu(hp2).oper[1]^, taicpu(hp1).oper[0]^.reg) and
            RegEndofLife(taicpu(p).oper[0]^.reg,taicpu(hp2)) and
            (taicpu(hp2).oper[2]^.typ = top_shifterop) and
            (taicpu(hp2).oper[2]^.shifterop^.rs = NR_NO) then
            begin
              { mov reg1,reg0, lsl imm1
                mov reg1,reg1, lsr/asr imm2
                mov reg2,reg1, lsl imm3 ...
                to
                mov reg1,reg0, lsl imm1
                mov reg2,reg1, lsr/asr imm2-imm3
                if
                imm1>=imm2
              }
              if (taicpu(p).oper[2]^.shifterop^.shiftmode=SM_LSL) and (taicpu(hp2).oper[2]^.shifterop^.shiftmode=SM_LSL) and
                (taicpu(hp1).oper[2]^.shifterop^.shiftmode in [SM_ASR,SM_LSR]) and
                (taicpu(p).oper[2]^.shifterop^.shiftimm>=taicpu(hp1).oper[2]^.shifterop^.shiftimm) then
                begin
                  if (taicpu(hp2).oper[2]^.shifterop^.shiftimm>=taicpu(hp1).oper[2]^.shifterop^.shiftimm) then
                    begin
                      if not(RegUsedBetween(taicpu(hp2).oper[0]^.reg,p,hp1)) and
                        not(RegUsedBetween(taicpu(hp2).oper[0]^.reg,hp1,hp2)) then
                        begin
                          DebugMsg('Peephole Optimization: ShiftShiftShift2ShiftShift 1a done', p);
                          inc(taicpu(p).oper[2]^.shifterop^.shiftimm,taicpu(hp2).oper[2]^.shifterop^.shiftimm-taicpu(hp1).oper[2]^.shifterop^.shiftimm);
                          taicpu(p).oper[0]^.reg:=taicpu(hp2).oper[0]^.reg;
                          asml.remove(hp1);
                          asml.remove(hp2);
                          hp1.free;
                          hp2.free;

                          if taicpu(p).oper[2]^.shifterop^.shiftimm>=32 then
                            begin
                              taicpu(p).freeop(1);
                              taicpu(p).freeop(2);
                              taicpu(p).loadconst(1,0);
                            end;
                          result := true;
                          Exit;
                        end;
                    end
                  else if not(RegUsedBetween(taicpu(hp2).oper[0]^.reg,hp1,hp2)) then
                    begin
                      DebugMsg('Peephole Optimization: ShiftShiftShift2ShiftShift 1b done', p);

                      dec(taicpu(hp1).oper[2]^.shifterop^.shiftimm,taicpu(hp2).oper[2]^.shifterop^.shiftimm);
                      taicpu(hp1).oper[0]^.reg:=taicpu(hp2).oper[0]^.reg;
                      asml.remove(hp2);
                      hp2.free;
                      result := true;
                      Exit;
                    end;
                end
              { mov reg1,reg0, lsr/asr imm1
                mov reg1,reg1, lsl imm2
                mov reg1,reg1, lsr/asr imm3 ...

                if imm3>=imm1 and imm2>=imm1
                to
                mov reg1,reg0, lsl imm2-imm1
                mov reg1,reg1, lsr/asr imm3 ...
              }
              else if (taicpu(p).oper[2]^.shifterop^.shiftmode in [SM_ASR,SM_LSR]) and (taicpu(hp2).oper[2]^.shifterop^.shiftmode in [SM_ASR,SM_LSR]) and
                (taicpu(hp1).oper[2]^.shifterop^.shiftmode=SM_LSL) and
                (taicpu(hp2).oper[2]^.shifterop^.shiftimm>=taicpu(p).oper[2]^.shifterop^.shiftimm) and
                (taicpu(hp1).oper[2]^.shifterop^.shiftimm>=taicpu(p).oper[2]^.shifterop^.shiftimm) then
                begin
                  dec(taicpu(hp1).oper[2]^.shifterop^.shiftimm,taicpu(p).oper[2]^.shifterop^.shiftimm);
                  taicpu(hp1).oper[1]^.reg:=taicpu(p).oper[1]^.reg;
                  DebugMsg('Peephole Optimization: ShiftShiftShift2ShiftShift 2 done', p);
                  if taicpu(hp1).oper[2]^.shifterop^.shiftimm=0 then
                    begin
                      taicpu(hp2).oper[1]^.reg:=taicpu(hp1).oper[1]^.reg;
                      asml.remove(hp1);
                      hp1.free;
                    end;

                  RemoveCurrentp(p);
                  result := true;
                  Exit;
                end;
            end;
        end;

      { All the optimisations from this point on require GetNextInstructionUsingReg
        to return True }
      while (
        GetNextInstructionUsingReg(p, hpfar1, taicpu(p).oper[0]^.reg) and
        (hpfar1.typ = ait_instruction)
      ) do
        begin

          { Change the common
            mov r0, r0, lsr #xxx
            and r0, r0, #yyy/bic r0, r0, #xxx

            and remove the superfluous and/bic if possible

            This could be extended to handle more cases.
          }

          { Change
            mov rx, ry, lsr/ror #xxx
            uxtb/uxth rz,rx/and rz,rx,0xFF
            dealloc rx

            to

            uxtb/uxth rz,ry,ror #xxx
          }
          if (GenerateThumb2Code) and
             (taicpu(p).ops=3) and
             (taicpu(p).oper[2]^.typ = top_shifterop) and
             (taicpu(p).oper[2]^.shifterop^.rs = NR_NO) and
             (taicpu(p).oper[2]^.shifterop^.shiftmode in [SM_LSR,SM_ROR]) and
             RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hpfar1)) then
             begin
               if MatchInstruction(hpfar1, A_UXTB, [C_None], [PF_None]) and
                 (taicpu(hpfar1).ops = 2) and
                 (taicpu(p).oper[2]^.shifterop^.shiftimm in [8,16,24]) and
                 MatchOperand(taicpu(hpfar1).oper[1]^, taicpu(p).oper[0]^.reg) then
                 begin
                   taicpu(hpfar1).oper[1]^.reg := taicpu(p).oper[1]^.reg;
                   taicpu(hpfar1).loadshifterop(2,taicpu(p).oper[2]^.shifterop^);
                   taicpu(hpfar1).oper[2]^.shifterop^.shiftmode:=SM_ROR;
                   taicpu(hpfar1).ops := 3;

                   if not Assigned(hp1) then
                     GetNextInstruction(p,hp1);

                   RemoveCurrentP(p, hp1);

                   result:=true;
                   exit;
                 end
               else if MatchInstruction(hpfar1, A_UXTH, [C_None], [PF_None]) and
                 (taicpu(hpfar1).ops=2) and
                 (taicpu(p).oper[2]^.shifterop^.shiftimm in [16]) and
                 MatchOperand(taicpu(hpfar1).oper[1]^, taicpu(p).oper[0]^.reg) then
                 begin
                   taicpu(hpfar1).oper[1]^.reg := taicpu(p).oper[1]^.reg;
                   taicpu(hpfar1).loadshifterop(2,taicpu(p).oper[2]^.shifterop^);
                   taicpu(hpfar1).oper[2]^.shifterop^.shiftmode:=SM_ROR;
                   taicpu(hpfar1).ops := 3;

                   if not Assigned(hp1) then
                     GetNextInstruction(p,hp1);

                   RemoveCurrentP(p, hp1);

                   result:=true;
                   exit;
                 end
               else if MatchInstruction(hpfar1, A_AND, [C_None], [PF_None]) and
                 (taicpu(hpfar1).ops = 3) and
                 (taicpu(hpfar1).oper[2]^.typ = top_const) and
                 (taicpu(hpfar1).oper[2]^.val = $FF) and
                 (taicpu(p).oper[2]^.shifterop^.shiftimm in [8,16,24]) and
                 MatchOperand(taicpu(hpfar1).oper[1]^, taicpu(p).oper[0]^.reg) then
                 begin
                   taicpu(hpfar1).ops := 3;
                   taicpu(hpfar1).opcode := A_UXTB;
                   taicpu(hpfar1).oper[1]^.reg := taicpu(p).oper[1]^.reg;
                   taicpu(hpfar1).loadshifterop(2,taicpu(p).oper[2]^.shifterop^);
                   taicpu(hpfar1).oper[2]^.shifterop^.shiftmode:=SM_ROR;

                   if not Assigned(hp1) then
                     GetNextInstruction(p,hp1);

                   RemoveCurrentP(p, hp1);

                   result:=true;
                   exit;
                 end;
             end;

          { 2-operald mov optimisations }
          if (taicpu(p).ops = 2) then
            begin
              {
                This removes the mul from
                mov rX,0
                ...
                mul ...,rX,...
              }
              if (taicpu(p).oper[1]^.typ = top_const) then
                begin
    (*          if false and
                (taicpu(p).oper[1]^.val=0) and
                MatchInstruction(hpfar1, [A_MUL,A_MLA], [taicpu(p).condition], [taicpu(p).oppostfix]) and
                (((taicpu(hpfar1).oper[1]^.typ=top_reg) and MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[1]^)) or
                 ((taicpu(hpfar1).oper[2]^.typ=top_reg) and MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[2]^))) then
                  begin
                    TransferUsedRegs(TmpUsedRegs);
                    UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                    UpdateUsedRegs(TmpUsedRegs, tai(hpfar1.next));
                    DebugMsg('Peephole Optimization: MovMUL/MLA2Mov0 done', p);
                    if taicpu(hpfar1).opcode=A_MUL then
                      taicpu(hpfar1).loadconst(1,0)
                    else
                      taicpu(hpfar1).loadreg(1,taicpu(hpfar1).oper[3]^.reg);
                    taicpu(hpfar1).ops:=2;
                    taicpu(hpfar1).opcode:=A_MOV;
                    if not(RegUsedAfterInstruction(taicpu(p).oper[0]^.reg,hpfar1,TmpUsedRegs)) then
                      RemoveCurrentP(p);
                    Result:=true;
                    exit;
                  end
              else*) if (taicpu(p).oper[1]^.val=0) and
                  MatchInstruction(hpfar1, A_MLA, [taicpu(p).condition], [taicpu(p).oppostfix]) and
                  MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[3]^) then
                    begin
                      TransferUsedRegs(TmpUsedRegs);
                      UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                      UpdateUsedRegs(TmpUsedRegs, tai(hpfar1.next));
                      DebugMsg('Peephole Optimization: MovMLA2MUL 1 done', p);
                      taicpu(hpfar1).ops:=3;
                      taicpu(hpfar1).opcode:=A_MUL;
                      if not(RegUsedAfterInstruction(taicpu(p).oper[0]^.reg,hpfar1,TmpUsedRegs)) then
                        begin
                          RemoveCurrentP(p);
                          Result:=true;
                        end;
                      exit;
                    end
                {
                  This changes the very common
                  mov r0, #0
                  str r0, [...]
                  mov r0, #0
                  str r0, [...]

                  and removes all superfluous mov instructions
                }
                else if (taicpu(hpfar1).opcode=A_STR) then
                  begin
                    hp1 := hpfar1;
                    while MatchInstruction(hp1, A_STR, [taicpu(p).condition], []) and
                          MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[0]^) and
                          GetNextInstruction(hp1, hp2) and
                          MatchInstruction(hp2, A_MOV, [taicpu(p).condition], [PF_None]) and
                          (taicpu(hp2).ops = 2) and
                          MatchOperand(taicpu(hp2).oper[0]^, taicpu(p).oper[0]^) and
                          MatchOperand(taicpu(hp2).oper[1]^, taicpu(p).oper[1]^) do
                      begin
                        DebugMsg('Peephole Optimization: MovStrMov done', hp2);
                        GetNextInstruction(hp2,hp1);
                        asml.remove(hp2);
                        hp2.free;
                        result:=true;
                        if not assigned(hp1) then break;
                      end;

                    if Result then
                      Exit;
                  end;
                end;
              {
                This removes the first mov from
                mov rX,...
                mov rX,...
              }
              if taicpu(hpfar1).opcode=A_MOV then
                begin
                  hp1 := p;
                  while MatchInstruction(hpfar1, A_MOV, [taicpu(hp1).condition], [taicpu(hp1).oppostfix]) and
                        (taicpu(hpfar1).ops = 2) and
                        MatchOperand(taicpu(hp1).oper[0]^, taicpu(hpfar1).oper[0]^) and
                        { don't remove the first mov if the second is a mov rX,rX }
                        not(MatchOperand(taicpu(hpfar1).oper[0]^, taicpu(hpfar1).oper[1]^)) do
                    begin
                      { Defer removing the first p until after the while loop }
                      if p <> hp1 then
                        begin
                          DebugMsg('Peephole Optimization: MovMov done', hp1);
                          asml.remove(hp1);
                          hp1.free;
                        end;
                      hp1:=hpfar1;
                      GetNextInstruction(hpfar1,hpfar1);
                      result:=true;
                      if not assigned(hpfar1) then
                        Break;
                    end;

                  if Result then
                    begin
                      DebugMsg('Peephole Optimization: MovMov done', p);
                      RemoveCurrentp(p);
                      Exit;
                    end;
                end;

              if RedundantMovProcess(p,hpfar1) then
                begin
                  Result:=true;
                  { p might not point at a mov anymore }
                  exit;
                end;

              { If hpfar1 is nil after the call to RedundantMovProcess, it is
                because it would have become a dangling pointer, so reinitialise it. }
              if not Assigned(hpfar1) then
                Continue;

              { Fold the very common sequence
                  mov  regA, regB
                  ldr* regA, [regA]
                to
                  ldr* regA, [regB]
                CAUTION! If this one is successful p might not be a mov instruction anymore!
              }
              if
                 // Make sure that Thumb code doesn't propagate a high register into a reference
                 (
                   (
                     GenerateThumbCode and
                     (getsupreg(taicpu(p).oper[1]^.reg) < RS_R8)
                   ) or (not GenerateThumbCode)
                 ) and
                 (taicpu(p).oper[1]^.typ = top_reg) and
                 (taicpu(p).oppostfix = PF_NONE) and
                 MatchInstruction(hpfar1, [A_LDR, A_STR], [taicpu(p).condition], []) and
                 (taicpu(hpfar1).oper[1]^.typ = top_ref) and
                 { We can change the base register only when the instruction uses AM_OFFSET }
                 ((taicpu(hpfar1).oper[1]^.ref^.index = taicpu(p).oper[0]^.reg) or
                   ((taicpu(hpfar1).oper[1]^.ref^.addressmode = AM_OFFSET) and
                    (taicpu(hpfar1).oper[1]^.ref^.base = taicpu(p).oper[0]^.reg))
                 ) and
                 not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hpfar1)) and
                 RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hpfar1)) then
                begin
                  DebugMsg('Peephole Optimization: MovLdr2Ldr done', hpfar1);
                  if (taicpu(hpfar1).oper[1]^.ref^.addressmode = AM_OFFSET) and
                     (taicpu(hpfar1).oper[1]^.ref^.base = taicpu(p).oper[0]^.reg) then
                    taicpu(hpfar1).oper[1]^.ref^.base := taicpu(p).oper[1]^.reg;

                  if taicpu(hpfar1).oper[1]^.ref^.index = taicpu(p).oper[0]^.reg then
                    taicpu(hpfar1).oper[1]^.ref^.index := taicpu(p).oper[1]^.reg;

                  dealloc:=FindRegDeAlloc(taicpu(p).oper[1]^.reg, tai(p.Next));
                  if Assigned(dealloc) then
                    begin
                      asml.remove(dealloc);
                      asml.InsertAfter(dealloc,hpfar1);
                    end;

                  if (not Assigned(hp1)) or (p=hp1) then
                    GetNextInstruction(p, hp1);

                  RemoveCurrentP(p, hp1);

                  result:=true;
                  Exit;
                end
            end

          { 3-operald mov optimisations }
          else if (taicpu(p).ops = 3) then
            begin

              if (taicpu(p).oper[2]^.typ = top_shifterop) and
                (taicpu(p).oper[2]^.shifterop^.rs = NR_NO) and
                (taicpu(p).oper[2]^.shifterop^.shiftmode = SM_LSR) and
                (taicpu(hpfar1).ops>=1) and
                (taicpu(hpfar1).oper[0]^.typ=top_reg) and
                (not RegModifiedBetween(taicpu(hpfar1).oper[0]^.reg, p, hpfar1)) and
                RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hpfar1)) then
                begin
                  if (taicpu(p).oper[2]^.shifterop^.shiftimm >= 24 ) and
                    MatchInstruction(hpfar1, A_AND, [taicpu(p).condition], [taicpu(p).oppostfix]) and
                    (taicpu(hpfar1).ops=3) and
                    MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[1]^) and
                    (taicpu(hpfar1).oper[2]^.typ = top_const) and
                    { Check if the AND actually would only mask out bits being already zero because of the shift
                    }
                    ((($ffffffff shr taicpu(p).oper[2]^.shifterop^.shiftimm) and taicpu(hpfar1).oper[2]^.val) =
                      ($ffffffff shr taicpu(p).oper[2]^.shifterop^.shiftimm)) then
                    begin
                      DebugMsg('Peephole Optimization: LsrAnd2Lsr done', hpfar1);
                      taicpu(p).oper[0]^.reg:=taicpu(hpfar1).oper[0]^.reg;
                      asml.remove(hpfar1);
                      hpfar1.free;
                      result:=true;
                      Exit;
                    end
                  else if MatchInstruction(hpfar1, A_BIC, [taicpu(p).condition], [taicpu(p).oppostfix]) and
                    (taicpu(hpfar1).ops=3) and
                    MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[1]^) and
                    (taicpu(hpfar1).oper[2]^.typ = top_const) and
                    { Check if the BIC actually would only mask out bits beeing already zero because of the shift }
                    (taicpu(hpfar1).oper[2]^.val<>0) and
                    (BsfDWord(taicpu(hpfar1).oper[2]^.val)>=32-taicpu(p).oper[2]^.shifterop^.shiftimm) then
                    begin
                      DebugMsg('Peephole Optimization: LsrBic2Lsr done', hpfar1);
                      taicpu(p).oper[0]^.reg:=taicpu(hpfar1).oper[0]^.reg;
                      asml.remove(hpfar1);
                      hpfar1.free;
                      result:=true;
                      Exit;
                    end;
                end;
              { This folds shifterops into following instructions
                mov r0, r1, lsl #8
                add r2, r3, r0

                to

                add r2, r3, r1, lsl #8
                CAUTION! If this one is successful p might not be a mov instruction anymore!
              }
              if (taicpu(p).oper[1]^.typ = top_reg) and
               (taicpu(p).oper[2]^.typ = top_shifterop) and
               (taicpu(p).oppostfix = PF_NONE) and
               MatchInstruction(hpfar1, [A_ADD, A_ADC, A_RSB, A_RSC, A_SUB, A_SBC,
                                      A_AND, A_BIC, A_EOR, A_ORR, A_TEQ, A_TST,
                                      A_CMP, A_CMN],
                                [taicpu(p).condition], [PF_None]) and
               (not ((GenerateThumb2Code) and
                     (taicpu(hpfar1).opcode in [A_SBC]) and
                     (((taicpu(hpfar1).ops=3) and
                       MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[1]^.reg)) or
                      ((taicpu(hpfar1).ops=2) and
                       MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[0]^.reg))))) and
               RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hpfar1)) and
               (taicpu(hpfar1).ops >= 2) and
               {Currently we can't fold into another shifterop}
               (taicpu(hpfar1).oper[taicpu(hpfar1).ops-1]^.typ = top_reg) and
               {Folding rrx is problematic because of the C-Flag, as we currently can't check
                NR_DEFAULTFLAGS for modification}
               (
                 {Everything is fine if we don't use RRX}
                 (taicpu(p).oper[2]^.shifterop^.shiftmode <> SM_RRX) or
                 (
                   {If it is RRX, then check if we're just accessing the next instruction}
                   Assigned(hp1) and
                   (hpfar1 = hp1)
                 )
               ) and
               { reg1 might not be modified inbetween }
               not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hpfar1)) and
               { The shifterop can contain a register, might not be modified}
               (
                 (taicpu(p).oper[2]^.shifterop^.rs = NR_NO) or
                 not(RegModifiedBetween(taicpu(p).oper[2]^.shifterop^.rs, p, hpfar1))
               ) and
               (
                 {Only ONE of the two src operands is allowed to match}
                 MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[taicpu(hpfar1).ops-2]^) xor
                 MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[taicpu(hpfar1).ops-1]^)
               ) then
              begin
                if taicpu(hpfar1).opcode in [A_TST, A_TEQ, A_CMN] then
                  I2:=0
                else
                  I2:=1;
                for I:=I2 to taicpu(hpfar1).ops-1 do
                  if MatchOperand(taicpu(p).oper[0]^, taicpu(hpfar1).oper[I]^.reg) then
                    begin
                      { If the parameter matched on the second op from the RIGHT
                        we have to switch the parameters, this will not happen for CMP
                        were we're only evaluating the most right parameter
                      }
                      if I <> taicpu(hpfar1).ops-1 then
                        begin
                          {The SUB operators need to be changed when we swap parameters}
                          case taicpu(hpfar1).opcode of
                            A_SUB: tempop:=A_RSB;
                            A_SBC: tempop:=A_RSC;
                            A_RSB: tempop:=A_SUB;
                            A_RSC: tempop:=A_SBC;
                            else tempop:=taicpu(hpfar1).opcode;
                          end;
                          if taicpu(hpfar1).ops = 3 then
                            hp2:=taicpu.op_reg_reg_reg_shifterop(tempop,
                                 taicpu(hpfar1).oper[0]^.reg, taicpu(hpfar1).oper[2]^.reg,
                                 taicpu(p).oper[1]^.reg, taicpu(p).oper[2]^.shifterop^)
                          else
                            hp2:=taicpu.op_reg_reg_shifterop(tempop,
                                 taicpu(hpfar1).oper[0]^.reg, taicpu(p).oper[1]^.reg,
                                 taicpu(p).oper[2]^.shifterop^);
                        end
                      else
                        if taicpu(hpfar1).ops = 3 then
                          hp2:=taicpu.op_reg_reg_reg_shifterop(taicpu(hpfar1).opcode,
                               taicpu(hpfar1).oper[0]^.reg, taicpu(hpfar1).oper[1]^.reg,
                               taicpu(p).oper[1]^.reg, taicpu(p).oper[2]^.shifterop^)
                        else
                          hp2:=taicpu.op_reg_reg_shifterop(taicpu(hpfar1).opcode,
                               taicpu(hpfar1).oper[0]^.reg, taicpu(p).oper[1]^.reg,
                               taicpu(p).oper[2]^.shifterop^);
                      if taicpu(p).oper[2]^.shifterop^.rs<>NR_NO then
                        AllocRegBetween(taicpu(p).oper[2]^.shifterop^.rs,p,hpfar1,UsedRegs);
                      AllocRegBetween(taicpu(p).oper[1]^.reg,p,hpfar1,UsedRegs);
                      asml.insertbefore(hp2, hpfar1);
                      asml.remove(hpfar1);
                      hpfar1.free;
                      DebugMsg('Peephole Optimization: FoldShiftProcess done', hp2);

                      if not Assigned(hp1) then
                        GetNextInstruction(p, hp1)
                      else if hp1 = hpfar1 then
                        { If hp1 = hpfar1, then it's a dangling pointer }
                        hp1 := hp2;

                      RemoveCurrentP(p, hp1);
                      Result:=true;
                      Exit;
                    end;
              end;
            {
              Fold
                mov r1, r1, lsl #2
                ldr/ldrb r0, [r0, r1]
              to
                ldr/ldrb r0, [r0, r1, lsl #2]

              XXX: This still needs some work, as we quite often encounter something like
                     mov r1, r2, lsl #2
                     add r2, r3, #imm
                     ldr r0, [r2, r1]
                   which can't be folded because r2 is overwritten between the shift and the ldr.
                   We could try to shuffle the registers around and fold it into.
                     add r1, r3, #imm
                     ldr r0, [r1, r2, lsl #2]
            }
            if (not(GenerateThumbCode)) and
              { thumb2 allows only lsl #0..#3 }
              (not(GenerateThumb2Code) or
               ((taicpu(p).oper[2]^.shifterop^.shiftimm in [0..3]) and
                (taicpu(p).oper[2]^.shifterop^.shiftmode=SM_LSL)
               )
              ) and
               (taicpu(p).oper[1]^.typ = top_reg) and
               (taicpu(p).oper[2]^.typ = top_shifterop) and
               { RRX is tough to handle, because it requires tracking the C-Flag,
                 it is also extremly unlikely to be emitted this way}
               (taicpu(p).oper[2]^.shifterop^.shiftmode <> SM_RRX) and
               (taicpu(p).oper[2]^.shifterop^.shiftimm <> 0) and
               (taicpu(p).oppostfix = PF_NONE) and
               {Only LDR, LDRB, STR, STRB can handle scaled register indexing}
               (MatchInstruction(hpfar1, [A_LDR, A_STR], [taicpu(p).condition], [PF_None, PF_B]) or
                (GenerateThumb2Code and
                 MatchInstruction(hpfar1, [A_LDR, A_STR], [taicpu(p).condition], [PF_None, PF_B, PF_SB, PF_H, PF_SH]))
               ) and
               (
                 {If this is address by offset, one of the two registers can be used}
                 ((taicpu(hpfar1).oper[1]^.ref^.addressmode=AM_OFFSET) and
                   (
                     (taicpu(hpfar1).oper[1]^.ref^.index = taicpu(p).oper[0]^.reg) xor
                     (taicpu(hpfar1).oper[1]^.ref^.base = taicpu(p).oper[0]^.reg)
                   )
                 ) or
                 {For post and preindexed only the index register can be used}
                 ((taicpu(hpfar1).oper[1]^.ref^.addressmode in [AM_POSTINDEXED, AM_PREINDEXED]) and
                   (
                     (taicpu(hpfar1).oper[1]^.ref^.index = taicpu(p).oper[0]^.reg) and
                     (taicpu(hpfar1).oper[1]^.ref^.base <> taicpu(p).oper[0]^.reg)
                   ) and
                   (not GenerateThumb2Code)
                 )
               ) and
               { Only fold if both registers are used. Otherwise we are folding p with itself }
               (taicpu(hpfar1).oper[1]^.ref^.index<>NR_NO) and
               (taicpu(hpfar1).oper[1]^.ref^.base<>NR_NO) and
               { Only fold if there isn't another shifterop already, and offset is zero. }
               (taicpu(hpfar1).oper[1]^.ref^.offset = 0) and
               (taicpu(hpfar1).oper[1]^.ref^.shiftmode = SM_None) and
               not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hpfar1)) and
               RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(hpfar1)) then
               begin
                 { If the register we want to do the shift for resides in base, we need to swap that}
                 if (taicpu(hpfar1).oper[1]^.ref^.base = taicpu(p).oper[0]^.reg) then
                   taicpu(hpfar1).oper[1]^.ref^.base := taicpu(hpfar1).oper[1]^.ref^.index;
                 taicpu(hpfar1).oper[1]^.ref^.index := taicpu(p).oper[1]^.reg;
                 taicpu(hpfar1).oper[1]^.ref^.shiftmode := taicpu(p).oper[2]^.shifterop^.shiftmode;
                 taicpu(hpfar1).oper[1]^.ref^.shiftimm := taicpu(p).oper[2]^.shifterop^.shiftimm;
                 DebugMsg('Peephole Optimization: FoldShiftLdrStr done', hpfar1);
                 RemoveCurrentP(p);
                 Result:=true;
                 Exit;
               end;
            end;
          {
            Often we see shifts and then a superfluous mov to another register
            In the future this might be handled in RedundantMovProcess when it uses RegisterTracking
          }
          if RemoveSuperfluousMove(p, hpfar1, 'MovMov2Mov') then
            Result:=true;

          Exit;
        end;
    end;


  function TCpuAsmOptimizer.OptPass1MVN(var p: tai): Boolean;
    var
      hp1: tai;
    begin
      {
        change
        mvn reg2,reg1
        and reg3,reg4,reg2
        dealloc reg2
        to
        bic reg3,reg4,reg1
      }
      Result := False;
      if (taicpu(p).oper[1]^.typ = top_reg) and
        GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[0]^.reg) and
        MatchInstruction(hp1,A_AND,[],[]) and
        (((taicpu(hp1).ops=3) and
          (taicpu(hp1).oper[2]^.typ=top_reg) and
          (MatchOperand(taicpu(hp1).oper[2]^, taicpu(p).oper[0]^.reg) or
           MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg))) or
         ((taicpu(hp1).ops=2) and
          (taicpu(hp1).oper[1]^.typ=top_reg) and
          MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg))) and
        assigned(FindRegDealloc(taicpu(p).oper[0]^.reg,tai(hp1.Next))) and
        { reg1 might not be modified inbetween }
        not(RegModifiedBetween(taicpu(p).oper[1]^.reg,p,hp1)) then
        begin
          DebugMsg('Peephole Optimization: MvnAnd2Bic done', p);
          taicpu(hp1).opcode:=A_BIC;

          if taicpu(hp1).ops=3 then
            begin
              if MatchOperand(taicpu(hp1).oper[1]^, taicpu(p).oper[0]^.reg) then
                taicpu(hp1).loadReg(1,taicpu(hp1).oper[2]^.reg); // Swap operands

              taicpu(hp1).loadReg(2,taicpu(p).oper[1]^.reg);
            end
          else
            taicpu(hp1).loadReg(1,taicpu(p).oper[1]^.reg);

          RemoveCurrentp(p);
          Result := True;
          Exit;
        end;
    end;


  function TCpuAsmOptimizer.OptPass1VMov(var p: tai): Boolean;
    var
      hp1: tai;
    begin
      {
        change
        vmov reg0,reg1,reg2
        vmov reg1,reg2,reg0
        into
        vmov reg0,reg1,reg2

        can be applied regardless if reg0 or reg2 is the vfp register
      }
      Result := False;
      if (taicpu(p).ops = 3) then
        while GetNextInstruction(p, hp1) and
          MatchInstruction(hp1, A_VMOV, [taicpu(p).condition], [taicpu(p).oppostfix]) and
          (taicpu(hp1).ops = 3) and
          MatchOperand(taicpu(p).oper[0]^, taicpu(hp1).oper[2]^) and
          MatchOperand(taicpu(p).oper[1]^, taicpu(hp1).oper[0]^) and
          MatchOperand(taicpu(p).oper[2]^, taicpu(hp1).oper[1]^) do
          begin
            asml.Remove(hp1);
            hp1.free;
            DebugMsg('Peephole Optimization: VMovVMov2VMov done', p);
            { Can we do it again? }
          end;
    end;


  function TCpuAsmOptimizer.OptPass1VOp(var p: tai): Boolean;
    var
      hp1: tai;
    begin
      Result := GetNextInstructionUsingReg(p, hp1, taicpu(p).oper[0]^.reg) and
        RemoveSuperfluousVMov(p, hp1, 'VOpVMov2VOp');
    end;


  function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    begin
      result := false;
      if p.typ = ait_instruction then
        begin
          case taicpu(p).opcode of
            A_CMP:
              Result := OptPass1CMP(p);
            A_STR:
              Result := OptPass1STR(p);
            A_LDR:
              Result := OptPass1LDR(p);
            A_MOV:
              Result := OptPass1MOV(p);
            A_AND:
              Result := OptPass1And(p);
            A_ADD,
            A_SUB:
              Result := OptPass1ADDSUB(p);
            A_MUL:
              REsult := OptPass1MUL(p);
            A_ADC,
            A_RSB,
            A_RSC,
            A_SBC,
            A_BIC,
            A_EOR,
            A_ORR,
            A_MLA,
            A_MLS,
            A_QADD,A_QADD16,A_QADD8,
            A_QSUB,A_QSUB16,A_QSUB8,
            A_QDADD,A_QDSUB,A_QASX,A_QSAX,
            A_SHADD16,A_SHADD8,A_UHADD16,A_UHADD8,
            A_SHSUB16,A_SHSUB8,A_UHSUB16,A_UHSUB8,
            A_PKHTB,A_PKHBT,
            A_SMUAD,A_SMUSD:
              Result := OptPass1DataCheckMov(p);
{$ifdef dummy}
            A_MVN:
              Result := OPtPass1MVN(p);
{$endif dummy}
            A_UXTB:
              Result := OptPass1UXTB(p);
            A_UXTH:
              Result := OptPass1UXTH(p);
            A_SXTB:
              Result := OptPass1SXTB(p);
            A_SXTH:
              Result := OptPass1SXTH(p);
            A_STM:
              Result := OptPass1STM(p);
            A_VMOV:
              Result := OptPass1VMov(p);
            A_VLDR,
            A_VADD,
            A_VMUL,
            A_VDIV,
            A_VSUB,
            A_VSQRT,
            A_VNEG,
            A_VCVT,
            A_VABS:
              Result := OptPass1VOp(p);
            else
              ;
          end;
        end;
    end;


  { instructions modifying the CPSR can be only the last instruction }
  function MustBeLast(p : tai) : boolean;
    begin
      Result:=(p.typ=ait_instruction) and
        ((taicpu(p).opcode in [A_BL,A_BLX,A_CMP,A_CMN,A_SWI,A_TEQ,A_TST,A_CMF,A_CMFE {,A_MSR}]) or
         ((taicpu(p).ops>=1) and (taicpu(p).oper[0]^.typ=top_reg) and (taicpu(p).oper[0]^.reg=NR_PC)) or
         (taicpu(p).oppostfix=PF_S));
    end;


  procedure TCpuAsmOptimizer.PeepHoleOptPass2;
    var
      p,hp1,hp2: tai;
      l : longint;
      condition : tasmcond;
      hp3: tai;
      WasLast: boolean;
      { UsedRegs, TmpUsedRegs: TRegSet; }

    begin
      p := BlockStart;
      { UsedRegs := []; }
      while (p <> BlockEnd) Do
        begin
          { UpdateUsedRegs(UsedRegs, tai(p.next)); }
          case p.Typ Of
            Ait_Instruction:
              begin
                case taicpu(p).opcode Of
                  A_B:
                    if (taicpu(p).condition<>C_None) and
                      not(GenerateThumbCode) then
                      begin
                         { check for
                                Bxx   xxx
                                <several instructions>
                             xxx:
                         }
                         l:=0;
                         WasLast:=False;
                         GetNextInstruction(p, hp1);
                         while assigned(hp1) and
                           (l<=4) and
                           CanBeCond(hp1) and
                           { stop on labels }
                           not(hp1.typ=ait_label) and
                           { avoid that we cannot recognize the case BccB2Cond }
                           not((hp1.typ=ait_instruction) and (taicpu(hp1).opcode=A_B)) do
                           begin
                              inc(l);
                              if MustBeLast(hp1) then
                                begin
                                  WasLast:=True;
                                  GetNextInstruction(hp1,hp1);
                                  break;
                                end
                              else
                                GetNextInstruction(hp1,hp1);
                           end;
                         if assigned(hp1) then
                           begin
                              if FindLabel(tasmlabel(taicpu(p).oper[0]^.ref^.symbol),hp1) then
                                begin
                                  if (l<=4) and (l>0) then
                                    begin
                                      condition:=inverse_cond(taicpu(p).condition);
                                      hp2:=p;
                                      GetNextInstruction(p,hp1);
                                      p:=hp1;
                                      repeat
                                        if hp1.typ=ait_instruction then
                                          taicpu(hp1).condition:=condition;
                                        if MustBeLast(hp1) then
                                          begin
                                            GetNextInstruction(hp1,hp1);
                                            break;
                                          end
                                        else
                                          GetNextInstruction(hp1,hp1);
                                      until not(assigned(hp1)) or
                                        not(CanBeCond(hp1)) or
                                        (hp1.typ=ait_label);
                                      DebugMsg('Peephole Bcc2Cond done',hp2);
                                      { wait with removing else GetNextInstruction could
                                        ignore the label if it was the only usage in the
                                        jump moved away }
                                      tasmlabel(taicpu(hp2).oper[0]^.ref^.symbol).decrefs;
                                      asml.remove(hp2);
                                      hp2.free;
                                      continue;
                                    end;
                                end
                              else
                                { do not perform further optimizations if there is inctructon
                                  in block #1 which can not be optimized.
                                 }
                                if not WasLast then
                                begin
                                   { check further for
                                          Bcc   xxx
                                          <several instructions 1>
                                          B   yyy
                                  xxx:
                                          <several instructions 2>
                                  yyy:
                                   }
                                  { hp2 points to jmp yyy }
                                  hp2:=hp1;
                                  { skip hp1 to xxx }
                                  GetNextInstruction(hp1, hp1);
                                  if assigned(hp2) and
                                    assigned(hp1) and
                                    (l<=3) and
                                    (hp2.typ=ait_instruction) and
                                    (taicpu(hp2).is_jmp) and
                                    (taicpu(hp2).condition=C_None) and
                                    { real label and jump, no further references to the
                                      label are allowed }
                                    (tasmlabel(taicpu(p).oper[0]^.ref^.symbol).getrefs=1) and
                                    FindLabel(tasmlabel(taicpu(p).oper[0]^.ref^.symbol),hp1) then
                                     begin
                                       l:=0;
                                       { skip hp1 to <several moves 2> }
                                       GetNextInstruction(hp1, hp1);
                                       while assigned(hp1) and
                                         CanBeCond(hp1) and
                                         (l<=3) do
                                         begin
                                           inc(l);
                                           if MustBeLast(hp1) then
                                             begin
                                               GetNextInstruction(hp1, hp1);
                                               break;
                                             end
                                           else
                                             GetNextInstruction(hp1, hp1);
                                         end;
                                       { hp1 points to yyy: }
                                       if assigned(hp1) and
                                         FindLabel(tasmlabel(taicpu(hp2).oper[0]^.ref^.symbol),hp1) then
                                         begin
                                            condition:=inverse_cond(taicpu(p).condition);
                                            GetNextInstruction(p,hp1);
                                            hp3:=p;
                                            p:=hp1;
                                            repeat
                                              if hp1.typ=ait_instruction then
                                                taicpu(hp1).condition:=condition;
                                              if MustBeLast(hp1) then
                                                begin
                                                  GetNextInstruction(hp1, hp1);
                                                  break;
                                                end
                                              else
                                                GetNextInstruction(hp1, hp1);
                                            until not(assigned(hp1)) or
                                              not(CanBeCond(hp1)) or
                                              ((hp1.typ=ait_instruction) and (taicpu(hp1).opcode=A_B));
                                            { hp2 is still at jmp yyy }
                                            GetNextInstruction(hp2,hp1);
                                            { hp1 is now at xxx: }
                                            condition:=inverse_cond(condition);
                                            GetNextInstruction(hp1,hp1);
                                            { hp1 is now at <several movs 2> }
                                            repeat
                                              if hp1.typ=ait_instruction then
                                                taicpu(hp1).condition:=condition;
                                              GetNextInstruction(hp1,hp1);
                                            until not(assigned(hp1)) or
                                              not(CanBeCond(hp1)) or
                                              (hp1.typ=ait_label);
                                            DebugMsg('Peephole BccB2Cond done',hp3);
                                            { remove Bcc }
                                            tasmlabel(taicpu(hp3).oper[0]^.ref^.symbol).decrefs;
                                            asml.remove(hp3);
                                            hp3.free;
                                            { remove B }
                                            tasmlabel(taicpu(hp2).oper[0]^.ref^.symbol).decrefs;
                                            asml.remove(hp2);
                                            hp2.free;
                                            continue;
                                         end;
                                     end;
                                end;
                           end;
                      end;
                  else
                    ;
                end;
              end;
            else
              ;
          end;
          p := tai(p.next)
        end;
    end;

  function TCpuAsmOptimizer.RegInInstruction(Reg: TRegister; p1: tai): Boolean;
    begin
      If (p1.typ = ait_instruction) and (taicpu(p1).opcode=A_BL) then
        Result:=true
      else If MatchInstruction(p1, [A_LDR, A_STR], [], [PF_D]) and
              (getsupreg(taicpu(p1).oper[0]^.reg)+1=getsupreg(reg)) then
        Result:=true
      else
        Result:=inherited RegInInstruction(Reg, p1);
    end;

  const
    { set of opcode which might or do write to memory }
    { TODO : extend armins.dat to contain r/w info }
    opcode_could_mem_write = [A_B,A_BL,A_BLX,A_BKPT,A_BX,A_STR,A_STRB,A_STRBT,
                              A_STRH,A_STRT,A_STF,A_SFM,A_STM,A_FSTS,A_FSTD,A_VSTR,A_VSTM];


  { adjust the register live information when swapping the two instructions p and hp1,
    they must follow one after the other }
  procedure TCpuPreRegallocScheduler.SwapRegLive(p,hp1 : taicpu);

    procedure CheckLiveEnd(reg : tregister);
      var
        supreg : TSuperRegister;
        regtype : TRegisterType;
      begin
        if reg=NR_NO then
          exit;
        regtype:=getregtype(reg);
        supreg:=getsupreg(reg);
        if assigned(cg.rg[regtype]) and (cg.rg[regtype].live_end[supreg]=hp1) and
          RegInInstruction(reg,p) then
          cg.rg[regtype].live_end[supreg]:=p;
      end;


    procedure CheckLiveStart(reg : TRegister);
      var
        supreg : TSuperRegister;
        regtype : TRegisterType;
      begin
        if reg=NR_NO then
          exit;
        regtype:=getregtype(reg);
        supreg:=getsupreg(reg);
        if assigned(cg.rg[regtype]) and (cg.rg[regtype].live_start[supreg]=p) and
          RegInInstruction(reg,hp1) then
         cg.rg[regtype].live_start[supreg]:=hp1;
      end;

    var
      i : longint;
      r : TSuperRegister;
    begin
      { assumption: p is directly followed by hp1 }

      { if live of any reg used by p starts at p and hp1 uses this register then
        set live start to hp1 }
      for i:=0 to p.ops-1 do
        case p.oper[i]^.typ of
          Top_Reg:
            CheckLiveStart(p.oper[i]^.reg);
          Top_Ref:
            begin
              CheckLiveStart(p.oper[i]^.ref^.base);
              CheckLiveStart(p.oper[i]^.ref^.index);
            end;
          Top_Shifterop:
            CheckLiveStart(p.oper[i]^.shifterop^.rs);
          Top_RegSet:
            for r:=RS_R0 to RS_R15 do
               if r in p.oper[i]^.regset^ then
                 CheckLiveStart(newreg(R_INTREGISTER,r,R_SUBWHOLE));
          else
            ;
        end;

      { if live of any reg used by hp1 ends at hp1 and p uses this register then
        set live end to p }
      for i:=0 to hp1.ops-1 do
        case hp1.oper[i]^.typ of
          Top_Reg:
            CheckLiveEnd(hp1.oper[i]^.reg);
          Top_Ref:
            begin
              CheckLiveEnd(hp1.oper[i]^.ref^.base);
              CheckLiveEnd(hp1.oper[i]^.ref^.index);
            end;
          Top_Shifterop:
            CheckLiveStart(hp1.oper[i]^.shifterop^.rs);
          Top_RegSet:
            for r:=RS_R0 to RS_R15 do
               if r in hp1.oper[i]^.regset^ then
                 CheckLiveEnd(newreg(R_INTREGISTER,r,R_SUBWHOLE));
          else
            ;
        end;
    end;


  function TCpuPreRegallocScheduler.SchedulerPass1Cpu(var p: tai): boolean;

  { TODO : schedule also forward }
  { TODO : schedule distance > 1 }

    { returns true if p might be a load of a pc relative tls offset }
    function PossibleTLSLoad(const p: tai) : boolean;
      begin
        Result:=(p.typ=ait_instruction) and (taicpu(p).opcode=A_LDR) and (taicpu(p).oper[1]^.typ=top_ref) and (((taicpu(p).oper[1]^.ref^.base=NR_PC) and
          (taicpu(p).oper[1]^.ref^.index<>NR_NO)) or ((taicpu(p).oper[1]^.ref^.base<>NR_NO) and
          (taicpu(p).oper[1]^.ref^.index=NR_PC)));
      end;

    var
      hp1,hp2,hp3,hp4,hp5,insertpos : tai;
      list : TAsmList;
    begin
      result:=true;

      list:=TAsmList.create;
      p:=BlockStart;
      while p<>BlockEnd Do
        begin
          if (p.typ=ait_instruction) and
            GetNextInstruction(p,hp1) and
            (hp1.typ=ait_instruction) and
            (taicpu(hp1).opcode in [A_LDR,A_LDRB,A_LDRH,A_LDRSB,A_LDRSH]) and
            (taicpu(hp1).oppostfix in [PF_NONE, PF_B, PF_H, PF_SB, PF_SH]) and
            { for now we don't reschedule if the previous instruction changes potentially a memory location }
            ( (not(taicpu(p).opcode in opcode_could_mem_write) and
               not(RegModifiedByInstruction(NR_PC,p))
              ) or
              ((taicpu(p).opcode in [A_STM,A_STRB,A_STRH,A_STR]) and
               ((taicpu(hp1).oper[1]^.ref^.base=NR_PC) or
                (assigned(taicpu(hp1).oper[1]^.ref^.symboldata) and
                (taicpu(hp1).oper[1]^.ref^.offset=0)
                )
               ) or
               { try to prove that the memory accesses don't overlapp }
               ((taicpu(p).opcode in [A_STRB,A_STRH,A_STR]) and
                (taicpu(p).oper[1]^.typ = top_ref) and
                (taicpu(p).oper[1]^.ref^.base=taicpu(hp1).oper[1]^.ref^.base) and
                (taicpu(p).oppostfix=PF_None) and
                (taicpu(hp1).oppostfix=PF_None) and
                (taicpu(p).oper[1]^.ref^.index=NR_NO) and
                (taicpu(hp1).oper[1]^.ref^.index=NR_NO) and
                { get operand sizes and check if the offset distance is large enough to ensure no overlapp }
                (abs(taicpu(p).oper[1]^.ref^.offset-taicpu(hp1).oper[1]^.ref^.offset)>=max(tcgsize2size[reg_cgsize(taicpu(p).oper[0]^.reg)],tcgsize2size[reg_cgsize(taicpu(hp1).oper[0]^.reg)]))
              )
            )
            ) and
            GetNextInstruction(hp1,hp2) and
            (hp2.typ=ait_instruction) and
            { loaded register used by next instruction?

              if we ever support labels (they could be skipped in theory) here, the gnu2 tls general-dynamic code could get broken (the ldr before
              the bl may not be scheduled away from the bl) and it needs to be taken care of this case
            }
            (RegInInstruction(taicpu(hp1).oper[0]^.reg,hp2)) and
            { loaded register not used by previous instruction? }
            not(RegInInstruction(taicpu(hp1).oper[0]^.reg,p)) and
            { same condition? }
            (taicpu(p).condition=taicpu(hp1).condition) and
            { first instruction might not change the register used as base }
            ((taicpu(hp1).oper[1]^.ref^.base=NR_NO) or
             not(RegModifiedByInstruction(taicpu(hp1).oper[1]^.ref^.base,p))
            ) and
            { first instruction might not change the register used as index }
            ((taicpu(hp1).oper[1]^.ref^.index=NR_NO) or
             not(RegModifiedByInstruction(taicpu(hp1).oper[1]^.ref^.index,p))
            ) and
            { if we modify the basereg AND the first instruction used that reg, we can not schedule }
            ((taicpu(hp1).oper[1]^.ref^.addressmode = AM_OFFSET) or
             not(instructionLoadsFromReg(taicpu(hp1).oper[1]^.ref^.base,p))) and
            not(PossibleTLSLoad(p)) and
            not(PossibleTLSLoad(hp1)) then
            begin
              hp3:=tai(p.Previous);
              hp5:=tai(p.next);
              asml.Remove(p);
              { if there is a reg. alloc/dealloc/sync instructions or address labels (e.g. for GOT-less PIC)
                associated with p, move it together with p }

              { before the instruction? }
              { find reg allocs,deallocs and PIC labels }
              while assigned(hp3) and (hp3.typ<>ait_instruction) do
                begin
                  if ( (hp3.typ=ait_regalloc) and (tai_regalloc(hp3).ratype in [ra_alloc, ra_dealloc]) and
                    RegInInstruction(tai_regalloc(hp3).reg,p) )
                    or ( (hp3.typ=ait_label) and (tai_label(hp3).labsym.typ=AT_ADDR) )
                  then
                    begin
                      hp4:=hp3;
                      hp3:=tai(hp3.Previous);
                      asml.Remove(hp4);
                      list.Insert(hp4);
                    end
                  else
                    hp3:=tai(hp3.Previous);
                end;

              list.Concat(p);
              SwapRegLive(taicpu(p),taicpu(hp1));

              { after the instruction? }
              { find reg deallocs and reg syncs }
              while assigned(hp5) and (hp5.typ<>ait_instruction) do
                begin
                  if (hp5.typ=ait_regalloc) and (tai_regalloc(hp5).ratype in [ra_dealloc, ra_sync]) and
                    RegInInstruction(tai_regalloc(hp5).reg,p) then
                    begin
                      hp4:=hp5;
                      hp5:=tai(hp5.next);
                      asml.Remove(hp4);
                      list.Concat(hp4);
                    end
                  else
                    hp5:=tai(hp5.Next);
                end;

              asml.Remove(hp1);
              { if there are address labels associated with hp2, those must
                stay with hp2 (e.g. for GOT-less PIC) }
              insertpos:=hp2;
              while assigned(hp2.previous) and
                    (tai(hp2.previous).typ<>ait_instruction) do
                begin
                  hp2:=tai(hp2.previous);
                  if (hp2.typ=ait_label) and
                     (tai_label(hp2).labsym.typ=AT_ADDR) then
                    insertpos:=hp2;
                end;
{$ifdef DEBUG_PREREGSCHEDULER}
              asml.insertbefore(tai_comment.Create(strpnew('Rescheduled')),insertpos);
{$endif DEBUG_PREREGSCHEDULER}
              asml.InsertBefore(hp1,insertpos);
              asml.InsertListBefore(insertpos,list);
              p:=tai(p.next);
            end
          else if p.typ=ait_instruction then
            p:=hp1
          else
            p:=tai(p.next);
        end;
      list.Free;
    end;


  procedure DecrementPreceedingIT(list: TAsmList; p: tai);
    var
      hp : tai;
      l : longint;
    begin
      hp := tai(p.Previous);
      l := 1;

      while assigned(hp) and
        (l <= 4) do
        begin
          if hp.typ=ait_instruction then
            begin
              if (taicpu(hp).opcode>=A_IT) and
                (taicpu(hp).opcode <= A_ITTTT) then
                begin
                  if (taicpu(hp).opcode = A_IT) and
                     (l=1) then
                    list.Remove(hp)
                  else
                    case taicpu(hp).opcode of
                      A_ITE:
                        if l=2 then taicpu(hp).opcode := A_IT;
                      A_ITT:
                        if l=2 then taicpu(hp).opcode := A_IT;
                      A_ITEE:
                        if l=3 then taicpu(hp).opcode := A_ITE;
                      A_ITTE:
                        if l=3 then taicpu(hp).opcode := A_ITT;
                      A_ITET:
                        if l=3 then taicpu(hp).opcode := A_ITE;
                      A_ITTT:
                        if l=3 then taicpu(hp).opcode := A_ITT;
                      A_ITEEE:
                        if l=4 then taicpu(hp).opcode := A_ITEE;
                      A_ITTEE:
                        if l=4 then taicpu(hp).opcode := A_ITTE;
                      A_ITETE:
                        if l=4 then taicpu(hp).opcode := A_ITET;
                      A_ITTTE:
                        if l=4 then taicpu(hp).opcode := A_ITTT;
                      A_ITEET:
                        if l=4 then taicpu(hp).opcode := A_ITEE;
                      A_ITTET:
                        if l=4 then taicpu(hp).opcode := A_ITTE;
                      A_ITETT:
                        if l=4 then taicpu(hp).opcode := A_ITET;
                      A_ITTTT:
                        begin
                          if l=4 then taicpu(hp).opcode := A_ITTT;
                        end
                      else
                        ;
                    end;

                  break;
                end;
              {else if (taicpu(hp).condition<>taicpu(p).condition) or
                (taicpu(hp).condition<>inverse_cond(taicpu(p).condition)) then
                break;}

              inc(l);
            end;
          hp := tai(hp.Previous);
        end;
    end;

  function TCpuThumb2AsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    var
      hp : taicpu;
      //hp1,hp2 : tai;
    begin
      result:=false;
      if inherited PeepHoleOptPass1Cpu(p) then
        result:=true
      else if (p.typ=ait_instruction) and
        MatchInstruction(p, A_STM, [C_None], [PF_FD,PF_DB]) and
        (taicpu(p).oper[0]^.ref^.addressmode=AM_PREINDEXED) and
        (taicpu(p).oper[0]^.ref^.index=NR_STACK_POINTER_REG) and
        ((taicpu(p).oper[1]^.regset^*[8..13,15])=[]) then
        begin
          DebugMsg('Peephole Stm2Push done', p);
          hp := taicpu.op_regset(A_PUSH, R_INTREGISTER, R_SUBWHOLE, taicpu(p).oper[1]^.regset^);
          AsmL.InsertAfter(hp, p);
          asml.Remove(p);
          p:=hp;
          result:=true;
        end
      {else if (p.typ=ait_instruction) and
        MatchInstruction(p, A_STR, [C_None], [PF_None]) and
        (taicpu(p).oper[1]^.ref^.addressmode=AM_PREINDEXED) and
        (taicpu(p).oper[1]^.ref^.index=NR_STACK_POINTER_REG) and
        (taicpu(p).oper[1]^.ref^.offset=-4) and
        (getsupreg(taicpu(p).oper[0]^.reg) in [0..7,14]) then
        begin
          DebugMsg('Peephole Str2Push done', p);
          hp := taicpu.op_regset(A_PUSH, R_INTREGISTER, R_SUBWHOLE, [getsupreg(taicpu(p).oper[0]^.reg)]);
          asml.InsertAfter(hp, p);
          asml.Remove(p);
          p.Free;
          p:=hp;
          result:=true;
        end}
      else if (p.typ=ait_instruction) and
        MatchInstruction(p, A_LDM, [C_None], [PF_FD,PF_IA]) and
        (taicpu(p).oper[0]^.ref^.addressmode=AM_PREINDEXED) and
        (taicpu(p).oper[0]^.ref^.index=NR_STACK_POINTER_REG) and
        ((taicpu(p).oper[1]^.regset^*[8..14])=[]) then
        begin
          DebugMsg('Peephole Ldm2Pop done', p);
          hp := taicpu.op_regset(A_POP, R_INTREGISTER, R_SUBWHOLE, taicpu(p).oper[1]^.regset^);
          asml.InsertBefore(hp, p);
          asml.Remove(p);
          p.Free;
          p:=hp;
          result:=true;
        end
      {else if (p.typ=ait_instruction) and
        MatchInstruction(p, A_LDR, [C_None], [PF_None]) and
        (taicpu(p).oper[1]^.ref^.addressmode=AM_POSTINDEXED) and
        (taicpu(p).oper[1]^.ref^.index=NR_STACK_POINTER_REG) and
        (taicpu(p).oper[1]^.ref^.offset=4) and
        (getsupreg(taicpu(p).oper[0]^.reg) in [0..7,15]) then
        begin
          DebugMsg('Peephole Ldr2Pop done', p);
          hp := taicpu.op_regset(A_POP, R_INTREGISTER, R_SUBWHOLE, [getsupreg(taicpu(p).oper[0]^.reg)]);
          asml.InsertBefore(hp, p);
          asml.Remove(p);
          p.Free;
          p:=hp;
          result:=true;
        end}
      else if (p.typ=ait_instruction) and
        MatchInstruction(p, [A_AND], [], [PF_None]) and
        (taicpu(p).ops = 2) and
        (taicpu(p).oper[1]^.typ=top_const) and
        ((taicpu(p).oper[1]^.val=255) or
         (taicpu(p).oper[1]^.val=65535)) then
        begin
          DebugMsg('Peephole AndR2Uxt done', p);
          if taicpu(p).oper[1]^.val=255 then
            taicpu(p).opcode:=A_UXTB
          else
            taicpu(p).opcode:=A_UXTH;

          taicpu(p).loadreg(1, taicpu(p).oper[0]^.reg);

          result := true;
        end
      else if (p.typ=ait_instruction) and
        MatchInstruction(p, [A_AND], [], [PF_None]) and
        (taicpu(p).ops = 3) and
        (taicpu(p).oper[2]^.typ=top_const) and
        ((taicpu(p).oper[2]^.val=255) or
         (taicpu(p).oper[2]^.val=65535)) then
        begin
          DebugMsg('Peephole AndRR2Uxt done', p);
          if taicpu(p).oper[2]^.val=255 then
            taicpu(p).opcode:=A_UXTB
          else
            taicpu(p).opcode:=A_UXTH;

          taicpu(p).ops:=2;

          result := true;
        end
      {else if (p.typ=ait_instruction) and
        MatchInstruction(p, [A_CMP], [C_None], [PF_None]) and
        (taicpu(p).oper[1]^.typ=top_const) and
        (taicpu(p).oper[1]^.val=0) and
        GetNextInstruction(p,hp1) and
        (taicpu(hp1).opcode=A_B) and
        (taicpu(hp1).condition in [C_EQ,C_NE]) then
        begin
          if taicpu(hp1).condition = C_EQ then
            hp2:=taicpu.op_reg_ref(A_CBZ, taicpu(p).oper[0]^.reg, taicpu(hp1).oper[0]^.ref^)
          else
            hp2:=taicpu.op_reg_ref(A_CBNZ, taicpu(p).oper[0]^.reg, taicpu(hp1).oper[0]^.ref^);

          taicpu(hp2).is_jmp := true;

          asml.InsertAfter(hp2, hp1);

          asml.Remove(hp1);
          hp1.Free;
          asml.Remove(p);
          p.Free;

          p := hp2;

          result := true;
        end}
    end;

  procedure TCpuThumb2AsmOptimizer.PeepHoleOptPass2;
    var
      p,hp1,hp2: tai;
      l : longint;
      condition : tasmcond;
      { UsedRegs, TmpUsedRegs: TRegSet; }

    begin
      p := BlockStart;
      { UsedRegs := []; }
      while (p <> BlockEnd) Do
        begin
          { UpdateUsedRegs(UsedRegs, tai(p.next)); }
          case p.Typ Of
            Ait_Instruction:
              begin
                case taicpu(p).opcode Of
                  A_B:
                    if taicpu(p).condition<>C_None then
                      begin
                         { check for
                                Bxx   xxx
                                <several instructions>
                             xxx:
                         }
                         l:=0;
                         GetNextInstruction(p, hp1);
                         while assigned(hp1) and
                           (l<=4) and
                           CanBeCond(hp1) and
                           { stop on labels }
                           not(hp1.typ=ait_label) do
                           begin
                              inc(l);
                              if MustBeLast(hp1) then
                                begin
                                  //hp1:=nil;
                                  GetNextInstruction(hp1,hp1);
                                  break;
                                end
                              else
                                GetNextInstruction(hp1,hp1);
                           end;
                         if assigned(hp1) then
                           begin
                              if FindLabel(tasmlabel(taicpu(p).oper[0]^.ref^.symbol),hp1) then
                                begin
                                  if (l<=4) and (l>0) then
                                    begin
                                      condition:=inverse_cond(taicpu(p).condition);
                                      hp2:=p;
                                      GetNextInstruction(p,hp1);
                                      p:=hp1;
                                      repeat
                                        if hp1.typ=ait_instruction then
                                          taicpu(hp1).condition:=condition;
                                        if MustBeLast(hp1) then
                                          begin
                                            GetNextInstruction(hp1,hp1);
                                            break;
                                          end
                                        else
                                          GetNextInstruction(hp1,hp1);
                                      until not(assigned(hp1)) or
                                        not(CanBeCond(hp1)) or
                                        (hp1.typ=ait_label);
                                      { wait with removing else GetNextInstruction could
                                        ignore the label if it was the only usage in the
                                        jump moved away }

                                      asml.InsertAfter(tai_comment.create(strpnew('Collapsed')), hp2);

                                      DecrementPreceedingIT(asml, hp2);

                                      case l of
                                        1: asml.InsertAfter(taicpu.op_cond(A_IT,condition), hp2);
                                        2: asml.InsertAfter(taicpu.op_cond(A_ITT,condition), hp2);
                                        3: asml.InsertAfter(taicpu.op_cond(A_ITTT,condition), hp2);
                                        4: asml.InsertAfter(taicpu.op_cond(A_ITTTT,condition), hp2);
                                      end;

                                      tasmlabel(taicpu(hp2).oper[0]^.ref^.symbol).decrefs;
                                      asml.remove(hp2);
                                      hp2.free;
                                      continue;
                                    end;
                                end;
                           end;
                      end;
                  else
                    ;
                end;
              end;
            else
              ;
          end;
          p := tai(p.next)
        end;
    end;


  function TCpuThumb2AsmOptimizer.PostPeepHoleOptsCpu(var p: tai): boolean;
    begin
      result:=false;

      if p.typ = ait_instruction then
        begin
          if MatchInstruction(p, A_MOV, [C_None], [PF_None]) and
            (taicpu(p).oper[1]^.typ=top_const) and
            (taicpu(p).oper[1]^.val >= 0) and
            (taicpu(p).oper[1]^.val < 256) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole Mov2Movs done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).oppostfix:=PF_S;
              result:=true;
            end
          else if MatchInstruction(p, A_MVN, [C_None], [PF_None]) and
            (taicpu(p).oper[1]^.typ=top_reg) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole Mvn2Mvns done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).oppostfix:=PF_S;
              result:=true;
            end
          else if MatchInstruction(p, A_RSB, [C_None], [PF_None]) and
            (taicpu(p).ops = 3) and
            (taicpu(p).oper[2]^.typ=top_const) and
            (taicpu(p).oper[2]^.val=0) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole Rsb2Rsbs done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).oppostfix:=PF_S;
              result:=true;
            end
          else if MatchInstruction(p, [A_ADD,A_SUB], [C_None], [PF_None]) and
            (taicpu(p).ops = 3) and
            MatchOperand(taicpu(p).oper[0]^, taicpu(p).oper[1]^) and
            (not MatchOperand(taicpu(p).oper[0]^, NR_STACK_POINTER_REG)) and
            (taicpu(p).oper[2]^.typ=top_const) and
            (taicpu(p).oper[2]^.val >= 0) and
            (taicpu(p).oper[2]^.val < 256) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole AddSub2*s done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).loadconst(1,taicpu(p).oper[2]^.val);
              taicpu(p).oppostfix:=PF_S;
              taicpu(p).ops := 2;
              result:=true;
            end
          else if MatchInstruction(p, [A_ADD,A_SUB], [C_None], [PF_None]) and
            (taicpu(p).ops = 2) and
            (taicpu(p).oper[1]^.typ=top_reg) and
            (not MatchOperand(taicpu(p).oper[0]^, NR_STACK_POINTER_REG)) and
            (not MatchOperand(taicpu(p).oper[1]^, NR_STACK_POINTER_REG)) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole AddSub2*s done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).oppostfix:=PF_S;
              result:=true;
            end
          else if MatchInstruction(p, [A_ADD], [C_None], [PF_None]) and
            (taicpu(p).ops = 3) and
            MatchOperand(taicpu(p).oper[0]^, taicpu(p).oper[1]^) and
            (taicpu(p).oper[2]^.typ=top_reg) then
            begin
              DebugMsg('Peephole AddRRR2AddRR done', p);
              taicpu(p).ops := 2;
              taicpu(p).loadreg(1,taicpu(p).oper[2]^.reg);
              result:=true;
            end
          else if MatchInstruction(p, [A_AND,A_ORR,A_EOR,A_BIC,A_LSL,A_LSR,A_ASR,A_ROR], [C_None], [PF_None]) and
            (taicpu(p).ops = 3) and
            MatchOperand(taicpu(p).oper[0]^, taicpu(p).oper[1]^) and
            (taicpu(p).oper[2]^.typ=top_reg) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole opXXY2opsXY done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).ops := 2;
              taicpu(p).loadreg(1,taicpu(p).oper[2]^.reg);
              taicpu(p).oppostfix:=PF_S;
              result:=true;
            end
          else if MatchInstruction(p, [A_AND,A_ORR,A_EOR,A_BIC,A_LSL,A_LSR,A_ASR,A_ROR], [C_None], [PF_S]) and
            (taicpu(p).ops = 3) and
            MatchOperand(taicpu(p).oper[0]^, taicpu(p).oper[1]^) and
            (taicpu(p).oper[2]^.typ in [top_reg,top_const]) then
            begin
              DebugMsg('Peephole opXXY2opXY done', p);
              taicpu(p).ops := 2;
              if taicpu(p).oper[2]^.typ=top_reg then
                taicpu(p).loadreg(1,taicpu(p).oper[2]^.reg)
              else
                taicpu(p).loadconst(1,taicpu(p).oper[2]^.val);
              result:=true;
            end
          else if MatchInstruction(p, [A_AND,A_ORR,A_EOR], [C_None], [PF_None,PF_S]) and
            (taicpu(p).ops = 3) and
            MatchOperand(taicpu(p).oper[0]^, taicpu(p).oper[2]^) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole opXYX2opsXY done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).oppostfix:=PF_S;
              taicpu(p).ops := 2;
              result:=true;
            end
          else if MatchInstruction(p, [A_MOV], [C_None], [PF_None]) and
            (taicpu(p).ops=3) and
            (taicpu(p).oper[2]^.typ=top_shifterop) and
            (taicpu(p).oper[2]^.shifterop^.shiftmode in [SM_LSL,SM_LSR,SM_ASR,SM_ROR]) and
            //MatchOperand(taicpu(p).oper[0]^, taicpu(p).oper[1]^) and
            (not RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
            begin
              DebugMsg('Peephole Mov2Shift done', p);
              asml.InsertBefore(tai_regalloc.alloc(NR_DEFAULTFLAGS,p), p);
              asml.InsertAfter(tai_regalloc.dealloc(NR_DEFAULTFLAGS,p), p);
              IncludeRegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs);
              taicpu(p).oppostfix:=PF_S;

              case taicpu(p).oper[2]^.shifterop^.shiftmode of
                SM_LSL: taicpu(p).opcode:=A_LSL;
                SM_LSR: taicpu(p).opcode:=A_LSR;
                SM_ASR: taicpu(p).opcode:=A_ASR;
                SM_ROR: taicpu(p).opcode:=A_ROR;
                else
                  internalerror(2019050912);
              end;

              if taicpu(p).oper[2]^.shifterop^.rs<>NR_NO then
                taicpu(p).loadreg(2, taicpu(p).oper[2]^.shifterop^.rs)
              else
                taicpu(p).loadconst(2, taicpu(p).oper[2]^.shifterop^.shiftimm);
              result:=true;
            end
        end;
    end;


begin
  casmoptimizer:=TCpuAsmOptimizer;
  cpreregallocscheduler:=TCpuPreRegallocScheduler;
End.
