{
    Copyright (c) 1998-2004 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit contains the processor independent assembler optimizer
    object, base for the dataflow analyzer, peepholeoptimizer and
    common subexpression elimination objects.

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
Unit AoptObj;

  {$i fpcdefs.inc}

  { general, processor independent objects for use by the assembler optimizer }

  Interface

    uses
      globtype,
      aasmbase,aasmcpu,aasmtai,aasmdata,
      cclasses,
      cgbase,cgutils,
      cpubase,
      aoptbase,aoptcpub,aoptda;

    { ************************************************************************* }
    { ********************************* Constants ***************************** }
    { ************************************************************************* }

    Const

    {Possible register content types}
      con_Unknown = 0;
      con_ref = 1;
      con_const = 2;

    {***************** Types ****************}

    Type

    { ************************************************************************* }
    { ************************* Some general type definitions ***************** }
    { ************************************************************************* }
      TRefCompare = Function(const r1, r2: TReference): Boolean;
      //!!! FIXME
      TRegArray = Array[byte] of tsuperregister;


      TRegSet = tcpuregisterset;
      { possible actions on an operand: read, write or modify (= read & write) }
      TOpAction = (OpAct_Read, OpAct_Write, OpAct_Modify, OpAct_Unknown);

    { ************************************************************************* }
    { * Object to hold information on which regiters are in use and which not * }
    { ************************************************************************* }

      { TUsedRegs }

      TUsedRegs = class
        Constructor create(aTyp : TRegisterType);
        Constructor create_regset(aTyp : TRegisterType;Const _RegSet: TRegSet);

        Destructor Destroy;override;

        Procedure Clear;
        { update the info with the pairegalloc objects coming after
          p                                                         }
        procedure Update(p: Tai; IgnoreNewAllocs: Boolean=false);
        { is Reg currently in use }
        Function IsUsed(Reg: TRegister): Boolean;
        { get all the currently used registers }
        Function GetUsedRegs: TRegSet;

        { outputs  the current set }
        Procedure Dump(var t : text);
      Private
        Typ : TRegisterType;
        UsedRegs: TRegSet;
      End;

    { ************************************************************************* }
    { ******************* Contents of the integer registers ******************* }
    { ************************************************************************* }

     { size of the integer that holds the state number of a register. Can be any }
     { integer type, so it can be changed to reduce the size of the TContent     }
     { structure or to improve alignment                                         }
      TStateInt = Byte;

      TContent = Record
        { start and end of block instructions that defines the }
        { content of this register. If Typ = con_const, then   }
        { Longint(StartMod) = value of the constant)           }
        StartMod: Tai;
        { starts at 0, gets increased everytime the register is }
        { written to                                            }
        WState: TStateInt;
        { starts at 0, gets increased everytime the register is read }
        { from                                                       }
        RState: TStateInt;
        { how many instructions starting with StarMod does the block }
        { consist of                                                 }
        NrOfMods: Byte;
        { the type of the content of the register: unknown, memory   }
        { (variable) or constant                                     }
        Typ: Byte;
      End;

      //!!! FIXME
      TRegContent = Array[byte] Of TContent;

    { ************************************************************************** }
    { information object with the contents of every register. Every Tai object   }
    { gets one of these assigned: a pointer to it is stored in the OptInfo field }
    { ************************************************************************** }

      { TPaiProp }

      TPaiProp = class(TAoptBaseCpu)
        Regs: TRegContent;
        { can this instruction be removed? }
        CanBeRemoved: Boolean;

        Constructor create; reintroduce;

        { checks the whole sequence of which (so regs[which].StartMod and and  }
        { the next NrOfMods Tai objects) to see whether Reg is used somewhere, }
        { without it being loaded with something else first                    }
        Function RegInSequence(Reg, which: TRegister): Boolean;
        { destroy the contents of a register, as well as those whose contents }
        { are based on those of that register                                 }
        Procedure DestroyReg(Reg: TRegister; var InstrSinceLastMod:
          TInstrSinceLastMod);
        { if the contents of WhichReg (can be R_NO in case of a constant) are  }
        { written to memory at the location Ref, the contents of the registers }
        { that depend on Ref have to be  destroyed                             }
        Procedure DestroyRefs(Const Ref: TReference; WhichReg: TRegister; var
          InstrSinceLastMod: TInstrSinceLastMod);

        { an instruction reads from operand o }
        Procedure ReadOp(const o:toper);
        { an instruction reads from reference Ref }
        Procedure ReadRef(Ref: PReference);
        { an instruction reads from register Reg }
        Procedure ReadReg(Reg: TRegister);

        { an instruction writes/modifies operand o and this has special     }
        { side-effects or modifies the contents in such a way that we can't }
        { simply add this instruction to the sequence of instructions that  }
        { describe the contents of the operand, so destroy it               }
        Procedure DestroyOp(const o:Toper; var InstrSinceLastMod:
          TInstrSinceLastMod);
        { destroy the contents of all registers }
        Procedure DestroyAllRegs(var InstrSinceLastMod: TInstrSinceLastMod);
        { a register's contents are modified, but not destroyed (the new value
          depends on the old one)                                              }
        Procedure ModifyReg(reg: TRegister; var InstrSinceLastMod:
          TInstrSinceLastMod);
        { an operand's contents are modified, but not destroyed (the new value
          depends on the old one)                                              }
        Procedure ModifyOp(const oper: TOper; var InstrSinceLastMod:
          TInstrSinceLastMod);

        { increase the write state of a register (call every time a register is
          written to)                                                           }
        Procedure IncWState(Reg: TRegister);
        { increase the read state of a register (call every time a register is }
        { read from)                                                           }
        Procedure IncRState(Reg: TRegister);
        { get the write state of a register }
        Function GetWState(Reg: TRegister): TStateInt;
        { get the read state of a register }
        Function GetRState(Reg: TRegister): TStateInt;

        { get the type of contents of a register }
        Function GetRegContentType(Reg: TRegister): Byte;

        Destructor Done;

        Private

        Procedure IncState(var s: TStateInt);

        { returns whether the reference Ref is used somewhere in the loading }
        { sequence Content                                                   }
        Function RefInSequence(Const Ref: TReference; Content: TContent;
          RefsEq: TRefCompare): Boolean;

        { returns whether the instruction P reads from and/or writes }
        { to Reg                                                     }
        Function RefInInstruction(Const Ref: TReference; p: Tai;
          RefsEq: TRefCompare): Boolean;

        { returns whether two references with at least one pointing to an array }
        { may point to the same memory location                                 }

      End;


    { ************************************************************************* }
    { ************************ Label information ****************************** }
    { ************************************************************************* }
      TLabelTableItem = Record
        PaiObj: Tai;
      End;

      TLabelTable = Array[0..2500000] Of TLabelTableItem;
      PLabelTable = ^TLabelTable;
      PLabelInfo = ^TLabelInfo;
      TLabelInfo = Record
        { the highest and lowest label number occurring in the current code }
        { fragment                                                          }
        LowLabel, HighLabel: longint;
        LabelDif: cardinal;
        { table that contains the addresses of the Pai_Label objects associated
          with each label number                                                }
        LabelTable: PLabelTable;
      End;

    { ************************************************************************* }
    { ********** General optimizer object, used to derive others from ********* }
    { ************************************************************************* }

      TAllUsedRegs = array[TRegisterType] of TUsedRegs;
      { TAOptObj }

      TAOptObj = class(TAoptBaseCpu)
        { the PAasmOutput list this optimizer instance works on }
        AsmL: TAsmList;

        { The labelinfo record contains the addresses of the Tai objects }
        { that are labels, how many labels there are and the min and max }
        { label numbers                                                  }
        LabelInfo: PLabelInfo;

        { Start and end of the block that is currently being optimized }
        BlockStart, BlockEnd: Tai;

        DFA: TAOptDFA;

        UsedRegs: TAllUsedRegs;

        { _AsmL is the PAasmOutpout list that has to be optimized,     }
        { _BlockStart and _BlockEnd the start and the end of the block }
        { that has to be optimized and _LabelInfo a pointer to a       }
        { TLabelInfo record                                            }
        Constructor create(_AsmL: TAsmList; _BlockStart, _BlockEnd: Tai;
                           _LabelInfo: PLabelInfo); virtual; reintroduce;
        Destructor Destroy;override;

        { processor independent methods }

        Procedure CreateUsedRegs(var regs: TAllUsedRegs);
        Procedure ClearUsedRegs;
        Procedure UpdateUsedRegs(p : Tai);
        class procedure UpdateUsedRegs(var Regs: TAllUsedRegs; p: Tai);
        Function CopyUsedRegs(var dest : TAllUsedRegs) : boolean;
        procedure RestoreUsedRegs(const Regs : TAllUsedRegs);
        procedure TransferUsedRegs(var dest: TAllUsedRegs);
        class Procedure ReleaseUsedRegs(const regs : TAllUsedRegs);
        class Function RegInUsedRegs(reg : TRegister;regs : TAllUsedRegs) : boolean;
        class Procedure IncludeRegInUsedRegs(reg : TRegister;var regs : TAllUsedRegs);
        class Procedure ExcludeRegFromUsedRegs(reg: TRegister;var regs : TAllUsedRegs);

        Function GetAllocationString(const regs : TAllUsedRegs) : string;

        { returns true if the label L is found between hp and the next }
        { instruction                                                  }
        Function FindLabel(L: TasmLabel; Var hp: Tai): Boolean;

        { inserts new_one between prev and foll in AsmL }
        Procedure InsertLLItem(prev, foll, new_one: TLinkedListItem);

        { If P is a Tai object releveant to the optimizer, P is returned
          If it is not relevant tot he optimizer, the first object after P
          that is relevant is returned                                     }
        Function SkipHead(P: Tai): Tai;

        { returns true if the operands o1 and o2 are completely equal }
        Function OpsEqual(const o1,o2:toper): Boolean;

        { Returns the next ait_alloc object with ratype ra_alloc for
          Reg is found in the block
          of Tai's starting with StartPai and ending with the next "real"
          instruction. If none is found, it returns
          nil
        }
        Function FindRegAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc;

        { Returns the last ait_alloc object with ratype ra_alloc for
          Reg is found in the block
          of Tai's starting with StartPai and ending with the next "real"
          instruction. If none is found, it returns
          nil
        }
        Function FindRegAllocBackward(Reg : TRegister; StartPai : Tai) : tai_regalloc;


        { Returns the next ait_alloc object with ratype ra_dealloc
          for Reg which is found in the block of Tai's starting with StartPai
          and ending with the next "real" instruction. If none is found, it returns
          nil                                                                        }
        Function FindRegDeAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc;

        { allocates register reg between (and including) instructions p1 and p2
          the type of p1 and p2 must not be in SkipInstr }
        procedure AllocRegBetween(reg : tregister; p1,p2 : tai; var initialusedregs : TAllUsedRegs);

        { reg used after p? }
        function RegUsedAfterInstruction(reg: Tregister; p: tai; var AllUsedRegs: TAllUsedRegs): Boolean;

        { returns true if reg reaches it's end of life at p, this means it is either
          reloaded with a new value or it is deallocated afterwards }
        function RegEndOfLife(reg: TRegister;p: taicpu): boolean;

        { removes p from asml, updates registers and replaces it by a valid value, if this is the case true is returned }
        function RemoveCurrentP(var p : tai): boolean;

       { traces sucessive jumps to their final destination and sets it, e.g.
         je l1                je l3
         <code>               <code>
         l1:       becomes    l1:
         je l2                je l3
         <code>               <code>
         l2:                  l2:
         jmp l3               jmp l3

         the level parameter denotes how deeep we have already followed the jump,
         to avoid endless loops with constructs such as "l5: ; jmp l5"           }
        function GetFinalDestination(hp: taicpu; level: longint): boolean;

        function getlabelwithsym(sym: tasmlabel): tai;

        { Removes an instruction following hp1 (possibly with reg.deallocations in between),
          if its opcode is A_NOP. }
        procedure RemoveDelaySlot(hp1: tai);

        { peephole optimizer }
        procedure PrePeepHoleOpts; virtual;
        procedure PeepHoleOptPass1; virtual;
        procedure PeepHoleOptPass2; virtual;
        procedure PostPeepHoleOpts; virtual;

        { processor dependent methods }
        // if it returns true, perform a "continue"
        function PrePeepHoleOptsCpu(var p: tai): boolean; virtual;
        function PeepHoleOptPass1Cpu(var p: tai): boolean; virtual;
        function PeepHoleOptPass2Cpu(var p: tai): boolean; virtual;
        function PostPeepHoleOptsCpu(var p: tai): boolean; virtual;

        { insert debug comments about which registers are read and written by
          each instruction. Useful for debugging the InstructionLoadsFromReg and
          other similar functions. }
        procedure Debug_InsertInstrRegisterDependencyInfo; virtual;
      End;

       Function ArrayRefsEq(const r1, r2: TReference): Boolean;

       { Returns a pointer to the operand that contains the destination label }
       function JumpTargetOp(ai: taicpu): poper;

       { Returns True if hp is any jump to a label }
       function IsJumpToLabel(hp: taicpu): boolean;

       { Returns True if hp is an unconditional jump to a label }
       function IsJumpToLabelUncond(hp: taicpu): boolean;

    { ***************************** Implementation **************************** }

  Implementation

    uses
      cutils,
      globals,
      verbose,
      aoptutils,
      aasmcfi,
      procinfo;


    function JumpTargetOp(ai: taicpu): poper; inline;
      begin
{$if defined(MIPS) or defined(riscv64) or defined(riscv32)}
        { MIPS or RiscV branches can have 1,2 or 3 operands, target label is the last one. }
        result:=ai.oper[ai.ops-1];
{$elseif defined(SPARC64)}
        if ai.ops=2 then
          result:=ai.oper[1]
        else
          result:=ai.oper[0];
{$else MIPS}
        result:=ai.oper[0];
{$endif}
      end;


      { ************************************************************************* }
      { ******************************** TUsedRegs ****************************** }
      { ************************************************************************* }

    Constructor TUsedRegs.create(aTyp : TRegisterType);
      Begin
        Typ:=aTyp;
        UsedRegs := [];
      End;


    Constructor TUsedRegs.create_regset(aTyp : TRegisterType;Const _RegSet: TRegSet);
      Begin
        Typ:=aTyp;
        UsedRegs := _RegSet;
      End;


    {
      updates UsedRegs with the RegAlloc Information coming after P
    }
    Procedure TUsedRegs.Update(p: Tai;IgnoreNewAllocs : Boolean = false);
      Begin
        { this code is normally not used because updating the register allocation information is done in
          TAOptObj.UpdateUsedRegs for speed reasons }
        repeat
          while assigned(p) and
                ((p.typ in (SkipInstr - [ait_RegAlloc])) or
                 (p.typ = ait_label) or
                 ((p.typ = ait_marker) and
                  (tai_Marker(p).Kind in [mark_AsmBlockEnd,mark_NoLineInfoStart,mark_NoLineInfoEnd]))) do
               p := tai(p.next);
          while assigned(p) and
                (p.typ=ait_RegAlloc) Do
            begin
              if (getregtype(tai_regalloc(p).reg) = typ) then
                begin
                  case tai_regalloc(p).ratype of
                    ra_alloc :
                      if not(IgnoreNewAllocs) then
                        Include(UsedRegs, getsupreg(tai_regalloc(p).reg));
                    ra_dealloc :
                      Exclude(UsedRegs, getsupreg(tai_regalloc(p).reg));
                    else
                      ;
                  end;
                end;
              p := tai(p.next);
            end;
        until not(assigned(p)) or
              (not(p.typ in SkipInstr) and
               not((p.typ = ait_label) and
                   labelCanBeSkipped(tai_label(p))));
      End;


    Function TUsedRegs.IsUsed(Reg: TRegister): Boolean;
      Begin
        IsUsed := (getregtype(Reg)=Typ) and (getsupreg(Reg) in UsedRegs);
      End;


    Function TUsedRegs.GetUsedRegs: TRegSet; inline;
      Begin
        GetUsedRegs := UsedRegs;
      End;


    procedure TUsedRegs.Dump(var t: text);
      var
        i: dword;
      begin
        write(t,Typ,' ');
        for i:=low(TRegSet) to high(TRegSet) do
          if i in UsedRegs then
            write(t,i,' ');
         writeln(t);
      end;


    Destructor TUsedRegs.Destroy;
      Begin
        inherited destroy;
      end;


    procedure TUsedRegs.Clear;
      begin
        UsedRegs := [];
      end;

      { ************************************************************************* }
      { **************************** TPaiProp *********************************** }
      { ************************************************************************* }

      Constructor TPaiProp.Create;
        Begin
        {!!!!!!
          UsedRegs.Init;
          CondRegs.init;
        }
        {  DirFlag: TFlagContents; I386 specific}
        End;


      Function TPaiProp.RegInSequence(Reg, which: TRegister): Boolean;
      {
      Var p: Tai;
          RegsChecked: TRegSet;
          content: TContent;
          Counter: Byte;
          TmpResult: Boolean;
      }
      begin
        Result:=False; { unimplemented }
      (*!!!!!!!!!!1
        RegsChecked := [];
        content := regs[which];
        p := content.StartMod;
        TmpResult := False;
        Counter := 1;
        While Not(TmpResult) And
              (Counter <= Content.NrOfMods) Do
          Begin
            If IsLoadMemReg(p) Then
              With PInstr(p)^.oper[LoadSrc]^.ref^ Do
                If (Base = ProcInfo.FramePointer)
      {$ifdef cpurefshaveindexreg}
                   And (Index = R_NO)
      {$endif cpurefshaveindexreg} Then
                  Begin
                    RegsChecked := RegsChecked +
                      [RegMaxSize(PInstr(p)^.oper[LoadDst]^.reg)];
                    If Reg = RegMaxSize(PInstr(p)^.oper[LoadDst]^.reg) Then
                      Break;
                  End
                Else
                  Begin
                    If (Base = Reg) And
                       Not(Base In RegsChecked)
                      Then TmpResult := True;
      {$ifdef cpurefshaveindexreg}
                    If Not(TmpResult) And
                       (Index = Reg) And
                         Not(Index In RegsChecked)
                      Then TmpResult := True;
      {$Endif cpurefshaveindexreg}
                  End
            Else TmpResult := RegInInstruction(Reg, p);
            Inc(Counter);
            GetNextInstruction(p,p)
          End;
        RegInSequence := TmpResult
      *)
      End;


      Procedure TPaiProp.DestroyReg(Reg: TRegister; var InstrSinceLastMod:
                  TInstrSinceLastMod);
      { Destroys the contents of the register Reg in the PPaiProp p1, as well as }
      { the contents of registers are loaded with a memory location based on Reg }
      {
      Var TmpWState, TmpRState: Byte;
          Counter: TRegister;
      }
      Begin
      {!!!!!!!
        Reg := RegMaxSize(Reg);
        If (Reg in [LoGPReg..HiGPReg]) Then
          For Counter := LoGPReg to HiGPReg Do
            With Regs[Counter] Do
              If (Counter = reg) Or
                 ((Typ = Con_Ref) And
                  RegInSequence(Reg, Counter)) Then
                Begin
                  InstrSinceLastMod[Counter] := 0;
                  IncWState(Counter);
                  TmpWState := GetWState(Counter);
                  TmpRState := GetRState(Counter);
                  FillChar(Regs[Counter], SizeOf(TContent), 0);
                  WState := TmpWState;
                  RState := TmpRState
                End
      }
      End;

      Function ArrayRefsEq(const r1, r2: TReference): Boolean;
      Begin
        Result:=False; { unimplemented }
      (*!!!!!!!!!!
        ArrayRefsEq := (R1.Offset+R1.OffsetFixup = R2.Offset+R2.OffsetFixup) And
      {$ifdef refsHaveSegmentReg}
                       (R1.Segment = R2.Segment) And
      {$endif}
                       (R1.Base = R2.Base) And
                       (R1.Symbol=R2.Symbol);
      *)
      End;

      Procedure TPaiProp.DestroyRefs(Const Ref: TReference; WhichReg: TRegister;
                  var InstrSinceLastMod: TInstrSinceLastMod);
      { destroys all registers which possibly contain a reference to Ref, WhichReg }
      { is the register whose contents are being written to memory (if this proc   }
      { is called because of a "mov?? %reg, (mem)" instruction)                    }
      {
      Var RefsEq: TRefCompare;
          Counter: TRegister;
      }
      Begin
      (*!!!!!!!!!!!
        WhichReg := RegMaxSize(WhichReg);
        If (Ref.base = procinfo.FramePointer) or
            Assigned(Ref.Symbol) Then
          Begin
            If
      {$ifdef cpurefshaveindexreg}
               (Ref.Index = R_NO) And
      {$endif cpurefshaveindexreg}
               (Not(Assigned(Ref.Symbol)) or
                (Ref.base = R_NO)) Then
        { local variable which is not an array }
              RefsEq := @RefsEqual
            Else
        { local variable which is an array }
              RefsEq := @ArrayRefsEq;
      {write something to a parameter, a local or global variable, so
         * with uncertain optimizations on:
            - destroy the contents of registers whose contents have somewhere a
              "mov?? (Ref), %reg". WhichReg (this is the register whose contents
              are being written to memory) is not destroyed if it's StartMod is
              of that form and NrOfMods = 1 (so if it holds ref, but is not a
              pointer or value based on Ref)
          * with uncertain optimizations off:
             - also destroy registers that contain any pointer}
            For Counter := LoGPReg to HiGPReg Do
              With Regs[Counter] Do
                Begin
                  If (typ = Con_Ref) And
                     ((Not(cs_opt_size in current_settings.optimizerswitches) And
                       (NrOfMods <> 1)
                      ) Or
                      (RefInSequence(Ref,Regs[Counter], RefsEq) And
                       ((Counter <> WhichReg) Or
                        ((NrOfMods <> 1) And
       {StarMod is always of the type ait_instruction}
                         (PInstr(StartMod)^.oper[0].typ = top_ref) And
                         RefsEq(PInstr(StartMod)^.oper[0].ref^, Ref)
                        )
                       )
                      )
                     )
                    Then
                      DestroyReg(Counter, InstrSinceLastMod)
                End
          End
        Else
      {write something to a pointer location, so
         * with uncertain optimzations on:
            - do not destroy registers which contain a local/global variable or a
              parameter, except if DestroyRefs is called because of a "movsl"
         * with uncertain optimzations off:
            - destroy every register which contains a memory location
            }
            For Counter := LoGPReg to HiGPReg Do
              With Regs[Counter] Do
                If (typ = Con_Ref) And
                   (Not(cs_opt_size in current_settings.optimizerswitches) Or
      {$ifdef x86}
              {for movsl}
                    (Ref.Base = R_EDI) Or
      {$endif}
              {don't destroy if reg contains a parameter, local or global variable}
                    Not((NrOfMods = 1) And
                        (PInstr(StartMod)^.oper[0].typ = top_ref) And
                        ((PInstr(StartMod)^.oper[0].ref^.base = ProcInfo.FramePointer) Or
                          Assigned(PInstr(StartMod)^.oper[0].ref^.Symbol)
                        )
                       )
                   )
                Then DestroyReg(Counter, InstrSinceLastMod)
      *)
      End;


      Procedure TPaiProp.DestroyAllRegs(var InstrSinceLastMod: TInstrSinceLastMod);
      {Var Counter: TRegister;}
      Begin {initializes/desrtoys all registers}
      (*!!!!!!!!!
        For Counter := LoGPReg To HiGPReg Do
          Begin
            ReadReg(Counter);
            DestroyReg(Counter, InstrSinceLastMod);
          End;
        CondRegs.Init;
      { FPURegs.Init; }
      *)
      End;

      Procedure TPaiProp.DestroyOp(const o:Toper; var InstrSinceLastMod:
                  TInstrSinceLastMod);
      Begin
      {!!!!!!!
        Case o.typ Of
          top_reg: DestroyReg(o.reg, InstrSinceLastMod);
          top_ref:
            Begin
              ReadRef(o.ref);
              DestroyRefs(o.ref^, R_NO, InstrSinceLastMod);
            End;
          top_symbol:;
        End;
      }
      End;

      Procedure TPaiProp.ReadReg(Reg: TRegister);
      Begin
      {!!!!!!!
        Reg := RegMaxSize(Reg);
        If Reg in General_Registers Then
          IncRState(RegMaxSize(Reg))
      }
      End;

      Procedure TPaiProp.ReadRef(Ref: PReference);
      Begin
      (*!!!!!!
        If Ref^.Base <> R_NO Then
          ReadReg(Ref^.Base);
      {$ifdef cpurefshaveindexreg}
        If Ref^.Index <> R_NO Then
          ReadReg(Ref^.Index);
      {$endif cpurefshaveindexreg}
      *)
      End;

      Procedure TPaiProp.ReadOp(const o:toper);
      Begin
        Case o.typ Of
          top_reg: ReadReg(o.reg);
          top_ref: ReadRef(o.ref);
        else
          internalerror(200410241);
        End;
      End;

      Procedure TPaiProp.ModifyReg(reg: TRegister; Var InstrSinceLastMod:
                                     TInstrSinceLastMod);
      Begin
      (*!!!!!!!
        With Regs[reg] Do
          If (Typ = Con_Ref)
            Then
              Begin
                IncState(WState);
       {also store how many instructions are part of the sequence in the first
        instructions PPaiProp, so it can be easily accessed from within
        CheckSequence}
                Inc(NrOfMods, InstrSinceLastMod[Reg]);
                PPaiProp(StartMod.OptInfo)^.Regs[Reg].NrOfMods := NrOfMods;
                InstrSinceLastMod[Reg] := 0;
              End
            Else
              DestroyReg(Reg, InstrSinceLastMod);
      *)
      End;

      Procedure TPaiProp.ModifyOp(const oper: TOper; var InstrSinceLastMod:
                  TInstrSinceLastMod);
      Begin
        If oper.typ = top_reg Then
          ModifyReg(RegMaxSize(oper.reg),InstrSinceLastMod)
        Else
          Begin
            ReadOp(oper);
            DestroyOp(oper, InstrSinceLastMod);
          End
      End;

      Procedure TPaiProp.IncWState(Reg: TRegister);{$ifdef inl} inline;{$endif inl}
      Begin
        //!!!! IncState(Regs[Reg].WState);
      End;

      Procedure TPaiProp.IncRState(Reg: TRegister);{$ifdef inl} inline;{$endif inl}
      Begin
        //!!!! IncState(Regs[Reg].RState);
      End;

      Function TPaiProp.GetWState(Reg: TRegister): TStateInt; {$ifdef inl} inline;{$endif inl}
      Begin
        Result:=0; { unimplemented }
        //!!!! GetWState := Regs[Reg].WState
      End;

      Function TPaiProp.GetRState(Reg: TRegister): TStateInt; {$ifdef inl} inline;{$endif inl}
      Begin
        Result:=0; { unimplemented }
        //!!!! GetRState := Regs[Reg].RState
      End;

      Function TPaiProp.GetRegContentType(Reg: TRegister): Byte; {$ifdef inl} inline;{$endif inl}
      Begin
        Result:=0; { unimplemented }
        //!!!! GetRegContentType := Regs[Reg].typ
      End;

      Destructor TPaiProp.Done;
      Begin
        //!!!! UsedRegs.Done;
        //!!!! CondRegs.Done;
      {  DirFlag: TFlagContents; I386 specific}
      End;
      { ************************ private TPaiProp stuff ************************* }

      Procedure TPaiProp.IncState(Var s: TStateInt); {$ifdef inl} inline;{$endif inl}
      Begin
        If s <> High(TStateInt) Then Inc(s)
        Else s := 0
      End;

      Function TPaiProp.RefInInstruction(Const Ref: TReference; p: Tai;
        RefsEq: TRefCompare): Boolean;
      Var Count: AWord;
          TmpResult: Boolean;
      Begin
        TmpResult := False;
        If (p.typ = ait_instruction) Then
          Begin
            Count := 0;
            Repeat
              If (TInstr(p).oper[Count]^.typ = Top_Ref) Then
                TmpResult := RefsEq(Ref, PInstr(p)^.oper[Count]^.ref^);
              Inc(Count);
            Until (Count = max_operands) or TmpResult;
          End;
        RefInInstruction := TmpResult;
      End;

      Function TPaiProp.RefInSequence(Const Ref: TReference; Content: TContent;
        RefsEq: TRefCompare): Boolean;
      Var p: Tai;
          Counter: Byte;
          TmpResult: Boolean;
      Begin
        p := Content.StartMod;
        TmpResult := False;
        Counter := 1;
        While Not(TmpResult) And
              (Counter <= Content.NrOfMods) Do
          Begin
            If (p.typ = ait_instruction) And
               RefInInstruction(Ref, p, @references_equal)
              Then TmpResult := True;
            Inc(Counter);
            GetNextInstruction(p,p)
          End;
        RefInSequence := TmpResult
      End;

      { ************************************************************************* }
      { ***************************** TAoptObj ********************************** }
      { ************************************************************************* }

      Constructor TAoptObj.create(_AsmL: TAsmList; _BlockStart, _BlockEnd: Tai;
                                  _LabelInfo: PLabelInfo);
      Begin
        AsmL := _AsmL;
        BlockStart := _BlockStart;
        BlockEnd := _BlockEnd;
        LabelInfo := _LabelInfo;
        CreateUsedRegs(UsedRegs);
      End;

      destructor TAOptObj.Destroy;
        var
          i : TRegisterType;
        begin
          for i:=low(TRegisterType) to high(TRegisterType) do
            UsedRegs[i].Destroy;
          inherited Destroy;
        end;


      procedure TAOptObj.CreateUsedRegs(var regs: TAllUsedRegs);
        var
          i : TRegisterType;
        begin
          for i:=low(TRegisterType) to high(TRegisterType) do
            Regs[i]:=TUsedRegs.Create(i);
        end;


      procedure TAOptObj.ClearUsedRegs;
        var
          i : TRegisterType;
        begin
          for i:=low(TRegisterType) to high(TRegisterType) do
            UsedRegs[i].Clear;
        end;


      procedure TAOptObj.UpdateUsedRegs(p : Tai);
        begin
          { this code is based on TUsedRegs.Update to avoid multiple passes through the asmlist,
            the code is duplicated here }
          repeat
            while assigned(p) and
                  ((p.typ in (SkipInstr - [ait_RegAlloc])) or
                   ((p.typ = ait_label) and
                    labelCanBeSkipped(tai_label(p))) or
                   ((p.typ = ait_marker) and
                    (tai_Marker(p).Kind in [mark_AsmBlockEnd,mark_NoLineInfoStart,mark_NoLineInfoEnd]))) do
                 p := tai(p.next);
            while assigned(p) and
                  (p.typ=ait_RegAlloc) Do
              begin
                case tai_regalloc(p).ratype of
                  ra_alloc :
                    Include(UsedRegs[getregtype(tai_regalloc(p).reg)].UsedRegs, getsupreg(tai_regalloc(p).reg));
                  ra_dealloc :
                    Exclude(UsedRegs[getregtype(tai_regalloc(p).reg)].UsedRegs, getsupreg(tai_regalloc(p).reg));
                  else
                    ;
                end;
                p := tai(p.next);
              end;
          until not(assigned(p)) or
                (not(p.typ in SkipInstr) and
                 not((p.typ = ait_label) and
                     labelCanBeSkipped(tai_label(p))));
        end;


      class procedure TAOptObj.UpdateUsedRegs(var Regs : TAllUsedRegs;p : Tai);
        var
          i : TRegisterType;
        begin
          for i:=low(TRegisterType) to high(TRegisterType) do
            Regs[i].Update(p);
        end;


      function TAOptObj.CopyUsedRegs(var dest: TAllUsedRegs): boolean;
      var
        i : TRegisterType;
      begin
        Result:=true;
        for i:=low(TRegisterType) to high(TRegisterType) do
          dest[i]:=TUsedRegs.Create_Regset(i,UsedRegs[i].GetUsedRegs);
      end;


      procedure TAOptObj.RestoreUsedRegs(const Regs: TAllUsedRegs);
      var
        i : TRegisterType;
      begin
        { Note that the constructor Create_Regset is being called as a regular
          method - it is not instantiating a new object.  This is because it is
          the only published means to modify the internal state en-masse. [Kit] }
        for i:=low(TRegisterType) to high(TRegisterType) do
          UsedRegs[i].Create_Regset(i,Regs[i].GetUsedRegs);
      end;


      procedure TAOptObj.TransferUsedRegs(var dest: TAllUsedRegs);
      var
        i : TRegisterType;
      begin
        { Note that the constructor Create_Regset is being called as a regular
          method - it is not instantiating a new object.  This is because it is
          the only published means to modify the internal state en-masse. [Kit] }
        for i:=low(TRegisterType) to high(TRegisterType) do
          dest[i].Create_Regset(i, UsedRegs[i].GetUsedRegs);
      end;


      class procedure TAOptObj.ReleaseUsedRegs(const regs: TAllUsedRegs);
        var
          i : TRegisterType;
      begin
        for i:=low(TRegisterType) to high(TRegisterType) do
          regs[i].Free;
      end;


      class Function TAOptObj.RegInUsedRegs(reg : TRegister;regs : TAllUsedRegs) : boolean;
      begin
        result:=regs[getregtype(reg)].IsUsed(reg);
      end;


      class procedure TAOptObj.IncludeRegInUsedRegs(reg: TRegister;
       var regs: TAllUsedRegs);
      begin
        include(regs[getregtype(reg)].UsedRegs,getsupreg(Reg));
      end;


      class procedure TAOptObj.ExcludeRegFromUsedRegs(reg: TRegister;
       var regs: TAllUsedRegs);
      begin
        exclude(regs[getregtype(reg)].UsedRegs,getsupreg(Reg));
      end;


      function TAOptObj.GetAllocationString(const regs: TAllUsedRegs): string;
      var
        i : TRegisterType;
        j : TSuperRegister;
      begin
        Result:='';
        for i:=low(TRegisterType) to high(TRegisterType) do
          for j in regs[i].UsedRegs do
            Result:=Result+std_regname(newreg(i,j,R_SUBWHOLE))+' ';
      end;


      Function TAOptObj.FindLabel(L: TasmLabel; Var hp: Tai): Boolean;
      Var TempP: Tai;
      Begin
        TempP := hp;
        While Assigned(TempP) and
             (TempP.typ In SkipInstr + [ait_label,ait_align]) Do
          If (TempP.typ <> ait_Label) Or
             (Tai_label(TempP).labsym <> L)
            Then GetNextInstruction(TempP, TempP)
            Else
              Begin
                hp := TempP;
                FindLabel := True;
                exit
              End;
        FindLabel := False;
      End;

      Procedure TAOptObj.InsertLLItem(prev, foll, new_one : TLinkedListItem);
      Begin
        If Assigned(prev) Then
          If Assigned(foll) Then
            Begin
              If Assigned(new_one) Then
                Begin
                  new_one.previous := prev;
                  new_one.next := foll;
                  prev.next := new_one;
                  foll.previous := new_one;
                  { should we update line information? }
                  if (not (tai(new_one).typ in SkipLineInfo)) and
                     (not (tai(foll).typ in SkipLineInfo)) then
                    Tailineinfo(new_one).fileinfo := Tailineinfo(foll).fileinfo
                End
            End
          Else AsmL.Concat(new_one)
        Else If Assigned(Foll) Then AsmL.Insert(new_one)
      End;


      Function TAOptObj.SkipHead(P: Tai): Tai;
      Var OldP: Tai;
      Begin
        Repeat
          OldP := P;
          If (P.typ in SkipInstr) Or
             ((P.typ = ait_marker) And
              (Tai_Marker(P).Kind = mark_AsmBlockEnd)) Then
            GetNextInstruction(P, P)
          Else If ((P.Typ = Ait_Marker) And
              (Tai_Marker(P).Kind = mark_NoPropInfoStart)) Then
       { a marker of the type mark_NoPropInfoStart can't be the first instruction of a }
       { paasmoutput list                                                         }
            GetNextInstruction(Tai(P.Previous),P);
          If (P.Typ = Ait_Marker) And
             (Tai_Marker(P).Kind = mark_AsmBlockStart) Then
            Begin
              P := Tai(P.Next);
              While (P.typ <> Ait_Marker) Or
                    (Tai_Marker(P).Kind <> mark_AsmBlockEnd) Do
                P := Tai(P.Next)
            End;
          Until P = OldP;
        SkipHead := P;
      End;

      Function TAOptObj.OpsEqual(const o1,o2:toper): Boolean;
      Begin
        if o1.typ=o2.typ then
          Case o1.typ Of
            Top_Reg :
              OpsEqual:=o1.reg=o2.reg;
            Top_Ref :
              OpsEqual:=
                references_equal(o1.ref^, o2.ref^) and
                (o1.ref^.volatility=[]) and
                (o2.ref^.volatility=[]);
            Top_Const :
              OpsEqual:=o1.val=o2.val;
            Top_None :
              OpsEqual := True
            else OpsEqual := False
          End
        else
          OpsEqual := False;
      End;


      Function TAOptObj.FindRegAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc;
      Begin
        Result:=nil;
        Repeat
          While Assigned(StartPai) And
                ((StartPai.typ in (SkipInstr - [ait_regAlloc])) Or
{$ifdef cpudelayslot}
                ((startpai.typ=ait_instruction) and (taicpu(startpai).opcode=A_NOP)) or
{$endif cpudelayslot}
                 ((StartPai.typ = ait_label) and
                  Not(Tai_Label(StartPai).labsym.Is_Used))) Do
            StartPai := Tai(StartPai.Next);
          If Assigned(StartPai) And
             (StartPai.typ = ait_regAlloc) Then
            Begin
              if (tai_regalloc(StartPai).ratype=ra_alloc) and
                (getregtype(tai_regalloc(StartPai).Reg) = getregtype(Reg)) and
                (getsupreg(tai_regalloc(StartPai).Reg) = getsupreg(Reg)) then
               begin
                 Result:=tai_regalloc(StartPai);
                 exit;
               end;
              StartPai := Tai(StartPai.Next);
            End
          else
            exit;
        Until false;
      End;


      Function TAOptObj.FindRegAllocBackward(Reg: TRegister; StartPai: Tai): tai_regalloc;
      Begin
        Result:=nil;
        Repeat
          While Assigned(StartPai) And
                ((StartPai.typ in (SkipInstr - [ait_regAlloc])) Or
                 ((StartPai.typ = ait_label) and
                  Not(Tai_Label(StartPai).labsym.Is_Used))) Do
            StartPai := Tai(StartPai.Previous);
          If Assigned(StartPai) And
             (StartPai.typ = ait_regAlloc) Then
            Begin
              if (tai_regalloc(StartPai).ratype=ra_alloc) and
                SuperRegistersEqual(tai_regalloc(StartPai).Reg,Reg) then
               begin
                 Result:=tai_regalloc(StartPai);
                 exit;
               end;
              StartPai := Tai(StartPai.Previous);
            End
          else
            exit;
        Until false;
      End;


      function TAOptObj.FindRegDeAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc;
      Begin
         Result:=nil;
         Repeat
           While Assigned(StartPai) And
                 ((StartPai.typ in (SkipInstr - [ait_regAlloc])) Or
                  ((StartPai.typ = ait_label) and
                   Not(Tai_Label(StartPai).labsym.Is_Used))) Do
             StartPai := Tai(StartPai.Next);
           If Assigned(StartPai) And
              (StartPai.typ = ait_regAlloc) Then
             Begin
               if (tai_regalloc(StartPai).ratype=ra_dealloc) and
                 (getregtype(tai_regalloc(StartPai).Reg) = getregtype(Reg)) and
                 (getsupreg(tai_regalloc(StartPai).Reg) = getsupreg(Reg)) then
                begin
                  Result:=tai_regalloc(StartPai);
                  exit;
                end;
               StartPai := Tai(StartPai.Next);
             End
           else
             exit;
         Until false;
       End;


    { allocates register reg between (and including) instructions p1 and p2
      the type of p1 and p2 must not be in SkipInstr }
    procedure TAOptObj.AllocRegBetween(reg: tregister; p1, p2: tai; var initialusedregs: TAllUsedRegs);
      var
        hp, start: tai;
        removedsomething,
        firstRemovedWasAlloc,
        lastRemovedWasDealloc: boolean;
      begin
{$ifdef EXTDEBUG}
{        if assigned(p1.optinfo) and
           (ptaiprop(p1.optinfo)^.usedregs <> initialusedregs) then
         internalerror(2004101010); }
{$endif EXTDEBUG}
        start := p1;
       if (reg = NR_STACK_POINTER_REG) or
          (reg = current_procinfo.framepointer) or
           not(assigned(p1)) then
          { this happens with registers which are loaded implicitely, outside the }
          { current block (e.g. esi with self)                                    }
          exit;
        { make sure we allocate it for this instruction }
        getnextinstruction(p2,p2);
        lastRemovedWasDealloc := false;
        removedSomething := false;
        firstRemovedWasAlloc := false;
{$ifdef allocregdebug}
        hp := tai_comment.Create(strpnew('allocating '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+
          ' from here...'));
        insertllitem(asml,p1.previous,p1,hp);
        hp := tai_comment.Create(strpnew('allocated '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+
          ' till here...'));
        insertllitem(asml,p2,p2.next,hp);
{$endif allocregdebug}
        { do it the safe way: always allocate the full super register,
          as we do no register re-allocation in the peephole optimizer,
          this does not hurt
        }
        case getregtype(reg) of
          R_MMREGISTER:
            reg:=newreg(R_MMREGISTER,getsupreg(reg),R_SUBMMWHOLE);
          R_INTREGISTER:
            reg:=newreg(R_INTREGISTER,getsupreg(reg),R_SUBWHOLE);
          R_FPUREGISTER:
            reg:=newreg(R_FPUREGISTER,getsupreg(reg),R_SUBWHOLE);
          R_ADDRESSREGISTER:
            reg:=newreg(R_ADDRESSREGISTER,getsupreg(reg),R_SUBWHOLE);
          else
            Internalerror(2018030701);
        end;
        if not(RegInUsedRegs(reg,initialusedregs)) then
          begin
            hp := tai_regalloc.alloc(reg,nil);
            insertllItem(p1.previous,p1,hp);
            IncludeRegInUsedRegs(reg,initialusedregs);
          end;
        while assigned(p1) and
              (p1 <> p2) do
          begin
            if assigned(p1.optinfo) then
              internalerror(2014022301); // IncludeRegInUsedRegs(reg,ptaiprop(p1.optinfo)^.usedregs);
            p1 := tai(p1.next);
            repeat
              while assigned(p1) and
                    (p1.typ in (SkipInstr-[ait_regalloc])) Do
                p1 := tai(p1.next);

              { remove all allocation/deallocation info about the register in between }
              if assigned(p1) and
                 (p1.typ = ait_regalloc) then
                begin
                  { same super register, different sub register? }
                  if SuperRegistersEqual(reg,tai_regalloc(p1).reg) and (tai_regalloc(p1).reg<>reg) then
                    begin
                      if (getsubreg(tai_regalloc(p1).reg)>getsubreg(reg)) or (getsubreg(reg)=R_SUBH) then
                        internalerror(2016101501);
                      tai_regalloc(p1).reg:=reg;
                    end;

                  if tai_regalloc(p1).reg=reg then
                    begin
                      if not removedSomething then
                        begin
                          firstRemovedWasAlloc := tai_regalloc(p1).ratype=ra_alloc;
                          removedSomething := true;
                        end;
                      lastRemovedWasDealloc := (tai_regalloc(p1).ratype=ra_dealloc);
                      hp := tai(p1.Next);
                      asml.Remove(p1);
                      p1.free;
                      p1 := hp;
                    end
                  else
                    p1 := tai(p1.next);
                end;
            until not(assigned(p1)) or
                  not(p1.typ in SkipInstr);
          end;
        if assigned(p1) then
          begin
            if firstRemovedWasAlloc then
              begin
                hp := tai_regalloc.Alloc(reg,nil);
                insertLLItem(start.previous,start,hp);
              end;
            if lastRemovedWasDealloc then
              begin
                hp := tai_regalloc.DeAlloc(reg,nil);
                insertLLItem(p1.previous,p1,hp);
              end;
          end;
      end;


    function TAOptObj.RegUsedAfterInstruction(reg: Tregister; p: tai;var AllUsedRegs: TAllUsedRegs): Boolean;
      begin
        AllUsedRegs[getregtype(reg)].Update(tai(p.Next),true);
        RegUsedAfterInstruction :=
          AllUsedRegs[getregtype(reg)].IsUsed(reg) and
          not(regLoadedWithNewValue(reg,p)) and
          (
            not(GetNextInstruction(p,p)) or
            InstructionLoadsFromReg(reg,p) or
            not(regLoadedWithNewValue(reg,p))
          );
      end;


    function TAOptObj.RegEndOfLife(reg : TRegister;p : taicpu) : boolean;
      begin
         Result:=assigned(FindRegDealloc(reg,tai(p.Next))) or
           RegLoadedWithNewValue(reg,p);
      end;


    function TAOptObj.RemoveCurrentP(var p : tai) : boolean;
      var
        hp1 : tai;
      begin
        result:=GetNextInstruction(p,hp1);
        { p will be removed, update used register as we continue
          with the next instruction after p }
        UpdateUsedRegs(tai(p.Next));
        AsmL.Remove(p);
        p.Free;
        p:=hp1;
      end;


    function FindAnyLabel(hp: tai; var l: tasmlabel): Boolean;
      begin
        FindAnyLabel := false;
        while assigned(hp.next) and
              (tai(hp.next).typ in (SkipInstr+[ait_align])) Do
          hp := tai(hp.next);
        if assigned(hp.next) and
           (tai(hp.next).typ = ait_label) then
          begin
            FindAnyLabel := true;
            l := tai_label(hp.next).labsym;
          end
      end;


{$push}
{$r-}
    function TAOptObj.getlabelwithsym(sym: tasmlabel): tai;
      begin
        if (int64(sym.labelnr) >= int64(labelinfo^.lowlabel)) and
           (int64(sym.labelnr) <= int64(labelinfo^.highlabel)) then   { range check, a jump can go past an assembler block! }
          getlabelwithsym := labelinfo^.labeltable^[sym.labelnr-labelinfo^.lowlabel].paiobj
        else
          getlabelwithsym := nil;
      end;
{$pop}


    { Returns True if hp is an unconditional jump to a label }
    function IsJumpToLabelUncond(hp: taicpu): boolean;
      begin
{$if defined(avr)}
        result:=(hp.opcode in aopt_uncondjmp) and
{$else avr}
        result:=(hp.opcode=aopt_uncondjmp) and
{$endif avr}
{$if defined(arm) or defined(aarch64)}
          (hp.condition=c_None) and
{$endif arm or aarch64}
{$if defined(riscv32) or defined(riscv64)}          
          (hp.ops>0) and
          (hp.oper[0]^.reg=NR_X0) and
{$else riscv}
          (hp.ops>0) and
{$endif riscv}
          (JumpTargetOp(hp)^.typ = top_ref) and
          (JumpTargetOp(hp)^.ref^.symbol is TAsmLabel);
      end;


    { Returns True if hp is any jump to a label }
    function IsJumpToLabel(hp: taicpu): boolean;
      begin
        result:=hp.is_jmp and
          (hp.ops>0) and
          (JumpTargetOp(hp)^.typ = top_ref) and
          (JumpTargetOp(hp)^.ref^.symbol is TAsmLabel);
      end;


    procedure TAOptObj.RemoveDelaySlot(hp1:tai);
      var
        hp2: tai;
      begin
        hp2:=tai(hp1.next);
        while assigned(hp2) and (hp2.typ in SkipInstr) do
          hp2:=tai(hp2.next);
        if assigned(hp2) and (hp2.typ=ait_instruction) and
          (taicpu(hp2).opcode=A_NOP) then
          begin
            asml.remove(hp2);
            hp2.free;
          end;
        { Anything except A_NOP must be left in place: these instructions
          execute before branch, so code stays correct if branch is removed. }
      end;


    function TAOptObj.GetFinalDestination(hp: taicpu; level: longint): boolean;
      {traces sucessive jumps to their final destination and sets it, e.g.
       je l1                je l3
       <code>               <code>
       l1:       becomes    l1:
       je l2                je l3
       <code>               <code>
       l2:                  l2:
       jmp l3               jmp l3

       the level parameter denotes how deeep we have already followed the jump,
       to avoid endless loops with constructs such as "l5: ; jmp l5"           }

      var p1: tai;
          {$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
          p2: tai;
          l: tasmlabel;
          {$endif}

      begin
        GetfinalDestination := false;
        if level > 20 then
          exit;
        p1 := getlabelwithsym(tasmlabel(JumpTargetOp(hp)^.ref^.symbol));
        if assigned(p1) then
          begin
            SkipLabels(p1,p1);
            if (tai(p1).typ = ait_instruction) and
               (taicpu(p1).is_jmp) then
              if { the next instruction after the label where the jump hp arrives}
                 { is unconditional or of the same type as hp, so continue       }
                 IsJumpToLabelUncond(taicpu(p1))
{$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
{ for MIPS, it isn't enough to check the condition; first operands must be same, too. }
                 or
                 conditions_equal(taicpu(p1).condition,hp.condition) or

                 { the next instruction after the label where the jump hp arrives
                   is the opposite of hp (so this one is never taken), but after
                   that one there is a branch that will be taken, so perform a
                   little hack: set p1 equal to this instruction (that's what the
                   last SkipLabels is for, only works with short bool evaluation)}
                 (conditions_equal(taicpu(p1).condition,inverse_cond(hp.condition)) and
                  SkipLabels(p1,p2) and
                  (p2.typ = ait_instruction) and
                  (taicpu(p2).is_jmp) and
                   (IsJumpToLabelUncond(taicpu(p2)) or
                   (conditions_equal(taicpu(p2).condition,hp.condition))) and
                  SkipLabels(p1,p1))
{$endif not MIPS and not RV64 and not RV32 and not JVM}
                 then
                begin
                  { quick check for loops of the form "l5: ; jmp l5 }
                  if (tasmlabel(JumpTargetOp(taicpu(p1))^.ref^.symbol).labelnr =
                       tasmlabel(JumpTargetOp(hp)^.ref^.symbol).labelnr) then
                    exit;
                  if not GetFinalDestination(taicpu(p1),succ(level)) then
                    exit;
{$if defined(aarch64)}
                  { can't have conditional branches to
                    global labels on AArch64, because the
                    offset may become too big }
                  if not(taicpu(hp).condition in [C_None,C_AL,C_NV]) and
                     (tasmlabel(JumpTargetOp(taicpu(p1))^.ref^.symbol).bind<>AB_LOCAL) then
                    exit;
{$endif aarch64}
                  tasmlabel(JumpTargetOp(hp)^.ref^.symbol).decrefs;
                  JumpTargetOp(hp)^.ref^.symbol:=JumpTargetOp(taicpu(p1))^.ref^.symbol;
                  tasmlabel(JumpTargetOp(hp)^.ref^.symbol).increfs;
                end
{$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
              else
                if conditions_equal(taicpu(p1).condition,inverse_cond(hp.condition)) then
                  if not FindAnyLabel(p1,l) then
                    begin
      {$ifdef finaldestdebug}
                      insertllitem(asml,p1,p1.next,tai_comment.Create(
                        strpnew('previous label inserted'))));
      {$endif finaldestdebug}
                      current_asmdata.getjumplabel(l);
                      insertllitem(p1,p1.next,tai_label.Create(l));
                      tasmlabel(JumpTargetOp(hp)^.ref^.symbol).decrefs;
                      JumpTargetOp(hp)^.ref^.symbol := l;
                      l.increfs;
      {               this won't work, since the new label isn't in the labeltable }
      {               so it will fail the rangecheck. Labeltable should become a   }
      {               hashtable to support this:                                   }
      {               GetFinalDestination(asml, hp);                               }
                    end
                  else
                    begin
      {$ifdef finaldestdebug}
                      insertllitem(asml,p1,p1.next,tai_comment.Create(
                        strpnew('next label reused'))));
      {$endif finaldestdebug}
                      l.increfs;
                      tasmlabel(JumpTargetOp(hp)^.ref^.symbol).decrefs;
                      JumpTargetOp(hp)^.ref^.symbol := l;
                      if not GetFinalDestination(hp,succ(level)) then
                        exit;
                    end;
{$endif not MIPS and not RV64 and not RV32 and not JVM}
          end;
        GetFinalDestination := true;
      end;


    procedure TAOptObj.PrePeepHoleOpts;
      var
        p: tai;
      begin
        p := BlockStart;
        ClearUsedRegs;
        while (p <> BlockEnd) Do
          begin
            UpdateUsedRegs(tai(p.next));
            if PrePeepHoleOptsCpu(p) then
              continue;
            if assigned(p) then
              begin
                UpdateUsedRegs(p);
                p:=tai(p.next);
              end;
          end;
      end;


    procedure TAOptObj.PeepHoleOptPass1;
      var
        p,hp1,hp2,hp3 : tai;
        stoploop:boolean;
      begin
        repeat
          stoploop:=true;
          p := BlockStart;
          ClearUsedRegs;
          while (p <> BlockEnd) Do
            begin
              { I'am not sure why this is done, UsedRegs should reflect the register usage before the instruction
                If an instruction needs the information of this, it can easily create a TempUsedRegs (FK)
              UpdateUsedRegs(tai(p.next));
              }
  {$ifdef DEBUG_OPTALLOC}
              if p.Typ=ait_instruction then
                InsertLLItem(tai(p.Previous),p,tai_comment.create(strpnew(GetAllocationString(UsedRegs))));
  {$endif DEBUG_OPTALLOC}
              if PeepHoleOptPass1Cpu(p) then
                begin
                  stoploop:=false;
                  UpdateUsedRegs(p);
                  continue;
                end;
              case p.Typ Of
                ait_instruction:
                  begin
                    { Handle Jmp Optimizations }
                    if taicpu(p).is_jmp then
                      begin
                        { the following if-block removes all code between a jmp and the next label,
                          because it can never be executed
                        }
                        if IsJumpToLabelUncond(taicpu(p)) then
                          begin
                            hp2:=p;
                            while GetNextInstruction(hp2, hp1) and
                                  (hp1.typ <> ait_label)
{$ifdef JVM}
                                  and (hp1.typ <> ait_jcatch)
{$endif}
                                  do
                              if not(hp1.typ in ([ait_label]+skipinstr)) then
                                begin
                                  if (hp1.typ = ait_instruction) and
                                     taicpu(hp1).is_jmp and
                                     (JumpTargetOp(taicpu(hp1))^.typ = top_ref) and
                                     (JumpTargetOp(taicpu(hp1))^.ref^.symbol is TAsmLabel) then
                                     TAsmLabel(JumpTargetOp(taicpu(hp1))^.ref^.symbol).decrefs;
                                  { don't kill start/end of assembler block,
                                    no-line-info-start/end, cfi end, etc }
                                  if not(hp1.typ in [ait_align,ait_marker]) and
                                     ((hp1.typ<>ait_cfi) or
                                      (tai_cfi_base(hp1).cfityp<>cfi_endproc)) then
                                    begin
{$ifdef cpudelayslot}
                                      if (hp1.typ=ait_instruction) and (taicpu(hp1).is_jmp) then
                                        RemoveDelaySlot(hp1);
{$endif cpudelayslot}
                                      asml.remove(hp1);
                                      hp1.free;
                                      stoploop:=false;
                                    end
                                  else
                                    hp2:=hp1;
                                end
                              else break;
                            end;
                        if GetNextInstruction(p, hp1) then
                          begin
                            SkipEntryExitMarker(hp1,hp1);
                            { remove unconditional jumps to a label coming right after them }
                            if IsJumpToLabelUncond(taicpu(p)) and
                              FindLabel(tasmlabel(JumpTargetOp(taicpu(p))^.ref^.symbol), hp1) and
          { TODO: FIXME removing the first instruction fails}
                                (p<>blockstart) then
                              begin
                                tasmlabel(JumpTargetOp(taicpu(p))^.ref^.symbol).decrefs;
{$ifdef cpudelayslot}
                                RemoveDelaySlot(p);
{$endif cpudelayslot}
                                hp2:=tai(hp1.next);
                                asml.remove(p);
                                p.free;
                                p:=hp2;
                                stoploop:=false;
                                continue;
                              end
                            else if assigned(hp1) then
                              begin
                                { change the following jumps:
                                    jmp<cond> lab_1         jmp<cond_inverted> lab_2
                                    jmp       lab_2  >>>    <code>
                                  lab_1:                  lab_2:
                                    <code>
                                  lab_2:
                                }
                                hp3:=hp1;
                                if hp1.typ = ait_label then
                                  SkipLabels(hp1,hp1);
                                if (tai(hp1).typ=ait_instruction) and
                                  IsJumpToLabelUncond(taicpu(hp1)) and
                                  GetNextInstruction(hp1, hp2) and
                                  IsJumpToLabel(taicpu(p)) and
                                  FindLabel(tasmlabel(JumpTargetOp(taicpu(p))^.ref^.symbol), hp2) and
                                  { prevent removal of infinite loop from
                                        jmp<cond> lab_1
                                      lab_2:
                                        jmp lab2
                                      ..
                                      lab_1:
                                  }
                                  not FindLabel(tasmlabel(JumpTargetOp(taicpu(hp1))^.ref^.symbol), hp3) then
                                  begin
                                    if (taicpu(p).opcode=aopt_condjmp)
{$if defined(arm) or defined(aarch64)}
                                      and (taicpu(p).condition<>C_None)
{$endif arm or aarch64}
{$if defined(aarch64)}
                                      { can't have conditional branches to
                                        global labels on AArch64, because the
                                        offset may become too big }
                                      and (tasmlabel(JumpTargetOp(taicpu(hp1))^.ref^.symbol).bind=AB_LOCAL)
{$endif aarch64}
                                    then
                                      begin
                                        taicpu(p).condition:=inverse_cond(taicpu(p).condition);
                                        tai_label(hp2).labsym.decrefs;
                                        JumpTargetOp(taicpu(p))^.ref^.symbol:=JumpTargetOp(taicpu(hp1))^.ref^.symbol;
                                        { when freeing hp1, the reference count
                                          isn't decreased, so don't increase

                                         taicpu(p).oper[0]^.ref^.symbol.increfs;
                                        }
{$ifdef cpudelayslot}
                                        RemoveDelaySlot(hp1);
{$endif cpudelayslot}
                                        asml.remove(hp1);
                                        hp1.free;
                                        stoploop:=false;
                                        GetFinalDestination(taicpu(p),0);
                                      end
                                    else
                                      begin
                                        GetFinalDestination(taicpu(p),0);
                                        p:=tai(p.next);
                                        continue;
                                      end;
                                  end
                                else if IsJumpToLabel(taicpu(p)) then
                                  GetFinalDestination(taicpu(p),0);
                              end;
                          end;
                      end
                    else
                    { All other optimizes }
                      begin
                      end; { if is_jmp }
                  end;
                else
                  ;
              end;
              if assigned(p) then
                begin
                  UpdateUsedRegs(p);
                  p:=tai(p.next);
                end;
            end;
        until stoploop or not(cs_opt_level3 in current_settings.optimizerswitches);
      end;


    procedure TAOptObj.PeepHoleOptPass2;
      var
        p: tai;
      begin
        p := BlockStart;
        ClearUsedRegs;
        while (p <> BlockEnd) Do
          begin
            if PeepHoleOptPass2Cpu(p) then
              continue;
            if assigned(p) then
              begin
                UpdateUsedRegs(p);
                p:=tai(p.next);
              end;
          end;
      end;


    procedure TAOptObj.PostPeepHoleOpts;
      var
        p: tai;
      begin
        p := BlockStart;
        ClearUsedRegs;
        while (p <> BlockEnd) Do
          begin
            UpdateUsedRegs(tai(p.next));
            if PostPeepHoleOptsCpu(p) then
              continue;
            if assigned(p) then
              begin
                UpdateUsedRegs(p);
                p:=tai(p.next);
              end;
          end;
      end;


    function TAOptObj.PrePeepHoleOptsCpu(var p : tai) : boolean;
      begin
        result := false;
      end;


    function TAOptObj.PeepHoleOptPass1Cpu(var p: tai): boolean;
      begin
        result := false;
      end;


    function TAOptObj.PeepHoleOptPass2Cpu(var p : tai) : boolean;
      begin
        result := false;
      end;


    function TAOptObj.PostPeepHoleOptsCpu(var p: tai): boolean;
      begin
        result := false;
      end;


    procedure TAOptObj.Debug_InsertInstrRegisterDependencyInfo;
      var
        p: tai;
        ri: tregisterindex;
        reg: TRegister;
        commentstr: AnsiString;
        registers_found: Boolean;
      begin
        p:=tai(AsmL.First);
        while (p<>AsmL.Last) Do
          begin
            if p.typ=ait_instruction then
              begin
{$ifdef x86}
                taicpu(p).SetOperandOrder(op_att);
{$endif x86}
                commentstr:='Instruction reads';
                registers_found:=false;
                for ri in tregisterindex do
                  begin
                    reg:=regnumber_table[ri];
                    if (reg<>NR_NO) and InstructionLoadsFromReg(reg,p) then
                      begin
                        commentstr:=commentstr+' '+std_regname(reg);
                        registers_found:=true;
                      end;
                  end;
                if not registers_found then
                  commentstr:=commentstr+' no registers';
                commentstr:=commentstr+' and writes new values in';
                registers_found:=false;
                for ri in tregisterindex do
                  begin
                    reg:=regnumber_table[ri];
                    if (reg<>NR_NO) and RegLoadedWithNewValue(reg,p) then
                      begin
                        commentstr:=commentstr+' '+std_regname(reg);
                        registers_found:=true;
                      end;
                  end;
                if not registers_found then
                  commentstr:=commentstr+' no registers';
                AsmL.InsertAfter(tai_comment.Create(strpnew(commentstr)),p);
              end;
            p:=tai(p.next);
          end;
      end;

End.
