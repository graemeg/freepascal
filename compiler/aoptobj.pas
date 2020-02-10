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

{ $define DEBUG_AOPTOBJ}
{ $define DEBUG_JUMP}

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
        class function RefInSequence(Const Ref: TReference; Content: TContent;
          RefsEq: TRefCompare): Boolean; static;

        { returns whether the instruction P reads from and/or writes }
        { to Reg                                                     }
        class function RefInInstruction(Const Ref: TReference; p: Tai;
          RefsEq: TRefCompare): Boolean; static;

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

        { Start and end of the block that is currently being optimized, and
          a selected start point after the start of the block }
        BlockStart, BlockEnd, StartPoint: Tai;

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
        class procedure UpdateUsedRegs(var Regs: TAllUsedRegs; p: Tai); static;

        { If UpdateUsedRegsAndOptimize has read ahead, the result is one before
          the next valid entry (so "p.Next" returns what's expected).  If no
          reading ahead happened, then the result is equal to p. }
        function UpdateUsedRegsAndOptimize(p : Tai): Tai;

        Function CopyUsedRegs(var dest : TAllUsedRegs) : boolean;
        procedure RestoreUsedRegs(const Regs : TAllUsedRegs);
        procedure TransferUsedRegs(var dest: TAllUsedRegs);
        class procedure ReleaseUsedRegs(const regs : TAllUsedRegs); static;
        class function RegInUsedRegs(reg : TRegister;regs : TAllUsedRegs) : boolean; static;
        class procedure IncludeRegInUsedRegs(reg : TRegister;var regs : TAllUsedRegs); static;
        class procedure ExcludeRegFromUsedRegs(reg: TRegister;var regs : TAllUsedRegs); static;

        class function GetAllocationString(const regs : TAllUsedRegs) : string; static;

        { returns true if the label L is found between hp and the next }
        { instruction                                                  }
        class function FindLabel(L: TasmLabel; Var hp: Tai): Boolean; static;

        { inserts new_one between prev and foll in AsmL }
        Procedure InsertLLItem(prev, foll, new_one: TLinkedListItem);

        { If P is a Tai object releveant to the optimizer, P is returned
          If it is not relevant tot he optimizer, the first object after P
          that is relevant is returned                                     }
        class function SkipHead(P: Tai): Tai; static;

        { returns true if the operands o1 and o2 are completely equal }
        class function OpsEqual(const o1,o2:toper): Boolean; static;

        { Returns the next ait_alloc object with ratype ra_alloc for
          Reg is found in the block
          of Tai's starting with StartPai and ending with the next "real"
          instruction. If none is found, it returns
          nil
        }
        class function FindRegAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc; static;

        { Returns the last ait_alloc object with ratype ra_alloc for
          Reg is found in the block
          of Tai's starting with StartPai and ending with the next "real"
          instruction. If none is found, it returns
          nil
        }
        class function FindRegAllocBackward(Reg : TRegister; StartPai : Tai) : tai_regalloc; static;


        { Returns the next ait_alloc object with ratype ra_dealloc
          for Reg which is found in the block of Tai's starting with StartPai
          and ending with the next "real" instruction. If none is found, it returns
          nil                                                                        }
        class function FindRegDeAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc; static;

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

        { Output debug message to console - null function if EXTDEBUG is not defined }
        class procedure DebugWrite(Message: string); static; inline;

        { Converts a conditional jump into an unconditional jump.  Only call this
          procedure on an instruction that you already know is a conditional jump }
        procedure MakeUnconditional(p: taicpu); virtual;

        { Removes all instructions between an unconditional jump and the next label.
          Returns True if a jump in between was removed (as it may open up new
          optimisations if the label appeared earlier in the stream) }
        function RemoveDeadCodeAfterJump(p: tai): Boolean;

        { If hp is a label, strip it if its reference count is zero.  Repeat until
          a non-label is found, or a label with a non-zero reference count.
          True is returned if something was stripped }
        function StripDeadLabels(hp: tai; var NextValid: tai): Boolean;

        { Strips a label and any aligns that appear before it (if hp points to
          them rather than the label).  Only call this procedure on a label that
          you already know is no longer referenced }
        procedure StripLabelFast(hp: tai); {$ifdef USEINLINE}inline;{$endif USEINLINE}

        { Checks and removes "jmp @@lbl; @lbl". Returns True if the jump was removed }
        function CollapseZeroDistJump(var p: tai; ThisLabel: TAsmLabel): Boolean;

        { If a group of labels are clustered, change the jump to point to the last one that is still referenced }
        function CollapseLabelCluster(jump: tai; var lbltai: tai): TAsmLabel;
{$ifndef JVM}
        function OptimizeConditionalJump(CJLabel: TAsmLabel; var p: tai; hp1: tai; var stoploop: Boolean): Boolean;
{$endif JVM}

        { Function to determine if the jump optimisations can be performed }
        function CanDoJumpOpts: Boolean; virtual;

        { Jump/label optimisation entry method }
        function DoJumpOptimizations(var p: tai; var stoploop: Boolean): Boolean;

        { insert debug comments about which registers are read and written by
          each instruction. Useful for debugging the InstructionLoadsFromReg and
          other similar functions. }
        procedure Debug_InsertInstrRegisterDependencyInfo; virtual;
      private
        procedure DebugMsg(const s: string; p: tai);
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
{$if defined(ARM)}
      cpuinfo,
{$endif defined(ARM)}
      procinfo;


{$ifdef DEBUG_AOPTOBJ}
    const
      SPeepholeOptimization: shortstring = 'Peephole Optimization: ';
{$else DEBUG_AOPTOBJ}
    { Empty strings help the optimizer to remove string concatenations that won't
      ever appear to the user on release builds. [Kit] }
    const
      SPeepholeOptimization = '';
{$endif DEBUG_AOPTOBJ}


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

      class Function TPaiProp.RefInInstruction(Const Ref: TReference; p: Tai;
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

      class function TPaiProp.RefInSequence(Const Ref: TReference; Content: TContent;
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

{$ifdef DEBUG_AOPTOBJ}
      procedure TAOptObj.DebugMsg(const s: string;p : tai);
        begin
          asml.insertbefore(tai_comment.Create(strpnew(s)), p);
        end;
{$else DEBUG_AOPTOBJ}
      procedure TAOptObj.DebugMsg(const s: string;p : tai);inline;
        begin
        end;
{$endif DEBUG_AOPTOBJ}

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


      { If UpdateUsedRegsAndOptimize has read ahead, the result is one before
        the next valid entry (so "p.Next" returns what's expected).  If no
        reading ahead happened, then the result is equal to p. }
      function TAOptObj.UpdateUsedRegsAndOptimize(p : Tai): Tai;
        var
          NotFirst: Boolean;
        begin
          { this code is based on TUsedRegs.Update to avoid multiple passes through the asmlist,
            the code is duplicated here }

          Result := p;
          if (p.typ in [ait_instruction, ait_label]) then
            begin
              if (p.next <> BlockEnd) and (tai(p.next).typ <> ait_instruction) then
                begin
                  { Advance one, otherwise the routine exits immediately and wastes time }
                  p := tai(p.Next);
                  NotFirst := True;
                end
              else
                { If the next entry is an instruction, nothing will be updated or
                  optimised here, so exit now to save time }
                Exit;
            end
          else
            NotFirst := False;

          repeat
            while assigned(p) and
                  ((p.typ in (SkipInstr + [ait_align, ait_label] - [ait_RegAlloc])) or
                   ((p.typ = ait_marker) and
                    (tai_Marker(p).Kind in [mark_AsmBlockEnd,mark_NoLineInfoStart,mark_NoLineInfoEnd]))) do
                 begin
                   prefetch(pointer(p.Next)^);
                   { Here's the optimise part }
                   if (p.typ in [ait_align, ait_label]) then
                     begin
                       if StripDeadLabels(p, p) then
                         begin
                           { Note, if the first instruction is stripped and is
                             the only one that gets removed, Result will now
                             contain a dangling pointer, so compensate for this. }
                           if not NotFirst then
                             Result := tai(p.Previous);

                           Continue;
                         end;

                       if ((p.typ = ait_label) and not labelCanBeSkipped(tai_label(p))) then
                         Break;
                     end;

                   Result := p;
                   p := tai(p.next);
                 end;
            while assigned(p) and
                  (p.typ=ait_RegAlloc) Do
              begin
                prefetch(pointer(p.Next)^);
                case tai_regalloc(p).ratype of
                  ra_alloc :
                    Include(UsedRegs[getregtype(tai_regalloc(p).reg)].UsedRegs, getsupreg(tai_regalloc(p).reg));
                  ra_dealloc :
                    Exclude(UsedRegs[getregtype(tai_regalloc(p).reg)].UsedRegs, getsupreg(tai_regalloc(p).reg));
                  else
                    { Do nothing };
                end;
                Result := p;
                p := tai(p.next);
              end;
            NotFirst := True;
          until not(assigned(p)) or
                (not(p.typ in SkipInstr + [ait_align]) and
                 not((p.typ = ait_label) and
                     labelCanBeSkipped(tai_label(p))));
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
                prefetch(pointer(p.Next)^);
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


      class function TAOptObj.GetAllocationString(const regs: TAllUsedRegs): string;
      var
        i : TRegisterType;
        j : TSuperRegister;
      begin
        Result:='';
        for i:=low(TRegisterType) to high(TRegisterType) do
          for j in regs[i].UsedRegs do
            Result:=Result+std_regname(newreg(i,j,R_SUBWHOLE))+' ';
      end;


      class function TAOptObj.FindLabel(L: TasmLabel; Var hp: Tai): Boolean;
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


      class function TAOptObj.SkipHead(P: Tai): Tai;
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

      class function TAOptObj.OpsEqual(const o1,o2:toper): Boolean;
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


      class function TAOptObj.FindRegAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc;
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


      class function TAOptObj.FindRegAllocBackward(Reg: TRegister; StartPai: Tai): tai_regalloc;
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


      class function TAOptObj.FindRegDeAlloc(Reg: TRegister; StartPai: Tai): tai_regalloc;
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


    function FindLiveLabel(hp: tai; var l: tasmlabel): Boolean;
      var
        next: tai;
      begin
        FindLiveLabel := false;

        while True do
          begin
            while assigned(hp.next) and
                  (tai(hp.next).typ in (SkipInstr+[ait_align])) Do
              hp := tai(hp.next);

            next := tai(hp.next);
            if assigned(next) and
              (tai(next).typ = ait_label) then
              begin
                l := tai_label(next).labsym;
                if not l.is_used then
                  begin
                    { Unsafe label }
                    hp := next;
                    Continue;
                  end;

                FindLiveLabel := true;
              end;
            Exit;
          end;
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
          (hp.ops>0) and
{$if defined(riscv32) or defined(riscv64)}
          (hp.oper[0]^.reg=NR_X0) and
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


    { Output debug message to console - null function if EXTDEBUG is not defined }
    class procedure TAOptObj.DebugWrite(Message: string); inline;
      begin
{$ifdef DEBUG_JUMP}
        WriteLn(Message);
{$else DEBUG_JUMP}
        { Do nothing }
{$endif DEBUG_JUMP}
      end;


    { Converts a conditional jump into an unconditional jump.  Only call this
      procedure on an instruction that you already know is a conditional jump }
    procedure TAOptObj.MakeUnconditional(p: taicpu);
      begin
        { TODO: If anyone can improve this particular optimisation to work on
          AVR, please do (it's currently not called at all). [Kit] }
{$if not defined(avr)}
{$if defined(powerpc) or defined(powerpc64)}
        p.condition.cond := C_None;
        p.condition.simple := True;
{$else powerpc}
        p.condition := C_None;
{$endif powerpc}
        p.opcode := aopt_uncondjmp;
{$ifdef RISCV}
        p.loadoper(1, p.oper[p.ops-1]^);
        p.loadreg(0, NR_X0);
        p.ops:=2;
{$endif}
{$endif not avr}
      end;


    { Removes all instructions between an unconditional jump and the next label.
      Returns True if a jump in between was removed (as it may open up new
      optimisations if the label appeared earlier in the stream) }
    function TAOptObj.RemoveDeadCodeAfterJump(p: tai): Boolean;
      const
{$ifdef JVM}
        TaiFence = SkipInstr + [ait_const, ait_realconst, ait_typedconst, ait_label, ait_jcatch];
{$else JVM}
        { Stop if it reaches SEH directive information in the form of
          consts, which may occur if RemoveDeadCodeAfterJump is called on
          the final RET instruction on x86, for example }
        TaiFence = SkipInstr + [ait_const, ait_realconst, ait_typedconst, ait_label];
{$endif JVM}
      var
        hp1, hp2: tai;
      begin
        { the following code removes all code between a jmp and the next label,
          because it can never be executed
        }
        Result := False;

        while GetNextInstruction(p, hp1) and
              (hp1 <> BlockEnd) and
              not (hp1.typ in TaiFence) do
            begin
              if (hp1.typ = ait_instruction) and
                 taicpu(hp1).is_jmp and
                 (JumpTargetOp(taicpu(hp1))^.typ = top_ref) and
                 (JumpTargetOp(taicpu(hp1))^.ref^.symbol is TAsmLabel) then
                begin
                  { If the destination label appears earlier, it may permit
                    further optimisations, so signal this in the Result }
                  Result := True;
                  TAsmLabel(JumpTargetOp(taicpu(hp1))^.ref^.symbol).decrefs;
                end;
              { don't kill start/end of assembler block,
                no-line-info-start/end etc }
              if (hp1.typ<>ait_marker) and
                 ((hp1.typ<>ait_cfi) or
                  (tai_cfi_base(hp1).cfityp<>cfi_endproc)) then
                begin
{$ifdef cpudelayslot}
                  if (hp1.typ=ait_instruction) and (taicpu(hp1).is_jmp) then
                    RemoveDelaySlot(hp1);
{$endif cpudelayslot}
                  if (hp1.typ = ait_align) then
                    begin
                      { Only remove the align if a label doesn't immediately follow }
                      if GetNextInstruction(hp1, hp2) and (hp2.typ = ait_label) then
                        { The label is unskippable }
                        Exit;
                    end;
                  asml.remove(hp1);
                  hp1.free;
                end
              else
                p:=hp1;
            end;
      end;

    { If hp is a label, strip it if its reference count is zero.  Repeat until
      a non-label is found, or a label with a non-zero reference count.
      True is returned if something was stripped }
    function TAOptObj.StripDeadLabels(hp: tai; var NextValid: tai): Boolean;
      var
        tmp, tmpNext: tai;
        hp1: tai;
        CurrentAlign: tai;
      begin
        CurrentAlign := nil;
        Result := False;
        hp1 := hp;
        NextValid := hp;

        { Stop if hp is an instruction, for example }
        while (hp1 <> BlockEnd) and (hp1.typ in [ait_label,ait_align]) do
          begin
            prefetch(pointer(hp1.Next)^);
            case hp1.typ of
              ait_label:
                begin
                  with tai_label(hp1).labsym do
                    if is_used or (bind <> AB_LOCAL) or (labeltype <> alt_jump) then
                      begin
                        { Valid label }
                        if Result then
                          NextValid := hp1;

                        DebugWrite('JUMP DEBUG: Last label in cluster:' + tostr(labelnr));

                        Exit;
                      end;

                  DebugWrite('JUMP DEBUG: Removed label ' + tostr(TAsmLabel(tai_label(hp1).labsym).labelnr));

                  { Set tmp to the next valid entry }
                  tmp := tai(hp1.Next);
                  { Remove label }
                  AsmL.Remove(hp1);
                  hp1.Free;

                  hp1 := tmp;

                  Result := True;
                  Continue;
                end;
              { Also remove the align if it comes before an unused label }
              ait_align:
                begin
                  tmp := tai(hp1.Next);
                  if tmp = BlockEnd then
                    { End of block }
                    Exit;

                  repeat

                    case tmp.typ of
                      ait_align: { Merge the aligns if permissible }
                        begin
                          { Check the maxbytes field though, since this may result in the
                            alignment being ignored }
                          if ((tai_align_abstract(hp1).maxbytes = 0) and (tai_align_abstract(tmp).maxbytes = 0)) or
                            { If a maxbytes field is present, only merge if the aligns have the same granularity }
                            ((tai_align_abstract(hp1).aligntype = tai_align_abstract(tmp).aligntype)) then
                            begin
                              with tai_align_abstract(hp1) do
                                begin
                                  aligntype := max(aligntype, tai_align_abstract(tmp).aligntype);
                                  maxbytes := max(maxbytes, tai_align_abstract(tmp).maxbytes);
                                  fillsize := max(fillsize, tai_align_abstract(tmp).fillsize);
                                  use_op := use_op or tai_align_abstract(tmp).use_op;

                                  if use_op and (tai_align_abstract(tmp).fillop <> 0) then
                                    fillop := tai_align_abstract(tmp).fillop;
                                end;

                              tmpNext := tai(tmp.Next);
                              AsmL.Remove(tmp);
                              tmp.Free;
                              Result := True;
                              tmp := tmpNext;
                            end
                          else
                            tmp := tai(tmp.Next);

                          Continue;
                        end;
                      ait_label:
                        begin
                          { Signal that we can possibly delete this align entry }
                          CurrentAlign := hp1;

                          repeat
                            with tai_label(tmp).labsym do
                              if is_used or (bind <> AB_LOCAL) or (labeltype <> alt_jump) then
                                begin
                                  { Valid label }
                                  if Result then
                                    NextValid := tmp;

                                  DebugWrite('JUMP DEBUG: Last label in cluster:' + tostr(labelnr));

                                  Exit;
                                end;

                            DebugWrite('JUMP DEBUG: Removed label ' + tostr(TAsmLabel(tai_label(tmp).labsym).labelnr));

                            { Remove label }
                            tmpNext := tai(tmp.Next);
                            AsmL.Remove(tmp);
                            tmp.Free;
                            Result := True;
                            tmp := tmpNext;

                            { Loop here for a minor performance gain }
                          until (tmp = BlockEnd) or (tmp.typ <> ait_label);

                          { Re-evaluate the align and see what follows }
                          Continue;
                        end
                      else
                        begin
                          { Set hp1 to the instruction after the align, because the
                            align might get deleted later and hence set NextValid
                            to a dangling pointer. [Kit] }
                          hp1 := tmp;
                          Break;
                        end;
                    end;
                  until (tmp = BlockEnd);

                  { Break out of the outer loop if the above Break is called }
                  if (hp1 = tmp) then
                    Break;
                end
              else
                Break;
            end;
            hp1 := tai(hp1.Next);
          end;

        { hp1 will be the next valid entry }
        NextValid := hp1;

        { Remove the alignment field (but only if the next valid entry is not a live label) }
        while Assigned(CurrentAlign) and (CurrentAlign.typ = ait_align) do
          begin
            DebugWrite('JUMP DEBUG: Alignment field removed');

            tmp := tai(CurrentAlign.next);

            AsmL.Remove(CurrentAlign);
            CurrentAlign.Free;

            CurrentAlign := tmp;
          end;
      end;


    { Strips a label and any aligns that appear before it (if hp points to
      them rather than the label).  Only call this procedure on a label that
      you already know is no longer referenced }
    procedure TAOptObj.StripLabelFast(hp: tai); {$ifdef USEINLINE}inline;{$endif USEINLINE}
      var
        tmp: tai;
      begin
        repeat
          case hp.typ of
            ait_align:
              begin
                tmp := tai(hp.Next);
                asml.Remove(hp);
                hp.Free;
                hp := tmp;
                { Control flow will now return to 'repeat' }
              end;
            ait_label:
              begin
{$ifdef EXTDEBUG}
                { When not in debug mode, deleting a live label will cause an
                  access violation later on. [Kit] }
                if tai_label(hp).labsym.getrefs <> 0 then
                  InternalError(2019110802);
{$endif EXTDEBUG}
                asml.Remove(hp);
                hp.Free;
                Exit;
              end;
            else
              begin
                { Might be a comment or temporary allocation entry }
                if not (hp.typ in SkipInstr) then
                  InternalError(2019110801);

                hp := tai(hp.Next);
              end;
          end;
        until False;
      end;

    { If a group of labels are clustered, change the jump to point to the last one
      that is still referenced }
    function TAOptObj.CollapseLabelCluster(jump: tai; var lbltai: tai): TAsmLabel;
      var
        LastLabel: TAsmLabel;
        hp2: tai;
      begin
        Result := tai_label(lbltai).labsym;
        LastLabel := Result;
        hp2 := tai(lbltai.next);

        while (hp2 <> BlockEnd) and (hp2.typ in SkipInstr + [ait_align, ait_label]) do
          begin

            if (hp2.typ = ait_label) and
              (tai_label(hp2).labsym.is_used) and
              (tai_label(hp2).labsym.labeltype = alt_jump) then
              LastLabel := tai_label(hp2).labsym;

            hp2 := tai(hp2.next);
          end;

        if (Result <> LastLabel) then
          begin
            Result.decrefs;
            JumpTargetOp(taicpu(jump))^.ref^.symbol := LastLabel;
            LastLabel.increfs;
            Result := LastLabel;
            lbltai := hp2;
          end;
      end;

{$ifndef JVM}
    function TAOptObj.OptimizeConditionalJump(CJLabel: TAsmLabel; var p: tai; hp1: tai; var stoploop: Boolean): Boolean;
      var
        hp2: tai;
        NCJLabel: TAsmLabel;
      begin
        Result := False;
        while (hp1 <> BlockEnd) do
          begin
            StripDeadLabels(hp1, hp1);

            if (hp1 <> BlockEnd) and
              (tai(hp1).typ=ait_instruction) and
              IsJumpToLabel(taicpu(hp1)) then
              begin
                NCJLabel := TAsmLabel(JumpTargetOp(taicpu(hp1))^.ref^.symbol);

                if IsJumpToLabelUncond(taicpu(hp1)) then
                  begin
                    { Do it now to get it out of the way and to aid optimisations
                      later on in this method }
                    if RemoveDeadCodeAfterJump(taicpu(hp1)) then
                      stoploop := False;

                    hp2 := getlabelwithsym(NCJLabel);
                    if Assigned(hp2) then
                      { Collapse the cluster now to aid optimisation and potentially
                        cut down on the number of iterations required }
                      NCJLabel := CollapseLabelCluster(hp1, hp2);

                    { GetNextInstruction could be factored out, but hp2 might be
                      different after "RemoveDeadCodeAfterJump" }
                    GetNextInstruction(hp1, hp2);

                    { Check for:
                        jmp<cond> @Lbl
                        jmp       @Lbl
                    }
                    if (CJLabel = NCJLabel) then
                      begin
                        DebugMsg(SPeepholeOptimization+'Short-circuited conditional jump',p);
                        { Both jumps go to the same label }
                        CJLabel.decrefs;
{$ifdef cpudelayslot}
                        RemoveDelaySlot(p);
{$endif cpudelayslot}
                        UpdateUsedRegs(tai(p.Next));
                        AsmL.Remove(p);
                        p.Free;
                        p := hp1;

                        Result := True;
                        Exit;
                      end;

                    if FindLabel(CJLabel, hp2) then
                      begin
                        { change the following jumps:
                            jmp<cond> CJLabel         jmp<inv_cond> NCJLabel
                            jmp       NCJLabel >>>    <code>
                          CJLabel:                  NCJLabel:
                            <code>
                          NCJLabel:
                        }
{$if defined(arm) or defined(aarch64)}
                        if (taicpu(p).condition<>C_None)
{$if defined(aarch64)}
                        { can't have conditional branches to
                          global labels on AArch64, because the
                          offset may become too big }
                        and (NCJLabel.bind=AB_LOCAL)
{$endif aarch64}
                        then
                          begin
{$endif arm or aarch64}
                            DebugMsg(SPeepholeOptimization+'Conditional jump inversion',p);

                            taicpu(p).condition:=inverse_cond(taicpu(p).condition);
                            CJLabel.decrefs;

                            JumpTargetOp(taicpu(p))^.ref^.symbol := NCJLabel;

                            { when freeing hp1, the reference count
                              isn't decreased, so don't increase }
{$ifdef cpudelayslot}
                            RemoveDelaySlot(hp1);
{$endif cpudelayslot}
                            asml.remove(hp1);
                            hp1.free;

                            stoploop := False;

                            if not CJLabel.is_used then
                              begin
                                CJLabel := NCJLabel;

                                StripDeadLabels(tai(p.Next), hp1);
                                if (hp1 = BlockEnd) then
                                  Exit;

                                { Attempt another iteration in case more jumps follow }
                                if (hp1.typ in SkipInstr) then
                                  GetNextInstruction(hp1, hp1);

                                Continue;
                              end;
{$if defined(arm) or defined(aarch64)}
                          end;
{$endif arm or aarch64}
                      end
                    else if CollapseZeroDistJump(hp1, NCJLabel) then
                      begin
                        if (hp1 = BlockEnd) then
                          Exit;

                        { Attempt another iteration in case more jumps follow }
                        if (hp1.typ in SkipInstr) then
                          GetNextInstruction(hp1, hp1);

                        Continue;
                      end;
                  end
                else
                  begin
                    { Check for:
                        jmp<cond1>    @Lbl1
                        jmp<cond2>    @Lbl2

                        Remove 2nd jump if conditions are equal or cond2 is fully covered by cond1
                    }

                    if condition_in(taicpu(hp1).condition, taicpu(p).condition) then
                      begin
                        DebugMsg(SPeepholeOptimization+'Dominated conditional jump',p);

                        NCJLabel.decrefs;
                        GetNextInstruction(hp1, hp2);

                        AsmL.Remove(hp1);
                        hp1.Free;

                        hp1 := hp2;

                        { Flag another pass in case @Lbl2 appeared earlier in the procedure and is now a dead label }
                        stoploop := False;
                        { Attempt another iteration in case more jumps follow }
                        Continue;
                      end;

                    { Check for:
                        jmp<cond1>  @Lbl1
                        jmp<cond2>  @Lbl2

                      And inv(cond2) is a subset of cond1 (e.g. je followed by jne, or jae followed by jbe) )
                    }
                    if condition_in(inverse_cond(taicpu(hp1).condition), taicpu(p).condition) then
                      begin
                        GetNextInstruction(hp1, hp2);

                        { If @lbl1 immediately follows jmp<cond2>, we can remove
                          the first jump completely }
                        if FindLabel(CJLabel, hp2) then
                          begin
                            DebugMsg(SPeepholeOptimization+'jmp<cond> before jmp<inv_cond> - removed first jump',p);

                            CJLabel.decrefs;
{$ifdef cpudelayslot}
                            RemoveDelaySlot(p);
{$endif cpudelayslot}
                            UpdateUsedRegs(tai(p.Next));
                            AsmL.Remove(p);
                            p.Free;
                            p := hp1;

                            Result := True;
                            Exit;

{$if not defined(avr)}
                          end
                        else
                          { NOTE: There is currently no watertight, cross-platform way to create
                            an unconditional jump without access to the cg object.  If anyone can
                            improve this particular optimisation to work on AVR,
                            please do. [Kit] }
                          begin
                            { Since cond1 is a subset of inv(cond2), jmp<cond2> will always branch if
                              jmp<cond1> does not, so change jmp<cond2> to an unconditional jump. }

                            DebugMsg(SPeepholeOptimization+'jmp<cond> before jmp<inv_cond> - made second jump unconditional',p);

                            MakeUnconditional(taicpu(hp1));

                            { NOTE: Changing the jump to unconditional won't open up new opportunities
                              for GetFinalDestination on earlier jumps because there's no live label
                              between the two jump instructions, so setting 'stoploop' to False only
                              wastes time. [Kit] }

                            { See if more optimisations are possible }
                            Continue;
{$endif}
                          end;
                      end;
                  end;
            end;

            if GetFinalDestination(taicpu(p),0) then
              stoploop := False;

            Exit;
          end;

      end;
{$endif JVM}

    function TAOptObj.CollapseZeroDistJump(var p: tai; ThisLabel: TAsmLabel): Boolean;
      var
        tmp, hp1: tai;
      begin
        Result := False;
        hp1 := tai(p.Next);
        tmp := hp1; { Might be an align before the label, so keep a note of it }
        if (hp1 = BlockEnd) then
          Exit;

        { remove jumps to labels coming right after them }
        if FindLabel(ThisLabel, hp1) and
            { Cannot remove the first instruction }
            (p<>StartPoint) then
          begin
            ThisLabel.decrefs;

{$ifdef cpudelayslot}
            RemoveDelaySlot(p);
{$endif cpudelayslot}
            asml.remove(p);
            p.free;

            StripDeadLabels(tmp, p);

            if p.typ <> ait_instruction then
              GetNextInstruction(UpdateUsedRegsAndOptimize(p), p);

            Result := True;
          end;

    end;


    function TAOptObj.CanDoJumpOpts: Boolean;
      begin
        { Always allow by default }
        Result := True;
      end;


    function TAOptObj.DoJumpOptimizations(var p: tai; var stoploop: Boolean): Boolean;
      var
        hp1, hp2: tai;
        ThisLabel: TAsmLabel;
        ThisPassResult: Boolean;
      begin
        Result := False;
        if (p.typ <> ait_instruction) or not IsJumpToLabel(taicpu(p)) then
          Exit;

        repeat
          ThisPassResult := False;

          if GetNextInstruction(p, hp1) and (hp1 <> BlockEnd) then
            begin
              SkipEntryExitMarker(hp1,hp1);
              if (hp1 = BlockEnd) then
                Exit;

              ThisLabel := TAsmLabel(JumpTargetOp(taicpu(p))^.ref^.symbol);

              hp2 := getlabelwithsym(ThisLabel);

              { getlabelwithsym returning nil occurs if a label is in a
                different block (e.g. on the other side of an asm...end pair). }
              if Assigned(hp2) then
                begin
                  { If there are multiple labels in a row, change the destination to the last one
                    in order to aid optimisation later }
                  ThisLabel := CollapseLabelCluster(p, hp2);

                  if CollapseZeroDistJump(p, ThisLabel) then
                    begin
                      stoploop := False;
                      Result := True;
                      Exit;
                    end;

                  if IsJumpToLabelUncond(taicpu(p)) then
                    begin
                      { Remove unreachable code between the jump and the next label }
                      ThisPassResult := RemoveDeadCodeAfterJump(taicpu(p));

                      if GetFinalDestination(taicpu(p), 0) or ThisPassResult then
                        { Might have caused some earlier labels to become dead }
                        stoploop := False;
                    end
{$ifndef JVM}
                  else if (taicpu(p).opcode = aopt_condjmp) then
                    ThisPassResult := OptimizeConditionalJump(ThisLabel, p, hp1, stoploop)
{$endif JVM}
                    ;
                end;

            end;

          Result := Result or ThisPassResult;
        until not (ThisPassResult and (p.typ = ait_instruction) and IsJumpToLabel(taicpu(p)));

      end;


    function TAOptObj.GetFinalDestination(hp: taicpu; level: longint): boolean;
      {traces sucessive jumps to their final destination and sets it, e.g.
       je l1                je l3       <code>               <code>
       l1:       becomes    l1:
       je l2                je l3
       <code>               <code>
       l2:                  l2:
       jmp l3               jmp l3

       the level parameter denotes how deep we have already followed the jump,
       to avoid endless loops with constructs such as "l5: ; jmp l5"           }

      var p1: tai;
          p2: tai;
{$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
          p3: tai;
{$endif}
          ThisLabel, l: tasmlabel;

      begin
        GetFinalDestination := false;
        if level > 20 then
          exit;

        ThisLabel := TAsmLabel(JumpTargetOp(hp)^.ref^.symbol);
        p1 := getlabelwithsym(ThisLabel);
        if assigned(p1) then
          begin
            SkipLabels(p1,p1);
            if (p1.typ = ait_instruction) and
               (taicpu(p1).is_jmp) then
              begin
                p2 := tai(p1.Next);
                if p2 = BlockEnd then
                  Exit;

                { Collapse any zero distance jumps we stumble across }
                while (p1<>StartPoint) and CollapseZeroDistJump(p1, TAsmLabel(JumpTargetOp(taicpu(p1))^.ref^.symbol)) do
                  begin
                    { Note: Cannot remove the first instruction }
                    if (p1.typ = ait_label) then
                      SkipLabels(p1, p1);

                    if not Assigned(p1) then
                      { No more valid commands }
                      Exit;

                    { Check to see that we are actually still at a jump }
                    if not ((tai(p1).typ = ait_instruction) and (taicpu(p1).is_jmp)) then
                      begin
                        { Required to ensure recursion works properly, but to also
                          return false if a jump isn't modified. [Kit] }
                        if level > 0 then GetFinalDestination := True;
                        Exit;
                      end;

                    p2 := tai(p1.Next);
                    if p2 = BlockEnd then
                      Exit;
                  end;

{$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
                p3 := p2;
{$endif not MIPS and not RV64 and not RV32 and not JVM}

                if { the next instruction after the label where the jump hp arrives}
                   { is unconditional or of the same type as hp, so continue       }
                   IsJumpToLabelUncond(taicpu(p1))

                   { TODO: For anyone with experience with MIPS or RISC-V, please add support for tracing
                     conditional jumps. [Kit] }

{$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
  { for MIPS, it isn't enough to check the condition; first operands must be same, too. }
                   or
                   condition_in(hp.condition, taicpu(p1).condition) or

                   { the next instruction after the label where the jump hp arrives
                     is the opposite of hp (so this one is never taken), but after
                     that one there is a branch that will be taken, so perform a
                     little hack: set p1 equal to this instruction }
                   (condition_in(hp.condition, inverse_cond(taicpu(p1).condition)) and
                     SkipLabels(p3,p2) and
                     (p2.typ = ait_instruction) and
                     (taicpu(p2).is_jmp) and
                       (IsJumpToLabelUncond(taicpu(p2)) or
                       (condition_in(hp.condition, taicpu(p2).condition))
                     ) and
                     SetAndTest(p2,p1)
                   )
{$endif not MIPS and not RV64 and not RV32 and not JVM}
                   then
                  begin
                    { quick check for loops of the form "l5: ; jmp l5" }
                    if (TAsmLabel(JumpTargetOp(taicpu(p1))^.ref^.symbol).labelnr = ThisLabel.labelnr) then
                      exit;
                    if not GetFinalDestination(taicpu(p1),succ(level)) then
                      exit;

                    { NOTE: Do not move this before the "l5: ; jmp l5" check,
                      because GetFinalDestination may change the destination
                      label of p1. [Kit] }

                    l := tasmlabel(JumpTargetOp(taicpu(p1))^.ref^.symbol);

{$if defined(aarch64)}
                    { can't have conditional branches to
                      global labels on AArch64, because the
                      offset may become too big }
                    if not(taicpu(hp).condition in [C_None,C_AL,C_NV]) and
                       (l.bind<>AB_LOCAL) then
                      exit;
{$endif aarch64}
                    ThisLabel.decrefs;
                    JumpTargetOp(hp)^.ref^.symbol:=l;
                    l.increfs;
                    GetFinalDestination := True;
                    Exit;
                  end
{$if not defined(MIPS) and not defined(riscv64) and not defined(riscv32) and not defined(JVM)}
                else
                  if condition_in(inverse_cond(hp.condition), taicpu(p1).condition) then
                    begin
                      if not FindLiveLabel(p1,l) then
                        begin
{$ifdef finaldestdebug}
                          insertllitem(asml,p1,p1.next,tai_comment.Create(
                            strpnew('previous label inserted'))));
{$endif finaldestdebug}
                          current_asmdata.getjumplabel(l);
                          insertllitem(p1,p1.next,tai_label.Create(l));

                          ThisLabel.decrefs;
                          JumpTargetOp(hp)^.ref^.symbol := l;
                          l.increfs;
                          GetFinalDestination := True;
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
                          ThisLabel.decrefs;
                          JumpTargetOp(hp)^.ref^.symbol := l;
                          if not GetFinalDestination(hp,succ(level)) then
                            exit;
                        end;
                      GetFinalDestination := True;
                      Exit;
                    end;
{$endif not MIPS and not RV64 and not RV32 and not JVM}
              end;
          end;

        { Required to ensure recursion works properly, but to also
          return false if a jump isn't modified. [Kit] }
        if level > 0 then GetFinalDestination := True;
      end;


    procedure TAOptObj.PrePeepHoleOpts;
      var
        p: tai;
      begin
        p := BlockStart;
        ClearUsedRegs;
        while (p <> BlockEnd) Do
          begin
            prefetch(pointer(p.Next)^);
            if PrePeepHoleOptsCpu(p) then
              continue;
            if assigned(p) then
              begin
                p:=tai(p.next);
                UpdateUsedRegs(p);
              end;
          end;
      end;


    procedure TAOptObj.PeepHoleOptPass1;
      var
        p : tai;
        stoploop, FirstInstruction, JumpOptsAvailable: boolean;
      begin
        JumpOptsAvailable := CanDoJumpOpts();

        StartPoint := BlockStart;

        repeat
          stoploop:=true;
          p := StartPoint;
          FirstInstruction := True;
          ClearUsedRegs;

          while Assigned(p) and (p <> BlockEnd) Do
            begin
              prefetch(pointer(p.Next)^);

              { I'am not sure why this is done, UsedRegs should reflect the register usage before the instruction
                If an instruction needs the information of this, it can easily create a TempUsedRegs (FK)
              UpdateUsedRegs(tai(p.next));
              }
{$ifdef DEBUG_OPTALLOC}
              if p.Typ=ait_instruction then
                InsertLLItem(tai(p.Previous),p,tai_comment.create(strpnew(GetAllocationString(UsedRegs))));
{$endif DEBUG_OPTALLOC}

              { Handle jump optimizations first }
              if JumpOptsAvailable and DoJumpOptimizations(p, stoploop) then
                begin
                  if FirstInstruction then
                    { Update StartPoint, since the old p was removed;
                      don't set FirstInstruction to False though, as
                      the new p might get removed too. }
                    StartPoint := p;

                  if (p.typ = ait_instruction) and IsJumpToLabel(taicpu(p)) then
                    Continue;
                end;

              if PeepHoleOptPass1Cpu(p) then
                begin
                  stoploop:=false;
                  UpdateUsedRegs(p);

                  if FirstInstruction then
                    { Update StartPoint, since the old p was modified;
                      don't set FirstInstruction to False though, as
                      the new p might get modified too. }
                    StartPoint := p;

                  continue;
                end;

              FirstInstruction := False;
              if assigned(p) then
                p := tai(UpdateUsedRegsAndOptimize(p).Next);

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
            prefetch(pointer(p.Next)^);
            if PeepHoleOptPass2Cpu(p) then
              continue;
            if assigned(p) then
              p := tai(UpdateUsedRegsAndOptimize(p).Next);
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
            prefetch(pointer(p.Next)^);
            if PostPeepHoleOptsCpu(p) then
              continue;
            if assigned(p) then
              begin
                p:=tai(p.next);
                UpdateUsedRegs(p);
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
