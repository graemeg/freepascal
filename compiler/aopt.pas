{
    Copyright (c) 1998-2004 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit contains the interface routines between the code generator
    and the optimizer.

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
Unit aopt;

{$i fpcdefs.inc}

{ $define DEBUG_OPTALLOC}
{ $define DEBUG_INSTRUCTIONREGISTERDEPENDENCIES}

  Interface

    Uses
      aasmbase,aasmtai,aasmdata,aasmcpu,
      aoptobj;

    Type
      TAsmOptimizer = class(TAoptObj)
        { Pooled object that can be used by optimisation procedures to evaluate
          future register usage without upsetting the current state. }
        TmpUsedRegs: TAllUsedRegs;

        { _AsmL is the PAasmOutpout list that has to be optimized }
        Constructor create(_AsmL: TAsmList); virtual; reintroduce;

        { call the necessary optimizer procedures }
        Procedure Optimize;virtual;
        Destructor destroy;override;

      private
        procedure FindLoHiLabels;

        { Builds a table with the locations of the labels in the TAsmList.
          Also fixes some RegDeallocs like "# %eax released; push (%eax)"  }
        Procedure BuildLabelTableAndFixRegAlloc;
      protected
        procedure pass_1;
        procedure clear;
      End;
      TAsmOptimizerClass = class of TAsmOptimizer;

      TAsmScheduler = class(TAoptObj)
        { _AsmL is the PAasmOutpout list that has to be re-scheduled }
        Constructor Create(_AsmL: TAsmList); virtual; reintroduce;
        Procedure Optimize;
        function SchedulerPass1Cpu(var p: tai): boolean; virtual; abstract;
        procedure SchedulerPass1;
      end;
      TAsmSchedulerClass = class of TAsmScheduler;

    var
      casmoptimizer : TAsmOptimizerClass;
      cpreregallocscheduler : TAsmSchedulerClass;

    procedure Optimize(AsmL:TAsmList);
    procedure PreRegallocSchedule(AsmL:TAsmList);

  Implementation

    uses
      cutils,
      cprofile,
      globtype, globals,
      verbose,
      cpubase,
      cgbase,
      aoptcpu;

    Constructor TAsmOptimizer.create(_AsmL: TAsmList);
      Begin
        inherited create(_asml,nil,nil,nil);
        { setup labeltable, always necessary }
        New(LabelInfo);
        CreateUsedRegs(TmpUsedRegs);
      End;

    procedure TAsmOptimizer.FindLoHiLabels;
      { Walks through the paasmlist to find the lowest and highest label number.  }
      { Returns the last Pai object of the current block                          }
      Var LabelFound: Boolean;
          p: tai;
      Begin
        LabelInfo^.LowLabel := High(longint);
        LabelInfo^.HighLabel := 0;
        LabelInfo^.LabelDif := 0;
        LabelInfo^.LabelTable:=nil;
        LabelFound := False;
        P := BlockStart;
        With LabelInfo^ Do
          Begin
            While Assigned(P) And
                  ((P.typ <> Ait_Marker) Or
                   (tai_Marker(P).Kind <> mark_AsmBlockStart)) Do
              Begin
                If (p.typ = ait_label) and
                   (tai_Label(p).labsym.labeltype=alt_jump) and
                   (tai_Label(p).labsym.is_used) Then
                  Begin
                    LabelFound := True;
                    If (tai_Label(p).labsym.labelnr < LowLabel) Then
                      LowLabel := tai_Label(p).labsym.labelnr;
                    If (tai_Label(p).labsym.labelnr > HighLabel) Then
                      HighLabel := tai_Label(p).labsym.labelnr
                  End;
                GetNextInstruction(p, p)
              End;
            blockend:=p;
            If LabelFound
              Then LabelDif := HighLabel-LowLabel+1
              Else LabelDif := 0
          End
      End;


    Procedure TAsmOptimizer.BuildLabelTableAndFixRegAlloc;
    Var p,hp1, hp2: tai;
        Regs: TAllUsedRegs;
        LabelIdx : longint;
    Begin
      CreateUsedRegs(Regs);
      With LabelInfo^ Do
        begin
          If (LabelDif <> 0) Then
            Begin
              GetMem(LabelTable, LabelDif*SizeOf(TLabelTableItem));
              FillChar(LabelTable^, LabelDif*SizeOf(TLabelTableItem), 0);
            end;
          p := BlockStart;
          While (P <> BlockEnd) Do
            Begin
              prefetch(pointer(p.Next)^);
              Case p.typ Of
                ait_Label:
                  begin
                    If tai_label(p).labsym.is_used and
                       (tai_Label(p).labsym.labeltype=alt_jump) then
                      begin
                        LabelIdx:=tai_label(p).labsym.labelnr-LowLabel;
                        if LabelIdx>int64(LabelDif) then
                          internalerror(200604202);
                        LabelTable^[LabelIdx].PaiObj := p;
                      end;
                  end;
                ait_regAlloc:
                  begin
                    if tai_regalloc(p).ratype=ra_alloc then
                      Begin
                        If Not(RegInUsedRegs(tai_regalloc(p).Reg,Regs)) Then
                          IncludeRegInUsedRegs(tai_regalloc(p).Reg,Regs)
                        Else
                          Begin
                            hp1 := tai(p.previous);
{$ifdef DEBUG_OPTALLOC}
                            AsmL.InsertAfter(tai_comment.Create(strpnew('Removed allocation of '+std_regname(tai_regalloc(p).Reg))),p);
{$endif DEBUG_OPTALLOC}
                            AsmL.remove(p);
                            p.free;
                            p := hp1;
                            { not sure if this is useful, it even skips previous deallocs of the register (FK)
                            hp1 := p;
                            hp2 := nil;
                            While GetLastInstruction(hp1, hp1) And
                                  Not(RegInInstruction(tai_regalloc(p).Reg, hp1)) Do
                              hp2:=hp1;
                            If hp2<>nil Then
                              Begin
                                hp1:=tai_regalloc.DeAlloc(tai_regalloc(p).Reg,hp2);
                                InsertLLItem(tai(hp2.previous), hp2, hp1);
                              End;
                            }
                          End;
                      End
                    else if tai_regalloc(p).ratype=ra_dealloc then
                      Begin
                        hp1 := p;
                        hp2 := nil;
                        While Not(assigned(FindRegAlloc(tai_regalloc(p).Reg, tai(hp1.Next)))) And
                              GetNextInstruction(hp1, hp1) And
                              InstructionLoadsFromReg(tai_regalloc(p).Reg, hp1) Do
                          hp2 := hp1;
                        { move deallocations }
                        If hp2 <> nil Then
                          Begin
                            hp1 := tai(p.previous);
{$ifdef DEBUG_OPTALLOC}
                            AsmL.InsertAfter(tai_comment.Create(strpnew('Moved deallocation of '+std_regname(tai_regalloc(p).Reg))),p);
{$endif DEBUG_OPTALLOC}
                            AsmL.Remove(p);
                            InsertLLItem(hp2, tai(hp2.Next), p);
                            { don't remove this deallocation later on when merging dealloc/alloc pairs because
                              it marks indenpendent use of a register

                              This could be also achieved by a separate passes for merging first and then later
                              moving but I did not choose this solution because it takes more time and code (FK) }
                            tai_regalloc(p).keep:=true;
{$ifdef DEBUG_OPTALLOC}
                            AsmL.InsertAfter(tai_comment.Create(strpnew('Moved deallocation of '+std_regname(tai_regalloc(p).Reg)+' here')),hp2);
{$endif DEBUG_OPTALLOC}
                            p := hp1;
                          End
                        { merge allocations/deallocations }
                        else if assigned(findregalloc(tai_regalloc(p).reg, tai(p.next)))
                          and getnextinstruction(p,hp1) and
                          { don't merge deallocations/allocation which mark a new use of register, this
                            enables more possibilities for the peephole optimizer }
                          not(tai_regalloc(p).keep) then
                          begin
                            hp1 := tai(p.previous);
{$ifdef DEBUG_OPTALLOC}
                            AsmL.InsertAfter(tai_comment.Create(strpnew('Removed deallocation of '+std_regname(tai_regalloc(p).Reg))),p);
{$endif DEBUG_OPTALLOC}
                            AsmL.remove(p);
                            p.free;
                            p := hp1;
                          end
                        else
                          ExcludeRegFromUsedRegs(tai_regalloc(p).Reg,Regs);
                      End
                  End
                else
                  ;
              End;
              P := tai(p.Next);
              While Assigned(p) and
                    (p <> blockend) and
                    (p.typ in (SkipInstr - [ait_regalloc])) Do
                P := tai(P.Next)
            End;
        end;
      ReleaseUsedRegs(Regs);
    End;

    procedure tasmoptimizer.clear;
      begin
        if assigned(LabelInfo^.labeltable) then
          begin
            freemem(LabelInfo^.labeltable);
            LabelInfo^.labeltable := nil;
          end;
        LabelInfo^.labeldif:=0;
        LabelInfo^.lowlabel:=high(longint);
        LabelInfo^.highlabel:=0;
      end;


    procedure tasmoptimizer.pass_1;
      begin
        findlohilabels;
        BuildLabelTableAndFixRegAlloc;
      end;


    Procedure TAsmOptimizer.Optimize;
      Var
        HP: tai;
        pass: longint;
      Begin
        pass:=0;
        BlockStart := tai(AsmL.First);
        pass_1;
        While Assigned(BlockStart) Do
          Begin
            if (cs_opt_peephole in current_settings.optimizerswitches) then
              begin
                if pass = 0 then
                  PrePeepHoleOpts;
                { Peephole optimizations }
                PeepHoleOptPass1;
                { Only perform them twice in the first pass }
                if pass = 0 then
                  PeepHoleOptPass1;
              end;
            { more peephole optimizations }
            if (cs_opt_peephole in current_settings.optimizerswitches) then
              begin
                PeepHoleOptPass2;
                { if pass = last_pass then }
                PostPeepHoleOpts;
              end;
            { free memory }
            clear;
            { continue where we left off, BlockEnd is either the start of an }
            { assembler block or nil}
            BlockStart := BlockEnd;
            While Assigned(BlockStart) And
                  (BlockStart.typ = ait_Marker) And
                  (tai_Marker(BlockStart).Kind = mark_AsmBlockStart) Do
              Begin
               { we stopped at an assembler block, so skip it    }
               While GetNextInstruction(BlockStart, BlockStart) And
                     ((BlockStart.Typ <> Ait_Marker) Or
                      (tai_Marker(Blockstart).Kind <> mark_AsmBlockEnd)) Do;
               { blockstart now contains a tai_marker(mark_AsmBlockEnd) }
               If GetNextInstruction(BlockStart, HP) And
                  ((HP.typ <> ait_Marker) Or
                   (Tai_Marker(HP).Kind <> mark_AsmBlockStart)) Then
               { There is no assembler block anymore after the current one, so }
               { optimize the next block of "normal" instructions              }
                 pass_1
               { Otherwise, skip the next assembler block }
               else
                 blockStart := hp;
              End
          End;
      End;


    Destructor TAsmOptimizer.Destroy;
      Begin
        ReleaseUsedRegs(TmpUsedRegs);
        if assigned(LabelInfo^.LabelTable) then
          Freemem(LabelInfo^.LabelTable);
        Dispose(LabelInfo);
        inherited Destroy;
      End;


    constructor TAsmScheduler.Create(_AsmL: TAsmList);
      begin
        inherited create(_asml,nil,nil,nil);
      end;


    procedure TAsmScheduler.SchedulerPass1;
      var
        p : tai;
      begin
        p:=BlockStart;
        while p<>BlockEnd Do
          begin
            prefetch(pointer(p.Next)^);
            if SchedulerPass1Cpu(p) then
              continue;
            p:=tai(p.next);
          end;
      end;


    procedure TAsmScheduler.Optimize;
      Var
        HP: tai;
      Begin
        BlockStart := tai(AsmL.First);
        While Assigned(BlockStart) Do
          Begin
            { Peephole optimizations }
            SchedulerPass1;
            { continue where we left off, BlockEnd is either the start of an }
            { assembler block or nil}
            BlockStart:=BlockEnd;
            While Assigned(BlockStart) And
                  (BlockStart.typ = ait_Marker) And
                  (tai_Marker(BlockStart).Kind = mark_AsmBlockStart) Do
              Begin
                { we stopped at an assembler block, so skip it    }
                While GetNextInstruction(BlockStart, BlockStart) And
                      ((BlockStart.Typ <> Ait_Marker) Or
                       (tai_Marker(Blockstart).Kind <> mark_AsmBlockEnd)) Do;
                { blockstart now contains a tai_marker(mark_AsmBlockEnd) }
                If not(GetNextInstruction(BlockStart, HP) And
                   ((HP.typ <> ait_Marker) Or
                    (Tai_Marker(HP).Kind <> mark_AsmBlockStart))) Then
                  { skip the next assembler block }
                  blockStart := hp;
              End
          End;
      End;


    procedure Optimize(AsmL:TAsmList);
      var
        p : TAsmOptimizer;
      begin
        ResumeTimer(ct_aopt);
        p:=casmoptimizer.Create(AsmL);
        p.Optimize;
{$ifdef DEBUG_INSTRUCTIONREGISTERDEPENDENCIES}
        p.Debug_InsertInstrRegisterDependencyInfo;
{$endif DEBUG_INSTRUCTIONREGISTERDEPENDENCIES}
        p.free;
        StopTimer;
      end;


    procedure PreRegallocSchedule(AsmL:TAsmList);
      var
        p : TAsmScheduler;
      begin
        p:=cpreregallocscheduler.Create(AsmL);
        p.Optimize;
        p.free
      end;


end.
