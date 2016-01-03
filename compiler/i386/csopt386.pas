{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
      development team

    This unit contains the common subexpression elimination procedure.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS for A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit CSOpt386;

{$i fpcdefs.inc}

interface

uses aasmbase,aasmtai,aasmdata,aasmcpu, cpuinfo, cpubase, optbase, cgbase;

function CSE(asml: TAsmList; first, last: tai; pass: longint): boolean;

function doReplaceReg(hp: taicpu; newReg, orgReg: tsuperregister): boolean;
function changeOp(var o: toper; newReg, orgReg: tsuperregister): boolean;
function storeBack(start, current: tai; orgReg, newReg: tsuperregister): boolean;
function NoHardCodedRegs(p: taicpu; orgReg, newReg: tsuperregister): boolean;
function RegSizesOK(oldReg,newReg: tsuperregister; p: taicpu): boolean;

implementation

uses
{$ifdef csdebug}
  cutils,
{$else}
  {$ifdef replaceregdebug}cutils,{$endif}
{$endif}
  globtype, verbose, procinfo, globals, daopt386, rgobj, rropt386,cgutils;

{
function TaiInSequence(P: tai; Const Seq: TContent): Boolean;
var P1: tai;
    Counter: Byte;
    TmpResult: Boolean;
begin
  TmpResult := False;
  P1 := Seq.StartMod;
  Counter := 1;
  while not(TmpResult) and
        (Counter <= Seq.NrofMods) do
    begin
      if (P = P1) then TmpResult := True;
      inc(Counter);
      p1 := tai(p1.Next);
    end;
  TaiInSequence := TmpResult;
end;
}

function modifiesConflictingMemLocation(p1: tai; supreg: tsuperregister; c: tregContent;
   var regsStillValid: tregset; onlymem: boolean; var invalsmemwrite: boolean): boolean;
var
  p, hp: taicpu;
  tmpRef: treference;
  r,regCounter: tsuperregister;
  opCount: longint;
  dummy: boolean;
begin
  modifiesConflictingMemLocation := false;
  invalsmemwrite := false;
  if p1.typ <> ait_instruction then
    exit;
  p := taicpu(p1);
  case p.opcode of
    A_MOV,A_MOVSX,A_MOVZX:
      if p.oper[1]^.typ = top_ref then
        for regCounter := RS_EAX to RS_EDI do
          begin
            if (p.oper[0]^.typ<>top_reg) or
               (getregtype(p.oper[0]^.reg) <> R_INTREGISTER) then
               break;
            if writeToMemDestroysContents(getsupreg(p.oper[0]^.reg),p.oper[1]^.ref^,
                 regCounter,topsize2tcgsize[p.opsize],c[regCounter],dummy) then
              begin
                exclude(regsStillValid,regCounter);
                modifiesConflictingMemLocation := not(supreg in regsStillValid);
              end;
            if (regcounter = supreg) then
              invalsmemwrite := invalsmemwrite or dummy;
          end
      else
{         if is_reg_var[getsupreg(p.oper[1]^.reg)] then }
        if not onlymem  then
          for regCounter := RS_EAX to RS_EDI do
            begin
              if writeDestroysContents(p.oper[1]^,regCounter,topsize2tcgsize[p.opsize],c[regCounter],dummy) then
                begin
                  exclude(regsStillValid,regCounter);
                  modifiesConflictingMemLocation := not(supreg in regsStillValid);
                end
            end;
    A_DIV, A_IDIV, A_MUL, A_IMUL:
      begin
        if not onlymem then
          if (p.ops = 1) then
            begin
              for regCounter := RS_EAX to RS_EDI do
                begin
                  if writeToRegDestroysContents(RS_EDX,regCounter,c[regCounter]) then
                    begin
                      exclude(regsStillValid,RS_EDX);
                      modifiesConflictingMemLocation := not(supreg in regsStillValid);
                    end;
                  if writeToRegDestroysContents(RS_EAX,regCounter,c[regCounter]) then
                    begin
                      exclude(regsStillValid,RS_EAX);
                      modifiesConflictingMemLocation := not(supreg in regsStillValid);
                    end;
                end
            end
          else
            { only possible for imul }
            { last operand is always destination }
            for regCounter := RS_EAX to RS_EDI do
              begin
                if writeDestroysContents(p.oper[p.ops-1]^,regCounter,topsize2tcgsize[p.opsize],c[regCounter],dummy) then
                  begin
                    exclude(regsStillValid,regCounter);
                    modifiesConflictingMemLocation := not(supreg in regsStillValid);
                  end
              end
      end;
    else
      for opCount := 1 to maxinschanges do
        case InsProp[p.opcode].Ch[opCount] of
          Ch_MOp1,CH_WOp1,CH_RWOp1:
              if not(onlymem) or
                 (p.oper[0]^.typ = top_ref) then
{                or ((p.oper[0]^.typ = top_reg) and }
{                 is_reg_var[getsupreg(p.oper[0]^.reg)]) then }
                for regCounter := RS_EAX to RS_EDI do
                  begin
                    if writeDestroysContents(p.oper[0]^,regCounter,topsize2tcgsize[p.opsize],c[regCounter],dummy) then
                      begin
                        exclude(regsStillValid,regCounter);
                        modifiesConflictingMemLocation := not(supreg in regsStillValid);
                      end;
                    if (regcounter = supreg) then
                      invalsmemwrite := invalsmemwrite or dummy;
                  end;
          Ch_MOp2,CH_WOp2,CH_RWOp2:
              if not(onlymem) or
                 (p.oper[1]^.typ = top_ref) then
{                or ((p.oper[1]^.typ = top_reg) and }
{                 is_reg_var[getsupreg(p.oper[1]^.reg)]) then }
                for regCounter := RS_EAX to RS_EDI do
                  begin
                    if writeDestroysContents(p.oper[1]^,regCounter,topsize2tcgsize[p.opsize],c[regCounter],dummy) then
                      begin
                        exclude(regsStillValid,regCounter);
                        modifiesConflictingMemLocation := not(supreg in regsStillValid);
                      end;
                    if (regcounter = supreg) then
                      invalsmemwrite := invalsmemwrite or dummy;
                  end;
          Ch_MOp3,CH_WOp3,CH_RWOp3:
              if not(onlymem) or
                 (p.oper[2]^.typ = top_ref) then
{                or ((p.oper[2]^.typ = top_reg) and }
{                 is_reg_var[getsupreg(p.oper[2]^.reg)]) then }
                for regCounter := RS_EAX to RS_EDI do
                  begin
                    if writeDestroysContents(p.oper[2]^,regCounter,topsize2tcgsize[p.opsize],c[regCounter],dummy) then
                      begin
                        exclude(regsStillValid,regCounter);
                        modifiesConflictingMemLocation := not(supreg in regsStillValid);
                      end;
                    if (regcounter = supreg) then
                      invalsmemwrite := invalsmemwrite or dummy;
                  end;
          Ch_WMemEDI:
            begin
              fillchar(tmpref,sizeof(tmpref),0);
              tmpRef.base := NR_EDI;
              tmpRef.index := NR_EDI;
              for regCounter := RS_EAX to RS_EDI do
                begin
                  if writeToMemDestroysContents(RS_INVALID,tmpRef,regCounter,OS_32,c[regCounter],dummy) then
                    begin
                      exclude(regsStillValid,regCounter);
                      modifiesConflictingMemLocation := not(supreg in regsStillValid);
                   end;
                  if (regcounter = supreg) then
                    invalsmemwrite := invalsmemwrite or dummy;
                end;
            end;
        end;
  end;
end;


function isSimpleMemLoc(const ref: treference): boolean;
begin
{  isSimpleMemLoc :=
    (ref.index = RS_NO) and
    not(ref.base in (rg.usableregsint+[RS_EDI]));}
  isSimpleMemLoc :=
    (ref.index = NR_NO) and
    ((ref.base = NR_NO) or
     not(getsupreg(ref.base) in [RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI]));
end;


{checks whether the current instruction sequence (starting with p) and the
 one between StartMod and EndMod of Reg are the same. if so, the number of
 instructions that match is stored in Found and true is returned, otherwise
 Found holds the number of instructions between StartMod and EndMod and false
 is returned}
function CheckSequence(p: tai; var prev: tai; supreg: tsuperregister; var Found: Longint;
           var reginfo: toptreginfo; findPrevSeqs: boolean): Boolean;

var
  regsNotRead, regsStillValid : tregset;
  checkingPrevSequences,
  passedFlagsModifyingInstr,
  invalsmemwrite               : boolean;

  function getPrevSequence(p: tai; supreg: tsuperregister; currentPrev: tai; var newPrev: tai): tsuperregister;

  const
    current_reg: tsuperregister = RS_INVALID;

    function stillValid(p: tai): boolean;
      var
        hp: tai;
      begin
        { only regvars are still used at jump instructions }
        if (cs_opt_regvar in current_settings.optimizerswitches) and
           (p.typ = ait_instruction) and
           taicpu(p).is_jmp then
         regsstillvalid := regsstillvalid - ptaiprop(p.optinfo)^.usedregs;

        stillValid :=
          (p.typ = ait_instruction) and
          (taicpu(p).opcode <> a_jmp) and
          (ptaiprop(p.optinfo)^.regs[supreg].wstate =
            ptaiprop(currentPrev.optinfo)^.regs[supreg].wstate) and
        { in case destroyreg is called with doIncState = false }
          (ptaiprop(p.optinfo)^.regs[supreg].typ =
            ptaiprop(currentPrev.optinfo)^.regs[supreg].typ) and
          (supreg in (regsNotRead * regsStillValid));
        { stop if the register was still used right before a (conditional) }
        { jump, since in that case its current contents could still be     }
        { used in the other path of the jump)                              }
        if (p.typ = ait_instruction) and
           (taicpu(p).is_jmp) and
           getlastinstruction(p,hp) then
          stillValid := stillValid and
           not(supreg in ptaiprop(hp.optinfo)^.usedregs);
        passedFlagsModifyingInstr := passedFlagsModifyingInstr or
          instrWritesFlags(currentPrev);
      end;


    function findChangedRegister(p: tai): tsuperregister;
    var
      regCounter, loopstart: tsuperregister;
    begin
      if (current_reg <> RS_INVALID) then
        loopstart := succ(current_reg)
      else
        loopstart := RS_EAX;
      for regCounter := loopstart to RS_EDI do
        with ptaiprop(p.optinfo)^.regs[regCounter] do
        if ((startmod <>
             ptaiprop(currentPrev.optinfo)^.regs[regCounter].startmod)  or
            (nrofMods <>
             ptaiprop(currentPrev.optinfo)^.regs[regCounter].nrofMods)) and
           (ptaiprop(p.optinfo)^.regs[regCounter].typ in [con_ref,con_noRemoveRef]) then
          begin
            findChangedRegister := regCounter;
            current_reg := regCounter;
            exit;
          end;
      current_reg := RS_INVALID;
      findChangedRegister := RS_INVALID;
    end;


  var
    hp, prevFound: tai;
    tmpResult, regCounter: tsuperregister;
    invalsmemwrite: boolean;
  begin
    if (current_reg <> RS_EDI) and
       (current_reg <> RS_INVALID) then
      begin
        tmpResult := findChangedRegister(currentPrev);
        if tmpResult <> RS_INVALID then
          begin
            getPrevSequence := tmpResult;
            exit;
          end;
      end;

    getPrevSequence := RS_INVALID;
    passedFlagsModifyingInstr := passedFlagsModifyingInstr or
      instrWritesFlags(currentPrev);
    if (cs_opt_regvar in current_settings.optimizerswitches) and
       (currentprev.typ = ait_instruction) and
       taicpu(currentprev).is_jmp then
      regsstillvalid := regsstillvalid - ptaiprop(currentprev.optinfo)^.usedregs;

    if not getLastInstruction(currentPrev,hp) then
      exit;

    prevFound := currentPrev;
    tmpResult := RS_INVALID;

    while (tmpResult = RS_INVALID) and
          stillValid(hp) and
          (ptaiprop(prevFound.optinfo)^.canBeRemoved or
           not(modifiesConflictingMemLocation(prevFound,supreg,
             ptaiprop(p.optinfo)^.regs,regsStillValid,false, invalsmemwrite))) do
      begin
        { only update the regsread for the instructions we already passed }
        if not(ptaiprop(prevFound.optinfo)^.canBeRemoved) then
          for regCounter := RS_EAX to RS_EDI do
            if regReadByInstruction(regCounter,prevFound) then
              exclude(regsNotRead,regCounter);

        { in case getPreviousInstruction fails and sets hp to nil in the }
        { next iteration                                                 }
        prevFound := hp;
        if not(ptaiprop(hp.optinfo)^.canBeRemoved) then
          tmpResult := findChangedRegister(hp);
        if not getLastInstruction(hp,hp) then
          break;
      end;
    getPrevSequence := tmpResult;
    if tmpResult <> RS_INVALID then
      newPrev := prevFound;
  end;


  function getNextRegToTest(var prev: tai; currentReg: tsuperregister): tsuperregister;
  begin
    if not checkingPrevSequences then
      begin
        if (currentreg = RS_INVALID) then
          currentreg := RS_EAX
        else
          inc(currentreg);
        while (currentReg <= RS_EDI) and
              not(ptaiprop(prev.optinfo)^.regs[currentReg].typ in [con_ref,con_noRemoveRef]) do
          inc(currentReg);
        if currentReg > RS_EDI then
          begin
            if (taicpu(p).oper[0]^.typ <> top_ref) or
               isSimpleMemLoc(taicpu(p).oper[0]^.ref^) then
              begin
                checkingPrevSequences := true;
              end
            else
              getNextRegToTest := RS_INVALID;
          end
        else
          getNextRegToTest := currentReg;
      end;
    if checkingPrevSequences then
      if findPrevSeqs then
        getNextRegToTest :=
          getPrevSequence(p,supreg,prev,prev)
      else
        getNextRegToTest := RS_INVALID;
  end;


  function changedreginvalidatedbetween(const oldreginfo: toptreginfo; var newreginfo: toptreginfo; startp,endp,current: tai): boolean;
    var
      orgdiffregs,diffregs: tregset;
      runner: tai;
      invalsmemwrite: boolean;
    begin
      diffregs := newreginfo.newregsencountered - oldreginfo.newregsencountered;
      orgdiffregs := diffregs;
      if diffregs <> [] then
        begin
          runner := startp;
          repeat
            modifiesConflictingMemLocation(runner,RS_EAX { dummy },ptaiprop(current.optinfo)^.regs,diffregs,true,invalsmemwrite);
            if orgdiffregs <> diffregs then
              begin
                changedreginvalidatedbetween := true;
                newreginfo := oldreginfo;
                exit;
              end;
            getnextinstruction(runner,runner);
          until (runner = endp);
        end;
      changedreginvalidatedbetween := false;
    end;

var
  prevreginfo: toptreginfo;
  hp2, hp3{, EndMod}, prevhp3, highPrev, orgPrev, pprev: tai;
  {Cnt,} OldNrofMods: Longint;
  startRegInfo, OrgRegInfo, HighRegInfo: toptreginfo;
  regModified, lastregloadremoved: array[RS_EAX..RS_ESP] of boolean;
  HighFound, OrgRegFound: longint;
  regcounter, regCounter2, tmpreg, base, index: tsuperregister;
  OrgRegResult: Boolean;
  TmpResult, flagResultsNeeded, stopchecking: Boolean;
begin {CheckSequence}
  TmpResult := False;
  FillChar(OrgRegInfo, Sizeof(OrgRegInfo), 0);
  FillChar(startRegInfo, sizeof(startRegInfo), 0);
  OrgRegFound := 0;
  HighFound := 0;
  OrgRegResult := False;
  with startRegInfo do
    begin
      newRegsEncountered := [RS_EBP, RS_ESP];
      fillword(new2oldreg,sizeof(new2oldreg),RS_INVALID);
      new2OldReg[RS_EBP] := RS_EBP;
      new2OldReg[RS_ESP] := RS_ESP;
      oldRegsEncountered := newRegsEncountered;
    end;

  checkingPrevSequences := false;
  passedFlagsModifyingInstr := false;
  flagResultsNeeded := false;
  regsNotRead := [RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESP,RS_EBP,RS_EDI,RS_ESI];
  regsStillValid := regsNotRead;
  GetLastInstruction(p, prev);
  pprev := prev;
  tmpreg:=RS_INVALID;
  regCounter := getNextRegToTest(prev,tmpreg);
  while (regcounter <> RS_INVALID) do
    begin
      fillchar(regModified,sizeof(regModified),0);
      fillchar(lastregloadremoved,sizeof(lastregloadremoved),0);
      reginfo := startRegInfo;
      Found := 0;
      hp2 := ptaiprop(prev.optinfo)^.Regs[regcounter].StartMod;
      if (prev <> ptaiprop(prev.optinfo)^.Regs[regcounter].StartMod) then
        OldNrofMods := ptaiprop(prev.optinfo)^.Regs[regcounter].NrofMods
      else
        OldNrofMods := 1;
      hp3 := p;
      if checkingprevsequences then
        prevreginfo := reginfo;
      stopchecking := false;
      while (Found <> OldNrofMods) and
                                  { old  new }
             InstructionsEquivalent(hp2, hp3, reginfo) and
             (not(checkingprevsequences) or
              not(changedreginvalidatedbetween(prevreginfo,reginfo,prev,p,hp3))) do
        begin
          if checkingprevsequences then
            begin
              prevreginfo := reginfo;
            end;
          if (hp3.typ = ait_instruction) and
             ((taicpu(hp3).opcode = A_MOV) or
              (taicpu(hp3).opcode = A_MOVZX) or
              (taicpu(hp3).opcode = A_LEA) or
             (taicpu(hp3).opcode = A_MOVSX)) and
             (taicpu(hp3).oper[1]^.typ = top_reg) and
             not(regInOp(getsupreg(taicpu(hp3).oper[1]^.reg),taicpu(hp3).oper[0]^)) then
            begin
              tmpreg := getsupreg(taicpu(hp3).oper[1]^.reg);
              lastregloadremoved[tmpreg] := ptaiprop(hp2.optinfo)^.canberemoved;
              reginfo.lastReload[tmpreg] := hp3;
              case taicpu(hp3).oper[0]^.typ of
                top_ref:
                  begin
                    base := getsupreg(taicpu(hp3).oper[0]^.ref^.base);
                    index := getsupreg(taicpu(hp3).oper[0]^.ref^.index);
                    if (found <> 0) and
                       ((taicpu(hp3).oper[0]^.ref^.base = NR_NO) or
                        regModified[base] or
                        (base = getsupreg(current_procinfo.framepointer))) and
                       ((taicpu(hp3).oper[0]^.ref^.index = NR_NO) or
                        regModified[index]) and
                       not(regInRef(tmpReg,taicpu(hp3).oper[0]^.ref^)) then
                      begin
                        with ptaiprop(hp3.optinfo)^.regs[tmpreg] do
                          if nrofMods > (oldNrofMods - found) then
                            oldNrofMods := found + nrofMods;
                        { next is safe because instructions are equivalent }
                        with ptaiprop(hp2.optinfo)^.regs[getsupreg(taicpu(hp2).oper[1]^.reg)] do
                          if nrofMods > (oldNrofMods - found) then
                            oldNrofMods := found + nrofMods;
                      end;
                  end;
                top_reg:
                  if regModified[getsupreg(taicpu(hp3).oper[0]^.reg)] then
                    begin
                      with ptaiprop(hp3.optinfo)^.regs[tmpreg] do
                        if nrofMods > (oldNrofMods - found) then
                          oldNrofMods := found + nrofMods;
                      with ptaiprop(hp2.optinfo)^.regs[getsupreg(taicpu(hp2).oper[1]^.reg)] do
                        if nrofMods > (oldNrofMods - found) then
                          oldNrofMods := found + nrofMods;
                    end;
              end;
            end;
          for regCounter2 := RS_EAX to RS_EDI do
            regModified[regCounter2] := regModified[regCounter2] or
              regModifiedByInstruction(regCounter2,hp3);
          if flagResultsNeeded then
            flagResultsNeeded := not instrReadsFlags(hp3);
          if not flagResultsNeeded then
            flagResultsNeeded := ptaiprop(hp3.optinfo)^.FlagsUsed;
          inc(Found);
          prevhp3 := hp3;
          if (Found <> OldNrofMods) then
            if not GetNextInstruction(hp2, hp2) or
               not GetNextInstruction(hp3, hp3) then
              break;
        end;

      if assigned(hp3) then
        begin
          prevhp3 := hp3;
          getnextinstruction(hp3,hp3);
        end;
      if not assigned(hp3) or
         { a marker has no optinfo, which is used below }
         (hp3.typ = ait_marker) then
        hp3 := prevhp3;
{
a) movl  -4(%ebp),%edx
   movl -12(%ebp),%ecx
   ...
   movl  -8(%ebp),%eax
   movl -12(%ebp),%edx (marked as removable)
   movl  (%eax,%edx),%eax (replaced by "movl (%eax,%ecx),%eax")
   ...
   movl  -8(%ebp),%eax
   movl -12(%ebp),%edx
   movl  (%eax,%edx),%eax
   movl  (%edx),%edx

-> the "movl -12(ebp),%edx" can't be removed in the last sequence, because
   edx has not been replaced with ecx there, and edx is still used after the
   sequence

b) tests/webtbs/tw4266.pp
}

      { hp2 = instruction after previous sequence, pprev = instruction before }
      { current sequence, prev = instruction where the loads of the registers }
      { will be inserted                                                      }
      for regCounter2 := RS_EAX to RS_EDI do
        if (reginfo.new2OldReg[regCounter2] <> RS_INVALID) and
           { case a) above }
           (((regCounter2 in ptaiprop(hp3.optinfo)^.usedRegs) and
             (not regLoadedWithNewValue(regCounter2,false,hp3) and
              lastregloadremoved[regcounter2])) or
           { case b) above }
            ((ptaiprop(hp2.optinfo)^.regs[regCounter2].wstate <>
              ptaiprop(pprev.optinfo)^.regs[regcounter2].wstate)) or
            ((ptaiprop(hp2.optinfo)^.regs[reginfo.new2OldReg[regCounter2]].wstate <>
              ptaiprop(prev.optinfo)^.regs[reginfo.new2OldReg[regCounter2]].wstate))) then
          begin
            found := 0;
            break;
          end;

      if checkingPrevSequences then
        begin
          for regCounter2 := RS_EAX to RS_EDI do
            if (reginfo.new2OldReg[regCounter2] <> RS_INVALID) and
               (reginfo.new2OldReg[regCounter2] <> regCounter2) and
               (not(regCounter2 in (regsNotRead * regsStillValid)) or
               not(reginfo.new2OldReg[regCounter2] in regsStillValid)) then
              begin
                found := 0;
                break;
              end;
           if passedFlagsModifyingInstr and flagResultsNeeded then
              found := 0;
        end;

      TmpResult := true;
      if (found <> OldNrofMods) then
        TmpResult := false
      else if assigned(hp3) then
        for regcounter2 := RS_EAX to RS_EDI do
          if (regcounter2 in reginfo.regsLoadedforRef) and
             regModified[regcounter2] and
             (regcounter2 in ptaiprop(hp3.optinfo)^.usedRegs) and
             not regLoadedWithNewValue(regcounter2,false,hp3) then
            begin
              TmpResult := False;
              if (found > 0) then
    {this is correct because we only need to turn off the CanBeRemoved flag
    when an instruction has already been processed by CheckSequence
    (otherwise CanBeRemoved can't be true and thus can't have to be turned off).
    if it has already been processed by CheckSequence and flagged to be
    removed, it means that it has been checked against a previous sequence
    and that it was equal (otherwise CheckSequence would have returned false
    and the instruction wouldn't have been removed). if this "if found > 0"
    check is left out, incorrect optimizations are performed.}
                Found := ptaiprop(tai(p).optinfo)^.Regs[supreg].NrofMods;
              break;
            end;

      if TmpResult and
         (Found > HighFound) then
        begin
          highPrev := prev;
          HighFound := Found;
          HighRegInfo := reginfo;
        end;
      if (regcounter = supreg) then
        begin
          orgPrev := prev;
          OrgRegFound := Found;
          OrgRegResult := TmpResult;
          OrgRegInfo := reginfo
        end;
      regCounter := getNextRegToTest(prev,regCounter);
    end;
  if (HighFound > 0) and
     (not(OrgRegResult) Or
      (HighFound > OrgRegFound))
    then
      begin
        CheckSequence := True;
        prev := highPrev;
        reginfo := HighRegInfo;
        Found := HighFound
      end
    else
      begin
        CheckSequence := OrgRegResult;
        prev := orgPrev;
        Found := OrgRegFound;
        reginfo := OrgRegInfo;
      end;
end; {CheckSequence}


procedure SetAlignReg(p: tai);
Const alignSearch = 12;
var regsUsable: TRegSet;
    prevInstrCount, nextInstrCount: Longint;
    prevState, nextWState,nextRState: Array[RS_EAX..RS_EDI] of byte;
    regCounter, lastRemoved: tsuperregister;
    prev, next: tai;
{$ifdef alignregdebug}
    temp: tai;
{$endif alignregdebug}
begin
  regsUsable := [RS_EAX,RS_ECX,RS_EDX,RS_EBX,{R_ESP,RS_EBP,}RS_ESI,RS_EDI];
  for regCounter := RS_EAX to RS_EDI do
    begin
      prevState[regCounter] := ptaiprop(p.optinfo)^.Regs[regCounter].wState;
      nextWState[regCounter] := ptaiprop(p.optinfo)^.Regs[regCounter].wState;
      nextRState[regCounter] := ptaiprop(p.optinfo)^.Regs[regCounter].rState;
    end;
  getLastInstruction(p,prev);
  getNextInstruction(p,next);
  lastRemoved := getsupreg(tai_align(p).reg);
  nextInstrCount := 0;
  prevInstrCount := 0;
  while ((assigned(prev) and
          assigned(prev.optinfo) and
          (prevInstrCount < alignSearch)) or
         (assigned(next) and
          assigned(next.optinfo) and
          (nextInstrCount < alignSearch))) and
        (regsUsable <> []) do
    begin
{$ifdef alignregdebug}
      if assigned(prev) then
        begin
          temp := tai_comment.Create(strpnew('got here'));
          temp.next := prev.next;
          temp.previous := prev;
          prev.next := temp;
          if assigned(temp.next) then
            temp.next.previous := temp;
        end;
{$endif alignregdebug}
      if assigned(prev) and assigned(prev.optinfo) and
         (prevInstrCount < alignSearch) then
        begin
          if (prev.typ = ait_instruction) and
             (insProp[taicpu(prev).opcode].ch[1] <> Ch_ALL) and
             (taicpu(prev).opcode <> A_JMP) then
            begin
              inc(prevInstrCount);
              for regCounter := RS_EAX to RS_EDI do
                begin
                  if (regCounter in regsUsable) and
                     (ptaiprop(prev.optinfo)^.Regs[regCounter].wState <>
                       prevState[regCounter]) then
                    begin
                      lastRemoved := regCounter;
                      exclude(regsUsable,regCounter);
{$ifdef alignregdebug}
                      temp := tai_comment.Create(strpnew(
                                std_regname(newreg(R_INTREGISTER,regCounter,R_SUBWHOLE))+' removed')));
                      temp.next := prev.next;
                      temp.previous := prev;
                      prev.next := temp;
                      if assigned(temp.next) then
                        temp.next.previous := temp;
                      if regsUsable = [] then
                        begin
                          temp := tai_comment.Create(strpnew(
                                    'regsUsable empty here')));
                          temp.next := prev.next;
                          temp.previous := prev;
                          prev.next := temp;
                          if assigned(temp.next) then
                            temp.next.previous := temp;
                        end;
{$endif alignregdebug}
                    end;
                  prevState[regCounter] :=
                    ptaiprop(prev.optinfo)^.Regs[regCounter].wState;
                end;
              getLastInstruction(prev,prev);
            end
          else
            if GetLastInstruction(prev,prev) and
               assigned(prev.optinfo) then
              for regCounter := RS_EAX to RS_EDI do
                prevState[regCounter] :=
                  ptaiprop(prev.optinfo)^.Regs[regCounter].wState
        end;
      if assigned(next) and assigned(next.optinfo) and
         (nextInstrCount < alignSearch) then
        begin
          if (next.typ = ait_instruction) and
             (insProp[taicpu(next).opcode].ch[1] <> Ch_ALL) and
             (taicpu(next).opcode <> A_JMP) then
            begin
              inc(nextInstrCount);
              for regCounter := RS_EAX to RS_EDI do
                begin
                  if (regCounter in regsUsable) and
                     ((ptaiprop(next.optinfo)^.Regs[regCounter].wState <>
                       nextWState[regCounter]) or
                      (ptaiprop(next.optinfo)^.Regs[regCounter].rState <>
                       nextRState[regCounter])) then
                    begin
                      lastRemoved := regCounter;
                      exclude(regsUsable,regCounter);
{$ifdef alignregdebug}
                      temp := tai_comment.Create(strpnew(
                                std_regname(newreg(R_INTREGISTER,regCounter,R_SUBWHOLE))+' removed')));
                      temp.next := next.next;
                      temp.previous := next;
                      next.next := temp;
                      if assigned(temp.next) then
                        temp.next.previous := temp;
                      if regsUsable = [] then
                        begin
                          temp := tai_comment.Create(strpnew(
                                    'regsUsable empty here')));
                          temp.next := next.next;
                          temp.previous := next;
                          next.next := temp;
                          if assigned(temp.next) then
                            temp.next.previous := temp;
                        end;
{$endif alignregdebug}
                    end;
                  nextWState[regCounter] :=
                    ptaiprop(next.optinfo)^.Regs[regCounter].wState;
                  nextRState[regCounter] :=
                    ptaiprop(next.optinfo)^.Regs[regCounter].rState;
                end
            end
          else
            for regCounter := RS_EAX to RS_EDI do
              begin
                nextWState[regCounter] :=
                  ptaiprop(next.optinfo)^.Regs[regCounter].wState;
                nextRState[regCounter] :=
                  ptaiprop(next.optinfo)^.Regs[regCounter].rState;
              end;
          getNextInstruction(next,next);
        end;
    end;
  if regsUsable <> [] then
    for regCounter := RS_EAX to RS_EDI do
      if regCounter in regsUsable then
        begin
          lastRemoved := regCounter;
          break
        end;
{$ifdef alignregdebug}
  next := tai_comment.Create(strpnew(std_regname(newreg(R_INTREGISTER,lastremoved,R_SUBWHOLE))+
               ' chosen as alignment register')));
  next.next := p.next;
  next.previous := p;
  p.next := next;
  if assigned(next.next) then
    next.next.previous := next;
{$endif alignregdebug}
  tai_align(p).reg := newreg(R_INTREGISTER,lastRemoved,R_SUBWHOLE);
end;


procedure clearmemwrites(p: tai; supreg: tsuperregister);
var
  beginmemwrite: tai;
begin
  beginmemwrite := ptaiprop(p.optinfo)^.regs[supreg].memwrite;
  repeat
    ptaiprop(p.optinfo)^.regs[supreg].memwrite := nil;
  until not getnextinstruction(p,p) or
        (ptaiprop(p.optinfo)^.regs[supreg].memwrite <> beginmemwrite);
end;


procedure ClearRegContentsFrom(asml: TAsmList; supreg: tsuperregister; p, endP: tai);
{ first clears the contents of reg from p till endP. then the contents are }
{ cleared until the first instruction that changes reg                     }
var
{$ifdef replaceregdebug}
    hp: tai;
    l: longint;
{$endif replaceregdebug}
    regcounter: tsuperregister;
    oldStartmod: tai;
    regstoclear: tregset;
begin
{$ifdef replaceregdebug}
  l := random(1000);
  hp := tai_comment.Create(strpnew(
          'cleared '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+' from here... '+tostr(l)));
  insertllitem(asml,p.previous,p,hp);
{$endif replaceregdebug}
  ptaiprop(p.optinfo)^.Regs[supreg].typ := con_unknown;
  regstoclear := [supreg];
  while (p <> endP) do
    begin
      for regcounter := RS_EAX to RS_EDI do
        begin
          if (regcounter <> supreg) and
             assigned(ptaiprop(p.optinfo)^.regs[supreg].memwrite) and
             reginref(regcounter,ptaiprop(p.optinfo)^.regs[supreg].memwrite.oper[1]^.ref^) then
            clearmemwrites(p,regcounter);
          { needs double loop to cheack for each dependency combination? }
          if assigned(ptaiprop(p.optinfo)^.regs[regcounter].startmod) and
             sequencedependsonreg(ptaiprop(p.optinfo)^.regs[regcounter],regcounter,supreg) then
            include(regstoclear,regcounter);

          if regcounter in regstoclear then
            with ptaiprop(p.optinfo)^.Regs[regcounter] do
              begin
                typ := con_unknown;
                memwrite := nil;
                startmod := nil;
                nrofmods := 0;
              end;
        end;
      getNextInstruction(p,p);
    end;
  oldStartmod := ptaiprop(p.optinfo)^.Regs[supreg].startmod;
  repeat
    for regcounter := RS_EAX to RS_EDI do
      begin
        { needs double loop to cheack for each dependency combination? }
        if assigned(ptaiprop(p.optinfo)^.regs[regcounter].startmod) and
           sequencedependsonreg(ptaiprop(p.optinfo)^.regs[regcounter],regcounter,supreg) then
          include(regstoclear,regcounter);
        with ptaiprop(p.optinfo)^.Regs[supreg] do
          if regcounter in regstoclear then
            begin
              typ := con_unknown;
              memwrite := nil;
            end;
      end;
  until not getNextInstruction(p,p) or
        (ptaiprop(p.optinfo)^.Regs[supreg].startmod <> oldStartmod);
{$ifdef replaceregdebug}
  if assigned(p) then
    begin
      hp := tai_comment.Create(strpnew(
        'cleared '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+' till here... '+tostr(l)));
      insertllitem(asml,p.previous,p,hp);
    end;
{$endif replaceregdebug}
end;

procedure RestoreRegContentsTo(asml: TAsmList; supreg: tsuperregister; const c: TContent; p, endP: tai);
var
{$ifdef replaceregdebug}
  l: longint;
{$endif replaceregdebug}
  hp: tai;
  validregs, prevvalidregs: tregset;
  regcounter: tsuperregister;
  tmpState, newrstate: byte;
  prevcontenttyp: byte;
  memconflict: boolean;
  invalsmemwrite: boolean;
begin
{$ifdef replaceregdebug}
  l := random(1000);
  hp := tai_comment.Create(strpnew(
          'restored '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+' with data from here... '+tostr(l)));
  insertllitem(asml,p.previous,p,hp);
{$endif replaceregdebug}
{  ptaiprop(p.optinfo)^.Regs[reg] := c;}
  newrstate := c.rstate;
  incstate(newrstate,$7f);
  memconflict := false;
  invalsmemwrite := false;
  validregs := [RS_EAX..RS_EDI];
  prevvalidregs := validregs;
  while (p <> endP) and
        not(memconflict) and
        not(invalsmemwrite) do
    begin
      if not(ptaiprop(p.optinfo)^.canberemoved) and
         regreadbyinstruction(supreg,p) then
        incstate(newrstate,1);
      // is this a write to memory that destroys the contents we are restoring?
      memconflict := modifiesConflictingMemLocation(p,supreg,ptaiprop(p.optinfo)^.regs,validregs,false,invalsmemwrite);
      if (validregs <> prevvalidregs) then
        begin
          prevvalidregs := validregs >< prevvalidregs;
          for regcounter := RS_EAX to RS_EDI do
            if regcounter in prevvalidregs then
              clearRegContentsFrom(asml,regcounter,p,endP);
        end;
      prevvalidregs := validregs;
      if (not memconflict and not invalsmemwrite) then
        begin
          ptaiprop(p.optinfo)^.Regs[supreg] := c;
          ptaiprop(p.optinfo)^.Regs[supreg].rstate := newrstate;
        end
      else
        begin
          clearRegContentsFrom(asml,supreg,p,endP);
{$ifdef replaceregdebug}
           if assigned(p) then
             begin
               hp := tai_comment.Create(strpnew(
                 'stopping restoring of '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+'because memory conflict... '+tostr(l)));
               insertllitem(asml,p,p.next,hp);
             end;
{$endif replaceregdebug}
          exit
        end;

      getNextInstruction(p,p);
    end;

  tmpState := ptaiprop(p.optinfo)^.Regs[supreg].wState;
  if (newrstate = ptaiprop(p.optinfo)^.Regs[supreg].rState) then
    begin
      incstate(ptaiprop(p.optinfo)^.regs[supreg].rstate,63);
      if not getnextinstruction(p,hp) then
        exit;
      if (ptaiprop(hp.optinfo)^.regs[supreg].rstate = ptaiprop(p.optinfo)^.regs[supreg].rstate) then
        internalerror(2004122710);
     end;
  repeat
    newrstate := ptaiprop(p.optinfo)^.Regs[supreg].rState;
    prevcontenttyp := ptaiprop(p.optinfo)^.Regs[supreg].typ;
    // is this a write to memory that destroys the contents we are restoring?
    memconflict := modifiesConflictingMemLocation(p,supreg,ptaiprop(p.optinfo)^.regs,validregs,false,invalsmemwrite);
    if (validregs <> prevvalidregs) then
      begin
        prevvalidregs := validregs >< prevvalidregs;
        for regcounter := RS_EAX to RS_EDI do
          if regcounter in prevvalidregs then
            clearRegContentsFrom(asml,regcounter,p,p);
      end;
    prevvalidregs := validregs;
    if (not memconflict and not invalsmemwrite) then
      begin
        ptaiprop(p.optinfo)^.Regs[supreg] := c;
        ptaiprop(p.optinfo)^.Regs[supreg].rstate := newrstate;
      end;
  until invalsmemwrite or
        memconflict or
        not getNextInstruction(p,p) or
        (ptaiprop(p.optinfo)^.Regs[supreg].wState <> tmpState) or
        (p.typ = ait_label) or
        ((prevcontenttyp <> con_invalid) and
         (ptaiprop(p.optinfo)^.Regs[supreg].typ = con_invalid));
  if assigned(p) and
     (p.typ <> ait_marker) then
    if ((p.typ = ait_label) or
       memconflict or
       invalsmemwrite) then
      clearRegContentsFrom(asml,supreg,p,p)
    else if (ptaiprop(p.optinfo)^.Regs[supreg].rstate = newrstate) then
      incstate(ptaiprop(p.optinfo)^.Regs[supreg].rstate,20);
{$ifdef replaceregdebug}
  if assigned(p) then
    begin
      hp := tai_comment.Create(strpnew(
        'restored '+std_regname(newreg(R_INTREGISTER,supreg,R_SUBWHOLE))+' till here... '+tostr(l)));
     insertllitem(asml,p,p.next,hp);
    end;
{$endif replaceregdebug}
end;

function NoHardCodedRegs(p: taicpu; orgReg, newReg: tsuperregister): boolean;
var
  chCount: byte;
begin
  case p.opcode of
    A_IMUL: noHardCodedRegs := p.ops <> 1;
    A_SHL,A_SHR,A_SHLD,A_SHRD: noHardCodedRegs :=
      (p.oper[0]^.typ <> top_reg) or
      ((orgReg <> RS_ECX) and (newReg <> RS_ECX));
    else
      begin
        NoHardCodedRegs := true;
        with InsProp[p.opcode] do
          for chCount := 1 to maxinschanges do
            if Ch[chCount] in ([Ch_REAX..Ch_MEDI,Ch_WMemEDI,Ch_All]-[Ch_RESP,Ch_WESP,Ch_RWESP]) then
              begin
                NoHardCodedRegs := false;
                break
              end;
      end;
  end;
end;


function ChangeReg(var Reg: TRegister; newReg, orgReg: tsuperregister): boolean;
begin
  changereg := false;
  if (reg <> NR_NO) and
     (getregtype(reg) = R_INTREGISTER) and
     (getsupreg(reg) = newreg) then
    begin
      changereg := true;
      setsupreg(reg,orgreg);
    end;
end;


function changeOp(var o: toper; newReg, orgReg: tsuperregister): boolean;
var
  tmpresult: boolean;
begin
  changeOp := false;
  case o.typ of
    top_reg: changeOp := changeReg(o.reg,newReg,orgReg);
    top_ref:
      begin
        tmpresult := changeReg(o.ref^.base,newReg,orgReg);
        changeop := changeReg(o.ref^.index,newReg,orgReg) or tmpresult;
      end;
  end;
end;


procedure updateStates(orgReg,newReg: tsuperregister; hp: tai; writeStateToo: boolean);
var
  prev: tai;
  newOrgRegRState, newOrgRegWState: byte;
begin
  if getLastInstruction(hp,prev) then
    with ptaiprop(prev.optinfo)^ do
      begin
        newOrgRegRState := byte(longint(regs[orgReg].rState) +
          longint(ptaiprop(hp.optinfo)^.regs[newReg].rState) - regs[newReg].rstate);
        if writeStateToo then
          newOrgRegWState := byte(longint(regs[orgReg].wState) +
            longint(ptaiprop(hp.optinfo)^.regs[newReg].wState) - regs[newReg].wstate);
      end
  else
    with ptaiprop(hp.optinfo)^.regs[newReg] do
      begin
        newOrgRegRState := rState;
        if writeStateToo then
          newOrgRegWState := wState;
      end;
  with ptaiprop(hp.optinfo)^.regs[orgReg] do
    begin
      rState := newOrgRegRState;
      if writeStateToo then
        wState := newOrgRegwState;
    end;
end;


function doReplaceReg(hp: taicpu; newReg, orgReg: tsuperregister): boolean;
var
  opCount: longint;
  tmpResult: boolean;
begin
  tmpresult := false;
  for opCount := 0 to hp.ops-1 do
    tmpResult :=
      changeOp(hp.oper[opCount]^,newReg,orgReg) or tmpResult;
  doReplaceReg := tmpResult;
end;


function RegSizesOK(oldReg,newReg: tsuperregister; p: taicpu): boolean;
{ oldreg and newreg must be 32bit components }
var
  opCount: longint;
  tmpreg: tsuperregister;
begin
  RegSizesOK := true;
  { if only one of them is a general purpose register ... }
  if (IsGP32reg(oldReg) xor IsGP32Reg(newReg)) then
    begin
      for opCount := 0 to p.ops-1 do
        if (p.oper[opCount]^.typ = top_reg) and
           (getsubreg(p.oper[opCount]^.reg) in [R_SUBL,R_SUBH]) then
          begin
            tmpreg := getsupreg(p.oper[opCount]^.reg);
            if (tmpreg = oldreg) or
               (tmpreg = newreg) then
              begin
                RegSizesOK := false;
                break
              end
          end;
    end;
end;


function doReplaceReadReg(p: taicpu; newReg,orgReg: tsuperregister): boolean;
var
  opCount: longint;
begin
  doReplaceReadReg := false;
  { handle special case }
  case p.opcode of
    A_IMUL:
      begin
        case p.ops of
          1: internalerror(1301001);
          2,3:
            begin
              if changeOp(p.oper[0]^,newReg,orgReg) then
                begin
{                  updateStates(orgReg,newReg,p,false);}
                  doReplaceReadReg := true;
                end;
             if p.ops = 3 then
                if changeOp(p.oper[1]^,newReg,orgReg) then
                  begin
{                    updateStates(orgReg,newReg,p,false);}
                    doReplaceReadReg := true;
                  end;
            end;
        end;
      end;
    A_DIV,A_IDIV,A_MUL: internalerror(1301002);
    else
      begin
        for opCount := 0 to p.ops-1 do
          if p.oper[opCount]^.typ = top_ref then
            if changeOp(p.oper[opCount]^,newReg,orgReg) then
              begin
{                updateStates(orgReg,newReg,p,false);}
                doReplaceReadReg := true;
              end;
        for opCount := 1 to maxinschanges do
          case InsProp[p.opcode].Ch[opCount] of
            Ch_ROp1:
              if p.oper[0]^.typ = top_reg then
                if changeReg(p.oper[0]^.reg,newReg,orgReg) then
                  begin
{                    updateStates(orgReg,newReg,p,false);}
                    doReplaceReadReg := true;
                  end;
            Ch_ROp2:
              if p.oper[1]^.typ = top_reg then
                if changeReg(p.oper[1]^.reg,newReg,orgReg) then
                  begin
{                    updateStates(orgReg,newReg,p,false);}
                    doReplaceReadReg := true;
                  end;
            Ch_ROp3:
              if p.oper[2]^.typ = top_reg then
                if changeReg(p.oper[2]^.reg,newReg,orgReg) then
                  begin
{                    updateStates(orgReg,newReg,p,false);}
                    doReplaceReadReg := true;
                  end;
          end;
      end;
  end;
end;


procedure updateState(supreg: tsuperregister; p: tai);
{ this procedure updates the read and write states of the instructions }
{ coming after p. It's called when the read/write state of p has been  }
{ changed and this change has to be propagated to the following        }
{ instructions as well                                                 }
var
  newRState, newWState: byte;
  prevRState, prevWState: byte;
  doRState, doWState: boolean;
begin
  { get the new read/write states from p }
  with ptaiprop(p.optinfo)^.regs[supreg] do
    begin
      newRState := rState;
      newWState := wState;
    end;
  if not GetNextInstruction(p,p) then
    exit;
  { get the old read/write states from the next instruction, to know }
  { when we can stop updating                                        }
  with ptaiprop(p.optinfo)^.regs[supreg] do
    begin
      prevRState := rState;
      prevWState := wState;
    end;
  { adjust the states if this next instruction reads/writes the register }
  if regReadByInstruction(supreg,p) then
    incState(newRState,1);
  if regModifiedByInstruction(supreg,p) then
    incState(newWState,1);
  { do we still have to update the read and/or write states? }
  doRState := true;
  doWState := true;
  repeat
    { update the states }
    with ptaiprop(p.optinfo)^.regs[supreg] do
      begin
        if doRState then
          rState := newRState;
        if doWState then
          wState := newWState;
      end;
    if not getNextInstruction(p,p) then
      break;
    with ptaiprop(p.optinfo)^.regs[supreg] do
      begin
        { stop updating the read state if it changes }
        doRState :=
          doRState and (rState = prevRState);
        { if, by accident, this changed state is the same as the one }
        { we've been using, change it to a value that's definitely   }
        { different from the previous and next state                 }
        if not doRState and
           (rState = newRState) then
          begin
            incState(newRState,1);
            prevRState := rState;
            doRState := true;
          end;
        { ditto for the write state }
        doWState :=
          doWState and (WState = prevWState);
        if not doWState and
           (wState = newWState) then
          begin
            incState(newWState,1);
            prevWState := wState;
            doWState := true;
          end;
      end;
  { stop when we don't have to update either state anymore }
  until not(doRState or doWState);
end;


function storeBack(start, current: tai; orgReg, newReg: tsuperregister): boolean;
{ returns true if p1 contains an instruction that stores the contents }
{ of newReg back to orgReg                                            }
begin
  storeback := false;
  if (current.typ = ait_instruction) and
     (taicpu(current).opcode = A_MOV) and
     (taicpu(current).oper[0]^.typ = top_reg) and
     (getsupreg(taicpu(current).oper[0]^.reg) = newReg) and
     (taicpu(current).oper[1]^.typ = top_reg) and
     (getsupreg(taicpu(current).oper[1]^.reg) = orgReg) then
    case taicpu(current).opsize of
      S_B:
        storeback := true;
      S_W:
        storeback := taicpu(start).opsize <> S_B;
      S_L:
        storeback := taicpu(start).opsize = S_L;
      else
        internalerror(2003121501);
    end;
end;


function canreplacereg(orgsupreg, newsupreg: tsuperregister; p: tai;
  orgRegCanBeModified: boolean; var resnewregmodified, resorgregread, resremovelast: boolean; var returnendp: tai): boolean;
var
  endP, hp: tai;
  removeLast, sequenceEnd, tmpResult, newRegModified, orgRegRead: boolean;
begin
  canreplacereg := false;
  tmpResult := true;
  sequenceEnd := false;
  newRegModified := false;
  orgRegRead := false;
  removeLast := false;
  endP := p;
  while tmpResult and not sequenceEnd do
    begin
      tmpResult :=
        getNextInstruction(endP,endP) and
        (endp.typ = ait_instruction) and
        not(taicpu(endp).is_jmp);
      if tmpresult and not assigned(endp.optinfo) then
        begin
{          hp := tai_comment.Create(strpnew('next no optinfo'));
          hp.next := endp;
          hp.previous := endp.previous;
          endp.previous := hp;
          if assigned(hp.previous) then
            hp.previous.next := hp;}
          exit;
        end;
      if tmpResult and
         { don't take into account instructions that will be removed }
         not (ptaiprop(endp.optinfo)^.canBeRemoved) then
        begin
          { if the newsupreg gets stored back to the oldReg, we can change }
          { "mov %oldReg,%newReg; <operations on %newReg>; mov %newReg, }
          { %oldReg" to "<operations on %oldReg>"                       }
          removeLast := storeBack(p,endP, orgsupreg, newsupreg);
          sequenceEnd :=
            { no support for (i)div, mul and imul with hardcoded operands }
            noHardCodedRegs(taicpu(endP),orgsupreg,newsupreg) and
            { if newsupreg gets loaded with a new value, we can stop   }
            { replacing newsupreg with oldReg here (possibly keeping   }
            { the original contents of oldReg so we still know them }
            { afterwards)                                           }
             (RegLoadedWithNewValue(newsupreg,true,taicpu(endP)) or
            { we can also stop if we reached the end of the use of }
            { newReg's current contents                            }
              (GetNextInstruction(endp,hp) and
               FindRegDealloc(newsupreg,hp)));
          { to be able to remove the first and last instruction of  }
          {   movl %reg1, %reg2                                     }
          {   <operations on %reg2> (replacing reg2 with reg1 here) }
          {   movl %reg2, %reg1                                     }
          { %reg2 must not be use afterwards (it can be as the      }
          { result of a peepholeoptimization)                       }
          removeLast := removeLast and sequenceEnd;
          newRegModified :=
            newRegModified or
            (not(regLoadedWithNewValue(newsupreg,true,taicpu(endP))) and
             RegModifiedByInstruction(newsupreg,endP));
          orgRegRead := newRegModified and RegReadByInstruction(orgsupreg,endP);
          sequenceEnd := SequenceEnd and
                         (removeLast or
    { since newsupreg will be replaced by orgsupreg, we can't allow that newsupreg }
    { gets modified if orgsupreg is still read afterwards (since after       }
    { replacing, this would mean that orgsupreg first gets modified and then }
    { gets read in the assumption it still contains the unmodified value) }
                         not(newRegModified and orgRegRead)) (* and
    { since newsupreg will be replaced by orgsupreg, we can't allow that newsupreg }
    { gets modified if orgRegCanBeModified = false                        }

    { this now gets checked after the loop (JM) }
                         (orgRegCanBeModified or not(newRegModified)) *);
          tmpResult :=
            not(removeLast) and
            not(newRegModified and orgRegRead) and
(*            (orgRegCanBeModified or not(newRegModified)) and *)
(*          already checked at the top
            (endp.typ = ait_instruction) and  *)
            NoHardCodedRegs(taicpu(endP),orgsupreg,newsupreg) and
            RegSizesOk(orgsupreg,newsupreg,taicpu(endP)) and
            not RegModifiedByInstruction(orgsupreg,endP);
        end;
    end;
  canreplacereg := sequenceEnd and
     (removeLast  or
      (orgRegCanBeModified or not(newRegModified))) and
     (not(assigned(endp)) or
      not(endp.typ = ait_instruction) or
      (noHardCodedRegs(taicpu(endP),orgsupreg,newsupreg) and
       RegSizesOk(orgsupreg,newsupreg,taicpu(endP)) and
       not(newRegModified and
           (orgsupreg in ptaiprop(endp.optinfo)^.usedRegs) and
           not(RegLoadedWithNewValue(orgsupreg,true,taicpu(endP))))));
  if canreplacereg then
    begin
      resnewregmodified := newregmodified;
      resorgregread := orgregread;
      resremovelast := removelast;
    end;
  { needed for replaceregdebug code }
  returnendp := endp;
end;



function ReplaceReg(asml: TAsmList; orgsupreg, newsupreg: tsuperregister; p,
          seqstart: tai; const c: TContent; orgRegCanBeModified: Boolean;
          var returnEndP: tai): Boolean;
{ Tries to replace orgsupreg with newsupreg in all instructions coming after p }
{ until orgsupreg gets loaded with a new value. Returns true if successful, }
{ false otherwise. if successful, the contents of newsupreg are set to c,   }
{ which should hold the contents of newsupreg before the current sequence   }
{ started                                                                }
{ if the function returns true, returnEndP holds the last instruction    }
{ where newsupreg was replaced by orgsupreg                                    }
var
  endP, hp: tai;
  removeLast, sequenceEnd, newRegModified, orgRegRead,
    stateChanged, readStateChanged: Boolean;
{$ifdef replaceregdebug}
  l: longint;
{$endif replaceregdebug}

begin
  replacereg := false;
  if canreplacereg(orgsupreg,newsupreg,p,orgregcanbemodified,newregmodified, orgregread, removelast,endp) then
    begin
{$ifdef replaceregdebug}
      l := random(1000);
      hp := tai_comment.Create(strpnew(
        'replacing '+std_regname(newreg(R_INTREGISTER,newsupreg,R_SUBWHOLE))+' with '+std_regname(newreg(R_INTREGISTER,orgsupreg,R_SUBWHOLE))+
        ' from here... '+tostr(l)));
      insertllitem(asml,p.previous,p,hp);
      hp := tai_comment.Create(strpnew(
        'replaced '+std_regname(newreg(R_INTREGISTER,newsupreg,R_SUBWHOLE))+' with '+std_regname(newreg(R_INTREGISTER,orgsupreg,R_SUBWHOLE))+
        ' till here ' + tostr(l)));
      insertllitem(asml,endp,endp.next,hp);
{$endif replaceregdebug}
      replaceReg := true;
      returnEndP := endP;

      if not getNextInstruction(p,hp) then
        exit;
      stateChanged := false;
      while hp <> endP do
        begin
          if {not(ptaiprop(hp.optinfo)^.canBeRemoved) and }
             (hp.typ = ait_instruction) then
            stateChanged :=
              doReplaceReg(taicpu(hp),newsupreg,orgsupreg) or stateChanged;
            if stateChanged then
              updateStates(orgsupreg,newsupreg,hp,true);
          getNextInstruction(hp,hp)
        end;
      if assigned(endp) and (endp.typ = ait_instruction) then
        readStateChanged :=
          doReplaceReadReg(taicpu(endP),newsupreg,orgsupreg);
      if stateChanged or readStateChanged then
        updateStates(orgsupreg,newsupreg,endP,stateChanged);

      if stateChanged or readStateChanged then
        updateState(orgsupreg,endP);

{ We replaced newreg with oldreg between p and endp, so restore the contents }
{ of newreg there with its contents from before the sequence.                }
      if removeLast or
         RegLoadedWithNewValue(newsupreg,true,endP) then
        GetLastInstruction(endP,hp)
      else hp := endP;
      RestoreRegContentsTo(asml,newsupreg,c,seqstart,hp);

{ Ot is possible that the new register was modified (e.g. an add/sub), so if  }
{ it was replaced by oldreg in that instruction, oldreg's contents have been  }
{ changed. To take this into account, we simply set the contents of orgsupreg }
{ to "unknown" after this sequence                                            }
      if newRegModified then
        ClearRegContentsFrom(asml,orgsupreg,p,hp);
      if removeLast then
        ptaiprop(endp.optinfo)^.canBeRemoved := true;
      allocRegBetween(asml,newreg(R_INTREGISTER,orgsupreg,R_SUBWHOLE),p,endP,ptaiprop(p.optinfo)^.usedregs);

    end
{$ifdef replaceregdebug}
     else
       begin
         l := random(1000);
         hp := tai_comment.Create(strpnew(
           'replacing '+std_regname(newreg(R_INTREGISTER,newsupreg,R_SUBWHOLE))+' with '+std_regname(newreg(R_INTREGISTER,orgsupreg,R_SUBWHOLE))+
           ' from here... '+ tostr(l)));
         insertllitem(asml,p.previous,p,hp);
        hp := tai_comment.Create(strpnew(
          'replacing '+std_regname(newreg(R_INTREGISTER,newsupreg,R_SUBWHOLE))+' with '+std_regname(newreg(R_INTREGISTER,orgsupreg,R_SUBWHOLE))+
          ' failed here ' + tostr(l)));
        insertllitem(asml,endp,endp.next,hp);
      end;
{$endif replaceregdebug}
end;


function FindRegWithConst(p: tai; size: topsize; l: aint; var Res: TRegister): Boolean;
{Finds a register which contains the constant l}
var
  Counter: tsuperregister;
{$ifdef testing}
    hp: tai;
{$endif testing}
begin
  Result:=false;
  Counter := RS_EAX;
  repeat
{$ifdef testing}
     if (ptaiprop(p.optinfo)^.regs[counter].typ in [con_const,con_noRemoveConst]) then
       begin
         hp := tai_comment.Create(strpnew(
           'checking const load of '+tostr(l)+' here...'));
         hp.next := ptaiprop(p.optinfo)^.Regs[Counter].StartMod;
         hp.previous := ptaiprop(p.optinfo)^.Regs[Counter].StartMod^.previous;
         ptaiprop(p.optinfo)^.Regs[Counter].StartMod^.previous := hp;
         if assigned(hp.previous) then
           hp.previous.next := hp;
       end;
{$endif testing}
     if (ptaiprop(p.optinfo)^.regs[counter].typ in [con_const,con_noRemoveConst]) and
        (taicpu(ptaiprop(p.optinfo)^.Regs[Counter].StartMod).opsize = size) and
        (taicpu(ptaiprop(p.optinfo)^.Regs[Counter].StartMod).oper[0]^.typ = top_const) and
        (taicpu(ptaiprop(p.optinfo)^.Regs[Counter].StartMod).oper[0]^.val = l) then
       begin
         res:=taicpu(ptaiprop(p.optinfo)^.Regs[Counter].StartMod).oper[1]^.reg;
         result:=true;
         exit;
       end;
     inc(counter);
  until (Counter > RS_EDI);
end;


procedure removePrevNotUsedLoad(asml: TAsmList; p: tai; supreg: tsuperregister; check: boolean);
{ if check = true, it means the procedure has to check whether it isn't  }
{ possible that the contents are still used after p (used when removing  }
{ instructions because of a "call"), otherwise this is not necessary     }
{ (e.g. when you have a "mov 8(%ebp),%eax", you can be sure the previous }
{ value of %eax isn't used anymore later on)                             }
var
  hp1, next, beforestartmod: tai;
begin
  if getLastInstruction(p,hp1) then
    with ptaiprop(hp1.optinfo)^.regs[supreg] do
      if (typ in [con_ref,con_invalid,con_const]) and
         (nrofMods = 1) and
         (rState = ptaiprop(startmod.optinfo)^.regs[supreg].rState) and
         (not(check) or
          (not(regInInstruction(supreg,p)) and
           (not(supreg in ptaiprop(hp1.optinfo)^.usedRegs) or
            findRegDealloc(supreg,p)))) then
        begin
          ptaiprop(startMod.optinfo)^.canBeRemoved := true;
          getnextinstruction(p,next);
          { give the register that was modified by this instruction again }
          { the contents it had before this instruction                   }
          if getlastinstruction(startmod,beforestartmod) then
            RestoreRegContentsTo(asml,supreg,ptaiprop(beforestartmod.optinfo)^.regs[supreg],
             startmod,hp1)
          else
            ClearRegContentsFrom(asml,supreg,startmod,hp1);
        end;
end;


{$ifdef notused}
function is_mov_for_div(p: taicpu): boolean;
begin
  is_mov_for_div :=
    (p.opcode = A_MOV) and
    (p.oper[0]^.typ = top_const) and
    (p.oper[1]^.typ = top_reg) and
    (p.oper[1]^.reg = RS_EDX) and
    getNextInstruction(p,p) and
    (p.typ = ait_instruction) and
    ((p.opcode = A_DIV) or
     (p.opcode = A_IDIV));
end;
{$endif notused}


function memtoreg(t: taicpu; const ref: treference; var startp: tai): tregister;
var
  hp: tai;
  p: ptaiprop;
  regcounter: tsuperregister;
  optimizable: boolean;
begin
  if not getlastinstruction(t,hp) or
     not issimplememloc(ref) then
    begin
      memtoreg := NR_NO;
      exit;
    end;
  p := ptaiprop(hp.optinfo);
  optimizable := false;
  for regcounter := RS_EAX to RS_EDI do
    begin
      if (assigned(p^.regs[regcounter].memwrite) and
         refsequal(ref,p^.regs[regcounter].memwrite.oper[1]^.ref^)) then
        begin
          optimizable := true;
          hp := p^.regs[regcounter].memwrite;
        end
      else if ((p^.regs[regcounter].typ in [CON_REF,CON_NOREMOVEREF]) and
             (p^.regs[regcounter].nrofmods = 1) and
             ((taicpu(p^.regs[regcounter].startmod).opcode = A_MOV) or
              (taicpu(p^.regs[regcounter].startmod).opcode = A_MOVZX) or
              (taicpu(p^.regs[regcounter].startmod).opcode = A_MOVSX)) and
             (taicpu(p^.regs[regcounter].startmod).oper[0]^.typ = top_ref) and
             refsequal(ref,taicpu(p^.regs[regcounter].startmod).oper[0]^.ref^)) then
        begin
          optimizable := true;
          hp := p^.regs[regcounter].startmod;
        end;
      if optimizable then
        if ((t.opsize <> S_B) or
            not(regcounter in [RS_ESI,RS_EDI])) and
            sizescompatible(taicpu(hp).opsize,t.opsize) then
          begin
            case t.opsize of
              S_B:
                begin
                  memtoreg := newreg(R_INTREGISTER,regcounter,R_SUBL)
                end;
              S_W,S_BW:
                begin
                  memtoreg := newreg(R_INTREGISTER,regcounter,R_SUBW);
                  if (t.opsize = S_BW) then
                      begin
                        t.opcode := A_MOV;
                        t.opsize := S_W;
                      end;
                end;
              S_L,S_BL,S_WL:
                begin
                  memtoreg := newreg(R_INTREGISTER,regcounter,R_SUBWHOLE);
                  if (t.opsize <> S_L) then
                    begin
                      t.opcode := A_MOV;
                      t.opsize := S_L;
                    end;
                end;
            end;
            startp := hp;
            exit;
          end;
    end;
  memtoreg := NR_NO;
end;


procedure removeLocalStores(const t1: tai);
{var
  p: tai;
  regcount: tregister; }
begin
{
  for regcount := LoGPReg to HiGPReg do
    if assigned(pTaiProp(t1.optinfo)^.regs[regcount].memwrite) and
       (taicpu(pTaiProp(t1.optinfo)^.regs[regcount].memwrite).oper[1]^.ref^.base
         = current_procinfo.framepointer) then
      begin
        pTaiProp(pTaiProp(t1.optinfo)^.regs[regcount].memwrite.optinfo)^.canberemoved := true;
        clearmemwrites(pTaiProp(t1.optinfo)^.regs[regcount].memwrite,regcount);
      end;
}
end;


procedure loadcseregs(asml: TAsmList; const reginfo: toptreginfo; curseqend, prevseqstart, curseqstart, curprev: tai; cnt: longint);
var
  regsloaded: tregset;
  regloads, reguses: array[RS_EAX..RS_EDI] of tai;
  regcounter, substreg: tsuperregister;
  hp, hp2: tai;
  insertpos, insertoptinfo, prevseq_next: tai;
  i: longint;
  opc: tasmop;
begin
  regsloaded := [];
  fillchar(regloads,sizeof(regloads),0);
  fillchar(reguses,sizeof(reguses),0);
  getnextinstruction(prevseqstart,prevseq_next);
  for regcounter := RS_EAX To RS_EDI do
    if (reginfo.new2oldreg[regcounter] <> RS_INVALID) Then
      begin
        include(regsloaded,regcounter);
        if assigned(ptaiprop(prevseqstart.optinfo)^.Regs[reginfo.new2oldreg[regcounter]].StartMod) then
          AllocRegBetween(asml,newreg(R_INTREGISTER,reginfo.new2oldreg[regcounter],R_SUBWHOLE),
            ptaiprop(prevseqstart.optinfo)^.Regs[reginfo.new2oldreg[regcounter]].StartMod,curseqstart,
            ptaiprop(ptaiprop(prevseqstart.optinfo)^.Regs[reginfo.new2oldreg[regcounter]].StartMod.optinfo)^.usedregs)
        else
          AllocRegBetween(asml,newreg(R_INTREGISTER,reginfo.new2oldreg[regcounter],R_SUBWHOLE),
            prevseqstart,curseqstart,ptaiprop(prevseqstart.optinfo)^.usedregs);

        if curprev <> prevseqstart then
          begin
            if assigned(reginfo.lastReload[regCounter]) then
              getLastInstruction(reginfo.lastReload[regCounter],hp)
            else if assigned(reginfo.lastReload[reginfo.new2oldreg[regCounter]]) then
              getLastInstruction(reginfo.lastReload[reginfo.new2OldReg[regCounter]],hp)
            else
              hp := curprev;
            clearRegContentsFrom(asml,regCounter,prevSeq_next,hp);
            getnextInstruction(hp,hp);
            allocRegBetween(asml,newreg(R_INTREGISTER,regCounter,R_SUBWHOLE),prevseqstart,hp,
              ptaiprop(prevseqstart.optinfo)^.usedregs);
          end;
        if not(regcounter in reginfo.RegsLoadedforRef) and
                      {old reg                new reg}
            (reginfo.new2oldreg[regcounter] <> regcounter) then
          begin
            getLastInstruction(curseqend,hp);
            opc := A_MOV;
            insertpos := prevseq_next;
            insertoptinfo := prevseqstart;
            if assigned(reguses[regcounter]) then
              if assigned(regloads[reginfo.new2oldreg[regcounter]]) then
                opc := A_XCHG
              else
                begin
                  insertoptinfo := reguses[regcounter];
                  insertpos := tai(insertoptinfo.next)
                end
            else
              if assigned(regloads[reginfo.new2oldreg[regcounter]]) then
                 begin
                   insertpos := regloads[reginfo.new2oldreg[regcounter]];
                   if not getlastinstruction(insertpos,insertoptinfo) then
                     internalerror(2006060701);
                 end;
            hp := Tai_Marker.Create(mark_NoPropInfoStart);
            InsertLLItem(asml, insertpos.previous,insertpos, hp);
            hp2 := taicpu.Op_Reg_Reg(opc, S_L,
                                            {old reg                                        new reg}
                     newreg(R_INTREGISTER,reginfo.new2oldreg[regcounter],R_SUBWHOLE), newreg(R_INTREGISTER,regcounter,R_SUBWHOLE));
            if (opc = A_XCHG) and
               (taicpu(regloads[reginfo.new2oldreg[regcounter]]).opcode <> A_XCHG) then
              begin
                asml.remove(regloads[reginfo.new2oldreg[regcounter]]);
                regloads[reginfo.new2oldreg[regcounter]].free;
                regloads[reginfo.new2oldreg[regcounter]] := hp2;
                reguses[regcounter] := hp2;
              end;
            regloads[regcounter] := hp2;
            reguses[reginfo.new2oldreg[regcounter]] := hp2;
            new(ptaiprop(hp2.optinfo));
            ptaiprop(hp2.optinfo)^ := ptaiprop(insertoptinfo.optinfo)^;
            ptaiprop(hp2.optinfo)^.canBeRemoved := false;
            InsertLLItem(asml, insertpos.previous, insertpos, hp2);
            hp := Tai_Marker.Create(mark_NoPropInfoEnd);
            InsertLLItem(asml, insertpos.previous, insertpos, hp);
            { adjusts states in previous instruction so that it will  }
            { definitely be different from the previous or next state }
            incstate(ptaiprop(hp2.optinfo)^.
              regs[reginfo.new2oldreg[regcounter]].rstate,20);
            incstate(ptaiprop(hp2.optinfo)^.
              regs[regCounter].wstate,20);
            updateState(reginfo.new2oldreg[regcounter],hp2);
            updateState(regcounter,hp2);
          end
        else
  {   imagine the following code:                                            }
  {        normal                    wrong optimized                         }
  {    movl 8(%ebp), %eax           movl 8(%ebp), %eax                       }
  {    movl (%eax), %eax            movl (%eax), %eax                        }
  {    cmpl 8(%ebp), %eax           cmpl 8(%ebp), %eax                       }
  {    jne l1                       jne l1                                   }
  {    movl 8(%ebp), %eax                                                    }
  {    movl (%eax), %edi            movl %eax, %edi                          }
  {    movl %edi, -4(%ebp)          movl %edi, -4(%ebp)                      }
  {    movl 8(%ebp), %eax                                                    }
  {    pushl 70(%eax)               pushl 70(%eax)                           }
  {                                                                          }
  {   The error is that at the moment that the last instruction is executed, }
  {   %eax doesn't contain 8(%ebp) anymore. Solution: the contents of        }
  {   registers that are completely removed from a sequence (= registers in  }
  {   RegLoadedforRef), have to be changed to their contents from before the }
  {   sequence.                                                              }
        { if regcounter in reginfo.RegsLoadedforRef then }
          begin
            hp := curseqstart;
            { cnt still holds the number of instructions }
            { of the sequence, so go to the end of it    }
            for i := 1 to pred(cnt) do
              getNextInstruction(hp,hp);
            { curprev = instruction prior to start of sequence }
            restoreRegContentsTo(asml,regCounter,
              ptaiprop(curprev.optinfo)^.Regs[regcounter],
              curseqstart,hp);
          end;
      end;
end;


procedure replaceoperandwithreg(asml: TAsmList; p: tai; opnr: byte; reg: tregister);
var
  hp: tai;
begin
  { new instruction -> it's info block is not in the big one allocated at the start }
  hp := Tai_Marker.Create(mark_NoPropInfoStart);
  InsertLLItem(asml, p.previous,p, hp);
  { duplicate the original instruction and replace it's designated operant with the register }
  hp := tai(p.getcopy);
  taicpu(hp).loadreg(opnr,reg);
  { add optimizer state info }
  new(ptaiprop(hp.optinfo));
  ptaiprop(hp.optinfo)^ := ptaiprop(p.optinfo)^;
  { new instruction can not be removed }
  ptaiprop(hp.optinfo)^.canBeRemoved := false;
  { but the old one can }
  ptaiprop(p.optinfo)^.canBeRemoved := true;
  { insert end marker }
  InsertLLItem(asml, p.previous, p, hp);
  hp := Tai_Marker.Create(mark_NoPropInfoEnd);
  InsertLLItem(asml, p.previous, p, hp);
end;


procedure doCSE(asml: TAsmList; First, Last: tai; findPrevSeqs, doSubOpts: boolean);
{marks the instructions that can be removed by RemoveInstructs. They're not
 removed immediately because sometimes an instruction needs to be checked in
 two different sequences}
var cnt, cnt2, {cnt3,} orgNrofMods: longint;
    p, hp1, hp2, prevSeq: tai;
    hp3, hp4: tai;
    hp5 : tai;
    reginfo: toptreginfo;
    memreg: tregister;
    regcounter: tsuperregister;
begin
  p := First;
  SkipHead(p);
  while (p <> Last) do
    begin
      case p.typ of
        ait_align:
          if not(tai_align(p).use_op) then
            SetAlignReg(p);
        ait_instruction:
          begin
            case taicpu(p).opcode of
{
     Does not work anymore with register calling because the registers are
     released before the call
              A_CALL:
                for regCounter := RS_EAX to RS_EBX do
                  removePrevNotUsedLoad(asml,p,regCounter,true);
}
              A_CLD: if GetLastInstruction(p, hp1) and
                        (ptaiprop(hp1.optinfo)^.DirFlag = F_NotSet) then
                       ptaiprop(tai(p).optinfo)^.CanBeRemoved := True;
              A_LEA, A_MOV, A_MOVZX, A_MOVSX:
                begin
                  hp2 := p;
                  case taicpu(p).oper[0]^.typ of
                    top_ref, top_reg:
                     if (taicpu(p).oper[1]^.typ = top_reg) then
                       begin
                        With ptaiprop(p.optinfo)^.Regs[getsupreg(taicpu(p).oper[1]^.reg)] do
                          begin
                            if (startmod = p) then
                              orgNrofMods := nrofMods
                            else
                              orgNrofMods := 0;
                            if (p = StartMod) and
                               GetLastInstruction (p, hp1) and
                               not(hp1.typ in [ait_marker,ait_label]) then
{so we don't try to check a sequence when p is the first instruction of the block}
                              begin
{$ifdef csdebug}
                               hp5 := tai_comment.Create(strpnew(
                                 'cse checking '+std_regname(taicpu(p).oper[1]^.reg)));
                               insertLLItem(asml,p,p.next,hp5);
{$endif csdebug}
                               if CheckSequence(p,prevSeq,getsupreg(taicpu(p).oper[1]^.reg), Cnt, reginfo, findPrevSeqs) and
                                  (Cnt > 0) then
                                 begin
(*
                                   hp1 := nil;
{ although it's perfectly ok to remove an instruction which doesn't contain }
{ the register that we've just checked (CheckSequence takes care of that),  }
{ the sequence containing this other register should also be completely     }
{ checked and removed, otherwise we may get situations like this:           }
{                                                                           }
{   movl 12(%ebp), %edx                       movl 12(%ebp), %edx           }
{   movl 16(%ebp), %eax                       movl 16(%ebp), %eax           }
{   movl 8(%edx), %edx                        movl 8(%edx), %edx            }
{   movl (%eax), eax                          movl (%eax), eax              }
{   cmpl %eax, %edx                           cmpl %eax, %edx               }
{   jnz  l123           getting converted to  jnz  l123                     }
{   movl 12(%ebp), %edx                       movl 4(%eax), eax             }
{   movl 16(%ebp), %eax                                                     }
{   movl 8(%edx), %edx                                                      }
{   movl 4(%eax), eax                                                       }
*)

{ not anymore: if the start of a new sequence is found while checking (e.g. }
{ above that of eax while checking edx, this new sequence is automatically  }
{ also checked                                                              }
                                   Cnt2 := 1;
                                   while Cnt2 <= Cnt do
                                     begin
{$ifndef noremove}
                                         ptaiprop(p.optinfo)^.CanBeRemoved := True
{$endif noremove}
                                       ; inc(Cnt2);
                                       GetNextInstruction(p, p);
                                     end;
 {hp4 is used to get the contents of the registers before the sequence}
                                   GetLastInstruction(hp2, hp4);

{$IfDef CSDebug}
                                   for regcounter := RS_EAX To RS_EDI do
                                     if (regcounter in reginfo.RegsLoadedforRef) then
                                       begin
                                         hp5 := tai_comment.Create(strpnew('New: '+std_regname(newreg(R_INTREGISTER,regcounter,R_SUBNONE))+', Old: '+
                                           std_regname(newreg(R_INTREGISTER,reginfo.new2oldreg[regcounter],R_SUBNONE))));
                                         InsertLLItem(asml, tai(hp2.previous), hp2, hp5);
                                       end;
{$EndIf CSDebug}
 { if some registers were different in the old and the new sequence, move }
 { the contents of those old registers to the new ones                    }
                                   loadcseregs(asml,reginfo,p,prevseq,hp2,hp4,cnt);
                                   continue;
                                 end
                              end;
                          end;
                      { try to replace the new reg with the old reg }
                      if not(ptaiprop(p.optinfo)^.canBeRemoved) then
                        if (taicpu(p).oper[0]^.typ = top_reg) and
                           (taicpu(p).oper[1]^.typ = top_reg) and
                           { only remove if we're not storing something in a regvar }
                           (getsupreg(taicpu(p).oper[1]^.reg) in [RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI]) and
{                           (taicpu(p).oper[1]^.reg in (rg.usableregsint+[RS_EDI])) and}
                           (taicpu(p).opcode = A_MOV) and
                           getLastInstruction(p,hp4) and
                          { we only have to start replacing from the instruction after the mov, }
                          { but replacereg only starts with getnextinstruction(p,p)             }
                            replaceReg(asml,getsupreg(taicpu(p).oper[0]^.reg),
                              getsupreg(taicpu(p).oper[1]^.reg),p,p,
                              ptaiprop(hp4.optinfo)^.regs[getsupreg(taicpu(p).oper[1]^.reg)],false,hp1) then
                          begin
                            ptaiprop(p.optinfo)^.canBeRemoved := true;
                            { this is just a regular move that was here, so the source register should be }
                            { allocated already at this point -> only allocate from here onwards          }
                            if not(getsupreg(taicpu(p).oper[0]^.reg) in pTaiProp(p.optinfo)^.usedregs) then
                              internalerror(2004101011);
                            allocRegBetween(asml,taicpu(p).oper[0]^.reg,
                              p,hp1,pTaiProp(p.optinfo)^.usedregs)
                          end
                        else
                          begin
                             if (taicpu(p).oper[1]^.typ = top_reg) and
                                not regInOp(getsupreg(taicpu(p).oper[1]^.reg),taicpu(p).oper[0]^) then
                               removePrevNotUsedLoad(asml,p,getsupreg(taicpu(p).oper[1]^.reg),false);
                             if doSubOpts and
                                (taicpu(p).opcode <> A_LEA) and
                                (taicpu(p).oper[0]^.typ = top_ref) then
                              begin
                                memreg :=
                                  memtoreg(taicpu(p),
                                  taicpu(p).oper[0]^.ref^,hp5);
                                if memreg <> NR_NO then
                                  if (taicpu(p).opcode = A_MOV) and
                                     (taicpu(p).oper[1]^.typ = top_reg) and
                                     (taicpu(p).oper[1]^.reg = memreg) then
                                    begin
                                      pTaiProp(p.optinfo)^.canberemoved := true;
                                      allocregbetween(asml,memreg,hp5,p,ptaiprop(hp5.optinfo)^.usedregs);
                                    end
                                  else
                                    begin
                                      replaceoperandwithreg(asml,p,0,memreg);
                                      allocregbetween(asml,memreg,hp5,p,ptaiprop(hp5.optinfo)^.usedregs);
                                      regcounter := getsupreg(memreg);
                                      incstate(pTaiProp(p.optinfo)^.regs[regcounter].rstate,1);
                                      updatestate(regcounter,p);
                                    end;
                              end;
                          end;
                        { at first, only try optimizations of large blocks, because doing }
                        { doing smaller ones may prevent bigger ones from completing in   }
                        { in the next pass                                                }
                        if not doSubOpts and (orgNrofMods <> 0) then
                          begin
                            p := hp2;
                            for cnt := 1 to pred(orgNrofMods) do
                              getNextInstruction(p,p);
                          end;
                      end;
                    top_Const:
                      begin
                        case taicpu(p).oper[1]^.typ of
                          Top_Reg:
                            begin
                              regCounter := getsupreg(taicpu(p).oper[1]^.reg);
                              if GetLastInstruction(p, hp1) then
                                With ptaiprop(hp1.optinfo)^.Regs[regCounter] do
                                  if (typ in [con_const,con_noRemoveConst]) and
                                     (taicpu(startMod).opsize >= taicpu(p).opsize) and
                                     opsequal(taicpu(StartMod).oper[0]^,taicpu(p).oper[0]^) then
                                    begin
                                      ptaiprop(p.optinfo)^.CanBeRemoved := True;
                                      allocRegBetween(asml,taicpu(p).oper[1]^.reg,startmod,p,
                                        ptaiprop(startmod.optinfo)^.usedregs);
                                    end
                                  else
                                    removePrevNotUsedLoad(asml,p,getsupreg(taicpu(p).oper[1]^.reg),false);

                            end;
                          Top_Ref:
                            if (taicpu(p).oper[0]^.typ = top_const) and
                               getLastInstruction(p,hp1) and
                               findRegWithConst(hp1,taicpu(p).opsize,taicpu(p).oper[0]^.val,memreg) then
                              begin
                                taicpu(p).loadreg(0,memreg);
                                { mark the used register as read }
                                incstate(ptaiprop(p.optinfo)^.
                                   regs[getsupreg(memreg)].rstate,20);
                                updateState(getsupreg(memreg),p);
                                allocRegBetween(asml,memreg,
                                  ptaiprop(hp1.optinfo)^.regs[getsupreg(memreg)].startMod,p,
                                  ptaiprop(ptaiprop(hp1.optinfo)^.regs[getsupreg(memreg)].startMod.optinfo)^.usedregs);
                              end;
                        end;
                      end;
                  end;

                end;
              A_LEAVE:
                begin
                  if getlastinstruction(p,hp1) then
                    removeLocalStores(hp1);
                end;
              A_STD: if GetLastInstruction(p, hp1) and
                        (ptaiprop(hp1.optinfo)^.DirFlag = F_Set) then
                        ptaiprop(tai(p).optinfo)^.CanBeRemoved := True;
              else
                begin
                  for cnt := 1 to maxinschanges do
                    begin
                      case InsProp[taicpu(p).opcode].Ch[cnt] of
                        Ch_ROp1:
                          if (taicpu(p).oper[0]^.typ = top_ref) and
                             ((taicpu(p).opcode < A_F2XM1) or
                              ((taicpu(p).opcode > A_IN) and
                               (taicpu(p).opcode < A_OUT)) or
                              (taicpu(p).opcode = A_PUSH) or
                              ((taicpu(p).opcode >= A_RCL) and
                               (taicpu(p).opcode <= A_XOR))) then
                            begin
                              memreg :=
                                memtoreg(taicpu(p),
                                taicpu(p).oper[0]^.ref^,hp5);
                              if memreg <> NR_NO then
                                begin
                                  replaceoperandwithreg(asml,p,0,memreg);
                                  allocregbetween(asml,memreg,hp5,p,ptaiprop(hp5.optinfo)^.usedregs);
                                  regcounter := getsupreg(memreg);
                                  incstate(pTaiProp(p.optinfo)^.regs[regcounter].rstate,1);
                                  updatestate(regcounter,p);
                                end;
                            end;
                        Ch_MOp1:
                          if not(cs_opt_size in current_settings.optimizerswitches) and
                             (taicpu(p).oper[0]^.typ = top_ref) then
                            begin
                              memreg :=
                                memtoreg(taicpu(p),
                                taicpu(p).oper[0]^.ref^,hp5);
                              if (memreg <> NR_NO) and
                                 (not getNextInstruction(p,hp1) or
                                  (RegLoadedWithNewValue(getsupreg(memreg),false,hp1) or
                                   FindRegDealloc(getsupreg(memreg),hp1))) then
                                begin
                                  hp1 := Tai_Marker.Create(mark_NoPropInfoEnd);
                                  insertllitem(asml,p,p.next,hp1);
                                  hp1 := taicpu.op_reg_ref(A_MOV,reg2opsize(memreg),
                                     memreg,taicpu(p).oper[0]^.ref^);
                                  new(ptaiprop(hp1.optinfo));
                                  pTaiProp(hp1.optinfo)^ := pTaiProp(p.optinfo)^;
                                  insertllitem(asml,p,p.next,hp1);
                                  regcounter := getsupreg(memreg);
                                  incstate(pTaiProp(hp1.optinfo)^.regs[regcounter].rstate,1);
                                  updatestate(regcounter,hp1);
                                  hp1 := Tai_Marker.Create(mark_NoPropInfoStart);
                                  insertllitem(asml,p,p.next,hp1);
                                  replaceoperandwithreg(asml,p,0,memreg);
                                  allocregbetween(asml,memreg,hp5,
                                    tai(p.next.next),ptaiprop(hp5.optinfo)^.usedregs);
                                  ClearRegContentsFrom(asml,regcounter,hp5,p);
                                end;
                            end;
                        Ch_ROp2:
                          if ((taicpu(p).opcode = A_CMP) or
                              (taicpu(p).opcode = A_TEST)) and
                             (taicpu(p).oper[1]^.typ = top_ref) then
                            begin
                              memreg :=
                                memtoreg(taicpu(p),
                                taicpu(p).oper[1]^.ref^,hp5);
                              if memreg <> NR_NO then
                                begin
                                  replaceoperandwithreg(asml,p,1,memreg);
                                  allocregbetween(asml,memreg,hp5,p,ptaiprop(hp5.optinfo)^.usedregs);
                                  regcounter := getsupreg(memreg);
                                  incstate(pTaiProp(p.optinfo)^.regs[regcounter].rstate,1);
                                  updatestate(regcounter,p);
                                end;
                            end;
                        Ch_MOp2:
                          if not(cs_opt_size in current_settings.optimizerswitches) and
                             (taicpu(p).oper[1]^.typ = top_ref) and
                             ((taicpu(p).opcode < A_BT) or
                              ((taicpu(p).opcode > A_IN) and
                               (taicpu(p).opcode < A_OUT)) or
                              (taicpu(p).opcode = A_PUSH) or
                              ((taicpu(p).opcode >= A_RCL) and
                               (taicpu(p).opcode <= A_XOR))) then
                            begin
                              memreg :=
                                memtoreg(taicpu(p),
                                taicpu(p).oper[1]^.ref^,hp5);
                              if (memreg <> NR_NO) and
                                 (not getNextInstruction(p,hp1) or
                                  (RegLoadedWithNewValue(getsupreg(memreg),false,hp1) or
                                   FindRegDealloc(getsupreg(memreg),hp1))) then
                                begin
                                  hp1 := Tai_Marker.Create(mark_NoPropInfoEnd);
                                  insertllitem(asml,p,p.next,hp1);
                                  hp1 := taicpu.op_reg_ref(A_MOV,reg2opsize(memreg),
                                    memreg,taicpu(p).oper[1]^.ref^);
                                  new(ptaiprop(hp1.optinfo));
                                  pTaiProp(hp1.optinfo)^ := pTaiProp(p.optinfo)^;
                                  insertllitem(asml,p,p.next,hp1);
                                  regcounter := getsupreg(memreg);
                                  incstate(pTaiProp(hp1.optinfo)^.regs[regcounter].rstate,1);
                                  updatestate(regcounter,hp1);
                                  hp1 := Tai_Marker.Create(mark_NoPropInfoStart);
                                  insertllitem(asml,p,p.next,hp1);
                                  replaceoperandwithreg(asml,p,1,memreg);
                                  allocregbetween(asml,memreg,hp5,
                                    tai(p.next.next),ptaiprop(hp5.optinfo)^.usedregs);
                                  ClearRegContentsFrom(asml,regcounter,hp5,p);
                                end;
                            end;
                      end;
                    end;
                end;
            end
          end;
      end;
      GetNextInstruction(p, p);
    end;
end;

function removeInstructs(asml: TAsmList; first, last: tai): boolean;
{ Removes the marked instructions and disposes the PTaiProps of the other }
{ instructions                                                            }
var
  p, hp1: tai;
  nopropinfolevel: longint;
begin
  removeInstructs := false;
  p := First;
  nopropinfolevel := 0;
  while (p <> Last) do
    begin
      if (p.typ = ait_marker) and
         (Tai_marker(p).kind = mark_NoPropInfoStart) then
        begin
          hp1 := tai(p.next);
          asml.remove(p);
          p.free;
          nopropinfolevel := 1;
          while (nopropinfolevel <> 0) do
            begin
              p := tai(hp1.next);
{$ifndef noinstremove}
              { allocregbetween can insert new ait_regalloc objects }
              { without optinfo                                     }
              if (hp1.typ = ait_marker) then
                begin
                  case Tai_marker(hp1).kind of
                    { they can be nested! }
                    mark_NoPropInfoStart: inc(nopropinfolevel);
                    mark_NoPropInfoEnd: dec(nopropinfolevel);
                    else
                      begin
                        hp1 := p;
                        continue;
                      end;
                  end;
                  asml.remove(hp1);
                  hp1.free;
                end
              else if assigned(hp1.optinfo) then
                if ptaiprop(hp1.optinfo)^.canBeRemoved then
                  begin
                    dispose(ptaiprop(hp1.optinfo));
                    hp1.optinfo := nil;
                    asml.remove(hp1);
                    hp1.free;
                  end
                else
{$endif noinstremove}
                  begin
                    dispose(ptaiprop(hp1.optinfo));
                    hp1.optinfo := nil;
                  end;
              hp1 := p;
            end;
        end
      else
{$ifndef noinstremove}
        if assigned(p.optinfo) and
              ptaiprop(p.optinfo)^.canBeRemoved then
          begin
            hp1 := tai(p.next);
            asml.Remove(p);
            p.free;
            p := hp1;
            removeInstructs := true;
          end
        else
{$endif noinstremove}
          begin
            p.optinfo := nil;
            p := tai(p.next);
          end;
    end;
end;

function CSE(asml: TAsmList; First, Last: tai; pass: longint): boolean;
begin
  doCSE(asml, First, Last, not(cs_opt_level3 in current_settings.optimizerswitches) or (pass >= 2),
        not(cs_opt_level3 in current_settings.optimizerswitches) or (pass >= 1));
 { register renaming }
  if not(cs_opt_level3 in current_settings.optimizerswitches) or (pass > 0) then
    doRenaming(asml, first, last);
  cse := removeInstructs(asml, first, last);
end;

end.
