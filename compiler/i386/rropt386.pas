{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
      development team

    This unit contains register renaming functionality

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
unit rropt386;

{$i fpcdefs.inc}

interface

uses aasmbase,aasmtai,aasmdata,aasmcpu;

procedure doRenaming(asml: TAsmList; first, last: tai);

implementation

uses
  {$ifdef replaceregdebug}cutils,{$endif}
  verbose,globals,cpubase,daopt386,csopt386,rgobj,
  cgbase,cgutils,cgobj;

function canBeFirstSwitch(p: taicpu; supreg: tsuperregister): boolean;
{ checks whether an operation on reg can be switched to another reg without an }
{ additional mov, e.g. "addl $4,%reg1" can be changed to "leal 4(%reg1),%reg2" }
begin
  canBeFirstSwitch := false;
  case p.opcode of
    A_MOV,A_MOVZX,A_MOVSX,A_LEA:
      canBeFirstSwitch :=
        (p.oper[1]^.typ = top_reg) and
        (getsupreg(p.oper[1]^.reg) = supreg);
    A_IMUL:
      canBeFirstSwitch :=
        (p.ops >= 2) and
        (p.oper[0]^.typ = top_const) and
        (getsupreg(p.oper[p.ops-1]^.reg) = supreg) and
        (not pTaiprop(p.optinfo)^.FlagsUsed);
    A_INC,A_DEC:
      canBeFirstSwitch :=
        (p.oper[0]^.typ = top_reg) and
        (p.opsize = S_L) and
        (not pTaiprop(p.optinfo)^.FlagsUsed);
    A_SUB,A_ADD:
      canBeFirstSwitch :=
        (p.oper[1]^.typ = top_reg) and
        (p.opsize = S_L) and
        (getsupreg(p.oper[1]^.reg) = supreg) and
        (p.oper[0]^.typ <> top_ref) and
        ((p.opcode <> A_SUB) or
         (p.oper[0]^.typ = top_const)) and
        (not pTaiprop(p.optinfo)^.FlagsUsed);
    A_SHL:
      canBeFirstSwitch :=
        (p.opsize = S_L) and
        (p.oper[1]^.typ = top_reg) and
        (getsupreg(p.oper[1]^.reg) = supreg) and
        (p.oper[0]^.typ = top_const) and
        (p.oper[0]^.val in [1,2,3]) and
        (not pTaiprop(p.optinfo)^.FlagsUsed);
  end;
end;


procedure switchReg(var reg: tregister; reg1, reg2: tsuperregister);
var
  supreg: tsuperregister;
begin
  if (reg = NR_NO) or
     (getregtype(reg) <> R_INTREGISTER) then
    exit;
  supreg := getsupreg(reg);
  if (supreg = reg1) then
    setsupreg(reg,reg2)
  else if (supreg = reg2) then
    setsupreg(reg,reg1);
end;


procedure switchOp(var op: toper; reg1, reg2: tsuperregister);
begin
  case op.typ of
    top_reg:
      switchReg(op.reg,reg1,reg2);
    top_ref:
      begin
        switchReg(op.ref^.base,reg1,reg2);
        switchReg(op.ref^.index,reg1,reg2);
      end;
  end;
end;


procedure doSwitchReg(hp: taicpu; reg1,reg2: tsuperregister);
var
  opCount: longint;
begin
  for opCount := 0 to hp.ops-1 do
    switchOp(hp.oper[opCount]^,reg1,reg2);
end;


procedure doFirstSwitch(p: taicpu; reg1, reg2: tsuperregister);
var
  tmpRef: treference;
begin
  case p.opcode of
    A_MOV,A_MOVZX,A_MOVSX,A_LEA:
       begin
         changeOp(p.oper[1]^,reg1,reg2);
         changeOp(p.oper[0]^,reg2,reg1);
       end;
    A_IMUL:
      begin
        p.ops := 3;
        p.loadreg(2,newreg(R_INTREGISTER,reg2,R_SUBWHOLE));
        changeOp(p.oper[1]^,reg2,reg1);
      end;
    A_INC,A_DEC:
      begin
        reference_reset(tmpref);
        tmpref.base := newreg(R_INTREGISTER,reg1,R_SUBWHOLE);
        case p.opcode of
          A_INC:
            tmpref.offset := 1;
          A_DEC:
            tmpref.offset := -1;
        end;
        p.ops := 2;
        p.opcode := A_LEA;
        p.loadreg(1,newreg(R_INTREGISTER,reg2,R_SUBWHOLE));
        p.loadref(0,tmpref);
      end;
    A_SUB,A_ADD:
      begin
        reference_reset(tmpref);
        tmpref.base := newreg(R_INTREGISTER,reg1,R_SUBWHOLE);
        case p.oper[0]^.typ of
          top_const:
            begin
              tmpref.offset := longint(p.oper[0]^.val);
              if p.opcode = A_SUB then
                tmpref.offset := - tmpRef.offset;
            end;
          top_ref:
            if (p.oper[0]^.ref^.refaddr=addr_full) then
              tmpref.symbol := p.oper[0]^.ref^.symbol
            else
              internalerror(200402263);
          top_reg:
            begin
              { "addl %reg2,%reg1" must become "leal (%reg1,%reg1),%reg2" }
              { since at this point reg1 holds the value that reg2 would  }
              { otherwise contain                                         }
              tmpref.index := p.oper[0]^.reg;
              if (getsupreg(tmpref.index)=reg2) then
                setsupreg(tmpref.index,reg1);
              tmpref.scalefactor := 1;
            end;
          else internalerror(200010031);
        end;
        p.opcode := A_LEA;
        p.loadref(0,tmpref);
        p.loadreg(1,newreg(R_INTREGISTER,reg2,R_SUBWHOLE));
      end;
    A_SHL:
      begin
        reference_reset(tmpref);
        tmpref.index := newreg(R_INTREGISTER,reg1,R_SUBWHOLE);
        tmpref.scalefactor := 1 shl p.oper[0]^.val;
        p.opcode := A_LEA;
        p.loadref(0,tmpref);
        p.loadreg(1,newreg(R_INTREGISTER,reg2,R_SUBWHOLE));
      end;
    else internalerror(200010032);
  end;
end;


function switchRegs(asml: TAsmList; reg1, reg2: tsuperregister; start: tai): Boolean;
{ change movl  %reg1,%reg2 ... bla ... to ... bla with reg1 and reg2 switched }
var
  endP, hp, lastreg1,lastreg2: tai;
  switchDone, switchLast, tmpResult, sequenceEnd, reg1Modified, reg2Modified: boolean;
  reg1StillUsed, reg2StillUsed, isInstruction: boolean;
begin
  switchRegs := false;
  tmpResult := true;
  sequenceEnd := false;
  reg1Modified := false;
  reg2Modified := false;
  endP := start;
  while tmpResult and not sequenceEnd do
    begin
      tmpResult :=
        getNextInstruction(endP,endP);
      If tmpResult and
         not pTaiprop(endp.optinfo)^.canBeRemoved then
        begin
          { if the newReg gets stored back to the oldReg, we can change }
          { "mov %oldReg,%newReg; <operations on %newReg>; mov %newReg, }
          { %oldReg" to "<operations on %oldReg>"                       }
          switchLast := storeBack(start,endP,reg1,reg2);
          reg1StillUsed := reg1 in pTaiprop(endp.optinfo)^.usedregs;
          reg2StillUsed := reg2 in pTaiprop(endp.optinfo)^.usedregs;
          isInstruction := endp.typ = ait_instruction;
          sequenceEnd :=
            switchLast or
            { if both registers are released right before an instruction }
            { that contains hardcoded regs, it's ok too                  }
            (not reg1StillUsed and not reg2StillUsed) or
            { no support for (i)div, mul and imul with hardcoded operands }
            (((not isInstruction) or
              noHardCodedRegs(taicpu(endP),reg1,reg2)) and
             (not reg1StillUsed or
              (isInstruction and findRegDealloc(reg1,endP) and
               regLoadedWithNewValue(reg1,false,taicpu(endP)))) and
             (not reg2StillUsed or
              (isInstruction and findRegDealloc(reg2,endP) and
               regLoadedWithNewValue(reg2,false,taicpu(endP)))));

          { we can't switch reg1 and reg2 in something like }
          {   movl  %reg1,%reg2                             }
          {   movl  (%reg2),%reg2                           }
          {   movl  4(%reg1),%reg1                          }
          if reg2Modified and not(reg1Modified) and
             regReadByInstruction(reg1,endP) then
            begin
              tmpResult := false;
              break
            end;

          if not reg1Modified then
            begin
              reg1Modified := regModifiedByInstruction(reg1,endP);
              if reg1Modified and not canBeFirstSwitch(taicpu(endP),reg1) then
                begin
                  tmpResult := false;
                  break;
                end;
            end;
          if not reg2Modified then
            reg2Modified := regModifiedByInstruction(reg2,endP);

          tmpResult :=
            ((not isInstruction) or
             (NoHardCodedRegs(taicpu(endP),reg1,reg2) and
              RegSizesOk(reg1,reg2,taicpu(endP))));

          if sequenceEnd then
            break;

          tmpResult :=
            tmpresult and
            (endp.typ <> ait_label) and
            ((not isInstruction) or
             (not taicpu(endp).is_jmp));
        end;
    end;

  if tmpResult and sequenceEnd then
    begin
      switchRegs := true;
      reg1Modified := false;
      reg2Modified := false;
      lastreg1 := start;
      lastreg2 := start;
      getNextInstruction(start,hp);
      while hp <> endP do
        begin
          if (not pTaiprop(hp.optinfo)^.canberemoved) and
             (hp.typ = ait_instruction) then
            begin
              switchDone := false;
              if not reg1Modified then
                begin
                  reg1Modified := regModifiedByInstruction(reg1,hp);
                  if reg1Modified then
                    begin
                      doFirstSwitch(taicpu(hp),reg1,reg2);
                      switchDone := true;
                    end;
                end;
              if not switchDone then
                if reg1Modified then
                  doSwitchReg(taicpu(hp),reg1,reg2)
                else
                  doReplaceReg(taicpu(hp),reg2,reg1);
            end;
          if regininstruction(reg1,hp) then
             lastreg1 := hp;
          if regininstruction(reg2,hp) then
             lastreg2 := hp;
          getNextInstruction(hp,hp);
        end;
      if switchLast then
        begin
          lastreg1 := hp;
          lastreg2 := hp;
          { this is in case of a storeback, make sure the same size of register }
          { contents as the initial move is transfered                          }
          doSwitchReg(taicpu(hp),reg1,reg2);
          if taicpu(hp).opsize <> taicpu(start).opsize then
            begin
              taicpu(hp).opsize := taicpu(start).opsize;
              taicpu(hp).oper[0]^.reg := taicpu(start).oper[0]^.reg;
              taicpu(hp).oper[1]^.reg := taicpu(start).oper[1]^.reg;
            end;
        end
      else
        getLastInstruction(hp,hp);
      allocRegBetween(asmL,newreg(R_INTREGISTER,reg1,R_SUBWHOLE),start,lastreg1,
        ptaiprop(start.optinfo)^.usedregs);
      allocRegBetween(asmL,newreg(R_INTREGISTER,reg2,R_SUBWHOLE),start,lastreg2,
        ptaiprop(start.optinfo)^.usedregs);
    end;
end;


procedure doRenaming(asml: TAsmList; first, last: tai);
var
  p: tai;
begin
  p := First;
  SkipHead(p);
  while p <> last do
    begin
      case p.typ of
        ait_instruction:
          begin
            case taicpu(p).opcode of
              A_MOV:
                begin
                  if not(pTaiprop(p.optinfo)^.canBeRemoved) and
                     (taicpu(p).oper[0]^.typ = top_reg) and
                     (taicpu(p).oper[1]^.typ = top_reg) and
                     (taicpu(p).opsize = S_L) and
                     (getsupreg(taicpu(p).oper[0]^.reg) in ([RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI])) and
                     (getsupreg(taicpu(p).oper[1]^.reg) in ([RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI])) then
                    if switchRegs(asml,getsupreg(taicpu(p).oper[0]^.reg),
                         getsupreg(taicpu(p).oper[1]^.reg),p) then
                      begin
                        pTaiprop(p.optinfo)^.canBeRemoved := true;
                      end;
                end;
            end;
          end;
      end;
      getNextInstruction(p,p);
    end;
end;


End.
