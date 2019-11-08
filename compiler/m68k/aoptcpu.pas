{
    Copyright (c) 1998-2014 by the Free Pascal development team

    This unit calls the optimization procedures to optimize the assembler
    code for m68k

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

{$define DEBUG_AOPTCPU}

  Interface

    uses
      cpubase, aoptobj, aoptcpub, aopt, aasmtai,
      cgbase;

    Type
      TCpuAsmOptimizer = class(TAsmOptimizer)
        function RegLoadedWithNewValue(reg: tregister; hp: tai): boolean; override;
        function PeepHoleOptPass1Cpu(var p: tai): boolean; override;

        function TryToOptimizeMove(var p: tai): boolean;
        function MaybeRealConstOperSimplify(var p: tai): boolean;

        { outputs a debug message into the assembler file }
        procedure DebugMsg(const s: string; p: tai);
      End;

  Implementation

    uses
      cutils, aasmcpu, cgutils, globals, verbose, cpuinfo, itcpugas;

{ Range check must be disabled explicitly as conversions between signed and unsigned
  32-bit values are done without explicit typecasts }
{$R-}

    function opname(var p: tai): string;
      begin
        result:=upcase(gas_op2str[taicpu(p).opcode]);
      end;

    function RefsEqual(const r1, r2: treference): boolean;
      begin
        RefsEqual :=
          (r1.offset = r2.offset) and
          (r1.base = r2.base) and
          (r1.index = r2.index) and (r1.scalefactor = r2.scalefactor) and
          (r1.symbol=r2.symbol) and (r1.refaddr = r2.refaddr) and
          (r1.relsymbol = r2.relsymbol) and
          (r1.volatility=[]) and
          (r2.volatility=[]);
      end;

    function MatchOperand(const oper1: TOper; const oper2: TOper): boolean;
      begin
        result := oper1.typ = oper2.typ;

        if result then
          case oper1.typ of
            top_const:
              Result:=oper1.val = oper2.val;
            top_reg:
              Result:=oper1.reg = oper2.reg;
            top_ref:
              Result:=RefsEqual(oper1.ref^, oper2.ref^);
            else
              internalerror(2016112401);
          end
      end;

    function TCpuAsmOptimizer.MaybeRealConstOperSimplify(var p: tai): boolean;
      var
        tmpint64: int64;
        tmpsingle: single;
      begin
        result:=false;
        if (taicpu(p).oper[0]^.typ = top_realconst) then
          begin
            { if we work with actual integers, turn the operand into one }
            if frac(taicpu(p).oper[0]^.val_real) = 0 then
              begin
                tmpint64:=trunc(taicpu(p).oper[0]^.val_real);
                if (high(shortint) >= tmpint64) and (low(shortint) <= tmpint64) then
                  begin
                    taicpu(p).opsize := S_B;
                    taicpu(p).oper[0]^.typ:=top_const;
                  end
                else
                  if (high(smallint) >= tmpint64) and (low(smallint) <= tmpint64) then
                    begin
                      taicpu(p).opsize := S_W;
                      taicpu(p).oper[0]^.typ:=top_const;
                    end
                  else
                    if (high(longint) >= tmpint64) and (low(longint) <= tmpint64) then
                      begin
                        taicpu(p).opsize := S_L;
                        taicpu(p).oper[0]^.typ:=top_const;
                      end;
                if (taicpu(p).oper[0]^.typ) = top_const then
                  begin
                    DebugMsg('Optimizer: FPU real const to integer',p);
                    taicpu(p).oper[0]^.val:=tmpint64;
                    result:=true;
                  end;
              end
            else
              begin
                tmpsingle:=taicpu(p).oper[0]^.val_real;
                if (taicpu(p).opsize = S_FD) and
                   ((taicpu(p).oper[0]^.val_real - tmpsingle) = 0.0) then
                  begin
                    DebugMsg('Optimizer: FPU real const to lesser precision',p);
                    taicpu(p).opsize:=S_FS;
                    result:=true;
                  end;
              end;
          end;
      end;

    function TCpuAsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
      var
        p: taicpu;
      begin
        if not assigned(hp) or
           (hp.typ <> ait_instruction) then
         begin
           Result := false;
           exit;
         end;
        p := taicpu(hp);
        Result :=
          ((p.opcode in [A_MOVE,A_MOVEA,A_MVS,A_MVZ,A_MOVEQ,A_LEA]) and
           (p.oper[1]^.typ = top_reg) and
           (SuperRegistersEqual(p.oper[1]^.reg,reg)) and
           ((p.oper[0]^.typ = top_const) or
            ((p.oper[0]^.typ = top_reg) and
             not(SuperRegistersEqual(p.oper[0]^.reg,reg))) or
            ((p.oper[0]^.typ = top_ref) and
             not RegInRef(reg,p.oper[0]^.ref^)))) or

          ((p.opcode = A_FMOVE) and
           (p.oper[1]^.typ = top_reg) and
           (SuperRegistersEqual(p.oper[1]^.reg,reg)) and
           ((p.oper[0]^.typ = top_realconst) or
            ((p.oper[0]^.typ = top_reg) and
            not(SuperRegistersEqual(p.oper[0]^.reg,reg))))) or

          ((p.opcode = A_MOVEM) and
           (p.oper[1]^.typ = top_regset) and
           ((getsupreg(reg) in p.oper[1]^.dataregset) or
            (getsupreg(reg) in p.oper[1]^.addrregset))) or

          ((p.opcode = A_FMOVEM) and
           (p.oper[1]^.typ = top_regset) and
           (getsupreg(reg) in p.oper[1]^.fpuregset));
      end;

{$ifdef DEBUG_AOPTCPU}
  procedure TCpuAsmOptimizer.DebugMsg(const s: string; p : tai);
    begin
      asml.insertbefore(tai_comment.Create(strpnew(s)), p);
    end;
{$else DEBUG_AOPTCPU}
  procedure TCpuAsmOptimizer.DebugMsg(const s: string; p : tai);inline;
    begin
    end;
{$endif DEBUG_AOPTCPU}

  function TCpuAsmOptimizer.TryToOptimizeMove(var p: tai): boolean;
    var
      next, next2: tai;
      opstr: string[15];
    begin
      result:=false;

      if GetNextInstruction(p,next) and 
         (next.typ = ait_instruction) and
         (taicpu(next).opcode = taicpu(p).opcode) and
         (taicpu(p).opsize = taicpu(next).opsize) and
         (taicpu(p).oper[1]^.typ = top_reg) and
         MatchOperand(taicpu(p).oper[1]^,taicpu(next).oper[0]^) then
        begin
          if not(RegInOp(taicpu(p).oper[1]^.reg,taicpu(next).oper[1]^)) and
             RegEndOfLife(taicpu(next).oper[0]^.reg, taicpu(next)) then
            begin
              opstr:=opname(p);
              case taicpu(p).oper[0]^.typ of
                top_reg:
                  begin
                    {  move %reg0, %tmpreg; move %tmpreg, <ea> -> move %reg0, <ea> }
                    taicpu(p).loadOper(1,taicpu(next).oper[1]^);
                    asml.remove(next);
                    next.free;
                    result:=true;
                    { also remove leftover move %reg0, %reg0, which can occur as the result
                      of the previous optimization, if %reg0 and %tmpreg was different types
                      (addr vs. data), so these moves were left in by the cg }
                    if MatchOperand(taicpu(p).oper[0]^,taicpu(p).oper[1]^) then
                      begin
                        DebugMsg('Optimizer: '+opstr+' + '+opstr+' removed',p);
                        GetNextInstruction(p,next);
                        asml.remove(p);
                        p.free;
                        p:=next;
                      end
                    else
                      DebugMsg('Optimizer: '+opstr+' + '+opstr+' to '+opstr+' #1',p)
                  end;
                top_const:
                  begin
                    // DebugMsg('Optimizer: '+opstr+' + '+opstr+' to '+opstr+' #2',p);
                  end;
                top_ref:
                  begin
                    { move ref, %tmpreg; move %tmpreg, <ea> -> move ref, <ea> }
                    { we only want to do this when <ea> is a reg or a simple reference }
                    with taicpu(next).oper[1]^ do
                      if (taicpu(next).opcode <> A_FMOVE) and
                         ((typ = top_reg) or
                          ((typ = top_ref) and
                           ((ref^.index = NR_NO) or
                            (ref^.base = NR_NO)) and
                           (ref^.symbol = nil) and
                           (ref^.offset = 0))) then
                        begin
                          DebugMsg('Optimizer: '+opstr+' + '+opstr+' to '+opstr+' #3',p);
                          taicpu(p).loadOper(1,taicpu(next).oper[1]^);
                          asml.remove(next);
                          next.free;
                          result:=true;
                        end;
                  end;
                else
                  ;
              end;
            end;
          exit;
        end;

      if GetNextInstruction(p,next) and
         (next.typ = ait_instruction) and
         GetNextInstruction(next,next2) and
         (next2.typ = ait_instruction) and
         (taicpu(next).opcode <> taicpu(p).opcode) and
         (taicpu(next2).opcode = taicpu(p).opcode) and
         (taicpu(p).oper[0]^.typ = top_reg) and
         (taicpu(p).oper[1]^.typ = top_reg) and
         (getregtype(taicpu(p).oper[0]^.reg) = getregtype(taicpu(p).oper[1]^.reg)) and
         MatchOperand(taicpu(p).oper[1]^,taicpu(next2).oper[0]^) and
         MatchOperand(taicpu(next2).oper[1]^,taicpu(p).oper[0]^) and
         (taicpu(p).opsize = taicpu(next2).opsize) and
         ((taicpu(p).opcode = A_FMOVE) or
          (taicpu(p).opsize = taicpu(next).opsize)) then
        begin
          opstr:=opname(p);

          if not(RegInOp(taicpu(p).oper[1]^.reg,taicpu(next2).oper[1]^)) and
             not(RegInOp(taicpu(p).oper[1]^.reg,taicpu(next).oper[0]^)) and
             RegEndOfLife(taicpu(p).oper[0]^.reg, taicpu(next2)) then
            begin
              {  move %reg0, %tmpreg
                 op   ???, %tmpreg
                 move %tmpreg, %reg0
                 to:
                 op   ???, %reg0 }
              if MatchOperand(taicpu(p).oper[1]^,taicpu(next).oper[taicpu(next).ops-1]^) then
                begin
                  {
                  Disabled, because it breaks some tests... :( (KB)
                  DebugMsg('Optimizer: '+opstr+' + OP + '+opstr+' to OP #1',next);
                  taicpu(next).loadOper(taicpu(next).ops-1,taicpu(p).oper[0]^);
                  asml.remove(p);
                  asml.remove(next2);
                  p.free;
                  next2.free;
                  result:=true;
                  }
                end;
            end;
        end;
    end;

  function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    var
      next: tai;
      tmpref: treference;
    begin
      result:=false;
      case p.typ of
        ait_instruction:
          begin
            //asml.insertbefore(tai_comment.Create(strpnew('pass1 called for instr')), p);

            case taicpu(p).opcode of
              A_MOVE:
                result:=TryToOptimizeMove(p);
              { LEA (Ax),Ax is a NOP if src and dest reg is equal, so remove it. }
              A_LEA:
                if not assigned(taicpu(p).oper[0]^.ref^.symbol) and
                   (((taicpu(p).oper[0]^.ref^.base = taicpu(p).oper[1]^.reg) and
                   (taicpu(p).oper[0]^.ref^.index = NR_NO)) or
                   ((taicpu(p).oper[0]^.ref^.index = taicpu(p).oper[1]^.reg) and
                   (taicpu(p).oper[0]^.ref^.base = NR_NO))) and
                   (taicpu(p).oper[0]^.ref^.offset = 0) then
                  begin
                    DebugMsg('Optimizer: LEA 0(Ax),Ax removed',p);
                    GetNextInstruction(p,next);
                    asml.remove(p);
                    p.free;
                    p:=next;
                    result:=true;
                  end;
              { Address register sub/add can be replaced with ADDQ/SUBQ or LEA if the value is in the
                SmallInt range, which is shorter to encode and faster to execute on most 68k }
              A_SUB,A_SUBA,A_ADD,A_ADDA:
                if (taicpu(p).oper[1]^.typ = top_reg) and isaddressregister(taicpu(p).oper[1]^.reg) and
                   (taicpu(p).oper[0]^.typ = top_const) then
                  begin
                    if isvalueforaddqsubq(taicpu(p).oper[0]^.val) then
                      begin
                        DebugMsg('Optimizer: SUB/ADD #val,Ax to SUBQ/ADDQ',p);
                        taicpu(p).opsize:=S_L; // this is safe, because we're targetting an address reg
                        if taicpu(p).opcode in [A_ADD,A_ADDA] then
                          taicpu(p).opcode:=A_ADDQ
                        else
                          taicpu(p).opcode:=A_SUBQ;
                        result:=true;
                      end
                    else
                      if isvalue16bit(abs(taicpu(p).oper[0]^.val)) then
                        begin
                          DebugMsg('Optimizer: SUB/ADD #val,Ax to LEA val(Ax),Ax',p);
                          if taicpu(p).opcode in [A_SUB,A_SUBA] then
                            reference_reset_base(tmpref,taicpu(p).oper[1]^.reg,-taicpu(p).oper[0]^.val,ctempposinvalid,0,[])
                          else
                            reference_reset_base(tmpref,taicpu(p).oper[1]^.reg,taicpu(p).oper[0]^.val,ctempposinvalid,0,[]);
                          taicpu(p).opcode:=A_LEA;
                          taicpu(p).loadref(0,tmpref);
                          result:=true;
                        end;
                  end;
              { MOVEA #0,Ax to SUBA Ax,Ax, because it's shorter }
              A_MOVEA:
                if (taicpu(p).oper[0]^.typ = top_const) and
                   (taicpu(p).oper[0]^.val = 0) then
                  begin
                    DebugMsg('Optimizer: MOVEA #0,Ax to SUBA Ax,Ax',p);
                    taicpu(p).opcode:=A_SUBA;
                    taicpu(p).opsize:=S_L; { otherwise it will be .W -> BOOM }
                    taicpu(p).loadoper(0,taicpu(p).oper[1]^);
                    result:=true;
                  end;
              { CLR.L Dx on a 68000 is slower than MOVEQ #0,Dx }
              A_CLR:
                if (current_settings.cputype in [cpu_mc68000]) and
                   (taicpu(p).oper[0]^.typ = top_reg) and
                   (taicpu(p).opsize = S_L) and
                   isintregister(taicpu(p).oper[0]^.reg) then
                  begin
                    //DebugMsg('Optimizer: CLR.L Dx to MOVEQ #0,Dx',p);
                    taicpu(p).opcode:=A_MOVEQ;
                    taicpu(p).loadoper(1,taicpu(p).oper[0]^);
                    taicpu(p).loadconst(0,0);
                    taicpu(p).ops:=2;
                    result:=true;
                  end;
              { CMP #0,<ea> equals to TST <ea>, just shorter and TST is more flexible anyway }
              A_CMP,A_CMPI:
                if (taicpu(p).oper[0]^.typ = top_const) and
                   (taicpu(p).oper[0]^.val = 0) then
                  begin
                    DebugMsg('Optimizer: CMP #0 to TST',p);
                    taicpu(p).opcode:=A_TST;
                    taicpu(p).loadoper(0,taicpu(p).oper[1]^);
                    taicpu(p).clearop(1);
                    taicpu(p).ops:=1;
                    result:=true;
                  end;
              A_FCMP:
                if (taicpu(p).oper[0]^.typ = top_realconst) then
                  begin
                    if (taicpu(p).oper[0]^.val_real = 0.0) then
                      begin 
                        DebugMsg('Optimizer: FCMP #0.0 to FTST',p);
                        taicpu(p).opcode:=A_FTST;
                        taicpu(p).opsize:=S_FX;
                        taicpu(p).loadoper(0,taicpu(p).oper[1]^);
                        taicpu(p).clearop(1);
                        taicpu(p).ops:=1;
                        result:=true;
                      end
                    else
                      result:=result or MaybeRealConstOperSimplify(p);
                  end;
              A_FMOVE,A_FSMOVE,A_FDMOVE,
              A_FADD,A_FSADD,A_FDADD,A_FSUB,A_FSSUB,A_FDSUB,
              A_FMUL,A_FSMUL,A_FDMUL,A_FDIV,A_FSDIV,A_FDDIV,
              A_FSGLMUL,A_FSGLDIV:
                  begin
                    if (taicpu(p).opcode = A_FMOVE) and TryToOptimizeMove(p) then
                      begin
                        result:=true;
                        exit;
                      end;
                    result:=result or MaybeRealConstOperSimplify(p);
                  end;
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
