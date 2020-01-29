{
    Copyright (c) 1998-2002 by Florian Klaempfl and Jonas Maebe

    This unit contains the peephole optimizer for i386

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
      cgbase,
      cpubase, aopt, aoptx86,
      Aasmbase,aasmtai,aasmdata;

    Type
      TCpuAsmOptimizer = class(TX86AsmOptimizer)
        function PrePeepHoleOptsCpu(var p: tai): boolean; override;
        function PeepHoleOptPass1Cpu(var p: tai): boolean; override;
        function PeepHoleOptPass2Cpu(var p: tai): boolean; override;
        function PostPeepHoleOptsCpu(var p : tai) : boolean; override;
      end;

    Var
      AsmOptimizer : TCpuAsmOptimizer;

  Implementation

    uses
      verbose,globtype,globals,
      cpuinfo,
      aasmcpu,
      aoptutils,
      aasmcfi,
      procinfo,
      cgutils,
      { units we should get rid off: }
      symsym,symconst;


    { Checks if the register is a 32 bit general purpose register }
    function isgp32reg(reg: TRegister): boolean;
      begin
        {$push}{$warnings off}
        isgp32reg:=(getregtype(reg)=R_INTREGISTER) and (getsupreg(reg)>=RS_EAX) and (getsupreg(reg)<=RS_EBX);
        {$pop}
      end;


    { returns true if p contains a memory operand with a segment set }
    function InsContainsSegRef(p: taicpu): boolean;
      var
        i: longint;
      begin
        result:=true;
        for i:=0 to p.opercnt-1 do
          if (p.oper[i]^.typ=top_ref) and
             (p.oper[i]^.ref^.segment<>NR_NO) then
            exit;
        result:=false;
      end;


    function TCPUAsmOPtimizer.PrePeepHoleOptsCpu(var p: tai): boolean;
      begin
        repeat
          Result:=False;
          case p.typ of
            ait_instruction:
              begin
                if InsContainsSegRef(taicpu(p)) then
                  begin
                    p := tai(p.next);
                    { Nothing's actually changed, so no need to set Result to True,
                      but try again to see if an instruction immediately follows }
                    Continue;
                  end;
                case taicpu(p).opcode Of
                  A_IMUL:
                    Result:=PrePeepholeOptIMUL(p);
                  A_SAR,A_SHR:
                    Result:=PrePeepholeOptSxx(p);
                  A_XOR:
                    begin
                      if (taicpu(p).oper[0]^.typ = top_reg) and
                         (taicpu(p).oper[1]^.typ = top_reg) and
                         (taicpu(p).oper[0]^.reg = taicpu(p).oper[1]^.reg) then
                       { temporarily change this to 'mov reg,0' to make it easier }
                       { for the CSE. Will be changed back in pass 2              }
                        begin
                          taicpu(p).opcode := A_MOV;
                          taicpu(p).loadConst(0,0);
                          Result:=true;
                        end;
                    end;
                  else
                    { Do nothing };
                end;
            end;
          else
            { Do nothing };
          end;
          Break;
        until False;
      end;


    function TCPUAsmOPtimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
      var
        hp1 : tai;
      begin
        result:=False;
        case p.Typ Of
          ait_instruction:
            begin
              current_filepos:=taicpu(p).fileinfo;
              if InsContainsSegRef(taicpu(p)) then
                exit;
              case taicpu(p).opcode Of
                A_AND:
                  Result:=OptPass1And(p);
                A_CMP:
                  Result:=OptPass1Cmp(p);
                A_FLD:
                  Result:=OptPass1FLD(p);
                A_FSTP,A_FISTP:
                  Result:=OptPass1FSTP(p);
                A_LEA:
                  Result:=OptPass1LEA(p);
                A_MOV:
                  Result:=OptPass1MOV(p);
                A_MOVSX,
                A_MOVZX :
                  Result:=OptPass1Movx(p);
                A_PUSH:
                  begin
                    if (taicpu(p).opsize = S_W) and
                       (taicpu(p).oper[0]^.typ = Top_Const) and
                       GetNextInstruction(p, hp1) and
                       (tai(hp1).typ = ait_instruction) and
                       (taicpu(hp1).opcode = A_PUSH) and
                       (taicpu(hp1).oper[0]^.typ = Top_Const) and
                       (taicpu(hp1).opsize = S_W) then
                      begin
                        taicpu(p).changeopsize(S_L);
                        taicpu(p).loadConst(0,taicpu(p).oper[0]^.val shl 16 + word(taicpu(hp1).oper[0]^.val));
                        asml.remove(hp1);
                        hp1.free;
                        Result:=true;
                      end;
                  end;
                A_SHL, A_SAL:
                  Result:=OptPass1SHLSAL(p);
                A_SUB:
                  Result:=OptPass1Sub(p);
                A_MOVAPD,
                A_MOVAPS,
                A_MOVUPD,
                A_MOVUPS,
                A_VMOVAPS,
                A_VMOVAPD,
                A_VMOVUPS,
                A_VMOVUPD:
                  Result:=OptPass1_V_MOVAP(p);
                A_VDIVSD,
                A_VDIVSS,
                A_VSUBSD,
                A_VSUBSS,
                A_VMULSD,
                A_VMULSS,
                A_VADDSD,
                A_VADDSS,
                A_VANDPD,
                A_VANDPS,
                A_VORPD,
                A_VORPS,
                A_VXORPD,
                A_VXORPS:
                  Result:=OptPass1VOP(p);
                A_MULSD,
                A_MULSS,
                A_ADDSD,
                A_ADDSS:
                  Result:=OptPass1OP(p);
                A_VMOVSD,
                A_VMOVSS,
                A_MOVSD,
                A_MOVSS:
                  Result:=OptPass1MOVXX(p);
                A_SETcc:
                  Result:=OptPass1SETcc(p);
                else
                  ;
              end;
            end;
          else
            ;
        end;
      end;


    function TCPUAsmOptimizer.PeepHoleOptPass2Cpu(var p: tai): boolean;
      begin
        Result:=false;
        case p.Typ Of
          Ait_Instruction:
            begin
              if InsContainsSegRef(taicpu(p)) then
                exit;
              case taicpu(p).opcode Of
                A_Jcc:
                  Result:=OptPass2Jcc(p);
                A_Lea:
                  Result:=OptPass2Lea(p);
                A_FSTP,A_FISTP:
                  Result:=OptPass1FSTP(p);
                A_IMUL:
                  Result:=OptPass2Imul(p);
                A_JMP:
                  Result:=OptPass2Jmp(p);
                A_MOV:
                  Result:=OptPass2MOV(p);
                A_SUB:
                  Result:=OptPass2SUB(p);
                else
                  ;
              end;
            end;
          else
            ;
        end;
      end;


    function TCPUAsmOptimizer.PostPeepHoleOptsCpu(var p : tai) : boolean;
      var
        hp1: tai;
      begin
        Result:=false;
        case p.Typ Of
          Ait_Instruction:
            begin
              if InsContainsSegRef(taicpu(p)) then
                Exit;
              case taicpu(p).opcode Of
                A_CALL:
                  Result:=PostPeepHoleOptCall(p);
                A_LEA:
                  Result:=PostPeepholeOptLea(p);
                A_CMP:
                  Result:=PostPeepholeOptCmp(p);
                A_MOV:
                  Result:=PostPeepholeOptMov(p);
                A_MOVZX:
                  { if register vars are on, it's possible there is code like }
                  {   "cmpl $3,%eax; movzbl 8(%ebp),%ebx; je .Lxxx"           }
                  { so we can't safely replace the movzx then with xor/mov,   }
                  { since that would change the flags (JM)                    }
                  if not(cs_opt_regvar in current_settings.optimizerswitches) then
                    begin
                      if (taicpu(p).oper[1]^.typ = top_reg) then
                        if (taicpu(p).oper[0]^.typ = top_reg)
                          then
                            case taicpu(p).opsize of
                              S_BL:
                                begin
                                  if IsGP32Reg(taicpu(p).oper[1]^.reg) and
                                     not(cs_opt_size in current_settings.optimizerswitches) and
                                     (current_settings.optimizecputype = cpu_Pentium) then
                                      {Change "movzbl %reg1, %reg2" to
                                       "xorl %reg2, %reg2; movb %reg1, %reg2" for Pentium and
                                       PentiumMMX}
                                    begin
                                      hp1 := taicpu.op_reg_reg(A_XOR, S_L,
                                                  taicpu(p).oper[1]^.reg, taicpu(p).oper[1]^.reg);
                                      InsertLLItem(p.previous, p, hp1);
                                      taicpu(p).opcode := A_MOV;
                                      taicpu(p).changeopsize(S_B);
                                      setsubreg(taicpu(p).oper[1]^.reg,R_SUBL);
                                      Result := True;
                                    end;
                                end;
                              else
                                ;
                            end
                          else if (taicpu(p).oper[0]^.typ = top_ref) and
                              (taicpu(p).oper[0]^.ref^.base <> taicpu(p).oper[1]^.reg) and
                              (taicpu(p).oper[0]^.ref^.index <> taicpu(p).oper[1]^.reg) and
                              not(cs_opt_size in current_settings.optimizerswitches) and
                              IsGP32Reg(taicpu(p).oper[1]^.reg) and
                              (current_settings.optimizecputype = cpu_Pentium) and
                              (taicpu(p).opsize = S_BL) then
                            {changes "movzbl mem, %reg" to "xorl %reg, %reg; movb mem, %reg8" for
                              Pentium and PentiumMMX}
                            begin
                              hp1 := taicpu.Op_reg_reg(A_XOR, S_L, taicpu(p).oper[1]^.reg,
                                          taicpu(p).oper[1]^.reg);
                              taicpu(p).opcode := A_MOV;
                              taicpu(p).changeopsize(S_B);
                              setsubreg(taicpu(p).oper[1]^.reg,R_SUBL);
                              InsertLLItem(p.previous, p, hp1);
                              Result := True;
                            end;
                   end;
                A_TEST, A_OR:
                  Result:=PostPeepholeOptTestOr(p);
                A_MOVSX:
                  Result:=PostPeepholeOptMOVSX(p);
                else
                  ;
              end;

              { Optimise any reference-type operands (if Result is True, the
                instruction will be checked on the next iteration) }
              if not Result then
                OptimizeRefs(taicpu(p));
            end;
          else
            ;
        end;
      end;


begin
  casmoptimizer:=TCpuAsmOptimizer;
end.

