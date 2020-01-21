{
    Copyright (c) 1998-2004 by Jonas Maebe

    This unit calls the optimization procedures to optimize the assembler
    code for sparc

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
      cpubase, aoptobj, aoptcpub, aopt, aoptx86,
      aasmtai;

    Type
      TCpuAsmOptimizer = class(TX86AsmOptimizer)
        function PeepHoleOptPass1Cpu(var p : tai) : boolean; override;
        function PostPeepHoleOptsCpu(var p : tai) : boolean; override;
      End;

  Implementation

    uses
      globals,
      verbose,
      cpuinfo,
      aasmcpu,
      aoptutils;

    function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p : tai) : boolean;
      var
        hp1 : tai;
        hp2 : tai;
      begin
        result:=false;
        case p.typ of
          ait_instruction:
            begin
              case taicpu(p).opcode of
                A_MOV:
                  begin
                    if MatchInstruction(p,A_MOV,[S_W]) and
                      MatchOpType(taicpu(p),top_ref,top_reg) and

                      GetNextInstruction(p, hp1) and
                      MatchInstruction(hp1,A_MOV,[S_W]) and
                      MatchOpType(taicpu(hp1),top_ref,top_reg) and

                      GetNextInstruction(hp1, hp2) and
                      MatchInstruction(hp2,A_MOV,[S_W]) and
                      MatchOpType(taicpu(hp2),top_reg,top_reg) and

                      not(OpsEqual(taicpu(p).oper[1]^,taicpu(hp1).oper[1]^)) and

                      OpsEqual(taicpu(hp1).oper[1]^,taicpu(hp2).oper[0]^) and

                      (MatchOperand(taicpu(hp2).oper[1]^,NR_ES) or MatchOperand(taicpu(hp2).oper[1]^,NR_DS) or
                       ((current_settings.cputype>=cpu_386) and
                        (MatchOperand(taicpu(hp2).oper[1]^,NR_SS) or
                         MatchOperand(taicpu(hp2).oper[1]^,NR_FS) or
                         MatchOperand(taicpu(hp2).oper[1]^,NR_GS)))) and

                      (taicpu(p).oper[0]^.ref^.base=taicpu(hp1).oper[0]^.ref^.base) and
                      (taicpu(p).oper[0]^.ref^.index=taicpu(hp1).oper[0]^.ref^.index) and
                      (taicpu(p).oper[0]^.ref^.segment=taicpu(hp1).oper[0]^.ref^.segment) and
                      (taicpu(p).oper[0]^.ref^.symbol=taicpu(hp1).oper[0]^.ref^.symbol) and
                      (taicpu(p).oper[0]^.ref^.relsymbol=taicpu(hp1).oper[0]^.ref^.relsymbol) and
                      (taicpu(p).oper[0]^.ref^.offset+2=taicpu(hp1).oper[0]^.ref^.offset) then
                      begin
                        case taicpu(hp2).oper[1]^.reg of
                          NR_DS:
                            taicpu(p).opcode:=A_LDS;
                          NR_ES:
                            taicpu(p).opcode:=A_LES;
                          NR_SS:
                            taicpu(p).opcode:=A_LSS;
                          NR_FS:
                            taicpu(p).opcode:=A_LFS;
                          NR_GS:
                            taicpu(p).opcode:=A_LGS;
                          else
                            internalerror(2015092601);
                        end;
                        if assigned(FindRegDealloc(taicpu(hp2).oper[0]^.reg,tai(hp2.Next))) then
                          begin
                            asml.remove(hp1);
                            hp1.free;
                            DebugMsg('Peephole optimizer MovMovMov2LXX',p);
                          end
                        else
                          begin
                            taicpu(hp1).loadreg(0,taicpu(hp2).oper[1]^.reg);
                            DebugMsg('Peephole optimizer MovMovMov2LXXMov',p);
                          end;

                        asml.remove(hp2);
                        hp2.free;
                        result:=true;
                      end
                    else if MatchInstruction(p,A_MOV,[S_W]) and
                      MatchOpType(taicpu(p),top_reg,top_reg) and
                      GetNextInstruction(p, hp1) and
                      MatchInstruction(hp1,A_PUSH,[S_W]) and
                      MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) and
                      assigned(FindRegDealloc(taicpu(hp1).oper[0]^.reg,tai(hp1.Next))) then
                      begin
                        DebugMsg('Peephole optimizer MovPush2Push',p);
                        taicpu(hp1).loadreg(0,taicpu(p).oper[0]^.reg);
                        { take care of the register (de)allocs following p }
                        UpdateUsedRegs(tai(p.next));
                        asml.remove(p);
                        p.free;
                        p:=hp1;
                        result:=true;
                      end;
                  end;
                A_SUB:
                  result:=OptPass1Sub(p);
                else
                  ;
              end;
            end
          else
            ;
        end;
      end;


    function TCpuAsmOptimizer.PostPeepHoleOptsCpu(var p: tai): boolean;
      begin
        result := false;
        case p.typ of
          ait_instruction:
            begin
              case taicpu(p).opcode of
                {A_MOV commented out, because it still breaks some i8086 code :( }
                {A_MOV:
                  Result:=PostPeepholeOptMov(p);}
                A_CMP:
                  Result:=PostPeepholeOptCmp(p);
                A_OR,
                A_TEST:
                  Result:=PostPeepholeOptTestOr(p);
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
