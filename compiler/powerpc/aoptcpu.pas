{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit implements the PowerPC optimizer object

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

Interface

{$i fpcdefs.inc}

uses cpubase, cgbase, aoptobj, aoptcpub, aopt, aasmtai,aasmdata, aasmcpu, aoptppc;

Type
  TCpuAsmOptimizer = class(TPPCAsmOptimizer)
    { uses the same constructor as TAopObj }
    function PeepHoleOptPass1Cpu(var p: tai): boolean; override;

    function PostPeepHoleOptsCpu(var p: tai): boolean; override;

   private
     function cmpi_mfcr_opt(p, next1, next2: taicpu): boolean;
  End;

Implementation

  uses
    cutils, verbose, cgcpu, cgobj;

  function TCpuAsmOptimizer.cmpi_mfcr_opt(p, next1, next2: taicpu): boolean;
    var
      next3, prev: tai;
      inverse, prevrlwinm: boolean;
    begin
      result := true;
      inverse :=
        getnextinstruction(next2,next3) and
        (next3.typ = ait_instruction) and
        (taicpu(next3).opcode = A_XORI) and
        (taicpu(next3).oper[0]^.reg = taicpu(next3).oper[1]^.reg) and
        (taicpu(next3).oper[0]^.reg = taicpu(next2).oper[0]^.reg) and
        (taicpu(next3).oper[2]^.val = 1);
      case taicpu(next2).oper[2]^.val of
        1:
         begin
           // less than zero or greater/equal than zero (the xori remains in
           // in the latter case). Doesn't make sense for unsigned comparisons.
           if (p.opcode = A_CMPWI) then
             begin
               p.opcode := A_SRWI;
               p.ops := 3;
               p.loadreg(1,p.oper[0]^.reg);
               p.loadreg(0,next1.oper[0]^.reg);
               p.loadconst(2,31);
               asml.remove(next1);
               next1.free;
               asml.remove(next2);
               next2.free;
             end
           else
             result := false;
         end;
{
    needs two registers to work with
        2:
         begin
           // greater or less/equal to zero
         end;
}
        3:
         begin
           prevrlwinm :=
             getlastinstruction(p,prev) and
             (prev.typ = ait_instruction) and
             ((taicpu(prev).opcode = A_RLWINM) or
              (taicpu(prev).opcode = A_RLWINM_)) and
             (taicpu(prev).oper[0]^.reg = p.oper[0]^.reg) and
             (taicpu(prev).oper[3]^.val = taicpu(prev).oper[4]^.val);

           if (prevrlwinm) then
             begin
               // isolate the bit we need
               if (taicpu(prev).oper[3]^.val <> 31) then
                 begin
                   p.opcode := A_RLWINM;
                   p.ops := 5;
                   p.loadreg(1,p.oper[0]^.reg);
                   p.loadreg(0,next1.oper[0]^.reg);
                   p.loadconst(2,taicpu(prev).oper[3]^.val + 1);
                   p.loadconst(3,31);
                   p.loadconst(4,31);
                 end
               else { if (taicpu(prev).oper[0]^.reg <> next1.oper[0]^.reg) then }
                 begin
                   p.opcode := A_MR;
                   p.loadreg(1,p.oper[0]^.reg);
                   p.loadreg(0,next1.oper[0]^.reg);
                 end;
               if not inverse then
                 begin
                   next1.ops := 3;
                   next1.opcode := A_XORI;
                   next1.loadreg(1,next1.oper[0]^.reg);
                   next1.loadconst(2,1);
                 end
               else
                 begin
                   asml.remove(next1);
                   next1.free;
                   asml.remove(next3);
                   next3.free;
                 end;
               asml.remove(next2);
               next2.free;
             end
           else
             begin
                // equal/not equal to zero (the xori remains in the latter case;
                // there's a more optimal sequence without it, but needs extra
                // register)
                p.opcode := A_CNTLZW;
                p.loadreg(1,p.oper[0]^.reg);
                p.loadreg(0,next1.oper[0]^.reg);
                next1.ops := 3;
                next1.opcode := A_SRWI;
                next1.loadreg(1,next1.oper[0]^.reg);
                next1.loadconst(2,5);
                asml.remove(next2);
                next2.free;
              end;
         end;
        else
          result := false;
      end;
    end;


  function rlwinm2mask(l1,l2: longint): longint;
    begin
       // 1 shl 32 = 1 instead of 0 on x86
      if (l1 <> 0) then
        result :=  longint(cardinal(1) shl (32 - l1) - 1) xor (cardinal(1) shl (31 - l2) - 1)
      else
        result := longint(not(cardinal(1) shl (31 - l2) - 1));
      if (l1 > l2) then
        result := not(result);
    end;


  function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    var
      next1, next2: tai;
      l1, l2, shlcount: longint;
    begin
      result := false;
      case p.typ of
        ait_instruction:
          begin
            case taicpu(p).opcode of
              A_CMPWI,
              A_CMPLWI:
                begin
                  if (taicpu(p).oper[1]^.typ = top_const) and
                     (taicpu(p).oper[1]^.val = 0) and
                     getnextinstruction(p,next1) and
                     (next1.typ = ait_instruction) and
                     (taicpu(next1).opcode = A_MFCR) and
                     getnextinstruction(next1,next2) and
                     (taicpu(next2).opcode = A_RLWINM) and
                     (taicpu(next2).oper[0]^.reg = taicpu(next2).oper[1]^.reg) and
                     (taicpu(next2).oper[0]^.reg = taicpu(next1).oper[0]^.reg) and
                     (taicpu(next2).oper[3]^.val = 31) and
                     (taicpu(next2).oper[4]^.val = 31) and
                     cmpi_mfcr_opt(taicpu(p),taicpu(next1),taicpu(next2)) then
                    result := true;
                end;
{ seems the register allocator doesn't generate superfluous fmr's }
{              A_FMR, }
              A_MR:
                begin
                  if getnextinstruction(p,next1) and
                     (next1.typ = ait_instruction) and
                     (taicpu(next1).ops >= 1) and
                     { spilling_get_operation_type does not support lmw/stmw }
                     (taicpu(next1).opcode <> A_LMW) and
                     (taicpu(next1).opcode <> A_STMW) and
                     (taicpu(next1).spilling_get_operation_type(0) = operand_write) and
                     (taicpu(next1).oper[0]^.reg = taicpu(p).oper[0]^.reg) then
                    begin
                      for l1 := 1 to taicpu(next1).ops - 1 do
                        case taicpu(next1).oper[l1]^.typ of
                          top_reg:
                            if taicpu(next1).oper[l1]^.reg = taicpu(p).oper[0]^.reg then
                              taicpu(next1).loadreg(l1,taicpu(p).oper[1]^.reg);
                          top_ref:
                            begin
                              if taicpu(next1).oper[l1]^.ref^.base = taicpu(p).oper[0]^.reg then
                                taicpu(next1).oper[l1]^.ref^.base := taicpu(p).oper[1]^.reg;
                              if taicpu(next1).oper[l1]^.ref^.index = taicpu(p).oper[0]^.reg then
                                taicpu(next1).oper[l1]^.ref^.index := taicpu(p).oper[1]^.reg;
                            end;
                          else
                            ;
                        end;
                      asml.remove(p);
                      p.free;
                      p := next1;
                      result := true;
                    end;
                end;
              A_SLWI:
                begin
                  if getnextinstruction(p,next1) and
                     (next1.typ = ait_instruction) and
                     ((taicpu(next1).opcode = A_RLWINM) or
                      (taicpu(next1).opcode = A_SLWI) or
                      (taicpu(next1).opcode = A_SRWI)) and
                     (taicpu(next1).oper[0]^.reg = taicpu(p).oper[0]^.reg) and
                     (taicpu(next1).oper[1]^.reg = taicpu(p).oper[0]^.reg) then
                    begin
                      { convert slwi to rlwinm and see if the rlwinm }
                      { optimization can do something with it        }
                      taicpu(p).opcode := A_RLWINM;
                      taicpu(p).ops := 5;
                      taicpu(p).loadconst(2,taicpu(p).oper[2]^.val);
                      taicpu(p).loadconst(3,0);
                      taicpu(p).loadconst(4,31-taicpu(p).oper[2]^.val);
                      result := true;
                    end;
                end;
              A_SRWI:
                begin
                  if getnextinstruction(p,next1) and
                     (next1.typ = ait_instruction) and
                     ((taicpu(next1).opcode = A_SLWI) or
                      (taicpu(next1).opcode = A_RLWINM) or
                      (taicpu(next1).opcode = A_SRWI)) and
                     (taicpu(next1).oper[0]^.reg = taicpu(p).oper[0]^.reg) and
                     (taicpu(next1).oper[1]^.reg = taicpu(p).oper[0]^.reg) then
                    case taicpu(next1).opcode of
                      A_SLWI:
                        begin
                          taicpu(p).opcode := A_RLWINM;
                          taicpu(p).ops := 5;
                          taicpu(p).loadconst(2,taicpu(next1).oper[2]^.val-taicpu(p).oper[2]^.val);
                          if (taicpu(p).oper[2]^.val < 0) then
                            begin
                              taicpu(p).loadconst(3,-taicpu(p).oper[2]^.val);
                              taicpu(p).loadconst(4,31-taicpu(next1).oper[2]^.val);
                              inc(taicpu(p).oper[2]^.val,32);
                            end
                          else
                            begin
                              taicpu(p).loadconst(3,0);
                              taicpu(p).loadconst(4,31-taicpu(next1).oper[2]^.val);
                            end;
                          asml.remove(next1);
                          next1.free;
                          result := true;
                        end;
                      A_RLWINM:
                        begin
                          { convert srwi to rlwinm and see if the rlwinm }
                          { optimization can do something with it        }
                          taicpu(p).opcode := A_RLWINM;
                          taicpu(p).ops := 5;
                          taicpu(p).loadconst(3,taicpu(p).oper[2]^.val);
                          taicpu(p).loadconst(4,31);
                          taicpu(p).loadconst(2,(32-taicpu(p).oper[2]^.val) and 31);
                          result := true;
                        end;
                      else
                        internalerror(2019050941);
                    end;
                end;
              A_RLWINM:
                begin
                  if getnextinstruction(p,next1) and
                     (next1.typ = ait_instruction) and
                     ((taicpu(next1).opcode = A_RLWINM) or
                      (taicpu(next1).opcode = A_SRWI) or
                      (taicpu(next1).opcode = A_SLWI)) and
                     (taicpu(next1).oper[0]^.reg = taicpu(p).oper[0]^.reg) and
                     // both source and target of next1 must equal target of p
                     (taicpu(next1).oper[1]^.reg = taicpu(p).oper[0]^.reg) then
                    begin
                      case taicpu(next1).opcode of
                        A_RLWINM:
                          begin
                            shlcount := taicpu(next1).oper[2]^.val;
                            l2 := rlwinm2mask(taicpu(next1).oper[3]^.val,taicpu(next1).oper[4]^.val);
                          end;
                        A_SLWI:
                          begin
                            shlcount := taicpu(next1).oper[2]^.val;
                            l2 := (-1) shl shlcount;
                          end;
                        A_SRWI:
                          begin
                            shlcount := 32-taicpu(next1).oper[2]^.val;
                            l2 := (-1) shr taicpu(next1).oper[2]^.val;
                          end;
                        else
                          internalerror(2013113008);
                      end;
                      l1 := rlwinm2mask((taicpu(p).oper[3]^.val-shlcount) and 31,(taicpu(p).oper[4]^.val-shlcount) and 31);
                      l1 := l1 and l2;
                      case l1 of
                        -1:
                          begin
                            taicpu(p).oper[2]^.val := (taicpu(p).oper[2]^.val + shlcount) and 31;
                            asml.remove(next1);
                            next1.free;
                            if (taicpu(p).oper[2]^.val = 0) then
                              begin
                                next1 := tai(p.next);
                                asml.remove(p);
                                p.free;
                                p := next1;
                                result := true;
                              end;
                          end;
                        0:
                          begin
                            // masks have no bits in common
                            taicpu(p).opcode := A_LI;
                            taicpu(p).loadconst(1,0);
                            taicpu(p).freeop(2);
                            taicpu(p).freeop(3);
                            taicpu(p).freeop(4);
                            taicpu(p).ops := 2;
                            taicpu(p).opercnt := 2;
                            asml.remove(next1);
                            next1.free;
                            result := true;
                          end
                        else if tcgppc(cg).get_rlwi_const(l1,l1,l2) then
                          begin
                            taicpu(p).oper[2]^.val := (taicpu(p).oper[2]^.val + shlcount) and 31;
                            taicpu(p).oper[3]^.val := l1;
                            taicpu(p).oper[4]^.val := l2;
                            asml.remove(next1);
                            next1.free;
                            result := true;
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
    end;


  const
    modifyflags: array[tasmop] of tasmop =
      (a_none, a_add_, a_add_, a_addo_, a_addo_, a_addc_, a_addc_, a_addco_, a_addco_,
      a_adde_, a_adde_, a_addeo_, a_addeo_, {a_addi could be addic_ if sure doesn't disturb carry} a_none, a_addic_, a_addic_, a_none,
      a_addme_, a_addme_, a_addmeo_, a_addmeo_, a_addze_, a_addze_, a_addzeo_,
      a_addzeo_, a_and_, a_and_, a_andc_, a_andc_, a_andi_, a_andis_, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_cntlzw_, a_cntlzw_, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_divw_, a_divw_, a_divwo_, a_divwo_,
      a_divwu_, a_divwu_, a_divwuo_, a_divwuo_, a_none, a_none, a_none, a_eqv_,
      a_eqv_, a_extsb_, a_extsb_, a_extsh_, a_extsh_, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_mffs, a_mffs_, a_mfmsr, a_mfspr, a_mfsr,
      a_mfsrin, a_mftb, a_mtcrf, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_mulhw_,
      a_mulhw_, a_mulhwu_, a_mulhwu_, a_none, a_mullw_, a_mullw_, a_mullwo_,
      a_mullwo_, a_nand_, a_nand_, a_neg_, a_neg_, a_nego_, a_nego_, a_nor_, a_nor_,
      a_or_, a_or_, a_orc_, a_orc_, a_none, a_none, a_none, a_rlwimi_, a_rlwimi_,
      a_rlwinm_, a_rlwinm_, a_rlwnm_, a_rlwnm_, a_none, a_slw_, a_slw_, a_sraw_, a_sraw_,
      a_srawi_, a_srawi_,a_srw_, a_srw_, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none, a_subf_, a_subf_, a_subfo_,
      a_subfo_, a_subfc_, a_subfc_, a_subfco_, a_subfco_, a_subfe_, a_subfe_,
      a_subfeo_, a_subfeo_, a_none, a_subfme_, a_subfme_, a_subfmeo_, a_subfmeo_,
      a_subfze_, a_subfze_, a_subfzeo_, a_subfzeo_, a_none, a_none, a_none,
      a_none, a_none, a_none, a_xor_, a_xor_, a_none, a_none,
      { simplified mnemonics }
      a_none, a_none, a_subic_, a_subic_, a_sub_, a_sub_, a_subo_, a_subo_,
      a_subc_, a_subc_, a_subco_, a_subco_, a_none, a_none, a_none, a_none,
      a_extlwi_, a_extlwi_, a_extrwi_, a_extrwi_, a_inslwi_, a_inslwi_, a_insrwi_,
      a_insrwi_, a_rotlwi_, a_rotlwi_, a_rotlw_, a_rotlw_, a_slwi_, a_slwi_,
      a_srwi_, a_srwi_, a_clrlwi_, a_clrlwi_, a_clrrwi_, a_clrrwi_, a_clrslwi_,
      a_clrslwi_, a_none, a_none, a_none, a_none, a_none, a_none, a_none,
      a_none, a_none {move to special purpose reg}, a_none {move from special purpose reg},
      a_none, a_none, a_none, a_none, a_mr_, a_mr_, a_not_, a_not_, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none,
      a_none, a_none, a_none, a_none, a_none);

  function changetomodifyflags(p: taicpu): boolean;
    begin
      result := false;
      if (modifyflags[p.opcode] <> a_none) then
        begin
          p.opcode := modifyflags[p.opcode];
          result := true;
        end;
    end;

  function TCpuAsmOptimizer.PostPeepHoleOptsCpu(var p: tai): boolean;
    var
      next1: tai;
    begin
      result := false;
      case p.typ of
        ait_instruction:
          begin
            case taicpu(p).opcode of
              A_RLWINM_:
                begin
                  // rlwinm_ is cracked on the G5, andi_/andis_ aren't
                  if (taicpu(p).oper[2]^.val = 0) then
                    if (taicpu(p).oper[3]^.val < 16) and
                       (taicpu(p).oper[4]^.val < 16) then
                      begin
                        taicpu(p).opcode := A_ANDIS_;
                        taicpu(p).oper[2]^.val := word(
                          ((1 shl (16-taicpu(p).oper[3]^.val)) - 1) xor
                          ((1 shl (15-taicpu(p).oper[4]^.val)) - 1));
                        taicpu(p).freeop(3);
                        taicpu(p).freeop(4);
                        taicpu(p).ops := 3;
                        taicpu(p).opercnt := 3;
                      end
                    else if (taicpu(p).oper[3]^.val >= 16) and
                       (taicpu(p).oper[4]^.val >= 16) then
                      begin
                        taicpu(p).opcode := A_ANDI_;
                        taicpu(p).oper[2]^.val := word(rlwinm2mask(taicpu(p).oper[3]^.val,taicpu(p).oper[4]^.val));
                        taicpu(p).freeop(3);
                        taicpu(p).freeop(4);
                        taicpu(p).ops := 3;
                        taicpu(p).opercnt := 3;
                      end;
                end;
              else
                ;
            end;

            // change "integer operation with destination reg" followed by a
            // comparison to zero of that reg, with a variant of that integer
            // operation which sets the flags (if it exists)
            if not(result) and
               (taicpu(p).ops >= 2) and
               (taicpu(p).oper[0]^.typ = top_reg) and
               (taicpu(p).oper[1]^.typ = top_reg) and
               getnextinstruction(p,next1) and
               (next1.typ = ait_instruction) and
               (taicpu(next1).opcode = A_CMPWI) and
               // make sure it the result goes to cr0
               (((taicpu(next1).ops = 2) and
                 (taicpu(next1).oper[1]^.val = 0) and
                 (taicpu(next1).oper[0]^.reg = taicpu(p).oper[0]^.reg)) or
                ((taicpu(next1).ops = 3) and
                 (taicpu(next1).oper[2]^.val = 0) and
                 (taicpu(next1).oper[0]^.typ = top_reg) and
                 (getsupreg(taicpu(next1).oper[0]^.reg) = RS_CR0) and
                 (taicpu(next1).oper[1]^.reg = taicpu(p).oper[0]^.reg))) and
               changetomodifyflags(taicpu(p)) then
              begin
                asml.remove(next1);
                next1.free;
                result := true;
              end;
          end;
        else
          ;
      end;
    end;

begin
  casmoptimizer:=TCpuAsmOptimizer;
End.
