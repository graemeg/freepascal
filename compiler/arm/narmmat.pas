{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate ARM assembler for math nodes

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
unit narmmat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat;

    type
      tarmmoddivnode = class(tmoddivnode)
        function first_moddivint: tnode;override;
        procedure pass_generate_code;override;
      end;

      tarmnotnode = class(tcgnotnode)
        procedure second_boolean;override;
      end;

      tarmunaryminusnode = class(tcgunaryminusnode)
        function pass_1: tnode; override;
        procedure second_float;override;
      end;

      tarmshlshrnode = class(tcgshlshrnode)
         procedure second_64bit;override;
         function first_shlshr64bitint: tnode; override;
      end;

implementation

    uses
      globtype,compinnr,
      cutils,verbose,globals,constexp,
      aasmbase,aasmcpu,aasmtai,aasmdata,
      defutil,
      symtype,symconst,symtable,
      cgbase,cgobj,hlcgobj,cgutils,
      pass_2,procinfo,
      ncon,ncnv,ncal,ninl,
      cpubase,cpuinfo,
      ncgutil,
      nadd,pass_1,symdef;

{*****************************************************************************
                             TARMMODDIVNODE
*****************************************************************************}

    function tarmmoddivnode.first_moddivint: tnode;
      var
        power  : longint;
      begin
        {We can handle all cases of constant division}
        if not(cs_check_overflow in current_settings.localswitches) and
           (right.nodetype=ordconstn) and
           (nodetype=divn) and
           not(is_64bit(resultdef)) and
           {Only the ARM and thumb2-isa support umull and smull, which are required for arbitary division by const optimization}
           (GenerateArmCode or
            GenerateThumb2Code or
            (ispowerof2(tordconstnode(right).value,power) or
            (tordconstnode(right).value=1) or
            (tordconstnode(right).value=int64(-1))
            )
           ) then
          result:=nil
        else if ((GenerateThumbCode or GenerateThumb2Code) and (CPUARM_HAS_THUMB_IDIV in cpu_capabilities[current_settings.cputype])) and
          (nodetype=divn) and
          not(is_64bit(resultdef)) then
          result:=nil
        else if ((GenerateThumbCode or GenerateThumb2Code) and (CPUARM_HAS_THUMB_IDIV in cpu_capabilities[current_settings.cputype])) and
          (nodetype=modn) and
          not(is_64bit(resultdef)) then
          begin
            if (right.nodetype=ordconstn) and
              ispowerof2(tordconstnode(right).value,power) and
              (tordconstnode(right).value<=256) and
              (tordconstnode(right).value>0) then
              result:=caddnode.create_internal(andn,left,cordconstnode.create(tordconstnode(right).value-1,sinttype,false))
            else
              begin
                result:=caddnode.create_internal(subn,left,caddnode.create_internal(muln,right,cmoddivnode.Create(divn,left.getcopy,right.getcopy)));
                right:=nil;
              end;
            left:=nil;
            firstpass(result);
          end
        else if (nodetype=modn) and
          (is_signed(left.resultdef)) and
          (right.nodetype=ordconstn) and
          (tordconstnode(right).value=2) then
          begin
            // result:=(0-(left and 1)) and (1+(sarlongint(left,31) shl 1))
            result:=caddnode.create_internal(andn,caddnode.create_internal(subn,cordconstnode.create(0,sinttype,false),caddnode.create_internal(andn,left,cordconstnode.create(1,sinttype,false))),
                                         caddnode.create_internal(addn,cordconstnode.create(1,sinttype,false),
                                                              cshlshrnode.create(shln,cinlinenode.create(in_sar_x_y,false,ccallparanode.create(cordconstnode.create(31,sinttype,false),ccallparanode.Create(left.getcopy,nil))),cordconstnode.create(1,sinttype,false))));
            left:=nil;
            firstpass(result);
          end
        else
          result:=inherited first_moddivint;

        { we may not change the result type here }
        if assigned(result) and (torddef(result.resultdef).ordtype<>torddef(resultdef).ordtype) then
          inserttypeconv(result,resultdef);
      end;


    procedure tarmmoddivnode.pass_generate_code;
      var
        power  : longint;
        numerator,
        helper1,
        helper2,
        resultreg  : tregister;
        size       : Tcgsize;
        so : tshifterop;

       procedure genOrdConstNodeDiv;
         begin
           if tordconstnode(right).value=0 then
             internalerror(2005061701)
           else if tordconstnode(right).value=1 then
             cg.a_load_reg_reg(current_asmdata.CurrAsmList, OS_INT, OS_INT, numerator, resultreg)
           else if (tordconstnode(right).value = int64(-1)) then
             begin
               // note: only in the signed case possible..., may overflow
               if cs_check_overflow in current_settings.localswitches then
                 cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);

               current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_MVN,
                 resultreg,numerator),toppostfix(ord(cs_check_overflow in current_settings.localswitches)*ord(PF_S))));
             end
           else if ispowerof2(tordconstnode(right).value,power) then
             begin
               if (is_signed(right.resultdef)) then
                 begin
                    helper1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                    helper2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                    if power = 1 then
                      cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,numerator,helper1)
                    else
                      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,OS_INT,31,numerator,helper1);
                    if GenerateThumbCode then
                      begin
                        cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,OS_INT,32-power,helper1);
                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ADD,helper2,numerator,helper1));
                      end
                    else
                      begin
                        shifterop_reset(so);
                        so.shiftmode:=SM_LSR;
                        so.shiftimm:=32-power;
                        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,helper2,numerator,helper1,so));
                      end;
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,OS_INT,power,helper2,resultreg);
                  end
               else
                 cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_INT,power,numerator,resultreg)
             end
           else if CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype] then
             {Everything else is handled the generic code}
             cg.g_div_const_reg_reg(current_asmdata.CurrAsmList,def_cgsize(resultdef),
               tordconstnode(right).value.svalue,numerator,resultreg)
           else
             internalerror(2019012601);
         end;

{
       procedure genOrdConstNodeMod;
         var
             modreg, maskreg, tempreg : tregister;
         begin
             if (tordconstnode(right).value = 0) then begin
                 internalerror(2005061702);
             end
             else if (abs(tordconstnode(right).value.svalue) = 1) then
             begin
                // x mod +/-1 is always zero
                cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, 0, resultreg);
             end
             else if (ispowerof2(tordconstnode(right).value, power)) then
             begin
                 if (is_signed(right.resultdef)) then begin

                     tempreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
                     maskreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
                     modreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);

                     cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, abs(tordconstnode(right).value.svalue)-1, modreg);
                     cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_SAR, OS_INT, 31, numerator, maskreg);
                     cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_AND, OS_INT, numerator, modreg, tempreg);

                     current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ANDC, maskreg, maskreg, modreg));
                     current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_SUBFIC, modreg, tempreg, 0));
                     current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SUBFE, modreg, modreg, modreg));
                     cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_AND, OS_INT, modreg, maskreg, maskreg);
                     cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_OR, OS_INT, maskreg, tempreg, resultreg);
                 end else begin
                     cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_AND, OS_INT, tordconstnode(right).value.svalue-1, numerator, resultreg);
                 end;
             end else begin
                 genOrdConstNodeDiv();
                 cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_MUL, OS_INT, tordconstnode(right).value.svalue, resultreg, resultreg);
                 cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_SUB, OS_INT, resultreg, numerator, resultreg);
             end;
         end;
}

      begin
        secondpass(left);
        secondpass(right);

        if ((GenerateThumbCode or GenerateThumb2Code) and (CPUARM_HAS_THUMB_IDIV in cpu_capabilities[current_settings.cputype])) and
           (nodetype=divn) and
           not(is_64bitint(resultdef)) then
          begin
            size:=def_cgsize(left.resultdef);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

            location_copy(location,left.location);
            location.loc := LOC_REGISTER;
            location.register := cg.getintregister(current_asmdata.CurrAsmList,size);
            resultreg:=location.register;

            if (right.nodetype=ordconstn) and
               ((tordconstnode(right).value=1) or
                (tordconstnode(right).value=int64(-1)) or
                (tordconstnode(right).value=0) or
                ispowerof2(tordconstnode(right).value,power)) then
              begin
                numerator:=left.location.register;

                genOrdConstNodeDiv;
              end
            else
              begin
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,left.resultdef,true);

                if is_signed(left.resultdef) or
                   is_signed(right.resultdef) then
                  cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_IDIV,OS_INT,right.location.register,left.location.register,location.register)
                else
                  cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_DIV,OS_INT,right.location.register,left.location.register,location.register);
              end;
          end
        else
          begin
            location_copy(location,left.location);

            { put numerator in register }
            size:=def_cgsize(left.resultdef);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,
              left.resultdef,left.resultdef,true);
            location_copy(location,left.location);
            numerator:=location.register;
            resultreg:=location.register;
            if location.loc=LOC_CREGISTER then
              begin
                location.loc := LOC_REGISTER;
                location.register := cg.getintregister(current_asmdata.CurrAsmList,size);
                resultreg:=location.register;
              end
            else if (nodetype=modn) or (right.nodetype=ordconstn) then
              begin
                // for a modulus op, and for const nodes we need the result register
                // to be an extra register
                resultreg:=cg.getintregister(current_asmdata.CurrAsmList,size);
              end;

            if (right.nodetype=ordconstn) then
              begin
                if nodetype=divn then
                  genOrdConstNodeDiv
                else
    //              genOrdConstNodeMod;
              end;

            location.register:=resultreg;
          end;

        { unsigned division/module can only overflow in case of division by zero }
        { (but checking this overflow flag is more convoluted than performing a  }
        {  simple comparison with 0)                                             }
        if is_signed(right.resultdef) then
          cg.g_overflowcheck(current_asmdata.CurrAsmList,location,resultdef);
      end;

{*****************************************************************************
                               TARMNOTNODE
*****************************************************************************}

    procedure tarmnotnode.second_boolean;
      var
        tmpreg : TRegister;
      begin
        { if the location is LOC_JUMP, we do the secondpass after the
          labels are allocated
        }
        if not handle_locjump then
          begin
            secondpass(left);
            case left.location.loc of
              LOC_FLAGS :
                begin
                  location_copy(location,left.location);
                  inverse_flags(location.resflags);
                end;
              LOC_REGISTER,LOC_CREGISTER,LOC_REFERENCE,LOC_CREFERENCE,
              LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF :
                begin
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  if is_64bit(resultdef) then
                    begin
                      tmpreg:=cg.GetIntRegister(current_asmdata.CurrAsmList,OS_INT);
                      { OR low and high parts together }
                      current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ORR,tmpreg,left.location.register64.reglo,left.location.register64.reghi),PF_S));
                    end
                  else
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,left.location.register,0));
                  location_reset(location,LOC_FLAGS,OS_NO);
                  location.resflags:=F_EQ;
                end;
              else
                internalerror(2003042401);
            end;
          end;
      end;

{*****************************************************************************
                               TARMUNARYMINUSNODE
*****************************************************************************}

    function tarmunaryminusnode.pass_1: tnode;
      var
        procname: string[31];
        fdef : tdef;
      begin
        if (current_settings.fputype=fpu_soft) and
           (left.resultdef.typ=floatdef) then
          begin
            result:=nil;
            firstpass(left);
            expectloc:=LOC_REGISTER;
            exit;
          end;

        if (FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype]) or
          (tfloatdef(resultdef).floattype=s32real) then
          exit(inherited pass_1);

        result:=nil;
        firstpass(left);
        if codegenerror then
          exit;

        if (left.resultdef.typ=floatdef) then
          begin
            case tfloatdef(resultdef).floattype of
              s64real:
                begin
                  procname:='float64_sub';
                  fdef:=search_system_type('FLOAT64').typedef;
                end;
              else
                internalerror(2005082801);
            end;
            result:=ctypeconvnode.create_internal(ccallnode.createintern(procname,ccallparanode.create(
              ctypeconvnode.create_internal(left,fDef),
              ccallparanode.create(ctypeconvnode.create_internal(crealconstnode.create(0,resultdef),fdef),nil))),resultdef);

            left:=nil;
          end
        else
          begin
            if (left.resultdef.typ=floatdef) then
              expectloc:=LOC_FPUREGISTER
             else if (left.resultdef.typ=orddef) then
               expectloc:=LOC_REGISTER;
          end;
      end;

    procedure tarmunaryminusnode.second_float;
      var
        pf: TOpPostfix;
      begin
        secondpass(left);
        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            begin
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
              location:=left.location;
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_const(A_RSF,
                location.register,left.location.register,0),
                cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_soft:
            begin
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
              location:=left.location;
              case location.size of
                OS_32:
                  cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_XOR,OS_32,tcgint($80000000),location.register);
                OS_64:
                  cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_XOR,OS_32,tcgint($80000000),location.registerhi);
              else
                internalerror(2014033101);
              end;
            end
          else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[init_settings.fputype] then
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              location:=left.location;
              if (left.location.loc=LOC_CMMREGISTER) then
                location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              if (tfloatdef(left.resultdef).floattype=s32real) then
                pf:=PF_F32
              else
                pf:=PF_F64;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VNEG,
                location.register,left.location.register), pf));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[init_settings.fputype] then
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              location:=left.location;
              if (left.location.loc=LOC_CMMREGISTER) then
                location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_VNEG,
                location.register,left.location.register), PF_F32));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else
            internalerror(2009112602);
        end;
      end;

    function tarmshlshrnode.first_shlshr64bitint: tnode;
      begin
        if GenerateThumbCode or GenerateThumb2Code then
          result:=inherited
        else
          result := nil;
      end;

    procedure tarmshlshrnode.second_64bit;
      var
        v : TConstExprInt;
        so: tshifterop;
        lreg, resreg: TRegister64;

      procedure emit_instr(p: tai);
        begin
          current_asmdata.CurrAsmList.concat(p);
        end;

      {This code is build like it gets called with sm=SM_LSR all the time, for SM_LSL dst* and src* have to be reversed}
      procedure shift_less_than_32(srchi, srclo, dsthi, dstlo: TRegister; shiftval: Byte; sm: TShiftMode);
        begin
          shifterop_reset(so);

          so.shiftimm:=shiftval;
          so.shiftmode:=sm;
          emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, dstlo, srclo, so));
          emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, dsthi, srchi, so));

          if sm = SM_LSR then so.shiftmode:=SM_LSL else so.shiftmode:=SM_LSR;
          so.shiftimm:=32-shiftval;
          emit_instr(taicpu.op_reg_reg_reg_shifterop(A_ORR, dstlo, dstlo, srchi, so));

        end;

      {This code is build like it gets called with sm=SM_LSR all the time, for SM_LSL dst* and src* have to be reversed
       This will generate
         mov   shiftval1, shiftval
         cmp   shiftval1, #64
         movcs shiftval1, #64
         rsb   shiftval2, shiftval1, #32
         mov   dstlo, srclo, lsr shiftval1
         mov   dsthi, srchi, lsr shiftval1
         orr   dstlo, srchi, lsl shiftval2
         subs  shiftval2, shiftval1, #32
         movpl dstlo, srchi, lsr shiftval2
      }
      procedure shift_by_variable(srchi, srclo, dsthi, dstlo, shiftval: TRegister; sm: TShiftMode);
        var
          shiftval1,shiftval2:TRegister;
        begin
          shifterop_reset(so);
          shiftval1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          shiftval2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);

          cg.a_load_reg_reg(current_asmdata.CurrAsmList, OS_INT, OS_INT, shiftval, shiftval1);

          {The ARM barrel shifter only considers the lower 8 bits of a register for the shift}
          cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
          emit_instr(taicpu.op_reg_const(A_CMP, shiftval1, 64));
          emit_instr(setcondition(taicpu.op_reg_const(A_MOV, shiftval1, 64), C_CS));
          cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);

          {Calculate how much the upper register needs to be shifted left}
          emit_instr(taicpu.op_reg_reg_const(A_RSB, shiftval2, shiftval1, 32));

          so.shiftmode:=sm;
          so.rs:=shiftval1;

          {Shift and zerofill the hi+lo register}
          emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, dstlo, srclo, so));
          emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, dsthi, srchi, so));

          {Fold in the lower 32-shiftval bits}
          if sm = SM_LSR then so.shiftmode:=SM_LSL else so.shiftmode:=SM_LSR;
          so.rs:=shiftval2;
          emit_instr(taicpu.op_reg_reg_reg_shifterop(A_ORR, dstlo, dstlo, srchi, so));

          cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
          emit_instr(setoppostfix(taicpu.op_reg_reg_const(A_SUB, shiftval2, shiftval1, 32), PF_S));

          so.shiftmode:=sm;
          emit_instr(setcondition(taicpu.op_reg_reg_shifterop(A_MOV, dstlo, srchi, so), C_PL));
          cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        end;

      begin
        if GenerateThumbCode or GenerateThumb2Code then
        begin
          inherited;
          exit;
        end;

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
        location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);

        { load left operator in a register }
        if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
           (left.location.size<>OS_64) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,true);

        lreg := left.location.register64;
        resreg := location.register64;
        shifterop_reset(so);

        { shifting by a constant directly coded: }
        if (right.nodetype=ordconstn) then
          begin
            v:=Tordconstnode(right).value and 63;
            {Single bit shift}
            if v = 1 then
              if nodetype=shln then
                begin
                  {Shift left by one by 2 simple 32bit additions}
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  emit_instr(setoppostfix(taicpu.op_reg_reg_reg(A_ADD, resreg.reglo, lreg.reglo, lreg.reglo), PF_S));
                  emit_instr(taicpu.op_reg_reg_reg(A_ADC, resreg.reghi, lreg.reghi, lreg.reghi));
                  cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                end
              else
                begin
                  {Shift right by first shifting hi by one and then using RRX (rotate right extended), which rotates through the carry}
                  shifterop_reset(so); so.shiftmode:=SM_LSR; so.shiftimm:=1;
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  emit_instr(setoppostfix(taicpu.op_reg_reg_shifterop(A_MOV, resreg.reghi, lreg.reghi, so), PF_S));
                  so.shiftmode:=SM_RRX; so.shiftimm:=0; {RRX does NOT have a shift amount}
                  emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, resreg.reglo, lreg.reglo, so));
                  cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                end
            {Clear one register and use the cg to generate a normal 32-bit shift}
            else if v >= 32 then
              if nodetype=shln then
              begin
                emit_instr(taicpu.op_reg_const(A_MOV, resreg.reglo, 0));
                cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHL,OS_32,v.uvalue-32,lreg.reglo,resreg.reghi);
              end
              else
              begin
                emit_instr(taicpu.op_reg_const(A_MOV, resreg.reghi, 0));
                cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_32,v.uvalue-32,lreg.reghi,resreg.reglo);
              end
            {Shift LESS than 32, thats the tricky one}
            else if (v < 32) and (v > 1) then
              if nodetype=shln then
                shift_less_than_32(lreg.reglo, lreg.reghi, resreg.reglo, resreg.reghi, v.uvalue, SM_LSL)
              else
                shift_less_than_32(lreg.reghi, lreg.reglo, resreg.reghi, resreg.reglo, v.uvalue, SM_LSR);
          end
        else
          begin
            { force right operator into a register }
            if not(right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
               (right.location.size<>OS_32) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,u32inttype,true);

            if nodetype = shln then
              shift_by_variable(lreg.reglo, lreg.reghi, resreg.reglo, resreg.reghi, right.location.register, SM_LSL)
            else
              shift_by_variable(lreg.reghi, lreg.reglo, resreg.reghi, resreg.reglo, right.location.register, SM_LSR);
          end;
      end;


begin
  cmoddivnode:=tarmmoddivnode;
  cnotnode:=tarmnotnode;
  cunaryminusnode:=tarmunaryminusnode;
  cshlshrnode:=tarmshlshrnode;
end.
