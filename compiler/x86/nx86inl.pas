{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate x86 inline nodes

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
unit nx86inl;

{$i fpcdefs.inc}

interface

    uses
       node,ninl,ncginl;

    type
       tx86inlinenode = class(tcginlinenode)
         protected
          procedure maybe_remove_round_trunc_typeconv; virtual;
         public
          { first pass override
            so that the code generator will actually generate
            these nodes.
          }
          function first_pi: tnode ; override;
          function first_arctan_real: tnode; override;
          function first_abs_real: tnode; override;
          function first_sqr_real: tnode; override;
          function first_sqrt_real: tnode; override;
          function first_ln_real: tnode; override;
          function first_cos_real: tnode; override;
          function first_sin_real: tnode; override;
          function first_round_real: tnode; override;
          function first_trunc_real: tnode; override;
          function first_popcnt: tnode; override;
          function first_fma: tnode; override;
          { second pass override to generate these nodes }
          procedure second_IncludeExclude;override;
          procedure second_pi; override;
          procedure second_arctan_real; override;
          procedure second_abs_real; override;
          procedure second_round_real; override;
          procedure second_sqr_real; override;
          procedure second_sqrt_real; override;
          procedure second_ln_real; override;
          procedure second_cos_real; override;
          procedure second_sin_real; override;
          procedure second_trunc_real; override;

          procedure second_prefetch;override;

{$ifndef i8086}
          procedure second_abs_long;override;
{$endif not i8086}
          procedure second_popcnt;override;
          procedure second_fma;override;
       private
          procedure load_fpu_location(lnode: tnode);
       end;

implementation

    uses
      systems,
      globtype,globals,
      cutils,verbose,
      symconst,
      defutil,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      symtype,symdef,symcpu,
      cgbase,pass_2,
      cpuinfo,cpubase,paramgr,
      nbas,ncon,ncal,ncnv,nld,ncgutil,
      tgobj,
      cga,cgutils,cgx86,cgobj,hlcgobj;


{*****************************************************************************
                              TX86INLINENODE
*****************************************************************************}

     procedure tx86inlinenode.maybe_remove_round_trunc_typeconv;
       begin
         { only makes a difference for x86_64 }
       end;


     function tx86inlinenode.first_pi : tnode;
      begin
        if (tfloatdef(pbestrealtype^).floattype=s80real) then
          begin
            expectloc:=LOC_FPUREGISTER;
            first_pi := nil;
          end
        else
          result:=inherited;
      end;


     function tx86inlinenode.first_arctan_real : tnode;
      begin
{$ifdef i8086}
        { FPATAN's range is limited to (0 <= value < 1) on the 8087 and 80287,
          so we need to use the RTL helper on these FPUs }
        if current_settings.cputype < cpu_386 then
          begin
            result := inherited;
            exit;
          end;
{$endif i8086}
        if (tfloatdef(pbestrealtype^).floattype=s80real) then
          begin
            expectloc:=LOC_FPUREGISTER;
            first_arctan_real := nil;
          end
        else
          result:=inherited;
      end;

     function tx86inlinenode.first_abs_real : tnode;
       begin
         if use_vectorfpu(resultdef) then
           expectloc:=LOC_MMREGISTER
         else
           expectloc:=LOC_FPUREGISTER;
        first_abs_real := nil;
      end;

     function tx86inlinenode.first_sqr_real : tnode;
      begin
        if use_vectorfpu(resultdef) then
          expectloc:=LOC_MMREGISTER
        else
          expectloc:=LOC_FPUREGISTER;
        first_sqr_real := nil;
      end;

     function tx86inlinenode.first_sqrt_real : tnode;
      begin
        if use_vectorfpu(resultdef) then
          expectloc:=LOC_MMREGISTER
        else
          expectloc:=LOC_FPUREGISTER;
        first_sqrt_real := nil;
      end;

     function tx86inlinenode.first_ln_real : tnode;
      begin
        if (tfloatdef(pbestrealtype^).floattype=s80real) then
          begin
            expectloc:=LOC_FPUREGISTER;
            first_ln_real := nil;
          end
        else
          result:=inherited;
      end;

     function tx86inlinenode.first_cos_real : tnode;
      begin
{$ifdef i8086}
        { FCOS is 387+ }
        if current_settings.cputype < cpu_386 then
          begin
            result := inherited;
            exit;
          end;
{$endif i8086}
        if (tfloatdef(pbestrealtype^).floattype=s80real) then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited;
      end;

     function tx86inlinenode.first_sin_real : tnode;
      begin
{$ifdef i8086}
        { FSIN is 387+ }
        if current_settings.cputype < cpu_386 then
          begin
            result := inherited;
            exit;
          end;
{$endif i8086}
        if (tfloatdef(pbestrealtype^).floattype=s80real) then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited;
      end;


     function tx86inlinenode.first_round_real : tnode;
      begin
        maybe_remove_round_trunc_typeconv;
{$ifdef x86_64}
        if use_vectorfpu(left.resultdef) then
          expectloc:=LOC_REGISTER
        else
{$endif x86_64}
          expectloc:=LOC_REFERENCE;
        result:=nil;
      end;


     function tx86inlinenode.first_trunc_real: tnode;
       begin
         maybe_remove_round_trunc_typeconv;
         if (cs_opt_size in current_settings.optimizerswitches)
{$ifdef x86_64}
           and not(use_vectorfpu(left.resultdef))
{$endif x86_64}
           then
           result:=inherited
         else
           begin
{$ifdef x86_64}
             if use_vectorfpu(left.resultdef) then
               expectloc:=LOC_REGISTER
             else
{$endif x86_64}
               expectloc:=LOC_REFERENCE;
             result:=nil;
           end;
       end;


     function tx86inlinenode.first_popcnt: tnode;
       begin
         Result:=nil;
{$ifndef i8086}
         if (CPUX86_HAS_POPCNT in cpu_capabilities[current_settings.cputype])
  {$ifdef i386}
            and not is_64bit(left.resultdef)
  {$endif i386}
           then
             expectloc:=LOC_REGISTER
         else
{$endif not i8086}
           Result:=inherited first_popcnt
       end;


     function tx86inlinenode.first_fma : tnode;
       begin
{$ifndef i8086}
         if ((cpu_capabilities[current_settings.cputype]*[CPUX86_HAS_FMA,CPUX86_HAS_FMA4])<>[]) and
           ((is_double(resultdef)) or (is_single(resultdef))) then
           begin
             expectloc:=LOC_MMREGISTER;
             Result:=nil;
           end
         else
{$endif i8086}
           Result:=inherited first_fma;
       end;


     procedure tx86inlinenode.second_pi;
       begin
         location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
         emit_none(A_FLDPI,S_NO);
         tcgx86(cg).inc_fpu_stack;
         location.register:=NR_FPU_RESULT_REG;
       end;

     { load the FPU into the an fpu register }
     procedure tx86inlinenode.load_fpu_location(lnode: tnode);
       begin
         location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
         location.register:=NR_FPU_RESULT_REG;
         secondpass(lnode);
         case lnode.location.loc of
           LOC_FPUREGISTER:
             ;
           LOC_CFPUREGISTER:
             begin
               cg.a_loadfpu_reg_reg(current_asmdata.CurrAsmList,lnode.location.size,
                 lnode.location.size,lnode.location.register,location.register);
             end;
           LOC_REFERENCE,LOC_CREFERENCE:
             begin
               cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList,
                  lnode.location.size,lnode.location.size,
                  lnode.location.reference,location.register);
             end;
           LOC_MMREGISTER,LOC_CMMREGISTER:
             begin
               location:=lnode.location;
               hlcg.location_force_fpureg(current_asmdata.CurrAsmList,location,resultdef,false);
             end;
           else
             internalerror(309991);
         end;
       end;


     procedure tx86inlinenode.second_arctan_real;
       begin
         load_fpu_location(left);
         emit_none(A_FLD1,S_NO);
         emit_none(A_FPATAN,S_NO);
       end;


     procedure tx86inlinenode.second_abs_real;
       var
         href : treference;
       begin
         if use_vectorfpu(resultdef) then
           begin
             secondpass(left);
             if left.location.loc<>LOC_MMREGISTER then
               hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
             if UseAVX then
               begin
                 location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
                 location.register:=cg.getmmregister(current_asmdata.CurrAsmList,def_cgsize(resultdef));
               end
             else
               location:=left.location;
             case tfloatdef(resultdef).floattype of
               s32real:
                 begin
                   reference_reset_symbol(href,current_asmdata.RefAsmSymbol(target_info.cprefix+'FPC_ABSMASK_SINGLE'),0,4);
                   tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList, href);
                   if UseAVX then
                     current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg_reg(
                       A_VANDPS,S_XMM,href,left.location.register,location.register))
                   else
                     current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_ANDPS,S_XMM,href,location.register));
                 end;
               s64real:
                 begin
                   reference_reset_symbol(href,current_asmdata.RefAsmSymbol(target_info.cprefix+'FPC_ABSMASK_DOUBLE'),0,4);
                   tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList, href);
                   if UseAVX then
                     current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg_reg(
                       A_VANDPD,S_XMM,href,left.location.register,location.register))
                   else
                     current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_ANDPD,S_XMM,href,location.register))
                 end;
               else
                 internalerror(200506081);
             end;
           end
         else
           begin
             load_fpu_location(left);
             emit_none(A_FABS,S_NO);
           end;
       end;


     procedure tx86inlinenode.second_round_real;
       begin
{$ifdef x86_64}
         if use_vectorfpu(left.resultdef) then
           begin
             secondpass(left);
             hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
             location_reset(location,LOC_REGISTER,OS_S64);
             location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_S64);
             if UseAVX then
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTSS2SI,S_Q,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTSD2SI,S_Q,left.location.register,location.register));
                 else
                   internalerror(2007031402);
               end
             else
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTSS2SI,S_Q,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTSD2SI,S_Q,left.location.register,location.register));
                 else
                   internalerror(2007031402);
               end;
           end
         else
{$endif x86_64}
          begin
            load_fpu_location(left);
            location_reset_ref(location,LOC_REFERENCE,OS_S64,0);
            tg.GetTemp(current_asmdata.CurrAsmList,resultdef.size,resultdef.alignment,tt_normal,location.reference);
            emit_ref(A_FISTP,S_IQ,location.reference);
            tcgx86(cg).dec_fpu_stack;
            emit_none(A_FWAIT,S_NO);
           end;
       end;


     procedure tx86inlinenode.second_trunc_real;
       var
         oldcw,newcw : treference;
       begin
{$ifdef x86_64}
         if use_vectorfpu(left.resultdef) and
           not((left.location.loc=LOC_FPUREGISTER) and (current_settings.fputype>=fpu_sse3)) then
           begin
             secondpass(left);
             hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
             location_reset(location,LOC_REGISTER,OS_S64);
             location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_S64);
             if UseAVX then
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTTSS2SI,S_Q,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTTSD2SI,S_Q,left.location.register,location.register));
                 else
                   internalerror(2007031401);
               end
             else
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTTSS2SI,S_Q,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTTSD2SI,S_Q,left.location.register,location.register));
                 else
                   internalerror(2007031401);
               end;
           end
         else
{$endif x86_64}
          begin
            if (current_settings.fputype>=fpu_sse3) then
              begin
                load_fpu_location(left);
                location_reset_ref(location,LOC_REFERENCE,OS_S64,0);
                tg.GetTemp(current_asmdata.CurrAsmList,resultdef.size,resultdef.alignment,tt_normal,location.reference);
                emit_ref(A_FISTTP,S_IQ,location.reference);
                tcgx86(cg).dec_fpu_stack;
              end
            else
              begin
                tg.GetTemp(current_asmdata.CurrAsmList,2,2,tt_normal,oldcw);
                tg.GetTemp(current_asmdata.CurrAsmList,2,2,tt_normal,newcw);
{$ifdef i8086}
                if current_settings.cputype<=cpu_286 then
                  begin
                    emit_ref(A_FSTCW,S_NO,newcw);
                    emit_ref(A_FSTCW,S_NO,oldcw);
                    emit_none(A_FWAIT,S_NO);
                  end
                else
{$endif i8086}
                  begin
                    emit_ref(A_FNSTCW,S_NO,newcw);
                    emit_ref(A_FNSTCW,S_NO,oldcw);
                  end;
                emit_const_ref(A_OR,S_W,$0f00,newcw);
                load_fpu_location(left);
                emit_ref(A_FLDCW,S_NO,newcw);
                location_reset_ref(location,LOC_REFERENCE,OS_S64,0);
                tg.GetTemp(current_asmdata.CurrAsmList,resultdef.size,resultdef.alignment,tt_normal,location.reference);
                emit_ref(A_FISTP,S_IQ,location.reference);
                tcgx86(cg).dec_fpu_stack;
                emit_ref(A_FLDCW,S_NO,oldcw);
                emit_none(A_FWAIT,S_NO);
                tg.UnGetTemp(current_asmdata.CurrAsmList,oldcw);
                tg.UnGetTemp(current_asmdata.CurrAsmList,newcw);
              end;
           end;
       end;


     procedure tx86inlinenode.second_sqr_real;

       begin
         if use_vectorfpu(resultdef) then
           begin
             secondpass(left);
             location_reset(location,LOC_MMREGISTER,left.location.size);
             location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
             if UseAVX then
               begin
                 hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
                 cg.a_opmm_reg_reg_reg(current_asmdata.CurrAsmList,OP_MUL,left.location.size,left.location.register,left.location.register,location.register,mms_movescalar);
               end
             else
               begin
                 if left.location.loc in [LOC_CFPUREGISTER,LOC_FPUREGISTER] then
                   hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
                 cg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,location.size,left.location,location.register,mms_movescalar);
                 cg.a_opmm_reg_reg(current_asmdata.CurrAsmList,OP_MUL,left.location.size,location.register,location.register,mms_movescalar);
               end;
           end
         else
           begin
             load_fpu_location(left);
             emit_reg_reg(A_FMUL,S_NO,NR_ST0,NR_ST0);
           end;
       end;


     procedure tx86inlinenode.second_sqrt_real;
       begin
         if use_vectorfpu(resultdef) then
           begin
             secondpass(left);
             hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
             location_reset(location,LOC_MMREGISTER,left.location.size);
             location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
             if UseAVX then
               case tfloatdef(resultdef).floattype of
                 s32real:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_VSQRTSS,S_XMM,left.location.register,location.register,location.register));
                 s64real:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_VSQRTSD,S_XMM,left.location.register,location.register,location.register));
                 else
                   internalerror(200510031);
               end
             else
               case tfloatdef(resultdef).floattype of
                 s32real:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SQRTSS,S_XMM,left.location.register,location.register));
                 s64real:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SQRTSD,S_XMM,left.location.register,location.register));
                 else
                   internalerror(200510031);
               end;
           end
         else
           begin
             load_fpu_location(left);
             emit_none(A_FSQRT,S_NO);
           end;
       end;

     procedure tx86inlinenode.second_ln_real;
       begin
         load_fpu_location(left);
         emit_none(A_FLDLN2,S_NO);
         emit_none(A_FXCH,S_NO);
         emit_none(A_FYL2X,S_NO);
       end;

     procedure tx86inlinenode.second_cos_real;
       begin
{$ifdef i8086}
       { FCOS is 387+ }
       if current_settings.cputype < cpu_386 then
         begin
           inherited;
           exit;
         end;
{$endif i8086}
         load_fpu_location(left);
         emit_none(A_FCOS,S_NO);
       end;

     procedure tx86inlinenode.second_sin_real;
       begin
{$ifdef i8086}
       { FSIN is 387+ }
       if current_settings.cputype < cpu_386 then
         begin
           inherited;
           exit;
         end;
{$endif i8086}
         load_fpu_location(left);
         emit_none(A_FSIN,S_NO)
       end;

     procedure tx86inlinenode.second_prefetch;
       var
         ref : treference;
         r : tregister;
       begin
{$if defined(i386) or defined(i8086)}
         if current_settings.cputype>=cpu_Pentium3 then
{$endif i386 or i8086}
           begin
             secondpass(left);
             case left.location.loc of
               LOC_CREFERENCE,
               LOC_REFERENCE:
                 begin
                   r:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
                   cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.location.reference,r);
                   reference_reset_base(ref,r,0,left.location.reference.alignment);
                   current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_PREFETCHNTA,S_NO,ref));
                 end;
               else
                 { nothing to prefetch };
             end;
           end;
       end;


{$ifndef i8086}
    procedure tx86inlinenode.second_abs_long;
      var
        hregister : tregister;
        opsize : tcgsize;
        hp : taicpu;
      begin
{$ifdef i386}
        if not(CPUX86_HAS_CMOV in cpu_capabilities[current_settings.cputype]) then
          begin
            opsize:=def_cgsize(left.resultdef);
            secondpass(left);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
            location:=left.location;
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
            emit_reg_reg(A_MOV,S_L,left.location.register,location.register);
            emit_const_reg(A_SAR,tcgsize2opsize[opsize],31,left.location.register);
            emit_reg_reg(A_XOR,S_L,left.location.register,location.register);
            emit_reg_reg(A_SUB,S_L,left.location.register,location.register);
          end
        else
{$endif i386}
          begin
            opsize:=def_cgsize(left.resultdef);
            secondpass(left);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            hregister:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
            location:=left.location;
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
            cg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,opsize,left.location.register,hregister);
            cg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,opsize,left.location.register,location.register);
            emit_reg(A_NEG,tcgsize2opsize[opsize],hregister);
            hp:=taicpu.op_reg_reg(A_CMOVcc,tcgsize2opsize[opsize],hregister,location.register);
            hp.condition:=C_NS;
            current_asmdata.CurrAsmList.concat(hp);
          end;
      end;
{$endif not i8086}

{*****************************************************************************
                     INCLUDE/EXCLUDE GENERIC HANDLING
*****************************************************************************}

      procedure tx86inlinenode.second_IncludeExclude;
        var
         hregister,
         hregister2: tregister;
         setbase   : aint;
         bitsperop,l : longint;
         cgop : topcg;
         asmop : tasmop;
<<<<<<< HEAD
<<<<<<< HEAD
         opdef : tdef;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         opsize,
         orgsize: tcgsize;
        begin
{$ifdef i8086}
          { BTS and BTR are 386+ }
          if current_settings.cputype < cpu_386 then
            begin
              inherited;
              exit;
            end;
{$endif i8086}
          if is_smallset(tcallparanode(left).resultdef) then
            begin
              opdef:=tcallparanode(left).resultdef;
              opsize:=int_cgsize(opdef.size)
            end
          else
            begin
              opdef:=u32inttype;
              opsize:=OS_32;
            end;
          bitsperop:=(8*tcgsize2size[opsize]);
          secondpass(tcallparanode(left).left);
          secondpass(tcallparanode(tcallparanode(left).right).left);
          setbase:=tsetdef(tcallparanode(left).left.resultdef).setbase;
          if tcallparanode(tcallparanode(left).right).left.location.loc=LOC_CONSTANT then
            begin
              { calculate bit position }
              l:=1 shl ((tcallparanode(tcallparanode(left).right).left.location.value-setbase) mod bitsperop);

              { determine operator }
              if inlinenumber=in_include_x_y then
                cgop:=OP_OR
              else
                begin
                  cgop:=OP_AND;
                  l:=not(l);
                end;
              case tcallparanode(left).left.location.loc of
                LOC_REFERENCE :
                  begin
                    inc(tcallparanode(left).left.location.reference.offset,
                      ((tcallparanode(tcallparanode(left).right).left.location.value-setbase) div bitsperop)*tcgsize2size[opsize]);
                    cg.a_op_const_ref(current_asmdata.CurrAsmList,cgop,opsize,l,tcallparanode(left).left.location.reference);
                  end;
                LOC_CREGISTER :
                  cg.a_op_const_reg(current_asmdata.CurrAsmList,cgop,tcallparanode(left).left.location.size,l,tcallparanode(left).left.location.register);
                else
                  internalerror(200405022);
              end;
            end
          else
            begin
              orgsize:=opsize;
              if opsize in [OS_8,OS_S8] then
                begin
<<<<<<< HEAD
<<<<<<< HEAD
                  opdef:=u32inttype;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  opsize:=OS_32;
                end;
              { determine asm operator }
              if inlinenumber=in_include_x_y then
                 asmop:=A_BTS
              else
                 asmop:=A_BTR;

<<<<<<< HEAD
<<<<<<< HEAD
              hlcg.location_force_reg(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,tcallparanode(tcallparanode(left).right).left.resultdef,opdef,true);
              register_maybe_adjust_setbase(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.resultdef,tcallparanode(tcallparanode(left).right).left.location,setbase);
=======
              location_force_reg(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,opsize,true);
              register_maybe_adjust_setbase(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,setbase);
>>>>>>> graemeg/fixes_2_2
=======
              location_force_reg(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,opsize,true);
              register_maybe_adjust_setbase(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,setbase);
>>>>>>> origin/fixes_2_2
              hregister:=tcallparanode(tcallparanode(left).right).left.location.register;
              if (tcallparanode(left).left.location.loc=LOC_REFERENCE) then
                emit_reg_ref(asmop,tcgsize2opsize[opsize],hregister,tcallparanode(left).left.location.reference)
              else
                begin
                  { second argument can't be an 8 bit register either }
                  hregister2:=tcallparanode(left).left.location.register;
                  if (orgsize in [OS_8,OS_S8]) then
                    hregister2:=cg.makeregsize(current_asmdata.CurrAsmList,hregister2,opsize);
                  emit_reg_reg(asmop,tcgsize2opsize[opsize],hregister,hregister2);
                end;
            end;
        end;


    procedure tx86inlinenode.second_popcnt;
      var
        opsize: tcgsize;
      begin
        secondpass(left);

        opsize:=tcgsize2unsigned[left.location.size];

        { no 8 Bit popcont }
        if opsize=OS_8 then
          opsize:=OS_16;

        if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_REFERENCE,LOC_CREFERENCE]) or
           (left.location.size<>opsize) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(opsize),true);

        location_reset(location,LOC_REGISTER,opsize);
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
        if left.location.loc in [LOC_REGISTER,LOC_CREGISTER] then
          emit_reg_reg(A_POPCNT,TCGSize2OpSize[opsize],left.location.register,location.register)
        else
          emit_ref_reg(A_POPCNT,TCGSize2OpSize[opsize],left.location.reference,location.register);
      end;


    procedure tx86inlinenode.second_fma;
      const
        op : array[false..true,false..true,s32real..s64real,0..3] of TAsmOp =
          (
           { positive product }
           (
            { positive third operand }
            ((A_VFMADD231SS,A_VFMADD231SS,A_VFMADD231SS,A_VFMADD213SS),
             (A_VFMADD231SD,A_VFMADD231SD,A_VFMADD231SD,A_VFMADD213SD)
            ),
            { negative third operand }
            ((A_VFMSUB231SS,A_VFMSUB231SS,A_VFMSUB231SS,A_VFMSUB213SS),
             (A_VFMSUB231SD,A_VFMSUB231SD,A_VFMSUB231SD,A_VFMSUB213SD)
            )
           ),
           { negative product }
           (
            { positive third operand }
            ((A_VFNMADD231SS,A_VFNMADD231SS,A_VFNMADD231SS,A_VFNMADD213SS),
             (A_VFNMADD231SD,A_VFNMADD231SD,A_VFNMADD231SD,A_VFNMADD213SD)
            ),
            { negative third operand }
            ((A_VFNMSUB231SS,A_VFNMSUB231SS,A_VFNMSUB231SS,A_VFNMSUB213SS),
             (A_VFNMSUB231SD,A_VFNMSUB231SD,A_VFNMSUB231SD,A_VFNMSUB213SD)
            )
           )
          );

      var
        paraarray : array[1..3] of tnode;
        memop,
        i : integer;
        negop3,
        negproduct,
        gotmem : boolean;
      begin
{$ifndef i8086}
         if (cpu_capabilities[current_settings.cputype]*[CPUX86_HAS_FMA,CPUX86_HAS_FMA4])<>[] then
           begin
             negop3:=false;
             negproduct:=false;
             paraarray[1]:=tcallparanode(tcallparanode(tcallparanode(parameters).nextpara).nextpara).paravalue;
             paraarray[2]:=tcallparanode(tcallparanode(parameters).nextpara).paravalue;
             paraarray[3]:=tcallparanode(parameters).paravalue;

             { check if a neg. node can be removed
               this is possible because changing the sign of
               a floating point number does not affect its absolute
               value in any way
             }
             if paraarray[1].nodetype=unaryminusn then
               begin
                 paraarray[1]:=tunarynode(paraarray[1]).left;
                 { do not release the unused unary minus node, it is kept and release together with the other nodes,
                   only no code is generated for it }
                 negproduct:=not(negproduct);
               end;

             if paraarray[2].nodetype=unaryminusn then
               begin
                 paraarray[2]:=tunarynode(paraarray[2]).left;
                 { do not release the unused unary minus node, it is kept and release together with the other nodes,
                   only no code is generated for it }
                 negproduct:=not(negproduct);
               end;

             if paraarray[3].nodetype=unaryminusn then
               begin
                 paraarray[3]:=tunarynode(paraarray[3]).left;
                 { do not release the unused unary minus node, it is kept and release together with the other nodes,
                   only no code is generated for it }
                 negop3:=true;
               end;

              for i:=1 to 3 do
               secondpass(paraarray[i]);

             { only one memory operand is allowed }
             gotmem:=false;
             memop:=0;
             for i:=1 to 3 do
               begin
                 if not(paraarray[i].location.loc in [LOC_MMREGISTER,LOC_CMMREGISTER]) then
                   begin
                     if (paraarray[i].location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and not(gotmem) then
                       begin
                         memop:=i;
                         gotmem:=true;
                       end
                     else
                       hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,paraarray[i].location,paraarray[i].resultdef,true);
                   end;
               end;

             location_reset(location,LOC_MMREGISTER,paraarray[1].location.size);
             location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

             if gotmem then
               begin
                 case memop of
                   1:
                     begin
                       hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,resultdef,
                         paraarray[3].location.register,location.register,mms_movescalar);
                       emit_ref_reg_reg(op[negproduct,negop3,tfloatdef(resultdef).floattype,memop],S_NO,
                         paraarray[1].location.reference,paraarray[2].location.register,location.register);
                     end;
                   2:
                     begin
                       hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,resultdef,
                         paraarray[3].location.register,location.register,mms_movescalar);
                       emit_ref_reg_reg(op[negproduct,negop3,tfloatdef(resultdef).floattype,memop],S_NO,
                         paraarray[2].location.reference,paraarray[1].location.register,location.register);
                     end;
                   3:
                     begin
                       hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[1].resultdef,resultdef,
                         paraarray[1].location.register,location.register,mms_movescalar);
                       emit_ref_reg_reg(op[negproduct,negop3,tfloatdef(resultdef).floattype,memop],S_NO,
                         paraarray[3].location.reference,paraarray[2].location.register,location.register);
                     end
                   else
                     internalerror(2014041301);
                 end;
               end
             else
               begin
                 { try to use the location which is already in a temp. mm register as destination,
                   so the compiler might be able to re-use the register }
                 if paraarray[1].location.loc=LOC_MMREGISTER then
                   begin
                     hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[1].resultdef,resultdef,
                       paraarray[1].location.register,location.register,mms_movescalar);
                     emit_reg_reg_reg(op[negproduct,negop3,tfloatdef(resultdef).floattype,3],S_NO,
                       paraarray[3].location.register,paraarray[2].location.register,location.register);
                   end
                 else if paraarray[2].location.loc=LOC_MMREGISTER then
                   begin
                     hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[2].resultdef,resultdef,
                       paraarray[2].location.register,location.register,mms_movescalar);
                     emit_reg_reg_reg(op[negproduct,negop3,tfloatdef(resultdef).floattype,3],S_NO,
                       paraarray[3].location.register,paraarray[1].location.register,location.register);
                   end
                 else
                   begin
                     hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,resultdef,
                       paraarray[3].location.register,location.register,mms_movescalar);
                     emit_reg_reg_reg(op[negproduct,negop3,tfloatdef(resultdef).floattype,0],S_NO,
                       paraarray[1].location.register,paraarray[2].location.register,location.register);
                   end;
               end;
           end
         else
{$endif i8086}
           internalerror(2014032301);
      end;

end.
