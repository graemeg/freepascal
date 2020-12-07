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
          function pass_typecheck_cpu:tnode;override;

          { first pass override
            so that the code generator will actually generate
            these nodes.
          }
          function first_cpu: tnode;override;
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
          function first_frac_real : tnode; override;
          function first_int_real : tnode; override;
          function first_minmax: tnode; override;

          function simplify(forinline : boolean) : tnode; override;

          { second pass override to generate these nodes }
          procedure pass_generate_code_cpu;override;
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

          procedure second_abs_long;override;
          procedure second_popcnt;override;
          procedure second_fma;override;
          procedure second_frac_real;override;
          procedure second_int_real;override;
          procedure second_high;override;
          procedure second_minmax;override;
       private
          procedure load_fpu_location(lnode: tnode);
       end;

implementation

    uses
      systems,
      globtype,globals,
      verbose,compinnr,fmodule,
      defutil,
      aasmbase,aasmdata,aasmcpu,
      symconst,symtype,symdef,symcpu,
      ncnv,
      htypechk,
      cgbase,pass_1,pass_2,
      cpuinfo,cpubase,nutils,
      ncal,ncgutil,nld,ncon,
      tgobj,
      cga,cgutils,cgx86,cgobj,hlcgobj;


{*****************************************************************************
                              TX86INLINENODE
*****************************************************************************}

     procedure tx86inlinenode.maybe_remove_round_trunc_typeconv;
       begin
         { only makes a difference for x86_64 }
       end;


     function tx86inlinenode.pass_typecheck_cpu: tnode;
       begin
         Result:=nil;
         case inlinenumber of
           in_x86_inportb:
             begin
               CheckParameters(1);
               resultdef:=u8inttype;
             end;
           in_x86_inportw:
             begin
               CheckParameters(1);
               resultdef:=u16inttype;
             end;
           in_x86_inportl:
             begin
               CheckParameters(1);
               resultdef:=s32inttype;
             end;
           in_x86_outportb,
           in_x86_outportw,
           in_x86_outportl:
             begin
               CheckParameters(2);
               resultdef:=voidtype;
             end;
           in_x86_cli,
           in_x86_sti:
             resultdef:=voidtype;
           in_x86_get_cs,
           in_x86_get_ss,
           in_x86_get_ds,
           in_x86_get_es,
           in_x86_get_fs,
           in_x86_get_gs:
{$ifdef i8086}
             resultdef:=u16inttype;
{$else i8086}
             resultdef:=s32inttype;
{$endif i8086}
           { include automatically generated code }
           {$i x86mmtype.inc}
           else
             Result:=inherited pass_typecheck_cpu;
         end;
       end;


     function tx86inlinenode.first_cpu: tnode;
       begin
         Result:=nil;
         case inlinenumber of
           in_x86_inportb,
           in_x86_inportw,
           in_x86_inportl,
           in_x86_get_cs,
           in_x86_get_ss,
           in_x86_get_ds,
           in_x86_get_es,
           in_x86_get_fs,
           in_x86_get_gs:
             expectloc:=LOC_REGISTER;
           in_x86_outportb,
           in_x86_outportw,
           in_x86_outportl,
           in_x86_cli,
           in_x86_sti:
             expectloc:=LOC_VOID;
           { include automatically generated code }
           {$i x86mmfirst.inc}
           else
             Result:=inherited first_cpu;
         end;
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


     function tx86inlinenode.first_frac_real : tnode;
       begin
         if (current_settings.fputype>=fpu_sse41) and
           ((is_double(resultdef)) or (is_single(resultdef))) then
           begin
             maybe_remove_round_trunc_typeconv;
             expectloc:=LOC_MMREGISTER;
             Result:=nil;
           end
         else
           Result:=inherited first_frac_real;
       end;


     function tx86inlinenode.first_int_real : tnode;
       begin
         if (current_settings.fputype>=fpu_sse41) and
           ((is_double(resultdef)) or (is_single(resultdef))) then
           begin
             Result:=nil;
             expectloc:=LOC_MMREGISTER;
           end
         else
           Result:=inherited first_int_real;
       end;


     function tx86inlinenode.first_minmax: tnode;
       begin
{$ifndef i8086}
         if
{$ifdef i386}
           ((current_settings.fputype>=fpu_sse) and is_single(resultdef)) or
           ((current_settings.fputype>=fpu_sse2) and is_double(resultdef))
{$else i386}
           ((is_double(resultdef)) or (is_single(resultdef)))
{$endif i386}
           then
           begin
             expectloc:=LOC_MMREGISTER;
             Result:=nil;
           end
         else
{$endif i8086}
           Result:=inherited first_minmax;
       end;


     function tx86inlinenode.simplify(forinline : boolean) : tnode;
       var
         temp : tnode;
       begin
         if (current_settings.fputype>=fpu_sse41) and
           (inlinenumber=in_int_real) and (left.nodetype=typeconvn) and
           not(nf_explicit in left.flags) and
           (ttypeconvnode(left).left.resultdef.typ=floatdef) and
           ((is_double(ttypeconvnode(left).left.resultdef)) or (is_single(ttypeconvnode(left).left.resultdef))) then
           begin
             { get rid of the type conversion }
             temp:=ttypeconvnode(left).left;
             ttypeconvnode(left).left:=nil;
             left.free;
             left:=temp;
             result:=self.getcopy;
             tinlinenode(result).resultdef:=temp.resultdef;
             typecheckpass(result);
           end
         else
           Result:=inherited simplify(forinline);
       end;


     procedure tx86inlinenode.pass_generate_code_cpu;

       var
         paraarray : array[1..4] of tnode;
         i : integer;
         op: TAsmOp;

       procedure inport(dreg:TRegister;dsize:topsize;dtype:tdef);
         var
           portnumber: tnode;
         begin
           portnumber:=left;
           secondpass(portnumber);
           if (portnumber.location.loc=LOC_CONSTANT) and
              (portnumber.location.value>=0) and
              (portnumber.location.value<=255) then
             begin
               hlcg.getcpuregister(current_asmdata.CurrAsmList,dreg);
               current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_IN,dsize,portnumber.location.value,dreg));
               location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
               location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
               hlcg.ungetcpuregister(current_asmdata.CurrAsmList,dreg);
               hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,dtype,resultdef,dreg,location.register);
             end
           else
             begin
               hlcg.getcpuregister(current_asmdata.CurrAsmList,NR_DX);
               hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,portnumber.resultdef,u16inttype,portnumber.location,NR_DX);
               hlcg.getcpuregister(current_asmdata.CurrAsmList,dreg);
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_IN,dsize,NR_DX,dreg));
               hlcg.ungetcpuregister(current_asmdata.CurrAsmList,NR_DX);
               location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
               location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
               hlcg.ungetcpuregister(current_asmdata.CurrAsmList,dreg);
               hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,dtype,resultdef,dreg,location.register);
             end;
         end;


       procedure outport(dreg:TRegister;dsize:topsize;dtype:tdef);
         var
           portnumber, portdata: tnode;
         begin
           portnumber:=tcallparanode(tcallparanode(left).right).left;
           portdata:=tcallparanode(left).left;
           secondpass(portdata);
           secondpass(portnumber);
           hlcg.getcpuregister(current_asmdata.CurrAsmList,dreg);
           hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,portdata.resultdef,dtype,portdata.location,dreg);
           if (portnumber.location.loc=LOC_CONSTANT) and
              (portnumber.location.value>=0) and
              (portnumber.location.value<=255) then
             current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_OUT,dsize,dreg,portnumber.location.value))
           else
             begin
               hlcg.getcpuregister(current_asmdata.CurrAsmList,NR_DX);
               hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,portnumber.resultdef,u16inttype,portnumber.location,NR_DX);
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_OUT,dsize,dreg,NR_DX));
               hlcg.ungetcpuregister(current_asmdata.CurrAsmList,NR_DX);
             end;
           hlcg.ungetcpuregister(current_asmdata.CurrAsmList,dreg);
         end;


       procedure get_segreg(segreg:tregister);
         begin
           location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
           location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_MOV,TCGSize2OpSize[def_cgsize(resultdef)],segreg,location.register));
         end;


      function GetConstInt(n: tnode): longint;
        begin
          Result:=0;
          if is_constintnode(n) then
            result:=tordconstnode(n).value.svalue
          else
            Message(type_e_constant_expr_expected);
        end;


      procedure GetParameters(count: longint);
        var
          i: longint;
          p: tnode;
        begin
          if (count=1) and
             (not (left is tcallparanode)) then
            paraarray[1]:=left
          else
            begin
              p:=left;
              for i := count downto 1 do
                begin
                  paraarray[i]:=tcallparanode(p).paravalue;
                  p:=tcallparanode(p).nextpara;
                end;
            end;
        end;

      procedure location_force_mmxreg(list:TAsmList;var l: tlocation;maybeconst:boolean);
        var
          reg : tregister;
        begin
          if (l.loc<>LOC_MMXREGISTER)  and
             ((l.loc<>LOC_CMMXREGISTER) or (not maybeconst)) then
            begin
              reg:=tcgx86(cg).getmmxregister(list);
              cg.a_loadmm_loc_reg(list,OS_M64,l,reg,nil);
              location_freetemp(list,l);
              location_reset(l,LOC_MMXREGISTER,OS_M64);
              l.register:=reg;
            end;
        end;

      procedure location_make_ref(var loc: tlocation);
        var
          hloc: tlocation;
        begin
          case loc.loc of
            LOC_CREGISTER,
            LOC_REGISTER:
              begin
                location_reset_ref(hloc, LOC_REFERENCE, OS_32, 1, []);
                hloc.reference.base:=loc.register;

                loc:=hloc;
              end;
            LOC_CREFERENCE,
            LOC_REFERENCE:
              begin
              end;
          else
            begin
              hlcg.location_force_reg(current_asmdata.CurrAsmList,loc,u32inttype,u32inttype,false);

              location_reset_ref(hloc, LOC_REFERENCE, OS_32, 1, []);
              hloc.reference.base:=loc.register;

              loc:=hloc;
            end;
          end;
        end;

       begin
         FillChar(paraarray,sizeof(paraarray),0);
         case inlinenumber of
           in_x86_inportb:
             inport(NR_AL,S_B,u8inttype);
           in_x86_inportw:
             inport(NR_AX,S_W,u16inttype);
           in_x86_inportl:
             inport(NR_EAX,S_L,s32inttype);
           in_x86_outportb:
             outport(NR_AL,S_B,u8inttype);
           in_x86_outportw:
             outport(NR_AX,S_W,u16inttype);
           in_x86_outportl:
             outport(NR_EAX,S_L,s32inttype);
           in_x86_cli:
             current_asmdata.CurrAsmList.concat(taicpu.op_none(A_CLI));
           in_x86_sti:
             current_asmdata.CurrAsmList.concat(taicpu.op_none(A_STI));
           in_x86_get_cs:
             get_segreg(NR_CS);
           in_x86_get_ss:
             get_segreg(NR_SS);
           in_x86_get_ds:
             get_segreg(NR_DS);
           in_x86_get_es:
             get_segreg(NR_ES);
           in_x86_get_fs:
             get_segreg(NR_FS);
           in_x86_get_gs:
             get_segreg(NR_GS);
           {$i x86mmsecond.inc}
           else
             inherited pass_generate_code_cpu;
         end;
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

       function needs_indirect:boolean; inline;
         begin
           result:=(tf_supports_packages in target_info.flags) and
                     (target_info.system in systems_indirect_var_imports);
         end;

       var
         href : treference;
         sym : tasmsymbol;
       begin
         if use_vectorfpu(resultdef) then
           begin
             secondpass(left);
             if left.location.loc<>LOC_MMREGISTER then
               hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,UseAVX);
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
                   sym:=current_asmdata.RefAsmSymbol(target_info.cprefix+'FPC_ABSMASK_SINGLE',AT_DATA,needs_indirect);
                   reference_reset_symbol(href,sym,0,4,[]);
                   current_module.add_extern_asmsym(sym);
                   tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList, href);
                   if UseAVX then
                     current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg_reg(
                       A_VANDPS,S_XMM,href,left.location.register,location.register))
                   else
                     current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_ANDPS,S_XMM,href,location.register));
                 end;
               s64real:
                 begin
                   sym:=current_asmdata.RefAsmSymbol(target_info.cprefix+'FPC_ABSMASK_DOUBLE',AT_DATA,needs_indirect);
                   reference_reset_symbol(href,sym,0,4,[]);
                   current_module.add_extern_asmsym(sym);
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
             hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
             location_reset(location,LOC_REGISTER,OS_S64);
             location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_S64);
             if UseAVX then
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTSS2SI,S_NO,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTSD2SI,S_NO,left.location.register,location.register));
                 else
                   internalerror(2007031402);
               end
             else
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTSS2SI,S_NO,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTSD2SI,S_NO,left.location.register,location.register));
                 else
                   internalerror(2007031404);
               end;
           end
         else
{$endif x86_64}
          begin
            load_fpu_location(left);
            location_reset_ref(location,LOC_REFERENCE,OS_S64,0,[]);
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
             hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
             location_reset(location,LOC_REGISTER,OS_S64);
             location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_S64);
             if UseAVX then
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTTSS2SI,S_NO,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VCVTTSD2SI,S_NO,left.location.register,location.register));
                 else
                   internalerror(2007031401);
               end
             else
               case left.location.size of
                 OS_F32:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTTSS2SI,S_NO,left.location.register,location.register));
                 OS_F64:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVTTSD2SI,S_NO,left.location.register,location.register));
                 else
                   internalerror(2007031403);
               end;
           end
         else
{$endif x86_64}
          begin
            if (current_settings.fputype>=fpu_sse3) then
              begin
                load_fpu_location(left);
                location_reset_ref(location,LOC_REFERENCE,OS_S64,0,[]);
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
                location_reset_ref(location,LOC_REFERENCE,OS_S64,0,[]);
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
                   { we use S_NO instead of S_XMM here, regardless of the register size, as the size of the memory location is 32/64 bit }
                   { using left.location.register here as 2nd parameter is crucial to break dependency chains }
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_VSQRTSS,S_NO,left.location.register,left.location.register,location.register));
                 s64real:
                   { we use S_NO instead of S_XMM here, regardless of the register size, as the size of the memory location is 32/64 bit }
                   { using left.location.register here as 2nd parameter is crucial to break dependency chains }
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_VSQRTSD,S_NO,left.location.register,left.location.register,location.register));
                 else
                   internalerror(200510031);
               end
             else
               case tfloatdef(resultdef).floattype of
                 s32real:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SQRTSS,S_NO,left.location.register,location.register));
                 s64real:
                   current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SQRTSD,S_NO,left.location.register,location.register));
                 else
                   internalerror(2005100303);
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
         checkpointer_used : boolean;
       begin
{$if defined(i386) or defined(i8086)}
         if current_settings.cputype>=cpu_Pentium3 then
{$endif i386 or i8086}
           begin
             { do not call Checkpointer for left node }
             checkpointer_used:=(cs_checkpointer in current_settings.localswitches);
             if checkpointer_used then
               node_change_local_switch(left,cs_checkpointer,false);
             secondpass(left);
             if checkpointer_used then
               node_change_local_switch(left,cs_checkpointer,false);
             case left.location.loc of
               LOC_CREFERENCE,
               LOC_REFERENCE:
                 begin
                   r:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
                   cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.location.reference,r);
                   reference_reset_base(ref,r,0,left.location.reference.temppos,left.location.reference.alignment,left.location.reference.volatility);
                   current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_PREFETCHNTA,S_NO,ref));
                 end;
               else
                 { nothing to prefetch };
             end;
           end;
       end;


    procedure tx86inlinenode.second_abs_long;
      var
        hregister : tregister;
        opsize : tcgsize;
        hp : taicpu;
      begin
{$if defined(i8086) or defined(i386)}
        if not(CPUX86_HAS_CMOV in cpu_capabilities[current_settings.cputype]) then
          begin
            opsize:=def_cgsize(left.resultdef);
            secondpass(left);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
            location:=left.location;
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
            cg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,opsize,left.location.register,location.register);
            cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SAR,opsize,tcgsize2size[opsize]*8-1,left.location.register);
            cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_XOR,opsize,left.location.register,location.register);
            cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_SUB,opsize,left.location.register,location.register);
          end
        else
{$endif i8086 or i386}
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
         opdef : tdef;
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
                  opdef:=u32inttype;
                  opsize:=OS_32;
                end;
              { determine asm operator }
              if inlinenumber=in_include_x_y then
                 asmop:=A_BTS
              else
                 asmop:=A_BTR;

              hlcg.location_force_reg(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,tcallparanode(tcallparanode(left).right).left.resultdef,opdef,true);
              register_maybe_adjust_setbase(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.resultdef,tcallparanode(tcallparanode(left).right).left.location,setbase);
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

        if resultdef.size=1 then
          begin
            location.size:=OS_8;
            location.register:=cg.makeregsize(current_asmdata.CurrAsmList,location.register,location.size);
          end;
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


    procedure tx86inlinenode.second_frac_real;
      var
        extrareg : TRegister;
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
                  begin
                    { using left.location.register here as 3rd parameter is crucial to break dependency chains }
                    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg_reg(A_VROUNDSS,S_NO,3,left.location.register,left.location.register,location.register));
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_VSUBSS,S_NO,location.register,left.location.register,location.register));
                  end;
                s64real:
                  begin
                    { using left.location.register here as 3rd parameter is crucial to break dependency chains }
                    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg_reg(A_VROUNDSD,S_NO,3,left.location.register,left.location.register,location.register));
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_VSUBSD,S_NO,location.register,left.location.register,location.register));
                  end;
                else
                  internalerror(2017052102);
              end
            else
              begin
                extrareg:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
                cg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,location.size,left.location,location.register,mms_movescalar);
                case tfloatdef(resultdef).floattype of
                  s32real:
                    begin
                      current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_ROUNDSS,S_NO,3,left.location.register,extrareg));
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SUBSS,S_NO,extrareg,location.register));
                    end;
                  s64real:
                    begin
                      current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_ROUNDSD,S_NO,3,left.location.register,extrareg));
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SUBSD,S_NO,extrareg,location.register));
                    end;
                  else
                    internalerror(2017052103);
                end;
              end;
          end
        else
          internalerror(2017052101);
      end;


    procedure tx86inlinenode.second_int_real;
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
                  { using left.location.register here as 3rd parameter is crucial to break dependency chains }
                  current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg_reg(A_VROUNDSS,S_NO,3,left.location.register,left.location.register,location.register));
                s64real:
                  { using left.location.register here as 3rd parameter is crucial to break dependency chains }
                  current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg_reg(A_VROUNDSD,S_NO,3,left.location.register,left.location.register,location.register));
                else
                  internalerror(2017052105);
              end
            else
              begin
                case tfloatdef(resultdef).floattype of
                  s32real:
                    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_ROUNDSS,S_NO,3,left.location.register,location.register));
                  s64real:
                    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_ROUNDSD,S_NO,3,left.location.register,location.register));
                  else
                    internalerror(2017052106);
                end;
              end;
          end
        else
          internalerror(2017052107);
      end;


    procedure tx86inlinenode.second_high;
      var
        donelab: tasmlabel;
        hregister : tregister;
        href : treference;
      begin
        secondpass(left);
        if not(is_dynamic_array(left.resultdef)) then
          Internalerror(2019122809);
        { length in dynamic arrays is at offset -sizeof(pint) }
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
        current_asmdata.getjumplabel(donelab);
        { by subtracting 1 here, we get the -1 into the register we need if the dyn. array is nil and the carry
          flag is set in this case, so we can jump depending on it

          when loading the actual high value, we have to take care later of the decreased value

          do not use the cgs, as they might emit dec instead of a sub instruction, however with dec the trick
          we are using is not working as dec does not touch the carry flag }
        current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_SUB,TCGSize2OpSize[def_cgsize(left.resultdef)],1,left.location.register));
        { volatility of the dyn. array refers to the volatility of the
          string pointer, not of the string data }
        cg.a_jmp_flags(current_asmdata.CurrAsmList,F_C,donelab);
        hlcg.reference_reset_base(href,left.resultdef,left.location.register,-ossinttype.size+1,ctempposinvalid,ossinttype.alignment,[]);
        { if the string pointer is nil, the length is 0 -> reuse the register
          that originally held the string pointer for the length, so that we
          can keep the original nil/0 as length in that case }
        hregister:=cg.makeregsize(current_asmdata.CurrAsmList,left.location.register,def_cgsize(resultdef));
        hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,ossinttype,resultdef,href,hregister);

        cg.a_label(current_asmdata.CurrAsmList,donelab);
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hregister;
      end;


    procedure tx86inlinenode.second_minmax;
      const
        oparray : array[false..true,false..true,s32real..s64real] of TAsmOp =
          (
           (
            (A_MINSS,A_MINSD),
            (A_VMINSS,A_VMINSD)
           ),
           (
            (A_MAXSS,A_MAXSD),
            (A_VMAXSS,A_VMAXSD)
           )
          );

      var
        paraarray : array[1..2] of tnode;
        memop,
        i : integer;
        gotmem : boolean;
        op: TAsmOp;
      begin
{$ifndef i8086}
         if
{$ifdef i386}
           ((current_settings.fputype>=fpu_sse) and is_single(resultdef)) or
           ((current_settings.fputype>=fpu_sse2) and is_double(resultdef))
{$else i386}
           is_single(resultdef) or is_double(resultdef)
{$endif i386}
           then
           begin
             paraarray[1]:=tcallparanode(tcallparanode(parameters).nextpara).paravalue;
             paraarray[2]:=tcallparanode(parameters).paravalue;

             for i:=low(paraarray) to high(paraarray) do
               secondpass(paraarray[i]);

             { only one memory operand is allowed }
             gotmem:=false;
             memop:=0;
             for i:=low(paraarray) to high(paraarray) do
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

             { due to min/max behaviour that it loads always the second operand (must be the else assignment) into destination if
               one of the operands is a NaN, we cannot swap operands to omit a mova operation in case fastmath is off }
             if not(cs_opt_fastmath in current_settings.optimizerswitches) and gotmem and (memop=1) then
               begin
                 hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,paraarray[1].location,paraarray[1].resultdef,true);
                 gotmem:=false;
               end;

             op:=oparray[inlinenumber in [in_max_single,in_max_double],UseAVX,tfloatdef(resultdef).floattype];

             location_reset(location,LOC_MMREGISTER,paraarray[1].location.size);
             location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

             if gotmem then
               begin
                 if UseAVX then
                   case memop of
                     1:
                       emit_ref_reg_reg(op,S_NO,
                         paraarray[1].location.reference,paraarray[2].location.register,location.register);
                     2:
                       emit_ref_reg_reg(op,S_NO,
                         paraarray[2].location.reference,paraarray[1].location.register,location.register);
                     else
                       internalerror(2020120504);
                   end
                 else
                   case memop of
                     1:
                       begin
                         hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[2].resultdef,resultdef,
                           paraarray[2].location.register,location.register,mms_movescalar);
                         emit_ref_reg(op,S_NO,
                           paraarray[1].location.reference,location.register);
                       end;
                     2:
                       begin
                         hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[1].resultdef,resultdef,
                           paraarray[1].location.register,location.register,mms_movescalar);
                         emit_ref_reg(op,S_NO,
                           paraarray[2].location.reference,location.register);
                       end;
                     else
                       internalerror(2020120601);
                   end;
               end
             else
               begin
                 if UseAVX then
                   emit_reg_reg_reg(op,S_NO,
                     paraarray[2].location.register,paraarray[1].location.register,location.register)
                 else
                   begin
                     hlcg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,paraarray[1].resultdef,resultdef,
                       paraarray[1].location.register,location.register,mms_movescalar);
                     emit_reg_reg(op,S_NO,
                       paraarray[2].location.register,location.register)
                   end;
               end;
           end
         else
{$endif i8086}
           internalerror(2020120503);
      end;


end.
