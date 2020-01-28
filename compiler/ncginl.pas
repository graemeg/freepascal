{
    Copyright (c) 1998-2002 by Florian Klaempfl and Carl Eric Codere

    Generate generic inline nodes

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
unit ncginl;

{$i fpcdefs.inc}

interface

    uses
       node,ninl,symtype;

    type
       tcginlinenode = class(tinlinenode)
          procedure pass_generate_code;override;
          procedure pass_generate_code_cpu;virtual;
          procedure second_sizeoftypeof;virtual;
          procedure second_length;virtual;
          procedure second_predsucc;virtual;
          procedure second_incdec;virtual;
          procedure second_AndOrXorShiftRot_assign;virtual;
          procedure second_NegNot_assign;virtual;
          procedure second_typeinfo;virtual;
          procedure second_includeexclude;virtual;
          procedure second_pi; virtual;
          procedure second_arctan_real; virtual;
          procedure second_abs_real; virtual;
          procedure second_sqr_real; virtual;
          procedure second_sqrt_real; virtual;
          procedure second_ln_real; virtual;
          procedure second_cos_real; virtual;
          procedure second_sin_real; virtual;
          procedure second_assigned; virtual;
          procedure second_get_frame;virtual;
          procedure second_get_caller_frame;virtual;
          procedure second_get_caller_addr;virtual;
          procedure second_prefetch; virtual;
          procedure second_round_real; virtual;
          procedure second_trunc_real; virtual;
          procedure second_int_real; virtual;
          procedure second_abs_long; virtual;
          procedure second_rox_sar; virtual;
          procedure second_bsfbsr; virtual;
          procedure second_new; virtual;
          procedure second_setlength; virtual; abstract;
          procedure second_box; virtual; abstract;
          procedure second_popcnt; virtual;
          procedure second_seg; virtual; abstract;
          procedure second_fma; virtual;
          procedure second_frac_real; virtual;
          procedure second_high; virtual;
       protected
          function  second_incdec_tempregdef: tdef;virtual;
       end;

implementation

    uses
      globtype,constexp,
      verbose,globals,compinnr,
      symconst,symdef,defutil,
      aasmbase,aasmdata,
      cgbase,pass_2,
      cpubase,procinfo,
      ncon,ncal,
      tgobj,ncgutil,
      cgutils,cgobj,hlcgobj
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
      ,cg64f32
{$endif not cpu64bitalu and not cpuhighleveltarget}
      ;


{*****************************************************************************
                              TCGINLINENODE
*****************************************************************************}


    procedure tcginlinenode.pass_generate_code;
      begin
         location_reset(location,LOC_VOID,OS_NO);

         case inlinenumber of
            in_sizeof_x,
            in_typeof_x :
              second_SizeofTypeOf;
            in_length_x :
              second_Length;
            in_pred_x,
            in_succ_x:
               second_PredSucc;
            in_dec_x,
            in_inc_x :
              second_IncDec;
            in_typeinfo_x:
              second_TypeInfo;
            in_include_x_y,
            in_exclude_x_y:
              second_IncludeExclude;
            in_pi_real:
              second_pi;
            in_sin_real:
              second_sin_real;
            in_arctan_real:
              second_arctan_real;
            in_abs_real:
              second_abs_real;
            in_abs_long:
              second_abs_long;
            in_round_real:
              second_round_real;
            in_trunc_real:
              second_trunc_real;
            in_int_real:
              second_int_real;
            in_sqr_real:
              second_sqr_real;
            in_sqrt_real:
              second_sqrt_real;
            in_ln_real:
              second_ln_real;
            in_cos_real:
               second_cos_real;
            in_frac_real:
              second_frac_real;
            in_prefetch_var:
              second_prefetch;
            in_assigned_x:
              second_assigned;
            in_get_frame:
              second_get_frame;
            in_get_caller_frame:
              second_get_caller_frame;
            in_get_caller_addr:
              second_get_caller_addr;
            in_unaligned_x:
              begin
                secondpass(tcallparanode(left).left);
                location:=tcallparanode(left).left.location;
                if location.loc in [LOC_CREFERENCE,LOC_REFERENCE] then
                  location.reference.alignment:=1;
              end;
            in_aligned_x:
              begin
                secondpass(tcallparanode(left).left);
                location:=tcallparanode(left).left.location;
                if location.loc in [LOC_CREFERENCE,LOC_REFERENCE] then
                  location.reference.alignment:=resultdef.alignment;
              end;
            in_volatile_x:
              begin
                secondpass(tcallparanode(left).left);
                location:=tcallparanode(left).left.location;
                if location.loc in [LOC_CREFERENCE,LOC_REFERENCE,LOC_SUBSETREF,LOC_CSUBSETREF] then
                  location.reference.volatility:=[vol_read,vol_write];
              end;
{$ifdef SUPPORT_MMX}
            in_mmx_pcmpeqb..in_mmx_pcmpgtw:
              begin
                 location_reset(location,LOC_MMXREGISTER,OS_NO);
                 if left.location.loc=LOC_REGISTER then
                   begin
                      {!!!!!!!}
                   end
                 else if tcallparanode(left).left.location.loc=LOC_REGISTER then
                   begin
                      {!!!!!!!}
                   end
                 else
                   begin
                      {!!!!!!!}
                   end;
              end;
{$endif SUPPORT_MMX}
            in_rol_x,
            in_rol_x_y,
            in_ror_x,
            in_ror_x_y,
            in_sar_x,
            in_sar_x_y:
              second_rox_sar;
            in_bsf_x,
            in_bsr_x:
               second_BsfBsr;
            in_new_x:
               second_new;
            in_setlength_x:
               second_setlength;
            in_box_x:
               second_box;
            in_popcnt_x:
               second_popcnt;
            in_seg_x:
               second_seg;
            in_fma_single,
            in_fma_double,
            in_fma_extended,
            in_fma_float128:
               second_fma;
            in_and_assign_x_y,
            in_or_assign_x_y,
            in_xor_assign_x_y,
            in_sar_assign_x_y,
            in_shl_assign_x_y,
            in_shr_assign_x_y,
            in_rol_assign_x_y,
            in_ror_assign_x_y:
               second_AndOrXorShiftRot_assign;
            in_neg_assign_x,
            in_not_assign_x:
               second_NegNot_assign;
            in_high_x:
              second_high;
            else
               pass_generate_code_cpu;
         end;
      end;


    procedure tcginlinenode.pass_generate_code_cpu;
      begin
        Internalerror(2017110103);
      end;

{*****************************************************************************
                          SIZEOF / TYPEOF GENERIC HANDLING
*****************************************************************************}

    { second_handle_ the sizeof and typeof routines }
    procedure tcginlinenode.second_SizeOfTypeOf;
      begin
        { handled in pass 1 }
        internalerror(2015122701);
     end;


{*****************************************************************************
                          LENGTH GENERIC HANDLING
*****************************************************************************}

    procedure tcginlinenode.second_Length;
      var
        lengthlab : tasmlabel;
        hregister : tregister;
        lendef : tdef;
        href : treference;
      begin
        secondpass(left);
        if is_shortstring(left.resultdef) then
         begin
           location_copy(location,left.location);
           location.size:=OS_8;
         end
        else
         begin
           { length in ansi/wide strings and high in dynamic arrays is at offset -sizeof(pint) }
           hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
           current_asmdata.getjumplabel(lengthlab);
           hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,left.resultdef,OC_EQ,0,left.location.register,lengthlab);
           { the length of a widestring is a 32 bit unsigned int. Since every
             character occupies 2 bytes, on a 32 bit platform you can express
             the maximum length using 31 bits. On a 64 bit platform, it may be
             32 bits. This means that regardless of the platform, a location
             with size OS_SINT/ossinttype can hold the length without
             overflowing (this code returns an ossinttype value) }
           if is_widestring(left.resultdef) then
             lendef:=u32inttype
           else
             lendef:=ossinttype;
           { volatility of the ansistring/widestring refers to the volatility of the
             string pointer, not of the string data }
           hlcg.reference_reset_base(href,left.resultdef,left.location.register,-lendef.size,ctempposinvalid,lendef.alignment,[]);
           { if the string pointer is nil, the length is 0 -> reuse the register
             that originally held the string pointer for the length, so that we
             can keep the original nil/0 as length in that case }
           hregister:=cg.makeregsize(current_asmdata.CurrAsmList,left.location.register,def_cgsize(resultdef));
           hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,lendef,resultdef,href,hregister);
           if is_widestring(left.resultdef) then
             hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,resultdef,1,hregister);

           { Dynamic arrays do not have their length attached but their maximum index }
           if is_dynamic_array(left.resultdef) then
             hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_ADD,resultdef,1,hregister);

           cg.a_label(current_asmdata.CurrAsmList,lengthlab);
           location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
           location.register:=hregister;
         end;
      end;

{*****************************************************************************
                          HIGH(<dyn. array>) GENERIC HANDLING
*****************************************************************************}

    procedure tcginlinenode.second_high;
      var
        loadlab, nillab, donelab: tasmlabel;
        hregister : tregister;
        href : treference;
      begin
        secondpass(left);
        if not(is_dynamic_array(left.resultdef)) then
          Internalerror(2019122801);
        { length in dynamic arrays is at offset -sizeof(pint) }
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
        current_asmdata.getjumplabel(loadlab);
        current_asmdata.getjumplabel(nillab);
        current_asmdata.getjumplabel(donelab);
        hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,left.resultdef,OC_EQ,0,left.location.register,nillab);
        { volatility of the dyn. array refers to the volatility of the
          string pointer, not of the string data }
        hlcg.reference_reset_base(href,left.resultdef,left.location.register,-ossinttype.size,ctempposinvalid,ossinttype.alignment,[]);
        { if the string pointer is nil, the length is 0 -> reuse the register
          that originally held the string pointer for the length, so that we
          can keep the original nil/0 as length in that case }
        hregister:=cg.makeregsize(current_asmdata.CurrAsmList,left.location.register,def_cgsize(resultdef));
        hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,ossinttype,resultdef,href,hregister);
        hlcg.a_jmp_always(current_asmdata.CurrAsmList,donelab);

        cg.a_label(current_asmdata.CurrAsmList,nillab);
        hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SUB,resultdef,1,hregister);
        cg.a_label(current_asmdata.CurrAsmList,donelab);
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hregister;
      end;


{*****************************************************************************
                         PRED/SUCC GENERIC HANDLING
*****************************************************************************}

    procedure tcginlinenode.second_PredSucc;
      var
         cgop : topcg;
      begin
        secondpass(left);
        if inlinenumber=in_pred_x then
           cgop:=OP_SUB
        else
           cgop:=OP_ADD;

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
        if def_cgsize(resultdef) in [OS_64,OS_S64] then
          begin
            location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            cg64.a_op64_const_reg_reg(current_asmdata.CurrAsmList,cgop,def_cgsize(resultdef),1,left.location.register64,location.register64);
          end
        else
{$endif not cpu64bitalu and not cpuhighleveltarget}
          begin
            location.register:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,resultdef);
            hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,cgop,resultdef,1,left.location.register,location.register);
          end;
      end;


{*****************************************************************************
                         INC/DEC GENERIC HANDLING
*****************************************************************************}
      function tcginlinenode.second_incdec_tempregdef: tdef;
        begin
          second_incdec_tempregdef:=left.resultdef;
        end;

      procedure tcginlinenode.second_IncDec;
       const
         addsubop:array[in_inc_x..in_dec_x] of TOpCG=(OP_ADD,OP_SUB);
        var
         addvalue : TConstExprInt;
         addconstant : boolean;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
         hregisterhi,
{$endif not cpu64bitalu and not cpuhighleveltarget}
         hregister : tregister;
        begin
          { set defaults }
          addconstant:=true;
          hregister:=NR_NO;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
          hregisterhi:=NR_NO;
{$endif not cpu64bitalu and not cpuhighleveltarget}

          { first secondpass second argument, because if the first arg }
          { is used in that expression then SSL may move it to another }
          { register                                                   }
          if assigned(tcallparanode(left).right) then
            secondpass(tcallparanode(tcallparanode(left).right).left);
          { load first parameter, must be a reference }
          secondpass(tcallparanode(left).left);
          { get addvalue }
          case tcallparanode(left).left.resultdef.typ of
            orddef,
            enumdef :
                addvalue:=1;
            pointerdef :
               begin
                 if is_void(tpointerdef(tcallparanode(left).left.resultdef).pointeddef) then
                   addvalue:=1
                 else
                   addvalue:=tpointerdef(tcallparanode(left).left.resultdef).pointeddef.size;
               end;
           else
               internalerror(10081);
          end;
          { second_ argument specified?, must be a s32bit in register }
          if assigned(tcallparanode(left).right) then
            begin
              { when constant, just multiply the addvalue }
              if is_constintnode(tcallparanode(tcallparanode(left).right).left) then
                 addvalue:=addvalue*get_ordinal_value(tcallparanode(tcallparanode(left).right).left)
              else if is_constpointernode(tcallparanode(tcallparanode(left).right).left) then
                 addvalue:=addvalue*tpointerconstnode(tcallparanode(tcallparanode(left).right).left).value
              else
                begin
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,tcallparanode(tcallparanode(left).right).left.location,tcallparanode(tcallparanode(left).right).left.resultdef,second_incdec_tempregdef,addvalue<=1);
                  hregister:=tcallparanode(tcallparanode(left).right).left.location.register;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
                  hregisterhi:=tcallparanode(tcallparanode(left).right).left.location.register64.reghi;
{$endif not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
                  { insert multiply with addvalue if its >1 }
                  if addvalue>1 then
                    hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_IMUL,left.resultdef,addvalue.svalue,hregister);
                  addconstant:=false;
                end;
            end;
          { write the add instruction }
          if addconstant then
            begin
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
              if def_cgsize(left.resultdef) in [OS_64,OS_S64] then
                { use addvalue.svalue here to avoid an internal error if addvalue is unsigned and overflows int64, see #35298,
                  we are only interested in the bit pattern here }
                cg64.a_op64_const_loc(current_asmdata.CurrAsmList,addsubop[inlinenumber],def_cgsize(left.resultdef),addvalue.svalue,tcallparanode(left).left.location)
              else
{$endif not cpu64bitalu and not cpuhighleveltarget}
                hlcg.a_op_const_loc(current_asmdata.CurrAsmList,addsubop[inlinenumber],left.resultdef,
{$ifdef cpu64bitalu}
                  aint(addvalue.svalue),
{$else cpu64bitalu}
                  longint(addvalue.svalue),  // can't use aint, because it breaks 16-bit and 8-bit CPUs
{$endif cpu64bitalu}
                  tcallparanode(left).left.location);
            end
           else
             begin
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
               if def_cgsize(left.resultdef) in [OS_64,OS_S64] then
                 cg64.a_op64_reg_loc(current_asmdata.CurrAsmList,addsubop[inlinenumber],def_cgsize(left.resultdef),
                   joinreg64(hregister,hregisterhi),tcallparanode(left).left.location)
               else
{$endif not cpu64bitalu and not cpuhighleveltarget}
                 hlcg.a_op_reg_loc(current_asmdata.CurrAsmList,addsubop[inlinenumber],left.resultdef,
                   hregister,tcallparanode(left).left.location);
             end;
          { no overflow checking for pointers (see ninl), and range checking }
          { is not applicable for them                                       }
          if (tcallparanode(left).left.resultdef.typ <> pointerdef) then
            begin
              { things which can overflow must NOT pass via here, but have to be  }
              { handled via a regular add node (conversion in tinlinenode.pass_1) }
              { Or someone has to rewrite the above to use a_op_const_reg_reg_ov  }
              { and friends in case of overflow checking, and ask everyone to     }
              { implement these methods since they don't exist for all cpus (JM)  }
              { Similarly, range checking also has to be handled separately, }
              { see mantis #14841 (JM)                                       }
              if ([cs_check_overflow,cs_check_range] * current_settings.localswitches <> []) then
                internalerror(2006111010);
//              cg.g_overflowcheck(current_asmdata.CurrAsmList,tcallparanode(left).left.location,tcallparanode(left).resultdef);
//              cg.g_rangecheck(current_asmdata.CurrAsmList,tcallparanode(left).left.location,tcallparanode(left).left.resultdef,
//                 tcallparanode(left).left.resultdef);
            end;
        end;


{*****************************************************************************
              AND/OR/XOR/SHIFT/ROTATE ASSIGN GENERIC HANDLING
*****************************************************************************}
      procedure tcginlinenode.second_AndOrXorShiftRot_assign;
        const
          andorxorop:array[in_and_assign_x_y..in_ror_assign_x_y] of TOpCG=
            (OP_AND,OP_OR,OP_XOR,OP_SAR,OP_SHL,OP_SHR,OP_ROL,OP_ROR);
        var
          maskvalue : TConstExprInt;
          maskconstant : boolean;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
          hregisterhi,
{$endif not cpu64bitalu and not cpuhighleveltarget}
          hregister : tregister;
        begin
          { set defaults }
          maskconstant:=true;
          hregister:=NR_NO;
          maskvalue:=0;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
          hregisterhi:=NR_NO;
{$endif not cpu64bitalu and not cpuhighleveltarget}

          { first secondpass first argument, because if the second arg }
          { is used in that expression then SSL may move it to another }
          { register                                                   }
          secondpass(tcallparanode(left).left);
          { load second parameter, must be a reference }
          secondpass(tcallparanode(tcallparanode(left).right).left);

          { when constant, just get the maskvalue }
          if is_constintnode(tcallparanode(left).left) then
             maskvalue:=get_ordinal_value(tcallparanode(left).left)
          else
            begin
              { for shift/rotate the shift count can be of different size than the shifted variable }
              if inlinenumber in [in_sar_assign_x_y,in_shl_assign_x_y,in_shr_assign_x_y,in_rol_assign_x_y,in_ror_assign_x_y] then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,tcallparanode(left).left.location,tcallparanode(left).left.resultdef,tcallparanode(left).left.resultdef,true)
              else
                hlcg.location_force_reg(current_asmdata.CurrAsmList,tcallparanode(left).left.location,tcallparanode(left).left.resultdef,tcallparanode(left).right.resultdef,true);
              hregister:=tcallparanode(left).left.location.register;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
              hregisterhi:=tcallparanode(left).left.location.register64.reghi;
{$endif not cpu64bitalu and not cpuhighleveltarget}
              maskconstant:=false;
            end;
          { write the and/or/xor/sar/shl/shr/rol/ror instruction }
          if maskconstant then
            begin
              if inlinenumber in [in_sar_assign_x_y,in_shl_assign_x_y,in_shr_assign_x_y,in_rol_assign_x_y,in_ror_assign_x_y] then
                if def_cgsize(tcallparanode(left).right.resultdef) in [OS_64,OS_S64] then
                  maskvalue:=maskvalue and 63
                else
                  maskvalue:=maskvalue and 31;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
              if def_cgsize(tcallparanode(left).right.resultdef) in [OS_64,OS_S64] then
                cg64.a_op64_const_loc(current_asmdata.CurrAsmList,andorxorop[inlinenumber],def_cgsize(tcallparanode(left).right.resultdef),maskvalue.svalue,tcallparanode(tcallparanode(left).right).left.location)
              else
{$endif not cpu64bitalu and not cpuhighleveltarget}
                hlcg.a_op_const_loc(current_asmdata.CurrAsmList,andorxorop[inlinenumber],tcallparanode(left).right.resultdef,
{$ifdef cpu64bitalu}
                  aint(maskvalue.svalue),
{$else cpu64bitalu}
                  longint(maskvalue.svalue),  // can't use aint, because it breaks 16-bit and 8-bit CPUs
{$endif cpu64bitalu}
                  tcallparanode(tcallparanode(left).right).left.location);
            end
           else
             begin
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
               if def_cgsize(tcallparanode(left).right.resultdef) in [OS_64,OS_S64] then
                 cg64.a_op64_reg_loc(current_asmdata.CurrAsmList,andorxorop[inlinenumber],def_cgsize(tcallparanode(left).right.resultdef),
                   joinreg64(hregister,hregisterhi),tcallparanode(tcallparanode(left).right).left.location)
               else
{$endif not cpu64bitalu and not cpuhighleveltarget}
                 hlcg.a_op_reg_loc(current_asmdata.CurrAsmList,andorxorop[inlinenumber],tcallparanode(left).right.resultdef,
                   hregister,tcallparanode(tcallparanode(left).right).left.location);
             end;
        end;


{*****************************************************************************
                       NEG/NOT ASSIGN GENERIC HANDLING
*****************************************************************************}
      procedure tcginlinenode.second_NegNot_assign;
        const
          negnotop:array[in_neg_assign_x..in_not_assign_x] of TOpCG=(OP_NEG,OP_NOT);
{$ifndef cpu64bitalu}
        var
          NR_NO64: tregister64=(reglo:NR_NO;reghi:NR_NO);
{$endif not cpu64bitalu}
        begin
          { load parameter, must be a reference }
          secondpass(left);

          location_reset(location,LOC_VOID,OS_NO);

          if left.location.loc in [LOC_REGISTER,LOC_CREGISTER] then
            begin
{$ifndef cpu64bitalu}
              if def_cgsize(left.resultdef) in [OS_64,OS_S64] then
                cg64.a_op64_reg_loc(current_asmdata.CurrAsmList,negnotop[inlinenumber],def_cgsize(left.resultdef),left.location.register64,left.location)
              else
{$endif not cpu64bitalu}
                hlcg.a_op_reg_loc(current_asmdata.CurrAsmList,negnotop[inlinenumber],left.resultdef,left.location.register,left.location);
            end
          else if left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
            begin
{$ifndef cpu64bitalu}
              if def_cgsize(left.resultdef) in [OS_64,OS_S64] then
                cg64.a_op64_reg_loc(current_asmdata.CurrAsmList,negnotop[inlinenumber],def_cgsize(left.resultdef),NR_NO64,left.location)
              else
{$endif not cpu64bitalu}
                hlcg.a_op_reg_loc(current_asmdata.CurrAsmList,negnotop[inlinenumber],left.resultdef,NR_NO,left.location);
            end
          else
            internalerror(2017040701);
        end;


{*****************************************************************************
                         TYPEINFO GENERIC HANDLING
*****************************************************************************}
      procedure tcginlinenode.second_typeinfo;
        begin
          internalerror(2013060301);
        end;


{*****************************************************************************
                     INCLUDE/EXCLUDE GENERIC HANDLING
*****************************************************************************}

      procedure tcginlinenode.second_IncludeExclude;
        var
          setpara, elepara: tnode;
        begin
          { the set }
          secondpass(tcallparanode(left).left);
          { the element to set }
          secondpass(tcallparanode(tcallparanode(left).right).left);

          setpara:=tcallparanode(left).left;
          elepara:=tcallparanode(tcallparanode(left).right).left;

          if elepara.location.loc=LOC_CONSTANT then
            begin
              hlcg.a_bit_set_const_loc(current_asmdata.CurrAsmList,(inlinenumber=in_include_x_y),
                setpara.resultdef,elepara.location.value-tsetdef(setpara.resultdef).setbase,setpara.location);
            end
          else
            begin
              hlcg.location_force_reg(current_asmdata.CurrAsmList,elepara.location,elepara.resultdef,u32inttype,true);
              register_maybe_adjust_setbase(current_asmdata.CurrAsmList,u32inttype,elepara.location,tsetdef(setpara.resultdef).setbase);
              hlcg.a_bit_set_reg_loc(current_asmdata.CurrAsmList,(inlinenumber=in_include_x_y),
                u32inttype,setpara.resultdef,elepara.location.register,setpara.location);
            end;
        end;


{*****************************************************************************
                            FLOAT GENERIC HANDLING
*****************************************************************************}

{
  These routines all call internal RTL routines, so if they are
  called here, they give an internal error
}
    procedure tcginlinenode.second_pi;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_arctan_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_abs_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_round_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_trunc_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_int_real;
      begin
        internalerror(2016112702);
      end;

    procedure tcginlinenode.second_sqr_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_sqrt_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_ln_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_cos_real;
      begin
        internalerror(20020718);
      end;

    procedure tcginlinenode.second_sin_real;
      begin
        internalerror(20020718);
      end;


    procedure tcginlinenode.second_prefetch;
      begin
      end;

    procedure tcginlinenode.second_frac_real;
      begin
        internalerror(2017052104);
      end;

    procedure tcginlinenode.second_abs_long;
      var
        tempreg1, tempreg2: tregister;
      begin
        secondpass(left);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
        location:=left.location;
        location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,left.resultdef);

        tempreg1:=hlcg.getintregister(current_asmdata.CurrAsmList,left.resultdef);
        tempreg2:=hlcg.getintregister(current_asmdata.CurrAsmList,left.resultdef);

        hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,left.resultdef,left.resultdef.size*8-1,left.location.register,tempreg1);
        hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_XOR,left.resultdef,left.location.register,tempreg1,tempreg2);
        hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmlist,OP_SUB,left.resultdef,tempreg1,tempreg2,location.register);
      end;


{*****************************************************************************
                         ASSIGNED GENERIC HANDLING
*****************************************************************************}

    procedure tcginlinenode.second_assigned;
      begin
        internalerror(2013091602);
      end;

    procedure Tcginlinenode.second_get_frame;

    begin
{$if defined(x86) or defined(arm)}
      if current_procinfo.framepointer=NR_STACK_POINTER_REG then
        begin
          location_reset(location,LOC_CONSTANT,OS_ADDR);
          location.value:=0;
        end
      else
{$endif defined(x86) or defined(arm)}
        begin
          location_reset(location,LOC_CREGISTER,OS_ADDR);
          location.register:=current_procinfo.framepointer;
        end;
    end;

    procedure Tcginlinenode.second_get_caller_frame;

    var
      frame_reg:Tregister;
      use_frame_pointer:boolean;

    begin
      frame_reg:=NR_NO;

      if left<>nil then
        begin
          secondpass(left);
          if left.location.loc=LOC_CONSTANT then
            use_frame_pointer:=true
          else
            begin
              hlcg.location_force_reg(current_asmdata.currasmlist,left.location,left.resultdef,voidpointertype,false);
              frame_reg:=left.location.register;
              use_frame_pointer:=false;
            end
        end
      else
        begin
          use_frame_pointer:=current_procinfo.framepointer=NR_STACK_POINTER_REG;
          frame_reg:=current_procinfo.framepointer;
        end;

      if use_frame_pointer then
        begin
          location_reset(location,LOC_CREGISTER,OS_ADDR);
          location.register:=NR_FRAME_POINTER_REG;
        end
      else
        begin
          location_reset_ref(location,LOC_REFERENCE,OS_ADDR,sizeof(pint),[]);
          location.reference.base:=frame_reg;
        end;
    end;

    procedure Tcginlinenode.second_get_caller_addr;
      var
        frame_ref:Treference;
      begin
        if current_procinfo.framepointer=NR_STACK_POINTER_REG then
          begin
            location_reset(location,LOC_REGISTER,OS_ADDR);
            location.register:=cg.getaddressregister(current_asmdata.currasmlist);
            reference_reset_base(frame_ref,NR_STACK_POINTER_REG,{current_procinfo.calc_stackframe_size}tg.lasttemp,ctempposinvalid,sizeof(pint),[]);
            cg.a_load_ref_reg(current_asmdata.currasmlist,OS_ADDR,OS_ADDR,frame_ref,location.register);
          end
        else
          begin
            location_reset(location,LOC_REGISTER,OS_ADDR);
            location.register:=cg.getaddressregister(current_asmdata.currasmlist);
            reference_reset_base(frame_ref,current_procinfo.framepointer,sizeof(pint),ctempposinvalid,sizeof(pint),[]);
            cg.a_load_ref_reg(current_asmdata.currasmlist,OS_ADDR,OS_ADDR,frame_ref,location.register);
          end;
      end;


    procedure tcginlinenode.second_rox_sar;
      var
        op : topcg;
        op1,op2 : tnode;
      begin
        { one or two parameters? }
        if (left.nodetype=callparan) and
           assigned(tcallparanode(left).right) then
          begin
            op1:=tcallparanode(tcallparanode(left).right).left;
            op2:=tcallparanode(left).left;
            secondpass(op2);
          end
        else
          begin
            op1:=left;
            op2:=nil;
          end;

        secondpass(op1);
        case inlinenumber of
          in_ror_x,
          in_ror_x_y:
            op:=OP_ROR;
          in_rol_x,
          in_rol_x_y:
            op:=OP_ROL;
          in_sar_x,
          in_sar_x_y:
            op:=OP_SAR;
          else
            internalerror(2013120110);
        end;

        hlcg.location_force_reg(current_asmdata.CurrAsmList,op1.location,op1.resultdef,resultdef,true);

        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
        if def_cgsize(resultdef) in [OS_64,OS_S64] then
          begin
            location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
          end
        else
{$endif not cpu64bitalu and not cpuhighleveltarget}
          location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);

        if assigned(op2) then
          begin
             { rotating by a constant directly coded: }
             if op2.nodetype=ordconstn then
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
               if def_cgsize(resultdef) in [OS_64,OS_S64] then
                 cg64.a_op64_const_reg_reg(current_asmdata.CurrAsmList,op,def_cgsize(resultdef),
                   tordconstnode(op2).value.uvalue and (resultdef.size*8-1),
                   op1.location.register64, location.register64)
               else
{$endif not cpu64bitalu and not cpuhighleveltarget}
                 hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,op,resultdef,
                   tordconstnode(op2).value.uvalue and (resultdef.size*8-1),
                   op1.location.register, location.register)
             else
               begin
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
                 if def_cgsize(resultdef) in [OS_64,OS_S64] then
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,op2.location,
                                             op2.resultdef,alusinttype,true);
                     cg64.a_op64_reg_reg_reg(current_asmdata.CurrAsmList,op,def_cgsize(resultdef),
                                             joinreg64(op2.location.register,NR_NO),op1.location.register64,
                                             location.register64);
                   end
                 else
{$endif not cpu64bitalu and not cpuhighleveltarget}
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,op2.location,
                                             op2.resultdef,resultdef,true);
                     hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,op,resultdef,
                                           op2.location.register,op1.location.register,
                                           location.register);
                   end;
               end;
          end
        else
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
          if def_cgsize(resultdef) in [OS_64,OS_S64] then
            cg64.a_op64_const_reg_reg(current_asmdata.CurrAsmList,op,def_cgsize(resultdef),1,
                                      op1.location.register64,location.register64)
          else
{$endif not cpu64bitalu and not cpuhighleveltarget}
            hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,op,resultdef,1,
                                    op1.location.register,location.register);
      end;


    procedure tcginlinenode.second_BsfBsr;
    var
      reverse: boolean;
      opsize: tcgsize;
    begin
      reverse:=(inlinenumber = in_bsr_x);
      secondpass(left);

      opsize:=tcgsize2unsigned[left.location.size];
      if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(opsize),true);

      location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
      location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
      cg.a_bit_scan_reg_reg(current_asmdata.CurrAsmList,reverse,opsize,location.size,left.location.register,location.register);
    end;


    procedure tcginlinenode.second_new;
      begin
        internalerror(2011012202);
      end;


    procedure tcginlinenode.second_popcnt;
      begin
        internalerror(2012082601);
      end;


    procedure tcginlinenode.second_fma;
      begin
        internalerror(2014032701);
      end;

begin
   cinlinenode:=tcginlinenode;
end.
