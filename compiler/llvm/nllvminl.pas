{
    Copyright (c) 2014 by Jonas Maebe

    Generate LLVM bytecode for inline nodes

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
unit nllvminl;

{$i fpcdefs.inc}

interface

    uses
      node,
      ncginl;

    type
      tllvminlinenode = class(tcginlinenode)
       protected
        procedure maybe_remove_round_trunc_typeconv;

        function first_get_frame: tnode; override;
        function first_abs_real: tnode; override;
        function first_bitscan: tnode; override;
        function first_fma: tnode; override;
        function first_sqr_real: tnode; override;
        function first_sqrt_real: tnode; override;
        function first_trunc_real: tnode; override;
        function first_popcnt: tnode; override;
       public
        procedure second_length; override;
        procedure second_high; override;
        procedure second_sqr_real; override;
        procedure second_trunc_real; override;
      end;


implementation

     uses
       verbose,globals,globtype,constexp,cutils,
       aasmbase, aasmdata,
       symconst,symtype,symdef,defutil,
       compinnr,
       nutils,nadd,nbas,ncal,ncnv,ncon,nflw,ninl,nld,nmat,
       pass_2,
       cgbase,cgutils,tgobj,hlcgobj,
       cpubase,
       llvmbase,aasmllvm,aasmllvmmetadata;

     procedure tllvminlinenode.maybe_remove_round_trunc_typeconv;
       var
         temp: tnode;
       begin
         { the prototype of trunc()/round() in the system unit is declared
           with valreal as parameter type, so the argument will always be
           extended -> remove the typeconversion to extended if any; not done
           in ninl, because there are other code generators that assume that
           the parameter to trunc has been converted to valreal (e.g. PowerPC).

           (copy from code in nx64inl, should be refactored)
         }
         if (left.nodetype=typeconvn) and
            not(nf_explicit in left.flags) and
            (ttypeconvnode(left).left.resultdef.typ=floatdef) then
           begin
             { get rid of the type conversion, so the use_vectorfpu will be
               applied to the original type }
             temp:=ttypeconvnode(left).left;
             ttypeconvnode(left).left:=nil;
             left.free;
             left:=temp;
           end;
       end;


     function tllvminlinenode.first_get_frame: tnode;
       begin
         result:=ccallnode.createintern('llvm_frameaddress',
           ccallparanode.create(genintconstnode(0),nil));
       end;

    { in general, generate regular expression rather than intrinsics: according
      to the "Performance Tips for Frontend Authors", "The optimizer is quite
      good at reasoning about general control flow and arithmetic, it is not
      anywhere near as strong at reasoning about the various intrinsics. If
      profitable for code generation purposes, the optimizer will likely form
      the intrinsics itself late in the optimization pipeline." }

    function tllvminlinenode.first_abs_real: tnode;
      var
        lefttemp,
        resulttemp: ttempcreatenode;
        stat: tstatementnode;
      begin
        result:=internalstatements(stat);
        lefttemp:=ctempcreatenode.create(left.resultdef,left.resultdef.size,tt_persistent,true);
        { assigned twice -> will be spilled if put in register }
        resulttemp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,false);

        addstatement(stat,lefttemp);
        addstatement(stat,resulttemp);

        { lefttemp:=left }
        addstatement(stat,
          cassignmentnode.create(ctemprefnode.create(lefttemp),left)
        );

        { if lefttemp>=0 then
            resulttemp:=lefttemp
          else
            resulttemp:=-lefttemp
        }
        addstatement(stat,
          cifnode.create(
            caddnode.create(
              gten,
              ctemprefnode.create(lefttemp),
              crealconstnode.create(0.0,left.resultdef)
            ),
            cassignmentnode.create(
              ctemprefnode.create(resulttemp),
              ctemprefnode.create(lefttemp)
            ),
            cassignmentnode.create(
              ctemprefnode.create(resulttemp),
              cunaryminusnode.create(ctemprefnode.create(lefttemp))
            )
          )
        );
        addstatement(stat,ctempdeletenode.create(lefttemp));
        addstatement(stat,ctempdeletenode.create_normal_temp(resulttemp));
        { return resulttemp }
        addstatement(stat,ctemprefnode.create(resulttemp));
        { reused }
        left:=nil;
      end;


    function tllvminlinenode.first_bitscan: tnode;
      var
        leftdef: tdef;
        resulttemp,
        lefttemp: ttempcreatenode;
        stat: tstatementnode;
        block: tblocknode;
        cntresult: tnode;
        procname: string[15];
      begin
        {
          if left<>0 then
            result:=llvm_ctlz/cttz(unsigned(left),true)
          else
            result:=255;
        }
        if inlinenumber=in_bsr_x then
          procname:='LLVM_CTLZ'
        else
          procname:='LLVM_CTTZ';
        leftdef:=left.resultdef;
        block:=internalstatements(stat);
        resulttemp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,false);
        addstatement(stat,resulttemp);
        lefttemp:=maybereplacewithtemp(left,block,stat,left.resultdef.size,true);
        cntresult:=
          ccallnode.createintern(
            procname,
            ccallparanode.create(cordconstnode.create(1,llvmbool1type,false),
              ccallparanode.create(
                ctypeconvnode.create_explicit(left,get_unsigned_inttype(leftdef)),nil
              )
            )
          );
        { ctlz returns the number of leading zero bits, while bsr returns the bit
          number of the first non-zero bit (with the least significant bit as 0)
          -> invert result }
        if inlinenumber=in_bsr_x then
          begin
            cntresult:=
              caddnode.create(xorn,
                cntresult,
                genintconstnode(leftdef.size*8-1)
              );
          end;
        addstatement(stat,
          cifnode.create(caddnode.create(unequaln,left.getcopy,genintconstnode(0)),
            cassignmentnode.create(
              ctemprefnode.create(resulttemp),
              cntresult
            ),
            cassignmentnode.create(
              ctemprefnode.create(resulttemp),
              genintconstnode(255)
            )
          )
        );
        if assigned(lefttemp) then
          addstatement(stat,ctempdeletenode.create(lefttemp));
        addstatement(stat,ctempdeletenode.create_normal_temp(resulttemp));
        addstatement(stat,ctemprefnode.create(resulttemp));
        left:=nil;
        result:=block;
      end;


    function tllvminlinenode.first_fma: tnode;
      var
        exceptmode: ansistring;
        procname: string[40];
      begin
        if cs_opt_fastmath in current_settings.optimizerswitches then
          begin
            case inlinenumber of
              in_fma_single:
                procname:='llvm_fma_f32';
              in_fma_double:
                procname:='llvm_fma_f64';
              in_fma_extended:
                procname:='llvm_fma_f80';
              in_fma_float128:
                procname:='llvm_fma_f128';
              else
                internalerror(2018122101);
            end;
            result:=ccallnode.createintern(procname,left);
          end
        else
          begin
            case inlinenumber of
              in_fma_single,
              in_fma_double,
              in_fma_extended,
              in_fma_float128:
                procname:='LLVM_EXPERIMENTAL_CONSTRAINED_FMA';
              else
                internalerror(2019122811);
            end;
            exceptmode:=llvm_constrainedexceptmodestring;
            result:=ccallnode.createintern(procname,
              ccallparanode.create(cstringconstnode.createpchar(ansistring2pchar(exceptmode),length(exceptmode),llvm_metadatatype),
                ccallparanode.create(cstringconstnode.createpchar(ansistring2pchar('round.dynamic'),length('round.dynamic'),llvm_metadatatype),
                  left
                )
              )
            );
          end;
        left:=nil;
      end;


    function tllvminlinenode.first_sqr_real: tnode;
      begin
        result:=nil;
        if use_vectorfpu(left.resultdef) then
          expectloc:=LOC_MMREGISTER
        else
          expectloc:=LOC_FPUREGISTER;
      end;


    function tllvminlinenode.first_sqrt_real: tnode;
      var
        exceptmode: ansistring;
        intrinsic: string[40];
      begin
        if left.resultdef.typ<>floatdef then
          internalerror(2018121601);
        if cs_opt_fastmath in current_settings.optimizerswitches then
          begin
            case tfloatdef(left.resultdef).floattype of
              s32real:
                intrinsic:='llvm_sqrt_f32';
              s64real:
                intrinsic:='llvm_sqrt_f64';
              s80real,sc80real:
                intrinsic:='llvm_sqrt_f80';
              s128real:
                intrinsic:='llvm_sqrt_f128';
              else
                internalerror(2018121602);
            end;
            result:=ccallnode.createintern(intrinsic, ccallparanode.create(left,nil));
          end
        else
          begin
            case tfloatdef(left.resultdef).floattype of
              s32real,
              s64real,
              s80real,sc80real,
              s128real:
                intrinsic:='LLVM_EXPERIMENTAL_CONSTRAINED_SQRT';
              else
                internalerror(2019122810);
            end;
            exceptmode:=llvm_constrainedexceptmodestring;
            result:=ccallnode.createintern(intrinsic,
              ccallparanode.create(cstringconstnode.createpchar(ansistring2pchar(exceptmode),length(exceptmode),llvm_metadatatype),
                ccallparanode.create(cstringconstnode.createpchar(ansistring2pchar('round.dynamic'),length('round.dynamic'),llvm_metadatatype),
                  ccallparanode.create(left,nil)
                )
              )
            );
          end;
        left:=nil;
      end;


    function tllvminlinenode.first_trunc_real: tnode;
      begin
        { fptosi is undefined if the value is out of range -> only generate
          in cast of fastmath }
        if cs_opt_fastmath in current_settings.optimizerswitches then
          begin
            maybe_remove_round_trunc_typeconv;
            expectloc:=LOC_REGISTER;
            result:=nil;
          end
        else
          result:=inherited;
      end;

    function tllvminlinenode.first_popcnt: tnode;
      begin
        result:=ctypeconvnode.create(ccallnode.createintern('LLVM_CTPOP', ccallparanode.create(left,nil)),resultdef);
        left:=nil;
      end;


    procedure tllvminlinenode.second_length;
      var
        hreg: tregister;
      begin
        second_high;
        { Dynamic arrays do not have their length attached but their maximum index }
        if is_dynamic_array(left.resultdef) then
          begin
            hreg:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
            hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_ADD,resultdef,1,location.register,hreg);
            location.register:=hreg;
          end;
      end;


    procedure tllvminlinenode.second_high;
      var
        lengthlab, nillab: tasmlabel;
        hregister: tregister;
        href: treference;
        lendef: tdef;
      begin
        secondpass(left);
        if is_shortstring(left.resultdef) then
         begin
            if not(left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
              internalerror(2014080806);
           { typecast the shortstring reference into a length byte reference }
           location_reset_ref(location,left.location.loc,def_cgsize(resultdef),left.location.reference.alignment,left.location.reference.volatility);
           hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,cpointerdef.getreusable(resultdef));
           hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.resultdef,cpointerdef.getreusable(resultdef),left.location.reference,hregister);
           hlcg.reference_reset_base(location.reference,cpointerdef.getreusable(resultdef),hregister,0,left.location.reference.temppos,left.location.reference.alignment,left.location.reference.volatility);
         end
        else
         begin
           { length in ansi/wide strings and high in dynamic arrays is at offset
             -sizeof(sizeint), for widestrings it's at -4 }
           if is_widestring(left.resultdef) then
             lendef:=u32inttype
           else
             lendef:=ossinttype;
           hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,
             left.resultdef,cpointerdef.getreusable(lendef),true);
           current_asmdata.getjumplabel(nillab);
           current_asmdata.getjumplabel(lengthlab);
           hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,cpointerdef.getreusable(lendef),OC_EQ,0,left.location.register,nillab);
           { volatility of the ansistring/widestring refers to the volatility of the
             string pointer, not of the string data }
           hlcg.reference_reset_base(href,cpointerdef.getreusable(lendef),left.location.register,-lendef.size,ctempposinvalid,lendef.alignment,[]);
           hregister:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
           hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,lendef,resultdef,href,hregister);
           if is_widestring(left.resultdef) then
             hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,resultdef,1,hregister);
           hlcg.a_jmp_always(current_asmdata.CurrAsmList,lengthlab);

           hlcg.a_label(current_asmdata.CurrAsmList,nillab);
           if is_dynamic_array(left.resultdef) then
             hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,-1,hregister)
           else
             hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,0,hregister);

           hlcg.a_label(current_asmdata.CurrAsmList,lengthlab);
           location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
           location.register:=hregister;
         end;
      end;


    procedure tllvminlinenode.second_sqr_real;
      begin
        secondpass(left);
        location.loc:=expectloc;
        if expectloc=LOC_MMREGISTER then
          begin
            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
            location.register:=hlcg.getmmregister(current_asmdata.CurrAsmList,resultdef);
          end
        else
          begin
            hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
            location.register:=hlcg.getfpuregister(current_asmdata.CurrAsmList,resultdef);
          end;
        current_asmdata.CurrAsmList.concat(
          taillvm.op_reg_size_reg_reg(la_fmul,
            location.register,resultdef,
            left.location.register,left.location.register
          )
        );
      end;


    procedure tllvminlinenode.second_trunc_real;
      begin
        secondpass(left);
        if use_vectorfpu(left.resultdef) then
          hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true)
        else
          hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,resultdef);
        current_asmdata.CurrAsmList.concat(
          taillvm.op_reg_size_reg_size(la_fptosi,location.register,left.resultdef,left.location.register,resultdef)
        );
      end;

begin
  cinlinenode:=tllvminlinenode;
end.

