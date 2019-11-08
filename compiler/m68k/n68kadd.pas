{
    Copyright (c) 2000-2002 by Florian Klaempfl and Jonas Maebe

    Code generation for add nodes on the Motorola 680x0 family

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
unit n68kadd;

{$i fpcdefs.inc}

interface

    uses
       node,nadd,ncgadd,cpubase,cgbase;


    type
       t68kaddnode = class(tcgaddnode)
       private
          function getresflags(unsigned: boolean) : tresflags;
          function getfloatresflags: tresflags;
          function inlineable_realconstnode(const n: tnode): boolean;
          procedure second_mul64bit;
       protected
          function use_generic_mul64bit: boolean; override;
          function use_generic_mul32to64: boolean; override;
          function use_mul_helper: boolean; override;
          procedure second_addfloat;override;
          procedure second_cmpfloat;override;
          procedure second_addordinal;override;
          procedure second_cmpordinal;override;
          procedure second_cmpsmallset;override;
          procedure second_add64bit;override;
          procedure second_cmp64bit;override;
       end;


implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,symtype,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,htypechk,
      cpuinfo,pass_1,pass_2,
      cpupara,cgutils,procinfo,
      ncon,nset,
      ncgutil,tgobj,rgobj,rgcpu,cgobj,cgcpu,hlcgobj,cg64f32;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function t68kaddnode.getresflags(unsigned : boolean) : tresflags;
      begin
         case nodetype of
           equaln : getresflags:=F_E;
           unequaln : getresflags:=F_NE;
          else
           if not(unsigned) then
             begin
                if nf_swapped in flags then
                  case nodetype of
                     ltn : getresflags:=F_G;
                     lten : getresflags:=F_GE;
                     gtn : getresflags:=F_L;
                     gten : getresflags:=F_LE;
                     else
                       internalerror(2014082030);
                  end
                else
                  case nodetype of
                     ltn : getresflags:=F_L;
                     lten : getresflags:=F_LE;
                     gtn : getresflags:=F_G;
                     gten : getresflags:=F_GE;
                     else
                       internalerror(2014082031);
                  end;
             end
           else
             begin
                if nf_swapped in flags then
                  case nodetype of
                     ltn : getresflags:=F_A;
                     lten : getresflags:=F_AE;
                     gtn : getresflags:=F_B;
                     gten : getresflags:=F_BE;
                     else
                       internalerror(2014082032);
                  end
                else
                  case nodetype of
                     ltn : getresflags:=F_B;
                     lten : getresflags:=F_BE;
                     gtn : getresflags:=F_A;
                     gten : getresflags:=F_AE;
                     else
                       internalerror(2014082033);
                  end;
             end;
         end;
      end;


    function t68kaddnode.getfloatresflags : tresflags;
      begin
        case nodetype of
          equaln : getfloatresflags:=F_FE;
          unequaln : getfloatresflags:=F_FNE;
          else
            if nf_swapped in flags then
              case nodetype of
                ltn : getfloatresflags:=F_FG;
                lten : getfloatresflags:=F_FGE;
                gtn : getfloatresflags:=F_FL;
                gten : getfloatresflags:=F_FLE;
                else
                  internalerror(201604260);
              end
            else
              case nodetype of
                ltn : getfloatresflags:=F_FL;
                lten : getfloatresflags:=F_FLE;
                gtn : getfloatresflags:=F_FG;
                gten : getfloatresflags:=F_FGE;
                else
                  internalerror(201604261);
              end;
        end;
      end;


    function t68kaddnode.inlineable_realconstnode(const n: tnode): boolean;
      begin
        result:=(n.nodetype = realconstn) and
            not ((trealconstnode(n).value_real=MathInf.Value) or
                 (trealconstnode(n).value_real=MathNegInf.Value) or
                 (trealconstnode(n).value_real=MathQNaN.value));
      end;


{*****************************************************************************
                                AddFloat
*****************************************************************************}

    procedure t68kaddnode.second_addfloat;
      var
        op    : TAsmOp;
        href  : TReference;
      begin
        pass_left_right;

        case nodetype of
          addn :
            op:=A_FADD;
          muln :
            op:=A_FMUL;
          subn :
            op:=A_FSUB;
          slashn :
            op:=A_FDIV;
          else
            internalerror(200403182);
        end;

        // get the operands in the correct order, there are no special cases
        // here, everything is register-based
        if nf_swapped in flags then
          swapleftright;

        if not (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) then
          internalerror(2015010201);

        location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));

        { have left in the register, right can be a memory location }
        if (FPUM68K_HAS_FLOATIMMEDIATE in fpu_capabilities[current_settings.fputype]) and
           inlineable_realconstnode(left) then
          begin
            location.register := cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
            current_asmdata.CurrAsmList.concat(taicpu.op_realconst_reg(A_FMOVE,tcgsize2opsize[left.location.size],trealconstnode(left).value_real,location.register))
          end
        else
          begin
            hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);

            location.register := cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
            cg.a_loadfpu_reg_reg(current_asmdata.CurrAsmlist,OS_NO,OS_NO,left.location.register,location.register);
          end;

        { emit the actual operation }
        case right.location.loc of
          LOC_FPUREGISTER,LOC_CFPUREGISTER:
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,fpuregopsize,right.location.register,location.register));
          LOC_REFERENCE,LOC_CREFERENCE:
              begin
                if (FPUM68K_HAS_FLOATIMMEDIATE in fpu_capabilities[current_settings.fputype]) and
                   inlineable_realconstnode(right) then
                  current_asmdata.CurrAsmList.concat(taicpu.op_realconst_reg(op,tcgsize2opsize[right.location.size],trealconstnode(right).value_real,location.register))
                else
                  begin
                    href:=right.location.reference;
                    tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,current_settings.fputype = fpu_coldfire);
                    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,tcgsize2opsize[right.location.size],href,location.register));
                  end;
              end
          else
            internalerror(2015021501);
        end;
      end;


    procedure t68kaddnode.second_cmpfloat;
      var
        tmpreg : tregister;
        ai: taicpu;
        href  : TReference;
      begin
        pass_left_right;
        if (nf_swapped in flags) then
          swapleftright;

        if not (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) then
          internalerror(2019090601);

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getfloatresflags;

        { emit compare }
        case right.location.loc of
          LOC_FPUREGISTER,LOC_CFPUREGISTER:
              begin
                //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_cmpfloat right reg!')));
                if left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
                  begin
                    href:=left.location.reference;
                    tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,current_settings.fputype = fpu_coldfire);
                    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_FCMP,tcgsize2opsize[left.location.size],href,right.location.register));
                    toggleflag(nf_swapped);
                    location.resflags:=getfloatresflags;
                  end
                else
                  begin
                    hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FCMP,fpuregopsize,right.location.register,left.location.register));
                  end;
              end;
          LOC_REFERENCE,LOC_CREFERENCE:
              begin
                { use FTST, if realconst is 0.0, it would be hard to do this in the
                  optimizer, because we would need to investigate the referenced value... }
                if (right.nodetype = realconstn) and
                   (trealconstnode(right).value_real = 0.0) then
                  begin
                    if left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_FTST,fpuregopsize,left.location.register))
                    else
                      if left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
                        begin
                          href:=left.location.reference;
                          tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,false);
                          current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_FTST,tcgsize2opsize[left.location.size],href))
                        end
                      else
                        internalerror(2016051001);
                  end
                else
                  begin
                    hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
                    if not (current_settings.fputype = fpu_coldfire) and
                       inlineable_realconstnode(right) then
                      current_asmdata.CurrAsmList.concat(taicpu.op_realconst_reg(A_FCMP,tcgsize2opsize[right.location.size],trealconstnode(right).value_real,left.location.register))
                    else
                      begin
                        href:=right.location.reference;
                        tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,current_settings.fputype = fpu_coldfire);
                        current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_FCMP,tcgsize2opsize[right.location.size],href,left.location.register));
                      end;
                  end;
              end
          else
            internalerror(2015021502);
        end;
      end;




{*****************************************************************************
                                Smallsets
*****************************************************************************}

    procedure t68kaddnode.second_cmpsmallset;
     var
       tmpreg : tregister;
       opsize: topsize;
       cmpsize : tcgsize;
     begin
       pass_left_right;

       location_reset(location,LOC_FLAGS,OS_NO);

       cmpsize:=def_cgsize(left.resultdef);
       opsize:=tcgsize2opsize[cmpsize];

       { Coldfire supports byte/word compares only starting with ISA_B,
         See remark about Qemu weirdness in tcg68k.a_cmp_const_reg_label }
       if (opsize<>S_L) and (current_settings.cputype in cpu_coldfire{-[cpu_isa_b,cpu_isa_c,cfv4e]}) then
         begin
           cmpsize:=OS_32;
           opsize:=S_L;
         end;

       if (not(nf_swapped in flags) and
           (nodetype = lten)) or
          ((nf_swapped in flags) and
           (nodetype = gten)) then
         swapleftright;

       { Try to keep right as a constant }
       if right.location.loc<>LOC_CONSTANT then
         hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cgsize_orddef(cmpsize),true);
       hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(cmpsize),true);

       case nodetype of
         equaln,
         unequaln:
           begin
             if right.location.loc=LOC_CONSTANT then
               current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_CMP,opsize,right.location.value,left.location.register))
             else
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,opsize,right.location.register,left.location.register));
             if nodetype=equaln then
               location.resflags:=F_E
             else
               location.resflags:=F_NE;
           end;
         lten,
         gten:
           begin
             tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,left.location.size);
             if right.location.loc=LOC_CONSTANT then
               hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cgsize_orddef(cmpsize),false);
             cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_AND,cmpsize,left.location.register,right.location.register,tmpreg);
             current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,opsize,tmpreg,right.location.register));
             location.resflags:=F_E;
           end;
         else
           internalerror(2013092701);
       end;
     end;


{*****************************************************************************
                                Ordinals
*****************************************************************************}

    function t68kaddnode.use_mul_helper: boolean;
      begin
        result:=(nodetype=muln) and not (CPUM68K_HAS_32BITMUL in cpu_capabilities[current_settings.cputype]);
      end;

    procedure t68kaddnode.second_addordinal;
      const
        mul_op_signed: array[boolean] of tasmop = ( A_MULU, A_MULS );
      var
        cgop    : topcg;
        asmop   : tasmop;
        list    : tasmlist;
        href    : treference;
      begin
        { if we need to handle overflow checking, fall back to the generic cg }
        if (nodetype in [addn,subn,muln]) and
           needoverflowcheck then
          begin
            inherited;
            exit;
          end;

        list:=current_asmdata.CurrAsmList;

        case nodetype of
          addn: cgop:=OP_ADD;
          xorn: cgop:=OP_XOR;
          orn : cgop:=OP_OR;
          andn: cgop:=OP_AND;
          subn: cgop:=OP_SUB;
          muln:
            begin
              if not(is_signed(left.resultdef)) or
                 not(is_signed(right.resultdef)) then
                cgop:=OP_MUL
              else
                cgop:=OP_IMUL;
            end;
          else
            internalerror(2013120104);
        end;

        pass_left_right;
        if (nodetype=subn) and (nf_swapped in flags) then
          swapleftright;

        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

        { initialize the result }
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));

        { this is only true, if the CPU supports 32x32 -> 64 bit MUL, see the relevant method }
        if (nodetype=muln) and is_64bit(resultdef) then
          begin
            list.concat(tai_comment.create(strpnew('second_addordinal: mul32to64bit')));

            asmop:=mul_op_signed[cgop = OP_IMUL];
            location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            cg.a_load_reg_reg(list,left.location.size,OS_INT,left.location.register,location.register64.reglo);

            if not (right.location.size in [OS_S32, OS_32]) or
               not (right.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_CONSTANT,LOC_REFERENCE,LOC_CREFERENCE]) or
               ((right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and needs_unaligned(right.location.reference.alignment,def_cgsize(resultdef))) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

            case right.location.loc of
              LOC_REGISTER,
              LOC_CREGISTER:
                list.concat(taicpu.op_reg_reg_reg(asmop,S_L,right.location.register,location.register64.reghi,location.register64.reglo));
              LOC_CONSTANT:
                list.concat(taicpu.op_const_reg_reg(asmop,S_L,right.location.value,location.register64.reghi,location.register64.reglo));
              LOC_REFERENCE,
              LOC_CREFERENCE:
                begin
                  href:=right.location.reference;
                  tcg68k(cg).fixref(list,href,false);
                  list.concat(taicpu.op_ref_reg_reg(asmop,S_L,href,location.register64.reghi,location.register64.reglo));
                 end;
              else
                internalerror(2017052601);
            end;
            exit;
          end;

        if isaddressregister(left.location.register) and (nodetype in [addn,subn]) then
           location.register := cg.getaddressregister(current_asmdata.CurrAsmList)
        else
           location.register := cg.getintregister(current_asmdata.CurrAsmList,location.size);
        cg.a_load_reg_reg(current_asmdata.CurrAsmlist,left.location.size,location.size,left.location.register,location.register);

        if ((location.size <> right.location.size) and not (right.location.loc in [LOC_CONSTANT])) or
           not (right.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_CONSTANT,LOC_REFERENCE,LOC_CREFERENCE]) or
           (not(CPUM68K_HAS_32BITMUL in cpu_capabilities[current_settings.cputype]) and (nodetype = muln)) or
           ((right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and needs_unaligned(right.location.reference.alignment,def_cgsize(resultdef))) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

        case right.location.loc of
          LOC_REGISTER,
          LOC_CREGISTER:
            cg.a_op_reg_reg(current_asmdata.CurrAsmList,cgop,def_cgsize(resultdef),right.location.register,location.register);
          LOC_CONSTANT:
            cg.a_op_const_reg(current_asmdata.CurrAsmList,cgop,def_cgsize(resultdef),right.location.value,location.register);
          LOC_REFERENCE,
          LOC_CREFERENCE:
            cg.a_op_ref_reg(current_asmdata.CurrAsmList,cgop,def_cgsize(resultdef),right.location.reference,location.register);
        else
          internalerror(2016052101);
        end;
      end;


    procedure t68kaddnode.second_cmpordinal;
     var
      unsigned : boolean;
      tmpreg : tregister;
      opsize : topsize;
      cmpsize : tcgsize;
      href: treference;
     begin
       { determine if the comparison will be unsigned }
       unsigned:=not(is_signed(left.resultdef)) or
                   not(is_signed(right.resultdef));
       { this puts constant operand (if any) to the right }
       pass_left_right;
       { tentatively assume left size (correct for possible TST, will fix later) }
       cmpsize:=def_cgsize(left.resultdef);
       opsize:=tcgsize2opsize[cmpsize];

       { set result location }
       location_reset(location,LOC_FLAGS,OS_NO);

       { see if we can optimize into TST }
       if (right.location.loc=LOC_CONSTANT) and (right.location.value=0) then
         begin
           { Unsigned <0 or >=0 should not reach pass2, most likely }
           if (left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and not needs_unaligned(left.location.reference.alignment,cmpsize) then
             begin
               href:=left.location.reference;
               tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,false);
               current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_TST,opsize,href));
               location_freetemp(current_asmdata.CurrAsmList,left.location);
             end
           else
             begin
               hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
               if (current_settings.cputype = cpu_mc68000) and isaddressregister(left.location.register) then
                 begin
                   tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,cmpsize);
                   cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_ADDR,cmpsize,left.location.register,tmpreg);
                 end
               else
                 tmpreg:=left.location.register;
               current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_TST,opsize,tmpreg));
             end;
           location.resflags := getresflags(unsigned);
           exit;
         end;

       { Coldfire supports byte/word compares only starting with ISA_B,
         !!see remark about Qemu weirdness in tcg68k.a_cmp_const_reg_label }
       if (opsize<>S_L) and (current_settings.cputype in cpu_coldfire{-[cpu_isa_b,cpu_isa_c,cfv4e]}) then
         begin
           { 1) Extension is needed for LOC_REFERENCE, but what about LOC_REGISTER ? Perhaps after fixing cg we can assume
                that high bits of registers are correct.
             2) Assuming that extension depends only on source signedness --> destination OS_32 is acceptable. }
           hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,cgsize_orddef(OS_32),false);
           if (right.location.loc<>LOC_CONSTANT) then
             hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cgsize_orddef(OS_32),false);
           opsize:=S_L;
         end
       else if not (left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
         begin
           if not (right.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
             hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true)
           else
             begin
               location_swap(left.location,right.location);
               toggleflag(nf_swapped);
             end;
         end;

       if (right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and needs_unaligned(right.location.reference.alignment,cmpsize) then
         hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

       { left is now in register }
       case right.location.loc of
         LOC_CONSTANT:
           current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_CMP,opsize,
             longint(right.location.value),left.location.register));
         LOC_REFERENCE,
         LOC_CREFERENCE:
           begin
             href:=right.location.reference;
             tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,false);
             current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_CMP,opsize,href,
               left.location.register));
           end;
         LOC_REGISTER,
         LOC_CREGISTER:
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,opsize,
             right.location.register,left.location.register));
       else
         hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
         current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,opsize,
           right.location.register,left.location.register));
       end;

       { update location because sides could have been swapped }
       location.resflags:=getresflags(unsigned);
     end;


{*****************************************************************************
                                64-bit
*****************************************************************************}

    function t68kaddnode.use_generic_mul32to64: boolean;
    begin
      result:=not (CPUM68K_HAS_64BITMUL in cpu_capabilities[current_settings.cputype]);
    end;

    function t68kaddnode.use_generic_mul64bit: boolean;
    begin
      result:=needoverflowcheck  or
        (cs_opt_size in current_settings.optimizerswitches) or
        not (CPUM68K_HAS_64BITMUL in cpu_capabilities[current_settings.cputype]);
    end;

    procedure t68kaddnode.second_add64bit;
    begin
      if (nodetype=muln) then
        second_mul64bit
      else
        inherited second_add64bit;
    end;

    procedure t68kaddnode.second_mul64bit;
      var
       list: TAsmList;
       hreg1,hreg2,tmpreg: TRegister;
      begin
        list:=current_asmdata.CurrAsmList;
        pass_left_right;
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        hlcg.location_force_reg(list,left.location,left.resultdef,left.resultdef,true);

        { calculate 32-bit terms lo(right)*hi(left) and hi(left)*lo(right) }
        hreg1:=NR_NO;
        hreg2:=NR_NO;
        tmpreg:=NR_NO;
        if (right.location.loc=LOC_CONSTANT) then
          begin
            //list.concat(tai_comment.create(strpnew('second_mul64bit: with const')));
            { Omit zero terms, if any }
            if hi(right.location.value64)<>0 then
              begin
                hreg2:=cg.getintregister(list,OS_INT);
                cg.a_load_const_reg(list,OS_INT,longint(hi(right.location.value64)),hreg2);
                list.concat(taicpu.op_reg_reg(A_MULU,S_L,left.location.register64.reglo,hreg2));
              end;
            if lo(right.location.value64)<>0 then
              begin
                hreg1:=cg.getintregister(list,OS_INT);
                tmpreg:=cg.getintregister(list,OS_INT);
                cg.a_load_const_reg(list,OS_INT,longint(lo(right.location.value64)),hreg1);
                cg.a_load_reg_reg(list,OS_INT,OS_INT,hreg1,tmpreg);
                list.concat(taicpu.op_reg_reg(A_MULU,S_L,left.location.register64.reghi,hreg1));
              end;
          end
        else
          begin
            //list.concat(tai_comment.create(strpnew('second_mul64bit: no const')));
            hlcg.location_force_reg(list,right.location,right.resultdef,right.resultdef,true);
            tmpreg:=right.location.register64.reglo;
            hreg1:=cg.getintregister(list,OS_INT);
            hreg2:=cg.getintregister(list,OS_INT);
            cg.a_load_reg_reg(list,OS_INT,OS_INT,right.location.register64.reglo,hreg1);
            cg.a_load_reg_reg(list,OS_INT,OS_INT,right.location.register64.reghi,hreg2);
            list.concat(taicpu.op_reg_reg(A_MULU,S_L,left.location.register64.reghi,hreg1));
            list.concat(taicpu.op_reg_reg(A_MULU,S_L,left.location.register64.reglo,hreg2));
          end;

        { At this point, tmpreg is either lo(right) or NR_NO if lo(left)*lo(right) is zero }
        if (tmpreg=NR_NO) then
          begin
            if (hreg2<>NR_NO) then
              begin
                location.register64.reghi:=hreg2;
                if (hreg1<>NR_NO) then
                  list.concat(taicpu.op_reg_reg(A_ADD,S_L,hreg1,location.register64.reghi));
              end
            else if (hreg1<>NR_NO) then
              location.register64.reghi:=hreg1
            else
              internalerror(2017052501);
            location.register64.reglo:=cg.getintregister(list,OS_INT);
            cg.a_load_const_reg(list,OS_INT,0,location.register64.reglo);
          end
        else
          begin
            location.register64.reghi:=cg.getintregister(list,OS_INT);
            location.register64.reglo:=cg.getintregister(list,OS_INT);
            cg.a_load_reg_reg(list,OS_INT,OS_INT,left.location.register64.reglo,location.register64.reglo);
            list.concat(taicpu.op_reg_reg_reg(A_MULU,S_L,tmpreg,location.register64.reghi,location.register64.reglo));
            if (hreg2<>NR_NO) then
              list.concat(taicpu.op_reg_reg(A_ADD,S_L,hreg2,location.register64.reghi));
            if (hreg1<>NR_NO) then
              list.concat(taicpu.op_reg_reg(A_ADD,S_L,hreg1,location.register64.reghi));
          end;
      end;

    procedure t68kaddnode.second_cmp64bit;
      var
        truelabel,
        falselabel: tasmlabel;
        hlab: tasmlabel;
        unsigned : boolean;
        href: treference;

      procedure firstjmp64bitcmp;
        var
          oldnodetype : tnodetype;
        begin
          case nodetype of
            ltn,gtn:
              begin
                if (hlab<>location.truelabel) then
                  cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.truelabel);
                { cheat a little bit for the negative test }
                toggleflag(nf_swapped);
                if (hlab<>location.falselabel) then
                  cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.falselabel);
                toggleflag(nf_swapped);
              end;
            lten,gten:
              begin
                oldnodetype:=nodetype;
                if nodetype=lten then
                  nodetype:=ltn
                else
                  nodetype:=gtn;
                if (hlab<>location.truelabel) then
                  cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.truelabel);
                { cheat for the negative test }
                if nodetype=ltn then
                  nodetype:=gtn
                else
                  nodetype:=ltn;
                if (hlab<>location.falselabel) then
                  cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.falselabel);
                nodetype:=oldnodetype;
              end;
            equaln:
              cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NE,location.falselabel);
            unequaln:
              cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NE,location.truelabel);
            else
              ;
          end;
        end;

      procedure secondjmp64bitcmp;
        begin
          case nodetype of
            ltn,gtn,lten,gten:
              begin
                cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(true),location.truelabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
              end;
            equaln:
              begin
                cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NE,location.falselabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,location.truelabel);
              end;
            unequaln:
              begin
                cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NE,location.truelabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
              end;
            else
              ;
          end;
        end;

      begin
        truelabel:=nil;
        falselabel:=nil;
        { This puts constant operand (if any) to the right }
        pass_left_right;

        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        current_asmdata.getjumplabel(truelabel);
        current_asmdata.getjumplabel(falselabel);
        location_reset_jump(location,truelabel,falselabel);

        { Relational compares against constants having low dword=0 can omit the
          second compare based on the fact that any unsigned value is >=0 }
        hlab:=nil;
        if (right.location.loc=LOC_CONSTANT) and
           (lo(right.location.value64)=0) then
          begin
            case getresflags(true) of
              F_AE: hlab:=location.truelabel;
              F_B:  hlab:=location.falselabel;
              else
                ;
            end;
          end;

        if (right.location.loc=LOC_CONSTANT) and (right.location.value64=0) and
          (nodetype in [equaln,unequaln]) then
          begin
            if (left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and not needs_unaligned(left.location.reference.alignment,OS_INT) then
              begin
                href:=left.location.reference;
                tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,false);
                current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_TST,S_L,href));
                firstjmp64bitcmp;
                inc(href.offset,4);
                current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_TST,S_L,href));
                secondjmp64bitcmp;
                location_freetemp(current_asmdata.CurrAsmList,left.location);
              end
            else
              begin
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_TST,S_L,left.location.register64.reglo));
                firstjmp64bitcmp;
                current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_TST,S_L,left.location.register64.reghi));
                secondjmp64bitcmp;
              end;
            exit;
          end;

        { left and right no register?  }
        { then one must be demanded    }
        if not (left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
          begin
            if not (right.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true)
            else
              begin
                location_swap(left.location,right.location);
                toggleflag(nf_swapped);
              end;
          end;

        if (right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and needs_unaligned(right.location.reference.alignment,OS_INT) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

        { left is now in register }
        case right.location.loc of
          LOC_REGISTER,LOC_CREGISTER:
            begin
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,S_L,right.location.register64.reghi,left.location.register64.reghi));
              firstjmp64bitcmp;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,S_L,right.location.register64.reglo,left.location.register64.reglo));
              secondjmp64bitcmp;
            end;
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              href:=right.location.reference;
              tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,false);
              current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_CMP,S_L,href,left.location.register64.reghi));
              firstjmp64bitcmp;
              inc(href.offset,4);
              current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_CMP,S_L,href,left.location.register64.reglo));
              secondjmp64bitcmp;
              location_freetemp(current_asmdata.CurrAsmList,right.location);
            end;
          LOC_CONSTANT:
            begin
              current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_CMP,S_L,aint(hi(right.location.value64)),left.location.register64.reghi));
              firstjmp64bitcmp;
              if assigned(hlab) then
                cg.a_jmp_always(current_asmdata.CurrAsmList,hlab)
              else
                begin
                  current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(A_CMP,S_L,aint(lo(right.location.value64)),left.location.register64.reglo));
                  secondjmp64bitcmp;
                end;
            end;
        else
          InternalError(2014072501);
        end;
      end;


begin
   caddnode:=t68kaddnode;
end.
