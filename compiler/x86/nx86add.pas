{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Common code generation for add nodes on the i386 and x86

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
unit nx86add;

{$i fpcdefs.inc}

  interface

    uses
      symtype,
      cgbase,
      cpubase,
      node,nadd,ncgadd;

    type
      tx86addnode = class(tcgaddnode)
      protected
        function  getresflags(unsigned : boolean) : tresflags;
        function  getfpuresflags : tresflags;
        procedure left_must_be_reg(opdef: tdef; opsize:TCGSize;noswap:boolean);
        procedure force_left_and_right_fpureg;
        procedure prepare_x87_locations(out refnode: tnode);
        procedure emit_op_right_left(op:TAsmOp;opsize:TCgSize);
        procedure emit_generic_code(op:TAsmOp;opsize:TCgSize;unsigned,extra_not,mboverflow:boolean);

        procedure second_cmpfloatvector;

        procedure second_addfloatsse;
        procedure second_addfloatavx;
      public
        function use_fma : boolean;override;
        procedure second_addfloat;override;
{$ifndef i8086}
        procedure second_addsmallset;override;
{$endif not i8086}
        procedure second_add64bit;override;
        procedure second_cmpfloat;override;
        procedure second_cmpsmallset;override;
        procedure second_cmp64bit;override;

        procedure second_cmpordinal;override;
        procedure second_addordinal;override;
{$ifdef SUPPORT_MMX}
        procedure second_opmmx;override;
{$endif SUPPORT_MMX}
        procedure second_opvector;override;
      end;


  implementation

    uses
      globtype,globals,
      verbose,cutils,compinnr,
      cpuinfo,
      aasmbase,aasmdata,aasmcpu,
      symconst,symdef,
      cgobj,hlcgobj,cgx86,cga,cgutils,
      tgobj,ncgutil,
      ncon,nset,ninl,
      defutil;

{ Range check must be disabled explicitly as the code serves
  on three different architecture sizes }
{$R-}

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    procedure tx86addnode.emit_generic_code(op:TAsmOp;opsize:TCGSize;unsigned,extra_not,mboverflow:boolean);
      var
        power : longint;
        hl4   : tasmlabel;
        r     : Tregister;
        href  : treference;
        overflowcheck: boolean;
      begin
        overflowcheck:=needoverflowcheck;
        { at this point, left.location.loc should be LOC_REGISTER }
        if right.location.loc=LOC_REGISTER then
         begin
           { right.location is a LOC_REGISTER }
           { when swapped another result register }
           if (nodetype=subn) and (nf_swapped in flags) then
            begin
              if extra_not then
               emit_reg(A_NOT,TCGSize2Opsize[opsize],left.location.register);
              emit_reg_reg(op,TCGSize2Opsize[opsize],left.location.register,right.location.register);
              { newly swapped also set swapped flag }
              location_swap(left.location,right.location);
              toggleflag(nf_swapped);
            end
           else
            begin
              if extra_not then
                emit_reg(A_NOT,TCGSize2Opsize[opsize],right.location.register);
              if (op=A_ADD) or (op=A_OR) or (op=A_AND) or (op=A_XOR) or (op=A_IMUL) then
                location_swap(left.location,right.location);
              emit_reg_reg(op,TCGSize2Opsize[opsize],right.location.register,left.location.register);
            end;
         end
        else
         begin
           { right.location is not a LOC_REGISTER }
           if (nodetype=subn) and (nf_swapped in flags) then
            begin
              if extra_not then
                cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NOT,opsize,left.location.register,left.location.register);
              r:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
              hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,cgsize_orddef(opsize),right.location,r);
              emit_reg_reg(op,TCGSize2Opsize[opsize],left.location.register,r);
              cg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,opsize,r,left.location.register);
            end
           else
            begin
               { Optimizations when right.location is a constant value }
               if (op=A_CMP) and
                  (nodetype in [equaln,unequaln]) and
                  (right.location.loc=LOC_CONSTANT) and
                  (right.location.value=0) then
                 begin
                   { 'test $-1,%reg' is transformable into 'test $-1,spilltemp' if %reg needs
                      spilling, while 'test %reg,%reg' still requires loading into register.
                      If spilling is not necessary, it is changed back into 'test %reg,%reg' by
                      peephole optimizer (this optimization is currently available only for i386). }
                   cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
{$ifdef i386}
                   emit_const_reg(A_TEST,TCGSize2Opsize[opsize],aint(-1),left.location.register)
{$else i386}
                   emit_reg_reg(A_TEST,TCGSize2Opsize[opsize],left.location.register,left.location.register);
{$endif i386}
                 end
               else
                 if (op=A_ADD) and
                    (right.location.loc=LOC_CONSTANT) and
                    (right.location.value=1) and
                    not overflowcheck and
                    UseIncDec then
                  begin
                    emit_reg(A_INC,TCGSize2Opsize[opsize],left.location.register);
                  end
               else
                 if (op=A_SUB) and
                    (right.location.loc=LOC_CONSTANT) and
                    (right.location.value=1) and
                    overflowcheck and
                    UseIncDec then
                  begin
                    emit_reg(A_DEC,TCGSize2Opsize[opsize],left.location.register);
                  end
               else
                 if (op=A_IMUL) and
                    (right.location.loc=LOC_CONSTANT) and
                    (ispowerof2(int64(right.location.value),power)) and
                    overflowcheck then
                  begin
                    emit_const_reg(A_SHL,TCGSize2Opsize[opsize],power,left.location.register);
                  end
                else if (op=A_IMUL) and
                    (right.location.loc=LOC_CONSTANT) and
                    (right.location.value>1) and (ispowerof2(int64(right.location.value)-1,power)) and
                    (power in [1..3]) and
                    not overflowcheck then
                  begin
                    reference_reset_base(href,left.location.register,0,ctempposinvalid,0,[]);
                    href.index:=left.location.register;
                    href.scalefactor:=int64(right.location.value)-1;
                    left.location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_LEA,TCgSize2OpSize[opsize],href,left.location.register));
                  end
               else
                 begin
                   if extra_not then
                     begin
                        r:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                        hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,cgsize_orddef(opsize),right.location,r);
                        emit_reg(A_NOT,TCGSize2Opsize[opsize],r);
                        emit_reg_reg(A_AND,TCGSize2Opsize[opsize],r,left.location.register);
                     end
                   else
                     begin
                        emit_op_right_left(op,opsize);
                     end;
                 end;
            end;
         end;

        { only in case of overflow operations }
        { produce overflow code }
        { we must put it here directly, because sign of operation }
        { is in unsigned VAR!!                                   }
        if mboverflow then
         begin
           if overflowcheck then
            begin
              current_asmdata.getjumplabel(hl4);
              if unsigned then
                cg.a_jmp_flags(current_asmdata.CurrAsmList,F_AE,hl4)
              else
                cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NO,hl4);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_OVERFLOW',false);
              cg.a_label(current_asmdata.CurrAsmList,hl4);
            end;
         end;
      end;


    procedure tx86addnode.left_must_be_reg(opdef: tdef; opsize:TCGSize;noswap:boolean);
      begin
        { left location is not a register? }
        if (left.location.loc<>LOC_REGISTER) then
         begin
           { if right is register then we can swap the locations }
           if (not noswap) and
              (right.location.loc=LOC_REGISTER) then
            begin
              location_swap(left.location,right.location);
              toggleflag(nf_swapped);
            end
           else if (not noswap) and
              (right.location.loc=LOC_CREGISTER) then
            begin
              location_swap(left.location,right.location);
              toggleflag(nf_swapped);
              { maybe we can reuse a constant register when the
                operation is a comparison that doesn't change the
                value of the register }
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,(nodetype in [ltn,lten,gtn,gten,equaln,unequaln]));
              location:=left.location;
            end
           else
            begin
              { maybe we can reuse a constant register when the
                operation is a comparison that doesn't change the
                value of the register }
                hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,(nodetype in [ltn,lten,gtn,gten,equaln,unequaln]));
            end;
          end;
        if (right.location.loc<>LOC_CONSTANT) and
           (tcgsize2unsigned[right.location.size]<>tcgsize2unsigned[opsize]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,opdef,true);
        if (left.location.loc<>LOC_CONSTANT) and
           (tcgsize2unsigned[left.location.size]<>tcgsize2unsigned[opsize]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
       end;


    procedure tx86addnode.force_left_and_right_fpureg;
      begin
        if (right.location.loc<>LOC_FPUREGISTER) then
          begin
            hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,false);
            if (left.location.loc<>LOC_FPUREGISTER) then
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,false)
            else
              { left was on the stack => swap }
              toggleflag(nf_swapped);
          end
        { the nominator in st0 }
        else if (left.location.loc<>LOC_FPUREGISTER) then
          begin
            hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,false)
          end
        else
          begin
            { fpu operands are always in the wrong order on the stack }
            toggleflag(nf_swapped);
          end;
      end;


    { Makes sides suitable for executing an x87 instruction:
      if either side is OS_F32/OS_F64-sized LOC_REFERENCE, it is returned in 'refnode'
      everything else is loaded to FPU stack. }
    procedure tx86addnode.prepare_x87_locations(out refnode: tnode);
      begin
        refnode:=nil;

        { later on, no mm registers are allowed, so transfer everything to memory here
          below it is loaded into an fpu register if neede }
        if left.location.loc in [LOC_CMMREGISTER,LOC_MMREGISTER] then
          hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);

        if right.location.loc in [LOC_CMMREGISTER,LOC_MMREGISTER] then
          hlcg.location_force_mem(current_asmdata.CurrAsmList,right.location,right.resultdef);

        case ord(left.location.loc=LOC_FPUREGISTER)+ord(right.location.loc=LOC_FPUREGISTER) of
          0:
            begin
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,false);
              if not(left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                InternalError(2013090803);
              if (left.location.size in [OS_F32,OS_F64]) then
                begin
                  refnode:=left;
                  toggleflag(nf_swapped);
                end
              else
                hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
            end;
          1:
            begin   { if left is on the stack then swap. }
              if (left.location.loc=LOC_FPUREGISTER) then
                refnode:=right
              else
                refnode:=left;
              if not(refnode.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                InternalError(2013090801);
              if not (refnode.location.size in [OS_F32,OS_F64]) then
                begin
                  hlcg.location_force_fpureg(current_asmdata.CurrAsmList,refnode.location,refnode.resultdef,false);
                  if (refnode=right) then
                    toggleflag(nf_swapped);
                  refnode:=nil;
                end
              else
                begin
                  if (refnode=left) then
                    toggleflag(nf_swapped);
                end;
            end;
          2: { fpu operands are always in the wrong order on the stack }
            toggleflag(nf_swapped);
        else
          InternalError(2013090802);
        end;
      end;


    procedure tx86addnode.emit_op_right_left(op:TAsmOp;opsize:TCgsize);
{$ifdef x86_64}
      var
        tmpreg : tregister;
{$endif x86_64}
      begin
        if (right.location.loc in [LOC_CSUBSETREG,LOC_SUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
        { left must be a register }
        case right.location.loc of
          LOC_REGISTER,
          LOC_CREGISTER :
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,TCGSize2Opsize[opsize],right.location.register,left.location.register));
          LOC_REFERENCE,
          LOC_CREFERENCE :
            begin
              tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,right.location.reference);
              current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,TCGSize2Opsize[opsize],right.location.reference,left.location.register));
            end;
          LOC_CONSTANT :
            begin
{$ifdef x86_64}
              { x86_64 only supports signed 32 bits constants directly }
              if (opsize in [OS_S64,OS_64]) and
                 ((right.location.value<low(longint)) or (right.location.value>high(longint))) then
                begin
                  tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                  cg.a_load_const_reg(current_asmdata.CurrAsmList,opsize,right.location.value,tmpreg);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,TCGSize2Opsize[opsize],tmpreg,left.location.register));
                end
              else
{$endif x86_64}
                current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(op,TCGSize2Opsize[opsize],right.location.value,left.location.register));
            end;
          else
            internalerror(200203232);
        end;
      end;


    function tx86addnode.getresflags(unsigned : boolean) : tresflags;
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
                       internalerror(2013120105);
                  end
                else
                  case nodetype of
                     ltn : getresflags:=F_L;
                     lten : getresflags:=F_LE;
                     gtn : getresflags:=F_G;
                     gten : getresflags:=F_GE;
                     else
                       internalerror(2013120106);
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
                       internalerror(2013120107);
                  end
                else
                  case nodetype of
                     ltn : getresflags:=F_B;
                     lten : getresflags:=F_BE;
                     gtn : getresflags:=F_A;
                     gten : getresflags:=F_AE;
                     else
                       internalerror(2013120108);
                  end;
             end;
         end;
      end;


    function tx86addnode.getfpuresflags : tresflags;
      begin
        if (nodetype=equaln) then
          result:=F_FE
        else if (nodetype=unequaln) then
          result:=F_FNE
        else if (nf_swapped in flags) then
          case nodetype of
            ltn : result:=F_FA;
            lten : result:=F_FAE;
            gtn : result:=F_FB;
            gten : result:=F_FBE;
          else
            internalerror(2014031402);
          end
        else
          case nodetype of
            ltn : result:=F_FB;
            lten : result:=F_FBE;
            gtn : result:=F_FA;
            gten : result:=F_FAE;
          else
            internalerror(2014031403);
          end;
      end;

{*****************************************************************************
                                AddSmallSet
*****************************************************************************}

{$ifndef i8086}
    procedure tx86addnode.second_addsmallset;
      var
        setbase : aint;
        opdef  : tdef;
        opsize : TCGSize;
        op     : TAsmOp;
        extra_not,
        noswap : boolean;
        all_member_optimization:boolean;

      begin
        pass_left_right;

        noswap:=false;
        extra_not:=false;
        all_member_optimization:=false;
        opdef:=resultdef;
        opsize:=int_cgsize(opdef.size);
        if (left.resultdef.typ=setdef) then
          setbase:=tsetdef(left.resultdef).setbase
        else
          setbase:=tsetdef(right.resultdef).setbase;
        case nodetype of
          addn :
            begin
              { adding elements is not commutative }
              if (nf_swapped in flags) and (left.nodetype=setelementn) then
               swapleftright;
              { are we adding set elements ? }
              if right.nodetype=setelementn then
               begin
                 { no range support for smallsets! }
                 if assigned(tsetelementnode(right).right) then
                   internalerror(43244);
                 { btsb isn't supported }
                 if opsize=OS_8 then
                   begin
                     opsize:=OS_32;
                     opdef:=u32inttype;
                   end;
                 { bts requires both elements to be registers }
                 hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
                 hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,opdef,true);
                 register_maybe_adjust_setbase(current_asmdata.CurrAsmList,opdef,right.location,setbase);
                 op:=A_BTS;
                 noswap:=true;
               end
              else
               op:=A_OR;
            end;
          symdifn :
            op:=A_XOR;
          muln :
            op:=A_AND;
          subn :
            begin
              op:=A_AND;
              if (not(nf_swapped in flags) and (left.location.loc=LOC_CONSTANT) and (left.location.value=-1)) or
                  ((nf_swapped in flags) and (right.location.loc=LOC_CONSTANT) and (right.location.value=-1)) then
                all_member_optimization:=true;

              if (not(nf_swapped in flags)) and
                 (right.location.loc=LOC_CONSTANT) then
                right.location.value := not(right.location.value)
              else if (nf_swapped in flags) and
                      (left.location.loc=LOC_CONSTANT) then
                left.location.value := not(left.location.value)
              else
                extra_not:=true;
            end;
          xorn :
            op:=A_XOR;
          orn :
            op:=A_OR;
          andn :
            op:=A_AND;
          else
            internalerror(2003042215);
        end;
        if all_member_optimization then
          begin
            {A set expression [0..31]-x can be implemented with a simple NOT.}
            if nf_swapped in flags then
              begin
                { newly swapped also set swapped flag }
                location_swap(left.location,right.location);
                toggleflag(nf_swapped);
              end;
            hlcg.location_force_reg(current_asmdata.currAsmList,right.location,right.resultdef,opdef,false);
            emit_reg(A_NOT,TCGSize2Opsize[opsize],right.location.register);
            location:=right.location;
          end
        else
          begin
            { can we use the BMI1 instruction andn? }
            if (op=A_AND) and extra_not and (CPUX86_HAS_BMI1 in cpu_capabilities[current_settings.cputype]) and
              (resultdef.size in [4{$ifdef x86_64},8{$endif x86_64}]) then
              begin
                location_reset(location,LOC_REGISTER,left.location.size);
                location.register:=cg.getintregister(current_asmdata.currAsmList,left.location.size);
                if nf_swapped in flags then
                  begin
                    location_swap(left.location,right.location);
                    toggleflag(nf_swapped);
                  end;
                hlcg.location_force_reg(current_asmdata.currAsmList,right.location,right.resultdef,opdef,true);
                if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER,LOC_CREFERENCE,LOC_REFERENCE]) then
                  hlcg.location_force_reg(current_asmdata.currAsmList,left.location,left.resultdef,opdef,true);
                case left.location.loc of
                  LOC_CREGISTER,LOC_REGISTER:
                    emit_reg_reg_reg(A_ANDN,TCGSize2Opsize[opsize],left.location.register,right.location.register,location.register);
                  LOC_CREFERENCE,LOC_REFERENCE:
                    emit_ref_reg_reg(A_ANDN,TCGSize2Opsize[opsize],left.location.reference,right.location.register,location.register);
                  else
                    Internalerror(2018040201);
                end;
              end
            else
              begin
                { left must be a register }
                left_must_be_reg(opdef,opsize,noswap);
                emit_generic_code(op,opsize,true,extra_not,false);
                location_freetemp(current_asmdata.CurrAsmList,right.location);

                { left is always a register and contains the result }
                location:=left.location;
              end;
          end;

        { fix the changed opsize we did above because of the missing btsb }
        if opsize<>int_cgsize(resultdef.size) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,location,opdef,cgsize_orddef(int_cgsize(resultdef.size)),false);
      end;
{$endif not i8086}


    procedure tx86addnode.second_cmpsmallset;
      var
        opdef  : tdef;
        opsize : TCGSize;
        op     : TAsmOp;
      begin
        pass_left_right;
        opdef:=left.resultdef;
        opsize:=int_cgsize(opdef.size);
        case nodetype of
          equaln,
          unequaln :
            op:=A_CMP;
          lten,gten:
            begin
              if (not(nf_swapped in flags) and (nodetype = lten)) or
                 ((nf_swapped in flags) and (nodetype = gten)) then
                swapleftright;
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,false);
              emit_op_right_left(A_AND,opsize);
              op:=A_CMP;
              { warning: ugly hack, we need a JE so change the node to equaln }
              nodetype:=equaln;
            end;
          else
            internalerror(2003042215);
        end;
        { left must be a register }
        left_must_be_reg(opdef,opsize,false);
        emit_generic_code(op,opsize,true,false,false);
        location_freetemp(current_asmdata.CurrAsmList,right.location);
        location_freetemp(current_asmdata.CurrAsmList,left.location);

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getresflags(true);
      end;


{*****************************************************************************
                                AddMMX
*****************************************************************************}

{$ifdef SUPPORT_MMX}
    procedure tx86addnode.second_opmmx;
      var
        op         : TAsmOp;
        cmpop      : boolean;
        mmxbase    : tmmxtype;
        hreg,
        hregister  : tregister;
      begin
        pass_left_right;

        cmpop:=false;
        op:=A_NOP;

        mmxbase:=mmx_type(left.resultdef);
        location_reset(location,LOC_MMXREGISTER,def_cgsize(resultdef));
        case nodetype of
          addn :
            begin
              if (cs_mmx_saturation in current_settings.localswitches) then
                begin
                   case mmxbase of
                      mmxs8bit:
                        op:=A_PADDSB;
                      mmxu8bit:
                        op:=A_PADDUSB;
                      mmxs16bit,mmxfixed16:
                        op:=A_PADDSW;
                      mmxu16bit:
                        op:=A_PADDUSW;
                      else
                       ;
                   end;
                end
              else
                begin
                   case mmxbase of
                      mmxs8bit,mmxu8bit:
                        op:=A_PADDB;
                      mmxs16bit,mmxu16bit,mmxfixed16:
                        op:=A_PADDW;
                      mmxs32bit,mmxu32bit:
                        op:=A_PADDD;
                      else
                       ;
                   end;
                end;
            end;
          muln :
            begin
               case mmxbase of
                  mmxs16bit,mmxu16bit:
                    op:=A_PMULLW;
                  mmxfixed16:
                    op:=A_PMULHW;
                  else
                   ;
               end;
            end;
          subn :
            begin
              if (cs_mmx_saturation in current_settings.localswitches) then
                begin
                   case mmxbase of
                      mmxs8bit:
                        op:=A_PSUBSB;
                      mmxu8bit:
                        op:=A_PSUBUSB;
                      mmxs16bit,mmxfixed16:
                        op:=A_PSUBSB;
                      mmxu16bit:
                        op:=A_PSUBUSW;
                      else
                       ;
                   end;
                end
              else
                begin
                   case mmxbase of
                      mmxs8bit,mmxu8bit:
                        op:=A_PSUBB;
                      mmxs16bit,mmxu16bit,mmxfixed16:
                        op:=A_PSUBW;
                      mmxs32bit,mmxu32bit:
                        op:=A_PSUBD;
                      else
                       ;
                   end;
                end;
            end;
          xorn:
            op:=A_PXOR;
          orn:
            op:=A_POR;
          andn:
            op:=A_PAND;
          else
            internalerror(2003042214);
        end;

        if op = A_NOP then
          internalerror(201408201);

        { left and right no register?  }
        { then one must be demanded    }
        if (left.location.loc<>LOC_MMXREGISTER) then
         begin
           if (right.location.loc=LOC_MMXREGISTER) then
            begin
              location_swap(left.location,right.location);
              toggleflag(nf_swapped);
            end
           else
            begin
              { register variable ? }
              if (left.location.loc=LOC_CMMXREGISTER) then
               begin
                 hregister:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
                 emit_reg_reg(A_MOVQ,S_NO,left.location.register,hregister);
               end
              else
               begin
                 if not(left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                  internalerror(200203245);

                 hregister:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
                 tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
                 emit_ref_reg(A_MOVQ,S_NO,left.location.reference,hregister);
               end;

              location_reset(left.location,LOC_MMXREGISTER,OS_NO);
              left.location.register:=hregister;
            end;
         end;

        { at this point, left.location.loc should be LOC_MMXREGISTER }
        if right.location.loc<>LOC_MMXREGISTER then
         begin
           if (nodetype=subn) and (nf_swapped in flags) then
            begin
              hreg:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
              if right.location.loc=LOC_CMMXREGISTER then
               begin
                 emit_reg_reg(A_MOVQ,S_NO,right.location.register,hreg);
                 emit_reg_reg(op,S_NO,left.location.register,hreg);
               end
              else
               begin
                 if not(left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                  internalerror(200203247);
                 tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,right.location.reference);
                 emit_ref_reg(A_MOVQ,S_NO,right.location.reference,hreg);
                 emit_reg_reg(op,S_NO,left.location.register,hreg);
               end;
               location.register:=hreg;
            end
           else
            begin
              if (right.location.loc=LOC_CMMXREGISTER) then
                emit_reg_reg(op,S_NO,right.location.register,left.location.register)
              else
               begin
                 if not(right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                  internalerror(200203246);
                 tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,right.location.reference);
                 emit_ref_reg(op,S_NO,right.location.reference,left.location.register);
               end;
              location.register:=left.location.register;
            end;
          end
        else
          begin
            { right.location=LOC_MMXREGISTER }
            if (nodetype=subn) and (nf_swapped in flags) then
             begin
               emit_reg_reg(op,S_NO,left.location.register,right.location.register);
               location_swap(left.location,right.location);
               toggleflag(nf_swapped);
             end
            else
             begin
               emit_reg_reg(op,S_NO,right.location.register,left.location.register);
             end;
            location.register:=left.location.register;
          end;

        location_freetemp(current_asmdata.CurrAsmList,right.location);
        if cmpop then
          location_freetemp(current_asmdata.CurrAsmList,left.location);
      end;
{$endif SUPPORT_MMX}


{*****************************************************************************
                                AddFloat
*****************************************************************************}

    procedure tx86addnode.second_addfloatsse;
      var
        op : topcg;
        sqr_sum : boolean;
        tmp : tnode;
      begin
        sqr_sum:=false;
        if (current_settings.fputype>=fpu_sse3) and
           use_vectorfpu(resultdef) and
           (nodetype in [addn,subn]) and
          (left.nodetype=inlinen) and (tinlinenode(left).inlinenumber=in_sqr_real) and
          (right.nodetype=inlinen) and (tinlinenode(right).inlinenumber=in_sqr_real) then
          begin
            sqr_sum:=true;
            tmp:=tinlinenode(left).left;
            tinlinenode(left).left:=nil;
            left.free;
            left:=tmp;

            tmp:=tinlinenode(right).left;
            tinlinenode(right).left:=nil;
            right.free;
            right:=tmp;
          end;

        pass_left_right;
        { fpu operands are always in reversed order on the stack }
        if (left.location.loc=LOC_FPUREGISTER) and (right.location.loc=LOC_FPUREGISTER) then
          toggleflag(nf_swapped);

        if (nf_swapped in flags) then
          { can't use swapleftright if both are on the fpu stack, since then }
          { both are "R_ST" -> nothing would change -> manually switch       }
          if (left.location.loc = LOC_FPUREGISTER) and
             (right.location.loc = LOC_FPUREGISTER) then
            emit_none(A_FXCH,S_NO)
          else
            swapleftright;

        case nodetype of
          addn :
            op:=OP_ADD;
          muln :
            op:=OP_MUL;
          subn :
            op:=OP_SUB;
          slashn :
            op:=OP_DIV;
          else
            internalerror(200312231);
        end;

        location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));

        if sqr_sum then
          begin
            if nf_swapped in flags then
              swapleftright;

            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);
            location:=left.location;
            if is_double(resultdef) then
              begin
                current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_SHUFPD,S_NO,%00,right.location.register,location.register));
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_MULPD,S_NO,location.register,location.register));
                case nodetype of
                  addn:
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HADDPD,S_NO,location.register,location.register));
                  subn:
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HSUBPD,S_NO,location.register,location.register));
                  else
                    internalerror(201108162);
                end;
              end
            else
              begin
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_UNPCKLPS,S_NO,right.location.register,location.register));
                { ensure that bits 64..127 contain valid values }
                current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_SHUFPD,S_NO,%00,location.register,location.register));
                { the data is now in bits 0..32 and 64..95 }
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_MULPS,S_NO,location.register,location.register));
                case nodetype of
                  addn:
                    begin
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HADDPS,S_NO,location.register,location.register));
                    end;
                  subn:
                    begin
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HSUBPS,S_NO,location.register,location.register));
                    end;
                  else
                    internalerror(201108163);
                end;
              end
          end
        { we can use only right as left operand if the operation is commutative }
        else if (right.location.loc=LOC_MMREGISTER) and (op in [OP_ADD,OP_MUL]) then
          begin
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
            cg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,right.location.size,location.size,right.location.register,location.register,mms_movescalar);
            { force floating point reg. location to be written to memory,
              we don't force it to mm register because writing to memory
              allows probably shorter code because there is no direct fpu->mm register
              copy instruction
            }
            if left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
              hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
            cg.a_opmm_loc_reg(current_asmdata.CurrAsmList,op,location.size,left.location,location.register,mms_movescalar);
          end
        else
          begin
            if nf_swapped in flags then
              swapleftright;

            { force floating point reg. location to be written to memory,
              we don't force it to mm register because writing to memory
              allows probably shorter code because there is no direct fpu->mm register
              copy instruction
            }
            if left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
              hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);

            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
            cg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,location.size,left.location,location.register,mms_movescalar);

            { force floating point reg. location to be written to memory,
              we don't force it to mm register because writing to memory
              allows probably shorter code because there is no direct fpu->mm register
              copy instruction
            }
            if right.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
              hlcg.location_force_mem(current_asmdata.CurrAsmList,right.location,right.resultdef);

            cg.a_opmm_loc_reg(current_asmdata.CurrAsmList,op,location.size,right.location,location.register,mms_movescalar);
          end;
      end;


    procedure tx86addnode.second_addfloatavx;
      var
        op : topcg;
        sqr_sum : boolean;
        {$ifdef dummy}
        tmp : tnode;
        {$endif dummy}
      begin
        sqr_sum:=false;
{$ifdef dummy}
        if (current_settings.fputype>=fpu_sse3) and
           use_vectorfpu(resultdef) and
           (nodetype in [addn,subn]) and
          (left.nodetype=inlinen) and (tinlinenode(left).inlinenumber=in_sqr_real) and
          (right.nodetype=inlinen) and (tinlinenode(right).inlinenumber=in_sqr_real) then
          begin
            sqr_sum:=true;
            tmp:=tinlinenode(left).left;
            tinlinenode(left).left:=nil;
            left.free;
            left:=tmp;

            tmp:=tinlinenode(right).left;
            tinlinenode(right).left:=nil;
            right.free;
            right:=tmp;
          end;
{$endif dummy}

        pass_left_right;
        { fpu operands are always in reversed order on the stack }
        if (left.location.loc=LOC_FPUREGISTER) and (right.location.loc=LOC_FPUREGISTER) then
          toggleflag(nf_swapped);

        if (nf_swapped in flags) then
          { can't use swapleftright if both are on the fpu stack, since then }
          { both are "R_ST" -> nothing would change -> manually switch       }
          if (left.location.loc = LOC_FPUREGISTER) and
             (right.location.loc = LOC_FPUREGISTER) then
            emit_none(A_FXCH,S_NO)
          else
            swapleftright;

        case nodetype of
          addn :
            op:=OP_ADD;
          muln :
            op:=OP_MUL;
          subn :
            op:=OP_SUB;
          slashn :
            op:=OP_DIV;
          else
            internalerror(200312231);
        end;

        location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));

        if sqr_sum then
          begin
            if nf_swapped in flags then
              swapleftright;

            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);
            location:=left.location;
            if is_double(resultdef) then
              begin
                current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_SHUFPD,S_NO,%00,right.location.register,location.register));
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_MULPD,S_NO,location.register,location.register));
                case nodetype of
                  addn:
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HADDPD,S_NO,location.register,location.register));
                  subn:
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HSUBPD,S_NO,location.register,location.register));
                  else
                    internalerror(201108162);
                end;
              end
            else
              begin
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_UNPCKLPS,S_NO,right.location.register,location.register));
                { ensure that bits 64..127 contain valid values }
                current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(A_SHUFPD,S_NO,%00,location.register,location.register));
                { the data is now in bits 0..32 and 64..95 }
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_MULPS,S_NO,location.register,location.register));
                case nodetype of
                  addn:
                    begin
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HADDPS,S_NO,location.register,location.register));
                    end;
                  subn:
                    begin
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_HSUBPS,S_NO,location.register,location.register));
                    end;
                  else
                    internalerror(201108163);
                end;
              end
          end
        { left*2 ? }
        else if (nodetype=muln) and is_constrealnode(right) and is_number_float(trealconstnode(right).value_real) and (trealconstnode(right).value_real=2) then
          begin
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
            cg.a_opmm_reg_reg_reg(current_asmdata.CurrAsmList,OP_ADD,location.size,
              left.location.register,
              left.location.register,
              location.register,
              mms_movescalar);
          end
        { right*2 ? }
        else if (nodetype=muln) and is_constrealnode(left) and is_number_float(trealconstnode(left).value_real) and (trealconstnode(left).value_real=2) then
          begin
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,right.location.size);
            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);
            cg.a_opmm_reg_reg_reg(current_asmdata.CurrAsmList,OP_ADD,location.size,
              right.location.register,
              right.location.register,
              location.register,
              mms_movescalar);
          end
        { we can use only right as left operand if the operation is commutative }
        else if (right.location.loc in [LOC_MMREGISTER,LOC_CMMREGISTER]) and (op in [OP_ADD,OP_MUL]) then
          begin
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
            { force floating point reg. location to be written to memory,
              we don't force it to mm register because writing to memory
              allows probably shorter code because there is no direct fpu->mm register
              copy instruction
            }
            if left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
              hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
            cg.a_opmm_loc_reg_reg(current_asmdata.CurrAsmList,op,location.size,
              left.location,
              right.location.register,
              location.register,
              mms_movescalar);
          end
        else
          begin
            if (nf_swapped in flags) then
              swapleftright;

            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,left.location.size);
            { force floating point reg. location to be written to memory,
              we don't force it to mm register because writing to memory
              allows probably shorter code because there is no direct fpu->mm register
              copy instruction
            }
            if right.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
              hlcg.location_force_mem(current_asmdata.CurrAsmList,right.location,right.resultdef);
            cg.a_opmm_loc_reg_reg(current_asmdata.CurrAsmList,op,location.size,
              right.location,
              left.location.register,
              location.register,
              mms_movescalar);
          end;
      end;


    function tx86addnode.use_fma : boolean;
      begin
{$ifndef i8086}
        { test if the result stays in an xmm register, fiddeling with fpu registers and fma makes no sense }
        Result:=use_vectorfpu(resultdef) and
          ((cpu_capabilities[current_settings.cputype]*[CPUX86_HAS_FMA,CPUX86_HAS_FMA4])<>[]);
{$else i8086}
        Result:=inherited use_fma;
{$endif i8086}
      end;


    procedure tx86addnode.second_cmpfloatvector;
      var
        op : tasmop;
      const
        ops_single: array[boolean] of tasmop = (A_COMISS,A_VCOMISS);
        ops_double: array[boolean] of tasmop = (A_COMISD,A_VCOMISD);
      begin
        if is_single(left.resultdef) then
          op:=ops_single[UseAVX]
        else if is_double(left.resultdef) then
          op:=ops_double[UseAVX]
        else
          internalerror(200402222);
        pass_left_right;

        location_reset(location,LOC_FLAGS,OS_NO);

        { Direct move fpu->mm register is not possible, so force any fpu operands to
          memory (not to mm registers because one of the memory locations can be used
          directly in compare instruction, yielding shorter code) }
        if left.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
          hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
        if right.location.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
          hlcg.location_force_mem(current_asmdata.CurrAsmList,right.location,right.resultdef);

        if (right.location.loc in [LOC_MMREGISTER,LOC_CMMREGISTER]) then
          begin
            case left.location.loc of
              LOC_REFERENCE,LOC_CREFERENCE:
                begin
                  tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
                  current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,left.location.reference,right.location.register));
                end;
              LOC_MMREGISTER,LOC_CMMREGISTER:
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,left.location.register,right.location.register));
              else
                internalerror(200402221);
            end;
            toggleflag(nf_swapped);
          end
        else
          begin
            hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
            case right.location.loc of
              LOC_REFERENCE,LOC_CREFERENCE:
                begin
                  tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,right.location.reference);
                  current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,right.location.reference,left.location.register));
                end;
              LOC_MMREGISTER,LOC_CMMREGISTER:
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,right.location.register,left.location.register));
              else
                internalerror(200402223);
            end;
          end;
        location.resflags:=getfpuresflags;
        location_freetemp(current_asmdata.CurrAsmList,left.location);
        location_freetemp(current_asmdata.CurrAsmList,right.location);
      end;


    procedure tx86addnode.second_opvector;
      var
        op : topcg;
      begin
        pass_left_right;
        if (nf_swapped in flags) then
          swapleftright;

        case nodetype of
          addn :
            op:=OP_ADD;
          muln :
            op:=OP_MUL;
          subn :
            op:=OP_SUB;
          slashn :
            op:=OP_DIV;
          else
            internalerror(200610071);
        end;

        if fits_in_mm_register(left.resultdef) then
          begin
            location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
            { we can use only right as left operand if the operation is commutative }
            if (right.location.loc=LOC_MMREGISTER) and (op in [OP_ADD,OP_MUL]) then
              begin
                location.register:=right.location.register;
                cg.a_opmm_loc_reg(current_asmdata.CurrAsmList,op,tfloat2tcgsize[tfloatdef(left.resultdef).floattype],left.location,location.register,nil);
              end
            else
              begin
                location_force_mmreg(current_asmdata.CurrAsmList,left.location,false);
                location.register:=left.location.register;
                cg.a_opmm_loc_reg(current_asmdata.CurrAsmList,op,
                  tfloat2tcgsize[tfloatdef(tarraydef(left.resultdef).elementdef).floattype],right.location,location.register,nil);
              end;
          end
        else
          begin
            { not yet supported }
            internalerror(200610072);
          end
      end;


    procedure tx86addnode.second_addfloat;
      const
        ops_add:  array[boolean] of TAsmOp = (A_FADDP,A_FADD);
        ops_mul:  array[boolean] of TAsmOp = (A_FMULP,A_FMUL);
        ops_sub:  array[boolean] of TAsmOp = (A_FSUBP,A_FSUB);
        ops_rsub: array[boolean] of TAsmOp = (A_FSUBRP,A_FSUBR);
        ops_div:  array[boolean] of TAsmOp = (A_FDIVP,A_FDIV);
        ops_rdiv: array[boolean] of TAsmOp = (A_FDIVRP,A_FDIVR);
      var
        op : TAsmOp;
        refnode : tnode;
        hasref : boolean;
      begin
        if use_vectorfpu(resultdef) then
          begin
            if UseAVX then
              second_addfloatavx
            else
              second_addfloatsse;
            exit;
          end;

        pass_left_right;
        prepare_x87_locations(refnode);
        hasref:=assigned(refnode);

        case nodetype of
          addn :
            op:=ops_add[hasref];
          muln :
            op:=ops_mul[hasref];
          subn :
            if (nf_swapped in flags) then
              op:=ops_rsub[hasref]
            else
              op:=ops_sub[hasref];
          slashn :
            if (nf_swapped in flags) then
              op:=ops_rdiv[hasref]
            else
              op:=ops_div[hasref];
          else
            internalerror(2003042214);
        end;

        if hasref then
          emit_ref(op,tcgsize2opsize[refnode.location.size],refnode.location.reference)
        else
          begin
            emit_reg_reg(op,S_NO,NR_ST,NR_ST1);
            tcgx86(cg).dec_fpu_stack;
          end;

        location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
        location.register:=NR_ST;
      end;


    procedure tx86addnode.second_cmpfloat;
{$ifdef i8086}
      var
        tmpref: treference;
{$endif i8086}
      begin
        if use_vectorfpu(left.resultdef) or use_vectorfpu(right.resultdef) then
          begin
            second_cmpfloatvector;
            exit;
          end;

        pass_left_right;
        force_left_and_right_fpureg;

{$ifndef x86_64}
        if current_settings.cputype<cpu_Pentium2 then
          begin
            emit_none(A_FCOMPP,S_NO);
            tcgx86(cg).dec_fpu_stack;
            tcgx86(cg).dec_fpu_stack;

            { load fpu flags }
{$ifdef i8086}
            if current_settings.cputype < cpu_286 then
              begin
                tg.gettemp(current_asmdata.CurrAsmList,2,2,tt_normal,tmpref);
                emit_ref(A_FSTSW,S_NO,tmpref);
                cg.getcpuregister(current_asmdata.CurrAsmList,NR_AX);
                inc(tmpref.offset);
                emit_ref_reg(A_MOV,S_B,tmpref,NR_AH);
                dec(tmpref.offset);
                emit_none(A_SAHF,S_NO);
                cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_AX);
                tg.ungettemp(current_asmdata.CurrAsmList,tmpref);
              end
            else
{$endif i8086}
              begin
                cg.getcpuregister(current_asmdata.CurrAsmList,NR_AX);
                emit_reg(A_FNSTSW,S_NO,NR_AX);
                emit_none(A_SAHF,S_NO);
                cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_AX);
              end;
          end
        else
{$endif x86_64}
          begin
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FCOMIP,S_NO,NR_ST1,NR_ST0));
            { fcomip pops only one fpu register }
            current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_FSTP,S_NO,NR_ST0));
            tcgx86(cg).dec_fpu_stack;
            tcgx86(cg).dec_fpu_stack;
          end;

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getfpuresflags;
      end;


{*****************************************************************************
                                  Add64bit
*****************************************************************************}

    procedure tx86addnode.second_add64bit;
      begin
{$ifdef cpu64bitalu}
        second_addordinal;
{$else cpu64bitalu}
        { must be implemented separate }
        internalerror(200402042);
{$endif cpu64bitalu}
      end;


    procedure tx86addnode.second_cmp64bit;
      begin
{$ifdef cpu64bitalu}
        second_cmpordinal;
{$else cpu64bitalu}
        { must be implemented separate }
        internalerror(200402043);
{$endif cpu64bitalu}
      end;


{*****************************************************************************
                                  AddOrdinal
*****************************************************************************}

    procedure tx86addnode.second_addordinal;
      var
         opsize : tcgsize;
         unsigned : boolean;
         cgop : topcg;
         checkoverflow : Boolean;
         ovloc : tlocation;
         tmpreg : TRegister;
      begin
        { determine if the comparison will be unsigned }
        unsigned:=not(is_signed(left.resultdef)) or
                    not(is_signed(right.resultdef));

        { assume no overflow checking is require }
        checkoverflow := false;

        ovloc.loc:=LOC_VOID;

        case nodetype of
          addn:
            begin
              cgop:=OP_ADD;
              checkoverflow:=true;
            end;
          xorn :
            begin
              cgop:=OP_XOR;
            end;
          orn :
            begin
              cgop:=OP_OR;
            end;
          andn:
            begin
              cgop:=OP_AND;
            end;
          muln:
            begin
              checkoverflow:=true;
              if unsigned then
                cgop:=OP_MUL
              else
                cgop:=OP_IMUL;
            end;
          subn :
            begin
              checkoverflow:=true;
              cgop:=OP_SUB;
            end;
          else
            internalerror(2015022501);
        end;

       checkoverflow:=
         checkoverflow and
         needoverflowcheck;

       opsize:=def_cgsize(left.resultdef);

       pass_left_right;

       { do have to allocate a register? If yes, then three opcode instructions are better, however for sub three op code instructions
         make no sense if right is a reference }
       if ((left.location.loc<>LOC_REGISTER) and (right.location.loc<>LOC_REGISTER) and ((nodetype<>subn) or not(right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]))) or
         ((nodetype=addn) and (left.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_CONSTANT]) and (right.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_CONSTANT])) then
         begin
           { allocate registers }
           force_reg_left_right(false,true);
           set_result_location_reg;
           if nodetype<>subn then
            begin
              if (right.location.loc<>LOC_CONSTANT) then
                hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,cgop,resultdef,
                   left.location.register,right.location.register,
                   location.register,checkoverflow,ovloc)
              else
                hlcg.a_op_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,cgop,resultdef,
                   right.location.value,left.location.register,
                   location.register,checkoverflow,ovloc);
            end
          else  { subtract is a special case since its not commutative }
            begin
              if (nf_swapped in flags) then
                swapleftright;
              if left.location.loc<>LOC_CONSTANT then
                begin
                  if right.location.loc<>LOC_CONSTANT then
                    hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                        right.location.register,left.location.register,
                        location.register,checkoverflow,ovloc)
                  else
                    hlcg.a_op_const_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                      right.location.value,left.location.register,
                      location.register,checkoverflow,ovloc);
                end
              else
                begin
                  tmpreg:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
                  hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,
                    left.location.value,tmpreg);
                  hlcg.a_op_reg_reg_reg_checkoverflow(current_asmdata.CurrAsmList,OP_SUB,resultdef,
                    right.location.register,tmpreg,location.register,checkoverflow,ovloc);
                end;
            end
         end
       else
         begin
           { at least one location should be a register, if yes, try to re-use it, so we can try two operand opcodes }
           if left.location.loc<>LOC_REGISTER then
              begin
                if right.location.loc<>LOC_REGISTER then
                  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false)
                else
                  begin
                    location_swap(left.location,right.location);
                    toggleflag(nf_swapped);
                  end;
              end;

           { at this point, left.location.loc should be LOC_REGISTER }
           if right.location.loc=LOC_REGISTER then
             begin
               { when swapped another result register }
               if (nodetype=subn) and (nf_swapped in flags) then
                 begin
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,cgop,opsize,
                     left.location.register,right.location.register);
                   location_swap(left.location,right.location);
                   toggleflag(nf_swapped);
                 end
               else
                 cg.a_op_reg_reg(current_asmdata.CurrAsmList,cgop,opsize,
                    right.location.register,left.location.register);
             end
           else
             begin
               { right.location<>LOC_REGISTER }
               if right.location.loc in [LOC_CSUBSETREF,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_SUBSETREG] then
                 hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,left.resultdef,true);
               if (nodetype=subn) and (nf_swapped in flags) then
                 begin
                   tmpreg:=left.location.register;
                   left.location.register:=cg.getintregister(current_asmdata.CurrAsmList,opsize);
                   cg.a_load_loc_reg(current_asmdata.CurrAsmList,opsize,right.location,left.location.register);
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,cgop,opsize,tmpreg,left.location.register);
                 end
               else
                 cg.a_op_loc_reg(current_asmdata.CurrAsmList,cgop,opsize,right.location,left.location.register);
               location_freetemp(current_asmdata.CurrAsmList,right.location);
             end;

           location_copy(location,left.location);
         end;

       { emit overflow check if required }
       if checkoverflow then
         cg.g_overflowcheck_loc(current_asmdata.CurrAsmList,Location,resultdef,ovloc);
     end;


    procedure tx86addnode.second_cmpordinal;
      var
         opdef  : tdef;
         opsize : tcgsize;
         unsigned : boolean;
      begin
         unsigned:=not(is_signed(left.resultdef)) or
                   not(is_signed(right.resultdef));
         opdef:=left.resultdef;
         opsize:=def_cgsize(opdef);

         pass_left_right;

         if (right.location.loc=LOC_CONSTANT) and
            (left.location.loc in [LOC_REFERENCE, LOC_CREFERENCE])
{$ifdef x86_64}
              and ((not (opsize in [OS_64,OS_S64])) or (
              (right.location.value>=low(longint)) and (right.location.value<=high(longint))
            ))
{$endif x86_64}
         then
           begin
             emit_const_ref(A_CMP, TCGSize2Opsize[opsize], right.location.value, left.location.reference);
             location_freetemp(current_asmdata.CurrAsmList,left.location);
           end
         else
           begin
             left_must_be_reg(opdef,opsize,false);
             emit_generic_code(A_CMP,opsize,unsigned,false,false);
             location_freetemp(current_asmdata.CurrAsmList,right.location);
             location_freetemp(current_asmdata.CurrAsmList,left.location);
           end;
         location_reset(location,LOC_FLAGS,OS_NO);
         location.resflags:=getresflags(unsigned);
      end;

begin
   caddnode:=tx86addnode;
end.
