{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Code generation for add nodes on the ARM

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
unit narmadd;

{$i fpcdefs.inc}

interface

    uses
       node,ncgadd,cpubase;

    type
       tarmaddnode = class(tcgaddnode)
       private
          function  GetResFlags(unsigned:Boolean):TResFlags;
          function  GetFpuResFlags:TResFlags;
       public
          function use_fma : boolean;override;
          function pass_1 : tnode;override;
          function use_generic_mul32to64: boolean; override;
          function use_generic_mul64bit: boolean; override;
       protected
          function first_addfloat: tnode; override;
          procedure second_addordinal;override;
          procedure second_addfloat;override;
          procedure second_cmpfloat;override;
          procedure second_cmpordinal;override;
          procedure second_cmpsmallset;override;
          procedure second_cmp64bit;override;
          procedure second_add64bit;override;
       end;

  implementation

    uses
      globtype,verbose,globals,systems,
      constexp,symdef,symtable,symtype,symconst,
      aasmbase,aasmdata,aasmcpu,
      defutil,htypechk,cgbase,cgutils,
      cpuinfo,pass_1,pass_2,procinfo,
      ncon,nadd,ncnv,ncal,nmat,
      ncgutil,cgobj,cgcpu,
      hlcgobj
      ;

{*****************************************************************************
                               TSparcAddNode
*****************************************************************************}

    function tarmaddnode.GetResFlags(unsigned:Boolean):TResFlags;
      begin
        case NodeType of
          equaln:
            GetResFlags:=F_EQ;
          unequaln:
            GetResFlags:=F_NE;
          else
            if not(unsigned) then
              begin
                if nf_swapped in flags then
                  case NodeType of
                    ltn:
                      GetResFlags:=F_GT;
                    lten:
                      GetResFlags:=F_GE;
                    gtn:
                      GetResFlags:=F_LT;
                    gten:
                      GetResFlags:=F_LE;
                    else
                      internalerror(201408203);
                  end
                else
                  case NodeType of
                    ltn:
                      GetResFlags:=F_LT;
                    lten:
                      GetResFlags:=F_LE;
                    gtn:
                      GetResFlags:=F_GT;
                    gten:
                      GetResFlags:=F_GE;
                    else
                      internalerror(201408204);
                  end;
              end
            else
              begin
                if nf_swapped in Flags then
                  case NodeType of
                    ltn:
                      GetResFlags:=F_HI;
                    lten:
                      GetResFlags:=F_CS;
                    gtn:
                      GetResFlags:=F_CC;
                    gten:
                      GetResFlags:=F_LS;
                    else
                      internalerror(201408205);
                  end
                else
                  case NodeType of
                    ltn:
                      GetResFlags:=F_CC;
                    lten:
                      GetResFlags:=F_LS;
                    gtn:
                      GetResFlags:=F_HI;
                    gten:
                      GetResFlags:=F_CS;
                    else
                      internalerror(201408206);
                  end;
              end;
        end;
      end;


    function tarmaddnode.GetFpuResFlags:TResFlags;
      begin
        if nf_swapped in Flags then
          internalerror(2014042001);
        case NodeType of
          equaln:
            result:=F_EQ;
          unequaln:
            result:=F_NE;
          ltn:
            result:=F_MI;
          lten:
            result:=F_LS;
          gtn:
            result:=F_GT;
          gten:
            result:=F_GE;
          else
            internalerror(201408207);
        end;
      end;


    function tarmaddnode.use_fma : boolean;
      begin
       Result:=FPUARM_HAS_FMA in fpu_capabilities[current_settings.fputype];
      end;


    procedure tarmaddnode.second_addfloat;
      var
        op : TAsmOp;
        singleprec: boolean;
        pf: TOpPostfix;
      begin
        pass_left_right;
        if (nf_swapped in flags) then
          swapleftright;

        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            begin
              { force fpureg as location, left right doesn't matter
                as both will be in a fpureg }
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);

              case nodetype of
                addn :
                  op:=A_ADF;
                muln :
                  op:=A_MUF;
                subn :
                  op:=A_SUF;
                slashn :
                  op:=A_DVF;
                else
                  internalerror(200308313);
              end;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register),
                 cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          fpu_soft:
            { this case should be handled already by pass1 }
            internalerror(200308252);
          else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              singleprec:=tfloatdef(left.resultdef).floattype=s32real;
              if singleprec then
                pf:=PF_F32
              else
                pf:=PF_F64;
              case nodetype of
                addn :
                  op:=A_VADD;
                muln :
                  op:=A_VMUL;
                subn :
                  op:=A_VSUB;
                slashn :
                  op:=A_VDIV;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op,
                 location.register,left.location.register,right.location.register),pf));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
            begin
              { force mmreg as location, left right doesn't matter
                as both will be in a fpureg }
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
              location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

              case nodetype of
                addn :
                  op:=A_VADD;
                muln :
                  op:=A_VMUL;
                subn :
                  op:=A_VSUB;
                slashn :
                  op:=A_VDIV;
                else
                  internalerror(2009111401);
              end;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(op, location.register,left.location.register,right.location.register), PF_F32));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
            end
          else
            internalerror(200308251);
        end;
      end;


    procedure tarmaddnode.second_cmpfloat;
      var
        op: TAsmOp;
        pf: TOpPostfix;
      begin
        pass_left_right;
        if (nf_swapped in flags) then
          swapleftright;

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getresflags(false);

        case current_settings.fputype of
          fpu_fpa,
          fpu_fpa10,
          fpu_fpa11:
            begin
              { force fpureg as location, left right doesn't matter
                as both will be in a fpureg }
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              if nodetype in [equaln,unequaln] then
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMF,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]))
              else
                current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(A_CMFE,
                   left.location.register,right.location.register),
                   cgsize2fpuoppostfix[def_cgsize(resultdef)]));
            end;
          else if FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype] then
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              if nodetype in [equaln,unequaln] then
                op:=A_VCMP
              else
                op:=A_VCMPE;

              if (tfloatdef(left.resultdef).floattype=s32real) then
                pf:=PF_F32
              else
                pf:=PF_F64;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(op,
                left.location.register,right.location.register), pf));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_VMRS,NR_APSR_nzcv,NR_FPSCR));
              location.resflags:=GetFpuResFlags;
            end
          else if FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype] then
            begin
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,right.location,right.resultdef,true);

              if nodetype in [equaln,unequaln] then
                op:=A_VCMP
              else
                op:=A_VCMPE;

              current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg(op,
                left.location.register,right.location.register),PF_F32));
              cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg(A_VMRS, NR_APSR_nzcv, NR_FPSCR));
            end
          else
            { this case should be handled already by pass1 }
            internalerror(2009112404);
        end;
      end;


    procedure tarmaddnode.second_cmpsmallset;
      var
        tmpreg : tregister;
        b: byte;
      begin
        pass_left_right;

        location_reset(location,LOC_FLAGS,OS_NO);

        if (not(nf_swapped in flags) and
            (nodetype = lten)) or
           ((nf_swapped in flags) and
            (nodetype = gten)) then
          swapleftright;

        (* Try to keep right as a constant *)
        if (right.location.loc <> LOC_CONSTANT) or
          not(is_shifter_const(right.location.value, b)) or
          ((GenerateThumbCode) and not(is_thumb_imm(right.location.value))) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

        case nodetype of
          equaln,
          unequaln:
            begin
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              if right.location.loc = LOC_CONSTANT then
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,left.location.register,right.location.value))
              else
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register,right.location.register));
              if nodetype = equaln then
                location.resflags:=F_EQ
              else
                location.resflags:=F_NE;
            end;
          lten,
          gten:
            begin
              tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
              if right.location.loc = LOC_CONSTANT then
                begin
                  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,right.location.value,left.location.register,tmpreg);
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,tmpreg,right.location.value));
                end
              else
                begin
                  cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_AND,OS_32,left.location.register,right.location.register,tmpreg);
                  cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,tmpreg,right.location.register));
                end;
              location.resflags:=F_EQ;
            end;
          else
            internalerror(2004012401);
        end;
      end;


    procedure tarmaddnode.second_cmp64bit;
      var
        unsigned : boolean;
        oldnodetype : tnodetype;
        dummyreg : tregister;
        truelabel, falselabel: tasmlabel;
        l: tasmlabel;
      const
        lt_zero_swapped: array[boolean] of tnodetype = (ltn, gtn);
      begin
        truelabel:=nil;
        falselabel:=nil;
        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        pass_left_right;

        { pass_left_right moves possible consts to the right, the only
          remaining case with left consts (currency) can take this path too (KB) }
        if (right.nodetype=ordconstn) and
           (tordconstnode(right).value=0) and
           ((nodetype in [equaln,unequaln]) or
            (not(GenerateThumbCode) and is_signed(left.resultdef) and (nodetype = lt_zero_swapped[nf_swapped in Flags]))
           ) then
          begin
            location_reset(location,LOC_FLAGS,OS_NO);
            if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

            cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
            { Optimize for the common case of int64 < 0 }
            if nodetype in [ltn, gtn] then
              begin
                {Just check for the MSB in reghi to be set or not, this is independed from nf_swapped}
                location.resflags:=F_NE;
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_TST,left.location.register64.reghi, aint($80000000)));
              end
            else
              begin
                location.resflags:=getresflags(unsigned);
                dummyreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);

                if GenerateThumbCode then
                  cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.register64.reglo,left.location.register64.reghi,dummyreg)
                else
                  current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ORR,dummyreg,left.location.register64.reglo,left.location.register64.reghi),PF_S));
              end;
          end
        else
          begin
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

            { operation requiring proper N, Z and C flags ? }
            if unsigned or (nodetype in [equaln,unequaln]) then
              begin
                location_reset(location,LOC_FLAGS,OS_NO);
                location.resflags:=getresflags(unsigned);
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reghi,right.location.register64.reghi));
                if GenerateThumbCode or GenerateThumb2Code then
                  begin
                    current_asmdata.getjumplabel(l);
                    cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NE,l);
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo));
                    cg.a_label(current_asmdata.CurrAsmList,l);
                  end
                else
                  current_asmdata.CurrAsmList.concat(setcondition(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo),C_EQ));
              end
            else
            { operation requiring proper N, Z and V flags ? }
              begin
                current_asmdata.getjumplabel(truelabel);
                current_asmdata.getjumplabel(falselabel);
                location_reset_jump(location,truelabel,falselabel);
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reghi,right.location.register64.reghi));
                { the jump the sequence is a little bit hairy }
                case nodetype of
                   ltn,gtn:
                     begin
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(false),location.truelabel);
                        { cheat a little bit for the negative test }
                        toggleflag(nf_swapped);
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(false),location.falselabel);
                        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        toggleflag(nf_swapped);
                     end;
                   lten,gten:
                     begin
                        oldnodetype:=nodetype;
                        if nodetype=lten then
                          nodetype:=ltn
                        else
                          nodetype:=gtn;
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.truelabel);
                        { cheat for the negative test }
                        if nodetype=ltn then
                          nodetype:=gtn
                        else
                          nodetype:=ltn;
                        cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(unsigned),location.falselabel);
                        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        nodetype:=oldnodetype;
                     end;
                   else
                     ;
                end;
                cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register64.reglo,right.location.register64.reglo));
                { the comparisaion of the low dword have to be
                   always unsigned!                            }
                cg.a_jmp_flags(current_asmdata.CurrAsmList,getresflags(true),location.truelabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,location.falselabel);
                cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              end;
          end;
      end;

    procedure tarmaddnode.second_add64bit;
      var
        asmList : TAsmList;
        ll,rl,res : TRegister64;
        tmpreg: TRegister;
      begin
        if (nodetype in [muln]) then
          begin
            asmList := current_asmdata.CurrAsmList;
            pass_left_right;
            force_reg_left_right(true, (left.location.loc<>LOC_CONSTANT) and (right.location.loc<>LOC_CONSTANT));
            set_result_location_reg;

            { shortcuts to register64s }
            ll:=left.location.register64;
            rl:=right.location.register64;
            res:=location.register64;

            tmpreg := cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            asmList.concat(taicpu.op_reg_reg_reg(A_MUL,tmpreg,ll.reglo,rl.reghi));
            asmList.concat(taicpu.op_reg_reg_reg_reg(A_UMULL,res.reglo,res.reghi,rl.reglo,ll.reglo));
            tbasecgarm(cg).safe_mla(asmList,tmpreg,rl.reglo,ll.reghi,tmpreg);
            asmList.concat(taicpu.op_reg_reg_reg(A_ADD,res.reghi,tmpreg,res.reghi));
          end
        else
          inherited second_add64bit;
      end;

    function tarmaddnode.pass_1 : tnode;
      var
        unsigned : boolean;
      begin
        result:=inherited pass_1;

        if not(assigned(result)) then
          begin
            unsigned:=not(is_signed(left.resultdef)) or
              not(is_signed(right.resultdef));

            if is_64bit(left.resultdef) and
              ((nodetype in [equaln,unequaln]) or
               (unsigned and (nodetype in [ltn,lten,gtn,gten]))
              ) then
              expectloc:=LOC_FLAGS;
          end;
      end;


    function tarmaddnode.first_addfloat: tnode;
      begin
        result := nil;

        if (FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[current_settings.fputype]) and
           not(FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[current_settings.fputype]) then
          begin
            case tfloatdef(left.resultdef).floattype of
              s32real:
                ;
              s64real:
                result:=first_addfloat_soft;
              else
                internalerror(2019050933);
            end;
          end
        else
          result:=inherited first_addfloat;
      end;


    procedure tarmaddnode.second_cmpordinal;
      var
        unsigned : boolean;
        tmpreg : tregister;
        b : byte;
      begin
        pass_left_right;
        force_reg_left_right(true,true);

        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));
        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        if right.location.loc = LOC_CONSTANT then
          begin
             if (not(GenerateThumbCode) and is_shifter_const(right.location.value,b)) or
                ((GenerateThumbCode) and is_thumb_imm(right.location.value)) then
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMP,left.location.register,right.location.value))
             else
               begin
                 tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
                 cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,
                   right.location.value,tmpreg);
                 current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register,tmpreg));
               end;
          end
        else
          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CMP,left.location.register,right.location.register));

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getresflags(unsigned);
      end;

    const
      multops: array[boolean] of TAsmOp = (A_SMULL, A_UMULL);

    procedure tarmaddnode.second_addordinal;
      var
        unsigned: boolean;
      begin
        if (nodetype=muln) and
           is_64bit(resultdef) and
           not(GenerateThumbCode) and
           (CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]) then
          begin
            pass_left_right;
            force_reg_left_right(true, false);
            set_result_location_reg;
            unsigned:=not(is_signed(left.resultdef)) or
                      not(is_signed(right.resultdef));
            current_asmdata.CurrAsmList.Concat(
              taicpu.op_reg_reg_reg_reg(multops[unsigned], location.register64.reglo, location.register64.reghi,
                                        left.location.register,right.location.register));
          end
        else
          inherited second_addordinal;
      end;

    function tarmaddnode.use_generic_mul32to64: boolean;
      begin
        result:=GenerateThumbCode or not(CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]);
      end;

    function tarmaddnode.use_generic_mul64bit: boolean;
      begin
        result:=GenerateThumbCode or
          not(CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]) or
          needoverflowcheck;
      end;

begin
  caddnode:=tarmaddnode;
end.
