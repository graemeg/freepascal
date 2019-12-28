{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Code generation for add nodes on the i386

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
unit n386add;

{$i fpcdefs.inc}

interface

    uses
       node,nadd,cpubase,nx86add;

    type
       ti386addnode = class(tx86addnode)
         function use_generic_mul32to64: boolean; override;
         function use_generic_mul64bit: boolean; override;
         procedure second_addordinal; override;
         procedure second_add64bit;override;
         procedure second_cmp64bit;override;
         procedure second_mul(unsigned: boolean);
         procedure second_mul64bit;
       protected
         procedure set_mul_result_location;
       end;

  implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,defutil,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      cgbase,procinfo,
      ncon,nset,cgutils,tgobj,
      cpuinfo,
      cga,ncgutil,cgobj,cg64f32,cgx86,
      hlcgobj;

{*****************************************************************************
                                use_generic_mul32to64
*****************************************************************************}

    function ti386addnode.use_generic_mul32to64: boolean;
    begin
      result := False;
    end;

    function ti386addnode.use_generic_mul64bit: boolean;
    begin
      result:=needoverflowcheck or
        (cs_opt_size in current_settings.optimizerswitches);
    end;

    { handles all unsigned multiplications, and 32->64 bit signed ones.
      32bit-only signed mul is handled by generic codegen }
    procedure ti386addnode.second_addordinal;
    var
      unsigned: boolean;
    begin
      unsigned:=not(is_signed(left.resultdef)) or
                not(is_signed(right.resultdef));
      { use IMUL instead of MUL in case overflow checking is off and we're
        doing a 32->32-bit multiplication }
      if not needoverflowcheck and
         not is_64bit(resultdef) then
        unsigned:=false;
      if (nodetype=muln) and (unsigned or is_64bit(resultdef)) then
        second_mul(unsigned)
      else
        inherited second_addordinal;
    end;

{*****************************************************************************
                                Add64bit
*****************************************************************************}

    procedure ti386addnode.second_add64bit;
      var
        op         : TOpCG;
        op1,op2    : TAsmOp;
        opsize     : TOpSize;
        hregister,
        hregister2 : tregister;
        hl4        : tasmlabel;
        mboverflow,
        unsigned:boolean;
        r:Tregister;
      begin
        pass_left_right;

        op1:=A_NONE;
        op2:=A_NONE;
        mboverflow:=false;
        opsize:=S_L;
        unsigned:=((left.resultdef.typ=orddef) and
                   (torddef(left.resultdef).ordtype=u64bit)) or
                  ((right.resultdef.typ=orddef) and
                   (torddef(right.resultdef).ordtype=u64bit));
        case nodetype of
          addn :
            begin
              op:=OP_ADD;
              mboverflow:=true;
            end;
          subn :
            begin
              op:=OP_SUB;
              op1:=A_SUB;
              op2:=A_SBB;
              mboverflow:=true;
            end;
          xorn:
            op:=OP_XOR;
          orn:
            op:=OP_OR;
          andn:
            op:=OP_AND;
          muln:
            begin
              second_mul64bit;
              exit;
            end
          else
            begin
              { everything should be handled in pass_1 (JM) }
              internalerror(200109051);
            end;
        end;

        { left and right no register?  }
        { then one must be demanded    }
        if (left.location.loc<>LOC_REGISTER) then
         begin
           if (right.location.loc<>LOC_REGISTER) then
            begin
              hregister:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
              hregister2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
              cg64.a_load64_loc_reg(current_asmdata.CurrAsmList,left.location,joinreg64(hregister,hregister2));
              location_reset(left.location,LOC_REGISTER,left.location.size);
              left.location.register64.reglo:=hregister;
              left.location.register64.reghi:=hregister2;
            end
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
              cg64.a_op64_reg_reg(current_asmdata.CurrAsmList,op,location.size,
                left.location.register64,
                right.location.register64);
              location_swap(left.location,right.location);
              toggleflag(nf_swapped);
            end
           else
            begin
              cg64.a_op64_reg_reg(current_asmdata.CurrAsmList,op,location.size,
                right.location.register64,
                left.location.register64);
            end;
         end
        else
         begin
           { right.location<>LOC_REGISTER }
           if (nodetype=subn) and (nf_swapped in flags) then
            begin
              r:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
              cg64.a_load64low_loc_reg(current_asmdata.CurrAsmList,right.location,r);
              cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              emit_reg_reg(op1,opsize,left.location.register64.reglo,r);
              emit_reg_reg(A_MOV,opsize,r,left.location.register64.reglo);
              cg64.a_load64high_loc_reg(current_asmdata.CurrAsmList,right.location,r);
              { the carry flag is still ok }
              emit_reg_reg(op2,opsize,left.location.register64.reghi,r);
              cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
              emit_reg_reg(A_MOV,opsize,r,left.location.register64.reghi);
            end
           else
            begin
              cg64.a_op64_loc_reg(current_asmdata.CurrAsmList,op,location.size,right.location,
                left.location.register64);
            end;
          location_freetemp(current_asmdata.CurrAsmList,right.location);
         end;

        { only in case of overflow operations }
        { produce overflow code }
        { we must put it here directly, because sign of operation }
        { is in unsigned VAR!!                              }
        if mboverflow then
         begin
           if needoverflowcheck then
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

        location_copy(location,left.location);
      end;


    procedure ti386addnode.second_cmp64bit;
      var
        truelabel,
        falselabel,
        hlab       : tasmlabel;
        href       : treference;
        unsigned   : boolean;

      procedure firstjmp64bitcmp;

        var
           oldnodetype : tnodetype;

        begin
{$ifdef OLDREGVARS}
           load_all_regvars(current_asmdata.CurrAsmList);
{$endif OLDREGVARS}
           { the jump the sequence is a little bit hairy }
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
                internalerror(2019050905);
           end;
        end;

      procedure secondjmp64bitcmp;

        begin
           { the jump the sequence is a little bit hairy }
           case nodetype of
              ltn,gtn,lten,gten:
                begin
                   { the comparisaion of the low dword have to be }
                   {  always unsigned!                            }
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
                internalerror(2019050904);
           end;
        end;

      begin
        truelabel:=nil;
        falselabel:=nil;
        pass_left_right;

        unsigned:=((left.resultdef.typ=orddef) and
                   (torddef(left.resultdef).ordtype=u64bit)) or
                  ((right.resultdef.typ=orddef) and
                   (torddef(right.resultdef).ordtype=u64bit));

        { we have LOC_JUMP as result }
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
              F_AE: hlab:=location.truelabel ;
              F_B:  hlab:=location.falselabel;
              else
                ;
            end;
          end;

        if (right.location.loc=LOC_CONSTANT) and
           (left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
          begin
            tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
            href:=left.location.reference;
            inc(href.offset,4);
            emit_const_ref(A_CMP,S_L,aint(hi(right.location.value64)),href);
            firstjmp64bitcmp;
            if assigned(hlab) then
              cg.a_jmp_always(current_asmdata.CurrAsmList,hlab)
            else
              begin
                emit_const_ref(A_CMP,S_L,aint(lo(right.location.value64)),left.location.reference);
                secondjmp64bitcmp;
              end;
            location_freetemp(current_asmdata.CurrAsmList,left.location);
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

        { at this point, left.location.loc should be LOC_[C]REGISTER }
        case right.location.loc of
          LOC_REGISTER,
          LOC_CREGISTER :
            begin
              emit_reg_reg(A_CMP,S_L,right.location.register64.reghi,left.location.register64.reghi);
              firstjmp64bitcmp;
              emit_reg_reg(A_CMP,S_L,right.location.register64.reglo,left.location.register64.reglo);
              secondjmp64bitcmp;
            end;
          LOC_CREFERENCE,
          LOC_REFERENCE :
            begin
              tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,right.location.reference);
              href:=right.location.reference;
              inc(href.offset,4);
              emit_ref_reg(A_CMP,S_L,href,left.location.register64.reghi);
              firstjmp64bitcmp;
              emit_ref_reg(A_CMP,S_L,right.location.reference,left.location.register64.reglo);
              secondjmp64bitcmp;
              location_freetemp(current_asmdata.CurrAsmList,right.location);
            end;
          LOC_CONSTANT :
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
          internalerror(200203282);
        end;

      end;


{*****************************************************************************
                                x86 MUL
*****************************************************************************}

    procedure ti386addnode.set_mul_result_location;
    begin
      location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
      {Free EAX,EDX}
      cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EDX);
      if is_64bit(resultdef) then
      begin
        {Allocate a couple of registers and store EDX:EAX into it}
        location.register64.reghi := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
        cg.a_load_reg_reg(current_asmdata.CurrAsmList, OS_INT, OS_INT, NR_EDX, location.register64.reghi);
        cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EAX);
        location.register64.reglo := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
        cg.a_load_reg_reg(current_asmdata.CurrAsmList, OS_INT, OS_INT, NR_EAX, location.register64.reglo);
      end
      else
      begin
        {Allocate a new register and store the result in EAX in it.}
        location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
        cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EAX);
        cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,NR_EAX,location.register);
      end;
      location_freetemp(current_asmdata.CurrAsmList,left.location);
      location_freetemp(current_asmdata.CurrAsmList,right.location);
    end;


    procedure ti386addnode.second_mul(unsigned: boolean);

    var reg,reghi,reglo:Tregister;
        ref:Treference;
        use_ref:boolean;
        hl4 : tasmlabel;

    const
      asmops: array[boolean] of tasmop = (A_IMUL, A_MUL);

    begin
      pass_left_right;
      reg:=NR_NO;
      reference_reset(ref,sizeof(pint),[]);

      { Mul supports registers and references, so if not register/reference,
        load the location into a register.
        The variant of IMUL which is capable of doing 32->64 bits has the same restrictions. }
      use_ref:=false;
      if left.location.loc in [LOC_REGISTER,LOC_CREGISTER] then
        reg:=left.location.register
      else if left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
        begin
          tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
          ref:=left.location.reference;
          use_ref:=true;
        end
      else
        begin
          { LOC_CONSTANT for example.}
          reg:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,left.resultdef,osuinttype,left.location,reg);
        end;

      if (CPUX86_HAS_BMI2 in cpu_capabilities[current_settings.cputype]) and
        (not(needoverflowcheck) or
        { 32->64 bit cannot overflow }
        is_64bit(resultdef)) then
        begin
          cg.getcpuregister(current_asmdata.CurrAsmList,NR_EDX);
          hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,osuinttype,right.location,NR_EDX);
          cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EDX);
          reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          if use_ref then
            current_asmdata.CurrAsmList.concat(Taicpu.Op_ref_reg_reg(A_MULX,S_L,ref,reglo,reghi))
          else
            emit_reg_reg_reg(A_MULX,S_L,reg,reglo,reghi);

          location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
          location.register64.reglo:=reglo;

          if is_64bit(resultdef) then
            location.register64.reghi:=reghi;

          location_freetemp(current_asmdata.CurrAsmList,left.location);
          location_freetemp(current_asmdata.CurrAsmList,right.location);
        end
      else
        begin
          { Allocate EAX. }
          cg.getcpuregister(current_asmdata.CurrAsmList,NR_EAX);
          { Load the right value. }
          hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,osuinttype,right.location,NR_EAX);
          { Also allocate EDX, since it is also modified by a mul (JM). }
          cg.getcpuregister(current_asmdata.CurrAsmList,NR_EDX);
          if use_ref then
            emit_ref(asmops[unsigned],S_L,ref)
          else
            emit_reg(asmops[unsigned],S_L,reg);
          if needoverflowcheck and
            { 32->64 bit cannot overflow }
            (not is_64bit(resultdef)) then
            begin
              current_asmdata.getjumplabel(hl4);
              cg.a_jmp_flags(current_asmdata.CurrAsmList,F_AE,hl4);
              cg.a_call_name(current_asmdata.CurrAsmList,'FPC_OVERFLOW',false);
              cg.a_label(current_asmdata.CurrAsmList,hl4);
            end;
          set_mul_result_location;
        end;
    end;


    procedure ti386addnode.second_mul64bit;
    var
      list: TAsmList;
      hreg1,hreg2: tregister;
    begin
      { 64x64 multiplication yields 128-bit result, but we're only
        interested in its lower 64 bits. This lower part is independent
        of operand signs, and so is the generated code. }
      { pass_left_right already called from second_add64bit }
      list:=current_asmdata.CurrAsmList;
      if left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
        tcgx86(cg).make_simple_ref(list,left.location.reference);
      if right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
        tcgx86(cg).make_simple_ref(list,right.location.reference);

      { calculate 32-bit terms lo(right)*hi(left) and hi(left)*lo(right) }
      if (right.location.loc=LOC_CONSTANT) then
        begin
          { Omit zero terms, if any }
          hreg1:=NR_NO;
          hreg2:=NR_NO;
          if lo(right.location.value64)<>0 then
            hreg1:=cg.getintregister(list,OS_INT);
          if hi(right.location.value64)<>0 then
            hreg2:=cg.getintregister(list,OS_INT);

          { Take advantage of 3-operand form of IMUL }
          case left.location.loc of
            LOC_REGISTER,LOC_CREGISTER:
              begin
                if hreg1<>NR_NO then
                  emit_const_reg_reg(A_IMUL,S_L,longint(lo(right.location.value64)),left.location.register64.reghi,hreg1);
                if hreg2<>NR_NO then
                  emit_const_reg_reg(A_IMUL,S_L,longint(hi(right.location.value64)),left.location.register64.reglo,hreg2);
              end;
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                if hreg2<>NR_NO then
                  list.concat(taicpu.op_const_ref_reg(A_IMUL,S_L,longint(hi(right.location.value64)),left.location.reference,hreg2));
                inc(left.location.reference.offset,4);
                if hreg1<>NR_NO then
                  list.concat(taicpu.op_const_ref_reg(A_IMUL,S_L,longint(lo(right.location.value64)),left.location.reference,hreg1));
                dec(left.location.reference.offset,4);
              end;
          else
            InternalError(2014011602);
          end;
        end
      else
        begin
          hreg1:=cg.getintregister(list,OS_INT);
          hreg2:=cg.getintregister(list,OS_INT);
          cg64.a_load64low_loc_reg(list,left.location,hreg1);
          cg64.a_load64high_loc_reg(list,left.location,hreg2);
          case right.location.loc of
            LOC_REGISTER,LOC_CREGISTER:
              begin
                emit_reg_reg(A_IMUL,S_L,right.location.register64.reghi,hreg1);
                emit_reg_reg(A_IMUL,S_L,right.location.register64.reglo,hreg2);
              end;
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                emit_ref_reg(A_IMUL,S_L,right.location.reference,hreg2);
                inc(right.location.reference.offset,4);
                emit_ref_reg(A_IMUL,S_L,right.location.reference,hreg1);
                dec(right.location.reference.offset,4);
              end;
          else
            InternalError(2014011603);
          end;
        end;
      { add hi*lo and lo*hi terms together }
      if (hreg1<>NR_NO) and (hreg2<>NR_NO) then
        emit_reg_reg(A_ADD,S_L,hreg2,hreg1);

      { load lo(right) into EAX }
      cg.getcpuregister(list,NR_EAX);
      cg64.a_load64low_loc_reg(list,right.location,NR_EAX);

      { multiply EAX by lo(left), producing 64-bit value in EDX:EAX }
      cg.getcpuregister(list,NR_EDX);
      if (left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
        emit_reg(A_MUL,S_L,left.location.register64.reglo)
      else if (left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
        emit_ref(A_MUL,S_L,left.location.reference)
      else
        InternalError(2014011604);
      { add previously calculated terms to the high half }
      if (hreg1<>NR_NO) then
        emit_reg_reg(A_ADD,S_L,hreg1,NR_EDX)
      else if (hreg2<>NR_NO) then
        emit_reg_reg(A_ADD,S_L,hreg2,NR_EDX)
      else
        InternalError(2014011604);

      { Result is now in EDX:EAX. Copy it to virtual registers. }
      set_mul_result_location;
    end;


begin
   caddnode:=ti386addnode;
end.
