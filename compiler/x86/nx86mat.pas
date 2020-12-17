{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate x86 code for math nodes

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
unit nx86mat;

{$i fpcdefs.inc}

interface

    uses
      node,ncgmat;

    type
      tx86unaryminusnode = class(tcgunaryminusnode)
{$ifdef SUPPORT_MMX}
         procedure second_mmx;override;
{$endif SUPPORT_MMX}
         procedure second_float;override;
         function pass_1:tnode;override;
      end;

      tx86notnode = class(tcgnotnode)
         procedure second_boolean;override;
{$ifdef SUPPORT_MMX}
         procedure second_mmx;override;
{$endif SUPPORT_MMX}
      end;

      tx86moddivnode = class(tcgmoddivnode)
         procedure pass_generate_code;override;
      end;

      tx86shlshrnode = class(tcgshlshrnode)
{$ifdef SUPPORT_MMX}
         procedure second_mmx;override;
{$endif SUPPORT_MMX}
      end;

  implementation

    uses
      globtype,
      constexp,
      cutils,verbose,globals,
      symconst,symdef,
      aasmbase,aasmtai,aasmcpu,aasmdata,defutil,
      cgbase,pass_1,pass_2,
      ncon,
      cpubase,cpuinfo,
      cga,cgobj,hlcgobj,cgx86,cgutils,
      tgobj;


{*****************************************************************************
                          TI386UNARYMINUSNODE
*****************************************************************************}

    function tx86unaryminusnode.pass_1 : tnode;
      begin
         result:=nil;
         firstpass(left);
         if codegenerror then
           exit;

         if (left.resultdef.typ=floatdef) then
           begin
             if use_vectorfpu(left.resultdef) then
               expectloc:=LOC_MMREGISTER
             else
               expectloc:=LOC_FPUREGISTER;
           end
{$ifdef SUPPORT_MMX}
         else
           if (cs_mmx in current_settings.localswitches) and
              is_mmx_able_array(left.resultdef) then
             begin
               expectloc:=LOC_MMXREGISTER;
             end
{$endif SUPPORT_MMX}
         else
           inherited pass_1;
      end;


{$ifdef SUPPORT_MMX}
    procedure tx86unaryminusnode.second_mmx;
      var
        op : tasmop;
        hreg : tregister;
      begin
        op:=A_NONE;
        secondpass(left);
        location_reset(location,LOC_MMXREGISTER,OS_NO);
        hreg:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
        emit_reg_reg(A_PXOR,S_NO,hreg,hreg);
        case left.location.loc of
          LOC_MMXREGISTER:
            begin
               location.register:=left.location.register;
            end;
          LOC_CMMXREGISTER:
            begin
               location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
               emit_reg_reg(A_MOVQ,S_NO,left.location.register,location.register);
            end;
          LOC_REFERENCE,
          LOC_CREFERENCE:
            begin
               location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
               emit_ref_reg(A_MOVQ,S_NO,left.location.reference,location.register);
            end;
          else
            internalerror(200203225);
        end;
        if cs_mmx_saturation in current_settings.localswitches then
          case mmx_type(resultdef) of
             mmxs8bit:
               op:=A_PSUBSB;
             mmxu8bit:
               op:=A_PSUBUSB;
             mmxs16bit,mmxfixed16:
               op:=A_PSUBSW;
             mmxu16bit:
               op:=A_PSUBUSW;
             else
               ;
          end
        else
          case mmx_type(resultdef) of
             mmxs8bit,mmxu8bit:
               op:=A_PSUBB;
             mmxs16bit,mmxu16bit,mmxfixed16:
               op:=A_PSUBW;
             mmxs32bit,mmxu32bit:
               op:=A_PSUBD;
             else
               ;
          end;
        if op = A_NONE then
          internalerror(201408202);

        emit_reg_reg(op,S_NO,location.register,hreg);
        emit_reg_reg(A_MOVQ,S_NO,hreg,location.register);
      end;
{$endif SUPPORT_MMX}


    procedure tx86unaryminusnode.second_float;
      var
        reg : tregister;
        href : treference;
        l1 : tasmlabel;
      begin
        secondpass(left);

        if expectloc=LOC_MMREGISTER then
          begin
            if not(left.location.loc in [LOC_MMREGISTER,LOC_CMMREGISTER,LOC_CREFERENCE,LOC_REFERENCE]) then
              hlcg.location_force_mmregscalar(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
            location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));

            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,def_cgsize(resultdef));
            cg.a_opmm_reg_reg(current_asmdata.CurrAsmList,OP_XOR,location.size,location.register,location.register,nil);
            cg.a_opmm_loc_reg(current_asmdata.CurrAsmList,OP_SUB,location.size,left.location,location.register,mms_movescalar);
          end
        else
          begin
            location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
            case left.location.loc of
              LOC_REFERENCE,
              LOC_CREFERENCE:
                begin
                  location.register:=NR_ST;
                  cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList,
                     left.location.size,location.size,
                     left.location.reference,location.register);
                  emit_none(A_FCHS,S_NO);
                end;
              LOC_FPUREGISTER,
              LOC_CFPUREGISTER:
                begin
                   { "load st,st" is ignored by the code generator }
                   cg.a_loadfpu_reg_reg(current_asmdata.CurrAsmList,left.location.size,location.size,left.location.register,NR_ST);
                   location.register:=NR_ST;
                   emit_none(A_FCHS,S_NO);
                end;
              else
                internalerror(200312241);
            end;
          end;
      end;


{*****************************************************************************
                               TX86NOTNODE
*****************************************************************************}

    procedure tx86notnode.second_boolean;
      var
         opsize : tcgsize;
         {$if defined(cpu32bitalu) or defined(cpu16bitalu)}
         hreg: tregister;
         {$endif}
      begin
        opsize:=def_cgsize(resultdef);

        secondpass(left);
        if not handle_locjump then
         begin
           case left.location.loc of
             LOC_FLAGS :
               begin
                 location_reset(location,LOC_FLAGS,OS_NO);
                 location.resflags:=left.location.resflags;
                 inverse_flags(location.resflags);
               end;
             LOC_CREFERENCE,
             LOC_REFERENCE:
               begin
{$if defined(cpu32bitalu)}
                 if is_64bit(resultdef) then
                   begin
                     hreg:=cg.GetIntRegister(current_asmdata.CurrAsmList,OS_32);
                     tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
                     cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_32,OS_32,left.location.reference,hreg);
                     inc(left.location.reference.offset,4);
                     cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.reference,hreg);
                   end
                 else
{$elseif defined(cpu16bitalu)}
                 if is_64bit(resultdef) then
                   begin
                     hreg:=cg.GetIntRegister(current_asmdata.CurrAsmList,OS_16);
                     tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
                     cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_16,OS_16,left.location.reference,hreg);
                     inc(left.location.reference.offset,2);
                     cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,left.location.reference,hreg);
                     inc(left.location.reference.offset,2);
                     cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,left.location.reference,hreg);
                     inc(left.location.reference.offset,2);
                     cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,left.location.reference,hreg);
                   end
                 else if is_32bit(resultdef) then
                   begin
                     hreg:=cg.GetIntRegister(current_asmdata.CurrAsmList,OS_16);
                     tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
                     cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_16,OS_16,left.location.reference,hreg);
                     inc(left.location.reference.offset,2);
                     cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,left.location.reference,hreg);
                   end
                 else
{$endif}
                   emit_const_ref(A_CMP, TCGSize2Opsize[opsize], 0, left.location.reference);
                 location_reset(location,LOC_FLAGS,OS_NO);
                 location.resflags:=F_E;
               end;
             LOC_CONSTANT,
             LOC_REGISTER,
             LOC_CREGISTER,
             LOC_SUBSETREG,
             LOC_CSUBSETREG,
             LOC_SUBSETREF,
             LOC_CSUBSETREF :
               begin
{$if defined(cpu32bitalu)}
                 if is_64bit(resultdef) then
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
                     emit_reg_reg(A_OR,S_L,left.location.register64.reghi,left.location.register64.reglo);
                   end
                 else
{$elseif defined(cpu16bitalu)}
                 if is_64bit(resultdef) then
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
                     emit_reg_reg(A_OR,S_W,cg.GetNextReg(left.location.register64.reghi),left.location.register64.reghi);
                     emit_reg_reg(A_OR,S_W,cg.GetNextReg(left.location.register64.reglo),left.location.register64.reglo);
                     emit_reg_reg(A_OR,S_W,left.location.register64.reghi,left.location.register64.reglo);
                   end
                 else if is_32bit(resultdef) then
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
                     emit_reg_reg(A_OR,S_L,cg.GetNextReg(left.location.register),left.location.register);
                   end
                 else
{$endif}
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,true);
                     emit_reg_reg(A_TEST,TCGSize2Opsize[opsize],left.location.register,left.location.register);
                   end;
                 location_reset(location,LOC_FLAGS,OS_NO);
                 location.resflags:=F_E;
               end;
            else
               internalerror(200203224);
           end;
         end;
      end;


{$ifdef SUPPORT_MMX}
    procedure tx86notnode.second_mmx;

    var hreg,r:Tregister;

    begin
      secondpass(left);
      location_reset(location,LOC_MMXREGISTER,OS_NO);
      r:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
      emit_const_reg(A_MOV,S_L,longint($ffffffff),r);
      { load operand }
      case left.location.loc of
        LOC_MMXREGISTER:
          location_copy(location,left.location);
        LOC_CMMXREGISTER:
          begin
            location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
            emit_reg_reg(A_MOVQ,S_NO,left.location.register,location.register);
          end;
        LOC_REFERENCE,
        LOC_CREFERENCE:
          begin
            location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
            emit_ref_reg(A_MOVQ,S_NO,left.location.reference,location.register);
          end;
        else
          internalerror(2019050906);
      end;
      { load mask }
      hreg:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
      emit_reg_reg(A_MOVD,S_NO,r,hreg);
      { lower 32 bit }
      emit_reg_reg(A_PXOR,S_NO,hreg,location.register);
      { shift mask }
      emit_const_reg(A_PSLLQ,S_B,32,hreg);
      { higher 32 bit }
      emit_reg_reg(A_PXOR,S_NO,hreg,location.register);
    end;
{$endif SUPPORT_MMX}


{*****************************************************************************
                             TX86MODDIVNODE
*****************************************************************************}

    procedure tx86moddivnode.pass_generate_code;
      var
        hreg1,hreg2,hreg3,rega,regd,tempreg:Tregister;
        power:longint;
        instr:TAiCpu;
        op:Tasmop;
        cgsize:TCgSize;
        opsize:topsize;
        e, sm: aint;
        d,m: aword;
        m_add, invertsign: boolean;
        s: byte;
      label
        DefaultDiv;
      begin
        secondpass(left);
        if codegenerror then
          exit;
        secondpass(right);
        if codegenerror then
          exit;

        { put numerator in register }
        cgsize:=def_cgsize(resultdef);
        opsize:=TCGSize2OpSize[cgsize];
        rega:=newreg(R_INTREGISTER,RS_EAX,cgsize2subreg(R_INTREGISTER,cgsize));
        regd:=newreg(R_INTREGISTER,RS_EDX,cgsize2subreg(R_INTREGISTER,cgsize));

        location_reset(location,LOC_REGISTER,cgsize);
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
        hreg1:=left.location.register;

        if (nodetype=divn) and (right.nodetype=ordconstn) then
          begin
            if isabspowerof2(tordconstnode(right).value,power) then
              begin
                { for signed numbers, the numerator must be adjusted before the
                  shift instruction, but not with unsigned numbers! Otherwise,
                  "Cardinal($ffffffff) div 16" overflows! (JM) }
                if is_signed(left.resultdef) Then
                  begin
                    invertsign:=tordconstnode(right).value<0;
                    { use a sequence without jumps, saw this in
                      comp.compilers (JM) }
                    { no jumps, but more operations }
                    hreg2:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                    emit_reg_reg(A_MOV,opsize,hreg1,hreg2);
                    if power=1 then
                      begin
                        {If the left value is negative, hreg2=(1 shl power)-1=1, otherwise 0.}
                        cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,cgsize,resultdef.size*8-1,hreg2);
                      end
                    else
                      begin
                        {If the left value is negative, hreg2=$ffffffff, otherwise 0.}
                        cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SAR,cgsize,resultdef.size*8-1,hreg2);
                        {If negative, hreg2=(1 shl power)-1, otherwise 0.}
                        { (don't use emit_const_reg, because if value>high(longint)
                           then it must first be loaded into a register) }
                        cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,cgsize,(aint(1) shl power)-1,hreg2);
                      end;
                    { add to the left value }
                    emit_reg_reg(A_ADD,opsize,hreg2,hreg1);
                    { do the shift }
                    cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SAR,cgsize,power,hreg1);
                    if invertsign then
                      emit_reg(A_NEG,opsize,hreg1);
                  end
                else
                  cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,cgsize,power,hreg1);
                location.register:=hreg1;
              end
            else
              begin
                if is_signed(left.resultdef) then
                  begin
                    e:=tordconstnode(right).value.svalue;
                    calc_divconst_magic_signed(resultdef.size*8,e,sm,s);
                    cg.getcpuregister(current_asmdata.CurrAsmList,rega);
                    emit_const_reg(A_MOV,opsize,sm,rega);
                    cg.getcpuregister(current_asmdata.CurrAsmList,regd);
                    emit_reg(A_IMUL,opsize,hreg1);
                    { only the high half of result is used }
                    cg.ungetcpuregister(current_asmdata.CurrAsmList,rega);
                    { add or subtract dividend }
                    if (e>0) and (sm<0) then
                      emit_reg_reg(A_ADD,opsize,hreg1,regd)
                    else if (e<0) and (sm>0) then
                      emit_reg_reg(A_SUB,opsize,hreg1,regd);
                    { shift if necessary }
                    if (s<>0) then
                      emit_const_reg(A_SAR,opsize,s,regd);
                    { extract and add the sign bit }
                    if (e<0) then
                      emit_reg_reg(A_MOV,opsize,regd,hreg1);
                    { if e>=0, hreg1 still contains dividend }
                    emit_const_reg(A_SHR,opsize,left.resultdef.size*8-1,hreg1);
                    emit_reg_reg(A_ADD,opsize,hreg1,regd);
                    cg.ungetcpuregister(current_asmdata.CurrAsmList,regd);
                    location.register:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                    cg.a_load_reg_reg(current_asmdata.CurrAsmList,cgsize,cgsize,regd,location.register)
                  end
                else
                  begin
                    d:=tordconstnode(right).value.svalue;
                    if d>=aword(1) shl (left.resultdef.size*8-1) then
                      begin
                        location.register:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                        { Ensure that the whole register is 0, since SETcc only sets the lowest byte }

                        { If the operands are 64 bits, this XOR routine will be shrunk by the
                          peephole optimizer. [Kit] }
                        emit_reg_reg(A_XOR,opsize,location.register,location.register);

                        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        if (cgsize in [OS_64,OS_S64]) then { Cannot use 64-bit constants in CMP }
                          begin
                            hreg2:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                            emit_const_reg(A_MOV,opsize,aint(d),hreg2);
                            emit_reg_reg(A_CMP,opsize,hreg2,hreg1);
                          end
                        else
                          emit_const_reg(A_CMP,opsize,aint(d),hreg1);
                        { NOTE: SBB and SETAE are both 3 bytes long without the REX prefix,
                          both use an ALU for their execution and take a single cycle to
                          run. The only difference is that SETAE does not modify the flags,
                          allowing for some possible reuse. [Kit] }

                        { Emit a SETcc instruction that depends on the carry bit being zero,
                          that is, the numerator is greater than or equal to the denominator. }
                        tempreg:=cg.makeregsize(current_asmdata.CurrAsmList,location.register,OS_8);
                        instr:=TAiCpu.op_reg(A_SETcc,S_B,tempreg);
                        instr.condition:=C_AE;
                        current_asmdata.CurrAsmList.concat(instr);
                        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                      end
                    else
                      begin
                        calc_divconst_magic_unsigned(resultdef.size*8,d,m,m_add,s);
                        cg.getcpuregister(current_asmdata.CurrAsmList,rega);
                        emit_const_reg(A_MOV,opsize,aint(m),rega);
                        cg.getcpuregister(current_asmdata.CurrAsmList,regd);
                        emit_reg(A_MUL,opsize,hreg1);
                        cg.ungetcpuregister(current_asmdata.CurrAsmList,rega);
                        if m_add then
                          begin
                            { addition can overflow, shift first bit considering carry,
                              then shift remaining bits in regular way. }
                            emit_reg_reg(A_ADD,opsize,hreg1,regd);
                            emit_const_reg(A_RCR,opsize,1,regd);
                            dec(s);
                          end;
                        if s<>0 then
                          emit_const_reg(A_SHR,opsize,aint(s),regd);
                        cg.ungetcpuregister(current_asmdata.CurrAsmList,regd);
                        location.register:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                        cg.a_load_reg_reg(current_asmdata.CurrAsmList,cgsize,cgsize,regd,location.register)
                      end;
                  end;
              end;
          end
        else if (nodetype=modn) and (right.nodetype=ordconstn) and not(is_signed(left.resultdef)) then
          begin
            { unsigned modulus by a (+/-)power-of-2 constant? }
            if isabspowerof2(tordconstnode(right).value,power) then
              begin
                emit_const_reg(A_AND,opsize,(aint(1) shl power)-1,hreg1);
                location.register:=hreg1;
              end
            else
              begin
                d:=tordconstnode(right).value.svalue;
                if d>=aword(1) shl (left.resultdef.size*8-1) then
                  begin

                    if not (CPUX86_HAS_CMOV in cpu_capabilities[current_settings.cputype]) then
                      goto DefaultDiv;

                    location.register:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                    hreg3:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);

                    m := aword(-aint(d)); { Two's complement of d }

                    if (cgsize in [OS_64,OS_S64]) then { Cannot use 64-bit constants in CMP }
                      begin
                        hreg2:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                        emit_const_reg(A_MOV,opsize,aint(d),hreg2);
                        emit_const_reg(A_MOV,opsize,aint(m),hreg3);
                        emit_reg_reg(A_XOR,opsize,location.register,location.register);
                        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        emit_reg_reg(A_CMP,opsize,hreg2,hreg1);
                      end
                    else
                      begin
                        emit_const_reg(A_MOV,opsize,aint(m),hreg3);
                        emit_reg_reg(A_XOR,opsize,location.register,location.register);

                        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        emit_const_reg(A_CMP,opsize,aint(d),hreg1);
                      end;

                    { Emit conditional move that depends on the carry flag being zero,
                      that is, the comparison result is above or equal }
                    instr:=TAiCpu.op_reg_reg(A_CMOVcc,opsize,hreg3,location.register);
                    instr.condition := C_AE;
                    current_asmdata.CurrAsmList.concat(instr);
                    cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);

                    emit_reg_reg(A_ADD,opsize,hreg1,location.register);
                  end
                else
                  begin
                    { Convert the division to a multiplication }
                    calc_divconst_magic_unsigned(resultdef.size*8,d,m,m_add,s);
                    cg.getcpuregister(current_asmdata.CurrAsmList,rega);
                    emit_const_reg(A_MOV,opsize,aint(m),rega);
                    cg.getcpuregister(current_asmdata.CurrAsmList,regd);
                    emit_reg(A_MUL,opsize,hreg1);
                    cg.ungetcpuregister(current_asmdata.CurrAsmList,rega);
                    hreg2:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                    emit_reg_reg(A_MOV,opsize,hreg1,hreg2);
                    if m_add then
                      begin
                        { addition can overflow, shift first bit considering carry,
                          then shift remaining bits in regular way. }
                        cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        emit_reg_reg(A_ADD,opsize,hreg1,regd);
                        emit_const_reg(A_RCR,opsize,1,regd);
                        cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
                        dec(s);
                      end;
                    if s<>0 then
                      emit_const_reg(A_SHR,opsize,aint(s),regd); { R/EDX now contains the quotient }

                    { Now multiply the quotient by the original denominator and
                      subtract the product from the original numerator to get
                      the remainder. }
                    if (cgsize in [OS_64,OS_S64]) then { Cannot use 64-bit constants in IMUL }
                      begin
                        hreg3:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
                        emit_const_reg(A_MOV,opsize,aint(d),hreg3);
                        emit_reg_reg(A_IMUL,opsize,hreg3,regd);
                      end
                    else
                      emit_const_reg(A_IMUL,opsize,aint(d),regd);

                    emit_reg_reg(A_SUB,opsize,regd,hreg2);
                    cg.ungetcpuregister(current_asmdata.CurrAsmList,regd);
                    location.register:=hreg2;
                  end;
              end;
          end
        else if (nodetype=modn) and (right.nodetype=ordconstn) and (is_signed(left.resultdef)) and isabspowerof2(tordconstnode(right).value,power) then
          begin
            hreg2:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
            if power=1 then
              cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,cgsize,resultdef.size*8-power,hreg1,hreg2)
            else
              begin
                cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SAR,cgsize,resultdef.size*8-1,hreg1,hreg2);
                cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,cgsize,resultdef.size*8-power,hreg2,hreg2);
              end;
            emit_reg_reg(A_ADD,opsize,hreg1,hreg2);
            cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,cgsize,not((aint(1) shl power)-1),hreg2);
            emit_reg_reg(A_SUB,opsize,hreg2,hreg1);
            location.register:=hreg1;
          end
        else
          begin
DefaultDiv:
            {Bring denominator to a register.}
            cg.getcpuregister(current_asmdata.CurrAsmList,rega);
            emit_reg_reg(A_MOV,opsize,hreg1,rega);
            cg.getcpuregister(current_asmdata.CurrAsmList,regd);
            {Sign extension depends on the left type.}
            if is_signed(left.resultdef) then
              case left.resultdef.size of
{$ifdef x86_64}
                8:
                  emit_none(A_CQO,S_NO);
{$endif x86_64}
                4:
                  emit_none(A_CDQ,S_NO);
                else
                  internalerror(2013102704);
              end
            else
              emit_reg_reg(A_XOR,opsize,regd,regd);

            { Division depends on the result type }
            if is_signed(resultdef) then
              op:=A_IDIV
            else
              op:=A_DIV;

            if right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE] then
              emit_ref(op,opsize,right.location.reference)
            else if right.location.loc in [LOC_REGISTER,LOC_CREGISTER] then
              emit_reg(op,opsize,right.location.register)
            else
              begin
                hreg1:=cg.getintregister(current_asmdata.CurrAsmList,right.location.size);
                hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,right.resultdef,right.resultdef,right.location,hreg1);
                emit_reg(op,opsize,hreg1);
              end;

            { Copy the result into a new register. Release R/EAX & R/EDX.}
            cg.ungetcpuregister(current_asmdata.CurrAsmList,regd);
            cg.ungetcpuregister(current_asmdata.CurrAsmList,rega);
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,cgsize);
            if nodetype=divn then
              cg.a_load_reg_reg(current_asmdata.CurrAsmList,cgsize,cgsize,rega,location.register)
            else
              cg.a_load_reg_reg(current_asmdata.CurrAsmList,cgsize,cgsize,regd,location.register);
          end;
      end;


{$ifdef SUPPORT_MMX}
    procedure tx86shlshrnode.second_mmx;
      var
        op         : TAsmOp;
        mmxbase    : tmmxtype;
        hregister  : tregister;
      begin
        secondpass(left);
        if codegenerror then
          exit;
        secondpass(right);
        if codegenerror then
          exit;

        op:=A_NOP;

        mmxbase:=mmx_type(left.resultdef);
        location_reset(location,LOC_MMXREGISTER,def_cgsize(resultdef));
        case nodetype of
          shrn :
            case mmxbase of
               mmxs16bit,mmxu16bit,mmxfixed16:
                 op:=A_PSRLW;
               mmxs32bit,mmxu32bit:
                 op:=A_PSRLD;
               mmxs64bit,mmxu64bit:
                 op:=A_PSRLQ;
               else
                 Internalerror(2018022504);
            end;
          shln :
            case mmxbase of
               mmxs16bit,mmxu16bit,mmxfixed16:
                 op:=A_PSLLW;
               mmxs32bit,mmxu32bit:
                 op:=A_PSLLD;
               mmxs64bit,mmxu64bit:
                 op:=A_PSLLD;
               else
                 Internalerror(2018022503);
            end;
          else
            internalerror(2018022502);
        end;

        { left and right no register?  }
        { then one must be demanded    }
        if (left.location.loc<>LOC_MMXREGISTER) then
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
               internalerror(2018022505);

              hregister:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
              tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,left.location.reference);
              emit_ref_reg(A_MOVQ,S_NO,left.location.reference,hregister);
            end;

           location_reset(left.location,LOC_MMXREGISTER,OS_NO);
           left.location.register:=hregister;
         end;

        { at this point, left.location.loc should be LOC_MMXREGISTER }
        case right.location.loc of
          LOC_MMXREGISTER,LOC_CMMXREGISTER:
            begin
              emit_reg_reg(op,S_NO,right.location.register,left.location.register);
              location.register:=left.location.register;
            end;
          LOC_CONSTANT:
            emit_const_reg(op,S_NO,right.location.value,left.location.register);
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,right.location.reference);
              emit_ref_reg(op,S_NO,right.location.reference,left.location.register);
            end;
          else
            internalerror(2018022506);
        end;
        location.register:=left.location.register;

        location_freetemp(current_asmdata.CurrAsmList,right.location);
      end;
{$endif SUPPORT_MMX}

end.
