{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate Xtensa assembler for math nodes

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
unit ncpumat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat;

    type
      tcpumoddivnode = class(tmoddivnode)
        procedure pass_generate_code;override;
      end;

      tcpunotnode = class(tcgnotnode)
        procedure second_boolean;override;
      end;

      tcpuunaryminusnode = class(tcgunaryminusnode)
        function pass_1: tnode; override;
        procedure second_float;override;
      end;

      tcpushlshrnode = class(tcgshlshrnode)
        procedure second_64bit;override;
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
                             TCPUMODDIVNODE
*****************************************************************************}

    procedure tcpumoddivnode.pass_generate_code;
      begin
        location.loc:=LOC_REGISTER;
      end;

{*****************************************************************************
                               TCPUNOTNODE
*****************************************************************************}

    procedure tcpunotnode.second_boolean;
      var
        tmpreg : TRegister;
      begin
        secondpass(left);

        location:=left.location;
        hlcg.location_force_reg(current_asmdata.CurrAsmList,location,resultdef,resultdef,false);
        { not supported yet }
        if is_64bit(resultdef) then
          Internalerror(2020031701);
        if is_cbool(resultdef) then
          cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NOT,def_cgsize(resultdef), location.register, location.register)
        else
          cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_XOR,def_cgsize(resultdef),1, location.register, location.register)
      end;

{*****************************************************************************
                               TARMUNARYMINUSNODE
*****************************************************************************}

    function tcpuunaryminusnode.pass_1: tnode;
      var
        procname: string[31];
        fdef : tdef;
      begin
        Result:=nil;
        if (current_settings.fputype=fpu_soft) and
           (left.resultdef.typ=floatdef) then
          begin
            result:=nil;
            firstpass(left);
            expectloc:=LOC_REGISTER;
            exit;
          end;

        result:=nil;
        firstpass(left);
        if codegenerror then
          exit;

        expectloc:=LOC_REGISTER;
      end;

    procedure tcpuunaryminusnode.second_float;
      begin
        secondpass(left);
        if (current_settings.fputype=fpu_soft) or (tfloatdef(left.resultdef).floattype<>s32real) or
          not(FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) then
          begin
            if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
            location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
            if location.size in [OS_64,OS_S64,OS_F64] then
              begin
                location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
              end
            else
              location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);

            case location.size of
              OS_32:
                cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_XOR,OS_32,tcgint($80000000),left.location.register,location.register);
              OS_64:
                cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_XOR,OS_32,tcgint($80000000),left.location.registerhi,location.registerhi);
            else
              internalerror(2014033101);
            end;
          end
        else
          begin
            if not(left.location.loc in [LOC_CFPUREGISTER,LOC_FPUREGISTER]) then
              hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,false);
            location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
            location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
            current_asmdata.CurrAsmList.Concat(taicpu.op_reg_reg(A_NEG_S,location.register,left.location.register));
          end;
      end;


    procedure tcpushlshrnode.second_64bit;
      var
        v : TConstExprInt;
        lreg, resreg: TRegister64;

      procedure emit_instr(p: tai);
        begin
          current_asmdata.CurrAsmList.concat(p);
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
      procedure shift_by_variable(srchi, srclo, dsthi, dstlo, shiftval: TRegister);
        var
          shiftval1,shiftval2:TRegister;
        begin
          //shifterop_reset(so);
          //shiftval1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          //shiftval2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
          //
          //cg.a_load_reg_reg(current_asmdata.CurrAsmList, OS_INT, OS_INT, shiftval, shiftval1);
          //
          //{The ARM barrel shifter only considers the lower 8 bits of a register for the shift}
          //cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
          //emit_instr(taicpu.op_reg_const(A_CMP, shiftval1, 64));
          //emit_instr(setcondition(taicpu.op_reg_const(A_MOV, shiftval1, 64), C_CS));
          //cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
          //
          //{Calculate how much the upper register needs to be shifted left}
          //emit_instr(taicpu.op_reg_reg_const(A_RSB, shiftval2, shiftval1, 32));
          //
          //so.shiftmode:=sm;
          //so.rs:=shiftval1;
          //
          //{Shift and zerofill the hi+lo register}
          //emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, dstlo, srclo, so));
          //emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, dsthi, srchi, so));
          //
          //{Fold in the lower 32-shiftval bits}
          //if sm = SM_LSR then so.shiftmode:=SM_LSL else so.shiftmode:=SM_LSR;
          //so.rs:=shiftval2;
          //emit_instr(taicpu.op_reg_reg_reg_shifterop(A_ORR, dstlo, dstlo, srchi, so));
          //
          //cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
          //emit_instr(setoppostfix(taicpu.op_reg_reg_const(A_SUB, shiftval2, shiftval1, 32), PF_S));
          //
          //so.shiftmode:=sm;
          //emit_instr(setcondition(taicpu.op_reg_reg_shifterop(A_MOV, dstlo, srchi, so), C_PL));
          //cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        end;

      begin         
        inherited;
        //if GenerateThumbCode or GenerateThumb2Code then
        //begin
        //  inherited;
        //  exit;
        //end;
        //
        //location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        //location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
        //location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
        //
        //{ load left operator in a register }
        //if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
        //   (left.location.size<>OS_64) then
        //  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,true);
        //
        //lreg := left.location.register64;
        //resreg := location.register64;
        //shifterop_reset(so);
        //
        //{ shifting by a constant directly coded: }
        //if (right.nodetype=ordconstn) then
        //  begin
        //    v:=Tordconstnode(right).value and 63;
        //    {Single bit shift}
        //    if v = 1 then
        //      if nodetype=shln then
        //        begin
        //          {Shift left by one by 2 simple 32bit additions}
        //          cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        //          emit_instr(setoppostfix(taicpu.op_reg_reg_reg(A_ADD, resreg.reglo, lreg.reglo, lreg.reglo), PF_S));
        //          emit_instr(taicpu.op_reg_reg_reg(A_ADC, resreg.reghi, lreg.reghi, lreg.reghi));
        //          cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        //        end
        //      else
        //        begin
        //          {Shift right by first shifting hi by one and then using RRX (rotate right extended), which rotates through the carry}
        //          shifterop_reset(so); so.shiftmode:=SM_LSR; so.shiftimm:=1;
        //          cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        //          emit_instr(setoppostfix(taicpu.op_reg_reg_shifterop(A_MOV, resreg.reghi, lreg.reghi, so), PF_S));
        //          so.shiftmode:=SM_RRX; so.shiftimm:=0; {RRX does NOT have a shift amount}
        //          emit_instr(taicpu.op_reg_reg_shifterop(A_MOV, resreg.reglo, lreg.reglo, so));
        //          cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_DEFAULTFLAGS);
        //        end
        //    {Clear one register and use the cg to generate a normal 32-bit shift}
        //    else if v >= 32 then
        //      if nodetype=shln then
        //      begin
        //        emit_instr(taicpu.op_reg_const(A_MOV, resreg.reglo, 0));
        //        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHL,OS_32,v.uvalue-32,lreg.reglo,resreg.reghi);
        //      end
        //      else
        //      begin
        //        emit_instr(taicpu.op_reg_const(A_MOV, resreg.reghi, 0));
        //        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_32,v.uvalue-32,lreg.reghi,resreg.reglo);
        //      end
        //    {Shift LESS than 32, thats the tricky one}
        //    else if (v < 32) and (v > 1) then
        //      if nodetype=shln then
        //        shift_less_than_32(lreg.reglo, lreg.reghi, resreg.reglo, resreg.reghi, v.uvalue, SM_LSL)
        //      else
        //        shift_less_than_32(lreg.reghi, lreg.reglo, resreg.reghi, resreg.reglo, v.uvalue, SM_LSR);
        //  end
        //else
        //  begin
        //    { force right operator into a register }
        //    if not(right.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
        //       (right.location.size<>OS_32) then
        //      hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,u32inttype,true);
        //
        //    if nodetype = shln then
        //      shift_by_variable(lreg.reglo, lreg.reghi, resreg.reglo, resreg.reghi, right.location.register, SM_LSL)
        //    else
        //      shift_by_variable(lreg.reghi, lreg.reglo, resreg.reghi, resreg.reglo, right.location.register, SM_LSR);
        //  end;
      end;


begin
  cmoddivnode:=tcpumoddivnode;
  cnotnode:=tcpunotnode;
  cunaryminusnode:=tcpuunaryminusnode;
  cshlshrnode:=tcpushlshrnode;
end.
