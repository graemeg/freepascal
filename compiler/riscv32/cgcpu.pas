{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the code generator for the Risc-V32

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
unit cgcpu;

{$i fpcdefs.inc}

  interface

    uses
       globtype,symtype,symdef,
       cgbase,cgobj,cgrv,
       aasmbase,aasmcpu,aasmtai,aasmdata,
       cpubase,cpuinfo,cgutils,cg64f32,rgcpu,
       parabase;

    type
      tcgrv32 = class(tcgrv)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        { move instructions }
        procedure a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);override;

        { 32x32 to 64 bit multiplication }
        procedure a_mul_reg_reg_pair(list: TAsmList;size: tcgsize; src1,src2,dstlo,dsthi: tregister); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;

        procedure g_concatcopy_move(list: tasmlist; const Source, dest: treference; len: tcgint);
        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;

        procedure g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef); override;
     end;

     tcg64frv = class(tcg64f32)
       procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
       procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
       procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);override;
       procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
     end;

  procedure create_codegen;

  implementation

    uses
       symtable,
       globals,verbose,systems,cutils,
       symconst,symsym,fmodule,
       rgobj,tgobj,cpupi,procinfo,paramgr;

{ Range check must be disabled explicitly as conversions between signed and unsigned
  32-bit values are done without explicit typecasts }
{$R-}

    procedure tcgrv32.init_register_allocators;
      begin
        inherited init_register_allocators;
        rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
          [RS_X10,RS_X11,RS_X12,RS_X13,RS_X14,RS_X15,RS_X16,RS_X17,
           RS_X31,RS_X30,RS_X29,RS_X28,
           RS_X5,RS_X6,RS_X7,
           RS_X3,RS_X4,
           RS_X9,RS_X27,RS_X26,RS_X25,RS_X24,RS_X23,RS_X22,
           RS_X21,RS_X20,RS_X19,RS_X18],first_int_imreg,[]);
        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBNONE,
          [RS_F10,RS_F11,RS_F12,RS_F13,RS_F14,RS_F15,RS_F16,RS_F17,
           RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7,
           RS_F28,RS_F29,RS_F30,RS_F31,
           RS_F8,RS_F9,
           RS_F27,
           RS_F26,RS_F25,RS_F24,RS_F23,RS_F22,RS_F21,RS_F20,RS_F19,RS_F18],first_fpu_imreg,[]);
      end;


    procedure tcgrv32.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        inherited done_register_allocators;
      end;


    procedure tcgrv32.a_load_reg_reg(list : TAsmList;fromsize, tosize : tcgsize;reg1,reg2 : tregister);
      var
        ai: taicpu;
      begin
        if (fromsize=tosize) or
           ((tcgsize2unsigned[fromsize]=tcgsize2unsigned[tosize]) and
            (tcgsize2unsigned[fromsize]=OS_32)) then
          begin
            ai:=taicpu.op_reg_reg_const(A_ADDI,reg2,reg1,0);
            list.concat(ai);
            rg[R_INTREGISTER].add_move_instruction(ai);
          end
        else if fromsize=OS_8 then
          begin
            list.Concat(taicpu.op_reg_reg_const(A_AND,reg2,reg1,$FF))
          end
        else
          begin
            if tcgsize2size[tosize]<tcgsize2size[fromsize] then
              fromsize:=tosize;

            if tcgsize2unsigned[fromsize]<>OS_32 then
              list.Concat(taicpu.op_reg_reg_const(A_SLLI,reg2,reg1,8*(4-tcgsize2size[fromsize])))
            else
              a_load_reg_reg(list,fromsize,fromsize,reg1,reg2);

            if tcgsize2unsigned[fromsize]=fromsize then
              list.Concat(taicpu.op_reg_reg_const(A_SRLI,reg2,reg2,8*(4-tcgsize2size[fromsize])))
            else
              list.Concat(taicpu.op_reg_reg_const(A_SRAI,reg2,reg2,8*(4-tcgsize2size[fromsize])));
          end;
      end;


    procedure tcgrv32.a_mul_reg_reg_pair(list: TAsmList;size: tcgsize; src1,src2,dstlo,dsthi: tregister);
      var
        op: tasmop;
      begin
        case size of
          OS_INT:  op:=A_MULHU;
          OS_SINT: op:=A_MULH;
        else
          InternalError(2014061501);
        end;
        if (dsthi<>NR_NO) then
          list.concat(taicpu.op_reg_reg_reg(op,dsthi,src1,src2));
        { low word is always unsigned }
        if (dstlo<>NR_NO) then
          list.concat(taicpu.op_reg_reg_reg(A_MUL,dstlo,src1,src2));
      end;


    procedure tcgrv32.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
        regs, fregs: tcpuregisterset;
        r: TSuperRegister;
        href: treference;
        stackcount: longint;
      begin
        if not(nostackframe) then
          begin
            a_reg_alloc(list,NR_STACK_POINTER_REG);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              a_reg_alloc(list,NR_FRAME_POINTER_REG);

            reference_reset_base(href,NR_STACK_POINTER_REG,-4,ctempposinvalid,0,[]);

            { Int registers }
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_RETURN_ADDRESS_REG];

            if (pi_do_call in current_procinfo.flags) then
              regs:=regs+[RS_RETURN_ADDRESS_REG];

            stackcount:=0;
            for r:=RS_X0 to RS_X31 do
              if r in regs then
                inc(stackcount,4);

            { Float registers }
            fregs:=rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall);
            for r:=RS_F0 to RS_F31 do
              if r in fregs then
                inc(stackcount,8);

            inc(localsize,stackcount);
            if not is_imm12(-localsize) then
              begin
                if not (RS_RETURN_ADDRESS_REG in regs) then
                  begin
                    include(regs,RS_RETURN_ADDRESS_REG);
                    inc(localsize,4);
                  end;
              end;

            stackcount:=0;
            for r:=RS_X0 to RS_X31 do
              if r in regs then
                begin
                  list.concat(taicpu.op_reg_ref(A_SW,newreg(R_INTREGISTER,r,R_SUBWHOLE),href));
                  dec(href.offset,4);
                end;

            { Float registers }
            for r:=RS_F0 to RS_F31 do
              if r in fregs then
                begin
                  list.concat(taicpu.op_reg_ref(A_FSD,newreg(R_FPUREGISTER,r,R_SUBWHOLE),href));
                  dec(href.offset,8);
                end;              

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_FRAME_POINTER_REG,NR_STACK_POINTER_REG,0));

            if localsize>0 then
              begin
                localsize:=align(localsize,4);

                if is_imm12(-localsize) then
                  list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,-localsize))
                else
                  begin
                    a_load_const_reg(list,OS_INT,localsize,NR_RETURN_ADDRESS_REG);
                    list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_RETURN_ADDRESS_REG));
                  end;
              end;
          end;
      end;


    procedure tcgrv32.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
      var
        r: tsuperregister;
        regs, fregs: tcpuregisterset;
        stackcount, localsize: longint;
        href: treference;
      begin
        if not(nostackframe) then
          begin
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_RETURN_ADDRESS_REG];

            if (pi_do_call in current_procinfo.flags) then
              regs:=regs+[RS_RETURN_ADDRESS_REG];

            stackcount:=0;
            reference_reset_base(href,NR_STACK_POINTER_REG,-4,ctempposinvalid,0,[]);
            for r:=RS_X31 downto RS_X0 do
              if r in regs then
                dec(href.offset,4);

            { Float registers }
            fregs:=rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall);
            for r:=RS_F0 to RS_F31 do
              if r in fregs then
                dec(stackcount,8);

            localsize:=current_procinfo.calc_stackframe_size+(-href.offset-4);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_FRAME_POINTER_REG,0))
            else if localsize>0 then
              begin                     
                localsize:=align(localsize,4);

                if is_imm12(localsize) then
                  list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,localsize))
                else
                  begin
                    if not (RS_RETURN_ADDRESS_REG in regs) then
                      begin
                        include(regs,RS_RETURN_ADDRESS_REG);
                        dec(href.offset,4);
                        inc(localsize,4);
                      end;

                    a_load_const_reg(list,OS_INT,localsize,NR_RETURN_ADDRESS_REG);
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_RETURN_ADDRESS_REG));
                  end;
              end;

            { Float registers }
            for r:=RS_F31 downto RS_F0 do
              if r in fregs then
                begin
                  inc(href.offset,8);
                  list.concat(taicpu.op_reg_ref(A_FLD,newreg(R_FPUREGISTER,r,R_SUBWHOLE),href));
                end;

            for r:=RS_X31 downto RS_X0 do
              if r in regs then
                begin
                  inc(href.offset,4);
                  list.concat(taicpu.op_reg_ref(A_LW,newreg(R_INTREGISTER,r,R_SUBWHOLE),href));
                  inc(stackcount);
                end;
          end;

        list.concat(taicpu.op_reg_reg(A_JALR,NR_X0,NR_RETURN_ADDRESS_REG));
      end;


    procedure tcgrv32.g_concatcopy_move(list: tasmlist; const Source, dest: treference; len: tcgint);
      var
        paraloc1, paraloc2, paraloc3: TCGPara;
        pd: tprocdef;
      begin
        pd:=search_system_proc('MOVE');
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
        paramanager.getcgtempparaloc(list, pd, 1, paraloc1);
        paramanager.getcgtempparaloc(list, pd, 2, paraloc2);
        paramanager.getcgtempparaloc(list, pd, 3, paraloc3);
        a_load_const_cgpara(list, OS_SINT, len, paraloc3);
        a_loadaddr_ref_cgpara(list, dest, paraloc2);
        a_loadaddr_ref_cgpara(list, Source, paraloc1);
        paramanager.freecgpara(list, paraloc3);
        paramanager.freecgpara(list, paraloc2);
        paramanager.freecgpara(list, paraloc1);
        alloccpuregisters(list, R_INTREGISTER, paramanager.get_volatile_registers_int(pocall_default));
        alloccpuregisters(list, R_FPUREGISTER, paramanager.get_volatile_registers_fpu(pocall_default));
        a_call_name(list, 'FPC_MOVE', false);
        dealloccpuregisters(list, R_FPUREGISTER, paramanager.get_volatile_registers_fpu(pocall_default));
        dealloccpuregisters(list, R_INTREGISTER, paramanager.get_volatile_registers_int(pocall_default));
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcgrv32.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        tmpreg1, hreg, countreg: TRegister;
        src, dst, src2, dst2: TReference;
        lab:      tasmlabel;
        Count, count2: aint;

        function reference_is_reusable(const ref: treference): boolean;
          begin
            result:=(ref.base<>NR_NO) and (ref.index=NR_NO) and
               (ref.symbol=nil) and
               is_imm12(ref.offset);
          end;

      begin
        src2:=source;
        fixref(list,src2);

        dst2:=dest;
        fixref(list,dst2);

        if len > high(longint) then
          internalerror(2002072704);
        { A call (to FPC_MOVE) requires the outgoing parameter area to be properly
          allocated on stack. This can only be done before tmipsprocinfo.set_first_temp_offset,
          i.e. before secondpass. Other internal procedures request correct stack frame
          by setting pi_do_call during firstpass, but for this particular one it is impossible.
          Therefore, if the current procedure is a leaf one, we have to leave it that way. }

        { anybody wants to determine a good value here :)? }
        if (len > 100) and
           assigned(current_procinfo) and
           (pi_do_call in current_procinfo.flags) then
          g_concatcopy_move(list, src2, dst2, len)
        else
        begin
          Count := len div 4;
          if (count<=4) and reference_is_reusable(src2) then
            src:=src2
          else
            begin
              reference_reset(src,sizeof(aint),[]);
              { load the address of src2 into src.base }
              src.base := GetAddressRegister(list);
              a_loadaddr_ref_reg(list, src2, src.base);
            end;
          if (count<=4) and reference_is_reusable(dst2) then
            dst:=dst2
          else
            begin
              reference_reset(dst,sizeof(aint),[]);
              { load the address of dst2 into dst.base }
              dst.base := GetAddressRegister(list);
              a_loadaddr_ref_reg(list, dst2, dst.base);
            end;
          { generate a loop }
          if Count > 4 then
          begin
            countreg := GetIntRegister(list, OS_INT);
            tmpreg1  := GetIntRegister(list, OS_INT);
            a_load_const_reg(list, OS_INT, Count, countreg);
            current_asmdata.getjumplabel(lab);
            a_label(list, lab);
            list.concat(taicpu.op_reg_ref(A_LW, tmpreg1, src));
            list.concat(taicpu.op_reg_ref(A_SW, tmpreg1, dst));
            list.concat(taicpu.op_reg_reg_const(A_ADDI, src.base, src.base, 4));
            list.concat(taicpu.op_reg_reg_const(A_ADDI, dst.base, dst.base, 4));
            list.concat(taicpu.op_reg_reg_const(A_ADDI, countreg, countreg, -1));
            a_cmp_reg_reg_label(list,OS_INT,OC_GT,NR_X0,countreg,lab);
            len := len mod 4;
          end;
          { unrolled loop }
          Count := len div 4;
          if Count > 0 then
          begin
            tmpreg1 := GetIntRegister(list, OS_INT);
            for count2 := 1 to Count do
            begin
              list.concat(taicpu.op_reg_ref(A_LW, tmpreg1, src));
              list.concat(taicpu.op_reg_ref(A_SW, tmpreg1, dst));
              Inc(src.offset, 4);
              Inc(dst.offset, 4);
            end;
            len := len mod 4;
          end;
          if (len and 4) <> 0 then
          begin
            hreg := GetIntRegister(list, OS_INT);
            a_load_ref_reg(list, OS_32, OS_32, src, hreg);
            a_load_reg_ref(list, OS_32, OS_32, hreg, dst);
            Inc(src.offset, 4);
            Inc(dst.offset, 4);
          end;
          { copy the leftovers }
          if (len and 2) <> 0 then
          begin
            hreg := GetIntRegister(list, OS_INT);
            a_load_ref_reg(list, OS_16, OS_16, src, hreg);
            a_load_reg_ref(list, OS_16, OS_16, hreg, dst);
            Inc(src.offset, 2);
            Inc(dst.offset, 2);
          end;
          if (len and 1) <> 0 then
          begin
            hreg := GetIntRegister(list, OS_INT);
            a_load_ref_reg(list, OS_8, OS_8, src, hreg);
            a_load_reg_ref(list, OS_8, OS_8, hreg, dst);
          end;
        end;
      end;


    procedure tcgrv32.g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef);
      begin

      end;


    procedure tcg64frv.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      var
        tmpreg1: TRegister;
      begin
        case op of
          OP_NOT:
            begin
              cg.a_op_reg_reg(list,OP_NOT,OS_32,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,OP_NOT,OS_32,regsrc.reghi,regdst.reghi);
            end;
          OP_NEG:
            begin
              tmpreg1 := cg.GetIntRegister(list, OS_INT);
              list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reglo, NR_X0, regsrc.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SLTU, tmpreg1, NR_X0, regdst.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reghi, NR_X0, regsrc.reghi));
              list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reghi, regdst.reghi, tmpreg1));
            end;
          else
            a_op64_reg_reg_reg(list,op,size,regsrc,regdst,regdst);
        end;
      end;


    procedure tcg64frv.a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);
      begin
        a_op64_const_reg_reg(list,op,size,value,reg,reg);
      end;


    procedure tcg64frv.a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);
      var
        signed: Boolean;
        tmplo, carry, tmphi, hreg: TRegister;
      begin
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reglo,regsrc2.reglo,regdst.reglo);
              cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reghi,regsrc2.reghi,regdst.reghi);
            end;
          OP_ADD:
            begin
              signed:=(size in [OS_S64]);
              tmplo := cg.GetIntRegister(list,OS_S32);
              carry := cg.GetIntRegister(list,OS_S32);
              // destreg.reglo could be regsrc1.reglo or regsrc2.reglo
              list.concat(taicpu.op_reg_reg_reg(A_ADD, tmplo, regsrc2.reglo, regsrc1.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SLTU, carry, tmplo, regsrc2.reglo));
              cg.a_load_reg_reg(list,OS_INT,OS_INT,tmplo,regdst.reglo);
              if signed then
                begin
                  list.concat(taicpu.op_reg_reg_reg(A_ADD, regdst.reghi, regsrc2.reghi, regsrc1.reghi));
                  list.concat(taicpu.op_reg_reg_reg(A_ADD, regdst.reghi, regdst.reghi, carry));
                end
              else
                begin
                  tmphi:=cg.GetIntRegister(list,OS_INT);
                  hreg:=cg.GetIntRegister(list,OS_INT);
                  cg.a_load_const_reg(list,OS_INT,$80000000,hreg);
                  // first add carry to one of the addends
                  list.concat(taicpu.op_reg_reg_reg(A_ADD, tmphi, regsrc2.reghi, carry));
                  list.concat(taicpu.op_reg_reg_reg(A_SLTU, carry, tmphi, regsrc2.reghi));
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, carry, hreg, carry));
                  // then add another addend
                  list.concat(taicpu.op_reg_reg_reg(A_ADD, regdst.reghi, tmphi, regsrc1.reghi));
                  list.concat(taicpu.op_reg_reg_reg(A_SLTU, carry, regdst.reghi, tmphi));
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, carry, hreg, carry));
                end;
            end;
          OP_SUB:
            begin
              signed:=(size in [OS_S64]);
              tmplo := cg.GetIntRegister(list,OS_S32);
              carry := cg.GetIntRegister(list,OS_S32);
              // destreg.reglo could be regsrc1.reglo or regsrc2.reglo
              list.concat(taicpu.op_reg_reg_reg(A_SUB, tmplo, regsrc2.reglo, regsrc1.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SLTU, carry, regsrc2.reglo,tmplo));
              cg.a_load_reg_reg(list,OS_INT,OS_INT,tmplo,regdst.reglo);
              if signed then
                begin
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reghi, regsrc2.reghi, regsrc1.reghi));
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reghi, regdst.reghi, carry));
                end
              else
                begin
                  tmphi:=cg.GetIntRegister(list,OS_INT);
                  hreg:=cg.GetIntRegister(list,OS_INT);
                  cg.a_load_const_reg(list,OS_INT,$80000000,hreg);
                  // first subtract the carry...
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, tmphi, regsrc2.reghi, carry));
                  list.concat(taicpu.op_reg_reg_reg(A_SLTU, carry, regsrc2.reghi, tmphi));
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, carry, hreg, carry));
                  // ...then the subtrahend
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reghi, tmphi, regsrc1.reghi));
                  list.concat(taicpu.op_reg_reg_reg(A_SLTU, carry, tmphi, regdst.reghi));
                  list.concat(taicpu.op_reg_reg_reg(A_SUB, carry, hreg, carry));
                end;
            end;
          else
            internalerror(2002072801);
        end;
      end;


    procedure tcg64frv.a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);
      var
        tmplo,carry: TRegister;
        hisize: tcgsize;
      begin
        carry:=NR_NO;
        if (size in [OS_S64]) then
          hisize:=OS_S32
        else
          hisize:=OS_32;

        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg_reg(list,op,OS_32,aint(lo(value)),regsrc.reglo,regdst.reglo);
              cg.a_op_const_reg_reg(list,op,OS_32,aint(hi(value)),regsrc.reghi,regdst.reghi);
            end;

          OP_ADD:
            begin
              if lo(value)<>0 then
                begin
                  tmplo:=cg.GetIntRegister(list,OS_32);
                  carry:=cg.GetIntRegister(list,OS_32);

                  if is_imm12(aint(lo(value))) then
                    list.concat(taicpu.op_reg_reg_const(A_ADDI,tmplo,regsrc.reglo,aint(lo(value))))
                  else
                    begin
                      cg.a_load_const_reg(list,OS_INT,aint(lo(value)),tmplo);
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmplo,tmplo,regsrc.reglo))
                    end;
                  list.concat(taicpu.op_reg_reg_reg(A_SLTU,carry,tmplo,regsrc.reglo));
                  cg.a_load_reg_reg(list,OS_32,OS_32,tmplo,regdst.reglo);
                end
              else
                cg.a_load_reg_reg(list,OS_32,OS_32,regsrc.reglo,regdst.reglo);

              { With overflow checking and unsigned args, this generates slighly suboptimal code
               ($80000000 constant loaded twice). Other cases are fine. Getting it perfect does not
               look worth the effort. }
              cg.a_op_const_reg_reg(list,OP_ADD,hisize,aint(hi(value)),regsrc.reghi,regdst.reghi);
              if carry<>NR_NO then
                cg.a_op_reg_reg_reg(list,OP_ADD,hisize,carry,regdst.reghi,regdst.reghi);
            end;

          OP_SUB:
            begin
              carry:=NR_NO;
              if lo(value)<>0 then
                begin
                  tmplo:=cg.GetIntRegister(list,OS_32);
                  carry:=cg.GetIntRegister(list,OS_32);

                  if is_imm12(-aint(lo(value))) then
                    list.concat(taicpu.op_reg_reg_const(A_ADDI,tmplo,regsrc.reglo,-aint(lo(value))))
                  else
                    begin
                      cg.a_load_const_reg(list,OS_INT,aint(lo(value)),tmplo);
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmplo,tmplo,regsrc.reglo))
                    end;
                  list.concat(taicpu.op_reg_reg_reg(A_SLTU,carry,regsrc.reglo,tmplo));
                  cg.a_load_reg_reg(list,OS_32,OS_32,tmplo,regdst.reglo);
                end
              else
                cg.a_load_reg_reg(list,OS_32,OS_32,regsrc.reglo,regdst.reglo);

              cg.a_op_const_reg_reg(list,OP_SUB,hisize,aint(hi(value)),regsrc.reghi,regdst.reghi);
              if carry<>NR_NO then
                cg.a_op_reg_reg_reg(list,OP_SUB,hisize,carry,regdst.reghi,regdst.reghi);
            end;
        else
          InternalError(2013050301);
        end;
      end;


    procedure create_codegen;
      begin
        cg := tcgrv32.create;
        cg64 :=tcg64frv.create;
      end;

end.
