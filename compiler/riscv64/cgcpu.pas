{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the code generator for the RiscV64

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

{$I fpcdefs.inc}

  interface

    uses
      globtype, symtype, symdef, symsym,
      cgbase, cgobj,cgrv,
      aasmbase, aasmcpu, aasmtai,aasmdata,
      cpubase, cpuinfo, cgutils, rgcpu,
      parabase;

    type
      tcgrv64 = class(tcgrv)
        procedure init_register_allocators; override;
        procedure done_register_allocators; override;

        { move instructions }
        procedure a_load_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;     
        procedure a_load_const_reg(list: TAsmList; size: tcgsize; a: tcgint; register: tregister); override;

        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister; setflags: boolean; var ovloc: tlocation); override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister; setflags: boolean; var ovloc: tlocation); override;

        procedure g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef); override;

        procedure g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean); override;
        procedure g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean); override;

        procedure g_concatcopy_move(list: tasmlist; const Source, dest: treference; len: tcgint);
        procedure g_concatcopy(list: TAsmList; const source, dest: treference; len: aint); override;
      end;

    procedure create_codegen;

implementation

    uses
      sysutils, cclasses,
      globals, verbose, systems, cutils,
      symconst, fmodule, symtable,
      rgobj, tgobj, cpupi, procinfo, paramgr, cpupara;

{ Range check must be disabled explicitly as conversions between signed and unsigned
  64-bit and 32-bit values are done without explicit typecasts }
{$R-}

    procedure tcgrv64.init_register_allocators;
      begin
        inherited init_register_allocators;
        rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
          [RS_X10,RS_X11,RS_X12,RS_X13,RS_X14,RS_X15,RS_X16,RS_X17,
           RS_X31,RS_X30,RS_X29,RS_X28,
           RS_X5,RS_X6,RS_X7,
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


    procedure tcgrv64.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        inherited done_register_allocators;
      end;


    procedure tcgrv64.a_load_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);
      var
        ai: taicpu;
      begin
        list.concat(tai_comment.Create(strpnew('Move '+tcgsize2str(fromsize)+'->'+tcgsize2str(tosize))));

        if (tcgsize2unsigned[tosize]=OS_64) and (fromsize=OS_S32) then
          list.Concat(taicpu.op_reg_reg_const(A_ADDIW,reg2,reg1,0))
        else if (tosize=OS_S32) and (tcgsize2unsigned[fromsize]=OS_64) then
          list.Concat(taicpu.op_reg_reg_const(A_ADDIW,reg2,reg1,0))
        else if (tosize=OS_S32) and (fromsize=OS_32) then
          list.Concat(taicpu.op_reg_reg_const(A_ADDIW,reg2,reg1,0))
        else if (tcgsize2unsigned[tosize]=OS_64) and (fromsize=OS_8) then
          list.Concat(taicpu.op_reg_reg_const(A_ANDI,reg2,reg1,$FF))
        else if (tcgsize2size[fromsize] > tcgsize2size[tosize]) or
          ((tcgsize2size[fromsize] = tcgsize2size[tosize]) and (fromsize <> tosize)) or
          { do we need to mask out the sign when loading from smaller signed to larger unsigned type? }
          ((tcgsize2unsigned[fromsize]<>fromsize) and ((tcgsize2unsigned[tosize]=tosize)) and
            (tcgsize2size[fromsize] < tcgsize2size[tosize]) and (tcgsize2size[tosize] <> sizeof(pint)) ) then
          begin
            if tcgsize2size[fromsize]<tcgsize2size[tosize] then
              begin
                list.Concat(taicpu.op_reg_reg_const(A_SLLI,reg2,reg1,8*(8-tcgsize2size[fromsize])));

                if tcgsize2unsigned[fromsize]<>fromsize then
                  list.Concat(taicpu.op_reg_reg_const(A_SRAI,reg2,reg2,8*(tcgsize2size[tosize]-tcgsize2size[fromsize])))
                else
                  list.Concat(taicpu.op_reg_reg_const(A_SRLI,reg2,reg2,8*(tcgsize2size[tosize]-tcgsize2size[fromsize])));
              end
            else if tcgsize2unsigned[tosize]<>OS_64 then
              list.Concat(taicpu.op_reg_reg_const(A_SLLI,reg2,reg1,8*(8-tcgsize2size[tosize])))
            else
              a_load_reg_reg(list,tosize,tosize,reg1,reg2);

            if tcgsize2unsigned[tosize]=tosize then
              list.Concat(taicpu.op_reg_reg_const(A_SRLI,reg2,reg2,8*(8-tcgsize2size[tosize])))
            else
              list.Concat(taicpu.op_reg_reg_const(A_SRAI,reg2,reg2,8*(8-tcgsize2size[tosize])));
          end
        else
          begin
            ai:=taicpu.op_reg_reg_const(A_ADDI,reg2,reg1,0);
            list.concat(ai);
            rg[R_INTREGISTER].add_move_instruction(ai);
          end;
      end;

    procedure tcgrv64.a_load_const_reg(list: TAsmList; size: tcgsize; a: tcgint; register: tregister);
      var
        l: TAsmLabel;
        hr: treference;
      begin
        if a=0 then
          a_load_reg_reg(list,size,size,NR_X0,register)
        else
          begin
            if is_imm12(a) then
              list.concat(taicpu.op_reg_reg_const(A_ADDI,register,NR_X0,a))
            else if is_lui_imm(a) then
              list.concat(taicpu.op_reg_const(A_LUI,register,(a shr 12) and $FFFFF))
            else if (int64(longint(a))=a) then
              begin
                if (a and $800)<>0 then
                  list.concat(taicpu.op_reg_const(A_LUI,register,((a shr 12)+1) and $FFFFF))
                else
                  list.concat(taicpu.op_reg_const(A_LUI,register,(a shr 12) and $FFFFF));

                list.concat(taicpu.op_reg_reg_const(A_ADDIW,register,register,SarSmallint(smallint(a shl 4),4)));
              end
            else
              begin
                reference_reset(hr,8,[]);

                current_asmdata.getjumplabel(l);
                current_procinfo.aktlocaldata.Concat(cai_align.Create(8));
                cg.a_label(current_procinfo.aktlocaldata,l);
                hr.symboldata:=current_procinfo.aktlocaldata.last;
                current_procinfo.aktlocaldata.concat(tai_const.Create_64bit(a));

                hr.symbol:=l;
                hr.refaddr:=addr_pcrel_hi20;

                current_asmdata.getjumplabel(l);
                a_label(list,l);

                list.concat(taicpu.op_reg_ref(A_AUIPC,register,hr));

                reference_reset_symbol(hr,l,0,0,[]);
                hr.refaddr:=addr_pcrel_lo12;
                hr.base:=register;
                list.concat(taicpu.op_reg_ref(A_LD,register,hr));
              end;
          end;
      end;


    procedure tcgrv64.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister; setflags: boolean; var ovloc: tlocation);
      var
        signed: Boolean;
        l: TAsmLabel;
        tmpreg: tregister;
        ai: taicpu;
      begin
        if setflags then
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            a_op_reg_reg_reg_checkoverflow(list,op,size,tmpreg,src,dst,setflags,ovloc);
          end
        else
          a_op_const_reg_reg(list,op,size,a,src,dst);
      end;


    procedure tcgrv64.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister; setflags: boolean; var ovloc: tlocation);
        var
        signed: Boolean;
        l: TAsmLabel;
        tmpreg, tmpreg0: tregister;
        ai: taicpu;
      begin
        signed:=tcgsize2unsigned[size]<>size;

        if setflags then
          case op of
            OP_ADD:
              begin
                current_asmdata.getjumplabel(l);

                list.Concat(taicpu.op_reg_reg_reg(A_ADD,dst,src2,src1));

                if signed then
                  begin
                    {
                      t0=src1<0
                      t1=result<src2
                      overflow if t0<>t1
                    }
                    tmpreg0:=getintregister(list,OS_INT);
                    tmpreg:=getintregister(list,OS_INT);
                    list.Concat(taicpu.op_reg_reg_reg(A_SLT,tmpreg0,src1,NR_X0));
                    list.Concat(taicpu.op_reg_reg_reg(A_SLT,tmpreg,dst,src2));

                    ai:=taicpu.op_reg_reg_sym_ofs(A_Bxx,tmpreg,tmpreg0,l,0);
                    ai.condition:=C_EQ;
                    list.concat(ai);
                  end
                else
                  begin
                    {
                      jump if sum>=x
                    }
                    if size in [OS_S32,OS_32] then
                      begin
                        tmpreg:=getintregister(list,OS_INT);
                        a_load_reg_reg(list,size,OS_64,dst,tmpreg);
                        dst:=tmpreg;
                      end;

                    ai:=taicpu.op_reg_reg_sym_ofs(A_Bxx,dst,src2,l,0);
                    ai.condition:=C_GEU;
                    list.concat(ai);
                  end;

                a_call_name(list,'FPC_OVERFLOW',false);
                a_label(list,l);
              end;
            OP_SUB:
              begin
                current_asmdata.getjumplabel(l);

                list.Concat(taicpu.op_reg_reg_reg(A_SUB,dst,src2,src1));

                if signed then
                  begin
                    tmpreg0:=getintregister(list,OS_INT);
                    tmpreg:=getintregister(list,OS_INT);
                    list.Concat(taicpu.op_reg_reg_reg(A_SLT,tmpreg0,NR_X0,src1));
                    list.Concat(taicpu.op_reg_reg_reg(A_SLT,tmpreg,dst,src2));

                    ai:=taicpu.op_reg_reg_sym_ofs(A_Bxx,tmpreg,tmpreg0,l,0);
                    ai.condition:=C_EQ;
                    list.concat(ai);
                  end
                else
                  begin
                    { no overflow if result<=src2 }
                    if size in [OS_S32,OS_32] then
                      begin
                        tmpreg:=getintregister(list,OS_INT);
                        a_load_reg_reg(list,size,OS_64,dst,tmpreg);
                        dst:=tmpreg;
                      end;

                    ai:=taicpu.op_reg_reg_sym_ofs(A_Bxx,src2,dst,l,0);
                    ai.condition:=C_GEU;
                    list.concat(ai);
                  end;

                a_call_name(list,'FPC_OVERFLOW',false);
                a_label(list,l);
              end;
            OP_IMUL:
              begin
                { No overflow if upper result is same as sign of result }
                current_asmdata.getjumplabel(l);

                tmpreg:=getintregister(list,OS_INT);
                tmpreg0:=getintregister(list,OS_INT);
                list.Concat(taicpu.op_reg_reg_reg(A_MUL,dst,src1,src2));
                list.Concat(taicpu.op_reg_reg_reg(A_MULH,tmpreg,src1,src2));

                list.concat(taicpu.op_reg_reg_const(A_SRAI,tmpreg0,dst,63));

                a_cmp_reg_reg_label(list,OS_INT,OC_EQ,tmpreg,tmpreg0,l);

                a_call_name(list,'FPC_OVERFLOW',false);
                a_label(list,l);
              end;
            OP_MUL:
              begin
                { No overflow if upper result is 0 }
                current_asmdata.getjumplabel(l);

                tmpreg:=getintregister(list,OS_INT);
                list.Concat(taicpu.op_reg_reg_reg(A_MUL,dst,src1,src2));
                list.Concat(taicpu.op_reg_reg_reg(A_MULHU,tmpreg,src1,src2));

                a_cmp_reg_reg_label(list,OS_INT,OC_EQ,tmpreg,NR_X0,l);

                a_call_name(list,'FPC_OVERFLOW',false);
                a_label(list,l);
              end;
            OP_IDIV:
              begin
                { Only overflow if dst is all 1's }
                current_asmdata.getjumplabel(l);

                tmpreg:=getintregister(list,OS_INT);
                list.Concat(taicpu.op_reg_reg_reg(A_DIV,dst,src1,src2));
                list.Concat(taicpu.op_reg_reg_const(A_ADDI,tmpreg,dst,1));

                a_cmp_reg_reg_label(list,OS_INT,OC_NE,tmpreg,NR_X0,l);

                a_call_name(list,'FPC_OVERFLOW',false);
                a_label(list,l);
              end;
            else
              internalerror(2019051032);
          end
        else
          a_op_reg_reg_reg(list,op,size,src1,src2,dst);
      end;


    procedure tcgrv64.g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef);
      begin
      end;


    procedure tcgrv64.g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);
      var
        regs, fregs: tcpuregisterset;
        r: TSuperRegister;
        href: treference;
        stackcount, stackAdjust: longint;
      begin
        if not(nostackframe) then
          begin
            a_reg_alloc(list,NR_STACK_POINTER_REG);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              a_reg_alloc(list,NR_FRAME_POINTER_REG);

            reference_reset_base(href,NR_STACK_POINTER_REG,-8,ctempposinvalid,0,[]);

            { Int registers }
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_RETURN_ADDRESS_REG];

            if (pi_do_call in current_procinfo.flags) then
              regs:=regs+[RS_RETURN_ADDRESS_REG];

            stackcount:=0;
            for r:=RS_X0 to RS_X31 do
              if r in regs then
                inc(stackcount,8);

            { Float registers }
            fregs:=rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall);
            for r:=RS_F0 to RS_F31 do
              if r in fregs then
                inc(stackcount,8);  

            inc(localsize,stackcount);
            if not is_imm12(-(localsize-stackcount)) then
              begin
                if not (RS_RETURN_ADDRESS_REG in regs) then
                  begin
                    include(regs,RS_RETURN_ADDRESS_REG);
                    inc(localsize,8);
                    inc(stackcount,8);
                  end;
              end;

            stackAdjust:=0;
            if (CPURV_HAS_COMPACT in cpu_capabilities[current_settings.cputype]) and
               (stackcount>0) then
              begin
                list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,-stackcount));
                inc(href.offset,stackcount);
                stackAdjust:=stackcount;
                dec(localsize,stackcount);
              end;

            for r:=RS_X0 to RS_X31 do
              if r in regs then
                begin
                  list.concat(taicpu.op_reg_ref(A_SD,newreg(R_INTREGISTER,r,R_SUBWHOLE),href));
                  dec(href.offset,8);
                end;

            { Float registers }
            for r:=RS_F0 to RS_F31 do
              if r in fregs then
                begin
                  list.concat(taicpu.op_reg_ref(A_FSD,newreg(R_FPUREGISTER,r,R_SUBWHOLE),href));
                  dec(href.offset,8);
                end;   

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_FRAME_POINTER_REG,NR_STACK_POINTER_REG,stackAdjust));

            if localsize>0 then
              begin
                localsize:=align(localsize,8);

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


    procedure tcgrv64.g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean);
      var
        r: tsuperregister;
        regs, fregs: tcpuregisterset;
        localsize: longint;
        href: treference;
      begin
        if not(nostackframe) then
          begin
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_RETURN_ADDRESS_REG];

            if (pi_do_call in current_procinfo.flags) then
              regs:=regs+[RS_RETURN_ADDRESS_REG];

            reference_reset_base(href,NR_STACK_POINTER_REG,-8,ctempposinvalid,0,[]);
            for r:=RS_X31 downto RS_X0 do
              if r in regs then
                dec(href.offset,8);

            { Float registers }
            fregs:=rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall);
            for r:=RS_F0 to RS_F31 do
              if r in fregs then
                dec(href.offset,8);

            localsize:=current_procinfo.calc_stackframe_size+(-href.offset-8);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_FRAME_POINTER_REG,0))
            else if localsize>0 then
              begin                 
                localsize:=align(localsize,8);

                if is_imm12(localsize) then
                  list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,localsize))
                else
                  begin
                    if not (RS_RETURN_ADDRESS_REG in regs) then
                      begin
                        include(regs,RS_RETURN_ADDRESS_REG);
                        dec(href.offset,8);
                        inc(localsize,8);
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
                  inc(href.offset,8);
                  list.concat(taicpu.op_reg_ref(A_LD,newreg(R_INTREGISTER,r,R_SUBWHOLE),href));
                end;
          end;

        list.concat(taicpu.op_reg_reg(A_JALR,NR_X0,NR_RETURN_ADDRESS_REG));
      end;


    procedure tcgrv64.g_concatcopy_move(list: tasmlist; const Source, dest: treference; len: tcgint);
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

    procedure tcgrv64.g_concatcopy(list: TAsmList; const source, dest: treference; len: aint);
      var
        tmpreg1, hreg, countreg: TRegister;
        src, dst, src2, dst2: TReference;
        lab:      tasmlabel;
        Count, count2: aint;
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
          Count := len div 8;
          reference_reset(src,sizeof(aint),[]);
          { load the address of src2 into src.base }
          src.base := GetAddressRegister(list);
          a_loadaddr_ref_reg(list, src2, src.base);

          reference_reset(dst,sizeof(aint),[]);
          { load the address of dst2 into dst.base }
          dst.base := GetAddressRegister(list);
          a_loadaddr_ref_reg(list, dst2, dst.base);

          { generate a loop }
          if Count > 4 then
          begin
            countreg := GetIntRegister(list, OS_INT);
            tmpreg1  := GetIntRegister(list, OS_INT);
            a_load_const_reg(list, OS_INT, Count, countreg);
            current_asmdata.getjumplabel(lab);
            a_label(list, lab);
            list.concat(taicpu.op_reg_ref(A_LD, tmpreg1, src));
            list.concat(taicpu.op_reg_ref(A_SD, tmpreg1, dst));
            list.concat(taicpu.op_reg_reg_const(A_ADDI, src.base, src.base, 8));
            list.concat(taicpu.op_reg_reg_const(A_ADDI, dst.base, dst.base, 8));
            list.concat(taicpu.op_reg_reg_const(A_ADDI, countreg, countreg, -1));
            a_cmp_reg_reg_label(list,OS_INT,OC_GT,NR_X0,countreg,lab);
            len := len mod 8;
          end;
          { unrolled loop }
          Count := len div 8;
          if Count > 0 then
          begin
            tmpreg1 := GetIntRegister(list, OS_INT);
            count2 := 1;
            while count2 <= Count do
              begin
                list.concat(taicpu.op_reg_ref(A_LD, tmpreg1, src));
                list.concat(taicpu.op_reg_ref(A_SD, tmpreg1, dst));
                Inc(src.offset, 8);
                Inc(dst.offset, 8);
                Inc(count2);
              end;
            len := len mod 8;
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

procedure create_codegen;
begin
  cg := tcgrv64.create;
  cg128:=tcg128.create;
end;

end.
