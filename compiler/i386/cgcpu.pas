{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the code generator for the i386

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
       globtype,
       cgbase,cgobj,cg64f32,cgx86,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,parabase,cgutils,
       symconst,symdef,symsym
       ;

    type
      tcg386 = class(tcgx86)
        procedure init_register_allocators;override;

        { passing parameter using push instead of mov }
        procedure a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : tcgpara);override;
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : tcgpara);override;
        procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : tcgpara);override;
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : tcgpara);override;

        procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);override;
        procedure g_copyvaluepara_openarray(list : TAsmList;const ref:treference;const lenloc:tlocation;elesize:tcgint;destreg:tregister);
        procedure g_releasevaluepara_openarray(list : TAsmList;const l:tlocation);

        procedure g_maybe_got_init(list: TAsmList); override;
     end;

      tcg64f386 = class(tcg64f32)
        procedure a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);override;
        procedure a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64; const ref: treference);override;
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
        procedure a_op64_const_ref(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const ref : treference);override;
      private
        procedure get_64bit_ops(op:TOpCG;var op1,op2:TAsmOp);
      end;

    procedure create_codegen;

  implementation

    uses
       globals,verbose,systems,cutils,
       paramgr,procinfo,fmodule,
       rgcpu,rgx86,cpuinfo;

    function use_push(const cgpara:tcgpara):boolean;
      begin
        result:=(not paramanager.use_fixed_stack) and
                assigned(cgpara.location) and
                (cgpara.location^.loc=LOC_REFERENCE) and
                (cgpara.location^.reference.index=NR_STACK_POINTER_REG);
      end;


    procedure tcg386.init_register_allocators;
      begin
        inherited init_register_allocators;
        if (cs_useebp in current_settings.optimizerswitches) and assigned(current_procinfo) and (current_procinfo.framepointer<>NR_EBP) then
          rg[R_INTREGISTER]:=trgcpu.create(R_INTREGISTER,R_SUBWHOLE,[RS_EAX,RS_EDX,RS_ECX,RS_EBX,RS_ESI,RS_EDI,RS_EBP],first_int_imreg,[])
        else
          rg[R_INTREGISTER]:=trgcpu.create(R_INTREGISTER,R_SUBWHOLE,[RS_EAX,RS_EDX,RS_ECX,RS_EBX,RS_ESI,RS_EDI],first_int_imreg,[RS_EBP]);
        rg[R_MMXREGISTER]:=trgcpu.create(R_MMXREGISTER,R_SUBNONE,[RS_XMM0,RS_XMM1,RS_XMM2,RS_XMM3,RS_XMM4,RS_XMM5,RS_XMM6,RS_XMM7],first_mm_imreg,[]);
        rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBWHOLE,[RS_XMM0,RS_XMM1,RS_XMM2,RS_XMM3,RS_XMM4,RS_XMM5,RS_XMM6,RS_XMM7],first_mm_imreg,[]);
        rgfpu:=Trgx86fpu.create;
      end;


    procedure tcg386.a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : tcgpara);
      var
        pushsize : tcgsize;
      begin
        check_register_size(size,r);
        if use_push(cgpara) then
          begin
            cgpara.check_simple_location;
            if tcgsize2size[cgpara.location^.size]>cgpara.alignment then
              pushsize:=cgpara.location^.size
            else
              pushsize:=int_cgsize(cgpara.alignment);
            list.concat(taicpu.op_reg(A_PUSH,tcgsize2opsize[pushsize],makeregsize(list,r,pushsize)));
          end
        else
          inherited a_load_reg_cgpara(list,size,r,cgpara);
      end;


    procedure tcg386.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : tcgpara);
      var
        pushsize : tcgsize;
      begin
        if use_push(cgpara) then
          begin
            cgpara.check_simple_location;
            if tcgsize2size[cgpara.location^.size]>cgpara.alignment then
              pushsize:=cgpara.location^.size
            else
              pushsize:=int_cgsize(cgpara.alignment);
            list.concat(taicpu.op_const(A_PUSH,tcgsize2opsize[pushsize],a));
          end
        else
          inherited a_load_const_cgpara(list,size,a,cgpara);
      end;


    procedure tcg386.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : tcgpara);

        procedure pushdata(paraloc:pcgparalocation;ofs:tcgint);
        var
          pushsize : tcgsize;
          opsize : topsize;
          tmpreg   : tregister;
          href     : treference;
        begin
          if not assigned(paraloc) then
            exit;
          if (paraloc^.loc<>LOC_REFERENCE) or
             (paraloc^.reference.index<>NR_STACK_POINTER_REG) or
             (tcgsize2size[paraloc^.size]>sizeof(aint)) then
            internalerror(200501162);
          { Pushes are needed in reverse order, add the size of the
            current location to the offset where to load from. This
            prevents wrong calculations for the last location when
            the size is not a power of 2 }
          if assigned(paraloc^.next) then
            pushdata(paraloc^.next,ofs+tcgsize2size[paraloc^.size]);
          { Push the data starting at ofs }
          href:=r;
          inc(href.offset,ofs);
          if tcgsize2size[paraloc^.size]>cgpara.alignment then
            pushsize:=paraloc^.size
          else
            pushsize:=int_cgsize(cgpara.alignment);
          opsize:=TCgsize2opsize[pushsize];
          { for go32v2 we obtain OS_F32,
            but pushs is not valid, we need pushl }
          if opsize=S_FS then
            opsize:=S_L;
          if tcgsize2size[paraloc^.size]<cgpara.alignment then
            begin
              tmpreg:=getintregister(list,pushsize);
              a_load_ref_reg(list,paraloc^.size,pushsize,href,tmpreg);
              list.concat(taicpu.op_reg(A_PUSH,opsize,tmpreg));
            end
          else
            begin
              make_simple_ref(list,href);
              list.concat(taicpu.op_ref(A_PUSH,opsize,href));
            end;
        end;

      var
        len : tcgint;
        href : treference;
      begin
        { cgpara.size=OS_NO requires a copy on the stack }
        if use_push(cgpara) then
          begin
            { Record copy? }
            if (cgpara.size=OS_NO) or (size=OS_NO) then
              begin
                cgpara.check_simple_location;
                len:=align(cgpara.intsize,cgpara.alignment);
                g_stackpointer_alloc(list,len);
                reference_reset_base(href,NR_STACK_POINTER_REG,0,ctempposinvalid,4,[]);
                g_concatcopy(list,r,href,len);
              end
            else
              begin
                if tcgsize2size[cgpara.size]<>tcgsize2size[size] then
                  internalerror(200501161);
                if (cgpara.size=OS_F64) then
                  begin
                    href:=r;
                    make_simple_ref(list,href);
                    inc(href.offset,4);
                    list.concat(taicpu.op_ref(A_PUSH,S_L,href));
                    dec(href.offset,4);
                    list.concat(taicpu.op_ref(A_PUSH,S_L,href));
                  end
                else
                  { We need to push the data in reverse order,
                    therefor we use a recursive algorithm }
                  pushdata(cgpara.location,0);
              end
          end
        else
          begin
            href:=r;
            make_simple_ref(list,href);
            inherited a_load_ref_cgpara(list,size,href,cgpara);
          end;
      end;


    procedure tcg386.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : tcgpara);
      var
        tmpreg : tregister;
        opsize : topsize;
        tmpref,dirref : treference;
      begin
        dirref:=r;

        { this could probably done in a more optimized way, but for now this
          is sufficent }
        make_direct_ref(list,dirref);

        with dirref do
          begin
            if use_push(cgpara) then
              begin
                cgpara.check_simple_location;
                opsize:=tcgsize2opsize[OS_ADDR];
                if (segment=NR_NO) and (base=NR_NO) and (index=NR_NO) then
                  begin
                    if assigned(symbol) then
                      begin
                        if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) and
                           ((dirref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL]) or
                            (cs_create_pic in current_settings.moduleswitches)) then
                          begin
                            tmpreg:=getaddressregister(list);
                            a_loadaddr_ref_reg(list,dirref,tmpreg);
                            list.concat(taicpu.op_reg(A_PUSH,opsize,tmpreg));
                          end
                        else if cs_create_pic in current_settings.moduleswitches then
                          begin
                            if offset<>0 then
                              begin
                                tmpreg:=getaddressregister(list);
                                a_loadaddr_ref_reg(list,dirref,tmpreg);
                                list.concat(taicpu.op_reg(A_PUSH,opsize,tmpreg));
                              end
                            else
                              begin
                                reference_reset_symbol(tmpref,dirref.symbol,0,sizeof(pint),[]);
                                tmpref.refaddr:=addr_pic;
                                tmpref.base:=current_procinfo.got;
                                include(current_procinfo.flags,pi_needs_got);
                                list.concat(taicpu.op_ref(A_PUSH,S_L,tmpref));
                              end
                          end
                        else
                          list.concat(Taicpu.Op_sym_ofs(A_PUSH,opsize,symbol,offset));
                      end
                    else
                      list.concat(Taicpu.Op_const(A_PUSH,opsize,offset));
                  end
                else if (segment=NR_NO) and (base=NR_NO) and (index<>NR_NO) and
                        (offset=0) and (scalefactor=0) and (symbol=nil) then
                  list.concat(Taicpu.Op_reg(A_PUSH,opsize,index))
                else if (segment=NR_NO) and (base<>NR_NO) and (index=NR_NO) and
                        (offset=0) and (symbol=nil) then
                  list.concat(Taicpu.Op_reg(A_PUSH,opsize,base))
                else
                  begin
                    tmpreg:=getaddressregister(list);
                    a_loadaddr_ref_reg(list,dirref,tmpreg);
                    list.concat(taicpu.op_reg(A_PUSH,opsize,tmpreg));
                  end;
              end
            else
              inherited a_loadaddr_ref_cgpara(list,dirref,cgpara);
          end;
      end;


    procedure tcg386.g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);

      procedure increase_sp(a : tcgint);
        var
          href : treference;
        begin
          reference_reset_base(href,NR_STACK_POINTER_REG,a,ctempposinvalid,0,[]);
          { normally, lea is a better choice than an add }
          list.concat(Taicpu.op_ref_reg(A_LEA,TCGSize2OpSize[OS_ADDR],href,NR_STACK_POINTER_REG));
        end;

      begin
        { MMX needs to call EMMS }
        if assigned(rg[R_MMXREGISTER]) and
           (rg[R_MMXREGISTER].uses_registers) then
          list.concat(Taicpu.op_none(A_EMMS,S_NO));

        { remove stackframe }
        if not(nostackframe) and
          { we do not need an exit stack frame when we never return

            * the final ret is left so the peephole optimizer can easily do call/ret -> jmp or call conversions
            * the entry stack frame must be normally generated because the subroutine could be still left by
              an exception and then the unwinding code might need to restore the registers stored by the entry code
          }
          not(po_noreturn in current_procinfo.procdef.procoptions) then
          begin
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) or
               (current_procinfo.procdef.proctypeoption=potype_exceptfilter) then
              begin
                if current_procinfo.final_localsize<>0 then
                  increase_sp(current_procinfo.final_localsize);
                if (not paramanager.use_fixed_stack) then
                  internal_restore_regs(list,true);
                if (current_procinfo.procdef.proctypeoption=potype_exceptfilter) then
                  list.concat(Taicpu.op_reg(A_POP,tcgsize2opsize[OS_ADDR],NR_FRAME_POINTER_REG));
                current_asmdata.asmcfi.cfa_def_cfa_offset(list,sizeof(pint));
              end
            else
              begin
                if (not paramanager.use_fixed_stack) then
                  internal_restore_regs(list,not (pi_has_stack_allocs in current_procinfo.flags));
                generate_leave(list);
              end;
            list.concat(tai_regalloc.dealloc(current_procinfo.framepointer,nil));
          end;

        { return from proc }
        if po_interrupt in current_procinfo.procdef.procoptions then
          begin
            if assigned(current_procinfo.procdef.funcretloc[calleeside].location) and
               (current_procinfo.procdef.funcretloc[calleeside].location^.loc=LOC_REGISTER) then
              begin
                if (getsupreg(current_procinfo.procdef.funcretloc[calleeside].location^.register)=RS_EAX) then
                  list.concat(Taicpu.Op_const_reg(A_ADD,S_L,4,NR_ESP))
                else
                  internalerror(2010053001);
              end
            else
              list.concat(Taicpu.Op_reg(A_POP,S_L,NR_EAX));
            list.concat(Taicpu.Op_reg(A_POP,S_L,NR_EBX));
            list.concat(Taicpu.Op_reg(A_POP,S_L,NR_ECX));

            if (current_procinfo.procdef.funcretloc[calleeside].size in [OS_64,OS_S64]) and
               assigned(current_procinfo.procdef.funcretloc[calleeside].location) and
               assigned(current_procinfo.procdef.funcretloc[calleeside].location^.next) and
               (current_procinfo.procdef.funcretloc[calleeside].location^.next^.loc=LOC_REGISTER) then
              begin
                if (getsupreg(current_procinfo.procdef.funcretloc[calleeside].location^.next^.register)=RS_EDX) then
                  list.concat(Taicpu.Op_const_reg(A_ADD,S_L,4,NR_ESP))
                else
                  internalerror(2010053002);
              end
            else
              list.concat(Taicpu.Op_reg(A_POP,S_L,NR_EDX));

            list.concat(Taicpu.Op_reg(A_POP,S_L,NR_ESI));
            list.concat(Taicpu.Op_reg(A_POP,S_L,NR_EDI));
            { .... also the segment registers }
            list.concat(Taicpu.Op_reg(A_POP,S_W,NR_DS));
            list.concat(Taicpu.Op_reg(A_POP,S_W,NR_ES));
            list.concat(Taicpu.Op_reg(A_POP,S_W,NR_FS));
            list.concat(Taicpu.Op_reg(A_POP,S_W,NR_GS));
            { this restores the flags }
            list.concat(Taicpu.Op_none(A_IRET,S_NO));
          end
        { Routines with the poclearstack flag set use only a ret }
        else if (current_procinfo.procdef.proccalloption in clearstack_pocalls) and
                (not paramanager.use_fixed_stack)  then
         begin
           { complex return values are removed from stack in C code PM }
           { but not on win32 }
           { and not for safecall with hidden exceptions, because the result }
           { wich contains the exception is passed in EAX }
           if ((target_info.system <> system_i386_win32) or
               (target_info.abi=abi_old_win32_gnu)) and
              not ((current_procinfo.procdef.proccalloption = pocall_safecall) and
               (tf_safecall_exceptions in target_info.flags)) and
              paramanager.ret_in_param(current_procinfo.procdef.returndef,
                                       current_procinfo.procdef) then
             list.concat(Taicpu.Op_const(A_RET,S_W,sizeof(aint)))
           else
             list.concat(Taicpu.Op_none(A_RET,S_NO));
         end
        { ... also routines with parasize=0 }
        else if (parasize=0) then
         list.concat(Taicpu.Op_none(A_RET,S_NO))
        else
         begin
           { parameters are limited to 65535 bytes because ret allows only imm16 }
           if (parasize>65535) then
             CGMessage(cg_e_parasize_too_big);
           list.concat(Taicpu.Op_const(A_RET,S_W,parasize));
         end;
      end;


    procedure tcg386.g_copyvaluepara_openarray(list : TAsmList;const ref:treference;const lenloc:tlocation;elesize:tcgint;destreg:tregister);
      var
        power  : longint;
        opsize : topsize;
{$ifndef __NOWINPECOFF__}
        again,ok : tasmlabel;
{$endif}
      begin
        { get stack space }
        getcpuregister(list,NR_EDI);
        a_load_loc_reg(list,OS_INT,lenloc,NR_EDI);
        list.concat(Taicpu.op_reg(A_INC,S_L,NR_EDI));
        { Now EDI contains (high+1). }

        { special case handling for elesize=8, 4 and 2:
          set ECX = (high+1) instead of ECX = (high+1)*elesize.

          In the case of elesize=4 and 2, this allows us to avoid the SHR later.
          In the case of elesize=8, we can later use a SHL ECX, 1 instead of
          SHR ECX, 2 which is one byte shorter. }
        if (elesize=8) or (elesize=4) or (elesize=2) then
          begin
            { Now EDI contains (high+1). Copy it to ECX for later use. }
            getcpuregister(list,NR_ECX);
            list.concat(Taicpu.op_reg_reg(A_MOV,S_L,NR_EDI,NR_ECX));
          end;
        { EDI := EDI * elesize }
        if (elesize<>1) then
         begin
           if ispowerof2(elesize, power) then
             list.concat(Taicpu.op_const_reg(A_SHL,S_L,power,NR_EDI))
           else
             list.concat(Taicpu.op_const_reg(A_IMUL,S_L,elesize,NR_EDI));
         end;
        if (elesize<>8) and (elesize<>4) and (elesize<>2) then
          begin
            { Now EDI contains (high+1)*elesize. Copy it to ECX for later use. }
            getcpuregister(list,NR_ECX);
            list.concat(Taicpu.op_reg_reg(A_MOV,S_L,NR_EDI,NR_ECX));
          end;
{$ifndef __NOWINPECOFF__}
        { windows guards only a few pages for stack growing, }
        { so we have to access every page first              }
        if target_info.system=system_i386_win32 then
          begin
             current_asmdata.getjumplabel(again);
             current_asmdata.getjumplabel(ok);
             a_label(list,again);
             list.concat(Taicpu.op_const_reg(A_CMP,S_L,winstackpagesize,NR_EDI));
             a_jmp_cond(list,OC_B,ok);
             list.concat(Taicpu.op_const_reg(A_SUB,S_L,winstackpagesize-4,NR_ESP));
             list.concat(Taicpu.op_reg(A_PUSH,S_L,NR_EDI));
             list.concat(Taicpu.op_const_reg(A_SUB,S_L,winstackpagesize,NR_EDI));
             a_jmp_always(list,again);

             a_label(list,ok);
          end;
{$endif __NOWINPECOFF__}
        { If we were probing pages, EDI=(size mod pagesize) and ESP is decremented
          by (size div pagesize)*pagesize, otherwise EDI=size.
          Either way, subtracting EDI from ESP will set ESP to desired final value. }
        list.concat(Taicpu.op_reg_reg(A_SUB,S_L,NR_EDI,NR_ESP));
        { align stack on 4 bytes }
        list.concat(Taicpu.op_const_reg(A_AND,S_L,aint($fffffff4),NR_ESP));
        { load destination, don't use a_load_reg_reg, that will add a move instruction
          that can confuse the reg allocator }
        list.concat(Taicpu.Op_reg_reg(A_MOV,S_L,NR_ESP,NR_EDI));

        { Allocate ESI and load it with source }
        getcpuregister(list,NR_ESI);
        a_loadaddr_ref_reg(list,ref,NR_ESI);

        { calculate size }
        opsize:=S_B;
        if elesize=8 then
          begin
            opsize:=S_L;
            { ECX is number of qwords, convert to dwords }
            list.concat(Taicpu.op_const_reg(A_SHL,S_L,1,NR_ECX))
          end
        else if elesize=4 then
          begin
            opsize:=S_L;
            { ECX is already number of dwords, so no need to SHL/SHR }
          end
        else if elesize=2 then
          begin
            opsize:=S_W;
            { ECX is already number of words, so no need to SHL/SHR }
          end
        else
         if (elesize and 3)=0 then
         begin
           opsize:=S_L;
           { ECX is number of bytes, convert to dwords }
           list.concat(Taicpu.op_const_reg(A_SHR,S_L,2,NR_ECX))
         end
        else
         if (elesize and 1)=0 then
          begin
            opsize:=S_W;
            { ECX is number of bytes, convert to words }
            list.concat(Taicpu.op_const_reg(A_SHR,S_L,1,NR_ECX))
          end;

        if ts_cld in current_settings.targetswitches then
          list.concat(Taicpu.op_none(A_CLD,S_NO));
        list.concat(Taicpu.op_none(A_REP,S_NO));
        case opsize of
          S_B : list.concat(Taicpu.Op_none(A_MOVSB,S_NO));
          S_W : list.concat(Taicpu.Op_none(A_MOVSW,S_NO));
          S_L : list.concat(Taicpu.Op_none(A_MOVSD,S_NO));
          else
            internalerror(2019050901);
        end;
        ungetcpuregister(list,NR_EDI);
        ungetcpuregister(list,NR_ECX);
        ungetcpuregister(list,NR_ESI);

        { patch the new address, but don't use a_load_reg_reg, that will add a move instruction
          that can confuse the reg allocator }
        list.concat(Taicpu.Op_reg_reg(A_MOV,S_L,NR_ESP,destreg));
        include(current_procinfo.flags,pi_has_stack_allocs);
      end;


    procedure tcg386.g_releasevaluepara_openarray(list : TAsmList;const l:tlocation);
      begin
        { Nothing to release }
      end;


    procedure tcg386.g_maybe_got_init(list: TAsmList);
      var
        i: longint;
        tmpreg: TRegister;
      begin
        { allocate PIC register }
        if (tf_pic_uses_got in target_info.flags) and
          (pi_needs_got in current_procinfo.flags) then
          begin
            if not (target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
              begin
                { Use ECX as a temp register by default }
                if current_procinfo.got = NR_EBX then
                  tmpreg:=NR_EBX
                else
                  tmpreg:=NR_ECX;
                { Allocate registers used for parameters to make sure they
                  never allocated during this PIC init code }
                for i:=0 to current_procinfo.procdef.paras.Count - 1 do
                  with tparavarsym(current_procinfo.procdef.paras[i]).paraloc[calleeside].Location^ do
                    if Loc in [LOC_REGISTER, LOC_CREGISTER] then begin
                      a_reg_alloc(list, register);
                      { If ECX is used for a parameter, use EBX as temp }
                      if getsupreg(register) = RS_ECX then
                        tmpreg:=NR_EBX;
                    end;

                if tmpreg = NR_EBX then
                  begin
                    { Mark EBX as used in the proc }
                    include(rg[R_INTREGISTER].used_in_proc,RS_EBX);
                    current_module.requires_ebx_pic_helper:=true;
                    a_call_name_static(list,'fpc_geteipasebx');
                  end
                else
                  begin
                    current_module.requires_ecx_pic_helper:=true;
                    a_call_name_static(list,'fpc_geteipasecx');
                  end;
                list.concat(taicpu.op_sym_ofs_reg(A_ADD,S_L,current_asmdata.RefAsmSymbol('_GLOBAL_OFFSET_TABLE_',AT_DATA),0,tmpreg));
                list.concat(taicpu.op_reg_reg(A_MOV,S_L,tmpreg,current_procinfo.got));

                { Deallocate parameter registers }
                for i:=0 to current_procinfo.procdef.paras.Count - 1 do
                  with tparavarsym(current_procinfo.procdef.paras[i]).paraloc[calleeside].Location^ do
                    if Loc in [LOC_REGISTER, LOC_CREGISTER] then
                      a_reg_dealloc(list, register);
              end
            else
              begin
                { call/pop is faster than call/ret/mov on Core Solo and later
                  according to Apple's benchmarking -- and all Intel Macs
                  have at least a Core Solo (furthermore, the i386 - Pentium 1
                  don't have a return stack buffer) }
                a_call_name_static(list,current_procinfo.CurrGOTLabel.name);
                a_label(list,current_procinfo.CurrGotLabel);
                list.concat(taicpu.op_reg(A_POP,S_L,current_procinfo.got))
              end;
          end;
      end;


{ ************* 64bit operations ************ }

    procedure tcg64f386.get_64bit_ops(op:TOpCG;var op1,op2:TAsmOp);
      begin
        case op of
          OP_ADD :
            begin
              op1:=A_ADD;
              op2:=A_ADC;
            end;
          OP_SUB :
            begin
              op1:=A_SUB;
              op2:=A_SBB;
            end;
          OP_XOR :
            begin
              op1:=A_XOR;
              op2:=A_XOR;
            end;
          OP_OR :
            begin
              op1:=A_OR;
              op2:=A_OR;
            end;
          OP_AND :
            begin
              op1:=A_AND;
              op2:=A_AND;
            end;
          else
            internalerror(200203241);
        end;
      end;


    procedure tcg64f386.a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);
      var
        op1,op2 : TAsmOp;
        tempref : treference;
      begin
        if not(op in [OP_NEG,OP_NOT]) then
          begin
            get_64bit_ops(op,op1,op2);
            tempref:=ref;
            tcgx86(cg).make_simple_ref(list,tempref);
            if op in [OP_ADD,OP_SUB] then
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
            list.concat(taicpu.op_ref_reg(op1,S_L,tempref,reg.reglo));
            inc(tempref.offset,4);
            list.concat(taicpu.op_ref_reg(op2,S_L,tempref,reg.reghi));
            if op in [OP_ADD,OP_SUB] then
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
          end
        else
          begin
            a_load64_ref_reg(list,ref,reg);
            a_op64_reg_reg(list,op,size,reg,reg);
          end;
      end;


    procedure tcg64f386.a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64; const ref: treference);
      var
        op1,op2 : TAsmOp;
        tempref : treference;
        tmpreg: TRegister;
        l1, l2: TAsmLabel;
      begin
        case op of
          OP_NOT:
            begin
              tempref:=ref;
              tcgx86(cg).make_simple_ref(list,tempref);
              list.concat(taicpu.op_ref(A_NOT,S_L,tempref));
              inc(tempref.offset,4);
              list.concat(taicpu.op_ref(A_NOT,S_L,tempref));
            end;
          OP_NEG:
            begin
              tempref:=ref;
              tcgx86(cg).make_simple_ref(list,tempref);
              inc(tempref.offset,4);
              list.concat(taicpu.op_ref(A_NOT,S_L,tempref));
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              dec(tempref.offset,4);
              list.concat(taicpu.op_ref(A_NEG,S_L,tempref));
              inc(tempref.offset,4);
              list.concat(taicpu.op_const_ref(A_SBB,S_L,-1,tempref));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
          OP_SHR,OP_SHL,OP_SAR:
            begin
              { load right operators in a register }
              cg.getcpuregister(list,NR_ECX);
              cg.a_load_reg_reg(list,OS_32,OS_32,reg.reglo,NR_ECX);

              tempref:=ref;
              tcgx86(cg).make_simple_ref(list,tempref);

              { the damned shift instructions work only til a count of 32 }
              { so we've to do some tricks here                           }
              current_asmdata.getjumplabel(l1);
              current_asmdata.getjumplabel(l2);
              list.Concat(taicpu.op_const_reg(A_TEST,S_B,32,NR_CL));
              cg.a_jmp_flags(list,F_E,l1);
              tmpreg:=cg.getintregister(list,OS_32);
              case op of
                OP_SHL:
                  begin
                    cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                    list.Concat(taicpu.op_reg_reg(A_SHL,S_L,NR_CL,tmpreg));
                    inc(tempref.offset,4);
                    cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                    dec(tempref.offset,4);
                    cg.a_load_const_ref(list,OS_32,0,tempref);
                    cg.a_jmp_always(list,l2);
                    cg.a_label(list,l1);
                    cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                    inc(tempref.offset,4);
                    list.Concat(taicpu.op_reg_reg_ref(A_SHLD,S_L,NR_CL,tmpreg,tempref));
                    dec(tempref.offset,4);
                    if cs_opt_size in current_settings.optimizerswitches then
                      list.concat(taicpu.op_reg_ref(A_SHL,S_L,NR_CL,tempref))
                    else
                      begin
                        list.concat(taicpu.op_reg_reg(A_SHL,S_L,NR_CL,tmpreg));
                        cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                      end;
                  end;
                OP_SHR:
                  begin
                    inc(tempref.offset,4);
                    cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                    list.Concat(taicpu.op_reg_reg(A_SHR,S_L,NR_CL,tmpreg));
                    dec(tempref.offset,4);
                    cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                    inc(tempref.offset,4);
                    cg.a_load_const_ref(list,OS_32,0,tempref);
                    cg.a_jmp_always(list,l2);
                    cg.a_label(list,l1);
                    cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                    dec(tempref.offset,4);
                    list.Concat(taicpu.op_reg_reg_ref(A_SHRD,S_L,NR_CL,tmpreg,tempref));
                    inc(tempref.offset,4);
                    if cs_opt_size in current_settings.optimizerswitches then
                      list.concat(taicpu.op_reg_ref(A_SHR,S_L,NR_CL,tempref))
                    else
                      begin
                        list.concat(taicpu.op_reg_reg(A_SHR,S_L,NR_CL,tmpreg));
                        cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                      end;
                  end;
                OP_SAR:
                  begin
                    inc(tempref.offset,4);
                    cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                    list.Concat(taicpu.op_reg_reg(A_SAR,S_L,NR_CL,tmpreg));
                    dec(tempref.offset,4);
                    cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                    inc(tempref.offset,4);
                    list.Concat(taicpu.op_const_reg(A_SAR,S_L,31,tmpreg));
                    cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                    cg.a_jmp_always(list,l2);
                    cg.a_label(list,l1);
                    cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                    dec(tempref.offset,4);
                    list.Concat(taicpu.op_reg_reg_ref(A_SHRD,S_L,NR_CL,tmpreg,tempref));
                    inc(tempref.offset,4);
                    if cs_opt_size in current_settings.optimizerswitches then
                      list.concat(taicpu.op_reg_ref(A_SAR,S_L,NR_CL,tempref))
                    else
                      begin
                        list.concat(taicpu.op_reg_reg(A_SAR,S_L,NR_CL,tmpreg));
                        cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                      end;
                  end;
                else
                  internalerror(2017041801);
              end;
              cg.a_label(list,l2);

              cg.ungetcpuregister(list,NR_ECX);
              exit;
            end;
          else
            begin
              get_64bit_ops(op,op1,op2);
              tempref:=ref;
              tcgx86(cg).make_simple_ref(list,tempref);
              if op in [OP_ADD,OP_SUB] then
                cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_reg_ref(op1,S_L,reg.reglo,tempref));
              inc(tempref.offset,4);
              list.concat(taicpu.op_reg_ref(op2,S_L,reg.reghi,tempref));
              if op in [OP_ADD,OP_SUB] then
                cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
        end;
      end;


    procedure tcg64f386.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      var
        op1,op2 : TAsmOp;
        l1, l2: TAsmLabel;
      begin
        case op of
          OP_NEG :
            begin
              if (regsrc.reglo<>regdst.reglo) then
                a_load64_reg_reg(list,regsrc,regdst);
              list.concat(taicpu.op_reg(A_NOT,S_L,regdst.reghi));
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_reg(A_NEG,S_L,regdst.reglo));
              list.concat(taicpu.op_const_reg(A_SBB,S_L,-1,regdst.reghi));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
              exit;
            end;
          OP_NOT :
            begin
              if (regsrc.reglo<>regdst.reglo) then
                a_load64_reg_reg(list,regsrc,regdst);
              list.concat(taicpu.op_reg(A_NOT,S_L,regdst.reghi));
              list.concat(taicpu.op_reg(A_NOT,S_L,regdst.reglo));
              exit;
            end;
          OP_SHR,OP_SHL,OP_SAR:
            begin
              { load right operators in a register }
              cg.getcpuregister(list,NR_ECX);
              cg.a_load_reg_reg(list,OS_32,OS_32,regsrc.reglo,NR_ECX);

              { the damned shift instructions work only til a count of 32 }
              { so we've to do some tricks here                           }
              current_asmdata.getjumplabel(l1);
              current_asmdata.getjumplabel(l2);
              list.Concat(taicpu.op_const_reg(A_TEST,S_B,32,NR_CL));
              cg.a_jmp_flags(list,F_E,l1);
              case op of
                OP_SHL:
                  begin
                    list.Concat(taicpu.op_reg_reg(A_SHL,S_L,NR_CL,regdst.reglo));
                    cg.a_load_reg_reg(list,OS_32,OS_32,regdst.reglo,regdst.reghi);
                    list.Concat(taicpu.op_reg_reg(A_XOR,S_L,regdst.reglo,regdst.reglo));
                    cg.a_jmp_always(list,l2);
                    cg.a_label(list,l1);
                    list.Concat(taicpu.op_reg_reg_reg(A_SHLD,S_L,NR_CL,regdst.reglo,regdst.reghi));
                    list.Concat(taicpu.op_reg_reg(A_SHL,S_L,NR_CL,regdst.reglo));
                  end;
                OP_SHR:
                  begin
                    list.Concat(taicpu.op_reg_reg(A_SHR,S_L,NR_CL,regdst.reghi));
                    cg.a_load_reg_reg(list,OS_32,OS_32,regdst.reghi,regdst.reglo);
                    list.Concat(taicpu.op_reg_reg(A_XOR,S_L,regdst.reghi,regdst.reghi));
                    cg.a_jmp_always(list,l2);
                    cg.a_label(list,l1);
                    list.Concat(taicpu.op_reg_reg_reg(A_SHRD,S_L,NR_CL,regdst.reghi,regdst.reglo));
                    list.Concat(taicpu.op_reg_reg(A_SHR,S_L,NR_CL,regdst.reghi));
                  end;
                OP_SAR:
                  begin
                    cg.a_load_reg_reg(list,OS_32,OS_32,regdst.reghi,regdst.reglo);
                    list.Concat(taicpu.op_reg_reg(A_SAR,S_L,NR_CL,regdst.reglo));
                    list.Concat(taicpu.op_const_reg(A_SAR,S_L,31,regdst.reghi));
                    cg.a_jmp_always(list,l2);
                    cg.a_label(list,l1);
                    list.Concat(taicpu.op_reg_reg_reg(A_SHRD,S_L,NR_CL,regdst.reghi,regdst.reglo));
                    list.Concat(taicpu.op_reg_reg(A_SAR,S_L,NR_CL,regdst.reghi));
                  end;
                else
                  internalerror(2017041801);
              end;
              cg.a_label(list,l2);

              cg.ungetcpuregister(list,NR_ECX);
              exit;
            end;
          else
            ;
        end;
        get_64bit_ops(op,op1,op2);
        if op in [OP_ADD,OP_SUB] then
          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
        list.concat(taicpu.op_reg_reg(op1,S_L,regsrc.reglo,regdst.reglo));
        list.concat(taicpu.op_reg_reg(op2,S_L,regsrc.reghi,regdst.reghi));
        if op in [OP_ADD,OP_SUB] then
          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
      end;


    procedure tcg64f386.a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);
      var
        op1,op2 : TAsmOp;
      begin
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg(list,op,OS_32,tcgint(lo(value)),reg.reglo);
              cg.a_op_const_reg(list,op,OS_32,tcgint(hi(value)),reg.reghi);
            end;
          OP_ADD, OP_SUB:
            begin
              // can't use a_op_const_ref because this may use dec/inc
              get_64bit_ops(op,op1,op2);
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_const_reg(op1,S_L,aint(lo(value)),reg.reglo));
              list.concat(taicpu.op_const_reg(op2,S_L,aint(hi(value)),reg.reghi));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
          OP_SHR,OP_SHL,OP_SAR:
            begin
              value:=value and 63;
              if value<>0 then
                begin
                  if (value=1) and (op=OP_SHL) and
                     (current_settings.optimizecputype<=cpu_486) and
                     not (cs_opt_size in current_settings.optimizerswitches) then
                    begin
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                      list.concat(taicpu.op_reg_reg(A_ADD,S_L,reg.reglo,reg.reglo));
                      list.concat(taicpu.op_reg_reg(A_ADC,S_L,reg.reghi,reg.reghi));
                      cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                    end
                  else if (value=1) and (cs_opt_size in current_settings.optimizerswitches) then
                    case op of
                      OP_SHR:
                        begin
                          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                          list.concat(taicpu.op_const_reg(A_SHR,S_L,value,reg.reghi));
                          list.concat(taicpu.op_const_reg(A_RCR,S_L,value,reg.reglo));
                          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                        end;
                      OP_SHL:
                        begin
                          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                          list.concat(taicpu.op_const_reg(A_SHL,S_L,value,reg.reglo));
                          list.concat(taicpu.op_const_reg(A_RCL,S_L,value,reg.reghi));
                          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                        end;
                      OP_SAR:
                        begin
                          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                          list.concat(taicpu.op_const_reg(A_SAR,S_L,value,reg.reghi));
                          list.concat(taicpu.op_const_reg(A_RCR,S_L,value,reg.reglo));
                          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                        end;
                      else
                        internalerror(2019050902);
                    end
                  else if value>31 then
                    case op of
                      OP_SAR:
                        begin
                          cg.a_load_reg_reg(list,OS_32,OS_32,reg.reghi,reg.reglo);
                          list.concat(taicpu.op_const_reg(A_SAR,S_L,31,reg.reghi));
                          if (value and 31)<>0 then
                            list.concat(taicpu.op_const_reg(A_SAR,S_L,value and 31,reg.reglo));
                        end;
                      OP_SHR:
                        begin
                          cg.a_load_reg_reg(list,OS_32,OS_32,reg.reghi,reg.reglo);
                          list.concat(taicpu.op_reg_reg(A_XOR,S_L,reg.reghi,reg.reghi));
                          if (value and 31)<>0 then
                            list.concat(taicpu.op_const_reg(A_SHR,S_L,value and 31,reg.reglo));
                        end;
                      OP_SHL:
                        begin
                          cg.a_load_reg_reg(list,OS_32,OS_32,reg.reglo,reg.reghi);
                          list.concat(taicpu.op_reg_reg(A_XOR,S_L,reg.reglo,reg.reglo));
                          if (value and 31)<>0 then
                            list.concat(taicpu.op_const_reg(A_SHL,S_L,value and 31,reg.reghi));
                        end;
                      else
                        internalerror(2017041201);
                    end
                  else
                    case op of
                      OP_SAR:
                        begin
                          list.concat(taicpu.op_const_reg_reg(A_SHRD,S_L,value,reg.reghi,reg.reglo));
                          list.concat(taicpu.op_const_reg(A_SAR,S_L,value,reg.reghi));
                        end;
                      OP_SHR:
                        begin
                          list.concat(taicpu.op_const_reg_reg(A_SHRD,S_L,value,reg.reghi,reg.reglo));
                          list.concat(taicpu.op_const_reg(A_SHR,S_L,value,reg.reghi));
                        end;
                      OP_SHL:
                        begin
                          list.concat(taicpu.op_const_reg_reg(A_SHLD,S_L,value,reg.reglo,reg.reghi));
                          list.concat(taicpu.op_const_reg(A_SHL,S_L,value,reg.reglo));
                        end;
                      else
                        internalerror(2017041201);
                    end;
                end;
            end;
          else
            internalerror(200204021);
        end;
      end;


    procedure tcg64f386.a_op64_const_ref(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const ref : treference);
      var
        op1,op2 : TAsmOp;
        tempref : treference;
        tmpreg: TRegister;
      begin
        tempref:=ref;
        tcgx86(cg).make_simple_ref(list,tempref);
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_ref(list,op,OS_32,aint(lo(value)),tempref);
              inc(tempref.offset,4);
              cg.a_op_const_ref(list,op,OS_32,aint(hi(value)),tempref);
            end;
          OP_ADD, OP_SUB:
            begin
              get_64bit_ops(op,op1,op2);
              // can't use a_op_const_ref because this may use dec/inc
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_const_ref(op1,S_L,aint(lo(value)),tempref));
              inc(tempref.offset,4);
              list.concat(taicpu.op_const_ref(op2,S_L,aint(hi(value)),tempref));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
          OP_SHR,OP_SHL,OP_SAR:
            begin
              value:=value and 63;
              if value<>0 then
                begin
                  if value=1 then
                    case op of
                      OP_SHR:
                        begin
                          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                          inc(tempref.offset,4);
                          list.concat(taicpu.op_const_ref(A_SHR,S_L,value,tempref));
                          dec(tempref.offset,4);
                          list.concat(taicpu.op_const_ref(A_RCR,S_L,value,tempref));
                          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                        end;
                      OP_SHL:
                        begin
                          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                          list.concat(taicpu.op_const_ref(A_SHL,S_L,value,tempref));
                          inc(tempref.offset,4);
                          list.concat(taicpu.op_const_ref(A_RCL,S_L,value,tempref));
                          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                        end;
                      OP_SAR:
                        begin
                          cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                          inc(tempref.offset,4);
                          list.concat(taicpu.op_const_ref(A_SAR,S_L,value,tempref));
                          dec(tempref.offset,4);
                          list.concat(taicpu.op_const_ref(A_RCR,S_L,value,tempref));
                          cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                        end;
                      else
                        internalerror(2019050901);
                    end
                  else if value>31 then
                    case op of
                      OP_SHR,OP_SAR:
                        begin
                          tmpreg:=cg.getintregister(list,OS_32);
                          inc(tempref.offset,4);
                          cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                          if (value and 31)<>0 then
                            if op=OP_SHR then
                              list.concat(taicpu.op_const_reg(A_SHR,S_L,value and 31,tmpreg))
                            else
                              list.concat(taicpu.op_const_reg(A_SAR,S_L,value and 31,tmpreg));
                          dec(tempref.offset,4);
                          cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                          inc(tempref.offset,4);
                          if op=OP_SHR then
                            cg.a_load_const_ref(list,OS_32,0,tempref)
                          else
                            begin
                              list.concat(taicpu.op_const_reg(A_SAR,S_L,31,tmpreg));
                              cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                            end;
                        end;
                      OP_SHL:
                        begin
                          tmpreg:=cg.getintregister(list,OS_32);
                          cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                          if (value and 31)<>0 then
                            list.concat(taicpu.op_const_reg(A_SHL,S_L,value and 31,tmpreg));
                          inc(tempref.offset,4);
                          cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                          dec(tempref.offset,4);
                          cg.a_load_const_ref(list,OS_32,0,tempref);
                        end;
                      else
                        internalerror(2017041801);
                    end
                  else
                    case op of
                      OP_SHR,OP_SAR:
                        begin
                          tmpreg:=cg.getintregister(list,OS_32);
                          inc(tempref.offset,4);
                          cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                          dec(tempref.offset,4);
                          list.concat(taicpu.op_const_reg_ref(A_SHRD,S_L,value,tmpreg,tempref));
                          inc(tempref.offset,4);
                          if cs_opt_size in current_settings.optimizerswitches then
                            begin
                              if op=OP_SHR then
                                list.concat(taicpu.op_const_ref(A_SHR,S_L,value,tempref))
                              else
                                list.concat(taicpu.op_const_ref(A_SAR,S_L,value,tempref));
                            end
                          else
                            begin
                              if op=OP_SHR then
                                list.concat(taicpu.op_const_reg(A_SHR,S_L,value,tmpreg))
                              else
                                list.concat(taicpu.op_const_reg(A_SAR,S_L,value,tmpreg));
                              cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                            end;
                        end;
                      OP_SHL:
                        begin
                          tmpreg:=cg.getintregister(list,OS_32);
                          cg.a_load_ref_reg(list,OS_32,OS_32,tempref,tmpreg);
                          inc(tempref.offset,4);
                          list.concat(taicpu.op_const_reg_ref(A_SHLD,S_L,value,tmpreg,tempref));
                          dec(tempref.offset,4);
                          if cs_opt_size in current_settings.optimizerswitches then
                            list.concat(taicpu.op_const_ref(A_SHL,S_L,value,tempref))
                          else
                            begin
                              list.concat(taicpu.op_const_reg(A_SHL,S_L,value,tmpreg));
                              cg.a_load_reg_ref(list,OS_32,OS_32,tmpreg,tempref);
                            end;
                        end;
                      else
                        internalerror(2017041201);
                    end;
                end;
            end;
          else
            internalerror(200204022);
        end;
      end;

    procedure create_codegen;
      begin
        cg := tcg386.create;
        cg64 := tcg64f386.create;
      end;

end.
