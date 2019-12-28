{
    Copyright (c) 2014 by Jonas Maebe

    This unit implements the code generator for AArch64

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
       globtype,parabase,
       cgbase,cgutils,cgobj,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpuinfo,
       node,symconst,SymType,symdef,
       rgcpu;

    type
      tcgaarch64=class(tcg)
       protected
        { changes register size without adding register allocation info }
        function makeregsize(reg: tregister; size: tcgsize): tregister; overload;
       public
        { simplifies "ref" so it can be used with "op". If "ref" can be used
          with a different load/Store operation that has the same meaning as the
          original one, "op" will be replaced with the alternative }
        procedure make_simple_ref(list:TAsmList; var op: tasmop; size: tcgsize; oppostfix: toppostfix; var ref: treference; preferred_newbasereg: tregister);
        function getfpuregister(list: TAsmList; size: Tcgsize): Tregister; override;
        procedure handle_reg_imm12_reg(list: TAsmList; op: Tasmop; size: tcgsize; src: tregister; a: tcgint; dst: tregister; tmpreg: tregister; setflags, usedest: boolean);
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;
        function  getmmregister(list:TAsmList;size:tcgsize):tregister;override;
        function handle_load_store(list:TAsmList; op: tasmop; size: tcgsize; oppostfix: toppostfix; reg: tregister; ref: treference):treference;
        procedure a_call_name(list:TAsmList;const s:string; weak: boolean);override;
        procedure a_call_reg(list:TAsmList;Reg:tregister);override;
        { General purpose instructions }
        procedure maybeadjustresult(list: TAsmList; op: topcg; size: tcgsize; dst: tregister);
        procedure a_op_const_reg(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; reg: tregister);override;
        procedure a_op_reg_reg(list: TAsmList; op: topcg; size: tcgsize; src, dst: tregister);override;
        procedure a_op_const_reg_reg(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; src, dst: tregister);override;
        procedure a_op_reg_reg_reg(list: TAsmList; op: topcg; size: tcgsize; src1, src2, dst: tregister);override;
        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; src, dst: tregister; setflags : boolean; var ovloc : tlocation);override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: topcg; size: tcgsize; src1, src2, dst: tregister; setflags : boolean; var ovloc : tlocation);override;
        { move instructions }
        procedure a_load_const_reg(list: TAsmList; size: tcgsize; a: tcgint; reg: tregister);override;
        procedure a_load_const_ref(list: TAsmList; size: tcgsize; a: tcgint; const ref: treference); override;
        procedure a_load_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister;const ref: TReference);override;
        procedure a_load_reg_ref_unaligned(list: TAsmList; fromsize, tosize: tcgsize; register: tregister; const ref: treference); override;
        procedure a_load_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: TReference; reg: tregister);override;
        procedure a_load_ref_reg_unaligned(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; register: tregister); override;
        procedure a_load_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);override;
        procedure a_loadaddr_ref_reg(list: TAsmList; const ref: TReference; r: tregister);override;
        { fpu move instructions (not used, all floating point is vector unit-based) }
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;
        procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister;shuffle : pmmshuffle);override;
        procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: TReference; reg: tregister; shuffle: pmmshuffle);override;
        procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: TReference; shuffle: pmmshuffle);override;

        procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tcgsize; intreg, mmreg: tregister; shuffle: pmmshuffle); override;
        procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tcgsize; mmreg, intreg: tregister; shuffle: pmmshuffle); override;

        procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tcgsize; src, dst: tregister; shuffle: pmmshuffle); override;

        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tcgsize; src, dst: TRegister); override;
        { comparison operations }
        procedure a_cmp_const_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel);override;
        procedure a_cmp_reg_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel);override;
        procedure a_jmp_always(list: TAsmList; l: TAsmLabel);override;
        procedure a_jmp_name(list: TAsmList; const s: string);override;
        procedure a_jmp_cond(list: TAsmList; cond: TOpCmp; l: tasmlabel);{ override;}
        procedure a_jmp_flags(list: TAsmList; const f: tresflags; l: tasmlabel);override;
        procedure g_flags2reg(list: TAsmList; size: tcgsize; const f:tresflags; reg: tregister);override;
        procedure g_overflowcheck(list: TAsmList; const loc: tlocation; def: tdef);override;
        procedure g_overflowcheck_loc(list: TAsmList; const loc: tlocation; def: tdef; ovloc: tlocation);override;
        procedure g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);override;
        procedure g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean);override;
        procedure g_maybe_got_init(list: TAsmList); override;
        procedure g_restore_registers(list: TAsmList);override;
        procedure g_save_registers(list: TAsmList);override;
        procedure g_concatcopy_move(list: TAsmList; const source, dest: treference; len: tcgint);
        procedure g_concatcopy(list: TAsmList; const source, dest: treference; len: tcgint);override;
        procedure g_adjust_self_value(list: TAsmList; procdef: tprocdef; ioffset: tcgint);override;
        procedure g_check_for_fpu_exception(list: TAsmList; force, clear: boolean);override;
       private
        function save_regs(list: TAsmList; rt: tregistertype; lowsr, highsr: tsuperregister; sub: tsubregister): longint;
        procedure load_regs(list: TAsmList; rt: tregistertype; lowsr, highsr: tsuperregister; sub: tsubregister);
      end;

    procedure create_codegen;

    const
      TOpCG2AsmOpReg: array[topcg] of TAsmOp = (
        A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_MUL,A_MUL,A_NEG,A_MVN,A_ORR,A_ASRV,A_LSLV,A_LSRV,A_SUB,A_EOR,A_NONE,A_RORV
      );
      TOpCG2AsmOpImm: array[topcg] of TAsmOp = (
        A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_MUL,A_MUL,A_NEG,A_MVN,A_ORR,A_ASR,A_LSL,A_LSR,A_SUB,A_EOR,A_NONE,A_ROR
      );
      TOpCmp2AsmCond: array[topcmp] of TAsmCond = (C_NONE,C_EQ,C_GT,
        C_LT,C_GE,C_LE,C_NE,C_LS,C_CC,C_CS,C_HI
      );


implementation

  uses
    globals,verbose,systems,cutils,
    paramgr,fmodule,
    symtable,symsym,
    tgobj,
    procinfo,cpupi;


    procedure tcgaarch64.make_simple_ref(list:TAsmList; var op: tasmop; size: tcgsize; oppostfix: toppostfix; var ref: treference; preferred_newbasereg: tregister);
      var
        href: treference;
        so: tshifterop;
        accesssize: longint;
      begin
        if (ref.base=NR_NO) then
          begin
            if ref.shiftmode<>SM_None then
              internalerror(2014110701);
            ref.base:=ref.index;
            ref.index:=NR_NO;
          end;
        { no abitrary scale factor support (the generic code doesn't set it,
          AArch-specific code shouldn't either) }
        if not(ref.scalefactor in [0,1]) then
          internalerror(2014111002);

        case simple_ref_type(op,size,oppostfix,ref) of
          sr_simple:
            exit;
          sr_internal_illegal:
            internalerror(2014121702);
          sr_complex:
            { continue } ;
        end;

        if assigned(ref.symbol) then
          begin
            { internal "load symbol" instructions should already be valid }
            if assigned(ref.symboldata) or
               (ref.refaddr in [addr_pic,addr_gotpage,addr_gotpageoffset,addr_page,addr_pageoffset]) then
              internalerror(2014110802);
            { no relative symbol support (needed) yet }
            if assigned(ref.relsymbol) then
              internalerror(2014111001);
            { loading a symbol address (whether it's in the GOT or not) consists
              of two parts: first load the page on which it is located, then
              either the offset in the page or load the value at that offset in
              the page. This final GOT-load can be relaxed by the linker in case
              the variable itself can be stored directly in the GOT }
            if (preferred_newbasereg=NR_NO) or
               (ref.base=preferred_newbasereg) or
               (ref.index=preferred_newbasereg) then
              preferred_newbasereg:=getaddressregister(list);
            { load the (GOT) page }
            reference_reset_symbol(href,ref.symbol,0,8,[]);
            if ((ref.symbol.typ in [AT_FUNCTION,AT_LABEL]) and
                (ref.symbol.bind in [AB_LOCAL,AB_GLOBAL])) or
               ((ref.symbol.typ=AT_DATA) and
                (ref.symbol.bind=AB_LOCAL)) then
              href.refaddr:=addr_page
            else
              href.refaddr:=addr_gotpage;
            list.concat(taicpu.op_reg_ref(A_ADRP,preferred_newbasereg,href));
            { load the GOT entry (= address of the variable) }
            reference_reset_base(href,preferred_newbasereg,0,ctempposinvalid,sizeof(pint),[]);
            href.symbol:=ref.symbol;
            { code symbols defined in the current compilation unit do not
              have to be accessed via the GOT }
            if ((ref.symbol.typ in [AT_FUNCTION,AT_LABEL]) and
                (ref.symbol.bind in [AB_LOCAL,AB_GLOBAL])) or
               ((ref.symbol.typ=AT_DATA) and
                (ref.symbol.bind=AB_LOCAL)) then
              begin
                href.base:=NR_NO;
                href.refaddr:=addr_pageoffset;
                list.concat(taicpu.op_reg_reg_ref(A_ADD,preferred_newbasereg,preferred_newbasereg,href));
              end
            else
              begin
                href.refaddr:=addr_gotpageoffset;
                { use a_load_ref_reg() rather than directly encoding the LDR,
                  so that we'll check the validity of the reference }
                a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,preferred_newbasereg);
              end;
            { set as new base register }
            if ref.base=NR_NO then
              ref.base:=preferred_newbasereg
            else if ref.index=NR_NO then
              ref.index:=preferred_newbasereg
            else
              begin
                { make sure it's valid in case ref.base is SP -> make it
                  the second operand}
                a_op_reg_reg_reg(list,OP_ADD,OS_ADDR,preferred_newbasereg,ref.base,preferred_newbasereg);
                ref.base:=preferred_newbasereg
              end;
            ref.symbol:=nil;
          end;

        { base & index }
        if (ref.base<>NR_NO) and
           (ref.index<>NR_NO) then
          begin
            case op of
              A_LDR, A_STR:
                begin
                  if (ref.shiftmode=SM_None) and
                     (ref.shiftimm<>0) then
                    internalerror(2014110805);
                  { wrong shift? (possible in case of something like
                     array_of_2byte_rec[x].bytefield -> shift will be set 1, but
                     the final load is a 1 byte -> can't use shift after all }
                  if (ref.shiftmode in [SM_LSL,SM_UXTW,SM_SXTW]) and
                     ((ref.shiftimm<>BsfDWord(tcgsizep2size[size])) or
                      (ref.offset<>0)) then
                    begin
                      if preferred_newbasereg=NR_NO then
                        preferred_newbasereg:=getaddressregister(list);
                      { "add" supports a superset of the shift modes supported by
                        load/store instructions }
                      shifterop_reset(so);
                      so.shiftmode:=ref.shiftmode;
                      so.shiftimm:=ref.shiftimm;
                      list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,preferred_newbasereg,ref.base,ref.index,so));
                      reference_reset_base(ref,preferred_newbasereg,ref.offset,ref.temppos,ref.alignment,ref.volatility);
                      { possibly still an invalid offset -> fall through }
                    end
                  else if ref.offset<>0 then
                    begin
                      if (preferred_newbasereg=NR_NO) or
                         { we keep ref.index, so it must not be overwritten }
                         (ref.index=preferred_newbasereg) then
                        preferred_newbasereg:=getaddressregister(list);
                      { add to the base and not to the index, because the index
                        may be scaled; this works even if the base is SP }
                      a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base,preferred_newbasereg);
                      ref.offset:=0;
                      ref.base:=preferred_newbasereg;
                      { finished }
                      exit;
                    end
                  else
                    { valid -> exit }
                    exit;
                end;
              { todo }
              A_LD1,A_LD2,A_LD3,A_LD4,
              A_ST1,A_ST2,A_ST3,A_ST4:
                internalerror(2014110704);
              { these don't support base+index }
              A_LDUR,A_STUR,
              A_LDP,A_STP:
                begin
                  { these either don't support pre-/post-indexing, or don't
                    support it with base+index }
                  if ref.addressmode<>AM_OFFSET then
                    internalerror(2014110911);
                  if preferred_newbasereg=NR_NO then
                    preferred_newbasereg:=getaddressregister(list);
                  if ref.shiftmode<>SM_None then
                    begin
                      { "add" supports a superset of the shift modes supported by
                        load/store instructions }
                      shifterop_reset(so);
                      so.shiftmode:=ref.shiftmode;
                      so.shiftimm:=ref.shiftimm;
                      list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,preferred_newbasereg,ref.base,ref.index,so));
                    end
                  else
                    a_op_reg_reg_reg(list,OP_ADD,OS_ADDR,ref.index,ref.base,preferred_newbasereg);
                  reference_reset_base(ref,preferred_newbasereg,ref.offset,ref.temppos,ref.alignment,ref.volatility);
                  { fall through to the handling of base + offset, since the
                    offset may still be too big }
                end;
              else
                internalerror(2014110901);
            end;
          end;

        { base + offset }
        if ref.base<>NR_NO then
          begin
            { valid offset for LDUR/STUR -> use that }
            if (ref.addressmode=AM_OFFSET) and
               (op in [A_LDR,A_STR]) and
               (ref.offset>=-256) and
               (ref.offset<=255) then
              begin
                if op=A_LDR then
                  op:=A_LDUR
                else
                  op:=A_STUR
              end
            { if it's not a valid LDUR/STUR, use LDR/STR }
            else if (op in [A_LDUR,A_STUR]) and
               ((ref.offset<-256) or
                (ref.offset>255) or
                (ref.addressmode<>AM_OFFSET)) then
              begin
                if op=A_LDUR then
                  op:=A_LDR
                else
                  op:=A_STR
              end;
            case op of
              A_LDR,A_STR:
                begin
                  case ref.addressmode of
                    AM_PREINDEXED:
                      begin
                        { since the loaded/stored register cannot be the same
                          as the base register, we can safely add the
                          offset to the base if it doesn't fit}
                        if (ref.offset<-256) or
                            (ref.offset>255) then
                          begin
                            a_op_const_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base);
                            ref.offset:=0;
                          end;
                      end;
                    AM_POSTINDEXED:
                      begin
                        { cannot emulate post-indexing if we have to fold the
                          offset into the base register }
                        if (ref.offset<-256) or
                            (ref.offset>255) then
                          internalerror(2014110909);
                        { ok }
                      end;
                    AM_OFFSET:
                      begin
                        { unsupported offset -> fold into base register }
                        accesssize:=1 shl tcgsizep2size[size];
                        if (ref.offset<0) or
                           (ref.offset>(((1 shl 12)-1)*accesssize)) or
                           ((ref.offset mod accesssize)<>0) then
                          begin
                            if preferred_newbasereg=NR_NO then
                              preferred_newbasereg:=getaddressregister(list);
                            { can we split the offset beween an
                              "add/sub (imm12 shl 12)" and the load (also an
                              imm12)?
                              -- the offset from the load will always be added,
                                that's why the lower bound has a smaller range
                                than the upper bound; it must also be a multiple
                                of the access size }
                            if (ref.offset>=-(((1 shl 12)-1) shl 12)) and
                               (ref.offset<=((1 shl 12)-1) shl 12 + ((1 shl 12)-1)) and
                               ((ref.offset mod accesssize)=0) then
                              begin
                                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,(ref.offset shr 12) shl 12,ref.base,preferred_newbasereg);
                                ref.offset:=ref.offset-(ref.offset shr 12) shl 12;
                              end
                            else
                              begin
                                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base,preferred_newbasereg);
                                ref.offset:=0;
                              end;
                            reference_reset_base(ref,preferred_newbasereg,ref.offset,ref.temppos,ref.alignment,ref.volatility);
                          end;
                      end
                  end;
                end;
              A_LDP,A_STP:
                begin
                  { unsupported offset -> fold into base register (these
                    instructions support all addressmodes) }
                  if (ref.offset<-(1 shl (6+tcgsizep2size[size]))) or
                     (ref.offset>(1 shl (6+tcgsizep2size[size]))-1) then
                    begin
                      case ref.addressmode of
                        AM_POSTINDEXED:
                          { don't emulate post-indexing if we have to fold the
                            offset into the base register }
                          internalerror(2014110910);
                        AM_PREINDEXED:
                          { this means the offset must be added to the current
                            base register }
                          preferred_newbasereg:=ref.base;
                        AM_OFFSET:
                          if preferred_newbasereg=NR_NO then
                            preferred_newbasereg:=getaddressregister(list);
                      end;
                      a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base,preferred_newbasereg);
                      reference_reset_base(ref,preferred_newbasereg,0,ref.temppos,ref.alignment,ref.volatility);
                    end
                end;
              A_LDUR,A_STUR:
                begin
                  { valid, checked above }
                end;
              { todo }
              A_LD1,A_LD2,A_LD3,A_LD4,
              A_ST1,A_ST2,A_ST3,A_ST4:
                internalerror(2014110908);
              else
                internalerror(2014110708);
            end;
            { done }
            exit;
          end;

        { only an offset -> change to base (+ offset 0) }
        if preferred_newbasereg=NR_NO then
          preferred_newbasereg:=getaddressregister(list);
        a_load_const_reg(list,OS_ADDR,ref.offset,preferred_newbasereg);
        reference_reset_base(ref,preferred_newbasereg,0,ref.temppos,newalignment(8,ref.offset),ref.volatility);
      end;


    function tcgaarch64.makeregsize(reg: tregister; size: tcgsize): tregister;
      var
        subreg:Tsubregister;
      begin
        subreg:=cgsize2subreg(getregtype(reg),size);
        result:=reg;
        setsubreg(result,subreg);
      end;


    function tcgaarch64.getfpuregister(list: TAsmList; size: Tcgsize): Tregister;
      begin
        internalerror(2014122110);
        { squash warning }
        result:=NR_NO;
      end;


    function tcgaarch64.handle_load_store(list: TAsmList; op: tasmop; size: tcgsize; oppostfix: toppostfix; reg: tregister; ref: treference):treference;
      begin
        make_simple_ref(list,op,size,oppostfix,ref,NR_NO);
        list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),oppostfix));
        result:=ref;
      end;


    procedure tcgaarch64.handle_reg_imm12_reg(list: TAsmList; op: Tasmop; size: tcgsize; src: tregister; a: tcgint; dst: tregister; tmpreg: tregister; setflags, usedest: boolean);
      var
        instr: taicpu;
        so: tshifterop;
        hadtmpreg: boolean;
      begin
        { imm12 }
        if (a>=0) and
           (a<=((1 shl 12)-1)) then
          if usedest then
            instr:=taicpu.op_reg_reg_const(op,dst,src,a)
          else
            instr:=taicpu.op_reg_const(op,src,a)
        { imm12 lsl 12 }
        else if (a and not(((tcgint(1) shl 12)-1) shl 12))=0 then
          begin
            so.shiftmode:=SM_LSL;
            so.shiftimm:=12;
            if usedest then
              instr:=taicpu.op_reg_reg_const_shifterop(op,dst,src,a shr 12,so)
            else
              instr:=taicpu.op_reg_const_shifterop(op,src,a shr 12,so)
          end
        else
          begin
            { todo: other possible optimizations (e.g. load 16 bit constant in
                register and then add/sub/cmp/cmn shifted the rest) }
            if tmpreg=NR_NO then
              begin
                hadtmpreg:=false;
                tmpreg:=getintregister(list,size);
              end
            else
              begin
                hadtmpreg:=true;
                getcpuregister(list,tmpreg);
              end;
            a_load_const_reg(list,size,a,tmpreg);
            if usedest then
              instr:=taicpu.op_reg_reg_reg(op,dst,src,tmpreg)
            else
              instr:=taicpu.op_reg_reg(op,src,tmpreg);
            if hadtmpreg then
              ungetcpuregister(list,tmpreg);
          end;
        if setflags then
          setoppostfix(instr,PF_S);
        list.concat(instr);
      end;


{****************************************************************************
                              Assembler code
****************************************************************************}

    procedure tcgaarch64.init_register_allocators;
      begin
        inherited init_register_allocators;

        rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
            [RS_X0,RS_X1,RS_X2,RS_X3,RS_X4,RS_X5,RS_X6,RS_X7,RS_X8,
             RS_X9,RS_X10,RS_X11,RS_X12,RS_X13,RS_X14,RS_X15,RS_X16,RS_X17,
             RS_X19,RS_X20,RS_X21,RS_X22,RS_X23,RS_X24,RS_X25,RS_X26,RS_X27,RS_X28
             { maybe we can enable this in the future for leaf functions (it's
               the frame pointer)
             ,RS_X29 }],
            first_int_imreg,[]);

        rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBMMD,
            [RS_Q0,RS_Q1,RS_Q2,RS_Q3,RS_Q4,RS_Q5,RS_Q6,RS_Q7,
             RS_Q8,RS_Q9,RS_Q10,RS_Q11,RS_Q12,RS_Q13,RS_Q14,RS_Q15,
             RS_Q16,RS_Q17,RS_Q18,RS_Q19,RS_Q20,RS_Q21,RS_Q22,RS_Q23,
             RS_Q24,RS_Q25,RS_Q26,RS_Q27,RS_Q28,RS_Q29,RS_Q30,RS_Q31],
            first_mm_imreg,[]);
      end;


    procedure tcgaarch64.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


    function tcgaarch64.getmmregister(list: TAsmList; size: tcgsize):tregister;
      begin
        case size of
          OS_F32:
            result:=rg[R_MMREGISTER].getregister(list,R_SUBMMS);
          OS_F64:
            result:=rg[R_MMREGISTER].getregister(list,R_SUBMMD)
          else
            internalerror(2014102701);
        end;
      end;


    procedure tcgaarch64.a_call_name(list: TAsmList; const s: string; weak: boolean);
      begin
        if not weak then
          list.concat(taicpu.op_sym(A_BL,current_asmdata.RefAsmSymbol(s,AT_FUNCTION)))
        else
          list.concat(taicpu.op_sym(A_BL,current_asmdata.WeakRefAsmSymbol(s,AT_FUNCTION)));
      end;


    procedure tcgaarch64.a_call_reg(list:TAsmList;Reg:tregister);
      begin
        list.concat(taicpu.op_reg(A_BLR,reg));
      end;


    {********************** load instructions ********************}

    procedure tcgaarch64.a_load_const_reg(list: TAsmList; size: tcgsize; a: tcgint; reg : tregister);
      var
        preva: tcgint;
        opc: tasmop;
        shift,maxshift: byte;
        so: tshifterop;
        reginited: boolean;
        mask: tcgint;
      begin
        { if we load a value into a 32 bit register, it is automatically
          zero-extended to 64 bit }
        if (hi(a)=0) and
           (size in [OS_64,OS_S64]) then
          begin
            size:=OS_32;
            reg:=makeregsize(reg,size);
          end;
        { values <= 32 bit are stored in a 32 bit register }
        if not(size in [OS_64,OS_S64]) then
          a:=cardinal(a);

        if size in [OS_64,OS_S64] then
          begin
            mask:=-1;
            maxshift:=64;
          end
        else
          begin
            mask:=$ffffffff;
            maxshift:=32;
          end;
        { single movn enough? (to be extended) }
        shift:=16;
        preva:=a;
        repeat
          if (a shr shift)=(mask shr shift) then
            begin
              if shift=16 then
                list.concat(taicpu.op_reg_const(A_MOVN,reg,not(word(preva))))
              else
                begin
                  shifterop_reset(so);
                  so.shiftmode:=SM_LSL;
                  so.shiftimm:=shift-16;
                  list.concat(taicpu.op_reg_const_shifterop(A_MOVN,reg,not(word(preva)),so));
                end;
              exit;
            end;
          { only try the next 16 bits if the current one is all 1 bits, since
            the movn will set all lower bits to 1 }
          if word(a shr (shift-16))<>$ffff then
            break;
          inc(shift,16);
        until shift=maxshift;
        reginited:=false;
        shift:=0;
        { can be optimized later to use more movn }
        repeat
          { leftover is shifterconst? (don't check if we can represent it just
            as effectively with movz/movk, as this check is expensive) }
          if ((shift<tcgsize2size[size]*(8 div 2)) and
              (word(a)<>0) and
              ((a shr 16)<>0)) and
             is_shifter_const(a shl shift,size) then
            begin
              if reginited then
                list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,a shl shift))
              else
                list.concat(taicpu.op_reg_reg_const(A_ORR,reg,makeregsize(NR_XZR,size),a shl shift));
              exit;
            end;
          { set all 16 bit parts <> 0 }
          if (word(a)<>0) or
             ((shift=0) and
              (a=0)) then
            if shift=0 then
              begin
                list.concat(taicpu.op_reg_const(A_MOVZ,reg,word(a)));
                reginited:=true;
              end
            else
              begin
                shifterop_reset(so);
                so.shiftmode:=SM_LSL;
                so.shiftimm:=shift;
                if not reginited then
                  begin
                    opc:=A_MOVZ;
                    reginited:=true;
                  end
                else
                  opc:=A_MOVK;
                list.concat(taicpu.op_reg_const_shifterop(opc,reg,word(a),so));
              end;
            preva:=a;
            a:=a shr 16;
           inc(shift,16);
        until word(preva)=preva;
        if not reginited then
          internalerror(2014102702);
      end;


    procedure tcgaarch64.a_load_const_ref(list: TAsmList; size: tcgsize; a: tcgint; const ref: treference);
      var
        reg: tregister;
      begin
        { use the zero register if possible }
        if a=0 then
          begin
            if size in [OS_64,OS_S64] then
              reg:=NR_XZR
            else
              reg:=NR_WZR;
            a_load_reg_ref(list,size,size,reg,ref);
          end
        else
          inherited;
      end;


    procedure tcgaarch64.a_load_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);
      var
        oppostfix:toppostfix;
        hreg: tregister;
      begin
        if tcgsize2Size[fromsize]>=tcgsize2Size[tosize] then
          begin
            fromsize:=tosize;
            reg:=makeregsize(list,reg,fromsize);
          end
        { have a 32 bit register but need a 64 bit one? }
        else if tosize in [OS_64,OS_S64] then
          begin
            { sign extend if necessary }
            if fromsize in [OS_S8,OS_S16,OS_S32] then
              begin
                { can't overwrite reg, may be a constant reg }
                hreg:=getintregister(list,tosize);
                a_load_reg_reg(list,fromsize,tosize,reg,hreg);
                reg:=hreg;
              end
            else
              { top 32 bit are zero by default }
              reg:=makeregsize(reg,OS_64);
            fromsize:=tosize;
          end;
        if (ref.alignment<>0) and
           (ref.alignment<tcgsize2size[tosize]) then
          begin
            a_load_reg_ref_unaligned(list,fromsize,tosize,reg,ref);
          end
        else
          begin
            case tosize of
              { signed integer registers }
              OS_8,
              OS_S8:
                oppostfix:=PF_B;
              OS_16,
              OS_S16:
                oppostfix:=PF_H;
              OS_32,
              OS_S32,
              OS_64,
              OS_S64:
                oppostfix:=PF_None;
              else
                InternalError(200308299);
            end;
            handle_load_store(list,A_STR,tosize,oppostfix,reg,ref);
          end;
      end;


    procedure tcgaarch64.a_load_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);
      var
        oppostfix:toppostfix;
      begin
        if tcgsize2Size[fromsize]>=tcgsize2Size[tosize] then
          fromsize:=tosize;
        { ensure that all bits of the 32/64 register are always correctly set:
           * default behaviour is always to zero-extend to the entire (64 bit)
             register -> unsigned 8/16/32 bit loads only exist with a 32 bit
             target register, as the upper 32 bit will be zeroed implicitly
             -> always make target register 32 bit
           * signed loads exist both with 32 and 64 bit target registers,
             depending on whether the value should be sign extended to 32 or
             to 64 bit (if sign extended to 32 bit, the upper 32 bits of the
             corresponding 64 bit register are again zeroed) -> no need to
             change anything (we only have 32 and 64 bit registers), except that
             when loading an OS_S32 to a 32 bit register, we don't need/can't
             use sign extension
        }
        if fromsize in [OS_8,OS_16,OS_32] then
          reg:=makeregsize(reg,OS_32);
        if (ref.alignment<>0) and
           (ref.alignment<tcgsize2size[fromsize]) then
          begin
            a_load_ref_reg_unaligned(list,fromsize,tosize,ref,reg);
            exit;
          end;
        case fromsize of
          { signed integer registers }
          OS_8:
            oppostfix:=PF_B;
          OS_S8:
            oppostfix:=PF_SB;
          OS_16:
            oppostfix:=PF_H;
          OS_S16:
            oppostfix:=PF_SH;
          OS_S32:
            if getsubreg(reg)=R_SUBD then
              oppostfix:=PF_NONE
            else
              oppostfix:=PF_SW;
          OS_32,
          OS_64,
          OS_S64:
            oppostfix:=PF_None;
          else
            InternalError(200308297);
        end;
        handle_load_store(list,A_LDR,fromsize,oppostfix,reg,ref);

        { clear upper 16 bits if the value was negative }
        if (fromsize=OS_S8) and (tosize=OS_16) then
          a_load_reg_reg(list,fromsize,tosize,reg,reg);
      end;


    procedure tcgaarch64.a_load_ref_reg_unaligned(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; register: tregister);
      var
        href: treference;
        hreg1, hreg2, tmpreg,tmpreg2: tregister;
        i : Integer;
      begin
        case fromsize of
          OS_64,OS_S64:
            begin
              { split into two 32 bit loads }
              hreg1:=getintregister(list,OS_32);
              hreg2:=getintregister(list,OS_32);
              if target_info.endian=endian_big then
                begin
                  tmpreg:=hreg1;
                  hreg1:=hreg2;
                  hreg2:=tmpreg;
                end;
              { can we use LDP? }
              if (ref.alignment=4) and
                 (simple_ref_type(A_LDP,OS_32,PF_None,ref)=sr_simple) then
                list.concat(taicpu.op_reg_reg_ref(A_LDP,hreg1,hreg2,ref))
              else
                begin
                  a_load_ref_reg(list,OS_32,OS_32,ref,hreg1);
                  href:=ref;
                  inc(href.offset,4);
                  a_load_ref_reg(list,OS_32,OS_32,href,hreg2);
                end;
              a_load_reg_reg(list,OS_32,OS_64,hreg1,register);
              list.concat(taicpu.op_reg_reg_const_const(A_BFI,register,makeregsize(hreg2,OS_64),32,32));
            end;
          OS_16,OS_S16,
          OS_32,OS_S32:
            begin
              if ref.alignment=2 then
                begin
                  href:=ref;
                  if target_info.endian=endian_big then
                    inc(href.offset,tcgsize2size[fromsize]-2);
                  tmpreg:=getintregister(list,OS_32);
                  a_load_ref_reg(list,OS_16,OS_32,href,tmpreg);
                  tmpreg2:=getintregister(list,OS_32);
                  for i:=1 to (tcgsize2size[fromsize]-1) div 2 do
                    begin
                      if target_info.endian=endian_big then
                        dec(href.offset,2)
                      else
                        inc(href.offset,2);
                      a_load_ref_reg(list,OS_16,OS_32,href,tmpreg2);
                      list.concat(taicpu.op_reg_reg_const_const(A_BFI,tmpreg,tmpreg2,i*16,16));
                    end;
                  a_load_reg_reg(list,fromsize,tosize,tmpreg,register);
                end
              else
                begin
                  href:=ref;
                  if target_info.endian=endian_big then
                    inc(href.offset,tcgsize2size[fromsize]-1);
                  tmpreg:=getintregister(list,OS_32);
                  a_load_ref_reg(list,OS_8,OS_32,href,tmpreg);
                  tmpreg2:=getintregister(list,OS_32);
                  for i:=1 to tcgsize2size[fromsize]-1 do
                    begin
                      if target_info.endian=endian_big then
                        dec(href.offset)
                      else
                        inc(href.offset);
                      a_load_ref_reg(list,OS_8,OS_32,href,tmpreg2);
                      list.concat(taicpu.op_reg_reg_const_const(A_BFI,tmpreg,tmpreg2,i*8,8));
                    end;
                  a_load_reg_reg(list,fromsize,tosize,tmpreg,register);
                end;
            end;
          else
            inherited;
        end;
      end;


    procedure tcgaarch64.a_load_reg_reg(list:TAsmList;fromsize,tosize:tcgsize;reg1,reg2:tregister);
      var
        instr: taicpu;
      begin
        { we use both 32 and 64 bit registers -> insert conversion when when
          we have to truncate/sign extend inside the (32 or 64 bit) register
          holding the value, and when we sign extend from a 32 to a 64 bit
          register }
        if (tcgsize2size[fromsize]>tcgsize2size[tosize]) or
           ((tcgsize2size[fromsize]=tcgsize2size[tosize]) and
            (fromsize<>tosize) and
            not(fromsize in [OS_32,OS_S32,OS_64,OS_S64])) or
           ((fromsize in [OS_S8,OS_S16,OS_S32]) and
            (tosize in [OS_64,OS_S64])) or
           { needs to mask out the sign in the top 16 bits }
           ((fromsize=OS_S8) and
            (tosize=OS_16)) then
          begin
            case tosize of
              OS_8:
                list.concat(setoppostfix(taicpu.op_reg_reg(A_UXT,reg2,makeregsize(reg1,OS_32)),PF_B));
              OS_16:
                list.concat(setoppostfix(taicpu.op_reg_reg(A_UXT,reg2,makeregsize(reg1,OS_32)),PF_H));
              OS_S8:
                list.concat(setoppostfix(taicpu.op_reg_reg(A_SXT,reg2,makeregsize(reg1,OS_32)),PF_B));
              OS_S16:
                list.concat(setoppostfix(taicpu.op_reg_reg(A_SXT,reg2,makeregsize(reg1,OS_32)),PF_H));
              { while "mov wN, wM" automatically inserts a zero-extension and
                hence we could encode a 64->32 bit move like that, the problem
                is that we then can't distinguish 64->32 from 32->32 moves, and
                the 64->32 truncation could be removed altogether... So use a
                different instruction }
              OS_32,
              OS_S32:
                { in theory, reg1 should be 64 bit here (since fromsize>tosize),
                  but because of the way location_force_register() tries to
                  avoid superfluous zero/sign extensions, it's not always the
                  case -> also force reg1 to to 64 bit }
                list.concat(taicpu.op_reg_reg_const_const(A_UBFIZ,makeregsize(reg2,OS_64),makeregsize(reg1,OS_64),0,32));
              OS_64,
              OS_S64:
                list.concat(setoppostfix(taicpu.op_reg_reg(A_SXT,reg2,makeregsize(reg1,OS_32)),PF_W));
              else
                internalerror(2002090901);
            end;
          end
        else
          begin
            { 32 -> 32 bit move implies zero extension (sign extensions have
              been handled above) -> also use for 32 <-> 64 bit moves }
            if not(fromsize in [OS_64,OS_S64]) or
               not(tosize in [OS_64,OS_S64]) then
              instr:=taicpu.op_reg_reg(A_MOV,makeregsize(reg2,OS_32),makeregsize(reg1,OS_32))
            else
              instr:=taicpu.op_reg_reg(A_MOV,reg2,reg1);
            list.Concat(instr);
            { Notify the register allocator that we have written a move instruction so
             it can try to eliminate it. }
            add_move_instruction(instr);
          end;
      end;


    procedure tcgaarch64.a_loadaddr_ref_reg(list: TAsmList; const ref: treference; r: tregister);
      var
         href: treference;
         so: tshifterop;
         op: tasmop;
      begin
        op:=A_LDR;
        href:=ref;
        { simplify as if we're going to perform a regular 64 bit load, using
          "r" as the new base register if possible/necessary }
        make_simple_ref(list,op,OS_ADDR,PF_None,href,r);
        { load literal? }
        if assigned(href.symbol) then
          begin
            if (href.base<>NR_NO) or
               (href.index<>NR_NO) or
               not assigned(href.symboldata) then
              internalerror(2014110912);
            list.concat(taicpu.op_reg_sym_ofs(A_ADR,r,href.symbol,href.offset));
          end
        else
          begin
            if href.index<>NR_NO then
              begin
                if href.shiftmode<>SM_None then
                  begin
                    { "add" supports a supperset of the shift modes supported by
                      load/store instructions }
                    shifterop_reset(so);
                    so.shiftmode:=href.shiftmode;
                    so.shiftimm:=href.shiftimm;
                    list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,r,href.base,href.index,so));
                  end
                else
                  a_op_reg_reg_reg(list,OP_ADD,OS_ADDR,href.index,href.base,r);
              end
            else if href.offset<>0 then
              a_op_const_reg_reg(list,OP_ADD,OS_ADDR,href.offset,href.base,r)
            else
              a_load_reg_reg(list,OS_ADDR,OS_ADDR,href.base,r);
          end;
      end;


    procedure tcgaarch64.a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);
      begin
        internalerror(2014122107)
      end;


    procedure tcgaarch64.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);
      begin
        internalerror(2014122108)
      end;


    procedure tcgaarch64.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);
      begin
        internalerror(2014122109)
      end;


    procedure tcgaarch64.a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister; shuffle: pmmshuffle);
      var
        instr: taicpu;
      begin
        if assigned(shuffle) and
           not shufflescalar(shuffle) then
          internalerror(2014122104);
        if fromsize=tosize then
          begin
            instr:=taicpu.op_reg_reg(A_FMOV,reg2,reg1);
            { Notify the register allocator that we have written a move
              instruction so it can try to eliminate it. }
            add_move_instruction(instr);
            { FMOV cannot generate a floating point exception }
          end
        else
          begin
            if (reg_cgsize(reg1)<>fromsize) or
               (reg_cgsize(reg2)<>tosize) then
              internalerror(2014110913);
            instr:=taicpu.op_reg_reg(A_FCVT,reg2,reg1);
            maybe_check_for_fpu_exception(list);
          end;
        list.Concat(instr);
      end;


    procedure tcgaarch64.a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister; shuffle: pmmshuffle);
       var
         tmpreg: tregister;
       begin
         if assigned(shuffle) and
            not shufflescalar(shuffle) then
           internalerror(2014122105);
         tmpreg:=NR_NO;
         if (fromsize<>tosize) then
           begin
             tmpreg:=reg;
             reg:=getmmregister(list,fromsize);
           end;
         handle_load_store(list,A_LDR,fromsize,PF_None,reg,ref);
         if (fromsize<>tosize) then
           a_loadmm_reg_reg(list,fromsize,tosize,reg,tmpreg,nil);
       end;


     procedure tcgaarch64.a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference; shuffle: pmmshuffle);
       var
         tmpreg: tregister;
       begin
         if assigned(shuffle) and
            not shufflescalar(shuffle) then
           internalerror(2014122106);
         if (fromsize<>tosize) then
           begin
             tmpreg:=getmmregister(list,tosize);
             a_loadmm_reg_reg(list,fromsize,tosize,reg,tmpreg,nil);
             reg:=tmpreg;
           end;
         handle_load_store(list,A_STR,tosize,PF_NONE,reg,ref);
       end;


     procedure tcgaarch64.a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tcgsize; intreg, mmreg: tregister; shuffle: pmmshuffle);
       begin
         if not shufflescalar(shuffle) then
           internalerror(2014122801);
         if not(tcgsize2size[fromsize] in [4,8]) or
            (tcgsize2size[fromsize]<>tcgsize2size[tosize]) then
           internalerror(2014122803);
         list.concat(taicpu.op_reg_reg(A_INS,mmreg,intreg));
       end;


     procedure tcgaarch64.a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tcgsize; mmreg, intreg: tregister; shuffle: pmmshuffle);
       var
         r : tregister;
       begin
         if not shufflescalar(shuffle) then
           internalerror(2014122802);
         if not(tcgsize2size[fromsize] in [4,8]) or
            (tcgsize2size[fromsize]>tcgsize2size[tosize]) then
           internalerror(2014122804);
         if tcgsize2size[fromsize]<tcgsize2size[tosize] then
           r:=makeregsize(intreg,fromsize)
         else
           r:=intreg;
         list.concat(taicpu.op_reg_reg(A_UMOV,r,mmreg));
       end;


    procedure tcgaarch64.a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tcgsize; src, dst: tregister; shuffle: pmmshuffle);
      begin
        case op of
          { "xor Vx,Vx" is used to initialize global regvars to 0 }
          OP_XOR:
            begin
              if (src<>dst) or
                 (reg_cgsize(src)<>size) or
                 assigned(shuffle) then
                internalerror(2015011401);
              case size of
                OS_F32,
                OS_F64:
                  list.concat(taicpu.op_reg_const(A_MOVI,makeregsize(dst,OS_F64),0));
                else
                  internalerror(2015011402);
              end;
            end
          else
            internalerror(2015011403);
        end;
      end;


    procedure tcgaarch64.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tcgsize; src, dst: TRegister);
      var
        bitsize: longint;
      begin
        if srcsize in [OS_64,OS_S64] then
          begin
            bitsize:=64;
          end
        else
          begin
            bitsize:=32;
          end;
        { source is 0 -> dst will have to become 255 }
        list.concat(taicpu.op_reg_const(A_CMP,src,0));
        if reverse then
          begin
            list.Concat(taicpu.op_reg_reg(A_CLZ,makeregsize(dst,srcsize),src));
            { xor 31/63 is the same as setting the lower 5/6 bits to
              "31/63-(lower 5/6 bits of dst)" }
            list.Concat(taicpu.op_reg_reg_const(A_EOR,dst,dst,bitsize-1));
          end
        else
          begin
            list.Concat(taicpu.op_reg_reg(A_RBIT,makeregsize(dst,srcsize),src));
            list.Concat(taicpu.op_reg_reg(A_CLZ,dst,dst));
          end;
        { set dst to -1 if src was 0 }
        list.Concat(taicpu.op_reg_reg_reg_cond(A_CSINV,dst,dst,makeregsize(NR_XZR,dstsize),C_NE));
        { mask the -1 to 255 if src was 0 (anyone find a two-instruction
          branch-free version? All of mine are 3...) }
        list.Concat(setoppostfix(taicpu.op_reg_reg(A_UXT,makeregsize(dst,OS_32),makeregsize(dst,OS_32)),PF_B));
      end;


    procedure tcgaarch64.a_load_reg_ref_unaligned(list: TAsmList; fromsize, tosize: tcgsize; register: tregister; const ref: treference);
      var
        href: treference;
        hreg1, hreg2, tmpreg: tregister;
      begin
        if fromsize in [OS_64,OS_S64] then
          begin
            { split into two 32 bit stores }
            hreg1:=getintregister(list,OS_32);
            hreg2:=getintregister(list,OS_32);
            a_load_reg_reg(list,OS_32,OS_32,makeregsize(register,OS_32),hreg1);
            a_op_const_reg_reg(list,OP_SHR,OS_64,32,register,makeregsize(hreg2,OS_64));
            if target_info.endian=endian_big then
              begin
                tmpreg:=hreg1;
                hreg1:=hreg2;
                hreg2:=tmpreg;
              end;
            { can we use STP? }
            if (ref.alignment=4) and
               (simple_ref_type(A_STP,OS_32,PF_None,ref)=sr_simple) then
              list.concat(taicpu.op_reg_reg_ref(A_STP,hreg1,hreg2,ref))
            else
              begin
                a_load_reg_ref(list,OS_32,OS_32,hreg1,ref);
                href:=ref;
                inc(href.offset,4);
                a_load_reg_ref(list,OS_32,OS_32,hreg2,href);
              end;
          end
       else
         inherited;
      end;


    procedure tcgaarch64.maybeadjustresult(list: TAsmList; op: topcg; size: tcgsize; dst: tregister);
      const
        overflowops = [OP_MUL,OP_IMUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16]) then
          a_load_reg_reg(list,OS_32,size,makeregsize(dst,OS_32),makeregsize(dst,OS_32))
      end;


    procedure tcgaarch64.a_op_const_reg(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; reg: tregister);
      begin
        optimize_op_const(size,op,a);
        case op of
          OP_NONE:
            exit;
          OP_MOVE:
            a_load_const_reg(list,size,a,reg);
          OP_NEG,OP_NOT:
            internalerror(200306011);
          else
            a_op_const_reg_reg(list,op,size,a,reg,reg);
        end;
      end;


    procedure tcgaarch64.a_op_reg_reg(list:TAsmList;op:topcg;size:tcgsize;src,dst:tregister);
      begin
        Case op of
          OP_NEG,
          OP_NOT:
            begin
              list.concat(taicpu.op_reg_reg(TOpCG2AsmOpReg[op],dst,src));
              maybeadjustresult(list,op,size,dst);
            end
          else
            a_op_reg_reg_reg(list,op,size,src,dst,dst);
        end;
      end;


    procedure tcgaarch64.a_op_const_reg_reg(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; src, dst: tregister);
      var
        l: tlocation;
      begin
        a_op_const_reg_reg_checkoverflow(list,op,size,a,src,dst,false,l);
      end;


    procedure tcgaarch64.a_op_reg_reg_reg(list: TAsmList; op: topcg; size: tcgsize; src1, src2, dst: tregister);
      var
        hreg: tregister;
      begin
        { no ROLV opcode... }
        if op=OP_ROL then
          begin
            case size of
              OS_32,OS_S32,
              OS_64,OS_S64:
                begin
                  hreg:=getintregister(list,size);
                  a_load_const_reg(list,size,tcgsize2size[size]*8,hreg);
                  a_op_reg_reg(list,OP_SUB,size,src1,hreg);
                  a_op_reg_reg_reg(list,OP_ROR,size,hreg,src2,dst);
                  exit;
                end;
              else
                internalerror(2014111005);
            end;
          end
        else if (op=OP_ROR) and
           not(size in [OS_32,OS_S32,OS_64,OS_S64]) then
          internalerror(2014111006);
        if TOpCG2AsmOpReg[op]=A_NONE then
          internalerror(2014111007);
        list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOpReg[op],dst,src2,src1));
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgaarch64.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; src, dst: tregister; setflags : boolean; var ovloc : tlocation);
      var
        shiftcountmask: longint;
        constreg: tregister;
      begin
        { add/sub instructions have only positive immediate operands }
        if (op in [OP_ADD,OP_SUB]) and
           (a<0) then
          begin
            if op=OP_ADD then
              op:=op_SUB
            else
              op:=OP_ADD;
            { avoid range/overflow error in case a = low(tcgint) }
{$push}{$r-}{$q-}
            a:=-a;
{$pop}
          end;
        ovloc.loc:=LOC_VOID;
        optimize_op_const(size,op,a);
        case op of
          OP_NONE:
            begin
              a_load_reg_reg(list,size,size,src,dst);
              exit;
            end;
          OP_MOVE:
            begin
              a_load_const_reg(list,size,a,dst);
              exit;
            end;
          else
            ;
        end;
        case op of
          OP_ADD,
          OP_SUB:
            begin
              handle_reg_imm12_reg(list,TOpCG2AsmOpImm[op],size,src,a,dst,NR_NO,setflags,true);
              { on a 64 bit target, overflows with smaller data types
                are handled via range errors }
              if setflags and
                 (size in [OS_64,OS_S64]) then
                begin
                  location_reset(ovloc,LOC_FLAGS,OS_8);
                  if size=OS_64 then
                    if op=OP_ADD then
                      ovloc.resflags:=F_CS
                    else
                      ovloc.resflags:=F_CC
                  else
                    ovloc.resflags:=F_VS;
                end;
            end;
          OP_OR,
          OP_AND,
          OP_XOR:
            begin
              if not(size in [OS_64,OS_S64]) then
                a:=cardinal(a);
              if is_shifter_const(a,size) then
                list.concat(taicpu.op_reg_reg_const(TOpCG2AsmOpReg[op],dst,src,a))
              else
                begin
                  constreg:=getintregister(list,size);
                  a_load_const_reg(list,size,a,constreg);
                  a_op_reg_reg_reg(list,op,size,constreg,src,dst);
                end;
            end;
          OP_SHL,
          OP_SHR,
          OP_SAR:
            begin
              if size in [OS_64,OS_S64] then
                shiftcountmask:=63
              else
                shiftcountmask:=31;
              if (a and shiftcountmask)<>0 Then
                list.concat(taicpu.op_reg_reg_const(
                  TOpCG2AsmOpImm[Op],dst,src,a and shiftcountmask))
              else
                a_load_reg_reg(list,size,size,src,dst);
              if (a and not(tcgint(shiftcountmask)))<>0 then
                internalError(2014112101);
            end;
          OP_ROL,
          OP_ROR:
            begin
              case size of
                OS_32,OS_S32:
                  if (a and not(tcgint(31)))<>0 then
                    internalError(2014112102);
                OS_64,OS_S64:
                  if (a and not(tcgint(63)))<>0 then
                    internalError(2014112103);
                else
                  internalError(2014112104);
              end;
              { there's only a ror opcode }
              if op=OP_ROL then
                a:=(tcgsize2size[size]*8)-a;
              list.concat(taicpu.op_reg_reg_const(A_ROR,dst,src,a));
            end;
          OP_MUL,
          OP_IMUL,
          OP_DIV,
          OP_IDIV:
            begin
              constreg:=getintregister(list,size);
              a_load_const_reg(list,size,a,constreg);
              a_op_reg_reg_reg_checkoverflow(list,op,size,constreg,src,dst,setflags,ovloc);
            end;
          else
            internalerror(2014111403);
        end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgaarch64.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: topcg; size: tcgsize; src1, src2, dst: tregister; setflags : boolean; var ovloc : tlocation);
      var
        tmpreg1, tmpreg2: tregister;
      begin
        ovloc.loc:=LOC_VOID;
        { overflow can only occur with 64 bit calculations on 64 bit cpus }
        if setflags and
           (size in [OS_64,OS_S64]) then
          begin
            case op of
              OP_ADD,
              OP_SUB:
                begin
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(TOpCG2AsmOpReg[op],dst,src2,src1),PF_S));
                  ovloc.loc:=LOC_FLAGS;
                  if size=OS_64 then
                    if op=OP_ADD then
                      ovloc.resflags:=F_CS
                    else
                      ovloc.resflags:=F_CC
                  else
                    ovloc.resflags:=F_VS;
                  { finished }
                  exit;
                end;
              OP_MUL:
                begin
                  { check whether the upper 64 bit of the 128 bit product is 0 }
                  tmpreg1:=getintregister(list,OS_64);
                  list.concat(taicpu.op_reg_reg_reg(A_UMULH,tmpreg1,src2,src1));
                  list.concat(taicpu.op_reg_const(A_CMP,tmpreg1,0));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags:=F_NE;
                  { still have to perform the actual multiplication  }
                end;
              OP_IMUL:
                begin
                  { check whether the upper 64 bits of the 128 bit multiplication
                    result have the same value as the replicated sign bit of the
                    lower 64 bits }
                  tmpreg1:=getintregister(list,OS_64);
                  list.concat(taicpu.op_reg_reg_reg(A_SMULH,tmpreg1,src2,src1));
                  { calculate lower 64 bits (afterwards, because dst may be
                    equal to src1 or src2) }
                  a_op_reg_reg_reg(list,op,size,src1,src2,dst);
                  { replicate sign bit }
                  tmpreg2:=getintregister(list,OS_64);
                  a_op_const_reg_reg(list,OP_SAR,OS_S64,63,dst,tmpreg2);
                  list.concat(taicpu.op_reg_reg(A_CMP,tmpreg1,tmpreg2));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags:=F_NE;
                  { finished }
                  exit;
                end;
              OP_IDIV,
              OP_DIV:
                begin
                  { not handled here, needs div-by-zero check (dividing by zero
                    just gives a 0 result on aarch64), and low(int64) div -1
                    check for overflow) }
                  internalerror(2014122101);
                end;
              else
                internalerror(2019050936);
            end;
          end;
        a_op_reg_reg_reg(list,op,size,src1,src2,dst);
      end;



  {*************** compare instructructions ****************}

    procedure tcgaarch64.a_cmp_const_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel);
      var
        op: tasmop;
      begin
        if a>=0 then
          op:=A_CMP
        else
          op:=A_CMN;
        { avoid range/overflow error in case a=low(tcgint) }
{$push}{$r-}{$q-}
        handle_reg_imm12_reg(list,op,size,reg,abs(a),NR_XZR,NR_NO,false,false);
{$pop}
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgaarch64.a_cmp_reg_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; reg1,reg2: tregister; l: tasmlabel);
      begin
        list.concat(taicpu.op_reg_reg(A_CMP,reg2,reg1));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgaarch64.a_jmp_always(list: TAsmList; l: TAsmLabel);
      var
        ai: taicpu;
      begin
        ai:=TAiCpu.op_sym(A_B,current_asmdata.RefAsmSymbol(l.name,AT_FUNCTION));
        ai.is_jmp:=true;
        list.Concat(ai);
      end;


    procedure tcgaarch64.a_jmp_name(list: TAsmList; const s: string);
      var
        ai: taicpu;
      begin
        ai:=TAiCpu.op_sym(A_B,current_asmdata.RefAsmSymbol(s,AT_FUNCTION));
        ai.is_jmp:=true;
        list.Concat(ai);
      end;


    procedure tcgaarch64.a_jmp_cond(list: TAsmList; cond: TOpCmp; l: TAsmLabel);
      var
        ai: taicpu;
      begin
        ai:=TAiCpu.op_sym(A_B,l);
        ai.is_jmp:=true;
        ai.SetCondition(TOpCmp2AsmCond[cond]);
        list.Concat(ai);
      end;


    procedure tcgaarch64.a_jmp_flags(list: TAsmList; const f: tresflags; l: tasmlabel);
      var
        ai : taicpu;
      begin
        ai:=Taicpu.op_sym(A_B,l);
        ai.is_jmp:=true;
        ai.SetCondition(flags_to_cond(f));
        list.Concat(ai);
      end;


    procedure tcgaarch64.g_flags2reg(list: TAsmList; size: tcgsize; const f: tresflags; reg: tregister);
      begin
        list.concat(taicpu.op_reg_cond(A_CSET,reg,flags_to_cond(f)));
      end;


    procedure tcgaarch64.g_overflowcheck(list: TAsmList; const loc: tlocation; def: tdef);
      begin
        { we need an explicit overflow location, because there are many
          possibilities (not just the overflow flag, which is only used for
          signed add/sub) }
        internalerror(2014112303);
      end;


    procedure tcgaarch64.g_overflowcheck_loc(list: TAsmList; const loc: tlocation; def: tdef; ovloc : tlocation);
      var
        hl : tasmlabel;
        hflags : tresflags;
      begin
        if not(cs_check_overflow in current_settings.localswitches) then
          exit;
        current_asmdata.getjumplabel(hl);
        case ovloc.loc of
          LOC_FLAGS:
            begin
              hflags:=ovloc.resflags;
              inverse_flags(hflags);
              cg.a_jmp_flags(list,hflags,hl);
            end;
          else
            internalerror(2014112304);
        end;
        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;

  { *********** entry/exit code and address loading ************ }

    function tcgaarch64.save_regs(list: TAsmList; rt: tregistertype; lowsr, highsr: tsuperregister; sub: tsubregister): longint;
      var
        ref: treference;
        sr: tsuperregister;
        pairreg: tregister;
      begin
        result:=0;
        reference_reset_base(ref,NR_SP,-16,ctempposinvalid,16,[]);
        ref.addressmode:=AM_PREINDEXED;
        pairreg:=NR_NO;
        { store all used registers pairwise }
        for sr:=lowsr to highsr do
          if sr in rg[rt].used_in_proc then
            if pairreg=NR_NO then
              pairreg:=newreg(rt,sr,sub)
            else
              begin
                inc(result,16);
                list.concat(taicpu.op_reg_reg_ref(A_STP,pairreg,newreg(rt,sr,sub),ref));
                pairreg:=NR_NO
              end;
        { one left -> store twice (stack must be 16 bytes aligned) }
        if pairreg<>NR_NO then
          begin
            list.concat(taicpu.op_reg_reg_ref(A_STP,pairreg,pairreg,ref));
            inc(result,16);
          end;
      end;


    procedure FixupOffsets(p:TObject;arg:pointer);
      var
        sym: tabstractnormalvarsym absolute p;
      begin
        if (tsym(p).typ in [paravarsym,localvarsym]) and
          (sym.localloc.loc=LOC_REFERENCE) and
          (sym.localloc.reference.base=NR_STACK_POINTER_REG) then
          begin
            sym.localloc.reference.base:=NR_FRAME_POINTER_REG;
            dec(sym.localloc.reference.offset,PLongint(arg)^);
          end;
      end;


    procedure tcgaarch64.g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);
      var
        ref: treference;
        totalstackframesize: longint;
      begin
        if nostackframe then
          exit;
        { stack pointer has to be aligned to 16 bytes at all times }
        localsize:=align(localsize,16);

        { save stack pointer and return address }
        reference_reset_base(ref,NR_SP,-16,ctempposinvalid,16,[]);
        ref.addressmode:=AM_PREINDEXED;
        list.concat(taicpu.op_reg_reg_ref(A_STP,NR_FP,NR_LR,ref));
        { initialise frame pointer }
        a_load_reg_reg(list,OS_ADDR,OS_ADDR,NR_SP,NR_FP);

        totalstackframesize:=localsize;
        { save modified integer registers }
        inc(totalstackframesize,
          save_regs(list,R_INTREGISTER,RS_X19,RS_X28,R_SUBWHOLE));
        { only the lower 64 bits of the modified vector registers need to be
          saved; if the caller needs the upper 64 bits, it has to save them
          itself }
        inc(totalstackframesize,
          save_regs(list,R_MMREGISTER,RS_D8,RS_D15,R_SUBMMD));

        { allocate stack space }
        if localsize<>0 then
          begin
            localsize:=align(localsize,16);
            current_procinfo.final_localsize:=localsize;
            handle_reg_imm12_reg(list,A_SUB,OS_ADDR,NR_SP,localsize,NR_SP,NR_IP0,false,true);
          end;
        { By default, we use the frame pointer to access parameters passed via
          the stack and the stack pointer to address local variables and temps
          because
           a) we can use bigger positive than negative offsets (so accessing
              locals via negative offsets from the frame pointer would be less
              efficient)
           b) we don't know the local size while generating the code, so
              accessing the parameters via the stack pointer is not possible
              without copying them
          The problem with this is the get_frame() intrinsic:
           a) it must return the same value as what we pass as parentfp
              parameter, since that's how it's used in the TP-style objects unit
           b) its return value must usable to access all local data from a
              routine (locals and parameters), since it's all the nested
              routines have access to
           c) its return value must be usable to construct a backtrace, as it's
              also used by the exception handling routines

          The solution we use here, based on something similar that's done in
          the MIPS port, is to generate all accesses to locals in the routine
          itself SP-relative, and then after the code is generated and the local
          size is known (namely, here), we change all SP-relative variables/
          parameters into FP-relative ones. This means that they'll be accessed
          less efficiently from nested routines, but those accesses are indirect
          anyway and at least this way they can be accessed at all
        }
        if current_procinfo.has_nestedprocs then
          begin
            current_procinfo.procdef.localst.SymList.ForEachCall(@FixupOffsets,@totalstackframesize);
            current_procinfo.procdef.parast.SymList.ForEachCall(@FixupOffsets,@totalstackframesize);
          end;
      end;


    procedure tcgaarch64.g_maybe_got_init(list : TAsmList);
      begin
        { nothing to do on Darwin or Linux }
      end;


    procedure tcgaarch64.g_restore_registers(list:TAsmList);
      begin
        { done in g_proc_exit }
      end;


    procedure tcgaarch64.load_regs(list: TAsmList; rt: tregistertype; lowsr, highsr: tsuperregister; sub: tsubregister);
      var
        ref: treference;
        sr, highestsetsr: tsuperregister;
        pairreg: tregister;
        regcount: longint;
      begin
        reference_reset_base(ref,NR_SP,16,ctempposinvalid,16,[]);
        ref.addressmode:=AM_POSTINDEXED;
        { highest reg stored twice? }
        regcount:=0;
        highestsetsr:=RS_NO;
        for sr:=lowsr to highsr do
          if sr in rg[rt].used_in_proc then
            begin
              inc(regcount);
              highestsetsr:=sr;
            end;
        if odd(regcount) then
          begin
            list.concat(taicpu.op_reg_ref(A_LDR,newreg(rt,highestsetsr,sub),ref));
            highestsetsr:=pred(highestsetsr);
          end;
        { load all (other) used registers pairwise }
        pairreg:=NR_NO;
        for sr:=highestsetsr downto lowsr do
          if sr in rg[rt].used_in_proc then
            if pairreg=NR_NO then
              pairreg:=newreg(rt,sr,sub)
            else
              begin
                list.concat(taicpu.op_reg_reg_ref(A_LDP,newreg(rt,sr,sub),pairreg,ref));
                pairreg:=NR_NO
              end;
        { There can't be any register left }
        if pairreg<>NR_NO then
          internalerror(2014112602);
      end;



    procedure tcgaarch64.g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);
      var
        ref: treference;
        regsstored: boolean;
        sr: tsuperregister;
      begin
        if not nostackframe then
          begin
            { if no registers have been stored, we don't have to subtract the
              allocated temp space from the stack pointer }
            regsstored:=false;
            for sr:=RS_X19 to RS_X28 do
              if sr in rg[R_INTREGISTER].used_in_proc then
                begin
                  regsstored:=true;
                  break;
                end;
            if not regsstored then
              for sr:=RS_D8 to RS_D15 do
                if sr in rg[R_MMREGISTER].used_in_proc then
                  begin
                    regsstored:=true;
                    break;
                  end;
            { restore registers (and stack pointer) }
            if regsstored then
              begin
                if current_procinfo.final_localsize<>0 then
                  handle_reg_imm12_reg(list,A_ADD,OS_ADDR,NR_SP,current_procinfo.final_localsize,NR_SP,NR_IP0,false,true);
                load_regs(list,R_MMREGISTER,RS_D8,RS_D15,R_SUBMMD);
                load_regs(list,R_INTREGISTER,RS_X19,RS_X28,R_SUBWHOLE);
              end
            else if current_procinfo.final_localsize<>0 then
              { restore stack pointer }
              a_load_reg_reg(list,OS_ADDR,OS_ADDR,NR_FP,NR_SP);

            { restore framepointer and return address }
            reference_reset_base(ref,NR_SP,16,ctempposinvalid,16,[]);
            ref.addressmode:=AM_POSTINDEXED;
            list.concat(taicpu.op_reg_reg_ref(A_LDP,NR_FP,NR_LR,ref));
          end;

        { return }
        list.concat(taicpu.op_none(A_RET));
      end;


    procedure tcgaarch64.g_save_registers(list : TAsmList);
      begin
        { done in g_proc_entry }
      end;


    { ************* concatcopy ************ }

    procedure tcgaarch64.g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        paraloc1,paraloc2,paraloc3 : TCGPara;
        pd : tprocdef;
      begin
        pd:=search_system_proc('MOVE');
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
        paramanager.getcgtempparaloc(list,pd,1,paraloc1);
        paramanager.getcgtempparaloc(list,pd,2,paraloc2);
        paramanager.getcgtempparaloc(list,pd,3,paraloc3);
        a_load_const_cgpara(list,OS_SINT,len,paraloc3);
        a_loadaddr_ref_cgpara(list,dest,paraloc2);
        a_loadaddr_ref_cgpara(list,source,paraloc1);
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        alloccpuregisters(list,R_MMREGISTER,paramanager.get_volatile_registers_mm(pocall_default));
        a_call_name(list,'FPC_MOVE',false);
        dealloccpuregisters(list,R_MMREGISTER,paramanager.get_volatile_registers_mm(pocall_default));
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcgaarch64.g_concatcopy(list: TAsmList; const source, dest: treference; len: tcgint);

      var
        sourcebasereplaced, destbasereplaced: boolean;

      { get optimal memory operation to use for loading/storing data
        in an unrolled loop }
      procedure getmemop(scaledop, unscaledop: tasmop; const startref, endref: treference; opsize: tcgsize; postfix: toppostfix; out memop: tasmop; out needsimplify: boolean);
        begin
          if (simple_ref_type(scaledop,opsize,postfix,startref)=sr_simple) and
             (simple_ref_type(scaledop,opsize,postfix,endref)=sr_simple) then
            begin
              memop:=unscaledop;
              needsimplify:=true;
            end
          else if (unscaledop<>A_NONE) and
             (simple_ref_type(unscaledop,opsize,postfix,startref)=sr_simple) and
             (simple_ref_type(unscaledop,opsize,postfix,endref)=sr_simple) then
            begin
              memop:=unscaledop;
              needsimplify:=false;
            end
          else
            begin
              memop:=scaledop;
              needsimplify:=true;
            end;
        end;

      { adjust the offset and/or addressing mode after a load/store so it's
        correct for the next one of the same size }
      procedure updaterefafterloadstore(var ref: treference; oplen: longint);
        begin
          case ref.addressmode of
            AM_OFFSET:
              inc(ref.offset,oplen);
            AM_POSTINDEXED:
              { base register updated by instruction, next offset can remain
                the same }
              ;
            AM_PREINDEXED:
              begin
                { base register updated by instruction -> next instruction can
                  use post-indexing with offset = sizeof(operation) }
                ref.offset:=0;
                ref.addressmode:=AM_OFFSET;
              end;
          end;
        end;

      { generate a load/store and adjust the reference offset to the next
        memory location if necessary }
      procedure genloadstore(list: TAsmList; op: tasmop; reg: tregister; var ref: treference; postfix: toppostfix; opsize: tcgsize);
        begin
          list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),postfix));
          updaterefafterloadstore(ref,tcgsize2size[opsize]);
        end;

      { generate a dual load/store (ldp/stp) and adjust the reference offset to
        the next memory location if necessary }
      procedure gendualloadstore(list: TAsmList; op: tasmop; reg1, reg2: tregister; var ref: treference; postfix: toppostfix; opsize: tcgsize);
        begin
          list.concat(setoppostfix(taicpu.op_reg_reg_ref(op,reg1,reg2,ref),postfix));
          updaterefafterloadstore(ref,tcgsize2size[opsize]*2);
        end;

      { turn a reference into a pre- or post-indexed reference for use in a
        load/store of a particular size }
      procedure makesimpleforcopy(list: TAsmList; var scaledop: tasmop; opsize: tcgsize; postfix: toppostfix; forcepostindexing: boolean; var ref: treference; var basereplaced: boolean);
        var
          tmpreg: tregister;
          scaledoffset: longint;
          orgaddressmode: taddressmode;
        begin
          scaledoffset:=tcgsize2size[opsize];
          if scaledop in [A_LDP,A_STP] then
            scaledoffset:=scaledoffset*2;
          { can we use the reference as post-indexed without changes? }
          if forcepostindexing then
            begin
              orgaddressmode:=ref.addressmode;
              ref.addressmode:=AM_POSTINDEXED;
              if (orgaddressmode=AM_POSTINDEXED) or
                 ((ref.offset=0) and
                  (simple_ref_type(scaledop,opsize,postfix,ref)=sr_simple)) then
                begin
                  { just change the post-indexed offset to the access size }
                  ref.offset:=scaledoffset;
                  { and replace the base register if that didn't happen yet
                    (could be sp or a regvar) }
                  if not basereplaced then
                    begin
                      tmpreg:=getaddressregister(list);
                      a_load_reg_reg(list,OS_ADDR,OS_ADDR,ref.base,tmpreg);
                      ref.base:=tmpreg;
                      basereplaced:=true;
                    end;
                  exit;
                end;
              ref.addressmode:=orgaddressmode;
            end;
{$ifdef dummy}
          This could in theory be useful in case you have a concatcopy from
          e.g. x1+255 to x1+267 *and* the reference is aligned, but this seems
          very unlikely. Disabled because it still needs fixes, as it
          also generates pre-indexed loads right now at the very end for the
          left-over gencopies

          { can we turn it into a pre-indexed reference for free? (after the
            first operation, it will be turned into an offset one) }
          if not forcepostindexing and
             (ref.offset<>0) then
            begin
              orgaddressmode:=ref.addressmode;
              ref.addressmode:=AM_PREINDEXED;
              tmpreg:=ref.base;
              if not basereplaced and
                 (ref.base=tmpreg) then
                begin
                  tmpreg:=getaddressregister(list);
                  a_load_reg_reg(list,OS_ADDR,OS_ADDR,ref.base,tmpreg);
                  ref.base:=tmpreg;
                  basereplaced:=true;
                end;
              if simple_ref_type(scaledop,opsize,postfix,ref)<>sr_simple then
                make_simple_ref(list,scaledop,opsize,postfix,ref,NR_NO);
              exit;
            end;
{$endif dummy}
          if not forcepostindexing then
            begin
              ref.addressmode:=AM_OFFSET;
              make_simple_ref(list,scaledop,opsize,postfix,ref,NR_NO);
              { this may still cause problems if the final offset is no longer
                a simple ref; it's a bit complicated to pass all information
                through at all places and check that here, so play safe: we
                currently never generate unrolled copies for more than 64
                bytes (32 with non-double-register copies) }
              if ref.index=NR_NO then
                begin
                  if ((scaledop in [A_LDP,A_STP]) and
                      (ref.offset<((64-8)*tcgsize2size[opsize]))) or
                     ((scaledop in [A_LDUR,A_STUR]) and
                      (ref.offset<(255-8*tcgsize2size[opsize]))) or
                     ((scaledop in [A_LDR,A_STR]) and
                      (ref.offset<((4096-8)*tcgsize2size[opsize]))) then
                    exit;
                end;
            end;
          tmpreg:=getaddressregister(list);
          a_loadaddr_ref_reg(list,ref,tmpreg);
          basereplaced:=true;
          if forcepostindexing then
            begin
              reference_reset_base(ref,tmpreg,scaledoffset,ref.temppos,ref.alignment,ref.volatility);
              ref.addressmode:=AM_POSTINDEXED;
            end
          else
            begin
              reference_reset_base(ref,tmpreg,0,ref.temppos,ref.alignment,ref.volatility);
              ref.addressmode:=AM_OFFSET;
            end
        end;

      { prepare a reference for use by gencopy. This is done both after the
        unrolled and regular copy loop -> get rid of post-indexing mode, make
        sure ref is valid }
      procedure preparecopy(list: tasmlist; scaledop, unscaledop: tasmop; var ref: treference; opsize: tcgsize; postfix: toppostfix; out op: tasmop; var basereplaced: boolean);
        var
          simplify: boolean;
        begin
          if ref.addressmode=AM_POSTINDEXED then
            ref.offset:=tcgsize2size[opsize];
          getmemop(scaledop,scaledop,ref,ref,opsize,postfix,op,simplify);
          if simplify then
            begin
              makesimpleforcopy(list,scaledop,opsize,postfix,false,ref,basereplaced);
              op:=scaledop;
            end;
        end;

      { generate a copy from source to dest of size opsize/postfix }
      procedure gencopy(list: TAsmList; var source, dest: treference; postfix: toppostfix; opsize: tcgsize);
        var
          reg: tregister;
          loadop, storeop: tasmop;
        begin
          preparecopy(list,A_LDR,A_LDUR,source,opsize,postfix,loadop,sourcebasereplaced);
          preparecopy(list,A_STR,A_STUR,dest,opsize,postfix,storeop,destbasereplaced);
          reg:=getintregister(list,opsize);
          genloadstore(list,loadop,reg,source,postfix,opsize);
          genloadstore(list,storeop,reg,dest,postfix,opsize);
        end;


      { copy the leftovers after an unrolled or regular copy loop }
      procedure gencopyleftovers(list: TAsmList; var source, dest: treference; len: longint);
        begin
          { stop post-indexing if we did so in the loop, since in that case all
            offsets definitely can be represented now }
          if source.addressmode=AM_POSTINDEXED then
            begin
              source.addressmode:=AM_OFFSET;
              source.offset:=0;
            end;
          if dest.addressmode=AM_POSTINDEXED then
            begin
              dest.addressmode:=AM_OFFSET;
              dest.offset:=0;
            end;
          { transfer the leftovers }
          if len>=8 then
            begin
              dec(len,8);
              gencopy(list,source,dest,PF_NONE,OS_64);
            end;
          if len>=4 then
            begin
              dec(len,4);
              gencopy(list,source,dest,PF_NONE,OS_32);
            end;
          if len>=2 then
            begin
              dec(len,2);
              gencopy(list,source,dest,PF_H,OS_16);
            end;
          if len>=1 then
            begin
              dec(len);
              gencopy(list,source,dest,PF_B,OS_8);
            end;
        end;


      const
        { load_length + loop dec + cbnz }
        loopoverhead=12;
        { loop overhead + load + store }
        totallooplen=loopoverhead + 8;
      var
        totalalign: longint;
        maxlenunrolled: tcgint;
        loadop, storeop: tasmop;
        opsize: tcgsize;
        postfix: toppostfix;
        tmpsource, tmpdest: treference;
        scaledstoreop, unscaledstoreop,
        scaledloadop, unscaledloadop: tasmop;
        regs: array[1..8] of tregister;
        countreg: tregister;
        i, regcount: longint;
        hl: tasmlabel;
        simplifysource, simplifydest: boolean;
      begin
        if len=0 then
          exit;
        sourcebasereplaced:=false;
        destbasereplaced:=false;
        { maximum common alignment }
        totalalign:=max(1,newalignment(source.alignment,dest.alignment));
        { use a simple load/store? }
        if (len in [1,2,4,8]) and
           ((totalalign>=(len div 2)) or
            (source.alignment=len) or
            (dest.alignment=len)) then
          begin
            opsize:=int_cgsize(len);
            a_load_ref_ref(list,opsize,opsize,source,dest);
            exit;
          end;

        { alignment > length is not useful, and would break some checks below }
        while totalalign>len do
          totalalign:=totalalign div 2;

        { operation sizes to use based on common alignment }
        case totalalign of
          1:
            begin
              postfix:=PF_B;
              opsize:=OS_8;
            end;
          2:
            begin
              postfix:=PF_H;
              opsize:=OS_16;
            end;
          4:
            begin
              postfix:=PF_None;
              opsize:=OS_32;
            end
          else
            begin
              totalalign:=8;
              postfix:=PF_None;
              opsize:=OS_64;
            end;
        end;
        { maximum length to handled with an unrolled loop (4 loads + 4 stores) }
        maxlenunrolled:=min(totalalign,8)*4;
        { ldp/stp -> 2 registers per instruction }
        if (totalalign>=4) and
           (len>=totalalign*2) then
          begin
            maxlenunrolled:=maxlenunrolled*2;
            scaledstoreop:=A_STP;
            scaledloadop:=A_LDP;
            unscaledstoreop:=A_NONE;
            unscaledloadop:=A_NONE;
          end
        else
          begin
            scaledstoreop:=A_STR;
            scaledloadop:=A_LDR;
            unscaledstoreop:=A_STUR;
            unscaledloadop:=A_LDUR;
          end;
        { we only need 4 instructions extra to call FPC_MOVE }
        if cs_opt_size in current_settings.optimizerswitches then
          maxlenunrolled:=maxlenunrolled div 2;
        if (len>maxlenunrolled) and
           (len>totalalign*8) then
          begin
            g_concatcopy_move(list,source,dest,len);
            exit;
          end;

        simplifysource:=true;
        simplifydest:=true;
        tmpsource:=source;
        tmpdest:=dest;
        { can we directly encode all offsets in an unrolled loop? }
        if len<=maxlenunrolled then
          begin
{$ifdef extdebug}
            list.concat(tai_comment.Create(strpnew('concatcopy unrolled loop; len/opsize/align: '+tostr(len)+'/'+tostr(tcgsize2size[opsize])+'/'+tostr(totalalign))));
{$endif extdebug}
            { the leftovers will be handled separately -> -(len mod opsize) }
            inc(tmpsource.offset,len-(len mod tcgsize2size[opsize]));
            { additionally, the last regular load/store will be at
              offset+len-opsize (if len-(len mod opsize)>len) }
            if tmpsource.offset>source.offset then
              dec(tmpsource.offset,tcgsize2size[opsize]);
            getmemop(scaledloadop,unscaledloadop,source,tmpsource,opsize,postfix,loadop,simplifysource);
            inc(tmpdest.offset,len-(len mod tcgsize2size[opsize]));
            if tmpdest.offset>dest.offset then
              dec(tmpdest.offset,tcgsize2size[opsize]);
            getmemop(scaledstoreop,unscaledstoreop,dest,tmpdest,opsize,postfix,storeop,simplifydest);
            tmpsource:=source;
            tmpdest:=dest;
            { if we can't directly encode all offsets, simplify }
            if simplifysource then
              begin
                loadop:=scaledloadop;
                makesimpleforcopy(list,loadop,opsize,postfix,false,tmpsource,sourcebasereplaced);
              end;
            if simplifydest then
              begin
                storeop:=scaledstoreop;
                makesimpleforcopy(list,storeop,opsize,postfix,false,tmpdest,destbasereplaced);
              end;
            regcount:=len div tcgsize2size[opsize];
            { in case we transfer two registers at a time, we copy an even
              number of registers }
            if loadop=A_LDP then
              regcount:=regcount and not(1);
            { initialise for dfa }
            regs[low(regs)]:=NR_NO;
            { max 4 loads/stores -> max 8 registers (in case of ldp/stdp) }
            for i:=1 to regcount do
              regs[i]:=getintregister(list,opsize);
            if loadop=A_LDP then
              begin
                { load registers }
                for i:=1 to (regcount div 2) do
                  gendualloadstore(list,loadop,regs[i*2-1],regs[i*2],tmpsource,postfix,opsize);
                { store registers }
                for i:=1 to (regcount div 2) do
                  gendualloadstore(list,storeop,regs[i*2-1],regs[i*2],tmpdest,postfix,opsize);
              end
            else
              begin
                for i:=1 to regcount do
                  genloadstore(list,loadop,regs[i],tmpsource,postfix,opsize);
                for i:=1 to regcount do
                  genloadstore(list,storeop,regs[i],tmpdest,postfix,opsize);
              end;
            { leftover }
            len:=len-regcount*tcgsize2size[opsize];
{$ifdef extdebug}
            list.concat(tai_comment.Create(strpnew('concatcopy unrolled loop leftover: '+tostr(len))));
{$endif extdebug}
          end
        else
          begin
{$ifdef extdebug}
            list.concat(tai_comment.Create(strpnew('concatcopy regular loop; len/align: '+tostr(len)+'/'+tostr(totalalign))));
{$endif extdebug}
            { regular loop -> definitely use post-indexing }
            loadop:=scaledloadop;
            makesimpleforcopy(list,loadop,opsize,postfix,true,tmpsource,sourcebasereplaced);
            storeop:=scaledstoreop;
            makesimpleforcopy(list,storeop,opsize,postfix,true,tmpdest,destbasereplaced);
            current_asmdata.getjumplabel(hl);
            countreg:=getintregister(list,OS_32);
            if loadop=A_LDP then
              a_load_const_reg(list,OS_32,len div tcgsize2size[opsize]*2,countreg)
            else
              a_load_const_reg(list,OS_32,len div tcgsize2size[opsize],countreg);
            a_label(list,hl);
            a_op_const_reg(list,OP_SUB,OS_32,1,countreg);
            if loadop=A_LDP then
              begin
                regs[1]:=getintregister(list,opsize);
                regs[2]:=getintregister(list,opsize);
                gendualloadstore(list,loadop,regs[1],regs[2],tmpsource,postfix,opsize);
                gendualloadstore(list,storeop,regs[1],regs[2],tmpdest,postfix,opsize);
              end
            else
              begin
                regs[1]:=getintregister(list,opsize);
                genloadstore(list,loadop,regs[1],tmpsource,postfix,opsize);
                genloadstore(list,storeop,regs[1],tmpdest,postfix,opsize);
              end;
            list.concat(taicpu.op_reg_sym_ofs(A_CBNZ,countreg,hl,0));
            len:=len mod tcgsize2size[opsize];
          end;
        gencopyleftovers(list,tmpsource,tmpdest,len);
      end;


    procedure tcgaarch64.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
      begin
        { This method is integrated into g_intf_wrapper and shouldn't be called separately }
        InternalError(2013020102);
      end;


    procedure tcgaarch64.g_check_for_fpu_exception(list: TAsmList;force,clear : boolean);
      var
        r : TRegister;
        ai: taicpu;
        l1,l2: TAsmLabel;
      begin
        { so far, we assume all flavours of AArch64 need explicit floating point exception checking }
        if ((cs_check_fpu_exceptions in current_settings.localswitches) and
            (force or current_procinfo.FPUExceptionCheckNeeded)) then
          begin
            r:=getintregister(list,OS_INT);
            list.concat(taicpu.op_reg_reg(A_MRS,r,NR_FPSR));
            list.concat(taicpu.op_reg_const(A_TST,r,$1f));
            current_asmdata.getjumplabel(l1);
            current_asmdata.getjumplabel(l2);
            ai:=taicpu.op_sym(A_B,l1);
            ai.is_jmp:=true;
            ai.condition:=C_NE;
            list.concat(ai);
            list.concat(taicpu.op_reg_const(A_TST,r,$80));
            ai:=taicpu.op_sym(A_B,l2);
            ai.is_jmp:=true;
            ai.condition:=C_EQ;
            list.concat(ai);
            a_label(list,l1);
            alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
            cg.a_call_name(list,'FPC_THROWFPUEXCEPTION',false);
            dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
            a_label(list,l2);
            if clear then
              current_procinfo.FPUExceptionCheckNeeded:=false;
          end;
      end;


    procedure create_codegen;
      begin
        cg:=tcgaarch64.Create;
        cg128:=tcg128.Create;
      end;

end.
