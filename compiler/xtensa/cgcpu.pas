{
    Copyright (c) 2014 by Jonas Maebe

    This unit implements the code generator for Xtensa

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
       rgcpu,
       cg64f32;

    type
      tcgcpu=class(tcg)
      private
        procedure fixref(list : TAsmList; var ref : treference);
        procedure g_concatcopy_move(list : tasmlist; const Source,dest : treference; len : tcgint);
      public
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

       { move instructions }
        procedure a_load_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);override;
        procedure a_load_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister;const ref: TReference);override;
        procedure a_load_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: TReference; reg: tregister);override;
        procedure a_load_const_reg(list: TAsmList; size: tcgsize; a: tcgint; reg: tregister);override;
        procedure a_loadaddr_ref_reg(list: TAsmList; const ref: TReference; r: tregister);override;

        procedure a_op_reg_reg(list: TAsmList; op: topcg; size: tcgsize; src, dst: tregister);override;
        procedure a_op_const_reg(list: TAsmList; op: topcg; size: tcgsize; a: tcgint; reg: tregister);override;
        procedure a_op_reg_reg_reg(list: TAsmList; op: topcg; size: tcgsize; src1, src2, dst: tregister);override;
        procedure a_op_const_reg_reg(list : TAsmList; op : TOpCg; size : tcgsize; a : tcgint; src,dst : tregister);override;

        procedure a_call_name(list:TAsmList;const s:string; weak: boolean);override;
        procedure a_call_reg(list:TAsmList;Reg:tregister);override;

        procedure a_jmp_name(list: TAsmList; const s: string);override;
        procedure a_jmp_flags(list: TAsmList; const f: TResFlags; l: tasmlabel);override;

        procedure g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);override;
        procedure g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean);override;

        { comparison operations }
        procedure a_cmp_const_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel); override;
        procedure a_cmp_reg_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel);override;
        procedure a_jmp_always(list: TAsmList; l: TAsmLabel);override;

        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: TCGSize; src, dst: TRegister);override;

        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister);override;

        procedure g_concatcopy(list : TAsmList; const source,dest : treference; len : tcgint);override;

        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);override;

        procedure a_loadfpu_intreg_reg(list: TAsmList; fromsize, tosize: tcgsize; intreg, fpureg: tregister);override;
        procedure a_loadfpu_reg_intreg(list: TAsmList; fromsize, tosize: tcgsize; fpureg, intreg: tregister);override;

        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);

        procedure g_overflowcheck(list: TAsmList; const Loc:tlocation; def:tdef);override;

        function create_data_entry(symbol: TAsmSymbol; offset: asizeint): TAsmLabel;
      end;

      tcg64fxtensa = class(tcg64f32)
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
        procedure a_op64_reg_reg_reg(list : TAsmList; op : TOpCG;size : tcgsize; regsrc1,regsrc2,regdst : tregister64);override;
        //procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
        //procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
        //procedure a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        //procedure a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        //procedure a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister);override;
        //procedure a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);override;
      end;

    procedure create_codegen;

    const
      TOpCG2AsmOp: array[topcg] of TAsmOp = (
        A_NONE,A_MOV,A_ADD,A_AND,A_NONE,A_NONE,A_MULL,A_MULL,A_NEG,A_NONE,A_OR,A_SRA,A_SLL,A_SRL,A_SUB,A_XOR,A_NONE,A_NONE
      );
{
      );TOpCG2AsmOpReg: array[topcg] of TAsmOp = (
        A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_MUL,A_MUL,A_NEG,A_MVN,A_ORR,A_ASRV,A_LSLV,A_LSRV,A_SUB,A_EOR,A_NONE,A_RORV
      );
      TOpCG2AsmOpImm: array[topcg] of TAsmOp = (
        A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_MUL,A_MUL,A_NEG,A_MVN,A_ORR,A_ASR,A_LSL,A_LSR,A_SUB,A_EOR,A_NONE,A_ROR
      );
      TOpCmp2AsmCond: array[topcmp] of TAsmCond = (C_NONE,C_EQ,C_GT,
        C_LT,C_GE,C_LE,C_NE,C_LS,C_CC,C_CS,C_HI
      );
 }

implementation

  uses
    globals,verbose,systems,cutils,
    paramgr,fmodule,
    symtable,symsym,
    tgobj,
    procinfo,cpupi;
                  
  const
    TOpCmp2AsmCond: array[TOpCmp] of TAsmCond = (
      C_None,
      C_EQ,
      C_None,
      C_LT,
      C_GE,
      C_None,
      C_NE,
      C_None,
      C_LTU,
      C_GEU,
      C_None
    );

    procedure tcgcpu.init_register_allocators;
      begin
        inherited init_register_allocators;
        if target_info.abi = abi_xtensa_call0 then
          rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_A2,RS_A3,RS_A4,RS_A5,RS_A6,RS_A7,{RS_A8,}RS_A9,
               RS_A10,RS_A11,RS_A12,RS_A13,RS_A14{,RS_A15}],first_int_imreg,[])
        else
          rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_A2,RS_A3,RS_A4,RS_A5,RS_A6,RS_A7,RS_A8,RS_A9,
               RS_A10,RS_A11,RS_A12,RS_A13,RS_A14,RS_A15],first_int_imreg,[]);

        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBNONE,
            [RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7,RS_F8,RS_F9,
             RS_F10,RS_F11,RS_F12,RS_F13,RS_F14,RS_F15],first_fpu_imreg,[]);
        rg[R_SPECIALREGISTER]:=trgcpu.create(R_SPECIALREGISTER,R_SUBNONE,
            [RS_B0,RS_B1,RS_B2,RS_B3,RS_B4,RS_B5,RS_B6,RS_B7,RS_B8,RS_B9,
             RS_B10,RS_B11,RS_B12,RS_B13,RS_B14,RS_B15],first_flag_imreg,[]);
      end;


    procedure tcgcpu.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_SPECIALREGISTER].free;
        inherited done_register_allocators;
      end;


    procedure tcgcpu.a_load_reg_reg(list : TAsmList; fromsize,tosize : tcgsize;
      reg1,reg2 : tregister);
      var
        conv_done : Boolean;
        instr : taicpu;
      begin
         if (tcgsize2size[fromsize]>32) or (tcgsize2size[tosize]>32) or (fromsize=OS_NO) or (tosize=OS_NO) then
           internalerror(2020030710);

         conv_done:=false;
         if tosize<>fromsize then
           begin
             conv_done:=true;
             if tcgsize2size[tosize]<=tcgsize2size[fromsize] then
              fromsize:=tosize;
             case fromsize of
                OS_8:
                  list.concat(taicpu.op_reg_reg_const_const(A_EXTUI,reg2,reg1,0,8));
                OS_S8:
                  begin
                    if CPUXTENSA_HAS_SEXT in cpu_capabilities[current_settings.cputype] then
                      list.concat(taicpu.op_reg_reg_const(A_SEXT,reg2,reg1,7))
                    else
                      begin
                        list.concat(taicpu.op_reg_reg_const(A_SLLI,reg2,reg1,24));
                        list.concat(taicpu.op_reg_reg_const(A_SRAI,reg2,reg2,24));
                      end;
                    if tosize=OS_16 then
                      list.concat(taicpu.op_reg_reg_const_const(A_EXTUI,reg2,reg2,0,16));
                  end;
                OS_16:
                  list.concat(taicpu.op_reg_reg_const_const(A_EXTUI,reg2,reg1,0,16));
                OS_S16:
                  if CPUXTENSA_HAS_SEXT in cpu_capabilities[current_settings.cputype] then
                    list.concat(taicpu.op_reg_reg_const(A_SEXT,reg2,reg1,15))
                  else
                    begin
                      list.concat(taicpu.op_reg_reg_const(A_SLLI,reg2,reg1,16));
                      list.concat(taicpu.op_reg_reg_const(A_SRAI,reg2,reg2,16));
                    end;
                else
                  conv_done:=false;
              end;
           end;
         if not conv_done and (reg1<>reg2) then
           begin
             { same size, only a register mov required }
             instr:=taicpu.op_reg_reg(A_MOV,reg2,reg1);
             list.Concat(instr);
             { Notify the register allocator that we have written a move instruction so
               it can try to eliminate it. }
             add_move_instruction(instr);
           end;
       end;


    procedure tcgcpu.a_load_reg_ref(list : TAsmList; fromsize,tosize : tcgsize;
      reg : tregister; const ref : TReference);
      var
        op: TAsmOp;
        href : treference;
      begin
        if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
          FromSize := ToSize;
        case tosize of
          { signed integer registers }
          OS_8,
          OS_S8:
            op:=A_S8I;
          OS_16,
          OS_S16:
            op:=A_S16I;
          OS_32,
          OS_S32:
            op:=A_S32I;
          else
            InternalError(2020030804);
        end;

        href:=ref;
        if assigned(href.symbol) or
          (href.index<>NR_NO) or
          ((op=A_S8I) and ((href.offset<0) or (href.offset>255))) or
          ((op=A_S16I) and ((href.offset<0) or (href.offset>510) or (href.offset mod 2<>0))) or
          ((op=A_S32I) and ((href.offset<0) or (href.offset>1020) or (href.offset mod 4<>0))) then
          fixref(list,href);

        list.concat(taicpu.op_reg_ref(op,reg,href));
      end;


    procedure tcgcpu.a_load_ref_reg(list : TAsmList; fromsize,tosize : tcgsize;
      const ref : TReference; reg : tregister);
      var
        href: treference;
        op: TAsmOp;
        tmpreg: TRegister;
      begin
        case fromsize of
          OS_8: op:=A_L8UI;
          OS_16: op:=A_L16UI;
          OS_S8: op:=A_L8UI;
          OS_S16: op:=A_L16SI;

          OS_64,OS_S64, { This only happens if tosize is smaller than fromsize }
          { We can therefore only consider the low 32-bit of the 64bit value }
          OS_32,
          OS_S32: op:=A_L32I;
        else
          internalerror(2020030805);
        end;

        href:=ref;
        if assigned(href.symbol) or
          (href.index<>NR_NO) or
          ((op=A_L8UI) and ((href.offset<0) or (href.offset>255))) or
          ((op in [A_L16SI,A_L16UI]) and ((href.offset<0) or (href.offset>510) or (href.offset mod 2<>0))) or
          ((op=A_L32I) and ((href.offset<0) or (href.offset>1020) or (href.offset mod 4<>0))) or
          ((href.base=NR_NO) and (href.index=NR_NO)) then
          fixref(list,href);

        list.concat(taicpu.op_reg_ref(op,reg,href));

        if (fromsize=OS_S8) and not(tosize in [OS_S8,OS_8]) then
          if CPUXTENSA_HAS_SEXT in cpu_capabilities[current_settings.cputype] then
            list.concat(taicpu.op_reg_reg_const(A_SEXT,reg,reg,7))
          else
            begin
              list.concat(taicpu.op_reg_reg_const(A_SLLI,reg,reg,24));
              list.concat(taicpu.op_reg_reg_const(A_SRAI,reg,reg,24));
            end;
        if (fromsize<>tosize) and (not (tosize in [OS_SINT,OS_INT])) then
          a_load_reg_reg(list,fromsize,tosize,reg,reg);
      end;


    procedure tcgcpu.a_load_const_reg(list : TAsmList; size : tcgsize;
      a : tcgint; reg : tregister);
      var
        hr : treference;
        l : TAsmLabel;
      begin
        if (a>=-2048) and (a<=2047) then
          list.Concat(taicpu.op_reg_const(A_MOVI,reg,a))
        else
          begin
            reference_reset(hr,4,[]);

            hr.symbol:=create_data_entry(nil,longint(a));
            list.concat(taicpu.op_reg_ref(A_L32R,reg,hr));
          end;
      end;


    procedure tcgcpu.fixref(list : TAsmList;var ref : treference);
      var
        tmpreg, tmpreg2 : tregister;
        tmpref : treference;
        l : tasmlabel;
      begin
        { create consts entry }
        if assigned(ref.symbol) or (ref.offset<-2048) or (ref.offset>2047) or
          ((ref.base=NR_NO) and (ref.index=NR_NO)) then
          begin
            reference_reset(tmpref,4,[]);
            tmpreg:=NR_NO;

            { load consts entry }
            tmpreg:=getintregister(list,OS_INT);
            if ref.symbol=nil then
              a_load_const_reg(list,OS_ADDR,ref.offset,tmpreg)
            else
              begin
                tmpref.symbol:=create_data_entry(ref.symbol,ref.offset);
                list.concat(taicpu.op_reg_ref(A_L32R,tmpreg,tmpref));
              end;

            if ref.base<>NR_NO then
              begin
                if ref.index<>NR_NO then
                  begin
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                    ref.base:=tmpreg;
                  end
                else
                  ref.index:=tmpreg;
              end
            else
              ref.base:=tmpreg;
          end
        else if ref.offset<>0 then
          begin
            tmpreg:=getintregister(list,OS_INT);
            if (ref.offset>=-128) and (ref.offset<=127) then
              begin
                list.concat(taicpu.op_reg_reg_const(A_ADDI,tmpreg,ref.base,ref.offset));
                ref.base:=tmpreg;
              end
            else
              begin
                list.concat(taicpu.op_reg_const(A_MOVI,tmpreg,ref.offset));
                if ref.base<>NR_NO then
                  begin
                    if ref.index<>NR_NO then
                      begin
                        list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                        ref.base:=tmpreg;
                      end
                    else
                      ref.index:=tmpreg;
                  end
                else
                  ref.base:=tmpreg;
              end;
          end;

        if ref.index<>NR_NO then
          begin
            if ref.base<>NR_NO then
              begin
                tmpreg:=getintregister(list,OS_INT);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,ref.index));
                ref.base:=tmpreg;
              end
            else
              ref.base:=ref.index;
            ref.index:=NR_NO;
          end;
        ref.offset:=0;
        ref.symbol:=nil;
      end;


    procedure tcgcpu.a_loadaddr_ref_reg(list : TAsmList;
      const ref : TReference; r : tregister);
       var
        b : byte;
        tmpref : treference;
        instr : taicpu;
      begin
        tmpref:=ref;
        { Be sure to have a base register }
        if tmpref.base=NR_NO then
          begin
            tmpref.base:=tmpref.index;
            tmpref.index:=NR_NO;
          end;

        if assigned(tmpref.symbol) then
          fixref(list,tmpref);

        { expect a base here if there is an index }
        if (tmpref.base=NR_NO) and (tmpref.index<>NR_NO) then
          internalerror(200312022);

        if tmpref.index<>NR_NO then
          begin
            a_op_reg_reg_reg(list,OP_ADD,OS_ADDR,tmpref.base,tmpref.index,r);
            if tmpref.offset<>0 then
              a_op_const_reg_reg(list,OP_ADD,OS_ADDR,tmpref.offset,r,r);
          end
        else
          begin
            if tmpref.base=NR_NO then
              a_load_const_reg(list,OS_ADDR,tmpref.offset,r)
            else
              if tmpref.offset<>0 then
                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,tmpref.offset,tmpref.base,r)
              else
                begin
                  instr:=taicpu.op_reg_reg(A_MOV,r,tmpref.base);
                  list.concat(instr);
                  add_move_instruction(instr);
                end;
          end;
      end;


    procedure tcgcpu.a_op_reg_reg(list : TAsmList; op : topcg; size : tcgsize; src,dst : tregister);
      var
        tmpreg : TRegister;
      begin
        if op = OP_NEG then
          begin
            list.concat(taicpu.op_reg_reg(A_NEG,dst,src));
            maybeadjustresult(list,OP_NEG,size,dst);
          end
        else if op = OP_NOT then
          begin
            tmpreg:=getintregister(list,size);
            list.concat(taicpu.op_reg_const(A_MOVI,tmpreg,-1));
            list.concat(taicpu.op_reg_reg_reg(A_XOR,dst,tmpreg,src));
            maybeadjustresult(list,OP_NOT,size,dst);
          end
        else
          a_op_reg_reg_reg(list,op,size,src,dst,dst);
      end;


    procedure tcgcpu.a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister);
      var
        l1 : longint;
        tmpreg : TRegister;
      begin
        optimize_op_const(size, op, a);
        case op of
          OP_NONE:
            begin
              if src <> dst then
                a_load_reg_reg(list, size, size, src, dst);
              exit;
            end;
          OP_MOVE:
            begin
              a_load_const_reg(list, size, a, dst);
              exit;
            end;
          else
            ;
        end;
        { there could be added some more sophisticated optimizations }
        if (op in [OP_IMUL,OP_IDIV]) and (a=-1) then
          a_op_reg_reg(list,OP_NEG,size,src,dst)
        { we do this here instead in the peephole optimizer because
          it saves us a register }
        else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a,l1) then
          a_op_const_reg_reg(list,OP_SHL,size,l1,src,dst)
        else if (op=OP_ADD) and (a>=-128) and (a<=127) then
          list.concat(taicpu.op_reg_reg_const(A_ADDI,dst,src,a))
        else if (op=OP_ADD) and (a>=-128-32768) and (a<=127+32512) then
          begin
{$ifdef EXTDEBUG}
            list.concat(tai_comment.Create(strpnew('Value: '+tostr(a))));
{$endif EXTDEBUG}
            list.concat(taicpu.op_reg_reg_const(A_ADDMI,dst,src,Smallint((a+128) and $ff00)));
            list.concat(taicpu.op_reg_reg_const(A_ADDI,dst,dst,Shortint(a and $ff)));
          end
        else if (op=OP_SUB) and (a>=-127) and (a<=128) then
          list.concat(taicpu.op_reg_reg_const(A_ADDI,dst,src,-a))
        else if (op=OP_SUB) and (a>=-127-32512) and (a<=128+32768) then
          begin
{$ifdef EXTDEBUG}
            list.concat(tai_comment.Create(strpnew('Value: '+tostr(a))));
{$endif EXTDEBUG}
            a:=-a;
            list.concat(taicpu.op_reg_reg_const(A_ADDMI,dst,src,Smallint((a+128) and $ff00)));
            list.concat(taicpu.op_reg_reg_const(A_ADDI,dst,dst,Shortint(a and $ff)));
          end
        else if (op=OP_SHL) and (a>=1) and (a<=31) then
          list.concat(taicpu.op_reg_reg_const(A_SLLI,dst,src,a))
        else if (op=OP_SAR) and (a>=0) and (a<=31) then
          list.concat(taicpu.op_reg_reg_const(A_SRAI,dst,src,a))
        else if (op=OP_SHR) and (a>=0) and (a<=15) then
          list.concat(taicpu.op_reg_reg_const(A_SRLI,dst,src,a))
        else if (op=OP_SHR) and (a>15) and (a<=31) then
          list.concat(taicpu.op_reg_reg_const_const(A_EXTUI,dst,src,a,32-a))
        else if (op=OP_AND) and (63-BsrQWord(qword(a))+PopCnt(QWord(a))=64) and (PopCnt(QWord(a))<=16) then
          list.concat(taicpu.op_reg_reg_const_const(A_EXTUI,dst,src,0,PopCnt(QWord(a))))
        else
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            a_op_reg_reg_reg(list,op,size,tmpreg,src,dst);
          end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgcpu.a_op_const_reg(list : TAsmList; op : topcg; size : tcgsize; a : tcgint; reg : tregister);
      begin
        a_op_const_reg_reg(list,op,size,a,reg,reg);
      end;


    procedure tcgcpu.a_op_reg_reg_reg(list : TAsmList; op : topcg;
      size : tcgsize; src1,src2,dst : tregister);
      var
        tmpreg : TRegister;
      begin
        if op=OP_NOT then
          begin
            tmpreg:=getintregister(list,size);
            list.concat(taicpu.op_reg_const(A_MOVI,tmpreg,-1));
            maybeadjustresult(list,op,size,dst);
          end
        else if op=OP_NEG then
          begin
            list.concat(taicpu.op_reg_reg(A_NEG,dst,src1));
            maybeadjustresult(list,op,size,dst);
          end
        else if op in [OP_SAR,OP_SHL,OP_SHR] then
          begin
            if op=OP_SHL then
              list.concat(taicpu.op_reg(A_SSL,src1))
            else
              list.concat(taicpu.op_reg(A_SSR,src1));
            list.concat(taicpu.op_reg_reg(TOpCG2AsmOp[op],dst,src2));
            maybeadjustresult(list,op,size,dst);
          end
        else
          case op of
            OP_MOVE:
              a_load_reg_reg(list,size,size,src1,dst);
            else
              begin
                list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[op],dst,src2,src1));
                maybeadjustresult(list,op,size,dst);
              end;
          end;
       end;


    procedure tcgcpu.a_call_name(list : TAsmList; const s : string;
      weak : boolean);
      begin
        if not weak then
          list.concat(taicpu.op_sym(txtensaprocinfo(current_procinfo).callins,current_asmdata.RefAsmSymbol(s,AT_FUNCTION)))
        else
          list.concat(taicpu.op_sym(txtensaprocinfo(current_procinfo).callins,current_asmdata.WeakRefAsmSymbol(s,AT_FUNCTION)));
      end;


    procedure tcgcpu.a_call_reg(list : TAsmList; Reg : tregister);
      begin
        list.concat(taicpu.op_reg(txtensaprocinfo(current_procinfo).callxins,reg));
      end;


    procedure tcgcpu.a_jmp_name(list : TAsmList; const s : string);
      var
        ai : taicpu;
        tmpreg: TRegister;
      begin
        { for now, we use A15 here, however, this is not save as it might contain an argument }
        ai:=TAiCpu.op_sym_reg(A_J,current_asmdata.RefAsmSymbol(s,AT_FUNCTION),NR_A15);
        ai.oppostfix := PF_L; // if destination is too far for J then assembler can convert to JX
        ai.is_jmp:=true;
        list.Concat(ai);
      end;


    procedure tcgcpu.a_jmp_flags(list: TAsmList; const f: TResFlags; l: tasmlabel);
      var
        instr: taicpu;
      begin
        if CPUXTENSA_HAS_BOOLEAN_OPTION in cpu_capabilities[current_settings.cputype] then
          begin
            instr:=taicpu.op_reg_sym(A_B,f.register,l);
            instr.condition:=flags_to_cond(f.flag);
            instr.is_jmp:=true;
            list.concat(instr);
          end
        else
          Internalerror(2020070401);
      end;


    procedure tcgcpu.g_proc_entry(list : TAsmList; localsize : longint;
      nostackframe : boolean);
      var
         ref : treference;
         r : byte;
         regs : tcpuregisterset;
         stackmisalignment : pint;
         regoffset : LongInt;
         stack_parameters : Boolean;
         registerarea : PtrInt;
         l : TAsmLabel;
      begin
        LocalSize:=align(LocalSize,4);
        stack_parameters:=current_procinfo.procdef.stack_tainting_parameter(calleeside);

        { call instruction does not put anything on the stack }
        registerarea:=0;
        if not(nostackframe) then
          begin
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);
            a_reg_alloc(list,NR_STACK_POINTER_REG);
            case target_info.abi of
              abi_xtensa_call0:
                begin
                  if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                    Include(regs,RS_A15);
                  if pi_do_call in current_procinfo.flags then
                    Include(regs,RS_A0);
                  if regs<>[] then
                     begin
                       for r:=RS_A0 to RS_A15 do
                         if r in regs then
                           inc(registerarea,4);
                     end;

                  if stack_parameters and (pi_estimatestacksize in current_procinfo.flags) then
                    begin
                      list.concat(tai_comment.Create(strpnew('Stackframe size was estimated before code generation due to stack parameters')));
                      list.concat(tai_comment.Create(strpnew('  Calculated stackframe size: '+tostr(txtensaprocinfo(current_procinfo).stackframesize))));
                      list.concat(tai_comment.Create(strpnew('  Max. outgoing parameter size: '+tostr(txtensaprocinfo(current_procinfo).maxpushedparasize))));
                      list.concat(tai_comment.Create(strpnew('  End of last temporary location: '+tostr(tg.lasttemp))));
                      list.concat(tai_comment.Create(strpnew('  Size of register area: '+tostr(registerarea))));
                      list.concat(tai_comment.Create(strpnew('  Required size after code generation: '+tostr(localsize))));

                      if localsize>txtensaprocinfo(current_procinfo).stackframesize then
                        internalerror(2020091001);
                      localsize:=txtensaprocinfo(current_procinfo).stackframesize;
                    end
                  else
                    begin
                      inc(localsize,registerarea);
                      localsize:=align(localsize,current_settings.alignment.localalignmax);
                    end;

                  if LocalSize<>0 then
                    begin
                      a_reg_alloc(list,NR_STACK_POINTER_REG);
                      list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,-localsize));
                    end;

                  reference_reset(ref,4,[]);
                  ref.base:=NR_STACK_POINTER_REG;
                  ref.offset:=localsize;
                  if ref.offset>1024 then
                    begin
                      if ref.offset<=1024+32512 then
                        begin
                          list.concat(taicpu.op_reg_reg_const(A_ADDMI,NR_A8,NR_STACK_POINTER_REG,ref.offset and $fffffc00));
                          ref.offset:=ref.offset and $3ff;
                          ref.base:=NR_A8;
                        end
                      else
                        { fix me! }
                        Internalerror(2020031101);
                    end;

                  if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                    begin
                      dec(ref.offset,4);
                      list.concat(taicpu.op_reg_ref(A_S32I,NR_A15,ref));
                      a_reg_alloc(list,NR_FRAME_POINTER_REG);
                      list.concat(taicpu.op_reg_reg(A_MOV,NR_A15,NR_STACK_POINTER_REG));
                    end;

                  if regs<>[] then
                    begin
                      for r:=RS_A14 downto RS_A0 do
                        if r in regs then
                          begin
                            dec(ref.offset,4);
                            list.concat(taicpu.op_reg_ref(A_S32I,newreg(R_INTREGISTER,r,R_SUBWHOLE),ref));
                          end;
                    end;
                end;
              abi_xtensa_windowed:
                begin
                  if stack_parameters and (pi_estimatestacksize in current_procinfo.flags) then
                    begin
                      list.concat(tai_comment.Create(strpnew('Stackframe size was estimated before code generation due to stack parameters')));
                      list.concat(tai_comment.Create(strpnew('  Calculated stackframe size: '+tostr(txtensaprocinfo(current_procinfo).stackframesize))));
                      list.concat(tai_comment.Create(strpnew('  Max. outgoing parameter size: '+tostr(txtensaprocinfo(current_procinfo).maxpushedparasize))));
                      list.concat(tai_comment.Create(strpnew('  End of last temporary location: '+tostr(tg.lasttemp))));
                      list.concat(tai_comment.Create(strpnew('  Max. window rotation in bytes: '+tostr(txtensaprocinfo(current_procinfo).maxcall*4))));
                      list.concat(tai_comment.Create(strpnew('  Required size after code generation: '+tostr(localsize))));

                      { should never happen as localsize is derived from
                        txtensaprocinfo(current_procinfo).stackframesize }
                      if localsize>txtensaprocinfo(current_procinfo).stackframesize then
                        internalerror(2020031402);
                      localsize:=txtensaprocinfo(current_procinfo).stackframesize;
                    end
                  else
                    begin
                      localsize:=align(localsize,current_settings.alignment.localalignmax);
                      inc(localsize,4*4);
                      if pi_do_call in current_procinfo.flags then
                        inc(localsize,txtensaprocinfo(current_procinfo).maxcall*4);
                    end;

                  if localsize<0 then
                    Internalerror(2020083001);
                  if localsize>32760 then
                    begin
                      list.concat(taicpu.op_reg_const(A_ENTRY,NR_STACK_POINTER_REG,32));

                      reference_reset(ref,4,[]);
                      ref.symbol:=create_data_entry(nil,longint(localsize-32));
                      list.concat(taicpu.op_reg_ref(A_L32R,NR_A8,ref));

                      list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_A8,NR_STACK_POINTER_REG,NR_A8));
                      list.concat(taicpu.op_reg_reg(A_MOVSP,NR_STACK_POINTER_REG,NR_A8));
                    end
                  else
                    list.concat(taicpu.op_reg_const(A_ENTRY,NR_STACK_POINTER_REG,localsize));
                end;
              else
                Internalerror(2020031401);
            end;
          end;
      end;


    procedure tcgcpu.g_proc_exit(list : TAsmList; parasize : longint;
     nostackframe : boolean);
      var
         ref : treference;
         r : byte;
         regs : tcpuregisterset;
         stackmisalignment : pint;
         regoffset : LongInt;
         stack_parameters : Boolean;
         registerarea : PtrInt;
         l : TAsmLabel;
         LocalSize: longint;
      begin
        case target_info.abi of
          abi_xtensa_windowed:
            list.Concat(taicpu.op_none(A_RETW));
          abi_xtensa_call0:
            begin
              if not(nostackframe) then
                begin
                  LocalSize:=current_procinfo.calc_stackframe_size;
                  LocalSize:=align(LocalSize,4);
                  stack_parameters:=current_procinfo.procdef.stack_tainting_parameter(calleeside);
                  registerarea:=0;
                  regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);
                  if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                    Include(regs,RS_A15);
                  if pi_do_call in current_procinfo.flags then
                    Include(regs,RS_A0);
                  if regs<>[] then
                     begin
                       for r:=RS_A0 to RS_A15 do
                         if r in regs then
                           inc(registerarea,4);
                     end;

                  { do we use then estimated stack size? }
                  if not(stack_parameters and (pi_estimatestacksize in current_procinfo.flags)) then
                    begin
                      inc(localsize,registerarea);
                      localsize:=align(localsize,current_settings.alignment.localalignmax);
                    end;

                  if LocalSize<>0 then
                    begin
                      // Determine reference mode required to access stack
                      reference_reset(ref,4,[]);
                      ref.base:=NR_STACK_POINTER_REG;
                      ref.offset:=localsize;
                      if ref.offset>1024 then
                        begin
                          if ref.offset<=1024+32512 then
                            begin
                              list.concat(taicpu.op_reg_reg_const(A_ADDMI,NR_A8,NR_STACK_POINTER_REG,ref.offset and $fffffc00));
                              ref.offset:=ref.offset and $3ff;
                              ref.base:=NR_A8;
                            end
                          else
                            { fix me! }
                            Internalerror(2020031102);
                        end;

                      // restore a15 if used
                      if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                        begin
                          dec(ref.offset,4);
                          list.concat(taicpu.op_reg_ref(A_L32I,NR_A15,ref));
                          a_reg_dealloc(list,NR_FRAME_POINTER_REG);
                        end;

                      // restore rest of registers
                      if regs<>[] then
                        begin
                          for r:=RS_A14 downto RS_A0 do
                            if r in regs then
                              begin
                                dec(ref.offset,4);
                                list.concat(taicpu.op_reg_ref(A_L32I,newreg(R_INTREGISTER,r,R_SUBWHOLE),ref));
                              end;
                        end;

                      // restore stack pointer
                      list.concat(taicpu.op_reg_reg_const(A_ADDI,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,localsize));
                      a_reg_dealloc(list,NR_STACK_POINTER_REG);
                    end;
                  end;
              list.Concat(taicpu.op_none(A_RET));
            end
          else
            Internalerror(2020031403);
        end;
      end;


    procedure tcgcpu.a_cmp_const_reg_label(list: TAsmList; size: tcgsize; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel);

      function is_b4const(v: tcgint): boolean;
        begin
          case v of
            -1,1,2,3,4,5,6,7,8,
            10,12,16,32,64,128,256:
              result:=true;
          else
            result:=false;
          end;
        end;

      function is_b4constu(v: tcgint): boolean;
        begin
          case v of
            32768,65536,
            2,3,4,5,6,7,8,
            10,12,16,32,64,128,256:
              result:=true;
          else
            result:=false;
          end;
        end;

      var
        op: TAsmCond;
        instr: taicpu;
      begin
        if (a=0) and (cmp_op in [OC_EQ,OC_NE,OC_LT,OC_GTE]) then
          begin
            case cmp_op of
              OC_EQ: op:=C_EQZ;
              OC_NE: op:=C_NEZ;
              OC_LT: op:=C_LTZ;
              OC_GTE: op:=C_GEZ;
            else
              Internalerror(2020030806);
            end;     
            instr:=taicpu.op_reg_sym(A_B,reg,l);
            instr.condition:=op;
            instr.is_jmp:=true;
            list.concat(instr);
          end
        else if is_b4const(a) and
                (cmp_op in [OC_EQ,OC_NE,OC_LT,OC_GTE]) then
          begin
            case cmp_op of
              OC_EQ: op:=C_EQI;
              OC_NE: op:=C_NEI;
              OC_LT: op:=C_LTI;
              OC_GTE: op:=C_GEI;
            else
              Internalerror(2020030807);
            end;

            instr:=taicpu.op_reg_const_sym(A_B,reg,a,l);
            instr.condition:=op;
            instr.is_jmp:=true;
            list.concat(instr);
          end
        else if is_b4constu(a) and
                (cmp_op in [OC_B,OC_AE]) then
          begin
            case cmp_op of
              OC_B: op:=C_LTUI;
              OC_AE: op:=C_GEUI;
            else
              Internalerror(2020030808);
            end;

            instr:=taicpu.op_reg_const_sym(A_B,reg,a,l);
            instr.condition:=op;
            instr.is_jmp:=true;
            list.concat(instr);
          end
        else
          inherited a_cmp_const_reg_label(list, size, cmp_op, a, reg, l);
      end;


    procedure tcgcpu.a_cmp_reg_reg_label(list : TAsmList; size : tcgsize;
        cmp_op : topcmp; reg1,reg2 : tregister; l : tasmlabel);
      var
        tmpreg: TRegister;
        instr: taicpu;
      begin
        if TOpCmp2AsmCond[cmp_op]=C_None then
          begin
            cmp_op:=swap_opcmp(cmp_op);
            tmpreg:=reg1;
            reg1:=reg2;
            reg2:=tmpreg;
          end;

        instr:=taicpu.op_reg_reg_sym(A_B,reg2,reg1,l);
        instr.condition:=TOpCmp2AsmCond[cmp_op];
        instr.is_jmp:=true;
        list.concat(instr);
      end;


    procedure tcgcpu.a_jmp_always(list : TAsmList; l : TAsmLabel);
      var
        ai : taicpu;
      begin
        if l.bind in [AB_GLOBAL] then
          begin
          { for now, we use A15 here, however, this is not save as it might contain an argument, I have not figured out a
            solution yet }
            ai:=taicpu.op_sym_reg(A_J,l,NR_A15);
            ai.oppostfix := PF_L;
          end
        else
          ai:=taicpu.op_sym(A_J,l);
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgcpu.g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister);
      var
        hregister: TRegister;
        instr: taicpu;
      begin
        a_load_const_reg(list,size,0,reg);
        hregister:=getintregister(list,size);
        a_load_const_reg(list,size,1,hregister);
        instr:=taicpu.op_reg_reg_reg(A_MOV,reg,hregister,f.register);
        instr.condition:=flags_to_cond(f.flag);
        list.concat(instr);
      end;


    procedure tcgcpu.g_concatcopy_move(list: tasmlist; const Source, dest: treference; len: tcgint);
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


    procedure tcgcpu.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        tmpreg1, hreg, countreg: TRegister;
        src, dst, src2, dst2: TReference;
        lab:      tasmlabel;
        Count, count2: aint;

        function reference_is_reusable(const ref: treference): boolean;
          begin
            result:=(ref.base<>NR_NO) and (ref.index=NR_NO) and
               (ref.symbol=nil);
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
            if CPUXTENSA_HAS_LOOPS in cpu_capabilities[current_settings.cputype] then
              begin
                list.concat(taicpu.op_reg_sym(A_LOOP, countreg, lab));
                list.concat(taicpu.op_reg_ref(A_L32I, tmpreg1, src));
                list.concat(taicpu.op_reg_ref(A_S32I, tmpreg1, dst));
                list.concat(taicpu.op_reg_reg_const(A_ADDI, src.base, src.base, 4));
                list.concat(taicpu.op_reg_reg_const(A_ADDI, dst.base, dst.base, 4));
                a_label(list, lab);
              end
            else
              begin
                a_label(list, lab);
                list.concat(taicpu.op_reg_ref(A_L32I, tmpreg1, src));
                list.concat(taicpu.op_reg_ref(A_S32I, tmpreg1, dst));
                list.concat(taicpu.op_reg_reg_const(A_ADDI, src.base, src.base, 4));
                list.concat(taicpu.op_reg_reg_const(A_ADDI, dst.base, dst.base, 4));
                list.concat(taicpu.op_reg_reg_const(A_ADDI, countreg, countreg, -1));
                a_cmp_const_reg_label(list,OS_INT,OC_NE,0,countreg,lab);
                { keep the registers alive }
                list.concat(taicpu.op_reg_reg(A_MOV,countreg,countreg));
              end;
            { keep the registers alive }
            list.concat(taicpu.op_reg_reg(A_MOV,src.base,src.base));
            list.concat(taicpu.op_reg_reg(A_MOV,dst.base,dst.base));
            len := len mod 4;
          end;
          { unrolled loop }
          Count := len div 4;
          if Count > 0 then
          begin
            tmpreg1 := GetIntRegister(list, OS_INT);
            for count2 := 1 to Count do
            begin
              list.concat(taicpu.op_reg_ref(A_L32I, tmpreg1, src));
              list.concat(taicpu.op_reg_ref(A_S32I, tmpreg1, dst));
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


     procedure tcgcpu.a_loadfpu_reg_reg(list: TAsmList; fromsize,tosize: tcgsize; reg1, reg2: tregister);
       var
         ai: taicpu;
       begin
         if not(fromsize in [OS_32,OS_F32]) then
           InternalError(2020032603);

         ai := taicpu.op_reg_reg(A_MOV,reg2,reg1);
         ai.oppostfix := PF_S;
         list.concat(ai);
       end;


     procedure tcgcpu.a_loadfpu_ref_reg(list: TAsmList; fromsize,tosize: tcgsize; const ref: treference; reg: tregister);
       var
         href: treference;
       begin
         if not(fromsize in [OS_32,OS_F32]) then
           InternalError(2020032602);
         href:=ref;
         if assigned(href.symbol) or
           ((href.index<>NR_NO) and (href.offset<>0)) or
           (((href.offset<0) or (href.offset>1020) or (href.offset mod 4<>0))) then
           fixref(list,href);

         if (href.base<>NR_NO) and (href.index<>NR_NO) then
           list.concat(taicpu.op_reg_ref(A_LSX,reg,href))
         else
           list.concat(taicpu.op_reg_ref(A_LSI,reg,href));

         if fromsize<>tosize then
           a_loadfpu_reg_reg(list,fromsize,tosize,reg,reg);
       end;


     procedure tcgcpu.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);
       var
         href: treference;
       begin
         if not(fromsize in [OS_32,OS_F32]) then
           InternalError(2020032604);
         href:=ref;
         if assigned(href.symbol) or
           ((href.index<>NR_NO) and (href.offset<>0)) or
           (((href.offset<0) or (href.offset>1020) or (href.offset mod 4<>0))) then
           fixref(list,href);

         if (href.base<>NR_NO) and (href.index<>NR_NO) then
           list.concat(taicpu.op_reg_ref(A_SSX,reg,href))
         else
           list.concat(taicpu.op_reg_ref(A_SSI,reg,href));
       end;


    procedure tcgcpu.a_loadfpu_intreg_reg(list : TAsmList; fromsize,tosize : tcgsize; intreg,fpureg : tregister);
      begin
        if not(tcgsize2size[fromsize]=4) or
           not(tcgsize2size[tosize]=4) then
          internalerror(2020091102);
        list.concat(taicpu.op_reg_reg(A_WFR,fpureg,intreg));
      end;


    procedure tcgcpu.a_loadfpu_reg_intreg(list : TAsmList; fromsize,tosize : tcgsize; fpureg,intreg : tregister);
      begin
        if not(tcgsize2size[fromsize]=4) or
           not(tcgsize2size[tosize]=4) then
          internalerror(2020091202);
        list.concat(taicpu.op_reg_reg(A_RFR,intreg,fpureg));
      end;


    procedure tcgcpu.maybeadjustresult(list : TAsmList; op : TOpCg; size : tcgsize; dst : tregister);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16]) then
          a_load_reg_reg(list,OS_32,size,dst,dst);
      end;


    procedure tcgcpu.g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef);
      begin
        { no overflow checking yet }
      end;


    function tcgcpu.create_data_entry(symbol: TAsmSymbol;offset: asizeint): TAsmLabel;
      var
        hp: tai;
      begin
        hp:=tai(current_procinfo.aktlocaldata.first);
        while assigned(hp) do
          begin
            if (hp.typ=ait_label) and assigned(hp.Next) and
              (tai(hp.Next).typ=ait_const) and
              (tai_const(hp.Next).consttype=aitconst_ptr) and
              (tai_const(hp.Next).sym=symbol) and
              (tai_const(hp.Next).endsym=nil) and
              ((assigned(symbol) and (tai_const(hp.Next).symofs=offset)) or
               (not(assigned(symbol)) and (tai_const(hp.Next).value=offset))
              ) then
              begin
                Result:=tai_label(hp).labsym;
                exit;
              end;
            hp:=tai(hp.Next);
          end;

        current_asmdata.getjumplabel(Result);
        cg.a_label(current_procinfo.aktlocaldata,Result);

        if assigned(symbol) then
          current_procinfo.aktlocaldata.concat(tai_const.create_sym_offset(symbol,offset))
        else
          current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(offset));
      end;


    procedure tcgcpu.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: TCGSize; src, dst: TRegister);
      var
        ai: taicpu;
        tmpreg: TRegister;
      begin
        if reverse then
          begin
            list.Concat(taicpu.op_reg_reg(A_NSAU,dst,src));
            tmpreg:=getintregister(list,OS_INT);
            a_load_const_reg(list,OS_INT,31,tmpreg);
            a_op_reg_reg_reg(list,OP_SUB,OS_INT,dst,tmpreg,dst);
            tmpreg:=getintregister(list,OS_INT);
            a_load_const_reg(list,OS_INT,255,tmpreg);
            ai:=taicpu.op_reg_reg_reg(A_MOV,dst,tmpreg,src);
            ai.condition:=C_EQZ;
            list.Concat(ai);
          end
        else
          Internalerror(2020092604);
      end;


    procedure tcg64fxtensa.a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);
      var
        instr: taicpu;
        no_carry: TAsmLabel;
        tmpreg: TRegister;
      begin
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(2020030810);
          else
            ;
        end;
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reglo,regsrc2.reglo,regdst.reglo);
              cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reghi,regsrc2.reghi,regdst.reghi);
            end;
          OP_ADD:
            begin
              if (regsrc1.reglo=regdst.reglo) or (regsrc1.reghi=regdst.reghi) then
                Internalerror(2020082205);
              list.concat(taicpu.op_reg_reg_reg(A_ADD, regdst.reglo, regsrc2.reglo, regsrc1.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_ADD, regdst.reghi, regsrc2.reghi, regsrc1.reghi));
              current_asmdata.getjumplabel(no_carry);
              cg.a_cmp_reg_reg_label(list,OS_INT,OC_AE, regsrc1.reglo, regdst.reglo, no_carry);
              list.concat(taicpu.op_reg_reg_const(A_ADDI, regdst.reghi, regdst.reghi, 1));
              cg.a_label(list,no_carry);
            end;
          OP_SUB:
            begin
              if (regsrc1.reglo=regdst.reglo) or (regsrc1.reghi=regdst.reghi) then
                Internalerror(2020082206);
              { we need the original src2 value for the comparison, do not overwrite it }
              if regsrc2.reglo=regdst.reglo then
                begin
                  tmpreg:=cg.GetIntRegister(list,OS_S32);
                  cg.a_load_reg_reg(list,OS_INT,OS_INT,regsrc2.reglo,tmpreg);
                  regsrc2.reglo:=tmpreg;
                end;
              list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reglo, regsrc2.reglo, regsrc1.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SUB, regdst.reghi, regsrc2.reghi, regsrc1.reghi));
              current_asmdata.getjumplabel(no_carry);
              cg.a_cmp_reg_reg_label(list,OS_INT,OC_AE, regsrc1.reglo, regsrc2.reglo, no_carry);
              list.concat(taicpu.op_reg_reg_const(A_ADDI, regdst.reghi, regdst.reghi, -1));
              cg.a_label(list,no_carry);
            end;
          else
            internalerror(2020030813);
        end;
      end;


    procedure tcg64fxtensa.a_op64_reg_reg(list : TAsmList; op : TOpCG; size : tcgsize; regsrc,regdst : tregister64);
      var
        tmpreg : TRegister;
        instr : taicpu;
      begin
        case op of
          OP_NEG:
            begin
              tmpreg:=cg.GetIntRegister(list, OS_INT);
              list.concat(taicpu.op_reg_reg(A_NEG,regdst.reglo,regsrc.reglo));
              list.concat(taicpu.op_reg_reg(A_NEG,regdst.reghi,regsrc.reghi));
              list.concat(taicpu.op_reg_reg_const(A_ADDI,tmpreg,regdst.reghi,-1));
              instr:=taicpu.op_reg_reg_reg(A_MOV,regdst.reghi,tmpreg,regdst.reglo);
              instr.condition:=C_NEZ;
              list.concat(instr);
            end;
          OP_NOT:
            begin
              cg.a_op_reg_reg(list,OP_NOT,OS_INT,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,OP_NOT,OS_INT,regsrc.reghi,regdst.reghi);
            end;
          else
            a_op64_reg_reg_reg(list,op,size,regsrc,regdst,regdst);
        end;
      end;


    procedure tcg64fxtensa.a_op64_const_reg_reg(list : TAsmList; op : TOpCG; size : tcgsize; value : int64; regsrc,regdst : tregister64);
      var
        tmpreg64 : tregister64;
        no_carry : TAsmLabel;
        tmpreg: tregister;
      begin
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(2020030904);
          else
            ;
        end;
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg_reg(list,op,OS_32,aint(lo(value)),regsrc.reglo,regdst.reglo);
              cg.a_op_const_reg_reg(list,op,OS_32,aint(hi(value)),regsrc.reghi,regdst.reghi);
            end;
          OP_ADD:
            begin
              { could do better here (hi(value) in 248..2047), for now we support only the simple cases }
              if (value>=-2048) and (value<=2047) then
                begin
                  { we need the original src value for the comparison, do not overwrite it }
                  if regsrc.reglo=regdst.reglo then
                    begin
                      tmpreg:=cg.GetIntRegister(list,OS_S32);
                      cg.a_load_reg_reg(list,OS_INT,OS_INT,regsrc.reglo,tmpreg);
                      regsrc.reglo:=tmpreg;
                    end;

                  list.concat(taicpu.op_reg_reg_const(A_ADDI, regdst.reglo, regsrc.reglo, value));
                  list.concat(taicpu.op_reg_reg(A_MOV, regdst.reghi, regsrc.reghi));
                  current_asmdata.getjumplabel(no_carry);
                  cg.a_cmp_reg_reg_label(list,OS_INT,OC_AE, regsrc.reglo, regdst.reglo, no_carry);
                  list.concat(taicpu.op_reg_reg_const(A_ADDI, regdst.reghi, regdst.reghi, 1));
                  cg.a_label(list,no_carry);
                  end
                else
                  begin
                    tmpreg64.reglo := cg.GetIntRegister(list,OS_S32);
                    tmpreg64.reghi := cg.GetIntRegister(list,OS_S32);
                    a_load64_const_reg(list,value,tmpreg64);
                    a_op64_reg_reg_reg(list,op,size,tmpreg64,regsrc,regdst);
                  end;
            end;
          OP_SHL:
            begin
              if (value>0) and (value<=16) then
                begin
                  tmpreg:=cg.GetIntRegister(list,OS_32);
                  list.concat(taicpu.op_reg_reg_const_const(A_EXTUI, tmpreg, regsrc.reglo, 32-value, value));
                  list.concat(taicpu.op_reg_reg_const(A_SLLI, regdst.reglo, regsrc.reglo, value));
                  list.concat(taicpu.op_reg_reg_const(A_SLLI, regdst.reghi, regsrc.reghi, value));
                  list.concat(taicpu.op_reg_reg_reg(A_OR, regdst.reghi, tmpreg, regdst.reghi));
                end
              else if value=32 then
                begin
                  cg.a_load_reg_reg(list,OS_INT,OS_INT,regsrc.reglo,regdst.reghi);
                  cg.a_load_const_reg(list,OS_INT,0,regdst.reglo);
                end
              else
                Internalerror(2020082209);
            end;
          OP_SHR:
            begin
              if (value>0) and (value<=15) then
                begin
                  tmpreg:=cg.GetIntRegister(list,OS_32);
                  list.concat(taicpu.op_reg_reg_const(A_SLLI, tmpreg, regsrc.reghi, 32-value));
                  list.concat(taicpu.op_reg_reg_const(A_SRLI, regdst.reglo, regsrc.reglo, value));
                  list.concat(taicpu.op_reg_reg_reg(A_OR, regdst.reglo, tmpreg, regdst.reglo));
                  list.concat(taicpu.op_reg_reg_const(A_SRLI, regdst.reghi, regsrc.reghi, value));
                end
              else if value=32 then
                begin
                  cg.a_load_reg_reg(list,OS_INT,OS_INT,regsrc.reghi,regdst.reglo);
                  cg.a_load_const_reg(list,OS_INT,0,regdst.reghi);
                end
              else
                Internalerror(2020082210);
            end;
          OP_SUB:
            begin
              { for now, we take the simple approach }
              tmpreg64.reglo := cg.GetIntRegister(list,OS_S32);
              tmpreg64.reghi := cg.GetIntRegister(list,OS_S32);
              a_load64_const_reg(list,value,tmpreg64);
              a_op64_reg_reg_reg(list,op,size,tmpreg64,regsrc,regdst);
            end;
          else
            internalerror(2020030901);
        end;
      end;


    procedure tcg64fxtensa.a_op64_const_reg(list : TAsmList; op : TOpCG; size : tcgsize; value : int64; reg : tregister64);
      begin
        a_op64_const_reg_reg(list,op,size,value,reg,reg);
      end;

{$warnings off}
    procedure create_codegen;
      begin
        cg:=tcgcpu.Create;
        cg64:=tcg64fxtensa.Create;
      end;

end.
