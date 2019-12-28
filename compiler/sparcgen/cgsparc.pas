{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the code generator for the SPARC

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
unit cgsparc;

{$i fpcdefs.inc}

interface

    uses
       globtype,parabase,
       cgbase,cgutils,cgobj,
{$ifndef SPARC64}
       cg64f32,
{$endif SPARC64}
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpuinfo,
       node,symconst,SymType,symdef,
       rgcpu;

    type
      TCGSparcGen=class(tcg)
      protected
        function IsSimpleRef(const ref:treference):boolean;
     public
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;
        function  getfpuregister(list:TAsmList;size:Tcgsize):Tregister;override;
        { sparc special, needed by cg64 }
        procedure make_simple_ref(list:TAsmList;var ref: treference);
        procedure handle_load_store(list:TAsmList;isstore:boolean;op: tasmop;reg:tregister;ref: treference);
        procedure handle_reg_const_reg(list:TAsmList;op:Tasmop;src:tregister;a:tcgint;dst:tregister);
        { parameter }
        procedure a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const r : tregister;const paraloc : TCGPara);override;
        procedure a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);override;
        procedure a_call_name(list:TAsmList;const s:string; weak: boolean);override;
        procedure a_call_reg(list:TAsmList;Reg:TRegister);override;
        { General purpose instructions }
        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
        procedure a_op_const_reg(list:TAsmList;Op:TOpCG;size:tcgsize;a:tcgint;reg:TRegister);override;
        procedure a_op_reg_reg(list:TAsmList;Op:TOpCG;size:TCGSize;src, dst:TRegister);override;
        procedure a_op_const_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;a:tcgint;src, dst:tregister);override;
        procedure a_op_reg_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;src1, src2, dst:tregister);override;
        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        { move instructions }
        procedure a_load_const_ref(list:TAsmList;size:tcgsize;a:tcgint;const ref:TReference);override;
        procedure a_load_reg_ref(list:TAsmList;FromSize,ToSize:TCgSize;reg:TRegister;const ref:TReference);override;
        procedure a_load_ref_reg(list:TAsmList;FromSize,ToSize:TCgSize;const ref:TReference;reg:tregister);override;
        procedure a_loadaddr_ref_reg(list:TAsmList;const ref:TReference;r:tregister);override;
        { fpu move instructions }
        procedure a_loadfpu_reg_reg(list:TAsmList;fromsize,tosize:tcgsize;reg1, reg2:tregister);override;
        procedure a_loadfpu_ref_reg(list:TAsmList;fromsize,tosize:tcgsize;const ref:TReference;reg:tregister);override;
        procedure a_loadfpu_reg_ref(list:TAsmList;fromsize,tosize:tcgsize;reg:tregister;const ref:TReference);override;
        { comparison operations }
        procedure a_cmp_const_reg_label(list:TAsmList;size:tcgsize;cmp_op:topcmp;a:tcgint;reg:tregister;l:tasmlabel);override;
        procedure a_cmp_reg_reg_label(list:TAsmList;size:tcgsize;cmp_op:topcmp;reg1,reg2:tregister;l:tasmlabel);override;
        procedure a_jmp_always(List:TAsmList;l:TAsmLabel);override;
        procedure a_jmp_name(list : TAsmList;const s : string);override;
        procedure a_jmp_cond(list:TAsmList;cond:TOpCmp;l:tasmlabel);{ override;}
{$ifdef SPARC64}
        procedure a_jmp_cond64(list:TAsmList;cond:TOpCmp;l:tasmlabel);{ override;}
{$endif SPARC64}
        procedure a_jmp_flags(list:TAsmList;const f:TResFlags;l:tasmlabel);override;
        procedure g_flags2reg(list:TAsmList;Size:TCgSize;const f:tresflags;reg:TRegister);override;
        procedure g_overflowCheck(List:TAsmList;const Loc:TLocation;def:TDef);override;
        procedure g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);override;
        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);override;
        procedure g_maybe_got_init(list: TAsmList); override;
        procedure g_restore_registers(list:TAsmList);override;
        procedure g_save_registers(list : TAsmList);override;
        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;
        procedure g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);override;
        procedure g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);override;
      protected
        use_unlimited_pic_mode : boolean;
      end;

    const
      TOpCG2AsmOp : array[boolean,topcg] of TAsmOp=(
        (
          A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_SMUL,A_UMUL,A_NEG,A_NOT,A_OR,A_SRA,A_SLL,A_SRL,A_SUB,A_XOR,A_NONE,A_NONE
        ),
        (
          A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_MULX,A_MULX,A_NEG,A_NOT,A_OR,A_SRAX,A_SLLX,A_SRLX,A_SUB,A_XOR,A_NONE,A_NONE
        )
      );

      TOpCG2AsmOpWithFlags : array[boolean,topcg] of TAsmOp=(
        (
          A_NONE,A_MOV,A_ADDcc,A_ANDcc,A_UDIVcc,A_SDIVcc,A_SMULcc,A_UMULcc,A_NEG,A_NOT,A_ORcc,A_SRA,A_SLL,A_SRL,A_SUBcc,A_XORcc,A_NONE,A_NONE
        ),
        (
          A_NONE,A_MOV,A_ADDcc,A_ANDcc,A_UDIVcc,A_SDIVcc,A_SMULcc,A_UMULcc,A_NEG,A_NOT,A_ORcc,A_SRAX,A_SLLX,A_SRLX,A_SUBcc,A_XORcc,A_NONE,A_NONE
        )
      );
      TOpCmp2AsmCond : array[topcmp] of TAsmCond=(C_NONE,
        C_E,C_G,C_L,C_GE,C_LE,C_NE,C_BE,C_B,C_AE,C_A
      );


implementation

  uses
    globals,verbose,systems,cutils,
    paramgr,fmodule,
    symtable,symsym,
    tgobj,
    procinfo,cpupi;


    function TCGSparcGen.IsSimpleRef(const ref:treference):boolean;
      begin
        result :=not(assigned(ref.symbol))and
                  (((ref.index = NR_NO) and
                   (ref.offset >= simm13lo) and
                    (ref.offset <= simm13hi)) or
                  ((ref.index <> NR_NO) and
                  (ref.offset = 0)));
      end;


    procedure TCGSparcGen.make_simple_ref(list:TAsmList;var ref: treference);
      var
        href: treference;
        hreg,hreg2: tregister;
      begin
        if (ref.refaddr<>addr_no) then
          InternalError(2013022802);

        if (ref.base=NR_NO) then
          begin
            ref.base:=ref.index;
            ref.index:=NR_NO;
          end;

        if IsSimpleRef(ref) then
          exit;

        if (ref.symbol=nil) then
          begin
            hreg:=getintregister(list,OS_ADDR);
            if (ref.index=NR_NO) then
              a_load_const_reg(list,OS_ADDR,ref.offset,hreg)
            else
              begin
                if (ref.offset<simm13lo) or (ref.offset>simm13hi-sizeof(pint)) then
                  begin
                    a_load_const_reg(list,OS_ADDR,ref.offset,hreg);
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,hreg,ref.index,hreg));
                  end
                else
                  list.concat(taicpu.op_reg_const_reg(A_ADD,ref.index,ref.offset,hreg));
              end;
            if (ref.base=NR_NO) then
              ref.base:=hreg
            else
              ref.index:=hreg;
            ref.offset:=0;
            exit;
          end;

        reference_reset_symbol(href,ref.symbol,ref.offset,ref.alignment,ref.volatility);
        hreg:=getintregister(list,OS_ADDR);
        if not (cs_create_pic in current_settings.moduleswitches) then
          begin
            { absolute loads allow any offset to be encoded into relocation }
            href.refaddr:=addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,href,hreg));
            if (ref.base=NR_NO) and (ref.index=NR_NO) then
              begin
                ref.base:=hreg;
                ref.refaddr:=addr_low;
                exit;
              end;
            { base present -> load the entire address and use it as index }
            href.refaddr:=addr_low;
            list.concat(taicpu.op_reg_ref_reg(A_OR,hreg,href,hreg));
            ref.symbol:=nil;
            ref.offset:=0;
            if (ref.index<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,ref.index,hreg,hreg));
            ref.index:=hreg;
          end
        else
          begin
            include(current_procinfo.flags,pi_needs_got);
            href.offset:=0;
            if use_unlimited_pic_mode then
              begin
                href.refaddr:=addr_high;
                list.concat(taicpu.op_ref_reg(A_SETHI,href,hreg));
                href.refaddr:=addr_low;
                list.concat(taicpu.op_reg_ref_reg(A_OR,hreg,href,hreg));
                reference_reset_base(href,hreg,0,href.temppos,sizeof(pint),[]);
                href.index:=current_procinfo.got;
              end
            else
              begin
                href.base:=current_procinfo.got;
                href.refaddr:=addr_pic;
              end;
            list.concat(taicpu.op_ref_reg(A_LD_R,href,hreg));
            ref.symbol:=nil;
            { hreg now holds symbol address. Add remaining members. }
            if (ref.offset>=simm13lo) and (ref.offset<=simm13hi-sizeof(pint)) then
              begin
                if (ref.base=NR_NO) then
                  ref.base:=hreg
                else
                  begin
                    if (ref.offset<>0) then
                      list.concat(taicpu.op_reg_const_reg(A_ADD,hreg,ref.offset,hreg));
                    if (ref.index<>NR_NO) then
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,hreg,ref.index,hreg));
                    ref.index:=hreg;
                    ref.offset:=0;
                  end;
              end
            else    { large offset, need another register to deal with it }
              begin
                hreg2:=getintregister(list,OS_ADDR);
                a_load_const_reg(list,OS_ADDR,ref.offset,hreg2);
                if (ref.index<>NR_NO) then
                  list.concat(taicpu.op_reg_reg_reg(A_ADD,hreg2,ref.index,hreg2));
                if (ref.base<>NR_NO) then
                  list.concat(taicpu.op_reg_reg_reg(A_ADD,hreg2,ref.base,hreg2));
                ref.base:=hreg;
                ref.index:=hreg2;
                ref.offset:=0;
              end;
          end;
      end;


    procedure TCGSparcGen.handle_load_store(list:TAsmList;isstore:boolean;op: tasmop;reg:tregister;ref: treference);
      begin
        make_simple_ref(list,ref);
        if isstore then
          list.concat(taicpu.op_reg_ref(op,reg,ref))
        else
          list.concat(taicpu.op_ref_reg(op,ref,reg));
      end;


    procedure TCGSparcGen.handle_reg_const_reg(list:TAsmList;op:Tasmop;src:tregister;a:tcgint;dst:tregister);
      var
        tmpreg : tregister;
      begin
        if (a<simm13lo) or
           (a>simm13hi) then
          begin
            tmpreg:=GetIntRegister(list,OS_INT);
            a_load_const_reg(list,OS_INT,a,tmpreg);
            list.concat(taicpu.op_reg_reg_reg(op,src,tmpreg,dst));
          end
        else
          list.concat(taicpu.op_reg_const_reg(op,src,a,dst));
      end;


{****************************************************************************
                              Assembler code
****************************************************************************}

    procedure TCGSparcGen.init_register_allocators;
      begin
        inherited init_register_allocators;

        rg[R_INTREGISTER]:=Trgcpu.create(R_INTREGISTER,R_SUBWHOLE,
            [RS_O0,RS_O1,RS_O2,RS_O3,RS_O4,RS_O5,RS_O7,
             RS_L0,RS_L1,RS_L2,RS_L3,RS_L4,RS_L5,RS_L6,RS_L7,
             RS_I0,RS_I1,RS_I2,RS_I3,RS_I4,RS_I5],
            first_int_imreg,[]);

        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBFS,
            [RS_F0,{RS_F1,}RS_F2,{RS_F3,}RS_F4,{RS_F5,}RS_F6,{RS_F7,}
             RS_F8,{RS_F9,}RS_F10,{RS_F11,}RS_F12,{RS_F13,}RS_F14,{RS_F15,}
             RS_F16,{RS_F17,}RS_F18,{RS_F19,}RS_F20,{RS_F21,}RS_F22,{RS_F23,}
             RS_F24,{RS_F25,}RS_F26,{RS_F27,}RS_F28,{RS_F29,}RS_F30{,RS_F31}],
            first_fpu_imreg,[]);
        { needs at least one element for rgobj not to crash }
        rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBNONE,
            [RS_L0],first_mm_imreg,[]);
      end;


    procedure TCGSparcGen.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


    function TCGSparcGen.getfpuregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if size=OS_F64 then
          result:=rg[R_FPUREGISTER].getregister(list,R_SUBFD)
        else
          result:=rg[R_FPUREGISTER].getregister(list,R_SUBFS);
      end;


    procedure TCGSparcGen.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
      var
         href,href2 : treference;
         hloc : pcgparalocation;
      begin
        href:=ref;
        hloc:=paraloc.location;
        while assigned(hloc) do
          begin
            paramanager.allocparaloc(list,hloc);
            case hloc^.loc of
              LOC_REGISTER,LOC_CREGISTER :
                a_load_ref_reg(list,hloc^.size,hloc^.size,href,hloc^.register);
              LOC_REFERENCE :
                begin
                  reference_reset_base(href2,hloc^.reference.index,hloc^.reference.offset,ctempposinvalid,paraloc.alignment,[]);
                  { concatcopy should choose the best way to copy the data }
                  g_concatcopy(list,href,href2,tcgsize2size[hloc^.size]);

                end;
              LOC_FPUREGISTER,LOC_CFPUREGISTER :
                a_loadfpu_ref_reg(list,hloc^.size,hloc^.size,href,hloc^.register);
              else
                internalerror(200408241);
           end;
           inc(href.offset,tcgsize2size[hloc^.size]);
           hloc:=hloc^.next;
         end;
      end;


    procedure TCGSparcGen.a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const r : tregister;const paraloc : TCGPara);
      var
        href : treference;
      begin
        { happens for function result loc }
        if paraloc.location^.loc in [LOC_FPUREGISTER,LOC_CFPUREGISTER] then
          begin
            paraloc.check_simple_location;
            paramanager.allocparaloc(list,paraloc.location);
            a_loadfpu_reg_reg(list,size,paraloc.location^.size,r,paraloc.location^.register);
          end
        else
          begin
            tg.GetTemp(list,TCGSize2Size[size],TCGSize2Size[size],tt_normal,href);
            a_loadfpu_reg_ref(list,size,size,r,href);
            a_loadfpu_ref_cgpara(list,size,href,paraloc);
            tg.Ungettemp(list,href);
          end;
      end;


    procedure TCGSparcGen.a_call_name(list:TAsmList;const s:string; weak: boolean);
      begin
        if not weak then
          list.concat(taicpu.op_sym(A_CALL,current_asmdata.RefAsmSymbol(s,AT_FUNCTION)))
        else
          list.concat(taicpu.op_sym(A_CALL,current_asmdata.WeakRefAsmSymbol(s,AT_FUNCTION)));
        { Delay slot }
        list.concat(taicpu.op_none(A_NOP));
      end;


    procedure TCGSparcGen.a_call_reg(list:TAsmList;Reg:TRegister);
      begin
        list.concat(taicpu.op_reg(A_CALL,reg));
        { Delay slot }
        list.concat(taicpu.op_none(A_NOP));
     end;


    {********************** load instructions ********************}

    procedure TCGSparcGen.a_load_const_ref(list : TAsmList;size : tcgsize;a : tcgint;const ref : TReference);
      begin
        if a=0 then
          a_load_reg_ref(list,size,size,NR_G0,ref)
        else
          inherited a_load_const_ref(list,size,a,ref);
      end;


    procedure TCGSparcGen.a_load_reg_ref(list:TAsmList;FromSize,ToSize:TCGSize;reg:tregister;const Ref:TReference);
      var
        op : tasmop;
      begin
        if (TCGSize2Size[fromsize] >= TCGSize2Size[tosize]) then
          fromsize := tosize;
        if (ref.alignment<>0) and
           (ref.alignment<tcgsize2size[tosize]) then
          begin
            a_load_reg_ref_unaligned(list,FromSize,ToSize,reg,ref);
          end
        else
          begin
            case tosize of
              { signed integer registers }
              OS_8,
              OS_S8:
                Op:=A_STB;
              OS_16,
              OS_S16:
                Op:=A_STH;
              OS_32,
              OS_S32:
                Op:=A_ST;
{$ifdef SPARC64}
              OS_64,
              OS_S64:
                Op:=A_STX;
{$endif SPARC64}
              else
                InternalError(2002122100);
            end;
            handle_load_store(list,true,op,reg,ref);
          end;
      end;


    procedure TCGSparcGen.a_load_ref_reg(list:TAsmList;FromSize,ToSize:TCgSize;const ref:TReference;reg:tregister);
      var
        op : tasmop;
      begin
        if (TCGSize2Size[fromsize] >= TCGSize2Size[tosize]) then
          fromsize := tosize;
        if (ref.alignment<>0) and
           (ref.alignment<tcgsize2size[fromsize]) then
           begin
             a_load_ref_reg_unaligned(list,FromSize,ToSize,ref,reg);
           end
         else
           begin
             case fromsize of
               OS_S8:
                 Op:=A_LDSB;{Load Signed Byte}
               OS_8:
                 Op:=A_LDUB;{Load Unsigned Byte}
               OS_S16:
                 Op:=A_LDSH;{Load Signed Halfword}
               OS_16:
                 Op:=A_LDUH;{Load Unsigned Halfword}
{$ifdef SPARC64}
               OS_S32:
                 Op:=A_LDSW;{Load Signed Word}
               OS_32:
                 Op:=A_LDUW;{Load Unsigned Word}
               OS_64,
               OS_S64:
                 Op:=A_LDX;
{$else SPARC64}
               OS_S32,
               OS_32:
                 Op:=A_LD;{Load Word}
               OS_S64,
               OS_64:
                 Op:=A_LDD;{Load a Long Word}
{$endif SPARC64}
               else
                 InternalError(2002122101);
             end;
             handle_load_store(list,false,op,reg,ref);
             if (fromsize=OS_S8) and
                (tosize=OS_16) then
               a_load_reg_reg(list,fromsize,tosize,reg,reg);
           end;
      end;


    procedure TCGSparcGen.a_loadaddr_ref_reg(list : TAsmList;const ref : TReference;r : tregister);
      var
         href: treference;
         hreg: tregister;
      begin
        if (ref.base=NR_NO) and (ref.index<>NR_NO) then
          internalerror(200306171);

        if (ref.symbol=nil) then
          begin
            if (ref.base<>NR_NO) then
              begin
                if (ref.offset<simm13lo) or (ref.offset>simm13hi) then
                  begin
                    hreg:=getintregister(list,OS_ADDR);
                    a_load_const_reg(list,OS_ADDR,ref.offset,hreg);
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,hreg,ref.base,r));
                    if (ref.index<>NR_NO) then
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.index,r));
                  end
                else if (ref.offset<>0) then
                  begin
                    list.concat(taicpu.op_reg_const_reg(A_ADD,ref.base,ref.offset,r));
                    if (ref.index<>NR_NO) then
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.index,r));
                  end
                else if (ref.index<>NR_NO) then
                  list.concat(taicpu.op_reg_reg_reg(A_ADD,ref.base,ref.index,r))
                else
                  a_load_reg_reg(list,OS_ADDR,OS_INT,ref.base,r);   { (try to) emit optimizable move }
              end
            else
              a_load_const_reg(list,OS_ADDR,ref.offset,r);
            exit;
          end;

        reference_reset_symbol(href,ref.symbol,ref.offset,ref.alignment,ref.volatility);
        if (cs_create_pic in current_settings.moduleswitches) then
          begin
            include(current_procinfo.flags,pi_needs_got);
            href.offset:=0;
            if use_unlimited_pic_mode then
              begin
                href.refaddr:=addr_high;
                list.concat(taicpu.op_ref_reg(A_SETHI,href,r));
                href.refaddr:=addr_low;
                list.concat(taicpu.op_reg_ref_reg(A_OR,r,href,r));
                reference_reset_base(href,r,0,ctempposinvalid,sizeof(pint),[]);
                href.index:=current_procinfo.got;
              end
            else
              begin
                href.base:=current_procinfo.got;
                href.refaddr:=addr_pic;            { should it be done THAT way?? }
              end;
            { load contents of GOT slot }
            list.concat(taicpu.op_ref_reg(A_LD_R,href,r));
            { add original base/index, if any }
            if (ref.base<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.base,r));
            if (ref.index<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.index,r));
            { finally, add offset }
            if (ref.offset<simm13lo) or (ref.offset>simm13hi) then
              begin
                hreg:=getintregister(list,OS_ADDR);
                a_load_const_reg(list,OS_ADDR,ref.offset,hreg);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,hreg,r,r));
              end
            else if (ref.offset<>0) then
              list.concat(taicpu.op_reg_const_reg(A_ADD,r,ref.offset,r));
          end
        else
          begin
            { load symbol+offset }
            href.refaddr:=addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,href,r));
            href.refaddr:=addr_low;
            list.concat(taicpu.op_reg_ref_reg(A_OR,r,href,r));
            { add original base/index, if any }
            if (ref.base<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.base,r));
            if (ref.index<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.index,r));
          end;
      end;


    procedure TCGSparcGen.a_loadfpu_reg_reg(list:TAsmList;fromsize,tosize:tcgsize;reg1, reg2:tregister);
      const
         FpuMovInstr : Array[OS_F32..OS_F64,OS_F32..OS_F64] of TAsmOp =
           ((A_FMOVS,A_FSTOD),(A_FDTOS,A_FMOVD));
      var
        op: TAsmOp;
        instr : taicpu;
      begin
        op:=fpumovinstr[fromsize,tosize];
        instr:=taicpu.op_reg_reg(op,reg1,reg2);
        list.Concat(instr);
        { Notify the register allocator that we have written a move instruction so
        it can try to eliminate it. }
        if (op = A_FMOVS) or
           (op = A_FMOVD) then
          add_move_instruction(instr);
      end;


    procedure TCGSparcGen.a_loadfpu_ref_reg(list:TAsmList;fromsize,tosize:tcgsize;const ref:TReference;reg:tregister);
       const
         FpuLoadInstr : Array[OS_F32..OS_F64] of TAsmOp =
           (A_LDF,A_LDDF);
       var
         tmpreg: tregister;
       begin
         tmpreg:=NR_NO;
         if (fromsize<>tosize) then
           begin
             tmpreg:=reg;
             reg:=getfpuregister(list,fromsize);
           end;
         handle_load_store(list,false,fpuloadinstr[fromsize],reg,ref);
         if (fromsize<>tosize) then
           a_loadfpu_reg_reg(list,fromsize,tosize,reg,tmpreg);
       end;


     procedure TCGSparcGen.a_loadfpu_reg_ref(list:TAsmList;fromsize,tosize:tcgsize;reg:tregister;const ref:TReference);
       const
         FpuLoadInstr : Array[OS_F32..OS_F64] of TAsmOp =
           (A_STF,A_STDF);
       var
         tmpreg: tregister;
       begin
         if (fromsize<>tosize) then
           begin
             tmpreg:=getfpuregister(list,tosize);
             a_loadfpu_reg_reg(list,fromsize,tosize,reg,tmpreg);
             reg:=tmpreg;
           end;
         handle_load_store(list,true,fpuloadinstr[tosize],reg,ref);
       end;


    procedure TCGSparcGen.maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16{$ifdef SPARC64},OS_32,OS_S32{$endif SPARC64}]) then
          a_load_reg_reg(list,OS_INT,size,dst,dst);
      end;


    procedure TCGSparcGen.a_op_const_reg(list:TAsmList;Op:TOpCG;size:tcgsize;a:tcgint;reg:TRegister);
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


    procedure TCGSparcGen.a_op_reg_reg(list:TAsmList;Op:TOpCG;size:TCGSize;src, dst:TRegister);
      begin
        Case Op of
          OP_NEG :
            list.concat(taicpu.op_reg_reg(TOpCG2AsmOp[size in [OS_64,OS_S64],op],src,dst));
          OP_NOT :
            list.concat(taicpu.op_reg_reg_reg(A_XNOR,src,NR_G0,dst));
          else
            list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[size in [OS_64,OS_S64],op],dst,src,dst));
        end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure TCGSparcGen.a_op_const_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;a:tcgint;src, dst:tregister);
      var
        l: TLocation;
      begin
        a_op_const_reg_reg_checkoverflow(list,op,size,a,src,dst,false,l);
      end;


    procedure TCGSparcGen.a_op_reg_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;src1, src2, dst:tregister);
      begin
        if (TOpcg2AsmOp[size in [OS_64,OS_S64],op]=A_NONE) then
          InternalError(2013070305);
        if (op=OP_SAR) then
          begin
            if (size in [OS_S8,OS_S16]) then
              begin
                { Sign-extend before shifting }
                list.concat(taicpu.op_reg_const_reg(A_SLL,src2,32-(tcgsize2size[size]*8),dst));
                list.concat(taicpu.op_reg_const_reg(A_SRA,dst,32-(tcgsize2size[size]*8),dst));
                src2:=dst;
              end
{$ifdef SPARC64}
            { allow 64 bit sar on sparc64 without ie }
            else if size in [OS_64,OS_S64] then
{$endif SPARC64}
            else if not (size in [OS_32,OS_S32]) then
              InternalError(2013070306);
          end;
        list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[size in [OS_64,OS_S64],op],src2,src1,dst));
        maybeadjustresult(list,op,size,dst);
      end;


    procedure TCGSparcGen.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        tmpreg1,tmpreg2 : tregister;
      begin
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

          OP_SAR:
            begin
              if (size in [OS_S8,OS_S16]) then
                begin
                  list.concat(taicpu.op_reg_const_reg(A_SLL,src,32-(tcgsize2size[size]*8),dst));
                  inc(a,32-tcgsize2size[size]*8);
                  src:=dst;
                end
{$ifndef SPARC64}
              else if not (size in [OS_32,OS_S32]) then
                InternalError(2013070303)
{$endif SPARC64}
                ;
            end;
          else
            ;
        end;
        if setflags then
          begin
            handle_reg_const_reg(list,TOpCG2AsmOpWithFlags[size in [OS_64,OS_S64],op],src,a,dst);
            case op of
              OP_MUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_reg(A_CMP,NR_G0,tmpreg1));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags.Init(NR_ICC,F_NE);
                end;
              OP_IMUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  tmpreg2:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_const_reg(A_SRA,dst,31,tmpreg2));
                  list.concat(taicpu.op_reg_reg(A_CMP,tmpreg1,tmpreg2));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags.Init(NR_ICC,F_NE);
                end;
              else
                ;
            end;
          end
        else
          handle_reg_const_reg(list,TOpCG2AsmOp[size in [OS_64,OS_S64],op],src,a,dst);
        maybeadjustresult(list,op,size,dst);
      end;


    procedure TCGSparcGen.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        tmpreg1,tmpreg2 : tregister;
      begin
        ovloc.loc:=LOC_VOID;
        if setflags then
          begin
            list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOpWithFlags[size in [OS_64,OS_S64],op],src2,src1,dst));
            case op of
              OP_MUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_reg(A_CMP,NR_G0,tmpreg1));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags.Init(NR_ICC,F_NE);
                end;
              OP_IMUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  tmpreg2:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_const_reg(A_SRL,dst,31,tmpreg2));
                  list.concat(taicpu.op_reg_reg(A_CMP,tmpreg1,tmpreg2));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags.Init(NR_ICC,F_NE);
                end;
              else
                ;
            end;
          end
        else
          list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[size in [OS_64,OS_S64],op],src2,src1,dst));
        maybeadjustresult(list,op,size,dst);
      end;


  {*************** compare instructructions ****************}

    procedure TCGSparcGen.a_cmp_const_reg_label(list:TAsmList;size:tcgsize;cmp_op:topcmp;a:tcgint;reg:tregister;l:tasmlabel);
      begin
        if (a=0) then
          list.concat(taicpu.op_reg_reg_reg(A_SUBcc,reg,NR_G0,NR_G0))
        else
          handle_reg_const_reg(list,A_SUBcc,reg,a,NR_G0);
{$ifdef SPARC64}
        if size in [OS_64,OS_S64] then
          a_jmp_cond64(list,cmp_op,l)
        else
{$endif SPARC64}
          a_jmp_cond(list,cmp_op,l);
      end;


    procedure TCGSparcGen.a_cmp_reg_reg_label(list:TAsmList;size:tcgsize;cmp_op:topcmp;reg1,reg2:tregister;l:tasmlabel);
      begin
        list.concat(taicpu.op_reg_reg_reg(A_SUBcc,reg2,reg1,NR_G0));
{$ifdef SPARC64}
        if size in [OS_64,OS_S64] then
          a_jmp_cond64(list,cmp_op,l)
        else
{$endif SPARC64}
          a_jmp_cond(list,cmp_op,l);
      end;


    procedure TCGSparcGen.a_jmp_always(List:TAsmList;l:TAsmLabel);
      begin
        List.Concat(TAiCpu.op_sym(A_BA,current_asmdata.RefAsmSymbol(l.name,AT_FUNCTION)));
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure TCGSparcGen.a_jmp_name(list : TAsmList;const s : string);
      begin
        List.Concat(TAiCpu.op_sym(A_BA,current_asmdata.RefAsmSymbol(s,AT_FUNCTION)));
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure TCGSparcGen.a_jmp_cond(list:TAsmList;cond:TOpCmp;l:TAsmLabel);
      var
        ai:TAiCpu;
      begin
        ai:=TAiCpu.Op_sym(A_Bxx,l);
        ai.SetCondition(TOpCmp2AsmCond[cond]);
        list.Concat(ai);
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


{$ifdef SPARC64}
    procedure TCGSparcGen.a_jmp_cond64(list : TAsmList; cond : TOpCmp; l : tasmlabel);
      var
        ai:TAiCpu;
      begin
        ai:=TAiCpu.Op_reg_sym(A_Bxx,NR_XCC,l);
        ai.SetCondition(TOpCmp2AsmCond[cond]);
        list.Concat(ai);
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;
{$endif SPARC64}


    procedure TCGSparcGen.a_jmp_flags(list:TAsmList;const f:TResFlags;l:tasmlabel);
      var
        ai : taicpu;
      begin
        case f.FlagReg of
{$ifdef SPARC64}
          NR_XCC:
            ai:=Taicpu.op_reg_sym(A_Bxx,f.FlagReg,l);
{$endif SPARC64}
          NR_ICC:
            ai:=Taicpu.op_sym(A_Bxx,l);
          NR_FCC0:
            ai:=Taicpu.op_sym(A_FBxx,l);
          NR_FCC1,NR_FCC2,NR_FCC3:
            ai:=Taicpu.op_reg_sym(A_FBxx,f.FlagReg,l);
          else
            Internalerror(2017070901);
        end;
        ai.SetCondition(flags_to_cond(f));
        list.Concat(ai);
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure TCGSparcGen.g_flags2reg(list:TAsmList;Size:TCgSize;const f:tresflags;reg:TRegister);
      var
        hl : tasmlabel;
        ai : taicpu;
      begin
        if (f.FlagReg=NR_ICC) and (f.Flags in [F_B]) then
          list.concat(taicpu.op_reg_reg_reg(A_ADDX,NR_G0,NR_G0,reg))
        else if (f.FlagReg=NR_ICC) and (f.Flags in [F_AE]) then
          list.concat(taicpu.op_reg_const_reg(A_SUBX,NR_G0,-1,reg))
        else
          begin
            if current_settings.cputype in [cpu_SPARC_V9] then
              begin
                ai:=Taicpu.op_reg_const_reg(A_MOVcc,f.FlagReg,0,reg);
                ai.SetCondition(inverse_cond(flags_to_cond(f)));
                list.Concat(ai);
                ai:=Taicpu.op_reg_const_reg(A_MOVcc,f.FlagReg,1,reg);
                ai.SetCondition(flags_to_cond(f));
                list.Concat(ai);
              end
            else
              begin
                current_asmdata.getjumplabel(hl);
                a_load_const_reg(list,size,1,reg);
                a_jmp_flags(list,f,hl);
                a_load_const_reg(list,size,0,reg);
                a_label(list,hl);
              end;
          end;
      end;


    procedure TCGSparcGen.g_overflowCheck(List:TAsmList;const Loc:TLocation;def:TDef);
      var
        l : tlocation;
      begin
        l.loc:=LOC_VOID;
        g_overflowCheck_loc(list,loc,def,l);
      end;


    procedure TCGSparcGen.g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);
      var
        hl : tasmlabel;
        ai:TAiCpu;
        hflags : tresflags;
      begin
        if not(cs_check_overflow in current_settings.localswitches) then
          exit;
        current_asmdata.getjumplabel(hl);
        case ovloc.loc of
          LOC_VOID:
            begin
              if not((def.typ=pointerdef) or
                    ((def.typ=orddef) and
                     (torddef(def).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                               pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]))) then
                begin
                  ai:=TAiCpu.Op_sym(A_Bxx,hl);
                  ai.SetCondition(C_VC);
                  list.Concat(ai);
                  { Delay slot }
                  list.Concat(TAiCpu.Op_none(A_NOP));
                end
              else
                a_jmp_cond(list,OC_AE,hl);
            end;
          LOC_FLAGS:
            begin
              hflags:=ovloc.resflags;
              inverse_flags(hflags);
              cg.a_jmp_flags(list,hflags,hl);
            end;
          else
            internalerror(200409281);
        end;

        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;

  { *********** entry/exit code and address loading ************ }

    procedure TCGSparcGen.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      begin
        if nostackframe then
          exit;
        { Althogh the SPARC architecture require only word alignment, software
          convention and the operating system require every stack frame to be double word
          aligned }
        LocalSize:=align(LocalSize,8);
        { Execute the SAVE instruction to get a new register window and create a new
          stack frame. In the "SAVE %i6,size,%i6" the first %i6 is related to the state
          before execution of the SAVE instrucion so it is the caller %i6, when the %i6
          after execution of that instruction is the called function stack pointer}
        { constant can be 13 bit signed, since it's negative, size can be max. 4096 }
        if LocalSize>4096 then
          begin
            a_load_const_reg(list,OS_ADDR,-LocalSize,NR_G1);
            list.concat(Taicpu.Op_reg_reg_reg(A_SAVE,NR_STACK_POINTER_REG,NR_G1,NR_STACK_POINTER_REG));
          end
        else
          list.concat(Taicpu.Op_reg_const_reg(A_SAVE,NR_STACK_POINTER_REG,-LocalSize,NR_STACK_POINTER_REG));
      end;

    procedure TCGSparcGen.g_maybe_got_init(list : TAsmList);
      var
        ref : treference;
        hl : tasmlabel;
      begin
        if (cs_create_pic in current_settings.moduleswitches) and
           ((pi_needs_got in current_procinfo.flags) or
           (current_procinfo.procdef.proctypeoption=potype_unitfinalize)) then
          begin
            current_asmdata.getjumplabel(hl);
            list.concat(taicpu.op_sym(A_CALL,hl));
            { ABI recommends the following sequence:
            1:   call   2f
                 sethi  %hi(_GLOBAL_OFFSET_TABLE_+(.-1b)), %l7
            2:   or     %l7, %lo(_GLOBAL_OFFSET_TABLE_+(.-1b)), %l7
                 add    %l7, %o7, %l7 }
            reference_reset_symbol(ref,current_asmdata.RefAsmSymbol('_GLOBAL_OFFSET_TABLE_',AT_DATA),4,sizeof(pint),[]);
            ref.refaddr:=addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,ref,NR_L7));
            cg.a_label(list,hl);
            ref.refaddr:=addr_low;
            ref.offset:=8;
            list.concat(Taicpu.Op_reg_ref_reg(A_OR,NR_L7,ref,NR_L7));
            list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_L7,NR_O7,NR_L7));
            { allocate NR_L7, so reg.allocator does not see it as available }
            list.concat(tai_regalloc.alloc(NR_L7,nil));
          end;
      end;


    procedure TCGSparcGen.g_restore_registers(list:TAsmList);
      begin
        { The sparc port uses the sparc standard calling convetions so this function has no used }
      end;


    procedure TCGSparcGen.g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);
      var
        hr : treference;
      begin
{$ifdef SPARC}
        if paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef) then
          begin
            reference_reset(hr,sizeof(pint),[]);
            hr.offset:=12;
            hr.refaddr:=addr_full;
            if nostackframe then
              begin
                hr.base:=NR_O7;
                list.concat(taicpu.op_ref_reg(A_JMPL,hr,NR_G0));
                list.concat(Taicpu.op_none(A_NOP))
              end
            else
              begin
                { We use trivial restore in the delay slot of the JMPL instruction, as we
                  already set result onto %i0 }
                hr.base:=NR_I7;
                list.concat(taicpu.op_ref_reg(A_JMPL,hr,NR_G0));
                list.concat(Taicpu.op_none(A_RESTORE));
              end;
          end
        else
{$endif SPARC}
          begin
            if nostackframe then
              begin
                { Here we need to use RETL instead of RET so it uses %o7 }
                list.concat(Taicpu.op_none(A_RETL));
                list.concat(Taicpu.op_none(A_NOP))
              end
            else
              begin
                { We use trivial restore in the delay slot of the JMPL instruction, as we
                  already set result onto %i0 }
                list.concat(Taicpu.op_none(A_RET));
                list.concat(Taicpu.op_none(A_RESTORE));
              end;
          end;
      end;


    procedure TCGSparcGen.g_save_registers(list : TAsmList);
      begin
        { The sparc port uses the sparc standard calling convetions so this function has no used }
      end;


    { ************* concatcopy ************ }

    procedure TCGSparcGen.g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
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
        alloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        a_call_name(list,'FPC_MOVE',false);
        dealloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure TCGSparcGen.g_concatcopy(list:TAsmList;const source,dest:treference;len:tcgint);
      var
        tmpreg1,
        hreg,
        countreg: TRegister;
        src, dst: TReference;
        lab: tasmlabel;
        count, count2: longint;

        function reference_is_reusable(const ref: treference): boolean;
          begin
            result:=(ref.base<>NR_NO) and (ref.index=NR_NO) and
              (ref.symbol=nil) and
              (ref.offset>=simm13lo) and (ref.offset+len<=simm13hi);
          end;

      begin
        if len>high(longint) then
          internalerror(2002072704);
        { anybody wants to determine a good value here :)? }
        if len>100 then
          g_concatcopy_move(list,source,dest,len)
        else
          begin
            count:=len div 4;
            if (count<=4) and reference_is_reusable(source) then
              src:=source
            else
              begin
                reference_reset_base(src,getintregister(list,OS_ADDR),0,source.temppos,sizeof(aint),source.volatility);
                a_loadaddr_ref_reg(list,source,src.base);
              end;
            if (count<=4) and reference_is_reusable(dest) then
              dst:=dest
            else
              begin
                reference_reset_base(dst,getintregister(list,OS_ADDR),0,dest.temppos,sizeof(aint),dest.volatility);
                a_loadaddr_ref_reg(list,dest,dst.base);
              end;
            { generate a loop }
            if count>4 then
              begin
                countreg:=GetIntRegister(list,OS_INT);
                tmpreg1:=GetIntRegister(list,OS_INT);
                a_load_const_reg(list,OS_ADDR,count,countreg);
                current_asmdata.getjumplabel(lab);
                a_label(list, lab);
                list.concat(taicpu.op_ref_reg(A_LD,src,tmpreg1));
                list.concat(taicpu.op_reg_ref(A_ST,tmpreg1,dst));
                list.concat(taicpu.op_reg_const_reg(A_ADD,src.base,4,src.base));
                list.concat(taicpu.op_reg_const_reg(A_ADD,dst.base,4,dst.base));
                list.concat(taicpu.op_reg_const_reg(A_SUBcc,countreg,1,countreg));
{$ifdef SPARC64}
                a_jmp_cond64(list,OC_NE,lab);
{$else SPARC64}
                a_jmp_cond(list,OC_NE,lab);
{$endif SPARC64}
                len := len mod 4;
              end;
            { unrolled loop }
            count:=len div 4;
            if count>0 then
              begin
                tmpreg1:=GetIntRegister(list,OS_INT);
                for count2 := 1 to count do
                  begin
                    list.concat(taicpu.op_ref_reg(A_LD,src,tmpreg1));
                    list.concat(taicpu.op_reg_ref(A_ST,tmpreg1,dst));
                    inc(src.offset,4);
                    inc(dst.offset,4);
                  end;
                len := len mod 4;
              end;
            if (len and 4) <> 0 then
              begin
                hreg:=GetIntRegister(list,OS_INT);
                a_load_ref_reg(list,OS_32,OS_32,src,hreg);
                a_load_reg_ref(list,OS_32,OS_32,hreg,dst);
                inc(src.offset,4);
                inc(dst.offset,4);
              end;
            { copy the leftovers }
            if (len and 2) <> 0 then
              begin
                hreg:=GetIntRegister(list,OS_INT);
                a_load_ref_reg(list,OS_16,OS_16,src,hreg);
                a_load_reg_ref(list,OS_16,OS_16,hreg,dst);
                inc(src.offset,2);
                inc(dst.offset,2);
              end;
            if (len and 1) <> 0 then
              begin
                hreg:=GetIntRegister(list,OS_INT);
                a_load_ref_reg(list,OS_8,OS_8,src,hreg);
                a_load_reg_ref(list,OS_8,OS_8,hreg,dst);
              end;
          end;
      end;


    procedure TCGSparcGen.g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        src, dst: TReference;
        tmpreg1,
        countreg: TRegister;
        i : longint;
        lab: tasmlabel;
      begin
        if len>31 then
          g_concatcopy_move(list,source,dest,len)
        else
          begin
            reference_reset(src,source.alignment,source.volatility);
            reference_reset(dst,dest.alignment,dest.volatility);
            { load the address of source into src.base }
            src.base:=GetAddressRegister(list);
            a_loadaddr_ref_reg(list,source,src.base);
            { load the address of dest into dst.base }
            dst.base:=GetAddressRegister(list);
            a_loadaddr_ref_reg(list,dest,dst.base);
            { generate a loop }
            if len>4 then
              begin
                countreg:=GetIntRegister(list,OS_ADDR);
                tmpreg1:=GetIntRegister(list,OS_ADDR);
                a_load_const_reg(list,OS_ADDR,len,countreg);
                current_asmdata.getjumplabel(lab);
                a_label(list, lab);
                list.concat(taicpu.op_ref_reg(A_LDUB,src,tmpreg1));
                list.concat(taicpu.op_reg_ref(A_STB,tmpreg1,dst));
                list.concat(taicpu.op_reg_const_reg(A_ADD,src.base,1,src.base));
                list.concat(taicpu.op_reg_const_reg(A_ADD,dst.base,1,dst.base));
                list.concat(taicpu.op_reg_const_reg(A_SUBcc,countreg,1,countreg));
{$ifdef SPARC64}
                a_jmp_cond64(list,OC_NE,lab);
{$else SPARC64}
                a_jmp_cond(list,OC_NE,lab);
{$endif SPARC64}
              end
            else
              begin
                { unrolled loop }
                tmpreg1:=GetIntRegister(list,OS_ADDR);
                for i:=1 to len do
                  begin
                    list.concat(taicpu.op_ref_reg(A_LDUB,src,tmpreg1));
                    list.concat(taicpu.op_reg_ref(A_STB,tmpreg1,dst));
                    inc(src.offset);
                    inc(dst.offset);
                  end;
              end;
          end;
      end;


    procedure TCGSparcGen.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
      begin
        { This method is integrated into g_intf_wrapper and shouldn't be called separately }
        InternalError(2013020102);
      end;

end.
