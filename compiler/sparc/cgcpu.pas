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
unit cgcpu;

{$i fpcdefs.inc}

interface

    uses
       globtype,parabase,
       cgbase,cgutils,cgobj,cg64f32,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpuinfo,
       node,symconst,SymType,symdef,
       rgcpu;

    type
      TCgSparc=class(tcg)
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
        procedure a_load_const_cgpara(list:TAsmList;size:tcgsize;a:aint;const paraloc:TCGPara);override;
        procedure a_load_ref_cgpara(list:TAsmList;sz:tcgsize;const r:TReference;const paraloc:TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list:TAsmList;const r:TReference;const paraloc:TCGPara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list:TAsmList;size:tcgsize;a:aint;const paraloc:TCGPara);override;
        procedure a_load_ref_cgpara(list:TAsmList;sz:tcgsize;const r:TReference;const paraloc:TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list:TAsmList;const r:TReference;const paraloc:TCGPara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list:TAsmList;size:tcgsize;a:aint;const paraloc:TCGPara);override;
        procedure a_load_ref_cgpara(list:TAsmList;sz:tcgsize;const r:TReference;const paraloc:TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list:TAsmList;const r:TReference;const paraloc:TCGPara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list:TAsmList;size:tcgsize;a:aint;const paraloc:TCGPara);override;
        procedure a_load_ref_cgpara(list:TAsmList;sz:tcgsize;const r:TReference;const paraloc:TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list:TAsmList;const r:TReference;const paraloc:TCGPara);override;
>>>>>>> origin/cpstrnew
        procedure a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const r : tregister;const paraloc : TCGPara);override;
        procedure a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);override;
        procedure a_call_name(list:TAsmList;const s:string; weak: boolean);override;
        procedure a_call_reg(list:TAsmList;Reg:TRegister);override;
        { General purpose instructions }
        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
<<<<<<< HEAD
<<<<<<< HEAD
        procedure a_op_const_reg(list:TAsmList;Op:TOpCG;size:tcgsize;a:tcgint;reg:TRegister);override;
=======
=======
>>>>>>> origin/fixes_2_2
        procedure a_op_const_reg(list:TAsmList;Op:TOpCG;size:tcgsize;a:aint;reg:TRegister);override;
>>>>>>> graemeg/fixes_2_2
        procedure a_op_reg_reg(list:TAsmList;Op:TOpCG;size:TCGSize;src, dst:TRegister);override;
        procedure a_op_const_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;a:tcgint;src, dst:tregister);override;
        procedure a_op_reg_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;src1, src2, dst:tregister);override;
        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        { move instructions }
        procedure a_load_const_reg(list:TAsmList;size:tcgsize;a:tcgint;reg:tregister);override;
        procedure a_load_const_ref(list:TAsmList;size:tcgsize;a:tcgint;const ref:TReference);override;
        procedure a_load_reg_ref(list:TAsmList;FromSize,ToSize:TCgSize;reg:TRegister;const ref:TReference);override;
        procedure a_load_ref_reg(list:TAsmList;FromSize,ToSize:TCgSize;const ref:TReference;reg:tregister);override;
        procedure a_load_reg_reg(list:TAsmList;FromSize,ToSize:TCgSize;reg1,reg2:tregister);override;
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
        procedure a_jmp_flags(list:TAsmList;const f:TResFlags;l:tasmlabel);override;
        procedure g_flags2reg(list:TAsmList;Size:TCgSize;const f:tresflags;reg:TRegister);override;
        procedure g_overflowCheck(List:TAsmList;const Loc:TLocation;def:TDef);override;
        procedure g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);override;
        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);override;
<<<<<<< HEAD
<<<<<<< HEAD
        procedure g_maybe_got_init(list: TAsmList); override;
        procedure g_restore_registers(list:TAsmList);override;
        procedure g_save_registers(list : TAsmList);override;
<<<<<<< HEAD
        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;
        procedure g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);override;
        procedure g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);override;
       private
        use_unlimited_pic_mode : boolean;
=======
=======
        procedure g_restore_registers(list:TAsmList);override;
        procedure g_save_registers(list : TAsmList);override;
>>>>>>> graemeg/fixes_2_2
=======
        procedure g_restore_registers(list:TAsmList);override;
        procedure g_save_registers(list : TAsmList);override;
>>>>>>> origin/fixes_2_2
        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : aint);override;
        procedure g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : aint);override;
        procedure g_concatcopy_move(list : TAsmList;const source,dest : treference;len : aint);
        procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);override;
       private
        g1_used : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;

      TCg64Sparc=class(tcg64f32)
      private
        procedure get_64bit_ops(op:TOpCG;var op1,op2:TAsmOp;checkoverflow : boolean);
      public
        procedure a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference);override;
        procedure a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64);override;
        procedure a_load64_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara);override;
        procedure a_op64_reg_reg(list:TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst:TRegister64);override;
        procedure a_op64_const_reg(list:TAsmList;op:TOpCG;size : tcgsize;value:int64;regdst:TRegister64);override;
        procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);override;
        procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
        procedure a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
      end;
      
    procedure create_codegen;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    procedure create_codegen;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    const
      TOpCG2AsmOp : array[topcg] of TAsmOp=(
        A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_SMUL,A_UMUL,A_NEG,A_NOT,A_OR,A_SRA,A_SLL,A_SRL,A_SUB,A_XOR,A_NONE,A_NONE
      );
      TOpCG2AsmOpWithFlags : array[topcg] of TAsmOp=(
        A_NONE,A_MOV,A_ADDcc,A_ANDcc,A_UDIVcc,A_SDIVcc,A_SMULcc,A_UMULcc,A_NEG,A_NOT,A_ORcc,A_SRA,A_SLL,A_SRL,A_SUBcc,A_XORcc,A_NONE,A_NONE
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


    function TCgSparc.IsSimpleRef(const ref:treference):boolean;
      begin
        result :=not(assigned(ref.symbol))and
                  (((ref.index = NR_NO) and
                   (ref.offset >= simm13lo) and
                    (ref.offset <= simm13hi)) or
                  ((ref.index <> NR_NO) and
                  (ref.offset = 0)));
      end;


    procedure tcgsparc.make_simple_ref(list:TAsmList;var ref: treference);
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
            hreg:=getintregister(list,OS_INT);
            if (ref.index=NR_NO) then
              a_load_const_reg(list,OS_INT,ref.offset,hreg)
            else
              begin
                if (ref.offset<simm13lo) or (ref.offset>simm13hi-sizeof(pint)) then
                  begin
                    a_load_const_reg(list,OS_INT,ref.offset,hreg);
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

        reference_reset_symbol(href,ref.symbol,ref.offset,ref.alignment);
        hreg:=getintregister(list,OS_INT);
        if not (cs_create_pic in current_settings.moduleswitches) then
          begin
<<<<<<< HEAD
            { absolute loads allow any offset to be encoded into relocation }
            href.refaddr:=addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,href,hreg));
            if (ref.base=NR_NO) and (ref.index=NR_NO) then
              begin
                ref.base:=hreg;
                ref.refaddr:=addr_low;
                exit;
=======
            tmpreg:=GetIntRegister(list,OS_INT);
            reference_reset(tmpref);
            tmpref.symbol:=ref.symbol;
            tmpref.offset:=ref.offset;
            tmpref.refaddr:=addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,tmpref,tmpreg));
            if (ref.offset=0) and (ref.index=NR_NO) and
              (ref.base=NR_NO) then
              begin
                ref.refaddr:=addr_low;
              end
            else
              begin
                { Load the low part is left }
                tmpref.refaddr:=addr_low;
                list.concat(taicpu.op_reg_ref_reg(A_OR,tmpreg,tmpref,tmpreg));
                ref.offset:=0;
                { symbol is loaded }
                ref.symbol:=nil;
>>>>>>> graemeg/fixes_2_2
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
                reference_reset_base(href,hreg,0,sizeof(pint));
                href.index:=current_procinfo.got;
              end
            else
              begin
                href.base:=current_procinfo.got;
                href.refaddr:=addr_pic;
              end;
            list.concat(taicpu.op_ref_reg(A_LD,href,hreg));
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
                hreg2:=getintregister(list,OS_INT);
                a_load_const_reg(list,OS_INT,ref.offset,hreg2);
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


    procedure tcgsparc.handle_load_store(list:TAsmList;isstore:boolean;op: tasmop;reg:tregister;ref: treference);
      begin
        make_simple_ref(list,ref);
        if isstore then
          list.concat(taicpu.op_reg_ref(op,reg,ref))
        else
          list.concat(taicpu.op_ref_reg(op,ref,reg));
      end;


    procedure tcgsparc.handle_reg_const_reg(list:TAsmList;op:Tasmop;src:tregister;a:tcgint;dst:tregister);
      var
        tmpreg : tregister;
      begin
        if (a<simm13lo) or
           (a>simm13hi) then
          begin
            if g1_used then
              GetIntRegister(list,OS_INT)
            else
              begin
                tmpreg:=NR_G1;
                g1_used:=true;
              end;   
            a_load_const_reg(list,OS_INT,a,tmpreg);
            list.concat(taicpu.op_reg_reg_reg(op,src,tmpreg,dst));
            if tmpreg=NR_G1 then
              g1_used:=false;
          end
        else
          list.concat(taicpu.op_reg_const_reg(op,src,a,dst));
      end;


{****************************************************************************
                              Assembler code
****************************************************************************}

    procedure Tcgsparc.init_register_allocators;
      begin
        inherited init_register_allocators;

        rg[R_INTREGISTER]:=Trgcpu.create(R_INTREGISTER,R_SUBD,
            [RS_O0,RS_O1,RS_O2,RS_O3,RS_O4,RS_O5,RS_O7,
             RS_L0,RS_L1,RS_L2,RS_L3,RS_L4,RS_L5,RS_L6,RS_L7,
             RS_I0,RS_I1,RS_I2,RS_I3,RS_I4,RS_I5],
            first_int_imreg,[]);

        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBFS,
            [RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7,
             RS_F8,RS_F9,RS_F10,RS_F11,RS_F12,RS_F13,RS_F14,RS_F15,
             RS_F16,RS_F17,RS_F18,RS_F19,RS_F20,RS_F21,RS_F22,RS_F23,
             RS_F24,RS_F25,RS_F26,RS_F27,RS_F28,RS_F29,RS_F30,RS_F31],
            first_fpu_imreg,[]);
        { needs at least one element for rgobj not to crash }
        rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBNONE,
            [RS_L0],first_mm_imreg,[]);
      end;


    procedure Tcgsparc.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


    function tcgsparc.getfpuregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if size=OS_F64 then
          result:=rg[R_FPUREGISTER].getregister(list,R_SUBFD)
        else
          result:=rg[R_FPUREGISTER].getregister(list,R_SUBFS);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure TCgSparc.a_load_const_cgpara(list:TAsmList;size:tcgsize;a:aint;const paraloc:TCGPara);
      var
        Ref:TReference;
      begin
        paraloc.check_simple_location;
        paramanager.alloccgpara(list,paraloc);
        case paraloc.location^.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_load_const_reg(list,size,a,paraloc.location^.register);
          LOC_REFERENCE:
            begin
              { Code conventions need the parameters being allocated in %o6+92 }
              with paraloc.location^.Reference do
                begin
                  if (Index=NR_SP) and (Offset<Target_info.first_parm_offset) then
                    InternalError(2002081104);
                  reference_reset_base(ref,index,offset,paraloc.alignment);
                end;
              a_load_const_ref(list,size,a,ref);
            end;
          else
            InternalError(2002122200);
        end;
      end;


    procedure TCgSparc.a_load_ref_cgpara(list:TAsmList;sz:TCgSize;const r:TReference;const paraloc:TCGPara);
      var
        ref: treference;
        tmpreg:TRegister;
      begin
        paraloc.check_simple_location;
        paramanager.alloccgpara(list,paraloc);
        with paraloc.location^ do
          begin
            case loc of
              LOC_REGISTER,LOC_CREGISTER :
                a_load_ref_reg(list,sz,paraloc.location^.size,r,Register);
              LOC_REFERENCE:
                begin
                  { Code conventions need the parameters being allocated in %o6+92 }
                  with Reference do
                    begin
                      if (Index=NR_SP) and (Offset<Target_info.first_parm_offset) then
                        InternalError(2002081104);
                      reference_reset_base(ref,index,offset,paraloc.alignment);
                    end;
                  if g1_used then
                    GetIntRegister(list,OS_INT)
                  else
                    begin
                      tmpreg:=NR_G1;
                      g1_used:=true;
                    end;   
                  a_load_ref_reg(list,sz,sz,r,tmpreg);
                  a_load_reg_ref(list,sz,sz,tmpreg,ref);
                  if tmpreg=NR_G1 then
                    g1_used:=false;
                end;
              else
                internalerror(2002081103);
            end;
          end;
      end;


    procedure TCgSparc.a_loadaddr_ref_cgpara(list:TAsmList;const r:TReference;const paraloc:TCGPara);
      var
        Ref:TReference;
        TmpReg:TRegister;
      begin
        paraloc.check_simple_location;
        paramanager.alloccgpara(list,paraloc);
        with paraloc.location^ do
          begin
            case loc of
              LOC_REGISTER,LOC_CREGISTER:
                a_loadaddr_ref_reg(list,r,register);
              LOC_REFERENCE:
                begin
                  reference_reset(ref,paraloc.alignment);
                  ref.base := reference.index;
                  ref.offset := reference.offset;
                  tmpreg:=GetAddressRegister(list);
                  a_loadaddr_ref_reg(list,r,tmpreg);
                  a_load_reg_ref(list,OS_ADDR,OS_ADDR,tmpreg,ref);
                end;
              else
                internalerror(2002080701);
            end;
          end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure tcgsparc.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
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
                  reference_reset_base(href2,hloc^.reference.index,hloc^.reference.offset,paraloc.alignment);
                  a_load_ref_ref(list,hloc^.size,hloc^.size,href,href2);
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


    procedure tcgsparc.a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const r : tregister;const paraloc : TCGPara);
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


    procedure TCgSparc.a_call_name(list:TAsmList;const s:string; weak: boolean);
      begin
        if not weak then
          list.concat(taicpu.op_sym(A_CALL,current_asmdata.RefAsmSymbol(s)))
        else
          list.concat(taicpu.op_sym(A_CALL,current_asmdata.WeakRefAsmSymbol(s)));
        { Delay slot }
        list.concat(taicpu.op_none(A_NOP));
      end;


    procedure TCgSparc.a_call_reg(list:TAsmList;Reg:TRegister);
      begin
        list.concat(taicpu.op_reg(A_CALL,reg));
        { Delay slot }
        list.concat(taicpu.op_none(A_NOP));
     end;


    {********************** load instructions ********************}

    procedure TCgSparc.a_load_const_reg(list : TAsmList;size : TCGSize;a : tcgint;reg : TRegister);
      begin
        { we don't use the set instruction here because it could be evalutated to two
          instructions which would cause problems with the delay slot (FK) }
        if (a=0) then
          list.concat(taicpu.op_reg(A_CLR,reg))
        else if (a>=simm13lo) and (a<=simm13hi) then
          list.concat(taicpu.op_const_reg(A_MOV,a,reg))
        else
          begin
            list.concat(taicpu.op_const_reg(A_SETHI,aint(a) shr 10,reg));
            if (aint(a) and aint($3ff))<>0 then
              list.concat(taicpu.op_reg_const_reg(A_OR,reg,aint(a) and aint($3ff),reg));
          end;
      end;


    procedure TCgSparc.a_load_const_ref(list : TAsmList;size : tcgsize;a : tcgint;const ref : TReference);
      begin
        if a=0 then
          a_load_reg_ref(list,size,size,NR_G0,ref)
        else
          inherited a_load_const_ref(list,size,a,ref);
      end;


    procedure TCgSparc.a_load_reg_ref(list:TAsmList;FromSize,ToSize:TCGSize;reg:tregister;const Ref:TReference);
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
              else
                InternalError(2002122100);
            end;
            handle_load_store(list,true,op,reg,ref);
          end;
      end;


    procedure TCgSparc.a_load_ref_reg(list:TAsmList;FromSize,ToSize:TCgSize;const ref:TReference;reg:tregister);
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
               OS_S32,
               OS_32:
                 Op:=A_LD;{Load Word}
               OS_S64,
               OS_64:
                 Op:=A_LDD;{Load a Long Word}
               else
                 InternalError(2002122101);
             end;
             handle_load_store(list,false,op,reg,ref);
             if (fromsize=OS_S8) and
                (tosize=OS_16) then
               a_load_reg_reg(list,fromsize,tosize,reg,reg);
           end;
      end;


    procedure TCgSparc.a_load_reg_reg(list:TAsmList;fromsize,tosize:tcgsize;reg1,reg2:tregister);
      var
        instr : taicpu;
      begin
         if (tcgsize2size[fromsize] > tcgsize2size[tosize]) or
            ((tcgsize2size[fromsize] = tcgsize2size[tosize]) and
             (fromsize <> tosize)) or
            { needs to mask out the sign in the top 16 bits }
            ((fromsize = OS_S8) and
             (tosize = OS_16)) then
           case tosize of
             OS_8 :
<<<<<<< HEAD
<<<<<<< HEAD
               list.concat(taicpu.op_reg_const_reg(A_AND,reg1,$ff,reg2));
             OS_16 :
               begin
                 list.concat(taicpu.op_reg_const_reg(A_SLL,reg1,16,reg2));
                 list.concat(taicpu.op_reg_const_reg(A_SRL,reg2,16,reg2));
               end;
=======
               a_op_const_reg_reg(list,OP_AND,tosize,$ff,reg1,reg2);
             OS_16 :
               a_op_const_reg_reg(list,OP_AND,tosize,$ffff,reg1,reg2);
>>>>>>> graemeg/fixes_2_2
=======
               a_op_const_reg_reg(list,OP_AND,tosize,$ff,reg1,reg2);
             OS_16 :
               a_op_const_reg_reg(list,OP_AND,tosize,$ffff,reg1,reg2);
>>>>>>> origin/fixes_2_2
             OS_32,
             OS_S32 :
               begin
                 instr:=taicpu.op_reg_reg(A_MOV,reg1,reg2);
                 list.Concat(instr);
                 { Notify the register allocator that we have written a move instruction so
                  it can try to eliminate it. }
                 add_move_instruction(instr);
               end;
             OS_S8 :
               begin
                 list.concat(taicpu.op_reg_const_reg(A_SLL,reg1,24,reg2));
                 list.concat(taicpu.op_reg_const_reg(A_SRA,reg2,24,reg2));
               end;
             OS_S16 :
               begin
                 list.concat(taicpu.op_reg_const_reg(A_SLL,reg1,16,reg2));
                 list.concat(taicpu.op_reg_const_reg(A_SRA,reg2,16,reg2));
               end;
             else
               internalerror(2002090901);
           end
         else
           begin
             instr:=taicpu.op_reg_reg(A_MOV,reg1,reg2);
             list.Concat(instr);
             { Notify the register allocator that we have written a move instruction so
              it can try to eliminate it. }
             add_move_instruction(instr);
           end;
      end;


    procedure TCgSparc.a_loadaddr_ref_reg(list : TAsmList;const ref : TReference;r : tregister);
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
                    hreg:=getintregister(list,OS_INT);
                    a_load_const_reg(list,OS_INT,ref.offset,hreg);
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
                  a_load_reg_reg(list,OS_INT,OS_INT,ref.base,r);   { (try to) emit optimizable move }
              end
            else
              a_load_const_reg(list,OS_INT,ref.offset,r);
            exit;
          end;

        reference_reset_symbol(href,ref.symbol,ref.offset,ref.alignment);
        if (cs_create_pic in current_settings.moduleswitches) then
          begin
<<<<<<< HEAD
            include(current_procinfo.flags,pi_needs_got);
            href.offset:=0;
            if use_unlimited_pic_mode then
=======
            hreg:=GetAddressRegister(list);
            reference_reset(tmpref);
            tmpref.symbol := href.symbol;
            tmpref.offset := href.offset;
            tmpref.refaddr := addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,tmpref,hreg));
            { Only the low part is left }
            tmpref.refaddr:=addr_low;
            list.concat(taicpu.op_reg_ref_reg(A_OR,hreg,tmpref,hreg));
            if href.base<>NR_NO then
>>>>>>> graemeg/fixes_2_2
              begin
                href.refaddr:=addr_high;
                list.concat(taicpu.op_ref_reg(A_SETHI,href,r));
                href.refaddr:=addr_low;
                list.concat(taicpu.op_reg_ref_reg(A_OR,r,href,r));
                reference_reset_base(href,r,0,sizeof(pint));
                href.index:=current_procinfo.got;
              end
            else
              begin
                href.base:=current_procinfo.got;
                href.refaddr:=addr_pic;            { should it be done THAT way?? }
              end;
            { load contents of GOT slot }
            list.concat(taicpu.op_ref_reg(A_LD,href,r));
            { add original base/index, if any }
            if (ref.base<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.base,r));
            if (ref.index<>NR_NO) then
              list.concat(taicpu.op_reg_reg_reg(A_ADD,r,ref.index,r));
            { finally, add offset }
            if (ref.offset<simm13lo) or (ref.offset>simm13hi) then
              begin
                hreg:=getintregister(list,OS_INT);
                a_load_const_reg(list,OS_INT,ref.offset,hreg);
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


    procedure TCgSparc.a_loadfpu_reg_reg(list:TAsmList;fromsize,tosize:tcgsize;reg1, reg2:tregister);
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


    procedure TCgSparc.a_loadfpu_ref_reg(list:TAsmList;fromsize,tosize:tcgsize;const ref:TReference;reg:tregister);
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


     procedure TCgSparc.a_loadfpu_reg_ref(list:TAsmList;fromsize,tosize:tcgsize;reg:tregister;const ref:TReference);
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


    procedure tcgsparc.maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16]) then
          a_load_reg_reg(list,OS_32,size,dst,dst);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure TCgSparc.a_op_const_reg(list:TAsmList;Op:TOpCG;size:tcgsize;a:tcgint;reg:TRegister);
=======
=======
>>>>>>> origin/fixes_2_2
    procedure TCgSparc.a_op_const_reg(list:TAsmList;Op:TOpCG;size:tcgsize;a:aint;reg:TRegister);
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
          a_op_const_reg_reg(list,op,size,a,reg,reg);
        end;
=======
          handle_reg_const_reg(list,TOpCG2AsmOp[op],reg,a,reg);
        maybeadjustresult(list,op,size,reg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure TCgSparc.a_op_reg_reg(list:TAsmList;Op:TOpCG;size:TCGSize;src, dst:TRegister);
      begin
        Case Op of
          OP_NEG :
            list.concat(taicpu.op_reg_reg(TOpCG2AsmOp[op],src,dst));
          OP_NOT :
            list.concat(taicpu.op_reg_reg_reg(A_XNOR,src,NR_G0,dst));
          else
            list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[op],dst,src,dst));
        end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure TCgSparc.a_op_const_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;a:tcgint;src, dst:tregister);
      var
        l: TLocation;
      begin
<<<<<<< HEAD
        a_op_const_reg_reg_checkoverflow(list,op,size,a,src,dst,false,l);
=======
        case op of
          OP_MUL,
          OP_IMUL:
            begin
              if ispowerof2(a,power) then
                begin
                  { can be done with a shift }
                  inherited a_op_const_reg_reg(list,op,size,a,src,dst);
                  exit;
                end;
            end;
          OP_SUB,
          OP_ADD :
            begin
              if (a=0) then
                begin
                  a_load_reg_reg(list,size,size,src,dst);
                  exit;
                end;
            end;
        end;
        handle_reg_const_reg(list,TOpCG2AsmOp[op],src,a,dst);
        maybeadjustresult(list,op,size,dst);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure TCgSparc.a_op_reg_reg_reg(list:TAsmList;op:TOpCg;size:tcgsize;src1, src2, dst:tregister);
      begin
        if (TOpcg2AsmOp[op]=A_NONE) then
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
            else if not (size in [OS_32,OS_S32]) then
              InternalError(2013070306);
          end;
        list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[op],src2,src1,dst));
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgsparc.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
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
              else if not (size in [OS_32,OS_S32]) then
                InternalError(2013070303);
            end;
        end;
        if setflags then
          begin
            handle_reg_const_reg(list,TOpCG2AsmOpWithFlags[op],src,a,dst);
            case op of
              OP_MUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_reg(A_CMP,NR_G0,tmpreg1));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags:=F_NE;
                end;
              OP_IMUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  tmpreg2:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_const_reg(A_SRA,dst,31,tmpreg2));
                  list.concat(taicpu.op_reg_reg(A_CMP,tmpreg1,tmpreg2));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags:=F_NE;
                end;
            end;
          end
        else
          handle_reg_const_reg(list,TOpCG2AsmOp[op],src,a,dst);
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgsparc.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        tmpreg1,tmpreg2 : tregister;
      begin
        ovloc.loc:=LOC_VOID;
        if setflags then
          begin
            list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOpWithFlags[op],src2,src1,dst));
            case op of
              OP_MUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_reg(A_CMP,NR_G0,tmpreg1));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags:=F_NE;
                end;
              OP_IMUL:
                begin
                  tmpreg1:=GetIntRegister(list,OS_INT);
                  tmpreg2:=GetIntRegister(list,OS_INT);
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_Y,tmpreg1));
                  list.concat(taicpu.op_reg_const_reg(A_SRL,dst,31,tmpreg2));
                  list.concat(taicpu.op_reg_reg(A_CMP,tmpreg1,tmpreg2));
                  ovloc.loc:=LOC_FLAGS;
                  ovloc.resflags:=F_NE;
                end;
            end;
          end
        else
          list.concat(taicpu.op_reg_reg_reg(TOpCG2AsmOp[op],src2,src1,dst));
        maybeadjustresult(list,op,size,dst);
      end;



  {*************** compare instructructions ****************}

    procedure TCgSparc.a_cmp_const_reg_label(list:TAsmList;size:tcgsize;cmp_op:topcmp;a:tcgint;reg:tregister;l:tasmlabel);
      begin
        if (a=0) then
          list.concat(taicpu.op_reg_reg_reg(A_SUBcc,reg,NR_G0,NR_G0))
        else
          handle_reg_const_reg(list,A_SUBcc,reg,a,NR_G0);
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure TCgSparc.a_cmp_reg_reg_label(list:TAsmList;size:tcgsize;cmp_op:topcmp;reg1,reg2:tregister;l:tasmlabel);
      begin
        list.concat(taicpu.op_reg_reg_reg(A_SUBcc,reg2,reg1,NR_G0));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure TCgSparc.a_jmp_always(List:TAsmList;l:TAsmLabel);
      begin
        List.Concat(TAiCpu.op_sym(A_BA,current_asmdata.RefAsmSymbol(l.name)));
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure tcgsparc.a_jmp_name(list : TAsmList;const s : string);
      begin
        List.Concat(TAiCpu.op_sym(A_BA,current_asmdata.RefAsmSymbol(s)));
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure TCgSparc.a_jmp_cond(list:TAsmList;cond:TOpCmp;l:TAsmLabel);
      var
        ai:TAiCpu;
      begin
        ai:=TAiCpu.Op_sym(A_Bxx,l);
        ai.SetCondition(TOpCmp2AsmCond[cond]);
        list.Concat(ai);
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure TCgSparc.a_jmp_flags(list:TAsmList;const f:TResFlags;l:tasmlabel);
      var
        ai : taicpu;
      begin
        ai:=Taicpu.op_sym(A_Bxx,l);
        ai.SetCondition(flags_to_cond(f));
        list.Concat(ai);
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));
      end;


    procedure TCgSparc.g_flags2reg(list:TAsmList;Size:TCgSize;const f:tresflags;reg:TRegister);
      var
        hl : tasmlabel;
      begin
        if (f in [F_B]) then
          list.concat(taicpu.op_reg_reg_reg(A_ADDX,NR_G0,NR_G0,reg))
        else if (f in [F_AE]) then
          list.concat(taicpu.op_reg_const_reg(A_SUBX,NR_G0,-1,reg))
        else
          begin
            current_asmdata.getjumplabel(hl);
            a_load_const_reg(list,size,1,reg);
            a_jmp_flags(list,f,hl);
            a_load_const_reg(list,size,0,reg);
            a_label(list,hl);
          end;
      end;


    procedure tcgsparc.g_overflowCheck(List:TAsmList;const Loc:TLocation;def:TDef);
      var
        l : tlocation;
      begin
        l.loc:=LOC_VOID;
        g_overflowCheck_loc(list,loc,def,l);
      end;


    procedure TCgSparc.g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);
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
                                               pasbool8,pasbool16,pasbool32,pasbool64]))) then
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

    procedure TCgSparc.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
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
            g1_used:=true;
            list.concat(Taicpu.Op_reg_reg_reg(A_SAVE,NR_STACK_POINTER_REG,NR_G1,NR_STACK_POINTER_REG));
            g1_used:=false;
          end
        else
          list.concat(Taicpu.Op_reg_const_reg(A_SAVE,NR_STACK_POINTER_REG,-LocalSize,NR_STACK_POINTER_REG));
      end;

    procedure TCgSparc.g_maybe_got_init(list : TAsmList);
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
            reference_reset_symbol(ref,current_asmdata.RefAsmSymbol('_GLOBAL_OFFSET_TABLE_'),4,sizeof(pint));
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


    procedure TCgSparc.g_restore_registers(list:TAsmList);
      begin
        { The sparc port uses the sparc standard calling convetions so this function has no used }
      end;


    procedure TCgSparc.g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);
      var
        hr : treference;
      begin
        if paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef) then
          begin
            reference_reset(hr,sizeof(pint));
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


    procedure TCgSparc.g_save_registers(list : TAsmList);
      begin
        { The sparc port uses the sparc standard calling convetions so this function has no used }
      end;


    { ************* concatcopy ************ }

    procedure tcgsparc.g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        paraloc1,paraloc2,paraloc3 : TCGPara;
        pd : tprocdef;
      begin
        pd:=search_system_proc('MOVE');
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
<<<<<<< HEAD
        paramanager.getintparaloc(list,pd,1,paraloc1);
        paramanager.getintparaloc(list,pd,2,paraloc2);
        paramanager.getintparaloc(list,pd,3,paraloc3);
        a_load_const_cgpara(list,OS_SINT,len,paraloc3);
=======
        paramanager.getintparaloc(pocall_default,1,paraloc1);
        paramanager.getintparaloc(pocall_default,2,paraloc2);
        paramanager.getintparaloc(pocall_default,3,paraloc3);
        a_load_const_cgpara(list,OS_INT,len,paraloc3);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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


    procedure TCgSparc.g_concatcopy(list:TAsmList;const source,dest:treference;len:tcgint);
      var
        tmpreg1,
        hreg,
        countreg: TRegister;
        src, dst: TReference;
        lab: tasmlabel;
        count, count2: aint;

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
                reference_reset_base(src,getintregister(list,OS_ADDR),0,sizeof(aint));
                a_loadaddr_ref_reg(list,source,src.base);
              end;
            if (count<=4) and reference_is_reusable(dest) then
              dst:=dest
            else
              begin
                reference_reset_base(dst,getintregister(list,OS_ADDR),0,sizeof(aint));
                a_loadaddr_ref_reg(list,dest,dst.base);
              end;
            { generate a loop }
            if count>4 then
              begin
                countreg:=GetIntRegister(list,OS_INT);
                tmpreg1:=GetIntRegister(list,OS_INT);
                a_load_const_reg(list,OS_INT,count,countreg);
                current_asmdata.getjumplabel(lab);
                a_label(list, lab);
                list.concat(taicpu.op_ref_reg(A_LD,src,tmpreg1));
                list.concat(taicpu.op_reg_ref(A_ST,tmpreg1,dst));
                list.concat(taicpu.op_reg_const_reg(A_ADD,src.base,4,src.base));
                list.concat(taicpu.op_reg_const_reg(A_ADD,dst.base,4,dst.base));
                list.concat(taicpu.op_reg_const_reg(A_SUBcc,countreg,1,countreg));
                a_jmp_cond(list,OC_NE,lab);
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


    procedure tcgsparc.g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        src, dst: TReference;
        tmpreg1,
        countreg: TRegister;
        i : aint;
        lab: tasmlabel;
      begin
        if len>31 then
          g_concatcopy_move(list,source,dest,len)
        else
          begin
            reference_reset(src,source.alignment);
            reference_reset(dst,dest.alignment);
            { load the address of source into src.base }
            src.base:=GetAddressRegister(list);
            a_loadaddr_ref_reg(list,source,src.base);
            { load the address of dest into dst.base }
            dst.base:=GetAddressRegister(list);
            a_loadaddr_ref_reg(list,dest,dst.base);
            { generate a loop }
            if len>4 then
              begin
                countreg:=GetIntRegister(list,OS_INT);
                tmpreg1:=GetIntRegister(list,OS_INT);
                a_load_const_reg(list,OS_INT,len,countreg);
                current_asmdata.getjumplabel(lab);
                a_label(list, lab);
                list.concat(taicpu.op_ref_reg(A_LDUB,src,tmpreg1));
                list.concat(taicpu.op_reg_ref(A_STB,tmpreg1,dst));
                list.concat(taicpu.op_reg_const_reg(A_ADD,src.base,1,src.base));
                list.concat(taicpu.op_reg_const_reg(A_ADD,dst.base,1,dst.base));
                list.concat(taicpu.op_reg_const_reg(A_SUBcc,countreg,1,countreg));
                a_jmp_cond(list,OC_NE,lab);
              end
            else
              begin
                { unrolled loop }
                tmpreg1:=GetIntRegister(list,OS_INT);
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


    procedure tcgsparc.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
      begin
<<<<<<< HEAD
        { This method is integrated into g_intf_wrapper and shouldn't be called separately }
        InternalError(2013020102);
=======
        if not(procdef.proctypeoption in [potype_function,potype_procedure]) then
          Internalerror(200006137);
        if not assigned(procdef.struct) or
           (procdef.procoptions*[po_classmethod, po_staticmethod,
             po_methodpointer, po_interrupt, po_iocheck]<>[]) then
          Internalerror(200006138);
        if procdef.owner.symtabletype<>ObjectSymtable then
          Internalerror(200109191);

        make_global:=false;
        if (not current_module.is_unit) or
           (procdef.owner.defowner.owner.symtabletype=globalsymtable) then
          make_global:=true;

        if make_global then
          List.concat(Tai_symbol.Createname_global(labelname,AT_FUNCTION,0))
        else
          List.concat(Tai_symbol.Createname(labelname,AT_FUNCTION,0));

        { set param1 interface to self  }
        g_adjust_self_value(list,procdef,ioffset);

        if po_virtualmethod in procdef.procoptions then
          begin
            if (procdef.extnumber=$ffff) then
              Internalerror(200006139);
            { mov  0(%rdi),%rax ; load vmt}
<<<<<<< HEAD
            reference_reset_base(href,NR_O0,0,sizeof(pint));
            cg.a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,NR_G1);
            g1_used:=true; 
            { jmp *vmtoffs(%eax) ; method offs }
            reference_reset_base(href,NR_G1,tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber),sizeof(pint));
            list.concat(taicpu.op_ref_reg(A_LD,href,NR_G1));
            list.concat(taicpu.op_reg(A_JMP,NR_G1));
	    g1_used:=false;
          end
        else
          begin
            reference_reset_symbol(href,current_asmdata.RefAsmSymbol(procdef.mangledname),0,sizeof(pint));
            href.refaddr := addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,href,NR_G1));
	    g1_used:=true;
            href.refaddr := addr_low;
            list.concat(taicpu.op_reg_ref_reg(A_OR,NR_G1,href,NR_G1));
            list.concat(taicpu.op_reg(A_JMP,NR_G1));
	    g1_used:=false;
=======
            reference_reset_base(href,NR_O0,0);
            cg.a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,NR_G1);
            { jmp *vmtoffs(%eax) ; method offs }
            reference_reset_base(href,NR_G1,procdef._class.vmtmethodoffset(procdef.extnumber));
            list.concat(taicpu.op_ref_reg(A_LD,href,NR_G1));
            list.concat(taicpu.op_reg(A_JMP,NR_G1));
          end
        else
          begin
            reference_reset_symbol(href,current_asmdata.RefAsmSymbol(procdef.mangledname),0);
            href.refaddr := addr_high;
            list.concat(taicpu.op_ref_reg(A_SETHI,href,NR_G1));
            href.refaddr := addr_low;
            list.concat(taicpu.op_reg_ref_reg(A_OR,NR_G1,href,NR_G1));
            list.concat(taicpu.op_reg(A_JMP,NR_G1));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
        { Delay slot }
        list.Concat(TAiCpu.Op_none(A_NOP));

        List.concat(Tai_symbol_end.Createname(labelname));
>>>>>>> graemeg/cpstrnew
      end;


{****************************************************************************
                               TCG64Sparc
****************************************************************************}


    procedure tcg64sparc.a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference);
      var
        tmpref: treference;
      begin
        { Override this function to prevent loading the reference twice }
        tmpref:=ref;
        cg.a_load_reg_ref(list,OS_32,OS_32,reg.reghi,tmpref);
        inc(tmpref.offset,4);
        cg.a_load_reg_ref(list,OS_32,OS_32,reg.reglo,tmpref);
      end;


    procedure tcg64sparc.a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64);
      var
        tmpref: treference;
      begin
        { Override this function to prevent loading the reference twice }
        tmpref:=ref;
        cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg.reghi);
        inc(tmpref.offset,4);
        cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg.reglo);
      end;


    procedure tcg64sparc.a_load64_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara);
      var
        hreg64 : tregister64;
      begin
        { Override this function to prevent loading the reference twice.
          Use here some extra registers, but those are optimized away by the RA }
        hreg64.reglo:=cg.GetIntRegister(list,OS_32);
        hreg64.reghi:=cg.GetIntRegister(list,OS_32);
        a_load64_ref_reg(list,r,hreg64);
        a_load64_reg_cgpara(list,hreg64,paraloc);
      end;


    procedure TCg64Sparc.get_64bit_ops(op:TOpCG;var op1,op2:TAsmOp;checkoverflow : boolean);
      begin
        case op of
          OP_ADD :
            begin
              op1:=A_ADDCC;
              if checkoverflow then
                op2:=A_ADDXCC
              else
                op2:=A_ADDX;
            end;
          OP_SUB :
            begin
              op1:=A_SUBCC;
              if checkoverflow then
                op2:=A_SUBXCC
              else
                op2:=A_SUBX;
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


    procedure TCg64Sparc.a_op64_reg_reg(list:TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst:TRegister64);
      begin
        case op of
          OP_NEG :
            begin
              { Use the simple code: y=0-z }
              list.concat(taicpu.op_reg_reg_reg(A_SUBcc,NR_G0,regsrc.reglo,regdst.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SUBX,NR_G0,regsrc.reghi,regdst.reghi));
            end;
          OP_NOT :
            begin
              list.concat(taicpu.op_reg_reg_reg(A_XNOR,regsrc.reglo,NR_G0,regdst.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_XNOR,regsrc.reghi,NR_G0,regdst.reghi));
            end;
        else
          a_op64_reg_reg_reg(list,op,size,regsrc,regdst,regdst);
        end;
      end;


    procedure TCg64Sparc.a_op64_const_reg(list:TAsmList;op:TOpCG;size : tcgsize;value:int64;regdst:TRegister64);
      begin
        a_op64_const_reg_reg(list,op,size,value,regdst,regdst);
      end;


    procedure tcg64sparc.a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64; regsrc,regdst : tregister64);
      var
        l : tlocation;
      begin
        a_op64_const_reg_reg_checkoverflow(list,op,size,value,regsrc,regdst,false,l);
      end;


    procedure tcg64sparc.a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);
      var
        l : tlocation;
      begin
        a_op64_reg_reg_reg_checkoverflow(list,op,size,regsrc1,regsrc2,regdst,false,l);
      end;


    procedure tcg64sparc.a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      var
        op1,op2:TAsmOp;
      begin
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(200306017);
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg_reg(list,op,OS_INT,tcgint(lo(value)),regsrc.reglo,regdst.reglo);
              cg.a_op_const_reg_reg(list,op,OS_INT,tcgint(hi(value)),regsrc.reghi,regdst.reghi);
            end;
        else
          get_64bit_ops(op,op1,op2,setflags);
          tcgsparc(cg).handle_reg_const_reg(list,op1,regsrc.reglo,tcgint(lo(value)),regdst.reglo);
          tcgsparc(cg).handle_reg_const_reg(list,op2,regsrc.reghi,tcgint(hi(value)),regdst.reghi);
        end;
      end;


    procedure tcg64sparc.a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      var
        op1,op2:TAsmOp;
      begin
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(200306017);
        end;
        get_64bit_ops(op,op1,op2,setflags);
        list.concat(taicpu.op_reg_reg_reg(op1,regsrc2.reglo,regsrc1.reglo,regdst.reglo));
        list.concat(taicpu.op_reg_reg_reg(op2,regsrc2.reghi,regsrc1.reghi,regdst.reghi));
      end;


    procedure create_codegen;
      begin
        cg:=TCgSparc.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if target_info.system=system_sparc_linux then
          TCgSparc(cg).use_unlimited_pic_mode:=true
        else
          TCgSparc(cg).use_unlimited_pic_mode:=false;
        cg64:=TCg64Sparc.Create;
      end;

=======
        cg64:=TCg64Sparc.Create;
      end;
      
>>>>>>> graemeg/cpstrnew
=======
        cg64:=TCg64Sparc.Create;
      end;
      
>>>>>>> graemeg/cpstrnew
=======
        cg64:=TCg64Sparc.Create;
      end;
      
>>>>>>> graemeg/cpstrnew
=======
        cg64:=TCg64Sparc.Create;
      end;
      
>>>>>>> origin/cpstrnew
end.
