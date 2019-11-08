{
    Copyright (c) 1998-2002 by the FPC team

    This unit implements the code generator for the 680x0

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
       cgbase,cgobj,globtype,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpuinfo,
       parabase,cpupara,
       node,symconst,symtype,symdef,
       cgutils,cg64f32;

    type
      tcg68k = class(tcg)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : tcgpara);override;
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : tcgpara);override;
        procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : tcgpara);override;
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : tcgpara);override;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_reg(list : TAsmList;reg : tregister);override;

        procedure a_load_const_reg(list : TAsmList;size : tcgsize;a : tcgint;register : tregister);override;
        procedure a_load_const_ref(list : TAsmList; tosize: tcgsize; a : tcgint;const ref : treference);override;

        procedure a_load_reg_ref(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);override;
        procedure a_load_reg_ref_unaligned(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);override;
        procedure a_load_reg_reg(list : TAsmList;fromsize,tosize : tcgsize;reg1,reg2 : tregister);override;
        procedure a_load_ref_reg(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);override;
        procedure a_load_ref_reg_unaligned(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);override;
        procedure a_load_ref_ref(list : TAsmList;fromsize,tosize : tcgsize;const sref : treference;const dref : treference);override;

        procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);override;
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;
        procedure a_loadfpu_reg_cgpara(list : TAsmList; size : tcgsize;const reg : tregister;const cgpara : TCGPara); override;
        procedure a_loadfpu_ref_cgpara(list : TAsmList; size : tcgsize;const ref : treference;const cgpara : TCGPara);override;

        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: tcgsize; a: tcgint; reg: TRegister); override;
        procedure a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;
        procedure a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize; reg: TRegister; const ref: TReference); override;
        procedure a_op_ref_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference; reg: TRegister); override;

        procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister; l : tasmlabel);override;
        procedure a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference; l : tasmlabel); override;
        procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel); override;
        procedure a_jmp_name(list : TAsmList;const s : string); override;
        procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;
        procedure a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel); override;
        procedure a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister); override;

        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;
        { generates overflow checking code for a node }
        procedure g_overflowcheck(list: TAsmList; const l:tlocation; def:tdef); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);override;

        procedure g_save_registers(list:TAsmList);override;
        procedure g_restore_registers(list:TAsmList);override;

        procedure g_adjust_self_value(list:TAsmList;procdef:tprocdef;ioffset:tcgint);override;

        { # Sign or zero extend the register to a full 32-bit value.
            The new value is left in the same register.
        }
        procedure sign_extend(list: TAsmList;_oldsize : tcgsize; reg: tregister);
        procedure sign_extend(list: TAsmList;_oldsize : tcgsize; _newsize : tcgsize; reg: tregister);

        procedure g_stackpointer_alloc(list : TAsmList;localsize : longint);override;
        function fixref(list: TAsmList; var ref: treference; fullyresolve: boolean): boolean;
        function force_to_dataregister(list: TAsmList; size: TCGSize; reg: TRegister): TRegister;
        procedure move_if_needed(list: TAsmList; size: TCGSize; src: TRegister; dest: TRegister);

        { optimize mul with const to a sequence of shifts and subs/adds, mainly for the '000 to '030 }
        function optimize_const_mul_to_shift_sub_add(list: TAsmList; maxops: longint; a: tcgint; size: tcgsize; reg: TRegister): boolean;
     protected
        procedure call_rtl_mul_const_reg(list:tasmlist;size:tcgsize;a:tcgint;reg:tregister;const name:string);
        procedure call_rtl_mul_reg_reg(list:tasmlist;reg1,reg2:tregister;const name:string);
        procedure check_register_size(size:tcgsize;reg:tregister);
     end;

     tcg64f68k = class(tcg64f32)
       procedure a_op64_reg_reg(list : TAsmList;op:TOpCG; size: tcgsize; regsrc,regdst : tregister64);override;
       procedure a_op64_const_reg(list : TAsmList;op:TOpCG; size: tcgsize; value : int64;regdst : tregister64);override;
       procedure a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);override;
       procedure a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64;const ref : treference);override;
       procedure a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference); override;
       procedure a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64); override;
     end;

     { This function returns true if the reference+offset is valid.
       Otherwise extra code must be generated to solve the reference.

       On the m68k, this verifies that the reference is valid
       (e.g : if index register is used, then the max displacement
        is 256 bytes, if only base is used, then max displacement
        is 32K
     }
     function isvalidrefoffset(const ref: treference): boolean;
     function isvalidreference(const ref: treference): boolean;

    procedure create_codegen;

  implementation

    uses
       globals,verbose,systems,cutils,
       symsym,symtable,defutil,paramgr,procinfo,
       rgobj,tgobj,rgcpu,fmodule;

{ Range check must be disabled explicitly as conversions between signed and unsigned
  32-bit values are done without explicit typecasts }
{$R-}

    const
      { opcode table lookup }
      topcg2tasmop: Array[topcg] of tasmop =
      (
       A_NONE,
       A_MOVE,
       A_ADD,
       A_AND,
       A_DIVU,
       A_DIVS,
       A_MULS,
       A_MULU,
       A_NEG,
       A_NOT,
       A_OR,
       A_ASR,
       A_LSL,
       A_LSR,
       A_SUB,
       A_EOR,
       A_ROL,
       A_ROR
      );

      { opcode with extend bits table lookup, used by 64bit cg }
      topcg2tasmopx: Array[topcg] of tasmop =
      (
       A_NONE,
       A_NONE,
       A_ADDX,
       A_NONE,
       A_NONE,
       A_NONE,
       A_NONE,
       A_NONE,
       A_NEGX,
       A_NONE,
       A_NONE,
       A_NONE,
       A_NONE,
       A_NONE,
       A_SUBX,
       A_NONE,
       A_NONE,
       A_NONE
      );

      TOpCmp2AsmCond: Array[topcmp] of TAsmCond =
      (
       C_NONE,
       C_EQ,
       C_GT,
       C_LT,
       C_GE,
       C_LE,
       C_NE,
       C_LS,
       C_CS,
       C_CC,
       C_HI
      );

     function isvalidreference(const ref: treference): boolean;
       begin
         isvalidreference:=isvalidrefoffset(ref) and

           { don't try to generate addressing with symbol and base reg and offset
             it might fail in linking stage if the symbol is more than 32k away (KB) }
           not (assigned(ref.symbol) and (ref.base <> NR_NO) and (ref.offset <> 0)) and

           { coldfire and 68000 cannot handle non-addressregs as bases }
           not ((current_settings.cputype in cpu_coldfire+[cpu_mc68000]) and
                not isaddressregister(ref.base));
       end;

     function isvalidrefoffset(const ref: treference): boolean;
      begin
         isvalidrefoffset := true;
         if ref.index <> NR_NO then
           begin
//             if ref.base <> NR_NO then
//                internalerror(2002081401);
             if (ref.offset < low(shortint)) or (ref.offset > high(shortint)) then
                isvalidrefoffset := false
           end
         else
           begin
             if (ref.offset < low(smallint)) or (ref.offset > high(smallint)) then
                isvalidrefoffset := false;
           end;
      end;


{****************************************************************************}
{                               TCG68K                                       }
{****************************************************************************}


    function use_push(const cgpara:tcgpara):boolean;
      begin
        result:=(not paramanager.use_fixed_stack) and
                assigned(cgpara.location) and
                (cgpara.location^.loc=LOC_REFERENCE) and
                (cgpara.location^.reference.index=NR_STACK_POINTER_REG);
      end;


    procedure tcg68k.init_register_allocators;
      var
        reg: TSuperRegister;
        address_regs: array of TSuperRegister;
      begin
        inherited init_register_allocators;
        address_regs:=nil;
        rg[R_INTREGISTER]:=trgcpu.create(R_INTREGISTER,R_SUBWHOLE,
          [RS_D0,RS_D1,RS_D2,RS_D3,RS_D4,RS_D5,RS_D6,RS_D7],
          first_int_imreg,[]);

        { set up the array of address registers to use }
        for reg:=RS_A0 to RS_A6 do
          begin
            { don't hardwire the frame pointer register, because it can vary between target OS }
            if (assigned(current_procinfo) and (current_procinfo.framepointer = NR_FRAME_POINTER_REG)
               and (reg = RS_FRAME_POINTER_REG))
               or ((reg = RS_PIC_OFFSET_REG) and (tf_static_reg_based in target_info.flags)) then
              continue;
            setlength(address_regs,length(address_regs)+1);
            address_regs[length(address_regs)-1]:=reg;
          end;
        rg[R_ADDRESSREGISTER]:=trgcpu.create(R_ADDRESSREGISTER,R_SUBWHOLE,
          address_regs, first_addr_imreg, []);

        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBNONE,
          [RS_FP0,RS_FP1,RS_FP2,RS_FP3,RS_FP4,RS_FP5,RS_FP6,RS_FP7],
          first_fpu_imreg,[]);
      end;


    procedure tcg68k.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_ADDRESSREGISTER].free;
        inherited done_register_allocators;
      end;


    procedure tcg68k.a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : tcgpara);
      var
        pushsize : tcgsize;
        ref : treference;
      begin
        { it's probably necessary to port this from x86 later, or provide an m68k solution (KB) }
{ TODO: FIX ME! check_register_size()}
        // check_register_size(size,r);
        if use_push(cgpara) then
          begin
            cgpara.check_simple_location;
            if tcgsize2size[cgpara.location^.size]>cgpara.alignment then
              pushsize:=cgpara.location^.size
            else
              pushsize:=int_cgsize(cgpara.alignment);

            reference_reset_base(ref, NR_STACK_POINTER_REG, 0, ctempposinvalid ,cgpara.alignment, []);
            ref.direction := dir_dec;
            list.concat(taicpu.op_reg_ref(A_MOVE,tcgsize2opsize[pushsize],makeregsize(list,r,pushsize),ref));
          end
        else
          inherited a_load_reg_cgpara(list,size,r,cgpara);
      end;


    procedure tcg68k.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : tcgpara);
      var
        pushsize : tcgsize;
        ref : treference;
      begin
        if use_push(cgpara) then
          begin
            cgpara.check_simple_location;
            if tcgsize2size[cgpara.location^.size]>cgpara.alignment then
              pushsize:=cgpara.location^.size
            else
              pushsize:=int_cgsize(cgpara.alignment);

            reference_reset_base(ref, NR_STACK_POINTER_REG, 0, ctempposinvalid, cgpara.alignment, []);
            ref.direction := dir_dec;
            a_load_const_ref(list, pushsize, a, ref);
          end
        else
          inherited a_load_const_cgpara(list,size,a,cgpara);
      end;


    procedure tcg68k.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : tcgpara);

        procedure pushdata(paraloc:pcgparalocation;ofs:tcgint);
        var
          pushsize : tcgsize;
          tmpreg   : tregister;
          href     : treference;
          ref      : treference;
        begin
          if not assigned(paraloc) then
            exit;

          if (paraloc^.loc<>LOC_REFERENCE) or
             (paraloc^.reference.index<>NR_STACK_POINTER_REG) or
             (tcgsize2size[paraloc^.size]>sizeof(tcgint)) then
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
          fixref(list,href,false);
          if tcgsize2size[paraloc^.size]>cgpara.alignment then
            pushsize:=paraloc^.size
          else
            pushsize:=int_cgsize(cgpara.alignment);

          reference_reset_base(ref, NR_STACK_POINTER_REG, 0, ctempposinvalid, tcgsize2size[pushsize], []);
          ref.direction := dir_dec;

          a_load_ref_ref(list,int_cgsize(tcgsize2size[paraloc^.size]),pushsize,href,ref);
        end;

      var
        len : tcgint;
        ofs : tcgint;
        href : treference;
      begin
        { cgpara.size=OS_NO requires a copy on the stack }
        if use_push(cgpara) then
          begin
            { Record copy? }
            if (cgpara.size in [OS_NO,OS_F64]) or (size in [OS_NO,OS_F64]) then
              begin
                //list.concat(tai_comment.create(strpnew('a_load_ref_cgpara: g_concatcopy')));
                cgpara.check_simple_location;
                len:=align(cgpara.intsize,cgpara.alignment);
                g_stackpointer_alloc(list,len);
                ofs:=0;
                if (cgpara.intsize<cgpara.alignment) then
                  ofs:=cgpara.alignment-cgpara.intsize;
                reference_reset_base(href,NR_STACK_POINTER_REG,ofs,ctempposinvalid,cgpara.alignment,[]);
                g_concatcopy(list,r,href,cgpara.intsize);
              end
            else
              begin
                if tcgsize2size[cgpara.size]<>tcgsize2size[size] then
                  internalerror(200501161);
                { We need to push the data in reverse order,
                  therefore we use a recursive algorithm }
                pushdata(cgpara.location,0);
              end
          end
        else
          inherited a_load_ref_cgpara(list,size,r,cgpara);
      end;


    procedure tcg68k.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : tcgpara);
      var
        tmpref : treference;
      begin
        { 68k always passes arguments on the stack }
        if use_push(cgpara) then
          begin
            //list.concat(tai_comment.create(strpnew('a_loadaddr_ref_cgpara: PEA')));
            cgpara.check_simple_location;
            tmpref:=r;
            fixref(list,tmpref,false);
            list.concat(taicpu.op_ref(A_PEA,S_NO,tmpref));
          end
        else
          inherited a_loadaddr_ref_cgpara(list,r,cgpara);
      end;


    function tcg68k.fixref(list: TAsmList; var ref: treference; fullyresolve: boolean): boolean;
       var
         hreg : tregister;
         href : treference;
         instr : taicpu;
       begin
         result:=false;
         hreg:=NR_NO;

         { NOTE: we don't have to fixup scaling in this function, because the memnode
           won't generate scaling on CPUs which don't support it }

         if (tf_static_reg_based in target_info.flags) and assigned(ref.symbol) and (ref.base=NR_NO) then
           fullyresolve:=true;

         { first, deal with the symbol, if we have an index or base register.
           in theory, the '020+ could deal with these, but it's better to avoid
           long displacements on most members of the 68k family anyway }
         if assigned(ref.symbol) and ((ref.base<>NR_NO) or (ref.index<>NR_NO)) then
           begin
             //list.concat(tai_comment.create(strpnew('fixref: symbol with base or index')));

             hreg:=getaddressregister(list);
             reference_reset_symbol(href,ref.symbol,ref.offset,ref.alignment,ref.volatility);
             if (tf_static_reg_based in target_info.flags) and (ref.base=NR_NO) then
               begin
                 if ref.symbol.typ in [AT_DATA,AT_DATA_FORCEINDIRECT,AT_DATA_NOINDIRECT] then
                   href.base:=NR_PIC_OFFSET_REG
                 else
                   href.base:=NR_PC;
               end;

             list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
             ref.offset:=0;
             ref.symbol:=nil;

             { if we have unused base or index, try to use it, otherwise fold the existing base,
               also handle the case where the base might be a data register. }
             if ref.base=NR_NO then
               ref.base:=hreg
             else
               if (ref.index=NR_NO) and not isintregister(ref.base) then
                 ref.index:=hreg
               else
                 begin
                   list.concat(taicpu.op_reg_reg(A_ADD,S_L,ref.base,hreg));
                   ref.base:=hreg;
                 end;

             { at this point we have base + (optional) index * scale }
           end;

         { deal with the case if our base is a dataregister }
         if (ref.base<>NR_NO) and not isaddressregister(ref.base) then
           begin

             hreg:=getaddressregister(list);
             if isaddressregister(ref.index) and (ref.scalefactor < 2) then
               begin
                 //list.concat(tai_comment.create(strpnew('fixref: base is dX, resolving with reverse regs')));

                 reference_reset_base(href,ref.index,0,ref.temppos,ref.alignment,ref.volatility);
                 href.index:=ref.base;
                 { we can fold in an 8 bit offset "for free" }
                 if isvalue8bit(ref.offset) then
                   begin
                     href.offset:=ref.offset;
                     ref.offset:=0;
                   end;
                 list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
                 ref.base:=hreg;
                 ref.index:=NR_NO;
                 result:=true;
               end
             else
               begin
                 //list.concat(tai_comment.create(strpnew('fixref: base is dX, can''t resolve with reverse regs')));

                 instr:=taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg);
                 add_move_instruction(instr);
                 list.concat(instr);
                 ref.base:=hreg;
                 result:=true;
               end;
           end;

         { deal with large offsets on non-020+ }
         if not (CPUM68K_HAS_BASEDISP in cpu_capabilities[current_settings.cputype]) then
           begin
             if ((ref.index<>NR_NO) and not isvalue8bit(ref.offset)) or
                ((ref.base<>NR_NO) and not isvalue16bit(ref.offset)) then
               begin
                 //list.concat(tai_comment.create(strpnew('fixref: handling large offsets')));
                 { if we have a temp register from above, we can just add to it }
                 if hreg=NR_NO then
                   hreg:=getaddressregister(list);

                 if isvalue16bit(ref.offset) then
                   begin
                     reference_reset_base(href,ref.base,ref.offset,ref.temppos,ref.alignment,ref.volatility);
                     list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
                   end
                 else
                   begin
                     instr:=taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg);
                     add_move_instruction(instr);
                     list.concat(instr);
                     list.concat(taicpu.op_const_reg(A_ADD,S_L,ref.offset,hreg));
                   end;
                 ref.offset:=0;
                 ref.base:=hreg;
                 result:=true;
               end;
           end;

         { fully resolve the reference to an address register, if we're told to do so
           and there's a reason to do so }
         if fullyresolve and
            ((ref.index<>NR_NO) or assigned(ref.symbol) or (ref.offset<>0)) then
           begin
             //list.concat(tai_comment.create(strpnew('fixref: fully resolve to register')));
             if hreg=NR_NO then
               hreg:=getaddressregister(list);
             if (tf_static_reg_based in target_info.flags) and (ref.base=NR_NO) then
               begin
                 if ref.symbol.typ in [AT_DATA,AT_DATA_FORCEINDIRECT,AT_DATA_NOINDIRECT] then
                   ref.base:=NR_PIC_OFFSET_REG
                 else
                   ref.base:=NR_PC;
               end;
             list.concat(taicpu.op_ref_reg(A_LEA,S_L,ref,hreg));
             ref.base:=hreg;
             ref.index:=NR_NO;
             ref.scalefactor:=1;
             ref.symbol:=nil;
             ref.offset:=0;
             result:=true;
           end;
       end;


    procedure tcg68k.call_rtl_mul_const_reg(list:tasmlist;size:tcgsize;a:tcgint;reg:tregister;const name:string);
      var
        paraloc1,paraloc2: tcgpara;
        pd : tprocdef;
      begin
        pd:=search_system_proc(name);
        paraloc1.init;
        paraloc2.init;
        paramanager.getintparaloc(list,pd,1,paraloc1);
        paramanager.getintparaloc(list,pd,2,paraloc2);
        a_load_const_cgpara(list,size,a,paraloc2);
        a_load_reg_cgpara(list,OS_32,reg,paraloc1);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);

        g_call(list,name);

        cg.a_reg_alloc(list,NR_FUNCTION_RESULT_REG);
        cg.a_load_reg_reg(list,OS_32,OS_32,NR_FUNCTION_RESULT_REG,reg);
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcg68k.call_rtl_mul_reg_reg(list:tasmlist;reg1,reg2:tregister;const name:string);
      var
        paraloc1,paraloc2: tcgpara;
        pd : tprocdef;
      begin
        pd:=search_system_proc(name);
        paraloc1.init;
        paraloc2.init;
        paramanager.getintparaloc(list,pd,1,paraloc1);
        paramanager.getintparaloc(list,pd,2,paraloc2);
        a_load_reg_cgpara(list,OS_32,reg1,paraloc2);
        a_load_reg_cgpara(list,OS_32,reg2,paraloc1);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);

        g_call(list,name);

        cg.a_reg_alloc(list,NR_FUNCTION_RESULT_REG);
        cg.a_load_reg_reg(list,OS_32,OS_32,NR_FUNCTION_RESULT_REG,reg2);
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcg68k.a_call_name(list : TAsmList;const s : string; weak: boolean);
      var
        sym: tasmsymbol;
      const
        jmp_inst: array[boolean] of tasmop = ( A_JSR, A_BSR );
      begin
        if not(weak) then
          sym:=current_asmdata.RefAsmSymbol(s,AT_FUNCTION)
        else
          sym:=current_asmdata.WeakRefAsmSymbol(s,AT_FUNCTION);

        list.concat(taicpu.op_sym(jmp_inst[tf_code_small in target_info.flags],S_NO,sym));
      end;


    procedure tcg68k.a_call_reg(list : TAsmList;reg: tregister);
      var
        tmpref : treference;
        tmpreg : tregister;
        instr : taicpu;
      begin
        if isaddressregister(reg) then
          begin
            { if we have an address register, we can jump to the address directly }
            reference_reset_base(tmpref,reg,0,ctempposinvalid,4,[]);
          end
        else
          begin
            { if we have a data register, we need to move it to an address register first }
            tmpreg:=getaddressregister(list);
            reference_reset_base(tmpref,tmpreg,0,ctempposinvalid,4,[]);
            instr:=taicpu.op_reg_reg(A_MOVE,S_L,reg,tmpreg);
            add_move_instruction(instr);
            list.concat(instr);
          end;
        list.concat(taicpu.op_ref(A_JSR,S_NO,tmpref));
     end;


    procedure tcg68k.a_load_const_reg(list : TAsmList;size : tcgsize;a : tcgint;register : tregister);
      var
        opsize: topsize;
      begin
        opsize:=tcgsize2opsize[size];

        if isaddressregister(register) then
          begin
            { an m68k manual I have recommends SUB Ax,Ax to be used instead of CLR for address regs }
            { Premature optimization is the root of all evil - this code breaks spilling if the
              register contains a spilled regvar, eg. a Pointer which is set to nil, then random
              havoc happens... This is kept here for reference now, to allow fixing of the spilling
              later. Most of the optimizations below here could be moved to the optimizer. (KB) }
            {if a = 0 then
              list.concat(taicpu.op_reg_reg(A_SUB,S_L,register,register))
            else}
              { ISA B/C Coldfire has MOV3Q which can move -1 or 1..7 to any reg }
              if (current_settings.cputype in [cpu_isa_b,cpu_isa_c,cpu_cfv4e]) and 
                 ((longint(a) = -1) or ((longint(a) > 0) and (longint(a) < 8))) then
                list.concat(taicpu.op_const_reg(A_MOV3Q,S_L,longint(a),register))
              else
                { MOVEA.W will sign extend the value in the dest. reg to full 32 bits 
                  (specific to Ax regs only) }
                if isvalue16bit(a) then
                  list.concat(taicpu.op_const_reg(A_MOVEA,S_W,longint(a),register))
                else
                  list.concat(taicpu.op_const_reg(A_MOVEA,S_L,longint(a),register));
          end
        else
        if a = 0 then
           list.concat(taicpu.op_reg(A_CLR,S_L,register))
        else
         begin
           { Prefer MOV3Q if applicable, it allows replacement spilling for register }
           if (current_settings.cputype in [cpu_isa_b,cpu_isa_c,cpu_cfv4e]) and
             ((longint(a)=-1) or ((longint(a)>0) and (longint(a)<8))) then
             list.concat(taicpu.op_const_reg(A_MOV3Q,S_L,longint(a),register))
           else if (longint(a) >= low(shortint)) and (longint(a) <= high(shortint)) then
              list.concat(taicpu.op_const_reg(A_MOVEQ,S_L,longint(a),register))
           else
             begin
               { ISA B/C Coldfire has sign extend/zero extend moves }
               if (current_settings.cputype in [cpu_isa_b,cpu_isa_c,cpu_cfv4e]) and 
                  (size in [OS_16, OS_8, OS_S16, OS_S8]) and 
                  ((longint(a) >= low(smallint)) and (longint(a) <= high(smallint))) then
                 begin
                   if size in [OS_16, OS_8] then
                     list.concat(taicpu.op_const_reg(A_MVZ,opsize,longint(a),register))
                   else
                     list.concat(taicpu.op_const_reg(A_MVS,opsize,longint(a),register));
                 end
               else
                 begin
                   { clear the register first, for unsigned and positive values, so
                     we don't need to zero extend after }
                   if (size in [OS_16,OS_8]) or
                      ((size in [OS_S16,OS_S8]) and (a > 0)) then
                     list.concat(taicpu.op_reg(A_CLR,S_L,register));
                   list.concat(taicpu.op_const_reg(A_MOVE,opsize,longint(a),register));
                   { only sign extend if we need to, zero extension is not necessary because the CLR.L above }
                   if (size in [OS_S16,OS_S8]) and (a < 0) then
                     sign_extend(list,size,register);
                 end;
             end;
         end;
      end;

    procedure tcg68k.a_load_const_ref(list : TAsmList; tosize: tcgsize; a : tcgint;const ref : treference);
      var
        hreg : tregister;
        href : treference;
      begin
        if needs_unaligned(ref.alignment,tosize) then
          begin
            inherited;
            exit;
          end;

        a:=longint(a);
        href:=ref;
        fixref(list,href,false);
        if (a=0) and not (current_settings.cputype = cpu_mc68000) then
          list.concat(taicpu.op_ref(A_CLR,tcgsize2opsize[tosize],href))
        else if (tcgsize2opsize[tosize]=S_L) and
           (current_settings.cputype in [cpu_isa_b,cpu_isa_c,cpu_cfv4e]) and
           ((a=-1) or ((a>0) and (a<8))) then
          list.concat(taicpu.op_const_ref(A_MOV3Q,S_L,a,href))
        { for coldfire we need to go through a temporary register if we have a
          offset, index or symbol given }
        else if (current_settings.cputype in cpu_coldfire) and
            (
              (href.offset<>0) or
              { TODO : check whether we really need this second condition }
              (href.index<>NR_NO) or
              assigned(href.symbol)
            ) then
          begin
            hreg:=getintregister(list,tosize);
            a_load_const_reg(list,tosize,a,hreg);
            list.concat(taicpu.op_reg_ref(A_MOVE,tcgsize2opsize[tosize],hreg,href));
          end
        else
          { loading via a register is almost always faster if the value is small.
            (with the 68040 being the only notable exception, so maybe disable
            this on a '040? but the difference is minor) it also results in shorter
            code. (KB) }
          if isvalue8bit(a) and (tcgsize2opsize[tosize] = S_L) then
            begin
              hreg:=getintregister(list,OS_INT);
              a_load_const_reg(list,OS_INT,a,hreg); // this will use moveq et.al.
              list.concat(taicpu.op_reg_ref(A_MOVE,tcgsize2opsize[tosize],hreg,href));
            end
          else
            list.concat(taicpu.op_const_ref(A_MOVE,tcgsize2opsize[tosize],longint(a),href));
      end;


    procedure tcg68k.a_load_reg_ref(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);
      var
        href : treference;
        hreg : tregister;
      begin
        if needs_unaligned(ref.alignment,tosize) then
          begin
            //list.concat(tai_comment.create(strpnew('a_load_reg_ref calling unaligned')));
            a_load_reg_ref_unaligned(list,fromsize,tosize,register,ref);
            exit;
          end;

        href := ref;
        hreg := register;
        fixref(list,href,false);
        if tcgsize2size[fromsize]<tcgsize2size[tosize] then
          begin
            hreg:=getintregister(list,tosize);
            a_load_reg_reg(list,fromsize,tosize,register,hreg);
          end;
        { move to destination reference }
        list.concat(taicpu.op_reg_ref(A_MOVE,TCGSize2OpSize[tosize],hreg,href));
      end;


    procedure tcg68k.a_load_reg_ref_unaligned(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);
      var
        tmpref : treference;
        tmpreg,
        tmpreg2 : tregister;
      begin
        if not needs_unaligned(ref.alignment,tosize) then
          begin
            a_load_reg_ref(list,fromsize,tosize,register,ref);
            exit;
          end;

        list.concat(tai_comment.create(strpnew('a_load_reg_ref_unaligned: generating unaligned store')));

        tmpreg2:=getaddressregister(list);
        tmpref:=ref;
        inc(tmpref.offset,tcgsize2size[tosize]-1);
        a_loadaddr_ref_reg(list,tmpref,tmpreg2);
        reference_reset_base(tmpref,tmpreg2,0,ctempposinvalid,1,ref.volatility);
        tmpref.direction:=dir_none;

        tmpreg:=getintregister(list,tosize);
        a_load_reg_reg(list,fromsize,tosize,register,tmpreg);

        case tosize of
          OS_16,OS_S16:
            begin
              list.concat(taicpu.op_reg_ref(A_MOVE,S_B,tmpreg,tmpref));
              list.concat(taicpu.op_const_reg(A_LSR,S_W,8,tmpreg));
              tmpref.direction:=dir_dec;
              list.concat(taicpu.op_reg_ref(A_MOVE,S_B,tmpreg,tmpref));
            end;
          OS_32,OS_S32:
            begin
              list.concat(taicpu.op_reg_ref(A_MOVE,S_B,tmpreg,tmpref));
              list.concat(taicpu.op_const_reg(A_LSR,S_W,8,tmpreg));
              tmpref.direction:=dir_dec;
              list.concat(taicpu.op_reg_ref(A_MOVE,S_B,tmpreg,tmpref));
              list.concat(taicpu.op_reg(A_SWAP,S_L,tmpreg));
              list.concat(taicpu.op_reg_ref(A_MOVE,S_B,tmpreg,tmpref));
              list.concat(taicpu.op_const_reg(A_LSR,S_W,8,tmpreg));
              list.concat(taicpu.op_reg_ref(A_MOVE,S_B,tmpreg,tmpref));
            end
          else
            internalerror(2016052201);
        end;
      end;


    procedure tcg68k.a_load_ref_ref(list : TAsmList;fromsize,tosize : tcgsize;const sref : treference;const dref : treference);
      var
        aref: treference;
        bref: treference;
        usetemp: boolean;
        hreg: TRegister;
      begin
        usetemp:=TCGSize2OpSize[fromsize]<>TCGSize2OpSize[tosize];
        usetemp:=usetemp or (needs_unaligned(sref.alignment,fromsize) or needs_unaligned(dref.alignment,tosize));

        aref := sref;
        bref := dref;

        if usetemp then
          begin
            { if we need to change the size then always use a temporary register }
            hreg:=getintregister(list,fromsize);

            if needs_unaligned(sref.alignment,fromsize) then
              a_load_ref_reg_unaligned(list,fromsize,tosize,sref,hreg)
            else
              begin
                fixref(list,aref,false);
                list.concat(taicpu.op_ref_reg(A_MOVE,TCGSize2OpSize[fromsize],aref,hreg));
                sign_extend(list,fromsize,tosize,hreg);
              end;

            if needs_unaligned(dref.alignment,tosize) then
              a_load_reg_ref_unaligned(list,tosize,tosize,hreg,dref)
            else
              begin
                { if we use a temp register, we don't need to fully resolve 
                  the dest ref, not even on coldfire }
                fixref(list,bref,false);
                list.concat(taicpu.op_reg_ref(A_MOVE,TCGSize2OpSize[tosize],hreg,bref));
              end;
          end
        else
          begin
            fixref(list,aref,false);
            fixref(list,bref,current_settings.cputype in cpu_coldfire);
            list.concat(taicpu.op_ref_ref(A_MOVE,TCGSize2OpSize[fromsize],aref,bref));
          end;
      end;


    procedure tcg68k.a_load_reg_reg(list : TAsmList;fromsize,tosize : tcgsize;reg1,reg2 : tregister);
      var
        instr : taicpu;
        hreg : tregister;
        opsize : topsize;
      begin
        { move to destination register }
        opsize:=TCGSize2OpSize[fromsize];
        if isaddressregister(reg2) and not (opsize in [S_L]) then
          begin
            hreg:=cg.getintregister(list,OS_ADDR);
            instr:=taicpu.op_reg_reg(A_MOVE,TCGSize2OpSize[fromsize],reg1,hreg);
            add_move_instruction(instr);
            list.concat(instr);
            sign_extend(list,fromsize,hreg);
            list.concat(taicpu.op_reg_reg(A_MOVE,S_L,hreg,reg2));
          end
        else
          begin
            if not isregoverlap(reg1,reg2) then
              begin
                instr:=taicpu.op_reg_reg(A_MOVE,opsize,reg1,reg2);
                add_move_instruction(instr);
                list.concat(instr);
              end;
            sign_extend(list,fromsize,tosize,reg2);
          end;
      end;


    procedure tcg68k.a_load_ref_reg(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);
      var
       href : treference;
       hreg : tregister;
       size : tcgsize;
       opsize: topsize;
       needsext: boolean;
      begin
         if needs_unaligned(ref.alignment,fromsize) then
           begin
             //list.concat(tai_comment.create(strpnew('a_load_ref_reg calling unaligned')));
             a_load_ref_reg_unaligned(list,fromsize,tosize,ref,register);
             exit;
           end;

         href:=ref;
         fixref(list,href,false);

         needsext:=tcgsize2size[fromsize]<tcgsize2size[tosize];
         if needsext then
           size:=fromsize
         else
           size:=tosize;
         opsize:=TCGSize2OpSize[size];
         if isaddressregister(register) and not (opsize in [S_L]) then
           hreg:=getintregister(list,OS_ADDR)
         else
           hreg:=register;

         if needsext and (CPUM68K_HAS_MVSMVZ in cpu_capabilities[current_settings.cputype]) and not (opsize in [S_L]) then
           begin
             if fromsize in [OS_S8,OS_S16] then
               list.concat(taicpu.op_ref_reg(A_MVS,opsize,href,hreg))
             else if fromsize in [OS_8,OS_16] then
               list.concat(taicpu.op_ref_reg(A_MVZ,opsize,href,hreg))
             else
               internalerror(2016050502);
           end
         else
           begin
             if needsext and (fromsize in [OS_8,OS_16]) then
               begin
                 //list.concat(tai_comment.create(strpnew('a_load_ref_reg: zero ext')));
                 a_load_const_reg(list,OS_32,0,hreg);
                 needsext:=false;
               end;
             list.concat(taicpu.op_ref_reg(A_MOVE,opsize,href,hreg));
             if needsext then
               sign_extend(list,size,hreg);
           end;

         if hreg<>register then
           a_load_reg_reg(list,OS_ADDR,OS_ADDR,hreg,register);
      end;


    procedure tcg68k.a_load_ref_reg_unaligned(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);
      var
        tmpref : treference;
        tmpreg,
        tmpreg2 : tregister;
      begin
        if not needs_unaligned(ref.alignment,fromsize) then
          begin
            a_load_ref_reg(list,fromsize,tosize,ref,register);
            exit;
          end;

        list.concat(tai_comment.create(strpnew('a_load_ref_reg_unaligned: generating unaligned load')));

        tmpreg2:=getaddressregister(list);
        a_loadaddr_ref_reg(list,ref,tmpreg2);
        reference_reset_base(tmpref,tmpreg2,0,ctempposinvalid,1,ref.volatility);
        tmpref.direction:=dir_inc;

        if isaddressregister(register) then
          tmpreg:=getintregister(list,OS_ADDR)
        else
          tmpreg:=register;

        case fromsize of
          OS_16,OS_S16:
            begin
              list.concat(taicpu.op_ref_reg(A_MOVE,S_B,tmpref,tmpreg));
              list.concat(taicpu.op_const_reg(A_LSL,S_W,8,tmpreg));
              tmpref.direction:=dir_none;
              list.concat(taicpu.op_ref_reg(A_MOVE,S_B,tmpref,tmpreg));
              sign_extend(list,fromsize,tmpreg);
            end;
          OS_32,OS_S32:
            begin
              list.concat(taicpu.op_ref_reg(A_MOVE,S_B,tmpref,tmpreg));
              list.concat(taicpu.op_const_reg(A_LSL,S_W,8,tmpreg));
              list.concat(taicpu.op_ref_reg(A_MOVE,S_B,tmpref,tmpreg));
              list.concat(taicpu.op_reg(A_SWAP,S_L,tmpreg));
              list.concat(taicpu.op_ref_reg(A_MOVE,S_B,tmpref,tmpreg));
              list.concat(taicpu.op_const_reg(A_LSL,S_W,8,tmpreg));
              tmpref.direction:=dir_none;
              list.concat(taicpu.op_ref_reg(A_MOVE,S_B,tmpref,tmpreg));
            end
          else
            internalerror(2016052103);
        end;
        if tmpreg<>register then
          a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpreg,register);
      end;


    procedure tcg68k.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);
      var
        href : treference;
        hreg : tregister;
      begin
        href:=ref;
        fixref(list, href, false);
        if not isaddressregister(r) then
          begin
            hreg:=getaddressregister(list);
            list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
            a_load_reg_reg(list, OS_ADDR, OS_ADDR, hreg, r);
          end
        else
          list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,r));
      end;


    procedure tcg68k.a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);
      var
        instr : taicpu;
      begin
        instr:=taicpu.op_reg_reg(A_FMOVE,fpuregopsize,reg1,reg2);
        add_move_instruction(instr);
        list.concat(instr);
      end;


    procedure tcg68k.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);
      var
        opsize : topsize;
        href : treference;
      begin
        opsize := tcgsize2opsize[fromsize];
        href := ref;
        fixref(list,href,current_settings.fputype = fpu_coldfire);
        list.concat(taicpu.op_ref_reg(A_FMOVE,opsize,href,reg));
      end;

    procedure tcg68k.a_loadfpu_reg_ref(list: TAsmList; fromsize,tosize: tcgsize; reg: tregister; const ref: treference);
      var
        opsize : topsize;
        href : treference;
      begin
        opsize := tcgsize2opsize[tosize];
        href := ref;
        fixref(list,href,current_settings.fputype = fpu_coldfire);
        list.concat(taicpu.op_reg_ref(A_FMOVE,opsize,reg,href));
      end;

    procedure tcg68k.a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const reg : tregister;const cgpara : tcgpara);
      var
        ref : treference;
      begin
        if use_push(cgpara) and (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) then
          begin
            cgpara.check_simple_location;
            reference_reset_base(ref, NR_STACK_POINTER_REG, 0, ctempposinvalid, cgpara.alignment, []);
            ref.direction := dir_dec;
            list.concat(taicpu.op_reg_ref(A_FMOVE,tcgsize2opsize[cgpara.location^.size],reg,ref));
          end
        else
          inherited a_loadfpu_reg_cgpara(list,size,reg,cgpara);
      end;

    procedure tcg68k.a_loadfpu_ref_cgpara(list : TAsmList; size : tcgsize;const ref : treference;const cgpara : TCGPara);
      var
        href, href2 : treference;
        freg : tregister;
      begin
        if current_settings.fputype = fpu_soft then
          case cgpara.location^.loc of
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                case size of
                  OS_F64:
                    cg64.a_load64_ref_cgpara(list,ref,cgpara);
                  OS_F32:
                    a_load_ref_cgpara(list,size,ref,cgpara);
                  else
                    internalerror(2013021201);
                end;
              end;
            else
              inherited a_loadfpu_ref_cgpara(list,size,ref,cgpara);
          end
        else
          if use_push(cgpara) and (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) then
            begin
              //list.concat(tai_comment.create(strpnew('a_loadfpu_ref_cgpara copy')));
              cgpara.check_simple_location;
              reference_reset_base(href, NR_STACK_POINTER_REG, 0, ctempposinvalid, cgpara.alignment, []);
              href.direction := dir_dec;
              case size of
                OS_F64:
                  begin
                    href2:=ref;
                    inc(href2.offset,8);
                    fixref(list,href2,true);
                    href2.direction := dir_dec;
                    cg.a_load_ref_ref(list,OS_32,OS_32,href2,href);
                    cg.a_load_ref_ref(list,OS_32,OS_32,href2,href);
                  end;
                OS_F32:
                  cg.a_load_ref_ref(list,OS_32,OS_32,ref,href);
                else
                  internalerror(2017052110);
              end;
            end
          else
            begin
              //list.concat(tai_comment.create(strpnew('a_loadfpu_ref_cgpara inherited')));
              inherited a_loadfpu_ref_cgpara(list,size,ref,cgpara);
            end;
      end;


    procedure tcg68k.a_op_const_reg(list : TAsmList; Op: TOpCG; size: tcgsize; a: tcgint; reg: TRegister);
      var
       scratch_reg : tregister;
       scratch_reg2: tregister;
       opcode : tasmop;
      begin
        optimize_op_const(size, op, a);
        opcode := topcg2tasmop[op];
        if (a >0) and (a<=high(dword)) then
          a:=longint(dword(a))
        else if (a>=low(longint)) then
          a:=longint(a)
        else
	  internalerror(201810201);
          
        case op of
          OP_NONE :
            begin
              { Opcode is optimized away }
            end;
          OP_MOVE :
            begin
              { Optimized, replaced with a simple load }
              a_load_const_reg(list,size,a,reg);
            end;
          OP_ADD,
          OP_SUB:
              begin
                { add/sub works the same way, so have it unified here }
                if (a >= 1) and (a <= 8) then
                  if (op = OP_ADD) then
                    opcode:=A_ADDQ
                  else
                    opcode:=A_SUBQ;
                list.concat(taicpu.op_const_reg(opcode, S_L, a, reg));
              end;
          OP_AND,
          OP_OR,
          OP_XOR:
              begin
                scratch_reg := force_to_dataregister(list, size, reg);
                list.concat(taicpu.op_const_reg(opcode, S_L, a, scratch_reg));
                move_if_needed(list, size, scratch_reg, reg);
              end;
          OP_DIV,
          OP_IDIV:
              begin
                 internalerror(20020816);
              end;
          OP_MUL,
          OP_IMUL:
              begin
                { NOTE: better have this as fast as possible on every CPU in all cases,
                        because the compiler uses OP_IMUL for array indexing... (KB) }
                { ColdFire doesn't support MULS/MULU <imm>,dX }
                if not (CPUM68K_HAS_MULIMM in cpu_capabilities[current_settings.cputype]) then
                  begin
                    { move const to a register first }
                    scratch_reg := getintregister(list,OS_INT);
                    a_load_const_reg(list, size, a, scratch_reg);

                    { do the multiplication }
                    scratch_reg2 := force_to_dataregister(list, size, reg);
                    sign_extend(list, size, scratch_reg2);
                    list.concat(taicpu.op_reg_reg(opcode,S_L,scratch_reg,scratch_reg2));

                    { move the value back to the original register }
                    move_if_needed(list, size, scratch_reg2, reg);
                  end
                else
                  begin
                    if CPUM68K_HAS_32BITMUL in cpu_capabilities[current_settings.cputype] then
                      begin
                        { do the multiplication }
                        scratch_reg := force_to_dataregister(list, size, reg);
                        sign_extend(list, size, scratch_reg);
                        list.concat(taicpu.op_const_reg(opcode,S_L,a,scratch_reg));

                        { move the value back to the original register }
                        move_if_needed(list, size, scratch_reg, reg);
                      end
                    else
                      { Fallback branch, plain 68000 for now }
                      if not optimize_const_mul_to_shift_sub_add(list, 5, a, size, reg) then
                        { FIX ME: this is slow as hell, but original 68000 doesn't have 32x32 -> 32bit MUL (KB) }
                        if op = OP_MUL then
                          call_rtl_mul_const_reg(list, size, a, reg,'fpc_mul_dword')
                        else
                          call_rtl_mul_const_reg(list, size, a, reg,'fpc_mul_longint');
                  end;
              end;
          OP_ROL,
          OP_ROR,
          OP_SAR,
          OP_SHL,
          OP_SHR :
              begin
                scratch_reg := force_to_dataregister(list, size, reg);
                sign_extend(list, size, scratch_reg);

                { some special cases which can generate smarter code 
                  using the SWAP instruction }
                if (a = 16) then
                  begin
                    if (op = OP_SHL) then
                      begin
                        list.concat(taicpu.op_reg(A_SWAP,S_NO,scratch_reg));
                        list.concat(taicpu.op_reg(A_CLR,S_W,scratch_reg));
                      end
                    else if (op = OP_SHR) then
                      begin
                        list.concat(taicpu.op_reg(A_CLR,S_W,scratch_reg));
                        list.concat(taicpu.op_reg(A_SWAP,S_NO,scratch_reg));
                      end
                    else if (op = OP_SAR) then
                      begin
                        list.concat(taicpu.op_reg(A_SWAP,S_NO,scratch_reg));
                        list.concat(taicpu.op_reg(A_EXT,S_L,scratch_reg));
                      end
                    else if (op = OP_ROR) or (op = OP_ROL) then
                      list.concat(taicpu.op_reg(A_SWAP,S_NO,scratch_reg))
                  end
                else if (a >= 1) and (a <= 8) then
                  begin
                    list.concat(taicpu.op_const_reg(opcode, S_L, a, scratch_reg));
                  end
                else if (a >= 9) and (a < 16) then
                  begin
                    { Use two ops instead of const -> reg + shift with reg, because
                      this way is the same in length and speed but has less register
                      pressure }
                    list.concat(taicpu.op_const_reg(opcode, S_L, 8, scratch_reg));
                    list.concat(taicpu.op_const_reg(opcode, S_L, a-8, scratch_reg));
                  end
                else
                  begin
                    { move const to a register first }
                    scratch_reg2 := getintregister(list,OS_INT);
                    a_load_const_reg(list, size, a, scratch_reg2);

                    { do the operation }
                    list.concat(taicpu.op_reg_reg(opcode, S_L, scratch_reg2, scratch_reg));
                  end;
                { move the value back to the original register }
                move_if_needed(list, size, scratch_reg, reg);
              end;
        else
            internalerror(20020729);
         end;
      end;


    procedure tcg68k.a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference);
      var
        opcode: tasmop;
        opsize: topsize;
        href  : treference;
        hreg  : tregister;
      begin
        optimize_op_const(size, op, a);
        opcode := topcg2tasmop[op];
        opsize := TCGSize2OpSize[size];

        { on ColdFire all arithmetic operations are only possible on 32bit }
        if needs_unaligned(ref.alignment,size) or
           ((current_settings.cputype in cpu_coldfire) and (opsize <> S_L)
           and not (op in [OP_NONE,OP_MOVE])) then
          begin
            inherited;
            exit;
          end;

        case op of
          OP_NONE :
            begin
              { opcode was optimized away }
            end;
          OP_MOVE :
            begin
              { Optimized, replaced with a simple load }
              a_load_const_ref(list,size,a,ref);
            end;
          OP_AND,
          OP_OR,
          OP_XOR :
            begin
              //list.concat(tai_comment.create(strpnew('a_op_const_ref: bitwise')));
              hreg:=getintregister(list,size);
              a_load_const_reg(list,size,a,hreg);
              href:=ref;
              fixref(list,href,false);
              list.concat(taicpu.op_reg_ref(opcode, opsize, hreg, href));
            end;
          OP_ADD,
          OP_SUB :
            begin
              href:=ref;
              { add/sub works the same way, so have it unified here }
              if (a >= 1) and (a <= 8) then
                begin
                  fixref(list,href,false);
                  if (op = OP_ADD) then
                    opcode:=A_ADDQ
                  else
                    opcode:=A_SUBQ;
                  list.concat(taicpu.op_const_ref(opcode, opsize, a, href));
                end
              else
                if not(current_settings.cputype in cpu_coldfire) then
                  begin
                    fixref(list,href,false);
                    list.concat(taicpu.op_const_ref(opcode, opsize, a, href));
                  end
                else
                  { on ColdFire, ADDI/SUBI cannot act on memory
                    so we can only go through a register }
                  inherited;
            end;
          else begin
//            list.concat(tai_comment.create(strpnew('a_op_const_ref inherited')));
            inherited;
          end;
        end;
      end;

    procedure tcg68k.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
      var
        hreg1, hreg2: tregister;
        opcode : tasmop;
        opsize : topsize;
      begin
        opcode := topcg2tasmop[op];
        if current_settings.cputype in cpu_coldfire then
          opsize := S_L
        else
          opsize := TCGSize2OpSize[size];

        case op of
          OP_ADD,
          OP_SUB:
              begin
                if current_settings.cputype in cpu_coldfire then
                  begin
                    { operation only allowed only a longword }
                    sign_extend(list, size, src);
                    sign_extend(list, size, dst);
                  end;
                list.concat(taicpu.op_reg_reg(opcode, opsize, src, dst));
              end;
          OP_AND,OP_OR,
          OP_SAR,OP_SHL,
          OP_SHR,OP_XOR:
              begin
                { load to data registers }
                hreg1 := force_to_dataregister(list, size, src);
                hreg2 := force_to_dataregister(list, size, dst);

                if current_settings.cputype in cpu_coldfire then
                  begin
                    { operation only allowed only a longword }
                    {!***************************************
                      in the case of shifts, the value to
                      shift by, should already be valid, so
                      no need to sign extend the value
                     !
                    }
                    if op in [OP_AND,OP_OR,OP_XOR] then
                      sign_extend(list, size, hreg1);
                    sign_extend(list, size, hreg2);
                  end;
                list.concat(taicpu.op_reg_reg(opcode, opsize, hreg1, hreg2));

                { move back result into destination register }
                move_if_needed(list, size, hreg2, dst);
              end;
          OP_DIV,
          OP_IDIV :
              begin
                internalerror(20020816);
              end;
          OP_MUL,
          OP_IMUL:
              begin
                if not (CPUM68K_HAS_32BITMUL in cpu_capabilities[current_settings.cputype]) then
                  if op = OP_MUL then
                    call_rtl_mul_reg_reg(list,src,dst,'fpc_mul_dword')
                  else
                    call_rtl_mul_reg_reg(list,src,dst,'fpc_mul_longint')
                else
                  begin
                    { 68020+ and ColdFire codepath, probably could be improved }
                    hreg1 := force_to_dataregister(list, size, src);
                    hreg2 := force_to_dataregister(list, size, dst);

                    sign_extend(list, size, hreg1);
                    sign_extend(list, size, hreg2);

                    list.concat(taicpu.op_reg_reg(opcode, opsize, hreg1, hreg2));

                    { move back result into destination register }
                    move_if_needed(list, size, hreg2, dst);
                  end;
              end;
          OP_NEG,
          OP_NOT :
              begin
                { if there are two operands, move the register,
                  since the operation will only be done on the result
                  register. }
                if (src<>dst) then
                  a_load_reg_reg(list,size,size,src,dst);

                hreg2 := force_to_dataregister(list, size, dst);

                { coldfire only supports long version }
                if current_settings.cputype in cpu_ColdFire then
                  sign_extend(list, size, hreg2);

                list.concat(taicpu.op_reg(opcode, opsize, hreg2));

                { move back the result to the result register if needed }
                move_if_needed(list, size, hreg2, dst);
              end;
        else
            internalerror(20020729);
         end;
      end;


    procedure tcg68k.a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize; reg: TRegister; const ref: TReference);
      var
        opcode : tasmop;
        opsize : topsize;
        href   : treference;
        hreg   : tregister;
      begin
        opcode := topcg2tasmop[op];
        opsize := TCGSize2OpSize[size];

        { on ColdFire all arithmetic operations are only possible on 32bit 
          and addressing modes are limited }
        if needs_unaligned(ref.alignment,size) or
           ((current_settings.cputype in cpu_coldfire) and (opsize <> S_L)) then
          begin
            //list.concat(tai_comment.create(strpnew('a_op_reg_ref: inherited #1')));
            inherited;
            exit;
          end;

        case op of
          OP_ADD,
          OP_SUB,
          OP_OR,
          OP_XOR,
          OP_AND:
            begin
              //list.concat(tai_comment.create(strpnew('a_op_reg_ref: normal op')));
              href:=ref;
              fixref(list,href,false);
              { areg -> ref arithmetic operations are impossible on 68k }
              hreg:=force_to_dataregister(list,size,reg);
              { add/sub works the same way, so have it unified here }
              list.concat(taicpu.op_reg_ref(opcode, opsize, hreg, href));
            end;
          else begin
            //list.concat(tai_comment.create(strpnew('a_op_reg_ref inherited #2')));
            inherited;
          end;
        end;
      end;


    procedure tcg68k.a_op_ref_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference; reg: TRegister);
      var
        opcode : tasmop;
        opsize : topsize;
        href   : treference;
        hreg   : tregister;
      begin
        opcode := topcg2tasmop[op];
        opsize := TCGSize2OpSize[size];

        { on ColdFire all arithmetic operations are only possible on 32bit 
          and addressing modes are limited }
        if needs_unaligned(ref.alignment,size) or
           ((current_settings.cputype in cpu_coldfire) and (opsize <> S_L)) then
          begin
            //list.concat(tai_comment.create(strpnew('a_op_ref_reg: inherited #1')));
            inherited;
            exit;
          end;

        case op of
          OP_ADD,
          OP_SUB,
          OP_OR,
          OP_AND,
          OP_MUL,
          OP_IMUL:
            begin
              //list.concat(tai_comment.create(strpnew('a_op_ref_reg: normal op')));
              href:=ref;
              { Coldfire doesn't support d(Ax,Dx) for long MULx... }
              fixref(list,href,(op in [OP_MUL,OP_IMUL]) and 
                               (current_settings.cputype in cpu_coldfire));
              list.concat(taicpu.op_ref_reg(opcode, opsize, href, reg));
            end;
          else begin
            //list.concat(tai_comment.create(strpnew('a_op_ref_reg inherited #2')));
            inherited;
          end;
        end;
      end;


    procedure tcg68k.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
            l : tasmlabel);
      var
        hregister : tregister;
        instr : taicpu;
        need_temp_reg : boolean;
        temp_size: topsize;
      begin
        need_temp_reg := false;

        { plain 68000 doesn't support address registers for TST }
        need_temp_reg := (current_settings.cputype = cpu_mc68000) and
          (a = 0) and isaddressregister(reg);

        { ColdFire doesn't support address registers for CMPI }
        need_temp_reg := need_temp_reg or ((current_settings.cputype in cpu_coldfire)
          and (a <> 0) and isaddressregister(reg));

        if need_temp_reg then
          begin
            hregister := getintregister(list,OS_INT);
            temp_size := TCGSize2OpSize[size];
            if temp_size < S_W then
              temp_size := S_W;
            instr:=taicpu.op_reg_reg(A_MOVE,temp_size,reg,hregister);
            add_move_instruction(instr);
            list.concat(instr);
            reg := hregister;

            { do sign extension if size had to be modified }
            if temp_size <> TCGSize2OpSize[size] then
              begin
                sign_extend(list, size, reg);
                size:=OS_INT;
              end;
          end;

        if a = 0 then
          list.concat(taicpu.op_reg(A_TST,TCGSize2OpSize[size],reg))
        else 
          begin
            { ColdFire ISA A also needs S_L for CMPI }
            { Note: older QEMU pukes from CMPI sizes <> .L even on ISA B/C, but
              it's actually *LEGAL*, see CFPRM, page 4-30, the bug also seems
              fixed in recent QEMU, but only when CPU cfv4e is forced, not by
              default. (KB) }
            if current_settings.cputype in cpu_coldfire{-[cpu_isa_b,cpu_isa_c,cpu_cfv4e]} then
              begin
                sign_extend(list, size, reg);
                size:=OS_INT;
              end;
            list.concat(taicpu.op_const_reg(A_CMPI,TCGSize2OpSize[size],a,reg));
          end;

         { emit the actual jump to the label }
         a_jmp_cond(list,cmp_op,l);
      end;

    procedure tcg68k.a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference; l : tasmlabel);
      var
        tmpref: treference;
      begin
        { optimize for usage of TST here, so ref compares against zero, which is the 
          most common case by far in the RTL code at least (KB) }
        if not needs_unaligned(ref.alignment,size) and (a = 0) then
          begin
            //list.concat(tai_comment.create(strpnew('a_cmp_const_ref_label with TST')));
            tmpref:=ref;
            fixref(list,tmpref,false);
            list.concat(taicpu.op_ref(A_TST,tcgsize2opsize[size],tmpref));
            a_jmp_cond(list,cmp_op,l);
          end
        else
          begin
            //list.concat(tai_comment.create(strpnew('a_cmp_const_ref_label inherited')));
            inherited;
          end;
      end;

    procedure tcg68k.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel);
      begin
         if (current_settings.cputype in cpu_coldfire-[cpu_isa_b,cpu_isa_c,cpu_cfv4e]) then
           begin
             sign_extend(list,size,reg1);
             sign_extend(list,size,reg2);
             size:=OS_INT;
           end;

         list.concat(taicpu.op_reg_reg(A_CMP,tcgsize2opsize[size],reg1,reg2));
         { emit the actual jump to the label }
         a_jmp_cond(list,cmp_op,l);
      end;

    procedure tcg68k.a_jmp_name(list: TAsmList; const s: string);
      var
       ai: taicpu;
      begin
         ai := Taicpu.op_sym(A_JMP,S_NO,current_asmdata.RefAsmSymbol(s,AT_FUNCTION));
         ai.is_jmp := true;
         list.concat(ai);
      end;

    procedure tcg68k.a_jmp_always(list : TAsmList;l: tasmlabel);
      var
       ai: taicpu;
      begin
         ai := Taicpu.op_sym(A_JMP,S_NO,l);
         ai.is_jmp := true;
         list.concat(ai);
      end;

    procedure tcg68k.a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel);
       var
         ai : taicpu;
       begin
         if not (f in FloatResFlags) then
           ai := Taicpu.op_sym(A_BXX,S_NO,l)
         else
           ai := Taicpu.op_sym(A_FBXX,S_NO,l);
         ai.SetCondition(flags_to_cond(f));
         ai.is_jmp := true;
         list.concat(ai);
       end;

    procedure tcg68k.g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister);
       var
         ai : taicpu;
         htrue: tasmlabel;
       begin
          if isaddressregister(reg) then
            internalerror(2017051701);

          if (f in FloatResFlags) then
            begin
              //list.concat(tai_comment.create(strpnew('flags2reg: float resflags')));
              current_asmdata.getjumplabel(htrue);
              a_load_const_reg(current_asmdata.CurrAsmList,OS_32,1,reg);
              a_jmp_flags(list, f, htrue);
              a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,reg);
              a_label(current_asmdata.CurrAsmList,htrue);
              exit;
            end;

          ai:=Taicpu.Op_reg(A_Sxx,S_B,reg);
          ai.SetCondition(flags_to_cond(f));
          list.concat(ai);

          { Scc stores a complete byte of 1s, but the compiler expects only one
            bit set, so ensure this is the case }
          if not (current_settings.cputype in cpu_coldfire) then
            begin
              if size in [OS_S8,OS_8] then
                list.concat(taicpu.op_reg(A_NEG,S_B,reg))
              else
                list.concat(taicpu.op_const_reg(A_AND,TCgSize2OpSize[size],1,reg));
            end
          else
            list.concat(taicpu.op_const_reg(A_AND,S_L,1,reg));
       end;



    procedure tcg68k.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);
     const
       lentocgsize: array[1..4] of tcgsize = (OS_8,OS_16,OS_NO,OS_32);
     var
         helpsize : longint;
         i : byte;
         hregister : tregister;
         iregister : tregister;
         jregister : tregister;
         hl : tasmlabel;
         srcrefp,dstrefp : treference;
         srcref,dstref : treference;
      begin
         if (len = 1) or ((len in [2,4]) and (current_settings.cputype <> cpu_mc68000)) then
           begin
             //list.concat(tai_comment.create(strpnew('g_concatcopy: small')));
             a_load_ref_ref(list,lentocgsize[len],lentocgsize[len],source,dest);
             exit;
           end;

         //list.concat(tai_comment.create(strpnew('g_concatcopy')));
         hregister := getintregister(list,OS_INT);

         iregister:=getaddressregister(list);
         reference_reset_base(srcref,iregister,0,source.temppos,source.alignment,source.volatility);
         srcrefp:=srcref;
         srcrefp.direction := dir_inc;

         jregister:=getaddressregister(list);
         reference_reset_base(dstref,jregister,0,dest.temppos,dest.alignment,dest.volatility);
         dstrefp:=dstref;
         dstrefp.direction := dir_inc;

         { iregister = source }
         { jregister = destination }

         a_loadaddr_ref_reg(list,source,iregister);
         a_loadaddr_ref_reg(list,dest,jregister);

         if not (needs_unaligned(source.alignment,OS_INT) or needs_unaligned(dest.alignment,OS_INT)) then
           begin
             if not ((len<=8) or (not(cs_opt_size in current_settings.optimizerswitches) and (len<=16))) then
               begin
                 //list.concat(tai_comment.create(strpnew('g_concatcopy tight copy loop 020+')));
                 helpsize := len - len mod 4;
                 len := len mod 4;
                 a_load_const_reg(list,OS_INT,(helpsize div 4)-1,hregister);
                 current_asmdata.getjumplabel(hl);
                 a_label(list,hl);
                 list.concat(taicpu.op_ref_ref(A_MOVE,S_L,srcrefp,dstrefp));
                 if (current_settings.cputype in cpu_coldfire) or ((helpsize div 4)-1 > high(smallint)) then
                   begin
                     { Coldfire does not support DBRA, also it is word only }
                     list.concat(taicpu.op_const_reg(A_SUBQ,S_L,1,hregister));
                     list.concat(taicpu.op_sym(A_BPL,S_NO,hl));
                   end
                 else
                   list.concat(taicpu.op_reg_sym(A_DBRA,S_NO,hregister,hl));
               end;
             helpsize:=len div 4;
             { move a dword x times }
             for i:=1 to helpsize do
               begin
                 dec(len,4);
                 if (len > 0) then
                   list.concat(taicpu.op_ref_ref(A_MOVE,S_L,srcrefp,dstrefp))
                 else
                   list.concat(taicpu.op_ref_ref(A_MOVE,S_L,srcref,dstref));
               end;
             { move a word }
             if len>1 then
               begin
                 dec(len,2);
                 if (len > 0) then
                   list.concat(taicpu.op_ref_ref(A_MOVE,S_W,srcrefp,dstrefp))
                 else
                   list.concat(taicpu.op_ref_ref(A_MOVE,S_W,srcref,dstref));
               end;
             { move a single byte }
             if len>0 then
               list.concat(taicpu.op_ref_ref(A_MOVE,S_B,srcref,dstref));
           end
         else
           begin
             { Fast 68010 loop mode with no possible alignment problems }
             //list.concat(tai_comment.create(strpnew('g_concatcopy tight byte copy loop')));
             a_load_const_reg(list,OS_INT,len - 1,hregister);
             current_asmdata.getjumplabel(hl);
             a_label(list,hl);
             list.concat(taicpu.op_ref_ref(A_MOVE,S_B,srcrefp,dstrefp));
             if (len - 1) > high(smallint) then
               begin
                 list.concat(taicpu.op_const_reg(A_SUBQ,S_L,1,hregister));
                 list.concat(taicpu.op_sym(A_BPL,S_NO,hl));
               end
             else
               list.concat(taicpu.op_reg_sym(A_DBRA,S_NO,hregister,hl));
           end;
      end;

    procedure tcg68k.g_overflowcheck(list: TAsmList; const l:tlocation; def:tdef);
      var
        hl : tasmlabel;
        ai : taicpu;
        cond : TAsmCond;
      begin
        if not(cs_check_overflow in current_settings.localswitches) then
          exit;
        current_asmdata.getjumplabel(hl);
        if not ((def.typ=pointerdef) or
               ((def.typ=orddef) and
                (torddef(def).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                          pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]))) then
          cond:=C_VC
        else
          begin
            { MUL/DIV always sets the overflow flag, and never the carry flag }
            { Note/Fixme: This still doesn't cover the ColdFire, where none of these opcodes
              set either the overflow or the carry flag. So CF must be handled in other ways. }
            if taicpu(list.last).opcode in [A_MULU,A_MULS,A_DIVS,A_DIVU,A_DIVUL,A_DIVSL] then
              cond:=C_VC
            else
              cond:=C_CC;
          end;
        ai:=Taicpu.Op_Sym(A_Bxx,S_NO,hl);
        ai.SetCondition(cond);
        ai.is_jmp:=true;
        list.concat(ai);

        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;

    procedure tcg68k.g_proc_entry(list: TAsmList; localsize: longint; nostackframe:boolean);
      begin
        { Carl's original code used 2x MOVE instead of LINK when localsize = 0.
          However, a LINK seems faster than two moves on everything from 68000
          to '060, so the two move branch here was dropped. (KB) }
        if not nostackframe then
          begin
            localsize:=align(localsize,4);

            if (localsize > high(smallint)) then
              begin
                list.concat(taicpu.op_reg_const(A_LINK,S_W,NR_FRAME_POINTER_REG,0));
                list.concat(taicpu.op_const_reg(A_SUBA,S_L,localsize,NR_STACK_POINTER_REG));
              end
            else
              list.concat(taicpu.op_reg_const(A_LINK,S_W,NR_FRAME_POINTER_REG,-localsize));
          end;
      end;

    procedure tcg68k.g_proc_exit(list : TAsmList; parasize: longint; nostackframe: boolean);
      var
        r,hregister : TRegister;
        ref : TReference;
        ref2: TReference;
      begin
        if not nostackframe then
          begin
            list.concat(taicpu.op_reg(A_UNLK,S_NO,NR_FRAME_POINTER_REG));

            { if parasize is less than zero here, we probably have a cdecl function.
              According to the info here: http://www.makestuff.eu/wordpress/gcc-68000-abi/
              68k GCC uses two different methods to free the stack, depending if the target
              architecture supports RTD or not, and one does callee side, the other does
              caller side free, which looks like a PITA to support. We have to figure this 
              out later. More info welcomed. (KB) }

            if (parasize > 0) and not (current_procinfo.procdef.proccalloption in clearstack_pocalls) then
              begin
                if CPUM68K_HAS_RTD in cpu_capabilities[current_settings.cputype] then
                  list.concat(taicpu.op_const(A_RTD,S_NO,parasize))
                else
                  begin
                    { We must pull the PC Counter from the stack, before  }
                    { restoring the stack pointer, otherwise the PC would }
                    { point to nowhere!                                   }

                    { Instead of doing a slow copy of the return address while trying    }
                    { to feed it to the RTS instruction, load the PC to A1 (scratch reg) }
                    { then free up the stack allocated for paras, then use a JMP (A1) to }
                    { return to the caller with the paras freed. (KB) }

                    hregister:=NR_A1;
                    cg.a_reg_alloc(list,hregister);
                    reference_reset_base(ref,NR_STACK_POINTER_REG,0,ctempposinvalid,4,[]);
                    list.concat(taicpu.op_ref_reg(A_MOVE,S_L,ref,hregister));

                    { instead of using a postincrement above (which also writes the     }
                    { stackpointer reg) simply add 4 to the parasize, the instructions  }
                    { below then take that size into account as well, so SP reg is only }
                    { written once (KB) }
                    parasize:=parasize+4;

                    r:=NR_SP;
                    { can we do a quick addition ... }
                    if (parasize < 9) then
                       list.concat(taicpu.op_const_reg(A_ADDQ,S_L,parasize,r))
                    else { nope ... }
                       begin
                         reference_reset_base(ref2,NR_STACK_POINTER_REG,parasize,ctempposinvalid,4,[]);
                         list.concat(taicpu.op_ref_reg(A_LEA,S_NO,ref2,r));
                       end;

                    reference_reset_base(ref,hregister,0,ctempposinvalid,4,[]);
                    list.concat(taicpu.op_ref(A_JMP,S_NO,ref));
                  end;
              end
            else
              list.concat(taicpu.op_none(A_RTS,S_NO));
          end
        else
          begin
            list.concat(taicpu.op_none(A_RTS,S_NO));
          end;

         { Routines with the poclearstack flag set use only a ret.
           also  routines with parasize=0 }
         { TODO: figure out if these are still relevant to us (KB) }
           (*
         if current_procinfo.procdef.proccalloption in clearstack_pocalls then
           begin
             { complex return values are removed from stack in C code PM }
             if paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef) then
               list.concat(taicpu.op_const(A_RTD,S_NO,4))
             else
               list.concat(taicpu.op_none(A_RTS,S_NO));
           end
         else if (parasize=0) then
           begin
             list.concat(taicpu.op_none(A_RTS,S_NO));
           end
         else
           *)
      end;


    procedure tcg68k.g_save_registers(list:TAsmList);
      var
        dataregs: tcpuregisterset;
        addrregs: tcpuregisterset;
        fpuregs: tcpuregisterset;
        href : treference;
        hreg : tregister;
        hfreg : tregister;
        size : longint;
        fsize : longint;
        r : integer;
        regs_to_save_int,
        regs_to_save_address,
        regs_to_save_fpu: tcpuregisterarray;
      begin
        { The code generated by the section below, particularly the movem.l
          instruction is known to cause an issue when compiled by some GNU 
          assembler versions (I had it with 2.17, while 2.24 seems OK.) 
          when you run into this problem, just call inherited here instead
          to skip the movem.l generation. But better just use working GNU
          AS version instead. (KB) }
        dataregs:=[];
        addrregs:=[];
        fpuregs:=[];

        regs_to_save_int:=paramanager.get_saved_registers_int(current_procinfo.procdef.proccalloption);
        regs_to_save_address:=paramanager.get_saved_registers_address(current_procinfo.procdef.proccalloption);
        regs_to_save_fpu:=paramanager.get_saved_registers_fpu(current_procinfo.procdef.proccalloption);
        { calculate temp. size }
        size:=0;
        fsize:=0;
        hreg:=NR_NO;
        hfreg:=NR_NO;
        for r:=low(regs_to_save_int) to high(regs_to_save_int) do
          if regs_to_save_int[r] in rg[R_INTREGISTER].used_in_proc then
            begin
              hreg:=newreg(R_INTREGISTER,regs_to_save_int[r],R_SUBWHOLE);
              inc(size,sizeof(aint));
              dataregs:=dataregs + [regs_to_save_int[r]];
            end;
        if uses_registers(R_ADDRESSREGISTER) then
          for r:=low(regs_to_save_address) to high(regs_to_save_address) do
            if regs_to_save_address[r] in rg[R_ADDRESSREGISTER].used_in_proc then
              begin
                hreg:=newreg(R_ADDRESSREGISTER,regs_to_save_address[r],R_SUBWHOLE);
                inc(size,sizeof(aint));
                addrregs:=addrregs + [regs_to_save_address[r]];
              end;
        if uses_registers(R_FPUREGISTER) then
          for r:=low(regs_to_save_fpu) to high(regs_to_save_fpu) do
            if regs_to_save_fpu[r] in rg[R_FPUREGISTER].used_in_proc then
              begin
                hfreg:=newreg(R_FPUREGISTER,regs_to_save_fpu[r],R_SUBNONE);
                inc(fsize,fpuregsize);
                fpuregs:=fpuregs + [regs_to_save_fpu[r]];
              end;

        { 68k has no MM registers }
        if uses_registers(R_MMREGISTER) then
          internalerror(2014030201);

        if (size+fsize) > 0 then
          begin
            tg.GetTemp(list,size+fsize,sizeof(aint),tt_noreuse,current_procinfo.save_regs_ref);
            include(current_procinfo.flags,pi_has_saved_regs);

            { Copy registers to temp }
            { NOTE: virtual registers allocated here won't be translated --> no higher-level stuff. }
            href:=current_procinfo.save_regs_ref;
            if (href.offset<low(smallint)) and (current_settings.cputype in cpu_coldfire+[cpu_mc68000]) then
              begin
                list.concat(taicpu.op_reg_reg(A_MOVE,S_L,href.base,NR_A0));
                list.concat(taicpu.op_const_reg(A_ADDA,S_L,href.offset,NR_A0));
                reference_reset_base(href,NR_A0,0,ctempposinvalid,sizeof(pint),[]);
              end;

            if size > 0 then
              if size = sizeof(aint) then
                list.concat(taicpu.op_reg_ref(A_MOVE,S_L,hreg,href))
              else
                list.concat(taicpu.op_regset_ref(A_MOVEM,S_L,dataregs,addrregs,[],href));

            if fsize > 0 then
              begin
                { size is always longword aligned, while fsize is not }
                inc(href.offset,size);
                if fsize = fpuregsize then
                  list.concat(taicpu.op_reg_ref(A_FMOVE,fpuregopsize,hfreg,href))
                else
                  list.concat(taicpu.op_regset_ref(A_FMOVEM,fpuregopsize,[],[],fpuregs,href));
              end;
          end;
      end;


    procedure tcg68k.g_restore_registers(list:TAsmList);
      var
        dataregs: tcpuregisterset;
        addrregs: tcpuregisterset;
        fpuregs : tcpuregisterset;
        href    : treference;
        r       : integer;
        hreg    : tregister;
        hfreg   : tregister;
        size    : longint;
        fsize   : longint;
        regs_to_save_int,
        regs_to_save_address,
        regs_to_save_fpu: tcpuregisterarray;
      begin
        { see the remark about buggy GNU AS versions in g_save_registers() (KB) }
        dataregs:=[];
        addrregs:=[];
        fpuregs:=[];

        if not(pi_has_saved_regs in current_procinfo.flags) then
          exit;
        regs_to_save_int:=paramanager.get_saved_registers_int(current_procinfo.procdef.proccalloption);
        regs_to_save_address:=paramanager.get_saved_registers_address(current_procinfo.procdef.proccalloption);
        regs_to_save_fpu:=paramanager.get_saved_registers_fpu(current_procinfo.procdef.proccalloption);
        { Copy registers from temp }
        size:=0;
        fsize:=0;
        hreg:=NR_NO;
        hfreg:=NR_NO;
        for r:=low(regs_to_save_int) to high(regs_to_save_int) do
          if regs_to_save_int[r] in rg[R_INTREGISTER].used_in_proc then
            begin
              inc(size,sizeof(aint));
              hreg:=newreg(R_INTREGISTER,regs_to_save_int[r],R_SUBWHOLE);
              { Allocate register so the optimizer does not remove the load }
              a_reg_alloc(list,hreg);
              dataregs:=dataregs + [regs_to_save_int[r]];
            end;

        if uses_registers(R_ADDRESSREGISTER) then
          for r:=low(regs_to_save_address) to high(regs_to_save_address) do
            if regs_to_save_address[r] in rg[R_ADDRESSREGISTER].used_in_proc then
              begin
                inc(size,sizeof(aint));
                hreg:=newreg(R_ADDRESSREGISTER,regs_to_save_address[r],R_SUBWHOLE);
                { Allocate register so the optimizer does not remove the load }
                a_reg_alloc(list,hreg);
                addrregs:=addrregs + [regs_to_save_address[r]];
              end;

        if uses_registers(R_FPUREGISTER) then
          for r:=low(regs_to_save_fpu) to high(regs_to_save_fpu) do
            if regs_to_save_fpu[r] in rg[R_FPUREGISTER].used_in_proc then
              begin
                inc(fsize,fpuregsize);
                hfreg:=newreg(R_FPUREGISTER,regs_to_save_fpu[r],R_SUBNONE);
                { Allocate register so the optimizer does not remove the load }
                a_reg_alloc(list,hfreg);
                fpuregs:=fpuregs + [regs_to_save_fpu[r]];
              end;

        { 68k has no MM registers }
        if uses_registers(R_MMREGISTER) then
          internalerror(2014030202);

        { Restore registers from temp }
        href:=current_procinfo.save_regs_ref;
        if (href.offset<low(smallint)) and (current_settings.cputype in cpu_coldfire+[cpu_mc68000]) then
          begin
            list.concat(taicpu.op_reg_reg(A_MOVE,S_L,href.base,NR_A0));
            list.concat(taicpu.op_const_reg(A_ADDA,S_L,href.offset,NR_A0));
            reference_reset_base(href,NR_A0,0,ctempposinvalid,sizeof(pint),[]);
          end;

        if size > 0 then
          if size = sizeof(aint) then
            list.concat(taicpu.op_ref_reg(A_MOVE,S_L,href,hreg))
          else
            list.concat(taicpu.op_ref_regset(A_MOVEM,S_L,href,dataregs,addrregs,[]));

        if fsize > 0 then
          begin
            { size is always longword aligned, while fsize is not }
            inc(href.offset,size);
            if fsize = fpuregsize then
              list.concat(taicpu.op_ref_reg(A_FMOVE,fpuregopsize,href,hfreg))
            else
              list.concat(taicpu.op_ref_regset(A_FMOVEM,fpuregopsize,href,[],[],fpuregs));
          end;

        tg.UnGetTemp(list,current_procinfo.save_regs_ref);
      end;

    procedure tcg68k.sign_extend(list: TAsmList;_oldsize : tcgsize; _newsize : tcgsize; reg: tregister);
      begin
        case _newsize of
          OS_S16, OS_16:
            case _oldsize of
              OS_S8:
                begin { 8 -> 16 bit sign extend }
                  if (isaddressregister(reg)) then
                     internalerror(2014031201);
                  list.concat(taicpu.op_reg(A_EXT,S_W,reg));
                end;
              OS_8: { 8 -> 16 bit zero extend }
                begin
                  if (current_settings.cputype in cpu_coldfire) then
                    { ColdFire has no ANDI.W }
                    list.concat(taicpu.op_const_reg(A_AND,S_L,$FF,reg))
                  else
                    list.concat(taicpu.op_const_reg(A_AND,S_W,$FF,reg));
                end;
              else
                ;
            end;
          OS_S32, OS_32:
            case _oldsize of
              OS_S8:
                begin { 8 -> 32 bit sign extend }
                  if (isaddressregister(reg)) then
                    internalerror(2014031202);
                  if (current_settings.cputype = cpu_MC68000) then
                    begin
                      list.concat(taicpu.op_reg(A_EXT,S_W,reg));
                      list.concat(taicpu.op_reg(A_EXT,S_L,reg));
                    end
                  else
                    begin
                      //list.concat(tai_comment.create(strpnew('sign extend byte')));
                      list.concat(taicpu.op_reg(A_EXTB,S_L,reg));
                    end;
                end;
              OS_8: { 8 -> 32 bit zero extend }
                begin
                  if (isaddressregister(reg)) then
                    internalerror(2015031501);
                  //list.concat(tai_comment.create(strpnew('zero extend byte')));
                  list.concat(taicpu.op_const_reg(A_AND,S_L,$FF,reg));
                end;
              OS_S16: { 16 -> 32 bit sign extend }
                begin
                  { address registers are sign-extended from 16->32 bit anyway
                    automagically on every W operation by the CPU, so this is a NOP }
                  if not isaddressregister(reg) then
                    begin
                      //list.concat(tai_comment.create(strpnew('sign extend word')));
                      list.concat(taicpu.op_reg(A_EXT,S_L,reg));
                    end;
                end;
              OS_16:
                begin
                  if (isaddressregister(reg)) then
                    internalerror(2015031502);
                  //list.concat(tai_comment.create(strpnew('zero extend word')));
                  list.concat(taicpu.op_const_reg(A_AND,S_L,$FFFF,reg));
                end;
              else
                ;
            end;
          else
            ;
        end; { otherwise the size is already correct }
      end; 

    procedure tcg68k.sign_extend(list: TAsmList;_oldsize : tcgsize; reg: tregister);
      begin
        sign_extend(list, _oldsize, OS_INT, reg);
      end;

     procedure tcg68k.a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);

       var
         ai : taicpu;

       begin
         if cond=OC_None then
           ai := Taicpu.Op_sym(A_JMP,S_NO,l)
         else
           begin
             ai:=Taicpu.Op_sym(A_Bxx,S_NO,l);
             ai.SetCondition(TOpCmp2AsmCond[cond]);
           end;
         ai.is_jmp:=true;
         list.concat(ai);
       end;

    { ensures a register is a dataregister. this is often used, as 68k can't do lots of
      operations on an address register. if the register is a dataregister anyway, it
      just returns it untouched.}
    function tcg68k.force_to_dataregister(list: TAsmList; size: TCGSize; reg: TRegister): TRegister;
      var
        scratch_reg: TRegister;
        instr: Taicpu;
      begin
        if isaddressregister(reg) then
          begin
            scratch_reg:=getintregister(list,OS_INT);
            instr:=taicpu.op_reg_reg(A_MOVE,S_L,reg,scratch_reg);
            add_move_instruction(instr);
            list.concat(instr);
            result:=scratch_reg;
          end
        else
          result:=reg;
      end;

    { moves source register to destination register, if the two are not the same. can be used in pair
      with force_to_dataregister() }
    procedure tcg68k.move_if_needed(list: TAsmList; size: TCGSize; src: TRegister; dest: TRegister);
      var
        instr: Taicpu;
      begin
        if (src <> dest) then
          begin
            instr:=taicpu.op_reg_reg(A_MOVE,S_L,src,dest);
            add_move_instruction(instr);
            list.concat(instr);
          end;
      end;


    procedure tcg68k.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
      var
        hsym : tsym;
        href : treference;
        paraloc : Pcgparalocation;
      begin
        { calculate the parameter info for the procdef }
        procdef.init_paraloc_info(callerside);
        hsym:=tsym(procdef.parast.Find('self'));
        if not(assigned(hsym) and
               (hsym.typ=paravarsym)) then
          internalerror(2013100702);
        paraloc:=tparavarsym(hsym).paraloc[callerside].location;
        while paraloc<>nil do
          with paraloc^ do
            begin
              case loc of
                LOC_REGISTER:
                  a_op_const_reg(list,OP_SUB,size,ioffset,register);
                LOC_REFERENCE:
                  begin
                    { offset in the wrapper needs to be adjusted for the stored
                      return address }
                    reference_reset_base(href,reference.index,reference.offset+sizeof(pint),ctempposinvalid,sizeof(pint),[]);
                    { plain 68k could use SUBI on href directly, but this way it works on Coldfire too
                      and it's probably smaller code for the majority of cases (if ioffset small, the
                      load will use MOVEQ) (KB) }
                    a_load_const_reg(list,OS_ADDR,ioffset,NR_D0);
                    list.concat(taicpu.op_reg_ref(A_SUB,S_L,NR_D0,href));
                  end
                else
                  internalerror(2013100703);
              end;
              paraloc:=next;
            end;
      end;


    procedure tcg68k.g_stackpointer_alloc(list : TAsmList;localsize : longint);
      begin
        list.concat(taicpu.op_const_reg(A_SUB,S_L,localsize,NR_STACK_POINTER_REG));
      end;


    procedure tcg68k.check_register_size(size:tcgsize;reg:tregister);
      begin
        if TCGSize2OpSize[size]<>TCGSize2OpSize[reg_cgsize(reg)] then
          internalerror(201512131);
      end;


    function tcg68k.optimize_const_mul_to_shift_sub_add(list: TAsmList; maxops: longint; a: tcgint; size: tcgsize; reg: TRegister): boolean;
      var
        i: longint;
        nextpower: tcgint;
        powerbit: longint;
        submask: tcgint;
        lastshift: longint;
        hreg: tregister;
        firstmov: boolean;
      begin
        nextpower:=nextpowerof2(a,powerbit);
        submask:=nextpower-a;
        result:=not ((popcnt(qword(a)) > maxops) and ((popcnt(qword(submask))+1) > maxops));
        if not result then
          exit;

        list.concat(tai_comment.create(strpnew('optimize_const_mul_to_shift_sub_add, multiplier: '+tostr(a))));

        lastshift:=0;
        hreg:=getintregister(list,OS_INT);
        if (popcnt(qword(a)) < (popcnt(qword(submask))+1)) then
          begin
            { doing additions }
            firstmov:=(a and 1) = 0;

            if not firstmov then
              a_load_reg_reg(list,size,OS_INT,reg,hreg);

            for i:=1 to bsrqword(a) do
              if ((a shr i) and 1) = 1 then
                begin
                  if firstmov then
                    begin
                      a_op_const_reg(list,OP_SHL,OS_INT,i-lastshift,reg);
                      a_load_reg_reg(list,OS_INT,OS_INT,reg,hreg);
                      firstmov:=false;
                    end
                  else
                    begin
                      a_op_const_reg(list,OP_SHL,OS_INT,i-lastshift,hreg);
                      a_op_reg_reg(list,OP_ADD,OS_INT,hreg,reg);
                    end;
                  lastshift:=i;
                end;
          end
        else
          begin
            { doing subtractions }
            a_load_const_reg(list,OS_INT,0,hreg);
            for i:=0 to bsrqword(submask) do
              if ((submask shr i) and 1) = 1 then
                begin
                  a_op_const_reg(list,OP_SHL,OS_INT,i-lastshift,reg);
                  a_op_reg_reg(list,OP_SUB,OS_INT,reg,hreg);
                  lastshift:=i;
                end;
            a_op_const_reg(list,OP_SHL,OS_INT,powerbit-lastshift,reg);
            a_op_reg_reg(list,OP_ADD,OS_INT,hreg,reg);
          end;
        result:=true;
      end;


{****************************************************************************}
{                               TCG64F68K                                    }
{****************************************************************************}
    procedure tcg64f68k.a_op64_reg_reg(list : TAsmList;op:TOpCG;size: tcgsize; regsrc,regdst : tregister64);
      var
        opcode : tasmop;
        xopcode : tasmop;
        instr : taicpu;
      begin
        opcode := topcg2tasmop[op];
        xopcode := topcg2tasmopx[op];

        case op of
          OP_ADD,OP_SUB:
            begin
              { if one of these three registers is an address
              register, we'll really get into problems! }
              if isaddressregister(regdst.reglo) or
                 isaddressregister(regdst.reghi) or
                 isaddressregister(regsrc.reghi) then
                internalerror(2014030101);
              list.concat(taicpu.op_reg_reg(opcode,S_L,regsrc.reglo,regdst.reglo));
              list.concat(taicpu.op_reg_reg(xopcode,S_L,regsrc.reghi,regdst.reghi));
            end;
          OP_AND,OP_OR:
            begin
              { at least one of the registers must be a data register }
              if (isaddressregister(regdst.reglo) and
                  isaddressregister(regsrc.reglo)) or
                 (isaddressregister(regsrc.reghi) and
                  isaddressregister(regdst.reghi)) then
                internalerror(2014030102);
              cg.a_op_reg_reg(list,op,OS_32,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,op,OS_32,regsrc.reghi,regdst.reghi);
            end;
          { this is handled in 1st pass for 32-bit cpu's (helper call) }
          OP_IDIV,OP_DIV,
          OP_IMUL,OP_MUL: 
            internalerror(2002081701);
          { this is also handled in 1st pass for 32-bit cpu's (helper call) }
          OP_SAR,OP_SHL,OP_SHR:
            internalerror(2002081702);
          OP_XOR:
            begin
              if isaddressregister(regdst.reglo) or
                 isaddressregister(regsrc.reglo) or
                 isaddressregister(regsrc.reghi) or
                 isaddressregister(regdst.reghi) then
                internalerror(2014030103);
              cg.a_op_reg_reg(list,op,OS_32,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,op,OS_32,regsrc.reghi,regdst.reghi);
            end;
          OP_NEG,OP_NOT:
            begin
              if isaddressregister(regdst.reglo) or
                 isaddressregister(regdst.reghi) then
               internalerror(2014030104);
              instr:=taicpu.op_reg_reg(A_MOVE,S_L,regsrc.reglo,regdst.reglo);
              cg.add_move_instruction(instr);
              list.concat(instr);
              instr:=taicpu.op_reg_reg(A_MOVE,S_L,regsrc.reghi,regdst.reghi);
              cg.add_move_instruction(instr);
              list.concat(instr);
              if (op = OP_NOT) then
                xopcode:=opcode;
              list.concat(taicpu.op_reg(opcode,S_L,regdst.reglo));
              list.concat(taicpu.op_reg(xopcode,S_L,regdst.reghi));
            end;
          else
            ;
        end; { end case }
      end;


    procedure tcg64f68k.a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);
      var
        href : treference;
        hreg: tregister;
      begin
        case op of
          OP_NEG,OP_NOT:
            begin
              a_load64_ref_reg(list,ref,reg);
              a_op64_reg_reg(list,op,size,reg,reg);
            end;
          OP_AND,OP_OR:
            begin
              href:=ref;
              tcg68k(cg).fixref(list,href,false);
              list.concat(taicpu.op_ref_reg(topcg2tasmop[op],S_L,href,reg.reghi));
              inc(href.offset,4);
              list.concat(taicpu.op_ref_reg(topcg2tasmop[op],S_L,href,reg.reglo));
            end;
          OP_ADD,OP_SUB:
            begin
              href:=ref;
              tcg68k(cg).fixref(list,href,false);
              hreg:=cg.getintregister(list,OS_32);
              cg.a_load_ref_reg(list,OS_32,OS_32,href,hreg);
              inc(href.offset,4);
              list.concat(taicpu.op_ref_reg(topcg2tasmop[op],S_L,href,reg.reglo));
              list.concat(taicpu.op_reg_reg(topcg2tasmopx[op],S_L,hreg,reg.reghi));
            end;
        else
          { XOR does not allow reference for source; ADD/SUB do not allow reference for
            high dword, although low dword can still be handled directly. }
          inherited a_op64_ref_reg(list,op,size,ref,reg);
        end;
      end;


    procedure tcg64f68k.a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64;const ref : treference);
      var
        href: treference;
        hreg: tregister;
      begin
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              href:=ref;
              tcg68k(cg).fixref(list,href,false);
              list.concat(taicpu.op_reg_ref(topcg2tasmop[op],S_L,reg.reghi,href));
              inc(href.offset,4);
              list.concat(taicpu.op_reg_ref(topcg2tasmop[op],S_L,reg.reglo,href));
            end;
          OP_ADD,OP_SUB:
            begin
              href:=ref;
              tcg68k(cg).fixref(list,href,false);
              hreg:=cg.getintregister(list,OS_32);
              cg.a_load_ref_reg(list,OS_32,OS_32,href,hreg);
              inc(href.offset,4);
              list.concat(taicpu.op_reg_ref(topcg2tasmop[op],S_L,reg.reglo,href));
              list.concat(taicpu.op_reg_reg(topcg2tasmopx[op],S_L,reg.reghi,hreg));
              dec(href.offset,4);
              cg.a_load_reg_ref(list,OS_32,OS_32,hreg,href);
            end;
        else
          inherited a_op64_reg_ref(list,op,size,reg,ref);
        end;
      end;


    procedure tcg64f68k.a_op64_const_reg(list : TAsmList;op:TOpCG;size: tcgsize; value : int64;regdst : tregister64);
      var
        lowvalue : cardinal;
        highvalue : cardinal;
        opcode : tasmop;
        xopcode : tasmop;
        hreg : tregister;
      begin
        { is it optimized out ? }
        { optimize64_op_const_reg doesn't seem to be used in any cg64f32 right now. why? (KB) }
        { if cg.optimize64_op_const_reg(list,op,value,reg) then
            exit; }

        lowvalue := cardinal(value);
        highvalue := value shr 32;

        opcode := topcg2tasmop[op];
        xopcode := topcg2tasmopx[op];

        { the destination registers must be data registers }
        if isaddressregister(regdst.reglo) or
           isaddressregister(regdst.reghi) then
          internalerror(2014030105);
        case op of
          OP_ADD,OP_SUB:
            begin
              hreg:=cg.getintregister(list,OS_INT);
              { cg.a_load_const_reg provides optimized loading to register for special cases }
              cg.a_load_const_reg(list,OS_S32,tcgint(highvalue),hreg);
              { don't use cg.a_op_const_reg() here, because a possible optimized
                ADDQ/SUBQ wouldn't set the eXtend bit }
              list.concat(taicpu.op_const_reg(opcode,S_L,tcgint(lowvalue),regdst.reglo));
              list.concat(taicpu.op_reg_reg(xopcode,S_L,hreg,regdst.reghi));
            end;
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg(list,op,OS_S32,tcgint(lowvalue),regdst.reglo);
              cg.a_op_const_reg(list,op,OS_S32,tcgint(highvalue),regdst.reghi);
            end;
          { this is handled in 1st pass for 32-bit cpus (helper call) }
          OP_IDIV,OP_DIV,
          OP_IMUL,OP_MUL:
            internalerror(2002081701);
          { this is also handled in 1st pass for 32-bit cpus (helper call) }
          OP_SAR,OP_SHL,OP_SHR:
            internalerror(2002081702);
          { these should have been handled already by earlier passes }
          OP_NOT,OP_NEG:
            internalerror(2012110403);
          else
            ;
        end; { end case }
      end;


    procedure tcg64f68k.a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference);
      var
        tmpref: treference;
      begin
        tmpref:=ref;
        tcg68k(cg).fixref(list,tmpref,false);
        cg.a_load_reg_ref(list,OS_32,OS_32,reg.reghi,tmpref);
        inc(tmpref.offset,4);
        cg.a_load_reg_ref(list,OS_32,OS_32,reg.reglo,tmpref);
      end;

    procedure tcg64f68k.a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64);
      var
        tmpref: treference;
      begin
        { do not allow 64bit values to be loaded to address registers }
        if isaddressregister(reg.reglo) or
           isaddressregister(reg.reghi) then
          internalerror(2016050501);

        tmpref:=ref;
        tcg68k(cg).fixref(list,tmpref,false);
        cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg.reghi);
        inc(tmpref.offset,4);
        cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg.reglo);
      end;


procedure create_codegen;
  begin
    cg := tcg68k.create;
    cg64 :=tcg64f68k.create;
  end;

end.
