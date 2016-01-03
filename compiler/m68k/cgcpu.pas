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
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{DEFINE DEBUG_CHARLIE}

{$IFNDEF DEBUG_CHARLIE}
{$WARNINGS OFF}
{$ENDIF}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : tcgpara);override;
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);override;
>>>>>>> origin/cpstrnew
        procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : tcgpara);override;
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : tcgpara);override;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_reg(list : TAsmList;reg : tregister);override;

        procedure a_load_const_reg(list : TAsmList;size : tcgsize;a : tcgint;register : tregister);override;
        procedure a_load_const_ref(list : TAsmList; tosize: tcgsize; a : tcgint;const ref : treference);override;

        procedure a_load_reg_ref(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);override;
        procedure a_load_reg_reg(list : TAsmList;fromsize,tosize : tcgsize;reg1,reg2 : tregister);override;
        procedure a_load_ref_reg(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);override;
        procedure a_load_ref_ref(list : TAsmList;fromsize,tosize : tcgsize;const sref : treference;const dref : treference);override;

        procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);override;
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;
        procedure a_loadfpu_reg_cgpara(list : TAsmList; size : tcgsize;const reg : tregister;const cgpara : TCGPara); override;
        procedure a_loadfpu_ref_cgpara(list : TAsmList; size : tcgsize;const ref : treference;const cgpara : TCGPara);override;

<<<<<<< HEAD
        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: tcgsize; a: tcgint; reg: TRegister); override;
        procedure a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;
        procedure a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize; reg: TRegister; const ref: TReference); override;
=======
        procedure a_loadmm_reg_reg(list: TAsmList;fromsize,tosize : tcgsize; reg1, reg2: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_ref_reg(list: TAsmList;fromsize,tosize : tcgsize; const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_ref(list: TAsmList;fromsize,tosize : tcgsize; reg: tregister; const ref: treference;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_cgpara(list: TAsmList; size: tcgsize; reg: tregister;const locpara : TCGPara;shuffle : pmmshuffle); override;

        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: tcgsize; a: aint; reg: TRegister); override;
//        procedure a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: aint; const ref: TReference); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; reg1, reg2: TRegister); override;
>>>>>>> graemeg/cpstrnew

        procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister; l : tasmlabel);override;
        procedure a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference; l : tasmlabel); override;
        procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel); override;
        procedure a_jmp_name(list : TAsmList;const s : string); override;
        procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;
        procedure a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel); override;
        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister); override;

        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;
        { generates overflow checking code for a node }
        procedure g_overflowcheck(list: TAsmList; const l:tlocation; def:tdef); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);override;

<<<<<<< HEAD
        procedure g_save_registers(list:TAsmList);override;
        procedure g_restore_registers(list:TAsmList);override;
=======
//        procedure g_restore_frame_pointer(list : TAsmList);override;
//        procedure g_return_from_proc(list : TAsmList;parasize : aint);override;
        procedure g_restore_registers(list:TAsmList);override;
        procedure g_save_registers(list:TAsmList);override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======

//        procedure g_save_all_registers(list : TAsmList);override;
//        procedure g_restore_all_registers(list : TAsmList;const funcretparaloc:TCGPara);override;
>>>>>>> origin/fixes_2_2

        procedure g_adjust_self_value(list:TAsmList;procdef:tprocdef;ioffset:tcgint);override;

        { # Sign or zero extend the register to a full 32-bit value.
            The new value is left in the same register.
        }
        procedure sign_extend(list: TAsmList;_oldsize : tcgsize; reg: tregister);
        procedure sign_extend(list: TAsmList;_oldsize : tcgsize; _newsize : tcgsize; reg: tregister);

        procedure g_stackpointer_alloc(list : TAsmList;localsize : longint);override;
        function fixref(list: TAsmList; var ref: treference): boolean;
        function force_to_dataregister(list: TAsmList; size: TCGSize; reg: TRegister): TRegister;
        procedure move_if_needed(list: TAsmList; size: TCGSize; src: TRegister; dest: TRegister);
     protected
        procedure call_rtl_mul_const_reg(list:tasmlist;size:tcgsize;a:tcgint;reg:tregister;const name:string);
        procedure call_rtl_mul_reg_reg(list:tasmlist;reg1,reg2:tregister;const name:string);
        procedure check_register_size(size:tcgsize;reg:tregister);
     private
        procedure a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
     end;

     tcg64f68k = class(tcg64f32)
       procedure a_op64_reg_reg(list : TAsmList;op:TOpCG; size: tcgsize; regsrc,regdst : tregister64);override;
       procedure a_op64_const_reg(list : TAsmList;op:TOpCG; size: tcgsize; value : int64;regdst : tregister64);override;
       procedure a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);override;
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

<<<<<<< HEAD
=======
    const
      TCGSize2OpSize: Array[tcgsize] of topsize =
        (S_NO,S_B,S_W,S_L,S_L,S_NO,S_B,S_W,S_L,S_L,S_NO,
         S_FS,S_FD,S_FX,S_NO,S_NO,
         S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO);

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
    procedure create_codegen;

  implementation

    uses
       globals,verbose,systems,cutils,
       symsym,symtable,defutil,paramgr,procinfo,
       rgobj,tgobj,rgcpu,fmodule;


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
            if assigned(current_procinfo) and (current_procinfo.framepointer = NR_FRAME_POINTER_REG)
               and (reg = RS_FRAME_POINTER_REG) then
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_load_reg');_cgpara
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_param_reg');
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_param_reg');
{$endif DEBUG_CHARLIE}
>>>>>>> origin/fixes_2_2
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

            reference_reset_base(ref, NR_STACK_POINTER_REG, 0, cgpara.alignment);
            ref.direction := dir_dec;
            list.concat(taicpu.op_reg_ref(A_MOVE,tcgsize2opsize[pushsize],makeregsize(list,r,pushsize),ref));
          end
        else
          inherited a_load_reg_cgpara(list,size,r,cgpara);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure tcg68k.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : tcgpara);
=======
    procedure tcg68k.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);
>>>>>>> graemeg/cpstrnew
=======
    procedure tcg68k.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);
>>>>>>> graemeg/cpstrnew
=======
    procedure tcg68k.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);
>>>>>>> graemeg/cpstrnew
=======
    procedure tcg68k.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const cgpara : tcgpara);
>>>>>>> origin/cpstrnew
      var
        pushsize : tcgsize;
        ref : treference;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_load_const');_cgpara
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_param_const');
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_param_const');
{$endif DEBUG_CHARLIE}
>>>>>>> origin/fixes_2_2
        if use_push(cgpara) then
          begin
            cgpara.check_simple_location;
            if tcgsize2size[cgpara.location^.size]>cgpara.alignment then
              pushsize:=cgpara.location^.size
            else
              pushsize:=int_cgsize(cgpara.alignment);

            reference_reset_base(ref, NR_STACK_POINTER_REG, 0, cgpara.alignment);
            ref.direction := dir_dec;
            list.concat(taicpu.op_const_ref(A_MOVE,tcgsize2opsize[pushsize],a,ref));
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
{ TODO: FIX ME!!! this also triggers location bug }
          {if (paraloc^.loc<>LOC_REFERENCE) or
             (paraloc^.reference.index<>NR_STACK_POINTER_REG) or
             (tcgsize2size[paraloc^.size]>sizeof(tcgint)) then
            internalerror(200501162);}

          { Pushes are needed in reverse order, add the size of the
            current location to the offset where to load from. This
            prevents wrong calculations for the last location when
            the size is not a power of 2 }
          if assigned(paraloc^.next) then
            pushdata(paraloc^.next,ofs+tcgsize2size[paraloc^.size]);
          { Push the data starting at ofs }
          href:=r;
          inc(href.offset,ofs);
          fixref(list,href);
          if tcgsize2size[paraloc^.size]>cgpara.alignment then
            pushsize:=paraloc^.size
          else
            pushsize:=int_cgsize(cgpara.alignment);

          reference_reset_base(ref, NR_STACK_POINTER_REG, 0, tcgsize2size[pushsize]);
          ref.direction := dir_dec;

          if tcgsize2size[paraloc^.size]<cgpara.alignment then
            begin
              tmpreg:=getintregister(list,pushsize);
              a_load_ref_reg(list,paraloc^.size,pushsize,href,tmpreg);
              list.concat(taicpu.op_reg_ref(A_MOVE,tcgsize2opsize[pushsize],tmpreg,ref));
            end
          else
              list.concat(taicpu.op_ref_ref(A_MOVE,tcgsize2opsize[pushsize],href,ref));
        end;

      var
        len : tcgint;
        href : treference;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_load_ref');_cgpara
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_param_ref');
>>>>>>> graemeg/fixes_2_2
{$endif DEBUG_CHARLIE}

>>>>>>> graemeg/cpstrnew
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_param_ref');
{$endif DEBUG_CHARLIE}

>>>>>>> origin/fixes_2_2
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
                reference_reset_base(href,NR_STACK_POINTER_REG,0,cgpara.alignment);
                g_concatcopy(list,r,href,len);
              end
            else
              begin
                if tcgsize2size[cgpara.size]<>tcgsize2size[size] then
                  internalerror(200501161);
                { We need to push the data in reverse order,
                  therefor we use a recursive algorithm }
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        { 68k always passes arguments on the stack }
        if use_push(cgpara) then
          begin
            //list.concat(tai_comment.create(strpnew('a_loadaddr_ref_cgpara: PEA')));
            cgpara.check_simple_location;
            tmpref:=r;
            fixref(list,tmpref);
            list.concat(taicpu.op_ref(A_PEA,S_NO,tmpref));
          end
        else
          inherited a_loadaddr_ref_cgpara(list,r,cgpara);
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_loadaddr_ref');_cgpara
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_paramaddr_ref');
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_paramaddr_ref');
>>>>>>> origin/fixes_2_2
{$endif DEBUG_CHARLIE}
        with r do
          begin
            { i suppose this is not required for m68k (KB) }
//            if (segment<>NR_NO) then
//              cgmessage(cg_e_cant_use_far_pointer_there);
            if not use_push(cgpara) then
              begin
                cgpara.check_simple_location;
                opsize:=tcgsize2opsize[OS_ADDR];
                if (segment=NR_NO) and (base=NR_NO) and (index=NR_NO) then
                  begin
                    if assigned(symbol) then
//                      list.concat(Taicpu.Op_sym_ofs(A_PUSH,opsize,symbol,offset))
                    else;
//                      list.concat(Taicpu.Op_const(A_PUSH,opsize,offset));
                  end
                else if (segment=NR_NO) and (base=NR_NO) and (index<>NR_NO) and
                        (offset=0) and (scalefactor=0) and (symbol=nil) then
//                  list.concat(Taicpu.Op_reg(A_PUSH,opsize,index))
                else if (segment=NR_NO) and (base<>NR_NO) and (index=NR_NO) and
                        (offset=0) and (symbol=nil) then
//                  list.concat(Taicpu.Op_reg(A_PUSH,opsize,base))
                else
                  begin
                    tmpreg:=getaddressregister(list);
                    a_loadaddr_ref_reg(list,r,tmpreg);
//                    list.concat(taicpu.op_reg(A_PUSH,opsize,tmpreg));
                  end;
              end
            else
              inherited a_loadaddr_ref_cgpara(list,r,cgpara);
          end;
>>>>>>> graemeg/cpstrnew
      end;

    function tcg68k.fixref(list: TAsmList; var ref: treference): boolean;
       var
         hreg,idxreg : tregister;
         href : treference;
         instr : taicpu;
         scale : aint;
       begin
         result:=false;
         { The MC68020+ has extended
           addressing capabilities with a 32-bit
           displacement.
         }
         { first ensure that base is an address register }
         if ((ref.base<>NR_NO) and (ref.index<>NR_NO)) and
            (not isaddressregister(ref.base) and isaddressregister(ref.index)) and
            (ref.scalefactor < 2) then
           begin
             { if we have both base and index registers, but base is data and index
               is address, we can just swap them, as FPC always uses long index.
               but we can only do this, if the index has no scalefactor }
             hreg:=ref.base;
             ref.base:=ref.index;
             ref.index:=hreg;
             //list.concat(tai_comment.create(strpnew('fixref: base and index swapped')));
           end;

         if (not assigned (ref.symbol) and (current_settings.cputype<>cpu_MC68000)) and
            (ref.base<>NR_NO) and not isaddressregister(ref.base) then
           begin
             hreg:=getaddressregister(list);
             instr:=taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg);
             add_move_instruction(instr);
             list.concat(instr);
             fixref:=true;
             ref.base:=hreg;
           end;
         if (current_settings.cputype=cpu_MC68020) then
           exit;
         { ToDo: check which constraints of Coldfire also apply to MC68000 }
         case current_settings.cputype of
           cpu_MC68000:
             begin
               if (ref.base<>NR_NO) then
                 begin
                   if (ref.index<>NR_NO) and assigned(ref.symbol) then
                     begin
                       hreg:=getaddressregister(list);
                       list.concat(taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg));
                       list.concat(taicpu.op_reg_reg(A_ADD,S_L,ref.index,hreg));
                       ref.index:=NR_NO;
                       ref.base:=hreg;
                     end;
                   { base + reg }
                   if ref.index <> NR_NO then
                      begin
                         { base + reg + offset }
                         if (ref.offset < low(shortint)) or (ref.offset > high(shortint)) then
                           begin
                             hreg:=getaddressregister(list);
                             list.concat(taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg));
                             list.concat(taicpu.op_const_reg(A_ADD,S_L,ref.offset,hreg));
                             fixref:=true;
                             ref.offset:=0;
                             ref.base:=hreg;
                             exit;
                           end;
                      end
                   else
                   { base + offset }
                   if (ref.offset < low(smallint)) or (ref.offset > high(smallint)) then
                     begin
                       hreg:=getaddressregister(list);
                       list.concat(taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg));
                       list.concat(taicpu.op_const_reg(A_ADD,S_L,ref.offset,hreg));
                       fixref:=true;
                       ref.offset:=0;
                       ref.base:=hreg;
                       exit;
                     end;
                   if assigned(ref.symbol) then
                     begin
                       hreg:=getaddressregister(list);
                       idxreg:=ref.base;
                       ref.base:=NR_NO;
                       list.concat(taicpu.op_ref_reg(A_LEA,S_L,ref,hreg));
                       reference_reset_base(ref,hreg,0,ref.alignment);
                       fixref:=true;
                       ref.index:=idxreg;
                     end
                   else if not isaddressregister(ref.base) then
                     begin
                       hreg:=getaddressregister(list);
                       instr:=taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg);
                       //add_move_instruction(instr);
                       list.concat(instr);
                       fixref:=true;
                       ref.base:=hreg;
                     end;
                 end
               else
                 { Note: symbol -> ref would be supported as long as ref does not
                         contain a offset or index... (maybe something for the
                         optimizer) }
                 if Assigned(ref.symbol) and (ref.index<>NR_NO) then
                   begin
                     hreg:=cg.getaddressregister(list);
                     idxreg:=ref.index;
                     ref.index:=NR_NO;
                     list.concat(taicpu.op_ref_reg(A_LEA,S_L,ref,hreg));
                     reference_reset_base(ref,hreg,0,ref.alignment);
                     ref.index:=idxreg;
                     fixref:=true;
                   end;
             end;
           cpu_isa_a,
           cpu_isa_a_p,
           cpu_isa_b,
           cpu_isa_c:
             begin
               if (ref.base<>NR_NO) then
                 begin
                   if assigned(ref.symbol) then
                     begin
                       //list.concat(tai_comment.create(strpnew('fixref: symbol')));
                       hreg:=cg.getaddressregister(list);
                       reference_reset_symbol(href,ref.symbol,ref.offset,ref.alignment);
                       list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
                       if ref.index<>NR_NO then
                         begin
                           { fold the symbol + offset into the base, not the base into the index,
                             because that might screw up the scalefactor of the reference }
                           //list.concat(tai_comment.create(strpnew('fixref: symbol + offset (index + base)')));
                           idxreg:=getaddressregister(list);
                           reference_reset_base(href,ref.base,0,ref.alignment);
                           href.index:=hreg;
                           hreg:=getaddressregister(list);
                           list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
                           ref.base:=hreg;
                         end
                       else
                         ref.index:=hreg;

                       ref.offset:=0;
                       ref.symbol:=nil;
                       fixref:=true;
                     end
                   else
                     { base + reg }
                     if ref.index <> NR_NO then
                       begin
                         { base + reg + offset }
                         if (ref.offset < low(shortint)) or (ref.offset > high(shortint)) then
                           begin
                             hreg:=getaddressregister(list);
                             if (ref.offset < low(smallint)) or (ref.offset > high(smallint)) then
                               begin
                                 instr:=taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg);
                                 //add_move_instruction(instr);
                                 list.concat(instr);
                                 list.concat(taicpu.op_const_reg(A_ADD,S_L,ref.offset,hreg));
                               end
                             else
                               begin
                                 //list.concat(tai_comment.create(strpnew('fixref: base + reg + offset lea')));
                                 reference_reset_base(href,ref.base,ref.offset,ref.alignment);
                                 list.concat(taicpu.op_ref_reg(A_LEA,S_NO,href,hreg));
                               end;
                             fixref:=true;
                             ref.base:=hreg;
                             ref.offset:=0;
                             exit;
                           end;
                       end
                     else
                       { base + offset }
                       if (ref.offset < low(smallint)) or (ref.offset > high(smallint)) then
                         begin
                           hreg:=getaddressregister(list);
                           instr:=taicpu.op_reg_reg(A_MOVE,S_L,ref.base,hreg);
                           //add_move_instruction(instr);
                           list.concat(instr);
                           list.concat(taicpu.op_const_reg(A_ADD,S_L,ref.offset,hreg));
                           fixref:=true;
                           ref.offset:=0;
                           ref.base:=hreg;
                           exit;
                         end;
                 end
               else
                 { Note: symbol -> ref would be supported as long as ref does not
                         contain a offset or index... (maybe something for the
                         optimizer) }
                 if Assigned(ref.symbol) {and (ref.index<>NR_NO)} then
                   begin
                     hreg:=cg.getaddressregister(list);
                     idxreg:=ref.index;
                     scale:=ref.scalefactor;
                     ref.index:=NR_NO;
                     list.concat(taicpu.op_ref_reg(A_LEA,S_L,ref,hreg));
                     reference_reset_base(ref,hreg,0,ref.alignment);
                     ref.index:=idxreg;
                     ref.scalefactor:=scale;
                     fixref:=true;
                   end;
             end;
         end;
       end;


    procedure tcg68k.call_rtl_mul_const_reg(list:tasmlist;size:tcgsize;a:tcgint;reg:tregister;const name:string);
      var
        paraloc1,paraloc2,paraloc3 : tcgpara;
        pd : tprocdef;
      begin
        pd:=search_system_proc(name);
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
        paramanager.getintparaloc(list,pd,1,paraloc1);
        paramanager.getintparaloc(list,pd,2,paraloc2);
        paramanager.getintparaloc(list,pd,3,paraloc3);
        a_load_const_cgpara(list,OS_8,0,paraloc3);
        a_load_const_cgpara(list,size,a,paraloc2);
        a_load_reg_cgpara(list,OS_32,reg,paraloc1);
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        if current_settings.fputype in [fpu_68881] then
          alloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        alloccpuregisters(list,R_ADDRESSREGISTER,paramanager.get_volatile_registers_address(pocall_default));
        alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        a_call_name(list,name,false);
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        dealloccpuregisters(list,R_ADDRESSREGISTER,paramanager.get_volatile_registers_address(pocall_default));
        if current_settings.fputype in [fpu_68881] then
          dealloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        cg.a_reg_alloc(list,NR_FUNCTION_RESULT_REG);
        cg.a_load_reg_reg(list,OS_32,OS_32,NR_FUNCTION_RESULT_REG,reg);
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcg68k.call_rtl_mul_reg_reg(list:tasmlist;reg1,reg2:tregister;const name:string);
      var
        paraloc1,paraloc2,paraloc3 : tcgpara;
        pd : tprocdef;
      begin
       pd:=search_system_proc(name);
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
        paramanager.getintparaloc(list,pd,1,paraloc1);
        paramanager.getintparaloc(list,pd,2,paraloc2);
        paramanager.getintparaloc(list,pd,3,paraloc3);
        a_load_const_cgpara(list,OS_8,0,paraloc3);
        a_load_reg_cgpara(list,OS_32,reg1,paraloc2);
        a_load_reg_cgpara(list,OS_32,reg2,paraloc1);
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        if current_settings.fputype in [fpu_68881] then
          alloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        alloccpuregisters(list,R_ADDRESSREGISTER,paramanager.get_volatile_registers_address(pocall_default));
        a_call_name(list,name,false);
        dealloccpuregisters(list,R_ADDRESSREGISTER,paramanager.get_volatile_registers_address(pocall_default));
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        if current_settings.fputype in [fpu_68881] then
          dealloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        cg.a_reg_alloc(list,NR_FUNCTION_RESULT_REG);
        cg.a_load_reg_reg(list,OS_32,OS_32,NR_FUNCTION_RESULT_REG,reg2);
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure tcg68k.a_call_name(list : TAsmList;const s : string; weak: boolean);
      var
        sym: tasmsymbol;
      begin
        if not(weak) then
          sym:=current_asmdata.RefAsmSymbol(s)
        else
          sym:=current_asmdata.WeakRefAsmSymbol(s);

        list.concat(taicpu.op_sym(A_JSR,S_NO,sym));
      end;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure tcg68k.a_call_reg(list : TAsmList;reg: tregister);
      var
        tmpref : treference;
	tmpreg : tregister;
      begin
{$ifdef DEBUG_CHARLIE}
	list.concat(tai_comment.create(strpnew('a_call_reg')));
{$endif}
	if isaddressregister(reg) then
	  begin
	    { if we have an address register, we can jump to the address directly }
            reference_reset_base(tmpref,reg,0);
	  end
	else
	  begin
	    { if we have a data register, we need to move it to an address register first }
	    tmpreg:=getaddressregister(list);
            reference_reset_base(tmpref,tmpreg,0);
	    list.concat(taicpu.op_reg_reg(A_MOVE,S_L,reg,tmpreg));
	  end;
	list.concat(taicpu.op_ref(A_JSR,S_NO,tmpref));
     end;
>>>>>>> graemeg/fixes_2_2


    procedure tcg68k.a_call_reg(list : TAsmList;reg: tregister);
      var
        tmpref : treference;
        tmpreg : tregister;
        instr : taicpu;
      begin
        if isaddressregister(reg) then
          begin
            { if we have an address register, we can jump to the address directly }
            reference_reset_base(tmpref,reg,0,4);
          end
        else
          begin
            { if we have a data register, we need to move it to an address register first }
            tmpreg:=getaddressregister(list);
            reference_reset_base(tmpref,tmpreg,0,4);
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
<<<<<<< HEAD
<<<<<<< HEAD
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
              if (current_settings.cputype in [cpu_isa_b,cpu_isa_c]) and 
                 ((longint(a) = -1) or ((longint(a) > 0) and (longint(a) < 8))) then
                list.concat(taicpu.op_const_reg(A_MOV3Q,S_L,longint(a),register))
              else
                { We don't have to specify the size here, the assembler will decide the size of
                  the operand it needs. If this ends up as a MOVEA.W, that will sign extend the
                  value in the dest. reg to full 32 bits (specific to Ax regs only) }
                list.concat(taicpu.op_const_reg(A_MOVEA,S_NO,longint(a),register));
          end
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef DEBUG_CHARLIE}
//        writeln('a_load_const_reg');
{$endif DEBUG_CHARLIE}

        if isaddressregister(register) then
         begin
           list.concat(taicpu.op_const_reg(A_MOVE,S_L,longint(a),register))
         end
>>>>>>> graemeg/fixes_2_2
        else
        if a = 0 then
           list.concat(taicpu.op_reg(A_CLR,S_L,register))
        else
         begin
           { Prefer MOV3Q if applicable, it allows replacement spilling for register }
           if (current_settings.cputype in [cpu_isa_b,cpu_isa_c]) and
             ((longint(a)=-1) or ((longint(a)>0) and (longint(a)<8))) then
             list.concat(taicpu.op_const_reg(A_MOV3Q,S_L,longint(a),register))
           else if (longint(a) >= low(shortint)) and (longint(a) <= high(shortint)) then
              list.concat(taicpu.op_const_reg(A_MOVEQ,S_L,longint(a),register))
           else
             begin
               { ISA B/C Coldfire has sign extend/zero extend moves }
               if (current_settings.cputype in [cpu_isa_b,cpu_isa_c]) and 
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
<<<<<<< HEAD
<<<<<<< HEAD
        a:=longint(a);
        href:=ref;
        fixref(list,href);
        if (a=0) and not (current_settings.cputype = cpu_mc68000) then
          list.concat(taicpu.op_ref(A_CLR,tcgsize2opsize[tosize],href))
        else if (tcgsize2opsize[tosize]=S_L) and
           (current_settings.cputype in [cpu_isa_b,cpu_isa_c]) and
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
          list.concat(taicpu.op_const_ref(A_MOVE,tcgsize2opsize[tosize],longint(a),href));
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef DEBUG_CHARLIE}
        list.concat(tai_comment.create(strpnew('a_load_const_ref')));
{$endif DEBUG_CHARLIE}

        list.concat(taicpu.op_const_ref(A_MOVE,S_L,longint(a),ref));
>>>>>>> graemeg/fixes_2_2
      end;


    procedure tcg68k.a_load_reg_ref(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);
      var
        href : treference;
      begin
        href := ref;
        fixref(list,href);
<<<<<<< HEAD
<<<<<<< HEAD
        if tcgsize2size[fromsize]<tcgsize2size[tosize] then
          a_load_reg_reg(list,fromsize,tosize,register,register);
        { move to destination reference }
        list.concat(taicpu.op_reg_ref(A_MOVE,TCGSize2OpSize[tosize],register,href));
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef DEBUG_CHARLIE}
        list.concat(tai_comment.create(strpnew('a_load_reg_ref')));
{$endif DEBUG_CHARLIE}
        { move to destination reference }
        list.concat(taicpu.op_reg_ref(A_MOVE,TCGSize2OpSize[fromsize],register,href));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure tcg68k.a_load_ref_ref(list : TAsmList;fromsize,tosize : tcgsize;const sref : treference;const dref : treference);
      var
        aref: treference;
        bref: treference;
        tmpref : treference;
        dofix : boolean;
        hreg: TRegister;
      begin
        aref := sref;
        bref := dref;
        fixref(list,aref);
        fixref(list,bref);
<<<<<<< HEAD
<<<<<<< HEAD
        if TCGSize2OpSize[fromsize]<>TCGSize2OpSize[tosize] then
          begin
            { if we need to change the size then always use a temporary
              register }
            hreg:=getintregister(list,fromsize);
            list.concat(taicpu.op_ref_reg(A_MOVE,TCGSize2OpSize[fromsize],aref,hreg));
            sign_extend(list,fromsize,tosize,hreg);
            list.concat(taicpu.op_reg_ref(A_MOVE,TCGSize2OpSize[tosize],hreg,bref));
            exit;
          end;
        { Coldfire dislikes certain move combinations }
        if current_settings.cputype in cpu_coldfire then
          begin
            { TODO : move.b/w only allowed in newer coldfires... (ISA_B+) }

            dofix:=false;
            if { (d16,Ax) and (d8,Ax,Xi) }
                (
                  (aref.base<>NR_NO) and
                  (
                    (aref.index<>NR_NO) or
                    (aref.offset<>0)
                  )
                ) or
                { (xxx) }
                assigned(aref.symbol) then
              begin
                if aref.index<>NR_NO then
                  begin
                    dofix:={ (d16,Ax) and (d8,Ax,Xi) }
                           (
                             (bref.base<>NR_NO) and
                             (
                               (bref.index<>NR_NO) or
                               (bref.offset<>0)
                             )
                           ) or
                           { (xxx) }
                           assigned(bref.symbol);
                  end
                else
                  { offset <> 0, but no index }
                  begin
                    dofix:={ (d8,Ax,Xi) }
                           (
                             (bref.base<>NR_NO) and
                             (bref.index<>NR_NO)
                           ) or
                           { (xxx) }
                           assigned(bref.symbol);
                  end;
              end;

            if dofix then
              begin
                hreg:=getaddressregister(list);
                reference_reset_base(tmpref,hreg,0,0);
                list.concat(taicpu.op_ref_reg(A_LEA,S_L,aref,hreg));
                list.concat(taicpu.op_ref_ref(A_MOVE,TCGSize2OpSize[fromsize],tmpref,bref));
                exit;
              end;
          end;
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_load_ref_ref');
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef DEBUG_CHARLIE}
//        writeln('a_load_ref_ref');
{$endif DEBUG_CHARLIE}
>>>>>>> origin/fixes_2_2
        list.concat(taicpu.op_ref_ref(A_MOVE,TCGSize2OpSize[fromsize],aref,bref));
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
            sign_extend(list,fromsize,reg2);
          end;
      end;


    procedure tcg68k.a_load_ref_reg(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);
      var
       href : treference;
       hreg : tregister;
       size : tcgsize;
       opsize: topsize;
      begin
         href:=ref;
         fixref(list,href);
         if tcgsize2size[fromsize]<tcgsize2size[tosize] then
           size:=fromsize
         else
           size:=tosize;
         opsize:=TCGSize2OpSize[size];
         if isaddressregister(register) and not (opsize in [S_L]) then
           begin
             hreg:=getintregister(list,OS_ADDR);
             list.concat(taicpu.op_ref_reg(A_MOVE,opsize,href,hreg));
             sign_extend(list,size,hreg);
             a_load_reg_reg(list,OS_ADDR,OS_ADDR,hreg,register);
           end
         else 
           begin
             list.concat(taicpu.op_ref_reg(A_MOVE,opsize,href,register));
             { extend the value in the register }
             sign_extend(list, size, register);
           end;
      end;


    procedure tcg68k.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);
      var
        href : treference;
        hreg : tregister;
      begin
        href:=ref;
        fixref(list, href);
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
        instr:=taicpu.op_reg_reg(A_FMOVE,S_FX,reg1,reg2);
        add_move_instruction(instr);
        list.concat(instr);
      end;


    procedure tcg68k.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);
      var
        opsize : topsize;
        href : treference;
      begin
        opsize := tcgsize2opsize[fromsize];
        { extended is not supported, since it is not available on Coldfire }
        if opsize = S_FX then
          internalerror(20020729);
        href := ref;
        fixref(list,href);
        list.concat(taicpu.op_ref_reg(A_FMOVE,opsize,href,reg));
      end;

    procedure tcg68k.a_loadfpu_reg_ref(list: TAsmList; fromsize,tosize: tcgsize; reg: tregister; const ref: treference);
      var
        opsize : topsize;
        href : treference;
      begin
        opsize := tcgsize2opsize[tosize];
        { extended is not supported, since it is not available on Coldfire }
        if opsize = S_FX then
          internalerror(20020729);
        href := ref;
        fixref(list,href);
        list.concat(taicpu.op_reg_ref(A_FMOVE,opsize,reg,href));
      end;

    procedure tcg68k.a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const reg : tregister;const cgpara : tcgpara);
      var
        ref : treference;
      begin
        if use_push(cgpara) and (current_settings.fputype in [fpu_68881]) then
          begin
            cgpara.check_simple_location;
            { FIXME: 68k cg really needs to support 2 byte stack alignment, otherwise the "Extended"
              floating point type cannot work (KB) }
            reference_reset_base(ref, NR_STACK_POINTER_REG, 0, cgpara.alignment);
            ref.direction := dir_dec;
            list.concat(taicpu.op_reg_ref(A_FMOVE,tcgsize2opsize[cgpara.location^.size],reg,ref));
          end
        else
          inherited a_loadfpu_reg_cgpara(list,size,reg,cgpara);
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure tcg68k.a_loadfpu_ref_cgpara(list : TAsmList; size : tcgsize;const ref : treference;const cgpara : TCGPara);
      var
        href : treference;
        fref : treference;
        freg : tregister;
=======

    procedure tcg68k.a_loadmm_reg_cgpara(list: TAsmList; size: tcgsize; reg: tregister;const locpara : TCGPara;shuffle : pmmshuffle);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

    procedure tcg68k.a_loadmm_reg_cgpara(list: TAsmList; size: tcgsize; reg: tregister;const locpara : TCGPara;shuffle : pmmshuffle);
>>>>>>> graemeg/cpstrnew
=======

    procedure tcg68k.a_loadmm_reg_cgpara(list: TAsmList; size: tcgsize; reg: tregister;const locpara : TCGPara;shuffle : pmmshuffle);
>>>>>>> origin/cpstrnew
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
          if use_push(cgpara) and (current_settings.fputype in [fpu_68881]) then
            begin
              fref:=ref;
              fixref(list,fref);
              { fmove can't do <ea> -> <ea>, so move it to an fpreg first }
              freg:=getfpuregister(list,size);
              a_loadfpu_ref_reg(list,size,size,fref,freg);
              reference_reset_base(href, NR_STACK_POINTER_REG, 0, cgpara.alignment);
              href.direction := dir_dec;
              list.concat(taicpu.op_reg_ref(A_FMOVE,tcgsize2opsize[cgpara.location^.size],freg,href));
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
                if current_settings.cputype in cpu_coldfire then
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
                    if current_settings.cputype = cpu_mc68020 then
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
      begin
        optimize_op_const(size, op, a);
        opcode := topcg2tasmop[op];
        opsize := TCGSize2OpSize[size];

        { on ColdFire all arithmetic operations are only possible on 32bit }
        if ((current_settings.cputype in cpu_coldfire) and (opsize <> S_L)
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
          OP_ADD,
          OP_SUB :
            begin
              href:=ref;
              fixref(list,href);
              { add/sub works the same way, so have it unified here }
              if (a >= 1) and (a <= 8) then
                begin
                  if (op = OP_ADD) then
                    opcode:=A_ADDQ
                  else
                    opcode:=A_SUBQ;
                  list.concat(taicpu.op_const_ref(opcode, opsize, a, href));
                end
              else
                if not(current_settings.cputype in cpu_coldfire) then
                  list.concat(taicpu.op_const_ref(opcode, opsize, a, href))
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
                if (current_settings.cputype <> cpu_mc68020) and
                   (not (current_settings.cputype in cpu_coldfire)) then
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
        if ((current_settings.cputype in cpu_coldfire) and (opsize <> S_L)) then
          begin
            inherited;
            exit;
          end;

        case op of
          OP_ADD,
          OP_SUB :
            begin
              href:=ref;
              fixref(list,href);
              { areg -> ref arithmetic operations are impossible on 68k }
              hreg:=force_to_dataregister(list,size,reg);
              { add/sub works the same way, so have it unified here }
              list.concat(taicpu.op_reg_ref(opcode, opsize, hreg, href));
            end;
          else begin
//            list.concat(tai_comment.create(strpnew('a_op_reg_ref inherited')));
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
            if current_settings.cputype in cpu_coldfire{-[cpu_isa_b,cpu_isa_c]} then
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
        if (a = 0) then
          begin
            //list.concat(tai_comment.create(strpnew('a_cmp_const_ref_label with TST')));
            tmpref:=ref;
            fixref(list,tmpref);
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
         if (current_settings.cputype in cpu_coldfire-[cpu_isa_b,cpu_isa_c]) then
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
         ai := Taicpu.op_sym(A_JMP,S_NO,current_asmdata.RefAsmSymbol(s));
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
         ai := Taicpu.op_sym(A_BXX,S_NO,l);
         ai.SetCondition(flags_to_cond(f));
         ai.is_jmp := true;
         list.concat(ai);
       end;

    procedure tcg68k.g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister);
       var
         ai : taicpu;
         hreg : tregister;
         instr : taicpu;
       begin
          { move to a Dx register? }
          if (isaddressregister(reg)) then
            hreg:=getintregister(list,OS_INT)
          else
            hreg:=reg;

          ai:=Taicpu.Op_reg(A_Sxx,S_B,hreg);
          ai.SetCondition(flags_to_cond(f));
          list.concat(ai);

          { Scc stores a complete byte of 1s, but the compiler expects only one
            bit set, so ensure this is the case }
          list.concat(taicpu.op_const_reg(A_AND,S_L,1,hreg));

          if hreg<>reg then
            begin
              instr:=taicpu.op_reg_reg(A_MOVE,S_L,hreg,reg);
              add_move_instruction(instr);
              list.concat(instr);
            end;
       end;



    procedure tcg68k.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);

     var
         helpsize : longint;
         i : byte;
         hregister : tregister;
         iregister : tregister;
         jregister : tregister;
         hp1 : treference;
         hp2 : treference;
         hl : tasmlabel;
         srcref,dstref : treference;
      begin
         hregister := getintregister(list,OS_INT);

         { from 12 bytes movs is being used }
         if ((len<=8) or (not(cs_opt_size in current_settings.optimizerswitches) and (len<=12))) then
           begin
              srcref := source;
              dstref := dest;
              helpsize:=len div 4;
              { move a dword x times }
              for i:=1 to helpsize do
                begin
                   a_load_ref_reg(list,OS_INT,OS_INT,srcref,hregister);
                   a_load_reg_ref(list,OS_INT,OS_INT,hregister,dstref);
                   inc(srcref.offset,4);
                   inc(dstref.offset,4);
                   dec(len,4);
                end;
              { move a word }
              if len>1 then
                begin
                   a_load_ref_reg(list,OS_16,OS_16,srcref,hregister);
                   a_load_reg_ref(list,OS_16,OS_16,hregister,dstref);
                   inc(srcref.offset,2);
                   inc(dstref.offset,2);
                   dec(len,2);
                end;
              { move a single byte }
              if len>0 then
                begin
                   a_load_ref_reg(list,OS_8,OS_8,srcref,hregister);
                   a_load_reg_ref(list,OS_8,OS_8,hregister,dstref);
                end
           end
         else
           begin
              iregister:=getaddressregister(list);
              jregister:=getaddressregister(list);
              { reference for move (An)+,(An)+ }
              reference_reset(hp1,source.alignment);
              hp1.base := iregister;   { source register }
              hp1.direction := dir_inc;
              reference_reset(hp2,dest.alignment);
              hp2.base := jregister;
              hp2.direction := dir_inc;
              { iregister = source }
              { jregister = destination }

              a_loadaddr_ref_reg(list,source,iregister);
              a_loadaddr_ref_reg(list,dest,jregister);

              { double word move only on 68020+ machines }
              { because of possible alignment problems   }
              { use fast loop mode }
              if (current_settings.cputype=cpu_MC68020) then
                begin
                   //list.concat(tai_comment.create(strpnew('g_concatcopy tight copy loop 020+')));
                   helpsize := len - len mod 4;
                   len := len mod 4;
                   a_load_const_reg(list,OS_INT,(helpsize div 4)-1,hregister);
                   current_asmdata.getjumplabel(hl);
                   a_label(list,hl);
                   list.concat(taicpu.op_ref_ref(A_MOVE,S_L,hp1,hp2));
                   list.concat(taicpu.op_reg_sym(A_DBRA,S_L,hregister,hl));
                   if len > 1 then
                     begin
                        dec(len,2);
                        list.concat(taicpu.op_ref_ref(A_MOVE,S_W,hp1,hp2));
                     end;
                   if len = 1 then
                     list.concat(taicpu.op_ref_ref(A_MOVE,S_B,hp1,hp2));
                end
              else
                begin
                   { Fast 68010 loop mode with no possible alignment problems }
                   //list.concat(tai_comment.create(strpnew('g_concatcopy tight byte copy loop')));
                   a_load_const_reg(list,OS_INT,len - 1,hregister);
                   current_asmdata.getjumplabel(hl);
                   a_label(list,hl);
                   list.concat(taicpu.op_ref_ref(A_MOVE,S_B,hp1,hp2));
                   if current_settings.cputype in cpu_coldfire then
                     begin
                       { Coldfire does not support DBRA }
                       list.concat(taicpu.op_const_reg(A_SUBQ,S_L,1,hregister));
                       list.concat(taicpu.op_sym(A_BPL,S_NO,hl));
                     end
                   else
                     list.concat(taicpu.op_reg_sym(A_DBRA,S_L,hregister,hl));
                end;
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
                                          pasbool8,pasbool16,pasbool32,pasbool64]))) then
          cond:=C_VC
        else
          cond:=C_CC;
        ai:=Taicpu.Op_Sym(A_Bxx,S_NO,hl);
        ai.SetCondition(cond);
        ai.is_jmp:=true;
        list.concat(ai);

        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;

    procedure tcg68k.g_proc_entry(list: TAsmList; localsize: longint; nostackframe:boolean);
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        { Carl's original code used 2x MOVE instead of LINK when localsize = 0.
          However, a LINK seems faster than two moves on everything from 68000
          to '060, so the two move branch here was dropped. (KB) }
        if not nostackframe then
          begin
            { size can't be negative }
            if (localsize < 0) then
              internalerror(2006122601);
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef DEBUG_CHARLIE}
//        writeln('proc entry, localsize:',localsize);
{$endif DEBUG_CHARLIE}
	
        if not nostackframe then
          begin
	    if localsize<>0 then
	      begin
	        { size can't be negative }
		if (localsize < 0) then
		  internalerror(2006122601);
	      
                { Not to complicate the code generator too much, and since some }
                { of the systems only support this format, the localsize cannot }
                { exceed 32K in size.                                           }
                if (localsize > high(smallint)) then
                  CGMessage(cg_e_localsize_too_big);
                
                list.concat(taicpu.op_reg_const(A_LINK,S_W,NR_FRAME_POINTER_REG,-localsize));
	      end
	    else
	      begin
	        list.concat(taicpu.op_reg_const(A_LINK,S_W,NR_FRAME_POINTER_REG,0));
(*		
		{ FIXME! - Carl's original code uses this method. However,
		  according to the 68060 users manual, a LINK is faster than
		  two moves. So, use a link in #0 case too, for now. I'm not
		  really sure tho', that LINK supports #0 disposition, but i
		  see no reason why it shouldn't support it. (KB) }
		  
	        { when localsize = 0, use two moves, instead of link }
		r:=NR_FRAME_POINTER_REG;
		rsp:=NR_STACK_POINTER_REG;
		
	        reference_reset_base(ref,NR_STACK_POINTER_REG,0);
		ref.direction:=dir_dec;
                list.concat(taicpu.op_reg_ref(A_MOVE,S_L,r,ref));
                list.concat(taicpu.op_reg_reg(A_MOVE,S_L,rsp,r));
		*)
	      end;
          end;
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
            list.concat(taicpu.op_reg(A_UNLK,S_NO,NR_FRAME_POINTER_REG));

            { if parasize is less than zero here, we probably have a cdecl function.
              According to the info here: http://www.makestuff.eu/wordpress/gcc-68000-abi/
              68k GCC uses two different methods to free the stack, depending if the target
              architecture supports RTD or not, and one does callee side, the other does
              caller side free, which looks like a PITA to support. We have to figure this 
              out later. More info welcomed. (KB) }

            if (parasize > 0) and not (current_procinfo.procdef.proccalloption in clearstack_pocalls) then
              begin
                if current_settings.cputype=cpu_mc68020 then
                  list.concat(taicpu.op_const(A_RTD,S_NO,parasize))
                else
                  begin
                    { We must pull the PC Counter from the stack, before  }
                    { restoring the stack pointer, otherwise the PC would }
                    { point to nowhere!                                   }

                    { Instead of doing a slow copy of the return address while trying    }
                    { to feed it to the RTS instruction, load the PC to A0 (scratch reg) }
                    { then free up the stack allocated for paras, then use a JMP (A0) to }
                    { return to the caller with the paras freed. (KB) }

                    hregister:=NR_A0;
                    cg.a_reg_alloc(list,hregister);
                    reference_reset_base(ref,NR_STACK_POINTER_REG,0,4);
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
                         reference_reset_base(ref2,NR_STACK_POINTER_REG,parasize,4);
                         list.concat(taicpu.op_ref_reg(A_LEA,S_NO,ref2,r));
                       end;

                    reference_reset_base(ref,hregister,0,4);
                    list.concat(taicpu.op_ref(A_JMP,S_NO,ref));
                  end;
=======
            localsize := current_procinfo.calc_stackframe_size;
{$ifdef DEBUG_CHARLIE}
//            writeln('proc exit with stackframe, size:',localsize,' parasize:',parasize);
{$endif DEBUG_CHARLIE}
            list.concat(taicpu.op_reg(A_UNLK,S_NO,NR_FRAME_POINTER_REG));
	    parasize := parasize - target_info.first_parm_offset; { i'm still not 100% confident that this is
	                                                            correct here, but at least it looks less
								    hacky, and makes some sense (KB) }
            if (parasize<>0) then
              begin
                { only 68020+ supports RTD, so this needs another code path
                  for 68000 and Coldfire (KB) }
{$WARNING 68020+ only code generation, without fallback}
                list.concat(taicpu.op_const(A_RTD,S_NO,parasize));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              end
            else
              list.concat(taicpu.op_none(A_RTS,S_NO));
          end
        else
          begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifdef DEBUG_CHARLIE}
//            writeln('proc exit, no stackframe');
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef DEBUG_CHARLIE}
//            writeln('proc exit, no stackframe');
{$endif DEBUG_CHARLIE}
>>>>>>> origin/fixes_2_2
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


<<<<<<< HEAD
<<<<<<< HEAD
    procedure tcg68k.g_save_registers(list:TAsmList);
=======
    procedure Tcg68k.g_save_registers(list:TAsmList);
>>>>>>> graemeg/fixes_2_2
=======
    procedure Tcg68k.g_save_registers(list:TAsmList);
>>>>>>> origin/fixes_2_2
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

        { calculate temp. size }
        size:=0;
        fsize:=0;
        hreg:=NR_NO;
        hfreg:=NR_NO;
        for r:=low(saved_standard_registers) to high(saved_standard_registers) do
          if saved_standard_registers[r] in rg[R_INTREGISTER].used_in_proc then
            begin
              hreg:=newreg(R_INTREGISTER,saved_address_registers[r],R_SUBWHOLE);
              inc(size,sizeof(aint));
              dataregs:=dataregs + [saved_standard_registers[r]];
            end;
        if uses_registers(R_ADDRESSREGISTER) then
          for r:=low(saved_address_registers) to high(saved_address_registers) do
            if saved_address_registers[r] in rg[R_ADDRESSREGISTER].used_in_proc then
              begin
                hreg:=newreg(R_ADDRESSREGISTER,saved_address_registers[r],R_SUBWHOLE);
                inc(size,sizeof(aint));
                addrregs:=addrregs + [saved_address_registers[r]];
              end;
        if uses_registers(R_FPUREGISTER) then
          for r:=low(saved_fpu_registers) to high(saved_fpu_registers) do
            if saved_fpu_registers[r] in rg[R_FPUREGISTER].used_in_proc then
              begin
                hfreg:=newreg(R_FPUREGISTER,saved_fpu_registers[r],R_SUBNONE);
                inc(fsize,12{sizeof(extended)});
                fpuregs:=fpuregs + [saved_fpu_registers[r]];
              end;

<<<<<<< HEAD
<<<<<<< HEAD
        { 68k has no MM registers }
        if uses_registers(R_MMREGISTER) then
          internalerror(2014030201);
=======
=======
>>>>>>> origin/fixes_2_2
    procedure Tcg68k.g_restore_registers(list:TAsmList);
      var
        torestore : tcpuregisterset;
        r:Tregister;
        ref : treference;
      begin
      {!!!!!!!!
        torestore:=std_saved_registers;
        { should be intersected with used regs, no ? }
        torestore:=torestore*(rg[R_INTREGISTER].used_in_proc+rg[R_ADDRESSREGISTER].used_in_proc);
        reference_reset_base(ref,NR_STACK_POINTER_REG,0);
        ref.direction:=dir_inc;
        if torestore<>[] then
          list.concat(taicpu.op_ref_regset(A_MOVEM,S_L,ref,torestore));
      }
      end;
>>>>>>> graemeg/fixes_2_2

        if (size+fsize) > 0 then
          begin
            tg.GetTemp(list,size+fsize,sizeof(aint),tt_noreuse,current_procinfo.save_regs_ref);
            include(current_procinfo.flags,pi_has_saved_regs);

            { Copy registers to temp }
            { NOTE: virtual registers allocated here won't be translated --> no higher-level stuff. }
            href:=current_procinfo.save_regs_ref;
            if (href.offset<low(smallint)) and (current_settings.cputype in cpu_coldfire) then
              begin
<<<<<<< HEAD
                list.concat(taicpu.op_reg_reg(A_MOVE,S_L,href.base,NR_A0));
                list.concat(taicpu.op_const_reg(A_ADDA,S_L,href.offset,NR_A0));
                reference_reset_base(href,NR_A0,0,sizeof(pint));
=======
                if (isaddressregister(reg)) then
                   internalerror(20020729);
                if (current_settings.cputype = cpu_MC68000) then
                  begin
                    list.concat(taicpu.op_reg(A_EXT,S_W,reg));
                    list.concat(taicpu.op_reg(A_EXT,S_L,reg));
                  end
                else
                  begin
//    		    list.concat(tai_comment.create(strpnew('sign extend byte')));
                    list.concat(taicpu.op_reg(A_EXTB,S_L,reg));
                  end;
>>>>>>> graemeg/fixes_2_2
              end;

            if size > 0 then
              if size = sizeof(aint) then
                list.concat(taicpu.op_reg_ref(A_MOVE,S_L,hreg,href))
              else
                list.concat(taicpu.op_regset_ref(A_MOVEM,S_L,dataregs,addrregs,[],href));

            if fsize > 0 then
              begin
<<<<<<< HEAD
                { size is always longword aligned, while fsize is not }
                inc(href.offset,size);
                if fsize = 12{sizeof(extended)} then
                  list.concat(taicpu.op_reg_ref(A_FMOVE,S_FX,hfreg,href))
                else
                  list.concat(taicpu.op_regset_ref(A_FMOVEM,S_FX,[],[],fpuregs,href));
=======
                if (isaddressregister(reg)) then
                   internalerror(20020729);
//    		list.concat(tai_comment.create(strpnew('sign extend word')));
                list.concat(taicpu.op_reg(A_EXT,S_L,reg));
>>>>>>> graemeg/fixes_2_2
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
      begin
        { see the remark about buggy GNU AS versions in g_save_registers() (KB) }
        dataregs:=[];
        addrregs:=[];
        fpuregs:=[];

        if not(pi_has_saved_regs in current_procinfo.flags) then
          exit;
        { Copy registers from temp }
        size:=0;
        fsize:=0;
        hreg:=NR_NO;
        hfreg:=NR_NO;
        for r:=low(saved_standard_registers) to high(saved_standard_registers) do
          if saved_standard_registers[r] in rg[R_INTREGISTER].used_in_proc then
            begin
              inc(size,sizeof(aint));
              hreg:=newreg(R_INTREGISTER,saved_standard_registers[r],R_SUBWHOLE);
              { Allocate register so the optimizer does not remove the load }
              a_reg_alloc(list,hreg);
              dataregs:=dataregs + [saved_standard_registers[r]];
            end;

        if uses_registers(R_ADDRESSREGISTER) then
          for r:=low(saved_address_registers) to high(saved_address_registers) do
            if saved_address_registers[r] in rg[R_ADDRESSREGISTER].used_in_proc then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                inc(size,sizeof(aint));
                hreg:=newreg(R_ADDRESSREGISTER,saved_address_registers[r],R_SUBWHOLE);
                { Allocate register so the optimizer does not remove the load }
                a_reg_alloc(list,hreg);
                addrregs:=addrregs + [saved_address_registers[r]];
=======
=======
>>>>>>> origin/fixes_2_2
//    		list.concat(tai_comment.create(strpnew('zero extend byte')));
                list.concat(taicpu.op_const_reg(A_AND,S_L,$FF,reg));
>>>>>>> graemeg/fixes_2_2
              end;

        if uses_registers(R_FPUREGISTER) then
          for r:=low(saved_fpu_registers) to high(saved_fpu_registers) do
            if saved_fpu_registers[r] in rg[R_FPUREGISTER].used_in_proc then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                inc(fsize,12{sizeof(extended)});
                hfreg:=newreg(R_FPUREGISTER,saved_fpu_registers[r],R_SUBNONE);
                { Allocate register so the optimizer does not remove the load }
                a_reg_alloc(list,hfreg);
                fpuregs:=fpuregs + [saved_fpu_registers[r]];
=======
=======
>>>>>>> origin/fixes_2_2
//    		list.concat(tai_comment.create(strpnew('zero extend word')));
                list.concat(taicpu.op_const_reg(A_AND,S_L,$FFFF,reg));
>>>>>>> graemeg/fixes_2_2
              end;

        { 68k has no MM registers }
        if uses_registers(R_MMREGISTER) then
          internalerror(2014030202);

        { Restore registers from temp }
        href:=current_procinfo.save_regs_ref;
        if (href.offset<low(smallint)) and (current_settings.cputype in cpu_coldfire) then
          begin
            list.concat(taicpu.op_reg_reg(A_MOVE,S_L,href.base,NR_A0));
            list.concat(taicpu.op_const_reg(A_ADDA,S_L,href.offset,NR_A0));
            reference_reset_base(href,NR_A0,0,sizeof(pint));
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
            if fsize = 12{sizeof(extended)} then
              list.concat(taicpu.op_ref_reg(A_FMOVE,S_FX,href,hfreg))
            else
              list.concat(taicpu.op_ref_regset(A_FMOVEM,S_FX,href,[],[],fpuregs));
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
                  //list.concat(tai_comment.create(strpnew('zero extend byte')));
                  list.concat(taicpu.op_const_reg(A_AND,S_L,$FFFF,reg));
                end;
            end;
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

<<<<<<< HEAD

    procedure tcg68k.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
=======
        procedure op_onr11methodaddr;
        var
          href : treference;
        begin
          if (procdef.extnumber=$ffff) then
            Internalerror(200006139);
          { call/jmp  vmtoffs(%eax) ; method offs }
          reference_reset_base(href,NR_R11,tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber));
          if not((longint(href.offset) >= low(smallint)) and
                 (longint(href.offset) <= high(smallint))) then
            begin
              list.concat(taicpu.op_reg_reg_const(A_ADDIS,NR_R11,NR_R11,
                smallint((href.offset shr 16)+ord(smallint(href.offset and $ffff) < 0))));
              href.offset := smallint(href.offset and $ffff);
            end;
          list.concat(taicpu.op_reg_ref(A_LWZ,NR_R11,href));
          list.concat(taicpu.op_reg(A_MTCTR,NR_R11));
          list.concat(taicpu.op_none(A_BCTR));
        end;
}
>>>>>>> graemeg/cpstrnew
      var
        hsym : tsym;
        href : treference;
        paraloc : Pcgparalocation;
      begin
<<<<<<< HEAD
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
                    reference_reset_base(href,reference.index,reference.offset+sizeof(pint),sizeof(pint));
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
           create_smartlink or
           (procdef.owner.defowner.owner.symtabletype=globalsymtable) then
          make_global:=true;

        if make_global then
          List.concat(Tai_symbol.Createname_global(labelname,AT_FUNCTION,0))
        else
          List.concat(Tai_symbol.Createname(labelname,AT_FUNCTION,0));
>>>>>>> graemeg/cpstrnew


    procedure tcg68k.g_stackpointer_alloc(list : TAsmList;localsize : longint);
      begin
        list.concat(taicpu.op_const_reg(A_SUB,S_L,localsize,NR_STACK_POINTER_REG));
      end;


    procedure tcg68k.check_register_size(size:tcgsize;reg:tregister);
      begin
        if TCGSize2OpSize[size]<>TCGSize2OpSize[reg_cgsize(reg)] then
          internalerror(201512131);
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
        end; { end case }
      end;


    procedure tcg64f68k.a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);
      var
        tempref : treference;
      begin
        case op of
          OP_NEG,OP_NOT:
            begin
              a_load64_ref_reg(list,ref,reg);
              a_op64_reg_reg(list,op,size,reg,reg);
            end;

          OP_AND,OP_OR:
            begin
              tempref:=ref;
              tcg68k(cg).fixref(list,tempref);
              inc(tempref.offset,4);
              list.concat(taicpu.op_ref_reg(topcg2tasmop[op],S_L,tempref,reg.reglo));
              dec(tempref.offset,4);
              list.concat(taicpu.op_ref_reg(topcg2tasmop[op],S_L,tempref,reg.reghi));
            end;
        else
          { XOR does not allow reference for source; ADD/SUB do not allow reference for
            high dword, although low dword can still be handled directly. }
          inherited a_op64_ref_reg(list,op,size,ref,reg);
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
              cg.a_load_const_reg(list,OS_S32,longint(highvalue),hreg);
              { don't use cg.a_op_const_reg() here, because a possible optimized
                ADDQ/SUBQ wouldn't set the eXtend bit }
              list.concat(taicpu.op_const_reg(opcode,S_L,lowvalue,regdst.reglo));
              list.concat(taicpu.op_reg_reg(xopcode,S_L,hreg,regdst.reghi));
            end;
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg(list,op,OS_S32,longint(lowvalue),regdst.reglo);
              cg.a_op_const_reg(list,op,OS_S32,longint(highvalue),regdst.reghi);
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
        end; { end case }
      end;


procedure create_codegen;
  begin
    cg := tcg68k.create;
    cg64 :=tcg64f68k.create;
  end;
  
  
procedure create_codegen;
  begin
    cg := tcg68k.create;
    cg64 :=tcg64f68k.create;
  end;
  
  
procedure create_codegen;
  begin
    cg := tcg68k.create;
    cg64 :=tcg64f68k.create;
  end;
  
  
procedure create_codegen;
  begin
    cg := tcg68k.create;
    cg64 :=tcg64f68k.create;
  end;
  
  
procedure create_codegen;
  begin
    cg := tcg68k.create;
    cg64 :=tcg64f68k.create;
  end;

end.
