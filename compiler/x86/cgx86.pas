{
    Copyright (c) 1998-2005 by Florian Klaempfl

    This unit implements the common parts of the code generator for the i386 and the x86-64.

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
{ This unit implements the common parts of the code generator for the i386 and the x86-64.
}
unit cgx86;

{$i fpcdefs.inc}

  interface

    uses
       globtype,
       cgbase,cgutils,cgobj,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpuinfo,rgobj,rgx86,rgcpu,
       symconst,symtype,symdef;

    type

      { tcgx86 }

      tcgx86 = class(tcg)
        rgfpu   : Trgx86fpu;
        procedure done_register_allocators;override;

        function getfpuregister(list:TAsmList;size:Tcgsize):Tregister;override;
        function getmmxregister(list:TAsmList):Tregister;
        function getmmregister(list:TAsmList;size:Tcgsize):Tregister;override;

        procedure getcpuregister(list:TAsmList;r:Tregister);override;
        procedure ungetcpuregister(list:TAsmList;r:Tregister);override;
        procedure alloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);override;
        procedure dealloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);override;
        function  uses_registers(rt:Tregistertype):boolean;override;
        procedure add_reg_instruction(instr:Tai;r:tregister);override;
        procedure dec_fpu_stack;
        procedure inc_fpu_stack;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_name_near(list : TAsmList;const s : string; weak: boolean);
        procedure a_call_name_static(list : TAsmList;const s : string);override;
        procedure a_call_name_static_near(list : TAsmList;const s : string);
        procedure a_call_reg(list : TAsmList;reg : tregister);override;
        procedure a_call_reg_near(list : TAsmList;reg : tregister);

        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister); override;
        procedure a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;
        procedure a_op_ref_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference; reg: TRegister); override;
        procedure a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize;reg: TRegister; const ref: TReference); override;

{$ifndef i8086}
        procedure a_op_const_reg_reg(list : TAsmList; op : Topcg; size : Tcgsize; a : tcgint; src,dst : Tregister); override;
        procedure a_op_reg_reg_reg(list : TAsmList; op : TOpCg; size : tcgsize; src1,src2,dst : tregister); override;
{$endif not i8086}

        { move instructions }
        procedure a_load_const_reg(list : TAsmList; tosize: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_const_ref(list : TAsmList; tosize: tcgsize; a : tcgint;const ref : treference);override;
        procedure a_load_reg_ref(list : TAsmList;fromsize,tosize: tcgsize; reg : tregister;const ref : treference);override;
        procedure a_load_ref_reg(list : TAsmList;fromsize,tosize: tcgsize;const ref : treference;reg : tregister);override;
        procedure a_load_reg_reg(list : TAsmList;fromsize,tosize: tcgsize;reg1,reg2 : tregister);override;
        procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);override;

        { bit scan instructions }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: TCGSize; src, dst: TRegister); override;
=======
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister); override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister); override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister); override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister); override;
>>>>>>> origin/cpstrnew

        { fpu move instructions }
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;

        { vector register move instructions }
        procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle); override;
        procedure a_opmm_ref_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src,dst: tregister;shuffle : pmmshuffle);override;
        procedure a_opmm_ref_reg_reg(list : TAsmList;Op : TOpCG;size : tcgsize;const ref : treference;src,dst : tregister;shuffle : pmmshuffle);override;
        procedure a_opmm_reg_reg_reg(list : TAsmList;Op : TOpCG;size : tcgsize;src1,src2,dst : tregister;shuffle : pmmshuffle);override;

        {  comparison operations }
        procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
          l : tasmlabel);override;
        procedure a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference;
          l : tasmlabel);override;
        procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel); override;
        procedure a_cmp_ref_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;const ref: treference; reg : tregister; l : tasmlabel); override;
        procedure a_cmp_reg_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg : tregister; const ref: treference; l : tasmlabel); override;

        procedure a_jmp_name(list : TAsmList;const s : string);override;
        procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;
        procedure a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel); override;

        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister); override;
        procedure g_flags2ref(list: TAsmList; size: TCgSize; const f: tresflags; const ref: TReference); override;

        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;

        { entry/exit code helpers }
        procedure g_profilecode(list : TAsmList);override;
        procedure g_stackpointer_alloc(list : TAsmList;localsize : longint);override;
        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_save_registers(list: TAsmList); override;
        procedure g_restore_registers(list: TAsmList); override;

        procedure g_overflowcheck(list: TAsmList; const l:tlocation;def:tdef);override;

        procedure g_external_wrapper(list: TAsmList; procdef: tprocdef; const externalname: string); override;

        procedure make_simple_ref(list:TAsmList;var ref: treference);

        function get_darwin_call_stub(const s: string; weak: boolean): tasmsymbol;

        procedure generate_leave(list : TAsmList);
      protected
        procedure a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
        procedure check_register_size(size:tcgsize;reg:tregister);

        procedure opmm_loc_reg(list: TAsmList; Op: TOpCG; size : tcgsize;loc : tlocation;dst: tregister; shuffle : pmmshuffle);
        procedure opmm_loc_reg_reg(list : TAsmList;Op : TOpCG;size : tcgsize;loc : tlocation;src,dst : tregister;shuffle : pmmshuffle);

        procedure sizes2load(s1,s2 : tcgsize;var op: tasmop; var s3: topsize);

        procedure floatload(list: TAsmList; t : tcgsize;const ref : treference);
        procedure floatstore(list: TAsmList; t : tcgsize;const ref : treference);
        procedure floatloadops(t : tcgsize;var op : tasmop;var s : topsize);
        procedure floatstoreops(t : tcgsize;var op : tasmop;var s : topsize);

        procedure internal_restore_regs(list: TAsmList; use_pop: boolean);
      end;

   const
{$if defined(x86_64)}
      TCGSize2OpSize: Array[tcgsize] of topsize =
        (S_NO,S_B,S_W,S_L,S_Q,S_XMM,S_B,S_W,S_L,S_Q,S_XMM,
         S_FS,S_FL,S_FX,S_IQ,S_FXX,
         S_NO,S_NO,S_NO,S_MD,S_XMM,S_YMM,
         S_NO,S_NO,S_NO,S_NO,S_XMM,S_YMM);
{$elseif defined(i386)}
      TCGSize2OpSize: Array[tcgsize] of topsize =
        (S_NO,S_B,S_W,S_L,S_L,S_T,S_B,S_W,S_L,S_L,S_L,
         S_FS,S_FL,S_FX,S_IQ,S_FXX,
         S_NO,S_NO,S_NO,S_MD,S_XMM,S_YMM,
         S_NO,S_NO,S_NO,S_NO,S_XMM,S_YMM);
{$elseif defined(i8086)}
      TCGSize2OpSize: Array[tcgsize] of topsize =
        (S_NO,S_B,S_W,S_W,S_W,S_T,S_B,S_W,S_W,S_W,S_W,
         S_FS,S_FL,S_FX,S_IQ,S_FXX,
         S_NO,S_NO,S_NO,S_MD,S_XMM,S_YMM,
         S_NO,S_NO,S_NO,S_NO,S_XMM,S_YMM);
{$endif}

{$ifndef NOTARGETWIN}
      winstackpagesize = 4096;
{$endif NOTARGETWIN}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function UseAVX: boolean;

    function UseIncDec: boolean;

    { returns true, if the compiler should use leave instead of mov/pop }
    function UseLeave: boolean;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  implementation

    uses
       globals,verbose,systems,cutils,
       defutil,paramgr,procinfo,
       tgobj,ncgutil,
<<<<<<< HEAD
<<<<<<< HEAD
       fmodule,symsym,symcpu;

    function UseAVX: boolean;
      begin
        Result:=(current_settings.fputype in fpu_avx_instructionsets) {$ifndef i8086}or (CPUX86_HAS_AVXUNIT in cpu_capabilities[current_settings.cputype]){$endif i8086};
      end;


    { modern CPUs prefer add/sub over inc/dec because add/sub break instructions dependencies on flags
      because they modify all flags }
    function UseIncDec: boolean;
      begin
{$if defined(x86_64)}
        Result:=cs_opt_size in current_settings.optimizerswitches;
{$elseif defined(i386)}
        Result:=(cs_opt_size in current_settings.optimizerswitches) or (current_settings.cputype in [cpu_386]);
{$elseif defined(i8086)}
        Result:=(cs_opt_size in current_settings.optimizerswitches) or (current_settings.cputype in [cpu_8086..cpu_386]);
{$endif}
      end;


    function UseLeave: boolean;
      begin
{$if defined(x86_64)}
        { Modern processors should be happy with mov;pop, maybe except older AMDs }
        Result:=cs_opt_size in current_settings.optimizerswitches;
{$elseif defined(i386)}
        Result:=(cs_opt_size in current_settings.optimizerswitches) or (current_settings.optimizecputype<cpu_Pentium2);
{$elseif defined(i8086)}
        Result:=current_settings.cputype>=cpu_186;
{$endif}
      end;
=======
=======
>>>>>>> origin/fixes_2_2
       fmodule;
>>>>>>> graemeg/fixes_2_2

    const
      TOpCG2AsmOp: Array[topcg] of TAsmOp = (A_NONE,A_MOV,A_ADD,A_AND,A_DIV,
                            A_IDIV,A_IMUL,A_MUL,A_NEG,A_NOT,A_OR,
                            A_SAR,A_SHL,A_SHR,A_SUB,A_XOR,A_ROL,A_ROR);

      TOpCmp2AsmCond: Array[topcmp] of TAsmCond = (C_NONE,
          C_E,C_G,C_L,C_GE,C_LE,C_NE,C_BE,C_B,C_AE,C_A);

    procedure Tcgx86.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_MMREGISTER].free;
        rg[R_MMXREGISTER].free;
        rgfpu.free;
        inherited done_register_allocators;
      end;


    function Tcgx86.getfpuregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        result:=rgfpu.getregisterfpu(list);
      end;


    function Tcgx86.getmmxregister(list:TAsmList):Tregister;
      begin
        if not assigned(rg[R_MMXREGISTER]) then
          internalerror(2003121214);
        result:=rg[R_MMXREGISTER].getregister(list,R_SUBNONE);
      end;


    function Tcgx86.getmmregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if not assigned(rg[R_MMREGISTER]) then
          internalerror(2003121234);
        case size of
          OS_F64:
            result:=rg[R_MMREGISTER].getregister(list,R_SUBMMD);
          OS_F32:
            result:=rg[R_MMREGISTER].getregister(list,R_SUBMMS);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          OS_M64:
            result:=rg[R_MMREGISTER].getregister(list,R_SUBQ);
=======
          OS_M64,
>>>>>>> graemeg/cpstrnew
=======
          OS_M64,
>>>>>>> graemeg/cpstrnew
=======
          OS_M64,
>>>>>>> graemeg/cpstrnew
=======
          OS_M64,
>>>>>>> origin/cpstrnew
          OS_M128:
            result:=rg[R_MMREGISTER].getregister(list,R_SUBMMWHOLE);
          else
            internalerror(200506041);
        end;
      end;


    procedure Tcgx86.getcpuregister(list:TAsmList;r:Tregister);
      begin
        if getregtype(r)=R_FPUREGISTER then
          internalerror(2003121210)
        else
          inherited getcpuregister(list,r);
      end;


    procedure tcgx86.ungetcpuregister(list:TAsmList;r:Tregister);
      begin
        if getregtype(r)=R_FPUREGISTER then
          rgfpu.ungetregisterfpu(list,r)
        else
          inherited ungetcpuregister(list,r);
      end;


    procedure Tcgx86.alloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);
      begin
        if rt<>R_FPUREGISTER then
          inherited alloccpuregisters(list,rt,r);
      end;


    procedure Tcgx86.dealloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);
      begin
        if rt<>R_FPUREGISTER then
          inherited dealloccpuregisters(list,rt,r);
      end;


    function  Tcgx86.uses_registers(rt:Tregistertype):boolean;
      begin
        if rt=R_FPUREGISTER then
          result:=false
        else
          result:=inherited uses_registers(rt);
      end;


    procedure tcgx86.add_reg_instruction(instr:Tai;r:tregister);
      begin
        if getregtype(r)<>R_FPUREGISTER then
          inherited add_reg_instruction(instr,r);
      end;


    procedure tcgx86.dec_fpu_stack;
      begin
        if rgfpu.fpuvaroffset<=0 then
          internalerror(200604201);
        dec(rgfpu.fpuvaroffset);
      end;


    procedure tcgx86.inc_fpu_stack;
      begin
        if rgfpu.fpuvaroffset>=7 then
          internalerror(2012062901);
        inc(rgfpu.fpuvaroffset);
      end;


{****************************************************************************
                       This is private property, keep out! :)
****************************************************************************}

    procedure tcgx86.sizes2load(s1,s2 : tcgsize; var op: tasmop; var s3: topsize);

       begin
         { ensure to have always valid sizes }
         if s1=OS_NO then
           s1:=s2;
         if s2=OS_NO then
           s2:=s1;
         case s2 of
           OS_8,OS_S8 :
             if S1 in [OS_8,OS_S8] then
               s3 := S_B
             else
               internalerror(200109221);
           OS_16,OS_S16:
             case s1 of
               OS_8,OS_S8:
                 s3 := S_BW;
               OS_16,OS_S16:
                 s3 := S_W;
               else
                 internalerror(200109222);
             end;
           OS_32,OS_S32:
             case s1 of
               OS_8,OS_S8:
                 s3 := S_BL;
               OS_16,OS_S16:
                 s3 := S_WL;
               OS_32,OS_S32:
                 s3 := S_L;
               else
                 internalerror(200109223);
             end;
{$ifdef x86_64}
           OS_64,OS_S64:
             case s1 of
               OS_8:
                 s3 := S_BL;
               OS_S8:
                 s3 := S_BQ;
               OS_16:
                 s3 := S_WL;
               OS_S16:
                 s3 := S_WQ;
               OS_32:
                 s3 := S_L;
               OS_S32:
                 s3 := S_LQ;
               OS_64,OS_S64:
                 s3 := S_Q;
               else
                 internalerror(200304302);
             end;
{$endif x86_64}
           else
             internalerror(200109227);
         end;
         if s3 in [S_B,S_W,S_L,S_Q] then
           op := A_MOV
         else if s1 in [OS_8,OS_16,OS_32,OS_64] then
           op := A_MOVZX
         else
{$ifdef x86_64}
           if s3 in [S_LQ] then
             op := A_MOVSXD
         else
{$endif x86_64}
           op := A_MOVSX;
       end;


    procedure tcgx86.make_simple_ref(list:TAsmList;var ref: treference);
      var
        hreg : tregister;
        href : treference;
{$ifndef x86_64}
        add_hreg: boolean;
{$endif not  x86_64}
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        hreg:=NR_NO;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        { make_simple_ref() may have already been called earlier, and in that
          case make sure we don't perform the PIC-simplifications twice }
        if (ref.refaddr in [addr_pic,addr_pic_no_got]) then
          exit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(x86_64)}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef x86_64}
>>>>>>> graemeg/cpstrnew
        { Only 32bit is allowed }
        { Note that this isn't entirely correct: for RIP-relative targets/memory models,
          it is actually (offset+@symbol-RIP) that should fit into 32 bits. Since two last
          members aren't known until link time, ABIs place very pessimistic limits
          on offset values, e.g. SysV AMD64 allows +/-$1000000 (16 megabytes) }
        if ((ref.offset<low(longint)) or (ref.offset>high(longint))) or
           { absolute address is not a common thing in x64, but nevertheless a possible one }
           ((ref.base=NR_NO) and (ref.index=NR_NO) and (ref.symbol=nil)) then
          begin
            { Load constant value to register }
            hreg:=GetAddressRegister(list);
            list.concat(taicpu.op_const_reg(A_MOV,S_Q,ref.offset,hreg));
            ref.offset:=0;
            {if assigned(ref.symbol) then
              begin
                list.concat(taicpu.op_sym_ofs_reg(A_ADD,S_Q,ref.symbol,0,hreg));
                ref.symbol:=nil;
              end;}
            { Add register to reference }
            if ref.base=NR_NO then
              ref.base:=hreg
            else if ref.index=NR_NO then
              ref.index:=hreg
            else
              begin
                { don't use add, as the flags may contain a value }
                reference_reset_base(href,ref.base,0,8);
                href.index:=hreg;
                if ref.scalefactor<>0 then
                  begin
                    reference_reset_base(href,ref.base,0,8);
                    href.index:=hreg;
                    list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,hreg));
                    ref.base:=hreg;
                  end
                else
                  begin
                    reference_reset_base(href,ref.index,0,8);
                    href.index:=hreg;
                    list.concat(taicpu.op_reg_reg(A_ADD,S_Q,ref.index,hreg));
                    ref.index:=hreg;
                  end;
               end;
          end;

        if assigned(ref.symbol) then
          begin
            if cs_create_pic in current_settings.moduleswitches then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                { Local symbols must not be accessed via the GOT }
                if (ref.symbol.bind=AB_LOCAL) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                { Local data symbols must not be accessed via the GOT on
                  darwin/x86_64 under certain circumstances (and do not
                  have to be in other cases); however, linux/x86_64 does
                  require it; don't know about others, so do use GOT for
                  safety reasons
                }
                if (ref.symbol.bind=AB_LOCAL) and
                   (ref.symbol.typ=AT_DATA) then
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
                  begin
                    { unfortunately, RIP-based addresses don't support an index }
                    if (ref.base<>NR_NO) or
                       (ref.index<>NR_NO) then
                      begin
                        reference_reset_symbol(href,ref.symbol,0,ref.alignment);
                        hreg:=getaddressregister(list);
                        href.refaddr:=addr_pic_no_got;
                        href.base:=NR_RIP;
                        list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,hreg));
                        ref.symbol:=nil;
                      end
                    else
                      begin
                        ref.refaddr:=addr_pic_no_got;
                        hreg:=NR_NO;
                        ref.base:=NR_RIP;
                      end;
                  end
                else
                  begin
                    reference_reset_symbol(href,ref.symbol,0,ref.alignment);
                    hreg:=getaddressregister(list);
                    href.refaddr:=addr_pic;
                    href.base:=NR_RIP;
                    list.concat(taicpu.op_ref_reg(A_MOV,S_Q,href,hreg));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
                    ref.symbol:=nil;
                  end;

                if ref.base=NR_NO then
                  ref.base:=hreg
                else if ref.index=NR_NO then
                  begin
                    ref.index:=hreg;
                    ref.scalefactor:=1;
                  end
                else
                  begin
                    { don't use add, as the flags may contain a value }
                    reference_reset_base(href,ref.base,0,8);
                    href.index:=hreg;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    ref.base:=getaddressregister(list);
                    list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,ref.base));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,hreg));
                    ref.base:=hreg;
>>>>>>> graemeg/cpstrnew
                  end;
              end
            else
              { Always use RIP relative symbol addressing for Windows and Darwin targets. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              if (target_info.system in (systems_all_windows+[system_x86_64_darwin,system_x86_64_iphonesim])) and (ref.base<>NR_RIP) then
=======
              if (target_info.system in (systems_all_windows+[system_x86_64_darwin])) and (ref.base<>NR_RIP) then
>>>>>>> graemeg/cpstrnew
=======
              if (target_info.system in (systems_all_windows+[system_x86_64_darwin])) and (ref.base<>NR_RIP) then
>>>>>>> graemeg/cpstrnew
=======
              if (target_info.system in (systems_all_windows+[system_x86_64_darwin])) and (ref.base<>NR_RIP) then
>>>>>>> graemeg/cpstrnew
=======
              if (target_info.system in (systems_all_windows+[system_x86_64_darwin])) and (ref.base<>NR_RIP) then
>>>>>>> origin/cpstrnew
                begin
                  if (ref.refaddr=addr_no) and (ref.base=NR_NO) and (ref.index=NR_NO) then
                    begin
                      { Set RIP relative addressing for simple symbol references }
                      ref.base:=NR_RIP;
                      ref.refaddr:=addr_pic_no_got
                    end
                  else
                    begin
                      { Use temp register to load calculated 64-bit symbol address for complex references }
                      reference_reset_symbol(href,ref.symbol,0,sizeof(pint));
                      href.base:=NR_RIP;
                      href.refaddr:=addr_pic_no_got;
                      hreg:=GetAddressRegister(list);
                      list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,hreg));
                      ref.symbol:=nil;
                      if ref.base=NR_NO then
                        ref.base:=hreg
                      else if ref.index=NR_NO then
                        begin
                          ref.index:=hreg;
                          ref.scalefactor:=0;
                        end
                      else
                        begin
                          { don't use add, as the flags may contain a value }
                          reference_reset_base(href,ref.base,0,8);
                          href.index:=hreg;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                          ref.base:=getaddressregister(list);
                          list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,ref.base));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                          list.concat(taicpu.op_ref_reg(A_LEA,S_Q,href,hreg));
                          ref.base:=hreg;
>>>>>>> graemeg/cpstrnew
                        end;
                    end;

                end;
          end;
<<<<<<< HEAD
{$elseif defined(i386)}
        add_hreg:=false;
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
          begin
            if assigned(ref.symbol) and
               not(assigned(ref.relsymbol)) and
               ((ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL,AB_PRIVATE_EXTERN]) or
                (cs_create_pic in current_settings.moduleswitches)) then
             begin
<<<<<<< HEAD
               if ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL,AB_PRIVATE_EXTERN] then
=======
               if (ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL]) or
                  ((cs_create_pic in current_settings.moduleswitches) and
                   (ref.symbol.bind in [AB_COMMON,AB_GLOBAL,AB_PRIVATE_EXTERN])) then
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
                 begin
                   hreg:=g_indirect_sym_load(list,ref.symbol.name,asmsym2indsymflags(ref.symbol));
                   ref.symbol:=nil;
                 end
               else
                 begin
                   include(current_procinfo.flags,pi_needs_got);
                   { make a copy of the got register, hreg can get modified }
                   hreg:=getaddressregister(list);
                   a_load_reg_reg(list,OS_ADDR,OS_ADDR,current_procinfo.got,hreg);
                   ref.relsymbol:=current_procinfo.CurrGOTLabel;
                 end;
               add_hreg:=true
             end
          end
        else if (cs_create_pic in current_settings.moduleswitches) and
           assigned(ref.symbol) then
          begin
            reference_reset_symbol(href,ref.symbol,0,sizeof(pint));
            href.base:=current_procinfo.got;
            href.refaddr:=addr_pic;
            include(current_procinfo.flags,pi_needs_got);
            hreg:=getaddressregister(list);
=======
{$else x86_64}
        add_hreg:=false;
        if (target_info.system=system_i386_darwin) then
          begin
            if assigned(ref.symbol) and
               not(assigned(ref.relsymbol)) and
               ((ref.symbol.bind = AB_EXTERNAL) or
                (cs_create_pic in current_settings.moduleswitches)) then
             begin
               if (ref.symbol.bind = AB_EXTERNAL) or
                  ((cs_create_pic in current_settings.moduleswitches) and
                   (ref.symbol.bind in [AB_COMMON,AB_GLOBAL])) then
                 begin
                   hreg:=g_indirect_sym_load(list,ref.symbol.name);
                   ref.symbol:=nil;
                 end
               else
                 begin
                   include(current_procinfo.flags,pi_needs_got);
                   hreg:=current_procinfo.got;
                   ref.relsymbol:=current_procinfo.CurrGOTLabel;
                 end;
               add_hreg:=true
             end
          end
        else if (cs_create_pic in current_settings.moduleswitches) and
           assigned(ref.symbol) and
           not((ref.symbol.bind=AB_LOCAL) and
               (ref.symbol.typ in [AT_LABEL,AT_FUNCTION])) then
          begin
            reference_reset_symbol(href,ref.symbol,0);
            href.base:=current_procinfo.got;
            href.refaddr:=addr_pic;
            include(current_procinfo.flags,pi_needs_got);
            hreg:=cg.getaddressregister(list);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            list.concat(taicpu.op_ref_reg(A_MOV,S_L,href,hreg));
            ref.symbol:=nil;
            add_hreg:=true;
          end;

        if add_hreg then
          begin
            if ref.base=NR_NO then
              ref.base:=hreg
            else if ref.index=NR_NO then
              begin
                ref.index:=hreg;
                ref.scalefactor:=1;
              end
            else
              begin
                { don't use add, as the flags may contain a value }
                reference_reset_base(href,ref.base,0,8);
                href.index:=hreg;
                list.concat(taicpu.op_ref_reg(A_LEA,S_L,href,hreg));
                ref.base:=hreg;
              end;
          end;
{$elseif defined(i8086)}
        { i8086 does not support stack relative addressing }
        if ref.base = NR_STACK_POINTER_REG then
          begin
            href:=ref;
            href.base:=getaddressregister(list);
            { let the register allocator find a suitable register for the reference }
            list.Concat(Taicpu.op_reg_reg(A_MOV, S_W, NR_SP, href.base));
            { if DS<>SS in the current memory model, we need to add an SS: segment override as well }
            if (ref.segment=NR_NO) and not segment_regs_equal(NR_DS,NR_SS) then
              href.segment:=NR_SS;
            ref:=href;
          end;

        { if there is a segment in an int register, move it to ES }
        if (ref.segment<>NR_NO) and (not is_segment_reg(ref.segment)) then
          begin
            list.concat(taicpu.op_reg_reg(A_MOV,S_W,ref.segment,NR_ES));
            ref.segment:=NR_ES;
          end;

        { can the segment override be dropped? }
        if ref.segment<>NR_NO then
          begin
            if (ref.base=NR_BP) and segment_regs_equal(ref.segment,NR_SS) then
              ref.segment:=NR_NO;
            if (ref.base<>NR_BP) and segment_regs_equal(ref.segment,NR_DS) then
              ref.segment:=NR_NO;
          end;
{$endif}
      end;


    procedure tcgx86.floatloadops(t : tcgsize;var op : tasmop;var s : topsize);
      begin
         case t of
            OS_F32 :
              begin
                 op:=A_FLD;
                 s:=S_FS;
              end;
            OS_F64 :
              begin
                 op:=A_FLD;
                 s:=S_FL;
              end;
            OS_F80 :
              begin
                 op:=A_FLD;
                 s:=S_FX;
              end;
            OS_C64 :
              begin
                 op:=A_FILD;
                 s:=S_IQ;
              end;
            else
              internalerror(200204043);
         end;
      end;


    procedure tcgx86.floatload(list: TAsmList; t : tcgsize;const ref : treference);

      var
         op : tasmop;
         s : topsize;
         tmpref : treference;
      begin
         tmpref:=ref;
         make_simple_ref(list,tmpref);
         floatloadops(t,op,s);
         list.concat(Taicpu.Op_ref(op,s,tmpref));
         inc_fpu_stack;
      end;


    procedure tcgx86.floatstoreops(t : tcgsize;var op : tasmop;var s : topsize);

      begin
         case t of
            OS_F32 :
              begin
                 op:=A_FSTP;
                 s:=S_FS;
              end;
            OS_F64 :
              begin
                 op:=A_FSTP;
                 s:=S_FL;
              end;
            OS_F80 :
              begin
                  op:=A_FSTP;
                  s:=S_FX;
               end;
            OS_C64 :
               begin
                  op:=A_FISTP;
                  s:=S_IQ;
               end;
            else
               internalerror(200204042);
         end;
      end;


    procedure tcgx86.floatstore(list: TAsmList; t : tcgsize;const ref : treference);

      var
         op : tasmop;
         s : topsize;
         tmpref : treference;
      begin
         tmpref:=ref;
         make_simple_ref(list,tmpref);
         floatstoreops(t,op,s);
         list.concat(Taicpu.Op_ref(op,s,tmpref));
         { storing non extended floats can cause a floating point overflow }
         if ((t<>OS_F80) and (cs_fpu_fwait in current_settings.localswitches))
{$ifdef i8086}
           { 8087 and 80287 need a FWAIT after a memory store, before it can be
             read with the integer unit }
           or (current_settings.cputype<=cpu_286)
{$endif i8086}
           then
           list.concat(Taicpu.Op_none(A_FWAIT,S_NO));
         dec_fpu_stack;
      end;


    procedure tcgx86.check_register_size(size:tcgsize;reg:tregister);
      begin
        if TCGSize2OpSize[size]<>TCGSize2OpSize[reg_cgsize(reg)] then
          internalerror(200306031);
      end;


{****************************************************************************
                              Assembler code
****************************************************************************}

    procedure tcgx86.a_jmp_name(list : TAsmList;const s : string);
      var
        r: treference;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        if (target_info.system <> system_i386_darwin) then
          list.concat(taicpu.op_sym(A_JMP,S_NO,current_asmdata.RefAsmSymbol(s)))
        else
          begin
            reference_reset_symbol(r,get_darwin_call_stub(s,false),0,sizeof(pint));
=======
=======
>>>>>>> origin/fixes_2_2
        if (target_info.system<>system_i386_darwin) then
          list.concat(taicpu.op_sym(A_JMP,S_NO,current_asmdata.RefAsmSymbol(s)))
        else
          begin 
            reference_reset_symbol(r,get_darwin_call_stub(s),0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            r.refaddr:=addr_full;
            list.concat(taicpu.op_ref(A_JMP,S_NO,r));
          end;
      end;


    procedure tcgx86.a_jmp_always(list : TAsmList;l: tasmlabel);
      begin
        a_jmp_cond(list, OC_NONE, l);
      end;


    function tcgx86.get_darwin_call_stub(const s: string; weak: boolean): tasmsymbol;
      var
        stubname: string;
      begin
        stubname := 'L'+s+'$stub';
        result := current_asmdata.getasmsymbol(stubname);
        if assigned(result) then
          exit;

        if current_asmdata.asmlists[al_imports]=nil then
          current_asmdata.asmlists[al_imports]:=TAsmList.create;

        new_section(current_asmdata.asmlists[al_imports],sec_stub,'',0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        result := current_asmdata.DefineAsmSymbol(stubname,AB_LOCAL,AT_FUNCTION);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        result := current_asmdata.RefAsmSymbol(stubname);
>>>>>>> graemeg/cpstrnew
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(result,0));
        { register as a weak symbol if necessary }
        if weak then
          current_asmdata.weakrefasmsymbol(s);
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));
        current_asmdata.asmlists[al_imports].concat(taicpu.op_none(A_HLT));
        current_asmdata.asmlists[al_imports].concat(taicpu.op_none(A_HLT));
        current_asmdata.asmlists[al_imports].concat(taicpu.op_none(A_HLT));
        current_asmdata.asmlists[al_imports].concat(taicpu.op_none(A_HLT));
        current_asmdata.asmlists[al_imports].concat(taicpu.op_none(A_HLT));
      end;


    procedure tcgx86.a_call_name(list : TAsmList;const s : string; weak: boolean);
      begin
        a_call_name_near(list,s,weak);
      end;


    procedure tcgx86.a_call_name_near(list : TAsmList;const s : string; weak: boolean);
      var
        sym : tasmsymbol;
        r : treference;
      begin

        if (target_info.system <> system_i386_darwin) then
          begin
            if not(weak) then
              sym:=current_asmdata.RefAsmSymbol(s)
            else
              sym:=current_asmdata.WeakRefAsmSymbol(s);
            reference_reset_symbol(r,sym,0,sizeof(pint));
            if (cs_create_pic in current_settings.moduleswitches) and
               { darwin's assembler doesn't want @PLT after call symbols }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               not(target_info.system in [system_x86_64_darwin,system_i386_iphonesim,system_x86_64_iphonesim]) then
=======
               not(target_info.system in [system_x86_64_darwin,system_i386_iphonesim]) then
>>>>>>> graemeg/cpstrnew
=======
               not(target_info.system in [system_x86_64_darwin,system_i386_iphonesim]) then
>>>>>>> graemeg/cpstrnew
=======
               not(target_info.system in [system_x86_64_darwin,system_i386_iphonesim]) then
>>>>>>> graemeg/cpstrnew
=======
               not(target_info.system in [system_x86_64_darwin,system_i386_iphonesim]) then
>>>>>>> origin/cpstrnew
              begin
{$ifdef i386}
                include(current_procinfo.flags,pi_needs_got);
{$endif i386}
                r.refaddr:=addr_pic
              end
            else
              r.refaddr:=addr_full;
          end
        else
          begin
            reference_reset_symbol(r,get_darwin_call_stub(s,weak),0,sizeof(pint));
            r.refaddr:=addr_full;
          end;
        list.concat(taicpu.op_ref(A_CALL,S_NO,r));
      end;


    procedure tcgx86.a_call_name_static(list : TAsmList;const s : string);
      begin
        a_call_name_static_near(list,s);
      end;


    procedure tcgx86.a_call_name_static_near(list : TAsmList;const s : string);
      var
        sym : tasmsymbol;
        r : treference;
      begin
        sym:=current_asmdata.RefAsmSymbol(s);
        reference_reset_symbol(r,sym,0,sizeof(pint));
        r.refaddr:=addr_full;
        list.concat(taicpu.op_ref(A_CALL,S_NO,r));
      end;


    procedure tcgx86.a_call_reg(list : TAsmList;reg : tregister);
      begin
        a_call_reg_near(list,reg);
      end;


    procedure tcgx86.a_call_reg_near(list: TAsmList; reg: tregister);
      begin
        list.concat(taicpu.op_reg(A_CALL,S_NO,reg));
      end;


{********************** load instructions ********************}

    procedure tcgx86.a_load_const_reg(list : TAsmList; tosize: TCGSize; a : tcgint; reg : TRegister);

      begin
        check_register_size(tosize,reg);
        { the optimizer will change it to "xor reg,reg" when loading zero, }
        { no need to do it here too (JM)                                   }
        list.concat(taicpu.op_const_reg(A_MOV,TCGSize2OpSize[tosize],a,reg))
      end;


    procedure tcgx86.a_load_const_ref(list : TAsmList; tosize: tcgsize; a : tcgint;const ref : treference);
      var
        tmpref : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
{$ifdef x86_64}
        { x86_64 only supports signed 32 bits constants directly }
        if (tosize in [OS_S64,OS_64]) and
           ((a<low(longint)) or (a>high(longint))) then
          begin
            a_load_const_ref(list,OS_32,longint(a and $ffffffff),tmpref);
            inc(tmpref.offset,4);
            a_load_const_ref(list,OS_32,longint(a shr 32),tmpref);
          end
        else
{$endif x86_64}
          list.concat(taicpu.op_const_ref(A_MOV,TCGSize2OpSize[tosize],a,tmpref));
      end;


    procedure tcgx86.a_load_reg_ref(list : TAsmList; fromsize,tosize: TCGSize; reg : tregister;const ref : treference);
      var
        op: tasmop;
        s: topsize;
        tmpsize : tcgsize;
        tmpreg  : tregister;
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
        check_register_size(fromsize,reg);
        sizes2load(fromsize,tosize,op,s);
        case s of
{$ifdef x86_64}
          S_BQ,S_WQ,S_LQ,
{$endif x86_64}
          S_BW,S_BL,S_WL :
            begin
              tmpreg:=getintregister(list,tosize);
{$ifdef x86_64}
              { zero extensions to 64 bit on the x86_64 are simply done by writting to the lower 32 bit
                which clears the upper 64 bit too, so it could be that s is S_L while the reg is
                64 bit (FK) }
              if s in [S_BL,S_WL,S_L] then
                begin
                  tmpreg:=makeregsize(list,tmpreg,OS_32);
                  tmpsize:=OS_32;
                end
              else
{$endif x86_64}
                tmpsize:=tosize;
              list.concat(taicpu.op_reg_reg(op,s,reg,tmpreg));
              a_load_reg_ref(list,tmpsize,tosize,tmpreg,tmpref);
            end;
        else
          list.concat(taicpu.op_reg_ref(op,s,reg,tmpref));
        end;
      end;


    procedure tcgx86.a_load_ref_reg(list : TAsmList;fromsize,tosize : tcgsize;const ref: treference;reg : tregister);
      var
        op: tasmop;
        s: topsize;
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
        check_register_size(tosize,reg);
        sizes2load(fromsize,tosize,op,s);
 {$ifdef x86_64}
        { zero extensions to 64 bit on the x86_64 are simply done by writting to the lower 32 bit
          which clears the upper 64 bit too, so it could be that s is S_L while the reg is
          64 bit (FK) }
        if s in [S_BL,S_WL,S_L] then
          reg:=makeregsize(list,reg,OS_32);
{$endif x86_64}
        list.concat(taicpu.op_ref_reg(op,s,tmpref,reg));
      end;


    procedure tcgx86.a_load_reg_reg(list : TAsmList;fromsize,tosize : tcgsize;reg1,reg2 : tregister);
      var
        op: tasmop;
        s: topsize;
        instr:Taicpu;
      begin
        check_register_size(fromsize,reg1);
        check_register_size(tosize,reg2);
        if tcgsize2size[fromsize]>tcgsize2size[tosize] then
          begin
            reg1:=makeregsize(list,reg1,tosize);
            s:=tcgsize2opsize[tosize];
            op:=A_MOV;
          end
        else
          sizes2load(fromsize,tosize,op,s);
{$ifdef x86_64}
        { zero extensions to 64 bit on the x86_64 are simply done by writting to the lower 32 bit
          which clears the upper 64 bit too, so it could be that s is S_L while the reg is
          64 bit (FK)
        }
        if s in [S_BL,S_WL,S_L] then
          reg2:=makeregsize(list,reg2,OS_32);
{$endif x86_64}
        if (reg1<>reg2) then
          begin
            instr:=taicpu.op_reg_reg(op,s,reg1,reg2);
            { Notify the register allocator that we have written a move instruction so
              it can try to eliminate it. }
            if (reg1<>current_procinfo.framepointer) and (reg1<>NR_STACK_POINTER_REG) then
              add_move_instruction(instr);
            list.concat(instr);
          end;
{$ifdef x86_64}
        { avoid merging of registers and killing the zero extensions (FK) }
        if (tosize in [OS_64,OS_S64]) and (s=S_L) then
          list.concat(taicpu.op_const_reg(A_AND,S_L,$ffffffff,reg2));
{$endif x86_64}
      end;


    procedure tcgx86.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);
      var
        tmpref  : treference;
      begin
        with ref do
          begin
            if (base=NR_NO) and (index=NR_NO) then
              begin
                if assigned(ref.symbol) then
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) and
                       ((ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL]) or
                        (cs_create_pic in current_settings.moduleswitches)) then
                      begin
                        if (ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL]) or
                           ((cs_create_pic in current_settings.moduleswitches) and
                            (ref.symbol.bind in [AB_COMMON,AB_GLOBAL,AB_PRIVATE_EXTERN])) then
                          begin
                             reference_reset_base(tmpref,
                               g_indirect_sym_load(list,ref.symbol.name,asmsym2indsymflags(ref.symbol)),
                               offset,sizeof(pint));
=======
=======
>>>>>>> origin/fixes_2_2
                    if (target_info.system=system_i386_darwin) and
                       ((ref.symbol.bind = AB_EXTERNAL) or
                        (cs_create_pic in current_settings.moduleswitches)) then
                      begin
                        if (ref.symbol.bind = AB_EXTERNAL) or
                           ((cs_create_pic in current_settings.moduleswitches) and
                            (ref.symbol.bind in [AB_COMMON,AB_GLOBAL])) then
                          begin
                             reference_reset_base(tmpref,
                               g_indirect_sym_load(list,ref.symbol.name),
                               offset);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                             a_loadaddr_ref_reg(list,tmpref,r);
                          end
                       else
                         begin
                           include(current_procinfo.flags,pi_needs_got);
<<<<<<< HEAD
<<<<<<< HEAD
                           reference_reset_base(tmpref,current_procinfo.got,offset,ref.alignment);
=======
                           reference_reset_base(tmpref,current_procinfo.got,offset);
>>>>>>> graemeg/fixes_2_2
=======
                           reference_reset_base(tmpref,current_procinfo.got,offset);
>>>>>>> origin/fixes_2_2
                           tmpref.symbol:=symbol;
                           tmpref.relsymbol:=current_procinfo.CurrGOTLabel;
                           list.concat(Taicpu.op_ref_reg(A_LEA,tcgsize2opsize[OS_ADDR],tmpref,r));
                         end;
                      end
<<<<<<< HEAD
<<<<<<< HEAD
                    else if (cs_create_pic in current_settings.moduleswitches)
{$ifdef x86_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                             and not(ref.symbol.bind=AB_LOCAL)
=======
                             and not((ref.symbol.bind=AB_LOCAL) and
                                     (ref.symbol.typ=AT_DATA))
>>>>>>> graemeg/cpstrnew
=======
                             and not((ref.symbol.bind=AB_LOCAL) and
                                     (ref.symbol.typ=AT_DATA))
>>>>>>> graemeg/cpstrnew
=======
                             and not((ref.symbol.bind=AB_LOCAL) and
                                     (ref.symbol.typ=AT_DATA))
>>>>>>> graemeg/cpstrnew
=======
                             and not((ref.symbol.bind=AB_LOCAL) and
                                     (ref.symbol.typ=AT_DATA))
>>>>>>> origin/cpstrnew
{$endif x86_64}
                            then
=======
                    else if (cs_create_pic in current_settings.moduleswitches) then
>>>>>>> graemeg/fixes_2_2
=======
                    else if (cs_create_pic in current_settings.moduleswitches) then
>>>>>>> origin/fixes_2_2
                      begin
{$ifdef x86_64}
                        reference_reset_symbol(tmpref,ref.symbol,0,ref.alignment);
                        tmpref.refaddr:=addr_pic;
                        tmpref.base:=NR_RIP;
                        list.concat(taicpu.op_ref_reg(A_MOV,S_Q,tmpref,r));
{$else x86_64}
                        reference_reset_symbol(tmpref,ref.symbol,0,ref.alignment);
                        tmpref.refaddr:=addr_pic;
                        tmpref.base:=current_procinfo.got;
                        include(current_procinfo.flags,pi_needs_got);
                        list.concat(taicpu.op_ref_reg(A_MOV,S_L,tmpref,r));
{$endif x86_64}
                        if offset<>0 then
                          a_op_const_reg(list,OP_ADD,OS_ADDR,offset,r);
                      end
{$ifdef x86_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    else if (target_info.system in (systems_all_windows+[system_x86_64_darwin,system_x86_64_iphonesim]))
=======
                    else if (target_info.system in (systems_all_windows+[system_x86_64_darwin]))
>>>>>>> graemeg/cpstrnew
=======
                    else if (target_info.system in (systems_all_windows+[system_x86_64_darwin]))
>>>>>>> graemeg/cpstrnew
=======
                    else if (target_info.system in (systems_all_windows+[system_x86_64_darwin]))
>>>>>>> graemeg/cpstrnew
=======
                    else if (target_info.system in (systems_all_windows+[system_x86_64_darwin]))
>>>>>>> origin/cpstrnew
			 or (cs_create_pic in current_settings.moduleswitches)
			 then
                      begin
                        { Win64 and Darwin/x86_64 always require RIP-relative addressing }
                        tmpref:=ref;
                        tmpref.base:=NR_RIP;
                        tmpref.refaddr:=addr_pic_no_got;
                        list.concat(Taicpu.op_ref_reg(A_LEA,S_Q,tmpref,r));
                      end
{$endif x86_64}
                    else
                      begin
                        tmpref:=ref;
                        tmpref.refaddr:=ADDR_FULL;
                        list.concat(Taicpu.op_ref_reg(A_MOV,tcgsize2opsize[OS_ADDR],tmpref,r));
                      end
                  end
                else
                  a_load_const_reg(list,OS_ADDR,offset,r)
              end
            else if (base=NR_NO) and (index<>NR_NO) and
                    (offset=0) and (scalefactor=0) and (symbol=nil) then
              a_load_reg_reg(list,OS_ADDR,OS_ADDR,index,r)
            else if (base<>NR_NO) and (index=NR_NO) and
                    (offset=0) and (symbol=nil) then
                a_load_reg_reg(list,OS_ADDR,OS_ADDR,base,r)
            else
              begin
                tmpref:=ref;
                make_simple_ref(list,tmpref);
                list.concat(Taicpu.op_ref_reg(A_LEA,tcgsize2opsize[OS_ADDR],tmpref,r));
              end;
            if segment<>NR_NO then
              begin
{$ifdef i8086}
                if is_segment_reg(segment) then
                  list.concat(Taicpu.op_reg_reg(A_MOV,S_W,segment,GetNextReg(r)))
                else
                  a_load_reg_reg(list,OS_16,OS_16,segment,GetNextReg(r));
{$else i8086}
                if (tf_section_threadvars in target_info.flags) then
                  begin
                    { Convert thread local address to a process global addres
                      as we cannot handle far pointers.}
                    case target_info.system of
                      system_i386_linux,system_i386_android:
                        if segment=NR_GS then
                          begin
                            reference_reset_symbol(tmpref,current_asmdata.RefAsmSymbol('___fpc_threadvar_offset'),0,ref.alignment);
                            tmpref.segment:=NR_GS;
                            list.concat(Taicpu.op_ref_reg(A_ADD,tcgsize2opsize[OS_ADDR],tmpref,r));
                          end
                        else
                          cgmessage(cg_e_cant_use_far_pointer_there);
                      else
                        cgmessage(cg_e_cant_use_far_pointer_there);
                    end;
                  end
                else
                  cgmessage(cg_e_cant_use_far_pointer_there);
{$endif i8086}
              end;
          end;
      end;


    { all fpu load routines expect that R_ST[0-7] means an fpu regvar and }
    { R_ST means "the current value at the top of the fpu stack" (JM)     }
    procedure tcgx86.a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister);

       var
         href: treference;
         op: tasmop;
         s: topsize;
       begin
         if (reg1<>NR_ST) then
           begin
             floatloadops(tosize,op,s);
             list.concat(taicpu.op_reg(op,s,rgfpu.correct_fpuregister(reg1,rgfpu.fpuvaroffset)));
             inc_fpu_stack;
           end;
         if (reg2<>NR_ST) then
           begin
             floatstoreops(tosize,op,s);
             list.concat(taicpu.op_reg(op,s,rgfpu.correct_fpuregister(reg2,rgfpu.fpuvaroffset)));
             dec_fpu_stack;
           end;
         { OS_F80 < OS_C64, but OS_C64 fits perfectly in OS_F80 }
         if (reg1=NR_ST) and
            (reg2=NR_ST) and
            (tosize<>OS_F80) and
            (tosize<fromsize) then
           begin
             { can't round down to lower precision in x87 :/ }
<<<<<<< HEAD
<<<<<<< HEAD
             tg.gettemp(list,tcgsize2size[tosize],tcgsize2size[tosize],tt_normal,href);
=======
             tg.gettemp(list,tcgsize2size[tosize],tt_normal,href);
>>>>>>> graemeg/fixes_2_2
=======
             tg.gettemp(list,tcgsize2size[tosize],tt_normal,href);
>>>>>>> origin/fixes_2_2
             a_loadfpu_reg_ref(list,fromsize,tosize,NR_ST,href);
             a_loadfpu_ref_reg(list,tosize,tosize,href,NR_ST);
             tg.ungettemp(list,href);
           end;
       end;


    procedure tcgx86.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister);
       begin
         floatload(list,fromsize,ref);
         a_loadfpu_reg_reg(list,fromsize,tosize,NR_ST,reg);
       end;


    procedure tcgx86.a_loadfpu_reg_ref(list: TAsmList; fromsize,tosize: tcgsize; reg: tregister; const ref: treference);
       begin
         { in case a record returned in a floating point register
           (LOC_FPUREGISTER with OS_F32/OS_F64) is stored in memory
           (LOC_REFERENCE with OS_32/OS_64), we have to adjust the
           tosize }
        if (fromsize in [OS_F32,OS_F64]) and
           (tcgsize2size[fromsize]=tcgsize2size[tosize]) then
          case tosize of
            OS_32:
              tosize:=OS_F32;
            OS_64:
              tosize:=OS_F64;
          end;
         if reg<>NR_ST then
           a_loadfpu_reg_reg(list,fromsize,tosize,reg,NR_ST);
         floatstore(list,tosize,ref);
       end;


    function get_scalar_mm_op(fromsize,tosize : tcgsize) : tasmop;
      const
        convertopsse : array[OS_F32..OS_F128,OS_F32..OS_F128] of tasmop = (
          (A_MOVSS,A_CVTSS2SD,A_NONE,A_NONE,A_NONE),
          (A_CVTSD2SS,A_MOVSD,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_MOVQ,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE));
        convertopavx : array[OS_F32..OS_F128,OS_F32..OS_F128] of tasmop = (
          (A_VMOVSS,A_VCVTSS2SD,A_NONE,A_NONE,A_NONE),
          (A_VCVTSD2SS,A_VMOVSD,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_MOVQ,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE));
      begin
        { we can have OS_F32/OS_F64 (record in function result/LOC_MMREGISTER) to
          OS_32/OS_64 (record in memory/LOC_REFERENCE) }
        if (fromsize in [OS_F32,OS_F64]) and
           (tcgsize2size[fromsize]=tcgsize2size[tosize]) then
          case tosize of
            OS_32:
              tosize:=OS_F32;
            OS_64:
              tosize:=OS_F64;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (fromsize in [low(convertopsse)..high(convertopsse)]) and
           (tosize in [low(convertopsse)..high(convertopsse)]) then
          begin
            if UseAVX then
              result:=convertopavx[fromsize,tosize]
            else
              result:=convertopsse[fromsize,tosize];
          end
=======
        if (fromsize in [low(convertop)..high(convertop)]) and
           (tosize in [low(convertop)..high(convertop)]) then
          result:=convertop[fromsize,tosize]
>>>>>>> graemeg/cpstrnew
=======
        if (fromsize in [low(convertop)..high(convertop)]) and
           (tosize in [low(convertop)..high(convertop)]) then
          result:=convertop[fromsize,tosize]
>>>>>>> graemeg/cpstrnew
=======
        if (fromsize in [low(convertop)..high(convertop)]) and
           (tosize in [low(convertop)..high(convertop)]) then
          result:=convertop[fromsize,tosize]
>>>>>>> graemeg/cpstrnew
=======
        if (fromsize in [low(convertop)..high(convertop)]) and
           (tosize in [low(convertop)..high(convertop)]) then
          result:=convertop[fromsize,tosize]
>>>>>>> origin/cpstrnew
        { we can have OS_M64 (record in function result/LOC_MMREGISTER) to
          OS_64 (record in memory/LOC_REFERENCE) }
        else if (tcgsize2size[fromsize]=tcgsize2size[tosize]) and
                (fromsize=OS_M64) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          begin
            if UseAVX then
              result:=A_VMOVQ
            else
              result:=A_MOVQ;
          end
=======
          result:=A_MOVQ
>>>>>>> graemeg/cpstrnew
=======
          result:=A_MOVQ
>>>>>>> graemeg/cpstrnew
=======
          result:=A_MOVQ
>>>>>>> graemeg/cpstrnew
=======
          result:=A_MOVQ
>>>>>>> origin/cpstrnew
        else
          internalerror(2010060104);
        if result=A_NONE then
          internalerror(200312205);
      end;


    procedure tcgx86.a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle);
      var
        instr : taicpu;
        op : TAsmOp;
      begin
        if shuffle=nil then
          begin
            if fromsize=tosize then
              { needs correct size in case of spilling }
              case fromsize of
                OS_F32:
                  instr:=taicpu.op_reg_reg(A_MOVAPS,S_NO,reg1,reg2);
                OS_F64:
                  instr:=taicpu.op_reg_reg(A_MOVAPD,S_NO,reg1,reg2);
                OS_M64:
                  instr:=taicpu.op_reg_reg(A_MOVQ,S_NO,reg1,reg2);
                else
                  internalerror(2006091201);
              end
            else
              internalerror(200312202);
            add_move_instruction(instr);
          end
        else if shufflescalar(shuffle) then
          begin
            op:=get_scalar_mm_op(fromsize,tosize);

            { MOVAPD/MOVAPS are normally faster }
            if op=A_MOVSD then
              op:=A_MOVAPD
            else if op=A_MOVSS then
              op:=A_MOVAPS
            { VMOVSD/SS is not available with two register operands }
            else if op=A_VMOVSD then
              op:=A_VMOVAPD
            else if op=A_VMOVSS then
              op:=A_VMOVAPS;

            { A_VCVTSD2SS and A_VCVTSS2SD require always three operands }
            if (op=A_VCVTSD2SS) or (op=A_VCVTSS2SD) then
              instr:=taicpu.op_reg_reg_reg(op,S_NO,reg1,reg2,reg2)
            else
              instr:=taicpu.op_reg_reg(op,S_NO,reg1,reg2);

            case op of
              A_VMOVAPD,
              A_VMOVAPS,
              A_VMOVSS,
              A_VMOVSD,
              A_VMOVQ,
              A_MOVAPD,
              A_MOVAPS,
              A_MOVSS,
              A_MOVSD,
              A_MOVQ:
                add_move_instruction(instr);
            end;
          end
        else
          internalerror(200312201);
        list.concat(instr);
      end;


    procedure tcgx86.a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle);
       var
         tmpref  : treference;
         op : tasmop;
       begin
         tmpref:=ref;
         make_simple_ref(list,tmpref);
         if shuffle=nil then
           begin
             if fromsize=OS_M64 then
               list.concat(taicpu.op_ref_reg(A_MOVQ,S_NO,tmpref,reg))
             else
{$ifdef x86_64}
               { x86-64 has always properly aligned data }
               list.concat(taicpu.op_ref_reg(A_MOVDQA,S_NO,tmpref,reg));
{$else x86_64}
               list.concat(taicpu.op_ref_reg(A_MOVDQU,S_NO,tmpref,reg));
{$endif x86_64}
           end
         else if shufflescalar(shuffle) then
           begin
             op:=get_scalar_mm_op(fromsize,tosize);

             { A_VCVTSD2SS and A_VCVTSS2SD require always three operands }
             if (op=A_VCVTSD2SS) or (op=A_VCVTSS2SD) then
               list.concat(taicpu.op_ref_reg_reg(op,S_NO,tmpref,reg,reg))
             else
               list.concat(taicpu.op_ref_reg(op,S_NO,tmpref,reg))
           end
         else
           internalerror(200312252);
       end;


    procedure tcgx86.a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle);
       var
         hreg : tregister;
         tmpref  : treference;
         op : tasmop;
       begin
         tmpref:=ref;
         make_simple_ref(list,tmpref);
         if shuffle=nil then
           begin
             if fromsize=OS_M64 then
               list.concat(taicpu.op_reg_ref(A_MOVQ,S_NO,reg,tmpref))
             else
{$ifdef x86_64}
               { x86-64 has always properly aligned data }
               list.concat(taicpu.op_reg_ref(A_MOVDQA,S_NO,reg,tmpref))
{$else x86_64}
               list.concat(taicpu.op_reg_ref(A_MOVDQU,S_NO,reg,tmpref))
{$endif x86_64}
           end
         else if shufflescalar(shuffle) then
           begin
             if tcgsize2size[tosize]<>tcgsize2size[fromsize] then
               begin
                 hreg:=getmmregister(list,tosize);
                 op:=get_scalar_mm_op(fromsize,tosize);

                 { A_VCVTSD2SS and A_VCVTSS2SD require always three operands }
                 if (op=A_VCVTSD2SS) or (op=A_VCVTSS2SD) then
                   list.concat(taicpu.op_reg_reg_reg(op,S_NO,reg,hreg,hreg))
                 else
                   list.concat(taicpu.op_reg_reg(op,S_NO,reg,hreg));

                 list.concat(taicpu.op_reg_ref(get_scalar_mm_op(tosize,tosize),S_NO,hreg,tmpref))
               end
             else
               list.concat(taicpu.op_reg_ref(get_scalar_mm_op(fromsize,tosize),S_NO,reg,tmpref));
           end
         else
           internalerror(200312252);
       end;


    procedure tcgx86.a_opmm_ref_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle);
      var
        l : tlocation;
      begin
        l.loc:=LOC_REFERENCE;
        l.reference:=ref;
        l.size:=size;
        opmm_loc_reg(list,op,size,l,reg,shuffle);
      end;


    procedure tcgx86.a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src,dst: tregister;shuffle : pmmshuffle);
     var
       l : tlocation;
     begin
       l.loc:=LOC_MMREGISTER;
       l.register:=src;
       l.size:=size;
       opmm_loc_reg(list,op,size,l,dst,shuffle);
     end;


    procedure tcgx86.opmm_loc_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;loc : tlocation;src,dst: tregister; shuffle : pmmshuffle);
      const
        opmm2asmop : array[0..1,OS_F32..OS_F64,topcg] of tasmop = (
          ( { scalar }
            ( { OS_F32 }
              A_NOP,A_NOP,A_VADDSS,A_NOP,A_VDIVSS,A_NOP,A_NOP,A_VMULSS,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_VSUBSS,A_NOP,A_NOP,A_NOP
            ),
            ( { OS_F64 }
              A_NOP,A_NOP,A_VADDSD,A_NOP,A_VDIVSD,A_NOP,A_NOP,A_VMULSD,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_VSUBSD,A_NOP,A_NOP,A_NOP
            )
          ),
          ( { vectorized/packed }
            { because the logical packed single instructions have shorter op codes, we use always
              these
            }
            ( { OS_F32 }
              A_NOP,A_NOP,A_VADDPS,A_NOP,A_VDIVPS,A_NOP,A_NOP,A_VMULPS,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_VSUBPS,A_VXORPS,A_NOP,A_NOP
            ),
            ( { OS_F64 }
              A_NOP,A_NOP,A_VADDPD,A_NOP,A_VDIVPD,A_NOP,A_NOP,A_VMULPD,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_VSUBPD,A_VXORPD,A_NOP,A_NOP
            )
          )
        );

      var
        resultreg : tregister;
        asmop : tasmop;
      begin
        { this is an internally used procedure so the parameters have
          some constrains
        }
        if loc.size<>size then
          internalerror(2013061108);
        resultreg:=dst;
        { deshuffle }
        //!!!
        if (shuffle<>nil) and not(shufflescalar(shuffle)) then
          begin
            internalerror(2013061107);
          end
        else if (shuffle=nil) then
          asmop:=opmm2asmop[1,size,op]
        else if shufflescalar(shuffle) then
          begin
            asmop:=opmm2asmop[0,size,op];
            { no scalar operation available? }
            if asmop=A_NOP then
              begin
                { do vectorized and shuffle finally }
                internalerror(2010060102);
              end;
          end
        else
          internalerror(2013061106);
        if asmop=A_NOP then
          internalerror(2013061105);
        case loc.loc of
          LOC_CREFERENCE,LOC_REFERENCE:
            begin
              make_simple_ref(current_asmdata.CurrAsmList,loc.reference);
              list.concat(taicpu.op_ref_reg_reg(asmop,S_NO,loc.reference,src,resultreg));
            end;
          LOC_CMMREGISTER,LOC_MMREGISTER:
            list.concat(taicpu.op_reg_reg_reg(asmop,S_NO,loc.register,src,resultreg));
          else
            internalerror(2013061104);
        end;
        { shuffle }
        if resultreg<>dst then
          begin
            internalerror(2013061103);
          end;
      end;


    procedure tcgx86.a_opmm_reg_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src1,src2,dst: tregister;shuffle : pmmshuffle);
      var
        l : tlocation;
      begin
        l.loc:=LOC_MMREGISTER;
        l.register:=src1;
        l.size:=size;
        opmm_loc_reg_reg(list,op,size,l,src2,dst,shuffle);
      end;


    procedure tcgx86.a_opmm_ref_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const ref: treference; src,dst: tregister;shuffle : pmmshuffle);
      var
        l : tlocation;
      begin
        l.loc:=LOC_REFERENCE;
        l.reference:=ref;
        l.size:=size;
        opmm_loc_reg_reg(list,op,size,l,src,dst,shuffle);
      end;


    procedure tcgx86.opmm_loc_reg(list: TAsmList; Op: TOpCG; size : tcgsize;loc : tlocation;dst: tregister; shuffle : pmmshuffle);
      const
        opmm2asmop : array[0..1,OS_F32..OS_F64,topcg] of tasmop = (
          ( { scalar }
            ( { OS_F32 }
              A_NOP,A_NOP,A_ADDSS,A_NOP,A_DIVSS,A_NOP,A_NOP,A_MULSS,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_SUBSS,A_NOP,A_NOP,A_NOP
            ),
            ( { OS_F64 }
              A_NOP,A_NOP,A_ADDSD,A_NOP,A_DIVSD,A_NOP,A_NOP,A_MULSD,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_SUBSD,A_NOP,A_NOP,A_NOP
            )
          ),
          ( { vectorized/packed }
            { because the logical packed single instructions have shorter op codes, we use always
              these
            }
            ( { OS_F32 }
              A_NOP,A_NOP,A_ADDPS,A_NOP,A_DIVPS,A_NOP,A_NOP,A_MULPS,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_SUBPS,A_XORPS,A_NOP,A_NOP
            ),
            ( { OS_F64 }
              A_NOP,A_NOP,A_ADDPD,A_NOP,A_DIVPD,A_NOP,A_NOP,A_MULPD,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_NOP,A_SUBPD,A_XORPD,A_NOP,A_NOP
            )
          )
        );
      var
        resultreg : tregister;
        asmop : tasmop;
      begin
        { this is an internally used procedure so the parameters have
          some constrains
        }
        if loc.size<>size then
          internalerror(200312213);
        resultreg:=dst;
        { deshuffle }
        //!!!
        if (shuffle<>nil) and not(shufflescalar(shuffle)) then
          begin
            internalerror(2010060101);
          end
        else if (shuffle=nil) then
          asmop:=opmm2asmop[1,size,op]
        else if shufflescalar(shuffle) then
          begin
            asmop:=opmm2asmop[0,size,op];
            { no scalar operation available? }
            if asmop=A_NOP then
              begin
                { do vectorized and shuffle finally }
                internalerror(2010060102);
              end;
          end
        else
          internalerror(200312211);
        if asmop=A_NOP then
          internalerror(200312216);
        case loc.loc of
          LOC_CREFERENCE,LOC_REFERENCE:
            begin
              make_simple_ref(current_asmdata.CurrAsmList,loc.reference);
              list.concat(taicpu.op_ref_reg(asmop,S_NO,loc.reference,resultreg));
            end;
          LOC_CMMREGISTER,LOC_MMREGISTER:
            list.concat(taicpu.op_reg_reg(asmop,S_NO,loc.register,resultreg));
          else
            internalerror(200312214);
        end;
        { shuffle }
        if resultreg<>dst then
          begin
            internalerror(200312212);
          end;
      end;


{$ifndef i8086}
    procedure tcgx86.a_op_const_reg_reg(list:TAsmList;op:Topcg;size:Tcgsize;
                                        a:tcgint;src,dst:Tregister);
      var
        power,al  : longint;
        href : treference;
      begin
        power:=0;
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
        end;
        if (op in [OP_MUL,OP_IMUL]) and (size in [OS_32,OS_S32,OS_64,OS_S64]) and
          not(cs_check_overflow in current_settings.localswitches) and
          (a>1) and ispowerof2(int64(a-1),power) and (power in [1..3]) then
          begin
            reference_reset_base(href,src,0,0);
            href.index:=src;
            href.scalefactor:=a-1;
            list.concat(taicpu.op_ref_reg(A_LEA,TCgSize2OpSize[size],href,dst));
          end
        else if (op in [OP_MUL,OP_IMUL]) and (size in [OS_32,OS_S32,OS_64,OS_S64]) and
          not(cs_check_overflow in current_settings.localswitches) and
          (a>1) and ispowerof2(int64(a),power) and (power in [1..3]) then
          begin
            reference_reset_base(href,NR_NO,0,0);
            href.index:=src;
            href.scalefactor:=a;
            list.concat(taicpu.op_ref_reg(A_LEA,TCgSize2OpSize[size],href,dst));
          end
        else if (op in [OP_MUL,OP_IMUL]) and (size in [OS_32,OS_S32,OS_64,OS_S64]) and
          (a>1) and (a<=maxLongint) and not ispowerof2(int64(a),power) then
          begin
            { MUL with overflow checking should be handled specifically in the code generator }
            if (op=OP_MUL) and (cs_check_overflow in current_settings.localswitches) then
              internalerror(2014011801);
            list.concat(taicpu.op_const_reg_reg(A_IMUL,TCgSize2OpSize[size],a,src,dst));
          end
        else if (op=OP_ADD) and
          ((size in [OS_32,OS_S32]) or
           { lea supports only 32 bit signed displacments }
           ((size=OS_64) and (a>=0) and (a<=maxLongint)) or
           ((size=OS_S64) and (a>=-maxLongint) and (a<=maxLongint))
          ) and
          not(cs_check_overflow in current_settings.localswitches) then
          begin
            { a might still be in the range 0x80000000 to 0xffffffff
              which might trigger a range check error as
              reference_reset_base expects a longint value. }
{$push} {$R-}{$Q-}
            al := longint (a); 
{$pop}
            reference_reset_base(href,src,al,0);
            list.concat(taicpu.op_ref_reg(A_LEA,TCgSize2OpSize[size],href,dst));
          end
        else if (op=OP_SUB) and
          ((size in [OS_32,OS_S32]) or
           { lea supports only 32 bit signed displacments }
           ((size=OS_64) and (a>=0) and (a<=maxLongint)) or
           ((size=OS_S64) and (a>=-maxLongint) and (a<=maxLongint))
          ) and
          not(cs_check_overflow in current_settings.localswitches) then
          begin
            reference_reset_base(href,src,-a,0);
            list.concat(taicpu.op_ref_reg(A_LEA,TCgSize2OpSize[size],href,dst));
          end
        else if (op in [OP_ROR,OP_ROL]) and
          (CPUX86_HAS_BMI2 in cpu_capabilities[current_settings.cputype]) and
          (size in [OS_32,OS_S32
{$ifdef x86_64}
            ,OS_64,OS_S64
{$endif x86_64}
          ]) then
          begin
            if op=OP_ROR then
              list.concat(taicpu.op_const_reg_reg(A_RORX,TCgSize2OpSize[size], a,src,dst))
            else
              list.concat(taicpu.op_const_reg_reg(A_RORX,TCgSize2OpSize[size],TCgSize2Size[size]*8-a,src,dst));
          end
        else
          inherited a_op_const_reg_reg(list,op,size,a,src,dst);
      end;


    procedure tcgx86.a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
      size: tcgsize; src1, src2, dst: tregister);
      var
        href : treference;
      begin
        if (op=OP_ADD) and (size in [OS_32,OS_S32,OS_64,OS_S64]) and
          not(cs_check_overflow in current_settings.localswitches) then
          begin
            reference_reset_base(href,src1,0,0);
            href.index:=src2;
            list.concat(taicpu.op_ref_reg(A_LEA,TCgSize2OpSize[size],href,dst));
          end
        else if (op in [OP_SHR,OP_SHL]) and
          (CPUX86_HAS_BMI2 in cpu_capabilities[current_settings.cputype]) and
          (size in [OS_32,OS_S32
{$ifdef x86_64}
            ,OS_64,OS_S64
{$endif x86_64}
          ]) then
          begin
            if op=OP_SHL then
              list.concat(taicpu.op_reg_reg_reg(A_SHLX,TCgSize2OpSize[size],src1,src2,dst))
            else
              list.concat(taicpu.op_reg_reg_reg(A_SHRX,TCgSize2OpSize[size],src1,src2,dst));
          end
        else
          inherited a_op_reg_reg_reg(list,op,size,src1,src2,dst);
      end;
{$endif not i8086}


    procedure tcgx86.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister);
{$ifdef x86_64}
      var
        tmpreg : tregister;
{$endif x86_64}
      begin
        optimize_op_const(size, op, a);
{$ifdef x86_64}
        { x86_64 only supports signed 32 bits constants directly }
        if not(op in [OP_NONE,OP_MOVE]) and
           (size in [OS_S64,OS_64]) and
            ((a<low(longint)) or (a>high(longint))) then
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            a_op_reg_reg(list,op,size,tmpreg,reg);
            exit;
          end;
{$endif x86_64}
        check_register_size(size,reg);
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
          OP_DIV, OP_IDIV:
            begin
              { should be handled specifically in the code               }
              { generator because of the silly register usage restraints }
              internalerror(200109224);
            end;
          OP_MUL,OP_IMUL:
            begin
              if not (cs_check_overflow in current_settings.localswitches) then
                op:=OP_IMUL;
              if op = OP_IMUL then
                list.concat(taicpu.op_const_reg(A_IMUL,TCgSize2OpSize[size],a,reg))
              else
                { OP_MUL should be handled specifically in the code        }
                { generator because of the silly register usage restraints }
                internalerror(200109225);
            end;
          OP_ADD, OP_SUB:
            if not(cs_check_overflow in current_settings.localswitches) and
               (a = 1) and
               UseIncDec then
              begin
                if op = OP_ADD then
                  list.concat(taicpu.op_reg(A_INC,TCgSize2OpSize[size],reg))
                else
                  list.concat(taicpu.op_reg(A_DEC,TCgSize2OpSize[size],reg))
              end
            else
              list.concat(taicpu.op_const_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],aint(a),reg));

          OP_AND,OP_OR:
            list.concat(taicpu.op_const_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],aint(a),reg));

          OP_XOR:
            if (aword(a)=high(aword)) then
              list.concat(taicpu.op_reg(A_NOT,TCgSize2OpSize[size],reg))
            else
              list.concat(taicpu.op_const_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],aint(a),reg));

          OP_SHL,OP_SHR,OP_SAR,OP_ROL,OP_ROR:
            begin
{$if defined(x86_64)}
              if (a and 63) <> 0 Then
                list.concat(taicpu.op_const_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],a and 63,reg));
              if (a shr 6) <> 0 Then
                internalerror(200609073);
{$elseif defined(i386)}
              if (a and 31) <> 0 Then
                list.concat(taicpu.op_const_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],a and 31,reg));
              if (a shr 5) <> 0 Then
                internalerror(200609071);
{$elseif defined(i8086)}
              if (a shr 5) <> 0 Then
                internalerror(2013043002);
              a := a and 31;
              if a <> 0 Then
                begin
                  if (current_settings.cputype < cpu_186) and (a <> 1) then
                    begin
                      getcpuregister(list,NR_CL);
                      a_load_const_reg(list,OS_8,a,NR_CL);
                      list.concat(taicpu.op_reg_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],NR_CL,reg));
                      ungetcpuregister(list,NR_CL);
                    end
                  else
                    list.concat(taicpu.op_const_reg(TOpCG2AsmOp[op],TCgSize2OpSize[size],a,reg));
                end;
{$endif}
            end
          else internalerror(200609072);
        end;
      end;


    procedure tcgx86.a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference);
      var
{$ifdef x86_64}
        tmpreg : tregister;
{$endif x86_64}
        tmpref  : treference;
      begin
        optimize_op_const(size, op, a);
        if op in [OP_NONE,OP_MOVE] then
          begin
            if (op=OP_MOVE) then
              a_load_const_ref(list,size,a,ref);
            exit;
          end;

{$ifdef x86_64}
        { x86_64 only supports signed 32 bits constants directly }
        if (size in [OS_S64,OS_64]) and
            ((a<low(longint)) or (a>high(longint))) then
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            a_op_reg_ref(list,op,size,tmpreg,ref);
            exit;
          end;
{$endif x86_64}

        tmpref:=ref;
        make_simple_ref(list,tmpref);

        Case Op of
          OP_DIV, OP_IDIV:
            Begin
              { should be handled specifically in the code               }
              { generator because of the silly register usage restraints }
              internalerror(200109231);
            End;
          OP_MUL,OP_IMUL:
            begin
              if not (cs_check_overflow in current_settings.localswitches) then
                op:=OP_IMUL;
              { can't multiply a memory location directly with a constant }
              if op = OP_IMUL then
                inherited a_op_const_ref(list,op,size,a,tmpref)
              else
                { OP_MUL should be handled specifically in the code        }
                { generator because of the silly register usage restraints }
                internalerror(200109232);
            end;
          OP_ADD, OP_SUB:
            if not(cs_check_overflow in current_settings.localswitches) and
               (a = 1) and
               UseIncDec then
              begin
                if op = OP_ADD then
                  list.concat(taicpu.op_ref(A_INC,TCgSize2OpSize[size],tmpref))
                else
                  list.concat(taicpu.op_ref(A_DEC,TCgSize2OpSize[size],tmpref))
              end
            else
              list.concat(taicpu.op_const_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],a,tmpref));

          OP_AND,OP_OR:
            list.concat(taicpu.op_const_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],a,tmpref));

          OP_XOR:
            if (aword(a)=high(aword)) then
              list.concat(taicpu.op_ref(A_NOT,TCgSize2OpSize[size],tmpref))
            else
              list.concat(taicpu.op_const_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],a,tmpref));

          OP_SHL,OP_SHR,OP_SAR,OP_ROL,OP_ROR:
            begin
{$if defined(x86_64)}
              if (a and 63) <> 0 Then
                list.concat(taicpu.op_const_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],a and 63,tmpref));
              if (a shr 6) <> 0 Then
                internalerror(2013111003);
{$elseif defined(i386)}
              if (a and 31) <> 0 Then
                list.concat(taicpu.op_const_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],a and 31,tmpref));
              if (a shr 5) <> 0 Then
                internalerror(2013111002);
{$elseif defined(i8086)}
              if (a shr 5) <> 0 Then
                internalerror(2013111001);
              a := a and 31;
              if a <> 0 Then
                begin
                  if (current_settings.cputype < cpu_186) and (a <> 1) then
                    begin
                      getcpuregister(list,NR_CL);
                      a_load_const_reg(list,OS_8,a,NR_CL);
                      list.concat(taicpu.op_reg_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],NR_CL,tmpref));
                      ungetcpuregister(list,NR_CL);
                    end
                  else
                    list.concat(taicpu.op_const_ref(TOpCG2AsmOp[op],TCgSize2OpSize[size],a,tmpref));
                end;
{$endif}
            end
          else internalerror(68992);
        end;
      end;


    procedure tcgx86.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
      const
{$if defined(cpu64bitalu)}
        REGCX=NR_RCX;
        REGCX_Size = OS_64;
{$elseif defined(cpu32bitalu)}
        REGCX=NR_ECX;
        REGCX_Size = OS_32;
{$elseif defined(cpu16bitalu)}
        REGCX=NR_CX;
        REGCX_Size = OS_16;
{$endif}
      var
        dstsize: topsize;
        instr:Taicpu;
      begin
        check_register_size(size,src);
        check_register_size(size,dst);
        dstsize := tcgsize2opsize[size];
        if (op=OP_MUL) and not (cs_check_overflow in current_settings.localswitches) then
          op:=OP_IMUL;
        case op of
          OP_NEG,OP_NOT:
            begin
              if src<>dst then
                a_load_reg_reg(list,size,size,src,dst);
              list.concat(taicpu.op_reg(TOpCG2AsmOp[op],dstsize,dst));
            end;
          OP_MUL,OP_DIV,OP_IDIV:
            { special stuff, needs separate handling inside code }
            { generator                                          }
            internalerror(200109233);
          OP_SHR,OP_SHL,OP_SAR,OP_ROL,OP_ROR:
            begin
              { Use ecx to load the value, that allows better coalescing }
              getcpuregister(list,REGCX);
              a_load_reg_reg(list,size,REGCX_Size,src,REGCX);
              list.concat(taicpu.op_reg_reg(Topcg2asmop[op],tcgsize2opsize[size],NR_CL,dst));
              ungetcpuregister(list,REGCX);
            end;
          else
            begin
              if reg2opsize(src) <> dstsize then
                internalerror(200109226);
              instr:=taicpu.op_reg_reg(TOpCG2AsmOp[op],dstsize,src,dst);
              list.concat(instr);
            end;
        end;
      end;


    procedure tcgx86.a_op_ref_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference; reg: TRegister);
      var
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
        check_register_size(size,reg);
        if (op=OP_MUL) and not (cs_check_overflow in current_settings.localswitches) then
          op:=OP_IMUL;
        case op of
          OP_NEG,OP_NOT,OP_IMUL:
            begin
              inherited a_op_ref_reg(list,op,size,tmpref,reg);
            end;
          OP_MUL,OP_DIV,OP_IDIV:
            { special stuff, needs separate handling inside code }
            { generator                                          }
            internalerror(200109239);
          else
            begin
              reg := makeregsize(list,reg,size);
              list.concat(taicpu.op_ref_reg(TOpCG2AsmOp[op],tcgsize2opsize[size],tmpref,reg));
            end;
        end;
      end;


    procedure tcgx86.a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize;reg: TRegister; const ref: TReference);
      var
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
        check_register_size(size,reg);
        if (op=OP_MUL) and not (cs_check_overflow in current_settings.localswitches) then
          op:=OP_IMUL;
        case op of
          OP_NEG,OP_NOT:
            begin
              if reg<>NR_NO then
                internalerror(200109237);
              list.concat(taicpu.op_ref(TOpCG2AsmOp[op],tcgsize2opsize[size],tmpref));
            end;
          OP_IMUL:
            begin
              { this one needs a load/imul/store, which is the default }
              inherited a_op_ref_reg(list,op,size,tmpref,reg);
            end;
          OP_MUL,OP_DIV,OP_IDIV:
            { special stuff, needs separate handling inside code }
            { generator                                          }
            internalerror(200109238);
          else
            begin
              list.concat(taicpu.op_reg_ref(TOpCG2AsmOp[op],tcgsize2opsize[size],reg,tmpref));
            end;
        end;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     procedure tcgx86.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: TCGSize; src, dst: TRegister);
     var
       tmpreg: tregister;
       opsize: topsize;
       l : TAsmLabel;
     begin
       { no bsf/bsr for byte }
       if srcsize in [OS_8,OS_S8] then
         begin
           tmpreg:=getintregister(list,OS_INT);
           a_load_reg_reg(list,srcsize,OS_INT,src,tmpreg);
           src:=tmpreg;
           srcsize:=OS_INT;
         end;
       { source and destination register must have the same size }
       if tcgsize2size[srcsize]<>tcgsize2size[dstsize] then
         tmpreg:=getintregister(list,srcsize)
       else
         tmpreg:=dst;
       opsize:=tcgsize2opsize[srcsize];
       if not reverse then
         list.concat(taicpu.op_reg_reg(A_BSF,opsize,src,tmpreg))
       else
         list.concat(taicpu.op_reg_reg(A_BSR,opsize,src,tmpreg));
       current_asmdata.getjumplabel(l);
       a_jmp_cond(list,OC_NE,l);
       list.concat(taicpu.op_const_reg(A_MOV,opsize,$ff,tmpreg));
       a_label(list,l);
       if tmpreg<>dst then
         a_load_reg_reg(list,srcsize,dstsize,tmpreg,dst);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     procedure tcgx86.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister);
     var
       opsize: topsize;
     begin
       opsize:=tcgsize2opsize[size];
       if not reverse then
         list.concat(taicpu.op_reg_reg(A_BSF,opsize,src,dst))
       else
         list.concat(taicpu.op_reg_reg(A_BSR,opsize,src,dst));
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

{*************** compare instructructions ****************}

    procedure tcgx86.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
      l : tasmlabel);

{$ifdef x86_64}
      var
        tmpreg : tregister;
{$endif x86_64}
      begin
{$ifdef x86_64}
        { x86_64 only supports signed 32 bits constants directly }
        if (size in [OS_S64,OS_64]) and
            ((a<low(longint)) or (a>high(longint))) then
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            a_cmp_reg_reg_label(list,size,cmp_op,tmpreg,reg,l);
            exit;
          end;
{$endif x86_64}
        if (a = 0) then
          list.concat(taicpu.op_reg_reg(A_TEST,tcgsize2opsize[size],reg,reg))
        else
          list.concat(taicpu.op_const_reg(A_CMP,tcgsize2opsize[size],a,reg));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgx86.a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference;
      l : tasmlabel);

      var
{$ifdef x86_64}
        tmpreg : tregister;
{$endif x86_64}
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
{$ifdef x86_64}
        { x86_64 only supports signed 32 bits constants directly }
        if (size in [OS_S64,OS_64]) and
           ((a<low(longint)) or (a>high(longint))) then
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            a_cmp_reg_ref_label(list,size,cmp_op,tmpreg,tmpref,l);
            exit;
          end;
{$endif x86_64}
        list.concat(taicpu.op_const_ref(A_CMP,TCgSize2OpSize[size],a,tmpref));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgx86.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;
      reg1,reg2 : tregister;l : tasmlabel);

      begin
        check_register_size(size,reg1);
        check_register_size(size,reg2);
        list.concat(taicpu.op_reg_reg(A_CMP,TCgSize2OpSize[size],reg1,reg2));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgx86.a_cmp_ref_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;const ref: treference; reg : tregister;l : tasmlabel);
      var
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
        check_register_size(size,reg);
        list.concat(taicpu.op_ref_reg(A_CMP,TCgSize2OpSize[size],tmpref,reg));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgx86.a_cmp_reg_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg : tregister;const ref: treference; l : tasmlabel);
      var
        tmpref  : treference;
      begin
        tmpref:=ref;
        make_simple_ref(list,tmpref);
        check_register_size(size,reg);
        list.concat(taicpu.op_reg_ref(A_CMP,TCgSize2OpSize[size],reg,tmpref));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgx86.a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
      var
        ai : taicpu;
      begin
        if cond=OC_None then
          ai := Taicpu.Op_sym(A_JMP,S_NO,l)
        else
          begin
            ai:=Taicpu.Op_sym(A_Jcc,S_NO,l);
            ai.SetCondition(TOpCmp2AsmCond[cond]);
          end;
        ai.is_jmp:=true;
        list.concat(ai);
      end;


     procedure tcgx86.a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel);
       var
         ai : taicpu;
         hl : tasmlabel;
         f2 : tresflags;
       begin
         hl:=nil;
         f2:=f;
         case f of
           F_FNE:
             begin
               ai:=Taicpu.op_sym(A_Jcc,S_NO,l);
               ai.SetCondition(C_P);
               ai.is_jmp:=true;
               list.concat(ai);
               f2:=F_NE;
             end;
           F_FE,F_FA,F_FAE,F_FB,F_FBE:
             begin
               { JP before JA/JAE is redundant, but it must be generated here
                 and left for peephole optimizer to remove. }
               current_asmdata.getjumplabel(hl);
               ai:=Taicpu.op_sym(A_Jcc,S_NO,hl);
               ai.SetCondition(C_P);
               ai.is_jmp:=true;
               list.concat(ai);
               f2:=FPUFlags2Flags[f];
             end;
         end;
         ai := Taicpu.op_sym(A_Jcc,S_NO,l);
         ai.SetCondition(flags_to_cond(f2));
         ai.is_jmp := true;
         list.concat(ai);
         if assigned(hl) then
           a_label(list,hl);
       end;


    procedure tcgx86.g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister);
      var
        ai : taicpu;
        f2 : tresflags;
        hreg,hreg2 : tregister;
        op: tasmop;
      begin
        hreg2:=NR_NO;
        op:=A_AND;
        f2:=f;
        case f of
          F_FE,F_FNE,F_FB,F_FBE:
            begin
              hreg2:=getintregister(list,OS_8);
              ai:=Taicpu.op_reg(A_SETcc,S_B,hreg2);
              if (f=F_FNE) then       { F_FNE means "PF or (not ZF)" }
                begin
                  ai.setcondition(C_P);
                  op:=A_OR;
                end
              else
                ai.setcondition(C_NP);
              list.concat(ai);
              f2:=FPUFlags2Flags[f];
            end;
          F_FA,F_FAE:                 { These do not need PF check }
            f2:=FPUFlags2Flags[f];
        end;
        hreg:=makeregsize(list,reg,OS_8);
        ai:=Taicpu.op_reg(A_SETcc,S_B,hreg);
        ai.setcondition(flags_to_cond(f2));
        list.concat(ai);
        if (hreg2<>NR_NO) then
          list.concat(taicpu.op_reg_reg(op,S_B,hreg2,hreg));
        if reg<>hreg then
          a_load_reg_reg(list,OS_8,size,hreg,reg);
      end;


    procedure tcgx86.g_flags2ref(list: TAsmList; size: TCgSize; const f: tresflags; const ref: TReference);
      var
        ai : taicpu;
        tmpref  : treference;
        f2 : tresflags;
      begin
        f2:=f;
        case f of
          F_FE,F_FNE,F_FB,F_FBE:
            begin
              inherited g_flags2ref(list,size,f,ref);
              exit;
            end;
          F_FA,F_FAE:
            f2:=FPUFlags2Flags[f];
        end;
         tmpref:=ref;
         make_simple_ref(list,tmpref);
         if not(size in [OS_8,OS_S8]) then
           a_load_const_ref(list,size,0,tmpref);
         ai:=Taicpu.op_ref(A_SETcc,S_B,tmpref);
         ai.setcondition(flags_to_cond(f2));
         list.concat(ai);
{$ifndef cpu64bitalu}
         if size in [OS_S64,OS_64] then
           begin
             inc(tmpref.offset,4);
             a_load_const_ref(list,OS_32,0,tmpref);
           end;
{$endif cpu64bitalu}
      end;


{ ************* concatcopy ************ }

    procedure Tcgx86.g_concatcopy(list:TAsmList;const source,dest:Treference;len:tcgint);

    const
{$if defined(cpu64bitalu)}
        REGCX=NR_RCX;
        REGSI=NR_RSI;
        REGDI=NR_RDI;
        copy_len_sizes = [1, 2, 4, 8];
        push_segment_size = S_L;
{$elseif defined(cpu32bitalu)}
        REGCX=NR_ECX;
        REGSI=NR_ESI;
        REGDI=NR_EDI;
        copy_len_sizes = [1, 2, 4];
        push_segment_size = S_L;
{$elseif defined(cpu16bitalu)}
        REGCX=NR_CX;
        REGSI=NR_SI;
        REGDI=NR_DI;
        copy_len_sizes = [1, 2, 4]; { 4 is included here, because it's still more
          efficient to use copy_move instead of copy_string for copying 4 bytes }
        push_segment_size = S_W;
{$endif}

    type  copymode=(copy_move,copy_mmx,copy_string,copy_mm,copy_avx);

    var srcref,dstref,tmpref:Treference;
        r,r0,r1,r2,r3:Tregister;
        helpsize:tcgint;
        copysize:byte;
        cgsize:Tcgsize;
        cm:copymode;
        saved_ds,saved_es: Boolean;

    begin
      cm:=copy_move;
      helpsize:=3*sizeof(aword);
      if cs_opt_size in current_settings.optimizerswitches then
        helpsize:=2*sizeof(aword);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef i8086}
      { avx helps only to reduce size, using it in general does at least not help on
        an i7-4770 (FK) }
      if (CPUX86_HAS_AVXUNIT in cpu_capabilities[current_settings.cputype]) and
        // (cs_opt_size in current_settings.optimizerswitches) and
         ((len=8) or (len=16) or (len=24) or (len=32) { or (len=40) or (len=48)}) then
         cm:=copy_avx
      else
{$ifdef dummy}
      { I'am not sure what CPUs would benefit from using sse instructions for moves (FK) }
      if
{$ifdef x86_64}
        ((current_settings.fputype>=fpu_sse64)
{$else x86_64}
        ((current_settings.fputype>=fpu_sse)
{$endif x86_64}
          or (CPUX86_HAS_SSEUNIT in cpu_capabilities[current_settings.cputype])) and
         ((len=8) or (len=16) or (len=24) or (len=32) or (len=40) or (len=48)) then
         cm:=copy_mm
      else
{$endif dummy}
{$endif i8086}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      if (cs_mmx in current_settings.localswitches) and
         not(pi_uses_fpu in current_procinfo.flags) and
         ((len=8) or (len=16) or (len=24) or (len=32)) then
        cm:=copy_mmx;
      if (len>helpsize) then
        cm:=copy_string;
      if (cs_opt_size in current_settings.optimizerswitches) and
         not((len<=16) and (cm in [copy_mmx,copy_mm,copy_avx])) and
         not(len in copy_len_sizes) then
        cm:=copy_string;
{$ifndef i8086}
      if (source.segment<>NR_NO) or
         (dest.segment<>NR_NO) then
        cm:=copy_string;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif not i8086}
=======
      if (source.segment<>NR_NO) or
         (dest.segment<>NR_NO) then
        cm:=copy_string;
>>>>>>> graemeg/fixes_2_2
=======
      if (source.segment<>NR_NO) or
         (dest.segment<>NR_NO) then
        cm:=copy_string;
>>>>>>> origin/fixes_2_2
      case cm of
        copy_move:
          begin
            dstref:=dest;
            srcref:=source;
            copysize:=sizeof(aint);
            cgsize:=int_cgsize(copysize);
            while len<>0 do
              begin
                if len<2 then
                  begin
                    copysize:=1;
                    cgsize:=OS_8;
                  end
                else if len<4 then
                  begin
                    copysize:=2;
                    cgsize:=OS_16;
                  end
{$if defined(cpu32bitalu) or defined(cpu64bitalu)}
                else if len<8 then
                  begin
                    copysize:=4;
                    cgsize:=OS_32;
                  end
<<<<<<< HEAD
<<<<<<< HEAD
{$endif cpu32bitalu or cpu64bitalu}
{$ifdef cpu64bitalu}
=======
{$ifdef cpu64bit}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef cpu64bit}
>>>>>>> origin/fixes_2_2
                else if len<16 then
                  begin
                    copysize:=8;
                    cgsize:=OS_64;
                  end
{$endif}
                ;
                dec(len,copysize);
                r:=getintregister(list,cgsize);
                a_load_ref_reg(list,cgsize,cgsize,srcref,r);
                a_load_reg_ref(list,cgsize,cgsize,r,dstref);
                inc(srcref.offset,copysize);
                inc(dstref.offset,copysize);
              end;
          end;

        copy_mmx:
          begin
            dstref:=dest;
            srcref:=source;
            r0:=getmmxregister(list);
            r1:=NR_NO;
            r2:=NR_NO;
            r3:=NR_NO;
            a_loadmm_ref_reg(list,OS_M64,OS_M64,srcref,r0,nil);
            if len>=16 then
              begin
                inc(srcref.offset,8);
                r1:=getmmxregister(list);
                a_loadmm_ref_reg(list,OS_M64,OS_M64,srcref,r1,nil);
              end;
            if len>=24 then
              begin
                inc(srcref.offset,8);
                r2:=getmmxregister(list);
                a_loadmm_ref_reg(list,OS_M64,OS_M64,srcref,r2,nil);
              end;
            if len>=32 then
              begin
                inc(srcref.offset,8);
                r3:=getmmxregister(list);
                a_loadmm_ref_reg(list,OS_M64,OS_M64,srcref,r3,nil);
              end;
            a_loadmm_reg_ref(list,OS_M64,OS_M64,r0,dstref,nil);
            if len>=16 then
              begin
                inc(dstref.offset,8);
                a_loadmm_reg_ref(list,OS_M64,OS_M64,r1,dstref,nil);
              end;
            if len>=24 then
              begin
                inc(dstref.offset,8);
                a_loadmm_reg_ref(list,OS_M64,OS_M64,r2,dstref,nil);
              end;
            if len>=32 then
              begin
                inc(dstref.offset,8);
                a_loadmm_reg_ref(list,OS_M64,OS_M64,r3,dstref,nil);
              end;
          end;

        copy_mm:
          begin
            dstref:=dest;
            srcref:=source;
            r0:=NR_NO;
            r1:=NR_NO;
            r2:=NR_NO;
            r3:=NR_NO;
            if len>=16 then
              begin
                r0:=getmmregister(list,OS_M128);
                a_loadmm_ref_reg(list,OS_M128,OS_M128,srcref,r0,nil);
                inc(srcref.offset,16);
              end;
            if len>=32 then
              begin
                r1:=getmmregister(list,OS_M128);
                a_loadmm_ref_reg(list,OS_M128,OS_M128,srcref,r1,nil);
                inc(srcref.offset,16);
              end;
            if len>=48 then
              begin
                r2:=getmmregister(list,OS_M128);
                a_loadmm_ref_reg(list,OS_M128,OS_M128,srcref,r2,nil);
                inc(srcref.offset,16);
              end;
            if (len=8) or (len=24) or (len=40) then
              begin
                r3:=getmmregister(list,OS_M64);
                a_loadmm_ref_reg(list,OS_M64,OS_M64,srcref,r3,nil);
              end;

            if len>=16 then
              begin
                a_loadmm_reg_ref(list,OS_M128,OS_M128,r0,dstref,nil);
                inc(dstref.offset,16);
              end;
            if len>=32 then
              begin
                a_loadmm_reg_ref(list,OS_M128,OS_M128,r1,dstref,nil);
                inc(dstref.offset,16);
              end;
            if len>=48 then
              begin
                a_loadmm_reg_ref(list,OS_M128,OS_M128,r2,dstref,nil);
                inc(dstref.offset,16);
              end;
            if (len=8) or (len=24) or (len=40) then
              begin
                a_loadmm_reg_ref(list,OS_M64,OS_M64,r3,dstref,nil);
              end;
          end;

        copy_avx:
          begin
            dstref:=dest;
            srcref:=source;
            r0:=NR_NO;
            r1:=NR_NO;
            r2:=NR_NO;
            r3:=NR_NO;
            if len>=16 then
              begin
                r0:=getmmregister(list,OS_M128);
                { we want to force the use of vmovups, so do not use a_loadmm_ref_reg }
                tmpref:=srcref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_ref_reg(A_VMOVUPS,S_NO,tmpref,r0));
                inc(srcref.offset,16);
              end;
            if len>=32 then
              begin
                r1:=getmmregister(list,OS_M128);
                tmpref:=srcref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_ref_reg(A_VMOVUPS,S_NO,tmpref,r1));
                inc(srcref.offset,16);
              end;
            if len>=48 then
              begin
                r2:=getmmregister(list,OS_M128);
                tmpref:=srcref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_ref_reg(A_VMOVUPS,S_NO,tmpref,r2));
                inc(srcref.offset,16);
              end;
            if (len=8) or (len=24) or (len=40) then
              begin
                r3:=getmmregister(list,OS_M64);
                tmpref:=srcref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_ref_reg(A_VMOVSD,S_NO,tmpref,r3));
              end;

            if len>=16 then
              begin
                tmpref:=dstref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_reg_ref(A_VMOVUPS,S_NO,r0,tmpref));
                inc(dstref.offset,16);
              end;
            if len>=32 then
              begin
                tmpref:=dstref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_reg_ref(A_VMOVUPS,S_NO,r1,tmpref));
                inc(dstref.offset,16);
              end;
            if len>=48 then
              begin
                tmpref:=dstref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_reg_ref(A_VMOVUPS,S_NO,r2,tmpref));
                inc(dstref.offset,16);
              end;
            if (len=8) or (len=24) or (len=40) then
              begin
                tmpref:=dstref;
                make_simple_ref(list,tmpref);
                list.concat(taicpu.op_reg_ref(A_VMOVSD,S_NO,r3,tmpref));
              end;
          end
        else {copy_string, should be a good fallback in case of unhandled}
          begin
            getcpuregister(list,REGDI);
<<<<<<< HEAD
<<<<<<< HEAD
            if (dest.segment=NR_NO) and
               (segment_regs_equal(NR_SS,NR_DS) or ((dest.base<>NR_BP) and (dest.base<>NR_SP))) then
              begin
                a_loadaddr_ref_reg(list,dest,REGDI);
                saved_es:=false;
{$ifdef volatile_es}
                list.concat(taicpu.op_reg(A_PUSH,push_segment_size,NR_DS));
                list.concat(taicpu.op_reg(A_POP,push_segment_size,NR_ES));
{$endif volatile_es}
              end
=======
            if (dest.segment=NR_NO) then
              a_loadaddr_ref_reg(list,dest,REGDI)
>>>>>>> graemeg/fixes_2_2
=======
            if (dest.segment=NR_NO) then
              a_loadaddr_ref_reg(list,dest,REGDI)
>>>>>>> origin/fixes_2_2
            else
              begin
                dstref:=dest;
                dstref.segment:=NR_NO;
                a_loadaddr_ref_reg(list,dstref,REGDI);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef volatile_es}
                saved_es:=false;
{$else volatile_es}
                list.concat(taicpu.op_reg(A_PUSH,push_segment_size,NR_ES));
                saved_es:=true;
{$endif volatile_es}
                if dest.segment<>NR_NO then
                  list.concat(taicpu.op_reg(A_PUSH,push_segment_size,dest.segment))
                else if (dest.base=NR_BP) or (dest.base=NR_SP) then
                  list.concat(taicpu.op_reg(A_PUSH,push_segment_size,NR_SS))
                else
                  internalerror(2014040401);
                list.concat(taicpu.op_reg(A_POP,push_segment_size,NR_ES));
              end;
            getcpuregister(list,REGSI);
            if ((source.segment=NR_NO) and (segment_regs_equal(NR_SS,NR_DS) or ((source.base<>NR_BP) and (source.base<>NR_SP)))) or
               (is_segment_reg(source.segment) and segment_regs_equal(source.segment,NR_DS)) then
              begin
                srcref:=source;
                srcref.segment:=NR_NO;
                a_loadaddr_ref_reg(list,srcref,REGSI);
                saved_ds:=false;
              end
=======
=======
>>>>>>> origin/fixes_2_2
                list.concat(taicpu.op_reg(A_PUSH,S_L,NR_ES));
                list.concat(taicpu.op_reg(A_PUSH,S_L,dest.segment));
                list.concat(taicpu.op_reg(A_POP,S_L,NR_ES));
              end;
            getcpuregister(list,REGSI);
            if (source.segment=NR_NO) then
              a_loadaddr_ref_reg(list,source,REGSI)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            else
              begin
                srcref:=source;
                srcref.segment:=NR_NO;
                a_loadaddr_ref_reg(list,srcref,REGSI);
<<<<<<< HEAD
<<<<<<< HEAD
                list.concat(taicpu.op_reg(A_PUSH,push_segment_size,NR_DS));
                saved_ds:=true;
                if source.segment<>NR_NO then
                  list.concat(taicpu.op_reg(A_PUSH,push_segment_size,source.segment))
                else if (source.base=NR_BP) or (source.base=NR_SP) then
                  list.concat(taicpu.op_reg(A_PUSH,push_segment_size,NR_SS))
                else
                  internalerror(2014040402);
                list.concat(taicpu.op_reg(A_POP,push_segment_size,NR_DS));
              end;

            getcpuregister(list,REGCX);
            if ts_cld in current_settings.targetswitches then
              list.concat(Taicpu.op_none(A_CLD,S_NO));
=======
=======
>>>>>>> origin/fixes_2_2
                list.concat(taicpu.op_reg(A_PUSH,S_L,NR_DS));
                list.concat(taicpu.op_reg(A_PUSH,S_L,source.segment));
                list.concat(taicpu.op_reg(A_POP,S_L,NR_DS));
              end;

            getcpuregister(list,REGCX);
{$ifdef i386}
           list.concat(Taicpu.op_none(A_CLD,S_NO));
{$endif i386}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            if (cs_opt_size in current_settings.optimizerswitches) and
               (len>sizeof(aint)+(sizeof(aint) div 2)) then
              begin
                a_load_const_reg(list,OS_INT,len,REGCX);
                list.concat(Taicpu.op_none(A_REP,S_NO));
                list.concat(Taicpu.op_none(A_MOVSB,S_NO));
              end
            else
              begin
                helpsize:=len div sizeof(aint);
                len:=len mod sizeof(aint);
                if helpsize>1 then
                  begin
                    a_load_const_reg(list,OS_INT,helpsize,REGCX);
                    list.concat(Taicpu.op_none(A_REP,S_NO));
                  end;
                if helpsize>0 then
                  begin
{$if defined(cpu64bitalu)}
                    list.concat(Taicpu.op_none(A_MOVSQ,S_NO))
{$elseif defined(cpu32bitalu)}
                    list.concat(Taicpu.op_none(A_MOVSD,S_NO));
{$elseif defined(cpu16bitalu)}
                    list.concat(Taicpu.op_none(A_MOVSW,S_NO));
{$endif}
                  end;
                if len>=4 then
                  begin
                    dec(len,4);
                    list.concat(Taicpu.op_none(A_MOVSD,S_NO));
                  end;
                if len>=2 then
                  begin
                    dec(len,2);
                    list.concat(Taicpu.op_none(A_MOVSW,S_NO));
                  end;
                if len=1 then
                  list.concat(Taicpu.op_none(A_MOVSB,S_NO));
              end;
            ungetcpuregister(list,REGCX);
            ungetcpuregister(list,REGSI);
            ungetcpuregister(list,REGDI);
<<<<<<< HEAD
<<<<<<< HEAD
            if saved_ds then
              list.concat(taicpu.op_reg(A_POP,push_segment_size,NR_DS));
            if saved_es then
              list.concat(taicpu.op_reg(A_POP,push_segment_size,NR_ES));
=======
=======
>>>>>>> origin/fixes_2_2
            if (source.segment<>NR_NO) then
              list.concat(taicpu.op_reg(A_POP,S_L,NR_DS));
            if (dest.segment<>NR_NO) then
              list.concat(taicpu.op_reg(A_POP,S_L,NR_ES));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
        end;
    end;


{****************************************************************************
                              Entry/Exit Code Helpers
****************************************************************************}

    procedure tcgx86.g_profilecode(list : TAsmList);

      var
        pl           : tasmlabel;
        mcountprefix : String[4];

      begin
        case target_info.system of
        {$ifndef NOTARGETWIN}
           system_i386_win32,
        {$endif}
           system_i386_freebsd,
           system_i386_netbsd,
//         system_i386_openbsd,
           system_i386_wdosx :
             begin
                Case target_info.system Of
                 system_i386_freebsd : mcountprefix:='.';
                 system_i386_netbsd : mcountprefix:='__';
//               system_i386_openbsd : mcountprefix:='.';
                else
                 mcountPrefix:='';
                end;
                current_asmdata.getaddrlabel(pl);
                new_section(list,sec_data,lower(current_procinfo.procdef.mangledname),sizeof(pint));
                list.concat(Tai_label.Create(pl));
                list.concat(Tai_const.Create_32bit(0));
                new_section(list,sec_code,lower(current_procinfo.procdef.mangledname),0);
                list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_EDX));
                list.concat(Taicpu.Op_sym_ofs_reg(A_MOV,S_L,pl,0,NR_EDX));
                a_call_name(list,target_info.Cprefix+mcountprefix+'mcount',false);
                list.concat(Taicpu.Op_reg(A_POP,S_L,NR_EDX));
             end;

           system_i386_linux:
             a_call_name(list,target_info.Cprefix+'mcount',false);

           system_i386_go32v2,system_i386_watcom:
             begin
               a_call_name(list,'MCOUNT',false);
             end;
           system_x86_64_linux,
           system_x86_64_darwin,
           system_x86_64_iphonesim:
             begin
               a_call_name(list,'mcount',false);
             end;
        end;
      end;


    procedure tcgx86.g_stackpointer_alloc(list : TAsmList;localsize : longint);

      procedure decrease_sp(a : tcgint);
        var
          href : treference;
        begin
          reference_reset_base(href,NR_STACK_POINTER_REG,-a,0);
          { normally, lea is a better choice than a sub to adjust the stack pointer }
          list.concat(Taicpu.op_ref_reg(A_LEA,TCGSize2OpSize[OS_ADDR],href,NR_STACK_POINTER_REG));
        end;

{$ifdef x86}
{$ifndef NOTARGETWIN}
      var
        href : treference;
        i : integer;
        again : tasmlabel;
{$endif NOTARGETWIN}
{$endif x86}
      begin
        if localsize>0 then
         begin
{$ifdef i386}
{$ifndef NOTARGETWIN}
           { windows guards only a few pages for stack growing,
             so we have to access every page first              }
           if (target_info.system in [system_i386_win32,system_i386_wince]) and
              (localsize>=winstackpagesize) then
             begin
               if localsize div winstackpagesize<=5 then
                 begin
                    decrease_sp(localsize-4);
                    for i:=1 to localsize div winstackpagesize do
                      begin
                         reference_reset_base(href,NR_ESP,localsize-i*winstackpagesize,4);
                         list.concat(Taicpu.op_reg_ref(A_MOV,S_L,NR_EAX,href));
                      end;
                    list.concat(Taicpu.op_reg(A_PUSH,S_L,NR_EAX));
                 end
               else
                 begin
                    current_asmdata.getjumplabel(again);
                    { Using a_reg_alloc instead of getcpuregister, so this procedure
                      does not change "used_in_proc" state of EDI and therefore can be
                      called after saving registers with "push" instruction
                      without creating an unbalanced "pop edi" in epilogue }
                    a_reg_alloc(list,NR_EDI);
                    list.concat(Taicpu.op_reg(A_PUSH,S_L,NR_EDI));
                    list.concat(Taicpu.op_const_reg(A_MOV,S_L,localsize div winstackpagesize,NR_EDI));
                    a_label(list,again);
                    decrease_sp(winstackpagesize-4);
                    list.concat(Taicpu.op_reg(A_PUSH,S_L,NR_EAX));
                    if UseIncDec then
                      list.concat(Taicpu.op_reg(A_DEC,S_L,NR_EDI))
                    else
                      list.concat(Taicpu.op_const_reg(A_SUB,S_L,1,NR_EDI));
                    a_jmp_cond(list,OC_NE,again);
                    decrease_sp(localsize mod winstackpagesize-4);
                    reference_reset_base(href,NR_ESP,localsize-4,4);
                    list.concat(Taicpu.op_ref_reg(A_MOV,S_L,href,NR_EDI));
                    a_reg_dealloc(list,NR_EDI);
                 end
             end
           else
{$endif NOTARGETWIN}
{$endif i386}
{$ifdef x86_64}
{$ifndef NOTARGETWIN}
           { windows guards only a few pages for stack growing,
             so we have to access every page first              }
           if (target_info.system=system_x86_64_win64) and
              (localsize>=winstackpagesize) then
             begin
               if localsize div winstackpagesize<=5 then
                 begin
                    decrease_sp(localsize);
                    for i:=1 to localsize div winstackpagesize do
                      begin
                         reference_reset_base(href,NR_RSP,localsize-i*winstackpagesize+4,4);
                         list.concat(Taicpu.op_reg_ref(A_MOV,S_L,NR_EAX,href));
                      end;
                    reference_reset_base(href,NR_RSP,0,4);
                    list.concat(Taicpu.op_reg_ref(A_MOV,S_L,NR_EAX,href));
                 end
               else
                 begin
                    current_asmdata.getjumplabel(again);
                    getcpuregister(list,NR_R10);
                    list.concat(Taicpu.op_const_reg(A_MOV,S_Q,localsize div winstackpagesize,NR_R10));
                    a_label(list,again);
                    decrease_sp(winstackpagesize);
                    reference_reset_base(href,NR_RSP,0,4);
                    list.concat(Taicpu.op_reg_ref(A_MOV,S_L,NR_EAX,href));
                    if UseIncDec then
                      list.concat(Taicpu.op_reg(A_DEC,S_Q,NR_R10))
                    else
                      list.concat(Taicpu.op_const_reg(A_SUB,S_Q,1,NR_R10));
                    a_jmp_cond(list,OC_NE,again);
                    decrease_sp(localsize mod winstackpagesize);
                    ungetcpuregister(list,NR_R10);
                 end
             end
           else
{$endif NOTARGETWIN}
{$endif x86_64}
            decrease_sp(localsize);
         end;
      end;


    procedure tcgx86.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
        stackmisalignment: longint;
        regsize: longint;
{$ifdef i8086}
        dgroup: treference;
        fardataseg: treference;
{$endif i8086}

      procedure push_regs;
        var
          r: longint;
          usedregs: tcpuregisterset;
        begin
          regsize:=0;
          usedregs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(current_procinfo.procdef.proccalloption);
          for r := low(saved_standard_registers) to high(saved_standard_registers) do
            if saved_standard_registers[r] in usedregs then
              begin
                inc(regsize,sizeof(aint));
                list.concat(Taicpu.Op_reg(A_PUSH,tcgsize2opsize[OS_ADDR],newreg(R_INTREGISTER,saved_standard_registers[r],R_SUBWHOLE)));
              end;
        end;

      begin
{$ifdef i8086}
        { Win16 callback/exported proc prologue support.
          Since callbacks can be called from different modules, DS on entry may be
          initialized with the data segment of a different module, so we need to
          get ours. But we can't do

                      push ds
                      mov ax, dgroup
                      mov ds, ax

          because code segments are shared between different instances of the same
          module (which have different instances of the current program's data segment),
          so the same 'mov ax, dgroup' instruction will be used for all instances
          of the program and it will load the same segment into ax.

          So, the standard win16 prologue looks like this:

                      mov ax, ds
                      nop
                      inc bp
                      push bp
                      mov bp, sp
                      push ds
                      mov ds, ax

          By default, this does nothing, except wasting a few extra machine cycles and
          destroying ax in the process. However, Windows checks the first three bytes
          of every exported function and if they are 'mov ax,ds/nop', they are replaced
          with nop/nop/nop. Then the MakeProcInstance api call should be used to create
          a thunk that loads ds for the current program instance in ax before calling
          the routine.

          And now the fun part comes: somebody (Michael Geary) figured out that all this
          crap was unnecessary, because in Win16 exe modules, we always have DS=SS, so we
          can simply initialize DS from SS :) And then calling MakeProcInstance becomes
          unnecessary. This is what "smart callbacks" (cs_win16_smartcallbacks) do. However,
          this only works for exe files, not for dlls, because dlls run with DS<>SS. There's
          another solution for dlls - since win16 dlls only have a single instance of their
          data segment, we can initialize ds from dgroup. However, there's not a single
          solution for both exe and dlls, so we don't know what to use e.g. in a unit. So,
          that's why there's still an option to turn smart callbacks off and go the
          MakeProcInstance way.

          Additional details here: http://www.geary.com/fixds.html }
        if (current_settings.x86memorymodel<>mm_huge) and
           (po_exports in current_procinfo.procdef.procoptions) and
           (target_info.system=system_i8086_win16) then
          begin
            if cs_win16_smartcallbacks in current_settings.moduleswitches then
              list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_SS,NR_AX))
            else
              list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_DS,NR_AX));
            list.concat(Taicpu.op_none(A_NOP));
          end
        { interrupt support for i8086 }
        else if po_interrupt in current_procinfo.procdef.procoptions then
          begin
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_AX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_BX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_CX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_DX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_SI));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_DI));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_DS));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_ES));
            if current_settings.x86memorymodel=mm_tiny then
              begin
                { in the tiny memory model, we can't use dgroup, because that
                  adds a relocation entry to the .exe and we can't produce a
                  .com file (because they don't support relactions), so instead
                  we initialize DS from CS. }
                if cs_opt_size in current_settings.optimizerswitches then
                  begin
                    list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_CS));
                    list.concat(Taicpu.Op_reg(A_POP,S_W,NR_DS));
                  end
                else
                  begin
                    list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_CS,NR_AX));
                    list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_AX,NR_DS));
                  end;
              end
            else if current_settings.x86memorymodel=mm_huge then
              begin
                reference_reset(fardataseg,0);
                fardataseg.refaddr:=addr_fardataseg;
                list.concat(Taicpu.Op_ref_reg(A_MOV,S_W,fardataseg,NR_AX));
                list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_AX,NR_DS));
              end
            else
              begin
                reference_reset(dgroup,0);
                dgroup.refaddr:=addr_dgroup;
                list.concat(Taicpu.Op_ref_reg(A_MOV,S_W,dgroup,NR_AX));
                list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_AX,NR_DS));
              end;
          end;
{$endif i8086}
{$ifdef i386}
        { interrupt support for i386 }
        if (po_interrupt in current_procinfo.procdef.procoptions) and
           { this messes up stack alignment }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           not(target_info.system in [system_i386_darwin,system_i386_iphonesim,system_i386_android]) then
=======
           not(target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
>>>>>>> graemeg/cpstrnew
=======
           not(target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
>>>>>>> graemeg/cpstrnew
=======
           not(target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
>>>>>>> graemeg/cpstrnew
=======
           not(target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
>>>>>>> origin/cpstrnew
          begin
            { .... also the segment registers }
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_GS));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_FS));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_ES));
            list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_DS));
            { save the registers of an interrupt procedure }
            list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_EDI));
            list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_ESI));
            list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_EDX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_ECX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_EBX));
            list.concat(Taicpu.Op_reg(A_PUSH,S_L,NR_EAX));
          end;
{$endif i386}

        { save old framepointer }
        if not nostackframe then
          begin
            { return address }
            stackmisalignment := sizeof(pint);
            list.concat(tai_regalloc.alloc(current_procinfo.framepointer,nil));
            if current_procinfo.framepointer=NR_STACK_POINTER_REG then
              begin
{$ifdef i386}
                if (not paramanager.use_fixed_stack) then
                  push_regs;
{$endif i386}
                CGmessage(cg_d_stackframe_omited);
              end
            else
              begin
{$ifdef i8086}
                if ((ts_x86_far_procs_push_odd_bp in current_settings.targetswitches) or
                    ((po_exports in current_procinfo.procdef.procoptions) and
                     (target_info.system=system_i8086_win16))) and
                    is_proc_far(current_procinfo.procdef) then
                  cg.a_op_const_reg(list,OP_ADD,OS_ADDR,1,current_procinfo.framepointer);
{$endif i8086}
                { push <frame_pointer> }
                inc(stackmisalignment,sizeof(pint));
                include(rg[R_INTREGISTER].preserved_by_proc,RS_FRAME_POINTER_REG);
                list.concat(Taicpu.op_reg(A_PUSH,tcgsize2opsize[OS_ADDR],NR_FRAME_POINTER_REG));
                { Return address and FP are both on stack }
                current_asmdata.asmcfi.cfa_def_cfa_offset(list,2*sizeof(pint));
                current_asmdata.asmcfi.cfa_offset(list,NR_FRAME_POINTER_REG,-(2*sizeof(pint)));
                if current_procinfo.procdef.proctypeoption<>potype_exceptfilter then
                  list.concat(Taicpu.op_reg_reg(A_MOV,tcgsize2opsize[OS_ADDR],NR_STACK_POINTER_REG,NR_FRAME_POINTER_REG))
                else
                  begin
                    push_regs;
                    gen_load_frame_for_exceptfilter(list);
                    { Need only as much stack space as necessary to do the calls.
                      Exception filters don't have own local vars, and temps are 'mapped'
                      to the parent procedure.
                      maxpushedparasize is already aligned at least on x86_64. }
                    localsize:=current_procinfo.maxpushedparasize;
                  end;
                current_asmdata.asmcfi.cfa_def_cfa_register(list,NR_FRAME_POINTER_REG);
              end;

            { allocate stackframe space }
            if (localsize<>0) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               ((target_info.stackalign>sizeof(pint)) and
=======
               ((target_info.system in systems_need_16_byte_stack_alignment) and
>>>>>>> graemeg/cpstrnew
=======
               ((target_info.system in systems_need_16_byte_stack_alignment) and
>>>>>>> graemeg/cpstrnew
=======
               ((target_info.system in systems_need_16_byte_stack_alignment) and
>>>>>>> graemeg/cpstrnew
=======
               ((target_info.system in systems_need_16_byte_stack_alignment) and
>>>>>>> origin/cpstrnew
=======
               ((target_info.system in [system_i386_darwin,
                 system_x86_64_win64,system_x86_64_linux,system_x86_64_freebsd]) and
>>>>>>> graemeg/fixes_2_2
=======
               ((target_info.system in [system_i386_darwin,
                 system_x86_64_win64,system_x86_64_linux,system_x86_64_freebsd]) and
>>>>>>> origin/fixes_2_2
                (stackmisalignment <> 0) and
                ((pi_do_call in current_procinfo.flags) or
                 (po_assembler in current_procinfo.procdef.procoptions))) then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                if target_info.stackalign>sizeof(pint) then
                  localsize := align(localsize+stackmisalignment,target_info.stackalign)-stackmisalignment;
                g_stackpointer_alloc(list,localsize);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                if (target_info.system in systems_need_16_byte_stack_alignment) then
=======
                if (target_info.system in [system_i386_darwin,
                      system_x86_64_win64,system_x86_64_linux,system_x86_64_freebsd]) then
>>>>>>> graemeg/fixes_2_2
=======
                if (target_info.system in [system_i386_darwin,
                      system_x86_64_win64,system_x86_64_linux,system_x86_64_freebsd]) then
>>>>>>> origin/fixes_2_2
                  localsize := align(localsize+stackmisalignment,16)-stackmisalignment;
                cg.g_stackpointer_alloc(list,localsize);
>>>>>>> graemeg/cpstrnew
                if current_procinfo.framepointer=NR_STACK_POINTER_REG then
                  current_asmdata.asmcfi.cfa_def_cfa_offset(list,localsize+sizeof(pint));
                current_procinfo.final_localsize:=localsize;
              end;

{$ifdef i8086}
              { win16 exported proc prologue follow-up (see the huge comment above for details) }
              if (current_settings.x86memorymodel<>mm_huge) and
                 (po_exports in current_procinfo.procdef.procoptions) and
                 (target_info.system=system_i8086_win16) then
                begin
                  list.concat(Taicpu.op_reg(A_PUSH,S_W,NR_DS));
                  list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_AX,NR_DS));
                end
              else if (current_settings.x86memorymodel=mm_huge) and
                      not (po_interrupt in current_procinfo.procdef.procoptions) then
                begin
                  list.concat(Taicpu.op_reg(A_PUSH,S_W,NR_DS));
                  reference_reset(fardataseg,0);
                  fardataseg.refaddr:=addr_fardataseg;
                  list.concat(Taicpu.Op_ref_reg(A_MOV,S_W,fardataseg,NR_AX));
                  list.concat(Taicpu.Op_reg_reg(A_MOV,S_W,NR_AX,NR_DS));
                end;
            { SI and DI are volatile in the BP7 and FPC's pascal calling convention,
              but must be preserved in Microsoft C's pascal calling convention, and
              since Windows is compiled with Microsoft compilers, these registers
              must be saved for exported procedures (BP7 for Win16 also does this). }
            if (po_exports in current_procinfo.procdef.procoptions) and
               (target_info.system=system_i8086_win16) then
              begin
                list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_SI));
                list.concat(Taicpu.Op_reg(A_PUSH,S_W,NR_DI));
              end;
{$endif i8086}

{$ifdef i386}
            if (not paramanager.use_fixed_stack) and
               (current_procinfo.framepointer<>NR_STACK_POINTER_REG) and
               (current_procinfo.procdef.proctypeoption<>potype_exceptfilter) then
              begin
                regsize:=0;
                push_regs;
                reference_reset_base(current_procinfo.save_regs_ref,
                  current_procinfo.framepointer,
                  -(localsize+regsize),sizeof(aint));
              end;
{$endif i386}
          end;
      end;


    procedure tcgx86.g_save_registers(list: TAsmList);
      begin
{$ifdef i386}
        if paramanager.use_fixed_stack then
{$endif i386}
          inherited g_save_registers(list);
      end;


    procedure tcgx86.g_restore_registers(list: TAsmList);
      begin
{$ifdef i386}
        if paramanager.use_fixed_stack then
{$endif i386}
          inherited g_restore_registers(list);
      end;


    procedure tcgx86.internal_restore_regs(list: TAsmList; use_pop: boolean);
      var
        r: longint;
        hreg: tregister;
        href: treference;
        usedregs: tcpuregisterset;
      begin
        href:=current_procinfo.save_regs_ref;
        usedregs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(current_procinfo.procdef.proccalloption);
        for r:=high(saved_standard_registers) downto low(saved_standard_registers) do
          if saved_standard_registers[r] in usedregs then
            begin
              hreg:=newreg(R_INTREGISTER,saved_standard_registers[r],R_SUBWHOLE);
              { Allocate register so the optimizer does not remove the load }
              a_reg_alloc(list,hreg);
              if use_pop then
                list.concat(Taicpu.Op_reg(A_POP,tcgsize2opsize[OS_ADDR],hreg))
              else
                begin
                  a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,hreg);
                  inc(href.offset,sizeof(aint));
                end;
            end;
      end;


    procedure tcgx86.generate_leave(list: TAsmList);
      begin
        if UseLeave then
          list.concat(taicpu.op_none(A_LEAVE,S_NO))
        else
          begin
{$if defined(x86_64)}
            list.Concat(taicpu.op_reg_reg(A_MOV,S_Q,NR_RBP,NR_RSP));
            list.Concat(taicpu.op_reg(A_POP,S_Q,NR_RBP));
{$elseif defined(i386)}
            list.Concat(taicpu.op_reg_reg(A_MOV,S_L,NR_EBP,NR_ESP));
            list.Concat(taicpu.op_reg(A_POP,S_L,NR_EBP));
{$elseif defined(i8086)}
            list.Concat(taicpu.op_reg_reg(A_MOV,S_W,NR_BP,NR_SP));
            list.Concat(taicpu.op_reg(A_POP,S_W,NR_BP));
{$endif}
          end;
      end;


    { produces if necessary overflowcode }
    procedure tcgx86.g_overflowcheck(list: TAsmList; const l:tlocation;def:tdef);
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
           cond:=C_NO
         else
           cond:=C_NB;
         ai:=Taicpu.Op_Sym(A_Jcc,S_NO,hl);
         ai.SetCondition(cond);
         ai.is_jmp:=true;
         list.concat(ai);

         a_call_name(list,'FPC_OVERFLOW',false);
         a_label(list,hl);
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure tcgx86.g_external_wrapper(list: TAsmList; procdef: tprocdef; const externalname: string);
      var
        ref : treference;
        sym : tasmsymbol;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
       if (target_info.system = system_i386_darwin) then
=======
       if (target_info.system=system_i386_darwin) then
>>>>>>> graemeg/fixes_2_2
=======
       if (target_info.system=system_i386_darwin) then
>>>>>>> origin/fixes_2_2
         begin
           { a_jmp_name jumps to a stub which is always pic-safe on darwin }
           inherited g_external_wrapper(list,procdef,externalname);
           exit;
         end;

        sym:=current_asmdata.RefAsmSymbol(externalname);
<<<<<<< HEAD
<<<<<<< HEAD
        reference_reset_symbol(ref,sym,0,sizeof(pint));

        { create pic'ed? }
        if (cs_create_pic in current_settings.moduleswitches) and
           { darwin/x86_64's assembler doesn't want @PLT after call symbols }
           not(target_info.system in [system_x86_64_darwin,system_i386_iphonesim]) then
          ref.refaddr:=addr_pic
=======
=======
>>>>>>> origin/fixes_2_2
        reference_reset_symbol(ref,sym,0);

        { create pic'ed? }
        if cs_create_pic in current_settings.moduleswitches then
          begin
            { it could be that we're called from a procedure not having the
              got loaded
            }
            g_maybe_got_init(list);
            ref.refaddr:=addr_pic
          end
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        else
          ref.refaddr:=addr_full;
        list.concat(taicpu.op_ref(A_JMP,S_NO,ref));
      end;

>>>>>>> graemeg/cpstrnew
end.
