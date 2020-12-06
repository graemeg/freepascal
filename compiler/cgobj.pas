{
    Copyright (c) 1998-2005 by Florian Klaempfl
    Member of the Free Pascal development team

    This unit implements the basic code generator object

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
{# @abstract(Abstract code generator unit)
   Abstreact code generator unit. This contains the base class
   to implement for all new supported processors.

   WARNING: None of the routines implemented in these modules,
   or their descendants, should use the temp. allocator, as
   these routines may be called inside genentrycode, and the
   stack frame is already setup!
}
unit cgobj;

{$i fpcdefs.inc}

  interface

    uses
       globtype,constexp,
       cpubase,cgbase,cgutils,parabase,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       symconst,symtype,symdef,rgobj
       ;

    type
       talignment = (AM_NATURAL,AM_NONE,AM_2BYTE,AM_4BYTE,AM_8BYTE);

       {# @abstract(Abstract code generator)
          This class implements an abstract instruction generator. Some of
          the methods of this class are generic, while others must
          be overridden for all new processors which will be supported
          by Free Pascal. For 32-bit processors, the base class
          should be @link(tcg64f32) and not @var(tcg).
       }

       { tcg }

       tcg = class
          { how many times is this current code executed }
          executionweight : longint;
          alignment : talignment;
          rg        : array[tregistertype] of trgobj;
{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
          has_next_reg: bitpacked array[TSuperRegister] of boolean;
{$endif cpu8bitalu or cpu16bitalu}
       {$ifdef flowgraph}
          aktflownode:word;
       {$endif}
          {************************************************}
          {                 basic routines                 }
          constructor create;

          {# Initialize the register allocators needed for the codegenerator.}
          procedure init_register_allocators;virtual;
          {# Clean up the register allocators needed for the codegenerator.}
          procedure done_register_allocators;virtual;
          {# Set whether live_start or live_end should be updated when allocating registers, needed when e.g. generating initcode after the rest of the code. }
          procedure set_regalloc_live_range_direction(dir: TRADirection);

       {$ifdef flowgraph}
          procedure init_flowgraph;
          procedure done_flowgraph;
       {$endif}
          {# Gets a register suitable to do integer operations on.}
          function getintregister(list:TAsmList;size:Tcgsize):Tregister;virtual;
          {# Gets a register suitable to do integer operations on.}
          function getaddressregister(list:TAsmList):Tregister;virtual;
          function getfpuregister(list:TAsmList;size:Tcgsize):Tregister;virtual;
          function getmmregister(list:TAsmList;size:Tcgsize):Tregister;virtual;
          function getflagregister(list:TAsmList;size:Tcgsize):Tregister;virtual;
          function gettempregister(list:TAsmList):Tregister;virtual;
          {Does the generic cg need SIMD registers, like getmmxregister? Or should
           the cpu specific child cg object have such a method?}

{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
          {# returns the next virtual register }
          function GetNextReg(const r: TRegister): TRegister;virtual;
{$endif cpu8bitalu or cpu16bitalu}
{$ifdef cpu8bitalu}
          {# returns the register with the offset of ofs of a continuous set of register starting with r }
          function GetOffsetReg(const r : TRegister;ofs : shortint) : TRegister;virtual;abstract;
          {# returns the register with the offset of ofs of a continuous set of register starting with r and being continued with rhi }
          function GetOffsetReg64(const r,rhi: TRegister;ofs : shortint): TRegister;virtual;abstract;
{$endif cpu8bitalu}

          procedure add_reg_instruction(instr:Tai;r:tregister);virtual;
          procedure add_move_instruction(instr:Taicpu);virtual;

          function  uses_registers(rt:Tregistertype):boolean;virtual;
          {# Get a specific register.}
          procedure getcpuregister(list:TAsmList;r:Tregister);virtual;
          procedure ungetcpuregister(list:TAsmList;r:Tregister);virtual;
          {# Get multiple registers specified.}
          procedure alloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);virtual;
          {# Free multiple registers specified.}
          procedure dealloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);virtual;

          procedure allocallcpuregisters(list:TAsmList);virtual;
          procedure deallocallcpuregisters(list:TAsmList);virtual;
          procedure do_register_allocation(list:TAsmList;headertai:tai);virtual;
          procedure translate_register(var reg : tregister);

          function makeregsize(list:TAsmList;reg:Tregister;size:Tcgsize):Tregister; virtual;

          {# Emit a label to the instruction stream. }
          procedure a_label(list : TAsmList;l : tasmlabel);virtual;

          {# Allocates register r by inserting a pai_realloc record }
          procedure a_reg_alloc(list : TAsmList;r : tregister);
          {# Deallocates register r by inserting a pa_regdealloc record}
          procedure a_reg_dealloc(list : TAsmList;r : tregister);
          { Synchronize register, make sure it is still valid }
          procedure a_reg_sync(list : TAsmList;r : tregister);

          {# Pass a parameter, which is located in a register, to a routine.

             This routine should push/send the parameter to the routine, as
             required by the specific processor ABI and routine modifiers.
             It must generate register allocation information for the cgpara in
             case it consists of cpuregisters.

             @param(size size of the operand in the register)
             @param(r register source of the operand)
             @param(cgpara where the parameter will be stored)
          }
          procedure a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : TCGPara);virtual;
          {# Pass a parameter, which is a constant, to a routine.

             A generic version is provided. This routine should
             be overridden for optimization purposes if the cpu
             permits directly sending this type of parameter.
             It must generate register allocation information for the cgpara in
             case it consists of cpuregisters.

             @param(size size of the operand in constant)
             @param(a value of constant to send)
             @param(cgpara where the parameter will be stored)
          }
          procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : TCGPara);virtual;
          {# Pass the value of a parameter, which is located in memory, to a routine.

             A generic version is provided. This routine should
             be overridden for optimization purposes if the cpu
             permits directly sending this type of parameter.
             It must generate register allocation information for the cgpara in
             case it consists of cpuregisters.

             @param(size size of the operand in constant)
             @param(r Memory reference of value to send)
             @param(cgpara where the parameter will be stored)
          }
          procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : TCGPara);virtual;
         protected
          procedure a_load_ref_cgparalocref(list: TAsmList; sourcesize: tcgsize; sizeleft: tcgint; const ref, paralocref: treference; const cgpara: tcgpara; const location: PCGParaLocation); virtual;
         public
          {# Pass the value of a parameter, which can be located either in a register or memory location,
             to a routine.

             A generic version is provided.

             @param(l location of the operand to send)
             @param(nr parameter number (starting from one) of routine (from left to right))
             @param(cgpara where the parameter will be stored)
          }
          procedure a_load_loc_cgpara(list : TAsmList;const l : tlocation;const cgpara : TCGPara);
          {# Pass the address of a reference to a routine. This routine
             will calculate the address of the reference, and pass this
             calculated address as a parameter.
             It must generate register allocation information for the cgpara in
             case it consists of cpuregisters.

             A generic version is provided. This routine should
             be overridden for optimization purposes if the cpu
             permits directly sending this type of parameter.

             @param(r reference to get address from)
             @param(nr parameter number (starting from one) of routine (from left to right))
          }
          procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : TCGPara);virtual;

          {# Load a cgparaloc into a memory reference.
             It must generate register allocation information for the cgpara in
             case it consists of cpuregisters.

           @param(paraloc the source parameter sublocation)
           @param(ref the destination reference)
           @param(sizeleft indicates the total number of bytes left in all of
                  the remaining sublocations of this parameter (the current
                  sublocation and all of the sublocations coming after it).
                  In case this location is also a reference, it is assumed
                  to be the final part sublocation of the parameter and that it
                  contains all of the "sizeleft" bytes).)
           @param(align the alignment of the paraloc in case it's a reference)
          }
          procedure a_load_cgparaloc_ref(list : TAsmList;const paraloc : TCGParaLocation;const ref : treference;sizeleft : tcgint;align : longint);

          {# Load a cgparaloc into any kind of register (int, fp, mm).

           @param(regsize the size of the destination register)
           @param(paraloc the source parameter sublocation)
           @param(reg the destination register)
           @param(align the alignment of the paraloc in case it's a reference)
          }
          procedure a_load_cgparaloc_anyreg(list : TAsmList;regsize : tcgsize;const paraloc : TCGParaLocation;reg : tregister;align : longint);

          { Remarks:
            * If a method specifies a size you have only to take care
              of that number of bits, i.e. load_const_reg with OP_8 must
              only load the lower 8 bit of the specified register
              the rest of the register can be undefined
              if  necessary the compiler will call a method
              to zero or sign extend the register
            * The a_load_XX_XX with OP_64 needn't to be
              implemented for 32 bit
              processors, the code generator takes care of that
            * the addr size is for work with the natural pointer
              size
            * the procedures without fpu/mm are only for integer usage
            * normally the first location is the source and the
              second the destination
          }

          {# Emits instruction to call the method specified by symbol name.
             This routine must be overridden for each new target cpu.
          }
          procedure a_call_name(list : TAsmList;const s : string; weak: boolean);virtual; abstract;
          procedure a_call_reg(list : TAsmList;reg : tregister);virtual; abstract;
          { same as a_call_name, might be overridden on certain architectures to emit
            static calls without usage of a got trampoline }
          procedure a_call_name_static(list : TAsmList;const s : string);virtual;

          { move instructions }
          procedure a_load_const_reg(list : TAsmList;size : tcgsize;a : tcgint;register : tregister);virtual; abstract;
          procedure a_load_const_ref(list : TAsmList;size : tcgsize;a : tcgint;const ref : treference);virtual;
          procedure a_load_const_loc(list : TAsmList;a : tcgint;const loc : tlocation);
          procedure a_load_reg_ref(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);virtual; abstract;
          procedure a_load_reg_ref_unaligned(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);virtual;
          procedure a_load_reg_reg(list : TAsmList;fromsize,tosize : tcgsize;reg1,reg2 : tregister);virtual; abstract;
          procedure a_load_reg_loc(list : TAsmList;fromsize : tcgsize;reg : tregister;const loc: tlocation);
          procedure a_load_ref_reg(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);virtual; abstract;
          procedure a_load_ref_reg_unaligned(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);virtual;
          procedure a_load_ref_ref(list : TAsmList;fromsize,tosize : tcgsize;const sref : treference;const dref : treference);virtual;
          procedure a_load_loc_reg(list : TAsmList;tosize: tcgsize; const loc: tlocation; reg : tregister);
          procedure a_load_loc_ref(list : TAsmList;tosize: tcgsize; const loc: tlocation; const ref : treference);
          procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);virtual; abstract;

          { bit scan instructions }
          procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tcgsize; src, dst: TRegister); virtual;

          { Multiplication with doubling result size.
            dstlo or dsthi may be NR_NO, in which case corresponding half of result is discarded. }
          procedure a_mul_reg_reg_pair(list: TAsmList; size: tcgsize; src1,src2,dstlo,dsthi: TRegister);virtual;

          { fpu move instructions }
          procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize:tcgsize; reg1, reg2: tregister); virtual; abstract;
          procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); virtual; abstract;
          procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); virtual; abstract;
          procedure a_loadfpu_ref_ref(list: TAsmList; fromsize, tosize: tcgsize; const ref1,ref2: treference);
          procedure a_loadfpu_loc_reg(list: TAsmList; tosize: tcgsize; const loc: tlocation; const reg: tregister);
          procedure a_loadfpu_reg_loc(list: TAsmList; fromsize: tcgsize; const reg: tregister; const loc: tlocation);
          procedure a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const r : tregister;const cgpara : TCGPara);virtual;
          procedure a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const cgpara : TCGPara);virtual;

          procedure a_loadfpu_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize; intreg, fpureg: tregister); virtual;
          procedure a_loadfpu_reg_intreg(list: TAsmList; fromsize, tosize: tcgsize; fpureg, intreg: tregister); virtual;

          { vector register move instructions }
          procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle); virtual;
          procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); virtual;
          procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle); virtual;
          procedure a_loadmm_loc_reg(list: TAsmList; size: tcgsize; const loc: tlocation; const reg: tregister;shuffle : pmmshuffle);
          procedure a_loadmm_reg_loc(list: TAsmList; size: tcgsize; const reg: tregister; const loc: tlocation;shuffle : pmmshuffle);
          procedure a_loadmm_reg_cgpara(list: TAsmList; size: tcgsize; reg: tregister;const cgpara : TCGPara;shuffle : pmmshuffle); virtual;
          procedure a_loadmm_ref_cgpara(list: TAsmList; size: tcgsize; const ref: treference;const cgpara : TCGPara;shuffle : pmmshuffle); virtual;
          procedure a_loadmm_loc_cgpara(list: TAsmList; const loc: tlocation; const cgpara : TCGPara;shuffle : pmmshuffle); virtual;
          procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src,dst: tregister;shuffle : pmmshuffle); virtual;
          procedure a_opmm_ref_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); virtual;
          procedure a_opmm_loc_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const loc: tlocation; reg: tregister;shuffle : pmmshuffle); virtual;
          procedure a_opmm_reg_ref(list: TAsmList; Op: TOpCG; size : tcgsize;reg: tregister;const ref: treference; shuffle : pmmshuffle); virtual;
          procedure a_opmm_loc_reg_reg(list: TAsmList;Op : TOpCG;size : tcgsize;const loc : tlocation;src,dst : tregister;shuffle : pmmshuffle); virtual;
          procedure a_opmm_reg_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src1,src2,dst: tregister;shuffle : pmmshuffle); virtual;
          procedure a_opmm_ref_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const ref: treference; src,dst: tregister;shuffle : pmmshuffle); virtual;

          procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize; intreg, mmreg: tregister; shuffle: pmmshuffle); virtual;
          procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize; mmreg, intreg: tregister; shuffle : pmmshuffle); virtual;

          { basic arithmetic operations }
          procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister); virtual; abstract;
          procedure a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference); virtual;
          procedure a_op_const_loc(list : TAsmList; Op: TOpCG; a: tcgint; const loc: tlocation);
          procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; reg1, reg2: TRegister); virtual; abstract;
          procedure a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize; reg: TRegister; const ref: TReference); virtual;
          procedure a_op_ref_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference; reg: TRegister); virtual;
          procedure a_op_reg_loc(list : TAsmList; Op: TOpCG; reg: tregister; const loc: tlocation);
          procedure a_op_loc_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const loc: tlocation; reg: tregister);
          procedure a_op_ref_loc(list : TAsmList; Op: TOpCG; const ref: TReference; const loc: tlocation);

          { trinary operations for processors that support them, 'emulated' }
          { on others. None with "ref" arguments since I don't think there  }
          { are any processors that support it (JM)                         }
          procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister); virtual;
          procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister); virtual;
          procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation); virtual;
          procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation); virtual;

          { unary operations (not, neg) }
          procedure a_op_reg(list : TAsmList; Op: TOpCG; size: TCGSize; reg: TRegister); virtual;
          procedure a_op_ref(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference); virtual;
          procedure a_op_loc(list : TAsmList; Op: TOpCG; const loc: tlocation);

          {  comparison operations }
          procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
            l : tasmlabel); virtual;
          procedure a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference;
            l : tasmlabel); virtual;
          procedure a_cmp_const_loc_label(list: TAsmList; size: tcgsize;cmp_op: topcmp; a: tcgint; const loc: tlocation;
            l : tasmlabel);
          procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel); virtual; abstract;
          procedure a_cmp_ref_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; const ref: treference; reg : tregister; l : tasmlabel); virtual;
          procedure a_cmp_reg_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg : tregister; const ref: treference; l : tasmlabel); virtual;

          procedure a_cmp_loc_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; const loc: tlocation; reg : tregister; l : tasmlabel);
          procedure a_cmp_reg_loc_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; reg: tregister; const loc: tlocation; l : tasmlabel);
          procedure a_cmp_ref_loc_label(list: TAsmList; size: tcgsize;cmp_op: topcmp; const ref: treference; const loc: tlocation;
            l : tasmlabel);

          procedure a_jmp_name(list : TAsmList;const s : string); virtual; abstract;
          procedure a_jmp_always(list : TAsmList;l: tasmlabel); virtual; abstract;
{$ifdef cpuflags}
          procedure a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel); virtual; abstract;

          {# Depending on the value to check in the flags, either sets the register reg to one (if the flag is set)
             or zero (if the flag is cleared). The size parameter indicates the destination size register.
          }
          procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: tresflags; reg: TRegister); virtual; abstract;
          procedure g_flags2ref(list: TAsmList; size: TCgSize; const f: tresflags; const ref:TReference); virtual;
{$endif cpuflags}

          {
             This routine tries to optimize the op_const_reg/ref opcode, and should be
             called at the start of a_op_const_reg/ref. It returns the actual opcode
             to emit, and the constant value to emit. This function can opcode OP_NONE to
             remove the opcode and OP_MOVE to replace it with a simple load

             @param(size Size of the operand in constant)
             @param(op The opcode to emit, returns the opcode which must be emitted)
             @param(a  The constant which should be emitted, returns the constant which must
                    be emitted)
          }
          procedure optimize_op_const(size: TCGSize; var op: topcg; var a : tcgint);virtual;


          {# This should emit the opcode to copy len bytes from the source
             to destination.

             It must be overridden for each new target processor.

             @param(source Source reference of copy)
             @param(dest Destination reference of copy)

          }
          procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);virtual; abstract;
          {# This should emit the opcode to copy len bytes from the an unaligned source
             to destination.

             It must be overridden for each new target processor.

             @param(source Source reference of copy)
             @param(dest Destination reference of copy)

          }
          procedure g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);virtual;

          {# Generates overflow checking code for a node }
          procedure g_overflowcheck(list: TAsmList; const Loc:tlocation; def:tdef); virtual;abstract;
          procedure g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);virtual;

          {# Emits instructions when compilation is done in profile
             mode (this is set as a command line option). The default
             behavior does nothing, should be overridden as required.
          }
          procedure g_profilecode(list : TAsmList);virtual;
          {# Emits instruction for allocating @var(size) bytes at the stackpointer

             @param(size Number of bytes to allocate)
          }
          procedure g_stackpointer_alloc(list : TAsmList;size : longint);virtual;
          {# Emits instruction for allocating the locals in entry
             code of a routine. This is one of the first
             routine called in @var(genentrycode).

             @param(localsize Number of bytes to allocate as locals)
          }
          procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);virtual; abstract;
          {# Emits instructions for returning from a subroutine.
             Should also restore the framepointer and stack.

             @param(parasize  Number of bytes of parameters to deallocate from stack)
          }
          procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean);virtual;abstract;
          {# This routine is called when generating the code for the entry point
             of a routine. It should save all registers which are not used in this
             routine, and which should be declared as saved in the std_saved_registers
             set.

             This routine is mainly used when linking to code which is generated
             by ABI-compliant compilers (like GCC), to make sure that the reserved
             registers of that ABI are not clobbered.

             @param(usedinproc Registers which are used in the code of this routine)
          }
          procedure g_save_registers(list:TAsmList);virtual;
          {# This routine is called when generating the code for the exit point
             of a routine. It should restore all registers which were previously
             saved in @var(g_save_standard_registers).

             @param(usedinproc Registers which are used in the code of this routine)
          }
          procedure g_restore_registers(list:TAsmList);virtual;

          procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);virtual;

          { initialize the pic/got register }
          procedure g_maybe_got_init(list: TAsmList); virtual;
          { initialize the tls register if needed }
          procedure g_maybe_tls_init(list : TAsmList); virtual;
          { allocallcpuregisters, a_call_name, deallocallcpuregisters sequence }
          procedure g_call(list: TAsmList; const s: string);
          { Generate code to exit an unwind-protected region. The default implementation
            produces a simple jump to destination label. }
          procedure g_local_unwind(list: TAsmList; l: TAsmLabel);virtual;
          { Generate code for integer division by constant,
            generic version is suitable for 3-address CPUs }
          procedure g_div_const_reg_reg(list:tasmlist; size: TCgSize; a: tcgint; src,dst: tregister); virtual;

          { some CPUs do not support hardware fpu exceptions, this procedure is called after instructions which
            might set FPU exception related flags, so it has to check these flags if needed and throw an exeception }
          procedure g_check_for_fpu_exception(list : TAsmList; force,clear : boolean); virtual;
          procedure maybe_check_for_fpu_exception(list: TAsmList);

         protected
          function g_indirect_sym_load(list:TAsmList;const symname: string; const flags: tindsymflags): tregister;virtual;
       end;

{$ifdef cpu64bitalu}
    {  This class implements an abstract code generator class
       for 128 Bit operations, it applies currently only to 64 Bit CPUs and supports only simple operations
    }
    tcg128 = class
        procedure a_load128_reg_reg(list : TAsmList;regsrc,regdst : tregister128);virtual;
        procedure a_load128_reg_ref(list : TAsmList;reg : tregister128;const ref : treference);virtual;
        procedure a_load128_ref_reg(list : TAsmList;const ref : treference;reg : tregister128);virtual;
        procedure a_load128_loc_ref(list : TAsmList;const l : tlocation;const ref : treference);virtual;
        procedure a_load128_reg_loc(list : TAsmList;reg : tregister128;const l : tlocation);virtual;
        procedure a_load128_const_reg(list : TAsmList;valuelo,valuehi : int64;reg : tregister128);virtual;

        procedure a_load128_loc_cgpara(list : TAsmList;const l : tlocation;const paraloc : TCGPara);virtual;
        procedure a_load128_ref_cgpara(list: TAsmList; const r: treference;const paraloc: tcgpara);
        procedure a_load128_reg_cgpara(list: TAsmList; reg: tregister128;const paraloc: tcgpara);
    end;

    { Creates a tregister128 record from 2 64 Bit registers. }
    function joinreg128(reglo,reghi : tregister) : tregister128;

{$else cpu64bitalu}
    {# @abstract(Abstract code generator for 64 Bit operations)
       This class implements an abstract code generator class
       for 64 Bit operations.
    }
    tcg64 = class
        procedure a_load64_const_ref(list : TAsmList;value : int64;const ref : treference);virtual;abstract;
        procedure a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference);virtual;abstract;
        procedure a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64);virtual;abstract;
        procedure a_load64_reg_reg(list : TAsmList;regsrc,regdst : tregister64);virtual;abstract;
        procedure a_load64_const_reg(list : TAsmList;value : int64;reg : tregister64);virtual;abstract;
        procedure a_load64_loc_reg(list : TAsmList;const l : tlocation;reg : tregister64);virtual;abstract;
        procedure a_load64_loc_ref(list : TAsmList;const l : tlocation;const ref : treference);virtual;abstract;
        procedure a_load64_const_loc(list : TAsmList;value : int64;const l : tlocation);virtual;abstract;
        procedure a_load64_reg_loc(list : TAsmList;reg : tregister64;const l : tlocation);virtual;abstract;

        procedure a_load64_subsetref_reg(list : TAsmList; const sref: tsubsetreference; destreg: tregister64);virtual;abstract;
        procedure a_load64_reg_subsetref(list : TAsmList; fromreg: tregister64; const sref: tsubsetreference);virtual;abstract;
        procedure a_load64_const_subsetref(list: TAsmlist; a: int64; const sref: tsubsetreference);virtual;abstract;
        procedure a_load64_ref_subsetref(list : TAsmList; const fromref: treference; const sref: tsubsetreference);virtual;abstract;
        procedure a_load64_subsetref_subsetref(list: TAsmlist; const fromsref, tosref: tsubsetreference); virtual;abstract;
        procedure a_load64_subsetref_ref(list : TAsmList; const sref: tsubsetreference; const destref: treference); virtual;abstract;
        procedure a_load64_loc_subsetref(list : TAsmList; const l: tlocation; const sref : tsubsetreference);
        procedure a_load64_subsetref_loc(list: TAsmlist; const sref: tsubsetreference; const l: tlocation);

        procedure a_load64high_reg_ref(list : TAsmList;reg : tregister;const ref : treference);virtual;abstract;
        procedure a_load64low_reg_ref(list : TAsmList;reg : tregister;const ref : treference);virtual;abstract;
        procedure a_load64high_ref_reg(list : TAsmList;const ref : treference;reg : tregister);virtual;abstract;
        procedure a_load64low_ref_reg(list : TAsmList;const ref : treference;reg : tregister);virtual;abstract;
        procedure a_load64high_loc_reg(list : TAsmList;const l : tlocation;reg : tregister);virtual;abstract;
        procedure a_load64low_loc_reg(list : TAsmList;const l : tlocation;reg : tregister);virtual;abstract;

        procedure a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);virtual;abstract;
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);virtual;abstract;
        procedure a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;regsrc : tregister64;const ref : treference);virtual;abstract;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;regdst : tregister64);virtual;abstract;
        procedure a_op64_const_ref(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const ref : treference);virtual;abstract;
        procedure a_op64_const_loc(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const l: tlocation);virtual;abstract;
        procedure a_op64_reg_loc(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64;const l : tlocation);virtual;abstract;
        procedure a_op64_ref_loc(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;const l : tlocation);virtual;abstract;
        procedure a_op64_loc_reg(list : TAsmList;op:TOpCG;size : tcgsize;const l : tlocation;reg64 : tregister64);virtual;abstract;
        procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);virtual;
        procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);virtual;
        procedure a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);virtual;
        procedure a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);virtual;
        procedure a_op64_reg(list : TAsmList;op:TOpCG;size : tcgsize;regdst : tregister64);virtual;
        procedure a_op64_ref(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference);virtual;
        procedure a_op64_loc(list : TAsmList;op:TOpCG;size : tcgsize;const l : tlocation);virtual;

        procedure a_op64_const_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; a : int64; const sref: tsubsetreference);
        procedure a_op64_reg_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; reg: tregister64; const sref: tsubsetreference);
        procedure a_op64_ref_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; const ref: treference; const sref: tsubsetreference);
        procedure a_op64_subsetref_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; const ssref,dsref: tsubsetreference);

        procedure a_load64_reg_cgpara(list : TAsmList;reg64 : tregister64;const loc : TCGPara);virtual;abstract;
        procedure a_load64_const_cgpara(list : TAsmList;value : int64;const loc : TCGPara);virtual;abstract;
        procedure a_load64_ref_cgpara(list : TAsmList;const r : treference;const loc : TCGPara);virtual;abstract;
        procedure a_load64_loc_cgpara(list : TAsmList;const l : tlocation;const loc : TCGPara);virtual;abstract;

        procedure a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister); virtual;abstract;
        procedure a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64); virtual;abstract;
        {
             This routine tries to optimize the const_reg opcode, and should be
             called at the start of a_op64_const_reg. It returns the actual opcode
             to emit, and the constant value to emit. If this routine returns
             TRUE, @var(no) instruction should be emitted (.eg : imul reg by 1 )

             @param(op The opcode to emit, returns the opcode which must be emitted)
             @param(a  The constant which should be emitted, returns the constant which must
                    be emitted)
             @param(reg The register to emit the opcode with, returns the register with
                   which the opcode will be emitted)
        }
        function optimize64_op_const_reg(list: TAsmList; var op: topcg; var a : int64; var reg: tregister64): boolean;virtual;abstract;


        { override to catch 64bit rangechecks }
        procedure g_rangecheck64(list: TAsmList; const l:tlocation; fromdef,todef: tdef);virtual;abstract;
    end;

    { Creates a tregister64 record from 2 32 Bit registers. }
    function joinreg64(reglo,reghi : tregister) : tregister64;
{$endif cpu64bitalu}

    var
       { Main code generator class }
       cg : tcg;
{$ifdef cpu64bitalu}
       { Code generator class for all operations working with 128-Bit operands }
       cg128 : tcg128;
{$else cpu64bitalu}
       { Code generator class for all operations working with 64-Bit operands }
       cg64 : tcg64;
{$endif cpu64bitalu}

    function asmsym2indsymflags(sym: TAsmSymbol): tindsymflags;

    procedure destroy_codegen;

implementation

    uses
       globals,systems,fmodule,
       verbose,paramgr,symsym,symtable,
       tgobj,cutils,procinfo,
       cpuinfo;

{*****************************************************************************
                            basic functionallity
******************************************************************************}

    constructor tcg.create;
      begin
      end;


{*****************************************************************************
                                register allocation
******************************************************************************}


    procedure tcg.init_register_allocators;
      begin
{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
        fillchar(has_next_reg,sizeof(has_next_reg),0);
{$endif cpu8bitalu or cpu16bitalu}
        fillchar(rg,sizeof(rg),0);
        add_reg_instruction_hook:=@add_reg_instruction;
        executionweight:=100;
      end;


    procedure tcg.done_register_allocators;
      begin
        { Safety }
        fillchar(rg,sizeof(rg),0);
        add_reg_instruction_hook:=nil;
{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
        fillchar(has_next_reg,sizeof(has_next_reg),0);
{$endif cpu8bitalu or cpu16bitalu}
      end;

    {$ifdef flowgraph}
    procedure Tcg.init_flowgraph;

    begin
      aktflownode:=0;
    end;

    procedure Tcg.done_flowgraph;

    begin
    end;
    {$endif}

    function tcg.getintregister(list:TAsmList;size:Tcgsize):Tregister;
{$ifdef cpu8bitalu}
      var
        tmp1,tmp2,tmp3 : TRegister;
{$endif cpu8bitalu}
      begin
        if not assigned(rg[R_INTREGISTER]) then
          internalerror(200312122);
{$if defined(cpu8bitalu)}
        case size of
          OS_8,OS_S8:
            Result:=rg[R_INTREGISTER].getregister(list,cgsize2subreg(R_INTREGISTER,size));
          OS_16,OS_S16:
            begin
              Result:=getintregister(list, OS_8);
              has_next_reg[getsupreg(Result)]:=true;
              { ensure that the high register can be retrieved by
                GetNextReg
              }
              if getintregister(list, OS_8)<>GetNextReg(Result) then
                internalerror(2011021331);
            end;
          OS_32,OS_S32:
            begin
              Result:=getintregister(list, OS_8);
              has_next_reg[getsupreg(Result)]:=true;
              tmp1:=getintregister(list, OS_8);
              has_next_reg[getsupreg(tmp1)]:=true;
              { ensure that the high register can be retrieved by
                GetNextReg
              }
              if tmp1<>GetNextReg(Result) then
                internalerror(2011021332);
              tmp2:=getintregister(list, OS_8);
              has_next_reg[getsupreg(tmp2)]:=true;
              { ensure that the upper register can be retrieved by
                GetNextReg
              }
              if tmp2<>GetNextReg(tmp1) then
                internalerror(2011021333);
              tmp3:=getintregister(list, OS_8);
              { ensure that the upper register can be retrieved by
                GetNextReg
              }
              if tmp3<>GetNextReg(tmp2) then
                internalerror(2011021334);
            end;
          else
            internalerror(2011021330);
        end;
{$elseif defined(cpu16bitalu)}
        case size of
          OS_8, OS_S8,
          OS_16, OS_S16:
            Result:=rg[R_INTREGISTER].getregister(list,cgsize2subreg(R_INTREGISTER,size));
          OS_32, OS_S32:
            begin
              Result:=getintregister(list, OS_16);
              has_next_reg[getsupreg(Result)]:=true;
              { ensure that the high register can be retrieved by
                GetNextReg
              }
              if getintregister(list, OS_16)<>GetNextReg(Result) then
                internalerror(2013030202);
            end;
          else
            internalerror(2013030201);
        end;
{$elseif defined(cpu32bitalu) or defined(cpu64bitalu)}
        result:=rg[R_INTREGISTER].getregister(list,cgsize2subreg(R_INTREGISTER,size));
{$endif}
      end;


    function tcg.getfpuregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if not assigned(rg[R_FPUREGISTER]) then
          internalerror(200312123);
        result:=rg[R_FPUREGISTER].getregister(list,cgsize2subreg(R_FPUREGISTER,size));
      end;


    function tcg.getmmregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if not assigned(rg[R_MMREGISTER]) then
          internalerror(2003121214);
        result:=rg[R_MMREGISTER].getregister(list,cgsize2subreg(R_MMREGISTER,size));
      end;


    function tcg.getaddressregister(list:TAsmList):Tregister;
      begin
        if assigned(rg[R_ADDRESSREGISTER]) then
          result:=rg[R_ADDRESSREGISTER].getregister(list,R_SUBWHOLE)
        else
          begin
            if not assigned(rg[R_INTREGISTER]) then
              internalerror(200312121);
            result:=rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
          end;
      end;


    function tcg.gettempregister(list: TAsmList): Tregister;
      begin
        result:=rg[R_TEMPREGISTER].getregister(list,R_SUBWHOLE);
      end;


{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
    function tcg.GetNextReg(const r: TRegister): TRegister;
      begin
{$ifdef AVR}
        { the AVR code generator depends on the fact that it can do GetNextReg also on physical registers }
        if (getsupreg(r)>=first_int_imreg) and not(has_next_reg[getsupreg(r)]) then
          internalerror(2017091103);
{$else AVR}
        if getsupreg(r)<first_int_imreg then
          internalerror(2013051401);
        if not has_next_reg[getsupreg(r)] then
          internalerror(2017091103);
{$endif AVR}
        if getregtype(r)<>R_INTREGISTER then
          internalerror(2017091101);
        if getsubreg(r)<>R_SUBWHOLE then
          internalerror(2017091102);
        result:=TRegister(longint(r)+1);
      end;
{$endif cpu8bitalu or cpu16bitalu}


    function Tcg.makeregsize(list:TAsmList;reg:Tregister;size:Tcgsize):Tregister;
      var
        subreg:Tsubregister;
      begin
        subreg:=cgsize2subreg(getregtype(reg),size);
        result:=reg;
        setsubreg(result,subreg);
        { notify RA }
        if result<>reg then
          list.concat(tai_regalloc.resize(result));
      end;


    procedure tcg.getcpuregister(list:TAsmList;r:Tregister);
      begin
        if not assigned(rg[getregtype(r)]) then
          internalerror(200312125);
        rg[getregtype(r)].getcpuregister(list,r);
      end;


    procedure tcg.ungetcpuregister(list:TAsmList;r:Tregister);
      begin
        if not assigned(rg[getregtype(r)]) then
          internalerror(200312126);
        rg[getregtype(r)].ungetcpuregister(list,r);
      end;


    procedure tcg.alloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);
      begin
        if assigned(rg[rt]) then
          rg[rt].alloccpuregisters(list,r)
        else
          internalerror(200310092);
      end;


    procedure tcg.allocallcpuregisters(list:TAsmList);
      begin
        alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        if uses_registers(R_ADDRESSREGISTER) then
          alloccpuregisters(list,R_ADDRESSREGISTER,paramanager.get_volatile_registers_address(pocall_default));
{$if not(defined(i386)) and not(defined(i8086)) and not(defined(avr))}
        if uses_registers(R_FPUREGISTER) then
          alloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
{$ifdef cpumm}
        if uses_registers(R_MMREGISTER) then
          alloccpuregisters(list,R_MMREGISTER,paramanager.get_volatile_registers_mm(pocall_default));
{$endif cpumm}
{$endif not(defined(i386)) and not(defined(i8086)) and not(defined(avr))}
      end;


    procedure tcg.dealloccpuregisters(list:TAsmList;rt:Tregistertype;const r:Tcpuregisterset);
      begin
        if assigned(rg[rt]) then
          rg[rt].dealloccpuregisters(list,r)
        else
          internalerror(200310093);
      end;


    procedure tcg.deallocallcpuregisters(list:TAsmList);
      begin
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        if uses_registers(R_ADDRESSREGISTER) then
          dealloccpuregisters(list,R_ADDRESSREGISTER,paramanager.get_volatile_registers_address(pocall_default));
{$if not(defined(i386)) and not(defined(i8086)) and not(defined(avr))}
        if uses_registers(R_FPUREGISTER) then
          dealloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
{$ifdef cpumm}
        if uses_registers(R_MMREGISTER) then
          dealloccpuregisters(list,R_MMREGISTER,paramanager.get_volatile_registers_mm(pocall_default));
{$endif cpumm}
{$endif not(defined(i386)) and not(defined(i8086)) and not(defined(avr))}
      end;


    function tcg.uses_registers(rt:Tregistertype):boolean;
      begin
        if assigned(rg[rt]) then
          result:=rg[rt].uses_registers
        else
          result:=false;
      end;


    procedure tcg.add_reg_instruction(instr:Tai;r:tregister);
      var
        rt : tregistertype;
      begin
        rt:=getregtype(r);
        { Only add it when a register allocator is configured.
          No IE can be generated, because the VMT is written
          without a valid rg[] }
        if assigned(rg[rt]) then
          rg[rt].add_reg_instruction(instr,r,executionweight);
      end;


    procedure tcg.add_move_instruction(instr:Taicpu);
      var
        rt : tregistertype;
      begin
        rt:=getregtype(instr.oper[O_MOV_SOURCE]^.reg);
        if assigned(rg[rt]) then
          rg[rt].add_move_instruction(instr)
        else
          internalerror(200310095);
      end;


    procedure tcg.set_regalloc_live_range_direction(dir: TRADirection);
      var
        rt : tregistertype;
      begin
        for rt:=low(rg) to high(rg) do
          begin
            if assigned(rg[rt]) then
              rg[rt].live_range_direction:=dir;
          end;
      end;


    procedure tcg.do_register_allocation(list:TAsmList;headertai:tai);
      var
        rt : tregistertype;
      begin
        for rt:=R_FPUREGISTER to R_SPECIALREGISTER do
          begin
            if assigned(rg[rt]) then
              rg[rt].do_register_allocation(list,headertai);
          end;
         { running the other register allocator passes could require addition int/addr. registers
           when spilling so run int/addr register allocation at the end }
         if assigned(rg[R_INTREGISTER]) then
           rg[R_INTREGISTER].do_register_allocation(list,headertai);
         if assigned(rg[R_ADDRESSREGISTER]) then
           rg[R_ADDRESSREGISTER].do_register_allocation(list,headertai);
      end;


    procedure tcg.translate_register(var reg : tregister);
      var
        rt: tregistertype;
      begin
        { Getting here without assigned rg is possible for an "assembler nostackframe"
          function returning x87 float, compiler tries to translate NR_ST which is used for
          result.  }
        rt:=getregtype(reg);
        if assigned(rg[rt]) then
          rg[rt].translate_register(reg);
      end;


    procedure tcg.a_reg_alloc(list : TAsmList;r : tregister);
      begin
         list.concat(tai_regalloc.alloc(r,nil));
      end;


    procedure tcg.a_reg_dealloc(list : TAsmList;r : tregister);
      begin
        if (r<>NR_NO) then
          list.concat(tai_regalloc.dealloc(r,nil));
      end;


    procedure tcg.a_reg_sync(list : TAsmList;r : tregister);
      var
        instr : tai;
      begin
        instr:=tai_regalloc.sync(r);
        list.concat(instr);
        add_reg_instruction(instr,r);
      end;


    procedure tcg.a_label(list : TAsmList;l : tasmlabel);
      begin
         list.concat(tai_label.create(l));
      end;


{*****************************************************************************
          for better code generation these methods should be overridden
******************************************************************************}

    procedure tcg.a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : TCGPara);
      var
         ref : treference;
         tmpreg : tregister;
      begin
         if assigned(cgpara.location^.next) then
           begin
             tg.gethltemp(list,cgpara.def,cgpara.def.size,tt_persistent,ref);
             a_load_reg_ref(list,size,size,r,ref);
             a_load_ref_cgpara(list,size,ref,cgpara);
             tg.ungettemp(list,ref);
             exit;
           end;
         paramanager.alloccgpara(list,cgpara);
         if cgpara.location^.shiftval<0 then
           begin
             tmpreg:=getintregister(list,cgpara.location^.size);
             a_op_const_reg_reg(list,OP_SHL,cgpara.location^.size,-cgpara.location^.shiftval,r,tmpreg);
             r:=tmpreg;
           end;
         case cgpara.location^.loc of
            LOC_REGISTER,LOC_CREGISTER:
              a_load_reg_reg(list,size,cgpara.location^.size,r,cgpara.location^.register);
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                 reference_reset_base(ref,cgpara.location^.reference.index,cgpara.location^.reference.offset,ctempposinvalid,cgpara.alignment,[]);
                 a_load_reg_ref(list,size,cgpara.location^.size,r,ref);
              end;
            LOC_MMREGISTER,LOC_CMMREGISTER:
              a_loadmm_intreg_reg(list,size,cgpara.location^.size,r,cgpara.location^.register,mms_movescalar);
            LOC_FPUREGISTER,LOC_CFPUREGISTER:
              begin
                tg.GetTemp(list,TCGSize2Size[size],TCGSize2Size[size],tt_normal,ref);
                a_load_reg_ref(list,size,size,r,ref);
                a_loadfpu_ref_cgpara(list,cgpara.location^.size,ref,cgpara);
                tg.Ungettemp(list,ref);
              end
            else
              internalerror(2002071004);
         end;
      end;


    procedure tcg.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const cgpara : TCGPara);
      var
         ref : treference;
      begin
         cgpara.check_simple_location;
         paramanager.alloccgpara(list,cgpara);
         if cgpara.location^.shiftval<0 then
           a:=a shl -cgpara.location^.shiftval;
         case cgpara.location^.loc of
            LOC_REGISTER,LOC_CREGISTER:
              a_load_const_reg(list,cgpara.location^.size,a,cgpara.location^.register);
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                 reference_reset_base(ref,cgpara.location^.reference.index,cgpara.location^.reference.offset,ctempposinvalid,cgpara.alignment,[]);
                 a_load_const_ref(list,cgpara.location^.size,a,ref);
              end
            else
              internalerror(2010053109);
         end;
      end;


    procedure tcg.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const cgpara : TCGPara);
      var
        tmpref, ref: treference;
        tmpreg: tregister;
        location: pcgparalocation;
        orgsizeleft,
        sizeleft: tcgint;
        usesize: tcgsize;
        reghasvalue: boolean;
      begin
        location:=cgpara.location;
        tmpref:=r;
        sizeleft:=cgpara.intsize;
        repeat
          paramanager.allocparaloc(list,location);
          case location^.loc of
            LOC_REGISTER,LOC_CREGISTER:
              begin
                 { Parameter locations are often allocated in multiples of
                   entire registers. If a parameter only occupies a part of
                   such a register (e.g. a 16 bit int on a 32 bit
                   architecture), the size of this parameter can only be
                   determined by looking at the "size" parameter of this
                   method -> if the size parameter is <= sizeof(aint), then
                   we check that there is only one parameter location and
                   then use this "size" to load the value into the parameter
                   location }
                 if (size<>OS_NO) and
                    (tcgsize2size[size]<=sizeof(aint)) then
                   begin
                     cgpara.check_simple_location;
                     a_load_ref_reg(list,size,location^.size,tmpref,location^.register);
                     if location^.shiftval<0 then
                       a_op_const_reg(list,OP_SHL,location^.size,-location^.shiftval,location^.register);
                   end
                 { there's a lot more data left, and the current paraloc's
                   register is entirely filled with part of that data }
                 else if (sizeleft>sizeof(aint)) then
                   begin
                     a_load_ref_reg(list,location^.size,location^.size,tmpref,location^.register);
                   end
                 { we're at the end of the data, and it can be loaded into
                   the current location's register with a single regular
                   load }
                 else if sizeleft in [1,2,4,8] then
                   begin
                     a_load_ref_reg(list,int_cgsize(sizeleft),location^.size,tmpref,location^.register);
                     if location^.shiftval<0 then
                       a_op_const_reg(list,OP_SHL,location^.size,-location^.shiftval,location^.register);
                   end
                 { we're at the end of the data, and we need multiple loads
                   to get it in the register because it's an irregular size }
                 else
                   begin
                     { should be the last part }
                     if assigned(location^.next) then
                       internalerror(2010052907);
                     { load the value piecewise to get it into the register }
                     orgsizeleft:=sizeleft;
                     reghasvalue:=false;
{$ifdef cpu64bitalu}
                     if sizeleft>=4 then
                       begin
                         a_load_ref_reg(list,OS_32,location^.size,tmpref,location^.register);
                         dec(sizeleft,4);
                         if target_info.endian=endian_big then
                           a_op_const_reg(list,OP_SHL,location^.size,sizeleft*8,location^.register);
                         inc(tmpref.offset,4);
                         reghasvalue:=true;
                       end;
{$endif cpu64bitalu}
                     if sizeleft>=2 then
                       begin
                         tmpreg:=getintregister(list,location^.size);
                         a_load_ref_reg(list,OS_16,location^.size,tmpref,tmpreg);
                         dec(sizeleft,2);
                         if reghasvalue then
                           begin
                             if target_info.endian=endian_big then
                               a_op_const_reg(list,OP_SHL,location^.size,sizeleft*8,tmpreg)
                             else
                               a_op_const_reg(list,OP_SHL,location^.size,(orgsizeleft-(sizeleft+2))*8,tmpreg);
                             a_op_reg_reg(list,OP_OR,location^.size,tmpreg,location^.register);
                           end
                         else
                           begin
                             if target_info.endian=endian_big then
                               a_op_const_reg_reg(list,OP_SHL,location^.size,sizeleft*8,tmpreg,location^.register)
                             else
                               a_load_reg_reg(list,location^.size,location^.size,tmpreg,location^.register);
                           end;
                         inc(tmpref.offset,2);
                         reghasvalue:=true;
                       end;
                     if sizeleft=1 then
                       begin
                         tmpreg:=getintregister(list,location^.size);
                         a_load_ref_reg(list,OS_8,location^.size,tmpref,tmpreg);
                         dec(sizeleft,1);
                         if reghasvalue then
                           begin
                             if target_info.endian=endian_little then
                               a_op_const_reg(list,OP_SHL,location^.size,(orgsizeleft-(sizeleft+1))*8,tmpreg);
                             a_op_reg_reg(list,OP_OR,location^.size,tmpreg,location^.register)
                           end
                         else
                           a_load_reg_reg(list,location^.size,location^.size,tmpreg,location^.register);
                         inc(tmpref.offset);
                       end;
                     if location^.shiftval<0 then
                       a_op_const_reg(list,OP_SHL,location^.size,-location^.shiftval,location^.register);
                     { the loop will already adjust the offset and sizeleft }
                     dec(tmpref.offset,orgsizeleft);
                     sizeleft:=orgsizeleft;
                   end;
              end;
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                reference_reset_base(ref,location^.reference.index,location^.reference.offset,ctempposinvalid,newalignment(cgpara.alignment,cgpara.intsize-sizeleft),[]);
                a_load_ref_cgparalocref(list,size,sizeleft,tmpref,ref,cgpara,location);
              end;
            LOC_MMREGISTER,LOC_CMMREGISTER:
              begin
                 case location^.size of
                   OS_F32,
                   OS_F64,
                   OS_F128:
                     a_loadmm_ref_reg(list,location^.size,location^.size,tmpref,location^.register,mms_movescalar);
                   OS_M8..OS_M512:
                     a_loadmm_ref_reg(list,location^.size,location^.size,tmpref,location^.register,nil);
                   else
                     internalerror(2010053101);
                 end;
              end;
            LOC_FPUREGISTER,LOC_CFPUREGISTER:
              begin
                { can be not a float size in case of a record passed in fpu registers }
                { the size comparison is to catch F128 passed in two 64 bit floating point registers }
                if is_float_cgsize(size) and
                   (tcgsize2size[location^.size]>=tcgsize2size[size]) then
                  usesize:=size
                else
                  usesize:=location^.size;
                a_loadfpu_ref_reg(list,usesize,location^.size,tmpref,location^.register);
              end
            else
              internalerror(2010053111);
          end;
          inc(tmpref.offset,tcgsize2size[location^.size]);
          dec(sizeleft,tcgsize2size[location^.size]);
          location:=location^.next;
        until not assigned(location);
      end;

    procedure tcg.a_load_ref_cgparalocref(list: TAsmList; sourcesize: tcgsize; sizeleft: tcgint; const ref, paralocref: treference; const cgpara: tcgpara; const location: PCGParaLocation);
      begin
        if assigned(location^.next) then
          internalerror(2010052906);
        if (sourcesize<>OS_NO) and
           (tcgsize2size[sourcesize]<=sizeof(aint)) then
           a_load_ref_ref(list,sourcesize,location^.size,ref,paralocref)
        else
          { use concatcopy, because the parameter can be larger than }
          { what the OS_* constants can handle                       }
          g_concatcopy(list,ref,paralocref,sizeleft);
       end;


    procedure tcg.a_load_loc_cgpara(list : TAsmList;const l:tlocation;const cgpara : TCGPara);
      begin
        case l.loc of
          LOC_REGISTER,
          LOC_CREGISTER :
            a_load_reg_cgpara(list,l.size,l.register,cgpara);
          LOC_CONSTANT :
            a_load_const_cgpara(list,l.size,l.value,cgpara);
          LOC_CREFERENCE,
          LOC_REFERENCE :
            a_load_ref_cgpara(list,l.size,l.reference,cgpara);
          else
            internalerror(2002032211);
        end;
      end;


    procedure tcg.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const cgpara : TCGPara);
      var
         hr : tregister;
      begin
         cgpara.check_simple_location;
         if cgpara.location^.loc in [LOC_CREGISTER,LOC_REGISTER] then
           begin
             paramanager.allocparaloc(list,cgpara.location);
             a_loadaddr_ref_reg(list,r,cgpara.location^.register)
           end
         else
           begin
             hr:=getaddressregister(list);
             a_loadaddr_ref_reg(list,r,hr);
             a_load_reg_cgpara(list,OS_ADDR,hr,cgpara);
           end;
      end;


    procedure tcg.a_load_cgparaloc_ref(list : TAsmList;const paraloc : TCGParaLocation;const ref : treference;sizeleft : tcgint;align : longint);
      var
        href : treference;
        hreg : tregister;
        cgsize: tcgsize;
      begin
        case paraloc.loc of
          LOC_REGISTER :
            begin
              hreg:=paraloc.register;
              cgsize:=paraloc.size;
              if paraloc.shiftval>0 then
                a_op_const_reg_reg(list,OP_SHL,OS_INT,paraloc.shiftval,paraloc.register,paraloc.register)
              { in case the original size was 3 or 5/6/7 bytes, the value was
                shifted to the top of the to 4 resp. 8 byte register on the
                caller side and needs to be stored with those bytes at the
                start of the reference -> don't shift right }
              else if (paraloc.shiftval<0)
{$ifdef LIMIT_NEG_SHIFTVALUES}
{$ifdef CPU64BITALU}
                      and ((-paraloc.shiftval) in [56{for byte},48{for two bytes},32{for four bytes}])
{$else}
                      and ((-paraloc.shiftval) in [24{for byte},16{for two bytes}])
{$endif}
{$endif}
                  then
                begin
                  a_op_const_reg_reg(list,OP_SHR,OS_INT,-paraloc.shiftval,paraloc.register,paraloc.register);
                  { convert to a register of 1/2/4 bytes in size, since the
                    original register had to be made larger to be able to hold
                    the shifted value }
                  cgsize:=int_cgsize(tcgsize2size[OS_INT]-(-paraloc.shiftval div 8));
                  if cgsize=OS_NO then
                    cgsize:=OS_INT;
                  hreg:=getintregister(list,cgsize);
                  a_load_reg_reg(list,OS_INT,cgsize,paraloc.register,hreg);
                end;
              { use the exact size to avoid overwriting of adjacent data }
              if tcgsize2size[cgsize]<=sizeleft then
                a_load_reg_ref(list,paraloc.size,cgsize,hreg,ref)
              else
                case sizeleft of
                  1,2,4,8:
                    a_load_reg_ref(list,paraloc.size,int_cgsize(sizeleft),hreg,ref);
                  3:
                    begin
                      if target_info.endian=endian_big then
                        begin
                          href:=ref;
                          inc(href.offset,2);
                          a_load_reg_ref(list,paraloc.size,OS_8,hreg,href);
                          a_op_const_reg_reg(list,OP_SHR,OS_INT,8,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_16,hreg,ref);
                        end
                      else
                        begin
                          a_load_reg_ref(list,paraloc.size,OS_16,hreg,ref);
                          href:=ref;
                          inc(href.offset,2);
                          a_op_const_reg_reg(list,OP_SHR,cgsize,16,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_8,hreg,href);
                        end
                    end;
                  5:
                    begin
                      if target_info.endian=endian_big then
                        begin
                          href:=ref;
                          inc(href.offset,4);
                          a_load_reg_ref(list,paraloc.size,OS_8,hreg,href);
                          a_op_const_reg_reg(list,OP_SHR,OS_INT,8,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_32,hreg,ref);
                        end
                      else
                       begin
                          a_load_reg_ref(list,paraloc.size,OS_32,hreg,ref);
                          href:=ref;
                          inc(href.offset,4);
                          a_op_const_reg_reg(list,OP_SHR,cgsize,32,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_8,hreg,href);
                        end
                    end;
                  6:
                    begin
                      if target_info.endian=endian_big then
                        begin
                          href:=ref;
                          inc(href.offset,4);
                          a_load_reg_ref(list,paraloc.size,OS_16,hreg,href);
                          a_op_const_reg_reg(list,OP_SHR,OS_INT,16,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_32,hreg,ref);
                        end
                      else
                       begin
                          a_load_reg_ref(list,paraloc.size,OS_32,hreg,ref);
                          href:=ref;
                          inc(href.offset,4);
                          a_op_const_reg_reg(list,OP_SHR,cgsize,32,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_16,hreg,href);
                        end
                    end;
                  7:
                    begin
                      if target_info.endian=endian_big then
                        begin
                          href:=ref;
                          inc(href.offset,6);
                          a_load_reg_ref(list,paraloc.size,OS_8,hreg,href);

                          a_op_const_reg_reg(list,OP_SHR,OS_INT,8,hreg,hreg);
                          href:=ref;
                          inc(href.offset,4);
                          a_load_reg_ref(list,paraloc.size,OS_16,hreg,href);

                          a_op_const_reg_reg(list,OP_SHR,OS_INT,16,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_32,hreg,ref);
                        end
                      else
                       begin
                          a_load_reg_ref(list,paraloc.size,OS_32,hreg,ref);

                          href:=ref;
                          inc(href.offset,4);
                          a_op_const_reg_reg(list,OP_SHR,cgsize,32,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_16,hreg,href);

                          inc(href.offset,2);
                          a_op_const_reg_reg(list,OP_SHR,cgsize,16,hreg,hreg);
                          a_load_reg_ref(list,paraloc.size,OS_8,hreg,href);
                        end
                    end;
                  else
                    { other sizes not allowed }
                    Internalerror(2017080901);
                end;
            end;
          LOC_MMREGISTER :
            begin
              case paraloc.size of
                OS_F32,
                OS_F64,
                OS_F128:
                  a_loadmm_reg_ref(list,paraloc.size,paraloc.size,paraloc.register,ref,mms_movescalar);
                OS_M8..OS_M512:
                  a_loadmm_reg_ref(list,paraloc.size,paraloc.size,paraloc.register,ref,nil);
                else
                  internalerror(2010053102);
              end;
            end;
          LOC_FPUREGISTER :
            a_loadfpu_reg_ref(list,paraloc.size,paraloc.size,paraloc.register,ref);
          LOC_REFERENCE :
            begin
              reference_reset_base(href,paraloc.reference.index,paraloc.reference.offset,ctempposinvalid,align,[]);
              { use concatcopy, because it can also be a float which fails when
                load_ref_ref is used. Don't copy data when the references are equal }
              if not((href.base=ref.base) and (href.offset=ref.offset)) then
                g_concatcopy(list,href,ref,sizeleft);
            end;
          else
            internalerror(2002081302);
        end;
      end;


    procedure tcg.a_load_cgparaloc_anyreg(list: TAsmList;regsize: tcgsize;const paraloc: TCGParaLocation;reg: tregister;align: longint);
      var
        href : treference;
      begin
         case paraloc.loc of
           LOC_REGISTER :
             begin
               if paraloc.shiftval<0 then
                 a_op_const_reg_reg(list,OP_SHR,OS_INT,-paraloc.shiftval,paraloc.register,paraloc.register);
               case getregtype(reg) of
                 R_ADDRESSREGISTER,
                 R_INTREGISTER:
                   a_load_reg_reg(list,paraloc.size,regsize,paraloc.register,reg);
                 R_MMREGISTER:
                   a_loadmm_intreg_reg(list,paraloc.size,regsize,paraloc.register,reg,mms_movescalar);
                 R_FPUREGISTER:
                   a_loadfpu_intreg_reg(list,paraloc.size,regsize,paraloc.register,reg);
                 else
                   internalerror(2009112422);
               end;
             end;
           LOC_MMREGISTER :
             begin
               case getregtype(reg) of
                 R_ADDRESSREGISTER,
                 R_INTREGISTER:
                   a_loadmm_reg_intreg(list,paraloc.size,regsize,paraloc.register,reg,mms_movescalar);
                 R_MMREGISTER:
                   begin
                     case paraloc.size of
                       OS_F32,
                       OS_F64,
                       OS_F128:
                        a_loadmm_reg_reg(list,paraloc.size,regsize,paraloc.register,reg,mms_movescalar);
                       OS_M8..OS_M512:
                         a_loadmm_reg_reg(list,paraloc.size,paraloc.size,paraloc.register,reg,nil);
                       else
                         internalerror(2010053106);
                     end;
                   end;
                 else
                   internalerror(2010053104);
               end;
             end;
           LOC_FPUREGISTER :
             begin
               case getregtype(reg) of
                 R_FPUREGISTER:
                   a_loadfpu_reg_reg(list,paraloc.size,regsize,paraloc.register,reg);
                 R_INTREGISTER:
                   a_loadfpu_reg_intreg(list,paraloc.size,regsize,paraloc.register,reg);
                 else
                   internalerror(2015031401);
                 end;
             end;
           LOC_REFERENCE :
             begin
               reference_reset_base(href,paraloc.reference.index,paraloc.reference.offset,ctempposinvalid,align,[]);
               case getregtype(reg) of
                 R_ADDRESSREGISTER,
                 R_INTREGISTER :
                   a_load_ref_reg(list,paraloc.size,regsize,href,reg);
                 R_FPUREGISTER :
                   a_loadfpu_ref_reg(list,paraloc.size,regsize,href,reg);
                 R_MMREGISTER :
                   { not paraloc.size, because it may be OS_64 instead of
                     OS_F64 in case the parameter is passed using integer
                     conventions (e.g., on ARM) }
                   a_loadmm_ref_reg(list,regsize,regsize,href,reg,mms_movescalar);
                 else
                   internalerror(2004101012);
               end;
             end;
           else
             internalerror(2002081303);
         end;
      end;


{****************************************************************************
                       some generic implementations
****************************************************************************}

    { memory/register loading }

    procedure tcg.a_load_reg_ref_unaligned(list : TAsmList;fromsize,tosize : tcgsize;register : tregister;const ref : treference);
      var
        tmpref : treference;
        tmpreg : tregister;
        i : longint;
      begin
        if ref.alignment<tcgsize2size[fromsize] then
          begin
            tmpref:=ref;
            { we take care of the alignment now }
            tmpref.alignment:=0;
            case FromSize of
              OS_16,OS_S16:
                begin
                  tmpreg:=getintregister(list,OS_16);
                  a_load_reg_reg(list,fromsize,OS_16,register,tmpreg);
                  if target_info.endian=endian_big then
                    inc(tmpref.offset);
                  tmpreg:=makeregsize(list,tmpreg,OS_8);
                  a_load_reg_ref(list,OS_8,OS_8,tmpreg,tmpref);
                  tmpreg:=makeregsize(list,tmpreg,OS_16);
                  a_op_const_reg(list,OP_SHR,OS_16,8,tmpreg);
                  if target_info.endian=endian_big then
                    dec(tmpref.offset)
                  else
                    inc(tmpref.offset);
                  tmpreg:=makeregsize(list,tmpreg,OS_8);
                  a_load_reg_ref(list,OS_8,OS_8,tmpreg,tmpref);
                end;
              OS_32,OS_S32:
                begin
                  { could add an optimised case for ref.alignment=2 }
                  tmpreg:=getintregister(list,OS_32);
                  a_load_reg_reg(list,fromsize,OS_32,register,tmpreg);
                  if target_info.endian=endian_big then
                    inc(tmpref.offset,3);
                  tmpreg:=makeregsize(list,tmpreg,OS_8);
                  a_load_reg_ref(list,OS_8,OS_8,tmpreg,tmpref);
                  tmpreg:=makeregsize(list,tmpreg,OS_32);
                  for i:=1 to 3 do
                    begin
                      a_op_const_reg(list,OP_SHR,OS_32,8,tmpreg);
                      if target_info.endian=endian_big then
                        dec(tmpref.offset)
                      else
                        inc(tmpref.offset);
                      tmpreg:=makeregsize(list,tmpreg,OS_8);
                      a_load_reg_ref(list,OS_8,OS_8,tmpreg,tmpref);
                      tmpreg:=makeregsize(list,tmpreg,OS_32);
                    end;
                end
              else
                a_load_reg_ref(list,fromsize,tosize,register,tmpref);
            end;
          end
        else
          a_load_reg_ref(list,fromsize,tosize,register,ref);
      end;


    procedure tcg.a_load_ref_reg_unaligned(list : TAsmList;fromsize,tosize : tcgsize;const ref : treference;register : tregister);
      var
        tmpref : treference;
        tmpreg,
        tmpreg2 : tregister;
        i : longint;
        hisize : tcgsize;
      begin
        if ref.alignment in [1,2] then
          begin
            tmpref:=ref;
            { we take care of the alignment now }
            tmpref.alignment:=0;
            case FromSize of
              OS_16,OS_S16:
                if ref.alignment=2 then
                  a_load_ref_reg(list,fromsize,tosize,tmpref,register)
                else
                  begin
                    if FromSize=OS_16 then
                      hisize:=OS_8
                    else
                      hisize:=OS_S8;
                    { first load in tmpreg, because the target register }
                    { may be used in ref as well                        }
                    if target_info.endian=endian_little then
                      inc(tmpref.offset);
                    tmpreg:=getintregister(list,OS_8);
                    a_load_ref_reg(list,hisize,hisize,tmpref,tmpreg);
                    tmpreg:=makeregsize(list,tmpreg,FromSize);
                    a_op_const_reg(list,OP_SHL,FromSize,8,tmpreg);
                    if target_info.endian=endian_little then
                      dec(tmpref.offset)
                    else
                      inc(tmpref.offset);
                    tmpreg2:=makeregsize(list,register,OS_16);
                    a_load_ref_reg(list,OS_8,OS_16,tmpref,tmpreg2);
                    a_op_reg_reg(list,OP_OR,OS_16,tmpreg,tmpreg2);
                    a_load_reg_reg(list,fromsize,tosize,tmpreg2,register);
                  end;
              OS_32,OS_S32:
                if ref.alignment=2 then
                  begin
                    if target_info.endian=endian_little then
                      inc(tmpref.offset,2);
                    tmpreg:=getintregister(list,OS_32);
                    a_load_ref_reg(list,OS_16,OS_32,tmpref,tmpreg);
                    a_op_const_reg(list,OP_SHL,OS_32,16,tmpreg);
                    if target_info.endian=endian_little then
                      dec(tmpref.offset,2)
                    else
                      inc(tmpref.offset,2);
                    tmpreg2:=makeregsize(list,register,OS_32);
                    a_load_ref_reg(list,OS_16,OS_32,tmpref,tmpreg2);
                    a_op_reg_reg(list,OP_OR,OS_32,tmpreg,tmpreg2);
                    a_load_reg_reg(list,fromsize,tosize,tmpreg2,register);
                  end
                else
                  begin
                    if target_info.endian=endian_little then
                      inc(tmpref.offset,3);
                    tmpreg:=getintregister(list,OS_32);
                    a_load_ref_reg(list,OS_8,OS_32,tmpref,tmpreg);
                    tmpreg2:=getintregister(list,OS_32);
                    for i:=1 to 3 do
                      begin
                        a_op_const_reg(list,OP_SHL,OS_32,8,tmpreg);
                        if target_info.endian=endian_little then
                          dec(tmpref.offset)
                        else
                          inc(tmpref.offset);
                        a_load_ref_reg(list,OS_8,OS_32,tmpref,tmpreg2);
                        a_op_reg_reg(list,OP_OR,OS_32,tmpreg2,tmpreg);
                      end;
                    a_load_reg_reg(list,fromsize,tosize,tmpreg,register);
                  end
              else
                a_load_ref_reg(list,fromsize,tosize,tmpref,register);
            end;
          end
        else
          a_load_ref_reg(list,fromsize,tosize,ref,register);
      end;


    procedure tcg.a_load_ref_ref(list : TAsmList;fromsize,tosize : tcgsize;const sref : treference;const dref : treference);
      var
        tmpreg: tregister;
      begin
        { verify if we have the same reference }
        if references_equal(sref,dref) then
          exit;
        tmpreg:=getintregister(list,tosize);
        a_load_ref_reg(list,fromsize,tosize,sref,tmpreg);
        a_load_reg_ref(list,tosize,tosize,tmpreg,dref);
      end;


    procedure tcg.a_load_const_ref(list : TAsmList;size : tcgsize;a : tcgint;const ref : treference);
      var
        tmpreg: tregister;
      begin
        tmpreg:=getintregister(list,size);
        a_load_const_reg(list,size,a,tmpreg);
        a_load_reg_ref(list,size,size,tmpreg,ref);
      end;


    procedure tcg.a_load_const_loc(list : TAsmList;a : tcgint;const loc: tlocation);
      begin
        case loc.loc of
          LOC_REFERENCE,LOC_CREFERENCE:
            a_load_const_ref(list,loc.size,a,loc.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_load_const_reg(list,loc.size,a,loc.register);
          else
            internalerror(200203272);
        end;
      end;


    procedure tcg.a_load_reg_loc(list : TAsmList;fromsize : tcgsize;reg : tregister;const loc: tlocation);
      begin
        case loc.loc of
          LOC_REFERENCE,LOC_CREFERENCE:
            a_load_reg_ref(list,fromsize,loc.size,reg,loc.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_load_reg_reg(list,fromsize,loc.size,reg,loc.register);
          LOC_MMREGISTER,LOC_CMMREGISTER:
            a_loadmm_intreg_reg(list,fromsize,loc.size,reg,loc.register,mms_movescalar);
          else
            internalerror(200203271);
        end;
      end;


    procedure tcg.a_load_loc_reg(list : TAsmList; tosize: tcgsize; const loc: tlocation; reg : tregister);
      begin
        case loc.loc of
          LOC_REFERENCE,LOC_CREFERENCE:
            a_load_ref_reg(list,loc.size,tosize,loc.reference,reg);
          LOC_REGISTER,LOC_CREGISTER:
            a_load_reg_reg(list,loc.size,tosize,loc.register,reg);
          LOC_CONSTANT:
            a_load_const_reg(list,tosize,loc.value,reg);
          LOC_MMREGISTER,LOC_CMMREGISTER:
            a_loadmm_reg_intreg(list,loc.size,tosize,loc.register,reg,mms_movescalar);
          else
            internalerror(200109092);
        end;
      end;


    procedure tcg.a_load_loc_ref(list : TAsmList;tosize: tcgsize; const loc: tlocation; const ref : treference);
      begin
        case loc.loc of
          LOC_REFERENCE,LOC_CREFERENCE:
            a_load_ref_ref(list,loc.size,tosize,loc.reference,ref);
          LOC_REGISTER,LOC_CREGISTER:
            a_load_reg_ref(list,loc.size,tosize,loc.register,ref);
          LOC_CONSTANT:
            a_load_const_ref(list,tosize,loc.value,ref);
          else
            internalerror(200109302);
        end;
      end;


    procedure tcg.optimize_op_const(size: TCGSize; var op: topcg; var a : tcgint);
      var
        powerval : longint;
        signext_a, zeroext_a: tcgint;
      begin
        case size of
          OS_64,OS_S64:
            begin
              signext_a:=int64(a);
              zeroext_a:=int64(a);
            end;
          OS_32,OS_S32:
            begin
              signext_a:=longint(a);
              zeroext_a:=dword(a);
            end;
          OS_16,OS_S16:
            begin
              signext_a:=smallint(a);
              zeroext_a:=word(a);
            end;
          OS_8,OS_S8:
            begin
              signext_a:=shortint(a);
              zeroext_a:=byte(a);
            end
          else
            begin
              { Should we internalerror() here instead? }
              signext_a:=a;
              zeroext_a:=a;
            end;
        end;
        case op of
          OP_OR :
            begin
              { or with zero returns same result }
              if a = 0 then
                op:=OP_NONE
              else
              { or with max returns max }
                if signext_a = -1 then
                  op:=OP_MOVE;
            end;
          OP_AND :
            begin
              { and with max returns same result }
              if (signext_a = -1) then
                op:=OP_NONE
              else
              { and with 0 returns 0 }
                if a=0 then
                  op:=OP_MOVE;
            end;
          OP_XOR :
            begin
              { xor with zero returns same result }
              if a = 0 then
                op:=OP_NONE;
            end;
          OP_DIV :
            begin
              { division by 1 returns result }
              if a = 1 then
                op:=OP_NONE
              else if ispowerof2(int64(zeroext_a), powerval) and not(cs_check_overflow in current_settings.localswitches) then
                begin
                  a := powerval;
                  op:= OP_SHR;
                end;
            end;
          OP_IDIV:
            begin
              if a = 1 then
                op:=OP_NONE;
            end;
         OP_MUL,OP_IMUL:
            begin
               if a = 1 then
                 op:=OP_NONE
               else
                 if a=0 then
                   op:=OP_MOVE
               else if ispowerof2(int64(zeroext_a), powerval) and not(cs_check_overflow in current_settings.localswitches)  then
                 begin
                   a := powerval;
                   op:= OP_SHL;
                 end;
            end;
        OP_ADD,OP_SUB:
            begin
               if a = 0 then
                 op:=OP_NONE;
            end;
        OP_SAR,OP_SHL,OP_SHR:
           begin
             if a = 0 then
               op:=OP_NONE;
           end;
        OP_ROL,OP_ROR:
          begin
            case size of
              OS_64,OS_S64:
                a:=a and 63;
              OS_32,OS_S32:
                a:=a and 31;
              OS_16,OS_S16:
                a:=a and 15;
              OS_8,OS_S8:
                a:=a and 7;
              else
                internalerror(2019050521);
            end;
            if a = 0 then
              op:=OP_NONE;
          end;
        else
          ;
        end;
      end;


    procedure tcg.a_loadfpu_loc_reg(list: TAsmList; tosize: tcgsize; const loc: tlocation; const reg: tregister);
      begin
        case loc.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_loadfpu_ref_reg(list,loc.size,tosize,loc.reference,reg);
          LOC_FPUREGISTER, LOC_CFPUREGISTER:
            a_loadfpu_reg_reg(list,loc.size,tosize,loc.register,reg);
          else
            internalerror(200203301);
        end;
      end;


    procedure tcg.a_loadfpu_reg_loc(list: TAsmList; fromsize: tcgsize; const reg: tregister; const loc: tlocation);
      begin
        case loc.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_loadfpu_reg_ref(list,fromsize,loc.size,reg,loc.reference);
          LOC_FPUREGISTER, LOC_CFPUREGISTER:
            a_loadfpu_reg_reg(list,fromsize,loc.size,reg,loc.register);
          else
            internalerror(48991);
         end;
      end;


    procedure tcg.a_loadfpu_ref_ref(list: TAsmList; fromsize, tosize: tcgsize; const ref1,ref2: treference);
      var
        reg: tregister;
        regsize: tcgsize;
      begin
        if (fromsize>=tosize) then
          regsize:=fromsize
        else
          regsize:=tosize;
        reg:=getfpuregister(list,regsize);
        a_loadfpu_ref_reg(list,fromsize,regsize,ref1,reg);
        a_loadfpu_reg_ref(list,regsize,tosize,reg,ref2);
      end;


    procedure tcg.a_loadfpu_reg_cgpara(list : TAsmList;size : tcgsize;const r : tregister;const cgpara : TCGPara);
      var
         ref : treference;
      begin
        paramanager.alloccgpara(list,cgpara);
         case cgpara.location^.loc of
            LOC_FPUREGISTER,LOC_CFPUREGISTER:
              begin
                cgpara.check_simple_location;
                a_loadfpu_reg_reg(list,size,size,r,cgpara.location^.register);
              end;
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                cgpara.check_simple_location;
                reference_reset_base(ref,cgpara.location^.reference.index,cgpara.location^.reference.offset,ctempposinvalid,cgpara.alignment,[]);
                a_loadfpu_reg_ref(list,size,size,r,ref);
              end;
            LOC_REGISTER,LOC_CREGISTER:
              begin
                { paramfpu_ref does the check_simpe_location check here if necessary }
                tg.GetTemp(list,TCGSize2Size[size],TCGSize2Size[size],tt_normal,ref);
                a_loadfpu_reg_ref(list,size,size,r,ref);
                a_loadfpu_ref_cgpara(list,size,ref,cgpara);
                tg.Ungettemp(list,ref);
              end;
            else
              internalerror(2010053112);
         end;
      end;


    procedure tcg.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const cgpara : TCGPara);
      var
        srcref,
        href : treference;
        srcsize,
        hsize: tcgsize;
        paraloc: PCGParaLocation;
        sizeleft: tcgint;
      begin
        sizeleft:=cgpara.intsize;
        paraloc:=cgpara.location;
        paramanager.alloccgpara(list,cgpara);
        srcref:=ref;
        repeat
          case paraloc^.loc of
            LOC_FPUREGISTER,LOC_CFPUREGISTER:
              begin
                { destination: can be something different in case of a record passed in fpu registers }
                if is_float_cgsize(paraloc^.size) then
                  hsize:=paraloc^.size
                else
                  hsize:=int_float_cgsize(tcgsize2size[paraloc^.size]);
                { source: the size comparison is to catch F128 passed in two 64 bit floating point registers }
                if is_float_cgsize(size) and
                   (tcgsize2size[size]<=tcgsize2size[paraloc^.size]) then
                  srcsize:=size
                else
                  srcsize:=hsize;
                a_loadfpu_ref_reg(list,srcsize,hsize,srcref,paraloc^.register);
              end;
            LOC_REFERENCE,LOC_CREFERENCE:
              begin
                if assigned(paraloc^.next) then
                  internalerror(2020050101);
                reference_reset_base(href,paraloc^.reference.index,paraloc^.reference.offset,ctempposinvalid,newalignment(cgpara.alignment,cgpara.intsize-sizeleft),[]);
                { concatcopy should choose the best way to copy the data }
                g_concatcopy(list,srcref,href,sizeleft);
              end;
            LOC_REGISTER,LOC_CREGISTER:
              begin
                { force integer size }
                hsize:=int_cgsize(tcgsize2size[paraloc^.size]);
  {$ifndef cpu64bitalu}
                if (hsize in [OS_S64,OS_64]) then
                  begin
                    { if this is not a simple location, we'll have to add support to cg64 to load parts of a cgpara }
                    cgpara.check_simple_location;
                    cg64.a_load64_ref_cgpara(list,srcref,cgpara)
                  end
                else
  {$endif not cpu64bitalu}
                  begin
                    a_load_ref_reg(list,hsize,hsize,srcref,paraloc^.register)
                  end;
              end
            else
              internalerror(200402201);
          end;
          inc(srcref.offset,tcgsize2size[paraloc^.size]);
          dec(sizeleft,tcgsize2size[paraloc^.size]);
          paraloc:=paraloc^.next;
        until not assigned(paraloc);
      end;


    procedure tcg.a_loadfpu_intreg_reg(list : TAsmList; fromsize,tosize : tcgsize; intreg,fpureg : tregister);
      var
        tmpref: treference;
      begin
        if not(tcgsize2size[fromsize] in [4,8]) or
           not(tcgsize2size[tosize] in [4,8]) or
           (tcgsize2size[fromsize]<>tcgsize2size[tosize]) then
          internalerror(2017070902);
        tg.gettemp(list,tcgsize2size[fromsize],tcgsize2size[fromsize],tt_normal,tmpref);
        a_load_reg_ref(list,fromsize,fromsize,intreg,tmpref);
        a_loadfpu_ref_reg(list,tosize,tosize,tmpref,fpureg);
        tg.ungettemp(list,tmpref);
      end;


    procedure tcg.a_loadfpu_reg_intreg(list : TAsmList; fromsize,tosize : tcgsize; fpureg,intreg : tregister);
      var
        tmpref: treference;
      begin
        if not(tcgsize2size[fromsize] in [4,8]) or
           not(tcgsize2size[tosize] in [4,8]) or
           (tcgsize2size[fromsize]<>tcgsize2size[tosize]) then
          internalerror(2020091201);
        tg.gettemp(list,tcgsize2size[fromsize],tcgsize2size[fromsize],tt_normal,tmpref);
        a_loadfpu_reg_ref(list,fromsize,fromsize,fpureg,tmpref);
        a_load_ref_reg(list,tosize,tosize,tmpref,intreg);
        tg.ungettemp(list,tmpref);
      end;


    procedure tcg.a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference);
      var
        tmpreg : tregister;
        tmpref : treference;
      begin
        if assigned(ref.symbol)
          { for avrtiny, the code generator generates a ref which is Z relative and while using it,
            Z is changed, so the following code breaks }
          {$ifdef avr}
            and not((CPUAVR_16_REGS in cpu_capabilities[current_settings.cputype]) or (tcgsize2size[size]=1))
          {$endif avr} then
          begin
            tmpreg:=getaddressregister(list);
            a_loadaddr_ref_reg(list,ref,tmpreg);
            reference_reset_base(tmpref,tmpreg,0,ref.temppos,ref.alignment,[]);
          end
        else
          tmpref:=ref;
        tmpreg:=getintregister(list,size);
        a_load_ref_reg(list,size,size,tmpref,tmpreg);
        a_op_const_reg(list,op,size,a,tmpreg);
        a_load_reg_ref(list,size,size,tmpreg,tmpref);
      end;


    procedure tcg.a_op_const_loc(list : TAsmList; Op: TOpCG; a: tcgint; const loc: tlocation);
      begin
        case loc.loc of
          LOC_REGISTER, LOC_CREGISTER:
            a_op_const_reg(list,op,loc.size,a,loc.register);
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op_const_ref(list,op,loc.size,a,loc.reference);
          else
            internalerror(200109061);
        end;
      end;


    procedure tcg.a_op_reg_ref(list : TAsmList; Op: TOpCG; size: TCGSize;reg: TRegister;  const ref: TReference);
      var
        tmpreg : tregister;
        tmpref : treference;
      begin
        if assigned(ref.symbol)
          { for avrtiny, the code generator generates a ref which is Z relative and while using it,
            Z is changed, so the following code breaks }
          {$ifdef avr}
            and not((CPUAVR_16_REGS in cpu_capabilities[current_settings.cputype]) or (tcgsize2size[size]=1))
          {$endif avr} then
          begin
            tmpreg:=getaddressregister(list);
            a_loadaddr_ref_reg(list,ref,tmpreg);
            reference_reset_base(tmpref,tmpreg,0,ref.temppos,ref.alignment,[]);
          end
        else
          tmpref:=ref;
        if op in [OP_NEG,OP_NOT] then
          begin
            tmpreg:=getintregister(list,size);
            a_op_reg_reg(list,op,size,reg,tmpreg);
            a_load_reg_ref(list,size,size,tmpreg,tmpref);
          end
        else
          begin
            tmpreg:=getintregister(list,size);
            a_load_ref_reg(list,size,size,tmpref,tmpreg);
            a_op_reg_reg(list,op,size,reg,tmpreg);
            a_load_reg_ref(list,size,size,tmpreg,tmpref);
          end;
      end;


    procedure tcg.a_op_ref_reg(list : TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference; reg: TRegister);

      var
        tmpreg: tregister;

      begin
        case op of
          OP_NOT,OP_NEG:
            { handle it as "load ref,reg; op reg" }
            begin
              a_load_ref_reg(list,size,size,ref,reg);
              a_op_reg_reg(list,op,size,reg,reg);
            end;
          else
            begin
              tmpreg:=getintregister(list,size);
              a_load_ref_reg(list,size,size,ref,tmpreg);
              a_op_reg_reg(list,op,size,tmpreg,reg);
            end;
        end;
      end;


    procedure tcg.a_op_reg_loc(list : TAsmList; Op: TOpCG; reg: tregister; const loc: tlocation);

      begin
        case loc.loc of
          LOC_REGISTER, LOC_CREGISTER:
            a_op_reg_reg(list,op,loc.size,reg,loc.register);
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op_reg_ref(list,op,loc.size,reg,loc.reference);
          else
            internalerror(2001090602);
        end;
      end;


    procedure tcg.a_op_loc_reg(list : TAsmList; Op : TOpCG; size: TCGSize; const loc : tlocation; reg : tregister);

      begin
        case loc.loc of
          LOC_REGISTER, LOC_CREGISTER:
            a_op_reg_reg(list,op,size,loc.register,reg);
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op_ref_reg(list,op,size,loc.reference,reg);
          LOC_CONSTANT:
            a_op_const_reg(list,op,size,loc.value,reg);
          else
            internalerror(2018031101);
        end;
      end;


    procedure tcg.a_op_ref_loc(list : TAsmList; Op: TOpCG; const ref: TReference; const loc: tlocation);

      var
        tmpreg: tregister;

      begin
        case loc.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_op_ref_reg(list,op,loc.size,ref,loc.register);
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              tmpreg:=getintregister(list,loc.size);
              a_load_ref_reg(list,loc.size,loc.size,ref,tmpreg);
              a_op_reg_ref(list,op,loc.size,tmpreg,loc.reference);
            end;
          else
            internalerror(2001090603);
        end;
      end;


    procedure Tcg.a_op_const_reg_reg(list:TAsmList;op:Topcg;size:Tcgsize;
                                     a:tcgint;src,dst:Tregister);
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
{$ifdef cpu8bitalu}
        OP_SHL:
          begin
            if a=8 then
              case size of
                OS_S16,OS_16:
                  begin
                    a_load_reg_reg(list,OS_8,OS_8,src,GetNextReg(dst));
                    a_load_const_reg(list,OS_8,0,dst);
                    exit;
                  end;
                else
                  ;
              end;
          end;
        OP_SHR:
          begin
            if a=8 then
              case size of
                OS_S16,OS_16:
                  begin
                    a_load_reg_reg(list,OS_8,OS_8,GetNextReg(src),dst);
                    a_load_const_reg(list,OS_8,0,GetNextReg(dst));
                    exit;
                  end;
                else
                  ;
              end;
          end;
{$endif cpu8bitalu}
{$ifdef cpu16bitalu}
        OP_SHL:
          begin
            if a=16 then
              case size of
                OS_S32,OS_32:
                  begin
                    a_load_reg_reg(list,OS_16,OS_16,src,GetNextReg(dst));
                    a_load_const_reg(list,OS_16,0,dst);
                    exit;
                  end;
                else
                  ;
              end;
          end;
        OP_SHR:
          begin
            if a=16 then
              case size of
                OS_S32,OS_32:
                  begin
                    a_load_reg_reg(list,OS_16,OS_16,GetNextReg(src),dst);
                    a_load_const_reg(list,OS_16,0,GetNextReg(dst));
                    exit;
                  end;
                else
                  ;
              end;
          end;
{$endif cpu16bitalu}
        else
          ;
      end;
      a_load_reg_reg(list,size,size,src,dst);
      a_op_const_reg(list,op,size,a,dst);
    end;

    procedure tcg.a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
        size: tcgsize; src1, src2, dst: tregister);
      var
        tmpreg: tregister;
      begin
        if (dst<>src1) then
          begin
            a_load_reg_reg(list,size,size,src2,dst);
            a_op_reg_reg(list,op,size,src1,dst);
          end
        else
          begin
            { can we do a direct operation on the target register ? }
            if op in [OP_ADD,OP_MUL,OP_AND,OP_MOVE,OP_XOR,OP_IMUL,OP_OR] then
              a_op_reg_reg(list,op,size,src2,dst)
            else
              begin
                tmpreg:=getintregister(list,size);
                a_load_reg_reg(list,size,size,src2,tmpreg);
                a_op_reg_reg(list,op,size,src1,tmpreg);
                a_load_reg_reg(list,size,size,tmpreg,dst);
              end;
          end;
      end;


    procedure tcg.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
      begin
        a_op_const_reg_reg(list,op,size,a,src,dst);
        ovloc.loc:=LOC_VOID;
      end;


    procedure tcg.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);
      begin
        a_op_reg_reg_reg(list,op,size,src1,src2,dst);
        ovloc.loc:=LOC_VOID;
      end;


    procedure tcg.a_op_reg(list: TAsmList; Op: TOpCG; size: TCGSize; reg: TRegister);
      begin
        if not (Op in [OP_NOT,OP_NEG]) then
          internalerror(2020050701);
        a_op_reg_reg(list,op,size,reg,reg);
      end;


    procedure tcg.a_op_ref(list: TAsmList; Op: TOpCG; size: TCGSize; const ref: TReference);
      var
        tmpreg: TRegister;
        tmpref: treference;
      begin
        if not (Op in [OP_NOT,OP_NEG]) then
          internalerror(2020050710);
        if assigned(ref.symbol)
          { for avrtiny, the code generator generates a ref which is Z relative and while using it,
            Z is changed, so the following code breaks }
          {$ifdef avr}
            and not((CPUAVR_16_REGS in cpu_capabilities[current_settings.cputype]) or (tcgsize2size[size]=1))
          {$endif avr} then
          begin
            tmpreg:=getaddressregister(list);
            a_loadaddr_ref_reg(list,ref,tmpreg);
            reference_reset_base(tmpref,tmpreg,0,ref.temppos,ref.alignment,[]);
          end
        else
          tmpref:=ref;
        tmpreg:=getintregister(list,size);
        a_load_ref_reg(list,size,size,tmpref,tmpreg);
        a_op_reg_reg(list,op,size,tmpreg,tmpreg);
        a_load_reg_ref(list,size,size,tmpreg,tmpref);
      end;


    procedure tcg.a_op_loc(list: TAsmList; Op: TOpCG; const loc: tlocation);
      begin
        case loc.loc of
          LOC_REGISTER, LOC_CREGISTER:
            a_op_reg(list,op,loc.size,loc.register);
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op_ref(list,op,loc.size,loc.reference);
          else
            internalerror(2020050702);
        end;
      end;


    procedure tcg.a_cmp_const_reg_label(list: TAsmList; size: tcgsize;
      cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel);
      var
        tmpreg: tregister;
      begin
        tmpreg:=getintregister(list,size);
        a_load_const_reg(list,size,a,tmpreg);
        a_cmp_reg_reg_label(list,size,cmp_op,tmpreg,reg,l);
      end;


    procedure tcg.a_cmp_const_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const ref : treference;
      l : tasmlabel);
      var
        tmpreg: tregister;
      begin
        tmpreg:=getintregister(list,size);
        a_load_ref_reg(list,size,size,ref,tmpreg);
        a_cmp_const_reg_label(list,size,cmp_op,a,tmpreg,l);
      end;


    procedure tcg.a_cmp_const_loc_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;const loc : tlocation;
      l : tasmlabel);
      begin
        case loc.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_cmp_const_reg_label(list,size,cmp_op,a,loc.register,l);
          LOC_REFERENCE,LOC_CREFERENCE:
            a_cmp_const_ref_label(list,size,cmp_op,a,loc.reference,l);
          else
            internalerror(2001090604);
        end;
      end;


    procedure tcg.a_cmp_ref_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; const ref: treference; reg : tregister; l : tasmlabel);
      var
        tmpreg: tregister;
      begin
        tmpreg:=getintregister(list,size);
        a_load_ref_reg(list,size,size,ref,tmpreg);
        a_cmp_reg_reg_label(list,size,cmp_op,tmpreg,reg,l);
      end;


    procedure tcg.a_cmp_reg_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; reg : tregister; const ref: treference; l : tasmlabel);
      var
        tmpreg: tregister;
      begin
        tmpreg:=getintregister(list,size);
        a_load_ref_reg(list,size,size,ref,tmpreg);
        a_cmp_reg_reg_label(list,size,cmp_op,reg,tmpreg,l);
      end;


    procedure tcg.a_cmp_reg_loc_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; reg: tregister; const loc: tlocation; l : tasmlabel);
      begin
        a_cmp_loc_reg_label(list,size,swap_opcmp(cmp_op),loc,reg,l);
      end;


    procedure tcg.a_cmp_loc_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp; const loc: tlocation; reg : tregister; l : tasmlabel);
      begin
        case loc.loc of
          LOC_REGISTER,
          LOC_CREGISTER:
            a_cmp_reg_reg_label(list,size,cmp_op,loc.register,reg,l);
          LOC_REFERENCE,
          LOC_CREFERENCE :
            a_cmp_ref_reg_label(list,size,cmp_op,loc.reference,reg,l);
          LOC_CONSTANT:
            a_cmp_const_reg_label(list,size,cmp_op,loc.value,reg,l);
          else
            internalerror(200203231);
        end;
      end;


    procedure tcg.a_cmp_ref_loc_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;const ref: treference;const loc : tlocation;
      l : tasmlabel);
      var
        tmpreg: tregister;
      begin
        case loc.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_cmp_ref_reg_label(list,size,cmp_op,ref,loc.register,l);
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              tmpreg:=getintregister(list,size);
              a_load_ref_reg(list,size,size,loc.reference,tmpreg);
              a_cmp_ref_reg_label(list,size,cmp_op,ref,tmpreg,l);
            end;
          else
            internalerror(2001090605);
        end;
      end;


    procedure tcg.a_loadmm_loc_reg(list: TAsmList; size: tcgsize; const loc: tlocation; const reg: tregister;shuffle : pmmshuffle);
      begin
        case loc.loc of
          LOC_MMREGISTER,LOC_CMMREGISTER:
            a_loadmm_reg_reg(list,loc.size,size,loc.register,reg,shuffle);
          LOC_REFERENCE,LOC_CREFERENCE:
            a_loadmm_ref_reg(list,loc.size,size,loc.reference,reg,shuffle);
          LOC_REGISTER,LOC_CREGISTER:
            a_loadmm_intreg_reg(list,loc.size,size,loc.register,reg,shuffle);
          else
            internalerror(200310121);
        end;
      end;


    procedure tcg.a_loadmm_reg_loc(list: TAsmList; size: tcgsize; const reg: tregister; const loc: tlocation;shuffle : pmmshuffle);
      begin
        case loc.loc of
          LOC_MMREGISTER,LOC_CMMREGISTER:
            a_loadmm_reg_reg(list,size,loc.size,reg,loc.register,shuffle);
          LOC_REFERENCE,LOC_CREFERENCE:
            a_loadmm_reg_ref(list,size,loc.size,reg,loc.reference,shuffle);
          else
            internalerror(200310122);
        end;
      end;


    procedure tcg.a_loadmm_reg_cgpara(list: TAsmList; size: tcgsize; reg: tregister;const cgpara : TCGPara;shuffle : pmmshuffle);
      var
        href  : treference;
{$ifndef cpu64bitalu}
        tmpreg : tregister;
        reg64 : tregister64;
{$endif not cpu64bitalu}
      begin
{$ifndef cpu64bitalu}
         if not(cgpara.location^.loc in [LOC_REGISTER,LOC_CREGISTER]) or
            (size<>OS_F64) then
{$endif not cpu64bitalu}
           cgpara.check_simple_location;
         paramanager.alloccgpara(list,cgpara);
         case cgpara.location^.loc of
          LOC_MMREGISTER,LOC_CMMREGISTER:
            a_loadmm_reg_reg(list,size,cgpara.location^.size,reg,cgpara.location^.register,shuffle);
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              reference_reset_base(href,cgpara.location^.reference.index,cgpara.location^.reference.offset,ctempposinvalid,cgpara.alignment,[]);
              a_loadmm_reg_ref(list,size,cgpara.location^.size,reg,href,shuffle);
            end;
          LOC_REGISTER,LOC_CREGISTER:
            begin
              if assigned(shuffle) and
                 not shufflescalar(shuffle) then
                internalerror(2009112510);
{$ifndef cpu64bitalu}
              if (size=OS_F64) then
                begin
                  if not assigned(cgpara.location^.next) or
                     assigned(cgpara.location^.next^.next) then
                    internalerror(2009112512);
                  case cgpara.location^.next^.loc of
                    LOC_REGISTER,LOC_CREGISTER:
                      tmpreg:=cgpara.location^.next^.register;
                    LOC_REFERENCE,LOC_CREFERENCE:
                      tmpreg:=getintregister(list,OS_32);
                    else
                      internalerror(2009112910);
                  end;
                  if (target_info.endian=ENDIAN_BIG) then
                    begin
                      { paraloc^ -> high
                        paraloc^.next -> low }
                      reg64.reghi:=cgpara.location^.register;
                      reg64.reglo:=tmpreg;
                    end
                  else
                    begin
                      { paraloc^ -> low
                        paraloc^.next -> high }
                      reg64.reglo:=cgpara.location^.register;
                      reg64.reghi:=tmpreg;
                    end;
                  cg64.a_loadmm_reg_intreg64(list,size,reg,reg64);
                  if (cgpara.location^.next^.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                    begin
                      if not(cgpara.location^.next^.size in [OS_32,OS_S32]) then
                        internalerror(2009112911);
                      reference_reset_base(href,cgpara.location^.next^.reference.index,cgpara.location^.next^.reference.offset,ctempposinvalid,cgpara.alignment,[]);
                      a_load_reg_ref(list,OS_32,cgpara.location^.next^.size,tmpreg,href);
                    end;
                end
              else
{$endif not cpu64bitalu}
                a_loadmm_reg_intreg(list,size,cgpara.location^.size,reg,cgpara.location^.register,mms_movescalar);
            end
          else
            internalerror(200310123);
        end;
      end;


    procedure tcg.a_loadmm_ref_cgpara(list: TAsmList; size: tcgsize;const ref: treference;const cgpara : TCGPara;shuffle : pmmshuffle);
      var
         hr : tregister;
         hs : tmmshuffle;
      begin
         cgpara.check_simple_location;
         hr:=getmmregister(list,cgpara.location^.size);
         a_loadmm_ref_reg(list,size,cgpara.location^.size,ref,hr,shuffle);
         if realshuffle(shuffle) then
           begin
             hs:=shuffle^;
             removeshuffles(hs);
             a_loadmm_reg_cgpara(list,cgpara.location^.size,hr,cgpara,@hs);
           end
         else
           a_loadmm_reg_cgpara(list,cgpara.location^.size,hr,cgpara,shuffle);
      end;


    procedure tcg.a_loadmm_loc_cgpara(list: TAsmList;const loc: tlocation; const cgpara : TCGPara;shuffle : pmmshuffle);
      begin
        case loc.loc of
          LOC_MMREGISTER,LOC_CMMREGISTER:
            a_loadmm_reg_cgpara(list,loc.size,loc.register,cgpara,shuffle);
          LOC_REFERENCE,LOC_CREFERENCE:
            a_loadmm_ref_cgpara(list,loc.size,loc.reference,cgpara,shuffle);
          else
            internalerror(2003101204);
        end;
      end;


    procedure tcg.a_opmm_ref_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle);
      var
         hr : tregister;
         hs : tmmshuffle;
      begin
         hr:=getmmregister(list,size);
         a_loadmm_ref_reg(list,size,size,ref,hr,shuffle);
         if realshuffle(shuffle) then
           begin
             hs:=shuffle^;
             removeshuffles(hs);
             a_opmm_reg_reg(list,op,size,hr,reg,@hs);
           end
         else
           a_opmm_reg_reg(list,op,size,hr,reg,shuffle);
      end;


    procedure tcg.a_opmm_reg_ref(list: TAsmList; Op: TOpCG; size : tcgsize;reg: tregister; const ref: treference; shuffle : pmmshuffle);
      var
         hr : tregister;
         hs : tmmshuffle;
      begin
         hr:=getmmregister(list,size);
         a_loadmm_ref_reg(list,size,size,ref,hr,shuffle);
         if realshuffle(shuffle) then
           begin
             hs:=shuffle^;
             removeshuffles(hs);
             a_opmm_reg_reg(list,op,size,reg,hr,@hs);
             a_loadmm_reg_ref(list,size,size,hr,ref,@hs);
           end
         else
           begin
             a_opmm_reg_reg(list,op,size,reg,hr,shuffle);
             a_loadmm_reg_ref(list,size,size,hr,ref,shuffle);
           end;
      end;


    procedure tcg.a_loadmm_intreg_reg(list: tasmlist; fromsize,tosize: tcgsize; intreg,mmreg: tregister; shuffle: pmmshuffle);
      var
        tmpref: treference;
      begin
        if (tcgsize2size[fromsize]<>4) or
           (tcgsize2size[tosize]<>4) then
          internalerror(2009112503);
        tg.gettemp(list,4,4,tt_normal,tmpref);
        a_load_reg_ref(list,fromsize,fromsize,intreg,tmpref);
        a_loadmm_ref_reg(list,tosize,tosize,tmpref,mmreg,shuffle);
        tg.ungettemp(list,tmpref);
      end;


    procedure tcg.a_loadmm_reg_intreg(list: tasmlist; fromsize,tosize: tcgsize; mmreg,intreg: tregister; shuffle: pmmshuffle);
      var
        tmpref: treference;
      begin
        if (tcgsize2size[fromsize]<>4) or
           (tcgsize2size[tosize]<>4) then
          internalerror(2009112504);
        tg.gettemp(list,8,8,tt_normal,tmpref);
        a_loadmm_reg_ref(list,fromsize,fromsize,mmreg,tmpref,shuffle);
        a_load_ref_reg(list,tosize,tosize,tmpref,intreg);
        tg.ungettemp(list,tmpref);
      end;


    procedure tcg.a_opmm_loc_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const loc: tlocation; reg: tregister;shuffle : pmmshuffle);
      begin
        case loc.loc of
          LOC_CMMREGISTER,LOC_MMREGISTER:
            a_opmm_reg_reg(list,op,size,loc.register,reg,shuffle);
          LOC_CREFERENCE,LOC_REFERENCE:
            a_opmm_ref_reg(list,op,size,loc.reference,reg,shuffle);
          else
            internalerror(200312232);
        end;
      end;


    procedure tcg.a_opmm_loc_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;const loc: tlocation; src,dst: tregister;shuffle : pmmshuffle);
      begin
        case loc.loc of
          LOC_CMMREGISTER,LOC_MMREGISTER:
            a_opmm_reg_reg_reg(list,op,size,loc.register,src,dst,shuffle);
          LOC_CREFERENCE,LOC_REFERENCE:
            a_opmm_ref_reg_reg(list,op,size,loc.reference,src,dst,shuffle);
          else
            internalerror(2003122304);
        end;
      end;


    procedure tcg.a_opmm_reg_reg_reg(list : TAsmList;Op : TOpCG;size : tcgsize;
      src1,src2,dst : tregister;shuffle : pmmshuffle);
      begin
        internalerror(2013061102);
      end;


    procedure tcg.a_opmm_ref_reg_reg(list : TAsmList;Op : TOpCG;size : tcgsize;
      const ref : treference;src,dst : tregister;shuffle : pmmshuffle);
      begin
        internalerror(2013061101);
      end;


    procedure tcg.g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);
      begin
        g_concatcopy(list,source,dest,len);
      end;


    procedure tcg.g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);
      begin
        g_overflowCheck(list,loc,def);
      end;


{$ifdef cpuflags}
    procedure tcg.g_flags2ref(list: TAsmList; size: TCgSize; const f: tresflags; const ref:TReference);

      var
        tmpreg : tregister;
      begin
        tmpreg:=getintregister(list,size);
        g_flags2reg(list,size,f,tmpreg);
        a_load_reg_ref(list,size,size,tmpreg,ref);
      end;
{$endif cpuflags}


{*****************************************************************************
                            Entry/Exit Code Functions
*****************************************************************************}


    procedure tcg.g_save_registers(list:TAsmList);
      var
        href : treference;
        size : longint;
        r : integer;
        regs_to_save_int,
        regs_to_save_address,
        regs_to_save_mm : tcpuregisterarray;
      begin
        regs_to_save_int:=paramanager.get_saved_registers_int(current_procinfo.procdef.proccalloption);
        regs_to_save_address:=paramanager.get_saved_registers_address(current_procinfo.procdef.proccalloption);
        regs_to_save_mm:=paramanager.get_saved_registers_mm(current_procinfo.procdef.proccalloption);
        { calculate temp. size }
        size:=0;
        for r:=low(regs_to_save_int) to high(regs_to_save_int) do
          if regs_to_save_int[r] in rg[R_INTREGISTER].used_in_proc then
            inc(size,sizeof(aint));
        if uses_registers(R_ADDRESSREGISTER) then
          for r:=low(regs_to_save_int) to high(regs_to_save_int) do
            if regs_to_save_int[r] in rg[R_ADDRESSREGISTER].used_in_proc then
              inc(size,sizeof(aint));

        { mm registers }
        if uses_registers(R_MMREGISTER) then
          begin
            { Make sure we reserve enough space to do the alignment based on the offset
              later on. We can't use the size for this, because the alignment of the start
              of the temp is smaller than needed for an OS_VECTOR }
            inc(size,tcgsize2size[OS_VECTOR]);

            for r:=low(regs_to_save_mm) to high(regs_to_save_mm) do
              if regs_to_save_mm[r] in rg[R_MMREGISTER].used_in_proc then
                inc(size,tcgsize2size[OS_VECTOR]);
          end;

        if size>0 then
          begin
            tg.GetTemp(list,size,sizeof(aint),tt_noreuse,current_procinfo.save_regs_ref);
            include(current_procinfo.flags,pi_has_saved_regs);

            { Copy registers to temp }
            href:=current_procinfo.save_regs_ref;
            for r:=low(regs_to_save_int) to high(regs_to_save_int) do
              begin
                if regs_to_save_int[r] in rg[R_INTREGISTER].used_in_proc then
                  begin
                    a_load_reg_ref(list,OS_ADDR,OS_ADDR,newreg(R_INTREGISTER,regs_to_save_int[r],R_SUBWHOLE),href);
                    inc(href.offset,sizeof(aint));
                  end;
                include(rg[R_INTREGISTER].preserved_by_proc,regs_to_save_int[r]);
              end;

            if uses_registers(R_ADDRESSREGISTER) then
              for r:=low(regs_to_save_address) to high(regs_to_save_address) do
                begin
                  if regs_to_save_address[r] in rg[R_ADDRESSREGISTER].used_in_proc then
                    begin
                      a_load_reg_ref(list,OS_ADDR,OS_ADDR,newreg(R_ADDRESSREGISTER,regs_to_save_address[r],R_SUBWHOLE),href);
                      inc(href.offset,sizeof(aint));
                    end;
                  include(rg[R_ADDRESSREGISTER].preserved_by_proc,regs_to_save_address[r]);
                end;

            if uses_registers(R_MMREGISTER) then
              begin
                if (href.offset mod tcgsize2size[OS_VECTOR])<>0 then
                  inc(href.offset,tcgsize2size[OS_VECTOR]-(href.offset mod tcgsize2size[OS_VECTOR]));

                for r:=low(regs_to_save_mm) to high(regs_to_save_mm) do
                  begin
                    { the array has to be declared even if no MM registers are saved
                      (such as with SSE on i386), and since 0-element arrays don't
                      exist, they contain a single RS_INVALID element in that case
                    }
                    if regs_to_save_mm[r]<>RS_INVALID then
                      begin
                        if regs_to_save_mm[r] in rg[R_MMREGISTER].used_in_proc then
                          begin
                            a_loadmm_reg_ref(list,OS_VECTOR,OS_VECTOR,newreg(R_MMREGISTER,regs_to_save_mm[r],R_SUBMMWHOLE),href,nil);
                            inc(href.offset,tcgsize2size[OS_VECTOR]);
                          end;
                        include(rg[R_MMREGISTER].preserved_by_proc,regs_to_save_mm[r]);
                      end;
                  end;
              end;
          end;
      end;


    procedure tcg.g_restore_registers(list:TAsmList);
      var
        href     : treference;
        r        : integer;
        hreg     : tregister;
        regs_to_save_int,
        regs_to_save_address,
        regs_to_save_mm : tcpuregisterarray;
      begin
        if not(pi_has_saved_regs in current_procinfo.flags) then
          exit;
        regs_to_save_int:=paramanager.get_saved_registers_int(current_procinfo.procdef.proccalloption);
        regs_to_save_address:=paramanager.get_saved_registers_address(current_procinfo.procdef.proccalloption);
        regs_to_save_mm:=paramanager.get_saved_registers_mm(current_procinfo.procdef.proccalloption);
        { Copy registers from temp }
        href:=current_procinfo.save_regs_ref;
        for r:=low(regs_to_save_int) to high(regs_to_save_int) do
          if regs_to_save_int[r] in rg[R_INTREGISTER].used_in_proc then
            begin
              hreg:=newreg(R_INTREGISTER,regs_to_save_int[r],R_SUBWHOLE);
              { Allocate register so the optimizer does not remove the load }
              a_reg_alloc(list,hreg);
              a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,hreg);
              inc(href.offset,sizeof(aint));
            end;

        if uses_registers(R_ADDRESSREGISTER) then
          for r:=low(regs_to_save_address) to high(regs_to_save_address) do
            if regs_to_save_address[r] in rg[R_ADDRESSREGISTER].used_in_proc then
              begin
                hreg:=newreg(R_ADDRESSREGISTER,regs_to_save_address[r],R_SUBWHOLE);
                { Allocate register so the optimizer does not remove the load }
                a_reg_alloc(list,hreg);
                a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,hreg);
                inc(href.offset,sizeof(aint));
              end;

        if uses_registers(R_MMREGISTER) then
          begin
            if (href.offset mod tcgsize2size[OS_VECTOR])<>0 then
              inc(href.offset,tcgsize2size[OS_VECTOR]-(href.offset mod tcgsize2size[OS_VECTOR]));

            for r:=low(regs_to_save_mm) to high(regs_to_save_mm) do
              begin
                if regs_to_save_mm[r] in rg[R_MMREGISTER].used_in_proc then
                  begin
                    hreg:=newreg(R_MMREGISTER,regs_to_save_mm[r],R_SUBMMWHOLE);
                    { Allocate register so the optimizer does not remove the load }
                    a_reg_alloc(list,hreg);
                    a_loadmm_ref_reg(list,OS_VECTOR,OS_VECTOR,href,hreg,nil);
                    inc(href.offset,tcgsize2size[OS_VECTOR]);
                  end;
              end;
          end;
        tg.UnGetTemp(list,current_procinfo.save_regs_ref);
      end;


    procedure tcg.g_profilecode(list : TAsmList);
      begin
      end;


    procedure tcg.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
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
          internalerror(200305251);
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
                    a_op_const_ref(list,OP_SUB,size,ioffset,href);
                  end
                else
                  internalerror(200309189);
              end;
              paraloc:=next;
            end;
      end;


    procedure tcg.a_call_name_static(list : TAsmList;const s : string);
      begin
        a_call_name(list,s,false);
      end;


   function tcg.g_indirect_sym_load(list:TAsmList;const symname: string; const flags: tindsymflags): tregister;
      var
        l: tasmsymbol;
        ref: treference;
        nlsymname: string;
        symtyp: TAsmsymtype;
      begin
        result := NR_NO;
        case target_info.system of
          system_powerpc_darwin,
          system_i386_darwin,
          system_i386_iphonesim,
          system_powerpc64_darwin,
          system_arm_ios:
            begin
              nlsymname:='L'+symname+'$non_lazy_ptr';
              l:=current_asmdata.getasmsymbol(nlsymname);
              if not(assigned(l)) then
                begin
                  if is_data in flags then
                    symtyp:=AT_DATA
                  else
                    symtyp:=AT_FUNCTION;
                  new_section(current_asmdata.asmlists[al_picdata],sec_data_nonlazy,'',sizeof(pint));
                  l:=current_asmdata.DefineAsmSymbol(nlsymname,AB_LOCAL,AT_DATA,voidpointertype);
                  current_asmdata.asmlists[al_picdata].concat(tai_symbol.create(l,0));
                  if not(is_weak in flags) then
                    current_asmdata.asmlists[al_picdata].concat(tai_directive.Create(asd_indirect_symbol,current_asmdata.RefAsmSymbol(symname,symtyp).Name))
                  else
                    current_asmdata.asmlists[al_picdata].concat(tai_directive.Create(asd_indirect_symbol,current_asmdata.WeakRefAsmSymbol(symname,symtyp).Name));
{$ifdef cpu64bitaddr}
                  current_asmdata.asmlists[al_picdata].concat(tai_const.create_64bit(0));
{$else cpu64bitaddr}
                  current_asmdata.asmlists[al_picdata].concat(tai_const.create_32bit(0));
{$endif cpu64bitaddr}
                end;
              result := getaddressregister(list);
              reference_reset_symbol(ref,l,0,sizeof(pint),[]);
              { a_load_ref_reg will turn this into a pic-load if needed }
              a_load_ref_reg(list,OS_ADDR,OS_ADDR,ref,result);
            end;
          else
            ;
        end;
      end;


    procedure tcg.g_maybe_got_init(list: TAsmList);
      begin
      end;


    procedure tcg.g_maybe_tls_init(list: TAsmList);
      begin
      end;


    procedure tcg.g_call(list: TAsmList;const s: string);
      begin
        allocallcpuregisters(list);
        if systemunit<>current_module.globalsymtable then
          current_module.add_extern_asmsym(s,AB_EXTERNAL,AT_FUNCTION);
        a_call_name(list,s,false);
        deallocallcpuregisters(list);
      end;

    procedure tcg.g_local_unwind(list: TAsmList; l: TAsmLabel);
      begin
        a_jmp_always(list,l);
      end;

    procedure tcg.a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister; shuffle: pmmshuffle);
      begin
        internalerror(200807231);
      end;


    procedure tcg.a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister; shuffle: pmmshuffle);
      begin
        internalerror(200807232);
      end;


    procedure tcg.a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference; shuffle: pmmshuffle);
      begin
        internalerror(200807233);
      end;


    procedure tcg.a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tcgsize; src, dst: tregister; shuffle: pmmshuffle);
      begin
        internalerror(200807234);
      end;


    function tcg.getflagregister(list: TAsmList; size: Tcgsize): Tregister;
      begin
        Result:=TRegister(0);
        internalerror(200807238);
      end;


    procedure tcg.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tcgsize; src, dst: TRegister);
      begin
        internalerror(2014070601);
      end;


    procedure tcg.g_stackpointer_alloc(list: TAsmList; size: longint);
      begin
        internalerror(2014070602);
      end;


    procedure tcg.a_mul_reg_reg_pair(list: TAsmList; size: TCgSize; src1,src2,dstlo,dsthi: TRegister);
      begin
        internalerror(2014060801);
      end;


    procedure tcg.g_div_const_reg_reg(list:tasmlist; size: TCgSize; a: tcgint; src,dst: tregister);
      var
        divreg: tregister;
        magic: aInt;
        u_magic: aWord;
        u_shift: byte;
        u_add: boolean;
      begin
        divreg:=getintregister(list,OS_INT);
        if (size in [OS_S32,OS_S64]) then
          begin
            calc_divconst_magic_signed(tcgsize2size[size]*8,a,magic,u_shift);
            { load magic value }
            a_load_const_reg(list,OS_INT,magic,divreg);
            { multiply, discarding low bits }
            a_mul_reg_reg_pair(list,size,src,divreg,NR_NO,dst);
            { add/subtract numerator }
            if (a>0) and (magic<0) then
              a_op_reg_reg_reg(list,OP_ADD,OS_INT,src,dst,dst)
            else if (a<0) and (magic>0) then
              a_op_reg_reg_reg(list,OP_SUB,OS_INT,src,dst,dst);
            { shift shift places to the right (arithmetic) }
            a_op_const_reg_reg(list,OP_SAR,OS_INT,u_shift,dst,dst);
            { extract and add sign bit }
            if (a>=0) then
              a_op_const_reg_reg(list,OP_SHR,OS_INT,tcgsize2size[size]*8-1,src,divreg)
            else
              a_op_const_reg_reg(list,OP_SHR,OS_INT,tcgsize2size[size]*8-1,dst,divreg);
            a_op_reg_reg_reg(list,OP_ADD,OS_INT,dst,divreg,dst);
          end
        else if (size in [OS_32,OS_64]) then
          begin
            calc_divconst_magic_unsigned(tcgsize2size[size]*8,a,u_magic,u_add,u_shift);
            { load magic in divreg }
            a_load_const_reg(list,OS_INT,tcgint(u_magic),divreg);
            { multiply, discarding low bits }
            a_mul_reg_reg_pair(list,size,src,divreg,NR_NO,dst);
            if (u_add) then
              begin
                { Calculate "(numerator+result) shr u_shift", avoiding possible overflow }
                a_op_reg_reg_reg(list,OP_SUB,OS_INT,dst,src,divreg);
                { divreg=(numerator-result) }
                a_op_const_reg_reg(list,OP_SHR,OS_INT,1,divreg,divreg);
                { divreg=(numerator-result)/2 }
                a_op_reg_reg_reg(list,OP_ADD,OS_INT,divreg,dst,divreg);
                { divreg=(numerator+result)/2, already shifted by 1, so decrease u_shift. }
                a_op_const_reg_reg(list,OP_SHR,OS_INT,u_shift-1,divreg,dst);
              end
            else
              a_op_const_reg_reg(list,OP_SHR,OS_INT,u_shift,dst,dst);
          end
        else
          InternalError(2014060601);
      end;


    procedure tcg.g_check_for_fpu_exception(list: TAsmList;force,clear : boolean);
      begin
        { empty by default }
      end;


    procedure tcg.maybe_check_for_fpu_exception(list: TAsmList);
      begin
        current_procinfo.FPUExceptionCheckNeeded:=true;
        g_check_for_fpu_exception(list,false,true);
      end;

{*****************************************************************************
                                    TCG64
*****************************************************************************}

{$ifndef cpu64bitalu}
    function joinreg64(reglo,reghi : tregister) : tregister64;
      begin
         result.reglo:=reglo;
         result.reghi:=reghi;
      end;


    procedure tcg64.a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64; regsrc,regdst : tregister64);
      begin
        a_load64_reg_reg(list,regsrc,regdst);
        a_op64_const_reg(list,op,size,value,regdst);
      end;


    procedure tcg64.a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);
      var
        tmpreg64 : tregister64;
      begin
        { when src1=dst then we need to first create a temp to prevent
          overwriting src1 with src2 }
        if (regsrc1.reghi=regdst.reghi) or
           (regsrc1.reglo=regdst.reghi) or
           (regsrc1.reghi=regdst.reglo) or
           (regsrc1.reglo=regdst.reglo) then
          begin
            tmpreg64.reglo:=cg.getintregister(list,OS_32);
            tmpreg64.reghi:=cg.getintregister(list,OS_32);
            a_load64_reg_reg(list,regsrc2,tmpreg64);
            a_op64_reg_reg(list,op,size,regsrc1,tmpreg64);
            a_load64_reg_reg(list,tmpreg64,regdst);
          end
        else
          begin
            a_load64_reg_reg(list,regsrc2,regdst);
            a_op64_reg_reg(list,op,size,regsrc1,regdst);
          end;
      end;


    procedure tcg64.a_op64_const_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; a : int64; const sref: tsubsetreference);
      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_subsetref_reg(list,sref,tmpreg64);
        a_op64_const_reg(list,op,size,a,tmpreg64);
        a_load64_reg_subsetref(list,tmpreg64,sref);
      end;


    procedure tcg64.a_op64_reg_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; reg: tregister64; const sref: tsubsetreference);
      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_subsetref_reg(list,sref,tmpreg64);
        a_op64_reg_reg(list,op,size,reg,tmpreg64);
        a_load64_reg_subsetref(list,tmpreg64,sref);
      end;


    procedure tcg64.a_op64_ref_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; const ref: treference; const sref: tsubsetreference);
      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_subsetref_reg(list,sref,tmpreg64);
        a_op64_ref_reg(list,op,size,ref,tmpreg64);
        a_load64_reg_subsetref(list,tmpreg64,sref);
      end;


    procedure tcg64.a_op64_subsetref_subsetref(list : TAsmList; Op : TOpCG; size : TCGSize; const ssref,dsref: tsubsetreference);
      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_subsetref_reg(list,ssref,tmpreg64);
        a_op64_reg_subsetref(list,op,size,tmpreg64,dsref);
      end;


    procedure tcg64.a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      begin
        a_op64_const_reg_reg(list,op,size,value,regsrc,regdst);
        ovloc.loc:=LOC_VOID;
      end;


    procedure tcg64.a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      begin
        a_op64_reg_reg_reg(list,op,size,regsrc1,regsrc2,regdst);
        ovloc.loc:=LOC_VOID;
      end;


    procedure tcg64.a_op64_reg(list: TAsmList; op: TOpCG; size: tcgsize; regdst: tregister64);
      begin
        if not (op in [OP_NOT,OP_NEG]) then
          internalerror(2020050706);
        a_op64_reg_reg(list,op,size,regdst,regdst);
      end;


    procedure tcg64.a_op64_ref(list: TAsmList; op: TOpCG; size: tcgsize; const ref: treference);
      var
        tempreg: tregister64;
      begin
        if not (op in [OP_NOT,OP_NEG]) then
          internalerror(2020050713);
        tempreg.reghi:=cg.getintregister(list,OS_32);
        tempreg.reglo:=cg.getintregister(list,OS_32);
        a_load64_ref_reg(list,ref,tempreg);
        a_op64_reg_reg(list,op,size,tempreg,tempreg);
        a_load64_reg_ref(list,tempreg,ref);
      end;


    procedure tcg64.a_op64_loc(list: TAsmList; op: TOpCG; size: tcgsize; const l: tlocation);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op64_ref(list,op,size,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_op64_reg(list,op,size,l.register64);
          else
            internalerror(2020050707);
        end;
      end;


    procedure tcg64.a_load64_loc_subsetref(list : TAsmList;const l: tlocation; const sref : tsubsetreference);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_load64_ref_subsetref(list,l.reference,sref);
          LOC_REGISTER,LOC_CREGISTER:
            a_load64_reg_subsetref(list,l.register64,sref);
          LOC_CONSTANT :
            a_load64_const_subsetref(list,l.value64,sref);
          LOC_SUBSETREF,LOC_CSUBSETREF:
            a_load64_subsetref_subsetref(list,l.sref,sref);
          else
            internalerror(2006082210);
        end;
      end;


    procedure tcg64.a_load64_subsetref_loc(list: TAsmlist; const sref: tsubsetreference; const l: tlocation);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_load64_subsetref_ref(list,sref,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_load64_subsetref_reg(list,sref,l.register64);
          LOC_SUBSETREF,LOC_CSUBSETREF:
            a_load64_subsetref_subsetref(list,sref,l.sref);
          else
            internalerror(2006082211);
        end;
      end;
{$else cpu64bitalu}

    function joinreg128(reglo, reghi: tregister): tregister128;
      begin
        result.reglo:=reglo;
        result.reghi:=reghi;
      end;


    procedure splitparaloc128(const cgpara:tcgpara;var cgparalo,cgparahi:tcgpara);
      var
        paraloclo,
        paralochi : pcgparalocation;
      begin
        if not(cgpara.size in [OS_128,OS_S128]) then
          internalerror(2012090604);
        if not assigned(cgpara.location) then
          internalerror(2012090605);
        { init lo/hi para }
        cgparahi.reset;
        if cgpara.size=OS_S128 then
          cgparahi.size:=OS_S64
        else
          cgparahi.size:=OS_64;
        cgparahi.intsize:=8;
        cgparahi.alignment:=cgpara.alignment;
        paralochi:=cgparahi.add_location;
        cgparalo.reset;
        cgparalo.size:=OS_64;
        cgparalo.intsize:=8;
        cgparalo.alignment:=cgpara.alignment;
        paraloclo:=cgparalo.add_location;
        { 2 parameter fields? }
        if assigned(cgpara.location^.next) then
          begin
            { Order for multiple locations is always
                paraloc^ -> high
                paraloc^.next -> low }
            if (target_info.endian=ENDIAN_BIG) then
              begin
                { paraloc^ -> high
                  paraloc^.next -> low }
                move(cgpara.location^,paralochi^,sizeof(paralochi^));
                move(cgpara.location^.next^,paraloclo^,sizeof(paraloclo^));
              end
            else
              begin
                { paraloc^ -> low
                  paraloc^.next -> high }
                move(cgpara.location^,paraloclo^,sizeof(paraloclo^));
                move(cgpara.location^.next^,paralochi^,sizeof(paralochi^));
              end;
          end
        else
          begin
            { single parameter, this can only be in memory }
            if cgpara.location^.loc<>LOC_REFERENCE then
              internalerror(2012090606);
            move(cgpara.location^,paraloclo^,sizeof(paraloclo^));
            move(cgpara.location^,paralochi^,sizeof(paralochi^));
            { for big endian low is at +8, for little endian high }
            if target_info.endian = endian_big then
              begin
                inc(cgparalo.location^.reference.offset,8);
                cgparalo.alignment:=newalignment(cgparalo.alignment,8);
              end
            else
              begin
                inc(cgparahi.location^.reference.offset,8);
                cgparahi.alignment:=newalignment(cgparahi.alignment,8);
              end;
          end;
        { fix size }
        paraloclo^.size:=cgparalo.size;
        paraloclo^.next:=nil;
        paralochi^.size:=cgparahi.size;
        paralochi^.next:=nil;
      end;


    procedure tcg128.a_load128_reg_reg(list: TAsmList; regsrc,
      regdst: tregister128);
      begin
        cg.a_load_reg_reg(list,OS_64,OS_64,regsrc.reglo,regdst.reglo);
        cg.a_load_reg_reg(list,OS_64,OS_64,regsrc.reghi,regdst.reghi);
      end;


    procedure tcg128.a_load128_reg_ref(list: TAsmList; reg: tregister128;
      const ref: treference);
      var
        tmpreg: tregister;
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          begin
            tmpreg:=reg.reglo;
            reg.reglo:=reg.reghi;
            reg.reghi:=tmpreg;
          end;
        cg.a_load_reg_ref(list,OS_64,OS_64,reg.reglo,ref);
        tmpref := ref;
        inc(tmpref.offset,8);
        cg.a_load_reg_ref(list,OS_64,OS_64,reg.reghi,tmpref);
      end;


    procedure tcg128.a_load128_ref_reg(list: TAsmList; const ref: treference;
      reg: tregister128);
      var
        tmpreg: tregister;
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          begin
            tmpreg := reg.reglo;
            reg.reglo := reg.reghi;
            reg.reghi := tmpreg;
          end;
        tmpref := ref;
        if (tmpref.base=reg.reglo) then
         begin
           tmpreg:=cg.getaddressregister(list);
           cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpref.base,tmpreg);
           tmpref.base:=tmpreg;
         end
        else
         { this works only for the i386, thus the i386 needs to override  }
         { this method and this method must be replaced by a more generic }
         { implementation FK                                              }
         if (tmpref.index=reg.reglo) then
          begin
            tmpreg:=cg.getaddressregister(list);
            cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpref.index,tmpreg);
            tmpref.index:=tmpreg;
          end;
        cg.a_load_ref_reg(list,OS_64,OS_64,tmpref,reg.reglo);
        inc(tmpref.offset,8);
        cg.a_load_ref_reg(list,OS_64,OS_64,tmpref,reg.reghi);
      end;


    procedure tcg128.a_load128_loc_ref(list: TAsmList; const l: tlocation;
      const ref: treference);
      begin
        case l.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_load128_reg_ref(list,l.register128,ref);
          { not yet implemented:
          LOC_CONSTANT :
            a_load128_const_ref(list,l.value128,ref);
          LOC_SUBSETREF, LOC_CSUBSETREF:
            a_load64_subsetref_ref(list,l.sref,ref); }
          else
            internalerror(201209061);
        end;
      end;


    procedure tcg128.a_load128_reg_loc(list: TAsmList; reg: tregister128;
      const l: tlocation);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_load128_reg_ref(list,reg,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_load128_reg_reg(list,reg,l.register128);
          { not yet implemented:
          LOC_SUBSETREF, LOC_CSUBSETREF:
            a_load64_reg_subsetref(list,reg,l.sref);
          LOC_MMREGISTER, LOC_CMMREGISTER:
            a_loadmm_intreg64_reg(list,l.size,reg,l.register); }
          else
            internalerror(201209062);
        end;
      end;

    procedure tcg128.a_load128_const_reg(list: TAsmList; valuelo,
     valuehi: int64; reg: tregister128);
     begin
       cg.a_load_const_reg(list,OS_64,aint(valuelo),reg.reglo);
       cg.a_load_const_reg(list,OS_64,aint(valuehi),reg.reghi);
     end;


    procedure tcg128.a_load128_loc_cgpara(list: TAsmList; const l: tlocation;
      const paraloc: TCGPara);
      begin
        case l.loc of
          LOC_REGISTER,
          LOC_CREGISTER :
            a_load128_reg_cgpara(list,l.register128,paraloc);
          {not yet implemented:
          LOC_CONSTANT :
            a_load128_const_cgpara(list,l.value64,paraloc);
          }
          LOC_CREFERENCE,
          LOC_REFERENCE :
            a_load128_ref_cgpara(list,l.reference,paraloc);
          else
            internalerror(2012090603);
        end;
      end;


    procedure tcg128.a_load128_reg_cgpara(list : TAsmList;reg : tregister128;const paraloc : tcgpara);
      var
        tmplochi,tmploclo: tcgpara;
      begin
        tmploclo.init;
        tmplochi.init;
        splitparaloc128(paraloc,tmploclo,tmplochi);
        cg.a_load_reg_cgpara(list,OS_64,reg.reghi,tmplochi);
        cg.a_load_reg_cgpara(list,OS_64,reg.reglo,tmploclo);
        tmploclo.done;
        tmplochi.done;
      end;


    procedure tcg128.a_load128_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara);
      var
        tmprefhi,tmpreflo : treference;
        tmploclo,tmplochi : tcgpara;
      begin
        tmploclo.init;
        tmplochi.init;
        splitparaloc128(paraloc,tmploclo,tmplochi);
        tmprefhi:=r;
        tmpreflo:=r;
        if target_info.endian=endian_big then
          inc(tmpreflo.offset,8)
        else
          inc(tmprefhi.offset,8);
        cg.a_load_ref_cgpara(list,OS_64,tmprefhi,tmplochi);
        cg.a_load_ref_cgpara(list,OS_64,tmpreflo,tmploclo);
        tmploclo.done;
        tmplochi.done;
      end;
{$endif cpu64bitalu}

    function asmsym2indsymflags(sym: TAsmSymbol): tindsymflags;
      begin
        result:=[];
        if sym.typ<>AT_FUNCTION then
          include(result,is_data);
        if sym.bind=AB_WEAK_EXTERNAL then
          include(result,is_weak);
      end;

    procedure destroy_codegen;
      begin
        cg.free;
        cg:=nil;
{$ifdef cpu64bitalu}
        cg128.free;
        cg128:=nil;
{$else cpu64bitalu}
        cg64.free;
        cg64:=nil;
{$endif cpu64bitalu}
      end;

end.
