{
    Copyright (c) 1998-2010 by Florian Klaempfl and Jonas Maebe
    Member of the Free Pascal development team

    This unit implements the WebAssembly high level code generator

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
unit hlcgcpu;

{$i fpcdefs.inc}

interface

uses
  globtype,
  aasmbase,aasmdata,
  symbase,symconst,symtype,symdef,symsym,
  node,
  cpubase, hlcgobj, cgbase, cgutils, parabase, wasmdef;

  type

    { thlcgwasm }

    thlcgwasm = class(thlcgobj)
     private
      fevalstackheight,
      fmaxevalstackheight: longint;
     public
      br_blocks: integer;
      loopContBr: integer; // the value is different depending of the condition test
                           // if it's in the beggning the jump should be done to the loop (1)
                           // if the condition at the end, the jump should done to the end of block (0)
      loopBreakBr: integer;
      exitBr: integer;
      fntypelookup : TWasmProcTypeLookup;

      constructor create;
      destructor Destroy; override;

      procedure incblock;
      procedure decblock;

      procedure incstack(list : TAsmList;slots: longint);
      procedure decstack(list : TAsmList;slots: longint);

      procedure a_load_const_cgpara(list : TAsmList;tosize : tdef;a : tcgint;const cgpara : TCGPara);override;

      function a_call_name(list : TAsmList;pd : tprocdef;const s : TSymStr; const paras: array of pcgpara; forceresdef: tdef; weak: boolean): tcgpara;override;
      function a_call_reg(list: TAsmList; pd: tabstractprocdef; reg: tregister; const paras: array of pcgpara): tcgpara; override;

      { move instructions - a_load_FROM_TO }
      procedure a_load_const_reg(list : TAsmList;tosize : tdef;a : tcgint;register : tregister);override;
      procedure a_load_const_ref(list : TAsmList;tosize : tdef;a : tcgint;const ref : treference);override;
      procedure a_load_reg_ref(list : TAsmList;fromsize, tosize : tdef;register : tregister;const ref : treference);override;
      procedure a_load_reg_reg(list : TAsmList;fromsize, tosize : tdef;reg1,reg2 : tregister);override;
      procedure a_load_ref_reg(list : TAsmList;fromsize, tosize : tdef;const ref : treference;register : tregister);override;
      procedure a_load_ref_ref(list : TAsmList;fromsize, tosize : tdef;const sref : treference;const dref : treference);override;
      procedure a_loadaddr_ref_reg(list : TAsmList;fromsize, tosize : tdef;const ref : treference;r : tregister);override;

      { basic arithmetic operations }
      procedure a_op_const_reg(list: TAsmList; Op: TOpCG; size: tdef; a: tcgint; reg: TRegister); override;
      procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister); override;
      procedure a_op_const_ref(list: TAsmList; Op: TOpCG; size: tdef; a: tcgint; const ref: TReference); override;

      procedure a_op_ref_reg(list: TAsmList; Op: TOpCG; size: tdef; const ref: TReference; reg: TRegister); override;
      procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister); override;
      procedure a_op_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; reg1, reg2: TRegister); override;
      procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation); override;
      procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation); override;

      procedure a_cmp_const_ref_label(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const ref: treference; l: tasmlabel); override;
      procedure a_cmp_const_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel); override;
      procedure a_cmp_ref_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; reg: tregister; l: tasmlabel); override;
      procedure a_cmp_reg_ref_label(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const ref: treference; l: tasmlabel); override;
      procedure a_cmp_reg_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel); override;

      procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;

      procedure a_loadfpu_ref_ref(list: TAsmList; fromsize, tosize: tdef; const ref1, ref2: treference); override;
      procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister); override;
      procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference); override;
      procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister); override;

      procedure g_concatcopy(list : TAsmList;size: tdef; const source,dest : treference); override;

      procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean); override;
      procedure g_proc_exit(list : TAsmList;parasize:longint;nostackframe:boolean); override;

      procedure g_rangecheck(list: TAsmList; const l:tlocation; fromdef,todef: tdef); override;

      procedure g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef); override;
      procedure g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;var ovloc : tlocation); override;

      procedure maybe_change_load_node_reg(list: TAsmList; var n: tnode; reload: boolean); override;

      procedure gen_entry_code(list: TAsmList); override;
      procedure gen_exit_code(list: TAsmList); override;

      { unimplemented/unnecessary routines }
      procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tdef; src, dst: tregister); override;
      procedure a_loadmm_loc_reg(list: TAsmList; fromsize, tosize: tdef; const loc: tlocation; const reg: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference; shuffle: pmmshuffle); override;
      procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; src, dst: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tdef; intreg, mmreg: tregister; shuffle: pmmshuffle); override;
      procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tdef; mmreg, intreg: tregister; shuffle: pmmshuffle); override;
      procedure g_stackpointer_alloc(list: TAsmList; size: longint); override;
      procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint); override;
      procedure g_adjust_self_value(list: TAsmList; procdef: tprocdef; ioffset: aint); override;
      procedure g_local_unwind(list: TAsmList; l: TAsmLabel); override;

      { Wasm-specific routines }

      procedure g_procdef(list:TAsmList;pd: tprocdef);

      procedure a_load_stack_reg(list : TAsmList;size: tdef;reg: tregister);
      { extra_slots are the slots that are used by the reference, and that
        will be removed by the store operation }
      procedure a_load_stack_ref(list : TAsmList;size: tdef;const ref: treference;extra_slots: longint);
      procedure a_load_reg_stack(list : TAsmList;size: tdef;reg: tregister);
      { extra_slots are the slots that are used by the reference, and that
        will be removed by the load operation }
      procedure a_load_ref_stack(list : TAsmList;size: tdef;const ref: treference;extra_slots: longint);
      procedure a_load_const_stack(list : TAsmList;size: tdef;a :tcgint; typ: TRegisterType);
      procedure a_loadaddr_ref_stack(list : TAsmList;fromsize, tosize : tdef;const ref : treference);

      procedure a_load_stack_loc(list : TAsmList;size: tdef;const loc: tlocation);
      procedure a_load_loc_stack(list : TAsmList;size: tdef;const loc: tlocation);

      procedure a_loadfpu_const_stack(list : TAsmList;size: tdef;a :double);

      procedure a_op_stack(list : TAsmList;op: topcg; size: tdef);
      procedure a_op_const_stack(list : TAsmList;op: topcg; size: tdef;a : tcgint);
      procedure a_op_reg_stack(list : TAsmList;op: topcg; size: tdef;reg: tregister);
      procedure a_op_ref_stack(list : TAsmList;op: topcg; size: tdef;const ref: treference);
      procedure a_op_loc_stack(list : TAsmList;op: topcg; size: tdef;const loc: tlocation);

      procedure a_cmp_const_loc_stack(list: TAsmList; size: tdef;cmp_op: topcmp; a: tcgint; const loc: tlocation);
      procedure a_cmp_const_ref_stack(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const ref: treference);
      procedure a_cmp_const_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister);
      procedure a_cmp_ref_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; reg: tregister);
      procedure a_cmp_reg_ref_stack(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const ref: treference);
      procedure a_cmp_reg_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister);
      procedure a_cmp_subsetreg_reg_stack(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sreg: tsubsetregister; reg: tregister);
      procedure a_cmp_subsetref_reg_stack(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sref: tsubsetreference; reg: tregister);

      procedure a_cmp_loc_reg_stack(list : TAsmList;size : tdef;cmp_op : topcmp; const loc: tlocation; reg : tregister);
      procedure a_cmp_reg_loc_stack(list : TAsmList;size : tdef;cmp_op : topcmp; reg: tregister; const loc: tlocation);
      procedure a_cmp_ref_loc_stack(list: TAsmList; size: tdef;cmp_op: topcmp; const ref: treference; const loc: tlocation);

      procedure a_cmp_const_loc_br(list: TAsmList; size: tdef;cmp_op: topcmp; a: tcgint; const loc: tlocation; br: Integer);
      procedure a_cmp_const_ref_br(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const ref: treference; br: Integer);
      procedure a_cmp_const_reg_br(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister; br: Integer);
      procedure a_cmp_ref_reg_br(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; reg: tregister; br: Integer);
      procedure a_cmp_reg_ref_br(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const ref: treference; br: Integer);
      procedure a_cmp_reg_reg_br(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister; br: Integer);
      procedure a_cmp_subsetreg_reg_br(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sreg: tsubsetregister; reg: tregister; br: Integer);
      procedure a_cmp_subsetref_reg_br(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sref: tsubsetreference; reg: tregister; br: Integer);

      procedure a_cmp_loc_reg_br(list : TAsmList;size : tdef;cmp_op : topcmp; const loc: tlocation; reg : tregister; br: Integer);
      procedure a_cmp_reg_loc_br(list : TAsmList;size : tdef;cmp_op : topcmp; reg: tregister; const loc: tlocation; br: Integer);
      procedure a_cmp_ref_loc_br(list: TAsmList; size: tdef;cmp_op: topcmp; const ref: treference; const loc: tlocation; br: Integer);

      procedure g_reference_loc(list: TAsmList; def: tdef; const fromloc: tlocation; out toloc: tlocation); override;

      procedure a_cmp_stack_stack(list : TAsmlist; size: tdef; cmp_op: topcmp);
      { truncate/sign extend after performing operations on values < 32 bit
        that may have overflowed outside the range }
      procedure maybe_adjust_op_result(list: TAsmList; op: TOpCg; size: tdef);

      { performs sign/zero extension as required }
      procedure resize_stack_int_val(list: TAsmList;fromsize,tosize: tdef; formemstore: boolean);

      { 8/16 bit unsigned parameters and return values must be sign-extended on
        the producer side, because the JVM does not support unsigned variants;
        then they have to be zero-extended again on the consumer side }
      procedure maybe_resize_stack_para_val(list: TAsmList; retdef: tdef; callside: boolean);

      { adjust the stack height after a call based on the specified number of
        slots used for parameters and the provided resultdef }
      procedure g_adjust_stack_after_call(list: TAsmList; pd: tabstractprocdef);

      { because WebAssembly has no spec for any sort of debug info, and the
        only linker that we support (LLVM's wasm-ld) does not support creating
        map files in its stable version, and crashes when attempting to create
        a map file in its development version from git, we have no way to
        identify which procedure a crash occurred in. So, to identify the
        procedure, we call this procedure on proc entry, which generates a few
        useless loads of random numbers on the stack, that are immediately
        discarded, so they are essentially equivalent to a nop. This allows
        finding the procedure in the FPC output assembly, produced with -al by
        searching for these random numbers, as taken from the disassembly of the
        final binary. }
      procedure g_fingerprint(list: TAsmList);

      property maxevalstackheight: longint read fmaxevalstackheight;

     protected
      procedure gen_load_uninitialized_function_result(list: TAsmList; pd: tprocdef; resdef: tdef; const resloc: tcgpara); override;

      function g_call_system_proc_intern(list: TAsmList; pd: tprocdef; const paras: array of pcgpara; forceresdef: tdef): tcgpara; override;

     public
      { in case of an array, the array base address and index have to be
        put on the evaluation stack before the stored value; similarly, for
        fields the self pointer has to be loaded first. Also checks whether
        the reference is valid. If dup is true, the necessary values are stored
        twice. Returns how many stack slots have been consumed, disregarding
        the "dup". }
      function prepare_stack_for_ref(list: TAsmList; var ref: treference; dup: boolean): longint;
     protected
      { return the load/store opcode to load/store from/to ref; if the result
        has to be and'ed after a load to get the final value, that constant
        is returned in finishandval (otherwise that value is set to -1) }
      function loadstoreopcref(def: tdef; isload: boolean; const ref: treference; out finishandval: tcgint): tasmop;
      procedure resizestackfpuval(list: TAsmList; fromsize, tosize: tcgsize);
    end;

implementation

  uses
    verbose,cutils,globals,fmodule,constexp,
    defutil,
    aasmtai,aasmcpu,
    symtable,symcpu,
    procinfo,cpuinfo,cgcpu,tgobj,tgcpu,paramgr;

  const
    TOpCG2IAsmOp : array[topcg] of TAsmOp=(
      A_None,      {OP_NONE}
      A_None,      {OP_MOVE, replaced operation with direct load }
      a_i32_add,   {OP_ADD,  simple addition          }
      a_i32_and,   {OP_AND,  simple logical and       }
      a_i32_div_u, {OP_DIV,  simple unsigned division }
      a_i32_div_s, {OP_IDIV, simple signed division   }
      a_i32_mul,   {OP_IMUL, simple signed multiply   }
      a_i32_mul,   {OP_MUL,  simple unsigned multiply }
      A_None,      {OP_NEG,  simple negate            } // neg = xor + 1
      A_None,      {OP_NOT,  simple logical not       } // not = xor - 1
      a_i32_or,    {OP_OR,   simple logical or        }
      a_i32_shr_s, {OP_SAR,  arithmetic shift-right   }
      a_i32_shl,   {OP_SHL,  logical shift left       }
      a_i32_shr_u, {OP_SHR,  logical shift right      }
      a_i32_sub,   {OP_SUB,  simple subtraction       }
      a_i32_xor,   {OP_XOR,  simple exclusive or      }
      a_i32_rotl,  {OP_ROL,  rotate left              }
      a_i32_rotr   {OP_ROR   rotate right             }
    );
    TOpCG2LAsmOp : array[topcg] of TAsmOp=(
      A_None,      {OP_NONE}
      a_i64_load,  {OP_MOVE, replaced operation with direct load }
      a_i64_add,   {OP_ADD,  simple addition          }
      a_i64_and,   {OP_AND,  simple logical and       }
      a_i64_div_u, {OP_DIV,  simple unsigned division }
      a_i64_div_s, {OP_IDIV, simple signed division   }
      a_i64_mul,   {OP_IMUL, simple signed multiply   }
      a_i64_mul,   {OP_MUL,  simple unsigned multiply }
      A_None,      {OP_NEG,  simple negate            } // neg = xor + 1
      A_None,      {OP_NOT,  simple logical not       } // not = xor - 1
      a_i64_or,    {OP_OR,   simple logical or        }
      a_i64_shr_s, {OP_SAR,  arithmetic shift-right   }
      a_i64_shl,   {OP_SHL,  logical shift left       }
      a_i64_shr_u, {OP_SHR,  logical shift right      }
      a_i64_sub,   {OP_SUB,  simple subtraction       }
      a_i64_xor,   {OP_XOR,  simple exclusive or      }
      a_i64_rotl,  {OP_ROL,  rotate left              }
      a_i64_rotr   {OP_ROR   rotate right             }
    );

  constructor thlcgwasm.create;
    begin
      fevalstackheight:=0;
      fmaxevalstackheight:=0;
      fntypelookup:=TWasmProcTypeLookup.Create;
    end;

  destructor thlcgwasm.Destroy;
    begin
      fntypelookup.Free;
      inherited Destroy;
    end;

  procedure thlcgwasm.incblock;
    begin
      inc(br_blocks);
    end;

  procedure thlcgwasm.decblock;
    begin
      dec(br_blocks);
      if br_blocks<0 then
        Internalerror(2019091807); // out of block
    end;

  procedure thlcgwasm.incstack(list: TAsmList; slots: longint);
    begin
      if slots=0 then
        exit;
      inc(fevalstackheight,slots);
      if (fevalstackheight>fmaxevalstackheight) then
        fmaxevalstackheight:=fevalstackheight;
      if cs_asm_regalloc in current_settings.globalswitches then
        list.concat(tai_comment.Create(strpnew('    allocated '+tostr(slots)+', stack height = '+tostr(fevalstackheight))));
    end;

  procedure thlcgwasm.decstack(list: TAsmList;slots: longint);
    begin
      if slots=0 then
        exit;
      dec(fevalstackheight,slots);
      if (fevalstackheight<0) and
         not(cs_no_regalloc in current_settings.globalswitches) then
        internalerror(2010120501);
      if cs_asm_regalloc in current_settings.globalswitches then
        list.concat(tai_comment.Create(strpnew('    freed '+tostr(slots)+', stack height = '+tostr(fevalstackheight))));
    end;

  procedure thlcgwasm.a_load_const_cgpara(list: TAsmList; tosize: tdef; a: tcgint; const cgpara: TCGPara);
    begin
      tosize:=get_para_push_size(tosize);
      if tosize=s8inttype then
        a:=shortint(a)
      else if tosize=s16inttype then
        a:=smallint(a);
      inherited a_load_const_cgpara(list, tosize, a, cgpara);
    end;

  function thlcgwasm.a_call_name(list: TAsmList; pd: tprocdef; const s: TSymStr; const paras: array of pcgpara; forceresdef: tdef; weak: boolean): tcgpara;
    begin
      list.concat(taicpu.op_sym(a_call,current_asmdata.RefAsmSymbol(s,AT_FUNCTION)));
      result:=get_call_result_cgpara(pd,forceresdef);
    end;


  function thlcgwasm.a_call_reg(list: TAsmList; pd: tabstractprocdef; reg: tregister; const paras: array of pcgpara): tcgpara;
    begin
      a_load_reg_stack(list, ptrsinttype, reg);
      if pd.typ=procvardef then
        current_asmdata.CurrAsmList.Concat(taicpu.op_functype(a_call_indirect,tcpuprocvardef(pd).create_functype))
      else
        current_asmdata.CurrAsmList.Concat(taicpu.op_functype(a_call_indirect,tcpuprocdef(pd).create_functype));
      decstack(list,1);
      result:=hlcg.get_call_result_cgpara(pd, nil);
    end;


  procedure thlcgwasm.a_load_const_stack(list : TAsmList;size : tdef;a : tcgint; typ: TRegisterType);
    begin
      case typ of
        R_INTREGISTER,
        R_ADDRESSREGISTER:
          begin
            case def_cgsize(size) of
              OS_8,OS_16,OS_32,
              OS_S8,OS_S16,OS_S32:
                begin
                  { convert cardinals to longints }
                  list.concat(taicpu.op_const(a_i32_const, a));
                end;
              OS_64,OS_S64:
                begin
                  list.concat(taicpu.op_const(a_i64_const, a));
                end;
              else
                internalerror(2010110702);
            end;
          end;
        else
          internalerror(2010110703);
      end;
      incstack(list,1);
    end;

  procedure thlcgwasm.a_loadaddr_ref_stack(list : TAsmList;fromsize, tosize : tdef;const ref : treference);
    var
      tmpref: treference;
    begin
      { you can't take the address of references, that are on the local stack }
      if (ref.base=NR_EVAL_STACK_BASE) or (ref.index=NR_EVAL_STACK_BASE) or
         (ref.base=NR_LOCAL_STACK_POINTER_REG) or (ref.index=NR_LOCAL_STACK_POINTER_REG) then
        internalerror(2021010101);

      tmpref:=ref;
      tmpref.base:=NR_NO;
      tmpref.index:=NR_NO;
      list.Concat(taicpu.op_ref(a_i32_const, tmpref));
      incstack(list, 1);
      if ref.base<>NR_NO then
        begin
          list.Concat(taicpu.op_reg(a_local_get,ref.base));
          incstack(list, 1);
          list.Concat(taicpu.op_none(a_i32_add));
          decstack(list, 1);
        end;
      if ref.index<>NR_NO then
        begin
          list.Concat(taicpu.op_reg(a_local_get,ref.index));
          incstack(list, 1);
          if ref.scalefactor>1 then
            begin
              list.Concat(taicpu.op_const(a_i32_const,ref.scalefactor));
              incstack(list, 1);
              list.Concat(taicpu.op_none(a_i32_mul));
              decstack(list, 1);
            end;
          list.Concat(taicpu.op_none(a_i32_add));
          decstack(list, 1);
        end;
    end;

  procedure thlcgwasm.a_load_stack_loc(list: TAsmList; size: tdef; const loc: tlocation);
    var
      tmpref: treference;
    begin
      case loc.loc of
        LOC_REGISTER,LOC_CREGISTER,
        LOC_FPUREGISTER,LOC_CFPUREGISTER:
          a_load_stack_reg(list,size,loc.register);
        LOC_REFERENCE:
          begin
            tmpref:=loc.reference;
            a_load_stack_ref(list,size,loc.reference,prepare_stack_for_ref(list,tmpref,false));
          end;
        else
          internalerror(2011020501);
      end;
    end;

  procedure thlcgwasm.a_load_loc_stack(list: TAsmList;size: tdef;const loc: tlocation);
    var
      tmpref: treference;
      extra_slots: LongInt;
    begin
      case loc.loc of
        LOC_REGISTER,LOC_CREGISTER,
        LOC_FPUREGISTER,LOC_CFPUREGISTER:
          a_load_reg_stack(list,size,loc.register);
        LOC_REFERENCE,LOC_CREFERENCE:
          begin
            tmpref:=loc.reference;
            extra_slots:=prepare_stack_for_ref(list,tmpref,false);
            a_load_ref_stack(list,size,tmpref,extra_slots);
          end;
        LOC_CONSTANT:
          a_load_const_stack(list,size,loc.value,def2regtyp(size));
        else
          internalerror(2011010401);
      end;
    end;

  procedure thlcgwasm.a_loadfpu_const_stack(list: TAsmList; size: tdef; a: double);
    begin
      case tfloatdef(size).floattype of
        s32real:
          begin
            list.concat(taicpu.op_single(a_f32_const, a));
            incstack(list,1);
          end;
        s64real:
          begin
            list.concat(taicpu.op_double(a_f64_const,a));
            incstack(list,1);
          end
        else
          internalerror(2011010501);
      end;
    end;

  procedure thlcgwasm.a_op_stack(list: TAsmList; op: topcg; size: tdef);
    begin
      case def_cgsize(size) of
        OS_8,OS_S8,
        OS_16,OS_S16,
        OS_32,OS_S32:
          begin
            { boolean not: =0? for boolean }
            { todo: should we also do this for cbool? }
            if (op=OP_NOT) and is_pasbool(size) then
              list.concat(taicpu.op_none(a_i32_eqz))
            else
              begin
                if op=OP_NOT then
                  begin
                    { not = xor -1 for integer }
                    a_load_const_stack(list,s32inttype,high(cardinal),R_INTREGISTER);
                    op:=OP_XOR;
                  end
                else if op=OP_NEG then
                  begin
                    { neg = *(-1) }
                    a_load_const_stack(list,s32inttype,-1,R_INTREGISTER);
                    op:=OP_MUL;
                  end;
                if TOpCG2IAsmOp[op]=A_None then
                  internalerror(2010120532);
                list.concat(taicpu.op_none(TOpCG2IAsmOp[op]));
                decstack(list,1);
              end;
            maybe_adjust_op_result(list,op,size);
          end;
        OS_64,OS_S64:
          begin
            { unsigned 64 bit division must be done via a helper }
            if op=OP_DIV then
              internalerror(2010120530);
            { boolean not: =0? for boolean }
            { todo: should we also do this for cbool? }
            if (op=OP_NOT) and is_pasbool(size) then
              list.concat(taicpu.op_none(a_i64_eqz))
            else
              begin
                if op=OP_NOT then
                  begin
                    { not = xor -1 for integer }
                    a_load_const_stack(list,s64inttype,-1,R_INTREGISTER);
                    op:=OP_XOR;
                  end
                else if op=OP_NEG then
                  begin
                    { neg = *(-1) }
                    a_load_const_stack(list,s64inttype,-1,R_INTREGISTER);
                    op:=OP_MUL;
                  end;
                if TOpCG2LAsmOp[op]=A_None then
                  internalerror(2010120533);
                list.concat(taicpu.op_none(TOpCG2LAsmOp[op]));
                decstack(list,1);
              end;
          end;
        else
          internalerror(2010120531);
      end;
    end;

  procedure thlcgwasm.a_op_const_stack(list: TAsmList;op: topcg;size: tdef;a: tcgint);
    begin
      case op of
        OP_NEG,OP_NOT:
          internalerror(2011010801);
        else
          a_load_const_stack(list,size,a,R_INTREGISTER);
      end;
      a_op_stack(list,op,size);
    end;

  procedure thlcgwasm.a_op_reg_stack(list: TAsmList; op: topcg; size: tdef; reg: tregister);
    begin
      a_load_reg_stack(list,size,reg);
      a_op_stack(list,op,size);
    end;

  procedure thlcgwasm.a_op_ref_stack(list: TAsmList; op: topcg; size: tdef; const ref: treference);
    var
      tmpref: treference;
    begin
      { ref must not be the stack top, because that may indicate an error
        (it means that we will perform an operation of the stack top onto
         itself, so that means the two values have been loaded manually prior
         to calling this routine, instead of letting this routine load one of
         them; if something like that is needed, call a_op_stack() directly) }
      if ref.base=NR_EVAL_STACK_BASE then
        internalerror(2010121102);
      tmpref:=ref;
      a_load_ref_stack(list,size,tmpref,prepare_stack_for_ref(list,tmpref,false));
      a_op_stack(list,op,size);
    end;

  procedure thlcgwasm.a_op_loc_stack(list: TAsmList; op: topcg; size: tdef; const loc: tlocation);
    begin
      case loc.loc of
        LOC_REGISTER,LOC_CREGISTER:
          a_op_reg_stack(list,op,size,loc.register);
        LOC_REFERENCE,LOC_CREFERENCE:
          a_op_ref_stack(list,op,size,loc.reference);
        LOC_CONSTANT:
          a_op_const_stack(list,op,size,loc.value);
        else
          internalerror(2011011415)
      end;
    end;

  procedure thlcgwasm.a_cmp_const_loc_stack(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const loc: tlocation);
    var
      tmpreg: tregister;
    begin
      case loc.loc of
        LOC_REGISTER,LOC_CREGISTER:
          a_cmp_const_reg_stack(list,size,cmp_op,a,loc.register);
        LOC_REFERENCE,LOC_CREFERENCE:
          a_cmp_const_ref_stack(list,size,cmp_op,a,loc.reference);
        LOC_SUBSETREG, LOC_CSUBSETREG:
          begin
            tmpreg:=getintregister(list,size);
            a_load_subsetreg_reg(list,size,size,loc.sreg,tmpreg);
            a_cmp_const_reg_stack(list,size,cmp_op,a,tmpreg);
          end;
        LOC_SUBSETREF, LOC_CSUBSETREF:
          begin
            tmpreg:=getintregister(list,size);
            a_load_subsetref_reg(list,size,size,loc.sref,tmpreg);
            a_cmp_const_reg_stack(list,size,cmp_op,a,tmpreg);
          end;
        else
          internalerror(2010120430);
      end;
    end;

  procedure thlcgwasm.a_cmp_const_ref_stack(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const ref: treference);
    var
      tmpref: treference;
    begin
      tmpref:=ref;
      if tmpref.base<>NR_EVAL_STACK_BASE then
        a_load_ref_stack(list,size,tmpref,prepare_stack_for_ref(list,tmpref,false));
      a_load_const_stack(list,size,a,def2regtyp(size));
      a_cmp_stack_stack(list,size,cmp_op);
    end;

  procedure thlcgwasm.a_cmp_const_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister);
    begin
      a_load_reg_stack(list,size,reg);
      a_load_const_stack(list,size,a,def2regtyp(size));
      a_cmp_stack_stack(list,size,cmp_op);
    end;

  procedure thlcgwasm.a_cmp_ref_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; reg: tregister);
    var
      tmpref: treference;
    begin
      tmpref:=ref;
      a_load_reg_stack(list,size,reg);
      if tmpref.base<>NR_EVAL_STACK_BASE then
        a_load_ref_stack(list,size,tmpref,prepare_stack_for_ref(list,tmpref,false))
      else
        cmp_op:=swap_opcmp(cmp_op);
      a_cmp_stack_stack(list,size,cmp_op);
    end;

  procedure thlcgwasm.a_cmp_reg_ref_stack(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const ref: treference);
    var
      tmpref: treference;
    begin
      tmpref:=ref;
      if tmpref.base<>NR_EVAL_STACK_BASE then
        a_load_ref_stack(list,size,ref,prepare_stack_for_ref(list,tmpref,false));
      a_load_reg_stack(list,size,reg);
      a_cmp_stack_stack(list,size,cmp_op);
    end;

  procedure thlcgwasm.a_cmp_reg_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister);
    begin
      a_load_reg_stack(list,size,reg2);
      a_load_reg_stack(list,size,reg1);
      a_cmp_stack_stack(list,size,cmp_op);
    end;

  procedure thlcgwasm.a_cmp_subsetreg_reg_stack(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sreg: tsubsetregister; reg: tregister);
    var
      tmpreg: tregister;
    begin
      tmpreg:=getintregister(list,cmpsize);
      a_load_subsetreg_reg(list,fromsubsetsize,cmpsize,sreg,tmpreg);
      a_cmp_reg_reg_stack(list,cmpsize,cmp_op,tmpreg,reg);
    end;

  procedure thlcgwasm.a_cmp_subsetref_reg_stack(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sref: tsubsetreference; reg: tregister);
    var
      tmpreg: tregister;
    begin
      tmpreg:=getintregister(list,cmpsize);
      a_load_subsetref_reg(list,fromsubsetsize,cmpsize,sref,tmpreg);
      a_cmp_reg_reg_stack(list,cmpsize,cmp_op,tmpreg,reg);
    end;

  procedure thlcgwasm.a_cmp_loc_reg_stack(list: TAsmList; size: tdef; cmp_op: topcmp; const loc: tlocation; reg: tregister);
    begin
      case loc.loc of
        LOC_REGISTER,
        LOC_CREGISTER:
          a_cmp_reg_reg_stack(list,size,cmp_op,loc.register,reg);
        LOC_REFERENCE,
        LOC_CREFERENCE :
          a_cmp_ref_reg_stack(list,size,cmp_op,loc.reference,reg);
        LOC_CONSTANT:
          a_cmp_const_reg_stack(list,size,cmp_op,loc.value,reg);
        LOC_SUBSETREG,
        LOC_CSUBSETREG:
          a_cmp_subsetreg_reg_stack(list,size,size,cmp_op,loc.sreg,reg);
        LOC_SUBSETREF,
        LOC_CSUBSETREF:
          a_cmp_subsetref_reg_stack(list,size,size,cmp_op,loc.sref,reg);
        else
          internalerror(2010120431);
      end;
    end;

  procedure thlcgwasm.a_cmp_reg_loc_stack(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const loc: tlocation);
    begin
      a_cmp_loc_reg_stack(list,size,swap_opcmp(cmp_op),loc,reg);
    end;

  procedure thlcgwasm.a_cmp_ref_loc_stack(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; const loc: tlocation);
    var
      tmpreg: tregister;
    begin
      case loc.loc of
        LOC_REGISTER,LOC_CREGISTER:
          a_cmp_ref_reg_stack(list,size,cmp_op,ref,loc.register);
        LOC_REFERENCE,LOC_CREFERENCE:
          begin
            tmpreg:=getintregister(list,size);
            a_load_ref_reg(list,size,size,loc.reference,tmpreg);
            a_cmp_ref_reg_stack(list,size,cmp_op,ref,tmpreg);
          end;
        LOC_CONSTANT:
          begin
            a_cmp_const_ref_stack(list,size,swap_opcmp(cmp_op),loc.value,ref);
          end;
        LOC_SUBSETREG, LOC_CSUBSETREG:
          begin
            tmpreg:=getintregister(list,size);
            a_load_ref_reg(list,size,size,loc.reference,tmpreg);
            a_cmp_subsetreg_reg_stack(list,size,size,swap_opcmp(cmp_op),loc.sreg,tmpreg);
          end;
        LOC_SUBSETREF, LOC_CSUBSETREF:
          begin
            tmpreg:=getintregister(list,size);
            a_load_ref_reg(list,size,size,loc.reference,tmpreg);
            a_cmp_subsetref_reg_stack(list,size,size,swap_opcmp(cmp_op),loc.sref,tmpreg);
          end;
        else
          internalerror(2010120432);
      end;
    end;

  procedure thlcgwasm.a_cmp_const_loc_br(list: TAsmList; size: tdef;cmp_op: topcmp; a: tcgint; const loc: tlocation; br: Integer);
    begin
      a_cmp_const_loc_stack(list,size,cmp_op,a,loc);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_const_ref_br(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const ref: treference; br: Integer);
    begin
      a_cmp_const_ref_stack(list,size,cmp_op,a,ref);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_const_reg_br(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister; br: Integer);
    begin
      a_cmp_const_reg_stack(list,size,cmp_op,a,reg);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_ref_reg_br(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; reg: tregister; br: Integer);
    begin
      a_cmp_ref_reg_stack(list,size,cmp_op,ref,reg);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_reg_ref_br(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const ref: treference; br: Integer);
    begin
      a_cmp_reg_ref_stack(list,size,cmp_op,reg,ref);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_reg_reg_br(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister; br: Integer);
    begin
      a_cmp_reg_reg_stack(list,size,cmp_op,reg1,reg2);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_subsetreg_reg_br(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sreg: tsubsetregister; reg: tregister; br: Integer);
    begin
      a_cmp_subsetreg_reg_stack(list,fromsubsetsize,cmpsize,cmp_op,sreg,reg);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_subsetref_reg_br(list: TAsmList; fromsubsetsize, cmpsize: tdef; cmp_op: topcmp; const sref: tsubsetreference; reg: tregister; br: Integer);
    begin
      a_cmp_subsetref_reg_stack(list,fromsubsetsize,cmpsize,cmp_op,sref,reg);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_loc_reg_br(list : TAsmList;size : tdef;cmp_op : topcmp; const loc: tlocation; reg : tregister; br: Integer);
    begin
      a_cmp_loc_reg_stack(list,size,cmp_op,loc,reg);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_reg_loc_br(list : TAsmList;size : tdef;cmp_op : topcmp; reg: tregister; const loc: tlocation; br: Integer);
    begin
      a_cmp_reg_loc_stack(list,size,cmp_op,reg,loc);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.a_cmp_ref_loc_br(list: TAsmList; size: tdef;cmp_op: topcmp; const ref: treference; const loc: tlocation; br: Integer);
    begin
      a_cmp_ref_loc_stack(list,size,cmp_op,ref,loc);
      current_asmdata.CurrAsmList.concat(taicpu.op_const(a_br_if,br));
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
    end;

  procedure thlcgwasm.g_reference_loc(list: TAsmList; def: tdef; const fromloc: tlocation; out toloc: tlocation);
    begin
      case fromloc.loc of
        LOC_CREFERENCE,
        LOC_REFERENCE:
          begin
            toloc:=fromloc;
            if (fromloc.reference.base<>NR_NO) and
               (fromloc.reference.base<>current_procinfo.framepointer) and
               (fromloc.reference.base<>NR_STACK_POINTER_REG) then
              g_allocload_reg_reg(list,voidpointertype,fromloc.reference.base,toloc.reference.base,R_ADDRESSREGISTER);
          end;
        else
          inherited;
      end;
    end;

    procedure thlcgwasm.a_cmp_stack_stack(list: TAsmlist; size: tdef; cmp_op: topcmp);
      const
        opcmp32: array[topcmp] of tasmop = (
          A_None,     { OC_NONE, }
          a_i32_eq,   { OC_EQ,   equality comparison              }
          a_i32_gt_s, { OC_GT,   greater than (signed)            }
          a_i32_lt_s, { OC_LT,   less than (signed)               }
          a_i32_ge_s, { OC_GTE,  greater or equal than (signed)   }
          a_i32_le_s, { OC_LTE,  less or equal than (signed)      }
          a_i32_ne,   { OC_NE,   not equal                        }
          a_i32_le_u, { OC_BE,   less or equal than (unsigned)    }
          a_i32_lt_u, { OC_B,    less than (unsigned)             }
          a_i32_ge_u, { OC_AE,   greater or equal than (unsigned) }
          a_i32_gt_u  { OC_A     greater than (unsigned)          }
        );
      const
        opcmp64: array[TOpCmp] of TAsmOp = (A_None,
           a_i64_eq,               // OC_EQ
           a_i64_gt_s, a_i64_lt_s, // OC_GT, OC_LT
           a_i64_ge_s, a_i64_le_s, // OC_GTE, OC_LTE
           a_i64_ne,               // OC_NE
           a_i64_le_u, a_i64_lt_u, // OC_BE, OC_B
           a_i64_ge_u, a_i64_gt_u  // OC_AE, OC_A
        );

      var
        cgsize: tcgsize;
      begin
        case def2regtyp(size) of
          R_INTREGISTER,
          R_ADDRESSREGISTER:
            begin
              cgsize:=def_cgsize(size);
              case cgsize of
                OS_S8,OS_8,
                OS_16,OS_S16,
                OS_S32,OS_32:
                  begin
                    list.concat(taicpu.op_none(opcmp32[cmp_op]));
                    decstack(list,1);
                  end;
                OS_64,OS_S64:
                  begin
                    list.concat(taicpu.op_none(opcmp64[cmp_op]));
                    decstack(list,1);
                  end;
                else
                  internalerror(2010120538);
              end;
            end;
          else
            internalerror(2010120538);
        end;
      end;

    procedure thlcgwasm.maybe_adjust_op_result(list: TAsmList; op: TOpCg; size: tdef);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
      begin
        if (op in overflowops) and
           (def_cgsize(size) in [OS_8,OS_S8,OS_16,OS_S16]) then
          resize_stack_int_val(list,s32inttype,size,false);
      end;

  procedure thlcgwasm.gen_load_uninitialized_function_result(list: TAsmList; pd: tprocdef; resdef: tdef; const resloc: tcgpara);
    begin
      { nothing to do for ret_in_param results }
      if paramanager.ret_in_param(pd.returndef,pd) then
        exit;
      { constructors don't return anything in Java }
      if pd.proctypeoption=potype_constructor then
        exit;
      { must return a value of the correct type on the evaluation stack }
      case def2regtyp(resdef) of
        R_INTREGISTER,
        R_ADDRESSREGISTER:
          a_load_const_cgpara(list,resdef,0,resloc);
        R_FPUREGISTER:
          case tfloatdef(resdef).floattype of
            s32real:
              begin
                list.concat(taicpu.op_single(a_f32_const, 0));
                incstack(list,1);
              end;
            s64real:
              begin
                list.concat(taicpu.op_double(a_f64_const, 0));
                incstack(list,1);
              end;
            else
              internalerror(2011010302);
          end
        else
          internalerror(2011010301);
      end;
    end;


  function thlcgwasm.g_call_system_proc_intern(list: TAsmList; pd: tprocdef; const paras: array of pcgpara; forceresdef: tdef): tcgpara;
    begin
      result:=inherited;
      pd.init_paraloc_info(callerside);
      g_adjust_stack_after_call(list,pd);
    end;


  function thlcgwasm.prepare_stack_for_ref(list: TAsmList; var ref: treference; dup: boolean): longint;
    begin
      result:=0;
      { fake location that indicates the value is already on the stack? }
      if (ref.base=NR_EVAL_STACK_BASE) or (ref.base=NR_LOCAL_STACK_POINTER_REG) then
        exit;

      if (ref.base=NR_NO) and (ref.index<>NR_NO) and (ref.scalefactor<=1) then
        begin
          ref.base:=ref.index;
          ref.index:=NR_NO;
        end;

      // setting up memory offset
      if assigned(ref.symbol) and (ref.base=NR_NO) and (ref.index=NR_NO) then
        begin
          list.Concat(taicpu.op_const(a_i32_const,ref.offset));
          incstack(list,1);
          if dup then
            begin
              list.Concat(taicpu.op_const(a_i32_const,ref.offset));
              incstack(list,1);
            end;
          ref.offset:=0;
          result:=1;
        end
      else if ref.index <> NR_NO then // array access
        begin
          // it's just faster to sum two of those together
          list.Concat(taicpu.op_reg(a_local_get, ref.base));
          incstack(list,1);
          list.Concat(taicpu.op_reg(a_local_get, ref.index));
          incstack(list,1);
          list.Concat(taicpu.op_none(a_i32_add));
          decstack(list,1);
          if ref.offset<0 then
            begin
              list.Concat(taicpu.op_const(a_i32_const,-ref.offset));
              incstack(list,1);
              list.Concat(taicpu.op_none(a_i32_sub));
              decstack(list,1);
            end;
          if dup then
            begin
              list.Concat(taicpu.op_reg(a_local_get, ref.base));
              incstack(list,1);
              list.Concat(taicpu.op_reg(a_local_get, ref.index));
              incstack(list,1);
              list.Concat(taicpu.op_none(a_i32_add));
              decstack(list,1);
              if ref.offset<0 then
                begin
                  list.Concat(taicpu.op_const(a_i32_const,-ref.offset));
                  incstack(list,1);
                  list.Concat(taicpu.op_none(a_i32_sub));
                  decstack(list,1);
                end;
            end;
          ref.base:=NR_NO;
          ref.index:=NR_NO;
          if ref.offset<0 then
            ref.offset:=0;
          result:=1;
        end
      else if (ref.base<>NR_NO) then
        begin
          if (ref.base<>NR_STACK_POINTER_REG) then
            begin
              { regular field -> load self on the stack }
              a_load_reg_stack(list,voidpointertype,ref.base);
              if ref.offset<0 then
                begin
                  list.Concat(taicpu.op_const(a_i32_const,-ref.offset));
                  incstack(list,1);
                  list.Concat(taicpu.op_none(a_i32_sub));
                  decstack(list,1);
                end;
              if dup then
                begin
                  a_load_reg_stack(list,voidpointertype,ref.base);
                  if ref.offset<0 then
                    begin
                      list.Concat(taicpu.op_const(a_i32_const,-ref.offset));
                      incstack(list,1);
                      list.Concat(taicpu.op_none(a_i32_sub));
                      decstack(list,1);
                    end;
                end;
              if ref.offset<0 then
                ref.offset:=0;
              ref.base:=NR_NO;
              result:=1;
            end
          else // if (ref.base = NR_FRAME_POINTER_REG) then
            begin
              internalerror(2021012202);
              //list.Concat(taicpu.op_sym(a_local_get, current_asmdata.RefAsmSymbol(FRAME_POINTER_SYM,AT_ADDR) ));
              //incstack(list,1);
            end;
        end
      else
        begin
          { static field -> nothing to do here, except for validity check }
          {if not assigned(ref.symbol) or
             (ref.offset<>0) then
          begin
            internalerror(2010120525);
          end;}
        end;
    end;

  procedure thlcgwasm.a_load_const_reg(list: TAsmList; tosize: tdef; a: tcgint; register: tregister);
    begin
      a_load_const_stack(list,tosize,a,def2regtyp(tosize));
      a_load_stack_reg(list,tosize,register);
    end;

  procedure thlcgwasm.a_load_const_ref(list: TAsmList; tosize: tdef; a: tcgint; const ref: treference);
    var
      extra_slots: longint;
      tmpref: treference;
    begin
      tmpref:=ref;
      extra_slots:=prepare_stack_for_ref(list,tmpref,false);
      a_load_const_stack(list,tosize,a,def2regtyp(tosize));
      a_load_stack_ref(list,tosize,tmpref,extra_slots);
    end;

  procedure thlcgwasm.a_load_reg_ref(list: TAsmList; fromsize, tosize: tdef; register: tregister; const ref: treference);
    var
      extra_slots: longint;
      tmpref: treference;
    begin
      tmpref:=ref;
      extra_slots:=prepare_stack_for_ref(list,tmpref,false);
      a_load_reg_stack(list,fromsize,register);
      if def2regtyp(fromsize)=R_INTREGISTER then
        resize_stack_int_val(list,fromsize,tosize,assigned(tmpref.symbol));
      a_load_stack_ref(list,tosize,tmpref,extra_slots);
    end;

  procedure thlcgwasm.a_load_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister);
    begin
      a_load_reg_stack(list,fromsize,reg1);
      if def2regtyp(fromsize)=R_INTREGISTER then
        resize_stack_int_val(list,fromsize,tosize,false);
      a_load_stack_reg(list,tosize,reg2);
    end;

  procedure thlcgwasm.a_load_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; register: tregister);
    var
      extra_slots: longint;
      tmpref: treference;
    begin
      tmpref:=ref;
      extra_slots:=prepare_stack_for_ref(list,tmpref,false);
      a_load_ref_stack(list,fromsize,tmpref,extra_slots);

      if def2regtyp(fromsize)=R_INTREGISTER then
        resize_stack_int_val(list,fromsize,tosize,false);
      a_load_stack_reg(list,tosize,register);
    end;

  procedure thlcgwasm.a_load_ref_ref(list: TAsmList; fromsize, tosize: tdef; const sref: treference; const dref: treference);
    var
      extra_sslots,
      extra_dslots: longint;
      tmpsref, tmpdref: treference;
      tmpreg: tregister;
    begin
      if sref.base<>NR_EVAL_STACK_BASE then
        begin
          tmpsref:=sref;
          tmpdref:=dref;
          { make sure the destination reference is on top, since in the end the
            order has to be "destref, value" -> first create "destref, sourceref" }
          extra_dslots:=prepare_stack_for_ref(list,tmpdref,false);
          extra_sslots:=prepare_stack_for_ref(list,tmpsref,false);
          a_load_ref_stack(list,fromsize,tmpsref,extra_sslots);
          if def2regtyp(fromsize)=R_INTREGISTER then
            resize_stack_int_val(list,fromsize,tosize,assigned(tmpdref.symbol));
          a_load_stack_ref(list,tosize,tmpdref,extra_dslots);
        end
      else
        begin
          { verify if we have the same reference }
          if references_equal(sref,dref) then
            exit;
          tmpreg:=getregisterfordef(list,tosize);
          a_load_ref_reg(list,fromsize,tosize,sref,tmpreg);
          a_load_reg_ref(list,tosize,tosize,tmpreg,dref);
        end;
    end;

  procedure thlcgwasm.a_loadaddr_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; r: tregister);
    begin
      a_loadaddr_ref_stack(list,fromsize,tosize,ref);
      a_load_stack_reg(list, tosize, r);
    end;

  procedure thlcgwasm.a_op_const_reg(list: TAsmList; Op: TOpCG; size: tdef; a: tcgint; reg: TRegister);
    begin
      a_op_const_reg_reg(list,op,size,a,reg,reg);
    end;

  procedure thlcgwasm.a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister);
    begin
      a_load_reg_stack(list,size,src);
      a_op_const_stack(list,op,size,a);
      a_load_stack_reg(list,size,dst);
    end;

  procedure thlcgwasm.a_op_const_ref(list: TAsmList; Op: TOpCG; size: tdef; a: tcgint; const ref: TReference);
    var
      extra_slots: longint;
      tmpref: treference;
    begin
      tmpref:=ref;
      extra_slots:=prepare_stack_for_ref(list,tmpref,true);
      { TODO, here or in peepholeopt: use iinc when possible }
      a_load_ref_stack(list,size,tmpref,extra_slots);
      a_op_const_stack(list,op,size,a);
      { for android verifier }
      if (def2regtyp(size)=R_INTREGISTER) and
         (assigned(tmpref.symbol)) then
        resize_stack_int_val(list,size,size,true);
      a_load_stack_ref(list,size,tmpref,extra_slots);
    end;

  procedure thlcgwasm.a_op_ref_reg(list: TAsmList; Op: TOpCG; size: tdef; const ref: TReference; reg: TRegister);
    begin
      if not(op in [OP_NOT,OP_NEG]) then
        a_load_reg_stack(list,size,reg);
      a_op_ref_stack(list,op,size,ref);
      a_load_stack_reg(list,size,reg);
    end;

  procedure thlcgwasm.a_op_reg_reg_reg(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister);
    begin
      if not(op in [OP_NOT,OP_NEG]) then
        a_load_reg_stack(list,size,src2);
      a_op_reg_stack(list,op,size,src1);
      a_load_stack_reg(list,size,dst);
    end;

  procedure thlcgwasm.a_op_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; reg1, reg2: TRegister);
    begin
      a_op_reg_reg_reg(list,op,size,reg1,reg2,reg2);
    end;

  procedure thlcgwasm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; a: tcgint; src, dst: tregister; setflags: boolean; var ovloc: tlocation);
    var
      tmpreg: tregister;
    begin
      if not setflags then
        begin
          inherited;
          exit;
        end;
      tmpreg:=getintregister(list,size);
      a_load_const_reg(list,size,a,tmpreg);
      a_op_reg_reg_reg_checkoverflow(list,op,size,tmpreg,src,dst,true,ovloc);
    end;

  procedure thlcgwasm.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tdef; src1, src2, dst: tregister; setflags: boolean; var ovloc: tlocation);
    var
      orgsrc1, orgsrc2: tregister;
      docheck: boolean;
      lab: tasmlabel;
    begin
      if not setflags then
        begin
          inherited;
          exit;
        end;
      { anything else cannot overflow }
      docheck:=size.size in [4,8];
      if docheck then
        begin
          orgsrc1:=src1;
          orgsrc2:=src2;
          if src1=dst then
            begin
              orgsrc1:=getintregister(list,size);
              a_load_reg_reg(list,size,size,src1,orgsrc1);
            end;
          if src2=dst then
            begin
              orgsrc2:=getintregister(list,size);
              a_load_reg_reg(list,size,size,src2,orgsrc2);
            end;
        end;
      a_op_reg_reg_reg(list,op,size,src1,src2,dst);
      if docheck then
        begin
          { * signed overflow for addition iff
             - src1 and src2 are negative and result is positive (excep in case of
               subtraction, then sign of src1 has to be inverted)
             - src1 and src2 are positive and result is negative
              -> Simplified boolean equivalent (in terms of sign bits):
                 not(src1 xor src2) and (src1 xor dst)

             for subtraction, multiplication: invert src1 sign bit
             for division: handle separately (div by zero, low(inttype) div -1),
               not supported by this code

            * unsigned overflow iff carry out, aka dst < src1 or dst < src2
          }
          location_reset(ovloc,LOC_REGISTER,OS_S32);
          { not pasbool8, because then we'd still have to convert the integer to
            a boolean via branches for Dalvik}
          ovloc.register:=getintregister(list,s32inttype);
          if not ((size.typ=pointerdef) or
                 ((size.typ=orddef) and
                  (torddef(size).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                             pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]))) then
            begin
              a_load_reg_stack(list,size,src1);
              if op in [OP_SUB,OP_IMUL] then
                a_op_stack(list,OP_NOT,size);
              a_op_reg_stack(list,OP_XOR,size,src2);
              a_op_stack(list,OP_NOT,size);
              a_load_reg_stack(list,size,src1);
              a_op_reg_stack(list,OP_XOR,size,dst);
              a_op_stack(list,OP_AND,size);
              a_op_const_stack(list,OP_SHR,size,(size.size*8)-1);
              if size.size=8 then
                begin
                  //todo: any operands needed?
                  list.concat(taicpu.op_none(a_i32_wrap_i64));
                end;
            end
          else
            begin
              a_load_const_stack(list,s32inttype,0,R_INTREGISTER);
              current_asmdata.getjumplabel(lab);
              { can be optimized by removing duplicate xor'ing to convert dst from
                signed to unsigned quadrant }
              a_cmp_reg_reg_label(list,size,OC_B,dst,src1,lab);
              a_cmp_reg_reg_label(list,size,OC_B,dst,src2,lab);
              a_op_const_stack(list,OP_XOR,s32inttype,1);
              a_label(list,lab);
            end;
          a_load_stack_reg(list,s32inttype,ovloc.register);
        end
      else
        ovloc.loc:=LOC_VOID;
    end;

  procedure thlcgwasm.a_cmp_const_ref_label(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; const ref: treference; l: tasmlabel);
    begin
      internalerror(2021011802);
    end;

  procedure thlcgwasm.a_cmp_const_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; a: tcgint; reg: tregister; l: tasmlabel);
    begin
      internalerror(2021011802);
    end;

  procedure thlcgwasm.a_cmp_ref_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; const ref: treference; reg: tregister; l: tasmlabel);
    begin
      internalerror(2021011802);
    end;

  procedure thlcgwasm.a_cmp_reg_ref_label(list: TAsmList; size: tdef; cmp_op: topcmp; reg: tregister; const ref: treference; l: tasmlabel);
    begin
      internalerror(2021011802);
    end;

  procedure thlcgwasm.a_cmp_reg_reg_label(list: TAsmList; size: tdef; cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel);
    begin
      internalerror(2021011802);
    end;

  procedure thlcgwasm.a_jmp_always(list: TAsmList; l: tasmlabel);
    begin
      if l=current_procinfo.CurrBreakLabel then
        list.concat(taicpu.op_const(a_br,br_blocks-loopBreakBr))
      else if l=current_procinfo.CurrContinueLabel then
        list.concat(taicpu.op_const(a_br,br_blocks-loopContBr))
      else if l=current_procinfo.CurrExitLabel then
        list.concat(taicpu.op_const(a_br,br_blocks-exitBr))
      else
        Internalerror(2019091806); // unexpected jump
    end;

  procedure thlcgwasm.a_loadfpu_ref_ref(list: TAsmList; fromsize, tosize: tdef; const ref1, ref2: treference);
    var
      dstack_slots: longint;
      tmpref1, tmpref2: treference;
    begin
      tmpref1:=ref1;
      tmpref2:=ref2;
      dstack_slots:=prepare_stack_for_ref(list,tmpref2,false);
      a_load_ref_stack(list,fromsize,tmpref1,prepare_stack_for_ref(list,tmpref1,false));
      resizestackfpuval(list,def_cgsize(fromsize),def_cgsize(tosize));
      a_load_stack_ref(list,tosize,tmpref2,dstack_slots);
    end;

  procedure thlcgwasm.a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister);
    var
      tmpref: treference;
    begin
      tmpref:=ref;
      a_load_ref_stack(list,fromsize,tmpref,prepare_stack_for_ref(list,tmpref,false));
      resizestackfpuval(list,def_cgsize(fromsize),def_cgsize(tosize));
      a_load_stack_reg(list,tosize,reg);
    end;

  procedure thlcgwasm.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference);
    var
      dstack_slots: longint;
      tmpref: treference;
    begin
      tmpref:=ref;
      dstack_slots:=prepare_stack_for_ref(list,tmpref,false);
      a_load_reg_stack(list,fromsize,reg);
      resizestackfpuval(list,def_cgsize(fromsize),def_cgsize(tosize));
      a_load_stack_ref(list,tosize,tmpref,dstack_slots);
    end;

  procedure thlcgwasm.a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister);
    begin
      a_load_reg_stack(list,fromsize,reg1);
      resizestackfpuval(list,def_cgsize(fromsize),def_cgsize(tosize));
      a_load_stack_reg(list,tosize,reg2);
    end;

  procedure thlcgwasm.g_concatcopy(list: TAsmList; size: tdef; const source, dest: treference);
    var
      pd: tprocdef;
      cgpara1,cgpara2,cgpara3 : TCGPara;
    begin
      if (source.base=NR_EVAL_STACK_BASE) or (source.base=NR_LOCAL_STACK_POINTER_REG) or
         (source.index=NR_EVAL_STACK_BASE) or (source.index=NR_LOCAL_STACK_POINTER_REG) or
         (dest.base=NR_EVAL_STACK_BASE) or (dest.base=NR_LOCAL_STACK_POINTER_REG) or
         (dest.index=NR_EVAL_STACK_BASE) or (dest.index=NR_LOCAL_STACK_POINTER_REG) or
         (size.size in [1,2,4,8]) then
        inherited
      else
        begin
          pd:=search_system_proc('MOVE');
          cgpara1.init;
          cgpara2.init;
          cgpara3.init;
          paramanager.getcgtempparaloc(list,pd,1,cgpara1);
          paramanager.getcgtempparaloc(list,pd,2,cgpara2);
          paramanager.getcgtempparaloc(list,pd,3,cgpara3);
          if pd.is_pushleftright then
            begin
              { load source }
              a_loadaddr_ref_cgpara(list,voidtype,source,cgpara1);
              { load destination }
              a_loadaddr_ref_cgpara(list,voidtype,dest,cgpara2);
              { load size }
              a_load_const_cgpara(list,sizesinttype,size.size,cgpara3);
            end
          else
            begin
              { load size }
              a_load_const_cgpara(list,sizesinttype,size.size,cgpara3);
              { load destination }
              a_loadaddr_ref_cgpara(list,voidtype,dest,cgpara2);
              { load source }
              a_loadaddr_ref_cgpara(list,voidtype,source,cgpara1);
            end;
          paramanager.freecgpara(list,cgpara3);
          paramanager.freecgpara(list,cgpara2);
          paramanager.freecgpara(list,cgpara1);
          g_call_system_proc(list,pd,[@cgpara1,@cgpara2,@cgpara3],nil).resetiftemp;
          cgpara3.done;
          cgpara2.done;
          cgpara1.done;
        end;
    end;

  procedure thlcgwasm.g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);
    var
      pd: tcpuprocdef;
    begin
      pd:=tcpuprocdef(current_procinfo.procdef);
      g_procdef(list,pd);

      ttgwasm(tg).allocframepointer(list,pd.frame_pointer_ref);
      ttgwasm(tg).allocbasepointer(list,pd.base_pointer_ref);

      g_fingerprint(list);

      list.Concat(taicpu.op_sym(a_global_get,current_asmdata.RefAsmSymbol(STACK_POINTER_SYM,AT_LABEL)));
      incstack(list,1);
      list.Concat(taicpu.op_ref(a_local_set,pd.base_pointer_ref));
      decstack(list,1);

      if (localsize>0) then begin
        list.Concat(taicpu.op_ref(a_local_get,pd.base_pointer_ref));
        incstack(list,1);
        list.concat(taicpu.op_const(a_i32_const, localsize ));
        incstack(list,1);
        list.concat(taicpu.op_none(a_i32_sub));
        decstack(list,1);
        list.Concat(taicpu.op_ref(a_local_set,pd.frame_pointer_ref));
        decstack(list,1);
        list.Concat(taicpu.op_ref(a_local_get,pd.frame_pointer_ref));
        incstack(list,1);
        list.Concat(taicpu.op_sym(a_global_set,current_asmdata.RefAsmSymbol(STACK_POINTER_SYM,AT_LABEL)));
        decstack(list,1);
      end;
    end;

  procedure thlcgwasm.g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean);
    var
      pd: tcpuprocdef;
    begin
      pd:=tcpuprocdef(current_procinfo.procdef);
      list.Concat(taicpu.op_ref(a_local_get,pd.base_pointer_ref));
      incstack(list,1);
      list.Concat(taicpu.op_sym(a_global_set,current_asmdata.RefAsmSymbol(STACK_POINTER_SYM,AT_LABEL)));
      decstack(list,1);

      list.concat(taicpu.op_none(a_return));
      list.concat(taicpu.op_none(a_end_function));
    end;

  procedure thlcgwasm.g_rangecheck(list: TAsmList; const l: tlocation; fromdef, todef: tdef);
    var
{$if defined(cpuhighleveltarget)}
      aintmax: tcgint;
{$elseif defined(cpu64bitalu) or defined(cpu32bitalu)}
      aintmax: aint;
{$else}
      aintmax: longint;
{$endif}
      //neglabel : tasmlabel;
      //hreg : tregister;
      lto,hto,
      lfrom,hfrom : TConstExprInt;
      fromsize, tosize: cardinal;
      maxdef: tdef;
      from_signed, to_signed: boolean;
    begin
      { range checking on and range checkable value? }
      if not(cs_check_range in current_settings.localswitches) or
         not(fromdef.typ in [orddef,enumdef]) or
         { C-style booleans can't really fail range checks, }
         { all values are always valid                      }
         is_cbool(todef) then
        exit;
{$if not defined(cpuhighleveltarget) and not defined(cpu64bitalu)}
        { handle 64bit rangechecks separate for 32bit processors }
        if is_64bit(fromdef) or is_64bit(todef) then
          begin
             cg64.g_rangecheck64(list,l,fromdef,todef);
             exit;
          end;
{$endif ndef cpuhighleveltarget and ndef cpu64bitalu}
      { only check when assigning to scalar, subranges are different, }
      { when todef=fromdef then the check is always generated         }
      getrange(fromdef,lfrom,hfrom);
      getrange(todef,lto,hto);
      from_signed := is_signed(fromdef);
      to_signed := is_signed(todef);
      { check the rangedef of the array, not the array itself }
      { (only change now, since getrange needs the arraydef)   }
      if (todef.typ = arraydef) then
        todef := tarraydef(todef).rangedef;
      { no range check if from and to are equal and are both longint/dword }
      { (if we have a 32bit processor) or int64/qword, since such          }
      { operations can at most cause overflows (JM)                        }
      { Note that these checks are mostly processor independent, they only }
      { have to be changed once we introduce 64bit subrange types          }
{$if defined(cpuhighleveltarget) or defined(cpu64bitalu)}
      if (fromdef=todef) and
         (fromdef.typ=orddef) and
         (((((torddef(fromdef).ordtype=s64bit) and
             (lfrom = low(int64)) and
             (hfrom = high(int64))) or
            ((torddef(fromdef).ordtype=u64bit) and
             (lfrom = low(qword)) and
             (hfrom = high(qword))) or
            ((torddef(fromdef).ordtype=scurrency) and
             (lfrom = low(int64)) and
             (hfrom = high(int64)))))) then
        exit;
{$endif cpuhighleveltarget or cpu64bitalu}
      { 32 bit operations are automatically widened to 64 bit on 64 bit addr
        targets }
{$ifdef cpu32bitaddr}
      if (fromdef = todef) and
         (fromdef.typ=orddef) and
         (((((torddef(fromdef).ordtype = s32bit) and
             (lfrom = int64(low(longint))) and
             (hfrom = int64(high(longint)))) or
            ((torddef(fromdef).ordtype = u32bit) and
             (lfrom = low(cardinal)) and
             (hfrom = high(cardinal)))))) then
        exit;
{$endif cpu32bitaddr}

      { optimize some range checks away in safe cases }
      fromsize := fromdef.size;
      tosize := todef.size;
      if ((from_signed = to_signed) or
          (not from_signed)) and
         (lto<=lfrom) and (hto>=hfrom) and
         (fromsize <= tosize) then
        begin
          { if fromsize < tosize, and both have the same signed-ness or }
          { fromdef is unsigned, then all bit patterns from fromdef are }
          { valid for todef as well                                     }
          if (fromsize < tosize) then
            exit;
          if (fromsize = tosize) and
             (from_signed = to_signed) then
            { only optimize away if all bit patterns which fit in fromsize }
            { are valid for the todef                                      }
            begin
{$ifopt Q+}
{$define overflowon}
{$Q-}
{$endif}
{$ifopt R+}
{$define rangeon}
{$R-}
{$endif}
              if to_signed then
                begin
                  { calculation of the low/high ranges must not overflow 64 bit
                   otherwise we end up comparing with zero for 64 bit data types on
                   64 bit processors }
                  if (lto = (int64(-1) << (tosize * 8 - 1))) and
                     (hto = (-((int64(-1) << (tosize * 8 - 1))+1))) then
                    exit
                end
              else
                begin
                  { calculation of the low/high ranges must not overflow 64 bit
                   otherwise we end up having all zeros for 64 bit data types on
                   64 bit processors }
                  if (lto = 0) and
                     (qword(hto) = (qword(-1) >> (64-(tosize * 8))) ) then
                    exit
                end;
{$ifdef overflowon}
{$Q+}
{$undef overflowon}
{$endif}
{$ifdef rangeon}
{$R+}
{$undef rangeon}
{$endif}
            end
        end;

      { depending on the types involved, we perform the range check for 64 or
        for 32 bit }
      if fromsize=8 then
        maxdef:=fromdef
      else
        maxdef:=todef;
{$if sizeof(aintmax) = 8}
      if maxdef.size=8 then
        aintmax:=high(int64)
      else
{$endif}
        begin
          aintmax:=high(longint);
          maxdef:=u32inttype;
        end;

      { generate the rangecheck code for the def where we are going to }
      { store the result                                               }

      { use the trick that                                                 }
      { a <= x <= b <=> 0 <= x-a <= b-a <=> unsigned(x-a) <= unsigned(b-a) }

      { To be able to do that, we have to make sure however that either    }
      { fromdef and todef are both signed or unsigned, or that we leave    }
      { the parts < 0 and > maxlongint out                                 }

      if from_signed xor to_signed then
        begin
           if from_signed then
             { from is signed, to is unsigned }
             begin
               { if high(from) < 0 -> always range error }
               if (hfrom < 0) or
                  { if low(to) > maxlongint also range error }
                  (lto > aintmax) then
                 begin
                   g_call_system_proc(list,'fpc_rangeerror',[],nil).resetiftemp;
                   exit
                 end;
               { from is signed and to is unsigned -> when looking at to }
               { as an signed value, it must be < maxaint (otherwise     }
               { it will become negative, which is invalid since "to" is unsigned) }
               if hto > aintmax then
                 hto := aintmax;
             end
           else
             { from is unsigned, to is signed }
             begin
               if (lfrom > aintmax) or
                  (hto < 0) then
                 begin
                   g_call_system_proc(list,'fpc_rangeerror',[],nil).resetiftemp;
                   exit
                 end;
               { from is unsigned and to is signed -> when looking at to }
               { as an unsigned value, it must be >= 0 (since negative   }
               { values are the same as values > maxlongint)             }
               if lto < 0 then
                 lto := 0;
             end;
        end;
      a_load_loc_stack(list,fromdef,l);
      resize_stack_int_val(list,fromdef,maxdef,false);
      a_load_const_stack(list,maxdef,tcgint(int64(lto)),R_INTREGISTER);
      a_op_stack(list,OP_SUB,maxdef);
      {
      if from_signed then
        a_cmp_const_reg_label(list,OS_INT,OC_GTE,aint(hto-lto),hreg,neglabel)
      else
      }
      if qword(hto-lto)>qword(aintmax) then
        a_load_const_stack(list,maxdef,aintmax,R_INTREGISTER)
      else
        a_load_const_stack(list,maxdef,tcgint(int64(hto-lto)),R_INTREGISTER);
      a_cmp_stack_stack(list,maxdef,OC_A);

      current_asmdata.CurrAsmList.concat(taicpu.op_none(a_if));
      thlcgwasm(hlcg).incblock;
      thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);

      g_call_system_proc(list,'fpc_rangeerror',[],nil).resetiftemp;

      current_asmdata.CurrAsmList.concat(taicpu.op_none(a_end_if));
      thlcgwasm(hlcg).decblock;
    end;

  procedure thlcgwasm.g_overflowcheck(list: TAsmList; const Loc: tlocation; def: tdef);
    begin
      { not possible, need the original operands }
      internalerror(2012102101);
    end;

  procedure thlcgwasm.g_overflowCheck_loc(List: TAsmList; const Loc: TLocation; def: TDef; var ovloc: tlocation);
    var
      hl : tasmlabel;
    begin
      if not(cs_check_overflow in current_settings.localswitches) then
        exit;
      current_asmdata.getjumplabel(hl);
      a_cmp_const_loc_label(list,s32inttype,OC_EQ,0,ovloc,hl);
      g_call_system_proc(list,'fpc_overflow',[],nil);
      a_label(list,hl);
    end;

  procedure thlcgwasm.maybe_change_load_node_reg(list: TAsmList; var n: tnode; reload: boolean);
    begin
      { don't do anything, all registers become stack locations anyway }
    end;

  procedure thlcgwasm.gen_entry_code(list: TAsmList);
    begin
      inherited;
      list.concat(taicpu.op_none(a_block));
      incblock;
      exitBr:=br_blocks;
    end;

  procedure thlcgwasm.gen_exit_code(list: TAsmList);
    begin
      list.concat(taicpu.op_none(a_end_block));
      decblock;
      if fevalstackheight<>0 then
        list.concat(tai_comment.Create(strpnew('!!! values remaining on stack at end of block !!!')));
      inherited;
    end;

  procedure thlcgwasm.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: tdef; src, dst: tregister);
    begin
      internalerror(2012090201);
    end;

  procedure thlcgwasm.a_loadmm_loc_reg(list: TAsmList; fromsize, tosize: tdef; const loc: tlocation; const reg: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2012090202);
    end;

  procedure thlcgwasm.a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tdef; reg1, reg2: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2012060130);
    end;

  procedure thlcgwasm.a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tdef; const ref: treference; reg: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2012060131);
    end;

  procedure thlcgwasm.a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tdef; reg: tregister; const ref: treference; shuffle: pmmshuffle);
    begin
      internalerror(2012060132);
    end;

  procedure thlcgwasm.a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size: tdef; src, dst: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2012060133);
    end;

  procedure thlcgwasm.a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tdef; intreg, mmreg: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2012060134);
    end;

  procedure thlcgwasm.a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tdef; mmreg, intreg: tregister; shuffle: pmmshuffle);
    begin
      internalerror(2012060135);
    end;

  procedure thlcgwasm.g_stackpointer_alloc(list: TAsmList; size: longint);
    begin
      internalerror(2012090203);
    end;

  procedure thlcgwasm.g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);
    begin
      internalerror(2012090204);
    end;

  procedure thlcgwasm.g_adjust_self_value(list: TAsmList; procdef: tprocdef; ioffset: aint);
    begin
      internalerror(2012090205);
    end;

  procedure thlcgwasm.g_local_unwind(list: TAsmList; l: TAsmLabel);
    begin
      internalerror(2012090206);
    end;

  procedure thlcgwasm.g_procdef(list: TAsmList; pd: tprocdef);
    begin
      list.Concat(tai_functype.create(pd.mangledname,tcpuprocdef(pd).create_functype));
    end;

  procedure thlcgwasm.a_load_stack_reg(list: TAsmList; size: tdef; reg: tregister);
    begin
      list.concat(taicpu.op_reg(a_local_set,reg));
      decstack(list,1);
    end;

  procedure thlcgwasm.a_load_stack_ref(list: TAsmList; size: tdef; const ref: treference; extra_slots: longint);
    var
      opc: tasmop;
      finishandval: tcgint;
    begin
      { fake location that indicates the value has to remain on the stack }
      if ref.base=NR_EVAL_STACK_BASE then
        exit;
      opc:=loadstoreopcref(size,false,ref,finishandval);

      list.concat(taicpu.op_ref(opc,ref));
      { avoid problems with getting the size of an open array etc }
      if wasmAlwayInMem(size) then
        size:=ptruinttype;
      decstack(list,1+extra_slots);
    end;

  procedure thlcgwasm.a_load_reg_stack(list: TAsmList; size: tdef; reg: tregister);
    begin
      list.concat(taicpu.op_reg(a_local_get,reg));
      incstack(list,1);
    end;

  procedure thlcgwasm.a_load_ref_stack(list: TAsmList; size: tdef; const ref: treference; extra_slots: longint);
    var
      opc: tasmop;
      finishandval: tcgint;
    begin
      { fake location that indicates the value is already on the stack? }
      if (ref.base=NR_EVAL_STACK_BASE) then
        exit;
      opc:=loadstoreopcref(size,true,ref,finishandval);

      list.concat(taicpu.op_ref(opc,ref));

      { avoid problems with getting the size of an open array etc }
      if wasmAlwayInMem(size) then
        size:=ptruinttype;
      incstack(list,1-extra_slots);
      if finishandval<>-1 then
        a_op_const_stack(list,OP_AND,size,finishandval);

      // there's no cast check in Wasm
      //if ref.checkcast then
      //  gen_typecheck(list,a_checkcast,size);
    end;

  function thlcgwasm.loadstoreopcref(def: tdef; isload: boolean; const ref: treference; out finishandval: tcgint): tasmop;
    const
                          {iisload} {issigned}
      getputmem8   : array [boolean, boolean] of TAsmOp = ((a_i32_store8, a_i32_store8),   (a_i32_load8_u, a_i32_load8_s));
      getputmem16  : array [boolean, boolean] of TAsmOp = ((a_i32_store16, a_i32_store16), (a_i32_load16_u ,a_i32_load16_s));
      getputmem32  : array [boolean, boolean] of TAsmOp = ((a_i32_store, a_i32_store),     (a_i32_load, a_i32_load));
      getputmem64  : array [boolean, boolean] of TAsmOp = ((a_i64_store,  a_i64_store),    (a_i64_load, a_i64_load));
      getputmemf32 : array [boolean] of TAsmOp = (a_f32_store, a_f32_load);
      getputmemf64 : array [boolean] of TAsmOp = (a_f64_store, a_f64_load);
    begin
      if (ref.base<>NR_LOCAL_STACK_POINTER_REG) or assigned(ref.symbol) then
        begin
          { -> either a global (static) field, or a regular field. If a regular
            field, then ref.base contains the self pointer, otherwise
            ref.base=NR_NO. In both cases, the symbol contains all other
            information (combined field name and type descriptor) }
          case def.size of
            1: result := getputmem8[isload, is_signed(def)];
            2: result := getputmem16[isload, is_signed(def)];
            4:
              if is_single(def) then
                result := getputmemf32[isload]
              else
                result := getputmem32[isload, is_signed(def)];
            8: if is_double(def) then
                 result := getputmemf64[isload]
               else
                result := getputmem64[isload, is_signed(def)];
          else
            Internalerror(2019091501);
          end;

          //result:=getputopc[isload,ref.base=NR_NO];
          finishandval:=-1;
          { erase sign extension for byte/smallint loads }
          if (def2regtyp(def)=R_INTREGISTER) and
             not is_signed(def) and
             (def.typ=orddef) and
             not is_widechar(def) then
            case def.size of
              1: if (torddef(def).high>127) then
                   finishandval:=255;
              2: if (torddef(def).high>32767) then
                   finishandval:=65535;
            end;
        end
      else
        begin
          finishandval:=-1;
          if isload then
            result := a_local_get
          else
            result := a_local_set;
        end;
    end;

  procedure thlcgwasm.resize_stack_int_val(list: TAsmList; fromsize, tosize: tdef; formemstore: boolean);
    var
      fromcgsize, tocgsize: tcgsize;
    begin
      { When storing to an array, field or global variable, make sure the
        static type verification can determine that the stored value fits
        within the boundaries of the declared type (to appease the Dalvik VM).
        Local variables either get their type upgraded in the debug info,
        or have no type information at all }
      if formemstore and
         (tosize.typ=orddef) then
        if (torddef(tosize).ordtype in [u8bit,uchar]) then
          tosize:=s8inttype
        else if torddef(tosize).ordtype=u16bit then
          tosize:=s16inttype;

      fromcgsize:=def_cgsize(fromsize);
      tocgsize:=def_cgsize(tosize);
      if fromcgsize in [OS_S64,OS_64] then
        begin
          if not(tocgsize in [OS_S64,OS_64]) then
            begin
              { truncate }
              list.concat(taicpu.op_none(a_i32_wrap_i64));
              case tocgsize of
                OS_8:
                  a_op_const_stack(list,OP_AND,s32inttype,255);
                OS_S8:
                  list.concat(taicpu.op_none(a_i32_extend8_s));
                OS_16:
                  a_op_const_stack(list,OP_AND,s32inttype,65535);
                OS_S16:
                  list.concat(taicpu.op_none(a_i32_extend16_s));
                OS_32,OS_S32:
                  ;
                else
                  internalerror(2021012201);
              end;
            end;
        end
      else if tocgsize in [OS_S64,OS_64] then
        begin
          { extend }
          case fromcgsize of
            OS_8:
              begin
                a_op_const_stack(list,OP_AND,s32inttype,255);
                list.concat(taicpu.op_none(a_i64_extend_i32_u));
              end;
            OS_S8:
              begin
                list.concat(taicpu.op_none(a_i64_extend_i32_u));
                list.concat(taicpu.op_none(a_i64_extend8_s));
              end;
            OS_16:
              begin
                a_op_const_stack(list,OP_AND,s32inttype,65535);
                list.concat(taicpu.op_none(a_i64_extend_i32_u));
              end;
            OS_S16:
              begin
                list.concat(taicpu.op_none(a_i64_extend_i32_u));
                list.concat(taicpu.op_none(a_i64_extend16_s));
              end;
            OS_32:
              list.concat(taicpu.op_none(a_i64_extend_i32_u));
            OS_S32:
              list.concat(taicpu.op_none(a_i64_extend_i32_s));
            OS_64,OS_S64:
              ;
            else
              internalerror(2021010301);
          end;
        end
      else
        begin
          if tcgsize2size[fromcgsize]<tcgsize2size[tocgsize] then
            begin
              { extend }
              case fromcgsize of
                OS_8:
                  a_op_const_stack(list,OP_AND,s32inttype,255);
                OS_S8:
                  list.concat(taicpu.op_none(a_i32_extend8_s));
                OS_16:
                  a_op_const_stack(list,OP_AND,s32inttype,65535);
                OS_S16:
                  list.concat(taicpu.op_none(a_i32_extend16_s));
                OS_32,OS_S32:
                  ;
                else
                  internalerror(2021010302);
              end;
            end
          else if tcgsize2size[fromcgsize]>=tcgsize2size[tocgsize] then
            begin
              { truncate }
              case tocgsize of
                OS_8:
                  a_op_const_stack(list,OP_AND,s32inttype,255);
                OS_S8:
                  list.concat(taicpu.op_none(a_i32_extend8_s));
                OS_16:
                  a_op_const_stack(list,OP_AND,s32inttype,65535);
                OS_S16:
                  list.concat(taicpu.op_none(a_i32_extend16_s));
                OS_32,OS_S32:
                  ;
                else
                  internalerror(2021010302);
              end;
            end;
        end;
    end;

    procedure thlcgwasm.maybe_resize_stack_para_val(list: TAsmList; retdef: tdef; callside: boolean);
      var
        convsize: tdef;
      begin
        if (retdef.typ=orddef) then
          begin
            if (torddef(retdef).ordtype in [u8bit,u16bit,uchar]) and
               (torddef(retdef).high>=(1 shl (retdef.size*8-1))) then
              begin
                convsize:=nil;
                if callside then
                  if torddef(retdef).ordtype in [u8bit,uchar] then
                    convsize:=s8inttype
                  else
                    convsize:=s16inttype
                else if torddef(retdef).ordtype in [u8bit,uchar] then
                    convsize:=u8inttype
                  else
                    convsize:=u16inttype;
                if assigned(convsize) then
                  resize_stack_int_val(list,s32inttype,convsize,false);
              end;
          end;
      end;


  procedure thlcgwasm.g_adjust_stack_after_call(list: TAsmList; pd: tabstractprocdef);
    var
      totalremovesize: longint;
      realresdef: tdef;
      ft: TWasmFuncType;
    begin
      if pd.typ=procvardef then
        ft:=tcpuprocvardef(pd).create_functype
      else
        ft:=tcpuprocdef(pd).create_functype;
      totalremovesize:=Length(ft.params)-Length(ft.results);
      if Length(ft.results)=0 then
        dec(totalremovesize);
      { remove parameters from internal evaluation stack counter (in case of
        e.g. no parameters and a result, it can also increase) }
      if totalremovesize>0 then
        decstack(list,totalremovesize)
      else if totalremovesize<0 then
        incstack(list,-totalremovesize);
      ft.free;
    end;


  procedure thlcgwasm.g_fingerprint(list: TAsmList);
    begin
      list.concat(taicpu.op_const(a_i64_const,Random(high(int64))));
      list.concat(taicpu.op_const(a_i64_const,Random(high(int64))));
      list.concat(taicpu.op_const(a_i64_const,Random(high(int64))));
      list.concat(taicpu.op_const(a_i64_const,Random(high(int64))));
      list.concat(taicpu.op_none(a_drop));
      list.concat(taicpu.op_none(a_drop));
      list.concat(taicpu.op_none(a_drop));
      list.concat(taicpu.op_none(a_drop));
    end;


  procedure thlcgwasm.resizestackfpuval(list: TAsmList; fromsize, tosize: tcgsize);
    begin
      if (fromsize=OS_F32) and
         (tosize=OS_F64) then
        begin
          list.concat(taicpu.op_none(a_f64_promote_f32));
        end
      else if (fromsize=OS_F64) and
              (tosize=OS_F32) then
        begin
          list.concat(taicpu.op_none(a_f32_demote_f64));
        end;
    end;

  procedure create_hlcodegen_cpu;
    begin
      hlcg:=thlcgwasm.create;
      create_codegen;
    end;

initialization
  chlcgobj:=thlcgwasm;
  create_hlcodegen:=@create_hlcodegen_cpu;
end.
