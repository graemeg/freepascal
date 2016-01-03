{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the code generator for the PowerPC

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

{$I fpcdefs.inc}

interface

uses
  globtype, symtype, symdef, symsym,
  cgbase, cgobj,cgppc,
  aasmbase, aasmcpu, aasmtai,aasmdata,
  cpubase, cpuinfo, cgutils, rgcpu,
  parabase;

type
  tcgppc = class(tcgppcgen)
    procedure init_register_allocators; override;
    procedure done_register_allocators; override;

<<<<<<< HEAD
=======
    { passing parameters, per default the parameter is pushed }
    { nr gives the number of the parameter (enumerated from   }
    { left to right), this allows to move the parameter to    }
    { register, if the cpu supports register calling          }
    { conventions                                             }
    procedure a_load_ref_cgpara(list: TAsmList; size: tcgsize; const r: treference;
      const paraloc: tcgpara); override;

>>>>>>> graemeg/cpstrnew
    procedure a_call_name(list: TAsmList; const s: string; weak: boolean); override;
    procedure a_call_reg(list: TAsmList; reg: tregister); override;

    procedure a_op_const_reg(list: TAsmList; Op: TOpCG; size: TCGSize; a:
      aint; reg: TRegister); override;
    procedure a_op_reg_reg(list: TAsmList; Op: TOpCG; size: TCGSize; src,
      dst: TRegister); override;

    procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg;
      size: tcgsize; a: aint; src, dst: tregister); override;
    procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
      size: tcgsize; src1, src2, dst: tregister); override;

    { move instructions }
    procedure a_load_const_reg(list: TAsmList; size: tcgsize; a: aint; reg:
      tregister); override;
    { loads the memory pointed to by ref into register reg }
    procedure a_load_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const
      Ref: treference; reg: tregister); override;
    procedure a_load_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1,
      reg2: tregister); override;

    {  comparison operations }
    procedure a_cmp_const_reg_label(list: TAsmList; size: tcgsize; cmp_op:
      topcmp; a: aint; reg: tregister;
      l: tasmlabel); override;
    procedure a_cmp_reg_reg_label(list: TAsmList; size: tcgsize; cmp_op:
      topcmp; reg1, reg2: tregister; l: tasmlabel); override;

    procedure a_jmp_name(list: TAsmList; const s: string); override;
    procedure a_jmp_always(list: TAsmList; l: tasmlabel); override;

    { need to override this for ppc64 to avoid calling CG methods which allocate
      registers during creation of the interface wrappers to subtract ioffset from
      the self pointer. But register allocation does not take place for them (which
      would probably be the generic fix) so we need to have a specialized method
      that uses the R11 scratch register in these cases.
      At the same time this allows > 32 bit offsets as well.
    }
    procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);override;

    { need to override this for ppc64 to avoid calling CG methods which allocate 
      registers during creation of the interface wrappers to subtract ioffset from 
      the self pointer. But register allocation does not take place for them (which
      would probably be the generic fix) so we need to have a specialized method
      that uses the R11 scratch register in these cases.
      At the same time this allows > 32 bit offsets as well.
    }
    procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);override;

    { need to override this for ppc64 to avoid calling CG methods which allocate 
      registers during creation of the interface wrappers to subtract ioffset from 
      the self pointer. But register allocation does not take place for them (which
      would probably be the generic fix) so we need to have a specialized method
      that uses the R11 scratch register in these cases.
      At the same time this allows > 32 bit offsets as well.
    }
    procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);override;

    { need to override this for ppc64 to avoid calling CG methods which allocate 
      registers during creation of the interface wrappers to subtract ioffset from 
      the self pointer. But register allocation does not take place for them (which
      would probably be the generic fix) so we need to have a specialized method
      that uses the R11 scratch register in these cases.
      At the same time this allows > 32 bit offsets as well.
    }
    procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);override;

    { need to override this for ppc64 to avoid calling CG methods which allocate 
      registers during creation of the interface wrappers to subtract ioffset from 
      the self pointer. But register allocation does not take place for them (which
      would probably be the generic fix) so we need to have a specialized method
      that uses the R11 scratch register in these cases.
      At the same time this allows > 32 bit offsets as well.
    }
    procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);override;

    procedure g_profilecode(list: TAsmList); override;
    procedure g_proc_entry(list: TAsmList; localsize: longint; nostackframe:
      boolean); override;
    procedure g_proc_exit(list: TAsmList; parasize: longint; nostackframe:
      boolean); override;
    procedure g_save_registers(list: TAsmList); override;
    procedure g_restore_registers(list: TAsmList); override;

    procedure a_loadaddr_ref_reg(list: TAsmList; const ref: treference; r:
      tregister); override;

    procedure g_concatcopy(list: TAsmList; const source, dest: treference;
      len: aint); override;

<<<<<<< HEAD
<<<<<<< HEAD
=======
    procedure g_external_wrapper(list: TAsmList; pd: TProcDef; const externalname: string); override;
=======
    procedure g_external_wrapper(list: TAsmList; pd: TProcDef; const externalname: string); override;

  private
>>>>>>> origin/fixes_2_2

>>>>>>> graemeg/fixes_2_2
  private

    procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);

    { returns whether a reference can be used immediately in a powerpc }
    { instruction                                                      }
    function issimpleref(const ref: treference): boolean;

    { contains the common code of a_load_reg_ref and a_load_ref_reg }
    procedure a_load_store(list: TAsmList; op: tasmop; reg: tregister;
      ref: treference); override;

    { returns the lowest numbered FP register in use, and the number of used FP registers
      for the current procedure }
    procedure calcFirstUsedFPR(out firstfpr : TSuperRegister; out fprcount : aint);
    { returns the lowest numbered GP register in use, and the number of used GP registers
      for the current procedure }
    procedure calcFirstUsedGPR(out firstgpr : TSuperRegister; out gprcount : aint);

    { generates code to call a method with the given string name. The boolean options
     control code generation. If prependDot is true, a single dot character is prepended to
     the string, if addNOP is true a single NOP instruction is added after the call, and
     if includeCall is true, the method is marked as having a call, not if false. This
     option is particularly useful to prevent generation of a larger stack frame for the
     register save and restore helper functions. }
    procedure a_call_name_direct(list: TAsmList; opc: tasmop; s: string; weak: boolean; prependDot : boolean;
      addNOP : boolean; includeCall : boolean = true);

    procedure a_jmp_name_direct(list : TAsmList; opc: tasmop; s : string; prependDot : boolean);

    { emits code to store the given value a into the TOC (if not already in there), and load it from there
     as well }
    procedure loadConstantPIC(list : TAsmList; size : TCGSize; a : aint; reg : TRegister);

    procedure profilecode_savepara(para : tparavarsym; list : TAsmList);
    procedure profilecode_restorepara(para : tparavarsym; list : TAsmList);
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
  TShiftOpCG2AsmOpConst : array[boolean, OP_SAR..OP_SHR] of TAsmOp = (
    (A_SRAWI, A_SLWI, A_SRWI), (A_SRADI, A_SLDI, A_SRDI)
    );

implementation

uses
  sysutils, cclasses,
  globals, verbose, systems, cutils,
  symconst, fmodule,
  rgobj, tgobj, cpupi, procinfo, paramgr, cpupara;

function is_signed_cgsize(const size : TCgSize) : Boolean;
begin
  case size of
    OS_S8,OS_S16,OS_S32,OS_S64 : result := true;
    OS_8,OS_16,OS_32,OS_64 : result := false;
    else
      internalerror(2006050701);
  end;
end;

{ finds positive and negative powers of two of the given value, returning the
 power and whether it's a negative power or not in addition to the actual result
 of the function }
function ispowerof2(value : aInt; out power : byte; out neg : boolean) : boolean;
var
  i : longint;
  hl : aInt;
begin
  result := false;
  neg := false;
  { also try to find negative power of two's by negating if the
   value is negative. low(aInt) is special because it can not be
   negated. Simply return the appropriate values for it }
  if (value < 0) then begin
    neg := true;
    if (value = low(aInt)) then begin
      power := sizeof(aInt)*8-1;
      result := true;
      exit;
    end;
    value := -value;
  end;

  if ((value and (value-1)) <> 0) then begin
    result := false;
    exit;
  end;
  hl := 1;
  for i := 0 to (sizeof(aInt)*8-1) do begin
    if (hl = value) then begin
      result := true;
      power := i;
      exit;
    end;
    hl := hl shl 1;
  end;
end;

{ returns the number of instruction required to load the given integer into a register.
 This is basically a stripped down version of a_load_const_reg, increasing a counter
 instead of emitting instructions. }
function getInstructionLength(a : aint) : longint;

  function get32bitlength(a : longint; var length : longint) : boolean; inline;
  var
    is_half_signed : byte;
  begin
    { if the lower 16 bits are zero, do a single LIS }
    if (smallint(a) = 0) and ((a shr 16) <> 0) then begin
      inc(length);
      get32bitlength := longint(a) < 0;
    end else begin
      is_half_signed := ord(smallint(lo(a)) < 0);
      inc(length);
      if smallint(hi(a) + is_half_signed) <> 0 then
        inc(length);
      get32bitlength := (smallint(a) < 0) or (a < 0);
    end;
  end;

var
  extendssign : boolean;

begin
  result := 0;
  if (lo(a) = 0) and (hi(a) <> 0) then begin
    get32bitlength(hi(a), result);
    inc(result);
  end else begin
    extendssign := get32bitlength(lo(a), result);
    if (extendssign) and (hi(a) = 0) then
      inc(result)
    else if (not
      ((extendssign and (longint(hi(a)) = -1)) or
       ((not extendssign) and (hi(a)=0)))
      ) then begin
      get32bitlength(hi(a), result);
      inc(result);
    end;
  end;
end;

procedure tcgppc.init_register_allocators;
begin
  inherited init_register_allocators;
  if (target_info.system <> system_powerpc64_darwin) then
    // r13 is tls, do not use, r2 is not available
<<<<<<< HEAD
<<<<<<< HEAD
    rg[R_INTREGISTER] := trgintcpu.create(R_INTREGISTER, R_SUBWHOLE,
=======
    rg[R_INTREGISTER] := trgcpu.create(R_INTREGISTER, R_SUBWHOLE,
>>>>>>> graemeg/fixes_2_2
=======
    rg[R_INTREGISTER] := trgcpu.create(R_INTREGISTER, R_SUBWHOLE,
>>>>>>> origin/fixes_2_2
      [{$ifdef user0} RS_R0, {$endif} RS_R3, RS_R4, RS_R5, RS_R6, RS_R7, RS_R8,
       RS_R9, RS_R10, RS_R11, RS_R12, RS_R31, RS_R30, RS_R29,
       RS_R28, RS_R27, RS_R26, RS_R25, RS_R24, RS_R23, RS_R22,
       RS_R21, RS_R20, RS_R19, RS_R18, RS_R17, RS_R16, RS_R15,
       RS_R14], first_int_imreg, [])
  else
    { special for darwin/ppc64: r2 available volatile, r13 = tls }
    rg[R_INTREGISTER] := trgintcpu.create(R_INTREGISTER, R_SUBWHOLE,
      [{$ifdef user0} RS_R0, {$endif} RS_R2, RS_R3, RS_R4, RS_R5, RS_R6, RS_R7, RS_R8,
<<<<<<< HEAD
<<<<<<< HEAD
       RS_R9, RS_R10, RS_R11, RS_R12, RS_R31, RS_R30, RS_R29,
       RS_R28, RS_R27, RS_R26, RS_R25, RS_R24, RS_R23, RS_R22,
       RS_R21, RS_R20, RS_R19, RS_R18, RS_R17, RS_R16, RS_R15,
       RS_R14], first_int_imreg, []);	
=======
=======
>>>>>>> origin/fixes_2_2
        RS_R9, RS_R10, RS_R11, RS_R12, RS_R31, RS_R30, RS_R29,
        RS_R28, RS_R27, RS_R26, RS_R25, RS_R24, RS_R23, RS_R22,
       RS_R21, RS_R20, RS_R19, RS_R18, RS_R17, RS_R16, RS_R15,
       RS_R14], first_int_imreg, []);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  rg[R_FPUREGISTER] := trgcpu.create(R_FPUREGISTER, R_SUBNONE,
    [RS_F0, RS_F1, RS_F2, RS_F3, RS_F4, RS_F5, RS_F6, RS_F7, RS_F8, RS_F9,
     RS_F10, RS_F11, RS_F12, RS_F13, RS_F31, RS_F30, RS_F29, RS_F28, RS_F27,
     RS_F26, RS_F25, RS_F24, RS_F23, RS_F22, RS_F21, RS_F20, RS_F19, RS_F18,
     RS_F17, RS_F16, RS_F15, RS_F14], first_fpu_imreg, []);
{ TODO: FIX ME}
  rg[R_MMREGISTER] := trgcpu.create(R_MMREGISTER, R_SUBNONE,
    [RS_M0, RS_M1, RS_M2], first_mm_imreg, []);
end;

procedure tcgppc.done_register_allocators;
begin
  rg[R_INTREGISTER].free;
  rg[R_FPUREGISTER].free;
  rg[R_MMREGISTER].free;
  inherited done_register_allocators;
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
procedure tcgppc.a_load_ref_cgpara(list: TAsmList; size: tcgsize; const r:
  treference; const paraloc: tcgpara);

var
  tmpref, ref: treference;
  location: pcgparalocation;
  sizeleft: aint;
  adjusttail : boolean;

begin
  location := paraloc.location;
  tmpref := r;
  sizeleft := paraloc.intsize;
  adjusttail := false;
  while assigned(location) do begin
    paramanager.allocparaloc(list,location);
    case location^.loc of
      LOC_REGISTER, LOC_CREGISTER:
        begin
          if not(size in [OS_NO,OS_128,OS_S128]) then
            a_load_ref_reg(list, size, location^.size, tmpref,
              location^.register)
          else begin
            { load non-integral sized memory location into register. This
             memory location be 1-sizeleft byte sized.
             Always assume that this memory area is properly aligned, eg. start
             loading the larger quantities for "odd" quantities first }
            case sizeleft of
              1,2,4,8 :
                a_load_ref_reg(list, int_cgsize(sizeleft), location^.size, tmpref,
                  location^.register);
              3 : begin
                a_reg_alloc(list, NR_R12);
                a_load_ref_reg(list, OS_16, location^.size, tmpref,
                  NR_R12);
                inc(tmpref.offset, tcgsize2size[OS_16]);
                a_load_ref_reg(list, OS_8, location^.size, tmpref,
                  location^.register);
                list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, location^.register, NR_R12, 8, 40));
                a_reg_dealloc(list, NR_R12);
              end;
              5 : begin
                a_reg_alloc(list, NR_R12);
                a_load_ref_reg(list, OS_32, location^.size, tmpref, NR_R12);
                inc(tmpref.offset, tcgsize2size[OS_32]);
                a_load_ref_reg(list, OS_8, location^.size, tmpref, location^.register);
                list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, location^.register, NR_R12, 8, 24));
                a_reg_dealloc(list, NR_R12);
              end;
              6 : begin
                a_reg_alloc(list, NR_R12);
                a_load_ref_reg(list, OS_32, location^.size, tmpref, NR_R12);
                inc(tmpref.offset, tcgsize2size[OS_32]);
                a_load_ref_reg(list, OS_16, location^.size, tmpref, location^.register);
                list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, location^.register, NR_R12, 16, 16));
                a_reg_dealloc(list, NR_R12);
              end;
              7 : begin
                a_reg_alloc(list, NR_R12);
                a_reg_alloc(list, NR_R0);
                a_load_ref_reg(list, OS_32, location^.size, tmpref, NR_R12);
                inc(tmpref.offset, tcgsize2size[OS_32]);
                a_load_ref_reg(list, OS_16, location^.size, tmpref, NR_R0);
                inc(tmpref.offset, tcgsize2size[OS_16]);
                a_load_ref_reg(list, OS_8, location^.size, tmpref, location^.register);
                list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, NR_R0, NR_R12, 16, 16));
                list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, location^.register, NR_R0, 8, 8));
                a_reg_dealloc(list, NR_R0);
                a_reg_dealloc(list, NR_R12);
              end;
              else begin
                { still > 8 bytes to load, so load data single register now }
                a_load_ref_reg(list, location^.size, location^.size, tmpref,
                  location^.register);
                { the block is > 8 bytes, so we have to store any bytes not
                 a multiple of the register size beginning with the MSB }
                adjusttail := true;
              end;
            end;
            if (adjusttail) and (sizeleft < sizeof(pint)) then
              a_op_const_reg(list, OP_SHL, OS_INT,
                (sizeof(pint) - sizeleft) * sizeof(pint),
                location^.register);
          end;
        end;
      LOC_REFERENCE:
        begin
          reference_reset_base(ref, location^.reference.index,
            location^.reference.offset,paraloc.alignment);
          g_concatcopy(list, tmpref, ref, sizeleft);
          if assigned(location^.next) then
            internalerror(2005010710);
        end;
      LOC_FPUREGISTER, LOC_CFPUREGISTER:
        case location^.size of
          OS_F32, OS_F64:
            a_loadfpu_ref_reg(list, location^.size, location^.size, tmpref, location^.register);
        else
          internalerror(2002072801);
        end;
      LOC_VOID:
        { nothing to do }
        ;
    else
      internalerror(2002081103);
    end;
    inc(tmpref.offset, tcgsize2size[location^.size]);
    dec(sizeleft, tcgsize2size[location^.size]);
    location := location^.next;
  end;
end;

>>>>>>> graemeg/cpstrnew
{ calling a procedure by name }

procedure tcgppc.a_call_name(list: TAsmList; const s: string; weak: boolean);
begin
    if (target_info.system <> system_powerpc64_darwin) then
<<<<<<< HEAD
<<<<<<< HEAD
      a_call_name_direct(list, A_BL, s, weak, target_info.system=system_powerpc64_aix, true)
=======
      a_call_name_direct(list, s, false, true)
>>>>>>> graemeg/fixes_2_2
=======
      a_call_name_direct(list, s, false, true)
>>>>>>> origin/fixes_2_2
    else
      begin
        list.concat(taicpu.op_sym(A_BL,get_darwin_call_stub(s,weak)));
        include(current_procinfo.flags,pi_do_call);
      end;
end;


procedure tcgppc.a_call_name_direct(list: TAsmList; opc: tasmop; s: string; weak: boolean; prependDot : boolean; addNOP : boolean; includeCall : boolean);
begin
  if (prependDot) then
    s := '.' + s;
  if not(weak) then
    list.concat(taicpu.op_sym(opc, current_asmdata.RefAsmSymbol(s)))
  else
    list.concat(taicpu.op_sym(opc, current_asmdata.WeakRefAsmSymbol(s)));
  if (addNOP) then
    list.concat(taicpu.op_none(A_NOP));

  if (includeCall) and
    assigned(current_procinfo) then
    include(current_procinfo.flags, pi_do_call);
end;


function get_rtoc_offset: longint;
begin
  result:=0;
  case target_info.abi of
    abi_powerpc_aix,
    abi_powerpc_darwin:
      result:=LA_RTOC_AIX;
    abi_powerpc_elfv1:
      result:=LA_RTOC_SYSV;
    abi_powerpc_elfv2:
      result:=LA_RTOC_ELFV2;
    else
      internalerror(2015021001);
  end;
end;

{ calling a procedure by address }

procedure tcgppc.a_call_reg(list: TAsmList; reg: tregister);
var
  tmpref: treference;
  tempreg : TRegister;
begin
  if (target_info.abi<>abi_powerpc_sysv) then
    inherited a_call_reg(list,reg)
  else if (not (cs_opt_size in current_settings.optimizerswitches)) then begin
    tempreg := getintregister(list, OS_INT);
    { load actual function entry (reg contains the reference to the function descriptor)
    into tempreg }
    reference_reset_base(tmpref, reg, 0, sizeof(pint));
    a_load_ref_reg(list, OS_ADDR, OS_ADDR, tmpref, tempreg);

    { save TOC pointer in stackframe }
    reference_reset_base(tmpref, NR_STACK_POINTER_REG, get_rtoc_offset, 8);
    a_load_reg_ref(list, OS_ADDR, OS_ADDR, NR_RTOC, tmpref);

    { move actual function pointer to CTR register }
    list.concat(taicpu.op_reg(A_MTCTR, tempreg));

    { load new TOC pointer from function descriptor into RTOC register }
    reference_reset_base(tmpref, reg, tcgsize2size[OS_ADDR], 8);
    a_load_ref_reg(list, OS_ADDR, OS_ADDR, tmpref, NR_RTOC);

    { load new environment pointer from function descriptor into R11 register }
    reference_reset_base(tmpref, reg, 2*tcgsize2size[OS_ADDR], 8);
    a_reg_alloc(list, NR_R11);
    a_load_ref_reg(list, OS_ADDR, OS_ADDR, tmpref, NR_R11);
    { call function }
    list.concat(taicpu.op_none(A_BCTRL));
    a_reg_dealloc(list, NR_R11);
  end else begin
    { call ptrgl helper routine which expects the pointer to the function descriptor
    in R11 }
    a_reg_alloc(list, NR_R11);
    a_load_reg_reg(list, OS_ADDR, OS_ADDR, reg, NR_R11);
    a_call_name_direct(list, A_BL, '.ptrgl', false, false, false);
    a_reg_dealloc(list, NR_R11);
  end;

  { we need to load the old RTOC from stackframe because we changed it}
  reference_reset_base(tmpref, NR_STACK_POINTER_REG, get_rtoc_offset, 8);
  a_load_ref_reg(list, OS_ADDR, OS_ADDR, tmpref, NR_RTOC);

  include(current_procinfo.flags, pi_do_call);
end;

{********************** load instructions ********************}

procedure tcgppc.a_load_const_reg(list: TAsmList; size: TCGSize; a: aint;
  reg: TRegister);

  { loads a 32 bit constant into the given register, using an optimal instruction sequence.
    This is either LIS, LI or LI+ADDIS.
    Returns true if during these operations the upper 32 bits were filled with 1 bits (e.g.
    sign extension was performed) }
  function load32bitconstant(list : TAsmList; size : TCGSize; a : longint;
    reg : TRegister) : boolean;
  var
    is_half_signed : byte;
  begin
    { if the lower 16 bits are zero, do a single LIS }
    if (smallint(a) = 0) and ((a shr 16) <> 0) then begin
      list.concat(taicpu.op_reg_const(A_LIS, reg, smallint(hi(a))));
      load32bitconstant := longint(a) < 0;
    end else begin
      is_half_signed := ord(smallint(lo(a)) < 0);
      list.concat(taicpu.op_reg_const(A_LI, reg, smallint(a and $ffff)));
      if smallint(hi(a) + is_half_signed) <> 0 then begin
        list.concat(taicpu.op_reg_reg_const(A_ADDIS, reg, reg, smallint(hi(a) + is_half_signed)));
      end;
      load32bitconstant := (smallint(a) < 0) or (a < 0);
    end;
  end;

  { loads a 32 bit constant into R0, using an optimal instruction sequence.
    This is either LIS, LI or LI+ORIS.
    Returns true if during these operations the upper 32 bits were filled with 1 bits (e.g.
    sign extension was performed) }
  function load32bitconstantR0(list : TAsmList; size : TCGSize; a : longint) : boolean;
  begin
    { if it's a value we can load with a single LI, do it }
    if (a >= low(smallint)) and (a <= high(smallint)) then begin
      list.concat(taicpu.op_reg_const(A_LI, NR_R0, smallint(a)));
    end else begin
      { if the lower 16 bits are zero, do a single LIS }
      list.concat(taicpu.op_reg_const(A_LIS, NR_R0, smallint(a shr 16)));
      if (smallint(a) <> 0) then begin
        list.concat(taicpu.op_reg_reg_const(A_ORI, NR_R0, NR_R0, word(a)));
      end;
    end;
    load32bitconstantR0 := a < 0;
  end;


  { emits the code to load a constant by emitting various instructions into the output
   code}
  procedure loadConstantNormal(list: TAsmList; size : TCgSize; a: aint; reg: TRegister);
  var
    extendssign : boolean;
    instr : taicpu;
  begin
    if (lo(a) = 0) and (hi(a) <> 0) then begin
      { load only upper 32 bits, and shift }
      load32bitconstant(list, size, longint(hi(a)), reg);
      list.concat(taicpu.op_reg_reg_const(A_SLDI, reg, reg, 32));
    end else begin
      { load lower 32 bits }
      extendssign := load32bitconstant(list, size, longint(lo(a)), reg);
      if (extendssign) and (hi(a) = 0) then
        { if upper 32 bits are zero, but loading the lower 32 bit resulted in automatic
          sign extension, clear those bits }
        list.concat(taicpu.op_reg_reg_const_const(A_RLDICL, reg, reg, 0, 32))
      else if (not
        ((extendssign and (longint(hi(a)) = -1)) or
         ((not extendssign) and (hi(a)=0)))
        ) then begin
        { only load the upper 32 bits, if the automatic sign extension is not okay,
          that is, _not_ if
          - loading the lower 32 bits resulted in -1 in the upper 32 bits, and the upper
           32 bits should contain -1
          - loading the lower 32 bits resulted in 0 in the upper 32 bits, and the upper
           32 bits should contain 0 }
        a_reg_alloc(list, NR_R0);
        load32bitconstantR0(list, size, longint(hi(a)));
        { combine both registers }
        list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, reg, NR_R0, 32, 0));
        a_reg_dealloc(list, NR_R0);
      end;
    end;
  end;

  {$IFDEF EXTDEBUG}
var
  astring : string;
  {$ENDIF EXTDEBUG}

begin
  {$IFDEF EXTDEBUG}
  astring := 'a_load_const_reg ' + inttostr(hi(a)) + ' ' + inttostr(lo(a)) + ' ' + inttostr(ord(size)) + ' ' + inttostr(tcgsize2size[size]) + ' ' + hexstr(a, 16);
  list.concat(tai_comment.create(strpnew(astring)));
  {$ENDIF EXTDEBUG}
  if not (size in [OS_8, OS_S8, OS_16, OS_S16, OS_32, OS_S32, OS_64, OS_S64]) then
    internalerror(2002090902);
  { if PIC or basic optimizations are enabled, and the number of instructions which would be
   required to load the value is greater than 2, store (and later load) the value from there }
//  if (((cs_opt_peephole in current_settings.optimizerswitches) or (cs_create_pic in current_settings.moduleswitches)) and
//    (getInstructionLength(a) > 2)) then
//    loadConstantPIC(list, size, a, reg)
//  else
    loadConstantNormal(list, size, a, reg);
end;


procedure tcgppc.a_load_ref_reg(list: TAsmList; fromsize, tosize: tcgsize;
  const ref: treference; reg: tregister);

const
  LoadInstr: array[OS_8..OS_S64, boolean, boolean] of TAsmOp =
  { indexed? updating? }
  (((A_LBZ, A_LBZU), (A_LBZX, A_LBZUX)),
    ((A_LHZ, A_LHZU), (A_LHZX, A_LHZUX)),
    ((A_LWZ, A_LWZU), (A_LWZX, A_LWZUX)),
    ((A_LD, A_LDU), (A_LDX, A_LDUX)),
    { 128bit stuff too }
    ((A_NONE, A_NONE), (A_NONE, A_NONE)),
    { there's no load-byte-with-sign-extend :( }
    ((A_LBZ, A_LBZU), (A_LBZX, A_LBZUX)),
    ((A_LHA, A_LHAU), (A_LHAX, A_LHAUX)),
    { there's no load-word-arithmetic-indexed with update, simulate it in code :( }
    ((A_LWA, A_NOP), (A_LWAX, A_LWAUX)),
    ((A_LD, A_LDU), (A_LDX, A_LDUX))
    );
var
  op: tasmop;
  ref2: treference;
  tmpreg: tregister;
begin
  if target_info.system=system_powerpc64_aix then
    g_load_check_simple(list,ref,65536);
  {$IFDEF EXTDEBUG}
  list.concat(tai_comment.create(strpnew('a_load_ref_reg ' + ref2string(ref))));
  {$ENDIF EXTDEBUG}

  if not (fromsize in [OS_8, OS_S8, OS_16, OS_S16, OS_32, OS_S32, OS_64, OS_S64]) then
    internalerror(2002090904);

  { the caller is expected to have adjusted the reference already
   in this case }
  if (TCGSize2Size[fromsize] >= TCGSize2Size[tosize]) then
    fromsize := tosize;

  ref2 := ref;
  fixref(list, ref2);

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  { unaligned 64 bit accesses are much slower than unaligned }
  { 32 bit accesses because they cause a hardware exception  }
  { (which isn't handled by linux, so there you even get a   }
  {  crash)                                                  }
  if (ref.alignment<>0) and
     (fromsize in [OS_64,OS_S64]) and
     (ref.alignment<4) then
    begin
      if (ref2.base<>NR_NO) and
         (ref2.index<>NR_NO) then
        begin
          tmpreg:=getintregister(list,OS_64);
          a_op_reg_reg_reg(list,OP_SHR,OS_64,ref2.base,ref2.index,tmpreg);
          ref2.base:=tmpreg;
          ref2.index:=NR_NO;
        end;
      tmpreg:=getintregister(list,OS_32);
      a_load_ref_reg(list,OS_32,OS_32,ref2,tmpreg);
      inc(ref2.offset,4);
      a_load_ref_reg(list,OS_32,OS_32,ref2,reg);
      list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, reg, tmpreg, 32, 0));
      exit;
    end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  op := loadinstr[fromsize, ref2.index <> NR_NO, false];
  { there is no LWAU instruction, simulate using ADDI and LWA }
  if (op = A_NOP) then begin
    list.concat(taicpu.op_reg_reg_const(A_ADDI, reg, reg, ref2.offset));
    ref2.offset := 0;
    op := A_LWA;
  end;
  a_load_store(list, op, reg, ref2);
<<<<<<< HEAD
  { sign extend shortint if necessary (because there is
   no load instruction to sign extend an 8 bit value automatically)
   and mask out extra sign bits when loading from a smaller
   signed to a larger unsigned type (where it matters) }
  if (fromsize = OS_S8) then begin
    a_load_reg_reg(list, OS_8, OS_S8, reg, reg);
    a_load_reg_reg(list, OS_S8, tosize, reg, reg);
  end else if (fromsize = OS_S16) and (tosize = OS_32) then
    a_load_reg_reg(list, fromsize, tosize, reg, reg);
=======
  { sign extend shortint if necessary, since there is no
   load instruction that does that automatically (JM) }
  if (fromsize = OS_S8) then
    begin
      list.concat(taicpu.op_reg_reg(A_EXTSB, reg, reg));
      if (tosize in [OS_16,OS_32]) then
        a_load_reg_reg(list,fromsize,tosize,reg,reg);
    end
  else if (fromsize = OS_S16) and
          (tosize = OS_32) then
    a_load_reg_reg(list,fromsize,tosize,reg,reg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure tcgppc.a_load_reg_reg(list: TAsmList; fromsize, tosize: tcgsize;
  reg1, reg2: tregister);
var
  instr: TAiCpu;
  bytesize : byte;
begin
  {$ifdef extdebug}
  list.concat(tai_comment.create(strpnew('a_load_reg_reg from : ' + cgsize2string(fromsize) + ' to ' + cgsize2string(tosize))));
  {$endif}

  if (tcgsize2size[fromsize] > tcgsize2size[tosize]) or
    ((tcgsize2size[fromsize] = tcgsize2size[tosize]) and (fromsize <> tosize)) or
    { do we need to mask out the sign when loading from smaller signed to larger unsigned type? }
    ( is_signed_cgsize(fromsize) and (not is_signed_cgsize(tosize)) and
      (tcgsize2size[fromsize] < tcgsize2size[tosize]) and (tcgsize2size[tosize] <> sizeof(pint)) ) then begin
    case tosize of
      OS_S8:
        instr := taicpu.op_reg_reg(A_EXTSB,reg2,reg1);
      OS_S16:
        instr := taicpu.op_reg_reg(A_EXTSH,reg2,reg1);
      OS_S32:
        instr := taicpu.op_reg_reg(A_EXTSW,reg2,reg1);
      OS_8, OS_16, OS_32:
        instr := taicpu.op_reg_reg_const_const(A_RLDICL, reg2, reg1, 0, (8-tcgsize2size[tosize])*8);
      OS_S64, OS_64:
        instr := taicpu.op_reg_reg(A_MR, reg2, reg1);
      else
        internalerror(2013113007);
    end;
  end else
    instr := taicpu.op_reg_reg(A_MR, reg2, reg1);

  list.concat(instr);
  rg[R_INTREGISTER].add_move_instruction(instr);
end;

<<<<<<< HEAD
=======
procedure tcgppc.a_load_subsetreg_reg(list : TAsmList; subsetsize, tosize: tcgsize; const sreg: tsubsetregister; destreg: tregister);
begin
  {$ifdef extdebug}
  list.concat(tai_comment.create(strpnew('a_load_subsetreg_reg subsetregsize = ' + cgsize2string(sreg.subsetregsize) + ' subsetsize = ' + cgsize2string(subsetsize) + ' startbit = ' + intToStr(sreg.startbit) + ' tosize = ' + cgsize2string(tosize))));
  {$endif}
  { do the extraction if required and then extend the sign correctly. (The latter is actually required only for signed subsets
  and if that subset is not >= the tosize). }
  if (sreg.startbit <> 0) or
     (sreg.bitlen <> tcgsize2size[subsetsize]*8) then begin
    list.concat(taicpu.op_reg_reg_const_const(A_RLDICL, destreg, sreg.subsetreg, (64 - sreg.startbit) and 63, 64 - sreg.bitlen));
    if (subsetsize in [OS_S8..OS_S128]) then
      if ((sreg.bitlen mod 8) = 0) then begin
        a_load_reg_reg(list, tcgsize2unsigned[subsetsize], subsetsize, destreg, destreg);
        a_load_reg_reg(list, subsetsize, tosize, destreg, destreg);
      end else begin
        a_op_const_reg(list,OP_SHL,OS_INT,64-sreg.bitlen,destreg);
        a_op_const_reg(list,OP_SAR,OS_INT,64-sreg.bitlen,destreg);
     end;
  end else begin
    a_load_reg_reg(list, tcgsize2unsigned[sreg.subsetregsize], subsetsize, sreg.subsetreg, destreg);
    a_load_reg_reg(list, subsetsize, tosize, destreg, destreg);
  end;
end;

procedure tcgppc.a_load_regconst_subsetreg_intern(list : TAsmList; fromsize, subsetsize: tcgsize; fromreg: tregister; const sreg: tsubsetregister; slopt: tsubsetloadopt);
begin
  {$ifdef extdebug}
  list.concat(tai_comment.create(strpnew('a_load_reg_subsetreg fromsize = ' + cgsize2string(fromsize) + ' subsetregsize = ' + cgsize2string(sreg.subsetregsize) + ' subsetsize = ' + cgsize2string(subsetsize) + ' startbit = ' + IntToStr(sreg.startbit))));
  {$endif}
  if (slopt in [SL_SETZERO,SL_SETMAX]) then
    inherited a_load_regconst_subsetreg_intern(list,fromsize,subsetsize,fromreg,sreg,slopt)
  else if (sreg.bitlen <> sizeof(aint)*8) then
    { simply use the INSRDI instruction }
    list.concat(taicpu.op_reg_reg_const_const(A_INSRDI, sreg.subsetreg, fromreg, sreg.bitlen, (64 - (sreg.startbit + sreg.bitlen)) and 63))
  else
    a_load_reg_reg(list, fromsize, subsetsize, fromreg, sreg.subsetreg);
end;

procedure tcgppc.a_load_const_subsetreg(list: TAsmlist; subsetsize: tcgsize;
  a: aint; const sreg: tsubsetregister);
var
  tmpreg : TRegister;
begin
  {$ifdef extdebug}
  list.concat(tai_comment.create(strpnew('a_load_const_subsetreg subsetregsize = ' + cgsize2string(sreg.subsetregsize) + ' subsetsize = ' + cgsize2string(subsetsize) + ' startbit = ' + intToStr(sreg.startbit) + ' a = ' + intToStr(a))));
  {$endif}
  { loading the constant into the lowest bits of a temp register and then inserting is
    better than loading some usually large constants and do some masking and shifting on ppc64 }
  tmpreg := getintregister(list,subsetsize);
  a_load_const_reg(list,subsetsize,a,tmpreg);
  a_load_reg_subsetreg(list, subsetsize, subsetsize, tmpreg, sreg);
end;

>>>>>>> graemeg/fixes_2_2
procedure tcgppc.a_op_const_reg(list: TAsmList; Op: TOpCG; size: TCGSize; a:
  aint; reg: TRegister);
begin
  a_op_const_reg_reg(list, op, size, a, reg, reg);
end;

procedure tcgppc.a_op_reg_reg(list: TAsmList; Op: TOpCG; size: TCGSize; src,
  dst: TRegister);
begin
  a_op_reg_reg_reg(list, op, size, src, dst, dst);
end;

procedure tcgppc.a_op_const_reg_reg(list: TAsmList; op: TOpCg;
  size: tcgsize; a: aint; src, dst: tregister);
var
  useReg : boolean;

  procedure do_lo_hi(loOp, hiOp : TAsmOp);
  begin
    { Optimization for logical ops (excluding AND), trying to do this as efficiently
     as possible by only generating code for the affected halfwords. Note that all
     the instructions handled here must have "X op 0 = X" for every halfword. }
    usereg := false;
    if (aword(a) > high(dword)) then begin
      usereg := true;
    end else begin
      if (word(a) <> 0) then begin
        list.concat(taicpu.op_reg_reg_const(loOp, dst, src, word(a)));
        if (word(a shr 16) <> 0) then
          list.concat(taicpu.op_reg_reg_const(hiOp, dst, dst, word(a shr 16)));
      end else if (word(a shr 16) <> 0) then
        list.concat(taicpu.op_reg_reg_const(hiOp, dst, src, word(a shr 16)));
    end;
  end;

  procedure do_lo_hi_and;
  begin
    { optimization logical and with immediate: only use "andi." for 16 bit
     ands, otherwise use register method. Doing this for 32 bit constants
     would not give any advantage to the register method (via useReg := true),
     requiring a scratch register and three instructions. }
    usereg := false;
    if (aword(a) > high(word)) then
      usereg := true
    else
      list.concat(taicpu.op_reg_reg_const(A_ANDI_, dst, src, word(a)));
  end;

  procedure do_constant_div(list : TAsmList; size : TCgSize; a : aint; src, dst : TRegister;
    signed : boolean);
  const
    negops : array[boolean] of tasmop = (A_NEG, A_NEGO);
  var
    magic : int64;
    u_magic : qword;
    u_shift : byte;
    u_add : boolean;
    power : byte;
    isNegPower : boolean;

    divreg : tregister;
  begin
    if (a = 0) then begin
      internalerror(2005061701);
    end else if (a = 1) then begin
      a_load_reg_reg(list, OS_INT, OS_INT, src, dst);
    end else if (a = -1) and (signed) then begin
      { note: only in the signed case possible..., may overflow }
      list.concat(taicpu.op_reg_reg(negops[cs_check_overflow in current_settings.localswitches], dst, src));
    end else if (ispowerof2(a, power, isNegPower)) then begin
      if (signed) then begin
        { From "The PowerPC Compiler Writer's Guide", pg. 52ff          }
        a_op_const_reg_reg(list, OP_SAR, OS_INT, power,
          src, dst);
        list.concat(taicpu.op_reg_reg(A_ADDZE, dst, dst));
        if (isNegPower) then
          list.concat(taicpu.op_reg_reg(A_NEG, dst, dst));
      end else begin
        a_op_const_reg_reg(list, OP_SHR, OS_INT, power, src, dst)
      end;
    end else begin
      { replace division by multiplication, both implementations }
      { from "The PowerPC Compiler Writer's Guide" pg. 53ff      }
      divreg := getintregister(list, OS_INT);
      if (signed) then begin
        calc_divconst_magic_signed(sizeof(aInt)*8, a, magic, u_shift);
        { load magic value }
        a_load_const_reg(list, OS_INT, magic, divreg);
        { multiply }
        list.concat(taicpu.op_reg_reg_reg(A_MULHD, dst, src, divreg));
        { add/subtract numerator }
        if (a > 0) and (magic < 0) then begin
          a_op_reg_reg_reg(list, OP_ADD, OS_INT, src, dst, dst);
        end else if (a < 0) and (magic > 0) then begin
          a_op_reg_reg_reg(list, OP_SUB, OS_INT, src, dst, dst);
        end;
        { shift shift places to the right (arithmetic) }
        a_op_const_reg_reg(list, OP_SAR, OS_INT, u_shift, dst, dst);
        { extract and add sign bit }
        if (a >= 0) then begin
          a_op_const_reg_reg(list, OP_SHR, OS_INT, 63, src, divreg);
        end else begin
          a_op_const_reg_reg(list, OP_SHR, OS_INT, 63, dst, divreg);
        end;
        a_op_reg_reg_reg(list, OP_ADD, OS_INT, dst, divreg, dst);
      end else begin
        calc_divconst_magic_unsigned(sizeof(aWord)*8, a, u_magic, u_add, u_shift);
        { load magic in divreg }
<<<<<<< HEAD
<<<<<<< HEAD
        a_load_const_reg(list, OS_INT, aint(u_magic), divreg);
        list.concat(taicpu.op_reg_reg_reg(A_MULHDU, dst, src, divreg));
=======
=======
>>>>>>> origin/fixes_2_2
        cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, aint(u_magic), divreg);
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULHDU, dst, src, divreg));
>>>>>>> graemeg/fixes_2_2
        if (u_add) then begin
          a_op_reg_reg_reg(list, OP_SUB, OS_INT, dst, src, divreg);
          a_op_const_reg_reg(list, OP_SHR, OS_INT,  1, divreg, divreg);
          a_op_reg_reg_reg(list, OP_ADD, OS_INT, divreg, dst, divreg);
          a_op_const_reg_reg(list, OP_SHR, OS_INT, u_shift-1, divreg, dst);
        end else begin
          a_op_const_reg_reg(list, OP_SHR, OS_INT, u_shift, dst, dst);
        end;
      end;
    end;
  end;

var
  scratchreg: tregister;
  shift : byte;
  shiftmask : longint;
  isneg : boolean;

begin
  { subtraction is the same as addition with negative constant }
  if op = OP_SUB then begin
    a_op_const_reg_reg(list, OP_ADD, size, -a, src, dst);
    exit;
  end;
  {$IFDEF EXTDEBUG}
  list.concat(tai_comment.create(strpnew('a_op_const_reg_reg ' + cgop2string(op))));
  {$ENDIF EXTDEBUG}

  { This case includes some peephole optimizations for the various operations,
   (e.g. AND, OR, XOR, ..) - can't this be done at some higher level,
   independent of architecture? }

  { assume that we do not need a scratch register for the operation }
  useReg := false;
  case (op) of
    OP_DIV, OP_IDIV:
      if (cs_opt_level1 in current_settings.optimizerswitches) then
        do_constant_div(list, size, a, src, dst, op = OP_IDIV)
      else
        usereg := true;
    OP_IMUL, OP_MUL:
      { idea: factorize constant multiplicands and use adds/shifts with few factors;
       however, even a 64 bit multiply is already quite fast on PPC64 }
      if (a = 0) then
        a_load_const_reg(list, size, 0, dst)
      else if (a = -1) then
        list.concat(taicpu.op_reg_reg(A_NEG, dst, dst))
      else if (a = 1) then
        a_load_reg_reg(list, OS_INT, OS_INT, src, dst)
      else if ispowerof2(a, shift, isneg) then begin
        list.concat(taicpu.op_reg_reg_const(A_SLDI, dst, src, shift));
        if (isneg) then
          list.concat(taicpu.op_reg_reg(A_NEG, dst, dst));
      end else if (a >= low(smallint)) and (a <= high(smallint)) then
        list.concat(taicpu.op_reg_reg_const(A_MULLI, dst, src,
          smallint(a)))
      else
        usereg := true;
    OP_ADD:
      if (a = 0) then
        a_load_reg_reg(list, size, size, src, dst)
      else if (a >= low(smallint)) and (a <= high(smallint)) then
        list.concat(taicpu.op_reg_reg_const(A_ADDI, dst, src, smallint(a)))
      else
        useReg := true;
    OP_OR:
      if (a = 0) then
        a_load_reg_reg(list, size, size, src, dst)
      else if (a = -1) then
        a_load_const_reg(list, size, -1, dst)
      else
        do_lo_hi(A_ORI, A_ORIS);
    OP_AND:
      if (a = 0) then
        a_load_const_reg(list, size, 0, dst)
      else if (a = -1) then
        a_load_reg_reg(list, size, size, src, dst)
      else
        do_lo_hi_and;
    OP_XOR:
      if (a = 0) then
        a_load_reg_reg(list, size, size, src, dst)
      else if (a = -1) then
        list.concat(taicpu.op_reg_reg(A_NOT, dst, src))
      else
        do_lo_hi(A_XORI, A_XORIS);
    OP_ROL:
      begin
        if (size in [OS_64, OS_S64]) then begin
	  list.concat(taicpu.op_reg_reg_const_const(A_RLDICL, dst, src, a and 63, 0));
	end else if (size in [OS_32, OS_S32]) then begin
	  list.concat(taicpu.op_reg_reg_const_const_const(A_RLWINM, dst, src, a and 31, 0, 31));
	end else begin
	  internalerror(2008091303);
	end;
      end;
    OP_ROR:
      begin
        if (size in [OS_64, OS_S64]) then begin
	  list.concat(taicpu.op_reg_reg_const_const(A_RLDICL, dst, src, ((64 - a) and 63), 0));
	end else if (size in [OS_32, OS_S32]) then begin
	  list.concat(taicpu.op_reg_reg_const_const_const(A_RLWINM, dst, src, (32 - a) and 31, 0, 31));
	end else begin
	  internalerror(2008091304);
	end;
      end;
    OP_SHL, OP_SHR, OP_SAR:
      begin
        if (size in [OS_64, OS_S64]) then
          shift := 6
        else
          shift := 5;

        shiftmask := (1 shl shift)-1;
        if (a and shiftmask) <> 0 then begin
          list.concat(taicpu.op_reg_reg_const(
            TShiftOpCG2AsmOpConst[size in [OS_64, OS_S64], op], dst, src, a and shiftmask));
        end else
          a_load_reg_reg(list, size, size, src, dst);
        if ((a shr shift) <> 0) then
          internalError(68991);
      end
    else
      internalerror(200109091);
  end;
  { if all else failed, load the constant in a register and then
   perform the operation }
  if (useReg) then begin
    scratchreg := rg[R_INTREGISTER].getregister(list, R_SUBWHOLE);
    a_load_const_reg(list, size, a, scratchreg);
    a_op_reg_reg_reg(list, op, size, scratchreg, src, dst);
  end else
    maybeadjustresult(list, op, size, dst);
end;

procedure tcgppc.a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
  size: tcgsize; src1, src2, dst: tregister);
const
  op_reg_reg_opcg2asmop32: array[TOpCG] of tasmop =
  (A_NONE, A_MR, A_ADD, A_AND, A_DIVWU, A_DIVW, A_MULLW, A_MULLW, A_NEG, A_NOT, A_OR,
   A_SRAW, A_SLW, A_SRW, A_SUB, A_XOR, A_NONE, A_NONE);
  op_reg_reg_opcg2asmop64: array[TOpCG] of tasmop =
  (A_NONE, A_MR, A_ADD, A_AND, A_DIVDU, A_DIVD, A_MULLD, A_MULLD, A_NEG, A_NOT, A_OR,
   A_SRAD, A_SLD, A_SRD, A_SUB, A_XOR, A_NONE, A_NONE);
var
  tmpreg : TRegister;
begin
  case op of
    OP_NEG, OP_NOT:
      begin
        list.concat(taicpu.op_reg_reg(op_reg_reg_opcg2asmop64[op], dst, src1));
        if (op = OP_NOT) and not (size in [OS_64, OS_S64]) then
          { zero/sign extend result again, fromsize is not important here }
          a_load_reg_reg(list, OS_S64, size, dst, dst)
      end;
    OP_ROL:
      begin
        if (size in [OS_64, OS_S64]) then begin
	  list.concat(taicpu.op_reg_reg_reg_const(A_RLDCL, dst, src2, src1, 0));
	end else if (size in [OS_32, OS_S32]) then begin
	  list.concat(taicpu.op_reg_reg_reg_const_const(A_RLWNM, dst, src2, src1, 0, 31));
	end else begin
	  internalerror(2008091301);
	end;
      end;
    OP_ROR:
      begin
        tmpreg := getintregister(list, OS_INT);
	list.concat(taicpu.op_reg_reg(A_NEG, tmpreg, src1));
        if (size in [OS_64, OS_S64]) then begin
	  list.concat(taicpu.op_reg_reg_reg_const(A_RLDCL, dst, src2, tmpreg, 0));
	end else if (size in [OS_32, OS_S32]) then begin
	  list.concat(taicpu.op_reg_reg_reg_const_const(A_RLWNM, dst, src2, tmpreg, 0, 31));
	end else begin
	  internalerror(2008091302);
	end;
      end;
    else
      if (size in [OS_64, OS_S64]) then begin
        list.concat(taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop64[op], dst, src2,
          src1));
      end else begin
        list.concat(taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop32[op], dst, src2,
          src1));
        maybeadjustresult(list, op, size, dst);
      end;
  end;
end;

{*************** compare instructructions ****************}

procedure tcgppc.a_cmp_const_reg_label(list: TAsmList; size: tcgsize;
  cmp_op: topcmp; a: aint; reg: tregister; l: tasmlabel);
const
  {                  unsigned  useconst  32bit-op }
  cmpop_table : array[boolean, boolean, boolean] of TAsmOp = (
    ((A_CMPD, A_CMPW), (A_CMPDI, A_CMPWI)),
    ((A_CMPLD, A_CMPLW), (A_CMPLDI, A_CMPLWI))
   );

var
  tmpreg : TRegister;
  signed, useconst : boolean;
  opsize : TCgSize;
  op : TAsmOp;
begin
  {$IFDEF EXTDEBUG}
  list.concat(tai_comment.create(strpnew('a_cmp_const_reg_label ' + cgsize2string(size) + ' ' + booltostr(cmp_op in [OC_GT, OC_LT, OC_GTE, OC_LTE]) + ' ' + inttostr(a) )));
  {$ENDIF EXTDEBUG}

  signed := cmp_op in [OC_GT, OC_LT, OC_GTE, OC_LTE];
  { in the following case, we generate more efficient code when
    signed is true }
  if (cmp_op in [OC_EQ, OC_NE]) and
    (aword(a) > $FFFF) then
    signed := true;

  opsize := size;

  { do we need to change the operand size because ppc64 only supports 32 and
    64 bit compares? }
  if (not (size in [OS_32, OS_S32, OS_64, OS_S64])) then begin
    if (signed) then
      opsize := OS_S32
    else
      opsize := OS_32;
    a_load_reg_reg(list, size, opsize, reg, reg);
  end;

  { can we use immediate compares? }
  useconst := (signed and ( (a >= low(smallint)) and (a <= high(smallint)))) or
    ((not signed) and (aword(a) <= $FFFF));

  op := cmpop_table[not signed, useconst, opsize in [OS_32, OS_S32]];

  if (useconst) then begin
    list.concat(taicpu.op_reg_reg_const(op, NR_CR0, reg, a));
  end else begin
    tmpreg := getintregister(list, OS_INT);
    a_load_const_reg(list, opsize, a, tmpreg);
    list.concat(taicpu.op_reg_reg_reg(op, NR_CR0, reg, tmpreg));
  end;

  a_jmp(list, A_BC, TOpCmp2AsmCond[cmp_op], 0, l);
end;

procedure tcgppc.a_cmp_reg_reg_label(list: TAsmList; size: tcgsize;
  cmp_op: topcmp; reg1, reg2: tregister; l: tasmlabel);
var
  op: tasmop;
begin
  {$IFDEF extdebug}
  list.concat(tai_comment.create(strpnew('a_cmp_reg_reg_label, size ' + cgsize2string(size) + ' op ' + inttostr(ord(cmp_op)))));
  {$ENDIF extdebug}

  {$note Commented out below check because of compiler weirdness}
  {
  if (not (size in [OS_32, OS_S32, OS_64, OS_S64])) then
    internalerror(200606041);
  }

  if cmp_op in [OC_GT, OC_LT, OC_GTE, OC_LTE] then
    if (size in [OS_64, OS_S64]) then
      op := A_CMPD
    else
      op := A_CMPW
  else
    if (size in [OS_64, OS_S64]) then
      op := A_CMPLD
    else
      op := A_CMPLW;
  list.concat(taicpu.op_reg_reg_reg(op, NR_CR0, reg2, reg1));
  a_jmp(list, A_BC, TOpCmp2AsmCond[cmp_op], 0, l);
end;

procedure tcgppc.a_jmp_name_direct(list : TAsmList; opc: tasmop; s : string; prependDot : boolean);
var
  p: taicpu;
begin
  if (prependDot) then
    s := '.' + s;
  p := taicpu.op_sym(opc, current_asmdata.RefAsmSymbol(s));
  p.is_jmp := true;
  list.concat(p)
end;

procedure tcgppc.a_jmp_name(list: TAsmList; const s: string);
var
  p: taicpu;
begin
  if (target_info.system = system_powerpc64_darwin) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      p := taicpu.op_sym(A_B,get_darwin_call_stub(s,false));
=======
      p := taicpu.op_sym(A_B,get_darwin_call_stub(s));
>>>>>>> graemeg/fixes_2_2
=======
      p := taicpu.op_sym(A_B,get_darwin_call_stub(s));
>>>>>>> origin/fixes_2_2
      p.is_jmp := true;
      list.concat(p)
    end
  else
<<<<<<< HEAD
<<<<<<< HEAD
    a_jmp_name_direct(list, A_B, s, true);
=======
    a_jmp_name_direct(list, s, true);
>>>>>>> graemeg/fixes_2_2
=======
    a_jmp_name_direct(list, s, true);
>>>>>>> origin/fixes_2_2
end;

procedure tcgppc.a_jmp_always(list: TAsmList; l: tasmlabel);

begin
  a_jmp(list, A_B, C_None, 0, l);
end;

{ *********** entry/exit code and address loading ************ }

procedure tcgppc.g_save_registers(list: TAsmList);
begin
  { this work is done in g_proc_entry; additionally it is not safe
  to use it because it is called at some weird time }
end;

procedure tcgppc.g_restore_registers(list: TAsmList);
begin
  { this work is done in g_proc_exit; mainly because it is not safe to
  put the register restore code here because it is called at some weird time }
end;

procedure tcgppc.calcFirstUsedFPR(out firstfpr : TSuperRegister; out fprcount : aint);
var
  reg : TSuperRegister;
begin
  fprcount := 0;
  firstfpr := RS_F31;
  if not (po_assembler in current_procinfo.procdef.procoptions) then
    for reg := RS_F14 to RS_F31 do
      if reg in rg[R_FPUREGISTER].used_in_proc then begin
        fprcount := ord(RS_F31)-ord(reg)+1;
        firstfpr := reg;
        break;
      end;
end;

procedure tcgppc.calcFirstUsedGPR(out firstgpr : TSuperRegister; out gprcount : aint);
var
  reg : TSuperRegister;
begin
  gprcount := 0;
  firstgpr := RS_R31;
  if not (po_assembler in current_procinfo.procdef.procoptions) then
    for reg := RS_R14 to RS_R31 do
      if reg in rg[R_INTREGISTER].used_in_proc then begin
        gprcount := ord(RS_R31)-ord(reg)+1;
        firstgpr := reg;
        break;
      end;
end;

procedure tcgppc.profilecode_savepara(para : tparavarsym; list : TAsmList);
begin
  case (para.paraloc[calleeside].location^.loc) of
    LOC_REGISTER, LOC_CREGISTER:
      a_load_reg_ref(list, OS_INT, para.paraloc[calleeside].Location^.size,
        para.paraloc[calleeside].Location^.register, para.localloc.reference);
    LOC_FPUREGISTER, LOC_CFPUREGISTER:
      a_loadfpu_reg_ref(list, para.paraloc[calleeside].Location^.size,
        para.paraloc[calleeside].Location^.size,
        para.paraloc[calleeside].Location^.register, para.localloc.reference);
    LOC_MMREGISTER, LOC_CMMREGISTER:
      { not supported }
      internalerror(2006041801);
  end;
end;

procedure tcgppc.profilecode_restorepara(para : tparavarsym; list : TAsmList);
begin
  case (para.paraloc[calleeside].Location^.loc) of
    LOC_REGISTER, LOC_CREGISTER:
      a_load_ref_reg(list, para.paraloc[calleeside].Location^.size, OS_INT,
        para.localloc.reference, para.paraloc[calleeside].Location^.register);
    LOC_FPUREGISTER, LOC_CFPUREGISTER:
      a_loadfpu_ref_reg(list, para.paraloc[calleeside].Location^.size,
        para.paraloc[calleeside].Location^.size,
        para.localloc.reference, para.paraloc[calleeside].Location^.register);
    LOC_MMREGISTER, LOC_CMMREGISTER:
      { not supported }
      internalerror(2006041802);
  end;
end;

procedure tcgppc.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
var
  hsym : tsym;
  href : treference;
  paraloc : Pcgparalocation;
begin
  if ((ioffset >= low(smallint)) and (ioffset < high(smallint))) then begin
    { the original method can handle this }
    inherited g_adjust_self_value(list, procdef, ioffset);
    exit;
  end;
  { calculate the parameter info for the procdef }
  procdef.init_paraloc_info(callerside);
  hsym:=tsym(procdef.parast.Find('self'));
  if not(assigned(hsym) and
    (hsym.typ=paravarsym)) then
    internalerror(2010103101);
  paraloc:=tparavarsym(hsym).paraloc[callerside].location;
  while paraloc<>nil do
    with paraloc^ do begin
      case loc of
        LOC_REGISTER:
        begin
          a_load_const_reg(list, size, ioffset, NR_R11);
          a_op_reg_reg(list, OP_SUB, size, NR_R11, register);
        end else
          internalerror(2010103102);
      end;
      paraloc:=next;
    end;
end;

procedure tcgppc.g_profilecode(list: TAsmList);
begin
  current_procinfo.procdef.paras.ForEachCall(TObjectListCallback(@profilecode_savepara), list);

  a_call_name_direct(list, A_BL, '_mcount', false, false, true);

  current_procinfo.procdef.paras.ForEachCall(TObjectListCallback(@profilecode_restorepara), list);
end;

{ Generates the entry code of a procedure/function.

 This procedure may be called before, as well as after g_return_from_proc
 is called. localsize is the sum of the size necessary for local variables
 and the maximum possible combined size of ALL the parameters of a procedure
 called by the current one

 IMPORTANT: registers are not to be allocated through the register
 allocator here, because the register colouring has already occured !!
}
procedure tcgppc.g_proc_entry(list: TAsmList; localsize: longint;
  nostackframe: boolean);
var
  firstregfpu, firstreggpr: TSuperRegister;
  needslinkreg: boolean;

  fprcount, gprcount : aint;

  { Save standard registers, both FPR and GPR; does not support VMX/Altivec }
  procedure save_standard_registers;
  var
    regcount : TSuperRegister;
    href : TReference;
    mayNeedLRStore : boolean;
    opc : tasmop;
  begin
    { there are two ways to do this: manually, by generating a few "std" instructions,
     or via the restore helper functions. The latter are selected by the -Og switch,
     i.e. "optimize for size" }
    if (cs_opt_size in current_settings.optimizerswitches) and
       (target_info.system <> system_powerpc64_darwin) then begin
      mayNeedLRStore := false;
      if target_info.system=system_powerpc64_aix then
        opc:=A_BLA
      else
        opc:=A_BL;
      if ((fprcount > 0) and (gprcount > 0)) then begin
        a_op_const_reg_reg(list, OP_SUB, OS_INT, 8 * fprcount, NR_R1, NR_R12);
        a_call_name_direct(list, opc, '_savegpr1_' + intToStr(32-gprcount), false, false, false, false);
        a_call_name_direct(list, opc, '_savefpr_' + intToStr(32-fprcount), false, false, false, false);
      end else if (gprcount > 0) then
        a_call_name_direct(list, opc, '_savegpr0_' + intToStr(32-gprcount), false, false, false, false)
      else if (fprcount > 0) then
        a_call_name_direct(list, opc, '_savefpr_' + intToStr(32-fprcount), false, false, false, false)
      else
        mayNeedLRStore := true;
    end else begin
      { save registers, FPU first, then GPR }
      reference_reset_base(href, NR_STACK_POINTER_REG, -8, 8);
      if (fprcount > 0) then
        for regcount := RS_F31 downto firstregfpu do begin
          a_loadfpu_reg_ref(list, OS_FLOAT, OS_FLOAT, newreg(R_FPUREGISTER,
            regcount, R_SUBNONE), href);
          dec(href.offset, tcgsize2size[OS_FLOAT]);
        end;
      if (gprcount > 0) then
        for regcount := RS_R31 downto firstreggpr do begin
          a_load_reg_ref(list, OS_INT, OS_INT, newreg(R_INTREGISTER, regcount,
            R_SUBNONE), href);
          dec(href.offset, sizeof(pint));
        end;
      { VMX registers not supported by FPC atm }

      { in this branch we always need to store LR ourselves}
      mayNeedLRStore := true;
    end;

    { we may need to store R0 (=LR) ourselves }
    if ((cs_profile in init_settings.moduleswitches) or (mayNeedLRStore)) and (needslinkreg) then begin
      reference_reset_base(href, NR_STACK_POINTER_REG, LA_LR_SYSV, 8);
      list.concat(taicpu.op_reg_ref(A_STD, NR_R0, href));
    end;
  end;

var
  href: treference;
  lab: tasmlabel;
  procmangledname: TSymStr;
begin
  { In ELFv2 the function is required to initialise the TOC register itself
    if necessary. Additionally, it has to mark the end of this TOC
    initialisation code with a .localfunc directive, which will be used as
    local entry code by the linker (when it knows the TOC value is the same
    for the caller and callee). It must load the TOC in a PIC-way, which it
    can do easily because R12 is guaranteed to hold the address of this function
    on entry. }
  if (target_info.abi=abi_powerpc_elfv2) and
     (pi_needs_got in current_procinfo.flags) and
     not nostackframe then
    begin
      current_asmdata.getlabel(lab,alt_addr);
      getcpuregister(list,NR_R12);
      getcpuregister(list,NR_R2);
      cg.a_label(list,lab);
      reference_reset_symbol(href,current_asmdata.RefAsmSymbol('.TOC.',AT_DATA),0,sizeof(PInt));
      href.relsymbol:=lab;
      href.refaddr:=addr_higha;
      list.concat(taicpu.op_reg_reg_ref(a_addis,NR_R2,NR_R12,href));
      href.refaddr:=addr_low;
      list.concat(taicpu.op_reg_reg_ref(a_addi,NR_R2,NR_R2,href));
      procmangledname:=current_procinfo.procdef.mangledname;
      list.concat(tai_symbolpair.create(spk_localentry,procmangledname,procmangledname));
    end;
  calcFirstUsedFPR(firstregfpu, fprcount);
  calcFirstUsedGPR(firstreggpr, gprcount);

  { calculate real stack frame size }
  localsize := tppcprocinfo(current_procinfo).calc_stackframe_size(
    gprcount, fprcount);

  { determine whether we need to save the link register }
  needslinkreg :=
    not(nostackframe) and
    (save_lr_in_prologue or
     ((cs_opt_size in current_settings.optimizerswitches) and
      ((fprcount > 0) or
       (gprcount > 0))));

  a_reg_alloc(list, NR_STACK_POINTER_REG);
  a_reg_alloc(list, NR_R0);

  { move link register to r0 }
  if (needslinkreg) then
    list.concat(taicpu.op_reg(A_MFLR, NR_R0));

  save_standard_registers;

  { save old stack frame pointer }
  if (tppcprocinfo(current_procinfo).needs_frame_pointer) then
    list.concat(taicpu.op_reg_reg(A_MR, NR_OLD_STACK_POINTER_REG, NR_STACK_POINTER_REG));

  { create stack frame }
  if (not nostackframe) and (localsize > 0) and
     tppcprocinfo(current_procinfo).needstackframe then begin
    if (localsize <= high(smallint)) then begin
      reference_reset_base(href, NR_STACK_POINTER_REG, -localsize, 8);
      a_load_store(list, A_STDU, NR_STACK_POINTER_REG, href);
    end else begin
      reference_reset_base(href, NR_NO, -localsize, 8);

      { Use R0 for loading the constant (which is definitely > 32k when entering
       this branch).

       Inlined at this position because it must not use temp registers because
       register allocations have already been done  }
      { Code template:
      lis   r0,ofs@highest
      ori   r0,r0,ofs@higher
      sldi  r0,r0,32
      oris  r0,r0,ofs@h
      ori   r0,r0,ofs@l
      }
      list.concat(taicpu.op_reg_const(A_LIS, NR_R0, word(href.offset shr 48)));
      list.concat(taicpu.op_reg_reg_const(A_ORI, NR_R0, NR_R0, word(href.offset shr 32)));
      list.concat(taicpu.op_reg_reg_const(A_SLDI, NR_R0, NR_R0, 32));
      list.concat(taicpu.op_reg_reg_const(A_ORIS, NR_R0, NR_R0, word(href.offset shr 16)));
      list.concat(taicpu.op_reg_reg_const(A_ORI, NR_R0, NR_R0, word(href.offset)));

      list.concat(taicpu.op_reg_reg_reg(A_STDUX, NR_R1, NR_R1, NR_R0));
    end;
  end;

  { CR register not used by FPC atm }

  { keep R1 allocated??? }
  a_reg_dealloc(list, NR_R0);
end;

{ Generates the exit code for a method.

 This procedure may be called before, as well as after g_stackframe_entry
 is called.

 IMPORTANT: registers are not to be allocated through the register
 allocator here, because the register colouring has already occured !!
}
procedure tcgppc.g_proc_exit(list: TAsmList; parasize: longint; nostackframe:
  boolean);
var
  firstregfpu, firstreggpr: TSuperRegister;
  needslinkreg : boolean;
  fprcount, gprcount: aint;

  { Restore standard registers, both FPR and GPR; does not support VMX/Altivec }
  procedure restore_standard_registers;
  var
    { flag indicating whether we need to manually add the exit code (e.g. blr instruction)
     or not }
    needsExitCode : Boolean;
    href : treference;
    regcount : TSuperRegister;
    callopc,
    jmpopc: tasmop;
  begin
    { there are two ways to do this: manually, by generating a few "ld" instructions,
     or via the restore helper functions. The latter are selected by the -Og switch,
     i.e. "optimize for size" }
    if (cs_opt_size in current_settings.optimizerswitches) then begin
      if target_info.system=system_powerpc64_aix then begin
        callopc:=A_BLA;
        jmpopc:=A_BA;
      end
      else begin
        callopc:=A_BL;
        jmpopc:=A_B;
      end;
      needsExitCode := false;
      if ((fprcount > 0) and (gprcount > 0)) then begin
        a_op_const_reg_reg(list, OP_SUB, OS_INT, 8 * fprcount, NR_R1, NR_R12);
        a_call_name_direct(list, callopc, '_restgpr1_' + intToStr(32-gprcount), false, false, false, false);
        a_jmp_name_direct(list, jmpopc, '_restfpr_' + intToStr(32-fprcount), false);
      end else if (gprcount > 0) then
        a_jmp_name_direct(list, jmpopc, '_restgpr0_' + intToStr(32-gprcount), false)
      else if (fprcount > 0) then
        a_jmp_name_direct(list, jmpopc, '_restfpr_' + intToStr(32-fprcount), false)
      else
        needsExitCode := true;
    end else begin
      needsExitCode := true;
      { restore registers, FPU first, GPR next }
      reference_reset_base(href, NR_STACK_POINTER_REG, -tcgsize2size[OS_FLOAT], 8);
      if (fprcount > 0) then
        for regcount := RS_F31 downto firstregfpu do begin
          a_loadfpu_ref_reg(list, OS_FLOAT, OS_FLOAT, href, newreg(R_FPUREGISTER, regcount,
            R_SUBNONE));
          dec(href.offset, tcgsize2size[OS_FLOAT]);
        end;
      if (gprcount > 0) then
        for regcount := RS_R31 downto firstreggpr do begin
          a_load_ref_reg(list, OS_INT, OS_INT, href, newreg(R_INTREGISTER, regcount,
            R_SUBNONE));
          dec(href.offset, sizeof(pint));
        end;

      { VMX not supported by FPC atm }
    end;

    if (needsExitCode) then begin

      { restore LR (if needed) }
      if (needslinkreg) then begin
        reference_reset_base(href, NR_STACK_POINTER_REG, LA_LR_SYSV, 8);
        list.concat(taicpu.op_reg_ref(A_LD, NR_R0, href));
        list.concat(taicpu.op_reg(A_MTLR, NR_R0));
      end;

      { generate return instruction }
      list.concat(taicpu.op_none(A_BLR));
    end;
  end;

var
  href: treference;
  localsize : aint;

begin
  calcFirstUsedFPR(firstregfpu, fprcount);
  calcFirstUsedGPR(firstreggpr, gprcount);

  { determine whether we need to restore the link register }
  needslinkreg :=
    not(nostackframe) and
    (((not (po_assembler in current_procinfo.procdef.procoptions)) and
       ((pi_do_call in current_procinfo.flags) or (cs_profile in init_settings.moduleswitches))) or
     ((cs_opt_size in current_settings.optimizerswitches) and ((fprcount > 0) or (gprcount > 0))) or
     ([cs_lineinfo, cs_debuginfo] * current_settings.moduleswitches <> []));

  { calculate stack frame }
  localsize := tppcprocinfo(current_procinfo).calc_stackframe_size(
    gprcount, fprcount);
  { CR register not supported }

  { restore stack pointer }
  if (not nostackframe) and (localsize > 0) and
    tppcprocinfo(current_procinfo).needstackframe then begin
    if (localsize <= high(smallint)) then begin
      list.concat(taicpu.op_reg_reg_const(A_ADDI, NR_STACK_POINTER_REG, NR_STACK_POINTER_REG, localsize));
    end else begin
      reference_reset_base(href, NR_NO, localsize, 8);

      { use R0 for loading the constant (which is definitely > 32k when entering
       this branch)
       Inlined because it must not use temp registers because register allocations
       have already been done
      }
      { Code template:
       lis   r0,ofs@highest
       ori   r0,ofs@higher
       sldi  r0,r0,32
       oris  r0,r0,ofs@h
       ori   r0,r0,ofs@l
      }
      list.concat(taicpu.op_reg_const(A_LIS, NR_R0, word(href.offset shr 48)));
      list.concat(taicpu.op_reg_reg_const(A_ORI, NR_R0, NR_R0, word(href.offset shr 32)));
      list.concat(taicpu.op_reg_reg_const(A_SLDI, NR_R0, NR_R0, 32));
      list.concat(taicpu.op_reg_reg_const(A_ORIS, NR_R0, NR_R0, word(href.offset shr 16)));
      list.concat(taicpu.op_reg_reg_const(A_ORI, NR_R0, NR_R0, word(href.offset)));

      list.concat(taicpu.op_reg_reg_reg(A_ADD, NR_R1, NR_R1, NR_R0));
    end;
  end;

  restore_standard_registers;
end;


procedure tcgppc.a_loadaddr_ref_reg(list: TAsmList; const ref: treference; r:
  tregister);

var
  ref2, tmpref: treference;
  { register used to construct address }
  tempreg : TRegister;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if (target_info.system in [system_powerpc64_darwin,system_powerpc64_aix]) then
=======
  if (target_info.system = system_powerpc64_darwin) then
>>>>>>> graemeg/fixes_2_2
=======
  if (target_info.system = system_powerpc64_darwin) then
>>>>>>> origin/fixes_2_2
    begin
      inherited a_loadaddr_ref_reg(list,ref,r);
      exit;
    end;

  ref2 := ref;
  fixref(list, ref2);
  { load a symbol }
  if (assigned(ref2.symbol) or (hasLargeOffset(ref2))) then begin
    { add the symbol's value to the base of the reference, and if the }
    { reference doesn't have a base, create one                       }
    reference_reset(tmpref, ref2.alignment);
    tmpref.offset := ref2.offset;
    tmpref.symbol := ref2.symbol;
    tmpref.relsymbol := ref2.relsymbol;
    { load 64 bit reference into r. If the reference already has a base register,
     first load the 64 bit value into a temp register, then add it to the result
     register rD }
    if (ref2.base <> NR_NO) then begin
      { already have a base register, so allocate a new one }
      tempreg := rg[R_INTREGISTER].getregister(list, R_SUBWHOLE);
    end else begin
      tempreg := r;
    end;

    { code for loading a reference from a symbol into a register rD }
    (*
    lis   rX,SYM@highest
    ori   rX,SYM@higher
    sldi  rX,rX,32
    oris  rX,rX,SYM@h
    ori   rX,rX,SYM@l
    *)
    {$IFDEF EXTDEBUG}
    list.concat(tai_comment.create(strpnew('loadaddr_ref_reg ')));
    {$ENDIF EXTDEBUG}
    if (assigned(tmpref.symbol)) then begin
      tmpref.refaddr := addr_highest;
      list.concat(taicpu.op_reg_ref(A_LIS, tempreg, tmpref));
      tmpref.refaddr := addr_higher;
      list.concat(taicpu.op_reg_reg_ref(A_ORI, tempreg, tempreg, tmpref));
      list.concat(taicpu.op_reg_reg_const(A_SLDI, tempreg, tempreg, 32));
      tmpref.refaddr := addr_high;
      list.concat(taicpu.op_reg_reg_ref(A_ORIS, tempreg, tempreg, tmpref));
      tmpref.refaddr := addr_low;
      list.concat(taicpu.op_reg_reg_ref(A_ORI, tempreg, tempreg, tmpref));
    end else
      a_load_const_reg(list, OS_ADDR, tmpref.offset, tempreg);

    { if there's already a base register, add the temp register contents to
     the base register }
    if (ref2.base <> NR_NO) then begin
      list.concat(taicpu.op_reg_reg_reg(A_ADD, r, tempreg, ref2.base));
    end;
  end else if (ref2.offset <> 0) then begin
    { no symbol, but offset <> 0 }
    if (ref2.base <> NR_NO) then begin
      a_op_const_reg_reg(list, OP_ADD, OS_64, ref2.offset, ref2.base, r)
      { FixRef makes sure that "(ref.index <> R_NO) and (ref.offset <> 0)" never
       occurs, so now only ref.offset has to be loaded }
    end else begin
      a_load_const_reg(list, OS_64, ref2.offset, r);
    end;
  end else if (ref2.index <> NR_NO) then begin
    list.concat(taicpu.op_reg_reg_reg(A_ADD, r, ref2.base, ref2.index))
  end else if (ref2.base <> NR_NO) and
    (r <> ref2.base) then begin
    a_load_reg_reg(list, OS_ADDR, OS_ADDR, ref2.base, r)
  end else begin
    list.concat(taicpu.op_reg_const(A_LI, r, 0));
  end;
end;

{ ************* concatcopy ************ }

procedure tcgppc.g_concatcopy(list: TAsmList; const source, dest: treference;
  len: aint);

var
  countreg, tempreg:TRegister;
  src, dst: TReference;
  lab: tasmlabel;
  count, count2, step: longint;
  size: tcgsize;

begin
{$IFDEF extdebug}
  if len > high(aint) then
    internalerror(2002072704);
  list.concat(tai_comment.create(strpnew('g_concatcopy1 ' + inttostr(len) + ' bytes left ')));
{$ENDIF extdebug}
  { if the references are equal, exit, there is no need to copy anything }
  if references_equal(source, dest) or
     (len=0) then
    exit;

  { make sure short loads are handled as optimally as possible;
   note that the data here never overlaps, so we can do a forward
   copy at all times.
   NOTE: maybe use some scratch registers to pair load/store instructions
  }

  if (len <= 8) then begin
    src := source; dst := dest;
    {$IFDEF extdebug}
    list.concat(tai_comment.create(strpnew('g_concatcopy3 ' + inttostr(src.offset) + ' ' + inttostr(dst.offset))));
    {$ENDIF extdebug}
    while (len <> 0) do begin
      if (len = 8) then begin
        a_load_ref_ref(list, OS_64, OS_64, src, dst);
        dec(len, 8);
      end else if (len >= 4) then begin
        a_load_ref_ref(list, OS_32, OS_32, src, dst);
        inc(src.offset, 4); inc(dst.offset, 4);
        dec(len, 4);
      end else if (len >= 2) then begin
        a_load_ref_ref(list, OS_16, OS_16, src, dst);
        inc(src.offset, 2); inc(dst.offset, 2);
        dec(len, 2);
      end else begin
        a_load_ref_ref(list, OS_8, OS_8, src, dst);
        inc(src.offset, 1); inc(dst.offset, 1);
        dec(len, 1);
      end;
    end;
    exit;
  end;
{$IFDEF extdebug}
  list.concat(tai_comment.create(strpnew('g_concatcopy2 ' + inttostr(len) + ' bytes left ')));
{$ENDIF extdebug}


  if not(source.alignment in [1,2]) and
     not(dest.alignment in [1,2]) then
    begin
      count:=len div 8;
      step:=8;
      size:=OS_64;
    end
  else
    begin
      count:=len div 4;
      step:=4;
      size:=OS_32;
    end;

  tempreg:=getintregister(list,size);
<<<<<<< HEAD
<<<<<<< HEAD
  reference_reset(src,source.alignment);
  reference_reset(dst,dest.alignment);
=======
=======
>>>>>>> origin/fixes_2_2
  reference_reset(src);
  reference_reset(dst);
>>>>>>> graemeg/fixes_2_2
  { load the address of source into src.base }
  if (count > 4) or
    not issimpleref(source) or
    ((source.index <> NR_NO) and
     ((source.offset + len) > high(smallint))) then begin
    src.base := getaddressregister(list);
    a_loadaddr_ref_reg(list, source, src.base);
  end else begin
    src := source;
  end;
  { load the address of dest into dst.base }
  if (count > 4) or
    not issimpleref(dest) or
    ((dest.index <> NR_NO) and
    ((dest.offset + len) > high(smallint))) then begin
    dst.base := getaddressregister(list);
    a_loadaddr_ref_reg(list, dest, dst.base);
  end else begin
    dst := dest;
  end;

  { generate a loop }
  if count > 4 then begin
    { the offsets are zero after the a_loadaddress_ref_reg and just
     have to be set to step. I put an Inc there so debugging may be
     easier (should offset be different from zero here, it will be
     easy to notice in the generated assembler }
    inc(dst.offset, step);
    inc(src.offset, step);
    list.concat(taicpu.op_reg_reg_const(A_SUBI, src.base, src.base, step));
    list.concat(taicpu.op_reg_reg_const(A_SUBI, dst.base, dst.base, step));
    countreg := getintregister(list, OS_INT);
    a_load_const_reg(list, OS_INT, count, countreg);
    current_asmdata.getjumplabel(lab);
    a_label(list, lab);
    list.concat(taicpu.op_reg_reg_const(A_SUBIC_, countreg, countreg, 1));
    if (size=OS_64) then
      begin
        list.concat(taicpu.op_reg_ref(A_LDU, tempreg, src));
        list.concat(taicpu.op_reg_ref(A_STDU, tempreg, dst));
      end
    else
      begin
        list.concat(taicpu.op_reg_ref(A_LWZU, tempreg, src));
        list.concat(taicpu.op_reg_ref(A_STWU, tempreg, dst));
      end;
    a_jmp(list, A_BC, C_NE, 0, lab);
    a_reg_sync(list,src.base);
    a_reg_sync(list,dst.base);
    a_reg_sync(list,countreg);
    len := len mod step;
    count := 0;
  end;

  { unrolled loop }
  if count > 0 then begin
    for count2 := 1 to count do begin
      a_load_ref_reg(list, size, size, src, tempreg);
      a_load_reg_ref(list, size, size, tempreg, dst);
      inc(src.offset, step);
      inc(dst.offset, step);
    end;
    len := len mod step;
  end;

  if (len and 4) <> 0 then begin
    a_load_ref_reg(list, OS_32, OS_32, src, tempreg);
    a_load_reg_ref(list, OS_32, OS_32, tempreg, dst);
    inc(src.offset, 4);
    inc(dst.offset, 4);
  end;
  { copy the leftovers }
  if (len and 2) <> 0 then begin
    a_load_ref_reg(list, OS_16, OS_16, src, tempreg);
    a_load_reg_ref(list, OS_16, OS_16, tempreg, dst);
    inc(src.offset, 2);
    inc(dst.offset, 2);
  end;
  if (len and 1) <> 0 then begin
    a_load_ref_reg(list, OS_8, OS_8, src, tempreg);
    a_load_reg_ref(list, OS_8, OS_8, tempreg, dst);
  end;
<<<<<<< HEAD

=======

<<<<<<< HEAD
end;

=======
>>>>>>> origin/fixes_2_2
procedure tcgppc.g_external_wrapper(list: TAsmList; pd: TProcDef; const externalname: string);
var
  href : treference;
begin
  if (target_info.system <> system_powerpc64_linux) then begin
    inherited;
    exit;
  end;

  { for ppc64/linux emit correct code which sets up a stack frame and then calls the
  external method normally to ensure that the GOT/TOC will be loaded correctly if
  required.

  It's not really advantageous to use cg methods here because they are too specialized.

  I.e. the resulting code sequence looks as follows:

  mflr r0
  std r0, 16(r1)
  stdu r1, -112(r1)
  bl <external_method>
  nop
  addi r1, r1, 112
  ld r0, 16(r1)
  mtlr r0
  blr
<<<<<<< HEAD

  }
  list.concat(taicpu.op_reg(A_MFLR, NR_R0));
  reference_reset_base(href, NR_STACK_POINTER_REG, 16);
  list.concat(taicpu.op_reg_ref(A_STD, NR_R0, href));
  reference_reset_base(href, NR_STACK_POINTER_REG, -MINIMUM_STACKFRAME_SIZE);
  list.concat(taicpu.op_reg_ref(A_STDU, NR_STACK_POINTER_REG, href));

  list.concat(taicpu.op_sym(A_BL, current_asmdata.RefAsmSymbol(externalname)));
  list.concat(taicpu.op_none(A_NOP));

=======

  }
  list.concat(taicpu.op_reg(A_MFLR, NR_R0));
  reference_reset_base(href, NR_STACK_POINTER_REG, 16);
  list.concat(taicpu.op_reg_ref(A_STD, NR_R0, href));
  reference_reset_base(href, NR_STACK_POINTER_REG, -MINIMUM_STACKFRAME_SIZE);
  list.concat(taicpu.op_reg_ref(A_STDU, NR_STACK_POINTER_REG, href));

  list.concat(taicpu.op_sym(A_BL, current_asmdata.RefAsmSymbol(externalname)));
  list.concat(taicpu.op_none(A_NOP));

>>>>>>> origin/fixes_2_2
  list.concat(taicpu.op_reg_reg_const(A_ADDI, NR_STACK_POINTER_REG, NR_STACK_POINTER_REG, MINIMUM_STACKFRAME_SIZE));

  reference_reset_base(href, NR_STACK_POINTER_REG, LA_LR_ELF);
  list.concat(taicpu.op_reg_ref(A_LD, NR_R0, href));
  list.concat(taicpu.op_reg(A_MTLR, NR_R0));
  list.concat(taicpu.op_none(A_BLR));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{***************** This is private property, keep out! :) *****************}

procedure tcgppc.maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
const
  overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
begin
  {$IFDEF EXTDEBUG}
  list.concat(tai_comment.create(strpnew('maybeadjustresult op = ' + cgop2string(op) + ' size = ' + cgsize2string(size))));
  {$ENDIF EXTDEBUG}

  if (op in overflowops) and (size in [OS_8, OS_S8, OS_16, OS_S16, OS_32, OS_S32]) then
    a_load_reg_reg(list, OS_64, size, dst, dst);
end;

function tcgppc.issimpleref(const ref: treference): boolean;

begin
  if (ref.base = NR_NO) and
    (ref.index <> NR_NO) then
    internalerror(200208101);
  result :=
    not (assigned(ref.symbol)) and
    (((ref.index = NR_NO) and
    (ref.offset >= low(smallint)) and
    (ref.offset <= high(smallint))) or
    ((ref.index <> NR_NO) and
    (ref.offset = 0)));
end;

procedure tcgppc.a_load_store(list: TAsmList; op: tasmop; reg: tregister;
  ref: treference);

  procedure maybefixup64bitoffset;
    var
      tmpreg: tregister;
    begin
      { for some instructions we need to check that the offset is divisible by at
       least four. If not, add the bytes which are "off" to the base register and
       adjust the offset accordingly }
      case op of
        A_LD, A_LDU, A_STD, A_STDU, A_LWA :
           if ((ref.offset mod 4) <> 0) then begin
            tmpreg := rg[R_INTREGISTER].getregister(list, R_SUBWHOLE);

            if (ref.base <> NR_NO) then begin
              a_op_const_reg_reg(list, OP_ADD, OS_ADDR, ref.offset mod 4, ref.base, tmpreg);
              ref.base := tmpreg;
            end else begin
              list.concat(taicpu.op_reg_const(A_LI, tmpreg, ref.offset mod 4));
              ref.base := tmpreg;
            end;
            ref.offset := (ref.offset div 4) * 4;
          end;
      end;
    end;

var
  tmpreg, tmpreg2: tregister;
  tmpref: treference;
  largeOffset: Boolean;
begin
  if (target_info.system = system_powerpc64_darwin) then
    begin
      { darwin/ppc64 works with 32 bit relocatable symbol addresses }
      maybefixup64bitoffset;
      inherited a_load_store(list,op,reg,ref);
      exit
    end;

  { at this point there must not be a combination of values in the ref treference
    which is not possible to directly map to instructions of the PowerPC architecture }
  if (ref.index <> NR_NO) and ((ref.offset <> 0) or (assigned(ref.symbol))) then
    internalerror(200310131);

  { if this is a PIC'ed address, handle it and exit }
  if (ref.refaddr in [addr_pic,addr_pic_no_got]) then begin
    if (ref.offset <> 0) then
      internalerror(2006010501);
    if (ref.index <> NR_NO) then
      internalerror(2006010502);
    if (not assigned(ref.symbol)) then
      internalerror(200601050);
    list.concat(taicpu.op_reg_ref(op, reg, ref));
    exit;
  end;

  maybefixup64bitoffset;
  {$IFDEF EXTDEBUG}
  list.concat(tai_comment.create(strpnew('a_load_store1 ' + BoolToStr(ref.refaddr = addr_pic))));
  {$ENDIF EXTDEBUG}
  { if we have to load/store from a symbol or large addresses, use a temporary register
   containing the address }
  if (assigned(ref.symbol) or (hasLargeOffset(ref))) then begin
    tmpreg := rg[R_INTREGISTER].getregister(list, R_SUBWHOLE);

    if (hasLargeOffset(ref) and (ref.base = NR_NO)) then begin
      ref.base := rg[R_INTREGISTER].getregister(list, R_SUBWHOLE);
      a_load_const_reg(list, OS_ADDR, ref.offset, ref.base);
      ref.offset := 0;
    end;

    reference_reset(tmpref, ref.alignment);
    tmpref.symbol := ref.symbol;
    tmpref.relsymbol := ref.relsymbol;
    tmpref.offset := ref.offset;

    if (ref.base <> NR_NO) then begin
      { As long as the TOC isn't working we try to achieve highest speed (in this
      case by allowing instructions execute in parallel) as possible at the cost
      of using another temporary register. So the code template when there is
      a base register and an offset is the following:

      lis rT1, SYM+offs@highest
      ori rT1, rT1, SYM+offs@higher
      lis rT2, SYM+offs@hi
      ori rT2, SYM+offs@lo
      rldimi rT2, rT1, 32

      <op>X reg, base, rT2
      }

      tmpreg2 := rg[R_INTREGISTER].getregister(list, R_SUBWHOLE);
      if (assigned(tmpref.symbol)) then begin
        tmpref.refaddr := addr_highest;
        list.concat(taicpu.op_reg_ref(A_LIS, tmpreg, tmpref));
        tmpref.refaddr := addr_higher;
        list.concat(taicpu.op_reg_reg_ref(A_ORI, tmpreg, tmpreg, tmpref));

        tmpref.refaddr := addr_high;
        list.concat(taicpu.op_reg_ref(A_LIS, tmpreg2, tmpref));
        tmpref.refaddr := addr_low;
        list.concat(taicpu.op_reg_reg_ref(A_ORI, tmpreg2, tmpreg2, tmpref));

        list.concat(taicpu.op_reg_reg_const_const(A_RLDIMI, tmpreg2, tmpreg, 32, 0));
      end else
        a_load_const_reg(list, OS_ADDR, tmpref.offset, tmpreg2);

      reference_reset(tmpref, ref.alignment);
      tmpref.base := ref.base;
      tmpref.index := tmpreg2;
      case op of
        { the code generator doesn't generate update instructions anyway, so
        error out on those instructions }
        A_LBZ : op := A_LBZX;
        A_LHZ : op := A_LHZX;
        A_LWZ : op := A_LWZX;
        A_LD : op := A_LDX;
        A_LHA : op := A_LHAX;
        A_LWA : op := A_LWAX;
        A_LFS : op := A_LFSX;
        A_LFD : op := A_LFDX;

        A_STB : op := A_STBX;
        A_STH : op := A_STHX;
        A_STW : op := A_STWX;
        A_STD : op := A_STDX;

        A_STFS : op := A_STFSX;
        A_STFD : op := A_STFDX;
        else
          { unknown load/store opcode }
          internalerror(2005101302);
      end;
      list.concat(taicpu.op_reg_ref(op, reg, tmpref));
    end else begin
      { when accessing value from a reference without a base register, use the
        following code template:

        lis rT,SYM+offs@highesta
        ori rT,SYM+offs@highera
        sldi rT,rT,32
        oris rT,rT,SYM+offs@ha
        ld rD,SYM+offs@l(rT)
      }
      tmpref.refaddr := addr_highesta;
      list.concat(taicpu.op_reg_ref(A_LIS, tmpreg, tmpref));
      tmpref.refaddr := addr_highera;
      list.concat(taicpu.op_reg_reg_ref(A_ORI, tmpreg, tmpreg, tmpref));
      list.concat(taicpu.op_reg_reg_const(A_SLDI, tmpreg, tmpreg, 32));
      tmpref.refaddr := addr_higha;
      list.concat(taicpu.op_reg_reg_ref(A_ORIS, tmpreg, tmpreg, tmpref));

      tmpref.base := tmpreg;
      tmpref.refaddr := addr_low;
      list.concat(taicpu.op_reg_ref(op, reg, tmpref));
    end;
  end else begin
    list.concat(taicpu.op_reg_ref(op, reg, ref));
  end;
end;

procedure tcgppc.loadConstantPIC(list : TAsmList; size : TCGSize; a : aint; reg : TRegister);
var
  l: tasmsymbol;
  ref: treference;
  symname : string;
begin
  maybe_new_object_file(current_asmdata.asmlists[al_picdata]);
  symname := '_$' + current_asmdata.name^ + '$toc$' + hexstr(a, sizeof(a)*2);
  l:=current_asmdata.getasmsymbol(symname);
  if not(assigned(l)) then begin
    l:=current_asmdata.DefineAsmSymbol(symname,AB_GLOBAL, AT_DATA);
    new_section(current_asmdata.asmlists[al_picdata],sec_toc, '.toc', 8);
    current_asmdata.asmlists[al_picdata].concat(tai_symbol.create_global(l,0));
    current_asmdata.asmlists[al_picdata].concat(tai_directive.create(asd_toc_entry, symname + '[TC], ' + inttostr(a)));
  end;
  reference_reset_symbol(ref,l,0, 8);
  ref.base := NR_R2;
  ref.refaddr := addr_no;

  {$IFDEF EXTDEBUG}
  list.concat(tai_comment.create(strpnew('loading value from TOC reference for ' + symname)));
  {$ENDIF EXTDEBUG}
  a_load_ref_reg(list, OS_INT, OS_INT, ref, reg);
end;


procedure create_codegen;
begin
  cg := tcgppc.create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  cg128:=tcg128.create;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

end.
