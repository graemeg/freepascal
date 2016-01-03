{
    Copyright (c) 1998-2002 by Florian Klaempfl and David Zhang

    Generate MIPSEL assembler for type converting nodes

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

 ****************************************************************************}
unit ncpucnv;

{$i fpcdefs.inc}

interface

uses
  node, ncnv, ncgcnv, defcmp;

type
  tMIPSELtypeconvnode = class(TCgTypeConvNode)
  protected
    { procedure second_int_to_int;override; }
    { procedure second_string_to_string;override; }
    { procedure second_cstring_to_pchar;override; }
    { procedure second_string_to_chararray;override; }
    { procedure second_array_to_pointer;override; }
    function first_int_to_real: tnode; override;
    { procedure second_pointer_to_array;override; }
    { procedure second_chararray_to_string;override; }
    { procedure second_char_to_string;override; }
    procedure second_int_to_real; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    { procedure second_real_to_real; override; }
=======
    procedure second_real_to_real; override;
>>>>>>> graemeg/cpstrnew
=======
    procedure second_real_to_real; override;
>>>>>>> graemeg/cpstrnew
=======
    procedure second_real_to_real; override;
>>>>>>> graemeg/cpstrnew
=======
    procedure second_real_to_real; override;
>>>>>>> origin/cpstrnew
    { procedure second_cord_to_pointer;override; }
    { procedure second_proc_to_procvar;override; }
    { procedure second_bool_to_int;override; }
    procedure second_int_to_bool; override;
    { procedure second_load_smallset;override;  }
    { procedure second_ansistring_to_pchar;override; }
    { procedure second_pchar_to_string;override; }
    { procedure second_class_to_intf;override; }
    { procedure second_char_to_char;override; }
  end;

implementation

uses
  verbose, globtype, globals, systems,
  symconst, symdef, aasmbase, aasmtai, aasmdata,
  defutil,
  cgbase, cgutils, pass_1, pass_2, procinfo,
  ncon, ncal,
  ncgutil,
  cpubase, aasmcpu,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  tgobj, cgobj,
  hlcgobj;
=======
  tgobj, cgobj;
>>>>>>> graemeg/cpstrnew
=======
  tgobj, cgobj;
>>>>>>> graemeg/cpstrnew
=======
  tgobj, cgobj;
>>>>>>> graemeg/cpstrnew
=======
  tgobj, cgobj;
>>>>>>> origin/cpstrnew


{*****************************************************************************
                             FirstTypeConv
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function tmipseltypeconvnode.first_int_to_real: tnode;
=======
function tMIPSELtypeconvnode.first_int_to_real: tnode;
>>>>>>> graemeg/cpstrnew
=======
function tMIPSELtypeconvnode.first_int_to_real: tnode;
>>>>>>> graemeg/cpstrnew
=======
function tMIPSELtypeconvnode.first_int_to_real: tnode;
>>>>>>> graemeg/cpstrnew
=======
function tMIPSELtypeconvnode.first_int_to_real: tnode;
>>>>>>> origin/cpstrnew
var
  fname: string[19];
begin
  { converting a 64bit integer to a float requires a helper }
  if is_64bitint(left.resultdef) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     is_currency(left.resultdef) then
    begin
      { hack to avoid double division by 10000, as it's
        already done by typecheckpass.resultdef_int_to_real }
      if is_currency(left.resultdef) then
        left.resultdef := s64inttype;
      if is_signed(left.resultdef) then
        fname := 'fpc_int64_to_double'
      else
        fname := 'fpc_qword_to_double';
      result := ccallnode.createintern(fname,ccallparanode.create(
        left,nil));
      left:=nil;
      if (tfloatdef(resultdef).floattype=s32real) then
        inserttypeconv(result,s32floattype);
      firstpass(result);
      exit;
    end
  else
    { other integers are supposed to be 32 bit }
    begin
      if is_signed(left.resultdef) then
        inserttypeconv(left,s32inttype)
      else
        begin
          inserttypeconv(left,u32inttype);
          if (cs_create_pic in current_settings.moduleswitches) then
            include(current_procinfo.flags,pi_needs_got);
        end;
      firstpass(left);
    end;
  result := nil;
  expectloc:=LOC_FPUREGISTER;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    is_currency(left.resultdef) then
  begin
            { hack to avoid double division by 10000, as it's
              already done by resulttypepass.resulttype_int_to_real }
    if is_currency(left.resultdef) then
      left.resultdef := s64inttype;
    if is_signed(left.resultdef) then
      fname := 'fpc_int64_to_double'
    else
      fname := 'fpc_qword_to_double';
    Result := ccallnode.createintern(fname, ccallparanode.Create(
      left, nil));
    left := nil;
    firstpass(Result);
    exit;
  end
  else
    { other integers are supposed to be 32 bit }
  begin
    if is_signed(left.resultdef) then
      inserttypeconv(left, s32inttype)
    else
      inserttypeconv(left, u32inttype);
    firstpass(left);
  end;
  Result := nil;
  expectloc := LOC_FPUREGISTER;
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


{*****************************************************************************
                             SecondTypeConv
*****************************************************************************}

procedure tMIPSELtypeconvnode.second_int_to_real;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  procedure loadsigned(restype: tfloattype);
  begin
    location.Register := cg.getfpuregister(current_asmdata.CurrAsmList, tfloat2tcgsize[restype]);
    if (left.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
      { 32-bit values can be loaded directly }
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_MTC1, left.location.register, location.register))
    else
      begin
        { Load memory in fpu register }
        hlcg.location_force_mem(current_asmdata.CurrAsmList, left.location, left.resultdef);
        cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList, OS_F32, OS_F32, left.location.reference, location.Register);
        tg.ungetiftemp(current_asmdata.CurrAsmList, left.location.reference);
      end;

    { Convert value in fpu register from integer to float }
    case restype of
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  procedure loadsigned;
  begin
    location_force_mem(current_asmdata.CurrAsmList, left.location);
    location.Register := cg.getfpuregister(current_asmdata.CurrAsmList, location.size);
    { Load memory in fpu register }
    cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList, OS_F32, OS_F32, left.location.reference, location.Register);
    tg.ungetiftemp(current_asmdata.CurrAsmList, left.location.reference);
    { Convert value in fpu register from integer to float }
    case tfloatdef(resultdef).floattype of
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
      s32real:
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVT_S_W, location.Register, location.Register));
      s64real:
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVT_D_W, location.Register, location.Register));
      else
        internalerror(200408011);
    end;
  end;

var
  href:      treference;
  hregister: tregister;
  l1, l2:    tasmlabel;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  location_reset(location, LOC_FPUREGISTER, def_cgsize(resultdef));
  if is_signed(left.resultdef) then
    loadsigned(tfloatdef(resultdef).floattype)
  else
  begin
    current_asmdata.getglobaldatalabel(l1);
    current_asmdata.getjumplabel(l2);
    reference_reset_symbol(href, l1, 0, sizeof(aint));
    hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

    { Always load into 64-bit FPU register }
    loadsigned(s64real);
    cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList, OS_INT, OC_GTE, 0, left.location.register, l2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
  location_reset(location, LOC_FPUREGISTER, def_cgsize(resultdef));
  if is_signed(left.resultdef) then
    loadsigned
  else
  begin
    current_asmdata.getdatalabel(l1);
    current_asmdata.getjumplabel(l2);
    reference_reset_symbol(href, l1, 0, sizeof(aint));
    hregister := cg.getintregister(current_asmdata.CurrAsmList, OS_32);
    cg.a_load_loc_reg(current_asmdata.CurrAsmList, OS_32, left.location, hregister);

    loadsigned;

    current_asmdata.CurrAsmList.concat(Taicpu.op_reg_reg_sym(A_BGE, hregister, NR_R0, l2));
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

    case tfloatdef(resultdef).floattype of
      { converting dword to s64real first and cut off at the end avoids precision loss }
      s32real,
      s64real:
      begin
        hregister := cg.getfpuregister(current_asmdata.CurrAsmList, OS_F64);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,l1.name,const_align(8));
        current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
        current_asmdata.asmlists[al_typedconsts].concat(tai_realconst.create_s64real(4294967296.0));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        current_asmdata.asmlists[al_typedconsts].concat(tai_align.Create(const_align(8)));
        current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));

        { I got this constant from a test program (FK) }
        current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit(0));
        current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit($0000f041));
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

        cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList, OS_F64, OS_F64, href, hregister);
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ADD_D, location.Register, hregister, location.Register));
        cg.a_label(current_asmdata.CurrAsmList, l2);

        { cut off if we should convert to single }
        if tfloatdef(resultdef).floattype = s32real then
        begin
          hregister := location.Register;
          location.Register := cg.getfpuregister(current_asmdata.CurrAsmList, location.size);
          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CVT_S_D, location.Register, hregister));
        end;
      end;
      else
        internalerror(200410031);
    end;
  end;
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
procedure tMIPSELtypeconvnode.second_real_to_real;
const
  conv_op: array[tfloattype, tfloattype] of tasmop = (
    {    from:   s32      s64         s80     sc80    c64     cur    f128 }
    { s32 }  (A_MOV_S,   A_CVT_S_D, A_NONE, A_NONE, A_NONE, A_NONE, A_NONE),
    { s64 }  (A_CVT_D_S, A_MOV_D,   A_NONE, A_NONE, A_NONE, A_NONE, A_NONE),
    { s80 }  (A_NONE,    A_NONE,    A_NONE, A_NONE, A_NONE, A_NONE, A_NONE),
    { sc80 } (A_NONE,    A_NONE,    A_NONE, A_NONE, A_NONE, A_NONE, A_NONE),
    { c64 }  (A_NONE,    A_NONE,    A_NONE, A_NONE, A_NONE, A_NONE, A_NONE),
    { cur }  (A_NONE,    A_NONE,    A_NONE, A_NONE, A_NONE, A_NONE, A_NONE),
    { f128 } (A_NONE,    A_NONE,    A_NONE, A_NONE, A_NONE, A_NONE, A_NONE)
    );
var
  op: tasmop;
begin
  location_reset(location, LOC_FPUREGISTER, def_cgsize(resultdef));
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, False);
  { Convert value in fpu register from integer to float }
  op := conv_op[tfloatdef(resultdef).floattype, tfloatdef(left.resultdef).floattype];
  if op = A_NONE then
    internalerror(200401121);
  location.Register := cg.getfpuregister(current_asmdata.CurrAsmList, location.size);
  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, location.Register, left.location.Register));
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
procedure tMIPSELtypeconvnode.second_int_to_bool;
var
  hreg1, hreg2: tregister;
  opsize: tcgsize;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  hlabel: tasmlabel;
  newsize  : tcgsize;
  href: treference;
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  hlabel, oldtruelabel, oldfalselabel: tasmlabel;
begin
  oldtruelabel  := current_procinfo.CurrTrueLabel;
  oldfalselabel := current_procinfo.CurrFalseLabel;
  current_asmdata.getjumplabel(current_procinfo.CurrTrueLabel);
  current_asmdata.getjumplabel(current_procinfo.CurrFalseLabel);
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
  secondpass(left);
  if codegenerror then
    exit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { Explicit typecasts from any ordinal type to a boolean type }
  { must not change the ordinal value                          }
  if (nf_explicit in flags) and
     not(left.location.loc in [LOC_FLAGS,LOC_JUMP]) then
    begin
       location_copy(location,left.location);
       newsize:=def_cgsize(resultdef);
       { change of size? change sign only if location is LOC_(C)REGISTER? Then we have to sign/zero-extend }
       if (tcgsize2size[newsize]<>tcgsize2size[left.location.size]) or
          ((newsize<>left.location.size) and (location.loc in [LOC_REGISTER,LOC_CREGISTER])) then
         hlcg.location_force_reg(current_asmdata.CurrAsmList,location,left.resultdef,resultdef,true)
       else
         location.size:=newsize;
       exit;
    end;

  location_reset(location, LOC_REGISTER, def_cgsize(resultdef));
  opsize := def_cgsize(left.resultdef);

  if (left.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF]) then
    hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { byte(boolean) or word(wordbool) or longint(longbool) must }
  { be accepted for var parameters                            }
  if (nf_explicit in flags) and
    (left.resultdef.size = resultdef.size) and
    (left.location.loc in [LOC_REFERENCE, LOC_CREFERENCE, LOC_CREGISTER]) then
  begin
    location_copy(location, left.location);
    current_procinfo.CurrTrueLabel  := oldtruelabel;
    current_procinfo.CurrFalseLabel := oldfalselabel;
    exit;
  end;
  location_reset(location, LOC_REGISTER, def_cgsize(resultdef));
  opsize := def_cgsize(left.resultdef);
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
  case left.location.loc of
    LOC_CREFERENCE, LOC_REFERENCE, LOC_REGISTER, LOC_CREGISTER:
    begin
      if left.location.loc in [LOC_CREFERENCE, LOC_REFERENCE] then
      begin
        hreg2 := cg.getintregister(current_asmdata.CurrAsmList, opsize);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef cpu64bitalu}
        if left.location.size in [OS_64,OS_S64] then
          begin
            cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,left.location.reference,hreg2);
            hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
            href:=left.location.reference;
            inc(href.offset,4);
            cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,href,hreg1);
            cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,hreg1,hreg2,hreg2);
          end
          else
{$endif not cpu64bitalu}
            cg.a_load_ref_reg(current_asmdata.CurrAsmList, opsize, opsize, left.location.reference, hreg2);
      end
      else
        begin
          hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
{$ifndef cpu64bitalu}
          if left.location.size in [OS_64,OS_S64] then
            begin
              hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
              cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.register64.reghi,left.location.register64.reglo,hreg2);
             end
           else
{$endif not cpu64bitalu}
             cg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,opsize,left.location.register,hreg2);
         end;
       hreg1 := cg.getintregister(current_asmdata.CurrAsmList, opsize);
       current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SLTU, hreg1, NR_R0, hreg2));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        cg.a_load_ref_reg(current_asmdata.CurrAsmList, opsize, opsize, left.location.reference, hreg2);
      end
      else
        hreg2 := left.location.Register;
{$ifndef cpu64bit}
      if left.location.size in [OS_64, OS_S64] then
      begin
        hreg1 := cg.getintregister(current_asmdata.CurrAsmList, OS_32);
        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_OR, OS_32, hreg2, tregister(succ(longint(hreg2))), hreg1);
        hreg2  := hreg1;
        opsize := OS_32;
      end;
{$endif cpu64bit}
      hreg1 := cg.getintregister(current_asmdata.CurrAsmList, opsize);
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SNE, hreg1, hreg2, NR_R0));
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
    LOC_JUMP:
    begin
      hreg1 := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
      current_asmdata.getjumplabel(hlabel);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      cg.a_label(current_asmdata.CurrAsmList, left.location.truelabel);
      cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, 1, hreg1);
      cg.a_jmp_always(current_asmdata.CurrAsmList, hlabel);
      cg.a_label(current_asmdata.CurrAsmList, left.location.falselabel);
      cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, 0, hreg1);
      cg.a_label(current_asmdata.CurrAsmList, hlabel);
    end;
    LOC_FLAGS:
    begin
      hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
      cg.g_flags2reg(current_asmdata.CurrAsmList,OS_INT,left.location.resflags,hreg1);
    end
    else
      internalerror(10062);
  end;
  { Now hreg1 is either 0 or 1. For C booleans it must be 0 or -1. }
  if is_cbool(resultdef) then
    cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,OS_SINT,hreg1,hreg1);

{$ifndef cpu64bitalu}
  if (location.size in [OS_64,OS_S64]) then
    begin
      location.register64.reglo:=hreg1;
      location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
      if (is_cbool(resultdef)) then
       { reglo is either 0 or -1 -> reghi has to become the same }
          cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_32,OS_32,location.register64.reglo,location.register64.reghi)
       else
       { unsigned }
         cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,location.register64.reghi);
       end
       else
{$endif not cpu64bitalu}
         location.Register := hreg1;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      cg.a_label(current_asmdata.CurrAsmList, current_procinfo.CurrTrueLabel);
      cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, 1, hreg1);
      cg.a_jmp_always(current_asmdata.CurrAsmList, hlabel);
      cg.a_label(current_asmdata.CurrAsmList, current_procinfo.CurrFalseLabel);
      cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, 0, hreg1);
      cg.a_label(current_asmdata.CurrAsmList, hlabel);
    end;
    else
      internalerror(10062);
  end;
  location.Register := hreg1;

  if location.size in [OS_64, OS_S64] then
    internalerror(200408241);

  current_procinfo.CurrTrueLabel  := oldtruelabel;
  current_procinfo.CurrFalseLabel := oldfalselabel;
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


begin
  ctypeconvnode := tMIPSELtypeconvnode;
end.
