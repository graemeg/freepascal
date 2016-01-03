{
    Copyright (c) 1998-2009 by Florian Klaempfl and David Zhang

    Generate MIPSEL inline nodes

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
unit ncpuinln;

{$i fpcdefs.inc}

interface

uses
  node, ninl, ncginl;

type
  tMIPSELinlinenode = class(tcgInlineNode)
    function first_abs_real: tnode; override;
    function first_sqr_real: tnode; override;
    function first_sqrt_real: tnode; override;
    procedure second_abs_real; override;
    procedure second_sqr_real; override;
    procedure second_sqrt_real; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure second_get_frame; override;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  private
    procedure load_fpu_location;
  end;


implementation

uses
  systems,
  globtype,
  cutils, verbose,
  symconst, symdef,
  aasmtai, aasmcpu, aasmdata,
  cgbase, pass_2,
  cpubase, paramgr,
  nbas, ncon, ncal, ncnv, nld,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  hlcgobj, ncgutil, cgobj, cgutils;
=======
  ncgutil, cgobj, cgutils;
>>>>>>> graemeg/cpstrnew
=======
  ncgutil, cgobj, cgutils;
>>>>>>> graemeg/cpstrnew
=======
  ncgutil, cgobj, cgutils;
>>>>>>> graemeg/cpstrnew
=======
  ncgutil, cgobj, cgutils;
>>>>>>> origin/cpstrnew

{*****************************************************************************
                              tMIPSELinlinenode
*****************************************************************************}

procedure tMIPSELinlinenode.load_fpu_location;
begin
  secondpass(left);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  hlcg.location_force_fpureg(current_asmdata.CurrAsmList, left.location, left.resultdef, True);
=======
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, True);
>>>>>>> graemeg/cpstrnew
=======
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, True);
>>>>>>> graemeg/cpstrnew
=======
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, True);
>>>>>>> graemeg/cpstrnew
=======
  location_force_fpureg(current_asmdata.CurrAsmList, left.location, True);
>>>>>>> origin/cpstrnew
  location_copy(location, left.location);
  if left.location.loc = LOC_CFPUREGISTER then
  begin
    location.Register := cg.getfpuregister(current_asmdata.CurrAsmList, location.size);
    location.loc      := LOC_FPUREGISTER;
  end;
end;


function tMIPSELinlinenode.first_abs_real: tnode;
begin
  expectloc      := LOC_FPUREGISTER;
  first_abs_real := nil;
end;


function tMIPSELinlinenode.first_sqr_real: tnode;
begin
  expectloc      := LOC_FPUREGISTER;
  first_sqr_real := nil;
end;


function tMIPSELinlinenode.first_sqrt_real: tnode;
begin
  expectloc    := LOC_FPUREGISTER;
  first_sqrt_real := nil;
end;


procedure tMIPSELinlinenode.second_abs_real;
begin
  load_fpu_location;
  case tfloatdef(left.resultdef).floattype of
    s32real:
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_ABS_s, location.Register, left.location.Register));
    s64real:
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_ABS_d, location.Register, left.location.Register));
    else
      internalerror(200410031);
  end;
end;


procedure tMIPSELinlinenode.second_sqr_real;
begin
  load_fpu_location;
  case tfloatdef(left.resultdef).floattype of
    s32real:
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MUL_s, location.Register, left.location.Register, left.location.Register));
    s64real:
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MUL_d, location.Register, left.location.Register, left.location.Register));
    else
      internalerror(200410032);
  end;
end;


procedure tMIPSELinlinenode.second_sqrt_real;
begin
  load_fpu_location;
  case tfloatdef(left.resultdef).floattype of
    s32real:
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SQRT_s, location.Register, left.location.Register));
    s64real:
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_SQRT_d, location.Register, left.location.Register));
    else
      internalerror(200410033);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure tMIPSELinlinenode.second_get_frame;
begin
  location_reset(location,LOC_CREGISTER,OS_ADDR);
  location.register:=NR_FRAME_POINTER_REG;
end;


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  cInlineNode := tMIPSELinlinenode;
end.
