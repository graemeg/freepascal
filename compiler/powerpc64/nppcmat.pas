{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate PowerPC assembler for math nodes

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
unit nppcmat;

{$I fpcdefs.inc}

interface

uses
  node, nmat, ncgmat;

type
  tppcmoddivnode = class(tmoddivnode)
    function pass_1: tnode; override;
    procedure pass_generate_code override;
  end;

  tppcshlshrnode = class(tshlshrnode)
    procedure pass_generate_code override;
  end;

  tppcunaryminusnode = class(tunaryminusnode)
    procedure pass_generate_code override;
  end;

  tppcnotnode = class(tcgnotnode)
    procedure pass_generate_code override;
  end;

implementation

uses
  sysutils,
  globtype, systems,constexp,
  cutils, verbose, globals,
  symconst, symdef,
  aasmbase, aasmcpu, aasmtai,aasmdata,
  defutil,
  cgbase, cgutils, cgobj, hlcgobj, pass_1, pass_2,
  ncon, procinfo, nbas, nld, nadd,
  cpubase, cpuinfo,
  ncgutil, cgcpu, rgobj;

{*****************************************************************************
                             TPPCMODDIVNODE
*****************************************************************************}

function tppcmoddivnode.pass_1: tnode;
var
    statementnode : tstatementnode;
    temp_left, temp_right : ttempcreatenode;
    left_copy, right_copy : tnode;
    block : tblocknode;
begin
  result := nil;
  (*
  // this code replaces all mod nodes by the equivalent div/mul/sub sequence
  // on node level, which might be advantageous when doing CSE on that level
  // However, optimal modulo code for some cases (in particular a 'x mod 2^n-1'
  // operation) can not be expressed using nodes, so this is commented out for now
  if (nodetype = modn) then begin
    block := internalstatements(statementnode);

    temp_left := ctempcreatenode.create(left.resultdef, left.resultdef.size, tt_persistent, true);
    addstatement(statementnode, temp_left);
    addstatement(statementnode, cassignmentnode.create(ctemprefnode.create(temp_left), left.getcopy));

    if (right.nodetype <> ordconstn) then begin
      // implementated optimization: use temps to store the right value, otherwise
      // it is calculated twice when simply copying it which might result in side
      // effects
      temp_right := ctempcreatenode.create(right.resultdef, right.resultdef.size, tt_persistent, true);
      addstatement(statementnode, temp_right);
      addstatement(statementnode, cassignmentnode.create(ctemprefnode.create(temp_right), right.getcopy));

      addstatement(statementnode, cassignmentnode.create(ctemprefnode.create(temp_left),
        caddnode.create(subn, ctemprefnode.create(temp_left),
        caddnode.create(muln, cmoddivnode.create(divn, ctemprefnode.create(temp_left), ctemprefnode.create(temp_right)),
        ctemprefnode.create(temp_right)))));

      addstatement(statementnode, ctempdeletenode.create(temp_right));
    end else begin
      // in case this is a modulo by a constant operation, do not use a temp for the
      // right hand side, because otherwise the div optimization will not recognize this
      // fact (and there is no constant propagator/recognizer in the compiler),
      // resulting in suboptimal code.
      addstatement(statementnode, cassignmentnode.create(ctemprefnode.create(temp_left),
        caddnode.create(subn, ctemprefnode.create(temp_left),
        caddnode.create(muln, cmoddivnode.create(divn, ctemprefnode.create(temp_left), right.getcopy),
          right.getcopy))));
    end;
    addstatement(statementnode, ctempdeletenode.create_normal_temp(temp_left));
    addstatement(statementnode, ctemprefnode.create(temp_left));
    result := block;
  end;
  *)
  if (not assigned(result)) then
    result := inherited pass_1;
  if not assigned(result) then
    include(current_procinfo.flags, pi_do_call);
end;

procedure tppcmoddivnode.pass_generate_code;
const         { signed   overflow }
  divops: array[boolean, boolean] of tasmop =
    ((A_DIVDU, A_DIVDU_),(A_DIVD, A_DIVDO_));
  divcgops : array[boolean] of TOpCG = (OP_DIV, OP_IDIV);
  zerocond: tasmcond = (dirhint: DH_Plus; simple: true; cond:C_NE; cr: RS_CR7);
  tcgsize2native : array[OS_8..OS_S128] of tcgsize = (
    OS_64, OS_64, OS_64, OS_64, OS_NO,
    OS_S64, OS_S64, OS_S64, OS_S64, OS_NO
    );
var
  power  : longint;
  op  : tasmop;
  numerator, divider,
  resultreg  : tregister;
  size       : TCgSize;
  hl : tasmlabel;
  done: boolean;

  procedure genOrdConstNodeMod;
  var
    modreg, maskreg, tempreg : tregister;
    isNegPower : boolean;
  begin
    if (tordconstnode(right).value = 0) then begin
      internalerror(2005061702);
    end else if (abs(tordconstnode(right).value.svalue) = 1) then begin
      { x mod +/-1 is always zero }
      cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, 0, resultreg);
    end else if (ispowerof2(tordconstnode(right).value, power)) then begin
      if (is_signed(right.resultdef)) then begin
        tempreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
        maskreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
        modreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);

        cg.a_load_const_reg(current_asmdata.CurrAsmList, OS_INT, abs(tordconstnode(right).value.svalue)-1, modreg);
        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_SAR, OS_INT, 63, numerator, maskreg);
        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_AND, OS_INT, numerator, modreg, tempreg);

        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_ANDC, maskreg, maskreg, modreg));
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_SUBFIC, modreg, tempreg, 0));
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SUBFE, modreg, modreg, modreg));
        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_AND, OS_INT, modreg, maskreg, maskreg);
        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_OR, OS_INT, maskreg, tempreg, resultreg);
      end else begin
        cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_AND, OS_INT, tordconstnode(right).value-1, numerator,
          resultreg);
      end;
    end else begin
      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, divCgOps[is_signed(right.resultdef)], OS_INT,
        tordconstnode(right).value, numerator, resultreg);
      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, OP_MUL, OS_INT, tordconstnode(right).value.svalue, resultreg,
        resultreg);
      cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, OP_SUB, OS_INT, resultreg, numerator, resultreg);
    end;
  end;


begin
  secondpass(left);
  secondpass(right);
  location_copy(location,left.location);

  { put numerator in register }
  size:=def_cgsize(left.resultdef);
  hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,
    left.resultdef,left.resultdef,true);
  location_copy(location,left.location);
  numerator := location.register;
  resultreg := location.register;
  if (location.loc = LOC_CREGISTER) then begin
    location.loc := LOC_REGISTER;
    location.register := cg.getintregister(current_asmdata.CurrAsmList,size);
    resultreg := location.register;
  end else if (nodetype = modn) or (right.nodetype = ordconstn) then begin
    { for a modulus op, and for const nodes we need the result register
     to be an extra register }
    resultreg := cg.getintregister(current_asmdata.CurrAsmList,size);
  end;
  done := false;
  if (cs_opt_level1 in current_settings.optimizerswitches) and (right.nodetype = ordconstn) then begin
    if (nodetype = divn) then
      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, divCgOps[is_signed(right.resultdef)],
        size, tordconstnode(right).value, numerator, resultreg)
    else
      genOrdConstNodeMod;
    done := true;
  end;

  if (not done) then begin
    { load divider in a register if necessary }
    hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
    if (right.nodetype <> ordconstn) then
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_CMPDI, NR_CR7,
        right.location.register, 0))
    else begin
      if (tordconstnode(right).value = 0) then
        internalerror(2005100301);
    end;
    divider := right.location.register;

    { select the correct opcode according to the sign of the result, whether we need
     overflow checking }
    op := divops[is_signed(right.resultdef), cs_check_overflow in current_settings.localswitches];
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op, resultreg, numerator,
      divider));

    if (nodetype = modn) then begin
      { multiply with the divisor again, taking care of the correct size }
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_MULLD,resultreg,
          divider,resultreg));
      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SUB,location.register,
        numerator,resultreg));
      resultreg := location.register;
    end;
  end;
  { set result location }
  location.loc:=LOC_REGISTER;
  location.register:=resultreg;
  if right.nodetype <> ordconstn then begin
    current_asmdata.getjumplabel(hl);
    current_asmdata.CurrAsmList.concat(taicpu.op_cond_sym(A_BC,zerocond,hl));
    cg.a_call_name(current_asmdata.CurrAsmList,'FPC_DIVBYZERO',false);
    cg.a_label(current_asmdata.CurrAsmList,hl);
  end;
  { unsigned division/module can only overflow in case of division by zero
   (but checking this overflow flag is more convoluted than performing a
   simple comparison with 0)                                             }
  if is_signed(right.resultdef) then
    cg.g_overflowcheck(current_asmdata.CurrAsmList,location,resultdef);
end;

{*****************************************************************************
                             TPPCSHLRSHRNODE
*****************************************************************************}

procedure tppcshlshrnode.pass_generate_code;

var
  resultreg, hregister1, hregister2 : tregister;

  op: topcg;
  asmop1, asmop2: tasmop;
  shiftval: aint;

begin
  secondpass(left);
  secondpass(right);

  { load left operators in a register }
  hlcg.location_force_reg(current_asmdata.CurrAsmList, left.location,
    left.resultdef, left.resultdef, true);
  location_copy(location, left.location);
  resultreg := location.register;
  hregister1 := location.register;
  if (location.loc = LOC_CREGISTER) then begin
    location.loc := LOC_REGISTER;
    resultreg := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
    location.register := resultreg;
  end;

  { determine operator }
  if nodetype = shln then
    op := OP_SHL
  else
    op := OP_SHR;

  { shifting by a constant directly coded: }
  if (right.nodetype = ordconstn) then begin
    // result types with size < 32 bits have their shift values masked
    // differently... :/
    shiftval := tordconstnode(right).value and (tcgsize2size[def_cgsize(resultdef)] * 8 -1);
    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList, op, def_cgsize(resultdef),
      shiftval, hregister1, resultreg)
  end else begin
    { load shift count in a register if necessary }
    hlcg.location_force_reg(current_asmdata.CurrAsmList, right.location,
      right.resultdef, right.resultdef, true);
    hregister2 := right.location.register;
    cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList, op, def_cgsize(resultdef), hregister2,
      hregister1, resultreg);
  end;
end;

{*****************************************************************************
                          TPPCUNARYMINUSNODE
*****************************************************************************}

procedure tppcunaryminusnode.pass_generate_code;

var
  src1: tregister;
  op: tasmop;

begin
  secondpass(left);
  begin
    if left.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF] then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
=======
      location_force_reg(current_asmdata.CurrAsmList,left.location,left.location.size,true);
>>>>>>> graemeg/cpstrnew
=======
      location_force_reg(current_asmdata.CurrAsmList,left.location,left.location.size,true);
>>>>>>> graemeg/cpstrnew
=======
      location_force_reg(current_asmdata.CurrAsmList,left.location,left.location.size,true);
>>>>>>> graemeg/cpstrnew
=======
      location_force_reg(current_asmdata.CurrAsmList,left.location,left.location.size,true);
>>>>>>> origin/cpstrnew
    location_copy(location, left.location);
    location.loc := LOC_REGISTER;
    case left.location.loc of
      LOC_FPUREGISTER, LOC_REGISTER:
        begin
          src1 := left.location.register;
          location.register := src1;
        end;
      LOC_CFPUREGISTER, LOC_CREGISTER:
        begin
          src1 := left.location.register;
          if left.location.loc = LOC_CREGISTER then
            location.register := cg.getintregister(current_asmdata.CurrAsmList, OS_INT)
          else
            location.register := cg.getfpuregister(current_asmdata.CurrAsmList, location.size);
        end;
      LOC_REFERENCE, LOC_CREFERENCE:
        begin
          if (left.resultdef.typ = floatdef) then begin
            src1 := cg.getfpuregister(current_asmdata.CurrAsmList,
              left.location.size);
            location.register := src1;
            cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmList,
              left.location.size,left.location.size,
              left.location.reference, src1);
          end else begin
            src1 := cg.getintregister(current_asmdata.CurrAsmList, OS_64);
            location.register := src1;
            cg.a_load_ref_reg(current_asmdata.CurrAsmList, OS_64, OS_64,
              left.location.reference, src1);
          end;
        end;
      else
       internalerror(2013120112);
    end;
    { choose appropriate operand }
    if left.resultdef.typ <> floatdef then begin
      if not (cs_check_overflow in current_settings.localswitches) then
        op := A_NEG
      else
        op := A_NEGO_;
      location.loc := LOC_REGISTER;
    end else begin
      op := A_FNEG;
      location.loc := LOC_FPUREGISTER;
    end;
    { emit operation }
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op, location.register, src1));
  end;
  cg.g_overflowcheck(current_asmdata.CurrAsmList, location, resultdef);
end;

{*****************************************************************************
                               TPPCNOTNODE
*****************************************************************************}

procedure tppcnotnode.pass_generate_code;

begin
  if is_boolean(resultdef) then
  begin
    if not handle_locjump then
    begin
      secondpass(left);
      case left.location.loc of
        LOC_FLAGS:
          begin
            location_copy(location, left.location);
            inverse_flags(location.resflags);
          end;
        LOC_REGISTER, LOC_CREGISTER,
        LOC_REFERENCE, LOC_CREFERENCE,
        LOC_SUBSETREG, LOC_CSUBSETREG,
        LOC_SUBSETREF, LOC_CSUBSETREF:
          begin
            hlcg.location_force_reg(current_asmdata.CurrAsmList, left.location,
              left.resultdef, left.resultdef, true);
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_CMPDI,
              left.location.register, 0));
            location_reset(location, LOC_FLAGS, OS_NO);
            location.resflags.cr := RS_CR0;
            location.resflags.flag := F_EQ;
          end;
      else
        internalerror(2003042401);
      end;
    end;
  end
  else
  begin
    secondpass(left);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, left.location,
      left.resultdef, left.resultdef, true);
    location_copy(location, left.location);
    location.loc := LOC_REGISTER;
    location.register := cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
    { perform the NOT operation }
    cg.a_op_reg_reg(current_asmdata.CurrAsmList, OP_NOT, def_cgsize(resultdef),
      left.location.register,
      location.register);
  end;
end;

begin
  cmoddivnode := tppcmoddivnode;
  cshlshrnode := tppcshlshrnode;
  cunaryminusnode := tppcunaryminusnode;
  cnotnode := tppcnotnode;
end.

