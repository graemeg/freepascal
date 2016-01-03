{
    Copyright (c) 1998-2009 by Florian Klaempfl and David Zhang

    Generate MIPSEL assembler for in call nodes

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
unit ncpucall;

{$i fpcdefs.inc}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  node, ncgcal;

type
  tMIPSELcallnode = class(tcgcallnode)
    function  pass_1 : tnode; override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ncgcal;

type
  tMIPSELcallnode = class(tcgcallnode)
    procedure extra_call_code; override;
    procedure extra_post_call_code; override;
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


implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  globtype,cpubase,procinfo,
=======
  cpubase,
>>>>>>> graemeg/cpstrnew
=======
  cpubase,
>>>>>>> graemeg/cpstrnew
=======
  cpubase,
>>>>>>> graemeg/cpstrnew
=======
  cpubase,
>>>>>>> origin/cpstrnew
  aasmtai,aasmcpu,aasmdata,
  paramgr,
  ncal;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TMIPSELcallnode.pass_1 : tnode;
begin
  pass_1 := inherited pass_1;
  if assigned(current_procinfo) and
     assigned(procdefinition) and
     (procdefinition.proccalloption=pocall_cdecl) then
    include(current_procinfo.flags,pi_needs_got);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure tMIPSELcallnode.extra_call_code;
begin
  if pushedparasize > 0 then
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ADDIU, NR_STACK_POINTER_REG, NR_STACK_POINTER_REG, -pushedparasize));
end;

procedure tMIPSELcallnode.extra_post_call_code;
begin
  if pushedparasize > 0 then
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_ADDIU, NR_STACK_POINTER_REG, NR_STACK_POINTER_REG, pushedparasize));

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
  ccallnode := TMIPSELCallNode;
end.
