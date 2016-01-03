{
    Copyright (c) 1998-2012 by Florian Klaempfl and others

    Generate i386 assembler for constants

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
unit nx86con;

{$i fpcdefs.inc}

interface

    uses
       node,ncon,ncgcon;

    type
       tx86realconstnode = class(tcgrealconstnode)
          function pass_1 : tnode;override;
          procedure pass_generate_code;override;
       end;

implementation

    uses
      systems,globals,
      symdef,
      defutil,
      cpubase,
      cga,cgx86,cgobj,cgbase,cgutils;

{*****************************************************************************
                           TX86REALCONSTNODE
*****************************************************************************}

    function tx86realconstnode.pass_1 : tnode;
      begin
         result:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if is_number_float(value_real) and not(use_vectorfpu(resultdef)) and ((value_real=1.0) or (value_real=0.0)) then
=======
         if is_number_float(value_real) and not(use_vectorfpu(resultdef)) and (value_real=1.0) or (value_real=0.0) then
>>>>>>> graemeg/cpstrnew
=======
         if is_number_float(value_real) and not(use_vectorfpu(resultdef)) and (value_real=1.0) or (value_real=0.0) then
>>>>>>> graemeg/cpstrnew
=======
         if is_number_float(value_real) and not(use_vectorfpu(resultdef)) and (value_real=1.0) or (value_real=0.0) then
>>>>>>> graemeg/cpstrnew
=======
         if is_number_float(value_real) and not(use_vectorfpu(resultdef)) and (value_real=1.0) or (value_real=0.0) then
>>>>>>> origin/cpstrnew
           expectloc:=LOC_FPUREGISTER
         else
           expectloc:=LOC_CREFERENCE;
      end;

    procedure tx86realconstnode.pass_generate_code;

      begin
         if is_number_float(value_real) then
           begin
             if (value_real=1.0) and not(use_vectorfpu(resultdef)) then
               begin
                  emit_none(A_FLD1,S_NO);
                  location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
                  location.register:=NR_ST;
                  tcgx86(cg).inc_fpu_stack;
               end
             else if (value_real=0.0) and not(use_vectorfpu(resultdef)) then
               begin
                  emit_none(A_FLDZ,S_NO);
                  if (get_real_sign(value_real) < 0) then
                    emit_none(A_FCHS,S_NO);
                  location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
                  location.register:=NR_ST;
                  tcgx86(cg).inc_fpu_stack;
               end
            else
              inherited pass_generate_code;
           end
         else
           inherited pass_generate_code;
      end;


begin
   crealconstnode:=tx86realconstnode;
end.
