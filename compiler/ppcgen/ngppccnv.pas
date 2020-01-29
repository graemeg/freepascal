{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate PowerPC assembler for type converting nodes

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
unit ngppccnv;

{$i fpcdefs.inc}

interface

    uses
      node,ncnv,ncgcnv;

    type
       tgenppctypeconvnode = class(tcgtypeconvnode)
         protected
         { procedure second_int_to_int;override; }
         { procedure second_string_to_string;override; }
         { procedure second_cstring_to_pchar;override; }
         { procedure second_string_to_chararray;override; }
         { procedure second_array_to_pointer;override; }
         { function first_int_to_real: tnode; override; }
         { procedure second_pointer_to_array;override; }
         { procedure second_chararray_to_string;override; }
         { procedure second_char_to_string;override; }
         { procedure second_int_to_real;override; }
         { procedure second_real_to_real;override; }
         { procedure second_cord_to_pointer;override; }
         { procedure second_proc_to_procvar;override; }
         { procedure second_bool_to_int;override; }
          procedure second_int_to_bool;override;
         { procedure second_load_smallset;override;  }
         { procedure second_ansistring_to_pchar;override; }
         { procedure second_pchar_to_string;override; }
         { procedure second_class_to_intf;override; }
         { procedure second_char_to_char;override; }
       end;


implementation

   uses
      verbose,globtype,globals,systems,
      symconst,symdef,aasmbase,aasmtai,aasmdata,
      defutil,cutils,
      cgbase,cgutils,pass_1,pass_2,
      ncgutil,procinfo,
      cpubase,aasmcpu,
      rgobj,tgobj,cgobj,hlcgobj;


    procedure tgenppctypeconvnode.second_int_to_bool;
      var
        hreg1,
        hreg2    : tregister;
{$ifndef cpu64bitalu}
        href     : treference;
{$endif not cpu64bitalu}
        resflags : tresflags;
        opsize   : tcgsize;
        hlabel   : tasmlabel;
        newsize  : tcgsize;
      begin
         secondpass(left);
         if codegenerror then
          exit;

         { Explicit typecasts from any ordinal type to a boolean type }
         { must not change the ordinal value                          }
         if (nf_explicit in flags) and
            not(left.location.loc in [LOC_FLAGS,LOC_JUMP]) then
           begin
              location_copy(location,left.location);
              newsize:=def_cgsize(resultdef);
              { change of size? change sign only if location is LOC_(C)REGISTER? Then we have to sign/zero-extend }
              if (tcgsize2size[newsize]>tcgsize2size[left.location.size]) or
                 ((newsize<>left.location.size) and (location.loc in [LOC_REGISTER,LOC_CREGISTER])) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,location,left.resultdef,resultdef,true)
              else
                begin
                  location.size:=newsize;
                  if (target_info.endian = ENDIAN_BIG) and
                     (location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                    begin
                      inc(location.reference.offset,TCGSize2Size[left.location.size]-TCGSize2Size[location.size]);
                      location.reference.alignment:=newalignment(location.reference.alignment,TCGSize2Size[left.location.size]-TCGSize2Size[location.size]);
                    end;
                end;
              exit;
           end;

         location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
         opsize := def_cgsize(left.resultdef);
{$ifndef cpu64bitalu}
         if (opsize in [OS_64,OS_S64]) then
           opsize:=OS_32;
{$endif not cpu64bitalu}

        if (left.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

         case left.location.loc of
            LOC_CREFERENCE,LOC_REFERENCE,LOC_REGISTER,LOC_CREGISTER :
              begin
                if left.location.loc in [LOC_CREFERENCE,LOC_REFERENCE] then
                  begin
                    hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
{$ifndef cpu64bitalu}
                    if left.location.size in [OS_64,OS_S64] then
                      begin
                        cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,left.location.reference,hreg1);
                        hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                        href:=left.location.reference;
                        inc(href.offset,4);
                        cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,href,hreg2);
                        cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,hreg1,hreg2,hreg1);
                      end
                    else
{$endif not cpu64bitalu}
                      cg.a_load_ref_reg(current_asmdata.CurrAsmList,opsize,opsize,left.location.reference,hreg1);
                  end
                else
                  begin
                    hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
{$ifndef cpu64bitalu}
                     if left.location.size in [OS_64,OS_S64] then
                       begin
                          hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                          cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.register64.reghi,left.location.register64.reglo,hreg1);
                       end
                     else
{$endif not cpu64bitalu}
                       cg.a_load_reg_reg(current_asmdata.CurrAsmList,opsize,opsize,left.location.register,hreg1);
                  end;
                hreg2 := cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                
                if not(is_cbool(resultdef)) then
                  begin
                    { hreg2:=hreg1-1; carry:=hreg1=0 }
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_SUBIC,hreg2,hreg1,1));
                    { hreg1:=hreg1-hreg2+carry (= hreg1-(hreg1-1)-carry) }
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SUBFE,hreg1,hreg2,hreg1));
                  end
                else
                  begin
                    { carry:=hreg1<>0 }
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_const(A_SUBFIC,hreg2,hreg1,0));
                    { hreg1:=hreg1-hreg1-carry }
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_SUBFE,hreg1,hreg1,hreg1));
                  end;
              end;
            LOC_FLAGS :
              begin
                hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                resflags:=left.location.resflags;
                cg.g_flags2reg(current_asmdata.CurrAsmList,location.size,resflags,hreg1);
                if (is_cbool(resultdef)) then
                  cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,location.size,hreg1,hreg1);
              end;
            LOC_JUMP :
              begin
                hreg1:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                current_asmdata.getjumplabel(hlabel);
                cg.a_label(current_asmdata.CurrAsmList,left.location.truelabel);
                if not(is_cbool(resultdef)) then
                  cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,1,hreg1)
                else
                  cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,-1,hreg1);
                cg.a_jmp_always(current_asmdata.CurrAsmList,hlabel);
                cg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
                cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_INT,0,hreg1);
                cg.a_label(current_asmdata.CurrAsmList,hlabel);
              end;
            else
              internalerror(10062);
         end;
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
{$endif cpu64bitalu}
           location.register:=hreg1;
      end;

end.
