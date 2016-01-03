{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate x86-64 assembler for type converting nodes

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
unit nx64cnv;

{$i fpcdefs.inc}

interface

    uses
      node,ncgcnv,defutil,defcmp,pass_1,
      nx86cnv;

    type
       tx8664typeconvnode = class(tx86typeconvnode)
         protected
         { procedure second_int_to_int;override; }
         { procedure second_string_to_string;override; }
         { procedure second_cstring_to_pchar;override; }
         { procedure second_string_to_chararray;override; }
         { procedure second_array_to_pointer;override; }
         { procedure second_pointer_to_array;override; }
         { procedure second_chararray_to_string;override; }
         { procedure second_char_to_string;override; }
         { function first_int_to_real: tnode; override; }
         function first_int_to_real : tnode;override;
         procedure second_int_to_real;override;
         { procedure second_real_to_real;override; }
         { procedure second_cord_to_pointer;override; }
         { procedure second_proc_to_procvar;override; }
         { procedure second_bool_to_int;override; }
         { procedure second_int_to_bool;override; }
         { procedure second_load_smallset;override;  }
         { procedure second_ansistring_to_pchar;override; }
         { procedure second_pchar_to_string;override; }
         { procedure second_class_to_intf;override;  }
         { procedure second_char_to_char;override; }
       end;


implementation

    uses
      verbose,systems,globals,globtype,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      symconst,symdef,
      cgbase,cga,procinfo,pass_2,
      ncon,ncal,ncnv,
      cpubase,
<<<<<<< HEAD
<<<<<<< HEAD
      cgutils,cgobj,hlcgobj,cgx86,ncgutil,
=======
      cgutils,cgobj,cgx86,ncgutil,
>>>>>>> graemeg/fixes_2_2
=======
      cgutils,cgobj,cgx86,ncgutil,
>>>>>>> origin/fixes_2_2
      tgobj;


    function tx8664typeconvnode.first_int_to_real : tnode;
      begin
        result:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
        if use_vectorfpu(resultdef) and
=======
        if use_sse(resultdef) and
>>>>>>> graemeg/fixes_2_2
=======
        if use_sse(resultdef) and
>>>>>>> origin/fixes_2_2
           (torddef(left.resultdef).ordtype=u32bit) then
          begin
            inserttypeconv(left,s64inttype);
            firstpass(left);
          end
        else
          result:=inherited first_int_to_real;
<<<<<<< HEAD
<<<<<<< HEAD
       if use_vectorfpu(resultdef) then
=======
       if use_sse(resultdef) then
>>>>>>> graemeg/fixes_2_2
=======
       if use_sse(resultdef) then
>>>>>>> origin/fixes_2_2
         expectloc:=LOC_MMREGISTER;
      end;


    procedure tx8664typeconvnode.second_int_to_real;
      var
         href : treference;
         l1,l2 : tasmlabel;
         op : tasmop;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        if use_vectorfpu(resultdef) then
=======
        if use_sse(resultdef) then
>>>>>>> graemeg/fixes_2_2
=======
        if use_sse(resultdef) then
>>>>>>> origin/fixes_2_2
          begin
            if is_double(resultdef) then
              op:=A_CVTSI2SD
            else if is_single(resultdef) then
              op:=A_CVTSI2SS
            else
              internalerror(200506061);

            location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);

            case torddef(left.resultdef).ordtype of
              u64bit:
                begin
                   { unsigned 64 bit ints are harder to handle:
                     we load bits 0..62 and then check bit 63:
                     if it is 1 then we add $80000000 000000000
                     as double                                  }
<<<<<<< HEAD
<<<<<<< HEAD
                   current_asmdata.getglobaldatalabel(l1);
=======
                   current_asmdata.getdatalabel(l1);
>>>>>>> graemeg/fixes_2_2
=======
                   current_asmdata.getdatalabel(l1);
>>>>>>> origin/fixes_2_2
                   current_asmdata.getjumplabel(l2);

                   { Get sign bit }
                   if not(left.location.loc in [LOC_REGISTER,LOC_REFERENCE]) then
<<<<<<< HEAD
<<<<<<< HEAD
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
=======
                     location_force_reg(current_asmdata.CurrAsmList,left.location,left.location.size,false);
>>>>>>> graemeg/fixes_2_2
=======
                     location_force_reg(current_asmdata.CurrAsmList,left.location,left.location.size,false);
>>>>>>> origin/fixes_2_2
                   case left.location.loc of
                     LOC_REGISTER :
                       begin
                         emit_const_reg(A_BT,S_Q,63,left.location.register);
                         current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_Q,left.location.register,location.register));
                       end;
                     LOC_REFERENCE :
                       begin
<<<<<<< HEAD
<<<<<<< HEAD
                         href:=left.location.reference;
                         tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
                         inc(href.offset,4);
                         emit_const_ref(A_BT,S_L,31,href);
                         dec(href.offset,4);
                         current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_Q,href,location.register));
=======
=======
>>>>>>> origin/fixes_2_2
                         inc(left.location.reference.offset,4);
                         emit_const_ref(A_BT,S_L,31,left.location.reference);
                         dec(left.location.reference.offset,4);
                         current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_Q,left.location.reference,location.register));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                       end;
                     else
                       internalerror(200710181);
                   end;

                   cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NC,l2);
<<<<<<< HEAD
<<<<<<< HEAD
                   new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,l1.name,const_align(sizeof(pint)));
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
                   reference_reset_symbol(href,l1,0,4);
                   { simplify for PIC }
                   tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
=======
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
                   reference_reset_symbol(href,l1,0);
>>>>>>> graemeg/fixes_2_2
=======
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
                   reference_reset_symbol(href,l1,0);
>>>>>>> origin/fixes_2_2

                   { I got these constant from a test program (FK) }
                   if is_double(resultdef) then
                     begin
                       { double (2^64) }
                       current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit(0));
                       current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit($43f00000));
<<<<<<< HEAD
<<<<<<< HEAD
=======
                       { simplify for PIC }
>>>>>>> graemeg/fixes_2_2
=======
                       { simplify for PIC }
>>>>>>> origin/fixes_2_2
                       tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
                       current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_ADDSD,S_NO,href,location.register));
                     end
                   else if is_single(resultdef) then
                     begin
                       { single(2^64) }
                       current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit($5f800000));
<<<<<<< HEAD
<<<<<<< HEAD
=======
                       { simplify for PIC }
                       tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
>>>>>>> graemeg/fixes_2_2
=======
                       { simplify for PIC }
                       tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
>>>>>>> origin/fixes_2_2
                       current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_ADDSS,S_NO,href,location.register));
                     end
                   else
                     internalerror(200506071);
                   cg.a_label(current_asmdata.CurrAsmList,l2);
                end
              else
                inherited second_int_to_real;
            end;
          end
        else
          inherited second_int_to_real;
      end;


begin
   ctypeconvnode:=tx8664typeconvnode;
end.
