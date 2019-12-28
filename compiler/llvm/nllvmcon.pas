{
    Copyright (c) 2013 by Jonas Maebe, member of the Free Pascal Compiler
    development team

    Generate llvm bitcode for constants

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
unit nllvmcon;

{$i fpcdefs.inc}

interface

    uses
      symtype,
      node,ncgcon;

    type
       tllvmrealconstnode = class(tcgrealconstnode)
          function pass_1 : tnode;override;
          procedure pass_generate_code;override;
       end;

       tllvmstringconstnode = class(tcgstringconstnode)
          procedure pass_generate_code; override;
       protected
          procedure load_dynstring(const strpointerdef: tdef; const elementdef: tdef; const winlikewidestring: boolean); override;
       end;

implementation

    uses
      globtype,globals,verbose,cutils,
      symbase,symtable,symconst,symdef,symsym,defutil,
      aasmdata,aasmcnst,
      ncon,
      llvmbase,aasmllvm,hlcgobj,
      cgbase,cgutils;

{*****************************************************************************
                           tllvmstringconstnode
*****************************************************************************}

    procedure tllvmstringconstnode.pass_generate_code;
      var
        datadef, resptrdef: tdef;
        hreg: tregister;
      begin
        inherited pass_generate_code;
        if cst_type in [cst_conststring,cst_shortstring] then
          begin
            if location.loc<>LOC_CREFERENCE then
              internalerror(2014071202);
            case cst_type of
              cst_conststring:
                { this kind of string const is used both for array of char
                  constants (-> excludes terminating #0) and pchars (-> includes
                  terminating #0). The resultdef excludes the #0 while the data
                  includes it -> insert typecast from datadef to resultdef }
                datadef:=carraydef.getreusable(cansichartype,len+1);
              cst_shortstring:
                { the resultdef of the string constant is the type of the
                  string to which it is assigned, which can be longer or shorter
                  than the length of the string itself -> typecast it to the
                  correct string type }
                datadef:=carraydef.getreusable(cansichartype,min(len,255)+1);
              else
                internalerror(2014071203);
            end;
            { get address of array as pchar }
            resptrdef:=cpointerdef.getreusable(resultdef);
            hreg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,resptrdef);
            hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,datadef,resptrdef,location.reference,hreg);
            hlcg.reference_reset_base(location.reference,resptrdef,hreg,0,location.reference.temppos,location.reference.alignment,location.reference.volatility);
          end;
      end;


    procedure tllvmstringconstnode.load_dynstring(const strpointerdef: tdef; const elementdef: tdef; const winlikewidestring: boolean);
      var
        stringtype: tstringtype;
        strrecdef: trecorddef;
        offset: pint;
        field: tfieldvarsym;
        llvmfield: tllvmshadowsymtableentry;
        dataptrdef: tdef;
        reg: tregister;
        href: treference;
      begin
        case cst_type of
          cst_ansistring:
            stringtype:=st_ansistring;
          cst_unicodestring:
            stringtype:=st_unicodestring;
          cst_widestring:
            stringtype:=st_widestring;
          else
            internalerror(2014040804);
        end;
        { get the recorddef for this string constant }
        strrecdef:=ctai_typedconstbuilder.get_dynstring_rec(stringtype,winlikewidestring,len);
        { offset in the record of the the string data }
        offset:=ctai_typedconstbuilder.get_string_symofs(stringtype,winlikewidestring);
        { field corresponding to this offset }
        field:=trecordsymtable(strrecdef.symtable).findfieldbyoffset(offset);
        llvmfield:=trecordsymtable(strrecdef.symtable).llvmst[field];
        if llvmfield.fieldoffset<>field.fieldoffset then
          internalerror(2015061001);
        { pointerdef to the string data array }
        dataptrdef:=cpointerdef.getreusable(field.vardef);
        { load the address of the string data }
        reg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,dataptrdef);
        reference_reset_symbol(href,lab_str,0,const_align(strpointerdef.size),[]);
        current_asmdata.CurrAsmList.concat(
          taillvm.getelementptr_reg_size_ref_size_const(reg,cpointerdef.getreusable(strrecdef),href,
          s32inttype,field.llvmfieldnr,true));
        { convert into a pointer to the individual elements }
        hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,dataptrdef,strpointerdef,reg,location.register);
      end;

{*****************************************************************************
                           tllvmrealconstnode
*****************************************************************************}

    function tllvmrealconstnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_FPUREGISTER;
      end;


    procedure tllvmrealconstnode.pass_generate_code;
      begin
         { llvm supports floating point constants directly }
         location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
         location.register:=hlcg.getfpuregister(current_asmdata.CurrAsmList,resultdef);
         case tfloatdef(resultdef).floattype of
           s32real,s64real:
             current_asmdata.CurrAsmList.concat(taillvm.op_reg_size_fpconst_size(la_bitcast,location.register,resultdef,value_real,resultdef));
           { comp and currency are handled as int64 at the llvm level }
           s64comp:
             { sc80floattype instead of resultdef, see comment in thlcgllvm.a_loadfpu_ref_reg }
             current_asmdata.CurrAsmList.concat(taillvm.op_reg_size_const_size(la_sitofp,location.register,s64inttype,trunc(value_real),sc80floattype));
           s64currency:
             { sc80floattype instead of resultdef, see comment in thlcgllvm.a_loadfpu_ref_reg }
             current_asmdata.CurrAsmList.concat(taillvm.op_reg_size_const_size(la_sitofp,location.register,s64inttype,round(value_real),sc80floattype));
{$ifdef cpuextended}
           s80real,sc80real:
             current_asmdata.CurrAsmList.concat(taillvm.op_reg_size_fpconst80_size(la_bitcast,location.register,resultdef,value_real,resultdef));
{$endif cpuextended}
           else
             internalerror(2013102501);
         end;
      end;


begin
   cstringconstnode:=tllvmstringconstnode;
   crealconstnode:=tllvmrealconstnode;
end.
