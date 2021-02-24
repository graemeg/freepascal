{
    Copyright (c) 2014 by the Free Pascal development team

    Generate m68k assembler for in memory related nodes

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
unit n68kmem;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      symtype,
      cgbase,cpuinfo,cpubase,
      node,nmem,ncgmem;

    type
       t68kvecnode = class(tcgvecnode)
          procedure update_reference_reg_mul(maybe_const_reg: tregister; regsize: tdef; l: aint); override;
          procedure update_reference_reg_packed(maybe_const_reg: tregister; regsize: tdef; l:aint); override;
          //procedure pass_generate_code;override;
       end;

implementation

    uses
      systems,globals,
      cutils,verbose,
      symdef,paramgr,
      aasmtai,aasmdata,
      nld,ncon,nadd,
      cgutils,cgobj,
      defutil;


{*****************************************************************************
                             T68KVECNODE
*****************************************************************************}

    { this routine must, like any other routine, not change the contents }
    { of base/index registers of references, as these may be regvars.    }
    { The register allocator can coalesce one LOC_REGISTER being moved   }
    { into another (as their live ranges won't overlap), but not a       }
    { LOC_CREGISTER moved into a LOC_(C)REGISTER most of the time (as    }
    { the live range of the LOC_CREGISTER will most likely overlap the   }
    { the live range of the target LOC_(C)REGISTER)                      }
    { The passed register may be a LOC_CREGISTER as well.                }
    procedure t68kvecnode.update_reference_reg_mul(maybe_const_reg: tregister; regsize: tdef; l: aint);
      var
        hreg: tregister;
        scaled: boolean;
        regcgsize: tcgsize;
      begin
        scaled:=false;
        regcgsize:=def_cgsize(regsize);

        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('updref: called')));
        if l<>1 then
          begin
            //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('updref: l <> 1')));
            { if we have a possibility, setup a scalefactor instead of the MUL }
            if not (((CPUM68K_HAS_INDEXSCALE in cpu_capabilities[current_settings.cputype]) and (l in [2,4])) or
               ((CPUM68K_HAS_INDEXSCALE8 in cpu_capabilities[current_settings.cputype]) and (l in [2,4,8]))) then
              begin
                //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('updref: mul')));
                hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
                cg.a_load_reg_reg(current_asmdata.CurrAsmList,regcgsize,OS_ADDR,maybe_const_reg,hreg);
                cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_IMUL,OS_ADDR,l,hreg);
                regcgsize:=OS_ADDR;
                maybe_const_reg:=hreg;
              end
            else
              begin
                //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('updref: scale')));
                scaled:=true;
              end;
          end;

        if (location.reference.base=NR_NO) and not (scaled) and not assigned(location.reference.symbol) then
          begin
            { prefer an address reg, if we will be a base, for indexes any register works }
            if isintregister(maybe_const_reg) then
              begin
                //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('updref: copytoa')));
                hreg:=cg.getaddressregister(current_asmdata.CurrAsmList);
                cg.a_load_reg_reg(current_asmdata.CurrAsmList,regcgsize,OS_ADDR,maybe_const_reg,hreg);
                maybe_const_reg:=hreg;
              end;
            location.reference.base:=maybe_const_reg;
          end
        else
          begin
            if location.reference.index<>NR_NO then
              begin
                { if we already have an index register, dereference the ref to a new base, to be able to insert an index }
                hreg:=cg.getaddressregister(current_asmdata.CurrAsmList);
                cg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,location.reference,hreg);
                reference_reset_base(location.reference,hreg,0,location.reference.temppos,location.reference.alignment,location.reference.volatility);
              end;
            if regcgsize in [OS_8,OS_16] then
              begin
                { index registers are always sign extended on m68k, so we have to zero extend by hand,
                  if the index variable is unsigned, and its width is less than the whole register }
                //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('updref: index zero extend')));
                hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
                cg.a_load_reg_reg(current_asmdata.CurrAsmList,regcgsize,OS_ADDR,maybe_const_reg,hreg);
                maybe_const_reg:=hreg;
              end;
            { insert new index register }
            location.reference.index:=maybe_const_reg;
            if (scaled) then
              location.reference.scalefactor:=l;
          end;
        { update alignment }
        if (location.reference.alignment=0) then
          internalerror(2009020704);
        location.reference.alignment:=newalignment(location.reference.alignment,l);
      end;

     { see remarks for tcgvecnode.update_reference_reg_mul above }
     procedure t68kvecnode.update_reference_reg_packed(maybe_const_reg: tregister; regsize: tdef; l:aint);
       var
         sref: tsubsetreference;
         offsetreg, hreg: tregister;
         alignpower: aint;
         temp : longint;
       begin
         { only orddefs are bitpacked. Even then we only need special code in }
         { case the bitpacked *byte size* is not a power of two, otherwise    }
         { everything can be handled using the the regular array code.        }
         if ((l mod 8) = 0) and
            (ispowerof2(l div 8,temp) or
             not is_ordinal(resultdef)
{$ifndef cpu64bitalu}
             or is_64bitint(resultdef)
{$endif not cpu64bitalu}
             ) then
           begin
             update_reference_reg_mul(maybe_const_reg,regsize,l div 8);
             exit;
           end;
         if (l > 8*sizeof(aint)) then
           internalerror(2006080503);
         sref.ref := location.reference;
         hreg := cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
         cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SUB,OS_INT,tarraydef(left.resultdef).lowrange,maybe_const_reg,hreg);
         cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_IMUL,OS_INT,l,hreg);
         { keep alignment for index }
         sref.ref.alignment := left.resultdef.alignment;
         if not ispowerof2(packedbitsloadsize(l),temp) then
           internalerror(2006081201);
         alignpower:=temp;
         offsetreg := cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
         cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_ADDR,3+alignpower,hreg,offsetreg);
         cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHL,OS_ADDR,alignpower,offsetreg);
         if (sref.ref.base = NR_NO) then
           sref.ref.base := offsetreg
         else if (sref.ref.index = NR_NO) then
           sref.ref.index := offsetreg
         else
           begin
             cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_ADD,OS_ADDR,sref.ref.base,offsetreg);
             sref.ref.base := offsetreg;
           end;
         cg.a_op_const_reg(current_asmdata.CurrAsmList,OP_AND,OS_INT,(1 shl (3+alignpower))-1,hreg);
         sref.bitindexreg := hreg;
         sref.startbit := 0;
         sref.bitlen := resultdef.packedbitsize;
         if (left.location.loc = LOC_REFERENCE) then
           location.loc := LOC_SUBSETREF
         else
           location.loc := LOC_CSUBSETREF;
         location.sref := sref;
       end;

    {procedure t68kvecnode.pass_generate_code;
      begin
        inherited pass_generate_code;
      end;}


begin
   cvecnode:=t68kvecnode;
end.
