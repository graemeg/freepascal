{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate i8086 assembler for in memory related nodes

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
unit n8086mem;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      symtype,
      cgbase,cpuinfo,cpubase,
      node,nmem,ncgmem,nx86mem,ni86mem;

    type
       ti8086addrnode = class(ti86addrnode)
        protected
         procedure set_labelsym_resultdef; override;
         procedure set_absvarsym_resultdef; override;
         procedure pass_generate_code;override;
        public
         get_offset_only: boolean;
       end;

       ti8086derefnode = class(tx86derefnode)
         procedure pass_generate_code;override;
       end;

       { tx86vecnode doesn't work for i8086, so we inherit tcgvecnode }
       ti8086vecnode = class(tcgvecnode)
        protected
         function first_arraydef: tnode;override;
         procedure update_reference_reg_mul(maybe_const_reg: tregister; regsize: tdef; l: aint);override;
       end;

implementation

    uses
      systems,globals,constexp,
      cutils,verbose,
      symbase,symconst,symdef,symtable,symsym,symx86,symcpu,
      parabase,paramgr,
      aasmtai,aasmdata,
      nld,ncon,nadd,ncal,ncnv,
      cgutils,cgobj,
      defutil,hlcgobj,
      pass_1,pass_2,ncgutil;

{*****************************************************************************
                             TI8086ADDRNODE
*****************************************************************************}

    procedure ti8086addrnode.set_labelsym_resultdef;
      begin
        if anf_ofs in addrnodeflags then
          resultdef:=voidnearcspointertype
        else
          inherited;
      end;


    procedure ti8086addrnode.set_absvarsym_resultdef;
      begin
        if not(anf_typedaddr in addrnodeflags) then
          resultdef:=voidfarpointertype
        else
          resultdef:=tcpupointerdefclass(cpointerdef).createx86(left.resultdef,x86pt_far);
      end;


    procedure ti8086addrnode.pass_generate_code;
      begin
        if get_offset_only then
          begin
            secondpass(left);

            location_reset(location,LOC_REGISTER,OS_16);
            location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,voidnearpointertype);
            if not(left.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
              internalerror(2015103001);
            hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,left.resultdef,voidnearpointertype,left.location.reference,location.register);
          end
        else
          inherited;
      end;

{*****************************************************************************
                             TI8086DEREFNODE
*****************************************************************************}

    procedure ti8086derefnode.pass_generate_code;
      var
        paraloc1 : tcgpara;
        pd : tprocdef;
        sym : tsym;
        st : tsymtable;
        tmpref: treference;
      begin
        if tcpupointerdef(left.resultdef).x86pointertyp in [x86pt_far,x86pt_huge] then
          begin
            secondpass(left);
            { assume natural alignment, except for packed records }
            if not(resultdef.typ in [recorddef,objectdef]) or
               (tabstractrecordsymtable(tabstractrecorddef(resultdef).symtable).usefieldalignment<>1) then
              location_reset_ref(location,LOC_REFERENCE,def_cgsize(resultdef),resultdef.alignment,[])
            else
              location_reset_ref(location,LOC_REFERENCE,def_cgsize(resultdef),1,[]);
            if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER,LOC_CREFERENCE,LOC_REFERENCE,LOC_CONSTANT]) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            case left.location.loc of
               LOC_CREGISTER,
               LOC_REGISTER:
                 begin
                   hlcg.maybe_change_load_node_reg(current_asmdata.CurrAsmList,left,true);
                   location.reference.base := left.location.register;
                   location.reference.segment := cg.GetNextReg(left.location.register);
                 end;
               LOC_CREFERENCE,
               LOC_REFERENCE:
                 begin
                    location.reference.base:=cg.getaddressregister(current_asmdata.CurrAsmList);
                    cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_16,OS_16,left.location.reference,location.reference.base);
                    location.reference.segment:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
                    tmpref:=left.location.reference;
                    inc(tmpref.offset,2);
                    cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_16,OS_16,tmpref,location.reference.segment);
                 end;
               LOC_CONSTANT:
                 begin
                   location.reference.offset:=left.location.value and $FFFF;
                   location.reference.segment:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
                   cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_16,(left.location.value shr 16) and $FFFF,location.reference.segment);
                 end;
               else
                 internalerror(200507031);
            end;
            if (cs_use_heaptrc in current_settings.globalswitches) and
               (cs_checkpointer in current_settings.localswitches) and
               not(cs_compilesystem in current_settings.moduleswitches) and
   {$ifdef x86}
               (tcpupointerdef(left.resultdef).x86pointertyp = tcpupointerdefclass(cpointerdef).default_x86_data_pointer_type) and
   {$endif x86}
               not(nf_no_checkpointer in flags) and
               { can be NR_NO in case of LOC_CONSTANT }
               (location.reference.base<>NR_NO) then
             begin
               if not searchsym_in_named_module('HEAPTRC','CHECKPOINTER',sym,st) or
                  (sym.typ<>procsym) then
                 internalerror(2012010601);
               pd:=tprocdef(tprocsym(sym).ProcdefList[0]);
               paraloc1.init;
               paramanager.getcgtempparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);
               hlcg.a_loadaddr_ref_cgpara(current_asmdata.CurrAsmList,resultdef,location.reference,paraloc1);
               paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
               paraloc1.done;
               hlcg.allocallcpuregisters(current_asmdata.CurrAsmList);
               hlcg.a_call_name(current_asmdata.CurrAsmList,pd,'FPC_CHECKPOINTER',[],nil,false);
               hlcg.deallocallcpuregisters(current_asmdata.CurrAsmList);
               system.include(current_settings.moduleswitches,cs_checkpointer_called);
             end;
          end
        else
          inherited pass_generate_code;
      end;

{*****************************************************************************
                             TI8086VECNODE
*****************************************************************************}

    function ti8086vecnode.first_arraydef: tnode;
      var
        arraydef: tcpuarraydef;
        procname:string;
      begin
        if tcpuarraydef(left.resultdef).is_huge then
          begin
            arraydef:=tcpuarraydef(left.resultdef);

            if not (ado_IsConvertedPointer in arraydef.arrayoptions) then
              internalerror(2014080701);

            if left.nodetype<>typeconvn then
              internalerror(2014080702);

            procname:='fpc_hugeptr_add_longint';
            if cs_hugeptr_arithmetic_normalization in current_settings.localswitches then
              procname:=procname+'_normalized';

            if arraydef.elementdef.size>1 then
              right:=caddnode.create(muln,right,
                cordconstnode.create(arraydef.elementdef.size,s32inttype,true));

            result:=ccallnode.createintern(procname,
              ccallparanode.create(right,
              ccallparanode.create(ttypeconvnode(left).left,nil)));
            inserttypeconv_internal(result,tx86pointerdef(cpointerdef).getreusablex86(arraydef.elementdef,x86pt_huge));
            result:=cderefnode.create(result);

            ttypeconvnode(left).left:=nil;
            ttypeconvnode(left).free;
            left := nil;
            right := nil;
            firstpass(result);
          end
        else
          result:=inherited;
      end;


    procedure ti8086vecnode.update_reference_reg_mul(maybe_const_reg: tregister; regsize: tdef; l: aint);
      var
        saveseg: TRegister;
      begin
        saveseg:=location.reference.segment;
        location.reference.segment:=NR_NO;
        inherited;
        location.reference.segment:=saveseg;
      end;

begin
  caddrnode:=ti8086addrnode;
  cderefnode:=ti8086derefnode;
  cvecnode:=ti8086vecnode;
end.
