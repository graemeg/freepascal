{
    Copyright (c) 2002-2014 by Florian Klaempfl

    Generate i8086 assembler for load nodes

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
unit n8086ld;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      symsym,symtype,
      node,ncgld,nx86ld,aasmbase;

    type

      { ti8086loadnode }

      ti8086loadnode = class(tx86loadnode)
        protected
         procedure generate_nested_access(vs: tsym); override;
         procedure generate_absaddr_access(vs: tabsolutevarsym); override;
         procedure generate_threadvar_access(gvs: tstaticvarsym); override;
        public
         procedure pass_generate_code;override;
      end;


implementation

    uses
      globals,verbose,aasmdata,defutil,
      symconst,symdef,symtable,symcpu,
      nld,
      cgbase,cgobj,cgutils,
      hlcgobj,
      cpubase,cpuinfo,
      parabase,paramgr;

{*****************************************************************************
                            TI8086LOADNODE
*****************************************************************************}

    procedure ti8086loadnode.generate_nested_access(vs: tsym);
      begin
        inherited;

        { the parentfp pointer is always a near pointer (this is turbo pascal
          compatible) regardless of memory model, so we need to set the segment
          manually.

          todo: once the far data memory models are fully implemented, the
          parentfp type should be changed to a near 'ss' pointer in all memory
          models and then this code can be removed. But this can only happen
          after:
          1) all calls to a_loadaddr_ref_reg go through the high level code
             generator
          2) a_loadaddr_ref_reg in the low level code generator stops using
             the presence of a segment in the source reference to determine the
             destination reg size
          3) make_simple_ref is updated to remove unnecessary segment prefixes
          4) hlcg.reference_reset_base is updated to set the segment on near_ss
             pointers }
        if (left.nodetype=loadparentfpn) and
           (current_settings.x86memorymodel in x86_far_data_models) then
          location.reference.segment:=NR_SS;
      end;

    procedure ti8086loadnode.generate_absaddr_access(vs: tabsolutevarsym);
      begin
        if tcpuabsolutevarsym(symtableentry).absseg then
          begin
            location.reference.segment:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
            cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_16,aint(tcpuabsolutevarsym(symtableentry).addrsegment),location.reference.segment);
          end;
        inherited;
      end;

    procedure ti8086loadnode.generate_threadvar_access(gvs: tstaticvarsym);
      var
        segref: treference;
        segreg: TRegister;
        newsize: TCgSize;
        norelocatelab: TAsmLabel;
        endrelocatelab: TAsmLabel;
        pvd: tdef;
        paraloc1 : tcgpara;
        hregister: TRegister;
        href: treference;
      begin
        if current_settings.x86memorymodel=mm_huge then
          begin
            if (cs_compilesystem in current_settings.moduleswitches) then
              begin
                inherited generate_threadvar_access(gvs);
                exit;
              end;

            { we don't know the size of all arrays }
            newsize:=def_cgsize(resultdef);
            { alignment is overridden per case below }
            location_reset_ref(location,LOC_REFERENCE,newsize,resultdef.alignment,[]);

            {
              Thread var loading is optimized to first check if
              a relocate function is available. When the function
              is available it is called to retrieve the address.
              Otherwise the address is loaded with the symbol

              The code needs to be in the order to first handle the
              call and then the address load to be sure that the
              register that is used for returning is the same (PFV)
            }
            current_asmdata.getjumplabel(norelocatelab);
            current_asmdata.getjumplabel(endrelocatelab);
            { make sure hregister can't allocate the register necessary for the parameter }
            pvd:=search_system_type('TRELOCATETHREADVARHANDLER').typedef;
            if pvd.typ<>procvardef then
              internalerror(2012120901);
            paraloc1.init;
            paramanager.getcgtempparaloc(current_asmdata.CurrAsmList,tprocvardef(pvd),1,paraloc1);
            hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,pvd);
            segreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
            reference_reset_symbol(segref,current_asmdata.RefAsmSymbol('FPC_THREADVAR_RELOCATE',AT_DATA),0,pvd.alignment,[]);
            segref.refaddr:=addr_seg;
            cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_16,OS_16,segref,segreg);
            reference_reset_symbol(href,current_asmdata.RefAsmSymbol('FPC_THREADVAR_RELOCATE',AT_DATA),0,pvd.alignment,[]);
            href.segment:=segreg;
            hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,pvd,pvd,href,hregister);
            hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,pvd,OC_EQ,0,hregister,norelocatelab);
            { don't save the allocated register else the result will be destroyed later }
            if not(vo_is_weak_external in gvs.varoptions) then
              reference_reset_symbol(href,current_asmdata.RefAsmSymbol(gvs.mangledname,AT_DATA),0,2,[])
            else
              reference_reset_symbol(href,current_asmdata.WeakRefAsmSymbol(gvs.mangledname,AT_DATA),0,2,[]);
            cg.a_load_ref_cgpara(current_asmdata.CurrAsmList,OS_16,href,paraloc1);
            paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
            paraloc1.done;
            cg.allocallcpuregisters(current_asmdata.CurrAsmList);
            cg.a_call_reg(current_asmdata.CurrAsmList,hregister);
            cg.deallocallcpuregisters(current_asmdata.CurrAsmList);
            cg.getcpuregister(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
            cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
            hregister:=hlcg.getaddressregister(current_asmdata.CurrAsmList,voidpointertype);
            cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_ADDR,NR_FUNCTION_RESULT_REG,hregister);
            cg.a_jmp_always(current_asmdata.CurrAsmList,endrelocatelab);
            cg.a_label(current_asmdata.CurrAsmList,norelocatelab);
            { no relocation needed, load the address of the variable only, the
              layout of a threadvar is (4 bytes pointer):
                0 - Threadvar index
                4 - Threadvar value in single threading }
            if not(vo_is_weak_external in gvs.varoptions) then
              reference_reset_symbol(href,current_asmdata.RefAsmSymbol(gvs.mangledname,AT_DATA),sizeof(pint),2,[])
            else
              reference_reset_symbol(href,current_asmdata.WeakRefAsmSymbol(gvs.mangledname,AT_DATA),sizeof(pint),2,[]);
            hlcg.a_loadaddr_ref_reg(current_asmdata.CurrAsmList,resultdef,voidpointertype,href,hregister);
            cg.a_label(current_asmdata.CurrAsmList,endrelocatelab);
            hlcg.reference_reset_base(location.reference,voidpointertype,hregister,0,ctempposinvalid,location.reference.alignment,[]);
          end
        else
          inherited generate_threadvar_access(gvs);
      end;

    procedure ti8086loadnode.pass_generate_code;
      var
        gvs: tstaticvarsym;
        segref: treference;
        refsym: TAsmSymbol;
        segreg: TRegister;
        newsize: TCgSize;
      begin
        case symtableentry.typ of
          staticvarsym:
            begin
              gvs:=tstaticvarsym(symtableentry);
              if (vo_is_dll_var in gvs.varoptions) then
              { DLL variable }
                begin
                  inherited pass_generate_code;
                  exit;
                end
              { Thread variable }
              else if (vo_is_thread_var in gvs.varoptions) then
                begin
                  { this will be handled in ti8086loadnode.generate_threadvar_access }
                  inherited pass_generate_code;
                  exit;
                end
              { Normal (or external) variable }
              else
                begin
                  if ((current_settings.x86memorymodel<>mm_huge) and not (vo_is_far in gvs.varoptions)) or
                     (not (vo_is_external in gvs.varoptions) and gvs.Owner.iscurrentunit) then
                    begin
                      inherited pass_generate_code;
                      if (location.loc<>LOC_REFERENCE) and (location.loc<>LOC_CREFERENCE) then
                        internalerror(2017121101);
                      location.reference.segment:=NR_DS;
                      exit;
                    end;

                  { we don't know the size of all arrays }
                  newsize:=def_cgsize(resultdef);
                  { alignment is overridden per case below }
                  location_reset_ref(location,LOC_REFERENCE,newsize,resultdef.alignment,[]);

                  if gvs.localloc.loc=LOC_INVALID then
                    begin
                      if not(vo_is_weak_external in gvs.varoptions) then
                        refsym:=current_asmdata.RefAsmSymbol(gvs.mangledname,AT_DATA)
                      else
                        refsym:=current_asmdata.WeakRefAsmSymbol(gvs.mangledname,AT_DATA);

                      segreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);

                      reference_reset_symbol(segref,refsym,0,0,[]);
                      segref.refaddr:=addr_seg;
                      cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_16,OS_16,segref,segreg);

                      reference_reset_symbol(location.reference,refsym,0,location.reference.alignment,[]);
                      location.reference.segment:=segreg;
                    end
                  else
                    location:=gvs.localloc;
                end;

              { make const a LOC_CREFERENCE }
              if (gvs.varspez=vs_const) and
                 (location.loc=LOC_REFERENCE) then
                location.loc:=LOC_CREFERENCE;
            end;
          procsym:
            begin
              inherited pass_generate_code;
              if current_settings.x86memorymodel in x86_near_code_models then
                begin
                  if (location.loc=LOC_REFERENCE) or (location.loc=LOC_CREFERENCE) then
                    location.reference.segment:=NR_CS;
                end;
            end;
          else
            inherited pass_generate_code;
        end;
      end;


begin
   cloadnode:=ti8086loadnode;
end.
