{
    Copyright (c) 1998-2002,2015 by Florian Klaempfl

    Generate x86 assembler for in load nodes

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
unit nx86ld;

{$i fpcdefs.inc}

interface
    uses
      globtype,
      symtype,symsym,
      node,nld,ncgld;

    type
      tx86loadnode = class(tcgloadnode)
       protected
         procedure generate_threadvar_access(gvs: tstaticvarsym); override;
      end;

implementation

    uses
      globals,
      cutils,verbose,systems,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      cgutils,cgobj,
      symconst,symdef,symtable,
      cgbase,cpubase,parabase,paramgr,
      procinfo;

{*****************************************************************************
                           TX86LOADNODE
*****************************************************************************}

    procedure tx86loadnode.generate_threadvar_access(gvs: tstaticvarsym);
      var
        paraloc1 : tcgpara;
        pd: tprocdef;
        href: treference;
        hregister : tregister;
        handled: boolean;
      begin
        handled:=false;
        if (tf_section_threadvars in target_info.flags) then
          begin
            if target_info.system in [system_i386_win32,system_x86_64_win64] then
              begin
                paraloc1.init;
                pd:=search_system_proc('fpc_tls_add');
                paramanager.getcgtempparaloc(current_asmdata.CurrAsmList,pd,1,paraloc1);
                if not(vo_is_weak_external in gvs.varoptions) then
                  reference_reset_symbol(href,current_asmdata.RefAsmSymbol(gvs.mangledname,AT_DATA,use_indirect_symbol(gvs)),0,sizeof(pint),[])
                else
                  reference_reset_symbol(href,current_asmdata.WeakRefAsmSymbol(gvs.mangledname,AT_DATA),0,sizeof(pint),[]);
                cg.a_loadaddr_ref_cgpara(current_asmdata.CurrAsmList,href,paraloc1);
                paramanager.freecgpara(current_asmdata.CurrAsmList,paraloc1);
                paraloc1.done;

                cg.g_call(current_asmdata.CurrAsmList,'FPC_TLS_ADD');
                cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);
                hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,NR_FUNCTION_RESULT_REG,hregister);
                location.reference.base:=hregister;
                handled:=true;
              end;
          end;

        if not handled then
          inherited;

        if (tf_section_threadvars in target_info.flags) then
          begin
{$ifdef i386}
            case target_info.system of
              system_i386_linux,system_i386_android:
                begin
                  case current_settings.tlsmodel of
                    tlsm_local_exec:
                      begin
                        location.reference.segment:=NR_GS;
                        location.reference.refaddr:=addr_ntpoff;
                      end;
                    tlsm_global_dynamic:
                      begin
                        include(current_procinfo.flags,pi_needs_got);
                        reference_reset(href,0,[]);
                        location.reference.index:=current_procinfo.got;
                        location.reference.scalefactor:=1;
                        location.reference.refaddr:=addr_tlsgd;
                        cg.getcpuregister(current_asmdata.CurrAsmList,NR_EAX);
                        current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_LEA,S_L,location.reference,NR_EAX));
                        cg.g_call(current_asmdata.CurrAsmList,'___tls_get_addr');
                        cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EAX);
                        hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                        cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,NR_EAX,hregister);
                        reference_reset(location.reference,location.reference.alignment,location.reference.volatility);
                        location.reference.base:=hregister;
                      end;
                    else
                      Internalerror(2018110401);
                  end;
                end;
              else
                ;
            end;
{$endif i386}
{$ifdef x86_64}
            case target_info.system of
              system_x86_64_linux:
                begin
                  case current_settings.tlsmodel of
                    tlsm_local_exec:
                      begin
                        location.reference.segment:=NR_FS;
                        location.reference.refaddr:=addr_tpoff;
                      end;
                    tlsm_global_dynamic:
                      begin
                        current_asmdata.CurrAsmList.concat(tai_const.Create_8bit($66));
                        reference_reset(href,0,[]);
                        location.reference.base:=NR_RIP;
                        location.reference.scalefactor:=1;
                        location.reference.refaddr:=addr_tlsgd;
                        cg.getcpuregister(current_asmdata.CurrAsmList,NR_RDI);
                        current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_LEA,S_Q,location.reference,NR_RDI));
                        current_asmdata.CurrAsmList.concat(tai_const.Create_8bit($66));
                        current_asmdata.CurrAsmList.concat(tai_const.Create_8bit($66));
                        current_asmdata.CurrAsmList.concat(tai_const.Create_8bit($48));
                        cg.g_call(current_asmdata.CurrAsmList,'__tls_get_addr');
                        cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_RDI);
                        cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_EAX);
                        hregister:=cg.getaddressregister(current_asmdata.CurrAsmList);
                        cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,NR_RAX,hregister);
                        reference_reset(location.reference,location.reference.alignment,location.reference.volatility);
                        location.reference.base:=hregister;
                      end;
                    else
                      Internalerror(2019012002);
                  end;
                end;
              else
                ;
            end;
{$endif x86_64}
          end;
      end;


begin
   cloadnode:=tx86loadnode;
end.
