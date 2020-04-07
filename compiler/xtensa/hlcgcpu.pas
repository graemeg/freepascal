{
    Copyright (c) 1998-2010 by Florian Klaempfl and Jonas Maebe
    Member of the Free Pascal development team

    This unit contains routines to create a pass-through high-level code
    generator. This is used by most regular code generators.

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

unit hlcgcpu;

{$i fpcdefs.inc}

interface

  uses
    symtype,
    aasmdata,
    symdef,
    cgbase,cgutils,
    hlcgobj, hlcg2ll;

  type
    thlcgxtensa = class(thlcg2ll)
      procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);override;
      procedure record_generated_code_for_procdef(pd: tprocdef; code, data: TAsmList);override;
    end;

implementation

  uses
    verbose,globtype,fmodule,
    aasmbase,aasmtai,
    symconst,symsym,defutil,
    cpubase,aasmcpu,parabase,
    procinfo,
    cgobj,cgcpu;

  procedure create_hlcodegen_cpu;
    begin
      hlcg:=thlcgxtensa.create;
      create_codegen;
    end;


  procedure thlcgxtensa.g_intf_wrapper(list : TAsmList; procdef : tprocdef;
   const labelname : string; ioffset : longint);
    var
      make_global : boolean;
      tmpref : treference;
      l : TAsmLabel;
    begin
      if not(procdef.proctypeoption in [potype_function,potype_procedure]) then
        Internalerror(200006137);
      if not assigned(procdef.struct) or
         (procdef.procoptions*[po_classmethod, po_staticmethod,
           po_methodpointer, po_interrupt, po_iocheck]<>[]) then
        Internalerror(200006138);
      if procdef.owner.symtabletype<>ObjectSymtable then
        Internalerror(200109191);

      make_global:=false;
      if (not current_module.is_unit) or
         create_smartlink or
         (procdef.owner.defowner.owner.symtabletype=globalsymtable) then
        make_global:=true;

      if make_global then
        list.concat(Tai_symbol.Createname_global(labelname,AT_FUNCTION,0,procdef))
      else
        list.concat(Tai_symbol.Createname_hidden(labelname,AT_FUNCTION,0,procdef));

      { the wrapper might need aktlocaldata for the additional data to
        load the constant }
      current_procinfo:=cprocinfo.create(nil);

      { set param1 interface to self  }
      g_adjust_self_value(list,procdef,ioffset);

      { case 4 }
      if (po_virtualmethod in procdef.procoptions) and
          not is_objectpascal_helper(procdef.struct) then
        begin
//          loadvmttor12;
//          op_onr12methodaddr;
        end
      else
        list.concat(taicpu.op_sym(A_CALL0,current_asmdata.RefAsmSymbol(procdef.mangledname,AT_FUNCTION)));
      list.concatlist(current_procinfo.aktlocaldata);

      current_procinfo.Free;
      current_procinfo:=nil;

      list.concat(Tai_symbol_end.Createname(labelname));
    end;


  procedure thlcgxtensa.record_generated_code_for_procdef(pd : tprocdef; code,
    data : TAsmList);
    var
      alt : TAsmListType;
    begin
      if not(po_assembler in pd.procoptions) then
        alt:=al_procedures
      else
        alt:=al_pure_assembler;
      { Xtensa needs the data before the subroutine }
      if assigned(data) and
         (not data.empty) then
        begin
          data.Insert(tai_align.Create(4));
          current_asmdata.asmlists[alt].concatlist(data);
        end;
      inherited record_generated_code_for_procdef(pd,code,nil);
    end;

begin
  chlcgobj:=thlcgxtensa;
  create_hlcodegen:=@create_hlcodegen_cpu;
end.
