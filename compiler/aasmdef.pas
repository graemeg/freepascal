{
    Copyright (c) 2016 by Jonas Maebe, member of the Free Pascal
    development team

    Contains asmsymbol functionality that depends on symdef (to avoid creating
    circular dependencies between symdef and aasmdata via aasmtai)

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
unit aasmdef;

{$i fpcdefs.inc}

interface

uses
  globtype,
  aasmbase,aasmdata,
  symtype;

type
  TAsmDataDef = class(TAsmData)
    function  DefineAsmSymbolByClass(symclass: TAsmSymbolClass; const s : TSymStr;_bind:TAsmSymBind;_typ:Tasmsymtype; def: tdef) : TAsmSymbol; override;
  end;

implementation

uses
  globals,cutils,systems,
  aasmtai,aasmcnst,
  symdef,
  fmodule;


function TAsmDataDef.DefineAsmSymbolByClass(symclass: TAsmSymbolClass; const s: TSymStr; _bind: TAsmSymBind; _typ: Tasmsymtype; def: tdef): TAsmSymbol;
  var
    symind: tasmsymbol;
    ptrdef: tdef;
    tcb: ttai_typedconstbuilder;
    wasdefined: boolean;
  begin
    result:=DefineAsmSymbolByClassBase(symclass,s,_bind,_typ,def,wasdefined);
    { define the indirect asmsymbol if necessary }
    if not wasdefined and
       (_bind in [AB_GLOBAL,AB_COMMON,AB_PRIVATE_EXTERN]) and
       (_typ<>AT_DATA_NOINDIRECT) and
       (((_typ=AT_DATA) and
         (tf_supports_packages in target_info.flags) and
         (target_info.system in systems_indirect_var_imports)
        ) or
        (_typ=AT_DATA_FORCEINDIRECT)
       ) then
      begin
        ptrdef:=cpointerdef.getreusable(def);
        symind:=current_asmdata.DefineAsmSymbol(s,AB_INDIRECT,AT_DATA,ptrdef);
        tcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_new_section]);
        tcb.emit_tai(Tai_const.Create_sym_offset(result,0),ptrdef);
        current_asmdata.AsmLists[al_indirectglobals].concatlist(tcb.get_final_asmlist(
          symind,ptrdef,
          sec_rodata,
          lower(symind.name),
          ptrdef.alignment));
        tcb.free;
        if (_typ=AT_DATA_FORCEINDIRECT) and not (target_info.system in systems_indirect_var_imports) then
          current_module.add_public_asmsym(symind.name,AB_INDIRECT,AT_DATA);
      end;
  end;


begin
  { Do not overwrite if already set, by powerpc specific code for example }
  if not assigned(casmdata) then
    casmdata:=TAsmDataDef;
end.

