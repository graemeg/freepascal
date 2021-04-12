{
    Copyright (c) 2021 by Karoly Balogh

    m68k version of some node tree helper routines

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
unit n68kutil;

{$i fpcdefs.inc}

interface

    uses
      cclasses,ngenutil;

    type
      t68knodeutils = class(tnodeutils)
        class procedure InsertObjectInfo; override;
      end;

implementation

    uses
      verbose,
      systems,
      globals,
      fmodule,
      aasmbase,aasmdata,aasmtai,aasmcpu,aasmcnst,
      symdef,symtype;


    class procedure t68knodeutils.InsertObjectInfo;
      var
        tcb: ttai_typedconstbuilder;
        s: shortstring;
        sym: tasmsymbol;
        def: tdef;
      begin
        inherited InsertObjectInfo;
        if (not current_module.is_unit) and (target_info.system in [system_m68k_sinclairql]) then 
          begin
            { insert the main program name into the object. this will be set as default job name by the system unit }
            tcb:=ctai_typedconstbuilder.create([tcalo_new_section]);
            s:=char(length(current_module.realmodulename^))+current_module.realmodulename^+#0;
            def:=carraydef.getreusable(cansichartype,length(s));
            tcb.maybe_begin_aggregate(def);
            tcb.emit_tai(Tai_string.Create(s),def);
            tcb.maybe_end_aggregate(def);
            sym:=current_asmdata.DefineAsmSymbol('__fpc_program_name',AB_GLOBAL,AT_DATA,def);
            current_asmdata.asmlists[al_globals].concatlist(
              tcb.get_final_asmlist(sym,def,sec_rodata,'__fpc_program_name',const_align(current_settings.alignment.constalignmax))
            );
            tcb.free;
          end;
      end;

begin
  cnodeutils:=t68knodeutils;
end.
