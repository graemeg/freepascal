{
    Copyright (c) 2019 by Dmitry Boyarintsev

    WebAssembly-specific code for call nodes

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
unit nwasmcal;

{$i fpcdefs.inc}

interface

    uses
      cgbase,
      symtype,symdef,cgutils,parabase,
      node,ncal,ncgcal,hlcgobj,aasmcpu,cpubase, wasmdef;

    type
       { twasmcallparanode }

       twasmcallparanode = class(tcgcallparanode)
       end;

       { twasmcallnode }

       twasmcallnode = class(tcgcallnode)
       protected
         procedure extra_post_call_code; override;
         procedure do_release_unused_return_value; override;
         procedure set_result_location(realresdef: tstoreddef); override;
       end;


implementation

    uses
      globtype, aasmdata, defutil, tgobj, hlcgcpu, symconst;

      { twasmcallnode }

    procedure twasmcallnode.extra_post_call_code;
      begin
        thlcgwasm(hlcg).g_adjust_stack_after_call(current_asmdata.CurrAsmList,procdefinition);
      end;

    procedure twasmcallnode.do_release_unused_return_value;
      begin
        if is_void(resultdef) then
          exit;
        current_asmdata.CurrAsmList.concat(taicpu.op_none(a_drop));
        thlcgwasm(hlcg).decstack(current_asmdata.CurrAsmList,1);
      end;

    procedure twasmcallnode.set_result_location(realresdef: tstoreddef);
      begin
        // default implementation is placing the return value on LOC_REGISTER.
        // WebAssembly always returns the value on stack.
        location_reset_ref(location,LOC_REFERENCE,def_cgsize(realresdef),1,[]);
        tg.gethltemp(current_asmdata.CurrAsmList,realresdef,retloc.intsize,tt_normal,location.reference);
      end;

begin
  ccallnode:=twasmcallnode;
  ccallparanode:=twasmcallparanode;
end.
