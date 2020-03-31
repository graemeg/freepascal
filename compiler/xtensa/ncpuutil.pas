{
    Copyright (c) 2015 by Jeppe Johansen

    Xtensa version of some node tree helper routines

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
unit ncpuutil;

{$i fpcdefs.inc}

interface

  uses
    cclasses,
    node,nbas,
    ngenutil,
    symtype,symconst,symsym,symdef;


  type
    txtensanodeutils = class(tnodeutils)
    protected
      class procedure insert_init_final_table(entries:tfplist); override;
    end;

implementation

    uses
      verbose,cutils,globtype,globals,constexp,fmodule,
      aasmdata,aasmtai,aasmcpu,aasmcnst,aasmbase,
      cpubase,
      symbase,symcpu,symtable,defutil,
      ncnv,ncon,ninl,ncal,nld,nmem,
      systems,
      CPUInfo,
      ppu,
      pass_1;


  class procedure txtensanodeutils.insert_init_final_table(entries:tfplist);
    var
      callop, retop: TAsmOp;
      initList, finalList, header: TAsmList;
      entry : pinitfinalentry;
      i : longint;
    begin
      initList:=TAsmList.create;
      finalList:=TAsmList.create;

      initList.Concat(tai_align.Create(target_info.alignment.procalign));
      finalList.Concat(tai_align.Create(target_info.alignment.procalign));

      case target_info.abi of
//        abi_xtensa_call0:
//          begin
//            initList.Concat(taicpu.op_none(A_RET));
//            finalList.Concat(taicpu.op_none(A_RET));
//            callop:=A_CALL0;
//            retop:=A_RET;
//          end;
        abi_xtensa_windowed:
          begin
            initList.Concat(taicpu.op_reg_const(A_ENTRY,NR_A1,16));
            finalList.Concat(taicpu.op_reg_const(A_ENTRY,NR_A1,16));
            callop:=A_CALL4;
            retop:=A_RETW;
          end;
        else
          Internalerror(2020031501);
      end;

      for i:=0 to entries.count-1 do
        begin
          entry:=pinitfinalentry(entries[i]);
          if entry^.finifunc<>'' then
            finalList.Concat(taicpu.op_sym(callop,current_asmdata.RefAsmSymbol(entry^.finifunc,AT_FUNCTION)));
          if entry^.initfunc<>'' then
            initList.Concat(taicpu.op_sym(callop,current_asmdata.RefAsmSymbol(entry^.initfunc,AT_FUNCTION)));
        end;

      initList.Concat(taicpu.op_none(retop));
      finalList.Concat(taicpu.op_none(retop));

      header:=TAsmList.create;
      new_section(header, sec_code, 'FPC_INIT_FUNC_TABLE', 1);
      header.concat(tai_symbol.Createname_global('FPC_INIT_FUNC_TABLE',AT_FUNCTION,0,voidcodepointertype));

      initList.insertList(header);
      header.free;

      current_asmdata.AsmLists[al_procedures].concatList(initList);

      header:=TAsmList.create;
      new_section(header, sec_code, 'FPC_FINALIZE_FUNC_TABLE', 1);
      header.concat(tai_symbol.Createname_global('FPC_FINALIZE_FUNC_TABLE',AT_FUNCTION,0,voidcodepointertype));

      finalList.insertList(header);
      header.free;

      current_asmdata.AsmLists[al_procedures].concatList(finalList);

      initList.Free;
      finalList.Free;

      inherited insert_init_final_table(entries);
    end;

begin
  cnodeutils:=txtensanodeutils;
end.

