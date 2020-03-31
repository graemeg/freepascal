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
    cgobj,cgcpu;

  procedure create_hlcodegen_cpu;
    begin
      hlcg:=thlcgxtensa.create;
      create_codegen;
    end;


  procedure thlcgxtensa.g_intf_wrapper(list : TAsmList; procdef : tprocdef;
   const labelname : string; ioffset : longint);
    begin
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
