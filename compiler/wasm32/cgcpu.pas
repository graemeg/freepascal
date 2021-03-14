{
    Copyright (c) 2019 by Dmitry Boyarintsev

    This unit implements the code generator for the WebAssembly

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
unit cgcpu;

{$i fpcdefs.inc}

interface

    uses
       globtype,parabase,
       cgbase,cgutils,cgobj,cghlcpu,
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpuinfo,
       node,symconst,SymType,symdef,
       rgcpu;

    type
      TCgWasm=class(thlbasecgcpu)
     public
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;
        function  getintregister(list:TAsmList;size:Tcgsize):Tregister;override;
        function  getfpuregister(list:TAsmList;size:Tcgsize):Tregister;override;
        function  getaddressregister(list:TAsmList):Tregister;override;
        procedure do_register_allocation(list:TAsmList;headertai:tai);override;
      end;

    procedure create_codegen;

implementation

  uses
    globals,verbose,systems,cutils,
    paramgr,fmodule,
    tgobj,
    procinfo,cpupi;


{****************************************************************************
                              Assembler code
****************************************************************************}

    procedure TCgWasm.init_register_allocators;
      begin
        inherited init_register_allocators;
{$ifndef cpu64bitaddr}
        rg[R_INTREGISTER]:=Trgcpu.create(R_INTREGISTER,R_SUBD,
          [RS_R0],first_int_imreg,[]);
{$else not cpu64bitaddr}
        rg[R_INTREGISTER]:=Trgcpu.create(R_INTREGISTER,R_SUBQ,
          [RS_R0],first_int_imreg,[]);
{$endif not cpu64bitaddr}
        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBFS,
          [RS_R0],first_fpu_imreg,[]);
        rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBNONE,
          [RS_R0],first_mm_imreg,[]);
      end;


    procedure TCgWasm.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


    function TCgWasm.getintregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if not(size in [OS_64,OS_S64]) then
          result:=rg[R_INTREGISTER].getregister(list,R_SUBD)
        else
          result:=rg[R_INTREGISTER].getregister(list,R_SUBQ);
      end;


    function TCgWasm.getfpuregister(list:TAsmList;size:Tcgsize):Tregister;
      begin
        if size=OS_F64 then
          result:=rg[R_FPUREGISTER].getregister(list,R_SUBFD)
        else
          result:=rg[R_FPUREGISTER].getregister(list,R_SUBFS);
      end;


    function tcgwasm.getaddressregister(list:TAsmList):Tregister;
      begin
        { avoid problems in the compiler where int and addr registers are
          mixed for now; we currently don't have to differentiate between the
          two as far as the jvm backend is concerned }
        result:=rg[R_INTREGISTER].getregister(list,R_SUBD)
      end;


    procedure tcgwasm.do_register_allocation(list:TAsmList;headertai:tai);
      begin
        { We only run the "register allocation" once for an arbitrary allocator,
          which will perform the register->temp mapping for all register types.
          This allows us to easily reuse temps. }
        trgcpu(rg[R_INTREGISTER]).do_all_register_allocation(list,headertai);
      end;


    procedure create_codegen;
      begin
        cg:=tcgwasm.Create;
      end;
      
end.
