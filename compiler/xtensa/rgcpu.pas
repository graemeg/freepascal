{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the Xtensa specific class for the register
    allocator

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

 ****************************************************************************}
unit rgcpu;

{$i fpcdefs.inc}

  interface

    uses
      aasmbase,aasmcpu,aasmtai,aasmdata,
      cgbase,cgutils,
      cpubase,
      globtype,
      rgobj;

    type
      trgcpu=class(trgobj)
        procedure do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
        procedure do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
       protected
        procedure do_spill_op(list: tasmlist; op: tasmop; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      end;

      trgintcpu=class(trgcpu)
        procedure add_cpu_interferences(p: tai); override;
      end;


implementation

    uses
      verbose,cutils,
      cgobj;

    procedure trgcpu.do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      begin
        do_spill_op(list,A_L32I,pos,spilltemp,tempreg,orgsupreg);
      end;


    procedure trgcpu.do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      begin
        do_spill_op(list,A_S32I,pos,spilltemp,tempreg,orgsupreg);
      end;


    procedure trgcpu.do_spill_op(list: tasmlist; op: tasmop; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        helpins  : tai;
        tmpref   : treference;
        helplist : TAsmList;
        hreg     : tregister;
        isload   : boolean;
      begin
        isload:=op in [A_L32I,A_LSI];
        //{ offset out of range for regular load/store? }
        //if simple_ref_type(op,reg_cgsize(tempreg),PF_None,spilltemp)<>sr_simple then
        //  begin
        //    helplist:=TAsmList.create;
        //
        //    if getregtype(tempreg)=R_INTREGISTER then
        //      hreg:=tempreg
        //    else
        //      hreg:=cg.getaddressregister(helplist);
        //
        //    cg.a_load_const_reg(helplist,OS_ADDR,spilltemp.offset,hreg);
        //    reference_reset_base(tmpref,spilltemp.base,0,spilltemp.temppos,sizeof(pint),[]);
        //    tmpref.index:=hreg;
        //    if isload then
        //      helpins:=spilling_create_load(tmpref,tempreg)
        //    else
        //      helpins:=spilling_create_store(tempreg,tmpref);
        //    helplist.concat(helpins);
        //    add_cpu_interferences(helpins);
        //    list.insertlistafter(pos,helplist);
        //    helplist.free;
        //  end
        //else
        if isload then
          inherited do_spill_read(list,pos,spilltemp,tempreg,orgsupreg)
        else
          inherited do_spill_written(list,pos,spilltemp,tempreg,orgsupreg)
      end;


    procedure trgintcpu.add_cpu_interferences(p: tai);
     var
       i, j: longint;
     begin
       if p.typ=ait_instruction then
         begin
         end;
     end;

end.
