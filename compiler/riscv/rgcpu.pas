{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the Risc-V specific class for the register
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

 ****************************************************************************
}

unit rgcpu;

{$i fpcdefs.inc}

  interface

     uses
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cgbase,cgutils,
       cpubase,
       rgobj;

     type
       trgcpu = class(trgobj)
         procedure do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
         procedure do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
       end;

       trgintcpu = class(trgcpu)
       end;

  implementation

    uses
      verbose, cutils,globtype,
      cgobj,
      procinfo;


    procedure trgcpu.do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        tmpref : treference;
        helplist : TAsmList;
        hreg : tregister;
        helpins: Taicpu;
      begin
        if not is_imm12(spilltemp.offset) then
          begin
            helplist:=tasmlist.create;

            if getregtype(tempreg)=R_INTREGISTER then
              hreg:=tempreg
            else
              hreg:=cg.getintregister(helplist,OS_ADDR);

            if (spilltemp.offset and $800)<>0 then
              helplist.concat(taicpu.op_reg_const(A_LUI,hreg,((spilltemp.offset shr 12)+1) and $FFFFF))
            else
              helplist.concat(taicpu.op_reg_const(A_LUI,hreg,(spilltemp.offset shr 12) and $FFFFF));
            helplist.concat(taicpu.op_reg_reg_const(A_ADDI,hreg,hreg,SarSmallint(smallint(spilltemp.offset shl 4),4)));

            helplist.concat(taicpu.op_reg_reg_reg(A_ADD,hreg,hreg,spilltemp.base));

            reference_reset_base(tmpref,hreg,0,ctempposinvalid,sizeof(aint),[]);

            helpins:=spilling_create_load(tmpref,tempreg);
            helplist.concat(helpins);
            list.insertlistafter(pos,helplist);
            helplist.free;
          end
        else
          inherited;
      end;


    procedure trgcpu.do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        tmpref   : treference;
        helplist : tasmlist;
        hreg     : tregister;
      begin
        if not is_imm12(spilltemp.offset) then
          begin
            helplist:=tasmlist.create;

            if getregtype(tempreg)=R_INTREGISTER then
              hreg:=getregisterinline(helplist,[R_SUBWHOLE])
            else
              hreg:=cg.getintregister(helplist,OS_ADDR);

            if (spilltemp.offset and $800)<>0 then
              helplist.concat(taicpu.op_reg_const(A_LUI,hreg,((spilltemp.offset shr 12)+1) and $FFFFF))
            else
              helplist.concat(taicpu.op_reg_const(A_LUI,hreg,(spilltemp.offset shr 12) and $FFFFF));
            helplist.concat(taicpu.op_reg_reg_const(A_ADDI,hreg,hreg,SarSmallint(smallint(spilltemp.offset shl 4),4)));

            helplist.concat(taicpu.op_reg_reg_reg(A_ADD,hreg,hreg,spilltemp.base));

            reference_reset_base(tmpref,hreg,0,ctempposinvalid,sizeof(aint),[]);

            helplist.concat(spilling_create_store(tempreg,tmpref));
            if getregtype(tempreg)=R_INTREGISTER then
              ungetregisterinline(helplist,hreg);

            list.insertlistafter(pos,helplist);
            helplist.free;
          end
        else
          inherited;
      end;


end.
