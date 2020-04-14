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
      var
        op: TAsmOp;
      begin
        case getregtype(tempreg) of
          R_FPUREGISTER:
            op:=A_LSI;
          R_INTREGISTER:
            op:=A_L32I;
          else
            Internalerror(2020041001);
        end;
        do_spill_op(list,op,pos,spilltemp,tempreg,orgsupreg);
      end;


    procedure trgcpu.do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        op: TAsmOp;
      begin
        case getregtype(tempreg) of
          R_FPUREGISTER:
            op:=A_SSI;
          R_INTREGISTER:
            op:=A_S32I;
          else
            Internalerror(2020041002);
        end;
        do_spill_op(list,op,pos,spilltemp,tempreg,orgsupreg);
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

        if abs(spilltemp.offset)>1020 then
          begin
            helplist:=TAsmList.create;

            if getregtype(tempreg)=R_INTREGISTER then
              begin
                if isload then
                  hreg:=tempreg
                else
                  hreg:=getregisterinline(helplist,[R_SUBWHOLE]);
              end
            else
              hreg:=cg.getintregister(helplist,OS_ADDR);

            if spilltemp.index<>NR_NO then
              Internalerror(2020032401);

            helplist.concat(taicpu.op_reg_reg_const(A_ADDMI,hreg,spilltemp.base,(spilltemp.offset div 256)*256));

            reference_reset(tmpref,sizeof(pint),[]);
            tmpref.base:=hreg;
            tmpref.offset:=spilltemp.offset mod 256;

            helpins:=taicpu.op_reg_ref(op,tempreg,tmpref);
            if getregtype(tempreg)=R_INTREGISTER then
              ungetregisterinline(helplist,hreg);
            helplist.concat(helpins);
            list.insertlistafter(pos,helplist);
            helplist.free;
          end
        else if isload then
          inherited do_spill_read(list,pos,spilltemp,tempreg,orgsupreg)
        else
          inherited do_spill_written(list,pos,spilltemp,tempreg,orgsupreg);
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
