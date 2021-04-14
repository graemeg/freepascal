{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the AArch64 specific class for the register
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
        function get_spill_subreg(r: tregister): tsubregister; override;
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

    function  trgcpu.get_spill_subreg(r:tregister) : tsubregister;
      begin
        if (getregtype(r)<>R_MMREGISTER) then
          result:=defaultsub
        else
          result:=getsubreg(r);
      end;


    procedure trgcpu.do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      begin
        do_spill_op(list,A_LDR,pos,spilltemp,tempreg,orgsupreg);
      end;


    procedure trgcpu.do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      begin
        do_spill_op(list,A_STR,pos,spilltemp,tempreg,orgsupreg);
      end;


    procedure trgcpu.do_spill_op(list: tasmlist; op: tasmop; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        helpins  : tai;
        tmpref   : treference;
        helplist : TAsmList;
        hreg     : tregister;
        isload   : boolean;
      begin
        isload:=op=A_LDR;
        { offset out of range for regular load/store? }
        if simple_ref_type(op,reg_cgsize(tempreg),PF_None,spilltemp)<>sr_simple then
          begin
            helplist:=TAsmList.create;

            if (getregtype(tempreg)=R_INTREGISTER) then
              hreg:=getregisterinline(helplist,[R_SUBWHOLE])
            else
              hreg:=cg.getaddressregister(helplist);

            cg.a_load_const_reg(helplist,OS_ADDR,spilltemp.offset,hreg);
            reference_reset_base(tmpref,spilltemp.base,0,spilltemp.temppos,sizeof(pint),[]);
            tmpref.index:=hreg;
            if isload then
              helpins:=spilling_create_load(tmpref,tempreg)
            else
              helpins:=spilling_create_store(tempreg,tmpref);
            helplist.concat(helpins);
            if (getregtype(tempreg)=R_INTREGISTER) then
              ungetregisterinline(helplist,hreg);
            add_cpu_interferences(helpins);
            list.insertlistafter(pos,helplist);
            helplist.free;
          end
        else if isload then
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
           { add interferences for instructions that can have SP as a register
             operand }
           case taicpu(p).opcode of
             A_MOV:
               { all operands can be SP }
               exit;
             A_ADD,
             A_SUB,
             A_CMP,
             A_CMN:
               { ok as destination or first source in immediate or extended
                 register form }
               if (taicpu(p).oper[taicpu(p).ops-1]^.typ<>top_shifterop) or
                  valid_shifter_operand(taicpu(p).opcode,false,true,
                    reg_cgsize(taicpu(p).oper[0]^.reg) in [OS_64,OS_S64],
                    taicpu(p).oper[taicpu(p).ops-1]^.shifterop^.shiftmode,
                    taicpu(p).oper[taicpu(p).ops-1]^.shifterop^.shiftimm) then
                 begin
                   if taicpu(p).oper[taicpu(p).ops-1]^.typ=top_shifterop then
                     i:=taicpu(p).ops-2
                   else
                     i:=taicpu(p).ops-1;
                   if (taicpu(p).oper[i]^.typ=top_reg) then
                     add_edge(getsupreg(taicpu(p).oper[i]^.reg),RS_SP);
                   exit;
                 end;
             A_AND,
             A_EOR,
             A_ORR,
             A_TST:
               { ok in immediate form }
               if taicpu(p).oper[taicpu(p).ops-1]^.typ=top_const then
                 exit;
             else
               ;
           end;
           { add interferences for other registers }
           for i:=0 to taicpu(p).ops-1 do
             begin
               case taicpu(p).oper[i]^.typ of
                 top_reg:
                   if getregtype(taicpu(p).oper[i]^.reg)=R_INTREGISTER then
                     add_edge(getsupreg(taicpu(p).oper[i]^.reg),RS_SP);
                 top_ref:
                   begin
                     { sp can always be base, never be index }
                     if taicpu(p).oper[i]^.ref^.index<>NR_NO then
                       add_edge(getsupreg(taicpu(p).oper[i]^.ref^.index),RS_SP);
                     { in case of write back, the base register must be
                       different from the loaded/stored register }
                     if (taicpu(p).oper[i]^.ref^.addressmode in [AM_PREINDEXED,AM_POSTINDEXED]) and
                        (taicpu(p).oper[i]^.ref^.base<>NR_NO) then
                       begin
                         for j:=pred(i) downto 0 do
                           if taicpu(p).oper[j]^.typ=TOP_REG then
                             add_edge(getsupreg(taicpu(p).oper[j]^.reg),getsupreg(taicpu(p).oper[i]^.ref^.base));
                       end;
                   end;
                 else
                   ;
               end;
             end;
         end;
     end;

end.
