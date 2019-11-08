{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the register allocator for m68k

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
       aasmbase,aasmtai,aasmdata,aasmsym,aasmcpu,
       cgbase,cgutils,cpubase,
       rgobj;

     type
       trgcpu = class(trgobj)
         procedure do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
         procedure do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
         function do_spill_replace(list : TAsmList;instr : tai_cpu_abstract_sym; orgreg : tsuperregister;const spilltemp : treference) : boolean; override;
       end;

  implementation

    uses
      cutils,cgobj,verbose,globtype,globals,cpuinfo;

    { returns True if source operand of MOVE can be replaced with spilltemp when its destination is ref^. }
    function isvalidmovedest(ref: preference): boolean; inline;
      begin
        { The following is for Coldfire, for other CPUs it maybe can be relaxed. }
        result:=(ref^.symbol=nil) and (ref^.scalefactor<=1) and
          (ref^.index=NR_NO) and (ref^.base<>NR_NO) and (ref^.offset>=low(smallint)) and
          (ref^.offset<=high(smallint));
      end;


    procedure trgcpu.do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        helpins  : tai;
        tmpref   : treference;
        helplist : tasmlist;
        hreg     : tregister;
      begin
        if (abs(spilltemp.offset)>32767) and (current_settings.cputype in (cpu_coldfire + [cpu_mc68000])) then
          begin
            helplist:=tasmlist.create;

            if getregtype(tempreg)=R_INTREGISTER then
              hreg:=tempreg
            else
              hreg:=cg.getintregister(helplist,OS_ADDR);
{$ifdef DEBUG_SPILLING}
            helplist.concat(tai_comment.Create(strpnew('Spilling: Read, large offset')));
{$endif}

            helplist.concat(taicpu.op_const_reg(A_MOVE,S_L,spilltemp.offset,hreg));
            reference_reset_base(tmpref,spilltemp.base,0,spilltemp.temppos,sizeof(aint),[]);
            tmpref.index:=hreg;

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
        if (abs(spilltemp.offset)>32767) and (current_settings.cputype in (cpu_coldfire + [cpu_mc68000])) then
          begin
            helplist:=tasmlist.create;

            if getregtype(tempreg)=R_INTREGISTER then
              hreg:=getregisterinline(helplist,[R_SUBWHOLE])
            else
              hreg:=cg.getintregister(helplist,OS_ADDR);
{$ifdef DEBUG_SPILLING}
            helplist.concat(tai_comment.Create(strpnew('Spilling: Write, large offset')));
{$endif}

            helplist.concat(taicpu.op_const_reg(A_MOVE,S_L,spilltemp.offset,hreg));
            reference_reset_base(tmpref,spilltemp.base,0,spilltemp.temppos,sizeof(aint),[]);
            tmpref.index:=hreg;

            helplist.concat(spilling_create_store(tempreg,tmpref));
            if getregtype(tempreg)=R_INTREGISTER then
              ungetregisterinline(helplist,hreg);

            list.insertlistafter(pos,helplist);
            helplist.free;
          end
        else
          inherited;
    end;


    function trgcpu.do_spill_replace(list : TAsmList;instr : tai_cpu_abstract_sym; orgreg : tsuperregister;const spilltemp : treference) : boolean;
      var
        opidx: longint;
      begin
        result:=false;
        opidx:=-1;
        if (abs(spilltemp.offset)>32767) and (current_settings.cputype in cpu_coldfire) then
          exit;
        case instr.ops of
          1:
            begin
              if (instr.oper[0]^.typ=top_reg) and (getregtype(instr.oper[0]^.reg)=regtype) and
                ((instr.opcode=A_TST) or (instr.opcode=A_CLR)) then
                begin
                  if get_alias(getsupreg(instr.oper[0]^.reg))<>orgreg then
                    InternalError(2014080101);
                  opidx:=0;
                end;
            end;
          2:
            begin
              if (instr.oper[0]^.typ=top_reg) and (getregtype(instr.oper[0]^.reg)=regtype) and
                (get_alias(getsupreg(instr.oper[0]^.reg))=orgreg) then
                begin
                  { source can be replaced if dest is register... }
                  if ((instr.oper[1]^.typ=top_reg) and (instr.opcode in [A_MOVE,A_ADD,A_SUB,A_AND,A_OR,A_CMP])) or
                    {... or a "simple" reference in case of MOVE }
                    ((instr.opcode=A_MOVE) and (instr.oper[1]^.typ=top_ref) and isvalidmovedest(instr.oper[1]^.ref)) then
                    opidx:=0;
                end
              else if (instr.oper[1]^.typ=top_reg) and (getregtype(instr.oper[1]^.reg)=regtype) and
                (get_alias(getsupreg(instr.oper[1]^.reg))=orgreg) and
                ((
                  (instr.opcode in [A_MOVE,A_ADD,A_SUB,A_AND,A_OR]) and
                  (instr.oper[0]^.typ=top_reg) and not
                  (isaddressregister(instr.oper[0]^.reg))
                ) or
                (instr.opcode in [A_ADDQ,A_SUBQ,A_MOV3Q])) then
                opidx:=1;
            end;
          else
            ;
        end;

        if (opidx<0) then
          exit;
        instr.oper[opidx]^.typ:=top_ref;
        new(instr.oper[opidx]^.ref);
        instr.oper[opidx]^.ref^:=spilltemp;
        case taicpu(instr).opsize of
          S_B: inc(instr.oper[opidx]^.ref^.offset,3);
          S_W: inc(instr.oper[opidx]^.ref^.offset,2);
          else
            ;
        end;
        result:=true;
      end;

end.
