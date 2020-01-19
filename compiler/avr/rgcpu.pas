{
    Copyright (c) 1998-2008 by Florian Klaempfl

    This unit implements the avr specific class for the register
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
       aasmbase,aasmtai,aasmdata,aasmcpu,aasmsym,
       cgbase,cgutils,
       cpubase,
       rgobj;

     type
       trgcpu = class(trgobj)
         procedure add_constraints(reg:tregister);override;
         procedure do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
         procedure do_spill_written(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister); override;
         function do_spill_replace(list : TAsmList;instr : tai_cpu_abstract_sym; orgreg : tsuperregister;const spilltemp : treference) : boolean; override;
       end;

       trgintcpu = class(trgcpu)
         procedure add_cpu_interferences(p : tai);override;
       end;

  implementation

    uses
      verbose, cutils,
      globals,
      cgobj,
      procinfo,
      cpuinfo;


    procedure trgcpu.add_constraints(reg:tregister);
      var
        supreg,i : Tsuperregister;
      begin
        case getsubreg(reg) of
          { Let 64bit floats conflict with all odd float regs }
          R_SUBFD:
            begin
            {
              supreg:=getsupreg(reg);
              i:=RS_F1;
              while (i<=RS_F31) do
                begin
                  add_edge(supreg,i);
                  inc(i,2);
                end;
            }
            end;
          { Let 64bit ints conflict with all odd int regs }
          R_SUBQ:
            begin
              supreg:=getsupreg(reg);
              {
              i:=RS_G1;
              while (i<=RS_I7) do
                begin
                  add_edge(supreg,i);
                  inc(i,2);
                end;
              }
            end;
        end;
      end;


    procedure trgcpu.do_spill_read(list: TAsmList; pos: tai; const spilltemp: treference; tempreg: tregister; orgsupreg: tsuperregister);
      var
        helpins  : tai;
        tmpref   : treference;
        helplist : TAsmList;
        hreg     : tregister;
      begin
        if (abs(spilltemp.offset)>63) or (CPUAVR_16_REGS in cpu_capabilities[current_settings.cputype]) then
          begin
            helplist:=TAsmList.create;

            helplist.concat(taicpu.op_reg_const(A_LDI,NR_R26,lo(word(spilltemp.offset))));
            helplist.concat(taicpu.op_reg_const(A_LDI,NR_R27,hi(word(spilltemp.offset))));
            helplist.concat(taicpu.op_reg_reg(A_ADD,NR_R26,spilltemp.base));
            helplist.concat(taicpu.op_reg_reg(A_ADC,NR_R27,cg.GetNextReg(spilltemp.base)));

            reference_reset_base(tmpref,NR_R26,0,spilltemp.temppos,1,[]);
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
        helplist : TAsmList;
        hreg     : tregister;
      begin
        if (abs(spilltemp.offset)>63) or (CPUAVR_16_REGS in cpu_capabilities[current_settings.cputype]) then
          begin
            helplist:=TAsmList.create;

            helplist.concat(taicpu.op_reg_const(A_LDI,NR_R26,lo(word(spilltemp.offset))));
            helplist.concat(taicpu.op_reg_const(A_LDI,NR_R27,hi(word(spilltemp.offset))));
            helplist.concat(taicpu.op_reg_reg(A_ADD,NR_R26,spilltemp.base));
            helplist.concat(taicpu.op_reg_reg(A_ADC,NR_R27,cg.GetNextReg(spilltemp.base)));

            reference_reset_base(tmpref,NR_R26,0,spilltemp.temppos,1,[]);
            helplist.concat(spilling_create_store(tempreg,tmpref));
            list.insertlistafter(pos,helplist);
            helplist.free;
          end
        else
          inherited;
      end;


    procedure trgintcpu.add_cpu_interferences(p : tai);
      var
        r : tsuperregister;
      begin
        if p.typ=ait_instruction then
          begin
            case taicpu(p).opcode of
              A_CPI,
              A_ANDI,
              A_ORI,
              A_SUBI,
              A_SBCI,
              A_LDI:
                for r:=RS_R0 to RS_R15 do
                  add_edge(r,GetSupReg(taicpu(p).oper[0]^.reg));
              A_STS:
                for r:=RS_R0 to RS_R15 do
                  add_edge(r,GetSupReg(taicpu(p).oper[1]^.reg));
              A_ADIW:
                for r:=RS_R0 to RS_R31 do
                  if not (r in [RS_R24,RS_R26,RS_R28,RS_R30]) then
                    add_edge(r,GetSupReg(taicpu(p).oper[0]^.reg));
              A_MULS:
                begin
                  for r:=RS_R0 to RS_R15 do
                    add_edge(r,GetSupReg(taicpu(p).oper[0]^.reg));
                  for r:=RS_R0 to RS_R15 do
                    add_edge(r,GetSupReg(taicpu(p).oper[1]^.reg));
                end;
              A_LDD:
                for r:=RS_R0 to RS_R31 do
                  if not (r in [RS_R28,RS_R30]) then
                    add_edge(r,GetSupReg(taicpu(p).oper[1]^.ref^.base));
              A_STD:
                for r:=RS_R0 to RS_R31 do
                  if not (r in [RS_R28,RS_R30]) then
                    add_edge(r,GetSupReg(taicpu(p).oper[0]^.ref^.base));
            end;
          end;
      end;


    function trgcpu.do_spill_replace(list:TAsmList;instr:tai_cpu_abstract_sym;orgreg:tsuperregister;const spilltemp:treference):boolean;
      var
        b : byte;
      begin
        result:=false;
        if not(spilltemp.offset in [0..63]) or (CPUAVR_16_REGS in cpu_capabilities[current_settings.cputype]) then
          exit;

        { Replace 'mov  dst,orgreg' with 'ldd  dst,spilltemp'
          and     'mov  orgreg,src' with 'std  spilltemp,src' }
        with instr do
          begin
            if (opcode=A_MOV) and (ops=2) and (oper[1]^.typ=top_reg) and (oper[0]^.typ=top_reg) then
              begin
                if (getregtype(oper[0]^.reg)=regtype) and
                   (get_alias(getsupreg(oper[0]^.reg))=orgreg) and
                   (get_alias(getsupreg(oper[1]^.reg))<>orgreg) then
                  begin
                    instr.loadref(0,spilltemp);
                    opcode:=A_STD;
                    result:=true;
                  end
                else if (getregtype(oper[1]^.reg)=regtype) and
                   (get_alias(getsupreg(oper[1]^.reg))=orgreg) and
                   (get_alias(getsupreg(oper[0]^.reg))<>orgreg) then
                  begin
                    instr.loadref(1,spilltemp);
                    opcode:=A_LDD;
                    result:=true;
                  end;
              end;
          end;
      end;

end.
