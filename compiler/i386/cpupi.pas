{
    Copyright (c) 2002 by Florian Klaempfl

    This unit contains the CPU specific part of tprocinfo

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

{ This unit contains the CPU specific part of tprocinfo. }
unit cpupi;

{$i fpcdefs.inc}

  interface

    uses
       psub,procinfo,psabiehpi,aasmdata;

    type
       tcpuprocinfo = class(tpsabiehprocinfo)
         constructor create(aparent:tprocinfo);override;
         procedure set_first_temp_offset;override;
         function calc_stackframe_size:longint;override;
         procedure generate_parameter_info;override;
         procedure allocate_got_register(list: tasmlist);override;
       end;


  implementation

    uses
      cutils,
      systems,globals,globtype,
      cgobj,tgobj,paramgr,
      cpubase,
      cgutils,
      symconst;

    constructor tcpuprocinfo.create(aparent:tprocinfo);
      begin
        inherited create(aparent);
        got:=NR_EBX;
      end;


    procedure tcpuprocinfo.set_first_temp_offset;
      begin
        if paramanager.use_fixed_stack then
          begin
            if not(po_assembler in procdef.procoptions) and
               (tg.direction > 0) then
              tg.setfirsttemp(tg.direction*maxpushedparasize);
            if (tg.direction < 0) and
               not(po_nostackframe in procdef.procoptions) then
              { compensate for the return address and the "pushl %ebp" }
              tg.setalignmentmismatch(sizeof(pint)*2);
          end;
      end;


    function tcpuprocinfo.calc_stackframe_size:longint;
      begin
        { align to 4 bytes at least
          otherwise all those subl $2,%esp are meaningless PM }
        if target_info.stackalign<=4 then
          result:=Align(tg.direction*tg.lasttemp,min(current_settings.alignment.localalignmax,4))
        else
          { aligned during stack frame allocation, because also depends number
            of saved registers }
          result:=tg.direction*tg.lasttemp+maxpushedparasize;
      end;


    procedure tcpuprocinfo.generate_parameter_info;
      begin
        inherited generate_parameter_info;
        { Para_stack_size is only used to determine how many bytes to remove
          from the stack at the end of the procedure (in the "ret $xx").
          If the stack is fixed, nothing has to be removed by the callee, except
          if a 16 byte aligned stack on i386-linux is used     }
        if paramanager.use_fixed_stack and not(target_info.abi=abi_linux386_sysv) then
          para_stack_size := 0;
      end;


    procedure tcpuprocinfo.allocate_got_register(list: tasmlist);
      begin
        if (pi_uses_threadvar in flags) and (tf_section_threadvars in target_info.flags) and (current_settings.tlsmodel in [tlsm_global_dynamic]) then
          begin
            { FIXME: It is better to use an imaginary register for GOT and
              if EBX is needed for some reason just allocate EBX and
              copy GOT into it before its usage. }
            cg.getcpuregister(list,NR_EBX);
            got := NR_EBX;
          end
        else if cs_create_pic in current_settings.moduleswitches then
          got := cg.getaddressregister(list);
      end;

begin
   cprocinfo:=tcpuprocinfo;
end.
