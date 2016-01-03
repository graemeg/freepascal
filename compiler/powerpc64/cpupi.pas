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

{$I fpcdefs.inc}

interface

uses
  cutils,aasmdata,
  procinfo, cpuinfo, psub;

type
  tppcprocinfo = class(tcgprocinfo)
    needstackframe: boolean;

    { offset where the frame pointer from the outer procedure is stored. }
    parent_framepointer_offset: longint;

    needs_frame_pointer : boolean;

    constructor create(aparent: tprocinfo); override;
    procedure set_first_temp_offset; override;
    function calc_stackframe_size: longint; override;
    function calc_stackframe_size(numgpr, numfpr : longint): longint;

<<<<<<< HEAD
    procedure allocate_got_register(list: TAsmList); override;
    procedure postprocess_code;override;
=======
    needs_frame_pointer : boolean;

    procedure allocate_got_register(list: TAsmList); override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

implementation

uses
  globtype, globals, systems,
  cpubase, cgbase,
  aasmtai,
  tgobj,cgobj,
  symconst, symsym, paramgr, symutil, symtable,
  verbose,
  aasmcpu;

constructor tppcprocinfo.create(aparent: tprocinfo);

begin
  inherited create(aparent);
  maxpushedparasize := 0;
  needs_frame_pointer := false;
end;

procedure tppcprocinfo.set_first_temp_offset;
var
  ofs,
  lasize,
  minstacksize: aword;
begin
  if target_info.abi<>abi_powerpc_elfv2 then
    lasize:=LinkageAreaSizeELF
  else
    lasize:=LinkageAreaSizeELFv2;
  if not (po_assembler in procdef.procoptions) then begin
    { align the stack properly }
    if target_info.abi<>abi_powerpc_elfv2 then
      begin
        { same for AIX/Darwin }
        minstacksize:=MINIMUM_STACKFRAME_SIZE;
      end
    else
      begin
        minstacksize:=MINIMUM_STACKFRAME_SIZE_ELFV2;
      end;
    ofs := align(maxpushedparasize + lasize, ELF_STACK_ALIGN);

    { the ABI specification says that it is required to always allocate space for 8 * 8 bytes
      for registers R3-R10 and stack header if there's a stack frame, but GCC doesn't do that,
      so we don't that too. Uncomment the next three lines if this is required }
<<<<<<< HEAD
<<<<<<< HEAD
    if (cs_profile in init_settings.moduleswitches) and (ofs < minstacksize) then begin
      ofs := minstacksize;
=======
    if (cs_profile in init_settings.moduleswitches) and (ofs < MINIMUM_STACKFRAME_SIZE) then begin
      ofs := MINIMUM_STACKFRAME_SIZE;
>>>>>>> graemeg/fixes_2_2
=======
    if (cs_profile in init_settings.moduleswitches) and (ofs < MINIMUM_STACKFRAME_SIZE) then begin
      ofs := MINIMUM_STACKFRAME_SIZE;
>>>>>>> origin/fixes_2_2
    end;
    tg.setfirsttemp(ofs);
  end else begin
    if (current_procinfo.procdef.localst.symtabletype=localsymtable) and
       (tabstractlocalsymtable(current_procinfo.procdef.localst).count_locals <> 0) then
      { at 0(r1), the previous value of r1 will be stored; also make sure
        there's room to store lr etc by potential callees}
      tg.setfirsttemp(lasize);
  end;
end;

function tppcprocinfo.calc_stackframe_size: longint;
begin
  result := calc_stackframe_size(18, 18);
end;

function tppcprocinfo.calc_stackframe_size(numgpr, numfpr : longint) : longint;
begin
  { more or less copied from cgcpu.pas/g_stackframe_entry }
  if not (po_assembler in procdef.procoptions) then begin
    // no VMX support
    result := align(numgpr * sizeof(pint) +
        numfpr * tcgsize2size[OS_FLOAT], ELF_STACK_ALIGN);

    if (pi_do_call in flags) or (tg.lasttemp <> tg.firsttemp) or
      (result > RED_ZONE_SIZE) {or (cs_profile in init_settings.moduleswitches)} then begin
      result := align(result + tg.lasttemp, ELF_STACK_ALIGN);
      needstackframe:=true;
    end else
      needstackframe:=false;
  end else begin
    result := align(tg.lasttemp, ELF_STACK_ALIGN);
    needstackframe:=result<>0;
  end;
end;


procedure tppcprocinfo.allocate_got_register(list: TAsmList);
  begin
    if (target_info.system = system_powerpc64_darwin) and
       (cs_create_pic in current_settings.moduleswitches) then
      begin
        got := cg.getaddressregister(list);
      end;
  end;

<<<<<<< HEAD
<<<<<<< HEAD

procedure tppcprocinfo.postprocess_code;
  begin
    fixup_jmps(aktproccode);
  end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  cprocinfo := tppcprocinfo;
end.

