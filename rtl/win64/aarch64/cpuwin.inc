{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2006 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit aarch64 specific part for the Win64 API.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$define FPC_SYSTEM_HAS_STACKTOP}
function StackTop: pointer; // assembler;nostackframe;
begin
  result:=pointer($ffffffffffffffff);
end;

function main_wrapper(arg: Pointer; proc: Pointer): ptrint; assembler; nostackframe;
asm
    stp fp,lr,[sp, #-16]!
.seh_savefplr_x -16
.seh_endprologue
    blr x1                  // { "arg" is passed in x0 }
    nop                     // { this nop is critical for exception handling }
    ldp	fp,lr,[sp], #16
.seh_handler __FPC_default_handler,@except,@unwind
end;


procedure Exe_entry(constref info: TEntryInformation);[public,alias:'_FPC_EXE_Entry'];
  begin
     SetupEntryInformation(info);
     IsLibrary:=false;
     { install the handlers for exe only ?
       or should we install them for DLL also ? (PM) }
     ExitCode:=0;
     asm
        mov x0,#0
        adrp x1,EntryInformation@PAGE
        add x1,x1,EntryInformation@PAGEOFF
        ldr x1,[x1,TEntryInformation.PascalMain]
        adrp x8,main_wrapper@PAGE
        add x8,x8,main_wrapper@PAGEOFF
        blr x8
     end ['X8'];
     { if we pass here there was no error ! }
     system_exit;
  end;
