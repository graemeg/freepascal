{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2006 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit x86-64 specific part for the Win64 API.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$asmmode att}

function StackTop: pointer; assembler;nostackframe;
asm
   movq  %gs:(8),%rax
end;

{$ifdef FPC_USE_WIN64_SEH}
function main_wrapper(arg: Pointer; proc: Pointer): ptrint; assembler; nostackframe;
asm
    subq   $40, %rsp
.seh_stackalloc 40
.seh_endprologue
    call   %rdx             { "arg" is passed in %rcx }
    nop                     { this nop is critical for exception handling }
    addq   $40, %rsp
.seh_handler __FPC_default_handler,@except,@unwind
end;
{$endif FPC_USE_WIN64_SEH}

var
  { old compilers emitted a reference to _fltused if a module contains
    floating type code so the linker could leave away floating point
    libraries or not. VC does this as well so we need to define this
    symbol as well (FK)
  }
  _fltused : int64;cvar;public;
  { value of the stack segment
    to check if the call stack can be written on exceptions }
  _SS : Cardinal;

{$ifdef VER3_0}
procedure Exe_entry;[public,alias:'_FPC_EXE_Entry'];
{$else VER3_0}
procedure Exe_entry(constref info: TEntryInformation);[public,alias:'_FPC_EXE_Entry'];
{$endif VER3_0}
  begin
{$ifndef VER3_0}
     SetupEntryInformation(info);
{$endif VER3_0}
     IsLibrary:=false;
     { install the handlers for exe only ?
       or should we install them for DLL also ? (PM) }
{$ifndef SYSTEM_USE_WIN_SEH}
     install_exception_handlers;
{$endif SYSTEM_USE_WIN_SEH}
     ExitCode:=0;
     asm
        xorq %rax,%rax
        movw %ss,%ax
        movl %eax,_SS(%rip)
        movq %rbp,%rsi
        xorq %rbp,%rbp
{$ifdef VER3_0}
{$ifdef FPC_USE_WIN64_SEH}
        xor  %rcx,%rcx
        lea  PASCALMAIN(%rip),%rdx
        call main_wrapper
{$else FPC_USE_WIN64_SEH}
        call PASCALMAIN
{$endif FPC_USE_WIN64_SEH}
{$else VER3_0}
{$ifdef FPC_USE_WIN64_SEH}
        xor  %rcx,%rcx
        lea  EntryInformation(%rip),%rdx
        movq TEntryInformation.PascalMain(%rdx),%rdx
        call main_wrapper
{$else FPC_USE_WIN64_SEH}
        lea  EntryInformation(%rip),%rdx
        call TEntryInformation.PascalMain(%rdx)
{$endif FPC_USE_WIN64_SEH}
{$endif VER3_0}
        movq %rsi,%rbp
     end ['RSI','RBP'];     { <-- specifying RSI allows compiler to save/restore it properly }
     { if we pass here there was no error ! }
     system_exit;
  end;

function is_prefetch(p : pointer) : boolean;
  var
    a : array[0..15] of byte;
    doagain : boolean;
    instrlo,instrhi,opcode : byte;
    i : longint;
  begin
    result:=false;
    { read memory savely without causing another exeception }
    if not(ReadProcessMemory(GetCurrentProcess,p,@a,sizeof(a),nil)) then
      exit;
    i:=0;
    doagain:=true;
    while doagain and (i<15) do
      begin
        opcode:=a[i];
        instrlo:=opcode and $f;
        instrhi:=opcode and $f0;
        case instrhi of
          { prefix? }
          $20,$30:
            doagain:=(instrlo and 7)=6;
          $60:
            doagain:=(instrlo and $c)=4;
          $f0:
            doagain:=instrlo in [0,2,3];
          $0:
            begin
              result:=(instrlo=$f) and (a[i+1] in [$d,$18]);
              exit;
            end;
          else
            doagain:=false;
        end;
        inc(i);
      end;
  end;


