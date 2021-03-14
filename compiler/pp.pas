{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Commandline compiler for Free Pascal

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
program pp;

{
  possible compiler switches:
  -----------------------------------------------------------------
  Supported CPUs, alphabetically sorted
  -----------------------------------------------------------------
  AARCH64             generate a compiler for the AARCH64 (64bit ARM)
  ARM                 generate a compiler for the ARM
  AVR                 generate a compiler for the AVR
  I386                generate a compiler for the Intel i386+
  I8086               generate a compiler for the Intel 8086+
  JVM                 generate a compiler for the JVM (Java Virtual Machine)
  M68K                generate a compiler for the M68000
  MIPS                generate a compiler for the MIPS (Big Endian)
  MIPSEL              generate a compiler for the MIPSEL (Littel Endian)
  POWERPC             generate a compiler for the PowerPC
  POWERPC64           generate a compiler for the PowerPC64 architecture
  RISCV32             generate a compiler for the RiscV32 architecture
  RISCV64             generate a compiler for the RiscV64 architecture
  SPARC               generate a compiler for SPARC
  SPARC64             generate a compiler for SPARC64
  WASM32              generate a compiler for WebAssembly 32-bit
  X86_64              generate a compiler for the AMD x86-64 architecture
  XTENSA              generate a compiler for XTENSA
  Z80                 generate a compiler for Z80

  -----------------------------------------------------------------
  Other compiler switches
  -----------------------------------------------------------------
  CMEM                use cmem unit for better memory debugging
=======
  DEBUG               version with debug code is generated
  EXTDEBUG            some extra debug code is executed
  EXTERN_MSG          Don't compile the msgfiles in the compiler, always
                      use external messagefiles, default for TP
  LLVM                Create an LLVM-based code generator for the selected
                      target architecture (not supported for all targets)

  -----------------------------------------------------------------
  ARM specfic switches
  -----------------------------------------------------------------
  FPC_ARMEL           create an arm eabi compiler
  FPC_ARMEB           create an arm big endian compiler
  FPC_OARM            create an arm oabi compiler, only needed when the host
                      compiler is ARMEL or ARMEB
  FPC_ARMHF           create an armhf (eabi vfp variant) compiler

  -----------------------------------------------------------------
  I386 specfic switches
  -----------------------------------------------------------------
  SUPPORT_MMX         only i386: releases the compiler switch
                      MMX which allows the compiler to generate
                      MMX instructions
  -----------------------------------------------------------------
  Switches automatically inside fpcdefs.inc  
  -----------------------------------------------------------------
  cpuflags            The target processor has status flags (on by default)
  cpufpemu            The target compiler will also support emitting software
                       floating point operations
  cpu64bitaddr        Generate code for a 64-bit address space
  cpu64bitalu         The target cpu has 64-bit registers and a 64 bit alu
                      (required for cpu64bitaddr; optional with 32 bit addr space)
  -----------------------------------------------------------------
}

{$i fpcdefs.inc}

{ Require at least 2.6.0 }
{$if FPC_FULLVERSION<20600}
  {$fatal At least FPC 2.6.0 is required to compile the compiler}
{$endif}

{ exactly one target CPU must be defined }
{$ifdef I8086}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif I8086}
{$ifdef I386}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif I386}
{$ifdef x86_64}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif x86_64}
{$ifdef M68K}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif M68K}
{$ifdef POWERPC}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif POWERPC}
{$ifdef POWERPC64}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif POWERPC64}
{$ifdef SPARC}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif SPARC}
{$ifdef SPARC64}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif SPARC64}
{$ifdef ARM}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif ARM}
{$ifdef MIPS}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif MIPS}
{$ifdef AVR}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif AVR}
{$ifdef JVM}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif}
{$ifdef AARCH64}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif AARCH64}
{$ifdef RISCV32}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif RISCV32}
{$ifdef RISCV64}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif RISCV64}
{$ifdef XTENSA}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif XTENSA}
{$ifdef Z80}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif Z80}
{$ifdef WASM32}
  {$ifdef CPUDEFINED}
    {$fatal ONLY one of the switches for the CPU type must be defined}
  {$endif CPUDEFINED}
  {$define CPUDEFINED}
{$endif WASM32}

{$ifndef CPUDEFINED}
  {$fatal A CPU type switch must be defined}
{$endif CPUDEFINED}
{$ifdef support_mmx}
  {$ifndef i386}
    {$fatal I386 switch must be on for MMX support}
  {$endif i386}
{$endif support_mmx}


{ Don't care about minstacksize or maxstacksize not beeing supported by current OS }
{$WARN 2077 OFF}
{$WARN 2078 OFF}

{$ifdef win32}
  { 256 MB stack }
  { under windows the stack can't grow }
  {$MAXSTACKSIZE 256000000}
  {$setpeflags $20}
{$else win32}
  {$ifdef win64}
    { 512 MB stack }
    { under windows the stack can't grow }
    {$MAXSTACKSIZE 512000000}
  {$else win64}
    { 1 MB stack }
    {$MINSTACKSIZE 1000000}
  {$endif win64}
{$endif win32}

uses
{$ifdef heaptrc}
  ppheap,
{$endif heaptrc}
{$ifdef cmem}
  cmem,
{$endif cmem}
{$ifdef profile}
  profile,
{$endif profile}
{$ifndef NOCATCH}
  {$if defined(Unix) or defined(Go32v2) or defined(Watcom)}
    catch,
  {$endif}
{$endif NOCATCH}
  globals,compiler;

var
  oldexit : pointer;
procedure myexit;
begin
  exitproc:=oldexit;
{$ifdef nocatch}
  exit;
{$endif nocatch}
{ Show Runtime error if there was an error }
  if (erroraddr<>nil) then
   begin
     case exitcode of
      100:
        begin
           erroraddr:=nil;
           writeln('Error while reading file');
        end;
      101:
        begin
           erroraddr:=nil;
           writeln('Error while writing file');
        end;
      202:
        begin
           erroraddr:=nil;
           writeln('Error: Stack Overflow');
        end;
      203:
        begin
           erroraddr:=nil;
           writeln('Error: Out of memory');
        end;
     end;
     { we cannot use current_filepos.file because all memory might have been
       freed already !
       But we can use global parser_current_file var }
     Writeln('Compilation aborted ',parser_current_file,':',current_filepos.line);
   end;
end;

begin
  oldexit:=exitproc;
  exitproc:=@myexit;
{$ifdef EXTDEBUG}
{ Increase the maximum stack trace depth, since the default 8 is often not
  enough for debugging the compiler }
  Max_Frame_Dump:=50;
{$endif EXTDEBUG}
{ Call the compiler with empty command, so it will take the parameters }
  Halt(compiler.Compile(''));
end.
