{
    Copyright (c) 1998-2008 by Florian Klaempfl

    This include contains the enumeration
    information about the target systems supported
    (these are not processor specific)

    This program is free software; you can redistribute it and/or modify
    iu under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge- MA 02139, USA.

 ****************************************************************************
}
   type
       tendian = (endian_little,endian_big);

     (*
       IMPORTANT NOTE:
       The value of this enumeration is stored in PPU files.
       Therefore adding new CPU targets should not change the
       values of the pre-existing targets. (CEC)
       FURTHERMORE : Make sure that this branch values, are
       consistant with the main branch version always.
     *)
       tsystemcpu=
       (
             cpu_no,                       { 0 }
             cpu_i386,                     { 1 }
             cpu_m68k,                     { 2 }
             obsolete_cpu_alpha,           { 3 }
             cpu_powerpc,                  { 4 }
             cpu_sparc,                    { 5 }
             obsolete_cpu_vm,              { 6 }
             obsolete_cpu_ia64,            { 7 }
             cpu_x86_64,                   { 8 }
             cpu_mipseb,                   { 9 }
             cpu_arm,                      { 10 }
             cpu_powerpc64,                { 11 }
             cpu_avr,                      { 12 }
             cpu_mipsel,                   { 13 }
             cpu_jvm,                      { 14 }
             cpu_i8086,                    { 15 }
             cpu_aarch64,                  { 16 }
             cpu_wasm,                     { 17 }
             cpu_sparc64,                  { 18 }
             cpu_riscv32,                  { 19 }
             cpu_riscv64                   { 20 }
       );

       tasmmode= (asmmode_none
            { standard assembler (cpu dependant) with full parsing }
            ,asmmode_standard
            ,asmmode_i386_att
            ,asmmode_i386_intel
            ,asmmode_ppc_gas
            ,asmmode_ppc_motorola
            ,asmmode_arm_gas
            ,asmmode_sparc_gas
            ,asmmode_x86_64_gas
            ,asmmode_m68k_mot
            ,asmmode_x86_64_intel
            ,asmmode_x86_64_att
            ,asmmode_avr_gas
            ,asmmode_i8086_intel
            ,asmmode_i8086_att
            ,asmmode_arm_gas_unified
       );

     (* IMPORTANT NOTE:
       the integer value of this enum is stored in PPU
       files to recognize the target, so if you add new targets
       allways add them at end PM
       FURTHERMORE : Make sure that this branch values are
       consistant with the main branch version always. (CEC)
       *)
     type
       tsystem =
       (
             system_none,               { 0 }
             obsolete_system_i386_GO32V1,{ 1 }
             system_i386_GO32V2,        { 2 }
             system_i386_linux,         { 3 }
             system_i386_OS2,           { 4 }
             system_i386_Win32,         { 5 }
             system_i386_freebsd,       { 6 }
             system_m68k_Amiga,         { 7 }
             system_m68k_Atari,         { 8 }
             system_m68k_macos,         { 9 }
             system_m68k_linux,         { 10 }
             system_m68k_PalmOS,        { 11 }
             obsolete_system_alpha_linux,{ 12 }
             system_powerpc_linux,      { 13 }
             system_powerpc_macos,      { 14 }
             system_i386_solaris,       { 15 }
             system_i386_beos,          { 16 }
             system_i386_netbsd,        { 17 }
             system_m68k_netbsd,        { 18 }
             system_i386_Netware,       { 19 }
             obsolete_system_i386_qnx,  { 20 }
             system_i386_wdosx,         { 21 }
             system_sparc_solaris,      { 22 }
             system_sparc_linux,        { 23 }
             system_i386_openbsd,       { 24 }
             obsolete_system_m68k_openbsd,{ 25 }
             system_x86_64_linux,       { 26 }
             system_powerpc_darwin,     { 27 }
             system_i386_EMX,           { 28 }
             system_powerpc_netbsd,     { 29 }
             system_powerpc_openbsd,    { 30 }
             system_arm_linux,          { 31 }
             system_i386_watcom,        { 32 }
             system_powerpc_MorphOS,    { 33 }
             system_x86_64_freebsd,     { 34 }
             system_i386_netwlibc,      { 35 }
             system_powerpc_Amiga,      { 36 }
             system_x86_64_win64,       { 37 }
             system_arm_wince,          { 38 }
             obsolete_system_ia64_win64,{ 39 }
             system_i386_wince,         { 40 }
             system_x86_6432_linux,     { 41 }
             system_arm_gba,            { 42 }
             system_powerpc64_linux,    { 43 }
             system_i386_darwin,        { 44 }
             system_arm_palmos,         { 45 }
             system_powerpc64_darwin,   { 46 }
             system_arm_nds,            { 47 }
             system_i386_embedded,      { 48 }
             system_m68k_embedded,      { 49 }
             obsolete_system_alpha_embedded,{ 50 }
             system_powerpc_embedded,   { 51 }
             system_sparc_embedded,     { 52 }
             obsolete_system_vm_embedded,{ 53 }
             obsolete_system_ia64_embedded,{ 54 }
             system_x86_64_embedded,    { 55 }
             system_mips_embedded,      { 56 }
             system_arm_embedded,       { 57 }
             system_powerpc64_embedded, { 58 }
             system_i386_symbian,       { 59 }
             system_arm_symbian,        { 60 }
             system_x86_64_darwin,      { 61 }
             system_avr_embedded,       { 62 }
             system_i386_haiku,         { 63 }
             system_arm_darwin,         { 64 }
             system_x86_64_solaris,     { 65 }
             system_mipseb_linux,       { 66 }
             system_mipsel_linux,       { 67 }
             system_i386_nativent,      { 68 }
             system_i386_iphonesim,     { 69 }
             system_powerpc_wii,        { 70 }
             system_x86_64_openbsd,     { 71 }
             system_x86_64_netbsd,      { 72 }
             system_powerpc_aix,        { 73 }
             system_powerpc64_aix,      { 74 }
             system_jvm_java32,         { 75 }
             system_jvm_android32,      { 76 }
             system_arm_android,        { 77 }
             system_i386_android,       { 78 }
             system_i8086_msdos,        { 79 }
             system_mipsel_android,     { 80 }
             system_mipseb_embedded,    { 81 }
             system_mipsel_embedded,    { 82 }
             system_i386_aros,          { 83 }
             system_x86_64_aros,        { 84 }
             system_x86_64_dragonfly,   { 85 }
             system_aarch64_darwin,     { 86 }
             system_x86_64_iphonesim,   { 87 }
             system_aarch64_linux,      { 88 }
             system_i8086_win16,        { 89 }
             system_i8086_embedded,     { 90 }
             system_arm_aros,           { 91 }
             system_wasm_wasm32,        { 92 }
             system_sparc64_linux,      { 93 }
             system_sparc64_solaris,    { 94 }
             system_arm_netbsd,         { 95 }
             system_riscv32_linux,      { 96 }
             system_riscv64_linux,      { 97 }
             system_riscv64_embedded,   { 98 }
             system_riscv32_embedded,   { 99 }
             system_aarch64_android,    { 100 }
             system_x86_64_android,     { 101 }
             system_x86_64_haiku        { 102 }
       );

     type
       tasm = (as_none
             ,as_gas                   { standard gnu assembler }
             ,as_i386_as_aout
             ,as_i386_nasmcoff
             ,as_i386_nasmwin32
             ,as_i386_nasmwdosx
             ,as_i386_nasmelf
             ,as_i386_nasmobj
             ,as_i386_nasmbeos
             ,as_i386_tasm
             ,as_i386_masm
             ,as_i386_wasm
             ,as_i386_coff
             ,as_i386_pecoff
             ,as_i386_elf32
             ,as_i386_pecoffwdosx
             ,as_m68k_mit
             ,as_powerpc_mpw
             ,as_darwin
             ,as_i386_macho
             ,as_x86_64_masm
             ,as_x86_64_pecoff
             ,as_i386_pecoffwince
             ,as_arm_pecoffwince
             ,as_x86_64_elf64
             ,as_sparc_elf32
             ,as_ggas                  { gnu assembler called "gas" instead of "as" }
             ,as_i386_nasmhaiku
             ,as_powerpc_vasm
             ,as_i386_nlmcoff
             ,as_powerpc_xcoff
             ,as_jvm_jasmin
             ,as_yasm
             ,as_i386_nasm
             ,as_i386_nasmdarwin
             ,as_x86_64_nasm
             ,as_x86_64_nasmwin64
             ,as_x86_64_nasmelf
             ,as_x86_64_nasmdarwin
             ,as_i8086_nasm
             ,as_i8086_nasmobj
             ,as_gas_powerpc_xcoff
             ,as_arm_elf32
             ,as_i8086_omf
             ,as_llvm_llc
             ,as_clang
             ,as_solaris_as
             ,as_m68k_vasm
             ,as_m68k_as_aout
             ,as_wasm_binaryen
             ,as_powerpc_gas_legacy    { for systems with very old GAS versions only, which don't support eg. named sections }
             ,as_llvm_clang
       );

       tlink = (ld_none,
             ld_aix,           { external linkers (one per OS, handles all CPUs) }
             ld_amiga,
             ld_aros,
             ld_atari,
             ld_android,
             ld_beos,
             ld_bsd,
             ld_embedded,
             ld_emx,
             ld_gba,
             ld_go32v2,
             ld_haiku,
             ld_jvm,
             ld_linux,
             ld_morphos,
             ld_mpw,
             ld_msdos,
             ld_nds,
             ld_netwlibc,
             ld_netware,
             ld_os2,
             ld_palmos,
             ld_solaris,
             ld_watcom,
             ld_wdosx,
             ld_wii,
             ld_windows,
             ld_win16,
             ld_int_go32v2,    { implemented internal linkers }
             ld_int_linux,
             ld_int_nativent,
             ld_int_netware,
             ld_int_windows,
             ld_int_msdos,
             ld_int_win16
       );

       tar = (ar_none
            ,ar_gnu_ar
            ,ar_mpw_ar
            ,ar_gnu_ar_scripted
            ,ar_gnu_gar
            ,ar_watcom_wlib_omf
            ,ar_watcom_wlib_omf_scripted
       );

       tres = (res_none
            ,res_gnu_windres,res_watcom_wrc_os2
            ,res_m68k_palmos,res_m68k_mpw
            ,res_powerpc_mpw,res_elf,res_xcoff
            ,res_win64_gorc, res_macho, res_ext
            ,res_jvm_raw
       );

       tresinfoflags = (res_external_file,res_arch_in_file_name
            ,res_single_file,res_no_compile);

       tdbg = (dbg_none
            ,dbg_stabs,dbg_stabx,dbg_dwarf2,dbg_dwarf3,dbg_dwarf4,dbg_jasmin
            ,dbg_codeview
       );

       tscripttype = (script_none
            ,script_dos,script_unix,script_amiga,
            script_mpw,
            script_fpcres
       );

       tabi = (abi_default
            ,abi_powerpc_sysv,abi_powerpc_aix,abi_powerpc_darwin,abi_powerpc_elfv2
            ,abi_eabi,abi_armeb,abi_eabihf
            ,abi_old_win32_gnu
            ,abi_aarch64_darwin
            ,abi_riscv_hf
            { stack is aligned and all room for parameters is reserved on
              entry, but depending on the calling convention, the parameters
              may still be removed by the callee (and then the stack needs to
              be restored by the caller) }
            ,abi_i386_dynalignedstack
       );

     const
       abi_powerpc_elfv1 = abi_powerpc_sysv;


