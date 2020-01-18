/ prt0.s (emx+fpc) -- Made from crt0.s,
/                     Copyright (c) 1990-1999-2001 by Eberhard Mattes.
/                     Changed for Free Pascal in 1997 Daniel Mantione.
/                     This code is based on source distributed under the 
/                     emx-lib license, see copying.emx for details.
/                     You should have received it with this
/                     product, write the author if you haven't.

        .globl  __text
        .globl  ___SYSCALL
        .globl  __data
        .globl  __init
        .globl  __dos_init
        .globl  __dos_syscall
        .comm _excptregptr, 4

        .text

__text:
        push    $__data
        call    __dos_init
        jmp     __init

___SYSCALL:
        call    __dos_syscall
        ret

        .space  6, 0x90

__init: cld
        pushl %eax
        pushl %eax
        pushl %eax
        movl %esp,%eax
        addl $4,%eax
        movl %eax, _excptregptr
        popl %eax

        call    _main

        movb    $0x4c,%ah
        call    ___SYSCALL
2:      jmp     2b

/ In executables created with emxbind, the call to _dos_init will
/ be fixed up at load time to _emx_init of emx.dll.  Under DOS,
/ this dummy is called instead as there is no fixup.  This module
/ must be linked statically to avoid having two fixups for the
/ same location.

__dos_init:
        ret     $4

        .align  2, 0x90

__dos_syscall:
        int     $0x21
        ret

        .data

/ The data segment starts with a table containing the start and end
/ addresses of the text, data and bss segments

__data:
        .long   __text
        .long   __etext
        .long   __data
        .long   __edata
        .long   __edata
        .long   __end
__heap_base:
        .long   0
__heap_end:
        .long   0
__heap_brk:
        .long   0
        .long   0
        .long   __os2dll
        .long   0
        .long   0
        .long   0x02000000
        .long   0
        .long   0
        .byte   0
        .space  63, 0

/ Don't touch this. It's EMX vodoo. In short, this causes the __os2dll symbol
/ point to table of DLL data that the linker includes in the executable.

        .stabs  "__os2dll", 21, 0, 0, 0xffffffff
        .stabs  "___CTOR_LIST__", 21, 0, 0, 0xffffffff
        .stabs  "___DTOR_LIST__", 21, 0, 0, 0xffffffff
        .stabs  "___crtinit1__", 21, 0, 0, 0xffffffff
        .stabs  "___crtexit1__", 21, 0, 0, 0xffffffff
        .stabs  "___eh_frame__", 21, 0, 0, 0xffffffff
