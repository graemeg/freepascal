/* Copyright (C) 1995 DJ Delorie, see COPYING.DJ for details */
/*****************************************************************************\
 * Interface to 32-bit executable (from stub.asm)
 *
 *   cs:eip     according to COFF header
 *   ds  32-bit data segment for COFF program
 *   fs  selector for our data segment (fs:0 is stubinfo)
 *   ss:sp      our stack (ss to be freed)
 *   <others>   All unspecified registers have unspecified values in them.
\*****************************************************************************/
/* modified by Pierre Muller to become the prt0.s for FPC Pascal */

        .file "v2prt0.as"

/* #include "stubinfo.h" */
 STUBINFO = 0
 STUBINFO_MAGIC = 0
 STUBINFO_SIZE = 0x10
 STUBINFO_MINSTACK = 0x14
 STUBINFO_MEMORY_HANDLE = 0x18
 STUBINFO_INITIAL_SIZE = 0x1c
 STUBINFO_MINKEEP = 0x20
 STUBINFO_DS_SELECTOR = 0x22
 STUBINFO_DS_SEGMENT = 0x24
 STUBINFO_PSP_SELECTOR = 0x26
 STUBINFO_CS_SELECTOR = 0x28
 STUBINFO_ENV_SIZE = 0x2a
 STUBINFO_BASENAME = 0x2c
 STUBINFO_ARGV0 = 0x34
 STUBINFO_DPMI_SERVER = 0x44
 STUBINFO_END = 0x54


/*      .comm   __stklen, 4
        this is added to the compiler so that we can specify
        the stack size */
        .comm   __stkbottom,4
        .comm   __stubinfo, 4
        .comm   ___djgpp_base_address, 4
        .comm   ___djgpp_selector_limit, 4
        .comm   __crt0_startup_flags, 4
        .comm   ___djgpp_stack_limit, 4
        .lcomm  sel_buf, 8

/* ___djgpp_ds_alias defined in go32/exceptn.s */
/* inserted at the end of this file  */
/* we use a local copy that will be copied to exceptn.s */
        .globl ___v2prt0_ds_alias
___v2prt0_ds_alias:
        .long  0
/* allocate 32*4 bytes for RMCB under the $ffff limit for Windows NT */
   .globl ___v2prt0_rmcb_regs
___v2prt0_rmcb_regs:
   .long  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
   .long  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
.data

___djgpp_memory_handle_pointer:
        .long   ___djgpp_memory_handle_list+8      /* Next free, first for stub */
        .comm   ___djgpp_memory_handle_list, 2048       /* Enough for 256 handles */

   /* simply get current state */
___sbrk_interrupt_state:
        .long   0x902

sbrk16_first_byte:
.include "sbrk16.ah"
sbrk16_last_byte:

sbrk16_api_ofs:
        .long   0
sbrk16_api_seg:
        .word   0
zero:
        .long   0

exit16_first_byte:
.include "exit16.ah"
exit16_last_byte:

/* hook_387_emulator:
        .long   ___emu387_load_hook */

/* this is for when main comes from a library */
        .long   _main


.text
        .globl  start
start:
        pushl   %ds                  /* set %es same as %ds */
        popl    %es                  /* push/pop 4 bytes shorter than ax */

/* Enable NULL pointer protection if DPMI supports it */
        testb   $0x1, __crt0_startup_flags+1        /* include/crt0.h */
        jnz     1f
        movl    $start, %eax
        cmpl    $0x1000, %eax
        jl      1f
        movw    $0x507, %ax
        .byte 0x64 /* fs: */
        movl    STUBINFO_MEMORY_HANDLE, %esi
        xorl    %ebx, %ebx                    /* Offset 0 in mem block */
        movl    $1, %ecx                        /* Set one page */
        movl    $zero, %edx
        int     $0x31              /* Make null page uncommitted */
        jnc     1f
        call    v2prt0_windows
1:
/* Create an alias for DS to be used by real-mode callbacks (exception handler messes with DS itself) */

        movw    %ds, %bx
        movw    $0x000a, %ax
        int     $0x31
        jnc     .Lds_alias_ok
        movb    $0x4c, %ah
        int     $0x21

.Lds_alias_ok:
        movw    %ax, ___v2prt0_ds_alias
        movl    %eax, %ebx
        movw    $0x0009, %ax
        movw    %cs, %cx        /* get CPL from %cs */
        andl    $3, %ecx
        shll    $5, %ecx                /* move it into place */
        orw     $0xc093, %cx
        int     $0x31      /* set access rights for alias */

/* Maybe set our DS limit to 4Gb in size if flag set */
        testb   $0x80, __crt0_startup_flags          /* include/crt0.h */
        jz      2f
        movw    $0xffff, %cx
        movl    %ecx, %edx
        movw    $0x0008, %ax                        /* reset alias limit to -1 */
        int     $0x31
        movw    %cs, %bx
        movw    $0x0008, %ax                        /* reset DS limit to -1 */
        int     $0x31
        movw    %ds, %bx
        movw    $0x0008, %ax                        /* reset DS limit to -1 */
        int     $0x31
        lsl     %ebx, %ebx                            /* Should be -1 */
        incl    %ebx
        jz      2f
        andb    $0x7f, __crt0_startup_flags          /* clear it if failure */
2:
/* Allocate some DOS memory and copy our sbrk helper into it. */
        movl    $sbrk16_first_byte, %esi
        movzwl  8(%esi), %ebx
        shrl    $4, %ebx
        movw    $0x0100, %ax
        int     $0x31
        jnc     .Ldos_alloc_ok
        movb    $0x4c, %ah
        int     $0x21

.Ldos_alloc_ok:
        movw    %cs, 2(%esi)
/* store API information */
        movw    %ds, 4(%esi)
        movw    %dx, 6(%esi)
/* selector for allocated block */

        movzwl  (%esi), %eax                /* calculate API address */
        movl    %eax, sbrk16_api_ofs

        pushl   %es                          /* move the data */
        movw    %dx, %es
        movl    $(sbrk16_last_byte - sbrk16_first_byte), %ecx
        shrl    $2,%ecx
        xorl    %edi, %edi
        cld
        rep
        movsl
        popl    %es

        movl    %edx, %ebx                    /* dos memory selector */
        movw    $0x000b, %ax                /* get descriptor */
        movl    $sel_buf, %edi
        int     $0x31

        andb    $0xbf, sel_buf+6                /* make 16-bit */
        andb    $0xf0, sel_buf+5                /* remove old type */
        orb     $0x0a, sel_buf+5                /* set new type to code/read */

        xorl    %eax, %eax                    /* allocate new selector */
        movw    $0x0001, %cx
        int     $0x31
        movw    %ax, sbrk16_api_seg

        movl    %eax, %ebx
        movw    $0x000c, %ax                /* set descriptor */
        movl    $sel_buf, %edi
        int     $0x31

/* Initialize the brk/sbrk variables */

/*      movl    $end, __what_size_app_thinks_it_is */
        .byte 0x64 /* fs: */
        movl    STUBINFO_INITIAL_SIZE, %eax
        movl    %eax, __what_size_dpmi_thinks_we_are

/* Maybe lock the initial block, expects BX:CX */
        movl    %ecx,%ebx
        movl    %edx,%ecx
        addw    $4096,%cx                      /* Skip null page */
        adcl    $0,%ebx
        subl    $4096,%eax
        pushl   %eax
        call    lock_memory

        .byte 0x64 /* fs: */
        movl    STUBINFO_MEMORY_HANDLE, %eax
        movl    %eax, ___djgpp_memory_handle_list

        .byte 0x64 /* fs: */                /* copy stubinfo into local memory */
        movl    STUBINFO_SIZE, %eax
        pushl   %eax
        call    ___sbrk
        movl    %eax, __stubinfo
        movl  %eax,operatingsystem_stub_info
        movl    %eax, %edi
        .byte 0x64 /* fs: */
        movl    STUBINFO_SIZE, %ecx
        shrl    $2, %ecx
        xorl    %esi, %esi                    /* Zero */
        pushl   %ds
        pushl   %fs
        popl    %ds
        cld
        rep
        movsl
        popl    %ds
        movl    __stklen, %eax    /* get program-requested stack size */
        .byte 0x64 /* fs: */
        movl    STUBINFO_MINSTACK, %ecx /* get stub-requested stack size */
        cmpl    %ecx, %eax
        jge     .Luse_stubinfo_stack_size /* use the larger of the two */
        movl    %ecx, %eax
        movl    %eax, __stklen    /* store the actual stack length */
.Luse_stubinfo_stack_size:
        pushl   %eax
        call    ___sbrk          /* allocate the memory */
        cmpl    $-1, %eax
        je      .Lno_memory
        movl    %eax, ___djgpp_stack_limit      /* Bottom of stack */
        addl    $256,%eax
        movl    %eax,__stkbottom               /* for stack checks */
        /* movl    %eax,operatingsystem_stackbottom  */
        /* StackBottom is
        a ThrteadVar and can not be given a symbol with name,
        copying value of __stkbottom to system.STackBottom variable
        is done in system unit startup code. PM */

        movl    ___djgpp_stack_limit,%eax       /* Bottom of stack */
        addl    __stklen, %eax
        movw    %ds, %dx                /* set stack */
        movw    %dx, %ss
        andl    $0xfffffffc,%eax
        movl    %eax, %esp

        xorl    %ebp, %ebp
        call    ___prt1_startup  /* run program */
        jmp     exit

.Lno_memory:
        movb    $0xff, %al
        jmp     exit

/*-----------------------------------------------------------------------------*/

/* #define FREESEL(x) movw x, %bx; movw $0x0001, %ax; int $0x31 */
   .macro FREESEL x
     movw \x,%bx
     movw $0x0001,%ax
     int $0x31
   .endm

        .global ___exit
        .align  2
___exit:
/* special exit from dpmiexcp.c */
        .global __exit
__exit:
        movl    4(%esp),%eax
exit:
        movl    %eax,%ecx
        xorl    %eax,%eax
        movw    %ax,%fs
        movw    %ax,%gs
        cmpl    $0,_exception_exit
        jz      .Lno_exception
        pushl   %ecx
        call    *_exception_exit
        popl    %ecx
.Lno_exception:
        cli                          /* Just in case they didn't unhook ints */
        FREESEL operatingsystem_go32_info_block+26     /* selector for linear memory */
        FREESEL ___v2prt0_ds_alias      /* DS alias for rmcb exceptions */
        FREESEL sbrk16_api_seg    /* sbrk cs */
        movw    sbrk16_first_byte+6,%dx /* selector for allocated DOS mem */
        movw    $0x101, %ax
        int     $0x31              /* Free block and selector */
9:
        movl    __stubinfo, %edx
        movl    STUBINFO_CS_SELECTOR(%edx), %eax
        movw    %ax, sbrk16_api_seg
        xorl    %edi, %edi
        movl    %edi, sbrk16_api_ofs    /* Offset is zero */

        movw    STUBINFO_DS_SELECTOR(%edx), %es
        movb    %cl, %dl                /* Exit status */
        movl    $exit16_first_byte, %esi
        movl    $(exit16_last_byte - exit16_first_byte), %ecx
        cld
        rep
        movsb

        movw    %es,%ax          /* We will free stack! */
        movw    %ax,%ss
        movl    $0x400,%esp          /* Transfer buffer >= 1024 bytes */

        xorl    %ebp, %ebp                            /* V1.10 bug fix */
        movl    ___djgpp_memory_handle_list, %edi
        movl    ___djgpp_memory_handle_list+2, %esi     /* Skip word prefixes */

        FREESEL %ds
        movw    %cs, %bx
/* Call exit procedure with BX=32-bit CS; SI+DI=32-bit handle; DL=exit status */
        .byte 0x2e
        ljmp    sbrk16_api_ofs

/*-----------------------------------------------------------------------------*/

/*      .lcomm  __what_size_app_thinks_it_is, 4 */
__what_size_app_thinks_it_is:
        .long   end
        .lcomm  __what_we_return_to_app_as_old_size, 4
        .lcomm  __what_size_dpmi_thinks_we_are, 4

lock_memory:
        /* BX:CX should be linear address; size is pushed on stack */
        testb   $0x10, __crt0_startup_flags+1      /* include/crt0.h */
        jz      13f
        pushl   %esi
        pushl   %edi
        pushl   %eax
        movl    16(%esp),%edi
        movw    18(%esp),%si
        movw    $0x600,%ax
        int     $0x31
        popl    %eax
        popl    %edi
        popl    %esi
13:     ret     $4                    /* Pop the argument */


        .global ___sbrk
        .align  2
___sbrk:
        movl    __what_size_app_thinks_it_is, %eax
        movl    4(%esp), %ecx              /* Increment size */
        addl    %ecx, %eax
        jnc     .Lbrk_common
        /* Carry is only set if a negative increment or wrap happens. Negative
           increment is semi-OK, wrap (only for multiple zone sbrk) isn't. */
        test    $0x80000000, %ecx              /* Clears carry */
        jnz     .Lbrk_common
        stc                                  /* Put carry back */
        jmp     .Lbrk_common

        .globl  ___brk
        .align  2
___brk:
        movl    4(%esp), %eax
        clc

.Lbrk_common:
        pushl   %esi
        pushl   %edi
        pushl   %ebx

        movl    __what_size_app_thinks_it_is, %edx            /* save info */
        movl    %edx, __what_we_return_to_app_as_old_size
        movl    %eax, __what_size_app_thinks_it_is

        /* multi code is not present */
        /* jc      10f                                           Wrap for multi-zone */
        cmpl    __what_size_dpmi_thinks_we_are, %eax        /* don't bother shrinking */
        jbe     .Lbrk_nochange

        addl    $0x0000ffff, %eax                              /* round up to 64K block */
        andl    $0xffff0000, %eax
        push    %eax                                        /* size - save for later */

        movl    ___djgpp_memory_handle_list, %edi              /* request new size */
        movw    ___djgpp_memory_handle_list+2, %si
        movl    %eax, %ecx                                    /* size not limit */
        movl    %eax, %ebx                                    /* size not limit */
        shrl    $16, %ebx                                      /* BX:CX size */

        movw    $0x0900, %ax                                /* disable interrupts */
        int     $0x31
        movl    %eax,___sbrk_interrupt_state
        lcall   sbrk16_api_ofs
        setc    %dl                                          /* Save carry */

        /* popl    %eax                                restore interrupts
        int     $0x31 postponed after ds alias is set correctly */

        test    %dl,%dl
        popl    %edx
        jne     .Lbrk_error

        movl    %edi, ___djgpp_memory_handle_list              /* store new handle */
        movw    %si, ___djgpp_memory_handle_list+2
        movl    %ecx, ___djgpp_base_address                  /* store new base address */
        movw    %bx, ___djgpp_base_address+2

        movl    %edx, %eax
        movl    __what_size_dpmi_thinks_we_are, %ecx
        subl    %ecx, %eax

        addl    ___djgpp_base_address, %ecx
        movl    %ecx, %ebx
        shrl    $16, %ebx                                      /* BX:CX addr */
        pushl   %eax                                        /* Size */
        call    lock_memory

        decl    %edx                                        /* limit now, not size */
5:      movl    %edx, ___djgpp_selector_limit
        orw     $0x0fff, %dx                                /* low bits set */
        movw    $0x0008, %ax                                /* reset CS limit */
        movw    %cs, %bx
        movl    %edx, %ecx
        shrl    $16, %ecx
        int     $0x31                                      /* CX:DX is limit */

        testb   $0x80, __crt0_startup_flags                  /* include/crt0.h */
        jnz     3f
        movw    $0x0008, %ax                                /* reset DS limit */
        movw    %ds, %bx
        int     $0x31

        movw    $0x0008, %ax                                /* reset DS alias limit */
        movl    ___v2prt0_ds_alias, %ebx
        int     $0x31
3:
        movw    $0x0007, %ax                                /* reset DS alias base */
        movl    ___v2prt0_ds_alias, %ebx
        movl    ___djgpp_base_address, %edx
        movw    ___djgpp_base_address+2, %cx
        int     $0x31

        movl    ___sbrk_interrupt_state,%eax                /* restore interrupts */
        int     $0x31
        movl    ___djgpp_selector_limit, %edx
12:     incl    %edx                                        /* Size not limit */
        testb   $0x60, __crt0_startup_flags     /* include/crt0.h */
        jz      .Lno_fill_sbrk_memory
        pushl   %ds
        popl    %es

        movl    __what_size_dpmi_thinks_we_are, %edi        /* set all newly resized bytes zero */
        movl    %edx, %ecx                                    /* Limit */
        subl    %edi, %ecx                    /* Adjust count for base */
        xorl    %eax, %eax
        testb   $0x40, __crt0_startup_flags
        jz      .Lno_deadbeef
        movl    $0xdeadbeef, %eax              /* something really easy to spot */
.Lno_deadbeef:
        shrl    $2, %ecx                        /* div 4 Longwords not bytes */
        cld
        rep
        stosl
.Lno_fill_sbrk_memory:
        movl    %edx, __what_size_dpmi_thinks_we_are

.Lbrk_nochange:                              /* successful return */
        movl    __what_we_return_to_app_as_old_size, %eax
        jmp     .Lbrk_return

.Lbrk_error:                                    /* error return */
        movl    __what_we_return_to_app_as_old_size, %eax
        movl    %eax, __what_size_app_thinks_it_is
        movl    $0, %eax

.Lbrk_return:
        popl    %ebx
        popl    %edi
        popl    %esi
        ret

/* From here on this are parts of crt1.c converted to assembler
and without any call to libc, so that it works without anything else
additions made by Pierre Muller*/
/* from dpmidefs.h * /
/* Copyright (C) 1995 DJ Delorie, see COPYING.DJ for details */
/* from include <libc/asmdefs.h> */
/* all macros removed here */
/* #define FUNC(x)            .globl x; x: */

/* #define ENTER                pushl %ebp; movl %esp,%ebp */

/* #define LEAVE(x)     movl %ebp,%esp; popl %ebp; ret $(x) */
/* #define ARG1  8(%ebp)
#define ARG1h      10(%ebp)
#define ARG2        12(%ebp)
#define ARG2h      14(%ebp)
#define ARG3        16(%ebp)
#define ARG4        20(%ebp)
#define ARG5        24(%ebp)
#define ARG6        28(%ebp)
#define ARG7        32(%ebp)
#define ARG8        36(%ebp) */

        .comm   ___dpmi_error,2

/* from dpmi0000.s */
/*      .globl ___dpmi_allocate_ldt_descriptors */
/* using pascal convention => not usabel by C code */
___dpmi_allocate_ldt_descriptors:
        pushl %ebp; movl %esp,%ebp

        movl    8(%ebp), %ecx
        movl $0x0000, %eax
        int $0x31
        jnc .L_noerror0000
        movw %ax,___dpmi_error
        movl $-1,%eax
        jmp .L_leave0000
.L_noerror0000:
        movzwl  %ax,%eax
.L_leave0000:
        movl %ebp,%esp
        popl %ebp
        ret $4

/* from file dpmi0008.s */
/*      .globl ___dpmi_set_segment_limit */
___dpmi_set_segment_limit:
   pushl %ebp; movl %esp,%ebp

        movl    8(%ebp), %ebx
        movzwl  12(%ebp), %edx
        movzwl  14(%ebp),%ecx

        movl     $0x0008,%eax
        int $0x31
        jnc .L_noerror0008
        movw %ax,___dpmi_error
        movl $-1,%eax
        jmp .L_leave0008
.L_noerror0008:
        xorl    %eax,%eax
.L_leave0008:
        movl %ebp,%esp
        popl %ebp
        ret $8

/*      .globl ___dpmi_get_version */
___dpmi_get_version:
   pushl %ebp; movl %esp,%ebp

        movl     $0x0400,%eax
        int $0x31
        jnc .L_noerror0400
        movw %ax,___dpmi_error
        movl $-1,%eax
        jmp .L_leave0400
.L_noerror0400:
        movl    8(%ebp), %esi
        movb    %ah, (%esi)
        movb    %al, 1(%esi)
        movw    %bx, 2(%esi)
        movb    %cl, 4(%esi)
        movb    %dh, 5(%esi)
        movb    %dl, 6(%esi)

        xorl    %eax,%eax
.L_leave0400:

        movl %ebp,%esp
        popl %ebp
        ret $4

_set_os_trueversion:
        pushl %ebp
        movl  %esp,%ebp
        movl  $0x3306,%eax
        xorl  %ebx,%ebx
        int   $0x21
        movzbl %bl,%eax
        shll  $8,%eax
        shrl  $8,%ebx
        andl  $0xff,%ebx
        addl  %ebx,%eax
        movw  %ax,__os_trueversion
        popl  %ebp
        ret
/*       .globl ___dpmi_get_segment_base_address*/
___dpmi_get_segment_base_address:
   pushl %ebp; movl %esp,%ebp

        movl    8(%ebp), %ebx
        movl     $0x0006,%eax
        int $0x31
        jnc .L_noerror0006
        movw %ax,___dpmi_error
        movl $-1,%eax
        jmp .L_leave0006
.L_noerror0006:

        movl    12(%ebp), %ebx
        movl    %edx, (%ebx)
        movw    %cx, 2(%ebx)

        xorl    %eax,%eax
.L_leave0006:
        movl %ebp,%esp
        popl %ebp
        ret $8

.globl ___bss_count
.data
        .align 2
___bss_count:
        .long 1
.text
        .align 2
        .globl _setup_core_selector
_setup_core_selector:
        pushl %ebp
        movl %esp,%ebp
        pushl $1
        call ___dpmi_allocate_ldt_descriptors
        /* addl $4,%esp */
        cmpl $-1,%eax
        jne .L24
        movw $0,operatingsystem_go32_info_block+26
        leave
        ret
        .align 2,0x90
.L24:
        movw %ax,operatingsystem_go32_info_block+26
        movw %ax,_core_selector
        pushl $0x10ffff
        andl $0xffff,%eax
        pushl %eax
        call ___dpmi_set_segment_limit
        leave
        ret
        .align 2
        .globl _setup_screens
_setup_screens:
        pushl %ebp
        movl %esp,%ebp
        movw operatingsystem_go32_info_block+26,%dx
        movl $1048563,%ecx
/APP
        movw %dx, %gs
        .byte 0x65
        movw (%ecx),%ax
/NO_APP
        cmpw $64896,%ax
        jne .L26
        movl $655360,operatingsystem_go32_info_block+8
        movl $655360,operatingsystem_go32_info_block+4
        leave
        ret
        .align 2,0x90
.L26:
        movl $1097,%ecx
/APP
        movw %dx,%gs
        .byte 0x65
        movb (%ecx),%al
/NO_APP
        cmpb $7,%al
        jne .L29
        movl $720896,operatingsystem_go32_info_block+4
        movl $753664,operatingsystem_go32_info_block+8
        leave
        ret
        .align 2,0x90
.L29:
        movl $753664,operatingsystem_go32_info_block+4
        movl $720896,operatingsystem_go32_info_block+8
        leave
        ret

        .align 2
        .globl _setup_go32_info_block
_setup_go32_info_block:
        pushl %ebp
        movl %esp,%ebp
        subl $8,%esp
        leal -8(%ebp),%eax
        pushl %eax
        call ___dpmi_get_version
        movl $40,operatingsystem_go32_info_block
        movl __stubinfo,%edx
        movzwl 36(%edx),%eax
        sall $4,%eax
        movl %eax,operatingsystem_go32_info_block+12
        movzwl 32(%edx),%ecx
        movl %ecx,operatingsystem_go32_info_block+16
        movzwl 38(%edx),%ecx
        movl %ecx,operatingsystem_go32_info_block+20
        movb -3(%ebp),%al
        movb %al,operatingsystem_go32_info_block+24
        movb -2(%ebp),%al
        movb %al,operatingsystem_go32_info_block+25
        movl $-1,operatingsystem_go32_info_block+28
        pushl $operatingsystem_go32_info_block+32
        movzwl 38(%edx),%eax
        pushl %eax
        call ___dpmi_get_segment_base_address
        movw $4,operatingsystem_go32_info_block+36
        movb -8(%ebp),%dl
        salw $8,%dx
        movzbw -7(%ebp),%ax
        orw %ax,%dx
        movw %dx,operatingsystem_go32_info_block+38
        call copy_to_c_go32_info_block
        leave
        ret

copy_to_c_go32_info_block:
        leal operatingsystem_go32_info_block,%esi
        leal __go32_info_block,%edi
        movl $10,%ecx
        rep
        movsl
        ret

.data
        /* fpu codeword */
___fpucw:
        .long   0x1332
        /* __go32_info_block for C programs */
        .align 2
        .globl __go32_info_block
.comm   __go32_info_block,40

/*
  -- prt1_startup --
*/
.text
        .align 2
        .globl ___prt1_startup
___prt1_startup:
        pushl %ebp
        movl %esp,%ebp
        pushl %ebx
        incl ___bss_count
        movl $0,___crt0_argv
        call _set_os_trueversion
        call _setup_core_selector
        call _setup_screens
        call _setup_go32_info_block
        incl ___environ_changed
        /* call set_processor emulation */
        /* neede to avoid FPU exception if calling from anothe DPMI program */
        movl    $0xe01,%eax
        movl    $1,%ebx
        int     $0x31
        fninit             /* initialize fpu */
        push    %eax       /* Dummy for status store check */
        movl    %esp,%esi
        movw    $0x5a5a,(%esi)
        /* fwait  maybe this one is responsible of exceptions */
        fnstsw  (%esi)
        cmpb    $0,(%esi)
        jne     .Lno_387
        fldcw   ___fpucw
.Lno_387:
        popl %eax
        pushl   operatingsystem_parameter_envp
        pushl   ___crt0_argv
        pushl   ___crt0_argc
        call    _pascal_start
        pushl   %eax
/*      call _exit changed to */
        call    exit
        .align 2,0x90
/* .comm dos_argv0,4 */
        .comm ___dos_argv0,4
        .comm ___crt0_argc,4
        .comm ___crt0_argv,4
        .comm ___environ_changed,4
/* ___environ_changed: not in data because it is defined in putenv.c */
/*        .long  0 */
        .globl _exception_exit
_exception_exit:
        .long  0
        .globl _swap_in
_swap_in:
        .long  0
        .globl _swap_out
_swap_out:
        .long  0
        .global _v2prt0_exceptions_on
_v2prt0_exceptions_on:
        .long  0

// Fill null page with NOPs
// and a jmp windows_error at the end
   .globl v2prt0_windows
v2prt0_windows:
        movl $0x90909090,%eax
        xorl %edi,%edi
        movl $0x400,%ecx
   cld
        rep
        stosl
   movl $0xffB,%edi
   movb $0xe9,%al
   stosb
   movl $_fpc_windows_error-4,%eax
   subl %edi,%eax
   stosl
   ret

// Raise SIGILL with  UD2 opcode

   .globl _fpc_windows_error
_fpc_windows_error:
   cmpl $0,_exception_exit
   je   .L_error_216
   .byte 0x0f,0x0b
.L_error_216:
        pushl   $216
        call    __exit
        jmp     exit
#enif

/* this was the prt0.s from the go32v1 version */
//
// call as start(argc, argv, envp) (C-calling convention)
//
        .globl  _pascal_start
_pascal_start:
        /* %ebx doesn't contain ScreenPrimary */
        movl    operatingsystem_go32_info_block+4,%ebx
        movl    %ebx,_ScreenPrimary
        /*  core selector in %fs */
        /*  keep original fs for debuggers !!!!! (PM) */
        movw    %fs,%ax
             movw       %ax,___v2prt0_start_fs

        movw    _core_selector,%ax
        movw    %ax,%fs

// Top of frame
        movl    $0x0,%ebp
        movl    %esp,%ebx
        movl    12(%ebx),%eax
        movl    %eax,operatingsystem_parameter_envp
        movl    %eax,__environ
        movl    %eax,_environ
        movl    8(%ebx),%eax
        movl    %eax,_args
        movl    4(%ebx),%eax
        movl    %eax,_argc

        call    PASCALMAIN
        movl    $0,%eax
        /* no error if passing here */
/*      movl    $0x4c00,%eax
        int     $0x21 */

        ret

        .data

/*      .comm   operatingsystem_parameter_envp,4 */
        .globl  _ScreenPrimary
_ScreenPrimary:
        .long   0
        .globl  _argc
_argc:
        .long   0
        .globl  _args
_args:
        .long   0
        .globl  _run_mode
_run_mode:
        .word   4
        .globl  _core_selector
_core_selector:
        .word   0
        .globl ___v2prt0_start_fs
___v2prt0_start_fs:
        .word 0
         /* DJGPP CVS crt1.c code uses __environ symbol */
         /* corresponding to _environ C variable */
         /* instead of _environ symbol since commit rev 1.11 */
         /* Thu Aug 19 9:11:52 2004 UTC by peuha */
         /* Provide both here to avoid crt1.o loading. */
        .comm  __environ,4
        .comm  _environ,4


/* Here Pierre Muller added all what was in crt1.c  */
/* in assembler                              */
/* Copyright (C) 1996 DJ Delorie, see COPYING.DJ for details */
/* Copyright (C) 1995 DJ Delorie, see COPYING.DJ for details */
/* adapted to assembler for FPC by Pierre Muller             */

/* Global variables */


/* This gets incremented each time the program is started.
   Programs (such as Emacs) which dump their code to create
   a new executable, cause this to be larger than 2.  Library
   functions that cache info in static variables should check
   the value of `__bss_count' if they need to reinitialize
   the static storage.  */
        .data
        .globl  ___bss_count
___bs_count:
        .long   1

        .globl  __dos_ds
__dos_ds:
        .long   0

        .globl ___PROXY
___PROXY:
        .ascii " !proxy"
        .byte  0

        .globl ___PROXY_LEN
___PROXY_LEN:
        .long 7

        .comm __os_trueversion,2
