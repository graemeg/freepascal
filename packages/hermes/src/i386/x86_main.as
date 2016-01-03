#
# x86 format converters for HERMES
# Some routines Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
# This source code is licensed under the GNU LGPL
# 
# Please refer to the file COPYING.LIB contained in the distribution for
# licensing conditions          
#



.globl _ConvertX86
.globl _ConvertX86Stretch
.globl _x86return
.globl _x86return_S

.globl _Hermes_X86_CPU


.data

cpu_flags: .long 0

.text

.equ s_pixels, 0
.equ s_width, 4
.equ s_height, 8
.equ s_add, 12
.equ d_pixels, 16
.equ d_width, 20
.equ d_height, 24
.equ d_add, 28
.equ conv_func, 32
.equ lookup, 36
.equ s_pitch, 40
.equ d_pitch, 44

## _ConvertX86:  
## [ESP+8] ConverterInfo*
## --------------------------------------------------------------------------
##
## ConvertX86Stretch 'abuses' the following info structure fields:
##      - d_pitch for the y increment
##      - s_add for the x increment
## because they're unused anyway and this is thread safe.. (it's a per
## converter handle structure)
_ConvertX86Stretch: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl s_height(%ebp),%eax
        sall $16,%eax
        cdq
        idivl d_height(%ebp)
        movl %eax,d_pitch(%ebp)

        movl s_width(%ebp),%eax
        sall $16,%eax
        cdq
        idivl d_width(%ebp)
        movl %eax,s_add(%ebp)

        movl $0,s_height(%ebp)

        movl d_pixels(%ebp),%edi
        movl s_pixels(%ebp),%esi

        movl s_add(%ebp),%edx
        movl d_width(%ebp),%ecx
        jmp *conv_func(%ebp)

.align 8
_x86return_S: 

        decl d_height(%ebp)
        jz endconvert_S

        movl s_height(%ebp),%eax
        addl d_add(%ebp),%edi

        addl d_pitch(%ebp),%eax

        movl %eax,%ebx

        shrl $16,%eax

        mull s_pitch(%ebp)
        andl $0x0ffff,%ebx

        movl %ebx,s_height(%ebp)
        addl %eax,%esi

        movl s_add(%ebp),%edx
        movl d_width(%ebp),%ecx

        jmp *conv_func(%ebp)

endconvert_S: 

        popl %ebp
        ret



_ConvertX86: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl s_pixels(%ebp),%esi
        movl d_width(%ebp),%ecx
        movl d_pixels(%ebp),%edi

        jmp *32(%ebp)

.align 8
_x86return: 
        decl s_height(%ebp)
        jz endconvert

        movl d_width(%ebp),%ecx
        addl s_add(%ebp),%esi
        addl d_add(%ebp),%edi

        jmp *32(%ebp)


endconvert: 
        popl %ebp
        ret



## Hermes_X86_CPU returns the CPUID flags in eax

_Hermes_X86_CPU: 
        pushfl
        popl %eax

        movl %eax,%ecx

        xorl $0x040000,%eax
        pushl %eax

        popfl
        pushfl

        popl %eax
        xorl %ecx,%eax
        jz _Hermes_X86_CPU.L1   # Processor is 386

        pushl %ecx
        popfl

        movl %ecx,%eax
        xorl $0x200000,%eax

        pushl %eax
        popfl
        pushfl

        popl %eax
        xorl %ecx,%eax
        je _Hermes_X86_CPU.L1

        pusha

        movl $1,%eax
        cpuid

        movl %edx,cpu_flags

        popa

        movl cpu_flags,%eax

_Hermes_X86_CPU.L1: 
        ret
