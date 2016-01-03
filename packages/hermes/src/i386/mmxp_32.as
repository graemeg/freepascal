#
# MMX format converters for HERMES
# Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
# This source code is licensed under the GNU LGPL
# 
# Please refer to the file COPYING.LIB contained in the distribution for
# licensing conditions          
#
# COPYRIGHT NOTICE
# 
# This file partly contains code that is (c) Intel Corporation, specifically
# the mode detection routine, and the converter to 15 bit (8 pixel
# conversion routine from the mmx programming tutorial pages).
# 

#BITS 32


.globl _ConvertMMXp32_16RGB555

.extern _mmxreturn

.data

.align 8

mmx32_rgb555_rb: .long 0x00f800f8,0x00f800f8 # Constants for conversion routines 
mmx32_rgb555_add: .long 0x20000008,0x20000008
mmx32_rgb555_g: .long 0x0000f800,0x0000f800



.text



## Gone for now, it didnt draw correctly AND was slower than the x86 routine 
_ConvertMMXp32_16RGB565: 

        jmp _mmxreturn




_ConvertMMXp32_16RGB555: 

        movq mmx32_rgb555_add,%mm7
        movq mmx32_rgb555_g,%mm6

        movl %ecx,%edx                     # Save ecx 

        andl $0x0fffffff8,%ecx             # clear lower three bits
        jnz _ConvertMMXp32_16RGB555.L_OK
        jmp _ConvertMMXp32_16RGB555.L2

_ConvertMMXp32_16RGB555.L_OK: 

        movq 8(%esi),%mm2

        movq (%esi),%mm0
        movq %mm2,%mm3

        pand mmx32_rgb555_rb,%mm3
        movq %mm0,%mm1

        pand mmx32_rgb555_rb,%mm1
        pmaddwd %mm7,%mm3

        pmaddwd %mm7,%mm1
        pand %mm6,%mm2

_ConvertMMXp32_16RGB555.L1: 
        movq 24(%esi),%mm4
        pand %mm6,%mm0

        movq 16(%esi),%mm5
        por %mm2,%mm3

        psrld $6,%mm3
        por %mm0,%mm1

        movq %mm4,%mm0
        psrld $6,%mm1

        pand mmx32_rgb555_rb,%mm0
        packssdw %mm3,%mm1

        movq %mm5,%mm3
        pmaddwd %mm7,%mm0

        pand mmx32_rgb555_rb,%mm3
        pand %mm6,%mm4

        movq %mm1,(%edi)
        pmaddwd %mm7,%mm3

        addl $32,%esi
        por %mm0,%mm4

        pand %mm6,%mm5
        psrld $6,%mm4

        movq 8(%esi),%mm2
        por %mm3,%mm5

        movq (%esi),%mm0
        psrld $6,%mm5

        movq %mm2,%mm3
        movq %mm0,%mm1

        pand mmx32_rgb555_rb,%mm3
        packssdw %mm4,%mm5

        pand mmx32_rgb555_rb,%mm1
        pand %mm6,%mm2

        movq %mm5,8(%edi)
        pmaddwd %mm7,%mm3

        pmaddwd %mm7,%mm1
        addl $16,%edi

        subl $8,%ecx
        jz _ConvertMMXp32_16RGB555.L2
        jmp _ConvertMMXp32_16RGB555.L1


_ConvertMMXp32_16RGB555.L2: 
        movl %edx,%ecx

        andl $7,%ecx
        jz _ConvertMMXp32_16RGB555.L4

_ConvertMMXp32_16RGB555.L3: 
        movl (%esi),%ebx
        addl $4,%esi

        movl %ebx,%eax
        movl %ebx,%edx

        shrl $3,%eax
        shrl $6,%edx

        andl $0b0000000000011111,%eax
        andl $0b0000001111100000,%edx

        shrl $9,%ebx

        orl %edx,%eax

        andl $0b0111110000000000,%ebx

        orl %ebx,%eax

        movw %ax,(%edi)
        addl $2,%edi

        decl %ecx
        jnz _ConvertMMXp32_16RGB555.L3

_ConvertMMXp32_16RGB555.L4: 
        jmp _mmxreturn
