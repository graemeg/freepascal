#
# pII-optimised MMX format converters for HERMES
# Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
#   and (c) 1999 Jonathan Matthew (jmatthew@uq.net.au)
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
#
# These routines aren't exactly pII optimised - it's just that as they
# are, theyre terrible on p5 MMXs, but less so on pIIs.  Someone needs to
# optimise them for p5 MMXs..

#BITS 32


.globl _ConvertMMXpII32_24RGB888
.globl _ConvertMMXpII32_16RGB565
.globl _ConvertMMXpII32_16BGR565
.globl _ConvertMMXpII32_16RGB555
.globl _ConvertMMXpII32_16BGR555

.extern _mmxreturn

.data

.align 8

## Constants for conversion routines

mmx32_rgb888_mask: .long 0x00ffffff,0x00ffffff

mmx32_rgb565_b: .long 0x000000f8,0x000000f8
mmx32_rgb565_g: .long 0x0000fc00,0x0000fc00
mmx32_rgb565_r: .long 0x00f80000,0x00f80000

mmx32_rgb555_rb: .long 0x00f800f8,0x00f800f8
mmx32_rgb555_g: .long 0x0000f800,0x0000f800
mmx32_rgb555_mul: .long 0x20000008,0x20000008
mmx32_bgr555_mul: .long 0x00082000,0x00082000



.text

_ConvertMMXpII32_24RGB888: 

        # set up mm6 as the mask, mm7 as zero
        movq mmx32_rgb888_mask,%mm6
        pxor %mm7,%mm7

        movl %ecx,%edx                  # save ecx
        andl $0x0fffffffc,%ecx          # clear lower two bits
        jnz _ConvertMMXpII32_24RGB888.L1
        jmp _ConvertMMXpII32_24RGB888.L2

_ConvertMMXpII32_24RGB888.L1: 

        movq (%esi),%mm0                # A R G B a r g b
        pand %mm6,%mm0                  # 0 R G B 0 r g b
        movq 8(%esi),%mm1               # A R G B a r g b
        pand %mm6,%mm1                  # 0 R G B 0 r g b

        movq %mm0,%mm2                  # 0 R G B 0 r g b
        punpckhdq %mm7,%mm2             # 0 0 0 0 0 R G B
        punpckldq %mm7,%mm0             # 0 0 0 0 0 r g b
        psllq $24,%mm2                  # 0 0 R G B 0 0 0
        por %mm2,%mm0                   # 0 0 R G B r g b

        movq %mm1,%mm3                  # 0 R G B 0 r g b
        psllq $48,%mm3                  # g b 0 0 0 0 0 0
        por %mm3,%mm0                   # g b R G B r g b

        movq %mm1,%mm4                  # 0 R G B 0 r g b
        punpckhdq %mm7,%mm4             # 0 0 0 0 0 R G B
        punpckldq %mm7,%mm1             # 0 0 0 0 0 r g b
        psrlq $16,%mm1                  # 0 0 0 R G B 0 r
        psllq $8,%mm4                   # 0 0 0 0 R G B 0
        por %mm4,%mm1                   # 0 0 0 0 R G B r

        movq %mm0,(%edi)
        addl $16,%esi
        movd %mm1,8(%edi)
        addl $12,%edi
        subl $4,%ecx
        jnz _ConvertMMXpII32_24RGB888.L1

_ConvertMMXpII32_24RGB888.L2: 
        movl %edx,%ecx
        andl $3,%ecx
        jz _ConvertMMXpII32_24RGB888.L4
_ConvertMMXpII32_24RGB888.L3: 
        movb (%esi),%al
        movb 1(%esi),%bl
        movb 2(%esi),%dl
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertMMXpII32_24RGB888.L3
_ConvertMMXpII32_24RGB888.L4: 
        jmp _mmxreturn



_ConvertMMXpII32_16RGB565: 

        # set up masks
        movq mmx32_rgb565_b,%mm5
        movq mmx32_rgb565_g,%mm6
        movq mmx32_rgb565_r,%mm7

        movl %ecx,%edx
        shrl $2,%ecx
        jnz _ConvertMMXpII32_16RGB565.L1
        jmp _ConvertMMXpII32_16RGB565.L2 # not necessary at the moment, but doesnt hurt (much)

_ConvertMMXpII32_16RGB565.L1: 
        movq (%esi),%mm0        # argb
        movq %mm0,%mm1          # argb
        pand %mm6,%mm0          # 00g0
        movq %mm1,%mm3          # argb
        pand %mm5,%mm1          # 000b
        pand %mm7,%mm3          # 0r00
        pslld $2,%mm1           # 0 0 000000bb bbb00000
        por %mm1,%mm0           # 0 0 ggggggbb bbb00000
        psrld $5,%mm0           # 0 0 00000ggg gggbbbbb

        movq 8(%esi),%mm4       # argb
        movq %mm4,%mm2          # argb
        pand %mm6,%mm4          # 00g0
        movq %mm2,%mm1          # argb
        pand %mm5,%mm2          # 000b
        pand %mm7,%mm1          # 0r00
        pslld $2,%mm2           # 0 0 000000bb bbb00000
        por %mm2,%mm4           # 0 0 ggggggbb bbb00000
        psrld $5,%mm4           # 0 0 00000ggg gggbbbbb

        packuswb %mm1,%mm3      # R 0 r 0
        packssdw %mm4,%mm0      # as above.. ish
        por %mm3,%mm0           # done.
        movq %mm0,(%edi)

        addl $16,%esi
        addl $8,%edi
        decl %ecx
        jnz _ConvertMMXpII32_16RGB565.L1

_ConvertMMXpII32_16RGB565.L2: 
        movl %edx,%ecx
        andl $3,%ecx
        jz _ConvertMMXpII32_16RGB565.L4
_ConvertMMXpII32_16RGB565.L3: 
        movb (%esi),%al
        movb 1(%esi),%bh
        movb 2(%esi),%ah
        shrb $3,%al
        andl $0x0F81F,%eax         # BYTE?
        shrl $5,%ebx
        andl $0x07E0,%ebx          # BYTE?
        addl %ebx,%eax
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx
        jnz _ConvertMMXpII32_16RGB565.L3

_ConvertMMXpII32_16RGB565.L4: 
        jmp _mmxreturn


_ConvertMMXpII32_16BGR565: 

        movq mmx32_rgb565_r,%mm5
        movq mmx32_rgb565_g,%mm6
        movq mmx32_rgb565_b,%mm7

        movl %ecx,%edx
        shrl $2,%ecx
        jnz _ConvertMMXpII32_16BGR565.L1
        jmp _ConvertMMXpII32_16BGR565.L2

_ConvertMMXpII32_16BGR565.L1: 
        movq (%esi),%mm0                # a r g b
        movq %mm0,%mm1                  # a r g b
        pand %mm6,%mm0                  # 0 0 g 0
        movq %mm1,%mm3                  # a r g b
        pand %mm5,%mm1                  # 0 r 0 0
        pand %mm7,%mm3                  # 0 0 0 b

        psllq $16,%mm3                  # 0 b 0 0
        psrld $14,%mm1                  # 0 0 000000rr rrr00000
        por %mm1,%mm0                   # 0 0 ggggggrr rrr00000
        psrld $5,%mm0                   # 0 0 00000ggg gggrrrrr

        movq 8(%esi),%mm4               # a r g b
        movq %mm4,%mm2                  # a r g b
        pand %mm6,%mm4                  # 0 0 g 0
        movq %mm2,%mm1                  # a r g b
        pand %mm5,%mm2                  # 0 r 0 0
        pand %mm7,%mm1                  # 0 0 0 b

        psllq $16,%mm1                  # 0 b 0 0
        psrld $14,%mm2                  # 0 0 000000rr rrr00000
        por %mm2,%mm4                   # 0 0 ggggggrr rrr00000
        psrld $5,%mm4                   # 0 0 00000ggg gggrrrrr

        packuswb %mm1,%mm3              # BBBBB000 00000000 bbbbb000 00000000
        packssdw %mm4,%mm0              # 00000GGG GGGRRRRR 00000GGG GGGRRRRR
        por %mm3,%mm0                   # BBBBBGGG GGGRRRRR bbbbbggg gggrrrrr
        movq %mm0,(%edi)

        addl $16,%esi
        addl $8,%edi
        decl %ecx
        jnz _ConvertMMXpII32_16BGR565.L1

_ConvertMMXpII32_16BGR565.L2: 
        andl $3,%edx
        jz _ConvertMMXpII32_16BGR565.L4
_ConvertMMXpII32_16BGR565.L3: 
        movb 2(%esi),%al
        movb 1(%esi),%bh
        movb (%esi),%ah
        shrb $3,%al
        andl $0x0F81F,%eax                 # BYTE ?
        shrl $5,%ebx
        andl $0x07E0,%ebx                  # BYTE ?
        addl %ebx,%eax
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %edx
        jnz _ConvertMMXpII32_16BGR565.L3

_ConvertMMXpII32_16BGR565.L4: 
        jmp _mmxreturn

_ConvertMMXpII32_16BGR555: 

        # the 16BGR555 converter is identical to the RGB555 one,
        # except it uses a different multiplier for the pmaddwd
        # instruction.  cool huh.

        movq mmx32_bgr555_mul,%mm7
        jmp _convert_bgr555_cheat

# This is the same as the Intel version.. they obviously went to
# much more trouble to expand/coil the loop than I did, so theirs
# would almost certainly be faster, even if only a little.
# I did rename 'mmx32_rgb555_add' to 'mmx32_rgb555_mul', which is
# (I think) a more accurate name..
_ConvertMMXpII32_16RGB555: 

        movq mmx32_rgb555_mul,%mm7
_convert_bgr555_cheat: 
        movq mmx32_rgb555_g,%mm6

        movl %ecx,%edx                     # Save ecx 

        andl $0x0fffffff8,%ecx             # clear lower three bits
        jnz _convert_bgr555_cheat.L_OK
        jmp _convert_bgr555_cheat.L2

_convert_bgr555_cheat.L_OK: 

        movq 8(%esi),%mm2

        movq (%esi),%mm0
        movq %mm2,%mm3

        pand mmx32_rgb555_rb,%mm3
        movq %mm0,%mm1

        pand mmx32_rgb555_rb,%mm1
        pmaddwd %mm7,%mm3

        pmaddwd %mm7,%mm1
        pand %mm6,%mm2

_convert_bgr555_cheat.L1: 
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
        jz _convert_bgr555_cheat.L2
        jmp _convert_bgr555_cheat.L1


_convert_bgr555_cheat.L2: 
        movl %edx,%ecx

        andl $7,%ecx
        jz _convert_bgr555_cheat.L4

_convert_bgr555_cheat.L3: 
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
        jnz _convert_bgr555_cheat.L3

_convert_bgr555_cheat.L4: 
        jmp _mmxreturn



