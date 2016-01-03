#
# MMX surface clear routines for HERMES
# Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
# This source code is licensed under the GNU LGPL
# 
# Please refer to the file COPYING.LIB contained in the distribution for
# licensing conditions
#


.globl _ClearMMX_32
.globl _ClearMMX_24
.globl _ClearMMX_16
.globl _ClearMMX_8

.text

##
## --------------------------------------------------------------------------
## HermesClearInterface (ebp+..)
##   0: char8 *dest
##   4: int32 value
##   8: unsigned int width (already checked to be >0!)
##  12: unsigned int height (already checked to be >0!)
##  16: int add


_ClearMMX_32: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl 4(%ebp),%eax       # pixel value   
        movd 4(%ebp),%mm0

        movl 12(%ebp),%edx      # height
        movq %mm0,%mm1

        psllq $32,%mm0
        movl (%ebp),%edi        # destination

        por %mm1,%mm0
_ClearMMX_32.L_y: 
        movl 8(%ebp),%ecx

        movl %ecx,%ebx

        shrl %ecx
        jz _ClearMMX_32.L_last

_ClearMMX_32.L_x: 
        movq %mm0,(%edi)
        addl $8,%edi

        decl %ecx
        jnz _ClearMMX_32.L_x


_ClearMMX_32.L_last: 
        testl $1,%ebx
        jz _ClearMMX_32.L_endline

        movl %eax,(%edi)
        addl $4,%edi

_ClearMMX_32.L_endline: 

        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearMMX_32.L_y

        emms

        popl %ebp
        ret



_ClearMMX_24: 
        ret



_ClearMMX_16: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl 4(%ebp),%eax       # pixel value   
        movl 4(%ebp),%ebx

        movl 12(%ebp),%edx      # height
        movl (%ebp),%edi        # destination

        shll $16,%eax           # Duplicate pixel value
        andl $0x0ffff,%ebx

        orl %ebx,%eax

        movd %eax,%mm0
        movd %eax,%mm1

        psllq $32,%mm0

        por %mm1,%mm0
_ClearMMX_16.L_y: 
        movl 8(%ebp),%ecx

        testl $3,%edi           # Check if destination is aligned mod 4
        jz _ClearMMX_16.L_aligned

        movw %ax,(%edi)         # otherwise write one pixel
        addl $2,%edi

        decl %ecx
        jz _ClearMMX_16.L_endline

_ClearMMX_16.L_aligned: 
        movl %ecx,%ebx
        shrl $2,%ecx

        jz _ClearMMX_16.L_last

_ClearMMX_16.L_x: 
        movq %mm0,(%edi)
        addl $8,%edi

        decl %ecx
        jnz _ClearMMX_16.L_x

_ClearMMX_16.L_last: 
        andl $3,%ebx
        jz _ClearMMX_16.L_endline

        movw %ax,(%edi)         # Write trailing pixels
        addl $2,%edi
        decl %ebx
        jz _ClearMMX_16.L_endline

        movw %ax,(%edi)
        addl $2,%edi
        decl %ebx
        jz _ClearMMX_16.L_endline

        movw %ax,(%edi)
        addl $2,%edi
        decl %ebx
        jnz _ClearMMX_16.L_endline

_ClearMMX_16.L_endline: 
        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearMMX_16.L_y

        emms

        popl %ebp
        ret



## Clear8_x86 isnt optimised fully yet as it seems to be a tiny bit slower
## than the C routine
_ClearMMX_8: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl 4(%ebp),%eax       # pixel value           
        movl 4(%ebp),%ebx

        movl 12(%ebp),%edx      # height
        andl $0x0ff,%ebx

        shll $8,%eax            # Put the byte pixel value in all four bytes
        movl (%ebp),%edi        # destination

        movb %bl,%al
        movb %bl,%bh

        shll $16,%eax

        movb %bh,%ah
        movb %bl,%al

        movd %eax,%mm0
        movd %eax,%mm1

        psllq $32,%mm0

        por %mm1,%mm0

_ClearMMX_8.L_y: 
        movl 8(%ebp),%ecx

        testl $3,%edi           # Align mod 4
        jz _ClearMMX_8.L_aligned

        movl %edi,%ebx

        andl $3,%ebx

        movb %al,(%edi)         # Unrolled (copy & paste), align and jump
        incl %edi               # if finished, faster than a loop...
        decl %ecx
        jz _ClearMMX_8.L_endline
        decl %ebx
        jz _ClearMMX_8.L_aligned

        movb %al,(%edi)         # Second pixel
        incl %edi
        decl %ecx
        jz _ClearMMX_8.L_endline
        decl %ebx
        jz _ClearMMX_8.L_aligned

        movb %al,(%edi)         # Third pixel
        incl %edi
        decl %ecx
        jz _ClearMMX_8.L_endline
        decl %ebx
        jz _ClearMMX_8.L_aligned

_ClearMMX_8.L_aligned: 
        movl %ecx,%ebx          # Store ecx for later

        shrl $3,%ecx            # We write 8 pixels at once
        jz _ClearMMX_8.L_last

_ClearMMX_8.L_x: 
        movq %mm0,(%edi)
        addl $8,%edi

        decl %ecx
        jnz _ClearMMX_8.L_x

_ClearMMX_8.L_last: 
        movl %ebx,%ecx          # Clean up trailing pixels

        andl $7,%ecx            # Could be up to 7 left
        jz _ClearMMX_8.L_endline

        testb $0b100,%cl        # If theres less than four jump
        jz _ClearMMX_8.L_lessthanfour

        movl %eax,(%edi)        # Otherwise write a dword
        addl $4,%edi

        subl $4,%ecx

_ClearMMX_8.L_lessthanfour: 
        rep
 stosb              # Clean up the very rest

_ClearMMX_8.L_endline: 
        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearMMX_8.L_y

        emms

        popl %ebp
        ret


