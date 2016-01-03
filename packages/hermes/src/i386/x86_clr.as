#
# x86 surface clear routines for HERMES
# Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
# This source code is licensed under the GNU LGPL
# 
# Please refer to the file COPYING.LIB contained in the distribution for
# licensing conditions
#
# (04/10/99)    Modified ClearX86_8             <Mikko.Tiihonen@hut.fi>




.globl _ClearX86_32
.globl _ClearX86_24
.globl _ClearX86_16
.globl _ClearX86_8

.text

##   
## --------------------------------------------------------------------------
## HermesClearInterface (ebp+..)
##   0: char8 *dest
##   4: int32 value
##   8: unsigned int width (already checked to be >0!)
##  12: unsigned int height (already checked to be >0!)
##  16: int add

.align 8
_ClearX86_32: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl (%ebp),%edi        # destination
        movl 4(%ebp),%eax       # pixel value   

        movl 12(%ebp),%edx      # height
.align 4
_ClearX86_32.L_y: 
        movl 8(%ebp),%ecx
        rep
 stosl

        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearX86_32.L_y

        popl %ebp
        ret



_ClearX86_24: 
        ret



.align 8
_ClearX86_16: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl (%ebp),%edi        # destination
        movl 4(%ebp),%eax       # pixel value   

        movl 12(%ebp),%edx      # height
        movl %eax,%ebx

        shll $16,%eax           # Duplicate pixel value
        andl $0x0ffff,%ebx

        orl %ebx,%eax
_ClearX86_16.L_y: 
        movl 8(%ebp),%ecx

        testl $3,%edi           # Check if destination is aligned mod 4
        jz _ClearX86_16.L_aligned

        movw %ax,(%edi)         # otherwise write one pixel
        addl $2,%edi

        decl %ecx
        jz _ClearX86_16.L_endline

_ClearX86_16.L_aligned: 
        shrl %ecx

rep
 stosl

        jnc _ClearX86_16.L_endline

        movw %ax,(%edi)
        addl $2,%edi

_ClearX86_16.L_endline: 
        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearX86_16.L_y

        popl %ebp
        ret



.align 8
_ClearX86_8: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl 4(%ebp),%eax       # pixel value           
        movl 12(%ebp),%edx      # height

        movb %al,%ah
        movl (%ebp),%edi        # destination

        movl %eax,%ecx

        shll $16,%eax           # Put the byte pixel value in all four bytes
        andl $0x0ffff,%ecx      # of eax

        movl 8(%ebp),%ebx
        orl %ecx,%eax

        cmpl $5,%ebx            # removes need for extra checks later
        jbe _ClearX86_8.L_short_y

.align 4
_ClearX86_8.L_y: 
        testl $3,%edi
        jz _ClearX86_8.L_aligned

        movl %edi,%ecx
        negl %ecx
        andl $3,%ecx

        subl %ecx,%ebx

        rep
 stosb

_ClearX86_8.L_aligned: 
        movl %ebx,%ecx

        shrl $2,%ecx
        andl $3,%ebx

        rep
 stosl

        movl %ebx,%ecx
        rep
 stosb

        addl 16(%ebp),%edi

        decl %edx
        movl 8(%ebp),%ebx
        jnz _ClearX86_8.L_y

        popl %ebp
        ret

## Short loop
.align 4
_ClearX86_8.L_short_y: 
        movl %ebx,%ecx

        rep
 stosb
        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearX86_8.L_short_y

        popl %ebp
        ret

## ClearX86_8 version 2,  
## Im not sure wheather this is faster or not... 
## too many jumps could confuse the CPU branch quessing


.align 8
_ClearX86_8_2: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%ebp

        movl 4(%ebp),%eax       # pixel value           
        movl 12(%ebp),%edx      # height

        movb %al,%ah
        movl (%ebp),%edi        # destination

        movl %eax,%ecx

        shll $16,%eax           # Put the byte pixel value in all four bytes
        andl $0x0ffff,%ecx      # of eax

        movl 8(%ebp),%ebx
        orl %ecx,%eax

        cmpl $5,%ebx            # removes need for extra checks in main loop
        jbe _ClearX86_8_2.L_short_y


.align 4
_ClearX86_8_2.L_y: 
        testl $3,%edi
        jz _ClearX86_8_2.L_aligned

        movl %edi,%ecx
        negl %ecx
        andl $3,%ecx

        movb %al,(%edi)
        subl %ecx,%ebx

        incl %edi

        decl %ecx
        jz _ClearX86_8_2.L_aligned

        movb %al,(%edi)
        incl %edi
        decl %ecx
        jz _ClearX86_8_2.L_aligned

        movb %al,(%edi)
        incl %edi

_ClearX86_8_2.L_aligned: 
        movl %ebx,%ecx

        shrl $2,%ecx
        andl $3,%ebx

        rep
 stosl

        jz _ClearX86_8_2.L_endline
                # ebx

        movb %al,(%edi)
                # Write remaining (1,2 or 3) pixels
        incl %edi
        decl %ebx
        jz _ClearX86_8_2.L_endline

        movb %al,(%edi)
        incl %edi
        decl %ebx
        jz _ClearX86_8_2.L_endline

        movb %al,(%edi)
        incl %edi
        decl %ebx
        jz _ClearX86_8_2.L_endline

        movb %al,(%edi)
        incl %edi

_ClearX86_8_2.L_endline: 
        addl 16(%ebp),%edi

        decl %edx
        movl 8(%ebp),%ebx
        jnz _ClearX86_8_2.L_y

        popl %ebp
        ret

## Short loop
.align 4
_ClearX86_8_2.L_short_y: 
        movl %ebx,%ecx

        rep
 stosb
        addl 16(%ebp),%edi

        decl %edx
        jnz _ClearX86_8_2.L_short_y

        popl %ebp
        ret
