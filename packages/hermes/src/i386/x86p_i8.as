#
# x86 format converters for HERMES
# Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
# This source code is licensed under the GNU LGPL
# 
# Please refer to the file COPYING.LIB contained in the distribution for
# licensing conditions          
# 
# Some routines are (c) Glenn Fiedler (ptc@gaffer.org), used with permission
#



.globl _ConvertX86pI8_32
.globl _ConvertX86pI8_24
.globl _ConvertX86pI8_16

.extern _ConvertX86
.extern _x86return

.text


## Convert_*
## Paramters:   
##   ESI = source 
##   EDI = dest
##   ECX = amount (NOT 0!!! (the ConvertX86 routine checks for that though))
## Destroys:
##   EAX, EBX, EDX

_ConvertX86pI8_32: 

        xorl %ebx,%ebx
        movl 36(%ebp),%edx
_ConvertX86pI8_32.L1: 
        movb (%esi),%bl
        incl %esi

        movl (%edx,%ebx,4),%eax

        movl %eax,(%edi)
        addl $4,%edi

        decl %ecx
        jnz _ConvertX86pI8_32.L1

        jmp _x86return



_ConvertX86pI8_24: 
        movl 36(%ebp),%ebx

        xorl %edx,%edx

        # check short
        cmpl $32,%ecx
        ja _ConvertX86pI8_24.L3


_ConvertX86pI8_24.L1:  # short loop
        movb (%esi),%dl
        movl (%ebx,%edx,4),%eax
        movb %al,(%edi) # blue
        movb %ah,1(%edi)# green
        shrl $16,%eax
        movb %al,2(%edi)# red
        incl %esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertX86pI8_24.L1
_ConvertX86pI8_24.L2: 
        jmp _x86return

_ConvertX86pI8_24.L3:  # head
        movl %edi,%eax
        andl $0b11,%eax
        jz _ConvertX86pI8_24.L4
        movb (%esi),%dl
        movl (%ebx,%edx,4),%eax
        movb %al,(%edi) # blue
        movb %ah,1(%edi)# green
        shrl $16,%eax
        movb %al,2(%edi)# red
        incl %esi
        addl $3,%edi
        decl %ecx
        jmp _ConvertX86pI8_24.L3

_ConvertX86pI8_24.L4:  # save ebp
        pushl %ebp
        movl %ebx,%ebp

        # save count
        pushl %ecx

        # unroll 4 times
        shrl $2,%ecx

_ConvertX86pI8_24.L5: pushl %ecx        # save ecx
        movb (%esi),%dl                 # index to "A"           

        movl (%ebp,%edx,4),%eax         # eax = [xx][A2][A1][A0]
        shll $8,%eax                    # eax = [A2][A1][A0][xx]

        movb 1(%esi),%dl                # index to "B"

        movb (%ebp,%edx,4),%al          # eax = [A2][A1][A0][B0]
        rorl $8,%eax                    # eax = [B0][A2][A1][A0] (done)
        movl %eax,(%edi)

        movl (%ebp,%edx,4),%eax         # eax = [xx][B2][B1][B0]
        shll $8,%eax                    # eax = [B2][B1][B0][xx]

        movb 3(%esi),%dl                # index to "D"

        movl (%ebp,%edx,4),%ecx         # ecx = [xx][D2][D1][D0]
        shll $8,%ecx                    # ecx = [D2][D1][D0][xx]

        movb 2(%esi),%dl                # index to "C"

        movb 1(%ebp,%edx,4),%ah         # eax = [B2][B1][C1][xx]
        movb (%ebp,%edx,4),%al          # eax = [B2][B1][C1][C0]
        rorl $16,%eax                   # eax = [C1][C0][B2][B1] (done)

        movb 2(%ebp,%edx,4),%cl         # ecx = [D2][D1][D0][C2] (done)

        movl %eax,4(%edi)
        movl %ecx,8(%edi)

        addl $4,%esi
        addl $3*4,%edi

        popl %ecx                       # restore ecx

        decl %ecx
        jnz _ConvertX86pI8_24.L5

        # tail
        popl %ecx
        andl $0b11,%ecx
        jz _ConvertX86pI8_24.L7

_ConvertX86pI8_24.L6: 
        movb (%esi),%dl
        movl (%ebx,%edx,4),%eax
        movb %al,(%edi) # blue
        movb %ah,1(%edi)# green
        shrl $16,%eax
        movb %al,2(%edi)# red
        incl %esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertX86pI8_24.L6

_ConvertX86pI8_24.L7: popl %ebp
        jmp _x86return


.align 8
_ConvertX86pI8_16: 
        xorl %ebx,%ebx
        movl 36(%ebp),%edx

        testl $3,%edi
        jz _ConvertX86pI8_16.Laligned

        movb (%esi),%bl

        movl (%edx,%ebx,4),%eax
        incl %esi

        movw %ax,(%edi)
        addl $2,%edi

        decl %ecx
        jz _ConvertX86pI8_16.out

_ConvertX86pI8_16.Laligned: 
        pushl %ecx

        xorl %eax,%eax
        xorl %ebx,%ebx

        shrl %ecx
        jz _ConvertX86pI8_16.last_pixel
.align 8
_ConvertX86pI8_16.Ly: 
        movb 1(%esi),%bl
        movb (%esi),%al

        movl (%edx,%ebx,4),%ebx
        addl $2,%esi

        shll $16,%ebx
        movl (%edx,%eax,4),%eax

        orl %ebx,%eax
        xorl %ebx,%ebx

        movl %eax,(%edi)
        addl $4,%edi

        xorl %eax,%eax
        decl %ecx
        jnz _ConvertX86pI8_16.Ly

_ConvertX86pI8_16.last_pixel: 
        popl %ecx

        testb $1,%cl
        jz _ConvertX86pI8_16.out

        movb (%esi),%bl

        movl (%edx,%ebx,4),%eax
        incl %esi

        movw %ax,(%edi)
        addl $2,%edi

_ConvertX86pI8_16.out: 
        jmp _x86return






