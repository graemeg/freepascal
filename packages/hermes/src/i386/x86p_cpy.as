




.globl _CopyX86p_4byte
.globl _CopyX86p_3byte
.globl _CopyX86p_2byte
.globl _CopyX86p_1byte

.extern _x86return


.text

## _Copy*
## Paramters:
##   ESI = source 
##   EDI = dest
##   ECX = amount (NOT 0!!! (the _ConvertX86 routine checks for that though))
## Destroys:
##   EAX, EBX, EDX

_CopyX86p_4byte: 

        rep
 movsl

        jmp _x86return


_CopyX86p_3byte: 

        leal (%ecx,%ecx,2),%ecx
        jmp _CopyX86p_1byte



_CopyX86p_2byte: 

        testl $3,%edi                   # Check if video memory is aligned
        jz _CopyX86p_2byte.L_ALIGNED

        movw (%esi),%ax
        addl $2,%esi

        movw %ax,(%edi)
        addl $2,%edi

        decl %ecx
        jz _CopyX86p_2byte.L3

_CopyX86p_2byte.L_ALIGNED: 

        movl %ecx,%ebx                  # Save ecx for later

        shrl %ecx
        jz _CopyX86p_2byte.L2

        rep
 movsl

_CopyX86p_2byte.L2: 
        andl $1,%ebx
        jz _CopyX86p_2byte.L3

        movw (%esi),%ax
        addl $2,%esi

        movw %ax,(%edi)
        addl $2,%edi

_CopyX86p_2byte.L3: 
        jmp _x86return



_CopyX86p_1byte: 

_CopyX86p_1byte.L_alignloop: 
        testl $3,%edi
        jz _CopyX86p_1byte.L_aligned

        movb (%esi),%al
        incl %esi

        movb %al,(%edi)
        incl %edi

        decl %ecx
        jz _CopyX86p_1byte.L4
        jmp _CopyX86p_1byte.L_alignloop

_CopyX86p_1byte.L_aligned: 
        movl %ecx,%edx

        shrl $2,%ecx
        jz _CopyX86p_1byte.L2

        rep
 movsl

_CopyX86p_1byte.L2: 
        movl %edx,%ecx          # Get the remaining pixels to draw

        andl $3,%ecx
        jz _CopyX86p_1byte.L4   # width was modulo 4

_CopyX86p_1byte.L3: 
        movb (%esi),%al
        incl %esi

        movb %al,(%edi)
        incl %edi

        decl %ecx
        jnz _CopyX86p_1byte.L3

_CopyX86p_1byte.L4: 
        jmp _x86return
