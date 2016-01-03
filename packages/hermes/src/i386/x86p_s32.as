


.globl _ConvertX86p32_16RGB565_S

.text

.extern _x86return_S

## _Convert*_S
## Paramters:   
##   ESI = source
##   EDI = dest
##   ECX = amount (NOT 0!!! (the _ConvertX86 routine checks for that though))
##   EDX = x increment
## Destroys:
##   EAX, EBX, ECX, EDX


_ConvertX86p32_16RGB565_S: 

        pushl %ebp
        pushl %edx              # increment now at [esp+4]!

        movl $0,%ebp

        pushl %ecx
        shrl %ecx
        jnz _ConvertX86p32_16RGB565_S.L_ok
        jmp _ConvertX86p32_16RGB565_S.L_final

_ConvertX86p32_16RGB565_S.L_ok: 

.align 8
_ConvertX86p32_16RGB565_S.Lx: 
        movl %ebp,%eax

        shrl $14,%eax
        addl 4(%esp),%ebp

        movl %ebp,%edx
        andl $0x0fffffffc,%eax

        shrl $14,%edx
        movl (%esi,%eax,),%ebx  # ebx = pixel one

        andl $0x0fffffffc,%edx
        andl $0x0f8fcf8,%ebx

        shrb $2,%bh
        movl (%esi,%edx,),%eax  # eax = pixel two

        andl $0x0f8fcf8,%eax
        addl 4(%esp),%ebp

        shrb $2,%ah             # eax & ebx= rrrrr000|00gggggg|bbbbb000

        movl %eax,%edx

        shrw $3,%dx
        andl $0x0f80000,%eax

        shrl $8,%eax

        orl %edx,%eax

        shll $16,%eax
        movl %ebx,%edx

        shrw $3,%dx
        andl $0x0f80000,%ebx

        shrl $8,%ebx
        orb %dh,%ah

        orb %bh,%ah
        orb %dl,%al

        movl %eax,(%edi)
        addl $4,%edi

        decl %ecx
        jnz _ConvertX86p32_16RGB565_S.Lx

_ConvertX86p32_16RGB565_S.L_final: 
        popl %ecx
        andl $1,%ecx
        jz _ConvertX86p32_16RGB565_S.L_out

        shrl $14,%ebp           # trailing pixel

        andl $0x0fffffffc,%ebp

        movl (%esi,%ebp,),%eax
        movl (%esi,%ebp,),%ebx

        shrl $8,%ebx
        andl $0x0fcf8,%eax

        shrb $2,%ah
        andl $0x0f800,%ebx

        shrl $3,%eax

        orl %ebx,%eax

        movw %ax,(%edi)
        addl $2,%edi

_ConvertX86p32_16RGB565_S.L_out: 

        popl %edx
        popl %ebp
        jmp _x86return_S


