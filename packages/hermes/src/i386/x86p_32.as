#
# x86 format converters for HERMES
# Some routines Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
# This source code is licensed under the GNU LGPL
# 
# Please refer to the file COPYING.LIB contained in the distribution for
# licensing conditions          
#
# Most routines are (c) Glenn Fiedler (ptc@gaffer.org), used with permission
# 



.globl _ConvertX86p32_32BGR888
.globl _ConvertX86p32_32RGBA888
.globl _ConvertX86p32_32BGRA888
.globl _ConvertX86p32_24RGB888
.globl _ConvertX86p32_24BGR888
.globl _ConvertX86p32_16RGB565
.globl _ConvertX86p32_16BGR565
.globl _ConvertX86p32_16RGB555
.globl _ConvertX86p32_16BGR555
.globl _ConvertX86p32_8RGB332

.extern _x86return

.text


## _Convert_*
## Paramters:   
##   ESI = source 
##   EDI = dest
##   ECX = amount (NOT 0!!! (the _ConvertX86 routine checks for that though))
## Destroys:
##   EAX, EBX, EDX


_ConvertX86p32_32BGR888: 

    # check short
    cmpl $32,%ecx
    ja _ConvertX86p32_32BGR888.L3

_ConvertX86p32_32BGR888.L1:  # short loop
    movl (%esi),%edx
    bswapl %edx
    rorl $8,%edx
    movl %edx,(%edi)
    addl $4,%esi
    addl $4,%edi
    decl %ecx
    jnz _ConvertX86p32_32BGR888.L1
_ConvertX86p32_32BGR888.L2: 
    jmp _x86return

_ConvertX86p32_32BGR888.L3:  # save ebp
    pushl %ebp

    # unroll four times
    movl %ecx,%ebp
    shrl $2,%ebp

    # save count
    pushl %ecx

_ConvertX86p32_32BGR888.L4: movl (%esi),%eax
        movl 4(%esi),%ebx

        bswapl %eax

        bswapl %ebx

        rorl $8,%eax
        movl 8(%esi),%ecx

        rorl $8,%ebx
        movl 12(%esi),%edx

        bswapl %ecx

        bswapl %edx

        rorl $8,%ecx
        movl %eax,(%edi)

        rorl $8,%edx
        movl %ebx,4(%edi)

        movl %ecx,8(%edi)
        movl %edx,12(%edi)

        addl $16,%esi
        addl $16,%edi

        decl %ebp
        jnz _ConvertX86p32_32BGR888.L4

    # check tail
    popl %ecx
    andl $0b11,%ecx
    jz _ConvertX86p32_32BGR888.L6

_ConvertX86p32_32BGR888.L5:  # tail loop
    movl (%esi),%edx
    bswapl %edx
    rorl $8,%edx
    movl %edx,(%edi)
    addl $4,%esi
    addl $4,%edi
    decl %ecx
    jnz _ConvertX86p32_32BGR888.L5

_ConvertX86p32_32BGR888.L6: popl %ebp
    jmp _x86return




_ConvertX86p32_32RGBA888: 

    # check short
    cmpl $32,%ecx
    ja _ConvertX86p32_32RGBA888.L3

_ConvertX86p32_32RGBA888.L1:  # short loop
    movl (%esi),%edx
    roll $8,%edx
    movl %edx,(%edi)
    addl $4,%esi
    addl $4,%edi
    decl %ecx
    jnz _ConvertX86p32_32RGBA888.L1
_ConvertX86p32_32RGBA888.L2: 
    jmp _x86return

_ConvertX86p32_32RGBA888.L3:  # save ebp
    pushl %ebp

    # unroll four times
    movl %ecx,%ebp
    shrl $2,%ebp

    # save count
    pushl %ecx

_ConvertX86p32_32RGBA888.L4: movl (%esi),%eax
        movl 4(%esi),%ebx

        roll $8,%eax
        movl 8(%esi),%ecx

        roll $8,%ebx
        movl 12(%esi),%edx

        roll $8,%ecx
        movl %eax,(%edi)

        roll $8,%edx
        movl %ebx,4(%edi)

        movl %ecx,8(%edi)
        movl %edx,12(%edi)

        addl $16,%esi
        addl $16,%edi

        decl %ebp
        jnz _ConvertX86p32_32RGBA888.L4

    # check tail
    popl %ecx
    andl $0b11,%ecx
    jz _ConvertX86p32_32RGBA888.L6

_ConvertX86p32_32RGBA888.L5:  # tail loop
    movl (%esi),%edx
    roll $8,%edx
    movl %edx,(%edi)
    addl $4,%esi
    addl $4,%edi
    decl %ecx
    jnz _ConvertX86p32_32RGBA888.L5

_ConvertX86p32_32RGBA888.L6: popl %ebp
    jmp _x86return




_ConvertX86p32_32BGRA888: 

    # check short
    cmpl $32,%ecx
    ja _ConvertX86p32_32BGRA888.L3

_ConvertX86p32_32BGRA888.L1:  # short loop
    movl (%esi),%edx
    bswapl %edx
    movl %edx,(%edi)
    addl $4,%esi
    addl $4,%edi
    decl %ecx
    jnz _ConvertX86p32_32BGRA888.L1
_ConvertX86p32_32BGRA888.L2: 
    jmp _x86return

_ConvertX86p32_32BGRA888.L3:  # save ebp
    pushl %ebp

    # unroll four times
    movl %ecx,%ebp
    shrl $2,%ebp

    # save count
    pushl %ecx

_ConvertX86p32_32BGRA888.L4: movl (%esi),%eax
        movl 4(%esi),%ebx

        movl 8(%esi),%ecx
        movl 12(%esi),%edx

        bswapl %eax

        bswapl %ebx

        bswapl %ecx

        bswapl %edx

        movl %eax,(%edi)
        movl %ebx,4(%edi)

        movl %ecx,8(%edi)
        movl %edx,12(%edi)

        addl $16,%esi
        addl $16,%edi

        decl %ebp
        jnz _ConvertX86p32_32BGRA888.L4

    # check tail
    popl %ecx
    andl $0b11,%ecx
    jz _ConvertX86p32_32BGRA888.L6

_ConvertX86p32_32BGRA888.L5:  # tail loop
    movl (%esi),%edx
    bswapl %edx
    movl %edx,(%edi)
    addl $4,%esi
    addl $4,%edi
    decl %ecx
    jnz _ConvertX86p32_32BGRA888.L5

_ConvertX86p32_32BGRA888.L6: popl %ebp
    jmp _x86return




## 32 bit RGB 888 to 24 BIT RGB 888

_ConvertX86p32_24RGB888: 

        # check short
        cmpl $32,%ecx
        ja _ConvertX86p32_24RGB888.L3

_ConvertX86p32_24RGB888.L1:  # short loop
        movb (%esi),%al
        movb 1(%esi),%bl
        movb 2(%esi),%dl
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertX86p32_24RGB888.L1
_ConvertX86p32_24RGB888.L2: 
        jmp _x86return

_ConvertX86p32_24RGB888.L3:  #        head
        movl %edi,%edx
        andl $0b11,%edx
        jz _ConvertX86p32_24RGB888.L4
        movb (%esi),%al
        movb 1(%esi),%bl
        movb 2(%esi),%dl
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jmp _ConvertX86p32_24RGB888.L3

_ConvertX86p32_24RGB888.L4:  # unroll 4 times
        pushl %ebp
        movl %ecx,%ebp
        shrl $2,%ebp

    # save count
        pushl %ecx

_ConvertX86p32_24RGB888.L5: movl (%esi),%eax # first dword            eax = [A][R][G][B]
        movl 4(%esi),%ebx               # second dword           ebx = [a][r][g][b]

        shll $8,%eax                    #                        eax = [R][G][B][.]
        movl 12(%esi),%ecx              # third dword            ecx = [a][r][g][b]

        shll $8,%ebx                    #                        ebx = [r][g][b][.]
        movb 4(%esi),%al                #                        eax = [R][G][B][b]

        rorl $8,%eax                    #                        eax = [b][R][G][B] (done)
        movb 8+1(%esi),%bh              #                        ebx = [r][g][G][.]

        movl %eax,(%edi)
        addl $3*4,%edi

        shll $8,%ecx                    #                        ecx = [r][g][b][.]
        movb 8+0(%esi),%bl              #                        ebx = [r][g][G][B]

        roll $16,%ebx                   #                        ebx = [G][B][r][g] (done)
        movb 8+2(%esi),%cl              #                        ecx = [r][g][b][R] (done)

        movl %ebx,4-3*4(%edi)
        addl $4*4,%esi

        movl %ecx,8-3*4(%edi)
        decl %ebp

        jnz _ConvertX86p32_24RGB888.L5

    # check tail
        popl %ecx
        andl $0b11,%ecx
        jz _ConvertX86p32_24RGB888.L7

_ConvertX86p32_24RGB888.L6:  # tail loop
        movb (%esi),%al
        movb 1(%esi),%bl
        movb 2(%esi),%dl
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertX86p32_24RGB888.L6

_ConvertX86p32_24RGB888.L7: popl %ebp
        jmp _x86return




## 32 bit RGB 888 to 24 bit BGR 888

_ConvertX86p32_24BGR888: 

        # check short
        cmpl $32,%ecx
        ja _ConvertX86p32_24BGR888.L3


_ConvertX86p32_24BGR888.L1:  # short loop
        movb (%esi),%dl
        movb 1(%esi),%bl
        movb 2(%esi),%al
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertX86p32_24BGR888.L1
_ConvertX86p32_24BGR888.L2: 
        jmp _x86return

_ConvertX86p32_24BGR888.L3:  # head
        movl %edi,%edx
        andl $0b11,%edx
        jz _ConvertX86p32_24BGR888.L4
        movb (%esi),%dl
        movb 1(%esi),%bl
        movb 2(%esi),%al
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jmp _ConvertX86p32_24BGR888.L3

_ConvertX86p32_24BGR888.L4:  # unroll 4 times
        pushl %ebp
        movl %ecx,%ebp
        shrl $2,%ebp

        # save count
        pushl %ecx

_ConvertX86p32_24BGR888.L5: 
        movl (%esi),%eax                # first dword            eax = [A][R][G][B]
        movl 4(%esi),%ebx               # second dword           ebx = [a][r][g][b]

        bswapl %eax                     #                        eax = [B][G][R][A]

        bswapl %ebx                     #                        ebx = [b][g][r][a]

        movb 4+2(%esi),%al              #                        eax = [B][G][R][r] 
        movb 4+4+1(%esi),%bh            #                        ebx = [b][g][G][a]

        rorl $8,%eax                    #                        eax = [r][B][G][R] (done)
        movb 4+4+2(%esi),%bl            #                        ebx = [b][g][G][R]

        rorl $16,%ebx                   #                        ebx = [G][R][b][g] (done)
        movl %eax,(%edi)

        movl %ebx,4(%edi)
        movl 12(%esi),%ecx              # third dword            ecx = [a][r][g][b]

        bswapl %ecx                     #                        ecx = [b][g][r][a]

        movb 8(%esi),%cl                #                        ecx = [b][g][r][B] (done)
        addl $4*4,%esi

        movl %ecx,8(%edi)
        addl $3*4,%edi

        decl %ebp
        jnz _ConvertX86p32_24BGR888.L5

        # check tail
        popl %ecx
        andl $0b11,%ecx
        jz _ConvertX86p32_24BGR888.L7

_ConvertX86p32_24BGR888.L6:  # tail loop
        movb (%esi),%dl
        movb 1(%esi),%bl
        movb 2(%esi),%al
        movb %al,(%edi)
        movb %bl,1(%edi)
        movb %dl,2(%edi)
        addl $4,%esi
        addl $3,%edi
        decl %ecx
        jnz _ConvertX86p32_24BGR888.L6

_ConvertX86p32_24BGR888.L7: 
        popl %ebp
        jmp _x86return




## 32 bit RGB 888 to 16 BIT RGB 565 
.align 8
_ConvertX86p32_16RGB565: 
        # check short
        cmpl $16,%ecx
        ja _ConvertX86p32_16RGB565.L3

_ConvertX86p32_16RGB565.L1:  # short loop
        movb (%esi),%bl   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%ah  # red
        shrb $3,%ah
        andb $0b11111100,%al
        shll $3,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx
        jnz _ConvertX86p32_16RGB565.L1

_ConvertX86p32_16RGB565.L2:     # End of short loop
        jmp _x86return


_ConvertX86p32_16RGB565.L3:  # head
        movl %edi,%ebx
        andl $0b11,%ebx
        jz _ConvertX86p32_16RGB565.L4

        movb (%esi),%bl   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%ah  # red
        shrb $3,%ah
        andb $0b11111100,%al
        shll $3,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx

_ConvertX86p32_16RGB565.L4: 
    # save count
        pushl %ecx

    # unroll twice
        shrl %ecx

    # point arrays to end
        leal (%esi,%ecx,8),%esi
        leal (%edi,%ecx,4),%edi

    # negative counter 
        negl %ecx
        jmp _ConvertX86p32_16RGB565.L6

_ConvertX86p32_16RGB565.L5: 
        movl %eax,-4(%edi,%ecx,4)
.align 8
_ConvertX86p32_16RGB565.L6: 
        movl (%esi,%ecx,8),%eax

        shrb $2,%ah
        movl 4(%esi,%ecx,8),%ebx

        shrl $3,%eax
        movl 4(%esi,%ecx,8),%edx

        shrb $2,%bh
        movb 2(%esi,%ecx,8),%dl

        shll $13,%ebx
        andl $0x000007FF,%eax

        shll $8,%edx
        andl $0x07FF0000,%ebx

        andl $0x0F800F800,%edx
        addl %ebx,%eax

        addl %edx,%eax
        incl %ecx

        jnz _ConvertX86p32_16RGB565.L5

        movl %eax,-4(%edi,%ecx,4)

    # tail
        popl %ecx
        testb $1,%cl
        jz _ConvertX86p32_16RGB565.L7

        movb (%esi),%bl   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%ah  # red
        shrb $3,%ah
        andb $0b11111100,%al
        shll $3,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi

_ConvertX86p32_16RGB565.L7: 
        jmp _x86return




## 32 bit RGB 888 to 16 BIT BGR 565 

_ConvertX86p32_16BGR565: 

        # check short
        cmpl $16,%ecx
        ja _ConvertX86p32_16BGR565.L3

_ConvertX86p32_16BGR565.L1:  # short loop
        movb (%esi),%ah   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%bl  # red
        shrb $3,%ah
        andb $0b11111100,%al
        shll $3,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx
        jnz _ConvertX86p32_16BGR565.L1
_ConvertX86p32_16BGR565.L2: 
        jmp _x86return

_ConvertX86p32_16BGR565.L3:  # head
        movl %edi,%ebx
        andl $0b11,%ebx
        jz _ConvertX86p32_16BGR565.L4
        movb (%esi),%ah   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%bl  # red
        shrb $3,%ah
        andb $0b11111100,%al
        shll $3,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx

_ConvertX86p32_16BGR565.L4:  # save count
        pushl %ecx

        # unroll twice
        shrl %ecx

        # point arrays to end
        leal (%esi,%ecx,8),%esi
        leal (%edi,%ecx,4),%edi

        # negative count
        negl %ecx
        jmp _ConvertX86p32_16BGR565.L6

_ConvertX86p32_16BGR565.L5: 
        movl %eax,-4(%edi,%ecx,4)
_ConvertX86p32_16BGR565.L6: 
        movl 4(%esi,%ecx,8),%edx

        movb 4(%esi,%ecx,8),%bh
        movb (%esi,%ecx,8),%ah

        shrb $3,%bh
        movb 1(%esi,%ecx,8),%al

        shrb $3,%ah
        movb 5(%esi,%ecx,8),%bl

        shll $3,%eax
        movb 2(%esi,%ecx,8),%dl

        shll $19,%ebx
        andl $0x0000FFE0,%eax

        shrl $3,%edx
        andl $0x0FFE00000,%ebx

        andl $0x001F001F,%edx
        addl %ebx,%eax

        addl %edx,%eax
        incl %ecx

        jnz _ConvertX86p32_16BGR565.L5

        movl %eax,-4(%edi,%ecx,4)

        # tail
        popl %ecx
        andl $1,%ecx
        jz _ConvertX86p32_16BGR565.L7
        movb (%esi),%ah   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%bl  # red
        shrb $3,%ah
        andb $0b11111100,%al
        shll $3,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi

_ConvertX86p32_16BGR565.L7: 
        jmp _x86return




## 32 BIT RGB TO 16 BIT RGB 555

_ConvertX86p32_16RGB555: 

        # check short
        cmpl $16,%ecx
        ja _ConvertX86p32_16RGB555.L3

_ConvertX86p32_16RGB555.L1:  # short loop
        movb (%esi),%bl   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%ah  # red
        shrb $3,%ah
        andb $0b11111000,%al
        shll $2,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx
        jnz _ConvertX86p32_16RGB555.L1
_ConvertX86p32_16RGB555.L2: 
        jmp _x86return

_ConvertX86p32_16RGB555.L3:  # head
        movl %edi,%ebx
        andl $0b11,%ebx
        jz _ConvertX86p32_16RGB555.L4
        movb (%esi),%bl   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%ah  # red
        shrb $3,%ah
        andb $0b11111000,%al
        shll $2,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx

_ConvertX86p32_16RGB555.L4:  # save count
        pushl %ecx

        # unroll twice
        shrl %ecx

        # point arrays to end
        leal (%esi,%ecx,8),%esi
        leal (%edi,%ecx,4),%edi

        # negative counter 
        negl %ecx
        jmp _ConvertX86p32_16RGB555.L6

_ConvertX86p32_16RGB555.L5: 
        movl %eax,-4(%edi,%ecx,4)
_ConvertX86p32_16RGB555.L6: 
        movl (%esi,%ecx,8),%eax

        shrb $3,%ah
        movl 4(%esi,%ecx,8),%ebx

        shrl $3,%eax
        movl 4(%esi,%ecx,8),%edx

        shrb $3,%bh
        movb 2(%esi,%ecx,8),%dl

        shll $13,%ebx
        andl $0x000007FF,%eax

        shll $7,%edx
        andl $0x07FF0000,%ebx

        andl $0x07C007C00,%edx
        addl %ebx,%eax

        addl %edx,%eax
        incl %ecx

        jnz _ConvertX86p32_16RGB555.L5

        movl %eax,-4(%edi,%ecx,4)

        # tail
        popl %ecx
        andl $1,%ecx
        jz _ConvertX86p32_16RGB555.L7
        movb (%esi),%bl   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%ah  # red
        shrb $3,%ah
        andb $0b11111000,%al
        shll $2,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi

_ConvertX86p32_16RGB555.L7: 
        jmp _x86return




## 32 BIT RGB TO 16 BIT BGR 555

_ConvertX86p32_16BGR555: 

        # check short
        cmpl $16,%ecx
        ja _ConvertX86p32_16BGR555.L3


_ConvertX86p32_16BGR555.L1:  # short loop
        movb (%esi),%ah   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%bl  # red
        shrb $3,%ah
        andb $0b11111000,%al
        shll $2,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx
        jnz _ConvertX86p32_16BGR555.L1
_ConvertX86p32_16BGR555.L2: 
        jmp _x86return

_ConvertX86p32_16BGR555.L3:  # head
        movl %edi,%ebx
        andl $0b11,%ebx
        jz _ConvertX86p32_16BGR555.L4
        movb (%esi),%ah   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%bl  # red
        shrb $3,%ah
        andb $0b11111000,%al
        shll $2,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi
        decl %ecx

_ConvertX86p32_16BGR555.L4:  # save count
        pushl %ecx

        # unroll twice
        shrl %ecx

        # point arrays to end
        leal (%esi,%ecx,8),%esi
        leal (%edi,%ecx,4),%edi

        # negative counter 
        negl %ecx
        jmp _ConvertX86p32_16BGR555.L6

_ConvertX86p32_16BGR555.L5: 
        movl %eax,-4(%edi,%ecx,4)
_ConvertX86p32_16BGR555.L6: 
        movl 4(%esi,%ecx,8),%edx

        movb 4(%esi,%ecx,8),%bh
        movb (%esi,%ecx,8),%ah

        shrb $3,%bh
        movb 1(%esi,%ecx,8),%al

        shrb $3,%ah
        movb 5(%esi,%ecx,8),%bl

        shll $2,%eax
        movb 2(%esi,%ecx,8),%dl

        shll $18,%ebx
        andl $0x00007FE0,%eax

        shrl $3,%edx
        andl $0x07FE00000,%ebx

        andl $0x001F001F,%edx
        addl %ebx,%eax

        addl %edx,%eax
        incl %ecx

        jnz _ConvertX86p32_16BGR555.L5

        movl %eax,-4(%edi,%ecx,4)

        # tail
        popl %ecx
        andl $1,%ecx
        jz _ConvertX86p32_16BGR555.L7
        movb (%esi),%ah   # blue
        movb 1(%esi),%al  # green
        movb 2(%esi),%bl  # red
        shrb $3,%ah
        andb $0b11111000,%al
        shll $2,%eax
        shrb $3,%bl
        addb %bl,%al
        movb %al,(%edi)
        movb %ah,1(%edi)
        addl $4,%esi
        addl $2,%edi

_ConvertX86p32_16BGR555.L7: 
        jmp _x86return





## FROM 32 BIT RGB to 8 BIT RGB (rrrgggbbb)
## This routine writes FOUR pixels at once (dword) and then, if they exist
## the trailing three pixels
_ConvertX86p32_8RGB332: 


_ConvertX86p32_8RGB332.L_ALIGNED: 
        pushl %ecx

        shrl $2,%ecx            # We will draw 4 pixels at once
        jnz _ConvertX86p32_8RGB332.L1

        jmp _ConvertX86p32_8RGB332.L2 # short jump out of range :(

_ConvertX86p32_8RGB332.L1: 
        movl (%esi),%eax        # first pair of pixels
        movl 4(%esi),%edx

        shrb $6,%dl
        movl %eax,%ebx

        shrb $6,%al
        andb $0x0e0,%ah

        shrl $16,%ebx
        andb $0x0e0,%dh

        shrb $3,%ah
        andb $0x0e0,%bl

        shrb $3,%dh

        orb %bl,%al

        movl %edx,%ebx
        orb %ah,%al

        shrl $16,%ebx
        orb %dh,%dl

        andb $0x0e0,%bl

        orb %bl,%dl

        movb %dl,%ah



        movl 8(%esi),%ebx       # second pair of pixels

        movl %ebx,%edx
        andb $0x0e0,%bh

        shrb $6,%bl
        andl $0x0e00000,%edx

        shrl $16,%edx

        shrb $3,%bh

        rorl $16,%eax
        orb %dl,%bl

        movl 12(%esi),%edx
        orb %bh,%bl

        movb %bl,%al

        movl %edx,%ebx
        andb $0x0e0,%dh

        shrb $6,%dl
        andl $0x0e00000,%ebx

        shrb $3,%dh
        movb %dl,%ah

        shrl $16,%ebx
        orb %dh,%ah

        orb %bl,%ah

        roll $16,%eax
        addl $16,%esi

        movl %eax,(%edi)
        addl $4,%edi

        decl %ecx
        jz _ConvertX86p32_8RGB332.L2 # L1 out of range for short jump :(

        jmp _ConvertX86p32_8RGB332.L1
_ConvertX86p32_8RGB332.L2: 

        popl %ecx
        andl $3,%ecx            # mask out number of pixels to draw

        jz _ConvertX86p32_8RGB332.L4 # Nothing to do anymore

_ConvertX86p32_8RGB332.L3: 
        movl (%esi),%eax        # single pixel conversion for trailing pixels

        movl %eax,%ebx

        shrb $6,%al
        andb $0x0e0,%ah

        shrl $16,%ebx

        shrb $3,%ah
        andb $0x0e0,%bl

        orb %ah,%al
        orb %bl,%al

        movb %al,(%edi)

        incl %edi
        addl $4,%esi

        decl %ecx
        jnz _ConvertX86p32_8RGB332.L3

_ConvertX86p32_8RGB332.L4: 
        jmp _x86return
