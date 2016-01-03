


.globl _CopyX86p_4byte_S
.globl _CopyX86p_3byte_S
.globl _CopyX86p_2byte_S
.globl _CopyX86p_1byte_S

.extern _x86return


.text

## _Copy*
## Paramters:
##   ESI = source 
##   EDI = dest
##   ECX = amount (NOT 0!!! (the _ConvertX86 routine checks for that though))
## Destroys:
##   EAX, EBX, EDX

_CopyX86p_4byte_S: 


        jmp _x86return_S


_CopyX86p_3byte_S: 

        jmp _x86return_S



_CopyX86p_2byte_S: 

        jmp _x86return_S

_CopyX86p_1byte_S: 

        jmp _x86return_S


