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

#BITS 32

.globl _ConvertMMX
.globl _mmxreturn


.text

## _ConvertMMX:  
## [ESP+8] ConverterInfo*
## --------------------------------------------------------------------------
## ConverterInfo (ebp+..)
##   0: void *s_pixels
##   4: int s_width
##   8: int s_height
##  12: int s_add
##  16: void *d_pixels
##  20: int d_width
##  24: int d_height
##  28: int d_add
##  32: void (*converter_function)() 
##  36: int32 *lookup

_ConvertMMX: 
        pushl %ebp
        movl %esp,%ebp

        movl 8(%ebp),%eax

        cmpl $0,4(%eax)
        je endconvert

        movl %eax,%ebp

        movl (%ebp),%esi
        movl 16(%ebp),%edi

y_loop: 
        movl 4(%ebp),%ecx

        jmp *32(%ebp)

_mmxreturn: 
        addl 12(%ebp),%esi
        addl 28(%ebp),%edi

        decl 8(%ebp)
        jnz y_loop


        popl %ebp

endconvert: 
        emms

        ret



