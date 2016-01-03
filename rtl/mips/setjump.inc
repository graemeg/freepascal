{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Jonas Maebe and David Zhang 
    
    SetJmp and LongJmp implementation for exception handling

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

procedure fpc_longjmp(var s : jmp_buf;value:longint);assembler;nostackframe;[Public,alias:'FPC_LONGJMP'];compilerproc;
  asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    lw    $31,jmp_buf.ra($a0)
    lw    $sp,jmp_buf.sp($a0)
    lw    $s0,jmp_buf.s0($a0)
    lw    $s1,jmp_buf.s1($a0)
    lw    $s2,jmp_buf.s2($a0)
    lw    $s3,jmp_buf.s3($a0)
    lw    $s4,jmp_buf.s4($a0)
    lw    $s5,jmp_buf.s5($a0)
    lw    $s6,jmp_buf.s6($a0)
    lw    $s7,jmp_buf.s7($a0)
    lw    $fp,jmp_buf.fp($a0)
    lw    $gp,jmp_buf.gp($a0)
{$if defined(FPUMIPS2) or defined(FPUMIPS3)}
    lw    $v0,jmp_buf._fcsr($a0)
    ctc1  $v0,$31
    lwc1  $f20,jmp_buf.f20($a0)
    lwc1  $f21,jmp_buf.f21($a0)
    lwc1  $f22,jmp_buf.f22($a0)
    lwc1  $f23,jmp_buf.f23($a0)
    lwc1  $f24,jmp_buf.f24($a0)
    lwc1  $f25,jmp_buf.f25($a0)
    lwc1  $f26,jmp_buf.f26($a0)
    lwc1  $f27,jmp_buf.f27($a0)
    lwc1  $f28,jmp_buf.f28($a0)
    lwc1  $f29,jmp_buf.f29($a0)
    lwc1  $f30,jmp_buf.f30($a0)
    lwc1  $f31,jmp_buf.f31($a0)
{$endif FPUMIPS2 or FPUMIPS3}
    // Put value into register $2 as in setjmp
    move  $v0,$a1
    bne   $a1,$zero,.L1
    nop
    addiu $v0,$zero,1
.L1:
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    lw $31,0($4) #PC
    lw $sp,4($4) #SP
    lw $16,8($4) #S0,$16
    lw $17,12($4)#S1,$16
    lw $18,16($4)#S2,$16
    lw $19,20($4)#S3,$16
    lw $20,24($4)#S4,$16
    lw $21,28($4)#S5,$16
    lw $22,32($4)#S6,$16
    lw $23,36($4)#S7,$16
    lw $fp,40($4)#FP
    lw $gp,44($4)#GP

    j $31
    nop
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;


function fpc_setjmp(var S:jmp_buf):longint;assembler;nostackframe;[Public,alias:'FPC_SETJMP'];compilerproc;
  asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    sw    $31,jmp_buf.ra($a0)
    sw    $sp,jmp_buf.sp($a0)
    sw    $s0,jmp_buf.s0($a0)
    sw    $s1,jmp_buf.s1($a0)
    sw    $s2,jmp_buf.s2($a0)
    sw    $s3,jmp_buf.s3($a0)
    sw    $s4,jmp_buf.s4($a0)
    sw    $s5,jmp_buf.s5($a0)
    sw    $s6,jmp_buf.s6($a0)
    sw    $s7,jmp_buf.s7($a0)
    sw    $fp,jmp_buf.fp($a0)
    sw    $gp,jmp_buf.gp($a0)
{$if defined(FPUMIPS2) or defined(FPUMIPS3)}
    cfc1  $v0,$31
    sw    $v0,jmp_buf._fcsr($a0)
    swc1  $f20,jmp_buf.f20($a0)
    swc1  $f21,jmp_buf.f21($a0)
    swc1  $f22,jmp_buf.f22($a0)
    swc1  $f23,jmp_buf.f23($a0)
    swc1  $f24,jmp_buf.f24($a0)
    swc1  $f25,jmp_buf.f25($a0)
    swc1  $f26,jmp_buf.f26($a0)
    swc1  $f27,jmp_buf.f27($a0)
    swc1  $f28,jmp_buf.f28($a0)
    swc1  $f29,jmp_buf.f29($a0)
    swc1  $f30,jmp_buf.f30($a0)
    swc1  $f31,jmp_buf.f31($a0)
{$endif FPUMIPS2 or FPUMIPS3}
    move  $v0,$zero
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    sw $31,0($4) #PC
    sw $sp,4($4) #SP
    sw $16,8($4) #S0,$16
    sw $17,12($4)#S1,$16
    sw $18,16($4)#S2,$16
    sw $19,20($4)#S3,$16
    sw $20,24($4)#S4,$16
    sw $21,28($4)#S5,$16
    sw $22,32($4)#S6,$16
    sw $23,36($4)#S7,$16
    sw $fp,40($4)#FP
    sw $gp,44($4)#GP

    move $2,$0
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;
