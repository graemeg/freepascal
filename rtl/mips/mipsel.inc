{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006-2007 by David Zhang

    Processor dependent implementation for the system unit for MIPS

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************
                           MIPS specific stuff
****************************************************************************}
function get_fsr : dword;assembler;nostackframe;[public, alias: 'FPC_GETFSR'];
  var
    fsr : dword;
  asm
    cfc1 $2,$31
  end;


procedure set_fsr(fsr : dword);assembler;[public, alias: 'FPC_SETFSR'];
  var
    _fsr : dword;
  asm
    ctc1 $4,$31
  end;


function get_got_z : pointer;assembler;nostackframe;[public, alias: 'FPC_GETGOT_Z'];
  asm
    move $2,$28
  end;


procedure fpc_cpuinit;
var
  tmp32: longint;
  begin
    { don't let libraries influence the FPU cw set by the host program }
    if not IsLibrary then
      begin
        { enable div by 0 and invalid operation fpu exceptions }
        { round towards zero; ieee compliant arithmetics }

        tmp32 := get_fsr();
        set_fsr((tmp32 and $fffffffc) or $00000001);
      end;
  end;


{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;nostackframe;
  asm
    lw $2,0($sp)
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer):pointer;assembler;nostackframe;
  asm
    lw $2,4($4) #movl    4(%eax),%eax
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer):pointer;assembler;nostackframe;
  asm
    lw $2,0($4)#movl    (%eax),%eax
  end;


{$define FPC_SYSTEM_HAS_SPTR}
function Sptr:Pointer;assembler;nostackframe;
  asm
    move $2,$sp
  end;


{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}
procedure Move(const source;var dest;count:longint);[public, alias: 'FPC_MOVE'];assembler;
  asm
{
  Registers:
    $7 temp. to do copying
    $8 inc/decrement
    $9/l0/l1/l2 qword move
}

  sw  $4,0($23)
  sw  $5,-4($23)
  sw  $6,-8($23)
  sw  $7,-12($23)
  sw  $8,-16($23)
  sw  $9,-20($23)
  sw  $10,-24($23)
  sw  $11,-28($23)
  sw  $12,-32($23)
  sw  $13,-36($23)
  sw  $14,-40($23)
  addiu  $23,$23,-44


    # count <= 0 ?
    ble $6,$0,.Lmoveexit
    nop

    # source = dest ?
    beq $4,$5,.Lmoveexit
    nop

    # possible overlap?
    bgt $4,$5,.Lnopossibleoverlap
    nop
    # source < dest ....
    addu $7,$6,$4
    # overlap?
    # source+count < dest ?
    blt $7,$5,.Lnopossibleoverlap
    nop

  .Lcopybackward:
    # check alignment of source and dest
    or $2,$4,$5

    # move src and dest to the end of the blocks
    # assuming 16 byte block size
    addiu $3,$6,-1
    addu $4,$4,$3
    addu $5,$5,$3

    b .Lmovebytewise
    li $3,-1

.Lnopossibleoverlap:

    # check alignment of source and dest
    or $2,$4,$5

    # everything 16 byte aligned ?
    andi $13,$2,15

    beq $13,$0,.Lmovetwordwise
    # load direction in delay slot
    li $3,16


    andi $13,$2,7
    beq $13,$0,.Lmoveqwordwise
    li $3,8

    andi $13,$2,3
    beq $13,$0,.Lmovedwordwise
    li $3,4

    andi $13,$2,1
    beq $13,$0,.Lmovewordwise
    li $3,2
    b .Lmovebytewise
    li $3,1

.Lmovetwordwise:
    srl $13,$6,4
    sll $14,$13,4
    beq $14,$0,.Lmoveqwordwise_shift
    nop

.Lmovetwordwise_loop:
    lw $9,0($4)
    lw $10,4($4)
    addiu $13,$13,-1
    lw $11,8($4)
    lw $12,12($4)
    addu $4,$4,$3
    sw $9,0($5)
    sw $10,4($5)
    sw $11,8($5)
    sw $12,12($5)
    addu $5,$5,$3
    bne $13,$0,.Lmovetwordwise_loop
    nop
    subu $6,$6,$14
    beq $6,$0,.Lmoveexit
    nop

.Lmoveqwordwise_shift:
    sra $3,$3,1

.Lmoveqwordwise:
    srl $13,$6,3
    sll $14,$13,3
    beq $14,$0,.Lmovedwordwise_shift
    nop

  .Lmoveqwordwise_loop:
    lw $9,0($4)
    lw $10,4($4)
    addiu $13,$13,-1
    addu $4,$3,$4
    sw $9,0($5)
    sw $10,4($5)
    addu $5,$3,$5
    bne $13,0,.Lmoveqwordwise_loop
    nop

    subu $6,$6,$14
    beq $6,$0,.Lmoveexit
    nop

  .Lmovedwordwise_shift:
    sra $3,$3,1

  .Lmovedwordwise:
    srl $13,$6,2
    sll $14,$13,2
    beq $14,$0,.Lmovewordwise_shift
    nop

  .Lmovedwordwise_loop:
    lw $9,0($4)
    addiu $13,$13,-1
    addu $4,$4,$3
    sw $9,0($5)
    addu $5,$5,$3
    bne $13,$0,.Lmovedwordwise_loop
    nop

    subu $6,$6,$14
    beq $6,$0,.Lmoveexit
    nop

  .Lmovewordwise_shift:
    sra $3,$3,1
  .Lmovewordwise:
    srl $13,$6,1
    sll $14,$13,1
    beq $14,$0, .Lmovebytewise_shift
    nop

  .Lmovewordwise_loop:
    lhu $9,0($4)
    addiu $13,$13,-1
    addu $4,$4,$3
    sh $9,0($5)
    addu $5,$5,$3
    bne $13,$0,.Lmovewordwise_loop
    nop

    subu $6,$6,$14
    beq $6,$0, .Lmoveexit
    nop

  .Lmovebytewise_shift:
    sra $3,$3,1
  .Lmovebytewise:
    beq $6,$0, .Lmoveexit
    nop

    lbu $9,0($4)
    addiu $6,$6,-1
    addu $4,$4,$3
    sb $9,0($5)
    addu $5,$5,$3
    bne $6,$0,.Lmovebytewise
    nop
  .Lmoveexit:

  addiu  $23,$23,44
  lw  $4,0($23)
  lw  $5,-4($23)
  lw  $6,-8($23)
  lw  $7,-12($23)
  lw  $8,-16($23)
  lw  $9,-20($23)
  lw  $10,-24($23)
  lw  $11,-28($23)
  lw  $12,-32($23)
  lw  $13,-36($23)
  lw  $14,-40($23)

  end;
{$endif FPC_SYSTEM_HAS_MOVE}

{****************************************************************************
                               Integer math
****************************************************************************}

{$define FPC_SYSTEM_HAS_ABS_LONGINT}
function abs(l:longint):longint; assembler;{$ifdef SYSTEMINLINE}inline;{$endif}nostackframe;{$ifndef INTERNCONSTINTF}[internconst:fpc_in_const_abs];{$endif}
asm
    sra $1,$4,31 #$at,$4,31
    xor $2,$4,$1 #$2,$4,$at
    sub $2,$2,$1 #$2,$2,$at
end;

var
  fpc_system_lock : longint; export name 'fpc_system_lock';


{$define FPC_SYSTEM_HAS_DECLOCKED_LONGINT}
function declocked(var l : longint) : boolean;assembler;nostackframe;
{ input:  address of l in $4                                      }
{ output: boolean indicating whether l is zero after decrementing }
asm
  sw  $4,0($23)
  sw  $5,-4($23)
  sw  $6,-8($23)
  sw  $7,-12($23)
  sw  $8,-16($23)
  sw  $9,-20($23)
  sw  $10,-24($23)
  sw  $11,-28($23)
  sw  $12,-32($23)
  sw  $13,-36($23)
  sw  $14,-40($23)
  addiu  $23,$23,-44



.Ldeclocked1:
  lui $5,%hi(fpc_system_lock)
  addiu $5,$5,%lo(fpc_system_lock)
  ll $6,0($5)
  ori $7,$6,1
  beq $7,$6,.Ldeclocked1
  nop
  sc  $7,0($5)

  beq $7,$0,.Ldeclocked1
  nop

  lw $5,0($4)
  addiu $5,$5,-1
  sw $5,0($4)
  seq $2,$5,$0


  { unlock }
  lui $5,%hi(fpc_system_lock)
  addiu $5,$5,%lo(fpc_system_lock)
  sw $0,0($5)

  addiu  $23,$23,44
  lw  $4,0($23)
  lw  $5,-4($23)
  lw  $6,-8($23)
  lw  $7,-12($23)
  lw  $8,-16($23)
  lw  $9,-20($23)
  lw  $10,-24($23)
  lw  $11,-28($23)
  lw  $12,-32($23)
  lw  $13,-36($23)
  lw  $14,-40($23)
end;


{$define FPC_SYSTEM_HAS_INCLOCKED_LONGINT}
procedure inclocked(var l : longint);assembler;nostackframe;
asm
  { usually, we shouldn't lock here so saving the stack frame for these extra intructions is
    worse the effort, especially while waiting :)
  }

  { unlock }

  sw  $4,0($23)
  sw  $5,-4($23)
  sw  $6,-8($23)
  sw  $7,-12($23)
  sw  $8,-16($23)
  sw  $9,-20($23)
  sw  $10,-24($23)
  sw  $11,-28($23)
  sw  $12,-32($23)
  sw  $13,-36($23)
  sw  $14,-40($23)
  addiu  $23,$23,-44


.Ldeclocked1:
  lui $5,%hi(fpc_system_lock)
  addiu $5,$5,%lo(fpc_system_lock)
  ll $6,0($5)
  ori $7,$6,1
  beq $7,$6,.Ldeclocked1
  nop
  sc  $7,0($5)

  beq $7,$0,.Ldeclocked1
  nop

  lw $5,0($4)
  addiu $5,$5,1
  sw $5,0($4)


  { unlock }
  lui $5,%hi(fpc_system_lock)
  addiu $5,$5,%lo(fpc_system_lock)
  sw $0,0($5)

  addiu  $23,$23,44
  lw  $4,0($23)
  lw  $5,-4($23)
  lw  $6,-8($23)
  lw  $7,-12($23)
  lw  $8,-16($23)
  lw  $9,-20($23)
  lw  $10,-24($23)
  lw  $11,-28($23)
  lw  $12,-32($23)
  lw  $13,-36($23)
  lw  $14,-40($23)

end;
