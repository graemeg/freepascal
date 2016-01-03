/*
 * This file is part of the Free Pascal run time library.
 * Copyright (c) 2005 by Thomas Schatzl,
 * member of the Free Pascal development team.
 *
 * Startup code for normal programs, PowerPC64 version.
 *
 * See the file COPYING.FPC, included in this distribution,
 * for details about the copyright.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

.macro LOAD_64BIT_VAL ra, value
    lis       \ra,\value@highest
    ori       \ra,\ra,\value@higher
    sldi      \ra,\ra,32
    oris      \ra,\ra,\value@h
    ori       \ra,\ra,\value@l
.endm

/* create function prolog for symbol "fn" */
.macro FUNCTION_PROLOG fn
    .section  ".text"
    .align    2
    .globl    \fn
    .section  ".opd", "aw"
    .align    3
\fn:
    .quad     .\fn, .TOC.@tocbase, 0
    .previous
    .size     \fn, 24
    .type     \fn, @function
    .globl    .\fn
.\fn:
.endm

/*
 * "ptrgl" glue code for calls via pointer. This function
 * sequence loads the data from the function descriptor
 * referenced by R11 into the CTR register (function address),
 * R2 (GOT/TOC pointer), and R11 (the outer frame pointer).
 *
 * On entry, R11 must be set to point to the function descriptor.
 *
 * See also the 64-bit PowerPC ABI specification for more
 * information, chapter 3.5.11 (in v1.7).
 */
.section ".text"
.align 3
.globl .ptrgl
.ptrgl:
    ld	    0, 0(11)
    std     2, 40(1)
    mtctr   0
    ld      2, 8(11)
<<<<<<< HEAD
<<<<<<< HEAD
    ld      11, 16(11)
=======
    ld      11, 8(11)
>>>>>>> graemeg/fixes_2_2
=======
    ld      11, 8(11)
>>>>>>> origin/fixes_2_2
    bctr
.long 0
.byte 0, 12, 128, 0, 0, 0, 0, 0
.type .ptrgl, @function
.size .ptrgl, . - .ptrgl

/*
 * Function prolog/epilog helpers, which are part of the 64-bit
 * PowerPC ABI.
 *
 * See also the 64-bit PowerPC ABI specification for more
 * information, chapter 3.5.5, "Register saving and restoring
 * function" (in v1.7).
 */

/* Each _savegpr0_N routine saves the general registers from rN to r31,
 * inclusive. When the routine is called, r1 must point to the start
 * of the general register save area. R0 must contain the old LR on
 * entry.
 */
_savegpr0_14: std 14,-144(1)
_savegpr0_15: std 15,-136(1)
_savegpr0_16: std 16,-128(1)
_savegpr0_17: std 17,-120(1)
_savegpr0_18: std 18,-112(1)
_savegpr0_19: std 19,-104(1)
_savegpr0_20: std 20,-96(1)
_savegpr0_21: std 21,-88(1)
_savegpr0_22: std 22,-80(1)
_savegpr0_23: std 23,-72(1)
_savegpr0_24: std 24,-64(1)
_savegpr0_25: std 25,-56(1)
_savegpr0_26: std 26,-48(1)
_savegpr0_27: std 27,-40(1)
_savegpr0_28: std 28,-32(1)
_savegpr0_29: std 29,-24(1)
_savegpr0_30: std 30,-16(1)
_savegpr0_31:
    std 31,-8(1)
    std 0, 16(1)
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

/* Each _restgpr0_N routine restores the general registers from rN to r31,
 * inclusive. When the routine is called, r1 must point to the start
 * of the general register save area.
 */
_restgpr0_14: ld 14,-144(1)
_restgpr0_15: ld 15,-136(1)
_restgpr0_16: ld 16,-128(1)
_restgpr0_17: ld 17,-120(1)
_restgpr0_18: ld 18,-112(1)
_restgpr0_19: ld 19,-104(1)
_restgpr0_20: ld 20,-96(1)
_restgpr0_21: ld 21,-88(1)
_restgpr0_22: ld 22,-80(1)
_restgpr0_23: ld 23,-72(1)
_restgpr0_24: ld 24,-64(1)
_restgpr0_25: ld 25,-56(1)
_restgpr0_26: ld 26,-48(1)
_restgpr0_27: ld 27,-40(1)
_restgpr0_28: ld 28,-32(1)
_restgpr0_29:
    ld 0, 16(1)
    ld 29,-24(1)
    mtlr 0
    ld 30,-16(1)
    ld 31,-8(1)
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

_restgpr0_30: ld 30,-16(1)
_restgpr0_31: ld 0, 16(1)
    ld 31,-8(1)
    mtlr 0
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

/* Each _savegpr1_N routine saves the general registers from rN to r31,
 * inclusive. When the routine is called, r12
 * must point to the start of the general register save area.
 */
_savegpr1_14: std 14,-144(12)
_savegpr1_15: std 15,-136(12)
_savegpr1_16: std 16,-128(12)
_savegpr1_17: std 17,-120(12)
_savegpr1_18: std 18,-112(12)
_savegpr1_19: std 19,-104(12)
_savegpr1_20: std 20,-96(12)
_savegpr1_21: std 21,-88(12)
_savegpr1_22: std 22,-80(12)
_savegpr1_23: std 23,-72(12)
_savegpr1_24: std 24,-64(12)
_savegpr1_25: std 25,-56(12)
_savegpr1_26: std 26,-48(12)
_savegpr1_27: std 27,-40(12)
_savegpr1_28: std 28,-32(12)
_savegpr1_29: std 29,-24(12)
_savegpr1_30: std 30,-16(12)
_savegpr1_31: std 31,-8(12)
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

/* The _restgpr1_N routines restore the general registers from rN to r31.
 * When the routine is called, r12 must point to the start of the general
 * register save area.
 */
_restgpr1_14: ld 14,-144(12)
_restgpr1_15: ld 15,-136(12)
_restgpr1_16: ld 16,-128(12)
_restgpr1_17: ld 17,-120(12)
_restgpr1_18: ld 18,-112(12)
_restgpr1_19: ld 19,-104(12)
_restgpr1_20: ld 20,-96(12)
_restgpr1_21: ld 21,-88(12)
_restgpr1_22: ld 22,-80(12)
_restgpr1_23: ld 23,-72(12)
_restgpr1_24: ld 24,-64(12)
_restgpr1_25: ld 25,-56(12)
_restgpr1_26: ld 26,-48(12)
_restgpr1_27: ld 27,-40(12)
_restgpr1_28: ld 28,-32(12)
_restgpr1_29: ld 29,-24(12)
_restgpr1_30: ld 30,-16(12)
_restgpr1_31: ld 31,-8(12)
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0
/* Each _savefpr_M routine saves the floating point registers from fM to f31,
 * inclusive. When the routine is called, r1 must point to the start of the
 * floating point register save area, and r0 must contain the value of LR on
 * function entry.
 */
_savefpr_14: stfd 14,-144(1)
_savefpr_15: stfd 15,-136(1)
_savefpr_16: stfd 16,-128(1)
_savefpr_17: stfd 17,-120(1)
_savefpr_18: stfd 18,-112(1)
_savefpr_19: stfd 19,-104(1)
_savefpr_20: stfd 20,-96(1)
_savefpr_21: stfd 21,-88(1)
_savefpr_22: stfd 22,-80(1)
_savefpr_23: stfd 23,-72(1)
_savefpr_24: stfd 24,-64(1)
_savefpr_25: stfd 25,-56(1)
_savefpr_26: stfd 26,-48(1)
_savefpr_27: stfd 27,-40(1)
_savefpr_28: stfd 28,-32(1)
_savefpr_29: stfd 29,-24(1)
_savefpr_30: stfd 30,-16(1)
_savefpr_31: stfd 31,-8(1)
    std 0, 16(1)
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

/* The _restfpr_M routines restore the floating point registers from fM to f31.
 * When the routine is called, r1 must point to the start of the floating point
 * register save area.
 */
_restfpr_14: lfd 14,-144(1)
_restfpr_15: lfd 15,-136(1)
_restfpr_16: lfd 16,-128(1)
_restfpr_17: lfd 17,-120(1)
_restfpr_18: lfd 18,-112(1)
_restfpr_19: lfd 19,-104(1)
_restfpr_20: lfd 20,-96(1)
_restfpr_21: lfd 21,-88(1)
_restfpr_22: lfd 22,-80(1)
_restfpr_23: lfd 23,-72(1)
_restfpr_24: lfd 24,-64(1)
_restfpr_25: lfd 25,-56(1)
_restfpr_26: lfd 26,-48(1)
_restfpr_27: lfd 27,-40(1)
_restfpr_28: lfd 28,-32(1)
_restfpr_29:
    ld 0, 16(1)
    lfd 29,-24(1)
    mtlr 0
    lfd 30,-16(1)
    lfd 31,-8(1)
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

_restfpr_30: lfd 30,-16(1)
_restfpr_31:
    ld 0, 16(1)
    lfd 31,-8(1)
    mtlr 0
    blr
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

/* Each _savevr_M routine saves the vector registers from vM to v31, inclusive.
 * When the routine is called, r0 must point to the word just beyound the end
 * of the vector register save area. On return the value of r0 is unchanged
 * while r12 may be modified.
 */
/* commented out for now, unused
_savevr_20: addi r12,r0,-192
    stvx v20,r12,r0
_savevr_21: addi r12,r0,-176
    stvx v21,r12,r0
_savevr_22: addi r12,r0,-160
    stvx v22,r12,r0
_savevr_23: addi r12,r0,-144
    stvx v23,r12,r0
_savevr_24: addi r12,r0,-128
    stvx v24,r12,r0
_savevr_25: addi r12,r0,-112
    stvx v25,r12,r0
_savevr_26: addi r12,r0,-96
    stvx v26,r12,r0
_savevr_27: addi r12,r0,-80
    stvx v27,r12,r0
_savevr_28: addi r12,r0,-64
    stvx v28,r12,r0
_savevr_29: addi r12,r0,-48
    stvx v29,r12,r0
_savevr_30: addi r12,r0,-32
    stvx v30,r12,r0
_savevr_31: addi r12,r0,-16
    stvx v31,r12,r0
    blr
*/
/* The _restvr_M routines restore the vector registers from vM to v31. When the
 * routine is called, r0 must point to the word just beyound the end of the
 * vector register save area. On return the value of r0 is unchanged while r12
 * may be modified.
 */
/* commented out for now, unused
_restvr_20: addi r12,r0,-192
    lvx v20,r12,r0
_restvr_21: addi r12,r0,-176
    lvx v21,r12,r0
_restvr_22: addi r12,r0,-160
    lvx v22,r12,r0
_restvr_23: addi r12,r0,-144
    lvx v23,r12,r0
_restvr_24: addi r12,r0,-128
    lvx v24,r12,r0
_restvr_25: addi r12,r0,-112
    lvx v25,r12,r0
_restvr_26: addi r12,r0,-96
    lvx v26,r12,r0
_restvr_27: addi r12,r0,-80
    lvx v27,r12,r0
_restvr_28: addi r12,r0,-64
    lvx v28,r12,r0
_restvr_29: addi r12,r0,-48
    lvx v29,r12,r0
_restvr_30: addi r12,r0,-32
    lvx v30,r12,r0
_restvr_31: addi r12,r0,-16
    lvx v31,r12,r0
    blr
*/

/*
<<<<<<< HEAD
<<<<<<< HEAD
 * Main program entry point for dynamic executables.
 *
 * r7 contains the function pointer that needs to be registered for calling at exit.
 * r3/r4/r5 contain argc/argv/envp
 */
FUNCTION_PROLOG _dynamic_start
  LOAD_64BIT_VAL 11, __dl_fini
  std      7,0(11)
  LOAD_64BIT_VAL 11, PASCALMAIN
  /* set up GOT pointer from PASCALMAIN */
  ld       2,8(11)
  /* and environment pointer */
  ld      11,16(11)
  /* store argument count */
  LOAD_64BIT_VAL 10,operatingsystem_parameter_argc
  stw     3,0(10)
  /* store argument address */
  LOAD_64BIT_VAL 10,operatingsystem_parameter_argv
  std     4,0(10)
  /* store environment pointer */
  std     5,0(10)

  LOAD_64BIT_VAL 8,__stkptr
  std     1,0(8)

  bl      PASCALMAIN
  nop

  /* we should not reach here. Crash horribly */
  trap
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0

/*
 * Main program entry point for static executables
 *
 * The document "64-bit PowerPC ELF Application Binary Interface Supplement 1.9"
 * pg. 24f specifies that argc/argv/envp are passed in registers r3/r4/r5 respectively,
 * but that does not seem to be the case.
 *
 * However the stack layout and contents are the same as for other platforms, so
 * use this.
 */
FUNCTION_PROLOG _start
    mr     26,1            /* save stack pointer */
    /* Set up an initial stack frame, and clear the LR */
    clrrdi  1,1,5          /* align r1 */
    li      0,0
    stdu    1,-128(1)
    mtlr    0
    std     0,0(1)        /* r1 = pointer to NULL value */

    /* store argument count (= 0(r1) )*/
    ld      3,0(26)
    LOAD_64BIT_VAL 10,operatingsystem_parameter_argc
    stw     3,0(10)
    /* calculate argument vector address and store (= 8(r1) + 8 ) */
    addi    4,26,8
    LOAD_64BIT_VAL 10,operatingsystem_parameter_argv
    std     4,0(10)
    /* store environment pointer (= argv + (argc+1)* 8 ) */
    addi    5,3,1
    sldi    5,5,3
    add     5,4,5
    LOAD_64BIT_VAL 10, operatingsystem_parameter_envp
    std     5,0(10)

    LOAD_64BIT_VAL 8,__stkptr
    std     1,0(8)

    bl      PASCALMAIN
    nop

    /* we should not reach here. Crash horribly */
    trap

FUNCTION_PROLOG _haltproc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    mflr  0
    std   0,16(1)
    stdu  1,-144(1)

    LOAD_64BIT_VAL 11,__dl_fini
    ld    11,0(11)
    cmpdi 11,0
    beq .LNoCallDlFini

    bl .ptrgl
    ld      2,40(1)

.LNoCallDlFini:

    LOAD_64BIT_VAL 3, operatingsystem_result
    lwz     3,0(3)
    /* exit group call */
    li      0,234
    sc

    LOAD_64BIT_VAL 3, operatingsystem_result
    lwz     3,0(3)
    /* exit call */
    li      0,1
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    /* exit group call */
    LOAD_64BIT_VAL 3, operatingsystem_result
    lwz     3, 0(3)
    li      0, 234
    sc
    /* exit call */
<<<<<<< HEAD
<<<<<<< HEAD
=======
    /* exit group call */
    LOAD_64BIT_VAL 3, operatingsystem_result
    lwz     3, 0(3)
    li      0, 234
    sc
    /* exit call */
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    LOAD_64BIT_VAL 3, operatingsystem_result
    lwz     3, 0(3)
    li      0, 1
>>>>>>> graemeg/cpstrnew
    sc
    /* we should not reach here. Crash horribly */
    trap
    /* do not bother cleaning up the stack frame, we should not reach here */
.long 0
.byte 0, 12, 64, 0, 0, 0, 0, 0
=======
=======
>>>>>>> origin/fixes_2_2
 * Main program entry point label (function), called by the loader
 */
FUNCTION_PROLOG _start

    mr   26, 1            /* save stack pointer */
    /* Set up an initial stack frame, and clear the LR */
    clrrdi  1, 1, 5       /* align r1 */
    li      0, 0
    stdu    1,-128(1)
    mtlr    0
    std     0, 0(1)       /* r1 = pointer to NULL value */

    /* store argument count (= 0(r1) )*/
    ld      3, 0(26)
    LOAD_64BIT_VAL 10, operatingsystem_parameter_argc
    stw     3, 0(10)
    /* calculate argument vector address and store (= 8(r1) + 8 ) */
    addi    4, 26, 8
    LOAD_64BIT_VAL 10, operatingsystem_parameter_argv
    std     4, 0(10)
    /* store environment pointer (= argv + (argc+1)* 8 ) */
    addi    5, 3, 1
    sldi    5, 5, 3
    add     5, 4, 5
    LOAD_64BIT_VAL 10, operatingsystem_parameter_envp
    std     5, 0(10)

    LOAD_64BIT_VAL 8, __stkptr
    std     1,0(8)

    bl      .PASCALMAIN
    nop

    /* directly jump to exit procedure, not via the function pointer */
    b       ._haltproc

FUNCTION_PROLOG _haltproc
    /* exit call */
    li      0, 1
    sc
    b       ._haltproc
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    /* Define a symbol for the first piece of initialized data.  */
    .section ".data"
    .globl  __data_start
__data_start:
data_start:

<<<<<<< HEAD
<<<<<<< HEAD
    .section ".bss"

    .type __stkptr, @object
    .size __stkptr, 8
    .global __stkptr
__stkptr:
    .skip 8
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    .type __dl_fini, @object
    .size __dl_fini, 8
    .global __dl_fini
__dl_fini:
    .skip 8
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    .type operatingsystem_parameters, @object
    .size operatingsystem_parameters, 24
operatingsystem_parameters:
    .skip 3 * 8
    .global operatingsystem_parameter_argc
    .global operatingsystem_parameter_argv
    .global operatingsystem_parameter_envp
    .set operatingsystem_parameter_argc, operatingsystem_parameters+0
    .set operatingsystem_parameter_argv, operatingsystem_parameters+8
    .set operatingsystem_parameter_envp, operatingsystem_parameters+16

.section .note.GNU-stack,"",%progbits
=======
=======
>>>>>>> origin/fixes_2_2
.text
    .comm __stkptr, 8

    .comm operatingsystem_parameter_argc, 4
    .comm operatingsystem_parameter_argv, 8
    .comm operatingsystem_parameter_envp, 8

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
