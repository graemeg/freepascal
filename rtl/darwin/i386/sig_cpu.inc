<<<<<<< HEAD
{$i ../x86/sig_x86.inc}
=======

{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}


  {
   * Copyright (c) 2000-2005 Apple Computer, Inc. All rights reserved.
   *
   * @APPLE_LICENSE_HEADER_START@
   * 
   * The contents of this file constitute Original Code as defined in and
   * are subject to the Apple Public Source License Version 1.1 (the
   * "License").  You may not use this file except in compliance with the
   * License.  Please obtain a copy of the License at
   * http://www.apple.com/publicsource and read it before using this file.
   * 
   * This Original Code and all software distributed under the License are
   * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
   * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
   * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
   * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
   * License for the specific language governing rights and limitations
   * under the License.
   * 
   * @APPLE_LICENSE_HEADER_END@
    }
  {
   * @OSF_COPYRIGHT@
    }
  { 
   * Mach Operating System
   * Copyright (c) 1991,1990,1989 Carnegie Mellon University
   * All Rights Reserved.
   * 
   * Permission to use, copy, modify and distribute this software and its
   * documentation is hereby granted, provided that both the copyright
   * notice and this permission notice appear in all copies of the
   * software, derivative works or modified versions, and any portions
   * thereof, and that both notices appear in supporting documentation.
   * 
   * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
   * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
   * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
   * 
   * Carnegie Mellon requests users of this software to return to
   * 
   *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
   *  School of Computer Science
   *  Carnegie Mellon University
   *  Pittsburgh PA 15213-3890
   * 
   * any improvements or extensions that they make and grant Carnegie Mellon
   * the rights to redistribute these changes.
    }
  {
    }
  {
   *	File:	thread_status.h
   *	Author:	Avadis Tevanian, Jr.
   *	Date:	1985
   *
   *	This file contains the structure definitions for the thread
   *	state as applied to I386 processors.
    }
  {
   *	i386_thread_state	this is the structure that is exported
   *				to user threads for use in status/mutate
   *				calls.  This structure should never
   *				change.
   *
   *	i386_float_state	exported to use threads for access to 
   *				floating point registers. Try not to 
   *				change this one, either.
   *
    }
  {     THREAD_STATE_FLAVOR_LIST 0  }

{
  const
     i386_THREAD_STATE = 1;     
     i386_FLOAT_STATE = 2;     
     i386_EXCEPTION_STATE = 3;     
     THREAD_STATE_NONE = 4;     
}
  {
   * Main thread state consists of
   * general registers, segment registers,
   * eip and eflags.
    }

  type
     i386_thread_state = record
          eax : cuint;
          ebx : cuint;
          ecx : cuint;
          edx : cuint;
          edi : cuint;
          esi : cuint;
          ebp : cuint;
          esp : cuint;
          ss : cuint;
          eflags : cuint;
          eip : cuint;
          cs : cuint;
          ds : cuint;
          es : cuint;
          fs : cuint;
          gs : cuint;
       end;
     i386_thread_state_t = i386_thread_state;
  {
   * Default segment register values.
    }

  const
     USER_CODE_SELECTOR = $0017;     
     USER_DATA_SELECTOR = $001f;     
     KERN_CODE_SELECTOR = $0008;     
     KERN_DATA_SELECTOR = $0010;     
     FP_PREC_24B = 0;     
     FP_PREC_53B = 2;     
     FP_PREC_64B = 3;     
     FP_RND_NEAR = 0;     
     FP_RND_DOWN = 1;     
     FP_RND_UP = 2;     
     FP_CHOP = 3;     

  type

     fp_control = record
          flag0 : cushort;
       end;
     fp_control_t = fp_control;

  const
     bm_fp_control_invalid = $1;
     bp_fp_control_invalid = 0;
     bm_fp_control_denorm = $2;
     bp_fp_control_denorm = 1;
     bm_fp_control_zdiv = $4;
     bp_fp_control_zdiv = 2;
     bm_fp_control_ovrfl = $8;
     bp_fp_control_ovrfl = 3;
     bm_fp_control_undfl = $10;
     bp_fp_control_undfl = 4;
     bm_fp_control_precis = $20;
     bp_fp_control_precis = 5;
     bm_fp_control_x = $C0;
     bp_fp_control_x = 6;
     bm_fp_control_pc = $300;
     bp_fp_control_pc = 8;
     bm_fp_control_rc = $C00;
     bp_fp_control_rc = 10;
     bm_fp_control_inf = $1000;
     bp_fp_control_inf = 12;
     bm_fp_control_y = $E000;
     bp_fp_control_y = 13;

(*
  function invalid(var a : fp_control) : word;
  procedure set_invalid(var a : fp_control; __invalid : word);
  function denorm(var a : fp_control) : word;
  procedure set_denorm(var a : fp_control; __denorm : word);
  function zdiv(var a : fp_control) : word;
  procedure set_zdiv(var a : fp_control; __zdiv : word);
  function ovrfl(var a : fp_control) : word;
  procedure set_ovrfl(var a : fp_control; __ovrfl : word);
  function undfl(var a : fp_control) : word;
  procedure set_undfl(var a : fp_control; __undfl : word);
  function precis(var a : fp_control) : word;
  procedure set_precis(var a : fp_control; __precis : word);
  function pc(var a : fp_control) : word;
  procedure set_pc(var a : fp_control; __pc : word);
  function rc(var a : fp_control) : word;
  procedure set_rc(var a : fp_control; __rc : word);
  function inf(var a : fp_control) : word;
  procedure set_inf(var a : fp_control; __inf : word);
*)
  {
   * Status word.
    }

  type

     fp_status = record
          flag0 : cushort;
       end;
     fp_status_t = fp_status;

  const
     bm_fp_status_invalid = $1;
     bp_fp_status_invalid = 0;
     bm_fp_status_denorm = $2;
     bp_fp_status_denorm = 1;
     bm_fp_status_zdiv = $4;
     bp_fp_status_zdiv = 2;
     bm_fp_status_ovrfl = $8;
     bp_fp_status_ovrfl = 3;
     bm_fp_status_undfl = $10;
     bp_fp_status_undfl = 4;
     bm_fp_status_precis = $20;
     bp_fp_status_precis = 5;
     bm_fp_status_stkflt = $40;
     bp_fp_status_stkflt = 6;
     bm_fp_status_errsumm = $80;
     bp_fp_status_errsumm = 7;
     bm_fp_status_c0 = $100;
     bp_fp_status_c0 = 8;
     bm_fp_status_c1 = $200;
     bp_fp_status_c1 = 9;
     bm_fp_status_c2 = $400;
     bp_fp_status_c2 = 10;
     bm_fp_status_tos = $3800;
     bp_fp_status_tos = 11;
     bm_fp_status_c3 = $4000;
     bp_fp_status_c3 = 14;
     bm_fp_status_busy = $8000;
     bp_fp_status_busy = 15;

{
  function invalid(var a : fp_status) : word;
  procedure set_invalid(var a : fp_status; __invalid : word);
  function denorm(var a : fp_status) : word;
  procedure set_denorm(var a : fp_status; __denorm : word);
  function zdiv(var a : fp_status) : word;
  procedure set_zdiv(var a : fp_status; __zdiv : word);
  function ovrfl(var a : fp_status) : word;
  procedure set_ovrfl(var a : fp_status; __ovrfl : word);
  function undfl(var a : fp_status) : word;
  procedure set_undfl(var a : fp_status; __undfl : word);
  function precis(var a : fp_status) : word;
  procedure set_precis(var a : fp_status; __precis : word);
  function stkflt(var a : fp_status) : word;
  procedure set_stkflt(var a : fp_status; __stkflt : word);
  function errsumm(var a : fp_status) : word;
  procedure set_errsumm(var a : fp_status; __errsumm : word);
  function c0(var a : fp_status) : word;
  procedure set_c0(var a : fp_status; __c0 : word);
  function c1(var a : fp_status) : word;
  procedure set_c1(var a : fp_status; __c1 : word);
  function c2(var a : fp_status) : word;
  procedure set_c2(var a : fp_status; __c2 : word);
  function tos(var a : fp_status) : word;
  procedure set_tos(var a : fp_status; __tos : word);
  function c3(var a : fp_status) : word;
  procedure set_c3(var a : fp_status; __c3 : word);
  function busy(var a : fp_status) : word;
  procedure set_busy(var a : fp_status; __busy : word);
}

  { defn of 80bit x87 FPU or MMX register   }

  type
     mmst_reg = record
          mmst_reg : array[0..9] of byte;
          mmst_rsrv : array[0..5] of byte;
       end;

  { defn of 128 bit XMM regs  }
     xmm_reg = record
          xmm_reg : array[0..15] of byte;
       end;

  { 
   * Floating point state.
    }
  { number of chars worth of data from fpu_fcw  }

  const
     FP_STATE_BYTES = 512;     
  { For legacy reasons we need to leave the hw_state as char bytes  }
  { x87 FPU control word  }
  { x87 FPU status word  }
  { x87 FPU tag word  }
  { reserved  }  { x87 FPU Opcode  }
  { x87 FPU Instruction Pointer offset  }
  { x87 FPU Instruction Pointer Selector  }
  { reserved  }
  { x87 FPU Instruction Operand(Data) Pointer offset  }
  { x87 FPU Instruction Operand(Data) Pointer Selector  }
  { reserved  }
  { MXCSR Register state  }
  { MXCSR mask  }
  { ST0/MM0    }
  { ST1/MM1   }
  { ST2/MM2   }
  { ST3/MM3   }
  { ST4/MM4   }
  { ST5/MM5   }
  { ST6/MM6   }
  { ST7/MM7   }
  { XMM 0   }
  { XMM 1   }
  { XMM 2   }
  { XMM 3   }
  { XMM 4   }
  { XMM 5   }
  { XMM 6   }
  { XMM 7   }
  { reserved  }

  type
     i386_float_state = record
          fpu_reserved : array[0..1] of cint;
          fpu_fcw : fp_control_t;
          fpu_fsw : fp_status_t;
          fpu_ftw : cuint8;
          fpu_rsrv1 : cuint8;
          fpu_fop : cuint16;
          fpu_ip : cuint32;
          fpu_cs : cuint16;
          fpu_rsrv2 : cuint16;
          fpu_dp : cuint32;
          fpu_ds : cuint16;
          fpu_rsrv3 : cuint16;
          fpu_mxcsr : cuint32;
          fpu_mxcsrmask : cuint32;
          fpu_stmm0 : mmst_reg;
          fpu_stmm1 : mmst_reg;
          fpu_stmm2 : mmst_reg;
          fpu_stmm3 : mmst_reg;
          fpu_stmm4 : mmst_reg;
          fpu_stmm5 : mmst_reg;
          fpu_stmm6 : mmst_reg;
          fpu_stmm7 : mmst_reg;
          fpu_xmm0 : xmm_reg;
          fpu_xmm1 : xmm_reg;
          fpu_xmm2 : xmm_reg;
          fpu_xmm3 : xmm_reg;
          fpu_xmm4 : xmm_reg;
          fpu_xmm5 : xmm_reg;
          fpu_xmm6 : xmm_reg;
          fpu_xmm7 : xmm_reg;
          fpu_rsrv4 : array[0..(14*16)-1] of byte;
          fpu_reserved1 : cint;
       end;

     i386_float_state_t = i386_float_state;
    {
     * Extra state that may be
     * useful to exception handlers.
      }
       i386_exception_state = record
            trapno : cuint;
            err : cuint;
            faultvaddr : cuint;
         end;

       i386_exception_state_t = i386_exception_state;


(*
  function invalid(var a : fp_control) : word;
    begin
       invalid:=(a.flag0 and bm_fp_control_invalid) shr bp_fp_control_invalid;
    end;

  procedure set_invalid(var a : fp_control; __invalid : word);
    begin
       a.flag0:=a.flag0 or ((__invalid shl bp_fp_control_invalid) and bm_fp_control_invalid);
    end;

  function denorm(var a : fp_control) : word;
    begin
       denorm:=(a.flag0 and bm_fp_control_denorm) shr bp_fp_control_denorm;
    end;

  procedure set_denorm(var a : fp_control; __denorm : word);
    begin
       a.flag0:=a.flag0 or ((__denorm shl bp_fp_control_denorm) and bm_fp_control_denorm);
    end;

  function zdiv(var a : fp_control) : word;
    begin
       zdiv:=(a.flag0 and bm_fp_control_zdiv) shr bp_fp_control_zdiv;
    end;

  procedure set_zdiv(var a : fp_control; __zdiv : word);
    begin
       a.flag0:=a.flag0 or ((__zdiv shl bp_fp_control_zdiv) and bm_fp_control_zdiv);
    end;

  function ovrfl(var a : fp_control) : word;
    begin
       ovrfl:=(a.flag0 and bm_fp_control_ovrfl) shr bp_fp_control_ovrfl;
    end;

  procedure set_ovrfl(var a : fp_control; __ovrfl : word);
    begin
       a.flag0:=a.flag0 or ((__ovrfl shl bp_fp_control_ovrfl) and bm_fp_control_ovrfl);
    end;

  function undfl(var a : fp_control) : word;
    begin
       undfl:=(a.flag0 and bm_fp_control_undfl) shr bp_fp_control_undfl;
    end;

  procedure set_undfl(var a : fp_control; __undfl : word);
    begin
       a.flag0:=a.flag0 or ((__undfl shl bp_fp_control_undfl) and bm_fp_control_undfl);
    end;

  function precis(var a : fp_control) : word;
    begin
       precis:=(a.flag0 and bm_fp_control_precis) shr bp_fp_control_precis;
    end;

  procedure set_precis(var a : fp_control; __precis : word);
    begin
       a.flag0:=a.flag0 or ((__precis shl bp_fp_control_precis) and bm_fp_control_precis);
    end;

  function x(var a : fp_control) : word;
    begin
       x:=(a.flag0 and bm_fp_control_x) shr bp_fp_control_x;
    end;

  procedure set_x(var a : fp_control; __x : word);
    begin
       a.flag0:=a.flag0 or ((__x shl bp_fp_control_x) and bm_fp_control_x);
    end;

  function pc(var a : fp_control) : word;
    begin
       pc:=(a.flag0 and bm_fp_control_pc) shr bp_fp_control_pc;
    end;

  procedure set_pc(var a : fp_control; __pc : word);
    begin
       a.flag0:=a.flag0 or ((__pc shl bp_fp_control_pc) and bm_fp_control_pc);
    end;

  function rc(var a : fp_control) : word;
    begin
       rc:=(a.flag0 and bm_fp_control_rc) shr bp_fp_control_rc;
    end;

  procedure set_rc(var a : fp_control; __rc : word);
    begin
       a.flag0:=a.flag0 or ((__rc shl bp_fp_control_rc) and bm_fp_control_rc);
    end;

  function inf(var a : fp_control) : word;
    begin
       inf:=(a.flag0 and bm_fp_control_inf) shr bp_fp_control_inf;
    end;

  procedure set_inf(var a : fp_control; __inf : word);
    begin
       a.flag0:=a.flag0 or ((__inf shl bp_fp_control_inf) and bm_fp_control_inf);
    end;

  function y(var a : fp_control) : word;
    begin
       y:=(a.flag0 and bm_fp_control_y) shr bp_fp_control_y;
    end;

  procedure set_y(var a : fp_control; __y : word);
    begin
       a.flag0:=a.flag0 or ((__y shl bp_fp_control_y) and bm_fp_control_y);
    end;

  function invalid(var a : fp_status) : word;
    begin
       invalid:=(a.flag0 and bm_fp_status_invalid) shr bp_fp_status_invalid;
    end;

  procedure set_invalid(var a : fp_status; __invalid : word);
    begin
       a.flag0:=a.flag0 or ((__invalid shl bp_fp_status_invalid) and bm_fp_status_invalid);
    end;

  function denorm(var a : fp_status) : word;
    begin
       denorm:=(a.flag0 and bm_fp_status_denorm) shr bp_fp_status_denorm;
    end;

  procedure set_denorm(var a : fp_status; __denorm : word);
    begin
       a.flag0:=a.flag0 or ((__denorm shl bp_fp_status_denorm) and bm_fp_status_denorm);
    end;

  function zdiv(var a : fp_status) : word;
    begin
       zdiv:=(a.flag0 and bm_fp_status_zdiv) shr bp_fp_status_zdiv;
    end;

  procedure set_zdiv(var a : fp_status; __zdiv : word);
    begin
       a.flag0:=a.flag0 or ((__zdiv shl bp_fp_status_zdiv) and bm_fp_status_zdiv);
    end;

  function ovrfl(var a : fp_status) : word;
    begin
       ovrfl:=(a.flag0 and bm_fp_status_ovrfl) shr bp_fp_status_ovrfl;
    end;

  procedure set_ovrfl(var a : fp_status; __ovrfl : word);
    begin
       a.flag0:=a.flag0 or ((__ovrfl shl bp_fp_status_ovrfl) and bm_fp_status_ovrfl);
    end;

  function undfl(var a : fp_status) : word;
    begin
       undfl:=(a.flag0 and bm_fp_status_undfl) shr bp_fp_status_undfl;
    end;

  procedure set_undfl(var a : fp_status; __undfl : word);
    begin
       a.flag0:=a.flag0 or ((__undfl shl bp_fp_status_undfl) and bm_fp_status_undfl);
    end;

  function precis(var a : fp_status) : word;
    begin
       precis:=(a.flag0 and bm_fp_status_precis) shr bp_fp_status_precis;
    end;

  procedure set_precis(var a : fp_status; __precis : word);
    begin
       a.flag0:=a.flag0 or ((__precis shl bp_fp_status_precis) and bm_fp_status_precis);
    end;

  function stkflt(var a : fp_status) : word;
    begin
       stkflt:=(a.flag0 and bm_fp_status_stkflt) shr bp_fp_status_stkflt;
    end;

  procedure set_stkflt(var a : fp_status; __stkflt : word);
    begin
       a.flag0:=a.flag0 or ((__stkflt shl bp_fp_status_stkflt) and bm_fp_status_stkflt);
    end;

  function errsumm(var a : fp_status) : word;
    begin
       errsumm:=(a.flag0 and bm_fp_status_errsumm) shr bp_fp_status_errsumm;
    end;

  procedure set_errsumm(var a : fp_status; __errsumm : word);
    begin
       a.flag0:=a.flag0 or ((__errsumm shl bp_fp_status_errsumm) and bm_fp_status_errsumm);
    end;

  function c0(var a : fp_status) : word;
    begin
       c0:=(a.flag0 and bm_fp_status_c0) shr bp_fp_status_c0;
    end;

  procedure set_c0(var a : fp_status; __c0 : word);
    begin
       a.flag0:=a.flag0 or ((__c0 shl bp_fp_status_c0) and bm_fp_status_c0);
    end;

  function c1(var a : fp_status) : word;
    begin
       c1:=(a.flag0 and bm_fp_status_c1) shr bp_fp_status_c1;
    end;

  procedure set_c1(var a : fp_status; __c1 : word);
    begin
       a.flag0:=a.flag0 or ((__c1 shl bp_fp_status_c1) and bm_fp_status_c1);
    end;

  function c2(var a : fp_status) : word;
    begin
       c2:=(a.flag0 and bm_fp_status_c2) shr bp_fp_status_c2;
    end;

  procedure set_c2(var a : fp_status; __c2 : word);
    begin
       a.flag0:=a.flag0 or ((__c2 shl bp_fp_status_c2) and bm_fp_status_c2);
    end;

  function tos(var a : fp_status) : word;
    begin
       tos:=(a.flag0 and bm_fp_status_tos) shr bp_fp_status_tos;
    end;

  procedure set_tos(var a : fp_status; __tos : word);
    begin
       a.flag0:=a.flag0 or ((__tos shl bp_fp_status_tos) and bm_fp_status_tos);
    end;

  function c3(var a : fp_status) : word;
    begin
       c3:=(a.flag0 and bm_fp_status_c3) shr bp_fp_status_c3;
    end;

  procedure set_c3(var a : fp_status; __c3 : word);
    begin
       a.flag0:=a.flag0 or ((__c3 shl bp_fp_status_c3) and bm_fp_status_c3);
    end;

  function busy(var a : fp_status) : word;
    begin
       busy:=(a.flag0 and bm_fp_status_busy) shr bp_fp_status_busy;
    end;

  procedure set_busy(var a : fp_status; __busy : word);
    begin
       a.flag0:=a.flag0 or ((__busy shl bp_fp_status_busy) and bm_fp_status_busy);
    end;

*)

    tdarwin_stack_t = record
        ss_sp    : pchar;        { signal stack base }
        ss_size  : clong;         { signal stack length }
        ss_flags : cint;         { SA_DISABLE and/or SA_ONSTACK }
    end;

    mcontext_t = record
      es: i386_exception_state_t;
      ts: i386_thread_state_t;
      fs: i386_float_state_t;
    end;

     psigcontext = ^sigcontextrec;
     psigcontextrec = ^sigcontextrec;
     sigcontextrec = record
        uc_onstack : cint;
        uc_sigmask : sigset_t;        { signal mask used by this context }
        uc_stack   : tdarwin_stack_t; { stack used by this context }
        uc_link    : psigcontextrec;  { pointer to resuming context }
        uc_mcsize  : size_t;          { size of the machine context passed in }
        uc_mcontext: ^mcontext_t;      { machine specific context }
     end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
