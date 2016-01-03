  {
     Copyright (c) 2000 Apple Computer, Inc. All rights reserved.

     @APPLE_LICENSE_HEADER_START@

     The contents of this file constitute Original Code as defined in and
     are subject to the Apple Public Source License Version 1.1 (the
     "License").  You may not use this file except in compliance with the
     License.  Please obtain a copy of the License at
     http://www.apple.com/publicsource and read it before using this file.

     This Original Code and all software distributed under the License are
     distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
     EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
     INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
     FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
     License for the specific language governing rights and limitations
     under the License.

     @APPLE_LICENSE_HEADER_END@
    }
  {
     Copyright (c) 1992, 1993 NeXT Computer, Inc.

     HISTORY

        Machine specific signal information.

     HISTORY
     25-MAR-97  Umesh Vaishampayan (umeshv@NeXT.com)
        Ported from m98k and hppa.

     13-Jan-92  Peter King (king) at NeXT Computer, Inc.
        Filled out struct sigcontext to hold all registers.
        Added regs_saved_t to specify which regs stored in the
        sigcontext are valid.

     09-Nov-92  Ben Fathi (benf) at NeXT, Inc.
        Ported to m98k.

     09-May-91  Mike DeMoney (mike) at NeXT, Inc.
        Ported to m88k.
    }

{$packrecords C}

  const
     _PPC_SIGNAL_ = 1;

  type

     sig_atomic_t = longint;
  {
     Machine-dependant flags used in sigvec call.
    }
  { Save all regs in sigcontext  }

  const
     SV_SAVE_REGS = $1000;
  {
     regs_saved_t -- Describes which registers beyond what the kernel cares
                   about are saved to and restored from this sigcontext.

     The default is REGS_SAVED_CALLER, only the caller saved registers
     are saved.  If the SV_SAVE_REGS flag was set when the signal
     handler was registered with sigvec() then all the registers will be
     saved in the sigcontext, and REGS_SAVED_ALL will be set.  The C
     library uses REGS_SAVED_NONE in order to quickly restore kernel
     state during a longjmp().
    }
  { Only kernel managed regs restored  }
  { "Caller saved" regs: rpc, a0-a7,
                                           t0-t4, at, lk0-lk1, xt1-xt20,
                                           xr0-xr1  }
  { All registers  }

  type

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  { Structure used in sigstack call. }
    tdarwin_stack_t = record
        ss_sp    : pchar;        { signal stack base }
        ss_size  : clong;         { signal stack length }
        ss_flags : cint;         { SA_DISABLE and/or SA_ONSTACK }
    end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     ppc_thread_state = record
  { Instruction address register (PC)  }
          srr0 : dword;
  { Machine state register (supervisor)  }
          srr1 : dword;
          r0 : dword;
          r1 : dword;
          r2 : dword;
          r3 : dword;
          r4 : dword;
          r5 : dword;
          r6 : dword;
          r7 : dword;
          r8 : dword;
          r9 : dword;
          r10 : dword;
          r11 : dword;
          r12 : dword;
          r13 : dword;
          r14 : dword;
          r15 : dword;
          r16 : dword;
          r17 : dword;
          r18 : dword;
          r19 : dword;
          r20 : dword;
          r21 : dword;
          r22 : dword;
          r23 : dword;
          r24 : dword;
          r25 : dword;
          r26 : dword;
          r27 : dword;
          r28 : dword;
          r29 : dword;
          r30 : dword;
          r31 : dword;
  { Condition register  }
          cr : dword;
  { User's integer exception register  }
          xer : dword;
  { Link register  }
          lr : dword;
  { Count register  }
          ctr : dword;
  { MQ register (601 only)  }
          mq : dword;
  { Vector Save Register  }
          vrsave : dword;
       end;
     ppc_thread_state_t = ppc_thread_state;

{$packrecords 4}
     ppc_thread_state64 = record
          srr0 : qword;
          srr1 : qword;
          r0 : qword;
          r1 : qword;
          r2 : qword;
          r3 : qword;
          r4 : qword;
          r5 : qword;
          r6 : qword;
          r7 : qword;
          r8 : qword;
          r9 : qword;
          r10 : qword;
          r11 : qword;
          r12 : qword;
          r13 : qword;
          r14 : qword;
          r15 : qword;
          r16 : qword;
          r17 : qword;
          r18 : qword;
          r19 : qword;
          r20 : qword;
          r21 : qword;
          r22 : qword;
          r23 : qword;
          r24 : qword;
          r25 : qword;
          r26 : qword;
          r27 : qword;
          r28 : qword;
          r29 : qword;
          r30 : qword;
          r31 : qword;
          cr : dword;
          xer : qword;
          lr : qword;
          ctr : qword;
          vrsave : dword;
       end;
     ppc_thread_state64_t = ppc_thread_state64;

{$packrecords C}

  { This structure should be double-word aligned for performance  }
  type

     ppc_float_state = record
          fpregs : array[0..31] of double;
  { fpscr is 64 bits, 32 bits of rubbish  }
          fpscr_pad : dword;
  { floating point status register  }
          fpscr : dword;
       end;
     ppc_float_state_t = ppc_float_state;

  { VRs that have been saved  }
     ppc_vector_state = record
          save_vr : array[0..31] of array[0..3] of dword;
          save_vscr : array[0..3] of dword;
          save_pad5 : array[0..3] of dword;
          save_vrvalid : dword;
          save_pad6 : array[0..6] of dword;
       end;
     ppc_vector_state_t = ppc_vector_state;

  {
     ppc_exception_state

     This structure corresponds to some additional state of the user
     registers as saved in the PCB upon kernel entry. They are only
     available if an exception is passed out of the kernel, and even
     then not all are guaranteed to be updated.

     Some padding is included in this structure which allows space for
     servers to store temporary values if need be, to maintain binary
     compatiblity.
    }

  type

     ppc_exception_state = record
  { Fault registers for coredump  }
          dar : dword;
          dsisr : dword;
  { number of powerpc exception taken  }
          exception : dword;
  { align to 16 bytes  }
          pad0 : dword;
  { space in PCB "just in case"  }
          pad1 : array[0..3] of dword;
       end;
     ppc_exception_state_t = ppc_exception_state;

{$packrecords 4}

  type

     ppc_exception_state64 = record
  { Fault registers for coredump  }
          dar : qword;
          dsisr : dword;
  { number of powerpc exception taken  }
          exception : dword;
  { space in PCB "just in case"  }
          pad1 : array[0..3] of dword;
       end;
     ppc_exception_state64_t = ppc_exception_state64;

{$packrecords C}

     mcontext_t = record
{$ifdef cpu64}
        es: ppc_exception_state64_t;
        ss: ppc_thread_state64_t;
{$else cpu64}
        es: ppc_exception_state_t;
        ss: ppc_thread_state_t;
{$endif cpu64}
        fs: ppc_float_state_t;
        vs: ppc_vector_state_t;
     end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

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
