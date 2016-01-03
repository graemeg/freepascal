{$PACKRECORDS C}

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
  { Copyright (c) 1995 NeXT Computer, Inc. All Rights Reserved  }
  {
     Copyright (c) 1982, 1986, 1989, 1991, 1993
        The Regents of the University of California.  All rights reserved.
     (c) UNIX System Laboratories, Inc.
     All or some portions of this file are derived from material licensed
     to the University of California by American Telephone and Telegraph
     Co. or Unix System Laboratories, Inc. and are reproduced herein with
     the permission of UNIX System Laboratories, Inc.

     Redistribution and use in source and binary forms, with or without
     modification, are permitted provided that the following conditions
     are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. All advertising materials mentioning features or use of this software
        must display the following acknowledgement:
        This product includes software developed by the University of
        California, Berkeley and its contributors.
     4. Neither the name of the University nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.

     THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
     FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
     OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
     OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
     SUCH DAMAGE.

        @(#)signal.h    8.2 (Berkeley) 1/21/94
    }

  const
    SA_NOCLDSTOP = 8;
    SA_ONSTACK   = $001; { take signal on signal stack }
    SA_RESTART   = $002; { restart system call on signal return }
    SA_DISABLE   = $004; { disable taking signals on alternate stack }
    SA_RESETHAND = $004; { reset to SIG_DFL when taking signal }
    SA_NODEFER   = $010; { don't mask the signal we're delivering }
    SA_NOCLDWAIT = $020; { don't keep zombies around }
    SA_SIGINFO   = $040; { signal handler with SA_SIGINFO args }
    SA_USERTRAMP = $100; { SUNOS compat: Do not bounce off kernel's sigtramp }
    SA_64REGSET  = $200; { SUNOS compat: Do not bounce off kernel's sigtramp }

  { hangup  }
     SIGHUP = 1;
  { interrupt  }
     SIGINT = 2;
  { quit  }
     SIGQUIT = 3;
  { illegal instruction (not reset when caught)  }
     SIGILL = 4;
  { trace trap (not reset when caught)  }
     SIGTRAP = 5;
  { abort()  }
     SIGABRT = 6;
  { compatibility  }
     SIGIOT = SIGABRT;
  { EMT instruction  }
     SIGEMT = 7;
  { floating point exception  }
     SIGFPE = 8;
  { kill (cannot be caught or ignored)  }
     SIGKILL = 9;
  { bus error  }
     SIGBUS = 10;
  { segmentation violation  }
     SIGSEGV = 11;
  { bad argument to system call  }
     SIGSYS = 12;
  { write on a pipe with no one to read it  }
     SIGPIPE = 13;
  { alarm clock  }
     SIGALRM = 14;
  { software termination signal from kill  }
     SIGTERM = 15;
  { urgent condition on IO channel  }
     SIGURG = 16;
  { sendable stop signal not from tty  }
     SIGSTOP = 17;
  { stop signal from tty  }
     SIGTSTP = 18;
  { continue a stopped process  }
     SIGCONT = 19;
  { to parent on child stop or exit  }
     SIGCHLD = 20;
  { to readers pgrp upon background tty read  }
     SIGTTIN = 21;
  { like TTIN for output if (tp->t_local&LTOSTOP)  }
     SIGTTOU = 22;
  { input/output possible signal  }
     SIGIO = 23;
  { exceeded CPU time limit  }
     SIGXCPU = 24;
  { exceeded file size limit  }
     SIGXFSZ = 25;
  { virtual time alarm  }
     SIGVTALRM = 26;
  { profiling time alarm  }
     SIGPROF = 27;
  { window size changes  }
     SIGWINCH = 28;
  { information request  }
     SIGINFO = 29;
  { user defined signal 1  }
     SIGUSR1 = 30;
  { user defined signal 2  }
     SIGUSR2 = 31;
     SIG_DFL = 0;
     SIG_IGN = 1;
     SIG_ERR = -1;

  type
     sigset_t = array[0..0] of cuint;

  Sigval = Record
            Case Boolean OF
        { Members as suggested by Annex C of POSIX 1003.1b. }
                false : (sigval_int : cint);
                True  : (sigval_ptr : Pointer);
            End;

  TSigInfo_t = record
                si_signo,                       { signal number }
                si_errno,                       { errno association }
        {
         * Cause of signal, one of the SI_ macros or signal-specific
         * values, i.e. one of the FPE_... values for SIGFPE. This
         * value is equivalent to the second argument to an old-style
         * FreeBSD signal handler.
         }
                si_code,                        { signal code }
                si_pid          : cint;         { sending process }
                si_uid          : cuint;        { sender's ruid }
                si_status       : cint;         { exit value }
                si_addr         : Pointer;      { faulting instruction }
                si_value        : SigVal;       { signal value }
                si_band         : cuint;        { band event for SIGPOLL }
                pad             : array[0..6] of cint; { Reserved for Future Use }
               end;

    TSigset=sigset_t;
    Sigset=sigset_t;
    PSigSet = ^TSigSet;

<<<<<<< HEAD
<<<<<<< HEAD
    tdarwin_stack_t = record
        ss_sp    : pchar;        { signal stack base }
        ss_size  : clong;         { signal stack length }
        ss_flags : cint;         { SA_DISABLE and/or SA_ONSTACK }
    end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$if defined(cpupowerpc) or defined(cpupowerpc64)}
    {$include ppcgen/sig_ppc.inc}    { SigContextRec }
{$else cpupowerpc or cpupowerpc64}
{$ifdef cpui386}
    {$include i386/sig_cpu.inc}       { SigContextRec }
{$else cpui386}
{$ifdef cpux86_64}
    {$include x86_64/sig_cpu.inc}       { SigContextRec }
{$else cpux86_64}
{$ifdef cpuarm}
    {$include arm/sig_cpu.inc}
{$else cpuarm}
{$ifdef cpuaarch64}
    {$include aarch64/sig_cpu.inc}
{$else cpuaarch64}
    {$error Unsupported cpu type!}
{$endif cpuaarch64}
{$endif cpuarm}
{$endif cpux86_64}
{$endif cpui386}
{$endif cpupowerpc or cpupowerpc64}

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


    SignalHandler   = Procedure(Sig : Longint);cdecl;
    PSignalHandler  = ^SignalHandler;
    SignalRestorer  = Procedure;cdecl;
    PSignalRestorer = ^SignalRestorer;
    PSigInfo        = ^TSigInfo_t;
    SigActionHandler = procedure (Sig: cint; info : psiginfo; SigContext:PSigContext); cdecl;


    SigActionRec = record
{
      case byte of
        0: (Sh: SignalHandler; Sa_Flags: longint; Sa_Mask: SigSet);
        1: (sa_handler: TSigAction);
}
      Sa_Handler: SigActionHandler;
      Sa_Mask: sigset_t;
      Sa_Flags: cint;
   end;
    PSigActionRec = ^SigActionRec;

  {
     Flags for sigprocmask:
    }
const
  { block specified signal set  }
     SIG_BLOCK = 1;
  { unblock specified signal set  }
     SIG_UNBLOCK = 2;
  { set specified signal set  }
     SIG_SETMASK = 3;
  { type of signal function  }
  {
     Structure used in sigaltstack call.
    }

  type
     sigaltstack = record
  { signal stack base  }
          ss_sp : pchar;
  { signal stack length  }
          ss_size : clong;
  { SA_DISABLE and/or SA_ONSTACK  }
          ss_flags : cint;
       end;

  { minimum allowable stack  }

  const
     MINSIGSTKSZ = 8192;
  { recommended stack size  }
  { already defined in another BSD include file }
//     SIGSTKSZ = MINSIGSTKSZ + 32768;
  {
     4.3 compatibility:
     Signal vector "template" used in sigvec call.
    }

  type
     sigvec = record
  { signal handler  }
          sv_handler : procedure(para: cint);
  { signal mask to apply  }
          sv_mask : cint;
  { see signal options below  }
          sv_flags : cint;
       end;


  const
     SV_ONSTACK = SA_ONSTACK;
  { same bit, opposite sense  }
     SV_INTERRUPT = SA_RESTART;

{ Codes for SIGFPE }
     FPE_NOOP      =  0;  { if only I knew... }
     FPE_FLTDIV    =  1;  { floating point divide by zero }
     FPE_FLTOVF    =  2;  { floating point overflow }
     FPE_FLTUND    =  3;  { floating point underflow }
     FPE_FLTRES    =  4;  { floating point inexact result }
     FPE_FLTINV    =  5;  { invalid floating point operation }
     FPE_FLTSUB    =  6;  { subscript out of range -NOTIMP on Mac OS X 10.4.7 }
     FPE_INTDIV    =  7;  { integer divide by zero -NOTIMP on Mac OS X 10.4.7 }
     FPE_INTOVF    =  8;  { integer overflow -NOTIMP on Mac OS X 10.4.7 }
