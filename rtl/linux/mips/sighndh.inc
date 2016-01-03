{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Jonas Maebe,
    member of the Free Pascal development team.

    TSigContext

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$packrecords C}

const
  __SUNOS_MAXWIN = 31;

type
  twbuf = record
    locals : array[0..7] of longint;
    ins    : array[0..7] of longint;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* MIPS OABI32 structure 
struct sigcontext {
  unsigned int sc_regmask;
  unsigned int sc_status;
  unsigned long long sc_pc;
  unsigned long long sc_regs[32];
  unsigned long long sc_fpregs[32];
  unsigned int sc_ownedfp;
  unsigned int sc_fpc_csr;
  unsigned int sc_fpc_eir;
  unsigned int sc_used_math;
  unsigned int sc_dsp;
  unsigned long long sc_mdhi;
  unsigned long long sc_mdlo;
  unsigned long sc_hi1;
  unsigned long sc_lo1;
  unsigned long sc_hi2;
  unsigned long sc_lo2;
  unsigned long sc_hi3;
  unsigned long sc_lo3;
};
typedef struct ucontext
  {
    unsigned long int uc_flags;
    struct ucontext *uc_link;
    stack_t uc_stack;
    mcontext_t uc_mcontext;
    __sigset_t uc_sigmask;
  } ucontext_t;

 *)
  FPReg = record
   case byte of
    0 : (fp_dreg : double;);
    1 : (fp_reg : single;
         fp_pad : cint; );
  end;

  PSigContext = ^TSigContext;
  TSigContext = record
    sigc_regmask,                   
    sigc_status: cuint; 
    sigc_pc : culonglong;    
    sigc_regs : array[0..31] of culonglong;
    sigc_fpregs : array[0..31] of fpreg; 
    sigc_fpc_csr, sigc_fpc_eir : cuint;
    sigc_used_math : cuint;
    sigc_dsp : cuint;
    sigc_mdhi, sigc_mdlo : culonglong;
    sigc_hi1,sigc_lo1,
    sigc_hi2,sigc_lo2,
    sigc_hi3,sigc_lo3 : culong;
  end;

  TStack = record
    ss_sp : pointer;
    ss_size : size_t;
    ss_flags : cint;
  end;

  PUContext = ^TUContext;
  TUContext = record
    uc_flags : culong;
    uc_link : PUContext;
    uc_stack : TStack;
    uc_mcontext : TSigContext;
    uc_sigmask : TSigSet;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  PSigContext = ^TSigContext;
  TSigContext = record
    sigc_onstack,      { state to restore }
    sigc_mask,         { sigmask to restore }
    sigc_sp,           { stack pointer }
    sigc_pc,           { program counter }
    sigc_npc,          { next program counter }
    sigc_psr,          { for condition codes etc }
    sigc_g1,           { User uses these two registers }
    sigc_o0, { within the trampoline code. }
    { Now comes information regarding the users window set
      * at the time of the signal. }
    sigc_oswins : longint;       { outstanding windows }
    { stack ptrs for each regwin buf }
    sigc_spbuf  : array[0..__SUNOS_MAXWIN-1] of pchar;
    { Windows to restore after signal }
    sigc_wbuf   : array[0..__SUNOS_MAXWIN] of twbuf;
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

