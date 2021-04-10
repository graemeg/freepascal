const
  {* Used by swapcontext(3). *}
  UCF_SWAPPED      = $00000001;
  _MC_FPFMT_NODEV  = $10000;     {* device not present or configured *}
  _MC_FPFMT_387    = $10001;
  _MC_FPFMT_XMM    = $10002;
  _MC_FPOWNED_NONE = $20000;     {* FP state not used *}
  _MC_FPOWNED_FPU  = $20001;     {* FP state came from FPU *}
  _MC_FPOWNED_PCB  = $20002;     {* FP state came from PCB *}

type
  plwpid_t = ^lwpid_t;
  lwpid_t = cint32;
  TLwPid = lwpid_t;
  PLwPid = ^lwpid_t;

  {$packrecords 16}
  TMCFPStateArray = record
    items: array[0..127] of cInt;
  end;
  {$packrecords C}

{$if (defined(CPUi386) or defined(CPUX86_64))}
  mcontext_t = record
    {*
     * The first 20 fields must match the definition of
     * sigcontext. So that we can support sigcontext
     * and ucontext_t at the same time.
     *}
    mc_onstack: cInt;     {* XXX - sigcontext compat. *}
    mc_gs: cInt;          {* machine state (struct trapframe) *}
    mc_fs: cInt;
    mc_es: cInt;
    mc_ds: cInt;
    mc_edi: cInt;
    mc_esi: cInt;
    mc_ebp: cInt;
    mc_isp: cInt;
    mc_ebx: cInt;
    mc_edx: cInt;
    mc_ecx: cInt;
    mc_eax: cInt;
    mc_trapno: cInt;
    mc_err: cInt;
    mc_eip: cInt;
    mc_cs: cInt;
    mc_eflags: cInt;
    mc_esp: cInt;
    mc_ss: cInt;
    mc_len: cInt;         {* sizeof(mcontext_t) *}
    mc_fpformat: cInt;
    mc_ownedfp: cInt;
    mc_spare1: array[0..0] of cInt;      {* align next field to 16 bytes *}
    mc_fpstate: TMCFPStateArray;
    mc_spare2: array[0..7] of cInt;
  end;
{$endif def x86}

{$ifdef CPUAARCH64}
  gpregs = record
    gp_x: array[0..30] of cInt; { __register_t gp_x[30]; }
    gp_lr: cInt;
    gp_sp: cInt;
    gp_elr: cInt;
    gp_spsr: cuint32;
    gp_pad: cInt;
  end;

  fpregs = record
    fp_q: array[0..64] of cInt; { __uint128_t fp_q[32] }
    fp_sr: cuint32;
    fp_cr: cuint32;
    fp_flags: cInt;
    fp_pad: cInt;
  end;

  mcontext_t = record
    mc_gpregs: gpregs;
    mc_fpregs: fpregs;
    mc_flags: cint32;
    mc_pad: cint32;
    mc_spare: array[0..8] of cInt;
  end;
{$endif cpuaarch64}

  pucontext_t = ^ucontext_t;
  ucontext_t = record  // required for kse threads
    {*
     * Keep the order of the first two fields. Also,
     * keep them the first two fields in the structure.
     * This way we can have a union with struct
     * sigcontext and ucontext_t. This allows us to
     * support them both at the same time.
     * note: the union is not defined, though.
     *}
    uc_sigmask: sigset_t;
    uc_mcontext: mcontext_t;
    uc_link: pucontext_t;
    uc_stack: stack_t;
    uc_flags: cInt;
    __spare__: array[0..3] of cInt;
  end;
