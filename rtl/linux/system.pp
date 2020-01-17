{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000 by Marco van de Voort
    member of the Free Pascal development team.

    System unit for Linux.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ These things are set in the makefile, }
{ But you can override them here.}
{ If you use an aout system, set the conditional AOUT}
{ $Define AOUT}

Unit System;

{*****************************************************************************}
                                    interface
{*****************************************************************************}

{$define FPC_IS_SYSTEM}
{$define HAS_CMDLINE}
{$define USE_NOTHREADMANAGER}
{$ifdef CPUM68K}
{$define FPC_68K_SYSTEM_HAS_FPU_EXCEPTIONS}
{$endif}

{$i osdefs.inc}

{$I sysunixh.inc}

{$if defined(VER3_0) and defined(CPUX86_64)}
{$define FPC_BOOTSTRAP_INDIRECT_ENTRY}
const
  { this constant only exists during bootstrapping of the RTL with FPC 3.0.x,
    so that the whole condition doesn't need to be repeated in si_intf }
  indirect_bootstrap = true;
{$endif defined(VER3_0) and defined(CPUX86_64)}


function get_cmdline:Pchar; deprecated 'use paramstr' ;
property cmdline:Pchar read get_cmdline;

{$if defined(CPURISCV32) or defined(CPURISCV64) or defined(CPUARM) or defined(CPUM68K) or (defined(CPUSPARC) and defined(VER2_6))}
{$define FPC_LOAD_SOFTFPU}
{$endif defined(CPURISCV32) or defined(CPURISCV64) or defined(CPUARM) or defined(CPUM68K) or (defined(CPUSPARC) and defined(VER2_6))}

{$ifdef FPC_SOFT_FPUX80}
{$define FPC_SOFTFLOAT_FLOATX80}
{$define LOAD_SOFTFPU}
{$endif}

{$ifdef FPC_SOFT_FPU128}
{$define FPC_SOFTFLOAT_FLOAT128}
{$define FPC_LOAD_SOFTFPU}
{$endif}

{$ifdef FPC_LOAD_SOFTFPU}
{$define fpc_softfpu_interface}
{$i softfpu.pp}
{$undef fpc_softfpu_interface}
{$endif FPC_LOAD_SOFTFPU}


{$ifdef android}
  {$I sysandroidh.inc}
{$endif android}

{*****************************************************************************}
                                 implementation
{*****************************************************************************}

{$if defined(CPUI386) and not defined(FPC_USE_LIBC)}
var
  sysenter_supported: LongInt = 0;
{$endif}

const
  calculated_cmdline:Pchar=nil;
{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
{$define FPC_SYSTEM_HAS_OSSETUPENTRYINFORMATION}
procedure OsSetupEntryInformation(constref info: TEntryInformation); forward;
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}

{$ifdef FPC_LOAD_SOFTFPU}

{$define fpc_softfpu_implementation}
{$if defined(CPUM68K)}
{$define softfpu_compiler_mul32to64}
{$define softfpu_inline}
{$endif}
{$i softfpu.pp}
{$undef fpc_softfpu_implementation}

{ we get these functions and types from the softfpu code }
{$define FPC_SYSTEM_HAS_float64}
{$define FPC_SYSTEM_HAS_float32}
{$define FPC_SYSTEM_HAS_flag}
{$define FPC_SYSTEM_HAS_extractFloat64Frac0}
{$define FPC_SYSTEM_HAS_extractFloat64Frac1}
{$define FPC_SYSTEM_HAS_extractFloat64Exp}
{$define FPC_SYSTEM_HAS_extractFloat64Sign}
{$define FPC_SYSTEM_HAS_ExtractFloat32Frac}
{$define FPC_SYSTEM_HAS_extractFloat32Exp}
{$define FPC_SYSTEM_HAS_extractFloat32Sign}

{$endif FPC_LOAD_SOFTFPU}

{$I system.inc}

{$ifdef android}
  {$I sysandroid.inc}
{$endif android}

{*****************************************************************************
                               TLS handling
*****************************************************************************}

{$if defined(CPUARM)}
{$define INITTLS}
Function fpset_tls(p : pointer;size : SizeUInt):cint;
begin
  Result:=do_syscall(syscall_nr___ARM_NR_set_tls,TSysParam(p));
end;
{$endif defined(CPUARM)}

{$if defined(CPUI386)}
{$define INITTLS}
Function fpset_tls(p : pointer;size : SizeUInt):cint;
var
  desc : record
    entry_number : dword;
    base_addr : dword;
    limit : dword;
    flags : dword;
  end;
  selector : word;
begin
  // get descriptor from the kernel
  desc.entry_number:=$ffffffff;
  // TLS is accessed by negative offsets, only the TCB pointer is at offset 0
  desc.base_addr:=dword(p)+size-SizeOf(Pointer);
  // 4 GB, limit is given in pages
  desc.limit:=$fffff;
  // seg_32bit:1, contents:0, read_exec_only:0, limit_in_pages:1, seg_not_present:0, useable:1
  desc.flags:=%1010001;
  Result:=do_syscall(syscall_nr_set_thread_area,TSysParam(@desc));
  if Result=0 then
    begin
      selector:=desc.entry_number*8+3;
      asm
        movw selector,%gs
        movl desc.base_addr,%eax
        movl %eax,%gs:0
      end;
    end;
end;
{$endif defined(CPUI386)}

{$if defined(CPUX86_64)}
{$define INITTLS}
const
  ARCH_SET_FS = $1002;

Function fpset_tls(p : pointer;size : SizeUInt):cint;
begin
  p:=pointer(qword(p)+size-SizeOf(Pointer));
  Result:=do_syscall(syscall_nr_arch_prctl,TSysParam(ARCH_SET_FS),TSysParam(p));
  if Result=0 then
    begin
      asm
        movq p,%rax
        movq %rax,%fs:0
      end;
    end;
end;
{$endif defined(CPUX86_64)}


{$ifdef INITTLS}
{ This code initialized the TLS segment for single threaded and static programs.

  In case of multithreaded and/or dynamically linked programs it is assumed that they
  dependent anyways on glibc which initializes tls properly.

  As soon as a purely FPC dynamic loading or multithreading is implemented, the code
  needs to be extended to handle these cases as well.
}

procedure InitTLS; [public,alias:'FPC_INITTLS'];
  const
    PT_TLS = 7;
    PT_DYNAMIC = 2;

  type
{$ifdef CPU64}
    tphdr = record
      p_type,
      p_flags : dword;
      p_offset,
      p_vaddr,
      p_paddr,
      p_filesz,
      p_memsz,
      p_align : qword;
    end;
{$else CPU64}
    tphdr = record
      p_type,
      p_offset,
      p_vaddr,
      p_paddr,
      p_filesz,
      p_memsz,
      p_flags,
      p_align : dword;
    end;
{$endif CPU64}
    pphdr = ^tphdr;

  var
    phdr : pphdr;
    phnum : dword;
    i   : integer;
    tls : pointer;
    auxp : ppointer;
    found : boolean;
    size : SizeUInt;
  begin
    auxp:=ppointer(envp);
    { skip environment }
    while assigned(auxp^) do
      inc(auxp);
    inc(auxp);
    phdr:=nil;
    phnum:=0;
    { now we are at the auxillary vector }
    while assigned(auxp^) do
      begin
        case plongint(auxp)^ of
          3:
            phdr:=pphdr(ppointer(auxp+1)^);
          5:
            phnum:=pdword(auxp+1)^;
        end;
        inc(auxp,2);
      end;
    found:=false;
    size:=0;
    for i:=1 to phnum do
      begin
        case phdr^.p_type of
          PT_TLS:
            begin
              found:=true;
              inc(size,phdr^.p_memsz);
              size:=Align(size,phdr^.p_align);
            end;
          PT_DYNAMIC:
            { if the program header contains a dynamic section, the program
              is linked dynamically so the dynamic linker takes care of the
              allocation of the TLS segment.

              We cannot allocate it by ourself anyways as PT_TLS provides only
              the size of TLS data of the executable itself
             }
            exit;
        end;
        inc(phdr);
      end;
    if found then
      begin
{$ifdef CPUI386}
        { threadvars should start at a page boundary,
          add extra space for the TCB }
        size:=Align(size,4096)+sizeof(Pointer);
{$endif CPUI386}
{$ifdef CPUX86_64}
        { threadvars should start at a page boundary,
          add extra space for the TCB }
        size:=Align(size,4096)+sizeof(Pointer);
{$endif CPUX86_64}
        tls:=Fpmmap(nil,size,3,MAP_PRIVATE+MAP_ANONYMOUS,-1,0);
        fpset_tls(tls,size);
      end;
  end;
{$endif INITTLS}

{*****************************************************************************
                       Indirect Entry Point
*****************************************************************************}

{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
var
  initialstkptr : Pointer;

procedure OsSetupEntryInformation(constref info: TEntryInformation);
begin
  argc := info.OS.argc;
  argv := info.OS.argv;
  envp := info.OS.envp;
  initialstkptr := info.OS.stkptr;
  initialstklen := info.OS.stklen;
end;

{ we need two variants here because TLS must be initialized by FPC only if no libc is linked however,
  InitTLS cannot be called from the start up files because when they are run, envp is not setup yet.}
procedure SysEntry(constref info: TEntryInformation);[public,alias:'FPC_SysEntry'];
begin
  SetupEntryInformation(info);
{$ifdef cpui386}
  Set8087CW(Default8087CW);
{$endif cpui386}
  info.PascalMain();
end;

procedure SysEntry_InitTLS(constref info: TEntryInformation);[public,alias:'FPC_SysEntry_InitTLS'];
begin
  SetupEntryInformation(info);
{$ifdef INITTLS}
  InitTLS;
{$endif INITTLS}
{$ifdef cpui386}
  Set8087CW(Default8087CW);
{$endif cpui386}
  info.PascalMain();
end;

{$else}
var
{$ifndef FPC_BOOTSTRAP_INDIRECT_ENTRY}
  initialstkptr : Pointer;external name '__stkptr';
{$else FPC_BOOTSTRAP_INDIRECT_ENTRY}
  initialstkptr : Pointer; public name '__stkptr';
  operatingsystem_parameter_envp : Pointer; public name 'operatingsystem_parameter_envp';
  operatingsystem_parameter_argc : LongInt; public name 'operatingsystem_parameter_argc';
  operatingsystem_parameter_argv : Pointer; public name 'operatingsystem_parameter_argv';


{ we need two variants here because TLS must be initialized by FPC only if no libc is linked however,
  InitTLS cannot be called from the start up files because when they are run, envp is not setup yet.}
procedure SysEntry(constref info: TEntryInformation);[public,alias:'FPC_SysEntry'];
begin
  initialstkptr := info.OS.stkptr;
  operatingsystem_parameter_envp := info.OS.envp;
  operatingsystem_parameter_argc := info.OS.argc;
  operatingsystem_parameter_argv := info.OS.argv;
{$ifdef cpui386}
  Set8087CW(Default8087CW);
{$endif cpui386}
  info.PascalMain();
end;


procedure SysEntry_InitTLS(constref info: TEntryInformation);[public,alias:'FPC_SysEntry_InitTLS'];
begin
  initialstkptr := info.OS.stkptr;
  operatingsystem_parameter_envp := info.OS.envp;
  operatingsystem_parameter_argc := info.OS.argc;
  operatingsystem_parameter_argv := info.OS.argv;
{$ifdef INITTLS}
  InitTLS;
{$endif INITTLS}
{$ifdef cpui386}
  Set8087CW(Default8087CW);
{$endif cpui386}
  info.PascalMain();
end;

{$endif FPC_BOOTSTRAP_INDIRECT_ENTRY}

{$if defined(CPUARM) and defined(FPC_ABI_EABI)}
procedure haltproc(e:longint);cdecl;external name '_haltproc_eabi';
{$else}
procedure haltproc(e:longint);cdecl;external name '_haltproc';
{$endif}
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}

{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}

{$ifdef FPC_USE_LIBC}
function  FpPrCtl(options : cInt; const args : ptruint) : cint; cdecl; external clib name 'prctl';
{$endif}

procedure System_exit;
begin
{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
  EntryInformation.OS.haltproc(ExitCode);
{$else FPC_HAS_INDIRECT_ENTRY_INFORMATION}
  haltproc(ExitCode);
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}
End;


Function ParamCount: Longint;
Begin
  Paramcount:=argc-1
End;


{function BackPos(c:char; const s: shortstring): integer;
var
 i: integer;
Begin
  for i:=length(s) downto 0 do
    if s[i] = c then break;
  if i=0 then
    BackPos := 0
  else
    BackPos := i;
end;}


 { variable where full path and filename and executable is stored }
 { is setup by the startup of the system unit.                    }
var
 execpathstr : shortstring;

procedure SysInitExecPath;
var
  i    : longint;
begin
  execpathstr[0]:=#0;
  i:=Fpreadlink('/proc/self/exe',@execpathstr[1],high(execpathstr));
  { it must also be an absolute filename, linux 2.0 points to a memory
    location so this will skip that }
  if (i>0) and (execpathstr[1]='/') then
     execpathstr[0]:=char(i);
end;

function paramstr(l: longint) : string;
 begin
   { stricly conforming POSIX applications  }
   { have the executing filename as argv[0] }
   if l=0 then
     begin
       if execpathstr='' then
         SysInitExecPath;
       paramstr := execpathstr;
     end
   else if (l < argc) then
     paramstr:=strpas(argv[l])
   else
     paramstr:='';
 end;

Procedure Randomize;
Begin
  randseed:=longint(Fptime(nil));
End;

{*****************************************************************************
                                    cmdline
*****************************************************************************}

procedure SetupCmdLine;
var
  bufsize,
  len,j,
  size,i : longint;
  found  : boolean;
  buf    : pchar;

  procedure AddBuf;
  var
    p : Pchar;
  begin
    p:=SysGetmem(size+bufsize);
    move(calculated_cmdline^,p^,size);
    move(buf^,p[size],bufsize);
    inc(size,bufsize);
    sysfreemem(calculated_cmdline);
    calculated_cmdline:=p;
    bufsize:=0;
  end;

begin
  if argc<=0 then
    exit;
  Buf:=SysGetMem(ARG_MAX);
  size:=0;
  bufsize:=0;
  i:=0;
  while (i<argc) do
   begin
     len:=strlen(argv[i]);
     if len>ARG_MAX-2 then
      len:=ARG_MAX-2;
     found:=false;
     for j:=1 to len do
      if argv[i][j]=' ' then
       begin
         found:=true;
         break;
       end;
     found:=found or (len=0); // also quote if len=0, bug 19114
     if bufsize+len>=ARG_MAX-2 then
      AddBuf;
     if found then
      begin
        buf[bufsize]:='"';
        inc(bufsize);
      end;
     if len>0 then
       begin
         move(argv[i]^,buf[bufsize],len);
         inc(bufsize,len);
       end;
     if found then
      begin
        buf[bufsize]:='"';
        inc(bufsize);
      end;
     if i<argc-1 then
      buf[bufsize]:=' '
     else
      buf[bufsize]:=#0;
     inc(bufsize);
     inc(i);
   end;
  AddBuf;
  SysFreeMem(buf);
end;

function get_cmdline:Pchar;

begin
  if calculated_cmdline=nil then
    setupcmdline;
  get_cmdline:=calculated_cmdline;
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

function  reenable_signal(sig : longint) : boolean;
var
  e : TSigSet;
  i,j : byte;
  olderrno: cint;
begin
  fillchar(e,sizeof(e),#0);
  { set is 1 based PM }
  dec(sig);
  i:=sig mod (sizeof(cuLong) * 8);
  j:=sig div (sizeof(cuLong) * 8);
  e[j]:=1 shl i;
  { this routine is called from a signal handler, so must not change errno }
  olderrno:=geterrno;
  fpsigprocmask(SIG_UNBLOCK,@e,nil);
  reenable_signal:=geterrno=0;
  seterrno(olderrno);
end;

// signal handler is arch dependant due to processorexception to language
// exception translation

{$i sighnd.inc}

procedure InstallDefaultSignalHandler(signum: longint; out oldact: SigActionRec); public name '_FPC_INSTALLDEFAULTSIGHANDLER';
var
  act: SigActionRec;
begin
  { Initialize the sigaction structure }
  { all flags and information set to zero }
  FillChar(act, sizeof(SigActionRec),0);
  { initialize handler                    }
  act.sa_handler := SigActionHandler(@SignalToRunError);
  act.sa_flags:=SA_SIGINFO;
  FpSigAction(signum,@act,@oldact);
end;

var
  oldsigfpe: SigActionRec; public name '_FPC_OLDSIGFPE';
  oldsigsegv: SigActionRec; public name '_FPC_OLDSIGSEGV';
  oldsigbus: SigActionRec; public name '_FPC_OLDSIGBUS';
  oldsigill: SigActionRec; public name '_FPC_OLDSIGILL';

Procedure InstallSignals;
begin
  InstallDefaultSignalHandler(SIGFPE,oldsigfpe);
  InstallDefaultSignalHandler(SIGSEGV,oldsigsegv);
  InstallDefaultSignalHandler(SIGBUS,oldsigbus);
  InstallDefaultSignalHandler(SIGILL,oldsigill);
end;

procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
{$ifdef android}
  InitStdIOAndroid;
{$endif android}
end;

Procedure RestoreOldSignalHandlers;
begin
  FpSigAction(SIGFPE,@oldsigfpe,nil);
  FpSigAction(SIGSEGV,@oldsigsegv,nil);
  FpSigAction(SIGBUS,@oldsigbus,nil);
  FpSigAction(SIGILL,@oldsigill,nil);
end;

function GetProcessID: SizeUInt;
begin
 GetProcessID := SizeUInt (fpGetPID);
end;

{$ifdef FPC_USE_LIBC}
{$ifdef HAS_UGETRLIMIT}
    { there is no ugetrlimit libc call, just map it to the getrlimit call in these cases }
function FpUGetRLimit(resource : cInt; rlim : PRLimit) : cInt; cdecl; external clib name 'getrlimit';
{$endif}
{$endif}

{$if defined(CPUPOWERPC) or defined(CPUPOWERPC64)}
const
  page_size = $10000;
  {$define LAST_PAGE_GENERATES_SIGNAL}
{$else}
const
  page_size = $1000;
{$endif}

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
var
  limits : TRLimit;
  success : boolean;
begin
  success := false;
  fillchar(limits, sizeof(limits), 0);
  {$ifdef has_ugetrlimit}
  success := fpugetrlimit(RLIMIT_STACK, @limits)=0;
  {$endif}
  {$ifndef NO_SYSCALL_GETRLIMIT}
  if (not success) then
    success := fpgetrlimit(RLIMIT_STACK, @limits)=0;
  {$endif}
  if (success) and (limits.rlim_cur < stklen) then
    result := limits.rlim_cur
  else
    result := stklen;
end;

{$if FPC_FULLVERSION>30200}
{$if defined(CPUI386) or defined(CPUARM)}
{$I abitag.inc}
{$endif defined(CPUI386) or defined(CPUARM)}
{$endif FPC_FULLVERSION>30200}

begin
{$if defined(i386) and not defined(FPC_USE_LIBC)}
  InitSyscallIntf;
{$endif}

{$ifndef FPUNONE}
{$if defined(cpupowerpc)}
  // some PPC kernels set the exception bits FE0/FE1 in the MSR to zero,
  // disabling all FPU exceptions. Enable them again.
  fpprctl(PR_SET_FPEXC, PR_FP_EXC_PRECISE);
{$endif}
{$endif}
  IsConsole := TRUE;
  StackLength := CheckInitialStkLen(initialStkLen);
  StackBottom := pointer(ptruint((ptruint(initialstkptr) or (page_size - 1)) + 1 - StackLength));
{$ifdef LAST_PAGE_GENERATES_SIGNAL}
  StackBottom:=StackBottom + page_size;
{$endif}
  { Set up signals handlers (may be needed by init code to test cpu features) }
  InstallSignals;
{$if defined(cpui386) or defined(cpuarm)}
  fpc_cpucodeinit;
{$endif cpui386}

  { Setup heap }
  InitHeap;
  SysInitExceptions;
  initunicodestringmanager;
  { Setup stdin, stdout and stderr }
  SysInitStdIO;
  { Reset IO Error }
  InOutRes:=0;
  { threading }
  InitSystemThreads;
  { dynamic libraries }
  InitSystemDynLibs;
{$ifdef android}
  InitAndroid;
{$endif android}
  { restore original signal handlers in case this is a library }
  if IsLibrary then
    RestoreOldSignalHandlers;
end.
