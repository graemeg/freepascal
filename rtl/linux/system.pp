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

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef CPUI386}
{$DEFINE ELFRES32}
{$endif}

>>>>>>> graemeg/fixes_2_2
Unit System;

{*****************************************************************************}
                                    interface
{*****************************************************************************}

{$define FPC_IS_SYSTEM}
{$define HAS_CMDLINE}
{$define USE_NOTHREADMANAGER}

{$i osdefs.inc}

{$I sysunixh.inc}

function get_cmdline:Pchar; 
property cmdline:Pchar read get_cmdline;

{$if defined(CPUARM) or defined(CPUM68K) or (defined(CPUSPARC) and defined(VER2_6))}

{$define fpc_softfpu_interface}
{$i softfpu.pp}
{$undef fpc_softfpu_interface}

{$endif defined(CPUARM) or defined(CPUM68K) or (defined(CPUSPARC) and defined(VER2_6))}

{*****************************************************************************}
                                 implementation
{*****************************************************************************}

{$if defined(CPUI386) and not defined(FPC_USE_LIBC)}
var
  sysenter_supported: LongInt = 0;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$endif}

{ Include ELF resources }

{$ifdef ELFRES32}
{$define HAS_RESOURCES}
{$i elfres32.inc}
>>>>>>> graemeg/fixes_2_2
{$endif}

const calculated_cmdline:Pchar=nil;

{$if defined(CPUARM) or defined(CPUM68K) or (defined(CPUSPARC) and defined(VER2_6))}

{$define fpc_softfpu_implementation}
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

{$endif defined(CPUARM) or defined(CPUM68K) or (defined(CPUSPARC) and defined(VER2_6))}

{$I system.inc}

{$ifdef android}
{$I sysandroid.inc}
{$endif android}

{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}

{$if defined(CPUARM) and defined(FPC_ABI_EABI)}
procedure haltproc(e:longint);cdecl;external name '_haltproc_eabi';
{$else}
procedure haltproc(e:longint);cdecl;external name '_haltproc';
{$endif}

{$ifdef FPC_USE_LIBC}
function  FpPrCtl(options : cInt; const args : ptruint) : cint; cdecl; external clib name 'prctl';
{$endif}

procedure System_exit;
begin
  haltproc(ExitCode);
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

function paramstr(l: longint) : string;
 begin
   { stricly conforming POSIX applications  }
   { have the executing filename as argv[0] }
   if l=0 then
     begin
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
  begin
    reallocmem(calculated_cmdline,size+bufsize);
    move(buf^,calculated_cmdline[size],bufsize);
    inc(size,bufsize);
    bufsize:=0;
  end;

begin
  if argc<=0 then
    exit;
  GetMem(buf,ARG_MAX);
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
  FreeMem(buf,ARG_MAX);
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
end;

Procedure RestoreOldSignalHandlers;
begin
  FpSigAction(SIGFPE,@oldsigfpe,nil);
  FpSigAction(SIGSEGV,@oldsigsegv,nil);
  FpSigAction(SIGBUS,@oldsigbus,nil);
  FpSigAction(SIGILL,@oldsigill,nil);
end;


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

var
  initialstkptr : Pointer;external name '__stkptr';
begin
{$if defined(i386) and not defined(FPC_USE_LIBC)}
  InitSyscallIntf;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
  SysResetFPU;
  if not(IsLibrary) then
    SysInitFPU;
>>>>>>> graemeg/fixes_2_2
=======
  SysResetFPU;
  if not(IsLibrary) then
    SysInitFPU;
>>>>>>> origin/fixes_2_2
{$if defined(cpupowerpc)}
  // some PPC kernels set the exception bits FE0/FE1 in the MSR to zero,
  // disabling all FPU exceptions. Enable them again.
  fpprctl(PR_SET_FPEXC, PR_FP_EXC_PRECISE);
{$endif}
{$endif}
  IsConsole := TRUE;
  StackLength := CheckInitialStkLen(initialStkLen);
  StackBottom := initialstkptr - StackLength;
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
  { Arguments }
  SysInitExecPath;
  { Reset IO Error }
  InOutRes:=0;
  { threading }
  InitSystemThreads;
  { dynamic libraries }
  InitSystemDynLibs;
  { restore original signal handlers in case this is a library }
  if IsLibrary then
    RestoreOldSignalHandlers;
end.
