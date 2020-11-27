{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    AIX system unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;

interface

{$define FPC_IS_SYSTEM}

{$linklib m}

{ include system-independent routine headers }

{$I sysunixh.inc}

var argc:longint;
    argv:PPchar;
    envp:PPchar;

implementation


{ OS independant parts}

{$I system.inc}

{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}

procedure pascalmain;external name 'PASCALMAIN';

procedure FPC_SYSTEMMAIN(argcparam: Longint; argvparam: ppchar; envpparam: ppchar); cdecl; [public];

begin
  argc:=argcparam;
  argv:=argvparam;
  envp:=envpparam;
  pascalmain;  {run the pascal main program}
end;


procedure System_exit;
begin
   Fpexit(cint(ExitCode));
End;


Function ParamCount: Longint;
Begin
  Paramcount:=argc-1
End;


function BackPos(c:char; const s: shortstring): integer;
var
 i: integer;
Begin
  for i:=length(s) downto 0 do
    if s[i] = c then break;
  if i=0 then
    BackPos := 0
  else
    BackPos := i;
end;


function paramstr(l: longint) : string;
 var
  s: string;
  s1: string;
 begin
   { stricly conforming POSIX applications  }
   { have the executing filename as argv[0] }
     if (l < argc) then
       paramstr:=strpas(argv[l])
     else
       paramstr:='';
 end;

Procedure Randomize;
Begin
  randseed:=longint(Fptime(nil));
End;


{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

function  reenable_signal(sig : longint) : boolean;
var
  e,oe : TSigSet;
  i,j : byte;
  olderrno: cint;
begin
  fillchar(e,sizeof(e),#0);
  fillchar(oe,sizeof(oe),#0);
  { set is 1 based PM }
  dec(sig);
  i:=sig mod sizeof(clong);
  j:=sig div sizeof(clong);
  e[j]:=1 shl i;
  { this routine is called from a signal handler, so must not change errno }
  olderrno:=geterrno;
  fpsigprocmask(SIG_UNBLOCK,@e,@oe);
  reenable_signal:=geterrno=0;
  seterrno(olderrno);
end;

{$i sighnd.inc}

procedure InstallDefaultSignalHandler(signum: longint; out oldact: SigActionRec); public name '_FPC_INSTALLDEFAULTSIGHANDLER';
var
  act: SigActionRec;
begin
  { Initialize the sigaction structure }
  { all flags and information set to zero }
  FillChar(act, sizeof(SigActionRec),0);
  { initialize handler                    }
  act.sa_handler:=@SignalToRunError;
  act.sa_flags:=SA_SIGINFO;
  FpSigAction(signum,act,oldact);
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

Procedure RestoreOldSignalHandlers;
begin
  FpSigAction(SIGFPE,@oldsigfpe,nil);
  FpSigAction(SIGSEGV,@oldsigsegv,nil);
  FpSigAction(SIGBUS,@oldsigbus,nil);
  FpSigAction(SIGILL,@oldsigill,nil);
end;


procedure SetupCmdLine;
var
  bufsize,
  len,j,
  size,i : longint;
  found  : boolean;
  buf    : pchar;

  procedure AddBuf;
  begin
    reallocmem(cmdline,size+bufsize);
    move(buf^,cmdline[size],bufsize);
    inc(size,bufsize);
    bufsize:=0;
  end;

begin
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
     if bufsize+len>=ARG_MAX-2 then
      AddBuf;
     if found then
      begin
        buf[bufsize]:='"';
        inc(bufsize);
      end;
     move(argv[i]^,buf[bufsize],len);
     inc(bufsize,len);
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


procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
end;


function GetProcessID: SizeUInt;
begin
 GetProcessID := SizeUInt (fpGetPID);
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;


const
  FP_TRAP_SYNC = 1;                { precise fpu exceptions }
  FP_TRAP_OFF = 0;                 { disable fpu exceptions }
  FP_TRAP_QUERY = 2;               { current fpu exception state }
  FP_TRAP_IMP = 3;                 { imprecise non-recoverable fpu exceptions }
  FP_TRAP_IMP_REC = 4;             { imprecise recoverable fpu exceptions }
  FP_TRAP_FASTMODE = 128;          { fastest fpu exception state }
  FP_TRAP_ERROR = -1;
  FP_TRAP_UNIMPL = -2;

  TRP_INVALID     = $00000080;
  TRP_OVERFLOW    = $00000040;
  TRP_UNDERFLOW   = $00000020;
  TRP_DIV_BY_ZERO = $00000010;
  TRP_INEXACT     = $00000008;


function fp_trap(flag: longint): longint; cdecl; external;
procedure fp_enable(Mask: DWord);cdecl;external;

Begin
  IsConsole := TRUE;
  StackLength := CheckInitialStkLen(InitialStkLen);
  StackBottom := Sptr - StackLength;
  { Set up signals handlers (may be needed by init code to test cpu features) }
  InstallSignals;

  SysResetFPU;
  if not(IsLibrary) then
    begin
      { clear pending exceptions }
      feclearexcept(FE_ALL_EXCEPT);
      { enable floating point exceptions process-wide (try two possibilities) }
      if fp_trap(FP_TRAP_SYNC)=FP_TRAP_UNIMPL then
        fp_trap(FP_TRAP_IMP);

      SysInitFPU;
      { now enable the actual individual exceptions, except for underflow and
        inexact (also disabled by default on x86 and in the softfpu mask) }
      fp_enable(TRP_INVALID or TRP_DIV_BY_ZERO or TRP_OVERFLOW);
    end;

{ Setup heap }
  InitHeap;
  SysInitExceptions;

  initunicodestringmanager;

{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Reset IO Error }
  InOutRes:=0;
{ Arguments }
  SetupCmdLine;
  InitSystemThreads;
  InitSystemDynLibs;
  { restore original signal handlers in case this is a library }
  if IsLibrary then
    RestoreOldSignalHandlers;
End.
