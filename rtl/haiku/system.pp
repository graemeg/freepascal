{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2019 the Free Pascal development team.

    System unit for Haiku

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


Unit System;

interface

{$define FPC_IS_SYSTEM}

{$I sysunixh.inc}

implementation

var
  initialstkptr : Pointer; external name '__stkptr';

procedure debugger(s : PChar); cdecl; external 'root' name 'debugger';
function disable_debugger(state : integer): integer; cdecl; external 'root' name 'disable_debugger';


{ OS independant parts}

{$I system.inc}

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}
{$ifdef legacy_startup}
procedure prthaltproc;external name '_haltproc';

procedure system_exit;
begin
  asm
    jmp prthaltproc
  end;
End;
{$else legacy_startup}
procedure haltproc(exitcode: longint); cdecl; external name '_haltproc';

procedure system_exit;
begin
  haltproc(ExitCode);
end;
{$endif legacy_startup}


{ OS dependant parts  }


{ $I text.inc}

{*****************************************************************************
                           UnTyped File Handling
*****************************************************************************}

{ $i file.inc}

{*****************************************************************************
                           Typed File Handling
*****************************************************************************}

{ $i typefile.inc}

{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}

Function ParamCount: Longint;
Begin
  Paramcount := argc - 1;
End;

 { variable where full path and filename and executable is stored }
 { is setup by the startup of the system unit.                    }
var
 execpathstr : shortstring;

{$ifdef FPC_USE_LIBC}

// private; use the macros, below
function _get_image_info(image : image_id; var info : image_info; size : size_t)
         : status_t; cdecl; external 'root' name '_get_image_info';

function _get_next_image_info(team : team_id; var cookie : Longint; var info : image_info; size : size_t)
         : status_t; cdecl; external 'root' name '_get_next_image_info';

function get_image_info(image : image_id; var info : image_info) : status_t;
begin
  Result := _get_image_info(image, info, SizeOf(info));
end;

function get_next_image_info(team : team_id; var cookie : Longint; var info : image_info) : status_t;
begin
  Result := _get_next_image_info(team, cookie, info, SizeOf(info));
end;

{$endif}

{ this routine sets up the paramstr(0) string at startup }
procedure setupexecname;
var
 cookie: longint;
 image : image_info;
 index : byte;
 s : string;
begin
  cookie:=0;
  fillchar(image, sizeof(image_info), 0);
  if get_next_image_info(0, cookie, image) = B_OK then
  begin
    execpathstr := strpas(@image.name);
  end
  else
    execpathstr := '';
  { problem with Be 4.5 noted... path contains . character }
  { if file is directly executed in CWD                    }
  index:=pos('/./',execpathstr);
  if index <> 0 then
    begin
      { remove the /. characters }
      Delete(execpathstr,index, 2);
    end;
end;

function paramstr(l: longint) : string;
var
  s: string;
  s1: string;
begin
  { stricly conforming POSIX applications  }
  { have the executing filename as argv[0] }
  if l = 0 then
  begin
    paramstr := execpathstr;
  end
  else if (l < argc) then
  begin
    paramstr:=strpas(argv[l]);
  end
  else
    paramstr := '';
end;

Procedure Randomize;
Begin
  randseed:=longint(Fptime(nil));
End;

function GetProcessID: SizeUInt;
begin
  GetProcessID := SizeUInt (fpGetPID);
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

procedure set_signal_stack(ptr : pointer; size : size_t); cdecl; external 'root' name 'set_signal_stack';
function sigaltstack(const stack : pstack_t; oldStack : pstack_t) : integer; cdecl; external 'root' name 'sigaltstack'; 

type
  {$PACKRECORDS C}
  TAlternateSignalStack = record
    case Integer of
      0 : (buffer : array[0..(SIGSTKSZ * 4)-1] of Char);
      1 : (ld : clonglong);
      2 : (l : integer);
      3 : (p : pointer);
  end;

var
  alternate_signal_stack : TAlternateSignalStack;

procedure InstallDefaultSignalHandler(signum: longint; out oldact: SigActionRec); public name '_FPC_INSTALLDEFAULTSIGHANDLER';
var
  r : integer;
  st : stack_t;
  act : SigActionRec;
begin
  st.ss_flags := 0;
  st.ss_sp := @alternate_signal_stack.buffer;
  st.ss_size := SizeOf(alternate_signal_stack.buffer);

  r := sigaltstack(@st, nil);

  if (r <> 0) then
  begin
    debugger('sigaltstack error');
  end;

  { Initialize the sigaction structure }
  { all flags and information set to zero }
  FillChar(act, sizeof(SigActionRec), #0);
  { initialize handler                    }
  act.sa_mask[0] := 0;
  act.sa_handler := SigActionHandler(@SignalToRunError);
  act.sa_flags := SA_ONSTACK or SA_SIGINFO;
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

Procedure RestoreOldSignalHandlers;
begin
  FpSigAction(SIGFPE,@oldsigfpe,nil);
  FpSigAction(SIGSEGV,@oldsigsegv,nil);
  FpSigAction(SIGBUS,@oldsigbus,nil);
  FpSigAction(SIGILL,@oldsigill,nil);
end;

procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in and messagebox }
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

begin
  IsConsole := TRUE;
  StackLength := CheckInitialStkLen(InitialStkLen);
{$if FPC_FULLVERSION >= 30301}
  StackBottom := initialstkptr - StackLength;
{$else}
  StackBottom := Sptr - StackLength;
{$endif}
  ReturnNilIfGrowHeapFails := False;

  { Set up signals handlers }
  InstallSignals;

{$ifdef cpui386}
  fpc_cpucodeinit;
{$endif}

  { Setup heap }
  InitHeap;

  SysInitExceptions;
  initunicodestringmanager;
  { Setup IO }
  SysInitStdIO;
  { Reset IO Error }
  InOutRes:=0;
  InitSystemThreads;
  InitSystemDynLibs;
  setupexecname;

  { restore original signal handlers in case this is a library }
  if IsLibrary then
    RestoreOldSignalHandlers;
end.
