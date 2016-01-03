{
   This file is part of the Free Pascal run time library.
   (c) 2000-2003 by Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   Signalhandler for FreeBSD/i386

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

<<<<<<< HEAD
<<<<<<< HEAD
procedure SignalToRunerror(Sig: cint; info : PSigInfo; SigContext:PSigContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl;
=======
procedure SignalToRunerror(Sig: cint; info : PSigInfo; SigContext:PSigContext); cdecl;
>>>>>>> graemeg/fixes_2_2
=======
procedure SignalToRunerror(Sig: cint; info : PSigInfo; SigContext:PSigContext); cdecl;
>>>>>>> origin/fixes_2_2

var
  res : word;

begin
  res:=0;
  case sig of
    SIGFPE :
      begin
        Case Info^.si_code Of
          FPE_FLTDIV,
          FPE_INTDIV : Res:=200;  { floating point divide by zero }
          FPE_FLTOVF : Res:=205;  { floating point overflow }
          FPE_FLTUND : Res:=206;  { floating point underflow }
          FPE_FLTRES,             { floating point inexact result }
          FPE_FLTINV : Res:=207;  { invalid floating point operation }
          Else
            Res:=207; {coprocessor error}
        end;
        { FPU exceptions are completely disabled by the kernel if one occurred, it  }
        { seems this is necessary to be able to return to user mode. They can be    }
        { enabled by executing a sigreturn, however then the exception is triggered }
        { again immediately if we don't turn off the "exception occurred" flags     }
        { in fpscr                                                                  }
        SigContext^.uc_mcontext^.fs.fpscr := SigContext^.uc_mcontext^.fs.fpscr and not($fffe0700);
      end;
    SIGILL,
    SIGBUS,
    SIGSEGV :
        res:=216;
<<<<<<< HEAD
<<<<<<< HEAD
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  {$ifdef FPC_USE_SIGPROCMASK}
   reenable_signal(sig);
  {$endif }

  { return to trampoline }
  if res <> 0 then
    begin
      SigContext^.uc_mcontext^.ss.r3 := res;
      SigContext^.uc_mcontext^.ss.r4 := SigContext^.uc_mcontext^.ss.srr0;
      SigContext^.uc_mcontext^.ss.r5 := SigContext^.uc_mcontext^.ss.r1;
      pointer(SigContext^.uc_mcontext^.ss.srr0) := @HandleErrorAddrFrame;
    end;
end;

