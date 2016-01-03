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

procedure SignalToRunerror(Sig: cint; info : psiginfo;  SigContext:PSigContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl;

var
  res : word;

begin
  res:=0;
  case sig of
    SIGFPE :
          begin
            Case Info^.si_code Of
                FPE_INTDIV : Res:=200;  {integer divide fault. Div0?}
                FPE_FLTOVF : Res:=205;  {Overflow trap}
                FPE_FLTUND : Res:=206;  {Stack over/underflow}
                FPE_FLTRES : Res:=216;  {Device not available}
                FPE_FLTINV : Res:=216;  {Invalid floating point operation}
               Else
                Res:=208; {coprocessor error}
                End;
             sysResetFPU;
          End;
    SIGILL:
      if sse_check then
        begin
          os_supports_sse:=false;
          res:=0;
          inc(sigcontext^.sc_eip,3);
        end
      else
        res:=216;
    SIGBUS:
      res:=214;
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
{ give runtime error at the position where the signal was raised }
  if res<>0 then
   begin
{$ifdef cpui386}
  HandleErrorAddrFrame(res,Pointer(SigContext^.sc_eip),pointer(SigContext^.sc_ebp));
{$else}
     HandleError(res);
{$endif}
   end;
end;
