{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    Signal handler is arch dependant due to processor to language
    exception conversion.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


<<<<<<< HEAD
<<<<<<< HEAD
procedure SignalToRunerror(sig : longint; SigContext: PSigContextRec; uContext: Pvregs);public name '_FPC_DEFAULTSIGHANDLER';cdecl;
=======
procedure SignalToRunerror(sig : longint; SigContext: PSigContextRec; uContext: Pvregs);cdecl;
>>>>>>> graemeg/fixes_2_2
=======
procedure SignalToRunerror(sig : longint; SigContext: PSigContextRec; uContext: Pvregs);cdecl;
>>>>>>> origin/fixes_2_2
var
  res,fpustate : word;
begin
  res:=0;
  case sig of
    SIGFPE :
      begin
        { this is not allways necessary but I don't know yet
          how to tell if it is or not PM }
        res:=200;
        // fp_status always here under BeOS and x86 CPU
        // (fp_status is not behind a pointer in the BeOS context record)
        FpuState:=ucontext^.xregs.state.old_format.fp_status;
            
        if (FpuState and FPU_ExceptionMask) <> 0 then
          begin
            { first check the more precise options }
            if (FpuState and FPU_DivisionByZero)<>0 then
              res:=200
            else if (FpuState and (FPU_StackOverflow or FPU_StackUnderflow or FPU_Invalid))<>0 Then
              res:=207
            else if (FpuState and FPU_Overflow)<>0 then
              res:=205
            else if (FpuState and FPU_Underflow)<>0 then
              res:=206
            else if (FpuState and FPU_Denormal)<>0 then
              res:=216
            else
              res:=207;  {'Coprocessor Error'}
          end;
        with ucontext^.xregs.state.old_format do
        begin
          fp_status := fp_status and not FPU_ExceptionMask;
        end;
        SysResetFPU;
      end;
(*    SIGBUS: {Same as SIGSEGV under BeOS}
      begin
        res:=214;
      end; *)
    SIGILL:
      begin
      if sse_check then
        begin
          os_supports_sse := false;
          res := 0;
          inc(ucontext^.eip, 3);
        end
      else
        res:=216;
      end;
    SIGSEGV :
      begin
        res:=216;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
    SIGINT:
      begin
        res:=217
      end;
    SIGQUIT :
      begin
        res:=233;
      end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  reenable_signal(sig);
{ give runtime error at the position where the signal was raised }
  if res<>0 then
  begin
    HandleErrorAddrFrame(res, pointer(ucontext^.eip),
                              pointer(ucontext^.ebp));    
  end;
end;


