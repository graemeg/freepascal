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

const
  FPE_INTDIV = 1;
  FPE_INTOVF = 2;
  FPE_FLTDIV = 3;
  FPE_FLTOVF = 4;
  FPE_FLTUND = 5;
  FPE_FLTRES = 6;
  FPE_FLTINV = 7;
  FPE_FLTSUB = 8;

 
procedure SignalToRunerror(sig : longint; SigInfo: PSigInfo; SigContext: PSigContext);public name '_FPC_DEFAULTSIGHANDLER';cdecl;
var
  res : word;
  addr : pointer;
  frame : pointer;
begin
  res:=0;
  if assigned(sigcontext) then
    begin
      addr := pointer(sigcontext^.uc_mcontext.gregs[REG_PC]);
      frame := pointer(sigcontext^.uc_mcontext.gregs[REG_FP])
    end
  else
    begin
      addr := nil;
      frame := nil;
    end;
  case sig of
    SIGFPE :
        begin
          case  siginfo^.si_code of
            FPE_INTDIV:
              res:=200;
            FPE_INTOVF:
              res:=205;
            FPE_FLTDIV:
              res:=200;
            FPE_FLTOVF:
              res:=205;
            FPE_FLTUND:
              res:=206;
            else
              res:=207;
         end;
         with sigcontext^.uc_mcontext.fpregs.fpchip_state do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           begin
             status:=status and not FPU_ExceptionMask;
             { Control word is index 0 }
             state[0]:=Default8087CW;
             { Status word is also index 1 }
             state[1]:=status;
             { Tag word is index 2 }
             state[2]:=$ffff;
          end;
=======
           status:=status and not FPU_ExceptionMask;
>>>>>>> graemeg/cpstrnew
=======
           status:=status and not FPU_ExceptionMask;
>>>>>>> graemeg/cpstrnew
=======
           status:=status and not FPU_ExceptionMask;
>>>>>>> graemeg/cpstrnew
=======
           status:=status and not FPU_ExceptionMask;
>>>>>>> origin/cpstrnew
       end;
   SIGILL:
       if sse_check then
        begin
          os_supports_sse:=false;
          res:=0;
          inc(sigcontext^.uc_mcontext.gregs[REG_EIP],3);
        end
      else
        res:=216;
    SIGSEGV :
        begin
          res:=216;
        end;
    SIGBUS :
      begin
        res:=214;
      end;
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
  end;
  reenable_signal(sig);
  { give runtime error at the position where the signal was raised }
  {if res<>0 then
    HandleErrorAddrFrame(res,addr,frame);}
{ give runtime error at the position where the signal was raised }
  if res<>0 then
  begin
    sigcontext^.uc_mcontext.gregs[REG_EAX] := res;
    sigcontext^.uc_mcontext.gregs[REG_EDX] := sigcontext^.uc_mcontext.gregs[REG_EIP];
    sigcontext^.uc_mcontext.gregs[REG_ECX] := sigcontext^.uc_mcontext.gregs[REG_EBP];
    sigcontext^.uc_mcontext.gregs[REG_EIP] := ptruint(@HandleErrorAddrFrame);
  end;

end;


