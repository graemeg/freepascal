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

procedure SignalToRunerror(sig : longint; SigInfo: PSigInfo; SigContext: PSigContext);public name '_FPC_DEFAULTSIGHANDLER';cdecl;
var
  res : word;
  addr, framebp : pointer;
begin
  res:=0;
  addr:=nil;
  framebp:=nil;
  case sig of
    SIGFPE :
        begin
<<<<<<< HEAD
          if assigned(siginfo) then
            begin
              addr := siginfo^._sifields._sigfault._addr;
              case  siginfo^.si_code of
                FPE_INTDIV:
                  res:=200;
                FPE_INTOVF:
                  res:=215;
                FPE_FLTDIV:
                  res:=200;
                FPE_FLTOVF:
                  res:=205;
                FPE_FLTUND:
                  res:=206;
                else
                  res:=207;
              end;
            end;
=======
          addr := siginfo^._sifields._sigfault._addr;
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
>>>>>>> graemeg/fixes_2_2
        end;
    SIGBUS :
        begin
          if assigned(siginfo) then
            addr := siginfo^._sifields._sigfault._addr;
          res:=214;
        end;
    SIGILL,
    SIGSEGV :
        begin
          if assigned(siginfo) then
            addr := siginfo^._sifields._sigfault._addr;
          res:=216;
        end;
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
  end;
  reenable_signal(sig);

  if assigned(sigcontext) then
    begin
      addr:=pointer(sigcontext^.sigc_pc);
      framebp:=pointer(sigcontext^.sigc_gregs[SIG_SP]);
    end;
  
  { give runtime error at the position where the signal was raised }
  if res<>0 then
    HandleErrorAddrFrame(res,addr,framebp);
end;


