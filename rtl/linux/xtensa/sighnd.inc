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

function GetHandleErrorAddrFrameAddr: pointer;
begin
  result:=@HandleErrorAddrFrame;
end;


Procedure SignalToHandleErrorAddrFrame(Errno : longint;addr : CodePointer; frame : Pointer); nostackframe; assembler;
asm
end;


procedure SignalToRunerror(Sig: longint; { _a2,_a3,_a4 : dword; } SigContext: PSigInfo; uContext : PuContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl;
var
  res : word;
begin
  res:=0;
  case sig of
    SIGFPE :
        begin
          res := 207;
          SysResetFPU;
        end;
    SIGILL:
        res:=216;
    SIGSEGV :
        res:=216;
    SIGBUS:
        res:=214;
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
  end;
  { give runtime error at the position where the signal was raised }
  if res<>0 then
    begin
{!!!      ucontext^.uc_mcontext.arm_r0:=res;
      ucontext^.uc_mcontext.arm_r1:=uContext^.uc_mcontext.arm_pc;
      ucontext^.uc_mcontext.arm_r2:=uContext^.uc_mcontext.arm_fp;
      ucontext^.uc_mcontext.arm_pc:=ptruint(@SignalToHandleErrorAddrFrame); }
    end;
end;


