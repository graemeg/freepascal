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
  { Bits in control register }
  RoundingMode = $30;
  RoundingPrecision = $c0;
  { Exception bits common to status and control registers }
  InexactDecimal = $100;
  InexactOperation = $200;
  DivideByZero = $400;
  UnderFlow = $800;
  OverFlow = $1000;
  OperandError = $2000;
  SignalingNaN = $4000;
  BranchOnUnordered = $8000;
  FPU_ES_Mask = $ff00;
  { Accrued exception bit only in status register }
  AE_Inexact = $8;
  AE_DivideByZero = $10;
  AE_Underflow = $20;
  AE_Overflow = $40;
  AE_InvalidOperation = $80;
  FPU_AE_Mask = $F80;



  reset_fpucw : longint = {InexactOperation or }DivideByZero or
    OverFlow or OperandError or
    SignalingNaN or BranchOnUnordered;

  reset_fpust : longint = 0;
  { Bits in psr SigContext field }
  PSR_Invalid = $80;
  PSR_Denormal = $8;
  PSR_DivisionByZero = $10;
  PSR_Overflow = $40;
  PSR_Underflow = $20;
  { m68k is not stack based }
  PSR_StackUnderflow = $0;
  PSR_StackOverflow = $0;
  FPU_Status_Exception_Mask = FPU_ES_Mask or FPU_AE_Mask;
  PSR_Exception_Mask =$f8;
  FPU_Control_Exception_Mask = FPU_ES_Mask;

Procedure ResetFPU;
var
  l_fpucw : longint;
begin
  
{$ifdef CPU68020}
  asm 
    fmove.l fpcr,l_fpucw
  end;
  { Reset only exception based control bits in fpcr }
  l_fpucw := (l_fpucw and not (dword(FPU_Control_Exception_Mask)))
             or (reset_fpucw and FPU_Control_Exception_Mask);
  asm
    fmove.l l_fpucw,fpcr
    { Reset fpsr to zero }
    fmove.l reset_fpust,fpsr
  end;
{$endif}
end;


function GetFPUState(const SigContext : TSigContext) : dword;
begin
  GetfpuState:=dword(SigContext.psr);
{$ifdef SYSTEM_DEBUG}
  Writeln(stderr,'FpuState = ',GetFpuState);
{$endif SYSTEM_DEBUG}
end;



procedure SignalToRunerror(Sig: longint; Info : pointer; var SigContext: TSigContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl;
var
  res : word;
  fpustate : dword;
begin
  res:=0;
  case sig of
    SIGFPE :
          begin
    { this is not allways necessary but I don't know yet
      how to tell if it is or not PM }
          res:=200;
          fpustate:=GetFPUState(SigContext);
          if (FpuState and FPU_Status_Exception_Mask) <> 0 then
            begin
              { first check the more precise options }
              if (FpuState and (DivideByZero or AE_DividebyZero))<>0 then
                res:=200
              else if (FpuState and (Overflow or AE_Overflow))<>0 then
                res:=205
              else if (FpuState and (Underflow or AE_Underflow))<>0 then
                res:=206
              else if (FpuState and PSR_Denormal)<>0 then
                res:=216
              { else if (FpuState and (PSR_StackOverflow or PRS_StackUnderflow))<>0 then
                res:=207, disabled, as there is no fpu stack }
              else if (FpuState and (OperandError or SignalingNan or BranchOnUnordered or AE_InvalidOperation))<>0 then
                res:=216
              else
                res:=207;  {'Coprocessor Error'}
            end;
          ResetFPU;
        end;
    SIGILL,
    SIGBUS,
    SIGSEGV :
        res:=216;
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
  end;

  reenable_signal(sig);

  { give runtime error at the position where the signal was raised }
  if res<>0 then
   begin
     { HandleErrorAddrFrame(res,SigContext.sc_pc,SigContext.sc_fp);}
     { fp is not saved in context record :( }
     HandleError(res);
     HandleError(res);
   end;
end;
