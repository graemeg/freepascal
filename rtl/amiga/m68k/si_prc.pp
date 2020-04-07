{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by the Free Pascal development team

    System Entry point for Amiga/68k, Pascal only programs

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit si_prc;

interface

implementation

var
  AOS_ExecBase: Pointer; public name '_ExecBase';
  realExecBase: Pointer absolute $4;
  StkLen: LongInt; external name '__stklen';
  sysinit_jmpbuf: jmp_buf;
  ExitCode: LongInt;

{ the definitions in there need AOS_Execbase }
{$include execd.inc}
{$include execf.inc}

{$if defined(AMIGA_V1_0_ONLY) or defined(AMIGA_V1_2_ONLY)}
{$define AMIGA_LEGACY}
{$include legacyexech.inc}
{$endif}

{$ifdef AMIGA_LEGACY}
var
  args: pointer; public name '__fpc_args';
  arglen: dword; public name '__fpc_arglen';
{$endif}

var
  sst: TStackSwapStruct;

procedure PascalMain; external name 'PASCALMAIN';


{ this function must be the first in this unit which contains code }
function _FPC_proc_start: longint; cdecl; public name '_start';
var
  newStack: Pointer;
  task: PTask;
begin
{$IFDEF AMIGA_LEGACY}
  asm
    move.l d0, arglen
    move.l a0, args
  end;
{$ENDIF}
  AOS_ExecBase:=realExecBase;
  newStack:=nil;

  task:=FindTask(nil);
  if (task^.tc_SPUpper-task^.tc_SPLower < StkLen) then
    begin
      newStack:=AllocVec(StkLen,MEMF_ANY);

      sst.stk_Lower:=newStack;
      sst.stk_Upper:=newStack+StkLen;
      sst.stk_Pointer:=newStack+StkLen;

      StackSwap(@sst);
    end;

  { Note: code between the two stackswaps only works because of the
    nature of the generated code. We're accessing globals which is
    safe, and the locals are either kept in reg, or accessed via
    the base pointer (A5), and because we don't use the stack for
    call arguments, only regs. If this CG behavior changes, this
    code might break. In that case an asm-written StackSwap+call
    wrapper code is the solution. (Basically the reimplementation
    of AROS' NewStackSwap or MorphOS' NewPPCStackSwap.) (KB) }

  if setjmp(sysinit_jmpbuf) = 0 then
    PascalMain;

  if newStack <> nil then
    begin
      StackSwap(@sst);
      FreeVec(newStack);
    end;

  _FPC_proc_start:=ExitCode;
end;

procedure _FPC_proc_halt(_ExitCode: longint); cdecl; public name '_haltproc';
begin
  ExitCode:=_ExitCode;
  longjmp(sysinit_jmpbuf,1);
end;


end.
