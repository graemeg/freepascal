{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team.

    Processor dependent implementation for the system unit for
    Xtensa

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$define FPC_SYSTEM_HAS_SYSRESETFPU}
Procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

{$define FPC_SYSTEM_HAS_SYSINITFPU}
Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

procedure fpc_cpuinit;
begin
  { don't let libraries influence the FPU cw set by the host program }
  if not IsLibrary then
    SysInitFPU;
end;


{$IFNDEF INTERNAL_BACKTRACE}
{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;nostackframe;
  asm
  end;
{$ENDIF not INTERNAL_BACKTRACE}


{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler;nostackframe;
  asm
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler;nostackframe;
  asm
  end;


{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : pointer;assembler;nostackframe;
  asm
  end;


function InterLockedDecrement (var Target: longint) : longint;
  var
    temp_sreg : byte;
  begin
    Result:=Target-1;
    Target:=Result;
  end;


function InterLockedIncrement (var Target: longint) : longint;
  var
    temp_sreg : byte;
  begin
    Result:=Target+1;
    Target:=Result;
  end;


function InterLockedExchange (var Target: longint;Source : longint) : longint;
  var
    temp_sreg : byte;
  begin
    Result:=Target;
    Target:=Source;
  end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint;
  var
    temp_sreg : byte;
  begin
    Result:=Target;
    if Result=Comperand then
      Target:=NewValue;
  end;


function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint;
  var
    temp_sreg : byte;
  begin
    Result:=Target;
    Target:=Result+Source;
  end;


function InterLockedDecrement (var Target: smallint) : smallint;
  var
    temp_sreg : byte;
  begin
    Result:=Target-1;
    Target:=Result;
  end;


function InterLockedIncrement (var Target: smallint) : smallint;
  var
    temp_sreg : byte;
  begin
    Result:=Target+1;
    Target:=Result;
  end;


function InterLockedExchange (var Target: smallint;Source : smallint) : smallint;
  var
    temp_sreg : byte;
  begin
    Result:=Target;
    Target:=Source;
  end;


function InterlockedCompareExchange(var Target: smallint; NewValue: smallint; Comperand: smallint): smallint;
  var
    temp_sreg : byte;
  begin
    Result:=Target;
    if Result=Comperand then
      Target:=NewValue;
  end;


function InterLockedExchangeAdd (var Target: smallint;Source : smallint) : smallint;
  var
    temp_sreg : byte;
  begin
    Result:=Target;
    Target:=Result+Source;
  end;


