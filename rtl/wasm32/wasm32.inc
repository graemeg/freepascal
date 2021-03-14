{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2017 by the Free Pascal development team.

    Processor dependent implementation for the system unit for
    WebAssembly 32-bit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

procedure fpc_cpuinit;
  begin
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;
  begin
    result:=nil;
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;
  begin
    result:=nil;
  end;


{$define FPC_SYSTEM_HAS_SPTR}
function Sptr : pointer;
  begin
    result:=nil;
  end;


function InterLockedDecrement (var Target: longint) : longint;
  begin
    dec(Target);
    Result:=Target;
  end;


function InterLockedIncrement (var Target: longint) : longint;
  begin
    inc(Target);
    Result:=Target;
  end;


function InterLockedExchange (var Target: longint;Source : longint) : longint;
  begin
    Result:=Target;
    Target:=Source;
  end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint;
  begin
    Result:=Target;
    if Target=Comperand then
      Target:=NewValue;
  end;


function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint;
  begin
    Result:=Target;
    inc(Target,Source);
  end;


function InterLockedDecrement (var Target: smallint) : smallint;
  begin
    dec(Target);
    Result:=Target;
  end;


function InterLockedIncrement (var Target: smallint) : smallint;
  begin
    inc(Target);
    Result:=Target;
  end;


function InterLockedExchange (var Target: smallint;Source : smallint) : smallint;
  begin
    Result:=Target;
    Target:=Source;
  end;


function InterlockedCompareExchange(var Target: smallint; NewValue: smallint; Comperand: smallint): smallint;
  begin
    Result:=Target;
    if Target=Comperand then
      Target:=NewValue;
  end;


function InterLockedExchangeAdd (var Target: smallint;Source : smallint) : smallint;
  begin
    Result:=Target;
    inc(Target,Source);
  end;
