{
    Copyright (c) 2018 by Florian Klaempfl

    Basic infrastructure for measuring timings of different compilation steps

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit cprofile;

{ to use this profile intrastructure, the epiktimer sources must be available, the official repository is currently:

  https://github.com/graemeg/epiktimer.git

  clone it into the same base directory fpc is checked out/cloned, i.e.
  <base dir>/fpc/compiler
  <base dir>/epiktimer

  As the offical branch requires the use of the classes units, I recommend to
  use my modified version of epiktimer which allows to disable the use of the classes unit,
  this is done automatically by the compiler sources through a define. You can get my epiktimer source from

  https://github.com/FPK/epiktimer.git

  clone them into the same base directory fpc is checked out/cloned, i.e.
  <base dir>/fpc/compiler
  <base dir>/epiktimer

                                                                           (FK)
}
{ $define COMPILER_TIMINGS}

{$i fpcdefs.inc}

interface

  uses
    globals;

  type
    TCTimer = (
      ct_none,
      ct_aopt);

  procedure ResumeTimer(t : TCTimer);
  procedure StopTimer;

implementation

{$ifndef COMPILER_TIMINGS}
  procedure ResumeTimer(t : TCTimer);
    begin
    end;


  procedure StopTimer;
    begin
    end;
{$else COMPILER_TIMINGS}
  uses
    cepiktimer;

  var
    currenttimer : TCTimer;
    timers : array[TCTimer] of TEpikTimer;


  procedure ResumeTimer(t : TCTimer);
    begin
      timers[currenttimer].Stop;
      timers[t].Start;
      currenttimer:=t;
    end;


  procedure StopTimer;
    begin
      timers[currenttimer].Stop;
      currenttimer:=ct_none;
      timers[currenttimer].Start;
    end;

var
  t : TCTimer;

initialization
  for t:=low(timers) to high(timers) do
    timers[t]:=TEpikTimer.Create;
  currenttimer:=ct_none;
finalization
  writeln(StdErr,'Compiler profiler results:');
  timers[currenttimer].Stop;
  for t:=low(timers) to high(timers) do
    writeln(StdErr,'  ',t,' ',timers[t].Elapsed:0:9,' s');
  for t:=low(timers) to high(timers) do
    timers[t].Free;
{$endif COMPILER_TIMINGS}
end.

