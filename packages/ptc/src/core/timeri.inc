{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2003, 2006, 2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
  TPTCTimer = class(TInterfacedObject, IPTCTimer)
  private
    FOld: Double;
    FTime: Double;
    FStart: Double;
    FCurrent: Double;
    FRunning: Boolean;
    {$IF defined(WIN32) OR defined(WIN64)}
    FFrequency: QWord;
    {$ENDIF defined(WIN32) OR defined(WIN64)}
    function Clock: Double;
    procedure internal_init_timer;
  public
    constructor Create;
    constructor Create(ATime: Double);
//    constructor Create(ATimer: IPTCTimer);
    destructor Destroy; override;
{    procedure Assign(const ATimer: TPTCTimer);
    function Equals(const ATimer: TPTCTimer): Boolean;}
    procedure SetTime(ATime: Double); {was 'set' in the C++ version}
    procedure Start;
    procedure Stop;
    function Time: Double;
    function Delta: Double;
    function Resolution: Double;
  end;

class function TPTCTimerFactory.CreateNew: IPTCTimer;
begin
  Result := TPTCTimer.Create;
end;

class function TPTCTimerFactory.CreateNew(ATime: Double): IPTCTimer;
begin
  Result := TPTCTimer.Create(ATime);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{Function timeGetTime: DWord; StdCall; external 'WINMM' name 'timeGetTime';}

constructor TPTCTimer.Create;
begin
  internal_init_timer;
  FOld := 0;
  FTime := 0;
  FStart := 0;
  FCurrent := 0;
  FRunning := False;
end;

constructor TPTCTimer.Create(ATime: Double);
begin
  internal_init_timer;
  FOld := 0;
  FTime := 0;
  FStart := 0;
  FCurrent := 0;
  FRunning := False;
  SetTime(ATime);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{constructor TPTCTimer.Create(ATimer: IPTCTimer);
begin
  internal_init_timer;

  FOld := ATimer.FOld;
  FTime := ATimer.FTime;
  FStart := ATimer.FStart;
  FCurrent := ATimer.FCurrent;
  FRunning := ATimer.FRunning;
end;
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TPTCTimer.Create(const ATimer: TPTCTimer);
begin
  internal_init_timer;
  Assign(ATimer);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
destructor TPTCTimer.Destroy;
begin
  Stop;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{procedure TPTCTimer.Assign(const ATimer: TPTCTimer);
=======
procedure TPTCTimer.Assign(const ATimer: TPTCTimer);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCTimer.Assign(const ATimer: TPTCTimer);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCTimer.Assign(const ATimer: TPTCTimer);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCTimer.Assign(const ATimer: TPTCTimer);
>>>>>>> origin/cpstrnew
begin
  if Self = ATimer then
    exit;

  FOld := ATimer.FOld;
  FTime := ATimer.FTime;
  FStart := ATimer.FStart;
  FCurrent := ATimer.FCurrent;
  FRunning := ATimer.FRunning;
end;

function TPTCTimer.Equals(const ATimer: TPTCTimer): Boolean;
begin
  Result := (FOld = ATimer.FOld) and (FTime = ATimer.FTime) and
            (FStart = ATimer.FStart) and (FCurrent = ATimer.FCurrent) and
            (FRunning = ATimer.FRunning);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
procedure TPTCTimer.SetTime(ATime: Double);
begin
  FCurrent := ATime;
  FStart := Clock;
  FTime := FStart + ATime;
  FOld := FTime - Delta;
end;

procedure TPTCTimer.Start;
begin
  if not FRunning then
  begin
    FStart := Clock;
    FOld := Clock;
    FRunning := True;
  end;
end;

procedure TPTCTimer.Stop;
begin
  FRunning := False;
end;

function TPTCTimer.Time: Double;
var
  _time: Double;
begin
  if FRunning then
  begin
    _time := Clock;
    if _time > FTime then
      FTime := _time;
    FCurrent := FTime - FStart;
  end;
  Result := FCurrent;
end;

function TPTCTimer.Delta: Double;
var
  _time: Double;
  _delta: Double;
begin
  if FRunning then
  begin
    _time := Clock;
    _delta := _time - FOld;
    FOld := _time;
    if _delta < 0 then
      _delta := 0;
    Result := _delta;
  end
  else
    Result := 0;
end;

function TPTCTimer.Resolution: Double;
begin
  {$IFDEF GO32V2}
  Result := TimerResolution;
  {$ENDIF GO32V2}
  {$IF defined(Win32) OR defined(Win64)}
  Result := 1 / FFrequency;
{  Result := 1 / 1000;}
  {$ENDIF defined(Win32) OR defined(Win64)}
  {$IFDEF WinCE}
  Result := 1 / 1000;
  {$ENDIF WinCE}
  {$IFDEF UNIX}
  Result := 1 / 1000000;
  {$ENDIF UNIX}
end;

procedure TPTCTimer.internal_init_timer;
{$IF defined(WIN32) OR defined(WIN64)}
var
  _freq: QWord;
{$ENDIF defined(WIN32) OR defined(WIN64)}
begin
{$IF defined(WIN32) OR defined(WIN64)}
  QueryPerformanceFrequency(PLARGE_INTEGER(@_freq));
  FFrequency := _freq;
{$ENDIF defined(WIN32) OR defined(WIN64)}
end;

{$IFDEF GO32V2}
function TPTCTimer.Clock: Double;
begin
  Result := GetClockTics() * TimerResolution;
end;
{$ENDIF GO32V2}

{$IF defined(WIN32) OR defined(WIN64)}
function TPTCTimer.Clock: Double;
var
  _time: QWord;
begin
  QueryPerformanceCounter(PLARGE_INTEGER(@_time));
  Result := _time / FFrequency;
{  Result := timeGetTime / 1000;}
end;
{$ENDIF defined(WIN32) OR defined(WIN64)}

{$IFDEF WinCE}
function TPTCTimer.Clock: Double;
begin
  Result := GetTickCount / 1000;
end;
{$ENDIF WinCE}

{$IFDEF UNIX}
function TPTCTimer.Clock: Double;
var
  tm: TimeVal;
begin
  fpGetTimeOfDay(@tm, nil);
  Result := tm.tv_sec + (Double(tm.tv_usec)) / 1000000;
end;
{$ENDIF UNIX}
