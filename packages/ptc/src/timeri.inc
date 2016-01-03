{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{Function timeGetTime : DWord; StdCall; External 'WINMM' name 'timeGetTime';}

Constructor TPTCTimer.Create;

Begin
  internal_init_timer;
  m_old := 0;
  m_time := 0;
  m_start := 0;
  m_current := 0;
  m_running := False;
End;

Constructor TPTCTimer.Create(_time : Double);

Begin
  internal_init_timer;
  m_old := 0;
  m_time := 0;
  m_start := 0;
  m_current := 0;
  m_running := False;
  settime(_time);
End;

Constructor TPTCTimer.Create(Const timer : TPTCTimer);

Begin
  internal_init_timer;
  Assign(timer);
End;

Destructor TPTCTimer.Destroy;

Begin
  stop;
  Inherited Destroy;
End;

Procedure TPTCTimer.Assign(Const timer : TPTCTimer);

Begin
  If Self = timer Then
    Raise TPTCError.Create('self assignment is not allowed');

  m_old := timer.m_old;
  m_time := timer.m_time;
  m_start := timer.m_start;
  m_current := timer.m_current;
  m_running := timer.m_running;
End;

Function TPTCTimer.Equals(Const timer : TPTCTimer) : Boolean;

Begin
  Equals := (m_old = timer.m_old) And (m_time = timer.m_time) And
            (m_start = timer.m_start) And (m_current = timer.m_current) And
            (m_running = timer.m_running);
End;

Procedure TPTCTimer.settime(_time : Double);

Begin
  m_current := _time;
  m_start := clock;
  m_time := m_start + _time;
  m_old := m_time - delta;
End;

Procedure TPTCTimer.start;

Begin
  If Not m_running Then
  Begin
    m_start := clock;
    m_old := clock;
    m_running := True;
  End;
End;

Procedure TPTCTimer.stop;

Begin
  m_running := False;
End;

Function TPTCTimer.time : Double;

Var
  _time : Double;

Begin
  If m_running Then
  Begin
    _time := clock;
    If _time > m_time Then
      m_time := _time;
    m_current := m_time - m_start;
  End;
  time := m_current;
End;

Function TPTCTimer.delta : Double;

Var
  _time : Double;
  _delta : Double;

Begin
  If m_running Then
  Begin
    _time := clock;
    _delta := _time - m_old;
    m_old := _time;
    If _delta < 0 Then
      _delta := 0;
    delta := _delta;
  End
  Else
    delta := 0;
End;

Function TPTCTimer.resolution : Double;

Begin
  {$IFDEF GO32V2}
  Result := TimerResolution;
  {$ENDIF GO32V2}
  {$IFDEF Win32}
  Result := 1 / m_frequency;
{  Result := 1 / 1000;}
  {$ENDIF Win32}
  {$IFDEF WinCE}
  Result := 1 / 1000;
  {$ENDIF WinCE}
  {$IFDEF UNIX}
  Result := 1 / 1000000;
  {$ENDIF UNIX}
End;

Procedure TPTCTimer.internal_init_timer;

{$IFDEF WIN32}
Var
  _freq : QWord;
{$ENDIF WIN32}

Begin
{$IFDEF WIN32}
  QueryPerformanceFrequency(PLARGE_INTEGER(@_freq));
  m_frequency := _freq;
{$ENDIF WIN32}
End;

{$IFDEF GO32V2}
Function TPTCTimer.clock : Double;

Begin
  clock := GetClockTics() * TimerResolution;
End;
{$ENDIF GO32V2}

{$IFDEF Win32}
Function TPTCTimer.clock : Double;

Var
  _time : QWord;

Begin
  QueryPerformanceCounter(PLARGE_INTEGER(@_time));
  clock := _time / m_frequency;
{  clock := timeGetTime / 1000;}
End;
{$ENDIF Win32}

{$IFDEF WinCE}
Function TPTCTimer.clock : Double;

Begin
  Result := GetTickCount / 1000;
End;
{$ENDIF WinCE}

{$IFDEF UNIX}
Function TPTCTimer.clock : Double;

Var
  tm : TimeVal;

Begin
  fpGetTimeOfDay(@tm, Nil);
  clock := tm.tv_sec + (Double(tm.tv_usec)) / 1000000;
End;
{$ENDIF UNIX}

