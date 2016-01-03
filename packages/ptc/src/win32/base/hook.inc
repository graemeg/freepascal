{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> graemeg/fixes_2_2
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> origin/fixes_2_2

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
{$WARNING this has to go to the windows unit}
const
  GWLP_WNDPROC = GWL_WNDPROC;

type
  PWin32Hook_Lookup = ^TWin32Hook_Lookup;
  TWin32Hook_Lookup = record
    window: HWND;
    wndproc: PtrUInt;
    hook: array [0..15] of TWin32Hook;
    count: Integer;
  end;

const
  TWin32Hook_Count: Integer = 0;
  TWin32Hook_Cached: PWin32Hook_Lookup = nil;
  TWin32Hook_Monitor: TWin32Monitor = nil;

var
{  TWin32Hook_m_hook: HHOOK;}
  TWin32Hook_m_registry: array [0..15] of TWin32Hook_Lookup;

function TWin32Hook_hook(hwnd: HWND; msg: DWord; wParam: WPARAM; lParam: LPARAM): LRESULT; stdcall;
var
  lookup: PWin32Hook_Lookup;
  i: Integer;
begin
  { enter monitor }
  TWin32Hook_Monitor.Enter;

  { lookup pointer }
  lookup := nil;

  { check cached lookup if valid }
  if (TWin32Hook_Cached <> nil) and (TWin32Hook_Cached^.window = hwnd) then
    { cached lookup match }
    lookup := TWin32Hook_Cached
  else
  begin
    { search for matching window }
    for i := 0 to TWin32Hook_Count - 1 do
      { check for lookup window match }
      if TWin32Hook_m_registry[i].window = hwnd then
      begin
        { setup cached lookup }
        TWin32Hook_Cached := @TWin32Hook_m_registry[i];

        { setup lookup }
        lookup := TWin32Hook_Cached;

        { break }
        break;
      end;
{$IFDEF DEBUG}
    { check for search failure }
    if lookup = nil then
      raise TPTCError.Create('TWin32Hook window lookup search failure!');
{$ENDIF}
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  PWin32Hook_Lookup = ^TWin32Hook_Lookup;
  TWin32Hook_Lookup = Record
    window : HWND;
    wndproc : DWord;
    hook : Array[0..15] Of TWin32Hook;
    count : Integer;
  End;

Const
  TWin32Hook_m_count : Integer = 0;
  TWin32Hook_m_cached : PWin32Hook_Lookup = Nil;
  TWin32Hook_m_monitor : TWin32Monitor = Nil;

Var
{  TWin32Hook_m_hook : HHOOK;}
  TWin32Hook_m_registry : Array[0..15] Of TWin32Hook_Lookup;

Function TWin32Hook_hook(hwnd : HWND; msg : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT; StdCall;

Var
  lookup : PWin32Hook_Lookup;
  i : Integer;

Begin
  { enter monitor }
  TWin32Hook_m_monitor.enter;

  { lookup pointer }
  lookup := Nil;

  { check cached lookup if valid }
  If (TWin32Hook_m_cached <> Nil) And (TWin32Hook_m_cached^.window = hwnd) Then
    { cached lookup match }
    lookup := TWin32Hook_m_cached
  Else
  Begin
    { search for matching window }
    For i := 0 To TWin32Hook_m_count - 1 Do
      { check for lookup window match }
      If TWin32Hook_m_registry[i].window = hwnd Then
      Begin
        { setup cached lookup }
        TWin32Hook_m_cached := @TWin32Hook_m_registry[i];

        { setup lookup }
        lookup := TWin32Hook_m_cached;

        { break }
        Break;
      End;
{$IFDEF DEBUG}
    { check for search failure }
    If lookup = Nil Then
      Raise TPTCError.Create('TWin32Hook window lookup search failure!');
{$ENDIF}
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { result value }
  TWin32Hook_hook := 0;

  { iterate all hooks for this window }
<<<<<<< HEAD
<<<<<<< HEAD
  for i := lookup^.count - 1 downto 0 do
  begin
    { call hook window procedure }
    Result := lookup^.hook[i].WndProc(hwnd, msg, wParam, lParam);

    { check result value ? }
    if result <> 0 then
      break;
  end;

  { check result }
  if result = 0 then
  begin
    { call original window procedure }
    Result := CallWindowProc(WNDPROC(lookup^.wndproc), hwnd, msg, wParam, lParam);
  end;

  { leave monitor }
  TWin32Hook_Monitor.Leave;
end;

constructor TWin32Hook.Create(AWindow: HWND; AThread: DWord);
begin
  { setup data }
  FWindow := AWindow;
  FThread := AThread;

  { add to registry }
  Add(FWindow, FThread);
end;

destructor TWin32Hook.Destroy;
begin
  { remove from registry }
  Remove(FWindow, FThread);
  inherited Destroy;
end;

procedure TWin32Hook.Add(AWindow: HWND; AThread: DWord);
var
  index, insert: Integer;
begin
  { enter monitor }
  TWin32Hook_Monitor.Enter;

  { invalidate cache }
  TWin32Hook_Cached := nil;
=======
=======
>>>>>>> origin/fixes_2_2
  For i := lookup^.count - 1 DownTo 0 Do
  Begin
    { call hook window procedure }
    TWin32Hook_hook := lookup^.hook[i].WndProc(hwnd, msg, wParam, lParam);

    { check result value ? }
    {If result = True Then Break;}
  End;

  { check result }
  {If result <> True Then}

  { call original window procedure }
  result := CallWindowProc(WNDPROC(lookup^.wndproc), hwnd, msg, wParam, lParam);

  { leave monitor }
  TWin32Hook_m_monitor.leave;
End;

Constructor TWin32Hook.Create(window : HWND; thread : DWord);

Begin
  { setup data }
  m_window := window;
  m_thread := thread;

  { add to registry }
  add(m_window, m_thread);
End;

Destructor TWin32Hook.Destroy;

Begin
  { remove from registry }
  remove(m_window, m_thread);
  Inherited Destroy;
End;

Procedure TWin32Hook.Add(window : HWND; thread : DWord);

Var
  index, insert : Integer;

Begin
  { enter monitor }
  TWin32Hook_m_monitor.enter;

  { invalidate cache }
  TWin32Hook_m_cached := Nil;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { registry index }
  index := 0;

  { iterate registry }
<<<<<<< HEAD
<<<<<<< HEAD
  while index < TWin32Hook_Count do
  begin
    { search for existing window hook }
    if TWin32Hook_m_registry[index].window = AWindow then
    { match }
      break;

    { next }
    Inc(index);
  end;

  { check results }
  if index <> TWin32Hook_Count then
  begin
=======
=======
>>>>>>> origin/fixes_2_2
  While index < TWin32Hook_m_count Do
  Begin
    { search for existing window hook }
    If TWin32Hook_m_registry[index].window = window Then
    { match }
      Break;

    { next }
    Inc(index);
  End;

  { check results }
  If index <> TWin32Hook_m_count Then
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { get insertion point for hook }
    insert := TWin32Hook_m_registry[index].count;

    { increase hook count }
    Inc(TWin32Hook_m_registry[index].count);

{$IFDEF DEBUG}
    { Check for maximum hook count }
<<<<<<< HEAD
<<<<<<< HEAD
    if TWin32Hook_m_registry[index].count > (High(TWin32Hook_m_registry[index].hook) + 1) then
      raise TPTCError.Create('TWin32Hook too many hooks created!');
=======
    If TWin32Hook_m_registry[index].count > (High(TWin32Hook_m_registry[index].hook) + 1) Then
      Raise TPTCError.Create('TWin32Hook too many hooks created!');
>>>>>>> graemeg/fixes_2_2
=======
    If TWin32Hook_m_registry[index].count > (High(TWin32Hook_m_registry[index].hook) + 1) Then
      Raise TPTCError.Create('TWin32Hook too many hooks created!');
>>>>>>> origin/fixes_2_2
{$ENDIF}

    { insert hook in registry }
    TWin32Hook_m_registry[index].hook[insert] := Self;
<<<<<<< HEAD
<<<<<<< HEAD
  end
  else
  begin
    { setup new lookup }
    TWin32Hook_m_registry[index].wndproc := GetWindowLongPtr(AWindow, GWLP_WNDPROC);
    TWin32Hook_m_registry[index].window := AWindow;
=======
=======
>>>>>>> origin/fixes_2_2
  End
  Else
  Begin
    { setup new lookup }
    TWin32Hook_m_registry[index].wndproc := GetWindowLong(window, GWL_WNDPROC);
    TWin32Hook_m_registry[index].window := window;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    TWin32Hook_m_registry[index].hook[0] := Self;
    TWin32Hook_m_registry[index].count := 1;

    { increase lookup count }
<<<<<<< HEAD
<<<<<<< HEAD
    Inc(TWin32Hook_Count);

{$IFDEF DEBUG}
    { check for maximum count }
    if TWin32Hook_Count > (High(TWin32Hook_m_registry) + 1) then
      raise TPTCError.Create('TWin32Hook too many lookups created!');
{$ENDIF}

    { set window procedure to hook procedure }
    SetWindowLongPtr(AWindow, GWLP_WNDPROC, PtrInt(@TWin32Hook_hook));
  end;

  { leave monitor }
  TWin32Hook_Monitor.Leave;
end;

procedure TWin32Hook.Remove(AWindow: HWND; AThread: DWord);
var
  index, i, j: Integer;
begin
  { enter monitor }
  TWin32Hook_Monitor.Enter;

  { invalidate cache }
  TWin32Hook_Cached := nil;
=======
=======
>>>>>>> origin/fixes_2_2
    Inc(TWin32Hook_m_count);

{$IFDEF DEBUG}
    { check for maximum count }
    If TWin32Hook_m_count > (High(TWin32Hook_m_registry) + 1) Then
      Raise TPTCError.Create('TWin32Hook too many lookups created!');
{$ENDIF}

    { set window procedure to hook procedure }
    SetWindowLong(window, GWL_WNDPROC, DWord(@TWin32Hook_hook));
  End;

  { leave monitor }
  TWin32Hook_m_monitor.leave;
End;

Procedure TWin32Hook.Remove(window : HWND; thread : DWord);

Var
  index, i, j : Integer;

Begin
  { enter monitor }
  TWin32Hook_m_monitor.enter;

  { invalidate cache }
  TWin32Hook_m_cached := Nil;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { registry index }
  index := 0;

  { iterate registry }
<<<<<<< HEAD
<<<<<<< HEAD
  while index < TWin32Hook_Count do
  begin
    { check for window match }
    if TWin32Hook_m_registry[index].window = AWindow then
    begin
      { search for Self }
      for i := 0 to TWin32Hook_m_registry[index].count do
        { check hook }
        if TWin32Hook_m_registry[index].hook[i] = Self then
        begin
          { remove this hook (quite inefficient for high count...) }
          for j := i to TWin32Hook_m_registry[index].count - 2 do
=======
=======
>>>>>>> origin/fixes_2_2
  While index < TWin32Hook_m_count Do
  Begin
    { check for window match }
    If TWin32Hook_m_registry[index].window = window Then
    Begin
      { search for Self }
      For i := 0 To TWin32Hook_m_registry[index].count Do
        { check hook }
        If TWin32Hook_m_registry[index].hook[i] = Self Then
        Begin
          { remove this hook (quite inefficient for high count...) }
          For j := i To TWin32Hook_m_registry[index].count - 2 Do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            TWin32Hook_m_registry[index].hook[j] :=
              TWin32Hook_m_registry[index].hook[j + 1];

          { decrease hook count }
          Dec(TWin32Hook_m_registry[index].count);

          { break }
<<<<<<< HEAD
<<<<<<< HEAD
          break;
        end;

      { check remaining hook count }
      if TWin32Hook_m_registry[index].count = 0 then
      begin
        { restore original window procedure }
        SetWindowLongPtr(AWindow, GWLP_WNDPROC, TWin32Hook_m_registry[i].wndproc);

        { remove this lookup (quite inefficient for high count...) }
        for i := index to TWin32Hook_Count - 2 do
          TWin32Hook_m_registry[i] := TWin32Hook_m_registry[i + 1];

        { decrease count }
        Dec(TWin32Hook_Count);
      end;

      { break }
      break;
    end;

    { next }
    Inc(index);
  end;

  { leave monitor }
  TWin32Hook_Monitor.Leave;
end;
=======
=======
>>>>>>> origin/fixes_2_2
          Break;
        End;

      { check remaining hook count }
      If TWin32Hook_m_registry[index].count = 0 Then
      Begin
        { restore original window procedure }
        SetWindowLong(window, GWL_WNDPROC, TWin32Hook_m_registry[i].wndproc);

        { remove this lookup (quite inefficient for high count...) }
        For i := index To TWin32Hook_m_count - 2 Do
          TWin32Hook_m_registry[i] := TWin32Hook_m_registry[i + 1];

        { decrease count }
        Dec(TWin32Hook_m_count);
      End;

      { break }
      Break;
    End;

    { next }
    Inc(index);
  End;

  { leave monitor }
  TWin32Hook_m_monitor.leave;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
