{ %target=win32,wince,win64 }
{ %opt=-gh }
{$AppType CONSOLE}
uses
  popuperr,
  Windows;
{$C+}


var
  dll1, dll2: HModule;

<<<<<<< HEAD
function T1(Parameter: Pointer): PtrInt;
=======
function T1(Parameter: Pointer): LongInt;
>>>>>>> graemeg/cpstrnew
begin
  //Sleep(100);
end;

<<<<<<< HEAD
function T2(Parameter: Pointer): PtrInt;
=======
function T2(Parameter: Pointer): LongInt;
>>>>>>> graemeg/cpstrnew
begin
  //Sleep(100);
end;

var
  h: array[0..1] of THandle;
<<<<<<< HEAD
  id1, id2: TThreadID;
=======
  id1, id2: DWORD;
>>>>>>> graemeg/cpstrnew
  p : pointer;
begin
  IsMultiThread:=True;
  dll1:=LoadLibrary('tw14992a.dll');
  dll2:=LoadLibrary('tw14992b.dll');
  h[0]:=BeginThread(nil, 0, @T1, nil, 0, id1);
  h[1]:=BeginThread(nil, 0, @T2, nil, 0, id2);
  WaitForMultipleObjects(Length(h), @h[0], true, infinite);
  FreeLibrary(dll2);
  FreeLibrary(dll1);
end.
