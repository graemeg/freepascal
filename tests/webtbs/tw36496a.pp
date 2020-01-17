{ %NORUN }

(*
  testing application for
  https://forum.lazarus.freepascal.org/index.php/topic,47936.0.html
*)
program tw36496a;

{$Mode delphi}

function TestGenRecurse<T>(const AInput : T) : Boolean;
begin
  //Result := False;

  (*
    below, if uncommented will fail to compile
    tester.lpr(12,19) Error: Identifier not found "TestGenRecurse$1"
  *)
  TestGenRecurse<T>(AInput);
  TestGenRecurse<String>('test');
  TestGenRecurse<LongInt>(42);
end;

procedure TestGenRecurseProc<T>(const AInput : T);
begin
  (*
    below method calls compile fine
  *)
  TestGenRecurseProc<T>(AInput);
  TestGenRecurseProc<String>('test');
  TestGenRecurseProc<LongInt>(42);
end;

begin
  TestGenRecurse<String>('testing');
  TestGenRecurseProc<String>('testing');
end.

