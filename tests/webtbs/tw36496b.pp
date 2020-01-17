{ %NORUN }

(*
  testing application for
  https://forum.lazarus.freepascal.org/index.php/topic,47936.0.html
*)
program tw36496b;

{$Mode objfpc}{$H+}

generic function TestGenRecurse<T>(const AInput : T) : Boolean;
begin
  //Result := False;

  (*
    below, if uncommented will fail to compile
    tester.lpr(12,19) Error: Identifier not found "TestGenRecurse$1"
  *)
  specialize TestGenRecurse<T>(AInput);
  specialize TestGenRecurse<String>('test');
  specialize TestGenRecurse<LongInt>(42);
end;

generic procedure TestGenRecurseProc<T>(const AInput : T);
begin
  (*
    below method calls compile fine
  *)
  specialize TestGenRecurseProc<T>(AInput);
  specialize TestGenRecurseProc<String>('test');
  specialize TestGenRecurseProc<LongInt>(42);
end;

begin
  specialize TestGenRecurse<String>('testing');
  specialize TestGenRecurseProc<String>('testing');
end.

