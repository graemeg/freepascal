program tb0665;

{$mode objfpc}
{$modeswitch advancedrecords}

type
  TTest = record
    b: Boolean;
    function Test(aArg: Pointer): Boolean; inline;
    generic function Test<T>: Boolean; inline;
  end;

function TTest.Test(aArg: Pointer): Boolean;
begin
  b := True;
  Result := True;
end;

generic function TTest.Test<T>: Boolean;
begin
  Result := Test(Nil);
end;

var
  t: TTest;
begin
  t.b := False;
  { check for side effects to ensure that the code was correctly generated }
  t.specialize Test<LongInt>;
  if not t.b then
    Halt(1);
  Writeln('ok');
end.
