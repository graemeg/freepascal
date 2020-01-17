{ %NORUN }

program tb0666a;

{$mode delphi}

function Test<T>: T;

  procedure Foo;
  begin
    Test<T>;
    Test<LongInt>;
    Test<String>;
  end;

begin
  Foo;
end;

begin
  Test<LongInt>;
end.
