{ %NORUN }

program tb0666b;

{$mode objfpc}

generic function Test<T>: T;

  procedure Foo;
  begin
    specialize Test<T>;
    specialize Test<LongInt>;
    specialize Test<String>;
  end;

begin
  Foo;
end;

begin
  specialize Test<LongInt>;
end.
