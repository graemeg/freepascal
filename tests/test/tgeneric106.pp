program tgeneric106;

{$Mode Delphi}

type G<T> = class
    var X: T;
    // EXPECTED: gets compiled
    // ACTUAL: 'Error: Generics without specialization cannot be used as a type for a variable'
    class var F: function(const X: T) : G<T> of object;
    function Foo(const X: T): G<T>;
end;

function G<T>.Foo(const X: T): G<T>;
begin
    result := G<T>.Create;
    result.X := X
end;

begin
    G<Integer>.F := G<Integer>.Create.Foo;
    if G<Integer>.F(42).X <> 42 then
      halt(1);
end.
