program tgeneric107;

{$Mode ObjFpc}

type generic G<T> = class
    var X: T;
    // EXPECTED: gets compiled
    // ACTUAL: 'Error: Generics without specialization cannot be used as a type for a variable'
    class var F: function(const X: T) : specialize G<T> of object;
    function Foo(const aX: T): specialize G<T>;
end;

function G.Foo(const aX: T): specialize G<T>;
begin
    result := specialize G<T>.Create;
    result.X := aX
end;

begin
    specialize G<Integer>.F := @specialize G<Integer>.Create.Foo;
    if specialize G<Integer>.F(42).X <> 42 then
      halt(1);
end.
