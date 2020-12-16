program tb0681;

{$Mode Delphi}

type R = record
    var X: Integer;
    function Foo: Integer;
end;

function R.Foo: Integer;
begin
    result := X
end;

var    F: function : Integer of object;
    Z: R = (X:42);
begin
    // EXPECTED: gets compiled
    // ACTUAL: 'Error: Incompatible types'
    F := Z.Foo;
    if F() <> 42 then
      Halt(1);
end.
