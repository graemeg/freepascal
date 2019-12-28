{ %fail }
{$mode objfpc}

program test;
uses
 FGL;

// Type identifier expected
// Internal error 2019112401
generic function CopyList<T>(source: specialize FGL.TFPGObjectList<T>): specialize FGL.TFPGObjectList<T>;
begin
end;

begin
end.