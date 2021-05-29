{ %fail }
program Project1;
type
   T1 = array of integer;
   T2 = array of T1;
   T3 = array of T2;
var
a: T3;
begin
   SetLength(a.[0].[0],1);
end.
