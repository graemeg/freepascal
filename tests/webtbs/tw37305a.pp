{$mode objfpc}
var
  i, j, c: Int32;
begin
  c := 1;
  for i := 0 to c do // SIGSEGV at runtime
  try
    j := i;
  finally
    if (j <> 0) then
      j := 0;
  end;
end.
