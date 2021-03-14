{$mode objfpc}
var
  i, j: Int32;
begin
  for i := 0 to 1 do // Error: Compilation raised exception internally
  try
    j := i;
  finally
    if (j <> 0) then
      j := 0;
  end;
end.
