var
  s: set of 3..40;
begin
  if (low(s)<>3) or
     (high(s)<>40) then
    halt(1);
end.
