{ %OPT=-Seh }
program Test;

{$HINTS ON}

var
  cur: Currency;
begin
  cur := 3.5;
  cur := cur / 1.5;
end.
