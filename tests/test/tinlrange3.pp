{ %fail }

{$mode objfpc}
{$q+}

function test(l1, l2: int64): int64; inline;
begin
  result:=l1+l2;
end;

{$q-}

begin
  test(high(int64), 1);
end.
