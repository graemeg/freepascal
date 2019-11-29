{ %fail }

{$mode objfpc}

{$ifdef cpu64}
{$r+}
{$else}
{$q+}
{$endif}

function test(l1, l2: longint): longint; inline;
begin
  result:=l1+l2;
end;

{ range checking state at caller site should not influence inline evaluation }
{$ifdef cpu64}
{$r-}
{$else}
{$q-}
{$endif}
begin
  test(high(longint), 1);
end.
