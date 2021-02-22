{%OPT=-O2}

{$mode objfpc}

function F(n: SizeUint): SizeUint;
begin
    result := 4 * n + 4 * n;
end;

begin
    writeln('Reference F(5): ', 4 * 5 + 4 * 5);
    writeln(' Actual F(5): ', F(5));
    if (F(5) <> 40) then
      halt(1);
end.
