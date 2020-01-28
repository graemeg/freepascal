{ %skipcpu=x86_64,powerpc64,aarch64 }
{ %FAIL }

{$mode delphi}

var
  { The next line should give an error as it's to big to allocate }
  a: array [ Integer ] of Integer;
  i: Integer;

begin

  for i := 0 to 1000 do begin
    a [ i ] := i
  end { for i }

; writeln ( a [ 1000 ] )
end.
