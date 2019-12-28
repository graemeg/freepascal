{ %fail }
{ %OPT=-Fcutf8 }
program cps;

{$mode objfpc}
{$h+}

type
  Utf7String = type AnsiString(CP_UTF7);

var
  U7: Utf7String;

begin
  U7 := 'U7';
end.
