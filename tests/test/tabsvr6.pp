{$mode objfpc}
{$B+}
uses
  sysutils;
var
  { absolute means volatile, so the expression below cannot be optimized 
    and must cause a sig fault }
  a : longint absolute 0;  
begin
  try
    while true or (a=0) do
      break;
  except
    writeln('ok');
    halt(0);
  end;
  halt(1);
end.
