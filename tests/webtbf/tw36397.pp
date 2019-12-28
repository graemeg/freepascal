{ %fail }
program Project1;

{$mode delphi}

type
  TTest = object
    class procedure myproc;
  end;

  class procedure TTest.myproc;
  begin
  end;

var
  p: procedure;
begin
  p := TTest.myproc;
end.

