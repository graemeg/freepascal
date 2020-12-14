{ %result=201 }
program Test;
{$apptype console}
{$ifdef fpc}
{$mode objfpc}
{$endif fpc}
{$R+}

var
  Arr: array[1..2] of integer;
  i: Integer;
begin
  i:=5;
  try
    try
      Arr[i] := 1;
    except
      writeln('Except block');
    end;
  finally
    writeln('Finally block');
  end;
end.
