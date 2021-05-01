program EmptyTryFinally1;

{$mode delphi}
{$apptype console}

var
  finallyrun: boolean;
begin
  finallyrun:=false;
  try
   // Empty try statement block
  finally
    WriteLn('I should actually visible . . .'); // but I'm not
    finallyrun:=true;
  end;
  if not finallyrun then
    halt(1);
end.
