<<<<<<< HEAD
<<<<<<< HEAD
{%target=win32,win64,wince}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{%cpu=x86_64,i386,arm}
{%result=229}

{$mode objfpc}

procedure DoTest1; safecall;
var
  i: integer;
begin
  i:=-1;
  i:=i - 1;
end;

function DoTest2: longint; safecall;
begin
  DoTest2:=$12345678;
end;

procedure DoTest3; safecall;
begin
  PChar(nil)^:='A';
end;

begin
  try
    DoTest1;
  except
    ErrorAddr:=nil;
    Halt(1);
  end;
  if DoTest2 <> $12345678 then
    Halt(2);
  DoTest3;
end.
