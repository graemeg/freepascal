{ %FAIL }
{ %RECOMPILE }

{$mode objfpc}
{$interfaces corba}

program tw36652;
uses
  uw36652;

type
  TClassB = class
    procedure DoThis;
  end;

// 2014010312
procedure TClassA.DoThis;
begin
end;

begin
end.
