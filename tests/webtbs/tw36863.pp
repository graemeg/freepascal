{ %OPT=-Ct -CR }
{$M 65536,65536}

type
  TObj = object
    v: array [0..$2000] of Byte;
    procedure Proc(depth: Integer);
    procedure VProc; virtual;
  end;

  procedure TObj.VProc;
  begin
  end;

  procedure TObj.Proc(depth: Integer);
  begin
    {stack is eaten here on the function entry}
    if (depth < 64) then
      Proc(depth+1);
    {do not actually call the method since the obj is not initialized, just for minimal demonstration}
    if (depth < 0) then
    VProc;
  end;

var
  Obj: TObj;
begin
  Obj.Proc(0);
  writeln('Completed');
end.
