{ %NORUN }

program tb0668b;

{$mode objfpc}

generic procedure FreeAndNil<T: class>(var Obj: T);
begin
end;

procedure FreeAndNil(var Obj);
begin
end;

var
  t: TObject;
begin
  FreeAndNil(t);
  specialize FreeAndNil<TObject>(t);
end.
