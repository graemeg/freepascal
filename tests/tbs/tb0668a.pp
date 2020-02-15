{ %NORUN }

program tb0668a;

{$mode objfpc}

procedure FreeAndNil(var Obj);
begin
end;

generic procedure FreeAndNil<T: class>(var Obj: T);
begin
end;

var
  t: TObject;
begin
  FreeAndNil(t);
  specialize FreeAndNil<TObject>(t);
end.
