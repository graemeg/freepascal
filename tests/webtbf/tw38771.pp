{ %FAIL }
{$mode objfpc}

program tw38771;

type
  TMyClass = class
    generic procedure DoThis<T>(msg: T);
    generic procedure DoThat<T>(msg: T); virtual;
  end;

generic procedure TMyClass.DoThis<T>(msg:T);
begin
  specialize DoThat<T>(msg);
end;

generic procedure TMyClass.DoThat<T>(msg: T);
begin
end;

begin
end.
