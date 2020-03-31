unit tgenfunc20;

{$mode objfpc}{$H+}

interface

{generic procedure TestProc1<T: class>;

type
  TTest = class
    generic procedure Test<T: class>;
  end;}

implementation

generic procedure TestProc2<T: class>; forward;

{generic procedure TestProc1<T>;
begin
end;

generic procedure TestProc1<T: class>(aArg1: T);
begin
end;}

generic procedure TestProc2<T: class>;
begin
end;

{generic procedure TTest.Test<T>;
begin
end;}

end.

