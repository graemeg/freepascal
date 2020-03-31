unit tgenfunc21;

{$mode delphi}

interface

procedure TestProc1<T: class>; overload;

type
  TTest = class
    procedure Test<T: class>;
  end;

implementation

procedure TestProc2<T: class>; forward;

procedure TestProc1<T>;
begin
end;

procedure TestProc1<T: class>(aArg1: T); overload;
begin
end;

procedure TestProc2<T: class>;
begin
end;

procedure TTest.Test<T>;
begin
end;

end.

