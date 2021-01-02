{ %FAIL }

program tgenfunc25;

{$mode delphi}

type
  TTest = class
  public
    procedure Test<T>;
  public type
    Test = class
    end;
  end;

procedure TTest.Test<T>;
begin

end;

begin

end.

