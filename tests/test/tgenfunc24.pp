{ %FAIL }

program tgenfunc24;

{$mode delphi}

type
  TTest = class
  public type
    Test = class
    end;

  public
    procedure Test<T>;
  end;

procedure TTest.Test<T>;
begin

end;

begin

end.

