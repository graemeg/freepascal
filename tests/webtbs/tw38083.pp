{ %NORUN }

program tw38083;

{$MODE Delphi}

  procedure Test<T>(A: T; B: Boolean); overload;
  begin

  end;

  procedure Test(A: String); overload;
  begin
    Test<String>(A, True);
  end;

begin

end.
