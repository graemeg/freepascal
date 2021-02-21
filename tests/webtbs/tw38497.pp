program project1;

{$mode delphi}

type
  TAlphabet = (A, B, C);
  TAlphabets = set of TAlphabet;

  procedure Test<TEnum, TSet>(E: TEnum; S: TSet);
  var
    I: TEnum;
    B: Boolean;
  begin
    B := [E] <= S;
    if E in S then
      WriteLn(E);
    for I := Low(TEnum) to High(TEnum) do
      if I in S then
      WriteLn(I);
  end;

begin
  Test<TAlphabet, TAlphabets>(A, [A, B]);
end.
