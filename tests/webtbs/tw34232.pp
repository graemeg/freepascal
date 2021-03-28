program Project1;
{$mode objfpc}{$H+}
type
generic TTest<TKey, TValue> = packed object
    type
    TPair = packed record
      key: TKey;
      value: TValue;
    end;
    TPairSizeEquivalent = packed array[1..sizeof(TPair)] of byte;
end;
TTestStringString = specialize TTest<string, string>;

begin
  writeln(sizeof(TTestStringString.TPairSizeEquivalent));
end.
