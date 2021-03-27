{ %FAIl }

{ outside of generics TypeInfo() of types without type information (e.g. enums
  with holes) throws a compile error }

program trtti22;

{$mode objfpc}

type
  TEnum = (teOne = 1, teTwo);

var
  p: Pointer;
begin
  p := TypeInfo(TEnum);
end.
