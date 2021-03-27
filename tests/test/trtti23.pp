{ %FAIl }

{ inside of generics TypeInfo() of types without type information (e.g. enums
  with holes) that are not generic parameters throws a compile error }

program trtti23;

{$mode objfpc}

type
  TEnum = (teOne = 1, teTwo);

  generic TTest<T> = class
    procedure Test;
  end;

{ TTest }

procedure TTest.Test;
var
  ti: Pointer;
begin
  ti := TypeInfo(TEnum);
end;

begin
end.
