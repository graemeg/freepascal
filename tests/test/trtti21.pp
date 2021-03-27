{ GetTypeKind() of an enumeration with holes returns tkEnumeration and
  TypeInfo() returns Nil, but *only* inside a generic/specialization when used
  with a generic parameter }

program trtti21;

{$mode objfpc}

type
  TEnum = (teOne = 1, teTwo);

  generic TTest<T> = class
    class function Test: Pointer;
  end;

class function TTest.Test: Pointer;
begin
  Result := TypeInfo(T);
end;

begin
  if GetTypeKind(TEnum) <> tkEnumeration then
    Halt(1);
  if specialize TTest<TEnum>.Test <> Nil then
    Halt(2);
end.
