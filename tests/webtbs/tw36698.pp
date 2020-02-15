program map_test;

{$mode objfpc}

uses
  SysUtils, ghashmap;

const
  TestSize = 270000;

type

  THash = class
    class function Hash(aValue: Integer; n: SizeUInt): SizeUInt; static;
  end;
  TMap = specialize THashmap<Integer, Integer, THash>;


class function THash.Hash(aValue: Integer; n: SizeUInt): SizeUInt;
begin
  aValue := aValue xor aValue shr 20 xor aValue shr 12;
  Result := SizeUInt(aValue xor aValue shr 7 xor aValue shr 4) and (n-1);
end;

procedure Test;
var
  Map: TMap;
  I: Integer;
begin
  Map := TMap.Create;
  try
    for I := 1 to TestSize do
      Map.Insert(I, I);
  finally
    Map.Free;
  end;
end;

begin
  Test;
end.
