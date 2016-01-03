program test_varbyref;

{$mode objfpc}{$H+}
{$apptype console}

uses
<<<<<<< HEAD
  Variants;
=======
  Variants, VarUtils;
>>>>>>> origin/fixes_2.4

var
  V: Variant;
  P: Integer;
begin
  P := 1;
  TVarData(V).vtype := varbyref or varinteger;
  TVarData(V).vpointer := @P;
<<<<<<< HEAD
  WriteLn(string(V));
  WriteLn(SmallInt(V));
  WriteLn(Longint(V));
  WriteLn(ShortInt(V));
  WriteLn(Cardinal(V));
  WriteLn(Single(V));
  WriteLn(Double(V));
  WriteLn(TDateTime(V));
  WriteLn(Currency(V));
  WriteLn(Boolean(V));
  WriteLn(Byte(V));
  WriteLn(Int64(V));
  WriteLn(QWord(V));
  WriteLn(WideString(V));
  WriteLn(ShortString(V));
=======
  WriteLn(VariantToAnsiString(TVarData(V)));
  WriteLn(VariantToSmallInt(TVarData(V)));
  WriteLn(VariantToLongint(TVarData(V)));
  WriteLn(VariantToShortint(TVarData(V)));
  WriteLn(VariantToCardinal(TVarData(V)));
  WriteLn(VariantToSingle(TVarData(V)));
  WriteLn(VariantToDouble(TVarData(V)));
  WriteLn(VariantToDate(TVarData(V)));
  WriteLn(VariantToCurrency(TVarData(V)));
  WriteLn(VariantToBoolean(TVarData(V)));
  WriteLn(VariantToByte(TVarData(V)));
  WriteLn(VariantToInt64(TVarData(V)));
  WriteLn(VariantToQWord(TVarData(V)));
  WriteLn(VariantToWideString(TVarData(V)));
  WriteLn(VariantToAnsiString(TVarData(V)));
  WriteLn(VariantToShortString(TVarData(V)));
>>>>>>> origin/fixes_2.4
  WriteLn(V);
  TVarData(V).vtype := varEmpty;
  TVarData(V).vpointer := nil;
end.

