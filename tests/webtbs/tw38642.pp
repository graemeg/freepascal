{ %NORUN }

program tw38642;
{$mode delphi}{$H+}
uses
  classes,
  generics.collections;
type
  TMonthType = (January, February, May=10, June, July);
  TMonthList = TList<TMonthType>;
var
  myList : TMonthList;
begin
end.
