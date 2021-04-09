{ %opt=-vh -Seh }
program CurrencyTest;
{$mode objfpc}{$H+}
var
  C: Currency;
  D: Integer;
begin
  C := 1234.56;
  D := 2;
  C := C / D; // Hint: Use DIV instead to get an integer result
  Writeln(C);
end.