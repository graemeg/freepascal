{$ifdef fpc}
{$mode objfpc}
{$endif fpc}

Uses ComObj;

Var
  Cells,
  ActiveSheet,
  WorkBooks,
  ExcelApp : Variant;
  I,j : Integer;

begin
  ExcelApp:=CreateOleObject('Excel.Application');
  WorkBooks:=ExcelApp.WorkBooks;
  WorkBooks.Add;
  ActiveSheet:=ExcelApp.ActiveSheet;
  For I:=1 to 5 do
    For J:=1 to 5 do
      begin
      Cells:=ActiveSheet[I,J];
      Cells.Value:=I+J;
      end;
end.
