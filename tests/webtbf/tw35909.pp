{ %fail% }
{$mode delphi}
program IS_Precedence;
uses
  Classes;
var
  O1, O2: TObject;
begin
  O1 := TComponent.Create(nil);
  O2 := TObject.Create;
  Writeln(O1 is TComponent or O2 is TComponent); // <<< should not compile because OR has precedence before IS
end.
