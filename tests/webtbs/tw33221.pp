program Project1;
 
{$mode objfpc}{$H+}
{$ModeSwitch duplicatelocals}
 
type
  TClass1 = class(TObject)
  public
    X: Integer;
    procedure Proc1(X: Integer);
    procedure Proc2;
  end;
 
// Parameter has same name as member field.
// This is okay with duplicatelocals
procedure TClass1.Proc1(X: Integer);
begin
end;
 
// Local variable has same name as member field.
// Unlike with delphi mode, this is compiler error, even with duplicatelocals!
procedure TClass1.Proc2;
var
  X: Integer;
begin
end;
 
begin
end.    
    
