{ %opt=-vn -Sen }
{ %norun }
{$mode objfpc}
type
  TForm = class
    procedure OnClick;
  end;
  
  TNotifyEvent = procedure of object;
  
procedure TForm.OnClick;
  begin
  end;
  
  
procedure Test (aObject: TObject);
var
  aForm: TForm;
  aEvent: TNotifyEvent;
begin
  if (aObject is TForm) then
  begin
    aForm := aObject as TForm;
    aEvent := @aForm.OnClick;
    aEvent();
  end;
end; 

begin
  Test(nil);
end.
