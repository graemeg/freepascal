{ %FAIL }

{$mode objfpc}
uses
  SysUtils, Classes;
type

  { TFoo }

  TFoo = class
    procedure Bar;
  end;

var
  AByte: Byte;

{ TFoo }

procedure TFoo.Bar;
begin
  AByte := Byte.SetBit(1);
end;

begin
end.
