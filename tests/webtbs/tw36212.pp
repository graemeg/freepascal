{ %OPT=-O4 }
program Project1;
{$mode delphi}
type
  TSinglePoint = record
    X, Y: Single;
  public
    constructor Create(const aX, aY: Single);
  end;
{ TSinglePoint }
constructor TSinglePoint.Create(const aX, aY: Single);
begin
  X := aX;
  Y := aY;
end;

type
  TMyObj = class
  public
    function Test: TSinglePoint;
  end;
{ TMyObj }
function TMyObj.Test: TSinglePoint;
begin
  Result := TSinglePoint.Create(0, 0);
end;

var
  MyObj: TMyObj;
  i : PtrUInt;
begin
  MyObj := TMyObj.Create;
  Writeln(PtrUInt(MyObj));
  i:=PtrUInt(MyObj);
  MyObj.Test;
  Writeln(PtrUInt(MyObj));
  if i<>PtrUInt(MyObj) then
    halt(1);
  writeln('ok');
end.
