{$inline on}
{$mode objfpc}
function Correct(TempInt: integer; Value: word): word; inline;
begin
 if TempInt = 32768 then
   Result := Value - TempInt
  else
   Result := 65536 - Value;
end;

procedure p;
var
 Arr: array of word;
 Temp: integer;
begin
 SetLength(Arr,1);
 Temp:= 42;
 Arr[0] := Correct(Temp, Arr[0]);
end; 

begin
end.
