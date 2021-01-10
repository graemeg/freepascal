program fs;

{$mode objfpc}{$H+}

function UTF8Length(const s: string): PtrInt; inline;
begin
  Result:=9;
end;


var
  v1: string;
  s: shortstring;
  i: Integer;
begin
  v1 := '123456789';
  s := v1;
  for i := 1 to UTF8Length(s)-8 do begin
  end;
end.
