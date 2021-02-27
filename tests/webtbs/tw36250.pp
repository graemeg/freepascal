{ %norun }
{ %target=darwin,ios,iphonesim}
{ %opt=-gw3 }

{$mode objfpc}{$h+}
{$ModeSwitch objectivec2}

function NSStringToString(ns: NSString): String;
begin
    Result := '';
end;                 

begin
  WriteLn(NSStringToString(nil));
end.
