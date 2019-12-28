{ %NORUN }

{$mode objfpc}

program tw36388;
uses
  SysUtils, FGL;

generic function CopyList<T: TFPSList> (source: T): T;
begin
 // Internal error 200204175
  result := T.Create;
end;

begin
end.
