{ %NORUN }

program tw38310a;

{$mode objfpc}{$H+}

uses
  SysUtils, StrUtils, Math;

begin
  IfThen(true, 'A', IfThen(true, 'B', 'C'));
end.
