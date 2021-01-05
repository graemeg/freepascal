{ %NORUN }

program tw38310b;

{$mode objfpc}{$H+}

uses
  StrUtils, SysUtils, Math;

begin
  IfThen(true, 'A', IfThen(true, 'B', 'C'));
end.
