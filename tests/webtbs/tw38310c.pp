{ %NORUN }

program tw38310c;

{$mode objfpc}{$H+}

uses
  StrUtils, Math, SysUtils;

begin
  IfThen(true, 'A', IfThen(true, 'B', 'C'));
end.
