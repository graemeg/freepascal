{ %INTERACTIVE }

{ Note: to test this, first compile this program, then change something inside
        unit uw26760 and make sure that it's recompiled (e.g. either by checking
        the compiler messages or by introducing a compile error) }

program tw26760;

{$mode objfpc}{$H+}

uses
  uw26760 in 'uw26760/uw26760.pp';

begin
  Test;
end.

