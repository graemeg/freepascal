program Project1;

{$mode objfpc}{$H+}

uses
  SysUtils;

var
  D: Double;
  Q: QWord;

begin
  D := -1;

  Q := D.Frac;
  D.Frac := Q; // the sign is lost!

  if D<>-1 then
    halt(1);
  WriteLn('ok');
end.
