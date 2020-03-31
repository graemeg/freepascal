{ %fail }
program project1;
{$mode objfpc}{$H+}

uses uw36720a, uw36720b;

var
  a: TObject;
begin
  ( a as IInterface2 ).DoSomethingElse;
end.

