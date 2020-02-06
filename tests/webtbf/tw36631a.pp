{ %FAIL }

program tw36631a;

{$APPTYPE CONSOLE}
{$mode objfpc}{$H+}

uses
  Classes,
  SysUtils;

var
  LongStr: String;
  SingleStr: String;

begin
  LongStr := 'Some example, test text. Another one, or something like that.';

  SingleStr := LongStr.Split([',', '.']).[1];
  writeln(SingleStr); // ' test text'

  writeln('done');
end.
