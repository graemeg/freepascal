program sha1performancetest;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  SysUtils,Classes,sha1;

var
  StartTime: TDateTime;
  EndTime: TDateTime;
  i: integer;
  s,ss: string;
begin
  writeln('MD5 of a million "a" symbols');
  Writeln('compile sha unit with -dSHA1SLOW to use unoptimized original version');
  SetLength(s, 1000000);
  for i := 1 to 1000000 do s[i] := 'a';

  StartTime:=now;
  for i := 0 to 1000 do
    ss := LowerCase(SHA1Print(SHA1string(s)));
  EndTime:=now;
  writeln('Performance test finished. Elapsed time:');
  writeln((EndTime-StartTime)*3600*24:0:3,' s');
end.

