{ Source provided for Free Pascal Bug Report 5001 }
{ Submitted by "Vincent Snijders" on  2006-04-09 }
{ e-mail: vsnijders@quicknet.nl }
program Project1;

{$mode objfpc}{$H+}
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;

resourcestring
  s = 'test';

begin
  writeln(s);
  if length(s)=0
    then halt(1);
  if (s<>'test')
    then halt(1);
end.
