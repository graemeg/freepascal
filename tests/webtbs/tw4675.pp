{ OPT=-Sew }

{ Source provided for Free Pascal Bug Report 4675 }
{ Submitted by "Vicnent Snijders" on  2006-01-09 }
{ e-mail: vsnijders@quicknet.nl }
program Project1;

{$mode objfpc}{$H+}
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;

function GotHint_WantNoHint: string;
  procedure Add(const s: string);
  begin
    Result := Result + s;
  end;
begin
  if result = 'abc' then;
  Result := '';
  Add('Test');
end;

function GotNoHint_OK: string;
var
  a: string;
  procedure Add(const s: string);
  begin
    a:= a+ s;
  end;
begin
  a:='';
  Add('Test');
  Result:=a;
end;

function GotNoHint_WantHint: string;
var
  a: string;
  procedure Add(const s: string);
  begin
    a:= a+ s;
  end;
begin
  Add('Test');
  Result:=a;
end;

begin
  writeln(GotHint_WantNoHint);
  writeln(GotNoHint_OK);
  writeln(GotNoHint_WantHint);
end.

