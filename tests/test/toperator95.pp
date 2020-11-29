{ %FAIL }

program toperator95;

{$mode objfpc}
{$modeswitch advancedrecords}

type
  TString80 = String[80];
  TString90 = String[90];

  TTest = record
    class operator :=(const aArg: TTest): TString80;
  end;

class operator TTest.:=(const aArg: TTest): TString80;
begin
end;

operator :=(const aArg: TTest): TString90;
begin
end;

var
  t: TTest;
  s80: TString80;
begin
  s80 := t;
end.
