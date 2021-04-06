{ %opt=-O4 }
{$mode objfpc}
{$R+}
program project1;

const
  MaxLoopDepth = 4;
type
  TES = record
    LoopDepth: Integer;
    Sums: array [1..MaxLoopDepth] of Double;
  end;
  PES = ^TES;
  TE = class
    ThreadStates: array of PES;
  end;

  TSF = class
  public
    function NI(Evaluator: TE; var a:array of Double): Double; virtual;
  end;

var
  E: TE;
  ES: TES;
  D: Double;
  SF: TSF;

threadvar
  ThreadIndex: Integer;

function TSF.NI(Evaluator: TE; var a: array of Double): Double;
begin
  with Evaluator.ThreadStates[ThreadIndex-1]^ do begin
    Sums[LoopDepth] := Sums[LoopDepth] + a[0];
    Result := Sums[LoopDepth];
  end;
end;

begin
  ThreadIndex := 2;
  SF := TSF.Create;
  E := TE.Create;
  SetLength(E.ThreadStates,2);
  E.ThreadStates[1] := @ES;
  ES.LoopDepth := 1;
  ES.Sums[1] := 0;
  D := 27;
  SF.NI(E, D);
  SF.NI(E, D);
  WriteLn(ES.Sums[1]); { should write 54 }
  if (ES.Sums[1]<53.999) or (ES.Sums[1]>54.001) then
    halt(1);
  writeln('ok');
end.

