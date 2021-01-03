{ %cpu=i386 }
{ %opt=-CfAVX -CpCOREAVX2 -OoFASTMATH }
uses
  cpu;
var
    a, b: uint32; // or (u)int64; int32 works
    r: single; // or double, or even extended
begin
  if FMASupport then
    begin
      a := 1;
      b := 3;
      r := a + b / 10;
      writeln(r:0:3);
      if r>2.0 then
         halt(1);
      writeln('ok');
    end;
end.
