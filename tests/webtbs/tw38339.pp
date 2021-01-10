{%OPT=-O3 }
program test48086;
{$mode objfpc}{$H+}
function IsFontNameXLogicalFontDesc(const LongFontName: string): boolean;
var MinusCnt, p: integer;
begin
  MinusCnt:=0;
  for p:=1 to length(LongFontName) do
    if LongFontName[p]='-' then inc(MinusCnt);
  Result:=(MinusCnt=14);
end;
var
myfont:string;
begin
 myfont:='Myfont--------------';
 if IsFontNameXLogicalFontDesc(myfont) then
  writeln('NO ERROR')
 else
  begin
    writeln('Error in count');
    halt(1);
  end;
end.
