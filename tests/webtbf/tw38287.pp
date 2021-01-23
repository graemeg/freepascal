{$macro on}
var
  a,b,s : real;

begin
  a:=1;
  b:=2;
{$define sum:=a+b }
{$define b:=sum} { DON’T do this !!!}
  s:=sum;         { Will be infinitely recursively expanded... }
end.
