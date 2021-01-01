{ %OPT=-O3 }
{$goto on}
label start0, end0, start1, end1;

var
	x: int16;

begin
	x := random(2);
	writeln('x := ', x);
	writeln;

start0:
	x :=
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+x;
end0:
	writeln('x := 1 + 1 + ...100 times ... + x, x = ', x, ': ');
    writeln(SizeUint(CodePointer(@end0) - CodePointer(@start0)), ' b of code');
    { hundred is actually arbitrarily chosen but should be sufficient for all targets
      to show that constant folding works }
    if SizeUint(CodePointer(@end0) - CodePointer(@start0))>100 then
       halt(1);
	writeln;

start1:
	x := x+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+
		1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1+1;
end1:
	writeln('x := x + 1 + 1 + ...100 times ..., x = ', x, ': ');
    { hundred is actually arbitrarily chosen but should be sufficient for all targets
      to show that constant folding works }
    writeln(SizeUint(CodePointer(@end1) - CodePointer(@start1)), ' b of code');
    if SizeUint(CodePointer(@end1) - CodePointer(@start1))>100 then
     halt(2);
   writeln('ok');
end.

