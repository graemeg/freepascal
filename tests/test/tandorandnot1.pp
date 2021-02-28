{ test (a and b) or (c and not(b)) into c xor ((c xor a) and b) optimization with random values }
var
  i,a,b,c,_a,_b,_c : word;
begin
  for i:=1 to 1000 do
    begin
      a:=random(65536);
      _a:=a;
      b:=random(65536);
      _b:=b;
      c:=random(65536);
      _c:=c;
      if (a and b) or (c and not(b))<>_c xor ((_c xor _a) and _b) then
        begin
          writeln('Error: ','a=',a,'b=',b,'c=',c);
          halt(1);
        end;
      if (a and b) or (not(b) and c)<>_c xor ((_c xor _a) and _b) then
        begin
          writeln('Error: ','a=',a,'b=',b,'c=',c);
          halt(1);
        end;
      if (not(b) and c) or (a and b)<>_c xor ((_c xor _a) and _b) then
        begin
          writeln('Error: ','a=',a,'b=',b,'c=',c);
          halt(1);
        end;
      if (not(b) and c) or (b and a)<>_c xor ((_c xor _a) and _b) then
        begin
          writeln('Error: ','a=',a,'b=',b,'c=',c);
          halt(1);
        end;
    end;
end.
