{ test (a and b) or (c and not(b)) into c xor ((c xor a) and b) optimization with random values }
procedure test_word;
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
  end;

procedure test_boolean;
  var
    i : word;
    a,b,c,_a,_b,_c : boolean;
  begin
    for i:=1 to 100 do
      begin
        a:=boolean(random(2));
        _a:=a;
        b:=boolean(random(2));
        _b:=b;
        c:=boolean(random(2));
        _c:=c;
        if (a and b) or (c and not(b))<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a,'b=',b,'c=',c);
            halt(11);
          end;
        if (a and b) or (not(b) and c)<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a,'b=',b,'c=',c);
            halt(11);
          end;
        if (not(b) and c) or (a and b)<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a,'b=',b,'c=',c);
            halt(11);
          end;
        if (not(b) and c) or (b and a)<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a,'b=',b,'c=',c);
            halt(11);
          end;
      end;
  end;


procedure test_pboolean;
  var
    i : word;
    a,b,c : pboolean;
    _a,_b,_c : boolean;
  begin
    new(a);
    new(b);
    for i:=1 to 100 do
      begin
        a^:=true;
        _a:=a^;
        b^:=true;
        _b:=b^;
        c:=nil;  
        { c should not matter in this case }      
        _c:=boolean(random(2));
        if (a^ and b^) or (c^ and not(b^))<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a^,'b=',b^,'c=',c^);
            halt(21);
          end;
        if (a^ and b^) or (not(b^) and c^)<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a^,'b=',b^,'c=',c^);
            halt(21);
          end;
        if (not(b^) and c^) or (a^ and b^)<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a^,'b=',b^,'c=',c^);
            halt(21);
          end;
        if (not(b^) and c^) or (b^ and a^)<>_c xor ((_c xor _a) and _b) then
          begin
            writeln('Error: ','a=',a^,'b=',b^,'c=',c^);
            halt(21);
          end;
      end;
    dispose(a);
    dispose(b);
  end;


begin
  test_word;
  test_boolean;
  test_pboolean;
end.

