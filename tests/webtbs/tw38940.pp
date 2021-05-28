{ %OPT=-O3 }
procedure Test;
var b:boolean;
    i:longint;
begin
// the following loop should be done 2 time, but it hangs up;
 b:=true;
 i:=0;
 repeat
   inc(i);
   if i>2 then 
     halt(1);
   b:=not b; // first time b is set to false thats why the loop should be done again
              // second time b is set to true thats why the loop should be leave,
              // but hangs up
 until b;
end;

begin
  Test;
  writeln('ok');
end.
