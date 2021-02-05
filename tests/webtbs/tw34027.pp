uses
  strings;

type tz = record
       name : pchar;
     end;
const aa :array[0..2] of char = 'aa'#0;

const testArrZ : array [0..4] of tz = (
     (name: @aa), { Ok }
     (name: pchar(@aa)), { Ok }
     (name: pchar(@aa)+1),
     (name: pchar(@aa)+1+1),
     (name: pchar(@aa)+1+1-1)
     );

var b : pchar;

begin
  b:=pchar(@aa)+1; {Ok}
  if strlen(testArrZ[2].name)<>1 then
    halt(1);
  if strlen(testArrZ[3].name)<>0 then
    halt(2);
  if strlen(testArrZ[4].name)<>1 then
    halt(2);
end.
