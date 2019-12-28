{ %target=darwin,iphonesim}
{ %skipcpu=powerpc,powerpc64 }

{$mode objfpc}
{$modeswitch cblocks}

type
  tblock = reference to procedure(j: longint); cdecl; cblock;

  tc = class
    i: longint;
    procedure callme(j: longint);
  end;

var
  b: tblock;
  c: tc;

procedure tc.callme(j: longint);
const
  invocationcount: longint = 0;
begin
  writeln('self: ',hexstr(pointer(self)),', i: ',i,', j: ',j);
  if self<>c then
    halt(1);
  if i<>12345 then
    halt(2);
  if invocationcount=0 then
    begin
      if j<>1 then
        halt(3)
    end
  else if j<>2 then
    halt(4);
  inc(invocationcount);
end;


procedure test(b: tblock);
  begin
    b(2);
  end;

begin
  c:=tc.create;
  c.i:=12345;
  b:=@c.callme;
  b(1);
  test(@c.callme);
  test(b);
end.

