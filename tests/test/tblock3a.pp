{ %target=darwin,iphonesim}
{ %skipcpu=powerpc,powerpc64 }

program tblock3a;

{$mode objfpc}
{$modeswitch cblocks}

type
  {$calling cdecl}
  tblock1 = reference to procedure(j: longint); cblock;

  {$calling mwpascal}
  tblock2 = reference to procedure(j : longint); cblock;

  tc = class
    i: longint;
    procedure callme(j: longint);
  end;

var
  b1: tblock1;
  b2: tblock2;
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
  case invocationcount of
    0:
      if j<>1 then
        halt(3);
    1, 2:
      if j<>2 then
        halt(4);
    3:
      if j<>3 then
        halt(5);
    4, 5:
      if j<>4 then
        halt(6);
  end;
  inc(invocationcount);
end;


procedure test1(b: tblock1);
  begin
    b1(2);
  end;

procedure test2(b: tblock2);
  begin
    b2(4);
  end;

begin
  c:=tc.create;
  c.i:=12345;
  b1:=@c.callme;
  b1(1);
  test1(@c.callme);
  test1(b1);
  b2:=@c.callme;
  b2(3);
  test2(@c.callme);
  test2(b2);
end.

