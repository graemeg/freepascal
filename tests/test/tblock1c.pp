{ %target=darwin,iphonesim}
{ %skipcpu=powerpc,powerpc64 }

{$modeswitch cblocks}

type
  tblock = reference to function(l: longint): longint; cdecl; cblock;

function test(b: tblock; l: longint): longint;
  begin
    test:=b(l);
  end;

function func(l: longint): longint;
  begin
    writeln('called as block');
    func:=l+1;
  end;

const
  bconst: tblock = @func;

var
  b: tblock;
begin
  b:=@func;
  if b(1)<>2 then
    halt(1);
  if test(@func,4)<>5 then
    halt(2);
  if test(b,123)<>124 then
    halt(3);
  if bconst(100)<>101 then
    halt(4);
  if test(bconst,10)<>11 then
    halt(5);
end.

