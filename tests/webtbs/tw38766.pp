{$mode objfpc}

type
  trec = record
    x, y: longint;
  end;

function max(x,y: longint): longint;
begin
  if x>y then
    result:=x
  else
    result:=y;
end;

function test: trec; inline;
begin
 result.x:=1;
 result.y:=2;
 result.x:=max(result.x,result.y);
end;
    
begin
  if test.x<>2 then
    halt(1);
  if test.y<>2 then
    halt(2);
end.
