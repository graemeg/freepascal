{$mode objfpc}
{$modeswitch objectivec2}

program test;
uses
  CocoaAll;

type
  tc = objcclass(NSObject)
    a: char;
  end;

operator := (const right: array of const): NSMutableArray;
begin
  if tc(right[0].vPointer).a<>'a' then
    halt(1);
  result:=NSMutableArray.alloc.initWithCapacity(1);
  result.addObject(tc(right[0].vPointer));
end;

var
  c: tc;
  a: NSMutableArray;
begin
  c:=tc.alloc.init;
  c.a:='a';
  a := [c];
  for c in a do
    if c.a<>'a' then
      halt(2);
end.
