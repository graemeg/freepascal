
type
  pnode = ^node;
  node = record
    i: integer;
    left: pnode;
    right: pnode;
  end;

procedure insert(var t: pnode; i: integer);
begin
  if t = nil then
    begin
      new(t);
      t^.i := i;
      t^.left := nil;
      t^.right := nil;
    end
  else
    if i < t^.i
      then insert(t^.left, i)
      else insert(t^.right, i);
end;

begin
end.
