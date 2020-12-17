program int64modint64bug;
{$mode delphi}
const
   a = int64($100000000);
 var
   b: int64 = 123;
   c: int64;
begin
  c := b mod a;
  if c <> 0 then
  begin
  end;
end.
