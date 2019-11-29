{ %norun }
{ %opt=-Sew }

{$r+}
{$warnings on}

const
  MH_MAGIC = $feedface;

var
  c: cardinal;
begin
  c:= NToBE(MH_MAGIC);
end.

