program tismngd2;

var
  l: LongInt;
  o: TObject;
  _as: AnsiString;
begin
  if IsManagedType(l) then
    Halt(1);
  if IsManagedType(o) then
    Halt(2);
  if not IsManagedType(_as) then
    Halt(3);
  Writeln('Ok');
end.
