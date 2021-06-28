
{$mode objfpc}
program gentest;

generic function ConstString<const P,Q,S: string>: PChar;
var
  size: Integer;
begin
  Size := SizeOf(P) + SizeOf(Q) + SizeOf(S);
  writeln(Size);

  Result := P+Q+S;
end;

var
  s: PChar;
begin
  s := specialize ConstString<'Hello', ' world', '!'>; // error gentest.lpr(16,50) Error: Incompatible types: got "Char" expected "AnsiString"
  if s<>'Hello world!' then
    halt(1);
  writeln(s);
end.
