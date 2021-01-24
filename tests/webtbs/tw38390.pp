program tw38390;
{$MODE Delphi}
uses SysUtils;

var
  s: String;
  x: UInt64;

begin
  s := '20000000000';
  x := UInt64.Parse(s);
  WriteLn(x);
  if x <> 20000000000 then
    Halt(1);
  UInt64.TryParse(s, x);
  WriteLn(x);
  if x <> 20000000000 then
    Halt(2);
  x := StrToQWord(s);
  WriteLn(x);
  if x <> 20000000000 then
    Halt(3);
end.
