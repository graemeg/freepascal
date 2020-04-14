uses
  sysutils,math;
var
  s: Single;
  d: Double;
begin
  s := MaxSingle;
  d := MaxDouble;
  Writeln(IntToHex(PLongInt(@s)^, 8));
  if IntToHex(PLongInt(@s)^, 8)<>'7F7FFFFF' then
     halt(1);
  Writeln(IntToHex(PInt64(@d)^, 16));
  if IntToHex(PInt64(@d)^, 16)<>'7FEFFFFFFFFFFFFF' then
    halt(2);
  s := MinSingle;
  d := MinDouble;
  Writeln(IntToHex(PLongInt(@s)^, 8));
  if IntToHex(PLongInt(@s)^, 8)<>'00800000' then
    halt(3);
  Writeln(IntToHex(PInt64(@d)^, 16));
  if IntToHex(PInt64(@d)^, 16)<>'0010000000000000' then
    halt(4);
  writeln('ok');
end.
