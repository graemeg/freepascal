{ %OPT=-O- -O1 }
program tshlshr;

procedure test(value, required: int64);
begin
  if value <> required then
    halt(1)
  else
    halt(0);
end;

var
 longres : longint;

begin
   longres := 32768;
   test(longres, 32768);
end.