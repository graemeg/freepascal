uses
  bench,sysutils;
var
  i : longint;
  ps : ansistring;
  sticks : int64;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  ps:='';
  if paramcount>0 then
    begin
      for i:=2 to paramcount do
        ps:=ps+' "'+paramstr(i)+'"';
=======
=======
>>>>>>> origin/fixes_2_2
  if paramcount>0 then
    begin
      for i:=2 to paramcount do
        ps:=ps+' '+paramstr(i);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      sticks:=GetMicroSTicks;
      ExecuteProcess(paramstr(1),ps);
      writeln(stderr,(GetMicroSTicks-sticks)/1000:0:3,' ms');
    end;
end.
