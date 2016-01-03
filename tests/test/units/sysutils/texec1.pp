{ %result=1 }
<<<<<<< HEAD
<<<<<<< HEAD
{ %NEEDEDAFTER }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ this test fails when it isn't called by execansi }
  uses
    sysutils;

  const
    comparestr='-Fu/usr/local/lib/fpc/1.0.10/units/freebsd/rtl/*';

  var
    i : Longint;

begin
   for i:=1 to 11 do
     if ParamStr(i)<>comparestr Then
       halt(1);
   if paramstr(12)<>'' then
     halt(1);
end.
