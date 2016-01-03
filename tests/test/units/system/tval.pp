<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

program TestVal;

uses
  { longint type, short string }
  tval1,
  { dword type, short string }
  tval2,
  { int64 type, short string }
  tval3,
  { uint64 type, short string }
  tval4,
  { common variables and functions }
  tvalc;



begin
  if (paramcount>0) and
     (paramstr(1)='verbose') then
       silent:=false;
  TestAllVal1;
  TestAllVal2;
  TestAllVal3;
  TestAllVal4;
  if HasErrors then
    begin
      Writeln('Test tval failed');
      Halt(1);
    end;
end.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

program TestVal;

uses
  { longint type, short string }
  tval1,
  { dword type, short string }
  tval2,
  { int64 type, short string }
  tval3,
  { uint64 type, short string }
  tval4,
  { common variables and functions }
  tvalc;



begin
  if (paramcount>0) and
     (paramstr(1)='verbose') then
       silent:=false;
  TestAllVal1;
  TestAllVal2;
  TestAllVal3;
  TestAllVal4;
  if HasErrors then
    begin
      Writeln('Test tval failed');
      Halt(1);
    end;
end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
