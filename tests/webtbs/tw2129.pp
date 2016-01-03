{ %version=1.1 }
{ Source provided for Free Pascal Bug Report 2129 }
{ Submitted by "Bill Rayer" on  2002-09-18 }
{ e-mail: lingolanguage@hotmail.com }

{$mode delphi}

(*
Comp() cast has different effect in FPC.
Compiles using Delphi4:
  dcc32 -CC fpc19
Compiles in FPC 1.0.6:
  ppc386 -WC fpc19

When run, the Delphi version shows -6.5E+18, but the FPC version
shows zero. In Delphi, the comp() cast actually moves 8 bytes from
the double into the comp without converting the data, but FPC uses
floating point instructions to convert the data and therefore prints
zero.

In Delphi, if you want to convert a double to a float, you just use
the assignment "comp1 := dbl1" which corresponds to the FLD/FIST
opcodes. FPC should not use the comp() cast for doing this, since
it introduces a subtle incompatibility with Delphi.
*)

program fpc19;
uses
  sysutils;

var
  comp1 : comp;
  dbl1 : double;
  s : string;
begin
  dbl1 := -1e-128;
  comp1 := comp(dbl1);
{$ifdef FPC_DOUBLE_HILO_SWAPPED}
  comp1:=swap(comp1);
{$endif FPC_DOUBLE_HILO_SWAPPED}
<<<<<<< HEAD
  str(comp1:23,s);
=======
  str(comp1,s);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC_HAS_TYPE_EXTENDED}
  if s<>'-6.53142228756617E+0018' then
{$else FPC_HAS_TYPE_EXTENDED}
  { this constant has been verified and is correct (FK) }
  { doubles have slightly different precision on processors <> x86, because }
  { intermediate calculations are not performed in 80 bit there (JM)        }
<<<<<<< HEAD
  if trim(s)<>'-6531422287566170211' then
=======
  if Copy(s,1,Length(s)-1)<>'-653142228756617021' then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_HAS_TYPE_EXTENDED}
    begin
      writeln(s);
      writeln('error');
      halt(1);
    end;
end.
