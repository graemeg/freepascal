{ %norun }

<<<<<<< HEAD
<<<<<<< HEAD
{$inline on}
program TEST;
{$EXTENDEDSYNTAX OFF}
VAR
   IO: WORD; { or LONGINT if wanted }

function test: word; inline;
begin
  test:=1;
  writeln(io);
end;

BEGIN
   IO:=test;
END.
=======
=======
>>>>>>> origin/fixes_2_2
program TEST;
{$EXTENDEDSYNTAX OFF}
USES
  DOS;
VAR
  IO: WORD; { or LONGINT if wanted }
BEGIN
  IO:=IORESULT;
END.


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
