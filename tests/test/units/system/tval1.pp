<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

unit tval1;

{$mode fpc}

interface

function TestAllVal1 : boolean;

implementation

uses
  tvalc;

type
  IntegerType = longint;

{$i tval.inc}


function TestAllVal1 : boolean;
begin
  Writeln('Test val for longint type');
  TestAllVal1:=TestAll;
end;

end.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

unit tval1;

{$mode fpc}

interface

function TestAllVal1 : boolean;

implementation

uses
  tvalc;

type
  IntegerType = longint;

{$i tval.inc}


function TestAllVal1 : boolean;
begin
  Writeln('Test val for longint type');
  TestAllVal1:=TestAll;
end;

end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
