<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

unit tval4;

{$mode fpc}

interface

function TestAllval4 : boolean;

implementation

uses
  tvalc;

type
  IntegerType = qword;

{$i tval.inc}


function TestAllval4 : boolean;
begin
  Writeln('Test val for qword type');
  TestAllval4:=TestAll;
end;

end.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

unit tval4;

{$mode fpc}

interface

function TestAllval4 : boolean;

implementation

uses
  tvalc;

type
  IntegerType = qword;

{$i tval.inc}


function TestAllval4 : boolean;
begin
  Writeln('Test val for qword type');
  TestAllval4:=TestAll;
end;

end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
