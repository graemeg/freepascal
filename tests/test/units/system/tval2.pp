<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

unit tval2;

{$mode fpc}

interface

function TestAllval2 : boolean;

implementation

uses
  tvalc;

type
  IntegerType = dword;

{$i tval.inc}


function TestAllval2 : boolean;
begin
  Writeln('Test val for dword type');
  TestAllval2:=TestAll;
end;

end.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

unit tval2;

{$mode fpc}

interface

function TestAllval2 : boolean;

implementation

uses
  tvalc;

type
  IntegerType = dword;

{$i tval.inc}


function TestAllval2 : boolean;
begin
  Writeln('Test val for dword type');
  TestAllval2:=TestAll;
end;

end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
