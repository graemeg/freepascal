<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef fpc}
{$mode objfpc}
{$endif}
=======
{$mode objfpc}
>>>>>>> graemeg/fixes_2_2
=======
{$mode objfpc}
>>>>>>> origin/fixes_2_2
program tw12385;

uses
  SysUtils;

var
  s: string; 
  cr: Extended; 

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TestIt(CR : Extended; Fmt,Expected : String);
  
begin
  S:=FormatFloat(Fmt,cr);
  If S<>Expected then
    begin
    Writeln('"',S,'"<>"',Expected,'" (latter is correct)');
    Halt(1);
    end;
end;

begin
  DecimalSeparator:='.';
  ThousandSeparator:=',';
  TestIt(1234.567,'00000000.00','00001234.57');
  TestIt(-1234.567,'00000000.00','-00001234.57');
  TestIt(-1234.567,'000.00','-1234.57');
  TestIt(-1,'000.000','-001.000');
  TestIt(-80,'#,##0.00','-80.00');
  TestIt(-140,'#,##0.00','-140.00');
  TestIt(140,'#,##0.00','140.00');
  TestIt(80,'#,##0.00','80.00');
  TestIt(-2.45,'#,##0.00','-2.45');
  TestIt(-1400,'#,##0.00','-1,400.00');
  TestIt(-1400,'##,##0.00','-1,400.00');
=======
=======
>>>>>>> origin/fixes_2_2
begin
  cr := -1234.567;
  s:=FormatFloat('0.00', cr);
  if s<>'-1234.57' then 
    begin
    Writeln(S,'<> -1234.57 (latter is correct)');
    Halt(1);
    end;
  S:=FormatFloat('000.000',-1); //returns 0-1.000
  if s<>'-001.000' then
     begin
     Writeln(S,'<> -001.000 (latter is correct)');
     Halt(1);
     end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
