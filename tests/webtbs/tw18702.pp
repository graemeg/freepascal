program project1;

{$mode objfpc}{$H+}
{$modeswitch nestedprocvars}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
  TGetSQLTextProc = function() : string is nested;

procedure TestSQLFieldType(AGetSQLTextProc: TGetSQLTextProc);
var
  i          : byte;
  s: string;
begin
  for i := 0 to 9 do
    begin
    s := AGetSQLTextProc();
    if s<>'hello' then
      begin
      writeln('Failed');
      halt(i+1);
      end;
    end;
end;

procedure TestSQLClob;

  function GetSQLText() : string;
  begin
    result := 'hello';
  end;

begin
  TestSQLFieldType(@GetSQLText);
end;

begin
  TestSQLClob;
end.
