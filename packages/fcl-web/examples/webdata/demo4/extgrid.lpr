program extgrid;

{$mode objfpc}{$H+}

uses
  {$ifdef unix}cwstring,xmliconv,{$endif}
  fpCGI, wmjsonusers, wmxmlusers, dmusers;

{$IFDEF WINDOWS}{$R extgrid.rc}{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$R *.res}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  Application.Initialize;
  Application.Run;
end.

