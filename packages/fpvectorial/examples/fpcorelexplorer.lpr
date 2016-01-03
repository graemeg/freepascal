program fpcorelexplorer;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms, fpce_mainform
  { you can add units after this };

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TformCorelExplorer, formCorelExplorer);
  Application.Run;
end.

