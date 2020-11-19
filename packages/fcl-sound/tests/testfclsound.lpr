program testfclsound;

{$mode objfpc}{$H+}

uses
  Interfaces, Forms, GuiTestRunner, tcwavreader;

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TGuiTestRunner, TestRunner);
  Application.Run;
end.

