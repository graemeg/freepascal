{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  P : TPackage;
  T : TTarget;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fpgtk');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fpgt';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := [beos,haiku,freebsd,solaris,netbsd,openbsd,linux,win32,win64,os2,emx,aix,dragonfly];
    // Do not build fpgtk on iPhone (=arm-darwin)
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];
    P.Author := 'Luk Vandelaer & Sebastian Guenther (?)';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Lightweight OOP wrapper over GTK1.';
    P.NeedLibC:= True;

    P.Dependencies.Add('gtk1');
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('fpglib.pp');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fpgtk';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('fpglib.pp');
      with T.Dependencies do
        begin
          AddUnit('glib');
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('fpgtkext.pp');
      with T.Dependencies do
        begin
          AddUnit('fpgtk');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('fpgtk.pp');
      with T.Dependencies do
        begin
          AddUnit('fpglib');
        end;
    T.ResourceStrings := True;

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('examples/lister.pp');
    P.Targets.AddExampleProgram('examples/testgtk.pp');
    // 'examples/Makefile
    // 'examples/Makefile.fpc
    // 'examples/testgtk.ppr
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('gtk');
          AddUnit('gdk');
          AddUnit('glib');
        end;
    T:=P.Targets.AddUnit('fpgtk.pp');
      with T.Dependencies do
        begin
          AddUnit('gtk');
          AddUnit('gdk');
          AddUnit('glib');
          AddUnit('fpglib');
        end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
