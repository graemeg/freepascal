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

    P:=AddPackage('nvapi');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
=======
    P.Directory:='nvapi';
{$endif ALLPACKAGES}
    P.Version:='2.4.0';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='nvapi';
{$endif ALLPACKAGES}
    P.Version:='2.4.0';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='nvapi';
{$endif ALLPACKAGES}
    P.Version:='2.4.0';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='nvapi';
{$endif ALLPACKAGES}
    P.Version:='2.4.0';
>>>>>>> origin/cpstrnew
    P.Author := 'NVidia, Andreas Hausladen, Dmitry "skalogryz" Boyarintsev';
    P.License := 'NVidia license';
    P.HomepageURL := 'nvidia.com';
    P.Email := '';
    P.Description := 'NvAPI header';
    P.NeedLibC:= true;
    P.OSes:=[win32,win64];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('nvapi.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
