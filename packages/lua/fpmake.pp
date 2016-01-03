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

    P:=AddPackage('lua');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
//    P.Dependencies.Add('x11');
    P.OSes := [linux,win32];
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='lua';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
//    P.Dependencies.Add('x11');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('lauxlib.pas');
      with T.Dependencies do
        begin
          AddUnit('lua');
        end;
    T:=P.Targets.AddUnit('lualib.pas');
      with T.Dependencies do
        begin
          AddUnit('lua');
        end;
    T:=P.Targets.AddUnit('lua.pas');


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
