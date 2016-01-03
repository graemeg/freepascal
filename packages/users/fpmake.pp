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

    P:=AddPackage('users');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Michael van Canneyt, Marco van de Voort';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.OSes := [freebsd,linux];
    P.Email := '';
    P.Description := 'Headers to access Unix groups and users.';
    P.NeedLibC:= false;

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('crypth.pp',[linux]);
    T:=P.Targets.AddUnit('grp.pp');
    T:=P.Targets.AddUnit('pwd.pp');
    T:=P.Targets.AddUnit('shadow.pp',[linux]);
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='users';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('grp.pp');
    T:=P.Targets.AddUnit('pwd.pp');
    T:=P.Targets.AddUnit('shadow.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('users.pp');
      with T.Dependencies do
        begin
          AddUnit('pwd');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('shadow',[linux]);
          AddUnit('grp');
        end;
    T.ResourceStrings := true;

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testpass.pp');
    P.Targets.AddExampleProgram('testpass2.pp');
    P.Targets.AddExampleProgram('testuser.pp');
=======
          AddUnit('shadow');
          AddUnit('grp');
        end;
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('shadow');
          AddUnit('grp');
        end;
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
