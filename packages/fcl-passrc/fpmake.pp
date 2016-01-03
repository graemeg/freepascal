{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fcl-passrc');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fcls';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Dependencies.Add('fcl-base');
    P.Author := 'Sebastian Guenther';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Pascal parsing parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16];
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-passrc';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.Dependencies.Add('fcl-base');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('pastree.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pscanner.pp');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pparser.pp');
      with T.Dependencies do
        begin
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pastounittest.pp');
      with T.Dependencies do
        begin
          AddUnit('pparser');
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('passrcutil.pp');
      with T.Dependencies do
        begin
          AddUnit('pparser');
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T.ResourceStrings := False;

    T:=P.Targets.AddUnit('paswrite.pp');
      with T.Dependencies do
        begin
          AddUnit('pastree');
        end;
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('paswrite.pp');
      with T.Dependencies do
        begin
          AddUnit('pastree');
        end;
    T:=P.Targets.AddUnit('pparser.pp');
      with T.Dependencies do
        begin
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T:=P.Targets.AddUnit('pscanner.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
