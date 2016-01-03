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

    P:=AddPackage('iconvenc');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Marco van de Voort';
    P.License := 'Library: LGPL2 or later, header: LGPL with modification, ';
    P.OSes := [beos,haiku,freebsd,darwin,iphonesim,solaris,linux,aix];
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A libiconv header translation.';
    P.NeedLibC:= true;

=======
    P.Directory:='iconvenc';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> graemeg/fixes_2_2
=======
    P.Directory:='iconvenc';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('iconvenc.pas');
<<<<<<< HEAD
<<<<<<< HEAD
    T.Dependencies.AddInclude('iconvert.inc');
    T:=P.Targets.AddUnit('iconvenc_dyn.pas');
    T.Dependencies.AddInclude('iconvert.inc');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('iconvtest.pp');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
