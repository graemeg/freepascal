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

    P:=AddPackage('unzip');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='zip';
    P.Description := 'Support for decompression of ZIP archives - either using a Pascal port of the library from InfoZIP, or interface to using the dynamically linked version of this library.';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes := P.OSes - [embedded,nativent,msdos,win16];

    T:=P.Targets.AddUnit('unzip51g.pp');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='unzip';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('unzip.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('ziptypes');
        end;
    T:=P.Targets.AddUnit('ziptypes.pp');
    T:=P.Targets.AddUnit('unzipdll.pp',[emx,os2]);
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
