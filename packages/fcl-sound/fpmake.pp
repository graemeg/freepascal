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

    P:=AddPackage('fcl-sound');
    P.ShortName:='fsnd';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');

    P.Author := 'Abou Al Montacir of the Free Pascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Sound loading, storing and conversion parts for the Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('fpwavformat.pas');
    T:=P.Targets.AddUnit('fpwavreader.pas');
      with T.Dependencies do
        begin
          AddUnit('fpwavformat');
        end;
    T:=P.Targets.AddUnit('fpwavwriter.pas');
      with T.Dependencies do
        begin
          AddUnit('fpwavformat');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

