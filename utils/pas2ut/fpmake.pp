{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_pas2ut(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-pas2ut');
    P.ShortName:='p2ut';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := 'Free Pascal Team';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Description := 'Pascal source to FPC Unit test generator program';
    P.Email := '';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-passrc');

    T:=P.Targets.AddProgram('pas2ut.pp');
    T.ResourceStrings:=true;
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_pas2ut('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




