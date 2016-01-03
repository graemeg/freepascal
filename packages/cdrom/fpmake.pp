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

    P:=AddPackage('cdrom');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='cdr';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=[Win32,Win64,Linux];

    P.Author := 'Michael van Canneyt';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Unit to read a CDROM disc TOC and get a list of CD Rom devices';
    P.NeedLibC:= False;

=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='cdrom';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=[Win32,Win64,Linux];

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('cdrom.pp');
      with T.Dependencies do
        begin
          AddInclude('cdromlin.inc',[Linux]);
          AddInclude('cdromw32.inc',[Win32,Win64]);
          AddUnit('lincd',[Linux]);
          AddUnit('wincd',[Win32,Win64]);
        end;
    T:=P.Targets.AddUnit('discid.pp');
      with T.Dependencies do
        begin
          AddUnit('cdrom');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpcddb.pp');
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    // Linux
    T:=P.Targets.AddUnit('lincd.pp',[Linux]);
      with T.Dependencies do
        begin
          AddUnit('major');
        end;
    T:=P.Targets.AddUnit('major.pp',[Linux]);

    // Windows
    T:=P.Targets.AddUnit('cdromioctl.pp',[Win32,Win64]);
    T:=P.Targets.AddUnit('scsidefs.pp',[Win32,Win64]);
    T:=P.Targets.AddUnit('wincd.pp',[Win32,Win64]);
      with T.Dependencies do
        begin
          AddUnit('cdromioctl');
          AddUnit('wnaspi32');
          AddUnit('scsidefs');
        end;
    T:=P.Targets.AddUnit('wnaspi32.pp',[Win32,Win64]);

<<<<<<< HEAD
<<<<<<< HEAD

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('getdiscid.pp');
    T:=P.Targets.AddExampleProgram('showcds.pp');

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
