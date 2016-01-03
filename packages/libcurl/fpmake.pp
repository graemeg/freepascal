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

    P:=AddPackage('libcurl');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';

    P.Author := 'Library: Daniel Stenberg, header: Free Pascal development team';
    P.License := 'Library: MIT, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.OSes := [beos,haiku,freebsd,darwin,iphonesim,solaris,netbsd,openbsd,linux,aix,dragonfly,win32,win64];
    P.Email := '';
    P.Description := 'Library to fetch files from URLs using many protocols.';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?

=======
    P.Directory:='libcurl';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> graemeg/fixes_2_2
=======
    P.Directory:='libcurl';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('libcurl.pp');

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testcurl.pp');
    P.Targets.AddExampleProgram('teststream.pp');
    P.Sources.AddExampleFiles('examples/*',P.Directory,false,'.');


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
