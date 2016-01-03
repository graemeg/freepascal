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
    P:=AddPackage('pxlib');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.OSes:=[Linux,beos,haiku,win32,freebsd,openbsd,netbsd,dragonfly];
    P.Version:='3.1.1';
    T:=P.Targets.AddUnit('src/pxlib.pp');
    T:=P.Targets.AddExampleunit('examples/ppxview.pp');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    P.OSes:=[Linux,beos,win32,darwin,freebsd,openbsd,netbsd];
    P.Version:='2.2.2-0';
    T:=P.Targets.AddUnit('src/pxlib.pp');
      T.OSes:=[Linux,beos,win32,darwin,freebsd,openbsd,netbsd];
    T:=P.Targets.AddExampleunit('examples/ppxview.pp');
      T.OSes:=[Linux,beos,win32,darwin,freebsd,openbsd,netbsd];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='pxlib';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    T:=P.Targets.AddUnit('src/pxlib.pp');
      T.OSes:=[Linux];
    T:=P.Targets.AddExampleunit('examples/ppxview.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
