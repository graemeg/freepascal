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

    P:=AddPackage('opencl');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
=======
    P.Directory:='opencl';
{$endif ALLPACKAGES}
    P.Version:='2.2.2-0';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='opencl';
{$endif ALLPACKAGES}
    P.Version:='2.2.2-0';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='opencl';
{$endif ALLPACKAGES}
    P.Version:='2.2.2-0';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='opencl';
{$endif ALLPACKAGES}
    P.Version:='2.2.2-0';
>>>>>>> origin/cpstrnew
    P.Author := ' Dmitry "skalogryz" Boyarintsev; Kronos group';
    P.License := 'Library: modified BSD, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A OpenCL header';
    P.NeedLibC:= true;
    P.OSes:=[linux,win64,win32,darwin];
    P.CPUs:=[i386,x86_64];

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Dependencies.Add('opengl');

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('cl.pp');
    T:=P.Targets.AddUnit('cl_gl.pp');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
