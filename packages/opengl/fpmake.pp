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

    P:=AddPackage('opengl');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='ogl';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=AllUnixOSes+[Win32,Win64]+[MorphOS]-[Android];

    P.Dependencies.Add('x11',AllUnixOSes-[darwin,iphonesim]);
    if Defaults.CPU<>arm then
      P.Dependencies.Add('x11',[darwin]);
    P.Dependencies.Add('morphunits',[morphos]);

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddImplicitUnit('freeglut.pp',AllOSes-[morphos]);
    T:=P.Targets.AddUnit('glext.pp',AllOSes-[morphos]);
    T:=P.Targets.AddUnit('gl.pp');
    T:=P.Targets.AddUnit('glu.pp');
    T:=P.Targets.AddUnit('glut.pp');
    // T.Dependencies.Add('freeglut',AllOSes-[morphos]);
    T:=P.Targets.AddUnit('glx.pp',AllUnixOSes-[darwin,iphonesim]);
    if Defaults.CPU<>arm then
      T:=P.Targets.AddUnit('glx.pp',[darwin]);

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('radblur.pp');
    P.Targets.AddExampleProgram('bounce.pp');
    P.Targets.AddExampleProgram('morph3d.pp');
    P.Targets.AddExampleProgram('glutdemo.pp');
    P.Targets.AddExampleProgram('glxtest.pp');
    P.Sources.AddSrc('readme');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='opengl';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=AllUnixOSes+[Win32,Win64];

    P.Dependencies.Add('x11',AllUnixOSes);

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('glext.pp');
    T:=P.Targets.AddUnit('gl.pp');
    T:=P.Targets.AddUnit('glu.pp');
    T:=P.Targets.AddUnit('glut.pp');
    T:=P.Targets.AddUnit('glx.pp',AllUnixOSes);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
