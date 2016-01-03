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

    P:=AddPackage('x11');
<<<<<<< HEAD
<<<<<<< HEAD
    P.Description := 'Interface units for X Window GUI libraries (X11).';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=[beos,haiku,freebsd,solaris,netbsd,openbsd,linux,os2,emx,aix,dragonfly];
    // Do not build x11 on iPhone (=arm-darwin)
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='x11';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('cursorfont.pp');
    T:=P.Targets.AddUnit('keysym.pp');
    T:=P.Targets.AddUnit('xatom.pp');
    T:=P.Targets.AddUnit('xcms.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xf86dga.pp');
      with T.Dependencies do
        begin
          AddInclude('xf86dga1.inc');
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xf86vmode.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xinerama.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xi.pp');
    T:=P.Targets.AddUnit('xkblib.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
          AddUnit('xkb');
        end;
    T:=P.Targets.AddUnit('xkb.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
          AddUnit('xi');
        end;
    T:=P.Targets.AddUnit('xlib.pp');
    T:=P.Targets.AddUnit('x.pp');
    T:=P.Targets.AddUnit('xrandr.pp');
      with T.Dependencies do
        begin
          AddInclude('randr.inc');
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xrender.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xresource.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xshm.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
        end;
    T:=P.Targets.AddUnit('xutil.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
          AddUnit('keysym');
        end;
    T:=P.Targets.AddUnit('xvlib.pp');
      with T.Dependencies do
        begin
          AddUnit('xlib');
          AddUnit('xshm');
        end;
    T:=P.Targets.AddUnit('xv.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fontconfig.pas');
    T.Dependencies.AddUnit('xlib');
    T:=P.Targets.AddUnit('xft.pas');
    T.Dependencies.AddUnit('xlib');
    T.Dependencies.AddUnit('xrender');
    T.Dependencies.AddUnit('fontconfig');
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
