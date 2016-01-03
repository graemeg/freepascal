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

    P:=AddPackage('ncurses');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := [beos,haiku,freebsd,darwin,iphonesim,solaris,netbsd,openbsd,linux,aix,dragonfly];
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='ncurses';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('menu.pp');
      with T.Dependencies do
        begin
          AddInclude('eti.inc');
          AddUnit('ncurses');
        end;
    T:=P.Targets.AddUnit('ncrt.pp');
      with T.Dependencies do
        begin
          AddInclude('ncrt.inc');
          AddUnit('ncurses');
        end;
    T:=P.Targets.AddUnit('ncurses.pp');
    T:=P.Targets.AddUnit('ocrt.pp');
      with T.Dependencies do
        begin
          AddInclude('ncrt.inc');
          AddInclude('pxpic.inc');
          AddUnit('ncurses');
          AddUnit('panel');
          AddUnit('menu');
        end;
    T:=P.Targets.AddUnit('panel.pp');
      with T.Dependencies do
        begin
          AddUnit('ncurses');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('form.pp');
      with T.Dependencies do
        begin
          AddUnit('ncurses');
        end;

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('t2menu.pp');
    P.Targets.AddExampleProgram('t3form.pp');
    P.Targets.AddExampleProgram('t1panel.pp');
    P.Targets.AddExampleProgram('screen_demo.pp');
    P.Targets.AddExampleProgram('tevent.pp');
    P.Targets.AddExampleProgram('t1form.pp');
    P.Targets.AddExampleProgram('tclock.pp');
    P.Targets.AddExampleProgram('menu_demo.pp');
    P.Targets.AddExampleProgram('twindow.pp');
    P.Targets.AddExampleProgram('tpad.pp');
    P.Targets.AddExampleProgram('t1menu.pp');
    P.Targets.AddExampleProgram('t2panel.pp');
    P.Targets.AddExampleProgram('tbackground.pp');
    P.Targets.AddExampleProgram('tnlshello.pp');
    P.Targets.AddExampleProgram('edit_demo.pp');
    P.Targets.AddExampleProgram('firework.pp');
    P.Targets.AddExampleProgram('db_demo.pp');
    P.Targets.AddExampleProgram('t2form.pp');
    P.Targets.AddExampleProgram('ocrt_demo.pp');
    P.Targets.AddExampleProgram('tmouse.pp');
    // 'tnlshello_ru_UTF8.pot

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
