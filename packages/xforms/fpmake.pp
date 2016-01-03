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

    P:=AddPackage('xforms');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes := [beos,haiku,freebsd,solaris,netbsd,openbsd,linux,aix,dragonfly];
    // Do not build x11 on iPhone (=arm-darwin)
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];

    P.IncludePath.Add('src');
    P.Dependencies.Add('x11');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='xforms';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
//    P.Dependencies.Add('x11');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('xforms.pp');
      with T.Dependencies do
        begin
          AddInclude('cursorfont.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddProgram('fd2pascal.pp');

<<<<<<< HEAD
=======
    
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
    P.Sources.AddSrc('README.txt');

    P.ExamplePath.Add('examples/');
    P.Targets.AddExampleProgram('newbutton.pp');
    P.Targets.AddExampleProgram('colbrowser.pp');
    P.Targets.AddExampleProgram('chartstrip.pp');
    P.Targets.AddExampleProgram('pup.pp');
    P.Targets.AddExampleProgram('ll.pp');
    P.Targets.AddExampleProgram('pushme.pp');
    P.Targets.AddExampleProgram('lalign.pp');
    P.Targets.AddExampleProgram('flclock.pp');
    P.Targets.AddExampleProgram('choice.pp');
    P.Targets.AddExampleProgram('positioner.pp');
    P.Targets.AddExampleProgram('objpos.pp');
    P.Targets.AddExampleProgram('canvas.pp');
    P.Targets.AddExampleProgram('colsel1.pp');
    P.Targets.AddExampleProgram('fbrowse.pp');
    P.Targets.AddExampleProgram('secretinput.pp');
    P.Targets.AddExampleProgram('group.pp');
    P.Targets.AddExampleProgram('inputall.pp');
    P.Targets.AddExampleProgram('pushbutton.pp');
    P.Targets.AddExampleProgram('touchbutton.pp');
    P.Targets.AddExampleProgram('buttonall.pp');
    P.Targets.AddExampleProgram('fonts.pp');
    P.Targets.AddExampleProgram('multilabel.pp');
    P.Targets.AddExampleProgram('sliderall.pp');
    P.Targets.AddExampleProgram('goodies.pp');
    P.Targets.AddExampleProgram('minput.pp');
    P.Targets.AddExampleProgram('counter.pp');
    P.Targets.AddExampleProgram('borderwidth.pp');
    P.Targets.AddExampleProgram('browserall.pp');
    P.Targets.AddExampleProgram('ldial.pp');
    P.Targets.AddExampleProgram('chartall.pp');
    P.Targets.AddExampleProgram('arrowbutton.pp');
    P.Targets.AddExampleProgram('longlabel.pp');
    P.Targets.AddExampleProgram('fdial.pp');
    P.Targets.AddExampleProgram('xyplotover.pp');
    P.Targets.AddExampleProgram('fbrowse1.pp');
    P.Targets.AddExampleProgram('objinactive.pp');
    P.Targets.AddExampleProgram('menu.pp');
    P.Targets.AddExampleProgram('free1.pp');
    P.Targets.AddExampleProgram('browserop.pp');
    P.Targets.AddExampleProgram('cursor.pp');
    P.Targets.AddExampleProgram('yesno.pp');
    P.Targets.AddExampleProgram('invslider.pp');
    P.Targets.AddExampleProgram('objreturn.pp');
    P.Targets.AddExampleProgram('colsel.pp');
    P.Targets.AddExampleProgram('butttypes.pp');
    P.Targets.AddExampleProgram('iconify.pp');
    P.Targets.AddExampleProgram('boxtype.pp');
    P.Targets.AddExampleProgram('ndial.pp');
    // 'Makefile
    // 'crab45.xpm
    // 'nomail.xbm
    // 'crab.xpm
    // 'Makefile.fpc
    // 'bm2.xbm
    // 'srs.xbm
    // 'bm1.xbm
    // 'porsche.xpm
=======
          AddUnit('xlib');
          AddUnit('xresource');
        end;
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('xlib');
          AddUnit('xresource');
        end;
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
