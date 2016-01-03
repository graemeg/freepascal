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

    P:=AddPackage('fv');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Leon De Boer and Pierre Mueller';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.OSes := [beos,haiku,freebsd,darwin,iphonesim,solaris,netbsd,openbsd,linux,win32,win64,os2,emx,netware,netwlibc,go32v2,aix,dragonfly]+AllAmigaLikeOSes;
    P.Email := '';
    P.Description := 'Free Vision, a portable Turbo Vision clone.';
    P.NeedLibC:= false;

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.Dependencies.add('rtl-console');
    P.Dependencies.add('rtl-extra');
    P.Dependencies.add('morphunits',[morphos]);
    P.Dependencies.add('arosunits',[aros]);
    P.Dependencies.add('amunits',[amiga]);
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='fv';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('app.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('menus');
          AddUnit('histlist');
          AddUnit('dialogs');
          AddUnit('msgbox');
          AddUnit('fvconsts');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('asciitab.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvconsts');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('app');
        end;
    T:=P.Targets.AddUnit('buildfv.pas');
<<<<<<< HEAD
<<<<<<< HEAD
    T.Install := false; // Build-unit
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fvcommon');
          AddUnit('drivers');
          AddUnit('fvconsts');
          AddUnit('views');
          AddUnit('validate');
          AddUnit('msgbox');
          AddUnit('dialogs');
          AddUnit('menus');
          AddUnit('app');
          AddUnit('stddlg');
          AddUnit('asciitab');
          AddUnit('tabs');
          AddUnit('outline');
          AddUnit('memory');
          AddUnit('colortxt');
          AddUnit('statuses');
          AddUnit('histlist');
          AddUnit('inplong');
          AddUnit('editors');
          AddUnit('gadgets');
          AddUnit('timeddlg');
          AddUnit('time');
        end;
    T:=P.Targets.AddUnit('colortxt.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('dialogs');
          AddUnit('app');
          AddUnit('fvconsts');
        end;
    T:=P.Targets.AddUnit('dialogs.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
          AddUnit('fvconsts');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('validate');
          AddUnit('app');
          AddUnit('histlist');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('drivers.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('sysmsg');
          AddUnit('fvcommon');
          AddUnit('fvconsts');
        end;
    T:=P.Targets.AddUnit('editors.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('dialogs');
          AddUnit('fvcommon');
          AddUnit('fvconsts');
          AddUnit('app');
          AddUnit('stddlg');
          AddUnit('msgbox');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('fvcommon.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
        end;
    T:=P.Targets.AddUnit('fvconsts.pas');
    T:=P.Targets.AddUnit('gadgets.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvconsts');
          AddUnit('time');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('app');
        end;
    T:=P.Targets.AddUnit('histlist.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
        end;
    T:=P.Targets.AddUnit('inplong.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('dialogs');
          AddUnit('msgbox');
          AddUnit('fvconsts');
        end;
    T:=P.Targets.AddUnit('memory.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
        end;
    T:=P.Targets.AddUnit('menus.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('fvconsts');
        end;
    T:=P.Targets.AddUnit('msgbox.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('dialogs');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('app');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('outline.pas');
      with T.Dependencies do
        begin
          AddUnit('drivers');
          AddUnit('views');
        end;
    T:=P.Targets.AddUnit('statuses.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
          AddUnit('fvconsts');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('dialogs');
          AddUnit('msgbox');
          AddUnit('app');
        end;
    T:=P.Targets.AddUnit('stddlg.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvconsts');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('dialogs');
          AddUnit('validate');
          AddUnit('app');
          AddUnit('histlist');
          AddUnit('msgbox');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('sysmsg.pas');
      with T.Dependencies do
        begin
          AddInclude('unixsmsg.inc',AllUnixOSes);
          AddInclude('w32smsg.inc',[win32,win64]);
          AddInclude('go32smsg.inc',[go32v2]);
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('sysmsg.pas');
      with T.Dependencies do
        begin
          AddInclude('unixsmsg.inc');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
    T:=P.Targets.AddUnit('tabs.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('fvconsts');
          AddUnit('fvcommon');
          AddUnit('dialogs');
        end;
    T:=P.Targets.AddUnit('timeddlg.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('dialogs');
          AddUnit('fvconsts');
          AddUnit('drivers');
          AddUnit('views');
          AddUnit('app');
          AddUnit('msgbox');
        end;
    T:=P.Targets.AddUnit('time.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
        end;
    T:=P.Targets.AddUnit('validate.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
          AddUnit('fvconsts');
          AddUnit('msgbox');
        end;
    T:=P.Targets.AddUnit('views.pas');
      with T.Dependencies do
        begin
          AddInclude('platform.inc');
          AddUnit('fvcommon');
          AddUnit('drivers');
          AddUnit('fvconsts');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.ExamplePath.Add('src');
    P.Targets.AddExampleProgram('examples/testapp.pas');
    P.Targets.AddExampleProgram('src/platform.inc');
    // 'examples/Makefile
    // 'examples/testapp.lpi
    // 'examples/Makefile.fpc


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
