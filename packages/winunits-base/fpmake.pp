{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('winunits-base');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='win';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=[win32,win64];
    P.Author := 'Florian Klaempfl, Marco van de Voort';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Base Delphi compatible windows headers units outside the RTL';
    P.NeedLibC:= true;

    P.Dependencies.Add('fcl-registry');
    P.Dependencies.Add('fcl-base');

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='winunits-base';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=[win32,win64];

    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('buildwinutilsbase.pp');
      T.Install:=False;
      with T.Dependencies do
        begin
          AddUnit('flatsb');
          AddUnit('winver');
          AddUnit('mmsystem');
          AddUnit('comconst');
          AddUnit('commctrl');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('commdlg');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          AddUnit('comobj');
          AddUnit('ole2');
          AddUnit('activex');
          AddUnit('shellapi');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('shlwapi');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          AddUnit('shlobj');
          AddUnit('oleserver');
          AddUnit('shfolder');
          AddUnit('richedit');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('imagehlp');
	  AddUnit('wininet');
          AddUnit('uxtheme');
          AddInclude('tmschema.inc');
          AddUnit('dwmapi');
          AddUnit('multimon');
          AddUnit('htmlhelp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('winutils');
          AddUnit('comserv');
          AddUnit('winspool');
          AddUnit('imm');
          AddUnit('imm_dyn');
          AddUnit('nb30');
          AddUnit('win9xwsmanager', [win32]);
		  AddUnit('stdole2');
		  AddUnit('eventsink');
		  AddUnit('typelib');
		  AddUnit('libkinect10');
          AddUnit('urlmon');
=======
          AddUnit('winspool');
>>>>>>> graemeg/cpstrnew
=======
          AddUnit('winspool');
>>>>>>> graemeg/cpstrnew
=======
          AddUnit('winspool');
>>>>>>> graemeg/cpstrnew
=======
          AddUnit('winspool');
>>>>>>> origin/cpstrnew
        end;
    T:=P.Targets.AddImplicitUnit('activex.pp');
    T:=P.Targets.AddImplicitUnit('urlmon.pp');
    T:=P.Targets.AddImplicitUnit('comconst.pp');
    T:=P.Targets.AddImplicitUnit('commctrl.pp');
    T:=P.Targets.AddImplicitUnit('commdlg.pp');
=======
=======
>>>>>>> origin/fixes_2_2
		  AddUnit('wininet');
        end;
    T:=P.Targets.AddImplicitUnit('activex.pp');
    T:=P.Targets.AddImplicitUnit('comconst.pp');
    T:=P.Targets.AddImplicitUnit('commctrl.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddImplicitUnit('comobj.pp');
    T:=P.Targets.AddImplicitUnit('flatsb.pp');
    T:=P.Targets.AddImplicitUnit('mmsystem.pp');
    T:=P.Targets.AddImplicitUnit('ole2.pp');
    T:=P.Targets.AddImplicitUnit('oleserver.pp');
    T:=P.Targets.AddImplicitUnit('richedit.pp');
    T:=P.Targets.AddImplicitUnit('shellapi.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddImplicitUnit('shlwapi.pp');
    T:=P.Targets.AddImplicitUnit('shfolder.pp');
    T:=P.Targets.AddImplicitUnit('shlobj.pp');
    T:=P.Targets.AddImplicitUnit('winver.pp');
    T:=P.Targets.AddImplicitUnit('wininet.pp');
    T:=P.Targets.AddImplicitUnit('imagehlp.pp');
    T:=P.Targets.AddImplicitUnit('commdlg.pp');
    T:=P.Targets.AddImplicitUnit('wininet.pp');
    T:=P.Targets.AddImplicitUnit('uxtheme.pp');
    T:=P.Targets.AddImplicitUnit('multimon.pp');
    T:=P.Targets.AddImplicitUnit('dwmapi.pp');
    T:=P.Targets.AddImplicitUnit('htmlhelp.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddImplicitUnit('winutils.pp');
    T:=P.Targets.AddImplicitUnit('comserv.pp');
    T:=P.Targets.AddImplicitUnit('winspool.pp');
    T:=P.Targets.AddImplicitUnit('imm.pas');
    T:=P.Targets.AddImplicitUnit('imm_dyn.pas');
    T:=P.Targets.AddImplicitUnit('nb30.pp');
    T:=P.Targets.AddImplicitUnit('win9xwsmanager.pp', [win32]);
	T:=P.Targets.AddImplicitUnit('stdole2.pas');
	T:=P.Targets.AddImplicitUnit('eventsink.pp');
	T:=P.Targets.AddImplicitUnit('typelib.pas');
	T:=P.Targets.AddImplicitUnit('libkinect10.pp');
=======
    T:=P.Targets.AddImplicitUnit('winspool.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddImplicitUnit('winspool.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddImplicitUnit('winspool.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddImplicitUnit('winspool.pp');
>>>>>>> origin/cpstrnew
    T.Dependencies.AddInclude('tmschema.inc');
    P.ExamplePath.Add('tests/');
    P.Targets.AddExampleProgram('testcom1.pp');
    P.Targets.AddExampleProgram('OOTest.pp');
    P.Targets.AddExampleProgram('OOHelper.pp');
    P.Targets.AddExampleProgram('testver.pp');
    P.Targets.AddExampleProgram('testcom2.pp');

=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddImplicitUnit('shfolder.pp');
    T:=P.Targets.AddImplicitUnit('shlobj.pp');
    T:=P.Targets.AddImplicitUnit('winver.pp');
    T:=P.Targets.AddImplicitUnit('imagehlp.pp');
    T:=P.Targets.AddImplicitUnit('commdlg.pp');
    T:=P.Targets.AddImplicitUnit('wininet.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
