{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
  D : TDependency;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fcl-xml');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fclx';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Options.Add('-S2h');
    D:=P.Dependencies.Add('fcl-base');
      D.Version:='3.1.1';
    D:=P.Dependencies.Add('iconvenc',[linux,darwin,iphonesim,freebsd,haiku,beos,aix]);

    P.Author := 'Sebastian Guenther, Sergei Gorelkin and FPC development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'XML and DOM parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16];
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-xml';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    D:=P.Dependencies.Add('paszlib');
      D.Version:='2.2.4';
    D:=P.Dependencies.Add('fcl-base');
      D.Version:='2.2.4';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('dom_html.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
        end;
    T:=P.Targets.AddUnit('dom.pp');
      with T.Dependencies do
        begin
          AddUnit('xmlutils');
        end;
    T:=P.Targets.AddUnit('htmldefs.pp');
    T:=P.Targets.AddUnit('htmlelements.pp');
      with T.Dependencies do
        begin
          AddInclude('tagsintf.inc');
          AddInclude('tagsimpl.inc');
          AddUnit('dom');
          AddUnit('htmldefs');
        end;
    T:=P.Targets.AddUnit('htmlwriter.pp');
      with T.Dependencies do
        begin
          AddInclude('wtagsintf.inc');
          AddInclude('wtagsimpl.inc');
          AddUnit('dom');
          AddUnit('htmlelements');
          AddUnit('htmldefs');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings:=True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('htmwrite.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('htmldefs');
        end;
    T:=P.Targets.AddUnit('sax_html.pp');
      with T.Dependencies do
        begin
          AddUnit('sax');
          AddUnit('dom');
          AddUnit('dom_html');
          AddUnit('htmldefs');
        end;
    T:=P.Targets.AddUnit('sax.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings:=True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('xhtml.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('dom_html');
        end;
    T:=P.Targets.AddUnit('xmlcfg.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('xmlread');
          AddUnit('xmlwrite');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings:=True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('xmlconf.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('xmlread');
          AddUnit('xmlwrite');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('xmlreader.pp');
      with T.Dependencies do
        begin
          AddUnit('xmlutils');
        end;
    T:=P.Targets.AddUnit('xmltextreader.pp');
      with T.Dependencies do
        begin
          AddUnit('xmlutils');
          AddUnit('xmlreader');
          AddUnit('dtdmodel');
        end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('xmlread.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('xmlutils');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('xmlreader');
          AddUnit('xmltextreader');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
    T:=P.Targets.AddUnit('xmlstreaming.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
        end;
    T:=P.Targets.AddUnit('xmlutils.pp');
      with T.Dependencies do
        begin
          AddInclude('names.inc');
        end;
    T:=P.Targets.AddUnit('xmlwrite.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
        end;
    T:=P.Targets.AddUnit('xpath.pp');
      with T.Dependencies do
        begin
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('xpathkw.inc');
          AddUnit('dom');
        end;
    T.ResourceStrings:=True;
=======
          AddUnit('dom');
        end;
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('dom');
        end;
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('sax_xml.pp');
      with T.Dependencies do
        begin
          AddUnit('sax');
          AddUnit('dom');
          AddUnit('htmldefs');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('xmliconv.pas',[linux,freebsd,darwin,iphonesim,haiku,beos,aix]);
      with T.Dependencies do
        begin
          AddUnit('xmlread');
        end;
    T:=P.Targets.AddUnit('xmliconv_windows.pas',[win32,win64]);
      with T.Dependencies do
        begin
          AddUnit('xmlread');
        end;
    T:=P.Targets.AddUnit('dtdmodel.pp');
      with T.Dependencies do
        begin
          AddUnit('xmlutils');
        end;


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
    P.Sources.AddSrc('src/README.txt');


    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('tests/xmlts.pp');
    P.Targets.AddExampleProgram('tests/domunit.pp');
    P.Targets.AddExampleProgram('tests/testgen.pp');
    // 'tests/api.xml
    // 'tests/README_DOM
    // 'tests/README
    // 'tests/template.xml

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
