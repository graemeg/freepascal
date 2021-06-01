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
    P.ShortName:='fclx';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Options.Add('-S2h');
    D:=P.Dependencies.Add('fcl-base');
    D:=P.Dependencies.Add('iconvenc',[linux,darwin,iphonesim,ios,freebsd,haiku,beos,aix]);

    P.Author := 'Sebastian Guenther, Sergei Gorelkin and FPC development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'XML and DOM parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

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
    T.ResourceStrings:=True;
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
    T.ResourceStrings:=True;
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
    T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('xmlconf.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('xmlread');
          AddUnit('xmlwrite');
        end;
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
    T:=P.Targets.AddUnit('xmlread.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('xmlutils');
          AddUnit('xmlreader');
          AddUnit('xmltextreader');
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
          AddInclude('xpathkw.inc');
          AddUnit('dom');
        end;
    T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('sax_xml.pp');
      with T.Dependencies do
        begin
          AddUnit('sax');
          AddUnit('dom');
          AddUnit('htmldefs');
        end;
    T:=P.Targets.AddUnit('xmliconv.pas',[linux,freebsd,darwin,iphonesim,ios,haiku,beos,aix]);
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


    P.Sources.AddSrc('src/README.txt');


    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('tests/xmlts.pp');
    P.Targets.AddExampleProgram('tests/domunit.pp');
    P.Targets.AddExampleProgram('tests/testgen.pp');
    // 'tests/api.xml
    // 'tests/README_DOM
    // 'tests/README
    // 'tests/template.xml


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

