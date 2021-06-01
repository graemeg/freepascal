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

    P:=AddPackage('fcl-base');
    P.ShortName:='fclb';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('univint',[Darwin,iPhoneSim,ios]);
    p.Dependencies.Add('rtl-objpas');

    P.Author := '<various>';
    P.License := 'LGPL with modification, ';
    P.Email := '';
    P.Description := 'Base library of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.SourcePath.Add('src/$(OS)');
    P.SourcePath.Add('src/unix',AllUnixOSes);
    P.SourcePath.Add('src/win',AllWindowsOSes);
    P.IncludePath.Add('src');
    P.IncludePath.Add('src/$(OS)');
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/win',AllWindowsOSes-[WinCE]);
    P.IncludePath.Add('src/dummy',AllOSes);

    T:=P.Targets.AddUnit('ascii85.pp');
    T:=P.Targets.AddUnit('avl_tree.pp');
    T:=P.Targets.AddUnit('base64.pp');
    T:=P.Targets.AddUnit('pascodegen.pp');
    T:=P.Targets.AddUnit('fpobserver.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('blowfish.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('bufstream.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('cachecls.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('contnrs.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('singleinstance.pp');
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
    with T.Dependencies do
      AddUnit('singleinstance');
    T:=P.Targets.AddUnit('eventlog.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('eventlog.inc');
        end;
    T:=P.Targets.AddUnit('fptimer.pp',AllWindowsOSes+AllUnixOSes);
    T:=P.Targets.AddUnit('gettext.pp');
    T:=P.Targets.AddUnit('idea.pp');

    T:=P.Targets.AddUnit('inicol.pp');

      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('inifiles');
        end;
    T:=P.Targets.AddUnit('inifiles.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('contnrs');
        end;
    T:=P.Targets.AddUnit('iostream.pp');
    T:=P.Targets.AddUnit('nullstream.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('maskutils.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('pooledmm.pp');
    T:=P.Targets.AddUnit('rtfpars.pp');
      with T.Dependencies do
        begin
          AddInclude('rtfdata.inc');
        end;
    T:=P.Targets.AddUnit('rttiutils.pp');
    T:=P.Targets.AddUnit('streamcoll.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('streamex.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('streamio.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('fptemplate.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('syncobjs.pp',AllOSes-[go32v2,nativent,atari]);
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('uriparser.pp');
    T:=P.Targets.AddUnit('wformat.pp');
    T:=P.Targets.AddUnit('whtml.pp');
      with T.Dependencies do
        begin
          AddUnit('wformat');
        end;
    T:=P.Targets.AddUnit('wtex.pp');
      with T.Dependencies do
        begin
          AddUnit('wformat');
        end;
    T:=P.Targets.AddUnit('fpexprpars.pp');
      T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('csvreadwrite.pp');
    T:=P.Targets.addUnit('csvdocument.pp');
    With T.Dependencies do
      begin
      AddUnit('csvreadwrite');
      AddUnit('contnrs');
      end;
    T:=P.Targets.addUnit('advancedipc.pp',AllOSes-[atari]);
      T.ResourceStrings:=true;
    T:=P.Targets.addUnit('advancedsingleinstance.pas',AllOSes-[atari]);
      T.ResourceStrings:=true;
    // Additional sources
    P.Sources.AddSrcFiles('src/win/fclel.*', P.Directory);
    // Install windows resources
    P.InstallFiles.Add('src/win/fclel.res',AllWindowsOSes,'$(unitinstalldir)');

    // Examples
    P.ExamplePath.Add('examples');
      T:=P.Targets.AddExampleProgram('asiotest.pp');
      T:=P.Targets.AddExampleProgram('b64dec.pp');
      T:=P.Targets.AddExampleProgram('b64enc.pp');
      T:=P.Targets.AddExampleProgram('b64.pp');
      T:=P.Targets.AddExampleProgram('b64test2.pp');
      T:=P.Targets.AddExampleProgram('b64test.pp');
      T:=P.Targets.AddExampleProgram('base64decodingtestcase.pas');
      T:=P.Targets.AddExampleProgram('cachetest.pp');
      T:=P.Targets.AddExampleProgram('cfgtest.pp');
      T:=P.Targets.AddExampleProgram('debugtest.pp');
      T:=P.Targets.AddExampleProgram('doecho.pp');
      T:=P.Targets.AddExampleProgram('dparser.pp');
      T:=P.Targets.AddExampleProgram('dsockcli.pp');
      T:=P.Targets.AddExampleProgram('dsocksvr.pp');
      T:=P.Targets.AddExampleProgram('fpdoc.dtd');
      T:=P.Targets.AddExampleProgram('fstream.pp');
      T:=P.Targets.AddExampleProgram('htdump.pp');
      T:=P.Targets.AddExampleProgram('isockcli.pp');
      T:=P.Targets.AddExampleProgram('isocksvr.pp');
      T:=P.Targets.AddExampleProgram('istream.pp');
      T:=P.Targets.AddExampleProgram('list.pp');
      T:=P.Targets.AddExampleProgram('mstream.pp');
      T:=P.Targets.AddExampleProgram('poolmm1.pp');
      T:=P.Targets.AddExampleProgram('poolmm2.pp');
      T:=P.Targets.AddExampleProgram('restest.pp');
      T:=P.Targets.AddExampleProgram('sockcli.pp');
      T:=P.Targets.AddExampleProgram('socksvr.pp');
      T:=P.Targets.AddExampleProgram('sstream.pp');
      T:=P.Targets.AddExampleProgram('stringl.pp');
      T:=P.Targets.AddExampleProgram('tarmakerconsgzip.pas');
      T:=P.Targets.AddExampleProgram('tarmakercons.pas');
      T:=P.Targets.AddExampleProgram('testapp.pp');
      T:=P.Targets.AddExampleProgram('testbf.pp');
      T:=P.Targets.AddExampleProgram('testbs.pp');
      T:=P.Targets.AddExampleProgram('testcgi.pp');
      T:=P.Targets.AddExampleProgram('testcont.pp');
      T:=P.Targets.AddExampleProgram('testez.pp');
      T:=P.Targets.AddExampleProgram('testhres.pp');
      T:=P.Targets.AddExampleProgram('testnres.pp');
      T:=P.Targets.AddExampleProgram('testol.pp');
      T:=P.Targets.AddExampleProgram('testproc.pp');
      T:=P.Targets.AddExampleProgram('testreg.pp');
      T:=P.Targets.AddExampleProgram('testrhre.pp');
      T:=P.Targets.AddExampleProgram('testrnre.pp');
      T:=P.Targets.AddExampleProgram('testrsre.pp');
      T:=P.Targets.AddExampleProgram('testrtf.pp');
      T:=P.Targets.AddExampleProgram('testser.pp');
      T:=P.Targets.AddExampleProgram('testsres.pp');
      T:=P.Targets.AddExampleProgram('testtimer.pp');
      T:=P.Targets.AddExampleProgram('testunzip.pp');
      T:=P.Targets.AddExampleProgram('testur.pp');
      T:=P.Targets.AddExampleProgram('testweb.pp');
      T:=P.Targets.AddExampleProgram('testz2.pp');
      T:=P.Targets.AddExampleProgram('testzip.pp');
      T:=P.Targets.AddExampleProgram('testz.pp');
      T:=P.Targets.AddExampleProgram('threads.pp');
      T:=P.Targets.AddExampleProgram('tidea.pp');
      T:=P.Targets.AddExampleProgram('tstelcmd.pp');
      T:=P.Targets.AddExampleProgram('tstelgtk.pp');
      T:=P.Targets.AddExampleProgram('txmlreg.pp');
      T:=P.Targets.AddExampleProgram('xmldump.pp');

      // example data files.
      // README
      // kword.xml
      // overview.rtf
      // simple.xml
      // parser.dat
      // testcgi.html

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

