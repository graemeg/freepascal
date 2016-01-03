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
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fclb';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
<<<<<<< HEAD
    P.Version:='3.1.1';
    P.Dependencies.Add('univint',[Darwin,iPhoneSim]);
    P.Dependencies.Add('fcl-res');
    p.Dependencies.Add('rtl-objpas');
=======
    P.Version:='2.2.2-0';
    P.Dependencies.Add('univint',[Darwin]);
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

    P.Author := '<various>';
    P.License := 'LGPL with modification, ';
    P.Email := '';
    P.Description := 'Base library of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16];

    P.SourcePath.Add('src');
    P.SourcePath.Add('src/$(OS)');
    P.SourcePath.Add('src/unix',AllUnixOSes);
    P.SourcePath.Add('src/win',AllWindowsOSes);
    P.IncludePath.Add('src');
    P.IncludePath.Add('src/$(OS)');
    P.IncludePath.Add('src/unix',AllUnixOSes);
<<<<<<< HEAD
    P.IncludePath.Add('src/win',AllWindowsOSes-[WinCE]);
=======
    P.IncludePath.Add('src/win',AllWindowsOSes);
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
    P.IncludePath.Add('src/dummy',AllOSes);

    T:=P.Targets.AddUnit('ascii85.pp');
    T:=P.Targets.AddUnit('avl_tree.pp');
    T:=P.Targets.AddUnit('base64.pp');
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('singleinstance.pp');
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
    with T.Dependencies do
      AddUnit('singleinstance');
=======
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-base';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.Dependencies.Add('winunits-jedi',[Win32,Win64]);

    P.SourcePath.Add('src');
    P.SourcePath.Add('src/unix',AllUnixOSes);
    P.SourcePath.Add('src/win',AllWindowsOSes);
    P.SourcePath.Add('src/$(OS)',AllOSes-AllWindowsOSes-AllUnixOSes);
    P.IncludePath.Add('src');
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/win',AllWindowsOSes);
    P.IncludePath.Add('src/$(OS)',AllOSes-AllWindowsOSes-AllUnixOSes);

    T:=P.Targets.AddUnit('avl_tree.pp');
    T:=P.Targets.AddUnit('base64.pp');
    T:=P.Targets.AddUnit('blowfish.pp');
    T:=P.Targets.AddUnit('bufstream.pp');
    T:=P.Targets.AddUnit('cachecls.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('contnrs.pp');
    T:=P.Targets.AddUnit('custapp.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('daemonapp.pp',AllWindowsOSes+AllUnixOSes);
      with T.Dependencies do
        begin
          AddInclude('daemonapp.inc');
          AddUnit('custapp');
          AddUnit('eventlog');
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('eventlog.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('eventlog.inc');
        end;
    T:=P.Targets.AddUnit('fptimer.pp',AllWindowsOSes+AllUnixOSes);
    T:=P.Targets.AddUnit('gettext.pp');
    T:=P.Targets.AddUnit('idea.pp');
<<<<<<< HEAD
<<<<<<< HEAD

    T:=P.Targets.AddUnit('inicol.pp');

=======
    T:=P.Targets.AddUnit('inicol.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('inicol.pp');
>>>>>>> origin/fixes_2_2
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('inifiles');
        end;
    T:=P.Targets.AddUnit('inifiles.pp');
      with T.Dependencies do
        begin
          AddUnit('contnrs');
        end;
    T:=P.Targets.AddUnit('iostream.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('nullstream.pp');
    T:=P.Targets.AddUnit('maskutils.pp');
      T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('libtar.pp');
    T:=P.Targets.AddUnit('maskutils.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('libtar.pp');
    T:=P.Targets.AddUnit('maskutils.pp');
>>>>>>> origin/fixes_2_2
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
    T:=P.Targets.AddUnit('streamio.pp');
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('fptemplate.pp');
      T.ResourceStrings:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('syncobjs.pp',AllOSes-[GO32v2,EMX,nativent]);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('syncobjs.pp',AllOSes-[GO32v2,OS2,EMX]);
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('syncobjs.pp',AllOSes-[GO32v2,OS2,EMX]);
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('syncobjs.pp',AllOSes-[GO32v2,OS2,EMX]);
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;

<<<<<<< HEAD
    T:=P.Targets.AddUnit('fileinfo.pp');
    T:=P.Targets.addUnit('fpmimetypes.pp');
    T:=P.Targets.AddUnit('csvreadwrite.pp');
    T:=P.Targets.addUnit('csvdocument.pp');
    With T.Dependencies do
      begin
      AddUnit('csvreadwrite');
      AddUnit('contnrs');
      end;
    T:=P.Targets.addUnit('advancedipc.pp');
      T.ResourceStrings:=true;
=======
    // Windows units
    T:=P.Targets.AddUnit('fileinfo.pp',AllWindowsOSes);

>>>>>>> graemeg/cpstrnew
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
      T:=P.Targets.AddExampleProgram('dbugsrv.pp');
      T:=P.Targets.AddExampleProgram('debugtest.pp');
      T:=P.Targets.AddExampleProgram('doecho.pp');
      T:=P.Targets.AddExampleProgram('dparser.pp');
      T:=P.Targets.AddExampleProgram('dsockcli.pp');
      T:=P.Targets.AddExampleProgram('dsocksvr.pp');
      T:=P.Targets.AddExampleProgram('fpdoc.dtd');
      T:=P.Targets.AddExampleProgram('fstream.pp');
      T:=P.Targets.AddExampleProgram('htdump.pp');
      T:=P.Targets.AddExampleProgram('ipcclient.pp');
      T:=P.Targets.AddExampleProgram('ipcserver.pp');
      T:=P.Targets.AddExampleProgram('isockcli.pp');
      T:=P.Targets.AddExampleProgram('isocksvr.pp');
      T:=P.Targets.AddExampleProgram('istream.pp');
      T:=P.Targets.AddExampleProgram('list.pp');
      T:=P.Targets.AddExampleProgram('mstream.pp');
      T:=P.Targets.AddExampleProgram('poolmm1.pp');
      T:=P.Targets.AddExampleProgram('poolmm2.pp');
      T:=P.Targets.AddExampleProgram('restest.pp');
      T:=P.Targets.AddExampleProgram('showver.pp');
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
      // showver.rc
      // showver.res
      // simple.xml
      // parser.dat
      // testcgi.html
=======
=======
>>>>>>> origin/fixes_2_2

    // Windows units
    T:=P.Targets.AddUnit('ServiceManager.pas',[Win32,Win64]);
    T:=P.Targets.AddUnit('fileinfo',AllWindowsOSes);

    // Additional sources
    P.Sources.AddSrcFiles('src/win/fclel.*');
<<<<<<< HEAD
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
