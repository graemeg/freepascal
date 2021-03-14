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

    P:=AddPackage('fcl-net');
    P.ShortName:='fcln';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('fcl-passrc');
    P.Dependencies.Add('fcl-async',[linux,freebsd,netbsd,openbsd,dragonfly]);
    P.Dependencies.Add('rtl-extra');

    P.Author := 'Sebastian Guenther and Free Pascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Network related parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=P.OSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/win',AllWindowsOSes);
    P.IncludePath.Add('src/os2',[EMX]);
    P.IncludePath.Add('src/amiga',[morphos]);
    P.IncludePath.Add('src/$(OS)',AllOSes-AllWindowsOSes-AllUnixOSes-[EMX]);

    // IP and Sockets
    T:=P.Targets.AddUnit('netdb.pp',AllUnixOSes);
    T:=P.Targets.AddUnit('sslbase.pp');
    T:=P.Targets.AddUnit('resolve.pp',AllUnixOSes+AllWindowsOSes+AllAmigaLikeOSes+[OS2,EMX]);
      with T.Dependencies do
        begin
          AddInclude('resolve.inc');
          AddUnit('netdb',AllUnixOSes);
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('ssockets.pp',AllUnixOSes+AllWindowsOSes+AllAmigaLikeOSes+[OS2,EMX]);
      with T.Dependencies do
        begin
          AddUnit('resolve');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('sslsockets.pp',AllUnixOSes+AllWindowsOSes+AllAmigaLikeOSes+[OS2,EMX]);
      with T.Dependencies do
        begin
          AddUnit('ssockets');
          Addunit('sslbase');
        end;
    T.ResourceStrings := True;

    // HTTP Client
    T:=P.Targets.AddUnit('fpsock.pp',[linux,freebsd,netbsd,openbsd,dragonfly]);
      with T.Dependencies do
        begin
          AddUnit('resolve');
        end;
    T.ResourceStrings := True;

    T:=P.Targets.AddUnit('cnetdb.pp',[linux,freebsd,solaris,android]);

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('examples/ip6test.pp');
    P.Targets.AddExampleProgram('examples/svrclass.pp');
    P.Targets.AddExampleProgram('examples/testdns.pp');
    P.Targets.AddExampleProgram('examples/testnet.pp');
    P.Targets.AddExampleProgram('examples/testhosts.pp');
    P.Targets.AddExampleProgram('examples/testsvc.pp');
    P.Targets.AddExampleProgram('examples/testhst.pp');
    P.Targets.AddExampleProgram('examples/testuri.pp');
    P.Targets.AddExampleProgram('examples/testproto.pp');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
