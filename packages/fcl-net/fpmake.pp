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
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fcln';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('openssl',AllOSes - AllAmigaLikeOSes);
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('fcl-passrc');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Dependencies.Add('fcl-async',[linux,freebsd,netbsd,openbsd,dragonfly]);
    P.Dependencies.Add('rtl-extra');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    P.Author := 'Sebastian Guenther and Free Pascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Network related parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=P.OSes-[embedded,msdos,win16];
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-net';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('fcl-passrc');
    P.Dependencies.Add('fcl-async');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    P.SourcePath.Add('src');
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/win',AllWindowsOSes);
    P.IncludePath.Add('src/os2',[EMX]);
<<<<<<< HEAD
<<<<<<< HEAD
    P.IncludePath.Add('src/amiga',[morphos]);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    P.IncludePath.Add('src/$(OS)',AllOSes-AllWindowsOSes-AllUnixOSes-[EMX]);

    // IP and Sockets
    T:=P.Targets.AddUnit('netdb.pp',AllUnixOSes);
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('resolve.pp',AllUnixOSes+AllWindowsOSes+AllAmigaLikeOSes+[OS2,EMX]);
      with T.Dependencies do
        begin
          AddInclude('resolve.inc');
          AddUnit('netdb',AllUnixOSes);
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('ssockets.pp',AllUnixOSes+AllWindowsOSes+AllAmigaLikeOSes+[OS2,EMX]);
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('resolve.pp',AllUnixOSes+AllWindowsOSes+[OS2,EMX]);
      with T.Dependencies do
        begin
          AddInclude('resolve.inc');
          AddUnit('netdb');
        end;
    T:=P.Targets.AddUnit('ssockets.pp',AllUnixOSes+AllWindowsOSes+[OS2,EMX]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('resolve');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('sslsockets.pp',AllUnixOSes+AllWindowsOSes);
      with T.Dependencies do
        begin
          AddUnit('ssockets');
        end;
    T.ResourceStrings := True;

    // HTTP Client
    T:=P.Targets.AddUnit('fpsock.pp',[linux,freebsd,netbsd,openbsd,dragonfly]);
=======

    // HTTP Client
    T:=P.Targets.AddUnit('fpsock.pp',AllUnixOSes);
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2

    // HTTP Client
    T:=P.Targets.AddUnit('servlets.pp');
    T:=P.Targets.AddUnit('fpsock.pp',AllUnixOSes);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('resolve');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;

    T:=P.Targets.AddUnit('cnetdb.pp',[linux,freebsd,solaris]);

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('httpbase.pp',AllUnixOSes);
    T:=P.Targets.AddUnit('httpclient.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('httpbase');
          AddUnit('fpsock');
        end;
    T:=P.Targets.AddUnit('httpsvlt.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('fpsock');
          AddUnit('httpbase');
          AddUnit('servlets');
        end;

    // XML-RPC
    T:=P.Targets.AddUnit('xmlrpc.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('ssockets');
          AddUnit('httpclient');
          AddUnit('httpsvlt');
        end;
    T:=P.Targets.AddProgram('mkxmlrpc.pp',AllUnixOSes);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
