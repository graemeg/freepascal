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

    P:=AddPackage('httpd13');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='hd13';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Library: Apache Foundation, header: Felipe Monteiro de Carvalho';
    P.License := 'Library: Apache License 2, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Headers for the Apache 1.3 www server';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?
{$ifdef ALLPACKAGES}
    P.OSes := []; // By default, do not build this package
{$else ALLPACKAGES}
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx];
{$endif ALLPACKAGES}
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='httpd13';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//    P.Dependencies.Add('x11');

    T:=P.Targets.AddUnit('httpd.pas');
      with T.Dependencies do
        begin
          AddInclude('httpd.inc');
          AddInclude('ap_config.inc');
          AddInclude('ap_mmn.inc');
          AddInclude('hsregex.inc');
          AddInclude('ap_alloc.inc');
          AddInclude('readdir.inc');
          AddInclude('buff.inc');
          AddInclude('ap.inc');
          AddInclude('util_uri.inc');
          AddInclude('http_config.inc');
          AddInclude('http_core.inc');
          AddInclude('http_log.inc');
          AddInclude('http_main.inc');
          AddInclude('http_protocol.inc');
          AddInclude('http_request.inc');
          AddInclude('http_vhost.inc');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
