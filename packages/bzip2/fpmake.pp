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

    P:=AddPackage('bzip2');
    P.ShortName:='bz2';

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}

    P.Version:='3.3.1';

    P.Author := 'Library: Julian R. Seward, header: Daniel Mantione';
    // 3 clause becaue "prominent notice" is not required.
    P.License := 'Library: 3 clause BSD, header: 3 clause BSD ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'BZip2 decompression unit.';
    P.NeedLibC:= true;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.Dependencies.Add('rtl-extra');

    T:=P.Targets.AddUnit('bzip2comn.pp');
    T:=P.Targets.AddUnit('bzip2.pas');
      with T.Dependencies do
        begin
          AddInclude('bzip2i386.inc',[i386],AllOSes);
          AddUnit('bzip2comn');
        end;
    T:=P.Targets.AddUnit('bzip2stream.pp');
      with T.Dependencies do
        begin
          AddInclude('bzip2si386.inc',[i386],AllOSes);
          AddUnit('bzip2comn');
        end;
    T.ResourceStrings := true;

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('pasbzip.pas');


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
