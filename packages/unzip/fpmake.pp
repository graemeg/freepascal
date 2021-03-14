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

    P:=AddPackage('unzip');
    P.ShortName:='zip';
    P.Description := 'Support for decompression of ZIP archives - either using a Pascal port of the library from InfoZIP, or interface to using the dynamically linked version of this library.';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.SourcePath.Add('src');
    P.OSes := P.OSes - [embedded,nativent,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    T:=P.Targets.AddUnit('unzip51g.pp');
      with T.Dependencies do
        begin
          AddUnit('ziptypes');
        end;
    T:=P.Targets.AddUnit('ziptypes.pp');
    T:=P.Targets.AddUnit('unzipdll.pp',[emx,os2]);
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
