{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  P : TPackage;
  T : TTarget;
  D : TDependency;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('chm');
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';

    P.Author := 'Andrew Haines';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Standalone CHM reader and writer library';
    P.NeedLibC:= false;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16,macos,palmos,atari];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    D:=P.Dependencies.Add('fcl-xml');
    D:=P.Dependencies.Add('fcl-base');
    D:=P.Dependencies.Add('rtl-generics');

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('chmbase.pas');
    T:=P.Targets.AddUnit('chmfilewriter.pas');
      with T.Dependencies do
        begin
          AddUnit('chmwriter');
        end;
    T:=P.Targets.AddUnit('chmreader.pas');
      with T.Dependencies do
        begin
          AddUnit('chmbase');
          AddUnit('paslzx');
        end;
    T:=P.Targets.AddUnit('chmsitemap.pas');
      with T.Dependencies do
        begin
          AddUnit('fasthtmlparser');
          AddUnit('htmlutil');
        end;
    T:=P.Targets.AddUnit('chmspecialfiles.pas');
      with T.Dependencies do
        begin
          AddUnit('chmtypes');
        end;
    T:=P.Targets.AddUnit('chmtypes.pas');
      with T.Dependencies do
       begin
          AddUnit('chmbase');
        end;
    T:=P.Targets.AddUnit('htmlindexer.pas');
      with T.Dependencies do
        begin
          AddUnit('fasthtmlparser');
          AddUnit('htmlutil');
        end;
    T:=P.Targets.AddUnit('chmwriter.pas');
      with T.Dependencies do
        begin
          AddUnit('chmbase');
          AddUnit('chmtypes');
          AddUnit('chmspecialfiles');
          AddUnit('paslzxcomp');
          AddUnit('chmfiftimain');
          AddInclude('chmobjinstconst.inc');
        end;
    T:=P.Targets.AddUnit('lzxcompressthread.pas');
      with T.Dependencies do
        begin
          AddUnit('paslzxcomp');
        end;
    T:=P.Targets.AddUnit('fasthtmlparser.pas');
    T:=P.Targets.AddUnit('htmlutil.pas');
    T:=P.Targets.AddUnit('paslznonslide.pas');
    T:=P.Targets.AddUnit('paslzx.pas');
    T:=P.Targets.AddUnit('paslzxcomp.pas');
      with T.Dependencies do
        begin
          AddUnit('paslznonslide');
        end;
    T:=P.Targets.AddUnit('chmfiftimain.pas');
      with T.Dependencies do
        begin
          AddUnit('htmlindexer');
        end;
   T:=P.Targets.AddUnit('itolitlstypes.pas');
   T:=P.Targets.AddUnit('itsftransform.pas');
      with T.Dependencies do
        begin
          AddUnit('itolitlstypes');
          AddUnit('paslzx');
        end;
   T:=P.Targets.AddUnit('itolitlsreader.pas');
      with T.Dependencies do
        begin  //chmreader, itolitlstypes, Sysutils, chmbase, itsftransform;
          AddUnit('chmbase');
          AddUnit('chmreader');
          AddUnit('itolitlstypes');
          AddUnit('itsftransform');

        end;

//    P.ProgramPath.Add('src');
    T:=P.Targets.AddProgram('chmls.lpr');
    T:=P.Targets.AddProgram('chmcmd.lpr');


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
