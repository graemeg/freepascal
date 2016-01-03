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
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';

    P.Author := 'Andrew Haines';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Standalone CHM reader and writer library';
    P.NeedLibC:= false;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16];

    D:=P.Dependencies.Add('fcl-xml');
    D:=P.Dependencies.Add('fcl-base');
    D.Version:='3.1.1';

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='chm';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    D:=P.Dependencies.Add('fcl-xml');
      D.Version:='2.2.4';
    
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       begin
=======
        begin
>>>>>>> graemeg/fixes_2_2
=======
        begin
>>>>>>> origin/fixes_2_2
=======
       begin
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('chmobjinstconst.inc');
        end;
    T:=P.Targets.AddUnit('lzxcompressthread.pas');
      with T.Dependencies do
        begin
          AddUnit('paslzxcomp');
        end;
    T:=P.Targets.AddUnit('lzxcompressthread.pas');
      with T.Dependencies do
        begin
          AddUnit('paslzxcomp');
        end;
    T:=P.Targets.AddUnit('lzxcompressthread.pas');
      with T.Dependencies do
        begin
          AddUnit('paslzxcomp');
        end;
    T:=P.Targets.AddUnit('lzxcompressthread.pas');
      with T.Dependencies do
        begin
          AddUnit('paslzxcomp');
        end;
    T:=P.Targets.AddUnit('lzxcompressthread.pas');
      with T.Dependencies do
        begin
          AddUnit('paslzxcomp');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
    T:=P.Targets.AddUnit('lzxcompressthread');
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
<<<<<<< HEAD
<<<<<<< HEAD
   T:=P.Targets.AddUnit('itolitlstypes.pas');
   T:=P.Targets.AddUnit('itsftransform.pas');
      with T.Dependencies do
        begin
          AddUnit('itolitlstypes');
          AddUnit('paslzx');
        end;
   T:=P.Targets.AddUnit('itolitlsreader.pas');
      with T.Dependencies do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        begin  //chmreader, itolitlstypes, Sysutils, chmbase, itsftransform;
=======
        begin  //chmreader, itolitlstypes, Sysutils, chmbase, itsftransform; 
>>>>>>> graemeg/cpstrnew
=======
        begin  //chmreader, itolitlstypes, Sysutils, chmbase, itsftransform; 
>>>>>>> graemeg/cpstrnew
=======
        begin  //chmreader, itolitlstypes, Sysutils, chmbase, itsftransform; 
>>>>>>> graemeg/cpstrnew
=======
        begin  //chmreader, itolitlstypes, Sysutils, chmbase, itsftransform; 
>>>>>>> origin/cpstrnew
          AddUnit('chmbase');
          AddUnit('chmreader');
          AddUnit('itolitlstypes');
          AddUnit('itsftransform');

        end;

//    P.ProgramPath.Add('src');
    T:=P.Targets.AddProgram('chmls.lpr');
    T:=P.Targets.AddProgram('chmcmd.lpr');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
