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

    P:=AddPackage('fcl-stl');
    P.ShortName:='fcst';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}

    P.Version:='3.3.1';
    P.Author := 'Vlado Boza';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Generic container library of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Options.Add('-S2h');
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('garrayutils.pp');
    T:=P.Targets.AddUnit('gdeque.pp');
    T:=P.Targets.AddUnit('gmap.pp');
      with T.Dependencies do
        begin
          AddUnit('gset');
        end;
    T:=P.Targets.AddUnit('gpriorityqueue.pp');
      with T.Dependencies do
        begin
          AddUnit('gvector');
        end;
    T:=P.Targets.AddUnit('gqueue.pp');
      with T.Dependencies do
        begin
          AddUnit('gdeque');
        end;
    T:=P.Targets.AddUnit('gset.pp');
    T:=P.Targets.AddUnit('glinkedlist.pp');
    T:=P.Targets.AddUnit('gtree.pp');
    T:=P.Targets.AddUnit('gstack.pp');
      with T.Dependencies do
        begin
          AddUnit('gvector');
        end;
    T:=P.Targets.AddUnit('gutil.pp');
    T:=P.Targets.AddUnit('gvector.pp');
    T:=P.Targets.AddUnit('ghashset.pp');
      with T.Dependencies do
        begin
          AddUnit('gvector');
          AddUnit('gutil');
          AddUnit('garrayutils');
        end;
    T:=P.Targets.AddUnit('ghashmap.pp');
      with T.Dependencies do
        begin
          AddUnit('gvector');
          AddUnit('gutil');
          AddUnit('garrayutils');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

