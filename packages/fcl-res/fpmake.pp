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

    P:=AddPackage('fcl-res');
    P.ShortName:='fcle';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('rtl-objpas');
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('tplylib');
    P.Author := 'Giulio Bernardi';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Resource handling of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('acceleratorsresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('bitmapresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resfactory');
          AddUnit('resdatastream');
        end;
    T:=P.Targets.AddUnit('coffconsts.pp');
    T:=P.Targets.AddUnit('cofftypes.pp');
    T:=P.Targets.AddUnit('coffreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resourcetree');
          AddUnit('cofftypes');
          AddUnit('coffconsts');
          AddUnit('resdatastream');
        end;
    T:=P.Targets.AddUnit('coffwriter.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resourcetree');
          AddUnit('cofftypes');
          AddUnit('coffconsts');
        end;
    T:=P.Targets.AddUnit('xcoffwriter.pp');
      with T.Dependencies do
        begin
          AddUnit('cofftypes');
          AddUnit('coffwriter');
          AddUnit('coffconsts');
          AddUnit('fpcrestypes');
        end;
    T:=P.Targets.AddUnit('dfmreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resdatastream');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('tlbreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resdatastream');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('elfconsts.pp');
    T:=P.Targets.AddUnit('elftypes.pp');
    T:=P.Targets.AddUnit('elfreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('elfconsts');
          AddUnit('elftypes');
          AddUnit('resdatastream');
          AddUnit('resfactory');
          AddUnit('resourcetree');
          AddUnit('strtable');
          AddUnit('fpcrestypes');
          AddInclude('elfsubreader.inc');
          AddInclude('elfdefaulttarget.inc');
        end;
    T:=P.Targets.AddUnit('elfwriter.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('elfconsts');
          AddUnit('elftypes');
          AddUnit('resourcetree');
          AddUnit('strtable');
          AddUnit('fpcrestypes');
          AddInclude('elfsubwriter.inc');
          AddInclude('elfdefaulttarget.inc');
        end;
    T:=P.Targets.AddUnit('externaltypes.pp');
    T:=P.Targets.AddUnit('externalreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resourcetree');
          AddUnit('externaltypes');
          AddUnit('resdatastream');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('externalwriter.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resourcetree');
          AddUnit('externaltypes');
          AddUnit('strtable');
        end;
    T:=P.Targets.AddUnit('fpcrestypes.pp');
    T:=P.Targets.AddUnit('groupcursorresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('groupresource');
          AddUnit('resfactory');
          AddUnit('resdatastream');
          AddUnit('icocurtypes');
        end;
    T:=P.Targets.AddUnit('groupiconresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('groupresource');
          AddUnit('resfactory');
          AddUnit('resdatastream');
          AddUnit('icocurtypes');
        end;
    T:=P.Targets.AddUnit('groupresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resdatastream');
          AddUnit('icocurtypes');
        end;
    T:=P.Targets.AddUnit('icocurtypes.pp');
    T:=P.Targets.AddUnit('machotypes.pp');
    T:=P.Targets.AddUnit('machoconsts.pp');
      with T.Dependencies do
        begin
          AddUnit('machotypes');
        end;
    T:=P.Targets.AddUnit('machoreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('machotypes');
          AddUnit('machoconsts');
          AddUnit('resfactory');
          AddUnit('resourcetree');
          AddUnit('resdatastream');
          AddUnit('fpcrestypes');
          AddInclude('machosubreader.inc');
          AddInclude('machodefaulttarget.inc');
        end;
    T:=P.Targets.AddUnit('machowriter.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('machotypes');
          AddUnit('resourcetree');
          AddUnit('machoconsts');
          AddUnit('strtable');
          AddUnit('fpcrestypes');
          AddInclude('machosubwriter.inc');
          AddInclude('machodefaulttarget.inc');
        end;
    T:=P.Targets.AddUnit('rcparser.pas');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('acceleratorsresource');
          AddUnit('groupiconresource');
          AddUnit('stringtableresource');
          AddUnit('bitmapresource');
          AddUnit('versionresource');
          AddUnit('versiontypes');
          AddUnit('groupcursorresource');
          AddInclude('rcparserfn.inc');
          AddInclude('rclex.inc');
          AddInclude('yyinclude.pp');
          AddInclude('yypreproc.pp');
        end;
    T:=P.Targets.AddUnit('rcreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resdatastream');
          AddUnit('resfactory');
          AddUnit('rcparser');
        end;
    T:=P.Targets.AddUnit('resdatastream.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
        end;
    T:=P.Targets.AddUnit('resfactory.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('resmerger.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('stringtableresource');
          AddUnit('groupiconresource');
          AddUnit('groupcursorresource');
        end;
    T:=P.Targets.AddUnit('resource.pp');
      with T.Dependencies do
        begin
          AddUnit('resdatastream');
          AddUnit('resourcetree');
          AddUnit('resmerger');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('resourcetree.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('resreader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resdatastream');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('reswriter.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
        end;
    T:=P.Targets.AddUnit('stringtableresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('resfactory');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('strtable.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
        end;
    T:=P.Targets.AddUnit('versionconsts.pp');
    T:=P.Targets.AddUnit('versionresource.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('versiontypes');
          AddUnit('resfactory');
        end;
    T:=P.Targets.AddUnit('versiontypes.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('versionconsts');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('winpeimagereader.pp');
      with T.Dependencies do
        begin
          AddUnit('resource');
          AddUnit('coffreader');
        end;
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
