{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit, sysutils;
{$endif ALLPACKAGES}

procedure add_fppkg(const ADirectory: string);

const
  TargetsWithWGet = [linux,beos,haiku,freebsd,netbsd,openbsd,darwin,iphonesim,ios,solaris,win32,win64,wince,aix,dragonfly];
  TargetsWithfpWeb = TargetsWithWGet;

Var
  T : TTarget;
  P : TPackage;
  Data2Inc : string;
begin
  AddCustomFpmakeCommandlineOption('data2inc', 'Use indicated data2inc executable.');
  AddCustomFpmakeCommandlineOption('genfpmkunit', 'Regenerate the fpmkunitsrc.inc file (fppkg).');

  With Installer do
    begin

    P:=AddPackage('fppkg');
    P.ShortName:='fppk';
    P.Directory:=ADirectory;

    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('fcl-process',AllOSes-[go32v2,os2]);
    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fpmkunit');

    P.Dependencies.Add('univint',[MacOSX,iphonesim,ios]);
    P.Dependencies.Add('fcl-net', TargetsWithfpWeb);
    P.Dependencies.Add('fcl-web', TargetsWithfpWeb);
    P.Dependencies.Add('httpd22', TargetsWithfpWeb);

    P.Author := 'FreePascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Libraries to create fppkg package managers.';
    P.NeedLibC:= false;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16,atari,macosclassic,palmos,symbian,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('fprepos.pp');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('fpxmlrep.pp');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('pkgrepos.pp');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('pkgmessages.pp');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('pkgoptions.pp');
    T:=P.Targets.AddUnit('pkgfppkg.pp');
    T:=P.Targets.AddUnit('pkgglobals.pp');
    T:=P.Targets.AddUnit('pkghandler.pp');
    T:=P.Targets.AddUnit('pkgmkconv.pp');
    T:=P.Targets.AddUnit('pkgdownload.pp');
    T:=P.Targets.AddUnit('pkgfpmake.pp');
    T.Dependencies.AddInclude('fpmkunitsrc.inc');
    T:=P.Targets.AddUnit('pkgcommands.pp');
    T:=P.Targets.AddUnit('pkgpackagesstructure.pp');
    T:=P.Targets.AddUnit('pkguninstalledsrcsrepo.pp');

    T:=P.Targets.AddUnit('pkgwget.pp', TargetsWithWGet);
    T:=P.Targets.AddUnit('pkgfphttp.pp', TargetsWithfpWeb);

    // Do not re-generate fpmkunitsrc.inc by default so it is possible to control
    // when we want to update the internal fpmkunitsrc
    if GetCustomFpmakeCommandlineOptionValue('genfpmkunit') <> '' then
      begin
      Data2Inc := GetCustomFpmakeCommandlineOptionValue('data2inc');
      if Data2Inc<>'' then
        Data2Inc:= ExpandFileName(Data2Inc);
      if Data2Inc='' then
        begin
        data2inc := ExeSearch(AddProgramExtension('data2inc', Defaults.BuildOS));
        end;
      if Data2Inc <> '' then
        P.Commands.AddCommand(Data2Inc,'-b -s $(SOURCE) $(DEST) fpmkunitsrc','src/fpmkunitsrc.inc','../fpmkunit/src/fpmkunit.pp');
      end;
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fppkg('');
  Installer.Run;
end.
{$endif ALLPACKAGES}
