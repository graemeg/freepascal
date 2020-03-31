{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_tply(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;
  lexyaccpath: string;

begin
  With Installer do
    begin
    P:=AddPackage('utils-lexyacc');
    P.ShortName:='tply';
    { java and jvm-android do not support 
      fpc_get_output used in these sources }
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];
    { palmos does not support command line parameters }
    P.OSes := P.OSes - [palmos];
    { Program does not fit in 16-bit memory constraints }
    P.OSes := P.OSes - [msdos,win16];
    { avr-embedded and i8086-embedded do not meet needed requirements }
    if Defaults.CPU in [avr,i8086] then
      P.OSes := P.OSes - [embedded];

    P.Author := '<various>';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A compiler generator for Turbo Pascal and compatibles.';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';

    P.Options.Add('-Sg');

    T:=P.Targets.AddProgram('plex.pas');
    T.Dependencies.AddUnit('lexbase');
    T.Dependencies.AddUnit('lexopt');
    T.Dependencies.AddUnit('lexdfa');
    T.Dependencies.AddUnit('lexpos');
    T.Dependencies.AddUnit('lexlist');
    T.Dependencies.AddUnit('lexrules');
    T.Dependencies.AddUnit('lexmsgs');
    T.Dependencies.AddUnit('lextable');


    T:=P.Targets.AddProgram('pyacc.pas');
    T.Dependencies.AddUnit('yaccbase');
    T.Dependencies.AddUnit('yaccmsgs');
    T.Dependencies.AddUnit('yaccclos');
    T.Dependencies.AddUnit('yaccpars');
    T.Dependencies.AddUnit('yacclook');
    T.Dependencies.AddUnit('yaccsem');
    T.Dependencies.AddUnit('yacclr0');
    T.Dependencies.AddUnit('yacctabl');


    P.Targets.AddUnit('lexlib.pas');
    P.Targets.AddUnit('yacclib.pas');

    P.Targets.AddUnit('lexbase.pas').install:=false;
    P.Targets.AddUnit('lexopt.pas').install:=false;
    P.Targets.AddUnit('lexdfa.pas').install:=false;
    P.Targets.AddUnit('lexpos.pas').install:=false;
    P.Targets.AddUnit('lexlist.pas').install:=false;
    P.Targets.AddUnit('lexrules.pas').install:=false;
    P.Targets.AddUnit('lexmsgs.pas').install:=false;
    P.Targets.AddUnit('lextable.pas').install:=false;
    P.Targets.AddUnit('yaccbase.pas').install:=false;
    P.Targets.AddUnit('yaccmsgs.pas').install:=false;
    P.Targets.AddUnit('yaccclos.pas').install:=false;
    P.Targets.AddUnit('yaccpars.pas').install:=false;
    P.Targets.AddUnit('yacclook.pas').install:=false;
    P.Targets.AddUnit('yaccsem.pas').install:=false;
    P.Targets.AddUnit('yacclr0.pas').install:=false;
    P.Targets.AddUnit('yacctabl.pas').install:=false;

     if (OSToString(defaults.OS)=lowercase({$I %FPCTARGETOS%})) and
       (CPUToString(defaults.CPU)=lowercase({$I %FPCTARGETCPU%})) then
      begin
      // Do not install these files when performing a cross-installation
      if Defaults.OS in AllUnixOSes then
        lexyaccpath:='$(prefix)lib/fpc/lexyacc'
      else
        lexyaccpath:='$(bininstalldir)';
      P.InstallFiles.Add('yylex.cod',lexyaccpath);
      P.InstallFiles.Add('yyparse.cod',lexyaccpath);
      end;
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_tply('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




