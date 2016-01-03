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

    P:=AddPackage('numlib');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx];
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='numlib';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//    P.Dependencies.Add('x11');

    T:=P.Targets.AddUnit('det.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('mdt');
        end;
    T:=P.Targets.AddUnit('dsl.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('eigh1.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('eigh2.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('eig.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('eigh1');
          AddUnit('eigh2');
        end;
    T:=P.Targets.AddUnit('int.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('inv.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('mdt');
          AddUnit('dsl');
        end;
    T:=P.Targets.AddUnit('iom.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('ipf.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('mdt');
          AddUnit('dsl');
          AddUnit('sle');
          AddUnit('spe');
        end;
    T:=P.Targets.AddUnit('mdt.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('dsl');
          AddUnit('omv');
        end;
    T:=P.Targets.AddUnit('numlib.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('ode.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('omv.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('roo.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('spe');
        end;
    T:=P.Targets.AddUnit('sle.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('omv');
          AddUnit('dsl');
          AddUnit('mdt');
        end;
    T:=P.Targets.AddUnit('spe.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;
    T:=P.Targets.AddUnit('spl.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
          AddUnit('sle');
        end;
    T:=P.Targets.AddUnit('typ.pas');
      with T.Dependencies do
        begin
          AddInclude('direct.inc');
        end;

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('iomwrvex.pas');
    P.Targets.AddExampleProgram('iomremex.pas');
    P.Targets.AddExampleProgram('iomrevex.pas');
    P.Targets.AddExampleProgram('invgenex.pas');
    P.Targets.AddExampleProgram('invgsyex.pas');
    P.Targets.AddExampleProgram('iomwrmex.pas');
    P.Targets.AddExampleProgram('invgpdex.pas');
    P.Targets.AddExampleProgram('iomrewrsex.pas');
    // 'invgsyex.dat
    // 'iomwrmex.dat
    // 'iomremex.dat
    // 'invgpdex.dat
    // 'invgenex.dat
    // 'iomrevex.dat
    P.ExamplePath.Add('tests');
    P.Targets.AddExampleProgram('eigts4te.pas');
    P.Targets.AddExampleProgram('detgpbte.pas');
    P.Targets.AddExampleProgram('eigsv3te.pas');
    P.Targets.AddExampleProgram('eiggg2te.pas');
    P.Targets.AddExampleProgram('eigge3te.pas');
    P.Targets.AddExampleProgram('sledtrte.pas');
    P.Targets.AddExampleProgram('spege1te.pas');
    P.Targets.AddExampleProgram('slegenlt.pas');
    P.Targets.AddExampleProgram('eigsv1te.pas');
    P.Targets.AddExampleProgram('eigts1te.pas');
    P.Targets.AddExampleProgram('turte.pas');
    P.Targets.AddExampleProgram('slegpdlt.pas');
    P.Targets.AddExampleProgram('invgsyte.pas');
    P.Targets.AddExampleProgram('timer.pas');
    P.Targets.AddExampleProgram('intge1te.pas');
    P.Targets.AddExampleProgram('eiggg4te.pas');
    P.Targets.AddExampleProgram('slegbalt.pas');
    P.Targets.AddExampleProgram('roof1rte.pas');
    P.Targets.AddExampleProgram('detgpdte.pas');
    P.Targets.AddExampleProgram('roofnrte.pas');
    P.Targets.AddExampleProgram('eigge1te.pas');
    P.Targets.AddExampleProgram('invgente.pas');
    P.Targets.AddExampleProgram('eiggs1te.pas');
    P.Targets.AddExampleProgram('intge2te.pas');
    P.Targets.AddExampleProgram('invgpdte.pas');
    P.Targets.AddExampleProgram('eigbs4te.pas');
    P.Targets.AddExampleProgram('slegpblt.pas');
    P.Targets.AddExampleProgram('eigbs2te.pas');
    P.Targets.AddExampleProgram('spesgnte.pas');
    P.Targets.AddExampleProgram('spemaxte.pas');
    P.Targets.AddExampleProgram('eigts3te.pas');
    P.Targets.AddExampleProgram('eigbs3te.pas');
    P.Targets.AddExampleProgram('sleglslt.pas');
    P.Targets.AddExampleProgram('eiggg3te.pas');
    P.Targets.AddExampleProgram('eiggg1te.pas');
    P.Targets.AddExampleProgram('slegpbte.pas');
    P.Targets.AddExampleProgram('eigbs1te.pas');
    P.Targets.AddExampleProgram('roopolte.pas');
    P.Targets.AddExampleProgram('eiggs4te.pas');
    P.Targets.AddExampleProgram('intge3te.pas');
    P.Targets.AddExampleProgram('slegbate.pas');
    P.Targets.AddExampleProgram('detgsyte.pas');
    P.Targets.AddExampleProgram('slegsyte.pas');
    P.Targets.AddExampleProgram('slegpdte.pas');
    P.Targets.AddExampleProgram('odeiv2te.pas');
    P.Targets.AddExampleProgram('sleglste.pas');
    P.Targets.AddExampleProgram('slegtrte.pas');
    P.Targets.AddExampleProgram('odeiv1te.pas');
    P.Targets.AddExampleProgram('speentte.pas');
    P.Targets.AddExampleProgram('spepowte.pas');
    P.Targets.AddExampleProgram('slegente.pas');
    P.Targets.AddExampleProgram('detgtrte.pas');
    P.Targets.AddExampleProgram('roofnrt1.pas');
    P.Targets.AddExampleProgram('eiggs2te.pas');
    P.Targets.AddExampleProgram('eiggs3te.pas');
    P.Targets.AddExampleProgram('slegsylt.pas');
    P.Targets.AddExampleProgram('test.pas');
    P.Targets.AddExampleProgram('spepolte.pas');
    P.Targets.AddExampleProgram('iomwrmte.pas');
    P.Targets.AddExampleProgram('eigts2te.pas');
    // 'spesgnte.dat
    // 'spebi1te.dat
    // 'slegpbte.dat
    // 'eigts2te.dat
    // 'speathte.dat
    // 'spebj0te.dat
    // 'speentte.dat
    // 'eigge3te.dat
    // 'spebk1te.dat
    // 'sleglslt.dat
    // 'spearcte.dat
    // 'eiggg1te.dat
    // 'speby1te.dat
    // 'spesihte.dat
    // 'spemaxte.dat
    // 'eigts1te.dat
    // 'eiggs1te.dat
    // 'detgtrte.dat
    // 'roopolte.dat
    // 'speerfte.dat
    // 'slegpdte.dat
    // 'invgpdte.dat
    // 'eigge1te.dat
    // 'speachte.dat
    // 'spebj1te.dat
    // 'spepolte.dat
    // 'spebk0te.dat
    // 'odeiv2te.dat
    // 'eiggs3te.dat
    // 'slegsylt.dat
    // 'eigbs4te.dat
    // 'invgsyte.dat
    // 'eigsv1te.dat
    // 'detgpdte.dat
    // 'detgpbte.dat
    // 'slegsyte.dat
    // 'spebi0te.dat
    // 'eigbs3te.dat
    // 'eigbs1te.dat
    // 'roof1rte.dat
    // 'slegenlt.dat
    // 'eigbs2te.dat
    // 'spetahte.dat
    // 'eiggs2te.dat
    // 'specohte.dat
    // 'test.bat
    // 'eiggg3te.dat
    // 'roofnrte.dat
    // 'detgsyte.dat
    // 'eiggg4te.dat
    // 'slegente.dat
    // 'speefcte.dat
    // 'slegpdlt.dat
    // 'invgente.dat
    // 'slegtrte.dat
    // 'spearste.dat
    // 'speby0te.dat
    // 'eigts4te.dat
    // 'eiggg2te.dat
    // 'sledtrte.dat
    // 'slegbate.dat
    // 'sleglste.dat
    // 'odeiv1te.dat
    // 'slegbalt.dat
    // 'eigts3te.dat
    // 'eiggs4te.dat
    // 'slegpblt.dat
    // 'spegamte.dat
    // 'eigsv3te.dat
    // 'speashte.dat
    // 'spepowte.dat

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
