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

    P:=AddPackage('hermes');
{$ifdef ALLPACKAGES}
    P.Directory:='hermes';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

T:=P.Targets.AddUnit('hermes.pp');
  with T.Dependencies do
    begin
      AddInclude('hermdef.inc');
      AddInclude('hermconf.inc');
      AddInclude('malloc.inc');
      AddInclude('debug.inc');
      AddInclude('dither.inc');
      AddInclude('headp.inc');
      AddInclude('p_16.inc');
      AddInclude('p_24.inc');
      AddInclude('p_32.inc');
      AddInclude('p_clr.inc');
      AddInclude('p_cnv.inc');
      AddInclude('p_cpy.inc');
      AddInclude('p_g.inc');
      AddInclude('p_ga.inc');
      AddInclude('p_gac.inc');
      AddInclude('p_gca.inc');
      AddInclude('p_gcc.inc');
      AddInclude('p_i8.inc');
      AddInclude('p_muhmu.inc');
      AddInclude('d_32.inc');
      AddInclude('headi386.inc');
      AddInclude('headmmx.inc');
      AddInclude('factconv.inc');
      AddInclude('list.inc');
      AddInclude('utility.inc');
      AddInclude('format.inc');
      AddInclude('palette.inc');
      AddInclude('convert.inc');
      AddInclude('clear.inc');
      AddInclude('factory.inc');
    end;


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
