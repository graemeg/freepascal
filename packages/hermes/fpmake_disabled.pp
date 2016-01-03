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
    P.Version:='2.2.2-0';

    P.Author := 'Library: Peter Mattis, Spencer Kimball and Josh MacDonald, header: Nikolay Nikolov';
    P.License := 'Library: GPL2, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Library for pixel graphics conversion';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.IncludePath.Add('src/i386',[i386],AllOSes);

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
      AddInclude('headi386.inc',[i386],AllOSes);
      AddInclude('headmmx.inc',[i386],AllOSes); 
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

// mmx_clr.as
// mmx_main.as
// mmxp2_32.as
// mmxp_32.as
// x8616lut.as
// x86_clr.as
// x86_main.as
// x86p_16.as
// x86p_32.as
// x86p_cpy.as
// x86p_i8.as
// x86p_s32.as
// x86pscpy.as');
