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

    P:=AddPackage('ptc');
{$ifdef ALLPACKAGES}
    P.Directory:='ptc';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

T:=P.Targets.AddUnit('ptc.pp');
  with T.Dependencies do
    begin
      AddInclude('extensions.inc');
      AddInclude('coreinterface.inc');
      AddInclude('aread.inc');
      AddInclude('colord.inc');
      AddInclude('formatd.inc');
      AddInclude('eventd.inc');
      AddInclude('keyeventd.inc');
      AddInclude('mouseeventd.inc');
      AddInclude('moded.inc');
      AddInclude('paletted.inc');
      AddInclude('cleard.inc');
      AddInclude('copyd.inc');
      AddInclude('clipperd.inc');
      AddInclude('basesurfaced.inc');
      AddInclude('surfaced.inc');
      AddInclude('baseconsoled.inc');
      AddInclude('consoled.inc');
      AddInclude('errord.inc');
      AddInclude('timerd.inc');
      AddInclude('log.inc');
      AddInclude('coreimplementation.inc');
      AddInclude('errori.inc');
      AddInclude('areai.inc');
      AddInclude('colori.inc');
      AddInclude('formati.inc');
      AddInclude('eventi.inc');
      AddInclude('keyeventi.inc');
      AddInclude('mouseeventi.inc');
      AddInclude('modei.inc');
      AddInclude('palettei.inc');
      AddInclude('cleari.inc');
      AddInclude('copyi.inc');
      AddInclude('clipperi.inc');
      AddInclude('basesurfacei.inc');
      AddInclude('baseconsolei.inc');
      AddInclude('surfacei.inc');
      AddInclude('timeri.inc');
      AddInclude('includes.inc');
      AddInclude('x11modesd.inc');
      AddInclude('x11imaged.inc');
      AddInclude('x11displayd.inc');
      AddInclude('x11windowdisplayd.inc');
      AddInclude('x11dga1displayd.inc');
      AddInclude('x11dga2displayd.inc');
      AddInclude('x11consoled.inc');
      AddInclude('check.inc');
      AddInclude('x11modesi.inc');
      AddInclude('x11imagei.inc');
      AddInclude('x11displayi.inc');
      AddInclude('xunikey.inc');
      AddInclude('x11windowdisplayi.inc');
      AddInclude('x11dga1displayi.inc');
      AddInclude('x11dga2displayi.inc');
      AddInclude('x11consolei.inc');
      AddInclude('consolei.inc');
      AddUnit('hermes');
      AddUnit('xlib');
      AddUnit('xatom');
      AddUnit('keysym');
      AddUnit('xrandr');
      AddUnit('xf86vmode');
      AddUnit('xf86dga');
      AddUnit('xshm');
    end;


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
