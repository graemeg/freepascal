{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

// Note this package is currently not compiled and a mess.

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
    P.Version:='2.2.2-0';
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.IncludePath.Add('src/dos',[go32v2]);
    P.IncludePath.Add('src/dos/base',[go32v2]);
    P.SourcePath.Add('src/dos/cga',[go32v2]);
    P.SourcePath.Add('src/dos/fakemode',[go32v2]);
    P.SourcePath.Add('src/dos/textfx2',[go32v2]);
    P.SourcePath.Add('src/dos/timeunit',[go32v2]);
    P.SourcePath.Add('src/dos/vesa',[go32v2]);
    P.IncludePath.Add('src/dos/cga',[go32v2]);
    P.IncludePath.Add('src/dos/fakemode',[go32v2]);
    P.IncludePath.Add('src/dos/textfx2',[go32v2]);
    P.IncludePath.Add('src/dos/timeunit',[go32v2]);
    P.IncludePath.Add('src/dos/vesa',[go32v2]);
    P.IncludePath.Add('src/win32',[win32,win64]);
    P.IncludePath.Add('src/wince',[wince]);
    P.IncludePath.Add('src/x11',AllUnixOSes);

  P.Dependencies.Add('hermes');
  P.Dependencies.Add('x');

  T:=P.Targets.AddUnit('ptc.pp');
  with T.Dependencies do
    begin
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
      AddInclude('includes.inc',allunixoses+[WinCE]);
      AddInclude('extensions.inc',allunixoses);
      AddInclude('x11modesd.inc',allunixoses);
      AddInclude('x11imaged.inc',allunixoses);
      AddInclude('x11displayd.inc',allunixoses);
      AddInclude('x11windowdisplayd.inc',allunixoses);
      AddInclude('x11dga1displayd.inc',allunixoses);
      AddInclude('x11dga2displayd.inc',allunixoses);
      AddInclude('x11consoled.inc',allunixoses);
      AddInclude('check.inc',allunixoses);
      AddInclude('x11modesi.inc',allunixoses);
      AddInclude('x11imagei.inc',allunixoses);
      AddInclude('x11displayi.inc',allunixoses);
      AddInclude('xunikey.inc',allunixoses);
      AddInclude('x11windowdisplayi.inc',allunixoses);
      AddInclude('x11dga1displayi.inc',allunixoses);
      AddInclude('x11dga2displayi.inc',allunixoses);
      AddInclude('x11consolei.inc',allunixoses);
      AddInclude('consolei.inc');
      AddUnit('directdr',[Win32,win64]);
      AddUnit('p_gx',[Wince]);
      AddUnit('textfx2',[Go32v2]);
      AddUnit('cga',[Go32v2]);
      AddUnit('timeunit',[Go32v2]);
      AddUnit('vesa',[Go32v2]);
      AddUnit('vga',[Go32v2]);
     end;

    P.ExamplePath.Add('examples/');
    P.Targets.AddExampleProgram('random.pp');
    P.Targets.AddExampleProgram('palette.pp');
    P.Targets.AddExampleProgram('pixel.pp');
    P.Targets.AddExampleProgram('console.pp');
    P.Targets.AddExampleProgram('image.pp');
    P.Targets.AddExampleProgram('flower.pp');
    P.Targets.AddExampleProgram('save.pp');
    P.Targets.AddExampleProgram('hicolor.pp');
    P.Targets.AddExampleProgram('stretch.pp');
    P.Targets.AddExampleProgram('tunnel.pp');
    P.Targets.AddExampleProgram('modes.pp');
    P.Targets.AddExampleProgram('buffer.pp');
    P.Targets.AddExampleProgram('texwarp.pp');
    P.Targets.AddExampleProgram('keyboard.pp');
    P.Targets.AddExampleProgram('timer.pp');
    P.Targets.AddExampleProgram('clip.pp');
    P.Targets.AddExampleProgram('lights.pp');
    P.Targets.AddExampleProgram('fire.pp');
    P.Targets.AddExampleProgram('mojo.pp');
    P.Targets.AddExampleProgram('land.pp');
    P.Targets.AddExampleProgram('keybrd2.pp');
    P.Targets.AddExampleProgram('clear.pp');
    P.Targets.AddExampleProgram('con_info.pp');
    P.Targets.AddExampleProgram('area.pp');
    P.Targets.AddExampleProgram('tunnel3d.pp');
    // 'Makefile
    // 'stretch.tga
    // 'Makefile.fpc
    // 'mojo.raw
    // 'tunnel3d.raw
    // 'image.tga


{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
