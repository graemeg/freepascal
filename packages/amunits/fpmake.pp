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

    P:=AddPackage('amunits');
    P.ShortName := 'amu';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';

    P.Author := 'Nils Sjoholm';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A set of units for classic AmigaOS. (68k only)';
    P.NeedLibC:= false;

    P.CPUs:=[m68k];
    P.OSes:=[Amiga];

    P.SourcePath.Add('src/coreunits');
    P.SourcePath.Add('src/otherlibs');
    P.SourcePath.Add('src/utilunits');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('amigautils.pas');
    T:=P.Targets.AddUnit('wbargs.pas');
    T:=P.Targets.AddUnit('vartags.pas');
    T:=P.Targets.AddUnit('pastoc.pas');
    T:=P.Targets.AddUnit('tagsarray.pas');
    T:=P.Targets.AddUnit('deadkeys.pas');
    T:=P.Targets.AddUnit('consoleio.pas');
    T:=P.Targets.AddUnit('linklist.pas');
    T:=P.Targets.AddUnit('hisoft.pas');
    T:=P.Targets.AddUnit('timerutils.pas');
    T:=P.Targets.AddUnit('easyasl.pas');
    T:=P.Targets.AddUnit('doublebuffer.pas');
    T:=P.Targets.AddUnit('exec.pas');
    T:=P.Targets.AddUnit('intuition.pas');
    T:=P.Targets.AddUnit('agraphics.pas');
    T:=P.Targets.AddUnit('amigalib.pas');
    T:=P.Targets.AddUnit('nonvolatile.pas');
    T:=P.Targets.AddUnit('iffparse.pas');
    T:=P.Targets.AddUnit('hardware.pas');
    T:=P.Targets.AddUnit('expansion.pas');
    T:=P.Targets.AddUnit('diskfont.pas');
    T:=P.Targets.AddUnit('conunit.pas');
    T:=P.Targets.AddUnit('amigados.pas');
    T:=P.Targets.AddUnit('configvars.pas');
    T:=P.Targets.AddUnit('akeyboard.pas');
    T:=P.Targets.AddUnit('bootblock.pas');
    T:=P.Targets.AddUnit('icon.pas');
    T:=P.Targets.AddUnit('cd.pas');
    T:=P.Targets.AddUnit('realtime.pas');
    T:=P.Targets.AddUnit('rexx.pas');
    T:=P.Targets.AddUnit('translator.pas');
    T:=P.Targets.AddUnit('scsidisk.pas');
    T:=P.Targets.AddUnit('lowlevel.pas');
    T:=P.Targets.AddUnit('configregs.pas');
    T:=P.Targets.AddUnit('prefs.pas');
    T:=P.Targets.AddUnit('parallel.pas');
    T:=P.Targets.AddUnit('gadtools.pas');
    T:=P.Targets.AddUnit('prtgfx.pas');
    T:=P.Targets.AddUnit('romboot_base.pas');
    T:=P.Targets.AddUnit('trackdisk.pas');
    T:=P.Targets.AddUnit('expansionbase.pas');
    T:=P.Targets.AddUnit('amigaguide.pas');
    T:=P.Targets.AddUnit('amigaprinter.pas');
    T:=P.Targets.AddUnit('bullet.pas');
    T:=P.Targets.AddUnit('clipboard.pas');
    T:=P.Targets.AddUnit('keymap.pas');
    T:=P.Targets.AddUnit('utility.pas');
    T:=P.Targets.AddUnit('timer.pas');
    T:=P.Targets.AddUnit('workbench.pas');
    T:=P.Targets.AddUnit('colorwheel.pas');
    T:=P.Targets.AddUnit('prtbase.pas');
    T:=P.Targets.AddUnit('console.pas');
    T:=P.Targets.AddUnit('locale.pas');
    T:=P.Targets.AddUnit('tapedeck.pas');
    T:=P.Targets.AddUnit('serial.pas');
    T:=P.Targets.AddUnit('gameport.pas');
    T:=P.Targets.AddUnit('gradientslider.pas');
    T:=P.Targets.AddUnit('input.pas');
    T:=P.Targets.AddUnit('audio.pas');
    T:=P.Targets.AddUnit('datatypes.pas');
    T:=P.Targets.AddUnit('asl.pas');
    T:=P.Targets.AddUnit('commodities.pas');
    T:=P.Targets.AddUnit('hardblocks.pas');
    T:=P.Targets.AddUnit('layers.pas');
    T:=P.Targets.AddUnit('inputevent.pas');
    T:=P.Targets.AddUnit('identify.pas');
    T:=P.Targets.AddUnit('gtlayout.pas');
    T:=P.Targets.AddUnit('tritonmacros.pas');
    T:=P.Targets.AddUnit('render.pas');
    T:=P.Targets.AddUnit('mui.pas');
    T:=P.Targets.AddUnit('picasso96api.pas');
    T:=P.Targets.AddUnit('guigfx.pas');
    T:=P.Targets.AddUnit('preferences.pas');
    T:=P.Targets.AddUnit('xadmaster.pas');
    T:=P.Targets.AddUnit('cybergraphics.pas');
    T:=P.Targets.AddUnit('ptreplay.pas');
    T:=P.Targets.AddUnit('reqtools.pas');
    T:=P.Targets.AddUnit('amarquee.pas');
    T:=P.Targets.AddUnit('ttengine.pas');
    T:=P.Targets.AddUnit('triton.pas');
    T:=P.Targets.AddUnit('zlib.pas');
    T:=P.Targets.AddUnit('mysticview.pas');
    T:=P.Targets.AddUnit('ahi.pas');
    T:=P.Targets.AddUnit('lucyplay.pas');
    T:=P.Targets.AddUnit('ahi_sub.pas');

    P.ExamplePath.Add('examples');
    P.ExamplePath.Add('examples/otherlibs');
    T:=P.Targets.AddExampleProgram('amicube.pas');
    T:=P.Targets.AddExampleProgram('simple_timer.pas');
    T:=P.Targets.AddExampleProgram('getfontasl.pas');
    T:=P.Targets.AddExampleProgram('asltest.pas');
    T:=P.Targets.AddExampleProgram('easygadtools.pas');
    T:=P.Targets.AddExampleProgram('bezier.pas');
    T:=P.Targets.AddExampleProgram('listtest.pas');
    T:=P.Targets.AddExampleProgram('dirdemo.pas');
    T:=P.Targets.AddExampleProgram('getmultifiles.pas');
    T:=P.Targets.AddExampleProgram('showdevs.pas');
    T:=P.Targets.AddExampleProgram('checkmem.pas');
    T:=P.Targets.AddExampleProgram('moire.pas');
    T:=P.Targets.AddExampleProgram('wbtest.pas');
    T:=P.Targets.AddExampleProgram('stars.pas');
    T:=P.Targets.AddExampleProgram('snow.pas');
    T:=P.Targets.AddExampleProgram('imagegadget.pas');
    T:=P.Targets.AddExampleProgram('talk2boopsi.pas');
    T:=P.Targets.AddExampleProgram('easter.pas');
    T:=P.Targets.AddExampleProgram('gtmenu.pas');
    T:=P.Targets.AddExampleProgram('getdate.pas');
    T:=P.Targets.AddExampleProgram('deviceinfo.pas');
    T:=P.Targets.AddExampleProgram('bezier2.pas');
    T:=P.Targets.AddExampleProgram('sortdemo.pas');
    T:=P.Targets.AddExampleProgram('toolmanager3.pas');
    T:=P.Targets.AddExampleProgram('tritongadgets.pas');
    T:=P.Targets.AddExampleProgram('slider.pas');
    T:=P.Targets.AddExampleProgram('scroller.pas');
    T:=P.Targets.AddExampleProgram('rtdemo.pas');
    T:=P.Targets.AddExampleProgram('palette.pas');
    T:=P.Targets.AddExampleProgram('toolmanager1.pas');
    T:=P.Targets.AddExampleProgram('checkbox.pas');
    T:=P.Targets.AddExampleProgram('toolmanager2.pas');
    T:=P.Targets.AddExampleProgram('requestmodeid.pas');
    T:=P.Targets.AddExampleProgram('bestmodeid.pas');
    T:=P.Targets.AddExampleProgram('writetruecolordata.pas');
    T:=P.Targets.AddExampleProgram('string.pas');
    T:=P.Targets.AddExampleProgram('smallplay.pas');
    T:=P.Targets.AddExampleProgram('progindex.pas');
    T:=P.Targets.AddExampleProgram('openscreen.pas');
    T:=P.Targets.AddExampleProgram('listview.pas');
    T:=P.Targets.AddExampleProgram('p96checkboards.pas');
    T:=P.Targets.AddExampleProgram('modelist.pas');
    T:=P.Targets.AddExampleProgram('gttest.pas');
    T:=P.Targets.AddExampleProgram('gadgetdemo.pas');
    T:=P.Targets.AddExampleProgram('linklib.pas');
    T:=P.Targets.AddExampleProgram('demo.pas');
    T:=P.Targets.AddExampleProgram('amarqueetest.pas');
    T:=P.Targets.AddExampleProgram('openpip.pas');
    T:=P.Targets.AddExampleProgram('envprint.pas');
    T:=P.Targets.AddExampleProgram('penshare.pas');

    P.Sources.AddDoc('README.txt');
    P.Sources.AddDoc('units.txt');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
