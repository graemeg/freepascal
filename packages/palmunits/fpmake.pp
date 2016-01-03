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

    P:=AddPackage('palmunits');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := [palmos];

    P.SourcePath.Add('src');
    P.Targets.AddUnit('day.pp',[PalmOS]);
    P.Targets.AddUnit('uicontrols.pp',[PalmOS]);
    P.Targets.AddUnit('inetmgr.pp',[PalmOS]);
    P.Targets.AddUnit('window.pp',[PalmOS]);
    P.Targets.AddUnit('textservicesmgr.pp',[PalmOS]);
    P.Targets.AddUnit('table.pp',[PalmOS]);
    P.Targets.AddUnit('serialmgr.pp',[PalmOS]);
    P.Targets.AddUnit('hal.pp',[PalmOS]);
    P.Targets.AddUnit('featuremgr.pp',[PalmOS]);
    P.Targets.AddUnit('hwrmiscflags.pp',[PalmOS]);
    P.Targets.AddUnit('crc.pp',[PalmOS]);
    P.Targets.AddUnit('palmlocale.pp',[PalmOS]);
    P.Targets.AddUnit('applaunchcmd.pp',[PalmOS]);
    P.Targets.AddUnit('fatalalert.pp',[PalmOS]);
    P.Targets.AddUnit('find_.pp',[PalmOS]);
    P.Targets.AddUnit('soundmgr.pp',[PalmOS]);
    P.Targets.AddUnit('form.pp',[PalmOS]);
    P.Targets.AddUnit('event_.pp',[PalmOS]);
    P.Targets.AddUnit('attentionmgr.pp',[PalmOS]);
    P.Targets.AddUnit('overlaymgr.pp',[PalmOS]);
    P.Targets.AddUnit('slotdrvrlib.pp',[PalmOS]);
    P.Targets.AddUnit('modemmgr.pp',[PalmOS]);
    P.Targets.AddUnit('menu_.pp',[PalmOS]);
    P.Targets.AddUnit('privaterecords.pp',[PalmOS]);
    P.Targets.AddUnit('exgmgr.pp',[PalmOS]);
    P.Targets.AddUnit('fslib.pp',[PalmOS]);
    P.Targets.AddUnit('timemgr.pp',[PalmOS]);
    P.Targets.AddUnit('libtraps.pp',[PalmOS]);
    P.Targets.AddUnit('control.pp',[PalmOS]);
    P.Targets.AddUnit('m68khwr.pp',[PalmOS]);
    P.Targets.AddUnit('rect.pp',[PalmOS]);
    P.Targets.AddUnit('graffiti.pp',[PalmOS]);
    P.Targets.AddUnit('telephonymgr.pp',[PalmOS]);
    P.Targets.AddUnit('keymgr.pp',[PalmOS]);
    P.Targets.AddUnit('consolemgr.pp',[PalmOS]);
    P.Targets.AddUnit('selday.pp',[PalmOS]);
    P.Targets.AddUnit('localize.pp',[PalmOS]);
    P.Targets.AddUnit('field.pp',[PalmOS]);
    P.Targets.AddUnit('seltimezone.pp',[PalmOS]);
    P.Targets.AddUnit('phonelookup.pp',[PalmOS]);
    P.Targets.AddUnit('pdiconst.pp',[PalmOS]);
    P.Targets.AddUnit('helper.pp',[PalmOS]);
    P.Targets.AddUnit('notifymgr.pp',[PalmOS]);
    P.Targets.AddUnit('systemresources.pp',[PalmOS]);
    P.Targets.AddUnit('intlmgr.pp',[PalmOS]);
    P.Targets.AddUnit('sysutil.pp',[PalmOS]);
    P.Targets.AddUnit('preferences.pp',[PalmOS]);
    P.Targets.AddUnit('netbitutils.pp',[PalmOS]);
    P.Targets.AddUnit('datamgr.pp',[PalmOS]);
    P.Targets.AddUnit('dlserver.pp',[PalmOS]);
    P.Targets.AddUnit('vfsmgr.pp',[PalmOS]);
    P.Targets.AddUnit('irlib.pp',[PalmOS]);
    P.Targets.AddUnit('font.pp',[PalmOS]);
    P.Targets.AddUnit('serialmgrold.pp',[PalmOS]);
    P.Targets.AddUnit('launcher.pp',[PalmOS]);
    P.Targets.AddUnit('palmos.pp',[PalmOS]);
    P.Targets.AddUnit('password.pp',[PalmOS]);
    P.Targets.AddUnit('seriallinkmgr.pp',[PalmOS]);
    P.Targets.AddUnit('palmcompatibility.pp',[PalmOS]);
    P.Targets.AddUnit('lz77mgr.pp',[PalmOS]);
    P.Targets.AddUnit('errorbase.pp',[PalmOS]);
    P.Targets.AddUnit('fontselect_.pp',[PalmOS]);
    P.Targets.AddUnit('keyboard.pp',[PalmOS]);
    P.Targets.AddUnit('telephonymgrui.pp',[PalmOS]);
    P.Targets.AddUnit('udamgr.pp',[PalmOS]);
    P.Targets.AddUnit('encrypt.pp',[PalmOS]);
    P.Targets.AddUnit('bitmap.pp',[PalmOS]);
    P.Targets.AddUnit('connectionmgr.pp',[PalmOS]);
    P.Targets.AddUnit('graffitireference.pp',[PalmOS]);
    P.Targets.AddUnit('seltime.pp',[PalmOS]);
    P.Targets.AddUnit('alarmmgr.pp',[PalmOS]);
    P.Targets.AddUnit('stringmgr.pp',[PalmOS]);
    P.Targets.AddUnit('smslib.pp',[PalmOS]);
    P.Targets.AddUnit('textmgr.pp',[PalmOS]);
    P.Targets.AddUnit('imcutils.pp',[PalmOS]);
    P.Targets.AddUnit('coretraps.pp',[PalmOS]);
    P.Targets.AddUnit('scrollbar.pp',[PalmOS]);
    P.Targets.AddUnit('category.pp',[PalmOS]);
    P.Targets.AddUnit('progress.pp',[PalmOS]);
    P.Targets.AddUnit('datetime.pp',[PalmOS]);
    P.Targets.AddUnit('chars.pp',[PalmOS]);
    P.Targets.AddUnit('list.pp',[PalmOS]);
    P.Targets.AddUnit('memorymgr.pp',[PalmOS]);
    P.Targets.AddUnit('netmgr.pp',[PalmOS]);
    P.Targets.AddUnit('exglib.pp',[PalmOS]);
    P.Targets.AddUnit('sysevent.pp',[PalmOS]);
    P.Targets.AddUnit('sysevtmgr.pp',[PalmOS]);
    P.Targets.AddUnit('aboutbox.pp',[PalmOS]);
    P.Targets.AddUnit('telephonymgrtypes.pp',[PalmOS]);
    P.Targets.AddUnit('localemgr.pp',[PalmOS]);
    P.Targets.AddUnit('floatmgr.pp',[PalmOS]);
    P.Targets.AddUnit('graffitishift.pp',[PalmOS]);
    P.Targets.AddUnit('clipboard.pp',[PalmOS]);
    P.Targets.AddUnit('inspoint.pp',[PalmOS]);
    P.Targets.AddUnit('expansionmgr.pp',[PalmOS]);
    P.Targets.AddUnit('filestream.pp',[PalmOS]);
    P.Targets.AddUnit('systemmgr.pp',[PalmOS]);
    P.Targets.AddUnit('uicolor.pp',[PalmOS]);
    P.Targets.AddUnit('uiresources.pp',[PalmOS]);
    P.Targets.AddUnit('penmgr.pp',[PalmOS]);
    P.Targets.AddUnit('pdilib.pp',[PalmOS]);
    P.Targets.AddUnit('helperserviceclass.pp',[PalmOS]);
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='palmunits';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
//    P.Dependencies.Add('x11');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
