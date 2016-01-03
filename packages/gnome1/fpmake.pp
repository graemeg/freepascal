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

    P:=AddPackage('gnome1');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=AllUnixOSes-[darwin,iphonesim,Android];
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];

    P.Author := 'Library: Gnome project, header: FPC team';
    P.License := 'Library: LGPL2 or later, header: LGPL2 with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Header for main library of Gnome (1?).';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='gnome1';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=AllUnixOSes;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    P.Dependencies.Add('gtk1');
    P.Dependencies.Add('imlib');

    T:=P.Targets.AddUnit('src/gconfclient/gconfclient.pp');
      T.IncludePath.Add('src/gconfclient');
      with T.Dependencies do
        begin
          AddInclude('gconflisteners.inc');
          AddInclude('gconfchangeset.inc');
          AddInclude('gconflisteners.inc');
          AddInclude('gconfchangeset.inc');
          AddUnit('gconf');
        end;
    T:=P.Targets.AddUnit('src/gconf/gconf.pp');
      T.IncludePath.Add('src/gconf');
      with T.Dependencies do
        begin
          AddInclude('gconfglibpublic.inc');
          AddInclude('gconferror.inc');
          AddInclude('gconfvalue.inc');
          AddInclude('gconfschema.inc');
          AddInclude('gconfengine.inc');
          AddInclude('gconfglibpublic.inc');
          AddInclude('gconferror.inc');
          AddInclude('gconfvalue.inc');
          AddInclude('gconfschema.inc');
          AddInclude('gconfengine.inc');
        end;
    T:=P.Targets.AddUnit('src/libart.pp');
    T:=P.Targets.AddUnit('src/libgnome/libgnome.pp');
      T.IncludePath.Add('src/libgnome');
      with T.Dependencies do
        begin
          AddInclude('gnomeutil.inc');
          AddInclude('gnomeconfig.inc');
          AddInclude('gnomedentry.inc');
          AddInclude('gnomeexec.inc');
          AddInclude('gnomehelp.inc');
          AddInclude('gnomei18n.inc');
          AddInclude('gnomemetadata.inc');
          AddInclude('gnomemime.inc');
          AddInclude('gnomemimeinfo.inc');
          AddInclude('gnomepaper.inc');
          AddInclude('gnomeremote.inc');
          AddInclude('gnomescore.inc');
          AddInclude('gnomesound.inc');
          AddInclude('gnometriggers.inc');
          AddInclude('gnomeurl.inc');
          AddInclude('gnomeutil.inc');
          AddInclude('gnomeconfig.inc');
          AddInclude('gnomedentry.inc');
          AddInclude('gnomeexec.inc');
          AddInclude('gnomehelp.inc');
          AddInclude('gnomei18n.inc');
          AddInclude('gnomemetadata.inc');
          AddInclude('gnomemime.inc');
          AddInclude('gnomemimeinfo.inc');
          AddInclude('gnomepaper.inc');
          AddInclude('gnomeremote.inc');
          AddInclude('gnomescore.inc');
          AddInclude('gnomesound.inc');
          AddInclude('gnometriggers.inc');
          AddInclude('gnomeurl.inc');
         end;
    T:=P.Targets.AddUnit('src/libgnomeui/libgnomeui.pp');
      T.IncludePath.Add('src/libgnomeui');
      with T.Dependencies do
        begin
          AddInclude('gnomeinit.inc');
          AddInclude('gtkpixmapmenuitem.inc');
          AddInclude('gtkclock.inc');
          AddInclude('gtkdial.inc');
          AddInclude('gnomeuidefs.inc');
          AddInclude('gnomegeometry.inc');
          AddInclude('gnometypebuiltins.inc');
          AddInclude('gnomeicontext.inc');
          AddInclude('gnomewinhints.inc');
          AddInclude('gnomedentryedit.inc');
          AddInclude('gnomepixmap.inc');
          AddInclude('gnomedialog.inc');
          AddInclude('gnomemessagebox.inc');
          AddInclude('gnomeclient.inc');
          AddInclude('gnomeabout.inc');
          AddInclude('gnomedock.inc');
          AddInclude('gnomeapp.inc');
          AddInclude('gnomeappbar.inc');
          AddInclude('gnomestock.inc');
          AddInclude('gnomeapphelper.inc');
          AddInclude('gnomedialogutil.inc');
          AddInclude('gnomeapputil.inc');
          AddInclude('gnomepopupmenu.inc');
          AddInclude('gnomepopuphelp.inc');
          AddInclude('gnomedateedit.inc');
          AddInclude('gnomeentry.inc');
          AddInclude('gnomefileentry.inc');
          AddInclude('gnomeiconentry.inc');
          AddInclude('gnomenumberentry.inc');
          AddInclude('gnomepixmapentry.inc');
          AddInclude('gnomecolorpicker.inc');
          AddInclude('gnomefontpicker.inc');
          AddInclude('gnomepaperselector.inc');
          AddInclude('gnomeiconsel.inc');
          AddInclude('gnomemdichild.inc');
          AddInclude('gnomemdigenericchild.inc');
          AddInclude('gnomemdi.inc');
          AddInclude('gnomemdisession.inc');
          AddInclude('gnomecanvas.inc');
          AddInclude('gnomecanvasline.inc');
          AddInclude('gnomecanvasimage.inc');
          AddInclude('gnomecanvasload.inc');
          AddInclude('gnomecanvasrectellipse.inc');
          AddInclude('gnomecanvaspolygon.inc');
          AddInclude('gnomecanvastext.inc');
          AddInclude('gnomecanvaswidget.inc');
          AddInclude('gnomeiconitem.inc');
          AddInclude('gnomecanvasutil.inc');
          AddInclude('gnomecalculator.inc');
          AddInclude('gnomeiconlist.inc');
          AddInclude('gnomehref.inc');
          AddInclude('gnomeprocbar.inc');
          AddInclude('gnomeanimator.inc');
          AddInclude('gnomescores.inc');
          AddInclude('gnomepropertybox.inc');
          AddInclude('gnomedruidpage.inc');
          AddInclude('gnomedruidpagestart.inc');
          AddInclude('gnomedruidpagestandard.inc');
          AddInclude('gnomedruidpagefinish.inc');
          AddInclude('gnomedruid.inc');
          AddInclude('gtkpixmapmenuitem.inc');
          AddInclude('gtkclock.inc');
          AddInclude('gtkdial.inc');
          AddInclude('gnomeuidefs.inc');
          AddInclude('gnomegeometry.inc');
          AddInclude('gnomeicontext.inc');
          AddInclude('gnometypebuiltins.inc');
          AddInclude('gnomewinhints.inc');
          AddInclude('gnomedentryedit.inc');
          AddInclude('gnomepixmap.inc');
          AddInclude('gnomedialog.inc');
          AddInclude('gnomemessagebox.inc');
          AddInclude('gnomeclient.inc');
          AddInclude('gnomeabout.inc');
          AddInclude('gnomedock.inc');
          AddInclude('gnomeapp.inc');
          AddInclude('gnomeappbar.inc');
          AddInclude('gnomestock.inc');
          AddInclude('gnomeapphelper.inc');
          AddInclude('gnomedialogutil.inc');
          AddInclude('gnomeapputil.inc');
          AddInclude('gnomepopupmenu.inc');
          AddInclude('gnomepopuphelp.inc');
          AddInclude('gnomedateedit.inc');
          AddInclude('gnomeentry.inc');
          AddInclude('gnomefileentry.inc');
          AddInclude('gnomeiconentry.inc');
          AddInclude('gnomenumberentry.inc');
          AddInclude('gnomepixmapentry.inc');
          AddInclude('gnomecolorpicker.inc');
          AddInclude('gnomefontpicker.inc');
          AddInclude('gnomepaperselector.inc');
          AddInclude('gnomeiconsel.inc');
          AddInclude('gnomemdichild.inc');
          AddInclude('gnomemdigenericchild.inc');
          AddInclude('gnomemdi.inc');
          AddInclude('gnomemdisession.inc');
          AddInclude('gnomecanvas.inc');
          AddInclude('gnomecanvasline.inc');
          AddInclude('gnomecanvasimage.inc');
          AddInclude('gnomecanvasload.inc');
          AddInclude('gnomecanvasrectellipse.inc');
          AddInclude('gnomecanvaspolygon.inc');
          AddInclude('gnomecanvastext.inc');
          AddInclude('gnomecanvaswidget.inc');
          AddInclude('gnomeiconitem.inc');
          AddInclude('gnomecanvasutil.inc');
          AddInclude('gnomecalculator.inc');
          AddInclude('gnomeiconlist.inc');
          AddInclude('gnomehref.inc');
          AddInclude('gnomeprocbar.inc');
          AddInclude('gnomeanimator.inc');
          AddInclude('gnomescores.inc');
          AddInclude('gnomepropertybox.inc');
          AddInclude('gnomedruidpage.inc');
          AddInclude('gnomedruidpagestart.inc');
          AddInclude('gnomedruidpagestandard.inc');
          AddInclude('gnomedruidpagefinish.inc');
          AddInclude('gnomedruid.inc');
          AddUnit('libgnome');
          AddUnit('libart');
        end;
    T:=P.Targets.AddUnit('src/zvt/libzvt.pp');
      T.IncludePath.Add('src/zvt');
      with T.Dependencies do
        begin
          AddInclude('lists.inc');
          AddInclude('vt.inc');
          AddInclude('vtx.inc');
          AddInclude('lists.inc');
          AddInclude('vt.inc');
          AddInclude('vtx.inc');
        end;

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('examples/testzvt.pp');
    P.Targets.AddExampleProgram('examples/gconfcallback2.pp');
    P.Targets.AddExampleProgram('examples/gconfexample.pp');
    P.Targets.AddExampleProgram('examples/gconfcallback1.pp');
    P.Targets.AddExampleProgram('examples/gnometest.pp');

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
