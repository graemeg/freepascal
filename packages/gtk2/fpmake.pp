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

    P:=AddPackage('gtk2');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SupportBuildModes := [bmOneByOne];
    P.OSes:=AllUnixOSes+[Win32,Win64]-[darwin,iphonesim,Android];
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];

    P.Author := 'Library: Peter Mattis, Spencer Kimball and Josh MacDonald, header: Mattias Gaertner, Olaf Leidinger';
    P.License := 'Library: LGPL2.1, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Header to the GTK widgetset (v2.x).';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?

    P.Dependencies.Add('x11',AllUnixOSes);
    P.Dependencies.Add('cairo');

    P.SourcePath.Add('src');
    P.SourcePath.Add('src/glib');
    P.SourcePath.Add('src/atk');
    P.SourcePath.Add('src/pango');
    P.SourcePath.Add('src/pangocairo');
    P.SourcePath.Add('src/gtk+');
    P.SourcePath.Add('src/gtk+/gdk-pixbuf');
    P.SourcePath.Add('src/gtk+/gdk');
    P.SourcePath.Add('src/gtk+/gtk');
    P.SourcePath.Add('src/libglade');
    P.SourcePath.Add('src/gtkglext');
    P.SourcePath.Add('src/gtkext');

    // This is all so complex... Use the build-unit just like the Makefile.fpc does
    // and be happy with it. ;)
    T:=P.Targets.AddUnit('buildgtk2.pp');
      with t.UnitPath do
        begin
          Add('src/glib');
          Add('src/atk');
          Add('src/pango');
          Add('src/pangocairo');
          Add('src/gtk+');
          Add('src/gtk+/gdk-pixbuf');
          Add('src/gtk+/gdk');
          Add('src/gtk+/gtk');
          Add('src/libglade');
          Add('src/gtkglext');
          Add('src/gtkext');
          Add('src/gtk2x11');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T.Install:=False;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    T:=P.Targets.AddImplicitUnit('src/atk/atk.pas');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='gtk2';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=AllUnixOSes+[Win32,Win64];

    P.Dependencies.Add('x11',AllUnixOSes);

    T:=P.Targets.AddUnit('src/atk/atk.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     T.IncludePath.Add('src/atk');
     with T.Dependencies do
       begin
         AddInclude('atkincludes.inc');
         AddInclude('atkobject.inc');
         AddInclude('atkaction.inc');
         AddInclude('atkcomponent.inc');
         AddInclude('atkdocument.inc');
         AddInclude('atkeditabletext.inc');
         AddInclude('atkgobjectaccessible.inc');
         AddInclude('atkhyperlink.inc');
         AddInclude('atkhypertext.inc');
         AddInclude('atkimage.inc');
         AddInclude('atkobjectfactory.inc');
         AddInclude('atkregistry.inc');
         AddInclude('atkrelation.inc');
         AddInclude('atkrelationset.inc');
         AddInclude('atkselection.inc');
         AddInclude('atkstate.inc');
         AddInclude('atkstateset.inc');
         AddInclude('atkstreamablecontent.inc');
         AddInclude('atktable.inc');
         AddInclude('atktext.inc');
         AddInclude('atkutil.inc');
         AddInclude('atkvalue.inc');
         AddInclude('atkincludes.inc');
         AddInclude('atkobject.inc');
         AddInclude('atkaction.inc');
         AddInclude('atkcomponent.inc');
         AddInclude('atkdocument.inc');
         AddInclude('atkeditabletext.inc');
         AddInclude('atkgobjectaccessible.inc');
         AddInclude('atkhyperlink.inc');
         AddInclude('atkhypertext.inc');
         AddInclude('atkimage.inc');
         AddInclude('atkobjectfactory.inc');
         AddInclude('atkregistry.inc');
         AddInclude('atkrelation.inc');
         AddInclude('atkrelationset.inc');
         AddInclude('atkselection.inc');
         AddInclude('atkstate.inc');
         AddInclude('atkstateset.inc');
         AddInclude('atkstreamablecontent.inc');
         AddInclude('atktable.inc');
         AddInclude('atktext.inc');
         AddInclude('atkutil.inc');
         AddInclude('atkvalue.inc');
         AddInclude('atkincludes.inc');
         AddInclude('atkobject.inc');
         AddInclude('atkaction.inc');
         AddInclude('atkcomponent.inc');
         AddInclude('atkdocument.inc');
         AddInclude('atkeditabletext.inc');
         AddInclude('atkgobjectaccessible.inc');
         AddInclude('atkhyperlink.inc');
         AddInclude('atkhypertext.inc');
         AddInclude('atkimage.inc');
         AddInclude('atkobjectfactory.inc');
         AddInclude('atkregistry.inc');
         AddInclude('atkrelation.inc');
         AddInclude('atkrelationset.inc');
         AddInclude('atkselection.inc');
         AddInclude('atkstate.inc');
         AddInclude('atkstateset.inc');
         AddInclude('atkstreamablecontent.inc');
         AddInclude('atktable.inc');
         AddInclude('atktext.inc');
         AddInclude('atkutil.inc');
         AddInclude('atkvalue.inc');
         AddInclude('atkincludes.inc');
         AddInclude('atkobject.inc');
         AddInclude('atkaction.inc');
         AddInclude('atkcomponent.inc');
         AddInclude('atkdocument.inc');
         AddInclude('atkeditabletext.inc');
         AddInclude('atkgobjectaccessible.inc');
         AddInclude('atkhyperlink.inc');
         AddInclude('atkhypertext.inc');
         AddInclude('atkimage.inc');
         AddInclude('atkobjectfactory.inc');
         AddInclude('atkregistry.inc');
         AddInclude('atkrelation.inc');
         AddInclude('atkrelationset.inc');
         AddInclude('atkselection.inc');
         AddInclude('atkstate.inc');
         AddInclude('atkstateset.inc');
         AddInclude('atkstreamablecontent.inc');
         AddInclude('atktable.inc');
         AddInclude('atktext.inc');
         AddInclude('atkutil.inc');
         AddInclude('atkvalue.inc');
<<<<<<< HEAD
<<<<<<< HEAD
       end;
    T:=P.Targets.AddImplicitUnit('src/gtk+/gdk-pixbuf/gdk2pixbuf.pas');
=======
=======
>>>>>>> origin/fixes_2_2
         AddUnit('glib2');
       end;
    T:=P.Targets.AddUnit('src/buildgtk2.pp');
      with T.Dependencies do
        begin
          AddUnit('gtk2');
          AddUnit('libglade2');
          AddUnit('gdkglext');
          AddUnit('gtkglext');
          AddUnit('gdk2x',AllUnixOSes);
        end;
    T:=P.Targets.AddUnit('src/gtk+/gdk-pixbuf/gdk2pixbuf.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtk+/gdk-pixbuf');
      with T.Dependencies do
        begin
          AddInclude('gdk-pixbuf-loader.inc');
          AddInclude('gdk-pixbuf-loader.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddImplicitUnit('src/gtk+/gdk/gdk2.pas');
=======
          AddUnit('glib2');
        end;
    T:=P.Targets.AddUnit('src/gtk+/gdk/gdk2.pas');
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('glib2');
        end;
    T:=P.Targets.AddUnit('src/gtk+/gdk/gdk2.pas');
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtk+/gdk');
      with T.Dependencies do
        begin
          AddInclude('gdkincludes.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('gdkdisplaymanager.inc');
          AddInclude('gdkspawn.inc');
          AddInclude('gdkcairo.inc');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          AddInclude('gdkcolor.inc');
          AddInclude('gdkcursor.inc');
          AddInclude('gdkdnd.inc');
          AddInclude('gdkregion-generic.inc');
          AddInclude('gdkdrawable.inc');
          AddInclude('gdkevents.inc');
          AddInclude('gdkfont.inc');
          AddInclude('gdkgc.inc');
          AddInclude('gdkimage.inc');
          AddInclude('gdkinput.inc');
          AddInclude('gdkkeys.inc');
          AddInclude('gdkkeysyms.inc');
          AddInclude('gdkpango.inc');
          AddInclude('gdkpixbuf.inc');
          AddInclude('gdkpixmap.inc');
          AddInclude('gdkproperty.inc');
          AddInclude('gdkregion.inc');
          AddInclude('gdkrgb.inc');
          AddInclude('gdkdisplay.inc');
          AddInclude('gdkscreen.inc');
          AddInclude('gdkselection.inc');
          AddInclude('gdktypes.inc');
          AddInclude('gdkvisual.inc');
          AddInclude('gdkwindow.inc');
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('gdkincludes.inc');
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('gdkincludes.inc');
>>>>>>> origin/fixes_2_2
          AddInclude('gdkcolor.inc');
          AddInclude('gdkcursor.inc');
          AddInclude('gdkdnd.inc');
          AddInclude('gdkregion-generic.inc');
          AddInclude('gdkdrawable.inc');
          AddInclude('gdkevents.inc');
          AddInclude('gdkfont.inc');
          AddInclude('gdkgc.inc');
          AddInclude('gdkimage.inc');
          AddInclude('gdkinput.inc');
          AddInclude('gdkkeys.inc');
          AddInclude('gdkkeysyms.inc');
          AddInclude('gdkpango.inc');
          AddInclude('gdkpixbuf.inc');
          AddInclude('gdkpixmap.inc');
          AddInclude('gdkproperty.inc');
          AddInclude('gdkregion.inc');
          AddInclude('gdkrgb.inc');
          AddInclude('gdkdisplay.inc');
          AddInclude('gdkscreen.inc');
          AddInclude('gdkselection.inc');
          AddInclude('gdktypes.inc');
          AddInclude('gdkvisual.inc');
          AddInclude('gdkwindow.inc');
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('gdkincludes.inc');
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('gdkincludes.inc');
>>>>>>> origin/fixes_2_2
          AddInclude('gdkcolor.inc');
          AddInclude('gdkcursor.inc');
          AddInclude('gdkdnd.inc');
          AddInclude('gdkregion-generic.inc');
          AddInclude('gdkdrawable.inc');
          AddInclude('gdkevents.inc');
          AddInclude('gdkfont.inc');
          AddInclude('gdkgc.inc');
          AddInclude('gdkimage.inc');
          AddInclude('gdkinput.inc');
          AddInclude('gdkkeys.inc');
          AddInclude('gdkkeysyms.inc');
          AddInclude('gdkpango.inc');
          AddInclude('gdkpixbuf.inc');
          AddInclude('gdkpixmap.inc');
          AddInclude('gdkproperty.inc');
          AddInclude('gdkregion.inc');
          AddInclude('gdkrgb.inc');
          AddInclude('gdkdisplay.inc');
          AddInclude('gdkscreen.inc');
          AddInclude('gdkselection.inc');
          AddInclude('gdktypes.inc');
          AddInclude('gdkvisual.inc');
          AddInclude('gdkwindow.inc');
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('gdkincludes.inc');
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('gdkincludes.inc');
>>>>>>> origin/fixes_2_2
          AddInclude('gdkcolor.inc');
          AddInclude('gdkcursor.inc');
          AddInclude('gdkdnd.inc');
          AddInclude('gdkregion-generic.inc');
          AddInclude('gdkdrawable.inc');
          AddInclude('gdkevents.inc');
          AddInclude('gdkfont.inc');
          AddInclude('gdkgc.inc');
          AddInclude('gdkimage.inc');
          AddInclude('gdkinput.inc');
          AddInclude('gdkkeys.inc');
          AddInclude('gdkkeysyms.inc');
          AddInclude('gdkpango.inc');
          AddInclude('gdkpixbuf.inc');
          AddInclude('gdkpixmap.inc');
          AddInclude('gdkproperty.inc');
          AddInclude('gdkregion.inc');
          AddInclude('gdkrgb.inc');
          AddInclude('gdkdisplay.inc');
          AddInclude('gdkscreen.inc');
          AddInclude('gdkselection.inc');
          AddInclude('gdktypes.inc');
          AddInclude('gdkvisual.inc');
          AddInclude('gdkwindow.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddImplicitUnit('src/gtk2x11/gdk2x.pas',AllUnixOSes);
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
          AddUnit('gdk2pixbuf');
          AddUnit('pango');
        end;
    T:=P.Targets.AddUnit('src/gtk2x11/gdk2x.pas',AllUnixOSes);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtk2x11');
      T.IncludePath.Add('src/gtk2x11/include');
      with T.Dependencies do
        begin
          AddInclude('gdk2x11includes.inc');
          AddInclude('xsettings-common.inc');
          AddInclude('xsettings-client.inc');
          AddInclude('gdkdisplay-x11.inc');
          AddInclude('gdkdrawable-x11.inc');
          AddInclude('gdkinputprivate.inc');
          AddInclude('gdkpixmap-x11.inc');
          AddInclude('gdkprivate-x11.inc');
          AddInclude('gdkscreen-x11.inc');
          AddInclude('gdkwindow-x11.inc');
          AddInclude('gdkx.inc');
          AddInclude('gxid_proto.inc');
          AddInclude('mwmutil.inc');
          AddInclude('gdk2x11includes.inc');
          AddInclude('xsettings-common.inc');
          AddInclude('xsettings-client.inc');
          AddInclude('gdkdisplay-x11.inc');
          AddInclude('gdkdrawable-x11.inc');
          AddInclude('gdkinputprivate.inc');
          AddInclude('gdkpixmap-x11.inc');
          AddInclude('gdkprivate-x11.inc');
          AddInclude('gdkscreen-x11.inc');
          AddInclude('gdkwindow-x11.inc');
          AddInclude('gdkx.inc');
          AddInclude('gxid_proto.inc');
          AddInclude('mwmutil.inc');
          AddInclude('gdk2x11includes.inc');
          AddInclude('xsettings-common.inc');
          AddInclude('xsettings-client.inc');
          AddInclude('gdkdisplay-x11.inc');
          AddInclude('gdkdrawable-x11.inc');
          AddInclude('gdkinputprivate.inc');
          AddInclude('gdkpixmap-x11.inc');
          AddInclude('gdkprivate-x11.inc');
          AddInclude('gdkscreen-x11.inc');
          AddInclude('gdkwindow-x11.inc');
          AddInclude('gdkx.inc');
          AddInclude('gxid_proto.inc');
          AddInclude('mwmutil.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddImplicitUnit('src/gtkglext/gdkglext.pas');
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
          AddUnit('gdk2');
        end;
    T:=P.Targets.AddUnit('src/gtkglext/gdkglext.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtkglext');
      with T.Dependencies do
        begin
          AddInclude('gdkglext_includes.inc');
          AddInclude('gdkgldefs.inc');
          AddInclude('gdkglversion.inc');
          AddInclude('gdkgltokens.inc');
          AddInclude('gdkgltypes.inc');
          AddInclude('gdkglenumtypes.inc');
          AddInclude('gdkglinit.inc');
          AddInclude('gdkglquery.inc');
          AddInclude('gdkglconfig.inc');
          AddInclude('gdkglcontext.inc');
          AddInclude('gdkgldrawable.inc');
          AddInclude('gdkglpixmap.inc');
          AddInclude('gdkglwindow.inc');
          AddInclude('gdkglfont.inc');
          AddInclude('gdkglshapes.inc');
          AddInclude('gdkglext_includes.inc');
          AddInclude('gdkgldefs.inc');
          AddInclude('gdkglversion.inc');
          AddInclude('gdkgltokens.inc');
          AddInclude('gdkgltypes.inc');
          AddInclude('gdkglenumtypes.inc');
          AddInclude('gdkglinit.inc');
          AddInclude('gdkglquery.inc');
          AddInclude('gdkglconfig.inc');
          AddInclude('gdkglcontext.inc');
          AddInclude('gdkgldrawable.inc');
          AddInclude('gdkglpixmap.inc');
          AddInclude('gdkglwindow.inc');
          AddInclude('gdkglfont.inc');
          AddInclude('gdkglshapes.inc');
          AddInclude('gdkglext_includes.inc');
          AddInclude('gdkgldefs.inc');
          AddInclude('gdkglversion.inc');
          AddInclude('gdkgltokens.inc');
          AddInclude('gdkgltypes.inc');
          AddInclude('gdkglenumtypes.inc');
          AddInclude('gdkglinit.inc');
          AddInclude('gdkglquery.inc');
          AddInclude('gdkglconfig.inc');
          AddInclude('gdkglcontext.inc');
          AddInclude('gdkgldrawable.inc');
          AddInclude('gdkglpixmap.inc');
          AddInclude('gdkglwindow.inc');
          AddInclude('gdkglfont.inc');
          AddInclude('gdkglshapes.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddImplicitUnit('src/glib/glib2.pas');
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
          AddUnit('gdk2');
        end;
    T:=P.Targets.AddUnit('src/glib/glib2.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/glib');
      with T.Dependencies do
        begin
          AddInclude('gtypes.inc');
          AddInclude('glibconfig.inc');
          AddInclude('gquark.inc');
          AddInclude('gvaluecollector.inc');
          AddInclude('gtype.inc');
          AddInclude('genums.inc');
          AddInclude('gvalue.inc');
          AddInclude('gtypeplugin.inc');
          AddInclude('gdataset.inc');
          AddInclude('gslist.inc');
          AddInclude('glist.inc');
          AddInclude('gparam.inc');
          AddInclude('gboxed.inc');
          AddInclude('gtype.inc');
          AddInclude('gvalue.inc');
          AddInclude('gvaluearray.inc');
          AddInclude('gvaluecollector.inc');
          AddInclude('gvaluetypes.inc');
          AddInclude('gparam.inc');
          AddInclude('gclosure.inc');
          AddInclude('gsignal.inc');
          AddInclude('gtypeplugin.inc');
          AddInclude('gobject.inc');
          AddInclude('gmacros.inc');
          AddInclude('gtypes.inc');
          AddInclude('genums.inc');
          AddInclude('glibconfig.inc');
          AddInclude('gstrfuncs.inc');
          AddInclude('gutils.inc');
          AddInclude('galloca.inc');
          AddInclude('ghash.inc');
          AddInclude('gquark.inc');
          AddInclude('gerror.inc');
          AddInclude('gbacktrace.inc');
          AddInclude('gmem.inc');
          AddInclude('garray.inc');
          AddInclude('gslist.inc');
          AddInclude('glist.inc');
          AddInclude('gcache.inc');
          AddInclude('gcompletion.inc');
          AddInclude('gconvert.inc');
          AddInclude('gdataset.inc');
          AddInclude('gdate.inc');
          AddInclude('gdir.inc');
          AddInclude('gfileutils.inc');
          AddInclude('ghook.inc');
          AddInclude('gthread.inc');
          AddInclude('gthreadpool.inc');
          AddInclude('gtimer.inc');
          AddInclude('gmain.inc');
          AddInclude('gasyncqueue.inc');
          AddInclude('gunicode.inc');
          AddInclude('gstring.inc');
          AddInclude('giochannel.inc');
          AddInclude('gmessages.inc');
          AddInclude('gmarkup.inc');
          AddInclude('gnode.inc');
          AddInclude('gtree.inc');
          AddInclude('gpattern.inc');
          AddInclude('gprimes.inc');
          AddInclude('gqsort.inc');
          AddInclude('gqueue.inc');
          AddInclude('grand.inc');
          AddInclude('grel.inc');
          AddInclude('gscanner.inc');
          AddInclude('gshell.inc');
          AddInclude('gspawn.inc');
          AddInclude('gboxed.inc');
          AddInclude('gmodule.inc');
          AddInclude('gmarshal.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('gincludes.inc');
          AddInclude('goption.inc');
          AddInclude('gwin32.inc',AllWindowsOSes);
        end;
    T:=P.Targets.AddImplicitUnit('src/gtk+/gtk/gtk2.pas');
=======
        end;
    T:=P.Targets.AddUnit('src/gtk+/gtk/gtk2.pas');
>>>>>>> graemeg/fixes_2_2
=======
        end;
    T:=P.Targets.AddUnit('src/gtk+/gtk/gtk2.pas');
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtk+/gtk');
      with T.Dependencies do
        begin
          AddInclude('gtkincludes.inc');
          AddInclude('gtkobject.inc');
          AddInclude('gtkdebug.inc');
          AddInclude('gtktypeutils.inc');
          AddInclude('gtkwidget.inc');
          AddInclude('gtkmisc.inc');
          AddInclude('gtkaccelgroup.inc');
          AddInclude('gtkcontainer.inc');
          AddInclude('gtkbin.inc');
          AddInclude('gtkwindow.inc');
          AddInclude('gtklabel.inc');
          AddInclude('gtkaccellabel.inc');
          AddInclude('gtkaccelmap.inc');
          AddInclude('gtkaccessible.inc');
          AddInclude('gtkadjustment.inc');
          AddInclude('gtkalignment.inc');
          AddInclude('gtkframe.inc');
          AddInclude('gtkaspectframe.inc');
          AddInclude('gtkarrow.inc');
          AddInclude('gtkbindings.inc');
          AddInclude('gtkbox.inc');
          AddInclude('gtkbbox.inc');
          AddInclude('gtkbutton.inc');
          AddInclude('gtkcalendar.inc');
          AddInclude('gtkcelleditable.inc');
          AddInclude('gtkcellrenderer.inc');
          AddInclude('gtkcellrenderertext.inc');
          AddInclude('gtkcellrenderertoggle.inc');
          AddInclude('gtkcellrendererpixbuf.inc');
          AddInclude('gtkitem.inc');
          AddInclude('gtkmenuitem.inc');
          AddInclude('gtktogglebutton.inc');
          AddInclude('gtkcheckbutton.inc');
          AddInclude('gtkcheckmenuitem.inc');
          AddInclude('gtkclipboard.inc');
          AddInclude('gtkclist.inc');
          AddInclude('gtkdialog.inc');
          AddInclude('gtkvbox.inc');
          AddInclude('gtkcolorsel.inc');
          AddInclude('gtkcolorseldialog.inc');
          AddInclude('gtkhbox.inc');
          AddInclude('gtkcombo.inc');
          AddInclude('gtkctree.inc');
          AddInclude('gtkdrawingarea.inc');
          AddInclude('gtkcurve.inc');
          AddInclude('gtkdnd.inc');
          AddInclude('gtkeditable.inc');
          AddInclude('gtkimcontext.inc');
          AddInclude('gtkmenushell.inc');
          AddInclude('gtkmenu.inc');
          AddInclude('gtkentry.inc');
          AddInclude('gtkenums.inc');
          AddInclude('gtkeventbox.inc');
          AddInclude('fnmatch.inc');
          AddInclude('gtkfilesel.inc');
          AddInclude('gtkfixed.inc');
          AddInclude('gtkfontsel.inc');
          AddInclude('gtkgamma.inc');
          AddInclude('gtkgc.inc');
          AddInclude('gtkhandlebox.inc');
          AddInclude('gtkpaned.inc');
          AddInclude('gtkhbbox.inc');
          AddInclude('gtkhpaned.inc');
          AddInclude('gtkruler.inc');
          AddInclude('gtkhruler.inc');
          AddInclude('gtksettings.inc');
          AddInclude('gtkrc.inc');
          AddInclude('gtkstyle.inc');
          AddInclude('gtkrange.inc');
          AddInclude('gtkscale.inc');
          AddInclude('gtkhscale.inc');
          AddInclude('gtkscrollbar.inc');
          AddInclude('gtkhscrollbar.inc');
          AddInclude('gtkseparator.inc');
          AddInclude('gtkhseparator.inc');
          AddInclude('gtkiconfactory.inc');
          AddInclude('gtkimage.inc');
          AddInclude('gtkimagemenuitem.inc');
          AddInclude('gtkimcontextsimple.inc');
          AddInclude('gtkimmulticontext.inc');
          AddInclude('gtkinputdialog.inc');
          AddInclude('gtkinvisible.inc');
          AddInclude('gtkitemfactory.inc');
          AddInclude('gtklayout.inc');
          AddInclude('gtklist.inc');
          AddInclude('gtklistitem.inc');
          AddInclude('gtktreemodel.inc');
          AddInclude('gtktreesortable.inc');
          AddInclude('gtktreemodelsort.inc');
          AddInclude('gtkliststore.inc');
          AddInclude('gtkmain.inc');
          AddInclude('gtkmenubar.inc');
          AddInclude('gtkmessagedialog.inc');
          AddInclude('gtknotebook.inc');
          AddInclude('gtkoldeditable.inc');
          AddInclude('gtkoptionmenu.inc');
          AddInclude('gtkpixmap.inc');
          AddInclude('gtkplug.inc');
          AddInclude('gtkpreview.inc');
          AddInclude('gtkprogress.inc');
          AddInclude('gtkprogressbar.inc');
          AddInclude('gtkradiobutton.inc');
          AddInclude('gtkradiomenuitem.inc');
          AddInclude('gtkscrolledwindow.inc');
          AddInclude('gtkselection.inc');
          AddInclude('gtkseparatormenuitem.inc');
          AddInclude('gtksignal.inc');
          AddInclude('gtksizegroup.inc');
          AddInclude('gtksocket.inc');
          AddInclude('gtkspinbutton.inc');
          AddInclude('gtkstock.inc');
          AddInclude('gtkstatusbar.inc');
          AddInclude('gtktable.inc');
          AddInclude('gtktearoffmenuitem.inc');
          AddInclude('gtktext.inc');
          AddInclude('gtktextiter.inc');
          AddInclude('gtktexttag.inc');
          AddInclude('gtktexttagtable.inc');
          AddInclude('gtktextmark.inc');
          AddInclude('gtktextmarkprivate.inc');
          AddInclude('gtktextchild.inc');
          AddInclude('gtktextchildprivate.inc');
          AddInclude('gtktextsegment.inc');
          AddInclude('gtktextbtree.inc');
          AddInclude('gtktextbuffer.inc');
          AddInclude('gtktextlayout.inc');
          AddInclude('gtktextview.inc');
          AddInclude('gtktipsquery.inc');
          AddInclude('gtktooltips.inc');
          AddInclude('gtktoolbar.inc');
          AddInclude('gtktree.inc');
          AddInclude('gtktreednd.inc');
          AddInclude('gtktreeitem.inc');
          AddInclude('gtktreeselection.inc');
          AddInclude('gtktreestore.inc');
          AddInclude('gtktreeviewcolumn.inc');
          AddInclude('gtkrbtree.inc');
          AddInclude('gtktreeprivate.inc');
          AddInclude('gtktreeview.inc');
          AddInclude('gtkvbbox.inc');
          AddInclude('gtkviewport.inc');
          AddInclude('gtkvpaned.inc');
          AddInclude('gtkvruler.inc');
          AddInclude('gtkvscale.inc');
          AddInclude('gtkvscrollbar.inc');
          AddInclude('gtkvseparator.inc');
          AddInclude('gtkfilefilter.inc');
          AddInclude('gtkfilesystem.inc');
          AddInclude('gtkcellrenderercombo.inc');
          AddInclude('gtkfilechooser.inc');
          AddInclude('gtkfilechooserprivate.inc');
          AddInclude('gtkfilechooserutils.inc');
          AddInclude('gtkfilechooserwidget.inc');
          AddInclude('gtkfilechooserdialog.inc');
          AddInclude('gtkexpander.inc');
          AddInclude('gtkaction.inc');
          AddInclude('gtkactiongroup.inc');
          AddInclude('gtktoggleaction.inc');
          AddInclude('gtkradioaction.inc');
          AddInclude('gtkcombobox.inc');
          AddInclude('gtkcomboboxentry.inc');
          AddInclude('gtktoolitem.inc');
          AddInclude('gtktoolbutton.inc');
          AddInclude('gtktoggletoolbutton.inc');
          AddInclude('gtkradiotoolbutton.inc');
          AddInclude('gtkfontbutton.inc');
          AddInclude('gtkicontheme.inc');
          AddInclude('gtkcolorbutton.inc');
          AddInclude('gtkcelllayout.inc');
          AddInclude('gtkentrycompletion.inc');
          AddInclude('gtkuimanager.inc');
          AddInclude('gtktreemodelfilter.inc');
          AddInclude('gtkincludes.inc');
          AddInclude('gtkobject.inc');
          AddInclude('gtkdebug.inc');
          AddInclude('gtktypeutils.inc');
          AddInclude('gtkwidget.inc');
          AddInclude('gtkmisc.inc');
          AddInclude('gtkaccelgroup.inc');
          AddInclude('gtkcontainer.inc');
          AddInclude('gtkbin.inc');
          AddInclude('gtkwindow.inc');
          AddInclude('gtklabel.inc');
          AddInclude('gtkaccellabel.inc');
          AddInclude('gtkaccelmap.inc');
          AddInclude('gtkaccessible.inc');
          AddInclude('gtkadjustment.inc');
          AddInclude('gtkalignment.inc');
          AddInclude('gtkframe.inc');
          AddInclude('gtkaspectframe.inc');
          AddInclude('gtkarrow.inc');
          AddInclude('gtkbindings.inc');
          AddInclude('gtkbox.inc');
          AddInclude('gtkbbox.inc');
          AddInclude('gtkbutton.inc');
          AddInclude('gtkcalendar.inc');
          AddInclude('gtkcelleditable.inc');
          AddInclude('gtkcellrenderer.inc');
          AddInclude('gtkcellrenderertext.inc');
          AddInclude('gtkcellrenderertoggle.inc');
          AddInclude('gtkcellrendererpixbuf.inc');
          AddInclude('gtkitem.inc');
          AddInclude('gtkmenuitem.inc');
          AddInclude('gtktogglebutton.inc');
          AddInclude('gtkcheckbutton.inc');
          AddInclude('gtkcheckmenuitem.inc');
          AddInclude('gtkclipboard.inc');
          AddInclude('gtkclist.inc');
          AddInclude('gtkdialog.inc');
          AddInclude('gtkvbox.inc');
          AddInclude('gtkcolorsel.inc');
          AddInclude('gtkcolorseldialog.inc');
          AddInclude('gtkhbox.inc');
          AddInclude('gtkcombo.inc');
          AddInclude('gtkctree.inc');
          AddInclude('gtkdrawingarea.inc');
          AddInclude('gtkcurve.inc');
          AddInclude('gtkdnd.inc');
          AddInclude('gtkeditable.inc');
          AddInclude('gtkimcontext.inc');
          AddInclude('gtkmenushell.inc');
          AddInclude('gtkmenu.inc');
          AddInclude('gtkentry.inc');
          AddInclude('gtkenums.inc');
          AddInclude('gtkeventbox.inc');
          AddInclude('fnmatch.inc');
          AddInclude('gtkfilesel.inc');
          AddInclude('gtkfixed.inc');
          AddInclude('gtkfontsel.inc');
          AddInclude('gtkgamma.inc');
          AddInclude('gtkgc.inc');
          AddInclude('gtkhandlebox.inc');
          AddInclude('gtkpaned.inc');
          AddInclude('gtkhbbox.inc');
          AddInclude('gtkhpaned.inc');
          AddInclude('gtkruler.inc');
          AddInclude('gtkhruler.inc');
          AddInclude('gtksettings.inc');
          AddInclude('gtkrc.inc');
          AddInclude('gtkstyle.inc');
          AddInclude('gtkrange.inc');
          AddInclude('gtkscale.inc');
          AddInclude('gtkhscale.inc');
          AddInclude('gtkscrollbar.inc');
          AddInclude('gtkhscrollbar.inc');
          AddInclude('gtkseparator.inc');
          AddInclude('gtkhseparator.inc');
          AddInclude('gtkiconfactory.inc');
          AddInclude('gtkimage.inc');
          AddInclude('gtkimagemenuitem.inc');
          AddInclude('gtkimcontextsimple.inc');
          AddInclude('gtkimmulticontext.inc');
          AddInclude('gtkinputdialog.inc');
          AddInclude('gtkinvisible.inc');
          AddInclude('gtkitemfactory.inc');
          AddInclude('gtklayout.inc');
          AddInclude('gtklist.inc');
          AddInclude('gtklistitem.inc');
          AddInclude('gtktreemodel.inc');
          AddInclude('gtktreesortable.inc');
          AddInclude('gtktreemodelsort.inc');
          AddInclude('gtkliststore.inc');
          AddInclude('gtkmain.inc');
          AddInclude('gtkmenubar.inc');
          AddInclude('gtkmessagedialog.inc');
          AddInclude('gtknotebook.inc');
          AddInclude('gtkoldeditable.inc');
          AddInclude('gtkoptionmenu.inc');
          AddInclude('gtkpixmap.inc');
          AddInclude('gtkplug.inc');
          AddInclude('gtkpreview.inc');
          AddInclude('gtkprogress.inc');
          AddInclude('gtkprogressbar.inc');
          AddInclude('gtkradiobutton.inc');
          AddInclude('gtkradiomenuitem.inc');
          AddInclude('gtkscrolledwindow.inc');
          AddInclude('gtkselection.inc');
          AddInclude('gtkseparatormenuitem.inc');
          AddInclude('gtksignal.inc');
          AddInclude('gtksizegroup.inc');
          AddInclude('gtksocket.inc');
          AddInclude('gtkspinbutton.inc');
          AddInclude('gtkstock.inc');
          AddInclude('gtkstatusbar.inc');
          AddInclude('gtktable.inc');
          AddInclude('gtktearoffmenuitem.inc');
          AddInclude('gtktext.inc');
          AddInclude('gtktextiter.inc');
          AddInclude('gtktexttag.inc');
          AddInclude('gtktexttagtable.inc');
          AddInclude('gtktextmark.inc');
          AddInclude('gtktextmarkprivate.inc');
          AddInclude('gtktextchild.inc');
          AddInclude('gtktextchildprivate.inc');
          AddInclude('gtktextsegment.inc');
          AddInclude('gtktextbtree.inc');
          AddInclude('gtktextbuffer.inc');
          AddInclude('gtktextlayout.inc');
          AddInclude('gtktextview.inc');
          AddInclude('gtktipsquery.inc');
          AddInclude('gtktooltips.inc');
          AddInclude('gtktoolbar.inc');
          AddInclude('gtktree.inc');
          AddInclude('gtktreednd.inc');
          AddInclude('gtktreeitem.inc');
          AddInclude('gtktreeselection.inc');
          AddInclude('gtktreestore.inc');
          AddInclude('gtktreeviewcolumn.inc');
          AddInclude('gtkrbtree.inc');
          AddInclude('gtktreeprivate.inc');
          AddInclude('gtktreeview.inc');
          AddInclude('gtkvbbox.inc');
          AddInclude('gtkviewport.inc');
          AddInclude('gtkvpaned.inc');
          AddInclude('gtkvruler.inc');
          AddInclude('gtkvscale.inc');
          AddInclude('gtkvscrollbar.inc');
          AddInclude('gtkvseparator.inc');
          AddInclude('gtkfilefilter.inc');
          AddInclude('gtkfilesystem.inc');
          AddInclude('gtkcellrenderercombo.inc');
          AddInclude('gtkfilechooser.inc');
          AddInclude('gtkfilechooserprivate.inc');
          AddInclude('gtkfilechooserutils.inc');
          AddInclude('gtkfilechooserwidget.inc');
          AddInclude('gtkfilechooserdialog.inc');
          AddInclude('gtkexpander.inc');
          AddInclude('gtkaction.inc');
          AddInclude('gtkactiongroup.inc');
          AddInclude('gtktoggleaction.inc');
          AddInclude('gtkradioaction.inc');
          AddInclude('gtkcombobox.inc');
          AddInclude('gtkcomboboxentry.inc');
          AddInclude('gtktoolitem.inc');
          AddInclude('gtktoolbutton.inc');
          AddInclude('gtktoggletoolbutton.inc');
          AddInclude('gtkradiotoolbutton.inc');
          AddInclude('gtkfontbutton.inc');
          AddInclude('gtkicontheme.inc');
          AddInclude('gtkcolorbutton.inc');
          AddInclude('gtkcelllayout.inc');
          AddInclude('gtkentrycompletion.inc');
          AddInclude('gtkuimanager.inc');
          AddInclude('gtktreemodelfilter.inc');
          AddInclude('gtkincludes.inc');
          AddInclude('gtkobject.inc');
          AddInclude('gtkdebug.inc');
          AddInclude('gtktypeutils.inc');
          AddInclude('gtkwidget.inc');
          AddInclude('gtkmisc.inc');
          AddInclude('gtkaccelgroup.inc');
          AddInclude('gtkcontainer.inc');
          AddInclude('gtkbin.inc');
          AddInclude('gtkwindow.inc');
          AddInclude('gtklabel.inc');
          AddInclude('gtkaccellabel.inc');
          AddInclude('gtkaccelmap.inc');
          AddInclude('gtkaccessible.inc');
          AddInclude('gtkadjustment.inc');
          AddInclude('gtkalignment.inc');
          AddInclude('gtkframe.inc');
          AddInclude('gtkaspectframe.inc');
          AddInclude('gtkarrow.inc');
          AddInclude('gtkbindings.inc');
          AddInclude('gtkbox.inc');
          AddInclude('gtkbbox.inc');
          AddInclude('gtkbutton.inc');
          AddInclude('gtkcalendar.inc');
          AddInclude('gtkcelleditable.inc');
          AddInclude('gtkcellrenderer.inc');
          AddInclude('gtkcellrenderertext.inc');
          AddInclude('gtkcellrenderertoggle.inc');
          AddInclude('gtkcellrendererpixbuf.inc');
          AddInclude('gtkitem.inc');
          AddInclude('gtkmenuitem.inc');
          AddInclude('gtktogglebutton.inc');
          AddInclude('gtkcheckbutton.inc');
          AddInclude('gtkcheckmenuitem.inc');
          AddInclude('gtkclipboard.inc');
          AddInclude('gtkclist.inc');
          AddInclude('gtkdialog.inc');
          AddInclude('gtkvbox.inc');
          AddInclude('gtkcolorsel.inc');
          AddInclude('gtkcolorseldialog.inc');
          AddInclude('gtkhbox.inc');
          AddInclude('gtkcombo.inc');
          AddInclude('gtkctree.inc');
          AddInclude('gtkdrawingarea.inc');
          AddInclude('gtkcurve.inc');
          AddInclude('gtkdnd.inc');
          AddInclude('gtkeditable.inc');
          AddInclude('gtkimcontext.inc');
          AddInclude('gtkmenushell.inc');
          AddInclude('gtkmenu.inc');
          AddInclude('gtkentry.inc');
          AddInclude('gtkenums.inc');
          AddInclude('gtkeventbox.inc');
          AddInclude('fnmatch.inc');
          AddInclude('gtkfilesel.inc');
          AddInclude('gtkfixed.inc');
          AddInclude('gtkfontsel.inc');
          AddInclude('gtkgamma.inc');
          AddInclude('gtkgc.inc');
          AddInclude('gtkhandlebox.inc');
          AddInclude('gtkpaned.inc');
          AddInclude('gtkhbbox.inc');
          AddInclude('gtkhpaned.inc');
          AddInclude('gtkruler.inc');
          AddInclude('gtkhruler.inc');
          AddInclude('gtksettings.inc');
          AddInclude('gtkrc.inc');
          AddInclude('gtkstyle.inc');
          AddInclude('gtkrange.inc');
          AddInclude('gtkscale.inc');
          AddInclude('gtkhscale.inc');
          AddInclude('gtkscrollbar.inc');
          AddInclude('gtkhscrollbar.inc');
          AddInclude('gtkseparator.inc');
          AddInclude('gtkhseparator.inc');
          AddInclude('gtkiconfactory.inc');
          AddInclude('gtkimage.inc');
          AddInclude('gtkimagemenuitem.inc');
          AddInclude('gtkimcontextsimple.inc');
          AddInclude('gtkimmulticontext.inc');
          AddInclude('gtkinputdialog.inc');
          AddInclude('gtkinvisible.inc');
          AddInclude('gtkitemfactory.inc');
          AddInclude('gtklayout.inc');
          AddInclude('gtklist.inc');
          AddInclude('gtklistitem.inc');
          AddInclude('gtktreemodel.inc');
          AddInclude('gtktreesortable.inc');
          AddInclude('gtktreemodelsort.inc');
          AddInclude('gtkliststore.inc');
          AddInclude('gtkmain.inc');
          AddInclude('gtkmenubar.inc');
          AddInclude('gtkmessagedialog.inc');
          AddInclude('gtknotebook.inc');
          AddInclude('gtkoldeditable.inc');
          AddInclude('gtkoptionmenu.inc');
          AddInclude('gtkpixmap.inc');
          AddInclude('gtkplug.inc');
          AddInclude('gtkpreview.inc');
          AddInclude('gtkprogress.inc');
          AddInclude('gtkprogressbar.inc');
          AddInclude('gtkradiobutton.inc');
          AddInclude('gtkradiomenuitem.inc');
          AddInclude('gtkscrolledwindow.inc');
          AddInclude('gtkselection.inc');
          AddInclude('gtkseparatormenuitem.inc');
          AddInclude('gtksignal.inc');
          AddInclude('gtksizegroup.inc');
          AddInclude('gtksocket.inc');
          AddInclude('gtkspinbutton.inc');
          AddInclude('gtkstock.inc');
          AddInclude('gtkstatusbar.inc');
          AddInclude('gtktable.inc');
          AddInclude('gtktearoffmenuitem.inc');
          AddInclude('gtktext.inc');
          AddInclude('gtktextiter.inc');
          AddInclude('gtktexttag.inc');
          AddInclude('gtktexttagtable.inc');
          AddInclude('gtktextmark.inc');
          AddInclude('gtktextmarkprivate.inc');
          AddInclude('gtktextchild.inc');
          AddInclude('gtktextchildprivate.inc');
          AddInclude('gtktextsegment.inc');
          AddInclude('gtktextbtree.inc');
          AddInclude('gtktextbuffer.inc');
          AddInclude('gtktextlayout.inc');
          AddInclude('gtktextview.inc');
          AddInclude('gtktipsquery.inc');
          AddInclude('gtktooltips.inc');
          AddInclude('gtktoolbar.inc');
          AddInclude('gtktree.inc');
          AddInclude('gtktreednd.inc');
          AddInclude('gtktreeitem.inc');
          AddInclude('gtktreeselection.inc');
          AddInclude('gtktreestore.inc');
          AddInclude('gtktreeviewcolumn.inc');
          AddInclude('gtkrbtree.inc');
          AddInclude('gtktreeprivate.inc');
          AddInclude('gtktreeview.inc');
          AddInclude('gtkvbbox.inc');
          AddInclude('gtkviewport.inc');
          AddInclude('gtkvpaned.inc');
          AddInclude('gtkvruler.inc');
          AddInclude('gtkvscale.inc');
          AddInclude('gtkvscrollbar.inc');
          AddInclude('gtkvseparator.inc');
          AddInclude('gtkfilefilter.inc');
          AddInclude('gtkfilesystem.inc');
          AddInclude('gtkcellrenderercombo.inc');
          AddInclude('gtkfilechooser.inc');
          AddInclude('gtkfilechooserprivate.inc');
          AddInclude('gtkfilechooserutils.inc');
          AddInclude('gtkfilechooserwidget.inc');
          AddInclude('gtkfilechooserdialog.inc');
          AddInclude('gtkexpander.inc');
          AddInclude('gtkaction.inc');
          AddInclude('gtkactiongroup.inc');
          AddInclude('gtktoggleaction.inc');
          AddInclude('gtkradioaction.inc');
          AddInclude('gtkcombobox.inc');
          AddInclude('gtkcomboboxentry.inc');
          AddInclude('gtktoolitem.inc');
          AddInclude('gtktoolbutton.inc');
          AddInclude('gtktoggletoolbutton.inc');
          AddInclude('gtkradiotoolbutton.inc');
          AddInclude('gtkfontbutton.inc');
          AddInclude('gtkicontheme.inc');
          AddInclude('gtkcolorbutton.inc');
          AddInclude('gtkcelllayout.inc');
          AddInclude('gtkentrycompletion.inc');
          AddInclude('gtkuimanager.inc');
          AddInclude('gtktreemodelfilter.inc');
          AddInclude('gtkincludes.inc');
          AddInclude('gtkobject.inc');
          AddInclude('gtkdebug.inc');
          AddInclude('gtktypeutils.inc');
          AddInclude('gtkwidget.inc');
          AddInclude('gtkmisc.inc');
          AddInclude('gtkaccelgroup.inc');
          AddInclude('gtkcontainer.inc');
          AddInclude('gtkbin.inc');
          AddInclude('gtkwindow.inc');
          AddInclude('gtklabel.inc');
          AddInclude('gtkaccellabel.inc');
          AddInclude('gtkaccelmap.inc');
          AddInclude('gtkaccessible.inc');
          AddInclude('gtkadjustment.inc');
          AddInclude('gtkalignment.inc');
          AddInclude('gtkframe.inc');
          AddInclude('gtkaspectframe.inc');
          AddInclude('gtkarrow.inc');
          AddInclude('gtkbindings.inc');
          AddInclude('gtkbox.inc');
          AddInclude('gtkbbox.inc');
          AddInclude('gtkbutton.inc');
          AddInclude('gtkcalendar.inc');
          AddInclude('gtkcelleditable.inc');
          AddInclude('gtkcellrenderer.inc');
          AddInclude('gtkcellrenderertext.inc');
          AddInclude('gtkcellrenderertoggle.inc');
          AddInclude('gtkcellrendererpixbuf.inc');
          AddInclude('gtkitem.inc');
          AddInclude('gtkmenuitem.inc');
          AddInclude('gtktogglebutton.inc');
          AddInclude('gtkcheckbutton.inc');
          AddInclude('gtkcheckmenuitem.inc');
          AddInclude('gtkclipboard.inc');
          AddInclude('gtkclist.inc');
          AddInclude('gtkdialog.inc');
          AddInclude('gtkvbox.inc');
          AddInclude('gtkcolorsel.inc');
          AddInclude('gtkcolorseldialog.inc');
          AddInclude('gtkhbox.inc');
          AddInclude('gtkcombo.inc');
          AddInclude('gtkctree.inc');
          AddInclude('gtkdrawingarea.inc');
          AddInclude('gtkcurve.inc');
          AddInclude('gtkdnd.inc');
          AddInclude('gtkeditable.inc');
          AddInclude('gtkimcontext.inc');
          AddInclude('gtkmenushell.inc');
          AddInclude('gtkmenu.inc');
          AddInclude('gtkentry.inc');
          AddInclude('gtkenums.inc');
          AddInclude('gtkeventbox.inc');
          AddInclude('fnmatch.inc');
          AddInclude('gtkfilesel.inc');
          AddInclude('gtkfixed.inc');
          AddInclude('gtkfontsel.inc');
          AddInclude('gtkgamma.inc');
          AddInclude('gtkgc.inc');
          AddInclude('gtkhandlebox.inc');
          AddInclude('gtkpaned.inc');
          AddInclude('gtkhbbox.inc');
          AddInclude('gtkhpaned.inc');
          AddInclude('gtkruler.inc');
          AddInclude('gtkhruler.inc');
          AddInclude('gtksettings.inc');
          AddInclude('gtkrc.inc');
          AddInclude('gtkstyle.inc');
          AddInclude('gtkrange.inc');
          AddInclude('gtkscale.inc');
          AddInclude('gtkhscale.inc');
          AddInclude('gtkscrollbar.inc');
          AddInclude('gtkhscrollbar.inc');
          AddInclude('gtkseparator.inc');
          AddInclude('gtkhseparator.inc');
          AddInclude('gtkiconfactory.inc');
          AddInclude('gtkimage.inc');
          AddInclude('gtkimagemenuitem.inc');
          AddInclude('gtkimcontextsimple.inc');
          AddInclude('gtkimmulticontext.inc');
          AddInclude('gtkinputdialog.inc');
          AddInclude('gtkinvisible.inc');
          AddInclude('gtkitemfactory.inc');
          AddInclude('gtklayout.inc');
          AddInclude('gtklist.inc');
          AddInclude('gtklistitem.inc');
          AddInclude('gtktreemodel.inc');
          AddInclude('gtktreesortable.inc');
          AddInclude('gtktreemodelsort.inc');
          AddInclude('gtkliststore.inc');
          AddInclude('gtkmain.inc');
          AddInclude('gtkmenubar.inc');
          AddInclude('gtkmessagedialog.inc');
          AddInclude('gtknotebook.inc');
          AddInclude('gtkoldeditable.inc');
          AddInclude('gtkoptionmenu.inc');
          AddInclude('gtkpixmap.inc');
          AddInclude('gtkplug.inc');
          AddInclude('gtkpreview.inc');
          AddInclude('gtkprogress.inc');
          AddInclude('gtkprogressbar.inc');
          AddInclude('gtkradiobutton.inc');
          AddInclude('gtkradiomenuitem.inc');
          AddInclude('gtkscrolledwindow.inc');
          AddInclude('gtkselection.inc');
          AddInclude('gtkseparatormenuitem.inc');
          AddInclude('gtksignal.inc');
          AddInclude('gtksizegroup.inc');
          AddInclude('gtksocket.inc');
          AddInclude('gtkspinbutton.inc');
          AddInclude('gtkstock.inc');
          AddInclude('gtkstatusbar.inc');
          AddInclude('gtktable.inc');
          AddInclude('gtktearoffmenuitem.inc');
          AddInclude('gtktext.inc');
          AddInclude('gtktextiter.inc');
          AddInclude('gtktexttag.inc');
          AddInclude('gtktexttagtable.inc');
          AddInclude('gtktextmark.inc');
          AddInclude('gtktextmarkprivate.inc');
          AddInclude('gtktextchild.inc');
          AddInclude('gtktextchildprivate.inc');
          AddInclude('gtktextsegment.inc');
          AddInclude('gtktextbtree.inc');
          AddInclude('gtktextbuffer.inc');
          AddInclude('gtktextlayout.inc');
          AddInclude('gtktextview.inc');
          AddInclude('gtktipsquery.inc');
          AddInclude('gtktooltips.inc');
          AddInclude('gtktoolbar.inc');
          AddInclude('gtktree.inc');
          AddInclude('gtktreednd.inc');
          AddInclude('gtktreeitem.inc');
          AddInclude('gtktreeselection.inc');
          AddInclude('gtktreestore.inc');
          AddInclude('gtktreeviewcolumn.inc');
          AddInclude('gtkrbtree.inc');
          AddInclude('gtktreeprivate.inc');
          AddInclude('gtktreeview.inc');
          AddInclude('gtkvbbox.inc');
          AddInclude('gtkviewport.inc');
          AddInclude('gtkvpaned.inc');
          AddInclude('gtkvruler.inc');
          AddInclude('gtkvscale.inc');
          AddInclude('gtkvscrollbar.inc');
          AddInclude('gtkvseparator.inc');
          AddInclude('gtkfilefilter.inc');
          AddInclude('gtkfilesystem.inc');
          AddInclude('gtkcellrenderercombo.inc');
          AddInclude('gtkfilechooser.inc');
          AddInclude('gtkfilechooserprivate.inc');
          AddInclude('gtkfilechooserutils.inc');
          AddInclude('gtkfilechooserwidget.inc');
          AddInclude('gtkfilechooserdialog.inc');
          AddInclude('gtkexpander.inc');
          AddInclude('gtkaction.inc');
          AddInclude('gtkactiongroup.inc');
          AddInclude('gtktoggleaction.inc');
          AddInclude('gtkradioaction.inc');
          AddInclude('gtkcombobox.inc');
          AddInclude('gtkcomboboxentry.inc');
          AddInclude('gtktoolitem.inc');
          AddInclude('gtktoolbutton.inc');
          AddInclude('gtktoggletoolbutton.inc');
          AddInclude('gtkradiotoolbutton.inc');
          AddInclude('gtkfontbutton.inc');
          AddInclude('gtkicontheme.inc');
          AddInclude('gtkcolorbutton.inc');
          AddInclude('gtkcelllayout.inc');
          AddInclude('gtkentrycompletion.inc');
          AddInclude('gtkuimanager.inc');
          AddInclude('gtktreemodelfilter.inc');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

          AddInclude('gtkseparatortoolitem.inc');
          AddInclude('gtkaboutdialog.inc');
          AddInclude('gtkcellrendererprogress.inc');
          AddInclude('gtkfilechooserbutton.inc');
          AddInclude('gtkcellview.inc');
          AddInclude('gtkiconview.inc');
          AddInclude('gtkmenutoolbutton.inc');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        end;
    T:=P.Targets.AddImplicitUnit('src/gtkglext/gtkglext.pas');
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
          AddUnit('atk');
          AddUnit('pango');
          AddUnit('gdk2pixbuf');
          AddUnit('gdk2');
        end;
    T:=P.Targets.AddUnit('src/gtkglext/gtkglext.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtkglext');
      with T.Dependencies do
        begin
          AddInclude('gtkglext_includes.inc');
          AddInclude('gtkgldefs.inc');
          AddInclude('gtkglversion.inc');
          AddInclude('gtkglinit.inc');
          AddInclude('gtkglwidget.inc');
          AddInclude('gtkglext_includes.inc');
          AddInclude('gtkgldefs.inc');
          AddInclude('gtkglversion.inc');
          AddInclude('gtkglinit.inc');
          AddInclude('gtkglwidget.inc');
          AddInclude('gtkglext_includes.inc');
          AddInclude('gtkgldefs.inc');
          AddInclude('gtkglversion.inc');
          AddInclude('gtkglinit.inc');
          AddInclude('gtkglwidget.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddImplicitUnit('src/libglade/libglade2.pas');
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
          AddUnit('gdk2');
          AddUnit('gtk2');
          AddUnit('gdkglext');
        end;
    T:=P.Targets.AddUnit('src/libglade/libglade2.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/libglade');
      with T.Dependencies do
        begin
          AddInclude('glade-init.inc');
          AddInclude('glade-xml.inc');
<<<<<<< HEAD
<<<<<<< HEAD
        end;
    T:=P.Targets.AddImplicitUnit('src/pango/pango.pas');
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
          AddUnit('gtk2');
        end;
    T:=P.Targets.AddUnit('src/pango/pango.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/pango');
      with T.Dependencies do
        begin
          AddInclude('pangoincludes.inc');
          AddInclude('pango-types.inc');
          AddInclude('pango-attributes.inc');
          AddInclude('pango-break.inc');
          AddInclude('pango-context.inc');
          AddInclude('pango-coverage.inc');
          AddInclude('pango-engine.inc');
          AddInclude('pango-fontset.inc');
          AddInclude('pango-font.inc');
          AddInclude('pango-fontmap.inc');
          AddInclude('pango-glyph.inc');
          AddInclude('pango-item.inc');
          AddInclude('pango-layout.inc');
          AddInclude('pango-tabs.inc');
          AddInclude('pangoincludes.inc');
          AddInclude('pango-types.inc');
          AddInclude('pango-attributes.inc');
          AddInclude('pango-break.inc');
          AddInclude('pango-context.inc');
          AddInclude('pango-coverage.inc');
          AddInclude('pango-engine.inc');
          AddInclude('pango-fontset.inc');
          AddInclude('pango-font.inc');
          AddInclude('pango-fontmap.inc');
          AddInclude('pango-glyph.inc');
          AddInclude('pango-item.inc');
          AddInclude('pango-layout.inc');
          AddInclude('pango-tabs.inc');
          AddInclude('pangoincludes.inc');
          AddInclude('pango-types.inc');
          AddInclude('pango-attributes.inc');
          AddInclude('pango-break.inc');
          AddInclude('pango-context.inc');
          AddInclude('pango-coverage.inc');
          AddInclude('pango-engine.inc');
          AddInclude('pango-fontset.inc');
          AddInclude('pango-font.inc');
          AddInclude('pango-fontmap.inc');
          AddInclude('pango-glyph.inc');
          AddInclude('pango-item.inc');
          AddInclude('pango-layout.inc');
          AddInclude('pango-tabs.inc');
          AddInclude('pangoincludes.inc');
          AddInclude('pango-types.inc');
          AddInclude('pango-attributes.inc');
          AddInclude('pango-break.inc');
          AddInclude('pango-context.inc');
          AddInclude('pango-coverage.inc');
          AddInclude('pango-engine.inc');
          AddInclude('pango-fontset.inc');
          AddInclude('pango-font.inc');
          AddInclude('pango-fontmap.inc');
          AddInclude('pango-glyph.inc');
          AddInclude('pango-item.inc');
          AddInclude('pango-layout.inc');
          AddInclude('pango-tabs.inc');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('pango-matrix.inc');
          AddInclude('pango-renderer.inc');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        end;
    
    T:=P.Targets.AddImplicitUnit('src/pangocairo/pangocairo.pas');
      T.IncludePath.Add('src/pangocairo');

    T:=P.Targets.AddImplicitUnit('src/gtkext/gtk2ext.pp');
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('glib2');
        end;

    T:=P.Targets.AddUnit('src/gtkext/gtk2ext.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      T.IncludePath.Add('src/gtkext');
      with T.Dependencies do
        begin
          AddInclude('gtkstatusiconh.inc');
          AddInclude('gtkstatusicon.inc');
	  AddInclude('gtkscalebuttonh.inc');
	  AddInclude('gtkscalebutton.inc');
	  AddInclude('gtkvolumebuttonh.inc');	  
	  AddInclude('gtkvolumebutton.inc');	  
	  AddInclude('gtktextmarkh.inc');
	  AddInclude('gtktextmark.inc');
	  AddInclude('gtktextiterh.inc');
	  AddInclude('gtktextiter.inc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
// For some reson this isn't build in the buildunit nor the Makefile.fpc
{     T:=P.Targets.AddUnit('src/gtkhtml/gtkhtml.pas');
=======

     T:=P.Targets.AddUnit('src/gtkhtml/gtkhtml.pas');
>>>>>>> graemeg/fixes_2_2
=======

     T:=P.Targets.AddUnit('src/gtkhtml/gtkhtml.pas');
>>>>>>> origin/fixes_2_2
       T.IncludePath.Add('src/gtkhtml');
       with T.Dependencies do
         begin
           AddInclude('gtkhtmlincludes.inc');
           AddInclude('htmlstream.inc');
           AddInclude('htmlstreambuffer.inc');
           AddInclude('htmldocument.inc');
           AddInclude('htmlview.inc');
           AddInclude('gtkhtmlincludes.inc');
           AddInclude('htmlstream.inc');
           AddInclude('htmlstreambuffer.inc');
           AddInclude('htmldocument.inc');
           AddInclude('htmlview.inc');
           AddInclude('gtkhtmlincludes.inc');
           AddInclude('htmlstream.inc');
           AddInclude('htmlstreambuffer.inc');
           AddInclude('htmldocument.inc');
           AddInclude('htmlview.inc');
           AddInclude('gtkhtmlincludes.inc');
           AddInclude('htmlstream.inc');
           AddInclude('htmlstreambuffer.inc');
           AddInclude('htmldocument.inc');
           AddInclude('htmlview.inc');
<<<<<<< HEAD
<<<<<<< HEAD
         end;}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Sources.AddExampleFiles('examples/*',P.Directory,false,'.');
    P.Sources.AddExampleFiles('examples/filechooser/*',P.Directory,false,'filechooser');
    P.Sources.AddExampleFiles('examples/gettingstarted/*',P.Directory,false,'gettingstarted');
    P.Sources.AddExampleFiles('examples/gtk_demo/*',P.Directory,false,'gtk_demo');
    P.Sources.AddExampleFiles('examples/gtkglext/*',P.Directory,false,'gtkglext');
    P.Sources.AddExampleFiles('examples/helloworld/*',P.Directory,false,'helloworld');
    P.Sources.AddExampleFiles('examples/helloworld2/*',P.Directory,false,'helloworld2');
    P.Sources.AddExampleFiles('examples/plugins/*',P.Directory,false,'plugins');
    P.Sources.AddExampleFiles('examples/scribble_simple/*',P.Directory,false,'scribble_simple');


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
           AddUnit('gtk2');
           AddUnit('glib2');
           AddUnit('atk');
           AddUnit('pango');
           AddUnit('gdk2pixbuf');
           AddUnit('gdk2');
         end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
