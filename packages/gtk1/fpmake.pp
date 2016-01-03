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

    P:=AddPackage('gtk1');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Library: Peter Mattis, Spencer Kimball and Josh MacDonald, header: Peter Vreman';
    P.License := 'Library: LGPL2.1, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Header to the GTK widgetset (v1).';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?

    P.OSes:=AllUnixOSes+[Win32,Win64,OS2,EMX]-[darwin,iphonesim,Android];
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];

    P.Dependencies.Add('opengl',AllUnixOSes+[Win32,Win64]);
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='gtk1';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=AllUnixOSes+[Win32,Win64];
    P.Dependencies.Add('opengl');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('src/gdk/gdkpixbuf.pp');
      with T.Dependencies do
        begin
          AddUnit('glib');
          AddUnit('gdk');
          AddUnit('gtk');
        end;
    T:=P.Targets.AddUnit('src/gdk/gdk.pp');
      T.IncludePath.Add('src/gdk');
      with T.Dependencies do
        begin
          AddInclude('gdktypes.pp');
          AddInclude('gdkkeysyms.pp');
          AddInclude('gdkprivate.pp');
          AddInclude('gdkrgb.pp');
          AddInclude('gdkx.pp');
          AddInclude('gdkmain.pp');
          AddInclude('gdktypes.pp');
          AddInclude('gdkkeysyms.pp');
          AddInclude('gdkprivate.pp');
          AddInclude('gdkrgb.pp');
          AddInclude('gdkx.pp');
          AddInclude('gdkmain.pp');
          AddUnit('glib');
        end;
    T:=P.Targets.AddUnit('src/glib/glib.pp');
      T.IncludePath.Add('src/glib');
    T:=P.Targets.AddUnit('src/glib/gmodule.pp');
      with T.Dependencies do
        begin
          AddUnit('glib');
        end;
    T:=P.Targets.AddUnit('src/gtkgl/gtkglarea.pp');
      T.IncludePath.Add('src/gtkgl');
      with T.Dependencies do
        begin
          AddUnit('gdk');
          AddUnit('gtk');
        end;
    T:=P.Targets.AddUnit('src/gtk/gtk.pp');
      T.IncludePath.Add('src/gtk');
      with T.Dependencies do
        begin
          AddInclude('gtkincludes.pp');
          AddInclude('gtkfeatures.pp');
          AddInclude('gtkenums.pp');
          AddInclude('gtkobjects.pp');
          AddInclude('gtktypeutils.pp');
          AddInclude('gtkdata.pp');
          AddInclude('gtkadjustment.pp');
          AddInclude('gtkaccelgroup.pp');
          AddInclude('gtkwidget.pp');
          AddInclude('gtkstyle.pp');
          AddInclude('gtkmisc.pp');
          AddInclude('gtklabel.pp');
          AddInclude('gtkprivate.pp');
          AddInclude('gtkaccellabel.pp');
          AddInclude('gtkthemes.pp');
          AddInclude('gtkmain.pp');
          AddInclude('gtkdrawingarea.pp');
          AddInclude('gtkcontainer.pp');
          AddInclude('gtkbin.pp');
          AddInclude('gtksignal.pp');
          AddInclude('gtkitem.pp');
          AddInclude('gtkarg.pp');
          AddInclude('gtkmarshal.pp');
          AddInclude('gtkbindings.pp');
          AddInclude('gtkinvisible.pp');
          AddInclude('gtkalignment.pp');
          AddInclude('gtkwindow.pp');
          AddInclude('gtkplug.pp');
          AddInclude('gtksocket.pp');
          AddInclude('gtkpacker.pp');
          AddInclude('gtklayout.pp');
          AddInclude('gtkframe.pp');
          AddInclude('gtkaspectframe.pp');
          AddInclude('gtkviewport.pp');
          AddInclude('gtkrange.pp');
          AddInclude('gtkrc.pp');
          AddInclude('gtkeditable.pp');
          AddInclude('gtktext.pp');
          AddInclude('gtkarrow.pp');
          AddInclude('gtkpixmap.pp');
          AddInclude('gtkbutton.pp');
          AddInclude('gtktogglebutton.pp');
          AddInclude('gtkcheckbutton.pp');
          AddInclude('gtkmenuitem.pp');
          AddInclude('gtkcheckmenuitem.pp');
          AddInclude('gtktearoffmenuitem.pp');
          AddInclude('gtkcurve.pp');
          AddInclude('gtkdialog.pp');
          AddInclude('gtkentry.pp');
          AddInclude('gtkeventbox.pp');
          AddInclude('gtkfilesel.pp');
          AddInclude('gtkfixed.pp');
          AddInclude('gtkgc.pp');
          AddInclude('gtkhandlebox.pp');
          AddInclude('gtkimage.pp');
          AddInclude('gtkinputdialog.pp');
          AddInclude('gtklist.pp');
          AddInclude('gtklistitem.pp');
          AddInclude('gtkmenushell.pp');
          AddInclude('gtkmenufactory.pp');
          AddInclude('gtkmenu.pp');
          AddInclude('gtkmenubar.pp');
          AddInclude('gtkoptionmenu.pp');
          AddInclude('gtkpreview.pp');
          AddInclude('gtkitemfactory.pp');
          AddInclude('gtkprogress.pp');
          AddInclude('gtkprogressbar.pp');
          AddInclude('gtkradiobutton.pp');
          AddInclude('gtkradiomenuitem.pp');
          AddInclude('gtkscrolledwindow.pp');
          AddInclude('gtkselection.pp');
          AddInclude('gtkdnd.pp');
          AddInclude('gtkspinbutton.pp');
          AddInclude('gtktable.pp');
          AddInclude('gtktipsquery.pp');
          AddInclude('gtktooltips.pp');
          AddInclude('gtktoolbar.pp');
          AddInclude('gtktreeitem.pp');
          AddInclude('gtktree.pp');
          AddInclude('gtkbox.pp');
          AddInclude('gtkhbox.pp');
          AddInclude('gtkvbox.pp');
          AddInclude('gtkbbox.pp');
          AddInclude('gtkhbbox.pp');
          AddInclude('gtkvbbox.pp');
          AddInclude('gtkpaned.pp');
          AddInclude('gtkhpaned.pp');
          AddInclude('gtkvpaned.pp');
          AddInclude('gtkruler.pp');
          AddInclude('gtkhruler.pp');
          AddInclude('gtkvruler.pp');
          AddInclude('gtkscale.pp');
          AddInclude('gtkhscale.pp');
          AddInclude('gtkvscale.pp');
          AddInclude('gtkscrollbar.pp');
          AddInclude('gtkhscrollbar.pp');
          AddInclude('gtkvscrollbar.pp');
          AddInclude('gtkseparator.pp');
          AddInclude('gtkhseparator.pp');
          AddInclude('gtkvseparator.pp');
          AddInclude('gtkcombo.pp');
          AddInclude('gtkstatusbar.pp');
          AddInclude('gtkcolorsel.pp');
          AddInclude('gtkgamma.pp');
          AddInclude('gtkclist.pp');
          AddInclude('gtknotebook.pp');
          AddInclude('gtkctree.pp');
          AddInclude('gtkcalendar.pp');
          AddInclude('gtkfontsel.pp');
          AddInclude('gtkincludes.pp');
          AddInclude('gtkfeatures.pp');
          AddInclude('gtkenums.pp');
          AddInclude('gtkobjects.pp');
          AddInclude('gtktypeutils.pp');
          AddInclude('gtkdata.pp');
          AddInclude('gtkadjustment.pp');
          AddInclude('gtkaccelgroup.pp');
          AddInclude('gtkwidget.pp');
          AddInclude('gtkstyle.pp');
          AddInclude('gtkmisc.pp');
          AddInclude('gtklabel.pp');
          AddInclude('gtkprivate.pp');
          AddInclude('gtkaccellabel.pp');
          AddInclude('gtkthemes.pp');
          AddInclude('gtkmain.pp');
          AddInclude('gtkdrawingarea.pp');
          AddInclude('gtkcontainer.pp');
          AddInclude('gtkbin.pp');
          AddInclude('gtksignal.pp');
          AddInclude('gtkitem.pp');
          AddInclude('gtkarg.pp');
          AddInclude('gtkmarshal.pp');
          AddInclude('gtkbindings.pp');
          AddInclude('gtkinvisible.pp');
          AddInclude('gtkalignment.pp');
          AddInclude('gtkwindow.pp');
          AddInclude('gtkplug.pp');
          AddInclude('gtksocket.pp');
          AddInclude('gtkpacker.pp');
          AddInclude('gtklayout.pp');
          AddInclude('gtkframe.pp');
          AddInclude('gtkaspectframe.pp');
          AddInclude('gtkviewport.pp');
          AddInclude('gtkrange.pp');
          AddInclude('gtkrc.pp');
          AddInclude('gtkeditable.pp');
          AddInclude('gtktext.pp');
          AddInclude('gtkarrow.pp');
          AddInclude('gtkpixmap.pp');
          AddInclude('gtkbutton.pp');
          AddInclude('gtktogglebutton.pp');
          AddInclude('gtkcheckbutton.pp');
          AddInclude('gtkmenuitem.pp');
          AddInclude('gtkcheckmenuitem.pp');
          AddInclude('gtktearoffmenuitem.pp');
          AddInclude('gtkcurve.pp');
          AddInclude('gtkdialog.pp');
          AddInclude('gtkentry.pp');
          AddInclude('gtkeventbox.pp');
          AddInclude('gtkfilesel.pp');
          AddInclude('gtkfixed.pp');
          AddInclude('gtkgc.pp');
          AddInclude('gtkhandlebox.pp');
          AddInclude('gtkimage.pp');
          AddInclude('gtkinputdialog.pp');
          AddInclude('gtklist.pp');
          AddInclude('gtklistitem.pp');
          AddInclude('gtkmenushell.pp');
          AddInclude('gtkmenufactory.pp');
          AddInclude('gtkmenu.pp');
          AddInclude('gtkmenubar.pp');
          AddInclude('gtkoptionmenu.pp');
          AddInclude('gtkpreview.pp');
          AddInclude('gtkitemfactory.pp');
          AddInclude('gtkprogress.pp');
          AddInclude('gtkprogressbar.pp');
          AddInclude('gtkradiobutton.pp');
          AddInclude('gtkradiomenuitem.pp');
          AddInclude('gtkscrolledwindow.pp');
          AddInclude('gtkselection.pp');
          AddInclude('gtkdnd.pp');
          AddInclude('gtkspinbutton.pp');
          AddInclude('gtktable.pp');
          AddInclude('gtktipsquery.pp');
          AddInclude('gtktooltips.pp');
          AddInclude('gtktoolbar.pp');
          AddInclude('gtktreeitem.pp');
          AddInclude('gtktree.pp');
          AddInclude('gtkbox.pp');
          AddInclude('gtkhbox.pp');
          AddInclude('gtkvbox.pp');
          AddInclude('gtkbbox.pp');
          AddInclude('gtkhbbox.pp');
          AddInclude('gtkvbbox.pp');
          AddInclude('gtkpaned.pp');
          AddInclude('gtkhpaned.pp');
          AddInclude('gtkvpaned.pp');
          AddInclude('gtkruler.pp');
          AddInclude('gtkhruler.pp');
          AddInclude('gtkvruler.pp');
          AddInclude('gtkscale.pp');
          AddInclude('gtkhscale.pp');
          AddInclude('gtkvscale.pp');
          AddInclude('gtkscrollbar.pp');
          AddInclude('gtkhscrollbar.pp');
          AddInclude('gtkvscrollbar.pp');
          AddInclude('gtkseparator.pp');
          AddInclude('gtkhseparator.pp');
          AddInclude('gtkvseparator.pp');
          AddInclude('gtkcombo.pp');
          AddInclude('gtkstatusbar.pp');
          AddInclude('gtkcolorsel.pp');
          AddInclude('gtkgamma.pp');
          AddInclude('gtkclist.pp');
          AddInclude('gtknotebook.pp');
          AddInclude('gtkctree.pp');
          AddInclude('gtkcalendar.pp');
          AddInclude('gtkfontsel.pp');
          AddUnit('glib');
          AddUnit('gdk');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('entry.pp');
    P.Targets.AddExampleProgram('filesel.pp');
    P.Targets.AddExampleProgram('spinbutton.pp');
    P.Targets.AddExampleProgram('gtkgldemo.pp');
    P.Targets.AddExampleProgram('editform.pp');
    P.Targets.AddExampleProgram('pixmap.pp');
    P.Targets.AddExampleProgram('scribble.pp');
    P.Targets.AddExampleProgram('tictactoe.pp');
    P.Targets.AddExampleProgram('paned.pp');
    P.Targets.AddExampleProgram('notebook.pp');
    P.Targets.AddExampleProgram('statusbar.pp');
    P.Targets.AddExampleProgram('ttt_test.pp');
    P.Targets.AddExampleProgram('list.pp');
    P.Targets.AddExampleProgram('clist.pp');
    P.Targets.AddExampleProgram('toolbar.pp');
    P.Targets.AddExampleProgram('progressbar.pp');
    P.Targets.AddExampleProgram('rulers.pp');
    // 'examples/Makefile
    // 'examples/Makefile.fpc
    P.ExamplePath.Add('examples/tutorial');
    P.Targets.AddExampleProgram('tut6_3.pp');
    P.Targets.AddExampleProgram('tut6_4.pp');
    P.Targets.AddExampleProgram('tut4_3.pp');
    P.Targets.AddExampleProgram('tut6_2.pp');
    P.Targets.AddExampleProgram('tut6_1.pp');
    P.Targets.AddExampleProgram('tut2_1.pp');
    P.Targets.AddExampleProgram('tut4_5.pp');
    P.Targets.AddExampleProgram('tut8_5.pp');
    P.Targets.AddExampleProgram('tut3_3.pp');
    // 'examples/tutorial/Makefile
    // 'examples/tutorial/Makefile.fpc
    // 'examples/tutorial/info.xpm
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
