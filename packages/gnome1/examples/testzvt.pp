{

   TestZVT - An FPC Example Program demonstrating the most common use
             of ZVTTerm in a GNOME application.

   Copyright (C) 2002 Andrew Johnson <aj_genius@hotmail.com>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.

 **********************************************************************}
Program TestZVT;

(* Try to Execute mc (midnight commander) instead of sh *)
{$Define exec_mc}

Uses
  SysUtils,

  { Linux/UNIX Unit, for execvp }
  {$IfDef ver1_0}linux{$Else}Unix{$EndIF},

  { Standard GTK+ 1.x Interface }
  glib, gdk, gtk,

  { Standard GNOME 1.x Interface }
  libgnome, libgnomeui,

  { Standard libzvt 1.x Interface }
  libzvt;

const
  (* what to execvp in terminal widget *)
  {$Ifdef exec_mc}
    Command : PChar = 'mc';
    Params : array[0..1] of PChar = ('TERM=xterm', nil);
  {$else}
    Command : PChar = 'sh';
    Params : array[0..0] of PChar = (nil);
  {$EndIf}
  Terminals : Longint = 0;//# of terminals currently open

  (* Program Information for GNOME & About Box *)
  ProgramName : PChar = 'TestZVT';
  ProgramVersion : PChar = '1.0';

  (* Information for About Box *)
  Copyright : PChar = 'Copyright (C) 2002 Andrew Johnson';
  Authors : array[0..1] of PChar = ('Andrew Johnson <aj_genius@hotmail.com>', nil);
  Comments : PChar = 'An FPC Example Program demonstrating the most common use of ZVTTerm in a GNOME application.';

var
  app, mdichild : pointer;

Procedure quit_testzvt(Widget : PGTKWidget; Data : Pointer); cdecl;
begin
  (* Quite Main Loop *)
  gtk_main_quit;
end;

Procedure exit_terminal(Widget : PGTKWidget; Data : Pointer); cdecl;
begin
  (* Destroy terminal on process exit, and quit if only terminal open *)
  gnome_mdi_remove_view(App, Data, 1);
  Dec(Terminals);
end;

Procedure close_activechild(Widget : PGTKWidget; Data : Pointer); cdecl;
begin
  (* close active view *)
  exit_terminal(Widget, gnome_mdi_get_active_view(App));
end;

Procedure new_child(Widget : PGTKWidget; Data : Pointer); cdecl;
begin
  (* create new view& set active *)
  gnome_mdi_add_view(app, mdichild);
end;

Procedure about_testzvt(Widget : PGTKWidget; Data : Pointer); cdecl;
var
  AboutBox : Pointer;
begin
  (* Create and Run an About Box *)
  AboutBox := gnome_about_new(gnome_app_id, ProgramVersion, Copyright,
                                @Authors[0],Comments,nil);
  gnome_dialog_set_parent(AboutBox, GTK_Window(gnome_mdi_get_active_window(App)));
  gnome_dialog_run_and_close(AboutBox);
end;

Procedure show_terminal(Widget : PGTKWidget; Data : Pointer); cdecl;
begin
  (* fork terminal process, and Exec Command *)
  If zvt_term_forkpty(ZVT_TERM(Widget), ZVT_TERM_DO_UTMP_LOG or ZVT_TERM_DO_WTMP_LOG or ZVT_TERM_DO_LASTLOG) = 0 then
    execvp (Command, @Command, @Params[0]);

  (* close app when fork'ed terminal process finishes/dies *)
  gtk_signal_connect (GTK_OBJECT(Widget), 'child_died', GTK_SIGNAL_FUNC (@exit_terminal), Data);
end;

Function NewTerminalView: PGTKWidget; cdecl;
var
  hBox, SB, Term : gPointer;
begin
  (* Create hbox for layout of Terminal/Scrollbar *)
  hBox := gtk_hbox_new(FALSE, 0);

  term := zvt_term_new_with_size(80,30);//start with average size

  (* Set up terminal options *)
  zvt_term_set_shadow_type(term, GTK_SHADOW_IN);//give the terminal a small indented frame
  zvt_term_set_font_name(term, '-misc-fixed-medium-r-normal-*-12-200-*-*-c-75-*-*');
  zvt_term_set_scrollback(term, 10000);//give a decent amount of scrollback
  zvt_term_set_scroll_on_keystroke(term, True);//default on most terminals
  zvt_term_set_scroll_on_output(term, False);//default on most terminals
  zvt_term_set_background(ZVT_TERM (term), nil, False, 0);//ensure is not transparent

  gtk_signal_connect_after(term, 'show', GTK_SIGNAL_FUNC (@show_terminal), hBox);
  (* Create scrollbar *)
  sb := gtk_vscrollbar_new(GTK_ADJUSTMENT (ZVT_TERM(term)^.adjustment));

  GTK_WIDGET_UNSET_FLAGS(sb, GTK_CAN_FOCUS);//Should never capture keyboard

  (* Pack Box *)
  gtk_box_pack_start(hBox, term, TRUE, TRUE, 0);
  gtk_box_pack_start(hBox, sb, FALSE, TRUE, 0);
  gtk_object_set_data(hbox, 'caption', Pchar('Terminal #' + IntToStr(Terminals)));
  gtk_widget_show_all(hBox);
  NewTerminalView := hBox;
  Inc(Terminals);
end;

Function CreateMDIChildWidget : Pointer;
var
  child : Pointer;
begin
  child := gnome_mdi_generic_child_new('Terminal');
  gnome_mdi_generic_child_set_view_creator(child, @NewTerminalView, nil);
  CreateMDIChildWidget := child;
end;

var
  file_menu : array[0..4] of TGnomeUIInfo;
  help_menu : array[0..1] of TGnomeUIInfo;
  Menus : array[0..2] of TGnomeUIInfo;
begin
  (* Initialize GNOME with Current Program Name and Version *)
  gnome_init(ProgramName, ProgramVersion, argc, argv);

  (* Create Main App *)
  app := gnome_mdi_new(gnome_app_id, 'FPC GNOME ZVT Test');
  gtk_signal_connect(app, 'destroy', GTK_SIGNAL_FUNC (@quit_testzvt), nil);

  (* Create Stock Menus *)
  file_menu[0] := GNOMEUIINFO_MENU_NEW_ITEM('New Shell Process', 'Opens a new shell process', @new_child, nil);
  file_menu[1] := GNOMEUIINFO_MENU_CLOSE_ITEM(@close_activechild,nil);
  file_menu[2] := GNOMEUIINFO_SEPARATOR;
  file_menu[3] := GNOMEUIINFO_MENU_EXIT_ITEM(@quit_testzvt,nil);
  file_menu[4] := GNOMEUIINFO_END;

  help_menu[0] := GNOMEUIINFO_MENU_ABOUT_ITEM(@about_testzvt, app);
  help_menu[1] := GNOMEUIINFO_END;

  menus[0] := GNOMEUIINFO_MENU_FILE_TREE(@file_menu[0]);
  menus[1] := GNOMEUIINFO_MENU_HELP_TREE(@help_menu[0]);
  menus[2] := GNOMEUIINFO_END;

  mdichild := CreateMDIChildWidget;

  (* Set App Menu/Contents, and Show All *)
  gnome_mdi_set_mode(App, GNOME_MDI_NOTEBOOK);
  gnome_mdi_set_menubar_template(App, @Menus[0]);
  gnome_mdi_open_toplevel(app);
  gnome_mdi_add_child(app, mdichild);
  gnome_mdi_add_view(app, mdichild);
  gnome_mdi_add_view(app, mdichild);

  (* Run Main Loop *)
  gtk_main();

  (* cleanup and exit *)
  gtk_exit(0);
end.
