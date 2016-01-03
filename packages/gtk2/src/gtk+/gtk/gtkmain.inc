// included by gtk2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
{ GTK_DISABLE_DEPRECATED  }
   TGtkModuleInitFunc = procedure (argc:Pgint; argv:PPPgchar); cdecl;

   TGtkModuleDisplayInitFunc = procedure (display: PGdkDisplay); cdecl;

   TGtkKeySnoopFunc = function (grab_widget:PGtkWidget; event:PGdkEventKey; func_data:gpointer):gint; cdecl;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
{ Priorities for redrawing and resizing }

const
   GTK_PRIORITY_RESIZE = G_PRIORITY_HIGH_IDLE + 10;

{$ifndef GTK_DISABLE_DEPRECATED}
{ Use GDK_PRIORITY_REDRAW  }
const
   GTK_PRIORITY_REDRAW = G_PRIORITY_HIGH_IDLE + 20;
{ Deprecated. Use G_PRIORITY #define's instead
  }
   GTK_PRIORITY_HIGH = G_PRIORITY_HIGH;
   GTK_PRIORITY_INTERNAL = GTK_PRIORITY_REDRAW;
   GTK_PRIORITY_DEFAULT = G_PRIORITY_DEFAULT_IDLE;
   GTK_PRIORITY_LOW = G_PRIORITY_LOW;
{$endif}

{$IFNDEF KYLIX}
{ Gtk version.  }
var
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF WINDOWS}
=======
  {$IFDEF WIN32}
>>>>>>> graemeg/fixes_2_2
=======
  {$IFDEF WIN32}
>>>>>>> origin/fixes_2_2
   gtk_major_version : guint;external gtklib name 'gtk_major_version';
   gtk_minor_version : guint;external gtklib name 'gtk_minor_version';
   gtk_micro_version : guint;external gtklib name 'gtk_micro_version';
   gtk_binary_age    : guint;external gtklib name 'gtk_binary_age';
   gtk_interface_age : guint;external gtklib name 'gtk_interface_type';
  {$ELSE}
   gtk_major_version : guint;cvar;external;
   gtk_minor_version : guint;cvar;external;
   gtk_micro_version : guint;cvar;external;
   gtk_binary_age    : guint;cvar;external;
   gtk_interface_age : guint;cvar;external;
  {$ENDIF}
{$ENDIF}

function gtk_check_version(required_major:guint; required_minor:guint; required_micro:guint):Pgchar; cdecl; external gtklib;

{$ifndef G_PLATFORM_WIN32}
{ Initialization, exit, mainloop and miscellaneous routines }
procedure gtk_init(argc:Plongint; argv:PPPchar); cdecl; external gtklib;
function gtk_init_check(argc:Plongint; argv:PPPchar):gboolean; cdecl; external gtklib;
{$else}
{ Variants that are used to check for correct struct packing
   when building GTK+-using code. }
procedure gtk_init_abi_check(argc:Plongint; argv:PPPchar; num_checks:longint; sizeof_GtkWindow:Tsize_t; sizeof_GtkBox:Tsize_t); cdecl; external gtklib;
function gtk_init_check_abi_check(argc:Plongint; argv:PPPchar; num_checks:longint; sizeof_GtkWindow:Tsize_t; sizeof_GtkBox:Tsize_t):gboolean; cdecl; external gtklib;
procedure gtk_init(argc: Plongint; argv : PPPchar);
function gtk_init_check(argc: Plongint; argv : PPPchar) : gboolean;
{$endif G_PLATFORM_WIN32}


{$ifndef GTK_DISABLE_DEPRECATED}
procedure gtk_exit(error_code:gint); cdecl; external gtklib;
{$endif}
{ GTK_DISABLE_DEPRECATED  }

procedure gtk_disable_setlocale; cdecl; external gtklib;
function gtk_set_locale:Pgchar; cdecl; external gtklib;
function gtk_get_default_language:PPangoLanguage; cdecl; external gtklib;
function gtk_events_pending:gint; cdecl; external gtklib;
{ The following is the event func GTK+ registers with GDK
   we expose it mainly to allow filtering of events between
   GDK and GTK+.
  }
procedure gtk_main_do_event(event:PGdkEvent); cdecl; external gtklib;
procedure gtk_main; cdecl; external gtklib;
function gtk_main_level:guint; cdecl; external gtklib;
procedure gtk_main_quit; cdecl; external gtklib;
function gtk_main_iteration:gboolean; cdecl; external gtklib;
{ gtk_main_iteration() calls gtk_main_iteration_do(TRUE)  }
function gtk_main_iteration_do(blocking:gboolean):gboolean; cdecl; external gtklib;
function gtk_true:gboolean; cdecl; external gtklib;
function gtk_false:gboolean; cdecl; external gtklib;
procedure gtk_grab_add(widget:PGtkWidget); cdecl; external gtklib;
function gtk_grab_get_current:PGtkWidget; cdecl; external gtklib;
procedure gtk_grab_remove(widget:PGtkWidget); cdecl; external gtklib;
procedure gtk_init_add(_function:TGtkFunction; data:gpointer); cdecl; external gtklib;
procedure gtk_quit_add_destroy(main_level:guint; anObject:PGtkObject); cdecl; external gtklib;
function gtk_quit_add(main_level:guint; _function:TGtkFunction; data:gpointer):guint; cdecl; external gtklib;
function gtk_quit_add_full(main_level:guint; _function:TGtkFunction; marshal:TGtkCallbackMarshal; data:gpointer; destroy:TGtkDestroyNotify):guint; cdecl; external gtklib;
procedure gtk_quit_remove(quit_handler_id:guint); cdecl; external gtklib;
procedure gtk_quit_remove_by_data(data:gpointer); cdecl; external gtklib;
function gtk_timeout_add(interval:guint32; _function:TGtkFunction; data:gpointer):guint; cdecl; external gtklib;
function gtk_timeout_add_full(interval:guint32; _function:TGtkFunction; marshal:TGtkCallbackMarshal; data:gpointer; destroy:TGtkDestroyNotify):guint; cdecl; external gtklib;
procedure gtk_timeout_remove(timeout_handler_id:guint); cdecl; external gtklib;
function gtk_idle_add(_function:TGtkFunction; data:gpointer):guint; cdecl; external gtklib;
function gtk_idle_add_priority(priority:gint; _function:TGtkFunction; data:gpointer):guint; cdecl; external gtklib;
function gtk_idle_add_full(priority:gint; _function:TGtkFunction; marshal:TGtkCallbackMarshal; data:gpointer; destroy:TGtkDestroyNotify):guint; cdecl; external gtklib;
procedure gtk_idle_remove(idle_handler_id:guint); cdecl; external gtklib;
procedure gtk_idle_remove_by_data(data:gpointer); cdecl; external gtklib;
function gtk_input_add_full(source:gint; condition:TGdkInputCondition; _function:TGdkInputFunction; marshal:TGtkCallbackMarshal; data:gpointer;
           destroy:TGtkDestroyNotify):guint; cdecl; external gtklib;
procedure gtk_input_remove(input_handler_id:guint); cdecl; external gtklib;
function gtk_key_snooper_install(snooper:TGtkKeySnoopFunc; func_data:gpointer):guint; cdecl; external gtklib;
procedure gtk_key_snooper_remove(snooper_handler_id:guint); cdecl; external gtklib;
function gtk_get_current_event:PGdkEvent; cdecl; external gtklib;
function gtk_get_current_event_time:guint32; cdecl; external gtklib;
function gtk_get_current_event_state(state:PGdkModifierType):gboolean; cdecl; external gtklib;
function gtk_get_event_widget(event:PGdkEvent):PGtkWidget; cdecl; external gtklib;

{ Private routines internal to GTK+ }
procedure gtk_propagate_event(widget:PGtkWidget; event:PGdkEvent); cdecl; external gtklib;
function _gtk_boolean_handled_accumulator(ihint:PGSignalInvocationHint; return_accu:PGValue; handler_return:PGValue; dummy:gpointer):gboolean; cdecl; external gtklib;
function _gtk_find_module(name:Pgchar; _type:Pgchar):Pgchar; cdecl; external gtklib;
function _gtk_get_module_path(_type:Pgchar):PPgchar; cdecl; external gtklib;

{$ifdef HasGTK2_6}
function gtk_get_option_group(open_default_display:gboolean):PGOptionGroup; cdecl; external gtklib;
function gtk_init_with_args(argc:PLongint; argv:PPPchar; parameter_string:Pchar; 
  entries:PGOptionEntry; translation_domain:Pchar; error:PPGError):gboolean; cdecl; external gtklib;
{$endif}

{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
{$ifdef G_PLATFORM_WIN32}
procedure gtk_init(argc: Plongint; argv : PPPchar);
begin
   gtk_init_abi_check(argc,argv,2,sizeof(TGtkWindow),sizeof(TGtkBox));
end;

function gtk_init_check(argc: Plongint; argv : PPPchar) : gboolean;
begin
   gtk_init_check:=gtk_init_check_abi_check(argc,argv,2,sizeof(TGtkWindow),sizeof(TGtkBox));
end;
{$endif  G_PLATFORM_WIN32}

{$ENDIF read_implementation}
// included by gtk2.pas

