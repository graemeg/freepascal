// included by gdk2.pp
{$IFDEF read_forward_definitions}
   PPGdkScreen = ^PGdkScreen;
   PGdkScreen = ^TGdkScreen;
   TGdkScreen = record
        parent_instance : TGObject;
     end;

{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
   PGdkScreenClass = ^TGdkScreenClass;
   TGdkScreenClass = record
        parent_class : TGObjectClass;
        get_display : function (screen:PGdkScreen):PGdkDisplay; cdecl;
        get_width : function (screen:PGdkScreen):gint; cdecl;
        get_height : function (screen:PGdkScreen):gint; cdecl;
        get_width_mm : function (screen:PGdkScreen):gint; cdecl;
        get_height_mm : function (screen:PGdkScreen):gint; cdecl;
        get_root_depth : function (screen:PGdkScreen):gint; cdecl;
        get_screen_num : function (screen:PGdkScreen):gint; cdecl;
        get_root_window : function (screen:PGdkScreen):PGdkWindow; cdecl;
        get_default_colormap : function (screen:PGdkScreen):PGdkColormap; cdecl;
        set_default_colormap : procedure (screen:PGdkScreen; colormap:PGdkColormap); cdecl;
        get_window_at_pointer : function (screen:PGdkScreen; win_x:Pgint; win_y:Pgint):PGdkWindow; cdecl;
        get_n_monitors : function (screen:PGdkScreen):gint; cdecl;
        get_monitor_geometry : procedure (screen:PGdkScreen; monitor_num:gint; dest:PGdkRectangle); cdecl;
     end;

{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
function GDK_TYPE_SCREEN : GType;
function GDK_SCREEN(anObject : Pointer) : PGdkScreen;
function GDK_SCREEN_CLASS(klass : Pointer) : PGdkScreenClass;
function GDK_IS_SCREEN(anObject : Pointer) : boolean;
function GDK_IS_SCREEN_CLASS(klass : Pointer) : boolean;
function GDK_SCREEN_GET_CLASS(obj : Pointer) : PGdkScreenClass;


//function gdk_screen_get_type:GType; cdecl; external gdklib;
function gdk_screen_get_default:PGdkScreen; cdecl; external gdklib;
function gdk_screen_get_default_colormap(screen:PGdkScreen):PGdkColormap; cdecl; external gdklib;
procedure gdk_screen_set_default_colormap(screen:PGdkScreen; colormap:PGdkColormap); cdecl; external gdklib;
function gdk_screen_get_system_colormap(screen:PGdkScreen):PGdkColormap; cdecl; external gdklib;
function gdk_screen_get_system_visual(screen:PGdkScreen):PGdkVisual; cdecl; external gdklib;
function gdk_screen_get_rgb_colormap(screen:PGdkScreen):PGdkColormap; cdecl; external gdklib;
function gdk_screen_get_rgb_visual(screen:PGdkScreen):PGdkVisual; cdecl; external gdklib;
function gdk_screen_get_root_window(screen:PGdkScreen):PGdkWindow; cdecl; external gdklib;
function gdk_screen_get_display(screen:PGdkScreen):PGdkDisplay; cdecl; external gdklib;
function gdk_screen_get_number(screen:PGdkScreen):gint; cdecl; external gdklib;
function gdk_screen_get_window_at_pointer(screen:PGdkScreen; win_x:Pgint; win_y:Pgint):PGdkWindow; cdecl; external gdklib;
function gdk_screen_get_width(screen:PGdkScreen):gint; cdecl; external gdklib;
function gdk_screen_get_height(screen:PGdkScreen):gint; cdecl; external gdklib;
function gdk_screen_get_width_mm(screen:PGdkScreen):gint; cdecl; external gdklib;
function gdk_screen_get_height_mm(screen:PGdkScreen):gint; cdecl; external gdklib;
procedure gdk_screen_close(screen:PGdkScreen); cdecl; external gdklib;
function gdk_screen_list_visuals(screen:PGdkScreen):PGList; cdecl; external gdklib;
function gdk_screen_get_toplevel_windows(screen:PGdkScreen):PGList; cdecl; external gdklib;
function gdk_screen_get_n_monitors(screen:PGdkScreen):gint; cdecl; external gdklib;
procedure gdk_screen_get_monitor_geometry(screen:PGdkScreen; monitor_num:gint; dest:PGdkRectangle); cdecl; external gdklib;
function gdk_screen_get_monitor_at_point(screen:PGdkScreen; x:gint; y:gint):gint; cdecl; external gdklib;
function gdk_screen_get_monitor_at_window(screen:PGdkScreen; window:PGdkWindow):gint; cdecl; external gdklib;
procedure gdk_screen_broadcast_client_message(screen:PGdkScreen; event:PGdkEvent); cdecl; external gdklib;
<<<<<<< HEAD
<<<<<<< HEAD
=======
function gdk_get_default_screen:PGdkScreen; cdecl; external gdklib;
>>>>>>> graemeg/fixes_2_2
=======
function gdk_get_default_screen:PGdkScreen; cdecl; external gdklib;
>>>>>>> origin/fixes_2_2
function gdk_screen_get_setting(screen:PGdkScreen; name:Pgchar; value:PGValue):gboolean; cdecl; external gdklib;
function gdk_screen_make_display_name(screen: PGdkScreen): Pgchar; cdecl; external gdklib;
{$endif read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function GDK_TYPE_SCREEN : GType;
begin
  GDK_TYPE_SCREEN := 0; //gdk_screen_get_type;
end;

function GDK_SCREEN(anObject : Pointer) : PGdkScreen;
begin
  GDK_SCREEN:=PGdkScreen(G_TYPE_CHECK_INSTANCE_CAST(anObject,GDK_TYPE_SCREEN));
end;

function GDK_SCREEN_CLASS(klass : Pointer) : PGdkScreenClass;
begin
  GDK_SCREEN_CLASS:=PGdkScreenClass(G_TYPE_CHECK_CLASS_CAST(klass,
                                                              GDK_TYPE_SCREEN));
end;

function GDK_IS_SCREEN(anObject : Pointer) : boolean;
begin
  GDK_IS_SCREEN:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,GDK_TYPE_SCREEN);
end;

function GDK_IS_SCREEN_CLASS(klass : Pointer) : boolean;
begin
  GDK_IS_SCREEN_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,GDK_TYPE_SCREEN);
end;

function GDK_SCREEN_GET_CLASS(obj : Pointer) : PGdkScreenClass;
begin
  GDK_SCREEN_GET_CLASS:=PGdkScreenClass(G_TYPE_INSTANCE_GET_CLASS(obj,
                                                              GDK_TYPE_SCREEN));
end;
{$ENDIF}

