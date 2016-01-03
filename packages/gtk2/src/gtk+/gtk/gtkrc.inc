// included by gtk2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
   PGtkRcFlags = ^TGtkRcFlags;
   TGtkRcFlags = longint;

{< public > }
{< private > }
{ list of RC style lists including this RC style  }
{ The RC file specified the engine  }
   PGtkRcStyle = ^TGtkRcStyle;
   TGtkRcStyle = record
        parent_instance : TGObject;
        name : Pgchar;
        bg_pixmap_name : array[0..4] of Pgchar;
        font_desc : PPangoFontDescription;
        color_flags : array[0..4] of TGtkRcFlags;
        fg : array[0..4] of TGdkColor;
        bg : array[0..4] of TGdkColor;
        text : array[0..4] of TGdkColor;
        base : array[0..4] of TGdkColor;
        xthickness : gint;
        ythickness : gint;
        rc_properties : PGArray;
        rc_style_lists : PGSList;
        icon_factories : PGSList;
        flag0 : word;
     end;

{ Create an empty RC style of the same type as this RC style.
     The default implementation, which does
     g_object_new (G_OBJECT_TYPE (style), NULL);
     should work in most cases.
    }
{ Fill in engine specific parts of GtkRcStyle by parsing contents
     of brackets. Returns G_TOKEN_NONE if succesful, otherwise returns
     the token it expected but didn't get.
    }
<<<<<<< HEAD
<<<<<<< HEAD
{ Combine RC style data from src into dest. If overridden, this
=======
{ Combine RC style data from src into dest. If overriden, this
>>>>>>> graemeg/fixes_2_2
=======
{ Combine RC style data from src into dest. If overriden, this
>>>>>>> origin/fixes_2_2
     function should chain to the parent.
    }
{ Create an empty style suitable to this RC style
    }
{ Padding for future expansion  }
   PGtkRcStyleClass = ^TGtkRcStyleClass;
   TGtkRcStyleClass = record
        parent_class : TGObjectClass;
        create_rc_style : function (rc_style:PGtkRcStyle):PGtkRcStyle; cdecl;
        parse : function (rc_style:PGtkRcStyle; settings:PGtkSettings; scanner:PGScanner):guint; cdecl;
        merge : procedure (dest:PGtkRcStyle; src:PGtkRcStyle); cdecl;
        create_style : function (rc_style:PGtkRcStyle):PGtkStyle; cdecl;
        _gtk_reserved1 : procedure ; cdecl;
        _gtk_reserved2 : procedure ; cdecl;
        _gtk_reserved3 : procedure ; cdecl;
        _gtk_reserved4 : procedure ; cdecl;
     end;

{ private functions/definitions  }
   PGtkRcTokenType = ^TGtkRcTokenType;
   TGtkRcTokenType = (
     GTK_RC_TOKEN_INVALID {:= G_TOKEN_LAST},
     GTK_RC_TOKEN_INCLUDE,
     GTK_RC_TOKEN_NORMAL,
     GTK_RC_TOKEN_ACTIVE,
     GTK_RC_TOKEN_PRELIGHT,
     GTK_RC_TOKEN_SELECTED,
     GTK_RC_TOKEN_INSENSITIVE,
     GTK_RC_TOKEN_FG,
     GTK_RC_TOKEN_BG,
     GTK_RC_TOKEN_TEXT,
     GTK_RC_TOKEN_BASE,
     GTK_RC_TOKEN_XTHICKNESS,
     GTK_RC_TOKEN_YTHICKNESS,
     GTK_RC_TOKEN_FONT,
     GTK_RC_TOKEN_FONTSET,
     GTK_RC_TOKEN_FONT_NAME,
     GTK_RC_TOKEN_BG_PIXMAP,
     GTK_RC_TOKEN_PIXMAP_PATH,
     GTK_RC_TOKEN_STYLE,
     GTK_RC_TOKEN_BINDING,
     GTK_RC_TOKEN_BIND,
     GTK_RC_TOKEN_WIDGET,
     GTK_RC_TOKEN_WIDGET_CLASS,
     GTK_RC_TOKEN_CLASS,
     GTK_RC_TOKEN_LOWEST,
     GTK_RC_TOKEN_GTK,
     GTK_RC_TOKEN_APPLICATION,
     GTK_RC_TOKEN_THEME,
     GTK_RC_TOKEN_RC,
     GTK_RC_TOKEN_HIGHEST,
     GTK_RC_TOKEN_ENGINE,
     GTK_RC_TOKEN_MODULE_PATH,
     GTK_RC_TOKEN_IM_MODULE_PATH,
     GTK_RC_TOKEN_IM_MODULE_FILE,
     GTK_RC_TOKEN_STOCK,
     GTK_RC_TOKEN_LTR,
     GTK_RC_TOKEN_RTL,
     GTK_RC_TOKEN_LAST
   );

{ rc properties
   (structure forward declared in gtkstyle.h)
  }
{ quark-ified property identifier like "GtkScrollbar::spacing"  }
{ fields similar to GtkSettingsValue  }
   PGtkRcProperty = ^TGtkRcProperty;
   TGtkRcProperty = record
        type_name : TGQuark;
        property_name : TGQuark;
        origin : Pgchar;
        value : TGValue;
     end;

{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
const
   GTK_RC_FG = 1 shl 0;
   GTK_RC_BG = 1 shl 1;
   GTK_RC_TEXT = 1 shl 2;
   GTK_RC_BASE = 1 shl 3;


const
   bm_TGtkRcStyle_engine_specified = $1;
   bp_TGtkRcStyle_engine_specified = 0;


function GTK_TYPE_RC_STYLE : GType;
function GTK_RC_STYLE_get(anObject: pointer) : PGtkRcStyle;
function GTK_RC_STYLE_CLASS(klass: pointer) : PGtkRcStyleClass;
function GTK_IS_RC_STYLE(anObject: pointer) : boolean;
function GTK_IS_RC_STYLE_CLASS(klass: pointer) : boolean;
function GTK_RC_STYLE_GET_CLASS(obj: pointer) : PGtkRcStyleClass;

function engine_specified(var a : TGtkRcStyle) : guint;
procedure set_engine_specified(var a : TGtkRcStyle; __engine_specified : guint);

procedure _gtk_rc_init; cdecl; external gtklib;
procedure gtk_rc_add_default_file(filename:Pgchar); cdecl; external gtklib;
procedure gtk_rc_set_default_files(filenames:PPgchar); cdecl; external gtklib;
function gtk_rc_get_default_files:PPgchar; cdecl; external gtklib;
function gtk_rc_get_style(widget:PGtkWidget):PGtkStyle; cdecl; external gtklib;
function gtk_rc_get_style_by_paths(settings:PGtkSettings; widget_path:Pchar; class_path:Pchar; _type:GType):PGtkStyle; cdecl; external gtklib;
function gtk_rc_reparse_all_for_settings(settings:PGtkSettings; force_load:gboolean):gboolean; cdecl; external gtklib;
function gtk_rc_find_pixmap_in_path(settings:PGtkSettings; scanner:PGScanner; pixmap_file:Pgchar):Pgchar; cdecl; external gtklib;
procedure gtk_rc_parse(filename:Pgchar); cdecl; external gtklib;
procedure gtk_rc_parse_string(rc_string:Pgchar); cdecl; external gtklib;
function gtk_rc_reparse_all:gboolean; cdecl; external gtklib;

{$ifndef GTK_DISABLE_DEPRECATED}
procedure gtk_rc_add_widget_name_style(rc_style:PGtkRcStyle; pattern:Pgchar); cdecl; external gtklib;
procedure gtk_rc_add_widget_class_style(rc_style:PGtkRcStyle; pattern:Pgchar); cdecl; external gtklib;
procedure gtk_rc_add_class_style(rc_style:PGtkRcStyle; pattern:Pgchar); cdecl; external gtklib;
{$endif}
{ GTK_DISABLE_DEPRECATED  }

function gtk_rc_style_get_type:GType; cdecl; external gtklib;
function gtk_rc_style_new:PGtkRcStyle; cdecl; external gtklib;
function gtk_rc_style_copy(orig:PGtkRcStyle):PGtkRcStyle; cdecl; external gtklib;
procedure gtk_rc_style_ref(rc_style:PGtkRcStyle); cdecl; external gtklib;
procedure gtk_rc_style_unref(rc_style:PGtkRcStyle); cdecl; external gtklib;
function gtk_rc_find_module_in_path(module_file:Pgchar):Pgchar; cdecl; external gtklib;
function gtk_rc_get_theme_dir:Pgchar; cdecl; external gtklib;
function gtk_rc_get_module_dir:Pgchar; cdecl; external gtklib;
function gtk_rc_get_im_module_path:Pgchar; cdecl; external gtklib;
function gtk_rc_get_im_module_file:Pgchar; cdecl; external gtklib;


function gtk_rc_scanner_new:PGScanner; cdecl; external gtklib;
function gtk_rc_parse_color(scanner:PGScanner; color:PGdkColor):guint; cdecl; external gtklib;
function gtk_rc_parse_state(scanner:PGScanner; state:PGtkStateType):guint; cdecl; external gtklib;
function gtk_rc_parse_priority(scanner:PGScanner; priority:PGtkPathPriorityType):guint; cdecl; external gtklib;

function _gtk_rc_style_lookup_rc_property(rc_style:PGtkRcStyle; type_name:TGQuark; property_name:TGQuark):PGtkRcProperty; cdecl; external gtklib;
function _gtk_rc_context_get_default_font_name(settings:PGtkSettings):Pgchar; cdecl; external gtklib;

{$ifdef HasGTK2_4}
procedure gtk_rc_reset_styles(settings: PGtkSettings); cdecl; external gtklib;
{$endif}

{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function GTK_TYPE_RC_STYLE : GType;
begin
   GTK_TYPE_RC_STYLE:=gtk_rc_style_get_type;
end;

function GTK_RC_STYLE_get(anObject: pointer) : PGtkRcStyle;
begin
   GTK_RC_STYLE_get:=PGtkRcStyle(G_TYPE_CHECK_INSTANCE_CAST(anObject,GTK_TYPE_RC_STYLE));
end;

function GTK_RC_STYLE_CLASS(klass: pointer) : PGtkRcStyleClass;
begin
   GTK_RC_STYLE_CLASS:=PGtkRcStyleClass(G_TYPE_CHECK_CLASS_CAST(klass,GTK_TYPE_RC_STYLE));
end;

function GTK_IS_RC_STYLE(anObject: pointer) : boolean;
begin
   GTK_IS_RC_STYLE:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,GTK_TYPE_RC_STYLE);
end;

function GTK_IS_RC_STYLE_CLASS(klass: pointer) : boolean;
begin
   GTK_IS_RC_STYLE_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,GTK_TYPE_RC_STYLE);
end;

function GTK_RC_STYLE_GET_CLASS(obj: pointer) : PGtkRcStyleClass;
begin
   GTK_RC_STYLE_GET_CLASS:=PGtkRcStyleClass(G_TYPE_INSTANCE_GET_CLASS(obj,GTK_TYPE_RC_STYLE));
end;

function engine_specified(var a : TGtkRcStyle) : guint;
begin
   engine_specified:=(a.flag0 and bm_TGtkRcStyle_engine_specified) shr bp_TGtkRcStyle_engine_specified;
end;

procedure set_engine_specified(var a : TGtkRcStyle; __engine_specified : guint);
begin
   a.flag0:=a.flag0 or ((__engine_specified shl bp_TGtkRcStyle_engine_specified) and bm_TGtkRcStyle_engine_specified);
end;

{$ENDIF read_implementation}
// included by gtk2.pas

