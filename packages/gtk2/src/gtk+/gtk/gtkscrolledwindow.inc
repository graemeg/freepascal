// included by gtk2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
{ Flag used by ::move-focus-out implementation  }
   PGtkScrolledWindow = ^TGtkScrolledWindow;
   TGtkScrolledWindow = record
        container : TGtkBin;
        hscrollbar : PGtkWidget;
        vscrollbar : PGtkWidget;
<<<<<<< HEAD
<<<<<<< HEAD
        flag0 : {$ifdef windows}longint{$else}word{$endif};
=======
        flag0 : {$ifdef win32}longint{$else}word{$endif};
>>>>>>> graemeg/fixes_2_2
=======
        flag0 : {$ifdef win32}longint{$else}word{$endif};
>>>>>>> origin/fixes_2_2
        shadow_type : guint16;
     end;

{ Action signals for keybindings. Do not connect to these signals
    }
{ Unfortunately, GtkScrollType is deficient in that there is
     no horizontal/vertical variants for GTK_SCROLL_START/END,
     so we have to add an additional boolean flag.
    }
{ Padding for future expansion  }
   PGtkScrolledWindowClass = ^TGtkScrolledWindowClass;
   TGtkScrolledWindowClass = record
        parent_class : TGtkBinClass;
        scrollbar_spacing : gint;
        scroll_child : procedure (scrolled_window:PGtkScrolledWindow; scroll:TGtkScrollType; horizontal:gboolean); cdecl;
        move_focus_out : procedure (scrolled_window:PGtkScrolledWindow; direction:TGtkDirectionType); cdecl;
        _gtk_reserved1 : procedure ; cdecl;
        _gtk_reserved2 : procedure ; cdecl;
        _gtk_reserved3 : procedure ; cdecl;
        _gtk_reserved4 : procedure ; cdecl;
     end;

{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
const
   bm_TGtkScrolledWindow_hscrollbar_policy = $3;
   bp_TGtkScrolledWindow_hscrollbar_policy = 0;
   bm_TGtkScrolledWindow_vscrollbar_policy = $C;
   bp_TGtkScrolledWindow_vscrollbar_policy = 2;
   bm_TGtkScrolledWindow_hscrollbar_visible = $10;
   bp_TGtkScrolledWindow_hscrollbar_visible = 4;
   bm_TGtkScrolledWindow_vscrollbar_visible = $20;
   bp_TGtkScrolledWindow_vscrollbar_visible = 5;
   bm_TGtkScrolledWindow_window_placement = $C0;
   bp_TGtkScrolledWindow_window_placement = 6;
   bm_TGtkScrolledWindow_focus_out = $100;
   bp_TGtkScrolledWindow_focus_out = 8;


function GTK_TYPE_SCROLLED_WINDOW : GType;
function GTK_SCROLLED_WINDOW(obj: pointer) : PGtkScrolledWindow;
function GTK_SCROLLED_WINDOW_CLASS(klass: pointer) : PGtkScrolledWindowClass;
function GTK_IS_SCROLLED_WINDOW(obj: pointer) : boolean;
function GTK_IS_SCROLLED_WINDOW_CLASS(klass: pointer) : boolean;
function GTK_SCROLLED_WINDOW_GET_CLASS(obj: pointer) : PGtkScrolledWindowClass;

function hscrollbar_policy(var a : TGtkScrolledWindow) : guint;
procedure set_hscrollbar_policy(var a : TGtkScrolledWindow; __hscrollbar_policy : guint);
function vscrollbar_policy(var a : TGtkScrolledWindow) : guint;
procedure set_vscrollbar_policy(var a : TGtkScrolledWindow; __vscrollbar_policy : guint);
function hscrollbar_visible(var a : TGtkScrolledWindow) : guint;
procedure set_hscrollbar_visible(var a : TGtkScrolledWindow; __hscrollbar_visible : guint);
function vscrollbar_visible(var a : TGtkScrolledWindow) : guint;
procedure set_vscrollbar_visible(var a : TGtkScrolledWindow; __vscrollbar_visible : guint);
function window_placement(var a : TGtkScrolledWindow) : guint;
procedure set_window_placement(var a : TGtkScrolledWindow; __window_placement : guint);
function focus_out(var a : TGtkScrolledWindow) : guint; overload;
procedure set_focus_out(var a : TGtkScrolledWindow; __focus_out : guint); overload;

function gtk_scrolled_window_get_type:TGtkType; cdecl; external gtklib;
function gtk_scrolled_window_new(hadjustment:PGtkAdjustment; vadjustment:PGtkAdjustment):PGtkWidget; cdecl; external gtklib;
procedure gtk_scrolled_window_set_hadjustment(scrolled_window:PGtkScrolledWindow; hadjustment:PGtkAdjustment); cdecl; external gtklib;
procedure gtk_scrolled_window_set_vadjustment(scrolled_window:PGtkScrolledWindow; hadjustment:PGtkAdjustment); cdecl; external gtklib;
function gtk_scrolled_window_get_hadjustment(scrolled_window:PGtkScrolledWindow):PGtkAdjustment; cdecl; external gtklib;
function gtk_scrolled_window_get_vadjustment(scrolled_window:PGtkScrolledWindow):PGtkAdjustment; cdecl; external gtklib;
procedure gtk_scrolled_window_set_policy(scrolled_window:PGtkScrolledWindow; hscrollbar_policy:TGtkPolicyType; vscrollbar_policy:TGtkPolicyType); cdecl; external gtklib;
procedure gtk_scrolled_window_get_policy(scrolled_window:PGtkScrolledWindow; hscrollbar_policy:PGtkPolicyType; vscrollbar_policy:PGtkPolicyType); cdecl; external gtklib;
procedure gtk_scrolled_window_set_placement(scrolled_window:PGtkScrolledWindow; window_placement:TGtkCornerType); cdecl; external gtklib;
function gtk_scrolled_window_get_placement(scrolled_window:PGtkScrolledWindow):TGtkCornerType; cdecl; external gtklib;
procedure gtk_scrolled_window_set_shadow_type(scrolled_window:PGtkScrolledWindow; _type:TGtkShadowType); cdecl; external gtklib;
function gtk_scrolled_window_get_shadow_type(scrolled_window:PGtkScrolledWindow):TGtkShadowType; cdecl; external gtklib;
procedure gtk_scrolled_window_add_with_viewport(scrolled_window:PGtkScrolledWindow; child:PGtkWidget); cdecl; external gtklib;
{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function GTK_TYPE_SCROLLED_WINDOW : GType;
begin
   GTK_TYPE_SCROLLED_WINDOW:=gtk_scrolled_window_get_type;
end;

function GTK_SCROLLED_WINDOW(obj: pointer) : PGtkScrolledWindow;
begin
   GTK_SCROLLED_WINDOW:=PGtkScrolledWindow(GTK_CHECK_CAST(obj,GTK_TYPE_SCROLLED_WINDOW));
end;

function GTK_SCROLLED_WINDOW_CLASS(klass: pointer) : PGtkScrolledWindowClass;
begin
   GTK_SCROLLED_WINDOW_CLASS:=PGtkScrolledWindowClass(GTK_CHECK_CLASS_CAST(klass,GTK_TYPE_SCROLLED_WINDOW));
end;

function GTK_IS_SCROLLED_WINDOW(obj: pointer) : boolean;
begin
   GTK_IS_SCROLLED_WINDOW:=GTK_CHECK_TYPE(obj,GTK_TYPE_SCROLLED_WINDOW);
end;

function GTK_IS_SCROLLED_WINDOW_CLASS(klass: pointer) : boolean;
begin
   GTK_IS_SCROLLED_WINDOW_CLASS:=GTK_CHECK_CLASS_TYPE(klass,GTK_TYPE_SCROLLED_WINDOW);
end;

function GTK_SCROLLED_WINDOW_GET_CLASS(obj: pointer) : PGtkScrolledWindowClass;
begin
   GTK_SCROLLED_WINDOW_GET_CLASS:=PGtkScrolledWindowClass(GTK_CHECK_GET_CLASS(obj,GTK_TYPE_SCROLLED_WINDOW));
end;

function hscrollbar_policy(var a : TGtkScrolledWindow) : guint;
begin
   hscrollbar_policy:=(a.flag0 and bm_TGtkScrolledWindow_hscrollbar_policy) shr bp_TGtkScrolledWindow_hscrollbar_policy;
end;

procedure set_hscrollbar_policy(var a : TGtkScrolledWindow; __hscrollbar_policy : guint);
begin
   a.flag0:=a.flag0 or ((__hscrollbar_policy shl bp_TGtkScrolledWindow_hscrollbar_policy) and bm_TGtkScrolledWindow_hscrollbar_policy);
end;

function vscrollbar_policy(var a : TGtkScrolledWindow) : guint;
begin
   vscrollbar_policy:=(a.flag0 and bm_TGtkScrolledWindow_vscrollbar_policy) shr bp_TGtkScrolledWindow_vscrollbar_policy;
end;

procedure set_vscrollbar_policy(var a : TGtkScrolledWindow; __vscrollbar_policy : guint);
begin
   a.flag0:=a.flag0 or ((__vscrollbar_policy shl bp_TGtkScrolledWindow_vscrollbar_policy) and bm_TGtkScrolledWindow_vscrollbar_policy);
end;

function hscrollbar_visible(var a : TGtkScrolledWindow) : guint;
begin
   hscrollbar_visible:=(a.flag0 and bm_TGtkScrolledWindow_hscrollbar_visible) shr bp_TGtkScrolledWindow_hscrollbar_visible;
end;

procedure set_hscrollbar_visible(var a : TGtkScrolledWindow; __hscrollbar_visible : guint);
begin
   a.flag0:=a.flag0 or ((__hscrollbar_visible shl bp_TGtkScrolledWindow_hscrollbar_visible) and bm_TGtkScrolledWindow_hscrollbar_visible);
end;

function vscrollbar_visible(var a : TGtkScrolledWindow) : guint;
begin
   vscrollbar_visible:=(a.flag0 and bm_TGtkScrolledWindow_vscrollbar_visible) shr bp_TGtkScrolledWindow_vscrollbar_visible;
end;

procedure set_vscrollbar_visible(var a : TGtkScrolledWindow; __vscrollbar_visible : guint);
begin
   a.flag0:=a.flag0 or ((__vscrollbar_visible shl bp_TGtkScrolledWindow_vscrollbar_visible) and bm_TGtkScrolledWindow_vscrollbar_visible);
end;

function window_placement(var a : TGtkScrolledWindow) : guint;
begin
   window_placement:=(a.flag0 and bm_TGtkScrolledWindow_window_placement) shr bp_TGtkScrolledWindow_window_placement;
end;

procedure set_window_placement(var a : TGtkScrolledWindow; __window_placement : guint);
begin
   a.flag0:=a.flag0 or ((__window_placement shl bp_TGtkScrolledWindow_window_placement) and bm_TGtkScrolledWindow_window_placement);
end;

function focus_out(var a : TGtkScrolledWindow) : guint;
begin
   focus_out:=(a.flag0 and bm_TGtkScrolledWindow_focus_out) shr bp_TGtkScrolledWindow_focus_out;
end;

procedure set_focus_out(var a : TGtkScrolledWindow; __focus_out : guint);
begin
   a.flag0:=a.flag0 or ((__focus_out shl bp_TGtkScrolledWindow_focus_out) and bm_TGtkScrolledWindow_focus_out);
end;

{$ENDIF read_implementation}
// included by gtk2.pas


