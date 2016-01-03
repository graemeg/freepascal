// included by gtk2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
   PGtkTextLineDisplay = ^TGtkTextLineDisplay;


{ This is a "semi-private" header; it is intended for
   use by the text widget, and the text canvas item,
   but that's all. We may have to install it so the
   canvas item can use it, but users are not supposed
   to use it.
  }
{ forward declarations that have to be here to avoid including
   gtktextbtree.h
  }
{ width of the display area on-screen,
     i.e. pixels we should wrap to fit inside.  }
{ width/height of the total logical area being layed out  }
{ Pixel offsets from the left and from the top to be used when we
     draw; these allow us to create left/top margins. We don't need
     anything special for bottom/right margins, because those don't
     affect drawing.
    }
{ gint left_edge;  }
{ gint top_edge;  }
{ Default style used if no tags override it  }
{ Pango contexts used for creating layouts  }
{ A cache of one style; this is used to ensure
     we don't constantly regenerate the style
     over long runs with the same style.  }
{ A cache of one line display. Getting the same line
     many times in a row is the most common case.
    }
{ Whether we are allowed to wrap right now  }
{ Whether to show the insertion cursor  }
{ For what GtkTextDirection to draw cursor GTK_TEXT_DIR_NONE -
     means draw both cursors.
    }
{ The preedit string and attributes, if any  }
   PGtkTextLayout = ^TGtkTextLayout;
   TGtkTextLayout = record
        parent_instance : TGObject;
        screen_width : gint;
        width : gint;
        height : gint;
        buffer : PGtkTextBuffer;
        default_style : PGtkTextAttributes;
        ltr_context : PPangoContext;
        rtl_context : PPangoContext;
        one_style_cache : PGtkTextAttributes;
        one_display_cache : PGtkTextLineDisplay;
        wrap_loop_count : gint;
        flag0 : word;
        preedit_string : Pgchar;
        preedit_attrs : PPangoAttrList;
        preedit_len : gint;
        preedit_cursor : gint;
     end;


{ Some portion of the layout was invalidated
    }
{ A range of the layout changed appearance and possibly height
    }
{ may be NULL  }


{ Padding for future expansion  }
   PGtkTextLayoutClass = ^TGtkTextLayoutClass;
   TGtkTextLayoutClass = record
        parent_class : TGObjectClass;
        invalidated : procedure (layout:PGtkTextLayout); cdecl;
        changed : procedure (layout:PGtkTextLayout; y:gint; old_height:gint; new_height:gint); cdecl;
        wrap : function (layout:PGtkTextLayout; line:PGtkTextLine; line_data:PGtkTextLineData):PGtkTextLineData; cdecl;
        get_log_attrs : procedure (layout:PGtkTextLayout; line:PGtkTextLine; var attrs:PPangoLogAttr; n_attrs:Pgint); cdecl;
        invalidate : procedure (layout:PGtkTextLayout; start:PGtkTextIter; theEnd:PGtkTextIter); cdecl;
        free_line_data : procedure (layout:PGtkTextLayout; line:PGtkTextLine; line_data:PGtkTextLineData); cdecl;
        allocate_child : procedure (layout:PGtkTextLayout; child:PGtkWidget; x:gint; y:gint); cdecl;
        _gtk_reserved1 : procedure ; cdecl;
        _gtk_reserved2 : procedure ; cdecl;
        _gtk_reserved3 : procedure ; cdecl;
        _gtk_reserved4 : procedure ; cdecl;
     end;

   PGtkTextAttrAppearance = ^TGtkTextAttrAppearance;
   TGtkTextAttrAppearance = record
        attr : TPangoAttribute;
        appearance : TGtkTextAppearance;
     end;

   PGtkTextCursorDisplay = ^TGtkTextCursorDisplay;
   TGtkTextCursorDisplay = record
        x : gint;
        y : gint;
        height : gint;
        flag0 : word;
     end;


{ Width of layout  }
{ width - margins, if no width set on layout, if width set on layout, -1  }
{ Amount layout is shifted from left edge - this is the left margin
     plus any other factors, such as alignment or indentation.
    }
{ Byte index of insert cursor within para or -1  }
   TGtkTextLineDisplay = record
        layout : PPangoLayout;
        cursors : PGSList;
        shaped_objects : PGSList;
        direction : TGtkTextDirection;
        width : gint;
        total_width : gint;
        height : gint;
        x_offset : gint;
        left_margin : gint;
        right_margin : gint;
        top_margin : gint;
        bottom_margin : gint;
        insert_index : gint;
        size_only : gboolean;
        line : PGtkTextLine;
     end;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
{$IFNDEF KYLIX}
var
  {$IFDEF WIN32}
   gtk_text_attr_appearance_type : TPangoAttrType; external gtklib name 'gtk_text_attr_appearance_type';
  {$ELSE}
   gtk_text_attr_appearance_type : TPangoAttrType;cvar; external;
  {$ENDIF}
{$ENDIF}


function GTK_TYPE_TEXT_LAYOUT : GType;
function GTK_TEXT_LAYOUT(obj: pointer) : PGtkTextLayout;
function GTK_TEXT_LAYOUT_CLASS(klass: pointer) : PGtkTextLayoutClass;
function GTK_IS_TEXT_LAYOUT(obj: pointer) : boolean;
function GTK_IS_TEXT_LAYOUT_CLASS(klass: pointer) : boolean;
function GTK_TEXT_LAYOUT_GET_CLASS(obj: pointer) : PGtkTextLayoutClass;


const
   bm_TGtkTextLayout_cursor_visible = $1;
   bp_TGtkTextLayout_cursor_visible = 0;
   bm_TGtkTextLayout_cursor_direction = $6;
   bp_TGtkTextLayout_cursor_direction = 1;

function cursor_visible(var a : TGtkTextLayout) : guint; overload;
procedure set_cursor_visible(var a : TGtkTextLayout; __cursor_visible : guint); overload;
function cursor_direction(var a : TGtkTextLayout) : gint;
procedure set_cursor_direction(var a : TGtkTextLayout; __cursor_direction : gint);

const
   bm_TGtkTextCursorDisplay_is_strong = $1;
   bp_TGtkTextCursorDisplay_is_strong = 0;
   bm_TGtkTextCursorDisplay_is_weak = $2;
   bp_TGtkTextCursorDisplay_is_weak = 1;

function is_strong(var a : TGtkTextCursorDisplay) : guint;
procedure set_is_strong(var a : TGtkTextCursorDisplay; __is_strong : guint);
function is_weak(var a : TGtkTextCursorDisplay) : guint;
procedure set_is_weak(var a : TGtkTextCursorDisplay; __is_weak : guint);

function gtk_text_layout_get_type:GType; cdecl; external gtklib;
function gtk_text_layout_new:PGtkTextLayout; cdecl; external gtklib;
procedure gtk_text_layout_set_buffer(layout:PGtkTextLayout; buffer:PGtkTextBuffer); cdecl; external gtklib;
function gtk_text_layout_get_buffer(layout:PGtkTextLayout):PGtkTextBuffer; cdecl; external gtklib;
procedure gtk_text_layout_set_default_style(layout:PGtkTextLayout; values:PGtkTextAttributes); cdecl; external gtklib;
procedure gtk_text_layout_set_contexts(layout:PGtkTextLayout; ltr_context:PPangoContext; rtl_context:PPangoContext); cdecl; external gtklib;
procedure gtk_text_layout_set_cursor_direction(layout:PGtkTextLayout; direction:TGtkTextDirection); cdecl; external gtklib;
procedure gtk_text_layout_default_style_changed(layout:PGtkTextLayout); cdecl; external gtklib;
procedure gtk_text_layout_set_screen_width(layout:PGtkTextLayout; width:gint); cdecl; external gtklib;

procedure gtk_text_layout_set_preedit_string(layout:PGtkTextLayout; preedit_string:Pgchar; preedit_attrs:PPangoAttrList; cursor_pos:gint); cdecl; external gtklib;
procedure gtk_text_layout_set_cursor_visible(layout:PGtkTextLayout; cursor_visible:gboolean); cdecl; external gtklib;
function gtk_text_layout_get_cursor_visible(layout:PGtkTextLayout):gboolean; cdecl; external gtklib;
{ Getting the size or the lines potentially results in a call to
   recompute, which is pretty massively expensive. Thus it should
   basically only be done in an idle handler.

   Long-term, we would really like to be able to do these without
   a full recompute so they may get cheaper over time.
  }
procedure gtk_text_layout_get_size(layout:PGtkTextLayout; width:Pgint; height:Pgint); cdecl; external gtklib;
{ [top_y, bottom_y)  }
function gtk_text_layout_get_lines(layout:PGtkTextLayout; top_y:gint; bottom_y:gint; first_line_y:Pgint):PGSList; cdecl; external gtklib;
procedure gtk_text_layout_wrap_loop_start(layout:PGtkTextLayout); cdecl; external gtklib;
procedure gtk_text_layout_wrap_loop_end(layout:PGtkTextLayout); cdecl; external gtklib;
function gtk_text_layout_get_line_display(layout:PGtkTextLayout; line:PGtkTextLine; size_only:gboolean):PGtkTextLineDisplay; cdecl; external gtklib;
procedure gtk_text_layout_free_line_display(layout:PGtkTextLayout; display:PGtkTextLineDisplay); cdecl; external gtklib;
procedure gtk_text_layout_get_line_at_y(layout:PGtkTextLayout; target_iter:PGtkTextIter; y:gint; line_top:Pgint); cdecl; external gtklib;
procedure gtk_text_layout_get_iter_at_pixel(layout:PGtkTextLayout; iter:PGtkTextIter; x:gint; y:gint); cdecl; external gtklib;


procedure gtk_text_layout_invalidate(layout:PGtkTextLayout; start:PGtkTextIter; theEnd:PGtkTextIter); cdecl; external gtklib;
procedure gtk_text_layout_free_line_data(layout:PGtkTextLayout; line:PGtkTextLine; line_data:PGtkTextLineData); cdecl; external gtklib;
function gtk_text_layout_is_valid(layout:PGtkTextLayout):gboolean; cdecl; external gtklib;
procedure gtk_text_layout_validate_yrange(layout:PGtkTextLayout; anchor_line:PGtkTextIter; y0:gint; y1:gint); cdecl; external gtklib;
procedure gtk_text_layout_validate(layout:PGtkTextLayout; max_pixels:gint); cdecl; external gtklib;
{ This function should return the passed-in line data,
   OR remove the existing line data from the line, and
   return a NEW line data after adding it to the line.
   That is, invariant after calling the callback is that
   there should be exactly one line data for this view
   stored on the btree line.
  }
function gtk_text_layout_wrap(layout:PGtkTextLayout; line:PGtkTextLine; line_data:PGtkTextLineData):PGtkTextLineData; cdecl; external gtklib;
{ may be NULL  }
procedure gtk_text_layout_changed(layout:PGtkTextLayout; y:gint; old_height:gint; new_height:gint); cdecl; external gtklib;
procedure gtk_text_layout_get_iter_location(layout:PGtkTextLayout; iter:PGtkTextIter; rect:PGdkRectangle); cdecl; external gtklib;
procedure gtk_text_layout_get_line_yrange(layout:PGtkTextLayout; iter:PGtkTextIter; y:Pgint; height:Pgint); cdecl; external gtklib;
procedure _gtk_text_layout_get_line_xrange(layout:PGtkTextLayout; iter:PGtkTextIter; x:Pgint; width:Pgint); cdecl; external gtklib;
procedure gtk_text_layout_get_cursor_locations(layout:PGtkTextLayout; iter:PGtkTextIter; strong_pos:PGdkRectangle; weak_pos:PGdkRectangle); cdecl; external gtklib;
function gtk_text_layout_clamp_iter_to_vrange(layout:PGtkTextLayout; iter:PGtkTextIter; top:gint; bottom:gint):gboolean; cdecl; external gtklib;
function gtk_text_layout_move_iter_to_line_end(layout:PGtkTextLayout; iter:PGtkTextIter; direction:gint):gboolean; cdecl; external gtklib;
function gtk_text_layout_move_iter_to_previous_line(layout:PGtkTextLayout; iter:PGtkTextIter):gboolean; cdecl; external gtklib;
function gtk_text_layout_move_iter_to_next_line(layout:PGtkTextLayout; iter:PGtkTextIter):gboolean; cdecl; external gtklib;
procedure gtk_text_layout_move_iter_to_x(layout:PGtkTextLayout; iter:PGtkTextIter; x:gint); cdecl; external gtklib;
function gtk_text_layout_move_iter_visually(layout:PGtkTextLayout; iter:PGtkTextIter; count:gint):gboolean; cdecl; external gtklib;
function gtk_text_layout_iter_starts_line(layout:PGtkTextLayout; iter:PGtkTextIter):gboolean; cdecl; external gtklib;
procedure gtk_text_layout_get_iter_at_line(layout:PGtkTextLayout; iter:PGtkTextIter; line:PGtkTextLine; byte_offset:gint); cdecl; external gtklib;
{ Don't use these. Use gtk_text_view_add_child_at_anchor().
   These functions are defined in gtktextchild.c, but here
   since they are semi-public and require GtkTextLayout to
   be declared.
  }
procedure gtk_text_child_anchor_register_child(anchor:PGtkTextChildAnchor; child:PGtkWidget; layout:PGtkTextLayout); cdecl; external gtklib;
procedure gtk_text_child_anchor_unregister_child(anchor:PGtkTextChildAnchor; child:PGtkWidget); cdecl; external gtklib;
procedure gtk_text_child_anchor_queue_resize(anchor:PGtkTextChildAnchor; layout:PGtkTextLayout); cdecl; external gtklib;
procedure gtk_text_anchored_child_set_layout(child:PGtkWidget; layout:PGtkTextLayout); cdecl; external gtklib;
procedure gtk_text_layout_spew(layout:PGtkTextLayout); cdecl; external gtklib;
{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function GTK_TYPE_TEXT_LAYOUT : GType;
begin
   GTK_TYPE_TEXT_LAYOUT:=gtk_text_layout_get_type;
end;

function GTK_TEXT_LAYOUT(obj: pointer) : PGtkTextLayout;
begin
   GTK_TEXT_LAYOUT:=PGtkTextLayout(G_TYPE_CHECK_INSTANCE_CAST(obj,GTK_TYPE_TEXT_LAYOUT));
end;

function GTK_TEXT_LAYOUT_CLASS(klass: pointer) : PGtkTextLayoutClass;
begin
   GTK_TEXT_LAYOUT_CLASS:=PGtkTextLayoutClass(G_TYPE_CHECK_CLASS_CAST(klass,GTK_TYPE_TEXT_LAYOUT));
end;

function GTK_IS_TEXT_LAYOUT(obj: pointer) : boolean;
begin
   GTK_IS_TEXT_LAYOUT:=G_TYPE_CHECK_INSTANCE_TYPE(obj,GTK_TYPE_TEXT_LAYOUT);
end;

function GTK_IS_TEXT_LAYOUT_CLASS(klass: pointer) : boolean;
begin
   GTK_IS_TEXT_LAYOUT_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,GTK_TYPE_TEXT_LAYOUT);
end;

function GTK_TEXT_LAYOUT_GET_CLASS(obj: pointer) : PGtkTextLayoutClass;
begin
   GTK_TEXT_LAYOUT_GET_CLASS:=PGtkTextLayoutClass(G_TYPE_INSTANCE_GET_CLASS(obj,GTK_TYPE_TEXT_LAYOUT));
end;

function cursor_visible(var a : TGtkTextLayout) : guint;
begin
   cursor_visible:=(a.flag0 and bm_TGtkTextLayout_cursor_visible) shr bp_TGtkTextLayout_cursor_visible;
end;

procedure set_cursor_visible(var a : TGtkTextLayout; __cursor_visible : guint);
begin
   a.flag0:=a.flag0 or ((__cursor_visible shl bp_TGtkTextLayout_cursor_visible) and bm_TGtkTextLayout_cursor_visible);
end;

function cursor_direction(var a : TGtkTextLayout) : gint;
begin
   cursor_direction:=(a.flag0 and bm_TGtkTextLayout_cursor_direction) shr bp_TGtkTextLayout_cursor_direction;
end;

procedure set_cursor_direction(var a : TGtkTextLayout; __cursor_direction : gint);
begin
   a.flag0:=a.flag0 or ((__cursor_direction shl bp_TGtkTextLayout_cursor_direction) and bm_TGtkTextLayout_cursor_direction);
end;

function is_strong(var a : TGtkTextCursorDisplay) : guint;
begin
   is_strong:=(a.flag0 and bm_TGtkTextCursorDisplay_is_strong) shr bp_TGtkTextCursorDisplay_is_strong;
end;

procedure set_is_strong(var a : TGtkTextCursorDisplay; __is_strong : guint);
begin
   a.flag0:=a.flag0 or ((__is_strong shl bp_TGtkTextCursorDisplay_is_strong) and bm_TGtkTextCursorDisplay_is_strong);
end;

function is_weak(var a : TGtkTextCursorDisplay) : guint;
begin
   is_weak:=(a.flag0 and bm_TGtkTextCursorDisplay_is_weak) shr bp_TGtkTextCursorDisplay_is_weak;
end;

procedure set_is_weak(var a : TGtkTextCursorDisplay; __is_weak : guint);
begin
   a.flag0:=a.flag0 or ((__is_weak shl bp_TGtkTextCursorDisplay_is_weak) and bm_TGtkTextCursorDisplay_is_weak);
end;

{$ENDIF read_implementation}
// included by gtk2.pas

