{

   libzvt - Zed's Virtual Terminal
   Copyright (C) 1998  Michael Zucchi

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.

 **********************************************************************}
unit libzvt;

{$PACKRECORDS C}
{$Mode ObjFPC}

interface

{$linklib util}

Uses glib, gdk, gtk, gdk_imlib;

const
  libzvtdll='zvt';

Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

  Ppid_t = ^pid_t;
  pid_t = longint;

{$define read_interface}
{$undef read_implementation}

{$Include lists.inc}
{$Include vt.inc}
{$Include vtx.inc}

const
   ZVT_TERM_PIXMAP_SUPPORT = $01;
   ZVT_TERM_EMBOLDEN_SUPPORT = $02;
   ZVT_TERM_TRANSPARENCY_SUPPORT = $04;
   ZVT_TERM_PIXMAPSCROLL_SUPPORT = $08;
   ZVT_TERM_MATCH_SUPPORT = $10;

type
   PZvtTerm = ^TZvtTerm;
   TZvtTerm = record
        widget : TGtkWidget;
        adjustment : PGtkAdjustment;
        shadow_type : TGtkShadowType;
        term_window : PGdkWindow;
        vx : Pvtx;
        charwidth : longint;
        charheight : longint;
        input_id : gint;
        msg_id : gint;
        timeout_id : gint;
        set_grid_size_pending : gboolean;
        grid_width : guint;
        grid_height : guint;
        cursor_bar : PGdkCursor;
        cursor_dot : PGdkCursor;
        cursor_current : PGdkCursor;
        font : PGdkFont;
        font_bold : PGdkFont;
        scroll_gc : PGdkGC;
        fore_gc : PGdkGC;
        back_gc : PGdkGC;
        fore_last : longint;
        back_last : longint;
        color_ctx : PGdkColorContext;
        colors : array[0..17] of gulong;
        ic : PGdkIC;
        pixmap_filename : Pchar;
        background : record
             pix : PGdkPixmap;
             x : longint;
             y : longint;
             w : longint;
             h : longint;
          end;
        flag0 : word;
     end;
    ZVT_TERM = PzvtTerm;

const
   bm_TZvtTerm_cursor_on = $1;
   bp_TZvtTerm_cursor_on = 0;
   bm_TZvtTerm_cursor_filled = $2;
   bp_TZvtTerm_cursor_filled = 1;
   bm_TZvtTerm_cursor_blink_state = $4;
   bp_TZvtTerm_cursor_blink_state = 2;
   bm_TZvtTerm_blink_enabled = $8;
   bp_TZvtTerm_blink_enabled = 3;
   bm_TZvtTerm_in_expose = $10;
   bp_TZvtTerm_in_expose = 4;
   bm_TZvtTerm_scroll_on_keystroke = $20;
   bp_TZvtTerm_scroll_on_keystroke = 5;
   bm_TZvtTerm_scroll_on_output = $40;
   bp_TZvtTerm_scroll_on_output = 6;
   bm_TZvtTerm_transparent = $80;
   bp_TZvtTerm_transparent = 7;
   bm_TZvtTerm_shaded = $100;
   bp_TZvtTerm_shaded = 8;
   bm_TZvtTerm_swap_del_key = $200;
   bp_TZvtTerm_swap_del_key = 9;
   bm_TZvtTerm_del_is_del = $400;
   bp_TZvtTerm_del_is_del = 10;

function cursor_on(var a : TZvtTerm) : dword;
procedure set_cursor_on(var a : TZvtTerm; __cursor_on : dword);
function cursor_filled(var a : TZvtTerm) : dword;
procedure set_cursor_filled(var a : TZvtTerm; __cursor_filled : dword);
function cursor_blink_state(var a : TZvtTerm) : dword;
procedure set_cursor_blink_state(var a : TZvtTerm; __cursor_blink_state : dword);
function blink_enabled(var a : TZvtTerm) : dword;
procedure set_blink_enabled(var a : TZvtTerm; __blink_enabled : dword);
function in_expose(var a : TZvtTerm) : dword;
procedure set_in_expose(var a : TZvtTerm; __in_expose : dword);
function scroll_on_keystroke(var a : TZvtTerm) : dword;
procedure set_scroll_on_keystroke(var a : TZvtTerm; __scroll_on_keystroke : dword);
function scroll_on_output(var a : TZvtTerm) : dword;
procedure set_scroll_on_output(var a : TZvtTerm; __scroll_on_output : dword);
function transparent(var a : TZvtTerm) : dword;
procedure set_transparent(var a : TZvtTerm; __transparent : dword);
function shaded(var a : TZvtTerm) : dword;
procedure set_shaded(var a : TZvtTerm; __shaded : dword);
function swap_del_key(var a : TZvtTerm) : dword;
procedure set_swap_del_key(var a : TZvtTerm; __swap_del_key : dword);
function del_is_del(var a : TZvtTerm) : dword;
procedure set_del_is_del(var a : TZvtTerm; __del_is_del : dword);

type
   PZvtTermClass = ^TZvtTermClass;
   TZvtTermClass = record
        parent_class : TGtkWidgetClass;
        child_died : procedure (term:PZvtTerm);cdecl;
        title_changed : procedure (term:PZvtTerm; thetype:TVTTITLE_TYPE; newtitle:Pchar);
        match_clicked : procedure (term:PZvtTerm; event:PGdkEventButton; match:Pchar; data:pointer);
     end;
   ZVT_TERM_CLASS = PZvtTermClass;

   Pzvtfont_t = ^Tzvtfont_t;
   Tzvtfont_t = (ZVT_FONT_1BYTE := 0,ZVT_FONT_2BYTE,ZVT_FONT_FONTSET);

const
   ZVT_TERM_DO_UTMP_LOG = 1;
   ZVT_TERM_DO_WTMP_LOG = 2;
   ZVT_TERM_DO_LASTLOG = 4;
   ZVT_BACKGROUND_SHADED = $01;
   ZVT_BACKGROUND_SCROLL = $02;

function ZVT_TYPE_TERM : TGTKType;
function ZVT_IS_TERM(obj : Pointer) : Boolean;
function ZVT_IS_TERM_CLASS(klass : Pointer) : Boolean;


function ZVT_GTK_TOPWGT(t : PGTKWidget) : PGTKWidget;
function ZVT_GTK_WINDOW(t : PGTKWidget) : PGTKWindow;
function ZVT_GDK_WINDOW(t : PGTKWidget) : PGDKWindow;
function ZVT_GDK_TOPWIN(t : PGTKWidget) : PGDKWindow;

function zvt_term_new:PGtkWidget;cdecl;external libzvtdll name 'zvt_term_new';
function zvt_term_new_with_size(cols:longint; rows:longint):PGtkWidget;cdecl;external libzvtdll name 'zvt_term_new_with_size';
procedure zvt_term_reset(term:PZvtTerm; hard:longint);cdecl;external libzvtdll name 'zvt_term_reset';
procedure zvt_term_feed(term:PZvtTerm; thetext:Pchar; len:longint);cdecl;external libzvtdll name 'zvt_term_feed';
function zvt_term_writechild(term:PZvtTerm; data:Pchar; len:longint):longint;cdecl;external libzvtdll name 'zvt_term_writechild';
function zvt_term_forkpty(term:PZvtTerm; do_uwtmp_log:longint):longint;cdecl;external libzvtdll name 'zvt_term_forkpty';
function zvt_term_closepty(term:PZvtTerm):longint;cdecl;external libzvtdll name 'zvt_term_closepty';
function zvt_term_killchild(term:PZvtTerm; signal:longint):longint;cdecl;external libzvtdll name 'zvt_term_killchild';
procedure zvt_term_bell(zvt_term:pointer);cdecl;external libzvtdll name 'zvt_term_bell';
function zvt_term_get_type:guint;cdecl;external libzvtdll name 'zvt_term_get_type';
procedure zvt_term_set_scrollback(term:PZvtTerm; lines:longint);cdecl;external libzvtdll name 'zvt_term_set_scrollback';
function zvt_term_get_buffer(term:PZvtTerm; len:Plongint; thetype:longint; sx:longint; sy:longint;
           ex:longint; ey:longint):Pchar;cdecl;external libzvtdll name 'zvt_term_get_buffer';
procedure zvt_term_set_font_name(term:PZvtTerm; thename:Pchar);cdecl;external libzvtdll name 'zvt_term_set_font_name';
procedure zvt_term_set_fonts(term:PZvtTerm; font:PGdkFont; font_bold:PGdkFont);cdecl;external libzvtdll name 'zvt_term_set_fonts';
procedure zvt_term_hide_pointer(term:PZvtTerm);cdecl;external libzvtdll name 'zvt_term_hide_pointer';
procedure zvt_term_show_pointer(term:PZvtTerm);cdecl;external libzvtdll name 'zvt_term_show_pointer';
procedure zvt_term_set_bell(term:PZvtTerm; state:gboolean);cdecl;external libzvtdll name 'zvt_term_set_bell';
function zvt_term_get_bell(term:PZvtTerm):gboolean;cdecl;external libzvtdll name 'zvt_term_get_bell';
procedure zvt_term_set_blink(term:PZvtTerm; state:gboolean);cdecl;external libzvtdll name 'zvt_term_set_blink';
procedure zvt_term_set_scroll_on_keystroke(term:PZvtTerm; state:gboolean);cdecl;external libzvtdll name 'zvt_term_set_scroll_on_keystroke';
procedure zvt_term_set_scroll_on_output(term:PZvtTerm; state:gboolean);cdecl;external libzvtdll name 'zvt_term_set_scroll_on_output';
procedure zvt_term_set_color_scheme(term:PZvtTerm; red:Pgushort; grn:Pgushort; blu:Pgushort);cdecl;external libzvtdll name 'zvt_term_set_color_scheme';
procedure zvt_term_set_default_color_scheme(term:PZvtTerm);cdecl;external libzvtdll name 'zvt_term_set_default_color_scheme';
procedure zvt_term_set_del_key_swap(term:PZvtTerm; state:longint);cdecl;external libzvtdll name 'zvt_term_set_del_key_swap';
procedure zvt_term_set_del_is_del(term:PZvtTerm; state:longint);cdecl;external libzvtdll name 'zvt_term_set_del_is_del';
procedure zvt_term_set_wordclass(term:PZvtTerm; klass:Pbyte);cdecl;external libzvtdll name 'zvt_term_set_wordclass';
function zvt_term_match_add(term:PZvtTerm; regex:Pchar; highlight_mask:uint32; data:pointer):longint;cdecl;external libzvtdll name 'zvt_term_match_add';
procedure zvt_term_match_clear(term:PZvtTerm; regex:Pchar);cdecl;external libzvtdll name 'zvt_term_match_clear';
function zvt_term_match_check(term:PZvtTerm; x:longint; y:longint; user_data_ptr:Ppointer):Pchar;cdecl;external libzvtdll name 'zvt_term_match_check';
procedure zvt_term_set_background(terminal:PZvtTerm; pixmap_file:Pchar; transparent:gboolean; flags:longint);cdecl;external libzvtdll name 'zvt_term_set_background';
procedure zvt_term_set_shadow_type(term:PZvtTerm; thetype:TGtkShadowType);cdecl;external libzvtdll name 'zvt_term_set_shadow_type';
procedure zvt_term_set_size(term:PZvtTerm; width:guint; height:guint);cdecl;external libzvtdll name 'zvt_term_set_size';
procedure zvt_term_set_open_im(term:PZvtTerm; state:gboolean);cdecl;external libzvtdll name 'zvt_term_set_open_im';
function zvt_term_get_capabilities(term:PZvtTerm):guint32;cdecl;external libzvtdll name 'zvt_term_get_capabilities';

implementation

{$define read_implementation}
{$undef read_interface}

{$Include lists.inc}
{$Include vt.inc}
{$Include vtx.inc}

function ZVT_TYPE_TERM : TGTKType;
begin
  ZVT_TYPE_TERM:=zvt_term_get_type;
end;

function ZVT_IS_TERM(obj : Pointer) : Boolean;
begin
   ZVT_IS_TERM:=(obj<>nil) and ZVT_IS_TERM_CLASS(PGtkTypeObject(obj)^.klass);
end;

function ZVT_IS_TERM_CLASS(klass : Pointer) : Boolean;
begin
   ZVT_IS_TERM_CLASS:=(klass<>nil) and (PGtkTypeClass(klass)^.thetype=ZVT_TYPE_TERM);
end;

function ZVT_GTK_TOPWGT(t : PGTKWidget) : PGTKWidget;
begin
   ZVT_GTK_TOPWGT := gtk_widget_get_toplevel(t);
end;

function ZVT_GTK_WINDOW(t : PGTKWidget) : PGTKWindow;
begin
   ZVT_GTK_WINDOW := GTK_WINDOW(ZVT_GTK_TOPWGT(t));
end;

function ZVT_GDK_WINDOW(t : PGTKWidget) : PGDKWindow;
begin
   ZVT_GDK_WINDOW := t^.window;
end;

function ZVT_GDK_TOPWIN(t : PGTKWidget) : PGDKWindow;
begin
   ZVT_GDK_TOPWIN := ZVT_GTK_TOPWGT(t)^.window;
end;

function cursor_on(var a : TZvtTerm) : dword;
begin
   cursor_on:=(a.flag0 and bm_TZvtTerm_cursor_on) shr bp_TZvtTerm_cursor_on;
end;

procedure set_cursor_on(var a : TZvtTerm; __cursor_on : dword);
begin
   a.flag0:=a.flag0 or ((__cursor_on shl bp_TZvtTerm_cursor_on) and bm_TZvtTerm_cursor_on);
end;

function cursor_filled(var a : TZvtTerm) : dword;
begin
   cursor_filled:=(a.flag0 and bm_TZvtTerm_cursor_filled) shr bp_TZvtTerm_cursor_filled;
end;

procedure set_cursor_filled(var a : TZvtTerm; __cursor_filled : dword);
begin
   a.flag0:=a.flag0 or ((__cursor_filled shl bp_TZvtTerm_cursor_filled) and bm_TZvtTerm_cursor_filled);
end;

function cursor_blink_state(var a : TZvtTerm) : dword;
begin
   cursor_blink_state:=(a.flag0 and bm_TZvtTerm_cursor_blink_state) shr bp_TZvtTerm_cursor_blink_state;
end;

procedure set_cursor_blink_state(var a : TZvtTerm; __cursor_blink_state : dword);
begin
   a.flag0:=a.flag0 or ((__cursor_blink_state shl bp_TZvtTerm_cursor_blink_state) and bm_TZvtTerm_cursor_blink_state);
end;

function blink_enabled(var a : TZvtTerm) : dword;
begin
   blink_enabled:=(a.flag0 and bm_TZvtTerm_blink_enabled) shr bp_TZvtTerm_blink_enabled;
end;

procedure set_blink_enabled(var a : TZvtTerm; __blink_enabled : dword);
begin
   a.flag0:=a.flag0 or ((__blink_enabled shl bp_TZvtTerm_blink_enabled) and bm_TZvtTerm_blink_enabled);
end;

function in_expose(var a : TZvtTerm) : dword;
begin
   in_expose:=(a.flag0 and bm_TZvtTerm_in_expose) shr bp_TZvtTerm_in_expose;
end;

procedure set_in_expose(var a : TZvtTerm; __in_expose : dword);
begin
   a.flag0:=a.flag0 or ((__in_expose shl bp_TZvtTerm_in_expose) and bm_TZvtTerm_in_expose);
end;

function scroll_on_keystroke(var a : TZvtTerm) : dword;
begin
   scroll_on_keystroke:=(a.flag0 and bm_TZvtTerm_scroll_on_keystroke) shr bp_TZvtTerm_scroll_on_keystroke;
end;

procedure set_scroll_on_keystroke(var a : TZvtTerm; __scroll_on_keystroke : dword);
begin
   a.flag0:=a.flag0 or ((__scroll_on_keystroke shl bp_TZvtTerm_scroll_on_keystroke) and bm_TZvtTerm_scroll_on_keystroke);
end;

function scroll_on_output(var a : TZvtTerm) : dword;
begin
   scroll_on_output:=(a.flag0 and bm_TZvtTerm_scroll_on_output) shr bp_TZvtTerm_scroll_on_output;
end;

procedure set_scroll_on_output(var a : TZvtTerm; __scroll_on_output : dword);
begin
   a.flag0:=a.flag0 or ((__scroll_on_output shl bp_TZvtTerm_scroll_on_output) and bm_TZvtTerm_scroll_on_output);
end;

function transparent(var a : TZvtTerm) : dword;
begin
   transparent:=(a.flag0 and bm_TZvtTerm_transparent) shr bp_TZvtTerm_transparent;
end;

procedure set_transparent(var a : TZvtTerm; __transparent : dword);
begin
   a.flag0:=a.flag0 or ((__transparent shl bp_TZvtTerm_transparent) and bm_TZvtTerm_transparent);
end;

function shaded(var a : TZvtTerm) : dword;
begin
   shaded:=(a.flag0 and bm_TZvtTerm_shaded) shr bp_TZvtTerm_shaded;
end;

procedure set_shaded(var a : TZvtTerm; __shaded : dword);
begin
   a.flag0:=a.flag0 or ((__shaded shl bp_TZvtTerm_shaded) and bm_TZvtTerm_shaded);
end;

function swap_del_key(var a : TZvtTerm) : dword;
begin
   swap_del_key:=(a.flag0 and bm_TZvtTerm_swap_del_key) shr bp_TZvtTerm_swap_del_key;
end;

procedure set_swap_del_key(var a : TZvtTerm; __swap_del_key : dword);
begin
   a.flag0:=a.flag0 or ((__swap_del_key shl bp_TZvtTerm_swap_del_key) and bm_TZvtTerm_swap_del_key);
end;

function del_is_del(var a : TZvtTerm) : dword;
begin
   del_is_del:=(a.flag0 and bm_TZvtTerm_del_is_del) shr bp_TZvtTerm_del_is_del;
end;

procedure set_del_is_del(var a : TZvtTerm; __del_is_del : dword);
begin
   a.flag0:=a.flag0 or ((__del_is_del shl bp_TZvtTerm_del_is_del) and bm_TZvtTerm_del_is_del);
end;


end.
