{%MainUnit ../gdk2x.pas}

{$IFDEF read_interface_rest}
var
   gdk_display : PDisplay;cvar;external;

function gdk_x11_drawable_get_xdisplay(drawable:PGdkDrawable):PDisplay;cdecl;external;
function gdk_x11_drawable_get_xid(drawable:PGdkDrawable):TXID;cdecl;external;
function gdk_x11_image_get_xdisplay(image:PGdkImage):PDisplay;cdecl;external;
function gdk_x11_image_get_ximage(image:PGdkImage):PXImage;cdecl;external;
function gdk_x11_colormap_get_xdisplay(colormap:PGdkColormap):PDisplay;cdecl;external;
function gdk_x11_colormap_get_xcolormap(colormap:PGdkColormap):TColormap;cdecl;external;
function gdk_x11_cursor_get_xdisplay(cursor:PGdkCursor):PDisplay;cdecl;external;
function gdk_x11_cursor_get_xcursor(cursor:PGdkCursor):TCursor;cdecl;external;
function gdk_x11_display_get_xdisplay(display:PGdkDisplay):PDisplay;cdecl;external;
function gdk_x11_visual_get_xvisual(visual:PGdkVisual):PVisual;cdecl;external;
function gdk_x11_gc_get_xdisplay(gc:PGdkGC):PDisplay;cdecl;external;
function gdk_x11_gc_get_xgc(gc:PGdkGC):TGC;cdecl;external;
function gdk_x11_screen_get_xscreen(screen:PGdkScreen):PScreen;cdecl;external;
function gdk_x11_screen_get_screen_number(screen:PGdkScreen):longint;cdecl;external;

{$ifndef GDK_MULTIHEAD_SAFE}

function gdk_x11_get_default_root_xwindow:TWindow;cdecl;external;
function gdk_x11_get_default_xdisplay:PDisplay;cdecl;external;
function gdk_x11_get_default_screen:gint;cdecl;external;

{$endif}
function GDK_COLORMAP_XDISPLAY(cmap : PGdkColormap) : PDisplay;
function GDK_COLORMAP_XCOLORMAP(cmap : PGdkColormap) : TColormap;
function GDK_CURSOR_XDISPLAY(cursor : PGdkCursor) : PDisplay;
function GDK_CURSOR_XCURSOR(cursor : PGdkCursor) : TCursor;
function GDK_IMAGE_XDISPLAY(image : PGdkImage) : PDisplay;
function GDK_IMAGE_XIMAGE(image : PGdkImage) : PXImage;

{$ifndef GDK_MULTIHEAD_SAFE}
function GDK_ROOT_WINDOW : TWindow;
{$endif}

function GDK_DISPLAY_XDISPLAY(display : PGdkDisplay) : PDisplay;
function GDK_WINDOW_XDISPLAY(win : PGdkDrawable) : PDisplay;
function GDK_WINDOW_XID(win : PGdkDrawable) : TXID;
function GDK_WINDOW_XWINDOW(win : PGdkDrawable) : TXID;
function GDK_PIXMAP_XDISPLAY(win : PGdkDrawable) : PDisplay;
function GDK_PIXMAP_XID(win : PGdkDrawable) : TXID;
function GDK_DRAWABLE_XDISPLAY(win : PGdkDrawable) : PDisplay;
function GDK_DRAWABLE_XID(win : PGdkDrawable) : TXID;
function GDK_GC_XDISPLAY(gc : PGdkGC) : PDisplay;
function GDK_GC_XGC(gc : PGdkGC) : TGC;
function GDK_SCREEN_XDISPLAY(screen : PGdkScreen) : PDisplay;
function GDK_SCREEN_XSCREEN(screen : PGdkScreen) : PScreen;
function GDK_SCREEN_XNUMBER(screen : PGdkScreen) : longint;
function GDK_VISUAL_XVISUAL(visual : PGdkVisual) : PVisual;

function gdkx_visual_get_for_screen(screen:PGdkScreen; xvisualid:TVisualID):PGdkVisual;cdecl;external;

{$ifndef GDK_MULTIHEAD_SAFE}

function gdkx_visual_get(xvisualid:TVisualID):PGdkVisual;cdecl;external;

{$endif}
{ XXX: Do not use this function until it is fixed. An X Colormap
        is useless unless we also have the visual.  }
function gdkx_colormap_get(xcolormap:TColormap):PGdkColormap;cdecl;external;

{ Return the Gdk  for a particular XID  }
function gdk_xid_table_lookup_for_display(display:PGdkDisplay; xid:TXID):gpointer;cdecl;external;

function gdk_x11_get_server_time(window:PGdkWindow):guint32;cdecl;external;

{ returns TRUE if we support the given WM spec feature  }
function gdk_x11_screen_supports_net_wm_hint(screen:PGdkScreen; _property:TGdkAtom):gboolean;cdecl;external;

{$ifndef GDK_MULTIHEAD_SAFE}

function gdk_xid_table_lookup(xid:TXID):gpointer;cdecl;external;
function gdk_net_wm_supports(_property:TGdkAtom):gboolean;cdecl;external;
procedure gdk_x11_grab_server;cdecl;external;
procedure gdk_x11_ungrab_server;cdecl;external;

{$endif}

function gdk_x11_lookup_xdisplay(xdisplay:PDisplay):PGdkDisplay;cdecl;external;

{ Functions to get the X Atom equivalent to the GdkAtom  }
function gdk_x11_atom_to_xatom_for_display(display:PGdkDisplay; atom:TGdkAtom):TAtom;cdecl;external;

function gdk_x11_xatom_to_atom_for_display(display:PGdkDisplay; xatom:TAtom):TGdkAtom;cdecl;external;

(* Const before type ignored *)
function gdk_x11_get_xatom_by_name_for_display(display:PGdkDisplay; atom_name:Pgchar):TAtom;cdecl;external;

(* Const before type ignored *)
function gdk_x11_get_xatom_name_for_display(display:PGdkDisplay; xatom:TAtom):Pgchar;cdecl;external;

{$ifndef GDK_MULTIHEAD_SAFE}
function gdk_x11_atom_to_xatom(atom:TGdkAtom):TAtom;cdecl;external;
function gdk_x11_xatom_to_atom(xatom:TAtom):TGdkAtom;cdecl;external;
(* Const before type ignored *)
function gdk_x11_get_xatom_by_name(atom_name:Pgchar):TAtom;cdecl;external;
(* Const before type ignored *)
function gdk_x11_get_xatom_name(xatom:TAtom):Pgchar;cdecl;external;
{$endif}

{$ifdef GDK_ENABLE_DEPRECATED}
function gdk_x11_font_get_xdisplay(font:PGdkFont):PDisplay;cdecl;external;
function gdk_x11_font_get_xfont(font:PGdkFont):gpointer;cdecl;external;
(* Const before type ignored *)
function gdk_x11_font_get_name(font:PGdkFont):Pchar;cdecl;external;
function GDK_FONT_XDISPLAY(font : PGdkFont): PDisplay;
function GDK_FONT_XFONT(font : PGdkFont): gpointer;

{$ifndef GDK_MULTIHEAD_SAFE}
function gdk_font_lookup(xid : TXID) : PGdkFont;
{$endif}

function gdk_font_lookup_for_display(display:PGdkDisplay; xid:TXID) : PGdkFont;

procedure gdk_x11_display_grab(display:PGdkDisplay);cdecl;external;

procedure gdk_x11_display_ungrab(display:PGdkDisplay);cdecl;external;
{$endif GDK_ENABLE_DEPRECATED}


{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}


function GDK_COLORMAP_XDISPLAY(cmap : PGdkColormap) : PDisplay;
begin
   GDK_COLORMAP_XDISPLAY:=gdk_x11_colormap_get_xdisplay(cmap);
end;

function GDK_COLORMAP_XCOLORMAP(cmap : PGdkColormap) : TColormap;
begin
   GDK_COLORMAP_XCOLORMAP:=gdk_x11_colormap_get_xcolormap(cmap);
end;

function GDK_CURSOR_XDISPLAY(cursor : PGdkCursor) : PDisplay;
begin
   GDK_CURSOR_XDISPLAY:=gdk_x11_cursor_get_xdisplay(cursor);
end;

function GDK_CURSOR_XCURSOR(cursor : PGdkCursor) : TCursor;
begin
   GDK_CURSOR_XCURSOR:=gdk_x11_cursor_get_xcursor(cursor);
end;

function GDK_IMAGE_XDISPLAY(image : PGdkImage) : PDisplay;
begin
   GDK_IMAGE_XDISPLAY:=gdk_x11_image_get_xdisplay(image);
end;

function GDK_IMAGE_XIMAGE(image : PGdkImage) : PXImage;
begin
   GDK_IMAGE_XIMAGE:=gdk_x11_image_get_ximage(image);
end;

function GDK_DRAWABLE_XDISPLAY(win : PGdkDrawable) : PDisplay;
var
   if_local1 : PDisplay;
begin
   if GDK_IS_WINDOW(win) then
     if_local1:=GDK_WINDOW_XDISPLAY(win)
   else
     if_local1:=GDK_PIXMAP_XDISPLAY(win);
   GDK_DRAWABLE_XDISPLAY:=if_local1;
end;

function GDK_DRAWABLE_XID(win : PGdkDrawable) : TWindow;
var
   if_local1 : TWindow;
begin
   if GDK_IS_WINDOW(win) then
     if_local1:=GDK_WINDOW_XID(win)
   else
     if_local1:=GDK_PIXMAP_XID(win);
   GDK_DRAWABLE_XID:=if_local1;
end;

function GDK_GC_XDISPLAY(gc : PGdkGC) : PDisplay;
begin
   GDK_GC_XDISPLAY:=GDK_SCREEN_XDISPLAY((GDK_GC_X11(gc))^.screen);
end;

function GDK_GC_XGC(gc : PGdkGC) : TGC;
begin
   GDK_GC_XGC:=(GDK_GC_X11(gc))^.xgc;
end;

function GDK_SCREEN_XSCREEN(screen : PGdkScreen) : PScreen;
begin
   GDK_SCREEN_XSCREEN:=(GDK_SCREEN_X11(screen))^.xscreen;
end;

function GDK_SCREEN_XNUMBER(screen : PGdkScreen) : longint;
begin
   GDK_SCREEN_XNUMBER:=(GDK_SCREEN_X11(screen))^.screen_num;
end;

function GDK_VISUAL_XVISUAL(visual : PGdkVisual) : PVisual;
begin
   GDK_VISUAL_XVISUAL:=(PGdkVisualPrivate(visual))^.xvisual;
end;

function GDK_GC_GET_XGC(gc : PGdkGC) : TGC;
var
   if_local1 : TGC;
begin
   if_local1:=(PGdkGCX11(gc))^.xgc;
   if (GDK_GC_X11(gc))^.dirty_mask=gTRUE then begin
     //if_local1:=_gdk_x11_gc_flush(gc)
   end;
   GDK_GC_GET_XGC:=if_local1;
end;

function GDK_ROOT_WINDOW : TWindow;
begin
   GDK_ROOT_WINDOW:=gdk_x11_get_default_root_xwindow;
end;

function GDK_DISPLAY_XDISPLAY(display : PGdkDisplay) : PDisplay;
begin
   GDK_DISPLAY_XDISPLAY:=gdk_x11_display_get_xdisplay(display);
end;

function GDK_WINDOW_XDISPLAY(win : PGdkDrawable) : PDisplay;
begin
   GDK_WINDOW_XDISPLAY:=gdk_x11_drawable_get_xdisplay((PGdkWindowObject(win))^.impl);
end;

function GDK_WINDOW_XID(win : PGdkDrawable) : TXID;
begin
   GDK_WINDOW_XID:=gdk_x11_drawable_get_xid(win);
end;

function GDK_WINDOW_XWINDOW(win : PGdkDrawable) : TXID;
begin
   GDK_WINDOW_XWINDOW:=gdk_x11_drawable_get_xid(win);
end;

function GDK_PIXMAP_XDISPLAY(win : PGdkDrawable) : PDisplay;
begin
   GDK_PIXMAP_XDISPLAY:=gdk_x11_drawable_get_xdisplay((PGdkPixmapObject(win))^.impl);
end;

function GDK_PIXMAP_XID(win : PGdkDrawable) : TXID;
begin
   GDK_PIXMAP_XID:=gdk_x11_drawable_get_xid(win);
end;

function GDK_SCREEN_XDISPLAY(screen : PGdkScreen) : PDisplay;
begin
   GDK_SCREEN_XDISPLAY:=gdk_x11_display_get_xdisplay(gdk_screen_get_display(screen));
end;

{$ifdef GDK_ENABLE_DEPRECATED}
function GDK_FONT_XDISPLAY(font : PGdkFont): PDisplay;
begin
   GDK_FONT_XDISPLAY:=gdk_x11_font_get_xdisplay(font);
end;

function gdk_font_lookup(xid : TXID) : PGdkFont;
begin
   gdk_font_lookup:=PGdkFont(gdk_xid_table_lookup(xid));
end;

function GDK_FONT_XFONT(font : PGdkFont): gpointer;
begin
   GDK_FONT_XFONT:=gdk_x11_font_get_xfont(font);
end;

function gdk_font_lookup_for_display(display:PGdkDisplay; xid:TXID) : PGdkFont;
begin
   gdk_font_lookup_for_display:=PGdkFont(gdk_xid_table_lookup_for_display(display,xid));
end;
{$ENDIF GDK_ENABLE_DEPRECATED}


{$ENDIF read_implementation}

