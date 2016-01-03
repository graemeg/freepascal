program scribble_simple;

{$mode objfpc} {$H+}

uses glib2, gtk2, gdk2;

var
  pixmap : PGdkPixmap;  (* Backing pixmap for drawing area *)



(* Create a new backing pixmap of the appropriate size *)

function configure_event ( widget : PGtkWidget;
                           event  : PGdkEventConfigure): gboolean; cdecl;
begin

  if pixmap <> nil then
    g_object_unref (pixmap);

  pixmap := gdk_pixmap_new (widget^.window,
                                    widget^.allocation.width,
                                        widget^.allocation.height,
                            -1);
  gdk_draw_rectangle (pixmap,
                              widget^.style^.white_gc,
                              gTRUE,
                              0, 0,
                              widget^.allocation.width,
                              widget^.allocation.height);

  configure_event := TRUE;
end;

(* Redraw the screen from the backing pixmap *)
function expose_event ( widget : PGtkWidget;
                        event  : PGdkEventExpose) : gboolean; cdecl;
begin
  gdk_draw_drawable (widget^.window,
                             widget^.style^.fg_gc[GTK_WIDGET_STATE (widget)],
                     pixmap,
                     event^.area.x, event^.area.y,
                             event^.area.x, event^.area.y,
                             event^.area.width, event^.area.height);

  expose_event := FALSE;
end;


(* Draw a rectangle on the screen *)
procedure draw_brush (widget : PGtkWidget;
                      x, y   : gdouble); cdecl;
var
  update_rect : TGdkRectangle;

begin
  update_rect.x      := round (x - 5.0);
  update_rect.y      := round (y - 5.0);
  update_rect.width  := 10;
  update_rect.height := 10;

  gdk_draw_rectangle (pixmap,
                              widget^.style^.black_gc,
                          gTRUE,
                              update_rect.x, update_rect.y,
                              update_rect.width, update_rect.height);

  gtk_widget_queue_draw_area (widget,
                              update_rect.x, update_rect.y,
                                      update_rect.width, update_rect.height);
end;

function button_press_event ( widget : PGtkWidget;
                              event  : PGdkEventbutton): gboolean; cdecl;
begin
  if (event^.button = 1) and (pixmap <> NULL) then
    draw_brush (widget, event^.x, event^.y);

  button_press_event := TRUE;
end;

function motion_notify_event ( widget: PGtkWidget;
                               event : PGdkEventMotion): gboolean; cdecl;
var
  x, y  : gint;
  state : TGdkModifierType;

begin
  if event^.is_hint = gTRUE then
    gdk_window_get_pointer (event^.window, @x, @y, @state)
  else begin
    x := round (event^.x);
    y := round (event^.y);
    state := event^.state;
  end;

  if ((state and GDK_BUTTON1_MASK) <> 0) and (pixmap <> NULL) then
    draw_brush (widget, x, y);

  motion_notify_event := TRUE;
end;

procedure quit;
begin
  halt;
end;

var
  window,
  drawing_area,
  vbox           : PGtkWidget;

  button         : PGtkWidget;

begin
  gtk_init (@argc, @argv);

  window := gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_name (window, 'Test Input');

  vbox := gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox);
  gtk_widget_show (vbox);

  g_signal_connect (G_OBJECT (window), 'destroy',
                    G_CALLBACK (@quit), NULL);

  (* Create the drawing area *)

  drawing_area := gtk_drawing_area_new ();
  gtk_widget_set_size_request (GTK_WIDGET (drawing_area), 200, 200);
  gtk_box_pack_start (GTK_BOX (vbox), drawing_area, TRUE, TRUE, 0);

  gtk_widget_show (drawing_area);

  (* Signals used to handle backing pixmap *)

  g_signal_connect (G_OBJECT (drawing_area), 'expose_event',
                            G_CALLBACK (@expose_event), NULL);

  g_signal_connect (G_OBJECT (drawing_area),'configure_event',
                            G_CALLBACK (@configure_event), NULL);

  (* Event signals *)

  g_signal_connect (G_OBJECT (drawing_area), 'motion_notify_event',
                            G_CALLBACK (@motion_notify_event), NULL);

  g_signal_connect (G_OBJECT (drawing_area), 'button_press_event',
                            G_CALLBACK (@button_press_event), NULL);

  gtk_widget_set_events (drawing_area, GDK_EXPOSURE_MASK
                                                 or  GDK_LEAVE_NOTIFY_MASK
                                                 or  GDK_BUTTON_PRESS_MASK
                                                 or  GDK_POINTER_MOTION_MASK
                                     or  GDK_POINTER_MOTION_HINT_MASK);

  (* .. And a quit button *)
  button := gtk_button_new_with_label ('Quit');
  gtk_box_pack_start (GTK_BOX (vbox), button, FALSE, FALSE, 0);

  g_signal_connect_swapped (G_OBJECT (button), 'clicked',
                            G_CALLBACK (@gtk_widget_destroy),
                            window);
  gtk_widget_show (button);

  gtk_widget_show (window);

  gtk_main ();
end.
