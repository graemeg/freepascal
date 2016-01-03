(* Images
 *
 * GtkImage is used to display an image; the image can be in a number of formats.
 * Typically, you load an image into a GdkPixbuf, then display the pixbuf.
 *
 * This demo code shows some of the more obscure cases, in the simple
 * case a call to gtk_image_new_from_file() is all you need.
 *
 * If you want to put image data in your program as a C variable,
 * use the make-inline-pixbuf program that comes with GTK+.
 * This way you won't need to depend on loading external files, your
 * application binary can be self-contained.
 *)


var
  image_window        : PGtkWidget;
  image_pixbuf_loader : PGdkPixbufLoader;
  image_load_timeout  : guint;
  image_stream        : file;

type
  TBuffer256  = array [0..255] of byte;

procedure progressive_prepared_callback (loader : PGdkPixbufLoader;
                                         data   : gpointer); cdecl;
var
  pixbuf : PGdkPixbuf;
  image  : PGtkWidget;

begin
  image := PGtkWidget (data);

  pixbuf := gdk_pixbuf_loader_get_pixbuf (loader);

  (* Avoid displaying random memory contents, since the pixbuf
   * isn't filled in yet.
   *)
  gdk_pixbuf_fill (pixbuf, $aaaaaaff);

  gtk_image_set_from_pixbuf (pGtkImage (image), pixbuf);
end;

procedure progressive_updated_callback (loader : PGdkPixbufLoader;
                                        x, y,
                                        width,
                                        height : gint;
                                        data   : gpointer); cdecl;
var
  image : PGtkWidget;

begin
  image := PGtkWidget (data);

  (* We know the pixbuf inside the GtkImage has changed, but the image
   * itself doesn't know this; so queue a redraw.  If we wanted to be
   * really efficient, we could use a drawing area or something
   * instead of a GtkImage, so we could control the exact position of
   * the pixbuf on the display, then we could queue a draw for only
   * the updated area of the image.
   *)

  gtk_widget_queue_draw (image);
end;

function progressive_timeout (data : gpointer): gboolean; cdecl;
var
  image      : PGtkWidget;
  buf        : TBuffer256;
  bytes_read : integer;
  error      : PGError;
  dialog     : PGtkWidget;
  error_msg,
  filename   : pgchar;

begin
  image := PGtkWidget (data);

  (* This shows off fully-paranoid error handling, so looks scary.
   * You could factor out the error handling code into a nice separate
   * function to make things nicer.
   *)

  if file_is_valid (image_stream)  then   // is there a better way???
  begin
    error := NULL;

    blockread (image_stream, buf, sizeof(buf), bytes_read);

    if not gdk_pixbuf_loader_write (image_pixbuf_loader,
                                    @buf[0], bytes_read, @error) then
    begin
          dialog := gtk_message_dialog_new (GTK_WINDOW (image_window),
                       GTK_DIALOG_DESTROY_WITH_PARENT,
                                           GTK_MESSAGE_ERROR,
                                           GTK_BUTTONS_CLOSE,
                                           'Failed to load image: %s',
                                           [error^.message]);

          g_error_free (error);

          g_signal_connect (dialog, 'response',
                            G_CALLBACK (@gtk_widget_destroy), NULL);

          close (image_stream);

          gtk_widget_show (dialog);

          image_load_timeout := 0;

          exit (FALSE); (* uninstall the timeout *)
    end; {of not gdk_pixbuf_loader_write}

    if eof (image_stream) then
    begin
          close (image_stream);

         (* Errors can happen on close, e.g. if the image
          * file was truncated we'll know on close that
          * it was incomplete.
          *)
          error := NULL;
          if not gdk_pixbuf_loader_close (image_pixbuf_loader, @error) then
      begin
            dialog := gtk_message_dialog_new (GTK_WINDOW (image_window),
                                               GTK_DIALOG_DESTROY_WITH_PARENT,
                                               GTK_MESSAGE_ERROR,
                                               GTK_BUTTONS_CLOSE,
                                               'Failed to load image: %s',
                                               [error^.message]);

        g_error_free (error);

        g_signal_connect (dialog, 'response',
                                G_CALLBACK (@gtk_widget_destroy), NULL);

        gtk_widget_show (dialog);

        g_object_unref (G_OBJECT (image_pixbuf_loader));
        image_pixbuf_loader := NULL;

        image_load_timeout := 0;

            exit(FALSE); (* uninstall the timeout *)
      end;  {of not gdk_pixbuf_loader_close}

          g_object_unref (G_OBJECT (image_pixbuf_loader));
          image_pixbuf_loader := NULL;
    end; {of eof}
  end {of image_stream}
  else begin
    error_msg := NULL;

    (* demo_find_file() looks in the the current directory first,
     * so you can run gtk-demo without installing GTK, then looks
     * in the location where the file is installed.
     *)

    filename := demo_find_file ('alphatest.png', @error);
    if error <> NULL then
    begin
      error_msg := g_strdup (error^.message);
          g_error_free (error);
    end else
    begin
      {$I-}
      assign (image_stream, filename);
      reset (image_stream, 1);
      {$I+}

          if IOResult <> 0 then
            error_msg := g_strdup_printf ('Error while opening file "%s"',
                                                              [filename]);
          g_free (filename);
    end;

    if not file_is_valid (image_stream) then
    begin
          dialog := gtk_message_dialog_new (GTK_WINDOW (image_window),
                                           GTK_DIALOG_DESTROY_WITH_PARENT,
                                           GTK_MESSAGE_ERROR,
                                           GTK_BUTTONS_CLOSE,
                                           '%s', [error_msg]);
          g_free (error_msg);

          g_signal_connect (dialog, 'response',
                            G_CALLBACK (@gtk_widget_destroy), NULL);

          gtk_widget_show (dialog);

          image_load_timeout := 0;

          exit (FALSE); (* uninstall the timeout *)
    end;

    if image_pixbuf_loader <> NULL then
    begin
          gdk_pixbuf_loader_close (image_pixbuf_loader, NULL);
          g_object_unref (G_OBJECT (image_pixbuf_loader));
          image_pixbuf_loader := NULL;
    end;

    image_pixbuf_loader := gdk_pixbuf_loader_new ();

    g_signal_connect (G_OBJECT (image_pixbuf_loader), 'area_prepared',
                        G_CALLBACK (@progressive_prepared_callback), image);

    g_signal_connect (G_OBJECT (image_pixbuf_loader), 'area_updated',
                        G_CALLBACK (@progressive_updated_callback), image);
  end; {of else image_stream}

  (* leave timeout installed *)
  exit (TRUE);
end;

procedure start_progressive_loading (image  : PGtkWidget); cdecl;
begin
  (* This is obviously totally contrived (we slow down loading
   * on purpose to show how incremental loading works).
   * The real purpose of incremental loading is the case where
   * you are reading data from a slow source such as the network.
   * The timeout simply simulates a slow data source by inserting
   * pauses in the reading process.
   *)
  image_load_timeout := g_timeout_add (150,
                                @progressive_timeout,
                                image);
end;

procedure images_cleanup_callback (theobject : PGtkObject;
                            data      : gpointer); cdecl;
begin
  if image_load_timeout <> 0 then
  begin
    g_source_remove (image_load_timeout);
    image_load_timeout := 0;
  end;

  if image_pixbuf_loader <> NULL then
  begin
    gdk_pixbuf_loader_close (image_pixbuf_loader, NULL);
    g_object_unref (G_OBJECT (image_pixbuf_loader));
    image_pixbuf_loader := NULL;
  end;

  if  file_is_valid (image_stream) then
    close (image_stream);
end;

procedure toggle_sensitivity_callback (togglebutton : PGtkWidget;
                                       user_data    : gpointer);cdecl;
var
  container : PGtkContainer;
  list,
  tmp       : PGList;

begin
  container := PGtkContainer (user_data);

  list := gtk_container_get_children (container);

  tmp := list;

  while tmp <> NULL do
  begin
    (* don't disable our toggle *)
    if pGtkWidget (tmp^.data) <> togglebutton then
      gtk_widget_set_sensitive (pGtkWidget (tmp^.data),
                                not gtk_toggle_button_get_active (pGtkToggleButton (togglebutton)));

      tmp := tmp^.next;
  end;

  g_list_free (list);
end;


function do_images         : PGtkWidget;
var
  frame,
  vbox,
  image,
  thelabel,
  align,
  dialog,
  button     : PGtkWidget;

  pixbuf     : PGdkPixbuf;
  error      : PGError;
  filename   : pgchar;

begin
  error := NULL;

  if image_window = NULL then
  begin
    image_window := gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (image_window), 'Images');

    g_signal_connect (image_window, 'destroy',
                        G_CALLBACK (@gtk_widget_destroyed), @image_window);
    g_signal_connect (image_window, 'destroy',
                        G_CALLBACK (@images_cleanup_callback), NULL);

    gtk_container_set_border_width (GTK_CONTAINER (image_window), 8);

    vbox := gtk_vbox_new (FALSE, 8);
    gtk_container_set_border_width (GTK_CONTAINER (vbox), 8);
    gtk_container_add (GTK_CONTAINER (image_window), vbox);

    thelabel := gtk_label_new (NULL);
    gtk_label_set_markup (GTK_LABEL (thelabel),
                            '<u>Image loaded from a file</u>');

    gtk_box_pack_start (GTK_BOX (vbox), thelabel, FALSE, FALSE, 0);

    frame := gtk_frame_new (NULL);
    gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);

    (* The alignment keeps the frame from growing when users resize
     * the window
     *)
    align := gtk_alignment_new (0.5, 0.5, 0, 0);
    gtk_container_add (GTK_CONTAINER (align), frame);
    gtk_box_pack_start (GTK_BOX (vbox), align, FALSE, FALSE, 0);

    (* demo_find_file() looks in the the current directory first,
     * so you can run gtk-demo without installing GTK, then looks
     * in the location where the file is installed.
     *)

    pixbuf := NULL;
    filename := demo_find_file ('gtk-logo-rgb.gif', @error);
    if filename <> NULL then
    begin
          pixbuf := gdk_pixbuf_new_from_file (filename, @error);
          g_free (filename);
    end;

    if error <> NULL then
    begin
          (* This code shows off error handling. You can just use
           * gtk_image_new_from_file() instead if you don't want to report
           * errors to the user. If the file doesn't load when using
           * gtk_image_new_from_file(), a "missing image" icon will
           * be displayed instead.
           *)

          dialog := gtk_message_dialog_new (GTK_WINDOW (image_window),
                                           GTK_DIALOG_DESTROY_WITH_PARENT,
                                           GTK_MESSAGE_ERROR,
                                           GTK_BUTTONS_CLOSE,
                                           'Unable to open image file "gtk-logo-rgb.gif": %s',
                                           [error^.message]);
          g_error_free (error);

          g_signal_connect (dialog, 'response',
                            G_CALLBACK (@gtk_widget_destroy), NULL);

          gtk_widget_show (dialog);
    end;

    image := gtk_image_new_from_pixbuf (pixbuf);

    gtk_container_add (GTK_CONTAINER (frame), image);


    (* Animation *)

    thelabel := gtk_label_new (NULL);
    gtk_label_set_markup (GTK_LABEL (thelabel),
                            '<u>Animation loaded from a file</u>');
    gtk_box_pack_start (GTK_BOX (vbox), thelabel, FALSE, FALSE, 0);

    frame := gtk_frame_new (NULL);
    gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);

    (* The alignment keeps the frame from growing when users resize
     * the window
     *)
    align := gtk_alignment_new (0.5, 0.5, 0, 0);
    gtk_container_add (GTK_CONTAINER (align), frame);
    gtk_box_pack_start (GTK_BOX (vbox), align, FALSE, FALSE, 0);

    filename := demo_find_file ('floppybuddy.gif', NULL);
    image := gtk_image_new_from_file (filename);
    g_free (filename);

    gtk_container_add (GTK_CONTAINER (frame), image);


    (* Progressive *)


    thelabel := gtk_label_new (NULL);
    gtk_label_set_markup (GTK_LABEL (thelabel),
                            '<u>Progressive image loading</u>');
    gtk_box_pack_start (GTK_BOX (vbox), thelabel, FALSE, FALSE, 0);

    frame := gtk_frame_new (NULL);
    gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);

    (* The alignment keeps the frame from growing when users resize
     * the window
     *)
    align := gtk_alignment_new (0.5, 0.5, 0, 0);
    gtk_container_add (GTK_CONTAINER (align), frame);
    gtk_box_pack_start (GTK_BOX (vbox), align, FALSE, FALSE, 0);

    (* Create an empty image for now; the progressive loader
     * will create the pixbuf and fill it in.
     *)

    image := gtk_image_new_from_pixbuf (NULL);
    gtk_container_add (GTK_CONTAINER (frame), image);

    start_progressive_loading (image);

    (* Sensitivity control *)
    button := gtk_toggle_button_new_with_mnemonic ('_Insensitive');
    gtk_box_pack_start (GTK_BOX (vbox), button, FALSE, FALSE, 0);

    g_signal_connect (G_OBJECT (button), 'toggled',
                        G_CALLBACK (@toggle_sensitivity_callback),
                        vbox);
  end;

  if not GTK_WIDGET_VISIBLE (image_window) then
    gtk_widget_show_all (image_window)
  else
  begin
    gtk_widget_destroy (image_window);
    image_window := NULL;
  end;

  do_images := image_window;
end;

