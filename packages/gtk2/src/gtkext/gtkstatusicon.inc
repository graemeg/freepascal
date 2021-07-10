
procedure Freegtkstatusicon;
begin
  gtk_status_icon_get_type:=nil;
  gtk_status_icon_new:=nil;
  gtk_status_icon_new_from_pixbuf:=nil;
  gtk_status_icon_new_from_file:=nil;
  gtk_status_icon_new_from_stock:=nil;
  gtk_status_icon_new_from_icon_name:=nil;
  gtk_status_icon_set_from_pixbuf:=nil;
  gtk_status_icon_set_from_file:=nil;
  gtk_status_icon_set_from_stock:=nil;
  gtk_status_icon_set_from_icon_name:=nil;
  gtk_status_icon_get_storage_type:=nil;
  gtk_status_icon_get_pixbuf:=nil;
  gtk_status_icon_get_size:=nil;
  gtk_status_icon_set_screen:=nil;
  gtk_status_icon_get_screen:=nil;
  gtk_status_icon_set_tooltip:=nil;
  gtk_status_icon_set_visible:=nil;
  gtk_status_icon_get_visible:=nil;
  gtk_status_icon_set_blinking:=nil;
  gtk_status_icon_get_blinking:=nil;
  gtk_status_icon_is_embedded:=nil;
  gtk_status_icon_position_menu:=nil;
  gtk_status_icon_get_geometry:=nil;
end;


procedure Loadgtkstatusicon;
begin
  Freegtkstatusicon;

  pointer(gtk_status_icon_get_type):=GetProcAddress(gtkhandle,'gtk_status_icon_get_type');
  pointer(gtk_status_icon_new):=GetProcAddress(gtkhandle,'gtk_status_icon_new');
  pointer(gtk_status_icon_new_from_pixbuf):=GetProcAddress(gtkhandle,'gtk_status_icon_new_from_pixbuf');
  pointer(gtk_status_icon_new_from_file):=GetProcAddress(gtkhandle,'gtk_status_icon_new_from_file');
  pointer(gtk_status_icon_new_from_stock):=GetProcAddress(gtkhandle,'gtk_status_icon_new_from_stock');
  pointer(gtk_status_icon_new_from_icon_name):=GetProcAddress(gtkhandle,'gtk_status_icon_new_from_icon_name');
  pointer(gtk_status_icon_set_from_pixbuf):=GetProcAddress(gtkhandle,'gtk_status_icon_set_from_pixbuf');
  pointer(gtk_status_icon_set_from_file):=GetProcAddress(gtkhandle,'gtk_status_icon_set_from_file');
  pointer(gtk_status_icon_set_from_stock):=GetProcAddress(gtkhandle,'gtk_status_icon_set_from_stock');
  pointer(gtk_status_icon_set_from_icon_name):=GetProcAddress(gtkhandle,'gtk_status_icon_set_from_icon_name');
  pointer(gtk_status_icon_get_storage_type):=GetProcAddress(gtkhandle,'gtk_status_icon_get_storage_type');
  pointer(gtk_status_icon_get_pixbuf):=GetProcAddress(gtkhandle,'gtk_status_icon_get_pixbuf');
  pointer(gtk_status_icon_get_stock):=GetProcAddress(gtkhandle, 'gtk_status_icon_get_stock');
  pointer(gtk_status_icon_get_icon_name):=GetProcAddress(gtkhandle, 'gtk_status_icon_get_icon_name');
  pointer(gtk_status_icon_get_size):=GetProcAddress(gtkhandle,'gtk_status_icon_get_size');
  pointer(gtk_status_icon_set_screen):=GetProcAddress(gtkhandle,'gtk_status_icon_set_screen');
  pointer(gtk_status_icon_get_screen):=GetProcAddress(gtkhandle,'gtk_status_icon_get_screen');
  pointer(gtk_status_icon_set_tooltip):=GetProcAddress(gtkhandle,'gtk_status_icon_set_tooltip');
  pointer(gtk_status_icon_set_tooltip_text):=GetProcAddress(gtkhandle,'gtk_status_icon_set_tooltip_text');
  pointer(gtk_status_icon_set_visible):=GetProcAddress(gtkhandle,'gtk_status_icon_set_visible');
  pointer(gtk_status_icon_get_visible):=GetProcAddress(gtkhandle,'gtk_status_icon_get_visible');
  pointer(gtk_status_icon_set_blinking):=GetProcAddress(gtkhandle,'gtk_status_icon_set_blinking');
  pointer(gtk_status_icon_get_blinking):=GetProcAddress(gtkhandle,'gtk_status_icon_get_blinking');
  pointer(gtk_status_icon_is_embedded):=GetProcAddress(gtkhandle,'gtk_status_icon_is_embedded');
  pointer(gtk_status_icon_position_menu):=GetProcAddress(gtkhandle,'gtk_status_icon_position_menu');
  pointer(gtk_status_icon_get_geometry):=GetProcAddress(gtkhandle,'gtk_status_icon_get_geometry');
end;

function GTK_TYPE_STATUS_ICON: GType; inline;
begin
  Result := gtk_status_icon_get_type();
end;

function GTK_STATUS_ICON(obj: Pointer): PGTypeInstance; inline;
begin
  GTK_STATUS_ICON := G_TYPE_CHECK_INSTANCE_CAST (obj, GTK_TYPE_STATUS_ICON);
end;

function GTK_STATUS_ICON_CLASS(klass: Pointer): Pointer; inline;
begin
  GTK_STATUS_ICON_CLASS := G_TYPE_CHECK_CLASS_CAST (klass, GTK_TYPE_STATUS_ICON);
end;

function GTK_IS_STATUS_ICON(obj: Pointer): boolean; inline;
begin
  GTK_IS_STATUS_ICON := G_TYPE_CHECK_INSTANCE_TYPE (obj, GTK_TYPE_STATUS_ICON);
end;

function GTK_IS_STATUS_ICON_CLASS(klass: Pointer): boolean; inline;
begin
  GTK_IS_STATUS_ICON_CLASS := G_TYPE_CHECK_CLASS_TYPE (klass, GTK_TYPE_STATUS_ICON);
end;

function GTK_STATUS_ICON_GET_CLASS(obj: Pointer): PGTypeClass; inline;
begin
  GTK_STATUS_ICON_GET_CLASS := G_TYPE_INSTANCE_GET_CLASS (obj, GTK_TYPE_STATUS_ICON);
end;

function Available_GtkStatusIcon_2_10: Boolean;
begin
  Result := Assigned(gtk_status_icon_new);
end;

function Available_GtkStatusIcon_2_12: Boolean;
begin
  Result := Assigned(gtk_status_icon_get_screen);
end;


