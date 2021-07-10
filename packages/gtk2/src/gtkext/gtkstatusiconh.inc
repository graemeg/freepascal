
{* gtkstatusicon.h:
 *
 * Copyright (C) 2003 Sun Microsystems, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 *
 * Authors:
 *      Mark McLoughlin <mark@skynet.ie>
 *}

type
  PGtkStatusIcon = Pointer;
  PGtkStatusIconClass = Pointer;

  PPGdkScreen = PGdkScreen;


var
  gtk_status_icon_get_type : function:GType; cdecl;
  gtk_status_icon_new : function:PGtkStatusIcon; cdecl;
  gtk_status_icon_new_from_pixbuf : function(pixbuf:PGdkPixbuf):PGtkStatusIcon; cdecl;
  gtk_status_icon_new_from_file : function(filename:Pgchar):PGtkStatusIcon; cdecl;
  gtk_status_icon_new_from_stock : function(stock_id:Pgchar):PGtkStatusIcon; cdecl;
  gtk_status_icon_new_from_icon_name : function(icon_name:Pgchar):PGtkStatusIcon; cdecl;
  gtk_status_icon_set_from_pixbuf : procedure(status_icon:PGtkStatusIcon; pixbuf:PGdkPixbuf); cdecl;
  gtk_status_icon_set_from_file : procedure(status_icon:PGtkStatusIcon; filename:Pgchar); cdecl;
  gtk_status_icon_set_from_stock : procedure(status_icon:PGtkStatusIcon; stock_id:Pgchar); cdecl;
  gtk_status_icon_set_from_icon_name : procedure(status_icon:PGtkStatusIcon; icon_name:Pgchar); cdecl;
  gtk_status_icon_get_storage_type : function(status_icon:PGtkStatusIcon):TGtkImageType; cdecl;
  gtk_status_icon_get_pixbuf : function(status_icon:PGtkStatusIcon):PGdkPixbuf; cdecl;
  gtk_status_icon_get_stock : function(status_icon: PGtkStatusIcon):Pgchar; cdecl;
  gtk_status_icon_get_icon_name : function(status_icon: PGtkStatusIcon):Pgchar; cdecl;
  gtk_status_icon_get_size : function(status_icon:PGtkStatusIcon):gint; cdecl;
  gtk_status_icon_set_screen : procedure(status_icon:PGtkStatusIcon; screen:PGdkScreen); cdecl;
  gtk_status_icon_get_screen : function(status_icon:PGtkStatusIcon):PGdkScreen; cdecl;
  gtk_status_icon_set_tooltip : procedure(status_icon:PGtkStatusIcon; tooltip_text:Pgchar); cdecl;
  gtk_status_icon_set_tooltip_text : procedure(status_icon:PGtkStatusIcon; text:Pgchar); cdecl;
  gtk_status_icon_set_visible : procedure(status_icon:PGtkStatusIcon; visible:gboolean); cdecl;
  gtk_status_icon_get_visible : function(status_icon:PGtkStatusIcon):gboolean; cdecl;
  gtk_status_icon_set_blinking : procedure(status_icon:PGtkStatusIcon; blinking:gboolean); cdecl;
  gtk_status_icon_get_blinking : function(status_icon:PGtkStatusIcon):gboolean; cdecl;
  gtk_status_icon_is_embedded : function(status_icon:PGtkStatusIcon):gboolean; cdecl;
  gtk_status_icon_position_menu : procedure(menu:PGtkMenu; x:Pgint; y:Pgint; push_in:Pgboolean; user_data:gpointer); cdecl;
  gtk_status_icon_get_geometry : function(status_icon:PGtkStatusIcon; screen:PPGdkScreen; area:PGdkRectangle; orientation:PGtkOrientation):gboolean; cdecl;

  function GTK_TYPE_STATUS_ICON: GType; inline;
  function GTK_STATUS_ICON(obj: Pointer): PGTypeInstance; inline;
  function GTK_STATUS_ICON_CLASS(klass: Pointer): Pointer; inline;
  function GTK_IS_STATUS_ICON(obj: Pointer): boolean; inline;
  function GTK_IS_STATUS_ICON_CLASS(klass: Pointer): boolean; inline;
  function GTK_STATUS_ICON_GET_CLASS(obj: Pointer): PGTypeClass; inline;

  function Available_GtkStatusIcon_2_10: Boolean;
  function Available_GtkStatusIcon_2_12: Boolean;

