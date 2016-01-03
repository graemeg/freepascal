{
   GDK - The GIMP Drawing Kit

   Copyright (C) 1995-1997 Peter Mattis, Spencer Kimball and Josh MacDonald

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the
<<<<<<< HEAD
<<<<<<< HEAD
   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301, USA.
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
unit gdk2; // keep unit name lowercase for kylix

{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

<<<<<<< HEAD
<<<<<<< HEAD
uses glib2, gdk2pixbuf, pango, cairo;

const
// OS dependent defines
{$ifdef windows}
=======
=======
>>>>>>> origin/fixes_2_2
uses glib2, gdk2pixbuf, pango;

const
// OS dependent defines
{$ifdef win32}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$DEFINE GDK_WINDOWING_WIN32}
  gdklib = 'libgdk-win32-2.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
  GDK_HAVE_WCHAR_H=1;
  GDK_HAVE_WCTYPE_H=1;
{$else}
  {$ifdef darwin}
    gdklib = 'gdk-x11-2.0';
    {$linklib gtk-x11-2.0}
    {$linklib gdk-x11-2.0}
    {$linklib pango-1.0.0}
    {$linklib glib-2.0.0}
    {$linklib gobject-2.0.0}
    {$linklib gdk_pixbuf-2.0.0}
    {$linklib atk-1.0.0}
  {$else}
    {$ifdef UseCustomLibs}
    gdklib = '';
    {$else}
    gdklib = 'libgdk-x11-2.0.so';
    {$endif}
  {$endif}
{$endif}

{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}


// the included header files:
{
   number of points to buffer before sending them off
   to scanlines() :  Must be an even number
  }
const
  NUMPTSTOBUFFER = 200;
  GDK_MAX_TIMECOORD_AXES = 128;

type
  PGdkDeviceClass = ^TGdkDeviceClass;
  TGdkDeviceClass = record
      parent_class: TGObjectClass;
    end;

  PGdkVisualClass = ^TGdkVisualClass;
  TGdkVisualClass = record
      parent_class: TGObjectClass;
    end;

{$DEFINE read_forward_definitions}
{$include gdkincludes.inc}
{$UNDEF read_forward_definitions}

{$DEFINE read_interface_types}
{$include gdkincludes.inc}
{$UNDEF read_interface_types}

{$DEFINE read_interface_rest}
{$include gdkincludes.inc}
{$UNDEF read_interface_rest}

procedure gdk_parse_args(argc:Pgint; var argv:PPgchar); cdecl; external gdklib;
procedure gdk_init(argc:Pgint; var argv:PPgchar); cdecl; external gdklib;
function gdk_init_check(argc:Pgint; var argv:PPgchar):gboolean; cdecl; external gdklib;
procedure gdk_notify_startup_complete; cdecl; external gdklib;

{$ifndef GDK_DISABLE_DEPRECATED}
procedure gdk_exit(error_code:gint); cdecl; external gdklib;
{$endif}
{ GDK_DISABLE_DEPRECATED  }

function gdk_set_locale:Pgchar; cdecl; external gdklib;
function gdk_get_program_class:Pchar; cdecl; external gdklib;
procedure gdk_set_program_class(program_class:Pchar); cdecl; external gdklib;

{ Push and pop error handlers for X errors }
procedure gdk_error_trap_push; cdecl; external gdklib;
function gdk_error_trap_pop:gint; cdecl; external gdklib;

{$ifndef GDK_DISABLE_DEPRECATED}
procedure gdk_set_use_xshm(use_xshm:gboolean); cdecl; external gdklib;
function gdk_get_use_xshm:gboolean; cdecl; external gdklib;
{$endif}
{ GDK_DISABLE_DEPRECATED  }

function gdk_get_display:Pgchar; cdecl; external gdklib;
function gdk_get_display_arg_name:Pgchar; cdecl; external gdklib;

{$ifndef GDK_DISABLE_DEPRECATED}
function gdk_input_add_full(source:gint; condition:TGdkInputCondition; _function:TGdkInputFunction; data:gpointer; destroy:TGdkDestroyNotify):gint; cdecl; external gdklib;
function gdk_input_add(source:gint; condition:TGdkInputCondition; _function:TGdkInputFunction; data:gpointer):gint; cdecl; external gdklib;
procedure gdk_input_remove(tag:gint); cdecl; external gdklib;
{$endif}
{ GDK_DISABLE_DEPRECATED  }

function gdk_pointer_grab(window:PGdkWindow; owner_events:gboolean; event_mask:TGdkEventMask; confine_to:PGdkWindow; cursor:PGdkCursor;
           time:guint32):TGdkGrabStatus; cdecl; external gdklib;
function gdk_keyboard_grab(window:PGdkWindow; owner_events:gboolean; time:guint32):TGdkGrabStatus; cdecl; external gdklib;

{$ifndef GDK_MULTIHEAD_SAFE}
procedure gdk_pointer_ungrab(time:guint32); cdecl; external gdklib;
procedure gdk_keyboard_ungrab(time:guint32); cdecl; external gdklib;
function gdk_pointer_is_grabbed:gboolean; cdecl; external gdklib;
function gdk_screen_width:gint; cdecl; external gdklib;
function gdk_screen_height:gint; cdecl; external gdklib;
function gdk_screen_width_mm:gint; cdecl; external gdklib;
function gdk_screen_height_mm:gint; cdecl; external gdklib;
procedure gdk_beep; cdecl; external gdklib;
{$endif}
{ GDK_MULTIHEAD_SAFE  }

procedure gdk_flush; cdecl; external gdklib;

{$ifndef GDK_MULTIHEAD_SAFE}
procedure gdk_set_double_click_time(msec:guint); cdecl; external gdklib;
{$endif}

{ Rectangle utilities }
function gdk_rectangle_intersect(src1:PGdkRectangle; src2:PGdkRectangle; dest:PGdkRectangle):gboolean; cdecl; external gdklib;
procedure gdk_rectangle_union(src1:PGdkRectangle; src2:PGdkRectangle; dest:PGdkRectangle); cdecl; external gdklib;
function gdk_rectangle_get_type:GType; cdecl; external gdklib;
function GDK_TYPE_RECTANGLE : GType;

{ Conversion functions between wide char and multibyte strings. }
function gdk_wcstombs(src:PGdkWChar):Pgchar; cdecl; external gdklib;
function gdk_mbstowcs(dest:PGdkWChar; src:Pgchar; dest_max:gint):gint; cdecl; external gdklib;
{ Miscellaneous  }

{$ifndef GDK_MULTIHEAD_SAFE}
function gdk_event_send_client_message(event:PGdkEvent; xid:guint32):gboolean; cdecl; external gdklib;
procedure gdk_event_send_clientmessage_toall(event:PGdkEvent); cdecl; external gdklib;
{$endif}


{$IFNDEF KYLIX}
{ Threading }
var
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF WINDOWS}
=======
  {$IFDEF WIN32}
>>>>>>> graemeg/fixes_2_2
=======
  {$IFDEF WIN32}
>>>>>>> origin/fixes_2_2
  gdk_threads_mutex : PGMutex; external gdklib name 'gdk_threads_mutex';
  {$ELSE}
  gdk_threads_mutex : PGMutex; cvar; external;
  {$ENDIF}
{$ENDIF}

procedure gdk_threads_enter; cdecl; external gdklib;
procedure gdk_threads_leave; cdecl; external gdklib;
procedure gdk_threads_init; cdecl; external gdklib;
procedure gdk_threads_set_lock_functions(enter_fn: TGCallback; leave_fn: TGCallback); cdecl; external gdklib;

procedure _GDK_THREADS_ENTER;
procedure _GDK_THREADS_LEAVE;



implementation

{$IFNDEF KYLIX}
{ There is a bug in the compiler. If an external variable is not used, it will
  create code, that can be relocated by the linker.
  So, use them in this hidden procedure.
}
procedure CheckUnusedVariable; [Public];
begin
  if (gdk_threads_mutex<>nil) then ;
end;
{$ENDIF}

//------------------------------------------------------------------------------

function GDK_TYPE_RECTANGLE : GType;
begin
  GDK_TYPE_RECTANGLE:=gdk_rectangle_get_type;
end;

procedure _GDK_THREADS_ENTER;
begin
{$ifdef G_THREADS_ENABLED}
  if (gdk_threads_mutex) then
    g_mutex_lock (gdk_threads_mutex);
{$endif}
end;

procedure _GDK_THREADS_LEAVE;
begin
{$ifdef G_THREADS_ENABLED}
  if (gdk_threads_mutex) then
    g_mutex_unlock (gdk_threads_mutex);
{$endif}
end;

// call implementation parts of header files
{$DEFINE read_implementation}
{$include gdkincludes.inc}
{$UNDEF read_implementation}

end.
