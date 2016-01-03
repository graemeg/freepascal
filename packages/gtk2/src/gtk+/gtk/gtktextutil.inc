{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$PACKRECORDS C}

{ GTK - The GIMP Toolkit
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
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
  }
{
   Modified by the GTK+ Team and others 1997-2001.  See the AUTHORS
   file for a list of people on the GTK+ Team.  See the ChangeLog
   files for a list of changes.  These files are distributed with
   GTK+ at ftp://ftp.gtk.org/pub/gtk/.
  }
{$ifndef __GTK_TEXT_UTIL_H__}
{$define __GTK_TEXT_UTIL_H__}
{$include gtkwidget.inc}
{$include gtkmenushell.inc}
{ C++ extern C conditionnal removed }
{ __cplusplus  }
{ This is a private uninstalled header shared between GtkTextView and GtkEntry  }

type

   TGtkTextUtilCharChosenFunc = procedure (text:Pchar; data:gpointer); cdecl;

procedure _gtk_text_util_append_special_char_menuitems(menushell:PGtkMenuShell; func:TGtkTextUtilCharChosenFunc; data:gpointer); cdecl; external gtklib;
{ C++ end of extern C conditionnal removed }
{ __cplusplus  }
{$endif}
{ __GTK_TEXT_UTIL_H__  }

