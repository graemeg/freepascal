{ GTK - The GIMP Toolkit
   Copyright (C) 2005 Mattias Gaertner, Olaf Leidinger

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
   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
  }
unit gtk2; // keep unit name lowercase for kylix

// default GTK2_8
{$define GTK2_8}

{ Smartlinking has problems on powerpc-linux because of commandline length, disable
  it for now }
{$ifdef powerpc}
  {$ifdef linux}
    {$smartlink off}
  {$endif}
{$endif}

{$H+}
{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

{$IFDEF GTK2_2}
{$DEFINE HasGTK2_0}
{$DEFINE HasGTK2_2}
{$ENDIF}

{$IFDEF GTK2_4}
{$DEFINE HasGTK2_0}
{$DEFINE HasGTK2_2}
{$DEFINE HasGTK2_4}
{$ENDIF}

{$IFDEF GTK2_6}
{$DEFINE HasGTK2_0}
{$DEFINE HasGTK2_2}
{$DEFINE HasGTK2_4}
{$DEFINE HasGTK2_6}
{$ENDIF}

{$IFDEF GTK2_8}
{$DEFINE HasGTK2_0}
{$DEFINE HasGTK2_2}
{$DEFINE HasGTK2_4}
{$DEFINE HasGTK2_6}
{$DEFINE HasGTK2_8}
{$ENDIF}

// {$define HasGTK_FileSystem}

interface

uses
  glib2, atk, pango, gdk2pixbuf, gdk2;

const
// OS dependent defines
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef windows}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
  {$DEFINE GTK_WINDOWING_WIN32}
  gtklib = 'libgtk-win32-2.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
{$else}
  {$IFDEF KYLIX}
    gtklib = 'libgtk-x11-2.0.so';
  {$ELSE}
    {$ifdef darwin}
      gtklib = 'gtk-x11-2.0';
      {$linklib gtk-x11-2.0}
      {$linklib gdk-x11-2.0}
      {$linklib pango-1.0.0}
      {$linklib glib-2.0.0}
      {$linklib gobject-2.0.0}
      {$linklib gdk_pixbuf-2.0.0}
      {$linklib atk-1.0.0}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      {$linklib gthread-2.0}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      {$linklib pangocairo-1.0}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
      {$linklib gthread-2.0}
>>>>>>> origin/fixes_2.4
    {$else}
      {$ifdef UseCustomLibs}
      gtklib = '';
      {$else}
      gtklib = 'libgtk-x11-2.0.so';
      {$endif}
    {$endif}
  {$ENDIF}
{$endif}


{$IFDEF KYLIX}
Type
  PPPchar   = PPPgchar;
{$ENDIF}


{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}

const
   GTK_MAX_COMPOSE_LEN = 7;

type
{$DEFINE read_forward_definitions}
{$include gtkincludes.inc}
{$UNDEF read_forward_definitions}

{$DEFINE read_interface_types}
{$include gtkincludes.inc}
{$UNDEF read_interface_types}

{$DEFINE read_interface_rest}
{$include gtkincludes.inc}
{$UNDEF read_interface_rest}

implementation

uses
  SysUtils;

{$IFDEF FPC}
{ There is a bug in the compiler. If an external variable is not used, it will
  create code, that can be relocated by the linker.
  So, use them in this hidden dummy procedure.
}
procedure CheckUnusedVariable; [Public];
begin
  if (gtk_major_version<>0) then ;
  if (gtk_minor_version<>0) then ;
  if (gtk_micro_version<>0) then ;
  if (gtk_binary_age<>0) then ;
  if (gtk_interface_age<>0) then ;
  if (ord(gtk_text_attr_appearance_type)<>0) then ;
end;
{$ENDIF}


// call implementation parts of header files
{$DEFINE read_implementation}
{$include gtkincludes.inc}
{$UNDEF read_implementation}

end.
