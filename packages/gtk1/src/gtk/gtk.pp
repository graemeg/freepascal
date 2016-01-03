{

   GTK - The GIMP Toolkit
   Copyright (C) 1995-1997 Peter Mattis, Spencer Kimball and Josh MacDonald

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

 **********************************************************************}
unit gtk;
interface

{ Always use smartlinking for win32, this solves some undefined functions
  in the development gtk versions which change often (PFV) }
{$ifdef win32}
  {$ifndef NO_SMART_LINK}
    {$smartlink on}
  {$endif}
{$endif}

{$mode objfpc} { needed for array of const }

uses
  glib,gdk;

{$ifdef win32}
  const
    gtkdll='libgtk-0';
  {$define gtkwin}

  {$packrecords c}
{$else}
  {$ifdef os2}
    const
      gtkdll='gtk12';
    {$define gtkos2}

    {$packrecords c}
  {$else}
    const
    {$ifdef FreeBSD}
      gtkdll='gtk-12';
      {$linklib gtk-12}
    {$else}
    {$ifdef darwin}
      {$define gtkdarwin}
      gtkdll='gtk-1.2.0';
      {$linklib gtk-1.2.0}
    {$else}
      gtkdll='gtk';
      {$linklib gtk}
    {$endif darwin}
    {$endif freebsd}
    {$linklib c}
    {$linklib Xi}
    {$linklib Xext}
    {$linklib X11}
    {$linklib m}

    {$packrecords C}
  {$endif}
{$endif}

{$define gtk_include_files}

{$define read_interface}
{$undef read_implementation}

{$i gtkincludes.pp}

  implementation

{$undef read_interface}
{$define read_implementation}

{$i gtkincludes.pp}

end.
