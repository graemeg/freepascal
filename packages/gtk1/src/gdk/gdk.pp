{

   GDK - The GIMP Drawing Kit
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
   Boston, MA 02111-1301, USA.
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2

 **********************************************************************}
unit gdk;
interface

{$mode objfpc} { needed for array of const }

{ Always use smartlinking for win32, this solves some undefined functions
  in the development gtk versions which change often (PFV) }
{$ifdef win32}
  {$ifndef NO_SMART_LINK}
    {$smartlink on}
  {$endif}
{$endif}

uses
  glib;

{$ifdef win32}
  const
    gdkdll='libgdk-0';
  {$define gtkwin}

  {$packrecords C}
{$else}
  {$ifdef os2}
    const
      gdkdll='gdk12';
      {$linklib gdk12}
    {$define gtkos2}

    {$packrecords C}
  {$else}
    const
    {$ifdef FreeBSD}
      gdkdll='gdk-12';
      {$linklib gdk-12}
    {$else}
    {$ifdef darwin}
      {$define gtkdarwin}
      gdkdll='gdk-1.2.0';
      {$linklib gdk-1.2.0}
    {$else}
      gdkdll='gdk';
      {$linklib gdk}
    {$endif}
    {$endif}
    {$linklib c}
    {$linklib X11}

    {$packrecords C}
  {$endif}
{$endif}

{$define gdk_include_files}

{$define read_interface}
{$undef read_implementation}

{$i gdktypes.pp}
{$i gdkkeysyms.pp}
{$i gdkprivate.pp}
{$i gdkrgb.pp}
{$i gdkx.pp}
{$i gdkmain.pp}


implementation

{$undef read_interface}
{$define read_implementation}

{$i gdktypes.pp}
{$i gdkkeysyms.pp}
{$i gdkprivate.pp}
{$i gdkrgb.pp}
{$i gdkx.pp}
{$i gdkmain.pp}

end.
