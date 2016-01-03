{
   Pango - an open-source framework for the layout and rendering of
           internationalized text.

   Copyright (C) 1999 Red Hat Software

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
}
unit pango; // keep unit name lowercase for kylix

{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses glib2;

{$DEFINE PANGO_ENABLE_ENGINE}
{$DEFINE PANGO_ENABLE_BACKEND}

const
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef windows}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
  {$define pangowin}
  pangolib = 'libpango-1.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
{$else}
  {$ifdef UseCustomLibs}
  pangolib = '';
  {$else}
  pangolib = 'libpango-1.0.so.0';
  {$endif}
{$endif}

{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{ $define HasPango1_20}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$DEFINE read_forward_definitions}
type
  // internal types:
  PPangoFontDescription = ^TPangoFontDescription;
  TPangoFontDescription = pointer;

  PPangoAttrList = ^TPangoAttrList;
  TPangoAttrList = pointer;

  PPangoAttrIterator = ^TPangoAttrIterator;
  TPangoAttrIterator = pointer;

  PPangoLayout = ^TPangoLayout;
  TPangoLayout = pointer;

  PPangoLayoutClass = ^TPangoLayoutClass;
  TPangoLayoutClass = pointer;

  PPangoLayoutIter = ^TPangoLayoutIter;
  TPangoLayoutIter = pointer;

  PPangoContext = ^TPangoContext;
  TPangoContext = pointer;

  PPangoContextClass = ^TPangoContextClass;
  TPangoContextClass = pointer;

  PPangoFontsetSimple = ^TPangoFontsetSimple;
  TPangoFontsetSimple = pointer;

  PPangoTabArray = ^TPangoTabArray;
  TPangoTabArray = pointer;

{$include pangoincludes.inc}
{$UNDEF read_forward_definitions}

{$DEFINE read_interface_types}
{$include pangoincludes.inc}
{$UNDEF read_interface_types}

{$DEFINE read_interface_functions}
{$include pangoincludes.inc}
{$UNDEF read_interface_functions}

implementation

{$DEFINE read_implementation}
{$include pangoincludes.inc}
{$UNDEF read_implementation}

end.
