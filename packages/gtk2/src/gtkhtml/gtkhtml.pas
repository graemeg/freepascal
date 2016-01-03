{
  Copyright (C) 2000 CodeFactory AB
  Copyright (C) 2000 Jonas Borgström <jonas@codefactory.se>
  Copyright (C) 2000 Anders Carlsson <andersca@codefactory.se>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Library General Public
  License as published by the Free Software Foundation; either
  version 2 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Library General Public License for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
<<<<<<< HEAD
<<<<<<< HEAD
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02111-1301, USA.
=======
  the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
  Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
  Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  !!!!! Maybe wrong for kylix !!!!!
}
unit gtkhtml;

{$H+}
{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses
  gtk2, glib2, atk, pango, gdk2pixbuf, gdk2;

const
// OS dependent defines
// !!!!! Maybe wrong for platforms other than linux !!!!!
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
  gtkhtmllib = 'libgtkhtml-win32-2.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
{$else}
  {$ifdef linux}
  gtkhtmllib = '';
  {$else}
  gtkhtmllib = 'libgtkhtml-2.so';
  {$endif}
{$endif}

const
  DOM_UNSPECIFIED_EVENT_TYPE_ERR = 0;
  DOM_INDEX_SIZE_ERR = 1;
  DOM_DOMSTRING_SIZE_ERR = 2;
  DOM_HIERARCHY_REQUEST_ERR = 3;
  DOM_WRONG_DOCUMENT_ERR = 4;
  DOM_INVALID_CHARACTER_ERR = 5;
  DOM_NO_DATA_ALLOWED_ERR = 6;
  DOM_NO_MODIFICATION_ALLOWED_ERR = 7;
  DOM_NOT_FOUND_ERR = 8;
  DOM_NOT_SUPPORTED_ERR = 9;
  DOM_INUSE_ATTRIBUTE_ERR = 10;
  DOM_INVALID_STATE_ERR = 11;
  DOM_SYNTAX_ERR = 12;
  DOM_INVALID_MODIFICATION_ERR = 13;
  DOM_NAMESPACE_ERR = 14;
  DOM_INVALID_ACCESS_ERR = 15;
  DOM_NO_EXCEPTION = 255;

  DOM_ELEMENT_NODE = 1;
  DOM_ATTRIBUTE_NODE = 2;
  DOM_TEXT_NODE = 3;
  DOM_CDATA_SECTION_NODE = 4;
  DOM_ENTITY_REFERENCE_NODE = 5;
  DOM_ENTITY_NODE = 6;
  DOM_PROCESSING_INSTRUCTION_NODE = 7;
  DOM_COMMENT_NODE = 8;
  DOM_DOCUMENT_NODE = 9;
  DOM_DOCUMENT_TYPE_NODE = 10;
  DOM_DOCUMENT_FRAGMENT_NODE = 11;
  DOM_NOTATION_NODE = 12;

  bm__HtmlFontSpecification_weight = $F;
  bp__HtmlFontSpecification_weight = 0;
  bm__HtmlFontSpecification_style = $30;
  bp__HtmlFontSpecification_style = 4;
  bm__HtmlFontSpecification_variant = $C0;
  bp__HtmlFontSpecification_variant = 6;
  bm__HtmlFontSpecification_stretch = $F00;
  bp__HtmlFontSpecification_stretch = 8;
  bm__HtmlFontSpecification_decoration = $7000;
  bp__HtmlFontSpecification_decoration = 12;

type
  {$DEFINE read_forward_definitions}
  {$include gtkhtmlincludes.inc}
  {$UNDEF read_forward_definitions}

  {$DEFINE read_interface_types}
  {$include gtkhtmlincludes.inc}
  {$UNDEF read_interface_types}

  {$DEFINE read_interface_rest}
  {$include gtkhtmlincludes.inc}
  {$UNDEF read_interface_rest}

implementation

// call implementation parts of header files
{$DEFINE read_implementation}
{$include gtkhtmlincludes.inc}
{$UNDEF read_implementation}

end.//unit gtkhtml
