{
   libglade - a library for building interfaces from XML files at runtime
   Copyright (C) 1998-2002  James Henstridge <james@daa.com.au>

   glade.h: the main include file for libglade.

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
   Boston, MA  02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA  02111-1307, USA.
>>>>>>> origin/fixes_2_2
  }
unit libglade2; // keep unit name lowercase for kylix

{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses
  glib2, gtk2;

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
  {$define gtkwin}
  LibGladeLib = 'libglade-2.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
{$else}
  LibGladeLib = 'libglade-2.0.so';
{$endif}

{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}


{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$include glade-init.inc}
{$include glade-xml.inc}
{ don't include glade-build.h -- it is only for widget set definitions  }

implementation

// glade-init.inc --------------------------------------------------------------

procedure glade_gnome_init;
begin
  glade_init;
end;

procedure glade_bonobo_init;
begin
  glade_init;
end;

// glade-xml.inc ---------------------------------------------------------------

function glade_xml_new_with_domain(fname:Pchar; root:Pchar;
  domain:Pchar):PGladeXML;
begin
  glade_xml_new_with_domain:=glade_xml_new(fname,root,domain);
end;

function glade_xml_new_from_memory(buffer:Pchar; size:longint; root:Pchar;
  domain:Pchar):PGladeXML;
begin
  glade_xml_new_from_memory:=glade_xml_new_from_buffer(buffer,size,root,domain);
end;

function GLADE_TYPE_XML : GType;
begin
   GLADE_TYPE_XML:=glade_xml_get_type;
end;

function GLADE_XML(obj: pointer) : PGladeXML;
begin
   GLADE_XML:=PGladeXML(G_TYPE_CHECK_INSTANCE_CAST(obj,GLADE_TYPE_XML));
end;

function GLADE_XML_CLASS(klass: pointer) : PGladeXMLClass;
begin
   GLADE_XML_CLASS:=PGladeXMLClass(G_TYPE_CHECK_CLASS_CAST(klass,GLADE_TYPE_XML));
end;

function GLADE_IS_XML(obj: pointer) : gboolean;
begin
   GLADE_IS_XML:=G_TYPE_CHECK_INSTANCE_TYPE(obj,GLADE_TYPE_XML);
end;

function GLADE_IS_XML_CLASS(klass: pointer) : gboolean;
begin
   GLADE_IS_XML_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,GLADE_TYPE_XML);
end;

function GLADE_XML_GET_CLASS(obj: pointer) : PGladeXMLClass;
begin
   GLADE_XML_GET_CLASS:=PGladeXMLClass(G_TYPE_INSTANCE_GET_CLASS(obj,GLADE_TYPE_XML));
end;


end.
