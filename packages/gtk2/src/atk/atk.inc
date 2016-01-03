{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$PACKRECORDS C}

{ ATK -  Accessibility Toolkit
   Copyright 2001 Sun Microsystems Inc.

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
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
  }
{$ifndef __ATK_H__}
{$define __ATK_H__}
{$include <atk/atkobject.inc}
{$include <atk/atkaction.inc}
{$include <atk/atkcomponent.inc}
{$include <atk/atkdocument.inc}
{$include <atk/atkeditabletext.inc}
{$include <atk/atkgobjectaccessible.inc}
{$include <atk/atkhyperlink.inc}
{$include <atk/atkhypertext.inc}
{$include <atk/atkimage.inc}
{$include <atk/atkobjectfactory.inc}
{$include <atk/atkregistry.inc}
{$include <atk/atkrelation.inc}
{$include <atk/atkrelationset.inc}
{$include <atk/atkselection.inc}
{$include <atk/atkstate.inc}
{$include <atk/atkstateset.inc}
{$include <atk/atkstreamablecontent.inc}
{$include <atk/atktable.inc}
{$include <atk/atktext.inc}
{$include <atk/atkutil.inc}
{$include <atk/atkvalue.inc}
{$endif}
{ __ATK_H__  }

