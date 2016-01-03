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
   Copyright (C) 2000 Red Hat Software

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
{$ifndef __GTK_IM_MODULE_H__}
{$define __GTK_IM_MODULE_H__}
{ C++ extern C conditionnal removed }
{ __cplusplus  }
{$include gtkimcontext.inc}
type





   PGtkIMContextInfo = ^TGtkIMContextInfo;
   TGtkIMContextInfo = record
        context_id : Pgchar;
        context_name : Pgchar;
        domain : Pgchar;
        domain_dirname : Pgchar;
        default_locales : Pgchar;
     end;

{ Functions for use within GTK+
  }


procedure _gtk_im_module_list(contexts:PPPGtkIMContextInfo; n_contexts:Pguint); cdecl; external gtklib;

function _gtk_im_module_create(context_id:Pgchar):PGtkIMContext; cdecl; external gtklib;


function _gtk_im_module_get_default_context_id(lang:Pgchar):Pgchar; cdecl; external gtklib;
{ The following entry points are exported by each input method module
  }
{
void          im_module_list   (const GtkIMContextInfo    contexts,
                                guint                     n_contexts);
void          im_module_init   (GtkModule              module);
void          im_module_exit   (void);
GtkIMContext  im_module_create (const gchar              context_id);
 }
{ C++ end of extern C conditionnal removed }
{ __cplusplus  }
{$endif}
{ __GTK_IM_MODULE_H__  }

