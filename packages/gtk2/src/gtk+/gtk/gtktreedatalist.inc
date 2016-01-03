{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$PACKRECORDS C}

{ gtktreedatalist.h
   Copyright (C) 2000  Red Hat, Inc.,  Jonathan Blandford <jrb@redhat.com>

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
{$ifndef __GTK_TREE_DATA_LIST_H__}
{$define __GTK_TREE_DATA_LIST_H__}


{$include "gtktreesortable.h"}
type
   PGtkTreeDataList = ^TGtkTreeDataList;
   TGtkTreeDataList = record
        next : PGtkTreeDataList;
        data : record
            case longint of
               0 : ( v_int : gint );
               1 : ( v_char : gint8 );
               2 : ( v_uchar : guint8 );
               3 : ( v_uint : guint );
               4 : ( v_float : gfloat );
               5 : ( v_double : gdouble );
               6 : ( v_pointer : gpointer );
            end;
     end;


   PGtkTreeDataSortHeader = ^TGtkTreeDataSortHeader;
   TGtkTreeDataSortHeader = record
        sort_column_id : gint;
        func : TGtkTreeIterCompareFunc;
        data : gpointer;
        destroy : TGtkDestroyNotify;
     end;

function _gtk_tree_data_list_alloc:PGtkTreeDataList; cdecl; external gtklib;
procedure _gtk_tree_data_list_free(list:PGtkTreeDataList; column_headers:PGType); cdecl; external gtklib;
function _gtk_tree_data_list_check_type(_type:GType):gboolean; cdecl; external gtklib;
procedure _gtk_tree_data_list_node_to_value(list:PGtkTreeDataList; _type:GType; value:PGValue); cdecl; external gtklib;
procedure _gtk_tree_data_list_value_to_node(list:PGtkTreeDataList; value:PGValue); cdecl; external gtklib;
function _gtk_tree_data_list_node_copy(list:PGtkTreeDataList; _type:GType):PGtkTreeDataList; cdecl; external gtklib;
{ Header code  }
function gtk_tree_data_list_compare_func(model:PGtkTreeModel; a:PGtkTreeIter; b:PGtkTreeIter; user_data:gpointer):gint; cdecl; external gtklib;
function _gtk_tree_data_list_header_new(n_columns:gint; types:PGType):PGList; cdecl; external gtklib;
procedure _gtk_tree_data_list_header_free(header_list:PGList); cdecl; external gtklib;
function _gtk_tree_data_list_get_header(header_list:PGList; sort_column_id:gint):PGtkTreeDataSortHeader; cdecl; external gtklib;
{$endif}
{ __GTK_TREE_DATA_LIST_H__  }

