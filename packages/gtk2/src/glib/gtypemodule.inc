{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$PACKRECORDS C}

{ GObject - GLib Type, Object, Parameter and Signal Library
   Copyright (C) 2000 Red Hat, Inc.

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
{$if !defined (__GLIB_GOBJECT_H_INSIDE__) && !defined (GOBJECT_COMPILATION)}
{$error "Only <glib-anObject.h> can be included directly."}
{$endif}
{$ifndef __G_TYPE_MODULE_H__}
{$define __G_TYPE_MODULE_H__}
{$include <gobject/gobject.inc}
type
function G_TYPE_TYPE_MODULE : longint;
    { return type might be wrong }

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_TYPE_MODULE(module : longint) : longint;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_TYPE_MODULE_CLASS(_class : longint) : longint;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_IS_TYPE_MODULE(module : longint) : longint;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_IS_TYPE_MODULE_CLASS(_class : longint) : longint;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_TYPE_MODULE_GET_CLASS(module : longint) : longint;

type
   PGTypeModule = ^TGTypeModule;
   TGTypeModule = record
        parent_instance : TGObject;
        use_count : guint;
        type_infos : PGSList;
        interface_infos : PGSList;
        name : Pgchar;
     end;

{ Padding for future expansion  }
   PGTypeModuleClass = ^TGTypeModuleClass;
   TGTypeModuleClass = record
        parent_class : TGObjectClass;
        load : function (module:PGTypeModule):gboolean; cdecl;
        unload : procedure (module:PGTypeModule); cdecl;
        reserved1 : procedure ; cdecl;
        reserved2 : procedure ; cdecl;
        reserved3 : procedure ; cdecl;
        reserved4 : procedure ; cdecl;
     end;


function g_type_module_get_type:GType; cdecl; external gliblib;
function g_type_module_use(module:PGTypeModule):gboolean; cdecl; external gliblib;
procedure g_type_module_unuse(module:PGTypeModule); cdecl; external gliblib;

procedure g_type_module_set_name(module:PGTypeModule; name:Pgchar); cdecl; external gliblib;


function g_type_module_register_type(module:PGTypeModule; parent_type:GType; type_name:Pgchar; type_info:PGTypeInfo; flags:TGTypeFlags):GType; cdecl; external gliblib;

procedure g_type_module_add_interface(module:PGTypeModule; instance_type:GType; interface_type:GType; interface_info:PGInterfaceInfo); cdecl; external gliblib;
{$endif}
{ __G_TYPE_MODULE_H__  }
function G_TYPE_TYPE_MODULE : longint;
begin
   G_TYPE_TYPE_MODULE:=g_type_module_get_type;
end;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_TYPE_MODULE(module : longint) : longint;
begin
   G_TYPE_MODULE:=PGTypeModule(G_TYPE_CHECK_INSTANCE_CAST(module,G_TYPE_TYPE_MODULE));
end;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_TYPE_MODULE_CLASS(_class : longint) : longint;
begin
   G_TYPE_MODULE_CLASS:=PGTypeModuleClass(G_TYPE_CHECK_CLASS_CAST(_class,G_TYPE_TYPE_MODULE));
end;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_IS_TYPE_MODULE(module : longint) : longint;
begin
   G_IS_TYPE_MODULE:=G_TYPE_CHECK_INSTANCE_TYPE(module,G_TYPE_TYPE_MODULE);
end;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_IS_TYPE_MODULE_CLASS(_class : longint) : longint;
begin
   G_IS_TYPE_MODULE_CLASS:=G_TYPE_CHECK_CLASS_TYPE(_class,G_TYPE_TYPE_MODULE);
end;

{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
function G_TYPE_MODULE_GET_CLASS(module : longint) : longint;
begin
   G_TYPE_MODULE_GET_CLASS:=PGTypeModuleClass(G_TYPE_INSTANCE_GET_CLASS(module,G_TYPE_TYPE_MODULE));
end;


