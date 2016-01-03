{

   gconfclient
   Copyright (C) 1999, 2000 Red Hat Inc.

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
unit gconfclient;

{$PACKRECORDS C}
{$mode objfpc}

interface

Uses glib, gtk, gconf;

const
 gconfclientdll ='gconf-gtk-1';

{$define read_interface}
{$undef read_implementation}

{$include gconflisteners.inc}
{$include gconfchangeset.inc}

type
   TGConfClientPreloadType = (GCONF_CLIENT_PRELOAD_NONE,GCONF_CLIENT_PRELOAD_ONELEVEL,
     GCONF_CLIENT_PRELOAD_RECURSIVE);

   TGConfClientErrorHandlingMode = (GCONF_CLIENT_HANDLE_NONE,GCONF_CLIENT_HANDLE_UNRETURNED,
     GCONF_CLIENT_HANDLE_ALL);


   PGConfClient  = ^TGConfClient;
   TGConfClient = record
        theobject : TGtkObject;
        engine : PGConfEngine;
        error_mode : TGConfClientErrorHandlingMode;
        dir_hash : PGHashTable;
        cache_hash : PGHashTable;
        listeners : PGConfListeners;
     end;
   GCONF_CLIENT = TGCONFCLIENT;

   PGConfClientClass = ^TGConfClientClass;
   TGConfClientClass = record
        parent_class : PGtkObjectClass;
        value_changed : procedure (client:PGConfClient; key:Pgchar; value:PGConfValue); cdecl;
        unreturned_error : procedure (client:PGConfClient; error:PGError); cdecl;
        error : procedure (client:PGConfClient; error:PGError); cdecl;
     end;
   GCONF_CLIENT_CLASS = TGCONFCLIENTCLASS;


   TGConfClientNotifyFunc = procedure (client:PGConfClient; cnxn_id:guint; entry:PGConfEntry; user_data:gpointer);cdecl;

   TGConfClientErrorHandlerFunc = procedure (client:PGConfClient; error:PGError);cdecl;

function GCONF_TYPE_CLIENT : TGTKType;

function GCONF_IS_CLIENT(obj : Pointer) : Boolean;
function GCONF_IS_CLIENT_CLASS(klass : Pointer) : Boolean;

function gconf_client_get_type:TGtkType;cdecl;external gconfclientdll name 'gconf_client_get_type';
function gconf_client_get_default:PGConfClient;cdecl;external gconfclientdll name 'gconf_client_get_default';
function gconf_client_get_for_engine(engine:PGConfEngine):PGConfClient;cdecl;external gconfclientdll name 'gconf_client_get_for_engine';
procedure gconf_client_add_dir(client:PGConfClient; dir:Pgchar; preload:TGConfClientPreloadType; err:PPGError);cdecl;external gconfclientdll name 'gconf_client_add_dir';
procedure gconf_client_remove_dir(client:PGConfClient; dir:Pgchar; err:PPGError);cdecl;external gconfclientdll name 'gconf_client_remove_dir';
function gconf_client_notify_add(client:PGConfClient; namespace_section:Pgchar; func:TGConfClientNotifyFunc; user_data:gpointer; destroy_notify:TGFreeFunc;
           err:PPGError):guint;cdecl;external gconfclientdll name 'gconf_client_notify_add';
procedure gconf_client_notify_remove(client:PGConfClient; cnxn:guint);cdecl;external gconfclientdll name 'gconf_client_notify_remove';
procedure gconf_client_set_error_handling(client:PGConfClient; mode:TGConfClientErrorHandlingMode);cdecl;external gconfclientdll name 'gconf_client_set_error_handling';
procedure gconf_client_set_global_default_error_handler(func:TGConfClientErrorHandlerFunc);cdecl;external gconfclientdll name 'gconf_client_set_global_default_error_handler';
procedure gconf_client_clear_cache(client:PGConfClient);cdecl;external gconfclientdll name 'gconf_client_clear_cache';
procedure gconf_client_preload(client:PGConfClient; dirname:Pgchar; thetype:TGConfClientPreloadType; err:PPGError);cdecl;external gconfclientdll name 'gconf_client_preload';
procedure gconf_client_set(client:PGConfClient; key:Pgchar; val:PGConfValue; err:PPGError);cdecl;external gconfclientdll name 'gconf_client_set';
function gconf_client_get(client:PGConfClient; key:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfclientdll name 'gconf_client_get';
function gconf_client_get_without_default(client:PGConfClient; key:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfclientdll name 'gconf_client_get_without_default';
function gconf_client_get_entry(client:PGConfClient; key:Pgchar; locale:Pgchar; use_schema_default:gboolean; err:PPGError):PGConfEntry;cdecl;external gconfclientdll name 'gconf_client_get_entry';
function gconf_client_get_default_from_schema(client:PGConfClient; key:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfclientdll name 'gconf_client_get_default_from_schema';
function gconf_client_unset(client:PGConfClient; key:Pgchar; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_unset';
function gconf_client_all_entries(client:PGConfClient; dir:Pgchar; err:PPGError):PGSList;cdecl;external gconfclientdll name 'gconf_client_all_entries';
function gconf_client_all_dirs(client:PGConfClient; dir:Pgchar; err:PPGError):PGSList;cdecl;external gconfclientdll name 'gconf_client_all_dirs';
procedure gconf_client_suggest_sync(client:PGConfClient; err:PPGError);cdecl;external gconfclientdll name 'gconf_client_suggest_sync';
function gconf_client_dir_exists(client:PGConfClient; dir:Pgchar; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_dir_exists';
function gconf_client_key_is_writable(client:PGConfClient; key:Pgchar; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_key_is_writable';
function gconf_client_get_float(client:PGConfClient; key:Pgchar; err:PPGError):gdouble;cdecl;external gconfclientdll name 'gconf_client_get_float';
function gconf_client_get_int(client:PGConfClient; key:Pgchar; err:PPGError):gint;cdecl;external gconfclientdll name 'gconf_client_get_int';
function gconf_client_get_string(client:PGConfClient; key:Pgchar; err:PPGError):Pgchar;cdecl;external gconfclientdll name 'gconf_client_get_string';
function gconf_client_get_bool(client:PGConfClient; key:Pgchar; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_get_bool';
function gconf_client_get_schema(client:PGConfClient; key:Pgchar; err:PPGError):PGConfSchema;cdecl;external gconfclientdll name 'gconf_client_get_schema';
function gconf_client_get_list(client:PGConfClient; key:Pgchar; list_type:TGConfValueType; err:PPGError):PGSList;cdecl;external gconfclientdll name 'gconf_client_get_list';
function gconf_client_get_pair(client:PGConfClient; key:Pgchar; car_type:TGConfValueType; cdr_type:TGConfValueType; car_retloc:gpointer;
           cdr_retloc:gpointer; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_get_pair';
function gconf_client_set_float(client:PGConfClient; key:Pgchar; val:gdouble; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_float';
function gconf_client_set_int(client:PGConfClient; key:Pgchar; val:gint; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_int';
function gconf_client_set_string(client:PGConfClient; key:Pgchar; val:Pgchar; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_string';
function gconf_client_set_bool(client:PGConfClient; key:Pgchar; val:gboolean; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_bool';
function gconf_client_set_schema(client:PGConfClient; key:Pgchar; val:PGConfSchema; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_schema';
function gconf_client_set_list(client:PGConfClient; key:Pgchar; list_type:TGConfValueType; list:PGSList; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_list';
function gconf_client_set_pair(client:PGConfClient; key:Pgchar; car_type:TGConfValueType; cdr_type:TGConfValueType; const address_of_car:gpointer;
           const address_of_cdr:gpointer; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_set_pair';
procedure gconf_client_error(client:PGConfClient; error:PGError);cdecl;external gconfclientdll name 'gconf_client_error';
procedure gconf_client_unreturned_error(client:PGConfClient; error:PGError);cdecl;external gconfclientdll name 'gconf_client_unreturned_error';
procedure gconf_client_value_changed(client:PGConfClient; key:Pgchar; value:PGConfValue);cdecl;external gconfclientdll name 'gconf_client_value_changed';
function gconf_client_commit_change_set(client:PGConfClient; cs:PGConfChangeSet; remove_committed:gboolean; err:PPGError):gboolean;cdecl;external gconfclientdll name 'gconf_client_commit_change_set';
function gconf_client_reverse_change_set(client:PGConfClient; cs:PGConfChangeSet; err:PPGError):PGConfChangeSet;cdecl;external gconfclientdll name 'gconf_client_reverse_change_set';
function gconf_client_change_set_from_currentv(client:PGConfClient; keys:PPgchar; err:PPGError):PGConfChangeSet;cdecl;external gconfclientdll name 'gconf_client_change_set_from_currentv';
function gconf_client_change_set_from_current(client:PGConfClient; err:PPGError; first_key:Pgchar; args:array of const):PGConfChangeSet;cdecl;external gconfclientdll name 'gconf_client_change_set_from_current';
function gconf_client_change_set_from_current(client:PGConfClient; err:PPGError; first_key:Pgchar):PGConfChangeSet;cdecl;external gconfclientdll name 'gconf_client_change_set_from_current';

implementation

function GCONF_TYPE_CLIENT : TGTKType;
  begin
     GCONF_TYPE_CLIENT:=gconf_client_get_type;
  end;

function  GCONF_IS_CLIENT(obj:pointer):boolean;
begin
  GCONF_IS_CLIENT:=(obj<>nil) and GCONF_IS_CLIENT_CLASS(PGtkTypeObject(obj)^.klass);
end;

function  GCONF_IS_CLIENT_CLASS(klass:pointer):boolean;
begin
  GCONF_IS_CLIENT_CLASS:=(klass<>nil) and (PGtkTypeClass(klass)^.thetype=GCONF_TYPE_CLIENT);
end;

{$define read_implementation}
{$undef read_interface}

{$include gconflisteners.inc}
{$include gconfchangeset.inc}

end.
