{

   GConf
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
unit gconf;

{$PACKRECORDS C}
{$mode objfpc}

interface

Uses glib;

const
 gconfdll='gconf-1';

type
   TGConfValueType = (GCONF_VALUE_INVALID,GCONF_VALUE_STRING,
     GCONF_VALUE_INT,GCONF_VALUE_FLOAT,GCONF_VALUE_BOOL,
     GCONF_VALUE_SCHEMA,GCONF_VALUE_LIST,GCONF_VALUE_PAIR
     );

type
   PGConfSchema = ^TGConfSchema;
   PPGConfValue = ^PGConfValue;
   PGConfValue = ^TGConfValue;

   TGConfSchema = record
        thetype : TGConfValueType;
        list_type : TGConfValueType;
        car_type : TGConfValueType;
        cdr_type : TGConfValueType;
        locale : pgchar;
        theowner : pgchar;
        short_desc : pgchar;
        long_desc : pgchar;
        default_value : PGConfValue;
     end;

   TGConfValue = record
        thetype : TGConfValueType;
        d : record
            case longint of
               0 : ( string_data : Pgchar );
               1 : ( int_data : gint );
               2 : ( bool_data : gboolean );
               3 : ( float_data : gdouble );
               4 : ( schema_data : PGConfSchema );
               5 : ( list_data : record
                    listtype : TGConfValueType;
                    list : PGSList;
                 end );
               6 : ( pair_data : record
                    car : PGConfValue;
                    cdr : PGConfValue;
                 end );
            end;
     end;

{$define read_interface}
{$undef read_implementation}

{$include gconfglibpublic.inc}
{$include gconferror.inc}
{$include gconfvalue.inc}
{$include gconfschema.inc}
{$include gconfengine.inc}

function gconf_is_initialized:gboolean;cdecl;external gconfdll name 'gconf_is_initialized';

type
   TGConfNotifyFunc = procedure (conf:PGConfEngine; cnxn_id:guint; entry:PGConfEntry; user_data:gpointer);cdecl;

function gconf_engine_notify_add(conf:PGConfEngine; namespace_section:Pgchar; func:TGConfNotifyFunc; user_data:gpointer; err:PPGError):guint;cdecl;external gconfdll name 'gconf_engine_notify_add';
procedure gconf_engine_notify_remove(conf:PGConfEngine; cnxn:guint);cdecl;external gconfdll name 'gconf_engine_notify_remove';
function gconf_engine_get(conf:PGConfEngine; key:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfdll name 'gconf_engine_get';
function gconf_engine_get_without_default(conf:PGConfEngine; key:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfdll name 'gconf_engine_get_without_default';
function gconf_engine_get_entry(conf:PGConfEngine; key:Pgchar; locale:Pgchar; use_schema_default:gboolean; err:PPGError):PGConfEntry;cdecl;external gconfdll name 'gconf_engine_get_entry';
function gconf_engine_get_with_locale(conf:PGConfEngine; key:Pgchar; locale:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfdll name 'gconf_engine_get_with_locale';
function gconf_engine_get_default_from_schema(conf:PGConfEngine; key:Pgchar; err:PPGError):PGConfValue;cdecl;external gconfdll name 'gconf_engine_get_default_from_schema';
function gconf_engine_set(conf:PGConfEngine; key:Pgchar; value:PGConfValue; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set';
function gconf_engine_unset(conf:PGConfEngine; key:Pgchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_unset';
function gconf_engine_associate_schema(conf:PGConfEngine; key:Pgchar; schema_key:Pgchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_associate_schema';
function gconf_engine_all_entries(conf:PGConfEngine; dir:Pgchar; err:PPGError):PGSList;cdecl;external gconfdll name 'gconf_engine_all_entries';
function gconf_engine_all_dirs(conf:PGConfEngine; dir:Pgchar; err:PPGError):PGSList;cdecl;external gconfdll name 'gconf_engine_all_dirs';
procedure gconf_engine_suggest_sync(conf:PGConfEngine; err:PPGError);cdecl;external gconfdll name 'gconf_engine_suggest_sync';
function gconf_engine_dir_exists(conf:PGConfEngine; dir:Pgchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_dir_exists';
procedure gconf_engine_remove_dir(conf:PGConfEngine; dir:Pgchar; err:PPGError);cdecl;external gconfdll name 'gconf_engine_remove_dir';
function gconf_engine_key_is_writable(conf:PGConfEngine; key:Pgchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_key_is_writable';
function gconf_valid_key(key:Pgchar; why_invalid:PPgchar):gboolean;cdecl;external gconfdll name 'gconf_valid_key';
function gconf_key_is_below(above:Pgchar; below:Pgchar):gboolean;cdecl;external gconfdll name 'gconf_key_is_below';
function gconf_concat_dir_and_key(dir:Pgchar; key:Pgchar):pgchar;cdecl;external gconfdll name 'gconf_concat_dir_and_key';
function gconf_unique_key:pgchar;cdecl;external gconfdll name 'gconf_unique_key';
function gconf_engine_get_float(conf:PGConfEngine; key:Pgchar; err:PPGError):gdouble;cdecl;external gconfdll name 'gconf_engine_get_float';
function gconf_engine_get_int(conf:PGConfEngine; key:Pgchar; err:PPGError):gint;cdecl;external gconfdll name 'gconf_engine_get_int';
function gconf_engine_get_string(conf:PGConfEngine; key:Pgchar; err:PPGError):pgchar;cdecl;external gconfdll name 'gconf_engine_get_string';
function gconf_engine_get_bool(conf:PGConfEngine; key:Pgchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_get_bool';
function gconf_engine_get_schema(conf:PGConfEngine; key:Pgchar; err:PPGError):PGConfSchema;cdecl;external gconfdll name 'gconf_engine_get_schema';
function gconf_engine_get_list(conf:PGConfEngine; key:Pgchar; list_type:TGConfValueType; err:PPGError):PGSList;cdecl;external gconfdll name 'gconf_engine_get_list';
function gconf_engine_get_pair(conf:PGConfEngine; key:Pgchar; car_type:TGConfValueType; cdr_type:TGConfValueType; car_retloc:gpointer;
           cdr_retloc:gpointer; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_get_pair';
function gconf_engine_set_float(conf:PGConfEngine; key:Pgchar; val:gdouble; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_float';
function gconf_engine_set_int(conf:PGConfEngine; key:Pgchar; val:gint; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_int';
function gconf_engine_set_string(conf:PGConfEngine; key:Pgchar; val:Pgchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_string';
function gconf_engine_set_bool(conf:PGConfEngine; key:Pgchar; val:gboolean; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_bool';
function gconf_engine_set_schema(conf:PGConfEngine; key:Pgchar; val:PGConfSchema; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_schema';
function gconf_engine_set_list(conf:PGConfEngine; key:Pgchar; list_type:TGConfValueType; list:PGSList; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_list';
function gconf_engine_set_pair(conf:PGConfEngine; key:Pgchar; car_type:TGConfValueType; cdr_type:TGConfValueType; const address_of_car:gpointer;
           const address_of_cdr:gpointer; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_engine_set_pair';
type
   TGConfEnumStringPair = record
        enum_value : gint;
        str : pgchar;
     end;

function gconf_string_to_enum(lookup_table:array of TGConfEnumStringPair; str:Pgchar; enum_value_retloc:Pgint):gboolean;cdecl;external gconfdll name 'gconf_string_to_enum';
function gconf_enum_to_string(lookup_table:array of TGConfEnumStringPair; enum_value:gint):pgchar;cdecl;external gconfdll name 'gconf_enum_to_string';
function gconf_init(argc:longint; argv:PPchar; err:PPGError):gboolean;cdecl;external gconfdll name 'gconf_init';

implementation

{$undef read_interface}
{$define read_implementation}

{$include gconfglibpublic.inc}
{$include gconferror.inc}
{$include gconfvalue.inc}
{$include gconfschema.inc}
{$include gconfengine.inc}

end.
