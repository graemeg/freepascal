{

   GLIB - Library of useful routines for C programming
   Copyright (C) 1995-1997  Peter Mattis, Spencer Kimball and Josh MacDonald

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
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
unit glib;
interface

{$mode objfpc}

{ Always use smartlinking for win32, this solves some undefined functions
  in the development gtk versions which change often (PFV) }
{$ifdef win32}
  {$ifndef NO_SMART_LINK}
    {$smartlink on}
  {$endif}
{$endif}

{$ifdef win32}
  const
    glibdll='libglib-2.0-0';
  {$define gtkwin}

  {$packrecords C}
{$else}
  {$ifdef os2}
    const
      glibdll='glib12';
    {$define gtkos2}

    {$packrecords C}
  {$else}
    const
     {$ifdef FreeBSD}
      glibdll='glib-12';
      {$linklib glib-12}
     {$else}
     {$ifdef darwin}
      glibdll='glib-1.2.0';
      {$linklib glib-1.2.0}
     {$else darwin}
      glibdll='glib';
      {$linklib glib}
     {$endif darwin}
     {$endif}
    {$linklib c}

    {$packrecords C}
  {$endif}
{$endif}

{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  FLoat     = Single;

  const
     NULL = nil;

     G_DIR_SEPARATOR = '/';
     G_DIR_SEPARATOR_S = '/';
     G_SEARCHPATH_SEPARATOR = ':';
     G_SEARCHPATH_SEPARATOR_S = ':';

type
   PPgchar = ^Pgchar;
   Pgchar = ^gchar;
   gchar = char;
   Pgshort = ^gshort;
   gshort = smallint;
   Pglong = ^glong;
   glong = ptrint;
   Pgint = ^gint;
   gint = longint;
   Pgboolean = ^gboolean;
   gboolean = longbool;
   Pguchar = ^guchar;
   guchar = char;
   Pgushort = ^gushort;
   gushort = word;
   Pgulong = ^gulong;
   gulong = ptruint;
   Pguint = ^guint;
   guint = cardinal;
   Pgfloat = ^gfloat;
   gfloat = float;
   Pgdouble = ^gdouble;
   gdouble = double;
   Pgpointer = ^gpointer;
   gpointer = pointer;

   gint8 = ShortInt;
   guint8 = byte;
   gint16 = smallint;
   guint16 = word;
   gint32 = longint;
   guint32 = cardinal;

   Pgint8 = ^ShortInt;
   Pguint8 = ^byte;
   Pgint16 = ^smallint;
   Pguint16 = ^word;
   Pgint32 = ^longint;
   Pguint32 = ^cardinal;

   PPgint8 = ^Pgint8;
   PPguint8 = ^Pguint8;
   PPgint16 = ^Pgint16;
   PPguint16 = ^Pguint16;
   PPgint32 = ^Pgint32;
   PPguint32 = ^Pguint32;

   gconstpointer = pointer;
   gssize = gint32;
   gsize = guint32;
   TGQuark = guint32;
   TGTime = gint32;

{$ifndef gtkos2}
var
   glib_major_version : guint;external glibdll name 'glib_major_version';
   glib_minor_version : guint;external glibdll name 'glib_minor_version';
   glib_micro_version : guint;external glibdll name 'glib_micro_version';
   glib_interface_age : guint;external glibdll name 'glib_interface_age';
   glib_binary_age : guint;external glibdll name 'glib_binary_age';
{$endif}

type
   PGList = ^TGList;
   TGList = record
        data : gpointer;
        next : PGList;
        prev : PGList;
     end;
   PGSList = ^TGSList;
   PPGSList = ^PGSList;
   TGSList = record
        data : gpointer;
        next : PGSList;
     end;
   PGString = ^TGString;
   TGString = record
        str : Pgchar;
        len : gint;
     end;
   PGArray = ^TGArray;
   TGArray = record
        data : Pgchar;
        len : guint;
     end;
   PGDebugKey = ^TGDebugKey;
   TGDebugKey = record
        key : Pgchar;
        value : guint;
     end;
   PGHashTable = ^TGHashTable;
   TGHashTable = record
        dummy : gint;
     end;
   PGCache = ^TGCache;
   TGCache = record
        dummy : gint;
     end;
   PGTree = ^TGTree;
   TGTree = record
        dummy : gint;
     end;
   PGTimer = ^TGTimer;
   TGTimer = record
        dummy : gint;
     end;
   PGMemChunk = ^TGMemChunk;
   TGMemChunk = record
        dummy : gint;
     end;
   PGStringChunk = ^TGStringChunk;
   TGStringChunk = record
        dummy : gint;
     end;

     PGByteArray = ^TGByteArray;
     TGByteArray = record
          data : Pguint8;
          len : guint;
       end;

     PGPtrArray = ^TGPtrArray;
     TGPtrArray = record
          pdata : Pgpointer;
          len : guint;
       end;

     PGTuples = ^TGTuples;
     TGTuples = record
          len : guint;
       end;

type
   PGTraverseType = ^TGTraverseType;
   TGTraverseType = longint;
const
   G_IN_ORDER = 0;
   G_PRE_ORDER = 1;
   G_POST_ORDER = 2;

type
    TGTraverseFlags = longint;
const
    G_TRAVERSE_LEAFS = 1 shl 0;
    G_TRAVERSE_NON_LEAFS = 1 shl 1;
    G_TRAVERSE_ALL = 1 shl 0 + 1 shl 1;
    G_TRAVERSE_MASK = 3;

type
    TGLogLevelFlags = longint;
const
    G_LOG_FLAG_RECURSION = 1 shl 0;
    G_LOG_FLAG_FATAL = 1 shl 1;
    G_LOG_LEVEL_ERROR = 1 shl 2;
    G_LOG_LEVEL_CRITICAL = 1 shl 3;
    G_LOG_LEVEL_WARNING = 1 shl 4;
    G_LOG_LEVEL_MESSAGE = 1 shl 5;
    G_LOG_LEVEL_INFO = 1 shl 6;
    G_LOG_LEVEL_DEBUG = 1 shl 7;
    G_LOG_LEVEL_MASK = (1 shl 8) - 2;

    type
       PGNode = ^TGNode;
       TGNode = record
            data : gpointer;
            next : PGNode;
            prev : PGNode;
            parent : PGNode;
            children : PGNode;
         end;

    const
       G_HOOK_FLAG_USER_SHIFT = 4;

    type
       TGHookFlagMask = longint;
    const
       G_HOOK_FLAG_ACTIVE = 1 shl 0;
       G_HOOK_FLAG_IN_CALL = 1 shl 1;
       G_HOOK_FLAG_MASK = $0f;

    type
       PGHook = ^TGHook;
       PGHookList = ^TGHookList;

       TGHookCompareFunc = function (new_hook:PGHook; sibling:PGHook):gint;cdecl;
       TGHookFindFunc = function (hook:PGHook; data:gpointer):gboolean;cdecl;
       TGHookMarshaller = procedure (hook:PGHook; data:gpointer);cdecl;
       TGHookCheckMarshaller = function (hook:PGHook; data:gpointer):gboolean;cdecl;
       TGHookFunc = procedure (data:gpointer);cdecl;
       TGHookCheckFunc = function (data:gpointer):gboolean;cdecl;
       TGHookFreeFunc = procedure (hook_list:PGHookList; hook:PGHook);cdecl;

       TGDestroyNotify = procedure (data:gpointer);cdecl;

       TGHookList = record
            seq_id : guint;
            hook_size : guint;
            flag0 : word;
            hooks : PGHook;
            hook_memchunk : PGMemChunk;
            hook_free : TGHookFreeFunc;
            hook_destroy : TGHookFreeFunc;
         end;

       TGHook = record
            data : gpointer;
            next : PGHook;
            prev : PGHook;
            ref_count : guint;
            hook_id : guint;
            flags : guint;
            func : gpointer;
            destroy : TGDestroyNotify;
         end;

    const
       bm_TGHookList_is_setup = $1;
       bp_TGHookList_is_setup = 0;

const
   G_CSET_UA_2_Z = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
   G_CSET_a_2_z = 'abcdefghijklmnopqrstuvwxyz';

type
   PGErrorType = ^TGErrorType;
   TGErrorType = longint;
const
   G_ERR_UNKNOWN = 0;
   G_ERR_UNEXP_EOF = 1;
   G_ERR_UNEXP_EOF_IN_STRING = 2;
   G_ERR_UNEXP_EOF_IN_COMMENT = 3;
   G_ERR_NON_DIGIT_IN_CONST = 4;
   G_ERR_DIGIT_RADIX = 5;
   G_ERR_FLOAT_RADIX = 6;
   G_ERR_FLOAT_MALFORMED = 7;

type
  PGTokenType = ^TGTokenType;
  TGTokenType = longint;
const
    G_TOKEN_EOF                   = 0;
    G_TOKEN_LEFT_PAREN            = ord('(');
    G_TOKEN_RIGHT_PAREN           = ord(')');
    G_TOKEN_LEFT_CURLY            = ord('{');
    G_TOKEN_RIGHT_CURLY           = ord('}');
    G_TOKEN_LEFT_BRACE            = ord('[');
    G_TOKEN_RIGHT_BRACE           = ord(']');
    G_TOKEN_EQUAL_SIGN            = ord('=');
    G_TOKEN_COMMA                 = ord(',');

    G_TOKEN_NONE                  = 256;

    G_TOKEN_ERROR = 257;

    G_TOKEN_CHAR = 258;
    G_TOKEN_BINARY = 259;
    G_TOKEN_OCTAL = 260;
    G_TOKEN_INT = 261;
    G_TOKEN_HEX = 262;
    G_TOKEN_FLOAT = 263;
    G_TOKEN_STRING = 264;

    G_TOKEN_SYMBOL = 265;
    G_TOKEN_IDENTIFIER = 266;
    G_TOKEN_IDENTIFIER_NULL = 267;

    G_TOKEN_COMMENT_SINGLE = 268;
    G_TOKEN_COMMENT_MULTI = 269;
    G_TOKEN_LAST = 270;

type
   PGTokenValue = ^TGTokenValue;
   TGTokenValue = record
       case longint of
          0 : ( v_symbol:gpointer );
          1 : ( v_identifier:Pgchar );
          2 : ( v_binary:gulong );
          3 : ( v_octal:gulong );
          4 : ( v_int:gulong );
          5 : ( v_float:gdouble );
          6 : ( v_hex:gulong );
          7 : ( v_string:Pgchar );
          8 : ( v_comment:Pgchar );
          9 : ( v_char:guchar );
          10 : ( v_error:guint );
     end;

PGScannerConfig = ^TGScannerConfig;
TGScannerConfig = Record
         cset_skip_characters,
         cset_identifier_first,
         cset_identifier_nth,
         cpair_comment_single : Pgchar;
         flag0 : guint;
end;
   PGScanner = ^TGScanner;

  TGScannerMsgFunc = procedure (scanner:PGScanner; message:Pgchar; error:gint);cdecl;

   TGScanner = record
            user_data : gpointer;
            max_parse_errors : guint;
            parse_errors : guint;
            input_name : Pgchar;
            derived_data : gpointer;
            config : PGScannerConfig;
            token :TGTokenType;
            value : TGTokenValue;
            line : guint;
            position : guint;
            next_token :TGTokenType;
            next_value : TGTokenValue;
            next_line : guint;
            next_position : guint;
            symbol_table : PGHashTable;
            input_fd : gint;
            text : Pgchar;
            text_end : Pgchar;
            buffer : Pgchar;
            scope_id : guint;
            msg_handler :TGScannerMsgFunc;
     end;


    const
       bm_TGScannerConfig_case_sensitive = $1;
       bp_TGScannerConfig_case_sensitive = 0;
       bm_TGScannerConfig_skip_comment_multi = $2;
       bp_TGScannerConfig_skip_comment_multi = 1;
       bm_TGScannerConfig_skip_comment_single = $4;
       bp_TGScannerConfig_skip_comment_single = 2;
       bm_TGScannerConfig_scan_comment_multi = $8;
       bp_TGScannerConfig_scan_comment_multi = 3;
       bm_TGScannerConfig_scan_identifier = $10;
       bp_TGScannerConfig_scan_identifier = 4;
       bm_TGScannerConfig_scan_identifier_1char = $20;
       bp_TGScannerConfig_scan_identifier_1char = 5;
       bm_TGScannerConfig_scan_identifier_NULL = $40;
       bp_TGScannerConfig_scan_identifier_NULL = 6;
       bm_TGScannerConfig_scan_symbols = $80;
       bp_TGScannerConfig_scan_symbols = 7;
       bm_TGScannerConfig_scan_binary = $100;
       bp_TGScannerConfig_scan_binary = 8;
       bm_TGScannerConfig_scan_octal = $200;
       bp_TGScannerConfig_scan_octal = 9;
       bm_TGScannerConfig_scan_float = $400;
       bp_TGScannerConfig_scan_float = 10;
       bm_TGScannerConfig_scan_hex = $800;
       bp_TGScannerConfig_scan_hex = 11;
       bm_TGScannerConfig_scan_hex_dollar = $1000;
       bp_TGScannerConfig_scan_hex_dollar = 12;
       bm_TGScannerConfig_scan_string_sq = $2000;
       bp_TGScannerConfig_scan_string_sq = 13;
       bm_TGScannerConfig_scan_string_dq = $4000;
       bp_TGScannerConfig_scan_string_dq = 14;
       bm_TGScannerConfig_numbers_2_int = $8000;
       bp_TGScannerConfig_numbers_2_int = 15;
       bm_TGScannerConfig_int_2_float = $10000;
       bp_TGScannerConfig_int_2_float = 16;
       bm_TGScannerConfig_identifier_2_string = $20000;
       bp_TGScannerConfig_identifier_2_string = 17;
       bm_TGScannerConfig_char_2_token = $40000;
       bp_TGScannerConfig_char_2_token = 18;
       bm_TGScannerConfig_symbol_2_token = $80000;
       bp_TGScannerConfig_symbol_2_token = 19;
       bm_TGScannerConfig_scope_0_fallback = $100000;
       bp_TGScannerConfig_scope_0_fallback = 20;

type
   PGFunc = ^TGFunc;
   TGFunc = procedure(data:gpointer; user_data:gpointer);cdecl;
   PGHFunc = ^TGHFunc;
   TGHFunc = procedure(key:gpointer; value:gpointer; user_data:gpointer);cdecl;
   PGHRFunc = ^TGHRFunc;
   TGHRFunc = function (key:gpointer; value:gpointer; user_data:gpointer):gboolean;cdecl;
   PGHashFunc = ^TGHashFunc;
   TGHashFunc = function(key:gpointer):guint;cdecl;
   PGCompareFunc = ^TGCompareFunc;
   TGCompareFunc = function (a:gpointer; b:gpointer):gint;cdecl;
   PGCacheNewFunc = ^TGCacheNewFunc;
   TGCacheNewFunc = function(key:gpointer):gpointer;cdecl;
   PGCacheDupFunc = ^TGCacheDupFunc;
   TGCacheDupFunc = function(value:gpointer):gpointer;cdecl;
   PGCacheDestroyFunc = ^TGCacheDestroyFunc;
   TGCacheDestroyFunc = procedure(value:gpointer);cdecl;
   PGTraverseFunc = ^TGTraverseFunc;
   TGTraverseFunc = function(key:gpointer; value:gpointer; data:gpointer):gint;cdecl;
   PGSearchFunc = ^TGSearchFunc;
   TGSearchFunc = function(key:gpointer; data:gpointer):gint;cdecl;
   PGErrorFunc = ^TGErrorFunc;
   TGErrorFunc = procedure(str:Pgchar);cdecl;
   PGWarningFunc = ^TGWarningFunc;
   TGWarningFunc = procedure(str:Pgchar);cdecl;
   PGPrintFunc = ^TGPrintFunc;
   TGPrintFunc = procedure(str:Pgchar);cdecl;

   TGCompletionFunc = function (_para1:gpointer):Pgchar;cdecl;
   TGDataForeachFunc = procedure (key_id:TGQuark; data:gpointer; user_data:gpointer);cdecl;
   TGFreeFunc = procedure (data:gpointer);cdecl;
   TGLogFunc = procedure (log_domain:Pgchar; log_level:TGLogLevelFlags; message:Pgchar; user_data:gpointer);cdecl;
   TGNodeTraverseFunc = function (node:PGNode; data:gpointer):gboolean;cdecl;
   TGNodeForeachFunc = procedure (node:PGNode; data:gpointer);cdecl;
   TGVoidFunc = procedure ;cdecl;

   PGallocator = pointer;
   PGrelation  = pointer;
   PGData      = pointer;
   PPGData     = ^PGData;
   PTm         = pointer;
   PGMainloop  = pointer;
   PGMutex     = pointer;

{$ifndef gtkwin}
procedure g_list_push_allocator(allocator:PGAllocator);cdecl;external glibdll name 'g_list_push_allocator';
{$endif}
procedure g_list_pop_allocator;cdecl;external glibdll name 'g_list_pop_allocator';
function  g_list_alloc:PGList;cdecl;external glibdll name 'g_list_alloc';
procedure g_list_free(list:PGList);cdecl;external glibdll name 'g_list_free';
procedure g_list_free_1(list:PGList);cdecl;external glibdll name 'g_list_free_1';
function  g_list_append(list:PGList; data:gpointer):PGList;cdecl;external glibdll name 'g_list_append';
function  g_list_prepend(list:PGList; data:gpointer):PGList;cdecl;external glibdll name 'g_list_prepend';
function  g_list_insert(list:PGList; data:gpointer; position:gint):PGList;cdecl;external glibdll name 'g_list_insert';
function  g_list_insert_sorted(list:PGList; data:gpointer; func:TGCompareFunc):PGList;cdecl;external glibdll name 'g_list_insert_sorted';
function  g_list_concat(list1:PGList; list2:PGList):PGList;cdecl;external glibdll name 'g_list_concat';
function  g_list_remove(list:PGList; data:gpointer):PGList;cdecl;external glibdll name 'g_list_remove';
function  g_list_remove_link(list:PGList; link:PGList):PGList;cdecl;external glibdll name 'g_list_remove_link';
function  g_list_reverse(list:PGList):PGList;cdecl;external glibdll name 'g_list_reverse';
function  g_list_copy(list:PGList):PGList;cdecl;external glibdll name 'g_list_copy';
function  g_list_nth(list:PGList; n:guint):PGList;cdecl;external glibdll name 'g_list_nth';
function  g_list_find(list:PGList; data:gpointer):PGList;cdecl;external glibdll name 'g_list_find';
function  g_list_find_custom(list:PGList; data:gpointer; func:TGCompareFunc):PGList;cdecl;external glibdll name 'g_list_find_custom';
function  g_list_index(list:PGList; data:gpointer):gint;cdecl;external glibdll name 'g_list_index';
function  g_list_position(list:PGList; llink:PGList):gint;cdecl;external glibdll name 'g_list_position';
function  g_list_last(list:PGList):PGList;cdecl;external glibdll name 'g_list_last';
function  g_list_first(list:PGList):PGList;cdecl;external glibdll name 'g_list_first';
function  g_list_length(list:PGList):guint;cdecl;external glibdll name 'g_list_length';
procedure g_list_foreach(list:PGList; func:TGFunc; user_data:gpointer);cdecl;external glibdll name 'g_list_foreach';
function  g_list_sort(list:PGList; compare_func:TGCompareFunc):PGList;cdecl;external glibdll name 'g_list_sort';
function  g_list_nth_data(list:PGList; n:guint):gpointer;cdecl;external glibdll name 'g_list_nth_data';
function  g_list_previous(list : PGlist) : PGlist;
function  g_list_next(list : PGlist) : PGlist;

procedure g_slist_push_allocator(allocator:PGAllocator);cdecl;external glibdll name 'g_slist_push_allocator';
procedure g_slist_pop_allocator;cdecl;external glibdll name 'g_slist_pop_allocator';
function  g_slist_alloc:PGSList;cdecl;external glibdll name 'g_slist_alloc';
procedure g_slist_free(list:PGSList);cdecl;external glibdll name 'g_slist_free';
procedure g_slist_free_1(list:PGSList);cdecl;external glibdll name 'g_slist_free_1';
function  g_slist_append(list:PGSList; data:gpointer):PGSList;cdecl;external glibdll name 'g_slist_append';
function  g_slist_prepend(list:PGSList; data:gpointer):PGSList;cdecl;external glibdll name 'g_slist_prepend';
function  g_slist_insert(list:PGSList; data:gpointer; position:gint):PGSList;cdecl;external glibdll name 'g_slist_insert';
function  g_slist_insert_sorted(list:PGSList; data:gpointer; func:TGCompareFunc):PGSList;cdecl;external glibdll name 'g_slist_insert_sorted';
function  g_slist_concat(list1:PGSList; list2:PGSList):PGSList;cdecl;external glibdll name 'g_slist_concat';
function  g_slist_remove(list:PGSList; data:gpointer):PGSList;cdecl;external glibdll name 'g_slist_remove';
function  g_slist_remove_link(list:PGSList; link:PGSList):PGSList;cdecl;external glibdll name 'g_slist_remove_link';
function  g_slist_reverse(list:PGSList):PGSList;cdecl;external glibdll name 'g_slist_reverse';
function  g_slist_copy(list:PGSList):PGSList;cdecl;external glibdll name 'g_slist_copy';
function  g_slist_nth(list:PGSList; n:guint):PGSList;cdecl;external glibdll name 'g_slist_nth';
function  g_slist_find(list:PGSList; data:gpointer):PGSList;cdecl;external glibdll name 'g_slist_find';
function  g_slist_find_custom(list:PGSList; data:gpointer; func:TGCompareFunc):PGSList;cdecl;external glibdll name 'g_slist_find_custom';
function  g_slist_position(list:PGSList; llink:PGSList):gint;cdecl;external glibdll name 'g_slist_position';
function  g_slist_index(list:PGSList; data:gpointer):gint;cdecl;external glibdll name 'g_slist_index';
function  g_slist_last(list:PGSList):PGSList;cdecl;external glibdll name 'g_slist_last';
function  g_slist_length(list:PGSList):guint;cdecl;external glibdll name 'g_slist_length';
procedure g_slist_foreach(list:PGSList; func:TGFunc; user_data:gpointer);cdecl;external glibdll name 'g_slist_foreach';
function  g_slist_sort(list:PGSList; compare_func:TGCompareFunc):PGSList;cdecl;external glibdll name 'g_slist_sort';
function  g_slist_nth_data(list:PGSList; n:guint):gpointer;cdecl;external glibdll name 'g_slist_nth_data';
function  g_slist_next(slist : PGSList) : PGSList;

function  g_hash_table_new(hash_func:TGHashFunc; key_compare_func:TGCompareFunc):PGHashTable;cdecl;external glibdll name 'g_hash_table_new';
procedure g_hash_table_destroy(hash_table:PGHashTable);cdecl;external glibdll name 'g_hash_table_destroy';
procedure g_hash_table_insert(hash_table:PGHashTable; key:gpointer; value:gpointer);cdecl;external glibdll name 'g_hash_table_insert';
procedure g_hash_table_remove(hash_table:PGHashTable; key:gpointer);cdecl;external glibdll name 'g_hash_table_remove';
function  g_hash_table_lookup(hash_table:PGHashTable; key:gpointer):gpointer;cdecl;external glibdll name 'g_hash_table_lookup';
function  g_hash_table_lookup_extended(hash_table:PGHashTable; lookup_key:gconstpointer; orig_key:Pgpointer; value:Pgpointer):gboolean;cdecl;external glibdll name 'g_hash_table_lookup_extended';
procedure g_hash_table_freeze(hash_table:PGHashTable);cdecl;external glibdll name 'g_hash_table_freeze';
procedure g_hash_table_thaw(hash_table:PGHashTable);cdecl;external glibdll name 'g_hash_table_thaw';
procedure g_hash_table_foreach(hash_table:PGHashTable; func:TGHFunc; user_data:gpointer);cdecl;external glibdll name 'g_hash_table_foreach';
function  g_hash_table_foreach_remove(hash_table:PGHashTable; func:TGHRFunc; user_data:gpointer):guint;cdecl;external glibdll name 'g_hash_table_foreach_remove';
function  g_hash_table_size(hash_table:PGHashTable):guint;cdecl;external glibdll name 'g_hash_table_size';

function  g_cache_new(value_new_func:TGCacheNewFunc; value_destroy_func:TGCacheDestroyFunc;
                      key_dup_func:TGCacheDupFunc; key_destroy_func:TGCacheDestroyFunc;
                      hash_key_func:TGHashFunc; hash_value_func:TGHashFunc; key_compare_func:TGCompareFunc):PGCache;
                      cdecl;external glibdll name 'g_cache_new';
procedure g_cache_destroy(cache:PGCache);cdecl;external glibdll name 'g_cache_destroy';
function  g_cache_insert(cache:PGCache; key:gpointer):gpointer;cdecl;external glibdll name 'g_cache_insert';
procedure g_cache_remove(cache:PGCache; value:gpointer);cdecl;external glibdll name 'g_cache_remove';
procedure g_cache_key_foreach(cache:PGCache; func:TGHFunc; user_data:gpointer);cdecl;external glibdll name 'g_cache_key_foreach';
procedure g_cache_value_foreach(cache:PGCache; func:TGHFunc; user_data:gpointer);cdecl;external glibdll name 'g_cache_value_foreach';

function  g_tree_new(key_compare_func:TGCompareFunc):PGTree;cdecl;external glibdll name 'g_tree_new';
procedure g_tree_destroy(tree:PGTree);cdecl;external glibdll name 'g_tree_destroy';
procedure g_tree_insert(tree:PGTree; key:gpointer; value:gpointer);cdecl;external glibdll name 'g_tree_insert';
procedure g_tree_remove(tree:PGTree; key:gpointer);cdecl;external glibdll name 'g_tree_remove';
function  g_tree_lookup(tree:PGTree; key:gpointer):gpointer;cdecl;external glibdll name 'g_tree_lookup';
procedure g_tree_traverse(tree:PGTree; traverse_func:TGTraverseFunc; traverse_type:TGTraverseType; data:gpointer);cdecl;external glibdll name 'g_tree_traverse';
function  g_tree_search(tree:PGTree; search_func:TGSearchFunc; data:gpointer):gpointer;cdecl;external glibdll name 'g_tree_search';
function  g_tree_height(tree:PGTree):gint;cdecl;external glibdll name 'g_tree_height';
function  g_tree_nnodes(tree:PGTree):gint;cdecl;external glibdll name 'g_tree_nnodes';

function  G_NODE_IS_ROOT(node : PGNode) : boolean;
function  G_NODE_IS_LEAF(node : PGNode) : boolean;
procedure g_node_push_allocator(allocator:PGAllocator);cdecl;external glibdll name 'g_node_push_allocator';
procedure g_node_pop_allocator;cdecl;external glibdll name 'g_node_pop_allocator';
function  g_node_new(data:gpointer):PGNode;cdecl;external glibdll name 'g_node_new';
procedure g_node_destroy(root:PGNode);cdecl;external glibdll name 'g_node_destroy';
procedure g_node_unlink(node:PGNode);cdecl;external glibdll name 'g_node_unlink';
function  g_node_insert(parent:PGNode; position:gint; node:PGNode):PGNode;cdecl;external glibdll name 'g_node_insert';
function  g_node_insert_before(parent:PGNode; sibling:PGNode; node:PGNode):PGNode;cdecl;external glibdll name 'g_node_insert_before';
function  g_node_prepend(parent:PGNode; node:PGNode):PGNode;cdecl;external glibdll name 'g_node_prepend';
function  g_node_n_nodes(root:PGNode; flags:TGTraverseFlags):guint;cdecl;external glibdll name 'g_node_n_nodes';
function  g_node_get_root(node:PGNode):PGNode;cdecl;external glibdll name 'g_node_get_root';
function  g_node_is_ancestor(node:PGNode; descendant:PGNode):gboolean;cdecl;external glibdll name 'g_node_is_ancestor';
function  g_node_depth(node:PGNode):guint;cdecl;external glibdll name 'g_node_depth';
function  g_node_find(root:PGNode; order:TGTraverseType; flags:TGTraverseFlags; data:gpointer):PGNode;cdecl;external glibdll name 'g_node_find';
function  g_node_append(parent,node : PGNode) : PGNode;
function  g_node_insert_data(parent:PGNode;position:gint;data : pointer) : PGNode;
function  g_node_insert_data_before(parent,sibling:PGNode;data : pointer) : PGNode;
function  g_node_prepend_data(parent:PGNode;data : pointer) : PGNode;
function  g_node_append_data(parent:PGNode;data : pointer) : PGNode;
procedure g_node_traverse(root:PGNode; order:TGTraverseType; flags:TGTraverseFlags; max_depth:gint; func:TGNodeTraverseFunc; data:gpointer);cdecl;external glibdll name 'g_node_traverse';
function  g_node_max_height(root:PGNode):guint;cdecl;external glibdll name 'g_node_max_height';
procedure g_node_children_foreach(node:PGNode; flags:TGTraverseFlags; func:TGNodeForeachFunc; data:gpointer);cdecl;external glibdll name 'g_node_children_foreach';
procedure g_node_reverse_children(node:PGNode);cdecl;external glibdll name 'g_node_reverse_children';
function  g_node_n_children(node:PGNode):guint;cdecl;external glibdll name 'g_node_n_children';
function  g_node_nth_child(node:PGNode; n:guint):PGNode;cdecl;external glibdll name 'g_node_nth_child';
function  g_node_last_child(node:PGNode):PGNode;cdecl;external glibdll name 'g_node_last_child';
function  g_node_find_child(node:PGNode; flags:TGTraverseFlags; data:gpointer):PGNode;cdecl;external glibdll name 'g_node_find_child';
function  g_node_child_position(node:PGNode; child:PGNode):gint;cdecl;external glibdll name 'g_node_child_position';
function  g_node_child_index(node:PGNode; data:gpointer):gint;cdecl;external glibdll name 'g_node_child_index';
function  g_node_first_sibling(node:PGNode):PGNode;cdecl;external glibdll name 'g_node_first_sibling';
function  g_node_last_sibling(node:PGNode):PGNode;cdecl;external glibdll name 'g_node_last_sibling';
function  g_node_prev_sibling(node : PGnode) : PGNode;
function  g_node_next_sibling(node : PGnode) : PGnode;
function  g_node_first_child(node : PGnode) : PGnode;

function  is_setup(var a : TGHookList) : guint;
procedure set_is_setup(var a : TGHookList; __is_setup : guint);
function  G_HOOK_DEFERRED_DESTROY : TGHookFreeFunc;
function  G_HOOK_ACTIVE(hook : PGHook) : boolean;
function  G_HOOK_IN_CALL(hook : PGHook) : boolean;
function  G_HOOK_IS_VALID(hook : PGHook) : boolean;
function  G_HOOK_IS_UNLINKED(hook : PGHook) : boolean;

procedure g_hook_list_init(hook_list:PGHookList; hook_size:guint);cdecl;external glibdll name 'g_hook_list_init';
procedure g_hook_list_clear(hook_list:PGHookList);cdecl;external glibdll name 'g_hook_list_clear';
function  g_hook_alloc(hook_list:PGHookList):PGHook;cdecl;external glibdll name 'g_hook_alloc';
procedure g_hook_free(hook_list:PGHookList; hook:PGHook);cdecl;external glibdll name 'g_hook_free';
procedure g_hook_ref(hook_list:PGHookList; hook:PGHook);cdecl;external glibdll name 'g_hook_ref';
procedure g_hook_unref(hook_list:PGHookList; hook:PGHook);cdecl;external glibdll name 'g_hook_unref';
function  g_hook_destroy(hook_list:PGHookList; hook_id:guint):gboolean;cdecl;external glibdll name 'g_hook_destroy';
procedure g_hook_destroy_link(hook_list:PGHookList; hook:PGHook);cdecl;external glibdll name 'g_hook_destroy_link';
procedure g_hook_prepend(hook_list:PGHookList; hook:PGHook);cdecl;external glibdll name 'g_hook_prepend';
procedure g_hook_insert_before(hook_list:PGHookList; sibling:PGHook; hook:PGHook);cdecl;external glibdll name 'g_hook_insert_before';
procedure g_hook_insert_sorted(hook_list:PGHookList; hook:PGHook; func:TGHookCompareFunc);cdecl;external glibdll name 'g_hook_insert_sorted';
function  g_hook_get(hook_list:PGHookList; hook_id:guint):PGHook;cdecl;external glibdll name 'g_hook_get';
function  g_hook_find(hook_list:PGHookList; need_valids:gboolean; func:TGHookFindFunc; data:gpointer):PGHook;cdecl;external glibdll name 'g_hook_find';
function  g_hook_find_data(hook_list:PGHookList; need_valids:gboolean; data:gpointer):PGHook;cdecl;external glibdll name 'g_hook_find_data';
function  g_hook_find_func(hook_list:PGHookList; need_valids:gboolean; func:gpointer):PGHook;cdecl;external glibdll name 'g_hook_find_func';
function  g_hook_find_func_data(hook_list:PGHookList; need_valids:gboolean; func:gpointer; data:gpointer):PGHook;cdecl;external glibdll name 'g_hook_find_func_data';
function  g_hook_first_valid(hook_list:PGHookList; may_be_in_call:gboolean):PGHook;cdecl;external glibdll name 'g_hook_first_valid';
function  g_hook_next_valid(hook_list:PGHookList; hook:PGHook; may_be_in_call:gboolean):PGHook;cdecl;external glibdll name 'g_hook_next_valid';
function  g_hook_compare_ids(new_hook:PGHook; sibling:PGHook):gint;cdecl;external glibdll name 'g_hook_compare_ids';
procedure g_hook_append(hook_list:PGhooklist;hook : PGhook);
procedure g_hook_list_invoke(hook_list:PGHookList; may_recurse:gboolean);cdecl;external glibdll name 'g_hook_list_invoke';
procedure g_hook_list_invoke_check(hook_list:PGHookList; may_recurse:gboolean);cdecl;external glibdll name 'g_hook_list_invoke_check';
procedure g_hook_list_marshal(hook_list:PGHookList; may_recurse:gboolean; marshaller:TGHookMarshaller; data:gpointer);cdecl;external glibdll name 'g_hook_list_marshal';
procedure g_hook_list_marshal_check(hook_list:PGHookList; may_recurse:gboolean; marshaller:TGHookCheckMarshaller; data:gpointer);cdecl;external glibdll name 'g_hook_list_marshal_check';

procedure g_on_error_query(prg_name:Pgchar);cdecl;external glibdll name 'g_on_error_query';
procedure g_on_error_stack_trace(prg_name:Pgchar);cdecl;external glibdll name 'g_on_error_stack_trace';

{$ifndef gtkwin}
  {$ifndef gtkos2}
var
  g_log_domain_glib : Pgchar;external glibdll name 'g_log_domain_glib';
  {$endif}
{$endif}

function  g_log_set_handler(log_domain:Pgchar; log_levels:TGLogLevelFlags; log_func:TGLogFunc; user_data:gpointer):guint;cdecl;external glibdll name 'g_log_set_handler';
procedure g_log_remove_handler(log_domain:Pgchar; handler_id:guint);cdecl;external glibdll name 'g_log_remove_handler';
procedure g_log_default_handler(log_domain:Pgchar; log_level:TGLogLevelFlags; message:Pgchar; unused_data:gpointer);cdecl;external glibdll name 'g_log_default_handler';
procedure g_logv(log_domain:Pgchar; log_level:TGLogLevelFlags; format:Pgchar; args:array of const);cdecl;external glibdll name 'g_logv';
function  g_log_set_fatal_mask(log_domain:Pgchar; fatal_mask:TGLogLevelFlags):TGLogLevelFlags;cdecl;external glibdll name 'g_log_set_fatal_mask';
function  g_log_set_always_fatal(fatal_mask:TGLogLevelFlags):TGLogLevelFlags;cdecl;external glibdll name 'g_log_set_always_fatal';

function  g_set_printerr_handler(func:TGPrintFunc):TGPrintFunc;cdecl;external glibdll name 'g_set_printerr_handler';
function  g_set_print_handler(func:TGPrintFunc):TGPrintFunc;cdecl;external glibdll name 'g_set_print_handler';
function  g_set_error_handler(func:TGErrorFunc):TGErrorFunc;cdecl;external glibdll name 'g_set_error_handler';
function  g_set_warning_handler(func:TGWarningFunc):TGWarningFunc;cdecl;external glibdll name 'g_set_warning_handler';
function  g_set_message_handler(func:TGPrintFunc):TGPrintFunc;cdecl;external glibdll name 'g_set_message_handler';

function  g_malloc(size:gulong):gpointer;cdecl;external glibdll name 'g_malloc';
function  g_malloc0(size:gulong):gpointer;cdecl;external glibdll name 'g_malloc0';
function  g_realloc(mem:gpointer; size:gulong):gpointer;cdecl;external glibdll name 'g_realloc';
procedure g_free(mem:gpointer);cdecl;external glibdll name 'g_free';
procedure g_mem_profile;cdecl;external glibdll name 'g_mem_profile';
procedure g_mem_check(mem:gpointer);cdecl;external glibdll name 'g_mem_check';

    const
       G_ALLOCATOR_LIST = 1;
       G_ALLOCATOR_SLIST = 2;
       G_ALLOCATOR_NODE = 3;

function  g_allocator_new(name:Pgchar; n_preallocs:guint):PGAllocator;cdecl;external glibdll name 'g_allocator_new';
procedure g_allocator_free(allocator:PGAllocator);cdecl;external glibdll name 'g_allocator_free';

   const
       G_ALLOC_ONLY = 1;
       G_ALLOC_AND_FREE = 2;

function  g_mem_chunk_new(name:Pgchar; atom_size:gint; area_size:gulong; thetype:gint):PGMemChunk;cdecl;external glibdll name 'g_mem_chunk_new';
procedure g_mem_chunk_destroy(mem_chunk:PGMemChunk);cdecl;external glibdll name 'g_mem_chunk_destroy';
function  g_mem_chunk_alloc(mem_chunk:PGMemChunk):gpointer;cdecl;external glibdll name 'g_mem_chunk_alloc';
function  g_mem_chunk_alloc0(mem_chunk:PGMemChunk):gpointer;cdecl;external glibdll name 'g_mem_chunk_alloc0';
procedure g_mem_chunk_free(mem_chunk:PGMemChunk; mem:gpointer);cdecl;external glibdll name 'g_mem_chunk_free';
procedure g_mem_chunk_clean(mem_chunk:PGMemChunk);cdecl;external glibdll name 'g_mem_chunk_clean';
procedure g_mem_chunk_reset(mem_chunk:PGMemChunk);cdecl;external glibdll name 'g_mem_chunk_reset';
procedure g_mem_chunk_print(mem_chunk:PGMemChunk);cdecl;external glibdll name 'g_mem_chunk_print';
procedure g_mem_chunk_info;cdecl;external glibdll name 'g_mem_chunk_info';
procedure g_blow_chunks;cdecl;external glibdll name 'g_blow_chunks';
function  g_timer_new:PGTimer;cdecl;external glibdll name 'g_timer_new';
procedure g_timer_destroy(timer:PGTimer);cdecl;external glibdll name 'g_timer_destroy';
procedure g_timer_start(timer:PGTimer);cdecl;external glibdll name 'g_timer_start';
procedure g_timer_stop(timer:PGTimer);cdecl;external glibdll name 'g_timer_stop';
procedure g_timer_reset(timer:PGTimer);cdecl;external glibdll name 'g_timer_reset';
function  g_timer_elapsed(timer:PGTimer; microseconds:Pgulong):gdouble;cdecl;external glibdll name 'g_timer_elapsed';

const
       G_STR_DELIMITERS = '_-|> <.';

function  g_strdelimit(thestring:Pgchar; delimiters:Pgchar; new_delimiter:gchar):Pgchar;cdecl;external glibdll name 'g_strdelimit';
function  g_strtod(nptr:Pgchar; endptr:PPgchar):gdouble;cdecl;external glibdll name 'g_strtod';
function  g_strerror(errnum:gint):Pgchar;cdecl;external glibdll name 'g_strerror';
function  g_strsignal(signum:gint):Pgchar;cdecl;external glibdll name 'g_strsignal';
function  g_strcasecmp(s1:Pgchar; s2:Pgchar):gint;cdecl;external glibdll name 'g_strcasecmp';
function  g_strncasecmp(s1:Pgchar; s2:Pgchar; n:guint):gint;cdecl;external glibdll name 'g_strncasecmp';
procedure g_strdown(thestring:Pgchar);cdecl;external glibdll name 'g_strdown';
procedure g_strup(thestring:Pgchar);cdecl;external glibdll name 'g_strup';
procedure g_strreverse(thestring:Pgchar);cdecl;external glibdll name 'g_strreverse';
function  g_strchug(thestring:Pgchar):Pgchar;cdecl;external glibdll name 'g_strchug';
function  g_strchomp(thestring:Pgchar):Pgchar;cdecl;external glibdll name 'g_strchomp';
function  g_strstrip(thestring : Pgchar) : Pgchar;
function  g_strdup(str:Pgchar):Pgchar;cdecl;external glibdll name 'g_strdup';
function  g_strdup_printf(format:Pgchar;args:array of const):Pgchar;cdecl;external glibdll name 'g_strdup_printf';
function  g_strdup_vprintf(format:Pgchar; args:array of const):Pgchar;cdecl;external glibdll name 'g_strdup_vprintf';
function  g_strndup(str:Pgchar; n:guint):Pgchar;cdecl;external glibdll name 'g_strndup';
function  g_strnfill(length:guint; fill_char:gchar):Pgchar;cdecl;external glibdll name 'g_strnfill';
function  g_strconcat(string1:Pgchar; args:array of const):Pgchar;cdecl;external glibdll name 'g_strconcat';
function  g_strjoin(separator:Pgchar; args:array of const):Pgchar;cdecl;external glibdll name 'g_strjoin';
function  g_strescape(thestring:Pgchar):Pgchar;cdecl;external glibdll name 'g_strescape';
function  g_memdup(mem:gconstpointer; byte_size:guint):gpointer;cdecl;external glibdll name 'g_memdup';
function  g_strsplit(thestring:Pgchar; delimiter:Pgchar; max_tokens:gint):PPgchar;cdecl;external glibdll name 'g_strsplit';
function  g_strjoinv(separator:Pgchar; str_array:PPgchar):Pgchar;cdecl;external glibdll name 'g_strjoinv';
procedure g_strfreev(str_array:PPgchar);cdecl;external glibdll name 'g_strfreev';
function  g_printf_string_upper_bound(format:Pgchar; args:array of const):guint;cdecl;external glibdll name 'g_printf_string_upper_bound';
function  g_get_user_name:Pgchar;cdecl;external glibdll name 'g_get_user_name';
function  g_get_real_name:Pgchar;cdecl;external glibdll name 'g_get_real_name';
function  g_get_home_dir:Pgchar;cdecl;external glibdll name 'g_get_home_dir';
function  g_get_tmp_dir:Pgchar;cdecl;external glibdll name 'g_get_tmp_dir';
function  g_get_prgname:Pgchar;cdecl;external glibdll name 'g_get_prgname';
procedure g_set_prgname(prgname:Pgchar);cdecl;external glibdll name 'g_set_prgname';
function  g_parse_debug_string(thestring:Pgchar; keys:PGDebugKey; nkeys:guint):guint;cdecl;external glibdll name 'g_parse_debug_string';
function  g_vsnprintf(thestring:Pgchar; n:gulong; format:Pgchar; args:array of const):gint;cdecl;external glibdll name 'g_vsnprintf';
function  g_basename(file_name:Pgchar):Pgchar;cdecl;external glibdll name 'g_basename';
function  g_path_is_absolute(file_name:Pgchar):gboolean;cdecl;external glibdll name 'g_path_is_absolute';
function  g_path_skip_root(file_name:Pgchar):Pgchar;cdecl;external glibdll name 'g_path_skip_root';
function  g_dirname(file_name:Pgchar):Pgchar;cdecl;external glibdll name 'g_dirname';
function  g_get_current_dir:Pgchar;cdecl;external glibdll name 'g_get_current_dir';
function  g_getenv(variable:Pgchar):Pgchar;cdecl;external glibdll name 'g_getenv';
procedure g_atexit(func:TGVoidFunc);cdecl;external glibdll name 'g_atexit';

function  g_string_chunk_new(size:gint):PGStringChunk;cdecl;external glibdll name 'g_string_chunk_new';
procedure g_string_chunk_free(chunk:PGStringChunk);cdecl;external glibdll name 'g_string_chunk_free';
function  g_string_chunk_insert(chunk:PGStringChunk; thestring:Pgchar):Pgchar;cdecl;external glibdll name 'g_string_chunk_insert';
function  g_string_chunk_insert_const(chunk:PGStringChunk; thestring:Pgchar):Pgchar;cdecl;external glibdll name 'g_string_chunk_insert_const';
function  g_string_new(init:Pgchar):PGString;cdecl;external glibdll name 'g_string_new';
function  g_string_sized_new(dfl_size:guint):PGString;cdecl;external glibdll name 'g_string_sized_new';
procedure g_string_free(thestring:PGString; free_segment:gint);cdecl;external glibdll name 'g_string_free';
function  g_string_assign(lval:PGString; rval:Pgchar):PGString;cdecl;external glibdll name 'g_string_assign';
function  g_string_truncate(thestring:PGString; len:gint):PGString;cdecl;external glibdll name 'g_string_truncate';
function  g_string_append(thestring:PGString; val:Pgchar):PGString;cdecl;external glibdll name 'g_string_append';
function  g_string_append_c(thestring:PGString; c:gchar):PGString;cdecl;external glibdll name 'g_string_append_c';
function  g_string_prepend(thestring:PGString; val:Pgchar):PGString;cdecl;external glibdll name 'g_string_prepend';
function  g_string_prepend_c(thestring:PGString; c:gchar):PGString;cdecl;external glibdll name 'g_string_prepend_c';
function  g_string_insert(thestring:PGString; pos:gint; val:Pgchar):PGString;cdecl;external glibdll name 'g_string_insert';
function  g_string_insert_c(thestring:PGString; pos:gint; c:gchar):PGString;cdecl;external glibdll name 'g_string_insert_c';
function  g_string_erase(thestring:PGString; pos:gint; len:gint):PGString;cdecl;external glibdll name 'g_string_erase';
function  g_string_down(thestring:PGString):PGString;cdecl;external glibdll name 'g_string_down';
function  g_string_up(thestring:PGString):PGString;cdecl;external glibdll name 'g_string_up';
procedure g_string_sprintf(thestring:PGString; fmt:Pgchar; args:array of const);cdecl;external glibdll name 'g_string_sprintf';
procedure g_string_sprintfa(thestring:PGString; fmt:Pgchar; args:array of const);cdecl;external glibdll name 'g_string_sprintfa';

function  g_array_append_val(a:PGarray;v : longint) : PGarray;
function  g_array_prepend_val(a:PGarray;v : longint) : PGarray;
function  g_array_insert_val(a:PGarray;i:guint;v : longint) : PGarray;
function  g_array_new(zero_terminated:gint):PGArray;cdecl;external glibdll name 'g_array_new';
procedure g_array_free(thearray:PGArray; free_segment:gint);cdecl;external glibdll name 'g_array_free';
function  g_array_append_vals(thearray:PGArray; data:gconstpointer; len:guint):PGArray;cdecl;external glibdll name 'g_array_append_vals';
function  g_array_prepend_vals(thearray:PGArray; data:gconstpointer; len:guint):PGArray;cdecl;external glibdll name 'g_array_prepend_vals';
function  g_array_insert_vals(thearray:PGArray; index:guint; data:gconstpointer; len:guint):PGArray;cdecl;external glibdll name 'g_array_insert_vals';
function  g_array_set_size(thearray:PGArray; length:guint):PGArray;cdecl;external glibdll name 'g_array_set_size';
function  g_array_remove_index(thearray:PGArray; index:guint):PGArray;cdecl;external glibdll name 'g_array_remove_index';
function  g_array_remove_index_fast(thearray:PGArray; index:guint):PGArray;cdecl;external glibdll name 'g_array_remove_index_fast';

function  g_ptr_array_new:PGPtrArray;cdecl;external glibdll name 'g_ptr_array_new';
procedure g_ptr_array_free(thearray:PGPtrArray; free_seg:gboolean);cdecl;external glibdll name 'g_ptr_array_free';
procedure g_ptr_array_set_size(thearray:PGPtrArray; length:gint);cdecl;external glibdll name 'g_ptr_array_set_size';
function  g_ptr_array_remove_index(thearray:PGPtrArray; index:guint):gpointer;cdecl;external glibdll name 'g_ptr_array_remove_index';
function  g_ptr_array_remove_index_fast(thearray:PGPtrArray; index:guint):gpointer;cdecl;external glibdll name 'g_ptr_array_remove_index_fast';
function  g_ptr_array_remove(thearray:PGPtrArray; data:gpointer):gboolean;cdecl;external glibdll name 'g_ptr_array_remove';
function  g_ptr_array_remove_fast(thearray:PGPtrArray; data:gpointer):gboolean;cdecl;external glibdll name 'g_ptr_array_remove_fast';
procedure g_ptr_array_add(thearray:PGPtrArray; data:gpointer);cdecl;external glibdll name 'g_ptr_array_add';

function  g_byte_array_new:PGByteArray;cdecl;external glibdll name 'g_byte_array_new';
procedure g_byte_array_free(thearray:PGByteArray; free_segment:gboolean);cdecl;external glibdll name 'g_byte_array_free';
function  g_byte_array_append(thearray:PGByteArray; data:Pguint8; len:guint):PGByteArray;cdecl;external glibdll name 'g_byte_array_append';
function  g_byte_array_prepend(thearray:PGByteArray; data:Pguint8; len:guint):PGByteArray;cdecl;external glibdll name 'g_byte_array_prepend';
function  g_byte_array_set_size(thearray:PGByteArray; length:guint):PGByteArray;cdecl;external glibdll name 'g_byte_array_set_size';
function  g_byte_array_remove_index(thearray:PGByteArray; index:guint):PGByteArray;cdecl;external glibdll name 'g_byte_array_remove_index';
function  g_byte_array_remove_index_fast(thearray:PGByteArray; index:guint):PGByteArray;cdecl;external glibdll name 'g_byte_array_remove_index_fast';

function  g_str_equal(v:gconstpointer; v2:gconstpointer):gint;cdecl;external glibdll name 'g_str_equal';
function  g_str_hash(v:gconstpointer):guint;cdecl;external glibdll name 'g_str_hash';

function  g_int_equal(v:gconstpointer; v2:gconstpointer):gint;cdecl;external glibdll name 'g_int_equal';
function  g_int_hash(v:gconstpointer):guint;cdecl;external glibdll name 'g_int_hash';

function  g_direct_hash(v:gconstpointer):guint;cdecl;external glibdll name 'g_direct_hash';
function  g_direct_equal(v:gconstpointer; v2:gconstpointer):gint;cdecl;external glibdll name 'g_direct_equal';

function  g_quark_try_string(thestring:Pgchar):TGQuark;cdecl;external glibdll name 'g_quark_try_string';
function  g_quark_from_static_string(thestring:Pgchar):TGQuark;cdecl;external glibdll name 'g_quark_from_static_string';
function  g_quark_from_string(thestring:Pgchar):TGQuark;cdecl;external glibdll name 'g_quark_from_string';
function  g_quark_to_string(quark:TGQuark):Pgchar;cdecl;external glibdll name 'g_quark_to_string';

procedure g_datalist_init(datalist:PPGData);cdecl;external glibdll name 'g_datalist_init';
procedure g_datalist_clear(datalist:PPGData);cdecl;external glibdll name 'g_datalist_clear';
function  g_datalist_id_get_data(datalist:PPGData; key_id:TGQuark):gpointer;cdecl;external glibdll name 'g_datalist_id_get_data';
procedure g_datalist_id_set_data_full(datalist:PPGData; key_id:TGQuark; data:gpointer; destroy_func:TGDestroyNotify);cdecl;external glibdll name 'g_datalist_id_set_data_full';
procedure g_datalist_id_remove_no_notify(datalist:PPGData; key_id:TGQuark);cdecl;external glibdll name 'g_datalist_id_remove_no_notify';
procedure g_datalist_foreach(datalist:PPGData; func:TGDataForeachFunc; user_data:gpointer);cdecl;external glibdll name 'g_datalist_foreach';
procedure g_datalist_id_set_data(dl:PPGdata;q:TGQuark;d : pointer);
procedure g_datalist_id_remove_data(dl:PPGData;q:TGQuark);
function  g_datalist_Get_data(dl:PPGData;k : Pgchar):gpointer;
procedure g_datalist_set_data_full(dl:PPGData;k:Pgchar;d:gpointer;f : TGDestroyNotify);
procedure g_datalist_remove_no_notify(dl:PPGdata;k : Pgchar);
procedure g_datalist_set_data(dl:PPGdata;k:Pgchar;d : pointer);
procedure g_datalist_remove_data(dl:PPGdata;k : Pgchar);

procedure g_dataset_destroy(dataset_location:gconstpointer);cdecl;external glibdll name 'g_dataset_destroy';
function  g_dataset_id_get_data(dataset_location:gconstpointer; key_id:TGQuark):gpointer;cdecl;external glibdll name 'g_dataset_id_get_data';
procedure g_dataset_id_set_data_full(dataset_location:gconstpointer; key_id:TGQuark; data:gpointer; destroy_func:TGDestroyNotify);cdecl;external glibdll name 'g_dataset_id_set_data_full';
procedure g_dataset_id_remove_no_notify(dataset_location:gconstpointer; key_id:TGQuark);cdecl;external glibdll name 'g_dataset_id_remove_no_notify';
procedure g_dataset_foreach(dataset_location:gconstpointer; func:TGDataForeachFunc; user_data:gpointer);cdecl;external glibdll name 'g_dataset_foreach';
procedure g_dataset_id_set_data(l:gconstpointer;k:TGQuark;d : gconstpointer);
procedure g_dataset_id_remove_data(l : gconstpointer;k:TGQuark);
function  g_dataset_get_data(l : gconstpointer;k:Pgchar) : gconstpointer;
procedure g_dataset_set_data_full(l:gconstpointer;k:Pgchar;d:pointer;f : TGDestroynotify);
procedure g_dataset_remove_no_notify(l : gconstpointer;k:Pgchar);
procedure g_dataset_set_data(l:gconstpointer;k:Pgchar;d:pointer);
procedure g_dataset_remove_data(l : gconstpointer;k:Pgchar);

function  case_sensitive(var a : TGScannerConfig) : guint;
procedure set_case_sensitive(var a : TGScannerConfig; __case_sensitive : guint);
function  skip_comment_multi(var a : TGScannerConfig) : guint;
procedure set_skip_comment_multi(var a : TGScannerConfig; __skip_comment_multi : guint);
function  skip_comment_single(var a : TGScannerConfig) : guint;
procedure set_skip_comment_single(var a : TGScannerConfig; __skip_comment_single : guint);
function  scan_comment_multi(var a : TGScannerConfig) : guint;
procedure set_scan_comment_multi(var a : TGScannerConfig; __scan_comment_multi : guint);
function  scan_identifier(var a : TGScannerConfig) : guint;
procedure set_scan_identifier(var a : TGScannerConfig; __scan_identifier : guint);
function  scan_identifier_1char(var a : TGScannerConfig) : guint;
procedure set_scan_identifier_1char(var a : TGScannerConfig; __scan_identifier_1char : guint);
function  scan_identifier_NULL(var a : TGScannerConfig) : guint;
procedure set_scan_identifier_NULL(var a : TGScannerConfig; __scan_identifier_NULL : guint);
function  scan_symbols(var a : TGScannerConfig) : guint;
procedure set_scan_symbols(var a : TGScannerConfig; __scan_symbols : guint);
function  scan_binary(var a : TGScannerConfig) : guint;
procedure set_scan_binary(var a : TGScannerConfig; __scan_binary : guint);
function  scan_octal(var a : TGScannerConfig) : guint;
procedure set_scan_octal(var a : TGScannerConfig; __scan_octal : guint);
function  scan_float(var a : TGScannerConfig) : guint;
procedure set_scan_float(var a : TGScannerConfig; __scan_float : guint);
function  scan_hex(var a : TGScannerConfig) : guint;
procedure set_scan_hex(var a : TGScannerConfig; __scan_hex : guint);
function  scan_hex_dollar(var a : TGScannerConfig) : guint;
procedure set_scan_hex_dollar(var a : TGScannerConfig; __scan_hex_dollar : guint);
function  scan_string_sq(var a : TGScannerConfig) : guint;cdecl;
procedure set_scan_string_sq(var a : TGScannerConfig; __scan_string_sq : guint);cdecl;
function  scan_string_dq(var a : TGScannerConfig) : guint;cdecl;
procedure set_scan_string_dq(var a : TGScannerConfig; __scan_string_dq : guint);cdecl;
function  numbers_2_int(var a : TGScannerConfig) : guint;
procedure set_numbers_2_int(var a : TGScannerConfig; __numbers_2_int : guint);
function  int_2_float(var a : TGScannerConfig) : guint;
procedure set_int_2_float(var a : TGScannerConfig; __int_2_float : guint);
function  identifier_2_string(var a : TGScannerConfig) : guint;
procedure set_identifier_2_string(var a : TGScannerConfig; __identifier_2_string : guint);
function  char_2_token(var a : TGScannerConfig) : guint;
procedure set_char_2_token(var a : TGScannerConfig; __char_2_token : guint);
function  symbol_2_token(var a : TGScannerConfig) : guint;
procedure set_symbol_2_token(var a : TGScannerConfig; __symbol_2_token : guint);
function  scope_0_fallback(var a : TGScannerConfig) : guint;
procedure set_scope_0_fallback(var a : TGScannerConfig; __scope_0_fallback : guint);

function  g_scanner_new(config_templ:PGScannerConfig):PGScanner;cdecl;external glibdll name 'g_scanner_new';
procedure g_scanner_destroy(scanner:PGScanner);cdecl;external glibdll name 'g_scanner_destroy';
procedure g_scanner_input_file(scanner:PGScanner; input_fd:gint);cdecl;external glibdll name 'g_scanner_input_file';
procedure g_scanner_sync_file_offset(scanner:PGScanner);cdecl;external glibdll name 'g_scanner_sync_file_offset';
procedure g_scanner_input_text(scanner:PGScanner; text:Pgchar; text_len:guint);cdecl;external glibdll name 'g_scanner_input_text';
function  g_scanner_get_next_token(scanner:PGScanner):TGTokenType;cdecl;external glibdll name 'g_scanner_get_next_token';
function  g_scanner_peek_next_token(scanner:PGScanner):TGTokenType;cdecl;external glibdll name 'g_scanner_peek_next_token';
function  g_scanner_cur_token(scanner:PGScanner):TGTokenType;cdecl;external glibdll name 'g_scanner_cur_token';
function  g_scanner_cur_value(scanner:PGScanner):TGTokenValue;cdecl;external glibdll name 'g_scanner_cur_value';
function  g_scanner_cur_line(scanner:PGScanner):guint;cdecl;external glibdll name 'g_scanner_cur_line';
function  g_scanner_cur_position(scanner:PGScanner):guint;cdecl;external glibdll name 'g_scanner_cur_position';
function  g_scanner_eof(scanner:PGScanner):gboolean;cdecl;external glibdll name 'g_scanner_eof';
function  g_scanner_set_scope(scanner:PGScanner; scope_id:guint):guint;cdecl;external glibdll name 'g_scanner_set_scope';
procedure g_scanner_scope_add_symbol(scanner:PGScanner; scope_id:guint; symbol:Pgchar; value:gpointer);cdecl;external glibdll name 'g_scanner_scope_add_symbol';
procedure g_scanner_scope_remove_symbol(scanner:PGScanner; scope_id:guint; symbol:Pgchar);cdecl;external glibdll name 'g_scanner_scope_remove_symbol';
function  g_scanner_scope_lookup_symbol(scanner:PGScanner; scope_id:guint; symbol:Pgchar):gpointer;cdecl;external glibdll name 'g_scanner_scope_lookup_symbol';
procedure g_scanner_scope_foreach_symbol(scanner:PGScanner; scope_id:guint; func:TGHFunc; user_data:gpointer);cdecl;external glibdll name 'g_scanner_scope_foreach_symbol';
function  g_scanner_lookup_symbol(scanner:PGScanner; symbol:Pgchar):gpointer;cdecl;external glibdll name 'g_scanner_lookup_symbol';
procedure g_scanner_freeze_symbol_table(scanner:PGScanner);cdecl;external glibdll name 'g_scanner_freeze_symbol_table';
procedure g_scanner_thaw_symbol_table(scanner:PGScanner);cdecl;external glibdll name 'g_scanner_thaw_symbol_table';
procedure g_scanner_unexp_token(scanner:PGScanner; expected_token:TGTokenType; identifier_spec:Pgchar; symbol_spec:Pgchar; symbol_name:Pgchar; message:Pgchar; is_error:gint);cdecl;external glibdll name 'g_scanner_unexp_token';
function  g_scanner_stat_mode(filename:Pgchar):gint;cdecl;external glibdll name 'g_scanner_stat_mode';

type
     PGCompletion = ^TGCompletion;
     TGCompletion = record
          items : PGList;
          func : TGCompletionFunc;
          prefix : Pgchar;
          cache : PGList;
       end;

function  g_completion_new(func:TGCompletionFunc):PGCompletion;cdecl;external glibdll name 'g_completion_new';
procedure g_completion_add_items(cmp:PGCompletion; items:PGList);cdecl;external glibdll name 'g_completion_add_items';
procedure g_completion_remove_items(cmp:PGCompletion; items:PGList);cdecl;external glibdll name 'g_completion_remove_items';
procedure g_completion_clear_items(cmp:PGCompletion);cdecl;external glibdll name 'g_completion_clear_items';
function  g_completion_complete(cmp:PGCompletion; prefix:Pgchar; new_prefix:PPgchar):PGList;cdecl;external glibdll name 'g_completion_complete';
procedure g_completion_free(cmp:PGCompletion);cdecl;external glibdll name 'g_completion_free';

    type
       TGDateYear = guint16;
       TGDateDay = guint8;

       TGDateDMY = longint;
const
       G_DATEDMY_DAY = 0;
       G_DATEDMY_MONTH = 1;
       G_DATEDMY_YEAR = 2;

type
       TGDateWeekday = longint;
const
       G_DATE_BAD_WEEKDAY = 0;
       G_DATE_MONDAY = 1;
         G_DATE_TUESDAY = 2;
         G_DATE_WEDNESDAY = 3;
         G_DATE_THURSDAY = 4;
         G_DATE_FRIDAY = 5;
         G_DATE_SATURDAY = 6;
         G_DATE_SUNDAY = 7;

type
       TGDateMonth = longint;
const
         G_DATE_BAD_MONTH = 0;
         G_DATE_JANUARY = 1;
         G_DATE_FEBRUARY = 2;
         G_DATE_MARCH = 3;
         G_DATE_APRIL = 4;
         G_DATE_MAY = 5;
         G_DATE_JUNE = 6;
         G_DATE_JULY = 7;
         G_DATE_AUGUST = 8;
         G_DATE_SEPTEMBER = 9;
         G_DATE_OCTOBER = 10;
         G_DATE_NOVEMBER = 11;
         G_DATE_DECEMBER = 12;

    const
       G_DATE_BAD_JULIAN = 0;
       G_DATE_BAD_DAY = 0;
       G_DATE_BAD_YEAR = 0;

    type
       PGDate = ^TGDate;
       TGDate = record
            flag0 : longint;
            flag1 : longint;
         end;

    const
       bm_TGDate_julian_days = $FFFFFFFF;
       bp_TGDate_julian_days = 0;
       bm_TGDate_julian = $1;
       bp_TGDate_julian = 0;
       bm_TGDate_dmy = $2;
       bp_TGDate_dmy = 1;
       bm_TGDate_day = $FC;
       bp_TGDate_day = 2;
       bm_TGDate_month = $F00;
       bp_TGDate_month = 8;
       bm_TGDate_year = $FFFF000;
       bp_TGDate_year = 12;
function  julian_days(var a : TGDate) : guint;
procedure set_julian_days(var a : TGDate; __julian_days : guint);
function  julian(var a : TGDate) : guint;
procedure set_julian(var a : TGDate; __julian : guint);
function  dmy(var a : TGDate) : guint;
procedure set_dmy(var a : TGDate; __dmy : guint);
function  day(var a : TGDate) : guint;
procedure set_day(var a : TGDate; __day : guint);
function  month(var a : TGDate) : guint;
procedure set_month(var a : TGDate; __month : guint);
function  year(var a : TGDate) : guint;
procedure set_year(var a : TGDate; __year : guint);

function  g_date_new:PGDate;cdecl;external glibdll name 'g_date_new';
function  g_date_new_dmy(day:TGDateDay; month:TGDateMonth; year:TGDateYear):PGDate;cdecl;external glibdll name 'g_date_new_dmy';
function  g_date_new_julian(julian_day:guint32):PGDate;cdecl;external glibdll name 'g_date_new_julian';
procedure g_date_free(date:PGDate);cdecl;external glibdll name 'g_date_free';
function  g_date_valid(date:PGDate):gboolean;cdecl;external glibdll name 'g_date_valid';
function  g_date_valid_day(day:TGDateDay):gboolean;cdecl;external glibdll name 'g_date_valid_day';
function  g_date_valid_month(month:TGDateMonth):gboolean;cdecl;external glibdll name 'g_date_valid_month';
function  g_date_valid_year(year:TGDateYear):gboolean;cdecl;external glibdll name 'g_date_valid_year';
function  g_date_valid_weekday(weekday:TGDateWeekday):gboolean;cdecl;external glibdll name 'g_date_valid_weekday';
function  g_date_valid_julian(julian_date:guint32):gboolean;cdecl;external glibdll name 'g_date_valid_julian';
function  g_date_valid_dmy(day:TGDateDay; month:TGDateMonth; year:TGDateYear):gboolean;cdecl;external glibdll name 'g_date_valid_dmy';
function  g_date_weekday(date:PGDate):TGDateWeekday;cdecl;external glibdll name 'g_date_weekday';
function  g_date_month(date:PGDate):TGDateMonth;cdecl;external glibdll name 'g_date_month';
function  g_date_year(date:PGDate):TGDateYear;cdecl;external glibdll name 'g_date_year';
function  g_date_day(date:PGDate):TGDateDay;cdecl;external glibdll name 'g_date_day';
function  g_date_julian(date:PGDate):guint32;cdecl;external glibdll name 'g_date_julian';
function  g_date_day_of_year(date:PGDate):guint;cdecl;external glibdll name 'g_date_day_of_year';
function  g_date_monday_week_of_year(date:PGDate):guint;cdecl;external glibdll name 'g_date_monday_week_of_year';
function  g_date_sunday_week_of_year(date:PGDate):guint;cdecl;external glibdll name 'g_date_sunday_week_of_year';
procedure g_date_clear(date:PGDate; n_dates:guint);cdecl;external glibdll name 'g_date_clear';
procedure g_date_set_parse(date:PGDate; str:Pgchar);cdecl;external glibdll name 'g_date_set_parse';
procedure g_date_set_time(date:PGDate; time:TGTime);cdecl;external glibdll name 'g_date_set_time';
procedure g_date_set_month(date:PGDate; month:TGDateMonth);cdecl;external glibdll name 'g_date_set_month';
procedure g_date_set_day(date:PGDate; day:TGDateDay);cdecl;external glibdll name 'g_date_set_day';
procedure g_date_set_year(date:PGDate; year:TGDateYear);cdecl;external glibdll name 'g_date_set_year';
procedure g_date_set_dmy(date:PGDate; day:TGDateDay; month:TGDateMonth; y:TGDateYear);cdecl;external glibdll name 'g_date_set_dmy';
procedure g_date_set_julian(date:PGDate; julian_date:guint32);cdecl;external glibdll name 'g_date_set_julian';
function  g_date_is_first_of_month(date:PGDate):gboolean;cdecl;external glibdll name 'g_date_is_first_of_month';
function  g_date_is_last_of_month(date:PGDate):gboolean;cdecl;external glibdll name 'g_date_is_last_of_month';
procedure g_date_add_days(date:PGDate; n_days:guint);cdecl;external glibdll name 'g_date_add_days';
procedure g_date_subtract_days(date:PGDate; n_days:guint);cdecl;external glibdll name 'g_date_subtract_days';
procedure g_date_add_months(date:PGDate; n_months:guint);cdecl;external glibdll name 'g_date_add_months';
procedure g_date_subtract_months(date:PGDate; n_months:guint);cdecl;external glibdll name 'g_date_subtract_months';
procedure g_date_add_years(date:PGDate; n_years:guint);cdecl;external glibdll name 'g_date_add_years';
procedure g_date_subtract_years(date:PGDate; n_years:guint);cdecl;external glibdll name 'g_date_subtract_years';
function  g_date_is_leap_year(year:TGDateYear):gboolean;cdecl;external glibdll name 'g_date_is_leap_year';
function  g_date_days_in_month(month:TGDateMonth; year:TGDateYear):guint8;cdecl;external glibdll name 'g_date_days_in_month';
function  g_date_monday_weeks_in_year(year:TGDateYear):guint8;cdecl;external glibdll name 'g_date_monday_weeks_in_year';
function  g_date_sunday_weeks_in_year(year:TGDateYear):guint8;cdecl;external glibdll name 'g_date_sunday_weeks_in_year';
function  g_date_compare(lhs:PGDate; rhs:PGDate):gint;cdecl;external glibdll name 'g_date_compare';
procedure g_date_to_struct_tm(date:PGDate; tm:ptm);cdecl;external glibdll name 'g_date_to_struct_tm';
function  g_date_strftime(s:Pgchar; slen:gsize; format:Pgchar; date:PGDate):gsize;cdecl;external glibdll name 'g_date_strftime';
function  g_relation_new(fields:gint):PGRelation;cdecl;external glibdll name 'g_relation_new';
procedure g_relation_destroy(relation:PGRelation);cdecl;external glibdll name 'g_relation_destroy';
procedure g_relation_index(relation:PGRelation; field:gint; hash_func:TGHashFunc; key_compare_func:TGCompareFunc);cdecl;external glibdll name 'g_relation_index';
procedure g_relation_insert(relation:PGRelation; args:array of const);cdecl;external glibdll name 'g_relation_insert';
function  g_relation_delete(relation:PGRelation; key:gconstpointer; field:gint):gint;cdecl;external glibdll name 'g_relation_delete';
function  g_relation_select(relation:PGRelation; key:gconstpointer; field:gint):PGTuples;cdecl;external glibdll name 'g_relation_select';
function  g_relation_count(relation:PGRelation; key:gconstpointer; field:gint):gint;cdecl;external glibdll name 'g_relation_count';
function  g_relation_exists(relation:PGRelation; args:array of const):gboolean;cdecl;external glibdll name 'g_relation_exists';
procedure g_relation_print(relation:PGRelation);cdecl;external glibdll name 'g_relation_print';
procedure g_tuples_destroy(tuples:PGTuples);cdecl;external glibdll name 'g_tuples_destroy';
function  g_tuples_index(tuples:PGTuples; index:gint; field:gint):gpointer;cdecl;external glibdll name 'g_tuples_index';
function  g_spaced_primes_closest(num:guint):guint;cdecl;external glibdll name 'g_spaced_primes_closest';

    type
       TGIOError = longint;
    const
       G_IO_ERROR_NONE = 0;
       G_IO_ERROR_AGAIN = 1;
       G_IO_ERROR_INVAL = 2;
       G_IO_ERROR_UNKNOWN = 3;

    type
       TGSeekType = longint;
    const
       G_SEEK_CUR = 0;
       G_SEEK_SET = 1;
       G_SEEK_END = 2;

       G_IO_IN    = 1;
       G_IO_OUT   = 4;
       G_IO_PRI   = 2;
       G_IO_ERR   = 8;

       G_IO_HUP   = 16;
       G_IO_NVAL  = 32;

    type
       TGIOCondition = longint;

       PGIOFuncs = ^TGIOFuncs;

       PGIOChannel = ^TGIOChannel;
       TGIOChannel = record
            channel_flags : guint;
            ref_count : guint;
            funcs : PGIOFuncs;
         end;

       TGIOFunc = function (source:PGIOChannel; condition:TGIOCondition; data:gpointer):gboolean;cdecl;

       TGIOFuncs = record
            io_read : function (channel:PGIOChannel; buf:Pgchar; count:guint; bytes_read:Pguint):TGIOError;cdecl;
            io_write : function (channel:PGIOChannel; buf:Pgchar; count:guint; bytes_written:Pguint):TGIOError;cdecl;
            io_seek : function (channel:PGIOChannel; offset:gint; thetype:TGSeekType):TGIOError;cdecl;
            io_close : procedure (channel:PGIOChannel);cdecl;
            io_add_watch : function (channel:PGIOChannel; priority:gint; condition:TGIOCondition; func:TGIOFunc; user_data:gpointer; notify:TGDestroyNotify):guint;cdecl;
            io_free : procedure (channel:PGIOChannel);cdecl;
         end;

procedure g_io_channel_init(channel:PGIOChannel);cdecl;external glibdll name 'g_io_channel_init';
procedure g_io_channel_ref(channel:PGIOChannel);cdecl;external glibdll name 'g_io_channel_ref';
procedure g_io_channel_unref(channel:PGIOChannel);cdecl;external glibdll name 'g_io_channel_unref';
function  g_io_channel_read(channel:PGIOChannel; buf:Pgchar; count:guint; bytes_read:Pguint):TGIOError;cdecl;external glibdll name 'g_io_channel_read';
function  g_io_channel_write(channel:PGIOChannel; buf:Pgchar; count:guint; bytes_written:Pguint):TGIOError;cdecl;external glibdll name 'g_io_channel_write';
function  g_io_channel_seek(channel:PGIOChannel; offset:gint; thetype:TGSeekType):TGIOError;cdecl;external glibdll name 'g_io_channel_seek';
procedure g_io_channel_close(channel:PGIOChannel);cdecl;external glibdll name 'g_io_channel_close';
function  g_io_add_watch_full(channel:PGIOChannel; priority:gint; condition:TGIOCondition; func:TGIOFunc; user_data:gpointer; notify:TGDestroyNotify):guint;cdecl;external glibdll name 'g_io_add_watch_full';
function  g_io_add_watch(channel:PGIOChannel; condition:TGIOCondition; func:TGIOFunc; user_data:gpointer):guint;cdecl;external glibdll name 'g_io_add_watch';

    type
       PGTimeVal = ^TGTimeVal;
       TGTimeVal = record
            tv_sec : glong;
            tv_usec : glong;
         end;

       PGSourceFuncs = ^TGSourceFuncs;
       TGSourceFuncs = record
            prepare : function (source_data:gpointer; current_time:PGTimeVal; timeout:Pgint; user_data:gpointer):gboolean;cdecl;
            check : function (source_data:gpointer; current_time:PGTimeVal; user_data:gpointer):gboolean;cdecl;
            dispatch : function (source_data:gpointer; current_time:PGTimeVal; user_data:gpointer):gboolean;cdecl;
            destroy : TGDestroyNotify;
         end;

    const
       G_PRIORITY_HIGH = -100;
       G_PRIORITY_DEFAULT = 0;
       G_PRIORITY_HIGH_IDLE = 100;
       G_PRIORITY_DEFAULT_IDLE = 200;
       G_PRIORITY_LOW = 300;

    type
       TGSourceFunc = function (data:gpointer):gboolean;cdecl;

function  g_source_add(priority:gint; can_recurse:gboolean; funcs:PGSourceFuncs; source_data:gpointer; user_data:gpointer; notify:TGDestroyNotify):guint;cdecl;external glibdll name 'g_source_add';
function  g_source_remove(tag:guint):gboolean;cdecl;external glibdll name 'g_source_remove';
function  g_source_remove_by_user_data(user_data:gpointer):gboolean;cdecl;external glibdll name 'g_source_remove_by_user_data';
function  g_source_remove_by_source_data(source_data:gpointer):gboolean;cdecl;external glibdll name 'g_source_remove_by_source_data';
function  g_source_remove_by_funcs_user_data(funcs:PGSourceFuncs; user_data:gpointer):gboolean;cdecl;external glibdll name 'g_source_remove_by_funcs_user_data';
procedure g_get_current_time(result:PGTimeVal);cdecl;external glibdll name 'g_get_current_time';
function  g_main_new(is_running:gboolean):PGMainLoop;cdecl;external glibdll name 'g_main_new';
procedure g_main_run(loop:PGMainLoop);cdecl;external glibdll name 'g_main_run';
procedure g_main_quit(loop:PGMainLoop);cdecl;external glibdll name 'g_main_quit';
procedure g_main_destroy(loop:PGMainLoop);cdecl;external glibdll name 'g_main_destroy';
function  g_main_is_running(loop:PGMainLoop):gboolean;cdecl;external glibdll name 'g_main_is_running';
function  g_main_iteration(may_block:gboolean):gboolean;cdecl;external glibdll name 'g_main_iteration';
function  g_main_pending:gboolean;cdecl;external glibdll name 'g_main_pending';
function  g_timeout_add_full(priority:gint; interval:guint; thefunction:TGSourceFunc; data:gpointer; notify:TGDestroyNotify):guint;cdecl;external glibdll name 'g_timeout_add_full';
function  g_timeout_add(interval:guint; thefunction:TGSourceFunc; data:gpointer):guint;cdecl;external glibdll name 'g_timeout_add';
function  g_idle_add(thefunction:TGSourceFunc; data:gpointer):guint;cdecl;external glibdll name 'g_idle_add';
function  g_idle_add_full(priority:gint; thefunction:TGSourceFunc; data:gpointer; destroy:TGDestroyNotify):guint;cdecl;external glibdll name 'g_idle_add_full';
function  g_idle_remove_by_data(data:gpointer):gboolean;cdecl;external glibdll name 'g_idle_remove_by_data';

    type
       PGPollFD = ^TGPollFD;
       TGPollFD = record
            fd : gint;
            events : gushort;
            revents : gushort;
         end;

       TGPollFunc = function (ufds:PGPollFD; nfsd:guint; timeout:gint):gint;cdecl;

procedure g_main_add_poll(fd:PGPollFD; priority:gint);cdecl;external glibdll name 'g_main_add_poll';
procedure g_main_remove_poll(fd:PGPollFD);cdecl;external glibdll name 'g_main_remove_poll';
procedure g_main_set_poll_func(func:TGPollFunc);cdecl;external glibdll name 'g_main_set_poll_func';
function  g_io_channel_unix_new(fd:longint):PGIOChannel;cdecl;external glibdll name 'g_io_channel_unix_new';
function  g_io_channel_unix_get_fd(channel:PGIOChannel):gint;cdecl;external glibdll name 'g_io_channel_unix_get_fd';


implementation

function  g_list_previous(list : PGlist) : PGlist;
begin
  if list<>nil then
    g_list_previous:=(PGList(list))^.prev
  else
    g_list_previous:=NULL;
end;

function  g_list_next(list : PGlist) : PGlist;
begin
  if list<>nil then
    g_list_next:=(PGList(list))^.next
  else
    g_list_next:=NULL;
end;

function  g_slist_next(slist : PGSList) : PGSList;
begin
  if slist<>nil then
    g_slist_next:=(PGSList(slist))^.next
  else
    g_slist_next:=NULL;
end;

function  G_NODE_IS_ROOT(node : PGNode) : boolean;
begin
  G_NODE_IS_ROOT:={((((PGNode(node))^.parent) = (NULL and (@((PGNode(node))^.prev)))) = (NULL and (@((PGNode(node))^.next)))) = NULL; }false;
end;

function  G_NODE_IS_LEAF(node : PGNode) : boolean;
begin
  G_NODE_IS_LEAF:=((PGNode(node))^.children) = NULL;
end;

function  g_node_append(parent,node : PGNode) : PGNode;
begin
  g_node_append:=g_node_insert_before(parent,NULL,node);
end;

function  g_node_insert_data(parent: PGNode;position:gint;data:pointer) : PGNode;
begin
  g_node_insert_data:=g_node_insert(parent,position,g_node_new(data));
end;

function  g_node_insert_data_before(parent,sibling : PGNode;data:pointer) : PGNode;
begin
  g_node_insert_data_before:=g_node_insert_before(parent,sibling,g_node_new(data));
end;

function  g_node_prepend_data(parent:PGNode;data : pointer) : PGNode;
begin
  g_node_prepend_data:=g_node_prepend(parent,g_node_new(data));
end;

function  g_node_append_data(parent : PGNode;data:pointer) : PGNode;
begin
  g_node_append_data:=g_node_insert_before(parent,NULL,g_node_new(data));
end;

function  g_node_prev_sibling(node : PGnode) : PGNode;
begin
  if node<>nil then
    g_node_prev_sibling:=(PGNode(node))^.prev
  else
    g_node_prev_sibling:=NULL;
end;

function  g_node_next_sibling(node : PGNode) : PGNode;
begin
  if node<>nil then
    g_node_next_sibling:=(PGNode(node))^.next
  else
    g_node_next_sibling:=NULL;
end;

function  g_node_first_child(node : PGNode) : PGNode;
begin
  if node<>nil then
    g_node_first_child:=(PGNode(node))^.children
  else
    g_node_first_child:=NULL;
end;

function G_HOOK_DEFERRED_DESTROY : TGHookFreeFunc;
begin
  G_HOOK_DEFERRED_DESTROY:=TGHookFreeFunc($01);
end;

function  is_setup(var a : TGHookList) : guint;
      begin
        is_setup:=(a.flag0 and bm_TGHookList_is_setup) shr bp_TGHookList_is_setup;
      end;

procedure set_is_setup(var a : TGHookList; __is_setup : guint);
      begin
         a.flag0:=a.flag0 or ((__is_setup shl bp_TGHookList_is_setup) and bm_TGHookList_is_setup);
      end;

function  G_HOOK_ACTIVE(hook : PGHook) : boolean;
begin
  G_HOOK_ACTIVE:=(((PGHook(hook))^.flags) and cardinal(G_HOOK_FLAG_ACTIVE)) <> 0;
end;

function  G_HOOK_IN_CALL(hook : PGHook) : boolean;
begin
  G_HOOK_IN_CALL:=(((PGHook(hook))^.flags) and cardinal(G_HOOK_FLAG_IN_CALL)) <> 0;
end;

function  G_HOOK_IS_VALID(hook : PGHook) : boolean;
begin
  G_HOOK_IS_VALID:=(hook^.hook_id<>0) and G_HOOK_ACTIVE(hook);
end;

function  G_HOOK_IS_UNLINKED(hook : PGHook) : boolean;
begin
  G_HOOK_IS_UNLINKED:=(hook^.next=NULL) and (hook^.prev=NULL) and (hook^.hook_id=0) and (hook^.ref_count = 0);
end;

procedure g_hook_append(hook_list:PGhooklist;hook : PGHook);
begin
  g_hook_insert_before(hook_list,nil,hook);
end;

function  g_strstrip(thestring : Pgchar) : Pgchar;
begin
  g_strstrip:=g_strchomp(g_strchug(thestring));
end;

function  g_array_append_val(a:PGArray;v : longint) : PGarray;
begin
  g_array_append_val:=g_array_append_vals(a,@(v),1);
end;

function  g_array_prepend_val(a:PGArray;v : longint) : PGarray;
begin
  g_array_prepend_val:=g_array_prepend_vals(a,@(v),1);
end;

function  g_array_insert_val(a:PGArray;i:guint;v : longint) : PGarray;
begin
  g_array_insert_val:=g_array_insert_vals(a,i,@(v),1);
end;

procedure g_datalist_id_set_data(dl:PPGdata;q:TGQuark;d : pointer);
begin
  g_datalist_id_set_data_full(dl,q,d,NULL);
end;

procedure g_datalist_id_remove_data(dl:PPGData;q:TGQuark);
begin
  g_datalist_id_set_data(dl,q,NULL);
end;

function  g_datalist_Get_data(dl:PPGData;k : Pgchar):gpointer;
begin
  g_datalist_Get_data:=g_datalist_id_get_data(dl,g_quark_try_string(k));
end;

procedure g_datalist_set_data_full(dl:PPGData;k:Pgchar;d:gpointer;f : TGdestroynotify);
begin
  g_datalist_id_set_data_full(dl,g_quark_from_string(k),d,f);
end;

procedure g_datalist_remove_no_notify(dl:PPGdata;k : Pgchar);
begin
  g_datalist_id_remove_no_notify(dl,g_quark_try_string(k));
end;

procedure g_datalist_set_data(dl:PPGdata;k:Pgchar;d : pointer);
begin
  g_datalist_set_data_full(dl,k,d,NULL);
end;

procedure g_datalist_remove_data(dl:PPGdata;k : Pgchar);
begin
  g_datalist_id_set_data(dl,g_quark_try_string(k),NULL);
end;

procedure g_dataset_id_set_data(l:gconstpointer;k:TGQuark;d : gconstpointer);
begin
  g_dataset_id_set_data_full(l,k,d,NULL);
end;

procedure g_dataset_id_remove_data(l : gconstpointer;k:TGQuark);
begin
  g_dataset_id_set_data(l,k,NULL);
end;

function  g_dataset_get_data(l : gconstpointer;k:Pgchar) : gconstpointer;
begin
  g_dataset_get_data:=g_dataset_id_get_data(l,g_quark_try_string(k));
end;

procedure g_dataset_set_data_full(l:gconstpointer;k:Pgchar;d:pointer;f : TGDestroynotify);
begin
  g_dataset_id_set_data_full(l,g_quark_from_string(k),d,f);
end;

procedure g_dataset_remove_no_notify(l : gconstpointer;k:Pgchar);
begin
  g_dataset_id_remove_no_notify(l,g_quark_try_string(k));
end;

procedure g_dataset_set_data(l:gconstpointer;k:Pgchar;d:pointer);
begin
  g_dataset_set_data_full(l,k,d,NULL);
end;

procedure g_dataset_remove_data(l : gconstpointer;k:Pgchar);
begin
  g_dataset_id_set_data(l,g_quark_try_string(k),NULL);
end;

function  case_sensitive(var a : TGScannerConfig) : guint;
      begin
         case_sensitive:=(a.flag0 and bm_TGScannerConfig_case_sensitive) shr bp_TGScannerConfig_case_sensitive;
      end;

procedure set_case_sensitive(var a : TGScannerConfig; __case_sensitive : guint);
      begin
         a.flag0:=a.flag0 or ((__case_sensitive shl bp_TGScannerConfig_case_sensitive) and bm_TGScannerConfig_case_sensitive);
      end;

function  skip_comment_multi(var a : TGScannerConfig) : guint;
      begin
         skip_comment_multi:=(a.flag0 and bm_TGScannerConfig_skip_comment_multi) shr bp_TGScannerConfig_skip_comment_multi;
      end;

procedure set_skip_comment_multi(var a : TGScannerConfig; __skip_comment_multi : guint);
      begin
         a.flag0:=a.flag0 or ((__skip_comment_multi shl bp_TGScannerConfig_skip_comment_multi) and bm_TGScannerConfig_skip_comment_multi);
      end;

function  skip_comment_single(var a : TGScannerConfig) : guint;
      begin
         skip_comment_single:=(a.flag0 and bm_TGScannerConfig_skip_comment_single) shr bp_TGScannerConfig_skip_comment_single;
      end;

procedure set_skip_comment_single(var a : TGScannerConfig; __skip_comment_single : guint);
      begin
         a.flag0:=a.flag0 or ((__skip_comment_single shl bp_TGScannerConfig_skip_comment_single) and bm_TGScannerConfig_skip_comment_single);
      end;

function  scan_comment_multi(var a : TGScannerConfig) : guint;
      begin
         scan_comment_multi:=(a.flag0 and bm_TGScannerConfig_scan_comment_multi) shr bp_TGScannerConfig_scan_comment_multi;
      end;

procedure set_scan_comment_multi(var a : TGScannerConfig; __scan_comment_multi : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_comment_multi shl bp_TGScannerConfig_scan_comment_multi) and bm_TGScannerConfig_scan_comment_multi);
      end;

function  scan_identifier(var a : TGScannerConfig) : guint;
      begin
         scan_identifier:=(a.flag0 and bm_TGScannerConfig_scan_identifier) shr bp_TGScannerConfig_scan_identifier;
      end;

procedure set_scan_identifier(var a : TGScannerConfig; __scan_identifier : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_identifier shl bp_TGScannerConfig_scan_identifier) and bm_TGScannerConfig_scan_identifier);
      end;

function  scan_identifier_1char(var a : TGScannerConfig) : guint;
      begin
         scan_identifier_1char:=(a.flag0 and bm_TGScannerConfig_scan_identifier_1char) shr bp_TGScannerConfig_scan_identifier_1char;
      end;

procedure set_scan_identifier_1char(var a : TGScannerConfig; __scan_identifier_1char : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_identifier_1char shl bp_TGScannerConfig_scan_identifier_1char) and bm_TGScannerConfig_scan_identifier_1char);
      end;

function  scan_identifier_NULL(var a : TGScannerConfig) : guint;
      begin
         scan_identifier_NULL:=(a.flag0 and bm_TGScannerConfig_scan_identifier_NULL) shr bp_TGScannerConfig_scan_identifier_NULL;
      end;

procedure set_scan_identifier_NULL(var a : TGScannerConfig; __scan_identifier_NULL : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_identifier_NULL shl bp_TGScannerConfig_scan_identifier_NULL) and bm_TGScannerConfig_scan_identifier_NULL);
      end;

function  scan_symbols(var a : TGScannerConfig) : guint;
      begin
         scan_symbols:=(a.flag0 and bm_TGScannerConfig_scan_symbols) shr bp_TGScannerConfig_scan_symbols;
      end;

procedure set_scan_symbols(var a : TGScannerConfig; __scan_symbols : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_symbols shl bp_TGScannerConfig_scan_symbols) and bm_TGScannerConfig_scan_symbols);
      end;

function  scan_binary(var a : TGScannerConfig) : guint;
      begin
         scan_binary:=(a.flag0 and bm_TGScannerConfig_scan_binary) shr bp_TGScannerConfig_scan_binary;
      end;

procedure set_scan_binary(var a : TGScannerConfig; __scan_binary : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_binary shl bp_TGScannerConfig_scan_binary) and bm_TGScannerConfig_scan_binary);
      end;

function  scan_octal(var a : TGScannerConfig) : guint;
      begin
         scan_octal:=(a.flag0 and bm_TGScannerConfig_scan_octal) shr bp_TGScannerConfig_scan_octal;
      end;

procedure set_scan_octal(var a : TGScannerConfig; __scan_octal : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_octal shl bp_TGScannerConfig_scan_octal) and bm_TGScannerConfig_scan_octal);
      end;

function  scan_float(var a : TGScannerConfig) : guint;
      begin
         scan_float:=(a.flag0 and bm_TGScannerConfig_scan_float) shr bp_TGScannerConfig_scan_float;
      end;

procedure set_scan_float(var a : TGScannerConfig; __scan_float : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_float shl bp_TGScannerConfig_scan_float) and bm_TGScannerConfig_scan_float);
      end;

function  scan_hex(var a : TGScannerConfig) : guint;
      begin
         scan_hex:=(a.flag0 and bm_TGScannerConfig_scan_hex) shr bp_TGScannerConfig_scan_hex;
      end;

procedure set_scan_hex(var a : TGScannerConfig; __scan_hex : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_hex shl bp_TGScannerConfig_scan_hex) and bm_TGScannerConfig_scan_hex);
      end;

function  scan_hex_dollar(var a : TGScannerConfig) : guint;
      begin
         scan_hex_dollar:=(a.flag0 and bm_TGScannerConfig_scan_hex_dollar) shr bp_TGScannerConfig_scan_hex_dollar;
      end;

procedure set_scan_hex_dollar(var a : TGScannerConfig; __scan_hex_dollar : guint);
      begin
         a.flag0:=a.flag0 or ((__scan_hex_dollar shl bp_TGScannerConfig_scan_hex_dollar) and bm_TGScannerConfig_scan_hex_dollar);
      end;

function  scan_string_sq(var a : TGScannerConfig) : guint;cdecl;
      begin
         scan_string_sq:=(a.flag0 and bm_TGScannerConfig_scan_string_sq) shr bp_TGScannerConfig_scan_string_sq;
      end;

procedure set_scan_string_sq(var a : TGScannerConfig; __scan_string_sq : guint);cdecl;
      begin
         a.flag0:=a.flag0 or ((__scan_string_sq shl bp_TGScannerConfig_scan_string_sq) and bm_TGScannerConfig_scan_string_sq);
      end;

function  scan_string_dq(var a : TGScannerConfig) : guint;cdecl;
      begin
         scan_string_dq:=(a.flag0 and bm_TGScannerConfig_scan_string_dq) shr bp_TGScannerConfig_scan_string_dq;
      end;

procedure set_scan_string_dq(var a : TGScannerConfig; __scan_string_dq : guint);cdecl;
      begin
         a.flag0:=a.flag0 or ((__scan_string_dq shl bp_TGScannerConfig_scan_string_dq) and bm_TGScannerConfig_scan_string_dq);
      end;

function  numbers_2_int(var a : TGScannerConfig) : guint;
      begin
         numbers_2_int:=(a.flag0 and bm_TGScannerConfig_numbers_2_int) shr bp_TGScannerConfig_numbers_2_int;
      end;

procedure set_numbers_2_int(var a : TGScannerConfig; __numbers_2_int : guint);
      begin
         a.flag0:=a.flag0 or ((__numbers_2_int shl bp_TGScannerConfig_numbers_2_int) and bm_TGScannerConfig_numbers_2_int);
      end;

function  int_2_float(var a : TGScannerConfig) : guint;
      begin
         int_2_float:=(a.flag0 and bm_TGScannerConfig_int_2_float) shr bp_TGScannerConfig_int_2_float;
      end;

procedure set_int_2_float(var a : TGScannerConfig; __int_2_float : guint);
      begin
         a.flag0:=a.flag0 or ((__int_2_float shl bp_TGScannerConfig_int_2_float) and bm_TGScannerConfig_int_2_float);
      end;

function  identifier_2_string(var a : TGScannerConfig) : guint;
      begin
         identifier_2_string:=(a.flag0 and bm_TGScannerConfig_identifier_2_string) shr bp_TGScannerConfig_identifier_2_string;
      end;

procedure set_identifier_2_string(var a : TGScannerConfig; __identifier_2_string : guint);
      begin
         a.flag0:=a.flag0 or ((__identifier_2_string shl bp_TGScannerConfig_identifier_2_string) and bm_TGScannerConfig_identifier_2_string);
      end;

function  char_2_token(var a : TGScannerConfig) : guint;
      begin
         char_2_token:=(a.flag0 and bm_TGScannerConfig_char_2_token) shr bp_TGScannerConfig_char_2_token;
      end;

procedure set_char_2_token(var a : TGScannerConfig; __char_2_token : guint);
      begin
         a.flag0:=a.flag0 or ((__char_2_token shl bp_TGScannerConfig_char_2_token) and bm_TGScannerConfig_char_2_token);
      end;

function  symbol_2_token(var a : TGScannerConfig) : guint;
      begin
         symbol_2_token:=(a.flag0 and bm_TGScannerConfig_symbol_2_token) shr bp_TGScannerConfig_symbol_2_token;
      end;

procedure set_symbol_2_token(var a : TGScannerConfig; __symbol_2_token : guint);
      begin
         a.flag0:=a.flag0 or ((__symbol_2_token shl bp_TGScannerConfig_symbol_2_token) and bm_TGScannerConfig_symbol_2_token);
      end;

function  scope_0_fallback(var a : TGScannerConfig) : guint;
      begin
         scope_0_fallback:=(a.flag0 and bm_TGScannerConfig_scope_0_fallback) shr bp_TGScannerConfig_scope_0_fallback;
      end;

procedure set_scope_0_fallback(var a : TGScannerConfig; __scope_0_fallback : guint);
      begin
         a.flag0:=a.flag0 or ((__scope_0_fallback shl bp_TGScannerConfig_scope_0_fallback) and bm_TGScannerConfig_scope_0_fallback);
      end;

function  julian_days(var a : TGDate) : guint;
     begin
        julian_days:=(a.flag0 and bm_TGDate_julian_days) shr bp_TGDate_julian_days;
     end;

procedure set_julian_days(var a : TGDate; __julian_days : guint);
      begin
         a.flag0:=a.flag0 or ((__julian_days shl bp_TGDate_julian_days) and bm_TGDate_julian_days);
      end;

function  julian(var a : TGDate) : guint;
      begin
         julian:=(a.flag0 and bm_TGDate_julian) shr bp_TGDate_julian;
      end;

procedure set_julian(var a : TGDate; __julian : guint);
      begin
         a.flag0:=a.flag0 or ((__julian shl bp_TGDate_julian) and bm_TGDate_julian);
      end;

function  dmy(var a : TGDate) : guint;
      begin
         dmy:=(a.flag0 and bm_TGDate_dmy) shr bp_TGDate_dmy;
      end;

procedure set_dmy(var a : TGDate; __dmy : guint);
      begin
         a.flag0:=a.flag0 or ((__dmy shl bp_TGDate_dmy) and bm_TGDate_dmy);
      end;

function  day(var a : TGDate) : guint;
      begin
         day:=(a.flag0 and bm_TGDate_day) shr bp_TGDate_day;
      end;

procedure set_day(var a : TGDate; __day : guint);
      begin
         a.flag0:=a.flag0 or ((__day shl bp_TGDate_day) and bm_TGDate_day);
      end;

function  month(var a : TGDate) : guint;
      begin
         month:=(a.flag0 and bm_TGDate_month) shr bp_TGDate_month;
      end;

procedure set_month(var a : TGDate; __month : guint);
      begin
         a.flag0:=a.flag0 or ((__month shl bp_TGDate_month) and bm_TGDate_month);
      end;

function  year(var a : TGDate) : guint;
      begin
         year:=(a.flag0 and bm_TGDate_year) shr bp_TGDate_year;
      end;

procedure set_year(var a : TGDate; __year : guint);
      begin
         a.flag0:=a.flag0 or ((__year shl bp_TGDate_year) and bm_TGDate_year);
      end;

end.
