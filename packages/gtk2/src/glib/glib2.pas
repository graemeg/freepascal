{ GLIB - Library of useful routines for C programming

   Copyright (C) 1995-1997  Peter Mattis, Spencer Kimball and Josh MacDonald

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
{
   Modified by the GLib Team and others 1997-2000.  See the AUTHORS
   file for a list of people on the GLib Team.  See the ChangeLog
   files for a list of changes.  These files are distributed with
   GLib at ftp://ftp.gtk.org/pub/gtk/.
  }
unit glib2; // keep unit name lowercase for kylix

{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses
  ctypes,SysUtils;

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
  gliblib    = 'libglib-2.0-0.dll';
  gthreadlib = 'libgthread-2.0-0.dll';
  gmodulelib = 'libgmodule-2.0-0.dll';
  gobjectlib = 'libgobject-2.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
{$else}
  {$ifdef UseCustomLibs}
  gliblib    = '';
  gthreadlib = '';
  gmodulelib = '';
  gobjectlib = '';
  {$else}
  gliblib    = 'libglib-2.0.so';
  gthreadlib = 'libgthread-2.0.so';
  gmodulelib = 'libgmodule-2.0.so';
  gobjectlib = 'libgobject-2.0.so';
  {$endif}
{$endif}

{$ifdef FREEBSD}
  {$linklib c}
  {$linklib pthread}
{$endif}

{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}

// {$define HasGLIB2_10}

{$DEFINE read_forward_definitions}
type
  // internal types
  PGTypePlugin = pointer;
  PGParamSpecPool = pointer;

{$include gincludes.inc}
{$UNDEF read_forward_definitions}

{$DEFINE read_interface_types}
{$include gincludes.inc}
{$UNDEF read_interface_types}

{$DEFINE read_interface_rest}
{$include gincludes.inc}
{$UNDEF read_interface_rest}

implementation

{$DEFINE read_implementation}
{$include gincludes.inc}
{$UNDEF read_implementation}


{$IFNDEF KYLIX}
{ There is a bug in the compiler. If an external variable is not used, it will
  create code, that can't be relocated by the linker.
  So, use them in this hidden dummy procedure.
}
procedure CheckUnusedVariable; [Public];
begin
  if glib_mem_profiler_table=nil then ;
  if (glib_interface_age=0) or (glib_binary_age=0)
  or (g_thread_use_default_impl) then ;
end;
{$ENDIF}


{************************************
 * macro functions
 *
 ************************************}

{*
 * gtypes.inc
 *}
function GUINT16_SWAP_LE_BE_CONSTANT(val: guint16): guint16;
begin
  Result:=((val and $ff) shl 8) or ((val and $ff00) shr 8);
end;

function GUINT32_SWAP_LE_BE_CONSTANT(val: guint32): guint32;
begin
  Result:=
    ((val and $000000ff) shl 24) or
    ((val and $0000ff00) shl  8) or
    ((val and $00ff0000) shr  8) or
    ((val and $ff000000) shr 24);
end;

{*
 * glibconfig.inc
 *}
function GUINT_TO_POINTER(i: guint): pointer;
begin
  Result:=Pointer(PtrInt(i));
end;

{*
 *  garray.inc
 *}

function g_array_append_val(a: PGArray; v : gpointer) : PGArray;
begin
   g_array_append_val := g_array_append_vals(a,@(v),1);
end;

function g_array_prepend_val(a: PGArray; v : gpointer) : PGArray;
begin
   g_array_prepend_val := g_array_prepend_vals(a,@(v),1);
end;

function g_array_insert_val(a: PGArray; i: guint; v : gpointer) : PGArray;
begin
   g_array_insert_val := g_array_insert_vals(a,i,@(v),1);
end;

function g_ptr_array_index (parray: PGPtrArray; index: guint): gpointer;
begin
  {$IFDEF FPC}
  g_ptr_array_index := parray^.pdata[index];
  {$ELSE}
  g_ptr_array_index := PGPointer(integer(parray^.pdata) + index*SizeOf(GPointer))^;
  {$ENDIF}
end;

{*
 *  gthread.inc
 *}

function G_THREAD_ERROR: TGQuark;
begin
  G_THREAD_ERROR:=g_thread_error_quark;
end;

procedure g_mutex_lock     (mutex: PGMutex);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.mutex_lock (mutex);
{$ENDIF}
end;

function  g_mutex_trylock   (mutex: PGMutex):gboolean;
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_mutex_trylock := g_thread_functions_for_glib_use.mutex_trylock (mutex)
  else
    g_mutex_trylock := true;
{$ENDIF}
end;

procedure g_mutex_unlock   (mutex: PGMutex);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.mutex_unlock (mutex);
{$ENDIF}
end;

procedure g_mutex_free     (mutex: PGMutex);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.mutex_free (mutex);
{$ENDIF}
end;

procedure g_cond_wait      (cond: PGCond; mutex: PGMutex);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.cond_wait (cond, mutex);
{$ENDIF}
end;

function g_cond_timed_wait (cond     : PGCond;
                          mutex    : PGMutex;
                          end_time : PGTimeVal):gboolean;
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_cond_timed_wait := g_thread_functions_for_glib_use.cond_timed_wait (cond,
                                                                          mutex,
                                                                          end_time)
  else
    g_cond_timed_wait := true;
{$ENDIF}
end;


function  g_thread_supported: gboolean;
begin
{$IFNDEF KYLIX}
  g_thread_supported := g_threads_got_initialized;
{$ENDIF}
end;

function  g_mutex_new : PGMutex;
begin
{$IFNDEF KYLIX}
  g_mutex_new := g_thread_functions_for_glib_use.mutex_new;
{$ENDIF}
end;

function  g_cond_new  : PGCond;
begin
{$IFNDEF KYLIX}
  g_cond_new := g_thread_functions_for_glib_use.cond_new;
{$ENDIF}
end;

procedure g_cond_signal    (cond: PGCond);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.cond_signal (cond);
{$ENDIF}
end;

procedure g_cond_broadcast (cond: PGCond);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.cond_broadcast (cond);
{$ENDIF}
end;

procedure g_cond_free      (cond: PGCond);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.cond_free (cond);
{$ENDIF}
end;

function  g_private_new    (dest: TGDestroyNotify): PGPrivate;
begin
{$IFNDEF KYLIX}
    g_private_new := g_thread_functions_for_glib_use.private_new (dest);
{$ENDIF}
end;

function  g_private_get    (private_key: PGPrivate): gpointer;
begin
{$IFNDEF KYLIX}
     // convert result to gpointer
  if g_threads_got_initialized then
    g_private_get := g_thread_functions_for_glib_use.private_get (private_key)
  else
    g_private_get := private_key;
{$ENDIF}
end;


procedure g_private_set    (var private_key: PGPrivate; data: gpointer);
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
//    g_private_set := g_thread_functions_for_glib_use.private_set (private_key, data)
  else
    private_key := data;  // data casted to GPrivate
{$ENDIF}
end;


procedure g_thread_yield;
begin
{$IFNDEF KYLIX}
  if g_threads_got_initialized then
    g_thread_functions_for_glib_use.thread_yield;
{$ENDIF}
end;

function g_thread_create (func: TGThreadFunc;
                             data: gpointer;
                             joinable: gboolean;
                             error: PPGError): PGThread;
 begin
   g_thread_create := g_thread_create_full (func, data, 0, joinable, false,
                                            G_THREAD_PRIORITY_NORMAL, error);
 end;

function g_static_mutex_get_mutex(mutex: PPGMutex):PGMutex;
begin
  g_static_mutex_get_mutex := g_static_mutex_get_mutex_impl (mutex);
end;

procedure g_static_mutex_lock   (mutex: PGStaticMutex);
begin
  g_mutex_lock (g_static_mutex_get_mutex_impl (PPGMutex(mutex)));
end;

function  g_static_mutex_trylock (mutex: PGStaticMutex): gboolean;
begin
  g_static_mutex_trylock := g_mutex_trylock (g_static_mutex_get_mutex (PPGMutex(mutex)));
end;

procedure g_static_mutex_unlock (mutex: PGStaticMutex);
begin
  g_mutex_unlock (g_static_mutex_get_mutex_impl (PPGMutex(mutex)));
end;

{*
 *  gmain.inc
 *}

function g_main_new(is_running: gboolean): PGMainLoop;
  begin
    g_main_new := g_main_loop_new (nil, is_running);
  end;

function  g_main_iteration  (may_block: gboolean): gboolean;
  begin
    g_main_iteration := g_main_context_iteration (nil, may_block);
  end;

function  g_main_pending: gboolean;
  begin
    g_main_pending := g_main_context_pending (nil);
  end;

procedure g_main_set_poll_func(func:  TGPollFunc);
  begin
    g_main_context_set_poll_func (nil, func);
  end;

{*
 * gslist.inc
 *}

function g_slist_next(slist : PGSList) : PGSList;
  begin
    if slist <> nil then
      g_slist_next := slist^.next
    else
      g_slist_next := nil;
  end;

{*
 *  gmem.inc
 *}

function g_new(bytes_per_struct, n_structs: gsize): gpointer;
begin
   g_new:=g_malloc(n_structs*bytes_per_struct);
end;

function g_new0(bytes_per_struct, n_structs: gsize): gpointer;
begin
   g_new0:=g_malloc0(n_structs*bytes_per_struct);
end;

function g_renew(struct_size: gsize; OldMem: gpointer; n_structs : gsize) : gpointer;
begin
   g_renew:=g_realloc(OldMem,struct_size*n_structs);
end;

function g_chunk_new(chunk : Pointer) : Pointer;
begin
   g_chunk_new:=g_mem_chunk_alloc(chunk);
end;

function g_chunk_new0(chunk : Pointer) : Pointer;
begin
   g_chunk_new0:=g_mem_chunk_alloc0(chunk);
end;

procedure g_chunk_free(mem_chunk:PGMemChunk; mem:gpointer);
begin
   g_mem_chunk_free(mem_chunk,mem);
end;


{*
 *  glist.inc
 *}
function g_list_previous (list : PGList) : PGList;
  begin
     if list <> nil then
       g_list_previous:=(PGList(list))^.prev
     else
       g_list_previous:=nil;
  end;

function g_list_next  (list : PGList)    : PGList;
  begin
     if list <> nil then
       g_list_next:=(PGList(list))^.next
     else
       g_list_next:=NULL;
  end;

{*
 *  gconvert,inc
 *}
function G_CONVERT_ERROR : TGQuark;
    begin
       G_CONVERT_ERROR:=g_convert_error_quark;
    end;

{*
 *   gdataset.inc
 *}

procedure g_datalist_id_set_data (datalist: PPGData; key_id:TGQuark; data: gpointer);
  begin
    g_datalist_id_set_data_full (datalist, key_id, data, nil);
  end;

procedure g_datalist_id_remove_data (datalist: PPGData; key_id:TGQuark);
  begin
    g_datalist_id_set_data (datalist, key_id, NULL);
  end;

function  g_datalist_get_data(datalist: PPGData; key_str:PGChar):PPGData;
  begin
    g_datalist_get_data := g_datalist_id_get_data (datalist, g_quark_try_string (key_str));
  end;

procedure g_datalist_set_data_full(datalist: PPGData; key_str:PGChar; data:gpointer; destroy_func:TGDestroyNotify);
  begin
    g_datalist_id_set_data_full (datalist, g_quark_from_string (key_str), data, destroy_func);
  end;

procedure g_datalist_set_data (datalist: PPGData; key_str:PGChar; data:gpointer);
  begin
    g_datalist_set_data_full (datalist, key_str, data, nil);
  end;

procedure g_datalist_remove_no_notify(datalist: PPGData; key_str:PGChar);
  begin
    g_datalist_id_remove_no_notify (datalist, g_quark_try_string (key_str) );
  end;

procedure g_datalist_remove_data(datalist:PPGData; key_str:PGChar);
  begin
    g_datalist_id_set_data (datalist, g_quark_try_string (key_str), nil);
  end;

procedure g_dataset_id_set_data(location: gconstpointer; key_id:TGQuark; data:gpointer);
  begin
    g_dataset_id_set_data_full (location, key_id, data, nil);
  end;

procedure g_dataset_id_remove_data(location: gconstpointer; key_id:TGQuark);
  begin
    g_dataset_id_set_data (location, key_id, nil);
  end;

function  g_dataset_get_data(location: gconstpointer; key_str:PGChar): gpointer;
  begin
    g_dataset_get_data:= g_dataset_id_get_data (location, g_quark_try_string (key_str));
  end;

procedure g_dataset_set_data_full(location: gconstpointer; key_str:PGChar; data:gpointer; destroy_func:TGDestroyNotify);
  begin
    g_dataset_id_set_data_full (location, g_quark_from_string (key_str), data, destroy_func);
  end;

procedure g_dataset_remove_no_notify(location: gconstpointer; key_str:PGChar);
  begin
    g_dataset_id_remove_no_notify (location, g_quark_try_string (key_str));
  end;

procedure g_dataset_set_data(location: gconstpointer; key_str:PGChar; data:gpointer);
  begin
    g_dataset_set_data_full (location, key_str, data, nil);
  end;

procedure g_dataset_remove_data(location:gconstpointer; key_str: PGChar);
  begin
    g_dataset_id_set_data (location, g_quark_try_string (key_str), nil);
  end;
{*
 * gfileutils.inc
 *}
function G_FILE_ERROR: TGQuark;
  begin
    G_FILE_ERROR:=g_file_error_quark;
  end;

{*
 *  ghook.inc
 *}
function  TGHookList_hook_size     (var a : TGHookList) : guint;
  begin
     TGHookList_hook_size:=(a.flag0 and bm_TGHookList_hook_size) shr bp_TGHookList_hook_size;
  end;

procedure TGHookList_set_hook_size (var a : TGHookList; __hook_size : guint);
  begin
     a.flag0:=a.flag0 or ((__hook_size shl bp_TGHookList_hook_size) and bm_TGHookList_hook_size);
  end;

function  TGHookList_is_setup      (var a : TGHookList) : guint;
  begin
     TGHookList_is_setup:=(a.flag0 and bm_TGHookList_is_setup) shr bp_TGHookList_is_setup;
  end;

procedure TGHookList_set_is_setup  (var a : TGHookList; __is_setup : guint);
  begin
     a.flag0:=a.flag0 or ((__is_setup shl bp_TGHookList_is_setup) and bm_TGHookList_is_setup);
  end;

function G_HOOK(hook : pointer) : PGHook;
  begin
    G_HOOK := PGHook(hook);
  end;

function G_HOOK_FLAGS(hook : PGHook) : guint;
  begin
    G_HOOK_FLAGS := hook^.flags;
  end;
{ from the old glib}
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
{ end of old glib}

procedure g_hook_append (hook_list: PGHookList; hook:PGHook);
  begin
    g_hook_insert_before (hook_list, nil, hook);
  end;

{*
 * giochannel.inc
 *}
 function G_IO_CHANNEL_ERROR : TGQuark;
    begin
       G_IO_CHANNEL_ERROR:=g_io_channel_error_quark;
    end;

function TGIOChannel_use_buffer(var a : TGIOChannel) : guint;
  begin
     TGIOChannel_use_buffer:=(a.flag0 and bm_TGIOChannel_use_buffer) shr bp_TGIOChannel_use_buffer;
  end;

procedure TGIOChannel_set_use_buffer(var a : TGIOChannel; __use_buffer : guint);
  begin
     a.flag0:=a.flag0 or ((__use_buffer shl bp_TGIOChannel_use_buffer) and bm_TGIOChannel_use_buffer);
  end;

function TGIOChannel_do_encode(var a : TGIOChannel) : guint;
  begin
     TGIOChannel_do_encode:=(a.flag0 and bm_TGIOChannel_do_encode) shr bp_TGIOChannel_do_encode;
  end;

procedure TGIOChannel_set_do_encode(var a : TGIOChannel; __do_encode : guint);
  begin
     a.flag0:=a.flag0 or ((__do_encode shl bp_TGIOChannel_do_encode) and bm_TGIOChannel_do_encode);
  end;

function TGIOChannel_close_on_unref(var a : TGIOChannel) : guint;
  begin
     TGIOChannel_close_on_unref:=(a.flag0 and bm_TGIOChannel_close_on_unref) shr bp_TGIOChannel_close_on_unref;
  end;

procedure TGIOChannel_set_close_on_unref(var a : TGIOChannel; __close_on_unref : guint);
  begin
     a.flag0:=a.flag0 or ((__close_on_unref shl bp_TGIOChannel_close_on_unref) and bm_TGIOChannel_close_on_unref);
  end;

function TGIOChannel_is_readable(var a : TGIOChannel) : guint;
  begin
     TGIOChannel_is_readable:=(a.flag0 and bm_TGIOChannel_is_readable) shr bp_TGIOChannel_is_readable;
  end;

procedure TGIOChannel_set_is_readable(var a : TGIOChannel; __is_readable : guint);
  begin
     a.flag0:=a.flag0 or ((__is_readable shl bp_TGIOChannel_is_readable) and bm_TGIOChannel_is_readable);
  end;

function TGIOChannel_is_writeable(var a : TGIOChannel) : guint;
  begin
     TGIOChannel_is_writeable:=(a.flag0 and bm_TGIOChannel_is_writeable) shr bp_TGIOChannel_is_writeable;
  end;

procedure TGIOChannel_set_is_writeable(var a : TGIOChannel; __is_writeable : guint);
  begin
     a.flag0:=a.flag0 or ((__is_writeable shl bp_TGIOChannel_is_writeable) and bm_TGIOChannel_is_writeable);
  end;

function TGIOChannel_is_seekable(var a : TGIOChannel) : guint;
  begin
     TGIOChannel_is_seekable:=(a.flag0 and bm_TGIOChannel_is_seekable) shr bp_TGIOChannel_is_seekable;
  end;

procedure TGIOChannel_set_is_seekable(var a : TGIOChannel; __is_seekable : guint);
  begin
     a.flag0:=a.flag0 or ((__is_seekable shl bp_TGIOChannel_is_seekable) and bm_TGIOChannel_is_seekable);
  end;
{*
 *  gunicode.inc
 *}
function g_utf8_next_char (p: pguchar):pguchar;
 begin
   {$IFNDEF KYLIX}
   g_utf8_next_char :=  p +  ord ((g_utf8_skip + p^ )^);     // needs to be tested
   {$ENDIF}
 end;

{*
 *  gutils.inc
 *}
function GLIB_CHECK_VERSION (major, minor, micro: guint):boolean;
  begin
   {$IFNDEF KYLIX}
    GLIB_CHECK_VERSION := ( (GLIB_MAJOR_VERSION > major) or
            ((GLIB_MAJOR_VERSION = major) and (GLIB_MINOR_VERSION > minor)) or
            ((GLIB_MAJOR_VERSION = major) and (GLIB_MINOR_VERSION = minor) and (GLIB_MICRO_VERSION >= micro)));
   {$ENDIF}
  end;
{*
 * gmessages.inc
 *}
procedure g_error    (format:Pgchar; args: array of const);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_ERROR, Pgchar(SysUtils.Format(string(format), args)));
end;

procedure g_error    (format:Pgchar);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_ERROR, format);
end;

procedure g_message  (format:Pgchar; args: array of const);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_MESSAGE, Pgchar(SysUtils.Format(string(format), args)));
end;

procedure g_message  (format:Pgchar);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_MESSAGE, format);
end;

procedure g_critical (format:Pgchar; args: array of const);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL, Pgchar(SysUtils.Format(string(format), args)));
end;

procedure g_critical (format:Pgchar);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL, format);
end;

procedure g_warning  (format:Pgchar; args: array of const);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING, Pgchar(SysUtils.Format(string(format), args)));
end;

procedure g_warning  (format:Pgchar);
begin
  g_log (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING, format);
end;

{*
 * gmarkup.inc
 *}
function G_MARKUP_ERROR : TGQuark;
begin
   G_MARKUP_ERROR:=g_markup_error_quark;
end;

{*
 * gnode.inc
 *}
function G_NODE_IS_ROOT (node: PGNode): boolean;
begin
  G_NODE_IS_ROOT := (node^.parent = nil) and (node^.next = nil) and (node^.prev = nil);
end;

function G_NODE_IS_LEAF (node: PGNode): boolean;
begin
  G_NODE_IS_LEAF := node^.children = nil;
end;

function  g_node_append (parent: PGNode; node: PGNode): PGNode;
begin
   g_node_append := g_node_insert_before (parent, nil, node);
end;

function  g_node_insert_data (parent: PGNode; position: gint; data: gpointer): PGNode;
begin
   g_node_insert_data := g_node_insert (parent, position, g_node_new(data));
end;

function  g_node_insert_data_before (parent: PGNode; sibling: PGNode; data: gpointer): PGNode;
begin
   g_node_insert_data_before := g_node_insert_before (parent, sibling, g_node_new(data));
end;

function  g_node_prepend_data (parent: PGNode; data: gpointer): PGNode;
begin
   g_node_prepend_data := g_node_prepend (parent, g_node_new(data));
end;

function  g_node_append_data (parent: PGNode; data: gpointer): PGNode;
begin
   g_node_append_data := g_node_insert_before (parent, nil, g_node_new(data));
end;

function g_node_prev_sibling (node : PGNode): PGNode;
begin
  if node <> nil then
    g_node_prev_sibling := node^.prev
  else
    g_node_prev_sibling := nil;
end;

function g_node_next_sibling (node : PGNode): PGNode;
begin
  if node <> nil then
    g_node_next_sibling := node^.next
  else
    g_node_next_sibling := nil;
end;

function g_node_first_child (node : PGNode): PGNode;
begin
  if node <> nil then
    g_node_first_child := node^.children
  else
    g_node_first_child := nil;
end;

{*
 * grand.inc
 *}

function g_rand_boolean(rand : PGRand) : gboolean;
begin
   g_rand_boolean:=((g_rand_int(rand)) and (1 shl 15)) <> 0;
end;
{
function g_rand_boolean(rand : PGRand) :gboolean;
begin
   if ((g_rand_int(rand)) and (1 shl 15)) <> 0 then
     g_rand_boolean := 1
   else
     g_rand_boolean := 0;
end;
}
function g_random_boolean : gboolean;
begin
   g_random_boolean:=((g_random_int) and (1 shl 15)) <> 0;
end;
{
function g_random_boolean : gboolean;
begin
   if (((g_random_int) and (1 shl 15))) <> 0 then
     g_random_boolean := 1
   else
     g_random_boolean := 0;
end;
}

{*
 * gscanner.inc
 *}
function TGScannerConfig_case_sensitive(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_case_sensitive:=(a.flag0 and bm_TGScannerConfig_case_sensitive) shr bp_TGScannerConfig_case_sensitive;
end;

procedure TGScannerConfig_set_case_sensitive(var a : TGScannerConfig; __case_sensitive : guint);
begin
   a.flag0:=a.flag0 or ((__case_sensitive shl bp_TGScannerConfig_case_sensitive) and bm_TGScannerConfig_case_sensitive);
end;

function TGScannerConfig_skip_comment_multi(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_skip_comment_multi:=(a.flag0 and bm_TGScannerConfig_skip_comment_multi) shr bp_TGScannerConfig_skip_comment_multi;
end;

procedure TGScannerConfig_set_skip_comment_multi(var a : TGScannerConfig; __skip_comment_multi : guint);
begin
   a.flag0:=a.flag0 or ((__skip_comment_multi shl bp_TGScannerConfig_skip_comment_multi) and bm_TGScannerConfig_skip_comment_multi);
end;

function TGScannerConfig_skip_comment_single(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_skip_comment_single:=(a.flag0 and bm_TGScannerConfig_skip_comment_single) shr bp_TGScannerConfig_skip_comment_single;
end;

procedure TGScannerConfig_set_skip_comment_single(var a : TGScannerConfig; __skip_comment_single : guint);
begin
   a.flag0:=a.flag0 or ((__skip_comment_single shl bp_TGScannerConfig_skip_comment_single) and bm_TGScannerConfig_skip_comment_single);
end;

function TGScannerConfig_scan_comment_multi(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_comment_multi:=(a.flag0 and bm_TGScannerConfig_scan_comment_multi) shr bp_TGScannerConfig_scan_comment_multi;
end;

procedure TGScannerConfig_set_scan_comment_multi(var a : TGScannerConfig; __scan_comment_multi : guint);
begin
   a.flag0:=a.flag0 or ((__scan_comment_multi shl bp_TGScannerConfig_scan_comment_multi) and bm_TGScannerConfig_scan_comment_multi);
end;

function TGScannerConfig_scan_identifier(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_identifier:=(a.flag0 and bm_TGScannerConfig_scan_identifier) shr bp_TGScannerConfig_scan_identifier;
end;

procedure TGScannerConfig_set_scan_identifier(var a : TGScannerConfig; __scan_identifier : guint);
begin
   a.flag0:=a.flag0 or ((__scan_identifier shl bp_TGScannerConfig_scan_identifier) and bm_TGScannerConfig_scan_identifier);
end;

function TGScannerConfig_scan_identifier_1char(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_identifier_1char:=(a.flag0 and bm_TGScannerConfig_scan_identifier_1char) shr bp_TGScannerConfig_scan_identifier_1char;
end;

procedure TGScannerConfig_set_scan_identifier_1char(var a : TGScannerConfig; __scan_identifier_1char : guint);
begin
   a.flag0:=a.flag0 or ((__scan_identifier_1char shl bp_TGScannerConfig_scan_identifier_1char) and bm_TGScannerConfig_scan_identifier_1char);
end;

function TGScannerConfig_scan_identifier_NULL(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_identifier_NULL:=(a.flag0 and bm_TGScannerConfig_scan_identifier_NULL) shr bp_TGScannerConfig_scan_identifier_NULL;
end;

procedure TGScannerConfig_set_scan_identifier_NULL(var a : TGScannerConfig; __scan_identifier_NULL : guint);
begin
   a.flag0:=a.flag0 or ((__scan_identifier_NULL shl bp_TGScannerConfig_scan_identifier_NULL) and bm_TGScannerConfig_scan_identifier_NULL);
end;

function TGScannerConfig_scan_symbols(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_symbols:=(a.flag0 and bm_TGScannerConfig_scan_symbols) shr bp_TGScannerConfig_scan_symbols;
end;

procedure TGScannerConfig_set_scan_symbols(var a : TGScannerConfig; __scan_symbols : guint);
begin
   a.flag0:=a.flag0 or ((__scan_symbols shl bp_TGScannerConfig_scan_symbols) and bm_TGScannerConfig_scan_symbols);
end;

function TGScannerConfig_scan_binary(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_binary:=(a.flag0 and bm_TGScannerConfig_scan_binary) shr bp_TGScannerConfig_scan_binary;
end;

procedure TGScannerConfig_set_scan_binary(var a : TGScannerConfig; __scan_binary : guint);
begin
   a.flag0:=a.flag0 or ((__scan_binary shl bp_TGScannerConfig_scan_binary) and bm_TGScannerConfig_scan_binary);
end;

function TGScannerConfig_scan_octal(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_octal:=(a.flag0 and bm_TGScannerConfig_scan_octal) shr bp_TGScannerConfig_scan_octal;
end;

procedure TGScannerConfig_set_scan_octal(var a : TGScannerConfig; __scan_octal : guint);
begin
   a.flag0:=a.flag0 or ((__scan_octal shl bp_TGScannerConfig_scan_octal) and bm_TGScannerConfig_scan_octal);
end;

function TGScannerConfig_scan_float(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_float:=(a.flag0 and bm_TGScannerConfig_scan_float) shr bp_TGScannerConfig_scan_float;
end;

procedure TGScannerConfig_set_scan_float(var a : TGScannerConfig; __scan_float : guint);
begin
   a.flag0:=a.flag0 or ((__scan_float shl bp_TGScannerConfig_scan_float) and bm_TGScannerConfig_scan_float);
end;

function TGScannerConfig_scan_hex(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_hex:=(a.flag0 and bm_TGScannerConfig_scan_hex) shr bp_TGScannerConfig_scan_hex;
end;

procedure TGScannerConfig_set_scan_hex(var a : TGScannerConfig; __scan_hex : guint);
begin
   a.flag0:=a.flag0 or ((__scan_hex shl bp_TGScannerConfig_scan_hex) and bm_TGScannerConfig_scan_hex);
end;

function TGScannerConfig_scan_hex_dollar(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_hex_dollar:=(a.flag0 and bm_TGScannerConfig_scan_hex_dollar) shr bp_TGScannerConfig_scan_hex_dollar;
end;

procedure TGScannerConfig_set_scan_hex_dollar(var a : TGScannerConfig; __scan_hex_dollar : guint);
begin
   a.flag0:=a.flag0 or ((__scan_hex_dollar shl bp_TGScannerConfig_scan_hex_dollar) and bm_TGScannerConfig_scan_hex_dollar);
end;

function TGScannerConfig_scan_string_sq(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_string_sq:=(a.flag0 and bm_TGScannerConfig_scan_string_sq) shr bp_TGScannerConfig_scan_string_sq;
end;

procedure TGScannerConfig_set_scan_string_sq(var a : TGScannerConfig; __scan_string_sq : guint);
begin
   a.flag0:=a.flag0 or ((__scan_string_sq shl bp_TGScannerConfig_scan_string_sq) and bm_TGScannerConfig_scan_string_sq);
end;

function TGScannerConfig_scan_string_dq(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scan_string_dq:=(a.flag0 and bm_TGScannerConfig_scan_string_dq) shr bp_TGScannerConfig_scan_string_dq;
end;

procedure TGScannerConfig_set_scan_string_dq(var a : TGScannerConfig; __scan_string_dq : guint);
begin
   a.flag0:=a.flag0 or ((__scan_string_dq shl bp_TGScannerConfig_scan_string_dq) and bm_TGScannerConfig_scan_string_dq);
end;

function TGScannerConfig_numbers_2_int(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_numbers_2_int:=(a.flag0 and bm_TGScannerConfig_numbers_2_int) shr bp_TGScannerConfig_numbers_2_int;
end;

procedure TGScannerConfig_set_numbers_2_int(var a : TGScannerConfig; __numbers_2_int : guint);
begin
   a.flag0:=a.flag0 or ((__numbers_2_int shl bp_TGScannerConfig_numbers_2_int) and bm_TGScannerConfig_numbers_2_int);
end;

function TGScannerConfig_int_2_float(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_int_2_float:=(a.flag0 and bm_TGScannerConfig_int_2_float) shr bp_TGScannerConfig_int_2_float;
end;

procedure TGScannerConfig_set_int_2_float(var a : TGScannerConfig; __int_2_float : guint);
begin
   a.flag0:=a.flag0 or ((__int_2_float shl bp_TGScannerConfig_int_2_float) and bm_TGScannerConfig_int_2_float);
end;

function TGScannerConfig_identifier_2_string(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_identifier_2_string:=(a.flag0 and bm_TGScannerConfig_identifier_2_string) shr bp_TGScannerConfig_identifier_2_string;
end;

procedure TGScannerConfig_set_identifier_2_string(var a : TGScannerConfig; __identifier_2_string : guint);
begin
   a.flag0:=a.flag0 or ((__identifier_2_string shl bp_TGScannerConfig_identifier_2_string) and bm_TGScannerConfig_identifier_2_string);
end;

function TGScannerConfig_char_2_token(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_char_2_token:=(a.flag0 and bm_TGScannerConfig_char_2_token) shr bp_TGScannerConfig_char_2_token;
end;

procedure TGScannerConfig_set_char_2_token(var a : TGScannerConfig; __char_2_token : guint);
begin
   a.flag0:=a.flag0 or ((__char_2_token shl bp_TGScannerConfig_char_2_token) and bm_TGScannerConfig_char_2_token);
end;

function TGScannerConfig_symbol_2_token(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_symbol_2_token:=(a.flag0 and bm_TGScannerConfig_symbol_2_token) shr bp_TGScannerConfig_symbol_2_token;
end;

procedure TGScannerConfig_set_symbol_2_token(var a : TGScannerConfig; __symbol_2_token : guint);
begin
   a.flag0:=a.flag0 or ((__symbol_2_token shl bp_TGScannerConfig_symbol_2_token) and bm_TGScannerConfig_symbol_2_token);
end;

function TGScannerConfig_scope_0_fallback(var a : TGScannerConfig) : guint;
begin
   TGScannerConfig_scope_0_fallback:=(a.flag0 and bm_TGScannerConfig_scope_0_fallback) shr bp_TGScannerConfig_scope_0_fallback;
end;

procedure TGScannerConfig_set_scope_0_fallback(var a : TGScannerConfig; __scope_0_fallback : guint);
begin
   a.flag0:=a.flag0 or ((__scope_0_fallback shl bp_TGScannerConfig_scope_0_fallback) and bm_TGScannerConfig_scope_0_fallback);
end;

procedure g_scanner_freeze_symbol_table(scanner : PGScanner);
begin
  { do nothing }
  if Scanner=nil then ;
end;

procedure g_scanner_thaw_symbol_table(scanner : PGScanner);
begin
  { do nothing }
  if Scanner=nil then ;
end;
{*
 * gshell.inc
 *}
function G_SHELL_ERROR : TGQuark;
begin
   G_SHELL_ERROR:=g_shell_error_quark;
end;

{*
 * gspawn.inc
 *}
function G_SPAWN_ERROR : TGQuark;
begin
   G_SPAWN_ERROR:=g_spawn_error_quark;
end;


// from gstrfuncs.inc ----------------------------------------------------------

function g_ascii_isalnum(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isalnum:=((g_ascii_table[guchar(c)]) and G_ASCII_ALNUM) <> 0;
  {$ENDIF}
end;

function g_ascii_isalpha(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isalpha:=((g_ascii_table[guchar(c)]) and G_ASCII_ALPHA) <> 0;
  {$ENDIF}
end;

function g_ascii_iscntrl(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_iscntrl:=((g_ascii_table[guchar(c)]) and G_ASCII_CNTRL) <> 0;
  {$ENDIF}
end;

function g_ascii_isdigit(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isdigit:=((g_ascii_table[guchar(c)]) and G_ASCII_DIGIT) <> 0;
  {$ENDIF}
end;

function g_ascii_isgraph(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isgraph:=((g_ascii_table[guchar(c)]) and G_ASCII_GRAPH) <> 0;
  {$ENDIF}
end;

function g_ascii_islower(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_islower:=((g_ascii_table[guchar(c)]) and G_ASCII_LOWER) <> 0;
  {$ENDIF}
end;

function g_ascii_isprint(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isprint:=((g_ascii_table[guchar(c)]) and G_ASCII_PRINT) <> 0;
  {$ENDIF}
end;

function g_ascii_ispunct(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_ispunct:=((g_ascii_table[guchar(c)]) and G_ASCII_PUNCT) <> 0;
  {$ENDIF}
end;

function g_ascii_isspace(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isspace:=((g_ascii_table[guchar(c)]) and G_ASCII_SPACE) <> 0;
  {$ENDIF}
end;

function g_ascii_isupper(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isupper:=((g_ascii_table[guchar(c)]) and G_ASCII_UPPER) <> 0;
  {$ENDIF}
end;

function g_ascii_isxdigit(c : gchar) : boolean;
begin
  {$IFNDEF KYLIX}
   g_ascii_isxdigit:=((g_ascii_table[guchar(c)]) and G_ASCII_XDIGIT) <> 0;
  {$ENDIF}
end;

function g_strstrip(_string : PGChar) : PGChar;
begin
   g_strstrip:=g_strchomp(g_strchug(_string));
end;

// gtype.inc -------------------------------------------------------------------

function G_TYPE_MAKE_FUNDAMENTAL(x : longint) : GType;
begin
   G_TYPE_MAKE_FUNDAMENTAL:=GType(x shl G_TYPE_FUNDAMENTAL_SHIFT);
end;

function G_TYPE_IS_FUNDAMENTAL(_type : GType) : boolean;
begin
   G_TYPE_IS_FUNDAMENTAL:=_type <= G_TYPE_FUNDAMENTAL_MAX;
end;

function G_TYPE_IS_DERIVED(_type : GType) : boolean;
begin
   G_TYPE_IS_DERIVED:=_type > G_TYPE_FUNDAMENTAL_MAX;
end;

function G_TYPE_IS_INTERFACE(_type : GType) : boolean;
begin
   G_TYPE_IS_INTERFACE:=(G_TYPE_FUNDAMENTAL(_type)) = G_TYPE_INTERFACE;
end;

function G_TYPE_IS_CLASSED(_type : GType) : gboolean;
begin
   G_TYPE_IS_CLASSED:=private_g_type_test_flags(_type,G_TYPE_FLAG_CLASSED);
end;

function G_TYPE_IS_INSTANTIATABLE(_type : GType) : boolean;
begin
   G_TYPE_IS_INSTANTIATABLE:=private_g_type_test_flags(_type,
                                                    G_TYPE_FLAG_INSTANTIATABLE);
end;

function G_TYPE_IS_DERIVABLE(_type : GType) : boolean;
begin
   G_TYPE_IS_DERIVABLE:=private_g_type_test_flags(_type,G_TYPE_FLAG_DERIVABLE);
end;

function G_TYPE_IS_DEEP_DERIVABLE(_type : GType) : boolean;
begin
   G_TYPE_IS_DEEP_DERIVABLE:=private_g_type_test_flags(_type,G_TYPE_FLAG_DEEP_DERIVABLE);
end;

function G_TYPE_IS_ABSTRACT(_type : GType) : boolean;
begin
   G_TYPE_IS_ABSTRACT:=private_g_type_test_flags(_type,G_TYPE_FLAG_ABSTRACT);
end;

function G_TYPE_IS_VALUE_ABSTRACT(_type : GType) : boolean;
begin
   G_TYPE_IS_VALUE_ABSTRACT:=private_g_type_test_flags(_type,G_TYPE_FLAG_VALUE_ABSTRACT);
end;

function G_TYPE_IS_VALUE_TYPE(_type : GType) : boolean;
begin
   G_TYPE_IS_VALUE_TYPE:=private_g_type_check_is_value_type(_type);
end;

function G_TYPE_HAS_VALUE_TABLE(_type : GType) : boolean;
begin
   G_TYPE_HAS_VALUE_TABLE:=(g_type_value_table_peek(_type)) <> NULL;
end;

function G_TYPE_CHECK_INSTANCE(instance : Pointer) : gboolean;
begin
  G_TYPE_CHECK_INSTANCE:=private_g_type_check_instance(PGTypeInstance(instance));
end;

function G_TYPE_CHECK_INSTANCE_CAST(instance: Pointer; g_type: GType) : PGTypeInstance;
begin
   G_TYPE_CHECK_INSTANCE_CAST:=private_g_type_check_instance_cast(instance,g_type);
end;

function G_TYPE_CHECK_INSTANCE_TYPE(instance: Pointer; g_type: GType) : boolean;
begin
   G_TYPE_CHECK_INSTANCE_TYPE:=private_g_type_check_instance_is_a(instance,g_type);
end;

function G_TYPE_INSTANCE_GET_CLASS(instance: Pointer; g_type: GType) : PGTypeClass;
// #define G_TYPE_INSTANCE_GET_CLASS(instance, g_type, c_type)     (_G_TYPE_IGC ((instance), (g_type), c_type))
// #define _G_TYPE_IGC(ip, gt, ct)         ((ct*) (((GTypeInstance*) ip)->g_class))
begin
   Result:=PGTypeInstance(Instance)^.g_class;
   Result:=private_g_type_check_class_cast(Result,g_type);
end;

function G_TYPE_INSTANCE_GET_INTERFACE(instance: Pointer; g_type: GType) : Pointer;
begin
   G_TYPE_INSTANCE_GET_INTERFACE:=
     g_type_interface_peek((PGTypeInstance(instance))^.g_class,g_type);
end;

function G_TYPE_CHECK_CLASS_CAST(g_class: pointer; g_type: GType) : Pointer;
begin
   G_TYPE_CHECK_CLASS_CAST:=
     private_g_type_check_class_cast(PGTypeClass(g_class),g_type);
end;

function G_TYPE_CHECK_CLASS_TYPE(g_class: pointer; g_type : GType) : boolean;
begin
   G_TYPE_CHECK_CLASS_TYPE:=private_g_type_check_class_is_a(PGTypeClass(g_class),g_type);
end;

function G_TYPE_CHECK_VALUE(value : Pointer) : boolean;
begin
   G_TYPE_CHECK_VALUE:=private_g_type_check_value(PGValue(Value));
end;

function G_TYPE_CHECK_VALUE_TYPE(value: pointer; g_type : GType) : boolean;
begin
   G_TYPE_CHECK_VALUE_TYPE:=private_g_type_check_value_holds(PGValue(value),g_type);
end;

function G_TYPE_FROM_INSTANCE(instance : Pointer) : GType;
begin
   G_TYPE_FROM_INSTANCE:=G_TYPE_FROM_CLASS((PGTypeInstance(instance))^.g_class);
end;

function G_TYPE_FROM_CLASS(g_class : Pointer) : GType;
begin
   G_TYPE_FROM_CLASS:=(PGTypeClass(g_class))^.g_type;
end;

function G_TYPE_FROM_INTERFACE(g_iface : Pointer) : GType;
begin
   G_TYPE_FROM_INTERFACE:=(PGTypeInterface(g_iface))^.g_type;
end;

// gvalue.inc ------------------------------------------------------------------

function G_TYPE_IS_VALUE(_type : GType) : boolean;
begin
   G_TYPE_IS_VALUE:=private_g_type_check_is_value_type(_type);
end;

function G_IS_VALUE(value : Pointer) : boolean;
begin
   G_IS_VALUE:=G_TYPE_CHECK_VALUE(value);
end;

function G_VALUE_TYPE(value : Pointer) : GType;
begin
   G_VALUE_TYPE:=(PGValue(value))^.g_type;
end;

function G_VALUE_TYPE_NAME(value : Pointer) : PGChar;
begin
   G_VALUE_TYPE_NAME:=g_type_name(G_VALUE_TYPE(value));
end;

function G_VALUE_HOLDS(value: pointer; g_type : GType) : boolean;
begin
   G_VALUE_HOLDS:=G_TYPE_CHECK_VALUE_TYPE(value,g_type);
end;

// gparam.inc ------------------------------------------------------------------

function G_TYPE_IS_PARAM(_type : GType) : boolean;
begin
   G_TYPE_IS_PARAM:=(G_TYPE_FUNDAMENTAL(_type)) = G_TYPE_PARAM;
end;

function G_PARAM_SPEC(pspec : Pointer) : PGParamSpec;
begin
   G_PARAM_SPEC:=PGParamSpec(G_TYPE_CHECK_INSTANCE_CAST(pspec,G_TYPE_PARAM));
end;

function G_IS_PARAM_SPEC(pspec : Pointer) : boolean;
begin
   G_IS_PARAM_SPEC:=G_TYPE_CHECK_INSTANCE_TYPE(pspec,G_TYPE_PARAM);
end;

function G_PARAM_SPEC_CLASS(pclass : Pointer) : PGParamSpecClass;
begin
   G_PARAM_SPEC_CLASS:=PGParamSpecClass(G_TYPE_CHECK_CLASS_CAST(pclass,G_TYPE_PARAM));
end;

function G_IS_PARAM_SPEC_CLASS(pclass : Pointer) : boolean;
begin
   G_IS_PARAM_SPEC_CLASS:=G_TYPE_CHECK_CLASS_TYPE(pclass,G_TYPE_PARAM);
end;

function G_PARAM_SPEC_GET_CLASS(pspec : Pointer) : PGParamSpecClass;
begin
   G_PARAM_SPEC_GET_CLASS:=PGParamSpecClass(G_TYPE_INSTANCE_GET_CLASS(pspec,G_TYPE_PARAM));
end;

function G_PARAM_SPEC_TYPE(pspec : Pointer) : GType;
begin
   G_PARAM_SPEC_TYPE:=G_TYPE_FROM_INSTANCE(pspec);
end;

function G_PARAM_SPEC_TYPE_NAME(pspec : Pointer) : PGChar;
begin
   G_PARAM_SPEC_TYPE_NAME:=g_type_name(G_PARAM_SPEC_TYPE(pspec));
end;

function G_PARAM_SPEC_VALUE_TYPE(pspec : Pointer) : GType;
begin
   G_PARAM_SPEC_VALUE_TYPE:=(G_PARAM_SPEC(pspec))^.value_type;
end;

function G_VALUE_HOLDS_PARAM(value : Pointer) : boolean;
begin
   G_VALUE_HOLDS_PARAM:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_PARAM);
end;

// gclosure.inc ----------------------------------------------------------------

function G_CLOSURE_NEEDS_MARSHAL(closure : Pointer) : boolean;
begin
   G_CLOSURE_NEEDS_MARSHAL := not Assigned((PGClosure(closure))^.marshal);
end;

function G_CLOSURE_N_NOTIFIERS(cl : PGClosure) : longint;
begin
   G_CLOSURE_N_NOTIFIERS:=((meta_marshal(cl) + ((n_guards(cl)) shl 1))
        + (n_fnotifiers(cl))) + (n_inotifiers(cl));
end;

function G_CCLOSURE_SWAP_DATA(cclosure : PGClosure) : longint;
begin
   G_CCLOSURE_SWAP_DATA:=derivative_flag(cclosure);
end;

function G_CALLBACK(f : pointer) : TGCallback;
begin
   G_CALLBACK:=TGCallback(f);
end;

function ref_count(var a : TGClosure) : guint;
begin
   ref_count:=(a.flag0 and bm_TGClosure_ref_count) shr bp_TGClosure_ref_count;
end;

procedure set_ref_count(var a : TGClosure; __ref_count : guint);
begin
   a.flag0:=a.flag0 or ((__ref_count shl bp_TGClosure_ref_count) and bm_TGClosure_ref_count);
end;

function meta_marshal(a : PGClosure) : guint;
begin
   meta_marshal:=(a^.flag0 and bm_TGClosure_meta_marshal) shr bp_TGClosure_meta_marshal;
end;

procedure set_meta_marshal(var a : TGClosure; __meta_marshal : guint);
begin
   a.flag0:=a.flag0 or ((__meta_marshal shl bp_TGClosure_meta_marshal) and bm_TGClosure_meta_marshal);
end;

function n_guards(a : PGClosure) : guint;
begin
   n_guards:=(a^.flag0 and bm_TGClosure_n_guards) shr bp_TGClosure_n_guards;
end;

procedure set_n_guards(var a : TGClosure; __n_guards : guint);
begin
   a.flag0:=a.flag0 or ((__n_guards shl bp_TGClosure_n_guards) and bm_TGClosure_n_guards);
end;

function n_fnotifiers(a : PGClosure) : guint;
begin
   n_fnotifiers:=(a^.flag0 and bm_TGClosure_n_fnotifiers) shr bp_TGClosure_n_fnotifiers;
end;

procedure set_n_fnotifiers(var a : TGClosure; __n_fnotifiers : guint);
begin
   a.flag0:=a.flag0 or ((__n_fnotifiers shl bp_TGClosure_n_fnotifiers) and bm_TGClosure_n_fnotifiers);
end;

function n_inotifiers(a : PGClosure) : guint;
begin
   n_inotifiers:=(a^.flag0 and bm_TGClosure_n_inotifiers) shr bp_TGClosure_n_inotifiers;
end;

procedure set_n_inotifiers(var a : TGClosure; __n_inotifiers : guint);
begin
   a.flag0:=a.flag0 or ((__n_inotifiers shl bp_TGClosure_n_inotifiers) and bm_TGClosure_n_inotifiers);
end;

function in_inotify(var a : TGClosure) : guint;
begin
   in_inotify:=(a.flag0 and bm_TGClosure_in_inotify) shr bp_TGClosure_in_inotify;
end;

procedure set_in_inotify(var a : TGClosure; __in_inotify : guint);
begin
   a.flag0:=a.flag0 or ((__in_inotify shl bp_TGClosure_in_inotify) and bm_TGClosure_in_inotify);
end;

function floating(var a : TGClosure) : guint;
begin
   floating:=(a.flag0 and bm_TGClosure_floating) shr bp_TGClosure_floating;
end;

procedure set_floating(var a : TGClosure; __floating : guint);
begin
   a.flag0:=a.flag0 or ((__floating shl bp_TGClosure_floating) and bm_TGClosure_floating);
end;

function derivative_flag(a : PGClosure) : guint;
begin
   derivative_flag:=(a^.flag0 and bm_TGClosure_derivative_flag) shr bp_TGClosure_derivative_flag;
end;

procedure set_derivative_flag(var a : TGClosure; __derivative_flag : guint);
begin
   a.flag0:=a.flag0 or ((__derivative_flag shl bp_TGClosure_derivative_flag) and bm_TGClosure_derivative_flag);
end;

function in_marshal(var a : TGClosure) : guint;
begin
   in_marshal:=(a.flag0 and bm_TGClosure_in_marshal) shr bp_TGClosure_in_marshal;
end;

procedure set_in_marshal(var a : TGClosure; __in_marshal : guint);
begin
   a.flag0:=a.flag0 or ((__in_marshal shl bp_TGClosure_in_marshal) and bm_TGClosure_in_marshal);
end;

function is_invalid(var a : TGClosure) : guint;
begin
   is_invalid:=(a.flag0 and bm_TGClosure_is_invalid) shr bp_TGClosure_is_invalid;
end;

procedure set_is_invalid(var a : TGClosure; __is_invalid : guint);
begin
   a.flag0:=a.flag0 or ((__is_invalid shl bp_TGClosure_is_invalid) and bm_TGClosure_is_invalid);
end;

// gsignal.inc -----------------------------------------------------------------

function g_signal_connect(instance:gpointer; detailed_signal:Pgchar;
  c_handler:TGCallback; data:gpointer) : gulong;
begin
   g_signal_connect:=g_signal_connect_data(instance,detailed_signal,c_handler,
                                           data,NULL,TGConnectFlags(0));
end;

function g_signal_connect_after(instance:gpointer; detailed_signal:Pgchar;
  c_handler:TGCallback; data:gpointer) : gulong;
begin
   g_signal_connect_after:=g_signal_connect_data(instance,detailed_signal,
                                           c_handler,data,NULL,G_CONNECT_AFTER);
end;

function g_signal_connect_swapped(instance:gpointer; detailed_signal:Pgchar;
  c_handler:TGCallback; data:gpointer) : gulong;
begin
   g_signal_connect_swapped:=g_signal_connect_data(instance,detailed_signal,
                                         c_handler,data,NULL,G_CONNECT_SWAPPED);
end;

function g_signal_handlers_disconnect_by_func(instance:gpointer;
  func, data: gpointer) : guint;
begin
   g_signal_handlers_disconnect_by_func:=g_signal_handlers_disconnect_matched(
     instance,TGSignalMatchType(G_SIGNAL_MATCH_FUNC or G_SIGNAL_MATCH_DATA),0,0,
     NULL,func,data);
end;

procedure g_signal_handlers_block_by_func(instance: gpointer; func, data: gpointer);
begin
  g_signal_handlers_block_matched(instance,
    TGSignalMatchType(G_SIGNAL_MATCH_FUNC or G_SIGNAL_MATCH_DATA),
    0, 0, nil, func, data);
end;

procedure g_signal_handlers_unblock_by_func(instance: gpointer; func, data: gpointer);
begin
  g_signal_handlers_unblock_matched(instance,
    TGSignalMatchType(G_SIGNAL_MATCH_FUNC or G_SIGNAL_MATCH_DATA),
    0, 0, nil, func, data);
end;


// gobject.inc -----------------------------------------------------------------

function G_TYPE_IS_OBJECT(_type : GType) : boolean;
begin
   G_TYPE_IS_OBJECT:=(G_TYPE_FUNDAMENTAL(_type)) = G_TYPE_OBJECT;
end;

function G_OBJECT(anObject: pointer) : PGObject;
begin
   G_OBJECT:=PGObject(G_TYPE_CHECK_INSTANCE_CAST(anObject,G_TYPE_OBJECT));
end;

function G_OBJECT_CLASS(_class : Pointer) : PGObjectClass;
begin
   G_OBJECT_CLASS:=PGObjectClass(G_TYPE_CHECK_CLASS_CAST(_class,G_TYPE_OBJECT));
end;

function G_IS_OBJECT(anObject: pointer) : boolean;
begin
   G_IS_OBJECT:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,G_TYPE_OBJECT);
end;

function G_IS_OBJECT_CLASS(_class : Pointer) : boolean;
begin
   G_IS_OBJECT_CLASS:=G_TYPE_CHECK_CLASS_TYPE(_class,G_TYPE_OBJECT);
end;

function G_OBJECT_GET_CLASS(anObject: pointer) : PGObjectClass;
begin
   G_OBJECT_GET_CLASS:=PGObjectClass(G_TYPE_INSTANCE_GET_CLASS(anObject,G_TYPE_OBJECT));
end;

function G_OBJECT_TYPE(anObject: pointer) : GType;
begin
   G_OBJECT_TYPE:=G_TYPE_FROM_INSTANCE(anObject);
end;

function G_OBJECT_TYPE_NAME(anObject: pointer) : Pgchar;
begin
   G_OBJECT_TYPE_NAME:=g_type_name(G_OBJECT_TYPE(anObject));
end;

function G_OBJECT_CLASS_TYPE(_class : Pointer) : GType;
begin
   G_OBJECT_CLASS_TYPE:=G_TYPE_FROM_CLASS(_class);
end;

function G_OBJECT_CLASS_NAME(_class : Pointer) : Pgchar;
begin
   G_OBJECT_CLASS_NAME:=g_type_name(G_OBJECT_CLASS_TYPE(_class));
end;

function G_VALUE_HOLDS_OBJECT(value : Pointer) : boolean;
begin
   G_VALUE_HOLDS_OBJECT:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_OBJECT);
end;

procedure G_OBJECT_WARN_INVALID_PROPERTY_ID(anObject: gpointer;
  property_id: gint; pspec : gpointer);
begin
  G_OBJECT_WARN_INVALID_PSPEC(anObject,'property',property_id,pspec);
end;

procedure G_OBJECT_WARN_INVALID_PSPEC(anObject: gpointer; pname: PGChar;
  property_id: gint; pspec: gpointer);
var
  _object: PGObject;
  _pspec: PGParamSpec;
  _property_id: guint;
begin
  _object := PGObject (anObject);
  _pspec := PGParamSpec (pspec);
  _property_id := (property_id);
  g_warning ('%s: invalid %s id %u for "%s" of type `%s'' in `%s''',
             ['',
             pname,
             _property_id,
             _pspec^.name,
             g_type_name (G_PARAM_SPEC_TYPE (_pspec)),
             G_OBJECT_TYPE_NAME (_object)]);
end;

// gtypeplugin.inc -------------------------------------------------------------



function G_TYPE_TYPE_PLUGIN : GType;
begin
   G_TYPE_TYPE_PLUGIN:=g_type_plugin_get_type;
end;

function G_TYPE_PLUGIN(inst : Pointer) : PGTypePlugin;
begin
   G_TYPE_PLUGIN:=PGTypePlugin(G_TYPE_CHECK_INSTANCE_CAST(inst,G_TYPE_TYPE_PLUGIN));
end;

function G_TYPE_PLUGIN_CLASS(vtable : Pointer) : PGTypePluginClass;
begin
   G_TYPE_PLUGIN_CLASS:=PGTypePluginClass(G_TYPE_CHECK_CLASS_CAST(vtable,G_TYPE_TYPE_PLUGIN));
end;

function G_IS_TYPE_PLUGIN(inst : Pointer) : boolean;
begin
   G_IS_TYPE_PLUGIN:=G_TYPE_CHECK_INSTANCE_TYPE(inst,G_TYPE_TYPE_PLUGIN);
end;

function G_IS_TYPE_PLUGIN_CLASS(vtable : Pointer) : boolean;
begin
   G_IS_TYPE_PLUGIN_CLASS:=G_TYPE_CHECK_CLASS_TYPE(vtable,G_TYPE_TYPE_PLUGIN);
end;

function G_TYPE_PLUGIN_GET_CLASS(inst : Pointer) : PGTypePluginClass;
begin
   G_TYPE_PLUGIN_GET_CLASS:=PGTypePluginClass(G_TYPE_INSTANCE_GET_INTERFACE(inst,G_TYPE_TYPE_PLUGIN));
end;

// genums.inc ------------------------------------------------------------------


function G_TYPE_IS_ENUM(_type : GType) : gboolean;
begin
   G_TYPE_IS_ENUM:=(G_TYPE_FUNDAMENTAL(_type) = G_TYPE_ENUM);
end;

function G_ENUM_CLASS(_class : pointer) : PGEnumClass;
begin
   G_ENUM_CLASS:=PGEnumClass(G_TYPE_CHECK_CLASS_CAST(_class,G_TYPE_ENUM));
end;

function G_IS_ENUM_CLASS(_class : pointer) : gboolean;
begin
   G_IS_ENUM_CLASS:=G_TYPE_CHECK_CLASS_TYPE(_class,G_TYPE_ENUM);
end;

function G_ENUM_CLASS_TYPE(_class : pointer) : GType;
begin
   G_ENUM_CLASS_TYPE:=G_TYPE_FROM_CLASS(_class);
end;

function G_ENUM_CLASS_TYPE_NAME(_class : pointer) : PGChar;
begin
   G_ENUM_CLASS_TYPE_NAME:=g_type_name(G_ENUM_CLASS_TYPE(_class));
end;

function G_TYPE_IS_FLAGS(_type : GType) : gboolean;
begin
   G_TYPE_IS_FLAGS:=(G_TYPE_FUNDAMENTAL(_type)) = G_TYPE_FLAGS;
end;

function G_FLAGS_CLASS(_class : pointer) : PGFlagsClass;
begin
   G_FLAGS_CLASS:=PGFlagsClass(G_TYPE_CHECK_CLASS_CAST(_class,G_TYPE_FLAGS));
end;

function G_IS_FLAGS_CLASS(_class : pointer) : gboolean;
begin
   G_IS_FLAGS_CLASS:=G_TYPE_CHECK_CLASS_TYPE(_class,G_TYPE_FLAGS);
end;

function G_FLAGS_CLASS_TYPE(_class : pointer) : GType;
begin
   G_FLAGS_CLASS_TYPE:=G_TYPE_FROM_CLASS(_class);
end;

function G_FLAGS_CLASS_TYPE_NAME(_class : pointer) : PGChar;
begin
   G_FLAGS_CLASS_TYPE_NAME:=g_type_name(G_FLAGS_TYPE(PtrInt(_class)));
end;

function G_VALUE_HOLDS_ENUM(value : pointer) : gboolean;
begin
   G_VALUE_HOLDS_ENUM:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_ENUM);
end;

function G_VALUE_HOLDS_FLAGS(value : pointer) : gboolean;
begin
   G_VALUE_HOLDS_FLAGS:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_FLAGS);
end;

// gmacros.inc -----------------------------------------------------------------

function CLAMP(x, MinX, MaxX: integer): integer;
begin
  if x<MinX then
    Result:=MinX
  else if x>MaxX then
    Result:=MaxX
  else
    Result:=x;
end;

function GPOINTER_TO_SIZE(p: GPointer): GSize;
begin
  Result:=GSize(PtrInt(p));
end;

function GSIZE_TO_POINTER(s: GSize): GPointer;
begin
  Result:=GPointer(PtrInt(s));
end;

// gvaluetypes.inc -------------------------------------------------------------

function G_VALUE_HOLDS_CHAR(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_CHAR:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_CHAR);
end;

function G_VALUE_HOLDS_UCHAR(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_UCHAR:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_UCHAR);
end;

function G_VALUE_HOLDS_BOOLEAN(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_BOOLEAN:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_BOOLEAN);
end;

function G_VALUE_HOLDS_INT(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_INT:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_INT);
end;

function G_VALUE_HOLDS_UINT(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_UINT:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_UINT);
end;

function G_VALUE_HOLDS_LONG(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_LONG:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_LONG);
end;

function G_VALUE_HOLDS_ULONG(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_ULONG:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_ULONG);
end;

function G_VALUE_HOLDS_INT64(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_INT64:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_INT64);
end;

function G_VALUE_HOLDS_UINT64(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_UINT64:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_UINT64);
end;

function G_VALUE_HOLDS_FLOAT(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_FLOAT:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_FLOAT);
end;

function G_VALUE_HOLDS_DOUBLE(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_DOUBLE:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_DOUBLE);
end;

function G_VALUE_HOLDS_STRING(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_STRING:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_STRING);
end;

function G_VALUE_HOLDS_POINTER(value : PGValue) : boolean;
begin
   G_VALUE_HOLDS_POINTER:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_POINTER);
end;


// gboxed.inc ------------------------------------------------------------------

function G_TYPE_IS_BOXED(_type : GType) : gboolean;
begin
   G_TYPE_IS_BOXED:=(G_TYPE_FUNDAMENTAL(_type)) = G_TYPE_BOXED;
end;

function G_VALUE_HOLDS_BOXED(value : PGValue) : gboolean;
begin
   G_VALUE_HOLDS_BOXED:=G_TYPE_CHECK_VALUE_TYPE(value,G_TYPE_BOXED);
end;

function G_TYPE_CLOSURE : GType;
begin
   G_TYPE_CLOSURE:=g_closure_get_type;
end;

function G_TYPE_VALUE : GType;
begin
   G_TYPE_VALUE:=g_value_get_type;
end;

function G_TYPE_VALUE_ARRAY : GType;
begin
   G_TYPE_VALUE_ARRAY:=g_value_array_get_type;
end;

function G_TYPE_GSTRING : GType;
begin
   G_TYPE_GSTRING:=g_gstring_get_type;
end;


end.
