{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-connection.h DBusConnection object
 *
 * Copyright (C) 2002, 2003  Red Hat Inc.
 *
 * Licensed under the Academic Free License version 2.1
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 }

{#include <dbus/dbus-errors.h>}
{.$include dbus-memory.inc}
{#include <dbus/dbus-message.h>}
{.$include dbus-shared.inc}

type
  DBusWatch = record end;
  PDBusWatch = ^DBusWatch;

  DBusTimeout = record end;
  PDBusTimeout = ^DBusTimeout;

  DBusPreallocatedSend = record end;
  PDBusPreallocatedSend = ^DBusPreallocatedSend;

  DBusPendingCall = record end;
  PDBusPendingCall = ^DBusPendingCall;
  PPDBusPendingCall = ^PDBusPendingCall;

  DBusConnection = record end;
  PDBusConnection = ^DBusConnection;

  DBusWatchFlags =
  (
    DBUS_WATCH_READABLE = 1 shl 0, {< As in POLLIN }
    DBUS_WATCH_WRITABLE = 1 shl 1, {< As in POLLOUT }
    DBUS_WATCH_ERROR    = 1 shl 2, {< As in POLLERR (can't watch for this, but
                                 *   the flag can be passed to dbus_watch_handle()).}
    DBUS_WATCH_HANGUP   = 1 shl 3  {< As in POLLHUP (can't watch for it, but
                                 *   can be present in current state). }
  );

  DBusDispatchStatus =
  (
    DBUS_DISPATCH_DATA_REMAINS,  {< There is more data to potentially convert to messages. }
    DBUS_DISPATCH_COMPLETE,      {< All currently available data has been processed. }
    DBUS_DISPATCH_NEED_MEMORY    {< More memory is needed to continue. }
  );

  DBusAddWatchFunction = function (watch: PDBusWatch; data: Pointer): dbus_bool_t;
  DBusWatchToggledFunction = procedure(watch: PDBusWatch; data: Pointer);
  DBusRemoveWatchFunction = procedure(watch: PDBusWatch; data: Pointer);
  DBusAddTimeoutFunction = function(timeout: PDBusTimeout; data: Pointer): dbus_bool_t;
  DBusTimeoutToggledFunction = procedure(timeout: PDBusTimeout; data: Pointer);
  DBusRemoveTimeoutFunction = procedure(timeout: PDBusTimeout; data: Pointer);
  DBusDispatchStatusFunction = procedure(connection: PDBusConnection;
   new_status: DBusDispatchStatus; data: Pointer);
  DBusWakeupMainFunction = procedure(data: Pointer);
  DBusAllowUnixUserFunction = function(connection: PDBusConnection;
   uid: cuint; data: Pointer): dbus_bool_t;

  DBusPendingCallNotifyFunction = procedure(pending: PDBusPendingCall;
   user_data: Pointer);

  DBusHandleMessageFunction = function(connection: PDBusConnection;
   message_: PDBusMessage; user_data: Pointer): DBusHandlerResult;

function dbus_connection_open(const address: PChar; error: PDBusError): PDBusConnection; cdecl; external LibDBus;
function dbus_connection_open_private(const address: PChar; error: PDBusError): PDBusConnection; cdecl; external LibDBus;
function dbus_connection_ref(connection: PDBusConnection): PDBusConnection; cdecl; external LibDBus;
procedure dbus_connection_unref(connection: PDBusConnection); cdecl; external LibDBus;
procedure dbus_connection_close(connection: PDBusConnection); cdecl; external LibDBus;
function dbus_connection_get_is_connected(connection: PDBusConnection): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_get_is_authenticated(connection: PDBusConnection): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_connection_set_exit_on_disconnect(connection: PDBusConnection;
 exit_on_disconnect: dbus_bool_t); cdecl; external LibDBus;
procedure dbus_connection_flush(connection: PDBusConnection); cdecl; external LibDBus;
function dbus_connection_read_write_dispatch(connection: PDBusConnection;
 timeout_milliseconds: Integer): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_read_write(connection: PDBusConnection;
 timeout_milliseconds: Integer): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_borrow_message(connection: PDBusConnection): PDBusMessage; cdecl; external LibDBus;
procedure dbus_connection_return_message(connection: PDBusConnection; message_: PDBusMessage); cdecl; external LibDBus;
procedure dbus_connection_steal_borrowed_message(connection: PDBusConnection; message_: PDBusMessage); cdecl; external LibDBus;
function dbus_connection_pop_message(connection: PDBusConnection): PDBusMessage; cdecl; external LibDBus;
function dbus_connection_get_dispatch_status(connection: PDBusConnection): DBusDispatchStatus; cdecl; external LibDBus;
function dbus_connection_dispatch(connection: PDBusConnection): DBusDispatchStatus; cdecl; external LibDBus;
function dbus_connection_has_messages_to_send(connection: PDBusConnection): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_send(connection: PDBusConnection; message_: PDBusMessage;
 client_serial: Pdbus_uint32_t): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_send_with_reply(connection: PDBusConnection;
 message_: PDBusMessage;
 pending_return: PPDBusPendingCall;
 timeout_milliseconds: cint): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_send_with_reply_and_block(connection: PDBusConnection;
 message_: PDBusMessage;
 timeout_milliseconds: cint;
 error: PDBusError): PDBusMessage; cdecl; external LibDBus;
function dbus_connection_set_watch_functions(connection: PDBusConnection;
 add_function: DBusAddWatchFunction;
 remove_function: DBusRemoveWatchFunction;
 toggled_function: DBusWatchToggledFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_set_timeout_functions(connection: PDBusConnection;
 add_function: DBusAddTimeoutFunction;
 remove_function: DBusRemoveTimeoutFunction;
 toggled_function: DBusTimeoutToggledFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_connection_set_wakeup_main_function(connection: PDBusConnection;
 wakeup_main_function: DBusWakeupMainFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction); cdecl; external LibDBus;
procedure dbus_connection_set_dispatch_status_function (connection: PDBusConnection;
 function_: DBusDispatchStatusFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction); cdecl; external LibDBus;
function dbus_connection_get_unix_user(connection: PDBusConnection;
 uid: Pculong): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_get_unix_process_id(connection: PDBusConnection;
 pid: Pculong): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_connection_set_unix_user_function(connection: PDBusConnection;
 function_: DBusAllowUnixUserFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction); cdecl; external LibDBus;


function dbus_watch_get_fd(watch: PDBusWatch): cint; cdecl; external LibDBus;
function dbus_watch_get_flags(watch: PDBusWatch): cuint; cdecl; external LibDBus;
function dbus_watch_get_data(watch: PDBusWatch): Pointer; cdecl; external LibDBus;
procedure dbus_watch_set_data(watch: PDBusWatch;
 data: Pointer; free_data_function: DBusFreeFunction); cdecl; external LibDBus;
function dbus_watch_handle(watch: PDBusWatch; flags: cuint): dbus_bool_t; cdecl; external LibDBus;
function  dbus_watch_get_enabled(watch: PDBusWatch): dbus_bool_t; cdecl; external LibDBus;

function dbus_timeout_get_interval(timeout: PDBusTimeout): cint; cdecl; external LibDBus;
function dbus_timeout_get_data(timeout: PDBusTimeout): Pointer; cdecl; external LibDBus;
procedure dbus_timeout_set_data(timeout: PDBusTimeout;
 data: Pointer; free_data_function: DBusFreeFunction); cdecl; external LibDBus;
function dbus_timeout_handle(timeout: PDBusTimeout): dbus_bool_t; cdecl; external LibDBus;
function dbus_timeout_get_enabled(timeout: PDBusTimeout): dbus_bool_t; cdecl; external LibDBus;

{ Filters }

function dbus_connection_add_filter(connection: PDBusConnection;
 function_: DBusHandleMessageFunction;
 user_data: Pointer; free_data_function: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_connection_remove_filter (connection: PDBusConnection;
 function_: DBusHandleMessageFunction; user_data: Pointer); cdecl; external LibDBus;


{ Other }
function dbus_connection_allocate_data_slot (slot_p: Pdbus_int32_t): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_connection_free_data_slot     (slot_p: Pdbus_int32_t); cdecl; external LibDBus;
function dbus_connection_set_data(connection: PDBusConnection;
 slot: dbus_int32_t; data: Pointer; free_data_func: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_get_data(connection: PDBusConnection; slot: dbus_int32_t): Pointer; cdecl; external LibDBus;

procedure dbus_connection_set_change_sigpipe (will_modify_sigpipe: dbus_bool_t); cdecl; external LibDBus;

procedure dbus_connection_set_max_message_size  (connection: PDBusConnection; size: clong); cdecl; external LibDBus;
function dbus_connection_get_max_message_size  (connection: PDBusConnection): clong; cdecl; external LibDBus;
procedure dbus_connection_set_max_received_size (connection: PDBusConnection; size: clong); cdecl; external LibDBus;
function dbus_connection_get_max_received_size (connection: PDBusConnection): clong; cdecl; external LibDBus;
function dbus_connection_get_outgoing_size     (connection: PDBusConnection): clong; cdecl; external LibDBus;

function dbus_connection_preallocate_send       (connection: PDBusConnection): PDBusPreallocatedSend; cdecl; external LibDBus;
procedure dbus_connection_free_preallocated_send (connection: PDBusConnection; preallocated: PDBusPreallocatedSend); cdecl; external LibDBus;
procedure dbus_connection_send_preallocated      (connection: PDBusConnection;
 preallocated: PDBusPreallocatedSend;
 message_: PDBusMessage; client_serial: Pdbus_uint32_t); cdecl; external LibDBus;


{ Object tree functionality }

type
  DBusObjectPathUnregisterFunction = procedure(connection: PDBusConnection;
   user_data: Pointer);
  DBusObjectPathMessageFunction = function(connection: PDBusConnection;
   message_: PDBusMessage; user_data: Pointer): DBusHandlerResult;

{
 * Virtual table that must be implemented to handle a portion of the
 * object path hierarchy.
 }
  dbus_internal_func = procedure(param1: Pointer);

  PDBusObjectPathVTable = ^DBusObjectPathVTable;

  DBusObjectPathVTable = record

    unregister_function: DBusObjectPathUnregisterFunction; {< Function to unregister this handler }
    message_function: DBusObjectPathMessageFunction; {< Function to handle messages }
  
    dbus_internal_pad1: dbus_internal_func; {< Reserved for future expansion }
    dbus_internal_pad2: dbus_internal_func; {< Reserved for future expansion }
    dbus_internal_pad3: dbus_internal_func; {< Reserved for future expansion }
    dbus_internal_pad4: dbus_internal_func; {< Reserved for future expansion }
  end;

function dbus_connection_register_object_path(connection: PDBusConnection;
 const path: PChar;
 const vtable: PDBusObjectPathVTable;
 user_data: Pointer): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_register_fallback(connection: PDBusConnection;
 const path: PChar;
 const vtable: PDBusObjectPathVTable;
 user_data: Pointer): dbus_bool_t; cdecl; external LibDBus;
function dbus_connection_unregister_object_path(connection: PDBusConnection;
 const path: PChar): dbus_bool_t; cdecl; external LibDBus;

function dbus_connection_get_object_path_data(connection: PDBusConnection;
 const path: PChar;
 data_p: PPointer): dbus_bool_t; cdecl; external LibDBus;

function dbus_connection_list_registered(connection: PDBusConnection;
 const parent_path: PChar;
 child_entries: PPPChar): dbus_bool_t; cdecl; external LibDBus;

function dbus_connection_get_unix_fd(connection: PDBusConnection;
 fd: Pcint): dbus_bool_t; cdecl; external LibDBus;

