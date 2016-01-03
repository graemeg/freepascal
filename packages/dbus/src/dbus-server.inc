{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-server.h DBusServer object
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

{#include <dbus/dbus-errors.h>
#include <dbus/dbus-message.h>
#include <dbus/dbus-connection.h>
#include <dbus/dbus-protocol.h>}

type
  DBusServer = record end;
  PDBusServer = ^DBusServer;

  DBusNewConnectionFunction = procedure (server: PDBusServer;
   new_connection: PDBusConnection; data: Pointer);

function dbus_server_listen(const address: PChar; error: PDBusError): PDBusServer; cdecl; external LibDBus;
function dbus_server_ref(error: PDBusError): PDBusServer; cdecl; external LibDBus;
procedure dbus_server_unref(error: PDBusError); cdecl; external LibDBus;
procedure dbus_server_disconnect(error: PDBusError); cdecl; external LibDBus;
function dbus_server_get_is_connected(error: PDBusError): dbus_bool_t; cdecl; external LibDBus;
function dbus_server_get_address(error: PDBusError): PChar; cdecl; external LibDBus;
procedure dbus_server_set_new_connection_function(server: PDBusServer;
 function_: DBusNewConnectionFunction; data: Pointer; free_data_function: DBusFreeFunction); cdecl; external LibDBus;
function dbus_server_set_watch_functions(server: PDBusServer;
 add_function: DBusAddWatchFunction;
 remove_function: DBusRemoveWatchFunction;
 toggled_function: DBusWatchToggledFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_server_set_timeout_functions(server: PDBusServer;
 add_function: DBusAddTimeoutFunction;
 remove_function: DBusRemoveTimeoutFunction;
 toggled_function: DBusTimeoutToggledFunction;
 data: Pointer;
 free_data_function: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_server_set_auth_mechanisms(server: PDBusServer; const mechanisms: PPChar): dbus_bool_t; cdecl; external LibDBus;

function dbus_server_allocate_data_slot(slot_p: Pdbus_int32_t): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_server_free_data_slot(slot_p: Pdbus_int32_t); cdecl; external LibDBus;
function dbus_server_set_data(server: PDBusServer;
 slot: cint; data: Pointer; free_data_func: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_server_get_data(server: PDBusServer; slot: cint): Pointer; cdecl; external LibDBus;

