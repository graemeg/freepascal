{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-bus.h  Convenience functions for communicating with the bus.
 *
 * Copyright (C) 2003  CodeFactory AB
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

//#include <dbus/dbus-connection.h>

function dbus_bus_get(type_: DBusBusType; error: PDBusError): PDBusConnection; cdecl; external LibDBus;
function dbus_bus_get_private(type_: DBusBusType; error: PDBusError): PDBusConnection; cdecl; external LibDBus;
 
function dbus_bus_register(connection: PDBusConnection; error: PDBusError): dbus_bool_t; cdecl; external LibDBus;
function dbus_bus_set_unique_name(connection: PDBusConnection;
 const unique_name: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_bus_get_unique_name(connection: PDBusConnection): PChar; cdecl; external LibDBus;
function dbus_bus_get_unix_user(connection: PDBusConnection;
 const name: PChar; error: PDBusError): culong; cdecl; external LibDBus;
function dbus_bus_request_name(connection: PDBusConnection;
 const name: PChar; flags: cuint; error: PDBusError): cint; cdecl; external LibDBus;
function dbus_bus_release_name(connection: PDBusConnection;
 const name: PChar; error: PDBusError): cint; cdecl; external LibDBus;
function dbus_bus_name_has_owner(connection: PDBusConnection;
 const name: PChar; error: PDBusError): dbus_bool_t; cdecl; external LibDBus;

function dbus_bus_start_service_by_name (connection: PDBusConnection;
 const name: PChar; flags: dbus_uint32_t; reply: Pdbus_uint32_t;
 error: PDBusError): dbus_bool_t; cdecl; external LibDBus;

procedure dbus_bus_add_match(connection: PDBusConnection;
 const rule: PChar; error: PDBusError); cdecl; external LibDBus;
procedure dbus_bus_remove_match(connection: PDBusConnection;
 const rule: PChar; error: PDBusError); cdecl; external LibDBus;

procedure _dbus_bus_check_connection_and_unref (connection: PDBusConnection); cdecl; external LibDBus;

