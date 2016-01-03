{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-address.h  Server address parser.
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

//#include <dbus/dbus-types.h>
//#include <dbus/dbus-errors.h>

type

  DBusAddressEntry = record end;

  PDBusAddressEntry = ^DBusAddressEntry;

  PPDBusAddressEntry = ^PDBusAddressEntry;

  PPPDBusAddressEntry = ^PPDBusAddressEntry;

function dbus_parse_address(const address: PChar; entry: PPPDBusAddressEntry;
 array_len: PInteger; error: PDBusError): dbus_bool_t; cdecl; external LibDBus;
function dbus_address_entry_get_value(entry: PDBusAddressEntry; const key: PChar): PChar; cdecl; external LibDBus;
function dbus_address_entry_get_method(entry: PDBusAddressEntry): PChar; cdecl; external LibDBus;
procedure dbus_address_entries_free(entries: PPDBusAddressEntry); cdecl; external LibDBus;

function dbus_address_escape_value(const value: PChar): PChar; cdecl; external LibDBus;
function dbus_address_unescape_value(const value: PChar; error: PDBusError): PChar; cdecl; external LibDBus;

