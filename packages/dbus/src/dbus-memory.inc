{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-memory.h  D-Bus memory handling
 *
 * Copyright (C) 2002  Red Hat Inc.
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

{#include <dbus/dbus-macros.h>
#include <stddef.h>}

function dbus_malloc(bytes: size_t): Pointer; cdecl; external LibDBus;
function dbus_malloc0(bytes: size_t): Pointer; cdecl; external LibDBus;
function dbus_realloc(memory: Pointer; bytes: size_t): Pointer; cdecl; external LibDBus;
procedure dbus_free(memory: Pointer); cdecl; external LibDBus;

//#define dbus_new(type, count)  ((type*)dbus_malloc (sizeof (type) * (count)));
//#define dbus_new0(type, count) ((type*)dbus_malloc0 (sizeof (type) * (count)));

procedure dbus_free_string_array(str_array: PPChar); cdecl; external LibDBus;

type
  DBusFreeFunction = procedure(memory: Pointer);

procedure dbus_shutdown; cdecl; external LibDBus;

