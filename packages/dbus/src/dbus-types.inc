{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-types.h  types such as dbus_bool_t
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

//#include <stddef.h>
//#include <dbus/dbus-arch-deps.h>

type
  dbus_unichar_t = dbus_uint32_t;

{ boolean size must be fixed at 4 bytes due to wire protocol! }
  dbus_bool_t = dbus_uint32_t;

  PPPChar = ^PPChar;
  
  va_list = Pointer;

{ Normally docs are in .c files, but there isn't a .c file for this. }
{
 * @defgroup DBusTypes Basic types
 * @ingroup  DBus
 * @brief dbus_bool_t, dbus_int32_t, etc.
 *
 * Typedefs for common primitive types.
 *
 }

{
 * @typedef dbus_bool_t
 *
 * A boolean, valid values are #TRUE and #FALSE.
 }

{
 * @typedef dbus_uint32_t
 *
 * A 32-bit unsigned integer on all platforms.
 }

{
 * @typedef dbus_int32_t
 *
 * A 32-bit signed integer on all platforms.
 }

{
 * @typedef dbus_uint16_t
 *
 * A 16-bit unsigned integer on all platforms.
 }

{
 * @typedef dbus_int16_t
 *
 * A 16-bit signed integer on all platforms.
 }


{
 * @typedef dbus_uint64_t
 *
 * A 64-bit unsigned integer on all platforms that support it.
 * If supported, #DBUS_HAVE_INT64 will be defined.
 *
 * C99 requires a 64-bit type and most likely all interesting
 * compilers support one. GLib for example flat-out requires
 * a 64-bit type.
 }

{
 * @typedef dbus_int64_t
 *
 * A 64-bit signed integer on all platforms that support it.
 * If supported, #DBUS_HAVE_INT64 will be defined.
 *
 * C99 requires a 64-bit type and most likely all interesting
 * compilers support one. GLib for example flat-out requires
 * a 64-bit type.
 }

{
 * @def DBUS_INT64_CONSTANT
 *
 * Declare a 64-bit signed integer constant. The macro
 * adds the necessary "LL" or whatever after the integer,
 * giving a literal such as "325145246765LL"
 }

{
 * @def DBUS_UINT64_CONSTANT
 *
 * Declare a 64-bit unsigned integer constant. The macro
 * adds the necessary "ULL" or whatever after the integer,
 * giving a literal such as "325145246765ULL"
 }

