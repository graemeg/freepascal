{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-arch-deps.h Header with architecture/compiler specific information, installed to libdir
 *
 * Copyright (C) 2003 Red Hat, Inc.
 *
 * Licensed under the Academic Free License version 2.0
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

{ Only <dbus/dbus.h> can be included directly, this file may disappear or change contents. }

//#include <dbus/dbus-macros.h>

const DBUS_HAVE_INT64 = 1;

type
  dbus_int64_t = Int64;

  dbus_uint64_t = Int64;

{#define DBUS_INT64_CONSTANT(val)  (val##LL)
#define DBUS_UINT64_CONSTANT(val) (val##ULL)}

  dbus_int32_t = cint;
  dbus_uint32_t = cuint;

  dbus_int16_t = cshort;
  dbus_uint16_t = cushort;
  
  size_t = Integer;

