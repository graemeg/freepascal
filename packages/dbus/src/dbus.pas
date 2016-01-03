{
  Pascal translation of the dbus headers
  
  Based on dbus version 0.92
}
{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus.h  Convenience header including all other headers
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
unit dbus;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

{$minenumsize 4}

{$packrecords c}

{ FPC 2.0.2 compatibility code }
{$ifdef win32}
  {$define windows}
{$endif}

{ Delphi compatibility code }
{$ifndef fpc}
  {$define windows}
{$endif}

interface

uses ctypes;

{ D-Bus hasn't reached 1.0 and is subject to protocol and API churn.
  See the README for a full explanation. }

const
{$ifdef unix}
  LibDBus = 'libdbus-1';
{$endif}
{$ifdef windows}
  LibDBus = 'libdbus.dll';
{$endif}

{$include dbus-macros.inc}
{$include dbus-arch-deps.inc}
{$include dbus-types.inc}
{$include dbus-errors.inc}
{$include dbus-address.inc}
{$include dbus-message.inc}
{$include dbus-shared.inc}
{$include dbus-connection.inc}
{$include dbus-bus.inc}

{$include dbus-pending-call.inc}
{$include dbus-protocol.inc}
{$include dbus-server.inc}

{$include dbus-signature.inc}
{$include dbus-threads.inc}

{
 * @defgroup DBus D-Bus low-level public API
 * @brief The low-level public API of the D-Bus library
 *
 * libdbus provides a low-level API intended primarily for use by
 * bindings to specific object systems and languages.  D-Bus is most
 * convenient when used with the GLib bindings, Python bindings, Qt
 * bindings, Mono bindings, and so forth.  This low-level API has a
 * lot of complexity useful only for bindings.
 * 
 }
implementation

end.
