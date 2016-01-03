{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-message.h DBusMessage object
 *
 * Copyright (C) 2002, 2003, 2005 Red Hat Inc.
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
#include <dbus/dbus-types.h>
#include <dbus/dbus-arch-deps.h>}
{$include dbus-memory.inc}
{#include <dbus/dbus-errors.h>
#include <stdarg.h>}

type
  DBusMessage = record end;
  PDBusMessage = ^DBusMessage;

  PDBusMessageIter = ^DBusMessageIter;

{
 * DBusMessageIter struct; contains no public fields 
 }
  DBusMessageIter = record
   dummy1: Pointer;        {< Don't use this }
   dummy2: Pointer;        {< Don't use this }
   dummy3: dbus_uint32_t;  {< Don't use this }
   dummy4: cint;           {< Don't use this }
   dummy5: cint;           {< Don't use this }
   dummy6: cint;           {< Don't use this }
   dummy7: cint;           {< Don't use this }
   dummy8: cint;           {< Don't use this }
   dummy9: cint;           {< Don't use this }
   dummy10: cint;          {< Don't use this }
   dummy11: cint;          {< Don't use this }
   pad1: cint;             {< Don't use this }
   pad2: cint;             {< Don't use this }
   pad3: Pointer           {< Don't use this }
  end;

function dbus_message_new(message_type: Integer): PDBusMessage; cdecl; external LibDBus;
function dbus_message_new_method_call(
 const bus_name, path, interface_, method: PChar): PDBusMessage; cdecl; external LibDBus;
function dbus_message_new_method_return(method_call: PDBusMessage): PDBusMessage; cdecl; external LibDBus;
function dbus_message_new_signal(const path, interface_, name: PChar): PDBusMessage; cdecl; external LibDBus;
function dbus_message_new_error(reply_to: PDBusMessage;
 const error_name, error_message: PChar): PDBusMessage; cdecl; external LibDBus;
function dbus_message_new_error_printf(reply_to: PDBusMessage;
 const error_name, error_format: PChar;
 others: array of const): PDBusMessage; cdecl; external LibDBus;

function dbus_message_copy(const message_: PDBusMessage): PDBusMessage; cdecl; external LibDBus;

function dbus_message_ref(message_: PDBusMessage): PDBusMessage; cdecl; external LibDBus;
procedure dbus_message_unref(message_: PDBusMessage); cdecl; external LibDBus;
function dbus_message_get_type(message_: PDBusMessage): cint; cdecl; external LibDBus;
function dbus_message_set_path(message_: PDBusMessage;
 const object_path: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_path(message_: PDBusMessage): PChar; cdecl; external LibDBus;
function dbus_message_has_path(message_: PDBusMessage;
 const object_path: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_set_interface(message_: PDBusMessage;
 const interface_: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_interface(message_: PDBusMessage): PChar; cdecl; external LibDBus;
function dbus_message_has_interface(message_: PDBusMessage;
 const interface_: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_set_member(message_: PDBusMessage;
 const member: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_member(message_: PDBusMessage): PChar; cdecl; external LibDBus;
function dbus_message_has_member(message_: PDBusMessage;
 const member: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_set_error_name(message_: PDBusMessage;
 const name: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_error_name(message_: PDBusMessage): PChar; cdecl; external LibDBus;
function dbus_message_set_destination(message_: PDBusMessage;
 const destination: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_destination(message_: PDBusMessage): PChar; cdecl; external LibDBus;
function dbus_message_set_sender(message_: PDBusMessage;
 const sender: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_sender(message_: PDBusMessage): PChar; cdecl; external LibDBus;
function dbus_message_get_signature(message_: PDBusMessage): PChar; cdecl; external LibDBus;
procedure dbus_message_set_no_reply(message_: PDBusMessage;
 no_reply: dbus_bool_t); cdecl; external LibDBus;
function dbus_message_get_no_reply(message_: PDBusMessage): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_is_method_call(message_: PDBusMessage;
 const interface_, method: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_is_signal(message_: PDBusMessage;
 const interface_, signal_name: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_is_error(message_: PDBusMessage;
 const error_name: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_has_destination(message_: PDBusMessage;
 const bus_name: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_has_sender(message_: PDBusMessage;
 const unique_bus_name: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_has_signature(message_: PDBusMessage;
 const signature: PChar): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_serial(message_: PDBusMessage): dbus_uint32_t; cdecl; external LibDBus;
function dbus_message_set_reply_serial(message_: PDBusMessage;
 reply_serial: dbus_uint32_t): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_reply_serial(message_: PDBusMessage): dbus_uint32_t; cdecl; external LibDBus;

procedure dbus_message_set_auto_start(message_: PDBusMessage; auto_start: dbus_bool_t); cdecl; external LibDBus;
function dbus_message_get_auto_start(message_: PDBusMessage): dbus_bool_t; cdecl; external LibDBus;

function dbus_message_get_path_decomposed(message_: PDBusMessage; path: PPPChar): dbus_bool_t; cdecl; external LibDBus;

function dbus_message_append_args(message_: PDBusMessage;
 first_arg_type: cint; others: array of const): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_append_args_valist(message_: PDBusMessage;
 first_arg_type: cint; var_args: va_list): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_args(message_: PDBusMessage;
 error: PDBusError; first_arg_type: cint; others: array of const): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_args_valist(message_: PDBusMessage;
 error: PDBusError; first_arg_type: cint; var_args: va_list): dbus_bool_t; cdecl; external LibDBus;


function dbus_message_iter_init(message_: PDBusMessage; iter: PDBusMessageIter): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_iter_has_next(iter: PDBusMessageIter): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_iter_next(iter: PDBusMessageIter): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_iter_get_signature(iter: PDBusMessageIter): PChar; cdecl; external LibDBus;
function dbus_message_iter_get_arg_type(iter: PDBusMessageIter): cint; cdecl; external LibDBus;
function dbus_message_iter_get_element_type(iter: PDBusMessageIter): cint; cdecl; external LibDBus;
procedure dbus_message_iter_recurse(iter, sub: PDBusMessageIter); cdecl; external LibDBus;
procedure dbus_message_iter_get_basic(iter: PDBusMessageIter; value: Pointer); cdecl; external LibDBus;
function dbus_message_iter_get_array_len(iter: PDBusMessageIter): cint; cdecl; external LibDBus;
procedure dbus_message_iter_get_fixed_array(iter: PDBusMessageIter; value: Pointer; n_elements: Pcint); cdecl; external LibDBus;


procedure dbus_message_iter_init_append(message_: PDBusMessage; iter: PDBusMessageIter); cdecl; external LibDBus;
function dbus_message_iter_append_basic(iter: PDBusMessageIter;
 type_: cint; const value: Pointer): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_iter_append_fixed_array(iter: PDBusMessageIter;
 element_type: cint; const value: Pointer; n_elements: cint): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_iter_open_container(iter: PDBusMessageIter;
 type_: cint; const contained_signature: PChar; sub: PDBusMessageIter): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_iter_close_container(iter, sub: PDBusMessageIter): dbus_bool_t; cdecl; external LibDBus;


function dbus_set_error_from_message(error: PDBusError; message_: PDBusMessage): dbus_bool_t; cdecl; external LibDBus;


function dbus_message_allocate_data_slot(slot_p: Pdbus_int32_t): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_message_free_data_slot(slot_p: Pdbus_int32_t); cdecl; external LibDBus;
function dbus_message_set_data(message_: PDBusMessage;
 slot: dbus_int32_t; data: Pointer; free_data_func: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_message_get_data(message_: PDBusMessage; slot: dbus_int32_t): Pointer; cdecl; external LibDBus;

function dbus_message_type_from_string(const type_str: PChar): cint; cdecl; external LibDBus;
function dbus_message_type_to_string(type_: cint): PChar; cdecl; external LibDBus;

