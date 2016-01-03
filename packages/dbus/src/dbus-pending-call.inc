{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-pending-call.h Object representing a call in progress.
 *
 * Copyright (C) 2002, 2003 Red Hat Inc.
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
#include <dbus/dbus-connection.h>}

function dbus_pending_call_ref(pending: PDBusPendingCall): PDBusPendingCall; cdecl; external LibDBus;
procedure dbus_pending_call_unref(pending: PDBusPendingCall); cdecl; external LibDBus;
function dbus_pending_call_set_notify(pending: PDBusPendingCall;
 function_: DBusPendingCallNotifyFunction;
 user_data: Pointer; free_user_data: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_pending_call_cancel(pending: PDBusPendingCall); cdecl; external LibDBus;
function dbus_pending_call_get_completed(pending: PDBusPendingCall): dbus_bool_t; cdecl; external LibDBus;
function dbus_pending_call_steal_reply(pending: PDBusPendingCall): PDBusMessage; cdecl; external LibDBus;
procedure dbus_pending_call_block(pending: PDBusPendingCall); cdecl; external LibDBus;

function dbus_pending_call_allocate_data_slot(slot_p: Pdbus_int32_t): dbus_bool_t; cdecl; external LibDBus;
procedure dbus_pending_call_free_data_slot(slot_p: Pdbus_int32_t); cdecl; external LibDBus;
function dbus_pending_call_set_data(pending: PDBusPendingCall;
 slot: dbus_int32_t; data: Pointer; free_data_func: DBusFreeFunction): dbus_bool_t; cdecl; external LibDBus;
function dbus_pending_call_get_data(pending: PDBusPendingCall; slot: dbus_int32_t): Pointer; cdecl; external LibDBus;

