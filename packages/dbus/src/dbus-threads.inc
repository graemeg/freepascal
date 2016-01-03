{ -*- mode: C; c-file-style: "gnu" -*- }
{ dbus-threads.h  D-Bus threads handling
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
#include <dbus/dbus-types.h>}

type
  DBusMutex = record end;
  PDBusMutex = ^DBusMutex;

  DBusCondVar = record end;
  PDBusCondVar = ^DBusCondVar;
  
  DBusMutexNewFunction = function (): DBusMutex;
  DBusMutexFreeFunction = procedure (mutex: PDBusMutex);
  DBusMutexLockFunction = function (mutex: PDBusMutex): dbus_bool_t;
  DBusMutexUnlockFunction = function (mutex: PDBusMutex): dbus_bool_t;

  DBusCondVarNewFunction = function (): PDBusCondVar;
  DBusCondVarFreeFunction = procedure (cond: PDBusCondVar);
  DBusCondVarWaitFunction = procedure (cond: PDBusCondVar; mutex: PDBusMutex);
  DBusCondVarWaitTimeoutFunction = procedure (cond: PDBusCondVar; mutex: PDBusMutex;
   timeout_milliseconds: cint);
  DBusCondVarWakeOneFunction = procedure (cond: PDBusCondVar);
  DBusCondVarWakeAllFunction = procedure (cond: PDBusCondVar);

  DBusThreadFunctionsMask =
  (
   DBUS_THREAD_FUNCTIONS_MUTEX_NEW_MASK      = 1 shl 0,
   DBUS_THREAD_FUNCTIONS_MUTEX_FREE_MASK     = 1 shl 1,
   DBUS_THREAD_FUNCTIONS_MUTEX_LOCK_MASK     = 1 shl 2,
   DBUS_THREAD_FUNCTIONS_MUTEX_UNLOCK_MASK   = 1 shl 3,
   DBUS_THREAD_FUNCTIONS_CONDVAR_NEW_MASK    = 1 shl 4,
   DBUS_THREAD_FUNCTIONS_CONDVAR_FREE_MASK   = 1 shl 5,
   DBUS_THREAD_FUNCTIONS_CONDVAR_WAIT_MASK   = 1 shl 6,
   DBUS_THREAD_FUNCTIONS_CONDVAR_WAIT_TIMEOUT_MASK   = 1 shl 7,
   DBUS_THREAD_FUNCTIONS_CONDVAR_WAKE_ONE_MASK = 1 shl 8,
   DBUS_THREAD_FUNCTIONS_CONDVAR_WAKE_ALL_MASK = 1 shl 9,

   DBUS_THREAD_FUNCTIONS_ALL_MASK     = (1 shl 10) - 1
  );

{
 * Functions that must be implemented to make the D-Bus
 * library thread-aware. 
 }
  DBusThreadFunctions = record
   mask: cuint; {< Mask indicating which functions are present. }

   mutex_new: DBusMutexNewFunction; {< Function to create a mutex }
   mutex_free: DBusMutexFreeFunction; {< Function to free a mutex }
   mutex_lock: DBusMutexLockFunction; {< Function to lock a mutex }
   mutex_unlock: DBusMutexUnlockFunction; {< Function to unlock a mutex }

   condvar_new: DBusCondVarNewFunction; {< Function to create a condition variable }
   condvar_free: DBusCondVarFreeFunction; {< Function to free a condition variable }
   condvar_wait: DBusCondVarWaitFunction; {< Function to wait on a condition }
   condvar_wait_timeout: DBusCondVarWaitTimeoutFunction; {< Function to wait on a condition with a timeout }
   condvar_wake_one: DBusCondVarWakeOneFunction; {< Function to wake one thread waiting on the condition }
   condvar_wake_all: DBusCondVarWakeAllFunction; {< Function to wake all threads waiting on the condition }
  
   padding1: procedure; {< Reserved for future expansion }
   padding2: procedure; {< Reserved for future expansion }
   padding3: procedure; {< Reserved for future expansion }
   padding4: procedure; {< Reserved for future expansion }
   padding5: procedure; {< Reserved for future expansion }
   padding6: procedure; {< Reserved for future expansion }
   padding7: procedure; {< Reserved for future expansion }
   padding8: procedure; {< Reserved for future expansion }
  end;
  
  PDBusThreadFunctions = ^DBusThreadFunctions;

function dbus_threads_init(const functions: PDBusThreadFunctions): dbus_bool_t; cdecl; external LibDBus;

