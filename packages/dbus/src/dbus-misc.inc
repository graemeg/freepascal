{ dbus-misc.h  A few assorted public functions that don't fit elsewhere
 *
 * Copyright (C) 2006 Red Hat, Inc.
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA
=======
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
>>>>>>> graemeg/cpstrnew
=======
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
>>>>>>> graemeg/cpstrnew
=======
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
>>>>>>> graemeg/cpstrnew
=======
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
>>>>>>> origin/cpstrnew
 *
 }


function dbus_get_local_machine_id: PChar; cdecl; external LibDBus;

procedure dbus_get_version(major_version_p, minor_version_p,
  micro_version_p: Pcint); cdecl; external LibDBus;

