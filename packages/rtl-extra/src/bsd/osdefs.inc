{
    Copyright (c) 2000-2002 by Marco van de Voort

    Target dependent defines used when compileing the baseunix unit

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301, USA.

 ****************************************************************************}

{$define usedomain}             // Allow uname with "domain" entry.
                                // (which is a GNU extension)
{$define hassysctl}             // Use sysctl unit

{$ifdef darwin}
  {$define FPC_USE_LIBC}
{$endif}
{$ifdef openbsd}
  {$ifndef FPC_USE_SYSCALL}
    {$define FPC_USE_LIBC}
  {$endif}
{$endif}

