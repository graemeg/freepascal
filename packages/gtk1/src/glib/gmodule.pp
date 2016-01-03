{

   GMODULE - GLIB wrapper code for dynamic module loading
   Copyright (C) 1998 Tim Janik

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the
<<<<<<< HEAD
<<<<<<< HEAD
   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
unit gmodule;
interface

{$mode objfpc}

{ Always use smartlinking for win32, this solves some undefined functions
  in the development gtk versions which change often (PFV) }
{$ifdef win32}
  {$ifndef NO_SMART_LINK}
    {$smartlink on}
  {$endif}
{$endif}

uses
  glib;

{$ifdef win32}
  const
    gmoduledll='libgmodule-2.0-0';
  {$define gtkwin}

  {$packrecords C}
{$else}
  {$ifdef os2}
    const
      gmoduledll='gmodule';
    {$define gtkos2}

    {$packrecords C}
  {$else}
    const
      gmoduledll='gmodule';

    {$packrecords C}
  {$endif}
{$endif}

{$ifndef gtkos2}
    var
       g_log_domain_gmodule : Pgchar;external gmoduledll name 'g_log_domain_gmodule';
{$endif}

    type
       PGModule=pointer;

       TGModuleFlags = longint;
    const
       G_MODULE_BIND_LAZY = 1 shl 0;
       G_MODULE_BIND_MASK = 1;

    type
       TGModuleCheckInit = function (module:PGModule):Pgchar;cdecl;
       TGModuleUnload = procedure (module:PGModule);cdecl;

function  g_module_supported:gboolean;cdecl;external gmoduledll name 'g_module_supported';
function  g_module_open(file_name:Pgchar; flags:TGModuleFlags):PGModule;cdecl;external gmoduledll name 'g_module_open';
function  g_module_close(module:PGModule):gboolean;cdecl;external gmoduledll name 'g_module_close';
procedure g_module_make_resident(module:PGModule);cdecl;external gmoduledll name 'g_module_make_resident';
function  g_module_error:Pgchar;cdecl;external gmoduledll name 'g_module_error';
function  g_module_symbol(module:PGModule; symbol_name:Pgchar; symbol:Pgpointer):gboolean;cdecl;external gmoduledll name 'g_module_symbol';
function  g_module_name(module:PGModule):Pgchar;cdecl;external gmoduledll name 'g_module_name';
function  g_module_build_path(directory:Pgchar; module_name:Pgchar):Pgchar;cdecl;external gmoduledll name 'g_module_build_path';


implementation

end.
