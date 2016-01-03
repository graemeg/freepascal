{$mode objfpc}{$H+}
{$MACRO on}

{$PACKRECORDS C}
unit mysql4_com;
interface

uses
  ctypes,dynlibs;

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
    External_library = 'libmysqlclient.'+sharedsuffix;
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    External_library = 'libmysql.dll';
{$ENDIF}

{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$i mysql4_comtypes.inc}

  { Copyright (C) 2000 MySQL AB

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
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  }


  function my_net_init(net:PNET; vio:PVio):longint;extdecl;external External_library name 'my_net_init';

  procedure net_end(net:PNET);extdecl;external External_library name 'net_end';

  procedure net_clear(net:PNET);extdecl;external External_library name 'net_clear';

  function net_flush(net:PNET):longint;extdecl;external External_library name 'net_flush';

(* Const before type ignored *)
  function my_net_write(net:PNET; packet:Pchar; len:dword):longint;extdecl;external External_library name 'my_net_write';

(* Const before type ignored *)
  function net_write_command(net:PNET; command:byte; packet:Pchar; len:dword):longint;extdecl;external External_library name 'net_write_command';

(* Const before type ignored *)
  function net_real_write(net:PNET; packet:Pchar; len:dword):longint;extdecl;external External_library name 'net_real_write';

  function my_net_read(net:PNET):dword;extdecl;external External_library name 'my_net_read';

  { The following function is not meant for normal usage  }
  {
  struct sockaddr;
  int my_connect(my_socket s, const struct sockaddr  name, unsigned int namelen,
               unsigned int timeout);
   }


  procedure randominit(_para1:Prand_struct; seed1:dword; seed2:dword);extdecl;external External_library name 'randominit';

  function rnd(_para1:Prand_struct):double;extdecl;external External_library name 'rnd';

(* Const before type ignored *)
  procedure make_scrambled_password(_to:Pchar; password:Pchar);extdecl;external External_library name 'make_scrambled_password';

(* Const before type ignored *)
  procedure get_salt_from_password(res:Pdword; password:Pchar);extdecl;external External_library name 'get_salt_from_password';

  procedure make_password_from_salt(_to:Pchar; hash_res:Pdword);extdecl;external External_library name 'make_password_from_salt';

(* Const before type ignored *)
(* Const before type ignored *)
  function scramble(_to:Pchar; message:Pchar; password:Pchar; old_ver:my_bool):Pchar;extdecl;external External_library name 'scramble';

(* Const before type ignored *)
(* Const before type ignored *)
  function check_scramble(_para1:Pchar; message:Pchar; salt:Pdword; old_ver:my_bool):my_bool;extdecl;external External_library name 'check_scramble';

  function get_tty_password(opt_message:Pchar):Pchar;extdecl;external External_library name 'get_tty_password';

(* Const before type ignored *)
  procedure hash_password(result:Pdword; password:Pchar);extdecl;external External_library name 'hash_password';

  { Some other useful functions  }
  procedure my_init;extdecl;external External_library name 'my_init';

(* Const before type ignored *)
(* Const before type ignored *)
  procedure load_defaults(conf_file:Pchar; groups:PPchar; argc:Plongint; argv:PPPchar);extdecl;external External_library name 'load_defaults';

  function my_thread_init:my_bool;extdecl;external External_library name 'my_thread_init';

  procedure my_thread_end;extdecl;external External_library name 'my_thread_end';


function packet_error : longint;

  { For net_store_length  }
  { was #define dname def_expr }
  function NULL_LENGTH : dword;


implementation

// Next function also defined in mysql4_comdyn
  { was #define dname def_expr }
  function packet_error : longint;
      { return type might be wrong }
      begin
<<<<<<< HEAD
<<<<<<< HEAD
         packet_error:= not ({dword}(0));
=======
         packet_error:= not (dword(0));
>>>>>>> graemeg/fixes_2_2
=======
         packet_error:= not (dword(0));
>>>>>>> origin/fixes_2_2
      end;

// Next function also defined in mysql4_comdyn
  { was #define dname def_expr }
  function NULL_LENGTH : dword;
      begin
         NULL_LENGTH:=dword( not (0));
      end;


end.
