{
  Contains the MySQL_com functions calls

  Call InitialiseMysql4_com before using any of the calls, and call ReleaseMysql4_com
  when finished.
}
unit mysql4_comdyn;

{$mode objfpc}{$H+}
{$MACRO on}

interface

uses ctypes,my4_sys,dynlibs, sysutils;

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
    Mysqllib = 'libmysqlclient.'+sharedsuffix;
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    Mysqllib = 'libmysql.dll';
{$ENDIF}

{$PACKRECORDS C}

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


var
  my_net_init : function (net:PNET; vio:PVio):longint;extdecl;
  net_end : procedure (net:PNET);extdecl;
  net_clear : procedure (net:PNET);extdecl;
  net_flush : function (net:PNET):longint;extdecl;
  my_net_write : function (net:PNET; packet:Pchar; len:dword):longint;extdecl;
  net_write_command : function (net:PNET; command:byte; packet:Pchar; len:dword):longint;extdecl;
  net_real_write : function (net:PNET; packet:Pchar; len:dword):longint;extdecl;
  my_net_read : function (net:PNET):dword;extdecl;
{ The following function is not meant for normal usage  }
{
struct sockaddr;
int my_connect(my_socket s, const struct sockaddr  name, unsigned int namelen,
unsigned int timeout);
}
  randominit : procedure (_para1:Prand_struct; seed1:dword; seed2:dword);extdecl;
  rnd : function (_para1:Prand_struct):double;extdecl;
  make_scrambled_password : procedure (_to:Pchar; password:Pchar);extdecl;
  get_salt_from_password : procedure (res:Pdword; password:Pchar);extdecl;
  make_password_from_salt : procedure (_to:Pchar; hash_res:Pdword);extdecl;
  scramble : function (_to:Pchar; message:Pchar; password:Pchar; old_ver:my_bool):Pchar;extdecl;
  check_scramble : function (_para1:Pchar; message:Pchar; salt:Pdword; old_ver:my_bool):my_bool;extdecl;
  get_tty_password : function (opt_message:Pchar):Pchar;extdecl;
  hash_password : procedure (result:Pdword; password:Pchar);extdecl;
  my_init : procedure;extdecl;
  load_defaults : procedure (conf_file:Pchar; groups:PPchar; argc:Plongint; argv:PPPchar);extdecl;
  my_thread_init : function : my_bool;extdecl;
  my_thread_end : procedure ;extdecl;

function packet_error : longint;
  { For net_store_length  }
  { was #define dname def_expr }
function NULL_LENGTH : dword;


Procedure InitialiseMysql4_com;
Procedure ReleaseMysql4_com;

var Mysql4_comLibraryHandle : TLibHandle;

implementation

var RefCount : integer;

Procedure InitialiseMysql4_com;

begin
  inc(RefCount);
  if RefCount = 1 then
    begin
    Mysql4_comLibraryHandle := loadlibrary(Mysqllib);
    if Mysql4_comLibraryHandle = nilhandle then
      begin
      RefCount := 0;
      Raise EInOutError.Create('Can not load MySQL client. Is it installed? ('+Mysqllib+')');
      end;

    pointer(my_net_init) := GetProcedureAddress(Mysql4_comLibraryHandle,'my_net_init');
    pointer(net_end) := GetProcedureAddress(Mysql4_comLibraryHandle,'net_end');
    pointer(net_clear) := GetProcedureAddress(Mysql4_comLibraryHandle,'net_clear');
    pointer(net_flush) := GetProcedureAddress(Mysql4_comLibraryHandle,'net_flush');
    pointer(my_net_write) := GetProcedureAddress(Mysql4_comLibraryHandle,'my_net_write');
    pointer(net_write_command) := GetProcedureAddress(Mysql4_comLibraryHandle,'net_write_command');
    pointer(net_real_write) := GetProcedureAddress(Mysql4_comLibraryHandle,'net_real_write');
    pointer(my_net_read) := GetProcedureAddress(Mysql4_comLibraryHandle,'my_net_read');
    pointer(randominit) := GetProcedureAddress(Mysql4_comLibraryHandle,'randominit');
    pointer(rnd) := GetProcedureAddress(Mysql4_comLibraryHandle,'rnd');
    pointer(make_scrambled_password) := GetProcedureAddress(Mysql4_comLibraryHandle,'make_scrambled_password');
    pointer(get_salt_from_password) := GetProcedureAddress(Mysql4_comLibraryHandle,'get_salt_from_password');
    pointer(make_password_from_salt) := GetProcedureAddress(Mysql4_comLibraryHandle,'make_password_from_salt');
    pointer(scramble) := GetProcedureAddress(Mysql4_comLibraryHandle,'scramble');
    pointer(check_scramble) := GetProcedureAddress(Mysql4_comLibraryHandle,'check_scramble');
    pointer(get_tty_password) := GetProcedureAddress(Mysql4_comLibraryHandle,'get_tty_password');
    pointer(hash_password) := GetProcedureAddress(Mysql4_comLibraryHandle,'hash_password');
    pointer(my_init) := GetProcedureAddress(Mysql4_comLibraryHandle,'my_init');
    pointer(load_defaults) := GetProcedureAddress(Mysql4_comLibraryHandle,'load_defaults');
    pointer(my_thread_init) := GetProcedureAddress(Mysql4_comLibraryHandle,'my_thread_init');
    pointer(my_thread_end) := GetProcedureAddress(Mysql4_comLibraryHandle,'my_thread_end');
    end;
end;

Procedure ReleaseMysql4_com;

begin
  if RefCount > 0 then dec(RefCount);
  if RefCount = 0 then
    begin
    if not UnloadLibrary(Mysql4_comLibraryHandle) then inc(RefCount);
    end;
end;

// Next function also defined in mysql4_com
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

// Next function also defined in mysql4_com
  { was #define dname def_expr }
  function NULL_LENGTH : dword;
      begin
         NULL_LENGTH:=dword( not (0));
      end;


end.
