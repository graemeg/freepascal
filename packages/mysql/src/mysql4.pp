unit mysql4;
interface

uses ctypes,my4_sys,mysql4_com,dynlibs;

{
  Automatically converted by H2Pas 0.99.15 from mysql.ph
  The following command line parameters were used:
    -p
    -D
    -l
    mysqlclient
    mysql.ph
}


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
<<<<<<< HEAD
<<<<<<< HEAD
     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA. }
=======
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  }
>>>>>>> graemeg/fixes_2_2
=======
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  }
>>>>>>> origin/fixes_2_2
{$mode objfpc}{$H+}
{$MACRO on}

{$PACKRECORDS C}

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

{$i mysql4types.inc}

function mysql_server_init(argc:longint; argv:PPchar; groups:PPchar):longint;extdecl;external External_library name 'mysql_server_init';
procedure mysql_server_end;extdecl;external External_library name 'mysql_server_end';
function mysql_thread_init:my_bool;extdecl;external External_library name 'mysql_thread_init';
procedure mysql_thread_end;extdecl;external External_library name 'mysql_thread_end';
function mysql_num_rows(res:PMYSQL_RES):my_ulonglong;extdecl;external External_library name 'mysql_num_rows';
function mysql_num_fields(res:PMYSQL_RES):dword;extdecl;external External_library name 'mysql_num_fields';
function mysql_eof(res:PMYSQL_RES):my_bool;extdecl;external External_library name 'mysql_eof';
function mysql_fetch_field_direct(res:PMYSQL_RES; fieldnr:dword):PMYSQL_FIELD;extdecl;external External_library name 'mysql_fetch_field_direct';
function mysql_fetch_fields(res:PMYSQL_RES):PMYSQL_FIELD;extdecl;external External_library name 'mysql_fetch_fields';
function mysql_row_tell(res:PMYSQL_RES):PMYSQL_ROWS;extdecl;external External_library name 'mysql_row_tell';
function mysql_field_tell(res:PMYSQL_RES):dword;extdecl;external External_library name 'mysql_field_tell';
function mysql_field_count(mysql:PMYSQL):dword;extdecl;external External_library name 'mysql_field_count';
function mysql_affected_rows(mysql:PMYSQL):my_ulonglong;extdecl;external External_library name 'mysql_affected_rows';
function mysql_insert_id(mysql:PMYSQL):my_ulonglong;extdecl;external External_library name 'mysql_insert_id';
function mysql_errno(mysql:PMYSQL):dword;extdecl;external External_library name 'mysql_errno';
function mysql_error(mysql:PMYSQL):Pchar;extdecl;external External_library name 'mysql_error';
function mysql_info(mysql:PMYSQL):Pchar;extdecl;external External_library name 'mysql_info';
function mysql_thread_id(mysql:PMYSQL):dword;extdecl;external External_library name 'mysql_thread_id';
function mysql_character_set_name(mysql:PMYSQL):Pchar;extdecl;external External_library name 'mysql_character_set_name';
function mysql_init(mysql:PMYSQL):PMYSQL;extdecl;external External_library name 'mysql_init';
function mysql_ssl_set(mysql:PMYSQL; key:Pchar; cert:Pchar; ca:Pchar; capath:Pchar;
           cipher:Pchar):longint;extdecl;external External_library name 'mysql_ssl_set';
function mysql_ssl_clear(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_ssl_clear';
function mysql_change_user(mysql:PMYSQL; user:Pchar; passwd:Pchar; db:Pchar):my_bool;extdecl;external External_library name 'mysql_change_user';
function mysql_real_connect(mysql:PMYSQL; host:Pchar; user:Pchar; passwd:Pchar; db:Pchar;
           port:dword; unix_socket:Pchar; clientflag:dword):PMYSQL;extdecl;external External_library name 'mysql_real_connect';
procedure mysql_close(sock:PMYSQL);extdecl;external External_library name 'mysql_close';
function mysql_select_db(mysql:PMYSQL; db:Pchar):longint;extdecl;external External_library name 'mysql_select_db';
function mysql_query(mysql:PMYSQL; q:Pchar):longint;extdecl;external External_library name 'mysql_query';
function mysql_send_query(mysql:PMYSQL; q:Pchar; length:dword):longint;extdecl;external External_library name 'mysql_send_query';
function mysql_read_query_result(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_read_query_result';
function mysql_real_query(mysql:PMYSQL; q:Pchar; length:dword):longint;extdecl;external External_library name 'mysql_real_query';
function mysql_master_query(mysql:PMYSQL; q:Pchar; length:dword):longint;extdecl;external External_library name 'mysql_master_query';
function mysql_master_send_query(mysql:PMYSQL; q:Pchar; length:dword):longint;extdecl;external External_library name 'mysql_master_send_query';
function mysql_slave_query(mysql:PMYSQL; q:Pchar; length:dword):longint;extdecl;external External_library name 'mysql_slave_query';
function mysql_slave_send_query(mysql:PMYSQL; q:Pchar; length:dword):longint;extdecl;external External_library name 'mysql_slave_send_query';

procedure mysql_enable_rpl_parse(mysql:PMYSQL);extdecl;external External_library name 'mysql_enable_rpl_parse';
procedure mysql_disable_rpl_parse(mysql:PMYSQL);extdecl;external External_library name 'mysql_disable_rpl_parse';
function mysql_rpl_parse_enabled(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_rpl_parse_enabled';
procedure mysql_enable_reads_from_master(mysql:PMYSQL);extdecl;external External_library name 'mysql_enable_reads_from_master';
procedure mysql_disable_reads_from_master(mysql:PMYSQL);extdecl;external External_library name 'mysql_disable_reads_from_master';
function mysql_reads_from_master_enabled(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_reads_from_master_enabled';
(* error
enum mysql_rpl_type      mysql_rpl_query_type(const char* q, int len);
in declaration at line 291 *)

function mysql_rpl_probe(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_rpl_probe';
function mysql_set_master(mysql:PMYSQL; host:Pchar; port:dword; user:Pchar; passwd:Pchar):longint;extdecl;external External_library name 'mysql_set_master';
function mysql_add_slave(mysql:PMYSQL; host:Pchar; port:dword; user:Pchar; passwd:Pchar):longint;extdecl;external External_library name 'mysql_add_slave';
function mysql_shutdown(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_shutdown';
function mysql_dump_debug_info(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_dump_debug_info';
function mysql_refresh(mysql:PMYSQL; refresh_options:dword):longint;extdecl;external External_library name 'mysql_refresh';
function mysql_kill(mysql:PMYSQL; pid:dword):longint;extdecl;external External_library name 'mysql_kill';
function mysql_ping(mysql:PMYSQL):longint;extdecl;external External_library name 'mysql_ping';
function mysql_stat(mysql:PMYSQL):Pchar;extdecl;external External_library name 'mysql_stat';
function mysql_get_server_info(mysql:PMYSQL):Pchar;extdecl;external External_library name 'mysql_get_server_info';
function mysql_get_client_info:Pchar;extdecl;external External_library name 'mysql_get_client_info';
function mysql_get_host_info(mysql:PMYSQL):Pchar;extdecl;external External_library name 'mysql_get_host_info';
function mysql_get_proto_info(mysql:PMYSQL):dword;extdecl;external External_library name 'mysql_get_proto_info';
function mysql_list_dbs(mysql:PMYSQL; wild:Pchar):PMYSQL_RES;extdecl;external External_library name 'mysql_list_dbs';
function mysql_list_tables(mysql:PMYSQL; wild:Pchar):PMYSQL_RES;extdecl;external External_library name 'mysql_list_tables';
function mysql_list_fields(mysql:PMYSQL; table:Pchar; wild:Pchar):PMYSQL_RES;extdecl;external External_library name 'mysql_list_fields';
function mysql_list_processes(mysql:PMYSQL):PMYSQL_RES;extdecl;external External_library name 'mysql_list_processes';
function mysql_store_result(mysql:PMYSQL):PMYSQL_RES;extdecl;external External_library name 'mysql_store_result';
function mysql_use_result(mysql:PMYSQL):PMYSQL_RES;extdecl;external External_library name 'mysql_use_result';
function mysql_options(mysql:PMYSQL; option:mysql_option; arg:Pchar):longint;extdecl;external External_library name 'mysql_options';
procedure mysql_free_result(result:PMYSQL_RES);extdecl;external External_library name 'mysql_free_result';
procedure mysql_data_seek(result:PMYSQL_RES; offset:my_ulonglong);extdecl;external External_library name 'mysql_data_seek';
function mysql_row_seek(result:PMYSQL_RES; _para2:MYSQL_ROW_OFFSET):MYSQL_ROW_OFFSET;extdecl;external External_library name 'mysql_row_seek';
function mysql_field_seek(result:PMYSQL_RES; offset:MYSQL_FIELD_OFFSET):MYSQL_FIELD_OFFSET;extdecl;external External_library name 'mysql_field_seek';
function mysql_fetch_row(result:PMYSQL_RES):MYSQL_ROW;extdecl;external External_library name 'mysql_fetch_row';
function mysql_fetch_lengths(result:PMYSQL_RES):Pdword;extdecl;external External_library name 'mysql_fetch_lengths';
function mysql_fetch_field(result:PMYSQL_RES):PMYSQL_FIELD;extdecl;external External_library name 'mysql_fetch_field';
function mysql_escape_string(_to:Pchar; from:Pchar; from_length:dword):dword;extdecl;external External_library name 'mysql_escape_string';
function mysql_real_escape_string(mysql:PMYSQL; _to:Pchar; from:Pchar; length:dword):dword;extdecl;external External_library name 'mysql_real_escape_string';
procedure mysql_debug(debug:Pchar);extdecl;external External_library name 'mysql_debug';

function mysql_odbc_escape_string(mysql:PMYSQL; _to:Pchar; to_length:dword; from:Pchar; from_length:dword;
         param:pointer; extend_buffer: TExdendBuffer):Pchar;extdecl;external External_library name 'mysql_odbc_escape_string';
procedure myodbc_remove_escape(mysql:PMYSQL; name:Pchar);extdecl;external External_library name 'myodbc_remove_escape';
function mysql_thread_safe:dword;extdecl;external External_library name 'mysql_thread_safe';
function mysql_manager_init(con:PMYSQL_MANAGER):PMYSQL_MANAGER;extdecl;external External_library name 'mysql_manager_init';
function mysql_manager_connect(con:PMYSQL_MANAGER; host:Pchar; user:Pchar; passwd:Pchar; port:dword):PMYSQL_MANAGER;extdecl;external External_library name 'mysql_manager_connect';
procedure mysql_manager_close(con:PMYSQL_MANAGER);extdecl;external External_library name 'mysql_manager_close';
function mysql_manager_command(con:PMYSQL_MANAGER; cmd:Pchar; cmd_len:longint):longint;extdecl;external External_library name 'mysql_manager_command';
function mysql_manager_fetch_line(con:PMYSQL_MANAGER; res_buf:Pchar; res_buf_size:longint):longint;extdecl;external External_library name 'mysql_manager_fetch_line';
function simple_command(mysql:PMYSQL; command:enum_server_command; arg:Pchar; length:dword; skipp_check:my_bool):longint;extdecl;external External_library name 'simple_command';
function net_safe_read(mysql:PMYSQL):dword;extdecl;external External_library name 'net_safe_read';

function IS_PRI_KEY(n : longint) : Boolean;
function IS_NOT_NULL(n : longint) :  Boolean;
function IS_BLOB(n : longint) : boolean;
function MYSQL_COUNT_ERROR : longint;
function mysql_reload(mysql : pmysql) : longint;


implementation

{$i mysql4impl.inc}

end.
