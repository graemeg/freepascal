
unit mysql3;

{$undef use_mysql_321} { if undefined, use mysql 3.23 interface }

{
  Import unit for the mysql header files.

  Translated form the original mysql.h by Michael Van Canneyt
  (michael@tfdec1.fys.kuleuven.ac.be)

  updated to mysql version 3.23 header files by Bernhard Steffen
  (bernhard.steffen@gmx.net)
  
  split into mysql/mysqldyn libraries by Bram Kuijvenhoven (Hexis BV, The Netherlands)
  }
{$mode objfpc}{$h+}
{$macro on}
{$r+,i+,o+}

interface

uses mysql3_com, mysql3_version;

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
    Mysqllib = 'mysqlclient';
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    Mysqllib = 'libmysql';
{$ENDIF}

{$ifndef Windows}
{$linklib c}
{$linklib m}
{$linklib mysqlclient}
{$endif}

{$packrecords C}

{$i mysql3types.inc}

Function mysql_num_rows (res : PMYSQL_RES) : my_ulonglong; extdecl; external mysqllib;
Function mysql_num_fields(res : PMYSQL_RES) : Cardinal; extdecl; external mysqllib;
Function mysql_eof(res : PMYSQL_RES) : my_bool; extdecl; external mysqllib;
Function mysql_fetch_field_direct(res : PMYSQL_RES; fieldnr : Cardinal) : PMYSQL_FIELD; extdecl; external mysqllib;
Function mysql_fetch_fields(res : PMYSQL_RES) : PMYSQL_FIELD; extdecl; external mysqllib;
Function mysql_row_tell(res : PMYSQL_RES) : PMYSQL_ROWS; extdecl; external mysqllib;
Function mysql_field_tell(res : PMYSQL_RES) : Cardinal; extdecl; external mysqllib;
Function mysql_affected_rows(mysql : PMYSQL): my_ulonglong; extdecl; external mysqllib;
Function mysql_insert_id(mysql : PMYSQL): my_ulonglong; extdecl; external mysqllib;
Function mysql_errno(mysql : PMYSQL) : Cardinal; extdecl; external mysqllib;
Function mysql_info(mysql : PMYSQL): Pchar; extdecl; external mysqllib;
Function mysql_thread_id(mysql : PMYSQL) : ptruint; extdecl; external mysqllib;
Function mysql_error(mysql : PMYSQL) : pchar; extdecl; external mysqllib;

function mysql_init(mysql: PMYSQL) : PMYSQL;extdecl; external mysqllib name 'mysql_init';
function mysql_connect (mysql : PMYSQL; host,user,passwd: pchar) : PMYSQL;extdecl; external mysqllib name 'mysql_connect';
function mysql_real_connect (mysql : PMYSQL; const host,user,passwd : pchar;
                                   port : cardinal;
                                   unix_socket : pchar;
                                   clientflag : cardinal) : PMYSQL;extdecl; external mysqllib;

function mysql_close(sock : PMYSQL) : longint ;extdecl; external mysqllib name 'mysql_close';
function mysql_select_db(MYSQL : PMYSQL; db : Pchar) : longint;extdecl; external mysqllib name 'mysql_select_db';
function mysql_query(mysql : PMYSQL; q : pchar) : longint;extdecl; external mysqllib name 'mysql_query';
function mysql_real_query(mysql : PMYSQL; q : Pchar; length : longint) : longint;extdecl; external mysqllib name 'mysql_real_query';
function mysql_create_db(mysql : PMYSQL; db : pchar) : longint;extdecl; external mysqllib name 'mysql_create_db';
Function mysql_drop_db(mysql : PMYSQL; DB : Pchar) : longint;extdecl; external mysqllib name 'mysql_drop_db';
Function mysql_shutdown(mysql : PMYSQL) : longint;extdecl; external mysqllib name 'mysql_shutdown';
Function mysql_dump_debug_info(mysql : PMYSQL) : longint;extdecl; external mysqllib name 'mysql_dump_debug_info';
Function mysql_refresh(mysql : PMYSQL; refresh_options : cardinal) : longint;extdecl; external mysqllib name 'mysql_refresh';
Function mysql_kill(mysql : PMYSQL; pid : Cardinal) : longint;extdecl; external mysqllib name 'mysql_kill';
Function mysql_stat(mysql : PMYSQL) : Pchar;extdecl; external mysqllib name 'mysql_stat';
Function mysql_get_server_info(mysql : PMYSQL) : pchar;extdecl; external mysqllib name 'mysql_get_server_info';
Function mysql_get_client_info : pchar;extdecl; external mysqllib;
Function mysql_get_host_info(mysql : PMYSQL) : pchar;extdecl; external mysqllib name 'mysql_get_host_info';
Function mysql_get_proto_info(mysql : PMYSQL) : Cardinal;extdecl; external mysqllib name 'mysql_get_proto_info';
Function mysql_list_dbs(mysql : PMYSQL;wild : Pchar) : PMYSQL_RES;extdecl; external mysqllib name 'mysql_list_dbs';
Function mysql_list_tables(mysql : PMYSQL;Wild : Pchar) : PMYSQL_RES;extdecl; external mysqllib name 'mysql_list_tables';
Function mysql_list_fields(mysql : PMYSQL; table,wild : pchar) : PMYSQL_RES;extdecl; external mysqllib name 'mysql_list_fields';
Function mysql_list_processes(mysql : PMYSQL) : PMYSQL_RES;extdecl; external mysqllib name 'mysql_list_processes';
Function mysql_store_result(mysql : PMYSQL) : PMYSQL_RES;extdecl; external mysqllib name 'mysql_store_result';
Function mysql_use_result(mysql : PMYSQL) : PMYSQL_RES;extdecl; external mysqllib name 'mysql_use_result';
Procedure mysql_free_result(res : PMYSQL_RES);extdecl; external mysqllib name 'mysql_free_result';
Procedure mysql_data_seek(mysql : PMYSQL_RES; offs : cardinal);extdecl; external mysqllib name 'mysql_data_seek';
Function mysql_row_seek(mysql : PMYSQL_RES; Offs: TMYSQL_ROW_OFFSET): TMYSQL_ROW_OFFSET;extdecl; external mysqllib name 'mysql_row_seek';
Function mysql_field_seek(musql : PMYSQL_RES;offs : TMYSQL_FIELD_OFFSET): TMYSQL_FIELD_OFFSET;extdecl; external mysqllib name 'mysql_field_seek';
function mysql_fetch_row(mysql : PMYSQL_RES) : TMYSQL_ROW;extdecl; external mysqllib name 'mysql_fetch_row';
function mysql_fetch_lengths(mysql : PMYSQL_RES) : PCardinal;extdecl; external mysqllib name 'mysql_fetch_lengths';
function mysql_fetch_field(handle : PMYSQL_RES) : PMYSQL_FIELD;extdecl; external mysqllib name 'mysql_fetch_field';
Function mysql_escape_string(escto,escfrom : pchar; length : Cardinal) : cardinal;extdecl; external mysqllib name 'mysql_escape_string';
Procedure mysql_debug(debug : pchar);extdecl; external mysqllib name 'mysql_debug';

implementation

{$i mysql3impl.inc}

end.
