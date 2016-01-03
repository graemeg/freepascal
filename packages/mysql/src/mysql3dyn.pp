{
  Contains the MySQL 3 functions calls

  Call InitialiseMysql3 before using any of the calls, and call ReleaseMysql3
  when finished.
}
unit mysql3dyn;

{
  Adapted from mysql4dyn by Bram Kuijvenhoven (Hexis BV, The Netherlands)
}

{$mode objfpc}{$H+}
{$MACRO on}

interface

uses dynlibs, classes, sysutils, mysql3_comdyn;

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

{$i mysql3types.inc}

type tpcharfunction = function : pchar; extdecl;

var
  mysql_num_rows : function(res : PMYSQL_RES) : my_ulonglong; extdecl;
  mysql_num_fields : function(res : PMYSQL_RES) : Cardinal; extdecl;
  mysql_eof : function(res : PMYSQL_RES) : my_bool; extdecl;
  mysql_fetch_field_direct : function(res : PMYSQL_RES; fieldnr : Cardinal) : PMYSQL_FIELD; extdecl;
  mysql_fetch_fields : function(res : PMYSQL_RES) : PMYSQL_FIELD; extdecl;
  mysql_row_tell : function(res : PMYSQL_RES) : PMYSQL_ROWS; extdecl;
  mysql_field_tell : function(res : PMYSQL_RES) : Cardinal; extdecl;
  mysql_affected_rows : function(mysql : PMYSQL): my_ulonglong; extdecl;
  mysql_insert_id : function(mysql : PMYSQL): my_ulonglong; extdecl;
  mysql_errno : function(mysql : PMYSQL) : Cardinal; extdecl;
  mysql_info : function(mysql : PMYSQL): Pchar; extdecl;
  mysql_thread_id : function(mysql : PMYSQL) : ptruint; extdecl;
  mysql_error : function(mysql : PMYSQL) : pchar; extdecl;

  mysql_init : function(mysql: PMYSQL) : PMYSQL;extdecl;
  mysql_connect : function(mysql : PMYSQL; host,user,passwd: pchar) : PMYSQL;extdecl;
  mysql_real_connect : function(mysql : PMYSQL; const host,user,passwd : pchar;
<<<<<<< HEAD
<<<<<<< HEAD
		                   {$ifndef use_mysql_321} const db : Pchar; {$endif}  // strictly speaking 3.22+ not 3.21+	      		
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                                   port : cardinal;
                                   unix_socket : pchar;
                                   clientflag : cardinal) : PMYSQL;extdecl;
  mysql_close : function(sock : PMYSQL) : longint ;extdecl;
  mysql_select_db : function(MYSQL : PMYSQL; db : Pchar) : longint;extdecl;
  mysql_query : function(mysql : PMYSQL; q : pchar) : longint;extdecl;
  mysql_real_query : function(mysql : PMYSQL; q : Pchar; length : longint) : longint;extdecl;
  mysql_create_db : function(mysql : PMYSQL; db : pchar) : longint;extdecl;
  mysql_drop_db : function(mysql : PMYSQL; DB : Pchar) : longint;extdecl;
  mysql_shutdown : function(mysql : PMYSQL) : longint;extdecl;
  mysql_dump_debug_info : function(mysql : PMYSQL) : longint;extdecl;
  mysql_refresh : function(mysql : PMYSQL; refresh_options : cardinal) : longint;extdecl;
  mysql_kill : function(mysql : PMYSQL; pid : Cardinal) : longint;extdecl;
  mysql_stat : function(mysql : PMYSQL) : Pchar;extdecl;
  mysql_get_server_info : function(mysql : PMYSQL) : pchar;extdecl;
  mysql_get_client_info : function : pchar;extdecl;
  mysql_get_host_info : function(mysql : PMYSQL) : pchar;extdecl;
  mysql_get_proto_info : function(mysql : PMYSQL) : Cardinal;extdecl;
  mysql_list_dbs : function(mysql : PMYSQL;wild : Pchar) : PMYSQL_RES;extdecl;
  mysql_list_tables : function(mysql : PMYSQL;Wild : Pchar) : PMYSQL_RES;extdecl;
  mysql_list_fields : function(mysql : PMYSQL; table,wild : pchar) : PMYSQL_RES;extdecl;
  mysql_list_processes : function(mysql : PMYSQL) : PMYSQL_RES;extdecl;
  mysql_store_result : function(mysql : PMYSQL) : PMYSQL_RES;extdecl;
  mysql_use_result : function(mysql : PMYSQL) : PMYSQL_RES;extdecl;
  mysql_free_result : procedure(res : PMYSQL_RES);extdecl;
  mysql_data_seek : procedure(mysql : PMYSQL_RES; offs : cardinal);extdecl;
  mysql_row_seek : function(mysql : PMYSQL_RES; Offs: TMYSQL_ROW_OFFSET): TMYSQL_ROW_OFFSET;extdecl;
  mysql_field_seek : function(musql : PMYSQL_RES;offs : TMYSQL_FIELD_OFFSET): TMYSQL_FIELD_OFFSET;extdecl;
  mysql_fetch_row : function(mysql : PMYSQL_RES) : TMYSQL_ROW;extdecl;
  mysql_fetch_lengths : function(mysql : PMYSQL_RES) : PCardinal;extdecl;
  mysql_fetch_field : function(handle : PMYSQL_RES) : PMYSQL_FIELD;extdecl;
  mysql_escape_string : function(escto,escfrom : pchar; length : Cardinal) : cardinal;extdecl;
  mysql_debug : procedure(debug : pchar);extdecl;

Procedure InitialiseMysql3;
Procedure ReleaseMysql3;

var Mysql3LibraryHandle : TLibHandle;

implementation

var RefCount : integer;

Procedure InitialiseMysql3;

begin
  inc(RefCount);
  if RefCount = 1 then
    begin
    Mysql3LibraryHandle := loadlibrary(Mysqllib);
    if Mysql3LibraryHandle = nilhandle then
      begin
      RefCount := 0;
      Raise EInOutError.Create('Can not load MySQL client. Is it installed? ('+Mysqllib+')');
      end;
    pointer(mysql_get_client_info) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_get_client_info');

    // To avoid the strangest problems for ppl using other client-libs
    if copy(strpas(mysql_get_client_info()),1,4) <> '3.23' then
      Raise EInOutError.Create('This program can only work with the MySQL client version 3.23.x. Please use the right version of '+Mysqllib+'.');

    pointer(mysql_num_rows) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_num_rows');
    pointer(mysql_num_fields) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_num_fields');
    pointer(mysql_eof) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_eof');
    pointer(mysql_fetch_field_direct) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_fetch_field_direct');
    pointer(mysql_fetch_fields) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_fetch_fields');
    pointer(mysql_row_tell) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_row_tell');
    pointer(mysql_field_tell) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_field_tell');
    pointer(mysql_affected_rows) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_affected_rows');
    pointer(mysql_insert_id) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_insert_id');
    pointer(mysql_errno) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_errno');
    pointer(mysql_info) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_info');
    pointer(mysql_thread_id) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_thread_id');
    pointer(mysql_error) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_error');

    pointer(mysql_init) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_init');
    pointer(mysql_connect) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_connect');
    pointer(mysql_real_connect) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_real_connect');
    pointer(mysql_close) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_close');
    pointer(mysql_select_db) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_select_db');
    pointer(mysql_query) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_query');
    pointer(mysql_real_query) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_real_query');
    pointer(mysql_create_db) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_create_db');
    pointer(mysql_drop_db) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_drop_db');
    pointer(mysql_shutdown) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_shutdown');
    pointer(mysql_dump_debug_info) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_dump_debug_info');
    pointer(mysql_refresh) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_refresh');
    pointer(mysql_kill) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_kill');
    pointer(mysql_stat) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_stat');
    pointer(mysql_get_server_info) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_get_server_info');
    pointer(mysql_get_host_info) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_get_host_info');
    pointer(mysql_get_proto_info) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_get_proto_info');
    pointer(mysql_list_dbs) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_list_dbs');
    pointer(mysql_list_tables) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_list_tables');
    pointer(mysql_list_fields) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_list_fields');
    pointer(mysql_list_processes) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_list_processes');
    pointer(mysql_store_result) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_store_result');
    pointer(mysql_use_result) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_use_result');
    pointer(mysql_free_result) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_free_result');
    pointer(mysql_data_seek) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_data_seek');
    pointer(mysql_row_seek) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_row_seek');
    pointer(mysql_field_seek) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_field_seek');
    pointer(mysql_fetch_row) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_fetch_row');
    pointer(mysql_fetch_lengths) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_fetch_lengths');
    pointer(mysql_fetch_field) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_fetch_field');
    pointer(mysql_escape_string) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_escape_string');
    pointer(mysql_debug) := GetProcedureAddress(Mysql3LibraryHandle,'mysql_debug');

    InitialiseMysql3_com;
    end;
end;

Procedure ReleaseMysql3;

begin
  if RefCount > 0 then dec(RefCount);
  if RefCount = 0 then
    begin
    if not UnloadLibrary(Mysql3LibraryHandle) then inc(RefCount);
    ReleaseMysql3_com;
    end;
end;

{$i mysql3impl.inc}

end.
