{
  This file is created by H2Pas, and thereafter heavily edited to make it
  readable and dynamically loadable.

  The goal was not to be complete, but to make it work and maintainable.

  The mysql_com.h, mysql.h and some other files are merged together into this
  one file.

  Automatically converted by H2Pas 1.0.0 from mysql_com.h / mysql.h
  The following command line parameters were used:
    -p
    -D
    -l
    mysqlclient
    mysql_com.h / mysql.h

}
{$MODE objfpc}
{$MACRO on}

interface
{$ifdef Load_Dynamically}{$define LinkDynamically}{$endif}
uses
{$IFDEF LinkDynamically}
     sysutils,
{$ENDIF}
     ctypes, dynlibs;

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
    mysqllib = 'libmysqlclient.'+sharedsuffix;
  {$IF DEFINED(mysql80)}
    mysqlvlib = mysqllib+'.21';
  {$ELSEIF DEFINED(mysql57)}
    mysqlvlib = mysqllib+'.20';
  {$ELSEIF DEFINED(mysql55) or DEFINED(mysql56)}
    mysqlvlib = mysqllib+'.18';
  {$ELSEIF DEFINED(mysql51)}
    mysqlvlib = mysqllib+'.16';
  {$ELSEIF DEFINED(mysql50)}
    mysqlvlib = mysqllib+'.15';
  {$ELSEIF DEFINED(mysql41)}
    mysqlvlib = mysqllib+'.14';
  {$ELSE}
    mysqlvlib = mysqllib+'.12';
  {$ENDIF}
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    mysqllib = 'libmysql.dll';
    mysqlvlib = 'libmysql.dll';
{$ENDIF}


{$IFDEF mysql80}
  {$DEFINE mysql57}
{$ENDIF mysql80}

{$IFDEF mysql57}
  {$DEFINE mysql56}
{$ENDIF mysql57}

{$IFDEF mysql56}
  {$DEFINE mysql55}
{$ENDIF mysql56}

{$IFDEF mysql55}
  {$DEFINE mysql51}
{$ENDIF mysql55}

{$IFDEF mysql51}
  {$DEFINE mysql50}
{$ENDIF mysql51}

{$IFDEF mysql50}
  {$DEFINE mysql41}
{$ENDIF mysql50}

{$PACKRECORDS C}

  { Copyright (C) 2000-2003 MySQL AB
  
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
     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA  }

    type
       my_bool = cchar;
       Pmy_bool  = ^my_bool;
       ppcchar = ^pcchar;
       psize_t = pointer;

       PVIO = Pointer;

       Pgptr = ^gptr;
       gptr = ^cchar;

       Pmy_socket = ^my_socket;
       my_socket = cint;
       
{  ------------ Start of declaration in "mysql_com.h"   ---------------------  }

  {
  ** Common definition between mysql server & client
   }

  { Field/table name length  }

  const
     HOSTNAME_LENGTH = 60;
{$IFDEF mysql51}
     SYSTEM_CHARSET_MBMAXLEN = 3;
     FILENAME_CHARSET_MBMAXLEN = 5;
     NAME_CHAR_LEN = 64;              // Field/table name length
     USERNAME_CHAR_LENGTH = 16;
     NAME_LEN = (NAME_CHAR_LEN*SYSTEM_CHARSET_MBMAXLEN);
     USERNAME_LENGTH = (USERNAME_CHAR_LENGTH*SYSTEM_CHARSET_MBMAXLEN);
{$ELSE}
     NAME_LEN = 64;
     USERNAME_LENGTH = 16;
{$ENDIF}
     MYSQL_AUTODETECT_CHARSET_NAME = 'auto';
     SERVER_VERSION_LENGTH = 60;
     SQLSTATE_LENGTH = 5;
     LOCAL_HOST = 'localhost';
     LOCAL_HOST_NAMEDPIPE = '.';

  { Maximum length of comments }
  const
     TABLE_COMMENT_INLINE_MAXLEN = 180;
     TABLE_COMMENT_MAXLEN = 2048;
     COLUMN_COMMENT_MAXLEN = 1024;
     INDEX_COMMENT_MAXLEN = 1024;
     TABLE_PARTITION_COMMENT_MAXLEN = 1024;

  { Maximum length of protocol packet. }
  { OK packet length limit also restricted to this value as any length greater
    than this value will have first byte of OK packet to be 254 thus does not
    provide a means to identify if this is OK or EOF packet. }
     MAX_PACKET_LENGTH = (256*256*256-1);

  const
     MYSQL_NAMEDPIPE = 'MySQL';
     MYSQL_SERVICENAME = 'MySQL';

  type
     enum_server_command = (
       COM_SLEEP,
       COM_QUIT,
       COM_INIT_DB,
       COM_QUERY,
       COM_FIELD_LIST,
       COM_CREATE_DB,
       COM_DROP_DB,
       COM_REFRESH,
       COM_SHUTDOWN, // deprecated
       COM_STATISTICS,
       COM_PROCESS_INFO,
       COM_CONNECT,
       COM_PROCESS_KILL,
       COM_DEBUG,
       COM_PING,
       COM_TIME,
       COM_DELAYED_INSERT,
       COM_CHANGE_USER,
       COM_BINLOG_DUMP,
       COM_TABLE_DUMP,
       COM_CONNECT_OUT,
       COM_REGISTER_SLAVE,
{$IFDEF mysql50}
       COM_STMT_PREPARE,
       COM_STMT_EXECUTE,
       COM_STMT_SEND_LONG_DATA,
       COM_STMT_CLOSE,
       COM_STMT_RESET,
       COM_SET_OPTION,
       COM_STMT_FETCH,
 {$IFDEF mysql51}
       COM_DAEMON,
  {$IFDEF mysql56}
       COM_BINLOG_DUMP_GTID,
   {$IFDEF mysql57}
       COM_RESET_CONNECTION,
     {$IFDEF mysql80}
       COM_CLONE,
     {$ENDIF}
   {$ENDIF}
  {$ENDIF}
 {$ENDIF}
{$ELSE}
  {$IFDEF mysql41}
       COM_PREPARE,COM_EXECUTE,COM_LONG_DATA,COM_CLOSE_STMT,
       COM_RESET_STMT,COM_SET_OPTION,
  {$ENDIF}
{$ENDIF}
       { Must be last }
       COM_END
       );

  {
    Length of random string sent by server on handshake; this is also length of
    obfuscated password, recieved from client
   }

  const
     SCRAMBLE_LENGTH = 20;
     SCRAMBLE_LENGTH_323 = 8;

  { length of password stored in the db: new passwords are preceeded with '*'  }

     SCRAMBLED_PASSWORD_CHAR_LENGTH = SCRAMBLE_LENGTH*2+1;
     SCRAMBLED_PASSWORD_CHAR_LENGTH_323 = SCRAMBLE_LENGTH_323*2;


       NOT_NULL_FLAG = 1;       //  Field can't be NULL
       PRI_KEY_FLAG = 2;        //  Field is part of a primary key
       UNIQUE_KEY_FLAG = 4;     //  Field is part of a unique key
       MULTIPLE_KEY_FLAG = 8;   //  Field is part of a key
       BLOB_FLAG = 16;          //  Field is a blob
       UNSIGNED_FLAG = 32;      //  Field is unsigned
       ZEROFILL_FLAG = 64;      //  Field is zerofill
       BINARY_FLAG = 128;       //  Field is binary

    { The following are only sent to new clients  }

       ENUM_FLAG = 256;            // field is an enum
       AUTO_INCREMENT_FLAG = 512;  // field is a autoincrement field
       TIMESTAMP_FLAG = 1024;      // Field is a timestamp
       SET_FLAG = 2048;            // field is a set
{$IFDEF mysql50}
       NO_DEFAULT_VALUE_FLAG=4096; // Field doesn't have default value
{$ENDIF}
{$IFDEF mysql51}
       ON_UPDATE_NOW_FLAG=8192;    // Field is set to NOW on UPDATE
{$ENDIF}
       NUM_FLAG = 32768;           // Field is num (for clients)
       PART_KEY_FLAG = 16384;      // Intern; Part of some key
       GROUP_FLAG = 32768;         // Intern: Group field
       UNIQUE_FLAG = 65536;        // Intern: Used by sql_yacc
       BINCMP_FLAG = 131072;       // Intern: Used by sql_yacc
{$IFDEF mysql51}
       GET_FIXED_FIELDS_FLAG = (1 shl 18);    // Used to get fields in item tree
       FIELD_IN_PART_FUNC_FLAG = (1 shl 19);  // Field part of partition func
       FIELD_IN_ADD_INDEX = (1 shl 20);       // Intern: Field in TABLE object for new version of altered table,
                                              //         which participates in a newly added index.
       FIELD_IS_RENAMED = (1 shl 21);         // Intern: Field is being renamed
 {$IFDEF mysql55}
       FIELD_FLAGS_STORAGE_MEDIA = 22;        // Field storage media, bit 22-23
       FIELD_FLAGS_STORAGE_MEDIA_MASK = (3 shl FIELD_FLAGS_STORAGE_MEDIA);
       FIELD_FLAGS_COLUMN_FORMAT = 24;        // Field column format, bit 24-25
       FIELD_FLAGS_COLUMN_FORMAT_MASK = (3 shl FIELD_FLAGS_COLUMN_FORMAT);
  {$IFDEF mysql56}
       FIELD_IS_DROPPED = (1 shl 26);         // Intern: Field is being dropped
   {$IFDEF mysql57}
       EXPLICIT_NULL_FLAG = (1 shl 27);       // Field is explicitly specified as NULL by the user
     {$IFDEF mysql80}
       FIELD_IS_MARKED = (1 shl 28);       // Intern: field is marked, general purpose
       NOT_SECONDARY_FLAG = (1 << 29); // Field will not be loaded in secondary engine.
       FIELD_IS_INVISIBLE = (1 << 30); // Field is explicitly marked as invisible by the user.
     {$ENDIF}
   {$ENDIF}
  {$ENDIF}
 {$ENDIF}
{$ENDIF}

       REFRESH_GRANT = 1;          // Refresh grant tables
       REFRESH_LOG = 2;            // Start on new log file
       REFRESH_TABLES = 4;         // close all tables
       REFRESH_HOSTS = 8;          // Flush host cache
       REFRESH_STATUS = 16;        // Flush status variables
       REFRESH_THREADS = 32;       // Flush thread cache
       REFRESH_REPLICA = 64;         // Reset master info and restart replica thread
       REFRESH_SLAVE = REFRESH_REPLICA;  // Reset master info and restart slave thread
       REFRESH_MASTER = 128;       // Remove all bin logs in the index and truncate the index
       REFRESH_ERROR_LOG = 256;    // Rotate only the erorr log
       REFRESH_ENGINE_LOG = 512;   // Flush all storage engine logs
       REFRESH_BINARY_LOG = 1024;  // Flush the binary log
       REFRESH_RELAY_LOG = 2048;   // Flush the relay log
       REFRESH_GENERAL_LOG = 4096; // Flush the general log
       REFRESH_SLOW_LOG = 8192;    // Flush the slow query log

    { The following can't be set with mysql_refresh()  }
       REFRESH_READ_LOCK = 16384;          // Lock tables for read
       REFRESH_FAST = 32768;               // Intern flag

       {$IFNDEF mysql80}
       REFRESH_QUERY_CACHE = 65536;        // RESET (remove all queries) from query cache
       REFRESH_QUERY_CACHE_FREE = $20000;  // pack query cache
       REFRESH_DES_KEY_FILE = $40000;
       {$ENDIF}

       REFRESH_USER_RESOURCES = $80000;
       REFRESH_FOR_EXPORT = $100000;       // FLUSH TABLES ... FOR EXPORT
       REFRESH_OPTIMIZER_COSTS = $200000;  // FLUSH OPTIMIZER_COSTS
       {$IFDEF mysql80}
       REFRESH_PERSIST = $400000;          // RESET PERSIST
       {$ENDIF}

       CLIENT_LONG_PASSWORD = 1;           // new more secure passwords
       CLIENT_FOUND_ROWS = 2;              // Found instead of affected rows
       CLIENT_LONG_FLAG = 4;               // Get all column flags
       CLIENT_CONNECT_WITH_DB = 8;         // One can specify db on connect
       CLIENT_NO_SCHEMA = 16;              // Don't allow database.table.column
       CLIENT_COMPRESS = 32;               // Can use compression protocol
       CLIENT_ODBC = 64;                   // Odbc client
       CLIENT_LOCAL_FILES = 128;           // Can use LOAD DATA LOCAL
       CLIENT_IGNORE_SPACE = 256;          // Ignore spaces before '('
       CLIENT_PROTOCOL_41 = 512;           // New 4.1 protocol
       CLIENT_INTERACTIVE = 1024;          // This is an interactive client
       CLIENT_SSL = 2048;                  // Switch to SSL after handshake
       CLIENT_IGNORE_SIGPIPE = 4096;       // IGNORE sigpipes
       CLIENT_TRANSACTIONS = 8192;         // Client knows about transactions
       CLIENT_RESERVED = 16384;            // Old flag for 4.1 protocol
       CLIENT_SECURE_CONNECTION = 32768;   // Old flag for 4.1 authentication
       CLIENT_MULTI_STATEMENTS = 65536;    // Enable/disable multi-stmt support
       CLIENT_MULTI_RESULTS = 131072;      // Enable/disable multi-results
       CLIENT_PS_MULTI_RESULTS : cardinal = 1 shl 18; // Multi-results in PS-protocol
       CLIENT_PLUGIN_AUTH : cardinal = 1 shl 19;      // Client supports plugin authentication
       CLIENT_CONNECT_ATTRS : cardinal = (1 shl 20);  // Client supports connection attributes
       CLIENT_PLUGIN_AUTH_LENENC_CLIENT_DATA : cardinal = (1 shl 21);  // Enable authentication response packet to be larger than 255 bytes.
       CLIENT_CAN_HANDLE_EXPIRED_PASSWORDS : cardinal = (1 shl 22);    // Don't close the connection for a connection with expired password.
       CLIENT_SESSION_TRACK : cardinal = (1 shl 23);  // Capable of handling server state change information. Its a hint to the server to include the state change information in Ok packet.
       CLIENT_DEPRECATE_EOF : cardinal = (1 shl 24);  // Client no longer needs EOF packet
       {$IFDEF mysql80}
       CLIENT_OPTIONAL_RESULTSET_METADATA : cardinal = (1 shl 25);  // client can handle optional metadata information in the resultset
       CLIENT_ZSTD_COMPRESSION_ALGORITHM : cardinal = (1 shl 26); // Client sets this flag when it is configured to use zstd compression method
       CLIENT_QUERY_ATTRIBUTES : cardinal = (1 shl 27); // Can send the optional part containing the query parameter set(s)
       {$ENDIF}
       CLIENT_SSL_VERIFY_SERVER_CERT : cardinal = 1 shl 30;
       CLIENT_REMEMBER_OPTIONS : cardinal = 1 shl 31;


       SERVER_STATUS_IN_TRANS = 1;         // Is raised when a multi-statement transaction
                                           //  has been started, either explicitly, by means
                                           //  of BEGIN or COMMIT AND CHAIN, or implicitly, by the first transactional
                                           //  statement, when autocommit=off
       SERVER_STATUS_AUTOCOMMIT = 2;       // Server in auto_commit mode
       SERVER_STATUS_MORE_RESULTS = 4;     // More results on server
       SERVER_MORE_RESULTS_EXISTS = 8;     // Multi query - next query exists
       SERVER_QUERY_NO_GOOD_INDEX_USED = 16;
       SERVER_QUERY_NO_INDEX_USED = 32;
{$IFDEF mysql50}
    { The server was able to fulfill the clients request and opened a
      read-only non-scrollable cursor for a query. This flag comes
      in reply to COM_STMT_EXECUTE and COM_STMT_FETCH commands. }
       SERVER_STATUS_CURSOR_EXISTS = 64;
    { This flag is sent when a read-only cursor is exhausted, in reply to
      COM_STMT_FETCH command. }
       SERVER_STATUS_LAST_ROW_SENT = 128;
{$ENDIF}
       SERVER_STATUS_DB_DROPPED = 256;     // A database was dropped
{$IFDEF mysql50}
       SERVER_STATUS_NO_BACKSLASH_ESCAPES = 512;
 {$IFDEF mysql51}
    {
      Sent to the client if after a prepared statement reprepare
      we discovered that the new statement returns a different
      number of result set columns.
    }
       SERVER_STATUS_METADATA_CHANGED = 1024;
  {$IFDEF mysql55}
       SERVER_QUERY_WAS_SLOW = 2048;
       SERVER_PS_OUT_PARAMS = 4096; // To mark ResultSet containing output parameter values.
   {$IFDEF mysql56}
       SERVER_STATUS_IN_TRANS_READONLY = 8192;
    {$IFDEF mysql57}
       SERVER_SESSION_STATE_CHANGED = (1 shl 14); // This status flag, when on, implies that one of the state information has changed on the server because of the execution of the last statement.
    {$ENDIF}
   {$ENDIF}
  {$ENDIF}
 {$ENDIF}
{$ENDIF}

{$IFDEF mysql41}
       MYSQL_ERRMSG_SIZE = 512;
{$ELSE}
       MYSQL_ERRMSG_SIZE = 200;
{$ENDIF}
       NET_READ_TIMEOUT = 30;              // Timeout on read
       NET_WRITE_TIMEOUT = 60;             // Timeout on write
       NET_WAIT_TIMEOUT	= 8*60*60;         // Wait for new query
{$IFDEF mysql50}
       ONLY_KILL_QUERY = 1;
{$ENDIF}


    const
       MAX_TINYINT_WIDTH = 3;           // Max width for a TINY w.o. sign
       MAX_SMALLINT_WIDTH = 5;          // Max width for a SHORT w.o. sign
       MAX_MEDIUMINT_WIDTH = 8;         // Max width for a INT24 w.o. sign
       MAX_INT_WIDTH = 10;              // Max width for a LONG w.o. sign
       MAX_BIGINT_WIDTH = 20;           // Max width for a LONGLONG
       MAX_CHAR_WIDTH = 255;            // Max length for a CHAR colum
{$IFDEF mysql51}
       MAX_BLOB_WIDTH = 16777216;       // Default width for blob
{$ELSE}
       MAX_BLOB_WIDTH = 8192;           // Default width for blob
{$ENDIF}

    type
       Pst_net = ^st_net;
       st_net = record
            vio : PVio;
            buff : pcuchar;
            buff_end : pcuchar;
            write_pos : pcuchar;
            read_pos : pcuchar;
            fd : my_socket;     // For Perl DBI/dbd
            { The following variable is set if we are doing several queries in one
              command ( as in LOAD TABLE ... FROM MASTER ),
              and do not want to confuse the client with OK at the wrong time }
{$IFDEF mysql51}
            remain_in_buf,length, buf_length, where_b: culong;
            max_packet,max_packet_size: culong;
            pkt_nr,compress_pkt_nr: cuint;
            write_timeout, read_timeout, retry_count: cuint;
            fcntl: cint;
            return_status: pcuint;
            reading_or_writing: cuchar;
            save_char: cchar;
              unused1: my_bool;  // Please remove with the next incompatible ABI change
              unused2: my_bool;  // Please remove with the next incompatible ABI change
            compress: my_bool;
              unused3: my_bool;  // Please remove with the next incompatible ABI change
            { Pointer to query object in query cache, do not equal NULL (0) for
              queries in cache that have not stored its results yet }
              unused: pcuchar;
            last_errno: cuint;
            error: cuchar;
              unused4: my_bool;  // Please remove with the next incompatible ABI change
              unused5: my_bool;  // Please remove with the next incompatible ABI change
            { Client library error message buffer. Actually belongs to struct MYSQL. }
            last_error: array[0..MYSQL_ERRMSG_SIZE-1] of cchar;
            { Client library sqlstate buffer. Set along with the error message. }
            sqlstate: array[0..SQLSTATE_LENGTH] of cchar;
            { Extension pointer, for the caller private use.
              Any program linking with the networking library can use this pointer,
              which is handy when private connection specific data needs to be
              maintained.
              The mysqld server process uses this pointer internally,
              to maintain the server internal instrumentation for the connection. }
            extension: Pointer;
{$ELSE} // pre 5.1 version ?
            max_packet : culong;
            max_packet_size : culong;
            {$IFNDEF mysql41}
            last_errno : cuint;
            {$ENDIF}
            pkt_nr : cuint;
            compress_pkt_nr : cuint;
            write_timeout : cuint;
            read_timeout : cuint;
            retry_count : cuint;
            fcntl : cint;
            {$IFNDEF mysql41}
            last_error : array[0..(MYSQL_ERRMSG_SIZE)-1] of char;
            error : cuchar;
            return_errno : my_bool;
            {$ENDIF}
            compress : my_bool;
            {   The following variable is set if we are doing several queries in one
            command ( as in LOAD TABLE ... FROM MASTER ),
            and do not want to confuse the client with OK at the wrong time }
            remain_in_buf : culong;
            length : culong;
            buf_length : culong;
            where_b : culong;
            return_status : pcint;
            reading_or_writing : cuchar;
            save_char : cchar;
            no_send_ok : my_bool;     // For SPs and other things that do multiple stmts
            {$IFDEF mysql50}
            no_send_eof : my_bool;    // For SPs' first version read-only cursors
            no_send_error : my_bool;  // Set if OK packet is already sent, and
                                      // we do not need to send error messages
            {$ENDIF}
            {   Pointer to query object in query cache, do not equal NULL (0) for
                queries in cache that have not stored its results yet }
            {$IFDEF mysql41}
            last_error : array[0..(MYSQL_ERRMSG_SIZE)-1] of char;
            sqlstate : array[0..(SQLSTATE_LENGTH+1)-1] of char;
            last_errno : cuint;
            error : cuchar;
            {$ENDIF}
            query_cache_query : gptr;
            {$IFDEF mysql41}
            report_error : my_bool;   // We should report error (we have unreported error)
            return_errno : my_bool;
            {$ENDIF}
{$ENDIF} // pre 5.1 version ?
         end;
       NET = st_net;
       PNET = ^NET;

    const
       packet_error : culong = culong(not(0));

    type
       enum_field_types = (
         MYSQL_TYPE_DECIMAL,
         MYSQL_TYPE_TINY,
         MYSQL_TYPE_SHORT,
         MYSQL_TYPE_LONG,
         MYSQL_TYPE_FLOAT,
         MYSQL_TYPE_DOUBLE,
         MYSQL_TYPE_NULL,
         MYSQL_TYPE_TIMESTAMP,
         MYSQL_TYPE_LONGLONG,
         MYSQL_TYPE_INT24,
         MYSQL_TYPE_DATE,
         MYSQL_TYPE_TIME,
         MYSQL_TYPE_DATETIME,
         MYSQL_TYPE_YEAR,
         MYSQL_TYPE_NEWDATE,
{$IFDEF mysql50}
         MYSQL_TYPE_VARCHAR, MYSQL_TYPE_BIT,
 {$IFDEF mysql56}
         MYSQL_TYPE_TIMESTAMP2, MYSQL_TYPE_DATETIME2, MYSQL_TYPE_TIME2,
   {$IFDEF mysql80}
         MYSQL_TYPE_TYPED_ARRAY, // Used for replication only
         MYSQL_TYPE_INVALID := 243,
         MYSQL_TYPE_BOOL := 244, // Currently just a placeholder
         MYSQL_TYPE_JSON := 245,
   {$ENDIF}
 {$ENDIF}
         MYSQL_TYPE_NEWDECIMAL := 246,
{$ENDIF}
         MYSQL_TYPE_ENUM := 247,
         MYSQL_TYPE_SET := 248,
         MYSQL_TYPE_TINY_BLOB := 249,
         MYSQL_TYPE_MEDIUM_BLOB := 250,
         MYSQL_TYPE_LONG_BLOB := 251,
         MYSQL_TYPE_BLOB := 252,
         MYSQL_TYPE_VAR_STRING := 253,
         MYSQL_TYPE_STRING := 254,
         MYSQL_TYPE_GEOMETRY := 255
         );

    { For backward compatibility  }

    const
       CLIENT_MULTI_QUERIES = CLIENT_MULTI_STATEMENTS;
       FIELD_TYPE_DECIMAL = MYSQL_TYPE_DECIMAL;
{$IFDEF mysql50}
       FIELD_TYPE_NEWDECIMAL = MYSQL_TYPE_NEWDECIMAL;
{$ENDIF}
       FIELD_TYPE_TINY = MYSQL_TYPE_TINY;
       FIELD_TYPE_SHORT = MYSQL_TYPE_SHORT;
       FIELD_TYPE_LONG = MYSQL_TYPE_LONG;
       FIELD_TYPE_FLOAT = MYSQL_TYPE_FLOAT;
       FIELD_TYPE_DOUBLE = MYSQL_TYPE_DOUBLE;
       FIELD_TYPE_NULL = MYSQL_TYPE_NULL;
       FIELD_TYPE_TIMESTAMP = MYSQL_TYPE_TIMESTAMP;
       FIELD_TYPE_LONGLONG = MYSQL_TYPE_LONGLONG;
       FIELD_TYPE_INT24 = MYSQL_TYPE_INT24;
       FIELD_TYPE_DATE = MYSQL_TYPE_DATE;
       FIELD_TYPE_TIME = MYSQL_TYPE_TIME;
       FIELD_TYPE_DATETIME = MYSQL_TYPE_DATETIME;
       FIELD_TYPE_YEAR = MYSQL_TYPE_YEAR;
       FIELD_TYPE_NEWDATE = MYSQL_TYPE_NEWDATE;
       FIELD_TYPE_ENUM = MYSQL_TYPE_ENUM;
       FIELD_TYPE_SET = MYSQL_TYPE_SET;
       FIELD_TYPE_TINY_BLOB = MYSQL_TYPE_TINY_BLOB;
       FIELD_TYPE_MEDIUM_BLOB = MYSQL_TYPE_MEDIUM_BLOB;
       FIELD_TYPE_LONG_BLOB = MYSQL_TYPE_LONG_BLOB;
       FIELD_TYPE_BLOB = MYSQL_TYPE_BLOB;
       FIELD_TYPE_VAR_STRING = MYSQL_TYPE_VAR_STRING;
       FIELD_TYPE_STRING = MYSQL_TYPE_STRING;
       FIELD_TYPE_CHAR = MYSQL_TYPE_TINY;
       FIELD_TYPE_INTERVAL = MYSQL_TYPE_ENUM;
       FIELD_TYPE_GEOMETRY = MYSQL_TYPE_GEOMETRY;
{$IFDEF mysql50}
       FIELD_TYPE_BIT = MYSQL_TYPE_BIT;
{$ENDIF}
    { Shutdown/kill enums and constants  }
    { Bits for THD::killable.  }
       MYSQL_SHUTDOWN_KILLABLE_CONNECT    : cuchar = 1 shl 0;
       MYSQL_SHUTDOWN_KILLABLE_TRANS      : cuchar = 1 shl 1;
       MYSQL_SHUTDOWN_KILLABLE_LOCK_TABLE : cuchar = 1 shl 2;
       MYSQL_SHUTDOWN_KILLABLE_UPDATE     : cuchar = 1 shl 3;


    {   We want levels to be in growing order of hardness (because we use number
        comparisons). Note that DEFAULT does not respect the growing property, but
        it's ok.  }
    type
       mysql_enum_shutdown_level = (SHUTDOWN_DEFAULT := 0,
         SHUTDOWN_WAIT_CONNECTIONS := 1, //MYSQL_SHUTDOWN_KILLABLE_CONNECT,     // wait for existing connections to finish
         SHUTDOWN_WAIT_TRANSACTIONS := 2, //MYSQL_SHUTDOWN_KILLABLE_TRANS,      // wait for existing trans to finish
         SHUTDOWN_WAIT_UPDATES := 8, //MYSQL_SHUTDOWN_KILLABLE_UPDATE,          // wait for existing updates to finish (=> no partial MyISAM update)
         SHUTDOWN_WAIT_ALL_BUFFERS := 16, //MYSQL_SHUTDOWN_KILLABLE_UPDATE shl 1,// flush InnoDB buffers and other storage engines' buffers
         SHUTDOWN_WAIT_CRITICAL_BUFFERS := 17, //(MYSQL_SHUTDOWN_KILLABLE_UPDATE shl 1)+1, // don't flush InnoDB buffers, flush other storage engines' buffers
    { Now the 2 levels of the KILL command  }
{ $if MYSQL_VERSION_ID >= 50000}
         KILL_QUERY := 254,
{ $endif}
         KILL_CONNECTION := 255
         );

{$IFDEF mysql50}
       enum_cursor_type = (CURSOR_TYPE_NO_CURSOR := 0,CURSOR_TYPE_READ_ONLY := 1,
         CURSOR_TYPE_FOR_UPDATE := 2,CURSOR_TYPE_SCROLLABLE := 4
         );
{$ENDIF}

    { options for mysql_set_option  }
       enum_mysql_set_option = (MYSQL_OPTION_MULTI_STATEMENTS_ON,
         MYSQL_OPTION_MULTI_STATEMENTS_OFF
         );

{$IFDEF mysql57}
    { Type of state change information that the server can include in the Ok
      packet.
      Note : 1) session_state_type shouldn't go past 255 (i.e. 1-byte boundary).
             2) Modify the definition of SESSION_TRACK_END when a new member is added.
    }
       enum_session_state_type = (
         SESSION_TRACK_SYSTEM_VARIABLES, // Session system variables
         SESSION_TRACK_SCHEMA,           // Current schema
         SESSION_TRACK_STATE_CHANGE,     // track session state changes
         SESSION_TRACK_GTIDS
       );

    const
       SESSION_TRACK_BEGIN = ord(SESSION_TRACK_SYSTEM_VARIABLES);
       SESSION_TRACK_END = ord(SESSION_TRACK_GTIDS);
{$ENDIF}

    function net_new_transaction(net : st_net) : st_net;

{$IFNDEF LinkDynamically}
    function my_net_init(net:PNET; vio:PVio):my_bool;cdecl;external mysqllib name 'my_net_init';
    procedure my_net_local_init(net:PNET);cdecl;external mysqllib name 'my_net_local_init';
    procedure net_end(net:PNET);cdecl;external mysqllib name 'net_end';
    procedure net_clear(net:PNET{$IFDEF mysql51};check_buffer:my_bool{$ENDIF});cdecl;external mysqllib name 'net_clear';
    function net_realloc(net:PNET; length:culong):my_bool;cdecl;external mysqllib name 'net_realloc';
    function net_flush(net:PNET):my_bool;cdecl;external mysqllib name 'net_flush';
    function my_net_write(net:PNET; packet:Pchar; len:culong):my_bool;cdecl;external mysqllib name 'my_net_write';
    function net_write_command(net:PNET; command:cuchar; header:Pchar; head_len:culong; packet:Pchar;
               len:culong):my_bool;cdecl;external mysqllib name 'net_write_command';
 {$IFDEF mysql56}
    function net_write_packet(net:PNET; packet:Pchar; length:culong):my_bool;cdecl;external mysqllib name 'net_write_packet';
 {$ELSE}
    function net_real_write(net:PNET; packet:Pchar; len:culong):cint;cdecl;external mysqllib name 'net_real_write';
 {$ENDIF}
    function my_net_read(net:PNET):culong;cdecl;external mysqllib name 'my_net_read';
{$ENDIF}
    { The following function is not meant for normal usage
      Currently it's used internally by manager.c  }

    type
       Psockaddr = ^sockaddr;
       sockaddr = record
           // undefined structure
         end;
{$IFNDEF LinkDynamically}
    function my_connect(s:my_socket; name:Psockaddr; namelen:cuint; timeout:cuint):cint;cdecl;external mysqllib name 'my_connect';
{$ENDIF}

    type
       Prand_struct = ^rand_struct;
       rand_struct = record
            seed1 : culong;
            seed2 : culong;
            max_value : culong;
            max_value_dbl : cdouble;
         end;

    { The following is for user defined functions  }
{$IFDEF mysql50}
       Item_result = (STRING_RESULT,REAL_RESULT,INT_RESULT,
         ROW_RESULT);
{$ELSE}
       Item_result = (STRING_RESULT := 0,REAL_RESULT,INT_RESULT,
         ROW_RESULT,DECIMAL_RESULT);
{$ENDIF}
       PItem_result = ^Item_result;

       Pst_udf_args = ^st_udf_args;
       st_udf_args = record
            arg_count : cuint;           // Number of arguments
            arg_type : PItem_result;     // Pointer to item_results
            args : PPChar;               // Pointer to item_results
            lengths : pculong;            // Length of string arguments
            maybe_null : Pchar;          // Length of string arguments
{$IFDEF mysql50}
            attributes : PPChar;         // Pointer to attribute name
            attribute_lengths : pculong;  // Length of attribute arguments
 {$IFDEF mysql51}
            extension: pointer;
 {$ENDIF}
{$ENDIF}
         end;
       UDF_ARGS = st_udf_args;
       PUDF_ARGS = ^UDF_ARGS;

    { This holds information about the result  }

       Pst_udf_init = ^st_udf_init;
       st_udf_init = record
            maybe_null : my_bool;        // 1 if function can return NULL
            decimals : cuint;            // for real functions
            max_length : culong;          // For string functions
            ptr : Pchar;                 // free pointer for function data
            const_item : my_bool;        // free pointer for function data
{$IFDEF mysql51}
            extension: pointer;
{$ENDIF}
         end;
       UDF_INIT = st_udf_init;
       PUDF_INIT = ^UDF_INIT;

    { Constants when using compression  }
    const
       NET_HEADER_SIZE = 4;              // standard header size
       COMP_HEADER_SIZE = 3;             // compression header extra size

    { Prototypes to password functions  }

    { These functions are used for authentication by client and server and
      implemented in sql/password.c     }
{$IFNDEF LinkDynamically}
    procedure randominit(_para1:Prand_struct; seed1:culong; seed2:culong);cdecl;external mysqllib name 'randominit';
    function my_rnd(_para1:Prand_struct):cdouble;cdecl;external mysqllib name 'my_rnd';
    procedure create_random_string(fto:Pchar; length:cuint; rand_st:Prand_struct);cdecl;external mysqllib name 'create_random_string';
    procedure hash_password(fto:culong; password:Pchar; password_len:cuint);cdecl;external mysqllib name 'hash_password';
    procedure make_scrambled_password_323(fto:Pchar; password:Pchar);cdecl;external mysqllib name 'make_scrambled_password_323';
    procedure scramble_323(fto:Pchar; message:Pchar; password:Pchar);cdecl;external mysqllib name 'scramble_323';
    function check_scramble_323(_para1:Pchar; message:Pchar; salt:culong):my_bool;cdecl;external mysqllib name 'check_scramble_323';
    procedure get_salt_from_password_323(res:pculong; password:Pchar);cdecl;external mysqllib name 'get_salt_from_password_323';
    procedure make_password_from_salt_323(fto:Pchar; salt:pculong);cdecl;external mysqllib name 'make_password_from_salt_323';
{$IFDEF mysql50}
    function octet2hex(fto:Pchar; str:Pchar; len:cuint):pchar;cdecl;external mysqllib name 'octet2hex';
{$ENDIF}
    procedure make_scrambled_password(fto:Pchar; password:Pchar);cdecl;external mysqllib name 'make_scrambled_password';
    procedure scramble(fto:Pchar; message:Pchar; password:Pchar);cdecl;external mysqllib name 'scramble';
    function check_scramble(reply:Pchar; message:Pchar; hash_stage2:Pbyte):my_bool;cdecl;external mysqllib name 'check_scramble';
    procedure get_salt_from_password(res:Pbyte; password:Pchar);cdecl;external mysqllib name 'get_salt_from_password';
    procedure make_password_from_salt(fto:Pchar; hash_stage2:Pbyte);cdecl;external mysqllib name 'make_password_from_salt';
    { end of password.c  }

    function get_tty_password(opt_message:Pchar):Pchar;cdecl;external mysqllib name 'get_tty_password';
    function mysql_errno_to_sqlstate(mysql_errno:cuint):Pchar;cdecl;external mysqllib name 'mysql_errno_to_sqlstate';

    { Some other useful functions  }
{$IFDEF mysql50}
    function modify_defaults_file(file_location:Pchar; option:Pchar; option_value:Pchar; section_name:Pchar; remove_option:cint):cint;cdecl;external mysqllib name 'load_defaults';
{$ENDIF}

    function load_defaults(conf_file:Pchar; groups:PPchar; argc:pcint; argv:PPPchar):cint;cdecl;external mysqllib name 'load_defaults';
    function my_init:my_bool;cdecl;external mysqllib name 'my_init';
    function my_thread_init:my_bool;cdecl;external mysqllib name 'my_thread_init';
    procedure my_thread_end;cdecl;external mysqllib name 'my_thread_end';
{$ELSE}
    var
      my_init : function :my_bool;cdecl;
      my_thread_init : function :my_bool;cdecl;
      my_thread_end : procedure ;cdecl;
{$ENDIF}

{$ifdef _global_h}
{$IFNDEF LinkDynamically}
    function net_field_length(packet:PPuchar):culong;extdecl;external mysqllib name 'net_field_length_ll';
    function net_field_length_ll(packet:PPuchar):my_ulonglong;cdecl;external mysqllib name 'net_field_length_ll';
    function net_store_length(pkg:Pchar; length:ulonglong):Pchar;cdecl;external mysqllib name 'net_store_length';
{$ENDIF}
{$endif}

    const
       NULL_LENGTH : culong = culong(not(0)); // For net_store_length

    const
       MYSQL_STMT_HEADER      = 4;
       MYSQL_LONG_DATA_HEADER = 6;
       NOT_FIXED_DEC          = 31;

{  ------------ Stop of declaration in "mysql_com.h"   -----------------------  }

{ $include "mysql_time.h"}
    type
        mysql_timestamp_type = (
          MYSQL_TIMESTAMP_NONE = -2,
          MYSQL_TIMESTAMP_ERROR = -1,
          MYSQL_TIMESTAMP_DATE = 0,
          MYSQL_TIMESTAMP_DATETIME = 1,
          MYSQL_TIMESTAMP_TIME = 2
        );

        Pst_mysql_time = ^st_mysql_time;
        st_mysql_time = record
          year:        cuint;
          month:       cuint;
          day:         cuint;
          hour:        cuint;
          minute:      cuint;
          second:      cuint;
          second_part: culong;
          neg:         my_bool;
          time_type:   mysql_timestamp_type;
        end;

        PMYSQL_TIME = ^MYSQL_TIME;
        MYSQL_TIME = st_mysql_time;

{ $include "mysql_version.h"}
{ $include "typelib.h"}
{ $include "my_list.h" /* for LISTs used in 'MYSQL' and 'MYSQL_STMT' */}

{$IFNDEF LinkDynamically}
      var
         mysql_port : cuint;cvar;external;
         mysql_unix_port : Pchar;cvar;external;
{$ENDIF}

      const
         CLIENT_NET_READ_TIMEOUT = 365*24*3600;     // Timeout on read
         CLIENT_NET_WRITE_TIMEOUT = 365*24*3600;    // Timeout on write
      
    type
       Pst_mysql_field = ^st_mysql_field;
       st_mysql_field = record
            name : Pchar;             // Name of column
{$IFDEF mysql41}
            org_name : Pchar;         // Original column name, if an alias
{$ENDIF}
            table : Pchar;            // Table of column if column was a field
            org_table : Pchar;        // Org table name, if table was an alias
            db : Pchar;               // Database for table
{$IFDEF mysql41}
            catalog : Pchar;          // Catalog for table
{$ENDIF}
            def : Pchar;              // Default value (set by mysql_list_fields)
            length : culong;          // Width of column (create length)
            max_length : culong;      // Max width for selected set
{$IFDEF mysql41}
            name_length : cuint;
            org_name_length : cuint;
            table_length : cuint;
            org_table_length : cuint;
            db_length : cuint;
            catalog_length : cuint;
            def_length : cuint;
{$ENDIF}
            flags : cuint;            // Div flags
            decimals : cuint;         // Number of decimals in field
{$IFDEF mysql41}
            charsetnr : cuint;        // Character set
{$ENDIF}
            ftype : enum_field_types; // Type of field. See mysql_com.h for types
{$IFDEF mysql51}
            extension: pointer;
{$ENDIF}
         end;
       MYSQL_FIELD = st_mysql_field;
       PMYSQL_FIELD = ^MYSQL_FIELD;

       PMYSQL_ROW = ^MYSQL_ROW;       // return data as array of strings
       MYSQL_ROW = ppchar;

       PMYSQL_FIELD_OFFSET = ^MYSQL_FIELD_OFFSET;     // offset to current field
       MYSQL_FIELD_OFFSET = cuint;

    function IS_PRI_KEY(n : longint) : boolean;
    function IS_NOT_NULL(n : longint) : boolean;
    function IS_BLOB(n : longint) : boolean;
    function IS_NUM(t : enum_field_types) : boolean;
    function INTERNAL_NUM_FIELD(f : Pst_mysql_field) : boolean;
    function IS_NUM_FIELD(f : Pst_mysql_field) : boolean;

    type
{$if defined(NO_CLIENT_LONG_LONG)}
       my_ulonglong = culong;
{$elseif defined(WINDOWS)}
       my_ulonglong = cuint64;
{$else}
       my_ulonglong = culonglong;
{$endif}
       Pmy_ulonglong = ^my_ulonglong;

    const
       MYSQL_COUNT_ERROR = not (my_ulonglong(0));

    type
       Pst_mysql_rows = ^st_mysql_rows;
       st_mysql_rows = record
            next : Pst_mysql_rows;                    // list of rows
            data : MYSQL_ROW;
{$IFDEF mysql41}
            length : culong;
{$ENDIF}
         end;
       MYSQL_ROWS = st_mysql_rows;
       PMYSQL_ROWS = ^MYSQL_ROWS;

       PMYSQL_ROW_OFFSET = ^MYSQL_ROW_OFFSET;         // offset to current row
       MYSQL_ROW_OFFSET = MYSQL_ROWS;

{  ------------ Start of declaration in "my_alloc.h"     --------------------  }
{ $include "my_alloc.h"}

  const
     ALLOC_MAX_BLOCK_TO_DROP = 4096;
     ALLOC_MAX_BLOCK_USAGE_BEFORE_DROP = 10;

 { struct for once_alloc (block)  }
  type
     Pst_used_mem = ^st_used_mem;
     st_used_mem = record
          next : Pst_used_mem;   // Next block in use
          left : cuint;          // memory left in block
          size : cuint;          // size of block
       end;
     USED_MEM = st_used_mem;
     PUSED_MEM = ^USED_MEM;


     Pst_mem_root = ^st_mem_root;
     st_mem_root = record
          free : PUSED_MEM;      // blocks with free memory in it
          used : PUSED_MEM;      // blocks almost without free memory
          pre_alloc : PUSED_MEM; // preallocated block
          min_malloc : cuint;    // if block have less memory it will be put in 'used' list
          block_size : cuint;    // initial block size
          block_num : cuint;     // allocated blocks counter
  {    first free block in queue test counter (if it exceed
       MAX_BLOCK_USAGE_BEFORE_DROP block will be dropped in 'used' list)     }
          first_block_usage : cuint;
          error_handler : procedure ;cdecl;
       end;
     MEM_ROOT = st_mem_root;
     PMEM_ROOT = ^MEM_ROOT;

{  ------------ Stop of declaration in "my_alloc.h"    ----------------------  }

    type
       embedded_query_result = record end;

       Pst_mysql_data = ^st_mysql_data;
       st_mysql_data = record
{$IFDEF mysql51}
            data: PMYSQL_ROWS;
            embedded_info: ^embedded_query_result;
            alloc: MEM_ROOT;
            rows: my_ulonglong;
            fields: cuint;
            // extra info for embedded library
            extension: pointer;
{$ELSE}
            rows : my_ulonglong;
            fields : cuint;
            data : PMYSQL_ROWS;
            alloc : MEM_ROOT;
  { $if !defined(CHECK_EMBEDDED_DIFFERENCES) || defined(EMBEDDED_LIBRARY)}
  {$IFDEF mysql41}
            prev_ptr : ^PMYSQL_ROWS;
  {$ENDIF}
  { $endif}
 {$ENDIF}
         end;
       MYSQL_DATA = st_mysql_data;
       PMYSQL_DATA = ^MYSQL_DATA;

       mysql_option = (
         MYSQL_OPT_CONNECT_TIMEOUT,
         MYSQL_OPT_COMPRESS,
         MYSQL_OPT_NAMED_PIPE,
         MYSQL_INIT_COMMAND,
         MYSQL_READ_DEFAULT_FILE,
         MYSQL_READ_DEFAULT_GROUP,
         MYSQL_SET_CHARSET_DIR,
         MYSQL_SET_CHARSET_NAME,
         MYSQL_OPT_LOCAL_INFILE,
         MYSQL_OPT_PROTOCOL,
         MYSQL_SHARED_MEMORY_BASE_NAME,
         MYSQL_OPT_READ_TIMEOUT,
         MYSQL_OPT_WRITE_TIMEOUT,
         MYSQL_OPT_USE_RESULT,
         {$IFDEF MYSQL80}
         MYSQL_REPORT_DATA_TRUNCATION,
         MYSQL_OPT_RECONNECT,
         MYSQL_PLUGIN_DIR,
         MYSQL_DEFAULT_AUTH,
         MYSQL_OPT_BIND,
         MYSQL_OPT_SSL_KEY,
         MYSQL_OPT_SSL_CERT,
         MYSQL_OPT_SSL_CA,
         MYSQL_OPT_SSL_CAPATH,
         MYSQL_OPT_SSL_CIPHER,
         MYSQL_OPT_SSL_CRL,
         MYSQL_OPT_SSL_CRLPATH,
         MYSQL_OPT_CONNECT_ATTR_RESET,
         MYSQL_OPT_CONNECT_ATTR_ADD,
         MYSQL_OPT_CONNECT_ATTR_DELETE,
         MYSQL_SERVER_PUBLIC_KEY,
         MYSQL_ENABLE_CLEARTEXT_PLUGIN,
         MYSQL_OPT_CAN_HANDLE_EXPIRED_PASSWORDS,
         MYSQL_OPT_MAX_ALLOWED_PACKET,
         MYSQL_OPT_NET_BUFFER_LENGTH,
         MYSQL_OPT_TLS_VERSION,
         MYSQL_OPT_SSL_MODE,
         MYSQL_OPT_GET_SERVER_PUBLIC_KEY,
         MYSQL_OPT_RETRY_COUNT,
         MYSQL_OPT_OPTIONAL_RESULTSET_METADATA,
         MYSQL_OPT_SSL_FIPS_MODE,
         MYSQL_OPT_TLS_CIPHERSUITES,
         MYSQL_OPT_COMPRESSION_ALGORITHMS,
         MYSQL_OPT_ZSTD_COMPRESSION_LEVEL,
         MYSQL_OPT_LOAD_DATA_LOCAL_DIR
         {$ELSE}
         MYSQL_OPT_USE_REMOTE_CONNECTION,
         MYSQL_OPT_USE_EMBEDDED_CONNECTION,
         MYSQL_OPT_GUESS_CONNECTION,
         MYSQL_SET_CLIENT_IP,
         MYSQL_SECURE_AUTH
{$IFDEF MYSQL50}
         ,MYSQL_REPORT_DATA_TRUNCATION, MYSQL_OPT_RECONNECT
 {$IFDEF mysql51}
         ,MYSQL_OPT_SSL_VERIFY_SERVER_CERT
  {$IFDEF mysql55}
         ,MYSQL_PLUGIN_DIR, MYSQL_DEFAULT_AUTH
   {$IFDEF mysql56}
         ,MYSQL_OPT_BIND
         ,MYSQL_OPT_SSL_KEY, MYSQL_OPT_SSL_CERT, MYSQL_OPT_SSL_CA, MYSQL_OPT_SSL_CAPATH, MYSQL_OPT_SSL_CIPHER, MYSQL_OPT_SSL_CRL, MYSQL_OPT_SSL_CRLPATH
         ,MYSQL_OPT_CONNECT_ATTR_RESET, MYSQL_OPT_CONNECT_ATTR_ADD, MYSQL_OPT_CONNECT_ATTR_DELETE
         ,MYSQL_SERVER_PUBLIC_KEY
         ,MYSQL_ENABLE_CLEARTEXT_PLUGIN
         ,MYSQL_OPT_CAN_HANDLE_EXPIRED_PASSWORDS
    {$IFDEF mysql57}
         ,MYSQL_OPT_SSL_ENFORCE
    {$ENDIF}
   {$ENDIF}
  {$ENDIF}
 {$ENDIF}
{$ENDIF}
{$ENDIF}
         );

    const
       MAX_MYSQL_MANAGER_ERR = 256;
       MAX_MYSQL_MANAGER_MSG = 256;
       MANAGER_OK = 200;
       MANAGER_INFO = 250;
       MANAGER_ACCESS = 401;
       MANAGER_CLIENT_ERR = 450;
       MANAGER_INTERNAL_ERR = 500;

    type
       st_dynamic_array = record
            buffer : ^char;
            elements : cuint;
            max_element : cuint;
            alloc_increment : cuint;
            size_of_element : cuint;
         end;
       DYNAMIC_ARRAY = st_dynamic_array;
       Pst_dynamic_array = ^st_dynamic_array;

       st_mysql_options_extention = record end;

       Pst_mysql_options = ^st_mysql_options;
       st_mysql_options = record
            connect_timeout : cuint;
{$IFNDEF mysql41}
            client_flag : cuint;
            port : cuint;
{$ELSE}
            read_timeout : cuint;
            write_timeout : cuint;
{$ENDIF}
{$IFDEF mysql41}
            port : cuint;
            protocol : cuint;
            client_flag : culong;
{$ENDIF}
            host : Pchar;
{$IFNDEF mysql41}
            init_command: Pchar;
{$ENDIF}
            user : Pchar;
            password : Pchar;
            unix_socket : Pchar;
            db : Pchar;
{$IFDEF mysql41}
            init_commands : Pst_dynamic_array;
{$ENDIF}
            my_cnf_file : Pchar;
            my_cnf_group : Pchar;
            charset_dir : Pchar;
            charset_name : Pchar;
            ssl_key : Pchar;                 // PEM key file
            ssl_cert : Pchar;                // PEM cert file
            ssl_ca : Pchar;                  // PEM CA file
            ssl_capath : Pchar;              // PEM directory of CA-s?
            ssl_cipher : Pchar;              // cipher to use
{$IFDEF mysql41}
            shared_memory_base_name : Pchar;
{$ENDIF}
            max_allowed_packet : culong;
{$IFDEF Mysql80}
            compress : my_bool;
            named_pipe : my_bool;
            bind_address: Pchar;
            report_data_truncation: my_bool;
            { function pointers for local infile support  }
            local_infile_init : function (_para1:Ppointer; _para2:Pchar; _para3:pointer):cint;cdecl;
            local_infile_read : function (_para1:pointer; _para2:Pchar; _para3:cuint):cint;
            local_infile_end : procedure (_para1:pointer);
            local_infile_error : function (_para1:pointer; _para2:Pchar; _para3:cuint):cint;
            local_infile_userdata : pointer;
            extension : ^st_mysql_options_extention;
{$ELSE}
            use_ssl : my_bool;               // if to use SSL or not
            compress : my_bool;
            named_pipe : my_bool;
    {  On connect, find out the replication role of the server, and
       establish connections to all the peers  }
            rpl_probe : my_bool;
    {  Each call to mysql_real_query() will parse it to tell if it is a read
       or a write, and direct it to the slave or the master      }
            rpl_parse : my_bool;
    {  If set, never read from a master, only from slave, when doing
       a read that is replication-aware    }
            no_master_reads : my_bool;
{ $if !defined(CHECK_EMBEDDED_DIFFERENCES) || defined(EMBEDDED_LIBRARY)}
{$IFDEF mysql41}
            separate_thread : my_bool;
{ $endif}
            methods_to_use : mysql_option;
{$IFDEF mysql56}
            ci: record case integer of  // C union
                {The ip/hostname to use when authenticating
                 client against embedded server built with
                 grant tables - only used in embedded server}
              0: (client_ip: PChar;);
                {The local address to bind when connecting to
                 remote server - not used in embedded server}
              1: (bind_address: PChar;);
            end;
{$ELSE}
            client_ip : Pchar;
{$ENDIF}
            secure_auth : my_bool;           // Refuse client connecting to server if it uses old (pre-4.1.1) protocol
{$IFDEF mysql50}
            report_data_truncation : my_bool;// 0 - never report, 1 - always report (default)
{$ENDIF}
    { function pointers for local infile support  }
            local_infile_init : function (_para1:Ppointer; _para2:Pchar; _para3:pointer):cint;cdecl;
            local_infile_read : function (_para1:pointer; _para2:Pchar; _para3:cuint):cint;
            local_infile_end : procedure (_para1:pointer);
            local_infile_error : function (_para1:pointer; _para2:Pchar; _para3:cuint):cint;
            local_infile_userdata : pointer;
{$IFDEF mysql51}
            extension : ^st_mysql_options_extention;
{$ENDIF}
{$ENDIF}
{$ENDIF}
         end;

       mysql_status = (MYSQL_STATUS_READY,MYSQL_STATUS_GET_RESULT, MYSQL_STATUS_USE_RESULT
{$IFDEF mysql51}
                       ,MYSQL_STATUS_STATEMENT_GET_RESULT
{$ENDIF}
       );

       mysql_protocol_type = (MYSQL_PROTOCOL_DEFAULT,MYSQL_PROTOCOL_TCP,
         MYSQL_PROTOCOL_SOCKET,MYSQL_PROTOCOL_PIPE,
         MYSQL_PROTOCOL_MEMORY);

    { There are three types of queries - the ones that have to go to
      the master, the ones that go to a slave, and the adminstrative
      type which must happen on the pivot connectioin     }
       mysql_rpl_type = (MYSQL_RPL_MASTER,MYSQL_RPL_SLAVE,MYSQL_RPL_ADMIN
         );

       charset_info_st = record
            number : cuint;
            primary_number : cuint;
            binary_number : cuint;
            state : cuint;
            csname : ^char;
            name : ^char;
            comment : ^char;
            tailoring : ^char;
            ftype : ^cuchar;
            to_lower : ^cuchar;
            to_upper : ^cuchar;
            sort_order : ^cuchar;
            contractions : ^cuint16;
            sort_order_big : ^pword;
            tab_to_uni : ^cuint16;
            tab_from_uni : pointer; // was ^MY_UNI_IDX
            state_map : ^cuchar;
            ident_map : ^cuchar;
            strxfrm_multiply : cuint;
            mbminlen : cuint;
            mbmaxlen : cuint;
            min_sort_char : cuint16;
            max_sort_char : cuint16;
            escape_with_backslash_is_dangerous : my_bool;
            cset : pointer; // was ^MY_CHARSET_HANDLER
            coll : pointer; // was ^MY_COLLATION_HANDLER;
         end;
       CHARSET_INFO = charset_info_st;
       Pcharset_info_st = ^charset_info_st;

{$IFDEF mysql50}
       Pcharacter_set = ^character_set;
       character_set = record
            number : cuint;
            state : cuint;
            csname : Pchar;
            name : Pchar;
            comment : Pchar;
            dir : Pchar;
            mbminlen : cuint;
            mbmaxlen : cuint;
         end;
       MY_CHARSET_INFO = character_set;
       PMY_CHARSET_INFO = ^MY_CHARSET_INFO;
{$ENDIF}

       Pst_mysql_methods = ^st_mysql_methods;

       Pst_mysql = ^st_mysql;
       st_mysql = record
            net : NET;                   // Communication parameters
            connector_fd : gptr;         // ConnectorFd for SSL
            host : Pchar;
            user : Pchar;
            passwd : Pchar;
            unix_socket : Pchar;
            server_version : Pchar;
            host_info : Pchar;
            info : Pchar;
            db : Pchar;
            charset : Pcharset_info_st;
            fields : PMYSQL_FIELD;
            field_alloc : MEM_ROOT;
            affected_rows : my_ulonglong;
            insert_id : my_ulonglong;    // id if insert on table with NEXTNR
            extra_info : my_ulonglong;   // Used by mysqlshow, not used by mysql 5.0 and up
            thread_id : culong;          // Id for connection in server
            packet_length : culong;
            port : cuint;
            client_flag : culong;
            server_capabilities : culong;
            protocol_version : cuint;
            field_count : cuint;
            server_status : cuint;
            server_language : cuint;
            warning_count : cuint;
            options : st_mysql_options;
            status : mysql_status;
            free_me : my_bool;           // If free in mysql_close
            reconnect : my_bool;         // set to 1 if automatic reconnect
            scramble : array[0..(SCRAMBLE_LENGTH+1)-1] of char;  // session-wide random string
{$IFDEF mysql80}
{$ELSE}
    {  Set if this is the original connection, not a master or a slave we have
       added though mysql_rpl_probe() or mysql_set_master()/ mysql_add_slave()      }
            rpl_pivot : my_bool;
    {   Pointers to the master, and the next slave connections, points to
        itself if lone connection.       }
            master : Pst_mysql;
            next_slave : Pst_mysql;
            last_used_slave : Pst_mysql; // needed for round-robin slave pick
            last_used_con : Pst_mysql;   // needed for send/read/store/use result to work correctly with replication
{$ENDIF}
{$IFDEF mysql41}
            stmts : Pointer;             // was PList, list of all statements
            methods : Pst_mysql_methods;
            thd : pointer;
    {   Points to boolean flag in MYSQL_RES  or MYSQL_STMT. We set this flag
        from mysql_stmt_close if close had to cancel result set of this object.       }
            unbuffered_fetch_owner : Pmy_bool;
 {$IFDEF mysql50}
            info_buffer: ^cchar;
  {$IFDEF mysql51}
            extension: pointer;
  {$ENDIF}
 {$ENDIF}
{$ENDIF}
         end;
       MYSQL = st_mysql;
       PMYSQL = ^MYSQL;


       Pst_mysql_res = ^st_mysql_res;
       st_mysql_res = record
            row_count : my_ulonglong;
            fields : PMYSQL_FIELD;
            data : PMYSQL_DATA;
            data_cursor : PMYSQL_ROWS;
            lengths : pculong;           // column lengths of current row
            handle : PMYSQL;             // for unbuffered reads
{$IFDEF mysql51}
            methods : Pst_mysql_methods;
            row : MYSQL_ROW;             // If unbuffered read
            current_row : MYSQL_ROW;     // buffer to current row
            field_alloc : MEM_ROOT;
            field_count, current_field : cuint;
            eof : my_bool;               // Used by mysql_fetch_row
            unbuffered_fetch_cancelled : my_bool;  // mysql_stmt_close() had to cancel this result
            extension : pointer;
{$ELSE}
            field_alloc : MEM_ROOT;
            field_count : cuint;
            current_field : cuint;
            row : MYSQL_ROW;             // If unbuffered read
            current_row : MYSQL_ROW;     // buffer to current row
            eof : my_bool;               // Used by mysql_fetch_row
 {$IFDEF mysql41}
            unbuffered_fetch_cancelled : my_bool;  // mysql_stmt_close() had to cancel this result
            methods : Pst_mysql_methods;
 {$ENDIF}
{$ENDIF}
         end;
       MYSQL_RES = st_mysql_res;
       PMYSQL_RES = ^MYSQL_RES;

       Pst_mysql_stmt = ^st_mysql_stmt;
       PMYSQL_STMT = ^MYSQL_STMT;

       st_mysql_methods = record
            read_query_result : function (mysql:PMYSQL):my_bool;cdecl;
            advanced_command : function (mysql:PMYSQL; command:enum_server_command; header:Pcuchar; header_length:culong; arg:Pcuchar;
                         arg_length:culong; skip_check:my_bool):my_bool;
            read_rows : function (mysql:PMYSQL; mysql_fields:PMYSQL_FIELD; fields:cuint):PMYSQL_DATA;
            use_result : function (mysql:PMYSQL):PMYSQL_RES;
            fetch_lengths : procedure (fto:pculong; column:MYSQL_ROW; field_count:cuint);
            flush_use_result : procedure (mysql:PMYSQL);
{ $if !defined(MYSQL_SERVER) || defined(EMBEDDED_LIBRARY)}
            list_fields : function (mysql:PMYSQL):PMYSQL_FIELD;
            read_prepare_result : function (mysql:PMYSQL; stmt:PMYSQL_STMT):my_bool;
            stmt_execute : function (stmt:PMYSQL_STMT):cint;
            read_binary_rows : function (stmt:PMYSQL_STMT):cint;
            unbuffered_fetch : function (mysql:PMYSQL; row:PPchar):cint;
            free_embedded_thd : procedure (mysql:PMYSQL);
            read_statistics : function (mysql:PMYSQL):Pchar;
            next_result : function (mysql:PMYSQL):my_bool;
            read_change_user_result : function (mysql:PMYSQL; buff:Pchar; passwd:Pchar):cint;
{$IFDEF mysql50}
            read_rowsfrom_cursor : function (stmt:PMYSQL_STMT):cint;
{$ENDIF mysql50}
{ $endif}
         end;
       MYSQL_METHODS = st_mysql_methods;
       PMYSQL_METHODS = ^MYSQL_METHODS;


       Pst_mysql_manager = ^st_mysql_manager;
       st_mysql_manager = record
            net : NET;
            host : Pchar;
            user : Pchar;
            passwd : Pchar;
{$IFDEF mysql51}
            net_buf, net_buf_pos, net_data_end : pcchar;
            port : cuint;
            cmd_status : cint;
            last_errno : cint;
            net_buf_size : cint;
            free_me : my_bool;
            eof : my_bool;
            last_error : array[0..(MAX_MYSQL_MANAGER_ERR)-1] of cchar;
            extension : pointer;
{$ELSE}
            port : cuint;
            free_me : my_bool;
            eof : my_bool;
            cmd_status : cint;
            last_errno : cint;
            net_buf : Pchar;
            net_buf_pos : Pchar;
            net_data_end : Pchar;
            net_buf_size : cint;
            last_error : array[0..(MAX_MYSQL_MANAGER_ERR)-1] of cchar;
{$ENDIF}
         end;
       MYSQL_MANAGER = st_mysql_manager;
       PMYSQL_MANAGER = ^MYSQL_MANAGER;

       Pst_mysql_parameters = ^st_mysql_parameters;
       st_mysql_parameters = record
            p_max_allowed_packet : pculong;
            p_net_buffer_length : pculong;
{$IFDEF mysql51}
            extension : pointer;
{$ENDIF}
         end;
       MYSQL_PARAMETERS = st_mysql_parameters;
       PMYSQL_PARAMETERS = ^MYSQL_PARAMETERS;

    { The following definitions are added for the enhanced
      client-server protocol }

    { statement state  }

       enum_mysql_stmt_state = (MYSQL_STMT_INIT_DONE := 1,MYSQL_STMT_PREPARE_DONE,
         MYSQL_STMT_EXECUTE_DONE,MYSQL_STMT_FETCH_DONE
         );

    {
      Note: this info is from the mysql-5.0 version:
    
      This structure is used to define bind information, and
      internally by the client library.
      Public members with their descriptions are listed below
      (conventionally `On input' refers to the binds given to
      mysql_stmt_bind_param, `On output' refers to the binds given
      to mysql_stmt_bind_result):

      buffer_type    - One of the MYSQL_* types, used to describe
                       the host language type of buffer.
                       On output: if column type is different from
                       buffer_type, column value is automatically converted
                       to buffer_type before it is stored in the buffer.
      buffer         - On input: points to the buffer with input data.
                       On output: points to the buffer capable to store
                       output data.
                       The type of memory pointed by buffer must correspond
                       to buffer_type. See the correspondence table in
                       the comment to mysql_stmt_bind_param.

      The two above members are mandatory for any kind of bind.

      buffer_length  - the length of the buffer. You don't have to set
                       it for any fixed length buffer: float, double,
                       int, etc. It must be set however for variable-length
                       types, such as BLOBs or STRINGs.

      length         - On input: in case when lengths of input values
                       are different for each execute, you can set this to
                       point at a variable containining value length. This
                       way the value length can be different in each execute.
                       If length is not NULL, buffer_length is not used.
                       Note, length can even point at buffer_length if
                       you keep bind structures around while fetching:
                       this way you can change buffer_length before
                       each execution, everything will work ok.
                       On output: if length is set, mysql_stmt_fetch will
                       write column length into it.

      is_null        - On input: points to a boolean variable that should
                       be set to TRUE for NULL values.
                       This member is useful only if your data may be
                       NULL in some but not all cases.
                       If your data is never NULL, is_null should be set to 0.
                       If your data is always NULL, set buffer_type
                       to MYSQL_TYPE_NULL, and is_null will not be used.

      is_unsigned    - On input: used to signify that values provided for one
                       of numeric types are unsigned.
                       On output describes signedness of the output buffer.
                       If, taking into account is_unsigned flag, column data
                       is out of range of the output buffer, data for this column
                       is regarded truncated. Note that this has no correspondence
                       to the sign of result set column, if you need to find it out
                       use mysql_stmt_result_metadata.
      error          - where to write a truncation error if it is present.
                       possible error value is:
                       0  no truncation
                       1  value is out of range or buffer is too small

      Please note that MYSQL_BIND also has internals members.
    }
       Pst_mysql_bind = ^st_mysql_bind;

       st_mysql_bind = record
{$IFDEF mysql51}
            length : pculong;               // output length pointer
            is_null : Pmy_bool;             // Pointer to null indicator
            buffer : pointer;               // buffer to get/put data
            error: pmy_bool;                // set this if you want to track data truncations happened during fetch
            row_ptr : PByte;                // for the current data position
            store_param_func : procedure (net:PNET; param:Pst_mysql_bind);cdecl;
            fetch_result : procedure (_para1:Pst_mysql_bind; _para2:PMYSQL_FIELD; row:PPbyte);
            skip_result : procedure (_para1:Pst_mysql_bind; _para2:PMYSQL_FIELD; row:PPbyte);
            buffer_length : culong;         // output buffer length, must be set when fetching str/binary
            offset : culong;                // offset position for char/binary fetch
            length_value : culong;          //  Used if length is 0
            param_number : cuint;           // For null count and error messages
            pack_length : cuint;            // Internal length for packed data
            buffer_type : enum_field_types; // buffer type
            error_value : my_bool;          // used if error is 0
            is_unsigned : my_bool;          // set if integer type is unsigned
            long_data_used : my_bool;       // If used with mysql_send_long_data
            is_null_value : my_bool;        // Used if is_null is 0
            extension : Pointer;
{$ELSE}
            length : pculong;               // output length pointer
            is_null : Pmy_bool;             // Pointer to null indicator
            buffer : pointer;               // buffer to get/put data
 {$IFDEF mysql50}
            error: pmy_bool;                // set this if you want to track data truncations happened during fetch
 {$ENDIF}
            buffer_type : enum_field_types; // buffer type
            buffer_length : culong;         // buffer length, must be set for str/binary
    { Following are for internal use. Set by mysql_stmt_bind_param  }
 {$IFNDEF mysql50}
            inter_buffer : Pbyte;           // for the current data position
 {$ELSE}
            row_ptr : PByte;                // for the current data position
 {$ENDIF}
            offset : culong;                // offset position for char/binary fetch
 {$IFNDEF mysql50}
            internal_length : culong;       //  Used if length is 0
 {$ELSE}
            length_value : culong;          //  Used if length is 0
 {$ENDIF}
            param_number : cuint;           // For null count and error messages
            pack_length : cuint;            // Internal length for packed data
 {$IFDEF mysql50}
            error_value : my_bool;         // used if error is 0
 {$ENDIF}
            is_unsigned : my_bool;          // set if integer type is unsigned
            long_data_used : my_bool;       // If used with mysql_send_long_data
 {$IFNDEF mysql50}
            internal_is_null : my_bool;     // Used if is_null is 0
 {$ELSE}
            is_null_value : my_bool;        // Used if is_null is 0
 {$ENDIF}
            store_param_func : procedure (net:PNET; param:Pst_mysql_bind);cdecl;
            fetch_result : procedure (_para1:Pst_mysql_bind; _para2:PMYSQL_FIELD; row:PPbyte);
            skip_result : procedure (_para1:Pst_mysql_bind; _para2:PMYSQL_FIELD; row:PPbyte);
{$ENDIF}
         end;
       MYSQL_BIND = st_mysql_bind;
       PMYSQL_BIND = ^MYSQL_BIND;

       { From  "my_list.h" }
       st_list = record
         prev, next : ^st_list;
         data : pointer;
       end;
       LIST = st_list;

       { statement handler  }

       st_mysql_stmt_extension = record end;

       st_mysql_stmt = record
            mem_root : MEM_ROOT;            // root allocations
            list : LIST;                    // list to keep track of all stmts
            mysql : PMYSQL;                 // connection handle
            params : PMYSQL_BIND;           // input parameters
            bind : PMYSQL_BIND;             // input parameters
            fields : PMYSQL_FIELD;          // result set metadata
            result : MYSQL_DATA;            // cached result set
            data_cursor : PMYSQL_ROWS;      // current row in cached result
{$IFDEF mysql51}
            read_row_func : function (stmt:Pst_mysql_stmt; row:PPbyte):cint;cdecl;
            affected_rows : my_ulonglong;   // copy of mysql->affected_rows after statement execution
            insert_id : my_ulonglong;       // copy of mysql->insert_id
{$ELSE}
            affected_rows : my_ulonglong;   // copy of mysql->affected_rows after statement execution
            insert_id : my_ulonglong;       // copy of mysql->insert_id
    {   mysql_stmt_fetch() calls this function to fetch one row (it's different
        for buffered, unbuffered and cursor fetch).       }
            read_row_func : function (stmt:Pst_mysql_stmt; row:PPbyte):cint;cdecl;
{$ENDIF}
            stmt_id : culong;               // Id for prepared statement
{$IFDEF mysql50}
            flags : culong;                 // i.e. type of cursor to open
            prefetch_rows : culong;         // number of rows per one COM_FETCH
            server_status : cuint;          // Copied from mysql->server_status after execute/fetch to know
                                            // server-side cursor status for this statement.
{$ENDIF}
            last_errno : cuint;             // error code
            param_count : cuint;            // input parameter count
            field_count : cuint;            // number of columns in result set
            state : enum_mysql_stmt_state;  // statement state
            last_error : array[0..(MYSQL_ERRMSG_SIZE)-1] of char;  // error message
            sqlstate : array[0..(SQLSTATE_LENGTH+1)-1] of char;
            send_types_to_server : my_bool; // Types of input parameters should be sent to server
            bind_param_done : my_bool;      // input buffers were supplied
{$IFNDEF mysql50}
            bind_result_done : my_bool;     // output buffers were supplied
{$ELSE}
            bind_result_done : cuchar;      // output buffers were supplied
{$ENDIF}

            unbuffered_fetch_cancelled : my_bool;   // mysql_stmt_close() had to cancel this result
    {   Is set to true if we need to calculate field->max_length for
        metadata fields when doing mysql_stmt_store_result.       }
            update_max_length : my_bool;
{$IFDEF mysql51}
            extension: ^st_mysql_stmt_extension;
{$ENDIF}
         end;
       MYSQL_STMT = st_mysql_stmt;
    {   When doing mysql_stmt_store_result calculate max_length attribute
        of statement metadata. This is to be consistent with the old API,
        where this was done automatically.
        In the new API we do that only by request because it slows down
        mysql_stmt_store_result sufficiently.       }
       enum_stmt_attr_type = (STMT_ATTR_UPDATE_MAX_LENGTH
{$IFDEF mysql50}
                              ,STMT_ATTR_CURSOR_TYPE,  // unsigned long with combination of cursor flags (read only, for update, etc)
                              STMT_ATTR_PREFETCH_ROWS // Amount of rows to retrieve from server per one fetch if using cursors.
                                                      // Accepts unsigned long attribute in the range 1 - ulong_max
{$ENDIF}
                             );


//#define max_allowed_packet (*mysql_get_parameters()->p_max_allowed_packet)
//#define net_buffer_length (*mysql_get_parameters()->p_net_buffer_length)

{$IFNDEF LinkDynamically}
    { Set up and bring down the server; to ensure that applications will
      work when linked against either the standard client library or the
      embedded server library, these functions should be called.     }
    function mysql_server_init(argc:cint; argv:PPchar; groups:PPchar):cint;extdecl;external mysqllib name 'mysql_server_init';
    procedure mysql_server_end;extdecl;external mysqllib name 'mysql_server_end';

    { mysql_server_init/end need to be called when using libmysqld or
      libmysqlclient (exactly, mysql_server_init() is called by mysql_init() so
      you don't need to call it explicitely; but you need to call
      mysql_server_end() to free memory). The names are a bit misleading
      (mysql_SERVER* to be used when using libmysqlCLIENT). So we add more general
      names which suit well whether you're using libmysqld or libmysqlclient. We
      intend to promote these aliases over the mysql_server* ones.     }

    function mysql_library_init(argc:cint; argv:PPchar; groups:PPchar):cint;extdecl;external mysqllib name 'mysql_server_init';
    procedure mysql_library_end;extdecl;external mysqllib name 'mysql_server_end';

    function mysql_get_parameters:PMYSQL_PARAMETERS;extdecl;external mysqllib name 'mysql_get_parameters';

    { Set up and bring down a thread; these function should be called
      for each thread in an application which opens at least one MySQL
      connection.  All uses of the connection(s) should be between these
      function calls.     }
    function mysql_thread_init:my_bool;extdecl;external mysqllib name 'mysql_thread_init';
    procedure mysql_thread_end;extdecl;external mysqllib name 'mysql_thread_end';
    { Functions to get information from the MYSQL and MYSQL_RES structures
      Should definitely be used if one uses shared libraries.     }
    function mysql_num_rows(res:PMYSQL_RES):my_ulonglong;extdecl;external mysqllib name 'mysql_num_rows';
    function mysql_num_fields(res:PMYSQL_RES):cuint;extdecl;external mysqllib name 'mysql_num_fields';
    function mysql_eof(res:PMYSQL_RES):my_bool;extdecl;external mysqllib name 'mysql_eof';
    function mysql_fetch_field_direct(res:PMYSQL_RES; fieldnr:cuint):PMYSQL_FIELD;extdecl;external mysqllib name 'mysql_fetch_field_direct';
    function mysql_fetch_fields(res:PMYSQL_RES):PMYSQL_FIELD;extdecl;external mysqllib name 'mysql_fetch_fields';
    function mysql_row_tell(res:PMYSQL_RES):MYSQL_ROW_OFFSET;extdecl;external mysqllib name 'mysql_row_tell';
    function mysql_field_tell(res:PMYSQL_RES):MYSQL_FIELD_OFFSET;extdecl;external mysqllib name 'mysql_field_tell';
    function mysql_field_count(mysql:PMYSQL):cuint;extdecl;external mysqllib name 'mysql_field_count';
    function mysql_affected_rows(mysql:PMYSQL):my_ulonglong;extdecl;external mysqllib name 'mysql_affected_rows';
    function mysql_insert_id(mysql:PMYSQL):my_ulonglong;extdecl;external mysqllib name 'mysql_insert_id';
    function mysql_errno(mysql:PMYSQL):cuint;extdecl;external mysqllib name 'mysql_errno';
    function mysql_error(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_error';
    function mysql_sqlstate(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_sqlstate';
    function mysql_warning_count(mysql:PMYSQL):cuint;extdecl;external mysqllib name 'mysql_warning_count';
    function mysql_info(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_info';
    function mysql_thread_id(mysql:PMYSQL):culong;extdecl;external mysqllib name 'mysql_thread_id';
    function mysql_character_set_name(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_character_set_name';
    function mysql_set_character_set(mysql:PMYSQL; csname:Pchar):longint;extdecl;external mysqllib name 'mysql_set_character_set';
    function mysql_init(mysql:PMYSQL):PMYSQL;extdecl;external mysqllib name 'mysql_init';
    function mysql_ssl_set(mysql:PMYSQL; key:Pchar; cert:Pchar; ca:Pchar; capath:Pchar;
               cipher:Pchar):my_bool;extdecl;external mysqllib name 'mysql_ssl_set';
    function mysql_change_user(mysql:PMYSQL; user:Pchar; passwd:Pchar; db:Pchar):my_bool;extdecl;external mysqllib name 'mysql_change_user';
    function mysql_real_connect(mysql:PMYSQL; host:Pchar; user:Pchar; passwd:Pchar; db:Pchar;
               port:cuint; unix_socket:Pchar; clientflag:culong):PMYSQL;extdecl;external mysqllib name 'mysql_real_connect';
    function mysql_select_db(mysql:PMYSQL; db:Pchar):cint;extdecl;external mysqllib name 'mysql_select_db';
    function mysql_query(mysql:PMYSQL; q:Pchar):cint;extdecl;external mysqllib name 'mysql_query';
    function mysql_send_query(mysql:PMYSQL; q:Pchar; length:culong):cint;extdecl;external mysqllib name 'mysql_send_query';
    function mysql_real_query(mysql:PMYSQL; q:Pchar; length:culong):cint;extdecl;external mysqllib name 'mysql_real_query';
    function mysql_store_result(mysql:PMYSQL):PMYSQL_RES;extdecl;external mysqllib name 'mysql_store_result';
    function mysql_use_result(mysql:PMYSQL):PMYSQL_RES;extdecl;external mysqllib name 'mysql_use_result';
 {$IFDEF mysql50}
    procedure mysql_get_character_set_info(mysql:PMYSQL; charset:PMY_CHARSET_INFO);extdecl;external mysqllib name 'mysql_get_character_set_info';
  {$IFDEF mysql57}
    function mysql_session_track_get_first(mysql:PMYSQL; typ:enum_session_state_type; data:ppcchar; length:psize_t):cint; extdecl; external mysqllib name 'mysql_session_track_get_first';
    function mysql_session_track_get_next(mysql:PMYSQL; typ:enum_session_state_type; data:ppcchar; length:psize_t):cint; extdecl; external mysqllib name 'mysql_session_track_get_next';
  {$ENDIF}
 {$ENDIF}

{$ELSE}

    var
      mysql_server_init: function (argc:cint; argv:PPchar; groups:PPchar):cint;extdecl;
      mysql_server_end: procedure ();extdecl;
      mysql_library_init: function (argc:cint; argv:PPchar; groups:PPchar):cint;extdecl;
      mysql_library_end: procedure ();extdecl;
      mysql_num_rows: function (res:PMYSQL_RES):my_ulonglong;extdecl;
      mysql_num_fields: function (res:PMYSQL_RES):cuint;extdecl;
      mysql_eof: function (res:PMYSQL_RES):my_bool;extdecl;
      mysql_fetch_field_direct: function (res:PMYSQL_RES; fieldnr:cuint):PMYSQL_FIELD;extdecl;
      mysql_fetch_fields: function (res:PMYSQL_RES):PMYSQL_FIELD;extdecl;
      mysql_row_tell: function (res:PMYSQL_RES):MYSQL_ROW_OFFSET;extdecl;
      mysql_field_tell: function (res:PMYSQL_RES):MYSQL_FIELD_OFFSET;extdecl;
      mysql_field_count: function (mysql:PMYSQL):cuint;extdecl;
      mysql_affected_rows: function (mysql:PMYSQL):my_ulonglong;extdecl;
      mysql_insert_id: function (mysql:PMYSQL):my_ulonglong;extdecl;
      mysql_errno: function (mysql:PMYSQL):cuint;extdecl;
      mysql_error: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_sqlstate: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_warning_count: function (mysql:PMYSQL):cuint;extdecl;
      mysql_info: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_thread_id: function (mysql:PMYSQL):culong;extdecl;
      mysql_character_set_name: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_set_character_set: function (mysql:PMYSQL; csname:Pchar):cint;extdecl;
      mysql_init: function (mysql:PMYSQL):PMYSQL;extdecl;
      mysql_ssl_set: function (mysql:PMYSQL; key:Pchar; cert:Pchar; ca:Pchar; capath:Pchar;
                 cipher:Pchar):my_bool;extdecl;
      mysql_change_user: function (mysql:PMYSQL; user:Pchar; passwd:Pchar; db:Pchar):my_bool;extdecl;
      mysql_real_connect: function (mysql:PMYSQL; host:Pchar; user:Pchar; passwd:Pchar; db:Pchar;
                 port:cuint; unix_socket:Pchar; clientflag:culong):PMYSQL;extdecl;
      mysql_select_db: function (mysql:PMYSQL; db:Pchar):cint;extdecl;
      mysql_query: function (mysql:PMYSQL; q:Pchar):cint;extdecl;
      mysql_send_query: function (mysql:PMYSQL; q:Pchar; length:culong):cint;extdecl;
      mysql_real_query: function (mysql:PMYSQL; q:Pchar; length:culong):cint;extdecl;
      mysql_store_result: function (mysql:PMYSQL):PMYSQL_RES;extdecl;
      mysql_use_result: function (mysql:PMYSQL):PMYSQL_RES;extdecl;
 {$IFDEF mysql50}
      mysql_get_character_set_info: procedure(mysql:PMYSQL; charset:PMY_CHARSET_INFO);extdecl;
  {$IFDEF mysql57}
      mysql_session_track_get_first: function(mysql:PMYSQL; typ:enum_session_state_type; data:ppcchar; length:psize_t):cint; extdecl;
      mysql_session_track_get_next: function(mysql:PMYSQL; typ:enum_session_state_type; data:ppcchar; length:psize_t):cint; extdecl;
  {$ENDIF}
 {$ENDIF}

{$ENDIF}

{$IFNDEF LinkDynamically}
    { perform query on master  }
    function mysql_master_query(mysql:PMYSQL; q:Pchar; length:culong):my_bool;extdecl;external mysqllib name 'mysql_master_query';
    function mysql_master_send_query(mysql:PMYSQL; q:Pchar; length:culong):my_bool;extdecl;external mysqllib name 'mysql_master_send_query';

    { perform query on slave  }
    function mysql_slave_query(mysql:PMYSQL; q:Pchar; length:culong):my_bool;extdecl;external mysqllib name 'mysql_slave_query';
    function mysql_slave_send_query(mysql:PMYSQL; q:Pchar; length:culong):my_bool;extdecl;external mysqllib name 'mysql_slave_send_query';
{$ENDIF}

    { local infile support  }

    const
       LOCAL_INFILE_ERROR_LEN = 512;

{$IFNDEF LinkDynamically}
{    procedure mysql_set_local_infile_handler(mysql:PMYSQL; local_infile_init:function (_para1:Ppointer; _para2:Pchar; _para3:pointer):longint; local_infile_read:function (_para1:pointer; _para2:Pchar; _para3:dword):longint; local_infile_end:procedure (_pa
                _para6:pointer);cdecl;external mysqllib name 'mysql_set_local_infile_handler';}
    procedure mysql_set_local_infile_default(mysql:PMYSQL);cdecl;external mysqllib name 'mysql_set_local_infile_default';

    { enable/disable parsing of all queries to decide if they go on master or
      slave     }
    procedure mysql_enable_rpl_parse(mysql:PMYSQL);extdecl;external mysqllib name 'mysql_enable_rpl_parse';
    procedure mysql_disable_rpl_parse(mysql:PMYSQL);extdecl;external mysqllib name 'mysql_disable_rpl_parse';

    { get the value of the parse flag  }
    function mysql_rpl_parse_enabled(mysql:PMYSQL):cint;extdecl;external mysqllib name 'mysql_rpl_parse_enabled';

    {  enable/disable reads from master  }
    procedure mysql_enable_reads_from_master(mysql:PMYSQL);extdecl;external mysqllib name 'mysql_enable_reads_from_master';
    procedure mysql_disable_reads_from_master(mysql:PMYSQL);extdecl;external mysqllib name 'mysql_disable_reads_from_master';

    { get the value of the master read flag  }
    function mysql_reads_from_master_enabled(mysql:PMYSQL):my_bool;extdecl;external mysqllib name 'mysql_reads_from_master_enabled';

    function mysql_rpl_query_type(q : pchar;len : cint):mysql_rpl_type;extdecl;external mysqllib name 'mysql_rpl_query_type';

    { discover the master and its slaves  }
    function mysql_rpl_probe(mysql:PMYSQL):my_bool;extdecl;external mysqllib name 'mysql_rpl_probe';

    { set the master, close/free the old one, if it is not a pivot  }
    function mysql_set_master(mysql:PMYSQL; host:Pchar; port:cuint; user:Pchar; passwd:Pchar):cint;extdecl;external mysqllib name 'mysql_set_master';
    function mysql_add_slave(mysql:PMYSQL; host:Pchar; port:cuint; user:Pchar; passwd:Pchar):cint;extdecl;external mysqllib name 'mysql_add_slave';
    function mysql_shutdown(mysql:PMYSQL; shutdown_level:mysql_enum_shutdown_level):cint;extdecl;external mysqllib name 'mysql_shutdown';
    function mysql_dump_debug_info(mysql:PMYSQL):cint;extdecl;external mysqllib name 'mysql_dump_debug_info';
    function mysql_refresh(mysql:PMYSQL; refresh_options:cuint):cint;extdecl;external mysqllib name 'mysql_refresh';
    function mysql_kill(mysql:PMYSQL; pid:culong):cint;extdecl;external mysqllib name 'mysql_kill';
    function mysql_set_server_option(mysql:PMYSQL; option:enum_mysql_set_option):cint;extdecl;external mysqllib name 'mysql_set_server_option';
    function mysql_ping(mysql:PMYSQL):cint;extdecl;external mysqllib name 'mysql_ping';
    function mysql_stat(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_stat';
    function mysql_get_server_info(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_get_server_info';
    function mysql_get_client_info:Pchar;extdecl;external mysqllib name 'mysql_get_client_info';
    function mysql_get_client_version:culong;extdecl;external mysqllib name 'mysql_get_client_version';
    function mysql_get_host_info(mysql:PMYSQL):Pchar;extdecl;external mysqllib name 'mysql_get_host_info';
    function mysql_get_server_version(mysql:PMYSQL):culong;extdecl;external mysqllib name 'mysql_get_server_version';
    function mysql_get_proto_info(mysql:PMYSQL):cuint;extdecl;external mysqllib name 'mysql_get_proto_info';
    function mysql_list_dbs(mysql:PMYSQL; wild:Pchar):PMYSQL_RES;extdecl;external mysqllib name 'mysql_list_dbs';

    function mysql_list_tables(mysql:PMYSQL; wild:Pchar):PMYSQL_RES;extdecl;external mysqllib name 'mysql_list_tables';
    function mysql_list_processes(mysql:PMYSQL):PMYSQL_RES;extdecl;external mysqllib name 'mysql_list_processes';
    function mysql_options(mysql:PMYSQL; option:mysql_option; arg:Pchar):cint;extdecl;external mysqllib name 'mysql_options';
    procedure mysql_free_result(result:PMYSQL_RES);extdecl;external mysqllib name 'mysql_free_result';
    procedure mysql_data_seek(result:PMYSQL_RES; offset:my_ulonglong);extdecl;external mysqllib name 'mysql_data_seek';
    function mysql_row_seek(result:PMYSQL_RES; offset:MYSQL_ROW_OFFSET):MYSQL_ROW_OFFSET;extdecl;external mysqllib name 'mysql_row_seek';
    function mysql_field_seek(result:PMYSQL_RES; offset:MYSQL_FIELD_OFFSET):MYSQL_FIELD_OFFSET;extdecl;external mysqllib name 'mysql_field_seek';
    function mysql_fetch_row(result:PMYSQL_RES):MYSQL_ROW;extdecl;external mysqllib name 'mysql_fetch_row';
    function mysql_fetch_lengths(result:PMYSQL_RES):pculong;extdecl;external mysqllib name 'mysql_fetch_lengths';
    function mysql_fetch_field(result:PMYSQL_RES):PMYSQL_FIELD;extdecl;external mysqllib name 'mysql_fetch_field';
    function mysql_list_fields(mysql:PMYSQL; table:Pchar; wild:Pchar):PMYSQL_RES;extdecl;external mysqllib name 'mysql_list_fields';
    function mysql_escape_string(fto:Pchar; from:Pchar; from_length:culong):culong;extdecl;external mysqllib name 'mysql_escape_string';
    function mysql_hex_string(fto:Pchar; from:Pchar; from_length:culong):culong;extdecl;external mysqllib name 'mysql_hex_string';
    function mysql_real_escape_string(mysql:PMYSQL; fto:Pchar; from:Pchar; length:culong):culong;extdecl;external mysqllib name 'mysql_real_escape_string';
    procedure mysql_debug(debug:Pchar);extdecl;external mysqllib name 'mysql_debug';
{    function mysql_odbc_escape_string(mysql:PMYSQL; fto:Pchar; to_length:dword; from:Pchar; from_length:dword;
               param:pointer; extend_buffer:function (_para1:pointer; to:Pchar; length:Pdword):Pchar):Pchar;extdecl;external mysqllib name 'mysql_odbc_escape_string';}
    procedure myodbc_remove_escape(mysql:PMYSQL; name:Pchar);extdecl;external mysqllib name 'myodbc_remove_escape';
    function mysql_thread_safe:cuint;extdecl;external mysqllib name 'mysql_thread_safe';
    function mysql_embedded:my_bool;extdecl;external mysqllib name 'mysql_embedded';
    function mysql_manager_init(con:PMYSQL_MANAGER):PMYSQL_MANAGER;extdecl;external mysqllib name 'mysql_manager_init';
    function mysql_manager_connect(con:PMYSQL_MANAGER; host:Pchar; user:Pchar; passwd:Pchar; port:cuint):PMYSQL_MANAGER;extdecl;external mysqllib name 'mysql_manager_connect';
    procedure mysql_manager_close(con:PMYSQL_MANAGER);extdecl;external mysqllib name 'mysql_manager_close';
    function mysql_manager_command(con:PMYSQL_MANAGER; cmd:Pchar; cmd_len:cint):cint;extdecl;external mysqllib name 'mysql_manager_command';
    function mysql_manager_fetch_line(con:PMYSQL_MANAGER; res_buf:Pchar; res_buf_size:cint):cint;extdecl;external mysqllib name 'mysql_manager_fetch_line';
    function mysql_read_query_result(mysql:PMYSQL):my_bool;extdecl;external mysqllib name 'mysql_read_query_result';

    function mysql_stmt_init(mysql:PMYSQL):PMYSQL_STMT;extdecl;external mysqllib name 'mysql_stmt_init';
    function mysql_stmt_prepare(stmt:PMYSQL_STMT; query:Pchar; length:culong):cint;extdecl;external mysqllib name 'mysql_stmt_prepare';
    function mysql_stmt_execute(stmt:PMYSQL_STMT):cint;extdecl;external mysqllib name 'mysql_stmt_execute';
    function mysql_stmt_fetch(stmt:PMYSQL_STMT):cint;extdecl;external mysqllib name 'mysql_stmt_fetch';
    function mysql_stmt_fetch_column(stmt:PMYSQL_STMT; bind:PMYSQL_BIND; column:cuint; offset:culong):cint;extdecl;external mysqllib name 'mysql_stmt_fetch_column';
    function mysql_stmt_store_result(stmt:PMYSQL_STMT):cint;extdecl;external mysqllib name 'mysql_stmt_store_result';
    function mysql_stmt_param_count(stmt:PMYSQL_STMT):culong;extdecl;external mysqllib name 'mysql_stmt_param_count';
    function mysql_stmt_attr_set(stmt:PMYSQL_STMT; attr_type:enum_stmt_attr_type; attr:pointer):my_bool;extdecl;external mysqllib name 'mysql_stmt_attr_set';
    function mysql_stmt_attr_get(stmt:PMYSQL_STMT; attr_type:enum_stmt_attr_type; attr:pointer):my_bool;extdecl;external mysqllib name 'mysql_stmt_attr_get';
    function mysql_stmt_bind_param(stmt:PMYSQL_STMT; bnd:PMYSQL_BIND):my_bool;extdecl;external mysqllib name 'mysql_stmt_bind_param';
    function mysql_stmt_bind_result(stmt:PMYSQL_STMT; bnd:PMYSQL_BIND):my_bool;extdecl;external mysqllib name 'mysql_stmt_bind_result';
    function mysql_stmt_close(stmt:PMYSQL_STMT):my_bool;extdecl;external mysqllib name 'mysql_stmt_close';
    function mysql_stmt_reset(stmt:PMYSQL_STMT):my_bool;extdecl;external mysqllib name 'mysql_stmt_reset';
    function mysql_stmt_free_result(stmt:PMYSQL_STMT):my_bool;extdecl;external mysqllib name 'mysql_stmt_free_result';
    function mysql_stmt_send_long_data(stmt:PMYSQL_STMT; param_number:cuint; data:Pchar; length:culong):my_bool;extdecl;external mysqllib name 'mysql_stmt_send_long_data';
    function mysql_stmt_result_metadata(stmt:PMYSQL_STMT):PMYSQL_RES;extdecl;external mysqllib name 'mysql_stmt_result_metadata';
    function mysql_stmt_param_metadata(stmt:PMYSQL_STMT):PMYSQL_RES;extdecl;external mysqllib name 'mysql_stmt_param_metadata';
    function mysql_stmt_errno(stmt:PMYSQL_STMT):cuint;extdecl;external mysqllib name 'mysql_stmt_errno';
    function mysql_stmt_error(stmt:PMYSQL_STMT):Pchar;extdecl;external mysqllib name 'mysql_stmt_error';
    function mysql_stmt_sqlstate(stmt:PMYSQL_STMT):Pchar;extdecl;external mysqllib name 'mysql_stmt_sqlstate';
    function mysql_stmt_row_seek(stmt:PMYSQL_STMT; offset:MYSQL_ROW_OFFSET):MYSQL_ROW_OFFSET;extdecl;external mysqllib name 'mysql_stmt_row_seek';
    function mysql_stmt_row_tell(stmt:PMYSQL_STMT):MYSQL_ROW_OFFSET;extdecl;external mysqllib name 'mysql_stmt_row_tell';
    procedure mysql_stmt_data_seek(stmt:PMYSQL_STMT; offset:my_ulonglong);extdecl;external mysqllib name 'mysql_stmt_data_seek';
    function mysql_stmt_num_rows(stmt:PMYSQL_STMT):my_ulonglong;extdecl;external mysqllib name 'mysql_stmt_num_rows';
    function mysql_stmt_affected_rows(stmt:PMYSQL_STMT):my_ulonglong;extdecl;external mysqllib name 'mysql_stmt_affected_rows';
    function mysql_stmt_insert_id(stmt:PMYSQL_STMT):my_ulonglong;extdecl;external mysqllib name 'mysql_stmt_insert_id';
    function mysql_stmt_field_count(stmt:PMYSQL_STMT):cuint;extdecl;external mysqllib name 'mysql_stmt_field_count';

    function mysql_commit(mysql:PMYSQL):my_bool;extdecl;external mysqllib name 'mysql_commit';
    function mysql_rollback(mysql:PMYSQL):my_bool;extdecl;external mysqllib name 'mysql_rollback';
    function mysql_autocommit(mysql:PMYSQL; auto_mode:my_bool):my_bool;extdecl;external mysqllib name 'mysql_autocommit';
    function mysql_more_results(mysql:PMYSQL):my_bool;extdecl;external mysqllib name 'mysql_more_results';
    function mysql_next_result(mysql:PMYSQL):cint;extdecl;external mysqllib name 'mysql_next_result';
    function mysql_stmt_next_result(stmt:PMYSQL_STMT):cint;extdecl;external mysqllib name 'mysql_stmt_next_result';
    procedure mysql_close(sock:PMYSQL);extdecl;external mysqllib name 'mysql_close';

{$ELSE}
    var
      mysql_shutdown: function (mysql:PMYSQL; shutdown_level:mysql_enum_shutdown_level):cint;extdecl;
      mysql_dump_debug_info: function (mysql:PMYSQL):cint;extdecl;
      mysql_refresh: function (mysql:PMYSQL; refresh_options:cuint):cint;extdecl;
      mysql_kill: function (mysql:PMYSQL; pid:culong):cint;extdecl;
      mysql_set_server_option: function (mysql:PMYSQL; option:enum_mysql_set_option):cint;extdecl;
      mysql_ping: function (mysql:PMYSQL):cint;extdecl;
      mysql_stat: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_get_server_info: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_get_client_info: function :Pchar;extdecl;
      mysql_get_client_version: function :culong;extdecl;
      mysql_get_host_info: function (mysql:PMYSQL):Pchar;extdecl;
      mysql_get_server_version: function (mysql:PMYSQL):culong;extdecl;
      mysql_get_proto_info: function (mysql:PMYSQL):cuint;extdecl;
      mysql_list_dbs: function (mysql:PMYSQL; wild:Pchar):PMYSQL_RES;extdecl;

      mysql_list_tables: function (mysql:PMYSQL; wild:Pchar):PMYSQL_RES;extdecl;
      mysql_list_processes: function (mysql:PMYSQL):PMYSQL_RES;extdecl;
      mysql_options: function (mysql:PMYSQL; option:mysql_option; arg:Pchar):cint;extdecl;
{$IFDEF mysql56}
      mysql_options4: function (mysql:PMYSQL; option:mysql_option; arg1,arg2:Pointer):cint;extdecl;
 {$IFDEF mysql57}
      mysql_get_option: function (mysql:PMYSQL; option:mysql_option; arg:Pointer):cint;extdecl;
 {$ENDIF}
{$ENDIF}
      mysql_free_result: procedure (result:PMYSQL_RES);extdecl;
      mysql_data_seek: procedure (result:PMYSQL_RES; offset:my_ulonglong);extdecl;
      mysql_row_seek: function (result:PMYSQL_RES; offset:MYSQL_ROW_OFFSET):MYSQL_ROW_OFFSET;extdecl;
      mysql_field_seek: function (result:PMYSQL_RES; offset:MYSQL_FIELD_OFFSET):MYSQL_FIELD_OFFSET;extdecl;
      mysql_fetch_row: function (result:PMYSQL_RES):MYSQL_ROW;extdecl;
      mysql_fetch_lengths: function (result:PMYSQL_RES):pculong;extdecl;
      mysql_fetch_field: function (result:PMYSQL_RES):PMYSQL_FIELD;extdecl;
      mysql_list_fields: function (mysql:PMYSQL; table:Pchar; wild:Pchar):PMYSQL_RES;extdecl;
      mysql_escape_string: function (fto:Pchar; from:Pchar; from_length:culong):culong;extdecl;
      mysql_hex_string: function (fto:Pchar; from:Pchar; from_length:culong):culong;extdecl;
      mysql_real_escape_string: function (mysql:PMYSQL; fto:Pchar; from:Pchar; length:culong):culong;extdecl;
{$IFDEF mysql57}
      mysql_real_escape_string_quote: function(mysql:PMYSQL; fto:pcchar; from:pcchar; length:culong; quote: cchar):culong;extdecl;
      mysql_reset_connection: function(mysql:PMYSQL):cint;extdecl;
{$ENDIF}
      mysql_debug: procedure (debug:Pchar);extdecl;

      mysql_rollback: function (mysql:PMYSQL):my_bool;extdecl;
      mysql_autocommit: function (mysql:PMYSQL; auto_mode:my_bool):my_bool;extdecl;
      mysql_commit: function (mysql:PMYSQL):my_bool;extdecl;
      mysql_more_results: function (mysql:PMYSQL):my_bool;extdecl;
      mysql_next_result: function (mysql:PMYSQL):cint;extdecl;
      mysql_close: procedure (sock:PMYSQL);extdecl;

      mysql_stmt_init: function (mysql:PMYSQL):PMYSQL_STMT;extdecl;
      mysql_stmt_prepare: function (stmt:PMYSQL_STMT; query:Pchar; length:culong):cint;extdecl;
      mysql_stmt_execute: function (stmt:PMYSQL_STMT):cint;extdecl;
      mysql_stmt_fetch: function (stmt:PMYSQL_STMT):cint;extdecl;
      mysql_stmt_fetch_column: function (stmt:PMYSQL_STMT; bind:PMYSQL_BIND; column:cuint; offset:culong):cint;extdecl;
      mysql_stmt_store_result: function (stmt:PMYSQL_STMT):cint;extdecl;
      mysql_stmt_param_count: function (stmt:PMYSQL_STMT):culong;extdecl;
      mysql_stmt_attr_set: function (stmt:PMYSQL_STMT; attr_type:enum_stmt_attr_type; attr:pointer):my_bool;extdecl;
      mysql_stmt_attr_get: function (stmt:PMYSQL_STMT; attr_type:enum_stmt_attr_type; attr:pointer):my_bool;extdecl;
      mysql_stmt_bind_param: function (stmt:PMYSQL_STMT; bnd:PMYSQL_BIND):my_bool;extdecl;
      mysql_stmt_bind_result: function (stmt:PMYSQL_STMT; bnd:PMYSQL_BIND):my_bool;extdecl;
      mysql_stmt_close: function (stmt:PMYSQL_STMT):my_bool;extdecl;
      mysql_stmt_reset: function (stmt:PMYSQL_STMT):my_bool;extdecl;
      mysql_stmt_free_result: function (stmt:PMYSQL_STMT):my_bool;extdecl;
      mysql_stmt_send_long_data: function (stmt:PMYSQL_STMT; param_number:cuint; data:Pchar; length:culong):my_bool;extdecl;
      mysql_stmt_result_metadata: function (stmt:PMYSQL_STMT):PMYSQL_RES;extdecl;
      mysql_stmt_param_metadata: function (stmt:PMYSQL_STMT):PMYSQL_RES;extdecl;
      mysql_stmt_errno: function (stmt:PMYSQL_STMT):cuint;extdecl;
      mysql_stmt_error: function (stmt:PMYSQL_STMT):Pchar;extdecl;
      mysql_stmt_sqlstate: function (stmt:PMYSQL_STMT):Pchar;extdecl;
      mysql_stmt_row_seek: function (stmt:PMYSQL_STMT; offset:MYSQL_ROW_OFFSET):MYSQL_ROW_OFFSET;extdecl;
      mysql_stmt_row_tell: function (stmt:PMYSQL_STMT):MYSQL_ROW_OFFSET;extdecl;
      mysql_stmt_data_seek: procedure (stmt:PMYSQL_STMT; offset:my_ulonglong);extdecl;
      mysql_stmt_num_rows: function (stmt:PMYSQL_STMT):my_ulonglong;extdecl;
      mysql_stmt_affected_rows: function (stmt:PMYSQL_STMT):my_ulonglong;extdecl;
      mysql_stmt_insert_id: function (stmt:PMYSQL_STMT):my_ulonglong;extdecl;
      mysql_stmt_field_count: function (stmt:PMYSQL_STMT):cuint;extdecl;
      mysql_stmt_next_result: function (stmt:PMYSQL_STMT):cint;extdecl;

{$ENDIF}


    { status return codes  }

    const
       MYSQL_NO_DATA = 100;
       MYSQL_DATA_TRUNCATED  = 101;

    function mysql_reload(mysql : PMySQL) : cint;

{$define HAVE_MYSQL_REAL_CONNECT}

    { The following functions are mainly exported because of mysqlbinlog;
      They are not for general usage     }

    function simple_command(mysql,command,arg,length,skip_check : cint) : cint;
{$IFNDEF LinkDynamically}
    function net_safe_read(mysql:PMYSQL):cuint;cdecl;external mysqllib name 'net_safe_read';
{$ENDIF}


{$IFDEF LinkDynamically}
Function InitialiseMysql(Const LibraryName : AnsiString) : Integer;
Function InitialiseMysql(Const LibraryName : AnsiString; argc: cint; argv:PPchar = Nil; groups:PPchar = nil) : Integer;
Function InitialiseMysql(argc:cint = -1; argv:PPchar = nil; groups:PPchar = nil) : Integer;
Procedure ReleaseMySQL;
Function DoReleaseMysql : Integer;

var MysqlLibraryHandle : TLibHandle;
  MysqlLoadedLibrary : String;
{$ENDIF}

implementation

{$IFDEF LinkDynamically}

ResourceString
  SErrAlreadyLoaded  = 'MySQL interface already initialized from library %s.';
  SErrLoadFailed     = 'Can not load MySQL library "%s". Please check your installation.';
  SErrDefaultsFailed = 'Can not load default MySQL library ("%s" or "%s"). Check your installation.';

var 
  RefCount : integer;

Function TryInitialiseMysql(Const LibraryName: AnsiString; argc: cint; argv: PPchar; groups: PPchar) : Integer;

begin
  Result := 0;
  if (RefCount=0) then
    begin
    MysqlLibraryHandle := loadlibrary(LibraryName);
    if (MysqlLibraryHandle=nilhandle) then
      Exit;
    Inc(RefCount);
    MysqlLoadedLibrary:=LibraryName;

// Only the procedure that are given in the c-library documentation are loaded, to
// avoid problems with 'incomplete' libraries
    pointer(my_init) := GetProcedureAddress(MysqlLibraryHandle,'my_init');
    pointer(my_thread_init) := GetProcedureAddress(MysqlLibraryHandle,'my_thread_init');
    pointer(my_thread_end) := GetProcedureAddress(MysqlLibraryHandle,'my_thread_end');

    pointer(mysql_affected_rows) := GetProcedureAddress(MysqlLibraryHandle,'mysql_affected_rows');
    pointer(mysql_autocommit) := GetProcedureAddress(MysqlLibraryHandle,'mysql_autocommit');
    pointer(mysql_change_user) := GetProcedureAddress(MysqlLibraryHandle,'mysql_change_user');
    pointer(mysql_character_set_name) := GetProcedureAddress(MysqlLibraryHandle,'mysql_character_set_name');
    pointer(mysql_close) := GetProcedureAddress(MysqlLibraryHandle,'mysql_close');
    pointer(mysql_commit) := GetProcedureAddress(MysqlLibraryHandle,'mysql_commit');
    pointer(mysql_data_seek) := GetProcedureAddress(MysqlLibraryHandle,'mysql_data_seek');
    pointer(mysql_debug) := GetProcedureAddress(MysqlLibraryHandle,'mysql_debug');
    pointer(mysql_dump_debug_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_dump_debug_info');
    pointer(mysql_eof) := GetProcedureAddress(MysqlLibraryHandle,'mysql_eof');
    pointer(mysql_errno) := GetProcedureAddress(MysqlLibraryHandle,'mysql_errno');
    pointer(mysql_error) := GetProcedureAddress(MysqlLibraryHandle,'mysql_error');
    pointer(mysql_escape_string) := GetProcedureAddress(MysqlLibraryHandle,'mysql_escape_string');
    pointer(mysql_fetch_field) := GetProcedureAddress(MysqlLibraryHandle,'mysql_fetch_field');
    pointer(mysql_fetch_field_direct) := GetProcedureAddress(MysqlLibraryHandle,'mysql_fetch_field_direct');
    pointer(mysql_fetch_fields) := GetProcedureAddress(MysqlLibraryHandle,'mysql_fetch_fields');
    pointer(mysql_fetch_lengths) := GetProcedureAddress(MysqlLibraryHandle,'mysql_fetch_lengths');
    pointer(mysql_fetch_row) := GetProcedureAddress(MysqlLibraryHandle,'mysql_fetch_row');
    pointer(mysql_field_seek) := GetProcedureAddress(MysqlLibraryHandle,'mysql_field_seek');
    pointer(mysql_field_count) := GetProcedureAddress(MysqlLibraryHandle,'mysql_field_count');
    pointer(mysql_field_tell) := GetProcedureAddress(MysqlLibraryHandle,'mysql_field_tell');
    pointer(mysql_free_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_free_result');
{$IFDEF mysql50}
    pointer(mysql_get_character_set_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_character_set_info');
{$ENDIF}
    pointer(mysql_get_client_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_client_info');
    pointer(mysql_get_client_version) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_client_version');
    pointer(mysql_get_host_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_host_info');
    pointer(mysql_get_server_version) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_server_version');
    pointer(mysql_get_proto_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_proto_info');
    pointer(mysql_get_server_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_get_server_info');
    pointer(mysql_info) := GetProcedureAddress(MysqlLibraryHandle,'mysql_info');
    pointer(mysql_init) := GetProcedureAddress(MysqlLibraryHandle,'mysql_init');
    pointer(mysql_insert_id) := GetProcedureAddress(MysqlLibraryHandle,'mysql_insert_id');
    pointer(mysql_kill) := GetProcedureAddress(MysqlLibraryHandle,'mysql_kill');
    pointer(mysql_library_end) := GetProcedureAddress(MysqlLibraryHandle,'mysql_server_end');
    pointer(mysql_library_init) := GetProcedureAddress(MysqlLibraryHandle,'mysql_server_init');
    pointer(mysql_list_dbs) := GetProcedureAddress(MysqlLibraryHandle,'mysql_list_dbs');
    pointer(mysql_list_fields) := GetProcedureAddress(MysqlLibraryHandle,'mysql_list_fields');
    pointer(mysql_list_processes) := GetProcedureAddress(MysqlLibraryHandle,'mysql_list_processes');
    pointer(mysql_list_tables) := GetProcedureAddress(MysqlLibraryHandle,'mysql_list_tables');
    pointer(mysql_more_results) := GetProcedureAddress(MysqlLibraryHandle,'mysql_more_results');
    pointer(mysql_next_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_next_result');
    pointer(mysql_num_fields) := GetProcedureAddress(MysqlLibraryHandle,'mysql_num_fields');
    pointer(mysql_num_rows) := GetProcedureAddress(MysqlLibraryHandle,'mysql_num_rows');
    pointer(mysql_options) := GetProcedureAddress(MysqlLibraryHandle,'mysql_options');
    pointer(mysql_ping) := GetProcedureAddress(MysqlLibraryHandle,'mysql_ping');
    pointer(mysql_query) := GetProcedureAddress(MysqlLibraryHandle,'mysql_query');
    pointer(mysql_real_connect) := GetProcedureAddress(MysqlLibraryHandle,'mysql_real_connect');
    pointer(mysql_real_escape_string) := GetProcedureAddress(MysqlLibraryHandle,'mysql_real_escape_string');
{$IFDEF mysql57}
    pointer(mysql_real_escape_string_quote) := GetProcedureAddress(MysqlLibraryHandle,'mysql_real_escape_string_quote');
{$ENDIF}
    pointer(mysql_real_query) := GetProcedureAddress(MysqlLibraryHandle,'mysql_real_query');
    pointer(mysql_refresh) := GetProcedureAddress(MysqlLibraryHandle,'mysql_refresh');
{$IFDEF mysql57}
    pointer(mysql_reset_connection) := GetProcedureAddress(MysqlLibraryHandle,'mysql_reset_connection');
{$ENDIF}
    pointer(mysql_rollback) := GetProcedureAddress(MysqlLibraryHandle,'mysql_rollback');
    pointer(mysql_row_seek) := GetProcedureAddress(MysqlLibraryHandle,'mysql_row_seek');
    pointer(mysql_row_tell) := GetProcedureAddress(MysqlLibraryHandle,'mysql_row_tell');
    pointer(mysql_select_db) := GetProcedureAddress(MysqlLibraryHandle,'mysql_select_db');
    pointer(mysql_server_end) := GetProcedureAddress(MysqlLibraryHandle,'mysql_server_end');
    pointer(mysql_server_init) := GetProcedureAddress(MysqlLibraryHandle,'mysql_server_init');
    pointer(mysql_set_character_set) := GetProcedureAddress(MysqlLibraryHandle,'mysql_set_character_set');
    pointer(mysql_set_server_option) := GetProcedureAddress(MysqlLibraryHandle,'mysql_set_server_option');
    pointer(mysql_sqlstate) := GetProcedureAddress(MysqlLibraryHandle,'mysql_sqlstate');
    pointer(mysql_shutdown) := GetProcedureAddress(MysqlLibraryHandle,'mysql_shutdown');
    pointer(mysql_stat) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stat');
    pointer(mysql_store_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_store_result');
    pointer(mysql_thread_id) := GetProcedureAddress(MysqlLibraryHandle,'mysql_thread_id');
    pointer(mysql_use_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_use_result');
    pointer(mysql_warning_count) := GetProcedureAddress(MysqlLibraryHandle,'mysql_warning_count');
    pointer(mysql_stmt_init) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_init');
    pointer(mysql_stmt_prepare) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_prepare');
    pointer(mysql_stmt_execute) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_execute');
    pointer(mysql_stmt_fetch) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_fetch');
    pointer(mysql_stmt_fetch_column) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_fetch_column');
    pointer(mysql_stmt_store_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_store_result');
    pointer(mysql_stmt_param_count) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_param_count');
    pointer(mysql_stmt_attr_set) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_attr_set');
    pointer(mysql_stmt_attr_get) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_attr_get');
    pointer(mysql_stmt_bind_param) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_bind_param');
    pointer(mysql_stmt_bind_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_bind_result');
    pointer(mysql_stmt_close) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_close');
    pointer(mysql_stmt_reset) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_reset');
    pointer(mysql_stmt_free_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_free_result');
    pointer(mysql_stmt_send_long_data) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_send_long_data');
    pointer(mysql_stmt_result_metadata) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_result_metadata');
    pointer(mysql_stmt_param_metadata) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_param_metadata');
    pointer(mysql_stmt_errno) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_errno');
    pointer(mysql_stmt_error) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_error');
    pointer(mysql_stmt_sqlstate) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_sqlstate');
    pointer(mysql_stmt_row_seek) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_row_seek');
    pointer(mysql_stmt_row_tell) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_row_tell');
    pointer(mysql_stmt_data_seek) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_data_seek');
    pointer(mysql_stmt_num_rows) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_num_rows');
    pointer(mysql_stmt_affected_rows) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_affected_rows');
    pointer(mysql_stmt_insert_id) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_insert_id');
    pointer(mysql_stmt_field_count) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_field_count');
    pointer(mysql_stmt_next_result) := GetProcedureAddress(MysqlLibraryHandle,'mysql_stmt_next_result');

    if mysql_library_init(argc, argv, groups) <> 0 then
      Exit;
    end
  else
    inc(RefCount);

  Result:=RefCount;
end;



Function InitialiseMysql(argc: cint; argv: PPchar; groups: PPchar) : Integer;

begin
  Result := 0;
  if (RefCount<>0) then
    // pretend to load whatever is already loaded, so we do not get a library name conflict.
    Inc(RefCount)
  else
    If (TryInitialiseMysql(mysqllib,argc,argv,groups)=0)
       and (TryInitialiseMysql(mysqlvlib,argc,argv,groups)=0) then
      Raise EInOutError.CreateFmt(SErrDefaultsFailed,[mysqlvlib,mysqllib]);
  Result := RefCount;
end;

Function InitialiseMysql(Const LibraryName: AnsiString) : Integer;

begin
  Result:=InitialiseMySQL(LibraryName,-1,Nil,Nil);
end;

Function InitialiseMysql(Const LibraryName: AnsiString; argc: cint; argv: PPchar; groups:PPchar) : Integer;

begin
  Result := TryInitialiseMysql(LibraryName,argc,argv,groups);
  If Result = 0 then
    Raise EInOutError.CreateFmt(SErrLoadFailed,[LibraryName])
  else If (LibraryName<>MysqlLoadedLibrary) then
    begin
    Dec(RefCount);
    Result := RefCount;
    Raise EInOUtError.CreateFmt(SErrAlreadyLoaded,[MysqlLoadedLibrary]);
    end;
end;

Procedure ReleaseMySQL;

begin
  DoReleaseMysql;
end;

Function DoReleaseMysql : Integer;

begin
  if RefCount> 1 then
    Dec(RefCount)
  else if RefCount = 1 then
    begin
    mysql_library_end;
    if UnloadLibrary(MysqlLibraryHandle) then
      begin
      Dec(RefCount);
      MysqlLibraryHandle := NilHandle;
      MysqlLoadedLibrary:='';
      end
    end;
  Result:=RefCount;
end;

{$ENDIF}
    function net_new_transaction(net : st_net) : st_net;
    begin
      net.pkt_nr := 0;
      result := net;
    end;

    function IS_PRI_KEY(n : longint) : boolean;
    begin
      IS_PRI_KEY:=(n and PRI_KEY_FLAG)<>0;
    end;

    function IS_NOT_NULL(n : longint) : boolean;
    begin
     IS_NOT_NULL:=(n and NOT_NULL_FLAG)<>0;
    end;

    function IS_BLOB(n : longint) : boolean;
    begin
     IS_BLOB:=(n and BLOB_FLAG)<>0;
    end;

    function IS_NUM_FIELD(f : pst_mysql_field) : boolean;
    begin
       IS_NUM_FIELD:=((f^.flags) and NUM_FLAG)<>0;
    end;

    function IS_NUM(t : enum_field_types) : boolean;
    begin
{$IF DEFINED(mysql55)}
      IS_NUM := ((t <= FIELD_TYPE_INT24) and (t<>FIELD_TYPE_TIMESTAMP)) or (t=FIELD_TYPE_YEAR) or (t=FIELD_TYPE_NEWDECIMAL);
{$ELSEIF DEFINED(mysql50) or DEFINED(mysql51)}
      IS_NUM := (t <= FIELD_TYPE_INT24) or (t=FIELD_TYPE_YEAR) or (t=FIELD_TYPE_NEWDECIMAL);
{$ELSE}
      IS_NUM := (t <= FIELD_TYPE_INT24) or (t=FIELD_TYPE_YEAR);
{$ENDIF}
    end;

    function INTERNAL_NUM_FIELD(f : Pst_mysql_field) : boolean;
    begin
      INTERNAL_NUM_FIELD := (f^.ftype <= FIELD_TYPE_INT24) and ((f^.ftype <> FIELD_TYPE_TIMESTAMP)
      or (f^.length = 14) or (f^.length=8)) or (f^.ftype=FIELD_TYPE_YEAR);
    end;

    function mysql_reload(mysql : PMySQL) : cint;
    begin
      mysql_reload:=mysql_refresh(mysql,REFRESH_GRANT);
    end;

    function simple_command(mysql,command,arg,length,skip_check : longint) : longint;
    begin
      //simple_command:=mysql^.(methods^.advanced_command)(mysqlcommandNullS0arglengthskip_check);
      result := -1;
    end;

{$IFDEF LinkDynamically}
initialization
  Refcount := 0;
{$ENDIF}
end.
