{
  Contains the Postgres protocol 3 functions calls

  Call InitialisePostgres3 before using any of the calls, and call ReleasePostgres3
  when finished.
}

unit postgres3dyn;

{$mode objfpc}{$H+}

interface

uses
  dynlibs, SysUtils, dllistdyn, ctypes;

{$IFDEF Unix}
  const
{$ifdef darwin}
    pqlib = 'libpq.'+sharedsuffix; // No version number.
{$else}
    pqlib5 = 'libpq.'+sharedsuffix+'.5'; // 8.2 and higher
    pqlib4 = 'libpq.'+sharedsuffix+'.4'; // 8.0, 8.1
    pqlib3 = 'libpq.'+sharedsuffix+'.3'; // 7.3, 7.4
    pqlib  = pqlib5;
{$endif}
{$ENDIF}
{$IFDEF Windows}
  const
    pqlib = 'libpq.dll'; // Not sure if it has a version number ?
{$ENDIF}


{$PACKRECORDS C}

{$i postgres3types.inc}

var
{ ----------------
* Exported functions of libpq
* ----------------
}
{ ===   in fe-connect.c ===  }
{ make a new client connection to the backend  }
{ Asynchronous (non-blocking)  }
(* Const before type ignored *)
  PQconnectStart : function (conninfo:Pchar):PPGconn;cdecl;
  PQconnectPoll : function (conn:PPGconn):PostgresPollingStatusType;cdecl;
{ Synchronous (blocking)  }
(* Const before type ignored *)
  PQconnectdb : function (conninfo:Pchar):PPGconn;cdecl;
  PQsetdbLogin : function (pghost:Pchar; pgport:Pchar; pgoptions:Pchar; pgtty:Pchar; dbName:Pchar;login:Pchar; pwd:Pchar):PPGconn;cdecl;
{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
{ close the current connection and free the PGconn data structure  }
  PQfinish : procedure (conn:PPGconn);cdecl;
{ get info about connection options known to PQconnectdb  }
  PQconndefaults : function : PPQconninfoOption;cdecl;
{ free the data structure returned by PQconndefaults()  }
  PQconninfoFree : procedure (connOptions:PPQconninfoOption);cdecl;
{
* close the current connection and restablish a new one with the same
* parameters
}
{ Asynchronous (non-blocking)  }
  PQresetStart : function (conn:PPGconn):longint;cdecl;
  PQresetPoll : function (conn:PPGconn):PostgresPollingStatusType;cdecl;
{ Synchronous (blocking)  }
  PQreset : procedure (conn:PPGconn);cdecl;
{ issue a cancel request  }
  PQrequestCancel : function (conn:PPGconn):longint;cdecl;
{ Accessor functions for PGconn objects  }
  PQdb : function (conn:PPGconn):Pchar;cdecl;
  PQuser : function (conn:PPGconn):Pchar;cdecl;
  PQpass : function (conn:PPGconn):Pchar;cdecl;
  PQhost : function (conn:PPGconn):Pchar;cdecl;
  PQport : function (conn:PPGconn):Pchar;cdecl;
  PQtty : function (conn:PPGconn):Pchar;cdecl;
  PQoptions : function (conn:PPGconn):Pchar;cdecl;
  PQstatus : function (conn:PPGconn):TConnStatusType;cdecl;
  PQtransactionStatus : function (conn:PPGconn):PGTransactionStatusType;cdecl;
  PQparameterStatus : function (conn:PPGconn; paramName:Pchar):Pchar;cdecl;
  PQprotocolVersion : function (conn:PPGconn):longint;cdecl;
  PQserverVersion : function (conn:PPGconn):longint;cdecl;
  PQerrorMessage : function (conn:PPGconn):Pchar;cdecl;
  PQsocket : function (conn:PPGconn):longint;cdecl;
  PQbackendPID : function (conn:PPGconn):longint;cdecl;
  PQclientEncoding : function (conn:PPGconn):longint;cdecl;
  PQsetClientEncoding : function (conn:PPGconn; encoding:Pchar):longint;cdecl;
{$ifdef USE_SSL}
{ Get the SSL structure associated with a connection  }
  PQgetssl : function (conn:PPGconn):PSSL;cdecl;
{$endif}
{ Set verbosity for PQerrorMessage and PQresultErrorMessage  }
  PQsetErrorVerbosity : function (conn:PPGconn; verbosity:PGVerbosity):PGVerbosity;cdecl;
{ Enable/disable tracing  }
  PQtrace : procedure (conn:PPGconn; debug_port:PFILE);cdecl;
  PQuntrace : procedure (conn:PPGconn);cdecl;
{ Override default notice handling routines  }
  PQsetNoticeReceiver : function (conn:PPGconn; proc:PQnoticeReceiver; arg:pointer):PQnoticeReceiver;cdecl;
  PQsetNoticeProcessor : function (conn:PPGconn; proc:PQnoticeProcessor; arg:pointer):PQnoticeProcessor;cdecl;
{ === in fe-exec.c ===  }
{ Simple synchronous query  }
  PQexec : function (conn:PPGconn; query:Pchar):PPGresult;cdecl;
  PQexecParams : function (conn:PPGconn; command:Pchar; nParams:longint; paramTypes:POid; paramValues:PPchar;paramLengths:Plongint; paramFormats:Plongint; resultFormat:longint):PPGresult;cdecl;
  PQexecPrepared : function (conn:PPGconn; stmtName:Pchar; nParams:longint; paramValues:PPchar; paramLengths:Plongint;paramFormats:Plongint; resultFormat:longint):PPGresult;cdecl;
  PQPrepare : function (conn:PPGconn; stmtName:Pchar; query:Pchar; nParams:longint; paramTypes:POid):PPGresult;cdecl;
  PQdescribePrepared : function (conn:PPGconn; stmtName:Pchar):PPGresult;cdecl;
{ Interface for multiple-result or asynchronous queries  }
  PQsendQuery : function (conn:PPGconn; query:Pchar):longint;cdecl;
  PQsendQueryParams : function (conn:PPGconn; command:Pchar; nParams:longint; paramTypes:POid; paramValues:PPchar;paramLengths:Plongint; paramFormats:Plongint; resultFormat:longint):longint;cdecl;
  PQsendQueryPrepared : function (conn:PPGconn; stmtName:Pchar; nParams:longint; paramValues:PPchar; paramLengths:Plongint;paramFormats:Plongint; resultFormat:longint):longint;cdecl;
  PQgetResult : function (conn:PPGconn):PPGresult;cdecl;
{ Routines for managing an asynchronous query  }
  PQisBusy : function (conn:PPGconn):longint;cdecl;
  PQconsumeInput : function (conn:PPGconn):longint;cdecl;
{ LISTEN/NOTIFY support  }
  PQnotifies : function (conn:PPGconn):PPGnotify;cdecl;
{ Routines for copy in/out  }
  PQputCopyData : function (conn:PPGconn; buffer:Pchar; nbytes:longint):longint;cdecl;
  PQputCopyEnd : function (conn:PPGconn; errormsg:Pchar):longint;cdecl;
  PQgetCopyData : function (conn:PPGconn; buffer:PPchar; async:longint):longint;cdecl;
{ Deprecated routines for copy in/out  }
  PQgetline : function (conn:PPGconn; _string:Pchar; length:longint):longint;cdecl;
  PQputline : function (conn:PPGconn; _string:Pchar):longint;cdecl;
  PQgetlineAsync : function (conn:PPGconn; buffer:Pchar; bufsize:longint):longint;cdecl;
  PQputnbytes : function (conn:PPGconn; buffer:Pchar; nbytes:longint):longint;cdecl;
  PQendcopy : function (conn:PPGconn):longint;cdecl;
{ Set blocking/nonblocking connection to the backend  }
  PQsetnonblocking : function (conn:PPGconn; arg:longint):longint;cdecl;
  PQisnonblocking : function (conn:PPGconn):longint;cdecl;
  PQisthreadsafe : function ():longint;cdecl;

{ Force the write buffer to be written (or at least try)  }
  PQflush : function (conn:PPGconn):longint;cdecl;
{
* "Fast path" interface --- not really recommended for application
* use
}
  PQfn : function (conn:PPGconn; fnid:longint; result_buf:Plongint; result_len:Plongint; result_is_int:longint;args:PPQArgBlock; nargs:longint):PPGresult;cdecl;
{ Accessor functions for PGresult objects  }
  PQresultStatus : function (res:PPGresult):TExecStatusType;cdecl;
  PQresStatus : function (status:TExecStatusType):Pchar;cdecl;
  PQresultErrorMessage : function (res:PPGresult):Pchar;cdecl;
  PQresultErrorField : function (res:PPGresult; fieldcode:longint):Pchar;cdecl;
  PQntuples : function (res:PPGresult):longint;cdecl;
  PQnfields : function (res:PPGresult):longint;cdecl;
  PQbinaryTuples : function (res:PPGresult):longint;cdecl;
  PQfname : function (res:PPGresult; field_num:longint):Pchar;cdecl;
  PQfnumber : function (res:PPGresult; field_name:Pchar):longint;cdecl;
  PQftable : function (res:PPGresult; field_num:longint):Oid;cdecl;
  PQftablecol : function (res:PPGresult; field_num:longint):longint;cdecl;
  PQfformat : function (res:PPGresult; field_num:longint):longint;cdecl;
  PQftype : function (res:PPGresult; field_num:longint):Oid;cdecl;
  PQfsize : function (res:PPGresult; field_num:longint):longint;cdecl;
  PQfmod : function (res:PPGresult; field_num:longint):longint;cdecl;
  PQcmdStatus : function (res:PPGresult):Pchar;cdecl;
  PQoidStatus : function (res:PPGresult):Pchar;cdecl;
{ old and ugly  }
  PQoidValue : function (res:PPGresult):Oid;cdecl;
{ new and improved  }
  PQcmdTuples : function (res:PPGresult):Pchar;cdecl;
  PQgetvalue : function (res:PPGresult; tup_num:longint; field_num:longint):Pchar;cdecl;
  PQgetlength : function (res:PPGresult; tup_num:longint; field_num:longint):longint;cdecl;
  PQgetisnull : function (res:PPGresult; tup_num:longint; field_num:longint):longint;cdecl;
  PQnparams : function (res:PPGresult):longint;cdecl;
  PQparamtype : function (res:PPGresult; param_num:longint):Oid;cdecl;
{ Delete a PGresult  }
  PQclear : procedure (res:PPGresult);cdecl;
{ For freeing other alloc'd results, such as PGnotify structs  }
  PQfreemem : procedure (ptr:pointer);cdecl;
{ Exists for backward compatibility.  bjm 2003-03-24  }
{ was #define dname(params) para_def_expr }
{ argument types are unknown }
{ return type might be wrong }
//  function PQfreeNotify(ptr : longint) : longint;
{
* Make an empty PGresult with given status (some apps find this
* useful). If conn is not NULL and status indicates an error, the
* conn's errorMessage is copied.
}
  PQmakeEmptyPGresult : function (conn:PPGconn; status:TExecStatusType):PPGresult;cdecl;
{ Quoting strings before inclusion in queries.  }
  PQescapeStringConn : function (conn:PPGconn; str:Pcchar; from:Pcchar; length:size_t; error:Pcint):size_t;cdecl;
  PQescapeLiteral : function (conn:PPGconn; str:Pcchar; len:size_t):Pcchar;cdecl;
  PQescapeIdentifier : function (conn:PPGconn; str:Pcchar; len:size_t):Pcchar;cdecl;
  PQescapeByteaConn : function (conn:PPGconn; from:Pbyte; from_length:size_t; to_length:Psize_t):Pbyte;cdecl;
  PQunescapeBytea : function (strtext:Pbyte; retbuflen:Psize_t):Pbyte;cdecl;
{ These forms are deprecated! }
  PQescapeString : function (till:Pchar; from:Pchar; length:size_t):size_t;cdecl;
  PQescapeBytea : function (bintext:Pbyte; binlen:size_t; bytealen:Psize_t):Pbyte;cdecl;

{ === in fe-print.c ===  }
{ output stream  }
  PQprint : procedure (fout:PFILE; res:PPGresult; ps:PPQprintOpt);cdecl;
{ option structure  }
{
* really old printing routines
}
{ where to send the output  }
{ pad the fields with spaces  }
{ field separator  }
{ display headers?  }
  PQdisplayTuples : procedure (res:PPGresult; fp:PFILE; fillAlign:longint; fieldSep:Pchar; printHeader:longint;quiet:longint);cdecl;
(* Const before type ignored *)
{ output stream  }
{ print attribute names  }
{ delimiter bars  }
  PQprintTuples : procedure (res:PPGresult; fout:PFILE; printAttName:longint; terseOutput:longint; width:longint);cdecl;
{ width of column, if 0, use variable
* width  }
{ === in fe-lobj.c ===  }
{ Large-object access routines  }
  lo_open : function (conn:PPGconn; lobjId:Oid; mode:longint):longint;cdecl;
  lo_close : function (conn:PPGconn; fd:longint):longint;cdecl;
  lo_read : function (conn:PPGconn; fd:longint; buf:Pchar; len:size_t):longint;cdecl;
  lo_write : function (conn:PPGconn; fd:longint; buf:Pchar; len:size_t):longint;cdecl;
  lo_lseek : function (conn:PPGconn; fd:longint; offset:longint; whence:longint):longint;cdecl;
  lo_creat : function (conn:PPGconn; mode:longint):Oid;cdecl;
  lo_tell : function (conn:PPGconn; fd:longint):longint;cdecl;
  lo_truncate : function (conn:PPGconn; fd:longint; len:size_t):longint;cdecl;
  lo_unlink : function (conn:PPGconn; lobjId:Oid):longint;cdecl;
  lo_import : function (conn:PPGconn; filename:Pchar):Oid;cdecl;
  lo_import_with_oid : function (conn:PPGconn; filename:Pcchar; lobjId:Oid):Oid;cdecl;
  lo_export : function (conn:PPGconn; lobjId:Oid; filename:Pchar):longint;cdecl;
{ === in fe-misc.c ===  }
{ Determine length of multibyte encoded char at *s  }
  PQmblen : function (s:Pbyte; encoding:longint):longint;cdecl;
{ Get encoding id from environment variable PGCLIENTENCODING  }
  PQenv2encoding: function :longint;cdecl;

{ === in fe-auth.c === }
  PQencryptPassword : function (passwd:Pcchar; user:Pcchar):Pcchar;cdecl;

{ === in encnames.c === }
  pg_encoding_to_char: function (encoding:cint):Pcchar;cdecl;

Function InitialisePostgres3(Const libpath : ansistring) : integer;
Procedure InitialisePostgres3;
Procedure ReleasePostgres3;

function PQsetdb(M_PGHOST,M_PGPORT,M_PGOPT,M_PGTTY,M_DBNAME : pchar) : ppgconn;

var Postgres3LibraryHandle : TLibHandle= NilHandle;
  Postgres3LoadedLibrary : String = '';

implementation

resourcestring
  SErrLoadFailed     = 'Can not load PostgreSQL client library "%s". Check your installation.';
  SErrAlreadyLoaded  = 'PostgreSQL interface already initialized from library %s.';

var
  RefCount : integer;

procedure InitialisePostgres3;

begin
  EnterCriticalsection(libpgCriticalSection);
  try
    if (RefCount<>0) then
        // pretend to load whatever is already loaded, so we do not get a library name conflict.
      inc(Refcount)
    else
      InitialisePostgres3(pqlib)
  finally
    LeaveCriticalsection(libpgCriticalSection);
  end;
end;


function InitialisePostgres3(Const libpath : ansistring) : Integer;

begin
  EnterCriticalsection(libpgCriticalSection);
  try
    inc(RefCount);
    Result:=Refcount;
    if RefCount = 1 then
      begin
      Postgres3LibraryHandle := loadlibrary(libpath);
      if Postgres3LibraryHandle = nilhandle then
        begin
        RefCount := 0;
        Raise EInOutError.CreateFmt(SErrLoadFailed,[libpath]);
        end;

      Postgres3LoadedLibrary:=libpath;
      pointer(PQconnectStart) := GetProcedureAddress(Postgres3LibraryHandle,'PQconnectStart');
      pointer(PQconnectPoll) := GetProcedureAddress(Postgres3LibraryHandle,'PQconnectPoll');
      pointer(PQconnectdb) := GetProcedureAddress(Postgres3LibraryHandle,'PQconnectdb');
      pointer(PQsetdbLogin) := GetProcedureAddress(Postgres3LibraryHandle,'PQsetdbLogin');
      pointer(PQfinish) := GetProcedureAddress(Postgres3LibraryHandle,'PQfinish');
      pointer(PQconndefaults) := GetProcedureAddress(Postgres3LibraryHandle,'PQconndefaults');
      pointer(PQconninfoFree) := GetProcedureAddress(Postgres3LibraryHandle,'PQconninfoFree');
      pointer(PQresetStart) := GetProcedureAddress(Postgres3LibraryHandle,'PQresetStart');
      pointer(PQresetPoll) := GetProcedureAddress(Postgres3LibraryHandle,'PQresetPoll');
      pointer(PQreset) := GetProcedureAddress(Postgres3LibraryHandle,'PQreset');
      pointer(PQrequestCancel) := GetProcedureAddress(Postgres3LibraryHandle,'PQrequestCancel');
      pointer(PQdb) := GetProcedureAddress(Postgres3LibraryHandle,'PQdb');
      pointer(PQuser) := GetProcedureAddress(Postgres3LibraryHandle,'PQuser');
      pointer(PQpass) := GetProcedureAddress(Postgres3LibraryHandle,'PQpass');
      pointer(PQhost) := GetProcedureAddress(Postgres3LibraryHandle,'PQhost');
      pointer(PQport) := GetProcedureAddress(Postgres3LibraryHandle,'PQport');
      pointer(PQtty) := GetProcedureAddress(Postgres3LibraryHandle,'PQtty');
      pointer(PQoptions) := GetProcedureAddress(Postgres3LibraryHandle,'PQoptions');
      pointer(PQstatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQstatus');
      pointer(PQtransactionStatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQtransactionStatus');
      pointer(PQparameterStatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQparameterStatus');
      pointer(PQprotocolVersion) := GetProcedureAddress(Postgres3LibraryHandle,'PQprotocolVersion');
      pointer(PQserverVersion) := GetProcedureAddress(Postgres3LibraryHandle,'PQserverVersion');
      pointer(PQerrorMessage) := GetProcedureAddress(Postgres3LibraryHandle,'PQerrorMessage');
      pointer(PQsocket) := GetProcedureAddress(Postgres3LibraryHandle,'PQsocket');
      pointer(PQbackendPID) := GetProcedureAddress(Postgres3LibraryHandle,'PQbackendPID');
      pointer(PQclientEncoding) := GetProcedureAddress(Postgres3LibraryHandle,'PQclientEncoding');
      pointer(PQsetClientEncoding) := GetProcedureAddress(Postgres3LibraryHandle,'PQsetClientEncoding');
  {$ifdef USE_SSL}
      pointer(PQgetssl) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetssl');
  {$endif}
      pointer(PQsetErrorVerbosity) := GetProcedureAddress(Postgres3LibraryHandle,'PQsetErrorVerbosity');
      pointer(PQtrace) := GetProcedureAddress(Postgres3LibraryHandle,'PQtrace');
      pointer(PQuntrace) := GetProcedureAddress(Postgres3LibraryHandle,'PQuntrace');
      pointer(PQsetNoticeReceiver) := GetProcedureAddress(Postgres3LibraryHandle,'PQsetNoticeReceiver');
      pointer(PQsetNoticeProcessor) := GetProcedureAddress(Postgres3LibraryHandle,'PQsetNoticeProcessor');
      pointer(PQexec) := GetProcedureAddress(Postgres3LibraryHandle,'PQexec');
      pointer(PQexecParams) := GetProcedureAddress(Postgres3LibraryHandle,'PQexecParams');
      pointer(PQexecPrepared) := GetProcedureAddress(Postgres3LibraryHandle,'PQexecPrepared');
      pointer(PQPrepare) := GetProcedureAddress(Postgres3LibraryHandle,'PQprepare');
      pointer(PQdescribePrepared) := GetProcedureAddress(Postgres3LibraryHandle,'PQdescribePrepared');
      pointer(PQsendQuery) := GetProcedureAddress(Postgres3LibraryHandle,'PQsendQuery');
      pointer(PQsendQueryParams) := GetProcedureAddress(Postgres3LibraryHandle,'PQsendQueryParams');
      pointer(PQsendQueryPrepared) := GetProcedureAddress(Postgres3LibraryHandle,'PQsendQueryPrepared');
      pointer(PQgetResult) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetResult');
      pointer(PQisBusy) := GetProcedureAddress(Postgres3LibraryHandle,'PQisBusy');
      pointer(PQconsumeInput) := GetProcedureAddress(Postgres3LibraryHandle,'PQconsumeInput');
      pointer(PQnotifies) := GetProcedureAddress(Postgres3LibraryHandle,'PQnotifies');
      pointer(PQputCopyData) := GetProcedureAddress(Postgres3LibraryHandle,'PQputCopyData');
      pointer(PQputCopyEnd) := GetProcedureAddress(Postgres3LibraryHandle,'PQputCopyEnd');
      pointer(PQgetCopyData) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetCopyData');
      pointer(PQgetline) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetline');
      pointer(PQputline) := GetProcedureAddress(Postgres3LibraryHandle,'PQputline');
      pointer(PQgetlineAsync) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetlineAsync');
      pointer(PQputnbytes) := GetProcedureAddress(Postgres3LibraryHandle,'PQputnbytes');
      pointer(PQendcopy) := GetProcedureAddress(Postgres3LibraryHandle,'PQendcopy');
      pointer(PQsetnonblocking) := GetProcedureAddress(Postgres3LibraryHandle,'PQsetnonblocking');
      pointer(PQisnonblocking) := GetProcedureAddress(Postgres3LibraryHandle,'PQisnonblocking');
      pointer(PQisthreadsafe) := GetProcedureAddress(Postgres3LibraryHandle,'PQisthreadsafe');
      pointer(PQflush) := GetProcedureAddress(Postgres3LibraryHandle,'PQflush');
      pointer(PQfn) := GetProcedureAddress(Postgres3LibraryHandle,'PQfn');
      pointer(PQresultStatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQresultStatus');
      pointer(PQresStatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQresStatus');
      pointer(PQresultErrorMessage) := GetProcedureAddress(Postgres3LibraryHandle,'PQresultErrorMessage');
      pointer(PQresultErrorField) := GetProcedureAddress(Postgres3LibraryHandle,'PQresultErrorField');
      pointer(PQntuples) := GetProcedureAddress(Postgres3LibraryHandle,'PQntuples');
      pointer(PQnfields) := GetProcedureAddress(Postgres3LibraryHandle,'PQnfields');
      pointer(PQbinaryTuples) := GetProcedureAddress(Postgres3LibraryHandle,'PQbinaryTuples');
      pointer(PQfname) := GetProcedureAddress(Postgres3LibraryHandle,'PQfname');
      pointer(PQfnumber) := GetProcedureAddress(Postgres3LibraryHandle,'PQfnumber');
      pointer(PQftable) := GetProcedureAddress(Postgres3LibraryHandle,'PQftable');
      pointer(PQftablecol) := GetProcedureAddress(Postgres3LibraryHandle,'PQftablecol');
      pointer(PQfformat) := GetProcedureAddress(Postgres3LibraryHandle,'PQfformat');
      pointer(PQftype) := GetProcedureAddress(Postgres3LibraryHandle,'PQftype');
      pointer(PQfsize) := GetProcedureAddress(Postgres3LibraryHandle,'PQfsize');
      pointer(PQfmod) := GetProcedureAddress(Postgres3LibraryHandle,'PQfmod');
      pointer(PQcmdStatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQcmdStatus');
      pointer(PQoidStatus) := GetProcedureAddress(Postgres3LibraryHandle,'PQoidStatus');
      pointer(PQoidValue) := GetProcedureAddress(Postgres3LibraryHandle,'PQoidValue');
      pointer(PQcmdTuples) := GetProcedureAddress(Postgres3LibraryHandle,'PQcmdTuples');
      pointer(PQgetvalue) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetvalue');
      pointer(PQgetlength) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetlength');
      pointer(PQgetisnull) := GetProcedureAddress(Postgres3LibraryHandle,'PQgetisnull');
      pointer(PQnparams) := GetProcedureAddress(Postgres3LibraryHandle,'PQnparams');
      pointer(PQparamtype) := GetProcedureAddress(Postgres3LibraryHandle,'PQparamtype');
      pointer(PQclear) := GetProcedureAddress(Postgres3LibraryHandle,'PQclear');
      pointer(PQfreemem) := GetProcedureAddress(Postgres3LibraryHandle,'PQfreemem');
      pointer(PQmakeEmptyPGresult) := GetProcedureAddress(Postgres3LibraryHandle,'PQmakeEmptyPGresult');
      pointer(PQescapeStringConn) := GetProcedureAddress(Postgres3LibraryHandle,'PQescapeStringConn');
      pointer(PQescapeLiteral) := GetProcedureAddress(Postgres3LibraryHandle,'PQescapeLiteral');
      pointer(PQescapeIdentifier) := GetProcedureAddress(Postgres3LibraryHandle,'PQescapeIdentifier');
      pointer(PQescapeByteaConn) := GetProcedureAddress(Postgres3LibraryHandle,'PQescapeByteaConn');
      pointer(PQunescapeBytea) := GetProcedureAddress(Postgres3LibraryHandle,'PQunescapeBytea');
      pointer(PQescapeString) := GetProcedureAddress(Postgres3LibraryHandle,'PQescapeString');
      pointer(PQescapeBytea) := GetProcedureAddress(Postgres3LibraryHandle,'PQescapeBytea');
      pointer(PQprint) := GetProcedureAddress(Postgres3LibraryHandle,'PQprint');
      pointer(PQdisplayTuples) := GetProcedureAddress(Postgres3LibraryHandle,'PQdisplayTuples');
      pointer(PQprintTuples) := GetProcedureAddress(Postgres3LibraryHandle,'PQprintTuples');
      pointer(lo_open) := GetProcedureAddress(Postgres3LibraryHandle,'lo_open');
      pointer(lo_close) := GetProcedureAddress(Postgres3LibraryHandle,'lo_close');
      pointer(lo_read) := GetProcedureAddress(Postgres3LibraryHandle,'lo_read');
      pointer(lo_write) := GetProcedureAddress(Postgres3LibraryHandle,'lo_write');
      pointer(lo_lseek) := GetProcedureAddress(Postgres3LibraryHandle,'lo_lseek');
      pointer(lo_creat) := GetProcedureAddress(Postgres3LibraryHandle,'lo_creat');
      pointer(lo_tell) := GetProcedureAddress(Postgres3LibraryHandle,'lo_tell');
      pointer(lo_truncate) := GetProcedureAddress(Postgres3LibraryHandle,'lo_truncate');
      pointer(lo_unlink) := GetProcedureAddress(Postgres3LibraryHandle,'lo_unlink');
      pointer(lo_import) := GetProcedureAddress(Postgres3LibraryHandle,'lo_import');
      pointer(lo_import_with_oid) := GetProcedureAddress(Postgres3LibraryHandle,'lo_import_with_oid');
      pointer(lo_export) := GetProcedureAddress(Postgres3LibraryHandle,'lo_export');
      pointer(PQmblen) := GetProcedureAddress(Postgres3LibraryHandle,'PQmblen');
      pointer(PQenv2encoding) := GetProcedureAddress(Postgres3LibraryHandle,'PQenv2encoding');
      pointer(PQencryptPassword) := GetProcedureAddress(Postgres3LibraryHandle,'PQencryptPassword');
      pointer(pg_encoding_to_char) := GetProcedureAddress(Postgres3LibraryHandle,'pg_encoding_to_char');

      InitialiseDllist(libpath);
      end
    else
      if (libpath<>pqlib) and (Postgres3LoadedLibrary<>libpath) then
        begin
        Dec(RefCount);
        Raise EInOUtError.CreateFmt(SErrAlreadyLoaded,[Postgres3LoadedLibrary]);
        end;
  finally
    LeaveCriticalsection(libpgCriticalSection);
  end;
end;

Procedure ReleasePostgres3;

begin
  EnterCriticalsection(libpgCriticalSection);
  try
    if RefCount > 0 then dec(RefCount);
    if (RefCount = 0) and ( Postgres3LibraryHandle <> nilhandle) then
      begin
      if not UnloadLibrary(Postgres3LibraryHandle) then inc(RefCount);
      ReleaseDllist;
      end;
  finally
    LeaveCriticalsection(libpgCriticalSection);
  end;
end;

// This function is also defined in postgres3!
function PQsetdb(M_PGHOST,M_PGPORT,M_PGOPT,M_PGTTY,M_DBNAME : pchar) : ppgconn;
begin
   PQsetdb:=PQsetdbLogin(M_PGHOST,M_PGPORT,M_PGOPT,M_PGTTY,M_DBNAME,'','');
end;


end.
