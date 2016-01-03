{$ifdef fpc}
<<<<<<< HEAD
<<<<<<< HEAD
 {$mode objfpc}
 {$macro on}
=======
{$mode objfpc}
{$macro on}
>>>>>>> graemeg/fixes_2_2
=======
{$mode objfpc}
{$macro on}
>>>>>>> origin/fixes_2_2
{$endif}

{$h+}

{$ifdef fpc}
<<<<<<< HEAD
<<<<<<< HEAD
 // define ODBC version 3.52 by default for Windows and MAC and 3.51 for Linux/Unix
 {$if defined(WINDOWS) or defined(DARWIN)}
  {$define ODBCVER:=$0352}
 {$else}
  {$define ODBCVER:=$0351}
 {$endif}
{$else fpc}
 {$define ODBCVER:=$0352}
{$endif fpc}
=======
 // define ODBC version 3.51 by default
{$define ODBCVER:=$0351}
>>>>>>> graemeg/fixes_2_2
=======
 // define ODBC version 3.51 by default
{$define ODBCVER:=$0351}
>>>>>>> origin/fixes_2_2
{$if ODBCVER >= $0300}
  {$define ODBCVER3}
{$endif}
{$if ODBCVER >= $0350}
  {$define ODBCVER35}
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
{$if ODBCVER >= $0352}
  {$define ODBCVER352}
{$endif}

{$ifndef DYNLOADINGODBC}
 {$IFDEF WINDOWS}
  { $linklib odbc32}
 {$ELSE}
  {$linklib odbc}
 {$ENDIF}
{$endif}

{$ifndef DYNLOADINGODBC}
{$IFDEF WINDOWS}
  {$linklib odbc32}
{$ELSE}
  {$linklib odbc}
{$endif} 
{$endif}

{$ifndef DYNLOADINGODBC}
{$IFDEF WINDOWS}
  {$linklib odbc32}
{$ELSE}
  {$linklib odbc}
{$endif} 
{$endif}

{$ifndef DYNLOADINGODBC}
{$IFDEF WINDOWS}
  {$linklib odbc32}
{$ELSE}
  {$linklib odbc}
{$endif} 
{$endif}

{$ifndef DYNLOADINGODBC}
{$IFDEF WINDOWS}
  {$linklib odbc32}
{$ELSE}
  {$linklib odbc}
{$endif} 
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
{$else fpc}
  {$define ODBCVER3}
  {$define ODBCVER35}
{$endif fpc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

interface

uses
     Dynlibs,
     ctypes,
     sysutils;

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
    odbclib = 'libodbc.'+sharedsuffix;
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    odbclib = 'odbc32.dll';
{$ENDIF}

(* DATA TYPES CORRESPONDENCE
   BDE fields  ODBC types
   ----------  ------------------
   ftBlob      SQL_BINARY
   ftBoolean   SQL_BIT
   ftDate      SQL_TYPE_DATE
   ftTime      SQL_TYPE_TIME
   ftDateTime  SQL_TYPE_TIMESTAMP
   ftInteger   SQL_INTEGER
   ftSmallint  SQL_SMALLINT
   ftFloat     SQL_DOUBLE
   ftString    SQL_CHAR
   ftMemo      SQL_BINARY // SQL_VARCHAR
*)

type
<<<<<<< HEAD
<<<<<<< HEAD
  SQLCHAR      = cuchar;
  SQLSCHAR     = cschar;
  SQLSMALLINT  = csshort;
  SQLUSMALLINT = cushort;
  SQLINTEGER   = clong;
  SQLUINTEGER  = culong;
  SQLLEN       = {$IFDEF ODBCVER352}PtrInt{$ELSE}SQLINTEGER{$ENDIF};
  SQLULEN      = {$IFDEF ODBCVER352}PtrUInt{$ELSE}SQLUINTEGER{$ENDIF};
  SQLREAL      = cfloat;
  SQLDOUBLE    = cdouble;
  SQLFLOAT     = cdouble;
  SQLRETURN    = SQLSMALLINT;
  SQLPOINTER   = pointer;
=======
=======
>>>>>>> origin/fixes_2_2
  SQLCHAR      = char;
  SQLSMALLINT  = csshort;
  SQLUSMALLINT = cushort;
  SQLRETURN    = SQLSMALLINT;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SQLHANDLE    = pointer;
  SQLHENV      = SQLHANDLE;
  SQLHDBC      = SQLHANDLE;
  SQLHSTMT     = SQLHANDLE;
  SQLHDESC     = SQLHANDLE;
<<<<<<< HEAD
<<<<<<< HEAD
  SQLHWND      = pointer;
  SQLSETPOSIROW= {$IF DEFINED(CPU64) AND DEFINED(ODBCVER352)}cuint64{$ELSE}SQLUSMALLINT{$ENDIF};
  PSQLCHAR      = PChar;
  PSQLSMALLINT  = ^SQLSMALLINT;
  PSQLUSMALLINT = ^SQLUSMALLINT;
  PSQLINTEGER   = ^SQLINTEGER;
  PSQLUINTEGER  = ^SQLUINTEGER;
  PSQLLEN       = ^SQLLEN;
  PSQLULEN      = ^SQLULEN;
=======
=======
>>>>>>> origin/fixes_2_2
  SQLINTEGER   = clong;
  SQLUINTEGER  = culong;
  SQLPOINTER   = pointer;
  SQLREAL      = cfloat;
  SQLDOUBLE    = cdouble;
  SQLFLOAT     = cdouble;
  SQLHWND      = pointer;
  PSQLCHAR      = PChar;
  PSQLINTEGER   = ^SQLINTEGER;
  PSQLUINTEGER  = ^SQLUINTEGER;
  PSQLSMALLINT  = ^SQLSMALLINT;
  PSQLUSMALLINT = ^SQLUSMALLINT;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  PSQLREAL      = ^SQLREAL;
  PSQLDOUBLE    = ^SQLDOUBLE;
  PSQLFLOAT     = ^SQLFLOAT;
  PSQLHANDLE    = ^SQLHANDLE;

const
  { SQL data type codes }
  SQL_UNKNOWN_TYPE = 0;
  SQL_LONGVARCHAR   =(-1);
  SQL_BINARY        =(-2);
  SQL_VARBINARY     =(-3);
  SQL_LONGVARBINARY =(-4);
  SQL_BIGINT        =(-5);
  SQL_TINYINT       =(-6);
  SQL_BIT           =(-7);
  SQL_WCHAR         =(-8);
  SQL_WVARCHAR      =(-9);
  SQL_WLONGVARCHAR  =(-10);

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
  SQL_CHAR          = 1;
  SQL_NUMERIC       = 2;
  SQL_DECIMAL       = 3;
  SQL_INTEGER       = 4;
  SQL_SMALLINT      = 5;
  SQL_FLOAT         = 6;
  SQL_REAL          = 7;
  SQL_DOUBLE        = 8;
 {$ifdef ODBCVER3}
  SQL_DATETIME      = 9;
 {$endif}
  SQL_VARCHAR       = 12;

 {$ifdef ODBCVER3}
  SQL_TYPE_DATE     = 91;
  SQL_TYPE_TIME     = 92;
  SQL_TYPE_TIMESTAMP= 93;
<<<<<<< HEAD
<<<<<<< HEAD
  // Microsoft has -150 thru -199 reserved for Microsoft SQL Server Native Client driver usage.
  SQL_SS_VARIANT          = -150;
  SQL_SS_UDT              = -151;
  SQL_SS_XML              = -152;
  SQL_SS_TABLE            = -153;
  SQL_SS_TIME2            = -154;
  SQL_SS_TIMESTAMPOFFSET  = -155;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 {$endif}

  SQL_DATE       = 9;
  SQL_TIME       = 10;
  SQL_TIMESTAMP  = 11;
  {$if ODBCVER >= $0300}
  SQL_INTERVAL   = 10;
  {$endif}
  {$if ODBCVER >= $0350}
  SQL_GUID       = -11;
  {$endif}
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  { interval codes}
  {$ifdef ODBCVER3}
  SQL_CODE_YEAR             = 1;
  SQL_CODE_MONTH            = 2;
  SQL_CODE_DAY              = 3;
  SQL_CODE_HOUR             = 4;
  SQL_CODE_MINUTE           = 5;
  SQL_CODE_SECOND           = 6;
  SQL_CODE_YEAR_TO_MONTH    = 7;
  SQL_CODE_DAY_TO_HOUR      = 8;
  SQL_CODE_DAY_TO_MINUTE    = 9;
  SQL_CODE_DAY_TO_SECOND    = 10;
  SQL_CODE_HOUR_TO_MINUTE   = 11;
  SQL_CODE_HOUR_TO_SECOND   = 12;
  SQL_CODE_MINUTE_TO_SECOND = 13;

  SQL_INTERVAL_YEAR             = 100 + SQL_CODE_YEAR;
  SQL_INTERVAL_MONTH            = 100 + SQL_CODE_MONTH;
  SQL_INTERVAL_DAY              = 100 + SQL_CODE_DAY;
  SQL_INTERVAL_HOUR             = 100 + SQL_CODE_HOUR;
  SQL_INTERVAL_MINUTE           = 100 + SQL_CODE_MINUTE;
  SQL_INTERVAL_SECOND           = 100 + SQL_CODE_SECOND;
  SQL_INTERVAL_YEAR_TO_MONTH    = 100 + SQL_CODE_YEAR_TO_MONTH;
  SQL_INTERVAL_DAY_TO_HOUR      = 100 + SQL_CODE_DAY_TO_HOUR;
  SQL_INTERVAL_DAY_TO_MINUTE    = 100 + SQL_CODE_DAY_TO_MINUTE;
  SQL_INTERVAL_DAY_TO_SECOND    = 100 + SQL_CODE_DAY_TO_SECOND;
  SQL_INTERVAL_HOUR_TO_MINUTE   = 100 + SQL_CODE_HOUR_TO_MINUTE;
  SQL_INTERVAL_HOUR_TO_SECOND   = 100 + SQL_CODE_HOUR_TO_SECOND;
  SQL_INTERVAL_MINUTE_TO_SECOND = 100 + SQL_CODE_MINUTE_TO_SECOND;
  {$else}
  SQL_INTERVAL_YEAR             = -80;
  SQL_INTERVAL_MONTH            = -81;
  SQL_INTERVAL_YEAR_TO_MONTH    = -82;
  SQL_INTERVAL_DAY              = -83;
  SQL_INTERVAL_HOUR             = -84;
  SQL_INTERVAL_MINUTE           = -85;
  SQL_INTERVAL_SECOND           = -86;
  SQL_INTERVAL_DAY_TO_HOUR      = -87;
  SQL_INTERVAL_DAY_TO_MINUTE    = -88;
  SQL_INTERVAL_DAY_TO_SECOND    = -89;
  SQL_INTERVAL_HOUR_TO_MINUTE   = -90;
  SQL_INTERVAL_HOUR_TO_SECOND   = -91;
  SQL_INTERVAL_MINUTE_TO_SECOND = -92;
  {$endif}
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  { Unicode data type codes }
  {$ifndef ODBCVER3}
  SQL_UNICODE             = -95;
  SQL_UNICODE_VARCHAR     = -96;
  SQL_UNICODE_LONGVARCHAR = -97;
  SQL_UNICODE_CHAR        = SQL_UNICODE;
  {$else}
  { The previous definitions for SQL_UNICODE_ are historical and obsolete }
  SQL_UNICODE             = SQL_WCHAR;
  SQL_UNICODE_VARCHAR     = SQL_WVARCHAR;
  SQL_UNICODE_LONGVARCHAR = SQL_WLONGVARCHAR;
  SQL_UNICODE_CHAR        = SQL_WCHAR;
  {$endif}

  { C datatype to SQL datatype mapping }
  SQL_C_CHAR   = SQL_CHAR;
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_C_WCHAR  = SQL_WCHAR;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SQL_C_LONG   = SQL_INTEGER;
  SQL_C_SHORT  = SQL_SMALLINT;
  SQL_C_FLOAT  = SQL_REAL;
  SQL_C_DOUBLE = SQL_DOUBLE;
{$ifdef ODBCVER3}
  SQL_C_NUMERIC = SQL_NUMERIC;
{$endif}
  SQL_C_DEFAULT = 99;

  SQL_SIGNED_OFFSET   = -20;
  SQL_UNSIGNED_OFFSET = -22;

  SQL_C_DATE      = SQL_DATE;
  SQL_C_TIME      = SQL_TIME;
  SQL_C_TIMESTAMP = SQL_TIMESTAMP;
{$ifdef ODBCVER3}
  SQL_C_TYPE_DATE       = SQL_TYPE_DATE;
  SQL_C_TYPE_TIME       = SQL_TYPE_TIME;
  SQL_C_TYPE_TIMESTAMP  = SQL_TYPE_TIMESTAMP;
  SQL_C_INTERVAL_YEAR   = SQL_INTERVAL_YEAR;
  SQL_C_INTERVAL_MONTH  = SQL_INTERVAL_MONTH;
  SQL_C_INTERVAL_DAY    = SQL_INTERVAL_DAY;
  SQL_C_INTERVAL_HOUR   = SQL_INTERVAL_HOUR;
  SQL_C_INTERVAL_MINUTE = SQL_INTERVAL_MINUTE;
  SQL_C_INTERVAL_SECOND = SQL_INTERVAL_SECOND;
  SQL_C_INTERVAL_YEAR_TO_MONTH    = SQL_INTERVAL_YEAR_TO_MONTH;
  SQL_C_INTERVAL_DAY_TO_HOUR      = SQL_INTERVAL_DAY_TO_HOUR;
  SQL_C_INTERVAL_DAY_TO_MINUTE    = SQL_INTERVAL_DAY_TO_MINUTE;
  SQL_C_INTERVAL_DAY_TO_SECOND    = SQL_INTERVAL_DAY_TO_SECOND;
  SQL_C_INTERVAL_HOUR_TO_MINUTE   = SQL_INTERVAL_HOUR_TO_MINUTE;
  SQL_C_INTERVAL_HOUR_TO_SECOND   = SQL_INTERVAL_HOUR_TO_SECOND;
  SQL_C_INTERVAL_MINUTE_TO_SECOND = SQL_INTERVAL_MINUTE_TO_SECOND;
{$endif}
  SQL_C_BINARY = SQL_BINARY;
  SQL_C_BIT    = SQL_BIT;
{$ifdef ODBCVER3}
  SQL_C_SBIGINT = SQL_BIGINT+SQL_SIGNED_OFFSET;   // SIGNED BIGINT
  SQL_C_UBIGINT = SQL_BIGINT+SQL_UNSIGNED_OFFSET; // UNSIGNED BIGINT
{$endif}
  SQL_C_TINYINT  =  SQL_TINYINT;
  SQL_C_SLONG    =  SQL_C_LONG +SQL_SIGNED_OFFSET;   // SIGNED INTEGER
  SQL_C_SSHORT   =  SQL_C_SHORT+SQL_SIGNED_OFFSET;   // SIGNED SMALLINT
  SQL_C_STINYINT =  SQL_TINYINT+SQL_SIGNED_OFFSET;   // SIGNED TINYINT
  SQL_C_ULONG    =  SQL_C_LONG +SQL_UNSIGNED_OFFSET; // UNSIGNED INTEGER
  SQL_C_USHORT   =  SQL_C_SHORT+SQL_UNSIGNED_OFFSET; // UNSIGNED SMALLINT
  SQL_C_UTINYINT =  SQL_TINYINT+SQL_UNSIGNED_OFFSET; // UNSIGNED TINYINT
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_C_BOOKMARK = {$if defined(CPU64) and defined(ODBCVER352)}SQL_C_UBIGINT{$else}SQL_C_ULONG{$endif}; // BOOKMARK

=======
  SQL_C_BOOKMARK = SQL_C_ULONG; // BOOKMARK
  
>>>>>>> graemeg/fixes_2_2
=======
  SQL_C_BOOKMARK = SQL_C_ULONG; // BOOKMARK
  
>>>>>>> origin/fixes_2_2
{$ifdef ODBCVER35}
  SQL_C_GUID    = SQL_GUID;
{$endif}

  SQL_TYPE_NULL = 0;
{$ifndef ODBCVER3}
  SQL_TYPE_MIN  = SQL_BIT;
  SQL_TYPE_MAX  = SQL_VARCHAR;
{$endif}

{$ifdef ODBCVER3}
  SQL_C_VARBOOKMARK = SQL_C_BINARY;
{$endif}

 SQL_API_SQLDESCRIBEPARAM=58;
 SQL_NO_TOTAL   = -4;

type
  SQL_DATE_STRUCT = packed record
    Year : SQLSMALLINT;
    Month : SQLUSMALLINT;
    Day : SQLUSMALLINT;
  end;
  PSQL_DATE_STRUCT = ^SQL_DATE_STRUCT;

  SQL_TIME_STRUCT = packed record
    Hour : SQLUSMALLINT;
    Minute : SQLUSMALLINT;
    Second : SQLUSMALLINT;
  end;
  PSQL_TIME_STRUCT = ^SQL_TIME_STRUCT;

  SQL_TIMESTAMP_STRUCT = packed record
    Year :     SQLUSMALLINT;
    Month :    SQLUSMALLINT;
    Day :      SQLUSMALLINT;
    Hour :     SQLUSMALLINT;
    Minute :   SQLUSMALLINT;
    Second :   SQLUSMALLINT;
    Fraction : SQLUINTEGER;
  end;
  PSQL_TIMESTAMP_STRUCT = ^SQL_TIMESTAMP_STRUCT;

<<<<<<< HEAD
<<<<<<< HEAD
  SQL_NUMERIC_STRUCT = packed record
    precision: SQLCHAR;
    scale    : SQLSCHAR;
    sign     : SQLCHAR; // 1 if positive, 0 if negative
    val      : array[0..15] of SQLCHAR;
  end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  SQL_NAME_LEN = 128;

  SQL_OV_ODBC3          = 3;
  SQL_OV_ODBC2          = 2;
  SQL_ATTR_ODBC_VERSION = 200;

  { Options for SQLDriverConnect }
  SQL_DRIVER_NOPROMPT          = 0;
  SQL_DRIVER_COMPLETE          = 1;
  SQL_DRIVER_PROMPT            = 2;
  SQL_DRIVER_COMPLETE_REQUIRED = 3;

  { whether an attribute is a pointer or not }
  SQL_IS_POINTER    = (-4);
  SQL_IS_UINTEGER   = (-5);
  SQL_IS_INTEGER    = (-6);
  SQL_IS_USMALLINT  = (-7);
  SQL_IS_SMALLINT   = (-8);
  { SQLExtendedFetch "fFetchType" values }
  SQL_FETCH_BOOKMARK = 8;

  SQL_SCROLL_OPTIONS = 44;

  { SQL_USE_BOOKMARKS options }
  SQL_UB_OFF      = 0;
  SQL_UB_ON       = 1;
  SQL_UB_DEFAULT  = SQL_UB_OFF;
  SQL_UB_FIXED    = SQL_UB_ON;
  SQL_UB_VARIABLE = 2;

  { SQL_SCROLL_OPTIONS masks }
  SQL_SO_FORWARD_ONLY  = $01;
  SQL_SO_KEYSET_DRIVEN = $02;
  SQL_SO_DYNAMIC       = $04;
  SQL_SO_MIXED         = $08;
  SQL_SO_STATIC        = $10;

  SQL_BOOKMARK_PERSISTENCE = 82;
  SQL_STATIC_SENSITIVITY   = 83;

  { SQL_BOOKMARK_PERSISTENCE values }
  SQL_BP_CLOSE       = $01;
  SQL_BP_DELETE      = $02;
  SQL_BP_DROP        = $04;
  SQL_BP_TRANSACTION = $08;
  SQL_BP_UPDATE      = $10;
  SQL_BP_OTHER_HSTMT = $20;
  SQL_BP_SCROLL      = $40;

  SQL_DYNAMIC_CURSOR_ATTRIBUTES1      = 144;
  SQL_DYNAMIC_CURSOR_ATTRIBUTES2      = 145;
  SQL_FORWARD_ONLY_CURSOR_ATTRIBUTES1 = 146;
  SQL_FORWARD_ONLY_CURSOR_ATTRIBUTES2 = 147;
  SQL_INDEX_KEYWORDS                  = 148;
  SQL_INFO_SCHEMA_VIEWS               = 149;
  SQL_KEYSET_CURSOR_ATTRIBUTES1       = 150;
  SQL_KEYSET_CURSOR_ATTRIBUTES2       = 151;
  SQL_STATIC_CURSOR_ATTRIBUTES1       = 167;
  SQL_STATIC_CURSOR_ATTRIBUTES2       = 168;

  { supported SQLFetchScroll FetchOrientation's }
  SQL_CA1_NEXT          = 1;
  SQL_CA1_ABSOLUTE      = 2;
  SQL_CA1_RELATIVE      = 4;
  SQL_CA1_BOOKMARK      = 8;

  { supported SQLSetPos LockType's }
  SQL_CA1_LOCK_NO_CHANGE= $40;
  SQL_CA1_LOCK_EXCLUSIVE= $80;
  SQL_CA1_LOCK_UNLOCK   =$100;

  { supported SQLSetPos Operations }
  SQL_CA1_POS_POSITION  = $200;
  SQL_CA1_POS_UPDATE    = $400;
  SQL_CA1_POS_DELETE    = $800;
  SQL_CA1_POS_REFRESH   =$1000;

  { positioned updates and deletes }
  SQL_CA1_POSITIONED_UPDATE=$2000;
  SQL_CA1_POSITIONED_DELETE=$4000;
  SQL_CA1_SELECT_FOR_UPDATE=$8000;

  { supported SQLBulkOperations operations }
  SQL_CA1_BULK_ADD                =$10000;
  SQL_CA1_BULK_UPDATE_BY_BOOKMARK =$20000;
  SQL_CA1_BULK_DELETE_BY_BOOKMARK =$40000;
  SQL_CA1_BULK_FETCH_BY_BOOKMARK  =$80000;

  { supported values for SQL_ATTR_SCROLL_CONCURRENCY }
  SQL_CA2_READ_ONLY_CONCURRENCY = 1;
  SQL_CA2_LOCK_CONCURRENCY      = 2;
  SQL_CA2_OPT_ROWVER_CONCURRENCY= 4;
  SQL_CA2_OPT_VALUES_CONCURRENCY= 8;

  { sensitivity of the cursor to its own inserts, deletes, and updates }
  SQL_CA2_SENSITIVITY_ADDITIONS =$10;
  SQL_CA2_SENSITIVITY_DELETIONS =$20;
  SQL_CA2_SENSITIVITY_UPDATES   =$40;

{  semantics of SQL_ATTR_MAX_ROWS }
  SQL_CA2_MAX_ROWS_SELECT       = $80;
  SQL_CA2_MAX_ROWS_INSERT       =$100;
  SQL_CA2_MAX_ROWS_DELETE       =$200;
  SQL_CA2_MAX_ROWS_UPDATE       =$400;
  SQL_CA2_MAX_ROWS_CATALOG      =$800;
  SQL_CA2_MAX_ROWS_AFFECTS_ALL  =(SQL_CA2_MAX_ROWS_SELECT or
                                        SQL_CA2_MAX_ROWS_INSERT or SQL_CA2_MAX_ROWS_DELETE or
                                        SQL_CA2_MAX_ROWS_UPDATE or SQL_CA2_MAX_ROWS_CATALOG);

  { semantics of SQL_DIAG_CURSOR_ROW_COUNT }
  SQL_CA2_CRC_EXACT             = $1000;
  SQL_CA2_CRC_APPROXIMATE       = $2000;

  {  the kinds of positioned statements that can be simulated }
  SQL_CA2_SIMULATE_NON_UNIQUE   = $4000;
  SQL_CA2_SIMULATE_TRY_UNIQUE   = $8000;
  SQL_CA2_SIMULATE_UNIQUE       =$10000;

  {  Operations in SQLBulkOperations }
  SQL_ADD                     = 4;
  SQL_SETPOS_MAX_OPTION_VALUE = SQL_ADD;
  SQL_UPDATE_BY_BOOKMARK      = 5;
  SQL_DELETE_BY_BOOKMARK      = 6;
  SQL_FETCH_BY_BOOKMARK       = 7;

  { Operations in SQLSetPos }
  SQL_POSITION                = 0;
  SQL_REFRESH                 = 1;
  SQL_UPDATE                  = 2;
  SQL_DELETE                  = 3;

  { Lock options in SQLSetPos }
  SQL_LOCK_NO_CHANGE          = 0;
  SQL_LOCK_EXCLUSIVE          = 1;
  SQL_LOCK_UNLOCK             = 2;

  { SQLExtendedFetch "rgfRowStatus" element values }
  SQL_ROW_SUCCESS           = 0;
  SQL_ROW_DELETED           = 1;
  SQL_ROW_UPDATED           = 2;
  SQL_ROW_NOROW             = 3;
  SQL_ROW_ADDED             = 4;
  SQL_ROW_ERROR             = 5;
  SQL_ROW_SUCCESS_WITH_INFO = 6;

  SQL_ROW_PROCEED           = 0;
  SQL_ROW_IGNORE            = 1;

  SQL_MAX_DSN_LENGTH           = 32; { maximum data source name size }

  SQL_MAX_OPTION_STRING_LENGTH = 256;

<<<<<<< HEAD
<<<<<<< HEAD
=======
  SQL_ODBC_CURSORS      = 110;
  SQL_ATTR_ODBC_CURSORS = SQL_ODBC_CURSORS;
>>>>>>> graemeg/fixes_2_2
=======
  SQL_ODBC_CURSORS      = 110;
  SQL_ATTR_ODBC_CURSORS = SQL_ODBC_CURSORS;
>>>>>>> origin/fixes_2_2
  { SQL_ODBC_CURSORS options }
  SQL_CUR_USE_IF_NEEDED = 0;
  SQL_CUR_USE_ODBC      = 1;
  SQL_CUR_USE_DRIVER    = 2;
  SQL_CUR_DEFAULT       = SQL_CUR_USE_DRIVER;

  SQL_PARAM_TYPE_UNKNOWN = 0;
  SQL_PARAM_INPUT        = 1;
  SQL_PARAM_INPUT_OUTPUT = 2;
  SQL_RESULT_COL         = 3;
  SQL_PARAM_OUTPUT       = 4;
  SQL_RETURN_VALUE       = 5;

  { special length/indicator values }
  SQL_NULL_DATA    = (-1);
  SQL_DATA_AT_EXEC = (-2);

  SQL_SUCCESS  = 0;
  SQL_SUCCESS_WITH_INFO = 1;

  SQL_NO_DATA = 100;
  SQL_ERROR   = (-1);
  SQL_INVALID_HANDLE = (-2);

  SQL_STILL_EXECUTING =  2;
  SQL_NEED_DATA       = 99;
  { flags for null-terminated string }
  SQL_NTS  = (-3);

  { maximum message length }
  SQL_MAX_MESSAGE_LENGTH = 512;

  { date/time length constants }
{$ifdef ODBCVER3}
  SQL_DATE_LEN      = 10;
  SQL_TIME_LEN      =  8;  { add P+1 if precision is nonzero }
  SQL_TIMESTAMP_LEN = 19;  { add P+1 if precision is nonzero }
{$endif}

  { handle type identifiers }
  SQL_HANDLE_ENV   = 1;
  SQL_HANDLE_DBC   = 2;
  SQL_HANDLE_STMT  = 3;
  SQL_HANDLE_DESC  = 4;

{$ifdef ODBCVER3}
  { environment attribute }
  SQL_ATTR_OUTPUT_NTS  = 10001;
  { connection attributes }
  SQL_ATTR_AUTO_IPD    = 10001;
  SQL_ATTR_METADATA_ID = 10014;
{$endif}  { ODBCVER >= 0x0300 }

  { statement attributes }
  SQL_ATTR_APP_ROW_DESC       = 10010;
  SQL_ATTR_APP_PARAM_DESC     = 10011;
  SQL_ATTR_IMP_ROW_DESC       = 10012;
  SQL_ATTR_IMP_PARAM_DESC     = 10013;
  SQL_ATTR_CURSOR_SCROLLABLE  = (-1);
  SQL_ATTR_CURSOR_SENSITIVITY = (-2);
  SQL_QUERY_TIMEOUT           =0;
  SQL_MAX_ROWS                =1;
  SQL_NOSCAN                  =2;
  SQL_MAX_LENGTH              =3;
  SQL_ASYNC_ENABLE            =4;       // same as SQL_ATTR_ASYNC_ENABLE */
  SQL_BIND_TYPE               =5;
  SQL_CURSOR_TYPE             = 6;
  SQL_CONCURRENCY             = 7;
  SQL_KEYSET_SIZE             =8;
  SQL_ROWSET_SIZE             =9;
  SQL_SIMULATE_CURSOR         =10;
  SQL_RETRIEVE_DATA           =11;
  SQL_USE_BOOKMARKS           =12;
  SQL_GET_BOOKMARK            =13;      //      GetStmtOption Only */
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_ROW_NUMBER              =14 ;     //      GetStmtOption Only */

=======
  SQL_ROW_NUMBER              = 14;     //      GetStmtOption Only */
  
>>>>>>> graemeg/fixes_2_2
=======
  SQL_ROW_NUMBER              = 14;     //      GetStmtOption Only */
  
>>>>>>> origin/fixes_2_2
  SQL_ATTR_CURSOR_TYPE        = SQL_CURSOR_TYPE;
  SQL_ATTR_CONCURRENCY        = SQL_CONCURRENCY;
  SQL_ATTR_FETCH_BOOKMARK_PTR = 16;
  SQL_ATTR_ROW_STATUS_PTR     = 25;
  SQL_ATTR_ROWS_FETCHED_PTR   = 26;
<<<<<<< HEAD
<<<<<<< HEAD

  SQL_ATTR_ROW_NUMBER         = SQL_ROW_NUMBER;
=======
=======
>>>>>>> origin/fixes_2_2
  SQL_AUTOCOMMIT              = 102;
  SQL_ATTR_AUTOCOMMIT         = SQL_AUTOCOMMIT;

  SQL_ATTR_ROW_NUMBER         = SQL_ROW_NUMBER;
  SQL_TXN_ISOLATION           = 108;
  SQL_ATTR_TXN_ISOLATION      = SQL_TXN_ISOLATION;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SQL_ATTR_MAX_ROWS           = SQL_MAX_ROWS;
  SQL_ATTR_USE_BOOKMARKS      = SQL_USE_BOOKMARKS;

//* connection attributes */
  SQL_ACCESS_MODE             =101;
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_AUTOCOMMIT              =102;
=======
//  SQL_AUTOCOMMIT              =102;
>>>>>>> graemeg/fixes_2_2
=======
//  SQL_AUTOCOMMIT              =102;
>>>>>>> origin/fixes_2_2
  SQL_LOGIN_TIMEOUT           =103;
  SQL_OPT_TRACE               =104;
  SQL_OPT_TRACEFILE           =105;
  SQL_TRANSLATE_DLL           =106;
  SQL_TRANSLATE_OPTION        =107;
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_TXN_ISOLATION           =108;
  SQL_CURRENT_QUALIFIER       =109;
  SQL_ODBC_CURSORS            =110;
=======
//  SQL_TXN_ISOLATION           =108;
  SQL_CURRENT_QUALIFIER       =109;
//  SQL_ODBC_CURSORS            =110;
>>>>>>> graemeg/fixes_2_2
=======
//  SQL_TXN_ISOLATION           =108;
  SQL_CURRENT_QUALIFIER       =109;
//  SQL_ODBC_CURSORS            =110;
>>>>>>> origin/fixes_2_2
  SQL_QUIET_MODE              =111;
  SQL_PACKET_SIZE             =112;


//* connection attributes with new names */
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_ATTR_ACCESS_MODE        =SQL_ACCESS_MODE;
  SQL_ATTR_AUTOCOMMIT         =SQL_AUTOCOMMIT;
  SQL_ATTR_CONNECTION_DEAD    =1209;        //* GetConnectAttr only */
  SQL_ATTR_CONNECTION_TIMEOUT =113;
  SQL_ATTR_CURRENT_CATALOG    =SQL_CURRENT_QUALIFIER;
  SQL_ATTR_DISCONNECT_BEHAVIOR=114;
  SQL_ATTR_ENLIST_IN_DTC      =1207;
  SQL_ATTR_ENLIST_IN_XA       =1208;
  SQL_ATTR_LOGIN_TIMEOUT      =SQL_LOGIN_TIMEOUT;
  SQL_ATTR_ODBC_CURSORS       =SQL_ODBC_CURSORS;
  SQL_ATTR_PACKET_SIZE        =SQL_PACKET_SIZE;
  SQL_ATTR_QUIET_MODE         =SQL_QUIET_MODE;
  SQL_ATTR_TRACE              =SQL_OPT_TRACE;
  SQL_ATTR_TRACEFILE          =SQL_OPT_TRACEFILE;
  SQL_ATTR_TRANSLATE_LIB      =SQL_TRANSLATE_DLL;
  SQL_ATTR_TRANSLATE_OPTION   =SQL_TRANSLATE_OPTION;
  SQL_ATTR_TXN_ISOLATION      =SQL_TXN_ISOLATION;
=======
=======
>>>>>>> origin/fixes_2_2
  SQL_ATTR_ACCESS_MODE              =SQL_ACCESS_MODE;
//  SQL_ATTR_AUTOCOMMIT                       =SQL_AUTOCOMMIT;
  SQL_ATTR_CONNECTION_DEAD        =1209;        //* GetConnectAttr only */
  SQL_ATTR_CONNECTION_TIMEOUT   =113;
  SQL_ATTR_CURRENT_CATALOG        =SQL_CURRENT_QUALIFIER;
  SQL_ATTR_DISCONNECT_BEHAVIOR=114;
  SQL_ATTR_ENLIST_IN_DTC                  =1207;
  SQL_ATTR_ENLIST_IN_XA             =1208;
  SQL_ATTR_LOGIN_TIMEOUT                  =SQL_LOGIN_TIMEOUT;
//  SQL_ATTR_ODBC_CURSORS             =SQL_ODBC_CURSORS;
  SQL_ATTR_PACKET_SIZE              =SQL_PACKET_SIZE;
  SQL_ATTR_QUIET_MODE                       =SQL_QUIET_MODE;
  SQL_ATTR_TRACE                                      =SQL_OPT_TRACE;
  SQL_ATTR_TRACEFILE                        =SQL_OPT_TRACEFILE;
  SQL_ATTR_TRANSLATE_LIB                  =SQL_TRANSLATE_DLL;
  SQL_ATTR_TRANSLATE_OPTION       =SQL_TRANSLATE_OPTION;
//  SQL_ATTR_TXN_ISOLATION                  =SQL_TXN_ISOLATION;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//* SQL_ACCESS_MODE options */
  SQL_MODE_READ_WRITE         =0;
  SQL_MODE_READ_ONLY          =1;
  SQL_MODE_DEFAULT            =SQL_MODE_READ_WRITE;

  //* SQL_AUTOCOMMIT options */
  SQL_AUTOCOMMIT_OFF          = 0;
  SQL_AUTOCOMMIT_ON           = 1;
  SQL_AUTOCOMMIT_DEFAULT      = SQL_AUTOCOMMIT_ON;
  { SQL_ATTR_CURSOR_SCROLLABLE values }
  SQL_NONSCROLLABLE              = 0;
  SQL_SCROLLABLE                 = 1;
  { SQL_CURSOR_TYPE options }
  SQL_CURSOR_FORWARD_ONLY     = 0;
  SQL_CURSOR_KEYSET_DRIVEN    = 1;
  SQL_CURSOR_DYNAMIC          = 2;
  SQL_CURSOR_STATIC           = 3;
  SQL_CURSOR_TYPE_DEFAULT     = SQL_CURSOR_FORWARD_ONLY;{ Default value }

  { SQL_CONCURRENCY options }
  SQL_CONCUR_READ_ONLY = 1;
  SQL_CONCUR_LOCK      = 2;
  SQL_CONCUR_ROWVER    = 3;
  SQL_CONCUR_VALUES    = 4;
  SQL_CONCUR_DEFAULT   = SQL_CONCUR_READ_ONLY; { Default value }

   { identifiers of fields in the SQL descriptor }
  {$ifdef ODBCVER3}
  SQL_DESC_COUNT                  = 1001;
  SQL_DESC_TYPE                   = 1002;
  SQL_DESC_LENGTH                 = 1003;
  SQL_DESC_OCTET_LENGTH_PTR       = 1004;
  SQL_DESC_PRECISION              = 1005;
  SQL_DESC_SCALE                  = 1006;
  SQL_DESC_DATETIME_INTERVAL_CODE = 1007;
  SQL_DESC_NULLABLE               = 1008;
  SQL_DESC_INDICATOR_PTR          = 1009;
  SQL_DESC_DATA_PTR               = 1010;
  SQL_DESC_NAME                   = 1011;
  SQL_DESC_UNNAMED                = 1012;
  SQL_DESC_OCTET_LENGTH           = 1013;
  SQL_DESC_ALLOC_TYPE             = 1099;
  {$endif}

  { identifiers of fields in the diagnostics area }
{$ifdef ODBCVER3}
  SQL_DIAG_RETURNCODE            = 1;
  SQL_DIAG_NUMBER                = 2;
  SQL_DIAG_ROW_COUNT             = 3;
  SQL_DIAG_SQLSTATE              = 4;
  SQL_DIAG_NATIVE                = 5;
  SQL_DIAG_MESSAGE_TEXT          = 6;
  SQL_DIAG_DYNAMIC_FUNCTION      = 7;
  SQL_DIAG_CLASS_ORIGIN          = 8;
  SQL_DIAG_SUBCLASS_ORIGIN       = 9;
  SQL_DIAG_CONNECTION_NAME       = 10;
  SQL_DIAG_SERVER_NAME           = 11;
  SQL_DIAG_DYNAMIC_FUNCTION_CODE = 12;
{$endif}

  { dynamic function codes }
{$ifdef ODBCVER3}
  SQL_DIAG_ALTER_TABLE           =  4;
  SQL_DIAG_CREATE_INDEX          = (-1);
  SQL_DIAG_CREATE_TABLE          = 77;
  SQL_DIAG_CREATE_VIEW           = 84;
  SQL_DIAG_DELETE_WHERE          = 19;
  SQL_DIAG_DROP_INDEX            = (-2);
  SQL_DIAG_DROP_TABLE            = 32;
  SQL_DIAG_DROP_VIEW             = 36;
  SQL_DIAG_DYNAMIC_DELETE_CURSOR = 38;
  SQL_DIAG_DYNAMIC_UPDATE_CURSOR = 81;
  SQL_DIAG_GRANT                 = 48;
  SQL_DIAG_INSERT                = 50;
  SQL_DIAG_REVOKE                = 59;
  SQL_DIAG_SELECT_CURSOR         = 85;
  SQL_DIAG_UNKNOWN_STATEMENT     =  0;
  SQL_DIAG_UPDATE_WHERE          = 82;
{$endif}  { ODBCVER >= 0x0300 }

  { Statement attribute values for cursor sensitivity }
{$ifdef ODBCVER3}
  SQL_UNSPECIFIED     = 0;
  SQL_INSENSITIVE     = 1;
  SQL_SENSITIVE       = 2;
{$endif}

  { GetTypeInfo() request for all data types }
  SQL_ALL_TYPES       = 0;

  { Default conversion code for SQLBindCol(), SQLBindParam() and SQLGetData() }
{$ifdef ODBCVER3}
  SQL_DEFAULT         = 99;
{$endif}

  { SQLGetData() code indicating that the application row descriptor
    specifies the data type }
{$ifdef ODBCVER3}
  SQL_ARD_TYPE      = (-99);
{$endif}

  { SQL date/time type subcodes }
{$ifdef ODBCVER3}
  SQL_CODE_DATE       = 1;
  SQL_CODE_TIME       = 2;
  SQL_CODE_TIMESTAMP  = 3;
{$endif}

  { CLI option values }
{$ifdef ODBCVER3}
  SQL_FALSE           = 0;
  SQL_TRUE            = 1;
{$endif}

     { values of NULLABLE field in descriptor }
  SQL_NO_NULLS = 0;
  SQL_NULLABLE = 1;

{ Value returned by SQLGetTypeInfo() to denote that it is
 not known whether or not a data type supports null values. }

  SQL_NULLABLE_UNKNOWN = 2;
{
/* Values returned by SQLGetTypeInfo() to show WHERE clause
 * supported

#if (ODBCVER >= 0x0300)
#define SQL_PRED_NONE     0
#define SQL_PRED_CHAR     1
#define SQL_PRED_BASIC    2
#endif

/* values of UNNAMED field in descriptor */
#if (ODBCVER >= 0x0300)
#define SQL_NAMED           0
#define SQL_UNNAMED         1
#endif

/* values of ALLOC_TYPE field in descriptor */
#if (ODBCVER >= 0x0300)
#define SQL_DESC_ALLOC_AUTO 1
#define SQL_DESC_ALLOC_USER 2
#endif
}
  { FreeStmt() options }
  SQL_CLOSE        = 0;
  SQL_DROP         = 1;
  SQL_UNBIND       = 2;
  SQL_RESET_PARAMS = 3;

  { Codes used for FetchOrientation in SQLFetchScroll(),
   and in SQLDataSources() }
  SQL_FETCH_NEXT     = 1;
  SQL_FETCH_FIRST    = 2;
{$ifdef odbcver3}
  SQL_FETCH_FIRST_USER = 31;
  SQL_FETCH_FIRST_SYSTEM = 32;
{$endif}

  { Other codes used for FetchOrientation in SQLFetchScroll() }
  SQL_FETCH_LAST     = 3;
  SQL_FETCH_PRIOR    = 4;
  SQL_FETCH_ABSOLUTE = 5;
  SQL_FETCH_RELATIVE = 6;
{
/* SQLEndTran() options */
#define SQL_COMMIT          0
#define SQL_ROLLBACK        1}

//* null handles returned by SQLAllocHandle() */
  SQL_NULL_HENV       = SQLHENV(0);
  SQL_NULL_HDBC       = SQLHDBC(0);
  SQL_NULL_HSTMT      = SQLHSTMT(0);
{$ifdef odbcver3}
  SQL_NULL_HDESC      = SQLHDESC(0);
{$endif}

//* null handle used in place of parent handle when allocating HENV */
  SQL_NULL_HANDLE = SQLHANDLE(0);

//* Values that may appear in the result set of SQLSpecialColumns() */
  SQL_SCOPE_CURROW      = 0;
  SQL_SCOPE_TRANSACTION = 1;
  SQL_SCOPE_SESSION     = 2;

//* Column types and scopes in SQLSpecialColumns.  */
  SQL_BEST_ROWID        = 1;
  SQL_ROWVER            = 2;

{
#define SQL_PC_UNKNOWN      0
#if (ODBCVER >= 0x0300)
#define SQL_PC_NON_PSEUDO   1
#endif
#define SQL_PC_PSEUDO       2
}

//* Reserved value for the IdentifierType argument of SQLSpecialColumns() */
{$ifdef ODBCVER3}
  SQL_ROW_IDENTIFIER = 1;
{$endif}

//* Reserved values for UNIQUE argument of SQLStatistics() */
  SQL_INDEX_UNIQUE   = 0;
  SQL_INDEX_ALL      = 1;

//* Reserved values for RESERVED argument of SQLStatistics() */
  SQL_QUICK   = 0;
  SQL_ENSURE  = 1;

//* Values that may appear in the result set of SQLStatistics() */
  SQL_TABLE_STAT      = 0;
  SQL_INDEX_CLUSTERED = 1;
  SQL_INDEX_HASHED    = 2;
  SQL_INDEX_OTHER     = 3;
//  SQL_INDEX_BTREE     = ???;
//  SQL_INDEX_CONTENT   = ???;

{
/* Information requested by SQLGetInfo() */
#if (ODBCVER >= 0x0300)
#define SQL_MAX_DRIVER_CONNECTIONS           0
#define SQL_MAXIMUM_DRIVER_CONNECTIONS          SQL_MAX_DRIVER_CONNECTIONS
#define SQL_MAX_CONCURRENT_ACTIVITIES        1
#define SQL_MAXIMUM_CONCURRENT_ACTIVITIES       SQL_MAX_CONCURRENT_ACTIVITIES
#endif
<<<<<<< HEAD
<<<<<<< HEAD
#define SQL_FETCH_DIRECTION                  8
#define SQL_SEARCH_PATTERN_ESCAPE           14
#define SQL_ACCESSIBLE_TABLES               19
#define SQL_ACCESSIBLE_PROCEDURES           20
#define SQL_CURSOR_COMMIT_BEHAVIOR          23
#define SQL_DATA_SOURCE_READ_ONLY           25
#define SQL_DEFAULT_TXN_ISOLATION           26
=======
=======
>>>>>>> origin/fixes_2_2
#define SQL_DATA_SOURCE_NAME                 2
#define SQL_FETCH_DIRECTION                  8
#define SQL_SERVER_NAME                     13
#define SQL_SEARCH_PATTERN_ESCAPE           14
#define SQL_DBMS_NAME                       17
#define SQL_DBMS_VER                        18
#define SQL_ACCESSIBLE_TABLES               19
#define SQL_ACCESSIBLE_PROCEDURES               20
#define SQL_CURSOR_COMMIT_BEHAVIOR          23
#define SQL_DATA_SOURCE_READ_ONLY           25
#define SQL_DEFAULT_TXN_ISOLATION           26
#define SQL_IDENTIFIER_CASE                 28
#define SQL_IDENTIFIER_QUOTE_CHAR           29
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
#define SQL_MAX_COLUMN_NAME_LEN             30
#define SQL_MAXIMUM_COLUMN_NAME_LENGTH          SQL_MAX_COLUMN_NAME_LEN
#define SQL_MAX_CURSOR_NAME_LEN             31
#define SQL_MAXIMUM_CURSOR_NAME_LENGTH          SQL_MAX_CURSOR_NAME_LEN
#define SQL_MAX_SCHEMA_NAME_LEN             32
#define SQL_MAXIMUM_SCHEMA_NAME_LENGTH          SQL_MAX_SCHEMA_NAME_LEN
#define SQL_MAX_CATALOG_NAME_LEN            34
#define SQL_MAXIMUM_CATALOG_NAME_LENGTH         SQL_MAX_CATALOG_NAME_LEN
#define SQL_MAX_TABLE_NAME_LEN              35
}
<<<<<<< HEAD
<<<<<<< HEAD
  SQL_DATA_SOURCE_NAME                = 2;
  SQL_DRIVER_NAME                     = 6;
  SQL_DRIVER_VER                      = 7;
  SQL_SERVER_NAME                     = 13;
  SQL_DBMS_NAME                       = 17;
  SQL_DBMS_VER                        = 18;
  SQL_IDENTIFIER_CASE                 = 28;
  SQL_IDENTIFIER_QUOTE_CHAR           = 29;
  SQL_SCROLL_CONCURRENCY              = 43;
  SQL_TXN_CAPABLE                     = 46;
  SQL_TRANSACTION_CAPABLE             = SQL_TXN_CAPABLE;
=======
  SQL_SCROLL_CONCURRENCY              = 43;
  SQL_TXN_CAPABLE                     = 46;
  SQL_TRANSACTION_CAPABLE              = SQL_TXN_CAPABLE;
>>>>>>> graemeg/fixes_2_2
=======
  SQL_SCROLL_CONCURRENCY              = 43;
  SQL_TXN_CAPABLE                     = 46;
  SQL_TRANSACTION_CAPABLE              = SQL_TXN_CAPABLE;
>>>>>>> origin/fixes_2_2
  SQL_USER_NAME                       = 47;
  SQL_TXN_ISOLATION_OPTION            = 72;
  SQL_TRANSACTION_ISOLATION_OPTION    = SQL_TXN_ISOLATION_OPTION;
{
#define SQL_INTEGRITY                       73
#define SQL_GETDATA_EXTENSIONS              81
#define SQL_NULL_COLLATION                  85
#define SQL_ALTER_TABLE                     86
#define SQL_ORDER_BY_COLUMNS_IN_SELECT      90
#define SQL_SPECIAL_CHARACTERS              94
#define SQL_MAX_COLUMNS_IN_GROUP_BY         97
#define SQL_MAXIMUM_COLUMNS_IN_GROUP_BY         SQL_MAX_COLUMNS_IN_GROUP_BY
#define SQL_MAX_COLUMNS_IN_INDEX            98
#define SQL_MAXIMUM_COLUMNS_IN_INDEX            SQL_MAX_COLUMNS_IN_INDEX
#define SQL_MAX_COLUMNS_IN_ORDER_BY         99
#define SQL_MAXIMUM_COLUMNS_IN_ORDER_BY         SQL_MAX_COLUMNS_IN_ORDER_BY
#define SQL_MAX_COLUMNS_IN_SELECT          100
#define SQL_MAXIMUM_COLUMNS_IN_SELECT      SQL_MAX_COLUMNS_IN_SELECT
#define SQL_MAX_COLUMNS_IN_TABLE           101
#define SQL_MAX_INDEX_SIZE                 102
#define SQL_MAXIMUM_INDEX_SIZE                     SQL_MAX_INDEX_SIZE
#define SQL_MAX_ROW_SIZE                   104
#define SQL_MAXIMUM_ROW_SIZE                       SQL_MAX_ROW_SIZE
#define SQL_MAX_STATEMENT_LEN              105
#define SQL_MAXIMUM_STATEMENT_LENGTH       SQL_MAX_STATEMENT_LEN
#define SQL_MAX_TABLES_IN_SELECT           106
#define SQL_MAXIMUM_TABLES_IN_SELECT       SQL_MAX_TABLES_IN_SELECT
#define SQL_MAX_USER_NAME_LEN              107
#define SQL_MAXIMUM_USER_NAME_LENGTH       SQL_MAX_USER_NAME_LEN}
{$ifdef ODBCVER3}
  SQL_OJ_CAPABILITIES         = 115;
  SQL_OUTER_JOIN_CAPABILITIES = SQL_OJ_CAPABILITIES;
{$endif} { ODBCVER >= 0x0300 }

{$ifdef ODBCVER3}
  SQL_XOPEN_CLI_YEAR            = 10000;
  SQL_CURSOR_SENSITIVITY        = 10001;
  SQL_DESCRIBE_PARAMETER        = 10002;
  SQL_CATALOG_NAME              = 10003;
  SQL_COLLATION_SEQ             = 10004;
  SQL_MAX_IDENTIFIER_LEN        = 10005;
  SQL_MAXIMUM_IDENTIFIER_LENGTH = SQL_MAX_IDENTIFIER_LEN;
{$endif} { ODBCVER >= 0x0300 }

{/* SQL_ALTER_TABLE bitmasks */
#if (ODBCVER >= 0x0200)
#define SQL_AT_ADD_COLUMN                       0x00000001L
#define SQL_AT_DROP_COLUMN                      0x00000002L
#endif /* ODBCVER >= 0x0200 */

#if (ODBCVER >= 0x0300)
#define SQL_AT_ADD_CONSTRAINT                   0x00000008L

/* The following bitmasks are ODBC extensions and defined in sqlext.h
*#define        SQL_AT_COLUMN_SINGLE                                    0x00000020L
*#define        SQL_AT_ADD_COLUMN_DEFAULT                               0x00000040L
*#define        SQL_AT_ADD_COLUMN_COLLATION                             0x00000080L
*#define        SQL_AT_SET_COLUMN_DEFAULT                               0x00000100L
*#define        SQL_AT_DROP_COLUMN_DEFAULT                              0x00000200L
*#define        SQL_AT_DROP_COLUMN_CASCADE                              0x00000400L
*#define        SQL_AT_DROP_COLUMN_RESTRICT                             0x00000800L
*#define SQL_AT_ADD_TABLE_CONSTRAINT                            0x00001000L
*#define SQL_AT_DROP_TABLE_CONSTRAINT_CASCADE           0x00002000L
*#define SQL_AT_DROP_TABLE_CONSTRAINT_RESTRICT          0x00004000L
*#define SQL_AT_CONSTRAINT_NAME_DEFINITION                      0x00008000L
*#define SQL_AT_CONSTRAINT_INITIALLY_DEFERRED           0x00010000L
*#define SQL_AT_CONSTRAINT_INITIALLY_IMMEDIATE          0x00020000L
*#define SQL_AT_CONSTRAINT_DEFERRABLE                           0x00040000L
*#define SQL_AT_CONSTRAINT_NON_DEFERRABLE                       0x00080000L

#endif  /* ODBCVER >= 0x0300 */


/* SQL_ASYNC_MODE values */
#if (ODBCVER >= 0x0300)
#define SQL_AM_NONE                         0
#define SQL_AM_CONNECTION                   1
#define SQL_AM_STATEMENT                    2
#endif

/* SQL_CURSOR_COMMIT_BEHAVIOR values */
#define SQL_CB_DELETE                       0
#define SQL_CB_CLOSE                        1
#define SQL_CB_PRESERVE                     2

/* SQL_FETCH_DIRECTION bitmasks */
#define SQL_FD_FETCH_NEXT                   0x00000001L
#define SQL_FD_FETCH_FIRST                  0x00000002L
#define SQL_FD_FETCH_LAST                   0x00000004L
#define SQL_FD_FETCH_PRIOR                  0x00000008L
#define SQL_FD_FETCH_ABSOLUTE               0x00000010L
#define SQL_FD_FETCH_RELATIVE               0x00000020L

/* SQL_GETDATA_EXTENSIONS bitmasks */
#define SQL_GD_ANY_COLUMN                   0x00000001L
#define SQL_GD_ANY_ORDER                    0x00000002L
<<<<<<< HEAD
<<<<<<< HEAD
}
{ SQL_IDENTIFIER_CASE values }
  SQL_IC_UPPER                       = 1;
  SQL_IC_LOWER                       = 2;
  SQL_IC_SENSITIVE                   = 3;
  SQL_IC_MIXED                       = 4;
{
=======
=======
>>>>>>> origin/fixes_2_2

/* SQL_IDENTIFIER_CASE values */
#define SQL_IC_UPPER                        1
#define SQL_IC_LOWER                        2
#define SQL_IC_SENSITIVE                    3
#define SQL_IC_MIXED                        4

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
/* SQL_OJ_CAPABILITIES bitmasks */
/* NB: this means 'outer join', not what  you may be thinking */


#if (ODBCVER >= 0x0201)
#define SQL_OJ_LEFT                         0x00000001L
#define SQL_OJ_RIGHT                        0x00000002L
#define SQL_OJ_FULL                         0x00000004L
#define SQL_OJ_NESTED                       0x00000008L
#define SQL_OJ_NOT_ORDERED                  0x00000010L
#define SQL_OJ_INNER                        0x00000020L
#define SQL_OJ_ALL_COMPARISON_OPS           0x00000040L
#endif
}
{ SQL_SCROLL_CONCURRENCY bitmasks }
  SQL_SCCO_READ_ONLY                 = 1;
  SQL_SCCO_LOCK                      = 2;
  SQL_SCCO_OPT_ROWVER                = 4;
  SQL_SCCO_OPT_VALUES                = 8;

//* SQL_TXN_CAPABLE values */
   SQL_TC_NONE                       = 0;
   SQL_TC_DML                        = 1;
   SQL_TC_ALL                        = 2;
   SQL_TC_DDL_COMMIT                 = 3;
   SQL_TC_DDL_IGNORE                 = 4;

//* SQL_TXN_ISOLATION_OPTION bitmasks */
   SQL_TXN_READ_UNCOMMITTED         = 1;
   SQL_TRANSACTION_READ_UNCOMMITTED = SQL_TXN_READ_UNCOMMITTED;
   SQL_TXN_READ_COMMITTED           = 2;
   SQL_TRANSACTION_READ_COMMITTED   = SQL_TXN_READ_COMMITTED;
   SQL_TXN_REPEATABLE_READ          = 4;
   SQL_TRANSACTION_REPEATABLE_READ  = SQL_TXN_REPEATABLE_READ;
   SQL_TXN_SERIALIZABLE             = 8;
   SQL_TRANSACTION_SERIALIZABLE     = SQL_TXN_SERIALIZABLE;
{
/* SQL_NULL_COLLATION values */
#define SQL_NC_HIGH                         0
#define SQL_NC_LOW                          1

}

{ SQL_STATIC_SENSITIVITY values }

  SQL_SS_ADDITIONS = 1;
  SQL_SS_DELETIONS = 2;
  SQL_SS_UPDATES   = 4;

{ SQLColAttributes defines }
  SQL_COLUMN_COUNT               = 0;
  SQL_COLUMN_NAME                = 1;
  SQL_COLUMN_TYPE                = 2;
  SQL_COLUMN_LENGTH              = 3;
  SQL_COLUMN_PRECISION           = 4;
  SQL_COLUMN_SCALE               = 5;
  SQL_COLUMN_DISPLAY_SIZE        = 6;
  SQL_COLUMN_NULLABLE            = 7;
  SQL_COLUMN_UNSIGNED            = 8;
  SQL_COLUMN_MONEY               = 9;
  SQL_COLUMN_UPDATABLE           = 10;
  SQL_COLUMN_AUTO_INCREMENT      = 11;
  SQL_COLUMN_CASE_SENSITIVE      = 12;
  SQL_COLUMN_SEARCHABLE          = 13;
  SQL_COLUMN_TYPE_NAME           = 14;
  SQL_COLUMN_TABLE_NAME          = 15;
  SQL_COLUMN_OWNER_NAME          = 16;
  SQL_COLUMN_QUALIFIER_NAME      = 17;
  SQL_COLUMN_LABEL               = 18;
  SQL_COLATT_OPT_MAX             = SQL_COLUMN_LABEL;
{$ifdef ODBCVER3}
  SQL_COLUMN_DRIVER_START        = 1000;
{$endif} { ODBCVER >= 0x0300 }

<<<<<<< HEAD
<<<<<<< HEAD
{ SQLColAttribute defines }
=======
 { SQLColAttribute defines }
>>>>>>> graemeg/fixes_2_2
=======
 { SQLColAttribute defines }
>>>>>>> origin/fixes_2_2
{$ifdef ODBCVER3}
  SQL_DESC_ARRAY_SIZE	     = 20;
  SQL_DESC_ARRAY_STATUS_PTR  = 21;
  SQL_DESC_AUTO_UNIQUE_VALUE = SQL_COLUMN_AUTO_INCREMENT;
  SQL_DESC_BASE_COLUMN_NAME  = 22;
  SQL_DESC_BASE_TABLE_NAME   = 23;
  SQL_DESC_BIND_OFFSET_PTR   = 24;
  SQL_DESC_BIND_TYPE         = 25;
  SQL_DESC_CASE_SENSITIVE    = SQL_COLUMN_CASE_SENSITIVE;
  SQL_DESC_CATALOG_NAME      = SQL_COLUMN_QUALIFIER_NAME;
  SQL_DESC_CONCISE_TYPE      = SQL_COLUMN_TYPE;
  SQL_DESC_DATETIME_INTERVAL_PRECISION = 26;
  SQL_DESC_DISPLAY_SIZE      = SQL_COLUMN_DISPLAY_SIZE;
  SQL_DESC_FIXED_PREC_SCALE  = SQL_COLUMN_MONEY;
  SQL_DESC_LABEL             = SQL_COLUMN_LABEL;
  SQL_DESC_LITERAL_PREFIX    = 27;
  SQL_DESC_LITERAL_SUFFIX    = 28;
  SQL_DESC_LOCAL_TYPE_NAME   = 29;
  SQL_DESC_MAXIMUM_SCALE     = 30;
  SQL_DESC_MINIMUM_SCALE     = 31;
  SQL_DESC_NUM_PREC_RADIX    = 32;
  SQL_DESC_PARAMETER_TYPE    = 33;
  SQL_DESC_ROWS_PROCESSED_PTR = 34;
  SQL_DESC_SCHEMA_NAME       = SQL_COLUMN_OWNER_NAME;
  SQL_DESC_SEARCHABLE        = SQL_COLUMN_SEARCHABLE;
  SQL_DESC_TYPE_NAME         = SQL_COLUMN_TYPE_NAME;
  SQL_DESC_TABLE_NAME        = SQL_COLUMN_TABLE_NAME;
  SQL_DESC_UNSIGNED          = SQL_COLUMN_UNSIGNED;
  SQL_DESC_UPDATABLE         = SQL_COLUMN_UPDATABLE;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
{ SQLColAttributes subdefines for SQL_COLUMN_UPDATABLE }
  SQL_ATTR_READONLY          = 0;
  SQL_ATTR_WRITE             = 1;
  SQL_ATTR_READWRITE_UNKNOWN = 2;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//* SQLEndTran() options */
  SQL_COMMIT    = 0;
  SQL_ROLLBACK  = 1;

  SQL_ATTR_ROW_ARRAY_SIZE = 27;

//* SQLConfigDataSource() options */
  ODBC_ADD_DSN = 1;
  ODBC_CONFIG_DSN = 2;
  ODBC_REMOVE_DSN = 3;
  ODBC_ADD_SYS_DSN = 4;
  ODBC_CONFIG_SYS_DSN = 5;
  ODBC_REMOVE_SYS_DSN = 6;

{$ifdef DYNLOADINGODBC}

type   tSQLAllocHandle =function(HandleType: SQLSMALLINT;
           InputHandle:SQLHANDLE;Var OutputHandlePtr: SQLHANDLE):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   tSQLSetEnvAttr=function (EnvironmentHandle:SQLHENV;
           Attribute:SQLINTEGER;Value:SQLPOINTER;
           StringLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLFreeHandle=function (HandleType:SQLSMALLINT;
           Handle:SQLHANDLE):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLGetDiagRec=function (HandleType:SQLSMALLINT;
           Handle:SQLHANDLE;RecNumber:SQLSMALLINT;
           Sqlstate:PSQLCHAR;var NativeError:SQLINTEGER;
           MessageText:PSQLCHAR;BufferLength:SQLSMALLINT;
           var TextLength:SQLSMALLINT ):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
<<<<<<< HEAD
<<<<<<< HEAD

=======
           
>>>>>>> graemeg/fixes_2_2
=======
           
>>>>>>> origin/fixes_2_2
type   TSQLGetDiagField=function (HandleType:SQLSMALLINT;
           Handle:SQLHANDLE;RecNumber:SQLSMALLINT;
           DiagIdentifier:SQLSMALLINT;DiagInfoPtr:SQLPOINTER;
           BufferLength:SQLSMALLINT;var StringLengthPtr:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLConnect=function (ConnectionHandle:SQLHDBC;
           ServerName:PSQLCHAR;NameLength1:SQLSMALLINT;
           UserName:PSQLCHAR;NameLength2:SQLSMALLINT;
           Authentication:PSQLCHAR;NameLength3:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLDisconnect=function(ConnectionHandle:SQLHDBC):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLDriverConnect=function (hdbc: SQLHDBC;
           hwnd: SQLHWND;szCsin: PChar;
           szCLen: SQLSMALLINT;szCsout: PChar;
           cbCSMax: SQLSMALLINT;Var cbCsOut: SQLSMALLINT;
           f: SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLExecDirect=function (StatementHandle:SQLHSTMT;
           StatementText:PSQLCHAR;TextLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLPrepare=function (StatementHandle:SQLHSTMT;
           StatementText:PSQLCHAR;TextLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLCloseCursor=function (StatementHandle:SQLHSTMT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLExecute=function (StatementHandle:SQLHSTMT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLFetch=function (StatementHandle:SQLHSTMT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLNumResultCols=function (StatementHandle:SQLHSTMT;
           var ColumnCount:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLDescribeCol=function (StatementHandle:SQLHSTMT;
           ColumnNumber:SQLUSMALLINT;ColumnName:PSQLCHAR;
           BufferLength:SQLSMALLINT;var NameLength:SQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
           var DataType:SQLSMALLINT;var ColumnSize:SQLULEN;
           var DecimalDigits:SQLSMALLINT;var Nullable:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLFetchScroll=function (StatementHandle:SQLHSTMT;
           FetchOrientation:SQLSMALLINT;FetchOffset:SQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLExtendedFetch=function (hstmt:SQLHSTMT;
           fFetchType:SQLUSMALLINT;irow:SQLLEN;
           pcrow:PSQLULEN;rgfRowStatus:PSQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLGetData=function (StatementHandle:SQLHSTMT;
           ColumnNumber:SQLUSMALLINT;TargetType:SQLSMALLINT;
           TargetValue:SQLPOINTER;BufferLength:SQLLEN;
           StrLen_or_Ind:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
=======
=======
>>>>>>> origin/fixes_2_2
           var DataType:SQLSMALLINT;var ColumnSize:SQLUINTEGER;
           var DecimalDigits:SQLSMALLINT;var Nullable:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLFetchScroll=function (StatementHandle:SQLHSTMT;
           FetchOrientation:SQLSMALLINT;FetchOffset:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLExtendedFetch=function (hstmt:SQLHSTMT;
           fFetchType:SQLUSMALLINT;irow:SQLINTEGER;
           pcrow:PSQLUINTEGER;rgfRowStatus:PSQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLGetData=function (StatementHandle:SQLHSTMT;
           ColumnNumber:SQLUSMALLINT;TargetType:SQLSMALLINT;
           TargetValue:SQLPOINTER;BufferLength:SQLINTEGER;
           StrLen_or_Ind:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type   TSQLSetStmtAttr=function (StatementHandle:SQLHSTMT;
           Attribute:SQLINTEGER;Value:SQLPOINTER;
           StringLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLGetStmtAttr=function (StatementHandle:SQLHSTMT;
           Attribute:SQLINTEGER;Value:SQLPOINTER;
           BufferLength:SQLINTEGER;StringLength:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

<<<<<<< HEAD
<<<<<<< HEAD
type   TSQLSetDescField=function (DescriptorHandle:SQLHDESC;
           RecNumber:SQLSMALLINT; FieldIdentifier:SQLSMALLINT;
           ValuePtr:SQLPOINTER; BufferLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLSetDescRec=function (DescriptorHandle:SQLHDESC;
           RecNumber:SQLSMALLINT; DescType, SubType:SQLSMALLINT;
           Length:SQLLEN; Precision, Scale: SQLSMALLINT;
           DataPtr:SQLPOINTER; StringLengthPtr,IndicatorPtr:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type   tSQLGetInfo=function (ConnectionHandle:SQLHDBC;
           InfoType:SQLUSMALLINT;InfoValue:SQLPOINTER;
           BufferLength:SQLSMALLINT;StringLength:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLBulkOperations=function (StatementHandle: SQLHSTMT;
           Operation:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLPutData=function (StatementHandle:SQLHSTMT;
<<<<<<< HEAD
<<<<<<< HEAD
           Data:SQLPOINTER;StrLen_or_Ind:SQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLBindCol=function (StatementHandle:SQLHSTMT;
           ColumnNumber:SQLUSMALLINT;TargetType:SQLSMALLINT;
           TargetValue:SQLPOINTER;BufferLength:SQLLEN;
           StrLen_or_Ind:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLSetPos=function (StatementHandle:SQLHSTMT;
           RowNumber:SQLSETPOSIROW; Operation:SQLUSMALLINT;
           LockType:SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
=======
=======
>>>>>>> origin/fixes_2_2
           Data:SQLPOINTER;StrLen_or_Ind:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLBindCol=function (StatementHandle:SQLHSTMT;
           ColumnNumber:SQLUSMALLINT;TargetType:SQLSMALLINT;
           TargetValue:SQLPOINTER;BufferLength:SQLINTEGER;
           StrLen_or_Ind:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLSetPos=function (hstmt:SQLHSTMT;
           irow:SQLUSMALLINT;fOption:SQLUSMALLINT;
           fLock:SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type   TSQLDataSources=function (EnvironmentHandle:SQLHENV;
           Direction:SQLUSMALLINT;ServerName:PSQLCHAR;
           BufferLength1:SQLSMALLINT;NameLength1:PSQLSMALLINT;
           Description:PSQLCHAR;BufferLength2:SQLSMALLINT;
           NameLength2:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLDrivers=function (EnvironmentHandle:SQLHENV;
           Direction:SQLUSMALLINT;DriverDescription:PSQLCHAR;
           BufferLength1:SQLSMALLINT;DescriptionLength1:PSQLSMALLINT;
           DriverAttributes:PSQLCHAR;BufferLength2:SQLSMALLINT;
           AttributesLength2:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLSetConnectAttr=function (ConnectionHandle:SQLHDBC;
           Attribute:SQLINTEGER; Value:SQLPOINTER;
           StringLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLGetCursorName=function (StatementHandle:SQLHSTMT;
           CursorName:PSQLCHAR; BufferLength:SQLSMALLINT;
           NameLength:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLSetCursorName=function (StatementHandle:SQLHSTMT;
           CursorName:PSQLCHAR; NameLength:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLRowCount=function (StatementHandle:SQLHSTMT;
<<<<<<< HEAD
<<<<<<< HEAD
           var RowCount:SQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
=======
           Var RowCount:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
>>>>>>> graemeg/fixes_2_2
=======
           Var RowCount:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
>>>>>>> origin/fixes_2_2

type   TSQLBindParameter=function (hstmt:SQLHSTMT;
           ipar:SQLUSMALLINT;fParamType:SQLSMALLINT;
           fCType:SQLSMALLINT;fSqlType:SQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
           cbColDef:SQLULEN;ibScale:SQLSMALLINT;
           rgbValue:SQLPOINTER;cbValueMax:SQLLEN;
           pcbValue:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
=======
           cbColDef:SQLUINTEGER;ibScale:SQLSMALLINT;
           rgbValue:SQLPOINTER;cbValueMax:SQLINTEGER;
           pcbValue:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
>>>>>>> graemeg/fixes_2_2
=======
           cbColDef:SQLUINTEGER;ibScale:SQLSMALLINT;
           rgbValue:SQLPOINTER;cbValueMax:SQLINTEGER;
           pcbValue:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
>>>>>>> origin/fixes_2_2

type   TSQLFreeStmt=function (StatementHandle:SQLHSTMT;
           Option:SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLColAttribute=function (StatementHandle:SQLHSTMT;
           ColumnNumber:SQLUSMALLINT;FieldIdentifier:SQLUSMALLINT;
           CharacterAttribute:PSQLCHAR;BufferLength:SQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
           StringLength:PSQLSMALLINT;NumericAttribute:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
=======
           StringLength:PSQLSMALLINT;NumericAttribute:SQLPOINTER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
>>>>>>> graemeg/fixes_2_2
=======
           StringLength:PSQLSMALLINT;NumericAttribute:SQLPOINTER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
>>>>>>> origin/fixes_2_2

type   TSQLEndTran=function (HandleType:SQLSMALLINT;
           Handle:SQLHANDLE;CompletionType:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLTables=function ( hstmt : SQLHSTMT;
           szTableQualifier : PSQLCHAR;cbTableQualifier : SQLSMALLINT;
           szTableOwner : PSQLCHAR;cbTableOwner : SQLSMALLINT;
           szTableName : PSQLCHAR;cbTableName : SQLSMALLINT;
           szTableType : PSQLCHAR;cbTableType : SQLSMALLINT ) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLColumns=function ( hstmt : SQLHSTMT;
           szTableQualifier : PSQLCHAR;cbTableQualifier : SQLSMALLINT;
           szTableOwner : PSQLCHAR;cbTableOwner : SQLSMALLINT;
           szTableName : PSQLCHAR;cbTableName : SQLSMALLINT;
           szColumnName : PSQLCHAR;cbColumnName : SQLSMALLINT ) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLSpecialColumns=function (StatementHandle:SQLHSTMT;
           IdentifierType:SQLUSMALLINT;CatalogName:PSQLCHAR;
           NameLength1:SQLSMALLINT;SchemaName:PSQLCHAR;
           NameLength2:SQLSMALLINT;TableName:PSQLCHAR;
           NameLength3:SQLSMALLINT;Scope:SQLUSMALLINT;
           Nullable:SQLUSMALLINT) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLProcedures=function ( hstmt : SQLHSTMT;
           szTableQualifier : PSQLCHAR;cbTableQualifier : SQLSMALLINT;
           szTableOwner : PSQLCHAR;cbTableOwner : SQLSMALLINT;
           szTableName : PSQLCHAR;cbTableName : SQLSMALLINT ) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};

type   TSQLPrimaryKeys=function (hstmt : SQLHSTMT;
           CatalogName:PSQLCHAR;NameLength1:SQLSMALLINT;
           SchemaName:PSQLCHAR;NameLength2:SQLSMALLINT;
           TableName:PSQLCHAR;NameLength3:SQLSMALLINT ):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};
type   TSQLProcedureColumns = function(hstmt: SQLHSTMT;
           CatalogName: PSQLCHAR; NameLength1: SQLSMALLINT;
           SchemaName: PSQLCHAR; NameLength2: SQLSMALLINT;
           ProcName: PSQLCHAR; NameLength3: SQLSMALLINT;
           ColumnName: PSQLCHAR; NameLength4: SQLSMALLINT): SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};
type   TSQLStatistics = function (hstmt: SQLHSTMT;
           CatalogName:PSQLCHAR; NameLength1:SQLSMALLINT;
           SchemaName:PSQLCHAR; NameLength2:SQLSMALLINT;
           TableName:PSQLCHAR; NameLength3:SQLSMALLINT;
           Unique:SQLUSMALLINT;
           Reserved:SQLUSMALLINT): SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};

var    SQLAllocHandle:tSQLAllocHandle;
var    SQLSetEnvAttr:tSQLSetEnvAttr;
var    SQLFreeHandle:tSQLFreeHandle;
var    SQLGetInfo:tSQLGetInfo;
var    SQLProcedures:TSQLProcedures;
var    SQLColumns:TSQLColumns;
var    SQLSpecialColumns:TSQLSpecialColumns;
var    SQLGetDiagRec:TSQLGetDiagRec;
var    SQLGetDiagField:TSQLGetDiagField;
var    SQLConnect:TSQLConnect;
var    SQLDisconnect:TSQLDisconnect;
var    SQLDriverConnect:TSQLDriverConnect;
var    SQLExecDirect:TSQLExecDirect;
var    SQLPrepare:TSQLPrepare;
var    SQLCloseCursor:TSQLCloseCursor;
var    SQLExecute:TSQLExecute;
var    SQLFetch:TSQLFetch;
var    SQLNumResultCols:TSQLNumResultCols;
var    SQLDescribeCol:TSQLDescribeCol;
var    SQLFetchScroll:TSQLFetchScroll;
var    SQLExtendedFetch:TSQLExtendedFetch;
var    SQLGetData:TSQLGetData;
var    SQLSetStmtAttr:TSQLSetStmtAttr;
var    SQLGetStmtAttr:TSQLGetStmtAttr;
<<<<<<< HEAD
<<<<<<< HEAD
//var    SQLSetDescField:TSQLSetDescField;
var    SQLSetDescRec:TSQLSetDescRec;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var    SQLBulkOperations:TSQLBulkOperations;
var    SQLPutData:TSQLPutData;
var    SQLBindCol:TSQLBindCol;
var    SQLSetPos:TSQLSetPos;
var    SQLDataSources:TSQLDataSources;
var    SQLDrivers:TSQLDrivers;
var    SQLSetConnectAttr:TSQLSetConnectAttr;
var    SQLGetCursorName:TSQLGetCursorName;
var    SQLSetCursorName:TSQLSetCursorName;
var    SQLRowCount:TSQLRowCount;
var    SQLBindParameter:TSQLBindParameter;
var    SQLFreeStmt:TSQLFreeStmt;
var    SQLColAttribute:TSQLColAttribute;
var    SQLEndTran:TSQLEndTran;
var    SQLTables:TSQLTables;
var    SQLPrimaryKeys:TSQLPrimaryKeys;
var    SQLProcedureColumns : TSQLProcedureColumns;
var    SQLStatistics: TSQLStatistics;
var    odbcversion:word;

{$else}

  function SQLAllocHandle(
               HandleType: SQLSMALLINT;
               InputHandle:SQLHANDLE;
               Var OutputHandlePtr: SQLHANDLE):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
  function SQLSetEnvAttr(
               EnvironmentHandle:SQLHENV;
               Attribute:        SQLINTEGER;
               Value:            SQLPOINTER;
               StringLength:     SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetEnvAttr(
               EnvironmentHandle:SQLHENV;
               Attribute:SQLINTEGER;
               Value:SQLPOINTER;
               BufferLength:SQLINTEGER;
               StringLength:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLFreeHandle(
               HandleType: SQLSMALLINT;
               Handle:     SQLHANDLE):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetDiagRec(
               HandleType:   SQLSMALLINT;
               Handle:       SQLHANDLE;
               RecNumber:    SQLSMALLINT;
               Sqlstate:     PSQLCHAR;
               var NativeError: SQLINTEGER;
               MessageText:     PSQLCHAR;
               BufferLength:    SQLSMALLINT;
               var TextLength:  SQLSMALLINT ):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetDiagField(
               HandleType:SQLSMALLINT;
               Handle:SQLHANDLE;
               RecNumber:SQLSMALLINT;
               DiagIdentifier:SQLSMALLINT;
               DiagInfoPtr:SQLPOINTER;
               BufferLength:SQLSMALLINT;
               var StringLengthPtr:SQLSMALLINT ):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLConnect(
               ConnectionHandle:SQLHDBC;
               ServerName:PSQLCHAR;    NameLength1:SQLSMALLINT;
               UserName:PSQLCHAR;      NameLength2:SQLSMALLINT;
               Authentication:PSQLCHAR;NameLength3:SQLSMALLINT
              ):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLDisconnect(
               ConnectionHandle:SQLHDBC):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLDriverConnect(
               hdbc: SQLHDBC;
               hwnd: SQLHWND;
               szCsin: PChar;
               szCLen: SQLSMALLINT;
               szCsout: PChar;
               cbCSMax: SQLSMALLINT;
               Var cbCsOut: SQLSMALLINT;
               f: SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLBrowseConnect(
               hdbc : SQLHDBC;
               szConnStrIn :PSQLCHAR;
               cbConnStrIn: SQLSMALLINT;
               szConnStrOut : PSQLCHAR;
               cbConnStrOutMax : SQLSMALLINT;
               Var cbConnStrOut : SQLSMALLINT) : SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLExecDirect(
               StatementHandle:SQLHSTMT;
               StatementText:  PSQLCHAR;
               TextLength:     SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLPrepare(
               StatementHandle:SQLHSTMT;
               StatementText:PSQLCHAR;
               TextLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLCloseCursor(
               StatementHandle:SQLHSTMT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLExecute(
               StatementHandle:SQLHSTMT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLFetch(
               StatementHandle:SQLHSTMT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLNumResultCols(
               StatementHandle:SQLHSTMT;
               var ColumnCount:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLDescribeCol(
               StatementHandle:SQLHSTMT;
               ColumnNumber:SQLUSMALLINT;
               ColumnName:PSQLCHAR;
               BufferLength:SQLSMALLINT;
               var NameLength:SQLSMALLINT;
               var DataType:SQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
               var ColumnSize:SQLULEN;
=======
               var ColumnSize:SQLUINTEGER;
>>>>>>> graemeg/fixes_2_2
=======
               var ColumnSize:SQLUINTEGER;
>>>>>>> origin/fixes_2_2
               var DecimalDigits:SQLSMALLINT;
               var Nullable:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLFetchScroll(
               StatementHandle:SQLHSTMT;
               FetchOrientation:SQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
               FetchOffset:SQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLExtendedFetch(
               hstmt:SQLHSTMT;
               fFetchType:SQLUSMALLINT;
               irow:SQLLEN;
               pcrow:PSQLULEN;
=======
=======
>>>>>>> origin/fixes_2_2
               FetchOffset:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLExtendedFetch(
               hstmt:SQLHSTMT;
               fFetchType:SQLUSMALLINT;
               irow:SQLINTEGER;
               pcrow:PSQLUINTEGER;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
               rgfRowStatus:PSQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetData(
               StatementHandle:SQLHSTMT;
               ColumnNumber:SQLUSMALLINT;
               TargetType:SQLSMALLINT;
               TargetValue:SQLPOINTER;
<<<<<<< HEAD
<<<<<<< HEAD
               BufferLength:SQLLEN;
               StrLen_or_Ind:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
=======
               BufferLength:SQLINTEGER;
               StrLen_or_Ind:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> graemeg/fixes_2_2
=======
               BufferLength:SQLINTEGER;
               StrLen_or_Ind:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> origin/fixes_2_2
   function SQLSetStmtAttr(
               StatementHandle:SQLHSTMT;
               Attribute:SQLINTEGER;
               Value:SQLPOINTER;
               StringLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetStmtAttr(
               StatementHandle:SQLHSTMT;
               Attribute:SQLINTEGER;
               Value:SQLPOINTER;
               BufferLength:SQLINTEGER;
               StringLength:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetInfo(
               ConnectionHandle:SQLHDBC;
               InfoType:SQLUSMALLINT;
               InfoValue:SQLPOINTER;
               BufferLength:SQLSMALLINT;
               StringLength:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLBulkOperations(
               StatementHandle: SQLHSTMT;
               Operation:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLPutData(
               StatementHandle:SQLHSTMT;
               Data:SQLPOINTER;
<<<<<<< HEAD
<<<<<<< HEAD
               StrLen_or_Ind:SQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
=======
               StrLen_or_Ind:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> graemeg/fixes_2_2
=======
               StrLen_or_Ind:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> origin/fixes_2_2
   function SQLBindCol(
               StatementHandle:SQLHSTMT;
               ColumnNumber:SQLUSMALLINT;
               TargetType:SQLSMALLINT;
               TargetValue:SQLPOINTER;
<<<<<<< HEAD
<<<<<<< HEAD
               BufferLength:SQLLEN;
               StrLen_or_Ind:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLSetPos(
               StatementHandle:SQLHSTMT;
               RowNumber:SQLSETPOSIROW;
               Operation:SQLUSMALLINT;
               LockType:SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
=======
=======
>>>>>>> origin/fixes_2_2
               BufferLength:SQLINTEGER;
               StrLen_or_Ind:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLSetPos(
               hstmt:SQLHSTMT;
               irow:SQLUSMALLINT;
               fOption:SQLUSMALLINT;
               fLock:SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   function SQLDataSources(
               EnvironmentHandle:SQLHENV;
               Direction:SQLUSMALLINT;
               ServerName:PSQLCHAR;
               BufferLength1:SQLSMALLINT;
               NameLength1:PSQLSMALLINT;
               Description:PSQLCHAR;
               BufferLength2:SQLSMALLINT;
               NameLength2:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLDrivers(
               EnvironmentHandle:SQLHENV;
               Direction:SQLUSMALLINT;
               DriverDescription:PSQLCHAR;
               BufferLength1:SQLSMALLINT;
               DescriptionLength1:PSQLSMALLINT;
               DriverAttributes:PSQLCHAR;
               BufferLength2:SQLSMALLINT;
               AttributesLength2:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLSetConnectAttr(
               ConnectionHandle:SQLHDBC;
               Attribute:SQLINTEGER; Value:SQLPOINTER;
               StringLength:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLGetCursorName(
               StatementHandle:SQLHSTMT;
               CursorName:PSQLCHAR; BufferLength:SQLSMALLINT;
               NameLength:PSQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLSetCursorName(
               StatementHandle:SQLHSTMT;
               CursorName:PSQLCHAR; NameLength:SQLSMALLINT
               ):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLRowCount(
               StatementHandle:SQLHSTMT;
<<<<<<< HEAD
<<<<<<< HEAD
               var RowCount:SQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
=======
               Var RowCount:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> graemeg/fixes_2_2
=======
               Var RowCount:SQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> origin/fixes_2_2
   function SQLBindParameter(
               hstmt:SQLHSTMT;
               ipar:SQLUSMALLINT;
               fParamType:SQLSMALLINT;
               fCType:SQLSMALLINT;
               fSqlType:SQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
               cbColDef:SQLULEN;
               ibScale:SQLSMALLINT;
               rgbValue:SQLPOINTER;
               cbValueMax:SQLLEN;
               pcbValue:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
=======
=======
>>>>>>> origin/fixes_2_2
               cbColDef:SQLUINTEGER;
               ibScale:SQLSMALLINT;
               rgbValue:SQLPOINTER;
               cbValueMax:SQLINTEGER;
               pcbValue:PSQLINTEGER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   function SQLFreeStmt(
               StatementHandle:SQLHSTMT;
               Option:SQLUSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLColAttribute (
               StatementHandle:SQLHSTMT;
               ColumnNumber:SQLUSMALLINT;
               FieldIdentifier:SQLUSMALLINT;
               CharacterAttribute:PSQLCHAR;
               BufferLength:SQLSMALLINT;
               StringLength:PSQLSMALLINT;
<<<<<<< HEAD
<<<<<<< HEAD
               NumericAttribute:PSQLLEN):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
=======
               NumericAttribute:SQLPOINTER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> graemeg/fixes_2_2
=======
               NumericAttribute:SQLPOINTER):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
>>>>>>> origin/fixes_2_2
{$ifdef ODBCVER3}
   function SQLEndTran(
               HandleType:SQLSMALLINT;
               Handle:SQLHANDLE;
               CompletionType:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
{$endif}
   function SQLTables( hstmt : SQLHSTMT;
                szTableQualifier : PSQLCHAR;
                cbTableQualifier : SQLSMALLINT;
                szTableOwner : PSQLCHAR;
                cbTableOwner : SQLSMALLINT;
                szTableName : PSQLCHAR;
                cbTableName : SQLSMALLINT;
                szTableType : PSQLCHAR;
                cbTableType : SQLSMALLINT ) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif}; external odbclib;
   function SQLColumns( hstmt : SQLHSTMT;
                szTableQualifier : PSQLCHAR;
                cbTableQualifier : SQLSMALLINT;
                szTableOwner : PSQLCHAR;
                cbTableOwner : SQLSMALLINT;
                szTableName : PSQLCHAR;
                cbTableName : SQLSMALLINT;
                szColumnName : PSQLCHAR;
                cbColumnName : SQLSMALLINT ) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif}; external odbclib;
   function SQLSpecialColumns(StatementHandle:SQLHSTMT;
                IdentifierType:SQLUSMALLINT;
                CatalogName:PSQLCHAR;
                NameLength1:SQLSMALLINT;
                SchemaName:PSQLCHAR;
                NameLength2:SQLSMALLINT;
                TableName:PSQLCHAR;
                NameLength3:SQLSMALLINT;
                Scope:SQLUSMALLINT;
                Nullable:SQLUSMALLINT) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif}; external odbclib;
   function SQLProcedures( hstmt : SQLHSTMT;
                szTableQualifier : PSQLCHAR;
                cbTableQualifier : SQLSMALLINT;
                szTableOwner : PSQLCHAR;
                cbTableOwner : SQLSMALLINT;
                szTableName : PSQLCHAR;
                cbTableName : SQLSMALLINT ) : SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif}; external odbclib;
   function SQLPrimaryKeys(hstmt : SQLHSTMT;
                CatalogName:PSQLCHAR;NameLength1:SQLSMALLINT;
                SchemaName:PSQLCHAR;NameLength2:SQLSMALLINT;
                TableName:PSQLCHAR;
                NameLength3:SQLSMALLINT):SQLRETURN;{$ifdef fpc} extdecl {$else} stdcall {$endif};external odbclib;
   function SQLProcedureColumns(hstmt: SQLHSTMT;
                CatalogName: PSQLCHAR; NameLength1: SQLSMALLINT;
                SchemaName: PSQLCHAR; NameLength2: SQLSMALLINT;
                ProcName: PSQLCHAR; NameLength3: SQLSMALLINT;
                ColumnName: PSQLCHAR; NameLength4: SQLSMALLINT): SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif};
                external odbclib;
   function SQLStatistics(hstmt: SQLHSTMT;
                CatalogName:PSQLCHAR; NameLength1:SQLSMALLINT;
                SchemaName:PSQLCHAR; NameLength2:SQLSMALLINT;
                TableName:PSQLCHAR; NameLength3:SQLSMALLINT;
                Unique:SQLUSMALLINT;
                Reserved:SQLUSMALLINT): SQLRETURN; {$ifdef fpc} extdecl {$else} stdcall {$endif}; external odbclib;
{$endif}
// This function always load dynamic

function DateStructToDateTime( b:PSQL_DATE_STRUCT):TDateTime;
function DateTimeToDateStruct( b:TDateTime):SQL_DATE_STRUCT;
procedure DateTime2TimeStampStruct( var Value:SQL_TIMESTAMP_STRUCT; b:TDateTime);
Function TimeStampStructToDateTime( B :  PSQL_TIMESTAMP_STRUCT) : TDateTime;
Function TimeStructToDateTime (B : PSQL_TIME_STRUCT) : TDateTime;
<<<<<<< HEAD
<<<<<<< HEAD
function DateTimeToTimeStruct(b: TDateTime) : SQL_TIME_STRUCT;
function CurrToNumericStruct(c: currency): SQL_NUMERIC_STRUCT;


{$IFDEF DYNLOADINGODBC}
Procedure InitialiseODBC(OverrideName : string ='');
=======
=======
>>>>>>> origin/fixes_2_2


{$IFDEF DYNLOADINGODBC}
Procedure InitialiseODBC;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Procedure ReleaseODBC;

var ODBCLibraryHandle : TLibHandle;
{$ENDIF}

implementation

{$IFDEF DYNLOADINGODBC}

var RefCount : integer;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure InitialiseODBC(OverrideName : string ='');

var libname : string;
=======
Procedure InitialiseODBC;
>>>>>>> graemeg/fixes_2_2
=======
Procedure InitialiseODBC;
>>>>>>> origin/fixes_2_2

begin
  inc(RefCount);
  if RefCount = 1 then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if OverrideName='' then
=======
    if OverrideName='' then 
>>>>>>> graemeg/cpstrnew
=======
    if OverrideName='' then 
>>>>>>> graemeg/cpstrnew
=======
    if OverrideName='' then 
>>>>>>> graemeg/cpstrnew
=======
    if OverrideName='' then 
>>>>>>> origin/cpstrnew
       libname:=odbclib
     else
       libname:=OverrideName;
    ODBCLibraryHandle := loadlibrary(libname);
=======
    ODBCLibraryHandle := loadlibrary(odbclib);
>>>>>>> graemeg/fixes_2_2
=======
    ODBCLibraryHandle := loadlibrary(odbclib);
>>>>>>> origin/fixes_2_2
    if ODBCLibraryHandle = nilhandle then
      begin
      RefCount := 0;
      Raise EInOutError.Create('Can not load ODBC client. Is it installed? ('+odbclib+')');
      end;

{$ifdef fpc}
    pointer(SQLAllocHandle) := GetProcedureAddress(ODBCLibraryHandle,'SQLAllocHandle');
    pointer(SQLSetEnvAttr) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetEnvAttr');
    pointer(SQLFreeHandle) := GetProcedureAddress(ODBCLibraryHandle,'SQLFreeHandle');
    pointer(SQLGetInfo) := GetProcedureAddress(ODBCLibraryHandle,'SQLGetInfo');
    pointer(SQLProcedures) := GetProcedureAddress(ODBCLibraryHandle,'SQLProcedures');
    pointer(SQLColumns) := GetProcedureAddress(ODBCLibraryHandle,'SQLColumns');
    pointer(SQLSpecialColumns) := GetProcedureAddress(ODBCLibraryHandle,'SQLSpecialColumns');
    pointer(SQLGetDiagRec) := GetProcedureAddress(ODBCLibraryHandle,'SQLGetDiagRec');
    pointer(SQLGetDiagField) := GetProcedureAddress(ODBCLibraryHandle,'SQLGetDiagField');
    pointer(SQLConnect) := GetProcedureAddress(ODBCLibraryHandle,'SQLConnect');
    pointer(SQLDisconnect) := GetProcedureAddress(ODBCLibraryHandle,'SQLDisconnect');
    pointer(SQLDriverConnect) := GetProcedureAddress(ODBCLibraryHandle,'SQLDriverConnect');
    pointer(SQLExecDirect) := GetProcedureAddress(ODBCLibraryHandle,'SQLExecDirect');
    pointer(SQLPrepare) := GetProcedureAddress(ODBCLibraryHandle,'SQLPrepare');
    pointer(SQLCloseCursor) := GetProcedureAddress(ODBCLibraryHandle,'SQLCloseCursor');
    pointer(SQLExecute) := GetProcedureAddress(ODBCLibraryHandle,'SQLExecute');
    pointer(SQLFetch) := GetProcedureAddress(ODBCLibraryHandle,'SQLFetch');
    pointer(SQLNumResultCols) := GetProcedureAddress(ODBCLibraryHandle,'SQLNumResultCols');
    pointer(SQLDescribeCol) := GetProcedureAddress(ODBCLibraryHandle,'SQLDescribeCol');
    pointer(SQLFetchScroll) := GetProcedureAddress(ODBCLibraryHandle,'SQLFetchScroll');
    pointer(SQLExtendedFetch) := GetProcedureAddress(ODBCLibraryHandle,'SQLExtendedFetch');
    pointer(SQLGetData) := GetProcedureAddress(ODBCLibraryHandle,'SQLGetData');
    pointer(SQLSetStmtAttr) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetStmtAttr');
    pointer(SQLGetStmtAttr) := GetProcedureAddress(ODBCLibraryHandle,'SQLGetStmtAttr');
<<<<<<< HEAD
<<<<<<< HEAD
    //pointer(SQLSetDescField) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetDescField');
    pointer(SQLSetDescRec) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetDescRec');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    pointer(SQLBulkOperations) := GetProcedureAddress(ODBCLibraryHandle,'SQLBulkOperations');
    pointer(SQLPutData) := GetProcedureAddress(ODBCLibraryHandle,'SQLPutData');
    pointer(SQLBindCol) := GetProcedureAddress(ODBCLibraryHandle,'SQLBindCol');
    pointer(SQLSetPos) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetPos');
    pointer(SQLDataSources) := GetProcedureAddress(ODBCLibraryHandle,'SQLDataSources');
    pointer(SQLDrivers) := GetProcedureAddress(ODBCLibraryHandle,'SQLDrivers');
    pointer(SQLSetConnectAttr) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetConnectAttr');
    pointer(SQLGetCursorName) := GetProcedureAddress(ODBCLibraryHandle,'SQLGetCursorName');
    pointer(SQLSetCursorName) := GetProcedureAddress(ODBCLibraryHandle,'SQLSetCursorName');
    pointer(SQLRowCount) := GetProcedureAddress(ODBCLibraryHandle,'SQLRowCount');
    pointer(SQLBindParameter) := GetProcedureAddress(ODBCLibraryHandle,'SQLBindParameter');
    pointer(SQLFreeStmt) := GetProcedureAddress(ODBCLibraryHandle,'SQLFreeStmt');
    pointer(SQLColAttribute) := GetProcedureAddress(ODBCLibraryHandle,'SQLColAttribute');
    pointer(SQLEndTran) := GetProcedureAddress(ODBCLibraryHandle,'SQLEndTran');
    pointer(SQLTables) := GetProcedureAddress(ODBCLibraryHandle,'SQLTables');
    pointer(SQLPrimaryKeys) := GetProcedureAddress(ODBCLibraryHandle,'SQLPrimaryKeys');
    pointer(SQLProcedureColumns) := GetProcedureAddress(ODBCLibraryHandle,'SQLProcedureColumns');
    pointer(SQLStatistics) := GetProcedureAddress(ODBCLibraryHandle,'SQLStatistics');
{$else}
    SQLAllocHandle := GetProcedureAddress(ODBCLibraryHandle,'SQLAllocHandle');
    SQLSetEnvAttr := GetProcedureAddress(ODBCLibraryHandle,'SQLSetEnvAttr');
    SQLFreeHandle := GetProcedureAddress(ODBCLibraryHandle,'SQLFreeHandle');
    SQLGetInfo := GetProcedureAddress(ODBCLibraryHandle,'SQLGetInfo');
    SQLProcedures := GetProcedureAddress(ODBCLibraryHandle,'SQLProcedures');
    SQLColumns := GetProcedureAddress(ODBCLibraryHandle,'SQLColumns');
    SQLSpecialColumns := GetProcedureAddress(ODBCLibraryHandle,'SQLSpecialColumns');
    SQLGetDiagRec := GetProcedureAddress(ODBCLibraryHandle,'SQLGetDiagRec');
    SQLGetDiagField := GetProcedureAddress(ODBCLibraryHandle,'SQLGetDiagField');
    SQLConnect := GetProcedureAddress(ODBCLibraryHandle,'SQLConnect');
    SQLDisconnect := GetProcedureAddress(ODBCLibraryHandle,'SQLDisconnect');
    SQLDriverConnect := GetProcedureAddress(ODBCLibraryHandle,'SQLDriverConnect');
    SQLExecDirect := GetProcedureAddress(ODBCLibraryHandle,'SQLExecDirect');
    SQLPrepare := GetProcedureAddress(ODBCLibraryHandle,'SQLPrepare');
    SQLCloseCursor := GetProcedureAddress(ODBCLibraryHandle,'SQLCloseCursor');
    SQLExecute := GetProcedureAddress(ODBCLibraryHandle,'SQLExecute');
    SQLFetch := GetProcedureAddress(ODBCLibraryHandle,'SQLFetch');
    SQLNumResultCols := GetProcedureAddress(ODBCLibraryHandle,'SQLNumResultCols');
    SQLDescribeCol := GetProcedureAddress(ODBCLibraryHandle,'SQLDescribeCol');
    SQLFetchScroll := GetProcedureAddress(ODBCLibraryHandle,'SQLFetchScroll');
    SQLExtendedFetch := GetProcedureAddress(ODBCLibraryHandle,'SQLExtendedFetch');
    SQLGetData := GetProcedureAddress(ODBCLibraryHandle,'SQLGetData');
    SQLSetStmtAttr := GetProcedureAddress(ODBCLibraryHandle,'SQLSetStmtAttr');
    SQLGetStmtAttr := GetProcedureAddress(ODBCLibraryHandle,'SQLGetStmtAttr');
<<<<<<< HEAD
<<<<<<< HEAD
    //SQLSetDescField := GetProcedureAddress(ODBCLibraryHandle,'SQLSetDescField');
    SQLSetDescRec := GetProcedureAddress(ODBCLibraryHandle,'SQLSetDescRec');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    SQLBulkOperations := GetProcedureAddress(ODBCLibraryHandle,'SQLBulkOperations');
    SQLPutData := GetProcedureAddress(ODBCLibraryHandle,'SQLPutData');
    SQLBindCol := GetProcedureAddress(ODBCLibraryHandle,'SQLBindCol');
    SQLSetPos := GetProcedureAddress(ODBCLibraryHandle,'SQLSetPos');
    SQLDataSources := GetProcedureAddress(ODBCLibraryHandle,'SQLDataSources');
    SQLDrivers := GetProcedureAddress(ODBCLibraryHandle,'SQLDrivers');
    SQLSetConnectAttr := GetProcedureAddress(ODBCLibraryHandle,'SQLSetConnectAttr');
    SQLGetCursorName := GetProcedureAddress(ODBCLibraryHandle,'SQLGetCursorName');
    SQLSetCursorName := GetProcedureAddress(ODBCLibraryHandle,'SQLSetCursorName');
    SQLRowCount := GetProcedureAddress(ODBCLibraryHandle,'SQLRowCount');
    SQLBindParameter := GetProcedureAddress(ODBCLibraryHandle,'SQLBindParameter');
    SQLGetFunctions := GetProcedureAddress(ODBCLibraryHandle,'SQLGetFunctions');
    SQLDescribeParam :=GetProcedureAddress(ODBCLibraryHandle,'SQLDescribeParam');
    SQLFreeStmt := GetProcedureAddress(ODBCLibraryHandle,'SQLFreeStmt');
    SQLColAttribute := GetProcedureAddress(ODBCLibraryHandle,'SQLColAttribute');
    SQLEndTran := GetProcedureAddress(ODBCLibraryHandle,'SQLEndTran');
    SQLTables := GetProcedureAddress(ODBCLibraryHandle,'SQLTables');
    SQLPrimaryKeys := GetProcedureAddress(ODBCLibraryHandle,'SQLPrimaryKeys');
    SQLProcedureColumns := GetProcedureAddress(ODBCLibraryHandle,'SQLProcedureColumns');
    SQLStatistics := GetProcedureAddress(ODBCLibraryHandle,'SQLStatistics');
{$endif}
    end;
end;

Procedure ReleaseODBC;

begin
  if RefCount > 0 then dec(RefCount);
  if RefCount = 0 then
    begin
    if not UnloadLibrary(ODBCLibraryHandle) then inc(RefCount);
    end;
end;

{$ENDIF}

function DateStructToDateTime( b:PSQL_DATE_STRUCT):TDateTime;
begin
  Result:=EncodeDate( b^.Year, b^.Month, b^.Day);
end;

function DateTimeToDateStruct( b:TDateTime):SQL_DATE_STRUCT;
var
  y,m,d: Word;
begin
  DecodeDate( b, y, m, d);
  with Result do
  begin
    Year:=y; Month:=m; Day:=d;
  end;
end;

procedure DateTime2TimeStampStruct( var Value:SQL_TIMESTAMP_STRUCT; b:TDateTime);
var
  w1,w2,w3,w4: Word;
begin
   with Value do
   begin
      DecodeDate(b,w1,w2,w3);
      Year   := w1;
      Month  := w2;
      Day    := w3;
      DecodeTime(b,w1,w2,w3,w4);
      Hour   := w1;
      Minute := w2;
      Second := w3;
      fraction := Integer(w4)*1000000;
   end;
end;
{
  SQL_DATE_STRUCT = packed record
    Year : SQLSMALLINT;
    Month : SQLUSMALLINT;
    Day : SQLUSMALLINT;
  end;
  PSQL_DATE_STRUCT = ^SQL_DATE_STRUCT;
}

Function TimeStampStructToDateTime( B :  PSQL_TIMESTAMP_STRUCT) : TDateTime;

begin
 With B^ do
<<<<<<< HEAD
<<<<<<< HEAD
   Result:=ComposeDateTime(EncodeDate(Year,Month,Day), EncodeTime(Hour,Minute,Second,Fraction div 1000000));
=======
   Result:=EncodeDate(Year,Month,Day)+
           EncodeTime(Hour,Minute,Second,0);
>>>>>>> graemeg/fixes_2_2
=======
   Result:=EncodeDate(Year,Month,Day)+
           EncodeTime(Hour,Minute,Second,0);
>>>>>>> origin/fixes_2_2
end;

Function TimeStructToDateTime (B : PSQL_TIME_STRUCT) : TDateTime;
begin
  With B^ do
<<<<<<< HEAD
<<<<<<< HEAD
    begin
      // TryEncodeTime can not be used, because it doesn't supports times with more then 24 hours.
      Result:=TDateTime(cardinal(Hour)*3600000+cardinal(Minute)*60000+cardinal(Second)*1000)/MSecsPerDay;
    end;
end;

function DateTimeToTimeStruct(b: TDateTime) : SQL_TIME_STRUCT;
var
  w1, w2, w3, w4: Word;
begin
  DecodeTime(b, w1, w2, w3, w4);
  Result.Hour:=w1 + Trunc(b)*24;
  Result.Minute:=w2;
  Result.Second:=w3;
end;

function CurrToNumericStruct(c: currency): SQL_NUMERIC_STRUCT;
var n: int64; i: integer;
begin
  Result.precision:=18;
  Result.scale:=4;
  if c >= 0 then
    Result.sign:=1
  else begin
    Result.sign:=0;
    c := -c;
  end;
  n := NtoLE(int64(c));
  for i:=0 to high(Result.val) do begin
    Result.val[i] := n and $ff;
    n := n shr 8;
  end;
end;
=======
=======
>>>>>>> origin/fixes_2_2
    Result:=EncodeTime(Hour,Minute,Second,0);
end;



<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
