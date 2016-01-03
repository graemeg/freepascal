{
    This file is part of the Free Pascal Classes Library (FCL).
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 2006-2014 by the Free Pascal development team
=======
    Copyright (c) 2006 by the Free Pascal development team
>>>>>>> graemeg/fixes_2_2
=======
    Copyright (c) 2006 by the Free Pascal development team
>>>>>>> origin/fixes_2_2

    SQLite3 connection for SQLDB

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
 
{ 
  Based on an implementation by Martin Schreiber, part of MSEIDE.
  Reworked all code so it conforms to FCL coding standards.
<<<<<<< HEAD
<<<<<<< HEAD

  TSQLite3Connection properties
      Params - "foreign_keys=ON" - enable foreign key support for this connection:
                                   http://www.sqlite.org/foreignkeys.html#fk_enable

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
} 
 
unit sqlite3conn;
{$mode objfpc}
{$h+}

interface

uses
  classes, db, bufdataset, sqldb, sqlite3dyn, types;

const
  sqliteerrormax = 99;
 
type
  PDateTime = ^TDateTime;
  
<<<<<<< HEAD
<<<<<<< HEAD
=======
  TSqliteOption = (sloTransactions,sloDesignTransactions);
  TSqliteOptions = set of TSqliteOption;
 
>>>>>>> graemeg/fixes_2_2
=======
  TSqliteOption = (sloTransactions,sloDesignTransactions);
  TSqliteOptions = set of TSqliteOption;
 
>>>>>>> origin/fixes_2_2
  TStringArray = Array of string;
  PStringArray = ^TStringArray;
 
  TArrayStringArray = Array of TStringArray;
  PArrayStringArray = ^TArrayStringArray;
 
  { TSQLite3Connection }

  TSQLite3Connection = class(TSQLConnection)
  private
    fhandle: psqlite3;
<<<<<<< HEAD
<<<<<<< HEAD
  protected
=======
=======
>>>>>>> origin/fixes_2_2
    foptions: TSQLiteOptions;
    procedure setoptions(const avalue: tsqliteoptions);
  protected
    function stringsquery(const asql: string): TArrayStringArray;
    procedure checkerror(const aerror: integer);
    
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure DoInternalConnect; override;
    procedure DoInternalDisconnect; override;
    function GetHandle : pointer; override;
 
    Function AllocateCursorHandle : TSQLCursor; override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); override;
    Function AllocateTransactionHandle : TSQLHandle; override;
 
    function StrToStatementType(s : string) : TStatementType; override;
    procedure PrepareStatement(cursor: TSQLCursor; ATransaction : TSQLTransaction; buf: string; AParams : TParams); override;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); override;
    function Fetch(cursor : TSQLCursor) : boolean; override;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TFieldDefs); override;
    procedure UnPrepareStatement(cursor : TSQLCursor); override;
 
    procedure FreeFldBuffers(cursor : TSQLCursor); override;
    function LoadField(cursor : TSQLCursor; FieldDef : TFieldDef; buffer : pointer; out CreateBlob : boolean) : boolean; override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;

    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function StartDBTransaction(trans : TSQLHandle; aParams : string) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    procedure RollBackRetaining(trans : TSQLHandle); override;

    procedure UpdateIndexDefs(IndexDefs : TIndexDefs; TableName : string); override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function RefreshLastInsertID(Query : TCustomSQLQuery; Field : TField): Boolean; override;
    // New methods
    procedure checkerror(const aerror: integer);
    function stringsquery(const asql: string): TArrayStringArray;
    procedure execsql(const asql: string);
<<<<<<< HEAD
=======
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override; // Differs from SQLDB.
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    function StrToStatementType(s : string) : TStatementType; override;
>>>>>>> graemeg/cpstrnew
  public
    constructor Create(AOwner : TComponent); override;
    procedure GetFieldNames(const TableName : string; List :  TStrings); override;
    function GetConnectionInfo(InfoType:TConnInfoType): string; override;
    procedure CreateDB; override;
    procedure DropDB; override;
    function GetInsertID: int64;
    // See http://www.sqlite.org/c3ref/create_collation.html for detailed information
    // If eTextRep=0 a default UTF-8 compare function is used (UTF8CompareCallback)
    // Warning: UTF8CompareCallback needs a wide string manager on Linux such as cwstring
    // Warning: CollationName has to be a UTF-8 string
    procedure CreateCollation(const CollationName: string; eTextRep: integer; Arg: Pointer=nil; Compare: xCompare=nil);
    procedure LoadExtension(LibraryFile: string);
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  { TSQLite3ConnectionDef }

  TSQLite3ConnectionDef = class(TConnectionDef)
    class function TypeName: string; override;
    class function ConnectionClass: TSQLConnectionClass; override;
    class function Description: string; override;
    class Function DefaultLibraryName : String; override;
    class Function LoadFunction : TLibraryLoadFunction; override;
    class Function UnLoadFunction : TLibraryUnLoadFunction; override;
    class function LoadedLibraryName: string; override;
  end;
  
Var
  SQLiteLibraryName : String absolute sqlite3dyn.SQLiteDefaultLibrary deprecated 'use sqlite3dyn.SQLiteDefaultLibrary instead';
=======
  
Var
  SQLiteLibraryName : String = sqlite3lib; 
>>>>>>> graemeg/cpstrnew
=======
  
Var
  SQLiteLibraryName : String = sqlite3lib; 
>>>>>>> graemeg/cpstrnew
=======
  
Var
  SQLiteLibraryName : String = sqlite3lib; 
>>>>>>> graemeg/cpstrnew
=======
  
Var
  SQLiteLibraryName : String = sqlite3lib; 
>>>>>>> origin/cpstrnew
   
implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  dbconst, sysutils, dateutils, FmtBCD;

{$IF NOT DECLARED(JulianEpoch)} // sysutils/datih.inc
const
  JulianEpoch = TDateTime(-2415018.5); // "julian day 0" is January 1, 4713 BC 12:00AM
{$ENDIF}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  dbconst, sysutils, dateutils,FmtBCD;
 
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
                        //aowner used as blob cache
    Procedure DeAllocateCursorHandle(var cursor : TSQLCursor); override;
    Function AllocateTransactionHandle : TSQLHandle; override;
 
    procedure PrepareStatement(cursor: TSQLCursor; ATransaction : TSQLTransaction; 
                          buf: string; AParams : TParams); override;
    procedure Execute(cursor: TSQLCursor;atransaction:tSQLtransaction; AParams : TParams); override;
    function Fetch(cursor : TSQLCursor) : boolean; override;
    procedure AddFieldDefs(cursor: TSQLCursor; FieldDefs : TfieldDefs); override;
    procedure UnPrepareStatement(cursor : TSQLCursor); override;
 
    procedure FreeFldBuffers(cursor : TSQLCursor); override;
    function LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; override;
           //if bufsize < 0 -> buffer was to small, should be -bufsize
    function GetTransactionHandle(trans : TSQLHandle): pointer; override;
    function Commit(trans : TSQLHandle) : boolean; override;
    function RollBack(trans : TSQLHandle) : boolean; override;
    function StartdbTransaction(trans : TSQLHandle; aParams : string) : boolean; override;
    procedure CommitRetaining(trans : TSQLHandle); override;
    procedure RollBackRetaining(trans : TSQLHandle); override;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); override;
    // New methods
    procedure execsql(const asql: string);
    procedure UpdateIndexDefs(IndexDefs : TIndexDefs;TableName : string); override; // Differs from SQLDB.
    function getprimarykeyfield(const atablename: string; const acursor: tsqlcursor): string;
    function RowsAffected(cursor: TSQLCursor): TRowsCount; override;
    function GetSchemaInfoSQL(SchemaType : TSchemaType; SchemaObjectName, SchemaPattern : string) : string; override;
    function StrToStatementType(s : string) : TStatementType; override;
  public
    constructor Create(AOwner : TComponent); override;
    function GetInsertID: int64;
    procedure GetFieldNames(const TableName : string; List :  TStrings); override;
  published
    property Options: TSqliteOptions read FOptions write SetOptions;
  end;
 
implementation

uses
  dbconst, sysutils, typinfo, dateutils;
 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type

 TStorageType = (stNone,stInteger,stFloat,stText,stBlob,stNull);
 
 TSQLite3Cursor = class(tsqlcursor)
  private
   fhandle : psqlite3;
<<<<<<< HEAD
<<<<<<< HEAD
   fconnection: TSQLite3Connection;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   fstatement: psqlite3_stmt;
   ftail: pchar;
   fstate: integer;
   fparambinding: array of Integer;
   procedure checkerror(const aerror: integer);
   procedure bindparams(AParams : TParams);
<<<<<<< HEAD
<<<<<<< HEAD
   Procedure Prepare(Buf : String; AParams : TParams);
=======
   Procedure Prepare(Buf : String; APArams : TParams);
>>>>>>> graemeg/fixes_2_2
=======
   Procedure Prepare(Buf : String; APArams : TParams);
>>>>>>> origin/fixes_2_2
   Procedure UnPrepare;
   Procedure Execute;
   Function Fetch : Boolean;
 public
   RowsAffected : Largeint;
 end;

procedure freebindstring(astring: pointer); cdecl;
begin
  StrDispose(AString);
end;

procedure TSQLite3Cursor.checkerror(const aerror: integer);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  fconnection.checkerror(aerror);
=======
=======
>>>>>>> origin/fixes_2_2

Var
  S : String;

begin
 if (aerror<>sqlite_ok) then 
   begin
   S:=strpas(sqlite3_errmsg(fhandle));
   DatabaseError(S);
   end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Procedure TSQLite3Cursor.bindparams(AParams : TParams);

  Function PCharStr(Const S : String) : PChar;
  
  begin
    Result:=StrAlloc(Length(S)+1);
    If (Result<>Nil) then
      StrPCopy(Result,S);
  end;
  
Var
  I : Integer;
  P : TParam;  
<<<<<<< HEAD
<<<<<<< HEAD
  str1: string;
  wstr1: widestring;
  
begin
  for I:=1 to high(fparambinding)+1 do
    begin
    P:=AParams[fparambinding[I-1]];
    if P.IsNull then
      checkerror(sqlite3_bind_null(fstatement,I))
    else 
      case P.DataType of
        ftInteger,
        ftAutoInc,
        ftSmallint: checkerror(sqlite3_bind_int(fstatement,I,P.AsInteger));
        ftWord:     checkerror(sqlite3_bind_int(fstatement,I,P.AsWord));
        ftBoolean:  checkerror(sqlite3_bind_int(fstatement,I,ord(P.AsBoolean)));
        ftLargeint: checkerror(sqlite3_bind_int64(fstatement,I,P.AsLargeint));
        ftBcd,
        ftFloat,
        ftCurrency: checkerror(sqlite3_bind_double(fstatement, I, P.AsFloat));
        ftDateTime,
        ftDate,
        ftTime:     checkerror(sqlite3_bind_double(fstatement, I, P.AsFloat - JulianEpoch));
        ftFMTBcd:
                begin
                str1:=BCDToStr(P.AsFMTBCD, Fconnection.FSQLFormatSettings);
                checkerror(sqlite3_bind_text(fstatement, I, PChar(str1), length(str1), sqlite3_destructor_type(SQLITE_TRANSIENT)));
                end;
<<<<<<< HEAD
<<<<<<< HEAD
        ftString,
        ftFixedChar,
        ftMemo: begin // According to SQLite documentation, CLOB's (ftMemo) have the Text affinity
                str1:= p.asstring;
                checkerror(sqlite3_bind_text(fstatement,I,pcharstr(str1), length(str1),@freebindstring));
                end;
<<<<<<< HEAD
        ftBytes,
        ftVarBytes,
        ftBlob: begin
=======
        ftstring,
        ftmemo: begin // According to SQLite documentation, CLOB's (ftMemo) have the Text affinity
                str1:= p.asstring;
                checkerror(sqlite3_bind_text(fstatement,I,pcharstr(str1), length(str1),@freebindstring));
                end;
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ftstring,
        ftmemo: begin // According to SQLite documentation, CLOB's (ftMemo) have the Text affinity
                str1:= p.asstring;
                checkerror(sqlite3_bind_text(fstatement,I,pcharstr(str1), length(str1),@freebindstring));
                end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ftblob: begin
>>>>>>> graemeg/cpstrnew
                str1:= P.asstring;
                checkerror(sqlite3_bind_blob(fstatement,I,pcharstr(str1), length(str1),@freebindstring));
                end; 
        ftWideString, ftFixedWideChar, ftWideMemo:
        begin
          wstr1:=P.AsWideString;
          checkerror(sqlite3_bind_text16(fstatement,I, PWideChar(wstr1), length(wstr1)*sizeof(WideChar), sqlite3_destructor_type(SQLITE_TRANSIENT)));
        end
      else 
        DatabaseErrorFmt(SUnsupportedParameter, [Fieldtypenames[P.DataType], Self]);
=======
=======
>>>>>>> origin/fixes_2_2
  pc : pchar;
  str1: string;
  cu1: currency;
  do1: double;
  parms : array of Integer;
  
begin
  for I:=1  to high(fparambinding)+1 do 
    begin
    P:=aparams[fparambinding[I-1]];
    if P.isnull then 
      checkerror(sqlite3_bind_null(fstatement,I))
    else 
      case P.datatype of
        ftinteger,
        ftboolean,
        ftsmallint: checkerror(sqlite3_bind_int(fstatement,I,p.asinteger));
        ftword:     checkerror(sqlite3_bind_int(fstatement,I,P.asword));
        ftlargeint: checkerror(sqlite3_bind_int64(fstatement,I,P.aslargeint));
        ftbcd,
        ftfloat,
        ftcurrency,
        ftdatetime,
        ftdate,
        fttime: begin
                do1:= P.asfloat;
                checkerror(sqlite3_bind_double(fstatement,I,do1));
                end;
        ftstring: begin
                  str1:= p.asstring;
                  checkerror(sqlite3_bind_text(fstatement,I,pcharstr(str1), length(str1),@freebindstring));
                  end;
        ftblob: begin
                str1:= P.asstring;
                checkerror(sqlite3_bind_blob(fstatement,I,pcharstr(str1), length(str1),@freebindstring));
                end; 
      else 
        databaseerror('Parameter type '+getenumname(typeinfo(tfieldtype),ord(P.datatype))+' not supported.');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end; { Case }
    end;   
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TSQLite3Cursor.Prepare(Buf : String; AParams : TParams);

begin
  if assigned(AParams) and (AParams.Count > 0) then
    Buf := AParams.ParseSQL(Buf,false,false,false,psInterbase,fparambinding);
  if (detActualSQL in fconnection.LogEvents) then
    fconnection.Log(detActualSQL,Buf);
  checkerror(sqlite3_prepare(fhandle,pchar(Buf),length(Buf),@fstatement,@ftail));
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TSQLite3Cursor.Prepare(Buf : String; APArams : TParams);

begin
  if assigned(aparams) and (aparams.count > 0) then 
    buf := aparams.parsesql(buf,false,false,false,psinterbase,fparambinding);
  checkerror(sqlite3_prepare(fhandle,pchar(buf),length(buf),@fstatement,@ftail));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FPrepared:=True;
end;

Procedure TSQLite3Cursor.UnPrepare;

begin
  sqlite3_finalize(fstatement); // No check.
  FPrepared:=False;
end;

Procedure TSQLite3Cursor.Execute;

var
 wo1: word;

begin
{$ifdef i386}
  wo1:= get8087cw;
  set8087cw(wo1 or $1f);             //mask exceptions, Sqlite3 has overflow
  Try  // Why do people always forget this ??
{$endif}
    fstate:= sqlite3_step(fstatement);
{$ifdef i386}
  finally  
    set8087cw(wo1);                    //restore
  end;
{$endif}
  if (fstate<=sqliteerrormax) then
    checkerror(sqlite3_reset(fstatement));
<<<<<<< HEAD
<<<<<<< HEAD
  FSelectable :=sqlite3_column_count(fstatement)>0;
  RowsAffected:=sqlite3_changes(fhandle);
  if (fstate=sqlite_row) then
=======
  RowsAffected:=sqlite3_changes(fhandle);
  if (fstate=sqlite_row) then 
>>>>>>> graemeg/fixes_2_2
=======
  RowsAffected:=sqlite3_changes(fhandle);
  if (fstate=sqlite_row) then 
>>>>>>> origin/fixes_2_2
    fstate:= sqliteerrormax; //first row
end;  

Function TSQLite3Cursor.Fetch : Boolean;

begin
  if (fstate=sqliteerrormax) then 
    fstate:=sqlite_row //first row;
  else if (fstate=sqlite_row) then 
    begin
    fstate:=sqlite3_step(fstatement);
    if (fstate<=sqliteerrormax) then 
      checkerror(sqlite3_reset(fstatement));  //right error returned??
    end;
  result:=(fstate=sqlite_row);
end;

{ TSQLite3Connection }

<<<<<<< HEAD
<<<<<<< HEAD
constructor TSQLite3Connection.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FConnOptions := [sqEscapeRepeat, sqEscapeSlash, sqImplicitTransaction, sqLastInsertID];
  FieldNameQuoteChars:=DoubleQuotes;
end;

procedure TSQLite3Connection.LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction);
=======
procedure TSQLite3Connection.LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); 
>>>>>>> graemeg/fixes_2_2
=======
procedure TSQLite3Connection.LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField; cursor: TSQLCursor; ATransaction : TSQLTransaction); 
>>>>>>> origin/fixes_2_2

var
 int1: integer;
 st: psqlite3_stmt;
 fnum: integer;
<<<<<<< HEAD
<<<<<<< HEAD
 p1: Pointer;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  st:=TSQLite3Cursor(cursor).fstatement;
  fnum:= FieldDef.fieldno - 1;

<<<<<<< HEAD
<<<<<<< HEAD
  case FieldDef.DataType of
    ftWideMemo:
      begin
      p1 := sqlite3_column_text16(st,fnum);
      int1 := sqlite3_column_bytes16(st,fnum);
      end;
    ftMemo:
      begin
      p1 := sqlite3_column_text(st,fnum);
      int1 := sqlite3_column_bytes(st,fnum);
      end;
    else //ftBlob
      begin
      p1 := sqlite3_column_blob(st,fnum);
      int1 := sqlite3_column_bytes(st,fnum);
      end;
  end;

  ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer, int1);
  if int1 > 0 then
    move(p1^, ABlobBuf^.BlobBuffer^.Buffer^, int1);
=======
=======
>>>>>>> origin/fixes_2_2
  int1:= sqlite3_column_bytes(st,fnum);

  ReAllocMem(ABlobBuf^.BlobBuffer^.Buffer,int1);
  if int1 > 0 then
    move(sqlite3_column_text(st,fnum)^,ABlobBuf^.BlobBuffer^.Buffer^,int1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ABlobBuf^.BlobBuffer^.Size := int1;
end;

function TSQLite3Connection.AllocateTransactionHandle: TSQLHandle;
begin
 result:= tsqlhandle.create;
end;

function TSQLite3Connection.AllocateCursorHandle: TSQLCursor;

Var
  Res : TSQLite3Cursor;

begin
  Res:= TSQLite3Cursor.create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Res.fconnection:=Self;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  Res.fhandle:=self.fhandle;
>>>>>>> graemeg/fixes_2_2
=======
  Res.fhandle:=self.fhandle;
>>>>>>> origin/fixes_2_2
  Result:=Res;
end;

procedure TSQLite3Connection.DeAllocateCursorHandle(var cursor: TSQLCursor);
begin
  freeandnil(cursor);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TSQLite3Connection.StrToStatementType(s: string): TStatementType;
begin
  S:=Lowercase(s);
  if s = 'pragma' then exit(stSelect);
  result := inherited StrToStatementType(s);
end;

procedure TSQLite3Connection.PrepareStatement(cursor: TSQLCursor;
               ATransaction: TSQLTransaction; buf: string; AParams: TParams);
begin
  TSQLite3Cursor(cursor).fhandle:=self.fhandle;
=======
procedure TSQLite3Connection.PrepareStatement(cursor: TSQLCursor;
               ATransaction: TSQLTransaction; buf: string; AParams: TParams);
begin
>>>>>>> graemeg/fixes_2_2
=======
procedure TSQLite3Connection.PrepareStatement(cursor: TSQLCursor;
               ATransaction: TSQLTransaction; buf: string; AParams: TParams);
begin
>>>>>>> origin/fixes_2_2
  TSQLite3Cursor(cursor).Prepare(Buf,AParams);
end;

procedure TSQLite3Connection.UnPrepareStatement(cursor: TSQLCursor);

begin
  TSQLite3Cursor(cursor).UnPrepare;
<<<<<<< HEAD
<<<<<<< HEAD
  TSQLite3Cursor(cursor).fhandle:=nil;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


Type
  TFieldMap = Record
    N : String;
    T : TFieldType;
  end;
  
Const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FieldMapCount = 28;
=======
  FieldMapCount = 20;
>>>>>>> graemeg/cpstrnew
=======
  FieldMapCount = 20;
>>>>>>> graemeg/cpstrnew
=======
  FieldMapCount = 20;
>>>>>>> graemeg/cpstrnew
=======
  FieldMapCount = 20;
>>>>>>> origin/cpstrnew
  FieldMap : Array [1..FieldMapCount] of TFieldMap = (
   (n:'INT'; t: ftInteger),
   (n:'LARGEINT'; t:ftLargeInt),
   (n:'BIGINT'; t:ftLargeInt),
   (n:'SMALLINT'; t: ftSmallint),
   (n:'TINYINT'; t: ftSmallint),
   (n:'WORD'; t: ftWord),
=======
=======
>>>>>>> origin/fixes_2_2
  FieldMapCount = 19;
  FieldMap : Array [1..FieldMapCount] of TFieldMap = (
   (n:'INT'; t: ftInteger),
   (n:'LARGEINT'; t:ftlargeInt),
   (n:'WORD'; t: ftWord),
   (n:'SMALLINT'; t: ftSmallint),
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   (n:'BOOLEAN'; t: ftBoolean),
   (n:'REAL'; t: ftFloat),
   (n:'FLOAT'; t: ftFloat),
   (n:'DOUBLE'; t: ftFloat),
   (n:'TIMESTAMP'; t: ftDateTime),
   (n:'DATETIME'; t: ftDateTime), // MUST be before date
   (n:'DATE'; t: ftDate),
   (n:'TIME'; t: ftTime),
   (n:'CURRENCY'; t: ftCurrency),
<<<<<<< HEAD
<<<<<<< HEAD
   (n:'MONEY'; t: ftCurrency),
   (n:'VARCHAR'; t: ftString),
   (n:'CHAR'; t: ftFixedChar),
   (n:'NUMERIC'; t: ftBCD),
   (n:'DECIMAL'; t: ftBCD),
<<<<<<< HEAD
   (n:'TEXT'; t: ftMemo),
   (n:'CLOB'; t: ftMemo),
   (n:'BLOB'; t: ftBlob),
   (n:'NCHAR'; t: ftFixedWideChar),
   (n:'NVARCHAR'; t: ftWideString),
   (n:'NCLOB'; t: ftWideMemo),
   (n:'VARBINARY'; t: ftVarBytes),
   (n:'BINARY'; t: ftBytes)
=======
   (n:'TEXT'; t: ftmemo),
   (n:'CLOB'; t: ftmemo),
   (n:'BLOB'; t: ftBlob)
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
   (n:'VARCHAR'; t: ftString),
   (n:'CHAR'; t: ftString),
   (n:'NUMERIC'; t: ftBCD),
   (n:'DECIMAL'; t: ftBCD),
   (n:'TEXT'; t: ftmemo),
   (n:'BLOB'; t: ftBlob)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ Template:
  (n:''; t: ft)
}
  );

<<<<<<< HEAD
<<<<<<< HEAD
procedure TSQLite3Connection.AddFieldDefs(cursor: TSQLCursor; FieldDefs: TFieldDefs);
var
 i, fi : integer;
 FN, FD, PrimaryKeyFields : string;
 ft1   : TFieldType;
 size1, size2 : integer;
 st    : psqlite3_stmt;

 function GetPrimaryKeyFields: string;
 var IndexDefs: TServerIndexDefs;
     i: integer;
 begin
   if FieldDefs.Dataset is TSQLQuery then
   begin
     IndexDefs := (FieldDefs.DataSet as TSQLQuery).ServerIndexDefs;
     for i:=IndexDefs.Count-1 downto 0 do
       if ixPrimary in IndexDefs[i].Options then
       begin
         Result := IndexDefs[i].Fields;
         Exit;
       end;
   end;
   Result := '';
 end;

 function ExtractPrecisionAndScale(decltype: string; var precision, scale: integer): boolean;
 var p: integer;
 begin
   p:=pos('(', decltype);
   Result:=p>0;
   if not Result then Exit;
   System.Delete(decltype,1,p);
   p:=pos(')', decltype);
   Result:=p>0;
   if not Result then Exit;
   decltype:=copy(decltype,1,p-1);
   p:=pos(',', decltype);
   if p=0 then
   begin
     precision:=StrToIntDef(decltype, precision);
     scale:=0;
   end
   else
   begin
     precision:=StrToIntDef(copy(decltype,1,p-1), precision);
     scale:=StrToIntDef(copy(decltype,p+1,length(decltype)-p), scale);
   end;
 end;

begin
  PrimaryKeyFields := GetPrimaryKeyFields;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TSQLite3Connection.AddFieldDefs(cursor: TSQLCursor;
               FieldDefs: TfieldDefs);
var
 i     : integer;
 FN,FD : string;
 ft1   : tfieldtype;
 size1 : word;
 ar1   : TStringArray;
 fi    : integer;
 st    : psqlite3_stmt;
 
begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  st:=TSQLite3Cursor(cursor).fstatement;
  for i:= 0 to sqlite3_column_count(st) - 1 do 
    begin
    FN:=sqlite3_column_name(st,i);
    FD:=uppercase(sqlite3_column_decltype(st,i));
    ft1:= ftUnknown;
    size1:= 0;
    for fi := 1 to FieldMapCount do if pos(FieldMap[fi].N,FD)=1 then
      begin
      ft1:=FieldMap[fi].t;
      break;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    // Column declared as INTEGER PRIMARY KEY [AUTOINCREMENT] becomes ROWID for given table
    // declared data type must be INTEGER (not INT, BIGINT, NUMERIC etc.)
    if (FD='INTEGER') and SameText(FN, PrimaryKeyFields) then
      ft1:=ftAutoInc;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    // In case of an empty fieldtype (FD='', which is allowed and used in calculated
    // columns (aggregates) and by pragma-statements) or an unknown fieldtype,
    // use the field's affinity:
    if ft1=ftUnknown then
      case TStorageType(sqlite3_column_type(st,i)) of
        stInteger: ft1:=ftLargeInt;
        stFloat:   ft1:=ftFloat;
        stBlob:    ft1:=ftBlob;
        else       ft1:=ftString;
      end;
    // handle some specials.
    size1:=0;
    case ft1 of
<<<<<<< HEAD
      ftString,
      ftFixedChar,
      ftFixedWideChar,
      ftWideString,
      ftBytes,
      ftVarBytes:
               begin
                 size1 := 255; //sql: if length is omitted then length is 1
                 size2 := 0;
                 ExtractPrecisionAndScale(FD, size1, size2);
                 if size1 > MaxSmallint then size1 := MaxSmallint;
               end;
      ftBCD:   begin
                 size2 := MaxBCDPrecision; //sql: if a precision is omitted, then use implementation-defined
                 size1 := 0;               //sql: if a scale is omitted then scale is 0
                 ExtractPrecisionAndScale(FD, size2, size1);
                 if (size2<=18) and (size1=0) then
                   ft1:=ftLargeInt
                 else if (size2-size1>MaxBCDPrecision-MaxBCDScale) or (size1>MaxBCDScale) then
                   ft1:=ftFmtBCD;
               end;
      ftUnknown : DatabaseErrorFmt('Unknown or unsupported data type %s of column %s', [FD, FN]);
=======
=======
=======
>>>>>>> origin/fixes_2_2
    // Empty field types are allowed and used in calculated columns (aggregates)
    // and by pragma-statements
    if FD='' then ft1 := ftString;
    // handle some specials.
    size1:=0;
    case ft1 of
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      ftString: begin
                fi:=pos('(',FD);
                if (fi>0) then
                  begin
                  System.Delete(FD,1,fi);
                  fi:=pos(')',FD);
                  size1:=StrToIntDef(trim(copy(FD,1,fi-1)),255);
                  if size1 > dsMaxStringSize then size1 := dsMaxStringSize;
                  end
                else size1 := 255;
                end;
      ftBCD:    begin
                fi:=pos(',',FD);
                if (fi>0) then
                  begin
                  System.Delete(FD,1,fi);
                  fi:=pos(')',FD);
                  size1:=StrToIntDef(trim(copy(FD,1,fi-1)),255);
<<<<<<< HEAD
<<<<<<< HEAD
                  if size1>4 then
                    ft1 := ftFMTBcd;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  end
                else size1 := 4;
                end;
      ftUnknown : DatabaseError('Unknown record type: '+FN);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
    end; // Case
    FieldDefs.Add(FieldDefs.MakeNameUnique(FN),ft1,size1,false,i+1);
    end;
end;

procedure TSQLite3Connection.Execute(cursor: TSQLCursor;
  atransaction: tSQLtransaction; AParams: TParams);
=======
=======
>>>>>>> origin/fixes_2_2
    end; // Case
    tfielddef.create(fielddefs,FieldDefs.MakeNameUnique(FN),ft1,size1,false,i+1);
    end;
end;

procedure TSQLite3Connection.Execute(cursor: TSQLCursor; atransaction: tsqltransaction; AParams: TParams);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
 SC : TSQLite3Cursor;
            
begin
  SC:=TSQLite3Cursor(cursor);
  checkerror(sqlite3_reset(sc.fstatement));
  If (AParams<>Nil) and (AParams.count > 0) then
    SC.BindParams(AParams);
<<<<<<< HEAD
<<<<<<< HEAD
  If LogEvent(detParamValue) then
    LogParams(AParams);
  SC.Execute;
=======
  SC.Execute;    
>>>>>>> graemeg/fixes_2_2
=======
  SC.Execute;    
>>>>>>> origin/fixes_2_2
end;

Function NextWord(Var S : ShortString; Sep : Char) : String;

Var
  P : Integer;

begin
  P:=Pos(Sep,S);
  If (P=0) then 
    P:=Length(S)+1;
  Result:=Copy(S,1,P-1);
  Delete(S,1,P);
end;

<<<<<<< HEAD
<<<<<<< HEAD
// Parses string-formatted date into TDateTime value
// Expected format: '2013-12-31 ' (without ')
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function ParseSQLiteDate(S : ShortString) : TDateTime;

Var
  Year, Month, Day : Integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=0;
  If TryStrToInt(NextWord(S,'-'),Year) then
    if TryStrToInt(NextWord(S,'-'),Month) then
      if TryStrToInt(NextWord(S,' '),Day) then
        Result:=EncodeDate(Year,Month,Day);
end;

// Parses string-formatted time into TDateTime value
// Expected formats
// 23:59
// 23:59:59
// 23:59:59.999
Function ParseSQLiteTime(S : ShortString; Interval: boolean) : TDateTime;

Var
  Hour, Min, Sec, MSec : Integer;
=======
=======
>>>>>>> origin/fixes_2_2
 Result:=0;
 If TryStrToInt(NextWord(S,'-'),Year) then
   if TryStrToInt(NextWord(S,'-'),Month) then
     if TryStrToInt(NextWord(S,'-'),Day) then
        Result:=EncodeDate(Year,Month,Day);
end;

Function ParseSQLiteTime(S : ShortString) : TDateTime;

Var
  Hour, Min, Sec : Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result:=0;
  If TryStrToInt(NextWord(S,':'),Hour) then
    if TryStrToInt(NextWord(S,':'),Min) then
<<<<<<< HEAD
<<<<<<< HEAD
    begin
      if TryStrToInt(NextWord(S,'.'),Sec) then
        // 23:59:59 or 23:59:59.999
        MSec:=StrToIntDef(S,0)
      else // 23:59
      begin
        Sec:=0;
        MSec:=0;
      end;
      if Interval then
        Result:=EncodeTimeInterval(Hour,Min,Sec,MSec)
      else
        Result:=EncodeTime(Hour,Min,Sec,MSec);
    end;
end;

// Parses string-formatted date/time into TDateTime value
=======
=======
>>>>>>> origin/fixes_2_2
      if TryStrToInt(NextWord(S,':'),Sec) then
        Result:=EncodeTime(Hour,Min,Sec,0);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function ParseSQLiteDateTime(S : String) : TDateTime;

var
  P : Integer;
  DS,TS : ShortString;

begin
  DS:='';
  TS:='';
<<<<<<< HEAD
<<<<<<< HEAD
  P:=Pos('T',S); //allow e.g. YYYY-MM-DDTHH:MM
  if P=0 then
    P:=Pos(' ',S); //allow e.g. YYYY-MM-DD HH:MM
=======
  P:=Pos(' ',S);
>>>>>>> graemeg/fixes_2_2
=======
  P:=Pos(' ',S);
>>>>>>> origin/fixes_2_2
  If (P<>0) then
    begin
    DS:=Copy(S,1,P-1);
    TS:=S;
    Delete(TS,1,P);
    end
  else
    begin  
    If (Pos('-',S)<>0) then
      DS:=S
    else if (Pos(':',S)<>0) then
      TS:=S;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=ComposeDateTime(ParseSQLiteDate(DS),ParseSQLiteTime(TS,False));
end;

function TSQLite3Connection.LoadField(cursor : TSQLCursor; FieldDef : TFieldDef; buffer : pointer; out CreateBlob : boolean) : boolean;
=======
  Result:=ComposeDateTime(ParseSQLiteDate(DS),ParseSQLiteTime(TS));
end;
function TSQLite3Connection.LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;
>>>>>>> graemeg/fixes_2_2
=======
  Result:=ComposeDateTime(ParseSQLiteDate(DS),ParseSQLiteTime(TS));
end;
function TSQLite3Connection.LoadField(cursor : TSQLCursor;FieldDef : TfieldDef;buffer : pointer; out CreateBlob : boolean) : boolean;
>>>>>>> origin/fixes_2_2

var
 st1: TStorageType;
 fnum: integer;
<<<<<<< HEAD
<<<<<<< HEAD
 str1: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 int1 : integer;
 bcd: tBCD;
 bcdstr: FmtBCDStringtype;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 bcd: tBCD;
 StoreDecimalPoint: tDecimalPoint;
 bcdstr: FmtBCDStringtype;
 ar1,ar2: TStringArray;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 i: integer;
 i64: int64;
 int1,int2: integer;
 str1: string;
 ar1,ar2: TStringArray;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 st    : psqlite3_stmt;

begin
  st:=TSQLite3Cursor(cursor).fstatement;
  fnum:= FieldDef.fieldno - 1;
  st1:= TStorageType(sqlite3_column_type(st,fnum));
  CreateBlob:=false;
  result:= st1 <> stnull;
  if Not result then 
    Exit;
<<<<<<< HEAD
<<<<<<< HEAD
  case FieldDef.DataType of
    ftAutoInc,
=======
  case FieldDef.datatype of
>>>>>>> graemeg/fixes_2_2
=======
  case FieldDef.datatype of
>>>>>>> origin/fixes_2_2
    ftInteger  : pinteger(buffer)^  := sqlite3_column_int(st,fnum);
    ftSmallInt : psmallint(buffer)^ := sqlite3_column_int(st,fnum);
    ftWord     : pword(buffer)^     := sqlite3_column_int(st,fnum);
    ftBoolean  : pwordbool(buffer)^ := sqlite3_column_int(st,fnum)<>0;
    ftLargeInt : PInt64(buffer)^:= sqlite3_column_int64(st,fnum);
    ftBCD      : PCurrency(buffer)^:= FloattoCurr(sqlite3_column_double(st,fnum));
    ftFloat,
    ftCurrency : pdouble(buffer)^:= sqlite3_column_double(st,fnum);
    ftDateTime,
    ftDate,
    ftTime:  if st1 = sttext then 
<<<<<<< HEAD
<<<<<<< HEAD
               begin { Stored as string }
               setlength(str1,sqlite3_column_bytes(st,fnum));
               move(sqlite3_column_text(st,fnum)^,str1[1],length(str1));
               case FieldDef.datatype of
                 ftDateTime: PDateTime(Buffer)^:=ParseSqliteDateTime(str1);
                 ftDate    : PDateTime(Buffer)^:=ParseSqliteDate(str1);
                 ftTime    : PDateTime(Buffer)^:=ParseSqliteTime(str1,true);
               end; {case}
               end
             else
               begin { Assume stored as double }
               PDateTime(buffer)^ := sqlite3_column_double(st,fnum);
               if PDateTime(buffer)^ > 1721059.5 {Julian 01/01/0000} then
                  PDateTime(buffer)^ := PDateTime(buffer)^ + JulianEpoch; //backward compatibility hack
               end;
    ftFixedChar,
=======
=======
>>>>>>> origin/fixes_2_2
               begin
               setlength(str1,sqlite3_column_bytes(st,fnum));
               move(sqlite3_column_text(st,fnum)^,str1[1],length(str1));
               PDateTime(Buffer)^:=ParseSqliteDateTime(str1)
               end
             else
               Pdatetime(buffer)^:= sqlite3_column_double(st,fnum);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    ftString: begin
              int1:= sqlite3_column_bytes(st,fnum);
              if int1>FieldDef.Size then 
                int1:=FieldDef.Size;
              if int1 > 0 then 
                 move(sqlite3_column_text(st,fnum)^,buffer^,int1);
<<<<<<< HEAD
<<<<<<< HEAD
              PAnsiChar(buffer + int1)^ := #0;
              end;
    ftFmtBCD: begin
              int1:= sqlite3_column_bytes(st,fnum);
              if (int1 > 0) and (int1 <= MAXFMTBcdFractionSize) then
                begin
                SetLength(bcdstr,int1);
                move(sqlite3_column_text(st,fnum)^,bcdstr[1],int1);
                // sqlite always uses the point as decimal-point
                if not TryStrToBCD(bcdstr,bcd,FSQLFormatSettings) then
                  // sqlite does the same, if the value can't be interpreted as a
                  // number in sqlite3_column_int, return 0
                  bcd := 0;
                end
              else
                bcd := 0;
              pBCD(buffer)^:= bcd;
              end;
<<<<<<< HEAD
    ftFixedWideChar,
    ftWideString:
      begin
      int1 := sqlite3_column_bytes16(st,fnum); //The value returned does not include the zero terminator at the end of the string
      if int1>FieldDef.Size*2 then
        int1:=FieldDef.Size*2;
      if int1 > 0 then
        move(sqlite3_column_text16(st,fnum)^, buffer^, int1); //Strings returned by sqlite3_column_text() and sqlite3_column_text16(), even empty strings, are always zero terminated.
      PWideChar(buffer + int1)^ := #0;
      end;
    ftVarBytes,
    ftBytes:
      begin
      int1 := sqlite3_column_bytes(st,fnum);
      if int1 > FieldDef.Size then
        int1 := FieldDef.Size;
      if FieldDef.DataType = ftVarBytes then
      begin
        PWord(buffer)^ := int1;
        inc(buffer, sizeof(Word));
      end;
      if int1 > 0 then
        move(sqlite3_column_blob(st,fnum)^, buffer^, int1);
      end;
    ftWideMemo,
=======
    ftFmtBCD: begin
              int1:= sqlite3_column_bytes(st,fnum);
              if int1>255 then
                int1:=255;
              if int1 > 0 then
                begin
                SetLength(bcdstr,int1);
                move(sqlite3_column_text(st,fnum)^,bcdstr[1],int1);
                StoreDecimalPoint:=FmtBCD.DecimalPoint;
                // sqlite always uses the point as decimal-point
                FmtBCD.DecimalPoint:=DecimalPoint_is_Point;
                if not TryStrToBCD(bcdstr,bcd) then
                  // sqlite does the same, if the value can't be interpreted as a
                  // number in sqlite3_column_int, return 0
                  bcd := 0;
                FmtBCD.DecimalPoint:=StoreDecimalPoint;
                end
              else
                bcd := 0;
              pBCD(buffer)^:= bcd;
              end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
    ftFmtBCD: begin
              int1:= sqlite3_column_bytes(st,fnum);
              if int1>255 then
                int1:=255;
              if int1 > 0 then
                begin
                SetLength(bcdstr,int1);
                move(sqlite3_column_text(st,fnum)^,bcdstr[1],int1);
                StoreDecimalPoint:=FmtBCD.DecimalPoint;
                // sqlite always uses the point as decimal-point
                FmtBCD.DecimalPoint:=DecimalPoint_is_Point;
                if not TryStrToBCD(bcdstr,bcd) then
                  // sqlite does the same, if the value can't be interpreted as a
                  // number in sqlite3_column_int, return 0
                  bcd := 0;
                FmtBCD.DecimalPoint:=StoreDecimalPoint;
                end
              else
                bcd := 0;
              pBCD(buffer)^:= bcd;
              end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    ftFmtBCD: begin
              int1:= sqlite3_column_bytes(st,fnum);
              if int1>255 then
                int1:=255;
              if int1 > 0 then
                begin
                SetLength(bcdstr,int1);
                move(sqlite3_column_text(st,fnum)^,bcdstr[1],int1);
                StoreDecimalPoint:=FmtBCD.DecimalPoint;
                // sqlite always uses the point as decimal-point
                FmtBCD.DecimalPoint:=DecimalPoint_is_Point;
                if not TryStrToBCD(bcdstr,bcd) then
                  // sqlite does the same, if the value can't be interpreted as a
                  // number in sqlite3_column_int, return 0
                  bcd := 0;
                FmtBCD.DecimalPoint:=StoreDecimalPoint;
                end
              else
                bcd := 0;
              pBCD(buffer)^:= bcd;
              end;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
              end;
>>>>>>> graemeg/fixes_2_2
=======
              end;
>>>>>>> origin/fixes_2_2
    ftMemo,
    ftBlob: CreateBlob:=True;
  else { Case }
   result:= false; // unknown
  end; { Case }
end;

function TSQLite3Connection.Fetch(cursor: TSQLCursor): boolean;

begin
  Result:=TSQLite3Cursor(cursor).Fetch;
end;

procedure TSQLite3Connection.FreeFldBuffers(cursor: TSQLCursor);
begin
 //dummy
end;

function TSQLite3Connection.GetTransactionHandle(trans: TSQLHandle): pointer;
begin
 result:= nil;
end;

function TSQLite3Connection.Commit(trans: TSQLHandle): boolean;
begin
  execsql('COMMIT');
  result:= true;
end;

function TSQLite3Connection.RollBack(trans: TSQLHandle): boolean;
begin
  execsql('ROLLBACK');
  result:= true;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TSQLite3Connection.StartDBTransaction(trans: TSQLHandle; aParams: string): boolean;
=======
function TSQLite3Connection.StartdbTransaction(trans: TSQLHandle;
               aParams: string): boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TSQLite3Connection.StartdbTransaction(trans: TSQLHandle;
               aParams: string): boolean;
>>>>>>> origin/fixes_2_2
begin
  execsql('BEGIN');
  result:= true;
end;

procedure TSQLite3Connection.CommitRetaining(trans: TSQLHandle);
begin
  commit(trans);  
  execsql('BEGIN');
end;

procedure TSQLite3Connection.RollBackRetaining(trans: TSQLHandle);
begin
  rollback(trans);
  execsql('BEGIN');
end;

procedure TSQLite3Connection.DoInternalConnect;
var
<<<<<<< HEAD
<<<<<<< HEAD
  filename: ansistring;
begin
  Inherited;
  if DatabaseName = '' then
    DatabaseError(SErrNoDatabaseName,self);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  InitializeSQLite;
  filename := DatabaseName;
  checkerror(sqlite3_open(PAnsiChar(filename),@fhandle));
  if (Length(Password)>0) and assigned(sqlite3_key) then
    checkerror(sqlite3_key(fhandle,PChar(Password),StrLen(PChar(Password))));
  if Params.IndexOfName('foreign_keys') <> -1 then
    execsql('PRAGMA foreign_keys =  '+Params.Values['foreign_keys']);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  InitializeSqlite(SQLiteLibraryName);
  str1:= databasename;
  checkerror(sqlite3_open(pchar(str1),@fhandle));
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  str1: string;
begin
  if Length(databasename)=0 then
    DatabaseError(SErrNoDatabaseName,self);
  initialisesqlite;
  str1:= databasename;
  checkerror(sqlite3_open(pchar(str1),@fhandle));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TSQLite3Connection.DoInternalDisconnect;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Inherited;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  if fhandle <> nil then 
    begin
    checkerror(sqlite3_close(fhandle));
    fhandle:= nil;
<<<<<<< HEAD
<<<<<<< HEAD
    ReleaseSQLite;
=======
    releasesqlite;
>>>>>>> graemeg/fixes_2_2
=======
    releasesqlite;
>>>>>>> origin/fixes_2_2
    end; 
end;

function TSQLite3Connection.GetHandle: pointer;
begin
  result:= fhandle;
end;

procedure TSQLite3Connection.checkerror(const aerror: integer);

Var
<<<<<<< HEAD
<<<<<<< HEAD
  ErrMsg : String;
  ErrCode : integer;
=======
  S : String;
>>>>>>> graemeg/fixes_2_2
=======
  S : String;
>>>>>>> origin/fixes_2_2

begin
 if (aerror<>sqlite_ok) then 
   begin
<<<<<<< HEAD
<<<<<<< HEAD
   ErrMsg := strpas(sqlite3_errmsg(fhandle));
   ErrCode := sqlite3_extended_errcode(fhandle);
   raise ESQLDatabaseError.CreateFmt(ErrMsg, [], Self, ErrCode, '');
=======
   S:=strpas(sqlite3_errmsg(fhandle));
   DatabaseError(S,Self);
>>>>>>> graemeg/fixes_2_2
=======
   S:=strpas(sqlite3_errmsg(fhandle));
   DatabaseError(S,Self);
>>>>>>> origin/fixes_2_2
   end;
end;

procedure TSQLite3Connection.execsql(const asql: string);
var
 err  : pchar;
 str1 : string;
 res  : integer;
begin
 err:= nil;
 Res := sqlite3_exec(fhandle,pchar(asql),nil,nil,@err);
 if err <> nil then 
   begin
   str1:= strpas(err);
   sqlite3_free(err);
   end;
 if (res<>sqlite_ok) then 
   databaseerror(str1);
end;

function execcallback(adata: pointer; ncols: longint; //adata = PStringArray
                avalues: PPchar; anames: PPchar):longint; cdecl;
var
  P : PStringArray;
  i : integer;
  
begin
  P:=PStringArray(adata); 
  SetLength(P^,ncols);
  for i:= 0 to ncols - 1 do 
    P^[i]:= strPas(avalues[i]);
  result:= 0;
end;

function execscallback(adata: pointer; ncols: longint; //adata = PArrayStringArray
                avalues: PPchar; anames: PPchar):longint; cdecl;
var
 I,N : integer;
 PP : PArrayStringArray;
 p  : PStringArray;
 
begin
 PP:=PArrayStringArray(adata);
 N:=high(PP^); // Length-1;
 setlength(PP^,N+2); // increase with 1;
 p:= @(PP^[N+1]); // newly added array, fill with data.
 setlength(p^,ncols); 
 for i:= 0 to ncols - 1 do 
   p^[i]:= strPas(avalues[i]);
 result:= 0;
end;

function TSQLite3Connection.stringsquery(const asql: string): TArrayStringArray;
begin
  SetLength(result,0);
  checkerror(sqlite3_exec(fhandle,pchar(asql),@execscallback,@result,nil));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TSQLite3Connection.GetSchemaInfoSQL(SchemaType: TSchemaType;
  SchemaObjectName, SchemaPattern: string): string;
  
begin
  case SchemaType of
    stTables     : result := 'select name as table_name from sqlite_master where type = ''table'' order by 1';
    stSysTables  : result := 'select ''sqlite_master'' as table_name';
    stColumns    : result := 'pragma table_info(''' + (SchemaObjectName) + ''')';
  else
    DatabaseError(SMetadataUnavailable)
  end; {case}
end;

procedure TSQLite3Connection.UpdateIndexDefs(IndexDefs: TIndexDefs; TableName: string);
var
  artableinfo, arindexlist, arindexinfo: TArrayStringArray;
  il,ii: integer;
  IndexName: string;
  IndexOptions: TIndexOptions;
  PKFields, IXFields: TStrings;

  function CheckPKFields:boolean;
  var i: integer;
  begin
    Result:=false;
    if IXFields.Count<>PKFields.Count then Exit;
    for i:=0 to IXFields.Count-1 do
      if PKFields.IndexOf(IXFields[i])<0 then Exit;
    Result:=true;
    PKFields.Clear;
  end;

begin
  PKFields:=TStringList.Create;
  PKFields.Delimiter:=';';
  IXFields:=TStringList.Create;
  IXFields.Delimiter:=';';

  //primary key fields; 5th column "pk" is zero for columns that are not part of PK
  artableinfo := stringsquery('PRAGMA table_info('+TableName+');');
  for ii:=low(artableinfo) to high(artableinfo) do
    if (high(artableinfo[ii]) >= 5) and (artableinfo[ii][5] >= '1') then
      PKFields.Add(artableinfo[ii][1]);

  //list of all table indexes
  arindexlist:=stringsquery('PRAGMA index_list('+TableName+');');
  for il:=low(arindexlist) to high(arindexlist) do
    begin
    IndexName:=arindexlist[il][1];
    if arindexlist[il][2]='1' then
      IndexOptions:=[ixUnique]
    else
      IndexOptions:=[];
    //list of columns in given index
    arindexinfo:=stringsquery('PRAGMA index_info('+IndexName+');');
    IXFields.Clear;
    for ii:=low(arindexinfo) to high(arindexinfo) do
      IXFields.Add(arindexinfo[ii][2]);

    if CheckPKFields then IndexOptions:=IndexOptions+[ixPrimary];

    IndexDefs.Add(IndexName, IXFields.DelimitedText, IndexOptions);
    end;

  if PKFields.Count > 0 then //in special case for INTEGER PRIMARY KEY column, unique index is not created
    IndexDefs.Add('$PRIMARY_KEY$', PKFields.DelimitedText, [ixPrimary,ixUnique]);

  PKFields.Free;
  IXFields.Free;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
function TSQLite3Connection.getprimarykeyfield(const atablename: string;
                                const acursor: tsqlcursor): string;
var
  int1,int2: integer;
  ar1: TArrayStringArray;
  str1: string;
  
begin
  result:= '';
  if atablename <> '' then 
    begin
    ar1:= stringsquery('PRAGMA table_info('+atablename+');');
    for int1:= 0 to high(ar1) do 
      begin
      if (high(ar1[int1]) >= 5) and (ar1[int1][5] <> '0') then 
        begin
        result:= ar1[int1][1];
        break;
        end;
      end;
    end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TSQLite3Connection.RowsAffected(cursor: TSQLCursor): TRowsCount;
begin
  if assigned(cursor) then
    Result := (cursor as TSQLite3Cursor).RowsAffected
  else
    Result := -1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TSQLite3Connection.RefreshLastInsertID(Query: TCustomSQLQuery; Field: TField): Boolean;
begin
  Field.AsLargeInt:=GetInsertID;
  Result:=True;
end;

function TSQLite3Connection.GetInsertID: int64;
begin
 result:= sqlite3_last_insert_rowid(fhandle);
end;

procedure TSQLite3Connection.GetFieldNames(const TableName: string;
  List: TStrings);
begin
<<<<<<< HEAD
  GetDBInfo(stColumns,TableName,'name',List);
end;

function TSQLite3Connection.GetConnectionInfo(InfoType: TConnInfoType): string;
begin
  Result:='';
  try
    InitializeSQLite;
    case InfoType of
      citServerType:
        Result:=TSQLite3ConnectionDef.TypeName;
      citServerVersion,
      citClientVersion:
        Result:=inttostr(sqlite3_libversion_number());
      citServerVersionString:
        Result:=sqlite3_libversion();
      citClientName:
        Result:=TSQLite3ConnectionDef.LoadedLibraryName;
    else
      Result:=inherited GetConnectionInfo(InfoType);
    end;
  finally
    ReleaseSqlite;
  end;
end;

procedure TSQLite3Connection.CreateDB;
var filename: ansistring;
begin
  CheckDisConnected;
  try
    InitializeSQLite;
    try
      filename := DatabaseName;
      checkerror(sqlite3_open(PAnsiChar(filename),@fhandle));
    finally
      sqlite3_close(fhandle);
      fhandle := nil;
    end;
  finally
    ReleaseSqlite;
  end;
end;

procedure TSQLite3Connection.DropDB;
begin
  CheckDisConnected;
  DeleteFile(DatabaseName);
end;

function UTF8CompareCallback(user: pointer; len1: longint; data1: pointer; len2: longint; data2: pointer): longint; cdecl;
var S1, S2: AnsiString;
begin
  SetString(S1, data1, len1);
  SetString(S2, data2, len2);
  Result := UnicodeCompareStr(UTF8Decode(S1), UTF8Decode(S2));
end;

procedure TSQLite3Connection.CreateCollation(const CollationName: string;
  eTextRep: integer; Arg: Pointer; Compare: xCompare);
begin
  if eTextRep = 0 then
  begin
    eTextRep := SQLITE_UTF8;
    Compare := @UTF8CompareCallback;
  end;
  CheckConnected;
  CheckError(sqlite3_create_collation(fhandle, PChar(CollationName), eTextRep, Arg, Compare));
end;

procedure TSQLite3Connection.LoadExtension(LibraryFile: string);
var
  LoadResult: integer;
begin
  CheckConnected; //Apparently we need a connection before we can load extensions.
  LoadResult:=SQLITE_ERROR; //Default to failed  
  try    
    LoadResult:=sqlite3_enable_load_extension(fhandle, 1); //Make sure we are allowed to load
    if LoadResult=SQLITE_OK then
      begin
      LoadResult:=sqlite3_load_extension(fhandle, PChar(LibraryFile), nil, nil); //Actually load extension
      if LoadResult=SQLITE_ERROR then
        begin
        DatabaseError('LoadExtension: failed to load SQLite extension (SQLite returned an error while loading).',Self);
        end;
      end
      else
      begin
        DatabaseError('LoadExtension: failed to load SQLite extension (SQLite returned an error while enabling extensions).',Self);
      end;
  except
    DatabaseError('LoadExtension: failed to load SQLite extension.',Self)
  end;
end;
=======
  inherited Create(AOwner);
  FConnOptions := FConnOptions + [sqEscapeRepeat] + [sqEscapeSlash];
  FieldNameQuoteChars:=DoubleQuotes;
=======
=======
>>>>>>> origin/fixes_2_2
function TSQLite3Connection.GetSchemaInfoSQL(SchemaType: TSchemaType;
  SchemaObjectName, SchemaPattern: string): string;
  
begin
  case SchemaType of
    stTables     : result := 'select name as table_name from sqlite_master where type = ''table''';
    stColumns    : result := 'pragma table_info(''' + (SchemaObjectName) + ''')';
  else
    DatabaseError(SMetadataUnavailable)
  end; {case}
end;

function TSQLite3Connection.StrToStatementType(s: string): TStatementType;
begin
  S:=Lowercase(s);
  if s = 'pragma' then exit(stSelect);
  result := inherited StrToStatementType(s);
end;

constructor TSQLite3Connection.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FConnOptions := FConnOptions + [sqEscapeRepeat] + [sqEscapeSlash] + [sqQuoteFieldnames];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TSQLite3Connection.UpdateIndexDefs(IndexDefs: TIndexDefs; TableName: string);
var
<<<<<<< HEAD
<<<<<<< HEAD
  artableinfo, arindexlist, arindexinfo: TArrayStringArray;
  il,ii: integer;
  IndexName: string;
  IndexOptions: TIndexOptions;
  PKFields, IXFields: TStrings;
  l: boolean;

  function CheckPKFields:boolean;
  var i: integer;
  begin
    Result:=false;
    if IXFields.Count<>PKFields.Count then Exit;
    for i:=0 to IXFields.Count-1 do
      if PKFields.IndexOf(IXFields[i])<0 then Exit;
    Result:=true;
    PKFields.Clear;
  end;

begin
  PKFields:=TStringList.Create;
  PKFields.Delimiter:=';';
  IXFields:=TStringList.Create;
  IXFields.Delimiter:=';';

  //primary key fields
  artableinfo := stringsquery('PRAGMA table_info('+TableName+');');
  for ii:=low(artableinfo) to high(artableinfo) do
    if (high(artableinfo[ii]) >= 5) and (artableinfo[ii][5] = '1') then
      PKFields.Add(artableinfo[ii][1]);

  //list of all table indexes
  arindexlist:=stringsquery('PRAGMA index_list('+TableName+');');
  for il:=low(arindexlist) to high(arindexlist) do
    begin
    IndexName:=arindexlist[il][1];
    if arindexlist[il][2]='1' then
      IndexOptions:=[ixUnique]
    else
      IndexOptions:=[];
    //list of columns in given index
    arindexinfo:=stringsquery('PRAGMA index_info('+IndexName+');');
    IXFields.Clear;
    for ii:=low(arindexinfo) to high(arindexinfo) do
      IXFields.Add(arindexinfo[ii][2]);

    if CheckPKFields then IndexOptions:=IndexOptions+[ixPrimary];

    IndexDefs.Add(IndexName, IXFields.DelimitedText, IndexOptions);
    end;

  if PKFields.Count > 0 then //in special case for INTEGER PRIMARY KEY column, unique index is not created
    IndexDefs.Add('$PRIMARY_KEY$', PKFields.DelimitedText, [ixPrimary,ixUnique]);

  PKFields.Free;
  IXFields.Free;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew


{ TSQLite3ConnectionDef }

class function TSQLite3ConnectionDef.TypeName: string;
begin
  Result := 'SQLite3';
end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

class function TSQLite3ConnectionDef.ConnectionClass: TSQLConnectionClass;
begin
  Result := TSQLite3Connection;
end;

class function TSQLite3ConnectionDef.Description: string;
begin
  Result := 'Connect to a SQLite3 database directly via the client library';
end;

class function TSQLite3ConnectionDef.DefaultLibraryName: string;
begin
  Result := SQLiteDefaultLibrary;
end;

class function TSQLite3ConnectionDef.LoadedLibraryName: string;
begin
  Result := SQLiteLoadedLibrary;
end;

class function TSQLite3ConnectionDef.LoadFunction: TLibraryLoadFunction;
begin
  Result:=@InitializeSQLiteANSI; //the function taking the filename argument
end;

class function TSQLite3ConnectionDef.UnLoadFunction: TLibraryUnLoadFunction;
begin
  Result:=@ReleaseSQLite;
end;

initialization
  RegisterConnection(TSQLite3ConnectionDef);

finalization
  UnRegisterConnection(TSQLite3ConnectionDef);

=======
=======
>>>>>>> origin/fixes_2_2
  str1: string;
  
begin
  str1:= getprimarykeyfield(tablename,nil);
  if str1 <> '' then 
    begin
    indexdefs.add('$PRIMARY_KEY$',str1,[ixPrimary,ixUnique]);
    end;
end;
{
procedure TSQLite3Connection.UpdateIndexDefs(var IndexDefs: TIndexDefs;
                              const TableName: string);
var
 int1,int2: integer;
 ar1: TArrayStringArray;
 str1: string;
begin
 ar1:= stringsquery('PRAGMA table_info('+tablename+');');
 for int1:= 0 to high(ar1) do begin
  if (high(ar1[int1]) >= 5) and (ar1[int1][5] <> '0') then begin
   indexdefs.add('$PRIMARY_KEY$',ar1[int1][1],[ixPrimary,ixUnique]);
   break;
  end;
 end;
end;
}

function TSQLite3Connection.getinsertid: int64;
begin
 result:= sqlite3_last_insert_rowid(fhandle);
end;

procedure TSQLite3Connection.GetFieldNames(const TableName: string;
  List: TStrings);
begin
  GetDBInfo(stColumns,TableName,'name',List);
end;

procedure TSQLite3Connection.setoptions(const avalue: tsqliteoptions);
begin
 if avalue <> foptions then 
   begin
   checkdisconnected;
   foptions:= avalue;
   end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
