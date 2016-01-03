unit CustomSqliteDS;

{
  This is TCustomSqliteDataset, a TDataset descendant class for use with fpc compiler
  Copyright (C) 2004-2007  Luiz Américo Pereira Câmara
  Email: pascalive@bol.com.br

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version with the following modification:

  As a special exception, the copyright holders of this library give you
  permission to link this library with independent modules to produce an
  executable, regardless of the license terms of these independent modules,and
  to copy and distribute the resulting executable under terms of your choice,
  provided that you also meet, for each linked independent module, the terms
  and conditions of the license of that module. An independent module is a
  module which is not derived from or based on this library. If you modify
  this library, you may extend this exception to your version of the library,
  but you are not obligated to do so. If you do not wish to do so, delete this
  exception statement from your version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
}

{$Mode ObjFpc}
{$H+}
{.$Define DEBUG_SQLITEDS}
{.$Define DEBUGACTIVEBUFFER}

interface

uses
  Classes, SysUtils, db;

const
  DefaultStringSize = 255;

type
  TCustomSqliteDataset = class;

  PDataRecord = ^DataRecord;
  PPDataRecord = ^PDataRecord;
  DataRecord = record
    Row: PPAnsiChar;
    BookmarkFlag: TBookmarkFlag;
    Next: PDataRecord;
    Previous: PDataRecord;
  end;
  
  { TDSStream }
  //todo: refactor into two or three classes
  TDSStream = class(TStream)
  private
    FEditItem: PDataRecord;
    FDataset: TCustomSqliteDataset;
    FFieldRow: PAnsiChar;
    FField: TField;
    FFieldOffset: Integer;
<<<<<<< HEAD
    FRowSize: Int64;
    FPosition: Int64;
    FWriteMode: Boolean;
  protected
    function GetPosition: Int64; override;
    function GetSize: Int64; override;
=======
    FRowSize: Integer;
    FPosition: LongInt;
    FWriteMode: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  public
    constructor Create(Dataset: TCustomSqliteDataset; Field: TField;
      FieldOffset: Integer; EditItem: PDataRecord; WriteMode: Boolean);
    destructor Destroy; override;
    function Write(const Buffer; Count: LongInt): LongInt; override;
    function Read(var Buffer; Count: LongInt): LongInt; override;
    function Seek(const Offset: int64; Origin: TSeekOrigin): int64; override;
  end;

  //callback types
  TSqliteCdeclCallback = function(UserData: Pointer; Count: LongInt; Values: PPAnsiChar; Names: PPAnsiChar): LongInt; cdecl;
  TSqliteCallback = function(UserData: Pointer; Count: LongInt; Values: PPAnsiChar; Names: PPAnsiChar): LongInt of object;
  TCallbackInfo = record
    Proc: TSqliteCallback;
    Data: Pointer;
  end;
  PCallbackInfo = ^TCallbackInfo;
  
  TRecordState = (rsAdded, rsDeleted, rsUpdated);
  TRecordStateSet = set of TRecordState;
  TQueryUpdatesCallback = procedure(UserData: Pointer; Values: PPAnsiChar; ABookmark: TBookmark; RecordState: TRecordState) of object;

  TGetSqlStrFunction = function(APChar: PAnsiChar): String;

  TSqliteOption = (soWildcardKey);
  TSqliteOptions = set of TSqliteOption;

  TSqliteOption = (soWildcardKey);
  TSqliteOptions = set of TSqliteOption;

  TSqliteOption = (soWildcardKey);
  TSqliteOptions = set of TSqliteOption;

  { TCustomSqliteDataset }

  TCustomSqliteDataset = class(TDataSet)
  private
    {$ifdef DEBUGACTIVEBUFFER}
    FFCurrentItem: PDataRecord;
    {$else}
    FCurrentItem: PDataRecord;
    {$endif}
    FInternalActiveBuffer: PDataRecord;
    FInsertBookmark: PDataRecord;
<<<<<<< HEAD
<<<<<<< HEAD
    FFilterBuffer: TRecordBuffer;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FOnCallback: TSqliteCallback;
    FMasterLink: TMasterDataLink;
    FIndexFieldNames: String;
    FIndexFieldList: TList;
    FOnGetHandle: TDataSetNotifyEvent;
    FOptions: TSqliteOptions;
<<<<<<< HEAD
<<<<<<< HEAD
    FSQLList: TStrings;
    FStoreDefs: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    FSqlList:TStrings;
>>>>>>> graemeg/fixes_2_2
    procedure CopyCacheToItem(AItem: PDataRecord);
>>>>>>> graemeg/cpstrnew
    function GetIndexFields(Value: Integer): TField;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetSQLList: TStrings;
    procedure SetMasterIndexValue;
    procedure SetOptions(const AValue: TSqliteOptions);
    procedure UpdateCalcFieldList;
    procedure UpdateIndexFieldList;
    function FindRecordItem(StartItem: PDataRecord; const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions; DoResync: Boolean): PDataRecord;
    procedure UpdateMasterDetailProperties;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure SetMasterIndexValue;
    procedure SetOptions(const AValue: TSqliteOptions);
    procedure UpdateIndexFields;
    function FindRecordItem(StartItem: PDataRecord; const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions; DoResync:Boolean):PDataRecord;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  protected
    FPrimaryKey: String;
    FPrimaryKeyNo: Integer;
    FFileName: UTF8String;
    FSQL: String;
    FEffectiveSQL: String;
    FTableName: String;
    FSqlFilterTemplate: String;
    FAutoIncFieldNo: Integer;
    FNextAutoInc: Integer;
    FUpdatedItems: TFPList;
    FAddedItems: TFPList;
    FDeletedItems: TFPList;
<<<<<<< HEAD
<<<<<<< HEAD
    FCalcFieldList: TFPList;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FReturnCode: Integer;
    FSqliteHandle: Pointer;
    FRowBufferSize: Integer;
    FRowCount: Integer;
    FRecordCount: Integer;
    FBeginItem: PDataRecord;
    FEndItem: PDataRecord;
    FSavedEditItem: PDataRecord;
    FGetSqlStr: array of TGetSqlStrFunction;
    FSaveOnClose: Boolean;
    FSaveOnRefetch: Boolean;
    FAutoIncrementKey: Boolean;
    FDataAllocated: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    function SqliteExec(Sql: PAnsiChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer; virtual; abstract;
    procedure InternalCloseHandle; virtual; abstract;
=======
=======
>>>>>>> origin/fixes_2_2
    function SqliteExec(Sql:PChar; ACallback: TSqliteCdeclCallback; Data: Pointer):Integer;virtual; abstract;
    procedure InternalCloseHandle;virtual;abstract;
>>>>>>> graemeg/fixes_2_2
    function InternalGetHandle: Pointer; virtual; abstract;
    function FieldDefsStored: Boolean;
    function GetLastInsertRowId: Int64; virtual; abstract;
    procedure GetSqliteHandle;
    procedure BuildLinkedList; virtual; abstract;
    procedure FreeItem(AItem: PDataRecord);
    procedure DisposeLinkedList;
    procedure SetDetailFilter;
    procedure MasterChanged(Sender: TObject);
<<<<<<< HEAD
    procedure SetMasterFields(const Value: String);
    function GetMasterFields: String;
    procedure SetMasterSource(Value: TDataSource);
    function GetMasterSource: TDataSource;
    procedure SetFileName(const Value: UTF8String);
    function GetRowsAffected: Integer; virtual; abstract;
    procedure RetrieveFieldDefs; virtual; abstract;
=======
    procedure MasterDisabled(Sender: TObject);
    procedure SetMasterFields(const Value:String);
    function GetMasterFields:String;
    procedure SetMasterSource(Value: TDataSource);
    function GetMasterSource:TDataSource;
    procedure SetFileName(const Value: String);
    function GetRowsAffected:Integer; virtual;abstract;
>>>>>>> graemeg/fixes_2_2
    //TDataSet overrides
<<<<<<< HEAD
    function AllocRecordBuffer: TRecordBuffer; override;
    procedure ClearCalcFields(Buffer: TRecordBuffer); override;
=======
    function AllocRecordBuffer: PChar; override;
<<<<<<< HEAD
    procedure ClearCalcFields(Buffer: PChar); override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
    function CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure DoBeforeClose; override;
    procedure DoAfterInsert; override;
    procedure DoBeforeInsert; override;
    procedure DoFilterRecord(var Acceptable: Boolean); virtual;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); override;
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    function GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; override;
    function GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
    function GetRecordCount: Integer; override;
    function GetRecNo: Integer; override;
    function GetRecordSize: Word; override; 
    procedure InternalAddRecord(Buffer: Pointer; DoAppend: Boolean); override;
    procedure InternalClose; override;
    procedure InternalCancel; override;
    procedure InternalDelete; override;
    procedure InternalEdit; override;
    procedure InternalFirst; override;
    procedure InternalGotoBookmark(ABookmark: Pointer); override;
    procedure InternalInitFieldDefs; override;
    procedure InternalInitRecord(Buffer: TRecordBuffer); override;
    procedure InternalLast; override;
    procedure InternalOpen; override;
    procedure InternalPost; override;
    procedure InternalSetToRecord(Buffer: TRecordBuffer); override;
    function IsCursorOpen: Boolean; override;
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); override;
    procedure SetExpectedAppends(AValue: Integer);
    procedure SetExpectedUpdates(AValue: Integer);
    procedure SetExpectedDeletes(AValue: Integer);
    procedure SetRecNo(Value: Integer); override;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    function BookmarkValid(ABookmark: TBookmark): Boolean; override;
    function CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint; override;
    function CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; override;
    function GetFieldData(Field: TField; Buffer: Pointer): Boolean; override;
    function GetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean): Boolean; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function Locate(const KeyFields: String; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean; override;
    function LocateNext(const KeyFields: String; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
    function Lookup(const KeyFields: String; const KeyValues: Variant; const ResultFields: String): Variant; override;
    procedure SetFieldData(Field: TField; Buffer: Pointer); override;
    procedure SetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean); override;
=======
    function Locate(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean; override;
    function LocateNext(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
    function Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant;override;
>>>>>>> graemeg/fixes_2_2
=======
    function Locate(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean; override;
    function LocateNext(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
    function Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant;override;
>>>>>>> origin/fixes_2_2
    // Additional procedures
    function ApplyUpdates: Boolean;
    procedure ClearUpdates(RecordStates: TRecordStateSet = [rsAdded, rsDeleted, rsUpdated]);
    function CreateTable: Boolean;
    function CreateTable(const ATableName: String): Boolean;
    procedure ExecCallback(const ASql: String; UserData: Pointer = nil);
    procedure ExecSQL;
    procedure ExecSQL(const ASql: String);
    procedure ExecSQL(ASqlList: TStrings);
    procedure ExecSQLList;
    procedure ExecuteDirect(const ASql: String); virtual; abstract;
    function GetSQLValue(Values: PPAnsiChar; FieldIndex: Integer): String;
    procedure QueryUpdates(RecordStates: TRecordStateSet; Callback: TQueryUpdatesCallback; UserData: Pointer = nil);
    function QuickQuery(const ASql: String):String;overload;
    function QuickQuery(const ASql: String; const AStrList: TStrings): String; overload;
    function QuickQuery(const ASql: String; const AStrList: TStrings; FillObjects: Boolean):String; virtual; abstract; overload;
    procedure RefetchData;
    function ReturnString: String; virtual; abstract;
    class function SqliteVersion: String; virtual; abstract;
    function TableExists: Boolean;
    function TableExists(const ATableName: String): Boolean;
    function UpdatesPending: Boolean;
    {$ifdef DEBUGACTIVEBUFFER}
    procedure SetCurrentItem(Value: PDataRecord);
    property FCurrentItem: PDataRecord read FFCurrentItem write SetCurrentItem;
    {$endif}
    property ExpectedAppends: Integer write SetExpectedAppends;
    property ExpectedUpdates: Integer write SetExpectedUpdates;
    property ExpectedDeletes: Integer write SetExpectedDeletes;
    property IndexFields[Value: Integer]: TField read GetIndexFields;
    property LastInsertRowId: Int64 read GetLastInsertRowId;
    property RowsAffected: Integer read GetRowsAffected;
    property ReturnCode: Integer read FReturnCode;
    property SqliteHandle: Pointer read FSqliteHandle;
    property SQLList: TStrings read GetSQLList;
   published
    property AutoIncrementKey: Boolean read FAutoIncrementKey write FAutoIncrementKey default False;
    property IndexFieldNames: string read FIndexFieldNames write FIndexFieldNames;
    property FileName: UTF8String read FFileName write SetFileName;
    property OnCallback: TSqliteCallback read FOnCallback write FOnCallback;
    property OnGetHandle: TDataSetNotifyEvent read FOnGetHandle write FOnGetHandle;
<<<<<<< HEAD
<<<<<<< HEAD
    property Options: TSqliteOptions read FOptions write SetOptions default [];
=======
    property Options: TSqliteOptions read FOptions write SetOptions;
>>>>>>> graemeg/fixes_2_2
=======
    property Options: TSqliteOptions read FOptions write SetOptions;
>>>>>>> origin/fixes_2_2
    property PrimaryKey: String read FPrimaryKey write FPrimaryKey;
    property SaveOnClose: Boolean read FSaveOnClose write FSaveOnClose default False;
    property SaveOnRefetch: Boolean read FSaveOnRefetch write FSaveOnRefetch default False;
    property SQL: String read FSQL write FSQL;
    property StoreDefs: Boolean read FStoreDefs write FStoreDefs default False;
    property TableName: String read FTableName write FTableName;   
    property MasterSource: TDataSource read GetMasterSource write SetMasterSource;
    property MasterFields: String read GetMasterFields write SetMasterFields;
    
    property Active;
<<<<<<< HEAD
<<<<<<< HEAD
    property FieldDefs stored FieldDefsStored;
=======
    property FieldDefs;   
>>>>>>> graemeg/fixes_2_2
=======
    property FieldDefs;   
>>>>>>> origin/fixes_2_2
    //Events
    property BeforeOpen;
    property AfterOpen;
    property BeforeClose;
    property AfterClose;
    property BeforeInsert;
    property AfterInsert;
    property BeforeEdit;
    property AfterEdit;
    property BeforePost;
    property AfterPost;
    property BeforeCancel;
    property AfterCancel;
    property BeforeDelete;
    property AfterDelete;
    property BeforeScroll;
    property AfterScroll;
    property BeforeRefresh;
    property AfterRefresh;
<<<<<<< HEAD
<<<<<<< HEAD
    property OnCalcFields;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property OnDeleteError;
    property OnEditError;
    property OnNewRecord;
    property OnPostError;
  end;
  
  function Num2SQLStr(APChar: PAnsiChar): String;
  function Char2SQLStr(APChar: PAnsiChar): String;
  function Memo2SQLStr(APChar: PAnsiChar): String;
  function StrBufNew(p : PAnsiChar): PAnsiChar;
  function StrBufNew(p : PAnsiChar; BufLen: Cardinal): PAnsiChar;

implementation

uses
  strutils, variants, dbconst;

const
  //sqlite2.x.x and sqlite3.x.x define these constants equally
  SQLITE_OK = 0;
  SQLITE_ROW = 100;
  SQLITE_DONE = 101;
  
  NullString = 'NULL';

function StrBufNew(p : PAnsiChar): PAnsiChar;
var
  BufLen : Cardinal;
begin
  Result := nil;
  if (p = nil) or (p^ = #0) then
    Exit;
  BufLen := StrBufSize(p);
  Result := StrAlloc(BufLen);
  if Result <> nil then
    Move(p^, Result^, BufLen);
end;

function StrBufNew(p: PChar; BufLen: Cardinal): PChar;
begin
  Result := nil;
  if (p = nil) or (p^ = #0) then
    Exit;
  Result := StrAlloc(BufLen);
  if Result <> nil then
    Move(p^, Result^, BufLen);
end;
  

function CallbackDispatcher(UserData: Pointer; Count: LongInt; Values: PPAnsiChar; Names: PPAnsiChar): LongInt; cdecl;
begin
  with PCallbackInfo(UserData)^ do
    Result:= Proc(Data, Count, Values, Names);
end;
  
function Num2SQLStr(APChar: PAnsiChar): String;
begin
  if APChar = nil then
  begin
    Result := NullString;
    Exit;
  end;
  Result := String(APChar);
end;

function Char2SQLStr(APChar: PAnsiChar): String;
begin
  if APChar = nil then
  begin
    Result := NullString;
    Exit;
  end;
<<<<<<< HEAD
  //todo: create custom routine to directly transform PAnsiChar -> SQL str
=======
  //todo: create custom routine to directly transform PChar -> SQL str
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result := String(APChar);
  if Pos('''', Result) > 0 then
    Result := AnsiReplaceStr(Result, '''', '''''');
  Result := '''' + Result + '''';
<<<<<<< HEAD
<<<<<<< HEAD
end;

function Memo2SQLStr(APChar: PAnsiChar): String;
var
  Len: Cardinal;
begin
  if APChar = nil then
  begin
    Result := NullString;
    Exit;
  end;
  //todo: create custom routine to directly transform PAnsiChar -> SQL str
  Len := StrBufSize(APChar) - 1;
  SetLength(Result, Len);
  Move(APChar^, Result[1], Len);
  if Pos('''', Result) > 0 then
    Result := AnsiReplaceStr(Result, '''', '''''');
  Result := '''' + Result + '''';
  if Pos(#0, Result) > 0 then
    Result := AnsiReplaceStr(Result, #0, '''||x''00''||''');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

// TDSStream

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TDSStream.GetPosition: Int64;
begin
  Result:=FPosition;
end;

function TDSStream.GetSize: Int64;
begin
  Result:=FRowSize;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TDSStream.Create(Dataset: TCustomSqliteDataset; Field: TField;
  FieldOffset: Integer; EditItem: PDataRecord; WriteMode: Boolean);
begin
  inherited Create;
  //FPosition := 0;
<<<<<<< HEAD
<<<<<<< HEAD
  FDataset := Dataset;
  FField := Field;
  FFieldOffset := FieldOffset;
  FWriteMode := WriteMode;
  FEditItem := EditItem;
  FFieldRow := FEditItem^.Row[FFieldOffset];
  if FFieldRow <> nil then
    FRowSize := StrBufSize(FFieldRow) - 1;
  //else
  //  FRowSize := 0;  
end;
<<<<<<< HEAD
=======
=======
  FActiveItem := ActiveItem;
  FFieldIndex := FieldIndex;
  FFieldRow := ActiveItem^.Row[FieldIndex];
  if FFieldRow <> nil then
=======
  FActiveItem := ActiveItem;
  FFieldIndex := FieldIndex;
  FFieldRow := ActiveItem^.Row[FieldIndex];
  if FFieldRow <> nil then
>>>>>>> origin/fixes_2_2
    FRowSize := StrLen(FFieldRow);
  //else
  //  FRowSize := 0;  
end;  
>>>>>>> graemeg/fixes_2_2

destructor TDSStream.Destroy;
begin
  if FWriteMode and not (FDataset.State in [dsCalcFields, dsFilter, dsNewValue]) then
    FDataset.DataEvent(deFieldChange, PtrInt(FField));
  inherited Destroy;
end;
>>>>>>> origin/cpstrnew

destructor TDSStream.Destroy;
begin
  if FWriteMode and not (FDataset.State in [dsCalcFields, dsFilter, dsNewValue]) then
    FDataset.DataEvent(deFieldChange, PtrInt(FField));
  inherited Destroy;
end;

function TDSStream.Seek(const Offset: int64; Origin: TSeekOrigin): int64;
begin
  Case Origin of
    soBeginning : FPosition := Offset;
    soEnd       : FPosition := FRowSize + Offset;
    soCurrent   : FPosition := FPosition + Offset;
  end;
  Result := FPosition;
end;

function TDSStream.Write(const Buffer; Count: LongInt): LongInt;
var
  NewRow: PAnsiChar;
begin
<<<<<<< HEAD
  Result := Count;
  if Count > 0 then
  begin
    //FRowSize is always 0 when FPosition = 0,
    //so there's no need to check FPosition
    NewRow := StrAlloc(FRowSize + Count + 1);
    (NewRow + Count + FRowSize)^ := #0;
    if FRowSize > 0 then
      Move(FFieldRow^, NewRow^, FRowSize);
    Move(Buffer, (NewRow + FRowSize)^, Count);
    FEditItem^.Row[FFieldOffset] := NewRow;
    StrDispose(FFieldRow);
    {$ifdef DEBUG_SQLITEDS}
    WriteLn('##TDSStream.Write##');
    WriteLn('  FPosition(Before): ', FPosition);
    WriteLn('  FRowSize(Before): ', FRowSize);
    WriteLn('  FPosition(After): ', FPosition+Count);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    WriteLn('  FRowSize(After): ', StrBufSize(NewRow) -1);
    //WriteLn('  Stream Value: ',NewRow);
    {$endif}
    FFieldRow := NewRow;
    FRowSize := StrBufSize(NewRow) - 1;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    WriteLn('  FRowSize(After): ', StrLen(NewRow));
    //WriteLn('  Stream Value: ',NewRow);
    {$endif}
    FFieldRow := NewRow;
    FRowSize := StrLen(NewRow);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Inc(FPosition, Count);
  end;
=======
  Result:=Count;
  if Count = 0 then
    Exit;
  //FRowSize is always 0 when FPosition = 0,
  //so there's no need to check FPosition
  NewRow:=StrAlloc(FRowSize+Count+1);
  (NewRow+Count+FRowSize)^:=#0;
  if FRowSize > 0 then
    Move(FFieldRow^,NewRow^,FRowSize);
  Move(Buffer,(NewRow+FRowSize)^,Count);
  FActiveItem^.Row[FFieldIndex]:=NewRow;    
  StrDispose(FFieldRow);
  {$ifdef DEBUG_SQLITEDS}
  WriteLn('##TDSStream.Write##');
  WriteLn('  FPosition(Before): ',FPosition);
  WriteLn('  FRowSize(Before): ',FRowSize);
  WriteLn('  FPosition(After): ',FPosition+Count);
  WriteLn('  FRowSize(After): ',StrLen(NewRow));
  //WriteLn('  Stream Value: ',NewRow);
  {$endif}
  FFieldRow:=NewRow;
  FRowSize:=StrLen(NewRow);
  Inc(FPosition,Count);
>>>>>>> graemeg/fixes_2_2
end; 
 
function TDSStream.Read(var Buffer; Count: Longint): LongInt;
var
  BytesToMove: Integer;
begin
  if (FRowSize - FPosition) >= Count then
    BytesToMove := Count
  else
<<<<<<< HEAD
    BytesToMove := FRowSize - FPosition;
  Move((FFieldRow + FPosition)^, Buffer, BytesToMove);
  Inc(FPosition, BytesToMove);
  Result := BytesToMove;
=======
    BytesToMove:=FRowSize - FPosition;   
  Move((FFieldRow+FPosition)^,Buffer,BytesToMove);
  Inc(FPosition,BytesToMove);
  Result:=BytesToMove;  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$ifdef DEBUG_SQLITEDS}
  WriteLn('##TDSStream.Read##');
  WriteLn('  Bytes requested: ', Count);
  WriteLn('  Bytes moved: ', BytesToMove);
  WriteLn('  Stream.Size: ', FRowSize);
  //WriteLn('  Stream Value: ', FFieldRow);
  {$endif}
end; 
 
// TCustomSqliteDataset override methods

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSqliteDataset.AllocRecordBuffer: TRecordBuffer;
begin
  Result := AllocMem(SizeOf(PPDataRecord));
  PDataRecord(Pointer(Result)^) := FBeginItem;
end;

procedure TCustomSqliteDataset.ClearCalcFields(Buffer: TRecordBuffer);
var
  i: Integer;
  RecordItem: PDataRecord;
begin
  if FCalcFieldList = nil then
    Exit;
  RecordItem := PPDataRecord(Buffer)^;
  for i := FieldDefs.Count to FieldDefs.Count + FCalcFieldList.Count - 1 do
  begin
    StrDispose(RecordItem^.Row[i]);
    RecordItem^.Row[i] := nil;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomSqliteDataset.AllocRecordBuffer: PChar;
begin
  Result := AllocMem(SizeOf(PPDataRecord));
  PDataRecord(Pointer(Result)^):=FBeginItem;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

constructor TCustomSqliteDataset.Create(AOwner: TComponent);
begin
  // setup special items
  New(FBeginItem);
  New(FSavedEditItem);
  New(FEndItem);
  
  FBeginItem^.Previous := nil;
  FEndItem^.Next := nil;
  
<<<<<<< HEAD
<<<<<<< HEAD
  FBeginItem^.BookmarkFlag := bfBOF;
  FEndItem^.BookmarkFlag := bfEOF;
  
  FMasterLink := TMasterDataLink.Create(Self);
  FMasterLink.OnMasterChange := @MasterChanged;
  FMasterLink.OnMasterDisable := @MasterChanged;
=======
  FBeginItem^.BookMarkFlag := bfBOF;
  FEndItem^.BookMarkFlag := bfEOF;
  
  FMasterLink := TMasterDataLink.Create(Self);
  FMasterLink.OnMasterChange := @MasterChanged;
  FMasterLink.OnMasterDisable := @MasterDisabled;
  FIndexFieldList := TList.Create;
>>>>>>> graemeg/fixes_2_2
=======
  FBeginItem^.BookMarkFlag := bfBOF;
  FEndItem^.BookMarkFlag := bfEOF;
  
  FMasterLink := TMasterDataLink.Create(Self);
  FMasterLink.OnMasterChange := @MasterChanged;
  FMasterLink.OnMasterDisable := @MasterDisabled;
  FIndexFieldList := TList.Create;
>>>>>>> origin/fixes_2_2
  BookmarkSize := SizeOf(Pointer);
  FUpdatedItems := TFPList.Create;
  FAddedItems := TFPList.Create;
  FDeletedItems := TFPList.Create;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  FSqlList := TStringList.Create;
>>>>>>> graemeg/fixes_2_2
=======
  FSqlList := TStringList.Create;
>>>>>>> origin/fixes_2_2
  inherited Create(AOwner);
end;

function TCustomSqliteDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;
var
  FieldOffset: Integer;
  EditItem: PDataRecord;
begin
  if Field.FieldNo >= 0 then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    EditItem := PPDataRecord(ActiveBuffer)^;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    if Mode = bmWrite then
      EditItem := FCacheItem
    else
      EditItem := PPDataRecord(ActiveBuffer)^;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FieldOffset := Field.FieldNo - 1;
  end
  else
  begin
    EditItem := PPDataRecord(CalcBuffer)^;
    FieldOffset := FieldDefs.Count + FCalcFieldList.IndexOf(Field);
  end;
  if Mode = bmWrite then
  begin
    if not (State in [dsEdit, dsInsert, dsCalcFields]) then
      DatabaseErrorFmt(SNotEditing, [Name], Self);
    StrDispose(EditItem^.Row[FieldOffset]);
    EditItem^.Row[FieldOffset] := nil;
  end;
  Result := TDSStream.Create(Self, Field, FieldOffset, EditItem, Mode = bmWrite);
end;

procedure TCustomSqliteDataset.DoBeforeClose;
begin
  if FSaveOnClose then
    ApplyUpdates;
  inherited DoBeforeClose;
end;

procedure TCustomSqliteDataset.DoBeforeClose;
begin
  if FSaveOnClose then
    ApplyUpdates;
  inherited DoBeforeClose;
end;

procedure TCustomSqliteDataset.DoBeforeClose;
begin
  if FSaveOnClose then
    ApplyUpdates;
  inherited DoBeforeClose;
end;

procedure TCustomSqliteDataset.DoAfterInsert;
begin
  //an append or an insert in an empty dataset
  if EOF then
    FInsertBookmark := FEndItem
  else
    FInsertBookmark := FInternalActiveBuffer;
  inherited DoAfterInsert;
end;

procedure TCustomSqliteDataset.DoBeforeInsert;
begin
  FInternalActiveBuffer := PPDataRecord(ActiveBuffer)^;
  inherited DoBeforeInsert;
end;

procedure TCustomSqliteDataset.DoFilterRecord(var Acceptable: Boolean);
begin
  Acceptable := True;
  if Assigned(OnFilterRecord) then
    OnFilterRecord(Self, Acceptable);
end;

destructor TCustomSqliteDataset.Destroy;
begin
  inherited Destroy;
  if FSqliteHandle <> nil then
    InternalCloseHandle;
  FUpdatedItems.Destroy;
  FAddedItems.Destroy;
  FDeletedItems.Destroy;
  FMasterLink.Destroy;
  //lists created on demand
  FSQLList.Free;
  FIndexFieldList.Free;
  FCalcFieldList.Free;
  // dispose special items
  Dispose(FBeginItem);
  Dispose(FSavedEditItem);
  Dispose(FEndItem);
end;

function TCustomSqliteDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
var
  TempItem: PDataRecord;
begin
  Result := False;
  if ABookmark = nil then
    Exit;
  TempItem := FBeginItem^.Next;
  while TempItem <> FEndItem do
  begin
    if TempItem = PPDataRecord(ABookmark)^ then
    begin
      Result := True;
      Exit;
    end;
    TempItem := TempItem^.Next;
  end;
end;

function TCustomSqliteDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
var
  TempItem: PDataRecord;
begin
  Result := 0;
  if (Bookmark1 = nil) or (Bookmark2 = nil) then
  begin
    if Bookmark1 <> nil then
      Result := -1
    else if Bookmark2 <> nil then
      Result := 1;
    Exit;
  end;
  if PPDataRecord(Bookmark1)^ = PPDataRecord(Bookmark2)^ then
    Exit;
  //assume Bookmark1 < Bookmark2
  Result := -1;
  TempItem := PPDataRecord(Bookmark1)^^.Previous;
  while TempItem <> FBeginItem do
  begin
    if TempItem = PPDataRecord(Bookmark2)^ then
    begin
      //Bookmark1 is greater than Bookmark2
      Result := 1;
      Exit;
    end;
    TempItem := TempItem^.Previous;
  end;
end;

function TCustomSqliteDataset.GetIndexFields(Value: Integer): TField;
begin
  Result := TField(FIndexFieldList[Value]);
end;

function TCustomSqliteDataset.GetSQLList: TStrings;
begin
  if FSQLList = nil then
    FSQLList := TStringList.Create;
  Result := FSQLList;
end;

function TCustomSqliteDataset.GetSQLList: TStrings;
begin
  if FSQLList = nil then
    FSQLList := TStringList.Create;
  Result := FSQLList;
end;

function TCustomSqliteDataset.GetSQLList: TStrings;
begin
  if FSQLList = nil then
    FSQLList := TStringList.Create;
  Result := FSQLList;
end;

function TCustomSqliteDataset.GetSQLList: TStrings;
begin
  if FSQLList = nil then
    FSQLList := TStringList.Create;
  Result := FSQLList;
end;

function TCustomSqliteDataset.GetSQLList: TStrings;
begin
  if FSQLList = nil then
    FSQLList := TStringList.Create;
  Result := FSQLList;
end;

procedure TCustomSqliteDataset.SetMasterIndexValue;
var
  i: Integer;
begin
  for i := 0 to FIndexFieldList.Count - 1 do
    TField(FIndexFieldList[i]).Value := TField(FMasterLink.Fields[i]).Value;
end;

procedure TCustomSqliteDataset.SetOptions(const AValue: TSqliteOptions);
begin
  FOptions := AValue;
end;

procedure TCustomSqliteDataset.UpdateCalcFieldList;
var
  i: Integer;
  AField: TField;
begin
<<<<<<< HEAD
  if FCalcFieldList = nil then
    FCalcFieldList := TFPList.Create
  else
    FCalcFieldList.Clear;
  for i := 0 to Fields.Count - 1 do
  begin
    AField := Fields[i];
    if AField.FieldKind in [fkCalculated, fkLookup] then
      FCalcFieldList.Add(AField);
  end;
=======
  Result := TField(FIndexFieldList[Value]);
end;

procedure TCustomSqliteDataset.SetMasterIndexValue;
var
  i: Integer;
begin
  for i := 0 to FIndexFieldList.Count - 1 do
    TField(FIndexFieldList[i]).AsString := TField(FMasterLink.Fields[i]).AsString;
end;

procedure TCustomSqliteDataset.SetOptions(const AValue: TSqliteOptions);
begin
<<<<<<< HEAD
  FOptions := AValue;
>>>>>>> graemeg/fixes_2_2
=======
  Result := TField(FIndexFieldList[Value]);
end;

procedure TCustomSqliteDataset.SetMasterIndexValue;
var
  i: Integer;
begin
  for i := 0 to FIndexFieldList.Count - 1 do
    TField(FIndexFieldList[i]).AsString := TField(FMasterLink.Fields[i]).AsString;
end;

procedure TCustomSqliteDataset.SetOptions(const AValue: TSqliteOptions);
begin
  FOptions := AValue;
>>>>>>> origin/fixes_2_2
end;

procedure TCustomSqliteDataset.DisposeLinkedList;
var
<<<<<<< HEAD
  TempItem: PDataRecord;
  i: Integer;
begin
  //Todo: insert debug info
  //Todo: see if FDataAllocated is still necessary
  FDataAllocated := False;
  TempItem := FBeginItem^.Next;
  while TempItem^.Next <> nil do
  begin
    TempItem := TempItem^.Next;
=======
  TempItem:PDataRecord;
  i:Integer;
begin
  //Todo: insert debug info
  //Todo: see if FDataAllocated is still necessary
  FDataAllocated:=False;
  TempItem:=FBeginItem^.Next;
  while TempItem^.Next <> nil do
  begin
    TempItem:=TempItem^.Next;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FreeItem(TempItem^.Previous);
  end; 

  //Dispose Deleted Items
  //Directly access list pointer since the index check is already done in the loop
<<<<<<< HEAD
<<<<<<< HEAD
  for i := 0 to FDeletedItems.Count - 1 do
=======
  for i:= 0 to FDeletedItems.Count - 1 do
>>>>>>> graemeg/fixes_2_2
=======
  for i:= 0 to FDeletedItems.Count - 1 do
>>>>>>> origin/fixes_2_2
    FreeItem(PDataRecord(FDeletedItems.List^[i]));

  //Dispose FBeginItem.Row
  for i := 0 to FRowCount - 1 do
    StrDispose(FBeginItem^.Row[i]);
  FreeMem(FBeginItem^.Row, FRowBufferSize);
    
<<<<<<< HEAD
<<<<<<< HEAD
  //Dispose edit item row
  for i := 0 to FRowCount - 1 do
    StrDispose(FSavedEditItem^.Row[i]);
  FreeMem(FSavedEditItem^.Row, FRowBufferSize);
=======
=======
>>>>>>> origin/fixes_2_2
  //Dispose cache item row
  for i:= 0 to FRowCount - 1 do
    StrDispose(FCacheItem^.Row[i]);
  FreeMem(FCacheItem^.Row,FRowBufferSize);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TCustomSqliteDataset.FreeRecordBuffer(var Buffer: TRecordBuffer);
begin
  FreeMem(Buffer);
end;

procedure TCustomSqliteDataset.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
begin
  Pointer(Data^) := PPDataRecord(Buffer)^;
end;

function TCustomSqliteDataset.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag;
begin
  Result := PPDataRecord(Buffer)^^.BookmarkFlag;
end;

function TCustomSqliteDataset.GetFieldData(Field: TField; Buffer: Pointer;
  NativeFormat: Boolean): Boolean;
var
  ValError: Word;
<<<<<<< HEAD
<<<<<<< HEAD
  FieldRow: PAnsiChar;
  FieldOffset: Integer;
begin
  if Field.FieldNo >= 0 then
    FieldOffset := Field.FieldNo - 1
  else
    FieldOffset := FieldDefs.Count + FCalcFieldList.IndexOf(Field);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  case State of
    dsCalcFields, dsInternalCalc:
      FieldRow := PPDataRecord(CalcBuffer)^^.Row[FieldOffset];
    dsFilter:
      FieldRow := PPDataRecord(FFilterBuffer)^^.Row[FieldOffset];
    else
      FieldRow := PPDataRecord(ActiveBuffer)^^.Row[FieldOffset];
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  if not (State in [dsCalcFields, dsInternalCalc]) then
    FieldRow := PPDataRecord(ActiveBuffer)^^.Row[FieldOffset]
  else
    FieldRow := PPDataRecord(CalcBuffer)^^.Row[FieldOffset];
>>>>>>> graemeg/cpstrnew

=======
  FieldRow: PChar;
begin
  FieldRow := PPDataRecord(ActiveBuffer)^^.Row[Field.FieldNo - 1];
>>>>>>> graemeg/fixes_2_2
=======
  FieldRow: PChar;
begin
  FieldRow := PPDataRecord(ActiveBuffer)^^.Row[Field.FieldNo - 1];
>>>>>>> origin/fixes_2_2
  Result := FieldRow <> nil;  
  if Result and (Buffer <> nil) then //supports GetIsNull
  begin
    case Field.Datatype of
    ftString:
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        Move(FieldRow^, PAnsiChar(Buffer)^, StrBufSize(FieldRow));
=======
        Move(FieldRow^, PChar(Buffer)^, StrLen(FieldRow) + 1);
>>>>>>> graemeg/fixes_2_2
=======
        Move(FieldRow^, PChar(Buffer)^, StrLen(FieldRow) + 1);
>>>>>>> origin/fixes_2_2
      end;
    ftInteger, ftAutoInc:
      begin
        Val(String(FieldRow), LongInt(Buffer^), ValError);
        Result := ValError = 0;  
      end;
    ftBoolean, ftWord:
      begin
        Val(String(FieldRow), Word(Buffer^), ValError);
        Result := ValError = 0;
      end;    
    ftFloat, ftDateTime, ftTime, ftDate, ftCurrency:
      begin
        Val(String(FieldRow), Double(Buffer^), ValError);
        Result := ValError = 0; 
      end;
    ftLargeInt:
      begin
        Val(String(FieldRow), Int64(Buffer^), ValError);
<<<<<<< HEAD
<<<<<<< HEAD
        Result := ValError = 0;
=======
=======
>>>>>>> origin/fixes_2_2
        Result:= ValError = 0;
>>>>>>> graemeg/fixes_2_2
      end;        
    end;
  end;        
end;

function TCustomSqliteDataset.GetFieldData(Field: TField; Buffer: Pointer): Boolean;
begin
  Result := GetFieldData(Field, Buffer, False);
end;

function TCustomSqliteDataset.GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
var
  Acceptable: Boolean;
  SaveState: TDataSetState;
begin
  Result := grOk;
  repeat
    Acceptable := True;
    case GetMode of
      gmPrior:
        if (FCurrentItem^.Previous = FBeginItem) or (FCurrentItem = FBeginItem) then
          Result := grBOF
        else
          FCurrentItem:=FCurrentItem^.Previous;
      gmCurrent:
        if (FCurrentItem = FBeginItem) or (FCurrentItem = FEndItem) then
           Result := grError;
      gmNext:
        if (FCurrentItem = FEndItem) or (FCurrentItem^.Next = FEndItem) then
          Result := grEOF
        else
          FCurrentItem := FCurrentItem^.Next;
    end; //case
    if Result = grOk then
    begin
      PDataRecord(Pointer(Buffer)^) := FCurrentItem;
      FCurrentItem^.BookmarkFlag := bfCurrent;
      GetCalcFields(Buffer);
      if Filtered then
      begin
        FFilterBuffer := Buffer;
        SaveState := SetTempState(dsFilter);
        DoFilterRecord(Acceptable);
        if (GetMode = gmCurrent) and not Acceptable then
          Result := grError;
        RestoreState(SaveState);
      end;
    end
      else if (Result = grError) and DoCheck then
        DatabaseError('No records found', Self);
  until (Result <> grOK) or Acceptable;
end;

function TCustomSqliteDataset.GetRecordCount: Integer;
begin
  Result := FRecordCount;
end;

function TCustomSqliteDataset.GetRecNo: Integer;
var
  RunItem, ActiveItem: PDataRecord;
begin
  Result := 0;
  if (FRecordCount = 0) or (State = dsInsert) then
    Exit;  
  RunItem := FBeginItem;
  ActiveItem := PPDataRecord(ActiveBuffer)^;
  while ActiveItem <> RunItem do
  begin
    if RunItem^.Next <> nil then
    begin
      Inc(Result);
      RunItem := RunItem^.Next;
    end  
    else
    begin
      Result := 0;
      DatabaseError('GetRecNo - ActiveItem Not Found', Self);
    end;      
  end;  
end;

function TCustomSqliteDataset.GetRecordSize: Word;
begin
  Result := SizeOf(PPDataRecord); //??
end;

procedure TCustomSqliteDataset.InternalAddRecord(Buffer: Pointer; DoAppend: Boolean);
var
  NewItem, ActiveItem: PDataRecord;
  i: Integer;
begin
  {$ifdef DEBUG_SQLITEDS}
  if PPDataRecord(ActiveBuffer)^ <> FCacheItem then
<<<<<<< HEAD
<<<<<<< HEAD
    DatabaseError('PPDataRecord(ActiveBuffer) <> FCacheItem - Problem', Self);
=======
    DatabaseError('PPDataRecord(ActiveBuffer) <> FCacheItem - Problem',Self);
>>>>>>> graemeg/fixes_2_2
=======
    DatabaseError('PPDataRecord(ActiveBuffer) <> FCacheItem - Problem',Self);
>>>>>>> origin/fixes_2_2
  {$endif}
  ActiveItem := PPDataRecord(Buffer)^; 
  New(NewItem);
<<<<<<< HEAD
  GetMem(NewItem^.Row, FRowBufferSize);
=======
  GetMem(NewItem^.Row,FRowBufferSize);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  //if is a detail dataset then set the index value
  if FMasterLink.Active then
    SetMasterIndexValue;
  //necessary to nullify the Row before copy the cache
  for i := 0 to FRowCount - 1 do
    NewItem^.Row[i] := StrBufNew(ActiveItem^.Row[i]);
  NewItem^.BookmarkFlag := bfCurrent;

  //insert in the linked list
  FInsertBookmark^.Previous^.Next := NewItem;
  NewItem^.Next := FInsertBookmark;
  NewItem^.Previous := FInsertBookmark^.Previous;
  FInsertBookmark^.Previous := NewItem;
  
  //update the cursor
  FCurrentItem := NewItem;
  
  Inc(FRecordCount);
  if FAutoIncFieldNo <> - 1 then
    Inc(FNextAutoInc);
  FAddedItems.Add(NewItem);
end;

procedure TCustomSqliteDataset.InternalClose;
begin
  //BindFields(False);
  if DefaultFields then
    DestroyFields;
  if FDataAllocated then
    DisposeLinkedList;  
  FAddedItems.Clear;
  FUpdatedItems.Clear;
  FDeletedItems.Clear;
<<<<<<< HEAD
<<<<<<< HEAD
  FRecordCount := 0;
=======
=======
>>>>>>> origin/fixes_2_2
  FRecordCount:=0;
>>>>>>> graemeg/fixes_2_2
end;

procedure TCustomSqliteDataset.InternalCancel;
var
  i: Integer;
  ActiveItem: PDataRecord;
begin
  ActiveItem := PPDataRecord(ActiveBuffer)^;
  //copy pristine data to active record
  for i:= 0 to FRowCount - 1 do
  begin
    StrDispose(ActiveItem^.Row[i]);
    ActiveItem^.Row[i] := FSavedEditItem^.Row[i];
    FSavedEditItem^.Row[i] := nil;
  end;
end;

procedure TCustomSqliteDataset.InternalDelete;
var
  TempItem: PDataRecord;
  ValError, TempInteger: Integer;
begin
  Dec(FRecordCount);
  TempItem := PPDataRecord(ActiveBuffer)^;
  TempItem^.Next^.Previous := TempItem^.Previous;
  TempItem^.Previous^.Next := TempItem^.Next;
  if FCurrentItem = TempItem then
  begin
<<<<<<< HEAD
    if FCurrentItem^.Next <> FEndItem then
      FCurrentItem := FCurrentItem^.Next
    else
      FCurrentItem := FCurrentItem^.Previous;  
=======
    if FCurrentItem^.Previous <> FBeginItem then
      FCurrentItem := FCurrentItem^.Previous
    else
      FCurrentItem := FCurrentItem^.Next;  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end; 
  // Dec FNextAutoInc (only if deleted item is the last record)  
  if FAutoIncFieldNo <> -1 then
  begin
    Val(String(TempItem^.Row[FAutoIncFieldNo]), TempInteger, ValError);
    if (ValError = 0) and (TempInteger = (FNextAutoInc - 1)) then
      Dec(FNextAutoInc);
  end;    
  // Update item lists
  FUpdatedItems.Remove(TempItem);
  if FAddedItems.Remove(TempItem) = -1 then
    FDeletedItems.Add(TempItem)
  else
    FreeItem(TempItem);
end;

procedure TCustomSqliteDataset.InternalEdit;
var
  i: Integer;
  ActiveItem: PDataRecord;
begin
  ActiveItem := PPDataRecord(ActiveBuffer)^;
  //copy active item to pristine
  for i:= 0 to FRowCount - 1 do
  begin
    StrDispose(FSavedEditItem^.Row[i]);
    FSavedEditItem^.Row[i] := StrBufNew(ActiveItem^.Row[i]);
  end;
end;

procedure TCustomSqliteDataset.InternalFirst;
begin
  FCurrentItem := FBeginItem;
end;

procedure TCustomSqliteDataset.InternalGotoBookmark(ABookmark: Pointer);
begin
  FCurrentItem := PDataRecord(ABookmark^);
end;

procedure TCustomSqliteDataset.InternalInitFieldDefs;
begin
  if FSQL = '' then
  begin
    if FTablename = '' then
      DatabaseError('Tablename not set', Self);
    FEffectiveSQL := 'Select * from ' + FTableName + ';';
  end
  else
    FEffectiveSQL := FSQL;

  if FSqliteHandle = nil then
    GetSqliteHandle;

  RetrieveFieldDefs;
end;

procedure TCustomSqliteDataset.InternalInitRecord(Buffer: TRecordBuffer);
var
  TempStr: String;
begin
  if FAutoIncFieldNo <> - 1 then
  begin
    Str(FNextAutoInc, TempStr);
    StrDispose(FBeginItem^.Row[FAutoIncFieldNo]);
    FBeginItem^.Row[FAutoIncFieldNo] := StrAlloc(Length(TempStr) + 1);
    StrPCopy(FBeginItem^.Row[FAutoIncFieldNo], TempStr);
  end;  
  //todo: see if use bfInserted or bfCurrent
  PPDataRecord(Buffer)^ := FBeginItem;
  FBeginItem^.BookmarkFlag := bfInserted;
end;

procedure TCustomSqliteDataset.InternalLast;
begin
  FCurrentItem := FEndItem;
end;

procedure TCustomSqliteDataset.InternalOpen;
begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  if FMasterLink.DataSource <> nil then
  begin
    //todo: retrieve only necessary fields
    FSql := 'Select * from '+FTableName+';'; // forced to obtain all fields
  end;

  if FSql = '' then
  begin
    if FTablename = '' then
      DatabaseError('Tablename not set',Self);
    FSql := 'Select * from '+FTableName+';';
  end;

  if FSqliteHandle = nil then
    GetSqliteHandle;
    
>>>>>>> graemeg/fixes_2_2
  InternalInitFieldDefs;

  if DefaultFields then 
    CreateFields;
  BindFields(True);

  if CalcFieldsSize > 0 then
    UpdateCalcFieldList;

  if FIndexFieldNames <> '' then
    UpdateIndexFieldList;

  if FMasterLink.DataSource <> nil then
    UpdateMasterDetailProperties;

  // Get PrimaryKeyNo if available
  if TDefCollection(FieldDefs).Find(FPrimaryKey) <> nil then
    FPrimaryKeyNo := FieldDefs.Find(FPrimaryKey).FieldNo - 1
  else
    FPrimaryKeyNo := FAutoIncFieldNo; // -1 if there's no AutoIncField

  BuildLinkedList;               
  FCurrentItem := FBeginItem;
end;

procedure TCustomSqliteDataset.InternalPost;
var
  ActiveItem: PDataRecord;
begin
  inherited InternalPost;

  if State <> dsEdit then
<<<<<<< HEAD
<<<<<<< HEAD
    InternalAddRecord(ActiveBuffer, True)
=======
    InternalAddRecord(nil, True)
>>>>>>> graemeg/fixes_2_2
=======
    InternalAddRecord(nil, True)
>>>>>>> origin/fixes_2_2
  else
  begin
    ActiveItem := PPDataRecord(ActiveBuffer)^;
    if (FUpdatedItems.IndexOf(ActiveItem) = -1) and
      (FAddedItems.IndexOf(ActiveItem) = -1) then
      FUpdatedItems.Add(ActiveItem);
  end;
end;

procedure TCustomSqliteDataset.InternalSetToRecord(Buffer: TRecordBuffer);
begin
  FCurrentItem := PPDataRecord(Buffer)^;
end;

function TCustomSqliteDataset.IsCursorOpen: Boolean;
begin
   Result := FDataAllocated;
end;

type
<<<<<<< HEAD
<<<<<<< HEAD
  TLocateCompareFunction = function (Value: PAnsiChar; const Key: String): Boolean;
=======
  TLocateCompareFunction = function (Value: PChar; const Key: String): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
  TLocateCompareFunction = function (Value: PChar; const Key: String): Boolean;
>>>>>>> origin/fixes_2_2
  
  TLocateFieldInfo = record
    Index: Integer;
    Key: String;
    CompFunction: TLocateCompareFunction;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
function CompInsensitivePartial(UTF8Value: PAnsiChar; const AnsiKey: String): Boolean;
var
  AnsiValue: AnsiString;
begin
  //see comments of CompInsensitive and CompInsensitiveWild functions
  if UTF8Value <> nil then
  begin
    AnsiValue := UTF8Decode(UTF8Value);
    Result := AnsiStrLIComp(PAnsiChar(AnsiValue), PAnsiChar(AnsiKey), Length(AnsiKey)) = 0;
  end
=======
function CompInsensitivePartial(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrLIComp(Value, PChar(Key), Length(Key)) = 0
>>>>>>> graemeg/fixes_2_2
  else
    Result := False;
end;

<<<<<<< HEAD
function CompSensitivePartial(UTF8Value: PAnsiChar; const UTF8Key: String): Boolean;
begin
  if UTF8Value <> nil then
    Result := StrLComp(UTF8Value, PAnsiChar(UTF8Key), Length(UTF8Key)) = 0
=======
function CompSensitivePartial(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrLComp(Value, PChar(Key), Length(Key)) = 0
>>>>>>> graemeg/fixes_2_2
  else
    Result := False;
end;

<<<<<<< HEAD
function CompInsensitive(UTF8Value: PAnsiChar; const AnsiKey: String): Boolean;
begin
  //fpc does not provide a function to compare UTF8 directly, so convert the
  //UTF8Value string to ansi through a temporary widestring and compare with the
  //AnsiKey (already encoded in the system ansi encoding).
  //In unix systems where UTF8 is the system ansi encoding this would not be
  //necessary but there's no direct way to check that
  //todo: change this code when fpc has better support for unicode
  if UTF8Value <> nil then
    Result := AnsiCompareText(UTF8Decode(UTF8Value), AnsiKey) = 0
=======
function CompInsensitive(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrIComp(Value, PChar(Key)) = 0
>>>>>>> graemeg/fixes_2_2
  else
    Result := False;
end;

<<<<<<< HEAD
function CompSensitive(UTF8Value: PAnsiChar; const UTF8Key: String): Boolean;
begin
  if UTF8Value <> nil then
    Result := StrComp(UTF8Value, PAnsiChar(UTF8Key)) = 0
=======
function CompSensitive(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrComp(Value, PChar(Key)) = 0
>>>>>>> graemeg/fixes_2_2
  else
    Result := False;
end;

<<<<<<< HEAD
function CompSensitiveWild(UTF8Value: PAnsiChar; const UTF8Key: String): Boolean;
begin
  if UTF8Value <> nil then
    Result := IsWild(String(UTF8Value), UTF8Key, False)
=======
function CompSensitiveWild(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := IsWild(String(Value), Key, False)
>>>>>>> graemeg/fixes_2_2
  else
    Result := False;
end;

<<<<<<< HEAD
function CompDouble(UTF8Value: PAnsiChar; const UTF8Key: String): Boolean;
var e1,e2:double;
begin
  if UTF8Value <> nil then
    begin
      val(UTF8Value,e1);
      val(UTF8Key,e2);
      result:=e1=e2;
    end
  else
    Result := False;
end;

function CompInsensitiveWild(UTF8Value: PAnsiChar; const AnsiKey: String): Boolean;
begin
  //IsWild does not work with UTF8 encoded strings for case insensitive searches,
  //so convert UTF8Value to the system ansi encoding before passing to IsWild.
  //AnsiKey is already encoded in ansi
  //todo: change this code when fpc has better support for unicode
  if UTF8Value <> nil then
    Result := IsWild(UTF8Decode(UTF8Value), AnsiKey, True)
=======
function CompInsensitiveWild(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := IsWild(String(Value), Key, True)
>>>>>>> graemeg/fixes_2_2
  else
    Result := False;
end;


<<<<<<< HEAD
function TCustomSqliteDataset.FindRecordItem(StartItem: PDataRecord; const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions; DoResync:Boolean): PDataRecord;
=======
function TCustomSqliteDataset.FindRecordItem(StartItem: PDataRecord; const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions; DoResync:Boolean):PDataRecord;
>>>>>>> graemeg/fixes_2_2
var
=======
function CompInsensitivePartial(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrLIComp(Value, PChar(Key), Length(Key)) = 0
  else
    Result := False;
end;

function CompSensitivePartial(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrLComp(Value, PChar(Key), Length(Key)) = 0
  else
    Result := False;
end;

function CompInsensitive(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrIComp(Value, PChar(Key)) = 0
  else
    Result := False;
end;

function CompSensitive(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := StrComp(Value, PChar(Key)) = 0
  else
    Result := False;
end;

function CompSensitiveWild(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := IsWild(String(Value), Key, False)
  else
    Result := False;
end;

function CompInsensitiveWild(Value: PChar; const Key: String): Boolean;
begin
  if Value <> nil then
    Result := IsWild(String(Value), Key, True)
  else
    Result := False;
end;


function TCustomSqliteDataset.FindRecordItem(StartItem: PDataRecord; const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions; DoResync:Boolean):PDataRecord;
var
>>>>>>> origin/fixes_2_2
  LocateFields: array of TLocateFieldInfo;
  AFieldList: TList;
  i, AFieldCount: Integer;
  MatchRecord: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  AValue: String;
>>>>>>> graemeg/fixes_2_2
=======
  AValue: String;
>>>>>>> origin/fixes_2_2
  TempItem: PDataRecord;
  
begin
  Result := nil;
  AFieldList := TList.Create;
  try
    GetFieldList(AFieldList, KeyFields);
    AFieldCount := AFieldList.Count;
    if AFieldCount > 1 then
    begin
      if VarIsArray(KeyValues) then
      begin
        if Succ(VarArrayHighBound(KeyValues, 1)) <> AFieldCount then
          DatabaseError('Number of fields does not correspond to number of values', Self);
      end
      else
        DatabaseError('Wrong number of values specified: expected an array of variants got a variant', Self);
    end;
    
    //set the array of the fields info
    SetLength(LocateFields, AFieldCount);
    
    for i := 0 to AFieldCount - 1 do
      with TField(AFieldList[i]) do
      begin
        if not (DataType in [ftFloat, ftDateTime, ftTime, ftDate]) then
<<<<<<< HEAD
        begin
          //the loPartialKey and loCaseInsensitive is ignored in numeric fields
          if DataType in [ftString, ftMemo] then
          begin
            if loPartialKey in LocateOptions then
            begin
              if loCaseInsensitive in LocateOptions then
                LocateFields[i].CompFunction := @CompInsensitivePartial
              else
                LocateFields[i].CompFunction := @CompSensitivePartial;
            end
            else
            if soWildcardKey in FOptions then
            begin
              if loCaseInsensitive in LocateOptions then
                LocateFields[i].CompFunction := @CompInsensitiveWild
              else
                LocateFields[i].CompFunction := @CompSensitiveWild;
            end
            else
            begin
              if loCaseInsensitive in LocateOptions then
                LocateFields[i].CompFunction := @CompInsensitive
              else
                LocateFields[i].CompFunction := @CompSensitive;
            end;
          end
          else
            LocateFields[i].CompFunction := @CompSensitive;
            
          if VarIsArray(KeyValues) then
<<<<<<< HEAD
            LocateFields[i].Key := VarToStr(KeyValues[i])
          else
            LocateFields[i].Key := VarToStr(KeyValues);
          //store Key encoded as the system ansi encoding
          if loCaseInsensitive in LocateOptions then
            LocateFields[i].Key := UTF8Decode(LocateFields[i].Key);
        end
        else
        begin
          LocateFields[i].CompFunction := @CompDouble;
          //get float types in appropriate format
          if VarIsArray(KeyValues) then
            Str(VarToDateTime(keyvalues[i]), LocateFields[i].Key)
          else
            Str(VarToDateTime(keyvalues), LocateFields[i].Key);
=======
=======
        begin
          //the loPartialKey and loCaseInsensitive is ignored in numeric fields
          if DataType in [ftString, ftMemo] then
          begin
            if loPartialKey in LocateOptions then
            begin
              if loCaseInsensitive in LocateOptions then
                LocateFields[i].CompFunction := @CompInsensitivePartial
              else
                LocateFields[i].CompFunction := @CompSensitivePartial;
            end
            else
            if soWildcardKey in FOptions then
            begin
              if loCaseInsensitive in LocateOptions then
                LocateFields[i].CompFunction := @CompInsensitiveWild
              else
                LocateFields[i].CompFunction := @CompSensitiveWild;
            end
            else
            begin
              if loCaseInsensitive in LocateOptions then
                LocateFields[i].CompFunction := @CompInsensitive
              else
                LocateFields[i].CompFunction := @CompSensitive;
            end;
          end
          else
            LocateFields[i].CompFunction := @CompSensitive;
            
          if VarIsArray(KeyValues) then
>>>>>>> origin/fixes_2_2
            LocateFields[i].Key := KeyValues[i]
          else
            LocateFields[i].Key := KeyValues;
        end
        else
        begin
          LocateFields[i].CompFunction := @CompSensitive;
          //get float types in appropriate format
          if VarIsArray(KeyValues) then
            Str(VarToDateTime(keyvalues[i]), AValue)
          else
            Str(VarToDateTime(keyvalues), AValue);
          LocateFields[i].Key := Trim(AValue);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
        LocateFields[i].Index := FieldNo - 1;
      end;
  finally
    AFieldList.Destroy;
  end;
  {$ifdef DEBUG_SQLITEDS}
<<<<<<< HEAD
<<<<<<< HEAD
  WriteLn('##TCustomSqliteDataset.FindRecordItem##');
  WriteLn('  KeyFields: ', KeyFields);
  for i := 0 to AFieldCount - 1 do
  begin
    WriteLn('LocateFields[', i, ']');
    WriteLn('  Key: ', LocateFields[i].Key);
    WriteLn('  Index: ', LocateFields[i].Index);
=======
=======
>>>>>>> origin/fixes_2_2
  writeln('##TCustomSqliteDataset.FindRecordItem##');
  writeln('  KeyFields: ', KeyFields);
  for i := 0 to AFieldCount - 1 do
  begin
    writeln('LocateFields[',i,']');
    writeln('  Key: ', LocateFields[i].Key);
    writeln('  Index: ', LocateFields[i].Index);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  {$endif}        
  //Search the list
  TempItem := StartItem;
  while TempItem <> FEndItem do
  begin
    MatchRecord := True;
    for i:= 0 to AFieldCount - 1 do
    begin
      with LocateFields[i] do
      if not CompFunction(TempItem^.Row[Index], Key) then
      begin
        MatchRecord := False;
<<<<<<< HEAD
<<<<<<< HEAD
        break; //for
=======
=======
>>>>>>> origin/fixes_2_2
        Break;//for
>>>>>>> graemeg/fixes_2_2
      end;
    end;
    if MatchRecord then
    begin
      Result := TempItem;
      if DoResync and (TempItem <> PPDataRecord(ActiveBuffer)^) then
      begin
        DoBeforeScroll;
        FCurrentItem := TempItem;
        Resync([]);
        DoAfterScroll;
      end;
      break; //while
    end;
    TempItem := TempItem^.Next;
  end;      
end;

procedure TCustomSqliteDataset.UpdateMasterDetailProperties;
var
  i: Integer;
begin
  if FMasterLink.Active and (FIndexFieldList.Count <> FMasterLink.Fields.Count) then
    DatabaseError('MasterFields count doesn''t match IndexFields count', Self);
  if FieldDefs.Count > 0 then
  begin
    //build the sql template used to filter the dataset
    FSqlFilterTemplate := 'SELECT ';
    for i := 0 to FieldDefs.Count - 2 do
      FSqlFilterTemplate := FSqlFilterTemplate + FieldDefs[i].Name + ',';
    FSqlFilterTemplate := FSqlFilterTemplate + FieldDefs[FieldDefs.Count - 1].Name +
      ' FROM ' + FTableName;
  end;
  //set FEffectiveSQL considering MasterSource active record
  SetDetailFilter;
end;

function TCustomSqliteDataset.FieldDefsStored: Boolean;
begin
  Result := FStoreDefs and (FieldDefs.Count > 0);
end;

procedure TCustomSqliteDataset.GetSqliteHandle;
begin
  if FFileName = '' then
<<<<<<< HEAD
<<<<<<< HEAD
    DatabaseError('Filename not set', Self);
  FSqliteHandle := InternalGetHandle;
  if Assigned(FOnGetHandle) then
    FOnGetHandle(Self);
end;

procedure TCustomSqliteDataset.FreeItem(AItem: PDataRecord);
var
  i: Integer;
begin
  for i:= 0 to FRowCount - 1 do
    StrDispose(AItem^.Row[i]);
  FreeMem(AItem^.Row, FRowBufferSize);
  Dispose(AItem);
end;

function TCustomSqliteDataset.Locate(const KeyFields: String; const KeyValues: Variant; LocateOptions: TLocateOptions): Boolean;
begin
=======
    DatabaseError('Filename not set',Self);
  FSqliteHandle := InternalGetHandle;
  if Assigned(FOnGetHandle) then
    FOnGetHandle(Self);
end;

procedure TCustomSqliteDataset.FreeItem(AItem: PDataRecord);
var
  i: Integer;
begin
  for i:= 0 to FRowCount - 1 do
    StrDispose(AItem^.Row[i]);
  FreeMem(AItem^.Row,FRowBufferSize);
  Dispose(AItem);
end;

function TCustomSqliteDataset.Locate(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
begin
>>>>>>> graemeg/fixes_2_2
=======
    DatabaseError('Filename not set',Self);
  FSqliteHandle := InternalGetHandle;
  if Assigned(FOnGetHandle) then
    FOnGetHandle(Self);
end;

procedure TCustomSqliteDataset.FreeItem(AItem: PDataRecord);
var
  i: Integer;
begin
  for i:= 0 to FRowCount - 1 do
    StrDispose(AItem^.Row[i]);
  FreeMem(AItem^.Row,FRowBufferSize);
  Dispose(AItem);
end;

function TCustomSqliteDataset.Locate(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
begin
>>>>>>> origin/fixes_2_2
  CheckBrowseMode;
  Result := FindRecordItem(FBeginItem^.Next, KeyFields, KeyValues, LocateOptions, True) <> nil;
end;
  
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSqliteDataset.LocateNext(const KeyFields: String; const KeyValues: Variant; LocateOptions: TLocateOptions): Boolean;
=======
function TCustomSqliteDataset.LocateNext(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TCustomSqliteDataset.LocateNext(const KeyFields: string; const KeyValues: Variant; LocateOptions: TLocateOptions) : Boolean;
>>>>>>> origin/fixes_2_2
begin
  CheckBrowseMode;
  Result := FindRecordItem(PPDataRecord(ActiveBuffer)^^.Next, KeyFields, KeyValues, LocateOptions, True) <> nil;
end;
  
function TCustomSqliteDataset.Lookup(const KeyFields: String; const KeyValues: Variant; const ResultFields: String): Variant;
var
  TempItem: PDataRecord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SaveState: TDataSetState;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  SaveState: TDataSetState;
>>>>>>> origin/fixes_2.4
begin
  CheckBrowseMode;
  TempItem := FindRecordItem(FBeginItem^.Next, KeyFields, KeyValues, [], False);
  if TempItem <> nil then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    SaveState := SetTempState(dsInternalCalc);
    try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      CalculateFields(TRecordBuffer(@TempItem));
      Result := FieldValues[ResultFields];
=======
      CalculateFields(PChar(@TempItem));
      Result := FieldByName(ResultFields).Value;
>>>>>>> graemeg/cpstrnew
=======
      CalculateFields(PChar(@TempItem));
      Result := FieldByName(ResultFields).Value;
>>>>>>> graemeg/cpstrnew
=======
      CalculateFields(PChar(@TempItem));
      Result := FieldByName(ResultFields).Value;
>>>>>>> graemeg/cpstrnew
=======
      CalculateFields(PChar(@TempItem));
      Result := FieldByName(ResultFields).Value;
>>>>>>> origin/cpstrnew
=======
  begin
    SaveState := SetTempState(dsInternalCalc);
    try
      CalculateFields(PChar(@TempItem));
      Result := FieldByName(ResultFields).Value;
>>>>>>> origin/fixes_2.4
    finally
      RestoreState(SaveState);
    end;
  end
  else
    Result := Null;
=======
    Result := TempItem^.Row[FieldByName(ResultFields).FieldNo - 1]
  else
    Result := False;
>>>>>>> graemeg/fixes_2_2
=======
    Result := TempItem^.Row[FieldByName(ResultFields).FieldNo - 1]
  else
    Result := False;
>>>>>>> origin/fixes_2_2
end;  

procedure TCustomSqliteDataset.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
begin
  //The BookMarkData is the Buffer itself: no need to set nothing;
end;

procedure TCustomSqliteDataset.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag);
begin
  PPDataRecord(Buffer)^^.BookmarkFlag := Value;
end;

procedure TCustomSqliteDataset.SetExpectedAppends(AValue: Integer);
begin
  FAddedItems.Capacity := AValue;
end;  

procedure TCustomSqliteDataset.SetExpectedUpdates(AValue: Integer);
begin
  FUpdatedItems.Capacity := AValue;
end;  

procedure TCustomSqliteDataset.SetExpectedDeletes(AValue: Integer);
begin
<<<<<<< HEAD
  FDeletedItems.Capacity := AValue;
=======
  FDeletedItems.Capacity:=AValue;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;  

procedure TCustomSqliteDataset.SetFieldData(Field: TField; Buffer: Pointer;
  NativeFormat: Boolean);
var
<<<<<<< HEAD
  TempStr: String;
  FieldOffset: Integer;
  EditItem: PDataRecord;
begin
  if not (State in [dsEdit, dsInsert, dsCalcFields]) then
    DatabaseErrorFmt(SNotEditing, [Name], Self);

  if Field.FieldNo >= 0 then
  begin
    if State in [dsEdit, dsInsert] then
      Field.Validate(Buffer);
    FieldOffset := Field.FieldNo - 1;
    EditItem := PPDataRecord(ActiveBuffer)^;
  end
  else
  begin
    FieldOffset := FieldDefs.Count + FCalcFieldList.IndexOf(Field);
    EditItem := PPDataRecord(CalcBuffer)^;
  end;

  StrDispose(EditItem^.Row[FieldOffset]);
=======
  TempStr:String;
  FloatStr: PChar;
  FloatLen: Integer;
begin
  if not (State in [dsEdit, dsInsert]) then
    DatabaseErrorFmt(SNotEditing,[Name],Self);

  StrDispose(FCacheItem^.Row[Pred(Field.FieldNo)]);
>>>>>>> graemeg/fixes_2_2
  if Buffer <> nil then
  begin
    case Field.Datatype of
    ftString:
      begin            
        EditItem^.Row[FieldOffset] := StrNew(PAnsiChar(Buffer));
      end;
    ftInteger:
      begin          
        Str(LongInt(Buffer^), TempStr);
        EditItem^.Row[FieldOffset] := StrAlloc(Length(TempStr) + 1);
        Move(PAnsiChar(TempStr)^, (EditItem^.Row[FieldOffset])^, Length(TempStr) + 1);
      end;
    ftBoolean, ftWord:
      begin
        //ensure that boolean True value is stored as 1
        if Field.DataType = ftBoolean then
          TempStr := IfThen(Boolean(Buffer^), '1', '0')
        else
          Str(Word(Buffer^), TempStr);
        EditItem^.Row[FieldOffset] := StrAlloc(Length(TempStr) + 1);
        Move(PAnsiChar(TempStr)^, (EditItem^.Row[FieldOffset])^, Length(TempStr) + 1);
      end;  
    ftFloat, ftDateTime, ftDate, ftTime, ftCurrency:
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        Str(Double(Buffer^), TempStr);
        EditItem^.Row[FieldOffset] := StrAlloc(Length(TempStr) + 1);
        Move(PAnsiChar(TempStr)^, (EditItem^.Row[FieldOffset])^, Length(TempStr) + 1);
=======
=======
>>>>>>> origin/fixes_2_2
        Str(Double(Buffer^),TempStr);
        //Str returns a space as the first character for positive values
        //and the - sign for negative values. It's necessary to remove the extra
        //space while keeping the - sign
        if TempStr[1] = ' ' then
        begin
          FloatStr := PChar(TempStr) + 1;
          FloatLen := Length(TempStr);
        end
        else
        begin
          FloatStr := PChar(TempStr);
          FloatLen := Length(TempStr) + 1;
        end;
        FCacheItem^.Row[Pred(Field.FieldNo)] := StrAlloc(FloatLen);
        Move(FloatStr^, (FCacheItem^.Row[Pred(Field.FieldNo)])^, FloatLen);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
    ftLargeInt:
      begin
        Str(Int64(Buffer^), TempStr);
        EditItem^.Row[FieldOffset] := StrAlloc(Length(TempStr) + 1);
        Move(PAnsiChar(TempStr)^, (EditItem^.Row[FieldOffset])^, Length(TempStr) + 1);
      end;        
    end;// case
  end//if
  else
    EditItem^.Row[FieldOffset] := nil;

  if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
    DataEvent(deFieldChange, Ptrint(Field));  
end;

procedure TCustomSqliteDataset.SetFieldData(Field: TField; Buffer: Pointer);
begin
  SetFieldData(Field, Buffer, False);
end;

procedure TCustomSqliteDataset.SetRecNo(Value: Integer);
var
  Counter: Integer;
  TempItem: PDataRecord;
begin
  if (Value > FRecordCount) or (Value <= 0) then
    DatabaseError('Record Number Out Of Range',Self);
  CheckBrowseMode;
  TempItem := FBeginItem;
  for Counter := 1 to Value do
    TempItem := TempItem^.Next;
  if TempItem <> PPDataRecord(ActiveBuffer)^ then
  begin
    DoBeforeScroll;
    FCurrentItem := TempItem;
    Resync([]);
    DoAfterScroll;
  end;
end;

// Specific functions

function GetFieldEqualExpression(AField: TField): String;
begin
  if not AField.IsNull then
  begin
    case AField.DataType of
      //todo: handle " caracter properly
      ftString, ftMemo:
        Result := '"' + AField.AsString + '"';
      ftDateTime, ftDate, ftTime:
        Str(AField.AsDateTime, Result);
    else
      Result := AField.AsString;
    end; //case
    Result := ' = ' + Result;
  end
  else
    Result := ' IS NULL';
end;

procedure TCustomSqliteDataset.SetDetailFilter;
var
  AFilter: String;
  i: Integer;
begin
<<<<<<< HEAD
  if not FMasterLink.Active then //Retrieve all data
=======
  if (FMasterLink.Dataset.RecordCount = 0) or not FMasterLink.Active then //Retrieve all data
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FEffectiveSQL := FSqlFilterTemplate
  else
  begin
    AFilter := ' where ';
    for i := 0 to FMasterLink.Fields.Count - 1 do
    begin
      AFilter := AFilter + IndexFields[i].FieldName + GetFieldEqualExpression(TField(FMasterLink.Fields[i]));
      if i <> FMasterLink.Fields.Count - 1 then
        AFilter := AFilter + ' and ';
    end;
    FEffectiveSQL := FSqlFilterTemplate + AFilter;
  end;
end;

procedure TCustomSqliteDataset.MasterChanged(Sender: TObject);
begin
  SetDetailFilter;
  {$ifdef DEBUG_SQLITEDS}
<<<<<<< HEAD
<<<<<<< HEAD
  WriteLn('##TCustomSqliteDataset.MasterChanged##');
  WriteLn('  SQL used to filter detail dataset:');
  WriteLn('  ', FEffectiveSQL);
=======
=======
>>>>>>> origin/fixes_2_2
  writeln('##TCustomSqliteDataset.MasterChanged##');
  writeln('  SQL used to filter detail dataset:');
  writeln('  ',FSql);
>>>>>>> graemeg/fixes_2_2
  {$endif}
  RefetchData;
end;

procedure TCustomSqliteDataset.SetMasterFields(const Value: String);
begin
<<<<<<< HEAD
  FMasterLink.FieldNames := Value;
=======
  FSql:='Select * from '+FTableName+';'; 
  RefetchData;
end;

procedure TCustomSqliteDataset.SetMasterFields(const Value: String);
begin
  FMasterLink.FieldNames:=Value;
>>>>>>> graemeg/fixes_2_2
  if Active and FMasterLink.Active then
  begin
    UpdateIndexFieldList;
    if (FIndexFieldList.Count <> FMasterLink.Fields.Count) then
      DatabaseError('MasterFields count doesn''t match IndexFields count', Self);
  end;
end;

function TCustomSqliteDataset.GetMasterFields: String;
begin
  Result := FMasterLink.FieldNames;
end;

procedure TCustomSqliteDataset.UpdateIndexFieldList;
begin
  if FIndexFieldList = nil then
    FIndexFieldList := TList.Create
  else
    FIndexFieldList.Clear;

  try
    GetFieldList(FIndexFieldList, FIndexFieldNames);
  except
    on E: Exception do
    begin
      FIndexFieldList.Clear;
      DatabaseError('Error retrieving index fields: ' + E.Message);
    end;
  end;
end;

function TCustomSqliteDataset.GetMasterSource: TDataSource;
begin
  Result := FMasterLink.DataSource;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomSqliteDataset.SetFileName(const Value: UTF8String);
=======
procedure TCustomSqliteDataset.SetFileName(const Value: String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TCustomSqliteDataset.SetFileName(const Value: String);
>>>>>>> origin/fixes_2_2
begin
  if Value <> FFileName then
  begin
    if Active then
      DatabaseError('It''s not allowed to change Filename in an open dataset', Self);
    if FSqliteHandle <> nil then
      InternalCloseHandle;
    FFileName := Value;
  end;
end;

procedure TCustomSqliteDataset.SetMasterSource(Value: TDataSource);
begin
  FMasterLink.DataSource := Value;
end;

procedure TCustomSqliteDataset.ExecSQL(const ASql: String);
begin
  if FSqliteHandle = nil then
    GetSqliteHandle;
  ExecuteDirect(ASQL);
end;

procedure TCustomSqliteDataset.ExecSQL(ASqlList: TStrings);
begin
  if FSqliteHandle = nil then
    GetSqliteHandle;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := SqliteExec(PAnsiChar(ASQLList.Text), nil, nil);
=======
  FReturnCode := SqliteExec(PChar(ASQLList.Text), nil, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := SqliteExec(PChar(ASQLList.Text), nil, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := SqliteExec(PChar(ASQLList.Text), nil, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := SqliteExec(PChar(ASQLList.Text), nil, nil);
>>>>>>> origin/cpstrnew
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);
end;

procedure TCustomSqliteDataset.ExecSQLList;
begin
  ExecSQL(SQLList);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomSqliteDataset.GetSQLValue(Values: PPAnsiChar; FieldIndex: Integer): String;
=======
function TCustomSqliteDataset.GetSQLValue(Values: PPChar; FieldIndex: Integer): String;
>>>>>>> graemeg/cpstrnew
=======
function TCustomSqliteDataset.GetSQLValue(Values: PPChar; FieldIndex: Integer): String;
>>>>>>> graemeg/cpstrnew
=======
function TCustomSqliteDataset.GetSQLValue(Values: PPChar; FieldIndex: Integer): String;
>>>>>>> graemeg/cpstrnew
=======
function TCustomSqliteDataset.GetSQLValue(Values: PPChar; FieldIndex: Integer): String;
>>>>>>> origin/cpstrnew
begin
  if (State = dsInactive) or (FieldIndex < 0) or (FieldIndex >= FieldDefs.Count) then
    DatabaseError('Error retrieving SQL value: dataset inactive or field out of range', Self);
  Result := FGetSqlStr[FieldIndex](Values[FieldIndex]);
end;

procedure TCustomSqliteDataset.ExecSQL;
begin
  ExecSQL(FSQL);
end;

function TCustomSqliteDataset.ApplyUpdates: Boolean;
var
<<<<<<< HEAD
<<<<<<< HEAD
  iFields, iItems, StatementsCounter: Integer;
  SQLTemp, WhereKeyNameEqual, SQLLine, TemplateStr: String;
  TempItem: PDataRecord;
begin
  Result := False;
=======
=======
>>>>>>> origin/fixes_2_2
  iFields, iItems, StatementsCounter, TempReturnCode:Integer;
  SqlTemp,WhereKeyNameEqual,ASqlLine,TemplateStr:String;
  TempItem: PDataRecord;
begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckBrowseMode;
  if not UpdatesPending then
  begin
    Result := True;
    Exit;
  end;
  //A PrimaryKey is only necessary to update or delete records
  if FPrimaryKeyNo <> -1 then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    WhereKeyNameEqual := ' WHERE ' + FieldDefs[FPrimaryKeyNo].Name + ' = ';
    Result := True;
  end else if (FUpdatedItems.Count + FDeletedItems.Count) = 0 then
    Result := True;
  if not Result then
    Exit;

  FReturnCode := SQLITE_OK;
  StatementsCounter := 0;
  SQLTemp := 'BEGIN;';
  {$ifdef DEBUG_SQLITEDS}
  WriteLn('##TCustomSqliteDataset.ApplyUpdates##');
  if FPrimaryKeyNo = FAutoIncFieldNo then
    WriteLn('  Using an AutoInc field as primary key');
  WriteLn('  PrimaryKey: ', WhereKeyNameEqual);
  WriteLn('  PrimaryKeyNo: ', FPrimaryKeyNo);
  {$endif}
  // Delete Records
  if FDeletedItems.Count > 0 then
  begin
    TemplateStr := 'DELETE FROM ' + FTableName + WhereKeyNameEqual;
    for iItems := 0 to FDeletedItems.Count - 1 do
    begin
      TempItem := PDataRecord(FDeletedItems.List^[iItems]);
      SQLTemp := SQLTemp + (TemplateStr +
        FGetSqlStr[FPrimaryKeyNo](TempItem^.Row[FPrimaryKeyNo]) + ';');
      FreeItem(TempItem);
      Inc(StatementsCounter);
      //ApplyUpdates each 400 statements
      if StatementsCounter = 400 then
      begin
        SQLTemp := SQLTemp + 'COMMIT;';
        FReturnCode := SqliteExec(PAnsiChar(SQLTemp), nil, nil);
        StatementsCounter := 0;
        SQLTemp := 'BEGIN;';
        if FReturnCode <> SQLITE_OK then
        begin
          SqliteExec('ROLLBACK;', nil, nil);
          Break;
        end;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
    FReturnCode := SQLITE_OK;
    StatementsCounter:=0;
    WhereKeyNameEqual:=' WHERE '+Fields[FPrimaryKeyNo].FieldName+' = ';
    {$ifdef DEBUG_SQLITEDS}
    WriteLn('##TCustomSqliteDataset.ApplyUpdates##');
    if FPrimaryKeyNo = FAutoIncFieldNo then
      WriteLn('  Using an AutoInc field as primary key');
    WriteLn('  PrimaryKey: ',WhereKeyNameEqual);
    WriteLn('  PrimaryKeyNo: ',FPrimaryKeyNo);
    {$endif}
    SqlTemp:='BEGIN;';
    // Delete Records
    if FDeletedItems.Count > 0 then
      TemplateStr:='DELETE FROM '+FTableName+WhereKeyNameEqual;
    for iItems:= 0 to FDeletedItems.Count - 1 do
    begin
      TempItem:=PDataRecord(FDeletedItems.List^[iItems]);
      SqlTemp:=SqlTemp+(TemplateStr+
        String(TempItem^.Row[FPrimaryKeyNo])+';');
      FreeItem(TempItem);
      inc(StatementsCounter);
      //ApplyUpdates each 400 statements
      if StatementsCounter = 400 then
      begin
        SqlTemp:=SqlTemp+'COMMIT;';
        TempReturnCode := SqliteExec(PChar(SqlTemp),nil,nil);
        if TempReturnCode <> SQLITE_OK then
          FReturnCode := TempReturnCode;  
        StatementsCounter:=0;
        SqlTemp:='BEGIN;';
      end;    
>>>>>>> graemeg/fixes_2_2
    end;
  end;
  // Update changed records
  if (FUpdatedItems.Count > 0) and (FReturnCode = SQLITE_OK) then
  begin
    TemplateStr := 'UPDATE ' + FTableName + ' SET ';
    for iItems := 0 to FUpdatedItems.Count - 1 do
    begin
      SQLLine := TemplateStr;
      for iFields := 0 to FieldDefs.Count - 2 do
      begin
        SQLLine := SQLLine + (FieldDefs[iFields].Name + ' = ' +
          FGetSqlStr[iFields](PDataRecord(FUpdatedItems[iItems])^.Row[iFields]) + ',');
      end;
<<<<<<< HEAD
      iFields := FieldDefs.Count - 1;
      SQLLine := SQLLine + (FieldDefs[iFields].Name + ' = ' +
        FGetSqlStr[iFields](PDataRecord(FUpdatedItems[iItems])^.Row[iFields]) +
        WhereKeyNameEqual +
        FGetSqlStr[FPrimaryKeyNo](PDataRecord(FUpdatedItems[iItems])^.Row[FPrimaryKeyNo]) + ';');
      SQLTemp := SQLTemp + SQLLine;
      Inc(StatementsCounter);
      //ApplyUpdates each 400 statements
      if StatementsCounter = 400 then
      begin
        SQLTemp := SQLTemp + 'COMMIT;';
        FReturnCode := SqliteExec(PAnsiChar(SQLTemp), nil, nil);
        StatementsCounter := 0;
        SQLTemp := 'BEGIN;';
        if FReturnCode <> SQLITE_OK then
        begin
          SqliteExec('ROLLBACK;', nil, nil);
          Break;
        end;
      end;
=======
      iFields:=Fields.Count - 1;
      ASqlLine:=ASqlLine + (Fields[iFields].FieldName +' = '+
        FGetSqlStr[iFields](PDataRecord(FUpdatedItems[iItems])^.Row[iFields])+
        WhereKeyNameEqual+String(PDataRecord(FUpdatedItems[iItems])^.Row[FPrimaryKeyNo])+';');
      SqlTemp:=SqlTemp + ASqlLine;
      inc(StatementsCounter);
      //ApplyUpdates each 400 statements
      if StatementsCounter = 400 then
      begin
        SqlTemp:=SqlTemp+'COMMIT;';
        TempReturnCode := SqliteExec(PChar(SqlTemp),nil,nil);
        if TempReturnCode <> SQLITE_OK then
          FReturnCode := TempReturnCode;
        StatementsCounter:=0;
        SqlTemp:='BEGIN;';
      end;  
>>>>>>> graemeg/fixes_2_2
    end;
  end;
  // Add new records
  if (FAddedItems.Count > 0) and (FReturnCode = SQLITE_OK) then
  begin
    // Build TemplateStr
    TemplateStr := 'INSERT INTO ' + FTableName + ' (';
    for iFields := 0 to FieldDefs.Count - 2 do
      TemplateStr := TemplateStr + FieldDefs[iFields].Name + ',';
    TemplateStr := TemplateStr + FieldDefs[FieldDefs.Count - 1].Name + ') VALUES (';
    for iItems := 0 to FAddedItems.Count - 1 do
    begin
<<<<<<< HEAD
      SQLLine := TemplateStr;
      for iFields := 0 to FieldDefs.Count - 2 do
        SQLLine := SQLLine + (FGetSqlStr[iFields](PDataRecord(FAddedItems[iItems])^.Row[iFields]) + ',');
      iFields := FieldDefs.Count - 1;
      SQLLine := SQLLine + (FGetSqlStr[iFields](PDataRecord(FAddedItems[iItems])^.Row[iFields]) + ');' );
      SQLTemp := SQLTemp + SQLLine;
      Inc(StatementsCounter);
      //ApplyUpdates each 400 statements
      if StatementsCounter = 400 then
      begin
        SQLTemp := SQLTemp + 'COMMIT;';
        FReturnCode := SqliteExec(PAnsiChar(SQLTemp), nil, nil);
        StatementsCounter := 0;
        SQLTemp := 'BEGIN;';
        if FReturnCode <> SQLITE_OK then
        begin
          SqliteExec('ROLLBACK;', nil, nil);
          Break;
        end;
      end;
    end;
  end;
  FAddedItems.Clear;
  FUpdatedItems.Clear;
  FDeletedItems.Clear;
  if FReturnCode = SQLITE_OK then
  begin
    SQLTemp := SQLTemp + 'COMMIT;';
    FReturnCode := SqliteExec(PAnsiChar(SQLTemp), nil, nil);
    if FReturnCode <> SQLITE_OK then
      SqliteExec('ROLLBACK;', nil, nil);
  end;
  Result := FReturnCode = SQLITE_OK;
  {$ifdef DEBUG_SQLITEDS}
  WriteLn('  Result: ', Result);
=======
      TemplateStr:='INSERT INTO '+FTableName+ ' (';
      for iFields:= 0 to Fields.Count - 2 do
        TemplateStr:=TemplateStr + Fields[iFields].FieldName+',';
      TemplateStr:= TemplateStr+Fields[Fields.Count - 1].FieldName+') VALUES (';
    end;  
    for iItems:= 0 to FAddedItems.Count - 1 do
    begin
      ASqlLine:=TemplateStr;
      for iFields:= 0 to Fields.Count - 2 do
      begin
        ASqlLine:=ASqlLine + (FGetSqlStr[iFields](PDataRecord(FAddedItems[iItems])^.Row[iFields])+',');
      end;
      //todo: see if i can assume iFields = Fields.Count-2 safely
      iFields:=Fields.Count - 1;
      ASqlLine:=ASqlLine + (FGetSqlStr[iFields](PDataRecord(FAddedItems[iItems])^.Row[iFields])+');');
      SqlTemp:=SqlTemp + ASqlLine;    
      inc(StatementsCounter);
      //ApplyUpdates each 400 statements
      if StatementsCounter = 400 then
      begin
        SqlTemp:=SqlTemp+'COMMIT;';
        TempReturnCode := SqliteExec(PChar(SqlTemp),nil,nil);
        if TempReturnCode <> SQLITE_OK then
          FReturnCode := TempReturnCode;
        StatementsCounter:=0;
        SqlTemp:='BEGIN;';
      end;  
    end;  
    SqlTemp:=SqlTemp+'COMMIT;';
    {$ifdef DEBUG_SQLITEDS}
    writeln('  SQL: ',SqlTemp);
    {$endif}  
   FAddedItems.Clear;
   FUpdatedItems.Clear;
   FDeletedItems.Clear;   
   TempReturnCode := SqliteExec(PChar(SqlTemp),nil,nil);
   if TempReturnCode <> SQLITE_OK then
     FReturnCode := TempReturnCode;
   Result:= FReturnCode = SQLITE_OK;
  end;  
  {$ifdef DEBUG_SQLITEDS}
    writeln('  Result: ',Result);
>>>>>>> graemeg/fixes_2_2
  {$endif}   
end;

procedure TCustomSqliteDataset.ClearUpdates(RecordStates: TRecordStateSet);
begin
  if rsUpdated in RecordStates then
    FUpdatedItems.Clear;
  if rsDeleted in RecordStates then
    FDeletedItems.Clear;
  if rsAdded in RecordStates then
    FAddedItems.Clear;
end;

function TCustomSqliteDataset.CreateTable: Boolean;
begin
  Result := CreateTable(FTableName);
end;

function TCustomSqliteDataset.CreateTable(const ATableName: String): Boolean;
var
  SQLTemp: String;
  i, StrSize: Integer;
begin
  {$ifdef DEBUG_SQLITEDS}
<<<<<<< HEAD
<<<<<<< HEAD
  WriteLn('##TCustomSqliteDataset.CreateTable##');
=======
=======
>>>>>>> origin/fixes_2_2
  writeln('##TCustomSqliteDataset.CreateTable##');
>>>>>>> graemeg/fixes_2_2
  if ATableName = '' then
    WriteLn('  TableName Not Set');
  if FieldDefs.Count = 0 then
    WriteLn('  FieldDefs Not Initialized');
  {$endif}
  if (ATableName <> '') and (FieldDefs.Count > 0) then
  begin
    SQLTemp := 'CREATE TABLE ' + ATableName + ' (';
    for i := 0 to FieldDefs.Count - 1 do
    begin
      //todo: add index to autoinc field
      SQLTemp := SQLTemp + FieldDefs[i].Name;
      case FieldDefs[i].DataType of
        ftInteger:
          SQLTemp := SQLTemp + ' INTEGER';
        ftString:
        begin
          StrSize := FieldDefs[i].Size;
          if StrSize = 0 then
            StrSize := DefaultStringSize;
          SQLTemp := SQLTemp + ' VARCHAR(' + IntToStr(StrSize) + ')';
        end;
        ftBoolean:
          SQLTemp := SQLTemp + ' BOOL_INT';
        ftFloat:
          SQLTemp := SQLTemp + ' FLOAT';
        ftWord:
          SQLTemp := SQLTemp + ' WORD';
        ftDateTime:
          SQLTemp := SQLTemp + ' DATETIME';
        ftDate:
          SQLTemp := SQLTemp + ' DATE';
        ftTime:
          SQLTemp := SQLTemp + ' TIME';
        ftLargeInt:
          SQLTemp := SQLTemp + ' LARGEINT';
        ftCurrency:
          SQLTemp := SQLTemp + ' CURRENCY';
        ftAutoInc:
          SQLTemp := SQLTemp + ' AUTOINC_INT';
        ftMemo:
          SQLTemp := SQLTemp + ' TEXT';
      else
        DatabaseError('Field type "' + FieldTypeNames[FieldDefs[i].DataType] +
          '" not supported', Self);
      end;
      if UpperCase(FieldDefs[i].Name) = UpperCase(FPrimaryKey) then
        SQLTemp := SQLTemp + ' PRIMARY KEY';
      if i <> FieldDefs.Count - 1 then
        SQLTemp := SQLTemp + ' , ';
    end;
<<<<<<< HEAD
    SQLTemp := SQLTemp + ');';
    {$ifdef DEBUG_SQLITEDS}
    WriteLn('  SQL: ',SqlTemp);
=======
    SqlTemp:=SqlTemp+');';
    {$ifdef DEBUG_SQLITEDS}
    writeln('  SQL: ',SqlTemp);
>>>>>>> graemeg/fixes_2_2
    {$endif}
    ExecSQL(SQLTemp);
    Result := FReturnCode = SQLITE_DONE;
  end
  else
    Result := False;
end;

procedure TCustomSqliteDataset.ExecCallback(const ASql: String;
  UserData: Pointer);
var
  CallbackInfo: TCallbackInfo;
begin
  if not Assigned(FOnCallback) then
    DatabaseError('OnCallback property not set', Self);
  if FSqliteHandle = nil then
    GetSqliteHandle;
  CallbackInfo.Data := UserData;
  CallbackInfo.Proc := FOnCallback;
  SqliteExec(PAnsiChar(ASQL), @CallbackDispatcher, @CallbackInfo);
end;


procedure TCustomSqliteDataset.QueryUpdates(RecordStates: TRecordStateSet; Callback: TQueryUpdatesCallback;
  UserData: Pointer = nil);
var
  i: Integer;
  TempItem: PDataRecord;
begin
  if not Assigned(Callback) then
<<<<<<< HEAD
    DatabaseError('Callback parameter not set', Self);
=======
    DatabaseError('Callback parameter not set',Self);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckBrowseMode;
  if rsDeleted in RecordStates then
    with FDeletedItems do
    for i := 0 to Count - 1 do
      Callback(UserData,PDataRecord(Items[i])^.Row, nil, rsDeleted);
  if rsUpdated in RecordStates then
    with FUpdatedItems do
    for i := 0 to Count - 1 do
    begin
      TempItem := PDataRecord(Items[i]);
      Callback(UserData, TempItem^.Row, TBookmark(@TempItem), rsUpdated);
    end;
  if rsAdded in RecordStates then
    with FAddedItems do
    for i := 0 to Count - 1 do
    begin
      TempItem := PDataRecord(Items[i]);
      Callback(UserData, TempItem^.Row, TBookmark(@TempItem), rsAdded);
    end;
end;


procedure TCustomSqliteDataset.RefetchData;
var
  i: Integer;
begin
  //Close
  if FSaveOnRefetch then
    ApplyUpdates;
  if FDataAllocated then
    DisposeLinkedList;  
  FAddedItems.Clear;
  FUpdatedItems.Clear;
  FDeletedItems.Clear;
  //Reopen
  BuildLinkedList;               
  FCurrentItem := FBeginItem;
  for i := 0 to BufferCount - 1 do
    PPDataRecord(Buffers[i])^ := FBeginItem;
  Resync([]);
  DoAfterScroll;
end;

function TCustomSqliteDataset.TableExists: Boolean;
begin
  Result:=TableExists(FTableName);
end;

function TCustomSqliteDataset.TableExists(const ATableName: String): Boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  ExecSql('SELECT name FROM SQLITE_MASTER WHERE type = ''table'' AND name LIKE ''' + ATableName + ''';');
=======
  ExecSql('SELECT name FROM SQLITE_MASTER WHERE type = ''table'' AND name LIKE '''+ ATableName+ ''';');
>>>>>>> graemeg/fixes_2_2
=======
  ExecSql('SELECT name FROM SQLITE_MASTER WHERE type = ''table'' AND name LIKE '''+ ATableName+ ''';');
>>>>>>> origin/fixes_2_2
  Result := FReturnCode = SQLITE_ROW;
end;

function TCustomSqliteDataset.UpdatesPending: Boolean;
begin
  Result := (FUpdatedItems.Count > 0) or
    (FAddedItems.Count > 0) or (FDeletedItems.Count > 0);
end;

function TCustomSqliteDataset.QuickQuery(const ASql: String): String;
begin
  Result := QuickQuery(ASQL, nil, False);
end;

function TCustomSqliteDataset.QuickQuery(const ASql: String;
  const AStrList: TStrings): String;
begin
  Result := QuickQuery(ASQL, AStrList, False)
end;  


{$ifdef DEBUGACTIVEBUFFER}
procedure TCustomSqliteDataset.SetCurrentItem(Value:PDataRecord);
var
 ANo:Integer;

  function GetItemPos:Integer;
  var
    TempItem:PDataRecord;
  begin
    Result:= -1;
    TempItem:=FBeginItem;
    if Value = FCacheItem then
       Result:=-2
    else
    while Value <> TempItem do
    begin
     if TempItem^.Next <> nil then
     begin
       inc(Result);
       TempItem:=TempItem^.Next;
     end
     else
     begin
      Result:=-1;
      break;
     end;
    end;
  end;

begin
  if Value = FBeginItem then
  begin
    writeln('FCurrentItem set to FBeginItem: ',IntToHex(Integer(Value),0));
    FFCurrentItem:=Value;
  end
  else
    if Value = FEndItem then
    begin
      writeln('FCurrentItem set to FEndItem: ',IntToHex(Integer(Value),0));
      FFCurrentItem:=Value;
    end
    else
      if Value = FCacheItem then
      begin
        writeln('FCurrentItem set to FCacheItem: ',IntToHex(Integer(Value),0));
        FFCurrentItem:=Value;
      end
      else
      begin
        writeln('FCurrentItem set from ',IntToHex(Integer(FFCurrentItem),0),' to ',IntToHex(Integer(Value),0));
        Ano:=GetItemPos;
        writeln('Item position is ',ANo);
        FFCurrentItem:=Value;
      end;
end;
{$endif}

end.
