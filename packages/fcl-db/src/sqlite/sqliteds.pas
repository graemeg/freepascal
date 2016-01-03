unit SqliteDS;

{
  This is TSqliteDataset, a TDataset descendant class for use with fpc compiler
  Copyright (C) 2004  Luiz Américo Pereira Câmara
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

{$mode objfpc}
{$H+}
{.$Define DEBUG_SQLITEDS}

interface

uses
  Classes, SysUtils, CustomSqliteDS;

type
  { TSqliteDataset }

  TSqliteDataset = class(TCustomSqliteDataset)
  private
    function GetSqliteEncoding: String;
  protected
    procedure BuildLinkedList; override;
    function GetLastInsertRowId: Int64; override;
    function GetRowsAffected:Integer; override;
    function InternalGetHandle: Pointer; override;
    procedure InternalCloseHandle; override;
    procedure RetrieveFieldDefs; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function SqliteExec(ASQL: PAnsiChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer; override;
=======
    function SqliteExec(ASQL: PChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer; override;
>>>>>>> graemeg/cpstrnew
=======
    function SqliteExec(ASQL: PChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer; override;
>>>>>>> graemeg/cpstrnew
=======
    function SqliteExec(ASQL: PChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer; override;
>>>>>>> graemeg/cpstrnew
=======
    function SqliteExec(ASQL: PChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer; override;
>>>>>>> origin/cpstrnew
  public
    procedure ExecuteDirect(const ASQL: String); override;
    function QuickQuery(const ASQL: String; const AStrList: TStrings; FillObjects: Boolean): String; override;
    function ReturnString: String; override;
    class function SqliteVersion: String; override;
    property SqliteEncoding: String read GetSqliteEncoding;
  end;

implementation

uses
  sqlite, db, strutils;

//function sqlite_last_statement_changes(dbhandle:Pointer):longint;cdecl;external 'sqlite' name 'sqlite_last_statement_changes';

function GetAutoIncValue(NextValue: Pointer; Columns: Integer; ColumnValues: PPAnsiChar; ColumnNames: PPAnsiChar): Integer; cdecl;
var
  CodeError, TempInt: Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  TempInt := 0;
=======
  TempInt := -1;
>>>>>>> graemeg/fixes_2_2
=======
  TempInt := -1;
>>>>>>> origin/fixes_2_2
  if ColumnValues[0] <> nil then
  begin
    Val(String(ColumnValues[0]), TempInt, CodeError);
    if CodeError <> 0 then
      DatabaseError('TSqliteDataset: Error trying to get last autoinc value');
  end;
  Integer(NextValue^) := Succ(TempInt);
  Result := 1;
end;

{ TSqliteDataset }

function TSqliteDataset.SqliteExec(ASQL: PAnsiChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer;
begin
  Result := sqlite_exec(FSqliteHandle, ASQL, ACallback, Data, nil);
end;

procedure TSqliteDataset.InternalCloseHandle;
begin
  sqlite_close(FSqliteHandle);
  FSqliteHandle := nil;
end;

function TSqliteDataset.InternalGetHandle: Pointer;
var
<<<<<<< HEAD
<<<<<<< HEAD
  ErrorStr: PAnsiChar;
begin
  Result := sqlite_open(PAnsiChar(FFileName), 0, @ErrorStr);
  if Result = nil then
  begin
    DatabaseError('Error opening "' + FFileName + '": ' + String(ErrorStr));
=======
  ErrorStr: PChar;
begin
=======
  ErrorStr: PChar;
begin
>>>>>>> origin/fixes_2_2
  Result := sqlite_open(PChar(FFileName), 0, @ErrorStr);
  if Result = nil then
  begin
    DatabaseError('Error opening "' + FFileName +'": ' + String(ErrorStr));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    sqlite_freemem(ErrorStr);
  end;
end;

procedure TSqliteDataset.RetrieveFieldDefs;
var
<<<<<<< HEAD
  ColumnCount, i, DataSize:Integer;
  AType: TFieldType;
  vm: Pointer;
  ColumnNames, ColumnValues:PPAnsiChar;
  ColumnStr: String;
begin
  FieldDefs.Clear;
  FAutoIncFieldNo := -1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := sqlite_compile(FSqliteHandle, PAnsiChar(FEffectiveSQL), nil, @vm, nil);
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> origin/cpstrnew
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);
  sqlite_step(vm, @ColumnCount, @ColumnValues, @ColumnNames);
=======
  ColumnCount,i:Integer;
  AType:TFieldType;
  vm:Pointer;
  ColumnNames,ColumnValues:PPChar;
  ColumnStr:String;
begin
  FieldDefs.Clear;
  FAutoIncFieldNo := -1;
  FReturnCode := sqlite_compile(FSqliteHandle,PChar(FSql),nil,@vm,nil);
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);
  sqlite_step(vm,@ColumnCount,@ColumnValues,@ColumnNames);
>>>>>>> graemeg/fixes_2_2
  //Prepare the array of pchar2sql functions
  SetLength(FGetSqlStr, ColumnCount);
  // Sqlite is typeless (allows any type in any field)
  // regardless of what is in Create Table, but returns
  // exactly what is in Create Table statement
  // here is a trick to get the datatype.
  // If the field contains another type, may have problems
  for i := 0 to ColumnCount - 1 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    DataSize := 0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    ColumnStr := UpperCase(String(ColumnNames[i + ColumnCount]));
    if (ColumnStr = 'INTEGER') or (ColumnStr = 'INT') then
    begin
      if AutoIncrementKey and
           (UpperCase(String(ColumnNames[i])) = UpperCase(PrimaryKey)) then
      begin
        AType := ftAutoInc;
        FAutoIncFieldNo := i;
      end
      else
        AType := ftInteger;
<<<<<<< HEAD
<<<<<<< HEAD
    end else if Pos('VARCHAR', ColumnStr) = 1 then
    begin
      AType := ftString;
      DataSize := StrToIntDef(Trim(ExtractDelimited(2, ColumnStr, ['(', ')'])), DefaultStringSize);
    end else if Pos('BOOL', ColumnStr) = 1 then
    begin
      AType := ftBoolean;
    end else if Pos('AUTOINC', ColumnStr) = 1 then
=======
=======
>>>>>>> origin/fixes_2_2
    end else if Pos('VARCHAR',ColumnStr) = 1 then
    begin
      AType := ftString;
    end else if Pos('BOOL',ColumnStr) = 1 then
    begin
      AType := ftBoolean;
    end else if Pos('AUTOINC',ColumnStr) = 1 then
>>>>>>> graemeg/fixes_2_2
    begin
      AType := ftAutoInc;
      if FAutoIncFieldNo = -1 then
        FAutoIncFieldNo := i;
<<<<<<< HEAD
<<<<<<< HEAD
    end else if (Pos('FLOAT', ColumnStr)=1) or (Pos('NUMERIC', ColumnStr) = 1) then
=======
=======
>>>>>>> origin/fixes_2_2
    end else if (Pos('FLOAT',ColumnStr)=1) or (Pos('NUMERIC',ColumnStr)=1) then
>>>>>>> graemeg/fixes_2_2
    begin
      AType := ftFloat;
    end else if (ColumnStr = 'DATETIME') then
    begin
      AType := ftDateTime;
    end else if (ColumnStr = 'DATE') then
    begin
      AType := ftDate;
    end else if (ColumnStr = 'TIME') then
    begin
      AType := ftTime;
<<<<<<< HEAD
<<<<<<< HEAD
    end else if (ColumnStr = 'LARGEINT') or (ColumnStr = 'BIGINT') then
=======
=======
>>>>>>> origin/fixes_2_2
    end else if (ColumnStr = 'LARGEINT') then
>>>>>>> graemeg/fixes_2_2
    begin
      AType := ftLargeInt;
    end else if (ColumnStr = 'TEXT') then
    begin
      AType := ftMemo;
    end else if (ColumnStr = 'CURRENCY') then
    begin
      AType := ftCurrency;
    end else if (ColumnStr = 'WORD') then
    begin
      AType := ftWord;
    end else
    begin
      AType := ftString;
<<<<<<< HEAD
<<<<<<< HEAD
    end;
    FieldDefs.Add(FieldDefs.MakeNameUnique(String(ColumnNames[i])), AType, DataSize);
=======
=======
>>>>>>> origin/fixes_2_2
    end;    
    if AType = ftString then
      FieldDefs.Add(String(ColumnNames[i]), AType, dsMaxStringSize)
    else
      FieldDefs.Add(String(ColumnNames[i]), AType);  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    //Set the pchar2sql function
    case AType of
      ftString:
        FGetSqlStr[i] := @Char2SQLStr;
      ftMemo:
        FGetSqlStr[i] := @Memo2SQLStr;
    else
      FGetSqlStr[i] := @Num2SQLStr;
    end;
  end;
  sqlite_finalize(vm, nil);
  {
  if FReturnCode <> SQLITE_ABORT then
     DatabaseError(ReturnString,Self);
  }
end;

function TSqliteDataset.GetRowsAffected: Integer;
begin
  Result := sqlite_changes(FSqliteHandle);
  //Result := sqlite_last_statement_changes(FSqliteHandle);
end;

procedure TSqliteDataset.ExecuteDirect(const ASQL: String);
var
  vm: Pointer;
  ColumnNames, ColumnValues: PPAnsiChar;
  ColCount: Integer;
begin
  FReturnCode := sqlite_compile(FSqliteHandle, PAnsiChar(ASQL), nil, @vm, nil);
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString,Self);

  FReturnCode := sqlite_step(vm, @ColCount, @ColumnValues, @ColumnNames);

  sqlite_finalize(vm, nil);
end;

procedure TSqliteDataset.BuildLinkedList;
var
  TempItem: PDataRecord;
  vm: Pointer;
  ColumnNames, ColumnValues: PPAnsiChar;
  Counter, ColumnCount: Integer;
begin
  //Get AutoInc Field initial value
  if FAutoIncFieldNo <> -1 then
    sqlite_exec(FSqliteHandle, PAnsiChar('Select Max(' + FieldDefs[FAutoIncFieldNo].Name + ') from ' + FTableName),
      @GetAutoIncValue, @FNextAutoInc, nil);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := sqlite_compile(FSqliteHandle, PAnsiChar(FEffectiveSQL), nil, @vm, nil);
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(FEffectiveSQL), nil, @vm, nil);
>>>>>>> origin/cpstrnew
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);

  FDataAllocated := True;

  TempItem := FBeginItem;
  FRecordCount := 0;
  FReturnCode := sqlite_step(vm, @ColumnCount, @ColumnValues, @ColumnNames);
  FRowCount := ColumnCount;
  //add extra rows for calculated fields
  if FCalcFieldList <> nil then
    Inc(FRowCount, FCalcFieldList.Count);
  FRowBufferSize := (SizeOf(PPAnsiChar) * FRowCount);

  while FReturnCode = SQLITE_ROW do
  begin
    Inc(FRecordCount);
    New(TempItem^.Next);
    TempItem^.Next^.Previous := TempItem;
    TempItem := TempItem^.Next;
    GetMem(TempItem^.Row, FRowBufferSize);
    for Counter := 0 to ColumnCount - 1 do
      TempItem^.Row[Counter] := StrNew(ColumnValues[Counter]);
    //initialize calculated fields with nil
    for Counter := ColumnCount to FRowCount - 1 do
      TempItem^.Row[Counter] := nil;
    FReturnCode := sqlite_step(vm, @FRowCount, @ColumnValues, @ColumnNames);
  end;
  sqlite_finalize(vm, nil);

  // Attach EndItem
  TempItem^.Next := FEndItem;
  FEndItem^.Previous := TempItem;

  // Alloc item used in append/insert
  GetMem(FSavedEditItem^.Row, FRowBufferSize);
  for Counter := 0 to FRowCount - 1 do
    FSavedEditItem^.Row[Counter] := nil;
  // Fill FBeginItem.Row with nil -> necessary for avoid exceptions in empty datasets
  GetMem(FBeginItem^.Row, FRowBufferSize);
  for Counter := 0 to FRowCount - 1 do
    FBeginItem^.Row[Counter] := nil;
end;

function TSqliteDataset.GetLastInsertRowId: Int64;
begin
  Result := sqlite_last_insert_rowid(FSqliteHandle);
end;

function TSqliteDataset.ReturnString: String;
begin
  case FReturnCode of
    SQLITE_OK           : Result := 'SQLITE_OK';
    SQLITE_ERROR        : Result := 'SQLITE_ERROR';
    SQLITE_INTERNAL     : Result := 'SQLITE_INTERNAL';
    SQLITE_PERM         : Result := 'SQLITE_PERM';
    SQLITE_ABORT        : Result := 'SQLITE_ABORT';
    SQLITE_BUSY         : Result := 'SQLITE_BUSY';
    SQLITE_LOCKED       : Result := 'SQLITE_LOCKED';
    SQLITE_NOMEM        : Result := 'SQLITE_NOMEM';
    SQLITE_READONLY     : Result := 'SQLITE_READONLY';
    SQLITE_INTERRUPT    : Result := 'SQLITE_INTERRUPT';
    SQLITE_IOERR        : Result := 'SQLITE_IOERR';
    SQLITE_CORRUPT      : Result := 'SQLITE_CORRUPT';
    SQLITE_NOTFOUND     : Result := 'SQLITE_NOTFOUND';
    SQLITE_FULL         : Result := 'SQLITE_FULL';
    SQLITE_CANTOPEN     : Result := 'SQLITE_CANTOPEN';
    SQLITE_PROTOCOL     : Result := 'SQLITE_PROTOCOL';
    SQLITE_EMPTY        : Result := 'SQLITE_EMPTY';
    SQLITE_SCHEMA       : Result := 'SQLITE_SCHEMA';
    SQLITE_TOOBIG       : Result := 'SQLITE_TOOBIG';
    SQLITE_CONSTRAINT   : Result := 'SQLITE_CONSTRAINT';
    SQLITE_MISMATCH     : Result := 'SQLITE_MISMATCH';
    SQLITE_MISUSE       : Result := 'SQLITE_MISUSE';
    SQLITE_NOLFS        : Result := 'SQLITE_NOLFS';
    SQLITE_AUTH         : Result := 'SQLITE_AUTH';
    SQLITE_FORMAT       : Result := 'SQLITE_FORMAT';
    SQLITE_RANGE        : Result := 'SQLITE_RANGE';
    SQLITE_ROW          :
      begin
        Result := 'SQLITE_ROW - not an error';
        Exit;
      end;
    SQLITE_DONE         :
      begin
        Result := 'SQLITE_DONE - not an error';
        Exit;
      end;
  else
    Result := 'Unknow Return Value';
  end;
  Result := Result + ' - ' + sqlite_error_string(FReturnCode);
end;

function TSqliteDataset.GetSqliteEncoding: String;
begin
  Result := String(sqlite_encoding);
end;
  
class function TSqliteDataset.SqliteVersion: String;
begin
  Result := String(sqlite_version);
end;

function TSqliteDataset.QuickQuery(const ASQL: String; const AStrList: TStrings; FillObjects: Boolean): String;
var
  vm: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
  ColumnNames, ColumnValues: PPAnsiChar;
=======
  ColumnNames, ColumnValues: PPChar;
>>>>>>> graemeg/fixes_2_2
=======
  ColumnNames, ColumnValues: PPChar;
>>>>>>> origin/fixes_2_2
  ColCount: Integer;
  
  procedure FillStrings;
  begin
    while FReturnCode = SQLITE_ROW do
    begin
      AStrList.Add(String(ColumnValues[0]));
      FReturnCode := sqlite_step(vm, @ColCount, @ColumnValues, @ColumnNames);
    end;
  end;
  procedure FillStringsAndObjects;
  begin
    while FReturnCode = SQLITE_ROW do
    begin
      // I know, this code is really dirty!!
<<<<<<< HEAD
<<<<<<< HEAD
      AStrList.AddObject(String(ColumnValues[0]),
        TObject(PtrInt(StrToInt(String(ColumnValues[1])))));
      FReturnCode := sqlite_step(vm, @ColCount, @ColumnValues, @ColumnNames);
=======
      AStrList.AddObject(String(ColumnValues[0]), TObject(PtrInt(StrToInt(String(ColumnValues[1])))));
      FReturnCode:=sqlite_step(vm, @ColCount, @ColumnValues, @ColumnNames);
>>>>>>> graemeg/fixes_2_2
=======
      AStrList.AddObject(String(ColumnValues[0]), TObject(PtrInt(StrToInt(String(ColumnValues[1])))));
      FReturnCode:=sqlite_step(vm, @ColCount, @ColumnValues, @ColumnNames);
>>>>>>> origin/fixes_2_2
    end;
  end;    
begin
  if FSqliteHandle = nil then
    GetSqliteHandle;
  Result := '';
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := sqlite_compile(FSqliteHandle, PAnsiChar(ASQL), nil, @vm, nil);
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(ASql), nil, @vm, nil);
>>>>>>> graemeg/fixes_2_2
=======
  FReturnCode := sqlite_compile(FSqliteHandle, PChar(ASql), nil, @vm, nil);
>>>>>>> origin/fixes_2_2
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString,Self);
    
  FReturnCode := sqlite_step(vm, @ColCount, @ColumnValues, @ColumnNames);
  if (FReturnCode = SQLITE_ROW) and (ColCount > 0) then
  begin
    Result := String(ColumnValues[0]);
    if AStrList <> nil then
    begin   
      if FillObjects and (ColCount > 1) then
        FillStringsAndObjects
      else
        FillStrings;
    end;          
  end;  
  sqlite_finalize(vm, nil); 
end;
    
end.

