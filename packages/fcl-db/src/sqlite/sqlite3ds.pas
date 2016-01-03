unit Sqlite3DS;

{
  This is TSqlite3Dataset, a TDataset descendant class for use with fpc compiler
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
  { TSqlite3Dataset }

  TSqlite3Dataset = class(TCustomSqliteDataset)
  protected
    procedure BuildLinkedList; override;
    function GetLastInsertRowId: Int64; override;
    function GetRowsAffected:Integer; override;
    procedure InternalCloseHandle; override;
    function InternalGetHandle: Pointer; override;
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
  end;

implementation

uses
<<<<<<< HEAD
  sqlite3, db, strutils;
=======
  sqlite3,db;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  
function SqliteCode2Str(Code: Integer): String;
begin
  case Code of
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
    SQLITE_ROW          : Result := 'SQLITE_ROW';
    SQLITE_NOTADB       : Result := 'SQLITE_NOTADB';
    SQLITE_DONE         : Result := 'SQLITE_DONE';
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Result := 'Unknown Return Value';
=======
    Result:='Unknown Return Value';
>>>>>>> graemeg/fixes_2_2
=======
    Result:='Unknown Return Value';
>>>>>>> origin/fixes_2_2
  end;
end;

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
      DatabaseError('TSqlite3Dataset: Error trying to get last autoinc value');
  end;
  Integer(NextValue^) := Succ(TempInt);
  Result := 1;
end;

{ TSqlite3Dataset }

function TSqlite3Dataset.SqliteExec(ASQL: PAnsiChar; ACallback: TSqliteCdeclCallback; Data: Pointer): Integer;
begin
  Result := sqlite3_exec(FSqliteHandle, ASQL, ACallback, Data, nil);
end;

procedure TSqlite3Dataset.InternalCloseHandle;
begin
  sqlite3_close(FSqliteHandle);
  FSqliteHandle := nil;
  //todo:handle return data
end;


function TSqlite3Dataset.InternalGetHandle: Pointer;
const
  CheckFileSql = 'Select Name from sqlite_master LIMIT 1';
var
  vm: Pointer;
  ErrorStr: String;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  sqlite3_open(PAnsiChar(FFileName), @Result);
  //sqlite3_open returns SQLITE_OK even for invalid files
  //do additional check here
  FReturnCode := sqlite3_prepare_v2(Result, CheckFileSql, -1, @vm, nil);
  if FReturnCode <> SQLITE_OK then
  begin
    ErrorStr := SqliteCode2Str(FReturnCode) + ' - ' + sqlite3_errmsg(Result);
=======
=======
>>>>>>> origin/fixes_2_2
  sqlite3_open(PChar(FFileName), @Result);
  //sqlite3_open returns SQLITE_OK even for invalid files
  //do additional check here
  FReturnCode := sqlite3_prepare(Result, CheckFileSql, -1, @vm, nil);
  if FReturnCode <> SQLITE_OK then
  begin
    ErrorStr := SqliteCode2Str(FReturnCode) + ' - ' + sqlite3_errmsg(Result);;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    sqlite3_close(Result);
    DatabaseError(ErrorStr, Self);
  end;
  sqlite3_finalize(vm);
end;

<<<<<<< HEAD
procedure TSqlite3Dataset.RetrieveFieldDefs;
var
  vm: Pointer;
  ColumnStr: String;
  i, ColumnCount, DataSize: Integer;
=======
procedure TSqlite3Dataset.InternalInitFieldDefs;
const
  FieldSizeMap: array[Boolean] of Integer = (0, dsMaxStringSize);
var
  vm: Pointer;
  ColumnStr: String;
  i, ColumnCount: Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  AType: TFieldType;
begin
  {$ifdef DEBUG_SQLITEDS}
  WriteLn('##TSqlite3Dataset.InternalInitFieldDefs##');
  {$endif}
  FAutoIncFieldNo := -1;
  FieldDefs.Clear;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := sqlite3_prepare_v2(FSqliteHandle, PAnsiChar(FEffectiveSQL), -1, @vm, nil);
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> origin/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FSql), -1, @vm, nil);
>>>>>>> graemeg/fixes_2_2
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FSql), -1, @vm, nil);
>>>>>>> origin/fixes_2_2
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);
  sqlite3_step(vm);
  ColumnCount := sqlite3_column_count(vm);
<<<<<<< HEAD
<<<<<<< HEAD
=======
  //Set BufferSize
  FRowBufferSize := (SizeOf(PPChar) * ColumnCount);
>>>>>>> graemeg/fixes_2_2
=======
  //Set BufferSize
  FRowBufferSize := (SizeOf(PPChar) * ColumnCount);
>>>>>>> origin/fixes_2_2
  //Prepare the array of pchar2sql functions
  SetLength(FGetSqlStr, ColumnCount);
  for i := 0 to ColumnCount - 1 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    DataSize := 0;
    ColumnStr := UpperCase(String(sqlite3_column_decltype(vm, i)));
    if (ColumnStr = 'INTEGER') or (ColumnStr = 'INT') then
    begin
      if AutoIncrementKey and (UpperCase(String(sqlite3_column_name(vm, i))) = UpperCase(PrimaryKey)) then
      begin
        AType := ftAutoInc;
        FAutoIncFieldNo := i;
      end
      else
        AType := ftInteger;
    end else if Pos('VARCHAR', ColumnStr) = 1 then
    begin
      AType := ftString;
      DataSize := StrToIntDef(Trim(ExtractDelimited(2, ColumnStr, ['(', ')'])), DefaultStringSize);
    end else if Pos('BOOL', ColumnStr) = 1 then
    begin
      AType := ftBoolean;
    end else if Pos('AUTOINC', ColumnStr) = 1 then
    begin
      AType := ftAutoInc;
      if FAutoIncFieldNo = -1 then
        FAutoIncFieldNo := i;
    end else if (Pos('FLOAT', ColumnStr) = 1) or (Pos('NUMERIC', ColumnStr) = 1) then
    begin
      AType := ftFloat;
    end else if (ColumnStr = 'DATETIME') then
    begin
      AType := ftDateTime;
    end else if (ColumnStr = 'DATE') then
    begin
      AType := ftDate;
    end else if (ColumnStr = 'LARGEINT') or (ColumnStr = 'BIGINT') then
    begin
      AType := ftLargeInt;
    end else if (ColumnStr = 'TIME') then
    begin
      AType := ftTime;
    end else if (ColumnStr = 'TEXT') then
    begin
      AType := ftMemo;
    end else if (ColumnStr = 'CURRENCY') then
    begin
      AType := ftCurrency;
    end else if (ColumnStr = 'WORD') then
    begin
      AType := ftWord;
    end else if (ColumnStr = '') then
    begin
      case sqlite3_column_type(vm, i) of
        SQLITE_INTEGER:
          AType := ftInteger;
        SQLITE_FLOAT:
          AType := ftFloat;
      else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        begin
          AType := ftString;
          DataSize := DefaultStringSize;
        end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	    begin
          AType := ftString;
		  DataSize := DefaultStringSize;
		end;  		
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
      end;
    end else
    begin
      AType := ftString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      DataSize := DefaultStringSize;
=======
	  DataSize := DefaultStringSize;
>>>>>>> graemeg/cpstrnew
=======
	  DataSize := DefaultStringSize;
>>>>>>> graemeg/cpstrnew
=======
	  DataSize := DefaultStringSize;
>>>>>>> graemeg/cpstrnew
=======
	  DataSize := DefaultStringSize;
>>>>>>> origin/cpstrnew
    end;
    FieldDefs.Add(FieldDefs.MakeNameUnique(String(sqlite3_column_name(vm, i))), AType, DataSize);
    //Set the pchar2sql function
    case AType of
      ftString:
        FGetSqlStr[i] := @Char2SQLStr;
      ftMemo:
        FGetSqlStr[i] := @Memo2SQLStr;
    else
      FGetSqlStr[i] := @Num2SQLStr;
    end;
    {$ifdef DEBUG_SQLITEDS}
    WriteLn('  Field[', i, '] Name: ', sqlite3_column_name(vm, i));
    WriteLn('  Field[', i, '] Type: ', sqlite3_column_decltype(vm, i));
    {$endif}
  end;
  sqlite3_finalize(vm);
  {$ifdef DEBUG_SQLITEDS}
  WriteLn('  FieldDefs.Count: ', FieldDefs.Count);
=======
=======
>>>>>>> origin/fixes_2_2
   ColumnStr := UpperCase(String(sqlite3_column_decltype(vm, i)));
   if (ColumnStr = 'INTEGER') or (ColumnStr = 'INT') then
   begin
     if AutoIncrementKey and (UpperCase(String(sqlite3_column_name(vm, i))) = UpperCase(PrimaryKey)) then
     begin
       AType := ftAutoInc;
       FAutoIncFieldNo := i;
     end
     else
       AType := ftInteger;     
   end else if Pos('VARCHAR', ColumnStr) = 1 then
   begin
     AType := ftString;
   end else if Pos('BOOL', ColumnStr) = 1 then
   begin
     AType := ftBoolean;
   end else if Pos('AUTOINC', ColumnStr) = 1 then
   begin
     AType := ftAutoInc;
     if FAutoIncFieldNo = -1 then
       FAutoIncFieldNo := i;
   end else if (Pos('FLOAT', ColumnStr) = 1) or (Pos('NUMERIC', ColumnStr) = 1) then
   begin
     AType := ftFloat;
   end else if (ColumnStr = 'DATETIME') then
   begin
     AType := ftDateTime;
   end else if (ColumnStr = 'DATE') then
   begin
     AType := ftDate;
   end else if (ColumnStr = 'LARGEINT') then
   begin
     AType := ftLargeInt;
   end else if (ColumnStr = 'TIME') then
   begin
     AType := ftTime;
   end else if (ColumnStr = 'TEXT') then
   begin
     AType := ftMemo;
   end else if (ColumnStr = 'CURRENCY') then
   begin
     AType := ftCurrency;
   end else if (ColumnStr = 'WORD') then
   begin
     AType := ftWord;
   end else if (ColumnStr = '') then
   begin
     case sqlite3_column_type(vm, i) of
       SQLITE_INTEGER:
         AType := ftInteger;
       SQLITE_FLOAT:
         AType := ftFloat;
     else
       AType := ftString;
     end;
   end else
   begin
     AType := ftString;
   end;
   FieldDefs.Add(String(sqlite3_column_name(vm, i)), AType, FieldSizeMap[AType = ftString]);
   //Set the pchar2sql function
   if AType in [ftString, ftMemo] then
     FGetSqlStr[i] := @Char2SqlStr
   else
     FGetSqlStr[i] := @Num2SqlStr;
   {$ifdef DEBUG}
   writeln('  Field[',i,'] Name: ', sqlite3_column_name(vm,i));
   writeln('  Field[',i,'] Type: ', sqlite3_column_decltype(vm,i));
   {$endif}
  end;
  sqlite3_finalize(vm);
  {$ifdef DEBUG}
  writeln('  FieldDefs.Count: ', FieldDefs.Count);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$endif}
end;

function TSqlite3Dataset.GetRowsAffected: Integer;
begin
  Result := sqlite3_changes(FSqliteHandle);
end;

procedure TSqlite3Dataset.ExecuteDirect(const ASQL: String);
var
  vm: Pointer;
begin
  FReturnCode := sqlite3_prepare_v2(FSqliteHandle, PAnsiChar(ASQL), -1, @vm, nil);
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);
  FReturnCode := sqlite3_step(vm);
  sqlite3_finalize(vm);
end;

procedure TSqlite3Dataset.BuildLinkedList;
var
  TempItem: PDataRecord;
  vm: Pointer;
  Counter, ColumnCount: Integer;
begin
  //Get AutoInc Field initial value
  if FAutoIncFieldNo <> -1 then
    sqlite3_exec(FSqliteHandle, PAnsiChar('Select Max(' + FieldDefs[FAutoIncFieldNo].Name +
      ') from ' + FTableName), @GetAutoIncValue, @FNextAutoInc, nil);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := sqlite3_prepare_v2(FSqliteHandle, PAnsiChar(FEffectiveSQL), -1, @vm, nil);
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> graemeg/cpstrnew
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle, PChar(FEffectiveSQL), -1, @vm, nil);
>>>>>>> origin/cpstrnew
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);

  FDataAllocated := True;

  TempItem := FBeginItem;
  FRecordCount := 0;
  ColumnCount := sqlite3_column_count(vm);
  FRowCount := ColumnCount;
  //add extra rows for calculated fields
  if FCalcFieldList <> nil then
    Inc(FRowCount, FCalcFieldList.Count);
  FRowBufferSize := (SizeOf(PPAnsiChar) * FRowCount);
  FReturnCode := sqlite3_step(vm);
  while FReturnCode = SQLITE_ROW do
  begin
    Inc(FRecordCount);
    New(TempItem^.Next);
    TempItem^.Next^.Previous := TempItem;
    TempItem := TempItem^.Next;
    GetMem(TempItem^.Row, FRowBufferSize);
    for Counter := 0 to ColumnCount - 1 do
      TempItem^.Row[Counter] := StrBufNew(sqlite3_column_text(vm, Counter), sqlite3_column_bytes(vm, Counter) + 1);
    //initialize calculated fields with nil
    for Counter := ColumnCount to FRowCount - 1 do
      TempItem^.Row[Counter] := nil;
    FReturnCode := sqlite3_step(vm);
  end;
  sqlite3_finalize(vm);

  // Attach EndItem
  TempItem^.Next := FEndItem;
  FEndItem^.Previous := TempItem;

  // Alloc temporary item used in edit
  GetMem(FSavedEditItem^.Row, FRowBufferSize);
  for Counter := 0 to FRowCount - 1 do
    FSavedEditItem^.Row[Counter] := nil;
  // Fill FBeginItem.Row with nil -> necessary for avoid exceptions in empty datasets
  GetMem(FBeginItem^.Row, FRowBufferSize);
  //Todo: see if is better to nullif using FillDWord
  for Counter := 0 to FRowCount - 1 do
    FBeginItem^.Row[Counter] := nil;
end;

function TSqlite3Dataset.GetLastInsertRowId: Int64;
begin
  Result := sqlite3_last_insert_rowid(FSqliteHandle);
end;

function TSqlite3Dataset.ReturnString: String;
begin
  Result := SqliteCode2Str(FReturnCode) + ' - ' + sqlite3_errmsg(FSqliteHandle);
end;

class function TSqlite3Dataset.SqliteVersion: String;
begin
  Result := String(sqlite3_version());
end;

function TSqlite3Dataset.QuickQuery(const ASQL: String; const AStrList: TStrings; FillObjects:Boolean): String;
var
  vm: Pointer;
    
  procedure FillStrings;
  begin
    while FReturnCode = SQLITE_ROW do
    begin
      AStrList.Add(String(sqlite3_column_text(vm,0)));
      FReturnCode := sqlite3_step(vm);
    end;
  end;
  procedure FillStringsAndObjects;
  begin
    while FReturnCode = SQLITE_ROW do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      AStrList.AddObject(String(sqlite3_column_text(vm, 0)),
        TObject(PtrInt(sqlite3_column_int(vm, 1))));
=======
      AStrList.AddObject(String(sqlite3_column_text(vm,0)), TObject(PtrInt(sqlite3_column_int(vm,1))));
>>>>>>> graemeg/fixes_2_2
=======
      AStrList.AddObject(String(sqlite3_column_text(vm,0)), TObject(PtrInt(sqlite3_column_int(vm,1))));
>>>>>>> origin/fixes_2_2
      FReturnCode := sqlite3_step(vm);
    end;
  end;    
begin
  if FSqliteHandle = nil then
    GetSqliteHandle;
  Result := '';
<<<<<<< HEAD
<<<<<<< HEAD
  FReturnCode := sqlite3_prepare_v2(FSqliteHandle,PAnsiChar(ASQL), -1, @vm, nil);
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle,Pchar(ASql), -1, @vm, nil);
>>>>>>> graemeg/fixes_2_2
=======
  FReturnCode := sqlite3_prepare(FSqliteHandle,Pchar(ASql), -1, @vm, nil);
>>>>>>> origin/fixes_2_2
  if FReturnCode <> SQLITE_OK then
    DatabaseError(ReturnString, Self);
    
  FReturnCode := sqlite3_step(vm);
  if (FReturnCode = SQLITE_ROW) and (sqlite3_column_count(vm) > 0) then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    Result := String(sqlite3_column_text(vm, 0));
=======
    Result := String(sqlite3_column_text(vm,0));
>>>>>>> graemeg/fixes_2_2
=======
    Result := String(sqlite3_column_text(vm,0));
>>>>>>> origin/fixes_2_2
    if AStrList <> nil then
    begin   
      if FillObjects and (sqlite3_column_count(vm) > 1) then
        FillStringsAndObjects
      else
        FillStrings;
    end;          
  end;  
  sqlite3_finalize(vm); 
end;

end.

