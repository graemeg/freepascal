{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Michael Van Canneyt, member of the
    Free Pascal development team

    SQLDB Data Dictionary Engine common Implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpddsqldb;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, sqldb, fpdatadict;

Type

  { TSQLDBDDEngine }

  TSQLDBDDEngine = Class(TFPDDEngine)
  Private
    FConn: TSQLConnection;
  Protected
    Function CreateConnection(AConnectString  : String) : TSQLConnection; virtual; abstract;
    Function CreateSQLQuery(ADatasetOwner: TComponent) : TSQLQuery;
    Property Connection : TSQLConnection Read FConn;
  Public
    Procedure Disconnect ; override;
    Function HostSupported: Boolean; virtual;
    Function Connect(const AConnectString : String) : Boolean; override;
    Function GetTableList(List : TStrings) : Integer; override;
    Function ImportFields(Table : TDDTableDef) : Integer; override;
    Function ImportIndexes(Table : TDDTableDef) : Integer; override;
    Function ViewTable(Const TableName: String; DatasetOwner : TComponent) : TDataset; override;
    Function RunQuery(SQL : String) : Integer; override;
    Function CreateQuery(SQL : String; DatasetOwner : TComponent) : TDataset; override;
    Procedure SetQueryStatement(SQL : String; AQuery : TDataset); override;
    Function GetTableIndexDefs(ATableName : String; Defs : TDDIndexDefs) : integer ; override;
    Class function EngineCapabilities : TFPDDEngineCapabilities; override;
  end;



Const
  // used in connectionstring
  KeyHostName     = 'Host';
  KeyDatabaseName = 'Database';
  KeyUserName     = 'User';
  KeyPassword     = 'Password';
  KeyEncode       = 'Trivial';
<<<<<<< HEAD
<<<<<<< HEAD
  KeyCharset      = 'Charset';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation


uses strutils;

Resourcestring
  SErrQueryNotSQLQuery = 'Query object "%s" is not a SQL Query';
  
{ TSQLDBDDEngine }

function TSQLDBDDEngine.HostSupported: Boolean;
begin
  Result:=True;
end;

function TSQLDBDDEngine.CreateSQLQuery(ADatasetOwner: TComponent): TSQLQuery;
begin
  Result:=TSQLQuery.Create(ADatasetOwner);
  Result.DataBase:=FConn;
  Result.Transaction:=FConn.TRansaction;
end;

procedure TSQLDBDDEngine.Disconnect;
begin
  FreeAndNil(FConn);
  FConnectString:='';
  FConnected:=False;
end;

function TSQLDBDDEngine.Connect(const AConnectString: String): Boolean;

Var
  L : TStringList;
  
begin
  FConn:=CreateConnection(AConnectString);
  FConn.Transaction:=TSQLTransaction.Create(FConn);
  L:=TStringList.Create;
  Try
    L.CommaText:=AConnectString;
    If HostSupported then
      FConn.HostName:=L.Values[KeyHostName];
    FConn.DatabaseName:=L.Values[KeyDatabaseName];
    FConn.UserName:=L.Values[KeyUserName];
    FConn.Password:=XorDecode(KeyEncode,L.Values[KeyPassword]);
    FConn.LoginPrompt:=False;
    FConn.Connected:=True;
<<<<<<< HEAD
<<<<<<< HEAD
    FConn.CharSet:=L.Values[KeyCharset];
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FConnected:=True;
    FConnectString:=AConnectString;
    Result:=True;
  Finally
    L.Free;
  end;
end;

function TSQLDBDDEngine.GetTableList(List: TStrings): Integer;
begin
  FConn.GetTableNames(List,False);
  result := list.count;
end;

function TSQLDBDDEngine.ImportFields(Table: TDDTableDef): Integer;

Const
  SQL = 'SELECT * from %s where (1=0)';

Var
  Q : TSQLQuery;

begin
  Q:=CreateSQLQuery(Nil);
  try
    Q.Sql.Text:=Format(SQL,[Table.TableName]);
    Q.Open;
    try
      Result:=Table.ImportFromDataset(Q);
    finally
      Q.CLose;
    end;
  finally
    Q.Free;
  end;
end;


Function TSQLDBDDEngine.ImportIndexes(Table : TDDTableDef) : Integer;
begin
end;


function TSQLDBDDEngine.ViewTable(const TableName: String;
  DatasetOwner: TComponent): TDataset;
  
Var
  Q : TSQLQuery;
  
begin
  Q:=CreateSQLQuery(DatasetOwner);
  Q.SQL.Text:='SELECT * FROM '+TableName;
  Result:=Q;
end;

function TSQLDBDDEngine.RunQuery(SQL: String): Integer;

Var
  Q : TSQLQuery;

begin
  Q:=CreateSQLQuery(Nil);
  Try
    Q.SQL.Text:=SQL;
    Q.ExecSQL;
    Result:=0;
  Finally
    Q.Free;
  end;
end;

function TSQLDBDDEngine.CreateQuery(SQL: String; DatasetOwner: TComponent
  ): TDataset;

Var
  Q : TSQLQuery;

begin
  Q:=CreateSQLQuery(Nil);
  Result:=Q;
  Q.SQL.Text:=SQL;
  Q.Open;
end;

procedure TSQLDBDDEngine.SetQueryStatement(SQL: String; AQuery: TDataset);
begin
  If Not (AQuery is TSQLQuery) then
    Raise EDataDict.CreateFmt(SErrQueryNotSQLQuery,[AQuery.ClassName]);
  (AQuery as TSQLQuery).SQL.Text:=SQL;
end;

function TSQLDBDDEngine.GetTableIndexDefs(ATableName: String; Defs: TDDIndexDefs
  ): integer;
  
Var
  Q : TSQLQuery;
  
begin
  Q:=TSQLQuery.Create(Self);
  Try
    Q.Database:=FConn;
    Q.Transaction:=FConn.Transaction;
    Q.SQL.text:=Format('SELECT * FROM %s WHERE (1=2)',[ATAbleName]);
    Q.ReadOnly:=False;
    Q.Prepare;
    Q.IndexDefs.Update;
    IndexDefsToDDIndexDefs(Q.IndexDefs,Defs);
    Result:=Defs.Count;
  finally
    Q.Free;
  end;
end;

class function TSQLDBDDEngine.EngineCapabilities: TFPDDEngineCapabilities;
begin
  Result:=[ecimport,ecViewTable, ecRunQuery, ecTableIndexes];
end;

end.

